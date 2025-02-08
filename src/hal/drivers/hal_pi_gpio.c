/********************************************************************
* Description:  hal_gpio.c
*               Driver for the Raspberry Pi GPIO pins
*
* Author: Michael Haberler
* License: GPL Version 2
* Copyright (c) 2012.
*
* some code  taken from the bcm2835 library by::
*
* Author: Mike McCauley (mikem@open.com.au)
* Copyright (C) 2011 Mike McCauley
* see http://www.open.com.au/mikem/bcm2835/
* Copyright (c) 2012 Ben Croston - cpuinfo.*
*
* made work for Raspberry2 9/2015 Michael Haberler
* Last change: Modify for Pi5 10/2019 andypugh
* Last change: Modify for Pi400 3/2022 elovalvo
s********************************************************************/


#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_bitops.h"
#include "rtapi_app.h"		/* RTAPI realtime module decls */
                                /* this also includes config.h */
#include "hal.h"		/* HAL public API decls */
#include "bcm2835.h"
#include "cpuinfo.h"

#define BCM2708_PERI_BASE   0x20000000
#define BCM2708_GPIO_BASE   (BCM2708_PERI_BASE + 0x200000)
#define BCM2709_PERI_BASE   0x3F000000
#define BCM2709_GPIO_BASE   (BCM2709_PERI_BASE + 0x200000)

#define RTAPI_BIT(nr)           (1UL << (nr))

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

// http://elinux.org/index.php?title=RPi_Low-level_peripherals&printable=yes
// Rev 1 Raspberry:
static unsigned char rev1_gpios[] = {0, 1, 4, 7,   8,  9, 10, 11, 14, 15, 17, 18, 21, 22, 23, 24, 25};
static unsigned char rev1_pins[] =  {3, 5, 7, 26, 24, 21, 19, 23,  8, 10, 11, 12, 13, 15, 16, 18, 22};

// Rev2 Raspberry:
static unsigned char rev2_gpios[] = {2, 3, 4,  7,  8,  9, 10, 11, 14, 15, 17, 18, 22, 23, 24, 25, 27};
static unsigned char rev2_pins[] =  {3, 5, 7, 26, 24, 21, 19, 23, 8,  10, 11, 12, 15, 16, 18, 22, 13};

// Raspberry2/3:
static unsigned char rpi2_gpios[] = {2, 3, 4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 };
static unsigned char rpi2_pins[] =  {3, 5, 7, 29, 31, 26, 24, 21, 19, 23, 32, 33,  8, 10, 36, 11, 12, 35, 38, 40, 15, 16, 18, 22, 37, 13 };

static int npins;
static int  mem_fd;
// I/O access
static volatile unsigned *gpio;

MODULE_AUTHOR("Michael Haberler");
MODULE_DESCRIPTION("Driver for Raspberry Pi GPIO pins");
MODULE_LICENSE("GPL");

// port direction bits, 1=output
static char *dir = "-1"; // all output
RTAPI_MP_STRING(dir, "port direction, 1=output");
static unsigned dir_map;

// exclude pins from usage
static char *exclude = "0"; // all used
RTAPI_MP_STRING(exclude, "exclude pins, 1=dont use");
static unsigned exclude_map;

static int comp_id;		/* component ID */
static unsigned char *pins, *gpios;
hal_bit_t **port_data;

static void write_port(void *arg, long period);
static void read_port(void *arg, long period);

static __inline__ uint32_t bcm2835_peri_read(volatile uint32_t* paddr)
{
  // Make sure we dont return the _last_ read which might get lost
  // if subsequent code changes to a different peripheral
  volatile uint32_t ret = *paddr;
  return ret;
}

// Read input pin
static __inline__ uint8_t bcm2835_gpio_lev(uint8_t pin)
{
  volatile uint32_t* paddr = gpio + BCM2835_GPLEV0/4 + pin/32;
  uint8_t shift = pin % 32;
  uint32_t value = bcm2835_peri_read(paddr);
  return (value & (1 << shift)) ? HIGH : LOW;
}

static __inline__ void bcm2835_peri_write(volatile uint32_t* paddr, uint32_t value)
{
  // Make sure we don't rely on the first write, which may get
  // lost if the previous access was to a different peripheral.
  *paddr = value;
  *paddr = value;
}

// Set output pin
static __inline__ void bcm2835_gpio_set(uint8_t pin)
{
  volatile uint32_t* paddr = gpio + BCM2835_GPSET0/4 + pin/32;
  uint8_t shift = pin % 32;
  bcm2835_peri_write(paddr, 1 << shift);
}

// Clear output pin
static __inline__ void bcm2835_gpio_clr(uint8_t pin)
{
  volatile uint32_t* paddr = gpio + BCM2835_GPCLR0/4 + pin/32;
  uint8_t shift = pin % 32;
  bcm2835_peri_write(paddr, 1 << shift);
}

// Set/clear only the bits in value covered by the mask
static __inline__ void bcm2835_peri_set_bits(volatile uint32_t* paddr, uint32_t value, uint32_t mask)
{
  uint32_t v = bcm2835_peri_read(paddr);
  v = (v & ~mask) | (value & mask);
  bcm2835_peri_write(paddr, v);
}

// Function select
// pin is a BCM2835 GPIO pin number NOT RPi pin number
//      There are 6 control registers, each control the functions of a block
//      of 10 pins.
//      Each control register has 10 sets of 3 bits per GPIO pin:
//
//      000 = GPIO Pin X is an input
//      001 = GPIO Pin X is an output
//      100 = GPIO Pin X takes alternate function 0
//      101 = GPIO Pin X takes alternate function 1
//      110 = GPIO Pin X takes alternate function 2
//      111 = GPIO Pin X takes alternate function 3
//      011 = GPIO Pin X takes alternate function 4
//      010 = GPIO Pin X takes alternate function 5
//
// So the 3 bits for port X are:
//      X / 10 + ((X % 10) * 3)
void bcm2835_gpio_fsel(uint8_t pin, uint8_t mode)
{
  // Function selects are 10 pins per 32 bit word, 3 bits per pin
  volatile uint32_t* paddr = gpio + BCM2835_GPFSEL0/4 + (pin/10);
  uint8_t   shift = (pin % 10) * 3;
  uint32_t  mask = BCM2835_GPIO_FSEL_MASK << shift;
  uint32_t  value = mode << shift;
  bcm2835_peri_set_bits(paddr, value, mask);
}

static int setup_gpiomem_access(void)
{
  if ((mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC)) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,"HAL_PI_GPIO: can't open /dev/gpiomem:  %d - %s\n"
        "If the error is 'permission denied' then try adding the user who runs\n"
        "LinuxCNC to the gpio group: sudo gpasswd -a username gpio\n", errno, strerror(errno));
    return -1;
  }

  gpio = mmap(NULL, BCM2835_BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0);

  if (gpio == MAP_FAILED) {
    close(mem_fd);
    mem_fd = -1;
    rtapi_print_msg(RTAPI_MSG_ERR, "HAL_PI_GPIO: mmap failed: %d - %s\n", errno, strerror(errno));
    return -1;
  }

  return 0;
}

static int  setup_gpio_access(int rev, int ncores)
{
  // open /dev/mem
  if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,"HAL_PI_GPIO: can't open /dev/mem:  %d - %s\n",
		      errno, strerror(errno));
    return -1;
  }

  if (rev <= 2  || ncores <= 2)
       gpio = mmap(NULL, BCM2835_BLOCK_SIZE, PROT_READ|PROT_WRITE,
		   MAP_SHARED, mem_fd, BCM2708_GPIO_BASE);
    else
       gpio = mmap(NULL, BCM2835_BLOCK_SIZE, PROT_READ|PROT_WRITE,
		   MAP_SHARED, mem_fd, BCM2709_GPIO_BASE);

  if (gpio == MAP_FAILED) {
    rtapi_print_msg(RTAPI_MSG_ERR,
		    "HAL_PI_GPIO: mmap failed: %d - %s\n",
		    errno, strerror(errno));
    return -1;;
  }
  return 0;
}

static int number_of_cores(void)
{
    char str[256];
    int procCount = 0;
    FILE *fp;

    if( (fp = fopen("/proc/cpuinfo", "r")) ) {
	while(fgets(str, sizeof str, fp))
	    if( !memcmp(str, "processor", 9) ) procCount++;
    }
    if ( !procCount ) {
	rtapi_print_msg(RTAPI_MSG_ERR,"HAL_PI_GPIO: Unable to get proc count. Defaulting to 2");
	procCount = 2;
    }
    return procCount;
}

int rtapi_app_main(void)
{
    int n, retval = 0;
    int rev, ncores, pinno;
    char *endptr;

    if ((rev = get_rpi_revision()) < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
		      "unrecognized Raspberry revision, see /proc/cpuinfo\n");
      return -EINVAL;
    }
    ncores = number_of_cores();
    rtapi_print_msg(RTAPI_MSG_INFO, "%d cores rev %d", ncores, rev);

    switch (rev) {
     case 1:
      pins = rev1_pins;
      gpios = rev1_gpios;
      npins = sizeof(rev1_pins);
      break;
     case 2:
      pins = rev2_pins;
      gpios = rev2_gpios;
      npins = sizeof(rev2_pins);
      break;
     default: // This will need to change if there is a V3 pinout
      pins = rpi2_pins;
      gpios = rpi2_gpios;
      npins = sizeof(rpi2_pins);
      if (rev > 20){ // Rev 20 is Compute Module 4
	int db = rtapi_get_msg_level();
	rtapi_set_msg_level(3);
	rev = get_rpi_revision(); // call the function with a higher message level to print model
	rtapi_print_msg(RTAPI_MSG_INFO, "The Pi model %i is not known to "
	      "work with this driver but will be assumed to be be using "
	      "the RPi2+ layout 40 pin connector\n", rev);
	rtapi_set_msg_level(db);
      }
      break;
    }
    port_data = hal_malloc(npins * sizeof(void *));
    if (port_data == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL_PI_GPIO: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -1;
    }

    if (dir == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "HAL_PI_GPIO: ERROR: no config string\n");
	return -1;
    }
    dir_map = strtoul(dir, &endptr,0);
    if (*endptr) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"HAL_PI_GPIO: dir=%s - trailing garbage: '%s'\n",
			dir, endptr);
	return -1;
    }

    if (exclude == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "HAL_PI_GPIO: ERROR: no exclude string\n");
	return -1;
    }
    exclude_map = strtoul(exclude, &endptr,0);
    if (*endptr) {
	rtapi_print_msg(RTAPI_MSG_ERR,
			"HAL_PI_GPIO: exclude=%s - trailing garbage: '%s'\n",
			exclude, endptr);
	return -1;
    }

    if (setup_gpiomem_access()) {
      if (setup_gpio_access(rev, ncores))
        return -1;
    }

    comp_id = hal_init("hal_pi_gpio");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL_PI_GPIO: ERROR: hal_init() failed\n");
	return -1;
    }

    for (n = 0; n < npins; n++) {
      if (exclude_map & RTAPI_BIT(n))
	continue;
      pinno = pins[n];
      if (dir_map & RTAPI_BIT(n)) {
	bcm2835_gpio_fsel(gpios[n], BCM2835_GPIO_FSEL_OUTP);
	if ((retval = hal_pin_bit_newf(HAL_IN, &port_data[n],
				       comp_id, "hal_pi_gpio.pin-%02d-out", pinno)) < 0)
	  break;
      } else {
	bcm2835_gpio_fsel(gpios[n], BCM2835_GPIO_FSEL_INPT);
	if ((retval = hal_pin_bit_newf(HAL_OUT, &port_data[n],
				       comp_id, "hal_pi_gpio.pin-%02d-in", pinno)) < 0)
	  break;
      }
    }
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
		      "HAL_PI_GPIO: ERROR: pin %d export failed with err=%i\n",
		      n,retval);
      hal_exit(comp_id);
      return -1;
    }

    retval = hal_export_funct("hal_pi_gpio.write", write_port, 0,
			      0, 0, comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL_PI_GPIO: ERROR: write funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }
    retval = hal_export_funct("hal_pi_gpio.read", read_port, 0,
			      0, 0, comp_id);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL_PI_GPIO: ERROR: read funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
	"HAL_PI_GPIO: installed driver\n");
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
  if (gpio)
    munmap((void *) gpio, BCM2835_BLOCK_SIZE);
  if (mem_fd > -1)
      close(mem_fd);
  hal_exit(comp_id);
}

static void write_port(void *arg, long period)
{
  (void)arg;
  (void)period;
  int n;

  for (n = 0; n < npins; n++) {
    if (exclude_map & RTAPI_BIT(n))
      continue;
    if (dir_map & RTAPI_BIT(n)) {
      if (*(port_data[n])) {
	bcm2835_gpio_set(gpios[n]);
      } else {
	bcm2835_gpio_clr(gpios[n]);
      }
    }
  }
}

static void read_port(void *arg, long period)
{
  (void)arg;
  (void)period;
  int n;

  for (n = 0; n < npins; n++) {
    if ((~dir_map & RTAPI_BIT(n)) && (~exclude_map & RTAPI_BIT(n)))
      *port_data[n] = bcm2835_gpio_lev(gpios[n]);
  }
}
