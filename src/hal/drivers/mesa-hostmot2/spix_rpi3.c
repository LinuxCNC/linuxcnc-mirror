/*
 * This is a component for hostmot2 over SPI for linuxcnc.
 * Copyright (c) 2024 B.Stultiens <lcnc@vagrearg.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, see <https://www.gnu.org/licenses/>.
 */

/*
 * NOTE: This driver will detect and drive both Raspberry Pi 3 and 4 variants
 */

#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <rtapi.h>

#define HM2_LLIO_NAME "spix_rpi3"

#include "hostmot2-lowlevel.h"

#include "eshellf.h"
#include "spix.h"
#include "dtcboards.h"
#include "spi_common_rpspi.h"

//#define RPSPI_DEBUG_PIN	23	// Define for pin-debugging

// The absolute max allowed frequency of the SPI clock
#define SCLK_FREQ_MAX		50000000

#define SPI_MAX_SPI			2		// SPI0 and SPI1

// GPIO pin definitions				// (header pin location)
#define SPI0_PIN_CE_1		7		// (pin 26)
#define SPI0_PIN_CE_0		8		// (pin 24)
#define SPI0_PIN_MISO		9		// (pin 21)
#define SPI0_PIN_MOSI		10		// (pin 19)
#define SPI0_PIN_SCLK		11		// (pin 23)
#define SPI1_PIN_CE_2		16		// (pin 36)
#define SPI1_PIN_CE_1		17		// (pin 11)
#define SPI1_PIN_CE_0		18		// (pin 12)
#define SPI1_PIN_MISO		19		// (pin 35)
#define SPI1_PIN_MOSI		20		// (pin 38)
#define SPI1_PIN_SCLK		21		// (pin 40)

// The default pullup/pulldown on the SPI data input pin
#define SPI_PULL_MISO_DEF	SPIX_PULL_DOWN
#define SPI_PULL_MOSI_DEF	SPIX_PULL_OFF
#define SPI_PULL_SCLK_DEF	SPIX_PULL_OFF
#define SPI_PULL_CE_X_DEF	SPIX_PULL_UP

/*
 * Our SPI port descriptor extends the spix port descriptor.
 */
typedef struct __spi_port_t {
	spix_port_t	spix;		// The upstream container
	int			isopen;		// Non-zero if successfully opened
	int			spiport;	// Set to 0 for SPI0 and 1 for SPI1
	uint32_t	clkdivw;	// Write clock divider setting
	uint32_t	clkdivr;	// Read clock divider setting
	uint32_t	cemask;		// Read clock divider setting
	uint32_t	freqmin;	// Calculated minimal frequency for port
	uint32_t	freqmax;	// Calculated maximum frequency for port
} rpi3_port_t;

/* Forward decls */
static int rpi3_detect(const char *dtcs[]);
static int rpi3_setup(int probemask);
static int rpi3_cleanup(void);
static const spix_port_t *rpi3_open(int port, const spix_args_t *args);
static int rpi3_close(const spix_port_t *sp);
static int spi0_transfer(const spix_port_t *sp, uint32_t *wptr, size_t txlen, int rw);
static int spi1_transfer(const spix_port_t *sp, uint32_t *wptr, size_t txlen, int rw);

#define PORT_MAX	5
#define PORT_SPI0	0	// port index for hardware SPI0
#define PORT_SPI1	2	// port index for hardware SPI1
static rpi3_port_t spi_ports[PORT_MAX] = {
	{	.spix = { .width =  8, .miso_pull = SPI_PULL_MISO_DEF, .name = "SPI0/CE0", .transfer = spi0_transfer },
		.spiport = 0, .cemask = 0,
	},
	{	.spix = { .width =  8, .miso_pull = SPI_PULL_MISO_DEF, .name = "SPI0/CE1", .transfer = spi0_transfer },
		.spiport = 0, .cemask = SPI_CS_CS_01,
	},
	{	.spix = { .width = 16, .miso_pull = SPI_PULL_MISO_DEF, .name = "SPI1/CE0", .transfer = spi1_transfer },
		.spiport = 1, .cemask = AUX_SPI_CNTL0_CS_1 | AUX_SPI_CNTL0_CS_2,
	},
	{	.spix = { .width = 16, .miso_pull = SPI_PULL_MISO_DEF, .name = "SPI1/CE1", .transfer = spi1_transfer },
		.spiport = 1, .cemask = AUX_SPI_CNTL0_CS_0 | AUX_SPI_CNTL0_CS_2,
	},
	{	.spix = { .width = 16, .miso_pull = SPI_PULL_MISO_DEF, .name = "SPI1/CE2", .transfer = spi1_transfer },
		.spiport = 1, .cemask = AUX_SPI_CNTL0_CS_0 | AUX_SPI_CNTL0_CS_1,
	},
};

/*
 * The driver interface structure
 */
spix_driver_t spix_driver_rpi3 = {
	.name		= HM2_LLIO_NAME,
	.num_ports	= PORT_MAX,

	.detect		= rpi3_detect,
	.setup		= rpi3_setup,
	.cleanup	= rpi3_cleanup,
	.open		= rpi3_open,
	.close		= rpi3_close,
};

static int has_spi_module;		// Set to non-zero when the kernel module spi_bcm2835 is loaded
static int driver_enabled;		// Set to non-zero when rpi3_setup() is successfully called
static int port_probe_mask;		// Which ports are requested

static void *peripheralmem = MAP_FAILED;	// mmap'ed peripheral memory
static size_t peripheralsize;	// Size of the mmap'ed block

static bcm2835_gpio_t *gpio;	// GPIO peripheral structure in mmap'ed address space
static bcm2835_spi_t *spi;		// SPI peripheral structure in mmap'ed address space
static bcm2835_aux_t *aux;		// AUX peripheral structure in mmap'ed address space
static uint32_t aux_enables;	// Previous state of SPI1 enable

#define F_PERI	500000000UL
static uint32_t spiclk_base = F_PERI;	// The base clock (sys_clk) for the SPI port dividers

/*********************************************************************/
#if defined(RPSPI_DEBUG_PIN)
/*
 * Set/Clear a GPIO pin
 */
HWREGACCESS_ALWAYS_INLINE static inline void gpio_set(uint32_t pin)
{
	if(pin <= 53) {	/* There are 54 GPIOs */
		reg_wr(&gpio->gpset[pin / 32], 1 << (pin % 32));
	}
}

HWREGACCESS_ALWAYS_INLINE static inline void gpio_clr(uint32_t pin)
{
	if(pin <= 53) {	/* There are 54 GPIOs */
		reg_wr(&gpio->gpclr[pin / 32], 1 << (pin % 32));
	}
}

HWREGACCESS_ALWAYS_INLINE static inline void gpio_debug_pin(bool set_reset)
{
	if(set_reset)
		gpio_set(RPSPI_DEBUG_PIN);
	else
		gpio_clr(RPSPI_DEBUG_PIN);
}

#else

HWREGACCESS_ALWAYS_INLINE static inline void gpio_debug_pin(bool set_reset)
{
	(void)set_reset;
}

#endif
/*********************************************************************/
/*
 * Calculate the clock divider for spi0 port
 */
static int32_t spi0_clkdiv_calc(uint32_t base, uint32_t rate)
{
	uint32_t clkdiv = (base + rate - 1) / rate;
	// Use only even divider values
	// This is what the documentation (probably) states
	if(clkdiv > 65534)
		clkdiv = 0;		// Slowest possible
	else
		clkdiv += clkdiv & 1;	// Must be multiple of 2 (round to lower frequency)
	return clkdiv;
}

/*
 * Reset the SPI peripheral to inactive state and flushed
 */
static inline void spi0_reset(void)
{
	uint32_t x = reg_rd(&spi->cs);
	// Disable all activity
	x &= ~(SPI_CS_INTR | SPI_CS_INTD | SPI_CS_DMAEN | SPI_CS_TA);
	// and reset RX/TX fifos
	x |= SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX;
	reg_wr(&spi->cs, x);
	// Other registers are don't care for us, not using DMA
}

/*
 * Transfer a buffer of words to the SPI port and fill the same buffer with the
 * data coming from the SPI port.
 */
static int spi0_transfer(const spix_port_t *sp, uint32_t *wptr, size_t txlen, int rw)
{
	rpi3_port_t *rp = (rpi3_port_t *)sp;
	uint8_t *w8ptr = (uint8_t *)wptr;
	uint8_t *r8ptr = (uint8_t *)wptr;	// read into write buffer
	size_t tx8len = txlen * sizeof(uint32_t);	// Bytes to send
	size_t rx8len = txlen * sizeof(uint32_t);	// Bytes to read
	size_t u;
	uint32_t cs;

	gpio_debug_pin(false);

	if(!txlen)
		return 1;	// Nothing to do, return success

	// Using 8-bit transfers; need to byte-swap to big-endian to get the word's
	// most significant bit shifted out first.
	for(u = 0; u < txlen; u++)
		wptr[u] = htobe32(wptr[u]);

	// Setup transfer
	cs = reg_rd(&spi->cs);
	cs &= ~(SPI_CS_CS_10 | SPI_CS_CS_01 | SPI_CS_REN);	// Reset CE and disable 3-wire mode
	cs |= rp->cemask | SPI_CS_TA;						// Set proper CE_x and activate transfer
	reg_wr(&spi->clk, rw ? rp->clkdivr : rp->clkdivw);	// Set clock divider
	reg_wr(&spi->cs, cs);								// Go!

	// Fill the TX fifo with as much as it can hold
	while(tx8len > 0 && (reg_rd(&spi->cs) & SPI_CS_TXD)) {
		reg_wr(&spi->fifo, *w8ptr);
		++w8ptr;
		--tx8len;
	}

	// Read and write until all done
	while(rx8len > 0) {
		cs = reg_rd(&spi->cs);
		if(cs & SPI_CS_RXD) {
			*r8ptr = (uint8_t)reg_rd(&spi->fifo);
			++r8ptr;
			--rx8len;
		}
		if(tx8len > 0 && (cs & SPI_CS_TXD)) {
			reg_wr(&spi->fifo, *w8ptr);
			++w8ptr;
			--tx8len;
		}
	}

	// Stop transfer, after last byte received we are done
	spi0_reset();

	// Put the received words into host order
	for(u = 0; u < txlen; u++)
		wptr[u] = be32toh(wptr[u]);

	gpio_debug_pin(true);
	return 1;
}

/*************************************************/

/*
 * Calculate the clock divider for spi1 port
 */
static int32_t spi1_clkdiv_calc(uint32_t base, uint32_t rate)
{
	uint32_t clkdiv;
	if(rate >= base / 2)
		return 0;
	clkdiv = (base + rate*2 - 1) / (rate * 2) - 1;
	if(clkdiv > 4095)
		clkdiv = 4095;		// Slowest possible
	return clkdiv;
}

static inline void spi1_reset(void)
{
	// Disable and clear fifos
	reg_wr(&aux->spi0_cntl0, 0);
	reg_wr(&aux->spi0_cntl1, AUX_SPI_CNTL0_CLEARFIFO);
}

/*
 * This is needed because the SPI ports on AUX suck. It seems that the BCM2835
 * hardware cannot keep track of its own fifo depth and we need to count for
 * it. Using the fifo level status bits from the status register (spi0_stat)
 * seems to work at first, but will fail very soon after. This may relate to
 * the AUX_SPI_STAT_TX_FULL not being updated with correct timing. We are
 * apparently losing data in the transfer, which causes LinuxCNC to stall in
 * the real-time thread, which is a Bad Thing(TM).
 * The Linux kernel driver seems to have the number 12, but I cannot get it to
 * work with that number. The depth of four seems the highest working value.
 * Either way, we already busy-loop here anyway, so it shouldn't matter.
 */
#define SPI1_FIFO_MAXDEPTH	4

static int spi1_transfer(const spix_port_t *sp, uint32_t *wptr, size_t txlen, int rw)
{
	rpi3_port_t *rp = (rpi3_port_t *)sp;
	uint16_t *w16ptr = (uint16_t *)wptr;
	uint16_t *r16ptr = (uint16_t *)wptr;
	size_t tx16len = txlen * 2;	// There are twice as many 16-bit words as there are 32-bit words
	size_t rx16len = txlen * 2;
	size_t u;
	unsigned pending = 0;

	if(!txlen)
		return 1;	// Nothing to do, return success

	gpio_debug_pin(false);

	// Word-swap to assure most significant bit to be sent first in 16-bit transfers.
	for(u = 0; u < txlen * 2; u += 2) {
		uint16_t tt = w16ptr[u+0];
		w16ptr[u+0] = w16ptr[u+1];
		w16ptr[u+1] = tt;
	}

	// Setup clock speed and format
	// Note: It seems that we cannot send 32 bits. Shift length 0 sends zero
	// bits and there are only 6 bits available to set the length, giving a
	// maximum of 31 bits. Furthermore, the shift seems to be delayed one
	// clock.
	// Using variable width requires the upper byte of the data-word to hold
	// the number of bits to shift. That too cannot be 32 because we cannot
	// both use it for data and the shift count at the same time. The variable
	// width needs to be left-aligned at bit 23.
	reg_wr(&aux->spi0_cntl0,  AUX_SPI_CNTL0_SPEED(rw ? rp->clkdivr : rp->clkdivw)
				| AUX_SPI_CNTL0_ENABLE
				| AUX_SPI_CNTL0_MSB_OUT
				| AUX_SPI_CNTL0_IN_RISING
				| AUX_SPI_CNTL0_VAR_WIDTH
				| rp->cemask);
	reg_wr(&aux->spi0_cntl1, AUX_SPI_CNTL1_MSB_IN);

	// Send data to the fifo
	while(tx16len > 0 && pending < SPI1_FIFO_MAXDEPTH && !(reg_rd(&aux->spi0_stat) & AUX_SPI_STAT_TX_FULL)) {
		if(tx16len > 1)
			reg_wr(&aux->spi0_hold, ((uint32_t)*w16ptr << 8) | (16 << 24));	// More data to follow
		else
			reg_wr(&aux->spi0_io, ((uint32_t)*w16ptr << 8) | (16 << 24));	// Final write
		++w16ptr;
		--tx16len;
		++pending;
	}

	// Read and write until all done
	while(rx16len > 0) {
		uint32_t stat = reg_rd(&aux->spi0_stat);
		if(!(stat & AUX_SPI_STAT_RX_EMPTY)) {
			*r16ptr = (uint16_t)reg_rd(&aux->spi0_io);	// Read available word
			++r16ptr;
			--rx16len;
			--pending;
		}
		if(tx16len > 0 && pending < SPI1_FIFO_MAXDEPTH && !(stat & AUX_SPI_STAT_TX_FULL)) {
			if(tx16len > 1)
				reg_wr(&aux->spi0_hold, ((uint32_t)*w16ptr << 8) | (16 << 24));	// More data to follow
			else
				reg_wr(&aux->spi0_io, ((uint32_t)*w16ptr << 8) | (16 << 24));	// Final write
			++w16ptr;
			--tx16len;
			++pending;
		}
	}

	// Stop transfer
	spi1_reset();

	// Word-swap to fix the word order back to host-order.
	w16ptr = (uint16_t *)wptr;
	for(u = 0; u < txlen * 2; u += 2) {
		uint16_t tt = w16ptr[u+0];
		w16ptr[u+0] = w16ptr[u+1];
		w16ptr[u+1] = tt;
	}

	gpio_debug_pin(true);
	return 1;
}

/*************************************************/
/*
 * Map peripheral I/O memory in the process' address space.
 * Setup pointers to the relevant structures to access the underlying hardware.
 */
static int peripheral_map(uintptr_t membase, size_t memsize)
{
	int fd;
	int err;

	peripheralsize = memsize;

	if((fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		LL_ERR("Can't open /dev/mem\n");
		return -errno;
	}

	/* mmap BCM2835 GPIO and SPI peripherals */
	peripheralmem = mmap(NULL, peripheralsize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, (off_t)membase);
	err = errno;
	close(fd);
	if(peripheralmem == MAP_FAILED) {
		LL_ERR("Can't map peripherals: %s\n", strerror(err));
		if(err == EPERM) {
			LL_ERR("Try adding 'iomem=relaxed' to your kernel command-line.\n");
		}
		return -err;
	}

	// The Right Way(TM) may be to extract the reg mappings for the
	// specific devices at /dev/device-tree/soc/*. Then we'd need to
	// inspect the gpio@... and spi@... device nodes, read the
	// corresponding reg file and correct the address with respect to the
	// virtual vs. physical mappings. Too much work... These are all
	// compatible devices and nobody in their right mind (famous last
	// words) will change that after the large quantity of devices out in
	// the wild.
	// Lets just say, when somebody decides to change the world, then we'll
	// fix all this code too.
	gpio = (bcm2835_gpio_t *)(peripheralmem + (BCM2835_GPIO_OFFSET / sizeof(*peripheralmem)));
	spi  = (bcm2835_spi_t *)(peripheralmem + (BCM2835_SPI_OFFSET  / sizeof(*peripheralmem)));
	aux  = (bcm2835_aux_t *)(peripheralmem + (BCM2835_AUX_OFFSET  / sizeof(*peripheralmem)));

	LL_INFO("Mapped peripherals from 0x%p (size 0x%08x) to gpio:0x%p, spi:0x%p, aux:0x%p\n",
			(void *)membase, (uint32_t)peripheralsize, gpio, spi, aux);

	return 0;
}

/*************************************************/
static void waste_150_cycles(void)
{
	uint32_t x __attribute__((unused));
	unsigned i;
	// A read, memory barrier, an increment, a test and a jump. Should be at least 150 cycles
	for(i = 0; i < 40; i++)
		x = reg_rd(&gpio->gplev0);	// Just read the pin register, nothing interesting to do here
}

static void gpio_fsel(uint32_t pin, uint32_t func)
{
	if(pin <= 53) {	/* There are 54 GPIOs */
		uint32_t bits = pin * 3;	/* Three bits per fsel field and 10 gpio per uint32_t */
		reg_wr(&gpio->gpfsel[bits / 30], (reg_rd(&gpio->gpfsel[bits / 30]) & ~(7 << (bits % 30))) | ((func & 7) << (bits % 30)));
	}
}

static void gpio_pull(unsigned pin, uint32_t pud)
{
	// Enable/disable pullups on the pins on request
	reg_wr(&gpio->gppudclk0, 0);	// We are not sure about the previous state, make sure
	reg_wr(&gpio->gppudclk1, 0);
	waste_150_cycles();		// See GPPUDCLKn description
	reg_wr(&gpio->gppud, pud);
	waste_150_cycles();
	if(pin <= 31) {
		reg_wr(&gpio->gppudclk0, 1u << pin);
		waste_150_cycles();
		reg_wr(&gpio->gppudclk0, 0);
	} else if(pin <= 53) {
		reg_wr(&gpio->gppudclk1, 1u << (pin - 32));
		waste_150_cycles();
		reg_wr(&gpio->gppudclk1, 0);
	}
}

static void peripheral_setup(void)
{
	// Setup SPI pins to SPI0
	if(port_probe_mask & SPIX_PROBE_SPI0_MASK) {
		if(port_probe_mask & SPIX_PROBE_SPI0_CE0) {
			gpio_fsel(SPI0_PIN_CE_0, GPIO_FSEL_8_SPI0_CE0_N);
			gpio_pull(SPI0_PIN_CE_0, GPIO_GPPUD_PULLUP);
		}
		if(port_probe_mask & SPIX_PROBE_SPI0_CE1) {
			gpio_fsel(SPI0_PIN_CE_1, GPIO_FSEL_7_SPI0_CE1_N);
			gpio_pull(SPI0_PIN_CE_1, GPIO_GPPUD_PULLUP);
		}
		gpio_fsel(SPI0_PIN_MISO, GPIO_FSEL_9_SPI0_MISO);
		gpio_fsel(SPI0_PIN_MOSI, GPIO_FSEL_10_SPI0_MOSI);
		gpio_fsel(SPI0_PIN_SCLK, GPIO_FSEL_11_SPI0_SCLK);
		// Enable/disable pullups on the pins
		gpio_pull(SPI0_PIN_MISO, SPI_PULL_MISO_DEF);
		gpio_pull(SPI0_PIN_MOSI, SPI_PULL_MOSI_DEF);
		gpio_pull(SPI0_PIN_SCLK, SPI_PULL_SCLK_DEF);
		spi0_reset();
	}

	// Setup SPI pins to SPI1
	if(port_probe_mask & SPIX_PROBE_SPI1_MASK) {
		if(port_probe_mask & SPIX_PROBE_SPI1_CE0) {
			gpio_fsel(SPI1_PIN_CE_0, GPIO_FSEL_18_SPI1_CE0_N);
			gpio_pull(SPI1_PIN_CE_0, GPIO_GPPUD_PULLUP);
		}
		if(port_probe_mask & SPIX_PROBE_SPI1_CE1) {
			gpio_fsel(SPI1_PIN_CE_1, GPIO_FSEL_17_SPI1_CE1_N);
			gpio_pull(SPI1_PIN_CE_1, GPIO_GPPUD_PULLUP);
		}
		if(port_probe_mask & SPIX_PROBE_SPI1_CE2) {
			gpio_fsel(SPI1_PIN_CE_1, GPIO_FSEL_16_SPI1_CE2_N);
			gpio_pull(SPI1_PIN_CE_2, GPIO_GPPUD_PULLUP);
		}
		gpio_fsel(SPI1_PIN_MISO, GPIO_FSEL_19_SPI1_MISO);
		gpio_fsel(SPI1_PIN_MOSI, GPIO_FSEL_20_SPI1_MOSI);
		gpio_fsel(SPI1_PIN_SCLK, GPIO_FSEL_21_SPI1_SCLK);
		// Enable/disable pullups on the pins
		gpio_pull(SPI1_PIN_MISO, SPI_PULL_MISO_DEF);
		gpio_pull(SPI1_PIN_MOSI, SPI_PULL_MOSI_DEF);
		gpio_pull(SPI1_PIN_SCLK, SPI_PULL_SCLK_DEF);

		// Check if SPI1 needs to be enabled
		aux_enables = reg_rd(&aux->enables);
		if(!(aux_enables & AUX_ENABLES_SPI1)) {
			reg_wr(&aux->enables, aux_enables | AUX_ENABLES_SPI1);	// Enable SPI1
			LL_WARN("SPI1 needed to be enabled.\n");
		}

		spi1_reset();
	}

#if defined(RPSPI_DEBUG_PIN)
	gpio_fsel(RPSPI_DEBUG_PIN, GPIO_FSEL_X_GPIO_OUTPUT);
	gpio_pull(RPSPI_DEBUG_PIN, GPIO_GPPUD_PULLUP);
	gpio_debug_pin(true);
#endif
}

static void peripheral_restore(void)
{
#if defined(RPSPI_DEBUG_PIN)
	gpio_debug_pin(true);
	gpio_fsel(RPSPI_DEBUG_PIN, GPIO_FSEL_X_GPIO_INPUT);
#endif
	// Restore all SPI pins to inputs and enable pull-up (no dangling inputs)
	if(port_probe_mask & SPIX_PROBE_SPI0_MASK) {
		gpio_pull(SPI0_PIN_MISO, GPIO_GPPUD_PULLUP);
		gpio_pull(SPI0_PIN_MOSI, GPIO_GPPUD_PULLUP);
		gpio_pull(SPI0_PIN_SCLK, GPIO_GPPUD_PULLUP);

		// Set SPI0 pins to GPIO input
		gpio_fsel(SPI0_PIN_MISO, GPIO_FSEL_X_GPIO_INPUT);
		gpio_fsel(SPI0_PIN_MOSI, GPIO_FSEL_X_GPIO_INPUT);
		gpio_fsel(SPI0_PIN_SCLK, GPIO_FSEL_X_GPIO_INPUT);
		if(port_probe_mask & SPIX_PROBE_SPI0_CE0)
			gpio_fsel(SPI0_PIN_CE_0, GPIO_FSEL_X_GPIO_INPUT);
		if(port_probe_mask & SPIX_PROBE_SPI0_CE1)
			gpio_fsel(SPI0_PIN_CE_1, GPIO_FSEL_X_GPIO_INPUT);
	}

	if(port_probe_mask & SPIX_PROBE_SPI1_MASK) {
		// Only disable SPI1 if it was disabled before
		if(!(aux_enables & AUX_ENABLES_SPI1))
			reg_wr(&aux->enables, reg_rd(&aux->enables) & ~AUX_ENABLES_SPI1);

		gpio_pull(SPI1_PIN_MISO, GPIO_GPPUD_PULLUP);
		gpio_pull(SPI1_PIN_MOSI, GPIO_GPPUD_PULLUP);
		gpio_pull(SPI1_PIN_SCLK, GPIO_GPPUD_PULLUP);

		// Set SPI1 pins to GPIO input
		gpio_fsel(SPI1_PIN_MISO, GPIO_FSEL_X_GPIO_INPUT);
		gpio_fsel(SPI1_PIN_MOSI, GPIO_FSEL_X_GPIO_INPUT);
		gpio_fsel(SPI1_PIN_SCLK, GPIO_FSEL_X_GPIO_INPUT);
		if(port_probe_mask & SPIX_PROBE_SPI1_CE0)
			gpio_fsel(SPI1_PIN_CE_0, GPIO_FSEL_X_GPIO_INPUT);
		if(port_probe_mask & SPIX_PROBE_SPI1_CE1)
			gpio_fsel(SPI1_PIN_CE_1, GPIO_FSEL_X_GPIO_INPUT);
		if(port_probe_mask & SPIX_PROBE_SPI1_CE2)
			gpio_fsel(SPI1_PIN_CE_2, GPIO_FSEL_X_GPIO_INPUT);
	}
}

/*************************************************/
#define RPSPI_SYS_CLKVPU	"/sys/kernel/debug/clk/vpu/clk_rate"	// Newer kernels (4.8+, I think) have detailed info
#define RPSPI_SYS_CLKCORE	"/sys/kernel/debug/clk/core/clk_rate"	// Older kernels only have core-clock info

static uint32_t read_spiclkbase(void)
{
	const char *sysclkref = RPSPI_SYS_CLKVPU;
	uint32_t rate;
	char buf[16];
	ssize_t err;

	if((err = spix_read_file(sysclkref, buf, sizeof(buf))) < 0) {
		// Failed VPU clock, try core clock
		LL_INFO("No VPU clock at '%s' (errno=%d), trying core clock as alternative.\n", sysclkref, errno);
		sysclkref = RPSPI_SYS_CLKCORE;
		if((err = spix_read_file(sysclkref, buf, sizeof(buf))) < 0) {
			// Neither clock available, complain and use default setting
			LL_ERR("Cannot open clock setting '%s' (errno=%d), using %d Hz\n", sysclkref, errno, spiclk_base);
			return spiclk_base;
		}
	}

	if(err >= (int)sizeof(buf)-1) {
		// There are probably too many digits in the number
		// 250000000 (250 MHz) has 9 digits and there is a newline
		// following the number
		LL_ERR("Read full buffer '%s' from '%s', number probably wrong or too large, using %d Hz\n", buf, sysclkref, spiclk_base);
		return spiclk_base;
	}

	if(1 != sscanf(buf, "%u", &rate)) {
		LL_ERR("Cannot interpret clock setting '%s' from '%s', using %d Hz\n", buf, sysclkref, spiclk_base);
		return spiclk_base;
	}
	LL_INFO("SPI clock base frequency: %u\n", rate);
	return rate;
}

/*************************************************/
#define DTC_SOC_RANGES	"/proc/device-tree/soc/ranges"

static int read_membase(uintptr_t *pmembase, size_t *pmemsize)
{
	uint32_t buf[4+1];
	ssize_t len;

	*pmembase = 0;
	*pmemsize = 0;

	// Extract the IO base and size
	// The ranges file in the device-tree has the physical mappings of the
	// IO space we need to map. There are several interesting values in big
	// endian:
	// RPi3 (BCM2836)
	//	[0]: Real register file address
	//	[1]: IO register file base address
	//	[2]: IO register file size
	//	...
	// RPi4 (BCM2838)
	//	[0]: Real register file address
	//	[1]: 0x00000000
	//	[2]: IO register file base address
	//	[3]: IO register file size
	//	...
	//
	// The definitions for the device-tree are in the (rpi) linux source
	// tree to be found at arch/arm/boot/dts/bcm283[568]*.
	//
	// We read into a buffer that is large enough for more than four 32-bit
	// words. These are required to be present for the RPi3 and RPi4 (and
	// older versions).
	if((len = spix_read_file(DTC_SOC_RANGES, buf, sizeof(buf))) < 0) {
		LL_ERR("Cannot read IO base address and size from '%s'.\n", DTC_SOC_RANGES);
		return len;
	}

	if(len / sizeof(uint32_t) < 3) {
		LL_ERR("Insufficient data read from '%s' for RPi3/RPi4 IO base address and size.\n", DTC_SOC_RANGES);
		return -ENXIO;
	}

	*pmembase = be32toh(buf[1]);	// This should do the trick for RPi3
	*pmemsize = be32toh(buf[2]);

	if(!*pmembase) {
		// This is (probably) a RPi4 and the ranges file has a zero at the base
		// address. Here we need to have read four 32-bit words to get to the
		// right values.
		if(len / sizeof(uint32_t) < 4) {
			LL_ERR("Insufficient data read from '%s' for RPi4 IO base address and size.\n", DTC_SOC_RANGES);
			return -ENXIO;
		}

		*pmembase = be32toh(buf[2]);
		*pmemsize = be32toh(buf[3]);
	}

	if(!*pmembase || !*pmemsize) {
		LL_ERR("IO base address (0x%p) or size (0x%08zx) are zero.\n", (void *)*pmembase, *pmemsize);
		return -ENXIO;
	}
	LL_INFO("Base address 0x%p size 0x%08zx\n", (void *)*pmembase, *pmemsize);
	return 0;
}

/*************************************************/

/*
 * Detect the presence of the hardware on basis of the device-tree compatible
 * string-list.
 * On success returns 9 (zero) and sets the driver information to the dtc
 * string and tries to set a human readable string as model.
 */
static int rpi3_detect(const char *dtcs[])
{
	int i;
	for(i = 0; dtcs[i] != NULL; i++) {
		if(		!strcmp(dtcs[i], DTC_RPI_MODEL_4B)
			||	!strcmp(dtcs[i], DTC_RPI_MODEL_4CM)
			||	!strcmp(dtcs[i], DTC_RPI_MODEL_3BP)
			||	!strcmp(dtcs[i], DTC_RPI_MODEL_3AP)
			||	!strcmp(dtcs[i], DTC_RPI_MODEL_3B)) {
			break;	// Found our supported board
		}
	}

	if(dtcs[i] == NULL)
		return -ENODEV;	// We are not the device the driver supports

	// Set the matched dtc and model informational strings
	strncpy(spix_driver_rpi3.dtc, dtcs[i], sizeof(spix_driver_rpi3.dtc)-1);
	if(spix_read_file("/proc/device-tree/model", spix_driver_rpi3.model, sizeof(spix_driver_rpi3.model)) < 0)
		strncpy(spix_driver_rpi3.model, "??? Unknown board ???", sizeof(spix_driver_rpi3.model)-1);

	return 0;
}

/*
 * Setup the driver.
 * - remove kernel spidev driver modules if detected
 * - map the I/O memory
 * - setup the GPIO pins and SPI peripheral(s)
 */
static int rpi3_setup(int probemask)
{
	int err;
	uintptr_t membase;
	size_t memsize;
	int i;

	if(driver_enabled) {
		LL_ERR("Driver is already setup.\n");
		return -EBUSY;
	}

	port_probe_mask = probemask;	// For peripheral_setup() and peripheral_restore()

	// Now we know what platform we are running, remove kernel SPI module if
	// detected
	if((has_spi_module = (0 == shell("/usr/bin/grep -qw ^spi_bcm2835 /proc/modules")))) {
		if(shell("/sbin/rmmod spi_bcm2835"))
			LL_ERR("Unable to remove kernel SPI module spi_bcm2835. "
					"Your system may become unstable using LinuxCNC with the " HM2_LLIO_NAME " driver.\n");
	}

	spiclk_base = read_spiclkbase();

	// calculate the actual min/max frequencies
	for(i = 0; i < PORT_MAX; i++) {
		if(!spi_ports[i].spiport) {
			spi_ports[i].freqmin = spiclk_base / 65536;
			spi_ports[i].freqmax = spix_min((spiclk_base / 2), SCLK_FREQ_MAX);
		} else {
			spi_ports[i].freqmin = spiclk_base / (2 * (4095+1));
			spi_ports[i].freqmax = spix_min((spiclk_base / 2), SCLK_FREQ_MAX);
		}
	}

	if((err = read_membase(&membase, &memsize)) < 0)
		return err;

	if((err = peripheral_map(membase, memsize)) < 0) {
		LL_ERR("Cannot map peripheral memory.\n");
		return err;
	}

	peripheral_setup();

	driver_enabled = 1;

	return 0;
}

/*
 * Cleanup the driver
 * - close any open ports
 * - restore pripheral settings
 * - unmap I/ memory
 * - re-insert kernel spidev hardware module(s) is previously detected
 */
static int rpi3_cleanup(void)
{
	int i;

	if(!driver_enabled) {
		LL_ERR("Driver is not setup.\n");
		return -ENODEV;
	}

	// Close any ports not closed already
	for(i = 0; i < PORT_MAX; i++) {
		if(spi_ports[i].isopen)
			spix_driver_rpi3.close(&spi_ports[i].spix);
	}

	if(peripheralmem != MAP_FAILED) {
		peripheral_restore();
		munmap(peripheralmem, peripheralsize);
	}

	// Restore kernel SPI module if it was detected before
	if(has_spi_module)
		shell("/sbin/modprobe spi_bcm2835");

	driver_enabled = 0;
	return 0;
}

/*
 * Open a SPI port at index 'port' with 'clkw' write clock and 'clkr' read
 * clock frequencies.
 */
static const spix_port_t *rpi3_open(int port, const spix_args_t *args)
{
	rpi3_port_t *rpp;
	uint32_t ccw, ccr;

	if(!driver_enabled) {
		LL_ERR("Driver is not setup.\n");
		return NULL;
	}

	if(port < 0 || port >= PORT_MAX) {
		LL_ERR("SPI port %d out of range.\n", port);
		return NULL;
	}

	rpp = &spi_ports[port];

	if(!(port_probe_mask & (1 << port))) {
		LL_ERR("%s: SPI port %d not setup, was not in probe mask (%02x).\n", rpp->spix.name, port, port_probe_mask);
		return NULL;
	}

	if(rpp->isopen) {
		LL_ERR("%s: SPI port already open.\n", rpp->spix.name);
		return NULL;
	}

	if(args->clkw < rpp->freqmin || args->clkw > rpp->freqmax) {
		LL_ERR("%s: SPI write clock frequency outside acceptable range (%d..%d kHz).\n", rpp->spix.name, rpp->freqmin / 1000, rpp->freqmax / 1000);
		return NULL;
	}

	if(!rpp->spiport) {
		rpp->clkdivw = spi0_clkdiv_calc(spiclk_base, args->clkw);
		rpp->clkdivr = spi0_clkdiv_calc(spiclk_base, args->clkr);
		ccw = spiclk_base / rpp->clkdivw;
		ccr = spiclk_base / rpp->clkdivr;
	} else {
		rpp->clkdivw = spi1_clkdiv_calc(spiclk_base, args->clkw);
		rpp->clkdivr = spi1_clkdiv_calc(spiclk_base, args->clkr);
		ccw = spiclk_base / (2 * (rpp->clkdivw + 1));
		ccr = spiclk_base / (2 * (rpp->clkdivr + 1));
	}
	LL_INFO("%s: write clock rate calculated: %u Hz (clkdiv: %u)\n", rpp->spix.name, ccw, rpp->clkdivw);
	LL_INFO("%s: read clock rate calculated: %u Hz (clkdiv: %u)\n",  rpp->spix.name, ccr, rpp->clkdivr);

	rpp->isopen = 1;
	return &rpp->spix;
}

/*
 * Close a SPI port.
 */
static int rpi3_close(const spix_port_t *sp)
{
	rpi3_port_t *rpp;

	if(!driver_enabled) {
		LL_ERR("Driver is not setup.\n");
		return -ENODEV;
	}

	if(!sp) {
		LL_ERR("Trying to close port NULL\n");
		return -EINVAL;
	}

	rpp = (rpi3_port_t *)sp;
	if(!rpp->isopen) {
		LL_ERR("%s: SPI port not open.\n", rpp->spix.name);
		return -ENODEV;
	}

	if(!rpp->spiport)
		spi0_reset();
	else
		spi1_reset();

	rpp->isopen = 0;
	return 0;
}

// vim: ts=4
