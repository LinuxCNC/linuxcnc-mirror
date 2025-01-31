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

#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <rtapi.h>

#define HM2_LLIO_NAME "spix_rpi5"

#include "hostmot2-lowlevel.h"

#include "eshellf.h"
#include "spix.h"
#include "dtcboards.h"
#include "rp1dev.h"

//#define RPSPI_DEBUG_PIN	23	// Define for pin-debugging

// The min/max allowed frequencies of the SPI clock
#define SCLK_FREQ_MIN		4000
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

// The default pullup/pulldown on the SPI pins
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
	dw_ssi_t	*port;		// Actual mapped I/O memory
	uint32_t	clkdivw;	// Write clock divider setting
	uint32_t	clkdivr;	// Read clock divider setting
	uint32_t	cemask;		// Read clock divider setting
} rpi5_port_t;

/* Forward decls */
static int rpi5_detect(const char *dtcs[]);
static int rpi5_setup(int probemask);
static int rpi5_cleanup(void);
static const spix_port_t *rpi5_open(int port, const spix_args_t *args);
static int rpi5_close(const spix_port_t *sp);
static int spi_transfer(const spix_port_t *sp, uint32_t *wptr, size_t txlen, int rw);

#define PORT_MAX	5
#define PORT_SPI0	0	// port index for hardware SPI0
#define PORT_SPI1	2	// port index for hardware SPI1
static rpi5_port_t spi_ports[PORT_MAX] = {
	{ .spix = { .width = 32, .miso_pull = SPI_PULL_MISO_DEF, .name = "SPI0/CE0", .transfer = spi_transfer }, .cemask = (1<<0) },
	{ .spix = { .width = 32, .miso_pull = SPI_PULL_MISO_DEF, .name = "SPI0/CE1", .transfer = spi_transfer }, .cemask = (1<<1) },
	{ .spix = { .width = 32, .miso_pull = SPI_PULL_MISO_DEF, .name = "SPI1/CE0", .transfer = spi_transfer }, .cemask = (1<<0) },
	{ .spix = { .width = 32, .miso_pull = SPI_PULL_MISO_DEF, .name = "SPI1/CE1", .transfer = spi_transfer }, .cemask = (1<<1) },
	{ .spix = { .width = 32, .miso_pull = SPI_PULL_MISO_DEF, .name = "SPI1/CE2", .transfer = spi_transfer }, .cemask = (1<<2) },
};

/*
 * The driver interface structure
 */
spix_driver_t spix_driver_rpi5 = {
	.name		= HM2_LLIO_NAME,
	.num_ports	= PORT_MAX,

	.detect		= rpi5_detect,
	.setup		= rpi5_setup,
	.cleanup	= rpi5_cleanup,
	.open		= rpi5_open,
	.close		= rpi5_close,
};

// Originals of the io_bank0.gpio[X].ctrl and pads_bank0.gpio registers so they
// can be restored later on exit.
typedef struct __spisave_t {
	uint32_t	bank_sclk;
	uint32_t	bank_mosi;
	uint32_t	bank_miso;
	uint32_t	bank_ce_0;
	uint32_t	bank_ce_1;
	uint32_t	bank_ce_2;
	uint32_t	pads_sclk;
	uint32_t	pads_mosi;
	uint32_t	pads_miso;
	uint32_t	pads_ce_0;
	uint32_t	pads_ce_1;
	uint32_t	pads_ce_2;
} spisave_t;

static spisave_t spi0save;	// Settings before our setup
static spisave_t spi1save;
static int has_spi_module;	// Set to non-zero when the kernel modules dw_spi and dw_spi_mmio are loaded
static int driver_enabled;	// Set to non-zero when rpi5_setup() is successfully called
static int port_probe_mask;	// Which ports are requested

static void *peripheralmem = MAP_FAILED;	// mmap'ed peripheral memory
static size_t peripheralsize;		// Size of the mmap'ed block
static rp1_rio_t *rio0;				// GPIO pin access structure in mmap'ed address space
static rp1_io_bank0_t *iobank0;		// GPIO pin config structure in mmap'ed address space
static rp1_pads_bank0_t *padsbank0;	// GPIO pin pads structure in mmap'ed address space


/*********************************************************************/
#ifdef RPSPI_DEBUG_PIN
HWREGACCESS_ALWAYS_INLINE static inline void gpio_set(int pin)
{
	if(pin >= 0 && pin < 28)
		reg_wr_raw(&rio0->set.out, 1 << pin);
}

HWREGACCESS_ALWAYS_INLINE static inline void gpio_clr(int pin)
{
	if(pin >= 0 && pin < 28)
		reg_wr_raw(&rio0->clr.out, 1 << pin);
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
 * Calculate the clock divider for any spi port
 */
static inline int32_t clkdiv_calc(uint32_t rate)
{
	uint32_t clkdiv = (RP1_SPI_CLK + rate - 1) / rate;
	// The documentation states: bit 0 is always zero, therefore, only even
	// divider values supported. Divider value 0 disables the SCLK output.
	if(clkdiv > 65534)
		clkdiv = 65534;			// Slowest possible
	else
		clkdiv += clkdiv & 1;	// Must be multiple of 2 (round to lower frequency)
	if(!clkdiv)
		clkdiv = 2;				// Do not disable, set it to absolute maximum frequency
	return clkdiv;
}

/*
 * Reset the SPI peripheral to inactive state and flushed
 */
static inline void spi_reset(dw_ssi_t *port)
{
	uint32_t dummy __attribute__((unused));
	reg_wr_raw(&port->ssienr, 0);	// Disable chip, will also clear fifo
	reg_wr_raw(&port->ser, 0);		// Clear all chip selects
	dummy = reg_rd_raw(&port->icr);	// Clear all interrupts
	// The chip will be enabled when a transfer is started
	// reg_wr(port->ssienr, DW_SSI_SSIENR_SSI_EN);	// Enable chip
}

/*
 * Transfer a buffer of words to the SPI port and fill the same buffer with the
 * data coming from the SPI port.
 */
static int spi_transfer(const spix_port_t *sp, uint32_t *wptr, size_t txlen, int rw)
{
	rpi5_port_t *rp = (rpi5_port_t *)sp;
	dw_ssi_t *port = rp->port;
	size_t rxlen = txlen;			// words to receive
	uint32_t *rptr = wptr;			// read into write buffer
	int fifo = RP1_SPI_FIFO_LEN;	// fifo level counter before starting transfer
	int rv = 1;						// function return value

	gpio_debug_pin(false);

	if(!txlen)
		return 1;	// Nothing to do, return success

	// Setup transfer
	// 32-bit, transmit/receive transfers, SPI mode 0 (CPHA=0, CPOL=0)
	reg_wr(&port->ctrlr0, DW_SSI_CTRLR0_DFS_32(32-1) | DW_SSI_CTRLR0_TMOD(DW_SSI_CTRLR0_TMOD_TXRX));
	reg_wr_raw(&port->baudr, rw ? rp->clkdivr : rp->clkdivw);
	reg_wr_raw(&port->ser, rp->cemask);

	reg_wr_raw(&port->ssienr, DW_SSI_SSIENR_SSI_EN);	// Enable port

	// Stuff the fifo until full or no more data words to write
	while(txlen > 0 && fifo > 0) {
		reg_wr_raw(&port->dr0, *wptr);	// Write data
		wptr++;
		txlen--;
		fifo--;
	}

	// We don't need to add a memory barrier. The next loop runs about rxlen
	// and the code will stall on the register reads. There is no read/write
	// overlap that may be problematic.

	while(rxlen > 0) {
		// Get the rx fifo level and read as many as available
		uint32_t tff = fifo = reg_rd_raw(&port->rxflr);
		// Already get the int status register; this read will pipeline
		uint32_t risr = reg_rd_raw(&port->risr);
		while(rxlen > 0 && fifo > 0) {
			*rptr = reg_rd_raw(&port->dr0);
			rptr++;
			rxlen--;
			fifo--;
		}

		// If we still have queued data, blindly write to stuff the tx-fifo.
		// For each received word we have one less entry in the transmit fifo.
		// It has to be. Therefore, we can use the receive level as a proxy for
		// how many words can be written
		while(txlen > 0 && tff > 0) {
			reg_wr_raw(&port->dr0, *wptr);
			wptr++;
			txlen--;
			tff--;
		}

		// Check for receive errors
		if(risr & (DW_SSI_RISR_RXOIR)) {
			// The receive fifo overflowed. A bad sign...
			// Abort the transfer
			rv = 0;	// Signal error
			LL_ERR("%s: Receive FIFO overflow during transfer\n", rp->spix.name);
			goto abort_rx;
		}
	}

	// We should no longer have to wait. Each word transmitted has been
	// received (rxlen == 0). Therefore, the transfer must be complete and
	// done. No (busy) wait required.

abort_rx:
	spi_reset(port);

	gpio_debug_pin(true);
	return rv;
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

	/* mmap PCIe translation address of the RP1 peripherals */
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

	// Calculate the addresses of the individual peripherals
	spi_ports[0].port =
	spi_ports[1].port = (dw_ssi_t *)((void *)((uintptr_t)peripheralmem + RP1_SPI0_OFFSET));
	spi_ports[2].port =
	spi_ports[3].port =
	spi_ports[4].port = (dw_ssi_t *)((void *)((uintptr_t)peripheralmem + RP1_SPI1_OFFSET));
	iobank0 = (rp1_io_bank0_t *)((void *)((uintptr_t)peripheralmem + RP1_IO_BANK0_OFFSET));
	padsbank0 = (rp1_pads_bank0_t *)((void *)((uintptr_t)peripheralmem + RP1_PADS_BANK0_OFFSET));
	rio0 = (rp1_rio_t *)((void *)((uintptr_t)peripheralmem + RP1_SYS_RIO0_OFFSET));

	LL_INFO("Mapped peripherals from 0x%p (size 0x%08zx)\n", (void *)membase, peripheralsize);

	return 0;
}

/*
 * Convert the pull-up/down enum into a register mask. Perform boundary checks
 * on the value and set it to a valid value if it was outside valid range.
 */
static uint32_t pud_val_to_mask(uint32_t pud)
{
	switch(pud) {
	default:
		/* Fallthrough */
	case SPIX_PULL_OFF:		return 0;
	case SPIX_PULL_DOWN:	return RP1_PADS_PDE;
	case SPIX_PULL_UP:		return RP1_PADS_PUE;
	}
}

// The required drive level in pads_bank0 registers (8mA drive and high slewrate)
#define DRIVE_LEVEL	(RP1_PADS_DRIVE(RP1_PADS_DRIVE_8) | RP1_PADS_SLEWFAST)

static void peripheral_setup(void)
{
	//
	// We do not touch the voltage_select register in pads_bank0. We simply
	// hope that the voltage_select register is at 3.3V. That should be a safe
	// assumption, considering the normal setup. Otherwise, the hardware may
	// already be fried at boot-time with high input voltages on input pins.
	// Raspberry Pi RP1 Peripherals bottom of section 3.1.3:
	//   "Using VDDIO voltages greater than 1.8V, with the input thresholds set
	//   for 1.8V may result in damage to the chip."
	// We are pretty sure that the stock board uses 3.3V VDDIO because of
	// compatibility with existing hardware. Therefore, fiddling with the
	// voltage_select register may be fatal to the hardware.
	//

	static const int spi_pull_miso[SPI_MAX_SPI] = { SPI_PULL_MISO_DEF, SPI_PULL_MISO_DEF };
	static const int spi_pull_mosi[SPI_MAX_SPI] = { SPI_PULL_MOSI_DEF, SPI_PULL_MOSI_DEF };
	static const int spi_pull_sclk[SPI_MAX_SPI] = { SPI_PULL_SCLK_DEF, SPI_PULL_SCLK_DEF };

	// Setup SPI pins for SPI0 and save the original setup
	if(port_probe_mask & SPIX_PROBE_SPI0_MASK) {
		uint32_t pud_miso = pud_val_to_mask(spi_pull_miso[0]);
		uint32_t pud_mosi = pud_val_to_mask(spi_pull_mosi[0]);
		uint32_t pud_sclk = pud_val_to_mask(spi_pull_sclk[0]);

		spi0save.bank_sclk = reg_rd(&iobank0->gpio[SPI0_PIN_SCLK].ctrl);
		spi0save.bank_mosi = reg_rd(&iobank0->gpio[SPI0_PIN_MOSI].ctrl);
		spi0save.bank_miso = reg_rd(&iobank0->gpio[SPI0_PIN_MISO].ctrl);
		spi0save.bank_ce_0 = reg_rd(&iobank0->gpio[SPI0_PIN_CE_0].ctrl);
		spi0save.bank_ce_1 = reg_rd(&iobank0->gpio[SPI0_PIN_CE_1].ctrl);
		spi0save.pads_sclk = reg_rd(&padsbank0->gpio[SPI0_PIN_SCLK]);
		spi0save.pads_mosi = reg_rd(&padsbank0->gpio[SPI0_PIN_MOSI]);
		spi0save.pads_miso = reg_rd(&padsbank0->gpio[SPI0_PIN_MISO]);
		spi0save.pads_ce_0 = reg_rd(&padsbank0->gpio[SPI0_PIN_CE_0]);
		spi0save.pads_ce_1 = reg_rd(&padsbank0->gpio[SPI0_PIN_CE_1]);

		reg_wr(&iobank0->gpio[SPI0_PIN_SCLK].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI0_PIN_SCLK], RP1_PADS_IE | DRIVE_LEVEL | pud_sclk);
		reg_wr(&iobank0->gpio[SPI0_PIN_MOSI].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI0_PIN_MOSI], RP1_PADS_IE | DRIVE_LEVEL | pud_mosi);
		reg_wr(&iobank0->gpio[ SPI0_PIN_MISO].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI0_PIN_MISO], RP1_PADS_IE | DRIVE_LEVEL | pud_miso);
		if(port_probe_mask & SPIX_PROBE_SPI0_CE0) {
			reg_wr(&iobank0->gpio[SPI0_PIN_CE_0].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
			reg_wr(&padsbank0->gpio[SPI0_PIN_CE_0], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
		}
		if(port_probe_mask & SPIX_PROBE_SPI0_CE1) {
			reg_wr(&iobank0->gpio[SPI0_PIN_CE_1].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
			reg_wr(&padsbank0->gpio[SPI0_PIN_CE_1], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
		}
		spi_reset(spi_ports[PORT_SPI0].port);
	}

	// Setup SPI pins for SPI1 and save the original setup
	if(port_probe_mask & SPIX_PROBE_SPI1_MASK) {
		uint32_t pud_miso = pud_val_to_mask(spi_pull_miso[1]);
		uint32_t pud_mosi = pud_val_to_mask(spi_pull_mosi[1]);
		uint32_t pud_sclk = pud_val_to_mask(spi_pull_sclk[1]);

		spi1save.bank_sclk = reg_rd(&iobank0->gpio[SPI1_PIN_SCLK].ctrl);
		spi1save.bank_mosi = reg_rd(&iobank0->gpio[SPI1_PIN_MOSI].ctrl);
		spi1save.bank_miso = reg_rd(&iobank0->gpio[SPI1_PIN_MISO].ctrl);
		spi1save.bank_ce_0 = reg_rd(&iobank0->gpio[SPI1_PIN_CE_0].ctrl);
		spi1save.bank_ce_1 = reg_rd(&iobank0->gpio[SPI1_PIN_CE_1].ctrl);
		spi1save.bank_ce_2 = reg_rd(&iobank0->gpio[SPI1_PIN_CE_2].ctrl);
		spi1save.pads_sclk = reg_rd(&padsbank0->gpio[SPI1_PIN_SCLK]);
		spi1save.pads_mosi = reg_rd(&padsbank0->gpio[SPI1_PIN_MOSI]);
		spi1save.pads_miso = reg_rd(&padsbank0->gpio[SPI1_PIN_MISO]);
		spi1save.pads_ce_0 = reg_rd(&padsbank0->gpio[SPI1_PIN_CE_0]);
		spi1save.pads_ce_1 = reg_rd(&padsbank0->gpio[SPI1_PIN_CE_1]);
		spi1save.pads_ce_2 = reg_rd(&padsbank0->gpio[SPI1_PIN_CE_2]);

		reg_wr(&iobank0->gpio[SPI1_PIN_SCLK].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI1_PIN_SCLK], RP1_PADS_IE | DRIVE_LEVEL | pud_sclk);
		reg_wr(&iobank0->gpio[SPI1_PIN_MOSI].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI1_PIN_MOSI], RP1_PADS_IE | DRIVE_LEVEL | pud_mosi);
		reg_wr(&iobank0->gpio[SPI1_PIN_MISO].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
		reg_wr(&padsbank0->gpio[SPI1_PIN_MISO], RP1_PADS_IE | DRIVE_LEVEL | pud_miso);
		if(port_probe_mask & SPIX_PROBE_SPI1_CE0) {
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_0].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_0], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
		}
		if(port_probe_mask & SPIX_PROBE_SPI1_CE1) {
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_1].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_1], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
		}
		if(port_probe_mask & SPIX_PROBE_SPI1_CE2) {
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_2].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_ALT0));
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_2], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
		}
		spi_reset(spi_ports[PORT_SPI1].port);
	}

#ifdef RPSPI_DEBUG_PIN
	reg_wr(&rio0->set.out, 1 << RPSPI_DEBUG_PIN);	// Set pin high
	reg_wr(&rio0->set.oe,  1 << RPSPI_DEBUG_PIN);	// Set pin to output
	reg_wr(&padsbank0->gpio[RPSPI_DEBUG_PIN], RP1_PADS_IE | DRIVE_LEVEL | RP1_PADS_PUE);
	reg_wr(&iobank0->gpio[RPSPI_DEBUG_PIN].ctrl, RP1_GPIO_CTRL_FUNCSEL(RP1_GPIO_CTRL_FUNCSEL_SYS_RIO));
#endif
}

/*************************************************/
static void peripheral_restore(void)
{
	// Restore all SPI pins to inputs and enable pull-up (no dangling inputs)
	if(port_probe_mask & SPIX_PROBE_SPI0_MASK) {
		spi_reset(spi_ports[PORT_SPI0].port);	// Disable SPI0 peripheral

		// Restore SPI0 pins
		reg_wr(&padsbank0->gpio[SPI0_PIN_SCLK], spi0save.pads_sclk);
		reg_wr(&padsbank0->gpio[SPI0_PIN_MOSI], spi0save.pads_mosi);
		reg_wr(&padsbank0->gpio[SPI0_PIN_MISO], spi0save.pads_miso);
		reg_wr(&iobank0->gpio[SPI0_PIN_SCLK].ctrl, spi0save.bank_sclk);
		reg_wr(&iobank0->gpio[SPI0_PIN_MOSI].ctrl, spi0save.bank_mosi);
		reg_wr(&iobank0->gpio[SPI0_PIN_MISO].ctrl, spi0save.bank_miso);
		if(port_probe_mask & SPIX_PROBE_SPI0_CE0) {
			reg_wr(&padsbank0->gpio[SPI0_PIN_CE_0], spi0save.pads_ce_0);
			reg_wr(&iobank0->gpio[SPI0_PIN_CE_0].ctrl, spi0save.bank_ce_0);
		}
		if(port_probe_mask & SPIX_PROBE_SPI0_CE1) {
			reg_wr(&padsbank0->gpio[SPI0_PIN_CE_1], spi0save.pads_ce_1);
			reg_wr(&iobank0->gpio[SPI0_PIN_CE_1].ctrl, spi0save.bank_ce_1);
		}
	}

	if(port_probe_mask & SPIX_PROBE_SPI1_MASK) {
		spi_reset(spi_ports[PORT_SPI1].port);	// Disable SPI1 peripheral

		// Restore SPI1 pins
		reg_wr(&padsbank0->gpio[SPI1_PIN_SCLK], spi1save.pads_sclk);
		reg_wr(&padsbank0->gpio[SPI1_PIN_MOSI], spi1save.pads_mosi);
		reg_wr(&padsbank0->gpio[SPI1_PIN_MISO], spi1save.pads_miso);
		reg_wr(&iobank0->gpio[SPI1_PIN_SCLK].ctrl, spi1save.bank_sclk);
		reg_wr(&iobank0->gpio[SPI1_PIN_MOSI].ctrl, spi1save.bank_mosi);
		reg_wr(&iobank0->gpio[SPI1_PIN_MISO].ctrl, spi1save.bank_miso);
		if(port_probe_mask & SPIX_PROBE_SPI1_CE0) {
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_0], spi1save.pads_ce_0);
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_0].ctrl, spi1save.bank_ce_0);
		}
		if(port_probe_mask & SPIX_PROBE_SPI1_CE1) {
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_1], spi1save.pads_ce_1);
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_1].ctrl, spi1save.bank_ce_1);
		}
		if(port_probe_mask & SPIX_PROBE_SPI1_CE2) {
			reg_wr(&padsbank0->gpio[SPI1_PIN_CE_2], spi1save.pads_ce_2);
			reg_wr(&iobank0->gpio[SPI1_PIN_CE_2].ctrl, spi1save.bank_ce_2);
		}
	}

#ifdef RPSPI_DEBUG_PIN
	// This leaves the pin in SYS_RIO mode so we may trigger the scope on the
	// next run
	reg_wr(&rio0->clr.oe,  1 << RPSPI_DEBUG_PIN);	// Set pin to input
#endif
}

/*************************************************/

/*
 * Detect the presence of the hardware on basis of the device-tree compatible
 * string-list.
 * On success returns 9 (zero) and sets the driver information to the dtc
 * string and tries to set a human readable string as model.
 */
static int rpi5_detect(const char *dtcs[])
{
	int i;
	for(i = 0; dtcs[i] != NULL; i++) {
		if(!strcmp(dtcs[i], DTC_RPI_MODEL_5B) || !strcmp(dtcs[i], DTC_RPI_MODEL_5CM)) {
			break;	// Found our supported board
		}
	}

	if(dtcs[i] == NULL)
		return -ENODEV;	// We are not the device the driver supports

	// Set the matched dtc and model informational strings
	strncpy(spix_driver_rpi5.dtc, dtcs[i], sizeof(spix_driver_rpi5.dtc)-1);
	if(spix_read_file("/proc/device-tree/model", spix_driver_rpi5.model, sizeof(spix_driver_rpi5.model)) < 0)
		strncpy(spix_driver_rpi5.model, "??? Unknown board ???", sizeof(spix_driver_rpi5.model)-1);

	return 0;
}

/*
 * Setup the driver.
 * - remove kernel spidev driver modules if detected
 * - map the I/O memory
 * - setup the GPIO pins and SPI peripheral(s)
 */
static int rpi5_setup(int probemask)
{
	int retval = -1;

	if(driver_enabled) {
		LL_ERR("Driver is already setup.\n");
		return -EBUSY;
	}

	port_probe_mask = probemask;	// For peripheral_setup() and peripheral_restore()

	// Now we know what platform we are running, remove kernel SPI module if
	// detected
	if((has_spi_module = (0 == shell("/usr/bin/grep -qw ^dw_spi_mmio /proc/modules")))) {
		if(shell("/sbin/rmmod dw_spi_mmio dw_spi"))
			LL_ERR("Unable to remove kernel SPI modules dw_spi_mmio and dw_spi. "
					"Your system may become unstable using LinuxCNC with the " HM2_LLIO_NAME " driver.\n");
	}

	// The IO address for the RPi5 is at a fixed address. No need to do fancy
	// stuff.
	if((retval = peripheral_map(RP1_PCIE_BAR1_ADDR, RP1_PCIE_BAR1_LEN)) < 0) {
		LL_ERR("Cannot map peripheral memory.\n");
		return retval;
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
static int rpi5_cleanup(void)
{
	int i;

	if(!driver_enabled) {
		LL_ERR("Driver is not setup.\n");
		return -ENODEV;
	}

	// Close any ports not closed already
	for(i = 0; i < PORT_MAX; i++) {
		if(spi_ports[i].isopen)
			spix_driver_rpi5.close(&spi_ports[i].spix);
	}

	if(peripheralmem != MAP_FAILED) {
		peripheral_restore();
		munmap(peripheralmem, peripheralsize);
	}

	// Restore kernel SPI module if it was detected before
	if(has_spi_module)
		shell("/sbin/modprobe dw_spi_mmio");

	driver_enabled = 0;
	return 0;
}

/*
 * Open a SPI port at index 'port' with 'clkw' write clock and 'clkr' read
 * clock frequencies.
 */
static const spix_port_t *rpi5_open(int port, const spix_args_t *args)
{
	rpi5_port_t *rpp;

	if(!driver_enabled) {
		LL_ERR("Driver is not setup.\n");
		return NULL;
	}

	if(port < 0 || port >= PORT_MAX) {
		LL_ERR("open(): SPI port %d out of range.\n", port);
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

	if(args->clkw < SCLK_FREQ_MIN || args->clkw > SCLK_FREQ_MAX) {
		LL_ERR("%s: SPI write clock frequency outside acceptable range (%d..%d kHz).\n", rpp->spix.name, SCLK_FREQ_MIN, SCLK_FREQ_MAX);
		return NULL;
	}

	rpp->isopen = 1;
	rpp->clkdivw = clkdiv_calc(args->clkw);
	rpp->clkdivr = clkdiv_calc(args->clkr);
	LL_INFO("%s: write clock rate calculated: %u Hz (clkdiv: %u)\n", rpp->spix.name, RP1_SPI_CLK / rpp->clkdivw, rpp->clkdivw);
	LL_INFO("%s: read clock rate calculated: %u Hz (clkdiv: %u)\n",  rpp->spix.name, RP1_SPI_CLK / rpp->clkdivr, rpp->clkdivr);

	return &rpp->spix;
}

/*
 * Close a SPI port.
 */
static int rpi5_close(const spix_port_t *sp)
{
	rpi5_port_t *rpp;

	if(!driver_enabled) {
		LL_ERR("Driver is not setup.\n");
		return -ENODEV;
	}

	if(!sp) {
		LL_ERR("close(): trying to close port NULL\n");
		return -EINVAL;
	}

	rpp = (rpi5_port_t *)sp;
	if(!rpp->isopen) {
		LL_ERR("%s: close(): SPI port not open.\n", rpp->spix.name);
		return -ENODEV;
	}

	spi_reset(rpp->port);	// make sure it is disabled
	rpp->isopen = 0;
	return 0;
}

// vim: ts=4
