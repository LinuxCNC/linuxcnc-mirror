/*    This is a component for RaspberryPi to hostmot2 over SPI for linuxcnc.
 *    Copyright 2016 Matsche <tinker@play-pla.net>
 *    Copyright 2017 B.Stultiens <lcnc@vagrearg.org>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/* Without Source Tree */
#undef WOST

#include <ctype.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include <hal.h>
#include <rtapi.h>
#include <rtapi_app.h>
#include <rtapi_slab.h>

#include "hostmot2-lowlevel.h"
#include "hostmot2.h"
#include "spi_common_rpspi.h"
#include "eshellf.h"
#include "llio_info.h"

#define HM2_LLIO_NAME "hm2_rpspi"

/*
 * Debugging options
 */
//#define RPSPI_DEBUG		1	// Comment to disable all debugging

#if defined(RPSPI_DEBUG)
#define RPSPI_DEBUG_MSG		1	// Redirect INFO to ERR channel
//#define RPSPI_DEBUG_WRITE	1	// Debug write command before cookie probe
//#define RPSPI_DEBUG_PIN	23	// Debug timing on GPIO 23
#endif

// Redirect message class for easy and noisy debugging purpose
#if defined(RPSPI_DEBUG_MSG)
#define RPSPI_ERR	RTAPI_MSG_ERR
#define RPSPI_WARN	RTAPI_MSG_ERR
#define RPSPI_INFO	RTAPI_MSG_ERR
#else
#define RPSPI_ERR	RTAPI_MSG_ERR
#define RPSPI_WARN	RTAPI_MSG_WARN
#define RPSPI_INFO	RTAPI_MSG_INFO
#endif

// Forced inline expansion
#define RPSPI_ALWAYS_INLINE	__attribute__((always_inline))


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matsche");
MODULE_DESCRIPTION("Driver for HostMot2 devices connected via SPI to RaspberryPi");
MODULE_SUPPORTED_DEVICE("Mesa-AnythingIO-7i90");

#define RPSPI_MAX_BOARDS	5
#define RPSPI_MAX_MSG		(127+1)		// The 7i90 docs say that the max. burstlen == 127 words (i.e. cmd+message <= 1+127)

// GPIO pin definitions
#define SPI0_PIN_CE_1		7
#define SPI0_PIN_CE_0		8
#define SPI0_PIN_MISO		9
#define SPI0_PIN_MOSI		10
#define SPI0_PIN_SCLK		11
#define SPI1_PIN_CE_2		16		// AUX SPI0 in docs
#define SPI1_PIN_CE_1		17
#define SPI1_PIN_CE_0		18
#define SPI1_PIN_MISO		19
#define SPI1_PIN_MOSI		20
#define SPI1_PIN_SCLK		21

typedef struct hm2_rpspi_struct {
	hm2_lowlevel_io_t llio;		// Upstream container
	int		nr;		// Board number
	uint32_t	spiclkratew;	// SPI write clock for divider calculation
	uint32_t	spiclkrater;	// SPI read clock for divider calculation
	uint32_t	spiclkbase;	// SPI base clock for divider calculation
	uint32_t	spice;		// Chip enable mask for this board
	int		spidevid;	// The SPI device id [01]
	int		spiceid;	// The SPI CE id [012]
} hm2_rpspi_t;

static uint32_t *peripheralmem = (uint32_t *)MAP_FAILED;	// mmap'ed peripheral memory
static size_t peripheralsize;					// Size of the mmap'ed block
static bcm2835_gpio_t *gpio;					// GPIO peripheral structure in mmap'ed address space
static bcm2835_spi_t *spi;					// SPI peripheral structure in mmap'ed address space
static bcm2835_aux_t *aux;					// AUX peripheral structure in mmap'ed address space
static uint32_t aux_enables;					// Previous state of SPI1 enable

static hm2_rpspi_t boards[RPSPI_MAX_BOARDS];	// Connected boards
static int comp_id;				// Upstream assigned component ID

/*
 * Configuration parameters
 */
static char *config[RPSPI_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(config, RPSPI_MAX_BOARDS, "config string for the AnyIO boards (see hostmot2(9) manpage)")

/*
 * RPI3 NOTE:
 * The core frequency is wildly variable when the ondemand cpufreq governor is
 * active. This may result in changing SPI frequencies depending on the system
 * load. You must set the performance governor for stable frequency. To set a
 * stable 1.2GHz core frequency, across all cores, put something like this in
 * /etc/rc.local:
 *   echo -n 1200000 > /sys/devices/system/cpu/cpufreq/policy0/scaling_min_freq
 *   echo -n performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
 *
 * We've seen both 400MHz and 250MHz base rates. It is a bit unclear how this
 * gets switched, and /what/ is responsible for switching. The kernel is doing
 * the actual work, but under which conditions is unknown at this moment.
 * A suggestion may be power supply issues, where the controlling PLL is thrown
 * off a bit if there is too much noise on the 5V supply. This can be countered
 * by adding a low-ESR decoupling to 5V pins on 40-pin I/O header.
 *
 * The documentation is a bit sparse, but the clock seems to follow the VPU
 * rate as a subordinate of PLLC. The peripheral documentation states that the
 * APB clock is used (for SPI0), but that is set at 126MHz, which does not
 * result in the seen frequencies at different divider settings. Both SPI0 and
 * SPI1 are apparently referenced to the core clock.
 *
 * Using 400MHz as the "safe" value is better than 250MHz. Better to get a
 * slower SPI rate than one too fast, which would result in communication
 * problems all around. If we assume a base-clock of 400MHz, derive the
 * divider from there, and the clock flips between 400MHz and 250MHz, then we
 * still should be ok. The current setup works just fine by getting the value
 * at program start. It simply means that we sometimes are at only 62.5% of the
 * speed we actually want.
 *
 * We should get the /actual/ peripheral clock setting before calculating the
 * divider for SPI transfers. However, how do we get the peripheral clock from
 * userland in an effective way? Reading a file, as is done now, is just too
 * darn slow.
 *
 * See also read_spiclkbase() below.
 */
static int spiclk_rate = 31250;
static int spiclk_rate_rd = -1;
RTAPI_MP_INT(spiclk_rate, "SPI clock rate in kHz (default 31250 kHz, slowest 3 kHz)")
RTAPI_MP_INT(spiclk_rate_rd, "SPI clock rate for reading in kHz (default same as spiclk_rate)")

/*
 * Override the "safe" base frequency of the SPI peripheral. The clock speed
 * is normally read from /sys/kernel/debug/clk/vpu/clk_rate or, for older
 * kernels, /sys/kernel/debug/clk/core/clk_rate and should give the correct
 * current base clock speed.
 */
#define F_PERI	400000000UL
static int spiclk_base = F_PERI;
RTAPI_MP_INT(spiclk_base, "SPI clock base rate in Hz (default 400000000 Hz)")

/*
 * Enable/disable pullup/pulldown on the SPI pins
 */
#define SPI_PULL_OFF	0	// == GPIO_GPPUD_OFF
#define SPI_PULL_DOWN	1	// == GPIO_GPPUD_PULLDOWN
#define SPI_PULL_UP	2	// == GPIO_GPPUD_PULLUP
static int spi_pull_miso = SPI_PULL_DOWN;
static int spi_pull_mosi = SPI_PULL_DOWN;
static int spi_pull_sclk = SPI_PULL_DOWN;
RTAPI_MP_INT(spi_pull_miso, "Enable/disable pull-{up,down} on SPI MISO (default pulldown, 0=off, 1=pulldown, 2=pullup)")
RTAPI_MP_INT(spi_pull_mosi, "Enable/disable pull-{up,down} on SPI MOSI (default pulldown, 0=off, 1=pulldown, 2=pullup)")
RTAPI_MP_INT(spi_pull_sclk, "Enable/disable pull-{up,down} on SPI SCLK (default pulldown, 0=off, 1=pulldown, 2=pullup)")

/*
 * Select which SPI channel(s) to probe. There are two SPI interfaces exposed
 * on the 40-pin I/O header, SPI0 and SPI1. SPI0 has two chip selects and SPI1
 * has three chip selects.
 *
 * GPIO pin setup:
 *      | MOSI | MISO | SCLK | CE0 | CE1 | CE2
 * -----+------+------+------+-----+-----+-----
 * SPI0 |  10  |   9  |  11  |   8 |   7 |
 * SPI1 |  20  |  19  |  21  |  18 |  17 |  16
 *
 * Boards will be numbered in the order found. The probe scan is ordered in the
 * following way:
 * - SPI0 - CE0
 * - SPI0 - CE1
 * - SPI1 - CE0
 * - SPI1 - CE1
 * - SPI1 - CE2
 */
#define SPI0_PROBE_CE0	(1 << 0)
#define SPI0_PROBE_CE1	(1 << 1)
#define SPI0_PROBE_MASK	(SPI0_PROBE_CE0 | SPI0_PROBE_CE1)
#define SPI1_PROBE_CE0	(1 << 2)
#define SPI1_PROBE_CE1	(1 << 3)
#define SPI1_PROBE_CE2	(1 << 4)
#define SPI1_PROBE_MASK	(SPI1_PROBE_CE0 | SPI1_PROBE_CE1 | SPI1_PROBE_CE2)
static int spi_probe = SPI0_PROBE_CE0;
RTAPI_MP_INT(spi_probe, "Bit-field to select which SPI/CE combinations to probe (default 1 (SPI0/CE0))")

/*
 * Set the message level for debugging purpose. This has the (side-)effect that
 * all modules within this process will start spitting out messages at the
 * requested level.
 * The upstream message level is not touched if spi_debug == -1.
 */
static int spi_debug = -1;
RTAPI_MP_INT(spi_debug, "Set message level for debugging purpose [0...5] where 0=none and 5=all (default: -1; upstream defined)")

/*********************************************************************/
#if defined(RPSPI_DEBUG_PIN)
/*
 * Set/Clear a GPIO pin
 */
RPSPI_ALWAYS_INLINE static inline void gpio_set(uint32_t pin)
{
	if(pin <= 53) {	/* There are 54 GPIOs */
		reg_wr(&gpio->gpset[pin / 32], 1 << (pin % 32));
	}
}

RPSPI_ALWAYS_INLINE static inline void gpio_clr(uint32_t pin)
{
	if(pin <= 53) {	/* There are 54 GPIOs */
		reg_wr(&gpio->gpclr[pin / 32], 1 << (pin % 32));
	}
}

RPSPI_ALWAYS_INLINE static inline void gpio_debug_pin(bool set_reset)
{
	if(set_reset)
		gpio_set(RPSPI_DEBUG_PIN);
	else
		gpio_clr(RPSPI_DEBUG_PIN);
}

#else

RPSPI_ALWAYS_INLINE static inline void gpio_debug_pin(bool set_reset)
{
	(void)set_reset;
}

#endif

/*********************************************************************/
#define CMD_7I90_READ		(0x0a << 12)
#define CMD_7I90_WRITE		(0x0b << 12)
#define CMD_7I90_ADDRINC	(1 << 11)
// aib (address increment bit)
static inline uint32_t mk_read_cmd(uint32_t addr, uint32_t msglen, bool aib)
{
	return (addr << 16) | CMD_7I90_READ | (aib ? CMD_7I90_ADDRINC : 0) | (msglen << 4);
}

static inline uint32_t mk_write_cmd(uint32_t addr, uint32_t msglen, bool aib)
{
	return (addr << 16) | CMD_7I90_WRITE | (aib ? CMD_7I90_ADDRINC : 0) | (msglen << 4);
}

/*********************************************************************/
static inline int32_t spi1_clkdiv_calc(uint32_t base, uint32_t rate)
{
	uint32_t clkdiv;
	if(rate >= base / 2)
		return 0;
	clkdiv = (base) / ((rate-1500) * 2) - 1;	// -1500 for same scaling as SPI0
	if(clkdiv > 4095)
		clkdiv = 4095;		// Slowest possible
	return clkdiv;
}

/*
 * Initialize/reset the SPI peripheral to inactive state and flushed
 */
static inline void spi1_reset(void)
{
	// Disable and clear fifos
	reg_wr(&aux->spi0_cntl0, 0);
	reg_wr(&aux->spi0_cntl1, AUX_SPI_CNTL0_CLEARFIFO);
}

/*
 * Setup SPI1 transfer start
 */
static inline void spi1_xfer_setup(hm2_rpspi_t *hm2, bool rd)
{
	// Set clock speed and format
	// Note: It seems that we cannot send 32 bits. Shift length 0 sends
	// zero bits and there are only 6 bits available to set the length,
	// giving a maximum of 31 bits. Furthermore, the shift seems to be
	// delayed one clock. Therefore we need to shift 15 places (below) and
	// not 16 as expected. The one-clock delay may be related to the
	// OUT_RISING flag, but that is speculation.
	reg_wr(&aux->spi0_cntl0,  AUX_SPI_CNTL0_SPEED(spi1_clkdiv_calc(hm2->spiclkbase, rd ? hm2->spiclkrater : hm2->spiclkratew))
				| AUX_SPI_CNTL0_ENABLE
				| AUX_SPI_CNTL0_MSB_OUT
				| AUX_SPI_CNTL0_OUT_RISING
				| AUX_SPI_CNTL0_SHIFT_LENGTH(16)
				| hm2->spice);
	reg_wr(&aux->spi0_cntl1, AUX_SPI_CNTL1_MSB_IN);
}

/*
 * HM2 interface: Write buffer to SPI1
 */
static int hm2_rpspi_write_spi1(hm2_lowlevel_io_t *llio, uint32_t addr, const void *buffer, int size)
{
	hm2_rpspi_t *hm2 = (hm2_rpspi_t *)llio;
	uint32_t cmd;
	int txlen;
	int rxlen;
	uint16_t *bptr;
	uint32_t *wptr;

	if(size == 0)
		return 0;
	if((size % 4) || size / 4 >= RPSPI_MAX_MSG)
		return -EINVAL;

	gpio_debug_pin(false);

	cmd = htobe32(mk_write_cmd(addr, size / 4, true));	// Setup write command
	spi1_xfer_setup(hm2, false);				// Setup transfer

	// Send the command. The fifo is big enough to hold the data
	txlen = 2;			// Command is two 16bit words
	bptr = (uint16_t *)&cmd;	// The command
	while(txlen) {
		uint32_t stat = reg_rd(&aux->spi0_stat);
		if(txlen && !(stat & AUX_SPI_STAT_TX_FULL)) {
			reg_wr(&aux->spi0_hold, (uint32_t)htobe16(*bptr++) << 15);	// More data to follow
			--txlen;
		}
	}

	rxlen = size / 2 + 2;		// We read back anything to be discarded
	txlen = size / 2;		// Number of data words to write
	wptr = (uint32_t *)buffer;	// And get it from here
	while(rxlen) {
		uint32_t stat = reg_rd(&aux->spi0_stat);
		if(txlen && !(stat & AUX_SPI_STAT_TX_FULL)) {
			// For each 32 bit read the buffer and convert
			if(!(txlen & 1)) {
				cmd = htobe32(*wptr++);
				bptr = (uint16_t *)&cmd;
			}
			if(--txlen)
				reg_wr(&aux->spi0_hold, (uint32_t)htobe16(*bptr++) << 15);	// Write while still more data to follow
			else
				reg_wr(&aux->spi0_io, (uint32_t)htobe16(*bptr++) << 15);	// Final write
		}

		if(!(stat & AUX_SPI_STAT_RX_EMPTY)) {
			// Discard the read data
			uint32_t dummy __attribute__((unused));
			dummy = reg_rd(&aux->spi0_io);
			rxlen--;
		}
	}

	// Stop transfer
	spi1_reset();

	gpio_debug_pin(true);
	return 1;
}

/*
 * HM2 interface: Read buffer from SPI1
 */
static int hm2_rpspi_read_spi1(hm2_lowlevel_io_t *llio, uint32_t addr, void *buffer, int size)
{
	hm2_rpspi_t *hm2 = (hm2_rpspi_t *)llio;
	uint32_t cmd;
	int txlen;
	int rxlen;
	uint16_t *bptr;
	uint32_t *wptr;

	if(size == 0)
		return 0;
	if((size % 4) || size / 4 >= RPSPI_MAX_MSG)
		return -EINVAL;

	gpio_debug_pin(false);

	cmd = htobe32(mk_read_cmd(addr, size / 4, true));	// Setup read command
	spi1_xfer_setup(hm2, true);				// Setup transfer

	// Send the command. The fifo is big enough to hold the data
	txlen = 2;			// Command is two 16bit words
	bptr = (uint16_t *)&cmd;	// The command
	while(txlen) {
		uint32_t stat = reg_rd(&aux->spi0_stat);
		if(txlen && !(stat & AUX_SPI_STAT_TX_FULL)) {
			reg_wr(&aux->spi0_hold, (uint32_t)be16toh(*bptr++) << 15);	// More data to follow
			--txlen;
		}
	}

	rxlen = 2;		// We read back the command to be discarded
	txlen = size / 2;	// Number of zeroes to write for the read command
	while(rxlen) {
		uint32_t stat = reg_rd(&aux->spi0_stat);
		if(txlen && !(stat & AUX_SPI_STAT_TX_FULL)) {
			if(--txlen)
				reg_wr(&aux->spi0_hold, 0);	// Write while still more data to follow
			else
				reg_wr(&aux->spi0_io, 0);	// Final write (maybe reached on very short reads)
		}

		if(!(stat & AUX_SPI_STAT_RX_EMPTY)) {
			uint32_t dummy __attribute__((unused));
			dummy = reg_rd(&aux->spi0_io);
			rxlen--;
		}
	}

	rxlen = size / 2;		// We read the data
	wptr = (uint32_t *)buffer;	// And save it here
	bptr = (uint16_t *)&cmd;	// Temporary store for byteswap
	while(rxlen) {
		uint32_t stat = reg_rd(&aux->spi0_stat);
		if(txlen && !(stat & AUX_SPI_STAT_TX_FULL)) {
			if(--txlen)
				reg_wr(&aux->spi0_hold, 0);	// Write while still more data to follow
			else
				reg_wr(&aux->spi0_io, 0);	// Final write
		}

		if(!(stat & AUX_SPI_STAT_RX_EMPTY)) {
			*bptr++ = be16toh(reg_rd(&aux->spi0_io));
			rxlen--;
			// Read 32 bits, convert and store
			if(!(rxlen & 1)) {
				*wptr++ = be32toh(cmd);
				bptr = (uint16_t *)&cmd;
			}
		}
	}

	// Stop transfer
	spi1_reset();

	gpio_debug_pin(true);
	return 1;
}

/*********************************************************************/
static inline int32_t spi0_clkdiv_calc(uint32_t base, uint32_t rate)
{
	uint32_t clkdiv = base / rate;
	// Use only even divider values
	// This is what the documentation (probably) states
	if(clkdiv > 65534)
		clkdiv = 0;		// Slowest possible
	else
		clkdiv += clkdiv & 1;	// Must be multiple of 2 (round to lower frequency)
	return clkdiv;
}

/*
 * Initialize/reset the SPI peripheral to inactive state and flushed
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
 * Setup SPI0 transfer start
 */
static inline void spi0_xfer_setup(hm2_rpspi_t *hm2, bool rd)
{
	uint32_t cs = reg_rd(&spi->cs);
	cs &= ~(SPI_CS_CS_10 | SPI_CS_CS_01 | SPI_CS_REN);	// Reset CE and disable 3-wire mode
	cs |= hm2->spice | SPI_CS_TA;				// Set proper CE_x and activate transfer
	reg_wr(&spi->clk, spi0_clkdiv_calc(hm2->spiclkbase, rd ? hm2->spiclkrater : hm2->spiclkratew));	// Set clock divider
	reg_wr(&spi->cs, cs);					// Go!
}

/*
 * HM2 interface: Write buffer to SPI0
 */
static int hm2_rpspi_write_spi0(hm2_lowlevel_io_t *llio, uint32_t addr, const void *buffer, int size)
{
	hm2_rpspi_t *hm2 = (hm2_rpspi_t *)llio;
	uint32_t cs;
	uint32_t cmd;
	int txlen;
	int rxlen;
	uint8_t *bptr;
	uint32_t *wptr;

	if(size == 0)
		return 0;
	if((size % 4) || size / 4 >= RPSPI_MAX_MSG)
		return -EINVAL;

	gpio_debug_pin(false);

	cmd = htobe32(mk_write_cmd(addr, size / 4, true));	// Setup write command
	spi0_xfer_setup(hm2, false);

	// Fill the TX fifo with the command
	// The fifo is large enough to hold the command and the returning RX
	// data while we write the command.
	bptr = (uint8_t *)&cmd;	// Command data
	txlen = 4;		// Command length
	while(txlen && (reg_rd(&spi->cs) & SPI_CS_TXD)) {
		reg_wr(&spi->fifo, *bptr++);
		--txlen;
	}

	// Read and write until all done
	// The read data is discarded. When the txlen reaches zero, then we
	// must keep reading until the rxlen also reaches zero to ensure that
	// all data has been written on the SPI bus.
	txlen = size;			// Buffer size to transmit
	rxlen = size + 4;		// Buffer size plus command length
	wptr = (uint32_t *)buffer;	// Actual write data
	while(rxlen) {
		cs = reg_rd(&spi->cs);
		if(cs & SPI_CS_RXD) {
			uint32_t dummy __attribute__((unused));
			dummy = reg_rd(&spi->fifo);
			--rxlen;
		}
		if(txlen && (cs & SPI_CS_TXD)) {
			// For each 32 bit do conversion to big endian
			if(!(txlen & 3)) {
				cmd = htobe32(*wptr++);
				bptr = (uint8_t *)&cmd;
			}
			reg_wr(&spi->fifo, *bptr++);
			--txlen;
		}
	}

	// We're done, there is no return data

	// Stop transfer
	spi0_reset();

	gpio_debug_pin(true);
	return 1;
}

/*
 * HM2 interface: Read buffer from SPI0
 */
static int hm2_rpspi_read_spi0(hm2_lowlevel_io_t *llio, uint32_t addr, void *buffer, int size)
{
	hm2_rpspi_t *hm2 = (hm2_rpspi_t *)llio;
	uint32_t cs;
	uint32_t cmd;
	int txlen;
	int rxlen;
	uint8_t *bptr;
	uint32_t *wptr;

	if(size == 0)
		return 0;
	if((size % 4) || size / 4 >= RPSPI_MAX_MSG)
		return -EINVAL;

	gpio_debug_pin(false);

	cmd = htobe32(mk_read_cmd(addr, size / 4, true));	// Setup read command
	spi0_xfer_setup(hm2, true);

	// Fill the TX fifo with the command
	// The fifo is large enough to hold the command and the returning RX
	// data while we write the command.
	bptr = (uint8_t *)&cmd;	// Command data
	txlen = 4;		// Command length
	while(txlen && (reg_rd(&spi->cs) & SPI_CS_TXD)) {
		reg_wr(&spi->fifo, *bptr++);
		--txlen;
	}

	// Read the command return and discard it. Simultaneously, we write
	// zeroes for as long as the read data goes.
	txlen = size;	// Now we write zeros for the read length
	rxlen = 4;	// The command read back needs to be discarded
	while(rxlen) {
		cs = reg_rd(&spi->cs);
		if(cs & SPI_CS_RXD) {
			uint32_t dummy __attribute__((unused));
			dummy = reg_rd(&spi->fifo);
			--rxlen;
		}
		if(txlen && (cs & SPI_CS_TXD)) {
			reg_wr(&spi->fifo, 0);	// Write zeroes
			--txlen;
		}
	}

	// The command (4 bytes) has been discarded. Read the rest of the data
	// into the return buffer and keep writing zeroes for as long as the
	// read requires.
	rxlen = size;	// Now we read the actual data
	wptr = (uint32_t *)buffer;
	bptr = (uint8_t *)&cmd;
	while(rxlen) {
		cs = reg_rd(&spi->cs);
		if(cs & SPI_CS_RXD) {
			*bptr++ = reg_rd(&spi->fifo);
			--rxlen;
			// Once we have 4 bytes, convert to little endian and save
			if(!(rxlen & 3)) {
				*wptr++ = be32toh(cmd);
				bptr = (uint8_t *)&cmd;
			}
		}
		if(txlen && (cs & SPI_CS_TXD)) {
			reg_wr(&spi->fifo, 0);	// Keep writing zeros
			--txlen;
		}
	}

	// The return data is now in the buffer

	// Stop transfer
	spi0_reset();

	gpio_debug_pin(true);
	return 1;
}

/*************************************************/
// Counting ones in a word is the also known as the Hamming weight or
// population count. The "best" algorithm is SWAR. The nibble-lookup seems to
// be a good alternative. Anyway, this routine is only called on a cookie
// error and has no real speed criteria.
// Newer GCC has a __builtin_popcount() to get the right number, but that may
// not be available on the current compiler.
static inline unsigned count_ones(uint32_t val)
{
	// Number of ones in a nibble
	static const unsigned nibble_table[16] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4 };
	unsigned i;
	for(i = 0; val; val >>= 4) {
		i += nibble_table[val & 0x0f];
	}
	return i;
}

static int32_t check_cookie(hm2_rpspi_t *board)
{
	// The secondary and tertiary cookie values are "HOSTMOT2", but we use
	// binary here to prevent bytesex problems and multibyte character
	// formats. Alternatively, we could use a byte-array, but then the
	// IOCOOKIE values has to be split. Doomed if you do, doomed if you
	// don't...
	static const uint32_t xcookie[3] = {HM2_IOCOOKIE, 0x54534f48, 0x32544f4d};
	uint32_t cookie[4];
	uint32_t ca;
	uint32_t co;

	// We read four (4) 32-bit words. The first three are the cookie and
	// the fourth entry is the idrom address offset. The offset is used in
	// the call to get the board ident if we successfully match a cookie.
	if(!board->llio.read(&board->llio, HM2_ADDR_IOCOOKIE, cookie, 16))
		return -ENODEV;

	if(!memcmp(cookie, xcookie, sizeof(xcookie)))
		return (int32_t)cookie[3];	// The cookie got read correctly

	rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: SPI%d/CE%d Invalid cookie, read: %08x %08x %08x,"
			" expected: %08x %08x %08x\n",
			board->spidevid, board->spiceid,
			cookie[0], cookie[1], cookie[2],
			xcookie[0], xcookie[1], xcookie[2]);

	// Lets see if we can tell why it went wrong
	ca = cookie[0] & cookie[1] & cookie[2];	// All ones -> ca == ones
	co = cookie[0] | cookie[1] | cookie[2];	// All zero -> co == zero

	if((!co && spi_pull_miso == SPI_PULL_DOWN) || (ca == 0xffffffff && spi_pull_miso == SPI_PULL_UP)) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: SPI%d/CE%d No drive seen on MISO line (kept at pull-%s level)."
			" No board connected or bad connection?\n",
			board->spidevid, board->spiceid, spi_pull_miso == SPI_PULL_DOWN ? "down" : "up");
	} else if(!co || ca == 0xffffffff) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: SPI%d/CE%d MISO line stuck at %s level."
			" Maybe bad connection, a short-circuit or no board attached?\n",
			board->spidevid, board->spiceid, !co ? "low" : "high");
	} else {
		// If you read too fast, then the bit-setup times are not
		// satisfied and the input may be shifted by one bit-clock.
		// Every 8th or 16th bit will most likely fall off the cliff
		// because there is a small pause in the spi-controller's
		// output (every 8-bit for SPI0 and every 16-bit for SPI1).
		//
		// We can detect this eventuality by checking the cookie
		// against a bit-shifted version and mask the bits that may
		// have fallen off the cliff. If the cookie matches (all zeroes
		// in the XOR result), then it is most likely that the
		// read-clock frequency is too high.
		//
		// Example (SPI0):
		//  read  : 2a d5 e5 ff
		//  shift : 55 ab cb fe = _0101010 1 1010101 1 1100101 1 1111111 (0)
		//  cookie: 55 aa ca fe =  0101010 1 1010101 0 1100101 0 1111111  0
		//
		//  read  : 2a 29 27 24
		//  shift : 54 52 4e 48 = _0101010 0 0101001 0 0100111 0 0100100 (0)
		//  cookie: 54 53 4f 48 =  0101010 0 0101001 1 0100111 1 0100100  0
		//
		//  read  : 19 2a 27 26
		//  read  : 32 54 4e 4c = _0011001 0 0101010 0 0100111 0 0100110 (0)
		//  cookie: 32 54 4f 4d =  0011001 0 0101010 0 0100111 1 0100110  1
		//
		// The '_' bit falls out of the read bit-string because of the
		// << operator and a (0) is added to the LSB. The "lone" bits
		// may have fallen off the cliff and are ignored by the mask.
		//
		uint32_t mask = board->spidevid ? ~0x00010001 : ~0x01010101;
		unsigned ones;
		unsigned i;
		for(ones = i = 0; i < 3; i++) {
			ones += count_ones((xcookie[i] ^ (cookie[i] << 1)) & mask);
		}
		if(!ones) {
			// No ones in the XOR result -> the cookie is probably bit-shifted
			rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: SPI%d/CE%d MISO input is bit-shifted by one bit."
				" SPI read clock frequency probably too high.\n",
				board->spidevid, board->spiceid);
		} else {
			// More bits are wrong, erratic behaviour
			rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: SPI%d/CE%d MISO input does not match any expected bit-pattern (>= %u bit difference)."
				" Maybe SPI read clock frequency too high or noise on the input?\n",
				board->spidevid, board->spiceid, ones);
		}
	}
	return -ENODEV;
}

/*************************************************/
#define RPSPI_SYS_CLKVPU	"/sys/kernel/debug/clk/vpu/clk_rate"	// Newer kernels (4.8+, I think) have detailed info
#define RPSPI_SYS_CLKCORE	"/sys/kernel/debug/clk/core/clk_rate"	// Older kernels only have core-clock info

static uint32_t read_spiclkbase(void)
{
	const char *sysclkref = RPSPI_SYS_CLKVPU;
	uint32_t rate;
	int fd;
	char buf[16];
	ssize_t err;

	if((fd = rtapi_open_as_root(sysclkref, O_RDONLY)) < 0) {
		// Failed VPU clock, try core clock
		rtapi_print_msg(RPSPI_INFO, "hm2_rpspi: No VPU clock at '%s' (errno=%d), trying core clock as alternative.\n", sysclkref, errno);
		sysclkref = RPSPI_SYS_CLKCORE;
		if((fd = rtapi_open_as_root(sysclkref, O_RDONLY)) < 0) {
			// Neither clock available, complain and use default setting
			rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Cannot open clock setting '%s' (errno=%d), using %d Hz\n", sysclkref, errno, spiclk_base);
			return spiclk_base;
		}
	}

	memset(buf, 0, sizeof(buf));
	if((err = read(fd, buf, sizeof(buf)-1)) < 0) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Cannot read clock setting '%s' (errno=%d), using %d Hz\n", sysclkref, errno, spiclk_base);
		close(fd);
		return spiclk_base;
	}

	close(fd);

	if(err >= (int)sizeof(buf)-1) {
		// There are probably too many digits in the number
		// 250000000 (250 MHz) has 9 digits and there is a newline
		// following the number
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Read full buffer '%s' from '%s', number probably wrong or too large, using %d Hz\n", buf, sysclkref, spiclk_base);
		return spiclk_base;
	}

	if(1 != sscanf(buf, "%u", &rate)) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Cannot interpret clock setting '%s' from '%s', using %d Hz\n", buf, sysclkref, spiclk_base);
		return spiclk_base;
	}
	return rate;
}

/*************************************************/
static int probe_board(hm2_rpspi_t *board) {
	int32_t ret;
	hm2_idrom_t idrom;
	const char *base;

#if defined(RPSPI_DEBUG_WRITE)
	uint32_t buf[8] = {0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f, 0x10111213, 0x14151617, 0x18191a1b, 0x1c1d1e1f};
	board->llio.write(&board->llio, 0x5ac3, buf, sizeof(buf));
#endif

	if((ret = check_cookie(board)) < 0)
		return ret;

	rtapi_print_msg(RPSPI_INFO, "hm2_rpspi: SPI%d/CE%d Valid cookie matched\n", board->spidevid, board->spiceid);

	// Read the IDROM from the board. The IDROM address offset was returned in
	// the cookie check.
	if(board->llio.read(&board->llio, (uint32_t)ret, &idrom, sizeof(hm2_idrom_t)) <= 0) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: SPI%d/CE%d Board ident read failed\n", board->spidevid, board->spiceid);
		return -EIO;	// Cookie could be read, so this is a comms error
	}

	// Detect board name and fill in informational values
	if(!(base = set_llio_info_spi(&board->llio, &idrom)))
		return -ENOENT;

	rtapi_print_msg(RPSPI_INFO, "hm2_rpspi: SPI%d/CE%d Base: %s.%d\n", board->spidevid, board->spiceid, base, board->nr);
	rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "%s.%d", base, board->nr);
	board->llio.comp_id = comp_id;
	board->llio.private = board;	// Self reference

	return 0;
}

/*************************************************/
static int peripheral_map(uint32_t membase, uint32_t memsize)
{
	int fd;
	int err;

	peripheralsize = memsize;

	if((fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: can't open /dev/mem\n");
		return -errno;
	}

	/* mmap BCM2835 GPIO and SPI peripherals */
	peripheralmem = mmap(NULL, peripheralsize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, (off_t)membase);
	err = errno;
	close(fd);
	if(peripheralmem == MAP_FAILED) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Can't map peripherals: %s\n", strerror(err));
		if (err == EPERM) {
			rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Try adding 'iomem=relaxed' to your kernel command-line.\n");
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

	rtapi_print_msg(RPSPI_INFO, "hm2_rpspi: Mapped peripherals from 0x%08x (size 0x%08x) to gpio:0x%p, spi:0x%p, aux:0x%p\n",
			membase, (uint32_t)peripheralsize, gpio, spi, aux);

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

/*************************************************/
static inline void gpio_fsel(uint32_t pin, uint32_t func)
{
	if(pin <= 53) {	/* There are 54 GPIOs */
		uint32_t bits = pin * 3;	/* Three bits per fsel field and 10 gpio per uint32_t */
		reg_wr(&gpio->gpfsel[bits / 30], (reg_rd(&gpio->gpfsel[bits / 30]) & ~(7 << (bits % 30))) | ((func & 7) << (bits % 30)));
	}
}

/*************************************************/
static inline void gpio_pull(unsigned pin, uint32_t pud)
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
	// Do bounds check on the parameters
	// The value follows GPIO_GPPUD_xxx definitions
	if(spi_pull_miso < 0 || spi_pull_miso > 2)	spi_pull_miso = 0;
	if(spi_pull_mosi < 0 || spi_pull_mosi > 2)	spi_pull_mosi = 0;
	if(spi_pull_sclk < 0 || spi_pull_sclk > 2)	spi_pull_sclk = 0;

	// Setup SPI pins to SPI0
	if(spi_probe & SPI0_PROBE_MASK) {
		if(spi_probe & SPI0_PROBE_CE0) {
			gpio_fsel(SPI0_PIN_CE_0, GPIO_FSEL_8_SPI0_CE0_N);
			gpio_pull(SPI0_PIN_CE_0, GPIO_GPPUD_PULLUP);
		}
		if(spi_probe & SPI0_PROBE_CE1) {
			gpio_fsel(SPI0_PIN_CE_1, GPIO_FSEL_7_SPI0_CE1_N);
			gpio_pull(SPI0_PIN_CE_1, GPIO_GPPUD_PULLUP);
		}
		gpio_fsel(SPI0_PIN_MISO, GPIO_FSEL_9_SPI0_MISO);
		gpio_fsel(SPI0_PIN_MOSI, GPIO_FSEL_10_SPI0_MOSI);
		gpio_fsel(SPI0_PIN_SCLK, GPIO_FSEL_11_SPI0_SCLK);
		// Enable/disable pullups on the pins on request
		gpio_pull(SPI0_PIN_MISO, spi_pull_miso);
		gpio_pull(SPI0_PIN_MOSI, spi_pull_mosi);
		gpio_pull(SPI0_PIN_SCLK, spi_pull_sclk);
		spi0_reset();
	}

	// Setup SPI pins to SPI1
	if(spi_probe & SPI1_PROBE_MASK) {
		if(spi_probe & SPI1_PROBE_CE0) {
			gpio_fsel(SPI1_PIN_CE_0, GPIO_FSEL_18_SPI1_CE0_N);
			gpio_pull(SPI1_PIN_CE_0, GPIO_GPPUD_PULLUP);
		}
		if(spi_probe & SPI1_PROBE_CE1) {
			gpio_fsel(SPI1_PIN_CE_1, GPIO_FSEL_17_SPI1_CE1_N);
			gpio_pull(SPI1_PIN_CE_1, GPIO_GPPUD_PULLUP);
		}
		if(spi_probe & SPI1_PROBE_CE2) {
			gpio_fsel(SPI1_PIN_CE_1, GPIO_FSEL_16_SPI1_CE2_N);
			gpio_pull(SPI1_PIN_CE_2, GPIO_GPPUD_PULLUP);
		}
		gpio_fsel(SPI1_PIN_MISO, GPIO_FSEL_19_SPI1_MISO);
		gpio_fsel(SPI1_PIN_MOSI, GPIO_FSEL_20_SPI1_MOSI);
		gpio_fsel(SPI1_PIN_SCLK, GPIO_FSEL_21_SPI1_SCLK);
		// Enable/disable pullups on the pins on request
		gpio_pull(SPI1_PIN_MISO, spi_pull_miso);
		gpio_pull(SPI1_PIN_MOSI, spi_pull_mosi);
		gpio_pull(SPI1_PIN_SCLK, spi_pull_sclk);

		// Check if SPI1 needs to be enabled
		aux_enables = reg_rd(&aux->enables);
		if(!(aux_enables & AUX_ENABLES_SPI1))
			reg_wr(&aux->enables, aux_enables | AUX_ENABLES_SPI1);	// Enable SPI1

		spi1_reset();
	}

#if defined(RPSPI_DEBUG_PIN)
	gpio_fsel(RPSPI_DEBUG_PIN, GPIO_FSEL_X_GPIO_OUTPUT);
	gpio_debug_pin(true);
#endif
}

/*************************************************/
static void peripheral_restore(void)
{
#if defined(RPSPI_DEBUG_PIN)
	gpio_debug_pin(true);
	gpio_fsel(RPSPI_DEBUG_PIN, GPIO_FSEL_X_GPIO_INPUT);
#endif
	// Restore all SPI pins to inputs and enable pull-up (no dangling inputs)
	if(spi_probe & SPI0_PROBE_MASK) {
		gpio_pull(SPI0_PIN_MISO, GPIO_GPPUD_PULLUP);	// MISO
		gpio_pull(SPI0_PIN_MOSI, GPIO_GPPUD_PULLUP);	// MOSI
		gpio_pull(SPI0_PIN_SCLK, GPIO_GPPUD_PULLUP);	// SCLK

		// Set SPI0 pins to GPIO input
		gpio_fsel(SPI0_PIN_MISO, GPIO_FSEL_X_GPIO_INPUT);
		gpio_fsel(SPI0_PIN_MOSI, GPIO_FSEL_X_GPIO_INPUT);
		gpio_fsel(SPI0_PIN_SCLK, GPIO_FSEL_X_GPIO_INPUT);
		if(spi_probe & SPI0_PROBE_CE0)
			gpio_fsel(SPI0_PIN_CE_0, GPIO_FSEL_X_GPIO_INPUT);
		if(spi_probe & SPI0_PROBE_CE1)
			gpio_fsel(SPI0_PIN_CE_1, GPIO_FSEL_X_GPIO_INPUT);
	}

	if(spi_probe & SPI1_PROBE_MASK) {
		// Only disable SPI1 if it was disabled before
		if(!(aux_enables & AUX_ENABLES_SPI1))
			reg_wr(&aux->enables, reg_rd(&aux->enables) & ~AUX_ENABLES_SPI1);

		gpio_pull(SPI1_PIN_MISO, GPIO_GPPUD_PULLUP);	// MISO
		gpio_pull(SPI1_PIN_MOSI, GPIO_GPPUD_PULLUP);	// MOSI
		gpio_pull(SPI1_PIN_SCLK, GPIO_GPPUD_PULLUP);	// SCLK

		// Set SPI1 pins to GPIO input
		gpio_fsel(SPI1_PIN_MISO, GPIO_FSEL_X_GPIO_INPUT);
		gpio_fsel(SPI1_PIN_MOSI, GPIO_FSEL_X_GPIO_INPUT);
		gpio_fsel(SPI1_PIN_SCLK, GPIO_FSEL_X_GPIO_INPUT);
		if(spi_probe & SPI1_PROBE_CE0)
			gpio_fsel(SPI1_PIN_CE_0, GPIO_FSEL_X_GPIO_INPUT);
		if(spi_probe & SPI1_PROBE_CE1)
			gpio_fsel(SPI1_PIN_CE_1, GPIO_FSEL_X_GPIO_INPUT);
		if(spi_probe & SPI1_PROBE_CE2)
			gpio_fsel(SPI1_PIN_CE_2, GPIO_FSEL_X_GPIO_INPUT);
	}
}

/*************************************************/
static uint8_t *read_file(const char *fname, size_t maxsize, size_t minsize)
{
	FILE *fp;
	struct stat sb;
	uint8_t *buf;
	size_t nn, fsize;

	if(-1 == stat(fname, &sb)) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Cannot stat '%s'\n", fname);
		return NULL;
	}

	if((size_t)sb.st_size < minsize) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Target file '%s' stat's less than minimum size of %zu bytes (st_size=%zu)\n", fname, minsize, (size_t)sb.st_size);
		return NULL;
	}

	nn = sb.st_size > (ssize_t)maxsize ? maxsize : (size_t)sb.st_size;
	if(!(buf = rtapi_kmalloc(nn+1, RTAPI_GFP_KERNEL))) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: No dynamic memory\n");
		return NULL;
	}
	memset(buf, 0, nn+1);
	if(!(fp = fopen(fname, "r"))) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Cannot open '%s' for read\n", fname);
		rtapi_kfree(buf);
		return NULL;
	}

	fsize = fread(buf, 1, nn, fp);
	fclose(fp);
	if(!fsize) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Nothing read from '%s' (errno=%d)\n", fname, errno);
		rtapi_kfree(buf);
		return NULL;
	}
	if(fsize < nn) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Short read from '%s'; read=%zu required>=%zu\n", fname, fsize, nn);
		rtapi_kfree(buf);
		return NULL;
	}

	return buf;
}

/*************************************************/
static inline int spi_devid(int index)
{
	return index < 2 ? 0 : 1;
}

static inline int spi_ceid(int index)
{
	return index < 2 ? index : index - 2;
}

static int hm2_rpspi_setup(void)
{
	int i, j;
	int retval = -1;
	uint8_t *buf;
	uint32_t pmembase;
	uint32_t pmemsize;

	// Set process-level message level if requested
	if(spi_debug >= RTAPI_MSG_NONE && spi_debug <= RTAPI_MSG_ALL)
		rtapi_set_msg_level(spi_debug);

	// Info about the hardware platform
	// This should read something like: "Raspberry Pi X Model Y Rev A.B"
	//
	// A 4095(+1) byte maximum buffer size should be somewhat future proof
	// for now, I guess. anyway, it is freed very fast again. Setting a
	// maximum prevents run-away allocations.
	if(!(buf = read_file("/proc/device-tree/model", 4095, 0))) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Unsupported Platform.\n");
		return -1;
	}
	rtapi_print_msg(RPSPI_INFO, "hm2_rpspi: Platform: %s\n", *buf ? (const char *)buf : "<no data from /proc/device-tree/model>");
	rtapi_kfree(buf);

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
	// A 4095(+1) byte maximum buffer size should be somewhat future proof
	// for now, I guess. anyway, it is freed very fast again. Setting a
	// maximum prevents run-away allocations.
	//
	// We require the ranges file to contain at least three 32-bit words.
	// These are required to be present for the RPi3 and RPi4 (and older
	// versions).
	if(!(buf = read_file("/proc/device-tree/soc/ranges", 4095, 12))) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Cannot determine IO base address and size.\n");
		return -1;
	}

	pmembase = be32toh(((uint32_t *)buf)[1]);	// This should do the trick for RPi3
	pmemsize = be32toh(((uint32_t *)buf)[2]);

	if(!pmembase) {
		// This is (probably) a RPi4 and the ranges file has a zero at
		// the base address. Here we need to read four 32-bit words to
		// get to the right values.
		rtapi_kfree(buf);
		if(!(buf = read_file("/proc/device-tree/soc/ranges", 4095, 16))) {
			rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: Cannot determine IO base address and size.\n");
			return -1;
		}
		pmembase = be32toh(((uint32_t *)buf)[2]);
		pmemsize = be32toh(((uint32_t *)buf)[3]);
	}
	rtapi_kfree(buf);

	if(!pmembase || !pmemsize) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: IO base address (0x%08x) or size (0x%08x) are zero.\n", pmembase, pmemsize);
		return -1;
	}
	rtapi_print_msg(RPSPI_INFO, "hm2_rpspi: Base address 0x%08x size 0x%08x\n", pmembase, pmemsize);


	if(-1 == spiclk_rate_rd)
		spiclk_rate_rd = spiclk_rate;

	// Lowest frequencies:
	// - SPI0  3814 Hz
	// - SPI1 30516 Hz
	// Therefore, we limit the frequency to 30 kHz. This is already too low
	// for any realtime stuff, but nice for debugging the interface.
	if(spiclk_rate < 30 || spiclk_rate > 63000) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: SPI clock rate '%d' too slow/fast. Must be >= 30 kHz and <= 63000 kHz\n", spiclk_rate);
		return -EINVAL;
	}

	if(spiclk_rate_rd < 30 || spiclk_rate_rd > 63000) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: SPI clock rate for reading '%d' too slow/fast. Must be >= 30 kHz and <= 63000 kHz\n", spiclk_rate_rd);
		return -EINVAL;
	}

	if((retval = peripheral_map(pmembase, pmemsize)) < 0) {
		rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: cannot map peripheral memory.\n");
		return retval;
	}

	peripheral_setup();

	memset(boards, 0, sizeof(boards));

	for(j = i = 0; i < RPSPI_MAX_BOARDS; i++) {
		int iddev = spi_devid(i);
		int idce = spi_ceid(i);
		uint32_t clkdiv;
		uint32_t clkratew;
		uint32_t clkrater;
		uint32_t clkbase;

		if(!(spi_probe & (1 << i)))		// Only probe if enabled
			continue;

		clkratew = spiclk_rate * 1000;		// Command-line specifies in kHz
		clkrater = spiclk_rate_rd * 1000;	// Command-line specifies in kHz
		clkbase  = read_spiclkbase();

		boards[j].nr = j;
		boards[j].spiclkratew = clkratew;
		boards[j].spiclkrater = clkrater;
		boards[j].spiclkbase  = clkbase;
		boards[j].spidevid    = iddev;
		boards[j].spiceid     = idce;

		rtapi_print_msg(RPSPI_INFO, "hm2_rpspi: SPI%d/CE%d clock rate: %d/%d Hz, VPU clock rate: %u Hz\n",
				iddev, idce, clkratew, clkrater, clkbase);
		if(!iddev) {
			switch(idce) {
			case 0: boards[j].spice = 0; break;			// Set SPI0 CE_0
			case 1: boards[j].spice = SPI_CS_CS_01; break;		// Set SPI0 CE_1
			}
			boards[j].llio.read  = hm2_rpspi_read_spi0;
			boards[j].llio.write = hm2_rpspi_write_spi0;
			if(!(clkdiv = spi0_clkdiv_calc(clkbase, clkratew)))
				clkdiv = 65536;
			rtapi_print_msg(RPSPI_INFO, "hm2_rpspi: SPI%d/CE%d write clock rate calculated: %d Hz (clkdiv=%u)\n", iddev, idce, clkbase / clkdiv, clkdiv);
			if(!(clkdiv = spi0_clkdiv_calc(clkbase, clkrater)))
				clkdiv = 65536;
			rtapi_print_msg(RPSPI_INFO, "hm2_rpspi: SPI%d/CE%d read clock rate calculated: %d Hz (clkdiv=%u)\n", iddev, idce, clkbase / clkdiv, clkdiv);
		} else {
			switch(idce) {
			case 0: boards[j].spice = AUX_SPI_CNTL0_CS_1 | AUX_SPI_CNTL0_CS_2; break;	// Set SPI1 CE_0
			case 1: boards[j].spice = AUX_SPI_CNTL0_CS_0 | AUX_SPI_CNTL0_CS_2; break;	// Set SPI1 CE_1
			case 2: boards[j].spice = AUX_SPI_CNTL0_CS_0 | AUX_SPI_CNTL0_CS_1; break;	// Set SPI1 CE_2
			}
			boards[j].llio.read  = hm2_rpspi_read_spi1;
			boards[j].llio.write = hm2_rpspi_write_spi1;
			clkdiv = 2 * (spi1_clkdiv_calc(clkbase, clkratew) + 1);
			rtapi_print_msg(RPSPI_INFO, "hm2_rpspi: SPI%d/CE%d write clock rate calculated: %d Hz (clkdiv=%u)\n", iddev, idce, clkbase / clkdiv, clkdiv);
			clkdiv = 2 * (spi1_clkdiv_calc(clkbase, clkrater) + 1);
			rtapi_print_msg(RPSPI_INFO, "hm2_rpspi: SPI%d/CE%d read clock rate calculated: %d Hz (clkdiv=%u)\n", iddev, idce, clkbase / clkdiv, clkdiv);
		}

		if((retval = probe_board(&boards[j])) < 0) {
			return retval;
		}

		if((retval = hm2_register(&boards[j].llio, config[j])) < 0) {
			rtapi_print_msg(RPSPI_ERR, "hm2_rpspi: hm2_register() failed for SPI%d/CE%d.\n", iddev, idce);
			return retval;
		}

		j++;	// Next board
	}

	return j > 0 ? 0 : -ENODEV;
}

/*************************************************/
static void hm2_rpspi_cleanup(void)
{
	if((void *)peripheralmem != MAP_FAILED) {
		peripheral_restore();
		munmap(peripheralmem, peripheralsize);
	}
	eshellf(HM2_LLIO_NAME, "/sbin/modprobe spi-bcm2835");
}

/*************************************************/
int rtapi_app_main()
{
	int ret;

	eshellf(HM2_LLIO_NAME, "/sbin/rmmod spi_bcm2835");

	if((comp_id = ret = hal_init("hm2_rpspi")) < 0)
		goto fail;

	if((ret = hm2_rpspi_setup()) < 0)
		goto fail;

	hal_ready(comp_id);
	return 0;

fail:
	hm2_rpspi_cleanup();
	return ret;
}

/*************************************************/
void rtapi_app_exit(void)
{
	hm2_rpspi_cleanup();
	hal_exit(comp_id);
}
