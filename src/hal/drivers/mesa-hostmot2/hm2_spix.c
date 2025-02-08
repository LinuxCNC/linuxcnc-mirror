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
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#include <rtapi.h>
#include <rtapi_app.h>
#include <rtapi_slab.h>

#define HM2_LLIO_NAME "hm2_spix"

#include "hostmot2-lowlevel.h"
#include "hostmot2.h"

#include "llio_info.h"
#include "spix.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("B.Stultiens");
MODULE_DESCRIPTION("Driver for HostMot2 devices connected via SPI");
MODULE_SUPPORTED_DEVICE("Mesa-AnythingIO-7i90,7c80,7c81,7i43");

#define NELEM(x)	(sizeof(x) / sizeof(*(x)))

/*
 * Buffer for queued transfers
 */
typedef struct __buffer_t {
	void		*ptr;	// Actual buffer
	size_t		n;		// Number of elements in buffer
	size_t		na;		// Allocated size of buffer in number of elements
} buffer_t;

/*
 * Buffer reference for copying read data back to original buffers
 */
typedef struct __rxref_t {
	void	*ptr;		// The read buffer from the queue_read call
	int		size;		// Size of read buffer from the queue_read call
	int		idx;		// Data position index into the board's rbuf data
} rxref_t;

/*
 * Our connected HM2 board data container
 */
typedef struct __spix_board_t {
	hm2_lowlevel_io_t llio;		// Upstream container
	int				nr;			// Board number
	buffer_t		wbuf;		// Queued writes buffer
	buffer_t		rbuf;		// Queued reads buffer
	buffer_t		rref;		// Queued read buffer references
	const spix_port_t *port;	// The low-level hardware port
} spix_board_t;

static spix_board_t boards[SPIX_MAX_BOARDS];	// Connected boards
static int comp_id;				// Upstream assigned component ID

/*
 * Supported hardware drivers
 * These are defined in spix_XXX source files.
 */
extern spix_driver_t spix_driver_rpi3;
extern spix_driver_t spix_driver_rpi5;
extern spix_driver_t spix_driver_spidev;

static const spix_driver_t *drivers[] = {
	&spix_driver_rpi3,		// RPi3, RPi3[ab]+, RPi4b and RPi4CM
	&spix_driver_rpi5,		// RPi5 and RPi5CM
	// TODO: orange pi
	// TODO: banana pi
	&spix_driver_spidev,	// Default spidev driver as fallback
};

static const spix_driver_t *hwdriver;	// The active driver

// The value of the cookie when read from the board. An actual cookie read
// consists of four words. The fourth value is the idrom address, which may
// vary per board.
static const uint32_t iocookie[3] = {
	HM2_IOCOOKIE,
	// The following words spell HOSTMOT2
	0x54534f48,	// TSOH
	0x32544f4d	// 2TOM
};

/*
 * Configuration parameters forwarded to hostmot2 hm2_register() call
 */
static char *config[SPIX_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(config, SPIX_MAX_BOARDS, "config string for the AnyIO boards (see hostmot2(9) manpage)")

/*
 * SPI clock rates for read and write.
 */
static int spiclk_rate[SPIX_MAX_BOARDS] = { 25000 };
static int spiclk_rate_rd[SPIX_MAX_BOARDS];
RTAPI_MP_ARRAY_INT(spiclk_rate, SPIX_MAX_BOARDS, "SPI clock rates in kHz (default 25000 kHz)")
RTAPI_MP_ARRAY_INT(spiclk_rate_rd, SPIX_MAX_BOARDS, "SPI clock rates for reading in kHz (default same as spiclk_rate)")

/*
 * Forcefully specify the hardware driver
 */
static const char *force_driver = NULL;
RTAPI_MP_STRING(force_driver, "Force one specific hardware driver (default empty, auto detecting hardware))")

/*
 * Which SPI port(s) to probe
 */
static int spi_probe = SPIX_PROBE_SPI0_CE0;
RTAPI_MP_INT(spi_probe, "Bit-field to select which SPI/CE combinations to probe (default 1 (SPI0/CE0))")

/*
 * Normally, all requests are queued if requested by upstream and sent in one
 * bulk transfer. This reduces overhead significantly. Disabling the queue make
 * each transfer visible and more easily debugable.
 */
static int spi_noqueue = 0;
RTAPI_MP_INT(spi_noqueue, "Disable queued SPI requests, use for debugging only (default 0 (off))")

/*
 * Set the message level for debugging purpose. This has the (side-)effect that
 * all modules within this process will start spitting out messages at the
 * requested level.
 * The upstream message level is not touched if spi_debug == -1.
 */
static int spi_debug = -1;
RTAPI_MP_INT(spi_debug, "Set message level for debugging purpose [0...5] where 0=none and 5=all (default: -1; upstream defined)")

/*
 * Spidev driver device node path overrides
 */
static char *spidev_path[SPIX_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(spidev_path, SPIX_MAX_BOARDS, "The device node path override(s) for the spidev driver (default /dev/spidev{0.[01],1.[012]})")

/*
 * We have these for compatibility with the hm2_rpspi driver. You can simply
 * change the driver name in the ini/hal files without having to switch
 * arguments.
 */
static int spi_pull_miso = -1;
static int spi_pull_mosi = -1;
static int spi_pull_sclk = -1;
RTAPI_MP_INT(spi_pull_miso, "Obsolete parameter")
RTAPI_MP_INT(spi_pull_mosi, "Obsolete parameter")
RTAPI_MP_INT(spi_pull_sclk, "Obsolete parameter")

/*********************************************************************/
/*
 * Buffer management for queued transfers.
 */
static int buffer_check_room(buffer_t *b, size_t n, size_t elmsize)
{
	if(!b->ptr || !b->na) {
		b->na = 64;	// Default to this many elements
		b->n = 0;
		b->ptr = rtapi_kmalloc(elmsize * b->na, RTAPI_GPF_KERNEL);
		return b->ptr == NULL;
	}

	if(b->n + n > b->na) {
		do {
			b->na *= 2;	// Double storage capacity
		} while(b->n + n > b->na);	// Until we have enough room
		void *p = rtapi_krealloc(b->ptr, elmsize * b->na, RTAPI_GPF_KERNEL);
		if(!p)
			return 1;
		b->ptr = p;
	}
	return 0;
}

static void buffer_free(buffer_t *b)
{
	if(b->ptr) {
		rtapi_kfree(b->ptr);
		b->ptr = NULL;
		b->n = b->na = 0;
	}
}

/*********************************************************************/
/*
 * HM2 interface: Write buffer to SPI
 * Writes the buffer to SPI, prepended with a command word.
 */
static int hm2_spix_write(hm2_lowlevel_io_t *llio, rtapi_u32 addr, const void *buffer, int size)
{
	spix_board_t *brd = (spix_board_t *)llio;
	int txlen = size / sizeof(uint32_t);	// uint32_t words to transmit
	uint32_t txbuf[SPIX_MAX_MSG];			// local buffer for entire transfer

	if(size == 0)
		return 1;	// Nothing to do, return success
	if((size % sizeof(uint32_t)) || txlen + 1 > SPIX_MAX_MSG)
		return 0;	// -EINVAL;

	txbuf[0] = spix_cmd_write(addr, txlen, true);	// Setup write command
	memcpy(&txbuf[1], buffer, size);			// Setup write data
	return brd->port->transfer(brd->port, txbuf, txlen + 1, 0);	// Do transfer
}

/*
 * HM2 interface: Read buffer from SPI
 * Reads from SPI after sending the appropriate command. Sends one word with
 * the command followed by writing zeros while reading.
 */
static int hm2_spix_read(hm2_lowlevel_io_t *llio, rtapi_u32 addr, void *buffer, int size)
{
	spix_board_t *brd = (spix_board_t *)llio;
	int rxlen = size / sizeof(uint32_t);	// uint32_t words to receive
	uint32_t rxbuf[SPIX_MAX_MSG];			// local buffer for entire transfer
	int rv;

	if(size == 0)
		return 1;	// Nothing to do, return success
	if((size % sizeof(uint32_t)) || rxlen + 1 > SPIX_MAX_MSG)
		return 0;	// -EINVAL;

	memset(rxbuf, 0, sizeof(rxbuf));			// Clear buffer; reads stuff zero writes
	rxbuf[0] = spix_cmd_read(addr, rxlen, true);	// Setup read command
	rv = brd->port->transfer(brd->port, rxbuf, rxlen + 1, 1);	// Do transfer
	memcpy(buffer, &rxbuf[1], size);			// Copy received data (even with errors...)
	return rv;
}

/*
 * HM2 interface: Queue read
 * Collects the read address and buffer for bulk-read later on.
 */
static int hm2_spix_queue_read(hm2_lowlevel_io_t *llio, rtapi_u32 addr, void *buffer, int size)
{
	spix_board_t *brd = (spix_board_t *)llio;
	int rxlen = size / sizeof(uint32_t);

	if(size == 0)
		return 1;	// Nothing to do, return success
	if((size % sizeof(uint32_t)) || rxlen + 1 > SPIX_MAX_MSG)
		return 0;	// -EINVAL;

	if(buffer_check_room(&brd->rbuf, rxlen + 1, sizeof(uint32_t))) {
		LL_ERR("Failed to allocate read buffer memory\n");
		return 0;	// -ENOMEM;
	}

	if(buffer_check_room(&brd->rref, 1, sizeof(rxref_t))) {
		LL_ERR("Failed to allocate read queue reference memory\n");
		return 0;	// -ENOMEM;
	}

	// Add a reference control structure to remember where the data will be put
	// after the read is executed
	rxref_t *ref = &((rxref_t *)brd->rref.ptr)[brd->rref.n];
	ref->ptr = buffer;
	ref->size = size;
	ref->idx = brd->rbuf.n + 1;	// offset 0 is command, 1 is data
	brd->rref.n += 1;

	uint32_t *rbptr = (uint32_t *)brd->rbuf.ptr;
	rbptr[brd->rbuf.n] = spix_cmd_read(addr, rxlen, true);			// The read command
	memset(&rbptr[brd->rbuf.n + 1], 0, rxlen * sizeof(uint32_t));	// Fill zeros as data
	brd->rbuf.n += rxlen + 1;

	return 1;
}

/*
 * HM2 interface: Send queued reads
 * Performs a SPI transfer of all collected read requests in one burst and
 * copies back the data received in the individual buffers.
 */
static int hm2_spix_send_queued_reads(hm2_lowlevel_io_t *llio)
{
	uint32_t cookie[3] = {0, 0, 0};
	spix_board_t *brd = (spix_board_t *)llio;
	int rv;

	// Add a cookie read at the end of the queued reads to verify comms
	hm2_spix_queue_read(llio, HM2_ADDR_IOCOOKIE, cookie, sizeof(cookie));

	rv = brd->port->transfer(brd->port, brd->rbuf.ptr, brd->rbuf.n, 1);
	if(rv > 0) {
		// The transfer read the data into the read buffer. Now copy it into
		// the individual read requests' buffers.
		size_t i;
		for(i = 0; i < brd->rref.n; i++) {
			rxref_t *rxref = &((rxref_t *)brd->rref.ptr)[i];
			memcpy(rxref->ptr, &((uint32_t *)brd->rbuf.ptr)[rxref->idx], rxref->size);
		}

		// Check the cookie read. Its an IO error if it does not match
		if(memcmp(cookie, iocookie, sizeof(iocookie)))
			rv = 0;	//-EIO;
	}
	brd->rbuf.n = 0;	// Reset the queue buffers
	brd->rref.n = 0;

	return rv;
}

/*
 * HM2 interface: Receive queued reads
 * This is a no-op in SPI. The data was already received when the transfer was
 * performed in hm2_spix_send_queued_reads() above. The data was copied to
 * the requester(s) immediately after the transfer.
 */
static int hm2_spix_receive_queued_reads(hm2_lowlevel_io_t *llio)
{
	(void)llio;
	return 1;
}

/*
 * HM2 interface: Queue write
 * Collects the write address and data for bulk-write later on.
 */
static int hm2_spix_queue_write(hm2_lowlevel_io_t *llio, rtapi_u32 addr, const void *buffer, int size)
{
	spix_board_t *brd = (spix_board_t *)llio;
	int txlen = size / sizeof(uint32_t);

	if(size == 0)
		return 1;	// Nothing to do, return success
	if((size % sizeof(uint32_t)) || txlen + 1 > SPIX_MAX_MSG)
		return 0;	// -EINVAL;

	if(buffer_check_room(&brd->wbuf, txlen + 1, sizeof(uint32_t))) {
		LL_ERR("Failed to allocate write buffer memory\n");
		return 0;	// -ENOMEM;
	}

	uint32_t *wbptr = (uint32_t *)brd->wbuf.ptr;
	wbptr[brd->wbuf.n] = spix_cmd_write(addr, txlen, true);				// The write command
	memcpy(&wbptr[brd->wbuf.n + 1], buffer, txlen * sizeof(uint32_t));	// The data
	brd->wbuf.n += txlen + 1;
	return 1;
}

/*
 * HM2 interface: Send queued writes
 * Performs a SPI transfer of all collected write requests in one burst.
 */
static int hm2_spix_send_queued_writes(hm2_lowlevel_io_t *llio)
{
	spix_board_t *brd = (spix_board_t *)llio;
	int rv = brd->port->transfer(brd->port, brd->wbuf.ptr, brd->wbuf.n, 0);
	brd->wbuf.n = 0;	// Reset the queue buffer
	return rv;
}

/*********************************************************************/

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

static int32_t check_cookie(spix_board_t *board)
{
	uint32_t cookie[4] = {0, 0, 0, 0};
	uint32_t ca;
	uint32_t co;
	const spix_port_t *port = board->port;

	// We read four (4) 32-bit words. The first three are the cookie and
	// the fourth entry is the idrom address offset. The offset is used in
	// the call to get the idrom if we successfully match a cookie.
	if(!board->llio.read(&board->llio, HM2_ADDR_IOCOOKIE, cookie, sizeof(cookie)))
		return -ENODEV;

	if(!memcmp(cookie, iocookie, sizeof(iocookie)) && cookie[3] < 0x10000) {
		LL_INFO("Cookie read: %08x %08x %08x, idrom@%04x\n", cookie[0], cookie[1], cookie[2], cookie[3]);
		return (int32_t)cookie[3];	// The cookie got read correctly
	}

	LL_ERR("%s: Invalid cookie, read: %08x %08x %08x %08x,"
			" expected: %08x %08x %08x followed by a value less than 0x10000\n",
			port->name,
			cookie[0], cookie[1], cookie[2], cookie[3],
			iocookie[0], iocookie[1], iocookie[2]);

	// Lets see if we can tell why it went wrong
	ca = cookie[0] & cookie[1] & cookie[2];	// All ones -> ca == ones
	co = cookie[0] | cookie[1] | cookie[2];	// All zero -> co == zero

	if((!co && port->miso_pull == SPIX_PULL_DOWN) || (ca == 0xffffffff && port->miso_pull == SPIX_PULL_UP)) {
		LL_ERR("%s: No drive seen on MISO line (kept at pull-%s level)."
			" No board connected or bad connection?\n",
			port->name, port->miso_pull == SPIX_PULL_DOWN ? "down" : "up");
	} else if(!co || ca == 0xffffffff) {
		LL_ERR("%s MISO line stuck at %s level."
			" Maybe bad connection, a short-circuit or no board attached?\n",
			port->name, !co ? "low" : "high");
	} else {
		// If you read too fast, then the bit-setup times are not satisfied and
		// the input may be shifted by one bit-clock. Depending transfer width,
		// every 8th, 16th or 32nd bit may not arrive soon enough and the
		// previous data will be clocked in.
		//
		// We can detect this eventuality by checking the cookie against a
		// bit-shifted version and mask the bits that may have fallen off the
		// cliff. If the cookie matches (all zeroes in the XOR result), then it
		// is most likely that the read-clock frequency is too high.
		//
		unsigned ones;
		unsigned i;
		uint32_t mask = ~0x00000001;	// for 32-bit transfers
		if(port->width == 8)
			mask = ~0x01010101;
		else if(port->width == 16)
			mask = ~0x00010001;

		for(ones = i = 0; i < 3; i++) {
			ones += count_ones((iocookie[i] ^ (cookie[i] << 1)) & mask);
		}
		if(!ones) {
			// No ones in the XOR result -> the cookie is probably bit-shifted
			LL_ERR("%s: MISO input is bit-shifted by one bit."
				" SPI read clock frequency probably too high.\n",
				port->name);
		} else {
			// More bits are wrong, erratic behaviour
			LL_ERR("%s: MISO input does not match any expected bit-pattern (>= %u bit difference)."
				" Maybe SPI read clock frequency too high or noise on the input?\n",
				port->name, ones);
		}
	}
	return -ENODEV;
}

/*************************************************/
static int probe_board(spix_board_t *board)
{
	const spix_port_t *port = board->port;
	int32_t ret;
	hm2_idrom_t idrom;
	const char *base;

	if((ret = check_cookie(board)) < 0)
		return ret;

	LL_INFO("%s: Valid cookie matched, idrom@%04x\n", port->name, ret);

	// Read the IDROM from the board. The IDROM address offset was returned in
	// the cookie check.
	if(!board->llio.read(&board->llio, (uint32_t)ret, &idrom, sizeof(hm2_idrom_t))) {
		LL_ERR("%s: Board idrom read failed\n", port->name);
		return -EIO;	// Cookie could be read, so this is a comms error
	}

	// Detect board name and fill in informational values
	if(!(base = set_llio_info_spi(&board->llio, &idrom)))
		return -ENOENT;

	LL_INFO("%s: Base: %s.%d\n", port->name, base, board->nr);

	rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "%s.%d", base, board->nr);
	board->llio.comp_id = comp_id;
	board->llio.private = board;	// Self reference

	return 0;
}

/*************************************************/

/* Read at most bufsize-1 bytes from fname */
ssize_t spix_read_file(const char *fname, void *buffer, size_t bufsize)
{
	int fd;
	ssize_t len;

	memset(buffer, 0, bufsize);

	if((fd = rtapi_open_as_root(fname, O_RDONLY)) < 0) {
		int e = errno;
		LL_ERR("Cannot open '%s' for read (errno=%d: %s)\n", fname, e, strerror(e));
		return -e;
	}

	while(1) {
		len = read(fd, buffer, bufsize - 1);
		if(len == 0) {
			LL_ERR("Nothing read from '%s', file contains no data\n", fname);
		} else if(len < 0) {
			int e = errno;
			if(e == EINTR)
				continue;	// Interrupted syscall, retry read
			LL_ERR("Error reading from '%s' (errno=%d: %s)\n", fname, e, strerror(e));
			return -e;
		}
		break;
	}
	close(fd);
	return len;
}

/*************************************************/

static int spix_setup(void)
{
	char buf[256];
	ssize_t buflen;
	char *cptr;
	const int DTC_MAX = 8;
	const char *dtcs[DTC_MAX + 1];	// Last entry will always be NULL

	// Setup the clock rate settings from the arguments.
	// The driver is responsible for actual checking min/max clock frequency.
	for(unsigned i = 0; i < SPIX_MAX_BOARDS; i++) {
		if(spiclk_rate[i] < 1)	// If not specified
			spiclk_rate[i] = spiclk_rate[0];	// use first

		if(spiclk_rate_rd[i] < 1)	// If not specified
			spiclk_rate_rd[i] = spiclk_rate[i];	// use write rate as read rate

	}

	// Check if spi_pull_{miso,mosi,sclk} were set and warn if they were
	if(spi_pull_miso != -1 || spi_pull_mosi != -1 || spi_pull_sclk != -1) {
		LL_WARN("Setting spi_pull_{miso,mosi,sclk} has no effect in the hm2_spix driver.\n");
	}

	if(spi_probe > (1 << SPIX_MAX_BOARDS) - 1) {
		LL_WARN("Probing with spi_probe must not have flags larger than %d included; truncating.\n", 1 << (SPIX_MAX_BOARDS - 1));
		spi_probe &= (1 << SPIX_MAX_BOARDS) - 1;
	}

	if(!spi_probe) {
		LL_ERR("No SPI ports to probe (spi_probe is zero).\n");
		return -EINVAL;
	}

	// Set process-level message level if requested
	if(spi_debug >= RTAPI_MSG_NONE && spi_debug <= RTAPI_MSG_ALL)
		rtapi_set_msg_level(spi_debug);

	// Read the 'compatible' string-list from the device-tree
	buflen = spix_read_file("/proc/device-tree/compatible", buf, sizeof(buf));
	if(buflen < 0) {
		LL_ERR("Failed to read platform identity.\n");
		return buflen;	// negative errno from read_file()
	}

	// Decompose the device-tree buffer into a string-list with the pointers to
	// each string in dtcs. Don't go beyond the buffer's size.
	memset(dtcs, 0, sizeof(dtcs));
	cptr = buf;
	for(unsigned i = 0; i < DTC_MAX && cptr; i++) {
		dtcs[i] = cptr;
		int j = strlen(cptr);
		if((cptr - buf) + j + 1 < buflen)
			cptr += j + 1;
		else
			cptr = NULL;
	}

	// If the driver is forced, check if it actually exists
	if(force_driver) {
		unsigned i;
		for(i = 0; i < NELEM(drivers); i++) {
			if(!strcmp(force_driver, drivers[i]->name))
				break;
		}
		if(i >= NELEM(drivers)) {
			LL_ERR("Unsupported hardware driver '%s' passed to force_driver option\n", force_driver);
			return -ENODEV;
		}
	}

	// Let each driver do a detect and stop when a match is found.
	for(unsigned i = 0; i < NELEM(drivers); i++) {
		if(force_driver && strcmp(force_driver, drivers[i]->name))
			continue;
		if(!drivers[i]->detect(dtcs)) {
			hwdriver = drivers[i];
			break;
		}
	}

	if(!hwdriver) {
		if(force_driver) {
			LL_ERR("Unsupported platform: '%s' for forced driver '%s'\n", buf, hwdriver->name);
		} else {
			LL_ERR("Unsupported platform: '%s'\n", buf);
		}
		return -ENODEV;
	}

	int i, j;
	if((i = hwdriver->setup(spi_probe)) < 0) {	// Let the hardware driver do its thing
		LL_INFO("Failed to initialize hardware driver\n");
		return i;
	}

	LL_INFO("Platform: '%s' (%s)\n", hwdriver->model, hwdriver->dtc);

	memset(boards, 0, sizeof(boards));

	// Follows SPI0/CE0, SPI0/CE1, SPI1/CE0, SPI1/CE1 and SPI1/CE2
	for(j = i = 0; i < SPIX_MAX_BOARDS; i++) {
		const spix_port_t *port;
		int err;
		spix_args_t sa;

		if(!(spi_probe & (1 << i)))		// Only probe if enabled
			continue;

		// The clock is increased by 1 kHz to compensate for the truncation of
		// the listed values. The clock divider calculations will always be
		// rounded down, making it consistent between hardware drivers and
		// kernel's spidev driver.
		// For example: On the RPi5, a clock of 33333 kHz would become 25000
		// kHz without compensation because of recurring 3 behind the comma in
		// 33333 kHz, which got truncated. With compensation we use 33334 kHz
		// as the rate and that gets rounded down to 33333 kHz in the
		// calculation.
		sa.clkw = (spiclk_rate[j] + 1) * 1000;
		sa.clkr = (spiclk_rate_rd[j] + 1) * 1000;
		sa.spidev = spidev_path[j];
		if(NULL == (port = hwdriver->open(i, &sa))) {
			LL_INFO("Failed to open hardware port index %d\n", i);
			return i;
		}

		LL_INFO("%s opened\n", port->name);

		boards[j].nr = j;
		boards[j].port = port;
		boards[j].llio.read  = hm2_spix_read;
		boards[j].llio.write = hm2_spix_write;
		if(!spi_noqueue) {
			boards[j].llio.queue_read  = hm2_spix_queue_read;
			boards[j].llio.send_queued_reads  = hm2_spix_send_queued_reads;
			boards[j].llio.receive_queued_reads  = hm2_spix_receive_queued_reads;
			boards[j].llio.queue_write  = hm2_spix_queue_write;
			boards[j].llio.send_queued_writes  = hm2_spix_send_queued_writes;
		}

		if((err = probe_board(&boards[j])) < 0) {
			return err;
		}

		if((err = hm2_register(&boards[j].llio, config[j])) < 0) {
			LL_ERR("%s: hm2_register() failed.\n", port->name);
			return err;
		}

		j++;	// Next board
	}

	return j > 0 ? 0 : -ENODEV;
}

/*************************************************/
static void spix_cleanup(void)
{
	int i;
	// Cleanup memory allocations
	for(i = 0; i < SPIX_MAX_BOARDS; i++) {
		buffer_free(&boards[i].wbuf);
		buffer_free(&boards[i].rbuf);
		buffer_free(&boards[i].rref);
	}

	if(hwdriver) {
		hwdriver->cleanup();
		hwdriver = NULL;
	}
}

/*************************************************/
int rtapi_app_main()
{
	int ret;

	if((comp_id = ret = hal_init(HM2_LLIO_NAME)) < 0)
		goto fail;

	if((ret = spix_setup()) < 0)
		goto fail;

	hal_ready(comp_id);
	return 0;

fail:
	spix_cleanup();
	return ret;
}

/*************************************************/
void rtapi_app_exit(void)
{
	spix_cleanup();
	hal_exit(comp_id);
}

// vim: ts=4
