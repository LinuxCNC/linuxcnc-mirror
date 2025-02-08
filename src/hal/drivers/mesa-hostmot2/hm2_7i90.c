
//
//    Copyright (C) 2013-2014 Kim Kirwan and Sebastian Kuzminsky
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//


#include <rtapi_io.h>

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_math.h"
#include "rtapi_string.h"

#include "hal.h"

#include "hal/drivers/mesa-hostmot2/bitfile.h"
#include "hal/drivers/mesa-hostmot2/hostmot2-lowlevel.h"
#include "hal/drivers/mesa-hostmot2/hm2_7i90.h"


static int comp_id;

#ifdef MODULE_INFO
MODULE_INFO(linuxcnc, "component:hm2_7i90:LinuxCNC HAL driver for the Mesa Electronics 7i90 EPP Anything IO board with HostMot2 firmware.");
MODULE_INFO(linuxcnc, "license:GPL");
#endif // MODULE_INFO

MODULE_LICENSE("GPL");

static int ioaddr[HM2_7I90_MAX_BOARDS] = { 0, [1 ... (HM2_7I90_MAX_BOARDS-1)] = -1 };
RTAPI_MP_ARRAY_INT(ioaddr, HM2_7I90_MAX_BOARDS, "base address of the parallel port(s) (see hm2_7i90(9) manpage)");

static int ioaddr_hi[HM2_7I90_MAX_BOARDS] = { [0 ... (HM2_7I90_MAX_BOARDS-1)] = 0 };
RTAPI_MP_ARRAY_INT(ioaddr_hi, HM2_7I90_MAX_BOARDS, "secondary address of the parallel port(s) (see hm2_7i90(9) manpage)");

static int epp_wide[HM2_7I90_MAX_BOARDS] = { [0 ... (HM2_7I90_MAX_BOARDS-1)] = 1 };
RTAPI_MP_ARRAY_INT(epp_wide, HM2_7I90_MAX_BOARDS, "set to 0 to disable wide EPP mode (see (hm2_7i90(9) manpage)");

int debug_epp = 0;
RTAPI_MP_INT(debug_epp, "Developer/debug use only!  Enable debug logging of most EPP\ntransfers.");

static char *config[HM2_7I90_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(config, HM2_7I90_MAX_BOARDS, "config string(s) for the 7i90 board(s) (see hostmot2(9) manpage)");

//
// this data structure keeps track of all the 7i90 boards found
//

static hm2_7i90_t board[HM2_7I90_MAX_BOARDS];
static int num_boards;




//
// EPP I/O code
//

static inline void hm2_7i90_epp_addr8(rtapi_u8 addr, hm2_7i90_t *board) {
    rtapi_outb(addr, board->port.base + HM2_7I90_EPP_ADDRESS_OFFSET);
    LL_PRINT_IF(debug_epp, "selected address 0x%02X\n", addr);
}

static inline void hm2_7i90_epp_addr16(rtapi_u16 addr, hm2_7i90_t *board) {
    rtapi_outb((addr & 0x00FF), board->port.base + HM2_7I90_EPP_ADDRESS_OFFSET);
    rtapi_outb((addr >> 8),     board->port.base + HM2_7I90_EPP_ADDRESS_OFFSET);
    LL_PRINT_IF(debug_epp, "selected address 0x%04X\n", addr);
}

static inline void hm2_7i90_epp_write(int w, hm2_7i90_t *board) {
    rtapi_outb(w, board->port.base + HM2_7I90_EPP_DATA_OFFSET);
    LL_PRINT_IF(debug_epp, "wrote data 0x%02X\n", w);
}

static inline int hm2_7i90_epp_read(hm2_7i90_t *board) {
    int val;
    val = rtapi_inb(board->port.base + HM2_7I90_EPP_DATA_OFFSET);
    LL_PRINT_IF(debug_epp, "read data 0x%02X\n", val);
    return val;
}

static inline rtapi_u32 hm2_7i90_epp_read32(hm2_7i90_t *board) {
    uint32_t data;

    if (board->epp_wide) {
        data = rtapi_inl(board->port.base + HM2_7I90_EPP_DATA_OFFSET);
        LL_PRINT_IF(debug_epp, "read data 0x%08X\n", data);
    } else {
        uint8_t a, b, c, d;
        a = hm2_7i90_epp_read(board);
        b = hm2_7i90_epp_read(board);
        c = hm2_7i90_epp_read(board);
        d = hm2_7i90_epp_read(board);
        data = a + (b<<8) + (c<<16) + (d<<24);
    }

    return data;
}

static inline void hm2_7i90_epp_write32(uint32_t w, hm2_7i90_t *board) {
    if (board->epp_wide) {
        rtapi_outl(w, board->port.base + HM2_7I90_EPP_DATA_OFFSET);
        LL_PRINT_IF(debug_epp, "wrote data 0x%08X\n", w);
    } else {
        hm2_7i90_epp_write((w) & 0xFF, board);
        hm2_7i90_epp_write((w >>  8) & 0xFF, board);
        hm2_7i90_epp_write((w >> 16) & 0xFF, board);
        hm2_7i90_epp_write((w >> 24) & 0xFF, board);
    }
}

static inline uint8_t hm2_7i90_epp_read_status(hm2_7i90_t *board) {
    uint8_t val;
    val = rtapi_inb(board->port.base + HM2_7I90_EPP_STATUS_OFFSET);
    LL_PRINT_IF(debug_epp, "read status 0x%02X\n", val);
    return val;
}

static inline void hm2_7i90_epp_write_status(uint8_t status_byte, hm2_7i90_t *board) {
    rtapi_outb(status_byte, board->port.base + HM2_7I90_EPP_STATUS_OFFSET);
    LL_PRINT_IF(debug_epp, "wrote status 0x%02X\n", status_byte);
}

static inline void hm2_7i90_epp_write_control(uint8_t control_byte, hm2_7i90_t *board) {
    rtapi_outb(control_byte, board->port.base + HM2_7I90_EPP_CONTROL_OFFSET);
    LL_PRINT_IF(debug_epp, "wrote control 0x%02X\n", control_byte);
}

// returns TRUE if there's a timeout
static inline int hm2_7i90_epp_check_for_timeout(hm2_7i90_t *board) {
    return (hm2_7i90_epp_read_status(board) & 0x01);
}

static int hm2_7i90_epp_clear_timeout(hm2_7i90_t *board) {
    uint8_t status;

    if (!hm2_7i90_epp_check_for_timeout(board)) {
        return 1;
    }

    /* To clear timeout some chips require double read */
    (void)hm2_7i90_epp_read_status(board);

    // read in the actual status register
    status = hm2_7i90_epp_read_status(board);

    hm2_7i90_epp_write_status(status | 0x01, board);  // Some reset by writing 1
    hm2_7i90_epp_write_status(status & 0xFE, board);  // Others by writing 0

    if (hm2_7i90_epp_check_for_timeout(board)) {
        LL_PRINT("failed to clear EPP Timeout!\n");
        return 0;  // fail
    }
    return 1;  // success
}


//
// misc generic helper functions
//

// FIXME: this is bogus
static void hm2_7i90_nanosleep(unsigned long int nanoseconds) {
    long unsigned int max_ns_delay;

    max_ns_delay = rtapi_delay_max();

    while (nanoseconds > max_ns_delay) {
        rtapi_delay(max_ns_delay);
        nanoseconds -= max_ns_delay;
    }

    rtapi_delay(nanoseconds);
}




//
// these are the low-level i/o functions exported to the hostmot2 driver
//

int hm2_7i90_read(hm2_lowlevel_io_t *this, rtapi_u32 addr, void *buffer, int size) {
    int bytes_remaining = size;
    hm2_7i90_t *board = this->private;

    hm2_7i90_epp_addr16(addr | HM2_7I90_ADDR_AUTOINCREMENT, board);

    for (; bytes_remaining > 3; bytes_remaining -= 4) {
        *((rtapi_u32*)buffer) = hm2_7i90_epp_read32(board);
        buffer += 4;
    }

    for ( ; bytes_remaining > 0; bytes_remaining --) {
        *((rtapi_u8*)buffer) = hm2_7i90_epp_read(board);
        buffer ++;
    }

    if (hm2_7i90_epp_check_for_timeout(board)) {
        THIS_PRINT("EPP timeout on data cycle of read(addr=0x%04x, size=%d)\n", addr, size);
        (*this->io_error) = 1;
        this->needs_reset = 1;
        hm2_7i90_epp_clear_timeout(board);
        return 0;  // fail
    }

    return 1;  // success
}




int hm2_7i90_write(hm2_lowlevel_io_t *this, rtapi_u32 addr, const void *buffer, int size) {
    int bytes_remaining = size;
    hm2_7i90_t *board = this->private;

    hm2_7i90_epp_addr16(addr | HM2_7I90_ADDR_AUTOINCREMENT, board);

    for (; bytes_remaining > 3; bytes_remaining -= 4) {
        hm2_7i90_epp_write32(*((rtapi_u32*)buffer), board);
        buffer += 4;
    }

    for ( ; bytes_remaining > 0; bytes_remaining --) {
        hm2_7i90_epp_write(*((rtapi_u8*)buffer), board);
        buffer ++;
    }

    if (hm2_7i90_epp_check_for_timeout(board)) {
        THIS_PRINT("EPP timeout on data cycle of write(addr=0x%04x, size=%d)\n", addr, size);
        (*this->io_error) = 1;
        this->needs_reset = 1;
        hm2_7i90_epp_clear_timeout(board);
        return 0;
    }

    return 1;
}




int hm2_7i90_program_fpga(hm2_lowlevel_io_t *this, const bitfile_t *bitfile) {
    int orig_debug_epp = debug_epp;  // we turn off EPP debugging for this part...
    hm2_7i90_t *board = this->private;
    int64_t start_time, end_time;
    const rtapi_u8 *firmware = bitfile->e.data;


    //
    // send the firmware
    //

    debug_epp = 0;
    start_time = rtapi_get_time();

    // select the CPLD's data address
    hm2_7i90_epp_addr8(0, board);

    for (unsigned i = 0; i < bitfile->e.size; i ++, firmware ++) {
        hm2_7i90_epp_write(bitfile_reverse_bits(*firmware), board);
    }

    end_time = rtapi_get_time();
    debug_epp = orig_debug_epp;


    // see if it worked
    if (hm2_7i90_epp_check_for_timeout(board)) {
        THIS_PRINT("EPP Timeout while sending firmware!\n");
        return -EIO;
    }


    //
    // brag about how fast it was
    //

    {
        uint32_t duration_ns;

        duration_ns = (uint32_t)(end_time - start_time);

        if (duration_ns != 0) {
            THIS_INFO(
                "%zu bytes of firmware sent (%u KB/s)\n",
                bitfile->e.size,
                (uint32_t)(((double)bitfile->e.size / ((double)duration_ns / (double)(1000 * 1000 * 1000))) / 1024)
            );
        }
    }


    return 0;
}




// return 0 if the board has been reset, -errno if not
int hm2_7i90_reset(hm2_lowlevel_io_t *this) {
    hm2_7i90_t *board = this->private;
    uint8_t byte;


    //
    // this resets the FPGA *only* if it's currently configured with the
    // HostMot2 or GPIO firmware
    //

    hm2_7i90_epp_addr16(0x7F7F, board);
    hm2_7i90_epp_write(0x5A, board);
    hm2_7i90_epp_addr16(0x7F7F, board);
    hm2_7i90_epp_write(0x5A, board);


    //
    // this code resets the FPGA *only* if the CPLD is in charge of the
    // parallel port
    //

    // select the control register
    hm2_7i90_epp_addr8(1, board);

    // bring the Spartan3's PROG_B line low for 1 us (the specs require 300-500 ns or longer)
    hm2_7i90_epp_write(0x00, board);
    hm2_7i90_nanosleep(1000);

    // bring the Spartan3's PROG_B line high and wait for 2 ms before sending firmware (required by spec)
    hm2_7i90_epp_write(0x01, board);
    hm2_7i90_nanosleep(2 * 1000 * 1000);

    // make sure the FPGA is not asserting its /DONE bit
    byte = hm2_7i90_epp_read(board);
    if ((byte & 0x01) != 0) {
        LL_PRINT("/DONE is not low after CPLD reset!\n");
        return -EIO;
    }

    return 0;
}




//
// setup and cleanup code
//


static void hm2_7i90_cleanup(void) {
    int i;

    // NOTE: hal_malloc() doesn't have a matching free

    for (i = 0; i < num_boards; i ++) {
        hm2_lowlevel_io_t *this = &board[i].llio;
        THIS_PRINT("releasing board\n");
        hm2_unregister(this);
        hal_parport_release(&board[i].port);
    }
}


static int hm2_7i90_setup(void) {
    int i;

    LL_PRINT("loading HostMot2 Mesa 7i90 driver version %s\n", HM2_7I90_VERSION);

    // zero the board structs
    memset(board, 0, HM2_7I90_MAX_BOARDS * sizeof(hm2_7i90_t));
    num_boards = 0;

    for (i = 0; i < HM2_7I90_MAX_BOARDS; i ++) {
        if (ioaddr[i] < 0) break;

        hm2_lowlevel_io_t *this;
        int r;

        board[i].epp_wide = epp_wide[i];

        //
        // claim the I/O regions for the parport
        //

        r = hal_parport_get(comp_id, &board[i].port,
                ioaddr[i], ioaddr_hi[i], PARPORT_MODE_EPP);
        if(r < 0)
            return r;

        // set up the parport for EPP
        if(board[i].port.base_hi) {
            rtapi_outb(0x94, board[i].port.base_hi + HM2_7I90_ECP_CONTROL_HIGH_OFFSET); // select EPP mode in ECR
        }

        //
        // when we get here, the parallel port is in epp mode and ready to go
        //

        // select the device and tell it to make itself ready for io
        hm2_7i90_epp_write_control(0x04, &board[i]);  // set control lines and input mode
        hm2_7i90_epp_clear_timeout(&board[i]);

        rtapi_snprintf(board[i].llio.name, sizeof(board[i].llio.name), "%s.%d", HM2_LLIO_NAME, i);
        board[i].llio.comp_id = comp_id;

        board[i].llio.read = hm2_7i90_read;
        board[i].llio.write = hm2_7i90_write;
        board[i].llio.program_fpga = hm2_7i90_program_fpga;
        board[i].llio.reset = hm2_7i90_reset;

        board[i].llio.num_ioport_connectors = 3;
        board[i].llio.pins_per_connector = 24;
        board[i].llio.ioport_connector_name[0] = "P1";
        board[i].llio.ioport_connector_name[1] = "P2";
        board[i].llio.ioport_connector_name[2] = "P3";
        board[i].llio.num_leds = 2;
        board[i].llio.private = &board[i];

        this = &board[i].llio;




        //  select CPLD data register
        hm2_7i90_epp_addr8(0, &board[i]);

        if (hm2_7i90_epp_read(&board[i]) & 0x01) {
            board[i].llio.fpga_part_number = "3s400tq144";
        } else {
            board[i].llio.fpga_part_number = "3s200tq144";
        }
        THIS_DBG("detected FPGA '%s'\n", board[i].llio.fpga_part_number);


        r = hm2_register(&board[i].llio, config[i]);
        if (r != 0) {
            hal_parport_release(&board[i].port);
            THIS_ERR(
                "board at (ioaddr=0x%04X, ioaddr_hi=0x%04X, epp_wide %s) not found!\n",
                board[i].port.base,
                board[i].port.base_hi,
                (board[i].epp_wide ? "ON" : "OFF"));
            return r;
        }

        THIS_PRINT(
            "board at (ioaddr=0x%04X, ioaddr_hi=0x%04X, epp_wide %s) found\n",
            board[i].port.base,
            board[i].port.base_hi,
            (board[i].epp_wide ? "ON" : "OFF")
        );

        num_boards ++;
    }

    return 0;
}


int rtapi_app_main(void) {
    int r = 0;

    comp_id = hal_init(HM2_LLIO_NAME);
    if (comp_id < 0) return comp_id;

    r = hm2_7i90_setup();
    if (r) {
        hm2_7i90_cleanup();
        hal_exit(comp_id);
    } else {
        hal_ready(comp_id);
    }

    return r;
}


void rtapi_app_exit(void) {
    hm2_7i90_cleanup();
    hal_exit(comp_id);
    LL_PRINT("driver unloaded\n");
}

