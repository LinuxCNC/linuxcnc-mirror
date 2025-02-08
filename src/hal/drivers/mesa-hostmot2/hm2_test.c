
//
//    Copyright (C) 2007-2008 Sebastian Kuzminsky
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


//
//  This driver behaves like a HostMot2 "low-level I/O" driver, but
//  provides unchanging, compiled-in, information.  It runs without any
//  special hardware plugged in.  Its job is to provide a test-pattern to
//  verify that the hostmot2 driver functions as it ought.
//


#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"

#include "hal.h"

#include "hostmot2.h"
#include "hostmot2-lowlevel.h"
#include "hm2_test.h"


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Kuzminsky");
MODULE_DESCRIPTION("Test pattern for the hostmot2 driver, does not talk to any hardware");


static char *config[HM2_TEST_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(config, HM2_TEST_MAX_BOARDS, "config string for the AnyIO boards (see hostmot2(9) manpage)");


int test_pattern = 0;
RTAPI_MP_INT(test_pattern, "The test pattern to show to the hostmot2 driver.");


static int comp_id;

static hm2_test_t board[1];




//
// Functions for initializing the register file on the pretend hm2 board.
//

static void set8(hm2_test_t *me, uint16_t addr, uint8_t val) {
    me->test_pattern.tp8[addr] = val;
}

static void set32(hm2_test_t *me, uint16_t addr, uint32_t val) {
    me->test_pattern.tp32[addr/4] = val;
}


// 
// these are the "low-level I/O" functions exported up
//


static int hm2_test_read(hm2_lowlevel_io_t *this, rtapi_u32 addr, void *buffer, int size) {
    hm2_test_t *me = this->private;
    memcpy(buffer, &me->test_pattern.tp8[addr], size);
    return 1;  // success
}


static int hm2_test_write(hm2_lowlevel_io_t *this, rtapi_u32 addr, const void *buffer, int size) {
    (void)this;
    (void)addr;
    (void)buffer;
    (void)size;
    return 1;  // success
}


static int hm2_test_program_fpga(hm2_lowlevel_io_t *this, const bitfile_t *bitfile) {
    (void)this;
    (void)bitfile;
    return 0;
}


static int hm2_test_reset(hm2_lowlevel_io_t *this) {
    (void)this;
    return 0;
}




int rtapi_app_main(void) {
    hm2_test_t *me;
    hm2_lowlevel_io_t *this;
    int r = 0;

    LL_ERR("loading HostMot2 test driver with test pattern %d\n", test_pattern);

    comp_id = hal_init(HM2_LLIO_NAME);
    if (comp_id < 0) return comp_id;

    me = &board[0];

    this = &me->llio;
    memset(this, 0, sizeof(hm2_lowlevel_io_t));

    me->llio.num_ioport_connectors = 1;
    me->llio.pins_per_connector = 24;
    me->llio.ioport_connector_name[0] = "P99";

    switch (test_pattern) {

        // 
        // this one has nothing
        // 

        case 0: {
            break;
        }


        // 
        // this one has a good IO Cookie, but that's it
        // 

        case 1: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            break;
        }


        // 
        // this one has a good IO Cookie and Config Name
        // the idrom offset is 0, and there's nothing there
        // 

        case 2: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            break;
        }


        // 
        // good IO Cookie, Config Name, and IDROM Type
        // the IDROM offset is the usual, 0x400, and there's an invalid IDROM type there
        // 

        case 3: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');

            // put the IDROM at 0x400, where it usually lives
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400);

            // bad idrom type
            set32(me, 0x400, 0x1234);

            break;
        }


        // 
        // good IO Cookie, Config Name, and IDROM Type
        // the IDROM offset is the usual, 0x400, and there's a good IDROM type there
        // but the portwidth is 0
        // 

        case 4: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400); // put the IDROM at 0x400, where it usually lives
            set32(me, 0x400, 2); // standard idrom type
            break;
        }


        // 
        // good IO Cookie, Config Name, and IDROM Type
        // the IDROM offset is the usual, 0x400, and there's a good IDROM type there
        // but the portwidth is 29 which is bogus
        // 

        case 5: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400); // put the IDROM at 0x400, where it usually lives
            set32(me, 0x400, 2); // standard idrom type

            // bad PortWidth
            set32(me, 0x424, 29);

            break;
        }


        // 
        // good IO Cookie, Config Name, and IDROM Type
        // the IDROM offset is the usual, 0x400, and there's a good IDROM type there
        // good PortWidth
        // 

        case 6: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400); // put the IDROM at 0x400, where it usually lives
            set32(me, 0x400, 2); // standard idrom type

            // good PortWidth
            set32(me, 0x424, 24);

            break;
        }


        // 
        // good IO Cookie, Config Name, and IDROM Type
        // the IDROM offset is the usual, 0x400, and there's a good IDROM type there
        // good PortWidth, but problematic IOPorts and IOWidth
        // 

        case 7: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400); // put the IDROM at 0x400, where it usually lives
            set32(me, 0x400, 2); // standard idrom type

            // good PortWidth = 24, which is standard
            set32(me, 0x424, 24);

            // IOPorts = 1
            set32(me, 0x41c, 1);

            // IOWidth = 99 (!= IOPorts * PortWidth)
            set32(me, 0x420, 99);

            break;
        }


        // 
        // good IO Cookie, Config Name, and IDROM Type
        // the IDROM offset is the usual, 0x400, and there's a good IDROM type there
        // good PortWidth, but IOPorts doesn't match what the llio said
        // 

        case 8: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400); // put the IDROM at 0x400, where it usually lives
            set32(me, 0x400, 2); // standard idrom type
            set32(me, 0x424, 24); // PortWidth = 24

            // IOPorts = 2 (!= what the llio said)
            set32(me, 0x41c, 2);

            // IOWidth == IOPorts * PortWidth)
            set32(me, 0x420, 48);

            break;
        }


        // 
        // good IO Cookie, Config Name, and IDROM Type
        // the IDROM offset is the usual, 0x400, and there's a good IDROM type there
        // good PortWidth, IOPorts, and IOWidth
        // but the clocks are bad
        // 

        case 9: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400); // put the IDROM at 0x400, where it usually lives
            set32(me, 0x400, 2); // standard idrom type
            set32(me, 0x424, 24); // PortWidth = 24

            // IOWidth = (IOPorts * PortWidth)
            set32(me, 0x41c, 1);
            set32(me, 0x420, 24);

            // ClockLow = 12345
            set32(me, 0x428, 12345);

            break;
        }


        // 
        // good IO Cookie, Config Name, and IDROM Type
        // the IDROM offset is the usual, 0x400, and there's a good IDROM type there
        // good PortWidth, IOPorts, and IOWidth
        // but the clocks are bad
        // 

        case 10: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400); // put the IDROM at 0x400, where it usually lives
            set32(me, 0x400, 2); // standard idrom type
            set32(me, 0x41c, 1);  // IOPorts = 1
            set32(me, 0x420, 24); // IOWidth = (IOPorts * PortWidth)
            set32(me, 0x424, 24); // PortWidth = 24

            // ClockLow = 2e6
            set32(me, 0x428, 2e6);

            // ClockHigh = 0
            set32(me, 0x42c, 0);

            break;
        }


        // 
        // good IO Cookie, Config Name, and IDROM Type
        // the IDROM offset is the usual, 0x400, and there's a good IDROM type there
        // good PortWidth, IOPorts, IOWidth, and clocks
        // 
        // The problem with this register file is that the Pin Descriptor
        // array contains no valid PDs, though the IDROM advertised 144 pins.
        //

        case 11: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400); // put the IDROM at 0x400, where it usually lives
            set32(me, 0x400, 2); // standard idrom type

            // normal offset to Module Descriptors
            set32(me, 0x404, 64);

            // unusual offset to PinDescriptors
            set32(me, 0x408, 0x1C0);

            // IOPorts
            set32(me, 0x41c, 6);

            // IOWidth
            set32(me, 0x420, 6*24);

            // PortWidth
            set32(me, 0x424, 24);

            // ClockLow = 2e6
            set32(me, 0x428, 2e6);

            // ClockHigh = 2e7
            set32(me, 0x42c, 2e7);

            me->llio.num_ioport_connectors = 6;
            me->llio.ioport_connector_name[0] = "P4";
            me->llio.ioport_connector_name[1] = "P5";
            me->llio.ioport_connector_name[2] = "P6";
            me->llio.ioport_connector_name[3] = "P9";
            me->llio.ioport_connector_name[4] = "P8";
            me->llio.ioport_connector_name[5] = "P7";

            break;
        }


        //
        // good IO Cookie, Config Name, and IDROM Type
        // the IDROM offset is the usual, 0x400, and there's a good IDROM type there
        // good PortWidth, IOWidth, and clocks
        // but there are no IOPorts instances according to the MDs
        // (this is the case with a firmware Jeff made for testing an RNG circuit)
        //

        case 12: {
            int num_io_pins = 24;
            int pd_index;

            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400); // put the IDROM at 0x400, where it usually lives
            set32(me, 0x400, 2); // standard idrom type

            // normal offset to Module Descriptors
            set32(me, 0x404, 64);

            // normal offset to PinDescriptors
            set32(me, 0x408, 0x200);

            // IOPorts
            set32(me, 0x41c, 1);

            // IOWidth
            set32(me, 0x420, num_io_pins);

            // PortWidth
            set32(me, 0x424, 24);

            // ClockLow = 2e6
            set32(me, 0x428, 2e6);

            // ClockHigh = 2e7
            set32(me, 0x42c, 2e7);

            me->llio.num_ioport_connectors = 1;
            me->llio.ioport_connector_name[0] = "P3";

            // make a bunch of valid Pin Descriptors
            for (pd_index = 0; pd_index < num_io_pins; pd_index ++) {
                set8(me, 0x600 + (pd_index * 4) + 0, 0);               // SecPin (byte) = Which pin of secondary function connects here eg: A,B,IDX.  Output pins have bit 7 = '1'
                set8(me, 0x600 + (pd_index * 4) + 1, 0);               // SecTag (byte) = Secondary function type (PWM,QCTR etc).  Same as module GTag
                set8(me, 0x600 + (pd_index * 4) + 2, 0);               // SecUnit (byte) = Which secondary unit or channel connects here
                set8(me, 0x600 + (pd_index * 4) + 3, HM2_GTAG_IOPORT); // PrimaryTag (byte) = Primary function tag (normally I/O port)
            }

            break;
        }


        // this board has a non-standard (ie, non-24) number of pins per connector, but the idrom does not match that
        case 13: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400); // put the IDROM at 0x400, where it usually lives
            set32(me, 0x400, 2); // standard idrom type

            // default PortWidth
            set32(me, 0x424, 24);

            // unusual number of pins per connector
            me->llio.pins_per_connector = 5;

            break;
        }


        // 
        // good IO Cookie, Config Name, and IDROM Type
        // the IDROM offset is the usual, 0x400, and there's a good IDROM type there
        // good but unusual (non-24) PortWidth
        // 

        case 14: {
            set32(me, HM2_ADDR_IOCOOKIE, HM2_IOCOOKIE);
            set8(me, HM2_ADDR_CONFIGNAME+0, 'H');
            set8(me, HM2_ADDR_CONFIGNAME+1, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+2, 'S');
            set8(me, HM2_ADDR_CONFIGNAME+3, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+4, 'M');
            set8(me, HM2_ADDR_CONFIGNAME+5, 'O');
            set8(me, HM2_ADDR_CONFIGNAME+6, 'T');
            set8(me, HM2_ADDR_CONFIGNAME+7, '2');
            set32(me, HM2_ADDR_IDROM_OFFSET, 0x400); // put the IDROM at 0x400, where it usually lives
            set32(me, 0x400, 2); // standard idrom type

            // good but unusual PortWidth
            set32(me, 0x424, 37);
            me->llio.pins_per_connector = 37;

            break;
        }

        default: {
            LL_ERR("unknown test pattern %d", test_pattern); 
            return -ENODEV;
        }
    }


    rtapi_snprintf(me->llio.name, sizeof(me->llio.name), "hm2_test.0");

    me->llio.fpga_part_number = "none";

    me->llio.program_fpga = hm2_test_program_fpga;
    me->llio.reset = hm2_test_reset;

    me->llio.comp_id = comp_id;
    me->llio.private = me;

    me->llio.threadsafe = 1;

    me->llio.read = hm2_test_read;
    me->llio.write = hm2_test_write;

    r = hm2_register(&board->llio, config[0]);
    if (r != 0) {
        THIS_ERR("hm2_test fails HM2 registration\n");
        return -EIO;
    }

    THIS_PRINT("initialized hm2 test-pattern %d\n", test_pattern);

    hal_ready(comp_id);
    return 0;
}


void rtapi_app_exit(void) {
    hm2_test_t *me = &board[0];

    hm2_unregister(&me->llio);

    LL_PRINT("driver unloaded\n");
    hal_exit(comp_id);
}

