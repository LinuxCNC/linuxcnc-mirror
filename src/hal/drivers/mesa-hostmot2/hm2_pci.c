
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


#include <rtapi_pci.h>
#include <rtapi_io.h>

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"

#include "hal.h"

#include "bitfile.h"
#include "hostmot2-lowlevel.h"
#include "hm2_pci.h"


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Kuzminsky");
MODULE_DESCRIPTION("Driver for HostMot2 on the 5i2[012345], 6i25, 4i6[589], and 3x20 Anything I/O boards from Mesa Electronics");
MODULE_SUPPORTED_DEVICE("Mesa-AnythingIO-5i20");  // FIXME


static char *config[HM2_PCI_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(config, HM2_PCI_MAX_BOARDS, "config string for the AnyIO boards (see hostmot2(9) manpage)");

static int comp_id;


// FIXME: should probably have a linked list of boards instead of an array
static hm2_pci_t hm2_pci_board[HM2_PCI_MAX_BOARDS];
static int num_boards = 0;
static int num_5i20 = 0;
static int num_5i21 = 0;
static int num_5i22 = 0;
static int num_5i23 = 0;
static int num_5i24 = 0;
static int num_5i25 = 0;
static int num_6i25 = 0;
static int num_4i65 = 0;
static int num_4i68 = 0;
static int num_4i69 = 0;
static int num_3x20 = 0;
static int failed_errno=0; // errno of last failed registration


static struct rtapi_pci_device_id hm2_pci_tbl[] = {

    // 5i20
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9030,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_5I20,
    },

   // 5i21
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9054,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_5I21,
    },

    // 4i65
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9030,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_4I65,
    },

    // 5i22-1.0M
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9054,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_5I22_10,
    },

    // 5i22-1.5M
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9054,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_5I22_15,
    },

    // 5i23
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9054,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_5I23,
    },

    // 5i24
    {
        .vendor =  HM2_PCI_VENDORID_MESA,
        .device = HM2_PCI_DEV_MESA5I24,
        .subvendor = HM2_PCI_VENDORID_MESA,
        .subdevice = HM2_PCI_SSDEV_5I24,
    },

    // 5i25
    {
        .vendor =  HM2_PCI_VENDORID_MESA,
        .device = HM2_PCI_DEV_MESA5I25,
        .subvendor = HM2_PCI_VENDORID_MESA,
        .subdevice = HM2_PCI_SSDEV_5I25,
    },

    // 5i25t
    {
        .vendor =  HM2_PCI_VENDORID_MESA,
        .device = HM2_PCI_DEV_MESA5I25T,
        .subvendor = HM2_PCI_VENDORID_MESA,
        .subdevice = HM2_PCI_SSDEV_5I25T,
    },

    // 6i25
    {
        .vendor =  HM2_PCI_VENDORID_MESA,
        .device = HM2_PCI_DEV_MESA6I25,
        .subvendor = HM2_PCI_VENDORID_MESA,
        .subdevice = HM2_PCI_SSDEV_6I25,
    },

    // 4i68 (old SSID)
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9054,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_4I68_OLD,
    },

    // 4i68 (new SSID)
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9054,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_4I68,
    },

   // 4i69-16
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9054,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_4I69_16,
    },

    // 4i69-25
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9054,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_4I69_25,
    },

    // 3X20-1.0M
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9056,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_3X20_10,
    },

    // 3X20-1.5M
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9056,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_3X20_15,
    },

    // 3X20-2.0M
    {
        .vendor = HM2_PCI_VENDORID_PLX,
        .device = HM2_PCI_DEV_PLX9056,
        .subvendor = HM2_PCI_VENDORID_PLX,
        .subdevice = HM2_PCI_SSDEV_3X20_20,
    },

    {0,},
};

MODULE_DEVICE_TABLE(pci, hm2_pci_tbl);




// 
// these are the "low-level I/O" functions exported up
//

static int hm2_pci_read(hm2_lowlevel_io_t *this, rtapi_u32 addr, void *buffer, int size) {
    hm2_pci_t *board = this->private;
    void *src = board->base + addr;

    while (size > 0) {
        *(rtapi_u32*)buffer = *(rtapi_u32*)src;
        src += 4;
        buffer += 4;
        size -=4;
    }

    return 1;  // success
}

static int hm2_pci_write(hm2_lowlevel_io_t *this, rtapi_u32 addr, const void *buffer, int size) {
    hm2_pci_t *board = this->private;
    void *dest = board->base + addr;

    while (size > 0) {
        *(rtapi_u32*)dest = *(rtapi_u32*)buffer;
        dest += 4;
        buffer += 4;
        size -=4;
    }

    return 1;  // success
}


static int hm2_plx9030_program_fpga(hm2_lowlevel_io_t *this, const bitfile_t *bitfile) {
    hm2_pci_t *board = this->private;
    rtapi_u32 status, control;

    // set /WRITE low for data transfer, and turn on LED
    status = rtapi_inl(board->ctrl_base_addr + CTRL_STAT_OFFSET);
    control = status & ~_WRITE_MASK & ~_LED_MASK;
    rtapi_outl(control, board->ctrl_base_addr + CTRL_STAT_OFFSET);

    // program the FPGA
    for (unsigned i = 0; i < bitfile->e.size; i ++) {
        rtapi_outb(bitfile_reverse_bits(bitfile->e.data[i]), board->data_base_addr);
    }

    // all bytes transferred, make sure FPGA is all set up now
    status = rtapi_inl(board->ctrl_base_addr + CTRL_STAT_OFFSET);
    if (!(status & _INIT_MASK)) {
	// /INIT goes low on CRC error
	THIS_ERR("FPGA asserted /INIT: CRC error\n");
        goto fail;
    }
    if (!(status & DONE_MASK)) {
	THIS_ERR("FPGA did not assert DONE\n");
	goto fail;
    }

    // turn off write enable and LED
    control = status | _WRITE_MASK | _LED_MASK;
    rtapi_outl(control, board->ctrl_base_addr + CTRL_STAT_OFFSET);

    return 0;


fail:
    // set /PROGRAM low (reset device), /WRITE high and LED off
    status = rtapi_inl(board->ctrl_base_addr + CTRL_STAT_OFFSET);
    control = status & ~_PROGRAM_MASK;
    control |= _WRITE_MASK | _LED_MASK;
    rtapi_outl(control, board->ctrl_base_addr + CTRL_STAT_OFFSET);
    return -EIO;
}


static int hm2_plx9030_reset(hm2_lowlevel_io_t *this) {
    hm2_pci_t *board = this->private;
    rtapi_u32 status;
    rtapi_u32 control;

    status = rtapi_inl(board->ctrl_base_addr + CTRL_STAT_OFFSET);

    // set /PROGRAM bit low to reset the FPGA
    control = status & ~_PROGRAM_MASK;

    // set /WRITE and /LED high (idle state)
    control |= _WRITE_MASK | _LED_MASK;

    // and write it back
    rtapi_outl(control, board->ctrl_base_addr + CTRL_STAT_OFFSET);

    // verify that /INIT and DONE went low
    status = rtapi_inl(board->ctrl_base_addr + CTRL_STAT_OFFSET);
    if (status & (DONE_MASK | _INIT_MASK)) {
	THIS_ERR(
            "FPGA did not reset: /INIT = %d, DONE = %d\n",
	    (status & _INIT_MASK ? 1 : 0),
            (status & DONE_MASK ? 1 : 0)
        );
	return -EIO;
    }

    // set /PROGRAM high, let FPGA come out of reset
    control = status | _PROGRAM_MASK;
    rtapi_outl(control, board->ctrl_base_addr + CTRL_STAT_OFFSET);

    // wait for /INIT to go high when it finishes clearing memory
    // This should take no more than 100uS.  If we assume each PCI read
    // takes 30nS (one PCI clock), that is 3300 reads.  Reads actually
    // take several clocks, but even at a microsecond each, 3.3mS is not
    // an excessive timeout value
    {
        int count = 3300;

        do {
            status = rtapi_inl(board->ctrl_base_addr + CTRL_STAT_OFFSET);
            if (status & _INIT_MASK) break;
        } while (count-- > 0);

        if (count == 0) {
            THIS_ERR("FPGA did not come out of /INIT\n");
            return -EIO;
        }
    }

    return 0;
}


// fix up LASxBRD READY if needed
static void hm2_plx9030_fixup_LASxBRD_READY(hm2_pci_t *board) {
    hm2_lowlevel_io_t *this = &board->llio;
    int offsets[] = { LAS0BRD_OFFSET, LAS1BRD_OFFSET, LAS2BRD_OFFSET, LAS3BRD_OFFSET };
    int i;

    for (i = 0; i < 4; i ++) {
        rtapi_u32 val;
        int addr = board->ctrl_base_addr + offsets[i];

        val = rtapi_inl(addr);
        if (!(val & LASxBRD_READY)) {
            THIS_INFO("LAS%dBRD #READY is off, enabling now\n", i);
            val |= LASxBRD_READY;
            rtapi_outl(val, addr);
        }
    }
}




static int hm2_plx9054_program_fpga(hm2_lowlevel_io_t *this, const bitfile_t *bitfile) {
    hm2_pci_t *board = this->private;
    unsigned i;
    rtapi_u32 status;

    // program the FPGA
    for (i = 0; i < bitfile->e.size; i ++) {
        rtapi_outb(bitfile_reverse_bits(bitfile->e.data[i]), board->data_base_addr);
    }

    // all bytes transferred, make sure FPGA is all set up now
    for (i = 0; i < DONE_WAIT_5I22; i++) {
        status = rtapi_inl(board->ctrl_base_addr + CTRL_STAT_OFFSET_5I22);
        if (status & DONE_MASK_5I22) break;
    }
    if (i >= DONE_WAIT_5I22) {
        THIS_ERR("Error: Not /DONE; programming not completed.\n");
        return -EIO;
    }

    return 0;
}


static int hm2_plx9054_reset(hm2_lowlevel_io_t *this) {
    hm2_pci_t *board = this->private;
    int i;
    rtapi_u32 status, control;

    // set GPIO bits to GPIO function
    status = rtapi_inl(board->ctrl_base_addr + CTRL_STAT_OFFSET_5I22);
    control = status | DONE_ENABLE_5I22 | _PROG_ENABLE_5I22;
    rtapi_outl(control, board->ctrl_base_addr + CTRL_STAT_OFFSET_5I22);

    // Turn off /PROGRAM bit and insure that DONE isn't asserted
    rtapi_outl(control & ~_PROGRAM_MASK_5I22, board->ctrl_base_addr + CTRL_STAT_OFFSET_5I22);

    status = rtapi_inl(board->ctrl_base_addr + CTRL_STAT_OFFSET_5I22);
    if (status & DONE_MASK_5I22) {
        // Note that if we see DONE at the start of programming, it's most
        // likely due to an attempt to access the FPGA at the wrong I/O
        // location.
        THIS_ERR("/DONE status bit indicates busy at start of programming\n");
        return -EIO;
    }

    // turn on /PROGRAM output bit
    rtapi_outl(control | _PROGRAM_MASK_5I22, board->ctrl_base_addr + CTRL_STAT_OFFSET_5I22);

    // Delay for at least 100 uS. to allow the FPGA to finish its reset
    // sequencing.  3300 reads is at least 100 us, could be as long as a
    // few ms
    for (i = 0; i < 3300; i++) {
        status = rtapi_inl(board->ctrl_base_addr + CTRL_STAT_OFFSET);
    }

    return 0;
}




// 
// misc internal functions
//


static int hm2_pci_probe(struct rtapi_pci_dev *dev, const struct rtapi_pci_device_id *id) {
    (void)id;
    int r;
    hm2_pci_t *board;
    hm2_lowlevel_io_t *this;


    if (num_boards >= HM2_PCI_MAX_BOARDS) {
        LL_PRINT("skipping AnyIO board at %s, this driver can only handle %d\n", rtapi_pci_name(dev), HM2_PCI_MAX_BOARDS);
        return -EINVAL;
    }

    // NOTE: this enables the board's BARs -- this fixes the Arty bug
    if (rtapi_pci_enable_device(dev)) {
        LL_PRINT("skipping AnyIO board at %s, failed to enable PCI device\n", rtapi_pci_name(dev));
        return failed_errno = -ENODEV;
    }


    board = &hm2_pci_board[num_boards];
    this = &board->llio;
    memset(this, 0, sizeof(hm2_lowlevel_io_t));

    switch (dev->subsystem_device) {
        case HM2_PCI_SSDEV_5I20: {
            LL_PRINT("discovered 5i20 at %s\n", rtapi_pci_name(dev));
            rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_5i20.%d", num_5i20);
            num_5i20 ++;
            board->llio.num_ioport_connectors = 3;
            board->llio.pins_per_connector = 24;
            board->llio.ioport_connector_name[0] = "P2";
            board->llio.ioport_connector_name[1] = "P3";
            board->llio.ioport_connector_name[2] = "P4";
            board->llio.fpga_part_number = "2s200pq208";
            board->llio.num_leds = 8;
            break;
        }

        case HM2_PCI_SSDEV_5I21: {
            LL_PRINT("discovered 5i21 at %s\n", rtapi_pci_name(dev));
            rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_5i21.%d", num_5i21);
            num_5i21 ++;
            board->llio.num_ioport_connectors = 2;
            board->llio.pins_per_connector = 32;
            board->llio.ioport_connector_name[0] = "P1";
            board->llio.ioport_connector_name[1] = "P1";
            board->llio.fpga_part_number = "3s400pq208";
            board->llio.num_leds = 8;
            break;
        }

        case HM2_PCI_SSDEV_4I65: {
            LL_PRINT("discovered 4i65 at %s\n", rtapi_pci_name(dev));
            rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_4i65.%d", num_4i65);
            num_4i65 ++;
            board->llio.num_ioport_connectors = 3;
            board->llio.pins_per_connector = 24;
            board->llio.ioport_connector_name[0] = "P1";
            board->llio.ioport_connector_name[1] = "P3";
            board->llio.ioport_connector_name[2] = "P4";
            board->llio.fpga_part_number = "2s200pq208";
            board->llio.num_leds = 8;
            break;
        }

        case HM2_PCI_SSDEV_5I22_10:
        case HM2_PCI_SSDEV_5I22_15: {
            if (dev->subsystem_device == HM2_PCI_SSDEV_5I22_10) {
                LL_PRINT("discovered 5i22-1.0M at %s\n", rtapi_pci_name(dev));
                board->llio.fpga_part_number = "3s1000fg320";
            } else {
                LL_PRINT("discovered 5i22-1.5M at %s\n", rtapi_pci_name(dev));
                board->llio.fpga_part_number = "3s1500fg320";
            }
            rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_5i22.%d", num_5i22);
            num_5i22 ++;
            board->llio.num_ioport_connectors = 4;
            board->llio.pins_per_connector = 24;
            board->llio.ioport_connector_name[0] = "P2";
            board->llio.ioport_connector_name[1] = "P3";
            board->llio.ioport_connector_name[2] = "P4";
            board->llio.ioport_connector_name[3] = "P5";
            board->llio.num_leds = 8;
            break;
        }

        case HM2_PCI_SSDEV_5I23: {
            LL_PRINT("discovered 5i23 at %s\n", rtapi_pci_name(dev));
            rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_5i23.%d", num_5i23);
            num_5i23 ++;
            board->llio.num_ioport_connectors = 3;
            board->llio.pins_per_connector = 24;
            board->llio.ioport_connector_name[0] = "P2";
            board->llio.ioport_connector_name[1] = "P3";
            board->llio.ioport_connector_name[2] = "P4";
            board->llio.fpga_part_number = "3s400pq208";
            board->llio.num_leds = 2;
            break;
        }

        case HM2_PCI_SSDEV_5I24: {
            LL_PRINT("discovered 5i24 at %s\n", rtapi_pci_name(dev));
            rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_5i24.%d", num_5i24);
            num_5i24 ++;
            board->llio.num_ioport_connectors = 3;
            board->llio.pins_per_connector = 24;
            board->llio.ioport_connector_name[0] = "P4";
            board->llio.ioport_connector_name[1] = "P3";
            board->llio.ioport_connector_name[2] = "P2";
            board->llio.fpga_part_number = "6slx16ftg256";
            board->llio.num_leds = 2;
            break;
        }

        case HM2_PCI_SSDEV_5I25:
        case HM2_PCI_SSDEV_6I25: {
            if (dev->subsystem_device == HM2_PCI_SSDEV_5I25) {
                LL_PRINT("discovered 5i25 at %s\n", rtapi_pci_name(dev));
                rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_5i25.%d", num_5i25);
                num_5i25 ++;
            } else {
                LL_PRINT("discovered 6i25 at %s\n", rtapi_pci_name(dev));
                rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_6i25.%d", num_6i25);
                num_6i25 ++;
            }
            board->llio.num_ioport_connectors = 2;
            board->llio.pins_per_connector = 17;
            board->llio.ioport_connector_name[0] = "P3";
            board->llio.ioport_connector_name[1] = "P2";
            board->llio.fpga_part_number = "6slx9tqg144";
            board->llio.num_leds = 2;
            break;
        }

        case HM2_PCI_SSDEV_5I25T:
        case HM2_PCI_SSDEV_6I25T: {
            if (dev->subsystem_device == HM2_PCI_SSDEV_5I25T) {
                LL_PRINT("discovered 5i25t at %s\n", rtapi_pci_name(dev));
                rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_5i25.%d", num_5i25);
                num_5i25 ++;
            } else {
                LL_PRINT("discovered 6i25t at %s\n", rtapi_pci_name(dev));
                rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_6i25.%d", num_6i25);
                num_6i25 ++;
            }
            board->llio.num_ioport_connectors = 2;
            board->llio.pins_per_connector = 17;
            board->llio.ioport_connector_name[0] = "P3";
            board->llio.ioport_connector_name[1] = "P2";
            board->llio.fpga_part_number = "t20f256";
            board->llio.num_leds = 2;
            break;
        }

        case HM2_PCI_SSDEV_4I68:
        case HM2_PCI_SSDEV_4I68_OLD: {
            if (dev->subsystem_device == HM2_PCI_SSDEV_4I68_OLD) {
                LL_PRINT("discovered OLD 4i68 at %s, please consider upgrading your EEPROM\n", rtapi_pci_name(dev));
            } else {
                LL_PRINT("discovered 4i68 at %s\n", rtapi_pci_name(dev));
            }
            rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_4i68.%d", num_4i68);
            num_4i68 ++;
            board->llio.num_ioport_connectors = 3;
            board->llio.pins_per_connector = 24;
            board->llio.ioport_connector_name[0] = "P1";
            board->llio.ioport_connector_name[1] = "P2";
            board->llio.ioport_connector_name[2] = "P4";
            board->llio.fpga_part_number = "3s400pq208";
            board->llio.num_leds = 4;
            break;
        }

        case HM2_PCI_SSDEV_4I69_16:
        case HM2_PCI_SSDEV_4I69_25: {
            if (dev->subsystem_device == HM2_PCI_SSDEV_4I69_16) {
                LL_PRINT("discovered 4I69-16 at %s\n", rtapi_pci_name(dev));
                board->llio.fpga_part_number = "6slx16ftg256";

            } else {
                LL_PRINT("discovered 4I69-25 at %s\n", rtapi_pci_name(dev));
                board->llio.fpga_part_number = "6slx25ftg256";
            }
            rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_4i69.%d", num_4i69);
            num_4i69 ++;
            board->llio.num_ioport_connectors = 3;
            board->llio.pins_per_connector = 24;
            board->llio.ioport_connector_name[0] = "P1";
            board->llio.ioport_connector_name[1] = "P3";
            board->llio.ioport_connector_name[2] = "P4";
            break;
        }

        case HM2_PCI_SSDEV_3X20_10:
        case HM2_PCI_SSDEV_3X20_15:
        case HM2_PCI_SSDEV_3X20_20: {
            if (dev->subsystem_device == HM2_PCI_SSDEV_3X20_10) {
                LL_PRINT("discovered 3x20-1.0M at %s\n", rtapi_pci_name(dev));
                board->llio.fpga_part_number = "3s1000fg456";
            } else if (dev->subsystem_device == HM2_PCI_SSDEV_3X20_15) {
                LL_PRINT("discovered 3x20-1.5M at %s\n", rtapi_pci_name(dev));
                board->llio.fpga_part_number = "3s1500fg456";
            } else {
                LL_PRINT("discovered 3x20-2.0M at %s\n", rtapi_pci_name(dev));
                board->llio.fpga_part_number = "3s2000fg456";
            }
            rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_3x20.%d", num_3x20);
            num_3x20 ++;
            board->llio.num_ioport_connectors = 6;
            board->llio.pins_per_connector = 24;
            board->llio.ioport_connector_name[0] = "P4";
            board->llio.ioport_connector_name[1] = "P5";
            board->llio.ioport_connector_name[2] = "P6";
            board->llio.ioport_connector_name[3] = "P9";
            board->llio.ioport_connector_name[4] = "P8";
            board->llio.ioport_connector_name[5] = "P7";
            board->llio.num_leds = 0;
            break;
        }

        default: {
            LL_ERR("unknown subsystem device id 0x%04x\n", dev->subsystem_device);
            return failed_errno = -ENODEV;
        }
    }


    switch (dev->device) {
        case HM2_PCI_DEV_PLX9030: {
            // get a hold of the IO resources we'll need later
            // FIXME: should request_region here
            board->ctrl_base_addr = rtapi_pci_resource_start(dev, 1);
            board->data_base_addr = rtapi_pci_resource_start(dev, 2);

            // BAR 5 is 64K mem (32 bit)
            board->len = rtapi_pci_resource_len(dev, dev->subsystem_device ? 5 : 3);
            board->base = rtapi_pci_ioremap_bar(dev, dev->subsystem_device ? 5 : 3);
            if (board->base == NULL) {
                THIS_ERR("could not map in FPGA address space\n");
                r = -ENODEV;
                goto fail0;
            }

            hm2_plx9030_fixup_LASxBRD_READY(board);

            board->llio.program_fpga = hm2_plx9030_program_fpga;
            board->llio.reset = hm2_plx9030_reset;
            break;
        }

        case HM2_PCI_DEV_PLX9056:
        case HM2_PCI_DEV_PLX9054: {
            // get a hold of the IO resources we'll need later
            // FIXME: should request_region here
            board->ctrl_base_addr = rtapi_pci_resource_start(dev, 1);
            board->data_base_addr = rtapi_pci_resource_start(dev, 2);

            // BAR 3 is 64K mem (32 bit)
            board->len = rtapi_pci_resource_len(dev, 3);
            board->base = rtapi_pci_ioremap_bar(dev, 3);
            if (board->base == NULL) {
                THIS_ERR("could not map in FPGA address space\n");
                r = -ENODEV;
                goto fail0;
            }

            board->llio.program_fpga = hm2_plx9054_program_fpga;
            board->llio.reset = hm2_plx9054_reset;

            break;
        }

        case HM2_PCI_DEV_MESA5I24:
        case HM2_PCI_DEV_MESA5I25:
        case HM2_PCI_DEV_MESA5I25T:
        case HM2_PCI_DEV_MESA6I25:
        case HM2_PCI_DEV_MESA6I25T: {
              // BAR 0 is 64K mem (32 bit)
            board->len = rtapi_pci_resource_len(dev, 0);
            board->base = rtapi_pci_ioremap_bar(dev, 0);
            if (board->base == NULL) {
                THIS_ERR("could not map in FPGA address space\n");
                r = -ENODEV;
                goto fail0;
            }
            break;
        }

        default: {
            THIS_ERR("unknown PCI Device ID 0x%04x\n", dev->device);
            r = -ENODEV;
            goto fail0;
        }
    }


    board->dev = dev;

    rtapi_pci_set_drvdata(dev, board);

    board->llio.comp_id = comp_id;
    board->llio.private = board;

    board->llio.threadsafe = 1;

    board->llio.read = hm2_pci_read;
    board->llio.write = hm2_pci_write;

    r = hm2_register(&board->llio, config[num_boards]);
    if (r != 0) {
        THIS_ERR("board fails HM2 registration\n");
        goto fail1;
    }

    THIS_PRINT("initialized AnyIO board at %s\n", rtapi_pci_name(dev));

    num_boards ++;
    return 0;


fail1:
    rtapi_pci_set_drvdata(dev, NULL);
    rtapi_iounmap(board->base);
    board->base = NULL;

fail0:
    rtapi_pci_disable_device(dev);
    return failed_errno = r;
}


static void hm2_pci_remove(struct rtapi_pci_dev *dev) {
    int i;

    for (i = 0; i < num_boards; i++) {
        hm2_pci_t *board = &hm2_pci_board[i];
        hm2_lowlevel_io_t *this = &board->llio;

        if (board->dev == dev) {
            THIS_PRINT("dropping AnyIO board at %s\n", rtapi_pci_name(dev));

            hm2_unregister(&board->llio);

            // Unmap board memory
            if (board->base != NULL) {
                rtapi_iounmap(board->base);
                board->base = NULL;
            }

            rtapi_pci_disable_device(dev);
            rtapi_pci_set_drvdata(dev, NULL);
            board->dev = NULL;
        }
    }
}


static struct rtapi_pci_driver hm2_pci_driver = {
	.name = HM2_LLIO_NAME,
	.id_table = hm2_pci_tbl,
	.probe = hm2_pci_probe,
	.remove = hm2_pci_remove,
};


int rtapi_app_main(void) {
    int r = 0;

    LL_PRINT("loading Mesa AnyIO HostMot2 driver version " HM2_PCI_VERSION "\n");

    comp_id = hal_init(HM2_LLIO_NAME);
    if (comp_id < 0) return comp_id;

    r = rtapi_pci_register_driver(&hm2_pci_driver);
    if (r != 0) {
        LL_ERR("error registering PCI driver\n");
        hal_exit(comp_id);
        return r;
    }

    if(failed_errno) {
	// at least one card registration failed
	hal_exit(comp_id);
	rtapi_pci_unregister_driver(&hm2_pci_driver);
	return failed_errno;
    }

    if(num_boards == 0) {
	// no cards were detected
	hal_exit(comp_id);
	rtapi_pci_unregister_driver(&hm2_pci_driver);
	return -ENODEV;
    }

    hal_ready(comp_id);
    return 0;
}


void rtapi_app_exit(void) {
    rtapi_pci_unregister_driver(&hm2_pci_driver);
    LL_PRINT("driver unloaded\n");
    hal_exit(comp_id);
}

