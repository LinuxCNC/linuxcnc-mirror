//
// Copyright (C) 2018 Sebastian Kuzminsky
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//
//
// This is a driver for the Hostmot2 SSR module.
//
// Register map:
//
// XfrmrOut: The xfrmrout module is used for isolated outputs using a
// transformer/rectifier to drive a MOSFET gate.
//
// 0x7D00  R/W  XfrmrOutData  1 to 32 bits of output data per module (1=on)
//     Bits 0..31 active high data control
//
// 0x7E00  XfrmrOutRate  drive frequency setting divisor and enable
//     Bits 0..11    Rate divisor sets frequency out
//                   frequency=(ClockLow/(Divisor+2))/2
//                                 (1 MHz is fine for all Mesa hardware)
//     Bit 12        Enable bit (high to enable) Should be enabled only after I/O
//                   is setup
//


#include <rtapi_slab.h>

#include "rtapi.h"
#include "hal.h"

#include "hal/drivers/mesa-hostmot2/hostmot2.h"


int hm2_ssr_parse_md(hostmot2_t *hm2, int md_index) {
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    int r;

    if (hm2->ssr.num_instances != 0) {
        HM2_ERR(
            "found duplicate Module Descriptor for %s (inconsistent firmware), not loading driver\n",
            hm2_get_general_function_name(md->gtag)
        );
        return -EINVAL;
    }

    if (hm2->config.num_ssrs > md->instances) {
        HM2_ERR(
            "config.num_ssrs=%d, but only %d are available, not loading driver\n",
            hm2->config.num_ssrs,
            md->instances
        );
        return -EINVAL;
    }

    if (hm2->config.num_ssrs == 0) {
        return 0;
    }


    //
    // Looks good, start initializing.
    //

    if (hm2->config.num_ssrs == -1) {
        hm2->ssr.num_instances = md->instances;
    } else {
        hm2->ssr.num_instances = hm2->config.num_ssrs;
    }

    hm2->ssr.clock_freq = md->clock_freq;
    hm2->ssr.version = md->version;

    hm2->ssr.instance = (hm2_ssr_instance_t *)hal_malloc(hm2->ssr.num_instances * sizeof(hm2_ssr_instance_t));
    if (hm2->ssr.instance == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }

    // Set all '.out' HAL pins to NULL, we'll initialize to non-NULL
    // the ones that have Pin Descriptors below.
    {
        int inst;
        for (inst = 0; inst < hm2->ssr.num_instances; inst ++) {
            unsigned out;
            for (
                out = 0;
                out < sizeof(hm2->ssr.instance[0].hal.pin.out)/sizeof(hm2->ssr.instance[0].hal.pin.out[0]);
                out ++
            ) {
                hm2->ssr.instance[inst].hal.pin.out[out] = NULL;
            }
        }
    }

    hm2->ssr.data_addr = md->base_address + (0 * md->register_stride);
    hm2->ssr.rate_addr = md->base_address + (1 * md->register_stride);

    hm2->ssr.rate_reg = (rtapi_u32*)rtapi_kmalloc(hm2->ssr.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
    if (hm2->ssr.rate_reg == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }

    r = hm2_register_tram_write_region(hm2, hm2->ssr.data_addr, (hm2->ssr.num_instances * sizeof(rtapi_u32)), &hm2->ssr.data_reg);
    if (r < 0) {
        HM2_ERR("error registering tram write region for SSR Data register (%d)\n", r);
        goto fail1;
    }

    //
    // Export to HAL.
    //

    {
        int i;
        char name[HAL_NAME_LEN + 1];

        for (i = 0; i < hm2->ssr.num_instances; i ++) {
            rtapi_snprintf(name, sizeof(name), "%s.ssr.%02d.rate", hm2->llio->name, i);
            r = hal_pin_u32_new(name, HAL_IN, &(hm2->ssr.instance[i].hal.pin.rate), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                r = -ENOMEM;
                goto fail1;
            }

            {
                int j = 0;
                int ssr_number;
                for (j = 0; j < hm2->num_pins; j++){
                    if (hm2->pin[j].sec_tag == HM2_GTAG_SSR && hm2->pin[j].sec_unit == i) {
                        if ((hm2->pin[j].sec_pin & 0x80) != 0x80) {
                            HM2_ERR("Pin Descriptor %d has an SSR pin that's not an output!\n", j);
                            r = -EINVAL;
                            goto fail1;
                        }
                        ssr_number = (hm2->pin[j].sec_pin & 0x7f) - 1;
                        if (ssr_number == 31) {
                            // This pin is special, doesn't correspond
                            // to any pins on the connectors, and should
                            // not be show in HAL.
                            continue;
                        } else if (ssr_number >= 32) {
                            HM2_ERR("Pin Descriptor %d has invalid secondary pin number %d for SSR module!\n", j, ssr_number);
                            r = -EINVAL;
                            goto fail1;
                        }

                        rtapi_snprintf(name, sizeof(name), "%s.ssr.%02d.out-%02d", hm2->llio->name, i, ssr_number);
                        r = hal_pin_bit_new(name, HAL_IN, &(hm2->ssr.instance[i].hal.pin.out[ssr_number]), hm2->llio->comp_id);
                        if (r < 0) {
                            HM2_ERR("error adding pin '%s', aborting\n", name);
                            r = -ENOMEM;
                            goto fail1;
                        }
                        rtapi_snprintf(name, sizeof(name), "%s.ssr.%02d.invert-%02d", hm2->llio->name, i, ssr_number);
                        r = hal_pin_bit_new(name, HAL_IN, &(hm2->ssr.instance[i].hal.pin.invert[ssr_number]), hm2->llio->comp_id);
                        if (r < 0) {
                            HM2_ERR("error adding pin '%s', aborting\n", name);
                            r = -ENOMEM;
                            goto fail1;
                        }
                        
                    }
                }
            }

        }
    }

    //
    // Set SSR output and rate values to 0, this is needed until
    // the Pin directions are set up.
    //
    // We can't use data_reg here, because it's in TRAM and TRAM hasn't
    // been allocated by the Hostmot2 main driver yet.
    //
    {
        int i;
        for (i = 0; i < hm2->ssr.num_instances; i ++) {
            int pin;
            rtapi_u32 zero = 0;

            *hm2->ssr.instance[i].hal.pin.rate = 1000*1000;

            for (pin = 0; pin < 32; pin ++) {
                if (hm2->ssr.instance[i].hal.pin.out[pin] != NULL) {
                    *hm2->ssr.instance[i].hal.pin.out[pin] = 0;
                    *hm2->ssr.instance[i].hal.pin.invert[pin] = 0;
                }
            }		 
            hm2->llio->write(hm2->llio, hm2->ssr.rate_addr + (i * md->instance_stride), &zero, sizeof(zero));
            hm2->llio->write(hm2->llio, hm2->ssr.data_addr + (i * md->instance_stride), &zero, sizeof(zero));
        }
    }

    return hm2->ssr.num_instances;

fail1:
    rtapi_kfree(hm2->ssr.rate_reg);

fail0:
    hm2->ssr.num_instances = 0;
    return r;
}


void hm2_ssr_cleanup(hostmot2_t *hm2) {
    if (hm2->ssr.num_instances <= 0) return;
}


// Set all instances' `rate_reg` variables according to the `.rate` pin.
// Does *not* write the value to the FPGA.
static void hm2_ssr_compute_rate_regs(hostmot2_t *hm2) {
    int i;

    for (i = 0; i < hm2->ssr.num_instances; i ++) {
        rtapi_u32 reg;

        if (*hm2->ssr.instance[i].hal.pin.rate <= 0) {
            // Writing all bits zero to the Rate register clears bit 12,
            // the enable bit.
            reg = 0;
        } else {
            double rate = *hm2->ssr.instance[i].hal.pin.rate;

            if (*hm2->ssr.instance[i].hal.pin.rate < 25000) {
                rate = 25000;
            } else if (*hm2->ssr.instance[i].hal.pin.rate > (25*1000*1000)) {
                rate = 25*1000*1000;
            }

            // The `rate` variable has the desired frequency in Hz.
            // The rate register wants a Divisor, and gives a frequency
            // of (ClockLow/(Divisor+2))/2.  So Divisor = ClockLow/(2*Rate) - 2
            reg = (hm2->ssr.clock_freq / (2 * rate)) - 2;

            // 12 bits of resolution
            // The range is:
            //     reg = 0xfff, f = (100M/0x1001)/2 = 12.2 kHz
            //     reg = 0x000, f = (100M/0x2)/2 = 25 MHz
            //     reg = 0x030, f = (100M/0x32)/2 = 1 MHz
            if (reg > 0xfff) {
                reg = 0xfff;
            }

            reg &= 0xfff;

            // ... and the enable bit
            reg |= 0x1000;
        }
        hm2->ssr.rate_reg[i] = reg;
    }
}


void hm2_ssr_force_write(hostmot2_t *hm2) {
    int size;
    int i;

    if (hm2->ssr.num_instances <= 0) {
        return;
    }

    hm2_ssr_compute_rate_regs(hm2);

    // Set register values from HAL pin values.
    for (i = 0; i < hm2->ssr.num_instances; i ++) {
        int pin;

        hm2->ssr.data_reg[i] = 0;
        for (pin = 0; pin < 32; pin ++) {
            if (hm2->ssr.instance[i].hal.pin.out[pin] != NULL) {
                hm2->ssr.data_reg[i] |= *hm2->ssr.instance[i].hal.pin.out[pin] << pin;
                hm2->ssr.data_reg[i] ^= *hm2->ssr.instance[i].hal.pin.invert[pin] << pin;
            }
        }
    }

    size = hm2->ssr.num_instances * sizeof(rtapi_u32);

    // Write register values to board.
    hm2->llio->write(hm2->llio, hm2->ssr.rate_addr, hm2->ssr.rate_reg, size);
    hm2->llio->write(hm2->llio, hm2->ssr.data_addr, hm2->ssr.data_reg, size);

    // Cache written-out register values.
    for (i = 0; i < hm2->ssr.num_instances; i ++) {
        hm2->ssr.instance[i].written_rate = hm2->ssr.rate_reg[i];
        hm2->ssr.instance[i].written_data = hm2->ssr.data_reg[i];
    }
}


void hm2_ssr_write(hostmot2_t *hm2) {
    int i;

    hm2_ssr_compute_rate_regs(hm2);

    for (i = 0; i < hm2->ssr.num_instances; i ++) {
        if (hm2->ssr.rate_reg[i] != hm2->ssr.instance[i].written_rate) {
            hm2->llio->write(hm2->llio, hm2->ssr.rate_addr, &hm2->ssr.rate_reg[i], sizeof(hm2->ssr.rate_reg[i]));
            hm2->ssr.instance[i].written_rate = hm2->ssr.rate_reg[i];
        }
    }
}


void hm2_ssr_prepare_tram_write(hostmot2_t *hm2) {
    int i;

    // Set register values from HAL pin values.
    for (i = 0; i < hm2->ssr.num_instances; i ++) {
        int pin;

        hm2->ssr.data_reg[i] = 0;
        for (pin = 0; pin < 32; pin ++) {
            if (hm2->ssr.instance[i].hal.pin.out[pin] != NULL) {
                hm2->ssr.data_reg[i] |= *hm2->ssr.instance[i].hal.pin.out[pin] << pin;
                hm2->ssr.data_reg[i] ^= *hm2->ssr.instance[i].hal.pin.invert[pin] << pin;
            }
        }
        if (hm2->ssr.data_reg[i] != hm2->ssr.instance[i].written_data) {
            hm2->ssr.instance[i].written_data = hm2->ssr.data_reg[i];
        }
    }
}


void hm2_ssr_print_module(hostmot2_t *hm2) {
    int i;
    if (hm2->ssr.num_instances <= 0) return;
    HM2_PRINT("SSRs: %d\n", hm2->ssr.num_instances);
    HM2_PRINT("    clock_frequency: %d Hz (%s MHz)\n", hm2->ssr.clock_freq, hm2_hz_to_mhz(hm2->ssr.clock_freq));
    HM2_PRINT("    version: %d\n", hm2->ssr.version);
    HM2_PRINT("    data_addr: 0x%04X\n", hm2->ssr.data_addr);
    HM2_PRINT("    rate_addr: 0x%04X\n", hm2->ssr.rate_addr);
    for (i = 0; i < hm2->ssr.num_instances; i ++) {
        HM2_PRINT("    instance %d:\n", i);
        HM2_PRINT("        data_reg = 0x%08X\n", hm2->ssr.data_reg[i]);
        HM2_PRINT("        rate_reg = 0x%08X\n", hm2->ssr.rate_reg[i]);
    }
}
