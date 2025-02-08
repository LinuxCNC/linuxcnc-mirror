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
// This is a driver for the Hostmot2 outm module.
//
// Register map:
//
// OutM: Simple output module to allow renumbering pins and simplifying hal setup
// both have debounce filtering with an option of long or
// short time on a per input basis. Up to 4 MPG encoder counters are provided
//
// 0xB000 output register
// Register bits 0..N map into output bits 0..N
// 



#include <rtapi_slab.h>

#include "rtapi.h"
#include "hal.h"

#include "hal/drivers/mesa-hostmot2/hostmot2.h"


int hm2_outm_parse_md(hostmot2_t *hm2, int md_index) {
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    int r;

    if (hm2->outm.num_instances != 0) {
        HM2_ERR(
            "found duplicate Module Descriptor for %s (inconsistent firmware), not loading driver\n",
            hm2_get_general_function_name(md->gtag)
        );
        return -EINVAL;
    }

    if (hm2->config.num_outms > md->instances) {
        HM2_ERR(
            "config.num_outms=%d, but only %d are available, not loading driver\n",
            hm2->config.num_outms,
            md->instances
        );
        return -EINVAL;
    }

    if (hm2->config.num_outms == 0) {
        return 0;
    }


    //
    // Looks good, start initializing.
    //

    if (hm2->config.num_outms == -1) {
        hm2->outm.num_instances = md->instances;
    } else {
        hm2->outm.num_instances = hm2->config.num_outms;
    }

    hm2->outm.clock_freq = md->clock_freq;
    hm2->outm.version = md->version;

    hm2->outm.instance = (hm2_outm_instance_t *)hal_malloc(hm2->outm.num_instances * sizeof(hm2_outm_instance_t));
    if (hm2->outm.instance == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }

    // Set all '.out' HAL pins to NULL, we'll initialize to non-NULL
    // the ones that have Pin Descriptors below.
    {
        int inst;
        for (inst = 0; inst < hm2->outm.num_instances; inst ++) {
            unsigned out;
            for (
                out = 0;
                out < sizeof(hm2->outm.instance[0].hal.pin.out)/sizeof(hm2->outm.instance[0].hal.pin.out[0]);
                out ++
            ) {
                hm2->outm.instance[inst].hal.pin.out[out] = NULL;
            }
        }
    }

    hm2->outm.data_addr = md->base_address + (0 * md->register_stride);



    r = hm2_register_tram_write_region(hm2, hm2->outm.data_addr, (hm2->outm.num_instances * sizeof(rtapi_u32)), &hm2->outm.data_reg);
    if (r < 0) {
        HM2_ERR("error registering tram write region for outm Data register (%d)\n", r);
        goto fail0;
    }

    //
    // Export to HAL.
    //

    {
        int i;
        char name[HAL_NAME_LEN + 1];

        for (i = 0; i < hm2->outm.num_instances; i ++) {
            int j = 0;
            int outm_number;
            for (j = 0; j < hm2->num_pins; j++){
                if (hm2->pin[j].sec_tag == HM2_GTAG_OUTM && hm2->pin[j].sec_unit == i) {
                    if ((hm2->pin[j].sec_pin & 0x80) != 0x80) {
                        HM2_ERR("Pin Descriptor %d has an outm pin that's not an output!\n", j);
                        r = -EINVAL;
                        goto fail0;
                    }
                    outm_number = (hm2->pin[j].sec_pin & 0x7f) - 1;
                    if (outm_number >= 32) {
                        HM2_ERR("Pin Descriptor %d has invalid secondary pin number %d for outm module!\n", j, outm_number);
                        r = -EINVAL;
                        goto fail0;
                    }
                    rtapi_snprintf(name, sizeof(name), "%s.outm.%02d.out-%02d", hm2->llio->name, i, outm_number);
                    r = hal_pin_bit_new(name, HAL_IN, &(hm2->outm.instance[i].hal.pin.out[outm_number]), hm2->llio->comp_id);
                    if (r < 0) {
                        HM2_ERR("error adding pin '%s', aborting\n", name);
                        r = -ENOMEM;
                        goto fail0;
                    }
                    rtapi_snprintf(name, sizeof(name), "%s.outm.%02d.invert-%02d", hm2->llio->name, i, outm_number);
                    r = hal_pin_bit_new(name, HAL_IN, &(hm2->outm.instance[i].hal.pin.invert[outm_number]), hm2->llio->comp_id);
                    if (r < 0) {
                        HM2_ERR("error adding pin '%s', aborting\n", name);
                        r = -ENOMEM;
                        goto fail0;
                    }
                    
                }
            }
        }
     }

    //
    // Set outm output  values to 0, this is needed until
    // the Pin directions are set up.
    //
    // We can't use data_reg here, because it's in TRAM and TRAM hasn't
    // been allocated by the Hostmot2 main driver yet.
    //
    {
        int i;
        for (i = 0; i < hm2->outm.num_instances; i ++) {
            int pin;
            rtapi_u32 zero = 0;


            for (pin = 0; pin < 32; pin ++) {
                if (hm2->outm.instance[i].hal.pin.out[pin] != NULL) {
                    *hm2->outm.instance[i].hal.pin.out[pin] = 0;
                    *hm2->outm.instance[i].hal.pin.invert[pin] = 0;
                }
            }		 
            hm2->llio->write(hm2->llio, hm2->outm.data_addr + (i * md->instance_stride), &zero, sizeof(zero));
        }
    }

    return hm2->outm.num_instances;

fail0:
    hm2->outm.num_instances = 0;
    return r;
}


void hm2_outm_cleanup(hostmot2_t *hm2) {
    if (hm2->outm.num_instances <= 0) return;
}



void hm2_outm_force_write(hostmot2_t *hm2) {
    int size;
    int i;

    if (hm2->outm.num_instances <= 0) {
        return;
    }

    // Set register values from HAL pin values.
    for (i = 0; i < hm2->outm.num_instances; i ++) {
        int pin;

        hm2->outm.data_reg[i] = 0;
        for (pin = 0; pin < 32; pin ++) {
            if (hm2->outm.instance[i].hal.pin.out[pin] != NULL) {
                hm2->outm.data_reg[i] |= *hm2->outm.instance[i].hal.pin.out[pin] << pin;
                hm2->outm.data_reg[i] ^= *hm2->outm.instance[i].hal.pin.invert[pin] << pin;
            }
        }
    }

    size = hm2->outm.num_instances * sizeof(rtapi_u32);

    // Write register values to board.

    hm2->llio->write(hm2->llio, hm2->outm.data_addr, hm2->outm.data_reg, size);

    // Cache written-out register values.
    for (i = 0; i < hm2->outm.num_instances; i ++) {
        hm2->outm.instance[i].written_data = hm2->outm.data_reg[i];
    }
}




void hm2_outm_prepare_tram_write(hostmot2_t *hm2) {
    int i;

    // Set register values from HAL pin values.
    for (i = 0; i < hm2->outm.num_instances; i ++) {
        int pin;

        hm2->outm.data_reg[i] = 0;
        for (pin = 0; pin < 32; pin ++) {
            if (hm2->outm.instance[i].hal.pin.out[pin] != NULL) {
                hm2->outm.data_reg[i] |= *hm2->outm.instance[i].hal.pin.out[pin] << pin;
                hm2->outm.data_reg[i] ^= *hm2->outm.instance[i].hal.pin.invert[pin] << pin;
            }
        }
        if (hm2->outm.data_reg[i] != hm2->outm.instance[i].written_data) {
            hm2->outm.instance[i].written_data = hm2->outm.data_reg[i];
        }
    }
}


void hm2_outm_print_module(hostmot2_t *hm2) {
    int i;
    if (hm2->outm.num_instances <= 0) return;
    HM2_PRINT("outms: %d\n", hm2->outm.num_instances);
    HM2_PRINT("    clock_frequency: %d Hz (%s MHz)\n", hm2->outm.clock_freq, hm2_hz_to_mhz(hm2->outm.clock_freq));
    HM2_PRINT("    version: %d\n", hm2->outm.version);
    HM2_PRINT("    data_addr: 0x%04X\n", hm2->outm.data_addr);

    for (i = 0; i < hm2->outm.num_instances; i ++) {
        HM2_PRINT("    instance %d:\n", i);
        HM2_PRINT("        data_reg = 0x%08X\n", hm2->outm.data_reg[i]);

    }
}
