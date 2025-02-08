
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

#include <rtapi_slab.h>

#include "rtapi.h"
#include "rtapi_string.h"
#include "rtapi_math.h"

#include "hal.h"

#include "hal/drivers/mesa-hostmot2/hostmot2.h"




int hm2_ioport_parse_md(hostmot2_t *hm2, int md_index) {
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    int i, r;


    // 
    // some standard sanity checks
    //

    if (!hm2_md_is_consistent_or_complain(hm2, md_index, 0, 5, 4, 0x001F)) {
        HM2_ERR("inconsistent Module Descriptor!\n");
        return -EINVAL;
    }

    if (hm2->ioport.num_instances != 0) {
        HM2_ERR(
            "found duplicate Module Descriptor for %s (inconsistent firmware), not loading driver\n",
            hm2_get_general_function_name(md->gtag)
        );
        return -EINVAL;
    }


    // 
    // special sanity check for io_ports
    // 

    if (hm2->idrom.io_ports != md->instances) {
        HM2_ERR(
            "IDROM IOPorts is %d but MD IOPort NumInstances is %d, inconsistent firmware, aborting driver load\n",
            hm2->idrom.io_ports,
            md->instances
        );
        return -EINVAL;
    }

    hm2->ioport.num_instances = md->instances;


    hm2->ioport.clock_frequency = md->clock_freq;
    hm2->ioport.version = md->version;

    hm2->ioport.data_addr = md->base_address + (0 * md->register_stride);
    hm2->ioport.ddr_addr = md->base_address + (1 * md->register_stride);
    hm2->ioport.alt_source_addr = md->base_address + (2 * md->register_stride);
    hm2->ioport.open_drain_addr = md->base_address + (3 * md->register_stride);
    hm2->ioport.output_invert_addr = md->base_address + (4 * md->register_stride);

    r = hm2_register_tram_read_region(hm2, hm2->ioport.data_addr, (hm2->ioport.num_instances * sizeof(rtapi_u32)), &hm2->ioport.data_read_reg);
    if (r < 0) {
        HM2_ERR("error registering tram read region for IOPort Data register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_write_region(hm2, hm2->ioport.data_addr, (hm2->ioport.num_instances * sizeof(rtapi_u32)), &hm2->ioport.data_write_reg);
    if (r < 0) {
        HM2_ERR("error registering tram write region for IOPort Data register (%d)\n", r);
        goto fail0;
    }

    hm2->ioport.ddr_reg = (rtapi_u32 *)rtapi_kmalloc(hm2->ioport.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
    if (hm2->ioport.ddr_reg == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }

    // this one's not a real register
    hm2->ioport.written_ddr = (rtapi_u32 *)rtapi_kmalloc(hm2->ioport.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
    if (hm2->ioport.written_ddr == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail1;
    }

    hm2->ioport.alt_source_reg = (rtapi_u32 *)rtapi_kmalloc(hm2->ioport.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
    if (hm2->ioport.alt_source_reg == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail2;
    }

    hm2->ioport.open_drain_reg = (rtapi_u32 *)rtapi_kmalloc(hm2->ioport.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
    if (hm2->ioport.open_drain_reg == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail3;
    }

    // this one's not a real register
    hm2->ioport.written_open_drain = (rtapi_u32 *)rtapi_kmalloc(hm2->ioport.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
    if (hm2->ioport.written_open_drain == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail4;
    }

    hm2->ioport.output_invert_reg = (rtapi_u32 *)rtapi_kmalloc(hm2->ioport.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
    if (hm2->ioport.output_invert_reg == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail5;
    }

    // this one's not a real register
    hm2->ioport.written_output_invert = (rtapi_u32 *)rtapi_kmalloc(hm2->ioport.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
    if (hm2->ioport.written_output_invert == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail6;
    }


    //
    // initialize to all gpios, all inputs, not open drain, not output-inverted
    //

    for (i = 0; i < hm2->ioport.num_instances; i ++) {
        hm2->ioport.ddr_reg[i] = 0;                // all are inputs
        hm2->ioport.written_ddr[i] = 0;            // we're starting out in sync
        hm2->ioport.alt_source_reg[i] = 0;         // they're all gpios
        hm2->ioport.open_drain_reg[i] = 0;         // none are open drain
        hm2->ioport.written_open_drain[i] = 0;     // starting out in sync
        hm2->ioport.output_invert_reg[i] = 0;      // none are output-inverted
        hm2->ioport.written_output_invert[i] = 0;  // starting out in sync 
    }

    // we can't export this one to HAL yet, because some pins may be allocated to other modules

    return hm2->ioport.num_instances;


fail6:
    rtapi_kfree(hm2->ioport.output_invert_reg);

fail5:
    rtapi_kfree(hm2->ioport.written_open_drain);

fail4:
    rtapi_kfree(hm2->ioport.open_drain_reg);

fail3:
    rtapi_kfree(hm2->ioport.alt_source_reg);

fail2:
    rtapi_kfree(hm2->ioport.written_ddr);

fail1:
    rtapi_kfree(hm2->ioport.ddr_reg);

fail0:
    hm2->ioport.num_instances = 0;
    return r;
}




void hm2_ioport_cleanup(hostmot2_t *hm2) {
    if (hm2->ioport.num_instances <= 0) return;
    if (hm2->ioport.ddr_reg != NULL) rtapi_kfree(hm2->ioport.ddr_reg);
    if (hm2->ioport.written_ddr != NULL) rtapi_kfree(hm2->ioport.written_ddr);
    if (hm2->ioport.alt_source_reg != NULL) rtapi_kfree(hm2->ioport.alt_source_reg);
    if (hm2->ioport.open_drain_reg != NULL) rtapi_kfree(hm2->ioport.open_drain_reg);
    if (hm2->ioport.written_open_drain!= NULL) rtapi_kfree(hm2->ioport.written_open_drain);
    if (hm2->ioport.output_invert_reg != NULL) rtapi_kfree(hm2->ioport.output_invert_reg);
    if (hm2->ioport.written_output_invert != NULL) rtapi_kfree(hm2->ioport.written_output_invert);
}


static int do_alias(const char *orig_base, const char *alias_base,
        const char *suffix, int (*funct)(const char *, const char *)) {
    char orig_name[HAL_NAME_LEN];
    char alias_name[HAL_NAME_LEN];
    snprintf(orig_name, sizeof(orig_name), "%s%s", orig_base, suffix);
    snprintf(alias_name, sizeof(alias_name), "%s%s", alias_base, suffix);
    return funct(orig_name, alias_name);
}

static void count_instances(hostmot2_t *hm2, int pin, int *this_instance, int *total_instances) {
    int i;
    int num_instances = 0;
    for(i=0; i != hm2->num_pins; i++) {
	if(i == pin) *this_instance = num_instances;
        if(hm2->pin[i].gtag == hm2->pin[pin].gtag &&
		hm2->pin[i].sec_unit == hm2->pin[pin].sec_unit &&
		hm2->pin[i].sec_pin == hm2->pin[pin].sec_pin)
	    num_instances++;
    }
    *total_instances = num_instances;
}


int hm2_ioport_gpio_export_hal(hostmot2_t *hm2) {
    int r;
    int i;
    const char *gtag_name, *funct_name;

    for (i = 0; i < hm2->num_pins; i ++) {
        // all pins get *some* gpio HAL presence
        hm2->pin[i].instance = (hm2_gpio_instance_t *)hal_malloc(sizeof(hm2_gpio_instance_t));
        if (hm2->pin[i].instance == NULL) {
            HM2_ERR("out of memory!\n");
            return -ENOMEM;
        }


        //
        // all pins' values can be read; for pins used as outputs
        // (including special-purpose outputs), the output value is sampled.
        //

        // pins
        r = hal_pin_bit_newf(
            HAL_OUT,
            &(hm2->pin[i].instance->hal.pin.in),
            hm2->llio->comp_id,
            "%s.gpio.%03d.in",
            hm2->llio->name,
            i
        );
        if (r < 0) {
            HM2_ERR("error %d adding gpio pin, aborting\n", r);
            return -EINVAL;
        }

        r = hal_pin_bit_newf(
            HAL_OUT,
            &(hm2->pin[i].instance->hal.pin.in_not),
            hm2->llio->comp_id,
            "%s.gpio.%03d.in_not",
            hm2->llio->name,
            i
        );
        if (r < 0) {
            HM2_ERR("error %d adding gpio pin, aborting\n", r);
            return -EINVAL;
        }


        //
        // it's a full GPIO or it's an output for some other module
        //

        if (
            (hm2->pin[i].gtag == HM2_GTAG_IOPORT)
            || (hm2->pin[i].direction_at_start == HM2_PIN_DIR_IS_OUTPUT)
        ) {

            r = hal_param_bit_newf(
                HAL_RW,
                &(hm2->pin[i].instance->hal.param.invert_output),
                hm2->llio->comp_id,
                "%s.gpio.%03d.invert_output",
                hm2->llio->name,
                i
            );
            if (r < 0) {
                HM2_ERR("error %d adding gpio param, aborting\n", r);
                return -EINVAL;
            }

            r = hal_param_bit_newf(
                HAL_RW,
                &(hm2->pin[i].instance->hal.param.is_opendrain),
                hm2->llio->comp_id,
                "%s.gpio.%03d.is_opendrain",
                hm2->llio->name,
                i
            );
            if (r < 0) {
                HM2_ERR("error %d adding gpio param, aborting\n", r);
                return -EINVAL;
            }

            hm2->pin[i].instance->hal.param.invert_output = 0;
            hm2->pin[i].instance->hal.param.is_opendrain = 0;
        }


        //
        // it's a full GPIO
        //

        if (hm2->pin[i].gtag == HM2_GTAG_IOPORT) {

            r = hal_pin_bit_newf(
                HAL_IN,
                &(hm2->pin[i].instance->hal.pin.out),
                hm2->llio->comp_id,
                "%s.gpio.%03d.out",
                hm2->llio->name,
                i
            );
            if (r < 0) {
                HM2_ERR("error %d adding gpio pin, aborting\n", r);
                return -EINVAL;
            }

            *(hm2->pin[i].instance->hal.pin.out) = 0;

            // parameters
            r = hal_param_bit_newf(
                HAL_RW,
                &(hm2->pin[i].instance->hal.param.is_output),
                hm2->llio->comp_id,
                "%s.gpio.%03d.is_output",
                hm2->llio->name,
                i
            );
            if (r < 0) {
                HM2_ERR("error %d adding gpio param, aborting\n", r);
                return -EINVAL;
            }

            hm2->pin[i].instance->hal.param.is_output = 0;
        }

        // it's an output of some other module
        if(hm2->pin[i].gtag != HM2_GTAG_IOPORT &&
            hm2->pin[i].direction_at_start == HM2_PIN_DIR_IS_OUTPUT &&
            (gtag_name = hm2_get_general_function_hal_name(hm2->pin[i].gtag)) &&
            (funct_name = hm2_get_pin_secondary_hal_name(&hm2->pin[i])))
        {
	    int this_instance = -1;
	    int total_instances = 0;
	    count_instances(hm2, i, &this_instance, &total_instances);
            char orig_base[HAL_NAME_LEN];
            char alias_base[HAL_NAME_LEN];
            size_t ret = snprintf(orig_base, sizeof(orig_base),
                "%s.gpio.%03d",
                hm2->llio->name,
                i);
            if (ret >= sizeof(orig_base)){
		HM2_ERR("string truncation\n");
                return -EINVAL;
	    }
	    if (total_instances == 0) {
                HM2_ERR("error counting instances of %s, aborting\n", gtag_name);
                return -EINVAL;
            } else if(total_instances == 1) {
		size_t ret = snprintf(alias_base, sizeof(alias_base),
		    "%s.%s.%02d.%s",
		    hm2->llio->name,
		    gtag_name,
		    hm2->pin[i].sec_unit,
		    funct_name
		    );
		if (ret >= sizeof(alias_base)) return -EINVAL;
	    } else {
		size_t ret = snprintf(alias_base, sizeof(alias_base),
		    "%s.%s.%02d.%d.%s",
		    hm2->llio->name,
		    gtag_name,
		    hm2->pin[i].sec_unit,
		    this_instance,
		    funct_name
		    );
		if (ret >= sizeof(alias_base)) return -EINVAL;
	    }
            r = do_alias(orig_base, alias_base, ".invert_output",
                hal_param_alias);
            if (r < 0) {
                HM2_ERR("Failed to add %s.invert_output alias, continuing\n",
                    orig_base);
            }
            r = do_alias(orig_base, alias_base, ".is_opendrain",
                hal_param_alias);
            if (r < 0) {
                HM2_ERR("Failed to add %s.is_opendrain alias, continuing\n",
                    orig_base);
            }
        }
    }

    return 0;
}




void hm2_ioport_print_module(hostmot2_t *hm2) {
    int i;
    HM2_PRINT("IO Ports: %d\n", hm2->ioport.num_instances);
    if (hm2->ioport.num_instances <= 0) return;
    HM2_PRINT("    clock_frequency: %d Hz (%s MHz)\n", hm2->ioport.clock_frequency, hm2_hz_to_mhz(hm2->ioport.clock_frequency));
    HM2_PRINT("    version: %d\n", hm2->ioport.version);
    HM2_PRINT("    data_addr: 0x%04X\n", hm2->ioport.data_addr);
    HM2_PRINT("    ddr_addr: 0x%04X\n", hm2->ioport.ddr_addr);
    HM2_PRINT("    alt_source_addr: 0x%04X\n", hm2->ioport.alt_source_addr);
    HM2_PRINT("    open_drain_addr: 0x%04X\n", hm2->ioport.open_drain_addr);
    HM2_PRINT("    output_invert_addr: 0x%04X\n", hm2->ioport.output_invert_addr);
    for (i = 0; i < hm2->ioport.num_instances; i ++) {
        HM2_PRINT("    instance %d:\n", i);
        HM2_PRINT("        data_read = 0x%06X\n", hm2->ioport.data_read_reg[i]);
        HM2_PRINT("        data_write = 0x%06X\n", hm2->ioport.data_write_reg[i]);
        HM2_PRINT("        ddr = 0x%06X\n", hm2->ioport.ddr_reg[i]);
        HM2_PRINT("        alt_source = 0x%06X\n", hm2->ioport.alt_source_reg[i]);
        HM2_PRINT("        open_drain = 0x%06X\n", hm2->ioport.open_drain_reg[i]);
        HM2_PRINT("        output_invert = 0x%06X\n", hm2->ioport.output_invert_reg[i]);
    }
}




static void hm2_ioport_force_write_ddr(hostmot2_t *hm2) {
    int size = hm2->ioport.num_instances * sizeof(rtapi_u32);
    hm2->llio->write(hm2->llio, hm2->ioport.ddr_addr, hm2->ioport.ddr_reg, size);
    memcpy(hm2->ioport.written_ddr, hm2->ioport.ddr_reg, size);
}


static void hm2_ioport_force_write_output_invert(hostmot2_t *hm2) {
    int size = hm2->ioport.num_instances * sizeof(rtapi_u32);
    hm2->llio->write(hm2->llio, hm2->ioport.output_invert_addr, hm2->ioport.output_invert_reg, size);
    memcpy(hm2->ioport.written_output_invert, hm2->ioport.output_invert_reg, size);
}


static void hm2_ioport_force_write_open_drain(hostmot2_t *hm2) {
    int size = hm2->ioport.num_instances * sizeof(rtapi_u32);
    hm2->llio->write(hm2->llio, hm2->ioport.open_drain_addr, hm2->ioport.open_drain_reg, size);
    memcpy(hm2->ioport.written_open_drain, hm2->ioport.open_drain_reg, size);
}

void hm2_ioport_initialize_ddr(hostmot2_t *hm2) {
    int port;

    for (port = 0; port < hm2->ioport.num_instances; port ++) {
        for (unsigned port_pin = 0; port_pin < hm2->idrom.port_width; port_pin ++) {
            int io_pin = (port * hm2->idrom.port_width) + port_pin;
            hm2->pin[io_pin].direction = hm2->pin[io_pin].direction_at_start;
        }
    }
}


void hm2_ioport_update(hostmot2_t *hm2) {
    int port;

    for (port = 0; port < hm2->ioport.num_instances; port ++) {
        for (unsigned port_pin = 0; port_pin < hm2->idrom.port_width; port_pin ++) {
            int io_pin = (port * hm2->idrom.port_width) + port_pin;

            if (hm2->pin[io_pin].gtag == HM2_GTAG_IOPORT) {
                if (hm2->pin[io_pin].instance->hal.param.is_output) {
                    hm2->pin[io_pin].direction = HM2_PIN_DIR_IS_OUTPUT;
                } else {
                    hm2->pin[io_pin].direction = HM2_PIN_DIR_IS_INPUT;
                }
            }

            if (hm2->pin[io_pin].direction == HM2_PIN_DIR_IS_OUTPUT) {
                hm2->ioport.ddr_reg[port] |= (1 << port_pin);  // set the bit in the ddr register

                // Open Drain Register
                if (hm2->pin[io_pin].instance->hal.param.is_opendrain) {
                    hm2->ioport.open_drain_reg[port] |= (1 << port_pin);  // set the bit in the open drain register
                } else {
                    hm2->ioport.open_drain_reg[port] &= ~(1 << port_pin);  // clear the bit in the open drain register
                }

                // Invert Output Register
                if (hm2->pin[io_pin].instance->hal.param.invert_output) {
                    hm2->ioport.output_invert_reg[port] |= (1 << port_pin);  // set the bit in the output invert register
                } else {
                    hm2->ioport.output_invert_reg[port] &= ~(1 << port_pin);  // clear the bit in the output invert register
                }
            } else {
                hm2->ioport.open_drain_reg[port] &= ~(1 << port_pin);  // clear the bit in the open drain register
                hm2->ioport.ddr_reg[port] &= ~(1 << port_pin);  // clear the bit in the ddr register
                // it doesn't matter what the Invert Output register says
            }
        }
    }
}


void hm2_ioport_force_write(hostmot2_t *hm2) {
    int size = hm2->ioport.num_instances * sizeof(rtapi_u32);

    hm2_ioport_update(hm2);

    hm2_ioport_force_write_output_invert(hm2);
    hm2_ioport_force_write_open_drain(hm2);
    hm2->llio->write(hm2->llio, hm2->ioport.alt_source_addr,    hm2->ioport.alt_source_reg,    size);
// write the DDR last to avoid startup glitches.
    hm2_ioport_force_write_ddr(hm2);

}


void hm2_ioport_write(hostmot2_t *hm2) {
    int port;

    hm2_ioport_update(hm2);

    for (port = 0; port < hm2->ioport.num_instances; port ++) {
        if (hm2->ioport.written_open_drain[port] != hm2->ioport.open_drain_reg[port]) {
            hm2_ioport_force_write_open_drain(hm2);
            break;
        }
    }

    for (port = 0; port < hm2->ioport.num_instances; port ++) {
        if (hm2->ioport.written_output_invert[port] != hm2->ioport.output_invert_reg[port]) {
            hm2_ioport_force_write_output_invert(hm2);
            break;
        }
    }
// write the DDR last to avoid startup glitches.
    for (port = 0; port < hm2->ioport.num_instances; port ++) {
        if (hm2->ioport.written_ddr[port] != hm2->ioport.ddr_reg[port]) {
            hm2_ioport_force_write_ddr(hm2);
            break;
        }
    }
}




//
// initialize the tram write registers
//

void hm2_ioport_gpio_tram_write_init(hostmot2_t *hm2) {
    int port;
    for (port = 0; port < hm2->ioport.num_instances; port ++) {
        hm2->ioport.data_write_reg[port] = 0;
    }
}




//
// the ioport.data_read buffer has been updated by a TRAM read from the values in the ioport data register
// this function sets the HAL pins based on the values in the data_read register buffer
//

void hm2_ioport_gpio_process_tram_read(hostmot2_t *hm2) {
    int port;

    // 
    // parse it out to the HAL pins
    //

    for (port = 0; port < hm2->ioport.num_instances; port ++) {
        for (unsigned port_pin = 0; port_pin < hm2->idrom.port_width; port_pin ++) {
            int io_pin = (port * hm2->idrom.port_width) + port_pin;
            hal_bit_t bit;

            bit = (hm2->ioport.data_read_reg[port] >> port_pin) & 0x1;
            *hm2->pin[io_pin].instance->hal.pin.in = bit;
            *hm2->pin[io_pin].instance->hal.pin.in_not = !bit;
        }
    }
}




//
// this function sets the data_write register TRAM buffer from the values of the HAL pins
// the data_write buffer will get written to the TRAM and thus to the ioport data register by the caller
// 

void hm2_ioport_gpio_prepare_tram_write(hostmot2_t *hm2) {
    int port;

    //
    // copy HAL pins to HM2 pins
    //

    for (port = 0; port < hm2->ioport.num_instances; port ++) {
        for (unsigned port_pin = 0; port_pin < hm2->idrom.port_width; port_pin ++) {
            int io_pin = (port * hm2->idrom.port_width) + port_pin;

            if (hm2->pin[io_pin].gtag != HM2_GTAG_IOPORT) continue;

            hm2->ioport.data_write_reg[port] &= ~(1 << port_pin);   // zero the bit
            if(*(hm2->pin[io_pin].instance->hal.pin.out))
                hm2->ioport.data_write_reg[port] |= (1 << port_pin);  // and set if appropriate
        }
    }
}


void hm2_ioport_gpio_read(hostmot2_t *hm2) {
    int port;

    // this should never happen - what's an AnyIO board without IO?
    if (hm2->ioport.num_instances <= 0) return;

    // llio->read is OK here, as this function is not used by hm2_eth
    hm2->llio->read(
        hm2->llio,
        hm2->ioport.data_addr,
        hm2->ioport.data_read_reg,
        hm2->ioport.num_instances * sizeof(rtapi_u32)
    );

    // FIXME: this block duplicates code in hm2_ioport_gpio_process_tram_read()
    for (port = 0; port < hm2->ioport.num_instances; port ++) {
        for (unsigned port_pin = 0; port_pin < hm2->idrom.port_width; port_pin ++) {
            int io_pin = (port * hm2->idrom.port_width) + port_pin;
            hal_bit_t bit;

            if (hm2->pin[io_pin].direction != HM2_PIN_DIR_IS_INPUT) continue;

            bit = (hm2->ioport.data_read_reg[port] >> port_pin) & 0x1;
            *hm2->pin[io_pin].instance->hal.pin.in = bit;
            *hm2->pin[io_pin].instance->hal.pin.in_not = !bit;
        }
    }
}


void hm2_ioport_gpio_write(hostmot2_t *hm2) {
    int port;

    // this should never happen - what's an AnyIO board without IO?
    if (hm2->ioport.num_instances <= 0) return;

    hm2_ioport_write(hm2);  // this updates any config registers that need it

    // FIXME: this block duplicates code in hm2_ioport_gpio_prepare_tram_write()
    for (port = 0; port < hm2->ioport.num_instances; port ++) {
        for (unsigned port_pin = 0; port_pin < hm2->idrom.port_width; port_pin ++) {
            int io_pin = (port * hm2->idrom.port_width) + port_pin;

            if (hm2->pin[io_pin].gtag != HM2_GTAG_IOPORT) continue;

            hm2->ioport.data_write_reg[port] &= ~(1 << port_pin);   // zero the bit
            hm2->ioport.data_write_reg[port] |= (*(hm2->pin[io_pin].instance->hal.pin.out) << port_pin);  // and set it as appropriate
        }
    }

    hm2->llio->write(
        hm2->llio,
        hm2->ioport.data_addr,
        hm2->ioport.data_write_reg,
        hm2->ioport.num_instances * sizeof(rtapi_u32)
    );
}


