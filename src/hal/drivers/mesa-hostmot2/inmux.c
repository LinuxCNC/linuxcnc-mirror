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
// This is a driver for the Hostmot2 inmux module.
//
// Register map:
//
// InMux: Input multiplexor/expander: uses external multiplexor chips to
// expand inputs. Debounce filtering is provided with an option of long or
// short time on a per input basis. Up to 4 MPG encoder counters are provided
// 
// 0x8000        Control Register
// Bits 0..4     5 bit MaxBit R/O last mux address (add 1 for number of bits scanned)
// Bit 5         Global invert R/W (inverts all input bits)
// Bits 6..15    10 bit Rate_Divisor R/W
// Bits 16..21   6 bit Short time register R/W (0 to 63 scans)
// Bits 22..31   10 bit Long time register R/W (0 to 1023 scans)
// 
// Per channel scan rate is (ClockLow/4)/(Rate_divisor+1)
// Total scan rate is per channel scan rate/(MaxBit+1)
// 
// 0x8100       Filter Register  RW (width = MaxBit+1)
// 
// A 1 bit in the filter register selects the long time input filter for
// the corresponding input bit, a 0 bit selects the short time.
// 
// 0x8200       Filtered Data R/O filtered demuxed data
// 
// 0x8300       Raw Data R/O raw demuxed data
// 
// 0x8400       MPG registers 0..3 all 8 bit counters
// (0=LSByte = ins 0,1) etc
// 0x8400 write LS bit of each byte is mode bit, 0 = 1X mode 1 = 4X mode



#include <rtapi_slab.h>

#include "rtapi.h"
#include "hal.h"

#include "hal/drivers/mesa-hostmot2/hostmot2.h"


int hm2_inmux_parse_md(hostmot2_t *hm2, int md_index) {
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    int r;

    if (hm2->inmux.num_instances != 0) {
        HM2_ERR(
            "found duplicate Module Descriptor for %s (inconsistent firmware), not loading driver\n",
            hm2_get_general_function_name(md->gtag)
        );
        return -EINVAL;
    }

    if (hm2->config.num_inmuxs > md->instances) {
        HM2_ERR(
            "config.num_inmuxs=%d, but only %d are available, not loading driver\n",
            hm2->config.num_inmuxs,
            md->instances
        );
        return -EINVAL;
    }

    if (hm2->config.num_inmuxs == 0) {
        return 0;
    }


    //
    // Looks good, start initializing.
    //

    if (hm2->config.num_inmuxs == -1) {
        hm2->inmux.num_instances = md->instances;
    } else {
        hm2->inmux.num_instances = hm2->config.num_inmuxs;
    }

    hm2->inmux.clock_frequency = md->clock_freq;
    hm2->inmux.version = md->version;

    hm2->inmux.instance = (hm2_inmux_instance_t *)hal_malloc(hm2->inmux.num_instances * sizeof(hm2_inmux_instance_t));
    if (hm2->inmux.instance == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }


    hm2->inmux.control_addr = md->base_address + (0 * md->register_stride);
    hm2->inmux.filter_addr = md->base_address + (1 * md->register_stride);
    hm2->inmux.filt_data_addr = md->base_address + (2 * md->register_stride);
    hm2->inmux.raw_data_addr = md->base_address + (3 * md->register_stride);
    hm2->inmux.mpg_read_addr = md->base_address + (4 * md->register_stride);
    hm2->inmux.mpg_mode_addr = md->base_address + (4 * md->register_stride);

    hm2->inmux.control_reg = (rtapi_u32*)rtapi_kmalloc(hm2->inmux.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
    if (hm2->inmux.control_reg == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }
    hm2->inmux.mpg_mode_reg = (rtapi_u32*)rtapi_kmalloc(hm2->inmux.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
    if (hm2->inmux.mpg_mode_reg == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }

    r = hm2_register_tram_read_region(hm2, hm2->inmux.filt_data_addr, (hm2->inmux.num_instances * sizeof(rtapi_u32)), &hm2->inmux.filt_data_reg);
    if (r < 0) {
        HM2_ERR("error registering tram read region for InMux Filtered Data register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_read_region(hm2, hm2->inmux.raw_data_addr, (hm2->inmux.num_instances * sizeof(rtapi_u32)), &hm2->inmux.raw_data_reg);
    if (r < 0) {
        HM2_ERR("error registering tram read region for InMux Raw Data register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_read_region(hm2, hm2->inmux.mpg_read_addr, (hm2->inmux.num_instances * sizeof(rtapi_u32)), &hm2->inmux.mpg_read_reg);
    if (r < 0) {
        HM2_ERR("error registering tram read region for InMux MPG register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_write_region(hm2, hm2->inmux.filter_addr, (hm2->inmux.num_instances * sizeof(rtapi_u32)), &hm2->inmux.filter_reg);
    if (r < 0) {
        HM2_ERR("error registering tram write region for InMux Filter register (%d)\n", r);
        goto fail1;
    }


    //
    // Export to HAL.
    //

    {
        int i;
	int temp;
        char name[HAL_NAME_LEN + 1];

        for (i = 0; i < hm2->inmux.num_instances; i ++) {
            

            // first do a low level read to determine the per instance scanwidth
            hm2->llio->read(hm2->llio,hm2->inmux.control_addr + (i * md->instance_stride),&temp, sizeof(rtapi_u32));
            temp  = (temp & 0x0000001f) +1;
	    hm2->inmux.instance[i].scanwidth = temp;         
	    hm2->inmux.instance[i].hal.param.scan_width = temp;

            rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.scan_rate", hm2->llio->name, i);
            r = hal_param_u32_new(name, HAL_RW, &(hm2->inmux.instance[i].hal.param.scan_rate), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }
            rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.slow_scans", hm2->llio->name, i);
            r = hal_param_u32_new(name, HAL_RW, &(hm2->inmux.instance[i].hal.param.slow_scans), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            } 
            rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.fast_scans", hm2->llio->name, i);
            r = hal_param_u32_new(name, HAL_RW, &(hm2->inmux.instance[i].hal.param.fast_scans), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }  
           rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc0_4xmode", hm2->llio->name, i);
            r = hal_param_bit_new(name, HAL_RW, &(hm2->inmux.instance[i].hal.param.enc0_mode), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }
            rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc1_4xmode", hm2->llio->name, i);
            r = hal_param_bit_new(name, HAL_RW, &(hm2->inmux.instance[i].hal.param.enc1_mode), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }
            rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc2_4xmode", hm2->llio->name, i);
            r = hal_param_bit_new(name, HAL_RW, &(hm2->inmux.instance[i].hal.param.enc2_mode), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            } 
            rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc3_4xmode", hm2->llio->name, i);
            r = hal_param_bit_new(name, HAL_RW, &(hm2->inmux.instance[i].hal.param.enc3_mode), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }
            rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.scan_width", hm2->llio->name, i);
            r = hal_param_u32_new(name, HAL_RO, &(hm2->inmux.instance[i].hal.param.scan_width), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }

            {
                unsigned j = 0;
                for (j = 0; j < hm2->inmux.instance[i].scanwidth; j++){
  

                        rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.input-%02u", hm2->llio->name, i, j);
                        r = hal_pin_bit_new(name, HAL_OUT, &(hm2->inmux.instance[i].hal.pin.filt_data[j]), hm2->llio->comp_id);
                        if (r < 0) {
                            HM2_ERR("error adding pin '%s', aborting\n", name);
                            r = -ENOMEM;
                            goto fail1;
                        }
                         rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.raw-input-%02u", hm2->llio->name, i, j);
                        r = hal_pin_bit_new(name, HAL_OUT, &(hm2->inmux.instance[i].hal.pin.raw_data[j]), hm2->llio->comp_id);
                        if (r < 0) {
                            HM2_ERR("error adding pin '%s', aborting\n", name);
                            r = -ENOMEM;
                            goto fail1;
                        }
                        rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.input-%02u-not", hm2->llio->name, i, j);
                        r = hal_pin_bit_new(name, HAL_OUT, &(hm2->inmux.instance[i].hal.pin.filt_data_not[j]), hm2->llio->comp_id);
                        if (r < 0) {
                            HM2_ERR("error adding pin '%s', aborting\n", name);
                            r = -ENOMEM;
                            goto fail1;
                        }
                         rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.raw-input-%02u-not", hm2->llio->name, i, j);
                        r = hal_pin_bit_new(name, HAL_OUT, &(hm2->inmux.instance[i].hal.pin.raw_data_not[j]), hm2->llio->comp_id);
                        if (r < 0) {
                            HM2_ERR("error adding pin '%s', aborting\n", name);
                            r = -ENOMEM;
                            goto fail1;
                        }
                        rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.input-%02u-slow", hm2->llio->name, i, j);
                        r = hal_pin_bit_new(name, HAL_IN, &(hm2->inmux.instance[i].hal.pin.slow[j]), hm2->llio->comp_id);
                        if (r < 0) {
                            HM2_ERR("error adding pin '%s', aborting\n", name);
                            r = -ENOMEM;
                            goto fail1;
                        }
                   }

                rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc0-count", hm2->llio->name, i);
                r = hal_pin_s32_new(name, HAL_OUT, &(hm2->inmux.instance[i].hal.pin.enc0_count), hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    r = -ENOMEM;
                    goto fail1;
                }

                rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc1-count", hm2->llio->name, i);
                r = hal_pin_s32_new(name, HAL_OUT, &(hm2->inmux.instance[i].hal.pin.enc1_count), hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    r = -ENOMEM;
                    goto fail1;
                }

                rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc2-count", hm2->llio->name, i);
                r = hal_pin_s32_new(name, HAL_OUT, &(hm2->inmux.instance[i].hal.pin.enc2_count), hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    r = -ENOMEM;
                    goto fail1;
                }

                rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc3-count", hm2->llio->name, i);
                r = hal_pin_s32_new(name, HAL_OUT, &(hm2->inmux.instance[i].hal.pin.enc3_count), hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    r = -ENOMEM;
                    goto fail1;
                }

                rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc0-reset", hm2->llio->name, i);
                r = hal_pin_bit_new(name, HAL_IN, &(hm2->inmux.instance[i].hal.pin.enc0_reset), hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    r = -ENOMEM;
                    goto fail1;
                }

                rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc1-reset", hm2->llio->name, i);
                r = hal_pin_bit_new(name, HAL_IN, &(hm2->inmux.instance[i].hal.pin.enc1_reset), hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    r = -ENOMEM;
                    goto fail1;
                }

                rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc2-reset", hm2->llio->name, i);
                r = hal_pin_bit_new(name, HAL_IN, &(hm2->inmux.instance[i].hal.pin.enc2_reset), hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    r = -ENOMEM;
                    goto fail1;
                }

                rtapi_snprintf(name, sizeof(name), "%s.inmux.%02d.enc3-reset", hm2->llio->name, i);
                r = hal_pin_bit_new(name, HAL_IN, &(hm2->inmux.instance[i].hal.pin.enc3_reset), hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    r = -ENOMEM;
                    goto fail1;
                }

            }

        }

    }

    //
    // Set inmux default scanrate and filter values 
    // and remove initial MPG offset

    {
        int i;
        int rawmpgs;
        for (i = 0; i < hm2->inmux.num_instances; i ++) {
            hm2->inmux.instance[i].hal.param.scan_rate = 20000; // 20 KHz = 50 usec/scan
            hm2->inmux.instance[i].hal.param.slow_scans = 500;  // 500*50 usec = 25 ms
            hm2->inmux.instance[i].hal.param.fast_scans = 5;   //  5*50 usec = 250 usec
            hm2->llio->read(hm2->llio,hm2->inmux.mpg_read_addr + (i * md->instance_stride),&rawmpgs, sizeof(rtapi_u32));
            hm2->inmux.instance[i].prev_enc0_count = (rtapi_s32)((rawmpgs >>  0) & 0x000000FF); 
            hm2->inmux.instance[i].prev_enc1_count = (rtapi_s32)((rawmpgs >>  8) & 0x000000FF); 
            hm2->inmux.instance[i].prev_enc2_count = (rtapi_s32)((rawmpgs >> 16) & 0x000000FF); 
            hm2->inmux.instance[i].prev_enc3_count = (rtapi_s32)((rawmpgs >> 24) & 0x000000FF); 

        }

    }

    return hm2->inmux.num_instances;

fail1:
    rtapi_kfree(hm2->inmux.control_reg);

fail0:
    hm2->inmux.num_instances = 0;
    return r;
}




void hm2_inmux_force_write(hostmot2_t *hm2) {
    int divisor;
    int i;
    int size;
    double muxrate;
    if (hm2->inmux.num_instances <= 0) {
        return;
    }

    // setup control register and mpg_write
    for (i = 0; i < hm2->inmux.num_instances; i ++) {
        muxrate = hm2->inmux.instance[i].scanwidth * hm2->inmux.instance[i].hal.param.scan_rate;
	if (muxrate > 5000000) {
            muxrate = 5000000;
	    hm2->inmux.instance[i].hal.param.scan_rate = muxrate/hm2->inmux.instance[i].scanwidth;
        }
        divisor = (hm2->inmux.clock_frequency / (4 * muxrate)) - 1;
	if (hm2->inmux.instance[i].hal.param.fast_scans > 63) {
            hm2->inmux.instance[i].hal.param.fast_scans = 63;
        }
        if (hm2->inmux.instance[i].hal.param.slow_scans > 1023) {
            hm2->inmux.instance[i].hal.param.slow_scans = 1023;
        }

         hm2->inmux.control_reg[i] = (1 << 5) +  
//       global invert bit(5) fixed to true for now as this matches all existing hardware
	(divisor << 6) +
        (hm2->inmux.instance[i].hal.param.fast_scans  << 16) +
        (hm2->inmux.instance[i].hal.param.slow_scans  << 22);
    }

    size = hm2->inmux.num_instances * sizeof(rtapi_u32);

    // Write register values to board.
    hm2->llio->write(hm2->llio, hm2->inmux.control_addr, hm2->inmux.control_reg, size);
    hm2->llio->write(hm2->llio, hm2->inmux.mpg_mode_addr, hm2->inmux.mpg_mode_reg, size);


    // Cache written-out register values.
    for (i = 0; i < hm2->inmux.num_instances; i ++) {
        hm2->inmux.instance[i].written_control_reg = hm2->inmux.control_reg[i];
        hm2->inmux.instance[i].written_mpg_mode_reg = hm2->inmux.mpg_mode_reg[i];

    }
}

void hm2_inmux_write(hostmot2_t *hm2) {
    int divisor;
    int i;
    int size;
    double muxrate;

    size = hm2->inmux.num_instances * sizeof(rtapi_u32);


    for (i = 0; i < hm2->inmux.num_instances; i ++) {
        muxrate = hm2->inmux.instance[i].scanwidth * hm2->inmux.instance[i].hal.param.scan_rate;
//      bound muxrate maximum frequency
	if (muxrate > 5000000) {
            muxrate = 5000000;
	    hm2->inmux.instance[i].hal.param.scan_rate = muxrate/hm2->inmux.instance[i].scanwidth;
            HM2_ERR("InMux %d scanrate too high, resetting to %d \n", i,hm2->inmux.instance[i].hal.param.scan_rate);
        }
        divisor = (hm2->inmux.clock_frequency / (4 * muxrate)) - 1;
//      bound divisor so we dont splatter into other fields
	if ((divisor > 1023 ) | (muxrate == 0 )) {
            divisor = 1023;
	    hm2->inmux.instance[i].hal.param.scan_rate = (hm2->inmux.clock_frequency/4)/(divisor +1)
            /hm2->inmux.instance[i].scanwidth;
            HM2_ERR("InMux %d scanrate too low, resetting to %d \n", i,hm2->inmux.instance[i].hal.param.scan_rate);
        }
	if (hm2->inmux.instance[i].hal.param.fast_scans > 63) {
            hm2->inmux.instance[i].hal.param.fast_scans = 63;
            HM2_ERR("InMux %d fastscans must be less than 63, resetting to %d \n", i,63);
        }
        if (hm2->inmux.instance[i].hal.param.slow_scans > 1023) {
            hm2->inmux.instance[i].hal.param.slow_scans = 1023;
            HM2_ERR("InMux %d slowscans  must be less than 1023, resetting to %d \n", i,1023);
        }
	if (hm2->inmux.instance[i].hal.param.fast_scans < 1 ) {
            hm2->inmux.instance[i].hal.param.fast_scans = 1;
            HM2_ERR("InMux %d fastscans must be greater than 0, resetting to %d \n", i,1);
        }
        if (hm2->inmux.instance[i].hal.param.slow_scans < 1) {
            hm2->inmux.instance[i].hal.param.slow_scans = 1;
            HM2_ERR("InMux %d slowscans must be greater than 0, resetting to %d \n", i,1);
        }

         hm2->inmux.control_reg[i] = (1 << 5) +  
//       global invert bit(5) fixed to true for now as this matches all existing hardware)
	(divisor << 6) +
        (hm2->inmux.instance[i].hal.param.fast_scans  << 16) +
        (hm2->inmux.instance[i].hal.param.slow_scans  << 22);
        if (hm2->inmux.control_reg[i] != hm2->inmux.instance[i].written_control_reg) {
            hm2->llio->write(hm2->llio, hm2->inmux.control_addr, hm2->inmux.control_reg, size);
            hm2->inmux.instance[i].written_control_reg = hm2->inmux.control_reg[i];
//            HM2_PRINT(" Debug: updating inmux control reg to = 0x%08X\n", hm2->inmux.control_reg[i]);
        }
        hm2->inmux.filter_reg[i] = 0;
	for (unsigned j = 0; j < hm2->inmux.instance[i].scanwidth; j ++) {
            hm2->inmux.filter_reg[i] |= (*hm2->inmux.instance[i].hal.pin.slow[j] << j);
        }
        if (hm2->inmux.filter_reg[i] != hm2->inmux.instance[i].written_filter_reg) {
            hm2->llio->write(hm2->llio, hm2->inmux.filter_addr, hm2->inmux.filter_reg, size);
            hm2->inmux.instance[i].written_filter_reg = hm2->inmux.filter_reg[i];
//            HM2_PRINT(" Debug: updating inmux filter reg to = 0x%08X\n", hm2->inmux.filter_reg[i]);
        }
        
	hm2->inmux.mpg_mode_reg[i] =        
	hm2->inmux.instance[i].hal.param.enc0_mode << 0  |
        hm2->inmux.instance[i].hal.param.enc1_mode << 8  |
        hm2->inmux.instance[i].hal.param.enc2_mode << 16 |
        hm2->inmux.instance[i].hal.param.enc3_mode << 24;
        if (hm2->inmux.mpg_mode_reg[i] != hm2->inmux.instance[i].written_mpg_mode_reg) {
            hm2->llio->write(hm2->llio, hm2->inmux.mpg_mode_addr, hm2->inmux.mpg_mode_reg, size);
            hm2->inmux.instance[i].written_mpg_mode_reg = hm2->inmux.mpg_mode_reg[i];
//            HM2_PRINT(" Debug: updating inmux mpg mode reg to = 0x%08X\n", hm2->inmux.mpg_mode_reg[i]);
        }       
    }
}


void hm2_inmux_prepare_tram_write(hostmot2_t *hm2) {
    int i;

    // Set register values from HAL pin values.
    for (i = 0; i < hm2->inmux.num_instances; i ++) {
        hm2->inmux.filter_reg[i] = 0;
        for (unsigned j = 0; j < hm2->inmux.instance[i].scanwidth; j ++) {
            hm2->inmux.filter_reg[i] |= (*hm2->inmux.instance[i].hal.pin.slow[j] << j);
        }
    }
}

void hm2_inmux_process_tram_read(hostmot2_t *hm2) {
    int i;
    int count_diff;
    int raw_count;

    if (hm2->inmux.num_instances <= 0) {
        return;
    }
    for (i = 0; i < hm2->inmux.num_instances; i ++) {
        for (unsigned j = 0; j < hm2->inmux.instance[i].scanwidth; j ++) {
           *hm2->inmux.instance[i].hal.pin.filt_data[j] = (hm2->inmux.filt_data_reg[i] >> j) &1;
           *hm2->inmux.instance[i].hal.pin.raw_data[j] = (hm2->inmux.raw_data_reg[i] >> j) &1;
           *hm2->inmux.instance[i].hal.pin.filt_data_not[j] = !((hm2->inmux.filt_data_reg[i] >> j) &1);
           *hm2->inmux.instance[i].hal.pin.raw_data_not[j] = !((hm2->inmux.raw_data_reg[i] >> j) &1);
        }

	raw_count = hm2->inmux.mpg_read_reg[i] & 0x000000FF;	
        count_diff = (rtapi_s32)raw_count - hm2->inmux.instance[i].prev_enc0_count;
        hm2->inmux.instance[i].prev_enc0_count = hm2->inmux.instance[i].prev_enc0_count + count_diff;
        if (count_diff >  128) count_diff -= 256;
        if (count_diff < -128) count_diff += 256;
	if (*hm2->inmux.instance[i].hal.pin.enc0_reset == 0) {
	    *hm2->inmux.instance[i].hal.pin.enc0_count = *hm2->inmux.instance[i].hal.pin.enc0_count + count_diff;
	    } else { *hm2->inmux.instance[i].hal.pin.enc0_count = 0;}
	raw_count = (hm2->inmux.mpg_read_reg[i] >> 8) & 0x000000FF;	
        count_diff = (rtapi_s32)raw_count - hm2->inmux.instance[i].prev_enc1_count;
        hm2->inmux.instance[i].prev_enc1_count = hm2->inmux.instance[i].prev_enc1_count + count_diff;
        if (count_diff >  128) count_diff -= 256;
        if (count_diff < -128) count_diff += 256;
	if (*hm2->inmux.instance[i].hal.pin.enc1_reset == 0) {
	    *hm2->inmux.instance[i].hal.pin.enc1_count = *hm2->inmux.instance[i].hal.pin.enc1_count + count_diff;
	    } else { *hm2->inmux.instance[i].hal.pin.enc1_count = 0;}
	raw_count = (hm2->inmux.mpg_read_reg[i] >> 16) & 0x000000FF;	
        count_diff = (rtapi_s32)raw_count - hm2->inmux.instance[i].prev_enc2_count;
        hm2->inmux.instance[i].prev_enc2_count = hm2->inmux.instance[i].prev_enc2_count + count_diff;
        if (count_diff >  128) count_diff -= 256;
        if (count_diff < -128) count_diff += 256;
	if (*hm2->inmux.instance[i].hal.pin.enc2_reset == 0) {
	    *hm2->inmux.instance[i].hal.pin.enc2_count = *hm2->inmux.instance[i].hal.pin.enc2_count + count_diff;
	    } else { *hm2->inmux.instance[i].hal.pin.enc2_count = 0;}
	raw_count = (hm2->inmux.mpg_read_reg[i] >> 24) & 0x000000FF;	
        count_diff = (rtapi_s32)raw_count - hm2->inmux.instance[i].prev_enc3_count;
        hm2->inmux.instance[i].prev_enc3_count = hm2->inmux.instance[i].prev_enc3_count + count_diff;
        if (count_diff >  128) count_diff -= 256;
        if (count_diff < -128) count_diff += 256;
	if (*hm2->inmux.instance[i].hal.pin.enc3_reset == 0) {
	    *hm2->inmux.instance[i].hal.pin.enc3_count = *hm2->inmux.instance[i].hal.pin.enc3_count + count_diff;
	    } else { *hm2->inmux.instance[i].hal.pin.enc3_count = 0;}
    }
}

void hm2_inmux_cleanup(hostmot2_t *hm2) {
    if (hm2->inmux.num_instances <= 0) {
        return;
    }

}


void hm2_inmux_print_module(hostmot2_t *hm2) {
    int i;
    if (hm2->inmux.num_instances <= 0) return;
    HM2_PRINT("inmuxs: %d\n", hm2->inmux.num_instances);
    HM2_PRINT("    clock_frequency: %d Hz (%s MHz)\n", hm2->inmux.clock_frequency, hm2_hz_to_mhz(hm2->inmux.clock_frequency));
    HM2_PRINT("    version: %d\n", hm2->inmux.version);
    HM2_PRINT("    control_addr: 0x%04X\n", hm2->inmux.control_addr);
    HM2_PRINT("    filter_addr: 0x%04X\n", hm2->inmux.filter_addr);
    HM2_PRINT("    input_data_addr: 0x%04X\n", hm2->inmux.filt_data_addr);
    HM2_PRINT("    raw_data_addr: 0x%04X\n", hm2->inmux.raw_data_addr);
    HM2_PRINT("    mpg_addr: 0x%04X\n", hm2->inmux.mpg_read_addr);
    for (i = 0; i < hm2->inmux.num_instances; i ++) {
        HM2_PRINT("    instance %d:\n", i);
        HM2_PRINT("        control_reg = 0x%08X\n", hm2->inmux.control_reg[i]);
        HM2_PRINT("        filter_reg = 0x%08X\n", hm2->inmux.filter_reg[i]);
        HM2_PRINT("        input_data_reg: 0x%08X\n", hm2->inmux.filt_data_reg[i]);
        HM2_PRINT("        raw_data_reg: 0x%08X\n", hm2->inmux.raw_data_reg[i]);
        HM2_PRINT("        mpg_reg = 0x%08X\n", hm2->inmux.mpg_read_reg[i]);
    }
}






