//
//    Copyright (C) 2011 Andy Pugh
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERinstTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
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
#include "hostmot2.h"

int hm2_uart_parse_md(hostmot2_t *hm2, int md_index) 
{
    // All this function actually does is allocate memory
    // and give the uart modules names. 
    
    
    // 
    // some standard sanity checks
    //
    
    int i, r = -EINVAL;
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    static int last_gtag = -1;
    
    //The UART declares a TX and RX module separately
    
    if (!hm2_md_is_consistent_or_complain(hm2, md_index, 0, 4, 0x10, 0x000F)) {
        HM2_ERR("inconsistent Module Descriptor!\n");
        return -EINVAL;
    }
    
    if (hm2->uart.num_instances > 1 && last_gtag == md->gtag) {
        HM2_ERR(
                "found duplicate Module Descriptor for %s (inconsistent "
                "firmware), not loading driver %i %i\n",
                hm2_get_general_function_name(md->gtag), md->gtag, last_gtag
                );
        return -EINVAL;
    }
    last_gtag = md->gtag;
    
    if (hm2->config.num_uarts > md->instances) {
        HM2_ERR(
                "config defines %d uarts, but only %d are available, "
                "not loading driver\n",
                hm2->config.num_uarts,
                md->instances
                );
        return -EINVAL;
    }
    
    if (hm2->config.num_uarts == 0) {
        return 0;
    }
    
    // 
    // looks good, start, or continue, initializing
    // 
    
    if (hm2->uart.num_instances == 0){
        if (hm2->config.num_uarts == -1) {
            hm2->uart.num_instances = md->instances;
        } else {
            hm2->uart.num_instances = hm2->config.num_uarts;
        }
        
        hm2->uart.instance = (hm2_uart_instance_t *)hal_malloc(hm2->uart.num_instances 
                                                               * sizeof(hm2_uart_instance_t));
        if (hm2->uart.instance == NULL) {
            HM2_ERR("out of memory!\n");
            r = -ENOMEM;
            goto fail0;
        }
    }
    
    for (i = 0 ; i < hm2->uart.num_instances ; i++){
        hm2_uart_instance_t *inst = &hm2->uart.instance[i];
        // For the time being we assume that all UARTS come on pairs
        if (inst->clock_freq == 0){
            inst->clock_freq = md->clock_freq;
            r = snprintf(inst->name, sizeof(inst->name), "%s.uart.%01d", hm2->llio->name, i);
            HM2_PRINT("created UART Interface function %s.\n", inst->name);
        }
        if (md->gtag == HM2_GTAG_UART_TX){
            inst->tx1_addr = md->base_address + i * md->instance_stride;
            inst->tx2_addr = md->base_address + i * md->instance_stride + 0x4;
            inst->tx3_addr = md->base_address + i * md->instance_stride + 0x8;
            inst->tx4_addr = md->base_address + i * md->instance_stride + 0xC;
            inst->tx_fifo_count_addr = (md->base_address 
                                        + md->register_stride 
                                        + i * md->instance_stride);
            inst->tx_bitrate_addr = (md->base_address 
                                     + 2 * md->register_stride
                                     + i * md->instance_stride);
            inst->tx_mode_addr = (md->base_address 
                                  + 3 * md->register_stride
                                  +i * md->instance_stride);  
        }
        else if (md->gtag == HM2_GTAG_UART_RX){
            inst->rx1_addr = md->base_address + i * md->instance_stride;
            inst->rx2_addr = md->base_address + i * md->instance_stride + 0x4;
            inst->rx3_addr = md->base_address + i * md->instance_stride + 0x8;
            inst->rx4_addr = md->base_address + i * md->instance_stride + 0xC;
            inst->rx_fifo_count_addr = (md->base_address 
                                        + md->register_stride 
                                        + i * md->instance_stride);
            inst->rx_bitrate_addr = (md->base_address 
                                     + 2 * md->register_stride
                                     + i * md->instance_stride);
            inst->rx_mode_addr = (md->base_address 
                                  + 3 * md->register_stride 
                                  +i * md->instance_stride);    
        }
        else{
            HM2_ERR("Something very weird happened");
            goto fail0;
        }
        
    }
    return hm2->uart.num_instances;
fail0:
    return r;
}

EXPORT_SYMBOL_GPL(hm2_uart_setup);
// use -1 for tx_mode and rx_mode to leave the mode unchanged
int hm2_uart_setup(char *name, int bitrate, rtapi_s32 tx_mode, rtapi_s32 rx_mode){
    hostmot2_t *hm2;
    hm2_uart_instance_t *inst = 0;
    rtapi_u32 buff;
    int i,r;
    
    i = hm2_get_uart(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find UART instance %s.\n", name);
        return -1;
    }
    inst = &hm2->uart.instance[i];
  
    buff = (rtapi_u32)((bitrate * 1048576.0)/inst->clock_freq); //20 bits in this version
    r = 0;
    if (buff != inst->bitrate){
        inst->bitrate = buff;
        r += hm2->llio->write(hm2->llio, inst->rx_bitrate_addr, &buff, sizeof(rtapi_u32));
        r += hm2->llio->write(hm2->llio, inst->tx_bitrate_addr, &buff, sizeof(rtapi_u32));
        buff = 0;
        r += hm2->llio->write(hm2->llio, inst->rx_mode_addr, &buff, sizeof(rtapi_u32)); // clear faults
        r += hm2->llio->write(hm2->llio, inst->rx_fifo_count_addr, &buff, sizeof(rtapi_u32)); // clear fifo
        r += hm2->llio->write(hm2->llio, inst->tx_fifo_count_addr, &buff, sizeof(rtapi_u32)); // clear fifo
    }
    
    if (tx_mode >= 0) {
        buff = ((rtapi_u32)tx_mode) & 0x7f;
        r += hm2->llio->write(hm2->llio, inst->tx_mode_addr, &buff, sizeof(rtapi_u32));
    }
    
    if (rx_mode >= 0) {
        buff = ((rtapi_u32)rx_mode) & 0xff;
        r += hm2->llio->write(hm2->llio, inst->rx_mode_addr, &buff, sizeof(rtapi_u32));
    }
        
    if (r < 0) {
        HM2_ERR("UART: hm2->llio->write failure %s\n", name);
        return -1;
    }
    
    return 0;
}


EXPORT_SYMBOL_GPL(hm2_uart_send);
int hm2_uart_send(char *name,  unsigned char data[], int count)
{
    hostmot2_t *hm2;
    rtapi_u32 buff;
    int r, c;
    int inst;
    static int err_flag = 0;
    
    inst = hm2_get_uart(&hm2, name);
    if (inst < 0 && !err_flag){
        HM2_ERR_NO_LL("Can not find UART instance %s.\n", name);
        err_flag = 1;
        return -1;
    }
    if (hm2->uart.instance[inst].bitrate == 0 && !err_flag){
        HM2_ERR("The selected UART instance %s.\n" 
                "Has not been configured.\n", name);
        err_flag = 1; // don't fill dmesg with junk. 
        return -1;
    }
    
    c = 0;
    err_flag = 0;
    while (c < count - 3){
        buff = (data[c] + 
                (data[c+1] << 8) +
                (data[c+2] << 16) +
                (data[c+3] << 24));
        r = hm2->llio->write(hm2->llio, hm2->uart.instance[inst].tx4_addr,
                             &buff, sizeof(rtapi_u32));
        if (r < 0) {
            HM2_ERR("UART WRITE: hm2->llio->write failure %s\n", name);
            return r;
        }
        c = c + 4;
    }
    switch(count - c){
        case 0: 
            return c;
        case 1:
            buff = data[c];
            r = hm2->llio->write(hm2->llio, hm2->uart.instance[inst].tx1_addr,
                                 &buff, sizeof(rtapi_u32));
            if (r < 0){
                HM2_ERR("UART WRITE: hm2->llio->write failure %s\n", name);
                return r;
            }else{
                return c + 1;
            }
        case 2:
            buff = (data[c] + 
                    (data[c+1] << 8));
            r = hm2->llio->write(hm2->llio, hm2->uart.instance[inst].tx2_addr,
                                 &buff, sizeof(rtapi_u32));
            if (r < 0){
                HM2_ERR("UART_WRITE: hm2->llio->write failure %s\n", name);
                return r;
            }else{
                return c + 2;
            }
        case 3:
            buff = (data[c] + 
                    (data[c+1] << 8) +
                    (data[c+2] << 16));
            r = hm2->llio->write(hm2->llio, hm2->uart.instance[inst].tx3_addr,
                                 &buff, sizeof(rtapi_u32));
            if (r < 0){
                HM2_ERR("UART WRITE: hm2->llio->write failure %s\n", name);
                return r;
            }else{
                return c + 3;
            }
        default:
            HM2_ERR("UART WRITE: Error in buffer parsing. count = %i, i = %i\n", count, c);
            return -1;
    }
}

// This function needs to be modified so that it does not call llio->read, which hurts performance on hm2-eth
EXPORT_SYMBOL_GPL(hm2_uart_read);
int hm2_uart_read(char *name, unsigned char data[])
{
    hostmot2_t *hm2;
    int r, c;
    int count;
    int inst;
    rtapi_u32 buff;
    static int err_flag = 0;
    
    inst = hm2_get_uart(&hm2, name);
    
    if (inst < 0){
        HM2_ERR_NO_LL("Can not find UART instance %s.\n", name);
        return -1;
    }
    if (hm2->uart.instance[inst].bitrate == 0 && !err_flag){
        HM2_ERR("The selected UART instance %s.\n" 
                "Has not been configured.\n", name);
        err_flag = 1;
        return -1;
    }
    
    err_flag = 0;
    
    r = hm2->llio->read(hm2->llio, hm2->uart.instance[inst].rx_fifo_count_addr,
                        &buff, sizeof(rtapi_u32));
    
    count = buff & 0x1F; 
    c = 0;
    while (c < count - 3 && c < 16){
        r = hm2->llio->read(hm2->llio, hm2->uart.instance[inst].rx4_addr,
                            &buff, sizeof(rtapi_u32));
        
        if (r < 0) {
            HM2_ERR("UART: hm2->llio->read failure %s\n", name);
            return r;
        }
          
        data[c]   = (buff & 0x000000FF);
        data[c+1] = (buff & 0x0000FF00) >> 8;
        data[c+2] = (buff & 0x00FF0000) >> 16;
        data[c+3] = (buff & 0xFF000000) >> 24;
        c = c + 4;
    }

    switch(count - c){
        case 0: 
            return c;
        case 1:
            r = hm2->llio->read(hm2->llio, hm2->uart.instance[inst].rx1_addr,
                                &buff, sizeof(rtapi_u32));
            if (r < 0) {
                HM2_ERR("UART READ: hm2->llio->read failure %s\n", name);
                return r;
            }
            data[c]   = (buff & 0x000000FF);
            return c + 1;
        case 2:
            r = hm2->llio->read(hm2->llio, hm2->uart.instance[inst].rx2_addr,
                                &buff, sizeof(rtapi_u32));
            if (r < 0) {
                HM2_ERR("UART READ: hm2->llio->read failure %s\n", name);
                return r;
            }
            data[c]   = (buff & 0x000000FF);
            data[c+1] = (buff & 0x0000FF00) >> 8;
            return c + 2;
        case 3:
            r = hm2->llio->read(hm2->llio, hm2->uart.instance[inst].rx3_addr,
                                &buff, sizeof(rtapi_u32));
            if (r < 0) {
                HM2_ERR("UART READ: hm2->llio->read failure %s\n", name);
                return r;
            }
            data[c]   = (buff & 0x000000FF);
            data[c+1] = (buff & 0x0000FF00) >> 8;
            data[c+2] = (buff & 0x00FF0000) >> 16;
            return c + 3;
        default:
            HM2_ERR("UART READ: Error in buffer parsing.\n");
            return -EINVAL;
    }
}

void hm2_uart_print_module(hostmot2_t *hm2){
    int i;
    HM2_PRINT("UART: %d\n", hm2->uart.num_instances);
    if (hm2->uart.num_instances <= 0) return;
    HM2_PRINT("    version: %d\n", hm2->uart.version);
    HM2_PRINT("    channel configurations\n");
    for (i = 0; i < hm2->uart.num_instances; i ++) {
        HM2_PRINT("    clock_frequency: %d Hz (%s MHz)\n", 
                  hm2->uart.instance[i].clock_freq, 
                  hm2_hz_to_mhz(hm2->uart.instance[i].clock_freq));
        HM2_PRINT("    instance %d:\n", i);
        HM2_PRINT("    HAL name = %s\n", hm2->uart.instance[i].name);
    }
}

// The following standard Hostmot2 functions are not currently used by uart. 

void hm2_uart_cleanup(hostmot2_t *hm2)
{
    (void)hm2;
}

void hm2_uart_write(hostmot2_t *hm2)
{
    (void)hm2;
}

