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
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

#include <rtapi_slab.h>
#include <rtapi_bool.h>
#include "rtapi.h"
#include "rtapi_string.h"
#include "rtapi_math.h"
#include "hal.h"
#include "hostmot2.h"

int hm2_bspi_parse_md(hostmot2_t *hm2, int md_index) 
{
    // All this function actually does is allocate memory
    // and give the bspi modules names. 
    
    
    // 
    // some standard sanity checks
    //
    
    int i, j, r = -EINVAL;
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    
    if (!hm2_md_is_consistent_or_complain(hm2, md_index, 0, 3, 0x40, 0x0007)) {
        HM2_ERR("inconsistent Module Descriptor!\n");
        return -EINVAL;
    }
    
    if (hm2->bspi.num_instances != 0) {
        HM2_ERR(
                "found duplicate Module Descriptor for %s (inconsistent "
                "firmware), not loading driver\n",
                hm2_get_general_function_name(md->gtag)
                );
        return -EINVAL;
    }
    
    if (hm2->config.num_bspis > md->instances) {
        HM2_ERR(
                "config defines %d bspis, but only %d are available, "
                "not loading driver\n",
                hm2->config.num_bspis,
                md->instances
                );
        return -EINVAL;
    }
    
    if (hm2->config.num_bspis == 0) {
        return 0;
    }
    
    // 
    // looks good, start initializing
    // 
    
    if (hm2->config.num_bspis == -1) {
        hm2->bspi.num_instances = md->instances;
    } else {
        hm2->bspi.num_instances = hm2->config.num_bspis;
    }
    
    hm2->bspi.instance = (hm2_bspi_instance_t *)hal_malloc(hm2->bspi.num_instances 
                                                     * sizeof(hm2_bspi_instance_t));
    if (hm2->bspi.instance == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }
    
    for (i = 0 ; i < hm2->bspi.num_instances ; i++){
        hm2_bspi_instance_t *chan = &hm2->bspi.instance[i];
        chan->clock_freq = md->clock_freq;
        r = snprintf(chan->name, sizeof(chan->name), "%s.bspi.%01d", hm2->llio->name, i);
        if (r >= (int)sizeof(chan->name)) {r = -EINVAL ; goto fail0;}
        HM2_PRINT("created Buffered SPI function %s.\n", chan->name);
        chan->base_address = md->base_address + i * md->instance_stride;
        chan->register_stride = md->register_stride;
        chan->instance_stride = md->instance_stride;
        chan->cd_addr = md->base_address + md->register_stride + i * sizeof(rtapi_u32);
        chan->count_addr = md->base_address + 2 * md->register_stride + i * sizeof(rtapi_u32);
        for (j = 0 ; j < 16 ; j++ ){
            chan->addr[j] = chan->base_address + j * sizeof(rtapi_u32);
        }
        
    }
    return hm2->bspi.num_instances;
fail0:
    return r;
}

void hm2_bspi_force_write(hostmot2_t *hm2)
{
    int i, j;
    for (i = 0 ; i < hm2->bspi.num_instances ; i++){
        hm2_bspi_instance_t chan = hm2->bspi.instance[i];
        // write the channel descriptors
        for (j = 15 ; j >=0 ; j--){
            hm2->llio->write(hm2->llio, chan.cd_addr, &(chan.cd[j]), sizeof(rtapi_u32));
        }
    }
}

EXPORT_SYMBOL_GPL(hm2_tram_add_bspi_frame);
int hm2_tram_add_bspi_frame(char *name, int chan, rtapi_u32 **wbuff, rtapi_u32 **rbuff)
{
    hostmot2_t *hm2;
    int i, r;
    i = hm2_get_bspi(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find BSPI instance %s.\n", name);
        return -1;
    }
    if (hm2->bspi.instance[i].conf_flag[chan] != true){
        HM2_ERR("The selected write channel (%i) on bspi instance %s.\n" 
                "Has not been configured.\n", chan, name);
        return -1;
    }
    if (wbuff != NULL) {
        r = hm2_register_tram_write_region(hm2,hm2->bspi.instance[i].addr[chan], sizeof(rtapi_u32),wbuff);
        if (r < 0) {
            HM2_ERR("Failed to add TRAM write entry for %s.\n", name);
            return -1;
        }
    } else {
        HM2_ERR("SPI frame must have a write entry for channel (%i) on %s.\n", chan, name);
        return -1;
    }
    bool will_echo = !(hm2->bspi.instance[i].cd[chan] & 0x80000000);
    bool has_rbuff = rbuff != NULL;
    if (will_echo != has_rbuff) {
        HM2_ERR("SPI frame must have a read entry for channel (%i) on %s.\n", chan, name);
        return -1;
    }
    if (rbuff != NULL){
        // Reading from addr[0] instead of addr[chan] is intentional
        // here - all the channels share one receive FIFO.
        r = hm2_register_tram_read_region(hm2,hm2->bspi.instance[i].addr[0], sizeof(rtapi_u32),rbuff);
        if (r < 0) {
            HM2_ERR( "Failed to add TRAM read entry for %s\n", name);
            return -1;
        }
    }
    
    return 0;
}

EXPORT_SYMBOL_GPL(hm2_allocate_bspi_tram);
int hm2_allocate_bspi_tram(char* name)
{
    hostmot2_t *hm2;
    int i, r;
    i = hm2_get_bspi(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find BSPI instance %s.\n", name);
        return -1;
    }
    r = hm2_allocate_tram_regions(hm2);
    if (r < 0) {
        HM2_ERR("Failed to register TRAM for BSPI %s\n", name);
        return -1;
    }
    
    return 0;
}

EXPORT_SYMBOL_GPL(hm2_bspi_clear_fifo);
int hm2_bspi_clear_fifo(char * name)
{
    hostmot2_t * hm2;
    int i, r;

    i = hm2_get_bspi(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find BSPI instance %s.\n", name);
        return -1;
    }
    rtapi_u32 zero = 0;
    r = hm2->llio->write(hm2->llio, hm2->bspi.instance[i].count_addr, &zero, sizeof(rtapi_u32));
    if (r < 0) {
        HM2_ERR("BSPI: hm2->llio->write failure %s\n", name);
    }
    
    return r;
}

EXPORT_SYMBOL_GPL(hm2_bspi_write_chan);
int hm2_bspi_write_chan(char* name, int chan, rtapi_u32 val)
{
    hostmot2_t *hm2;
    rtapi_u32 buff = val;
    int i, r;
    i = hm2_get_bspi(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find BSPI instance %s.\n", name);
        return -1;
    }
    if (hm2->bspi.instance[i].conf_flag[chan] != true){
        HM2_ERR("The selected write channel (%i) on bspi instance %s.\n" 
                "Has not been configured.\n", chan, name);
        return -1;
    }
    r = hm2->llio->write(hm2->llio, hm2->bspi.instance[i].addr[chan], &buff, sizeof(rtapi_u32));
    if (r < 0) {
        HM2_ERR("BSPI: hm2->llio->write failure %s\n", name);
    }
    
    return r;
}

EXPORT_SYMBOL_GPL(hm2_bspi_setup_chan);
int hm2_bspi_setup_chan(char *name, int chan, int cs, int bits, double mhz,
                        int delay, int cpol, int cpha, int noclear, int noecho,
                        int samplelate)
{
    hostmot2_t *hm2;
    rtapi_u32 buff = 0;
    int i;
    double board_mhz;
    i = hm2_get_bspi(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find BSPI instance %s.\n", name);
        return -1;
    }
    if (chan<0 || chan > 15){
        HM2_ERR("BSPI %s: Channel number (%i) is out of range, BSPI only" 
                "supports channels 0-15\n", name, chan);
        return -1;
    }
    if (cs > 15 || cs < 0){
        HM2_ERR("BSPI %s: Chip Select for channel %i (%i) out of range, only "
                "values 0 - 15 are accepted\n", name, chan, cs);
        return -1;
    }
    if (bits > 64 || bits < 1){
        HM2_ERR("BSPI %s: Number of bits for chan %i (%i) is out of range, "
                "BSPI only supports 1-64 bits\n", name, chan, bits);
        return -1;
    }
    if (delay < 0 || delay > 1e6){
        HM2_ERR("The requested frame delay on channel %i of %inS seems "
                "rather implausible for an SPI device. Exiting.\n", delay, chan);
        return -1;
    }
    board_mhz = hm2->bspi.instance[i].clock_freq / 1e6;
    
    // reduce clock rate of the FPGA isn't fast enough
    if (mhz > board_mhz/2){
        mhz=board_mhz/2;
    }

    buff = (noecho != 0) << 31
        |  (noclear != 0) << 30
        |  (samplelate != 0) << 29
        | ((delay <= 0)? 0x10 : (rtapi_u32)((delay*board_mhz/1000.0)-1) & 0x1f) << 24
        | (cs & 0xF) << 16
        | (((rtapi_u16)(board_mhz / (mhz * 2) - 1) & 0xFF)) << 8
        | (cpha != 0) << 7
        | (cpol != 0) << 6
        | (((rtapi_u16)(bits - 1)) & 0x3F);
    HM2_DBG("BSPI %s Channel %i setup %x\n", name, chan, buff);
    hm2->bspi.instance[i].cd[chan] = buff;
    hm2->bspi.instance[i].conf_flag[chan] = true;
    hm2_bspi_force_write(hm2);
    return 0;
}

void hm2_bspi_print_module(hostmot2_t *hm2){
    int i,j;
    if (hm2->bspi.num_instances <= 0) return;
    HM2_PRINT("Buffered SPI: %d\n", hm2->bspi.num_instances);
    HM2_PRINT("    version: %d\n", hm2->bspi.version);
    HM2_PRINT("    channel configurations\n");
    for (i = 0; i < hm2->bspi.num_instances; i ++) {
        HM2_PRINT("    clock_frequency: %d Hz (%s MHz)\n", 
                  hm2->bspi.instance[i].clock_freq, 
                  hm2_hz_to_mhz(hm2->bspi.instance[i].clock_freq));
        HM2_PRINT("    instance %d:\n", i);
        HM2_PRINT("    HAL name = %s\n", hm2->bspi.instance[i].name);
        for (j = 0; j < 16 ; j++){
            HM2_PRINT("         frame %i config = %08x\n", j, 
                      hm2->bspi.instance[i].cd[j]);
            HM2_PRINT("                address = %08x\n", 
                      hm2->bspi.instance[i].addr[j]);
        }
    }
}

EXPORT_SYMBOL_GPL(hm2_bspi_set_read_function);
int hm2_bspi_set_read_function(char *name, int (*func)(void *subdata), void *subdata){
    hostmot2_t *hm2;
    int i;
    i = hm2_get_bspi(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find BSPI instance %s.\n", name);
        return -1;
    }
    if (func == NULL) { 
        HM2_ERR("Invalid function pointer passed to "
                "hm2_bspi_set_read_function.\n");
        return -1;
    }
    if (subdata == NULL) { 
        HM2_ERR("Invalid data pointer passed to "
                "hm2_bspi_set_read_function.\n");
        return -1;
    }
    hm2->bspi.instance[i].read_function = func;
    hm2->bspi.instance[i].subdata = subdata;
    return 0;
}

EXPORT_SYMBOL_GPL(hm2_bspi_set_write_function);
int hm2_bspi_set_write_function(char *name, int (*func)(void *subdata), void *subdata){
    hostmot2_t *hm2;
    int i;
    i = hm2_get_bspi(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find BSPI instance %s.\n", name);
        return -1;
    }
    if (func == NULL) { 
        HM2_ERR("Invalid function pointer passed to "
                "hm2_bspi_set_write_function.\n");
        return -1;
    }
    if (subdata == NULL) { 
        HM2_ERR("Invalid data pointer passed to "
                "hm2_bspi_set_write_function.\n");
        return -1;
    }
    hm2->bspi.instance[i].write_function = func;
    hm2->bspi.instance[i].subdata = subdata;
    return 0;
}

    
void hm2_bspi_process_tram_read(hostmot2_t *hm2, long period)
{
    (void)period;
    int i, r;
    int (*func)(void *subdata);
    for (i = 0 ; i < hm2->bspi.num_instances ; i++ ){
        func = hm2->bspi.instance[i].read_function;
        if (func != NULL){
            r = func(hm2->bspi.instance[i].subdata);
            if(r < 0)
                HM2_ERR("BSPI read function @%p failed (returned %d)\n",
                        func, r);
        }
    }
}

void hm2_bspi_prepare_tram_write(hostmot2_t *hm2, long period)
{
    (void)period;
    int i, r;
    int (*func)(void *subdata);
    for (i = 0 ; i < hm2->bspi.num_instances ; i++ ){
        func = hm2->bspi.instance[i].write_function;
        if (func != NULL){
            r = func(hm2->bspi.instance[i].subdata);
            if(r < 0)
                HM2_ERR("BSPI read function @%p failed (returned %d)\n",
                        func, r);
        }
    }
}

// The following standard Hostmot2 functions are not currently used by bspi. 

void hm2_bspi_cleanup(hostmot2_t *hm2)
{
    (void)hm2;
}

void hm2_bspi_write(hostmot2_t *hm2)
{
    (void)hm2;
}



