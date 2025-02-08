//
//   Copyright (C) 2010 Andy Pugh
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU General Public License for more details.
//
//   You should have received a copy of the GNU General Public License
//   along with this program; if not, write to the Free Software
//   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//


#include <rtapi_slab.h>

#include "rtapi.h"
#include "rtapi_string.h"
#include "rtapi_math.h"
#include "rtapi_math64.h"

#include "hal.h"

#include "hostmot2.h"
#include "bitfile.h"


int getbits(hm2_sserial_remote_t *chan, rtapi_u64 *val, int start, int len){
    //load the bits from the registers in to bit 0+ of *val
    int i;
    
    *val = 0LL;
    for (i = (start + len - 1) / 32; i >= start / 32; i--){
        *val <<= 32;
        *val |= *chan->read[i];
    }
    *val >>= start % 32;
    // mask to leave only the required bits
    *val &= (~0ull >> (64 - len));
    return 0;
}


int setbits(hm2_sserial_remote_t *chan, rtapi_u64 *val, int start, int len){
    //load the bits from *val into the registers
    // Assumes that all registers are zeroed elsewhere as required
    int i;
    *val <<= start % 32;
    for (i = start / 32; i <= (start + len - 1) / 32; i++){
        *chan->write[i] |= (rtapi_u32)*val;
        *val >>= 32;
    }
    return start + len;
}

int hm2_sserial_wait(hostmot2_t *hm2, hm2_sserial_instance_t *inst, long period){
    // real-time wait function (relies on process data)
    *inst->command_reg_write = 0x80000000; // mask pointless writes
    inst->timer -= period;
    *inst->debug = inst->timer;
    if (*inst->command_reg_read != 0) {
        if (inst->timer > 0) {
            return 1;
        }
        HM2_ERR("hm2_sserial_wait: "
                "Timeout waiting for CMD to clear\n");
        return -1;
    }
    if (*(inst->data_reg_read) & (1 < inst->remotes[inst->r_index].index)){
        HM2_ERR("Error after doit clear\n");
        return -1;
    }
    return 0;
}

int hm2_sserial_waitfor(hostmot2_t *hm2, rtapi_u32 addr, rtapi_u32 mask, int ms){
    // standalone wait function
    rtapi_u64 t1, t2;
    rtapi_u32 d;
    t1 = rtapi_get_time();
    do { // wait for addr to clear
        rtapi_delay(50000);
        hm2->llio->read(hm2->llio, addr, &d, sizeof(rtapi_u32));
        t2 = rtapi_get_time();
        if ((rtapi_u32)(t2 - t1) > 1000000L * ms) {
            HM2_ERR("hm2_sserial_waitfor: Timeout (%dmS) waiting for addr %x &"
                    "mask %x val %x\n", ms, addr, mask, d & mask);
            addr += 0x100;
            hm2->llio->read(hm2->llio, addr, &d, sizeof(rtapi_u32));
            HM2_ERR("DATA addr %x after timeout: %x\n", addr, d);
            return -1;
        }
    }while (d & mask);
    return 0;
}

int getlocal8(hostmot2_t *hm2, hm2_sserial_instance_t *inst, int addr){
    rtapi_u32 val = 0;
    rtapi_u32 buff;
    buff = READ_LOCAL_CMD | addr;
    HM2WRITE(inst->command_reg_addr, buff);
    hm2_sserial_waitfor(hm2, inst->command_reg_addr, 0xFFFFFFFF, 22);
    HM2READ(inst->data_reg_addr, buff);
    val = (val << 8) | buff;
    return val;
}

int getlocal32(hostmot2_t *hm2, hm2_sserial_instance_t *inst, int addr){
    rtapi_u32 val = 0;
    int bytes = 4;
    rtapi_u32 buff;
    for (;bytes--;){
        buff = READ_LOCAL_CMD | (addr + bytes);
        HM2WRITE(inst->command_reg_addr, buff);
        hm2_sserial_waitfor(hm2, inst->command_reg_addr, 0xFFFFFFFF, 22);
        HM2READ(inst->data_reg_addr, buff);
        val = (val << 8) | buff;
    }
    return val;
}

int setlocal32(hostmot2_t *hm2, hm2_sserial_instance_t *inst, int addr, int val){
    int bytes = 0;
    rtapi_u32 buff;
    for (;bytes < 4; bytes++){

        if (hm2_sserial_waitfor(hm2, inst->command_reg_addr, 0xFFFFFFFF, 22) < 0) {
            HM2_ERR("Command register not ready\n");
            return -1;
        }

        buff = val & 0xff;
        val >>= 8;
        HM2WRITE(inst->data_reg_addr, buff);
        buff = WRITE_LOCAL_CMD | (addr + bytes);
        HM2WRITE(inst->command_reg_addr, buff);

        if (hm2_sserial_waitfor(hm2, inst->command_reg_addr, 0xFFFFFFFF, 22) < 0) {
            HM2_ERR("Write failure attempting to set baud rate\n");
            return -1;
        }
    }
    return 0;
}

int hm2_sserial_read_nvram_word(hostmot2_t *hm2,
                                hm2_sserial_remote_t *chan,
                                void *data,
                                int addr,
                                int length){
    rtapi_u32 buff;
    buff = 0xEC000000;
    hm2->llio->write(hm2->llio, chan->reg_cs_addr, &buff, sizeof(rtapi_u32));
    buff = 0x01;
    hm2->llio->write(hm2->llio, chan->rw_addr[0], &buff, sizeof(rtapi_u32));
    buff = 0x1000 | (1 << chan->index);
    hm2->llio->write(hm2->llio, chan->command_reg_addr, &buff, sizeof(rtapi_u32));
    if (0 > hm2_sserial_waitfor(hm2, chan->command_reg_addr, 0xFFFFFFFF, 1012)){
        HM2_ERR("Timeout in sserial_read_nvram_word(2)\n");
        goto fail0;
    }
    switch (length){
        case 1:
            buff = READ_REM_BYTE_CMD + addr; break;
        case 2:
            buff = READ_REM_WORD_CMD + addr; break;
        case 4:
            buff = READ_REM_LONG_CMD + addr; break;
        case 8:
            buff = READ_REM_DOUBLE_CMD + addr; break;
        default:
            HM2_ERR("Unsupported global variable bitlength  (length = %i)\n", length);
            return -EINVAL;
    }
    hm2->llio->write(hm2->llio, chan->reg_cs_addr, &buff, sizeof(rtapi_u32));
    buff = 0x1000 | (1 << chan->index);
    hm2->llio->write(hm2->llio, chan->command_reg_addr, &buff, sizeof(rtapi_u32));
    if (0 > hm2_sserial_waitfor(hm2, chan->command_reg_addr, 0xFFFFFFFF, 1013)){
        HM2_ERR("Timeout in sserial_read_nvram_word(4)\n");
        goto fail0;
    }
    hm2->llio->read(hm2->llio, chan->rw_addr[0], data, sizeof(rtapi_u32));

fail0: // attempt to set back to normal access
    buff = 0xEC000000;
    hm2->llio->write(hm2->llio, chan->reg_cs_addr, &buff, sizeof(rtapi_u32));
    buff = 0x00;
    hm2->llio->write(hm2->llio, chan->rw_addr[0], &buff, sizeof(rtapi_u32));
    buff = 0x1000 | (1 << chan->index);
    hm2->llio->write(hm2->llio, chan->command_reg_addr, &buff, sizeof(rtapi_u32));
    if (0 > hm2_sserial_waitfor(hm2, chan->command_reg_addr, 0xFFFFFFFF, 1014)){
        HM2_ERR("Timeout in sserial_read_nvram_word(6)\n");
        return -EINVAL;
    }
    return 0;
}

int check_set_baudrate(hostmot2_t *hm2, hm2_sserial_instance_t *inst){
    rtapi_u32 baudrate;
    int baudaddr;
    int lbpstride;
    rtapi_u32 buff;
    int c;

    if (hm2->sserial.baudrate < 0){ return 0;}
    if (hm2->sserial.version < 34) {
    HM2_ERR("Setting baudrate is not supported in the current firmware version\n"
    "Version must be > v33 and you have version %i.", hm2->sserial.version);
    return -EINVAL;
    }
    lbpstride = getlocal8(hm2, inst, SSLBPCHANNELSTRIDELOC);
    HM2_PRINT("num_channels = %i\n", inst->num_channels);
    for (c = 0; c < inst->num_channels; c++){
        baudaddr = getlocal8(hm2, inst, SSLBPCHANNELSTARTLOC) + (c * lbpstride) + 42;
        baudrate = getlocal32(hm2, inst, baudaddr);
        HM2_PRINT("Chan %i baudrate = %i\n", c, baudrate);
        if (baudrate != (unsigned)hm2->sserial.baudrate) {
            if (setlocal32(hm2, inst, baudaddr, hm2->sserial.baudrate) < 0) {
                HM2_ERR("Problem setting new baudrate, power-off reset may be needed to"
                        " recover from this.\n");
                return -EINVAL;
            }
            baudrate = getlocal32(hm2, inst, baudaddr);
            HM2_PRINT("Chan %i. Baudrate set to %i\n", c, baudrate);
        }
    }
    buff = 0x800; HM2WRITE(inst->command_reg_addr, buff); // stop all

    return 0;
}

int hm2_sserial_stopstart(hostmot2_t *hm2, hm2_module_descriptor_t *md,
                          hm2_sserial_instance_t *inst, rtapi_u32 start_mode){
    rtapi_u32 buff, addr;
    int i = inst->index;
    int c;

    buff=0x800; //Stop All
    hm2->llio->write(hm2->llio, inst->command_reg_addr, &buff, sizeof(rtapi_u32));
    if (hm2_sserial_waitfor(hm2, inst->command_reg_addr, 0xFFFFFFFF,51) < 0){
        return -EINVAL;
    }

    for (c = 0 ; c < inst->num_channels ; c++){
        if (hm2->config.sserial_modes[i][c] != 'x'){
            start_mode |= 1 << c;
            HM2_DBG("Start-mode = %x\n", start_mode);
            // CS addr - write card mode
            addr = md->base_address + 2 * md->register_stride
            + i * md->instance_stride + c * sizeof(rtapi_u32);
            buff = (hm2->config.sserial_modes[i][c] - '0') << 24;
            hm2->llio->write(hm2->llio, addr, &buff, sizeof(rtapi_u32));
        }
    }
    hm2->llio->write(hm2->llio, inst->command_reg_addr, &start_mode, sizeof(rtapi_u32));
    if (hm2_sserial_waitfor(hm2, inst->command_reg_addr, 0xFFFFFFFF, 8000) < 0){
        return -EINVAL;
    }
    return 0;
}

int hm2_sserial_get_bytes(hostmot2_t *hm2,
                          hm2_sserial_remote_t *chan,
                          void *buffer,
                          int addr,
                          int size ){
    // Gets the bytes one at a time. This could be done more efficiently.
    char *ptr;
    rtapi_u32 data;
    int string = size;
    // -1 in size means "find null" for strings. -2 means don't lcase

    ptr = (char*)buffer;
    while(0 != size){
        data = 0x4C000000 | addr++;
        hm2->llio->write(hm2->llio, chan->reg_cs_addr, &data, sizeof(rtapi_u32));

        if (0 > hm2_sserial_waitfor(hm2, chan->reg_cs_addr, 0x0000FF00, 50)){
            HM2_ERR("Timeout trying to read config data in sserial_get_bytes\n");
            return -EINVAL;
        }
        data = 0x1000 | (1 << chan->index);
        hm2->llio->write(hm2->llio, chan->command_reg_addr, &data, sizeof(rtapi_u32));

        if (0 > hm2_sserial_waitfor(hm2, chan->command_reg_addr, 0xFFFFFFFF, 51)){
            HM2_ERR("Timeout during do-it in sserial_get_bytes\n");
            return -EINVAL;
        }

        hm2->llio->read(hm2->llio, chan->rw_addr[0], &data, sizeof(rtapi_u32));
        data &= 0x000000FF;
        size--;
        if (size < 0) { // string data
            if (data == 0 || size < (-HM2_SSERIAL_MAX_STRING_LENGTH)){
                size = 0;
            } else if (string > -2 && data >= 'A' && data <= 'Z') {
                data |= 0x20; // lower case
            }
        }

        *(ptr++) = (unsigned char)data;
    }
    return addr;
}

void config_8i20(hostmot2_t *hm2, hm2_sserial_remote_t *chan){
    rtapi_u32 buff;
    chan->num_modes=0;
    chan->num_confs = sizeof(hm2_8i20_params) / sizeof(hm2_sserial_data_t);
    chan->confs = rtapi_kzalloc(sizeof(hm2_8i20_params),RTAPI_GFP_KERNEL);
    memcpy(chan->confs, hm2_8i20_params, sizeof(hm2_8i20_params));

    //8i20 has reprogrammable current scaling:
    buff = 0;
    hm2_sserial_get_bytes(hm2, chan, &buff, 0x8E8, 2);
    chan->confs[1].ParmMax = buff * 0.01;
    chan->confs[1].ParmMin = buff * -0.01;
    chan->globals = rtapi_kzalloc(sizeof(hm2_8i20_globals), RTAPI_GFP_KERNEL);
    memcpy(chan->globals, hm2_8i20_globals, sizeof(hm2_8i20_globals));
    chan->num_globals = sizeof(hm2_8i20_globals) / sizeof(hm2_sserial_data_t);
}

void config_7i64(hostmot2_t *hm2, hm2_sserial_remote_t *chan){
    (void)hm2;
    chan->num_modes=0;
    chan->num_confs = sizeof(hm2_7i64_params) / sizeof(hm2_sserial_data_t);
    chan->confs = rtapi_kzalloc(sizeof(hm2_7i64_params), RTAPI_GFP_KERNEL);
    memcpy(chan->confs, hm2_7i64_params, sizeof(hm2_7i64_params));
}

int hm2_sserial_get_param_value(hostmot2_t *hm2,
                                hm2_sserial_remote_t *chan,
                                int index,
                                int set_hal){
    hm2_sserial_params_t *p;
    hm2_sserial_data_t *g;
    int r = 0;

    if (index >= chan->num_globals || index < 0) return -1;

    p = &(chan->params[index]);
    g = &(chan->globals[index]);

    switch (chan->globals[index].DataType) {
        case LBP_PAD:
            break;
        case LBP_BITS:
            break;
        case LBP_UNSIGNED:
            r = hm2_sserial_get_bytes(hm2, chan, (void*)&(p->u32_written),
                                      g->ParmAddr, g->DataLength/8);
            if (r < 0) {HM2_ERR("SSerial Parameter read error\n") ; return -EINVAL;}
            if (set_hal) p->u32_param = p->u32_written;
            HM2_DBG("LBP_UNSIGNED %i %i \n", p->u32_param, p->u32_written);
            if ((strcmp(g->NameString, "swrevision") == 0) && (p->u32_param < 14)) {
                HM2_ERR("Warning: sserial remote device %s channel %d has old firmware that should be updated\n", chan->raw_name, chan->index);
            }
            break;
        case LBP_SIGNED:
            r = hm2_sserial_get_bytes(hm2, chan, (void*)&(p->s32_written),
                                      g->ParmAddr, g->DataLength/8);
            if (set_hal) p->s32_param = p->s32_written;
            HM2_DBG("LBP_SIGNED %i %i \n", p->s32_param, p->s32_written);
            break;
        case LBP_NONVOL_UNSIGNED:
            r = hm2_sserial_read_nvram_word(hm2, chan, (void*)&(p->u32_written),
                                                g->ParmAddr,
                                                g->DataLength/8);
            if (set_hal) p->u32_param = p->u32_written;
            HM2_DBG("LBP_NONVOL_UNSIGNED %i %i \n", p->u32_param, p->u32_written);
            break;
        case LBP_NONVOL_SIGNED:
            r = hm2_sserial_read_nvram_word(hm2, chan, (void*)&(p->s32_written),
                                                g->ParmAddr,
                                                g->DataLength/8);
            if (set_hal) p->s32_param = p->s32_written;
        case LBP_STREAM:
            break; // Have not seen a stream type yet
        case LBP_BOOLEAN:
            break;
        case LBP_ENCODER:
            break; // Hard to imagine an encoder not in Process data
        case LBP_FLOAT:
            {
                char buf[HM2_SSERIAL_MAX_DATALENGTH/8];
                r = hm2_sserial_get_bytes(hm2, chan, &buf[0], g->ParmAddr, g->DataLength/8);
                if (g->DataLength == sizeof(float) * 8) {
                    float temp;
                    memcpy((void*)&temp, &buf[0], sizeof(float));
                    p->float_written = temp;
                } else if (g->DataLength == sizeof(double) * 8) {
                    double temp;
                    memcpy((void*)&temp, &buf[0], sizeof(double));
                    p->float_written = temp;
                } else {
                    HM2_ERR("sserial get param value: LBP_FLOAT of bit-length %i not handled\n", g->DataLength);
                }
            }
            if (set_hal) p->float_param = p->float_written;
            HM2_DBG("LBP_FLOAT %f %f \n", p->float_param, p->float_written);
            break;
        case LBP_ENCODER_H:
        case LBP_ENCODER_L:
            break; // Hard to imagine an encoder not in Process data
        default:
            HM2_PRINT("Unsupported datatype %02X\n", chan->globals[index].DataType);
    }
    if (r < 0) {HM2_ERR("SSerial Parameter read error\n") ; return -EINVAL;}
    return 0;
}

int hm2_sserial_create_params(hostmot2_t *hm2, hm2_sserial_remote_t *chan){
    int i, r;
    hm2_sserial_data_t global;
    int hal_dir;

    chan->params = hal_malloc(chan->num_globals * sizeof(hm2_sserial_params_t));
    for (i = 0 ; i < chan->num_globals ; i++){
        global = chan->globals[i];

        r = 0;

        hal_dir = (global.DataDir == LBP_IN) ? HAL_RO : HAL_RW;

        chan->params[i].type = global.DataType;
        switch (chan->params[i].type) {
            case LBP_BITS:
                break;
            case LBP_UNSIGNED:
            case LBP_NONVOL_UNSIGNED:
                r = hal_param_u32_newf(hal_dir,
                                       &(chan->params[i].u32_param),
                                       hm2->llio->comp_id,
                                       "%s.%s",
                                       chan->name,
                                       global.NameString);
                if (r < 0) {HM2_ERR("Out of memory\n") ; return -ENOMEM;}
                break;
            case LBP_SIGNED:
            case LBP_NONVOL_SIGNED:
                r = hal_param_s32_newf(hal_dir,
                                       &(chan->params[i].s32_param),
                                       hm2->llio->comp_id,
                                       "%s.%s",
                                       chan->name,
                                       global.NameString);
                if (r < 0) {HM2_ERR("Out of memory\n") ; return -ENOMEM;}
                break;
            case LBP_FLOAT:
            case LBP_NONVOL_FLOAT:
                r = hal_param_float_newf(hal_dir,
                                       &(chan->params[i].float_param),
                                       hm2->llio->comp_id,
                                       "%s.%s",
                                       chan->name,
                                       global.NameString);
                if (r < 0) {HM2_ERR("Out of memory\n") ; return -ENOMEM;}
            case LBP_STREAM: // Don't anticipate seeing these as params
            case LBP_BOOLEAN:
            case LBP_ENCODER:
            case LBP_ENCODER_H:
            case LBP_ENCODER_L:
                break;
        }

        hm2_sserial_get_param_value(hm2, chan, i, 1);

    }
    return 0;
}



int hm2_sserial_get_globals_list(hostmot2_t *hm2, hm2_sserial_remote_t *chan){

    int gtoc, addr, buff;

    hm2_sserial_data_t data;

    chan->num_globals = 0;
    hm2->llio->read(hm2->llio, chan->rw_addr[2], &buff, sizeof(rtapi_u32));
    gtoc=(buff & 0xffff0000) >> 16;
    if (gtoc == 0){
        if (hm2->sserial.baudrate == 115200) {
            HM2_DBG("Setup mode, creating no pins for smart-serial channel %s\n",
                      chan->name);
            chan->num_confs = 0;
            chan->num_globals = 0;
            return 0;
        }
        else if (strstr(chan->name, "8i20")){
            config_8i20(hm2, chan);
        }
        else if (strstr(chan->name, "7i64")){
            config_7i64(hm2, chan);
        }
        else { HM2_ERR("No GTOC in sserial read globals\n"); return -1;}
    }
    else
    {
        do {
            int i;
            addr = 0;
            gtoc = hm2_sserial_get_bytes(hm2, chan, &addr, gtoc, 2);
            if (((addr &= 0xFFFF) <= 0) || (gtoc < 0)) break;
            if ((addr = hm2_sserial_get_bytes(hm2, chan, &data, addr, 14)) < 0) {
                return -EINVAL;
            }
            // process is a subset of global. The only way to tell is to compare
            for (i = 0; i <= chan->num_confs ; i ++) {
                if (chan->confs[i].ParmAddr == data.ParmAddr){i = 1000;}
            }
            if (data.RecordType == LBP_DATA && i < 1000) {
                addr = hm2_sserial_get_bytes(hm2, chan, &(data.UnitString), addr, -1);
                if (addr < 0){ return -EINVAL;}
                addr = hm2_sserial_get_bytes(hm2, chan, &(data.NameString), addr, -1);
                if (addr < 0){ return -EINVAL;}
                HM2_DBG("Global: %s  RecordType: %02X Datatype: %02X Dir: %02X Addr: %04X Length: %i\n",
                           data.NameString, data.RecordType, data.DataType, data.DataDir, data.ParmAddr, data.DataLength);
                chan->num_globals++;
                chan->globals = (hm2_sserial_data_t *)
                         rtapi_krealloc(chan->globals,
                         chan->num_globals * sizeof(hm2_sserial_data_t),
                         RTAPI_GFP_KERNEL);
                chan->globals[chan->num_globals - 1] = data;
            }
            else if (data.RecordType== LBP_MODE){
                char * type;
                hm2_sserial_mode_t mode;
                // We assumed a 14-byte LBP_DATA before we found it wasn't
                addr -= 14;
                addr = hm2_sserial_get_bytes(hm2, chan, &mode, addr, 4);
                addr = hm2_sserial_get_bytes(hm2, chan, &mode.NameString, addr, -1);
                type = (mode.ModeType == 0x01)? "Software" : "Hardware";
                rtapi_print("Board %s %s Mode %i = %s\n",
                            chan->name,
                            type,
                            mode.ModeIndex,
                            mode.NameString);
            }
        } while (addr > 0);
    }

    if ( hm2_sserial_create_params(hm2, chan) < 0) {
        HM2_ERR("Failed to create parameters for device %s\n", chan->name);
        return -EINVAL;
    }

    return 0;
}

// Main functions

int hm2_sserial_parse_md(hostmot2_t *hm2, int md_index){
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    int i, c;
    int pin = -1;
    int port;
    rtapi_u32 ddr_reg, src_reg, buff;
    int r = -EINVAL;
    int count = 0;
    int chan_counts[] = {0,0,0,0,0,0,0,0};

    hm2->sserial.version = md->version;

    //
    // some standard sanity checks
    //

    switch(md->gtag){
        case HM2_GTAG_SMARTSERIAL:
            if (hm2_md_is_consistent(hm2, md_index, 0, 5, 0x40, 0x001F)) {
                HM2_ERR("The bitfile contains Smart Serial modules for a firmware "
                        "revision < rev22. This Driver now requires rev22 or newer "
                        "firmwares\n");
                return -EINVAL;
            }

            if (!hm2_md_is_consistent_or_complain(hm2, md_index, 0, 6, 0x40, 0x003C)) {
                HM2_ERR("inconsistent Module Descriptor!\n");
                return -EINVAL;
            }
            break;
        case HM2_GTAG_SMARTSERIALB:
            if (!hm2_md_is_consistent_or_complain(hm2, md_index, 0, 10, 0x40, 0x03FC)) {
                HM2_ERR("inconsistent Module Descriptor!\n");
                return -EINVAL;
            }
            break;
    }
    if (hm2->sserial.num_instances != 0) {
        HM2_ERR(
                "found duplicate Module Descriptor for %s (inconsistent firmwar"
                "e), not loading driver\n",
                hm2_get_general_function_name(md->gtag)
                );
        return -EINVAL;
    }
    if (hm2->config.num_sserials > md->instances) {
        HM2_ERR(
                "num_sserials references %d instances, but only %d are available"
                ", not loading driver\n",
                hm2->config.num_sserials,
                md->instances
                );
        return -EINVAL;
    }

    //
    // looks good, start initializing
    //

    if (hm2->config.num_sserials == -1) {
        hm2->sserial.num_instances = md->instances;
    } else {
        hm2->sserial.num_instances = hm2->config.num_sserials;
    }
    //num_instances may be revised down depending on detected hardware
    HM2_DBG("sserial_num_instances = %i\n", hm2->sserial.num_instances);

    // allocate the per-instance HAL shared memory
    hm2->sserial.instance = (hm2_sserial_instance_t *)
    hal_malloc(hm2->sserial.num_instances * sizeof(hm2_sserial_instance_t));
    if (hm2->sserial.instance == NULL) {
        HM2_ERR("hm2_sserial_parse_md: hm2_sserial_instance: out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }
    // We can't create the pins until we know what is on each channel, and
    // can't communicate until the pin directions are set up.

    // Temporarily enable the pins that are not masked by sserial_mode

    for (port  = 0; port < hm2->ioport.num_instances; port ++) {
        ddr_reg = 0;
        src_reg = 0;
        for (unsigned port_pin = 0 ; port_pin < hm2->idrom.port_width; port_pin ++){
            pin++;
            if (hm2->pin[pin].sec_tag == HM2_GTAG_SMARTSERIAL
                || hm2->pin[pin].sec_tag == HM2_GTAG_SMARTSERIALB) {
                // look for highest-indexed pin to determine number of channels
                if ((hm2->pin[pin].sec_pin & 0x0F) > chan_counts[hm2->pin[pin].sec_unit]) {
                    chan_counts[hm2->pin[pin].sec_unit] = (hm2->pin[pin].sec_pin & 0x0F);
                }
                // check if the channel is enabled
                HM2_DBG("port %i sec unit = %i, sec pin = %i mode=%c\n", port, hm2->pin[pin].sec_unit, hm2->pin[pin].sec_pin & 0x0F,
                               hm2->config.sserial_modes[hm2->pin[pin].sec_unit][(hm2->pin[pin].sec_pin & 0x0F) - 1] );
                if (hm2->config.sserial_modes[hm2->pin[pin].sec_unit]
                                        [(hm2->pin[pin].sec_pin & 0x0F) - 1] != 'x') {
                    src_reg |= (1 << port_pin);
                    if (hm2->pin[pin].sec_pin & 0x80){ ddr_reg |= (1 << port_pin); }
                }
            }
        }

        hm2->llio->write(hm2->llio, hm2->ioport.ddr_addr + 4 * port,
                         &ddr_reg, sizeof(rtapi_u32));
        hm2->llio->write(hm2->llio, hm2->ioport.alt_source_addr + 4 * port,
                         &src_reg, sizeof(rtapi_u32));

    }

    // Now iterate through the sserial instances, seeing what is on the enabled pins.
    for (i = 0 ; i < md->instances ; i++) {
        hm2_sserial_instance_t *inst = &hm2->sserial.instance[count];
        inst->index = i;
        inst->num_channels = chan_counts[i];
        inst->command_reg_addr = md->base_address + i * md->instance_stride;
        inst->data_reg_addr = md->base_address + i * md->instance_stride + md->register_stride;

        buff=0x4000; //Reset
        HM2WRITE(inst->command_reg_addr, buff);
        buff=0x0001; //Clear
        HM2WRITE(inst->command_reg_addr, buff);
        if (hm2_sserial_waitfor(hm2, inst->command_reg_addr, 0xFFFFFFFF,1007) < 0){
            r = -EINVAL;
            goto fail0;
        }
        do {
            buff=getlocal8(hm2, inst, SSLBPMINORREVISIONLOC);
        } while (buff == 0xAA);
        HM2_PRINT("Smart Serial Firmware Version %i\n",buff);
        hm2->sserial.version = buff;

        r = check_set_baudrate(hm2, inst);
        if (r < 0) goto fail0;

        //start up in setup mode
        r = hm2_sserial_stopstart(hm2, md, inst, 0xF00);
        if(r < 0) {goto fail0;}

        inst->num_remotes = 0;

        for (c = 0 ; c < inst->num_channels ; c++) {
            rtapi_u32 addr0, addr1, addr2;
            rtapi_u32 user0, user1, user2;

            addr0 = md->base_address + 3 * md->register_stride
                                    + i * md->instance_stride + c * sizeof(rtapi_u32);
            HM2READ(addr0, user0);
            HM2_DBG("Inst %i Chan %i Addr %x User0 = %x\n", i, c, addr0, user0);

            addr1 = md->base_address + 4 * md->register_stride
                                    + i * md->instance_stride + c * sizeof(rtapi_u32);
            HM2READ(addr1, user1);
            HM2_DBG("Inst %i Chan %i Addr %x User1 = %x\n", i, c, addr1, user1);

            addr2 = md->base_address + 5 * md->register_stride
            + i * md->instance_stride + c * sizeof(rtapi_u32);
            HM2READ(addr2, user2);
            HM2_DBG("Inst %i Chan %i Addr %x User2 = %x\n", i, c, addr2, user2);

            if (hm2->sserial.baudrate == 115200
                && hm2->config.sserial_modes[i][c] != 'x') { //setup mode
                rtapi_print("Setup mode\n");
                if ((user1 & 0xFF00) == 0x4900){ //XiXXboard

                    rtapi_print("found a %4s\n", (char*)&user1);

                    inst->num_remotes += 1;
                    inst->tag |= 1<<c;
                }
                else { // Look for 8i20s with the CRC check off
                    int lbpstride = getlocal8(hm2, inst, SSLBPCHANNELSTRIDELOC);
                    int crc_addr = getlocal8(hm2, inst, SSLBPCHANNELSTARTLOC)
                    + (c * lbpstride) + 30;

                    rtapi_print("Looking for 8i20s, crc_addr = %i\n", crc_addr);

                    if (getlocal8(hm2, inst, SSLBPMINORREVISIONLOC) < 37 ){
                        HM2_PRINT("Unable to check for 8i20s with firmware < 37 "
                                  "If you are not trying to reflash an 8i20 then"
                                  " ignore this message.\n");
                    }
                    else if (setlocal32(hm2, inst, crc_addr, 0xFF) < 0) {
                        HM2_ERR("Unable to disable CRC to check for old 8i20s");
                    }
                    else if ( (r = hm2_sserial_stopstart(hm2, md, inst, 0xF00)) < 0) {
                        goto fail0;
                    }
                    else {
                        HM2READ(addr1, user1);

                        rtapi_print("found a %4s\n", (char*)&user1);

                        if ((user1 & 0xFF00) == 0x4900){ //XiXXboard
                            inst->num_remotes += 1;
                            inst->tag |= 1<<c;
                        }
                    }
                }
            }
            else if ((user2 & 0x0000ffff) // Parameter discovery
                     || user1 == HM2_SSERIAL_TYPE_7I64 //predate discovery
                     || user1 == HM2_SSERIAL_TYPE_8I20 ){
                inst->num_remotes += 1;
                inst->tag |= 1<<c;
            }

            // nothing connected, or masked by config or wrong baudrate or.....
            // make the pins into GPIO.
            else if (user1 == 0
                     || (inst->tag & 1<<c) == 0
                     || hm2->config.sserial_modes[i][c] == 'x'){
                for (pin = 0 ; pin < hm2->num_pins ; pin++){
                    if (hm2->pin[pin].sec_tag == HM2_GTAG_SMARTSERIAL
                     || hm2->pin[pin].sec_tag == HM2_GTAG_SMARTSERIALB){
                        if((hm2->pin[pin].sec_pin & 0x0F) - 1  == c
                        && hm2->pin[pin].sec_unit == i){
                            hm2->pin[pin].sec_tag = 0;
                            HM2_DBG("Masking pin %i\n", pin);
                        }
                    }
                }
            }
            else if (hm2->config.sserial_modes[i][c] != 'x'){
                HM2_ERR("Unsupported Device (%4s) found on sserial %d "
                        "channel %d\n", (char*)&user1, i, c);
            }
        }
        if (inst->num_remotes > 0){
            if ((r = hm2_sserial_setup_channel(hm2, inst, i)) < 0 ) {
                HM2_ERR("Smart Serial setup failure on instance %i\n",
                        inst->device_id);
                goto fail0;}
            if ((r = hm2_sserial_setup_remotes(hm2, inst, md)) < 0 ) {
                HM2_ERR("Remote setup failure on instance %i\n",
                        inst->device_id);
                goto fail0;}
            // Nothing happens without a "Do It" command
            if ((r = hm2_register_tram_write_region(hm2,inst->command_reg_addr,
                                       sizeof(rtapi_u32),
                                       &inst->command_reg_write)) < 0){
                HM2_ERR("error registering tram DoIt write to sserial "
                "command register (%d)\n", i);
                goto fail0;}

            if ((r = hm2_sserial_stopstart(hm2, md, inst, 0x900)) < 0 ){
                HM2_ERR("Failed to restart device %i on instance\n",
                        inst->device_id);
                goto fail0;}
            if ((r = hm2_sserial_check_local_errors(hm2, inst)) < 0) {
                //goto fail0; // Ignore it for the moment.
            }
            //only increment the instance index if this one is populated
            //otherwise the "slot" is re-used to keep active ports
            //contiguous in the array
            count++ ;
        }
    }

    hm2->sserial.num_instances = count; // because of the extra increment

    // Stop the sserial ports.
    buff=0x800; //Stop All
    for (i = 0 ; i < hm2->sserial.num_instances ; i++) {
        hm2_sserial_instance_t *inst = &hm2->sserial.instance[i];
        hm2->llio->write(hm2->llio, inst->command_reg_addr, &buff, sizeof(rtapi_u32));
        if (hm2_sserial_waitfor(hm2, inst->command_reg_addr, 0xFFFFFFFF,51) < 0){
            return -EINVAL;
        }
    }
    // Return the physical ports to default
    ddr_reg = 0;
    src_reg = 0;
    for (port  = 0; port < hm2->ioport.num_instances; port ++) {
        hm2->llio->write(hm2->llio, hm2->ioport.ddr_addr + 4 * port,
                         &ddr_reg, sizeof(rtapi_u32));
        hm2->llio->write(hm2->llio, hm2->ioport.alt_source_addr + 4 * port,
                         &src_reg, sizeof(rtapi_u32));
    }
    return hm2->sserial.num_instances;

fail0:
    hm2_sserial_cleanup(hm2);
    hm2->sserial.num_instances = 0;
    return r;
}

int hm2_sserial_setup_channel(hostmot2_t *hm2, hm2_sserial_instance_t *inst, int index){
    int r;

    r = hal_pin_s32_newf(HAL_OUT, &(inst->debug),
                         hm2->llio->comp_id,
                         "%s.%i.debug",
                         hm2->llio->name, index);
    if (r < 0) {
        HM2_ERR("error adding pin %s.sserial.%1d.run. aborting\n",
                hm2->llio->name, index);
        return -EINVAL;
    }

    r = hal_pin_bit_newf(HAL_IN, &(inst->run),
                         hm2->llio->comp_id,
                         "%s.sserial.port-%1d.run",
                         hm2->llio->name, index);
    if (r < 0) {
        HM2_ERR("error adding pin %s.sserial.%1d.run. aborting\n",
                hm2->llio->name, index);
        return -EINVAL;
    }
    *inst->run = true;

    r = hal_pin_u32_newf(HAL_OUT, &(inst->state),
                         hm2->llio->comp_id,
                         "%s.sserial.port-%1d.port_state",
                         hm2->llio->name, index);
    if (r < 0) {
        HM2_ERR("error adding pin %s.sserial.%1d.port_state. aborting\n",
                hm2->llio->name, index);
        return -EINVAL;
    }
    r = hal_pin_u32_newf(HAL_OUT, &(inst->state2),
                         hm2->llio->comp_id,
                         "%s.sserial.port-%1d.port_state2",
                         hm2->llio->name, index);
    if (r < 0) {
        HM2_ERR("error adding pin %s.sserial.%1d.port_state. aborting\n",
                hm2->llio->name, index);
        return -EINVAL;
    }
     r = hal_pin_u32_newf(HAL_OUT, &(inst->state3),
                         hm2->llio->comp_id,
                         "%s.sserial.port-%1d.port_state3",
                         hm2->llio->name, index);
    if (r < 0) {
        HM2_ERR("error adding pin %s.sserial.%1d.port_state. aborting\n",
                hm2->llio->name, index);
        return -EINVAL;
    }

    r = hal_pin_u32_newf(HAL_OUT, &(inst->fault_count),
                         hm2->llio->comp_id,
                         "%s.sserial.port-%1d.fault-count",
                         hm2->llio->name, index);
    if (r < 0) {
        HM2_ERR("error adding pin %s.sserial.%1d.fault-count. aborting\n",
                hm2->llio->name, index);
        return -EINVAL;
    }
    r = hal_param_u32_newf(HAL_RW, &(inst->fault_inc),
                           hm2->llio->comp_id,
                           "%s.sserial.port-%1d.fault-inc",
                           hm2->llio->name, index);
    if (r < 0) {
        HM2_ERR("error adding parameter %s.sserial.port-%1d.fault-inc"
                " aborting\n",hm2->llio->name, index);
        return -EINVAL;
    }

    r = hal_param_u32_newf(HAL_RW, &(inst->fault_dec),
                           hm2->llio->comp_id,
                           "%s.sserial.port-%1d.fault-dec",
                           hm2->llio->name, index);
    if (r < 0) {
        HM2_ERR("error adding parameter %s.sserial.port-%1d.fault-dec"
                " aborting\n",hm2->llio->name, index);
        return -EINVAL;
    }

    r = hal_param_u32_newf(HAL_RW, &(inst->fault_lim),
                           hm2->llio->comp_id,
                           "%s.sserial.port-%1d.fault-lim",
                           hm2->llio->name, index);
    if (r < 0) {
        HM2_ERR("error adding parameter %s.sserial.port-%1d.fault-lim"
                " aborting\n",hm2->llio->name, index);
        return -EINVAL;
    }
    //parameter defaults;
    inst->fault_dec = 1;
    inst->fault_inc = 10;
    inst->fault_lim = 200;

    // setup read-back in all modes

    r = hm2_register_tram_read_region(hm2, inst->command_reg_addr,
                                      sizeof(rtapi_u32),
                                      &inst->command_reg_read);
    if (r < 0) {
        HM2_ERR("error registering tram read region for sserial"
                "command register (%d)\n", index);
        return -EINVAL;
    }

    r = hm2_register_tram_read_region(hm2, inst->data_reg_addr,
                                      sizeof(rtapi_u32),
                                      &inst->data_reg_read);
    if (r < 0) {
        HM2_ERR("error registering tram read region for sserial "
                "command register (%d)\n", index);
        return -EINVAL;

    }
    return 0;
}

int hm2_sserial_setup_remotes(hostmot2_t *hm2,
                              hm2_sserial_instance_t *inst,
                              hm2_module_descriptor_t *md) {
    int c, r, i;
    int buff;

    inst->remotes =
    (hm2_sserial_remote_t *)rtapi_kzalloc(inst->num_remotes*sizeof(hm2_sserial_remote_t),
                                    RTAPI_GFP_KERNEL);
    if (inst->remotes == NULL) {
        HM2_ERR("out of memory!\n");
        return -ENOMEM;
    }
    r = -1;
    for (c = 0 ; c < inst->num_channels ; c++ ) {
        if (inst->tag & (1 << c)) {
            hm2_sserial_remote_t *chan = &inst->remotes[++r];
            chan->num_confs = 0;
            chan->num_modes = 0;
            chan->command_reg_addr = inst->command_reg_addr;
            chan->myinst = inst->index;
            chan->data_reg_addr= inst->data_reg_addr;
            chan->index = c;
            HM2_DBG("Instance %i, channel %i / %i\n", inst->index, c, r);
            chan->reg_cs_addr = md->base_address + 2 * md->register_stride
            + inst->index * md->instance_stride + c * sizeof(rtapi_u32);
            HM2_DBG("reg_cs_addr = %x\n", chan->reg_cs_addr);
            // Assume that all process data registers are in use for now. 
            for (i = 0; i < HM2_SSERIAL_NUMREGS; i++){
                chan->rw_addr[i] = md->base_address + (3 + i ) * md->register_stride
                + inst->index * md->instance_stride + c * sizeof(rtapi_u32);
                HM2_DBG("rw_addr[%i] = %x\n", i, chan->rw_addr[i]);
            }
            
            // Get the board ID and name before it is over-written by DoIts
            hm2->llio->read(hm2->llio, chan->rw_addr[0],
                            &buff, sizeof(rtapi_u32));
            chan->serialnumber = buff;
            HM2_DBG("BoardSerial %08x\n", chan->serialnumber);
            hm2->llio->read(hm2->llio, chan->rw_addr[1], chan->raw_name, sizeof(rtapi_u32));
            chan->raw_name[1] |= 0x20; ///lower case
            if (hm2->use_serial_numbers){
                rtapi_snprintf(chan->name, sizeof(chan->name),
                               "hm2_%2s.%04x",
                               chan->raw_name,
                               (chan->serialnumber & 0xffff));
            } else {
                rtapi_snprintf(chan->name, sizeof(chan->name),
                               "%s.%2s.%d.%d",
                               hm2->llio->name,
                               chan->raw_name,
                               inst->index,
                               c);
            }

            HM2_DBG("BoardName %s\n", chan->name);

            if (hm2_sserial_read_configs(hm2, chan) < 0) {
                HM2_ERR("Failed to read/setup the config data on %s\n",
                        chan->name);
                return -EINVAL;
            }

            if (hm2_sserial_get_globals_list(hm2, chan) < 0) {
                HM2_ERR("Failed to read/setup the globals on %s\n",

                        chan->name);
                return -EINVAL;
            }


            if ( hm2_sserial_create_pins(hm2, chan) < 0) {
                HM2_ERR("Failed to create the pins on %s\n",
                        chan->name);
                return -EINVAL;
            }

            if ( hm2_sserial_register_tram(hm2, chan) < 0) {
                HM2_ERR("Failed to register TRAM for %s\n",
                        chan->name);
                return -EINVAL;
            }
        }
    }
    return 0;
}
int hm2_sserial_read_configs(hostmot2_t *hm2,  hm2_sserial_remote_t *chan){

    int ptoc, addr, buff, c, m;
    unsigned char rectype;

    hm2->llio->read(hm2->llio, chan->rw_addr[2], &buff, sizeof(rtapi_u32));
    ptoc=(buff & 0xffff);
    if (ptoc == 0) {return chan->num_confs;} // Old 8i20 or 7i64

    c = m = 0;
    chan->num_confs = 0;
    do {
        addr = 0;
        ptoc = hm2_sserial_get_bytes(hm2, chan, &addr, ptoc, 2);
        if (((addr &= 0xFFFF) <= 0) || (ptoc < 0)) break;
        if (hm2_sserial_get_bytes(hm2, chan, &rectype, addr, 1) < 0) {
            return -EINVAL;
        }

        if (rectype == LBP_DATA) {
            c = chan->num_confs++;
            chan->confs = (hm2_sserial_data_t *)
                            rtapi_krealloc(chan->confs,
                                    chan->num_confs * sizeof(hm2_sserial_data_t),
                                    RTAPI_GFP_KERNEL);
            addr = hm2_sserial_get_bytes(hm2, chan, &chan->confs[c], addr, 14);
            if (addr < 0){ return -EINVAL;}
            addr = hm2_sserial_get_bytes(hm2, chan,
                                         &(chan->confs[c].UnitString),
                                         addr, -1);
            if (addr < 0){ return -EINVAL;}
            addr = hm2_sserial_get_bytes(hm2, chan,
                                         &(chan->confs[c].NameString),
                                         addr, -1);
            if (addr < 0){ return -EINVAL;}

            if (chan->confs[c].ParmMin == chan->confs[c].ParmMax){
                chan->confs[c].ParmMin = 0;
                chan->confs[c].ParmMax = 1;
            }

            // SmartSerial knows nothing of graycode or wrapless, used only by abs_encoder
            chan->confs[c].Flags = 0;

            HM2_DBG("Process: %s  RecordType: %02X Datatype: %02X Dir: %02X Addr: %04X Length: %i\n",
                           chan->confs[c].NameString, chan->confs[c].RecordType,chan->confs[c].DataType, chan->confs[c].DataDir, chan->confs[c].ParmAddr, chan->confs[c].DataLength);
        } else if (rectype == LBP_MODE ) {
            chan->num_modes++;
            m = chan->num_modes - 1;
            chan->modes = (hm2_sserial_mode_t *)
                            rtapi_krealloc(chan->modes,
                                     chan->num_modes * sizeof(hm2_sserial_mode_t),
                                     RTAPI_GFP_KERNEL);
            addr = hm2_sserial_get_bytes(hm2, chan, &chan->modes[m], addr, 4);
            if (addr < 0){ return -EINVAL;}
            addr = hm2_sserial_get_bytes(hm2, chan,
                                         &(chan->modes[m].NameString),
                                         addr, -1);
            if (addr < 0){ return -EINVAL;}
        }
    } while (addr > 0);

    return chan->num_confs;
}

int hm2_sserial_create_pins(hostmot2_t *hm2, hm2_sserial_remote_t *chan){
    int i, j;
    int r = 0;
    char name[HAL_NAME_LEN + 1];
    int data_dir;
    chan->pins = (hm2_sserial_pins_t*)hal_malloc(chan->num_confs
                                                 * sizeof(hm2_sserial_pins_t));

    chan->num_read_bits = 0 ; chan->num_write_bits = 0;

    for (i = 0 ; i < chan->num_confs ; i++ ){

        if (chan->confs[i].DataDir == LBP_IN){
            data_dir = HAL_OUT;
            chan->num_read_bits += chan->confs[i].DataLength;
        }
        else if (chan->confs[i].DataDir == LBP_IO){
            data_dir = HAL_IO;
            chan->num_read_bits += chan->confs[i].DataLength;
            chan->num_write_bits += chan->confs[i].DataLength;
        }
        else if (chan->confs[i].DataDir == LBP_OUT){
            data_dir = HAL_IN;
            chan->num_write_bits += chan->confs[i].DataLength;
        }
        else {
            HM2_ERR("Invalid Pin direction (%x). Aborting\n",
                    chan->confs[i].DataDir);
            return -EINVAL;
        }

        chan->num_read_regs = ceil(chan->num_read_bits / 32.0);
        chan->num_write_regs = ceil(chan->num_write_bits / 32.0);
        if (chan->num_read_regs >  HM2_SSERIAL_NUMREGS ||
            chan->num_write_regs > HM2_SSERIAL_NUMREGS) {
            HM2_ERR("Data width, %i in / %i out bits, exceeds the supported "
                    "number of smart-serial 32-bit registers (%i)\n",
                     chan->num_read_bits, chan->num_write_bits, HM2_SSERIAL_NUMREGS);
            return -EOVERFLOW;
        }

        if (chan->confs[i].Flags & 0x01){
            chan->pins[i].graycode = 1;
        } else {
            chan->pins[i].graycode = 0;
        }

        if (chan->confs[i].Flags & 0x02){
            chan->pins[i].nowrap = 1;
        } else {
            chan->pins[i].nowrap = 0;
        }


        switch (chan->confs[i].DataType){
            case LBP_PAD:
                break;
            case LBP_BITS:
                chan->pins[i].bit_pins = (hal_bit_t**)
                hal_malloc(chan->confs[i].DataLength * sizeof(hal_bit_t*));
                chan->pins[i].bit_pins_not = (hal_bit_t**)
                hal_malloc(chan->confs[i].DataLength * sizeof(hal_bit_t*));
                chan->pins[i].invert = (hal_bit_t*)
                hal_malloc(chan->confs[i].DataLength * sizeof(hal_bit_t));
                for (j = 0; j < chan->confs[i].DataLength ; j++){

                    rtapi_snprintf(name, sizeof(name), "%s.%s-%02d",
                                   chan->name,
                                   chan->confs[i].NameString,
                                   j);
                    r = hal_pin_bit_new(name,
                                        data_dir,
                                        &(chan->pins[i].bit_pins[j]),
                                        hm2->llio->comp_id);
                    if (r < 0) {
                        HM2_ERR("error adding pin '%s', aborting\n", name);
                        return r;
                    }
                    if (data_dir == HAL_OUT) {
                        rtapi_snprintf(name, sizeof(name), "%s.%s-%02d-not",
                                       chan->name,
                                       chan->confs[i].NameString,
                                       j);
                        r = hal_pin_bit_new(name,
                                            data_dir,
                                            &(chan->pins[i].bit_pins_not[j]),
                                            hm2->llio->comp_id);
                        if (r < 0) {
                            HM2_ERR("error adding pin '%s', aborting\n", name);
                            return r;
                        }
                    }
                    if (data_dir == HAL_IN){
                        rtapi_snprintf(name, sizeof(name), "%s.%s-%02d-invert",
                                       chan->name,
                                       chan->confs[i].NameString,
                                       j);
                        r = hal_param_bit_new(name,
                                              HAL_RW,
                                              &(chan->pins[i].invert[j]),
                                              hm2->llio->comp_id);
                        if (r < 0) {
                            HM2_ERR("error adding pin '%s', aborting\n", name);
                            return r;
                        }
                    }
                }
                break;
            case LBP_UNSIGNED:
            case LBP_SIGNED:
                rtapi_snprintf(name, sizeof(name), "%s.%s",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_pin_float_new(name,
                                      data_dir,
                                      &(chan->pins[i].float_pin),
                                      hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return r;
                }
                rtapi_snprintf(name, sizeof(name), "%s.%s-scalemax",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_param_float_new(name,
                                        HAL_RW,
                                        &(chan->pins[i].fullscale),
                                        hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return r;
                }
                chan->pins[i].fullscale = chan->confs[i].ParmMax;
                if (data_dir == HAL_OUT) {break;}
                rtapi_snprintf(name, sizeof(name), "%s.%s-maxlim",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_param_float_new(name,
                                        HAL_RW,
                                        &(chan->pins[i].maxlim),
                                        hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return r;
                }
                chan->pins[i].maxlim = chan->confs[i].ParmMax;
                rtapi_snprintf(name, sizeof(name), "%s.%s-minlim",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_param_float_new(name,
                                        HAL_RW,
                                        &(chan->pins[i].minlim),
                                        hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return r;
                }
                chan->pins[i].minlim = chan->confs[i].ParmMin;
                break;
            case LBP_NONVOL_UNSIGNED:
            case LBP_NONVOL_SIGNED:
                HM2_ERR("Non-Volatile data type found in PTOC. This should "
                        "never happen. Aborting");
                return r;
            case LBP_STREAM:
                rtapi_snprintf(name, sizeof(name), "%s.%s",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_pin_u32_new(name,
                                    data_dir,
                                    &(chan->pins[i].u32_pin),
                                    hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return r;
                }
                break;
            case LBP_BOOLEAN:
                rtapi_snprintf(name, sizeof(name), "%s.%s",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_pin_bit_new(name,
                                    data_dir,
                                    &(chan->pins[i].boolean),
                                    hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return r;
                }
                if (data_dir == HAL_OUT) {
                    rtapi_snprintf(name, sizeof(name), "%s.%s-not",
                                   chan->name,
                                   chan->confs[i].NameString);
                    r = hal_pin_bit_new(name,
                                        data_dir,
                                        &(chan->pins[i].boolean2),
                                        hm2->llio->comp_id);
                    if (r < 0) {
                        HM2_ERR("error adding pin '%s', aborting\n", name);
                        return r;
                    }
                }
                if (data_dir == HAL_IN) {
                    chan->pins[i].invert = hal_malloc(sizeof(hal_bit_t));
                    rtapi_snprintf(name, sizeof(name), "%s.%s-invert",
                                   chan->name,
                                   chan->confs[i].NameString);
                    r = hal_param_bit_new(name,
                                          HAL_RW,
                                          chan->pins[i].invert,
                                          hm2->llio->comp_id);
                    if (r < 0) {
                        HM2_ERR("error adding pin '%s', aborting\n", name);
                        return r;
                    }
                }
                break;
            case LBP_ENCODER:
            case LBP_ENCODER_H:

                rtapi_snprintf(name, sizeof(name), "%s.%s.count",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_pin_s32_new(name,
                                    HAL_OUT,
                                    &(chan->pins[i].s32_pin),
                                    hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return -EINVAL;
                }
                rtapi_snprintf(name, sizeof(name), "%s.%s.rawcounts",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_pin_s32_new(name,
                                    HAL_OUT,
                                    &(chan->pins[i].s32_pin2),
                                    hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return -EINVAL;
                }
                rtapi_snprintf(name, sizeof(name), "%s.%s.position",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_pin_float_new(name,
                                    HAL_OUT,
                                    &(chan->pins[i].float_pin),
                                    hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return -EINVAL;
                }
                rtapi_snprintf(name, sizeof(name), "%s.%s.index-enable",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_pin_bit_new(name,
                                    HAL_IO,
                                    &(chan->pins[i].boolean),
                                    hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return -EINVAL;
                }

                rtapi_snprintf(name, sizeof(name), "%s.%s.reset",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_pin_bit_new(name,
                                    HAL_IO,
                                    &(chan->pins[i].boolean2),
                                    hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return -EINVAL;
                }
                rtapi_snprintf(name, sizeof(name), "%s.%s.scale",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_param_float_new(name,
                                    HAL_RW,
                                    &(chan->pins[i].fullscale),
                                    hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return -EINVAL;
                }

                rtapi_snprintf(name, sizeof(name), "%s.%s.counts-per-rev",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_param_u32_new(name,
                                    HAL_RW,
                                    &(chan->pins[i].u32_param),
                                    hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return -EINVAL;
                }
                chan->pins[i].fullscale = chan->confs[i].ParmMax;
                chan->pins[i].u32_param = 256;
                break;
            case LBP_ENCODER_L:
                //No pins for encoder L
                break;
            case LBP_FLOAT:
                rtapi_snprintf(name, sizeof(name), "%s.%s",
                               chan->name,
                               chan->confs[i].NameString);
                r = hal_pin_float_new(name,
                                      data_dir,
                                      &(chan->pins[i].float_pin),
                                      hm2->llio->comp_id);
                if (r < 0) {
                    HM2_ERR("error adding pin '%s', aborting\n", name);
                    return r;
                }
                break;
            default:
                HM2_ERR("Unhandled sserial data type (%i) Name %s Units %s\n",
                        chan->confs[i].DataType,
                        chan->confs[i].NameString,
                        chan->confs[i].UnitString);
        }
    }
    return 0;
}

int hm2_sserial_register_tram(hostmot2_t *hm2, hm2_sserial_remote_t *chan){

    int r = 0, i = 0;

    HM2_DBG("%s read-bits = %i, write-bits = %i\n"
            "read-regs = %i, write-regs = %i\n",
            chan->name,
            chan->num_read_bits, chan->num_write_bits,
            chan->num_read_regs, chan->num_write_regs);

    r = hm2_register_tram_read_region(hm2, chan->reg_cs_addr, sizeof(rtapi_u32),
                                      &chan->reg_cs_read);
    if (r < 0) { HM2_ERR("error registering tram read region for sserial CS"
                         "register (%d)\n", r);
        goto fail1;
    }
    for (i = 0; i < chan->num_read_regs; i++){
        r = hm2_register_tram_read_region(hm2, chan->rw_addr[i], sizeof(rtapi_u32),
                                          &chan->read[i]);
        if (r < 0) { HM2_ERR("error registering tram read region for sserial "
                             "interface %i register (%d)\n" , i, r);
            goto fail1;
        }
    }

    // Register the TRAM WRITE

    r = hm2_register_tram_write_region(hm2, chan->reg_cs_addr, sizeof(rtapi_u32),
                                       &(chan->reg_cs_write));
    if (r < 0) { HM2_ERR("error registering tram write region for sserial"
                        "interface cs register (%d)\n", r);
        goto fail1;
    }
    
    for (i = 0; i < chan->num_write_regs; i++) {
        r = hm2_register_tram_write_region(hm2, chan->rw_addr[i], sizeof(rtapi_u32),
                                          &chan->write[i]);
        if (r < 0) { HM2_ERR("error registering tram read region for sserial "
                             "interface %i register (%d)\n" , i, r);
            goto fail1;
        }
    }
    
    return 0;

fail1:
    return -EINVAL;
}

 int hm2_sserial_update_params(hostmot2_t *hm2, hm2_sserial_instance_t *inst, long period){
    // init this here to silence a compiler warning
    hm2_sserial_remote_t *r = &(inst->remotes[0]);
    hm2_sserial_params_t *p;
    hm2_sserial_data_t   *g;
    int shift; // used for floating point comparisons

    switch (*inst->state2){
        case 0: // init loop counters
            inst->r_index = 0;
            inst->g_index = 0;
            *inst->state2 = 1;
            /* Fallthrough */
        case 1:
            if (inst->num_remotes == 0) return 0;
            r = &(inst->remotes[inst->r_index]);
            if (r->num_globals > 0) {
                p = &(r->params[inst->g_index]);
                g = &(r->globals[inst->g_index]);
            } else {
                *inst->state2 = 2;
                break;
            }
            switch (*inst->state3){
                //Commands are queued for TRAM write, so every change in
                //command needs a break to poll the thread
                int ret;
                default:
                    HM2_ERR("Unhandled state %i", *inst->state3);
                    return 1;
                case 0:
                    HM2_DBG("Checking Param %s datatype %02X\n", g->NameString, p->type);
                    switch (p->type){
                        case LBP_SIGNED:
                        case LBP_NONVOL_SIGNED:
                            if (p->s32_param != p->s32_written) break;
                            *inst->state2 = 2; // increment indices
                            return *inst->state2;
                        case LBP_UNSIGNED:
                        case LBP_NONVOL_UNSIGNED:
                            if (p->u32_param != p->u32_written) break;
                            *inst->state2 = 2; // increment indices
                            return *inst->state2;
                        case LBP_FLOAT:
                        case LBP_NONVOL_FLOAT:
                            // comparing floats that might have different sizes is not trivial
                            // this does a bitwise comparison of as many mantissa bits as might
                            // be expected to have been sent by the sserial remote
                            switch (g->DataLength){
                                // ( double significand - variable type significand)
                                default:
                                HM2_ERR("Non IEEE float type parameter of length %i\n", g->DataLength);
                                /* Fallthrough */
                                case 8:
                                    shift = (52 -  4); break; // 1.3.4 minifloat, if we ever add them
                                case 16:
                                    shift = (52 - 10); break;
                                case 32:
                                    shift = (52 - 23); break;
                                case 64:
                                    shift = 0;
                                }
                            if (abs(((int)(p->s64_param) - (int)(p->s64_written)) >> shift) > 2) break;
                            *inst->state2 = 2; // increment indices
                            return *inst->state2;
                        default:
                            *inst->state2 = 2; // increment indices
                            return *inst->state2;
                        }
                    HM2_WARN("Writing value of %s datatype %02X\n", g->NameString, p->type);
                    *inst->state3 = 1;
                    inst->timer = 20000000;
                    *inst->command_reg_write = 0x800; // stop all
                    break;
                 case 1:
                    ret = hm2_sserial_wait(hm2, inst, period);
                    if (ret > 0) break;
                    if (ret < 0) *inst->state3 = 100; // quit and tidy up
                    *inst->state3 = 2;
                    inst->timer = 20000000;
                    *inst->command_reg_write = 0xF00 | (1 << r->index); // channel in setup mode
                    break;
                case 2: // Unlock Nonvol access
                    ret = hm2_sserial_wait(hm2, inst, period);
                    if (ret > 0) break;
                    if (ret < 0) *inst->state3 = 100; // quit and tidy up
                    if (   p->type == LBP_NONVOL_FLOAT
                        || p->type == LBP_NONVOL_UNSIGNED
                        || p->type == LBP_NONVOL_SIGNED){
                        *r->reg_cs_write = LBPNONVOL_flag + LBPWRITE;
                        *r->write[0] = LBPNONVOLEEPROM;
                        *inst->command_reg_write = 0x1000 | (1 << r->index); // doit command
                        inst->timer = 20000000;
                        *inst->state3 = 3;
                        HM2_PRINT("A non-volatile smart-serial parameter has been changed\n"
                                        "A full power-cycle will be needed before the effect is seen\n");
                    } else {
                        *inst->state3 = 4;
                    }
                    break;
                case 3: // wait for doit clear
                    ret = hm2_sserial_wait(hm2, inst, period);
                    if (ret > 0) break;
                    if (ret < 0) *inst->state3 = 100; // quit and tidy up
                    // NV access now enabled.
                    HM2_DBG("NV Access unlocked: param %s\n", g->NameString);
                    *inst->state3 = 4;
                    break;
                case 4: // Now send the data
                    switch (p->type){
                        case LBP_SIGNED:
                        case LBP_NONVOL_SIGNED:
                            *r->write[0] = (rtapi_u32) p->s32_param;
                            break;
                        case LBP_UNSIGNED:
                        case LBP_NONVOL_UNSIGNED:
                            *r->write[0] = p->u32_param;
                            break;
                        case LBP_FLOAT:
                        case LBP_NONVOL_FLOAT:
                            if (g->DataLength == sizeof(float) * 8 ){
                                float temp = p->float_param;
                                memcpy(r->write[0], &temp, sizeof(float));    // Data Value
                            } else if (g->DataLength == sizeof(double) * 8){
                                double temp = p->float_param;
                                memcpy(r->write[0], &temp, sizeof(double));
                            } else {
                                HM2_ERR("sserial write: LBP_FLOAT of bit-length %i not handled\n", g->DataLength);
                                p->type= LBP_PAD; // only warn once, then ignore
                            }
                            break;
                        default:
                            break;
                        }
                    switch (g->DataLength){
                        case 8:
                            *r->reg_cs_write = WRITE_REM_BYTE_CMD | g->ParmAddr;// Data Address
                            break;
                        case 16:
                            *r->reg_cs_write = WRITE_REM_WORD_CMD | g->ParmAddr;// Data Address
                            break;
                        case 32:
                            *r->reg_cs_write = WRITE_REM_LONG_CMD | g->ParmAddr;// Data Address
                            break;
                        case 64:
                            *r->reg_cs_write = WRITE_REM_DOUBLE_CMD | g->ParmAddr;// Data Address
                            break;
                        }
                    *inst->command_reg_write = 0x1000 | (1 << r->index); // doit command
                    inst->timer = 200000000;
                    *inst->state3 = 5;
                    break;
                case 5: // wait for doit clear
                    ret = hm2_sserial_wait(hm2, inst, period);
                    if (ret > 0) break;
                    if (ret < 0) *inst->state3 = 100; // quit and tidy up

                    // success? Set the written = param
                    switch (p->type){
                        case LBP_SIGNED:
                        case LBP_NONVOL_SIGNED:
                            p->s32_written = p->s32_param;
                            break;
                        case LBP_UNSIGNED:
                        case LBP_NONVOL_UNSIGNED:
                            p->u32_written = p->u32_param;
                            break;
                        case LBP_FLOAT:
                        case LBP_NONVOL_FLOAT:
                            p->float_written = p->float_param;
                            break;
                        default:
                            break;
                    }

                    HM2_DBG("New value set for %s\n", g->NameString);

                    if (   p->type == LBP_NONVOL_FLOAT
                        || p->type == LBP_NONVOL_UNSIGNED
                        || p->type == LBP_NONVOL_SIGNED){
                        *r->reg_cs_write = LBPNONVOL_flag + LBPWRITE;
                        *r->write[0] = LBPNONVOLCLEAR;
                        *inst->command_reg_write = 0x1000 | (1 << r->index); // doit command
                        inst->timer = 0x2000000;
                        *inst->state3 = 7;
                    } else {
                        *inst->state3 = 8;
                    }
                    break;
                case 7: // wait for doit clear
                    ret = hm2_sserial_wait(hm2, inst, period);
                    if (ret > 0) break;
                    if (ret < 0) *inst->state3 = 100; // quit and tidy up
                    // NV access now cleared
                    HM2_DBG("NV Access Cleared: param %s\n", g->NameString);
                    * inst->state3 = 8;
                    break;
                case 8: // stop-all
                    inst->timer = 0x2000000;
                    *inst->command_reg_write = 0x800; //stop
                    *inst->state3 = 9;
                    break;
                case 9: //wait for final stop-all
                    ret = hm2_sserial_wait(hm2, inst, period);
                    if (ret > 0) break;
                    if (ret < 0) *inst->state3 = 100; // quit and tidy up
                    // NV access now cleared
                    HM2_DBG("Board out of setup mode: param %s\n", g->NameString);
                    *inst->state3 = 0;
                    *inst->state2 = 2; // increment indices
                    break;
                case 100: // error recovery
                    HM2_ERR("Problem found writing sserial parameter %s\n", g->NameString);
                    *inst->command_reg_write = 0x800; //stop all command
                    *inst->state3 = 0;
                    *inst->state2 = 2; // increment indices
                    break;
            } // End of switch(inst->state3)
        break;
    case 2:
        *inst->state2 = 1;
        if ((int)++inst->g_index >= inst->remotes[inst->r_index].num_globals){
            inst->g_index = 0;
            if ((int)++inst->r_index >= inst->num_remotes){//checked them all
                *inst->state2 = 0;
            }
        }
        break;
    } // end of switch(inst->state2)
    return *inst->state2;
}

void hm2_sserial_write_pins(hostmot2_t *hm2, hm2_sserial_instance_t *inst){
    int b, p, r, i;
    int bitcount;
    rtapi_u64 buff;
    double val;

    // the side effect of reporting this error will suffice
    (void)hm2_sserial_check_remote_errors(hm2, inst);

    if (*inst->fault_count > inst->fault_lim) {
        // If there have been a large percentage of misses, for quite
        // a long time, it's time to take it seriously.
        hm2_sserial_check_local_errors(hm2, inst);
        HM2_ERR("Smart Serial Comms Error: "
                "There have been more than %i errors in %i "
                "thread executions at least %i times. "
                "See other error messages for details.\n",
                inst->fault_dec,
                inst->fault_inc,
                inst->fault_lim);
        HM2_ERR("***Smart Serial Port %i will be stopped***\n",inst->index);
        static bool printed;
        if(!inst->ever_read && !printed) {
            HM2_ERR("Smart Serial Error: "
                "You may see this error if the FPGA card "
                """read"" function is not running. "
                "This error message will not repeat.\n");
            printed = true;
        }
        *inst->state = 10;
        *inst->command_reg_write = 0x800; // stop command
        return;
    }
    if (*inst->command_reg_read) {
        if (inst->doit_err_count < 6){ inst->doit_err_count++; }
        if (inst->doit_err_count == 4 ){ // ignore 4 errors at startup
            HM2_ERR("Smart Serial port %i: DoIt not cleared from previous "
                    "servo thread. Servo thread rate probably too fast. "
                    "This message will not be repeated, but the "
                    "%s.sserial.%1d.fault-count pin will indicate "
                    "if this is happening frequently.\n",
                    inst->index, hm2->llio->name, inst->index);
        }
        *inst->fault_count += inst->fault_inc;
        *inst->command_reg_write = 0x80000000; // set bit31 for ignored cmd
        return; // give the register chance to clear
    }
    if (*inst->data_reg_read & 0xff) { // indicates a failed transfer
        *inst->fault_count += inst->fault_inc;
    }

    if (*inst->fault_count > inst->fault_dec) {
        *inst->fault_count -= inst->fault_dec;
    }
    else
    {
        *inst->fault_count = 0;
    }

    // All seems well, handle the pins.
    for (r = 0 ; r < inst->num_remotes ; r++ ) {
        hm2_sserial_remote_t *chan = &inst->remotes[r];

        // Only update this channel's pins if no transfer error
        if (*inst->data_reg_read & (1 << chan->index)) continue;

        bitcount = 0;
        for (i = 0; i < chan->num_write_regs; i++){
            *chan->write[i] = 0;
        }
        for (p = 0 ; p < chan->num_confs ; p++){
            hm2_sserial_data_t *conf = &chan->confs[p];
            hm2_sserial_pins_t *pin = &chan->pins[p];
            if (conf->DataDir & 0xC0){
                switch (conf->DataType){
                    case LBP_PAD:
                        // do nothing
                        break;
                    case LBP_BITS:
                        buff = 0;
                        for (b = 0 ; b < conf->DataLength ; b++){
                            buff |= ((rtapi_u64)(*pin->bit_pins[b] != 0) << b)
                            ^ ((rtapi_u64)(pin->invert[b] != 0) << b);
                        }
                        break;
                    case LBP_UNSIGNED:
                        val = *pin->float_pin;
                        if (val > pin->maxlim) val = pin->maxlim;
                        if (val < pin->minlim) val = pin->minlim;
                        buff = (rtapi_u64)((val / pin->fullscale)
                                     * (~0ull >> (64 - conf->DataLength)));
                        break;
                    case LBP_SIGNED:
                        //this only works if DataLength <= 32
                        val = *pin->float_pin;
                        if (val > pin->maxlim) val = pin->maxlim;
                        if (val < pin->minlim) val = pin->minlim;
                        buff = (((rtapi_s32)(val / pin->fullscale * 2147483647))
                                >> (32 - conf->DataLength))
                        & (~0ull >> (64 - conf->DataLength));
                        break;
                    case LBP_STREAM:
                        buff = *pin->u32_pin & (~0ull >> (64 - conf->DataLength));
                        break;
                    case LBP_BOOLEAN:
                        buff = 0;
                        if (*pin->boolean ^ ((conf->DataDir == LBP_OUT)?(*pin->invert):0)){
                            buff = (~0ull >> (64 - conf->DataLength));
                        }
                        break;
                    case LBP_ENCODER:
                         // Would we ever write to a counter?
                        // Assume not for the time being
                        break;
                    case LBP_FLOAT:
                        if (conf->DataLength == sizeof(float) * 8 ){
                            float temp = *pin->float_pin;
                            memcpy(&buff, &temp, sizeof(float));
                        } else if (conf->DataLength == sizeof(double) * 8){
                            double temp = *pin->float_pin;
                            memcpy(&buff, &temp, sizeof(double));
                        } else {
                            HM2_ERR_NO_LL("sserial write: LBP_FLOAT of bit-length %i not handled\n", conf->DataLength);
                            conf->DataType = 0; // only warn once, then ignore
                        }
                        break;
                    default:
                        HM2_ERR("Unsupported output datatype 0x%02X (name: ""%s"")\n",
                                conf->DataType, conf->NameString);
                        conf->DataType = 0; // Warn once, then ignore
                }
                bitcount = setbits(chan, &buff, bitcount, conf->DataLength);
            }
        }
    }

    *inst->command_reg_write = 0x1000 | inst->tag;
}

void hm2_sserial_prepare_tram_write(hostmot2_t *hm2, long period){
    // This function contains a state machine to handle starting and stopping
    // The ports as well as setting up the pin data

    int i;

    if (hm2->sserial.num_instances <= 0) return;

    for (i = 0 ; i < hm2->sserial.num_instances ; i++ ) {
        // a state-machine to start and stop the ports,
        // supply Do-It commands and set up params at init.

        hm2_sserial_instance_t *inst = &(hm2->sserial.instance[i]);

        switch ((*inst->state) & 0xFF){

            case 0: // Idle
                if (! *inst->run){ break; }
                // Check for any changed parameters
                if (hm2_sserial_update_params(hm2, inst, period) > 0) break;
                //set the modes for the cards
                hm2_sserial_setmode(hm2, inst);
                *inst->command_reg_write = 0x900 | inst->tag;
                HM2_DBG("Enabled Remotes tag = = %x\n", inst->tag);
                *inst->fault_count = 0;
                inst->doit_err_count = 0;
                inst->timer = 2100000000;
                 *inst->state = 2;
                break;
            case 2: // just transitioning to running
                if (hm2_sserial_wait(hm2, inst, period) > 0) break;
                *inst->state = 3;
                break;
            case 3: // normal running
                if (!*inst->run){
                     *inst->state = 4;
                    break;
                }
                hm2_sserial_write_pins(hm2, inst);
                break;
            case 4: // run to stop transition
                *inst->command_reg_write = 0x800;
                inst->timer = 2100000000;
                *inst->state = 5;
                break;
            case 5:
                if (hm2_sserial_wait(hm2, inst, period) > 0) break;
                *inst->state = 0;
                break;
            case 10:// Do-nothing state for serious errors. require run pin to cycle
                *inst->command_reg_write = 0x80000000; // set bit31 for ignored cmd
                if ( ! *inst->run){*inst->state = 0;}
                break;
            default: // Should never happen
                HM2_ERR("Unhandled run/stop configuration in \n"
                        "hm2_sserial_write (%x)\n",
                        *inst->state);
                *inst->state = 0;
        }
    }
}

int hm2_sserial_read_pins(hm2_sserial_remote_t *chan){
    static int h_flag = 0, l_flag = 0;//these are the "memory" for 2-part
    static int bitshift = 1;               //Fanuc encoders where the full turns
    static rtapi_u64 buff_store;             //and part turns are not contiguous
    int b, p, r;
    int bitcount = 0;
    rtapi_u64 buff;
    rtapi_s32 buff32;
    rtapi_s64 buff64;
    chan->status = *chan->reg_cs_read;
    for (p=0 ; p < chan->num_confs ; p++){
        hm2_sserial_data_t *conf = &chan->confs[p];
        hm2_sserial_pins_t *pin = &chan->pins[p];
        if (! (conf->DataDir & 0x80)){
            r = getbits(chan, &buff, bitcount, conf->DataLength);
            if(r < 0) return r;

            switch (conf->DataType){
            case LBP_PAD:
                // do nothing
                break;
            case LBP_BITS:
                for (b = 0 ; b < conf->DataLength ; b++){
                    *pin->bit_pins[b] = ((buff & (1LL << b)) != 0);
                    *pin->bit_pins_not[b] = ! *pin->bit_pins[b];
                }
                break;
            case LBP_UNSIGNED:

                if (pin->graycode){
                    rtapi_u64 mask;
                    for(mask = buff >> 1 ; mask != 0 ; mask = mask >> 1){
                        buff ^= mask;
                    }
                }

                *pin->float_pin = (buff * pin->fullscale)
                / ((1 << conf->DataLength) - 1);
                break;
            case LBP_SIGNED:
                buff32 = (buff & 0xFFFFFFFFL) << (32 - conf->DataLength);
                *pin->float_pin = (buff32 / 2147483647.0 )
                                    * pin->fullscale;
                break;
            case LBP_STREAM:
                *pin->u32_pin = buff & (~0ull >> (64 - conf->DataLength));
                break;
            case LBP_BOOLEAN:
                *pin->boolean = (buff != 0);
                if(conf->DataDir == LBP_IN){
                    *pin->boolean2 = (buff == 0);
                }
                break;
            case LBP_ENCODER_H:
            case LBP_ENCODER_L:
                if (conf->DataType == LBP_ENCODER_H){
                    h_flag = conf->DataLength;
                    buff_store |= (buff << bitshift);
                } else {
                    l_flag = conf->DataLength;
                    bitshift = conf->DataLength;
                    buff_store |= buff;
                }
                if ( ! (h_flag && l_flag)){
                    break;
                }
                buff = buff_store;
                /* Fallthrough */
            case LBP_ENCODER:
            {
                int bitlength;
                rtapi_s32 rem1, rem2;
                rtapi_s64 previous;
                rtapi_u32 ppr = pin->u32_param;

                if (conf->DataType == LBP_ENCODER){
                    bitlength = conf->DataLength;
                } else {
                    bitlength = h_flag + l_flag;
                    h_flag = 0; l_flag = 0;
                    buff_store = 0;
                }


                if (pin->graycode){
                    rtapi_u64 mask;
                    for(mask = buff >> 1 ; mask != 0 ; mask = mask >> 1){
                        buff ^= mask;
                    }
                }

                // sign-extend buff into buff64
                buff64 = (1U << (bitlength - 1));
                buff64 = (buff ^ buff64) - buff64;
                previous = pin->accum;

                if (pin->nowrap == 0){
                    if ((buff64 - pin->oldval) > (1 << (bitlength - 2))){
                        pin->accum -= (1 << bitlength);
                    } else if ((pin->oldval - buff64) > (1 << (bitlength - 2))){
                        pin->accum += (1 << bitlength);
                    }
                }
                pin->accum += (buff64 - pin->oldval);

                //reset
                if (*pin->boolean2){pin->offset = pin->accum;}

                //index-enable
                if (*pin->boolean && ppr > 0){ // index-enable set
                    rtapi_div_s64_rem(previous, ppr, &rem1);
                    rtapi_div_s64_rem(pin->accum, ppr, &rem2);
                    if ((unsigned)abs(rem1 - rem2) > ppr / 2
                            || (rem1 >= 0 && rem2 < 0)
                            || (rem1 < 0 && rem2 >= 0)){
                        if (pin->accum > previous){
                            if (pin->accum > 0){
                                pin->offset = pin->accum - rem2;
                            } else if (pin->accum < 0){
                                pin->offset = pin->accum - rem2;
                            } else {
                                pin->offset = 0;
                            }
                        } else {
                            if (pin->accum > 0){
                                pin->offset = pin->accum - rem2 + ppr;
                            } else if (pin->accum < 0){
                                pin->offset = pin->accum - rem2;
                            } else {
                                pin->offset = 0;
                            }
                        }
                        *pin->boolean = 0;
                    }
                }
                pin->oldval = buff64;
                *pin->s32_pin = pin->accum - pin->offset;
                *pin->s32_pin2 = pin->accum;
                *pin->float_pin = (double)(pin->accum - pin->offset) / pin->fullscale ;
                break;
            case LBP_FLOAT:
                if (conf->DataLength == sizeof(float) * 8){
                    float temp;
                    memcpy(&temp, &buff, sizeof(float));
                    *pin->float_pin = temp;
                } else if (conf->DataLength == sizeof(double) * 8){
                    double temp;
                    memcpy(&temp, &buff, sizeof(double));
                    *pin->float_pin = temp;
                } else {
                    HM2_ERR_NO_LL("sserial read: LBP_FLOAT of bit-length %i not handled\n", conf->DataLength);
                    conf->DataType = 0; // Only warn once, then ignore
                }
                break;
            }
            default:
                HM2_ERR_NO_LL("Unsupported input datatype 0x%02X (name: ""%s"")\n",
                        conf->DataType, conf->NameString);

                conf->DataType = 0; // Only warn once, then ignore

            }
            bitcount += conf->DataLength;
        }
    }
    return 0;
}

void hm2_sserial_process_tram_read(hostmot2_t *hm2, long period){
    (void)period;
    int i, c;
    for (i = 0 ; i < hm2->sserial.num_instances ; i++){
        hm2_sserial_instance_t *inst = &hm2->sserial.instance[i];
        inst->ever_read = true;
        if (*inst->state != 3) continue ; // Only work on running instances
        for (c = 0 ; c < inst->num_remotes ; c++ ) {
            hm2_sserial_remote_t *chan = &inst->remotes[c];
            hm2_sserial_read_pins(chan);
        }
    }
}

void hm2_sserial_print_module(hostmot2_t *hm2) {
    int i,r,c,g,m;
    if (hm2->sserial.num_instances <= 0) return;
    HM2_PRINT("SSerial: %d\n", hm2->sserial.num_instances);
    HM2_PRINT("  version %d\n", hm2->sserial.version);
    for (i = 0; i < hm2->sserial.num_instances; i ++) {
        HM2_PRINT("    instance %d:\n", i);
        HM2_PRINT("        Command Addr 0x%04x\n", hm2->sserial.instance[i].command_reg_addr);
        HM2_PRINT("        Data Addr    0x%04x\n", hm2->sserial.instance[i].data_reg_addr);
        for (r = 0; r < hm2->sserial.instance[i].num_remotes; r++) {
            HM2_PRINT("        port %i device %s\n", r, hm2->sserial.instance[i].remotes[r].name);
            HM2_PRINT("             Parameters:\n");
            for (c = 0 ; c < hm2->sserial.instance[i].remotes[r].num_confs; c++){
                hm2_sserial_data_t conf = hm2->sserial.instance[i].remotes[r].confs[c];
                HM2_PRINT("                   RecordType = 0x%02x\n", conf.RecordType);
                HM2_PRINT("                   DataLength = 0x%02x\n", conf.DataLength);
                HM2_PRINT("                   DataType = 0x%02x\n", conf.DataType);
                HM2_PRINT("                   DataDir = 0x%02x\n", conf.DataDir);
                HM2_PRINT("                   ParmMax %0i.%02i\n", (int)conf.ParmMax,
                        (int)((conf.ParmMax - (int)conf.ParmMax) * 100.0));
                HM2_PRINT("                   ParmMin %0i.%02i\n",(int)conf.ParmMin,
                        (int)((conf.ParmMin - (int)conf.ParmMin) * 100.0));
                HM2_PRINT("                   SizeOf ParmMin 0x%02zx\n", sizeof(conf.ParmMax));
                HM2_PRINT("                   ParmAddr = 0x%04x\n", conf.ParmAddr);
                HM2_PRINT("                   UnitString = %s\n", conf.UnitString);
                HM2_PRINT("                   NameString = %s\n\n", conf.NameString);
            }
            HM2_PRINT("             Globals:\n");
            for (g = 0 ; g < hm2->sserial.instance[i].remotes[r].num_globals; g++){
                hm2_sserial_data_t conf = hm2->sserial.instance[i].remotes[r].globals[g];
                HM2_PRINT("                   RecordType = 0x%02x\n", conf.RecordType);
                HM2_PRINT("                   DataLength = 0x%02x\n", conf.DataLength);
                HM2_PRINT("                   DataType = 0x%02x\n", conf.DataType);
                HM2_PRINT("                   DataDir = 0x%02x\n", conf.DataDir);
                HM2_PRINT("                   ParmMax %0i.%02i\n", (int)conf.ParmMax,
                        (int)((conf.ParmMax - (int)conf.ParmMax) * 100.0));
                HM2_PRINT("                   ParmMin %0i.%02i\n",(int)conf.ParmMin,
                        (int)((conf.ParmMin - (int)conf.ParmMin) * 100.0));
                HM2_PRINT("                   SizeOf ParmMin %zi\n", sizeof(conf.ParmMax));
                HM2_PRINT("                   ParmAddr = 0x%04x\n", conf.ParmAddr);
                HM2_PRINT("                   UnitString = %s\n", conf.UnitString);
                HM2_PRINT("                   NameString = %s\n\n", conf.NameString);
            }
            HM2_PRINT("             Modes:\n");
            for (m = 0; m < hm2->sserial.instance[i].remotes[r].num_modes; m++){
                hm2_sserial_mode_t mode = hm2->sserial.instance[i].remotes[r].modes[m];
                HM2_PRINT("               RecordType = 0x%02x\n", mode.RecordType);
                HM2_PRINT("               ModeIndex = 0x%02x\n", mode.ModeIndex);
                HM2_PRINT("               ModeType = 0x%02x\n", mode.ModeType);
                HM2_PRINT("               Unused = %i\n", mode.Unused);
                HM2_PRINT("               NameString = %s\n\n", mode.NameString);
            }
        }
    }
    HM2_PRINT("\n");
}

void hm2_sserial_setmode(hostmot2_t *hm2, hm2_sserial_instance_t *inst){
    int c;
    int n = 0;
    int i = inst->index;
    HM2_DBG("Num Auto = %i\n", inst->num_remotes);
    for (c = 0 ; c < inst->num_remotes ; c++){
        n = inst->remotes[c].index;
        if (hm2->config.sserial_modes[i][n] != 'x') {
            // CS - write card mode
            *inst->remotes[c].reg_cs_write = (hm2->config.sserial_modes[i][n] - '0') << 24;
        }
    }
}

// All sserial faults other than timeouts are indicated by fault
// bits in the CS register.
//
// 1. Communication faults
//
// these are indicated when bit 13 (communication error) is set
// after a doit command.  Further decoding of communication faults
// should not be done unless bit 13 is set after a doit.  (but note
// that for sserial firmware version <= 43, these bits are
// unintentionally sticky and are not reset after a doit)
//
// If bit 13 is set, bits 0 through 5 (local communication faults)
// should be decoded and reported if they meet the inc/dec criteria
// (bits 6 and 7 should  always be ignored)
//
// 2. Remote faults
//
// These are indicated when bit 8 (remote fault) is set after a doit
// command.  Further decoding of remote faults should not be done
// unless bit 8 is set.
//
// If bit 8 is set, the specific remote fault error bits (24 through 31)
// should be decoded and reported. These are mostly sticky and tend to
// be fatal so should only be reported once and should not be subject to
// the inc/dec reporting criteria

static const char *err_list[32] = {"CRC error", "Invalid cookie", "Overrun",
    "Timeout", "Extra character", "Serial Break Error", "Remote Fault",
    "Too many errors",

    "Remote fault", NULL, NULL, NULL, NULL,
    "Communication error", "No Remote ID", "Communication Not Ready",

    NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,

    "Watchdog Fault", "No Enable", "Over Temperature", "Over Current",
    "Over Voltage", "Under Voltage", "Illegal Remote Mode", "LBPCOM Fault"};

int hm2_sserial_check_local_errors(hostmot2_t *hm2, hm2_sserial_instance_t *inst){
    rtapi_u32 buff;
    int i,r;
    int err_flag = 0;
    rtapi_u32 err_mask = 0x0000E0FF;

    for (r = 0 ; r < inst->num_remotes ; r++){
        hm2_sserial_remote_t *chan=&inst->remotes[r];
        buff = chan->status;
        buff &= err_mask;
        for (i = 31 ; i >= 0 && buff != 0 ; i--){
            if (buff & (1 << i) && err_list[i]) {
                HM2_ERR("Smart serial card %s local error = (%i) %s\n",
                        chan->name, i, err_list[i]);
                err_flag = -EINVAL;
            }
        }
    }
    return err_flag;
}

int hm2_sserial_check_remote_errors(hostmot2_t *hm2, hm2_sserial_instance_t *inst) {
    rtapi_u32 buff;
    int i,r;
    int err_flag = 0;
    rtapi_u32 err_mask = 0xFF000100;


    for (r = 0 ; r < inst->num_remotes ; r++){
        hm2_sserial_remote_t *chan=&inst->remotes[r];

        if((chan->status & 0x100) == 0) return 0;
        buff = chan->status & ~chan->seen_remote_errors & err_mask;
        chan->seen_remote_errors |= chan->status;
        for (i = 31 ; i >= 23 ; i--){
            if (buff & (1 << i) && err_list[i]) {
                HM2_ERR("Smart serial card %s remote error = (%i) %s\n",
                        chan->name, i, err_list[i]);
                err_flag = -EINVAL;
            }
        }
    }
    return err_flag;
}

void hm2_sserial_force_write(hostmot2_t *hm2){
    (void)hm2;
    // there's nothing to do here, because hm2_sserial_prepare_tram_write takes
    // charge of recovering after communication error.
}

void hm2_sserial_cleanup(hostmot2_t *hm2){
    int i,r;
    rtapi_u32 buff;
    for (i = 1 ; i < hm2->sserial.num_instances; i++){
        //Shut down the sserial devices rather than leave that to the watchdog.
        buff = 0x800;
        hm2->llio->write(hm2->llio,
                         hm2->sserial.instance[i].command_reg_addr,
                         &buff,
                         sizeof(rtapi_u32));
        if (hm2->sserial.instance[i].remotes != NULL){
            if (hm2->sserial.instance[i].remotes){
                for (r = 0 ; r < hm2->sserial.instance[i].num_remotes; r++){
                    if (hm2->sserial.instance[i].remotes[r].num_confs > 0){
                        rtapi_kfree(hm2->sserial.instance[i].remotes[r].confs);
                    };
                    if (hm2->sserial.instance[i].remotes[r].num_modes > 0){
                        rtapi_kfree(hm2->sserial.instance[i].remotes[r].modes);
                    }
                }
                rtapi_kfree(hm2->sserial.instance[i].remotes);
            }
        }

    }
}
