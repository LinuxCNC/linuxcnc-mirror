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
//
//    The code in this file is based on UFLBP.PAS by Peter C. Wallace.  

#include <rtapi_firmware.h>
#include <rtapi_string.h>
#include <rtapi_gfp.h>
#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include "hostmot2.h"
#include "sserial.h"

static int comp_id;

/* module information */
MODULE_AUTHOR("Andy Pugh");
MODULE_DESCRIPTION("A simple util for nvram setting on smart-serial cards");
MODULE_LICENSE("GPL");

static char *cmd;
RTAPI_MP_STRING(cmd, "smart-serial setting commands");

char **cmd_list;

hostmot2_t *hm2;
hm2_sserial_remote_t *remote;

int waitfor(void){
    rtapi_u32 buff;
    long long int starttime = rtapi_get_time();
    do {
        rtapi_delay(50000);
        HM2READ(remote->command_reg_addr, buff);
        if (rtapi_get_time() - starttime > 1000000000){
            rtapi_print_msg(RTAPI_MSG_ERR, "Timeout waiting for CMD to clear\n");
            return -1;
        }
    } while (buff);
    
    return 0;
}

int doit(void){
    rtapi_u32 buff = 0x1000 | (1 << remote->index);
    HM2WRITE(remote->command_reg_addr, buff);
    if (waitfor() < 0) return -1;
    HM2READ(remote->data_reg_addr, buff);
    if (buff & (1 << remote->index)){
        rtapi_print_msg(RTAPI_MSG_ERR, "Error flag set after CMD Clear %08x\n",
                        buff);
        return -1;
    }
    return 0;
}

int stop_all(void){
    rtapi_u32 buff=0x8FF;
    HM2WRITE(remote->command_reg_addr, buff);
    return waitfor();
}

int setup_start(void){
    rtapi_u32 buff=0xF00 | 1 << remote->index;
    HM2WRITE(remote->command_reg_addr, buff);
    if (waitfor() < 0) return -1;
    HM2READ(remote->data_reg_addr, buff); 
    rtapi_print("setup start: data_reg readback = %x\n", buff);
    if (buff & (1 << remote->index)){
        rtapi_print("Remote failed to start\n");
        return -1;
    }
    return 0;
}

int nv_access(rtapi_u32 type){
    rtapi_u32 buff = LBPNONVOL_flag + LBPWRITE;
    rtapi_print("buff = %x\n", buff);
    HM2WRITE(remote->reg_cs_addr, buff);
    HM2WRITE(remote->rw_addr[0], type);
    return doit();
}

int set_nvram_param(rtapi_u32 addr, rtapi_u32 value){
    rtapi_u32 buff;
    
    if (stop_all() < 0) goto fail0;
    if (setup_start() < 0) goto fail0;
    if (nv_access(LBPNONVOLEEPROM) < 0) goto fail0;
    
    // value to set
    HM2WRITE(remote->rw_addr[0], value);
    buff = WRITE_REM_WORD_CMD | addr; 
    HM2WRITE(remote->reg_cs_addr, buff);
    if (doit() < 0) goto fail0;
    
    if (nv_access(LBPNONVOLCLEAR) < 0) goto fail0;
    
    return 0;
fail0: // It's all gone wrong
    buff=0x800; //Stop
    HM2WRITE(remote->command_reg_addr, buff);
    rtapi_print_msg(RTAPI_MSG_ERR,
                    "Problem with Smart Serial parameter setting, see dmesg\n");
    return -1;
}

static void setsserial_release(struct rtapi_device *dev) {
    (void)dev;
    // nothing to do here
}

int getlocal(int addr, int bytes){
    rtapi_u32 val = 0;
    rtapi_u32 buff;
    for (;bytes--;){
        buff = READ_LOCAL_CMD | (addr + bytes);
        HM2WRITE(remote->command_reg_addr, buff);
        waitfor();
        HM2READ(remote->data_reg_addr, buff);
        val = (val << 8) | buff;
    }    return val;
}

int setlocal(int addr, int val, int bytes){
    rtapi_u32 b = 0;
    rtapi_u32 buff;
    int i;
    for (i = 0; i < bytes; i++){
        b = val & 0xFF;
        val >>= 8;
        HM2WRITE(remote->data_reg_addr, b);
        buff = WRITE_LOCAL_CMD | (addr + i);
        HM2WRITE(remote->command_reg_addr, buff);
        if (waitfor() < 0) return -1;
    }   
    return 0;
}

void sslbp_write_lbp(rtapi_u32 cmd, rtapi_u32 data){
    rtapi_u32 buff = LBPWRITE + cmd;
    HM2WRITE(remote->reg_cs_addr, buff);
    HM2WRITE(remote->rw_addr[0], data);
    doit();
    buff = 0;
    HM2WRITE(remote->reg_cs_addr, buff);
}

int sslbp_read_cookie(void){
    rtapi_u32 buff = READ_COOKIE_CMD;
    rtapi_u32 res;
    HM2WRITE(remote->reg_cs_addr, buff);
    if (doit() < 0){
        HM2_ERR("Error in sslbp_read_cookie, trying to abort\n");
        return -1;
    }
    HM2READ(remote->rw_addr[0], res);
    buff = 0;
    HM2WRITE(remote->reg_cs_addr, buff);
    return res;
}

rtapi_u8 sslbp_read_byte(rtapi_u32 addr){
    rtapi_u32 buff = READ_REM_BYTE_CMD + addr;
    rtapi_u32 res;
    HM2WRITE(remote->reg_cs_addr, buff);
    if (doit() < 0){
        HM2_ERR("Error in sslbp_read_byte, trying to abort\n");
        return -1;
    }
    HM2READ(remote->rw_addr[0], res);
    buff = 0;
    HM2WRITE(remote->reg_cs_addr, buff);
    return (rtapi_u8)res;
}

rtapi_u16 sslbp_read_word(rtapi_u32 addr){
    rtapi_u32 buff = READ_REM_WORD_CMD + addr;
    rtapi_u32 res;
    HM2WRITE(remote->reg_cs_addr, buff);
    if (doit() < 0){
        HM2_ERR("Error in sslbp_read_word, trying to abort\n");
        return -1;
    }
    HM2READ(remote->rw_addr[0], res);
    buff = 0;
    HM2WRITE(remote->reg_cs_addr, buff);
    return (rtapi_u16)res;
}

rtapi_u32 sslbp_read_long(rtapi_u32 addr){
    rtapi_u32 buff = READ_REM_LONG_CMD + addr;
    rtapi_u32 res=0;
    HM2WRITE(remote->reg_cs_addr, buff);
    if (doit() < 0){
        HM2_ERR("Error in sslbp_read_long, trying to abort\n");
        return -1;
    }
    HM2READ(remote->rw_addr[0], buff);
    buff = 0;
    HM2WRITE(remote->reg_cs_addr, buff);
    return res;
}

rtapi_u64 sslbp_read_double(rtapi_u32 addr){
    rtapi_u64 res;
    rtapi_u32 buff = READ_REM_DOUBLE_CMD + addr;
    HM2WRITE(remote->reg_cs_addr, buff);
    if (doit() < 0){
        HM2_ERR("Error in sslbp_read_double, trying to abort\n");
        return -1;
    }
    HM2READ(remote->rw_addr[1], buff);
    res = buff;
    res <<= 32;
    HM2READ(remote->rw_addr[0], buff);
    res += buff;
    buff = 0;
    HM2WRITE(remote->reg_cs_addr, buff);
    return res;
}

int sslbp_write_byte(rtapi_u32 addr, rtapi_u32 data){
    rtapi_u32 buff = WRITE_REM_BYTE_CMD + addr;
    HM2WRITE(remote->reg_cs_addr, buff);
    HM2WRITE(remote->rw_addr[0], data);
    if (doit() < 0){
        HM2_ERR("Error in sslbp_write_byte, trying to abort\n");
        return -1;
    }
    buff = 0;
    HM2WRITE(remote->reg_cs_addr, buff);
    return 0;
}

int sslbp_write_word(rtapi_u32 addr, rtapi_u32 data){
    rtapi_u32 buff = WRITE_REM_WORD_CMD + addr;
    HM2WRITE(remote->reg_cs_addr, buff);
    HM2WRITE(remote->rw_addr[0], data);
    if (doit() < 0){
        HM2_ERR("Error in sslbp_write_word, trying to abort\n");
        return -1;
    }
    buff = 0;
    HM2WRITE(remote->reg_cs_addr, buff);
    return 0;
}

int sslbp_write_long(rtapi_u32 addr, rtapi_u32 data){
    rtapi_u32 buff = WRITE_REM_LONG_CMD + addr;
    HM2WRITE(remote->reg_cs_addr, buff);
    HM2WRITE(remote->rw_addr[0], data);
    if (doit() < 0){
        HM2_ERR("Error in sslbp_write_long, trying to abort\n");
        return -1;
    }
    buff = 0;
    HM2WRITE(remote->reg_cs_addr, buff);
    return 0;
}

int sslbp_write_double(rtapi_u32 addr, rtapi_u32 data0, rtapi_u32 data1){
    rtapi_u32 buff = WRITE_REM_DOUBLE_CMD + addr;
    HM2WRITE(remote->reg_cs_addr, buff);
    HM2WRITE(remote->rw_addr[0], data0);
    HM2WRITE(remote->rw_addr[1], data1);
    if (doit() < 0){
        HM2_ERR("Error in sslbp_write_double, trying to abort\n");
        return -1;
    }
    buff = 0;
    HM2WRITE(remote->reg_cs_addr, buff);
    return 0;
}

    
void flash_start(void){
    sslbp_write_lbp(LBPNONVOL_flag, LBPNONVOLFLASH);
}

void flash_stop(void){
    sslbp_write_lbp(LBPNONVOL_flag, 0);
}
    
int sslbp_flash(char *fname){
    const struct rtapi_firmware *fw;
    struct rtapi_device dev;
    int r;
    unsigned write_sz, erase_sz;
    
    if (strstr("8i20", remote->name)){
        if (hm2->sserial.version < 37){
            rtapi_print("SSLBP Version must be at least v37 to flash the 8i20"
                        "This firmware has v%i. Sorry about that\n"
                        ,hm2->sserial.version);
            return -1;
        }
    }
    else if (hm2->sserial.version < 34){
        rtapi_print("SSLBP Version must be at least v34. This firmware has v%i"
                    "\n",hm2->sserial.version);
        return -1;
    }
    
    if (hm2->sserial.baudrate != 115200){
        rtapi_print("To flash firmware the baud rate of the board must be set "
                    "to 115200 by jumper, and in Hostmot2 using the "
                    "sserial_baudrate modparam\n");
        return -1;
    }
     
    //Copied direct from hostmot2.c. A bit of a faff, but seems to be necessary. 
    memset(&dev, '\0', sizeof(dev));
    rtapi_dev_set_name(&dev, "%s", hm2->llio->name);
    dev.release = setsserial_release;
    r = rtapi_device_register(&dev);
    if (r != 0) {
        HM2_ERR("error with device_register\n");
        return -1;
    }
    r = rtapi_request_firmware(&fw, fname, &dev);
    rtapi_device_unregister(&dev);
    if (r == -ENOENT) {
        HM2_ERR("firmware %s not found\n",fname);
        return -1;
    }
    if (r != 0) {
        HM2_ERR("request for firmware %s failed, aborting\n", fname);
        return -1;
    }    
    rtapi_print("Firmware size 0x%zx\n", fw->size);
    
    if (setup_start() < 0) goto fail0;
    flash_start();
    write_sz = 1u << sslbp_read_byte(LBPFLASHWRITESIZELOC);
    erase_sz = 1u << sslbp_read_byte(LBPFLASHERASESIZELOC);
    HM2_PRINT("Write Size = %x, Erase Size = %x\n", write_sz, erase_sz);
    flash_stop();
    
    //Programming Loop
    {
        unsigned ReservedBlock = 0;
        unsigned StartBlock = ReservedBlock + 1;
        
        unsigned blocknum = StartBlock;
        unsigned i, j, t;
        while (blocknum * erase_sz < fw->size){
            unsigned block_start = blocknum * erase_sz;
            for (t = 0; t < erase_sz && fw->data[block_start + t] == 0 ; t++){ }
            if (t <  erase_sz){ // found a non-zero byte
                flash_start();
                sslbp_write_long(LBPFLASHOFFSETLOC, block_start);
                sslbp_write_byte(LBPFLASHCOMMITLOC, FLASHERASE_CMD);
                if (sslbp_read_cookie() != LBPCOOKIE){
                    HM2_ERR("Synch failed during block erase: aborting\n");
                    goto fail0;
                }
                flash_stop();
                HM2_PRINT("Erased block %i\n", blocknum);
                flash_start();
                for (i = 0; i < erase_sz ; i += write_sz){
                    sslbp_write_long(LBPFLASHOFFSETLOC, block_start + i);
                    for (j = 0 ; j < write_sz ; j += 8){
                        rtapi_u32 data0, data1, m;
                        m = block_start + i + j;
                        data0 = (fw->data[m] 
                              + (fw->data[m + 1] << 8)
                              + (fw->data[m + 2] << 16)
                              + (fw->data[m + 3] << 24));
                        data1 = (fw->data[m + 4] 
                              + (fw->data[m + 5] << 8)
                              + (fw->data[m + 6] << 16)
                              + (fw->data[m + 7] << 24));
                        sslbp_write_double(j, data0, data1);
                    }
                    sslbp_write_byte(LBPFLASHCOMMITLOC, FLASHWRITE_CMD);
                    if (sslbp_read_cookie() != LBPCOOKIE){
                        HM2_ERR("Synch failed during block write: aborting\n");
                        goto fail0;
                    }
                }
                flash_stop();
                HM2_PRINT("Wrote block %i\n", blocknum);
            }
            else // Looks like an all-zeros block
            { 
                HM2_PRINT("Skipped Block %i\n", blocknum);
            }
            blocknum++;
        }
    }
    
    rtapi_release_firmware(fw);
    
    return 0;
    
fail0:
    flash_stop();
    return -1;
}

int rtapi_app_main(void)
{
    int cnt;
    
    comp_id = hal_init("setsserial");
    hal_ready(comp_id);
    
    cmd_list = rtapi_argv_split(RTAPI_GFP_KERNEL, cmd, &cnt);
    
    remote = hm2_get_sserial(&hm2, cmd_list[1]);
    if (! remote) {   
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "Unable to find sserial remote corresponding to %s\n", 
                        cmd_list[1]);
        return -1;
    }    
    
    if (! strncmp("set", cmd_list[0], 3) && cnt == 3){
        rtapi_u32 value;
        rtapi_u32 addr;
        int i;
        rtapi_print("set command %s\n", cmd_list[1]);
        addr = 0;
        for (i = 0; i < remote->num_globals; i++){
            if (strstr(cmd_list[1], remote->globals[i].NameString)){
                addr = remote->globals[i].ParmAddr;
                break;
            }
        }
        if (! addr) {   
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "Unable to find parameter corresponding to %s\n", 
                            cmd_list[1]);
            return -1;
        }
        value = simple_strtol(cmd_list[2], NULL, 0);
        rtapi_print("remote name = %s ParamAddr = %x Value = %i\n",
                    remote->name,
                    addr,
                    value);
        if (set_nvram_param(addr, value) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "Parameter setting failed\n");
            return -1;
        } 
        else
        {   rtapi_print_msg(RTAPI_MSG_ERR, "Parameter setting success\n");
            return 0;
        }
    } 
    else if (! strncmp("flash", cmd_list[0], 5) && cnt == 3){
        rtapi_print("flash command\n");
        if ( ! strstr(cmd_list[2], ".BIN")){
            rtapi_print("Smart-Serial remote firmwares are .BIN format\n "
                        "flashing with the wrong one would be bad. Aborting\n");
            return -EINVAL;
        }
        if (sslbp_flash(cmd_list[2]) < 0){
            rtapi_print_msg(RTAPI_MSG_ERR, "Firmware Flash Failed\n");
            return -1;
        } 
        else
        {   rtapi_print_msg(RTAPI_MSG_ERR, "Firmware Flash Success\n");
            return 0;
        } 
    }
    else {
        rtapi_print_msg(RTAPI_MSG_ERR, 
                        "Unknown command or wrong number of parameters to "
                        "setsserial command");
        return -1;
    }
    
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}




