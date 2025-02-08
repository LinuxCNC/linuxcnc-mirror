//
//    Copyright (C) 2012 Andy Pugh
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

#ifndef __SSERIAL_H
#define __SSERIAL_H

#include <rtapi_stdint.h>
#include "hostmot2.h"

#define HM2_SSERIAL_TYPE_8I20               0x30324938  // '8i20' as 4 ascii
#define HM2_SSERIAL_TYPE_7I64               0x34364937  // '7i64' All newer cards self-declare
#define HM2_SSERIAL_MAX_STRING_LENGTH       48
#define HM2_SSERIAL_MAX_DATALENGTH          (255+1)     // unsigned char max value plus one for buffer sizing
#define HM2_SSERIAL_MAX_PORTS               8           // only used in pins.c
#define HM2_SSERIAL_NUMREGS                 7           // 224 bits of data per channel in sserialb

//Commands etc
#define LBPNONVOL_flag      0xCC000000
#define LBPWRITE            0x20000000
#define LBPNONVOLCLEAR      0x0
#define LBPNONVOLEEPROM     0x01             
#define LBPNONVOLFLASH      0x02

#define READ_LOCAL_CMD      0x2000
#define WRITE_LOCAL_CMD     0xA000

#define READ_REM_BYTE_CMD     0x44000000
#define READ_REM_WORD_CMD     0x45000000
#define READ_REM_LONG_CMD     0x46000000
#define READ_REM_DOUBLE_CMD   0x47000000
#define READ_COOKIE_CMD       0xDF000000
#define WRITE_REM_BYTE_CMD    0x64000000
#define WRITE_REM_WORD_CMD    0x65000000
#define WRITE_REM_LONG_CMD    0x66000000
#define WRITE_REM_DOUBLE_CMD  0x67000000
#define FLASHERASE_CMD        0xFE
#define FLASHWRITE_CMD        0xFD

#define SSLBPMAJORREVISIONLOC   2
#define SSLBPMINORREVISIONLOC   3
#define SSLBPCHANNELSTARTLOC    4
#define SSLBPCHANNELSTRIDELOC   5
#define LBPFLASHOFFSETLOC       0x8000
#define LBPFLASHERASESIZELOC    0x8004
#define LBPFLASHWRITESIZELOC    0x8005
#define LBPFLASHCOMMITLOC       0x8007

#define LBPCOOKIE               0x5A

#define HM2WRITE(a,b)  hm2->llio->write(hm2->llio, a, &b, sizeof(rtapi_u32))
#define HM2READ(a,b)  hm2->llio->read(hm2->llio, a, &b, sizeof(rtapi_u32))

#define LBP_IN                  0x00
#define LBP_IO                  0x40
#define LBP_OUT                 0x80

#define LBP_DATA                0xA0
#define LBP_MODE                0xB0

#define LBP_PAD                 0x00
#define LBP_BITS                0x01
#define LBP_UNSIGNED            0x02
#define LBP_SIGNED              0x03
#define LBP_NONVOL_UNSIGNED     0x04
#define LBP_NONVOL_SIGNED       0x05
#define LBP_STREAM              0x06
#define LBP_BOOLEAN             0x07
#define LBP_ENCODER             0x08
#define LBP_FLOAT               0x10 // New for STMBL
#define LBP_NONVOL_FLOAT        0x11 
#define LBP_ENCODER_H           0x18 // For Fanuc Absolute Encoders with separate
#define LBP_ENCODER_L           0x28 // part and full count fields. 

// NOTE: This structure must match the layout of some device memory.  In
// particular, you can't change the "float"s to "double"s.
typedef struct {
    unsigned char RecordType;
    unsigned char DataLength;
    unsigned char DataType;
    unsigned char DataDir;
    float ParmMin;
    float ParmMax;
    short ParmAddr;
    char UnitString[HM2_SSERIAL_MAX_STRING_LENGTH+1];
    char NameString[HM2_SSERIAL_MAX_STRING_LENGTH+1];
    char Flags; //0x01 = graycode 0x02 = nowrap
}hm2_sserial_data_t;

typedef struct {
    unsigned char RecordType;
    unsigned char ModeIndex;
    unsigned char ModeType;
    unsigned char Unused;
    char NameString[HM2_SSERIAL_MAX_STRING_LENGTH+1];
}hm2_sserial_mode_t;

typedef struct {
    hal_u32_t *u32_pin;
    hal_s32_t *s32_pin;
    hal_s32_t *s32_pin2;
    hal_float_t *float_pin;
    hal_bit_t **bit_pins;
    hal_bit_t **bit_pins_not;
    hal_bit_t *invert;
    hal_bit_t *boolean;
    hal_bit_t *boolean2;
    hal_float_t maxlim;
    hal_float_t minlim;
    hal_float_t fullscale;
    hal_u32_t u32_param;
    hal_bit_t graycode;
    hal_bit_t nowrap;
    rtapi_s64 oldval; // not pins, but this way every pin can have one
    rtapi_s64 accum; // these two are only currently used by encoders
    rtapi_s64 offset;
}hm2_sserial_pins_t;

typedef struct {
    int type;
    union {
        long long s64_param;
        hal_u32_t u32_param;
        hal_s32_t s32_param;
        hal_float_t float_param;
        hal_bit_t bit_param;
    };
    union {
        long long s64_written;
        hal_u32_t u32_written;
        hal_s32_t s32_written;
        hal_float_t float_written;
        hal_bit_t bit_written;
    };
    hal_u32_t timer_num;
    hal_bit_t *error;
}hm2_sserial_params_t;

typedef struct {
    int num_confs;
    int num_modes;
    int num_globals;
    int num_pins;
    int num_read_regs; // data bits / 32. Was previously always 3. 
    int num_write_regs;
    int num_read_bits;
    int num_write_bits;
    hm2_sserial_mode_t *modes;
    hm2_sserial_data_t *confs;
    hm2_sserial_data_t *globals;
    hm2_sserial_pins_t *pins;
    hm2_sserial_params_t *params;
    hal_u32_t serialnumber;
    hal_u32_t status, seen_remote_errors;

    rtapi_u32 *reg_cs_read;
    rtapi_u32 *reg_cs_write;
    rtapi_u32 *read[HM2_SSERIAL_NUMREGS];
    rtapi_u32 *write[HM2_SSERIAL_NUMREGS];
    rtapi_u32 reg_cs_addr;
    rtapi_u32 rw_addr[HM2_SSERIAL_NUMREGS];
    int index;
    rtapi_u32 command_reg_addr; // a duplicate so that a single channel can be passed
    rtapi_u32 data_reg_addr;
    rtapi_u32 data_written[3]; // only used by abs_encoder.c
    int myinst;
    char name[29];
    char raw_name[5];
}hm2_sserial_remote_t;

typedef struct {
    int device_id;
    int use_serial_numbers;
    int num_remotes;
    int num_channels;
    int tag;
    hm2_sserial_remote_t *remotes;
    int index;
    rtapi_u32 command_reg_addr;
    rtapi_u32 *command_reg_read;
    rtapi_u32 *command_reg_write;
    rtapi_u32 data_reg_addr;
    rtapi_u32 *data_reg_read;
    rtapi_u32 *data_reg_write;
    hal_u32_t *fault_count;
    hal_u32_t fault_inc;
    hal_u32_t fault_dec;
    hal_u32_t fault_lim;
    
    hal_bit_t *run;
    hal_u32_t *state;
    hal_u32_t *state2;
    hal_u32_t *state3;
    hal_s32_t *debug;
    hal_u32_t r_index;
    hal_u32_t g_index;
    int doit_err_count;
    rtapi_s32 timer;
    bool ever_read;
} hm2_sserial_instance_t;

typedef struct {
    rtapi_u8 version;
    int baudrate;
    int num_instances; // number of active instances
    hm2_sserial_instance_t *instance ;
} hm2_sserial_t;

// The 8i20 and 7i64 pre-date parameter discovery

static const hm2_sserial_data_t hm2_8i20_params[] = {
    {LBP_DATA,0x10,LBP_UNSIGNED,LBP_OUT,0.0,1.0,0,"degrees","angle",0},
    {LBP_DATA,0x10,LBP_SIGNED,LBP_OUT,-10.0,10.0,0,"amps","current",0},
    {LBP_DATA,0x10,LBP_UNSIGNED,LBP_IN,0,65535,0,"Deg C","card-temp",0},
    {LBP_DATA,0x10,LBP_UNSIGNED,LBP_IN,0,655.35,0,"volts","bus-voltage",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","status.current-lim",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","status.brake-on",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","status.brake-old",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","status.bus-underv",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","status.wd-reset",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","status.sw-reset",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","status.ext-reset",0},
    {LBP_DATA,0x01,LBP_PAD,LBP_IN,0,0,0,"pad","pad",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","status.no-enable",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","status.pid-on",0},
    {LBP_DATA,0x06,LBP_PAD,LBP_IN,0,0,0,"pad","pad",0},
    
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.watchdog",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.no-enable",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.overtemp",0},
    {LBP_DATA,0x01,LBP_PAD,LBP_IN,0,0,0,"pad","pad",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.overcurrent",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.U-current",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.V-current",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.W-current",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.bus-underv",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.bus-high",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.bus-overv",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.module",0},
    {LBP_DATA,0x01,LBP_PAD,LBP_IN,0,0,0,"pad","pad",0},
    {LBP_DATA,0x01,LBP_PAD,LBP_IN,0,0,0,"pad","pad",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.overrun",0},
    {LBP_DATA,0x01,LBP_BOOLEAN,LBP_IN,0,0,0,"none","fault.framingr",0}
};
static const hm2_sserial_data_t hm2_8i20_globals[] = { 
    {LBP_DATA,0x10,LBP_UNSIGNED,LBP_IN,0,0,2164,"none","swrevision",0},
    {LBP_DATA,0x20,LBP_UNSIGNED,LBP_IN,0,0,2344,"none","unitnumber",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,32.768,2,"amps","nvmaxcurrent",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,36,"volts","nvbusundervmax",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,34,"volts","nvbusundervmin",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,38,"volts","nvbusoverv",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,40,"volts","nvbrakeonv",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,42,"volts","nvbrakeoffv",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,6,"bps","nvrembaudrate",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,10,"none","nvkqp",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,14,"none","nvkqihi",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,12,"none","nvkqilo",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,16,"none","nvkqil",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,18,"none","nvkdp",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,22,"none","nvkdihi",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,20,"none","nvkdilo",0},
    {LBP_DATA,0x10,LBP_NONVOL_UNSIGNED,LBP_OUT,0,0,24,"none","nvkdil",0}
};

static const hm2_sserial_data_t hm2_7i64_params[] = {
    {LBP_DATA,0x18,LBP_BITS,LBP_OUT,0,0,0,"none","output",0},
    {LBP_DATA,0x18,LBP_BITS,LBP_IN,0,0,0,"none","input",0},
    {LBP_DATA,0x08,LBP_PAD,LBP_IN,0,0,0,"pad","pad",0},
    {LBP_DATA,0x10,LBP_UNSIGNED,LBP_IN,0,3.3,0,"volts","analog0",0},
    {LBP_DATA,0x10,LBP_UNSIGNED,LBP_IN,0,3.3,0,"volts","analog1",0}
};

#endif
