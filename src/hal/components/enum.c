//    Copyright (C) 2023 Andy Pugh

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

// Convert bit pins to enumerated ints and vice-versa

#include "rtapi.h"
#include "rtapi_slab.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "hal.h"

#if !defined(__KERNEL__)
#include <stdio.h>
#include <stdlib.h>
#endif

/* module information */
MODULE_AUTHOR("Andy Pugh");
MODULE_DESCRIPTION("convert enumerated types to HAL_BIT pins");
MODULE_LICENSE("GPL");

#define MAX_CHAN 256

typedef struct {
    hal_bit_t *bit;
    hal_u32_t *en; // note use index 0 differently
} enum_hal_t;

typedef struct{
    int dir;
    int num_pins;
    enum_hal_t *hal;
} enum_inst_t;

typedef struct {
    int num_insts;
    enum_inst_t *insts;
} enum_t;

static int comp_id;

static enum_t e;

static char *enums[MAX_CHAN] = {0,};
RTAPI_MP_ARRAY_STRING(enums, MAX_CHAN, "states, ; delimited");
static char *names[MAX_CHAN] = {0,};
RTAPI_MP_ARRAY_STRING(names, MAX_CHAN, "component names (optional)");

static void decode(void *inst, long period);
static void encode(void *inst, long period);

int rtapi_app_main(void){
    int i, j, v;
    int retval;
    char *token;

    if (!enums[0]) {
        rtapi_print_msg(RTAPI_MSG_ERR, "The enum_decode component requires at least"
                " one enumeration list\n");
        return -EINVAL;
    }

    // count instances
    e.num_insts = MAX_CHAN;
    for (i = 0; i < MAX_CHAN; i++){
        if (! enums[i] && ! names[i]){
            e.num_insts = i;
            rtapi_print_msg(RTAPI_MSG_ERR, "making %i instances\n", e.num_insts);
            break;
        }
        if ((! enums[i] && names[i]) || ( ! names[i] && names[0] && enums[i])){
            rtapi_print_msg(RTAPI_MSG_ERR, "Inconsistent number of names and enums\n");
           return -EINVAL;
        }
    }

    comp_id = hal_init("enum");

    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: hal_init() failed\n");
        return -EINVAL;
    }
    // allocate memory for the base struct
    e.insts = (enum_inst_t *)rtapi_kmalloc(e.num_insts * sizeof(enum_inst_t), RTAPI_GFP_KERNEL);
    for (i = 0; i < e.num_insts; i++){
        enum_inst_t *inst = &(e.insts[i]);
        char this[HAL_NAME_LEN];

        // Count the pins
        inst->num_pins = 0;
        inst->dir = HAL_OUT; // direction of bit pin, out for decode
        for (j = strlen(enums[i]); j > 0; j--){
            if (enums[i][j] == ';'){
                if (enums[i][j-1] != ';' ) inst->num_pins++;
                // insert a string terminator
                enums[i][j] = 0;
            }
        }
        inst->hal = (enum_hal_t *)hal_malloc((inst->num_pins + 1) * sizeof(enum_hal_t));
        token = enums[i];
        switch (*token){
            case 'E':
            case 'e': // encode
                inst->dir = HAL_IN;
                break;
            case 'D':
            case 'd':
                inst->dir = HAL_OUT;
                break;
            default:
                rtapi_print_msg(RTAPI_MSG_ERR, "Each enum string must start"
                        "with either E; or D; to define the mode\n");
                goto fail0;
        }

        if (names[i]) {
            rtapi_snprintf(this, HAL_NAME_LEN, "%s", names[i]);
        } else if (inst->dir == HAL_IN) {
            rtapi_snprintf(this, HAL_NAME_LEN, "enum-encode.%02i", i);
        } else {
            rtapi_snprintf(this, HAL_NAME_LEN, "enum-decode.%02i", i);
        }

        // create single per-instance int pin in index 0
        if (inst->dir == HAL_OUT) {
            retval = hal_pin_u32_newf(HAL_IN, &(inst->hal[0].en), comp_id,
                                    "%s.input", this);
        } else {
            retval = hal_pin_u32_newf(HAL_OUT, &(inst->hal[0].en), comp_id,
                                    "%s.output", this);
        }
        v = 0;
        for (j = 1; j <= inst->num_pins; j++){ // 1-based indexing
            // skip to the next pin name
            while (*(++token) != 0){}
            //increment for skipped enumerations
            while (*(++token) == 0) v++;

            retval = hal_pin_bit_newf(inst->dir, &(inst->hal[j].bit),
                    comp_id, "%s.%s-%s",this, token,
                    (inst->dir == HAL_IN)?"in":"out");
            retval += hal_pin_u32_newf(HAL_IN, &(inst->hal[j].en),
                    comp_id, "%s.%s-val",this, token);
            *(inst->hal[j].en) = v++;

            if (retval < 0){
                rtapi_print_msg(RTAPI_MSG_ERR, "Failed to create HAL pins\n");
                goto fail0;
            }
        }
        if (inst->dir == HAL_OUT){
            hal_export_funct(this, decode, inst, 0, 0, comp_id);
        } else {
            hal_export_funct(this, encode, inst, 0, 0, comp_id);
        }
        if (retval < 0){
            rtapi_print_msg(RTAPI_MSG_ERR, "Failed to export functions\n");
            goto fail0;
        }
    }


    hal_ready(comp_id);
    return 0;

    fail0:
    rtapi_kfree(e.insts);
    hal_exit(comp_id);
    return -1;

}

static void decode(void *v_inst, long period){
    (void)period;
    int i;
    enum_inst_t *inst = v_inst;
    for (i = 1; i <= inst->num_pins; i++){
        if (*(inst->hal[0].en) == *(inst->hal[i].en)){
           *(inst->hal[i].bit) = 1;
        } else {
           *(inst->hal[i].bit) = 0;
        }
    }
}
static void encode(void *v_inst, long period){
    (void)period;
    int i;
    enum_inst_t *inst = v_inst;
    *(inst->hal[0].en) = 0;
    for (i = 1; i <= inst->num_pins; i++){
        if (*(inst->hal[i].bit)){
            *(inst->hal[0].en) = *(inst->hal[i].en);
        }
    }
}
