//    Copyright (C) 2013 Andy Pugh

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

/* A component to convert 7i73 bytecodes to bit pins */
#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#define MAX_CHAN 8

/* module information */
MODULE_AUTHOR("Andy Pugh");
MODULE_DESCRIPTION("Hal-to-text component for Mesa 7I73 and similar");
MODULE_LICENSE("GPL");

typedef struct {
    struct {
        hal_bit_t **key;
        hal_bit_t **rows;
        hal_bit_t **cols;
        hal_u32_t *keycode;
    } hal;
    struct {
        hal_u32_t rollover;
        hal_bit_t invert;
    } param;
    hal_u32_t ncols;
    hal_u32_t nrows;
    hal_u32_t *now;
    hal_u32_t *then;
    hal_bit_t invert;
    char name[HAL_NAME_LEN + 1];
    struct input_dev *key_dev;
    hal_u32_t index;
    unsigned keydown;
    unsigned keyup;
    unsigned rowshift;
    unsigned row;
    unsigned num_keys;
    hal_bit_t scan;
    hal_bit_t keystroke;
}kb_inst_t;

typedef struct {
    kb_inst_t *insts;
    int num_insts;
}kb_t;

static int comp_id;
static kb_t *kb;

char *config[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(config, MAX_CHAN, "screen formatting scancodes")
char *names[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(names, MAX_CHAN, "component names")

void keyup(kb_inst_t *inst){
    unsigned r, c;
    unsigned keycode = *inst->hal.keycode & ~(inst->keydown | inst->keyup);

    r = keycode >> inst->rowshift;
    c = keycode & ~(0xFFFFFFFF << inst->rowshift);
    
    if  (   r >= inst->nrows
         || c >= inst->ncols
         || inst->hal.key[r * inst->ncols + c] == NULL){
        return;
    }
    
    if (inst->num_keys > 0) inst->num_keys--;
    
    *inst->hal.key[r * inst->ncols + c] = 0;
}
void keydown(kb_inst_t *inst){
    unsigned r, c;
    unsigned keycode = *inst->hal.keycode & ~(inst->keydown | inst->keyup);
    
    r = keycode >> inst->rowshift;
    c = keycode & ~(0xFFFFFFFF << inst->rowshift);
    
    if  (   r >= inst->nrows
         || c >= inst->ncols
         || inst->hal.key[r * inst->ncols + c] == NULL){
        return;
    }
    
    if (inst->num_keys >= inst->param.rollover) return;
    inst->num_keys++;
    
    *inst->hal.key[r * inst->ncols + c] = 1;
}

void loop(void *arg, long period){
    (void)period;
    unsigned c;
    hal_u32_t scan = 0;
    kb_inst_t *inst = arg;
    
    if (inst->scan){ //scanning request
        for (c = 0; c < inst->ncols; c++){
            scan += ((*inst->hal.cols[c] != inst->param.invert) << c);
        }
        if (scan == inst->now[inst->row] && scan != inst->then[inst->row]){
            // debounced and changed
            for (c = 0; c < inst->ncols; c++){
                int mask = 1 << c;
                if ((inst->then[inst->row] & mask) && !(scan & mask)){ //keyup
                    *inst->hal.keycode = inst->keyup 
                    + (inst->row << inst->rowshift) 
                    + c;
                    keyup(inst);
                }
                else if (!(inst->then[inst->row] & mask) && (scan & mask)){//keydown
                    *inst->hal.keycode = inst->keydown 
                    + (inst->row << inst->rowshift) 
                    + c;
                    
                    keydown(inst);
                }
            }
        }
        else {
            *inst->hal.keycode = 0x40;//nochange
        }
        
        inst->then[inst->row] = inst->now[inst->row];
        inst->now[inst->row] = scan;
        
        *inst->hal.rows[inst->row] = inst->param.invert;
        inst->row++;
        if (inst->row >= inst->nrows) inst->row = 0;
        *inst->hal.rows[inst->row] = !inst->param.invert;
    }
    else
    {
        if (*inst->hal.keycode == 0x40) return;
        if ((*inst->hal.keycode & inst->keydown) == inst->keydown){
            keydown(inst);
        }
        else if ((*inst->hal.keycode & inst->keydown) == inst->keyup)
        {
            keyup(inst);
        }
    }
}


int rtapi_app_main(void){
    int i, j, n;
    int retval;
    comp_id = hal_init("matrix_kb");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "matrix_kb: ERROR: hal_init() failed\n");
        return -1;
    }
    
    // allocate shared memory for data
    kb = hal_malloc(sizeof(kb_t));
    if (kb == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "matrix_kb component: Out of Memory\n");
        hal_exit(comp_id);
        return -1;
    }
    
    // Count the instances.
    for (kb->num_insts = 0; config[kb->num_insts];kb->num_insts++);
    // Count the names.
    for (n = 0; names[n];n++);
    
    if (n && n != kb->num_insts){
        rtapi_print_msg(RTAPI_MSG_ERR, "matrix_kb: Number of sizes and number"
                        " of names must match\n");
        hal_exit(comp_id);
        return -1;
    }
    
    kb->insts = hal_malloc(kb->num_insts * sizeof(kb_inst_t));
    
    for (i = 0; i < kb->num_insts; i++){
        int a = 0;
        unsigned c, r;
        kb_inst_t *inst = &kb->insts[i];

        inst->index = i;
        inst->nrows = 0;
        inst->ncols = 0;
        inst->scan = 0;
        inst->keystroke = 0;
        inst->param.invert = 1;
        
        for(j = 0; config[i][j] !=0; j++){
            int n = (config[i][j] | 0x20); //lower case
            if (n == 'x'){
                inst->nrows = a;
                a = 0;
            }
            else if (n >= '0' && n <= '9'){
                a = (a * 10) + (n - '0');
            }
            else if (n == 's'){
                inst->scan = 1;
            }
        }
        inst->ncols = a;
        
        if (inst->ncols == 0 || inst->nrows == 0){
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "matrix_kb: Invalid size format. should be NxN\n");
            hal_exit(comp_id);
            return -1;
        }
        
        if (inst->ncols > 32){
            rtapi_print_msg(RTAPI_MSG_ERR,
                            "matrix_kb: maximum number of columns is 32. Sorry\n");
            hal_exit(comp_id);
            return -1;
        }
        
        for (inst->rowshift = 1; inst->ncols > (1u << inst->rowshift); inst->rowshift++);
        for (inst->keydown = 0xC0, inst->keyup = 0x80
             ; (inst->nrows << inst->rowshift) > inst->keydown
             ; inst->keydown <<= 1, inst->keyup <<= 1);
        
        inst->hal.key = (hal_bit_t **)hal_malloc(inst->nrows * inst->ncols * sizeof(hal_bit_t*));
        inst->now = hal_malloc(inst->nrows * sizeof(hal_u32_t));
        inst->then = hal_malloc(inst->nrows * sizeof(hal_u32_t));
        inst->row = 0;
        inst->param.rollover = 2;
        
        
        if (names[i]){
            rtapi_snprintf(inst->name, sizeof(inst->name), "%s", names[i]);
        }
        else
        {
            rtapi_snprintf(inst->name, sizeof(inst->name), "matrix_kb.%i", i);
        }
        
        for (c = 0; c < inst->ncols; c++){
            for (r = 0; r < inst->nrows; r++){  
                retval = hal_pin_bit_newf(HAL_OUT,
                                          &(inst->hal.key[r * inst->ncols + c]), 
                                          comp_id,
                                          "%s.key.r%xc%x", 
                                          inst->name, r, c);
                if (retval != 0) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                                    "matrix_kb: Failed to create output pin\n");
                    hal_exit(comp_id);
                    return -1;
                }
            }
        }
        
        if (inst->scan){ //internally generated scanning
            inst->hal.rows = (hal_bit_t **)hal_malloc(inst->nrows * sizeof(hal_bit_t*));
            inst->hal.cols = (hal_bit_t **)hal_malloc(inst->ncols * sizeof(hal_bit_t*));
            
            for (r = 0; r < inst->nrows; r++){
                retval = hal_pin_bit_newf(HAL_OUT,
                                          &(inst->hal.rows[r]), comp_id,
                                          "%s.row-%02i-out",inst->name, r);
                if (retval != 0) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                                    "matrix_kb: Failed to create output row pin\n");
                    hal_exit(comp_id);
                    return -1;
                }
            }
            for (c = 0; c < inst->ncols; c++){
                retval = hal_pin_bit_newf(HAL_IN,
                                          &(inst->hal.cols[c]), comp_id,
                                          "%s.col-%02i-in",inst->name, c);
                if (retval != 0) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                                    "matrix_kb: Failed to create input col pin\n");
                    hal_exit(comp_id);
                    return -1;
                }
            }
                
            retval = hal_pin_u32_newf(HAL_OUT,
                                      &(inst->hal.keycode), comp_id,
                                      "%s.keycode",inst->name);
            if (retval != 0) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                                "matrix_kb: Failed to create output pin\n");
                hal_exit(comp_id);
                return -1;
            }
            
            retval = hal_param_bit_newf(HAL_RW,
                                      &(inst->param.invert), comp_id,
                                      "%s.negative-logic",inst->name);
            if (retval != 0) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                                "matrix_kb: Failed to create output pin\n");
                hal_exit(comp_id);
                return -1;
            }
            
            
            retval = hal_param_u32_newf(HAL_RW,
                                      &(inst->param.rollover), comp_id,
                                      "%s.key_rollover",inst->name);
            if (retval != 0) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                                "matrix_kb: Failed to create rollover param\n");
                hal_exit(comp_id);
                return -1;
            }
            
        }
        else // scanning by 7i73 or similar
        {
            retval = hal_pin_u32_newf(HAL_IN,
                                      &(inst->hal.keycode), comp_id,
                                      "%s.keycode",inst->name);
            if (retval != 0) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                                "matrix_kb: Failed to create input pin\n");
                hal_exit(comp_id);
                return -1;
            }
        }
        
        retval = hal_export_funct(inst->name, loop, inst, 1, 0, comp_id); //needs fp?
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "matrix_kb: ERROR: function export failed\n");
            return -1;
        }
    }
    hal_ready(comp_id);
    
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}
