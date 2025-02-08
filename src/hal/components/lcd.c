
//
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

/* A component to stream HAL data to an LCD in a user-defined format */

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "hal.h"
#include "rtapi_math.h"

#if !defined(__KERNEL__)
#include <stdio.h>
#include <stdlib.h>
#endif

/* module information */
MODULE_AUTHOR("Andy Pugh");
MODULE_DESCRIPTION("Hal-to-text component for Mesa 7I73 and similar");
MODULE_LICENSE("GPL");

#define MAX_CHAN 16
#define MAX_ENTRY 20

typedef struct {
    void **args;
    int num_args;
    char *fmt;
    int length;
}lcd_page_t;

typedef struct {
    lcd_page_t *pages;
    unsigned num_pages;
    hal_u32_t *page_num;
    hal_u32_t last_page;
    hal_u32_t *out;
    hal_float_t *contrast;
    hal_float_t last_contrast;
    char buff[MAX_ENTRY + 1];
    int c_ptr;
    int f_ptr;
    int a_ptr;
    unsigned int dp;
}lcd_inst_t;

typedef struct {
    lcd_inst_t *insts;
    int num_insts;
}lcd_t;

static int comp_id;
static lcd_t *lcd;
static void write(void *arg, long period);
static void write_one(lcd_inst_t *inst);

static int parse_fmt(char *in, int *ptr, char *out, void *val, char dp);

char *digits = "0123456789ABCDEF";

char *fmt_strings[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(fmt_strings, MAX_CHAN, "screen formatting scancodes")

#ifndef do_div
# define do_div(n,base) ({					 \
	    rtapi_u32 __base = (base);				 \
	    rtapi_u32 __rem;					 \
	    __rem = ((rtapi_u64)(n)) % __base;			 \
	    (n) = ((rtapi_u64)(n)) / __base;			 \
	    __rem;						 \
	})
#endif

int rtapi_app_main(void){
    int i, f, f1, k, p;
    int retval;
    
    if (!fmt_strings[0]){
        rtapi_print_msg(RTAPI_MSG_ERR, "The LCD component requires at least one valid format string");
        return -EINVAL;
    }

    comp_id = hal_init("lcd");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "LCD: ERROR: hal_init() failed\n");
        return -1;
    }
    
    // allocate shared memory for data
    lcd = hal_malloc(sizeof(lcd_t));
    if (lcd == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "lcd component: Out of Memory\n");
        hal_exit(comp_id);
        return -1;
    }
    
    // Count the instances. Very unlikely to be more than one, but...
    for (lcd->num_insts = 0; fmt_strings[lcd->num_insts];lcd->num_insts++){}
    lcd->insts = hal_malloc(lcd->num_insts * sizeof(lcd_inst_t));
    
    for (i = 0; i < lcd->num_insts; i++){
        lcd_inst_t *inst = &lcd->insts[i];
        inst->num_pages = 1;
        
        // count the pages demarked by | chars.
        for (f = 0; fmt_strings[i][f]; f++){
            if (fmt_strings[i][f] =='|') inst->num_pages++;
        }
        inst->pages = hal_malloc(inst->num_pages * sizeof(lcd_page_t));
        
        //second pass
        f1 = k = p = 0;
        for (f = 0; fmt_strings[i][f]; f++){
            if (fmt_strings[i][f] =='%') {
                int type = parse_fmt(fmt_strings[i], &f, NULL, NULL, 0);
                if (type > 0) {
                    inst->pages[p].num_args++;
                }
            }
            
            if (fmt_strings[i][f + 1] =='|' || fmt_strings[i][f + 1] == 0) {
                inst->pages[p].fmt = hal_malloc(f - f1 + 2);
                retval = snprintf(inst->pages[p].fmt,
                                  f - f1 + 2, "%s",
                                  fmt_strings[i] + f1);
                inst->pages[p].length = f - f1 + 2;
                inst->pages[p].args = hal_malloc(inst->pages[p].num_args
                                                 * sizeof(hal_float_t));
                f1 = f + 2;
                {
                    int a = -1, s = -1;
                    lcd_page_t page = inst->pages[p];
                    
                    while (page.fmt[++s]){
                        
                        if (page.fmt[s] == '%'){
                            int type = parse_fmt(page.fmt, &s, NULL, NULL, 0);
                            a++;
                            switch (type){
                                case 'f':
                                    retval = hal_pin_float_newf(HAL_IN,
                                                                (hal_float_t**)&(inst->pages[p].args[a]), comp_id,
                                                                "lcd.%02i.page.%02i.arg.%02i",
                                                                i, p, a);
                                    if (retval != 0) {
                                        return retval;
                                    }
                                    break;
                                case 'u':
                                case 'c':
                                    retval = hal_pin_u32_newf(HAL_IN,
                                                              (hal_u32_t **)&(inst->pages[p].args[a]), comp_id,
                                                              "lcd.%02i.page.%02i.arg.%02i",
                                                              i, p, a);
                                    if (retval != 0) {
                                        return retval;
                                    }
                                    
                                    break;
                                case 's':
                                    retval = hal_pin_s32_newf(HAL_IN,
                                                              (hal_s32_t **)&(inst->pages[p].args[a]), comp_id,
                                                              "lcd.%02i.page.%02i.arg.%02i",
                                                              i, p, a);
                                    if (retval != 0) {
                                        return retval;
                                    }
                                    break;
                                case 'b':
                                    retval = hal_pin_bit_newf(HAL_IN,
                                                              (hal_bit_t **)&(inst->pages[p].args[a]), comp_id,
                                                              "lcd.%02i.page.%02i.arg.%02i",
                                                              i, p, a);
                                    if (retval != 0) {
                                        return retval;
                                    }
                                    break;
                            }
                        }
                    }
                }
                p++; // increment the page index
            }
        }
    }
    retval = hal_export_funct("lcd", write, lcd, 1, 0, comp_id); //needs fp?
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "LCD: ERROR: function export failed\n");
        return -1;
    }
    
    for (i = 0; i < lcd->num_insts; i++){
        retval = hal_pin_u32_newf(HAL_IN, &(lcd->insts[i].page_num), comp_id,
                                  "lcd.%02i.page_num", i);
        if (retval != 0) {
            return retval;
        }
        lcd->insts[i].last_page = -1; // force screen refresh
        retval = hal_pin_u32_newf(HAL_OUT, &(lcd->insts[i].out), comp_id,
                                  "lcd.%02i.out",i);
        if (retval != 0) {
            return retval;
        }
        retval = hal_pin_float_newf(HAL_IN, &(lcd->insts[i].contrast), comp_id,
                                    "lcd.%02i.contrast",i);
        if (retval != 0) {
            return retval;
        }
        retval = hal_param_u32_newf(HAL_RW, &(lcd->insts[i].dp), comp_id,
                                    "lcd.%02i.decimal-separator",i);
        lcd->insts[i].dp = '.';
        if (retval != 0) {
            return retval;
        }
        lcd->insts[i].f_ptr = 0;
        lcd->insts[i].buff[0]  = 0x0D; // CR  (0x1A - clear screen does
        lcd->insts[i].buff[1]  = 0x0A; // LF  not seem to work reliably)
        lcd->insts[i].buff[2]  = 0x18; // CLEAR
        lcd->insts[i].buff[3]  = 0x0D; // CR
        lcd->insts[i].buff[4]  = 0x0A; // LF
        lcd->insts[i].buff[5]  = 0x18; // CLEAR
        lcd->insts[i].buff[6]  = 0x0D; // CR
        lcd->insts[i].buff[7]  = 0x0A; // LF
        lcd->insts[i].buff[8] = 0x18; // CLEAR
        lcd->insts[i].buff[9] = 0x0D; // CR
        lcd->insts[i].buff[10] = 0x0A; // LF
        lcd->insts[i].buff[11] = 0x18; // CLEAR
        lcd->insts[i].buff[12] = 0x11; // cursor off
                                       // More init can be added
                                       // Max = MAX_ENTRY
        lcd->insts[i].buff[13] = 0;
        lcd->insts[i].c_ptr = 0;
    }

    hal_ready(comp_id);

    return 0;
}

void write(void *arg, long period){
    (void)period;
    lcd_t *lcd;
    int i;
    
    lcd = arg;

    for (i = 0; i < lcd->num_insts; i++){
        lcd_inst_t *inst = &lcd->insts[i];
        write_one(inst);
    }
}

static void write_one(lcd_inst_t *inst){
    static int counter = 100; //100 cycle delay before start
    int retval;
    char c1, c2;
    
    if (counter > 0){
        --counter;
        return;}

    if (inst->buff[inst->c_ptr] != 0){
        *inst->out = inst->buff[inst->c_ptr++];
        return;
    }
    
    inst->c_ptr = 0;
    inst->buff[0] = 0;

    if (*inst->page_num >= inst->num_pages) return; // should this error?
    
    if (*inst->page_num != inst->last_page){
        inst->last_page = *inst->page_num;
        *inst->out = 0x11; //cursor off
        inst->buff[0] = 0x1E; //cursor home
        inst->buff[1] = 0x1A; //clear screen
        inst->buff[2] = 0; // end
        inst->c_ptr = 0;
        inst->f_ptr = 0;
        inst->a_ptr = 0;
        return;
    }

    if (inst->f_ptr >= inst->pages[*inst->page_num].length){
        *inst->out = 0x1B; // ESC
        inst->buff[0] = 0x3D; // =
        inst->buff[1] = 0x20; // Line 0
        inst->buff[2] = 0x20; // Column 0
        inst->buff[3] = 0; // end
        inst->c_ptr = 0;
        inst->f_ptr = 0;
        inst->a_ptr = 0;
        return;
    }
    
    if (*inst->contrast != inst->last_contrast){
        int c = *inst->contrast * 159.0 + 0x20;
        if (c > 0xBF) c = 0xBF;
        if (c < 0x20) c = 0x20;
        inst->last_contrast = *inst->contrast;
        *inst->out = 0x1B;
        inst->buff[0] = 'C';
        inst->buff[1] = c;
        inst->buff[2] = 0;
        inst->c_ptr = 0;
        return;
    }
    
    switch (inst->pages[*inst->page_num].fmt[inst->f_ptr]){
        case '\\': //escape chars
            c1 = inst->pages[*inst->page_num].fmt[++inst->f_ptr];
            switch (c1){
                case 'n':
                case 'N':
                    inst->f_ptr++;
                    inst->buff[0] = 0x18; //erase to end
                    inst->buff[1] = 0x0A; //CR
                    inst->buff[2] = 0x0D; //LF
                    inst->buff[3] = 0;
                    break;
                case 't':
                case 'T':
                    inst->f_ptr++;
                    inst->buff[0] = ' ';
                    inst->buff[1] = ' ';
                    inst->buff[2] = ' ';
                    inst->buff[3] = ' ';
                    inst->buff[4] = 0;
                    break;
                case '\\':
                    inst->f_ptr++;
                    inst->buff[0] = '\\';
                    inst->buff[1] = 0;
                    /* Fallthrough */
                    
                default: //check for hex
                    c2 = inst->pages[*inst->page_num].fmt[++inst->f_ptr];
                    inst->f_ptr++;
                    if (c1 > '9') c1 &= 0xDF; //upper case
                    if (c2 > '9') c2 &= 0xDF;
                    if (strchr(digits, c1) && strchr(digits, c2)){
                        inst->buff[0] = (16 * (strchr(digits, c1) - digits) 
                                               + (strchr(digits, c2) - digits));
                        inst->buff[1] = 0;
                    } else {
                        inst->buff[0] = 0;
                    }
            }
            *inst->out = inst->buff[0];
            inst->c_ptr = 1;
            return;
        case '%':
            retval = parse_fmt(inst->pages[*inst->page_num].fmt,
                               &inst->f_ptr,
                               inst->buff,
                               inst->pages[*inst->page_num].args[inst->a_ptr++],
                               (char)inst->dp);
            if (retval >= 0) {
                *inst->out = inst->buff[0];
                inst->c_ptr = 1;
                inst->f_ptr++;
                return;
            }
            /* Fallthrough */
        default:
            *inst->out = inst->pages[*inst->page_num].fmt[inst->f_ptr++];
    }
}

int num_digits_baseN(rtapi_u64 val, int base){
    int n = 1;
    rtapi_u64 m = 1;
    while ((m *= base) <= val){
        n += 1;
    }
    return n;
}

static int parse_fmt(char *in, int *ptr, char *out, void *val, char dp){
    /*parse val into the format in in pointed to by ptr.
     if out is null, then simply return the type of the format
     as u, s or f. a return value of -1 indicates an invalid pointer
     on exit ptr will point after the end of the format*/
    
    const double pow10[] = {1,10,1e2,1e3,1e4,1e5,1e6,1e7,1e8,1e9,1e10,1e11,
        1e12,1e13,1e14,1e15,1e16,1e17,1e18,1e19,1e20};
    
    char fill = ' ';

    int d = 0; // dot found
    char sgn = ' ';
    int c = -1, m = -1; // Length and Mantissa sig. figs
    int base = 10;
    if (in[*ptr] != '%') return -EINVAL;
    (*ptr)++;
    while (in[*ptr]){
        switch (in[*ptr]){
            case '%':
                if (out != NULL) out[0] = '%';
                return 't';
            case '.':
                if (d) return -EINVAL;
                d = 1;
                break;
            case ' ':
                fill = ' ';
                break;
            case '+':
                sgn = '+';
                break;
            case '0':
                if (c < 0 && d == 0){
                    fill = '0';
                    break;
                }
                /* Fallthrough */
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
                if (d){
                    if (m > 0){
                        m = m * 10 + (in[*ptr] - '0');
                    }
                    else
                    {
                        m = (in[*ptr] - '0');
                    }
                }
                else
                {
                    if (c > 0){
                        c = c * 10 + (in[*ptr] - '0');
                    }
                    else
                    {
                        c = (in[*ptr] - '0');
                    }
                }
                
                if (c > MAX_ENTRY) c = MAX_ENTRY;
                
                break;
                        
            case 'x':
            case 'X':
                base = 16;
                /* Fallthrough */
            case 'o':
                if (base == 10) base = 8;
                /* Fallthrough */
            case 'u':
                if (out == NULL || val == NULL) return 'u';
            {
                unsigned int tmp;
                int i;
                unsigned int v = *((hal_u32_t*)val);

                if (c < 1) c = num_digits_baseN(v, base);

                tmp = v;
                for (i = c - 1; i >= 0; i--){
                    if (tmp != 0 || i == (c-1)){
                        out[i] = digits[tmp % base];
                    }
                    else
                    {
                        out[i] = fill;
                    }
                    tmp /= base;
                }
                if (tmp !=0){ // it didn't fit
                    for (i = 0; i < c; i++){out[i] = '*';}
                }
                out[c] = 0; //terminate string
            }
                return 'u';
                break;
            case 'd':
            case 'i':
                if (out == NULL || val == NULL) return 's';
            {
                int tmp, s = 0;
                int i;
                int v = *((hal_s32_t*)val);
                
                if (sgn == '+') s = 1;
                if (v < 0) {s = 1; sgn = '-'; v = -v;}
                
                if (c < 1) c = num_digits_baseN(v, base) + s;
 
                tmp = abs(v);
                for (i = c - 1; i >= s; i--){
                    if (tmp != 0 || i == c - 1){
                        out[i] = digits[tmp % 10];
                        tmp /= 10;
                    }
                    else
                    {
                        if (s && fill == ' '){
                            out[i] = sgn;
                            s = 0;
                        }
                        else
                        {
                            out[i] = fill;
                        }
                    }
                }
                if (tmp !=0){ // it didn't fit
                    for (i = 0; i < c; i++){out[i] = '*';}
                }
                if (s) out[0] = sgn;
                
                out[c] = 0;
                
                return 's';
            }
            case 'f':
            case 'F': //The reason for all this messing about, no %f format in kernel code!!!!
                if (out == NULL || val == NULL) return 'f';
            {
                int i;
                double v = *((hal_float_t*)val);
                rtapi_u64 tmp = 0; //enough bits for 9 decimal digits.
                int s = 0;
                
                if (sgn != ' ') s = 1;
                if (v < 0) {s = 1; sgn = '-'; v = -v;}
                
                if (m < 0) m = 4;
                
                if (c < 1) c = num_digits_baseN(v, base) + m + 1 + s;
                
                if (c > MAX_ENTRY){ // then it won't fit
                    tmp = 2;
                    goto overflow;
                }
                
                tmp = v * pow10[m] + 0.5;
                
                for (i = c - 1; i > c - m - 1 ; i--){
                    out[i] = digits[do_div(tmp, 10)];
                }
                
                if (m > 0) {
                    out[i] = dp;
                }
                else
                {
                    m = -1; // shuffle down into decimal point
                }
                
                for (i = c - m - 2; i >= s ; i--){
                    if (tmp > 0 || i == c - m - 2){
                        out[i] = digits[do_div(tmp, 10)];
                    }
                    else
                    {
                        if (sgn != ' ' && fill == ' '){
                            s = 0;
                            out[i] = sgn;
                            sgn = ' ';
                        }
                        else
                        {
                            out[i] = fill;
                        }
                    }
                }
                
                if (s){
                    out[0] = sgn;
                }
                
            overflow:
                if (tmp > 0){ // it didn't fit
                    for (i = 1; i < c; i++){out[i] = '*';}
                }
                
                out[c] = 0; //terminate the string
                
                return 'f';
            }
            case 'c':
                if (out == NULL || val == NULL) return 'c';
            {
                int i;
                unsigned char v = *((hal_u32_t*)val);
                
                if (c == 0) c = 1;
                for (i = 0; i < c; i++){
                    out[i] = (v > ' ')? v : ' ';
                }
                out[i] = 0;
                return 'c';
            }
            case 'b':
                if (out == NULL || val == NULL) return 'b';
            {
                char c1, c2;
                char bt = 'E', bf = 'E'; // a hint that there is a format error
                int v = *((hal_bit_t*)val);
                if (in[++(*ptr)] == '\\'){
                    c1 = in[++(*ptr)];
                    if (c1 > '9') c1 &= 0xDF;
                    c2 = in[++(*ptr)];
                    if (c2 > '9') c2 &= 0xDF;
                    if (strchr(digits, c1) && strchr(digits, c2)){
                        bt = (16 * (strchr(digits, c1) - digits) 
                              + (strchr(digits, c2) - digits));
                    }
                } 
                else
                {
                    bt = in[*ptr];
                }
                if (in[++(*ptr)] == '\\'){
                    c1 = in[++(*ptr)] & 0xDF;
                    c2 = in[++(*ptr)] & 0xDF;
                    if (strchr(digits, c1) && strchr(digits, c2)){
                        bf = (16 * (strchr(digits, c1) - digits) 
                              + (strchr(digits, c2) - digits));
                    }
                } 
                else
                {
                    bf = in[*ptr];
                }
                out[0] = (v)?bt:bf;
                out[1] = 0;
                return 'b';
            }
        }
        (*ptr)++;
    }
    return -EINVAL;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}
