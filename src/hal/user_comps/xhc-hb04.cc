/*
   XHC-HB04 Wireless MPG pendant LinuxCNC HAL module for LinuxCNC

   Copyright (C) 2013 Frederic Rible (frible@teaser.fr)
   Copyright (C) 2013 Rene Hopf (renehopf@mac.com)
   Copyright (C) 2014 Marius Alksnys (marius.alksnys@gmail.com)

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the program; if not, write to the Free
   Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
   MA 02110-1301 USA.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <signal.h>
#include <string.h>
#include <libusb.h>
#include <unistd.h>
#include <stdarg.h>

#include <hal.h>
#include <inifile.hh>

#include "config.h"

const char *modname = "xhc-hb04";
int hal_comp_id;
const char *section = "XHC-HB04";
bool simu_mode = true;

typedef struct {
	char pin_name[256];
	unsigned int code;
} xhc_button_t;

typedef enum {
	axis_off = 0x00,
	axis_x = 0x11,
	axis_y = 0x12,
	axis_z = 0x13,
	axis_a = 0x18,
	axis_spindle = 0x14,
	axis_feed = 0x15
} xhc_axis_t;

#define NB_MAX_BUTTONS 32

#define STEPSIZE_BYTE 35
#define FLAGS_BYTE    36

// the defines below were found for an 18 button device
// bit 'or' patterns for STEPSIZE_BYTE
#define DISPLAY_HOME_ICON           0x10
#define DISPLAY_HEIGHT_SETTING_ICON 0x20
#define DISPLAY_HEIGHT_UNKNOWN_40   0x40
#define DISPLAY_HEIGHT_UNKNOWN_80   0x80

#define STEPSIZE_DISPLAY_0          0x00
#define STEPSIZE_DISPLAY_1          0x01
#define STEPSIZE_DISPLAY_5          0x02
#define STEPSIZE_DISPLAY_10         0x03
#define STEPSIZE_DISPLAY_20         0x04
#define STEPSIZE_DISPLAY_30         0x05
#define STEPSIZE_DISPLAY_40         0x06
#define STEPSIZE_DISPLAY_50         0x07
#define STEPSIZE_DISPLAY_100        0x08
#define STEPSIZE_DISPLAY_500        0x09
#define STEPSIZE_DISPLAY_1000       0x0A
#define STEPSIZE_DISPLAY_P6         0x0B
#define STEPSIZE_DISPLAY_UNKNOWN_0C 0x0C
#define STEPSIZE_DISPLAY_UNKNOWN_0D 0x0D
#define STEPSIZE_DISPLAY_UNKNOWN_0E 0x0E
#define STEPSIZE_DISPLAY_UNKNOWN_0F 0x0F

// guard for maximum number of items in a stepsize_sequence
#define MAX_STEPSIZE_SEQUENCE 10

// alternate stepsize sequences (use STEPSIZE_DISPLAY_*), terminate with 0:
static const int  stepsize_sequence_1[MAX_STEPSIZE_SEQUENCE +1] = {1,10,100,1000, 0}; // default
static const int  stepsize_sequence_2[MAX_STEPSIZE_SEQUENCE +1] = {1, 5, 10, 20,         0};
static const int  stepsize_sequence_3[MAX_STEPSIZE_SEQUENCE +1] = {1,10,100,             0};
static const int  stepsize_sequence_4[MAX_STEPSIZE_SEQUENCE +1] = {1, 5, 10, 20,  50,100,0};
static const int  stepsize_sequence_5[MAX_STEPSIZE_SEQUENCE +1] = {1,10, 50,100,1000,    0};
static const int* stepsize_sequence = stepsize_sequence_1; // use the default
static int stepsize_idx = 0; // start at initial (zeroth) sequence
static int stepsize_last_idx  =  0; //calculated`


typedef struct {
	hal_float_t *x_wc, *y_wc, *z_wc, *a_wc;
	hal_float_t *x_mc, *y_mc, *z_mc, *a_mc;

	hal_float_t *feedrate_override, *feedrate;
	hal_float_t *spindle_override, *spindle_rps;

	hal_bit_t *button_pin[NB_MAX_BUTTONS];

    hal_bit_t *jog_enable_off;
	hal_bit_t *jog_enable_x;
	hal_bit_t *jog_enable_y;
	hal_bit_t *jog_enable_z;
	hal_bit_t *jog_enable_a;
	hal_bit_t *jog_enable_feedrate;
	hal_bit_t *jog_enable_spindle;
	hal_float_t *jog_scale;
	hal_s32_t *jog_counts, *jog_counts_neg;

	hal_float_t *jog_velocity;
	hal_float_t *jog_max_velocity;
	hal_float_t *jog_increment;
	hal_bit_t *jog_plus_x, *jog_plus_y, *jog_plus_z, *jog_plus_a;
	hal_bit_t *jog_minus_x, *jog_minus_y, *jog_minus_z, *jog_minus_a;

	hal_bit_t *stepsize_up;
	hal_bit_t *stepsize_down;
	hal_s32_t *stepsize;
	hal_bit_t *sleeping;
	hal_bit_t *connected;
	hal_bit_t *require_pendant;
	hal_bit_t *inch_icon;
	hal_bit_t *zero_x;
	hal_bit_t *zero_y;
	hal_bit_t *zero_z;
	hal_bit_t *zero_a;
	hal_bit_t *gotozero_x;
	hal_bit_t *gotozero_y;
	hal_bit_t *gotozero_z;
	hal_bit_t *gotozero_a;
	hal_bit_t *half_x;
	hal_bit_t *half_y;
	hal_bit_t *half_z;
	hal_bit_t *half_a;
} xhc_hal_t;

#define STEP_UNDEFINED  -1
#define STEP_NONE      0x0
#define STEP_UP        0x1
#define STEP_DOWN      0x2

typedef struct {
	xhc_hal_t *hal;
	xhc_axis_t axis;
	xhc_button_t buttons[NB_MAX_BUTTONS];
	unsigned char button_code;

	char old_inc_step_status;
	unsigned char button_step;	// Used in simulation mode to handle the STEP increment

	// Variables for velocity computation
	hal_s32_t last_jog_counts;
	struct timeval last_tv;

	struct timeval last_wakeup;
} xhc_t;

static xhc_t xhc;

static int do_exit = 0;
static int do_reconnect = 0;
static bool wait_for_pendant_before_HAL = false;

struct libusb_transfer *transfer_in  = NULL;
unsigned char in_buf[32];
void setup_asynch_transfer(libusb_device_handle *dev_handle);


void init_xhc(xhc_t *xhc)
{
	memset(xhc, 0, sizeof(*xhc));
	xhc->old_inc_step_status = STEP_UNDEFINED;
	gettimeofday(&xhc->last_wakeup, NULL);
}

int xhc_encode_float(float v, unsigned char *buf)
{
	unsigned int int_v = round(fabs(v) * 10000.0);
	unsigned short int_part = int_v / 10000;
	unsigned short fract_part = int_v % 10000;
	if (v < 0) fract_part = fract_part | 0x8000;
	*(short *)buf = int_part;
	*((short *)buf+1) = fract_part;
	return 4;
}

int xhc_encode_s16(int v, unsigned char *buf)
{
	*(short *)buf = v;
	return 2;
}

void xhc_display_encode(xhc_t *xhc, unsigned char *data, int len)
{
	unsigned char buf[6*7];
	unsigned char *p = buf;
	int i;
	int packet;

	assert(len == 6*8);

	memset(buf, 0, sizeof(buf));

	*p++ = 0xFE;
	*p++ = 0xFD;
	*p++ = 0x0C;

	if (xhc->axis == axis_a) p += xhc_encode_float(round(1000 * *(xhc->hal->a_wc)) / 1000, p);
	else p += xhc_encode_float(round(1000 * *(xhc->hal->x_wc)) / 1000, p);
	p += xhc_encode_float(round(1000 * *(xhc->hal->y_wc)) / 1000, p);
	p += xhc_encode_float(round(1000 * *(xhc->hal->z_wc)) / 1000, p);
	if (xhc->axis == axis_a) p += xhc_encode_float(round(1000 * *(xhc->hal->a_mc)) / 1000, p);
	else p += xhc_encode_float(round(1000 * *(xhc->hal->x_mc)) / 1000, p);
	p += xhc_encode_float(round(1000 * *(xhc->hal->y_mc)) / 1000, p);
	p += xhc_encode_float(round(1000 * *(xhc->hal->z_mc)) / 1000, p);
	p += xhc_encode_s16(round(100.0 * *(xhc->hal->feedrate_override)), p);
	p += xhc_encode_s16(round(100.0 * *(xhc->hal->spindle_override)), p);
	p += xhc_encode_s16(round(60.0 * *(xhc->hal->feedrate)), p);
	p += xhc_encode_s16(round(60.0 * *(xhc->hal->spindle_rps)), p);

	switch (*(xhc->hal->stepsize)) {
	case    0: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_0; break;
	case    1: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_1; break;
	case    5: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_5; break;
	case   10: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_10; break;
	case   20: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_20; break;
	case   30: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_30; break;
	case   40: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_40; break;
	case   50: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_50; break;
	case  100: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_100; break;
	case  500: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_500; break;
	case 1000: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_1000; break;
	default:   //stepsize not supported on the display:
			   buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_0; break;
	}

    buf[FLAGS_BYTE] = 0;
    if (*(xhc->hal->inch_icon)) {
        buf[FLAGS_BYTE] |= 0x80;
    }

	// Multiplex to 6 USB transactions

	p = buf;
	for (packet=0; packet<6; packet++) {
		for (i=0; i<8; i++) {
			if (i == 0) data[i+8*packet] = 6;
			else data[i+8*packet] = *p++;
		}
	}
}

void xhc_set_display(libusb_device_handle *dev_handle, xhc_t *xhc)
{
	unsigned char data[6*8];
	int packet;

	xhc_display_encode(xhc, data, sizeof(data));

	for (packet=0; packet<6; packet++) {
		int r = libusb_control_transfer(dev_handle,
		              LIBUSB_DT_HID, //bmRequestType 0x21
		              LIBUSB_REQUEST_SET_CONFIGURATION, //bRequest 0x09
		              0x0306,         //wValue
		              0x00,           //wIndex
		              data+8*packet,  //*data
		              8,              //wLength
		              0);             //timeout
		if (r < 0) {
			do_reconnect = 1;
		}
	}
}

void hexdump(unsigned char *data, int len)
{
	int i;

	for (i=0; i<len; i++) printf("%02X ", data[i]);
}

void linuxcnc_simu(xhc_t *xhc)
{
	static int last_jog_counts;
	xhc_hal_t *hal = xhc->hal;

	// for simu, always step up
	*(hal->stepsize_up) = (xhc->button_step && xhc->button_code == xhc->button_step);

	if (*(hal->jog_counts) != last_jog_counts) {
		int delta_int = *(hal->jog_counts) - last_jog_counts;
		float delta = delta_int * *(hal->jog_scale);
		if (*(hal->jog_enable_x)) {
			*(hal->x_mc) = *(hal->x_mc) + delta;
			*(hal->x_wc) = *(hal->x_wc) + delta;
		}

		if (*(hal->jog_enable_y)) {
			*(hal->y_mc) = *(hal->y_mc) + delta;
			*(hal->y_wc) = *(hal->y_wc) + delta;
		}

		if (*(hal->jog_enable_z)) {
			*(hal->z_mc) = *(hal->z_mc) + delta;
			*(hal->z_wc) = *(hal->z_wc) + delta;
		}

		if (*(hal->jog_enable_a)) {
			*(hal->a_mc) = *(hal->a_mc) + delta;
			*(hal->a_wc) = *(hal->a_wc) + delta;
		}

		if (*(hal->jog_enable_spindle)) {
			*(hal->spindle_override) = *(hal->spindle_override) + delta_int * 0.01;
			if (*(hal->spindle_override) > 1) *(hal->spindle_override) = 1;
			if (*(hal->spindle_override) < 0) *(hal->spindle_override) = 0;
			*(hal->spindle_rps) = 25000.0/60.0 * *(hal->spindle_override);
		}

		if (*(hal->jog_enable_feedrate)) {
			*(hal->feedrate_override) = *(hal->feedrate_override) + delta_int * 0.01;
			if (*(hal->feedrate_override) > 1) *(hal->feedrate_override) = 1;
			if (*(hal->feedrate_override) < 0) *(hal->feedrate_override) = 0;
			*(hal->feedrate) = 3000.0/60.0 * *(hal->feedrate_override);
		}

		last_jog_counts = *(hal->jog_counts);
	}
}

void compute_velocity(xhc_t *xhc)
{
	timeval now, delta_tv;
	gettimeofday(&now, NULL);

	if (xhc->last_tv.tv_sec == 0) xhc->last_tv = now;
	timersub(&now, &xhc->last_tv, &delta_tv);
	float elapsed = delta_tv.tv_sec + 1e-6f*delta_tv.tv_usec;
	if (elapsed <= 0) return;

	float delta_pos = (*(xhc->hal->jog_counts) - xhc->last_jog_counts) * *(xhc->hal->jog_scale);
	float velocity = *(xhc->hal->jog_max_velocity) * 60.0f * *(xhc->hal->jog_scale);
	float k = 0.05f;

	if (delta_pos) {
		*(xhc->hal->jog_velocity) = (1 - k) * *(xhc->hal->jog_velocity) + k * velocity;
		*(xhc->hal->jog_increment) = fabs(delta_pos);
		*(xhc->hal->jog_plus_x) = (delta_pos > 0) && *(xhc->hal->jog_enable_x);
		*(xhc->hal->jog_minus_x) = (delta_pos < 0) && *(xhc->hal->jog_enable_x);
		*(xhc->hal->jog_plus_y) = (delta_pos > 0) && *(xhc->hal->jog_enable_y);
		*(xhc->hal->jog_minus_y) = (delta_pos < 0) && *(xhc->hal->jog_enable_y);
		*(xhc->hal->jog_plus_z) = (delta_pos > 0) && *(xhc->hal->jog_enable_z);
		*(xhc->hal->jog_minus_z) = (delta_pos < 0) && *(xhc->hal->jog_enable_z);
		*(xhc->hal->jog_plus_a) = (delta_pos > 0) && *(xhc->hal->jog_enable_a);
		*(xhc->hal->jog_minus_a) = (delta_pos < 0) && *(xhc->hal->jog_enable_a);
		xhc->last_jog_counts = *(xhc->hal->jog_counts);
		xhc->last_tv = now;
	}
	else {
		*(xhc->hal->jog_velocity) = (1 - k) * *(xhc->hal->jog_velocity);
		if (elapsed > 0.25) {
			*(xhc->hal->jog_velocity) = 0;
			*(xhc->hal->jog_plus_x) = 0;
			*(xhc->hal->jog_minus_x) = 0;
			*(xhc->hal->jog_plus_y) = 0;
			*(xhc->hal->jog_minus_y) = 0;
			*(xhc->hal->jog_plus_z) = 0;
			*(xhc->hal->jog_minus_z) = 0;
			*(xhc->hal->jog_plus_a) = 0;
			*(xhc->hal->jog_minus_a) = 0;
		}
	}
}

void handle_step(xhc_t *xhc)
{
	int _inc_step_status = STEP_NONE;
	int _stepsize = *(xhc->hal->stepsize);	// Use a local variable to avoid STEP display as 0 on pendant during transitions

	if (*(xhc->hal->stepsize_up)) {
	       _inc_step_status = STEP_UP;
	   if (*(xhc->hal->stepsize_down)) {
	       _inc_step_status = STEP_NONE; // none if both pressed
	   }
	} else if (*(xhc->hal->stepsize_down)) {
	      _inc_step_status = STEP_DOWN;
	} else {
	      _inc_step_status = STEP_NONE;
	}

	if (_inc_step_status  !=  xhc->old_inc_step_status) {
	    if (_inc_step_status == STEP_UP) {
	        stepsize_idx++;
	        // restart idx when 0 terminator reached:
	        if (stepsize_sequence[stepsize_idx] == 0) stepsize_idx = 0;
	    }
	    if (_inc_step_status == STEP_DOWN) {
	        stepsize_idx--;
	        // restart at stepsize_last_idx when stepsize_idx < 0
	        if (stepsize_idx < 0) stepsize_idx = stepsize_last_idx;
	    }
	    _stepsize = stepsize_sequence[stepsize_idx];
	}

	xhc->old_inc_step_status = _inc_step_status;

	*(xhc->hal->stepsize) = _stepsize;
	*(xhc->hal->jog_scale) = *(xhc->hal->stepsize) * 0.001f;
}

void cb_response_in(struct libusb_transfer *transfer)
{
	int i;
	if (!*(xhc.hal->connected)) return;

	if (transfer->actual_length > 0) {
		if (simu_mode) hexdump(in_buf, transfer->actual_length);

		xhc.button_code = in_buf[1];
		xhc.axis = (xhc_axis_t)in_buf[3];

		*(xhc.hal->jog_counts) = *(xhc.hal->jog_counts) + ((signed char)in_buf[4]);
		*(xhc.hal->jog_counts_neg) = - *(xhc.hal->jog_counts);
		*(xhc.hal->jog_enable_off) = (xhc.axis == axis_off);
		*(xhc.hal->jog_enable_x) = (xhc.axis == axis_x);
		*(xhc.hal->jog_enable_y) = (xhc.axis == axis_y);
		*(xhc.hal->jog_enable_z) = (xhc.axis == axis_z);
		*(xhc.hal->jog_enable_a) = (xhc.axis == axis_a);
		*(xhc.hal->jog_enable_feedrate) = (xhc.axis == axis_feed);
		*(xhc.hal->jog_enable_spindle) = (xhc.axis == axis_spindle);

		for (i=0; i<NB_MAX_BUTTONS; i++) {
			if (!xhc.hal->button_pin[i]) continue;
			*(xhc.hal->button_pin[i]) = (xhc.button_code == xhc.buttons[i].code);
            if (strcmp("button-zero", xhc.buttons[i].pin_name) == 0) {
               *(xhc.hal->zero_x) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_x);
               *(xhc.hal->zero_y) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_y);
               *(xhc.hal->zero_z) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_z);
               *(xhc.hal->zero_a) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_a);
            }
            if (strcmp("button-goto-zero", xhc.buttons[i].pin_name) == 0) {
               *(xhc.hal->gotozero_x) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_x);
               *(xhc.hal->gotozero_y) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_y);
               *(xhc.hal->gotozero_z) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_z);
               *(xhc.hal->gotozero_a) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_a);
            }
            if (strcmp("button-half", xhc.buttons[i].pin_name) == 0) {
               *(xhc.hal->half_x) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_x);
               *(xhc.hal->half_y) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_y);
               *(xhc.hal->half_z) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_z);
               *(xhc.hal->half_a) = (xhc.button_code == xhc.buttons[i].code) && (xhc.axis == axis_a);
            }
			if (simu_mode && *(xhc.hal->button_pin[i])) {
				printf("%s pressed", xhc.buttons[i].pin_name);
			}
		}
		if (simu_mode) {
			if ((signed char)in_buf[4] != 0) printf(" delta %+3d",(signed char)in_buf[4]);
			printf("\n");
		}

		//detect pendant going to sleep (occurs for 18 button pendant)
		if (   in_buf[0]==0x04
			&& in_buf[1]==0
			&& in_buf[2]==0
			&& in_buf[3]==0
			&& in_buf[4]==0
			&& in_buf[5]==0) {
				*(xhc.hal->sleeping) = 1;
				if (simu_mode) {
					struct timeval now;
					gettimeofday(&now, NULL);
					fprintf(stderr,"Sleep, idle for %ld seconds\n",
						               now.tv_sec - xhc.last_wakeup.tv_sec);
				}
			} else {
				gettimeofday(&xhc.last_wakeup, NULL);
				if (*(xhc.hal->sleeping)) {
					if (simu_mode) {
						fprintf(stderr,"Wake\n");
					}
				}
				*(xhc.hal->sleeping) = 0;
			}
	}

	libusb_submit_transfer(transfer);
}

void setup_asynch_transfer(libusb_device_handle *dev_handle)
{
	libusb_fill_bulk_transfer( transfer_in, dev_handle, (0x1 | LIBUSB_ENDPOINT_IN),
		in_buf, sizeof(in_buf),
		cb_response_in, NULL, 0); // no user data
	libusb_submit_transfer(transfer_in);
}

static void quit(int sig)
{
	(void)sig;
	do_exit = 1;
}

static int hal_pin_simu(char *pin_name, void **ptr, int s)
{
	printf("Creating pin: %s\n", pin_name);
	*ptr = calloc(1, s);
	return 0;
}

int _hal_pin_float_newf(hal_pin_dir_t dir, hal_float_t ** data_ptr_addr, int comp_id, const char *fmt, ...)
{
	char pin_name[256];
    va_list args;
    va_start(args,fmt);
	vsprintf(pin_name, fmt, args);
	va_end(args);

    if (simu_mode) {
    	return hal_pin_simu(pin_name, ( void**)data_ptr_addr, sizeof(*data_ptr_addr));
    }
    else {
    	return hal_pin_float_new(pin_name, dir, data_ptr_addr, comp_id);
    }
}

int _hal_pin_s32_newf(hal_pin_dir_t dir, hal_s32_t ** data_ptr_addr, int comp_id, const char *fmt, ...)
{
	char pin_name[256];
    va_list args;
    va_start(args,fmt);
	vsprintf(pin_name, fmt, args);
	va_end(args);

    if (simu_mode) {
    	return hal_pin_simu(pin_name, ( void**)data_ptr_addr, sizeof(*data_ptr_addr));
    }
    else {
    	return hal_pin_s32_new(pin_name, dir, data_ptr_addr, comp_id);
    }
}

int _hal_pin_bit_newf(hal_pin_dir_t dir, hal_bit_t ** data_ptr_addr, int comp_id, const char *fmt, ...)
{
	char pin_name[256];
    va_list args;
    va_start(args,fmt);
	vsprintf(pin_name, fmt, args);
	va_end(args);

    if (simu_mode) {
    	return hal_pin_simu(pin_name, ( void**)data_ptr_addr, sizeof(*data_ptr_addr));
    }
    else {
    	return hal_pin_bit_new(pin_name, dir, data_ptr_addr, comp_id);
    }
}

static int hal_setup()
{
	int r, i;

	if (!simu_mode) {
		hal_comp_id = hal_init(modname);
		if (hal_comp_id < 1) {
			fprintf(stderr, "%s: ERROR: hal_init failed\n", modname);
			exit(1);
		}

		xhc.hal = (xhc_hal_t *)hal_malloc(sizeof(xhc_hal_t));
		if (xhc.hal == NULL) {
			fprintf(stderr, "%s: ERROR: unable to allocate HAL shared memory\n", modname);
			exit(1);
		}
	}
	else {
		xhc.hal = (xhc_hal_t *)calloc(1, sizeof(xhc_hal_t));
	}

    r = 0;

    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->x_mc), hal_comp_id, "%s.x.pos-absolute", modname);
    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->y_mc), hal_comp_id, "%s.y.pos-absolute", modname);
    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->z_mc), hal_comp_id, "%s.z.pos-absolute", modname);
    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->a_mc), hal_comp_id, "%s.a.pos-absolute", modname);

    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->x_wc), hal_comp_id, "%s.x.pos-relative", modname);
    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->y_wc), hal_comp_id, "%s.y.pos-relative", modname);
    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->z_wc), hal_comp_id, "%s.z.pos-relative", modname);
    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->a_wc), hal_comp_id, "%s.a.pos-relative", modname);

    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->feedrate), hal_comp_id, "%s.feed-value", modname);
    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->feedrate_override), hal_comp_id, "%s.feed-override", modname);
    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->spindle_rps), hal_comp_id, "%s.spindle-rps", modname);
    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->spindle_override), hal_comp_id, "%s.spindle-override", modname);

    for (i=0; i<NB_MAX_BUTTONS; i++) {
        if (!xhc.buttons[i].pin_name[0]) continue;
        r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->button_pin[i]), hal_comp_id, "%s.%s", modname, xhc.buttons[i].pin_name);
        if (strcmp("button-zero", xhc.buttons[i].pin_name) == 0) {
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->zero_x), hal_comp_id, "%s.%s-x", modname, xhc.buttons[i].pin_name);
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->zero_y), hal_comp_id, "%s.%s-y", modname, xhc.buttons[i].pin_name);
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->zero_z), hal_comp_id, "%s.%s-z", modname, xhc.buttons[i].pin_name);
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->zero_a), hal_comp_id, "%s.%s-a", modname, xhc.buttons[i].pin_name);
        }
        if (strcmp("button-goto-zero", xhc.buttons[i].pin_name) == 0) {
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->gotozero_x), hal_comp_id, "%s.%s-x", modname, xhc.buttons[i].pin_name);
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->gotozero_y), hal_comp_id, "%s.%s-y", modname, xhc.buttons[i].pin_name);
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->gotozero_z), hal_comp_id, "%s.%s-z", modname, xhc.buttons[i].pin_name);
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->gotozero_a), hal_comp_id, "%s.%s-a", modname, xhc.buttons[i].pin_name);
        }
        if (strcmp("button-half", xhc.buttons[i].pin_name) == 0) {
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->half_x), hal_comp_id, "%s.%s-x", modname, xhc.buttons[i].pin_name);
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->half_y), hal_comp_id, "%s.%s-y", modname, xhc.buttons[i].pin_name);
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->half_z), hal_comp_id, "%s.%s-z", modname, xhc.buttons[i].pin_name);
            r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->half_a), hal_comp_id, "%s.%s-a", modname, xhc.buttons[i].pin_name);
        }
        if (strcmp("button-step", xhc.buttons[i].pin_name) == 0) xhc.button_step = xhc.buttons[i].code;
    }

    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->sleeping), hal_comp_id, "%s.sleeping", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->connected), hal_comp_id, "%s.connected", modname);
    r |= _hal_pin_bit_newf(HAL_IN,  &(xhc.hal->stepsize_up), hal_comp_id, "%s.stepsize-up", modname);
    r |= _hal_pin_bit_newf(HAL_IN,  &(xhc.hal->stepsize_down), hal_comp_id, "%s.stepsize-down", modname);
    r |= _hal_pin_s32_newf(HAL_OUT, &(xhc.hal->stepsize), hal_comp_id, "%s.stepsize", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->require_pendant), hal_comp_id, "%s.require_pendant", modname);
    r |= _hal_pin_bit_newf(HAL_IN,  &(xhc.hal->inch_icon), hal_comp_id, "%s.inch-icon", modname);

    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_off), hal_comp_id, "%s.jog.enable-off", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_x), hal_comp_id, "%s.jog.enable-x", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_y), hal_comp_id, "%s.jog.enable-y", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_z), hal_comp_id, "%s.jog.enable-z", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_a), hal_comp_id, "%s.jog.enable-a", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_feedrate), hal_comp_id, "%s.jog.enable-feed-override", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_spindle), hal_comp_id, "%s.jog.enable-spindle-override", modname);

    r |= _hal_pin_float_newf(HAL_OUT, &(xhc.hal->jog_scale), hal_comp_id, "%s.jog.scale", modname);
    r |= _hal_pin_s32_newf(HAL_OUT, &(xhc.hal->jog_counts), hal_comp_id, "%s.jog.counts", modname);
    r |= _hal_pin_s32_newf(HAL_OUT, &(xhc.hal->jog_counts_neg), hal_comp_id, "%s.jog.counts-neg", modname);

    r |= _hal_pin_float_newf(HAL_OUT, &(xhc.hal->jog_velocity), hal_comp_id, "%s.jog.velocity", modname);
    r |= _hal_pin_float_newf(HAL_IN, &(xhc.hal->jog_max_velocity), hal_comp_id, "%s.jog.max-velocity", modname);
    r |= _hal_pin_float_newf(HAL_OUT, &(xhc.hal->jog_increment), hal_comp_id, "%s.jog.increment", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_plus_x), hal_comp_id, "%s.jog.plus-x", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_minus_x), hal_comp_id, "%s.jog.minus-x", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_plus_y), hal_comp_id, "%s.jog.plus-y", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_minus_y), hal_comp_id, "%s.jog.minus-y", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_plus_z), hal_comp_id, "%s.jog.plus-z", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_minus_z), hal_comp_id, "%s.jog.minus-z", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_plus_a), hal_comp_id, "%s.jog.plus-a", modname);
    r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_minus_a), hal_comp_id, "%s.jog.minus-a", modname);

	return r;
}

int read_ini_file(char *filename)
{
	IniFile iniFile;
	std::optional<const char*> bt;
	int nb_buttons = 0;
	if (!iniFile.Open(filename)) {
		fprintf(stderr, "%s: Could not open configuration file: %s\n",
			modname, filename);
		return -1;
	}

	while ((bt = iniFile.Find("BUTTON", section, nb_buttons+1)) && nb_buttons < NB_MAX_BUTTONS) {
		if (sscanf(*bt, "%x:%255s", &xhc.buttons[nb_buttons].code, xhc.buttons[nb_buttons].pin_name) !=2 ) {
			fprintf(stderr, "%s: syntax error\n", *bt);
			return -1;
		}
		nb_buttons++;
	}

	return 0;
}

#define STRINGIFY_IMPL(S) #S
#define STRINGIFY(s) STRINGIFY_IMPL(s)

static void Usage(char *name)
{
	fprintf(stderr, "%s version %s by Frederic RIBLE (frible@teaser.fr)\n", name, PACKAGE_VERSION);
    fprintf(stderr, "\n");
    fprintf(stderr, "Usage: %s [-I button-cfg-file] [-h] [-H] [-s n]\n", name);
    fprintf(stderr, " -I button-cfg-file: configuration file defining the MPG keyboard layout\n");
    fprintf(stderr, " -h: usage (this)\n");
    fprintf(stderr, " -H: run in real-time HAL mode (run in simulation mode by default)\n");
    fprintf(stderr, " -x: wait for pendant detection before creating HAL pins\n");
    fprintf(stderr, " -s: step sequence (multiplied by 0.001 unit):\n");
    fprintf(stderr, "     1: 1,10,100,1000 (default)\n");
    fprintf(stderr, "     2: 1,5,10,20\n");
    fprintf(stderr, "     3: 1,10,100\n");
    fprintf(stderr, "     4: 1,5,10,20,50,100\n");
    fprintf(stderr, "     5: 1,10,50,100,1000\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "Configuration file section format:\n");
    fprintf(stderr, "[XHC-HB04]\n");
    fprintf(stderr, "BUTTON=XN:button-thenameN\n");
    fprintf(stderr, "...\n");
    fprintf(stderr, "    where XN=hexcode, button-thenameN=nameforbutton\n");
}

int main (int argc,char **argv)
{
	libusb_device **devs;
    libusb_device_handle *dev_handle;
	libusb_context *ctx = NULL;
	int r,idx;
	ssize_t cnt;
#define MAX_WAIT_SECS 10
	int wait_secs = 0;

    int opt;
    bool hal_ready_done = false;

    init_xhc(&xhc);

    while ((opt = getopt(argc, argv, "HhI:xs:")) != -1) {
        switch (opt) {
        case 'I':
            if (read_ini_file(optarg)) {
                printf("Problem reading ini file: %s\n\n",optarg);
                Usage(argv[0]);
                exit(EXIT_FAILURE);
            }
            break;
        case 'H':
        	simu_mode = false;
        	break;
        case 's':
            switch (optarg[0]) {
              case '1': stepsize_sequence = stepsize_sequence_1;break;
              case '2': stepsize_sequence = stepsize_sequence_2;break;
              case '3': stepsize_sequence = stepsize_sequence_3;break;
              case '4': stepsize_sequence = stepsize_sequence_4;break;
              case '5': stepsize_sequence = stepsize_sequence_5;break;
              default:
                printf("Unknown sequence: %s\n\n",optarg);
                Usage(argv[0]);
                exit(EXIT_FAILURE);
                break;
            }
            break;
        case 'x':
        	wait_for_pendant_before_HAL = true;
        	break;
        default:
        	Usage(argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    // compute the last valid idx for use with stepsize-down
    for (idx=0; idx < MAX_STEPSIZE_SEQUENCE; idx++) {
       if (stepsize_sequence[idx] == 0) break;
    }
    stepsize_last_idx  =  idx - 1;

	if(hal_setup()) {
		fprintf(stderr, "%s: ERROR: hal pin creation failed\n", modname);
		exit(EXIT_FAILURE);
	}

    signal(SIGINT, quit);
	signal(SIGTERM, quit);

    if (!wait_for_pendant_before_HAL && !simu_mode) {
    	hal_ready(hal_comp_id);
    	hal_ready_done = true;
    }

	while (!do_exit) {
    	//on reconnect wait for device to be gone
    	if (do_reconnect == 1) {
    		sleep(5);
    		do_reconnect = 0;
    	}

		r = libusb_init(&ctx);

		if(r < 0) {
			perror("libusb_init");
			return 1;
		}
#if LIBUSB_API_VERSION >= 0x01000106
		libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, 2);
#else
		libusb_set_debug(ctx, 2);
#endif
		// use environmental variable LIBUSB_DEBUG if needed

		printf("%s: waiting for XHC-HB04 device\n",modname);
		*(xhc.hal->connected) = 0;
		wait_secs = 0;
		*(xhc.hal->require_pendant) = wait_for_pendant_before_HAL;
		*(xhc.hal->stepsize) = stepsize_sequence[0];

		do {
			cnt = libusb_get_device_list(ctx, &devs);
			if (cnt < 0) {
				perror("libusb_get_device_list");
				return 1;
			}

			dev_handle = libusb_open_device_with_vid_pid(ctx, 0x10CE, 0xEB70);
			libusb_free_device_list(devs, 1);
			if (dev_handle == NULL) {
				if (wait_for_pendant_before_HAL) {
					wait_secs++;
					if (wait_secs >= MAX_WAIT_SECS/2) {
						printf("%s: waiting for XHC-HB04 device (%d)\n",modname,wait_secs);
					}
					if (wait_secs > MAX_WAIT_SECS) {
						printf("%s: MAX_WAIT_SECS exceeded, exiting\n",modname);
						exit(1);
					}
				}
				sleep(1);
			}
		} while(dev_handle == NULL && !do_exit);

		if (dev_handle) printf("%s: found XHC-HB04 device\n",modname);

		if (dev_handle) {
			if 	(libusb_kernel_driver_active(dev_handle, 0) == 1) {
				libusb_detach_kernel_driver(dev_handle, 0);
			}

			r = libusb_claim_interface(dev_handle, 0);
			if (r < 0) {
				perror("libusb_claim_interface");
				return 1;
			}
			//allocate the transfer struct here only once after successful connection
			transfer_in  = libusb_alloc_transfer(0);
		}

		*(xhc.hal->connected) = 1;

	    if (!hal_ready_done && !simu_mode) {
	    	hal_ready(hal_comp_id);
	    	hal_ready_done = true;
	    }

		if (dev_handle) {
			setup_asynch_transfer(dev_handle);
			xhc_set_display(dev_handle, &xhc);
		}

		if (dev_handle) {
			while (!do_exit && !do_reconnect) {
				struct timeval tv;
				tv.tv_sec  = 0;
				tv.tv_usec = 30000;
				r = libusb_handle_events_timeout(ctx, &tv);
				compute_velocity(&xhc);
			    if (simu_mode) linuxcnc_simu(&xhc);
				handle_step(&xhc);
				xhc_set_display(dev_handle, &xhc);
			}
			*(xhc.hal->connected) = 0;
            printf("%s: connection lost, cleaning up\n",modname);
			if (*(xhc.hal->require_pendant)) {
				do_exit = 1;
			}
			libusb_cancel_transfer(transfer_in); // ignore result
			assert (0 == libusb_handle_events_completed(ctx, nullptr));
			libusb_free_transfer(transfer_in);
			r = libusb_release_interface(dev_handle, 0);
			assert (r == 0 || r == LIBUSB_ERROR_NO_DEVICE);
			libusb_close(dev_handle);
		} else {
			while (!do_exit) usleep(70000);
		}
		libusb_exit(ctx);
	} // while (!do_exit)
}
