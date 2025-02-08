/********************************************************************
* Description:  encoder_ratio.c
*               A HAL component that can be used to synchronize two
*               axes (like an "electronic gear").
*
* Author: John Kasunich
* License: GPL Version 2
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change: 
********************************************************************/
/** This file, 'encoder_ratio.c', is a HAL component that can be used
    to synchronize two axes (like an "electronic gear").  It counts
    encoder pulses from both axes in software, and produces an error
    value that can be used with a PID loop to make the slave encoder
    track the master encoder with a specific ratio. The maximum count
    rate will depend on the speed of the PC, but is expected to exceed
    5KHz for even the slowest computers, and may reach 10-15KHz on fast
    ones.  It is a realtime component.

    This module supports up to eight axis pairs.  The number of pairs
    is set by the module parameter 'num_chan' when the component is
    insmod'ed.  Alternatively, use the names= specifier and a list of
    unique names separated by commas.
    The names= and num_chan= specifiers are mutually exclusive.

    The module exports pins and parameters for each axis pair as follows:

    Input Pins:

    encoder-ratio.N.master-A   (bit) Phase A of master axis encoder
    encoder-ratio.N.master-B   (bit) Phase B of master axis encoder
    encoder-ratio.N.slave-A    (bit) Phase A of slave axis encoder
    encoder-ratio.N.slave-B    (bit) Phase B of slave axis encoder
    encoder-ratio.N.enable     (bit) Enables master-slave tracking

    Output Pins:

    encoder-ratio.N.error      (float) Position error of slave (in revs)

    Parameters:

    encoder-ratio.N.master-ppr     (u32) Master axis PPR
    encoder-ratio.N.slave-ppr      (u32) Slave axis PPR
    encoder-ratio.N.master-teeth   (u32) "teeth" on master "gear"
    encoder-ratio.N.slave-teeth    (u32) "teeth" on slave "gear"

    The module also exports two functions.  "encoder-ratio.sample"
    must be called in a high speed thread, at least twice the maximum
    desired count rate.  "encoder-ratio.update" can be called at a
    much slower rate, and updates the output pin(s).

    When the enable pin is FALSE, the error pin simply reports the
    slave axis position, in revolutions.  As such, it would normally
    be connected to the feedback pin of a PID block for closed loop
    control of the slave axis.  Normally the command input of the
    PID block is left unconnected (zero), so the slave axis simply
    sits still.  However when the enable input goes TRUE, the error
    pin becomes the slave position minus the scaled master position.
    The scale factor is the ratio of master teeth to slave teeth.
    As the master moves, error becomes non-zero, and the PID loop
    will drive the slave axis to track the master.
*/

/** Copyright (C) 2004 John Kasunich
                       <jmkasunich AT users DOT sourceforge DOT net>
*/

/** This program is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the authors of this software can not, and do not, take
    any responsibility for such compliance.

    This code was written as part of the EMC HAL project.  For more
    information, go to www.linuxcnc.org.
*/

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"		/* HAL public API decls */

/* module information */
MODULE_AUTHOR("John Kasunich");
MODULE_DESCRIPTION("Encoder Ratio Module for HAL");
MODULE_LICENSE("GPL");
static int num_chan;	/* number of channels*/
static int default_num_chan = 1;
RTAPI_MP_INT(num_chan, "number of channels");

static int howmany;
#define MAX_CHAN 8
static char *names[MAX_CHAN] = {0,};
RTAPI_MP_ARRAY_STRING(names,MAX_CHAN,"encoder_ratio names");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* this structure contains the runtime data for a single counter */

typedef struct {
    hal_bit_t *master_A;	/* quadrature input */
    hal_bit_t *master_B;	/* quadrature input */
    hal_bit_t *slave_A;		/* quadrature input */
    hal_bit_t *slave_B;		/* quadrature input */
    hal_bit_t *enable;		/* enable input */
    unsigned char master_state;	/* quad decode state machine state */
    unsigned char slave_state;	/* quad decode state machine state */
    int raw_error;		/* internal data */
    int master_increment;	/* internal data */
    int slave_increment;	/* internal data */
    double output_scale;	/* internal data */
    hal_float_t *error;		/* error output */
    hal_u32_t *master_ppr;	/* parameter: master encoder PPR */
    hal_u32_t *slave_ppr;	/* parameter: slave encoder PPR */
    hal_u32_t *master_teeth;	/* parameter: master "gear" tooth count */
    hal_u32_t *slave_teeth;	/* parameter: slave "gear" tooth count */
} encoder_pair_t;

/* pointer to array of counter_t structs in shmem, 1 per counter */
static encoder_pair_t *encoder_pair_array;

/* bitmasks for quadrature decode state machine */
#define SM_PHASE_A_MASK 0x01
#define SM_PHASE_B_MASK 0x02
#define SM_LOOKUP_MASK  0x0F
#define SM_CNT_UP_MASK  0x40
#define SM_CNT_DN_MASK  0x80

/* Lookup table for quadrature decode state machine.  This machine
   will reject glitches on either input (will count up 1 on glitch,
   down 1 after glitch), and on both inputs simultaneously (no count
   at all)  In theory, it can count once per cycle, in practice the
   maximum count rate should be at _least_ 10% below the sample rate,
   and preferable around half the sample rate.  It counts every
   edge of the quadrature waveform, 4 counts per complete cycle.
*/
static const unsigned char lut[16] = {
    0x00, 0x44, 0x88, 0x0C, 0x80, 0x04, 0x08, 0x4C,
    0x40, 0x04, 0x08, 0x8C, 0x00, 0x84, 0x48, 0x0C
};

/* other globals */
static int comp_id;		/* component ID */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static int export_encoder_pair(int num, encoder_pair_t * addr, char* prefix);
static void sample(void *arg, long period);
static void update(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/


int rtapi_app_main(void)
{
    int n, retval,i;

    if(num_chan && names[0]) {
        rtapi_print_msg(RTAPI_MSG_ERR,"num_chan= and names= are mutually exclusive\n");
        return -EINVAL;
    }
    if(!num_chan && !names[0]) num_chan = default_num_chan;

    if(num_chan) {
        howmany = num_chan;
    } else {
        howmany = 0;
        for (i = 0; i < MAX_CHAN; i++) {
            if ( (names[i] == NULL) || (*names[i] == 0) ){
                break;
            }
            howmany = i + 1;
        }
    }

    /* test for number of channels */
    if ((howmany <= 0) || (howmany > MAX_CHAN)) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "ENCODER_RATIO: ERROR: invalid number of channels: %d\n", howmany);
	return -1;
    }
    /* have good config info, connect to the HAL */
    comp_id = hal_init("encoder_ratio");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "ENCODER_RATIO: ERROR: hal_init() failed\n");
	return -1;
    }
    /* allocate shared memory for encoder data */
    encoder_pair_array = hal_malloc(howmany * sizeof(encoder_pair_t));
    if (encoder_pair_array == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "ENCODER_RATIO: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -1;
    }
    /* set up each encoder pair */
    i = 0; // for names= items
    for (n = 0; n < howmany; n++) {
	/* export all vars */
        if(num_chan) {
            char buf[HAL_NAME_LEN + 1];
            rtapi_snprintf(buf, sizeof(buf), "encoder-ratio.%d", n);
	    retval = export_encoder_pair(n, &(encoder_pair_array[n]), buf);
        } else {
	    retval = export_encoder_pair(n, &(encoder_pair_array[n]), names[i++]);
        }

	if (retval != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"ENCODER_RATIO: ERROR: counter %d var export failed\n", n);
	    hal_exit(comp_id);
	    return -1;
	}
	/* init encoder pair */
	encoder_pair_array[n].master_state = 0;
	encoder_pair_array[n].slave_state = 0;
	encoder_pair_array[n].master_increment = 0;
	encoder_pair_array[n].slave_increment = 0;
	encoder_pair_array[n].raw_error = 0;
	encoder_pair_array[n].output_scale = 1.0;
	*(encoder_pair_array[n].error) = 0.0;
    }
    /* export functions */
    retval = hal_export_funct("encoder-ratio.sample", sample,
	encoder_pair_array, 0, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "ENCODER_RATIO: ERROR: sample funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }
    retval = hal_export_funct("encoder-ratio.update", update,
	encoder_pair_array, 1, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "ENCODER_RATIO: ERROR: update funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }
    rtapi_print_msg(RTAPI_MSG_INFO,
	"ENCODER_RATIO: installed %d encoder_ratio blocks\n", howmany);
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}

/***********************************************************************
*            REALTIME ENCODER COUNTING AND UPDATE FUNCTIONS            *
************************************************************************/

static void sample(void *arg, long period)
{
    (void)period;
    encoder_pair_t *pair;
    int n;
    unsigned char state;

    pair = arg;
    for (n = 0; n < howmany; n++) {
	/* detect transitions on master encoder */
	/* get state machine current state */
	state = pair->master_state;
	/* add input bits to state code */
	if (*(pair->master_A)) {
	    state |= SM_PHASE_A_MASK;
	}
	if (*(pair->master_B)) {
	    state |= SM_PHASE_B_MASK;
	}
	/* look up new state */
	state = lut[state & SM_LOOKUP_MASK];
	/* are we enabled? */
	if ( *(pair->enable) != 0 ) {
	    /* has an edge been detected? */
	    if (state & SM_CNT_UP_MASK) {
		pair->raw_error -= pair->master_increment;
	    } else if (state & SM_CNT_DN_MASK) {
		pair->raw_error += pair->master_increment;
	    }
	}
	/* save state machine state */
	pair->master_state = state;
	/* detect transitions on slave encoder */
	/* get state machine current state */
	state = pair->slave_state;
	/* add input bits to state code */
	if (*(pair->slave_A)) {
	    state |= SM_PHASE_A_MASK;
	}
	if (*(pair->slave_B)) {
	    state |= SM_PHASE_B_MASK;
	}
	/* look up new state */
	state = lut[state & SM_LOOKUP_MASK];
	/* has an edge been detected? */
	if (state & SM_CNT_UP_MASK) {
	    pair->raw_error += pair->slave_increment;
	} else if (state & SM_CNT_DN_MASK) {
	    pair->raw_error -= pair->slave_increment;
	}
	/* save state machine state */
	pair->slave_state = state;
	/* move on to next pair */
	pair++;
    }
    /* done */
}

static void update(void *arg, long period)
{
    (void)period;
    encoder_pair_t *pair;
    int n;

    pair = arg;
    for (n = 0; n < howmany; n++) {
	/* scale raw error to output pin */
	if ( pair->output_scale > 0 ) {
	    *(pair->error) = pair->raw_error / pair->output_scale;
	}
	/* update scale factors (only needed if params change, but
	   it's faster to do it every time than to detect changes.) */
	pair->master_increment = *(pair->master_teeth) * *(pair->slave_ppr);
	pair->slave_increment = *(pair->slave_teeth) * *(pair->master_ppr);
	pair->output_scale = *(pair->master_ppr) * *(pair->slave_ppr) * *(pair->slave_teeth);
	/* move on to next pair */
	pair++;
    }
    /* done */
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static int export_encoder_pair(int num, encoder_pair_t * addr, char* prefix)
{
    (void)num;
    int retval, msg;

    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);

    /* export pins for the quadrature inputs */
    retval = hal_pin_bit_newf(HAL_IN, &(addr->master_A), comp_id,
			      "%s.master-A", prefix);
    if (retval != 0) {
	return retval;
    }
    retval = hal_pin_bit_newf(HAL_IN, &(addr->master_B), comp_id,
			      "%s.master-B", prefix);
    if (retval != 0) {
	return retval;
    }
    retval = hal_pin_bit_newf(HAL_IN, &(addr->slave_A), comp_id,
			      "%s.slave-A", prefix);
    if (retval != 0) {
	return retval;
    }
    retval = hal_pin_bit_newf(HAL_IN, &(addr->slave_B), comp_id,
			      "%s.slave-B", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for the enable input */
    retval = hal_pin_bit_newf(HAL_IN, &(addr->enable), comp_id,
			      "%s.enable", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for output */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->error), comp_id,
				"%s.error", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pins for config info() */
    retval = hal_pin_u32_newf(HAL_IO, &(addr->master_ppr), comp_id,
			      "%s.master-ppr", prefix);
    if (retval != 0) {
	return retval;
    }
    retval = hal_pin_u32_newf(HAL_IO, &(addr->slave_ppr), comp_id,
			      "%s.slave-ppr", prefix);
    if (retval != 0) {
	return retval;
    }
    retval = hal_pin_u32_newf(HAL_IO, &(addr->master_teeth), comp_id,
			      "%s.master-teeth", prefix);
    if (retval != 0) {
	return retval;
    }
    retval = hal_pin_u32_newf(HAL_IO, &(addr->slave_teeth), comp_id,
			      "%s.slave-teeth", prefix);
    if (retval != 0) {
	return retval;
    }
    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
}
