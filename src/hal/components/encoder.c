/********************************************************************
* Description:  encoder.c
*               This file, 'encoder.c', is a HAL component that 
*               provides software based counting of quadrature 
*               encoder signals.
*
* Author: John Kasunich
* License: GPL Version 2
*    
* Copyright (c) 2003 All rights reserved.
*
* Last change: 
********************************************************************/
/** This file, 'encoder.c', is a HAL component that provides software
    based counting of quadrature encoder signals.  The maximum count
    rate will depend on the speed of the PC, but is expected to exceed
    1KHz for even the slowest computers, and may reach 10KHz on fast
    ones.  It is a realtime component.

    It supports up to eight counters, with optional index pulses.
    The number of counters is set by the module parameter 'num_chan='
    when the component is insmod'ed.  Alternatively, use the
    names= specifier and a list of unique names separated by commas.
    The names= and num_chan= specifiers are mutually exclusive.

    The driver exports variables for each counters inputs and output.
    It also exports two functions.  "encoder.update-counters" must be
    called in a high speed thread, at least twice the maximum desired
    count rate.  "encoder.capture-position" can be called at a much
    slower rate, and updates the output variables.
*/

/** Copyright (C) 2003 John Kasunich
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
#include "rtapi_string.h"
#include "hal.h"		/* HAL public API decls */

/* module information */
MODULE_AUTHOR("John Kasunich");
MODULE_DESCRIPTION("Encoder Counter for EMC HAL");
MODULE_LICENSE("GPL");

static int num_chan;
static int default_num_chan=3;
static int howmany;
RTAPI_MP_INT(num_chan, "number of encoder channels");

#define MAX_CHAN 8
char *names[MAX_CHAN] = {0,};
RTAPI_MP_ARRAY_STRING(names, MAX_CHAN, "names of encoder");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* data that is atomically passed from fast function to slow one */

typedef struct {
    char count_detected;
    char index_detected;
    char latch_detected;
    rtapi_s32 raw_count;
    rtapi_u32 timestamp;
    rtapi_s32 index_count;
    rtapi_s32 latch_count;
} atomic;

/* this structure contains the runtime data for a single counter
   u:rw means update() reads and writes the
   c:w  means capture() writes the field
   c:s u:rc means capture() sets (to 1), update() reads and clears
*/

typedef struct {
    unsigned char state;	/* u:rw quad decode state machine state */
    unsigned char oldZ;		/* u:rw previous value of phase Z */
    unsigned char Zmask;	/* u:rc c:s mask for oldZ, from index-ena */
    hal_bit_t *x4_mode;		/* u:r enables x4 counting (default) */
    hal_bit_t *counter_mode;	/* u:r enables counter mode */
    hal_s32_t *missing_teeth;   /* u:r non-zero enables missing-teeth index */
    hal_s32_t dt;		/* u:w most recent tooth space */
    hal_s32_t limit_dt;         /* u:r c:w inter-count gap (nS) to define index */
    atomic buf[2];		/* u:w c:r double buffer for atomic data */
    volatile atomic *bp;	/* u:r c:w ptr to in-use buffer */
    hal_s32_t *raw_counts;	/* u:rw raw count value, in update() only */
    hal_bit_t *phaseA;		/* u:r quadrature input */
    hal_bit_t *phaseB;		/* u:r quadrature input */
    hal_bit_t *phaseZ;		/* u:r index pulse input */
    hal_bit_t *index_ena;	/* c:rw index enable input */
    hal_bit_t *reset;		/* c:r counter reset input */
    hal_bit_t *latch_in;        /* c:r counter latch input */
    hal_bit_t *latch_rising;    /* u:r latch on rising edge? */
    hal_bit_t *latch_falling;   /* u:r latch on falling edge? */
    rtapi_s32 raw_count;		/* c:rw captured raw_count */
    rtapi_u32 timestamp;		/* c:rw captured timestamp */
    rtapi_s32 index_count;		/* c:rw captured index count */
    rtapi_s32 latch_count;		/* c:rw captured index count */
    hal_s32_t *count;		/* c:w captured binary count value */
    hal_s32_t *count_latch;     /* c:w captured binary count value */
    hal_float_t *min_speed;     /* c:r minimum velocity to estimate nonzero */
    hal_float_t *pos;		/* c:w scaled position (floating point) */
    hal_float_t *pos_interp;	/* c:w scaled and interpolated position (float) */
    hal_float_t *pos_latch;     /* c:w scaled latched position (floating point) */
    hal_float_t *vel;		/* c:w scaled velocity (floating point) */
    hal_float_t *vel_rpm;   /* rps * 60 for convenience */
    hal_float_t *pos_scale;	/* c:r pin: scaling factor for pos */
    hal_bit_t old_latch;        /* value of latch on previous cycle */
    double old_scale;		/* c:rw stored scale value */
    double scale;		/* c:rw reciprocal value used for scaling */
    int counts_since_timeout;	/* c:rw used for velocity calcs */
    int gaps; 			/* gap counter (unlikely to ever be > 1) */
} counter_t;

static rtapi_u32 timebase;		/* master timestamp for all counters */

/* pointer to array of counter_t structs in shmem, 1 per counter */
static counter_t *counter_array;

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
static const unsigned char lut_x4[16] = {
    0x00, 0x44, 0x88, 0x0C, 0x80, 0x04, 0x08, 0x4C,
    0x40, 0x04, 0x08, 0x8C, 0x00, 0x84, 0x48, 0x0C
};

/* same thing, but counts only once per complete cycle */

static const unsigned char lut_x1[16] = {
    0x00, 0x44, 0x08, 0x0C, 0x80, 0x04, 0x08, 0x0C,
    0x00, 0x04, 0x08, 0x0C, 0x00, 0x04, 0x08, 0x0C
};

/* look-up table for a one-wire counter */

static const unsigned char lut_ctr[16] = {
   0x00, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* other globals */
static int comp_id;		/* component ID */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static int export_encoder(counter_t * addr,char * prefix);
static void update(void *arg, long period);
static void capture(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/


int rtapi_app_main(void)
{
    int n, retval, i;
    counter_t *cntr;

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
	    "ENCODER: ERROR: invalid number of channels: %d\n", howmany);
	return -1;
    }
    /* have good config info, connect to the HAL */
    comp_id = hal_init("encoder");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "ENCODER: ERROR: hal_init() failed\n");
	return -1;
    }
    /* allocate shared memory for counter data */
    counter_array = hal_malloc(howmany * sizeof(counter_t));
    if (counter_array == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "ENCODER: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -1;
    }
    /* init master timestamp counter */
    timebase = 0;
    /* export all the variables for each counter */
    i = 0; // for names= items
    for (n = 0; n < howmany; n++) {
	/* point to struct */
	cntr = &(counter_array[n]);
	/* export all vars */
        if(num_chan) {
            char buf[HAL_NAME_LEN + 1];
            rtapi_snprintf(buf, sizeof(buf), "encoder.%d", n);
	    retval = export_encoder(cntr,buf);
        } else {
	    retval = export_encoder(cntr,names[i++]);
        }
	if (retval != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"ENCODER: ERROR: counter %d var export failed\n", n);
	    hal_exit(comp_id);
	    return -1;
	}
	/* init counter */
	cntr->state = 0;
	cntr->oldZ = 0;
	cntr->Zmask = 0;
	cntr->gaps = 0;
	*(cntr->x4_mode) = 1;
	*(cntr->counter_mode) = 0;
	*(cntr->latch_rising) = 1;
	*(cntr->latch_falling) = 1;
	cntr->buf[0].count_detected = 0;
	cntr->buf[1].count_detected = 0;
	cntr->buf[0].index_detected = 0;
	cntr->buf[1].index_detected = 0;
	cntr->bp = &(cntr->buf[0]);
	*(cntr->raw_counts) = 0;
	cntr->raw_count = 0;
	cntr->timestamp = 0;
	cntr->index_count = 0;
	cntr->latch_count = 0;
	*(cntr->count) = 0;
	*(cntr->min_speed) = 1.0;
	*(cntr->pos) = 0.0;
	*(cntr->pos_latch) = 0.0;
	*(cntr->vel) = 0.0;
	*(cntr->vel_rpm) = 0.0;
	*(cntr->pos_scale) = 1.0;
	cntr->old_scale = 1.0;
	cntr->scale = 1.0;
	cntr->counts_since_timeout = 0;
    }
    /* export functions */
    retval = hal_export_funct("encoder.update-counters", update,
	counter_array, 0, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "ENCODER: ERROR: count funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }
    retval = hal_export_funct("encoder.capture-position", capture,
	counter_array, 1, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "ENCODER: ERROR: capture funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }
    rtapi_print_msg(RTAPI_MSG_INFO,
	"ENCODER: installed %d encoder counters\n", howmany);
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

static void update(void *arg, long period)
{
    counter_t *cntr;
    atomic *buf;
    int n;
    unsigned char state;
    int latch, old_latch, rising, falling;

    cntr = arg;
    for (n = 0; n < howmany; n++) {
	buf = (atomic *) cntr->bp;
	cntr->dt += period;
	/* get state machine current state */
	state = cntr->state;
	/* add input bits to state code */
	if (*(cntr->phaseA)) {
	    state |= SM_PHASE_A_MASK;
	}
	if (*(cntr->phaseB)) {
	    state |= SM_PHASE_B_MASK;
	}
	/* look up new state */
	if ( *(cntr->counter_mode) ) {
	    state = lut_ctr[state & (SM_LOOKUP_MASK & ~SM_PHASE_B_MASK)];
	} else if ( *(cntr->x4_mode) ) {
	    state = lut_x4[state & SM_LOOKUP_MASK];
	} else {
	    state = lut_x1[state & SM_LOOKUP_MASK];
	}
	/* should we count? */
	if (state & SM_CNT_UP_MASK) {
	    if (*(cntr->missing_teeth) && cntr->dt > cntr->limit_dt){
		cntr->gaps++;
	    }
	    (*cntr->raw_counts)++;
	    buf->raw_count = *(cntr->raw_counts);
	    buf->timestamp = timebase;
	    buf->count_detected = 1;
	    cntr->dt = 0;
	} else if (state & SM_CNT_DN_MASK) {
	    (*cntr->raw_counts)--;
	    buf->raw_count = *(cntr->raw_counts);
	    buf->timestamp = timebase;
	    buf->count_detected = 1;
	}
	/* save state machine state */
	cntr->state = state;
	/* get old phase Z state, make room for new bit value */
	state = cntr->oldZ << 1;
	/* add new value of phase Z */
	if (*(cntr->phaseZ) || cntr->gaps) {
	    state |= 1;
	}
	cntr->oldZ = state & 3;
	/* test for index enabled and rising edge on phase Z */
	if ((state & cntr->Zmask) == 1) {
	    /* capture counts, reset Zmask */
	    buf->index_count = *(cntr->raw_counts);
	    buf->index_detected = 1;
	    cntr->Zmask = 0;
	}
        /* test for latch enabled and desired edge on latch-in */
        latch = *(cntr->latch_in), old_latch = cntr->old_latch;
        rising = latch && !old_latch;
        falling = !latch && old_latch;

        if((rising && *(cntr->latch_rising))
                || (falling && *(cntr->latch_falling))) {
            buf->latch_detected = 1;
            buf->latch_count = *(cntr->raw_counts);
        }
        cntr->old_latch = latch;

	/* move on to next channel */
	cntr++;
    }
    /* increment main timestamp counter */
    timebase += period;
    /* done */
}


static void capture(void *arg, long period)
{
    (void)period;
    counter_t *cntr;
    atomic *buf;
    int n;
    rtapi_s32 delta_counts;
    rtapi_u32 delta_time;
    double vel, interp;

    cntr = arg;
    for (n = 0; n < howmany; n++) {
	/* point to active buffer */
	buf = (atomic *) cntr->bp;
	/* tell update() to use the other buffer */
	if ( buf == &(cntr->buf[0]) ) {
	    cntr->bp = &(cntr->buf[1]);
	} else {
	    cntr->bp = &(cntr->buf[0]);
	}
	/* handle index */
	if ( buf->index_detected ) {
	    buf->index_detected = 0;
	    cntr->index_count = buf->index_count;
	    *(cntr->index_ena) = 0;
	}
        /* handle latch */
	if ( buf->latch_detected ) {
	    buf->latch_detected = 0;
	    cntr->latch_count = buf->latch_count;
	}

	/* update Zmask based on index_ena */
	if (*(cntr->index_ena)) {
	    cntr->Zmask = 3;
	} else {
	    cntr->Zmask = 0;
	}
	/* done interacting with update() */
	/* check for change in scale value */
	if ( *(cntr->pos_scale) != cntr->old_scale ) {
	    /* save new scale to detect future changes */
	    cntr->old_scale = *(cntr->pos_scale);
	    /* scale value has changed, test and update it */
	    if ((*(cntr->pos_scale) < 1e-20) && (*(cntr->pos_scale) > -1e-20)) {
		/* value too small, divide by zero is a bad thing */
		*(cntr->pos_scale) = 1.0;
	    }
	    /* we actually want the reciprocal */
	    cntr->scale = 1.0 / *(cntr->pos_scale);
	}
        /* check for valid min_speed */
        if ( *(cntr->min_speed) == 0 ) {
            *(cntr->min_speed) = 1;
        }

	/* check reset input */
	if (*(cntr->reset)) {
	    /* reset is active, reset the counter */
	    /* note: we NEVER reset raw_counts, that is always a
		running count of edges seen since startup.  The
		public "count" is the difference between raw_count
		and index_count, so it will become zero. */
	    cntr->raw_count = *(cntr->raw_counts);
	    cntr->index_count = cntr->raw_count;
	}
	/* process data from update() */
	if ( buf->count_detected ) {
	    /* one or more counts in the last period */
	    buf->count_detected = 0;
	    delta_time = buf->timestamp - cntr->timestamp;
	    delta_counts = buf->raw_count - cntr->raw_count;
	    // lowpass the gap detector deliberately ignoring missing teeth
	    // see https://github.com/LinuxCNC/linuxcnc/issues/2635
	    if (delta_counts != 0){
		cntr->limit_dt *= 0.9;
		cntr->limit_dt += 0.1 * ((*(cntr->missing_teeth) + 0.5) * (delta_time / delta_counts));
	    }
	    // correct counts for tooth gap
	    *cntr->raw_counts += *(cntr->missing_teeth) * cntr->gaps;
	    cntr->raw_count = buf->raw_count + *(cntr->missing_teeth) * cntr->gaps;
	    delta_counts += *(cntr->missing_teeth) * cntr->gaps;
	    cntr->gaps = 0;
	    cntr->timestamp = buf->timestamp;
	    if ( cntr->counts_since_timeout < 2 ) {
		cntr->counts_since_timeout++;
	    } else {
		vel = (delta_counts * cntr->scale ) / (delta_time * 1e-9);
		*(cntr->vel) = vel;
	    }
	} else {
	    /* no count */
	    if ( cntr->counts_since_timeout ) {
		/* calc time since last count */
		delta_time = timebase - cntr->timestamp;
		if (*(cntr->missing_teeth) && delta_time > 1.5 * cntr->limit_dt){
			// dont update the velocity in the tooth gap
		} else if ( delta_time < 1e9 / ( *(cntr->min_speed) * cntr->scale )) {
		    /* not to long, estimate vel if a count arrived now */
		    vel = ( cntr->scale ) / (delta_time * 1e-9);
		    /* make vel positive, even if scale is negative */
		    if ( vel < 0.0 ) vel = -vel;
		    /* use lesser of estimate and previous value */
		    /* use sign of previous value, magnitude of estimate */
		    if ( vel < *(cntr->vel) ) {
			*(cntr->vel) = vel;
		    }
		    if ( -vel > *(cntr->vel) ) {
			*(cntr->vel) = -vel;
		    }
		} else {
		    /* its been a long time, stop estimating */
		    cntr->counts_since_timeout = 0;
		    *(cntr->vel) = 0;
		}
	    } else {
		/* we already stopped estimating */
		*(cntr->vel) = 0;
	    }
	}
	*(cntr->vel_rpm) = *(cntr->vel) * 60.0;
	/* compute net counts */
	*(cntr->count) = cntr->raw_count - cntr->index_count;
        *(cntr->count_latch) = cntr->latch_count - cntr->index_count;
	/* scale count to make floating point position */
	*(cntr->pos) = *(cntr->count) * cntr->scale;
	*(cntr->pos_latch) = *(cntr->count_latch) * cntr->scale;
	/* add interpolation value */
	delta_time = timebase - cntr->timestamp;
	interp = *(cntr->vel) * (delta_time * 1e-9);
	*(cntr->pos_interp) = *(cntr->pos) + interp;
	/* move on to next channel */
	cntr++;
    }
    /* done */
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static int export_encoder(counter_t * addr,char * prefix)
{
    int retval, msg;

    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);

    /* export pins for the quadrature inputs */
    retval = hal_pin_bit_newf(HAL_IN, &(addr->phaseA), comp_id,
            "%s.phase-A", prefix);
    if (retval != 0) {
	return retval;
    }
    retval = hal_pin_bit_newf(HAL_IN, &(addr->phaseB), comp_id,
            "%s.phase-B", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for the index input */
    retval = hal_pin_bit_newf(HAL_IN, &(addr->phaseZ), comp_id,
            "%s.phase-Z", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for the index enable input */
    retval = hal_pin_bit_newf(HAL_IO, &(addr->index_ena), comp_id,
            "%s.index-enable", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for the reset input */
    retval = hal_pin_bit_newf(HAL_IN, &(addr->reset), comp_id,
            "%s.reset", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pins for position latching */
    retval = hal_pin_bit_newf(HAL_IN, &(addr->latch_in), comp_id,
            "%s.latch-input", prefix);
    if (retval != 0) {
	return retval;
    }
    retval = hal_pin_bit_newf(HAL_IN, &(addr->latch_rising), comp_id,
            "%s.latch-rising", prefix);
    if (retval != 0) {
	return retval;
    }
    retval = hal_pin_bit_newf(HAL_IN, &(addr->latch_falling), comp_id,
            "%s.latch-falling", prefix);
    if (retval != 0) {
	return retval;
    }

    /* export parameter for raw counts */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->raw_counts), comp_id,
            "%s.rawcounts", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for counts captured by capture() */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->count), comp_id,
            "%s.counts", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for counts latched by capture() */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->count_latch), comp_id,
            "%s.counts-latched", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for minimum speed estimated by capture() */
    retval = hal_pin_float_newf(HAL_IN, &(addr->min_speed), comp_id,
            "%s.min-speed-estimate", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for scaled position captured by capture() */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->pos), comp_id,
            "%s.position", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for scaled and interpolated position captured by capture() */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->pos_interp), comp_id,
            "%s.position-interpolated", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for latched position captured by capture() */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->pos_latch), comp_id,
            "%s.position-latched", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for scaled velocity captured by capture() */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->vel), comp_id,
            "%s.velocity", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for scaled velocity captured by capture() */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->vel_rpm), comp_id,
            "%s.velocity-rpm", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for scaling */
    retval = hal_pin_float_newf(HAL_IO, &(addr->pos_scale), comp_id,
            "%s.position-scale", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for x4 mode */
    retval = hal_pin_bit_newf(HAL_IO, &(addr->x4_mode), comp_id,
            "%s.x4-mode", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for counter mode */
    retval = hal_pin_bit_newf(HAL_IO, &(addr->counter_mode), comp_id,
            "%s.counter-mode", prefix);
    if (retval != 0) {
	return retval;
    }
    /* export pin for missing-tooth index */
    retval = hal_pin_s32_newf(HAL_IN, &(addr->missing_teeth), comp_id,
            "%s.missing-teeth", prefix);
    if (retval != 0) {
	return retval;
    }

/* uncomment to debug missing tooth
    retval = hal_param_s32_newf(HAL_RO, &(addr->dt), comp_id,
            "%s.dt", prefix);
    if (retval != 0) {
	return retval;
    }

    retval = hal_param_s32_newf(HAL_RO, &(addr->limit_dt), comp_id,
            "%s.limit_dt", prefix);
    if (retval != 0) {
	return retval;
    }
*/

    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
}
