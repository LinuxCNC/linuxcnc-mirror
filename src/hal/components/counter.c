/********************************************************************
* Description:  counter.c
*               This file, 'counter.c', is a HAL component that 
*               provides software-based counting of pulse streams
*               with an optional reset input.
*
* Author: Chris Radek <chris@timeguy.com>
* License: GPL Version 2
*    
* Copyright (c) 2006 All rights reserved.
*
********************************************************************/
/** This file, 'counter.c', is a HAL component that provides software-
    based counting that is useful for spindle position sensing and
    maybe other things.  Instead of using a real encoder that outputs
    quadrature, some lathes have a sensor that generates a simple pulse
    stream as the spindle turns and an index pulse once per revolution.
    This component simply counts up when a "count" pulse (phase-A)
    is received, and if reset is enabled, resets when the "index"
    (phase-Z) pulse is received.

    This is of course only useful for a unidirectional spindle, as it
    is not possible to sense the direction of rotation.

    The maximum count rate will depend on the speed of the PC, but is
    expected to exceed 2kHz for even the slowest computers, and may
    well be over 25kHz on fast ones.  It is a realtime component.

    It supports up to eight counters, with optional index pulses.
    The number of counters is set by the module parameter 'num_chan'
    when the component is insmod'ed.

    The driver exports variables for each counter's inputs and outputs.
    It also exports two functions:  "counter.update-counters" must be
    called in a high speed thread, at least twice the maximum desired
    count rate.  "counter.capture-position" can be called at a much 
    slower rate, and updates the output variables.
*/

/** Copyright (C) 2006 Chris Radek <chris@timeguy.com>
 *
 *  Based heavily on the "encoder" hal module by John Kasunich
 */

#include "rtapi.h"              /* RTAPI realtime OS API */
#include "rtapi_app.h"          /* RTAPI realtime module decls */
#include "rtapi_errno.h"        /* EINVAL etc */
#include "hal.h"                /* HAL public API decls */

/* module information */
MODULE_AUTHOR("Chris Radek");
MODULE_DESCRIPTION("Pulse Counter for EMC HAL");
MODULE_LICENSE("GPL");
static int num_chan = 1;        /* number of channels - default = 1 */
RTAPI_MP_INT(num_chan, "number of channels");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* this structure contains the runtime data for a single counter */

typedef struct {
    unsigned char oldZ;		/* previous value of phase Z */
    unsigned char oldA;		/* previous value of phase A */
    unsigned char reset_on_index;
    unsigned char pad;		/* padding for alignment */
    hal_s32_t *raw_count;	/* pin: raw binary count value */
    hal_bit_t *phaseA;		/* quadrature input */
    hal_bit_t *phaseZ;		/* index pulse input */
    hal_bit_t *index_ena;	/* index enable input */
    hal_bit_t *reset;		/* counter reset input */
    hal_s32_t *count;		/* captured binary count value */
    hal_float_t *pos;		/* scaled position (floating point) */
    hal_float_t *vel;		/* scaled velocity (floating point) */
    hal_float_t *pos_scale;	/* pin: scaling factor for pos */
    double old_scale;		/* stored scale value */
    double scale;		/* reciprocal value used for scaling */
    hal_s32_t last_count;
    hal_s32_t last_index_count;
} counter_t;

/* pointer to array of counter_t structs in shmem, 1 per counter */
static counter_t *counter_array;

/* other globals */
static int comp_id;		/* component ID */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static int export_counter(int num, counter_t * addr);
static void update(void *arg, long period);
static void capture(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

#define MAX_CHAN 8

int rtapi_app_main(void)
{
    int n, retval;

    /* test for number of channels */
    if ((num_chan <= 0) || (num_chan > MAX_CHAN)) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "COUNTER: ERROR: invalid num_chan: %d\n", num_chan);
	return -EINVAL;
    }
    /* have good config info, connect to the HAL */
    comp_id = hal_init("counter");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "COUNTER: ERROR: hal_init() failed\n");
	return -EINVAL;
    }
    /* allocate shared memory for counter data */
    counter_array = hal_malloc(num_chan * sizeof(counter_t));
    if (!counter_array) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "COUNTER: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -ENOMEM;
    }
    /* export all the variables for each counter */
    for (n = 0; n < num_chan; n++) {
	/* export all vars */
	retval = export_counter(n, &(counter_array[n]));
	if (retval != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"COUNTER: ERROR: counter %d var export failed\n", n);
	    hal_exit(comp_id);
	    return -EIO;
	}
	/* init counter */
	counter_array[n].oldZ = 0;
	counter_array[n].oldA = 0;
	counter_array[n].reset_on_index = 0;
	*(counter_array[n].raw_count) = 0;
	*(counter_array[n].count) = 0;
	*(counter_array[n].pos) = 0.0;
	*(counter_array[n].pos_scale) = 1.0;
	counter_array[n].old_scale = 1.0;
	counter_array[n].scale = 1.0;
    }
    /* export functions */
    retval = hal_export_funct("counter.update-counters", update,
	counter_array, 0, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "COUNTER: ERROR: count funct export failed\n");
	hal_exit(comp_id);
	return -EIO;
    }
    retval = hal_export_funct("counter.capture-position", capture,
	counter_array, 1, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "COUNTER: ERROR: capture funct export failed\n");
	hal_exit(comp_id);
	return -EIO;
    }
    rtapi_print_msg(RTAPI_MSG_INFO,
	"COUNTER: installed %d counter counters\n", num_chan);
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}

/***********************************************************************
*            REALTIME COUNTER COUNTING AND UPDATE FUNCTIONS            *
************************************************************************/

static void update(void *arg, long period)
{
    (void)period;
    counter_t *cntr;
    int n;

    for (cntr = arg, n = 0; n < num_chan; cntr++, n++) {
        // count on rising edge
        if(!cntr->oldA && *cntr->phaseA)
            (*cntr->raw_count)++;
        cntr->oldA = *cntr->phaseA;

        // reset on rising edge
        if(cntr->reset_on_index && !cntr->oldZ && *cntr->phaseZ) {
            cntr->last_index_count = *(cntr->raw_count);
            *(cntr->index_ena) = 0;
        }
        cntr->oldZ = *cntr->phaseZ;
    }
}

static void capture(void *arg, long period)
{
    counter_t *cntr;
    int n;

    for (cntr = arg, n = 0; n < num_chan; cntr++, n++) {
	/* check reset input */
        int raw_count;
        int counts;
	if (*(cntr->reset)) {
	    /* reset is active, reset the counter */
	    *(cntr->raw_count) = 0;
            cntr->last_index_count = 0;
            cntr->last_count = 0;
	}
	/* capture raw counts to latches */
        raw_count = *(cntr->raw_count);
	*(cntr->count) = raw_count - cntr->last_index_count;
        counts = (raw_count - cntr->last_count);
        cntr->last_count = raw_count;

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
	/* scale count to make floating point position */
	*(cntr->pos) = *(cntr->count) * cntr->scale;
	/* scale counts to make floating point velocity */
        *(cntr->vel) = counts * cntr->scale * 1e9 / period;

	/* update reset_on_index based on index_ena */
        cntr->reset_on_index = *(cntr->index_ena);
    }
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static int export_counter(int num, counter_t * addr)
{
    int retval, msg;

    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    msg = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);

    /* export pins for the quadrature inputs */
    retval = hal_pin_bit_newf(HAL_IN, &(addr->phaseA), comp_id, "counter.%d.phase-A", num);
    if (retval != 0) {
	return retval;
    }
    /* export pin for the index input */
    retval = hal_pin_bit_newf(HAL_IN, &(addr->phaseZ), comp_id, "counter.%d.phase-Z", num);
    if (retval != 0) {
	return retval;
    }
    /* export pin for the index enable input */
    retval = hal_pin_bit_newf(HAL_IO, &(addr->index_ena), comp_id, "counter.%d.index-enable", num);
    if (retval != 0) {
	return retval;
    }
    /* export pin for the reset input */
    retval = hal_pin_bit_newf(HAL_IN, &(addr->reset), comp_id, "counter.%d.reset", num);
    if (retval != 0) {
	return retval;
    }
    /* export parameter for raw counts */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->raw_count), comp_id, "counter.%d.rawcounts", num);
    if (retval != 0) {
	return retval;
    }
    /* export pin for counts captured by capture() */
    retval = hal_pin_s32_newf(HAL_OUT, &(addr->count), comp_id, "counter.%d.counts", num);
    if (retval != 0) {
	return retval;
    }
    /* export pin for scaled position captured by capture() */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->pos), comp_id, "counter.%d.position", num);
    if (retval != 0) {
	return retval;
    }
    /* export pin for scaled velocity captured by capture() */
    retval = hal_pin_float_newf(HAL_OUT, &(addr->vel), comp_id, "counter.%d.velocity", num);
    if (retval != 0) {
	return retval;
    }
    /* export parameter for scaling */
    retval = hal_pin_float_newf(HAL_IO, &(addr->pos_scale), comp_id, "counter.%d.position-scale", num);
    if (retval != 0) {
	return retval;
    }
    /* restore saved message level */
    rtapi_set_msg_level(msg);
    return 0;
}
