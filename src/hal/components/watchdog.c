/********************************************************************
* Description: watchdog.c
*   Watchdog for HAL
*
*   See the "Users Manual" at emc2/docs/Hal_Introduction.pdf
*
*
* This module provides a single module that can monitor heartbeats from
* several other modules (like a charge pump signal), to make sure that
* no monitored process or HAL thread dies or takes too long between
* execution cycles
*
*********************************************************************
*
* Author: Stephen Wille Padnos (swpadnos AT sourceforge DOT net)
* License: GPL Version 2
* Created on: Jue 19, 2010
* System: Linux
*
* Copyright (c) 2010 All rights reserved.
*
********************************************************************/

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"		/* HAL public API decls */

/* module information */
MODULE_AUTHOR("Stephen Wille Padnos");
MODULE_DESCRIPTION("Multiple input watchdog for EMC HAL");
MODULE_LICENSE("GPL");
int num_inputs=-1;			// must specify a count on the loadrt line
RTAPI_MP_INT(num_inputs, "Number of inputs");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* Data needed for each input */
typedef struct {
    hal_bit_t *input;		/* pin: the input bit HAL pin */
    hal_float_t timeout;	/* param: maximum alloewd timeb without a transition on bit */
    hal_float_t oldtimeout;	/* internal:  used to determine whether the timeout has changed */
    hal_s32_t c_secs, c_nsecs;	/* internal:  elapsed seconds and nanoseconds */
    hal_s32_t t_secs, t_nsecs;	/* internal:  seconds and nanoseconds for timeout */
    hal_bit_t last;		/* internal:  last value of the input pin */
} watchdog_input_t;

#define MAX_INPUTS	32

/* Base data for a weighted summer. */
typedef struct {
  hal_bit_t *output;		/* output pin: high if all inputs are toggling, low otherwise */
  hal_bit_t *enable;		/* pin: only runs while this is high (kind of like an enable) */
} watchdog_data_t;

/* other globals */
static int comp_id;		/* component ID */
watchdog_input_t *inputs;	/* internal: pointer to the input bits and weights */
watchdog_data_t *data;		/* common data */
hal_bit_t old_enable;
/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static void process(void *arg, long period);
static void set_timeouts(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int n, retval;


    /* check that there's at least one valid input requested */
    if (num_inputs<1) {
      rtapi_print_msg(RTAPI_MSG_ERR, "WATCHDOG: ERROR: must specify at least one input\n");
      return -1;
    }

    /* but not too many */
    if (num_inputs> MAX_INPUTS) {
      rtapi_print_msg(RTAPI_MSG_ERR, "WATCHDOG: ERROR: too many inputs requested (%d > %d)\n", num_inputs, MAX_INPUTS);
      return -1;
    }

    /* have good config info, connect to the HAL */
    comp_id = hal_init("watchdog");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WATCHDOG: ERROR: hal_init() failed (Return code %d)\n", comp_id);
	return -1;
    }

    /* allocate shared memory for watchdog global and pin info */
    data = hal_malloc(sizeof(watchdog_data_t));
    if (data == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WATCHDOG: ERROR: hal_malloc() for common data failed\n");
	hal_exit(comp_id);
	goto err;
    }

    inputs = hal_malloc(num_inputs * sizeof(watchdog_input_t));
    if (inputs == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WATCHDOG: ERROR: hal_malloc() for input pins failed\n");
	hal_exit(comp_id);
	goto err;
    }

    /* export pins/params for all inputs */
    for (n = 0; n < num_inputs; n++) {
      retval=hal_pin_bit_newf(HAL_IN, &(inputs[n].input), comp_id, "watchdog.input-%d", n);
      if (retval != 0) {
	  rtapi_print_msg(RTAPI_MSG_ERR,
	      "WATCHDOG: ERROR: couldn't create input pin watchdog.input-%d\n", n);
	  goto err;
      }
      retval=hal_param_float_newf(HAL_RW, &(inputs[n].timeout), comp_id, "watchdog.timeout-%d", n);
      if (retval != 0) {
	  rtapi_print_msg(RTAPI_MSG_ERR,
	      "WATCHDOG: ERROR: couldn't create input parameter watchdog.timeout-%d\n", n);
	  goto err;
      }
      
      inputs[n].timeout=0;
      inputs[n].oldtimeout=-1;
      inputs[n].c_secs = inputs[n].t_secs = 0;
      inputs[n].c_nsecs = inputs[n].t_nsecs = 0;
      inputs[n].last = *(inputs[n].input);
    }

    /* export "global" pins */
    retval=hal_pin_bit_newf(HAL_OUT, &(data->output), comp_id, "watchdog.ok-out");
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WATCHDOG: ERROR: couldn't create output pin watchdog.ok-out\n");
	goto err;
    }
    retval=hal_pin_bit_newf(HAL_IN, &(data->enable), comp_id, "watchdog.enable-in");
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WATCHDOG: ERROR: couldn't create input pin watchdog.enable-in\n");
	goto err;
    }
    
    /* export functions */
    retval = hal_export_funct("watchdog.process", process, inputs, 0, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WATCHDOG: ERROR: process funct export failed\n");
	goto err;
    }

    retval = hal_export_funct("watchdog.set-timeouts", set_timeouts, inputs, 1, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WATCHDOG: ERROR: set_timeouts funct export failed\n");
	goto err;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
	"WATCHDOG: installed watchdog with %d inputs\n", num_inputs);
    hal_ready(comp_id);
    return 0;
    
err:
    hal_exit(comp_id);
    return -1;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}

/***********************************************************************
*                     REALTIME FUNCTIONS                               *
************************************************************************/

/*  This procedude checks all the input bits for changes.  If a change
    is detected, the corresponding timer is reset.  If no change was
    detected, "period" is subtracted from the timer.  If any timer gets
    to zero, the output is cleared and enable must go low and high again
    to re-start the watchdog.
*/
static void process(void *arg, long period)
{
    (void)arg;
    int i, fault=0;
    // set_timeouts has to turn on the output when it detects a valid
    // transition on enable
    if (!(*data->enable) || (!(*data->output))) return;
    for (i=0;i<num_inputs;i++) {
      if (*(inputs[i].input) != inputs[i].last) {
	inputs[i].c_secs = inputs[i].t_secs;
	inputs[i].c_nsecs = inputs[i].t_nsecs;
      } else {
	inputs[i].c_nsecs -= period;
	if (inputs[i].c_nsecs<0) {
	  inputs[i].c_nsecs += 1000000000;
	  if (inputs[i].c_secs>0) {
	    inputs[i].c_secs--;
	  } else {
	    fault=1;
	    inputs[i].c_secs = inputs[i].c_nsecs = 0;
	  }
	}
      }
      inputs[i].last=*(inputs[i].input);
    }
    if (fault) *(data->output)=0;
}

static void set_timeouts(void *arg, long period)
{
    (void)arg;
    (void)period;
    int i;
    hal_float_t temp;
    
    for (i=0;i<num_inputs;i++) {
      temp=inputs[i].timeout;
      if (temp<0) temp=0;	// no negative timeout periods
      if (temp != inputs[i].oldtimeout) {
	// new timeout, convert to secs/ns
	inputs[i].oldtimeout=temp;
	inputs[i].t_secs=temp;
	temp -= inputs[i].t_secs;
	inputs[i].t_nsecs=(1e9*temp);
      }
    }
    if (!*(data->output)) {
	if (*(data->enable) && !old_enable) {
	  // rising edge on enable, so we can restart
	  for (i=0;i<num_inputs;i++) {
	    inputs[i].c_secs = inputs[i].t_secs;
	    inputs[i].c_nsecs = inputs[i].t_nsecs;
	  }
	  *(data->output) = 1;
	}
    }
    old_enable=*(data->enable);
}
/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/
