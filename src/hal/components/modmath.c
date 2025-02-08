/********************************************************************
* Description: modmath.c
*   Assorted modulo arithmetic functions for HAL.
*
*   This HAL component has just one function at the moment: closest_dir
*   this component takes a "actual" input and a "desired" input, and
*   has one parameter: the "base" (or, number of elements).  It outputs
*   an up or down bit, depending on which direction has the shortest path
*   from the actual to the desired input.
*
*   Each individual type of block is invoked by a parameter on
*   the insmdo command line.  Each parameter is of the form:
*   <blockname>=<number_of_blocks>
*
*   For example, mod_dir=2 installs two "closest direction" components.
*
*   List of functions currently implemented:
*      mod_dir: tells the direction to the closest neighbor, for sequences that roll over
*
*   Eventually, there should be more functions
*
*********************************************************************
*
* Author: Stephen Wille Padnos  (swpadnos AT sourceforge DOT net)
*       Based on a work by John Kasunich (jmkasunich AT att DOT net)
* License: GPL Version 2
* Created on: 2006/5/18
* System: Linux
*
* Copyright (c) 2006 All rights reserved.
*
********************************************************************/

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"		/* HAL public API decls */

#include "rtapi_math.h"

/* module information */
MODULE_AUTHOR("Stephen Wille Padnos");
MODULE_DESCRIPTION("Modulo math blocks for EMC HAL");
MODULE_LICENSE("GPL");
static int mod_dir = 0;	/* number of mod_dirs */
RTAPI_MP_INT(mod_dir, "Modulo direction blocks");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/** These structures contain the runtime data for a single block. */

typedef struct {
    hal_bit_t *up;		/* output pin: go up to get to the desired position */
    hal_bit_t *down;		/* output pin: go down to get to the desired position */
    hal_bit_t *on_target;	/* output pin: go at desired position */
    hal_s32_t *actual;		/* input pin: actual position */
    hal_s32_t *desired;		/* input pin: desired position */
    hal_s32_t *max_num;		/* input/output pin: highest value to allow */
    hal_s32_t *min_num;		/* input/output pin: lowest value to allow */
    hal_bit_t *wrap;		/* input/output pin: set true if the array is circular, false if linear */
} mod_dir_t;

/* other globals */
static int comp_id;		/* component ID */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static int export_mod_dir(int num);

static void mod_dir_funct(void *arg, long period);


/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int n;

    /* connect to the HAL */
    comp_id = hal_init("modmath");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "MODMATH: ERROR: hal_init() failed\n");
	return -1;
    }
    /* allocate and export modulo direction finders */
    if (mod_dir > 0) {
	for (n = 0; n < mod_dir; n++) {
	    if (export_mod_dir(n) != 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		    "MODMATH: ERROR: export_mod_dir(%d) failed\n", n);
		hal_exit(comp_id);
		return -1;
	    }
	}
	rtapi_print_msg(RTAPI_MSG_INFO,
	    "MODMATH: installed %d mod-dirs\n", mod_dir);
    }
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}

/***********************************************************************
*                     REALTIME BLOCK FUNCTIONS                         *
************************************************************************/

static void mod_dir_funct(void *arg, long period)
{
    (void)period;
    mod_dir_t *mod;
    int range, act, des, to_go;

    /* point to block data */
    mod = (mod_dir_t *) arg;
    range = *(mod->max_num) - *(mod->min_num) + 1;
    act = *(mod->actual);
    if (act > *(mod->max_num) || act < *(mod->min_num)) {
	act = *(mod->min_num) + ((act-*(mod->min_num)) % (range));
    }
    des = *(mod->desired);
    if (des > *(mod->max_num) || des < *(mod->min_num)) {
	des = *(mod->min_num) + ((des-*(mod->min_num)) % (range));
    }

    to_go = des-act;

    if ((*(mod->wrap)) && (to_go > range/2)) {
	to_go -= range;
    }
    if ((*(mod->wrap)) && (to_go < -range/2)) {
	to_go += range;
    }

    /* if (desired-actual) >= (actual+(max-min+1)-desired), output "up" */
    if (to_go == 0) {
	*(mod->up) = 0;
	*(mod->down) = 0;
	*(mod->on_target) = 1;
    } else if (to_go > 0 ) {
	*(mod->down) = 0;
	*(mod->on_target) = 0;
	*(mod->up) = 1;
    } else {
	*(mod->up) = 0;
	*(mod->on_target) = 0;
	*(mod->down) = 1;
    }
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static int export_mod_dir(int num)
{
    int retval;
    char buf[HAL_NAME_LEN + 1];
    mod_dir_t *moddir;

    /* allocate shared memory for modulo "closest direction finder" */
    moddir = hal_malloc(sizeof(mod_dir_t));
    if (moddir == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MODMATH: ERROR: hal_malloc() failed\n");
	return -1;
    }
    /* export output pins */
    rtapi_snprintf(buf, sizeof(buf), "mod-dir.%d.up", num);
    retval = hal_pin_bit_new(buf, HAL_OUT, &(moddir->up), comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MODMATH: ERROR: '%s' pin export failed\n", buf);
	return retval;
    }
    rtapi_snprintf(buf, sizeof(buf), "mod-dir.%d.down", num);
    retval = hal_pin_bit_new(buf, HAL_OUT, &(moddir->down), comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MODMATH: ERROR: '%s' pin export failed\n", buf);
	return retval;
    }
    rtapi_snprintf(buf, sizeof(buf), "mod-dir.%d.on-target", num);
    retval = hal_pin_bit_new(buf, HAL_OUT, &(moddir->on_target), comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MODMATH: ERROR: '%s' pin export failed\n", buf);
	return retval;
    }

    /* export input pins */
    rtapi_snprintf(buf, sizeof(buf), "mod-dir.%d.actual", num);
    retval = hal_pin_s32_new(buf, HAL_IN, &(moddir->actual), comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MODMATH: ERROR: '%s' pin export failed\n", buf);
	return retval;
    }
    rtapi_snprintf(buf, sizeof(buf), "mod-dir.%d.desired", num);
    retval = hal_pin_s32_new(buf, HAL_IN, &(moddir->desired), comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MODMATH: ERROR: '%s' pin export failed\n", buf);
	return retval;
    }

    /* export pins for max and min values */
    rtapi_snprintf(buf, sizeof(buf), "mod-dir.%d.min-num", num);
    retval = hal_pin_s32_new(buf, HAL_IO, &(moddir->min_num), comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MODMATH: ERROR: '%s' pin export failed\n", buf);
	return retval;
    }
    rtapi_snprintf(buf, sizeof(buf), "mod-dir.%d.max-num", num);
    retval = hal_pin_s32_new(buf, HAL_IO, &(moddir->max_num), comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MODMATH: ERROR: '%s' pin export failed\n", buf);
	return retval;
    }
    rtapi_snprintf(buf, sizeof(buf), "mod-dir.%d.wrap", num);
    retval = hal_pin_bit_new(buf, HAL_IO, &(moddir->wrap), comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MODMATH: ERROR: '%s' param export failed\n", buf);
	return retval;
    }
    /* export function */
    rtapi_snprintf(buf, sizeof(buf), "mod-dir.%d", num);
    retval = hal_export_funct(buf, mod_dir_funct, moddir, 1, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MODMATH: ERROR: '%s' funct export failed\n", buf);
	return -1;
    }
    /* set default parameter values */
    *(moddir->up) = 0;
    *(moddir->down) = 0;
    *(moddir->on_target) = 1;
    *(moddir->min_num) = 0;
    *(moddir->max_num) = 15;
    *(moddir->actual) = 0;
    *(moddir->desired) = 0;
    *(moddir->wrap) = 1;		/* wrap by default */
    return 0;
}
