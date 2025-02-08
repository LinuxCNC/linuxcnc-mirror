/********************************************************************
* Description:  sampler.c
*               A HAL component that can be used to capture data
*               from HAL pins at a specific realtime sample rate,
*		and allows the data to be written to stdout.
*
* Author: John Kasunich <jmkasunich at sourceforge dot net>
* License: GPL Version 2
*    
* Copyright (c) 2006 All rights reserved.
*
********************************************************************/
/** This file, 'sampler.c', is the realtime part of a HAL component
    that allows data from HAL pins to be sampled at a uniform realtime
    sample rate and then be transferred to a file.  When this realtime
    module is loaded, it creates a fifo in shared memory and begins 
    capturing data from HAL pins to the fifo.  Then, the user space 
    program 'halsampler' is invoked, which reads the fifo and writes
    the data to stdout.

    Loading:

    loadrt sampler depth=100 cfg=uffb


*/

/** Copyright (C) 2006 John Kasunich
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

#include "rtapi.h"              /* RTAPI realtime OS API */
#include "rtapi_app.h"          /* RTAPI realtime module decls */
#include "hal.h"                /* HAL public API decls */
#include "streamer.h"		/* decls and such for fifos */
#include "rtapi_errno.h"
#include "rtapi_string.h"

/* module information */
MODULE_AUTHOR("John Kasunich");
MODULE_DESCRIPTION("Realtime HAL Sampler");
MODULE_LICENSE("GPL");
static char *cfg[MAX_SAMPLERS];	/* config string, no default */
RTAPI_MP_ARRAY_STRING(cfg,MAX_SAMPLERS,"config string");
static int depth[MAX_SAMPLERS];	/* depth of fifo, default 0 */
RTAPI_MP_ARRAY_INT(depth,MAX_SAMPLERS,"fifo depth");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* this structure contains the HAL shared memory data for one sampler */

typedef struct {
    hal_stream_t fifo;		/* pointer to user/RT fifo */
    hal_s32_t *curr_depth;	/* pin: current fifo depth */
    hal_bit_t *full;		/* pin: overrun flag */
    hal_bit_t *enable;		/* pin: enable sampling */
    hal_s32_t *overruns;	/* pin: number of overruns */
    hal_s32_t *sample_num;	/* pin: sample ID / timestamp */
    int num_pins;
    pin_data_t pins[HAL_STREAM_MAX_PINS];
} sampler_t;

/* other globals */
static int comp_id;		/* component ID */
static int nsamplers;
static sampler_t *samplers;

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static int init_sampler(int num, sampler_t *tmp_fifo);
static void sample(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int n, retval;

    comp_id = hal_init("sampler");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "SAMPLER: ERROR: hal_init() failed\n");
	return -EINVAL;
    }

    samplers = hal_malloc(MAX_SAMPLERS * sizeof(sampler_t));
    /* validate config info */
    for ( n = 0 ; n < MAX_SAMPLERS ; n++ ) {
	if (( cfg[n] == NULL ) || ( *(cfg[n]) == '\0' ) || ( depth[n] <= 0 )) {
	    break;
	}
	retval = hal_stream_create(&samplers[n].fifo, comp_id, SAMPLER_SHMEM_KEY+n, depth[n], cfg[n]);
	if(retval < 0) {
	    goto fail;
	}
	nsamplers++;
	retval = init_sampler(n, &samplers[n]);
    }
    if ( n == 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SAMPLER: ERROR: no channels specified\n");
	return -EINVAL;
    }

    hal_ready(comp_id);
    return 0;
fail:
    for(n=0; n<nsamplers; n++) hal_stream_detach(&samplers[n].fifo);
    hal_exit(comp_id);
    return retval;
}

void rtapi_app_exit(void)
{
    int i;
    for(i=0; i<nsamplers; i++) hal_stream_detach(&samplers[i].fifo);
    hal_exit(comp_id);
}

/***********************************************************************
*            REALTIME COUNTER COUNTING AND UPDATE FUNCTIONS            *
************************************************************************/

static void sample(void *arg, long period)
{
    (void)period;
    sampler_t *samp;
    pin_data_t *pptr;
    int n;

    /* point at sampler struct in HAL shmem */
    samp = arg;
    /* are we enabled? */
    if ( ! *(samp->enable) ) {
	*(samp->curr_depth) = hal_stream_depth(&samp->fifo);
	*(samp->full) = !hal_stream_writable(&samp->fifo);
	return;
    }
    /* point at pins in hal shmem */
    pptr = samp->pins;
    union hal_stream_data data[HAL_STREAM_MAX_PINS], *dptr=data;
    /* copy data from HAL pins to fifo */
    int num_pins = hal_stream_element_count(&samp->fifo);
    for ( n = 0 ; n < num_pins ; n++ ) {
	switch ( hal_stream_element_type(&samp->fifo, n) ) {
	case HAL_FLOAT:
	    dptr->f = *(pptr->hfloat);
	    break;
	case HAL_BIT:
	    if ( *(pptr->hbit) ) {
		dptr->b = 1;
	    } else {
		dptr->b = 0;
	    }
	    break;
	case HAL_U32:
	    dptr->u = *(pptr->hu32);
	    break;
	case HAL_S32:
	    dptr->s = *(pptr->hs32);
	    break;
	default:
	    break;
	}
	dptr++;
	pptr++;
    }
    if ( hal_stream_write(&samp->fifo, data) < 0) {
	/* fifo is full, data is lost */
        /* log the overrun */
	(*samp->overruns)++;
	*(samp->full) = 1;
	*(samp->curr_depth) = hal_stream_maxdepth(&samp->fifo);
    } else {
	*(samp->full) = 0;
	*(samp->curr_depth) = hal_stream_depth(&samp->fifo);
    }
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static int init_sampler(int num, sampler_t *str)
{
    int retval, usefp, n;
    pin_data_t *pptr;
    char buf[HAL_NAME_LEN + 1];

    /* export "standard" pins and params */
    retval = hal_pin_bit_newf(HAL_OUT, &(str->full), comp_id,
	"sampler.%d.full", num);
    if (retval != 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SAMPLER: ERROR: 'full' pin export failed\n");
	return -EIO;
    }
    retval = hal_pin_bit_newf(HAL_IN, &(str->enable), comp_id,
	"sampler.%d.enable", num);
    if (retval != 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SAMPLER: ERROR: 'enable' pin export failed\n");
	return -EIO;
    }
    retval = hal_pin_s32_newf(HAL_OUT, &(str->curr_depth), comp_id,
	"sampler.%d.curr-depth", num);
    if (retval != 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SAMPLEr: ERROR: 'curr_depth' pin export failed\n");
	return -EIO;
    }
    retval = hal_pin_s32_newf(HAL_IO, &(str->overruns), comp_id,
	"sampler.%d.overruns", num);
    if (retval != 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SAMPLER: ERROR: 'overruns' parameter export failed\n");
	return -EIO;
    }
    retval = hal_pin_s32_newf(HAL_IO, &(str->sample_num), comp_id,
	"sampler.%d.sample-num", num);
    if (retval != 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SAMPLER: ERROR: 'sample-num' parameter export failed\n");
	return -EIO;
    }
    /* init the standard pins and params */
    *(str->full) = 0;
    *(str->enable) = 1;
    *(str->curr_depth) = 0;
    *(str->overruns) = 0;
    *(str->sample_num) = 0;
    pptr = str->pins;
    usefp = 0;
    /* export user specified pins (the ones that sample data) */
    for ( n = 0 ; n < hal_stream_element_count(&str->fifo) ; n++ ) {
	rtapi_snprintf(buf, sizeof(buf), "sampler.%d.pin.%d", num, n);
	retval = hal_pin_new(buf, hal_stream_element_type(&str->fifo, n), HAL_IN, (void **)pptr, comp_id );
	if (retval != 0 ) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"SAMPLER: ERROR: pin '%s' export failed\n", buf);
	    return -EIO;
	}
	/* init the pin value */
	switch ( hal_stream_element_type(&str->fifo, n) ) {
	case HAL_FLOAT:
	    *(pptr->hfloat) = 0.0;
	    usefp = 1;
	    break;
	case HAL_BIT:
	    *(pptr->hbit) = 0;
	    break;
	case HAL_U32:
	    *(pptr->hu32) = 0;
	    break;
	case HAL_S32:
	    *(pptr->hs32) = 0;
	    break;
	default:
	    break;
	}
	pptr++;
    }
    /* export update function */
    retval = hal_export_functf(sample, str, usefp, 0, comp_id, "sampler.%d", num);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SAMPLER: ERROR: function export failed\n");
	return retval;
    }

    return 0;
}

