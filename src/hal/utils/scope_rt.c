/** This file, 'scope_rt.c', is a HAL component that together with
    'scope.c' provides an oscilloscope to view HAL pins, signals,
    and parameters
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
    information, go to https://linuxcnc.org.
*/

#include <rtapi.h>		/* RTAPI realtime OS API */
#include <rtapi_app.h>		/* RTAPI realtime module decls */
#include <hal.h>		/* HAL public API decls */
#include "../hal_priv.h"	/* HAL private API decls */
#include "scope_rt.h"		/* scope related declarations */
#include "rtapi_string.h"

/* module information */
MODULE_AUTHOR("John Kasunich");
MODULE_DESCRIPTION("Oscilloscope for EMC HAL");
MODULE_LICENSE("GPL");

long num_samples = 16000;
long shm_size;
RTAPI_MP_LONG(num_samples, "Number of samples in the shared memory block")

/***********************************************************************
*                         GLOBAL VARIABLES                             *
************************************************************************/

scope_rt_control_t *ctrl_rt;	/* ptr to main RT control structure */
scope_shm_control_t *ctrl_shm;	/* ptr to shared mem control struct */

/***********************************************************************
*                         LOCAL VARIABLES                              *
************************************************************************/

static int comp_id;		/* component ID */
static int shm_id;		/* shared memory ID */
static scope_rt_control_t ctrl_struct;	/* realtime control structure */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static void init_rt_control_struct(void *shmem);
static void init_shm_control_struct(void);

static void sample(void *arg, long period);
static void capture_sample(void);
static int check_trigger(void);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int retval;
    void *shm_base;
    long skip;
    /* connect to the HAL */
    comp_id = hal_init("scope_rt");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "SCOPE: ERROR: hal_init() failed\n");
	return -1;
    }
    /* connect to scope shared memory block */
    skip = (sizeof(scope_shm_control_t) + 3) & ~3;
    shm_size = skip + num_samples * sizeof(scope_data_t);
    shm_id = rtapi_shmem_new(SCOPE_SHM_KEY, comp_id, shm_size);
    if (shm_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SCOPE RT: ERROR: failed to get shared memory (key=0x%x, size=%lu)\n",
            SCOPE_SHM_KEY,
            shm_size
        );
	hal_exit(comp_id);
	return -1;
    }
    retval = rtapi_shmem_getptr(shm_id, &shm_base);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SCOPE: ERROR: failed to map shared memory\n");
	rtapi_shmem_delete(shm_id, comp_id);
	hal_exit(comp_id);
	return -1;
    }

    /* init control structure */
    ctrl_rt = &ctrl_struct;
    init_rt_control_struct(shm_base);

    /* export scope data sampling function */
    retval = hal_export_funct("scope.sample", sample, NULL, 0, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SCOPE_RT: ERROR: sample funct export failed\n");
	hal_exit(comp_id);
	return -1;
    }
    rtapi_print_msg(RTAPI_MSG_DBG, "SCOPE_RT: installed sample function\n");
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    /* is sample function linked to a thread? */
    if (ctrl_shm->thread_name[0] != '\0') {
	/* need to unlink it before we release the scope shared memory */
	hal_del_funct_from_thread("scope.sample", ctrl_shm->thread_name);
    }
    rtapi_shmem_delete(shm_id, comp_id);
    hal_exit(comp_id);
}

/***********************************************************************
*                          REALTIME FUNCTIONS                          *
************************************************************************/

static void sample(void *arg, long period)
{
    (void)arg;
    (void)period;
    int n;

    ctrl_shm->watchdog = 0;
    if (ctrl_shm->state == RESET) {
	/* sampling interrupted, reset everything */
	ctrl_shm->curr = 0;
	ctrl_shm->start = ctrl_shm->curr;
	ctrl_shm->samples = 0;
	ctrl_shm->force_trig = 0;
	/* reset completed, set new state */
	ctrl_shm->state = IDLE;
    }
    ctrl_rt->mult_cntr++;
    if (ctrl_rt->mult_cntr < ctrl_shm->mult) {
	/* not time to do anything yet */
	return;
    }
    /* reset counter */
    ctrl_rt->mult_cntr = 0;
    /* run the sampling state machine */
    switch (ctrl_shm->state) {
    case IDLE:
	/* do nothing while waiting for INIT */
	break;
    case INIT:
	/* init start pointer, curr pointer, sample count */
	ctrl_shm->curr = 0;
	ctrl_shm->start = ctrl_shm->curr;
	ctrl_shm->samples = 0;
	ctrl_shm->force_trig = 0;
	ctrl_rt->auto_timer = 0;
	/* get info about channels */
	for (n = 0; n < 16; n++) {
	    ctrl_rt->data_addr[n] = SHMPTR(ctrl_shm->data_offset[n]);
	    ctrl_rt->data_type[n] = ctrl_shm->data_type[n];
	    ctrl_rt->data_len[n] = ctrl_shm->data_len[n];
	}
	/* set next state */
	ctrl_shm->state = PRE_TRIG;
	break;
    case PRE_TRIG:
	/* acquire a sample */
	capture_sample();
	/* increment sample counter */
	ctrl_shm->samples++;
	/* check if all pre-trigger samples captured */
	if (ctrl_shm->samples >= ctrl_shm->pre_trig) {
	    /* yes - start waiting for trigger */
	    ctrl_shm->state = TRIG_WAIT;
	    /* dummy call to preset 'compare_result' */
	    check_trigger();
	}
	break;
    case TRIG_WAIT:
	/* acquire a sample */
	capture_sample();
	/* increment sample counter */
	ctrl_shm->samples++;
	/* check if trigger condition met */
	if (check_trigger()) {
	    /* yes - start acquiring post trigger data */
	    ctrl_shm->state = POST_TRIG;
	} else {
	    /* no, discard oldest pre-trig sample */
	    ctrl_shm->samples--;
	    ctrl_shm->start += ctrl_shm->sample_len;
	    /* is there a valid sample here, or end of buffer? */
	    if ((ctrl_shm->start + ctrl_shm->sample_len) > ctrl_shm->buf_len) {
		/* end of buffer, wrap back to beginning */
		ctrl_shm->start = 0;
	    }
	}
	break;
    case POST_TRIG:
	/* acquire a sample */
	capture_sample();
	/* increment sample counter */
	ctrl_shm->samples++;
	/* check if all post-trigger samples captured */
	if (ctrl_shm->samples >= ctrl_shm->rec_len) {
	    /* yes - stop sampling and cleanup */
	    ctrl_shm->state = DONE;
	}
	break;
    case DONE:
	/* do nothing while GUI displays waveform */
	break;
    default:
	/* shouldn't get here - if we do, set a legal state */
	ctrl_shm->state = IDLE;
	break;
    }
    /* done */
}

static void capture_sample(void)
{
    scope_data_t *dest;
    int n;

    dest = &(ctrl_rt->buffer[ctrl_shm->curr]);
    /* loop through all channels to acquire data */
    for (n = 0; n < 16; n++) {
	/* capture 1, 2, or 4 bytes, based on data size */
	switch (ctrl_rt->data_len[n]) {
	case 1:
	    dest->d_u8 = *((unsigned char *) (ctrl_rt->data_addr[n]));
	    dest++;
	    break;
	case 4:
	    dest->d_u32 = *((unsigned long *) (ctrl_rt->data_addr[n]));
	    dest++;
	    break;
	case 8:
            {
                ireal_t sample_a, sample_b;
                do {
                    sample_a = *((volatile ireal_t *) (ctrl_rt->data_addr[n]));
                    sample_b = *((volatile ireal_t *) (ctrl_rt->data_addr[n]));
                } while( sample_a != sample_b );
                dest->d_ireal = sample_a;
                dest++;
            }
	    break;
	default:
	    break;
	}
    }
    /* increment sample pointer */
    ctrl_shm->curr += ctrl_shm->sample_len;
    /* is there room in the buffer for another sample? */
    if ((ctrl_shm->curr + ctrl_shm->sample_len) > ctrl_shm->buf_len) {
	/* no, wrap back to beginning of buffer */
	ctrl_shm->curr = 0;
    }
}

// TODO: type-independent way to get high bit
// #define SIGN_BIT (~(((ireal_t)~(ireal_t)0)>>1))
static int check_trigger(void)
{
    static int compare_result = 0;
    int prev_compare_result;
    scope_data_t *value, *level;

    /* has user forced trigger? */
    if (ctrl_shm->force_trig != 0) {
	return 1;
    }
    /* is auto trigger enabled? */
    if (ctrl_shm->auto_trig != 0) {
	/* yes, has the delay time expired? */
	if (++ctrl_rt->auto_timer >= ctrl_shm->rec_len) {
	    return 1;
	}
    } else {
	/* no auto, reset delay timer */
	ctrl_rt->auto_timer = 0;
    }
    /* if no trigger channel is selected we're done */
    if (ctrl_shm->trig_chan == 0) {
	return 0;
    }
    /* point a scope_data_t union at the signal value */
    value = ctrl_rt->data_addr[ctrl_shm->trig_chan - 1];
    /* and at the trigger level */
    level = &(ctrl_shm->trig_level);
    /* save previous compare result */
    prev_compare_result = compare_result;
    /* compare actual value to trigger level */
    switch (ctrl_rt->data_type[ctrl_shm->trig_chan - 1]) {
    case HAL_BIT:
	/* for bits, we don't even look at the trigger level */
	compare_result = value->d_u8;
	break;
    case HAL_FLOAT:
	{
	ireal_t tmp1, tmp2;
	/* don't want to use the FPU in this function, so we use */
	/* a hack - see http://en.wikipedia.org/wiki/IEEE_754 */
	/* this _only_ works with IEEE-754 floating point numbers */
	/* and will probably fail for infinities, NANs, etc. */
	/* OK, here we go! */
	/* get the value as an integer */
	tmp1 = value->d_ireal;
	/* get the trigger as an integer */
	tmp2 = level->d_ireal;
	/* is the value negative? (highest bit) */
	if (tmp1 & 0x8000000000000000ull) {
	    /* yes, is the trigger level negative? */
	    if (tmp2 & 0x8000000000000000ull) {
		/* yes, make both positive */
		tmp1 ^= 0x8000000000000000ull;
		tmp2 ^= 0x8000000000000000ull;
		/* and compare them as unsigned ints */
		/* because of negation, we reverse the compare */
		compare_result = (tmp1 < tmp2);
	    } else {
		/* trigger level positive, value negative */
		compare_result = 0;
	    }
	} else {
	    /* value is positive, is trigger level negative? */
	    if (tmp2 & 0x8000000000000000ull) {
		/* trigger level negative, value positive */
		compare_result = 1;
	    } else {
		/* both are positive */
		/* compare them as unsigned ints */
		compare_result = (tmp1 > tmp2);
	    }
	}
	}
	break;
    case HAL_S32:
	compare_result = (value->d_s32 > level->d_s32);
	break;
    case HAL_U32:
	compare_result = (value->d_u32 > level->d_u32);
	break;
    default:
	compare_result = 0;
	break;
    }
    /* test for rising edge */
    if (ctrl_shm->trig_edge && compare_result && !prev_compare_result) {
	return 1;
    }
    /* test for falling edge */
    if (!ctrl_shm->trig_edge && !compare_result && prev_compare_result) {
	return 1;
    }
    return 0;
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static void init_rt_control_struct(void *shmem)
{
    char *cp;
    int n, skip;

    /* first clear entire struct to all zeros */
    cp = (char *) ctrl_rt;
    for (n = 0; n < (int)sizeof(scope_rt_control_t); n++) {
	cp[n] = 0;
    }
    /* save pointer to shared control structure */
    ctrl_shm = shmem;
    /* round size of shared struct up to a multiple of 4 for alignment */
    skip = (sizeof(scope_shm_control_t) + 3) & ~3;
    /* the rest of the shared memory area is the data buffer */
    ctrl_rt->buffer = (scope_data_t *) (((char *) (shmem)) + skip);
    init_shm_control_struct();
    /* init any non-zero fields */

    /* done */
}

static void init_shm_control_struct(void)
{
    char *cp;
    int skip, n;

    /* first clear entire struct to all zeros */
    cp = (char *) ctrl_shm;
    for (n = 0; n < (int)sizeof(scope_shm_control_t); n++) {
	cp[n] = 0;
    }
    /* round size of shared struct up to a multiple of 4 for alignment */
    ctrl_shm->shm_size = shm_size;
    skip = (sizeof(scope_shm_control_t) + 3) & ~3;
    /* remainder of shmem area is buffer */
    ctrl_shm->buf_len = (shm_size - skip) / sizeof(scope_data_t);
    /* init any non-zero fields */
    ctrl_shm->mult = 1;
    ctrl_shm->state = IDLE;
}
