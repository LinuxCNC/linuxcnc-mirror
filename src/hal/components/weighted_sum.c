/********************************************************************
* Description: weighted_sum.c
*   Weighted summer for HAL
*
*   See the "Users Manual" at emc2/docs/Hal_Introduction.pdf
*
* This component is a "weighted summer".  It has a (user specified)
* number of HAL_BIT input pins, and a HAL_S32 parameter corresponding
* to each bit input.
* There is one HAL_S32 output.  The output value is the sum of the
* parameters for which the corresponding bit input is true.
*
* The default value for the parameters is 2^n, where n is the bit number.
* Default behavior results in a binary -> unsigned conversion.
*
* There is one array parameter at module load time, the number of bits for
* each weighted summer.  There is a limit of 8 weighted summers, and each
* may have up to 16 input bits.
*
*********************************************************************
*
* Author: Stephen Wille Padnos (swpadnos AT sourceforge DOT net)
*       Based on a work by John Kasunich
* License: GPL Version 2
* Created on: May 17, 2006
* System: Linux
*
* Copyright (c) 2006 All rights reserved.
*
********************************************************************/

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"		/* HAL public API decls */

#define MAX_SUMMERS	8

/* module information */
MODULE_AUTHOR("Stephen Wille Padnos");
MODULE_DESCRIPTION("Weighted Summer for EMC HAL");
MODULE_LICENSE("GPL");
int wsum_sizes[MAX_SUMMERS] = { -1, -1, -1, -1, -1, -1, -1, -1 };
RTAPI_MP_ARRAY_INT(wsum_sizes, MAX_SUMMERS, "Sizes of up to 8 weighted summers");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* Data needed for each bit of a weighted summer */
typedef struct {
    hal_bit_t *bit;		/* pin: the input bit HAL pin */
    hal_s32_t *weight;		/* pin: the numeric weight of this pin */
} wsum_bit_t;

/* Base data for a weighted summer. */
typedef struct {
    hal_s32_t *sum;		/* output pin: the calculated sum */
    hal_s32_t *offset;		/* pin: offset for this summer */
    hal_bit_t *hold;		/* input pin: hold value if 1, update if 0 */
    int num_bits;		/* internal: How many bits are in this summer */
    wsum_bit_t *bits;		/* internal: pointer to the input bits and weights */
} wsum_t;

/* pointer to array of wsum structs in shmem */
static wsum_t *wsum_array;
static wsum_bit_t *wsum_bit_array;

/* other globals */
static int comp_id;		/* component ID */
static int num_summers;		/* number of summers created */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static int export_wsum(int num, int num_bits, wsum_t *addr, wsum_bit_t *bitaddr);
static void process_wsums(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int n, total_bits, retval;

    total_bits = 0;

    /* check that there's at least one valid summer requested */
    for (n = 0; n < MAX_SUMMERS && wsum_sizes[n] != -1 ; n++) {
	if (wsum_sizes[n] < 1) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
			    "WEIGHTED_SUM: ERROR: Invalid number of bits '%i' for summer %i\n",
			    wsum_sizes[n], n);
	    return -1;
	} else {
	    num_summers++;
	    total_bits += wsum_sizes[n];
	}
    }

    if (num_summers == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WEIGHTED_SUM: ERROR: no summers specified\n");
	return -1;
    }

    /* have good config info, connect to the HAL */
    comp_id = hal_init("weighted_sum");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WEIGHTED_SUM: ERROR: hal_init() failed\n");
	return -1;
    }
    /* allocate shared memory for summer base info */
    wsum_array = hal_malloc(num_summers * sizeof(wsum_t));
    if (wsum_array == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WEIGHTED_SUM: ERROR: hal_malloc() for summer array failed\n");
	hal_exit(comp_id);
	return -1;
    }
    /* allocate shared memory for summer bit arrays */
    wsum_bit_array = hal_malloc(total_bits * sizeof(wsum_bit_t));
    if (wsum_bit_array == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WEIGHTED_SUM: ERROR: hal_malloc() for summer bit array failed\n");
	hal_exit(comp_id);
	return -1;
    }

    /* export pins/params for all summers */
    total_bits = 0;
    for (n = 0; n < num_summers; n++) {
	/* export all vars */
	retval = export_wsum(n, wsum_sizes[n], &(wsum_array[n]), &(wsum_bit_array[total_bits]));
	if (retval != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"WEIGHTED_SUM: ERROR: group %d export failed\n", n);
	    hal_exit(comp_id);
	    return -1;
	}
	total_bits += wsum_array[n].num_bits;
    }

    /* export update function */
    retval = hal_export_funct("process_wsums", process_wsums, wsum_array, 1, 0, comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WEIGHTED_SUM: ERROR: process_wsums funct export failed\n");
	return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
	"WEIGHTED_SUM: installed %d weighted summers\n", num_summers);
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}

/***********************************************************************
*                     REALTIME DELAY FUNCTION                          *
************************************************************************/

/*  The summer works by checking the input bits, and adding the
    weight to the sum if the input is true.
*/
static void process_wsums(void *arg, long period)
{
    (void)period;
    wsum_t *wsums, *thissum;
    int n, b, running_total;

    /* point to filter group */
    wsums = (wsum_t *)arg;

    for (n=0 ; n<num_summers ; n++) {
	thissum = &(wsums[n]);
	if (*(thissum->hold)) continue;
	else {
	    running_total = *(thissum->offset);
	    for (b=0 ; b<thissum->num_bits ; b++) {
		if (*(thissum->bits[b].bit)) {
		    running_total += *(thissum->bits[b].weight);
		}
	    }
	}
	*(thissum->sum) = running_total;
    }
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

static int export_wsum(int num, int num_bits, wsum_t *addr, wsum_bit_t *bitaddr)
{
    int retval, i, w;
    char buf[HAL_NAME_LEN+1], base[HAL_NAME_LEN+1];

    rtapi_snprintf(base, sizeof(base), "wsum.%d", num);
    /* export pin for offset (input) */
    rtapi_snprintf(buf, sizeof(buf), "%s.offset", base);
    retval = hal_pin_s32_new(buf, HAL_IO, &(addr->offset), comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WEIGHTED_SUM: ERROR: '%s' param export failed\n", buf);
	return retval;
    }

    /* export pin for output sum */
    rtapi_snprintf(buf, sizeof(buf), "%s.sum", base);
    retval = hal_pin_s32_new(buf, HAL_OUT, &(addr->sum), comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WEIGHTED_SUM: ERROR: '%s' pin export failed\n", buf);
	return retval;
    }

    /* export pin for update hold */
    rtapi_snprintf(buf, sizeof(buf), "%s.hold", base);
    retval = hal_pin_bit_new(buf, HAL_IN, &(addr->hold), comp_id);
    if (retval != 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "WEIGHTED_SUM: ERROR: '%s' pin export failed\n", buf);
	return retval;
    }

    addr->bits = bitaddr;
    addr->num_bits = num_bits;
    /* export the input bits and weight parameters, and set the default weights */
    w = 1;
    for (i=0;i<num_bits;i++) {
	rtapi_snprintf(buf, sizeof(buf), "%s.bit.%d.in", base, i);
	retval = hal_pin_bit_new(buf, HAL_IN, &(addr->bits[i].bit), comp_id);
	if (retval != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
	    "WEIGHTED_SUM: ERROR: '%s' pin export failed\n", buf);
	    return retval;
	}
	rtapi_snprintf(buf, sizeof(buf), "%s.bit.%d.weight", base, i);
	retval = hal_pin_s32_new(buf, HAL_IO, &(addr->bits[i].weight), comp_id);
	if (retval != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
	    "WEIGHTED_SUM: ERROR: '%s' param export failed\n", buf);
	    return retval;
	}
	*(addr->bits[i].weight) = w;
	w <<= 1;
    }

    /* set initial parameter and pin values */
    *(addr->offset) = 0;
    *(addr->sum) = 0;
    return 0;
}
