
/********************************************************************
* Description:  hal_speaker.c
*               This file, 'hal_speaker.c', drives the PC speaker based
*               on up to 8 bit outputs.  When the new outputs differ
*               from the old outputs, a click is output on the speaker.
*               This implementation only work on the x86 architecture.
*               
* Author: John Kasunich and Jeff Epler
* License: GPL Version 2
*    
* Copyright (c) 2003, 2006 All rights reserved.
*
* Last change: 
********************************************************************/

/* 
 Most of this code is taken from hal_skeleton by John Kasunich,
 which is also a good starting point for new drivers.

 This driver support no configuration strings so installing is easy:
     halcmd loadrt hal_speaker

 The driver creates a HAL pin and a function as follows:

 Pin: 'speaker.<portnum>.pin-<pinnum>-out'
 Function: 'speaker.<portnum>.write'

*/

/** Copyright (C) 2003 John Kasunich
                       <jmkasunich AT users DOT sourceforge DOT net>
                       Martin Kuhnle
                       <mkuhnle AT users DOT sourceforge DOT net>
                       Jeff Epler
                       <jepler@unpythonic.net>
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

#include "config.h"     /* environment flags */
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "rtapi_io.h"
#include "hal.h"		/* HAL public API decls */

/* If FASTIO is defined, uses outb() and inb() from <asm.io>,
   instead of rtapi_outb() and rtapi_inb() - the <asm.io> ones
   are inlined, and save a microsecond or two (on my 233MHz box)
*/
#if defined(RTAPI_RTAI)
#define FASTIO
#endif /* RTAPI_RTAI */

#ifdef FASTIO
#define rtapi_inb inb
#define rtapi_outb outb
#include <asm/io.h>
#endif

/* module information */
MODULE_AUTHOR("Jeff Epler");
MODULE_DESCRIPTION("PC Speaker Driver");
MODULE_LICENSE("GPL");
/* static char *cfg = 0; */
/* config string
RTAPI_MP_STRING(cfg, "config string"); */

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* this structure contains the runtime data needed by the
   driver for a single port/channel
*/

typedef struct {
    hal_bit_t *signals[8];
    uint8_t last;
} speaker_t;

/* pointer to array of speaker_t structs in shared memory, 1 per port */
static speaker_t *port_data_array;

/* other globals */
static int comp_id;		/* component ID */
static int num_ports;		/* number of ports configured */

/**************************************************************
* REALTIME PORT WRITE FUNCTION                                *
**************************************************************/

#define SPEAKER_PORT 0x61

static void write_port(void *arg, long period)
{
    (void)period;
    uint8_t v = 0;
    uint8_t oldval;
    int i;
    speaker_t *port;
    port = arg;
    
    for(i=0; i<8; i++) {
        if(*(port->signals[i])) v = v | (1<<i);
    }

    /* write it to the hardware */
    oldval = rtapi_inb(SPEAKER_PORT) & 0xfc;

    if(v != port->last) {
        rtapi_outb(oldval | 2, SPEAKER_PORT);
    } else {
        rtapi_outb(oldval, SPEAKER_PORT);
    }

    port->last = v;
}

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int i, n, retval;

    /* only one port at the moment */
    num_ports = 1;
    n = 0; /* port number */

    /* STEP 1: initialise the driver */
    comp_id = hal_init("hal_speaker");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SPEAKER: ERROR: hal_init() failed\n");
	return -1;
    }

#if !defined(RTAPI_RTAI)
    /* STEP 1.1: get access to port, only needed in uspace builds */
    if (rtapi_ioperm(SPEAKER_PORT, 1, 1) < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SPEAKER: ERROR: ioperm() failed\n");
	hal_exit(comp_id);
	return -1;
    }
#endif /* RTAPI_RTAI */

    /* STEP 2: allocate shared memory for skeleton data */
    port_data_array = hal_malloc(num_ports * sizeof(speaker_t));
    if (port_data_array == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SPEAKER: ERROR: hal_malloc() failed\n");
	hal_exit(comp_id);
	return -1;
    }

    /* STEP 3: export the pin(s) */
    for(i = 0; i < 8; i++) {
        retval = hal_pin_bit_newf(HAL_IN, &(port_data_array->signals[i]),
				  comp_id, "speaker.%d.pin-%02d-out", n, i);
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "SPEAKER: ERROR: port %d var export failed with err=%i\n", n,
                retval);
            hal_exit(comp_id);
            return -1;
        }
    }

    /* STEP 4: export write function */
    retval =
	hal_export_functf(write_port, &(port_data_array[n]), 0, 0,
	comp_id, "speaker.%d.write", n);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SPEAKER: ERROR: port %d write funct export failed\n", n);
	hal_exit(comp_id);
	return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
	"SPEAKER: installed driver for %d ports\n", num_ports);
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}


