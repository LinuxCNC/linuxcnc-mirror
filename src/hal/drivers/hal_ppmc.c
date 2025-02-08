/********************************************************************
* Description:  hal_ppmc.c
*               HAL driver for the the Pico Systems family of
*               parallel port motion control boards, including
*               the PPMC board set, the USC, and the UPC. 
*
* Usage:  halcmd loadrt hal_ppmc port_addr=<addr1>[,addr2[,addr3]]
*		[extradac=<slotcode1>,[<slotcode2>]]
*		[extradout=<slotcode1>,[<slotcode2>]]
*               [timestamp=<slotcode1>,[<slotcode2>]]
*               [enc_clock=<slotcode1>,[<slotcode2>]]
*               [epp_dir=<1 | 0>  [, 1 | 0]  [,1 | 0]]
*               where 'addr1', 'addr2', and 'addr3' are the addresses
*               of up to three parallel ports.
*
*		extradac and extradout allow you to tell the driver
* 		that you have an 8 bit DAC, or 8 digital outputs, on
*		the 'extra' port of a USC or UPC board.  The slotcode
*		is a two digit hex number of the form 0X12, where the
*		first digit is the bus number (0, 1, or 2), and the
*		second digit is the slot number on that bus (0 to F).
*		The slotcode tells the driver which board(s) have the
*		optional DAC or digital outs installed.
*               timestamp works the same way, for UPC boards of rev 4
*               or higher that have the timestamp feature.
*               enc_clock specifies a 3-digit hex value, where the 1st is a
*               code of 1,2, 5 or 10 to indicate an encoder clock rate of 1, 2.5, 5 or 10 MHz.
*               The following 2 digits work as above, bus and board address.
*               Only rev 4 and above PPMC encoder boards have this clock select feature.
*               epp_dir defaults to 0, in which case the EPP parallel port automatically selects the
*               par port data bus direction.  If epp_dir=1 is given, then this driver explicitly forces the
*               port direction every time it needs to be changed.  This is known to cause at least one PCIe
*               port card to malfunction.

* Author: John Kasunich, Jon Elson, Stephen Wille Padnos
* License: GPL Version 2
*    
* Copyright (c) 2005 All rights reserved.
*
********************************************************************/

/** The driver searches the entire address space of the enhanced
    parallel port (EPP) at 'port_addr', looking for any board(s)
    in the PPMC family.  It then exports HAL pins for whatever it
    finds, as well as a pair of functions, one that reads all 
    inputs, and one that writes all outputs.
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

#include <rtapi_slab.h>		/* kmalloc() */
#include <rtapi_io.h>		/* kmalloc() */
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"		/* HAL public API decls */
#include "hal_parport.h"

#define MAX_BUS 3	/* max number of parports (EPP busses) */

#define	EPSILON		1e-20

/* module information */
MODULE_AUTHOR("John Kasunich");
MODULE_DESCRIPTION("HAL driver for Universal PWM Controller");
MODULE_LICENSE("GPL");
int port_addr[MAX_BUS] = { 0x378, [1 ... MAX_BUS-1] = -1 };
    /* default, 1 bus at 0x0378 */
hal_parport_t port_registration[MAX_BUS];
RTAPI_MP_ARRAY_INT(port_addr, MAX_BUS, "port address(es) for EPP bus(es)");
int extradac[MAX_BUS*8] = {
        -1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1 };  /* default, no extra stuff */
RTAPI_MP_ARRAY_INT(extradac, MAX_BUS*8, "bus/slot locations of extra DAC modules");
int extradout[MAX_BUS*8] = {
        -1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1 };  /* default, no extra stuff */
RTAPI_MP_ARRAY_INT(extradout, MAX_BUS*8, "bus/slot locations of extra dig out modules");
int timestamp[MAX_BUS*8] = {
        -1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1 };  /* default, no extra stuff */
RTAPI_MP_ARRAY_INT(timestamp, MAX_BUS*8, "bus/slot locations of timestamped encoders");
int enc_clock[MAX_BUS*8] = {
        -1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1 };  /* default, no extra stuff */
RTAPI_MP_ARRAY_INT(enc_clock, MAX_BUS*8, "bus/slot locations of encoder clock settings");
int  epp_dir[MAX_BUS] = {0 , [1 ... MAX_BUS-1] = 0 };
RTAPI_MP_ARRAY_INT(epp_dir, MAX_BUS, "EPP is commanded port direction");

/***********************************************************************
*                DEFINES (MOSTLY REGISTER ADDRESSES)                   *
************************************************************************/

#define SPPDATA(addr) addr
#define STATUSPORT(addr) addr+1
#define CONTROLPORT(addr) addr+2
#define ADDRPORT(addr) addr+3
#define DATAPORT(addr) addr+4

#define NUM_SLOTS      16
#define SLOT_SIZE      16
#define SLOT_ID_OFFSET 15

#define ENCCNT0     0x00	/*  EPP address to read first byte of encoder counter */
#define ENCCNT1     0x03
#define ENCCNT2     0x06
#define ENCCNT3     0x09

#define ENCCTRL     0x03	/* EPP address of encoder control register */
#define ENCRATE     0x04	/* interrupt rate register, only on master encoder */
#define ENCCLOCK    0x05	/* encoder digital filter clock rate (1,2.5, 5 or 10 MHz PPMC only) */
#define ENCISR      0x0C	/* index detect latch register (read only) */
#define ENCINDX     0x0D	/* index reset register (write only) */
                                /* only available with rev 2 and above FPGA config */
#define ENCLOAD     0x00	/* EPP address to write into first byte of preset */
				/* register for channels 0 - 3 */
// following regs for new UPC with encoder count timestamp feature
#define ENCTS       0x10        /* timestamp low byte for axis 0 */
#define ENCTS1      0x11        /* timestamp high byte for axis 0 */
#define ENCTB       0x18        /* timebase low byte */
#define ENCTB1      0x19        /* timebase high byte */

#define DAC_0       0x00	/* EPP address of low byte of DAC chan 0 */
#define DAC_1       0x02	/* EPP address of low byte of DAC chan 1 */
#define DAC_2       0x04	/* EPP address of low byte of DAC chan 2 */
#define DAC_3       0x06	/* EPP address of low byte of DAC chan 3 */

#define DAC_MODE_0  0x4C
#define DAC_WRITE_0 0x48

#define DAC_MODE_1  0x5C
#define DAC_WRITE_1 0x58

#define PWM_GEN_0       0x10	/* EPP addr of low byte of PWM generator 0 LSB */
//      PWM_GEN_0a      0x11	/* high byte MSB */
#define PWM_GEN_1       0x12
#define PWM_GEN_2       0x14
#define PWM_GEN_3       0x16
#define PWM_CTRL_0      0x1C
#define PWM_FREQ_LO     0x1D
#define PWM_FREQ_HI     0x1E

#define RATE_GEN_0      0x10	/* EPP addr of low byte of rate generator 0 LSB */
//      RATE_GEN_0a     0x11	/* middle byte */
//      RATE_GEN_0b     0x12	/* MSB */
#define RATE_GEN_1      0x13
#define RATE_GEN_2      0x16
#define RATE_GEN_3      0x19
#define RATE_CTRL_0     0x1C
#define RATE_SETUP_0    0x1D
#define RATE_WIDTH_0    0x1E

#define UxC_DINA        0x0D	/* EPP address of digital inputs on univstep */
#define UxC_DINB        0x0E
#define UxC_ESTOP_IN    0x0E
/* The "estop" input will always report OFF to the software, regardless
   of the actual state of the physical input, unless the "estop" output
   has been turned on by the software. */
#define UxC_EXTRA       0x0F    /* 8 bit "extra" port on USC and UPC */

/* codes to indicate if/how the extra port is being used */
#define EXTRA_UNUSED	0
#define	EXTRA_DAC	1
#define	EXTRA_DOUT	2

#define UxC_DOUTA       0x1F	/* EPP address of digital outputs */
#define UxC_ESTOP_OUT   0x1F
#define UxC_SLAVE       0x06	/* EPP address of slave register */
/* The physical output associated with the "estop" output will not come on
   unless the physical "estop" input is also on.  All physical outputs
   will not come on unless the physical "estop" output is on. */
   
/* The ESTOP function is completely implemented in FPGA hardware.  To get
out of ESTOP, the safety chain must be a closed circuit (Green LED lit on
board), you then must satisfy the watchdog (if watchdog jumper is in ON
position) by writing to two adjacent velocity output channels (step, PWM or
DAC) and finally by writing a 1 to the ESTOP command bit.  If all these
safetys are satisfied, the board will come out of ESTOP, and the velocity
and digital outputs will be enabled.  If any of these safety inputs indicates
an unsafe condition, then the board will immediately return to the ESTOP state.
*/

#define DIO_DINA        0x00	/* EPP address of digital inputs on DIO */
#define DIO_DINB        0x01
#define DIO_ESTOP_IN    0x02

#define DIO_DOUTA       0x00	/* EPP address of digital outputs */
#define DIO_ESTOP_OUT   0x02

/***********************************************************************
*                       STRUCTURE DEFINITIONS                          *
************************************************************************/

/* this structure contains the runtime data for a digital output */
typedef struct {
    hal_bit_t *data;		/* output pin value */
    hal_bit_t invert;		/* parameter to invert output pin */
} dout_t;

/* this structure contains the runtime data for a digital input */
typedef struct {
    hal_bit_t *data;		/* input pin value */
    hal_bit_t *data_not;	/* inverted input pin value */
} din_t;

/* this structure contains the runtime data for a step pulse generator */
typedef struct {
    hal_bit_t *enable;		/* enable pin for step generator */
    hal_float_t *vel;		/* velocity command pin*/
    hal_float_t scale;		/* parameter: scaling for vel to Hz */
    hal_float_t max_vel;	/* velocity limit */
    hal_float_t freq;		/* parameter: velocity cmd scaled to Hz */
} stepgen_t;

/* runtime data for a set of 4 step pulse generators */
typedef struct {
    stepgen_t sg[4];		/* per generator data */
    hal_u32_t setup_time_ns;	/* setup time in nanoseconds */
    hal_u32_t pulse_width_ns;	/* pulse width in nanoseconds */
    hal_u32_t pulse_space_ns;	/* min pulse space in nanoseconds */
} stepgens_t;

#define BOOT_NORMAL 0
#define BOOT_REV    1
#define BOOT_FWD    2

/* this structure contains the runtime data for a PWM generator */
typedef struct {
    hal_bit_t *enable;		/* enable pin for PWM generator */
    hal_float_t *value;		/* value command pin */
    hal_float_t scale;		/* parameter: scaling */
    hal_float_t max_dc;		/* maximum duty cycle 0.0-1.0 */
    hal_float_t min_dc;		/* minimum duty cycle 0.0-1.0 */
    hal_float_t duty_cycle;	/* actual duty cycle output */
    hal_bit_t bootstrap;	/* enable bootstrap mode (pulses at startup) */
    unsigned char boot_state;	/* state for bootstrap state machine */
    unsigned char old_enable;	/* used to detect rising edge, for boot */
} pwmgen_t;

/* runtime data for a set of 4 PWM generators */
typedef struct {
    pwmgen_t pg[4];		/* per generator data */
    hal_float_t freq;		/* PWM frequency */
    hal_float_t old_freq;	/* previous value, to detect changes */
    unsigned short period;	/* period in clock ticks */
    double period_recip;	/* reciprocal of period */
} pwmgens_t;

/* this structure contains the runtime data for a 16-bit DAC */
typedef struct {
    hal_float_t *value;		/* value command pin */
    hal_float_t scale;		/* parameter: scaling */
} DAC_t;

/* runtime data for a 4-channel 16-bit DAC */
typedef struct {
    DAC_t pg[4];		/* per DAC data */
} DACs_t;

/* runtime data for an "extra" port - can be either a DAC, or 8 digouts */
typedef union {
    DAC_t  dac;			/* if the port is used for a DAC */
    dout_t douts[8];		/* if the port is used for digital outs */
} extra_t;

/* runtime data for a single encoder */
typedef struct {
  hal_float_t *position;      /* output: scaled position pointer */
  hal_s32_t *count;           /* output: unscaled encoder counts */
  hal_s32_t *delta;		/* output: delta counts since last read */
  hal_s32_t prevdir;		/* local: previous direction  */
  hal_float_t scale;          /* parameter: scale factor */
  hal_bit_t *index;           /* output: index flag */
  hal_bit_t *index_enable;    /* enable index pulse to reset encoder count */
  hal_s32_t oldreading;     /* used to detect overflow / underflow of the counter JE001 */
  unsigned int indres;        /* copy of reset-on-index register bits (only valid on 1st encoder of board)*/
  unsigned int indrescnt;    /* counts servo cycles since index reset was turned on */
  hal_float_t *vel;             /* output: scaled velocity */
  hal_float_t min_speed;        /* parameter: min speed for velocity estimation */
  hal_u32_t counts_since_timeout;    /* for velocity estimation */
  unsigned short old_timestamp;
  unsigned short timestamp;
} encoder_t;

/* this structure contains the runtime data for a single EPP bus slot */
/* A single slot can contain a wide variety of "stuff", ranging 
   from PWM or stepper or DAC outputs, to encoder inputs, to digital
   I/O.  at runtime, the proper function(s) need to be invoked to 
   handle it.  We do that by having an array of functions that are
   called in order.  The entries are filled in when the init code
   scans the bus and determines what is in each slot */

#define MAX_FUNCT 10

struct slot_data_s;

typedef void (slot_funct_t)(struct slot_data_s *slot);

typedef struct slot_data_s {
    unsigned char id;		/* slot id code */
    unsigned char ver;		/* slot version code */
    unsigned char strobe;	/* does this slot need a latch strobe */
    unsigned char slot_base;	/* base addr of this 16 byte slot */
    unsigned int port_addr;	/* addr of parport */
    rtapi_u32 read_bitmap;		/* map showing which registers to read */
    unsigned char num_rd_functs;/* number of read functions */
    unsigned char rd_buf[32];	/* cached data read from epp bus */
    slot_funct_t *rd_functs[MAX_FUNCT];	/* array of read functions */
    rtapi_u32 write_bitmap;		/* map showing which registers to write */
    unsigned char num_wr_functs;/* number of write functions */
    unsigned char wr_buf[32];	/* cached data to be written to epp bus */
    slot_funct_t *wr_functs[MAX_FUNCT];	/* array of write functions */
    dout_t *digout;		/* ptr to shmem data for digital outputs */
    din_t *digin;		/* ptr to shmem data for digital inputs */
    stepgens_t *stepgen;	/* ptr to shmem data for step generators */
    pwmgens_t *pwmgen;		/* ptr to shmem data for PWM generators */
    encoder_t *encoder;         /* ptr to shmem data for encoders */
    DACs_t *DAC;                /* ptr to shmem data for DACs */
    int extra_mode;		/* indicates if/how "extra" port is used */
    extra_t *extra;		/* ptr to shmem for "extra" port */
    unsigned int use_timestamp;   /* indicates whether to use timestamp encoder feature */
    unsigned int enc_freq  ;     /* encoder clock rate (PPMC only) */
} slot_data_t;

/* this structure contains the runtime data for a complete EPP bus */

typedef struct {
//    unsigned int port_addr;	/* addr of parport to talk to board */
    int busnum;			/* bus number */
    unsigned char have_master;	/* true if a master has been configured */
    unsigned int last_digout;	/* used for numbering digital outputs */
    unsigned int last_digin;	/* used for numbering digital outputs */
    unsigned int last_stepgen;	/* used for numbering step generators */
    unsigned int last_pwmgen;	/* used for numbering PWM generators */
    unsigned int last_encoder;	/* used for numbering encoders */
    unsigned int last_DAC;	/* used for numbering DACs */
    unsigned int last_extraDAC;	/* used for numbering DACs */
//    unsigned int last_extradout;/* used for numbering digital outputs */
    char slot_valid[NUM_SLOTS];	/* tags for slots that are used */
    slot_data_t slot_data[NUM_SLOTS];  /* data for slots on EPP bus */
} bus_data_t;


/***********************************************************************
*                          GLOBAL VARIABLES                            *
************************************************************************/

static bus_data_t *bus_array[MAX_BUS];
static int comp_id;		/* component ID */
static long read_period;        /* makes real time period available to called functions */
static int slotnum;             
static int currentbus;             /* made global so SelRead can see which parport is being handled */
                                /* to deal with epp_dir option  */

/***********************************************************************
*                    REALTIME FUNCTION DECLARATIONS                    *
************************************************************************/

static void read_all(void *arg, long period);
static void write_all(void *arg, long period);

static void read_digins(slot_data_t *slot);
static void write_digouts(slot_data_t *slot);
static void write_stepgens(slot_data_t *slot);
static void write_pwmgens(slot_data_t *slot);
static void read_encoders(slot_data_t *slot);
static void write_DACs(slot_data_t *slot);
static void write_extraDAC(slot_data_t *slot);

/***********************************************************************
*                  REALTIME I/O FUNCTION DECLARATIONS                  *
************************************************************************/

#if 0 /* FIXME not used */
static void BusReset(unsigned int port_addr);
#endif
static int ClrTimeout(unsigned int port_addr);
static unsigned short SelRead(unsigned char epp_addr, unsigned int port_addr);
static unsigned short ReadMore(unsigned int port_addr);
static void SelWrt(unsigned char byte, unsigned char epp_addr, unsigned int port_addr);
static void WrtMore(unsigned char byte, unsigned int port_addr);


/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static rtapi_u32 block(int min, int max);
static int add_rd_funct(slot_funct_t *funct, slot_data_t *slot, rtapi_u32 cache_bitmap );
static int add_wr_funct(slot_funct_t *funct, slot_data_t *slot, rtapi_u32 cache_bitmap );

static int export_UxC_digin(slot_data_t *slot, bus_data_t *bus);
static int export_UxC_digout(slot_data_t *slot, bus_data_t *bus);
static int export_PPMC_digin(slot_data_t *slot, bus_data_t *bus);
static int export_PPMC_digout(slot_data_t *slot, bus_data_t *bus);
static int export_USC_stepgen(slot_data_t *slot, bus_data_t *bus);
static int export_UPC_pwmgen(slot_data_t *slot, bus_data_t *bus);
static int export_PPMC_DAC(slot_data_t *slot, bus_data_t *bus);
static int export_encoders(slot_data_t *slot, bus_data_t *bus);
static int export_extra_dac(slot_data_t *slot, bus_data_t *bus);
static int export_extra_dout(slot_data_t *slot, bus_data_t *bus);
static int export_timestamp(slot_data_t *slot, bus_data_t *bus);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

void rtapi_app_exit(void);

int rtapi_app_main(void)
{
  int msg, rv, rv1, busnum, slotnum, n, boards;
    int bus_slot_code, need_extra_dac, need_extra_dout, need_timestamp;
    int idcode, id, ver;
    bus_data_t *bus;
    slot_data_t *slot;

    /* connect to the HAL */
    comp_id = hal_init("hal_ppmc");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "PPMC: ERROR: hal_init() failed\n");
	return -1;
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: installing driver\n");
    /* This function exports a lot of stuff, which results in a lot of
       logging if msg_level is at INFO or ALL. So we save the current value
       of msg_level and restore it later.  If you actually need to log this
       function's actions, change the second line below */
    msg = rtapi_get_msg_level();
        rtapi_set_msg_level(RTAPI_MSG_INFO);
    //    rtapi_set_msg_level(RTAPI_MSG_ERR);

    /* validate port addresses */
    n = 0;
    /* if an error occurs, bailing out in the middle of init can leave
       a mess of partially allocated stuff.  So we don't normally bail
       out, instead we set rv non-zero to indicate failure, and carry
       on.  Later, we see that rv is set, and clean up anything that
       might have been allocated before we return. */
    rv = 0;
    for ( busnum = 0 ; busnum < MAX_BUS ; busnum++ ) {
      rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: bus %d epp_dir = %d\n",busnum, epp_dir[busnum]);

	/* init pointer to bus data */
	bus_array[busnum] = NULL;
	/* check to see if a port address was specified */
	if ( port_addr[busnum] == -1 ) {
	    /* nope, skip it */
	    continue;
	}

        rv = hal_parport_get(comp_id, &port_registration[busnum],
                port_addr[busnum], 0, PARPORT_MODE_EPP);

        if(rv < 0)
            return rv;

        port_addr[busnum] = port_registration[busnum].base;
        if(port_registration[busnum].base_hi)
            rtapi_outb(0x80, port_registration[busnum].base_hi + 2);

        /* got a good one */
        n++;
    }
    if ( n == 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR, 
	    "PPMC: ERROR: no ports specified\n");
	hal_exit(comp_id);
	return -1;
    }
    /* have valid config info */
    /* begin init - loop thru all busses */
    for ( busnum = 0 ; busnum < MAX_BUS ; busnum++ ) {
	/* check to see if a port address was specified */
	if ( port_addr[busnum] == -1 ) {
	    /* nope, skip to next bus */
	    continue;
	}
	rtapi_print_msg(RTAPI_MSG_INFO,
	    "PPMC: checking EPP bus %d at port %04X\n",
	    busnum, port_addr[busnum]);
	boards = 0;
	/* allocate memory for bus data - this is not shared memory */
	bus = rtapi_kmalloc(sizeof(bus_data_t), RTAPI_GFP_KERNEL);
	if (bus == 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"PPMC: ERROR: kmalloc() failed\n");
	    rv = -1;
	    /* skip to next bus */
	    continue;
	}
	/* clear the bus data structure */
	bus->busnum = busnum;
	bus->have_master = 0;
	bus->last_digout = 0;
	bus->last_digin = 0;
	bus->last_stepgen = 0;
	bus->last_pwmgen = 0;
	bus->last_DAC = 0;
	bus->last_encoder = 0;
	bus->last_extraDAC = 0;
	/* clear the slot data structures (part of the bus struct) */
	for ( slotnum = 0 ; slotnum < NUM_SLOTS ; slotnum ++ ) {
	    /* clear slot valid flag in bus struct */
	    bus->slot_valid[slotnum] = 0;
	    /* point to slot struct */
	    slot = &(bus->slot_data[slotnum]);
	    /* clear stuff */
	    slot->id = 0;
	    slot->ver = 0;
	    slot->strobe = 0;
	    slot->slot_base = slotnum * SLOT_SIZE;
	    slot->port_addr = port_addr[busnum];
	    slot->read_bitmap = 0;
	    slot->write_bitmap = 0;
	    /* clear EPP read and write caches */
	    for ( n = 0 ; n < 32 ; n++ ) {
		slot->rd_buf[n] = 0;
		slot->wr_buf[n] = 0;
	    }
	    /* clear function pointers */
	    slot->num_rd_functs = 0;
	    slot->num_wr_functs = 0;
	    for ( n = 0 ; n < MAX_FUNCT ; n++ ) {
		slot->rd_functs[n] = NULL;
		slot->wr_functs[n] = NULL;
	    }
	    slot->digout = NULL;
            slot->digin = NULL;
            slot->stepgen = NULL;
            slot->pwmgen = NULL;
            slot->DAC = NULL;
            slot->encoder = NULL;
	    slot->extra_mode = EXTRA_UNUSED;
	    slot->extra = NULL;
	}    
	/* scan the bus */
	for ( slotnum = 0 ; slotnum < NUM_SLOTS ; slotnum ++ ) {
	    /* point to slot struct */
	    slot = &(bus->slot_data[slotnum]);
	    /* rv1 is used to flag errors that fail one bus */
	    rv1 = 0;
	    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: slot %d: ", slotnum);
	
	    /* check slot */
	    idcode = SelRead(slot->slot_base+SLOT_ID_OFFSET, slot->port_addr);
	    if ((idcode == 0)||(idcode == 0xFF)||((idcode&0x0f) == 0x0f)) {
	    // version of 0x0f will not be used so we can detect SLOT_ID_OFFSET
	    //   being left on the bus by a non-implemented device slot
		slot->id = 0;
		slot->ver = 0;
		rtapi_print_msg(RTAPI_MSG_INFO, "nothing detected at addr %x reads %x\n",
				slotnum,idcode);
		ClrTimeout(slot->port_addr);
		/* skip to next slot */
		continue;
	    }
	    rtapi_print_msg(RTAPI_MSG_INFO, "ID code: %02X ", idcode);
	    slot->id = id = idcode & 0xF0;
	    slot->ver = ver = idcode & 0x0F;
	    slot->use_timestamp = 0; /* default is no timestamp */
	    slot->enc_freq = 0; /* default is 1 MHz */
	    /* mark slot as occupied */
	    bus->slot_valid[slotnum] = 1;
	    
	    /* do board specific init and configuration */
	    switch ( id ) {
	    case 0x10:
		boards++;
		//		rtapi_print_msg(RTAPI_MSG_INFO, "PPMC encoder card\n");
		bus_slot_code = (busnum << 4) | slotnum;
		rtapi_print_msg(RTAPI_MSG_INFO, "PPMC encoder card %x\n",bus_slot_code);
		need_timestamp = 0;
		for ( n = 0; n < MAX_BUS*8 ; n++ ) {
		  if ( timestamp[n] == bus_slot_code ) {
		    need_timestamp = 1;
		    timestamp[n] = -1;
		  }
		}
		if ( need_timestamp ) {
		    rv1 += export_timestamp(slot, bus);
		}		
		for ( n = 0; n < MAX_BUS*8 ; n++ ) {
		  if ( (enc_clock[n] & 0xff) == bus_slot_code) {
		    //		    rtapi_print_msg(RTAPI_MSG_ERR,"PPMC detected enc_clock parameter%x\n",enc_clock[n]);
		    if (slot->ver < 4) {
		      rtapi_print_msg(RTAPI_MSG_ERR, 
				      "PPMC encoder does not support adjustable encoder clock, ignoring\n");
		    }
		    slot->enc_freq = (enc_clock[n]) >> 8; // the clock selection is in bits 12-8
		    //		    rtapi_print_msg(RTAPI_MSG_ERR,"PPMC enc_freq=%x\n",slot->enc_freq);
		  }
		}
		// can't export encoder until we know if it uses timestamp
		// and/or enc_clock feature
		rv1 += export_encoders(slot, bus);
		/* encoder ver 4 and above occupy two slot addresses */
		if (slot->ver >= 4) slotnum++;
		break;
	    case 0x20:
		boards++;
		rtapi_print_msg(RTAPI_MSG_INFO, "PPMC DAC card\n");
		rv1 += export_PPMC_DAC(slot, bus);
		break;
	    case 0x30:
		boards++;
		rtapi_print_msg(RTAPI_MSG_INFO, "PPMC Digital I/O card\n");
		rv1 += export_PPMC_digin(slot, bus);
		rv1 += export_PPMC_digout(slot, bus);
		break;
	    case 0x40:
		boards++;
		rtapi_print_msg(RTAPI_MSG_INFO, "Univ. Stepper Controller\n");
		rv1 += export_UxC_digin(slot, bus);
		rv1 += export_UxC_digout(slot, bus);
		rv1 += export_USC_stepgen(slot, bus);
		rv1 += export_encoders(slot, bus);
		bus_slot_code = (busnum << 4) | slotnum;
		need_extra_dac = 0;
		need_extra_dout = 0;
		for ( n = 0; n < MAX_BUS*8 ; n++ ) {
		    if ( extradac[n] == bus_slot_code ) {
			need_extra_dac = 1;
			extradac[n] = -1;
		    }
		    if ( extradout[n] == bus_slot_code ) {
			need_extra_dout = 1;
			extradout[n] = -1;
		    }
		}
		if ( need_extra_dac && need_extra_dout ) {
		    rtapi_print_msg(RTAPI_MSG_ERR,
			"PPMC: ERROR: Can't have extra DAC and DOUT on same slot\n");
		} else if ( need_extra_dac ) {
		    rv1 += export_extra_dac(slot, bus);
		} else if ( need_extra_dout ) {
		    rv1 += export_extra_dout(slot, bus);
		}		
		/* the USC occupies two slots, so skip the second one */
		slotnum++;
		break;
	    case 0x50:
		boards++;
		rtapi_print_msg(RTAPI_MSG_INFO, "Univ. PWM Controller\n");
		rv1 += export_UxC_digin(slot, bus);
		rv1 += export_UxC_digout(slot, bus);
		rv1 += export_UPC_pwmgen(slot, bus);
		bus_slot_code = (busnum << 4) | slotnum;
		need_extra_dac = 0;
		need_extra_dout = 0;
		need_timestamp = 0;
		for ( n = 0; n < MAX_BUS*8 ; n++ ) {
		    if ( extradac[n] == bus_slot_code ) {
			need_extra_dac = 1;
			extradac[n] = -1;
		    }
		    if ( extradout[n] == bus_slot_code ) {
			need_extra_dout = 1;
			extradout[n] = -1;
		    }
		    if ( timestamp[n] == bus_slot_code ) {
			need_timestamp = 1;
			timestamp[n] = -1;
		    }
		}
		if ( need_extra_dac && need_extra_dout ) {
		    rtapi_print_msg(RTAPI_MSG_ERR,
			"PPMC: ERROR: Can't have extra DAC and DOUT on same slot\n");
		} else if ( need_extra_dac ) {
		    rv1 += export_extra_dac(slot, bus);
		} else if ( need_extra_dout ) {
		    rv1 += export_extra_dout(slot, bus);
		}
		if ( need_timestamp ) {
		    rv1 += export_timestamp(slot, bus);
		}		
		// can't export encoders until we know if they use timestamp feature
		rv1 += export_encoders(slot, bus);
		/* the UPC occupies two slots, so skip the second one */
		slotnum++;
		break;
	    default:
	      rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: Check Parallel Port connection.\n");
		/* mark slot as empty */
		bus->slot_valid[slotnum] = 0;
		/* mark bus failed */
		rv1 = -1;
		break;
            }
	    rtapi_print_msg(RTAPI_MSG_INFO,"read cache bitmap: %08x\n", slot->read_bitmap );
	    rtapi_print_msg(RTAPI_MSG_INFO,"write cache bitmap: %08x\n", slot->write_bitmap );
	} /* end of slot loop */
	if ( rv1 != 0 ) {
	    /* error during slot scan, already printed */
	    rv = -1;
	    /* skip to next bus */
	    continue;
	}
	if ( boards == 0 ) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"PPMC: ERROR: no boards found on bus %d, port %04X\n",
		busnum, port_addr[busnum] );
	    rv = -1;
	    /* skip to next bus */
	    continue;
	}
	/* export functions */
	rv1 = hal_export_functf(read_all, &(bus_array[busnum]),
	    1, 0, comp_id, "ppmc.%d.read", busnum);
	if (rv1 != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"PPMC: ERROR: read funct export failed\n");
	    rv = -1;
	    /* skip to next bus */
	    continue;
	}
	rv1 = hal_export_functf(write_all, &(bus_array[busnum]),
	    1, 0, comp_id, "ppmc.%d.write", busnum);
	if (rv1 != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"PPMC: ERROR: write funct export failed\n");
	    rv = -1;
	    /* skip to next bus */
	    continue;
	}
	/* save pointer to bus data */
	bus_array[busnum] = bus;
	rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: bus %d complete\n", busnum);
    }
    for ( n = 0 ; n < MAX_BUS*8 ; n++ ) {
	if ( extradac[n] != -1 ) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"PPMC: ERROR: no USC/UPC for extra dac at bus %d, slot %d\n",
		extradac[n]>>4, extradac[n] & 0x0F );
	    rv = -1;
	}
	if ( extradout[n] != -1 ) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"PPMC: ERROR: no USC/UPC for extra douts at bus %d, slot %d\n",
		extradout[n]>>4, extradout[n] & 0x0F );
	    rv = -1;
	}
    }
    /* restore saved message level */
    rtapi_set_msg_level(msg);
    /* final check for errors */
    if ( rv != 0 ) {
	/* something went wrong, cleanup and exit */
        rtapi_app_exit();
	return rv;
    }    
    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: driver installed\n");
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    int busnum, n, m;
    bus_data_t *bus;

    rtapi_print_msg(RTAPI_MSG_ERR, "PPMC: shutting down\n");
    for ( busnum = 0 ; busnum < MAX_BUS ; busnum++ ) {
	/* check to see if memory was allocated for bus */
	if ( bus_array[busnum] != NULL ) {
	    /* save ptr to memory block */
	    bus = bus_array[busnum];
	    /* mark it invalid so RT code won't access */
	    bus_array[busnum] = NULL;
	    /* want to make sure everything is turned off */
	    /* write zero to the first byte of each slot */
	    for ( n = 0 ; n < 256 ; n += 16 ) {
		SelWrt(0, n, bus->slot_data[0].port_addr);
		/* and to the remainder of the slot */
		for ( m = 1 ; m < 32 ; m++ ) {
		    WrtMore(0, bus->slot_data[0].port_addr);
		}
	    }
	    /* and free the memory block */
	    rtapi_kfree(bus);
	}
    }

    for(busnum = 0; busnum < MAX_BUS; busnum++) {
        /* if ioports were requested, release them */
        hal_parport_release(&port_registration[busnum]);
    }

    /* disconnect from HAL */
    hal_exit(comp_id);
}

/***********************************************************************
*                         REALTIME FUNCTIONS                           *
************************************************************************/

static void read_all(void *arg, long period)
{
    bus_data_t *bus;
    slot_data_t *slot;
    //    int slotnum, functnum, addr_ok;
    int functnum, addr_ok;
    unsigned char n, eppaddr;
    rtapi_u32 bitmap;

    read_period = period;          /* make thread period available to called functions */
    /* get pointer to bus data structure */
    bus = *(bus_data_t **)(arg);
    /* test to make sure it hasn't been freed */
    if ( bus == NULL ) {
	return;
    }
    /* loop thru all slots */
    for ( slotnum = 0 ; slotnum < NUM_SLOTS ; slotnum++ ) {
      currentbus = bus->busnum;  /* make bus in use available for epp_dir logic */
	/* check for anything in slot */
	if ( bus->slot_valid[slotnum] ) {
	    /* point at slot data */
	    slot = &(bus->slot_data[slotnum]);
	    /* We only need to send a latch strobe on the master encoder */
	    if ( slot->strobe == 1 ) {
	      /* set the strobe bit, slave mode */
	      SelWrt(0x20, slot->slot_base + ENCRATE, slot->port_addr);
	      /* repeat to guarantee at least 2uS */
	      SelWrt(0x20, slot->slot_base + ENCRATE, slot->port_addr);
	      /* end of strobe pulse, stay in slave mode */
	      SelWrt(0x00, slot->slot_base + ENCRATE, slot->port_addr);
	    }
	    /* fetch data from EPP to cache */
	    addr_ok = 0;
	    bitmap = slot->read_bitmap;
	    n = 0;
	    while ( bitmap ) {
		if ( bitmap & 1 ) {
		    /* need to read register 'n' */
		    if ( addr_ok ) {
			/* auto-increment address is usable */
			slot->rd_buf[n] = ReadMore(slot->port_addr);
		    } else {
			/* need to specify address */
			eppaddr = slot->slot_base + n;
			/* send address and read byte */
			slot->rd_buf[n] = SelRead(eppaddr, slot->port_addr);
			/* mark auto-incrementing address as valid */
			addr_ok = 1;
		    }
		} else {
		    /* don't need to read this register */
		    /* mark auto-incrementing address as invalid */
		    addr_ok = 0;
		}
		/* next register */
		n++;
		bitmap >>= 1;
	    }
	    /* loop thru all functions associated with slot */
	    for ( functnum = 0 ; functnum < slot->num_rd_functs ; functnum++ ) {
		/* call function */
		(slot->rd_functs[functnum])(slot);
	    }
	}
    }
}

static void write_all(void *arg, long period)
{
    (void)period;
    bus_data_t *bus;
    slot_data_t *slot;
    int slotnum, functnum, addr_ok;
    unsigned char n, eppaddr;
    rtapi_u32 bitmap;

    /* get pointer to bus data structure */
    bus = *(bus_data_t **)(arg);
    /* test to make sure it hasn't been freed */
    if ( bus == NULL ) {
	return;
    }
    /* loop thru all slots */
    for ( slotnum = 0 ; slotnum < NUM_SLOTS ; slotnum++ ) {
	/* check for anything in slot */
	if ( bus->slot_valid[slotnum] ) {
	  currentbus = bus->busnum;  /* make bus in use available for epp_dir logic */
	    /* point at slot data */
	    slot = &(bus->slot_data[slotnum]);
	    /* loop thru all functions associated with slot */
	    for ( functnum = 0 ; functnum < slot->num_wr_functs ; functnum++ ) {
		/* call function */
		(slot->wr_functs[functnum])(slot);
	    }
	    /* write data from cache to EPP */
	    addr_ok = 0;
	    bitmap = slot->write_bitmap;
	    n = 0;
	    while ( bitmap ) {
		if ( bitmap & 1 ) {
		    /* need to write data register 'n' */
		    if ( addr_ok ) {
			/* auto-increment address is usable */
			WrtMore(slot->wr_buf[n], slot->port_addr);
		    } else {
			/* need to specify address */
			eppaddr = slot->slot_base + n;
			/* send address and write byte */
			SelWrt(slot->wr_buf[n], eppaddr, slot->port_addr);
			/* mark auto-incrementing address as valid */
			addr_ok = 1;
		    }
		} else {
		    /* don't need to write this one */
		    /* mark auto-incrementing address as invalid */
		    addr_ok = 0;
		}
		/* next register */
		n++;
		bitmap >>= 1;
	    }
	}
    }
}

static void read_digins(slot_data_t *slot)
{
    int b;
    unsigned char indata, mask;

    /* read the first 8 inputs */
    indata = slot->rd_buf[UxC_DINA];
    /* split the bits into 16 variables (8 regular, 8 inverted) */
    b = 0;
    mask = 0x01;
    while ( b < 8 ) {
	*(slot->digin[b].data) = indata & mask;
	*(slot->digin[b].data_not) = !(indata & mask);
	mask <<= 1;
	b++;
    }
    /* read the next 8 inputs */
    indata = slot->rd_buf[UxC_DINB];
    /* and split them too */
    mask = 0x01;
    while ( b < 16 ) {
	*(slot->digin[b].data) = indata & mask;
	*(slot->digin[b].data_not) = !(indata & mask);
	mask <<= 1;
	b++;
    }
}

static void write_digouts(slot_data_t *slot)
{
    int b;
    unsigned char outdata, mask;

    outdata = 0x00;
    mask = 0x01;
    /* assemble output byte from 8 source variables */
    for (b = 0; b < 8; b++) {
	/* get the data, add to output byte */
	if ((*(slot->digout[b].data)) && (!slot->digout[b].invert)) {
	    outdata |= mask;
	}
	if ((!*(slot->digout[b].data)) && (slot->digout[b].invert)) {
	    outdata |= mask;
	}
	mask <<= 1;
    }
    /* write it to the hardware (cache) */
    slot->wr_buf[UxC_DOUTA] = outdata;
}

static void read_PPMC_digins(slot_data_t *slot)
{
    int b;
    unsigned char indata, mask;

    //    rtapi_print_msg(RTAPI_MSG_INFO, "enter read_digins()\n");
    /* read the first 8 inputs */
    indata = slot->rd_buf[DIO_DINA];
    /* split the bits into 16 variables (8 regular, 8 inverted) */
    b = 0;
    mask = 0x01;
    while ( b < 8 ) {
	*(slot->digin[b].data) = indata & mask;
	*(slot->digin[b].data_not) = !(indata & mask);
	mask <<= 1;
	b++;
    }
    /* read the next 8 inputs */
    indata = slot->rd_buf[DIO_DINB];
    /* and split them too */
    mask = 0x01;
    while ( b < 16 ) {
	*(slot->digin[b].data) = indata & mask;
	*(slot->digin[b].data_not) = !(indata & mask);
	mask <<= 1;
	b++;
    }
    if (slot->digin[b].data != NULL) {
      /* read the 2 Estop-related inputs */
      indata = slot->rd_buf[DIO_ESTOP_IN];
      /* and split them too */
      mask = 0x01;
      while ( b < 18 ) {
	*(slot->digin[b].data) = indata & mask;
	*(slot->digin[b].data_not) = !(indata & mask);
	mask <<= 1;
	b++;
      }
    }
}

static void write_PPMC_digouts(slot_data_t *slot)
{
    int b;
    unsigned char outdata, mask;

    //    rtapi_print_msg(RTAPI_MSG_INFO, "enter write_PPMC_digouts()\n");
    outdata = 0x00;
    mask = 0x01;
    /* assemble output byte from 8 source variables */
    for (b = 0; b < 8; b++) {
	/* get the data, add to output byte */
	if ((*(slot->digout[b].data)) && (!slot->digout[b].invert)) {
	    outdata |= mask;
	}
	if ((!*(slot->digout[b].data)) && (slot->digout[b].invert)) {
	    outdata |= mask;
	}
	mask <<= 1;
    }
    /* write it to the hardware (cache) */
    slot->wr_buf[DIO_DOUTA] = outdata;
    if (slot->digout[8].data != NULL) {  // no estop funct on slave boards - hal pin doesn't exist
      outdata = 0;  // now process estop bit
      if ((*(slot->digout[8].data)) && (!slot->digout[8].invert)) {
	outdata =1;
      }
      if ((!*(slot->digout[8].data)) && (slot->digout[8].invert)) {
	outdata |= 1;
      }
      slot->wr_buf[DIO_ESTOP_OUT] = outdata;
    }
    else slot->wr_buf[DIO_ESTOP_OUT] = 2;  // force 2 to set additional boards to slave
}

static void read_encoders(slot_data_t *slot)
{
  int i, byteindex, byteindx2;
  double vel;                    // local temporary velocity
    union pos_tag {
      hal_s32_t l;              // JE001
        struct byte_tag {
            signed char b0;
            signed char b1;
            signed char b2;
            signed char b3;
        } byte;
    } pos, oldpos;
    union time_tag {
      unsigned short s;
      struct byte2_tag {
	unsigned char b0;
	unsigned char b1;
      } byte;
    } timebase, timestamp;
    unsigned short delta_time;
    //    hal_u32_t timestamp;
    //    hal_u32_t timebase;
      
    // sample timebase only on boards so equipped
    if (slot->use_timestamp) {
      byteindex = ENCTB;
      timebase.byte.b0 = (unsigned char)slot->rd_buf[byteindex++];
      timebase.byte.b1 = (unsigned char)slot->rd_buf[byteindex];
    }
    byteindex = ENCCNT0;        /* first encoder count register */
    byteindx2 = ENCTS;          /* first encoder timestamp register */
    for (i = 0; i < 4; i++) {
      slot->encoder[i].indrescnt++;  /* increment counter each servo cycle */
        oldpos.l = slot->encoder[i].oldreading;
	pos.byte.b0 = (signed char)slot->rd_buf[byteindex++];
	pos.byte.b1 = (signed char)slot->rd_buf[byteindex++];
	pos.byte.b2 = (signed char)slot->rd_buf[byteindex++];
        pos.byte.b3 = oldpos.byte.b3;
        /* check for - to + transition */
        if ((oldpos.byte.b2 & 0xc0) == 0xc0 && (pos.byte.b2 == 0))
            pos.byte.b3++;
        else
            if ((oldpos.byte.b2 == 0) && (pos.byte.b2 & 0xc0) == 0xc0)
                pos.byte.b3--;
	*(slot->encoder[i].delta) = pos.l - slot->encoder[i].oldreading;
	vel = (pos.l - slot->encoder[i].oldreading) /
	           (read_period * 1e-9 * slot->encoder[i].scale);
	/* index processing */
	if ( (slot->rd_buf[ENCISR] & ( 1 << i )) != 0 ) {
	  //	  rtapi_print_msg(RTAPI_MSG_INFO, "index seen for axis %d",i);
	  //	  rtapi_print_msg(RTAPI_MSG_INFO, "indrescnt %d\n",slot->encoder[i].indrescnt);
	    /* index edge occurred since last time this code ran */
	    *(slot->encoder[i].index) = 1;
	    /* index-enable only works on version 2 and up */
	    if (slot->ver >= 2) {
		/* were we looking for an index edge? */
		if ( ((slot->encoder[0].indres & ( 1 << i )) != 0) &&
		     (slot->encoder[i].indrescnt > 3)) {
		    /* yes, clear index-enable to announce that we found it */
		    *(slot->encoder[i].index_enable) = 0;
		    /* need to properly set the 24->32 bit extension byte */
		    if ( pos.byte.b2 < 0 ) {
		      /* going backwards */
		      pos.byte.b3 = 0xFF;
		    } else {
		      pos.byte.b3 = 0;
		    }
		    oldpos.byte.b3 = pos.byte.b3;
		}
	    }
	} else {
	  /* no index edge since last check */
	  *(slot->encoder[i].index) = 0;
	}
	slot->encoder[i].oldreading = pos.l;
	*(slot->encoder[i].count) = pos.l;
	if (slot->encoder[i].scale < 0.0) {
	  if (slot->encoder[i].scale > -EPSILON)
	    slot->encoder[i].scale = -1.0;
	} else {
	  if (slot->encoder[i].scale < EPSILON)
	    slot->encoder[i].scale = 1.0;
	}
	*(slot->encoder[i].position) = pos.l / slot->encoder[i].scale;
	// perform velocity estimate when hardware provides timestamps
	if (slot->use_timestamp) {
	  slot->encoder[i].old_timestamp = slot->encoder[i].timestamp;
	  timestamp.byte.b0 = slot->rd_buf[byteindx2++];
	  timestamp.byte.b1 = slot->rd_buf[byteindx2++];
	  slot->encoder[i].timestamp = timestamp.s;
	  // one or more counts this sample
	  if (*(slot->encoder[i].delta) != 0.0) {
	    delta_time = timestamp.s - slot->encoder[i].old_timestamp;
	    delta_time = delta_time & 0xffff;
	    if (slot->encoder[i].counts_since_timeout < 2) {
	      // just keep simple vel calc from above
	      slot->encoder[i].counts_since_timeout++;
	      *(slot->encoder[i].vel) = vel;  // cannot make estimate
	    } else {
	      vel = *(slot->encoder[i].delta) / (delta_time * 1e-6 * slot->encoder[i].scale);
	      *(slot->encoder[i].vel) = vel;
	    }
	    if (((slot->encoder[i].prevdir > 0) && (*(slot->encoder[i].delta) < 0)) ||
		((slot->encoder[i].prevdir < 0) && (*(slot->encoder[i].delta) > 0))) {
	      *(slot->encoder[i].vel) = 0.0;    /* suppress velocity of dithering encoder at reversal */
	    }
	  } else {
	    // no counts this sample
	    if (slot->encoder[i].counts_since_timeout) {
	      delta_time = timebase.s - timestamp.s;	
	      delta_time = delta_time & 0xffff;
	      //  if (delta_time < slot->encoder[i].scale * slot->encoder[i].min_speed) {
	      if (delta_time < 65500) {
		// 1e-6 is timebase period
		vel = 1.0 / (slot->encoder[i].scale * delta_time * 1e-6);
		if (vel < 0.0) vel = -vel;
		if (vel < *(slot->encoder[i].vel)) {
		  *(slot->encoder[i].vel) = vel;
		}
		if (-vel > *(slot->encoder[i].vel)) {
		  *(slot->encoder[i].vel) = -vel;
		}
	      } else {
		slot->encoder[i].counts_since_timeout = 0;
		*(slot->encoder[i].vel) = 0;
	      }
	    } else {
	      *(slot->encoder[i].vel) = 0;
	    }
	  }
	} else {
	  *(slot->encoder[i].vel) = vel;  // encoder without timestamp
	}
	if (*(slot->encoder[i].delta) > 0) slot->encoder[i].prevdir = 1;  // mark last direction moved
	if (*(slot->encoder[i].delta) < 0) slot->encoder[i].prevdir = -1;
    }
}

/* I can see the puzzled look on your face now.  Why do we need
   a write function for encoders?  You don't write to encoders...
   Well, you do write to the index latching hardware.
*/

static void write_encoders(slot_data_t *slot)
{
    int i;

    if ( slot->ver < 2 ) {
	/* no index support in old boards */
	return;
    }
    for (i = 0; i < 4; i++) {
	if ( *(slot->encoder[i].index_enable) ) {
	    /* all 4 control bits are packed into the same register */
	  if ((slot->encoder[0].indres & (1 << i)) == 0) {
	    slot->encoder[i].indrescnt = 0; /* clear counter first time only */
	    /* set bit to force reset on index pulse */
	    (slot->encoder[0].indres) |= (1 << i);
	  }
	} else {
	    /* clear bit to ignore index pulses */
	    (slot->encoder[0].indres) &= ~(1 << i);
	}
    }
    /* put the control bits in cache for write to hardware */
    slot->wr_buf[ENCINDX] = slot->encoder[0].indres;
}


/* fetch a time parameter (in nS), make sure it is a multiple
   of 100nS, and is between min_ns and 25.4uS, and return the
   value in 10MHz clock pulses. */
static unsigned int ns2cp( hal_u32_t *pns, unsigned int min_ns )
{
    unsigned ns, cp;

    ns = *pns;
    if ( ns < min_ns ) ns = min_ns;
    if ( ns > 25400 ) ns = 25400;
    cp = ns / 100;
    ns = cp * 100;
    *pns = ns;
    return cp;
}


static void write_stepgens(slot_data_t *slot)
{
    int n, reverse, run, pulse_width, pulse_space, setup_time;
    unsigned int divisor;
    stepgen_t *sg;
    double bd_max_freq, ch_max_freq, abs_scale, freq;
    unsigned char control_byte;

    /* pulse width cannot be less than 200nS (HW limit) */
    pulse_width = ns2cp(&(slot->stepgen->pulse_width_ns), 200);
    /* write pulse width to the cache, inverted */
    slot->wr_buf[RATE_WIDTH_0] = 256 - pulse_width;
    /* pulse space cannot be less than 300nS (HW limitation) */
    pulse_space = ns2cp(&(slot->stepgen->pulse_space_ns), 300);
    /* setup time cannot be less than 2 (HW limit) */
    setup_time = ns2cp(&(slot->stepgen->setup_time_ns), 200);
    /* write it to the cache, inverted */
    slot->wr_buf[RATE_SETUP_0] = 256 - setup_time;
    /* calculate the max frequency, varies with pulse width and
       min pulse spacing */
    bd_max_freq = 10000000.0 / (pulse_width + pulse_space);
    /* now do the four individual stepgens */
    control_byte = 0;
    for ( n = 0 ; n < 4 ; n++ ) {
	/* point to the specific stepgen */
	sg = &(slot->stepgen->sg[n]);
	/* validate the scale value */
	if ( sg->scale < 0.0 ) {
	    if ( sg->scale > -EPSILON ) {
		/* too small, divide by zero is bad */
		sg->scale = -1.0;
	    }
	    abs_scale = -sg->scale;
	} else {
	    if ( sg->scale < EPSILON ) {
		sg->scale = 1.0;
	    }
	    abs_scale = sg->scale;
	}
	ch_max_freq = bd_max_freq;
	/* check for user specified max velocity */
	if (sg->max_vel <= 0.0) {
	    /* set to zero if negative, and ignore if zero */
	    sg->max_vel = 0.0;
	} else {
	    /* parameter is non-zero and positive, compare to max_freq */
	    if ( (sg->max_vel * abs_scale) > ch_max_freq) {
		/* parameter is too high, lower it */
		sg->max_vel = ch_max_freq / abs_scale;
	    } else {
		/* lower max_freq to match parameter */
		ch_max_freq = sg->max_vel * abs_scale;
	    }
	}
	/* calculate desired frequency */
	freq = *(sg->vel) * sg->scale;
	/* should we be running? */
	if ( *(sg->enable) != 0 ) {
	    run = 1;
	} else {
	    run = 0;
	}
	/* deal with special cases - negative and very low frequency */
	reverse = 0;
	if ( freq < 0.0 ) {
	    /* negative */
	    freq = -freq;
	    reverse = 1;
	}
	/* apply limits */
	if ( freq > ch_max_freq ) {
	    freq = ch_max_freq;
	    divisor = 10000000.0 / freq;
	} else if ( freq < (10000000.0/16777215.0) ) {
	    /* frequency would result in a divisor greater than 2^24-1 */
	    freq = 0.0;
	    divisor = 16777215;
	    /* only way to get zero is to turn it off */
	    run = 0;
	} else {
	    /* calculate divisor, round to nearest instead of truncating */
	    divisor = ( 10000000.0 / freq ) + 0.5;
	    /* calculate actual frequency (due to divisor roundoff) */
	    freq = 10000000.0 / divisor;
	}
	/* set run bit in the control byte */
	control_byte >>= 2;
	if ( run ) {
	    control_byte |= 0x80;
	}
	/* set dir bit in the control byte, and save the frequency */
	if ( reverse ) {
	    sg->freq = -freq;
	} else {
	    sg->freq = freq;
	    control_byte |= 0x40;
	}
	/* correct for an offset of 4 in the hardware */
	divisor -= 4;
	/* write divisor to the cache */
	slot->wr_buf[RATE_GEN_0+(n*3)] = divisor & 0xff;
	divisor >>= 8;
	slot->wr_buf[RATE_GEN_0+(n*3)+1] = divisor & 0xff;
	divisor >>= 8;
	slot->wr_buf[RATE_GEN_0+(n*3)+2] = divisor & 0xff;
    }
    /* write control byte to cache */
    slot->wr_buf[RATE_CTRL_0] = control_byte;
}


static void write_pwmgens(slot_data_t *slot)
{
    int n, reverse;
    unsigned int period, start, len, stop;
    pwmgen_t *pg;
    double freq, dc, abs_dc;
    unsigned char control_byte;

    /* zero frequency is a special case, turn off everything */
    if ( slot->pwmgen->freq == 0.0 ) {
	slot->pwmgen->old_freq = slot->pwmgen->freq;
	/* write control byte to cache */
	slot->wr_buf[PWM_CTRL_0] = 0;
	/* done */
	return;
    }
    /* check for new frequency setting */
    if ( slot->pwmgen->freq != slot->pwmgen->old_freq ) {
	/* process new frequency value */
	freq = slot->pwmgen->freq;
	/* frequency must be between 153Hz and 500KHz */
	if ( freq < 153.0 ) {
	    freq = 153.0;
	}
	if ( freq > 500000.0 ) {
	    freq = 500000.0;
	}
	/* calculate divisor */
	if (slot->ver >= 3)  // accommodate 3.1 and newer boards with 40MHz clk
	  period = (40000000.0 / freq) + 0.5;  // 40 MHz clock on ver 3 and up
	else
	  period = (10000000.0 / freq) + 0.5;  // 10 MHz on lower version
	/* calculate actual frequency (after rounding, etc) */
	freq = 10000000.0 / period;
	/* save values */
	slot->pwmgen->freq = freq;
	slot->pwmgen->old_freq = freq;
	slot->pwmgen->period = period;
	slot->pwmgen->period_recip = 1.0 / period;
    }
    /* calculate counter start value */
    start = 65536 - slot->pwmgen->period;
    /* write to cache */
    slot->wr_buf[PWM_FREQ_LO] = start & 0xFF;
    slot->wr_buf[PWM_FREQ_HI] = (start >> 8) & 0xFF;
    /* now do the four individual pwmgens */
    control_byte = 0;
    for ( n = 0 ; n < 4 ; n++ ) {
	/* point to the specific pwm generator */
	pg = &(slot->pwmgen->pg[n]);
	/* validate the scale value */
	if ( pg->scale < 0.0 ) {
	    if ( pg->scale > -EPSILON ) {
		/* too small, divide by zero is bad */
		pg->scale = -1.0;
	    }
	} else {
	    if ( pg->scale < EPSILON ) {
		pg->scale = 1.0;
	    }
	}
	/* calculate desired duty cycle */
	dc = *(pg->value) / pg->scale;
	/* Special code to deal with the requirements of the Pico PWM
	   amps.  They need at least one PWM pulse in each direction 
	   every time you enable the amps.  So we override the commanded
	   duty cycle with +5%, then -5%, for one thread execution time 
	   each, when we see a rising edge on enable.
	*/
	if ( pg->bootstrap != 0 ) {
	    /* check for rising edge on enable */
	    if (( *(pg->enable) != 0 ) && ( pg->old_enable == 0 )) {
		/* kick off state machine */
		pg->boot_state = BOOT_FWD;
	    }
	    pg->old_enable = *(pg->enable);
	    /* now execute a state machine */
	    switch(pg->boot_state) {
	    case BOOT_NORMAL:
		break;
	    case BOOT_REV:
		dc = -0.05;
		pg->boot_state = BOOT_NORMAL;
		break;
	    case BOOT_FWD:
		dc =  0.05;
		pg->boot_state = BOOT_REV;
		break;
	    default:
		pg->boot_state = BOOT_NORMAL;
		break;
	    }
	}
	/* deal with negative values */
	reverse = 0;
	if ( dc < 0.0 ) {
	    reverse = 1;
	    abs_dc = -dc;
	} else {
	    abs_dc = dc;
	}
	/* reset any illegal duty cycle limits */
	if (( pg->min_dc > 1.0 ) || ( pg->min_dc < 0.0 )) {
	    pg->min_dc = 0.0;
	} 
	if (( pg->max_dc > 1.0 ) || ( pg->max_dc < 0.0 )) {
	    pg->max_dc = 1.0;
	} 
	if ( pg->min_dc >= pg->max_dc ) {
	    pg->min_dc = 0.0;
	    pg->max_dc = 1.0;
	}
	/* apply limits */
	if ( abs_dc > pg->max_dc ) {
	    abs_dc = pg->max_dc;
	} else if ( abs_dc < pg->min_dc ) {
	    abs_dc = pg->min_dc;
	}
	/* calculate length of PWM pulse in clocks */
	len = ( abs_dc * slot->pwmgen->period ) + 0.5;
	/* calculate actual duty cycle (after rounding) */
	abs_dc = len * slot->pwmgen->period_recip;
	/* set run bit in the control byte */
	control_byte >>= 2;
	if ( *(pg->enable) != 0 ) {
	    control_byte |= 0x80;
	}
	/* set dir bit in the control byte, and save the duty cycle */
	if ( reverse ) {
	    pg->duty_cycle = -abs_dc;
	} else {
	    pg->duty_cycle = abs_dc;
	    control_byte |= 0x40;
	}
	/* calculate count at which to turn off output */
	stop = 65535 - len;
	/* write count to the cache */
	slot->wr_buf[PWM_GEN_0+(n*2)] = stop & 0xff;
	stop >>= 8;
	slot->wr_buf[PWM_GEN_0+(n*2)+1] = stop & 0xff;
    }
    /* write control byte to cache */
    slot->wr_buf[PWM_CTRL_0] = control_byte;
}

static void write_DACs(slot_data_t *slot)
{
    int n;
    DAC_t *pg;
    long dc;
    double volts;

    //    rtapi_print_msg(RTAPI_MSG_INFO, "enter write_DACs()\n");
    /* now do the four individual DACs */
    for ( n = 0 ; n < 4 ; n++ ) {
      /* point to the specific DAC */
      pg = &(slot->DAC->pg[n]);
      /* validate the scale value */
      if ( pg->scale < 0.0 ) {
	if ( pg->scale > -EPSILON ) {
	  /* too small, divide by zero is bad */
	  pg->scale = -1.0;
	}
      } else {
	if ( pg->scale < EPSILON ) {
	  pg->scale = 1.0;
	}
      }
      /* calculate desired output voltage */
      volts = *(pg->value) / pg->scale;
      /* output to DAC word works like:
	 0xFFFF -> +10 V
	 0x8000 ->   0 V
	 0x0000 -> -10 V   */
      dc = (long) (((volts / 10.0) * 0x7FFF)+0x8000);
      if (dc > 0xffff)
	{
	  dc = 0xffff;
	}
      if (dc < 0 )
	{
	  dc = 0;
	}
      slot->wr_buf[DAC_0+(n*2)] = dc & 0xff;  // put low byte in cache
      dc >>= 8;
      slot->wr_buf[DAC_0+(n*2)+1] = dc & 0xff;  // put high byte in cache
    }
}


static void write_extraDAC(slot_data_t *slot)
{
    DAC_t *pg;
    long dc;
    double volts;

    /* point to the DAC */
    pg = &(slot->extra->dac);
    /* validate the scale value */
    if ( pg->scale < 0.0 ) {
	if ( pg->scale > -EPSILON ) {
	    /* too small, divide by zero is bad */
	    pg->scale = -1.0;
	}
    } else {
	if ( pg->scale < EPSILON ) {
	    pg->scale = 1.0;
	}
    }
    /* calculate desired output voltage */
//    volts = *(pg->value) / pg->scale;
    /* this DAC is sign-magnitude, and the sign enable
       bits are controlled by digital output bits.
       driving P8 pin 12 to ground enables + output.
       driving P8 pin 11 to ground enables - output.
       leaving both pins high or floating forces output to 0 V.
       don't drive both outputs low at the same time - undefined.
       UPC and USC boards sold with the spindle DAC option
       have SSR1 set up for + output, and SSR2 for - output.  */

    volts = *(pg->value) / pg->scale;
    if (volts < 0.0 ) volts = -volts;  // no fabs function available!!
    /* output to DAC word works like:
	0xFF -> +10 V
	0x00 ->   0 V */
    /* there is NO negative.  You can invert the signal with
	an external signal into the post-DAC amplifier, but
	the software is completely unaware of that */
    dc = (long) ((volts / 10.0) * 0xff);
    if (dc > 0xff) {
	dc = 0xff;
    }
    if (dc < 0 ) {
	dc = 0;
    }
    slot->wr_buf[UxC_EXTRA] = dc & 0xff;  // put in cache
}

static void write_extra_dout(slot_data_t *slot)
{
    dout_t *pg;
    int b;
    unsigned char outdata, mask;

    //    rtapi_print_msg(RTAPI_MSG_INFO, "enter write_extra_dout()\n");
    outdata = 0x00;
    mask = 0x01;
    /* assemble output byte from 8 source variables */
    for (b = 0; b < 8; b++) {
      pg = &(slot->extra->douts[b]);
	/* get the data, add to output byte */
	if ((*(pg->data)) && (!pg->invert)) {
	    outdata |= mask;
	}
	if ((!*(pg->data)) && (pg->invert)) {
	    outdata |= mask;
	}
	mask <<= 1;
    }
    slot->wr_buf[UxC_EXTRA] = outdata;  // put in cache
}


/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

/* this function converts a range of EPP addresses into a bitmap
   to be passed to add_rd_funct() or add_wr_funct()
*/

static rtapi_u32 block(int min, int max)
{
    int n;
    rtapi_u32 mask;

    mask = 0;
    for ( n = min ; n <= max ; n++ ) {
	mask |= ( 1u << n );
    }
    return mask;
}

/* these functions are used to register a runtime function to be called
   by either read_all or write_all.  'cache_bitmap' defines the EPP
   addresses that the function needs.  All addresses needed by all 
   functions associated with the slot will be sequentially read into
   the rd_buf cache (or written from the wr_buf cache) by read_all 
   or write_all respectively, to minimize the number of slow inb
   and outb operations needed.
*/

static int add_rd_funct(slot_funct_t *funct, slot_data_t *slot,
			rtapi_u32 cache_bitmap )
{
    if ( slot->num_rd_functs >= MAX_FUNCT ) {
	rtapi_print_msg(RTAPI_MSG_ERR, 
	    "PPMC: ERROR: too many read functions\n");
	return -1;
    }
    slot->rd_functs[slot->num_rd_functs++] = funct;
    slot->read_bitmap |= cache_bitmap;
    return 0;
}

static int add_wr_funct(slot_funct_t *funct, slot_data_t *slot,
			rtapi_u32 cache_bitmap )
{
    if ( slot->num_wr_functs >= MAX_FUNCT ) {
	rtapi_print_msg(RTAPI_MSG_ERR, 
	    "PPMC: ERROR: too many write functions\n");
	return -1;
    }
    slot->wr_functs[slot->num_wr_functs++] = funct;
    slot->write_bitmap |= cache_bitmap;
    return 0;
}

static int export_UxC_digin(slot_data_t *slot, bus_data_t *bus)
{
    int retval, n;

    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  exporting UxC digital inputs\n");

    /* do hardware init */

    /* allocate shared memory for the digital input data */
    slot->digin = hal_malloc(16 * sizeof(din_t));
    if (slot->digin == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: hal_malloc() failed\n");
	return -1;
    }
    for ( n = 0 ; n < 16 ; n++ ) {
	/* export pins for input data */
	retval = hal_pin_bit_newf(HAL_OUT, &(slot->digin[n].data), comp_id,
				  "ppmc.%d.din.%02d.in", bus->busnum, bus->last_digin);
	if (retval != 0) {
	    return retval;
	}
	retval = hal_pin_bit_newf(HAL_OUT, &(slot->digin[n].data_not), comp_id,
				  "ppmc.%d.din.%02d.in-not", bus->busnum, bus->last_digin);
	if (retval != 0) {
	    return retval;
	}
	/* increment number to prepare for next output */
	bus->last_digin++;
    }
    add_rd_funct(read_digins, slot, block(UxC_DINA, UxC_DINB));
    return 0;
}

static int export_UxC_digout(slot_data_t *slot, bus_data_t *bus)
{
    int retval, n;

    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  exporting UxC digital outputs\n");

    /* do hardware init */
    /* turn off all outputs */
    SelWrt(0, slot->slot_base+UxC_DOUTA, slot->port_addr);
    if (bus->last_digout > 7) {   // if not first UxC, set it to slave mode
      rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  slave UxC addr %x\n",slot->slot_base+UxC_SLAVE);
      SelWrt(1,slot->slot_base+UxC_SLAVE,slot->port_addr);
      rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  slave UxC # %d\n",bus->last_digout);
    }
    /* allocate shared memory for the digital output data */
    slot->digout = hal_malloc(8 * sizeof(dout_t));
    if (slot->digout == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: hal_malloc() failed\n");
	return -1;
    }
    for ( n = 0 ; n < 8 ; n++ ) {
	/* export pin for output data */
	retval = hal_pin_bit_newf(HAL_IN, &(slot->digout[n].data), comp_id,
				  "ppmc.%d.dout.%02d.out", bus->busnum, bus->last_digout);
	if (retval != 0) {
	    return retval;
	}
	/* export parameter for inversion */
	retval = hal_param_bit_newf(HAL_RW, &(slot->digout[n].invert), comp_id,
				    "ppmc.%d.dout.%02d-invert", bus->busnum, bus->last_digout);
	if (retval != 0) {
	    return retval;
	}
	slot->digout[n].invert = 0;
	/* increment number to prepare for next output */
	bus->last_digout++;
    }
    add_wr_funct(write_digouts, slot, block(UxC_DOUTA, UxC_DOUTA));
    return 0;
}

static int export_PPMC_digin(slot_data_t *slot, bus_data_t *bus)
{
    int retval, n;

    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  exporting PPMC digital inputs\n");

    /* do hardware init */

    /* allocate shared memory for the digital input data */
    slot->digin = hal_malloc(18 * sizeof(din_t));  // 18 inputs per unit
    if (slot->digin == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: hal_malloc() failed\n");
	return -1;
    }
    for ( n = 0 ; n < 16 ; n++ ) {
	/* export pins for input data */
	retval = hal_pin_bit_newf(HAL_OUT, &(slot->digin[n].data), comp_id,
				  "ppmc.%d.din.%02d.in", bus->busnum, bus->last_digin);
	if (retval != 0) {
	    return retval;
	}
	retval = hal_pin_bit_newf(HAL_OUT, &(slot->digin[n].data_not), comp_id,
				  "ppmc.%d.din.%02d.in-not", bus->busnum, bus->last_digin);
	if (retval != 0) {
	    return retval;
	}
	/* increment number to prepare for next output */
	bus->last_digin++;
    }
    if (bus->last_digin < 31) {
	retval = hal_pin_bit_newf(HAL_OUT, &(slot->digin[16].data), comp_id,
				  "ppmc.%d.din.estop.in", bus->busnum);
	if (retval != 0) {
	    return retval;
	}
	retval = hal_pin_bit_newf(HAL_OUT, &(slot->digin[16].data_not), comp_id,
				  "ppmc.%d.din.estop.in-not", bus->busnum);
	if (retval != 0) {
	    return retval;
	}
	retval = hal_pin_bit_newf(HAL_OUT, &(slot->digin[17].data), comp_id,
				  "ppmc.%d.din.fault.in", bus->busnum);
	if (retval != 0) {
	    return retval;
	}
	retval = hal_pin_bit_newf(HAL_OUT, &(slot->digin[17].data_not), comp_id,
				  "ppmc.%d.din.fault.in-not", bus->busnum);
	if (retval != 0) {
	    return retval;
	}
    add_rd_funct(read_PPMC_digins, slot, block(DIO_DINA, DIO_ESTOP_IN));
    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  exporting as MASTER D In\n");
    }
    else {
      add_rd_funct(read_PPMC_digins, slot, block(DIO_DINA, DIO_DINB));
      rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  exporting as SLAVE D In\n");
    }
    return 0;
}

static int export_PPMC_digout(slot_data_t *slot, bus_data_t *bus)
{
    int retval, n;

    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  exporting PPMC digital outputs\n");

    /* do hardware init */
    /* turn off all outputs */
    SelWrt(0, slot->slot_base+DIO_DOUTA, slot->port_addr);
    if (bus->last_digout > 7) {   // if not first PPMC DIO, set it to slave mode
      rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  slave DIO addr %x\n",slot->slot_base+DIO_ESTOP_OUT);
      SelWrt(2,slot->slot_base+DIO_ESTOP_OUT,slot->port_addr);
      rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  slave DIO # %d\n",bus->last_digout);
    }

    /* allocate shared memory for the digital output data */
    slot->digout = hal_malloc(9 * sizeof(dout_t));              // 8 outputs per board + estop
    if (slot->digout == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: hal_malloc() failed\n");
	return -1;
    }
    for ( n = 0 ; n < 8 ; n++ ) {
	/* export pin for output data */
	retval = hal_pin_bit_newf(HAL_IN, &(slot->digout[n].data), comp_id,
				  "ppmc.%d.dout.%02d.out", bus->busnum, bus->last_digout);
	if (retval != 0) {
	    return retval;
	}
	/* export parameter for inversion */
	retval = hal_param_bit_newf(HAL_RW, &(slot->digout[n].invert), comp_id,
				    "ppmc.%d.dout.%02d.invert", bus->busnum, bus->last_digout);
	if (retval != 0) {
	    return retval;
	}
	slot->digout[n].invert = 0;
	/* increment number to prepare for next output */
	bus->last_digout++;
    }
	/* export pin for E-Stop control */
    if (bus->last_digout < 15) {                // only on first DIO board
      rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  master DIO at # %d\n",bus->last_digout);
      retval = hal_pin_bit_newf(HAL_IN, &(slot->digout[8].data), comp_id,
				"ppmc.%d.dout.Estop.out", bus->busnum);
      if (retval != 0) {
	return retval;
      }
      /* export parameter for inversion */
      retval = hal_param_bit_newf(HAL_RW, &(slot->digout[8].invert), comp_id,
				  "ppmc.%d.dout.Estop.invert", bus->busnum);
      if (retval != 0) {
	return retval;
      }
      slot->digout[8].invert = 0;
      add_wr_funct(write_PPMC_digouts, slot, block(DIO_DOUTA, DIO_ESTOP_OUT));
      rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  exporting as MASTER D Out\n");
    }
    else {
      add_wr_funct(write_PPMC_digouts, slot, block(DIO_DOUTA, DIO_DOUTA));
      //add_wr_funct(write_PPMC_digouts, slot, DIO_DOUTA, DIO_ESTOP_OUT); // hack to keep slave boards in slave
      rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  exporting as SLAVE D Out\n");
      SelWrt(2,slot->slot_base+DIO_ESTOP_OUT,slot->port_addr);
    }
    return 0;
}

static int export_USC_stepgen(slot_data_t *slot, bus_data_t *bus)
{
    int retval, n;
    stepgen_t *sg;

    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  exporting step generators\n");

    /* do hardware init */

    /* allocate shared memory for the digital output data */
    slot->stepgen = hal_malloc(sizeof(stepgens_t));
    if (slot->stepgen == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: hal_malloc() failed\n");
	return -1;
    }
    /* export params that apply to all four stepgens */
    retval = hal_param_u32_newf(HAL_RW, &(slot->stepgen->setup_time_ns), comp_id,
	"ppmc.%d.stepgen.%02d-%02d.setup-time-ns", bus->busnum, bus->last_stepgen, bus->last_stepgen+3);
    if (retval != 0) {
	return retval;
    }
    /* 10uS default setup time */
    slot->stepgen->setup_time_ns = 10000;
    retval = hal_param_u32_newf(HAL_RW, &(slot->stepgen->pulse_width_ns), comp_id,
	"ppmc.%d.stepgen.%02d-%02d.pulse-width-ns", bus->busnum, bus->last_stepgen, bus->last_stepgen+3);
    if (retval != 0) {
	return retval;
    }
    /* 4uS default pulse width */
    slot->stepgen->pulse_width_ns = 4000;
    retval = hal_param_u32_newf(HAL_RW, &(slot->stepgen->pulse_space_ns), comp_id,
	"ppmc.%d.stepgen.%02d-%02d.pulse-space-min-ns", bus->busnum, bus->last_stepgen, bus->last_stepgen+3);
    if (retval != 0) {
	return retval;
    }
    /* 4uS default pulse spacing */
    slot->stepgen->pulse_space_ns = 4000;
    /* export per-stepgen pins and params */
    for ( n = 0 ; n < 4 ; n++ ) {
	/* pointer to the stepgen struct */
	sg = &(slot->stepgen->sg[n]);
	/* enable pin */
	retval = hal_pin_bit_newf(HAL_IN, &(sg->enable), comp_id,
		"ppmc.%d.stepgen.%02d.enable", bus->busnum, bus->last_stepgen);
	if (retval != 0) {
	    return retval;
	}
	/* velocity command pin */
	retval = hal_pin_float_newf(HAL_IN, &(sg->vel), comp_id,
		"ppmc.%d.stepgen.%02d.velocity", bus->busnum, bus->last_stepgen);
	if (retval != 0) {
	    return retval;
	}
	/* velocity scaling parameter */
	retval = hal_param_float_newf(HAL_RW, &(sg->scale), comp_id,
		"ppmc.%d.stepgen.%02d.scale", bus->busnum, bus->last_stepgen);
	if (retval != 0) {
	    return retval;
	}
	sg->scale = 1.0;
	/* maximum velocity parameter */
	retval = hal_param_float_newf(HAL_RW, &(sg->max_vel), comp_id,
		"ppmc.%d.stepgen.%02d.max-vel", bus->busnum, bus->last_stepgen);
	if (retval != 0) {
	    return retval;
	}
	sg->max_vel = 0.0;
	/* actual frequency parameter */
	retval = hal_param_float_newf(HAL_RO, &(sg->freq), comp_id,
		"ppmc.%d.stepgen.%02d.freq", bus->busnum, bus->last_stepgen);
	if (retval != 0) {
	    return retval;
	}
	/* increment number to prepare for next output */
	bus->last_stepgen++;
    }
    add_wr_funct(write_stepgens, slot, block(RATE_GEN_0, RATE_WIDTH_0));
    return 0;
}


static int export_UPC_pwmgen(slot_data_t *slot, bus_data_t *bus)
{
    int retval, n;
    pwmgen_t *pg;

    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  exporting PWM generators\n");

    /* do hardware init */

    /* allocate shared memory for the PWM generators */
    slot->pwmgen = hal_malloc(sizeof(pwmgens_t));
    if (slot->pwmgen == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: hal_malloc() failed\n");
	return -1;
    }
    /* export params that apply to all four pwmgens */
    retval = hal_param_float_newf(HAL_RW, &(slot->pwmgen->freq), comp_id,
	"ppmc.%d.pwm.%02d-%02d.freq", bus->busnum, bus->last_pwmgen, bus->last_pwmgen+3);
    if (retval != 0) {
	return retval;
    }
    /* set initial value for param */
    slot->pwmgen->freq = 0.0;
    /* export per-pwmgen pins and params, and set initial values */
    for ( n = 0 ; n < 4 ; n++ ) {
	/* pointer to the pwmgen struct */
	pg = &(slot->pwmgen->pg[n]);
	/* enable pin */
	retval = hal_pin_bit_newf(HAL_IN, &(pg->enable), comp_id,
		"ppmc.%d.pwm.%02d.enable", bus->busnum, bus->last_pwmgen);
	if (retval != 0) {
	    return retval;
	}
	/* value command pin */
	retval = hal_pin_float_newf(HAL_IN, &(pg->value), comp_id,
		"ppmc.%d.pwm.%02d.value", bus->busnum, bus->last_pwmgen);
	if (retval != 0) {
	    return retval;
	}
	/* output scaling parameter */
	retval = hal_param_float_newf(HAL_RW, &(pg->scale), comp_id,
		"ppmc.%d.pwm.%02d.scale", bus->busnum, bus->last_pwmgen);
	if (retval != 0) {
	    return retval;
	}
	pg->scale = 1.0;
	/* maximum duty cycle parameter */
	retval = hal_param_float_newf(HAL_RW, &(pg->max_dc), comp_id,
		"ppmc.%d.pwm.%02d.max-dc", bus->busnum, bus->last_pwmgen);
	if (retval != 0) {
	    return retval;
	}
	pg->max_dc = 1.0;
	/* minimum duty cycle parameter */
	retval = hal_param_float_newf(HAL_RW, &(pg->min_dc), comp_id,
		"ppmc.%d.pwm.%02d.min-dc", bus->busnum, bus->last_pwmgen);
	if (retval != 0) {
	    return retval;
	}
	pg->min_dc = 0.0;
	/* actual duty cycle parameter */
	retval = hal_param_float_newf(HAL_RO, &(pg->duty_cycle), comp_id,
		"ppmc.%d.pwm.%02d.duty-cycle", bus->busnum, bus->last_pwmgen);
	if (retval != 0) {
	    return retval;
	}
	/* bootstrap mode parameter */
	retval = hal_param_bit_newf(HAL_RW, &(pg->bootstrap), comp_id,
		"ppmc.%d.pwm.%02d.bootstrap", bus->busnum, bus->last_pwmgen);
	if (retval != 0) {
	    return retval;
	}
	pg->bootstrap = 0;
	pg->boot_state = 0;
	pg->old_enable = 0;
	/* increment number to prepare for next output */
	bus->last_pwmgen++;
    }
    add_wr_funct(write_pwmgens, slot, block(PWM_GEN_0, PWM_GEN_3+1) | block(PWM_CTRL_0,PWM_FREQ_HI));
    //    add_wr_funct(write_pwmgens, slot, block(PWM_GEN_0, PWM_GEN_3+1));
    return 0;
}
static int export_PPMC_DAC(slot_data_t *slot, bus_data_t *bus)
{
    int retval, n;
    DAC_t *pg;

    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC:  exporting PPMC DAC\n");

    /* do hardware init */

    /* allocate shared memory for the DAC */
    slot->DAC = hal_malloc(sizeof(DACs_t));
    if (slot->DAC == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: hal_malloc() failed\n");
	return -1;
    }
    /* export per-DAC pins and params, and set initial values */
    for ( n = 0 ; n < 4 ; n++ ) {
	/* pointer to the DAC struct */
	pg = &(slot->DAC->pg[n]);
	/* value command pin */
	retval = hal_pin_float_newf(HAL_IN, &(pg->value), comp_id,
		"ppmc.%d.DAC.%02d.value", bus->busnum, bus->last_DAC);
	if (retval != 0) {
	    return retval;
	}
	/* output scaling parameter */
	retval = hal_param_float_newf(HAL_RW, &(pg->scale), comp_id,
		"ppmc.%d.DAC.%02d.scale", bus->busnum, bus->last_DAC);
	if (retval != 0) {
	    return retval;
	}
	pg->scale = 1.0;
	/* increment number to prepare for next output */
	bus->last_DAC++;
    }
    add_wr_funct(write_DACs, slot, block(DAC_0, DAC_3+1));
    return 0;
}



/* Each of the encoders has the following:
    params:
        ppmc.n.encoder.m.scale      float
    pins:
        ppmc.n.encoder.m.position   float
        ppmc.n.encoder.m.counts     s32
        ppmc.n.encoder.m.index      bit
    
    the output value is position=counts / scale

    Additionally, the encoder registers are zeroed, and the mode is set to latch 
 */

static int export_encoders(slot_data_t *slot, bus_data_t *bus)
{
  int retval, n, m;
    
    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: exporting encoder pins / params\n");

    /* do hardware init */
    /* clear encoder control register */
    SelWrt(0x00, slot->slot_base+ENCCTRL, slot->port_addr);
    /* is there already another board generating a latch strobe? */
    if ( bus->have_master == 0 ) {
	/* no, flag this slot to generate the strobe */
	slot->strobe = 1;
	/* make this a master board */
	SelWrt(0x10, slot->slot_base+ENCRATE, slot->port_addr);
	/* don't need any more masters */
	bus->have_master = 1;
    } else {
	/* already have a master, don't need a strobe */
	slot->strobe = 0;
	/* make this a slave board */
	SelWrt(0x00, slot->slot_base+ENCRATE, slot->port_addr);
    }
    /* we'll reset all counters to 0 */
    SelWrt(0xF0, slot->slot_base+ENCCTRL, slot->port_addr);
    /* clear encoder count load register */
    SelWrt(0x00, slot->slot_base+ENCLOAD, slot->port_addr);
    WrtMore(0x00, slot->port_addr);
    WrtMore(0x00, slot->port_addr);
    /* extra delay, just to be sure */
    ClrTimeout(slot->port_addr);
    ClrTimeout(slot->port_addr);
    ClrTimeout(slot->port_addr);
    /* clear encoder control register */
    SelWrt(0x00, slot->slot_base+ENCCTRL, slot->port_addr);
    /* turn off encoder clear count on index pulse for all axes */
    SelWrt(0x00,slot->slot_base+ENCINDX, slot->port_addr);

    /* allocate shared memory for the encoder data */
    slot->encoder = hal_malloc(4 * sizeof(encoder_t));
    if (slot->encoder == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "PPMC: ERROR: hal_malloc() failed\n");
        return -1;
    }
    slot->encoder[0].indres = 0;  /* clear reset-on-index reg copy */
    /* export per-encoder pins and params */
    if (slot->enc_freq != 0) {
      switch (slot->enc_freq) {
      case 1:
	m = 0;
	rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: setting encoder clock to 1 MHz.\n");
	break;
      case 2:
	m = 1;
	rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: setting encoder clock to 2.5 MHz.\n");
	break;
      case 5:
	m = 2;
	rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: setting encoder clock to 5 MHz.\n");
	break;
      case 10:
	m = 3;
	rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: setting encoder clock to 10 MHz.\n");
	break;
      default:
	m = 0;
	rtapi_print_msg(RTAPI_MSG_ERR, "PPMC: invalid encoder clock setting.\n");
	break;
      }
      SelWrt(m, slot->slot_base+ENCCLOCK, slot->port_addr);  // make the setting
    }
    for ( n = 0 ; n < 4 ; n++ ) {
        /* scale input parameter */
        retval = hal_param_float_newf(HAL_RW, &(slot->encoder[n].scale), comp_id,
		"ppmc.%d.encoder.%02d.scale", bus->busnum, bus->last_encoder);
        if (retval != 0) {
            return retval;
        }
        /* scaled encoder position */
        retval = hal_pin_float_newf(HAL_OUT, &(slot->encoder[n].position), comp_id,
		"ppmc.%d.encoder.%02d.position", bus->busnum, bus->last_encoder);
        if (retval != 0) {
            return retval;
        }
	/* raw encoder position */
	retval = hal_pin_s32_newf(HAL_OUT, &(slot->encoder[n].count), comp_id,
		"ppmc.%d.encoder.%02d.count", bus->busnum, bus->last_encoder);
	if (retval != 0) {
		return retval;
	}
	/* raw encoder delta */
	retval = hal_pin_s32_newf(HAL_OUT, &(slot->encoder[n].delta), comp_id,
		"ppmc.%d.encoder.%02d.delta", bus->busnum, bus->last_encoder);
	if (retval != 0) {
		return retval;
	}
	/* encoder index bit */
	retval = hal_pin_bit_newf(HAL_OUT, &(slot->encoder[n].index), comp_id,
		"ppmc.%d.encoder.%02d.index", bus->busnum, bus->last_encoder);
	if (retval != 0) {
		return retval;
	}
	retval = hal_pin_float_newf(HAL_OUT, &(slot->encoder[n].vel), comp_id,
	    "ppmc.%d.encoder.%02d.velocity",bus->busnum,bus->last_encoder);
	if (retval != 0) {
	  return retval;
	}
	if (slot->ver >= 2) {
	  /* encoder index enable bit */
	  /* if the ver of the board firmware is >= 2 then the board supports
	     this function, so export the pin */
	  retval = hal_pin_bit_newf(HAL_IO, &(slot->encoder[n].index_enable), comp_id,
		"ppmc.%d.encoder.%02d.index-enable", bus->busnum, bus->last_encoder);
	  if (retval != 0) {
	    return retval;
	  }
	  if (slot->use_timestamp) {
	    /* encoder time stamp function / velocity estimation */
	    /* only implemented on latest UPC right now */
	    retval = hal_param_float_newf(HAL_RW, &(slot->encoder[n].min_speed), comp_id,
		   "ppmc.%d.encoder.%02d.min-speed-estimate", bus->busnum, bus->last_encoder);
	    if (retval != 0) {
	      return retval;
	    }
	  }
	}
	/* increment number to prepare for next output */
	bus->last_encoder++;
    }
    // need to read more registers from board if has timestamp
    if (slot->use_timestamp) {
      add_rd_funct(read_encoders, slot, block(ENCCNT0, ENCISR) | block(ENCTS, ENCTB+1));
    } else {
      add_rd_funct(read_encoders, slot, block(ENCCNT0, ENCISR));
    }
    add_wr_funct(write_encoders, slot, block(ENCINDX, ENCINDX));
    return 0;
}

static int export_extra_dac(slot_data_t *slot, bus_data_t *bus)
{
    int retval, n;
    DAC_t *pg;

    /* does the board have the extra port? */
    n=0;
    if (slot->id == 0x40) n=1;
    if (slot->id == 0x50 && slot->ver >= 2) n=1;
    if ( n == 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: board firmware doesn't support 'extra' port\n");
	return -1;
    }
    /* allocate shared memory for the DAC */
    slot->extra = hal_malloc(sizeof(extra_t));
    if (slot->extra == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: hal_malloc() failed\n");
	return -1;
    }
    slot->extra_mode = EXTRA_DAC;
    /* export DAC pins and params, and set initial values */
    /* pointer to the DAC struct */
    pg = &(slot->extra->dac);
    /* value command pin */
    retval = hal_pin_float_newf(HAL_IN, &(pg->value), comp_id,
	"ppmc.%d.DAC8.%02d.value", bus->busnum, bus->last_extraDAC);
    if (retval != 0) {
	return retval;
    }
    /* output scaling parameter */
    retval = hal_param_float_newf(HAL_RW, &(pg->scale), comp_id,
	"ppmc.%d.DAC8.%02d.scale", bus->busnum, bus->last_extraDAC);
    if (retval != 0) {
	return retval;
    }
    pg->scale = 1.0;
    /* increment number to prepare for next output */
    bus->last_extraDAC++;
    add_wr_funct(write_extraDAC, slot, block(UxC_EXTRA, UxC_EXTRA));
    return 0;
}

 int export_timestamp(slot_data_t *slot, bus_data_t *bus)
{
    (void)bus;
    int n;

    /* does the board have the timestamp feature? */
    /* ID = 10 for PPMC encoder, ID = 50 for UPC, either needs to be
       above ver 4 to have the timestamp feature */
    n=0;
    if ((slot->id == 0x10 || slot->id == 0x50) && slot->ver >= 4) n=1;
    if ( n == 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: board firmware doesn't support encoder timestamp.\n");
	return -1;
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "PPMC: exporting encoder timestamp pins\n");

    slot->use_timestamp = 1;    /* tell encoder function to process timestamp */
    return 0;
}

static int export_extra_dout(slot_data_t *slot, bus_data_t *bus)
{
    int retval, n;
    dout_t *pg;

    /* does the board have the extra port? */
    n=0;
    if (slot->id == 0x40) n=1;
    if (slot->id == 0x50 && slot->ver >= 2) n=1;
    if ( n == 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: board firmware doesn't support 'extra' port\n");
	return -1;
    }
    /* allocate shared memory for the douts */
    slot->extra = hal_malloc(sizeof(extra_t));
    if (slot->extra == 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "PPMC: ERROR: hal_malloc() failed\n");
	return -1;
    }
    slot->extra_mode = EXTRA_DOUT;
    for ( n = 0 ; n < 8 ; n++ ) {
      pg = &(slot->extra->douts[n]);
	/* export pin for output data */
	retval = hal_pin_bit_newf(HAL_IN, &(pg->data), comp_id,
		"ppmc.%d.dout.%02d.out", bus->busnum, bus->last_digout);
	if (retval != 0) {
	    return retval;
	}
	/* export parameter for inversion */
	retval = hal_param_bit_newf(HAL_RW, &(pg->invert), comp_id,
		"ppmc.%d.dout.%02d.invert", bus->busnum, bus->last_digout);
	if (retval != 0) {
	    return retval;
	}
	pg->invert = 0;
	/* increment number to prepare for next output */
	bus->last_digout++;
    }
    add_wr_funct(write_extra_dout, slot, block(UxC_EXTRA, UxC_EXTRA));
    return 0;
}


/* utility functions for EPP bus */

/* reset all boards attached to the EPP bus */
#if 0  /* FIXME not used */
static void BusReset(unsigned int port_addr)
{
  rtapi_outb(0,CONTROLPORT(port_addr));
  rtapi_outb(4,CONTROLPORT(port_addr));
  return;
}
#endif

/* tests for an EPP bus timeout, and clears it if so */
static int ClrTimeout(unsigned int port_addr)
{
    unsigned char r;

    r = rtapi_inb(STATUSPORT(port_addr));
    
    if  (!(r & 0x01)) {
	return 0;
    }
/* remove after testing */
/* rtapi_print("EPP Bus Timeout!\n" ); */
    /* To clear timeout some chips require double read */
/*    BusReset(port_addr);  don't do this, it resets all registers in PPMC boards!!  */
    r = rtapi_inb(STATUSPORT(port_addr));
    rtapi_outb(r | 0x01, STATUSPORT(port_addr)); /* Some reset by writing 1 */
    r = rtapi_inb(STATUSPORT(port_addr));

    return !(r & 0x01);
}

/* sets the EPP address and then reads one byte from that address */
static unsigned short SelRead(unsigned char epp_addr, unsigned int port_addr)
{
    unsigned char b;
    
    ClrTimeout(port_addr);
    /* set port direction to output */
    rtapi_outb(0x04,CONTROLPORT(port_addr));
    /* write epp address to port */
    rtapi_outb(epp_addr,ADDRPORT(port_addr));
    if (epp_dir[currentbus] == 1) {
      /* set port direction to input */
      rtapi_outb(0x24,CONTROLPORT(port_addr));
    }
    /* read data value */
    b = rtapi_inb(DATAPORT(port_addr));
    return b;

}

/* reads one byte from EPP, use only after SelRead, and only 
   when hardware has auto-increment address cntr */
static unsigned short ReadMore(unsigned int port_addr)
{
    unsigned char b;
    b = rtapi_inb(DATAPORT(port_addr));
    return b;

}

/* sets the EPP address and then writes one byte to that address */
static void SelWrt(unsigned char byte, unsigned char epp_addr, unsigned int port_addr)
{
    ClrTimeout(port_addr);
    /* set port direction to output */
    rtapi_outb(0x04,CONTROLPORT(port_addr));
    /* write epp address to port */
    rtapi_outb(epp_addr,ADDRPORT(port_addr));
    /* write data to port */
    rtapi_outb(byte,DATAPORT(port_addr));
    return;
}

/* writes one byte to EPP, use only after SelWrt, and only 
   when hardware has auto-increment address cntr */
static void WrtMore(unsigned char byte, unsigned int port_addr)
{
    rtapi_outb(byte,DATAPORT(port_addr));
    return;
}

