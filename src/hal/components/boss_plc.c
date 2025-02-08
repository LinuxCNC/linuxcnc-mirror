/******************************************************************************
 *
 * Copyright (C) 2007 Peter G. Vavaroutsos <pete AT vavaroutsos DOT com>
 *
 *
 * This module is a hard coded PLC for use on a Bridgeport Boss.
 *
 * Installation of the component (realtime only):
 *
 * insmod boss_plc count=<1>
 *
 *
 * The following items are exported to the HAL. <id> is
 * the component id number and is formatted as "%d".
 *
 *  Pins (former parameters):
 *        u32       boss_plc.<id>.amp-ready-delay
 *        u32       boss_plc.<id>.brake-on-delay
 *        u32       boss_plc.<id>.brake-off-delay
 *        u32       boss_plc.<id>.spindle-lo-to-hi
 *        float     boss_plc.<id>.jog-scale-0
 *        float     boss_plc.<id>.jog-scale-1
 *        float     boss_plc.<id>.jog-scale-2
 *
 *  Pins:
 *        bit       boss_plc.<id>.cycle-start-in
 *        bit       boss_plc.<id>.cycle-hold-in
 *        bit       boss_plc.<id>.feed-hold-out
 *        float     boss_plc.<id>.adaptive-feed-in
 *        float     boss_plc.<id>.adaptive-feed-out
 *        bit       boss_plc.<id>.tool-change-in
 *        bit       boss_plc.<id>.tool-changed-out
 *        bit       boss_plc.<id>.wait-user-out
 *        bit       boss_plc.<id>.mist-on-in
 *        bit       boss_plc.<id>.mist-on-out
 *        bit       boss_plc.<id>.flood-on-in
 *        bit       boss_plc.<id>.flood-on-out
 *
 *        bit       boss_plc.<id>.limit-override-in
 *        bit       boss_plc.<id>.limit-active-out
 *        float     boss_plc.<id>.x-position-in
 *        bit       boss_plc.<id>.x-jog-en-in
 *        bit       boss_plc.<id>.x-limit-in
 *        bit       boss_plc.<id>.x-limit-pos-out
 *        bit       boss_plc.<id>.x-limit-neg-out
 *        float     boss_plc.<id>.y-position-in
 *        bit       boss_plc.<id>.y-jog-en-in
 *        bit       boss_plc.<id>.y-limit-in
 *        bit       boss_plc.<id>.y-limit-pos-out
 *        bit       boss_plc.<id>.y-limit-neg-out
 *        bit       boss_plc.<id>.z-jog-en-in
 *        bit       boss_plc.<id>.z-limit-pos-in
 *        bit       boss_plc.<id>.z-limit-neg-in
 *        bit       boss_plc.<id>.z-limit-pos-out
 *        bit       boss_plc.<id>.z-limit-neg-out
 *
 *        bit       boss_plc.<id>.x-amp-enable-in
 *        bit       boss_plc.<id>.x-amp-ready-in
 *        bit       boss_plc.<id>.x-amp-fault-out
 *        bit       boss_plc.<id>.y-amp-enable-in
 *        bit       boss_plc.<id>.y-amp-ready-in
 *        bit       boss_plc.<id>.y-amp-fault-out
 *        bit       boss_plc.<id>.z-amp-enable-in
 *        bit       boss_plc.<id>.z-amp-ready-in
 *        bit       boss_plc.<id>.z-amp-fault-out
 *        bit       boss_plc.<id>.a-amp-enable-in
 *        bit       boss_plc.<id>.a-amp-ready-in
 *        bit       boss_plc.<id>.a-amp-fault-out
 *
 *        float     boss_plc.<id>.spindle-speed-in
 *        bit       boss_plc.<id>.spindle-is-on-in
 *        bit       boss_plc.<id>.spindle-fwd-out
 *        bit       boss_plc.<id>.spindle-rev-out
 *        bit       boss_plc.<id>.spindle-inc-in
 *        bit       boss_plc.<id>.spindle-dec-in
 *        bit       boss_plc.<id>.spindle-inc-out
 *        bit       boss_plc.<id>.spindle-dec-out
 *        bit       boss_plc.<id>.brake-en-in
 *        bit       boss_plc.<id>.brake-en-out
 *
 *        bit       boss_plc.<id>.jog-sel-in-0
 *        bit       boss_plc.<id>.jog-sel-in-1
 *        bit       boss_plc.<id>.jog-sel-in-2
 *        bit       boss_plc.<id>.jog-scale-out
 *
 *   Functions:
 *        void      boss_plc.<id>.refresh
 *
 ******************************************************************************
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General
 * Public License as published by the Free Software Foundation.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
 * ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
 * TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
 * harming persons must have provisions for completely removing power
 * from all motors, etc, before persons enter any danger area.  All
 * machinery must be designed to comply with local and national safety
 * codes, and the authors of this software can not, and do not, take
 * any responsibility for such compliance.
 *
 * This code was written as part of the EMC HAL project.  For more
 * information, go to www.linuxcnc.org.
 *
 ******************************************************************************/


#include "rtapi.h"              // RTAPI realtime OS API.
#include "rtapi_app.h"          // RTAPI realtime module decls.
#include "hal.h"                // HAL public API decls.


#define TRUE                    1
#define FALSE                   0
typedef int                     BOOL;


// Module information.
MODULE_AUTHOR("Pete Vavaroutsos");
MODULE_DESCRIPTION("Bridgeport BOSS PLC for EMC HAL");
MODULE_LICENSE("GPL");
static unsigned long                    count = 1;
RTAPI_MP_LONG(count, "Number of BOSS PLCs to instance");
static int                              debug = 0;
RTAPI_MP_INT(debug, "Enables optional params");


/******************************************************************************
 * TIMER OBJECT
 *
 * This object implements a timer with mSec resolution.
 *
 ******************************************************************************/

typedef enum {
    TM_ONE_SHOT,
    TM_CONTINUOUS,
} TimerMode;

typedef void                    (*TIMER_ROUTINE)(void *pArgs);

typedef struct {
    // Private data.
    BOOL                        enabled;
    hal_u32_t                   nSec;
    hal_u32_t                   count;
    hal_u32_t                   timeout;
    TIMER_ROUTINE               pTimeout;
    void                        *pArgs;
    TimerMode                   mode;
} Timer;

static void Timer_Init(Timer *this);
static void Timer_Enable(Timer *this, TimerMode mode);
static void Timer_Disable(Timer *this);
static BOOL Timer_IsEnabled(Timer *this);
static void Timer_Update(Timer *this, long period);
static void Timer_SetTimeout(Timer *this, hal_u32_t timeout);
#if 0
static void Timer_SetCallback(Timer *this, TIMER_ROUTINE pCallback, void *pArgs);
#endif


/******************************************************************************
 * LIMIT OBJECT
 *
 * This object converts a bi-directional limit switch into to limit signals.
 *
 ******************************************************************************/

typedef enum {
    LS_INIT,
    LS_ON_LIMIT,
    LS_NO_MOTION,
    LS_POS_MOTION,
    LS_NEG_MOTION,
} LimitState;

typedef struct {
    // Exported pins.
    hal_float_t                 *pPositionIn;
    hal_bit_t                   *pJogEnIn;
    hal_bit_t                   *pIn;
    hal_bit_t                   *pPosOut;
    hal_bit_t                   *pNegOut;

    // Internal data.
    LimitState                  state;
    hal_float_t                 position;
    hal_bit_t                   limitPos;
    hal_bit_t                   limitNeg;
} Limit;

static int Limit_Export(Limit *this, int compId, int id, char axis);
static void Limit_Init(Limit *this);
static BOOL Limit_IsActive(Limit *this);
static void Limit_Refresh(Limit *this, hal_bit_t override);


/******************************************************************************
 * AMP OBJECT
 *
 * This object creates the amp fault signal from the enable and ready signal.
 *
 ******************************************************************************/

typedef struct {
    // Exported pins.
    hal_bit_t                   *pEnableIn;
    hal_bit_t                   *pReadyIn;
    hal_bit_t                   *pFaultOut;

    // Internal data.
    Timer                       timer;
    hal_bit_t                   lastEnable;
} Amp;

static int Amp_Export(Amp *this, int compId, int id, char axis);
static void Amp_Init(Amp *this);
static void Amp_Refresh(Amp *this, long period, hal_u32_t readyDelay);


/******************************************************************************
 * PLC OBJECT
 *
 * This object contains all the data for one PLC. A component object is
 * dynamically allocated in shmem for each PLC during initialization.
 *
 ******************************************************************************/

#define NUM_JOG_SEL             3
#define NUM_AXIS                4

static char                     axisNames[NUM_AXIS] = {
    'x', 'y', 'z', 'a'
};

typedef enum {
    SS_OFF,
    SS_WAIT_BRAKE_OFF,
    SS_WAIT_ON,
    SS_ON,
    SS_WAIT_OFF,
    SS_WAIT_BRAKE_ON,
} SpindleState;

typedef struct {
    // Pins. (former parameters)
    hal_u32_t                   *ampReadyDelay;
    hal_u32_t                   *brakeOnDelay;
    hal_u32_t                   *brakeOffDelay;
    hal_float_t                 *spindleLoToHi;
    hal_float_t                 *jogScale[NUM_JOG_SEL];

    // Pins.
    hal_bit_t                   *pCycleStartIn;
    hal_bit_t                   *pCycleHoldIn;
    hal_bit_t                   *pFeedHoldOut;
    hal_float_t                 *pAdaptiveFeedIn;
    hal_float_t                 *pAdaptiveFeedOut;
    hal_bit_t                   *pToolChangeIn;
    hal_bit_t                   *pToolChangedOut;
    hal_bit_t                   *pWaitUserOut;
    hal_bit_t                   *pMistOnIn;
    hal_bit_t                   *pMistOnOut;
    hal_bit_t                   *pFloodOnIn;
    hal_bit_t                   *pFloodOnOut;

    hal_bit_t                   *pLimitOverrideIn;
    hal_bit_t                   *pLimitActiveOut;
    Limit                       xLimit;
    Limit                       yLimit;
    hal_bit_t                   *pZJogEnIn;
    hal_bit_t                   *pZLimitPosIn;
    hal_bit_t                   *pZLimitNegIn;
    hal_bit_t                   *pZLimitPosOut;
    hal_bit_t                   *pZLimitNegOut;

    Amp                         amps[NUM_AXIS];

    hal_float_t                 *pSpindleSpeedIn;
    hal_bit_t                   *pSpindleIsOnIn;
    hal_bit_t                   *pSpindleFwdOut;
    hal_bit_t                   *pSpindleRevOut;
    hal_bit_t                   *pSpindleIncIn;
    hal_bit_t                   *pSpindleDecIn;
    hal_bit_t                   *pSpindleIncOut;
    hal_bit_t                   *pSpindleDecOut;
    hal_bit_t                   *pBrakeEnIn;
    hal_bit_t                   *pBrakeEnOut;

    hal_bit_t                   *pJogSelIn[NUM_JOG_SEL];
    hal_float_t                 *pJogScaleOut;

    // Private data.
    SpindleState                spindleState;
    Timer                       spindleTimer;
    hal_float_t                 lastSpindleSpeed;
    hal_bit_t                   lastCycleStart;
} Plc;


// These methods are used for initialization.
static int Plc_Init(Plc *this);
static int Plc_Export(Plc *this, int compId, int id);
static int Plc_ExportFeed(Plc *this, int compId, int id);
static int Plc_ExportLimits(Plc *this, int compId, int id);
static int Plc_ExportAmps(Plc *this, int compId, int id);
static int Plc_ExportSpindle(Plc *this, int compId, int id);
static int Plc_ExportJog(Plc *this, int compId, int id);

// These methods are exported to the HAL.
static void Plc_Refresh(void *this, long period);

// Private helper methods.
static void Plc_RefreshFeed(Plc *this, long period);
static void Plc_RefreshLimits(Plc *this, long period);
static void Plc_RefreshAmps(Plc *this, long period);
static void Plc_RefreshSpindle(Plc *this, long period);
static void Plc_RefreshJog(Plc *this, long period);


/******************************************************************************
 * COMPONENT OBJECT
 *
 * This object contains all the data for this HAL component.
 *
 ******************************************************************************/

#define MAX_DEVICES             4

typedef struct {
    int                         id;             // HAL component ID.
    Plc                         *plcTable[MAX_DEVICES];
} Component;

static Component                component;


/******************************************************************************
 * INIT AND EXIT CODE
 ******************************************************************************/

int
rtapi_app_main(void)
{
    unsigned                    i;
    Plc                         *pComp;

    // Connect to the HAL.
    component.id = hal_init("boss_plc");
    if (component.id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "BOSS_PLC: ERROR: hal_init() failed\n");
        return(-1);
    }

    for(i = 0; i < MAX_DEVICES; i++){
        component.plcTable[i] = NULL;
    }

    if(count > MAX_DEVICES)
        count = MAX_DEVICES;

    for(i = 0; i < count; i++){
        // Allocate memory for device object.
        pComp = hal_malloc(sizeof(Plc));

        if (pComp == NULL) {
            rtapi_print_msg(RTAPI_MSG_ERR, "BOSS_PLC: ERROR: hal_malloc() failed\n");
            hal_exit(component.id);
            return(-1);
        }

        // Save pointer to device object.
        component.plcTable[i] = pComp;

        // Initialize device.
        if(Plc_Init(pComp)){
            hal_exit(component.id);
            return(-1);
        }

        // Export pins, parameters, and functions.
        if(Plc_Export(pComp, component.id, i)){
            hal_exit(component.id);
            return(-1);
        }
    }

    hal_ready(component.id);
    return(0);
}


void
rtapi_app_exit(void)
{
    int                         i;
    Plc                         *pComp;

    hal_exit(component.id);

    for(i = 0; i < MAX_DEVICES; i++){
        if((pComp = component.plcTable[i]) != NULL){
            // TODO: Free device object when HAL supports free.
//            hal_free(pComp);
        }
    }
}


/******************************************************************************
 * PLC OBJECT FUNCTION DEFINITIONS
 ******************************************************************************/

/*
 * LOCAL FUNCTIONS
 */

static int
Plc_Init(Plc *this)
{
    int                         i;

    // Initialize variables.
    this->spindleState = SS_OFF;
    this->lastSpindleSpeed = 0.0;
    this->lastCycleStart = 1;

    // Initialize parameters.
    *(this->brakeOffDelay) = 500;
    *(this->brakeOnDelay) = 300;
    *(this->ampReadyDelay) = 50;
    *(this->spindleLoToHi) = 500;

    *(this->jogScale[0]) = 0.0001;
    for(i = 1; i < NUM_JOG_SEL; i++){
        *(this->jogScale[i]) = *(this->jogScale[i-1]) * 10;
    }

    // Initialize timer.
    Timer_Init(&this->spindleTimer);

    // Initialize limits.
    Limit_Init(&this->xLimit);
    Limit_Init(&this->yLimit);

    // Initialize amps.
    for(i = 0; i < NUM_AXIS; i++){
        Amp_Init(&this->amps[i]);
    }

    return(0);
}


static int
Plc_Export(Plc *this, int compId, int id)
{
    int                         msgLevel, error;

    // This function exports a lot of stuff, which results in a lot of
    // logging if msg_level is at INFO or ALL. So we save the current value
    // of msg_level and restore it later.  If you actually need to log this
    // function's actions, change the second line below.
    msgLevel = rtapi_get_msg_level();
    rtapi_set_msg_level(RTAPI_MSG_WARN);

    // Export pins and parameters.
    error = Plc_ExportFeed(this, compId, id);

    if(!error){
        error = Plc_ExportLimits(this, compId, id);
    }

    if(!error){
        error = Plc_ExportAmps(this, compId, id);
    }

    if(!error){
        error = Plc_ExportSpindle(this, compId, id);
    }
    if(!error){
        error = Plc_ExportJog(this, compId, id);
    }

    // Export functions.
    if(!error){
        error = hal_export_functf(Plc_Refresh, this, 1, 0, compId, "boss_plc.%d.refresh", id);
    }

    // Restore saved message level.
    rtapi_set_msg_level(msgLevel);

    return(error);
}


static int
Plc_ExportFeed(Plc *this, int compId, int id)
{
    int                         error;

    // Export pins.
    error = hal_pin_bit_newf(HAL_IN, &this->pCycleStartIn, compId,
			     "boss_plc.%d.cycle-start-in", id);

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pCycleHoldIn, compId,
				 "boss_plc.%d.cycle-hold-in", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pFeedHoldOut, compId,
				 "boss_plc.%d.feed-hold-out", id);
    }

    if(!error){
        error = hal_pin_float_newf(HAL_IN, &this->pAdaptiveFeedIn, compId,
				   "boss_plc.%d.adaptive-feed-in", id);
    }

    if(!error){
        *this->pAdaptiveFeedIn = 1.0;
        error = hal_pin_float_newf(HAL_OUT, &this->pAdaptiveFeedOut, compId,
				   "boss_plc.%d.adaptive-feed-out", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pToolChangeIn, compId,
				 "boss_plc.%d.tool-change-in", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pToolChangedOut, compId,
				 "boss_plc.%d.tool-changed-out", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pWaitUserOut, compId,
				 "boss_plc.%d.wait-user-out", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pMistOnIn, compId,
				 "boss_plc.%d.mist-on-in", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pMistOnOut, compId,
				 "boss_plc.%d.mist-on-out", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pFloodOnIn, compId,
				 "boss_plc.%d.flood-on-in", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pFloodOnOut, compId,
				 "boss_plc.%d.flood-on-out", id);
    }

    return(error);
}


static int
Plc_ExportLimits(Plc *this, int compId, int id)
{
    int                         error;

    // Export pins.
    error = hal_pin_bit_newf(HAL_IN, &this->pLimitOverrideIn, compId,
			     "boss_plc.%d.limit-override-in", id);

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pLimitActiveOut, compId,
				 "boss_plc.%d.limit-active-out", id);
    }

    if(!error){
        error = Limit_Export(&this->xLimit, compId, id, axisNames[0]);
    }

    if(!error){
        error = Limit_Export(&this->yLimit, compId, id, axisNames[1]);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pZLimitPosIn, compId,
				 "boss_plc.%d.%c-limit-pos-in", id, axisNames[2]);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pZJogEnIn, compId,
				 "boss_plc.%d.%c-jog-en-in", id, axisNames[2]);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pZLimitNegIn, compId,
				 "boss_plc.%d.%c-limit-neg-in", id, axisNames[2]);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pZLimitPosOut, compId,
				 "boss_plc.%d.%c-limit-pos-out", id, axisNames[2]);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pZLimitNegOut, compId,
				 "boss_plc.%d.%c-limit-neg-out", id, axisNames[2]);
    }

    // Export optional parameters.
    if(debug > 0){
        if(!error){
            error = hal_param_u32_newf(HAL_RO, &this->xLimit.state, compId,
				       "boss_plc.%d.%c-limit-state", id, axisNames[0]);
        }

        if(!error){
            error = hal_param_u32_newf(HAL_RO, &this->yLimit.state, compId,
				       "boss_plc.%d.%c-limit-state", id, axisNames[1]);
        }
    }

    return(error);
}


static int
Plc_ExportAmps(Plc *this, int compId, int id)
{
    int                         error, i;
    Amp                         *pAmp;

    error = hal_pin_u32_newf(HAL_IO, &this->ampReadyDelay, compId,
			     "boss_plc.%d.amp-ready-delay", id);

    pAmp = this->amps;
    for(i = 0; i < NUM_AXIS && !error; i++, pAmp++){
        error = Amp_Export(pAmp, compId, id, axisNames[i]);
    }

    return(error);
}


static int
Plc_ExportSpindle(Plc *this, int compId, int id)
{
    int                         error;

    // Export parameters.
    error = hal_pin_u32_newf(HAL_IO, &this->brakeOnDelay, compId,
			     "boss_plc.%d.brake-on-delay", id);

    if(!error){
        error = hal_pin_u32_newf(HAL_IO, &this->brakeOffDelay, compId,
				 "boss_plc.%d.brake-off-delay", id);
    }

    if(!error){
        error = hal_pin_float_newf(HAL_IO, &this->spindleLoToHi, compId,
				   "boss_plc.%d.spindle-lo-to-hi", id);
    }

    // Export optional parameters.
    if(debug > 0){
        if(!error){
            error = hal_param_u32_newf(HAL_RO, &this->spindleState, compId,
				       "boss_plc.%d.spindle-state", id);
        }
    }

    // Export pins.
    if(!error){
        error = hal_pin_float_newf(HAL_IN, &this->pSpindleSpeedIn, compId,
				   "boss_plc.%d.spindle-speed-in", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pSpindleIsOnIn, compId,
				 "boss_plc.%d.spindle-is-on-in", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pSpindleFwdOut, compId,
				 "boss_plc.%d.spindle-fwd-out", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pSpindleRevOut, compId,
				 "boss_plc.%d.spindle-rev-out", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pSpindleIncIn, compId,
				 "boss_plc.%d.spindle-inc-in", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pSpindleDecIn, compId,
				 "boss_plc.%d.spindle-dec-in", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pSpindleIncOut, compId,
				 "boss_plc.%d.spindle-inc-out", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pSpindleDecOut, compId,
				 "boss_plc.%d.spindle-dec-out", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pBrakeEnIn, compId,
				 "boss_plc.%d.brake-en-in", id);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pBrakeEnOut, compId,
				 "boss_plc.%d.brake-en-out", id);
    }

    return(error);
}


static int
Plc_ExportJog(Plc *this, int compId, int id)
{
    int                         error, i;

    // Export parameters.
    for(i = 0, error = 0; i < NUM_JOG_SEL && !error; i++){
        error = hal_pin_float_newf(HAL_IO, &this->jogScale[i], compId,
				   "boss_plc.%d.jog-scale-%d", id, i);
    }

    if(!error){
        for(i = 0; i < NUM_JOG_SEL && !error; i++){
            error = hal_pin_bit_newf(HAL_IN, &this->pJogSelIn[i], compId,
				     "boss_plc.%d.jog-sel-in-%d", id, i);
        }
    }

    // Export pins.
    if(!error){
        error = hal_pin_float_newf(HAL_OUT, &this->pJogScaleOut, compId,
				   "boss_plc.%d.jog-scale-out", id);
    }

    return(error);
}


/*
 * HAL EXPORTED FUNCTIONS
 */

static void
Plc_Refresh(void *arg, long period)
{
    Plc                         *this = (Plc *)arg;

    Plc_RefreshFeed(this, period);
    Plc_RefreshLimits(this, period);
    Plc_RefreshAmps(this, period);
    Plc_RefreshSpindle(this, period);
    Plc_RefreshJog(this, period);
}


static void
Plc_RefreshFeed(Plc *this, long period)
{
    (void)period;
    BOOL                        riseCycleStart;

    riseCycleStart = !this->lastCycleStart && *this->pCycleStartIn;
    this->lastCycleStart = *this->pCycleStartIn;

    // Condition feed hold so machine waits for cycle start and spindle to be
    // running if it is enabled.
    *this->pFeedHoldOut = *this->pCycleHoldIn
                            || (*this->pSpindleSpeedIn && !*this->pSpindleIsOnIn)
                            || (*this->pSpindleIsOnIn
                                && (this->lastSpindleSpeed != *this->pSpindleSpeedIn))
                            || (*this->pFeedHoldOut && !riseCycleStart);
    this->lastSpindleSpeed = *this->pSpindleSpeedIn;

    // Limit rapid/feed to 1% when limits are being overridden.
    if(*this->pLimitOverrideIn && (*this->pAdaptiveFeedIn > 0.01))
        *this->pAdaptiveFeedOut = 0.01;
    else
        *this->pAdaptiveFeedOut = *this->pAdaptiveFeedIn;

    // Wait for cycle start to acknowledge tool change.
    *this->pToolChangedOut = (*this->pToolChangeIn && riseCycleStart)
                            || (*this->pToolChangedOut && *this->pToolChangeIn);

    // Indicates waiting for user to press cycle start.
    *this->pWaitUserOut = *this->pFeedHoldOut
                            || (*this->pToolChangeIn && !*this->pToolChangedOut);

    // Turn coolant off during tool changes.
    *this->pMistOnOut = *this->pMistOnIn && !*this->pToolChangeIn;
    *this->pFloodOnOut = *this->pFloodOnIn && !*this->pToolChangeIn;
}


static void
Plc_RefreshLimits(Plc *this, long period)
{
    (void)period;
    Limit_Refresh(&this->xLimit, *this->pLimitOverrideIn);
    Limit_Refresh(&this->yLimit, *this->pLimitOverrideIn);

    // Condition Z limits with override in manual mode.
    *this->pZLimitPosOut = *this->pZLimitPosIn
                            && !(*this->pZJogEnIn && *this->pLimitOverrideIn);
    *this->pZLimitNegOut = *this->pZLimitNegIn
                            && !(*this->pZJogEnIn && *this->pLimitOverrideIn);

    // Generate limit active signal for pilot lamp.
    *this->pLimitActiveOut = Limit_IsActive(&this->xLimit)
                            || Limit_IsActive(&this->yLimit)
                            || *this->pZLimitPosIn || *this->pZLimitNegIn;
}


static void
Plc_RefreshAmps(Plc *this, long period)
{
    int                         i;
    Amp                         *pAmp;

    pAmp = this->amps;
    for(i = 0; i < NUM_AXIS; i++, pAmp++){
        Amp_Refresh(pAmp, period, *(this->ampReadyDelay));
    }
}

static void
Plc_RefreshSpindle(Plc *this, long period)
{
    Timer_Update(&this->spindleTimer, period);

    switch(this->spindleState){
    // Spindle is off, brake is on.
    case SS_OFF:
        if(!*this->pBrakeEnIn){
            this->spindleState = SS_WAIT_BRAKE_OFF;
            *this->pBrakeEnOut = 0;
            Timer_SetTimeout(&this->spindleTimer, *(this->brakeOffDelay));
            Timer_Enable(&this->spindleTimer, TM_ONE_SHOT);
        }
        break;

    // Spindle is off, brake has been turned off. Wait at least a brake off
    // delay before turning spindle on.
    case SS_WAIT_BRAKE_OFF:
        if(*this->pBrakeEnIn){
            this->spindleState = SS_OFF;
            *this->pBrakeEnOut = 1;
            Timer_Disable(&this->spindleTimer);

        }else if((*this->pSpindleSpeedIn != 0.0)
                 && !Timer_IsEnabled(&this->spindleTimer)){

            this->spindleState = SS_WAIT_ON;

            if(*this->pSpindleSpeedIn > *(this->spindleLoToHi)
              || (*this->pSpindleSpeedIn < 0.0
                && *this->pSpindleSpeedIn >= -*(this->spindleLoToHi))){

                *this->pSpindleFwdOut = 1;
            }else{
                *this->pSpindleRevOut = 1;
            }
        }
        break;

    // Spindle has been turned on. Wait for confirmation that it is running.
    case SS_WAIT_ON:
        if(*this->pSpindleIsOnIn){
            this->spindleState = SS_ON;

        }else if(*this->pSpindleSpeedIn == 0.0){
            this->spindleState = SS_WAIT_BRAKE_OFF;

            *this->pSpindleFwdOut = 0;
            *this->pSpindleRevOut = 0;
        }
        break;

    // Spindle is running.
    case SS_ON:
        if(*this->pSpindleSpeedIn == 0.0){
            this->spindleState = SS_WAIT_OFF;

            *this->pSpindleFwdOut = 0;
            *this->pSpindleRevOut = 0;
        }
        break;

    // Spindle has been turned off. Wait for confirmation.
    case SS_WAIT_OFF:
        if(!*this->pSpindleIsOnIn){
            this->spindleState = SS_WAIT_BRAKE_ON;

            Timer_SetTimeout(&this->spindleTimer, *(this->brakeOnDelay));
            Timer_Enable(&this->spindleTimer, TM_ONE_SHOT);
        }
        break;

    case SS_WAIT_BRAKE_ON:
        if(!Timer_IsEnabled(&this->spindleTimer)){
            this->spindleState = SS_WAIT_BRAKE_OFF;
        }
        break;

    default:
        this->spindleState = SS_WAIT_OFF;

        *this->pSpindleFwdOut = 0;
        *this->pSpindleRevOut = 0;
    }

    // Condition spindle increase and decrease so they are disabled when
    // spindle is not running and both cannot be enabled at the same time.
    *this->pSpindleIncOut = *this->pSpindleIncIn && !*this->pSpindleDecIn
                            && *this->pSpindleIsOnIn;
    *this->pSpindleDecOut = *this->pSpindleDecIn && !*this->pSpindleIncIn
                            && *this->pSpindleIsOnIn;
}


static void
Plc_RefreshJog(Plc *this, long period)
{
    (void)period;
    int                         i;

    // Jog scale.
    for(i = 0; i < NUM_JOG_SEL; i++){
        if(*this->pJogSelIn[i]){
            *this->pJogScaleOut = *(this->jogScale[i]);
            break;
        }
    }
}


/******************************************************************************
 * LIMIT OBJECT FUNCTION DEFINITIONS
 ******************************************************************************/

static int
Limit_Export(Limit *this, int compId, int id, char axis)
{
    int                         error;

    error = hal_pin_float_newf(HAL_IN, &this->pPositionIn, compId,
			       "boss_plc.%d.%c-position-in", id, axis);

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pJogEnIn, compId,
				 "boss_plc.%d.%c-jog-en-in", id, axis);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pIn, compId,
				 "boss_plc.%d.%c-limit-in", id, axis);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pPosOut, compId,
				 "boss_plc.%d.%c-limit-pos-out", id, axis);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pNegOut, compId,
				 "boss_plc.%d.%c-limit-neg-out", id, axis);
    }

    return(error);
}


static void
Limit_Init(Limit *this)
{
    this->state = LS_INIT;
}


static BOOL
Limit_IsActive(Limit *this)
{
    return(*this->pIn);
}

static void
Limit_Refresh(Limit *this, hal_bit_t override)
{
    switch(this->state){
    case LS_INIT:
    default:
        this->state = LS_ON_LIMIT;
        this->limitNeg = this->limitPos = 1;
        this->position = *this->pPositionIn;
        // Fall through.

    case LS_ON_LIMIT:
        if(*this->pIn == 0){
            this->limitNeg = this->limitPos = 0;

            if(*this->pPositionIn == this->position)
                this->state = LS_NO_MOTION;
            else if(*this->pPositionIn > this->position)
                this->state = LS_POS_MOTION;
            else if(*this->pPositionIn < this->position)
                this->state = LS_NEG_MOTION;
        }
        break;

    case LS_NO_MOTION:
        if(*this->pIn){
            this->state = LS_ON_LIMIT;
            this->limitNeg = this->limitPos = 1;
        }else if(*this->pPositionIn > this->position){
            this->state = LS_POS_MOTION;
        }else if(*this->pPositionIn < this->position){
            this->state = LS_NEG_MOTION;
        }
        break;

    case LS_POS_MOTION:
        if(*this->pIn){
            this->state = LS_ON_LIMIT;
            this->limitPos = 1;
        }else if(*this->pPositionIn == this->position){
            this->state = LS_NO_MOTION;
        }else if(*this->pPositionIn < this->position){
            this->state = LS_NEG_MOTION;
        }
        break;

    case LS_NEG_MOTION:
        if(*this->pIn){
            this->state = LS_ON_LIMIT;
            this->limitNeg = 1;
        }else if(*this->pPositionIn == this->position){
            this->state = LS_NO_MOTION;
        }else if(*this->pPositionIn > this->position){
            this->state = LS_POS_MOTION;
        }
        break;
    }

    this->position = *this->pPositionIn;

    // Condition limits with override in manual mode.
    *this->pPosOut = this->limitPos && !(*this->pJogEnIn && override);
    *this->pNegOut = this->limitNeg && !(*this->pJogEnIn && override);
}


/******************************************************************************
 * AMP OBJECT FUNCTION DEFINITIONS
 ******************************************************************************/

static int
Amp_Export(Amp *this, int compId, int id, char axis)
{
    int                         error;

    error = hal_pin_bit_newf(HAL_IN, &this->pEnableIn, compId,
			     "boss_plc.%d.%c-amp-enable-in", id, axis);

    if(!error){
        error = hal_pin_bit_newf(HAL_IN, &this->pReadyIn, compId,
				 "boss_plc.%d.%c-amp-ready-in", id, axis);
    }

    if(!error){
        error = hal_pin_bit_newf(HAL_OUT, &this->pFaultOut, compId,
				 "boss_plc.%d.%c-amp-fault-out", id, axis);
    }

    return(error);
}


static void
Amp_Init(Amp *this)
{
    // Initialize variables.
    this->lastEnable = 0;

    // Initialize timer.
    Timer_Init(&this->timer);
}


static void
Amp_Refresh(Amp *this, long period, hal_u32_t readyDelay)
{
    Timer_Update(&this->timer, period);

    if(*this->pEnableIn){
        if(!this->lastEnable){
            Timer_SetTimeout(&this->timer, readyDelay);
            Timer_Enable(&this->timer, TM_ONE_SHOT);
        }
    } else {
        Timer_Disable(&this->timer);
    }

    *this->pFaultOut = *this->pEnableIn && !*this->pReadyIn
                        && !Timer_IsEnabled(&this->timer);

    this->lastEnable = *this->pEnableIn;
}


/******************************************************************************
 * TIMER OBJECT FUNCTION DEFINITIONS
 ******************************************************************************/

static void
Timer_Init(Timer *this)
{
    this->enabled = FALSE;
    this->pTimeout = NULL;
}

static void
Timer_Enable(Timer *this, TimerMode mode)
{
    this->mode = mode;
    this->enabled = TRUE;
    this->count = 0;
}


static void
Timer_Disable(Timer *this)
{
    this->enabled = FALSE;
}


static BOOL
Timer_IsEnabled(Timer *this)
{
    return(this->enabled);
}


static void
Timer_Update(Timer *this, long period)
{
    if(!this->enabled)
        return;

    this->nSec += period;

    if(this->nSec > 1000000){
        this->count += this->nSec / 1000000;
        this->nSec %= 1000000;
    }

    if(this->count >= this->timeout){
        if(this->pTimeout != NULL)
            this->pTimeout(this->pArgs);

        if(this->mode != TM_CONTINUOUS){
            this->enabled = FALSE;
        }else{
            this->count -= this->timeout;
        }
    }
}


static void
Timer_SetTimeout(Timer *this, hal_u32_t timeout)
{
    this->count = 0;
    this->timeout = timeout;
}


#if 0
static void
Timer_SetCallback(Timer *this, TIMER_ROUTINE pCallback, void *pArgs)
{
    this->pTimeout = pCallback;
    this->pArgs = pArgs;
}
#endif
