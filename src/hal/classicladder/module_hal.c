/* ClassicLadder Realtime module for emc2/hal */
/* This file is based on the module_rtlinux.c by Marc Le Douarain */

/* Classic Ladder Project */
/* Copyright (C) 2001 Marc Le Douarain */
/* http://www.multimania.com/mavati/classicladder */
/* mavati@club-internet.fr */
/* Copyright (C) 2006 Jeff Epler */
/* jepler@unpy.net */

/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_errno.h"
#include "hal.h"

#include "classicladder.h"
#include "global.h"
#include "calc.h"
#include "vars_access.h"

MODULE_LICENSE("LGPL");
MODULE_AUTHOR("Marc Le Douarain");
MODULE_DESCRIPTION("ClassicLadder HAL module");

int comedi_to_open_mask;
#ifdef DYNAMIC_PLCSIZE
int numRungs=NBR_RUNGS_DEF, numBits=NBR_BITS_DEF,numWords=NBR_WORDS_DEF, numTimers=NBR_TIMERS_DEF, numMonostables=NBR_MONOSTABLES_DEF;
int numCounters=NBR_COUNTERS_DEF,numTimersIec=NBR_TIMERS_IEC_DEF,numPhysInputs=NBR_PHYS_INPUTS_DEF, numPhysOutputs=NBR_PHYS_OUTPUTS_DEF, numArithmExpr=NBR_ARITHM_EXPR_DEF, numSections=NBR_SECTIONS_DEF;
int numSymbols=NBR_SYMBOLS_DEF,numS32in=NBR_PHYS_WORDS_INPUTS_DEF,numS32out=NBR_PHYS_WORDS_OUTPUTS_DEF;
int numFloatIn=NBR_PHYS_FLOAT_INPUTS_DEF,numFloatOut=NBR_PHYS_FLOAT_OUTPUTS_DEF;
RTAPI_MP_INT(numRungs, "i");
RTAPI_MP_INT(numBits, "i");
RTAPI_MP_INT(numWords, "i");
RTAPI_MP_INT(numTimers, "i");
RTAPI_MP_INT(numMonostables, "i");
RTAPI_MP_INT(numCounters, "i");
RTAPI_MP_INT(numTimersIec, "i");
RTAPI_MP_INT(numPhysInputs, "i");
RTAPI_MP_INT(numPhysOutputs, "i");
RTAPI_MP_INT(numArithmExpr, "i");
RTAPI_MP_INT(numSections, "i");
RTAPI_MP_INT(numSymbols, "i");
RTAPI_MP_INT(numS32in, "i");
RTAPI_MP_INT(numS32out, "i");
RTAPI_MP_INT(numFloatIn,"i");
RTAPI_MP_INT(numFloatOut,"i");
#else
#define numPhysInputs InfosGene->SizesInfos.nbr_phys_inputs
#define numPhysOutputs InfosGene->SizesInfos.nbr_phys_outputs
#define numWords InfosGene->SizesInfos.nbr_words
#endif

hal_bit_t **hal_inputs;
hal_bit_t **hide_gui;
hal_bit_t **hal_outputs;
hal_s32_t **hal_s32_inputs;
hal_s32_t **hal_s32_outputs;
hal_s32_t *hal_state;
hal_float_t **hal_float_inputs;
hal_float_t **hal_float_outputs;

extern StrGeneralParams GeneralParamsMirror; 

#define TIME_REFRESH_RUNG_NS (1000 * 1000 * (TIME_REFRESH_RUNG_MS))

void HalReadPhysicalInputs(void) {
	int i;
	for( i=0; i<InfosGene->GeneralParams.SizesInfos.nbr_phys_inputs; i++) {
		WriteVar(VAR_PHYS_INPUT, i, *hal_inputs[i]);
	}
}

void HalWritePhysicalOutputs(void) {
	int i;
	for( i=0; i<InfosGene->GeneralParams.SizesInfos.nbr_phys_outputs; i++) {
		*(hal_outputs[i]) = ReadVar(VAR_PHYS_OUTPUT, i);
	}
}

void HalReads32Inputs(void) {
	int i;
	for( i=0; i<InfosGene->GeneralParams.SizesInfos.nbr_phys_words_inputs; i++) {
		WriteVar(VAR_PHYS_WORD_INPUT, i, *hal_s32_inputs[i]);
	}
}	
void HalWrites32Outputs(void) {
	int i;
	for( i=0; i<InfosGene->GeneralParams.SizesInfos.nbr_phys_words_outputs; i++) {
		*(hal_s32_outputs[i]) = ReadVar(VAR_PHYS_WORD_OUTPUT, i);
	}
}
void HalReadFloatInputs(void) {
	int i;
	for( i=0; i<InfosGene->GeneralParams.SizesInfos.nbr_phys_float_inputs; i++) {
		WriteVar(VAR_PHYS_FLOAT_INPUT, i, *hal_float_inputs[i]);
	}
}	
void HalWriteFloatOutputs(void) {
	int i;
	for( i=0; i<InfosGene->GeneralParams.SizesInfos.nbr_phys_float_outputs; i++) {
		*(hal_float_outputs[i]) = ReadVar(VAR_PHYS_FLOAT_OUTPUT, i);
	}
}
// This actually does the magic of periodic refresh of pins and
// calculations. This function runs at the period rate of the thread
// that you added it to.
// period, leftover, t0,and t1 are in nanoseconds. 
// This function first checks to see if at least 1 millisecond has gone by
// if the period is under 1 MS then if will not refresh rungs yet but 
// will keep track of how many NS were left over. Does this each period
// till at least 1 MS has occurred, if more then 1 MS then keeps track of
// leftover NS for accuracy. Bottom line is you can run classiclader in
// a thread faster than 1 millisecond but it will not refresh the rungs
// any faster (it can be slower though). If your refresh is too slow and 
// your timer are using multiples of 100 microseconds they might not be accurate.
// t0 and t1 are for keeping track of how long the refresh of sections, 
// and HAL pins take (it is displayed in the 'section display' GUI (in microseconds). 

static void hal_task(void *arg, long period) {
	(void)arg;
	unsigned long t0, t1,milliseconds;
	static unsigned long leftover=0;
	leftover += period;
	milliseconds= leftover / 1000000;
	leftover %= 1000000;

	if (milliseconds >= 1) {
		InfosGene->GeneralParams.PeriodicRefreshMilliSecs=milliseconds;
		*hal_state = InfosGene->LadderState;
		t0 = rtapi_get_time();
		if (InfosGene->LadderState==STATE_RUN)
			{
				HalReadPhysicalInputs();

				HalReads32Inputs();
				
				HalReadFloatInputs();

				InfosGene->HideGuiState = *hide_gui[0];

				ClassicLadder_RefreshAllSections();
		
				HalWritePhysicalOutputs();

				HalWrites32Outputs();
    
				HalWriteFloatOutputs();
			}
	 	t1 = rtapi_get_time();
	 	InfosGene->DurationOfLastScan = t1 - t0;
	}
}

extern void CopySizesInfosFromModuleParams( void );

int rtapi_app_main(void) {
	int result, i;
	CopySizesInfosFromModuleParams();

	compId = hal_init("classicladder_rt");
	if(compId < 0) return compId;

	rtapi_print("creating ladder-state\n");

	result = hal_export_funct("classicladder.0.refresh",hal_task,0,1, 0, compId);
	if(result < 0) {
error:
		hal_exit(compId);
		return result;
	}

	hal_state = hal_malloc(sizeof(hal_s32_t));
	result = hal_param_s32_new("classicladder.ladder-state", HAL_RO, hal_state, compId);
	if(result < 0) {
		 hal_exit(compId);
		 return result;
	}

	hal_inputs = hal_malloc(sizeof(hal_bit_t*) * numPhysInputs);
	if(!hal_inputs) { result = -ENOMEM; goto error; }
	hide_gui = hal_malloc(sizeof(hal_bit_t*));
	if(!hide_gui) { result = -ENOMEM; goto error; }
	hal_s32_inputs = hal_malloc(sizeof(hal_s32_t*) * numS32in);
	if(!hal_s32_inputs) { result = -ENOMEM; goto error; }
	hal_float_inputs = hal_malloc(sizeof(hal_float_t*) * numFloatIn);
	if(!hal_float_inputs) { result = -ENOMEM; goto error; }
	hal_outputs = hal_malloc(sizeof(hal_bit_t*) * numPhysOutputs);
	if(!hal_outputs) { result = -ENOMEM; goto error; }
	hal_s32_outputs = hal_malloc(sizeof(hal_s32_t*) * numS32out);
	if(!hal_s32_outputs) { result = -ENOMEM; goto error; }
	hal_float_outputs = hal_malloc(sizeof(hal_float_t*) * numFloatOut);
	if(!hal_float_outputs) { result = -ENOMEM; goto error; }

	for(i=0; i<numPhysInputs; i++) {
		result = hal_pin_bit_newf(HAL_IN, &hal_inputs[i], compId,
				"classicladder.0.in-%02d", i);
		if(result < 0) goto error;
	}
	result = hal_pin_bit_newf(HAL_IN, &hide_gui[0], compId,
				"classicladder.0.hide_gui");
		if(result < 0) goto error;

	for(i=0; i<numS32in; i++) {
		result = hal_pin_s32_newf(HAL_IN, &hal_s32_inputs[i], compId,
				"classicladder.0.s32in-%02d", i);
		if(result < 0) goto error;
	}

	for(i=0; i<numFloatIn; i++) {
		result = hal_pin_float_newf(HAL_IN, &hal_float_inputs[i], compId,
				"classicladder.0.floatin-%02d", i);
		if(result < 0) goto error;
	}

	for(i=0; i<numPhysOutputs; i++) {
		result = hal_pin_bit_newf(HAL_OUT, &hal_outputs[i], compId,
				"classicladder.0.out-%02d", i);
		if(result < 0) goto error;
	}

	for(i=0; i<numS32out; i++) {
		result = hal_pin_s32_newf(HAL_OUT, &hal_s32_outputs[i], compId,
				"classicladder.0.s32out-%02d", i);
		if(result < 0) goto error;
	}

	for(i=0; i<numFloatOut; i++) {
		result = hal_pin_float_newf(HAL_OUT, &hal_float_outputs[i], compId,
				"classicladder.0.floatout-%02d", i);
		if(result < 0) goto error;
	}

	hal_ready(compId);
	ClassicLadder_AllocAll( );
	return 0;
}

void rtapi_app_exit(void) {
	ClassicLadder_FreeAll( FALSE);
	hal_exit(compId);
}

// this function copies any requested (from the cmd line) changes to ladder element amounts to GeneralParamsMirror
// so memory allotment and pin numbers can be varied. Note no error checking is done.
// Symbols allotment is calculated to be large enough to have symbols for all the elements unless specified smaller
// on the command line (the symbols window will only assign -GeneralParamsMirror.SizesInfos.nbr_symbols- number of symbols )
void CopySizesInfosFromModuleParams( void ) {
	plc_sizeinfo_s *pSizesInfos;
	pSizesInfos = &GeneralParamsMirror.SizesInfos;
#ifdef DYNAMIC_PLCSIZE
	if ( numRungs>0 )
		GeneralParamsMirror.SizesInfos.nbr_rungs = numRungs;
	if ( numBits>0 )
		GeneralParamsMirror.SizesInfos.nbr_bits = numBits;
	if ( numWords>0 )
		GeneralParamsMirror.SizesInfos.nbr_words = numWords;
	if ( numTimers>0 )
		GeneralParamsMirror.SizesInfos.nbr_timers = numTimers;
	if ( numMonostables>0 )
		GeneralParamsMirror.SizesInfos.nbr_monostables = numMonostables;
	if ( numCounters>0 )
		GeneralParamsMirror.SizesInfos.nbr_counters = numCounters;
	if ( numTimersIec>0 )
		GeneralParamsMirror.SizesInfos.nbr_timers_iec = numTimersIec;
	if ( numPhysInputs>0 )
		GeneralParamsMirror.SizesInfos.nbr_phys_inputs = numPhysInputs;
	if ( numPhysOutputs>0 )
		GeneralParamsMirror.SizesInfos.nbr_phys_outputs = numPhysOutputs;
	if ( numArithmExpr>0 )
		GeneralParamsMirror.SizesInfos.nbr_arithm_expr = numArithmExpr;
	if ( numSections>0 )
		GeneralParamsMirror.SizesInfos.nbr_sections = numSections;
	if ( numS32in>0 )
		GeneralParamsMirror.SizesInfos.nbr_phys_words_inputs = numS32in;
	if ( numS32out>0 )
		GeneralParamsMirror.SizesInfos.nbr_phys_words_outputs = numS32out;
	if ( numFloatIn>0 )
		GeneralParamsMirror.SizesInfos.nbr_phys_float_inputs = numFloatIn;
	if ( numFloatOut>0 )
		GeneralParamsMirror.SizesInfos.nbr_phys_float_outputs = numFloatOut;        
        GeneralParamsMirror.SizesInfos.nbr_symbols = pSizesInfos->nbr_bits + pSizesInfos->nbr_words ;
        GeneralParamsMirror.SizesInfos.nbr_symbols += pSizesInfos->nbr_timers + pSizesInfos->nbr_monostables ;
        GeneralParamsMirror.SizesInfos.nbr_symbols += pSizesInfos->nbr_counters + pSizesInfos->nbr_timers_iec ;
        GeneralParamsMirror.SizesInfos.nbr_symbols += pSizesInfos->nbr_phys_inputs + pSizesInfos->nbr_phys_outputs ;
        GeneralParamsMirror.SizesInfos.nbr_symbols += pSizesInfos->nbr_phys_words_inputs + pSizesInfos->nbr_phys_words_outputs;
        GeneralParamsMirror.SizesInfos.nbr_symbols += pSizesInfos->nbr_phys_float_inputs + pSizesInfos->nbr_phys_float_outputs + NBR_ERROR_BITS_DEF ;
	if (numSymbols < GeneralParamsMirror.SizesInfos.nbr_symbols ) {  GeneralParamsMirror.SizesInfos.nbr_symbols = numSymbols;  }
    
	#endif
}
