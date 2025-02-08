/* Classic Ladder Project */
/* Copyright (C) 2001-2009 Marc Le Douarain */
/* http://membres.lycos.fr/mavati/classicladder/ */
/* http://www.sourceforge.net/projects/classicladder */
/* February 2001 */
/* --------------------------- */
/* Alloc/free global variables */
/* --------------------------- */
/* This library is free software; you can redistribute it and/or */
/* modify it under the terms of the GNU Lesser General Public */
/* License as published by the Free Software Foundation; either */
/* version 2.1 of the License, or (at your option) any later version. */

/* This library is distributed in the hope that it will be useful, */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU */
/* Lesser General Public License for more details. */

/* You should have received a copy of the GNU Lesser General Public */
/* License along with this library; if not, write to the Free Software */
/* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. */

// this code has been highly modified for EMC 2
// EMC uses RTAPI realtime code to allocate shared memory / run calculations
// and HAL code for input/output 'pins' to other programs
// this adaptation was started Jan 2008 by Chis Morley
// see EMC_readme for more info

#ifndef MODULE
#include <stdio.h>
#include <stdlib.h>
#endif

//for emc next 2 lines
#include "rtapi.h"
#include "rtapi_string.h"
//#include <linux/string.h>

#include "classicladder.h"
#ifndef RTAPI //for EMC :realtime has no directory access
#include "files.h" 
#endif
#include "calc.h"
#include "vars_access.h"
#include "manager.h"
#include "calc_sequential.h"
#include "symbols.h"
//#include "vars_system.h"


#ifdef GTK_INTERFACE
#include <gtk/gtk.h>
#include "classicladder_gtk.h"
#include "manager_gtk.h"
#include "symbols_gtk.h"
//#include <gtk/gtk.h>
#endif




#ifdef HAL_SUPPORT
#include "rtapi.h"
#include "hal.h"
#define CL_SHMEM_KEY 0x434C522b // "CLR+"
int compId;
static int ShmemId;
#endif

StrRung * RungArray;
TYPE_FOR_BOOL_VAR * VarArray;
//unsigned char * LogVarArray;
int * VarWordArray;
double * VarFloatArray;
#ifdef OLD_TIMERS_MONOS_SUPPORT
StrTimer * TimerArray;
StrMonostable * MonostableArray;
#endif
StrCounter * CounterArray;
StrTimerIEC * NewTimerArray;
StrArithmExpr * ArithmExpr;
StrInfosGene * InfosGene;
StrSection * SectionArray;
#ifdef SEQUENTIAL_SUPPORT
StrSequential * Sequential;
#endif
StrSymbol * SymbolArray;

#ifdef GTK_INTERFACE
/* used for the editor */
StrEditRung EditDatas;
StrArithmExpr * EditArithmExpr;
#endif


// Default sizes values
// and variable used to store parameters before malloc()/init_rt_thread done!
// After init, we used it again to store the configuration modifications !!!
// The real values allocated are in InfosGene->GeneralParams.SizesInfos...
StrGeneralParams GeneralParamsMirror = {
	.SizesInfos.nbr_rungs = NBR_RUNGS_DEF,
	.SizesInfos.nbr_bits = NBR_BITS_DEF,
	.SizesInfos.nbr_words = NBR_WORDS_DEF,
#ifdef OLD_TIMERS_MONOS_SUPPORT
	.SizesInfos.nbr_timers = NBR_TIMERS_DEF,
	.SizesInfos.nbr_monostables = NBR_MONOSTABLES_DEF,
#endif
	.SizesInfos.nbr_counters = NBR_COUNTERS_DEF,
	.SizesInfos.nbr_timers_iec = NBR_TIMERS_IEC_DEF,
	.SizesInfos.nbr_phys_inputs = NBR_PHYS_INPUTS_DEF,
	.SizesInfos.nbr_phys_outputs = NBR_PHYS_OUTPUTS_DEF,
	.SizesInfos.nbr_arithm_expr = NBR_ARITHM_EXPR_DEF,
	.SizesInfos.nbr_sections = NBR_SECTIONS_DEF,
	.SizesInfos.nbr_symbols = NBR_SYMBOLS_DEF,
	.SizesInfos.nbr_phys_words_inputs = NBR_PHYS_WORDS_INPUTS_DEF,
	.SizesInfos.nbr_phys_words_outputs = NBR_PHYS_WORDS_OUTPUTS_DEF,
        .SizesInfos.nbr_phys_float_inputs = NBR_PHYS_FLOAT_INPUTS_DEF,
	.SizesInfos.nbr_phys_float_outputs = NBR_PHYS_FLOAT_OUTPUTS_DEF,
        .SizesInfos.nbr_error_bits = NBR_ERROR_BITS_DEF,
	.PeriodicRefreshMilliSecs = PERIODIC_REFRESH_MS_DEF
};



void InitInfosGene( void )
{
	InfosGene->LadderState = STATE_LOADING;
	InfosGene->UnderCalculationPleaseWait = FALSE;
	InfosGene->LadderStoppedToRunBack = FALSE;

	InfosGene->CmdRefreshVarsBits = FALSE;

	InfosGene->BlockWidth = BLOCK_WIDTH_DEF;
	InfosGene->BlockHeight = BLOCK_HEIGHT_DEF;
	InfosGene->PageWidth = 0;
	InfosGene->PageHeight = 0;
	InfosGene->TopRungDisplayed = 0;
	InfosGene->OffsetHiddenTopRungDisplayed = 0;
	InfosGene->OffsetCurrentRungDisplayed = 0;
	InfosGene->VScrollValue = 0;
	InfosGene->HScrollValue = 0;
	InfosGene->HeaderLabelCommentHeight = BLOCK_HEIGHT_DEF*65/100;

	InfosGene->DurationOfLastScan = 0;
	InfosGene->CurrentSection = 0;

	InitIOConf( );
	InfosGene->AskConfirmationToQuit = FALSE;
	InfosGene->HasBeenModifiedForExitCode = FALSE;
	InfosGene->DisplaySymbols = TRUE;

	InfosGene->AskToConfHard = FALSE;
	//InfosGene->LogContentModified = FALSE;
	//InfosGene->DefaultLogListModified = FALSE;
	InfosGene->HardwareErrMsgToDisplay[ 0 ] = '\0'; //no error for now!
}


// Classicladder_allocAll() is used by realtime module and userspace program
// the RTAPI define is used to select which allocation is done
//           ***REALTIME***
// -module_hal.c copies any changes to the number of elements into GeneralParamsMirror structure
// then calls Classicladder_Alloc() , which uses that info to calculate how much shared memory to reserve 
// -register the realtime side of shared memory
// -copies GeneralPararsmirror into GeneralParams now that the number of elements are set
// -set each element pointer to it's realtime shared memory address
// -Initialize realtime Infosgene and returns
//          ***USERSPACE***
// -regester user space side of shared memory
// -check that realtime shared memory has been done 
// -set each element pointer to it's user space shared memory address
// -Initialize user space Infosgene and return

int ClassicLadder_AllocAll()
{
   	 unsigned char *pByte; 
   	 unsigned long bytes = sizeof(StrInfosGene) + sizeof(long);
   	 unsigned long *shmBase;
 	 plc_sizeinfo_s *pSizesInfos;

#ifdef RTAPI // for realtime
    int numBits, numWords, numFloats;
    pSizesInfos = &GeneralParamsMirror.SizesInfos;
    // Calculate SHMEM size.
    numBits = pSizesInfos->nbr_bits + pSizesInfos->nbr_phys_inputs + pSizesInfos->nbr_phys_outputs + pSizesInfos->nbr_error_bits;
    numWords = pSizesInfos->nbr_words+pSizesInfos->nbr_phys_words_inputs+pSizesInfos->nbr_phys_words_outputs;
    numFloats = pSizesInfos->nbr_phys_float_inputs+pSizesInfos->nbr_phys_float_outputs;
#ifdef SEQUENTIAL_SUPPORT
    numBits += NBR_STEPS;
    numWords += NBR_STEPS;
#endif
    bytes += pSizesInfos->nbr_rungs * sizeof(StrRung);
    bytes += pSizesInfos->nbr_timers * sizeof(StrTimer);
    bytes += pSizesInfos->nbr_monostables * sizeof(StrMonostable);
    bytes += pSizesInfos->nbr_counters * sizeof(StrCounter);
    bytes += pSizesInfos->nbr_timers_iec * sizeof(StrTimerIEC);
    bytes += pSizesInfos->nbr_arithm_expr * sizeof(StrArithmExpr);
    bytes += pSizesInfos->nbr_sections * sizeof(StrSection);
    bytes += pSizesInfos->nbr_symbols * sizeof(StrSymbol);
    
#ifdef SEQUENTIAL_SUPPORT
    bytes += sizeof(StrSequential);
#endif
    bytes += numWords * sizeof(int);
    bytes += numFloats * sizeof(double);
    bytes += numBits * sizeof(TYPE_FOR_BOOL_VAR);
    
    // Attach SHMEM with proper size.
    if ((ShmemId = rtapi_shmem_new(CL_SHMEM_KEY, compId, bytes)) < 0) 
              {
                rtapi_print_msg(RTAPI_MSG_DBG,"Failed to alloc shared memory (%x %d %lu) !\n",CL_SHMEM_KEY, compId, bytes);
               return FALSE;
              }
    rtapi_print_msg(RTAPI_MSG_INFO,"Shared memory:key- %x component id-%d # of bytes-%lu\n",CL_SHMEM_KEY, compId, bytes);
    // Map SHMEM (shared memory).
    if (rtapi_shmem_getptr(ShmemId, (void **) &shmBase) < 0) 
              {
  	       rtapi_print("Failed to map shared memory !\n");
  	       return FALSE;
              }
	
     shmBase[0] = CL_SHMEM_KEY;
     shmBase[1] = bytes;
     InfosGene = (StrInfosGene*)(shmBase+1);
     InfosGene->GeneralParams.SizesInfos = *pSizesInfos;
     memcpy( &InfosGene->GeneralParams, &GeneralParamsMirror, sizeof( StrGeneralParams ) );
     rtapi_print_msg(RTAPI_MSG_INFO,"INFO----REALTIME allocations for classicladder:\n");
#endif //end of realtime code

#ifndef RTAPI// for user space

	
    // Attach SHMEM with proper size.
    if ((ShmemId = rtapi_shmem_new(CL_SHMEM_KEY, compId, bytes)) < 0)
              {
               rtapi_print("Failed to alloc shared memory (%x %d %lu) !\n",
               CL_SHMEM_KEY, compId, bytes);
               return FALSE;
              }
     rtapi_print_msg(RTAPI_MSG_INFO,"Shared memory:key- %x component id-%d # of bytes-%lu\n",
     CL_SHMEM_KEY, compId, bytes);
    // Map SHMEM.
    if (rtapi_shmem_getptr(ShmemId, (void **) &shmBase) < 0) 
          {
           rtapi_print("Failed to map shared memory !\n");
           return FALSE;
          }

    // Check signature written by RT module to make sure we have the
    // right region and RT module is loaded.
    rtapi_print_msg(RTAPI_MSG_INFO,"INFO----USERSPACE allocations for classicladder\n");
    if (shmBase[0] != CL_SHMEM_KEY) 
          {
           rtapi_print("Shared memory conflict or RT component not loaded!\n");
           return FALSE;
          }
     bytes = shmBase[1];   
     InfosGene = (StrInfosGene*)(shmBase+1);
     pSizesInfos = &(InfosGene->GeneralParams.SizesInfos);

        // copy generalparams to gen paramsMirror so Config window displays properly
	memcpy(  &GeneralParamsMirror,&InfosGene->GeneralParams, sizeof( StrGeneralParams ) );
  	UpdateSizesOfConvVarNameTable();
#ifdef GTK_INTERFACE
	EditArithmExpr = (StrArithmExpr *)malloc(  pSizesInfos->nbr_arithm_expr * sizeof(StrArithmExpr) );
	if (!EditArithmExpr)
	{
		rtapi_print("Failed to alloc EditArithmExpr !\n");
		return FALSE;
	}
#endif
#endif

// the rest is for both realtime and userspace program

      rtapi_print_msg(RTAPI_MSG_INFO,"Sizes: rungs- %d bits- %d words- %d timers- %d mono- %d count- %d IEC timers- %d\n HAL Bin- %d HAL Bout- %d expressions- %d sections- %d symbols - %d\n  s32in - %d s32out- %d Error bits-%d\n",
        pSizesInfos->nbr_rungs,
        pSizesInfos->nbr_bits,
        pSizesInfos->nbr_words,
        pSizesInfos->nbr_timers,
        pSizesInfos->nbr_monostables,
        pSizesInfos->nbr_counters,
	pSizesInfos->nbr_timers_iec,
        pSizesInfos->nbr_phys_inputs,
        pSizesInfos->nbr_phys_outputs,
        pSizesInfos->nbr_arithm_expr,
        pSizesInfos->nbr_sections,
        pSizesInfos->nbr_symbols,
	pSizesInfos->nbr_phys_words_inputs,
	pSizesInfos->nbr_phys_words_outputs,
        pSizesInfos->nbr_error_bits);

    	// Set global SHMEM pointers for each element
    pByte = (unsigned char *) InfosGene;
	   pByte += sizeof(StrInfosGene);
    RungArray = (StrRung *) pByte;
 	   pByte += pSizesInfos->nbr_rungs * sizeof(StrRung);
    TimerArray = (StrTimer *) pByte;	
   	   pByte += pSizesInfos->nbr_timers * sizeof(StrTimer);
    MonostableArray = (StrMonostable *) pByte;
   	   pByte += pSizesInfos->nbr_monostables * sizeof(StrMonostable);
    CounterArray= (StrCounter *) pByte;
	   pByte += pSizesInfos->nbr_counters * sizeof(StrCounter);	
    NewTimerArray = (StrTimerIEC *) pByte;
	   pByte += pSizesInfos->nbr_timers_iec * sizeof(StrTimerIEC);
           ArithmExpr = (StrArithmExpr *) pByte;	
 	   pByte += pSizesInfos->nbr_arithm_expr * sizeof(StrArithmExpr);
    SectionArray = (StrSection *) pByte;	
 	   pByte += pSizesInfos->nbr_sections * sizeof(StrSection);
    SymbolArray = (StrSymbol *) pByte;	
	   pByte += pSizesInfos->nbr_symbols * sizeof(StrSymbol);
#ifdef SEQUENTIAL_SUPPORT
    Sequential = (StrSequential *) pByte;	
  	  pByte += sizeof(StrSequential);
#endif
    VarWordArray = (int *) pByte;
 	   pByte += SIZE_VAR_WORD_ARRAY * sizeof(int);
    VarFloatArray =(double *) pByte;
           pByte += SIZE_VAR_FLOAT_ARRAY * sizeof(double);
    // Allocate last for alignment reasons.
    VarArray = (TYPE_FOR_BOOL_VAR *) pByte;

	InitInfosGene( );

return TRUE;
}

void ClassicLadder_FreeAll(char CleanAndRemoveTmpDir)
{
	(void)CleanAndRemoveTmpDir;
#ifdef GTK_INTERFACE
	if (EditArithmExpr)
		free(EditArithmExpr);
#endif
#ifndef RTAPI // there is no directory in realtime!
if ( CleanAndRemoveTmpDir )
		CleanTmpLadderDirectory( TRUE/*RemoveTmpDirAtEnd*/ );
#endif
#ifdef HAL_SUPPORT
	rtapi_shmem_delete(ShmemId,compId);
#endif	
}

void ClassicLadder_InitAllDatas( void )
{
	InitVars();
#ifdef OLD_TIMERS_MONOS_SUPPORT
	InitTimers();
	InitMonostables();
#endif
	InitCounters();
	InitTimersIEC();
	InitArithmExpr();
	InitRungs();
	InitSections( );
#ifdef SEQUENTIAL_SUPPORT
	InitSequential( );
#endif
	InitSymbols( );
	//InitSystemVars( TRUE );

	//InitLog( );
	//InitVarsArrayLogTags( );

	InfosGene->AskConfirmationToQuit = FALSE;
	InfosGene->HasBeenModifiedForExitCode = FALSE;
}

