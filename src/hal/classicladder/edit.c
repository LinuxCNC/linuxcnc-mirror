/* Classic Ladder Project */
/* Copyright (C) 2001-2010 Marc Le Douarain */
/* http://membres.lycos.fr/mavati/classicladder/ */
/* http://www.sourceforge.net/projects/classicladder */
/* May 2001 */
/* ------ */
/* Editor */
/* ------ */
/* This part of the editor is the one who will not change even if if we use */
/* another gui instead of gtk... who know? */
/* ------------------------------------------------------------- */
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gtk/gtk.h>
#include <libintl.h> // i18n
#include <locale.h> // i18n

#include "classicladder.h"
#include "global.h"
#include "drawing.h"
#include "editproperties_gtk.h"
#include "calc.h"
#include "files.h"
#include "arithm_eval.h"
#include "classicladder_gtk.h"
#include "manager.h"
#ifdef SEQUENTIAL_SUPPORT
#include "edit_sequential.h"
#endif
#include "symbols.h"
#include "vars_names.h"
#include "edit_copy.h"
#include "edit.h"

#include <rtapi_string.h>

/* This array give for each special elements the size used */
#define TYPEELERULE 0
#define XSIZEELERULE 1
#define YSIZEELERULE 2
static short int RulesForSpecialElements[][3] =
            { {ELE_TIMER , 2, 2 } ,
              {ELE_MONOSTABLE, 2, 2 },
              {ELE_COUNTER, 2, 4 },
              {ELE_TIMER_IEC, 2, 2 },
              {ELE_COMPAR, 3, 1 },
              {ELE_OUTPUT_OPERATE, 3, 1 },
              {-1, -1, -1}/*end*/ };

// pointer on a string error explanation...
char * ErrorMessageVarParser = NULL;

// used to mark the functions blocs already used
#ifdef OLD_TIMERS_MONOS_SUPPORT
char NumTimersUsedInRung[ 256 ];
char NumMonostablesUsedInRung[ 256 ];
#endif
char NumCountersUsedInRung[ 256 ];
char NumNewTimersUsedInRung[ 256 ];


int ConvBaseInTextToId(char * text)
{
	int ScanBase = 0;
	int BaseFound = BASE_SECS;
	do
	{
		if (strcmp(text,CorresDatasForBase[ScanBase].ParamSelect)==0)
			BaseFound = ScanBase;
		ScanBase++;
	}
	while(ScanBase<NBR_TIMEBASES);
	return BaseFound;
}
int ConvTimerModeInTextToId(char * text)
{
	int ScanMode = 0;
	int TimerModeFound = TIMER_IEC_MODE_ON;
	do
	{
		if (strcmp(text,TimersModesStrings[ ScanMode ])==0)
			TimerModeFound = ScanMode;
		ScanMode++;
	}
	while(ScanMode<NBR_TIMERSMODES);
	return TimerModeFound;
}

int ConvLabelToNumRung(char * LabelName)
{
	int ScanRung = 0;
	int RungFound = -1;
	int Done = FALSE;
	ScanRung = InfosGene->FirstRung;
	do
	{
		if (strcmp(RungArray[ScanRung].Label,LabelName)==0)
			RungFound = ScanRung;
		if (ScanRung == InfosGene->LastRung)
			Done = TRUE;
		else
			ScanRung = RungArray[ScanRung].NextRung;
	}
	while(!Done);
	return RungFound;
}

void LoadElementProperties(StrElement * Element)
{
	char TextToWrite[100];
	int NumParam;
#ifdef OLD_TIMERS_MONOS_SUPPORT
	StrTimer * Timer = NULL;
	StrMonostable * Monostable = NULL;
#endif
	StrCounter * Counter = NULL;
	StrTimerIEC * TimerIEC = NULL;
	for(NumParam=0;NumParam<NBR_PARAMS_PER_OBJ;NumParam++)
		SetProperty(NumParam,"---","",FALSE);
	if (Element)
	{
		switch(Element->Type)
		{
			case ELE_INPUT:
			case ELE_INPUT_NOT:
			case ELE_RISING_INPUT:
			case ELE_FALLING_INPUT:
			case ELE_OUTPUT:
			case ELE_OUTPUT_NOT:
			case ELE_OUTPUT_SET:
			case ELE_OUTPUT_RESET:
				rtapi_strxcpy(TextToWrite,CreateVarName(Element->VarType,Element->VarNum, InfosGene->DisplaySymbols));
//				CreateVarNameForElement( TextToWrite, sizeof(TextToWrite), Element, InfosGene->DisplaySymbols );
				SetProperty(0,_("Variable"),TextToWrite,TRUE);
				break;
			case ELE_OUTPUT_JUMP:
				SetProperty(0,_("JumpToLabel"),RungArray[Element->VarNum].Label,TRUE);
				break;
			case ELE_OUTPUT_CALL:
				snprintf(TextToWrite, sizeof(TextToWrite),"%d",Element->VarNum);
				SetProperty(0,_("Sub-Routine"),TextToWrite,TRUE);
				break;
#ifdef OLD_TIMERS_MONOS_SUPPORT
			case ELE_TIMER:
				Timer = &TimerArray[Element->VarNum];
				snprintf(TextToWrite, sizeof(TextToWrite),"%d",Element->VarNum);
				SetProperty(0,_("TimerNbr"),TextToWrite,FALSE);
				snprintf(TextToWrite, sizeof(TextToWrite),"%s",CorresDatasForBase[ ConvBaseInMilliSecsToId(Timer->Base) ].ParamSelect);
				SetProperty(1,_("Base"),TextToWrite,FALSE);
				snprintf(TextToWrite, sizeof(TextToWrite),"%d",Timer->Preset/Timer->Base);
				SetProperty(2,_("Preset"),TextToWrite,TRUE);
				break;
			case ELE_MONOSTABLE:
				Monostable = &MonostableArray[Element->VarNum];
				snprintf(TextToWrite, sizeof(TextToWrite),"%d",Element->VarNum);
				SetProperty(0,_("MonostNbr"),TextToWrite,FALSE);
				snprintf(TextToWrite, sizeof(TextToWrite),"%s",CorresDatasForBase[ ConvBaseInMilliSecsToId(Monostable->Base) ].ParamSelect);
				SetProperty(1,_("Base"),TextToWrite,FALSE);
				snprintf(TextToWrite, sizeof(TextToWrite),"%d",Monostable->Preset/Monostable->Base);
				SetProperty(2,_("Preset"),TextToWrite,TRUE);
				break;
#endif
			case ELE_COUNTER:
				Counter = &CounterArray[Element->VarNum];
				snprintf(TextToWrite, sizeof(TextToWrite),"%d",Element->VarNum);
				SetProperty(0,_("CounterNbr"),TextToWrite,FALSE);
				snprintf(TextToWrite, sizeof(TextToWrite),"%d",Counter->Preset);
				SetProperty(1,_("Preset"),TextToWrite,TRUE);
				break;
			case ELE_TIMER_IEC:
				TimerIEC = &NewTimerArray[Element->VarNum];
				snprintf(TextToWrite, sizeof(TextToWrite),"%d",Element->VarNum);
				SetProperty(0,_("TimerNbr"),TextToWrite,FALSE);
				snprintf(TextToWrite, sizeof(TextToWrite),"%s",CorresDatasForBase[ ConvBaseInMilliSecsToId(TimerIEC->Base) ].ParamSelect);
				SetProperty(1,_("Base"),TextToWrite,FALSE);
				snprintf(TextToWrite, sizeof(TextToWrite),"%d",TimerIEC->Preset);
				SetProperty(2,_("Preset"),TextToWrite,TRUE);
				snprintf(TextToWrite, sizeof(TextToWrite),"%s",TimersModesStrings[ (int)TimerIEC->TimerMode ]);
				SetProperty(3,_("TimerMode"),TextToWrite,FALSE);
				break;
			case ELE_COMPAR:
			case ELE_OUTPUT_OPERATE:
				rtapi_strxcpy(TextToWrite,DisplayArithmExpr(EditArithmExpr[Element->VarNum].Expr,InfosGene->DisplaySymbols));
				SetProperty(0,_("Expression"),TextToWrite,TRUE);
				break;
		}
	}
}

char * GetElementPropertiesForStatusBar(StrElement * Element)
{
//	char BufTxt[ARITHM_EXPR_SIZE+30];
//	char BufTxt2[ARITHM_EXPR_SIZE+30];
	static char PropertiesText[ARITHM_EXPR_SIZE+100];
	rtapi_strxcpy( PropertiesText, "" );
#ifdef OLD_TIMERS_MONOS_SUPPORT
	StrTimer * Timer = NULL;
	StrMonostable * Monostable = NULL;
#endif
	StrCounter * Counter = NULL;
	StrTimerIEC * TimerIEC = NULL;
	if (Element)
	{
		switch(Element->Type)
		{
			case ELE_INPUT:
			case ELE_INPUT_NOT:
			case ELE_RISING_INPUT:
			case ELE_FALLING_INPUT:
			case ELE_OUTPUT:
			case ELE_OUTPUT_NOT:
			case ELE_OUTPUT_SET:
			case ELE_OUTPUT_RESET:
				snprintf(PropertiesText, sizeof(PropertiesText), _("Variable: %s    Hal sig: %s"),CreateVarName(Element->VarType,Element->VarNum,InfosGene->DisplaySymbols),ConvVarNameToHalSigName(CreateVarName(Element->VarType,Element->VarNum,InfosGene->DisplaySymbols)));
//				CreateVarNameForElement( BufTxt, Element, TRUE/*SymbolsVarsNamesIfAvail*/ );
//				CreateVarNameForElement( BufTxt2, Element, FALSE/*SymbolsVarsNamesIfAvail*/ );
//				snprintf(PropertiesText, sizeof(PropertiesText), "Variable: %s (%s)",BufTxt,BufTxt2);
				break;
			case ELE_OUTPUT_JUMP:
				snprintf(PropertiesText, sizeof(PropertiesText), _("Label: %s"), RungArray[Element->VarNum].Label);
				break;
			case ELE_OUTPUT_CALL:
				snprintf(PropertiesText, sizeof(PropertiesText), _("Sub-Routine: %d"), Element->VarNum);
				break;
#ifdef OLD_TIMERS_MONOS_SUPPORT
			case ELE_TIMER:
				Timer = &TimerArray[Element->VarNum];
				snprintf(PropertiesText, sizeof(PropertiesText), _("%cT%d: Preset=%d Base=%s"), '%', Element->VarNum, Timer->Preset/Timer->Base, CorresDatasForBase[ ConvBaseInMilliSecsToId(Timer->Base) ].ParamSelect);
				break;
			case ELE_MONOSTABLE:
				Monostable = &MonostableArray[Element->VarNum];
				snprintf(PropertiesText, sizeof(PropertiesText), _("%cM%d: Preset=%d Base=%s"), '%', Element->VarNum, Monostable->Preset/Monostable->Base, CorresDatasForBase[ ConvBaseInMilliSecsToId(Monostable->Base) ].ParamSelect);
				break;
#endif
			case ELE_COUNTER:
				Counter = &CounterArray[Element->VarNum];
				snprintf(PropertiesText, sizeof(PropertiesText), _("%cC%d: Preset=%d"), '%', Element->VarNum, Counter->Preset);
				break;
			case ELE_TIMER_IEC:
				TimerIEC = &NewTimerArray[Element->VarNum];
				snprintf(PropertiesText, sizeof(PropertiesText), _("%cTM%d: Preset=%d Base=%s Mode=%s"), '%', Element->VarNum, TimerIEC->Preset, CorresDatasForBase[ ConvBaseInMilliSecsToId(TimerIEC->Base) ].ParamSelect, TimersModesStrings[ (int)TimerIEC->TimerMode ]);
				break;
			case ELE_COMPAR:
			case ELE_OUTPUT_OPERATE:
				snprintf(PropertiesText, sizeof(PropertiesText),_("HAL sig: %s"),FirstVariableInArithm(DisplayArithmExpr(ArithmExpr[Element->VarNum].Expr,0)));
//				//DisplayArithmExpr() returns pointer on static buffer 
//				strcpy( BufTxt, DisplayArithmExpr(ArithmExpr[Element->VarNum].Expr,TRUE/*SymbolsVarsNamesIfAvail*/) );
//				snprintf(PropertiesText, sizeof(PropertiesText), "Expression: %s (%s)", BufTxt, DisplayArithmExpr(ArithmExpr[Element->VarNum].Expr,FALSE/*SymbolsVarsNamesIfAvail*/));
				break;
		}
	}
	return PropertiesText;
}
/* Convert a string to a number if >=Min and <=Max */
/* return TRUE if okay */
int TextToNumber(char * text,int ValMin,int ValMaxi,int *ValFound)
{
	int IsOk = TRUE;
	int Value;
	Value = atoi(text);
	if ( (Value<ValMin) || (Value>ValMaxi) )
		IsOk = FALSE;
	if (IsOk)
		*ValFound = Value;
	return IsOk;
}

/* Convert a string of arithmetic expression for the arithm_eval format : */
/* Variables becomes @type/offset@ or @type/offset[indextype/indexoffset]@*/
/* Then verify all the expression using arithm_eval */
/* return pointer on the new string converted and verified if okay, else NULL */
char * TextParserForArithmExpr(char * text, int TypeElement)
{
	static char NewExpr[100];
	char Buffer[20];
	int ItIsOk = TRUE;
	int VarType;
	int VarOffset;
	char * ptr = text;
	int StringLength = 0;
	char VarIndexedIsFound = FALSE;
	ErrorMessageVarParser = NULL;

	if ( text[0]=='\0' )
		return NULL;

	rtapi_strxcpy(NewExpr,"");
	/* Convert expression for variables authorized */
	do
	{
		char SimpleCharCopy = FALSE;

		char SymbolBuff[ LGT_SYMBOL_STRING+10 ];
		char SymbolFound = FALSE;
		int SymbolLength = 1;

//printf("current ptr = '%c'\n", *ptr );

		// verify if it seems to be a symbol name...
		if ( ( *ptr>='A' && *ptr<='Z' ) || ( *ptr>='a' && * ptr<='z' ) )
		{
			char * End = ptr;
			char CharIn = FALSE;
			// possible end here ?
			do
			{
				CharIn = FALSE;
				End++;
				if ( *End==VAR_ATTRIBUTE_SEP || ( *End>='A' && *End<='Z' ) || ( *End>='a' && *End<='z' ) || ( *End>='0' && *End<='9' ) || *End=='_' )
					CharIn = TRUE;
				if ( CharIn )
					SymbolLength++;
//printf("--> %c / lgt=%d / In=%d\n", *End, SymbolLength, CharIn );
			}
			while ( (*End!='\0' ) && CharIn );
			strncpy( SymbolBuff, ptr, SymbolLength );
			SymbolBuff[ SymbolLength ] = '\0'; 
//printf("symbol search on this string : %s - lgt=%d (text=%s)\n", SymbolBuff, SymbolLength, ptr );
			if ( ConvSymbolToVarName( SymbolBuff )!=NULL )
				SymbolFound = TRUE;
		}

		/* Verify if it is a variable (character and number) (or a symbol already found) */
//		if ( SymbolFound==TRUE || ( *ptr=='%' && ( ( *(ptr+2)>='0' && *(ptr+2)<='9' ) || ( *(ptr+3)>='0' && *(ptr+3)<='9' ) ) ) )
		// Call variable parser if symbol found -or- '%' character
		if ( SymbolFound==TRUE || *ptr=='%' )
		{
//printf("calling parser var...\n");
			if (TextParserForAVar( SymbolFound==TRUE?SymbolBuff:ptr,&VarType,&VarOffset,&StringLength,FALSE/*PartialNames*/))
			{
//printf( "parser var give => %d/%d length=%d\n", VarType, VarOffset, StringLength );
				if ( VarIndexedIsFound==FALSE )
					rtapi_strxcat( NewExpr, "@" );
				snprintf(Buffer, sizeof(Buffer),"%d/%d",VarType,VarOffset);
				rtapi_strxcat(NewExpr,Buffer);
				ptr = ptr + (SymbolFound==TRUE?SymbolLength:StringLength);
				if ( *ptr!='[' && VarIndexedIsFound==FALSE )
					rtapi_strxcat( NewExpr, "@" );

				if ( TEST_VAR_IS_A_BOOL( VarType,VarOffset ) )

				{
					ItIsOk = FALSE;
					ErrorMessageVarParser = _("Incompatible type of variable (must be an integer!)");
				}
			}
			else
			{
//printf("parser var give FALSE!\n");
				SimpleCharCopy = TRUE;
			}
		}
		else
		{
			SimpleCharCopy = TRUE;
		}

		if ( SimpleCharCopy )
		{
//printf("copy:'%c'.\n", *ptr);
			snprintf(Buffer, sizeof(Buffer),"%c",*ptr);
			rtapi_strxcat(NewExpr, Buffer);
			// end of variable mark to add now after an indexed one !
			if ( *ptr==']' )
			{
				rtapi_strxcat( NewExpr, "@" );
				VarIndexedIsFound = FALSE;
			}
			if ( *ptr=='[' )
				VarIndexedIsFound = TRUE;
			ptr++;
		}
	}
	while( (*ptr) && ItIsOk );

	/* Verify length of the expression */
	if (ItIsOk)
	{
		if (strlen(NewExpr)>=ARITHM_EXPR_SIZE-1)
		{
			NewExpr[ ARITHM_EXPR_SIZE-1 ] = '\0';
			printf(_("New expr=%s\n"), NewExpr );
			ItIsOk = FALSE;
			ErrorMessageVarParser = _("Expression too long");
		}
	}
//printf("Parser Arithm Expr; ItIsOk=%d ; OriExpr=%s ; NewExpr=%s\n",ItIsOk, text, NewExpr);
	/* Verify expression converted */
	if (ItIsOk)
	{
		if (TypeElement==ELE_OUTPUT_OPERATE)
			ErrorMessageVarParser = VerifySyntaxForMakeCalc(NewExpr);
		else
			ErrorMessageVarParser = VerifySyntaxForEvalCompare(NewExpr);
		if (ErrorMessageVarParser)
			ItIsOk = FALSE;
	}
	/* Error Message */
	if (ErrorMessageVarParser)
	{
		ShowMessageBox(_("Error"),ErrorMessageVarParser,_("Ok"));
	}
	/* Give result */
	if (ItIsOk)
		return NewExpr;
	else
		return NULL;
}

void SaveElementProperties()
{
	int IdBase;
	int Preset;
	int NumRungToJump;
#ifdef OLD_TIMERS_MONOS_SUPPORT
	StrTimer * Timer = NULL;
	StrMonostable * Monostable = NULL;
#endif
	StrCounter * Counter = NULL;
	StrTimerIEC * TimerIEC = NULL;
	char * NewArithmExpr;
	int SubRoutineToCall;
	int VarTypeEntered,VarNumEntered;
	int IndexedVarTypeEntered=-1,IndexedVarNumEntered=0;

#ifdef SEQUENTIAL_SUPPORT
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_SEQUENTIAL )
	{
		SaveSeqElementProperties( );
		return;
	}
#endif

	if (EditDatas.ElementUnderEdit)
	{
		switch(EditDatas.ElementUnderEdit->Type)
		{
			case ELE_INPUT:
			case ELE_INPUT_NOT:
			case ELE_RISING_INPUT:
			case ELE_FALLING_INPUT:
			case ELE_OUTPUT:
			case ELE_OUTPUT_NOT:
			case ELE_OUTPUT_SET:
			case ELE_OUTPUT_RESET:
				if ( TextParserForAVar( GetProperty(0),&VarTypeEntered,&VarNumEntered,NULL,FALSE/*PartialNames*/) )
				{
					char cCorrectRules = TRUE;
					char * pStartIndexed = GetProperty(0);
					// verify if an index seems to be present... 
					while( *pStartIndexed!='\0' && *pStartIndexed!='[' )
					{
						pStartIndexed++;
					}
					if ( *pStartIndexed++ =='[' )
					{
						if ( pStartIndexed[strlen(pStartIndexed)-1]==']')
							pStartIndexed[strlen(pStartIndexed)-1] = '\0';
						if ( TextParserForAVar( pStartIndexed,&IndexedVarTypeEntered,&IndexedVarNumEntered,NULL,FALSE/*PartialNames*/) )
						{
							if ( TEST_VAR_IS_A_BOOL( IndexedVarTypeEntered, IndexedVarNumEntered ) )
							{
								ShowMessageBoxError( _("Incompatible type of variable for index (must be an integer!)") );
								cCorrectRules = FALSE;
							}
						}
						else
						{
							ShowMessageBoxError( _("Parser error for indexed variable !") );
							cCorrectRules = FALSE;
						}
					}
					if ( !TEST_VAR_IS_A_BOOL( VarTypeEntered,VarNumEntered ) )
					{
						ShowMessageBoxError( _("You must select a boolean variable !") );
						cCorrectRules = FALSE;
					}
					if ( (EditDatas.ElementUnderEdit->Type==ELE_OUTPUT ||
							 EditDatas.ElementUnderEdit->Type==ELE_OUTPUT_NOT ||
							 EditDatas.ElementUnderEdit->Type==ELE_OUTPUT_SET ||
							 EditDatas.ElementUnderEdit->Type==ELE_OUTPUT_RESET )
							&& !TestVarIsReadWrite( VarTypeEntered, VarNumEntered ) )
					{
						ShowMessageBoxError( _("You must select a read/write variable for a coil!") );
						cCorrectRules = FALSE;
					}
					if ( cCorrectRules )
					{
						EditDatas.ElementUnderEdit->VarType = VarTypeEntered;
						EditDatas.ElementUnderEdit->VarNum = VarNumEntered;
						EditDatas.ElementUnderEdit->IndexedVarType = IndexedVarTypeEntered;
						EditDatas.ElementUnderEdit->IndexedVarNum = IndexedVarNumEntered;
					}
				}
				else
				{
					if (ErrorMessageVarParser)
						ShowMessageBoxError( ErrorMessageVarParser );
					else
						ShowMessageBoxError( _("Unknown variable...") );
				}
				break;
#ifdef OLD_TIMERS_MONOS_SUPPORT
			case ELE_TIMER:
				TextToNumber(GetProperty(0),0,NBR_TIMERS-1,&EditDatas.ElementUnderEdit->VarNum);
				Timer = &TimerArray[EditDatas.ElementUnderEdit->VarNum];
				IdBase = ConvBaseInTextToId(GetProperty(1));
				Timer->Base = CorresDatasForBase[IdBase].ValueInMS;
				rtapi_strxcpy(Timer->DisplayFormat,CorresDatasForBase[IdBase].DisplayFormat);
				if (TextToNumber(GetProperty(2),0,999,&Preset))
					Timer->Preset = Preset * Timer->Base;
				break;
			case ELE_MONOSTABLE:
				TextToNumber(GetProperty(0),0,NBR_MONOSTABLES-1,&EditDatas.ElementUnderEdit->VarNum);
				Monostable = &MonostableArray[EditDatas.ElementUnderEdit->VarNum];
				IdBase = ConvBaseInTextToId(GetProperty(1));
				Monostable->Base = CorresDatasForBase[IdBase].ValueInMS;
				rtapi_strxcpy(Monostable->DisplayFormat,CorresDatasForBase[IdBase].DisplayFormat);
				if (TextToNumber(GetProperty(2),0,999,&Preset))
					Monostable->Preset = Preset * Monostable->Base;
				break;
#endif
			case ELE_COUNTER:
				TextToNumber(GetProperty(0),0,NBR_COUNTERS-1,&EditDatas.ElementUnderEdit->VarNum);
				Counter = &CounterArray[EditDatas.ElementUnderEdit->VarNum];
				if (TextToNumber(GetProperty(1),0,9999,&Preset))
					Counter->Preset = Preset;
				break;
			case ELE_TIMER_IEC:
				TextToNumber(GetProperty(0),0,NBR_TIMERS_IEC-1,&EditDatas.ElementUnderEdit->VarNum);
				TimerIEC = &NewTimerArray[EditDatas.ElementUnderEdit->VarNum];
				IdBase = ConvBaseInTextToId(GetProperty(1));
				TimerIEC->Base = CorresDatasForBase[IdBase].ValueInMS;
				rtapi_strxcpy(TimerIEC->DisplayFormat,CorresDatasForBase[IdBase].DisplayFormat);
				if (TextToNumber(GetProperty(2),0,9999,&Preset))
					TimerIEC->Preset = Preset;
				TimerIEC->TimerMode = ConvTimerModeInTextToId( GetProperty(3) );
				break;
			case ELE_OUTPUT_JUMP:
				NumRungToJump = ConvLabelToNumRung(GetProperty(0));
				if (NumRungToJump>=0)
					EditDatas.ElementUnderEdit->VarNum = NumRungToJump;
				break;
			case ELE_OUTPUT_CALL:
				if ( TextToNumber(GetProperty(0),0,NBR_SECTIONS-1,&SubRoutineToCall) )
				{
					if ( VerifyIfSubRoutineNumberExist( SubRoutineToCall ) )
						EditDatas.ElementUnderEdit->VarNum = SubRoutineToCall;
				}
				break;
			case ELE_COMPAR:
				NewArithmExpr = TextParserForArithmExpr(GetProperty(0), ELE_COMPAR);
				if (NewArithmExpr)
					rtapi_strxcpy(EditArithmExpr[EditDatas.ElementUnderEdit->VarNum].Expr,NewArithmExpr);
//keep old!				else
//keep old!					rtapi_strxcpy(EditArithmExpr[EditDatas.ElementUnderEdit->VarNum].Expr,"#"); //used but invalid!
				break;
			case ELE_OUTPUT_OPERATE:
				NewArithmExpr = TextParserForArithmExpr(GetProperty(0), ELE_OUTPUT_OPERATE);
//printf( "expr=%s\n",EditArithmExpr[EditDatas.ElementUnderEdit->VarNum].Expr );
				if (NewArithmExpr)
					rtapi_strxcpy(EditArithmExpr[EditDatas.ElementUnderEdit->VarNum].Expr,NewArithmExpr);
//keep old!				else
//keep old!					rtapi_strxcpy(EditArithmExpr[EditDatas.ElementUnderEdit->VarNum].Expr,"#"); //used but invalid!
				break;
		}
		/* display back to show what we have really understand... */
		LoadElementProperties(EditDatas.ElementUnderEdit);
	}
}

/* For editing, we do not touch directly the current arithm
expressions. We work on the edit ones. It is only when the
edited rung is applied that we copy the expressions edited */
void CopyCurrentArithmExpr()
{
	int NumExpr;
	for (NumExpr=0; NumExpr<NBR_ARITHM_EXPR; NumExpr++)
		rtapi_strxcpy(EditArithmExpr[NumExpr].Expr,ArithmExpr[NumExpr].Expr);
}
void ApplyNewArithmExpr()
{
	int NumExpr;
	for (NumExpr=0; NumExpr<NBR_ARITHM_EXPR; NumExpr++)
		rtapi_strxcpy(ArithmExpr[NumExpr].Expr,EditArithmExpr[NumExpr].Expr);
}
void CheckForFreeingArithmExpr(int PosiX,int PosiY)
{
	int TypeElement = EditDatas.Rung.Element[PosiX][PosiY].Type;
	if ( (TypeElement==ELE_COMPAR) || (TypeElement==ELE_OUTPUT_OPERATE) )
	{
		/* Freeing Expr */
		EditArithmExpr[ EditDatas.Rung.Element[PosiX][PosiY].VarNum ].Expr[0] = '\0';
	}
}
char CheckForAllocatingArithmExpr(int NumTypeEle, int PosiX,int PosiY)
{
	int NumExpr = 0;
	int Found = FALSE;
	char ResultOk = TRUE;
	if ( (NumTypeEle==ELE_COMPAR) || (NumTypeEle==ELE_OUTPUT_OPERATE) )
	{
		do
		{
			/* Expr free ? */
			if (EditArithmExpr[ NumExpr ].Expr[0]=='\0')
			{
				Found = TRUE;
				/* Allocate this expr for the operate/compar block ! */
				EditDatas.Rung.Element[PosiX][PosiY].VarNum = NumExpr;
				rtapi_strxcpy(EditArithmExpr[NumExpr].Expr,"#"); //used but invalid!
			}
			NumExpr++;
		}
		while( (NumExpr<NBR_ARITHM_EXPR) && (!Found) );
		if ( !Found )
			ResultOk = FALSE;
	}
	return ResultOk;
}

void SetUsedStateFunctionBlock( int Type, int Num, char Val )
{
	switch( Type )
	{
#ifdef OLD_TIMERS_MONOS_SUPPORT
		case ELE_TIMER:
			NumTimersUsedInRung[Num] = Val;
			break;
		case ELE_MONOSTABLE:
			NumMonostablesUsedInRung[Num] = Val;
			break;
#endif
		case ELE_COUNTER:
			NumCountersUsedInRung[Num] = Val;
			break;
		case ELE_TIMER_IEC:
			NumNewTimersUsedInRung[Num] = Val;
			break;
	}
}
// update the lists with the functions blocks number already used in the rungs defined
void CopyUsedFonctionBlockBeforeEdit( )
{
	int ScanBlock;
	int ScanRung;
#ifdef OLD_TIMERS_MONOS_SUPPORT
	for( ScanBlock=0; ScanBlock<NBR_TIMERS; ScanBlock++ )
		NumTimersUsedInRung[ ScanBlock ] = FALSE;
	for( ScanBlock=0; ScanBlock<NBR_MONOSTABLES; ScanBlock++ )
		NumMonostablesUsedInRung[ ScanBlock ] = FALSE;
#endif
	for( ScanBlock=0; ScanBlock<NBR_COUNTERS; ScanBlock++ )
		NumCountersUsedInRung[ ScanBlock ] = FALSE;
	for( ScanBlock=0; ScanBlock<NBR_TIMERS_IEC; ScanBlock++ )
		NumNewTimersUsedInRung[ ScanBlock ] = FALSE;
	// now search in the rungs defined
	for (ScanRung=0;ScanRung<NBR_RUNGS;ScanRung++)
	{
		int x,y;
		if ( RungArray[ScanRung].Used )
		{
			for (y=0;y<RUNG_HEIGHT;y++)
			{
				for(x=0;x<RUNG_WIDTH;x++)
				{
					if ( RungArray[ScanRung].Element[x][y].Type!=ELE_FREE )
						SetUsedStateFunctionBlock( RungArray[ScanRung].Element[x][y].Type, RungArray[ScanRung].Element[x][y].VarNum, TRUE );
				}
			}
		}
	}
}
// if return -1: for a function block: no more available...
int GetFreeNumberFunctionBlock( int Type )
{
	int Result = -1;
	int ScanArray = 0;
	char IsFunctionBlock = FALSE;
	switch( Type )
	{
#ifdef OLD_TIMERS_MONOS_SUPPORT
		case ELE_TIMER:
			IsFunctionBlock = TRUE;
			do
			{
				if ( NumTimersUsedInRung[ScanArray]==FALSE )
					Result = ScanArray;
				ScanArray++;
			}
			while( ScanArray<NBR_TIMERS && Result==-1 );
			break;
		case ELE_MONOSTABLE:
			IsFunctionBlock = TRUE;
			do
			{
				if ( NumMonostablesUsedInRung[ScanArray]==FALSE )
					Result = ScanArray;
				ScanArray++;
			}
			while( ScanArray<NBR_MONOSTABLES && Result==-1 );
			break;
#endif
		case ELE_COUNTER:
			IsFunctionBlock = TRUE;
			do
			{
				if ( NumCountersUsedInRung[ScanArray]==FALSE )
					Result = ScanArray;
				ScanArray++;
			}
			while( ScanArray<NBR_COUNTERS && Result==-1 );
			break;
		case ELE_TIMER_IEC:
			IsFunctionBlock = TRUE;
			do
			{
				if ( NumNewTimersUsedInRung[ScanArray]==FALSE )
					Result = ScanArray;
				ScanArray++;
			}
			while( ScanArray<NBR_TIMERS_IEC && Result==-1 );
			break;
	}
	return (IsFunctionBlock)?Result:0;
}

void InitBufferRungEdited( StrRung * pRung )
{
	int x,y;
	for (y=0;y<RUNG_HEIGHT;y++)
	{
		for(x=0;x<RUNG_WIDTH;x++)
		{
			pRung->Element[x][y].Type = ELE_FREE;
			pRung->Element[x][y].ConnectedWithTop = 0;
			pRung->Element[x][y].VarType = 0;
			pRung->Element[x][y].VarNum = 0;
			pRung->Element[x][y].IndexedVarType = -1;
			pRung->Element[x][y].IndexedVarNum = 0;
			pRung->Element[x][y].DynamicState = 0;
		}
	}
	pRung->Used = TRUE;
	pRung->PrevRung = -1;
	pRung->NextRung = -1;
}

int GetNbrRungsDefined( void )
{
	int NbrRungsUsed = 0;
	int ScanRung;
	for( ScanRung=0; ScanRung<NBR_RUNGS; ScanRung++ )
	{
		if (RungArray[ScanRung].Used)
			NbrRungsUsed++;
	}
	return NbrRungsUsed;
}

int FindFreeRung()
{
	int NumFree = -1;
	int ScanRung = 0;
	do
	{
		if (!RungArray[ScanRung].Used)
			NumFree = ScanRung;
		ScanRung++;
	}
	while( (NumFree==-1)&&(ScanRung<NBR_RUNGS) );
	return NumFree;
}

void AddRung()
{
	InitBufferRungEdited( &EditDatas.Rung );
	EditDatas.DoBeforeFinalCopy = MODE_ADD;
	EditDatas.NumRung = FindFreeRung();
	/* we have found a free rung ? */
	if (EditDatas.NumRung>=0)
	{
		EditDatas.ModeEdit = TRUE;
		EditDatas.ElementUnderEdit = NULL;
		autorize_prevnext_buttons(FALSE);
		clear_label_comment();
		CopyCurrentArithmExpr();
		CopyUsedFonctionBlockBeforeEdit( );
	}
}

void InsertRung()
{
	/* not really different than adding a rung... */
	AddRung();
	EditDatas.DoBeforeFinalCopy = MODE_INSERT;
}

void ModifyCurrentRung()
{
	CopyRungToRung(&RungArray[InfosGene->CurrentRung],&EditDatas.Rung);
	EditDatas.DoBeforeFinalCopy = MODE_MODIFY;
	EditDatas.NumRung = InfosGene->CurrentRung;
	EditDatas.ModeEdit = TRUE;
	EditDatas.ElementUnderEdit = NULL;
	autorize_prevnext_buttons(FALSE);
	CopyCurrentArithmExpr();
	CopyUsedFonctionBlockBeforeEdit( );
}

void DeleteCurrentRung()
{
	int NextDeleted;
	int PrevDeleted;
	int NewCurrent;
	int OldCurrent;
	int x,y;
	/* do not destroy the last one ! */
	if ( !((InfosGene->CurrentRung==InfosGene->FirstRung) && (InfosGene->CurrentRung==InfosGene->LastRung)) )
	{
		PrevDeleted = RungArray[InfosGene->CurrentRung].PrevRung;
		NextDeleted = RungArray[InfosGene->CurrentRung].NextRung;
		NewCurrent = NextDeleted;
		/* deleting first rung ?*/
		if (InfosGene->CurrentRung==InfosGene->FirstRung)
		{
			InfosGene->FirstRung = NextDeleted;
		}
		else
		{
			RungArray[PrevDeleted].NextRung = NextDeleted;
		}
		/* deleting last rung ? */
		if (InfosGene->CurrentRung==InfosGene->LastRung)
		{
			InfosGene->LastRung = PrevDeleted;
			NewCurrent = InfosGene->LastRung;
		}
		else
		{
			RungArray[NextDeleted].PrevRung = PrevDeleted;
		}
		/* the rung is now free ! */
		OldCurrent = InfosGene->CurrentRung;
		RungArray[InfosGene->CurrentRung].Used = FALSE;
		InfosGene->CurrentRung = NewCurrent;
//		DrawRungs();
RedrawSignalDrawingArea( );
		UpdateVScrollBar();
		refresh_label_comment( );
		/* save infos for the current section */
		SectionArray[ InfosGene->CurrentSection ].FirstRung = InfosGene->FirstRung;
		SectionArray[ InfosGene->CurrentSection ].LastRung = InfosGene->LastRung;
		/* kill the expressions used on the rung deleted */
		for (y=0;y<RUNG_HEIGHT;y++)
		{
			for(x=0;x<RUNG_WIDTH;x++)
			{
				if ( (RungArray[OldCurrent].Element[x][y].Type == ELE_COMPAR)
				|| (RungArray[OldCurrent].Element[x][y].Type == ELE_OUTPUT_OPERATE) )
				{
					rtapi_strxcpy(ArithmExpr[ RungArray[OldCurrent].Element[x][y].VarNum ].Expr,"");
				}
			}
		}
	}
}

void CancelRungEdited()
{
	EditDatas.ModeEdit = FALSE;
	EditDatas.ElementUnderEdit = NULL;
	EditDatas.NumElementSelectedInToolBar = -1;
	LoadElementProperties(NULL);
//	DrawRungs();
RedrawSignalDrawingArea( );
	refresh_label_comment( );
	autorize_prevnext_buttons(TRUE);
}

void ApplyRungEdited()
{
	int PrevNew;
	int NextNew;
	save_label_comment_edited();
	CopyRungToRung(&EditDatas.Rung,&RungArray[EditDatas.NumRung]);
	ApplyNewArithmExpr();

	/* if we have added or inserted, we will have to */
	/* modify the links between rungs */
	/* order is important when setting all this, if multi-tasking : */
	/* calc.c is using only .NextRung to updates all the rungs. */
	/* and also global FirstRung & LastRung to start and end */
	switch(EditDatas.DoBeforeFinalCopy)
	{
		/* Add rung edited after current rung */
		case MODE_ADD:
			PrevNew = InfosGene->CurrentRung;
			NextNew = RungArray[InfosGene->CurrentRung].NextRung;
			RungArray[EditDatas.NumRung].PrevRung = PrevNew;
			/* new added is not the last now ? */
			if (InfosGene->CurrentRung!=InfosGene->LastRung)
			{
				RungArray[EditDatas.NumRung].NextRung = NextNew;
				RungArray[NextNew].PrevRung = EditDatas.NumRung;
			}
			RungArray[PrevNew].NextRung = EditDatas.NumRung;
			/* we can now update LastRung */
			if (InfosGene->CurrentRung==InfosGene->LastRung)
			{
				InfosGene->LastRung = EditDatas.NumRung;
			}
			/* Here is the new current ! */
			InfosGene->CurrentRung = EditDatas.NumRung;
			UpdateVScrollBar();
			break;

		/* Insert rung edited before current rung */
		case MODE_INSERT:
			PrevNew = RungArray[InfosGene->CurrentRung].PrevRung;
			NextNew = InfosGene->CurrentRung;
			RungArray[EditDatas.NumRung].NextRung = NextNew;
			/* new added is not the first now ? */
			if (InfosGene->CurrentRung!=InfosGene->FirstRung)
			{
				RungArray[EditDatas.NumRung].PrevRung = PrevNew;
				RungArray[PrevNew].NextRung = EditDatas.NumRung;
			}
			else
			{
				InfosGene->FirstRung = EditDatas.NumRung;
			}
			RungArray[NextNew].PrevRung = EditDatas.NumRung;
			/* Here is the new current ! */
			InfosGene->CurrentRung = EditDatas.NumRung;
			UpdateVScrollBar();
			break;
	}

	/* save infos for the current section */
	SectionArray[ InfosGene->CurrentSection ].FirstRung = InfosGene->FirstRung;
	SectionArray[ InfosGene->CurrentSection ].LastRung = InfosGene->LastRung;

	EditDatas.ModeEdit = FALSE;
	EditDatas.ElementUnderEdit = NULL;
	EditDatas.NumElementSelectedInToolBar = -1;
	LoadElementProperties(NULL);
//	DrawRungs();
RedrawSignalDrawingArea( );
	autorize_prevnext_buttons(TRUE);
	InfosGene->AskConfirmationToQuit = TRUE;
	InfosGene->HasBeenModifiedForExitCode = TRUE;
}



/* When we fall on an "unusable" element of a big one,
we return new posix and posiy of the "alive" block */
void CheckForBlocksOfBigElement(StrRung *pRungToCheck, int * PosiX,int * PosiY )
{
	int ScanX,ScanY;
	/* on an "unusable" ? */
	if (pRungToCheck->Element[*PosiX][*PosiY].Type==ELE_UNUSABLE)
	{
		/* we now will have to check for all the "alive" block of
		bigs elements */
		for (ScanY=0;ScanY<RUNG_HEIGHT;ScanY++)
		{
			for(ScanX=0;ScanX<RUNG_WIDTH;ScanX++)
			{
				int RuleSizeX,RuleSizeY;
				/* Is is an element with a rule ? */
				if ( GetSizesOfAnElement( pRungToCheck->Element[ScanX][ScanY].Type, &RuleSizeX, &RuleSizeY ) )
				{
					/* Have we clicked in it ? */
					if ( (*PosiX>=ScanX-RuleSizeX-1)
						&& (*PosiX<=ScanX)
						&& (*PosiY>=ScanY)
						&& (*PosiY<=ScanY+RuleSizeY-1) )
					{
						/* We've got it ! */
						/* We make as we have clicked on the "alive" block ! */
						*PosiX = ScanX;
						*PosiY = ScanY;
						return;
					}
				}
			}
		}
	}
}

int VerifyConstraintsAndRulesForElement(short int NumEle,int PosiX,int PosiY)
{
	int ItIsOk = TRUE;
	int ItIsAnOutputEle = FALSE;
	int RuleSizeX = 1,RuleSizeY = 1;
	int OldEleType = EditDatas.Rung.Element[PosiX][PosiY].Type;
//printf("Verify rules: typeele=%d x=%d,y=%d\n", NumEle, PosiX, PosiY );

	if ( (NumEle==ELE_OUTPUT) || (NumEle==ELE_OUTPUT_NOT)
			|| (NumEle==ELE_OUTPUT_SET) || (NumEle==ELE_OUTPUT_RESET)
			|| (NumEle==ELE_OUTPUT_JUMP) || (NumEle==ELE_OUTPUT_CALL)
			|| (NumEle==ELE_OUTPUT_OPERATE) )
		ItIsAnOutputEle = TRUE;
	if ( NumEle==EDIT_COPY && GetIsOutputEleLastColumnSelection( ) )
		ItIsAnOutputEle = TRUE;

	/* verify for outputs if we are under output zone (right column) */
	if ( (PosiX==RUNG_WIDTH-1) && !ItIsAnOutputEle )
	{
		ItIsOk = FALSE;
	}
	/* verify for inputs if we are under input zone (not right column) */
	if ( (PosiX<RUNG_WIDTH-1) && ItIsAnOutputEle )
	{
		ItIsOk = FALSE;
	}

	/* verify if for elements bigger than one block it will fit */
	if ( GetSizesOfAnElement( NumEle, &RuleSizeX, &RuleSizeY ) )
	{
		if ( (PosiX-RuleSizeX+1 < 0) || (PosiY+RuleSizeY-1 >= RUNG_HEIGHT) )
			ItIsOk = FALSE;
	}

	//v0.8.3: now we don't allow a "one" block (or big) element to destroy a big
	if ( RuleSizeX<=1 && RuleSizeY<=1 )
	{
		int SizeElementToDestroyX;
		int SizeElementToDestroyY;
		/* verify if already a big element here ? */
		GetSizesOfAnElement( OldEleType, &SizeElementToDestroyX, &SizeElementToDestroyY );
		if ( SizeElementToDestroyX>1 || SizeElementToDestroyY>1 )
		{
			ItIsOk = FALSE;
		}
		else
		{
			// just verify not on part of a big
			if ( OldEleType==ELE_UNUSABLE )
				ItIsOk = FALSE;
		}
	}
	else
	{
		/* We are a big element, verify all blocks under it are available... */
		int PassX,PassY;
		for (PassX = PosiX - RuleSizeX +1 ; PassX<=PosiX ; PassX++)
		{
			for (PassY = PosiY ; PassY<=PosiY + RuleSizeY -1 ; PassY++)
			{
				int PassTypeEle = EditDatas.Rung.Element[PassX][PassY].Type; 
//v0.8.3				if (EditDatas.Rung.Element[PassX][PassY].Type==ELE_UNUSABLE)
				if ( ! (PassTypeEle==ELE_FREE || PassTypeEle==ELE_CONNECTION) )
					ItIsOk = FALSE;
			}
		}
	}
	return ItIsOk;
}

/* used for destroying or adding a big element */
/* when destroying : filling all the blocks with ELE_FREE */
/* when adding : filling all the blocks with ELE_UNUSABLE */
/* the block "alive" is written elsewhere */
void CleanForBigElement(short int NumEle,int PosiX,int PosiY,short int FillWithThis)
{
	int PassX,PassY;
	int RuleSizeX,RuleSizeY;
	if ( GetSizesOfAnElement( NumEle, &RuleSizeX, &RuleSizeY ) )
	{
		for (PassX = PosiX - RuleSizeX +1 ; PassX<=PosiX ; PassX++)
		{
			for (PassY = PosiY ; PassY<=PosiY + RuleSizeY -1 ; PassY++)
			{
				CheckForFreeingArithmExpr(PassX,PassY);

				SetUsedStateFunctionBlock( EditDatas.Rung.Element[PassX][PassY].Type, EditDatas.Rung.Element[PassX][PassY].VarNum, FALSE );

				EditDatas.Rung.Element[PassX][PassY].Type = FillWithThis;
				EditDatas.Rung.Element[PassX][PassY].ConnectedWithTop = 0;
				EditDatas.Rung.Element[PassX][PassY].DynamicOutput = 0;
			}
		}
	}
}

void MovePosiForBigElementToAliveBlock(short int NumEle,int * PosiX)
{
	int RuleSizeX,RuleSizeY;
	if ( GetSizesOfAnElement( NumEle, &RuleSizeX, &RuleSizeY ) )
	{
		*PosiX = *PosiX + RuleSizeX - 1;
		if ( *PosiX>=RUNG_WIDTH )
		{
			*PosiX = RUNG_WIDTH-1;
		}
	}
}

/* return TRUE if a rule is defined */
char GetSizesOfAnElement(short int NumTypeEle,int * pSizeX, int * pSizeY)
{
	char cRuleDefined = FALSE;
	// size for the current part selected ?
	if ( NumTypeEle==EDIT_COPY )
	{
		GetSizesOfTheSelectionToCopy( pSizeX, pSizeY );
		cRuleDefined = TRUE;
//printf( "Size for selection = %d,%d\n", *pSizeX, *pSizeY );
	}
	else
	{
		int RulePass;
		// default size of an element	
		*pSizeX = 1;
		*pSizeY = 1;
		RulePass = 0;
		do
		{
			if (RulesForSpecialElements[RulePass][TYPEELERULE] == NumTypeEle )
			{
				*pSizeX = RulesForSpecialElements[RulePass][XSIZEELERULE];
				*pSizeY = RulesForSpecialElements[RulePass][YSIZEELERULE];
				cRuleDefined = TRUE;
			}
			RulePass++;
		}
		while( RulesForSpecialElements[RulePass][0]!=-1 && *pSizeX==1 && *pSizeY==1 );
		if ( NumTypeEle==ELE_FREE || NumTypeEle==ELE_CONNECTION || NumTypeEle==ELE_UNUSABLE )
		{
			*pSizeX = 0;
			*pSizeY = 0;
		}
//printf( "GetSizesOfAnElement:%d = %d/%d\n",NumTypeEle, *pSizeX, *pSizeY );
		if ( NumTypeEle>=EDIT_CNX_WITH_TOP )
		{
			printf(_("!!!Abnormal current type=%d in rung...(file %s,line %d)\n"), NumTypeEle, __FILE__, __LINE__ );
			*pSizeX = 0;
			*pSizeY = 0;
		}
	}
	return cRuleDefined;
}

/* return TRUE if contact or coil element */
int IsASimpleElement(int Element)
{
	int Res = FALSE;
	if ( (Element==ELE_INPUT) || (Element==ELE_INPUT_NOT)
		|| (Element==ELE_RISING_INPUT) || (Element==ELE_FALLING_INPUT)
		|| (Element==ELE_OUTPUT) || (Element==ELE_OUTPUT_NOT)
		|| (Element==ELE_OUTPUT_SET) || (Element==ELE_OUTPUT_RESET)
		|| (Element==ELE_OUTPUT_JUMP) || (Element==ELE_OUTPUT_CALL) )
		Res = TRUE;
	return Res;
}

// this sets the variable based on the type of edit-element (NumElement) being used
int SetDefaultVariableForElement(int NumElement)
{
	switch( NumElement )
	{
			case ELE_INPUT:
			case ELE_INPUT_NOT:
			case ELE_RISING_INPUT:
			case ELE_FALLING_INPUT:
				return DEFAULT_VAR_FOR_CONTACT; //variables %I
				break;
			case ELE_OUTPUT:
			case ELE_OUTPUT_NOT:
			case ELE_OUTPUT_SET:
			case ELE_OUTPUT_RESET:
				return DEFAULT_VAR_FOR_COIL; //variables %Q
				break;
	}
	return -1;
}

void VerticalCleanupWhenErasingElement( int PosiX, int PosiY )
{
	char CleanUp = TRUE;
	// do not delete if there is a connexion at x & y+1
	if ( PosiY<RUNG_HEIGHT-1 )
	{
		if ( EditDatas.Rung.Element[PosiX][PosiY+1].ConnectedWithTop )
			CleanUp = FALSE;
	}
	// do not delete if there is an element at x & y
	// (useful when trying to erase second right vertical)
	if ( PosiX<RUNG_WIDTH-1 )
	{
		if ( EditDatas.Rung.Element[PosiX][PosiY].Type!=ELE_FREE )
			CleanUp = FALSE;
	}
	// do not delete if there is an element at x-1 & y
	if ( PosiX>0 )
	{
		if ( EditDatas.Rung.Element[PosiX-1][PosiY].Type!=ELE_FREE )
			CleanUp = FALSE;
	}
	if ( CleanUp )
		EditDatas.Rung.Element[ PosiX ][ PosiY ].ConnectedWithTop = 0;
}

char ConvertDoublesToRungCoor( double coorx, double coory, int * pRungX, int * pRungY )
{
	char cOk = FALSE;
	if ( coory>=InfosGene->HeaderLabelCommentHeight )
	{
		/* correspond to which block ? */
		*pRungX = coorx/InfosGene->BlockWidth;
		*pRungY = (coory-InfosGene->HeaderLabelCommentHeight)/InfosGene->BlockHeight;
		if ( (*pRungX<RUNG_WIDTH) && (*pRungY<RUNG_HEIGHT) )
		{
			cOk = TRUE;
		}
	}
	return cOk;
}

// return TRUE if ok (noting to do, or things to do for complex ones ok :
// if function block, getting a free number one ; if expr, allocating a string)
char PrepBeforeSettingTypeEleForComplexBlocsAndExpr( int NumTypeEle, int PosiX, int PosiY )
{
	char ResultOk = TRUE;
	// function block element, set directly a free number one...
	if ( !(IsASimpleElement(NumTypeEle) ) )
	{
		int BlockNumber = GetFreeNumberFunctionBlock( NumTypeEle );
		if ( BlockNumber==-1 )
		{
			ShowMessageBoxError( _("No more free function block of this type available...") );
			ResultOk = FALSE;
		}
		else
		{
//printf( "free block nbr=%d\n", BlockNumber );
			EditDatas.Rung.Element[PosiX][PosiY].VarNum = BlockNumber;
			SetUsedStateFunctionBlock( NumTypeEle, EditDatas.Rung.Element[PosiX][PosiY].VarNum, TRUE );
		}
	}
	if ( !CheckForAllocatingArithmExpr( NumTypeEle, PosiX, PosiY ) )
	{
		ShowMessageBoxError( _("No more free arithmetic expression for this type available...") );
		ResultOk = FALSE;
	}
	return ResultOk;
}

/* click with the mouse in x and y pixels of the rung */
void EditElementInRung(double x,double y)
{
	int RungX,RungY;
	short int NumElement;
	if ( ConvertDoublesToRungCoor( x, y, &RungX, &RungY )
		&& (EditDatas.NumElementSelectedInToolBar!=-1) )
	{
		/* check for "unusable" blocks */
		if (EditDatas.NumElementSelectedInToolBar==EDIT_POINTER || EditDatas.NumElementSelectedInToolBar==EDIT_ERASER )
			CheckForBlocksOfBigElement( &EditDatas.Rung, &RungX,&RungY );
		if (EditDatas.NumElementSelectedInToolBar!=EDIT_CNX_WITH_TOP
				&& EditDatas.NumElementSelectedInToolBar!=EDIT_POINTER
				&& EditDatas.NumElementSelectedInToolBar!=EDIT_LONG_CONNECTION )
		{
			NumElement = EditDatas.NumElementSelectedInToolBar;
			/* For big element, click insert top-left of the element */
			if ( NumElement<EDIT_CNX_WITH_TOP )
				MovePosiForBigElementToAliveBlock(NumElement,&RungX);
			/* already the same element ? => if yes kill it! */
			if (NumElement==EditDatas.Rung.Element[RungX][RungY].Type)
			{
				NumElement = EDIT_ERASER;
			}
			/* apply the new element */
			if (NumElement==EDIT_ERASER)
			{
//printf("erasing...\n");
				if ( EditDatas.Rung.Element[RungX][RungY].Type!=ELE_FREE )
				{
					/* the blocks other than the "alive" are now free... */
					CleanForBigElement(EditDatas.Rung.Element[RungX][RungY].Type,RungX,RungY,ELE_FREE);
					EditDatas.Rung.Element[RungX][RungY].DynamicOutput = 0;
					EditDatas.Rung.Element[RungX][RungY].Type = ELE_FREE;
					// delete verticals connexions associated to that element if seems no more necessary !
					VerticalCleanupWhenErasingElement( RungX, RungY );
					VerticalCleanupWhenErasingElement( RungX+1, RungY );
				}
				else
				{
					// if free, perhaps user just wants to delete a vertical cnx...
					EditDatas.Rung.Element[RungX][RungY].ConnectedWithTop = 0;
				}
			}
			else
			{
				
				if (VerifyConstraintsAndRulesForElement(NumElement,RungX,RungY))
				{
					/* the blocks other than the "alive" are now free... */
//v0.8.3: no more necessary, as we do not more kill big		CleanForBigElement(EditDatas.Rung.Element[RungX][RungY].Type,RungX,RungY,ELE_FREE);
					//v0.8.9, moved prep() here to not do anything if type no more available!
					// if function block, getting a free one, if expr, allocating a string...
					// failed if no more function block available or arthmetic expressions...
					if ( PrepBeforeSettingTypeEleForComplexBlocsAndExpr( NumElement, RungX, RungY ) )
					{
						
						/* for big element with only one block "alive" we must mark the others */
						/* blocks as unusable */
						CleanForBigElement(NumElement,RungX,RungY,ELE_UNUSABLE);
						// if not a simple element (contact/coil) replaced by a simple element, clean VarNum
						//v0.8.9if ( !(IsASimpleElement(NumElement) && IsASimpleElement(EditDatas.Rung.Element[RungX][RungY].Type)) )
						//v0.8.9{
						//v0.8.9	EditDatas.Rung.Element[RungX][RungY].VarNum = 0;
						//v0.8.9}
						// if the area was free before then set the default variable based on type of element edited
						if (EditDatas.Rung.Element[RungX][RungY].Type == ELE_FREE)
						{
							int DefaultVarForIt = SetDefaultVariableForElement(NumElement);
							if (DefaultVarForIt!=-1)
							{
#ifdef IDVAR_IS_TYPE_AND_OFFSET
								EditDatas.Rung.Element[RungX][RungY].VarType = DefaultVarForIt;
#else
								EditDatas.Rung.Element[RungX][RungY].VarType = VAR_DEFAULT_TYPE;
								EditDatas.Rung.Element[RungX][RungY].VarNum = DefaultVarForIt;
#endif
							}
						}
						// set type to the "alive" block
						EditDatas.Rung.Element[RungX][RungY].Type = NumElement;
						
					}
				}//if (VerifyConstraintsAndRulesForElement(
				
			}
		}
		if (EditDatas.NumElementSelectedInToolBar==EDIT_CNX_WITH_TOP)
		{
			if (EditDatas.Rung.Element[RungX][RungY].ConnectedWithTop)
			{
				EditDatas.Rung.Element[RungX][RungY].ConnectedWithTop = 0;
			}
			else
			{
				if (RungY>0)
					EditDatas.Rung.Element[RungX][RungY].ConnectedWithTop = 1;
			}
		}
		if (EditDatas.NumElementSelectedInToolBar==EDIT_LONG_CONNECTION)
		{
			int ScanX = RungX;
			while(ScanX<RUNG_WIDTH-1 && EditDatas.Rung.Element[ScanX][RungY].Type==ELE_FREE)
			{
				EditDatas.Rung.Element[ScanX++][RungY].Type = ELE_CONNECTION;
			}
		}
//printf("current type = %d\n",EditDatas.Rung.Element[RungX][RungY].Type);
		LoadElementProperties(&EditDatas.Rung.Element[RungX][RungY]);
		EditDatas.ElementUnderEdit = &EditDatas.Rung.Element[RungX][RungY];
		// infos used to display the "selected element" box
		EditDatas.CurrentElementPosiX = RungX;
		EditDatas.CurrentElementPosiY = RungY;
		GetSizesOfAnElement( EditDatas.Rung.Element[RungX][RungY].Type, &EditDatas.CurrentElementSizeX, &EditDatas.CurrentElementSizeY);
	}
}

void MouseMotionOnRung(double x,double y)
{
	if ( EditDatas.NumElementSelectedInToolBar<EDIT_CNX_WITH_TOP || EditDatas.NumElementSelectedInToolBar==EDIT_COPY )
	{
		int RungX,RungY;
		if ( ConvertDoublesToRungCoor( x, y, &RungX, &RungY ) )
		{
//printf( "Motion Position %d,%d for ghost\n", RungX, RungY );
			MovePosiForBigElementToAliveBlock( EditDatas.NumElementSelectedInToolBar, &RungX );
//printf( "Position to alive one %d,%d for ghost\n", RungX, RungY );
			if (VerifyConstraintsAndRulesForElement( EditDatas.NumElementSelectedInToolBar, RungX, RungY) )
			{
				EditDatas.GhostZonePosiX = RungX-EditDatas.GhostZoneSizeX+1;
				EditDatas.GhostZonePosiY = RungY;
//printf( "Ok rules Position %d,%d for ghost\n", EditDatas.GhostZonePosiX, EditDatas.GhostZonePosiY );
				return;
			}
		}
	}
	EditDatas.GhostZonePosiX = -1;
	EditDatas.GhostZonePosiY = -1;
}

/* click with the mouse in x and y pixels of the section display */
void EditElementInThePage(double x,double y)
{
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
	{
		if ( ( y >= InfosGene->OffsetCurrentRungDisplayed ) && ( y < InfosGene->OffsetCurrentRungDisplayed+TOTAL_PX_RUNG_HEIGHT ) )
		{
			if ( EditDatas.NumElementSelectedInToolBar==EDIT_SELECTION )
				StartOrMotionPartSelection( x, y - InfosGene->OffsetCurrentRungDisplayed, TRUE/*StartToClick*/ );
			else if ( EditDatas.NumElementSelectedInToolBar==EDIT_COPY )
				CopyNowPartSelected( x, y - InfosGene->OffsetCurrentRungDisplayed );
			else
				EditElementInRung( x, y - InfosGene->OffsetCurrentRungDisplayed );
		}
		else
		{
			ShowMessageBoxError( _("You clicked outside of the current rung actually selected...") );
		}
	}
#ifdef SEQUENTIAL_SUPPORT
	if ( iCurrentLanguage==SECTION_IN_SEQUENTIAL )
		EditElementInSeqPage( x+InfosGene->HScrollValue, y+InfosGene->VScrollValue );
#endif
}

/* move of the mouse in x and y pixels in the section display */
void MouseMotionOnThePage( double x, double y )
{
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
	{
		if ( ( y >= InfosGene->OffsetCurrentRungDisplayed ) && ( y < InfosGene->OffsetCurrentRungDisplayed+TOTAL_PX_RUNG_HEIGHT ) )
		{
			if ( EditDatas.NumElementSelectedInToolBar==EDIT_SELECTION )
				StartOrMotionPartSelection( x, y - InfosGene->OffsetCurrentRungDisplayed, FALSE/*StartToClick*/ );
			else
				MouseMotionOnRung( x, y - InfosGene->OffsetCurrentRungDisplayed );
		}
		else
		{
			if ( EditDatas.NumElementSelectedInToolBar!=EDIT_SELECTION )
			{
				EditDatas.GhostZonePosiX = -1;
				EditDatas.GhostZonePosiY = -1;
			}
		}
	}
}
void EditButtonReleaseEventOnThePage( void )
{
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
	{
		if ( EditDatas.NumElementSelectedInToolBar==EDIT_SELECTION )
			EndPartSelection( );
	}
}

char * GetLadderElePropertiesForStatusBar(double x,double y)
{
	if ( ( y >= InfosGene->OffsetCurrentRungDisplayed ) && ( y < InfosGene->OffsetCurrentRungDisplayed+TOTAL_PX_RUNG_HEIGHT ) )
	{
		int RungX,RungY;
		if ( ConvertDoublesToRungCoor( x, y - InfosGene->OffsetCurrentRungDisplayed, &RungX, &RungY ) )
		{
			StrElement * Element;
			CheckForBlocksOfBigElement( &RungArray[InfosGene->CurrentRung], &RungX,&RungY );
			Element = &RungArray[InfosGene->CurrentRung].Element[RungX][RungY];
			return GetElementPropertiesForStatusBar( Element );
		}
	}
	return "";
}

