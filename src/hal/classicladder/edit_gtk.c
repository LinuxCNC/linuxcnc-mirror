/* Classic Ladder Project */
/* Copyright (C) 2001-2010 Marc Le Douarain */
/* http://membres.lycos.fr/mavati/classicladder/ */
/* http://www.sourceforge.net/projects/classicladder */
/* May 2001 */
/* ------------------------------------- */
/* Editor - GTK interface part (Toolbar) */
/* ------------------------------------- */
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
#include <gtk/gtk.h>
#include <libintl.h> // i18n
#include <locale.h> // i18n

#include "classicladder.h"
#include "global.h"
#include "drawing.h"
#include "edit.h"
#include "classicladder_gtk.h"
#include "edit_gtk.h"
#include "editproperties_gtk.h"
#include "manager_gtk.h"
#include "edit_copy.h"
#include "menu_and_toolbar_gtk.h"

static GtkWidget *EditorButtonOk,*EditorButtonCancel;
static GtkWidget *EditorButtonAdd,*EditorButtonIns,*EditorButtonDel;
static GtkWidget *EditorButtonModify;

#define NBR_ELE_IN_TOOLBAR 50
#define NBR_ELE_TOOLBAR_Y_MAX 12 // used for each GtkTable
#define NBR_ELE_TOOLBAR_X_MAX 4
typedef struct StrToolbarDatas
{
	GtkWidget * ToolbarTable;
	GtkWidget * ToolbarBtnRadio[ NBR_ELE_TOOLBAR_Y_MAX ][ NBR_ELE_TOOLBAR_X_MAX ];
	GtkWidget * ToolbarImage[ NBR_ELE_TOOLBAR_Y_MAX ][ NBR_ELE_TOOLBAR_X_MAX ];
#if GTK_MAJOR_VERSION<3
	GdkPixmap * ToolbarPixmap[ NBR_ELE_TOOLBAR_Y_MAX ][ NBR_ELE_TOOLBAR_X_MAX ];
#else
	cairo_surface_t * ToolBarSurface[ NBR_ELE_TOOLBAR_Y_MAX ][ NBR_ELE_TOOLBAR_X_MAX ];
#endif
//	short int PtrOnToolBarElementsList[ ][ NBR_ELE_TOOLBAR_X_MAX ]; //hard....
	short int (*PtrOnToolBarElementsList)[ NBR_ELE_TOOLBAR_X_MAX ];
}StrToolbarDatas;

#define NUM_TOOLBAR_FOR_LADDER 0
#ifdef SEQUENTIAL_SUPPORT
#define NUM_TOOLBAR_FOR_SEQ 1
#define NBR_TOOLBARS 2
#else
#define NBR_TOOLBARS 1
#endif
static StrToolbarDatas ToolbarDatas[ NBR_TOOLBARS ];
//ForGTK3, deprecated...GtkTooltips * TheTooltips;

#define PIXELS_SIZE_IN_TOOLBAR 32

static short int ToolBarElementsLadder[ ][NBR_ELE_TOOLBAR_X_MAX] =
            { {EDIT_POINTER , EDIT_ERASER, EDIT_SELECTION , EDIT_COPY} ,
              {ELE_INPUT , ELE_INPUT_NOT , ELE_RISING_INPUT , ELE_FALLING_INPUT} ,
              {ELE_CONNECTION , EDIT_CNX_WITH_TOP, EDIT_LONG_CONNECTION , 0} ,
              {ELE_TIMER_IEC , ELE_COUNTER , ELE_COMPAR , 0} ,
#ifdef OLD_TIMERS_MONOS_SUPPORT
              {ELE_TIMER , ELE_MONOSTABLE , 0 , 0} ,
#endif
              {ELE_OUTPUT , ELE_OUTPUT_NOT, ELE_OUTPUT_SET , ELE_OUTPUT_RESET} ,
              {ELE_OUTPUT_JUMP, ELE_OUTPUT_CALL , ELE_OUTPUT_OPERATE , 0} ,
              {-1,-1}/*end*/ };
char * ToolBarToolTipsTextLadder[ ][NBR_ELE_TOOLBAR_X_MAX] =
            { { N_("Current Object\nSelector"), N_("Eraser"), N_("Select a rung part (drag and release)"), N_("Copy rung part selected") },
              { N_("N.O. Input"), N_("N.C. Input"), N_("Rising Edge\n Input"), N_("Falling Edge\n Input") },
              { N_("Horizontal\nConnection"), N_("Vertical\nConnection"), N_("Long Horizontal\nConnection"), NULL },
              { N_("Timer IEC Block"), N_("Counter Block"), N_("Variable\nComparison"), NULL },
#ifdef OLD_TIMERS_MONOS_SUPPORT
              { N_("Old Timer Block"), N_("Old Monostable Block"), NULL, NULL },
#endif
              { N_("N.O. Output"), N_("N.C. Output"), N_("Set Output"), N_("Reset Output") },
              { N_("Jump Coil"), N_("Call Coil"), N_("Variable\nAssignment"), NULL },
              { NULL, NULL, NULL, NULL } };


#ifdef SEQUENTIAL_SUPPORT
#include "drawing_sequential.h"
#include "edit_sequential.h"
static short int ToolBarElementsSequential[ ][NBR_ELE_TOOLBAR_X_MAX] =
            { {EDIT_POINTER , EDIT_ERASER , 0 , 0} ,
              {ELE_SEQ_STEP , EDIT_SEQ_INIT_STEP , 0 , 0} ,
              {ELE_SEQ_TRANSITION , EDIT_SEQ_STEP_AND_TRANS , 0 , 0} ,
              {EDIT_SEQ_START_MANY_TRANS , EDIT_SEQ_END_MANY_TRANS , 0 , 0} ,
              {EDIT_SEQ_START_MANY_STEPS , EDIT_SEQ_END_MANY_STEPS , 0 , 0} ,
              {EDIT_SEQ_LINK , 0 , 0 , 0} ,
              {ELE_SEQ_COMMENT , 0 , 0 , 0} ,
              {-1,-1}/*end*/ };
char * ToolBarToolTipsTextSequential[ ][NBR_ELE_TOOLBAR_X_MAX] =
            { { N_("Current Object\nSelector"), N_("Eraser"), NULL, NULL },
              { N_("Step"), NULL, NULL, NULL },
              { N_("Transition"), NULL, NULL, NULL },
              { NULL, NULL, NULL, NULL },
              { NULL, NULL, NULL, NULL },
              { N_("Link"), NULL, NULL, NULL },
              { N_("Comment"), NULL, NULL, NULL },
              { NULL, NULL, NULL, NULL } };
#endif

GtkWidget *EditWindow;
static char FirstOpenToSetPosition = FALSE;


void ButtonsForStart()
{
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	gtk_widget_hide (EditorButtonAdd);
	gtk_widget_hide (EditorButtonIns);
	gtk_widget_hide (EditorButtonDel);
	gtk_widget_hide (EditorButtonModify);
	gtk_widget_show (EditorButtonOk);
	gtk_widget_show (EditorButtonCancel);
	ShowPropertiesWindow( TRUE );
	// select directly the pointer in toolbar per default...
	EditDatas.NumElementSelectedInToolBar = EDIT_POINTER;
	// ...in rung toolbar
	SelectAnElementInToolBar( NUM_TOOLBAR_FOR_LADDER, EDIT_POINTER );
	gtk_widget_set_sensitive( ToolbarDatas[ NUM_TOOLBAR_FOR_LADDER ].ToolbarTable, TRUE );
#ifdef SEQUENTIAL_SUPPORT
	// ...in sequential toolbar
	SelectAnElementInToolBar( NUM_TOOLBAR_FOR_SEQ, EDIT_POINTER );
	gtk_widget_set_sensitive( ToolbarDatas[ NUM_TOOLBAR_FOR_SEQ ].ToolbarTable, TRUE );
#endif
	MessageInStatusBar( iCurrentLanguage==SECTION_IN_LADDER?_("Current rung in edit mode..."):_("Edit mode...") );
	ManagerEnableActionsSectionsList( FALSE );
}
void ButtonsForEnd( char ForRung )
{
	if ( ForRung )
	{
		gtk_widget_show (EditorButtonAdd);
		gtk_widget_show (EditorButtonIns);
		gtk_widget_show (EditorButtonDel);
		gtk_widget_set_sensitive( ToolbarDatas[ NUM_TOOLBAR_FOR_LADDER ].ToolbarTable, FALSE );
	}
	else
	{
		gtk_widget_hide (EditorButtonAdd);
		gtk_widget_hide (EditorButtonIns);
		gtk_widget_hide (EditorButtonDel);
#ifdef SEQUENTIAL_SUPPORT
		gtk_widget_set_sensitive( ToolbarDatas[ NUM_TOOLBAR_FOR_SEQ ].ToolbarTable, FALSE );
#endif
	}
	gtk_widget_show (EditorButtonModify);
	gtk_widget_hide (EditorButtonOk);
	gtk_widget_hide (EditorButtonCancel);
	ShowPropertiesWindow( FALSE );
	MessageInStatusBar( "" );
	ManagerEnableActionsSectionsList( TRUE );
}

void EditorButtonsAccordingSectionType( )
{
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	// if under edit, cancel current operation
	if ( EditDatas.ModeEdit )
		ButtonCancelCurrentRung( );
	if ( iCurrentLanguage==SECTION_IN_SEQUENTIAL )
	{
		gtk_widget_hide( ToolbarDatas[ NUM_TOOLBAR_FOR_LADDER ].ToolbarTable );
#ifdef SEQUENTIAL_SUPPORT
		gtk_widget_show( ToolbarDatas[ NUM_TOOLBAR_FOR_SEQ ].ToolbarTable );
#endif
	}
    else
	{
#ifdef SEQUENTIAL_SUPPORT
		gtk_widget_hide( ToolbarDatas[ NUM_TOOLBAR_FOR_SEQ ].ToolbarTable );
#endif
		gtk_widget_show( ToolbarDatas[ NUM_TOOLBAR_FOR_LADDER ].ToolbarTable );
	}
	ButtonsForEnd( iCurrentLanguage==SECTION_IN_LADDER );
	MessageInStatusBar( "" );
}

void ButtonAddRung()
{
	AddRung();
	ButtonsForStart();
}
void ButtonInsertRung()
{
	InsertRung();
	ButtonsForStart();
}
void ButtonDeleteCurrentRung()
{
	ShowConfirmationBox(_("Delete"),_("Do you really want to delete the current rung ?"),DeleteCurrentRung);
}
void ButtonModifyCurrentRung()
{
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
	{
		ModifyCurrentRung();
		ButtonsForStart();
	}
#ifdef SEQUENTIAL_SUPPORT
	if ( iCurrentLanguage==SECTION_IN_SEQUENTIAL )
	{
		ModifyCurrentSeqPage();
		ButtonsForStart();
	}
#endif
}
void ButtonOkCurrentRung()
{
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
		ApplyRungEdited();
#ifdef SEQUENTIAL_SUPPORT
	if ( iCurrentLanguage==SECTION_IN_SEQUENTIAL )
		ApplySeqPageEdited();
#endif
	ButtonsForEnd( iCurrentLanguage==SECTION_IN_LADDER );
}
void ButtonCancelCurrentRung()
{
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
		CancelRungEdited();
#ifdef SEQUENTIAL_SUPPORT
	if ( iCurrentLanguage==SECTION_IN_SEQUENTIAL )
		CancelSeqPageEdited();
#endif
	ButtonsForEnd( iCurrentLanguage==SECTION_IN_LADDER );
}

gint EditorWindowDeleteEvent( GtkWidget * widget, GdkEvent * event, gpointer data )
{
	(void)widget;
	(void)event;
	(void)data;
	gtk_widget_hide( EditWindow );
	SetToggleMenuForEditorWindow( FALSE/*OpenedWin*/ );
	// we do not want that the window be destroyed.
	return TRUE;
}

void OpenEditWindow( GtkAction * ActionOpen, gboolean OpenIt )
{
	if ( ActionOpen!=NULL )
		OpenIt = gtk_toggle_action_get_active( GTK_TOGGLE_ACTION(ActionOpen) );
	if ( OpenIt )
	{
		gtk_widget_show (EditWindow);
		gtk_window_present( GTK_WINDOW(EditWindow) );
		if ( !FirstOpenToSetPosition )
		{
			// try to move at a nice place the window (to left or right of the main window)
			int mainx,mainy,mainw,mainh;
			int winw,winh;
			int winx=0,winy=0;
			gtk_window_get_size( GTK_WINDOW(MainSectionWindow), &mainw, &mainh );
			gtk_window_get_position( GTK_WINDOW(MainSectionWindow), &mainx, &mainy );
			gtk_window_get_size( GTK_WINDOW(EditWindow), &winw, &winh );
			// place on the left ?
			if ( mainx>winw+10 )
			{
				winx = mainx-winw-10;
				winy = mainy;
			}
			else
			{
				// move the window on the right
				winx = mainx+mainw+10;
				winy = mainy;
			}
			printf("to move edit window: x%d,y%d,w%d,h%d => x%d,y%d,w%d,h%d\n",mainx,mainy,mainw,mainh,winx,winy,winw,winh);
			gtk_window_move( GTK_WINDOW(EditWindow), winx,winh );
			FirstOpenToSetPosition = TRUE;
        }
	}
	else
	{
		gtk_widget_hide( EditWindow );
	}
}

char ConvertNumElementInToolbarPosisXY( int NumToolbar, int NumElementWanted, int *FoundX, int *FoundY )
{
	StrToolbarDatas *pToolbarDatas = &ToolbarDatas[ NumToolbar ];
	int ScanX = 0;
	int ScanY = 0;
	char Found = FALSE;
	char End = FALSE;
	do
	{
		do
		{
			int EleValue = pToolbarDatas->PtrOnToolBarElementsList[ ScanY ][ ScanX ];
			if ( EleValue==NumElementWanted )
				Found = TRUE;
			else if ( EleValue==-1 )
				End = TRUE;
			else
				ScanX++;
			if ( ScanX>NBR_ELE_TOOLBAR_X_MAX )
			{
				ScanX = 0;
				ScanY++;
			}
		}
		while( ScanX<NBR_ELE_TOOLBAR_X_MAX && !Found && !End );
	}
	while( !Found && !End );
	if( Found )
	{
		*FoundX = ScanX;
		*FoundY = ScanY;
	}
	else
	{
		printf("ERROR ELEMENT NOT FOUND IN ARRAY FOR THIS ELEMENT !!!!!\n");
	}
	return Found;
}

void SelectAnElementInToolBar( int iNumToolbar, int iNumElementSelectedWithPopup )
{
	int SearchEleX, SearchEleY;
	if ( ConvertNumElementInToolbarPosisXY( iNumToolbar, iNumElementSelectedWithPopup, &SearchEleX, &SearchEleY ) )
		gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON(ToolbarDatas[iNumToolbar].ToolbarBtnRadio[ SearchEleY ][ SearchEleX ]), TRUE );
}

void ButtonToolbarSignal( GtkWidget * widget, gpointer data )
{
	// this callback is called 2 times:
	// firstly for the element unselected (not taken into account here), and secondly for the selected (used!)
	if ( gtk_toggle_button_get_active( GTK_TOGGLE_BUTTON( widget ) ) )
	{
		EditDatas.NumElementSelectedInToolBar = GPOINTER_TO_INT( data );
		EditDatas.GhostZonePosiX = -1;
		EditDatas.GhostZonePosiY = -1;
		if ( EditDatas.NumElementSelectedInToolBar<EDIT_CNX_WITH_TOP || EditDatas.NumElementSelectedInToolBar==EDIT_COPY )
		{
			GetSizesOfAnElement( EditDatas.NumElementSelectedInToolBar, &EditDatas.GhostZoneSizeX, &EditDatas.GhostZoneSizeY );
printf( "Ghost Size %d,%d for type=%d\n", EditDatas.GhostZoneSizeX, EditDatas.GhostZoneSizeY, EditDatas.NumElementSelectedInToolBar );
		}
	}
}

void InitAllForToolbar( void )
{
	int ScanToolbar = 0;
	for ( ScanToolbar=0; ScanToolbar<NBR_TOOLBARS; ScanToolbar++ )
	{
		int ScanX,ScanY;
		StrToolbarDatas *pToolbarDatas = &ToolbarDatas[ ScanToolbar ];
		pToolbarDatas->ToolbarTable = NULL;
		for( ScanY=0; ScanY<NBR_ELE_TOOLBAR_Y_MAX; ScanY++ ) 
		{
			for( ScanX=0; ScanX<NBR_ELE_TOOLBAR_X_MAX; ScanX++ ) 
			{
				pToolbarDatas->ToolbarBtnRadio[ ScanY ][ ScanX ] = NULL;
				pToolbarDatas->ToolbarImage[ ScanY ][ ScanX ] = NULL;
#if GTK_MAJOR_VERSION<3
				pToolbarDatas->ToolbarPixmap[ ScanY ][ ScanX ] = NULL;
#else
				pToolbarDatas->ToolBarSurface[ ScanY ][ ScanX ] = NULL;
#endif
			}
		}
	}
	ToolbarDatas[ NUM_TOOLBAR_FOR_LADDER ].PtrOnToolBarElementsList = ToolBarElementsLadder;
#ifdef SEQUENTIAL_SUPPORT
	ToolbarDatas[ NUM_TOOLBAR_FOR_SEQ ].PtrOnToolBarElementsList = ToolBarElementsSequential;
#endif
}

void CreateOneToolbar( GtkWidget * Box, int NumTable, char * PtrOnToolTipsText[][NBR_ELE_TOOLBAR_X_MAX] )
{
	StrElement ToolBarEle = {};
	int ScanToolBarX,ScanToolBarY;
	GSList * PtrListRadiosBtn = NULL;
	StrToolbarDatas * pToolbarDatas = &ToolbarDatas[ NumTable ];
	ScanToolBarX = 0;
	ScanToolBarY = 0;
	pToolbarDatas->ToolbarTable = gtk_table_new( NBR_ELE_TOOLBAR_X_MAX, NBR_ELE_TOOLBAR_Y_MAX, FALSE/*homogeneous*/ );
	gtk_box_pack_start (GTK_BOX(Box), pToolbarDatas->ToolbarTable, TRUE, TRUE, 0);

	do
	{
		ToolBarEle.Type = pToolbarDatas->PtrOnToolBarElementsList[ScanToolBarY][ScanToolBarX];
		ToolBarEle.ConnectedWithTop = 0;

		if ( ToolBarEle.Type!=0 )
		{
			char * pHelpText = gettext( PtrOnToolTipsText[ ScanToolBarY ][ ScanToolBarX ] );
#if GTK_MAJOR_VERSION<3
			GdkGC * gc = drawing_area->style->bg_gc[0];
			pToolbarDatas->ToolbarPixmap[ ScanToolBarY ][ ScanToolBarX ] = gdk_pixmap_new( GDK_DRAWABLE(drawing_area->window), PIXELS_SIZE_IN_TOOLBAR, PIXELS_SIZE_IN_TOOLBAR, -1 );
			gdk_draw_rectangle (GDK_DRAWABLE(pToolbarDatas->ToolbarPixmap[ ScanToolBarY ][ ScanToolBarX ]), gc, TRUE, 0, 0, PIXELS_SIZE_IN_TOOLBAR, PIXELS_SIZE_IN_TOOLBAR);
			cairo_t *cr = gdk_cairo_create( pToolbarDatas->ToolbarPixmap[ ScanToolBarY ][ ScanToolBarX ] );
#else
			pToolbarDatas->ToolBarSurface[ ScanToolBarY ][ ScanToolBarX ] = cairo_image_surface_create( CAIRO_FORMAT_A8, PIXELS_SIZE_IN_TOOLBAR, PIXELS_SIZE_IN_TOOLBAR );
			cairo_t *cr = cairo_create( pToolbarDatas->ToolBarSurface[ ScanToolBarY ][ ScanToolBarX ] );
#endif
			CreateFontPangoLayout( cr, PIXELS_SIZE_IN_TOOLBAR, DRAW_FOR_TOOLBAR );

#ifdef SEQUENTIAL_SUPPORT
			if ( NumTable==NUM_TOOLBAR_FOR_SEQ )
				DrawSeqElementForToolBar(cr, 0, 0, PIXELS_SIZE_IN_TOOLBAR, ToolBarEle.Type );
			else
#endif
				DrawElement( cr, 0, 0, PIXELS_SIZE_IN_TOOLBAR, PIXELS_SIZE_IN_TOOLBAR, ToolBarEle, TRUE);

#if GTK_MAJOR_VERSION<3
			pToolbarDatas->ToolbarImage[ ScanToolBarY ][ ScanToolBarX ] = gtk_image_new_from_pixmap( pToolbarDatas->ToolbarPixmap[ ScanToolBarY ][ ScanToolBarX ], NULL );
#else
			pToolbarDatas->ToolbarImage[ ScanToolBarY ][ ScanToolBarX ] =  gtk_image_new_from_surface( pToolbarDatas->ToolBarSurface[ ScanToolBarY ][ ScanToolBarX ] );
			cairo_surface_destroy( pToolbarDatas->ToolBarSurface[ ScanToolBarY ][ ScanToolBarX ] );
#endif
			pToolbarDatas->ToolbarBtnRadio[ ScanToolBarY ][ ScanToolBarX ] = gtk_radio_button_new( PtrListRadiosBtn );
			PtrListRadiosBtn = gtk_radio_button_get_group (GTK_RADIO_BUTTON(pToolbarDatas->ToolbarBtnRadio[ ScanToolBarY ][ ScanToolBarX ]));
			gtk_button_set_relief (GTK_BUTTON( pToolbarDatas->ToolbarBtnRadio[ ScanToolBarY ][ ScanToolBarX ] ), GTK_RELIEF_NONE);
			gtk_toggle_button_set_mode (GTK_TOGGLE_BUTTON(pToolbarDatas->ToolbarBtnRadio[ ScanToolBarY ][ ScanToolBarX ]), FALSE);
			gtk_container_add( GTK_CONTAINER( pToolbarDatas->ToolbarBtnRadio[ ScanToolBarY ][ ScanToolBarX ] ), pToolbarDatas->ToolbarImage[ ScanToolBarY ][ ScanToolBarX ] );
			gtk_widget_show( pToolbarDatas->ToolbarImage[ ScanToolBarY ][ ScanToolBarX ] );
			gtk_table_attach( GTK_TABLE( pToolbarDatas->ToolbarTable ), pToolbarDatas->ToolbarBtnRadio[ ScanToolBarY ][ ScanToolBarX ], 
								ScanToolBarX, ScanToolBarX+1, ScanToolBarY, ScanToolBarY+1,
								0, 0, 0, 0 );

			gtk_signal_connect( GTK_OBJECT (pToolbarDatas->ToolbarBtnRadio[ ScanToolBarY ][ ScanToolBarX ]), "clicked", GTK_SIGNAL_FUNC(ButtonToolbarSignal), GINT_TO_POINTER((int)ToolBarEle.Type) );

			if (pHelpText!=NULL )
			{
//ForGTK3, deprecated...				gtk_tooltips_set_tip (TheTooltips, pToolbarDatas->ToolbarBtnRadio[ ScanToolBarY ][ ScanToolBarX ], pHelpText, NULL);
				gtk_widget_set_tooltip_text( pToolbarDatas->ToolbarBtnRadio[ ScanToolBarY ][ ScanToolBarX ], pHelpText );
			}

			gtk_widget_show( pToolbarDatas->ToolbarBtnRadio[ ScanToolBarY ][ ScanToolBarX ] );

			cairo_destroy( cr );
		}//if ( ToolBarEle.Type!=0 )

		ScanToolBarX++;
		if (ScanToolBarX>=NBR_ELE_TOOLBAR_X_MAX)
		{
			ScanToolBarX = 0;
			ScanToolBarY++;
		}
	}
	while( pToolbarDatas->PtrOnToolBarElementsList[ScanToolBarY][ScanToolBarX]!=-1 );
}


void EditorInitGtk()
{
	GtkWidget *vbox;

	EditWindow = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ( GTK_WINDOW( EditWindow ), _("Editor"));

	vbox = gtk_vbox_new (FALSE, 0);
	gtk_container_add (GTK_CONTAINER (EditWindow), vbox);
	gtk_widget_show (vbox);

	EditorButtonAdd = gtk_button_new_with_label (_("Add"));
	gtk_box_pack_start (GTK_BOX (vbox), EditorButtonAdd, FALSE, FALSE, 0);
	gtk_signal_connect(GTK_OBJECT (EditorButtonAdd), "clicked",
						GTK_SIGNAL_FUNC(ButtonAddRung), 0);
	gtk_widget_show (EditorButtonAdd);
	EditorButtonIns = gtk_button_new_with_label (_("Insert"));
	gtk_box_pack_start (GTK_BOX (vbox), EditorButtonIns, FALSE, FALSE, 0);
	gtk_signal_connect(GTK_OBJECT (EditorButtonIns), "clicked",
						GTK_SIGNAL_FUNC(ButtonInsertRung), 0);
	gtk_widget_show (EditorButtonIns);
	EditorButtonDel = gtk_button_new_with_label (_("Delete"));
	gtk_box_pack_start (GTK_BOX (vbox), EditorButtonDel, FALSE, FALSE, 0);
	gtk_signal_connect(GTK_OBJECT (EditorButtonDel), "clicked",
						GTK_SIGNAL_FUNC(ButtonDeleteCurrentRung), 0);
	gtk_widget_show (EditorButtonDel);
	EditorButtonModify = gtk_button_new_with_label (_("Modify"));
	gtk_box_pack_start (GTK_BOX (vbox), EditorButtonModify, FALSE, FALSE, 0);
	gtk_signal_connect(GTK_OBJECT (EditorButtonModify), "clicked",
						GTK_SIGNAL_FUNC(ButtonModifyCurrentRung), 0);
	gtk_widget_show (EditorButtonModify);
	EditorButtonOk = gtk_button_new_with_label (_("Ok"));
	gtk_box_pack_start (GTK_BOX (vbox), EditorButtonOk, FALSE, FALSE, 0);
	gtk_signal_connect(GTK_OBJECT (EditorButtonOk), "clicked",
						GTK_SIGNAL_FUNC(ButtonOkCurrentRung), 0);
	EditorButtonCancel = gtk_button_new_with_label (_("Cancel"));
	gtk_box_pack_start (GTK_BOX (vbox), EditorButtonCancel, FALSE, FALSE, 0);
	gtk_signal_connect(GTK_OBJECT (EditorButtonCancel), "clicked",
						GTK_SIGNAL_FUNC(ButtonCancelCurrentRung), 0);

	InitAllForToolbar( );
//ForGTK3, deprecated...	TheTooltips = gtk_tooltips_new();
	/* Rungs elements toolbar */
	CreateOneToolbar( vbox, NUM_TOOLBAR_FOR_LADDER, ToolBarToolTipsTextLadder );
	gtk_widget_set_sensitive( ToolbarDatas[ NUM_TOOLBAR_FOR_LADDER ].ToolbarTable, FALSE );
	gtk_widget_show( ToolbarDatas[ NUM_TOOLBAR_FOR_LADDER ].ToolbarTable );
	/* Sequential elements toolbar */
#ifdef SEQUENTIAL_SUPPORT
	CreateOneToolbar( vbox, NUM_TOOLBAR_FOR_SEQ, ToolBarToolTipsTextSequential );
	gtk_widget_set_sensitive( ToolbarDatas[ NUM_TOOLBAR_FOR_SEQ ].ToolbarTable, FALSE );
#endif

	gtk_signal_connect( GTK_OBJECT(EditWindow), "delete_event",
		GTK_SIGNAL_FUNC(EditorWindowDeleteEvent), 0 );

	gtk_window_set_resizable( GTK_WINDOW( EditWindow ), FALSE );
//gtk_widget_show (EditWindow);

	EditDatas.NumElementSelectedInToolBar = -1;
	EditDatas.GhostZonePosiX = -1;
	EditDatas.GhostZonePosiY = -1;
	EditDatas.GhostZoneSizeX = -1;
	EditDatas.GhostZoneSizeY = -1;
}

