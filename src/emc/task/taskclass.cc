/********************************************************************
* Description: iotaskintf.cc
*   NML interface functions for IO
*
*   Based on a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#include <math.h>		// fabs()
#include <float.h>		// DBL_MAX
#include <string.h>		// memcpy() strncpy()
#include <stdlib.h>		// malloc()
#include <sys/wait.h>

#include "rcs.hh"		// RCS_CMD_CHANNEL, etc.
#include "rcs_print.hh"
#include "timer.hh"             // esleep, etc.
#include "emcglb.h"		// EMC_INIFILE

#include "python_plugin.hh"
#include "taskclass.hh"
#include <rtapi_string.h>

#include "hal.hh"

/********************************************************************
*
* Description: iocontrol_hal_init(void)
*
* Side Effects: Exports HAL pins.
*
* Called By: main
********************************************************************/
int Task::iocontrol_hal_init(void)
{
    iocontrol.add_pin("user-enable-out", hal_dir::OUT, iocontrol_data.user_enable_out);
    iocontrol.add_pin("emc-enable-in", hal_dir::IN, iocontrol_data.emc_enable_in);
    iocontrol.add_pin("user-request-enable", hal_dir::OUT, iocontrol_data.user_request_enable);
    iocontrol.add_pin("coolant-mist", hal_dir::OUT, iocontrol_data.coolant_mist);
    iocontrol.add_pin("coolant-flood", hal_dir::OUT, iocontrol_data.coolant_flood);
    iocontrol.add_pin("tool-prep-pocket", hal_dir::OUT, iocontrol_data.tool_prep_pocket);
    iocontrol.add_pin("tool-from-pocket", hal_dir::OUT, iocontrol_data.tool_from_pocket);
    iocontrol.add_pin("tool-prep-index", hal_dir::OUT, iocontrol_data.tool_prep_index);
    iocontrol.add_pin("tool-prep-number", hal_dir::OUT, iocontrol_data.tool_prep_number);
    iocontrol.add_pin("tool-number", hal_dir::OUT, iocontrol_data.tool_number);
    iocontrol.add_pin("tool-prepare", hal_dir::OUT, iocontrol_data.tool_prepare);
    iocontrol.add_pin("tool-prepared", hal_dir::IN, iocontrol_data.tool_prepared);
    iocontrol.add_pin("tool-change", hal_dir::OUT, iocontrol_data.tool_change);
    iocontrol.add_pin("tool-changed", hal_dir::IN, iocontrol_data.tool_changed);
    iocontrol.ready();
    if (iocontrol.error < 0)
        return -1;
    return 0;
}

/********************************************************************
*
* Description: hal_init_pins(void)
*
* Side Effects: Sets HAL pins default values.
*
* Called By: main
********************************************************************/
void Task::hal_init_pins(void)
{
    iocontrol_data.user_enable_out=0;    /* output, FALSE when EMC wants stop */
    iocontrol_data.user_request_enable=0;/* output, used to reset HAL latch */
    iocontrol_data.coolant_mist=0;       /* coolant mist output pin */
    iocontrol_data.coolant_flood=0;      /* coolant flood output pin */
    iocontrol_data.tool_prepare=0;       /* output, pin that notifies HAL it needs to prepare a tool */
    iocontrol_data.tool_prep_number=0;   /* output, pin that holds the tool number to be prepared, only valid when tool-prepare=TRUE */
    iocontrol_data.tool_prep_pocket=0;   /* output, pin that holds the pocketno for the tool to be prepared, only valid when tool-prepare=TRUE */
    iocontrol_data.tool_from_pocket=0;   /* output, always 0 at startup */
    iocontrol_data.tool_prep_index=0;      /* output, pin that holds the internal index (idx) of the tool to be prepared, for debug */
    iocontrol_data.tool_change=0;        /* output, notifies a tool-change should happen (emc should be in the tool-change position) */
    iocontrol_data.tool_number = emcioStatus.tool.toolInSpindle;
}

Task *task_methods;

// global status structure
EMC_IO_STAT *emcIoStatus = 0;

// glue

int emcIoInit() { return task_methods->emcIoInit(); }
int emcIoAbort(EMC_ABORT reason) { return task_methods->emcIoAbort(reason); }
int emcAuxEstopOn()  { return task_methods->emcAuxEstopOn(); }
int emcAuxEstopOff() { return task_methods->emcAuxEstopOff(); }
int emcCoolantMistOn() { return task_methods->emcCoolantMistOn(); }
int emcCoolantMistOff() { return task_methods->emcCoolantMistOff(); }
int emcCoolantFloodOn() { return task_methods->emcCoolantFloodOn(); }
int emcCoolantFloodOff() { return task_methods->emcCoolantFloodOff(); }
int emcToolPrepare(int tool) { return task_methods->emcToolPrepare(tool); }
int emcToolLoad() { return task_methods->emcToolLoad(); }
int emcToolUnload()  { return task_methods->emcToolUnload(); }
int emcToolLoadToolTable(const char *file) { return task_methods->emcToolLoadToolTable(file); }
int emcToolSetOffset(int pocket, int toolno, const EmcPose& offset, double diameter,
                     double frontangle, double backangle, int orientation) {
    return task_methods->emcToolSetOffset( pocket,  toolno,  offset,  diameter,
					   frontangle,  backangle,  orientation); }
int emcToolSetNumber(int number) { return task_methods->emcToolSetNumber(number); }

int emcTaskOnce(const char * /*filename*/, EMC_IO_STAT &emcioStatus)
{
	task_methods = new Task(emcioStatus);
    if (int res = task_methods->iocontrol_hal_init()) {
        return res;
    }
    return 0;
}


extern "C" PyObject* PyInit_interpreter(void);
extern "C" PyObject* PyInit_emccanon(void);
struct _inittab builtin_modules[] = {
    { "interpreter", PyInit_interpreter },
    { "emccanon", PyInit_emccanon },
    // any others...
    { NULL, NULL }
};

Task::Task(EMC_IO_STAT & emcioStatus_in) :
    emcioStatus(emcioStatus_in),
    iocontrol("iocontrol.0"),
    ini_filename(emc_inifile),
    tool_status(0)
    {

    IniFile inifile;

    if (inifile.Open(ini_filename)) {
        inifile.Find(&random_toolchanger, "RANDOM_TOOLCHANGER", "EMCIO");
        std::optional<const char*> t;
        if ((t = inifile.Find("TOOL_TABLE", "EMCIO")))
            tooltable_filename = strdup(*t);

        if ((t = inifile.Find("DB_PROGRAM", "EMCIO"))) {
            db_mode = tooldb_t::DB_ACTIVE;
            tooldata_set_db(db_mode);
            strncpy(db_program, *t, LINELEN - 1);
        }

        if (tooltable_filename != NULL && db_program[0] != '\0') {
            fprintf(stderr,"DB_PROGRAM active: IGNORING tool table file %s\n",
                    tooltable_filename);
        }
        inifile.Close();
    }

#ifdef TOOL_NML //{
    tool_nml_register( (CANON_TOOL_TABLE*)&emcStatus->io.tool.toolTable);
#else //}{
    tool_mmap_creator((EMC_TOOL_STAT*)&(emcioStatus.tool), random_toolchanger);
    tool_mmap_user();
    // initialize database tool finder:
#endif //}

    tooldata_init(random_toolchanger);
    if (db_mode == tooldb_t::DB_ACTIVE) {
        if (0 != tooldata_db_init(db_program, random_toolchanger)) {
            rcs_print_error("can't initialize DB_PROGRAM.\n");
            db_mode = tooldb_t::DB_NOTUSED;
            tooldata_set_db(db_mode);
        }
    }

    emcioStatus.status = RCS_STATUS::DONE;//TODO??
    emcioStatus.tool.pocketPrepped = -1;

    if(!random_toolchanger) {
        CANON_TOOL_TABLE tdata = tooldata_entry_init();
        tdata.pocketno =  0; //nonrandom init
        tdata.toolno   = -1; //nonrandom init
        if (tooldata_put(tdata,0) != IDX_OK) {
            UNEXPECTED_MSG;
        }
    }

    if (0 != tooldata_load(tooltable_filename)) {
        rcs_print_error("can't load tool table.\n");
    }

    if (random_toolchanger) {
        CANON_TOOL_TABLE tdata;
        if (tooldata_get(&tdata,0) != IDX_OK) {
            UNEXPECTED_MSG;//todo: handle error
        }
        emcioStatus.tool.toolInSpindle = tdata.toolno;
    } else {
        emcioStatus.tool.toolInSpindle = 0;
    }    
};


Task::~Task() {};

// set the have_tool_change_position global
static int readToolChange(IniFile *toolInifile)
{
    int retval = 0;
    std::optional<const char*> inistring;

    if ((inistring = toolInifile->Find("TOOL_CHANGE_POSITION", "EMCIO"))) {
	/* found an entry */
        if (9 == sscanf(*inistring, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                        &tool_change_position.tran.x,
                        &tool_change_position.tran.y,
                        &tool_change_position.tran.z,
                        &tool_change_position.a,
                        &tool_change_position.b,
                        &tool_change_position.c,
                        &tool_change_position.u,
                        &tool_change_position.v,
                        &tool_change_position.w)) {
            have_tool_change_position=9;
            retval=0;
        } else if (6 == sscanf(*inistring, "%lf %lf %lf %lf %lf %lf",
                        &tool_change_position.tran.x,
                        &tool_change_position.tran.y,
                        &tool_change_position.tran.z,
                        &tool_change_position.a,
                        &tool_change_position.b,
                        &tool_change_position.c)) {
	    tool_change_position.u = 0.0;
	    tool_change_position.v = 0.0;
	    tool_change_position.w = 0.0;
            have_tool_change_position = 6;
            retval = 0;
        } else if (3 == sscanf(*inistring, "%lf %lf %lf",
                               &tool_change_position.tran.x,
                               &tool_change_position.tran.y,
                               &tool_change_position.tran.z)) {
	    /* read them OK */
	    tool_change_position.a = 0.0;
	    tool_change_position.b = 0.0;
	    tool_change_position.c = 0.0;
	    tool_change_position.u = 0.0;
	    tool_change_position.v = 0.0;
	    tool_change_position.w = 0.0;
	    have_tool_change_position = 3;
	    retval = 0;
	} else {
	    /* bad format */
	    rcs_print("bad format for TOOL_CHANGE_POSITION\n");
	    have_tool_change_position = 0;
	    retval = -1;
	}
    } else {
	/* didn't find an entry */
	have_tool_change_position = 0;
    }
    return retval;
}

static int iniTool(const char *filename)
{
    int retval = 0;
    IniFile toolInifile;

    if (toolInifile.Open(filename) == false) {
	return -1;
    }
    // read the tool change positions
    if (0 != readToolChange(&toolInifile)) {
	retval = -1;
    }
    // close the inifile
    toolInifile.Close();

    return retval;
}

void Task::load_tool(int idx) {
    CANON_TOOL_TABLE tdata;
    if(random_toolchanger) {
        // swap the tools between the desired pocket and the spindle pocket

        CANON_TOOL_TABLE tzero,tpocket;
        if (   tooldata_get(&tzero,0    ) != IDX_OK
            || tooldata_get(&tpocket,idx) != IDX_OK) {
            UNEXPECTED_MSG; return;
        }
        // spindle-->pocket (specified by idx)
        tooldata_db_notify(SPINDLE_UNLOAD,tzero.toolno,idx,tzero);
        tzero.pocketno = tpocket.pocketno;
        if (tooldata_put(tzero,idx) != IDX_OK) {
            UNEXPECTED_MSG;
        }

        // pocket-->spindle (idx==0)
        tooldata_db_notify(SPINDLE_LOAD,tpocket.toolno,0,tpocket);
        tpocket.pocketno = 0;
        if (tooldata_put(tpocket,0) != IDX_OK) {
            UNEXPECTED_MSG;
        }

        if (0 != tooldata_save(tooltable_filename)) {
            emcioStatus.status = RCS_STATUS::ERROR;
        }
    } else if(idx == 0) {
        // on non-random tool-changers, asking for pocket 0 is the secret
        // handshake for "unload the tool from the spindle"
        tdata = tooldata_entry_init();
        tdata.toolno   = 0; // nonrandom unload tool from spindle
        tdata.pocketno = 0; // nonrandom unload tool from spindle
        if (tooldata_put(tdata,0) != IDX_OK) {
            UNEXPECTED_MSG; return;
        }
        if (tooldata_db_notify(SPINDLE_UNLOAD,0,0,tdata)) { UNEXPECTED_MSG; }
    } else {
        // just copy the desired tool to the spindle
        if (tooldata_get(&tdata,idx) != IDX_OK) {
            UNEXPECTED_MSG; return;
        }
        if (tooldata_put(tdata,0) != IDX_OK) {
            UNEXPECTED_MSG; return;
        }
        // notify idx==0 tool in spindle:
        CANON_TOOL_TABLE temp;
        if (tooldata_get(&temp,0) != IDX_OK) { UNEXPECTED_MSG; }
        if (tooldata_db_notify(SPINDLE_LOAD,temp.toolno,0,temp)) { UNEXPECTED_MSG; }
    }
} // load_tool()

void Task::reload_tool_number(int toolno) {
    CANON_TOOL_TABLE tdata;
    if(random_toolchanger) return; // doesn't need special handling here
    for(int idx = 1; idx <= tooldata_last_index_get(); idx++) { //note <=
        if (tooldata_get(&tdata,idx) != IDX_OK) {
            UNEXPECTED_MSG; return;
        }
        if(tdata.toolno == toolno) {
            load_tool(idx);
            break;
        }
    }
}

// NML commands

int Task::emcIoInit()//EMC_TOOL_INIT
{
    tooldata_load(tooltable_filename);
    reload_tool_number(emcioStatus.tool.toolInSpindle);

    if (0 != iniTool(emc_inifile)) {
	return -1;
    }
    return 0;
}

int Task::emcIoAbort(EMC_ABORT /*reason*/)//EMC_TOOL_ABORT_TYPE
{
    // only used in v2
    // this gets sent on any Task Abort, so it might be safer to stop
    // the spindle  and coolant
    rtapi_print_msg(RTAPI_MSG_DBG, "EMC_TOOL_ABORT\n");
    emcioStatus.coolant.mist = 0;
    emcioStatus.coolant.flood = 0;
    iocontrol_data.coolant_mist = 0;                /* coolant mist output pin */
    iocontrol_data.coolant_flood = 0;                /* coolant flood output pin */
    iocontrol_data.tool_change = 0;                /* abort tool change if in progress */
    iocontrol_data.tool_prepare = 0;                /* abort tool prepare if in progress */
    return 0;
}

int Task::emcAuxEstopOn()//EMC_AUX_ESTOP_ON_TYPE
{
    /* assert an ESTOP to the outside world (thru HAL) */
    iocontrol_data.user_enable_out = 0; //disable on ESTOP_ON
    hal_init_pins(); //resets all HAL pins to safe valuea
    return 0;
}

int Task::emcAuxEstopOff()
{
    /* remove ESTOP */
    iocontrol_data.user_enable_out = 1; //we're good to enable on ESTOP_OFF
    /* generate a rising edge to reset optional HAL latch */
    iocontrol_data.user_request_enable = 1;
    emcioStatus.aux.estop = 0;
    return 0;
}

int Task::emcCoolantMistOn()
{
    emcioStatus.coolant.mist = 1;
    iocontrol_data.coolant_mist = 1;
    return 0;
}

int Task::emcCoolantMistOff()
{
    emcioStatus.coolant.mist = 0;
    iocontrol_data.coolant_mist = 0;
    return 0;
}

int Task::emcCoolantFloodOn()
{
    emcioStatus.coolant.flood = 1;
    iocontrol_data.coolant_flood = 1;
    return 0;
}

int Task::emcCoolantFloodOff()
{
    emcioStatus.coolant.flood = 0;
    iocontrol_data.coolant_flood = 0;
    return 0;
}

int Task::emcToolPrepare(int toolno)
{
    int idx = 0;
    CANON_TOOL_TABLE tdata;
    idx  = tooldata_find_index_for_tool(toolno);
#ifdef TOOL_NML
    if (!random_toolchanger && toolno == 0) { idx = 0; }
#endif
    if (idx == -1) {  // not found
        emcioStatus.tool.pocketPrepped = -1;
    } else {
        if (tooldata_get(&tdata,idx) != IDX_OK) {
            UNEXPECTED_MSG;
        }
        rtapi_print_msg(RTAPI_MSG_DBG, "EMC_TOOL_PREPARE tool=%d idx=%d\n", toolno, idx);

        iocontrol_data.tool_prep_index = idx; // any type of changer

        // Note: some of the following logic could be simplified
        //       but is maintained to preserve runtests expectations
        // Set HAL pins for tool number, pocket, and index.
        if (random_toolchanger) {
            // RANDOM_TOOLCHANGER
            iocontrol_data.tool_prep_number = tdata.toolno;
            if (idx == 0) {
                emcioStatus.tool.pocketPrepped      = 0; // pocketPrepped is an idx
                iocontrol_data.tool_prep_pocket = 0;
                return 0;
            }
            (iocontrol_data.tool_prep_pocket) = tdata.pocketno;
        } else {
            // NON_RANDOM_TOOLCHANGER
            if (idx == 0) {
                emcioStatus.tool.pocketPrepped      = 0; // pocketPrepped is an idx
                iocontrol_data.tool_prep_number = 0;
                iocontrol_data.tool_prep_pocket = 0;
            } else {
                iocontrol_data.tool_prep_number = tdata.toolno;
                iocontrol_data.tool_prep_pocket = tdata.pocketno;
            }
        }
        // it doesn't make sense to prep the spindle pocket
        if (random_toolchanger && idx == 0) {
            emcioStatus.tool.pocketPrepped = 0; // idx
            return 0;
        }
    }

    /* then set the prepare pin to tell external logic to get started */
    iocontrol_data.tool_prepare = 1;
    // the feedback logic is done inside read_hal_inputs()
    // we only need to set RCS_EXEC if RCS_DONE is not already set by the above logic
    if (tool_status != 10) //set above to 10 in case PREP already finished (HAL loopback machine)
        emcioStatus.status = RCS_STATUS::EXEC;
    return 0;
}

int Task::emcToolLoad()//EMC_TOOL_LOAD_TYPE
{
    // it doesn't make sense to load a tool from the spindle pocket
    if (random_toolchanger && emcioStatus.tool.pocketPrepped == 0) {
        return 0;
    }

    // it's not necessary to load the tool already in the spindle
    CANON_TOOL_TABLE tdata;
    if (tooldata_get(&tdata, emcioStatus.tool.pocketPrepped) != IDX_OK) {
        UNEXPECTED_MSG;
    }
    if (!random_toolchanger && (emcioStatus.tool.pocketPrepped > 0) &&
        (emcioStatus.tool.toolInSpindle == tdata.toolno) ) {
        return 0;
    }

    if (emcioStatus.tool.pocketPrepped != -1) {
        //notify HW for toolchange
        iocontrol_data.tool_change = 1;
        // the feedback logic is done inside read_hal_inputs() we only
        // need to set RCS_EXEC if RCS_DONE is not already set by the
        // above logic
        if (tool_status != 11)
            // set above to 11 in case LOAD already finished (HAL
            // loopback machine)
            emcioStatus.status = RCS_STATUS::EXEC;
    }
    return 0;
}

int Task::emcToolUnload()//EMC_TOOL_UNLOAD_TYPE
{
    emcioStatus.tool.toolInSpindle = 0;
    return 0;
}

int Task::emcToolLoadToolTable(const char *file)//EMC_TOOL_LOAD_TOOL_TABLE_TYPE
{
    if(!strlen(file)) file = tooltable_filename;//use filename from ini if none is provided
    tooldata_load(file);
    reload_tool_number(emcioStatus.tool.toolInSpindle);
    return 0;
}

int Task::emcToolSetOffset(int idx, int toolno, const EmcPose& offset, double diameter,
                     double frontangle, double backangle, int orientation)//EMC_TOOL_SET_OFFSET
{

    int o;
    double d, f, b;

    d = diameter;
    f = frontangle;
    b = backangle;
    o = orientation;

    rtapi_print_msg(RTAPI_MSG_DBG,
            "EMC_TOOL_SET_OFFSET idx=%d toolno=%d zoffset=%lf, "
            "xoffset=%lf, diameter=%lf, "
            "frontangle=%lf, backangle=%lf, orientation=%d\n",
            idx, toolno, offset.tran.z, offset.tran.x, d, f, b, o);
    CANON_TOOL_TABLE tdata;
    if (tooldata_get(&tdata,idx) != IDX_OK) {
        UNEXPECTED_MSG;
    }
    tdata.toolno = toolno;
    tdata.offset = offset;
    tdata.diameter = d;
    tdata.frontangle = f;
    tdata.backangle = b;
    tdata.orientation = o;
    if (tooldata_put(tdata,idx) != IDX_OK) {
        UNEXPECTED_MSG;
    }
    if (0 != tooldata_save(tooltable_filename)) {
        emcioStatus.status = RCS_STATUS::ERROR;
    }
    //TODO
    // if (io_db_mode == DB_ACTIVE) {
    //     int pno = idx; // for random_toolchanger
    //     if (!random_toolchanger) { pno = tdata.pocketno; }
    //     if (tooldata_db_notify(TOOL_OFFSET,toolno,pno,tdata)) {
    //         UNEXPECTED_MSG;
    //     }
    // }

    return 0;
}

int Task::emcToolSetNumber(int number)//EMC_TOOL_SET_NUMBER
{
    int idx;

    idx = number;//TODO: should be toolno
    CANON_TOOL_TABLE tdata;
    if (tooldata_get(&tdata,idx) != IDX_OK) {
        UNEXPECTED_MSG;
    }
    load_tool(idx);

    idx=0; // update spindle (fix legacy behavior)
    if (tooldata_get(&tdata,idx) != IDX_OK) {
        UNEXPECTED_MSG;
    }
    emcioStatus.tool.toolInSpindle = tdata.toolno;
    rtapi_print_msg(RTAPI_MSG_DBG,
            "EMC_TOOL_SET_NUMBER old_loaded_tool=%d new_idx_number=%d new_tool=%d\n"
            , emcioStatus.tool.toolInSpindle, idx, tdata.toolno);
    //likewise in HAL
    iocontrol_data.tool_number = emcioStatus.tool.toolInSpindle;
    if (emcioStatus.tool.toolInSpindle == 0) {
        emcioStatus.tool.toolFromPocket = iocontrol_data.tool_from_pocket = 0; // no tool in spindle
    }

    return 0;
}

/********************************************************************
*
* Description: read_tool_inputs(void)
*                        Reads the tool-pin values from HAL
*                        this function gets called once per cycle
*                        It sets the values for the emcioStatus.aux.*
*
* Returns:        returns which of the status has changed
*                we then need to update through NML (a bit different as read_hal_inputs)
*
* Side Effects: updates values
*
* Called By: main every CYCLE
********************************************************************/
int Task::read_tool_inputs(void)
{
    if (iocontrol_data.tool_prepare && iocontrol_data.tool_prepared) {
        emcioStatus.tool.pocketPrepped = iocontrol_data.tool_prep_index; //check if tool has been (idx) prepared
        iocontrol_data.tool_prepare = 0;
        emcioStatus.status = RCS_STATUS::DONE;  // we finally finished to do tool-changing, signal task with RCS_DONE
        return 10; //prepped finished
    }

    if (iocontrol_data.tool_change && iocontrol_data.tool_changed) {
        if(!random_toolchanger && emcioStatus.tool.pocketPrepped == 0) {
            emcioStatus.tool.toolInSpindle = 0;
            emcioStatus.tool.toolFromPocket = iocontrol_data.tool_from_pocket = 0;
        } else {
            // the tool now in the spindle is the one that was prepared
            CANON_TOOL_TABLE tdata;
            if (tooldata_get(&tdata,emcioStatus.tool.pocketPrepped) != IDX_OK) {
                UNEXPECTED_MSG; return -1;
            }
            emcioStatus.tool.toolInSpindle = tdata.toolno;
            emcioStatus.tool.toolFromPocket = iocontrol_data.tool_from_pocket = tdata.pocketno;
        }
        if (emcioStatus.tool.toolInSpindle == 0) {
             emcioStatus.tool.toolFromPocket = iocontrol_data.tool_from_pocket = 0;
        }
        iocontrol_data.tool_number = emcioStatus.tool.toolInSpindle; //likewise in HAL
        load_tool(emcioStatus.tool.pocketPrepped);
        emcioStatus.tool.pocketPrepped = -1; //reset the tool preped number, -1 to permit tool 0 to be loaded
        iocontrol_data.tool_prep_number = 0; //likewise in HAL
        iocontrol_data.tool_prep_pocket = 0; //likewise in HAL
        iocontrol_data.tool_prep_index = 0; //likewise in HAL
        iocontrol_data.tool_change = 0; //also reset the tool change signal
        emcioStatus.status = RCS_STATUS::DONE;        // we finally finished to do tool-changing, signal task with RCS_DONE
        return 11; //change finished
    }
    return 0;
}

void Task::run(){ // called periodically from emctaskmain.cc
    tool_status = read_tool_inputs();
    if (iocontrol_data.emc_enable_in == 0) //check for estop from HW
        emcioStatus.aux.estop = 1;
    else
        emcioStatus.aux.estop = 0;
}
