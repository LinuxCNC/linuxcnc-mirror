/********************************************************************
* Description:  canterp.cc
*               This file, 'canterp.cc', implements an interpreter 
*               of a file of printed canonical interfaces
*
* Author: proctor
* License: GPL Version 2
*    
* Copyright (c) 2005 All rights reserved.
*
* Last change: 
********************************************************************/
/*
  canterp.cc

  Straight-through interpreter of a file of printed canonical interface
  commands like these:

  1 N..... USE_LENGTH_UNITS(CANON_UNITS_MM)
  2 N..... SET_ORIGIN_OFFSETS(0.0000, 0.0000, 0.0000)
  3 N..... SET_FEED_REFERENCE(CANON_XYZ)
  4 N..... COMMENT("Circle Diamond Square Program")
  5 N..... COMMENT("Tom Kramer")
  6 N..... COMMENT("26-Sep-1994")
  7 N..... COMMENT("Assumes 4"x4"x2" finished stock")
  8 N..... COMMENT("Top of stock at Z=2"")
  9 N..... COMMENT("Cutter does not descend more than 0.94" below top")
  10 N0080  MIST_OFF()
  11 N0080  FLOOD_OFF()
  12 N0090  USE_TOOL_LENGTH_OFFSET(1.0000)
  13 N0110  STRAIGHT_TRAVERSE(0.0000, 0.0000, 3.0000)
  14 N0140  SET_FEED_RATE(16.0000)
  15 N0140  SET_SPINDLE_SPEED(0, 3500.0000)
  16 N0140  START_SPINDLE_CLOCKWISE(0)
  17 N0150  COMMENT("MILLING AN ENCLOSED POCKET")
  18 N0160  STRAIGHT_TRAVERSE(0.0000, 3.9150, 3.0000)
  19 N0170  STRAIGHT_TRAVERSE(0.0000, 3.9150, 2.1000)
  20 N0180  COMMENT("start left circle zigzag")
  21 N0180  STRAIGHT_FEED(0.0000, 3.9150, 1.6875)
  22 N0190  STRAIGHT_FEED(4.0000, 3.9150, 1.6875)
  23 N0200  STRAIGHT_FEED(4.0000, 3.7250, 1.6875)
  24 N0210  STRAIGHT_FEED(0.0000, 3.7250, 1.6875)
  25 N0220  STRAIGHT_FEED(0.0000, 3.5350, 1.6875)
  26 N0230  STRAIGHT_FEED(1.4370, 3.5350, 1.6875)
  27 N0240  ARC_FEED(1.0704, 3.3450, 2.0000, 2.0000, 1, 1.6875)

  which typically come out of one of Tom Kramer's interpreters.
  The first two columns are ignored, the rest is converted to
  equivalent canonical calls.
*/

#include <stdio.h>		// FILE, fopen(), fclose()
#include <string.h>		// strcpy()
#include <ctype.h>		// isspace()
#include <limits.h>
#include <algorithm>
#include "config.h"
#include "emc/linuxcnc.h"
#include "emc/nml_intf/interp_return.hh"
#include "emc/nml_intf/canon.hh"
#include "emc/rs274ngc/interp_base.hh"
#include "modal_state.hh"

static char the_command[LINELEN] = { 0 };	// our current command
static char the_command_name[LINELEN] = { 0 };	// just the name part
static char the_command_args[LINELEN] = { 0 };	// just the args part

class Canterp : public InterpBase {
public:
    Canterp () : f(0), filename{} {}
    char *error_text(int errcode, char *buf, size_t buflen) override;
    char *stack_name(int index, char *buf, size_t buflen) override;
    char *line_text(char *buf, size_t buflen) override;
    char *file_name(char *buf, size_t buflen) override;
    size_t line_length() override;
    int sequence_number() override;
    int ini_load(const char *inifile) override;
    int init() override;
    int execute() override;
    int execute(const char *line) override;
    int execute(const char *line, int line_number) override;
    int synch() override;
    int exit() override;
    int open(const char *filename) override;
    int read() override;
    int read(const char *line) override;
    int close() override;
    int reset() override;
    int line() override;
    int call_level() override;
    char *command(char *buf, size_t buflen) override;
    char *file(char *buf, size_t buflen) override;
    int on_abort(int reason, const char *message) override;
    void active_g_codes(int active_gcodes[ACTIVE_G_CODES]) override;
    void active_m_codes(int active_mcodes[ACTIVE_M_CODES]) override;
    void active_settings(double active_settings[ACTIVE_SETTINGS]) override;
    int active_modes(int g_codes[ACTIVE_G_CODES],
            int m_codes[ACTIVE_M_CODES],
            double settings[ACTIVE_SETTINGS],
            StateTag const &tag) override;
    int restore_from_tag(StateTag const &tag) override;
    void print_state_tag(StateTag const &tag) override;
    void set_loglevel(int level) override;
    void set_loop_on_main_m99(bool state) override;
    FILE* get_stdout() override { return NULL; };
    FILE *f;
    char filename[PATH_MAX];
};

char *Canterp::error_text(int errcode, char *buf, size_t buflen) {
    if(errcode < INTERP_MIN_ERROR) snprintf(buf, buflen, "OK %d", errcode);
    else snprintf(buf, buflen, "ERROR %d", errcode);
    return buf;
}

char *Canterp::stack_name(int index, char *buf, size_t buflen) {
    snprintf(buf, buflen, "<stack %d>", index);
    return buf;
}

int Canterp::ini_load(const char * /*inifile*/) {
    return 0;
}

/*
  We expect lines like this:

  {ws}1<white>N1<ws>cmd{ws}({ws}{arg1{ws}}){extra}

 */

static char *skipwhite(char *ptr)
{
    while (isspace(*ptr))
	ptr++;
    return ptr;
}

static char *findwhite(char *ptr)
{
    while (!isspace(*ptr) && 0 != *ptr)
	ptr++;
    return ptr;
}

static int canterp_parse(char *buffer)
{
    char *ptr = buffer;
    char *cmd_ptr = the_command;
    char *name_ptr = the_command_name;
    char *args_ptr = the_command_args;
    char *last_quote_ptr;
    int inquote;

    *cmd_ptr = 0;
    *name_ptr = 0;
    *args_ptr = 0;

    // skip leading white space, return if nothing else found
    if (0 == *(ptr = skipwhite(ptr)))
	return 0;

    // ---cut here if no leading line number, N-number columns---

    // skip the first column, return if nothing else found
    if (0 == *(ptr = findwhite(ptr)))
	return 0;

    // skip following white space, return if nothing else found
    if (0 == *(ptr = skipwhite(ptr)))
	return 0;

    // skip the second column, return if nothing else found
    if (0 == *(ptr = findwhite(ptr)))
	return 0;

    // skip following white space, return if nothing else found
    if (0 == *(ptr = skipwhite(ptr)))
	return 0;

    // ---cut to here---

    // we got something; store the name, up to space or the '('
    while (!isspace(*ptr) && '(' != *ptr && 0 != *ptr) {
	*name_ptr++ = *ptr;
	*cmd_ptr++ = *ptr++;
    }
    if (isspace(*ptr))
	ptr = skipwhite(ptr);

    if (0 == *ptr) {
	// no parens, just a command name
	*name_ptr = 0;
	*cmd_ptr = 0;
	return 0;
    }

    if ('(' != *ptr) {
	// we're missing the '(', so flag an error
	*name_ptr = 0;
	*cmd_ptr = 0;
	return INTERP_ERROR;
    }
    // we got the '(', so keep going
    *name_ptr = 0;		// terminate the name
    *cmd_ptr++ = *ptr++;	// add the '(' to the full command

    /*
       now we're at the args; skip first and last quotes when building
       args_ptr, and make commas spaces for easy parsing later
     */
    last_quote_ptr = 0;
    inquote = 0;
    while (')' != *ptr && 0 != *ptr) {
	if ('"' == *ptr) {
	    if (!inquote) {
		// here's the first quote, so suppress it in args_ptr
		inquote = 1;
		*cmd_ptr++ = *ptr++;
		continue;
	    }
	    /*
	       else it's a quote-in-quote, so mark it as the last so we
	       can delete it, thus handling any internal quotes, perhaps
	       used for inch marks
	     */
	    last_quote_ptr = args_ptr;
	}
	*args_ptr++ = (*ptr == ',' ? ' ' : *ptr);
	*cmd_ptr++ = *ptr++;
    }
    if (0 == *ptr) {
	// finished args without ')', so error
	*args_ptr = 0;
	*cmd_ptr = 0;
	return INTERP_ERROR;
    }
    if (0 != last_quote_ptr)
	*last_quote_ptr = 0;
    *args_ptr = 0;
    *cmd_ptr++ = ')';
    *cmd_ptr = 0;

    return 0;
}

int Canterp::read(const char *line) {
    return canterp_parse((char *) line);
}

int Canterp::read() {
    char buf[LINELEN];
    if(!f) return INTERP_ERROR;
    if(!fgets(buf, sizeof(buf), f)) return INTERP_ENDFILE;
    return canterp_parse(buf);
}

int Canterp::execute(const char *line) {
    int retval;
    double d1 = 0, d2 = 0, d3 = 0, d4 = 0, d5 = 0, d6 = 0, d7 = 0, d8 = 0, d9 = 0, d10 = 0, d11 = 0;
    int i1, i2, ln=-1;
    char s1[256];

    if (line) {
	retval = canterp_parse((char *) line);
	if (retval)
	    return retval;
    }

    // a blank line
    if (strlen(the_command_name) == 0) return INTERP_OK;

    if (!strcmp(the_command_name, "STRAIGHT_FEED")) {
	if (9 != sscanf(the_command_args, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			&d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9)) {
	    return INTERP_ERROR;
	}
	STRAIGHT_FEED(ln, d1, d2, d3, d4, d5, d6, d7, d8, d9);
	return 0;
    }

    if (!strcmp(the_command_name, "ARC_FEED")) {
	if (12 != sscanf(the_command_args,
			"%lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf",
			&d1, &d2, &d3, &d4, &i1, &d5, &d6, &d7, &d8, &d9, &d10, &d11)) {
	    return INTERP_ERROR;
	}
	ARC_FEED(ln, d1, d2, d3, d4, i1, d5, d6, d7, d8, d9, d10, d11);
	return 0;
    }

    if (!strcmp(the_command_name, "STRAIGHT_TRAVERSE")) {
	if (9 != sscanf(the_command_args, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			&d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9)) {
	    return INTERP_ERROR;
	}
	STRAIGHT_TRAVERSE(ln, d1, d2, d3, d4, d5, d6, d7, d8, d9);
	return 0;
    }

    if (!strcmp(the_command_name, "STRAIGHT_PROBE")) {
	if (6 != sscanf(the_command_args, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			&d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9)) {
	    return INTERP_ERROR;
	}
	STRAIGHT_PROBE(ln, d1, d2, d3, d4, d5, d6, d7, d8, d9, 0);
	return 0;
    }

    if (!strcmp(the_command_name, "USE_LENGTH_UNITS")) {
	if (!strcmp(the_command_args, "CANON_UNITS_MM")) {
	    USE_LENGTH_UNITS(CANON_UNITS_MM);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_UNITS_CM")) {
	    USE_LENGTH_UNITS(CANON_UNITS_MM);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_UNITS_INCHES")) {
	    USE_LENGTH_UNITS(CANON_UNITS_INCHES);
	    return 0;
	}
	return INTERP_ERROR;
    }

#if 0
    if (!strcmp(the_command_name, "SET_ORIGIN_OFFSETS")) {
	if (6 != sscanf(the_command_args, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			&d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9)) {
	    return INTERP_ERROR;
	}
	SET_ORIGIN_OFFSETS(d1, d2, d3, d4, d5, d6, d7, d8, d9);
	return 0;
    }
#endif

    if (!strcmp(the_command_name, "SET_FEED_REFERENCE")) {
	if (!strcmp(the_command_args, "CANON_WORKPIECE")) {
	    SET_FEED_REFERENCE(CANON_WORKPIECE);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_XYZ")) {
	    SET_FEED_REFERENCE(CANON_XYZ);
	    return 0;
	}
	return INTERP_ERROR;
    }

    if (!strcmp(the_command_name, "SELECT_PLANE")) {
	if (!strcmp(the_command_args, "CANON_PLANE_XY")) {
	    SELECT_PLANE(CANON_PLANE::XY);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_PLANE_YZ")) {
	    SELECT_PLANE(CANON_PLANE::YZ);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_PLANE_XZ")) {
	    SELECT_PLANE(CANON_PLANE::XZ);
	    return 0;
	}
	return INTERP_ERROR;
    }

    if (!strcmp(the_command_name, "COMMENT")) {
	COMMENT(the_command_args);
	return 0;
    }

    if (!strcmp(the_command_name, "MIST_OFF")) {
	MIST_OFF();
	return 0;
    }

    if (!strcmp(the_command_name, "FLOOD_OFF")) {
	FLOOD_OFF();
	return 0;
    }

    if (!strcmp(the_command_name, "MIST_ON")) {
	MIST_ON();
	return 0;
    }

    if (!strcmp(the_command_name, "FLOOD_ON")) {
	FLOOD_ON();
	return 0;
    }

#if 0
    if (!strcmp(the_command_name, "USE_TOOL_LENGTH_OFFSET")) {
	if (1 != sscanf(the_command_args, "%lf %lf %lf", &d1, &d2, &d3)) {
	    return INTERP_ERROR;
	}
	USE_TOOL_LENGTH_OFFSET(d1, d2, d3);
	return 0;
    }
#endif

    if (!strcmp(the_command_name, "SET_FEED_RATE")) {
	if (1 != sscanf(the_command_args, "%lf", &d1)) {
	    return INTERP_ERROR;
	}
	SET_FEED_RATE(d1);
	return 0;
    }

#if 0
    if (!strcmp(the_command_name, "SET_TRAVERSE_RATE")) {
	if (1 != sscanf(the_command_args, "%lf", &d1)) {
	    return INTERP_ERROR;
	}
	SET_TRAVERSE_RATE(d1);
	return 0;
    }
#endif

    if (!strcmp(the_command_name, "SELECT_TOOL")) {
	if (1 != sscanf(the_command_args, "%d", &i1)) {
	    return INTERP_ERROR;
	}
	SELECT_TOOL(i1);
	return 0;
    }

    if (!strcmp(the_command_name, "CHANGE_TOOL")) {
	CHANGE_TOOL();
	return 0;
    }

    if (!strcmp(the_command_name, "DWELL")) {
	if (1 != sscanf(the_command_args, "%lf", &d1)) {
	    return INTERP_ERROR;
	}
	DWELL(d1);
	return 0;
    }

#if 0
    if (!strcmp(the_command_name, "SPINDLE_RETRACT")) {
	SPINDLE_RETRACT();
	return 0;
    }

    if (!strcmp(the_command_name, "SPINDLE_RETRACT_TRAVERSE")) {
	SPINDLE_RETRACT_TRAVERSE();
	return 0;
    }

    if (!strcmp(the_command_name, "LOCK_SPINDLE_Z")) {
	LOCK_SPINDLE_Z();
	return 0;
    }

    if (!strcmp(the_command_name, "USE_SPINDLE_FORCE")) {
	USE_SPINDLE_FORCE();
	return 0;
    }

    if (!strcmp(the_command_name, "USE_NO_SPINDLE_FORCE")) {
	USE_NO_SPINDLE_FORCE();
	return 0;
    }

    if (!strcmp(the_command_name, "CLAMP_AXIS")) {
	if (!strcmp(the_command_args, "CANON_AXIS_X")) {
	    CLAMP_AXIS(CANON_AXIS_X);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_AXIS_Y")) {
	    CLAMP_AXIS(CANON_AXIS_Y);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_AXIS_Z")) {
	    CLAMP_AXIS(CANON_AXIS_Z);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_AXIS_A")) {
	    CLAMP_AXIS(CANON_AXIS_A);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_AXIS_B")) {
	    CLAMP_AXIS(CANON_AXIS_B);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_AXIS_C")) {
	    CLAMP_AXIS(CANON_AXIS_C);
	    return 0;
	}
	return INTERP_ERROR;
    }

    if (!strcmp(the_command_name, "UNCLAMP_AXIS")) {
	if (!strcmp(the_command_args, "CANON_AXIS_X")) {
	    UNCLAMP_AXIS(CANON_AXIS_X);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_AXIS_Y")) {
	    UNCLAMP_AXIS(CANON_AXIS_Y);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_AXIS_Z")) {
	    UNCLAMP_AXIS(CANON_AXIS_Z);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_AXIS_A")) {
	    UNCLAMP_AXIS(CANON_AXIS_A);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_AXIS_B")) {
	    UNCLAMP_AXIS(CANON_AXIS_B);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_AXIS_C")) {
	    UNCLAMP_AXIS(CANON_AXIS_C);
	    return 0;
	}
	return INTERP_ERROR;
    }

    if (!strcmp(the_command_name, "SET_CUTTER_RADIUS_COMPENSATION")) {
	if (1 != sscanf(the_command_args, "%lf", &d1)) {
	    return INTERP_ERROR;
	}
	SET_CUTTER_RADIUS_COMPENSATION(d1);
	return 0;
    }

    if (!strcmp(the_command_name, "START_CUTTER_RADIUS_COMPENSATION")) {
	if (1 != sscanf(the_command_args, "%d", &i1)) {
	    return INTERP_ERROR;
	}
	START_CUTTER_RADIUS_COMPENSATION(i1);
	return 0;
    }

    if (!strcmp(the_command_name, "STOP_CUTTER_RADIUS_COMPENSATION")) {
	STOP_CUTTER_RADIUS_COMPENSATION();
	return 0;
    }
#endif

    if (!strcmp(the_command_name, "START_SPEED_FEED_SYNCH")) {
	if (3 != sscanf(the_command_args, "%d %lf %d", &i1, &d1, &i2)) {
            return INTERP_ERROR;
        }
	START_SPEED_FEED_SYNCH(i1, d1, i2);
	return 0;
    }

    if (!strcmp(the_command_name, "STOP_SPEED_FEED_SYNCH")) {
	STOP_SPEED_FEED_SYNCH();
	return 0;
    }

    if (!strcmp(the_command_name, "SET_SPINDLE_SPEED")) {
	if (2 != sscanf(the_command_args, "%i %lf", &i1, &d1)) {
	    return INTERP_ERROR;
	}
	SET_SPINDLE_SPEED(i1, d1);
	return 0;
    }

    if (!strcmp(the_command_name, "START_SPINDLE_CLOCKWISE")) {
    	if (1 != sscanf(the_command_args, "%d", &i1)) {
    	    return INTERP_ERROR;
    	}
	START_SPINDLE_CLOCKWISE(i1);
	return 0;
    }

    if (!strcmp(the_command_name, "START_SPINDLE_COUNTERCLOCKWISE")) {
    	if (1 != sscanf(the_command_args, "%d", &i1)) {
    	    return INTERP_ERROR;
    	}
	START_SPINDLE_COUNTERCLOCKWISE(i1);
	return 0;
    }

    if (!strcmp(the_command_name, "STOP_SPINDLE_TURNING")) {
    	if (1 != sscanf(the_command_args, "%d", &i1)) {
    	    return INTERP_ERROR;
    	}
	STOP_SPINDLE_TURNING(i1);
	return 0;
    }

    if (!strcmp(the_command_name, "ORIENT_SPINDLE")) {
	if (3 != sscanf(the_command_args, "%d %lf %s", &i1, &d1, s1)) {
	    return INTERP_ERROR;
	}
	if (!strcmp(s1, "CANON_CLOCKWISE")) {
	    ORIENT_SPINDLE(i1, d2, CANON_CLOCKWISE);
	    return 0;
	}
	if (!strcmp(s1, "CANON_COUNTERCLOCKWISE")) {
	    ORIENT_SPINDLE(i1, d2, CANON_COUNTERCLOCKWISE);
	    return 0;
	}
	return INTERP_ERROR;
    }

    if (!strcmp(the_command_name, "DISABLE_SPEED_OVERRIDE")) {
    	if (1 != sscanf(the_command_args, "%d", &i1)) {
    	    return INTERP_ERROR;
    	}
	DISABLE_SPEED_OVERRIDE(i1);
	return 0;
    }

    if (!strcmp(the_command_name, "DISABLE_FEED_OVERRIDE")) {
	DISABLE_FEED_OVERRIDE();
	return 0;
    }

    if (!strcmp(the_command_name, "ENABLE_SPEED_OVERRIDE")) {
    	if (1 != sscanf(the_command_args, "%d", &i1)) {
    	    return INTERP_ERROR;
    	}
	ENABLE_SPEED_OVERRIDE(i1);
	return 0;
    }

    if (!strcmp(the_command_name, "ENABLE_FEED_OVERRIDE")) {
	ENABLE_FEED_OVERRIDE();
	return 0;
    }

    if (!strcmp(the_command_name, "PROGRAM_STOP")) {
	PROGRAM_STOP();
	return 0;
    }

    if (!strcmp(the_command_name, "OPTIONAL_PROGRAM_STOP")) {
	OPTIONAL_PROGRAM_STOP();
	return 0;
    }

    if (!strcmp(the_command_name, "PROGRAM_END")) {
	PROGRAM_END();
	return 0;
    }

    if (!strcmp(the_command_name, "PALLET_SHUTTLE")) {
	PALLET_SHUTTLE();
	return 0;
    }

    if (!strcmp(the_command_name, "SET_MOTION_CONTROL_MODE")) {
	if (!strcmp(the_command_args, "CANON_EXACT_PATH")) {
	    SET_MOTION_CONTROL_MODE(CANON_EXACT_PATH, 0);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_EXACT_STOP")) {
	    SET_MOTION_CONTROL_MODE(CANON_EXACT_STOP, 0);
	    return 0;
	}
	if (!strcmp(the_command_args, "CANON_CONTINUOUS")) {
	    SET_MOTION_CONTROL_MODE(CANON_CONTINUOUS, 0);
	    return 0;
	}
	return INTERP_ERROR;
    }

    if (!strcmp(the_command_name, "MESSAGE")) {
	MESSAGE(the_command_args);
	return 0;
    }

    if (!strcmp(the_command_name, "INIT_CANON")) {
	INIT_CANON();
	return 0;
    }

    if (!strcmp(the_command_name, "TURN_PROBE_OFF")) {
	TURN_PROBE_OFF();
	return 0;
    }

    if (!strcmp(the_command_name, "TURN_PROBE_ON")) {
	TURN_PROBE_ON();
	return 0;
    }

    fprintf(stderr, "canterp: unrecognized canonical command %s\n",
	    the_command);
    return INTERP_ERROR;
}

int Canterp::execute(const char *line, int /*line_number*/) {
    return execute(line);
}

int Canterp::execute() {
    return execute(0);
}

int Canterp::open(const char *newfilename) {
    if(f) fclose(f);
    f = fopen(newfilename, "r");
    if(f) snprintf(filename, sizeof(filename), "%s", newfilename);
    return f ? INTERP_OK : INTERP_ERROR;
}

int Canterp::close() {
    return INTERP_OK;
}

int Canterp::exit() { return 0; }
int Canterp::synch() { return 0; }
int Canterp::reset() { return 0; }
int Canterp::line() { return 0; }
int Canterp::call_level() { return 0; }

char *Canterp::line_text(char *buf, size_t bufsize) {
   snprintf(buf, bufsize, "<Canterp::line_text>");
   return buf;
}
char *Canterp::file_name(char *buf, size_t bufsize) {
   snprintf(buf, bufsize, "%s", filename);
   return buf;
}
char *Canterp::file(char *buf, size_t bufsize) {
   snprintf(buf, bufsize, "%s", filename);
   return buf;
}
int Canterp::on_abort(int reason, const char *message)
{
    fprintf(stderr, "Canterp::on_abort reason=%d message='%s'", reason, message);
    reset();
    return INTERP_OK;
}
char *Canterp::command(char *buf, size_t bufsize) {
   snprintf(buf, bufsize, "<Canterp::command>");
   return buf;
}
size_t Canterp::line_length() {
   return 0;
}
int Canterp::sequence_number() {
   return -1;
}
int Canterp::init() { return INTERP_OK; }
void Canterp::active_g_codes(int gees[ACTIVE_G_CODES]) { std::fill(gees, gees + ACTIVE_G_CODES, 0); }
void Canterp::active_m_codes(int emms[ACTIVE_M_CODES]) { std::fill(emms, emms + ACTIVE_M_CODES, 0); }
void Canterp::active_settings(double sets[ACTIVE_SETTINGS]) { std::fill(sets, sets + ACTIVE_SETTINGS, 0.0); }
//NOT necessary for canterp
int Canterp::restore_from_tag(StateTag const & /*tag*/) {return -1;}
void Canterp::print_state_tag(StateTag const & /*tag*/) {}

int Canterp::active_modes(int g_codes[ACTIVE_G_CODES],
			  int m_codes[ACTIVE_M_CODES],
			  double settings[ACTIVE_SETTINGS],
			  StateTag const & /*tag*/){ (void)g_codes; (void)m_codes; (void)settings; return -1;}

void Canterp::set_loglevel(int /*level*/) {}
void Canterp::set_loop_on_main_m99(bool /*state*/) {}

InterpBase *makeInterp() { return new Canterp; }
