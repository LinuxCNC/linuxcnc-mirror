/********************************************************************
* Description: interp_o_word.cc
*
*
* Author: Kenneth Lerman
* License: GPL Version 2
* System: Linux
*    
* Copyright 2005 All rights reserved.
*
* Last change: Michael Haberler 7/2011
*
********************************************************************/

#define BOOST_PYTHON_MAX_ARITY 4
#include <boost/python/list.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/dict.hpp>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <new>
#include "rs274ngc.hh"
#include "rs274ngc_return.hh"
#include "interp_return.hh"
#include "interp_internal.hh"
#include "rs274ngc_interp.hh"
#include "python_plugin.hh"
#include "interp_python.hh"
#include <rtapi_string.h>	// rtapi_strlcpy()

namespace bp = boost::python;

//========================================================================
// Functions for control stuff (O-words)
//========================================================================

/*
  Given the root of a directory tree and a file name,
  find the path to the file, if any.
*/

int Interp::findFile( // ARGUMENTS
		     char *direct,  // the directory to start looking in
		     char *target,  // the name of the file to find
		     char *foundFileDirect) // where to store the result
{
    FILE *file;
    DIR *aDir;
    struct dirent *aFile;
    char targetPath[PATH_MAX+1];

    snprintf(targetPath, PATH_MAX, "%s/%s", direct, target);
    file = fopen(targetPath, "r");
    if (file) {
        rtapi_strlcpy(foundFileDirect, direct, PATH_MAX);
        fclose(file);
        return INTERP_OK;
    }
    aDir = opendir(direct);
    if (!aDir) {
	ERS(NCE_FILE_NOT_OPEN);
    }

    while ((aFile = readdir(aDir))) {
        if (aFile->d_type == DT_DIR &&
	    (0 != strncmp(aFile->d_name, "..", 3)) &&
	    (0 != strncmp(aFile->d_name, ".", 2))) {

            char path[PATH_MAX+1];
            snprintf(path, PATH_MAX, "%s/%s", direct, aFile->d_name);
            if (INTERP_OK == findFile(path, target, foundFileDirect)) {
	        closedir(aDir);
                return INTERP_OK;
            }
        }
    }
    closedir(aDir);
    ERS(NCE_FILE_NOT_OPEN);
}


/*
 *  this now uses STL maps for offset access
 */
int Interp::control_save_offset(block_pointer block,        /* pointer to a block of RS274/NGC instructions */
				setup_pointer settings)     /* pointer to machine settings */
{
    static char name[] = "control_save_offset";
    offset_pointer op = NULL;

    logOword("Entered:%s for o_name:|%s|", name, block->o_name);

    if (control_find_oword(block, settings, &op) == INTERP_OK) {
	if (settings->sequence_number -1 == op->sequence_number)
	    // hit this definition before; must be in illegal location
	    ERS(_("File:%s line:%d sub: o|%s| found in illegal location"),
		settings->filename, settings->sequence_number, block->o_name);
	else
	    // already exists
	    ERS(_("File:%s line:%d redefining sub: o|%s| already defined in file:%s"),
		settings->filename, settings->sequence_number,
		block->o_name,
		op->filename);
    }
    offset new_offset;

    new_offset.type = block->o_type;
    new_offset.offset = block->offset;
    new_offset.filename = strstore(settings->filename);
    new_offset.repeat_count = -1;
    // the sequence number has already been bumped, so save
    // the proper value
    new_offset.sequence_number = settings->sequence_number - 1;
    settings->offset_map[block->o_name] = new_offset;
    return INTERP_OK;
}


int Interp::control_find_oword(block_pointer block,      // pointer to block
			       setup_pointer settings,   // pointer to machine settings
			       offset_pointer *op)       // pointer to offset descriptor
{
    static char name[] = "control_find_oword";
    offset_map_iterator it;

    it = settings->offset_map.find(block->o_name);
    if (it != settings->offset_map.end()) {
	*op =  &it->second;
	return INTERP_OK;
    } else {
	logOword("%s: Unknown oword name: |%s|", name, block->o_name);
	ERS(NCE_UNKNOWN_OWORD_NUMBER);
    }
}

const char *o_ops[] = {
    "O_none",
    "O_sub",
    "O_endsub",
    "O_call",
    "O_do",
    "O_while",
    "O_if",
    "O_elseif",
    "O_else",
    "O_endif",
    "O_break",
    "O_continue",
    "O_endwhile",
    "O_return",
    "O_repeat",
    "O_endrepeat",
    "M_98",
    "M_99",
    "O_",

};

const char *call_statenames[] = {
    "CS_NORMAL",
    "CS_REEXEC_PROLOG",
    "CS_REEXEC_PYBODY",
    "CS_REEXEC_EPILOG",
    "CS_REEXEC_PYOSUB",
};

const char *call_typenames[] = {
    "CT_NONE",
    "CT_NGC_OWORD_SUB",
    "CT_NGC_M98_SUB",
    "CT_PYTHON_OWORD_SUB",
    "CT_REMAP",  
};

int Interp::execute_call(setup_pointer settings, 
			 context_pointer current_frame,
			 int call_type)
{
    int status = INTERP_OK;
    int i;
    bp::list plist;

    context_pointer previous_frame = &settings->sub_context[settings->call_level-1];

    block_pointer eblock = &EXECUTING_BLOCK(*settings);

    logOword("execute_call %s type=%s state=%s cl=%d rl=%d", 
	     current_frame->subName, 
	     call_typenames[call_type],
	     call_statenames[settings->call_state],
	     settings->call_level,settings->remap_level);

    switch (call_type) {

    case CT_NONE:
	ERS("BUG:  execute_call():  arrived with call_type=CT_NONE");
	break;

    case CT_NGC_OWORD_SUB:
    case CT_NGC_M98_SUB:
	// if we were skipping, no longer
	if (settings->skipping_o) {
	    logOword("case O_call/M_98 -- no longer skipping to:|%s|",
		     settings->skipping_o);
	    settings->skipping_o = NULL;
	}

	// copy parameters from context
	// save old values of parameters
	if (call_type != CT_NGC_M98_SUB) // M98:  pass #1..#30 from parent
	    for (i = 0; i < INTERP_SUB_PARAMS; i++) {
		previous_frame->saved_params[i] =
		    settings->parameters[i + INTERP_FIRST_SUBROUTINE_PARAM];
		settings->parameters[i + INTERP_FIRST_SUBROUTINE_PARAM] =
		    eblock->params[i];
	    }

	// Set up return to next block (may be overridden by M98)
	if (settings->file_pointer == NULL)
	    // if the previous file was NULL, mark position as -1 so as not to
	    // reopen it on return.
	    previous_frame->position = -1;
	else
	    previous_frame->position = ftell(settings->file_pointer);
	previous_frame->filename = strstore(settings->filename);
	previous_frame->sequence_number = settings->sequence_number;
	logOword("saving return location[cl=%d]: %s:%d offset=%ld", 
		 settings->call_level-1,
		 previous_frame->filename,
		 previous_frame->sequence_number,
		 previous_frame->position);

	if (FEATURE(OWORD_N_ARGS)) {
	    // let any  Oword sub know the number of parameters
	    CHP(add_named_param("n_args", PA_READONLY));
	    CHP(store_named_param(settings, "n_args",
				  (double )eblock->param_cnt,
				  OVERRIDE_READONLY));
	}

	// M98 w/ L-word:  Handle looping
	if (eblock->o_type == M_98 && eblock->l_flag) {
	    // Set up loop counter

	    // Loop count from L-word
	    if (previous_frame->m98_loop_counter == -1)
		// If m98_loop_counter == -1, this is a new loop
		previous_frame->m98_loop_counter = round_to_int(eblock->l_number);
	    logOword("Looping on O%s; remaining: %d",
		     eblock->o_name, previous_frame->m98_loop_counter);

	    // if repeats remain, set up another loop
	    previous_frame->m98_loop_counter--;
	    if (previous_frame->m98_loop_counter > 0) {

		// Set return location to repeat this block for next
		// iteration
		previous_frame->position = eblock->offset;
		// Unbump line number for next iteration
		previous_frame->sequence_number--;

	    } else
		// No loops remain; reset loop counter
		previous_frame->m98_loop_counter = -1;
	}


	if (eblock->o_type == M_98 && eblock->l_flag && eblock->l_number == 0)
	    logOword("M98 L0 instruction; skipping call");
	else if (control_back_to(eblock, settings) == INTERP_ERROR) {
	    settings->call_level--;
	    ERS(NCE_UNABLE_TO_OPEN_FILE,eblock->o_name);
	    return INTERP_ERROR;
	}

	break;

    case CT_PYTHON_OWORD_SUB:
	switch (settings->call_state) {
	case CS_NORMAL:
	    settings->return_value = 0.0;
	    settings->value_returned = 0;
	    previous_frame->sequence_number = settings->sequence_number;
	    previous_frame->filename = strstore(settings->filename);
	    plist.append(*settings->pythis); // self
	    for(int i = 0; i < eblock->param_cnt; i++)
		plist.append(eblock->params[i]); // positional args
	    current_frame->pystuff.impl->tupleargs = bp::tuple(plist);
	    current_frame->pystuff.impl->kwargs = bp::dict();

	case CS_REEXEC_PYOSUB:
	    if (settings->call_state ==  CS_REEXEC_PYOSUB)
		CHP(read_inputs(settings));
	    status = pycall(settings, current_frame, OWORD_MODULE,
			    current_frame->subName, 
			    settings->call_state == CS_NORMAL ? PY_OWORDCALL : PY_FINISH_OWORDCALL);
	    CHKS(status == INTERP_ERROR, "pycall(%s.%s) failed", OWORD_MODULE, current_frame->subName) ;
	    switch (status = handler_returned(settings, current_frame, current_frame->subName, true)) {
	    case INTERP_EXECUTE_FINISH:
		settings->call_state = CS_REEXEC_PYOSUB;
		break;
	    default:
		settings->call_state = CS_NORMAL;
		settings->sequence_number = previous_frame->sequence_number;
		CHP(status);
		// M73 auto-restore is of dubious value in a Python subroutine
		CHP(leave_context(settings,false)); 
	    }
	    break;
	}
	break;

    case CT_REMAP:
	block_pointer cblock = &CONTROLLING_BLOCK(*settings);
	remap_pointer remap = cblock->executing_remap;

	switch (settings->call_state) {
	case CS_NORMAL:
	    if (remap->remap_py || remap->prolog_func || remap->epilog_func) {
		CHKS(!PYUSABLE, "%s (remapped) uses Python functions, but the Python plugin is not available", 
		     remap->name);
		plist.append(*settings->pythis);   //self
		current_frame->pystuff.impl->tupleargs = bp::tuple(plist);
		current_frame->pystuff.impl->kwargs = bp::dict();
	    }
	    if (remap->argspec && (strchr(remap->argspec, '@') == NULL)) {
		// add_parameters will decorate kwargs as per argspec
		// if named local parameters specified
		CHP(add_parameters(settings, cblock, NULL));
	    }
	    // fall through

	case CS_REEXEC_PROLOG:
	    if (remap->prolog_func) { 
		status = pycall(settings, current_frame, REMAP_MODULE,remap->prolog_func,
				settings->call_state == CS_NORMAL ? PY_PROLOG : PY_FINISH_PROLOG);
		CHKS(status == INTERP_ERROR, "pycall(%s.%s) failed", REMAP_MODULE, remap->prolog_func);
		switch (status = handler_returned(settings, current_frame, current_frame->subName, false)) {
		case INTERP_EXECUTE_FINISH:
		    settings->call_state = CS_REEXEC_PROLOG;
		    return status;
		default:
		    settings->call_state  = CS_NORMAL;
		    //settings->sequence_number = previous_frame->sequence_number;
		    CHP(status);
		}
	    }
	    // fall through

	case CS_REEXEC_PYBODY:
	    if (remap->remap_py) { 
		status = pycall(settings, current_frame, REMAP_MODULE, remap->remap_py,
				settings->call_state == CS_NORMAL ? PY_BODY : PY_FINISH_BODY);
		CHP(status);
		switch (status = handler_returned(settings, current_frame, current_frame->subName, false)) {
		case INTERP_EXECUTE_FINISH:
		    settings->call_state = CS_REEXEC_PYBODY;
		    return status;
		default:
		    settings->call_state = CS_NORMAL;
		    settings->sequence_number = previous_frame->sequence_number;
		    CHP(status);
		    // epilog is not supported on python body -  makes no sense
		    CHP(leave_context(settings,false)); 
		    ERP(remap_finished(-cblock->phase));
		}
	    }

	    // call the NGC remap procedure
	    assert(settings->call_state == CS_NORMAL);
	    if (remap->remap_ngc) {
		CHP(execute_call(settings, current_frame,
				 CT_NGC_OWORD_SUB));
	    }
	}
    }
    return status;
}

// this is executed only for NGC subs, either normal ones or part of a remap
// subs whose name is a Py callable are handled inline in execute_call()
// since there is no corresponding O_return/O_endsub to execute.
int Interp::execute_return(setup_pointer settings, context_pointer current_frame,int call_type)
{
    int status = INTERP_OK;

    logOword("execute_return %s type=%s state=%s", 
	     current_frame->subName, 
	     call_typenames[call_type],
	     call_statenames[settings->call_state]);

    block_pointer cblock = &CONTROLLING_BLOCK(*settings);
    block_pointer eblock = &EXECUTING_BLOCK(*settings);
    context_pointer previous_frame = settings->call_level > 0 ?
        &settings->sub_context[settings->call_level - 1] : nullptr;

    // if level is not zero, in a call
    // otherwise in a defn
    // if we were skipping, no longer
    if (settings->skipping_o && (eblock->o_type == O_endsub)) {
	logOword("case O_%s -- no longer skipping to:|%s|",
		 (eblock->o_type == O_endsub) ? "endsub" : "return",
		 settings->skipping_o);
	settings->skipping_o = NULL;
    }
    
    switch (call_type) {

    case CT_REMAP:
	switch (settings->call_state) {
	case CS_NORMAL:
   	case CS_REEXEC_EPILOG:
	    if (cblock->executing_remap && cblock->executing_remap->epilog_func) {
		if (settings->call_state ==  CS_REEXEC_EPILOG)
		    CHP(read_inputs(settings));
		status = pycall(settings, current_frame, REMAP_MODULE,
	    			cblock->executing_remap->epilog_func,
				settings->call_state == CS_NORMAL ? PY_EPILOG : PY_FINISH_EPILOG);
		CHP(status);
		switch (status = handler_returned(settings, current_frame, current_frame->subName, false)) {
		case INTERP_EXECUTE_FINISH:
		    settings->call_state = CS_REEXEC_EPILOG;
		    eblock->call_type = CT_REMAP;
		    CHP(status);
		default:
		    settings->call_state = CS_NORMAL;
		    settings->sequence_number = previous_frame->sequence_number;
		    CHP(status);
		    // leave_context() is done by falling through into CT_NGC_OWORD_SUB code
		}
	    }
	}
	// fall through to normal NGC return handling 
    case CT_NGC_OWORD_SUB:
    case CT_NGC_M98_SUB:
    case CT_NONE:  // sub definition
	if (settings->call_level != 0) {

	    // restore subroutine parameters.
	    if (call_type != CT_NGC_M98_SUB) // M98:  pass #1..#30 from parent
		for(int i = 0; i < INTERP_SUB_PARAMS; i++) {
		    settings->parameters[i+INTERP_FIRST_SUBROUTINE_PARAM] =
			previous_frame->saved_params[i];
		}

	    // file at this level was marked as closed, so dont reopen.
	    if (previous_frame->position == -1) {
		if (settings->file_pointer) fclose(settings->file_pointer);
		settings->file_pointer = NULL;
		rtapi_strxcpy(settings->filename, "");
	    } else {
		if(settings->file_pointer == NULL) {
		    ERS(NCE_FILE_NOT_OPEN);
		}
		//!!!KL must open the new file, if changed
		if (0 != strcmp(settings->filename, previous_frame->filename))  {
		    fclose(settings->file_pointer);
		    settings->file_pointer = fopen(previous_frame->filename, "r");
		    if (settings->file_pointer == NULL)  {
			ERS(NCE_CANNOT_REOPEN_FILE, 
			    previous_frame->filename,
			    strerror(errno));
		    }
		    rtapi_strxcpy(settings->filename, previous_frame->filename);
		}
		fseek(settings->file_pointer, previous_frame->position, SEEK_SET);
		settings->sequence_number = previous_frame->sequence_number;
		logOword("endsub/return: %s:%d pos=%ld", 
			 settings->filename,previous_frame->sequence_number,
			 previous_frame->position);

	    }
	    // cleanups on return:
	    CHP(leave_context(settings, true));

	    // if this was a remap frame we're done
	    if (current_frame->context_status & REMAP_FRAME) {
		CHP(remap_finished(-cblock->phase));
	    }


	    settings->sub_name = 0;
	    if (previous_frame->subName)  {
		settings->sub_name = previous_frame->subName;
	    } else {
		settings->sub_name = NULL;
	    }
	} else { // call_level == 0
	    // a definition
	    if (eblock->o_type == O_endsub) {
		CHKS((settings->defining_sub != 1), NCE_NOT_IN_SUBROUTINE_DEFN);
		// no longer skipping or defining
		if (settings->skipping_o)  {
		    logOword("case O_endsub in defn -- no longer skipping to:|%s|",
			     settings->skipping_o);
		    settings->skipping_o = NULL;
		}
		settings->defining_sub = 0;
		settings->sub_name = NULL;
	    }
	}
    } 
    return status;
}

// this is executed for m99 in the main program, signifying an endless
// loop; m99 main program endless loop is handled in
// interp_convert.cc, but because this is like an O-word function
// skipping around the file, it's placed here instead.
void Interp::loop_to_beginning(setup_pointer settings)
{
    logOword("loop_to_beginning state=%s file=%s",
	     call_statenames[settings->call_state],
	     settings->filename);

    // scroll back to beginning of file/first block
    fseek(settings->file_pointer, 0, SEEK_SET);
    settings->sequence_number = 0;
}

// Prevent any possibility of the Posix version of basename(3) being used
// (possibly altering the argument). Therefore, local reimplementation
// of the *GNU version* of basename(3).
static const char *thebasename(const char *filename)
{
     const char *p = strrchr(filename, '/');
     return p ? p + 1 : filename;
}

//
// TESTME!!! MORE THOROUGHLY !!!KL
//
// In the past, calls had to be to predefined subs
//
// Now they don't. Do things in the following sequence:
// 1 -- if o_word is already defined, just go back to it, else
// 2 -- if there is a file with the name of the o_word,
//             open it and start skipping (as in 3, below)
// 3 -- skip to the o_word (will be an error if not found)
//
int Interp::control_back_to( block_pointer block, // pointer to block
			     setup_pointer settings)   // pointer to machine settings
{
    static char name[] = "control_back_to";
    char newFileName[PATH_MAX];
    FILE *newFP;
    offset_map_iterator it;
    offset_pointer op;
    const char *obasename = thebasename(block->o_name);
    logOword("Entered:%s %s", name, obasename);
    it = settings->offset_map.find(obasename);

    // #1 already defined
    if (it != settings->offset_map.end()) {
	op = &it->second;
	if ((settings->filename[0] != 0) &
	    (settings->file_pointer == NULL))  {
	    ERS(NCE_FILE_NOT_OPEN);
	}
	if (0 != strcmp(settings->filename,
			op->filename)) {
	    // open the new file...
	    newFP = fopen(op->filename, "r");
	    // set the line number
	    settings->sequence_number = 0;
            if (strlen(op->filename) >= sizeof(settings->filename)) {
                fclose(settings->file_pointer);
                logOword("filename too long: %s", op->filename);
                ERS(NCE_UNABLE_TO_OPEN_FILE, op->filename);
            }
            strncpy(settings->filename, op->filename, sizeof(settings->filename));

	    if (newFP) {
		// close the old file...
		if (settings->file_pointer) // only close if it was open
		    fclose(settings->file_pointer);
		settings->file_pointer = newFP;
	    } else {
		logOword("Unable to open file: %s", settings->filename);
		ERS(NCE_UNABLE_TO_OPEN_FILE,settings->filename);
	    }
	}
	if (settings->file_pointer) { // only seek if it was open
	    fseek(settings->file_pointer,
		  op->offset, SEEK_SET);
	}
	settings->sequence_number = op->sequence_number;
	return INTERP_OK;
    }

    // #2 open the File
    newFP = find_ngc_file(settings, block->o_name, newFileName);

    if (newFP) {
	logOword("fopen: |%s| OK", newFileName);
	settings->sequence_number = 0;

	// close the old file...
	if (settings->file_pointer)
	    fclose(settings->file_pointer);
	settings->file_pointer = newFP;
        if (strlen(newFileName) >= sizeof(settings->filename)) {
            logOword("new filename '%s' is too long (max len %zu)\n", newFileName, sizeof(settings->filename)-1);
            ERS(NCE_UNABLE_TO_OPEN_FILE, newFileName);
        }
        strncpy(settings->filename, newFileName, sizeof(settings->filename));
    } else {
	char *dirname = getcwd(NULL, 0);
	logOword("fopen: |%s| failed CWD:|%s|", newFileName,
		 dirname);
	free(dirname);
    }

    settings->skipping_o = obasename; // start skipping
    settings->skipping_to_sub = obasename; // start skipping
    settings->skipping_start = settings->sequence_number;
    return INTERP_OK;
}


int Interp::handler_returned( setup_pointer settings,  context_pointer active_frame, 
			      const char *name, bool osub)
{
    int status = INTERP_OK;
    
    switch (active_frame->pystuff.impl->py_return_type) {
    case RET_YIELD:
	// yield <integer> was executed
	CHP(active_frame->pystuff.impl->py_returned_int);
	break;

    case RET_STOPITERATION:  // a bare 'return' in a generator - treat as INTERP_OK
    case RET_NONE:
	break;
	
    case RET_DOUBLE: 
	if (osub) { // float values are ok for osubs
	    settings->return_value = active_frame->pystuff.impl->py_returned_double;
	    settings->value_returned = 1;
	} else {
	    ERS("handler_returned: %s returned double: %f - invalid", 
			name, active_frame->pystuff.impl->py_returned_double);
	}
	break;

    case RET_INT: 
	if (osub) { // let's be liberal with types - widen to double return value
	    settings->return_value = (double) active_frame->pystuff.impl->py_returned_int;
	    settings->value_returned = 1;
	} else 
	    return active_frame->pystuff.impl->py_returned_int;

    case RET_ERRORMSG:
	status = INTERP_ERROR;
	break;

    }
    return status;
}
 

// prepare a new call frame.
int Interp::enter_context(setup_pointer settings, block_pointer block) 
{
    logOword("enter_context cl=%d->%d type=%s", 
	     settings->call_level, settings->call_level+1, 
	     call_typenames[block->call_type]);

    settings->call_level++;
    if (settings->call_level >= INTERP_SUB_ROUTINE_LEVELS) {
	ERS(NCE_TOO_MANY_SUBROUTINE_LEVELS);
    }
    context_pointer frame = &settings->sub_context[settings->call_level];
    frame->clear();
    // mark frame for finishing remap
    frame->context_status = (block->call_type  == CT_REMAP) ? REMAP_FRAME : 0;
    frame->subName = block->o_name;
    frame->pystuff.impl->py_returned_int = 0;
    frame->pystuff.impl->py_returned_double = 0.0;
    frame->pystuff.impl->py_return_type = -1;
    // distinguish call frames: oword,m99,python,remap
    frame->call_type = block->call_type;
    return INTERP_OK;
}

int Interp::leave_context(setup_pointer settings, bool restore) 
{
    context_pointer leaving_frame = &settings->sub_context[settings->call_level];

   if (settings->call_level < 1) {
	ERS(NCE_CALL_STACK_UNDERRUN);
    }
    logOword("leave_context cl=%d->%d  type=%s state=%s" , 
	     settings->call_level, settings->call_level-1, 
	     call_typenames[leaving_frame->call_type],
	     call_statenames[settings->call_state]);

    free_named_parameters(leaving_frame);
    leaving_frame->subName = NULL;
    settings->call_level--;  // drop back

    if (restore && ((leaving_frame->context_status &
		     (CONTEXT_RESTORE_ON_RETURN|CONTEXT_VALID)) ==
		    (CONTEXT_RESTORE_ON_RETURN|CONTEXT_VALID))) {
	// a valid previous context was marked by an M73 as auto-restore

    	// NB: this means an M71 invalidate context will prevent an
    	// auto-restore on return/endsub
    	CHP(restore_settings(settings, settings->call_level + 1));
    }
    return INTERP_OK;
}

/************************************************************************/
/* convert_control_functions

Returned Value: int (INTERP_OK)
Side effects:
   Changes the flow of control.

Called by: execute

Calls: control_skip_to
       control_back_to
       control_save_offset
*/

int Interp::convert_control_functions(block_pointer block, // pointer to a block of RS274/NGC instructions
				      setup_pointer settings)  // pointer to machine settings
{
    int status = INTERP_OK;
    context_pointer current_frame;
    offset_pointer op = NULL;

    logOword("convert_control_functions %s", o_ops[block->o_type]);

    // must skip if skipping
    if (settings->skipping_o && (0 != strcmp(settings->skipping_o, block->o_name)))  {
	logOword("skipping to line: |%s|", settings->skipping_o);
	return INTERP_OK;
    }
    if (settings->skipping_to_sub && (block->o_type != O_sub)
	&& (block->o_type != O_)) {
	logOword("skipping to sub: |%s|", settings->skipping_to_sub);
	return INTERP_OK;
    }

    // if skipping_o was set, we are now on a line which contains that O-word.
    // if skipping_to_sub was set, we are now on the 'O-name sub' definition line.

    switch (block->o_type) {

    case O_none:
	// not an error because we use this to signal that we
	// are not evaluating functions
	break;

    case O_sub:
    case O_:
	current_frame = &settings->sub_context[settings->call_level];

	// Mixing Fanuc- and rs274ngc-style sub calls & defs is not allowed:
	// - Fanuc:  'O....' subprogram must be called with 'M98'
	CHKS((block->o_type == O_ &&
	      current_frame->call_type != CT_NGC_M98_SUB &&
	      current_frame->call_type != CT_NONE),
	     "Fanuc 'O....' subroutine must be called with 'M98'");
	// - rs274ngc:  'O.... sub' subprogram must be called with 'O.... call'
	CHKS((block->o_type == O_sub &&
	      current_frame->call_type != CT_NGC_OWORD_SUB &&
	      current_frame->call_type != CT_PYTHON_OWORD_SUB &&
	      current_frame->call_type != CT_REMAP &&
	      current_frame->call_type != CT_NONE),
	     "'O.... sub' subroutine must be called with 'O.... call'");

	// if we were skipping, no longer
	if (settings->skipping_o) {
	    logOword("sub(o_|%s|) was skipping to here", settings->skipping_o);
	    // skipping to a sub means that we must define this now
	    CHP(control_save_offset( block, settings));
	    logOword("no longer skipping to:|%s|", settings->skipping_o);
	    settings->skipping_o = NULL; // this IS our block number
	}
	settings->skipping_to_sub = NULL; // this IS our block number

	if (block->o_type == O_) {
	    // Done:  continue executing into Fanuc-style programs w/o skipping
	    logOword("Entering Fanuc-style program number %s", block->o_name);
	    break;
	}

	// if the level is not zero, this is a call
	// not the definition
	if (settings->call_level != 0) {
	    logOword("call:%f:%f:%f",
		     settings->parameters[1],
		     settings->parameters[2],
		     settings->parameters[3]);
	} else {
	    // a definition. We're on the O<name> sub line.
	    logOword("started a rs274-style sub-program defn: %s",
		     block->o_name);
	    CHKS((settings->defining_sub == 1), NCE_NESTED_SUBROUTINE_DEFN);
	    CHP(control_save_offset( block, settings));

	    // start skipping to the corresponding ensub.
	    settings->skipping_o = block->o_name; 
	    settings->skipping_start = settings->sequence_number;
	    settings->defining_sub = 1;
	    settings->sub_name = block->o_name;
	    logOword("will now skip to: |%s|", settings->sub_name);
	}
	break;

    case O_endsub:
    case O_return:
    case M_99:

	// For M99, this only handles return from a subprogram
	CHKS((block->o_type == M_99 && settings->call_level == 0),
	     "Bug:  Reached convert_control_functions() "
	     "from M99 in main program");

	if  ((settings->call_level == 0) && 
	     (settings->sub_name == NULL)) {
	    // detect a standalone 'o<label> return|endsub'
	    OERR(_("%d: not in a subroutine definition: '%s'"),
		 settings->sequence_number, settings->linetext);
	} 
	current_frame = &settings->sub_context[settings->call_level];

	// 'O....' sub defn must be closed with 'M99'
	CHKS((current_frame->call_type == CT_NGC_M98_SUB &&
	      block->o_type != M_99),
	     "Fanuc 'O....' subroutine definition must end with 'M99'");
	// 'O.... sub' sub defn must be closed with 'O.... endsub' or
	// 'O.... return'
	CHKS((current_frame->call_type == CT_NGC_OWORD_SUB &&
	      block->o_type != O_endsub && block->o_type != O_return),
	     "'O.... endsub' or 'O.... return' must follow 'O.... sub' "
	     "subroutine definition");

	// proper label semantics (only refer to defined sub, within sub defn etc)
	// is handled in read_o() for return & endsub
	CHP(execute_return(settings, current_frame,
			   current_frame->call_type)); 
	break;

    case O_call:
    case M_98:
	// only enter new frame if not reexecuting a Python handler 
	// which returned INTERP_EXECUTE_FINISH
	if (settings->call_state == CS_NORMAL) {
	    CHP(enter_context(settings, block));
	}
	current_frame = &settings->sub_context[settings->call_level];
	CHP(execute_call(settings, current_frame,
			 current_frame->call_type)); 
	break;

      case O_do:
	// if we were skipping, no longer
	settings->skipping_o = NULL;
	// save the loop point
	// we hit this again on loop back -- so test first
	if(INTERP_OK != control_find_oword(block, settings, &op)) // &index))
	    {
		// save offset if not found in offset table
		CHP(control_save_offset( block, settings));
	    }
	break;

    case O_repeat:
	if (control_find_oword(block, settings, &op) == INTERP_OK) {
	    if (settings->sequence_number != (op->sequence_number + 1)) 
	        OERR(_("%d: duplicate O-word label: '%s' - defined in line %d"),
		    settings->sequence_number, 
		    settings->linetext, op->sequence_number + 1);
	} else 
	    CHP(control_save_offset(block, settings));

	// if we were skipping, no longer
	settings->skipping_o = NULL;
	status = control_find_oword(block, settings, &op); // &index);

	// test if not already seen OR
	// if seen and this is a repeat
	if ((status != INTERP_OK) ||
	    (op->type == block->o_type))	{
	    // this is the beginning of a 'repeat' loop
	    // add it to the table if not already there
	    if(status != INTERP_OK)
		CHP(control_save_offset( block, settings));

	    // note the repeat count.  it should only be calculated at the
	    // start of the repeat loop.
	    control_find_oword(block, settings, &op); // &index);
	    if(op->repeat_count == -1)
		op->repeat_count =
		    round_to_int(settings->test_value);

	    // are we still repeating?
	    if(op->repeat_count > 0) {
		// execute forward
		logOword("executing forward: [%s] in 'repeat' test value-- %g",
			 block->o_name, settings->test_value);
		// one less repeat remains
		op->repeat_count--;
	    } else {
		// skip forward
		logOword("skipping forward: [%s] in 'repeat'",
			 block->o_name);
		settings->skipping_o = block->o_name;
		settings->skipping_start = settings->sequence_number;
		// cause the repeat count to be recalculated
		// if we do this loop again
		op->repeat_count = -1;
	    }
	}
	break;

    case O_while:
	if (control_find_oword(block, settings, &op) == INTERP_OK) {
	    if ((op->type != O_do) &&
		(settings->sequence_number != (op->sequence_number + 1)))
	        OERR(_("%d: duplicate O-word label: '%s' - defined in line %d"),
		    settings->sequence_number, 
		    settings->linetext, op->sequence_number + 1);
	} else 
	    // record only if this is a while/endwhile loop
	    CHP(control_save_offset(block, settings));

	// if we were skipping, no longer
	settings->skipping_o = NULL;
	status = control_find_oword(block, settings, &op); //  &index);

	// test if not already seen OR
	// if seen and this is a while (alternative is that it is a do)
	if ((status != INTERP_OK) ||
	    (op->type == block->o_type))  {

	    // this is the beginning of a 'while' loop
	    // add it to the table if not already there
	    if (status != INTERP_OK)
		CHP(control_save_offset( block, settings));

	    // test the condition
	    if (settings->test_value != 0.0) // true - execute forward
		logOword("executing forward: [%s] in 'while'",
			 block->o_name);
	    else  {
		// false -  skip forward
		logOword("skipping forward: [%s] in 'while'",
			 block->o_name);
		settings->skipping_o = block->o_name;
		settings->skipping_start = settings->sequence_number;
	    }
	} else {
	    // this is the end of a 'do'
	    // test the condition
	    if ((settings->test_value != 0.0) && !settings->doing_break) {
		// true - loop on back
		logOword("looping back to: [%s] in 'do while'",
			 block->o_name);
		CHP(control_back_to(block, settings));
	    } else {
		// false
		logOword("not looping back to: [%s] in 'do while'",
			 block->o_name);
		settings->doing_break = 0;
	    }
	}
	break;

	    
    case O_if:
	if (control_find_oword(block, settings, &op) == INTERP_OK) {
	    if (settings->sequence_number != (op->sequence_number + 1)) 
	        OERR(_("%d: duplicate O-word label - already defined in line %d: '%s'"),
		    settings->sequence_number, op->sequence_number + 1,
		    settings->linetext);
	} else 
	    CHP(control_save_offset(block, settings));

	if (settings->test_value != 0.0) {
	    //true
	    logOword("executing forward: [%s] in 'if'",
		     block->o_name);
	    settings->skipping_o = NULL;
	    settings->executed_if = 1;
	} else {
	    //false
	    logOword("skipping forward: [%s] in 'if'",
		     block->o_name);
	    settings->skipping_o = block->o_name;
	    settings->skipping_start = settings->sequence_number;
	    settings->executed_if = 0;
	}
	break;

    case O_elseif:
	if (control_find_oword(block, settings, &op) != INTERP_OK) 
	    OERR(_("%d: undefined O-word label: '%s'"),
		settings->sequence_number, settings->linetext);
	if (op->type != O_if)
	    OERR(_("%d: no matching 'if' label: '%s' (found '%s' in line %d)"),
		settings->sequence_number, settings->linetext, 
		o_ops[op->type] + 2, op->sequence_number + 1);

	if ((settings->skipping_o) &&
	    (0 != strcmp(settings->skipping_o, block->o_name)))  {

	    //!!!KL -- the if conditions here say that we were skipping
	    //!!!KL but that the target o_name is not ours.
	    //!!!KL so we should continue skipping -- that's not what
	    //!!!KL the code below says.

	    // mah: so?
#if 0
	    // we were not skipping -- start skipping
	    logOword("start skipping forward: [%s] in 'elseif'",
		     block->o_name);
	    settings->skipping_o = block->o_name;
	    settings->skipping_start = settings->sequence_number;
	    return INTERP_OK;
#else
	    // we were skipping -- continue skipping
	    logOword("continue skipping forward: [%s] in 'elseif'",
		     block->o_name);
	    return INTERP_OK;
#endif
	}

	// we were skipping
	// but were we ever not skipping
	if (settings->executed_if) {
	    // we have already executed, keep on skipping
	    logOword("already executed, continue  "
		     "skipping forward: [%s] in 'elseif'",
		     block->o_name);
	    settings->skipping_o = block->o_name;
	    settings->skipping_start = settings->sequence_number;
	    return INTERP_OK;
	}

	if (settings->test_value != 0.0) {
	    //true -- start executing
	    logOword("start executing forward: [%s] in 'elseif'",
		     block->o_name);
	    settings->skipping_o = NULL;
	    settings->executed_if = 1;
	} else {
	    //false
	    logOword("continue skipping forward: [%s] in 'elseif'",
		     block->o_name);
	}
	break;

    case O_else:
	if (control_find_oword(block, settings, &op) != INTERP_OK) 
	    OERR(_("%d: undefined O-word label: '%s'"),
		settings->sequence_number, settings->linetext);
	if (op->type != O_if)
	    OERR(_("%d: no matching 'if' label: '%s' (found '%s' in line %d)"),
		settings->sequence_number, settings->linetext, 
		o_ops[op->type] + 2, op->sequence_number + 1);

	// were we ever not skipping
	if (settings->executed_if) {
	    // we have already executed, skip
	    logOword("already executed, "
		     "skipping forward: [%s] in 'else'",
		     block->o_name);
	    settings->skipping_o = block->o_name;
	    settings->skipping_start = settings->sequence_number;
	    return INTERP_OK;
	}

	if ((settings->skipping_o) &&
	    (0 == strcmp(settings->skipping_o, block->o_name))) {
	    // we were skipping so stop skipping
	    logOword("stop skipping forward: [%s] in 'else'",
		     block->o_name);
	    settings->executed_if = 1;
	    settings->skipping_o = NULL;
	} else	{
	    // we were not skipping -- so skip
	    logOword("start skipping forward: [%s] in 'else'",
		     block->o_name);
	}
	break;

    case O_endif:
	if (control_find_oword(block, settings, &op) != INTERP_OK) 
	    OERR(_("%d: undefined O-word label: '%s'"),
		settings->sequence_number, settings->linetext);
	if (op->type != O_if)
	    OERR(_("%d: no matching label: '%s' (found '%s' in line %d): '%s'"),
		settings->sequence_number, block->o_name, 
		o_ops[op->type] + 2, op->sequence_number + 1, settings->linetext);

	// stop skipping if we were
	settings->skipping_o = NULL;
	logOword("stop skipping forward: [%s] in 'endif'",
		 block->o_name);
	// the KEY -- outside if clearly must have executed
	// or this would not have executed
	settings->executed_if = 1;
	break;

    case O_break:
	if (control_find_oword(block, settings, &op) != INTERP_OK) 
	    OERR(_("%d: undefined O-word label: '%s'"),
		settings->sequence_number, settings->linetext);
	if ((op->type != O_while) && (op->type != O_do))
	    OERR(_("%d: no matching while/do label: '%s' (found '%s' in line %d)"),
		settings->sequence_number, settings->linetext, 
		o_ops[op->type] + 2, op->sequence_number + 1);

      // start skipping
      settings->skipping_o = block->o_name;
      settings->skipping_start = settings->sequence_number;
      settings->doing_break = 1;
      logOword("start skipping forward: [%s] in 'break'",
	      block->o_name);
      break;

    case O_continue:
	if (control_find_oword(block, settings, &op) != INTERP_OK) 
	    OERR(_("%d: undefined O-word label: '%s'"),
		settings->sequence_number, settings->linetext);
	if ((op->type != O_while) && (op->type != O_do))
	    OERR(_("%d: no matching while/do label: '%s' (found '%s' in line %d)"),
		settings->sequence_number, settings->linetext, 
		o_ops[op->type] + 2, op->sequence_number + 1);

	// if already skipping, do nothing
	if ((settings->skipping_o) &&
	    (0 == strcmp(settings->skipping_o, block->o_name))) {
	    logOword("already skipping: [%s] in 'continue'",
		     block->o_name);
	    return INTERP_OK;
	}
	// start skipping
	settings->skipping_o = block->o_name;
	settings->skipping_start = settings->sequence_number;
	settings->doing_continue = 1;
	logOword("start skipping forward: [%s] in 'continue'",
		 block->o_name);
	break;

    case O_endrepeat:
    case O_endwhile:
	if (control_find_oword(block, settings, &op) != INTERP_OK) 
	    OERR(_("%d: undefined O-word label: '%s'"),
		settings->sequence_number, settings->linetext);
	if (((block->o_type == O_endrepeat) && (op->type != O_repeat)) ||
	    ((block->o_type == O_endwhile) && (op->type != O_while)))
	    OERR(_("%d: no matching label: '%s' (found '%s' in line %d)"),
		settings->sequence_number, settings->linetext, 
		o_ops[op->type] + 2, op->sequence_number + 1);

	// end of a while loop
	logOword("endwhile: skipping_o:%s", settings->skipping_o);
	if ((settings->skipping_o) &&
	    (0 == strcmp(settings->skipping_o, block->o_name))) {
	    // we were skipping, so this is the end
	    settings->skipping_o = NULL;

	    if (settings->doing_continue) {
		settings->doing_continue = 0;

		// loop on back
		logOword("looping back (continue) to: [%s] in while/repeat",
			 block->o_name);
		CHP(control_back_to(block, settings));
	    } else {
		// not doing continue, we are done
		logOword("falling thru the complete while/repeat: [%s]",
			 block->o_name);
		return INTERP_OK;
	    }
	} else {
	    // loop on back
	    logOword("looping back to: [%s] in 'endwhile/endrepeat'",
		     block->o_name);
	    CHP(control_back_to(block, settings));
	}
	break;

    default:
	// FIXME !!!KL should probably be an error
	return INTERP_ERROR;
	break;
    }
    // return status;
    return INTERP_OK;
}
//========================================================================
// End of functions for control stuff (O-words)
//========================================================================
