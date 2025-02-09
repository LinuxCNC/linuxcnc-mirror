/********************************************************************
* Description: nml.cc
*   C++ file for the  Neutral Manufacturing Language (NML).
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: LGPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include "rcsversion.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>		// memcpy()
#include <stdlib.h>		// atexit()
#include <sys/param.h>		// MAXHOSTNAMELEN
#include <netdb.h>
#include <arpa/inet.h>		// inet_ntoa

#ifdef __cplusplus
}
#endif
#include <rtapi_string.h>	// rtapi_strlcpy()
#include "nml.hh"		// class NML
#include "nmlmsg.hh"		// class NMLmsg
#include "cms.hh"		// class CMS
#include "timer.hh"		// esleep()
#include "nml_srv.hh"		// NML_Default_Super_Server
#include "cms_cfg.hh"		// cms_config(), cms_copy()
#include "linklist.hh"		// class LinkedList
#include "rcs_print.hh"		// rcs_print_error()
#include "physmem.hh"
#ifndef MAXHOSTNAMELEN
#define MAXHOSTNAMELEN 64
#endif
#include "nmldiag.hh"		// NML_DIAGNOSTICS_INFO
/* Pointer to a global list of NML channels. */
LinkedList *NML_Main_Channel_List = (LinkedList *) NULL;

LinkedList *Dynamically_Allocated_NML_Objects = (LinkedList *) NULL;

int nml_print_hostname_on_error = 0;

int verbose_nml_error_messages = 1;

char NML_ERROR_TYPE_STRINGS[8][80] = {
    "NML_NO_ERROR",
    "NML_BUFFER_NOT_READ",
    "NML_TIMED_OUT",
    "NML_INVALID_CONFIGURATION",
    "NML_FORMAT_ERROR",
    "NML_INTERNAL_CMS_ERROR",
    "NML_NO_MASTER_ERROR",
    "NML_INVALID_MESSAGE_ERROR",
};

static char *default_nml_config_file = NULL;
int nml_reset_errors_printed = 1;

void set_default_nml_config_file(const char * const cfg_file)
{
    if (cfg_file == NULL) {
	default_nml_config_file = NULL;
        return;
    }

    if (default_nml_config_file) free(default_nml_config_file);
    default_nml_config_file = strndup( cfg_file, 10000 );
}

const char *get_default_nml_config_file()
{
    return default_nml_config_file;
}

/*
* NML Member Functions
*/

/* Special new operator to allow the nml_cleanup function to
distinguish between dynamically and statically allocated NML
objects. */
void *NML::operator new(size_t size)
{
    if (size < sizeof(NML)) {
	rcs_print_error
	    ("void *NML::operator new() called with size (%zd) < sizeof(NML) (%zu) the code calling NML was probably not compiled with the correct header file version.\n",
	    size, sizeof(NML));
	size = sizeof(NML);
    }
    void *nml_space = NULL;
    char *cptr = (char *) NULL;
    int dynamic_list_id = 0;
    nml_space = malloc(size + sizeof(int) * 2);
    if (NULL != nml_space) {
	memset(nml_space, 0, size);
    }
    if (NULL == Dynamically_Allocated_NML_Objects) {
	Dynamically_Allocated_NML_Objects = new LinkedList();
    }
    if (NULL != Dynamically_Allocated_NML_Objects) {
	dynamic_list_id =
	    Dynamically_Allocated_NML_Objects->store_at_tail(nml_space,
	    sizeof(NML), 0);
	cptr = ((char *) nml_space) + sizeof(NML);
	// guarantee alignment
	cptr += sizeof(int) - (((size_t) cptr) % sizeof(int));
	*((int *) cptr) = dynamic_list_id;
    }
    rcs_print_debug(PRINT_NML_CONSTRUCTORS, "%p = NML::operator new(%zd)\n",
	nml_space, size);
    return nml_space;
}

void NML::operator delete(void *nml_space)
{
    int dynamic_list_id = 0;
    char *cptr = (char *) NULL;

    rcs_print_debug(PRINT_NML_DESTRUCTORS, "NML::operator delete(%p)\n",
	nml_space);

    if (NULL == nml_space) {
	return;
    }

    if (NULL != Dynamically_Allocated_NML_Objects) {
	cptr = ((char *) nml_space) + sizeof(NML);
	cptr += sizeof(int) - (((size_t) cptr) % sizeof(int));
	dynamic_list_id = *((int *) cptr);
	Dynamically_Allocated_NML_Objects->delete_node(dynamic_list_id);
	if (Dynamically_Allocated_NML_Objects->list_size == 0) {
	    delete Dynamically_Allocated_NML_Objects;
	    Dynamically_Allocated_NML_Objects = (LinkedList *) NULL;
	}
    }
    free(nml_space);
}

/******************************************************************
* Constructor for NML:
* Parameters:
* NML_FORMAT_PTR f_ptr - address of the function to be used to format messages.
* char *buf - Name of the buffer to connect to as written in the config file.
* char *proc - Name of the calling process as expected in the config file.
* char *file - Name of the configuration file.
* int set_to_server - If 1 this NML will consider its calling process to
*   be an NML_SERVER, which effects how and when messages are encoded and
*   decoded.
* int set_to_master - Passed to the CMS constructor - how this is used
*   depends on the type of CMS buffer. In general the master is responsible
*   for creating/initializing the buffer. If set_to_master == 1 then this
*   process will be considered the master, if set_to_master == 0 then
*   the the configuration file determines if this is the master, and it
*   set_to_master == -1 then this will not be the master.
* NOTES:
*  1. Borland C++(for DOS and Windows) does not allow default
*   parameters to be specified both here and in the header file.
*  2. All pointers are first initialized to NULL so that it can be determined
*  later if the constructor returned before creating the objects
*  the pointers are intended to point at.
******************************************************************/
NML::NML(NML_FORMAT_PTR f_ptr, const char *buf, const char *proc, const char *file,
    const int set_to_server, const int set_to_master)
{
    registered_with_server = 0;
    cms_for_msg_string_conversions = 0;
    info_printed = 0;
    blocking_read_poll_interval = -1.0;
    forced_type = 0;

    snprintf(bufname, 40, "%s", buf);
    snprintf(procname, 40, "%s", proc);
    if (NULL == file) {
	rtapi_strlcpy(cfgfilename, default_nml_config_file, 160);
    }
    else {
	rtapi_strlcpy(cfgfilename, file, 160);
    }
    snprintf(cfgfilename, 160, "%s", file);

    if (rcs_errors_printed >= max_rcs_errors_to_print
	&& max_rcs_errors_to_print > 0 && nml_reset_errors_printed) {
	rcs_errors_printed = 0;
	rcs_print
	    ("\nResetting rcs_errors_printed because a new NML channel is being created.\n");
    }

    already_deleted = 0;
    channel_type = NML_GENERIC_CHANNEL_TYPE;

    reconstruct(f_ptr, buf, proc, cfgfilename, set_to_server, set_to_master);

    if (NULL != cms) {
	char *forced_type_eq = strstr(cms->buflineupper, "FORCE_TYPE=");
	if (forced_type_eq != NULL) {
	    long temp = 0;
	    temp = strtol(forced_type_eq + 11, NULL, 0);
	    if (temp > 0) {
		forced_type = temp;
		fast_mode = 0;
	    }
	}
    }
}

int NML::login(const char *name, const char *passwd)
{
    if (NULL == cms) {
	return 1;
    }
    return cms->login(name, passwd);
}

void NML::reconstruct(NML_FORMAT_PTR f_ptr, const char *buf, const char *proc,
    const char *file, const int set_to_server, const int set_to_master)
{

    cms = (CMS *) NULL;
    format_chain = (LinkedList *) NULL;
    phantom_read = (NMLTYPE(*)())NULL;
    phantom_peek = (NMLTYPE(*)())NULL;
    phantom_write = (int (*)(NMLmsg *, int *)) NULL;
    phantom_write_if_read = (int (*)(NMLmsg *, int *)) NULL;
    phantom_check_if_read = (int (*)()) NULL;
    phantom_clear = (int (*)()) NULL;
    channel_list_id = 0;
    error_type = NML_NO_ERROR;
    fast_mode = 0;
    ignore_format_chain = 0;
    info_printed = 0;

    format_chain = new LinkedList;
    if (NULL != format_chain) {
	prefix_format_chain(f_ptr);
    }

    if (NULL == f_ptr) {
	rcs_print_error("NML:(Format Function Pointer) f_ptr == NULL.\n");
    }

    if (-1 == cms_config(&cms, buf, proc, file, set_to_server, set_to_master)) {
	set_error();
	if (!info_printed) {
	    print_info(buf, proc, file);
	}
	if (NULL != cms) {
	    rcs_print_debug(PRINT_NML_DESTRUCTORS, " delete (CMS *) %p;\n", cms);
	    delete cms;
	    cms = (CMS *) NULL;
	}
	return;
    }

    if (NULL == cms) {
	if (!info_printed) {
	    print_info(buf, proc, file);
	}
	error_type = NML_INVALID_CONFIGURATION;
	return;
    }

    if (cms->status < 0) {
	error_type = NML_INVALID_CONFIGURATION;
	if (!info_printed) {
	    print_info(buf, proc, file);
	}
	rcs_print_debug(PRINT_NML_DESTRUCTORS, " delete (CMS *) %p;\n", cms);
	delete cms;
	cms = (CMS *) NULL;
	return;
    }
    if (!set_to_server) {
	register_with_server();
    }
    add_to_channel_list();
    // FAST MODE is a combination of options which allow certain checks
    // during
    // a read or write operation to be avoided and therefore reduce the
    // NML/CMS
    // overhead.
    if (!cms->is_phantom &&
	cms->ProcessType == CMS_LOCAL_TYPE &&
	!cms->neutral && !cms->isserver && !cms->enable_diagnostics) {
	fast_mode = 1;
    }

    cms_status = (int *) &(cms->status);
    cms_inbuffer_header_size = &(cms->header.in_buffer_size);
    char *forced_type_eq = strstr(cms->buflineupper, "FORCE_TYPE=");
    if (forced_type_eq != NULL) {
	long temp = 0;
        temp = strtol(forced_type_eq + 11, NULL, 0);
	if (temp > 0) {
	    forced_type = temp;
	    fast_mode = 0;
	}
    }
    char *brpi_eq = strstr(cms->buflineupper, "BRPI=");
    if (brpi_eq != NULL) {
	blocking_read_poll_interval = strtod(brpi_eq + 5, NULL);
    }

}

/******************************************************************
* Constructor for NML:
* Parameters:
* char *buf - Name of the buffer to connect to as written in the config file.
* char *proc - Name of the calling process as expected in the config file.
* char *file - Name of the configuration file.
* int set_to_server - If 1 this NML will consider its calling process to
*   be an NML_SERVER, which effects how and when messages are encoded and
*   decoded.
* int set_to_master - Passed to the CMS constructor - how this is used
*   depends on the type of CMS buffer. In general the master is responsible
*   for creating/initializing the buffer. If set_to_master == 1 then this
*   process will be considered the master, if set_to_master == 0 then
*   the the configuration file determines if this is the master, and it
*   set_to_master == -1 then this will not be the master.
* NOTES:
*  1. Borland C++(for DOS and Windows) does not allow default
*   parameters to be specified both here and in the header file.
*  2. All pointers are first initialized to NULL so that it can be determined
*  later if the constructor returned before creating the objects
*  the pointers are intended to point at.
*  3. This constructor does not register itself with the default server.
*  4. The NML object created by this constructor can not be used until
* the format_chain is constructed. (This may be done by
* derived classes. )
******************************************************************/
NML::NML(const char *buf, const char *proc, const char *file, const int set_to_server,
    const int set_to_master)
{
    if (NULL == file) {
	file = default_nml_config_file;
    }
    registered_with_server = 0;
    cms_for_msg_string_conversions = 0;
    snprintf(bufname, 40 , "%s", buf);
    snprintf(procname, 40, "%s", proc);
    snprintf(cfgfilename, 160, "%s", file);
    blocking_read_poll_interval = -1.0;
    info_printed = 0;
    forced_type = 0;
    already_deleted = 0;
    cms = (CMS *) NULL;
    format_chain = (LinkedList *) NULL;
    phantom_read = (NMLTYPE(*)())NULL;
    phantom_peek = (NMLTYPE(*)())NULL;
    phantom_write = (int (*)(NMLmsg *, int *)) NULL;
    phantom_write_if_read = (int (*)(NMLmsg *, int *)) NULL;
    phantom_check_if_read = (int (*)()) NULL;
    phantom_clear = (int (*)()) NULL;
    channel_list_id = 0;
    error_type = NML_NO_ERROR;
    ignore_format_chain = 0;
    fast_mode = 0;

    channel_type = NML_GENERIC_CHANNEL_TYPE;

    if (-1 == cms_config(&cms, buf, proc, file, set_to_server, set_to_master)) {
	if (verbose_nml_error_messages) {
	    rcs_print_error("NML: cms_config returned -1.\n");
	}
	if (!info_printed) {
	    print_info(buf, proc, file);
	}
	if (NULL != cms) {
	    rcs_print_debug(PRINT_NML_DESTRUCTORS, " delete (CMS *) %p;\n",
		cms);
	    delete cms;
	    cms = (CMS *) NULL;
	}
	error_type = NML_INVALID_CONFIGURATION;
	return;
    }

    if (NULL == cms) {
	error_type = NML_INVALID_CONFIGURATION;
	if (!info_printed) {
	    print_info(buf, proc, file);
	}
	return;
    }

    if (cms->status < 0) {
	error_type = NML_INVALID_CONFIGURATION;
	if (!info_printed) {
	    print_info(buf, proc, file);
	}
	rcs_print_debug(PRINT_NML_DESTRUCTORS, " delete (CMS *) %p;\n", cms);
	delete cms;
	cms = (CMS *) NULL;
	return;
    }

    add_to_channel_list();
    if (!cms->is_phantom &&
	cms->ProcessType == CMS_LOCAL_TYPE && !cms->neutral && !cms->isserver)
    {
	fast_mode = 1;
    }
    cms_status = (int *) &(cms->status);
    cms_inbuffer_header_size = &(cms->header.in_buffer_size);

    if (NULL != cms) {
	char *forced_type_eq = strstr(cms->buflineupper, "FORCE_TYPE=");
	if (forced_type_eq != NULL) {
	    long temp = 0;
	    temp = strtol(forced_type_eq + 11, NULL, 0);
	    if (temp > 0) {
		forced_type = temp;
		fast_mode = 0;
	    }
	}
	char *brpi_eq = strstr(cms->buflineupper, "BRPI=");
	if (brpi_eq != NULL) {
	    blocking_read_poll_interval = strtod(brpi_eq + 5, NULL);
	}
    }

}

/******************************************************************
* Constructor for NML:
* Parameters:
* char *buffer_line - Buffer line  as written in the config file.
* char *proc_line - Process Line as expected in the config file.
* char *file - Name of the configuration file.
* int set_to_server - If 1 this NML will consider its calling process to
*   be an NML_SERVER, which effects how and when messages are encoded and
*   decoded.
* int set_to_master - Passed to the CMS constructor - how this is used
*   depends on the type of CMS buffer. In general the master is responsible
*   for creating/initializing the buffer. If set_to_master == 1 then this
*   process will be considered the master, if set_to_master == 0 then
*   the the configuration file determines if this is the master, and it
*   set_to_master == -1 then this will not be the master.
* NOTES:
*  1. Borland C++(for DOS and Windows) does not allow default
*   parameters to be specified both here and in the header file.
*  2. All pointers are first initialized to NULL so that it can be determined
*  later if the constructor returned before creating the objects
*  the pointers are intended to point at.
*  3. This constructor does not register itself with the default server.
*  4. The NML object created by this constructor can not be used until
* the format_chain is constructed. (This may be done by
* derived classes. )
******************************************************************/
NML::NML(const char * buffer_line, const char * proc_line)
{
    registered_with_server = 0;
    cms_for_msg_string_conversions = 0;
    cms = (CMS *) NULL;
    blocking_read_poll_interval = -1.0;
    forced_type = 0;
    info_printed = 0;
    already_deleted = 0;
    format_chain = (LinkedList *) NULL;
    phantom_read = (NMLTYPE(*)())NULL;
    phantom_peek = (NMLTYPE(*)())NULL;
    phantom_write = (int (*)(NMLmsg *, int *)) NULL;
    phantom_write_if_read = (int (*)(NMLmsg *, int *)) NULL;
    phantom_check_if_read = (int (*)()) NULL;
    phantom_clear = (int (*)()) NULL;
    channel_list_id = 0;
    error_type = NML_NO_ERROR;
    ignore_format_chain = 0;
    fast_mode = 0;

    channel_type = NML_GENERIC_CHANNEL_TYPE;

    if (-1 == cms_create_from_lines(&cms, buffer_line, proc_line)) {
	if (verbose_nml_error_messages) {
	    rcs_print_error("NML: cms_create_from_lines returned -1.\n");
	}
	if (!info_printed) {
	    print_info();
	}
	if (NULL != cms) {
	    rcs_print_debug(PRINT_NML_DESTRUCTORS, " delete (CMS *) %p;\n",
		cms);
	    delete cms;
	    cms = (CMS *) NULL;
	}
	error_type = NML_INVALID_CONFIGURATION;
	return;
    }
    if (NULL == cms) {
	error_type = NML_INVALID_CONFIGURATION;
	return;
    }
    if (cms->status < 0) {
	error_type = NML_INVALID_CONFIGURATION;
	if (verbose_nml_error_messages) {
	    rcs_print_error("NML: cms->status = %d.\n", cms->status);
	}
	if (!info_printed) {
	    print_info();
	}
	rcs_print_debug(PRINT_NML_DESTRUCTORS, " delete (CMS *) %p;\n", cms);
	delete cms;
	cms = (CMS *) NULL;
	return;
    }
    add_to_channel_list();
    if (!cms->is_phantom &&
	cms->ProcessType == CMS_LOCAL_TYPE && !cms->neutral && !cms->isserver)
    {
	fast_mode = 1;
    }
    cms_status = (int *) &(cms->status);
    cms_inbuffer_header_size = &(cms->header.in_buffer_size);
    if (NULL != cms) {
	char *forced_type_eq = strstr(cms->buflineupper, "FORCE_TYPE=");
	if (forced_type_eq != NULL) {
	    long temp = 0;
	    temp = strtol(forced_type_eq + 11, NULL, 0);
	    if (temp > 0) {
		forced_type = temp;
		fast_mode = 0;
	    }
	}
	char *brpi_eq = strstr(cms->buflineupper, "BRPI=");
	if (brpi_eq != NULL) {
	    blocking_read_poll_interval = strtod(brpi_eq + 5, NULL);
	}
	register_with_server();
    }

}

/***************************************************************
* NML Member Function: add_to_channel_list()
* Purpose:
*  Adds a pointer to this NML object to the main NML list.
* The list is used by nml_cleanup to delete all NML objects owned by
* a process.
* Under VxWorks the list must be shared by all processes so we must
* remember which process (also known as a task) added it to the list.
* (This is a great reason for tossing VxWorks in the circular file!)
*****************************************************************/
void NML::add_to_channel_list()
{
    if (NULL == NML_Main_Channel_List) {
	NML_Main_Channel_List = new LinkedList;
    }
    if (NULL != NML_Main_Channel_List) {
	channel_list_id =
	    NML_Main_Channel_List->store_at_tail(this, sizeof(NML), 0);
    }
}

/********************************************************************
* NML Member Function: register_with_server()
* Purpose:
*  The NML_Default_Super_Server keeps a list of all of the NML objects
* that were specified as servers in the config file.
* When nml_start is called servers will be spawned for these buffers.
* The NML_Default_Super_Server also tries to reduce the number of processes
* spawned by grouping buffers with the same buffer number.
********************************************************************/
void NML::register_with_server()
{
    if (NULL != cms && !registered_with_server) {
	if (cms->spawn_server) {
	    if (NULL == NML_Default_Super_Server) {
		NML_Default_Super_Server = new NML_SUPER_SERVER;
	    }
	    NML_Default_Super_Server->add_to_list(this);
	    registered_with_server = 1;
	}
    }
}

/**************************************************************
* NML Constructor:
* Parameters:
* NML *nml; - pointer to NML object to duplicate.
* int set_to_server - If 1 this NML will consider its calling process to
*   be an NML_SERVER, which effects how and when messages are encoded and
*   decoded.
* int set_to_master - Passed to the CMS constructor - how this is used
*   depends on the type of CMS buffer. In general the master is responsible
*   for creating/initializing the buffer. If set_to_master == 1 then this
*   process will be considered the master, if set_to_master == 0 then
*   the the configuration file determines if this is the master, and it
*   set_to_master == -1 then this will not be the master.
* NOTES:
*  1. Borland C++(for DOS and Windows) does not allow default
*   parameters to be specified both here and in the header file.
*  2. All pointers are first initialized to NULL so that it can be determined
*  later if the constructor returned before creating the objects
*  the pointers are intended to point at.
*************************************************************/
NML::NML(NML * nml_ptr, const int set_to_server, const int set_to_master)
{
    registered_with_server = 0;
    cms_for_msg_string_conversions = 0;
    already_deleted = 0;
    forced_type = 0;
    cms = (CMS *) NULL;
    format_chain = (LinkedList *) NULL;
    error_type = NML_NO_ERROR;
    ignore_format_chain = 0;
    channel_list_id = 0;
    fast_mode = 0;
    info_printed = 0;
    blocking_read_poll_interval = -1.0;

    channel_type = NML_GENERIC_CHANNEL_TYPE;

    if (NULL != nml_ptr) {
	strncpy(bufname, nml_ptr->bufname, 40);
	strncpy(procname, nml_ptr->procname, 40);
	strncpy(cfgfilename, nml_ptr->cfgfilename, 160);
	if (NULL != nml_ptr->cms) {
	    // Create a CMS channel identitical to the one from the argument
	    // NML channel accept that the channel may be set_to_server or
	    // set_to_master differently.
	    cms_copy(&cms, nml_ptr->cms, set_to_server, set_to_master);
	    if (NULL != cms) {
		cms->current_subdivision = nml_ptr->cms->current_subdivision;
	    }
	}
    }
    if (!ignore_format_chain) {
	format_chain = new LinkedList;
	if ((NULL != nml_ptr->format_chain) && (NULL != format_chain)) {
	    LinkedList *from, *to;
	    NML_FORMAT_PTR format_func_ptr;
	    from = nml_ptr->format_chain;
	    to = format_chain;
	    format_func_ptr = (NML_FORMAT_PTR) from->get_head();
	    while (NULL != format_func_ptr) {
		to->store_at_tail((void *) format_func_ptr, 0, 0);
		format_func_ptr = (NML_FORMAT_PTR) from->get_next();
	    }
	}
    }
    if (NULL == cms) {
	return;
    }
    add_to_channel_list();
    if (!cms->is_phantom &&
	cms->ProcessType == CMS_LOCAL_TYPE && !cms->neutral && !cms->isserver)
    {
	fast_mode = 1;
    }
    cms_status = (int *) &(cms->status);
    cms_inbuffer_header_size = &(cms->header.in_buffer_size);
    char *forced_type_eq = strstr(cms->buflineupper, "FORCE_TYPE=");
    if (forced_type_eq != NULL) {
	long temp = 0;
	temp = strtol(forced_type_eq + 11, NULL, 0);
	if (temp > 0) {
	    forced_type = temp;
	    fast_mode = 0;
	}
    }
    char *brpi_eq = strstr(cms->buflineupper, "BRPI=");
    if (brpi_eq != NULL) {
	blocking_read_poll_interval = strtod(brpi_eq + 5, NULL);
    }
    if (NULL != nml_ptr->cms->dpi) {
	CMS_DIAG_PROC_INFO *dpi = cms->get_diag_proc_info();
	*dpi = *(nml_ptr->cms->get_diag_proc_info());
	cms->set_diag_proc_info(dpi);
    }
    cms->first_diag_store = nml_ptr->cms->first_diag_store;
    if (NULL != cms->handle_to_global_data &&
	NULL != nml_ptr->cms->handle_to_global_data) {
	cms->handle_to_global_data->total_bytes_moved =
	    nml_ptr->cms->handle_to_global_data->total_bytes_moved;
    }
}

/**************************************************************
* NML Reset Function
* Can be used instead of deleting and recreating an NML channel.
*************************************************************/
int NML::reset()
{
    int cms_copy_ret = 0;
    if (valid()) {
	return 1;
    }
    if (NULL != cms) {
	// Recreate a CMS channel identitical to the old one, do not
	// re-read the config file.
	CMS *oldcms = cms;
	cms = NULL;
	cms_copy_ret = cms_copy(&cms, oldcms, 0, 0);
	if (cms_copy_ret < 0) {
	    if (cms != NULL && cms != oldcms) {
		delete oldcms;
	    }
	    return 0;
	}
	register_with_server();
	add_to_channel_list();
	// FAST MODE is a combination of options which allow certain checks
	// during
	// a read or write operation to be avoided and therefore reduce the
	// NML/CMS
	// overhead.
	if (!cms->is_phantom &&
	    cms->ProcessType == CMS_LOCAL_TYPE &&
	    !cms->neutral && !cms->isserver && !cms->enable_diagnostics) {
	    fast_mode = 1;
	}

	cms_status = (int *) &(cms->status);
	cms_inbuffer_header_size = &(cms->header.in_buffer_size);
	char *forced_type_eq = strstr(cms->buflineupper, "FORCE_TYPE=");
	if (forced_type_eq != NULL) {
	    long temp = strtol(forced_type_eq + 11, NULL, 0);
	    if (temp > 0) {
		forced_type = temp;
		fast_mode = 0;
	    }
	}
	char *brpi_eq = strstr(cms->buflineupper, "BRPI=");
	if (brpi_eq != NULL) {
	    blocking_read_poll_interval = strtod(brpi_eq + 5, NULL);
	}

	delete oldcms;
    } else {
	// Re-read the config file before creating a new CMS object.
	if (cms_config(&cms, bufname, procname, cfgfilename, 0, 0) < 0) {
	    return 0;
	}
    }
    return valid();
}

/*********************************************************
* NML Destructor:
* Notes:
*  1. Use if(NULL != ???) to avoid deleting objects that were
* never constructed.
*  2. The delete channel function was added because an error occurred in
* running a server unded WIN32. An exception would occur as
* the last NML channel was being deleted from within nml_cleanup.
* After two days of being unable to debug this problem I
* replaced the delete nml with nml->delete_channel() as a workaround.
* The cause of this exception is still not understood.
*********************************************************/
NML::~NML()
{
    if (already_deleted) {
	if (verbose_nml_error_messages) {
	    rcs_print_error("NML channel being deleted more than once.\n");
	}
    }
    already_deleted = 1;
    delete_channel();
}

void NML::delete_channel()
{
    rcs_print_debug(PRINT_NML_DESTRUCTORS, "deleting NML (%d)\n",
	channel_list_id);
    if (NULL != cms_for_msg_string_conversions
	&& cms != cms_for_msg_string_conversions) {
	delete cms_for_msg_string_conversions;
	cms_for_msg_string_conversions = 0;
    }
    if (NULL != cms) {
	rcs_print_debug(PRINT_NML_DESTRUCTORS, " delete (CMS *) %p;\n", cms);
	delete cms;
	cms = (CMS *) NULL;
    }
    if (NULL != format_chain) {
	delete format_chain;
	format_chain = (LinkedList *) NULL;
    }
    if (NULL != NML_Main_Channel_List && (0 != channel_list_id)) {
	NML_Main_Channel_List->delete_node(channel_list_id);
    }
    rcs_print_debug(PRINT_NML_DESTRUCTORS, "Leaving ~NML()\n");
}

/*************************************************************
* NML Member Function: get_address()
* Purpose:
*  Returns the address of the local copy of the buffer.
*  Use this function instead of nml->cms->subdiv_data directly to prevent
*   users from pointing nml->cms->subdiv_data at something else. Or getting
*   a segmentation fault if cms was not properly constructed.
***************************************************************/
NMLmsg *NML::get_address()
{
    if (NULL == cms) {
	if (NULL != cms_for_msg_string_conversions) {
	    return ((NMLmsg *) cms_for_msg_string_conversions->subdiv_data);
	}
	error_type = NML_INVALID_CONFIGURATION;
	return ((NMLmsg *) NULL);
    } else {
	return ((NMLmsg *) cms->subdiv_data);
    }
}

/*************************************************************
* NML Member Function: get_address_subdivision(int subdiv)
* Purpose:
*  Returns the address of the local copy of the buffer.
*  Use this function instead of nml->cms->subdiv_data directly to prevent
*   users from pointing nml->cms->subdiv_data at something else. Or getting
*   a segmentation fault if cms was not properly constructed.
***************************************************************/
NMLmsg *NML::get_address_subdivision(int subdiv)
{
    if (NULL == cms) {
	error_type = NML_INVALID_CONFIGURATION;
	return ((NMLmsg *) NULL);
    } else {
	cms->set_subdivision(subdiv);
	return ((NMLmsg *) cms->subdiv_data);
    }
}

/*************************************************************
* NML Member Function: get_address_subdivision(int subdiv)
* Purpose:
*  Returns the total number of subdivisions configured for this
* NML channel.
***************************************************************/
int NML::get_total_subdivisions()
{
    if (NULL == cms) {
	return (1);
    } else {
	return (cms->total_subdivisions);
    }
}

/***********************************************************
* NML Member Function: clear()
* Purpose:
*  Clears the CMS buffer associated with this NML channel.
*
*  Returns:
* 0 if no error occurred or -1 if error occurred.
*
* Notes:
*  1. Some buffers can be identified as PHANTOM in the config file.
* this will cause cms->is_phantom to be set. This will cause this
* function to call the function pointed to by phantom_clear
* if it is not NULL rather than using CMS.
*  2. If an error occurs, users can check error_type before
* making any other NML calls
************************************************************/
int NML::clear()
{
    if (NULL == cms) {
	error_type = NML_INVALID_CONFIGURATION;
	return (-1);
    } else {
	if (cms->is_phantom) {
	    if (NULL != phantom_clear) {
		return ((*phantom_clear) ());
	    } else {
		return (0);
	    }
	}
	CMS_STATUS return_value;
	error_type = NML_NO_ERROR;
	if (((int) (return_value = cms->clear())) > 0) {
	    error_type = NML_INTERNAL_CMS_ERROR;
	}
	if (cms->status == CMS_TIMED_OUT) {
	    error_type = NML_TIMED_OUT;
	}
	return (((int) return_value < 0) ? -1 : 0);
    }
}

/************************************************************
* NML Member Function: clean_buffers()
*  Tells cms to set buffers to zero so that we eliminate strange
* history effects. -- Should only be used by programs testing CMS/NML.
************************************************************/
void NML::clean_buffers()
{
    if (NULL != cms) {
	cms->clean_buffers();
    }
}

/***********************************************************
* NML Member Function: check_if_read()
* Purpose:
*  Returns 1 if the buffer has been read since the last write,
* 0 if it hadn't, or -1 if some communications error prevented
* it from finding out.
* Notes:
*  1. Some buffers can be identified as PHANTOM in the config file.
* this will cause cms->is_phantom to be set. This will cause this
* function to call the function pointed to by phantom_check_if_read
* if it is not NULL rather than using CMS.
*  2. If an error occurs, users can check error_type before
* making any other NML calls
************************************************************/
int NML::check_if_read()
{
    if (NULL == cms) {
	error_type = NML_INVALID_CONFIGURATION;
	return (-1);
    } else {
	if (cms->is_phantom) {
	    if (NULL != phantom_check_if_read) {
		return ((*phantom_check_if_read) ());
	    } else {
		return (0);
	    }
	}
	int return_value;
	error_type = NML_NO_ERROR;
	if ((return_value = cms->check_if_read()) == -1) {
	    error_type = NML_INTERNAL_CMS_ERROR;
	}
	if (cms->status == CMS_TIMED_OUT) {
	    error_type = NML_TIMED_OUT;
	}
	return (return_value);
    }
}

/***********************************************************
* NML Member Function: get_queue_length()
* Purpose:
*  Returns the number of messages queued in the buffer if queueing
* was enabled for this buffer. 0 otherwise.
************************************************************/
int NML::get_queue_length()
{
    if (NULL == cms) {
	error_type = NML_INVALID_CONFIGURATION;
	return (-1);
    } else {
	error_type = NML_NO_ERROR;
	return cms->get_queue_length();
    }
}

/***********************************************************
* NML Member Function: get_space_available()
* Purpose:
*  Returns the approximate number of bytes that can be written
* to a queued buffer.
*
************************************************************/
int NML::get_space_available()
{
    if (NULL == cms) {
	error_type = NML_INVALID_CONFIGURATION;
	return (-1);
    } else {
	error_type = NML_NO_ERROR;
	return cms->get_space_available();
    }
}

/*************************************************************
* NML Member Function: valid()
* Purpose:
* Provides a check which can be used after an NML object is
* constructed to determine if everything is in order.
* Returns: 1 if everything is O.K. 0 otherwise.
*************************************************************/
int NML::valid()
{
    if (NULL == cms) {
	error_type = NML_INVALID_CONFIGURATION;
	return (0);
    }

    if (cms->is_phantom) {
	error_type = NML_NO_ERROR;
	return (1);
    }

    if (CMS_MISC_ERROR == cms->status) {
	error_type = NML_INTERNAL_CMS_ERROR;
	return (0);
    }

    if (CMS_NO_MASTER_ERROR == cms->status) {
	error_type = NML_NO_MASTER_ERROR;
	return (0);
    }

    if (NULL == cms->data) {
	error_type = NML_INVALID_CONFIGURATION;
	return (0);
    }

    if (cms->neutral && (NULL == cms->encoded_data) && !cms->isserver) {
	error_type = NML_INVALID_CONFIGURATION;
	return (0);
    }

    if (!ignore_format_chain) {
	if (NULL == format_chain) {
	    error_type = NML_INVALID_CONFIGURATION;
	    return (0);
	}
    }

    error_type = NML_NO_ERROR;
    return (1);
}

/***********************************************************
* NML Member Function: read()
* Purpose: Reads an NMLmsg from a CMS buffer.
* Returns:
*  0 The read was successful but the data was not updated since the last read.
*  -1 The buffer could not be read.
*  o.w. The type of the new NMLmsg is returned.
* Notes:
*   1. Users need to call NML::get_address in order to access the
* messages that were read.
*  2. Some buffers can be identified as PHANTOM in the config file.
* this will cause cms->is_phantom to be set. This will cause this
* function to call the function pointed to by phantom_read
* if it is not NULL rather than using CMS.
*  3. If an error occurs, users can check error_type before
* making any other NML calls
***********************************************************/
NMLTYPE NML::read()
{
    error_type = NML_NO_ERROR;
    if (fast_mode) {
	cms->read();
	switch (cms->status) {
	case CMS_READ_OLD:
	    return (0);
	case CMS_READ_OK:
	    if (((NMLmsg *) cms->subdiv_data)->type <= 0 && !cms->isserver) {
		rcs_print_error
		    ("NML: New data received but type of %d is invalid.\n",
		    (int)((NMLmsg *) cms->subdiv_data)->type);
		return -1;
	    }
	    return (((NMLmsg *) cms->subdiv_data)->type);

	default:
	    set_error();
	    return -1;
	}
    }
    /* Check pointers. */
    if (NULL == cms) {
	if (error_type != NML_INVALID_CONFIGURATION) {
	    error_type = NML_INVALID_CONFIGURATION;
	    rcs_print_error("NML::read: CMS not configured.\n");
	}
	return (-1);
    }

    /* Handle PHANTOMs */
    if (cms->is_phantom) {
	if (NULL != phantom_read) {
	    return ((*phantom_read) ());
	} else {
	    return (0);
	}
    }

    /* Read using CMS */
    if (!cms->force_raw) {
	cms->set_mode(CMS_READ);
    }
    cms->read();

    if (!cms->force_raw) {
	if (cms->status == CMS_READ_OK) {
	    if (-1 == format_output()) {
		error_type = NML_FORMAT_ERROR;
		return (-1);
	    }
	}
    }

    /* Choose the return value. */
    switch (cms->status) {
    case CMS_READ_OLD:
	error_type = NML_NO_ERROR;
	return (0);
    case CMS_READ_OK:
	error_type = NML_NO_ERROR;
	if (((NMLmsg *) cms->subdiv_data)->type <= 0 && !cms->isserver) {
	    rcs_print_error
		("NML: New data received but type of %d is invalid.\n",
		(int)((NMLmsg *) cms->subdiv_data)->type);
	    return -1;
	}
	return (((NMLmsg *) cms->subdiv_data)->type);

    default:
	set_error();
	return -1;
    }
}

/***********************************************************
* NML Member Function: blocking_read(double timeout)
* Purpose: Reads an NMLmsg from a CMS buffer after waiting up to timeout seconds for new data.
* Returns:
*  0 The read was successful but the data was not updated since the last read.
*  -1 The buffer could not be read.
*  o.w. The type of the new NMLmsg is returned.
* Notes:
*   1. Users need to call NML::get_address in order to access the
* messages that were read.
*  2. Some buffers can be identified as PHANTOM in the config file.
* this will cause cms->is_phantom to be set. This will cause this
* function to call the function pointed to by phantom_read
* if it is not NULL rather than using CMS.
*  3. If an error occurs, users can check error_type before
* making any other NML calls
***********************************************************/
NMLTYPE NML::blocking_read(double blocking_timeout)
{
    error_type = NML_NO_ERROR;
    if (fast_mode) {
	cms->blocking_read(blocking_timeout);
	switch (cms->status) {
	case CMS_READ_OLD:
	    return (0);
	case CMS_READ_OK:
	    if (((NMLmsg *) cms->subdiv_data)->type <= 0 && !cms->isserver) {
		rcs_print_error
		    ("NML: New data received but type of %d is invalid.\n",
		    (int)((NMLmsg *) cms->subdiv_data)->type);
		return -1;
	    }
	    return (((NMLmsg *) cms->subdiv_data)->type);
	case CMS_TIMED_OUT:
	    error_type = NML_NO_ERROR;
	    return 0;

	default:
	    set_error();
	    return (-1);
	}
    }
    /* Check pointers. */
    if (NULL == cms) {
	if (error_type != NML_INVALID_CONFIGURATION) {
	    error_type = NML_INVALID_CONFIGURATION;
	    rcs_print_error("NML::blocking_read: CMS not configured.\n");
	}
	return (-1);
    }

    /* Handle PHANTOMs */
    if (cms->is_phantom) {
	if (NULL != phantom_read) {
	    return ((*phantom_read) ());
	} else {
	    return (0);
	}
    }

    /* Read using CMS */
    if (!cms->force_raw) {
	cms->set_mode(CMS_READ);
    }
    if (cms->BufferType == CMS_SHMEM_TYPE) {
	cms->blocking_read(blocking_timeout);
    } else {
	double time_elapsed = 0.0;
	double start_time = 0.0;
	if (blocking_timeout > 0.0) {
	    start_time = etime();
	}
	double current_brpi = blocking_read_poll_interval;
	cms->status = CMS_READ_OLD;
	if (current_brpi < 2e-2) {
	    current_brpi = 2e-2;
	}
	if (current_brpi > blocking_timeout / 2.0 && blocking_timeout > 1e-6) {
	    current_brpi = blocking_timeout / 2.0;
	}
	while (cms->status == CMS_READ_OLD &&
	    (time_elapsed < blocking_timeout || blocking_timeout < 0.0)) {

	    esleep(current_brpi);
	    cms->read();
	    if (blocking_timeout > 0.0 && cms->status == CMS_READ_OLD) {
		time_elapsed = etime() - start_time;
	    }
	    if (time_elapsed < 0.0) {
		break;
	    }
	}
    }

    if (!cms->force_raw) {
	if (cms->status == CMS_READ_OK) {
	    if (-1 == format_output()) {
		error_type = NML_FORMAT_ERROR;
		return (-1);
	    }
	}
    }

    /* Choose the return value. */
    switch (cms->status) {
    case CMS_READ_OLD:
	return (0);
    case CMS_READ_OK:
	if (((NMLmsg *) cms->subdiv_data)->type <= 0 && !cms->isserver) {
	    rcs_print_error
		("NML: New data received but type of %d is invalid.\n",
		(int)((NMLmsg *) cms->subdiv_data)->type);
	    return -1;
	}
	return (((NMLmsg *) cms->subdiv_data)->type);
    case CMS_TIMED_OUT:
	error_type = NML_NO_ERROR;
	return 0;

    default:
	set_error();
	return (-1);
    }

}

void NML::reconnect()
{
    if (NULL != cms) {
	cms->reconnect();
    }
}

void NML::disconnect()
{
    if (NULL != cms) {
	cms->disconnect();
    }
}

/* Same as the read with no arguments except that the data is
 stored in a user supplied location . */
NMLTYPE NML::read(void *temp_data, long temp_size)
{
    NMLTYPE return_value = 0;
    void *original_data;
    long original_size = cms->size;
    long original_max_message_size = cms->max_message_size;
    original_data = cms->data;
    cms->data = temp_data;
    cms->size = temp_size;
    if (cms->max_message_size > ((long) temp_size)) {
	cms->max_message_size = temp_size;
    }
    return_value = peek();
    cms->data = original_data;
    cms->size = original_size;
    cms->max_message_size = original_max_message_size;
    return return_value;
}

/* Same as the peek with no arguments except that the data is
 stored in a user supplied location . */
NMLTYPE NML::peek(void *temp_data, long temp_size)
{
    NMLTYPE return_value = 0;
    void *original_data;
    long original_size = cms->size;
    long original_max_message_size = cms->max_message_size;
    original_data = cms->data;
    cms->data = temp_data;
    cms->size = temp_size;
    if (cms->max_message_size > ((long) temp_size)) {
	cms->max_message_size = temp_size;
    }
    return_value = peek();
    cms->data = original_data;
    cms->size = original_size;
    cms->max_message_size = original_max_message_size;
    return return_value;
}

/***********************************************************
* NML Member Function: peek()
* Purpose: Reads an NMLmsg from a CMS buffer without setting the
* was_read flag. The was_read flag is used by check_if_read and
* write_if_read.
* Returns:
*  0 The read was successful but the data was not updated since the last read.
*  -1 The buffer could not be read.
*  o.w. The type of the new NMLmsg is returned.
* Notes:
*   1. Users need to call NML::get_address in order to access the
* messages that were read.
*  2. Some buffers can be identified as PHANTOM in the config file.
* this will cause cms->is_phantom to be set. This will cause this
* function to call the function pointed to by phantom_read
* if it is not NULL rather than using CMS.
*  3. If an error occurs, users can check error_type before
* making any other NML calls
***********************************************************/
NMLTYPE NML::peek()
{
    error_type = NML_NO_ERROR;
    if (fast_mode) {
	cms->peek();
	switch (cms->status) {
	case CMS_READ_OLD:
	    return (0);
	case CMS_READ_OK:
	    if (((NMLmsg *) cms->subdiv_data)->type <= 0 && !cms->isserver) {
		rcs_print_error
		    ("NML: New data received but type of %d is invalid.\n",
		    (int)((NMLmsg *) cms->subdiv_data)->type);
		return -1;
	    }
	    return (((NMLmsg *) cms->subdiv_data)->type);

	default:
	    set_error();
	    return -1;
	}
    }
    if (NULL == cms) {
	if (error_type != NML_INVALID_CONFIGURATION) {
	    error_type = NML_INVALID_CONFIGURATION;
	    rcs_print_error("NML::peek: CMS not configured.\n");
	}
	return (-1);
    }

    if (cms->is_phantom) {
	if (NULL != phantom_peek) {
	    return ((*phantom_peek) ());
	} else {
	    return (0);
	}
    }
    if (!cms->force_raw) {
	cms->set_mode(CMS_READ);
    }

    cms->peek();
    if (!cms->force_raw) {
	if (cms->status == CMS_READ_OK) {
	    if (-1 == format_output()) {
		error_type = NML_FORMAT_ERROR;
		return (-1);
	    }
	}
    }

    switch (cms->status) {
    case CMS_READ_OLD:
	return (0);
    case CMS_READ_OK:
	if (((NMLmsg *) cms->subdiv_data)->type <= 0 && !cms->isserver) {
	    rcs_print_error
		("NML: New data received but type of %d is invalid.\n",
		(int)((NMLmsg *) cms->subdiv_data)->type);
	    return -1;
	}
	return (((NMLmsg *) cms->subdiv_data)->type);

    default:
	set_error();
	return -1;
    }

}

/***********************************************************
* NML Member Function: format_output()
* Purpose: Formats the data read from a CMS buffer as required
* by the process that created this NML. The formatting converts
* messages from some platform independent format to a platform
* specific format or vice versa. (Performing byte-swapping etc.)
* Returns:
*  0  The format was successful.
*  -1 An error occurred.
* Notes:
*  1. There are 3 conditions under which format_output may be
* called.
*     i. The data is being read out as is. (cms->mode == CMS_RAW_OUT).
*    ii. A user needs the data in the local platform-specific or raw format
*        but the buffer has been encoded in a platform-independent or
*         neutral format.   (cms->mode == CMS_DECODE).
*   iii. An NML_SERVER needs the data encoded in a platform-independent
*        or neutral format to send it out over the network but the buffer
*        is in a local platform-specific or raw format.
*         (cms->mode == CMS_ENCODE)
*  2. This function uses a list of format functions supplied
* by the user and stored int the format_chain.
* Returns:
* 0 = Success.
* -1 = Error.
***********************************************************/
int NML::format_output()
{
    NMLTYPE new_type;
    long new_size;

    /* Check pointers */
    if (NULL == cms) {
	rcs_print_error("NML: cms is NULL.\n");
	return (-1);
    }

    if (cms->force_raw) {
	return 0;
    }

    if (forced_type > 0) {
	new_type = forced_type;
    }

    switch (cms->mode) {
    case CMS_RAW_OUT:
	break;
    case CMS_DECODE:
	/* Check the status of CMS. */
	if (cms->status == CMS_READ_OK) {
	    /* Handle the generic part of the message. */
	    cms->format_low_ptr = cms->format_high_ptr = (char *) NULL;
	    cms->rewind();	/* Move to the start of encoded buffer. */
	    cms->update(new_type);	/* Get the message type from encoded
					   buffer. */
	    cms->update(new_size);	/* Get the message size from encoded
					   buffer. */
	    if (forced_type > 0) {
		new_type = forced_type;
	    }
	    ((NMLmsg *) cms->subdiv_data)->type = new_type;	/* Store type 
								   in
								   message. */
	    ((NMLmsg *) cms->subdiv_data)->size = new_size;	/* Store size 
								   in
								   message. */

	    if (new_size > cms->max_message_size) {
		rcs_print_error("NML: Message %" PRId32 " of size  %ld \n", new_type,
		    new_size);
		rcs_print_error
		    ("     too large for local buffer of %s of size %ld.\n",
		    cms->BufferName, cms->max_message_size);
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("Check that all processes agree on buffer size.\n");
		}
		cms->status = CMS_INSUFFICIENT_SPACE_ERROR;
		return (-1);
	    }

	    /* Check the list of format functions. */
	    if (!ignore_format_chain) {
		cms->format_low_ptr = (char *) cms->subdiv_data;
		cms->format_high_ptr = cms->format_low_ptr + cms->size;
		if (NULL == format_chain) {
		    rcs_print_error("NML::read: Format chain is NULL.\n");
		    return (-1);
		}
		/* Run through the list of format functions. */
		if (-1 == run_format_chain(new_type, cms->subdiv_data)) {
		    rcs_print_error("NMLread: NMLformat error\n");
		    if (verbose_nml_error_messages) {
			rcs_print_error("   (Buffer = %s, Process = %s)\n",
			    cms->BufferName, cms->ProcessName);
		    }
		    return (-1);
		}
	    }
	}
	break;
    case CMS_ENCODE:
	/* Check the status of CMS. */
	if (cms->status != CMS_MISC_ERROR) {
	    cms->format_low_ptr = cms->format_high_ptr = (char *) NULL;
	    cms->rewind();	/* Move to the start of the encoded buffer. */

	    /* Get the type and size from the message. */
	    new_type = ((NMLmsg *) cms->subdiv_data)->type;
	    new_size = ((NMLmsg *) cms->subdiv_data)->size;

	    if (forced_type > 0) {
		new_type = forced_type;
		((NMLmsg *) cms->subdiv_data)->type = forced_type;
	    }

	    /* Store the type and size in the encoded buffer. */
	    cms->update(new_type);
	    cms->update(new_size);

	    if (new_size > cms->max_message_size) {
		rcs_print_error("NML: Message %" PRId32 " of size  %ld\n", new_type,
		    new_size);
		rcs_print_error
		    ("     too large for local buffer of %s of size %ld.\n",
		    cms->BufferName, cms->max_message_size);
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("Check that all processes agree on buffer size.\n");
		}
		cms->status = CMS_INSUFFICIENT_SPACE_ERROR;
		return (-1);
	    }

	    /* Check the list of format functions. */
	    if (!ignore_format_chain) {
		cms->format_low_ptr = (char *) cms->subdiv_data;
		cms->format_high_ptr = cms->format_low_ptr + cms->size;
		if (NULL == format_chain) {
		    rcs_print_error("NML::read: Format chain is NULL.\n");
		    return (-1);
		}

		/* Run through the list of format functions. */
		if (-1 == run_format_chain(new_type, cms->subdiv_data)) {
		    rcs_print_error("NMLread: NMLformat error\n");
		    if (verbose_nml_error_messages) {
			rcs_print_error("   (Buffer = %s, Process = %s)\n",
			    cms->BufferName, cms->ProcessName);
		    }
		    return (-1);
		}

		/* Get the new size of the message now that it's been
		   encoded. */
		cms->get_encoded_msg_size();
	    }
	}
	break;
    default:
	rcs_print_error("NML::format_output: invalid format mode. (%d)\n",
	    cms->mode);
	return (-1);
    }
    if (forced_type > 0) {
	((NMLmsg *) cms->subdiv_data)->type = forced_type;
    }

    return (((int) cms->status < 0) ? -1 : 0);
}

/*************************************************************
* NML Member Function: write()
* Purpose: This write function provides users with an alternative
* style of writing a message.
* Parameters:
* NMLmsg &nml_msg - Reference to the message to be written.
* Returns:
*  0 - The message was successfully written.
*  -1 - An error occurred. (Timeouts are considered errors.)
*************************************************************/
int NML::write(NMLmsg & nml_msg, int *serial_number)
{
    return (write(&nml_msg, serial_number));	/* Call the other NML::write() */
}

/*************************************************************
* NML Member Function: write()
* Purpose: Writes a message to the global buffer.
* Parameters:
* NMLmsg *nml_msg - Address of the message to be written.
* Returns:
*  0 - The message was successfully written.
*  -1 - An error occurred. (Timeouts are considered errors.)
*************************************************************/
int NML::write(NMLmsg * nml_msg, int *serial_number)
{
    error_type = NML_NO_ERROR;
    if (fast_mode) {
	*cms_inbuffer_header_size = nml_msg->size;
	cms->write(nml_msg, serial_number);
	if (*cms_status == CMS_WRITE_OK) {
	    return (0);
	}
	set_error();
	return (-1);
    }
    /* Check pointers. */
    if (NULL == cms) {
	if (error_type != NML_INVALID_CONFIGURATION) {
	    error_type = NML_INVALID_CONFIGURATION;
	    rcs_print_error("NML::write: CMS not configured.\n");
	}
	return (-1);
    }

    if (NULL == nml_msg) {
	error_type = NML_INVALID_MESSAGE_ERROR;
	rcs_print_error("NML::write: Message is NULL.\n");
	return (-1);
    }

    if ((nml_msg->size == 0 || nml_msg->type == 0) && !cms->isserver) {
	error_type = NML_INVALID_MESSAGE_ERROR;
	rcs_print_error("NML::write: Message size or type is zero.\n");
	rcs_print_error
	    ("NML: Check that the message was properly constructed.\n");
    }

    /* Handle Phantom Buffers. */
    if (cms->is_phantom) {
	if (NULL != phantom_write) {
	    return ((*phantom_write) (nml_msg, serial_number));
	} else {
	    return (0);
	}
    }

    /* Set CMS to a write mode. */
    cms->set_mode(CMS_WRITE);

    /* Format the message if necessary. */
    if (-1 == format_input(nml_msg)) {
	error_type = NML_FORMAT_ERROR;
	return -1;
    }

    if (CMS_RAW_IN == cms->mode) {
	cms->write(nml_msg, serial_number);	/* Write the unformatted message.  */
    } else {
	cms->write(cms->subdiv_data, serial_number);	/* Write the formatted message.  */
    }

    if (CMS_WRITE_OK == cms->status) {
	error_type = NML_NO_ERROR;
	return (0);
    }

    return set_error();
}

/*************************************************************
* NML Member Function: set_error
* Purpose: This write function provides users with an alternative
* style of writing a message.
* Parameters:
* NMLmsg &nml_msg - Reference to the message to be written.
* Returns:
*  0 - The message was successfully written.
*  -1 - An error occurred. (Timeouts are considered errors.)
* Check error_type for more info.
*************************************************************/
int NML::set_error()
{
    if (error_type != NML_NO_ERROR) {
	return -1;
    }

    if (NULL == cms) {
	error_type = NML_INVALID_CONFIGURATION;
	return 0;
    }

    /* Choose return value. */
    switch (cms->status) {
    case CMS_TIMED_OUT:
	error_type = NML_TIMED_OUT;
	return (-1);

    case CMS_QUEUE_FULL:
	error_type = NML_QUEUE_FULL_ERROR;
	break;

    case CMS_NO_MASTER_ERROR:
	error_type = NML_NO_MASTER_ERROR;
	break;

    case CMS_WRITE_WAS_BLOCKED:
	error_type = NML_BUFFER_NOT_READ;
	break;

    case CMS_STATUS_NOT_SET:	/* The status variable has not been set yet. */
    case CMS_READ_OLD:		/* Read successful, but data is old. */
    case CMS_READ_OK:		/* Read successful so far. */
    case CMS_WRITE_OK:		/* Write successful so far. */
    case CMS_CLEAR_OK:		/* A clear operation was successful.  */
	error_type = NML_NO_ERROR;
	break;

    case CMS_RESOURCE_CONFLICT_ERROR:
    case CMS_CREATE_ERROR:
    case CMS_CONFIG_ERROR:
	error_type = NML_INVALID_CONFIGURATION;
	break;

    case CMS_MISC_ERROR:
    default:
	error_type = NML_INTERNAL_CMS_ERROR;
	break;

    }

    if (error_type == NML_NO_ERROR) {
	return 0;
    }
    if (!info_printed) {
	print_info();
    }

    return -1;
}

/*************************************************************
* NML Member Function: write_if_read()
* Purpose: This write function provides users with an alternative
* style of writing a message.
* Parameters:
* NMLmsg &nml_msg - Reference to the message to be written.
* Returns:
*  0 - The message was successfully written.
*  -1 - An error occurred. (Timeouts are considered errors.)
* Check error_type for more info.
*************************************************************/
int NML::write_if_read(NMLmsg & nml_msg, int *serial_number)
{
    return (write_if_read(&nml_msg, serial_number));
}

/***********************************************************
* NML Member Function: write_if_read()
* Purpose: Write a message to the global buffer, but do not
*  over-write another message if that message is unread.
* Parameters:
*  NMLmsg *nml_msg - Address of the message to be written.
* Returns:
*  0 - The message was successfully written.
*  -1 - An error occurred.
* (Timeouts, and unread buffers  are considered errors.)
* Check error_type for more info.
************************************************************/
int NML::write_if_read(NMLmsg * nml_msg, int *serial_number)
{
    error_type = NML_NO_ERROR;
    if (fast_mode) {
	cms->header.in_buffer_size = nml_msg->size;
	cms->write(nml_msg, serial_number);
	if (cms->status == CMS_WRITE_OK) {
	    return (0);
	}
	set_error();
	return (-1);
    }
    if (NULL == cms) {
	if (error_type != NML_INVALID_CONFIGURATION) {
	    error_type = NML_INVALID_CONFIGURATION;
	    rcs_print_error("NML::write_if_read: CMS not configured.\n");
	}
	return (-1);
    }

    if (NULL == nml_msg) {
	error_type = NML_INVALID_MESSAGE_ERROR;
	rcs_print_error("NML::write_if_read: Message is NULL.\n");
	return (-1);
    }

    if ((nml_msg->size == 0 || nml_msg->type == 0) && !cms->isserver) {
	error_type = NML_INVALID_MESSAGE_ERROR;
	rcs_print_error
	    ("NML::write_if_read: Message size or type is zero.\n");
	if (verbose_nml_error_messages) {
	    rcs_print_error
		("NML: Check that the message was properly constructed.\n");
	}
    }

    if (cms->is_phantom) {
	if (NULL != phantom_write_if_read) {
	    return ((*phantom_write_if_read) (nml_msg, serial_number));
	} else {
	    return (0);
	}
    }

    cms->set_mode(CMS_WRITE);
    if (-1 == format_input(nml_msg)) {
	error_type = NML_FORMAT_ERROR;
	return -1;
    }

    if (CMS_RAW_IN == cms->mode) {
	cms->write_if_read(nml_msg, serial_number);
    } else {
	cms->write_if_read(cms->subdiv_data, serial_number);
    }

    return (set_error());
}

/*******************************************************************
* NML Member Function: format_input()
* Purpose: Formats the in an NML message to be written to a CMS buffer
*  as required by the configuration file. The formatting converts
*  messages from some platform independent format to a platform
*  specific format or vice versa. (Performing byte-swapping etc.)
* Parameters:
* NMLmsg *nml_msg - The address of the NML message.
* Returns:
* 0 = Success.
* -1 = Error.
* Notes:
*  1. There are 3 conditions that format_input may be called under.
*    i. The message will be written to the buffer without any formatting.
*       (cms->mode == CMS_RAW_IN)
*    ii. The message is in a native or raw format and needs to be encoded
*  in a neutral format before being sent over the network or into a
*  neutral buffer.
*        (cms->mode == CMS_ENCODE)
*   iii. The process calling this is a server which received a neutrally
* encoded buffer over the network which must be converted to native
* format before being written into a raw buffer.
*        (cms->mode == CMS_DECODE)
*  2. This function is for internal NML use only.
******************************************************************/
int NML::format_input(NMLmsg * nml_msg)
{
    NMLTYPE new_type;
    long new_size;
    if (NULL == cms) {
	return -1;
    }

    if (cms->force_raw) {
	cms->mode = CMS_RAW_IN;
    }

    switch (cms->mode) {
    case CMS_RAW_IN:
	/* Make sure the message size is not larger than the buffer size. */
	if (nml_msg->size > cms->max_message_size) {
	    rcs_print_error("NML: Message size(%ld) too large for"
		" CMS buffer size of %ld.\n",
		nml_msg->size, cms->max_message_size);
	    cms->status = CMS_INSUFFICIENT_SPACE_ERROR;
	    return (-1);
	}
	cms->header.in_buffer_size = nml_msg->size;
	break;
    case CMS_ENCODE:
	/* Make sure the message size is not larger than the buffer size. */
	if (nml_msg->size > cms->max_message_size) {
	    rcs_print_error("NML: Message size(%ld) too large for"
		" CMS buffer size of %ld.\n",
		nml_msg->size, cms->max_message_size);
	    cms->status = CMS_INSUFFICIENT_SPACE_ERROR;
	    return (-1);
	}

	cms->format_low_ptr = (char *) nml_msg;
	cms->format_high_ptr = cms->format_low_ptr + nml_msg->size;
	/* Handle the generic part of the message. */
	cms->rewind();		/* Move to the start of the encoded buffer. */
	cms->update(nml_msg->type);	/* Store message type in encoded
					   buffer. */
	cms->update(nml_msg->size);	/* Store message size in encoded
					   buffer. */

	/* Check list of format functions. */
	if (!ignore_format_chain) {
	    if (NULL == format_chain) {
		rcs_print_error("NML::read: Format chain is NULL.\n");
		return (-1);
	    }

	    /* Run through list of format functions. */
	    if (-1 == run_format_chain(nml_msg->type, nml_msg)) {
		rcs_print_error("NMLwrite: format error\n");
		if (verbose_nml_error_messages) {
		    rcs_print_error("   (Buffer = %s, Process = %s)\n",
			cms->BufferName, cms->ProcessName);
		}
		return (-1);
	    }
	}
	/* Determine the new size of the message now that its encoded. */
	cms->header.in_buffer_size = cms->get_encoded_msg_size();
	break;
    case CMS_DECODE:
	cms->format_low_ptr = cms->format_high_ptr = (char *) NULL;
	cms->rewind();		/* Move to the start of the encoded buffer. */
	cms->update(new_type);	/* Get message type from encoded buffer. */
	cms->update(new_size);	/* Get message size from encoded buffer. */

	/* Make sure the message size is not larger than the buffer size. */
	if (new_size > cms->max_message_size) {
	    rcs_print_error("NMLwrite: Message size(%ld) too large for"
		" CMS buffer size of %ld.\n", new_size, cms->max_message_size);
	    cms->status = CMS_INSUFFICIENT_SPACE_ERROR;
	    return (-1);
	}
	cms->format_low_ptr = (char *) cms->subdiv_data;
	cms->format_high_ptr = cms->format_low_ptr + cms->size;

	/* Store the new type and size in the raw message. */
	((NMLmsg *) cms->subdiv_data)->type = new_type;
	((NMLmsg *) cms->subdiv_data)->size = new_size;

	/* Check the list of format functions. */
	if (!ignore_format_chain) {
	    if (NULL == format_chain) {
		rcs_print_error("NML::read: Format chain is NULL.\n");
		return (-1);
	    }

	    /* Run through the list of format functions. */
	    if (-1 == run_format_chain(new_type, cms->subdiv_data)) {
		rcs_print_error("NMLwrite: format error\n");
		rcs_print_error("   (Buffer = %s, Process = %s)\n",
		    cms->BufferName, cms->ProcessName);
		return (-1);
	    }
	}
	/* Choose a size that will ensure the entire message will be read
	   out. */
	if (cms->format_size < ((long) sizeof(NMLmsg))) {
	    cms->format_size = sizeof(NMLmsg);
	}
	if (cms->format_size > new_size) {
	    ((NMLmsg *) cms->subdiv_data)->size = (long) cms->format_size;
	}
	cms->header.in_buffer_size = ((NMLmsg *) cms->subdiv_data)->size;
	break;
    default:
	rcs_print_error("NML::format_input: invalid mode (%d).\n", cms->mode);
	return (-1);
    }

    return (((int) cms->status < 0) ? -1 : 0);
}

int NML::run_format_chain(NMLTYPE type, void *buf)
{
    NML_FORMAT_PTR format_function;

    format_function = (NML_FORMAT_PTR) format_chain->get_head();
    while (NULL != format_function) {
	switch ((*format_function) (type, buf, cms)) {
	case -1:
	    return (-1);
	case 0:
	    break;
	case 1:
	    return (0);
	}
	format_function = (NML_FORMAT_PTR) format_chain->get_next();
    }
    return (0);
}

int NML::prefix_format_chain(NML_FORMAT_PTR f_ptr)
{
    if (NULL == format_chain) {
	format_chain = new LinkedList;
    }
    if (NULL != format_chain) {
	format_chain->store_at_head((void *) f_ptr, 0, 0);
    }
    return (0);
}

/**************************************************************************
* NML member function: msg2str
* Parameter: NMLmsg &msg -- Reference to message to be converted into a string.
* Returns: Returns a pointer to the cms->encoded_data buffer if successful
* since this should contain the string or NULL if there was an error.
***************************************************************************/
const char *NML::msg2str(NMLmsg & msg)
{
    return msg2str(&msg);
}

/**************************************************************************
* NML member function: msg2str
* Parameter: NMLmsg *msg -- Pointer to message to be converted into a string.
* Returns: Returns a pointer to the cms->encoded_data buffer if successful
* since this should contain the string or NULL if there was an error.
***************************************************************************/
const char *NML::msg2str(NMLmsg * nml_msg)
{
    CMS *orig_cms = cms;
    char *str = NULL;
    if (NULL == nml_msg) {
	return NULL;
    }
    if (NULL == cms) {
	int msg_length = nml_msg->size;
	if (NULL != cms_for_msg_string_conversions) {
	    if ((cms_for_msg_string_conversions->size > 16 * msg_length &&
		    cms_for_msg_string_conversions->size > 2048) ||
		cms_for_msg_string_conversions->size < 4 * msg_length) {
		delete cms_for_msg_string_conversions;
		cms_for_msg_string_conversions = 0;
	    }
	}
	if (NULL == cms_for_msg_string_conversions) {
	    cms_for_msg_string_conversions =
		new CMS(nml_msg->size * 4 + 16 + (16 - (nml_msg->size % 16)));
	}
	cms = cms_for_msg_string_conversions;
    }
    cms->set_temp_updater(CMS_DISPLAY_ASCII_ENCODING);
    cms->set_mode(CMS_ENCODE);
    if (-1 == format_input(nml_msg)) {
	cms->restore_normal_updater();
	error_type = NML_FORMAT_ERROR;
	cms = orig_cms;
	return ((char *) NULL);
    }
    cms->restore_normal_updater();
    str = (char *) cms->encoded_data;
    cms = orig_cms;
    return (const char *) str;
}

/**************************************************************************
* NML member function: str2msg
* Parameter: NMLmsg *msg -- Pointer to message to be converted into a NMLmsg.
* Returns: Returns a pointer to the cms->encoded_data buffer if successful
* since this should contain the string or NULL if there was an error.
***************************************************************************/
NMLTYPE NML::str2msg(const char *string)
{
    CMS *orig_cms = cms;
    if (NULL == string) {
	return -1;
    }
    if (NULL == cms) {
	int string_length = strlen(string);
	if (NULL != cms_for_msg_string_conversions) {
	    if ((cms_for_msg_string_conversions->size > 16 * string_length &&
		    cms_for_msg_string_conversions->size > 2048) ||
		cms_for_msg_string_conversions->size < 4 * string_length) {
		delete cms_for_msg_string_conversions;
		cms_for_msg_string_conversions = 0;
	    }
	}
	if (NULL == cms_for_msg_string_conversions) {
	    cms_for_msg_string_conversions =
		new CMS(string_length * 4 + 16 + (16 - (string_length % 16)));
	}
	cms = cms_for_msg_string_conversions;
    }
    cms->set_temp_updater(CMS_DISPLAY_ASCII_ENCODING);
    cms->set_mode(CMS_DECODE);
    strcpy((char *) cms->encoded_data, (const char *) string);
    cms->status = CMS_READ_OK;
    if (-1 == format_output()) {
	cms->restore_normal_updater();
	error_type = NML_FORMAT_ERROR;
	cms = orig_cms;
	return -1;
    }
    cms->restore_normal_updater();
    cms = orig_cms;

    switch (cms->status) {
    case CMS_READ_OLD:
	error_type = NML_NO_ERROR;
	return (0);
    case CMS_READ_OK:
	error_type = NML_NO_ERROR;
	return (((NMLmsg *) cms->subdiv_data)->type);
    case CMS_TIMED_OUT:
	error_type = NML_TIMED_OUT;
	return -1;
    case CMS_MISC_ERROR:
    case CMS_NO_MASTER_ERROR:
	error_type = NML_INTERNAL_CMS_ERROR;
	/* Fallthrough */
    default:
	return -1;
    }

}

static int info_message_printed = 0;
char cwd_buf[256];
char host_name_buf[MAXHOSTNAMELEN];

char last_bufname[10];
char last_procname[10];
char last_cfg_file[40];

/**************************************************************************
* NML member function: print_info()
* Prints the buffer, process names and configuration file information.
***************************************************************************/
void NML::print_info(const char * const bufname, const char * const procname, const char * const cfg_file)
{
    info_printed = 1;
    if (!verbose_nml_error_messages) {
	return;
    }
    if (NULL == cms || error_type != NML_NO_ERROR) {
	if (max_rcs_errors_to_print <= rcs_errors_printed &&
	    max_rcs_errors_to_print >= 0) {
	    return;
	}
    }
    if (error_type == NML_QUEUE_FULL_ERROR && !cms_print_queue_full_messages) {
	return;
    }
    if (NULL != cms) {
	if (cms->status < 0) {
	    if (max_rcs_errors_to_print <= rcs_errors_printed &&
		max_rcs_errors_to_print >= 0) {
		return;
	    }
	}
    }
    if (NULL != bufname && NULL != procname && NULL != cfg_file) {
	if (!strncmp(bufname, last_bufname, 10)
	    && !strncmp(procname, last_procname, 10)
	    && !strncmp(cfg_file, last_cfg_file, 40)) {
	    return;
	}
	snprintf(last_bufname, 10, "%s", bufname);
	snprintf(last_procname, 10, "%s", procname);
	snprintf(last_cfg_file, 40, "%s", cfg_file);
    }
    if (!info_message_printed) {
	rcs_print
	    ("\n**********************************************************\n");
	rcs_print("* Current Directory = %s\n", getcwd(cwd_buf, 256));
	if (nml_print_hostname_on_error) {
	    gethostname(host_name_buf, MAXHOSTNAMELEN);
	    if (host_name_buf[0] != 0) {
		rcs_print("* Host = %s\n", host_name_buf);
	    }
	}
	rcs_print("* ");
	info_message_printed = 1;
    }
    rcs_print
	("\n**********************************************************\n");
    if (NULL != cms) {
	rcs_print("* BufferName = %s\n", cms->BufferName);
	rcs_print("* BufferType = %d\n", cms->BufferType);
	rcs_print("* ProcessName = %s\n", cms->ProcessName);
	rcs_print("* Configuration File = %s\n", cfgfilename);
	rcs_print("* CMS Status = %d (%s)\n", cms->status,
	    cms->status_string(cms->status));
	rcs_print("* Recent errors repeated:\n");
	rcs_print("%s\n", last_error_bufs[0]);
	rcs_print("%s\n", last_error_bufs[1]);
	rcs_print("%s\n", last_error_bufs[2]);
	rcs_print("%s\n", last_error_bufs[3]);
	memset(last_error_bufs[0], 0, 100);
	memset(last_error_bufs[1], 0, 100);
	memset(last_error_bufs[2], 0, 100);
	memset(last_error_bufs[3], 0, 100);
	if (NULL == strstr(cms->BufferLine, "\n")) {
	    rcs_print("* BufferLine: %s\n", cms->BufferLine);

	} else {
	    rcs_print("* BufferLine: %s", cms->BufferLine);
	}
	if (NULL == strstr(cms->ProcessLine, "\n")) {
	    rcs_print("* ProcessLine: %s\n", cms->ProcessLine);
	} else {
	    rcs_print("* ProcessLine: %s", cms->ProcessLine);
	}
    } else {
	if (NULL != bufname) {
	    rcs_print("* BufferName = %s\n", bufname);
	}
	if (NULL != procname) {
	    rcs_print("* ProcessName = %s\n", procname);
	}
    }
    if (NULL != cfg_file) {
	rcs_print("* Config File = %s\n", cfg_file);
    }
    rcs_print("* error_type = %d (%s)\n", error_type,
	NML_ERROR_TYPE_STRINGS[error_type]);
    rcs_print
	("************************************************************\n\n");
}

void nml_start()
{
    spawn_nml_servers();
}

void nml_cleanup()
{
    NML *nml;
    nml_server_cleanup();

    if (NULL != NML_Main_Channel_List) {
	rcs_print_debug(PRINT_NML_DESTRUCTORS,
	    "Deleting %d channels from the NML_Main_Channel_List.\n",
	    NML_Main_Channel_List->list_size);
	nml = (NML *) NML_Main_Channel_List->get_head();
	while (NULL != nml) {
	    if (nml->cms != NULL) {
		rcs_print_debug(PRINT_NML_DESTRUCTORS,
		    "Deleting %s NML channel from NML_Main_Channel_List.\n",
		    nml->cms->BufferName);
	    }
	    nml->delete_channel();

	    rcs_print_debug(PRINT_NML_DESTRUCTORS,
		"NML channel deleted from NML_Main_Channel_List\n");
	    if (NULL == NML_Main_Channel_List) {
		return;
	    }
	    NML_Main_Channel_List->delete_current_node();
	    nml = (NML *) NML_Main_Channel_List->get_next();
	}
	if (NULL != NML_Main_Channel_List) {
	    delete NML_Main_Channel_List;
	    NML_Main_Channel_List = (LinkedList *) NULL;
	}
    }
    if (NULL != Dynamically_Allocated_NML_Objects) {
	nml = (NML *) Dynamically_Allocated_NML_Objects->get_head();
	while (NULL != nml) {
	    if (nml->cms != NULL) {
		rcs_print_debug(PRINT_NML_DESTRUCTORS,
		    "Deleting %s NML channel from Dynamically_Allocated_NML_Objects.\n",
		    nml->cms->BufferName);
	    }
	    delete nml;

	    rcs_print_debug(PRINT_NML_DESTRUCTORS,
		"NML channel deleted from Dynamically_Allocated_NML_Objects\n");
	    if (NULL == Dynamically_Allocated_NML_Objects) {
		return;
	    }
	    Dynamically_Allocated_NML_Objects->delete_current_node();
	    nml = (NML *) Dynamically_Allocated_NML_Objects->get_next();
	}
	if (NULL != Dynamically_Allocated_NML_Objects) {
	    delete Dynamically_Allocated_NML_Objects;
	    Dynamically_Allocated_NML_Objects = (LinkedList *) NULL;
	}
    }
    nmlClearHostAliases();
}
/*! \todo Another #if 0 */
#if 0
void nml_wipeout_lists()
{
    if (NULL != NML_Main_Channel_List) {
	delete NML_Main_Channel_List;
	NML_Main_Channel_List = (LinkedList *) NULL;
    }
    if (NULL != Dynamically_Allocated_NML_Objects) {
	delete Dynamically_Allocated_NML_Objects;
	Dynamically_Allocated_NML_Objects = (LinkedList *) NULL;
    }
    if (NULL != NML_Default_Super_Server) {
	delete NML_Default_Super_Server;
	NML_Default_Super_Server = (NML_SUPER_SERVER *) NULL;
    }
}
#endif
int NML::print_queue_info()
{
    if (NULL == cms) {
	rcs_print_error("NML::print_queue_info() - NULL == cms\n");
	return (-1);
    }
    if (!cms->queuing_enabled) {
	rcs_print_error("NML::print_queue_info() - Queueing Not Enabled.\n");
	return (-1);
    }
    if (cms->ProcessType != CMS_LOCAL_TYPE) {
	rcs_print_error
	    ("NML::print_queue_info() - REMOTE Connection: Queueing Data Not Available.\n");
	return (-1);
    }
    rcs_print
	("head = %ld(0x%lX); tail=%ld(0x%lX); queue_length=%ld,end_queue_space=%ld(0x%lX); write_id=%ld\n",
	cms->queuing_header.head, cms->queuing_header.head,
	cms->queuing_header.tail, cms->queuing_header.tail,
	cms->queuing_header.queue_length, cms->queuing_header.end_queue_space,
	cms->queuing_header.end_queue_space, cms->queuing_header.write_id);
    return (0);
}

// Function added at Steve Balakirsky's request. Polls a channel indefinitely
// waiting for it to open.
NML *nmlWaitOpen(NML_FORMAT_PTR fPtr, char *buffer, char *name, char *file,
    double sleepTime)
{
    NML *nmlChannel = 0;

    RCS_PRINT_DESTINATION_TYPE olddest = get_rcs_print_destination();
    set_rcs_print_destination(RCS_PRINT_TO_NULL);
    nmlChannel = new NML(fPtr, buffer, name, file);
    while (!nmlChannel->reset()) {
	esleep(sleepTime);
    }
    set_rcs_print_destination(olddest);
    return (nmlChannel);
}

// Special functions for dealing with subdivisions

/* Write a message. (Use reference) */
int NML::write_subdivision(int subdiv, NMLmsg & nml_msg)
{
    if (NULL != cms) {
	if (cms->set_subdivision(subdiv) < 0) {
	    return -1;
	}
    }
    return write(nml_msg);
}

/* Write a message. (Use pointer) */
int NML::write_subdivision(int subdiv, NMLmsg * nml_msg)
{
    if (NULL != cms) {
	if (cms->set_subdivision(subdiv) < 0) {
	    return -1;
	}
    }
    return write(nml_msg);
}

/* Write to subdivision only if buffer was_read. (Use reference) */
int NML::write_if_read_subdivision(int subdiv, NMLmsg & nml_msg)
{
    if (NULL != cms) {
	if (cms->set_subdivision(subdiv) < 0) {
	    return -1;
	}
    }
    return write_if_read(nml_msg);
}

/* Write to subdivision only if buffer was_read. (Use pointer) */
int NML::write_if_read_subdivision(int subdiv, NMLmsg * nml_msg)
{
    if (NULL != cms) {
	if (cms->set_subdivision(subdiv) < 0) {
	    return -1;
	}
    }
    return write_if_read(nml_msg);
}

/* Read from a particular subdivision. */
NMLTYPE NML::read_subdivision(int subdiv)
{
    if (NULL != cms) {
	if (cms->set_subdivision(subdiv) < 0) {
	    return -1;
	}
    }
    return read();
}

/* Read from a particular subdivision. (Wait for new data). */
NMLTYPE NML::blocking_read_subdivision(int subdiv, double timeout)
{
    if (NULL != cms) {
	if (cms->set_subdivision(subdiv) < 0) {
	    return -1;
	}
    }
    return blocking_read(timeout);
}

/* Read buffer without changing was_read */
NMLTYPE NML::peek_subdivision(int subdiv)
{
    if (NULL != cms) {
	if (cms->set_subdivision(subdiv) < 0) {
	    return -1;
	}
    }
    return peek();
}

NMLTYPE NML::blocking_read_extended(double timeout, double poll_interval)
{
    if (cms == NULL) {
	return -1;
    }

    if (cms->BufferType == CMS_SHMEM_TYPE) {
	return blocking_read(timeout);
    } else {
	NMLTYPE type = 0;
	double time_elapsed = 0.0;
	double start_time = etime();
	while (!type && (time_elapsed < timeout || timeout < 0.0)) {
	    esleep(poll_interval);
	    type = read();
	    if (timeout > 0.0 && !type) {
		time_elapsed = etime() - start_time;
	    }
	    if (time_elapsed < 0.0) {
		break;
	    }
	}
	return type;
    }
}

/* Get the number of messages written to this buffer so far. */
int NML::get_msg_count()
{
    if (NULL == cms) {
	return -1;
    }
    return cms->get_msg_count();
}

/* Get Diagnostics Information. */
NML_DIAGNOSTICS_INFO *NML::get_diagnostics_info()
{
    if (NULL == cms) {
	return NULL;
    }
    return (NML_DIAGNOSTICS_INFO *) cms->get_diagnostics_info();
}

void nmlSetHostAlias(const char * const hostName, const char * const hostAlias)
{
    if (NULL == cmsHostAliases) {
	cmsHostAliases = new LinkedList;
    }
    CMS_HOST_ALIAS_ENTRY entry;
    snprintf(entry.host, 64, "%s", hostName);
    snprintf(entry.alias, 64,  "%s", hostAlias);
    cmsHostAliases->store_at_tail(&entry, sizeof(entry), 1);
}

void nmlClearHostAliases()
{
    if (NULL != cmsHostAliases) {
	delete cmsHostAliases;
	cmsHostAliases = NULL;
    }
}

void nmlAllowNormalConnection()
{
    cms_connection_mode = CMS_NORMAL_CONNECTION_MODE;
}

void nmlForceRemoteConnection()
{
    cms_connection_mode = CMS_FORCE_REMOTE_CONNECTION_MODE;
}

void nmlForceLocalConnection()
{
    cms_connection_mode = CMS_FORCE_LOCAL_CONNECTION_MODE;
}
