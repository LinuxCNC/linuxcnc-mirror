/********************************************************************
* Description: cms.cc
*   C++ file for the Communication Management System (CMS).
*   Includes member functions for class CMS.
*   See cms_in.cc for the internal interface member functions and
*   cms_up.cc for the update functions.
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

#include "rcsversion.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>		/* malloc(), free() */
#include <stddef.h>		/* size_t */
#include <string.h>		/* strcpy(), strlen(),memcpy() */
    /* strcmp(),strchr() */
#include <ctype.h>		// tolower(), toupper()
#include <errno.h>		/* errno, ERANGE */

#ifdef __cplusplus
}
#endif
#include <rtapi_string.h>
#include "cms_cfg.hh"
#include "cms.hh"		/* class CMS */
#include "cms_up.hh"		/* class CMS_UPDATER */
#include "cms_xup.hh"		/* class CMS_XDR_UPDATER */
#include "cms_aup.hh"		/* class CMS_ASCII_UPDATER */
#include "cms_dup.hh"		/* class CMS_DISPLAY_ASCII_UPDATER */
#include "rcs_print.hh"		/* rcs_print_error(), separate_words() */
				/* rcs_print_debug() */
#include "cmsdiag.hh"
#include "linklist.hh"          /* LinkedList */
#include "physmem.hh"

LinkedList *cmsHostAliases = NULL;
CMS_CONNECTION_MODE cms_connection_mode = CMS_NORMAL_CONNECTION_MODE;

/* Static Class Data Members. */
int CMS::number_of_cms_objects = 0;
int cms_encoded_data_explosion_factor = 4;

/*! \todo Another #if 0 */
#if 0
static int convert2lower(char *dest, const char *src, int len)
{
    int i;
    for (i = 0; i < len; i++) {
	if (src[i] == 0) {
	    dest[i] = 0;
	    return i;
	}
	dest[i] = tolower(src[i]);
    }
    return i;
}
#endif

static int convert2upper(char *dest, const char *src, int len)
{
    int i;
    for (i = 0; i < len; i++) {
	if (src[i] == 0) {
	    dest[i] = 0;
	    return i;
	}
	dest[i] = toupper(src[i]);
    }
    return i;
}

/* Class CMS Member Functions */

void *CMS::operator new(size_t size)
{
    if (size < sizeof(CMS)) {
	rcs_print_error
	    ("CMS::operator new -- The size requested %zu is less than the mininimum size of CMS %zu.\n",
	    size, sizeof(CMS));
	rcs_print_error("This could indicate a version mismatch problem.\n");
	size = sizeof(CMS);
    }
    void *space = (void *) malloc(size);
    if (NULL != space) {
	memset(space, 0, size);
    }
    rcs_print_debug(PRINT_CMS_CONSTRUCTORS, "%p = CMS::new(%zu)\n", space,
	size);
    return space;
}

void CMS::operator delete(void *space)
{
    rcs_print_debug(PRINT_CMS_DESTRUCTORS, " CMS::delete(%p)\n", space);
    free(space);
    rcs_print_debug(PRINT_CMS_DESTRUCTORS, " CMS::delete successful.\n");
}

/* Constructor used for hard coded tests. */
/* Parameters: */
 /* n - Name of the buffer. */
 /* s - Size of the buffer. */
 /* nt - 0 buffer is not neutrally encoded, 1 buffer is neutrally encoded */
 /* set_to_server - 0 do NOT be a server, 1 be a server */
CMS::CMS(long s)
{
    /* Print a message if the PRINT_CMS_CONSTUCTORS */
    /* member of the print flags is set. */
    rcs_print_debug(PRINT_CMS_CONSTRUCTORS, "new CMS (%lu)", s);

    /* Init string buffers */
    memset(BufferName, 0, LINELEN);
    memset(BufferHost, 0, LINELEN);
    memset(ProcessName, 0, LINELEN);
    memset(BufferLine, 0, LINELEN);
    memset(ProcessLine, 0, LINELEN);
    memset(ProcessHost, 0, LINELEN);
    memset(buflineupper, 0, LINELEN);
    memset(proclineupper, 0, LINELEN);
    memset(PermissionString, 0, LINELEN);

    /* save constructor args */
    free_space = size = s;
    force_raw = 0;
    neutral = 0;
    isserver = 0;
    last_im = CMS_NOT_A_MODE;
    min_compatible_version = 0;
    confirm_write = 0;
    disable_final_write_raw_for_dma = 0;
    subdiv_data = 0;
    enable_diagnostics = 0;
    dpi = NULL;
    di = NULL;
    skip_area = 0;
    half_offset = s / 2;
    free_space = half_size = s / 2;
    fast_mode = 0;
    disable_diag_store = 0;
    diag_offset = 0;

    /* Initialize some variables. */
    read_permission_flag = 0;	/* Allow both read and write by default.  */
    write_permission_flag = 0;
    queuing_enabled = 0;
    fatal_error_occurred = 0;
    write_just_completed = 0;
    neutral_encoding_method = CMS_XDR_ENCODING;
    blocking_timeout = 0;
    total_subdivisions = 1;
    subdiv_size = size;
    current_subdivision = 0;
    enc_max_size = s;
    max_encoded_message_size = s;
    last_id_side0 = 0;
    last_id_side1 = 0;
    handle_to_global_data = NULL;
    dummy_handle = (PHYSMEM_HANDLE *) NULL;	/* Set pointers to NULL */
    /* so we'll know whether it really */
    /* points to something */

    delete_totally = 0;		/* If this object is deleted only do */
    /* normal delete instead of deleting totally. */

    mode = CMS_NOT_A_MODE;	/* Force user to set the mode before using. */

    open();			/* Allocate memory and initialize XDR streams */
}

/* Constructor used by cms_config. */
/* Parameters:  */
/*  bufline - The buffer line from a CMS configuration file. */
/* procline - The process line from a CMS configuration file. */
/* set_to_server - */
 /* -1 force this CMS object NOT to be in server mode. */
 /* 0 allow the parameter in the procline to set whether server mode is used */
 /* 1 force this CMS object to be in server mode. */
CMS::CMS(const char *bufline_in, const char *procline_in, int set_to_server)
{
    char *word[32]={0,};	/* Array of pointers to strings.  */
    char *buffer_type_name;	/* pointer to buffer type name from bufline */
    char *proc_type_name;	/* pointer to process type from procline */
    int i;
    min_compatible_version = 0;
    force_raw = 0;
    serial = 0;
    confirm_write = 0;
    disable_final_write_raw_for_dma = 0;
    /* Init string buffers */
    memset(BufferName, 0, LINELEN);
    memset(BufferHost, 0, LINELEN);
    memset(ProcessName, 0, LINELEN);
    memset(BufferLine, 0, LINELEN);
    memset(ProcessLine, 0, LINELEN);
    memset(ProcessHost, 0, LINELEN);
    memset(buflineupper, 0, LINELEN);
    memset(proclineupper, 0, LINELEN);
    memset(PermissionString, 0, LINELEN);

    /* Initialize some variables. */
    read_permission_flag = 0;	/* Allow both read and write by default.  */
    write_permission_flag = 0;
    queuing_enabled = 0;
    fatal_error_occurred = 0;
    write_just_completed = 0;
    neutral_encoding_method = CMS_XDR_ENCODING;
    blocking_timeout = 0;
    min_compatible_version = 0;
    enc_max_size = -1;
    max_encoded_message_size = 0;
    enable_diagnostics = 0;
    dpi = NULL;
    di = NULL;
    disable_diag_store = 0;
    diag_offset = 0;
    use_autokey_for_connection_number = 0;

    if ((NULL == bufline_in) || (NULL == procline_in)) {
	rcs_print_error("CMS: Pointer to bufline or procline is NULL.\n");
	return;
    }

    char *bufline = strdup(bufline_in);
    char *procline = strdup(procline_in);

    convert2upper(buflineupper, bufline, LINELEN);
    convert2upper(proclineupper, procline, LINELEN);

    is_phantom = 0;
    max_message_size = 0;
    using_external_encoded_data = 0;
    in_buffer_id = 0;
    last_id_side0 = 0;
    last_id_side1 = 0;
    delete_totally = 0;
    queuing_enabled = 0;
    split_buffer = 0;
    fatal_error_occurred = 0;
    consecutive_timeouts = 0;
    write_just_completed = 0;
    pointer_check_disabled = 0;
    blocking_timeout = 0;
    last_im = CMS_NOT_A_MODE;
    total_subdivisions = 1;
    size = 0;
    subdiv_size = 0;
    current_subdivision = 0;
    max_encoded_message_size = 0;
    skip_area = 0;
    half_offset = 0;
    half_size = 0;
    fast_mode = 0;
    last_id_side0 = 0;
    last_id_side1 = 0;
    free_space = 0;
    handle_to_global_data = NULL;

    dummy_handle = (PHYSMEM_HANDLE *) NULL;
    remote_port_type = CMS_NO_REMOTE_PORT_TYPE;

    /* Store the bufline and procline for debugging later. */
    rtapi_strxcpy(BufferLine, bufline);
    rtapi_strxcpy(ProcessLine, procline);

    /* Get parameters from the buffer's line in the config file. */
    if (separate_words(word, 9, bufline) != 9) {
	rcs_print_error("CMS: Error in buffer line from config file.\n");
	rcs_print_error("%s\n", bufline);
	status = CMS_CONFIG_ERROR;
	free(bufline);
	free(procline);
	return;
    }

    /* Use the words from the buffer line to initialize some class variables. 
     */
    rtapi_strxcpy(BufferName, word[1]);
    rcs_print_debug(PRINT_CMS_CONSTRUCTORS, "new CMS (%s)\n", BufferName);

    /* Clear errno so we can determine if all of the parameters in the */
    /* buffer line were in an acceptable form. */
    if (errno == ERANGE) {
	errno = 0;
    }
    char *realname = cms_check_for_host_alias(word[3]);
    if (realname == NULL) {
	rtapi_strxcpy(BufferHost, word[3]);
    } else {
	rtapi_strxcpy(BufferHost, realname);
    }

    buffer_type_name = word[2];

    /* strtol should allow us to use the C syntax for specifying the radix of 
       the numbers in the configuration file. (i.e. 0x???? for hexadecimal,
       0??? for octal and ???? for decimal.) */
    size = (long) strtol(word[4], (char **) NULL, 0);
    neutral = (int) strtol(word[5], (char **) NULL, 0);
    rpc_program_number = strtol(word[6], (char **) NULL, 0);
    buffer_number = strtol(word[7], (char **) NULL, 0);
    total_connections = strtol(word[8], (char **) NULL, 0);
    free_space = size;

    /* Check errno to see if all of the strtol's were successful. */
    if (ERANGE == errno) {
	rcs_print_error("CMS: Error in buffer line from config file.\n");
	rcs_print_error("%s\n", bufline);
	status = CMS_CONFIG_ERROR;
	free(bufline);
	free(procline);
	return;
    }

    /* Determine the BufferType. */
    if (!strcmp(buffer_type_name, "SHMEM")) {
	BufferType = CMS_SHMEM_TYPE;
    } else if (!strcmp(buffer_type_name, "PHANTOM")) {
	BufferType = CMS_PHANTOM_BUFFER;
	is_phantom = 1;
    } else if (!strcmp(buffer_type_name, "LOCMEM")) {
	BufferType = CMS_LOCMEM_TYPE;
    } else if (!strcmp(buffer_type_name, "FILEMEM")) {
	BufferType = CMS_FILEMEM_TYPE;
    } else {
	rcs_print_error("CMS: invalid buffer type (%s)\n", buffer_type_name);
	status = CMS_CONFIG_ERROR;
	free(bufline);
	free(procline);
	return;
    }

    int num_words = separate_words(word, 32, buflineupper);
    if (num_words < 8) {
	rcs_print_error("CMS: Error in buffer line from config file.\n");
	rcs_print_error("%s\n", bufline);
	status = CMS_CONFIG_ERROR;
	free(bufline);
	free(procline);
	return;
    }
    for (i = 8; i < num_words && i < 32; i++) {
	if (word[i] == NULL) {
	    break;
	}

	if (!strcmp(word[i], "QUEUE")) {
	    queuing_enabled = 1;
	    continue;
	}

	if (!strcmp(word[i], "DIAG")) {
	    enable_diagnostics = 1;
	    continue;
	}

	if (!strcmp(word[i], "SPLIT")) {
	    split_buffer = 1;
	    continue;
	}
	if (!strcmp(word[i], "DISP")) {
	    neutral_encoding_method = CMS_DISPLAY_ASCII_ENCODING;
	    continue;
	}
	if (!strcmp(buflineupper, "ASCII")) {
	    neutral_encoding_method = CMS_ASCII_ENCODING;
	    continue;
	}
	if (!strcmp(buflineupper, "XDR")) {
	    neutral_encoding_method = CMS_XDR_ENCODING;
	    continue;
	}

	char *port_string;
	if (NULL != (port_string = strstr(word[i], "STCP="))) {
	    remote_port_type = CMS_STCP_REMOTE_PORT_TYPE;
	    stcp_port_number =
		(int) strtol(port_string + 5, (char **) NULL, 0);
	    continue;
	} else if (NULL != (port_string = strstr(word[i], "TCP="))) {
	    remote_port_type = CMS_TCP_REMOTE_PORT_TYPE;
	    tcp_port_number =
		(int) strtol(port_string + 4, (char **) NULL, 0);
	    continue;
	} else if (NULL != (port_string = strstr(word[i], "UDP="))) {
	    remote_port_type = CMS_UDP_REMOTE_PORT_TYPE;
	    udp_port_number =
		(int) strtol(port_string + 4, (char **) NULL, 0);
	    continue;
	}

	char *version_string;
	if (NULL != (version_string = strstr(word[i], "VERSION="))) {
	    min_compatible_version =
		strtod(version_string + 8, (char **) NULL);
	    continue;
	}

	char *subdiv_string;
	if (NULL != (subdiv_string = strstr(word[i], "SUBDIV="))) {
	    total_subdivisions = strtol(subdiv_string + 7, (char **) NULL, 0);
	    subdiv_size = size / total_subdivisions;
	    subdiv_size -= subdiv_size % 4;
	    continue;
	}

	char *enc_max_string;
	if (NULL != (enc_max_string = strstr(word[i], "ENC_MAX_SIZE="))) {
	    enc_max_size = strtoul(enc_max_string + 13, (char **) NULL, 0);
	    continue;
	}

	if (!strcmp(word[i], "SERIAL")) {
	    serial = 1;
	    continue;
	}

	if (!strcmp(word[i], "CONFIRM_WRITE")) {
	    confirm_write = 1;
	    continue;
	}
	if (!strcmp(word[i], "FORCE_RAW")) {
	    force_raw = 1;
	    continue;
	}
	if (!strcmp(word[i], "AUTOCNUM")) {
	    use_autokey_for_connection_number = 1;
	    continue;
	}
    }

    /* Get parameters from the process's line in the config file. */
    if (use_autokey_for_connection_number) {
	if (separate_words(word, 9, procline) != 9) {
	    rcs_print_error
		("CMS: Error parsing process line from config file.\n");
	    rcs_print_error("%s\n", procline);
	    status = CMS_CONFIG_ERROR;
	    free(bufline);
	    free(procline);
	    return;
	}
    } else {
	if (separate_words(word, 10, procline) != 10) {
	    rcs_print_error
		("CMS: Error parsing process line from config file.\n");
	    rcs_print_error("%s\n", procline);
	    status = CMS_CONFIG_ERROR;
	    free(bufline);
	    free(procline);
	    return;
	}
    }
    /* Clear errno so we can determine if all of the parameters in the */
    /* buffer line were in an acceptable form. */
    if (errno == ERANGE) {
	errno = 0;
    }

    rtapi_strxcpy(ProcessName, word[1]);
    rtapi_strxcpy(ProcessHost, word[4]);

    /* Clear errno so we can determine if all of the parameters in the */
    /* buffer line were in an acceptable form. */
    if (errno == ERANGE) {
	errno = 0;
    }

    proc_type_name = word[3];
    rtapi_strxcpy(PermissionString, word[5]);
    spawn_server = atoi(word[6]);

    /* Compute timeout. */
    if (!strcmp(word[7], "INF")) {	/* Never Time Out. */
	timeout = -1;
    } else {
	timeout = strtod(word[7], (char **) NULL);
    }

    is_local_master = (int) atol(word[8]);

    if (!use_autokey_for_connection_number) {

	connection_number = atol(word[9]);

	if (total_connections <= connection_number) {
	    rcs_print_error
		("CMS: connection number(%lu) must be less than total connections (%lu).\n",
		connection_number, total_connections);
	    status = CMS_CONFIG_ERROR;
	    free(bufline);
	    free(procline);
	    return;
	}
    }
    /* Check errno to see if all of the strtol's were successful. */
    if (ERANGE == errno) {
	rcs_print_error("CMS: Error in proc line from config file.\n");
	rcs_print_error("%s\n", procline);
	status = CMS_CONFIG_ERROR;
	free(bufline);
	free(procline);
	return;
    }

    if (set_to_server < 0) {
	isserver = 0;
    } else if (set_to_server > 0) {
	isserver = 1;
    } else {
	isserver = (spawn_server == 1);
    }

    /* Determine the ProcessType. */
    switch (cms_connection_mode) {
    case CMS_NORMAL_CONNECTION_MODE:
	if (!strcmp(proc_type_name, "REMOTE")) {
	    ProcessType = CMS_REMOTE_TYPE;
	    spawn_server = 0;
	} else if (!strcmp(proc_type_name, "LOCAL")) {
	    ProcessType = CMS_LOCAL_TYPE;
	} else if (!strcmp(proc_type_name, "AUTO")) {
	    if (hostname_matches_bufferline(BufferLine)) {
		ProcessType = CMS_LOCAL_TYPE;
	    } else {
		ProcessType = CMS_REMOTE_TYPE;
		spawn_server = 0;
	    }
	} else if (!strcmp(proc_type_name, "PHANTOM")) {
	    ProcessType = CMS_PHANTOM_USER;
	    spawn_server = 0;
	    is_phantom = 1;
	} else {
	    rcs_print_error("CMS: invalid process type (%s)/n",
		proc_type_name);
	    status = CMS_CONFIG_ERROR;
	    return;
	}
	break;

    case CMS_FORCE_LOCAL_CONNECTION_MODE:
	ProcessType = CMS_LOCAL_TYPE;
	break;

    case CMS_FORCE_REMOTE_CONNECTION_MODE:
	ProcessType = CMS_REMOTE_TYPE;
	break;

    }

    /* Set flags to make sure ops section of config file is correct. */
    if (NULL != strchr(PermissionString, 'R')) {
	read_permission_flag = 1;
    } else {
	read_permission_flag = 0;
    }

    if (NULL != strchr(PermissionString, 'W')) {
	write_permission_flag = 1;
    } else {
	write_permission_flag = 0;
    }
    if (isserver) {
	read_permission_flag = 1;
	write_permission_flag = 1;
    }

    mode = CMS_NOT_A_MODE;	/* Make sure user sets the mode before using. 
				 */

    // Search the end of the bufferline for key words.

    if (NULL != strstr(ProcessLine, "serialPortDevName=")) {
	remote_port_type = CMS_TTY_REMOTE_PORT_TYPE;
    }
    if (min_compatible_version < 3.44 && min_compatible_version > 0) {
	total_subdivisions = 1;
    }
    if (queuing_enabled && split_buffer) {
	rcs_print_error("CMS: Can not split buffer with queuing enabled.\n");
	status = CMS_CONFIG_ERROR;
	return;
    }
    if (min_compatible_version > 3.39 || min_compatible_version <= 0.0) {
	if (neutral_encoding_method == CMS_ASCII_ENCODING) {
	    neutral_encoding_method = CMS_DISPLAY_ASCII_ENCODING;
	}
    }

    if (min_compatible_version <= 3.71 && min_compatible_version >= 1e-6) {
        rcs_print("NO DIAGNOSTICS\n");
	enable_diagnostics = 0;
    }

    open();			/* Allocate memory and initialize XDR streams */
    if (enable_diagnostics) {
	setup_diag_proc_info();
    }
    free(bufline);
    free(procline);
}

/* Function for allocating memory and initializing XDR streams, which */
/*    is called from both CMS constructors.                           */
void CMS::open(void)
{
    int encode_header_ret;
    int encode_queuing_header_ret;

    /* Clear some status checking variables. */
    status = CMS_STATUS_NOT_SET;

    /* Set all the pointers to null before requesting memory so that we only */
    /* free successfully allocated memory. */
    data = NULL;
    subdiv_data = NULL;
    encoded_data = NULL;
    encoded_header = NULL;
    encoded_queuing_header = NULL;
    encoded_header_size = 0;
    updater = (CMS_UPDATER *) NULL;
    normal_updater = (CMS_UPDATER *) NULL;
    temp_updater = (CMS_UPDATER *) NULL;
    last_im = CMS_NOT_A_MODE;
    pointer_check_disabled = 0;

    dummy_handle = (PHYSMEM_HANDLE *) NULL;

    /* Initialize some debug variables. */
    first_read_done = 0;
    first_write_done = 0;
    total_messages_missed = 0;
    messages_missed_on_last_read = 0;
    format_low_ptr = (char *) NULL;
    format_high_ptr = (char *) NULL;
    header.was_read = 0;
    header.write_id = 0;
    header.in_buffer_size = 0;

    number_of_cms_objects++;	/* Increment the static variable.  */
    /* Save some memory and time if this is a PHANTOMMEM object. */
    if (!is_phantom) {
	/* Allocate memory for the local copy of global buffer. */
	data = malloc(size);
	memset(data, 0, size);
	subdiv_data = data;
	if (force_raw) {
	    encoded_data = data;
	}
	rcs_print_debug(PRINT_CMS_CONSTRUCTORS, "%p = data = calloc(%lu,1);\n",
	    data, size);
	/* Check to see if allocating memory was successful. */
	if (data == NULL) {
	    rcs_print_error("CMS: Can't allocate memory for local buffer.\n");
	    status = CMS_CREATE_ERROR;
	    return;
	}
    }
    if (isserver || neutral || ((ProcessType == CMS_REMOTE_TYPE) && !force_raw)) {
	switch (neutral_encoding_method) {
	case CMS_XDR_ENCODING:
	    updater = new CMS_XDR_UPDATER(this);
	    break;

	case CMS_ASCII_ENCODING:
	    updater = new CMS_ASCII_UPDATER(this);
	    break;

	case CMS_DISPLAY_ASCII_ENCODING:
	    updater = new CMS_DISPLAY_ASCII_UPDATER(this);
	    break;

	default:
	    updater = (CMS_UPDATER *) NULL;
	    status = CMS_UPDATE_ERROR;
	    rcs_print_error("CMS: Invalid encoding method(%d)\n",
		neutral_encoding_method);
	    break;
	}
	normal_updater = updater;
	if (((int) status) < 0) {
	    return;
	}
	/* Find out what size the header is after it has been encoded. */
	if ((encode_header_ret = encode_header()) == -1) {
	    rcs_print_error("CMS:Error encoding CMS header.\n");
	    status = CMS_MISC_ERROR;
	    return;
	}
	encoded_header_size = (long) encode_header_ret;
	if (min_compatible_version <= 0.0 || min_compatible_version > 3.29) {
	    if (neutral_encoding_method == CMS_DISPLAY_ASCII_ENCODING) {
		encoded_header_size = 16;
	    }
	}

	if (queuing_enabled) {
	    /* Initialize queuing header to avoid test center error message. */
	    memset(&queuing_header, 0, sizeof(queuing_header));

	    /* Find out what size the queuing_header is after being encoded. */
	    if ((encode_queuing_header_ret = encode_queuing_header()) == -1) {
		rcs_print_error("CMS:Error encoding CMS queuing_header.\n");
		status = CMS_MISC_ERROR;
		return;
	    }
	    encoded_queuing_header_size = (long) encode_queuing_header_ret;
	}
    }

    if (split_buffer && total_subdivisions > 1) {
	rcs_print_error
	    ("Can't split buffer and use subdivisions. (total_subsivisions=%d)",
	    total_subdivisions);
	status = CMS_MISC_ERROR;
	return;
    }

    int nfactor = 4;
    if (NULL != updater) {
	nfactor = updater->neutral_size_factor;
    }

    /* Set some variables to let the user know how much space is left. */
    size_without_diagnostics = size;
    diag_offset = 0;
    if (enable_diagnostics) {
	diag_offset = (sizeof(CMS_DIAG_HEADER) +
	    (total_connections * sizeof(CMS_DIAG_PROC_INFO)));
	size_without_diagnostics -= diag_offset;
    }
    skip_area = 0;
    half_offset = (size_without_diagnostics / 2);
    half_size = (size_without_diagnostics / 2);
    fast_mode = 0;
    if (split_buffer) {
	if (neutral) {
	    subdiv_size = (size_without_diagnostics / 2) - total_connections;
	    subdiv_size -= (subdiv_size % 4);
	    max_message_size =
		(size_without_diagnostics / 2) - total_connections -
		encoded_header_size - 2;
	    max_encoded_message_size =
		size_without_diagnostics - total_connections -
		encoded_header_size;
	    guaranteed_message_space =
		max_message_size / cms_encoded_data_explosion_factor;
	} else {
	    if (ProcessType == CMS_REMOTE_TYPE) {
		subdiv_size =
		    (size_without_diagnostics / 2) - total_connections;
		subdiv_size -= (subdiv_size % 4);
		max_message_size =
		    (size_without_diagnostics / 2) - total_connections -
		    sizeof(CMS_HEADER) - 2;
		max_encoded_message_size = nfactor * max_message_size;
		guaranteed_message_space = max_message_size / nfactor;
	    } else {
		subdiv_size =
		    (size_without_diagnostics / 2) - total_connections;
		subdiv_size -= (subdiv_size % 4);
		max_message_size =
		    (size_without_diagnostics / 2) - total_connections -
		    sizeof(CMS_HEADER) - 2;
		max_encoded_message_size = nfactor * max_message_size;
		guaranteed_message_space = max_message_size;
	    }
	}
    } else {
	if (neutral) {
	    subdiv_size =
		(size_without_diagnostics -
		total_connections) / total_subdivisions;
	    subdiv_size -= (subdiv_size % 4);
	    max_message_size = subdiv_size - encoded_header_size;
	    max_encoded_message_size = subdiv_size - encoded_header_size;
	    guaranteed_message_space = max_message_size / nfactor;
	} else {
	    if (ProcessType == CMS_REMOTE_TYPE) {
		subdiv_size =
		    (size_without_diagnostics -
		    total_connections) / total_subdivisions;
		subdiv_size -= (subdiv_size % 4);
		max_message_size = subdiv_size - sizeof(CMS_HEADER);
		max_encoded_message_size = nfactor * max_message_size;
		guaranteed_message_space = max_message_size / nfactor;
	    } else {
		subdiv_size =
		    (size_without_diagnostics -
		    total_connections) / total_subdivisions;
		subdiv_size -= (subdiv_size % 4);
		max_message_size = subdiv_size - sizeof(CMS_HEADER);
		max_encoded_message_size = nfactor * max_message_size;
		guaranteed_message_space = max_message_size;
	    }
	}
    }
    if (enc_max_size > 0 && enc_max_size < max_encoded_message_size) {
	max_encoded_message_size = enc_max_size;
    }

    if ((neutral || ProcessType == CMS_REMOTE_TYPE) && !isserver) {
	/* Local processes that are use a neutral buffer and */
	/* All remote processes. */
	read_mode = CMS_DECODE;
	read_updater_mode = CMS_DECODE_DATA;
	write_mode = CMS_ENCODE;
	write_updater_mode = CMS_ENCODE_DATA;
    } else if (!neutral && isserver && !force_raw) {
	/* Servers. */
	read_mode = CMS_ENCODE;
	read_updater_mode = CMS_ENCODE_DATA;
	write_mode = CMS_DECODE;
	write_updater_mode = CMS_DECODE_DATA;
    } else {
	/* Everybody else. */
	read_mode = CMS_RAW_OUT;
	read_updater_mode = CMS_NO_UPDATE;
	write_mode = CMS_RAW_IN;
	write_updater_mode = CMS_NO_UPDATE;
    }
}

/* Set the area used for the encoded data buffer, and initialize the */
 /* XDR streams to use this area. */
/* This function is called from open, which is called by the constructor */
 /* and by one of the CMS_SERVER functions. */
/* _encoded_data should point to an area of memory at least cms_encoded_data_explosion_factor*size .*/
void CMS::set_encoded_data(void *_encoded_data, long _encoded_data_size)
{
    if (force_raw) {
	if (NULL != data && data != _encoded_data) {
	    free(data);
	}
	data = encoded_data = _encoded_data;
	encoded_data_size = size;
	subdiv_data = data;
	using_external_encoded_data = 1;
    } else {
	if (max_encoded_message_size > _encoded_data_size) {
	    max_encoded_message_size = _encoded_data_size;
	}
	if (NULL != updater) {
	    updater->set_encoded_data(_encoded_data, _encoded_data_size);
	}
	if (NULL != _encoded_data) {
	    memset(_encoded_data, 0, max_encoded_message_size);
	}
	using_external_encoded_data = 1;
    }
}

/* Destructor */
CMS::~CMS()
{
    rcs_print_debug(PRINT_CMS_DESTRUCTORS, "deleting CMS (%s)\n", BufferName);

    if (NULL != updater) {
	delete updater;
	updater = (CMS_UPDATER *) NULL;
    }

    /* Free the memory used for the local copy of the global buffer. */
    if (NULL != data && (!force_raw || !using_external_encoded_data)) {
	rcs_print_debug(PRINT_CMS_DESTRUCTORS, "free( data = %p);\n", data);
	free(data);
	data = NULL;
	if (force_raw) {
	    encoded_data = NULL;
	}
    }
    number_of_cms_objects--;

    if (NULL != dummy_handle) {
	delete dummy_handle;
	dummy_handle = (PHYSMEM_HANDLE *) NULL;
    }
    rcs_print_debug(PRINT_CMS_DESTRUCTORS, "Leaving ~CMS()\n");
}

/* This function should never be called. It exists so that classes  which */
 /* overload read, write etc don't have to bother creating it. */
CMS_STATUS CMS::main_access(void *_local, int * /*serial_number*/)
{
    rcs_print_error("CMS::main_access called by %s for %s.\n",
	ProcessName, BufferName);
    rcs_print_error("This should never happen.\n");
    rcs_print_error
	("Derived classes should either override main_access() or\n");
    rcs_print_error("the functions that call it.(read(), write(), etc.)\n");
    rcs_print_error("_local = %p\n", _local);
    return (CMS_MISC_ERROR);
}

/* General Utility Functions. */

/* Check the buffer id against in_buffer_id to see if it is new. */
CMS_STATUS CMS::check_id(CMSID id)
{
    if (status < 0) {
	return (status);
    }

    if (0 == id) {
	messages_missed_on_last_read = 0;
	in_buffer_id = 0;
	return (status = CMS_READ_OLD);
    }

    if (id == in_buffer_id) {
	status = CMS_READ_OLD;
	messages_missed_on_last_read = 0;
    } else {
	if (split_buffer) {
	    if (id == last_id_side0 || id == last_id_side1) {
		status = CMS_READ_OLD;
		messages_missed_on_last_read = 0;
		return (status);
	    }
	    if (toggle_bit) {
		last_id_side0 = id;
	    } else {
		last_id_side1 = id;
	    }
	}
	status = CMS_READ_OK;
	messages_missed_on_last_read = id - in_buffer_id - 1;
	if (messages_missed_on_last_read < 0) {
	    messages_missed_on_last_read = 0;
	}
	total_messages_missed += messages_missed_on_last_read;
	in_buffer_id = id;
    }
    return (status);
}

void CMS::clean_buffers()
{
    in_buffer_id = 0;
    last_id_side0 = 0;
    last_id_side1 = 0;
    if (NULL != data) {
	memset(data, 0, size);
    }
    if (NULL != encoded_data) {
	memset(encoded_data, 0, max_encoded_message_size);
    }
}

/* Read and Write interface functions call appropriate virtual function. */
CMS_STATUS CMS::clear()
{
    in_buffer_id = 0;
    last_id_side0 = 0;
    last_id_side1 = 0;
    status = CMS_STATUS_NOT_SET;
    internal_access_type = CMS_CLEAR_ACCESS;
    main_access(data);
    return (status);
}

int CMS::check_if_read()
{
    internal_access_type = CMS_CHECK_IF_READ_ACCESS;
    status = CMS_STATUS_NOT_SET;
    main_access(data);
    return ((int) header.was_read);
}

int CMS::get_queue_length()
{
    internal_access_type = CMS_GET_QUEUE_LENGTH_ACCESS;
    status = CMS_STATUS_NOT_SET;
    if (!queuing_enabled) {
	return 0;
    }
    main_access(data);
    return ((int) queuing_header.queue_length);
}

int CMS::get_space_available()
{
    internal_access_type = CMS_GET_SPACE_AVAILABLE_ACCESS;
    status = CMS_STATUS_NOT_SET;
    if (!queuing_enabled) {
	return size;
    }
    main_access(data);
    return ((int) free_space);
}

CMS_STATUS CMS::read()
{
    internal_access_type = CMS_READ_ACCESS;
    status = CMS_STATUS_NOT_SET;
    blocking_timeout = 0;
    main_access(data);
    return (status);
}

CMS_STATUS CMS::blocking_read(double _blocking_timeout)
{
    status = CMS_STATUS_NOT_SET;
    internal_access_type = CMS_READ_ACCESS;
    blocking_timeout = _blocking_timeout;
    main_access(data);
    return (status);
}

void CMS::disconnect()
{
}

void CMS::reconnect()
{
}

CMS_STATUS CMS::peek()
{
    internal_access_type = CMS_PEEK_ACCESS;
    status = CMS_STATUS_NOT_SET;
    blocking_timeout = 0;
    main_access(data);
    return (status);
}

CMS_STATUS CMS::write(void *user_data, int *serial_number)
{
    internal_access_type = CMS_WRITE_ACCESS;
    status = CMS_STATUS_NOT_SET;
    main_access(user_data, serial_number);
    return (status);
}

CMS_STATUS CMS::write_if_read(void *user_data, int *serial_number)
{
    internal_access_type = CMS_WRITE_IF_READ_ACCESS;
    status = CMS_STATUS_NOT_SET;
    main_access(user_data, serial_number);
    return (status);
}

// For protocols that provide No security, tell the
// application the login was successful.
// This method needs to be overloaded to have any security.
int CMS::login(const char * /*name*/, const char * /*passwd*/)
{
    return 1;
}

/* Function to set the mode to appropriate read or write mode. */
void CMS::set_mode(CMSMODE im)
{
    status = CMS_STATUS_NOT_SET;
    if (last_im == im) {
	return;
    }
    if (!force_raw) {
	if (CMS_WRITE == im) {
	    mode = write_mode;
	    if (NULL != updater) {
		updater->set_mode((CMS_UPDATER_MODE) write_updater_mode);
	    }
	    last_im = im;
	    return;
	}
	if (CMS_READ == im) {
	    mode = read_mode;
	    if (NULL != updater) {
		updater->set_mode((CMS_UPDATER_MODE) read_updater_mode);
	    }
	    last_im = im;
	    return;
	}
	if (CMS_DECODE == im) {
	    mode = CMS_DECODE;
	    if (NULL != updater) {
		updater->set_mode(CMS_DECODE_DATA);
	    }
	}
	if (CMS_ENCODE == im) {
	    mode = CMS_ENCODE;
	    if (NULL != updater) {
		updater->set_mode(CMS_ENCODE_DATA);
	    }
	}
    }
    last_im = im;
    mode = im;
}

/* Functions for changing/restoring the updator type. */
void CMS::set_temp_updater(CMS_NEUTRAL_ENCODING_METHOD temp_encoding_method)
{
    if (force_raw) {
	return;
    }
    if (temp_updater_encoding_method != temp_encoding_method &&
	NULL != temp_updater) {
	delete temp_updater;
	temp_updater = (CMS_UPDATER *) NULL;
    }
    if (NULL == temp_updater) {
	switch (temp_encoding_method) {
	case CMS_XDR_ENCODING:
	    temp_updater = new CMS_XDR_UPDATER(this);
	    break;

	case CMS_ASCII_ENCODING:
	    temp_updater = new CMS_ASCII_UPDATER(this);
	    break;

	case CMS_DISPLAY_ASCII_ENCODING:
	    temp_updater = new CMS_DISPLAY_ASCII_UPDATER(this);
	    break;

	default:
	    temp_updater = (CMS_UPDATER *) NULL;
	    status = CMS_UPDATE_ERROR;
	    rcs_print_error("CMS: Invalid encoding method(%d)\n",
		neutral_encoding_method);
	    break;
	}
    }
    if (NULL != temp_updater) {
	updater = temp_updater;
	temp_updater_encoding_method = temp_encoding_method;
    }
}

void CMS::restore_normal_updater()
{
    updater = normal_updater;
}

/* Updater Positioning Functions. */
void CMS::rewind()
{
    if (force_raw) {
	return;
    }
    if (NULL != updater) {
	updater->rewind();
    }
}

/* XDR routines for accessing an encoded header. */

int CMS::encode_header()
{
    if (force_raw) {
	return 0;
    }
    if (NULL == updater) {
	return -1;
    }
    CMS_UPDATER_MODE original_mode;
    original_mode = updater->get_mode();
    format_low_ptr = (char *) &header;
    format_high_ptr = ((char *) &header) + sizeof(CMS_HEADER);
    updater->set_mode(CMS_ENCODE_HEADER);
    updater->rewind();
    updater->update(header.was_read);
    updater->update(header.write_id);
    updater->update(header.in_buffer_size);
    if (status == CMS_UPDATE_ERROR || status == CMS_MISC_ERROR) {
	return (-1);
    }
    encoded_header_size = updater->get_encoded_msg_size();
    if (min_compatible_version <= 0.0 || min_compatible_version > 3.29) {
	if (neutral_encoding_method == CMS_DISPLAY_ASCII_ENCODING) {
	    encoded_header_size = 16;
	}
    }
    updater->set_mode(original_mode);
    return (encoded_header_size);
}

int CMS::decode_header()
{
    if (force_raw) {
	return 0;
    }
    if (NULL == updater) {
	return -1;
    }
    CMS_UPDATER_MODE original_mode = updater->get_mode();
    format_low_ptr = (char *) &header;
    format_high_ptr = ((char *) &header) + sizeof(CMS_HEADER);
    updater->set_mode(CMS_DECODE_HEADER);
    updater->rewind();
    updater->update(header.was_read);
    updater->update(header.write_id);
    updater->update(header.in_buffer_size);
    updater->set_mode(original_mode);
    return ((int) (status != CMS_UPDATE_ERROR
	    && status != CMS_MISC_ERROR) ? 0 : -1);
}

int CMS::encode_queuing_header()
{
    if (force_raw) {
	return 0;
    }
    if (NULL == updater) {
	return -1;
    }
    CMS_UPDATER_MODE original_mode = updater->get_mode();
    format_low_ptr = (char *) &queuing_header;
    format_high_ptr = ((char *) &queuing_header) + sizeof(CMS_QUEUING_HEADER);
    updater->set_mode(CMS_ENCODE_QUEUING_HEADER);
    updater->rewind();
    updater->update(queuing_header.head);
    updater->update(queuing_header.tail);
    updater->update(queuing_header.queue_length);
    updater->update(queuing_header.end_queue_space);
    updater->update(queuing_header.write_id);
    if (status == CMS_UPDATE_ERROR || status == CMS_MISC_ERROR) {
	return (-1);
    }
    encoded_queuing_header_size = updater->get_encoded_msg_size();
    if (min_compatible_version <= 0.0 || min_compatible_version > 3.29) {
	if (neutral_encoding_method == CMS_DISPLAY_ASCII_ENCODING) {
	    encoded_queuing_header_size = 24;
	}
    }
    updater->set_mode(original_mode);
    return (encoded_queuing_header_size);
}

int CMS::decode_queuing_header()
{
    if (force_raw) {
	return 0;
    }
    if (NULL == updater) {
	return -1;
    }
    CMS_UPDATER_MODE original_mode = updater->get_mode();
    format_low_ptr = (char *) &queuing_header;
    format_high_ptr = ((char *) &queuing_header) + sizeof(CMS_QUEUING_HEADER);
    updater->set_mode(CMS_DECODE_QUEUING_HEADER);
    updater->rewind();
    updater->update(queuing_header.head);
    updater->update(queuing_header.tail);
    updater->update(queuing_header.queue_length);
    updater->update(queuing_header.end_queue_space);
    updater->update(queuing_header.write_id);
    updater->set_mode(original_mode);
    return ((int) (status != CMS_UPDATE_ERROR
	    && status != CMS_MISC_ERROR) ? 0 : -1);
}

int CMS::get_encoded_msg_size()
{
    if (force_raw) {
	return 0;
    }
    if (NULL == updater) {
	return (-1);
    }
    return (header.in_buffer_size = updater->get_encoded_msg_size());
}

int CMS::check_pointer(char *ptr, long bytes)
{
    if (force_raw) {
	return 0;
    }
    if (NULL == format_low_ptr || NULL == format_high_ptr
	|| pointer_check_disabled) {
	return 0;
    }
    if (ptr < format_low_ptr || ptr > (format_high_ptr - bytes)) {
	rcs_print_error("CMS: pointer %p to %ld bytes out of range %p to %p\n",
	    ptr, bytes, format_low_ptr, format_high_ptr);
	rcs_print_error("CMS: Check buffer and message sizes.\n");
	status = CMS_UPDATE_ERROR;
	return -1;
    }
    format_size = (long) (ptr - format_low_ptr) + bytes;
    return 0;
}

void CMS::set_cms_status(CMS_STATUS new_status)
{
    status = new_status;
}

  /* Access functions for primitive C language data types */
CMS_STATUS CMS::update(bool &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(char &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(unsigned char &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(short int &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(unsigned short int &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(int &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(unsigned int &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(long int &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(unsigned long int &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(float &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(double &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(long double &x)
{
    if (NULL != updater) {
	return (updater->update(x));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(char *x, unsigned int len)
{
    if (NULL != updater) {
	return (updater->update(x, len));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(unsigned char *x, unsigned int len)
{
    if (NULL != updater) {
	return (updater->update(x, len));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(short *x, unsigned int len)
{
    if (NULL != updater) {
	return (updater->update(x, len));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(unsigned short *x, unsigned int len)
{
    if (NULL != updater) {
	return (updater->update(x, len));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(int *x, unsigned int len)
{
    if (NULL != updater) {
	return (updater->update(x, len));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(unsigned int *x, unsigned int len)
{
    if (NULL != updater) {
	return (updater->update(x, len));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(long *x, unsigned int len)
{
    if (NULL != updater) {
	return (updater->update(x, len));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(unsigned long *x, unsigned int len)
{
    if (NULL != updater) {
	return (updater->update(x, len));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(float *x, unsigned int len)
{
    if (NULL != updater) {
	return (updater->update(x, len));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(double *x, unsigned int len)
{
    if (NULL != updater) {
	return (updater->update(x, len));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

CMS_STATUS CMS::update(long double *x, unsigned int len)
{
    if (NULL != updater) {
	return (updater->update(x, len));
    } else {
	return (status = CMS_UPDATE_ERROR);
    }
}

const char *CMS::status_string(int status_type)
{
    switch (status_type) {
	/* ERROR conditions */
    case CMS_MISC_ERROR:
	return ("CMS_MISC_ERROR:   A miscellaneous  error occurred.");

    case CMS_UPDATE_ERROR:
	return ("CMS_UPDATE_ERROR: An error occurred during an update. ");

    case CMS_INTERNAL_ACCESS_ERROR:
	return
	    ("CMS_INTERNAL_ACCESS_ERROR: An error occurred during an internal access function. ");

    case CMS_NO_MASTER_ERROR:
	return
	    ("CMS_NO_MASTER_ERROR: An error occurred because the master was not started.");

    case CMS_CONFIG_ERROR:
	return ("CMS_CONFIG_ERROR: There was an error in the configuration.");

    case CMS_TIMED_OUT:
	return ("CMS_TIMED_OUT: operation timed out.");

    case CMS_QUEUE_FULL:
	return
	    ("CMS_QUEUE_FULL:=  A write failed because queuing was enabled but there was no room to add to the queue. ");

    case CMS_CREATE_ERROR:
	return
	    ("CMS_CREATE_ERROR: Something could not be created because we were out of memory or another system resource.");

    case CMS_PERMISSIONS_ERROR:
	return ("CMS_PERMISSIONS_ERROR: Problem with permissions.");

	/* NON Error Conditions. */
    case CMS_STATUS_NOT_SET:
	return
	    ("CMS_STATUS_NOT_SET: The status variable has not been set yet.");

    case CMS_READ_OLD:
	return ("CMS_READ_OLD:  Read successful, but data is old. \n");

    case CMS_READ_OK:
	return ("CMS_READ_OK: Read successful so far.");

    case CMS_WRITE_OK:
	return ("CMS_WRITE_OK:  Write successful so far. ");

    case CMS_WRITE_WAS_BLOCKED:
	return
	    ("CMS_WRITE_WAS_BLOCKED: Write if read did not succeed, because the buffer had not been read yet.");

    case CMS_CLEAR_OK:
	return ("CMS_CLEAR_OK: A clear operation was successful.");

    case CMS_CLOSED:
	return ("CMS_CLOSED: The channel has been closed.");

    case CMS_NO_SERVER_ERROR:
	return
	    (" CMS_NO_SERVER_ERROR: The server has not been started or could not be contacted.");

    case CMS_RESOURCE_CONFLICT_ERROR:
	return
	    ("CMS_RESOURCE_CONFLICT_ERROR: Two or more CMS buffers are trying to use the same resource.");

    case CMS_NO_IMPLEMENTATION_ERROR:
	return
	    ("CMS_NO_IMPLEMENTATION_ERROR: An operation was attempted which has not yet been implemented for the current platform or protocol.");

    case CMS_INSUFFICIENT_SPACE_ERROR:
	return
	    ("CMS_INSUFFICIENT_SPACE_ERROR: The size of the buffer was insufficient for the requested operation.");

    case CMS_LIBRARY_UNAVAILABLE_ERROR:
	return
	    ("CMS_LIBRARY_UNAVAILABLE_ERROR: A DLL or Shared Object library needed for the current protocol could not be found or initialized.");

    case CMS_SERVER_SIDE_ERROR:
	return ("CMS_SERVER_SIDE_ERROR: The server reported an error.");

    case CMS_NO_BLOCKING_SEM_ERROR:
	return
	    ("CMS_NO_BLOCKING_SEM_ERROR: A blocking_read operartion was tried but no semaphore for the blocking was configured or available.");

    default:
	return ("UNKNOWN");
    }
}

int CMS::set_subdivision(int _subdiv)
{
    if (_subdiv < 0 || _subdiv > total_subdivisions) {
	return -1;
    }
    current_subdivision = _subdiv;
    subdiv_data = ((char *) data) + _subdiv * (subdiv_size);
    return (0);
}

int
  CMS::get_msg_count()
{
    internal_access_type = CMS_GET_MSG_COUNT_ACCESS;
    status = CMS_STATUS_NOT_SET;
    blocking_timeout = 0;
    main_access(data);
    return (header.write_id);
}

char *cms_check_for_host_alias(char *in)
{
    if (NULL == in) {
	return NULL;
    }
    if (NULL == cmsHostAliases) {
	return NULL;
    }
    CMS_HOST_ALIAS_ENTRY *entry =
	(CMS_HOST_ALIAS_ENTRY *) cmsHostAliases->get_head();
    while (NULL != entry) {
	if (!strncmp(entry->alias, in, 64)) {
	    return entry->host;
	}
	entry = (CMS_HOST_ALIAS_ENTRY *) cmsHostAliases->get_next();
    }
    return NULL;
}
