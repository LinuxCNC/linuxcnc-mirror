/********************************************************************
* Description: cms_cfg.cc
*   C++ file for the  Communication Management System (CMS).
*   Provides cms_config -- CMS function which reads configuration file.
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

extern int verbose_nml_error_messages;

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>		/* sscanf(), NULL FILE, fopen(), fgets() */
#include <unistd.h>		/* gethostname() */
#include <string.h>		/* strcpy(), strlen(),memcpy()
				   strcmp(),strchr() */
#include <errno.h>		/* errno */
#include <ctype.h>		/* toupper(), tolower() */
#include <netdb.h>
#include <arpa/inet.h>		/* inet_ntoa */
#include <netinet/in.h>		/* sockaddr_in */
#include <stdlib.h>

#ifdef __cplusplus
}
#endif
#include <rtapi_string.h>
#include "cms.hh"		/* class CMS */
#include "cms_cfg.hh"

 /* TCP stands for "Transmission Control Protocol". This is the recommended
    method for remote connection, for most applications. It is more reliable
    and can handle larger messages than UDP and is more widely available than 
    RPC. */
#include "tcpmem.hh"		/* class TCPMEM */

 /* If the buffer type or process type specified in the configuration file is 
    "PHANTOM" then every NML call of that type will result in calling your
    phantom function. */
#include "phantom.hh"		/* class PHANTOMMEM */

 /* LOCMEM is useful when many modules are linked together in one thread of
    execution but you want to write them such that each module uses the NML
    API just as it would if it needed to communicate with the other modules
    running separately. There is no need for any mutual exclusion mechanism
    and memory is obtained with a simple malloc so the operating system will
    not exceed its limits for semaphores or shared memory segments. */
#include "locmem.hh"		/* class LOCMEM */

 /* SHMEM is intended for communications between tasks managed by the same
    operating system. The operating system allocates the memory to be shared
    so users do not need to specify one. Users have a choice of mutual
    exclusion techniques, using an operating system semaphore or mutex, or
    disabling and enabling context switching or interrupts during the
    appropriate critical sections. */
#include "shmem.hh"		/* class SHMEM */

#include "rcs_print.hh"		/* rcs_print_error() */
#include "linklist.hh"		/* LinkedList */

struct CONFIG_FILE_INFO {
    CONFIG_FILE_INFO()
      : lines_list(NULL),
        file_name{}
    {};

    ~CONFIG_FILE_INFO() {
	if (NULL != lines_list) {
	    delete lines_list;
	    lines_list = NULL;
	}
    };

    LinkedList *lines_list;
    char file_name[80];
};

static LinkedList *config_file_list = NULL;
static int loading_config_file = 0;

int load_nml_config_file(const char *file)
{
    unload_nml_config_file(file);
    if (loading_config_file) {
	return -1;
    }
    loading_config_file = 1;
    if (NULL == file) {
	loading_config_file = 0;
	return -1;
    }
    if (NULL == config_file_list) {
	config_file_list = new LinkedList();
    }
    if (NULL == config_file_list) {
	loading_config_file = 0;
	return -1;
    }
    char line[LINELEN];	/* Temporary buffer for line from
					   file. */

    CONFIG_FILE_INFO *info = new CONFIG_FILE_INFO();
    info->lines_list = new LinkedList();
    if (strlen(file) >= 80) {
        rcs_print_error("cms_config: file name too long\n");
        loading_config_file = 0;
        return -1;
    }
    rtapi_strlcpy(info->file_name, file, 80);
    FILE *fp;
    fp = fopen(file, "r");
    if (fp == NULL) {
	rcs_print_error("cms_config: can't open '%s'. Error = %d -- %s\n",
	    file, errno, strerror(errno));
	if (NULL != info) {
	    delete info;
	}
	loading_config_file = 0;
	return -1;
    }
    while (!feof(fp)) {
	if ((fgets(line, LINELEN, fp)) == NULL) {
	    break;
	}
	int linelen = strlen(line);
	if (linelen < 3) {
	    continue;
	}
	while (line[linelen - 1] == '\\') {
	    int pos = linelen - 2;
	    if ((fgets(line + pos, LINELEN - pos, fp)) == NULL) {
		break;
	    }
	    linelen = strlen(line);
	    if (linelen > LINELEN - 2) {
		break;
	    }
	}

	if (line[0] == '#') {
	    continue;
	}
	info->lines_list->store_at_tail(line, linelen + 1, 1);
    }

    if (NULL != fp) {
	fclose(fp);
	fp = NULL;
    }
    config_file_list->store_at_tail(info, sizeof(info), 0);
    loading_config_file = 0;
    return 0;
}

int unload_nml_config_file(const char *file)
{
    if (loading_config_file) {
	return -1;
    }
    if (NULL == file) {
	return -1;
    }
    if (NULL == config_file_list) {
	return -1;
    }
    CONFIG_FILE_INFO *info =
	(CONFIG_FILE_INFO *) config_file_list->get_head();
    while (NULL != info) {
	if (!strncmp(info->file_name, file, 80)) {
	    config_file_list->delete_current_node();
	    delete info;
	    return 0;
	}
	info = (CONFIG_FILE_INFO *) config_file_list->get_next();
    }
    return -1;
}

CONFIG_FILE_INFO *get_loaded_nml_config_file(const char *file)
{
    if (NULL == file) {
	return NULL;
    }
    if (NULL == config_file_list) {
	return NULL;
    }
    CONFIG_FILE_INFO *info =
	(CONFIG_FILE_INFO *) config_file_list->get_head();
    while (NULL != info) {
	if (!strncmp(info->file_name, file, 80)) {
	    return info;
	}
	info = (CONFIG_FILE_INFO *) config_file_list->get_next();
    }
    return NULL;
}

/*! \todo Another #if 0 */
#if 0
int print_loaded_nml_config_file(const char *file)
{
    CONFIG_FILE_INFO *info = get_loaded_nml_config_file(file);
    if (NULL == info) {
	rcs_print("Config file %s not loaded.\n");
	return -1;
    }
    if (NULL == info->lines_list) {
	return -1;
    }
    char *line = (char *) info->lines_list->get_head();
    while (NULL != line) {
	rcs_print("%s", line);
	int linelen = strlen(line);
	if (linelen > 1) {
	    char last_char = line[linelen - 1];
	    if (last_char != '\n' && last_char != '\r') {
		rcs_print("\n");
	    }
	}
	line = (char *) info->lines_list->get_next();
    }
    rcs_print("\n");
    return 0;
}

int print_loaded_nml_config_file_list()
{
    if (loading_config_file) {
	rcs_print
	    ("In the process of loading a config file, please try again later.\n");
	return -1;
    }
    if (NULL == config_file_list) {
	rcs_print("No Configuration files loaded.\n");
	return 0;
    }
    CONFIG_FILE_INFO *info =
	(CONFIG_FILE_INFO *) config_file_list->get_head();
    while (NULL != info) {
	if (NULL != info->lines_list) {
	    rcs_print("%s \t- - \t%d lines\n", info->file_name,
		info->lines_list->list_size);
	} else {
	    rcs_print("%s \t-1 lines", info->file_name);
	}
	info = (CONFIG_FILE_INFO *) config_file_list->get_next();
    }
    return 0;
}
#endif

int unload_all_nml_config_file()
{
    if (loading_config_file) {
	return -1;
    }
    if (NULL == config_file_list) {
	return -1;
    }
    CONFIG_FILE_INFO *info =
	(CONFIG_FILE_INFO *) config_file_list->get_head();
    while (NULL != info) {
	config_file_list->delete_current_node();
	delete info;
	info = (CONFIG_FILE_INFO *) config_file_list->get_next();
    }
    if (config_file_list->list_size <= 0) {
	delete config_file_list;
	config_file_list = NULL;
    }
    return 0;
}

static int convert2upper(char *dest, char *src, int len)
{
    int i;
    if (src == NULL || dest == NULL) {
	rcs_print_error("convert2upper passed NULL argument.\n");
	return -1;
    }

    for (i = 0; i < len; i++) {
	if (src[i] == 0) {
	    dest[i] = 0;
	    return i;
	}
	dest[i] = toupper(src[i]);
    }
    return i;
}

int cms_copy(CMS ** dest, CMS * src, int set_to_server, int set_to_master)
{
    if (NULL == dest || NULL == src) {
	return -1;
    }
    return cms_create_from_lines(dest, src->BufferLine, src->ProcessLine,
	set_to_server, set_to_master);
}

enum CONFIG_SEARCH_ERROR_TYPE {
    CONFIG_SEARCH_ERROR_NOT_SET,
    CONFIG_SEARCH_OK,
    BAD_CONFIG_FILE,
    NO_PROCESS_LINE,
    NO_BUFFER_LINE,
    MISC_CONFIG_SEARCH_ERROR
};

struct CONFIG_SEARCH_STRUCT {
    enum CONFIG_SEARCH_ERROR_TYPE error_type;
    int bufline_found;
    int bufline_number;
    int procline_found;
    int procline_number;
    const char *bufname;
    const char *bufname_for_procline;
    const char *procname;
    const char *filename;
    char buffer_line[LINELEN];	/* Line matching bufname. */
    char proc_line[LINELEN];	/* Line matching procname & bufname. */
    char buffer_type[LINELEN];	/* "SHMEM" or "GLOBMEM" */
    char proc_type[LINELEN];	/* "REMOTE" or "LOCAL" */
};

void find_proc_and_buffer_lines(CONFIG_SEARCH_STRUCT * s);

/* Function for initializing a CMS from a configuration file. */
 /* Returns 0 for success or -1 for error. */
int cms_config(CMS ** cms, const char *bufname, const char *procname, const char *filename,
    int set_to_server, int set_to_master)
{
    CONFIG_SEARCH_STRUCT search;
    char buf[LINELEN];
    char buf2[LINELEN];
    char *default_ptr = 0;

    if (0 == bufname || 0 == procname || 0 == filename) {
	return -1;
    }
    rcs_print_debug(PRINT_CMS_CONFIG_INFO, "cms_config arguments:\n");
    rcs_print_debug(PRINT_CMS_CONFIG_INFO, "bufname = %s\n", bufname);
    rcs_print_debug(PRINT_CMS_CONFIG_INFO, "procname = %s\n", procname);
    rcs_print_debug(PRINT_CMS_CONFIG_INFO, "filename = %s\n", filename);

    search.error_type = CONFIG_SEARCH_ERROR_NOT_SET;
    search.bufline_found = 0;
    search.bufline_number = -1;
    search.procline_found = 0;
    search.procline_number = -1;
    search.bufname = bufname;
    search.bufname_for_procline = bufname;
    search.procname = procname;
    search.filename = filename;
    find_proc_and_buffer_lines(&search);
    if (NO_PROCESS_LINE == search.error_type) {
	search.bufname_for_procline = "default";
	find_proc_and_buffer_lines(&search);
	if (search.error_type == CONFIG_SEARCH_OK) {
	    default_ptr = 0;
	    strncpy(buf, search.proc_line, LINELEN);
	    default_ptr = strstr(buf, "default");
	    if (default_ptr) {
		rtapi_strxcpy(buf2, default_ptr + 7);
		strcpy(default_ptr, bufname);
		default_ptr += strlen(bufname);
		strcpy(default_ptr, buf2);
		rtapi_strlcpy(search.proc_line, buf, LINELEN);
	    }
	    rtapi_strxcat(search.proc_line, " defaultbuf");
	}
    }
    if (NO_PROCESS_LINE == search.error_type) {
	search.bufname_for_procline = "default";
	search.procname = "default";
	find_proc_and_buffer_lines(&search);
	if (search.error_type == CONFIG_SEARCH_OK) {
	    strncpy(buf, search.proc_line, LINELEN);
	    default_ptr = strstr(buf, "default");
	    if (default_ptr) {
		rtapi_strxcpy(buf2, default_ptr + 7);
		strcpy(default_ptr, procname);
		default_ptr += strlen(procname);
		strcpy(default_ptr, buf2);
		default_ptr = strstr(buf, "default");
	    }
	    if (default_ptr) {
		rtapi_strxcpy(buf2, default_ptr + 7);
		strcpy(default_ptr, bufname);
		default_ptr += strlen(bufname);
		strcpy(default_ptr, buf2);
		rtapi_strlcpy(search.proc_line, buf, LINELEN);
	    }
	    rtapi_strxcat(search.proc_line, " defaultproc defaultbuf");
	}
    }
    if (CONFIG_SEARCH_OK == search.error_type) {
	return (cms_create(cms, search.buffer_line, search.proc_line,
		search.buffer_type, search.proc_type,
		set_to_server, set_to_master));
    }
    switch (search.error_type) {
    case NO_BUFFER_LINE:
	rcs_print_error
	    ("No buffer-line entry found for buffer %s in config file %s.\n",
	    bufname, filename);
	break;

    case NO_PROCESS_LINE:
	rcs_print_error
	    ("No process-line entry found for process %s connecting to buffer %s in config file %s and no applicable defaults were found.\n",
	    procname, bufname, filename);
	break;

    default:
	break;
    }

    return (-1);
}

int hostname_matches_bufferline(char *bufline)
{
    char my_hostname[256];
    struct hostent *my_hostent_ptr = 0;
    struct hostent *buffer_hostent_ptr = 0;
    struct hostent my_hostent;
    char my_hostent_addresses[16][16];
    int num_my_hostent_addresses = 0;
    struct in_addr myaddress;
    int j, k;
    char *buffer_host = 0;
    char *word[4];		/* array of pointers to words from line */

    if (0 == bufline) {
	return 0;
    }

    /* Separate out the first four strings in the line. */
    if (separate_words(word, 4, bufline) != 4) {
	return 0;
    }
    buffer_host = word[3];
    if (buffer_host == 0) {
	return 0;
    }

    if (!strncmp(buffer_host, "localhost", 9)) {
	return 1;
    }
    gethostname(my_hostname, 256);
    if (!strcmp(buffer_host, my_hostname)) {
	return 1;
    }
    my_hostent_ptr = gethostbyname(my_hostname);
    if (0 == my_hostent_ptr) {
	return 0;
    }
    myaddress.s_addr = *((int *) my_hostent_ptr->h_addr_list[0]);
    if (!strcmp(buffer_host, inet_ntoa(myaddress))) {
	return 1;
    }
    if (my_hostent_ptr->h_length < 1 || my_hostent_ptr->h_length > 16) {
	rcs_print_error("Bad hostentry length.\n");
	return 0;
    }
    /* We need to make a copy of my_hostent and all its addresses in case
       they are clobbered when we try to get the hostentry for buffer_host */
    my_hostent = *my_hostent_ptr;
    memset(my_hostent_addresses, 0, 256);
    for (j = 0; j < 16 && 0 != my_hostent.h_addr_list[j]; j++) {
	memcpy(my_hostent_addresses[j], my_hostent.h_addr_list[j],
	    my_hostent.h_length);
    }
    num_my_hostent_addresses = j;
    if (num_my_hostent_addresses < 1) {
	return 0;
    }
    buffer_hostent_ptr = gethostbyname(buffer_host);
    if (0 == buffer_hostent_ptr) {
	return 0;
    }
    j = 0;
    if (buffer_hostent_ptr->h_length != my_hostent.h_length) {
	rcs_print_error("Mismatched hostentry lengths.\n");
	return 0;
    }
    while (j < num_my_hostent_addresses && j < 16) {
	k = 0;
	while (buffer_hostent_ptr->h_addr_list[k] != 0 && k < 16) {
	    if (!memcmp
		(my_hostent_addresses[j], buffer_hostent_ptr->h_addr_list[k],
		    my_hostent.h_length)) {
		return 1;
	    }
	    k++;
	}
	j++;
    }

    return 0;
}

void find_proc_and_buffer_lines(CONFIG_SEARCH_STRUCT * s)
{
    if (s == 0) {
	return;
    }

    loading_config_file = 1;
    FILE *fp = NULL;		/* FILE ptr to config file.  */
    char linebuf[LINELEN];	/* Temporary buffer for line from
					   file. */
    char *line = linebuf;
    int line_len, line_number;
    char *word[4];		/* array of pointers to words from line */

    int first_line = 1;

    /* Open the configuration file. */
    LinkedList *lines_list = NULL;
    CONFIG_FILE_INFO *info = get_loaded_nml_config_file(s->filename);
    if (NULL != info) {
	lines_list = info->lines_list;
	line = (char *) lines_list->get_head();
    }

    if (NULL == lines_list) {
	fp = fopen(s->filename, "r");
	if (fp == NULL) {
	    rcs_print_error("cms_config: can't open '%s'. Error = %d -- %s\n",
		s->filename, errno, strerror(errno));
	    loading_config_file = 0;
	    s->error_type = BAD_CONFIG_FILE;
	    return;
	}
    }

    /* Read the configuration file line by line until the lines matching */
    /* bufname and procname are found.  */
    line_number = 0;

    while (1) {
	if (NULL != lines_list) {
	    if (!first_line) {
		line = (char *) lines_list->get_next();
	    }
	    first_line = 0;
	    if (NULL == line) {
		break;
	    }
	} else {
	    if (feof(fp)) {
		break;
	    }
	    if ((fgets(line, LINELEN, fp)) == NULL) {
		break;
	    }
	}

	line_number++;
	line_len = strlen(line);
	if (line_len < 3) {
	    continue;
	}
	while (line[line_len - 1] == '\\') {
	    int pos = line_len - 2;
	    if ((fgets(line + pos, LINELEN - pos, fp)) == NULL) {
		break;
	    }
	    line_len = strlen(line);
	    if (line_len > LINELEN) {
		break;
	    }
	    line_number++;
	}
	if (line_len > LINELEN) {
	    rcs_print_error
		("cms_cfg: Line length of line number %d in %s exceeds max length of %d",
		line_number, s->filename, LINELEN);
	}

	/* Skip comment lines and lines starting with white space. */
	if (line[0] == CMS_CONFIG_COMMENTCHAR ||
	    strchr(" \t\n\r\0", line[0]) != NULL) {
	    continue;
	}

	/* Separate out the first four strings in the line. */
	if (separate_words(word, 4, line) != 4) {
	    continue;
	}

	if (!s->bufline_found && !strcmp(word[1], s->bufname) &&
	    line[0] == 'B') {
	    /* Buffer line found, store the line and type. */
	    strncpy(s->buffer_line, line, LINELEN);
	    convert2upper(s->buffer_type, word[2], LINELEN);
	    s->bufline_found = 1;
	    s->bufline_number = line_number;
	    rcs_print_debug(PRINT_CMS_CONFIG_INFO,
		"cms_config found buffer line on line %d\n", line_number);
	} else if (!s->procline_found && !strcmp(word[1], s->procname) &&
	    line[0] == 'P' && !strcmp(word[2], s->bufname_for_procline)) {
	    /* Procedure line found, store the line and type. */
	    strncpy(s->proc_line, line, LINELEN);
	    switch (cms_connection_mode) {
	    case CMS_NORMAL_CONNECTION_MODE:
		convert2upper(s->proc_type, word[3], LINELEN);
		if (!strncmp(s->proc_type, "AUTO", 4)) {
		    if (!s->bufline_found) {
			rcs_print_error
			    ("Can't use process type AUTO unless the buffer line for %s is found earlier in the config file.\n",
			    s->bufname);
			rcs_print_error("Bad line:\n%s:%d %s\n", s->filename,
			    line_number, line);
			s->error_type = MISC_CONFIG_SEARCH_ERROR;
			return;
		    }
		    if (hostname_matches_bufferline(s->buffer_line)) {
			rtapi_strxcpy(s->proc_type, "LOCAL");
		    } else {
			rtapi_strxcpy(s->proc_type, "REMOTE");
		    }
		}
		break;

	    case CMS_FORCE_LOCAL_CONNECTION_MODE:
		rtapi_strxcpy(s->proc_type, "LOCAL");
		break;

	    case CMS_FORCE_REMOTE_CONNECTION_MODE:
		rtapi_strxcpy(s->proc_type, "REMOTE");
		break;
	    }
	    s->procline_found = 1;
	    s->procline_number = line_number;
	    rcs_print_debug(PRINT_CMS_CONFIG_INFO,
		"cms_config found process line on line %d\n", line_number);
	}

	if (s->procline_found && s->bufline_found) {
	    /* Close the configuration file. */
	    if (NULL != fp) {
		fclose(fp);
		fp = NULL;
	    }
	    loading_config_file = 0;
	    s->error_type = CONFIG_SEARCH_OK;
	    return;
	}
    }

    /* Close the configuration file. */
    if (NULL != fp) {
	fclose(fp);
	fp = NULL;
    }
    loading_config_file = 0;

    /* Missing either procname or bufname or both. */
    if (!s->bufline_found) {
	s->error_type = NO_BUFFER_LINE;
	return;
    }
    if (!s->procline_found) {
	s->error_type = NO_PROCESS_LINE;
	return;
    }
    return;
}

int
cms_create_from_lines(CMS ** cms, const char *buffer_line_in, const char *proc_line_in,
    int set_to_server, int set_to_master)
{
    char proc_type[LINELEN];
    char buffer_type[LINELEN];
    char *word[4];		/* array of pointers to words from line */

    char *proc_line = strdup(proc_line_in);
    if (4 != separate_words(word, 4, proc_line)) {
	rcs_print_error("cms_config: invalid proc_line=(%s)\n", proc_line);
	free(proc_line);
	return -1;
    }

    convert2upper(proc_type, word[3], LINELEN);

    char *buffer_line = strdup(buffer_line_in);
    if (4 != separate_words(word, 4, buffer_line)) {
	rcs_print_error("cms_config: invalid buffer_line=(%s)\n",
	    buffer_line);
	free(proc_line);
	free(buffer_line);
	return -1;
    }

    convert2upper(buffer_type, word[2], LINELEN);

    int result = (cms_create(cms, buffer_line, proc_line,
	    buffer_type, proc_type, set_to_server, set_to_master));

    free(proc_line);
    free(buffer_line);

    return result;
}

int cms_create(CMS ** cms, const char *buffer_line, const char *proc_line,
    const char *buffer_type, const char *proc_type, int set_to_server, int set_to_master)
{
    if (NULL == cms || NULL == buffer_line || NULL == proc_line ||
	NULL == buffer_type || NULL == proc_type) {
	rcs_print_error("cms_create passed NULL argument.\n");
	return -1;
    }

    /* Both lines have been found, select the appropriate class from */
    /* CMS's derived classes and call its constructor. */
    if (!strcmp(buffer_type, "PHANTOM") || !strcmp(proc_type, "PHANTOM")) {
	*cms = new PHANTOMMEM(buffer_line, proc_line);
	rcs_print_debug(PRINT_CMS_CONFIG_INFO, "%p = new PHANTOMEM(%s,%s)\n",
	    *cms, buffer_line, proc_line);
	if (NULL == *cms) {
	    if (verbose_nml_error_messages) {
		rcs_print_error
		    ("cms_config: Can't create PHANTOMMEM object.\n");
	    }
	    return (-1);
	} else {
	    return (0);
	}
    }
    if (!strcmp(proc_type, "REMOTE")) {
	if (NULL != strstr(proc_line, "serialPortDevName=")) {
/*! \todo Another #if 0 */
#if 0
	    *cms = new TTYMEM(buffer_line, proc_line);
	    rcs_print_debug(PRINT_CMS_CONFIG_INFO, "%X = new TTYMEM(%s,%s)\n",
		*cms, buffer_line, proc_line);
	    if (NULL == *cms) {
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("cms_config: Can't create new TTYMEM object.\n");
		}
		return (-1);
	    } else if ((*cms)->status < 0) {
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("cms_config: Error  %d(%s) occurred during TTYMEM create.\n",
			(*cms)->status,
			(*cms)->status_string((*cms)->status));
		}
		return (-1);
	    }
#else
	    rcs_print_error("TTYMEM not supported on this platform.\n");
	    return (-1);
#endif
	} else if (NULL != strstr(buffer_line, "STCP=")) {
/*! \todo Another #if 0 */
#if 0
	    *cms = new STCPMEM(buffer_line, proc_line);
	    rcs_print_debug(PRINT_CMS_CONFIG_INFO,
		"%X = new STCPMEM(%s,%s)\n", *cms, buffer_line, proc_line);
	    if (NULL == *cms) {
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("cms_config: Can't create new STPCMEM object.\n");
		}
		return (-1);
	    } else if ((*cms)->status < 0) {
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("cms_config: Error  %d(%s) occurred during STPCMEM create.\n",
			(*cms)->status,
			(*cms)->status_string((*cms)->status));
		}
		return (-1);
	    }
#endif
	} else if (NULL != strstr(buffer_line, "TCP=")) {
	    *cms = new TCPMEM(buffer_line, proc_line);
	    rcs_print_debug(PRINT_CMS_CONFIG_INFO, "%p = new TCPMEM(%s,%s)\n",
		*cms, buffer_line, proc_line);
	    if (NULL == *cms) {
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("cms_config: Can't create new TPCMEM object.\n");
		}
		return (-1);
	    } else if ((*cms)->status < 0) {
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("cms_config: Error  %d(%s) occurred during TPCMEM create.\n",
			(*cms)->status,
			(*cms)->status_string((*cms)->status));
		}
		return (-1);
	    }
	} else if (NULL != strstr(buffer_line, "UDP=")) {
	    rcs_print_error("UPDMEM not supported.\n");
	    return (-1);
	} else {
	    rcs_print_error("No remote connection configured.\n");
	    return (-1);
	}
    } else if (!strcmp(proc_type, "LOCAL")) {

	if (!strcmp(buffer_type, "SHMEM")) {
	    *cms = new SHMEM(buffer_line, proc_line, set_to_server,
		set_to_master);
	    rcs_print_debug(PRINT_CMS_CONFIG_INFO,
		"%p = new SHMEM(%s,%s,%d,%d)\n", *cms, buffer_line,
		proc_line, set_to_server, set_to_master);
	    if (NULL == *cms) {
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("cms_config: Can't create new SHMEM object.\n");
		}
		return (-1);
	    } else if ((*cms)->status < 0) {
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("cms_config: %d(%s) Error occurred during SHMEM create.\n",
			(*cms)->status,
			(*cms)->status_string((*cms)->status));
		}
		return (-1);
	    } else {
		return (0);
	    }
	}

	if (!strcmp(buffer_type, "RTLMEM")) {
	    rcs_print_error("RTLMEM not supported.\n");
	    return (-1);
	}

	if (!strcmp(buffer_type, "LOCMEM")) {
	    *cms =
		new LOCMEM(buffer_line, proc_line, set_to_server,
		set_to_master);
	    rcs_print_debug(PRINT_CMS_CONFIG_INFO,
		"%p = new LOCMEM(%s,%s,%d,%d)\n", *cms, buffer_line,
		proc_line, set_to_server, set_to_master);
	    if (NULL == *cms) {
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("cms_config: Can't create new LOCMEM object.\n");
		}
		return (-1);
	    }
	    if ((*cms)->status < 0) {
		if (verbose_nml_error_messages) {
		    rcs_print_error
			("cms_config: %d(%s) Error occurred during LOCMEM create.\n",
			(*cms)->status,
			(*cms)->status_string((*cms)->status));
		}
		return (-1);
	    }
	    return (0);
	}

	rcs_print_error("cms_config: invalid buffer_type (%s)\n",
	    buffer_type);
	rcs_print_error("cms_config: buffer_line = (%s)\n", buffer_line);
	return (-1);
    } else {
	rcs_print_error("cms_config: invalid proc_type (%s)\n", proc_type);
	rcs_print_error("cms_config: proc_line = (%s)\n", proc_line);
	return (-1);
    }
    return (0);
}
