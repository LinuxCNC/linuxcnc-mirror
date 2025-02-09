/********************************************************************
* Description: rcs_print.cc
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

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>		/* va_list, va_start(), va_end() */
#include <stdio.h>		/* __printf()'s */
#include <string.h>		/* strchr(), memmove() */
#include <stdlib.h>		/* malloc(), free(), realloc() */
#include <errno.h>		// errno()

#include <sys/types.h>
#include <unistd.h>		/* getpid() */

#ifdef __cplusplus
}
#endif
#include <rtapi_string.h>
#include "rcs_print.hh"
#include "linklist.hh"
#ifndef _TIMER_H
extern "C" double etime(void);
#endif

LinkedList *rcs_print_list = NULL;
char **rcs_lines_table = NULL;
void (*rcs_print_notify) () = NULL;
RCS_PRINT_DESTINATION_TYPE rcs_print_destination = RCS_PRINT_TO_STDOUT;

int max_rcs_errors_to_print = 30;
int rcs_errors_printed = 0;

long rcs_print_mode_flags = PRINT_RCS_ERRORS;
FILE *rcs_print_file_stream = NULL;
char rcs_print_file_name[80] = "rcs_out.txt";

char last_error_bufs[4][error_buf_size];
int error_bufs_initialized = 0;
int last_error_buf_filled = 0;

void set_rcs_print_destination(RCS_PRINT_DESTINATION_TYPE _dest)
{
    if (rcs_print_destination == RCS_PRINT_TO_NULL) {
	rcs_errors_printed = 0;
    }
    rcs_print_destination = _dest;
}

RCS_PRINT_DESTINATION_TYPE get_rcs_print_destination()
{
    return (rcs_print_destination);
}

char **get_rcs_lines_table()
{
    return (rcs_lines_table);
}

LinkedList *get_rcs_print_list()
{
    return (rcs_print_list);
}

int get_rcs_print_list_size()
{
    if (NULL != rcs_print_list) {
	return (rcs_print_list->list_size);
    } else {
	return (-1);
    }
}
void
set_rcs_print_list_sizing(int _new_max_size,
    LIST_SIZING_MODE _new_sizing_mode)
{
    if (NULL == rcs_print_list) {
	rcs_print_list = new LinkedList;
    }
    if (NULL != rcs_print_list) {
	rcs_print_list->set_list_sizing_mode(_new_max_size, _new_sizing_mode);
    }
}
void set_rcs_print_notify(RCS_PRINT_NOTIFY_FUNC_PTR _rcs_print_notify)
{
    rcs_print_notify = _rcs_print_notify;
}

void clean_print_list()
{
    if (NULL != rcs_print_list) {
	delete rcs_print_list;
	rcs_print_list = NULL;
    }
}

void output_print_list(int output_func(char *))
{
    if (NULL != rcs_print_list) {
	char *string_from_list;
	string_from_list = (char *) rcs_print_list->get_head();
	while (NULL != string_from_list) {
	    if (output_func(string_from_list) != EOF) {
		break;
	    }
	    string_from_list = (char *) rcs_print_list->get_next();
	}
    }
}

int count_characters_in_print_list()
{
    int count = 0;
    if (NULL != rcs_print_list) {
	char *string_from_list;
	string_from_list = (char *) rcs_print_list->get_head();
	while (NULL != string_from_list) {
	    count += strlen(string_from_list);
	    string_from_list = (char *) rcs_print_list->get_next();
	}
    }
    return (count);
}

int count_lines_in_print_list()
{
    int count = 1;
    if (NULL != rcs_print_list) {
	char *string_from_list;
	string_from_list = (char *) rcs_print_list->get_head();
	while (NULL != string_from_list) {
	    char *line;
	    line = strchr(string_from_list, '\n');
	    while (NULL != line) {
		count++;
		line = strchr(line + 1, '\n');
	    }
	    string_from_list = (char *) rcs_print_list->get_next();
	}
    }
    return (count);
}

void convert_print_list_to_lines()
{
    char *temp_buf = NULL;
    static int last_id_converted = -1;
    if (NULL != rcs_print_list) {
	char *string_from_list;
	if (-1 == last_id_converted) {
	    string_from_list = (char *) rcs_print_list->get_head();
	} else {
	    string_from_list =
		(char *) rcs_print_list->get_first_newer(last_id_converted);
	}
	while (NULL != string_from_list) {
	    char *next_line;
	    next_line = strchr(string_from_list, '\n');
	    if (NULL == next_line) {
		if (NULL == temp_buf) {
		    temp_buf = (char *) malloc(strlen(string_from_list) + 1);
		    rtapi_strlcpy(temp_buf, string_from_list, strlen(string_from_list) + 1);
		} else {
		    char *ra = (char *) realloc(temp_buf, strlen(temp_buf) + strlen(string_from_list) + 1);
		    if(ra) {
			temp_buf = ra;
			strcat(temp_buf, string_from_list);
		    }
		}
		rcs_print_list->delete_current_node();
	    } else {
		if (temp_buf != NULL) {
		    char *ra = (char *) realloc(temp_buf, strlen(temp_buf) + strlen(string_from_list) + 1);
		    if(ra) {
			temp_buf = ra;
			strcat(temp_buf, string_from_list);
			rcs_print_list->delete_current_node();
			rcs_print_list->store_after_current_node(temp_buf, strlen(temp_buf) + 1, 1);
		    }
		    free(temp_buf);
		    temp_buf = NULL;
		} else if (next_line[1] != 0) {
		    rcs_print_list->store_after_current_node(next_line + 1,
			strlen(next_line + 1) + 1, 1);
		    next_line[1] = 0;
		}
	    }
	    string_from_list = (char *) rcs_print_list->get_next();
	}
    }
    last_id_converted = rcs_print_list->get_newest_id();
    if (temp_buf != NULL) {
	rcs_print_list->store_at_tail(temp_buf, strlen(temp_buf) + 1, 1);
	free(temp_buf);
	temp_buf = NULL;
    }
}

void update_lines_table()
{
    if (NULL != rcs_lines_table) {
	free(rcs_lines_table);
	rcs_lines_table = NULL;
    }
    if (NULL != rcs_print_list) {
	convert_print_list_to_lines();
	rcs_lines_table = (char **) malloc(sizeof(char *) * rcs_print_list->list_size);
	if (NULL != rcs_lines_table) {
	    char *string_from_list;
	    string_from_list = (char *) rcs_print_list->get_head();
	    int i = 0;
	    while (NULL != string_from_list) {
		rcs_lines_table[i] = string_from_list;
		i++;
		string_from_list = (char *) rcs_print_list->get_next();
	    }
	}
    }
}

char *strip_control_characters(char *_dest, char *_src)
{
    static char line_buffer[255];
    char *destination;
    char *control_char_loc;
    if (NULL == _dest) {
	destination = line_buffer;
	if (strlen(_src) < 255) {
	    rtapi_strxcpy(line_buffer, _src);
	} else {
	    if (NULL == strpbrk(_src, "\n\r\t\b")) {
		return (_src);
	    } else {
		return (NULL);
	    }
	}
    } else {
	destination = _dest;
	if (_dest != _src) {
	    memmove(_dest, _src, strlen(_src));
	}
    }
    control_char_loc = strpbrk(destination, "\n\r\t\b");
    while (NULL != control_char_loc) {
	*control_char_loc = ' ';	/* Replace control character with
					   SPACE */
	control_char_loc = strpbrk(control_char_loc, "\n\r\t\b");
    }
    return (destination);

}

/* In windows DLLs for Microsoft Visual C++ sscanf is not supported so
use separate_words to parse the string followed by commands like strtod to
convert each word. */
int separate_words(char **_dest, int _max, char *_src)
{
    static char word_buffer[256];
    int i;
    if (NULL == _dest || NULL == _src) {
	return -1;
    }
    if (strlen(_src) > 255) {
	return -1;
    }
    rtapi_strxcpy(word_buffer, _src);
    _dest[0] = strtok(word_buffer, " \n\r\t");
    for (i = 0; i < _max - 1 && NULL != _dest[i]; i++) {
	_dest[i + 1] = strtok(NULL, " \n\r\t");
    }
    if (_dest[_max - 1] == NULL && i == _max - 1) {
	i--;
    }
    return (i + 1);
}

int rcs_vprint(const char *_fmt, va_list _args, int save_string)
{
    static char temp_string[error_buf_size];

    if (NULL == _fmt) {
	return (EOF);
    }
    if (strlen(_fmt) > 200) {	/* Might overflow temp_string. */
	return (EOF);
    }
    if (EOF == (int) vsnprintf(temp_string, sizeof(temp_string), _fmt, _args)) {
	return (EOF);
    }
    if (save_string) {
	if (!error_bufs_initialized) {
	    memset(last_error_bufs[0], 0, 100);
	    memset(last_error_bufs[1], 0, 100);
	    memset(last_error_bufs[2], 0, 100);
	    memset(last_error_bufs[3], 0, 100);
	    error_bufs_initialized = 1;
	}
	last_error_buf_filled++;
	last_error_buf_filled %= 4;
	rtapi_strlcpy(last_error_bufs[last_error_buf_filled], temp_string, 99);
    }
    return (rcs_fputs(temp_string));
}

int rcs_puts(const char *_str)
{
    int retval, retval2;
    retval = rcs_fputs(const_cast< char * >(_str));
    if (retval != EOF) {
	retval2 = rcs_fputs(const_cast< char * >("\n"));
	if (retval2 != EOF) {
	    retval += retval;
	} else {
	    retval = EOF;
	}
    }
    return (retval);
}

int rcs_fputs(const char *_str)
{
    int retval = EOF;
    if (NULL != _str) {
	if (0 == _str[0]) {
	    return (0);
	}
	switch (rcs_print_destination) {
	case RCS_PRINT_TO_LOGGER:

	case RCS_PRINT_TO_STDOUT:
	    retval = fputs(_str, stdout);
	    fflush(stdout);
	    break;

	case RCS_PRINT_TO_STDERR:
	    retval = fputs(_str, stderr);
	    fflush(stderr);
	    break;

	case RCS_PRINT_TO_LIST:
	    if (NULL == rcs_print_list) {
		rcs_print_list = new LinkedList;
		if (NULL != rcs_print_list) {
		    rcs_print_list->set_list_sizing_mode(256,
			DELETE_FROM_HEAD);
		}
	    }
	    if (NULL != rcs_print_list) {
		if (-1 ==
		    rcs_print_list->store_at_tail((void*)_str,
			(retval = strlen(_str)) + 1, 1)) {
		    retval = EOF;
		}
	    }
	    break;
	case RCS_PRINT_TO_NULL:
	    retval = strlen(_str);
	    break;
	case RCS_PRINT_TO_FILE:
	    if (NULL == rcs_print_file_stream) {
		rcs_print_file_stream = fopen(rcs_print_file_name, "a+");
	    }
	    if (NULL == rcs_print_file_stream) {
		return EOF;
	    }
	    retval = fputs(_str, rcs_print_file_stream);
	    fflush(rcs_print_file_stream);
	    break;

	default:
	    break;
	}
	if (NULL != rcs_print_notify) {
	    (*rcs_print_notify) ();
	}
    }
    return (retval);
}

void close_rcs_printing()
{
    switch (rcs_print_destination) {
    case RCS_PRINT_TO_LIST:
	clean_print_list();
	break;

    case RCS_PRINT_TO_FILE:
	if (NULL != rcs_print_file_stream) {
	    fclose(rcs_print_file_stream);
	    rcs_print_file_stream = NULL;
	}
	break;
    default:
	break;
    }
    return;
}

int set_rcs_print_file(char *_file_name)
{
    if (_file_name == NULL) {
	return -1;
    }
    if (strlen(_file_name) > 80) {
	return -1;
    }
    rtapi_strxcpy(rcs_print_file_name, _file_name);
    if (NULL != rcs_print_file_stream) {
	fclose(rcs_print_file_stream);
    }
    rcs_print_file_stream = fopen(rcs_print_file_name, "a+");
    if (NULL == rcs_print_file_stream) {
	return -1;
    }
    return 0;
}

int rcs_print(const char *_fmt, ...)
{
    static char temp_buffer[400];
    int retval;
    va_list args;
    va_start(args, _fmt);
    retval = vsnprintf(temp_buffer, sizeof(temp_buffer), _fmt, args);
    va_end(args);
    if (retval == (EOF)) {
	return EOF;
    }
    retval = rcs_fputs(temp_buffer);
    return (retval);
}

#ifndef DO_NOT_USE_RCS_PRINT_ERROR_NEW
static const char *rcs_error_filename = NULL;
static int rcs_error_linenum = -1;
int set_print_rcs_error_info(const char *file, int line)
{
    rcs_error_filename = file;
    rcs_error_linenum = line;
    return (0);
}

int print_rcs_error_new(const char *_fmt, ...)
{
    int retval = 0;
    va_list args;
    va_start(args, _fmt);
    if ((rcs_print_mode_flags & PRINT_RCS_ERRORS)
	&& ((max_rcs_errors_to_print >= rcs_errors_printed)
	    || max_rcs_errors_to_print < 0)) {
	if (NULL != rcs_error_filename && rcs_error_linenum > 0) {
	    rcs_print("%s %d: ", rcs_error_filename, rcs_error_linenum);
	    rcs_error_filename = NULL;
	    rcs_error_linenum = -1;
	}
	retval = rcs_vprint(_fmt, args, 1);
	if (max_rcs_errors_to_print == rcs_errors_printed &&
	    max_rcs_errors_to_print >= 0) {
	    rcs_print("\nMaximum number of errors to print exceeded!\n");
	}
    }
    if (rcs_print_destination != RCS_PRINT_TO_NULL) {
	rcs_errors_printed++;
    }
    va_end(args);
    return (retval);
}

#endif

int rcs_print_debug(long flag_to_check, const char *_fmt, ...)
{
    int retval = 0;
    int pid = 0;
    va_list args;
    va_start(args, _fmt);

    if (flag_to_check & rcs_print_mode_flags) {
	pid = getpid();
	rcs_print("(time=%f,pid=%d): ", etime(), pid);
	retval = rcs_vprint(_fmt, args, 0);
    }
    va_end(args);
    return (retval);
}

void set_rcs_print_flag(long flag_to_set)
{
    rcs_print_mode_flags |= flag_to_set;
}

void clear_rcs_print_flag(long flag_to_clear)
{
    rcs_print_mode_flags &= ~(flag_to_clear);
}

int rcs_print_sys_error(int error_source, const char *_fmt, ...)
{
    static char temp_string[256];
    static char message_string[512];
    va_list args;
    int r;

    if (NULL == _fmt) {
	return (EOF);
    }
    if (strlen(_fmt) > 200) {	/* Might overflow temp_string. */
	return (EOF);
    }

    va_start(args, _fmt);
    r = vsnprintf(temp_string, sizeof(temp_string), _fmt, args);
    va_end(args);

    if (r < 0) {
	return EOF;
    }

    if (max_rcs_errors_to_print == rcs_errors_printed &&
	max_rcs_errors_to_print >= 0) {
	rcs_print("\nMaximum number of errors to print exceeded!\n");
    }
    rcs_errors_printed++;
    if (max_rcs_errors_to_print <= rcs_errors_printed &&
	max_rcs_errors_to_print >= 0) {
	return (EOF);
    }

    switch (error_source) {
    case ERRNO_ERROR_SOURCE:
	snprintf(message_string, sizeof(message_string),
	    "%s %d %s\n", temp_string, errno,
	    strerror(errno));
	rcs_puts(message_string);
	break;

    default:
	rcs_puts(temp_string);
	break;
    }
    return (strlen(temp_string));
}

#ifdef rcs_print_error
#undef rcs_print_error
#endif
extern "C" int rcs_print_error(const char *_fmt, ...);

int rcs_print_error(const char *_fmt, ...)
{
    int retval = 0;
    va_list args;

    va_start(args, _fmt);
    if ((rcs_print_mode_flags & PRINT_RCS_ERRORS)
	&& ((max_rcs_errors_to_print >= rcs_errors_printed)
	    || max_rcs_errors_to_print < 0)) {
	retval = rcs_vprint(_fmt, args, 1);
	if (max_rcs_errors_to_print == rcs_errors_printed &&
	    max_rcs_errors_to_print >= 0) {
	    rcs_print("\nMaximum number of errors to print exceeded!\n");
	}
    }
    if (rcs_print_destination != RCS_PRINT_TO_NULL) {
	rcs_errors_printed++;
    }
    va_end(args);
    return (retval);
}
