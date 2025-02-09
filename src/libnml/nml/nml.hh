/********************************************************************
* Description: nml.hh
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

#ifndef NML_HH
#define NML_HH

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>		/* size_t */

#ifdef __cplusplus
}
#endif
#include "cms_user.hh"		/* class CMS_USER */
class LinkedList;
/* Generic NML Stuff */
#include "nml_type.hh"

class NMLmsg;			/* Use only partial definition to avoid */
				/* depending on nmlmsg.hh. */

/* Typedef for pointer to the function used to decode a message */
 /* by its id number. */
typedef int (*NML_FORMAT_PTR) (NMLTYPE, void *, CMS *);

/* Values for NML::error_type. */
enum NML_ERROR_TYPE {
    NML_NO_ERROR,
    NML_BUFFER_NOT_READ,
    NML_TIMED_OUT,
    NML_INVALID_CONFIGURATION,
    NML_FORMAT_ERROR,
    NML_INTERNAL_CMS_ERROR,
    NML_NO_MASTER_ERROR,
    NML_INVALID_MESSAGE_ERROR,
    NML_QUEUE_FULL_ERROR
};

enum NML_CHANNEL_TYPE {
    INVALID_NML_CHANNEL_TYPE = 0,
    NML_GENERIC_CHANNEL_TYPE,
    RCS_CMD_CHANNEL_TYPE,
    RCS_STAT_CHANNEL_TYPE,
    NML_QUERY_CHANNEL_TYPE,
    NML_ID_CHANNEL_TYPE
};

extern char NML_ERROR_TYPE_STRINGS[8][80];
class NML_DIAGNOSTICS_INFO;

/* nml interface to CMS. */
class NML:public virtual CMS_USER {
  protected:
    int run_format_chain(NMLTYPE, void *);
    int format_input(NMLmsg * nml_msg);	/* Format message if necessary */
    int format_output();	/* Decode message if necessary. */

  public:
    void *operator                          new(size_t);
    void operator                          delete(void *);
    LinkedList *format_chain;
    void register_with_server();	/* Add this channel to the server's
					   list. */
    void add_to_channel_list();	/* Add this channel to the main list.  */
    int channel_list_id;	/* List id of this channel. */
    NML_ERROR_TYPE error_type;	/* check here if an NML function returns -1 */
    /* Get Address of message for user after read. */
      NMLTYPE(*phantom_read) ();
      NMLTYPE(*phantom_peek) ();
    int (*phantom_write) (NMLmsg * nml_msg, int *serial_number);
    int (*phantom_write_if_read) (NMLmsg * nml_msg, int *serial_number);
    int (*phantom_check_if_read) ();
    int (*phantom_clear) ();
    int ignore_format_chain;

    NMLmsg *get_address();
    void delete_channel();

    /* Read and Write Functions. */
    NMLTYPE read();		/* Read the buffer. */
    NMLTYPE blocking_read(double timeout);	/* Read the buffer. (Wait for 
						   new data). */
    NMLTYPE peek();		/* Read buffer without changing was_read */
    NMLTYPE read(void *, long);
    NMLTYPE peek(void *, long);
    int write(NMLmsg & nml_msg, int *serial_number = NULL);	/* Write a message. (Use reference) */
    int write(NMLmsg * nml_msg, int *serial_number = NULL);	/* Write a message. (Use pointer) */
    int write_if_read(NMLmsg & nml_msg, int *serial_number = NULL);	/* Write only if buffer
						   was_read */
    int write_if_read(NMLmsg * nml_msg, int *serial_number = NULL);	/* '' */
    NMLTYPE blocking_read_extended(double timeout, double poll_interval);

    int write_subdivision(int subdiv, NMLmsg & nml_msg);	/* Write a
								   message.
								   (Use
								   reference) 
								 */
    int write_subdivision(int subdiv, NMLmsg * nml_msg);	/* Write a
								   message.
								   (Use
								   pointer) */
    int write_if_read_subdivision(int subdiv, NMLmsg & nml_msg);	/* Write 
									   only 
									   if 
									   buffer 
									   was_read 
									 */
    int write_if_read_subdivision(int subdiv, NMLmsg * nml_msg);	/* '' 
									 */
    NMLTYPE read_subdivision(int subdiv);	/* Read the buffer. */
    NMLTYPE blocking_read_subdivision(int subdiv, double timeout);	/* Read 
									   the 
									   buffer. 
									   (Wait 
									   for 
									   new 
									   data). 
									 */
    NMLTYPE peek_subdivision(int subdiv);	/* Read buffer without
						   changing was_read */
    NMLmsg *get_address_subdivision(int subdiv);
    int get_total_subdivisions();

    void clean_buffers();
    const char *msg2str(NMLmsg & nml_msg);
    const char *msg2str(NMLmsg * nml_msg);
    NMLTYPE str2msg(const char *);
    int login(const char *name, const char *passwd);
    void reconnect();
    void disconnect();

    /* Function to check to see if this NML object is properly configured. */
    int valid();

    /* Make just the check_if_read function from cms available to NML users. */
    int check_if_read();

    /* Make just the clear function from cms available to NML users. */
    int clear();

    /* Get the number of messages written to this buffer so far. */
    int get_msg_count();

    /* Get an approximate estimate of the space available to store messages
       in, for non queuing buffers this is just the fixed size of the buffer, 
       for queuing buffers it subtracts the space used by messages in the
       queue. */
    int get_space_available();

    /* How many messages are currently stored in the queue. */
    int get_queue_length();

    /* Get Diagnostics Information. */
    NML_DIAGNOSTICS_INFO *get_diagnostics_info();

    int prefix_format_chain(NML_FORMAT_PTR);

    /* Constructors and destructors. */
      NML(NML_FORMAT_PTR f_ptr,
	const char *, const char *, const char *, const int set_to_server = 0, const int set_to_master = 0);
      NML(NML *, const int set_to_server = 0, const int set_to_master = 0);
      NML(const char *buffer_line, const char *proc_line);
      virtual ~ NML();
    int reset();

    int queue_length;
    int print_queue_info();
    int set_error();
    void print_info(const char * const bufname = NULL, const char * const procname = NULL, const char * const cfg_file = NULL);
  protected:

    int fast_mode;
    int *cms_status;
    long *cms_inbuffer_header_size;
      NML(const char *, const char *, const char *, const int set_to_server = 0, const int set_to_master =
	0);
    void reconstruct(NML_FORMAT_PTR, const char *, const char *, const char *,
	const int set_to_server = 0, const int set_to_master = 0);

    int info_printed;

  public:
      NML_CHANNEL_TYPE channel_type;
    int forced_type;

  protected:
    int already_deleted;
    char bufname[40];
    char procname[40];
    char cfgfilename[160];
    double blocking_read_poll_interval;
    CMS *cms_for_msg_string_conversions;
    int registered_with_server;

  public:
    // Don't copy me.
    NML(const NML & nml) = delete;
    NML& operator=(const NML & nml) = delete;
};

extern LinkedList *NML_Main_Channel_List;
extern "C" {
    extern void nml_start();
    extern void nml_cleanup();
//    extern void nml_wipeout_lists();
    extern void set_default_nml_config_file(const char *);
    extern const char *get_default_nml_config_file();
    extern NML *nmlWaitOpen(NML_FORMAT_PTR fPtr, char *buffer, char *name, char *file, double sleepTime);

    extern void nmlSetHostAlias(const char * const hostName, const char * const hostAlias);
    extern void nmlClearHostAliases();
    extern void nmlAllowNormalConnection();
    extern void nmlForceRemoteConnection();
    extern void nmlForceLocalConnection();
} extern int verbose_nml_error_messages;
extern int nml_print_hostname_on_error;
extern int nml_reset_errors_printed;

#endif /* !NML_HH */
