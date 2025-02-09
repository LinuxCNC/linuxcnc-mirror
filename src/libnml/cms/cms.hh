/********************************************************************
* Description: cms.hh
*   C++ Header file for the Communication Management System (CMS).
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

#ifndef CMS_HH
#define CMS_HH

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>		/* size_t */

#ifdef __cplusplus
}
#endif
#include "linuxcnc.h"		/* LINELEN */

class PHYSMEM_HANDLE;
struct PM_CARTESIAN;
struct PM_CYLINDRICAL;
struct PM_EULER_ZYX;
struct PM_EULER_ZYZ;
struct PM_HOMOGENEOUS;
struct PM_POSE;
struct PM_QUATERNION;
struct PM_ROTATION_MATRIX;
struct PM_ROTATION_VECTOR;
struct PM_RPY;
struct PM_SPHERICAL;
class LinkedList;

enum CMS_STATUS {
/* ERROR conditions */
    CMS_MISC_ERROR = -1,	/* A miscellaneous error occurred. */
    CMS_UPDATE_ERROR = -2,	/* An error occurred during an update. */
    CMS_INTERNAL_ACCESS_ERROR = -3,	/* An error occurred during an
					   internal access function. */
    CMS_NO_MASTER_ERROR = -4,	/* An error occurred because the master was
				   not started */
    CMS_CONFIG_ERROR = -5,	/* There was an error in the configuration */
    CMS_TIMED_OUT = -6,		/* operation timed out. */
    CMS_QUEUE_FULL = -7,	/* A write failed because queuing was enabled 
				   but there was no room to add to the queue. 
				 */
    CMS_CREATE_ERROR = -8,	/* Something could not be created.  */
    CMS_PERMISSIONS_ERROR = -9,	/* Problem with permissions */
    CMS_NO_SERVER_ERROR = -10,	/* The server has not been started or could
				   not be contacted. */
    CMS_RESOURCE_CONFLICT_ERROR = -11,	/* Two or more CMS buffers are trying 
					   to use the same resource. */
    CMS_NO_IMPLEMENTATION_ERROR = -12,	/* An operation was attempted which
					   has not yet been implemented for
					   the current platform or protocol. */
    CMS_INSUFFICIENT_SPACE_ERROR = -13,	/* The size of the buffer was
					   insufficient for the requested
					   operation. */
    CMS_LIBRARY_UNAVAILABLE_ERROR = -14,	/* A DLL or Shared Object
						   library needed for the
						   current protocol could not 
						   be found or initialized. */
    CMS_SERVER_SIDE_ERROR = -15,	/* The server reported an error. */
    CMS_NO_BLOCKING_SEM_ERROR = -16,	/* A blocking_read operartion was
					   tried but no semaphore for the
					   blocking was configured or
					   available. */

/* NON Error Conditions.*/
    CMS_STATUS_NOT_SET = 0,	/* The status variable has not been set yet. */
    CMS_READ_OLD = 1,		/* Read successful, but data is old. */
    CMS_READ_OK = 2,		/* Read successful so far. */
    CMS_WRITE_OK = 3,		/* Write successful so far. */
    CMS_WRITE_WAS_BLOCKED = 4,	/* Write if read did not succeed, because the 
				   buffer had not been read yet. */
    CMS_CLEAR_OK = 5,		/* A clear operation was successful.  */
    CMS_CLOSED = 6		/* The channel has been closed.  */
};

/* Mode used within update functions. */
enum CMSMODE {
    CMS_NOT_A_MODE = 0,
    CMS_ENCODE,
    CMS_DECODE,
    CMS_RAW_OUT,
    CMS_RAW_IN,
    CMS_READ,
    CMS_WRITE
};

typedef long int CMSID;

/* Mode stored for use by the internal access function. */
enum CMS_INTERNAL_ACCESS_TYPE {
    CMS_ZERO_ACCESS = 0,
    CMS_READ_ACCESS,
    CMS_CHECK_IF_READ_ACCESS,
    CMS_PEEK_ACCESS,
    CMS_WRITE_ACCESS,
    CMS_WRITE_IF_READ_ACCESS,
    CMS_CLEAR_ACCESS,
    CMS_GET_MSG_COUNT_ACCESS,
    CMS_GET_DIAG_INFO_ACCESS,
    CMS_GET_QUEUE_LENGTH_ACCESS,
    CMS_GET_SPACE_AVAILABLE_ACCESS
};

/* What type of global memory buffer. */
enum CMS_BUFFERTYPE {
    CMS_SHMEM_TYPE,
    CMS_PHANTOM_BUFFER,
    CMS_LOCMEM_TYPE,
    CMS_FILEMEM_TYPE,
};

/* How will this process access the buffer. */
enum CMS_PROCESSTYPE {
    CMS_REMOTE_TYPE,
    CMS_LOCAL_TYPE,
    CMS_PHANTOM_USER,
    CMS_AUTO_TYPE
};

enum CMS_REMOTE_PORT_TYPE {
    CMS_NO_REMOTE_PORT_TYPE = 0,
    CMS_TTY_REMOTE_PORT_TYPE,
    CMS_TCP_REMOTE_PORT_TYPE,
    CMS_STCP_REMOTE_PORT_TYPE,
    CMS_UDP_REMOTE_PORT_TYPE
};

/* This structure will be placed at the beginning of every CMS buffer. */
struct CMS_HEADER {
    long was_read;		/* Has the buffer been read since the last
				   write? */
    long write_id;		/* Id of last write. */
    long in_buffer_size;	/* How much of the buffer is currently used. */
};

class CMS_DIAG_PROC_INFO;
class CMS_DIAG_HEADER;
class CMS_DIAGNOSTICS_INFO;

struct CMS_QUEUING_HEADER {
    long head;
    long tail;
    long queue_length;
    long end_queue_space;
    long write_id;
};

enum CMS_NEUTRAL_ENCODING_METHOD {
    CMS_NO_ENCODING,
    CMS_XDR_ENCODING,
    CMS_ASCII_ENCODING,
    CMS_DISPLAY_ASCII_ENCODING
};

/* CMS class declaration. */
class CMS;
class CMS_UPDATER;

/* CMS class definition. */
class CMS {
  public:
    void *operator                         new(size_t);
    void operator                         delete(void *);

  public:
    /* Constructors and Destructors. */
      CMS(long size);
      CMS(const char *bufline, const char *procline, int set_to_server = 0);
      virtual ~ CMS();

    // Don't copy me.
    CMS(const CMS & cms) = delete;
    CMS& operator=(const CMS & cms) = delete;

    /* Simple read/write interface functions. */
    virtual CMS_STATUS clear();	/* Has the buffer been read recently? */
    virtual int check_if_read();	/* Has the buffer been read recently? 
					 */
    virtual int get_msg_count();	/* Has the buffer been read recently? 
					 */
    virtual CMS_STATUS read();	/* Read from buffer. */
    virtual CMS_STATUS blocking_read(double _timeout);	/* Read from buffer,
							   wait for new data. 
							 */
    virtual CMS_STATUS peek();	/* Read without setting flag. */
    virtual CMS_STATUS write(void *user_data, int *serial_number = NULL);	/* Write to buffer. */
    virtual CMS_STATUS write_if_read(void *user_data, int *serial_number = NULL);	/* Write to buffer. */
    virtual int login(const char *name, const char *passwd);
    virtual void reconnect();
    virtual void disconnect();
    virtual int get_queue_length();
    virtual int get_space_available();

    /* Protocol Defined Virtual Function Stubs. */
    virtual CMS_STATUS main_access(void *_local, int *serial_number = NULL);

    /* Neutrally Encoded Buffer positioning functions. */
    void rewind();		/* positions at beginning */
    int get_encoded_msg_size();	/* Store last position in header.size */

    /* Buffer access control functions. */
    void set_mode(CMSMODE im);	/* Determine read/write mode.(check neutral) */

    /* Select a temporary updator -- This is used by the nml msg2string and
       string2msg functions. */
    void set_temp_updater(CMS_NEUTRAL_ENCODING_METHOD);

    /* Restore the normal update. */
    void restore_normal_updater();

  /*******************************************************/
    /* CMS INTERNAL ACCESS FUNCTIONS located in cms_in.cc */
  /*******************************************************/
    CMS_STATUS internal_access(PHYSMEM_HANDLE * _global, void *_local, int *serial_number);
    CMS_STATUS internal_access(void *_global, long global_size, void *_local, int *serial_number);
    CMS_STATUS internal_clear();	/* Zero the global memory.  */
    int check_if_read_raw();
    int check_if_read_encoded();
    int get_msg_count_raw();
    int get_msg_count_encoded();
    CMS_STATUS read_raw();	/* Read from raw buffers. */
    CMS_STATUS read_encoded();	/* Read from neutrally encoded buffers. */
    CMS_STATUS peek_raw();	/* Read without setting flags. */
    CMS_STATUS peek_encoded();	/* Read without setting flags. */
    CMS_STATUS write_raw(void *user_data, int *serial_number);	/* Write to raw buffers. */
    CMS_STATUS write_encoded();	/* Write to neutrally encoded buffers. */
    CMS_STATUS write_if_read_raw(void *user_data, int *serial_number);	/* Write if read. */
    CMS_STATUS write_if_read_encoded();	/* Write if read. */
    int queue_check_if_read_raw();
    int queue_check_if_read_encoded();
    int queue_get_msg_count_raw();
    int queue_get_msg_count_encoded();
    int queue_get_queue_length_raw();
    int queue_get_queue_length_encoded();
    int queue_get_space_available_raw();
    int queue_get_space_available_encoded();
    CMS_STATUS queue_read_raw();	/* Read from raw buffers. */
    CMS_STATUS queue_read_encoded();	/* Read from neutral buffers. */
    CMS_STATUS queue_peek_raw();	/* Read without setting flags. */
    CMS_STATUS queue_peek_encoded();	/* Read without setting flags. */
    CMS_STATUS queue_write_raw(void *user_data, int *serial_number);	/* Write to raw bufs */
    CMS_STATUS queue_write_encoded();	/* Write to neutral buffers. */
    CMS_STATUS queue_write_if_read_raw(void *user_data, int *serial_number);
    CMS_STATUS queue_write_if_read_encoded();	/* Write if read. */
    virtual void clean_buffers();

  /***********************************************/
    /* CMS UPDATE FUNCTIONS located in cms_up.cc */
  /***********************************************/
    /* Access functions for primitive C language data types */
    CMS_STATUS update(bool &x);
    CMS_STATUS update(char &x);                            /* Used by emc2 */
    CMS_STATUS update(unsigned char &x);                   /* Used by emc2 */
    CMS_STATUS update(short int &x);
    CMS_STATUS update(unsigned short int &x);
    CMS_STATUS update(int &x);                             /* Used by emc2 */
    CMS_STATUS update(unsigned int &x);
    CMS_STATUS update(long int &x);                        /* Used by emc2 */
    CMS_STATUS update(unsigned long int &x);               /* Used by emc2 */
    CMS_STATUS update(float &x);
    CMS_STATUS update(double &x);                          /* Used by emc2 */
    CMS_STATUS update(long double &x);
    CMS_STATUS update(char *x, unsigned int len);          /* Used by emc2 */
    CMS_STATUS update(unsigned char *x, unsigned int len); /* Used by emc2 */
    CMS_STATUS update(short *x, unsigned int len);
    CMS_STATUS update(unsigned short *x, unsigned int len);
    CMS_STATUS update(int *x, unsigned int len);           /* Used by emc2 */
    CMS_STATUS update(unsigned int *x, unsigned int len);
    CMS_STATUS update(long *x, unsigned int len);
    CMS_STATUS update(unsigned long *x, unsigned int len);
    CMS_STATUS update(float *x, unsigned int len);
    CMS_STATUS update(double *x, unsigned int len);        /* Used by emc2 */
    CMS_STATUS update(long double *x, unsigned int len);

  /*************************************************************************
   * CMS UPDATE FUNCTIONS for POSEMATH classes, defined in cms_pm.cc       *
   ************************************************************************/
    // translation types
    CMS_STATUS update(PM_CARTESIAN & x);	// Cart /* Used by emc2 */
    CMS_STATUS update(PM_SPHERICAL & x);	// Sph
    CMS_STATUS update(PM_CYLINDRICAL & x);	// Cyl

    // rotation types
    CMS_STATUS update(PM_ROTATION_VECTOR & x);	// Rot
    CMS_STATUS update(PM_ROTATION_MATRIX & x);	// Mat
    CMS_STATUS update(PM_QUATERNION & x);	// Quat
    CMS_STATUS update(PM_EULER_ZYZ & x);	// Zyz
    CMS_STATUS update(PM_EULER_ZYX & x);	// Zyx
    CMS_STATUS update(PM_RPY & x);	// Rpy

    // pose types
    CMS_STATUS update(PM_POSE & x);	// Pose
    CMS_STATUS update(PM_HOMOGENEOUS & x);	// Hom

    // CMS UPDATE FUNCTIONS for arrays of POSEMATH types.
    // translation types
    CMS_STATUS update(PM_CARTESIAN * x, int n);	// Cart
    CMS_STATUS update(PM_SPHERICAL * x, int n);	// Sph
    CMS_STATUS update(PM_CYLINDRICAL * x, int n);	// Cyl

    // rotation types
    CMS_STATUS update(PM_ROTATION_VECTOR * x, int n);	// Rot
    CMS_STATUS update(PM_ROTATION_MATRIX * x, int n);	// Mat
    CMS_STATUS update(PM_QUATERNION * x, int n);	// Quat
    CMS_STATUS update(PM_EULER_ZYZ * x, int n);	// Zyz
    CMS_STATUS update(PM_EULER_ZYX * x, int n);	// Zyx
    CMS_STATUS update(PM_RPY * x, int n);	// Rpy

    // pose types
    CMS_STATUS update(PM_POSE * x, int n);	// Pose
    CMS_STATUS update(PM_HOMOGENEOUS * x, int n);	// Hom

    /* comm protocol parameters shared by all protocols */
    int fatal_error_occurred;
    int consecutive_timeouts;
    CMS_HEADER header;		/* Information to be stored in CMS buffer. */
    int queuing_enabled;	/* queue messages in the buffer */
    CMS_QUEUING_HEADER queuing_header;	/* information for multi-slot
					   buffers.  */
    CMSMODE mode;		/* This process is reading or writing? */
    long size;			/* size of cms */
    long free_space;
    long max_message_size;	/* size of cms buffer available for user */
    /* messages = size - CMS Header space */
    long max_encoded_message_size;	/* Maximum size of message after
					   being encoded. */
    long guaranteed_message_space;	/* Largest size message before being
					   encoded that can be guaranteed to
					   fit after xdr. */
    int neutral;		/* neutral data format in buffer */

    CMS_STATUS status;		/* Status of the last CMS access. */
    void set_cms_status(CMS_STATUS);	/* Catch changes in cms status.  */
    int spawn_server;

    /* Buffers for local copies of global buffer. */
    void *encoded_data;		/* pointer to local copy of encoded data */
    int using_external_encoded_data;
    void set_encoded_data(void *, long _encoded_data_size);
    void *data;			/* pointer to local copy of data (raw) */
    void *subdiv_data;		/* pointer to current subdiv; */

    /* Interesting Info Saved from the Configuration File. */
    char BufferName[LINELEN];
    char BufferHost[LINELEN];
    char ProcessName[LINELEN];
    char BufferLine[LINELEN];
    char ProcessLine[LINELEN];
    char ProcessHost[LINELEN];
    char buflineupper[LINELEN];
    char proclineupper[LINELEN];
    char PermissionString[LINELEN];
    int is_local_master;
    int force_raw;
    bool serial;
    int split_buffer;		/* Will the buffer be split into two areas so 
				   that one area can be read while the other
				   is written to ? */
    char toggle_bit;
    int first_read_done;
    int first_write_done;
    int write_permission_flag;
    int read_permission_flag;
    unsigned long rpc_program_number;
    int tcp_port_number;
    int stcp_port_number;
    int udp_port_number;
    long buffer_number;
    int delete_totally;
    long total_messages_missed;
    long messages_missed_on_last_read;
    char *format_low_ptr;
    char *format_high_ptr;
    long format_size;
    int check_pointer(char *ptr, long bytes);
    int isserver;		/* Is the process a server. */
    int is_phantom;		/* Is this a phantom CMS channel? */
    CMS_BUFFERTYPE BufferType;
    CMS_PROCESSTYPE ProcessType;
    CMS_REMOTE_PORT_TYPE remote_port_type;
    int pointer_check_disabled;

    CMSID in_buffer_id;		/* Last id read, used to determine if new. */
    void *encoded_header;	/* pointer to local copy of encoded header */
    void *encoded_queuing_header;	/* pointer to local copy of encoded
					   queue info */
    long encoded_header_size;	/* Dynamically determined size */
    long encoded_queuing_header_size;	/* Dynamically determined size */

    /* Header Neutral Formatting Functions. */
    int encode_header();	/* header-> ENCODE-> encoded_header */
    int decode_header();	/* encoded_header -> DECODE -> header */
    int encode_queuing_header();	/* queuing_header ->
					   encoded_queuing_header */
    int decode_queuing_header();	/* encoded_queuing_header
					   ->queuing_header */
    /* XDR of ASCII */
    CMS_NEUTRAL_ENCODING_METHOD neutral_encoding_method;
    CMS_NEUTRAL_ENCODING_METHOD temp_updater_encoding_method;

  public:
    /* Type of internal access. */
      CMS_INTERNAL_ACCESS_TYPE internal_access_type;
    PHYSMEM_HANDLE *handle_to_global_data;
    PHYSMEM_HANDLE *dummy_handle;
    int write_just_completed;
    CMSMODE read_mode;
    CMSMODE write_mode;
    int read_updater_mode;
    int write_updater_mode;
    CMSMODE last_im;
    /* data buffer stuff */

    CMS_STATUS check_id(CMSID id);	/* Determine if the buffer is new. */
    friend class CMS_SERVER;
    friend class CMS_SERVER_HANDLER;
  public:
    double timeout;
    long connection_number;
    long total_connections;
    CMS_UPDATER *updater;
    CMS_UPDATER *normal_updater;
    CMS_UPDATER *temp_updater;

  private:
    unsigned long encode_state;	/* Store position for save, restore. */
    unsigned long decode_state;	/* Store position for save, restore. */
    void open(void);		/* Allocate memory and initialize XDR streams */
    static int number_of_cms_objects;	/* Used to decide when to initialize
					   and cleanup PC-NFS Toolkit DLLs */

  public:
    double blocking_timeout;
    double min_compatible_version;
    int confirm_write;
    int disable_final_write_raw_for_dma;
    virtual const char *status_string(int);

    int total_subdivisions;
    int current_subdivision;
    long subdiv_size;
    int set_subdivision(int _subdiv);
    long encoded_data_size;
    long enc_max_size;
    long enable_diagnostics;
    CMS_DIAG_PROC_INFO *dpi;
    virtual CMS_DIAG_PROC_INFO *get_diag_proc_info();
    virtual void set_diag_proc_info(CMS_DIAG_PROC_INFO *);
    virtual void setup_diag_proc_info();
    virtual void calculate_and_store_diag_info(PHYSMEM_HANDLE * _handle,
	void *);
    virtual void internal_retrieve_diag_info(PHYSMEM_HANDLE * _handle,
	void *);
    CMS_DIAGNOSTICS_INFO *di;
    virtual CMS_DIAGNOSTICS_INFO *get_diagnostics_info();
    int first_diag_store;
    double pre_op_total_bytes_moved;
    double time_bias;
    int skip_area;
    unsigned long half_offset;
    long half_size;
    int fast_mode;
    long size_without_diagnostics;
    int disable_diag_store;
    long diag_offset;
    int last_id_side0;
    int last_id_side1;
    int use_autokey_for_connection_number;
    /* RCS_CMD_MSG, RCS_STAT_MSG stuff */
};

class CMS_HOST_ALIAS_ENTRY {
  public:
    char host[64];
    char alias[64];
};

extern LinkedList *cmsHostAliases;

enum CMS_CONNECTION_MODE {
    CMS_NORMAL_CONNECTION_MODE = 0,	/* all config file parameters are
					   honored. */
    CMS_FORCE_LOCAL_CONNECTION_MODE = 1,	/* all connections are forced 
						   to be local */
    CMS_FORCE_REMOTE_CONNECTION_MODE = 2	/* all connections are forced 
						   to be remote */
};

extern CMS_CONNECTION_MODE cms_connection_mode;

extern char *cms_check_for_host_alias(char *in);
extern int cms_encoded_data_explosion_factor;
extern int cms_print_queue_free_space;
extern int cms_print_queue_full_messages;

#endif /* !defined(CMS_HH) */
