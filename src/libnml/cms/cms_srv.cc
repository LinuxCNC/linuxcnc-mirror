/********************************************************************
* Description: cms_srv.cc
*   Server to read and write to a local CMS buffer for remote processes.
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

#include <stdio.h>		/* sscanf(),NULL, FILE, fopen(), fgets() */
#include <string.h>		/* strchr(), memcpy() */
#include <stdlib.h>		/* malloc(), free(), exit() */
#include <ctype.h>		// isgraph()
#include <math.h>		/* fmod() */

#include <sys/types.h>
#include <unistd.h>		/* getpid() */
#include <sys/wait.h>		/* waitpid() */
#include <signal.h>		/* sigvec(), struct sigvec, SIGINT, kill() */

#ifdef __cplusplus
}
#endif
#include "cms.hh"		/* class CMS */
#include "rem_msg.hh"		/* struct REMOTE_READ_REQUEST, */
				/* struct REMOTE_WRITE_REQUEST, */
#include "cms_srv.hh"		/* class CMS_SERVER */
#include "cms_cfg.hh"		/* cms_config() */
#include "rcs_print.hh"		/* rcs_print_error() */
#ifndef NO_DCE_RPC
#define NO_DCE_RPC
#endif
#include "tcp_srv.hh"		/* CMS_SERVER_TCP_PORT */
#include "timer.hh"		// etime()
#include "cmsdiag.hh"

int cms_server_count = 0;
int cms_server_task_priority = 100;
int cms_server_task_stack_size = 32768;

void wait_for_servers(int count_to_waitfor)
{
    do {
	esleep(0.1);
    }
    while (cms_server_count < count_to_waitfor);

}

class CMS_USER_INFO {
  public:
    CMS_USER_INFO();
  private:
    char passwd[16];
    char epasswd[16];
    char name[16];
    char passwd_file_line[256];
    char key1[8];
    char key2[8];
    int has_passwd;
    int user_number;
    int allow_read;
    int allow_write;
    friend class CMS_SERVER;
};

CMS_USER_INFO::CMS_USER_INFO()
{
    memset(passwd, 0, 16);
    memset(epasswd, 0, 16);
    memset(name, 0, 16);
    memset(passwd_file_line, 0, 256);
    memset(key1, 0, 8);
    memset(key2, 0, 8);
    has_passwd = 0;
    user_number = 0;
    allow_read = 0;
    allow_write = 0;
}

CMS_USER_CONNECT_STRUCT::CMS_USER_CONNECT_STRUCT()
{
    user_info = NULL;
    fd = -1;
}

/* CMS_SERVER Global  Variables */
LinkedList *cms_server_list = NULL;

CMS_SERVER_LOCAL_PORT::CMS_SERVER_LOCAL_PORT(CMS * _cms)
  : list_id(0),
    orig_info(NULL),
    read_reply(),
    write_reply(),
    namereply(),
    get_diag_info_reply(),
    local_channel_reused(1)
{
    cms = _cms;
    if (NULL != cms) {
	buffer_number = cms->buffer_number;
    } else {
	buffer_number = 0;
    }
};

CMS_SERVER_LOCAL_PORT::~CMS_SERVER_LOCAL_PORT()
{
    if (NULL != orig_info) {
	delete orig_info;
	orig_info = NULL;
    }
}

/* local_port function for reads */
REMOTE_READ_REPLY *CMS_SERVER_LOCAL_PORT::reader(REMOTE_READ_REQUEST * /*_req*/)
{
    return (NULL);
}

REMOTE_READ_REPLY *CMS_SERVER_LOCAL_PORT::blocking_read(REMOTE_READ_REQUEST *
    /*_req*/)
{
    return (NULL);
}

/* local_port function for writes */
REMOTE_WRITE_REPLY *CMS_SERVER_LOCAL_PORT::writer(REMOTE_WRITE_REQUEST * /*_req*/)
{
    return (NULL);
}

REMOTE_SET_DIAG_INFO_REPLY *CMS_SERVER_LOCAL_PORT::
set_diag_info(REMOTE_SET_DIAG_INFO_REQUEST * /*_req*/)
{
    return (NULL);
}

REMOTE_GET_DIAG_INFO_REPLY *CMS_SERVER_LOCAL_PORT::
get_diag_info(REMOTE_GET_DIAG_INFO_REQUEST * /*_req*/)
{
    get_diag_info_reply.cdi = cms->get_diagnostics_info();
    get_diag_info_reply.status = cms->status;
    return (&get_diag_info_reply);
}

REMOTE_GET_MSG_COUNT_REPLY *CMS_SERVER_LOCAL_PORT::
get_msg_count(REMOTE_GET_DIAG_INFO_REQUEST * /*_req*/)
{
    return (NULL);
}

void CMS_SERVER_LOCAL_PORT::reset_diag_info()
{
}

CMS_SERVER_REMOTE_PORT::CMS_SERVER_REMOTE_PORT(CMS_SERVER *_cms_server_parent)
  : port_registered(0),
    current_user_info(NULL),
    connected_users(NULL),
    current_connected_user_struct(NULL),
    cms_server_parent(_cms_server_parent),
    min_compatible_version(0.0),
    confirm_write(0),
    running(0),
    max_total_subdivisions(_cms_server_parent->max_total_subdivisions),
    port_num(0),
    max_clients(0),
    current_clients(0)
{
}

CMS_SERVER_REMOTE_PORT::~CMS_SERVER_REMOTE_PORT()
{
    if (NULL != connected_users) {
	CMS_USER_CONNECT_STRUCT *connected_user_struct =
	    (CMS_USER_CONNECT_STRUCT *) connected_users->get_head();
	while (NULL != connected_user_struct) {
	    delete connected_user_struct;
	    connected_user_struct = NULL;
	    connected_users->delete_current_node();
	    connected_user_struct =
		(CMS_USER_CONNECT_STRUCT *) connected_users->get_next();
	}
	delete connected_users;
    }
    current_connected_user_struct = NULL;
}

void CMS_SERVER_REMOTE_PORT::add_connected_user(int _fd)
{
    current_connected_user_struct = NULL;
    rcs_print_debug(PRINT_SOCKET_CONNECT, "Adding connected user %d\n", _fd);
    CMS_USER_CONNECT_STRUCT *connected_user_struct =
	new CMS_USER_CONNECT_STRUCT();
    if (NULL == connected_user_struct) {
	return;
    }
    connected_user_struct->fd = _fd;
    if (NULL == connected_users) {
	connected_users = new LinkedList();
    }
    if (NULL == connected_users) {
	return;
    }
    connected_users->store_at_tail(connected_user_struct,
	sizeof(connected_user_struct), 0);
    current_connected_user_struct = connected_user_struct;
    // delete connected_user_struct;
}

CMS_USER_INFO *CMS_SERVER_REMOTE_PORT::get_connected_user(int _fd)
{
    current_connected_user_struct = NULL;
    if (NULL == connected_users) {
	connected_users = new LinkedList();
    }
    if (NULL == connected_users) {
	return NULL;
    }
    CMS_USER_CONNECT_STRUCT *connected_user_struct =
	(CMS_USER_CONNECT_STRUCT *) connected_users->get_head();
    while (NULL != connected_user_struct) {
	if (connected_user_struct->fd == _fd) {
	    current_connected_user_struct = connected_user_struct;
	    return connected_user_struct->user_info;
	}
	connected_user_struct =
	    (CMS_USER_CONNECT_STRUCT *) connected_users->get_next();
    }
    add_connected_user(_fd);
    return NULL;
}

void CMS_SERVER_REMOTE_PORT::register_port()
{
    return;
}

void CMS_SERVER_REMOTE_PORT::unregister_port()
{
    return;
}

int CMS_SERVER_REMOTE_PORT::accept_local_port_cms(CMS * _cms)
{
    if (NULL != _cms) {
	if (min_compatible_version < 1e-6 ||
	    (min_compatible_version > _cms->min_compatible_version &&
		_cms->min_compatible_version > 1e-6)) {
	    min_compatible_version = _cms->min_compatible_version;
	}
        if (_cms->total_subdivisions > max_total_subdivisions) {
            max_total_subdivisions = _cms->total_subdivisions;
        }
    }
    return 1;
}

CMS_SERVER *CMS_SERVER_REMOTE_PORT::find_server(long _pid, long _tid	/* =0 
									 */ )
{
    CMS_SERVER *cms_server;

    if (NULL == cms_server_list) {
	return NULL;
    }

    cms_server = (CMS_SERVER *) cms_server_list->get_head();
    while (NULL != cms_server) {
	if (cms_server->server_pid == _pid && cms_server->server_tid == _tid) {
	    break;
	}
	cms_server = (CMS_SERVER *) cms_server_list->get_next();
    }

    return (cms_server);
}

void CMS_SERVER_REMOTE_PORT::print_servers()
{
    CMS_SERVER *cms_server;

    if (NULL == cms_server_list) {
	rcs_print("cms_server_list is NULL.\n");
	return;
    }

    cms_server = (CMS_SERVER *) cms_server_list->get_head();
    rcs_print("Server Tasks for this remote port.\n");
    while (NULL != cms_server) {
	rcs_print(" \t(%ld (0x%lX), %ld (0x%lX))\n",
	    cms_server->server_pid, cms_server->server_pid,
	    cms_server->server_tid, cms_server->server_tid);

	cms_server = (CMS_SERVER *) cms_server_list->get_next();
    }
}
/*! \todo Another #if 0 */
#if 0
void CMS_SERVER::read_passwd_file()
{
    using_passwd_file = 1;
    int user_name_length;
    int passwd_length;
    if (NULL == known_users) {
	known_users = new LinkedList();
    }
    srand((int) ((2 ^ 32) * etime()));
    char buf[256];
    INET_FILE *ifp = NULL;
    rcs_print("Reading passwd file %s.\n", passwd_file);
    ifp = inet_file_open(passwd_file, "r");
    if (NULL == ifp) {
	rcs_print_error("Can not open passwd file %s.\n", passwd_file);
	return;
    }
    CMS_USER_INFO *user_info = NULL;

    while (!inet_file_eof(ifp)) {
	memset(buf, 0, 256);
	inet_file_gets(buf, 256, ifp);
	user_name_length = strcspn(buf, "\n\r \t:");
	if (user_name_length > 16) {
	    rcs_print_error("CMS_SERVER: user name is too long.\n");
	    continue;
	}
	if (user_name_length < 2) {
	    continue;
	}
	user_info = new CMS_USER_INFO();
	if (NULL == user_info) {
	    break;
	}
	rtapi_strxcpy(user_info->passwd_file_line, buf);
	memcpy(user_info->name, buf, user_name_length);
	passwd_length = strcspn(buf + user_name_length + 1, "\n\r \t:");
	if (passwd_length > 16) {
	    rcs_print_error("CMS_SERVER: password is too long.\n");
	    continue;
	}
	if (passwd_length > 16) {
	    rcs_print_error("CMS_SERVER: password is too long.\n");
	}
	if (passwd_length > 2) {
	    memcpy(user_info->passwd, buf + user_name_length + 1,
		passwd_length);
	    memcpy(user_info->key1, buf + user_name_length + 1, 2);
	    user_info->has_passwd = 1;
	} else {
	    user_info->has_passwd = 0;
	}
	gen_random_key(user_info->key2, 2);
	rtapi_strxcpy(user_info->epasswd,
	    crypt(user_info->passwd, user_info->key2));
	user_info->allow_read = (NULL != strstr(buf, "read=true"));
	user_info->allow_write = (NULL != strstr(buf, "write=true"));
	user_info->user_number =
	    known_users->store_at_tail(user_info, sizeof(user_info), 0);
	rcs_print("Storing info for user (%s).\n", user_info->name);
	// delete user_info;
	if (!strcmp(user_info->name, "guest")) {
	    guest_can_read = user_info->allow_read;
	    guest_can_write = user_info->allow_write;
	}
	user_info = NULL;
    }
}
#endif

void CMS_SERVER::gen_random_key(char key[], int len)
{
    for (int i = 0; i < len; i++) {
	while (!isgraph(key[i]) || !key[i]) {
	    key[i] = (char) ((rand() % 128));
	}
    }
}

CMS_USER_INFO *CMS_SERVER::find_user(const char *name)
{
    if (NULL == known_users) {
	return NULL;
    }
    CMS_USER_INFO *user_info = (CMS_USER_INFO *) known_users->get_head();
    while (NULL != user_info) {
	rcs_print("CMS_SERVER::find_user: strcmp(%s,%s)\n", name,
	    user_info->name);
	if (!strcmp(name, user_info->name)) {
	    return user_info;
	}
	user_info = (CMS_USER_INFO *) known_users->get_next();
    }
    rcs_print_error("CMS_SERVER: Can't find entry for user %s.\n", name);
    return NULL;
}
/*! \todo Another #if 0 */
#if 0
int CMS_SERVER::get_user_keys(const char *name, char *key1, char *key2)
{
    if (NULL == known_users) {
	gen_random_key(key1, 2);
	gen_random_key(key2, 2);
	return -1;
    }
    CMS_USER_INFO *user_info = find_user(name);
    if (NULL == user_info) {
	gen_random_key(key1, 2);
	gen_random_key(key2, 2);
	return -1;
    }
    rtapi_strxcpy(key1, user_info->key1);
    if (fabs(etime() - time_of_last_key_request) > 30.0) {
	memset(user_info->key2, 0, 8);
	memset(user_info->epasswd, 0, 16);
	gen_random_key(user_info->key2, 2);
	rtapi_strxcpy(user_info->epasswd,
	    crypt(user_info->passwd, user_info->key2));
    }
    rtapi_strxcpy(key2, user_info->key2);
    time_of_last_key_request = etime();
    return 0;
}
#endif

CMS_USER_INFO *CMS_SERVER::get_user_info(const char *name,
    const char *epasswd)
{
    if (NULL == known_users) {
	return NULL;
    }
    CMS_USER_INFO *user_info = find_user(name);
    if (NULL == user_info) {
	return NULL;
    }
    if (!strcmp(user_info->epasswd, epasswd) || !user_info->has_passwd) {
	return user_info;
    }
    rcs_print_error("CMS_SERVER: %s gave the wrong passwd.\n", name);
    rcs_print_error("CMS_SERVER: user_info->passwd = %s\n",
	user_info->passwd);
    rcs_print_error("CMS_SERVER: user_info->epasswd = %s\n",
	user_info->epasswd);
    rcs_print_error("CMS_SERVER: epasswd = %s\n", epasswd);

    return NULL;
}

void CMS_SERVER::add_local_port(CMS_SERVER_LOCAL_PORT * _local_port)
{
    if (NULL == _local_port) {
	rcs_print_error("CMS_SERVER: Attempt to add NULL local port.\n");
	return;
    }
    if (NULL == _local_port->cms) {
	rcs_print_error
	    ("CMS_SERVER: Attempt to add local port with NULL cms object.\n");
	return;
    }
    if (NULL == cms_local_ports) {
	rcs_print_error
	    ("CMS_SERVER: Attempt to add local port when local ports list is NULL.\n");
	return;
    }

    if (NULL == remote_port) {
	switch (_local_port->cms->remote_port_type) {

	case CMS_TCP_REMOTE_PORT_TYPE:
	    remote_port = new CMS_SERVER_REMOTE_TCP_PORT(this);
	    break;
/*! \todo Another #if 0 */
#if 0
	case CMS_STCP_REMOTE_PORT_TYPE:
	    remote_port = new CMS_SERVER_REMOTE_STCP_PORT(this);
	    break;
	case CMS_TTY_REMOTE_PORT_TYPE:
	    remote_port = new CMS_SERVER_REMOTE_TTY_PORT(this);
	    break;
	case CMS_UDP_REMOTE_PORT_TYPE:
	    remote_port = new CMS_SERVER_REMOTE_UDP_PORT(this);
	    break;
#endif
	default:
	    rcs_print_error("CMS_SERVER: Invalid remote port type. (%d)\n",
		_local_port->cms->remote_port_type);
	    return;
	}
    }
    if (NULL == remote_port) {
	rcs_print_error("CMS_SERVER: couldn't create remote port object.\n");
	return;
    }
    if (!accept_local_port_cms(_local_port->cms)) {
	rcs_print_error
	    ("CMS_SERVER: Attempt to add local port failed because the port was of an incompatible type.\n");
    }
    char *passwd_eq = strstr(_local_port->cms->BufferLine, "passwd=");
    if (NULL != passwd_eq) {
	if (!using_passwd_file) {
	    memset(passwd_file, 0, 256);
	    for (int i = 0; i < 256 && passwd_eq[i + 7]; i++) {
		if (passwd_eq[i + 7] == ' ' || passwd_eq[i + 7] == '\t'
		    || passwd_eq[i + 7] == '\n' || passwd_eq[i + 7] == '\r') {
		    break;
		}
		passwd_file[i] = passwd_eq[i + 7];
	    }
/*! \todo Another #if 0 */
#if 0
	    if (strlen(passwd_file) > 0) {
		read_passwd_file();
	    }
#endif
	}
    }

    _local_port->list_id =
	cms_local_ports->store_at_tail(_local_port,
	sizeof(CMS_SERVER_LOCAL_PORT), 0);
    if (-1 == _local_port->list_id) {
	rcs_print_error
	    ("CMS_SERVER: Can not store local port on linked list.\n");
    }
}

int CMS_SERVER::accept_local_port_cms(CMS * _cms)
{
    if (NULL == remote_port || NULL == _cms) {
	return (0);
    }

    return (remote_port->accept_local_port_cms(_cms));
}

CMS_SERVER_LOCAL_PORT *CMS_SERVER::find_local_port(long _buffer_number)
{
    CMS_SERVER_LOCAL_PORT *cms_local_port;
    cms_local_port = (CMS_SERVER_LOCAL_PORT *) cms_local_ports->get_head();
    while (NULL != cms_local_port) {
	if (cms_local_port->buffer_number == _buffer_number) {
	    break;
	}
	cms_local_port =
	    (CMS_SERVER_LOCAL_PORT *) cms_local_ports->get_next();
    }
    return (cms_local_port);
}

int CMS_SERVER::get_total_subdivisions(long _buffer_number)
{
    CMS_SERVER_LOCAL_PORT *cms_local_port = find_local_port(_buffer_number);
    if (NULL == cms_local_port) {
	return 1;
    }
    if (NULL == cms_local_port->cms) {
	return 1;
    }
    return cms_local_port->cms->total_subdivisions;
}

void CMS_SERVER::set_diag_info(REMOTE_SET_DIAG_INFO_REQUEST * _diag_info)
{
    diag_enabled = 1;
    CMS_SERVER_LOCAL_PORT *local_port =
	find_local_port(_diag_info->buffer_number);
    if (NULL == local_port) {
	rcs_print_error
	    ("CMS_SERVER: Cannot find local port for buffer number %ld\n",
	    _diag_info->buffer_number);
	return;
    }
    local_port->set_diag_info(_diag_info);
    last_local_port_used = local_port;
}

void CMS_SERVER::reset_diag_info(int buffer_number)
{
    diag_enabled = 0;
    CMS_SERVER_LOCAL_PORT *local_port = find_local_port(buffer_number);
    if (NULL == local_port) {
	rcs_print_error
	    ("CMS_SERVER: Cannot find local port for buffer number %d\n",
	    buffer_number);
	return;
    }
    local_port->reset_diag_info();
    last_local_port_used = NULL;
}

REMOTE_CMS_REPLY *CMS_SERVER::process_request(REMOTE_CMS_REQUEST * _request)
{
    CMS_SERVER_LOCAL_PORT *local_port;

    requests_processed++;

    request = _request;
    if (NULL == request) {
	rcs_print_error("CMS_SERVER: Request is NULL.\n");
	return NULL;
    }

    local_port = find_local_port(request->buffer_number);
    last_local_port_used = local_port;
    if (NULL == local_port) {
	rcs_print_error
	    ("CMS_SERVER: Cannot find local port for buffer number %ld\n",
	    request->buffer_number);
	return (NULL);
    }
    if (!security_check
	(remote_port->current_user_info, request->buffer_number)) {
	return NULL;
    }

    local_port->cms->set_subdivision(_request->subdiv);
    _request->subdiv = 0;

    switch (request->type) {
    case REMOTE_CMS_GET_BUF_NAME_REQUEST_TYPE:
	{
	    REMOTE_GET_BUF_NAME_REPLY *namereply = &local_port->namereply;
	    const char *name = get_buffer_name(request->buffer_number);
	    if (0 == name) {
		return NULL;
	    }
	    strncpy(namereply->name, name, 31);
	    return namereply;
	}

    case REMOTE_CMS_READ_REQUEST_TYPE:
	return (local_port->reader((REMOTE_READ_REQUEST *) request));
    case REMOTE_CMS_GET_DIAG_INFO_REQUEST_TYPE:
	return (local_port->get_diag_info
	    ((REMOTE_GET_DIAG_INFO_REQUEST *) request));
    case REMOTE_CMS_BLOCKING_READ_REQUEST_TYPE:
	return (local_port->blocking_read((REMOTE_READ_REQUEST *) request));
    case REMOTE_CMS_WRITE_REQUEST_TYPE:
	return (local_port->writer((REMOTE_WRITE_REQUEST *) request));
    case REMOTE_CMS_CHECK_IF_READ_REQUEST_TYPE:
	if (NULL == local_port->cms) {
	    rcs_print_error
		("CMS_SERVER: cms object associated with local port is NULL.\n");
	    return (NULL);
	}
	cir_reply.was_read = local_port->cms->check_if_read();
	cir_reply.status = local_port->cms->status;
	return (&cir_reply);

    case REMOTE_CMS_GET_MSG_COUNT_REQUEST_TYPE:
	if (NULL == local_port->cms) {
	    rcs_print_error
		("CMS_SERVER: cms object associated with local port is NULL.\n");
	    return (NULL);
	}
	gmc_reply.count = local_port->cms->get_msg_count();
	gmc_reply.status = local_port->cms->status;
	return (&gmc_reply);

    case REMOTE_CMS_GET_QUEUE_LENGTH_REQUEST_TYPE:
	if (NULL == local_port->cms) {
	    rcs_print_error
		("CMS_SERVER: cms object associated with local port is NULL.\n");
	    return (NULL);
	}
	gql_reply.queue_length = local_port->cms->get_queue_length();
	gql_reply.status = local_port->cms->status;
	return (&gql_reply);

    case REMOTE_CMS_GET_SPACE_AVAILABLE_REQUEST_TYPE:
	if (NULL == local_port->cms) {
	    rcs_print_error
		("CMS_SERVER: cms object associated with local port is NULL.\n");
	    return (NULL);
	}
	gsa_reply.space_available = local_port->cms->get_space_available();
	gsa_reply.status = local_port->cms->status;
	return (&gsa_reply);

    case REMOTE_CMS_CLEAR_REQUEST_TYPE:
	if (NULL == local_port->cms) {
	    rcs_print_error
		("CMS_SERVER: cms object associated with local port is NULL.\n");
	    return (NULL);
	}
	local_port->cms->clear();
	clear_reply_struct.status = local_port->cms->status;
	return (&clear_reply_struct);
/*! \todo Another #if 0 */
#if 0
    case REMOTE_CMS_GET_KEYS_REQUEST_TYPE:
	get_keys_reply = &perm_get_keys_reply;
	get_user_keys(
	    ((REMOTE_GET_KEYS_REQUEST *) request)->name,
	    get_keys_reply->key1, get_keys_reply->key2);
	return (&perm_get_keys_reply);
#endif
    case REMOTE_CMS_LOGIN_REQUEST_TYPE:
	login_reply = &perm_login_reply;
	if (NULL == remote_port->current_connected_user_struct) {
	    login_reply->success = 0;
	    return (&perm_login_reply);
	}
	remote_port->current_connected_user_struct->user_info =
	    get_user_info(
	    ((REMOTE_LOGIN_REQUEST *) request)->name,
	    ((REMOTE_LOGIN_REQUEST *) request)->passwd);
	login_reply->success =
	    (NULL != remote_port->current_connected_user_struct->user_info);
	if (login_reply->success) {
	    rcs_print("%s logged in.\n",
		remote_port->current_connected_user_struct->user_info->name);
	}
	return (&perm_login_reply);

    case REMOTE_CMS_SET_SUBSCRIPTION_REQUEST_TYPE:
	set_subscription_reply = &perm_set_subscription_reply;
	set_subscription_reply->success = 1;
	return (&perm_set_subscription_reply);

    default:
	rcs_print_error("CMS_SERVER: Invalid request type (%d)\n",
	    request->type);
	return (NULL);
    }
}

/* Spawning Routine. */
int CMS_SERVER::spawn()
{
    if (0 == server_spawned) {
	if (NULL != remote_port) {
	    remote_port->running = 0;
	}
	server_spawned = 1;
	current_pid = spawner_pid = getpid();
	if (0 == (server_pid = fork())) {
	    /* Child */
	    run();		/* This will only return if an error occurs. */
	    clean(2);
	    exit(-1);
	} else {
	    /* Parent */
	}
	int waits = 0;
	while (waits < 20) {
	    esleep(0.01);
	    if (NULL == remote_port) {
		break;
	    }
	    if (remote_port->running) {
		break;
	    }
	    waits++;
	}
	return 1;
    }
    return 0;
}

void CMS_SERVER::kill_server()
{
    if (0 != server_pid) {
	signal(SIGINT, SIG_DFL);
	cms_server_count--;
	kill(server_pid, SIGINT);
	waitpid(server_pid, NULL, 0);
	server_pid = 0;
    }
}

/* MAIN ROUTINE */
void CMS_SERVER::register_server(int setup_CC_signal_local_port)
{
    last_local_port_used = NULL;
    server_registered = 1;

    if (NULL == cms_server_list) {
	cms_server_list = new LinkedList;
    }
    list_id = cms_server_list->store_at_tail(this, sizeof(CMS_SERVER), 0);

    /* Set up interrupt local_port. */
    if (setup_CC_signal_local_port) {
	signal(SIGINT, clean);	/* Set up interrupt local_port. */
    }

    if (NULL == remote_port) {
	rcs_print_error
	    ("CMS_SERVER: Can't register with NULL remote port.\n");
	return;
    }
    remote_port->register_port();

}

void CMS_SERVER::run(int setup_CC_signal_local_port)
{
    server_tid = current_tid = 0;
    current_pid = server_pid = getpid();
    if (!server_registered) {
	register_server(setup_CC_signal_local_port);
    }
    initialize_write_request_space();
    if (NULL == remote_port) {
	rcs_print_error
	    ("CMS_SERVER: Cannot run with remote port equal to NULL.\n");
	return;
    }
    remote_port->running = 1;
    if (remote_port->port_registered) {
	remote_port->run();
    }
}

void CMS_SERVER::initialize_write_request_space()
{
    max_total_subdivisions = 1;
    maximum_cms_size = 1;
    CMS_SERVER_LOCAL_PORT *local_port;
    if (NULL == cms_local_ports) {
	rcs_print_error
	    ("CMS_SERVER: Can not search list of local ports to determine the size of space needed for the write request\n"
	    "because the list is NULL.\n");
	return;
    }
    local_port = (CMS_SERVER_LOCAL_PORT *) cms_local_ports->get_head();
    while (NULL != local_port) {
	if (NULL != local_port->cms) {
	    if (local_port->cms->size > maximum_cms_size) {
		maximum_cms_size = local_port->cms->size;
	    }
	    if (local_port->cms->total_subdivisions > max_total_subdivisions) {
		max_total_subdivisions = local_port->cms->total_subdivisions;
	    }
	    if (NULL != remote_port) {
		if (local_port->cms->total_subdivisions >
		    remote_port->max_total_subdivisions) {
		    remote_port->max_total_subdivisions =
			local_port->cms->total_subdivisions;
		}
	    }
	    if (local_port->cms->max_encoded_message_size > maximum_cms_size) {
		maximum_cms_size = local_port->cms->max_encoded_message_size;
	    }
	}
	local_port = (CMS_SERVER_LOCAL_PORT *) cms_local_ports->get_next();
    }
    if (NULL != write_req.data) {
	free(write_req.data);
	write_req.data = NULL;
    }
    write_req.data = malloc(maximum_cms_size);
    if (NULL == write_req.data) {
	rcs_print_error("malloc(%ld) failed.\n", maximum_cms_size);
    }
    local_port = (CMS_SERVER_LOCAL_PORT *) cms_local_ports->get_head();
    while (NULL != local_port) {
	if (NULL != local_port->cms) {
	    local_port->cms->set_encoded_data(write_req.data,
		maximum_cms_size);
	}
	local_port = (CMS_SERVER_LOCAL_PORT *) cms_local_ports->get_next();
    }
}

CMS_SERVER::CMS_SERVER()
  : request(NULL),
    server_spawned(0),
    server_registered(0),
    list_id(0),
    cir_reply(),
    gmc_reply(),
    gql_reply(),
    gsa_reply(),
    clear_reply_struct(),
    using_passwd_file(0),
    requests_processed(0),
    remote_port(NULL),
    current_pid(0), current_tid(0),
    creator_tid(0),
    spawner_pid(0), spawner_tid(0),
    server_pid(0), server_tid(0),
    maximum_cms_size(0),
    read_req(),
    write_req(),
    get_keys_req(),
    login_req(),
    set_subscription_req(),
    check_if_read_req(),
    get_msg_count_req(),
    get_queue_length_req(),
    get_space_available_req(),
    clear_req(),
    set_diag_info_req(),
    get_diag_info_req(),
    read_reply(NULL),
    write_reply(NULL),
    get_keys_reply(NULL),
    perm_get_keys_reply(),
    login_reply(NULL),
    perm_login_reply(),
    set_subscription_reply(NULL),
    perm_set_subscription_reply(),
    check_if_read_reply(NULL),
    get_msg_count_reply(NULL),
    get_queue_length_reply(NULL),
    get_space_available_reply(NULL),
    clear_reply(NULL),
    set_diag_info_reply(NULL),
    get_diag_info_reply(NULL),
    last_local_port_used(NULL),
    diag_enabled(0),
    set_diag_info_buf{},
    max_total_subdivisions(1),
    time_of_last_key_request(0.0),
    known_users(NULL),
    passwd_file{},
    guest_can_read(0),
    guest_can_write(0)
{
    cms_local_ports = new LinkedList;
    creator_pid = getpid();
}

CMS_SERVER::~CMS_SERVER()
{
    last_local_port_used = NULL;
    if (server_registered && (!server_spawned || current_pid == server_pid)) {
	unregister_server();
    } else if (server_spawned && current_pid == spawner_pid) {
	kill_server();
    }
    delete_all_local_ports();
    if (NULL != remote_port) {
	delete remote_port;
	remote_port = NULL;
    }
    if (NULL != cms_local_ports) {
	delete cms_local_ports;
	cms_local_ports = NULL;
    }
    // Leave this to NML_SERVER destructor.
    // delete_from_list();

    if (NULL != write_req.data) {
	free(write_req.data);
	write_req.data = NULL;
    }
}

void CMS_SERVER::delete_all_local_ports()
{
    if (NULL != cms_local_ports) {
	CMS_SERVER_LOCAL_PORT *local_port;
	local_port = (CMS_SERVER_LOCAL_PORT *) cms_local_ports->get_head();
	while (NULL != local_port) {
	    delete local_port;
	    cms_local_ports->delete_current_node();
	    local_port =
		(CMS_SERVER_LOCAL_PORT *) cms_local_ports->get_next();
	}
    }
}

static int last_cms_server_signum = 0;

void CMS_SERVER::clean(int signum)
{
    last_cms_server_signum = signum;
    pid_t current_pid;
    pid_t current_tid = 0;
    current_pid = getpid();
    CMS_SERVER *cms_server = NULL;

    cms_server = (CMS_SERVER *) cms_server_list->get_head();
    while (NULL != cms_server) {
	if (cms_server->server_pid == current_pid
	    && cms_server->server_tid == current_tid) {
	    cms_server->unregister_server();
	    delete cms_server;
	    cms_server = NULL;
	}
	cms_server = (CMS_SERVER *) cms_server_list->get_next();
    }

    exit(0);
}

void CMS_SERVER::unregister_server()
{
    if (server_registered) {
	server_registered = 0;
	if (NULL != remote_port) {
	    remote_port->unregister_port();
	}
    }
}

void CMS_SERVER::delete_from_list()
{
    current_pid = getpid();
    current_tid = 0;

    if (current_pid == server_pid && current_tid == server_tid) {
	if (NULL != cms_server_list && list_id > 0) {
	    cms_server_list->delete_node(list_id);
	    list_id = -1;
	}
    }
}

const char *CMS_SERVER::get_buffer_name(int buffer_number)
{
    CMS_SERVER_LOCAL_PORT *local_port;
    local_port = find_local_port(buffer_number);
    if (NULL == local_port) {
	return NULL;
    }
    return (const char *) local_port->cms->BufferName;
}

long CMS_SERVER::get_message_type()
{
    return -1;
    // I need to be overloaded.
}

int CMS_SERVER::get_access_type()
{
    if (NULL == request) {
	return -1;
    }
    return request->type;
}

int CMS_SERVER::security_check(CMS_USER_INFO * user_info, int buffer_number)
{
    if (!using_passwd_file) {
	return 1;
    }
    if (request->type == REMOTE_CMS_GET_KEYS_REQUEST_TYPE ||
	request->type == REMOTE_CMS_LOGIN_REQUEST_TYPE) {
	return 1;
    }

    if (NULL == user_info) {

	if (guest_can_read && (request->type == REMOTE_CMS_READ_REQUEST_TYPE
		|| request->type ==
		REMOTE_CMS_SET_SUBSCRIPTION_REQUEST_TYPE)) {
	    return 1;
	}

	if (guest_can_write && request->type == REMOTE_CMS_WRITE_REQUEST_TYPE) {
	    return 1;
	}
	rcs_print_error
	    ("CMS_SERVER: Refusing to process request of unknown user.\n");
	return 0;
    }

    if (user_info->allow_read
	&& (request->type == REMOTE_CMS_READ_REQUEST_TYPE
	    || request->type == REMOTE_CMS_SET_SUBSCRIPTION_REQUEST_TYPE)) {
	return 1;
    }

    if (user_info->allow_write
	&& request->type == REMOTE_CMS_WRITE_REQUEST_TYPE) {
	return 1;
    }

    if (NULL != detailed_security_check) {
	return detailed_security_check(user_info->name,
	    get_buffer_name(buffer_number),
	    get_message_type(), get_access_type());
    }

    if (!user_info->allow_read
	&& request->type == REMOTE_CMS_READ_REQUEST_TYPE) {
	rcs_print_error("CMS_SERVER:: %s does not have read permission.", user_info->name);
	return 0;
    }

    if (!user_info->allow_write
	&& request->type == REMOTE_CMS_WRITE_REQUEST_TYPE) {
	rcs_print_error("CMS_SERVER:: %s does not have write permission.", user_info->name);
	return 0;
    }
    return 1;

}

int (*detailed_security_check) (const char *user_name,
    const char *buffer_name, long msg_type, int access_type) = NULL;
