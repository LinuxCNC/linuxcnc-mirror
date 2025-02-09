/********************************************************************
* Description: nml_srv.cc
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

#include <string.h>		// memcpy()

#include <signal.h>		// kill()
#include <sys/types.h>
#include <unistd.h>		// getpid()
#include <sys/wait.h>		// waitpid()
#include <stdlib.h>		// atexit()

#ifdef __cplusplus
}
#endif
#include <rtapi_string.h>	// rtapi_strlcpy()
#include "nml.hh"
#include "nmlmsg.hh"
#include "cms.hh"
#include "nml_srv.hh"
#include "rem_msg.hh"		// struct REMOTE_READ_REQUEST
#include "rcs_print.hh"		// rcs_print_error()
#include "timer.hh"		// esleep()
#include "rcs_exit.hh"		// rcs_exit
#include "linklist.hh"
#include "physmem.hh"
#include "cmsdiag.hh"
#include "cmd_msg.hh"		// layering violation ahoy
NML_SERVER::NML_SERVER(NML * _nml, int _set_to_master):CMS_SERVER()
{
    NML_SERVER_LOCAL_PORT *new_local_port = NULL;
    being_deleted = 0;
    if (NULL != _nml) {
	if (NULL != _nml->cms) {
	    if (CMS_REMOTE_TYPE != _nml->cms->ProcessType) {
		NML *new_nml;
		if (_nml->cms->isserver &&
		    (0 == _set_to_master ||
			(_nml->cms->is_local_master == 1
			    && _set_to_master == 1)
			|| (_nml->cms->is_local_master == 0
			    && _set_to_master == -1))) {
		    new_nml = _nml;
		    if (NULL != new_nml) {
			new_local_port = new NML_SERVER_LOCAL_PORT(new_nml);
			add_local_port(new_local_port);
		    }
		    new_local_port->local_channel_reused = 1;
		} else {
		    new_nml = new NML(_nml, 1, -1);
		    if (NULL != new_nml) {
			new_local_port = new NML_SERVER_LOCAL_PORT(new_nml);
			add_local_port(new_local_port);
		    }
		    new_local_port->local_channel_reused = 0;
		}
	    } else {
		rcs_print_error
		    ("NML_SERVER:(ERROR) ProcessType was REMOTE.\n");
		_nml = NULL;
	    }
	} else {
	    rcs_print_error("NML_SERVER:(ERROR) cms was NULL.\n");
	}
    } else {
	rcs_print_error("NML_SERVER:(ERROR) nml_ptr was NULL.\n");
    }
    add_to_nml_server_list();
}

void NML_SERVER::add_to_nml_server_list()
{
    if (NULL == NML_Default_Super_Server) {
	NML_Default_Super_Server = new NML_SUPER_SERVER;
    }
    if (NULL != NML_Default_Super_Server) {
	NML_Default_Super_Server->add_to_list(this);
    }
}

NML_SERVER::~NML_SERVER()
{
    being_deleted = 1;
    delete_from_list();
}

void NML_SERVER::delete_from_list()
{
    CMS_SERVER::delete_from_list();
    if (NULL != NML_Default_Super_Server) {
	if (NULL != NML_Default_Super_Server->servers) {
	    NML_Default_Super_Server->servers->
		delete_node(super_server_list_id);
	}
    }
}

NML_SERVER_LOCAL_PORT::NML_SERVER_LOCAL_PORT(NML * _nml)
  : CMS_SERVER_LOCAL_PORT((CMS *)NULL),
    batch_list_id(0)
{
    local_channel_reused = 1;
    nml = _nml;
    if (NULL != nml) {
	cms = nml->cms;
	if (NULL != cms) {
	    buffer_number = cms->buffer_number;
	}
    }
}

NML_SERVER_LOCAL_PORT::~NML_SERVER_LOCAL_PORT()
{
    if (NULL != nml && !local_channel_reused) {
	delete nml;
    }
    nml = NULL;
    cms = (CMS *) NULL;
}

REMOTE_READ_REPLY *NML_SERVER_LOCAL_PORT::reader(REMOTE_READ_REQUEST * _req)
{
    if ((NULL == cms) || (NULL == nml)) {
	rcs_print_error("NMLserver:reader: CMS object is NULL.\n");
	return ((REMOTE_READ_REPLY *) NULL);
    }

    /* Setup CMS channel from request arguments. */
    cms->in_buffer_id = _req->last_id_read;

    /* Read and encode the buffer. */
    switch (_req->access_type) {
    case CMS_READ_ACCESS:
	nml->read();
	break;
    case CMS_PEEK_ACCESS:
	nml->peek();
	break;
    default:
	rcs_print_error("NML_SERVER: Invalid access type.(%d)\n",
	    _req->access_type);
	break;
    }

    /* Setup reply structure to be returned to remote process. */
    read_reply.status = (int) cms->status;
    if (cms->status == CMS_READ_OLD) {
	read_reply.size = 0;
	read_reply.data = NULL;
	read_reply.write_id = _req->last_id_read;
	read_reply.was_read = 1;
    } else {
	read_reply.size = cms->header.in_buffer_size;
	read_reply.data = (unsigned char *) cms->encoded_data;
	read_reply.write_id = cms->in_buffer_id;
	read_reply.was_read = cms->header.was_read;
    }

    /* Reply structure contains the latest shared memory info-- now return it 
       to cms_dispatch for return to caller */
    return (&read_reply);
}

REMOTE_READ_REPLY *NML_SERVER_LOCAL_PORT::blocking_read(REMOTE_READ_REQUEST *
    _req)
{
    if ((NULL == cms) || (NULL == nml)) {
	rcs_print_error("NMLserver:blocking_read: CMS object is NULL.\n");
	return ((REMOTE_READ_REPLY *) NULL);
    }
    nml->cms->first_diag_store = 0;
    if (_req->type != REMOTE_CMS_BLOCKING_READ_REQUEST_TYPE) {
	rcs_print_error
	    ("NMLserver::blocking_read: Invalid request type(%d)\n",
	    _req->type);
	return NULL;
    }
    double orig_bytes_moved = 0.0;

    REMOTE_BLOCKING_READ_REQUEST *breq =
	(REMOTE_BLOCKING_READ_REQUEST *) _req;
    breq->_nml = new NML(nml, 1, -1);

    NML *nmlcopy = breq->_nml;
    if (NULL == nmlcopy) {
	rcs_print_error("NMLserver:blocking_read: NML object is NULL.\n");
	return ((REMOTE_READ_REPLY *) NULL);
    }

    CMS *cmscopy = nmlcopy->cms;
    if (NULL == cmscopy) {
	rcs_print_error("NMLserver:blocking_read: CMS object is NULL.\n");
	return ((REMOTE_READ_REPLY *) NULL);
    }

    double blocking_timeout = (double) (breq->timeout_millis / 1000.0);
    REMOTE_READ_REPLY *temp_read_reply = new REMOTE_READ_REPLY();
    breq->_reply = temp_read_reply;
    long data_size = (long) cmscopy->max_encoded_message_size;
    temp_read_reply->data = malloc(data_size);
    breq->_data = temp_read_reply->data;
    if (NULL != cmscopy->handle_to_global_data) {
	orig_bytes_moved = cmscopy->handle_to_global_data->total_bytes_moved;
    }
    if (NULL == temp_read_reply->data) {
	rcs_print_error
	    ("NMLserver:blocking_read: temp_read_reply->data object is NULL.\n");
	return ((REMOTE_READ_REPLY *) NULL);
    }
    nmlcopy->cms->set_encoded_data(temp_read_reply->data, data_size);

    /* Setup CMS channel from request arguments. */
    cmscopy->in_buffer_id = _req->last_id_read;

    /* Read and encode the buffer. */
    nmlcopy->blocking_read(blocking_timeout);

    /* Setup reply structure to be returned to remote process. */
    temp_read_reply->status = (int) cmscopy->status;
    if (cmscopy->status == CMS_READ_OLD) {
	temp_read_reply->size = 0;
	if (NULL != temp_read_reply->data) {
	    breq->_data = NULL;
	    free(temp_read_reply->data);
	    temp_read_reply->data = NULL;
	}
	temp_read_reply->write_id = _req->last_id_read;
	temp_read_reply->was_read = 1;
    } else {
	temp_read_reply->size = cmscopy->header.in_buffer_size;
	temp_read_reply->write_id = cmscopy->in_buffer_id;
	temp_read_reply->was_read = cmscopy->header.was_read;
    }
    if (NULL != nml->cms->handle_to_global_data &&
	NULL != cmscopy->handle_to_global_data) {
	nml->cms->handle_to_global_data->total_bytes_moved
	    +=
	    (cmscopy->handle_to_global_data->total_bytes_moved -
	    orig_bytes_moved);
	nml->cms->first_diag_store = cmscopy->first_diag_store;
    }
    delete breq->_nml;
    breq->_nml=nmlcopy=NULL;

    /* Reply structure contains the latest shared memory info-- now return it 
       to cms_dispatch for return to caller */
    return (temp_read_reply);
}

/* REMOTE local_port function for writes */
REMOTE_WRITE_REPLY *NML_SERVER_LOCAL_PORT::writer(REMOTE_WRITE_REQUEST * _req)
{
    NMLmsg *temp;		/* Temporary Pointer */

    if ((NULL == cms) || (NULL == nml)) {
	rcs_print_error("NMLserver:writer: CMS object is NULL.\n");
	return ((REMOTE_WRITE_REPLY *) NULL);
    }

    temp = (NMLmsg *) cms->data;
    /* Check to see if remote process writing too much into local buffer. */
    if (_req->size > cms_encoded_data_explosion_factor * cms->size) {
	rcs_print_error
	    ("CMSserver:cms_writer: CMS buffer size is too small.\n");
	return ((REMOTE_WRITE_REPLY *) NULL);
    }

    /* Copy the encoded data to the location set up in CMS. */
    // memcpy(cms->encoded_data, _req->data, _req->size);
    cms->header.in_buffer_size = _req->size;
    temp->size = _req->size;
    int *serial_number = cms->serial
	? &(((RCS_CMD_MSG*)temp)->serial_number)
	: NULL;

    switch (_req->access_type) {
    case CMS_WRITE_ACCESS:
	nml->write(*temp, serial_number);
	break;
    case CMS_WRITE_IF_READ_ACCESS:
	nml->write_if_read(*temp, serial_number);
	break;
    default:
	rcs_print_error("NML_SERVER: Invalid Access type. (%d)\n",
	    _req->access_type);
	break;
    }

    write_reply.status = (int) cms->status;
    write_reply.write_id = cms->header.write_id;
    write_reply.was_read = cms->header.was_read;
    write_reply.confirm_write = cms->confirm_write;

    return (&write_reply);
}

REMOTE_SET_DIAG_INFO_REPLY *NML_SERVER_LOCAL_PORT::
set_diag_info(REMOTE_SET_DIAG_INFO_REQUEST * _req)
{
    if (NULL == _req) {
	return (NULL);
    }
    CMS_DIAG_PROC_INFO *dpi = cms->get_diag_proc_info();
    if (NULL == dpi) {
	return (NULL);
    }
    if (orig_info == NULL) {
	orig_info = new CMS_DIAG_PROC_INFO();
	*orig_info = *dpi;
    }
    rtapi_strlcpy(dpi->name, _req->process_name, 16);
    rtapi_strlcpy(dpi->host_sysinfo, _req->host_sysinfo, 32);
    if (cms->total_connections > _req->c_num && _req->c_num >= 0) {
	cms->connection_number = _req->c_num;
    }
    if (NULL != cms->handle_to_global_data) {
	cms->handle_to_global_data->total_bytes_moved = _req->bytes_moved;
    }
    dpi->pid = _req->pid;
    dpi->rcslib_ver = _req->rcslib_ver;
    cms->set_diag_proc_info(dpi);
    return (NULL);
}

REMOTE_GET_DIAG_INFO_REPLY *NML_SERVER_LOCAL_PORT::
get_diag_info(REMOTE_GET_DIAG_INFO_REQUEST * /*_req*/)
{
    get_diag_info_reply.cdi = cms->get_diagnostics_info();
    get_diag_info_reply.status = cms->status;
    return (&get_diag_info_reply);
}

REMOTE_GET_MSG_COUNT_REPLY *NML_SERVER_LOCAL_PORT::
get_msg_count(REMOTE_GET_DIAG_INFO_REQUEST * /*_req*/)
{
    return (NULL);
}

void NML_SERVER_LOCAL_PORT::reset_diag_info()
{
    if (NULL != orig_info) {
	CMS_DIAG_PROC_INFO *dpi = cms->get_diag_proc_info();
	*dpi = *orig_info;
	cms->set_diag_proc_info(dpi);
    }
}

NML_SUPER_SERVER *NML_Default_Super_Server = (NML_SUPER_SERVER *) NULL;

NML_SUPER_SERVER::NML_SUPER_SERVER()
{
    servers = (LinkedList *) NULL;
    unspawned_servers = 0;

    servers = new LinkedList;
}

NML_SUPER_SERVER::~NML_SUPER_SERVER()
{
    kill_all_servers();
    delete_all_servers();
    if (NULL != servers) {
	delete servers;
	servers = (LinkedList *) NULL;
    }
}

void NML_SUPER_SERVER::add_to_list(NML * _nml)
{
    NML_SERVER *server = (NML_SERVER *) NULL;
    NML_SERVER_LOCAL_PORT *local_port = (NML_SERVER_LOCAL_PORT *) NULL;
    NML *new_nml = (NML *) NULL;

    if (NULL != servers) {
	server = (NML_SERVER *) servers->get_head();
	while (NULL != server) {
	    if (server->accept_local_port_cms(_nml->cms)) {
		break;
	    }
	    server = (NML_SERVER *) servers->get_next();
	}
	if (NULL == server) {
	    server = new NML_SERVER(_nml);
	    if (NULL == server) {
		rcs_print_error
		    ("NML_SERVER: Unable to create server object.\n");
	    }
	} else {
	    if (_nml->cms->isserver) {
		new_nml = _nml;
		local_port = new NML_SERVER_LOCAL_PORT(new_nml);
                if (NULL == local_port) {
                    rcs_print_error("NML_SERVER: Unable to create local port.\n");
                    return;
                }
		local_port->local_channel_reused = 1;
	    } else {
		new_nml = new NML(_nml, 1, -1);
		local_port = new NML_SERVER_LOCAL_PORT(new_nml);
                if (NULL == local_port) {
                    rcs_print_error("NML_SERVER: Unable to create local port.\n");
                    return;
                }
		local_port->local_channel_reused = 0;
	    }
	    server->add_local_port(local_port);
	}
    }
}

void NML_SUPER_SERVER::add_to_list(NML_SERVER * _server)
{
    if ((NULL != servers) && (NULL != _server)) {
	_server->super_server_list_id
	    = servers->store_at_tail(_server, sizeof(NML_SERVER), 0);
	unspawned_servers++;
    }
}

void NML_SUPER_SERVER::spawn_all_servers()
{
    NML_SERVER *server;

    if (NULL != servers) {
	server = (NML_SERVER *) servers->get_head();
	while (NULL != server) {
	    if (server->spawn() > 0 && unspawned_servers > 0) {
		unspawned_servers--;
	    }
	    server = (NML_SERVER *) servers->get_next();
	}
    }
}

void NML_SUPER_SERVER::kill_all_servers()
{
    NML_SERVER *server;

    if (NULL != servers) {
	server = (NML_SERVER *) servers->get_head();
	while (NULL != server) {
	    if (server->server_spawned) {
		server->kill_server();
	    }
	    server = (NML_SERVER *) servers->get_next();
	}
    }
}

void NML_SUPER_SERVER::delete_all_servers()
{
    NML_SERVER *server;
    if (NULL != servers) {
	server = (NML_SERVER *) servers->get_head();
	while (NULL != server) {
	    if (!server->server_spawned && unspawned_servers > 0) {
		unspawned_servers--;
	    }
	    delete server;
	    server = (NML_SERVER *) servers->get_next();
	}
    }
}

int nml_control_C_caught = 0;
int nml_sigint_count = 0;
int dont_kill_servers = 0;
int dont_cleanup_servers = 0;
static int nmlsrv_last_sig = 0;

static void catch_control_C1(int sig)
{
    nmlsrv_last_sig = sig;
    nml_sigint_count++;
    signal(SIGINT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);
    if (NULL != NML_Default_Super_Server) {
	delete NML_Default_Super_Server;
	NML_Default_Super_Server = (NML_SUPER_SERVER *) NULL;
    }
    dont_kill_servers = 1;
    dont_cleanup_servers = 1;
    nml_cleanup();
    dont_kill_servers = 0;
    dont_cleanup_servers = 0;
    exit(0);
}				/* */

static void catch_control_C2(int sig)
{
    nmlsrv_last_sig = sig;
    signal(SIGINT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);
    nml_control_C_caught = 1;
}

void run_nml_server_exit(int i)
{
    rcs_exit(i);
}

void run_nml_servers()
{
    if (NULL != NML_Default_Super_Server) {
	if (NML_Default_Super_Server->servers != NULL) {
	    if (NML_Default_Super_Server->servers->list_size <
		NML_Default_Super_Server->unspawned_servers) {
		NML_Default_Super_Server->unspawned_servers =
		    NML_Default_Super_Server->servers->list_size;
	    }
	    if (NML_Default_Super_Server->unspawned_servers <= 0) {
		rcs_print_error
		    ("run_nml_servers(): No buffers without servers already spawned for them.\n");
		return;
	    }
	    if (NML_Default_Super_Server->unspawned_servers == 1) {
		NML_Default_Super_Server->unspawned_servers = 0;
		NML_SERVER *sole_server;
		sole_server =
		    (NML_SERVER *) NML_Default_Super_Server->servers->
		    get_head();
		while (sole_server != NULL) {
		    if (NULL != sole_server->remote_port) {
			if (!sole_server->remote_port->running &&
			    !sole_server->server_spawned) {
			    break;
			}
		    }
		    sole_server =
			(NML_SERVER *) NML_Default_Super_Server->servers->
			get_next();
		}
		if (NULL == sole_server) {
		    rcs_print_error
			("run_nml_servers() : sole_server is NULL.\n");
		    run_nml_server_exit(-1);
		} else {
		    signal(SIGINT, catch_control_C1);
		    signal(SIGTERM, catch_control_C1);
		    sole_server->run(0);
		    run_nml_server_exit(-1);
		}
	    } else {
		nml_control_C_caught = 0;
		NML_Default_Super_Server->spawn_all_servers();
		signal(SIGINT, catch_control_C2);
		signal(SIGTERM, catch_control_C2);
		while (!nml_control_C_caught)
		    esleep(1.0);
		NML_Default_Super_Server->kill_all_servers();
		nml_cleanup();
		run_nml_server_exit(0);
	    }
	} else {
	    rcs_print_error
		("run_nml_servers(): No buffers without servers already spawned for them.\n");
	}
    } else {
	rcs_print_error
	    ("run_nml_servers(): No buffers without servers already spawned for them.\n");
    }
    run_nml_server_exit(-1);
}

void spawn_nml_servers()
{
    if (NULL != NML_Default_Super_Server) {
	NML_Default_Super_Server->spawn_all_servers();
    }
}

void kill_nml_servers()
{
    if (!dont_kill_servers) {
	if (NULL != NML_Default_Super_Server) {
	    NML_Default_Super_Server->kill_all_servers();
	}
    }
}

void nml_server_cleanup()
{
    if (!dont_cleanup_servers) {
	if (NULL != NML_Default_Super_Server) {
	    NML_Default_Super_Server->kill_all_servers();
	    NML_Default_Super_Server->delete_all_servers();
	    delete NML_Default_Super_Server;
	    NML_Default_Super_Server = (NML_SUPER_SERVER *) NULL;
	}
    }
}
