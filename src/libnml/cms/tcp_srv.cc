/********************************************************************
* Description: tcp_srv.cc
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
/****************************************************************************
* File: tcp_srv.cc
* Purpose: Provides the functions for the class CMS_SERVER_REMOTE_TCP_PORT
*  which provides TCP specific overrides of the CMS_SERVER_REMOTE_PORT class.
****************************************************************************/

#if defined(__GNUC__) && ((__GNUC__ > 4) || ((__GNUC__ == 4) && (__GNUC_MINOR__ >= 4)))
#pragma GCC optimize "-fno-strict-aliasing"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>		// memset(), strerror()
#include <stdlib.h>		// malloc(), free()
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <errno.h>		/* errno */
#include <signal.h>		// SIGPIPE, signal()

#ifdef __cplusplus
}
#endif

#include <rtapi_string.h>	// rtapi_strlcpy

#include <sys/types.h>
#include <sys/wait.h>		// waitpid

#include <arpa/inet.h>		/* inet_ntoa */
#include "cms.hh"		/* class CMS */
#include "nml.hh"		// class NML
#include "tcp_srv.hh"		/* class CMS_SERVER_REMOTE_TCP_PORT */
#include "rcs_print.hh"		/* rcs_print_error() */
#include "linklist.hh"		/* class LinkedList */
#include "tcp_opts.hh"		/* SET_TCP_NODELAY */
#include "timer.hh"		// esleep()
#include "_timer.h"
#include "cmsdiag.hh"		// class CMS_DIAGNOSTICS_INFO
extern "C" {
#include "recvn.h"		/* recvn() */
#include "sendn.h"		/* sendn() */
}
#include "physmem.hh"           // PHYSMEM_HANDLE

int tcpsvr_threads_created = 0;
int tcpsvr_threads_killed = 0;
int tcpsvr_threads_exited = 0;
int tcpsvr_threads_returned_early = 0;

TCPSVR_BLOCKING_READ_REQUEST::TCPSVR_BLOCKING_READ_REQUEST()
{
    access_type = CMS_READ_ACCESS;	/* read or just peek */
    last_id_read = 0;		/* The server can compare with id from buffer 
				 */
    /* to determine if the buffer is new */
    /* to this client */
    timeout_millis = -1;	/* Milliseconds for blocking_timeout or -1 to 
				   wait forever */
    _client_tcp_port = NULL;
    remport = NULL;
    server = NULL;
    _nml = NULL;
    _reply = NULL;
    _data = NULL;
    read_reply = NULL;
}

static inline double tcp_svr_reverse_double(double in)
{
    double out;
    char *c1, *c2;

    c1 = ((char *) &in) + 7;
    c2 = (char *) &out;
    for (int i = 0; i < 8; i++) {
	*c2 = *c1;
	c1--;
	c2++;
    }
    return out;
}

TCPSVR_BLOCKING_READ_REQUEST::~TCPSVR_BLOCKING_READ_REQUEST()
{
    if (NULL != _nml) {
	NML *nmlcopy = (NML *) _nml;
	_nml = NULL;
	delete nmlcopy;
    }
    if (NULL != _data) {
	void *_datacopy = _data;
	if (NULL != read_reply) {
	    if (_data == read_reply->data) {
		read_reply->data = NULL;
	    }
	}
	_data = NULL;
	free(_datacopy);
    }
    if (NULL != _reply) {
	free(_reply);
	_reply = NULL;
	read_reply = NULL;
    }
    if (NULL != read_reply) {
	if (NULL != read_reply->data) {
	    free(read_reply->data);
	    read_reply->data = NULL;
	}
	delete read_reply;
	read_reply = NULL;
    }
}

CMS_SERVER_REMOTE_TCP_PORT::CMS_SERVER_REMOTE_TCP_PORT(CMS_SERVER * _cms_server)
  : CMS_SERVER_REMOTE_PORT(_cms_server),
    dtimeout(20.0),
    read_fd_set{},
    write_fd_set{},
    maxfdpl(0),
    subscription_buffers(NULL),
    connection_socket(0),
    connection_port(0),
    request(NULL),
    temp_buffer{},
    current_poll_interval_millis(30000),
    polling_enabled(0),
    select_timeout{.tv_sec = 30, .tv_usec = 30}
{
    memset(&server_socket_address, 0, sizeof(server_socket_address));
    server_socket_address.sin_family = AF_INET;
    server_socket_address.sin_addr.s_addr = htonl(INADDR_ANY);
    //server_socket_address.sin_port = 0;

    client_ports = new LinkedList;
    if (NULL == client_ports) {
	rcs_print_error("Can not create linked list for client ports.\n");
	return;
    }
}

CMS_SERVER_REMOTE_TCP_PORT::~CMS_SERVER_REMOTE_TCP_PORT()
{
    if (client_ports == NULL) return;
    unregister_port();
    if (NULL != client_ports) {
	delete client_ports;
	client_ports = (LinkedList *) NULL;
    }
}

void blocking_thread_kill(long int id)
{

    if (id <= 0) {
	return;
    }
#ifdef POSIX_THREADS
    pthread_kill(id, SIGINT);
    pthread_join(id, NULL);
#endif
#ifdef NO_THREADS
    kill(id, SIGINT);
    waitpid(id, NULL, 0);
#endif
    tcpsvr_threads_killed++;
}

void CMS_SERVER_REMOTE_TCP_PORT::unregister_port()
{
    CLIENT_TCP_PORT *client;
    int number_of_connected_clients = 0;

    client = (CLIENT_TCP_PORT *) client_ports->get_head();
    while (NULL != client) {
	rcs_print("Exiting even though client on %s is still connected.\n",
	    inet_ntoa(client->address.sin_addr));
	client = (CLIENT_TCP_PORT *) client_ports->get_next();
	number_of_connected_clients++;
    }
    client = (CLIENT_TCP_PORT *) client_ports->get_head();
    while (NULL != client) {
	delete client;
	client_ports->delete_current_node();
	client = (CLIENT_TCP_PORT *) client_ports->get_next();
    }
    if (NULL != subscription_buffers) {
	TCP_BUFFER_SUBSCRIPTION_INFO *sub_info =
	    (TCP_BUFFER_SUBSCRIPTION_INFO *) subscription_buffers->get_head();
	while (NULL != sub_info) {
	    delete sub_info;
	    sub_info = (TCP_BUFFER_SUBSCRIPTION_INFO *)
		subscription_buffers->get_next();
	}
	delete subscription_buffers;
	subscription_buffers = NULL;
    }
    if (number_of_connected_clients > 0) {
	esleep(2.0);
    }
    if (connection_socket > 0) {
	close(connection_socket);
	connection_socket = 0;
    }
}

int CMS_SERVER_REMOTE_TCP_PORT::accept_local_port_cms(CMS * _cms)
{
    if (NULL == _cms) {
	return 0;
    }
    if (_cms->remote_port_type != CMS_TCP_REMOTE_PORT_TYPE) {
	return 0;
    }
    if (NULL != _cms) {
	if (min_compatible_version < 1e-6 ||
	    (min_compatible_version > _cms->min_compatible_version &&
		_cms->min_compatible_version > 1e-6)) {
	    min_compatible_version = _cms->min_compatible_version;
	}
	if (_cms->confirm_write) {
	    confirm_write = _cms->confirm_write;
	}
    }
    if (_cms->total_subdivisions > max_total_subdivisions) {
	max_total_subdivisions = _cms->total_subdivisions;
    }
    if (server_socket_address.sin_port == 0) {
	server_socket_address.sin_port =
	    htons(((u_short) _cms->tcp_port_number));
	port_num = _cms->tcp_port_number;
	return 1;
    }
    if (server_socket_address.sin_port ==
	htons(((u_short) _cms->tcp_port_number))) {
	port_num = _cms->tcp_port_number;
	return 1;
    }
    return 0;
}

void CMS_SERVER_REMOTE_TCP_PORT::register_port()
{
    port_registered = 0;
    rcs_print_debug(PRINT_CMS_CONFIG_INFO,
	"Registering server on TCP port %d.\n",
	ntohs(server_socket_address.sin_port));
    if (server_socket_address.sin_port == 0) {
	rcs_print_error("server can not register on port number 0.\n");
	return;
    }
    if ((connection_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
	rcs_print_error("socket error: %d -- %s\n", errno, strerror(errno));
	rcs_print_error("Server can not open stream socket.\n");
	return;
    }

    if (set_tcp_socket_options(connection_socket) < 0) {
	return;
    }
    if (bind(connection_socket, (struct sockaddr *) &server_socket_address,
	    sizeof(server_socket_address)) < 0) {
	rcs_print_error("bind error: %d -- %s\n", errno, strerror(errno));
	rcs_print_error
	    ("Server can not bind the connection socket on port %d.\n",
	    ntohs(server_socket_address.sin_port));
	return;
    }
    if (listen(connection_socket, 5) < 0) {
	rcs_print_error("listen error: %d -- %s\n", errno, strerror(errno));
	rcs_print_error("TCP Server: error on call to listen for port %d.\n",
	    ntohs(server_socket_address.sin_port));
	return;
    }
    port_registered = 1;

}

static int last_pipe_signum = 0;

static void handle_pipe_error(int signum)
{
    last_pipe_signum = signum;
    rcs_print_error("SIGPIPE intercepted.\n");
}

void CMS_SERVER_REMOTE_TCP_PORT::run()
{
    int bytes_ready;
    int ready_descriptors;
    if (NULL == client_ports) {
	rcs_print_error("CMS_SERVER: List of client ports is NULL.\n");
	return;
    }
    CLIENT_TCP_PORT *new_client_port, *client_port_to_check;
    FD_ZERO(&read_fd_set);
    FD_ZERO(&write_fd_set);
    FD_SET(connection_socket, &read_fd_set);
    maxfdpl = connection_socket + 1;
    signal(SIGPIPE, handle_pipe_error);
    rcs_print_debug(PRINT_CMS_CONFIG_INFO,
	"running server for TCP port %d (connection_socket = %d).\n",
	ntohs(server_socket_address.sin_port), connection_socket);

    cms_server_count++;
    fd_set read_fd_set_copy, write_fd_set_copy;
    FD_ZERO(&read_fd_set_copy);
    FD_ZERO(&write_fd_set_copy);
    FD_SET(connection_socket, &read_fd_set_copy);

    while (1) {
	if (polling_enabled) {
	    memcpy(&read_fd_set_copy, &read_fd_set, sizeof(fd_set));
	    memcpy(&write_fd_set_copy, &write_fd_set, sizeof(fd_set));
	    select_timeout.tv_sec = current_poll_interval_millis / 1000;
	    select_timeout.tv_usec =
		(current_poll_interval_millis % 1000) * 1000;
	    ready_descriptors =
		select(maxfdpl, &read_fd_set, &write_fd_set,
		(fd_set *) NULL, (timeval *) & select_timeout);
	    if (ready_descriptors == 0) {
		update_subscriptions();
		memcpy(&read_fd_set, &read_fd_set_copy, sizeof(fd_set));
		memcpy(&write_fd_set, &write_fd_set_copy, sizeof(fd_set));
		continue;
	    }
	} else {
	    ready_descriptors =
		select(maxfdpl, &read_fd_set, &write_fd_set,
		(fd_set *) NULL, (timeval *) NULL);

	}
	if (ready_descriptors < 0) {
	    rcs_print_error("server: select error.(errno = %d | %s)\n",
		errno, strerror(errno));
	}
	if (NULL == client_ports) {
	    rcs_print_error("CMS_SERVER: List of client ports is NULL.\n");
	    return;
	}
	client_port_to_check = (CLIENT_TCP_PORT *) client_ports->get_head();
	while (NULL != client_port_to_check) {
	    if (FD_ISSET(client_port_to_check->socket_fd, &read_fd_set)) {
		ioctl(client_port_to_check->socket_fd, FIONREAD,
		    (caddr_t) & bytes_ready);
		if (bytes_ready <= 0) {
		    rcs_print_debug(PRINT_SOCKET_CONNECT,
			"Socket closed by host with IP address %s.\n",
			inet_ntoa(client_port_to_check->address.sin_addr));
		    if (NULL != client_port_to_check->subscriptions) {
			TCP_CLIENT_SUBSCRIPTION_INFO *clnt_sub_info =
			    (TCP_CLIENT_SUBSCRIPTION_INFO *)
			    client_port_to_check->subscriptions->get_head();
			while (NULL != clnt_sub_info) {
			    if (NULL != clnt_sub_info->sub_buf_info &&
				clnt_sub_info->subscription_list_id >= 0) {
				if (NULL !=
				    clnt_sub_info->sub_buf_info->
				    sub_clnt_info) {
				    clnt_sub_info->sub_buf_info->
					sub_clnt_info->
					delete_node(clnt_sub_info->
					subscription_list_id);
				    if (clnt_sub_info->sub_buf_info->
					sub_clnt_info->list_size < 1) {
					delete clnt_sub_info->sub_buf_info->
					    sub_clnt_info;
					clnt_sub_info->sub_buf_info->
					    sub_clnt_info = NULL;
					if (NULL != subscription_buffers
					    && clnt_sub_info->sub_buf_info->
					    list_id >= 0) {
					    subscription_buffers->
						delete_node(clnt_sub_info->
						sub_buf_info->list_id);
					    delete clnt_sub_info->
						sub_buf_info;
					    clnt_sub_info->sub_buf_info =
						NULL;
					}
				    }
				    clnt_sub_info->sub_buf_info = NULL;
				}
				delete clnt_sub_info;
				clnt_sub_info =
				    (TCP_CLIENT_SUBSCRIPTION_INFO *)
				    client_port_to_check->subscriptions->
				    get_next();
			    }
			    delete client_port_to_check->subscriptions;
			    client_port_to_check->subscriptions = NULL;
			    recalculate_polling_interval();
			}
		    }
		    if (client_port_to_check->threadId > 0
			&& client_port_to_check->blocking) {
			blocking_thread_kill(client_port_to_check->threadId);
		    }
		    close(client_port_to_check->socket_fd);
		    FD_CLR(client_port_to_check->socket_fd, &read_fd_set);
		    client_port_to_check->socket_fd = -1;
		    delete client_port_to_check;
		    client_ports->delete_current_node();
		} else {
		    if (client_port_to_check->blocking) {
			if (client_port_to_check->threadId > 0) {
			    rcs_print_debug(PRINT_SERVER_THREAD_ACTIVITY,
				"Data received from %s:%d when it should be blocking (bytes_ready=%d).\n",
				inet_ntoa
				(client_port_to_check->address.
				    sin_addr),
				client_port_to_check->socket_fd, bytes_ready);
			    rcs_print_debug(PRINT_SERVER_THREAD_ACTIVITY,
				"Killing handler %d.\n",
				client_port_to_check->threadId);

			    blocking_thread_kill
				(client_port_to_check->threadId);
#if 0
			    *((uint32_t *) temp_buffer) =
				htonl(client_port_to_check->serial_number);
			    *((uint32_t *) temp_buffer + 1) =
				htonl((unsigned long)
				CMS_SERVER_SIDE_ERROR);
			    putbe32(temp_buffer + 8, 0);	/* size
									 */
			    putbe32(temp_buffer + 12, 0);	/* write_id
									 */
			    putbe32(temp_buffer + 16, 0);	/* was_read
									 */
			    sendn(client_port_to_check->socket_fd,
				temp_buffer, 20, 0, dtimeout);
#endif
			    client_port_to_check->threadId = 0;
			    client_port_to_check->blocking = 0;
			}
		    }
		    handle_request(client_port_to_check);
		}
		ready_descriptors--;
	    } else {
		FD_SET(client_port_to_check->socket_fd, &read_fd_set);
	    }
	    client_port_to_check =
		(CLIENT_TCP_PORT *) client_ports->get_next();
	}
	if (FD_ISSET(connection_socket, &read_fd_set)
	    && ready_descriptors > 0) {
	    ready_descriptors--;
	    socklen_t client_address_length;
	    new_client_port = new CLIENT_TCP_PORT();
	    client_address_length = sizeof(new_client_port->address);
	    new_client_port->socket_fd = accept(connection_socket,
		(struct sockaddr *)
		&new_client_port->address, &client_address_length);
	    current_clients++;
	    if (current_clients > max_clients) {
		max_clients = current_clients;
	    }
	    if (new_client_port->socket_fd < 0) {
		rcs_print_error("server: accept error -- %d %s \n", errno,
		    strerror(errno));
		continue;
	    }
	    rcs_print_debug(PRINT_SOCKET_CONNECT,
		"Socket opened by host with IP address %s.\n",
		inet_ntoa(new_client_port->address.sin_addr));
	    new_client_port->serial_number = 0;
	    new_client_port->blocking = 0;
	    if (NULL != client_ports) {
		client_ports->store_at_tail(new_client_port,
		    sizeof(new_client_port), 0);
	    }
	    if (maxfdpl < new_client_port->socket_fd + 1) {
		maxfdpl = new_client_port->socket_fd + 1;
	    }
	    FD_SET(new_client_port->socket_fd, &read_fd_set);
	} else {
	    FD_SET(connection_socket, &read_fd_set);
	}
	if (0 != ready_descriptors) {
	    rcs_print_error("%d descriptors ready but not serviced.\n",
		ready_descriptors);
	}
	update_subscriptions();
    }
}

static int tcpsvr_handle_blocking_request_sigint_count = 0;
static int tcpsvr_last_sig = 0;

void tcpsvr_handle_blocking_request_sigint_handler(int sig)
{
    tcpsvr_last_sig = sig;
    tcpsvr_handle_blocking_request_sigint_count++;
}

static void putbe32(char *addr, uint32_t val) {
    val = htonl(val);
    memcpy(addr, &val, sizeof(val));
}

static uint32_t getbe32(char *addr) {
    uint32_t val;
    memcpy(&val, addr, sizeof(val));
    return ntohl(val);
}

#if defined(POSIX_THREADS) || defined(NO_THREADS)
void *tcpsvr_handle_blocking_request(void *_req)
{
    signal(SIGINT, tcpsvr_handle_blocking_request_sigint_handler);
    TCPSVR_BLOCKING_READ_REQUEST *blocking_read_req =
	(TCPSVR_BLOCKING_READ_REQUEST *) _req;
    char temp_buffer[0x2000];
    if (_req == NULL) {
	tcpsvr_threads_returned_early++;
	return 0;
    }
    double dtimeout =
	((double) (blocking_read_req->timeout_millis + 10)) / 1000.0;
    if (dtimeout < 0) {
	dtimeout = 600.0;
    }
    if (dtimeout < 0.5) {
	dtimeout = 0.5;
    }
    if (dtimeout > 600.0) {
	dtimeout = 600.0;
    }
    CLIENT_TCP_PORT *_client_tcp_port = blocking_read_req->_client_tcp_port;
    CMS_SERVER *server = blocking_read_req->server;

    if (NULL == server || NULL == _client_tcp_port) {
	tcpsvr_threads_returned_early++;
	return 0;
    }
    memset(temp_buffer, 0, 0x2000);
    REMOTE_BLOCKING_READ_REPLY *read_reply;

    if (NULL != _client_tcp_port->diag_info) {
	_client_tcp_port->diag_info->buffer_number =
	    blocking_read_req->buffer_number;
	server->set_diag_info(_client_tcp_port->diag_info);
    } else if (server->diag_enabled) {
	server->reset_diag_info(blocking_read_req->buffer_number);
    }

    read_reply = (REMOTE_BLOCKING_READ_REPLY *)
	server->process_request(blocking_read_req);
    blocking_read_req->read_reply = read_reply;
    if (NULL == read_reply) {
	_client_tcp_port->blocking = 0;
	rcs_print_error("Server could not process request.\n");
	putbe32(temp_buffer, _client_tcp_port->serial_number);
	putbe32(temp_buffer + 4, CMS_SERVER_SIDE_ERROR);
	putbe32(temp_buffer + 8, 0);	/* size */
	putbe32(temp_buffer + 12, 0)	/* write_id */;
	putbe32(temp_buffer + 16, 0)	/* was_read */;
	sendn(_client_tcp_port->socket_fd, temp_buffer, 20, 0, dtimeout);
	_client_tcp_port->errors++;
	_client_tcp_port->blocking_read_req = NULL;
	delete blocking_read_req;
	_client_tcp_port->threadId = 0;
	tcpsvr_threads_returned_early++;
	return 0;
    }
    putbe32(temp_buffer, _client_tcp_port->serial_number);
    putbe32(temp_buffer + 4, read_reply->status);
    putbe32(temp_buffer + 8, read_reply->size);
    putbe32(temp_buffer + 12, read_reply->write_id);
    putbe32(temp_buffer + 16, read_reply->was_read);
    if (read_reply->size < (0x2000 - 20) && read_reply->size > 0) {
	memcpy(temp_buffer + 20, read_reply->data, read_reply->size);
	_client_tcp_port->blocking = 0;
	if (sendn
	    (_client_tcp_port->socket_fd, temp_buffer, 20 + read_reply->size,
		0, dtimeout) < 0) {
	    _client_tcp_port->blocking = 0;
	    _client_tcp_port->errors++;
	    _client_tcp_port->blocking_read_req = NULL;
	    delete blocking_read_req;
	    _client_tcp_port->threadId = 0;
	    tcpsvr_threads_returned_early++;
	    return 0;
	}
    } else {
	_client_tcp_port->blocking = 0;
	if (sendn(_client_tcp_port->socket_fd, temp_buffer, 20, 0, dtimeout) <
	    0) {
	    _client_tcp_port->blocking = 0;
	    _client_tcp_port->errors++;
	    _client_tcp_port->blocking_read_req = NULL;
	    delete blocking_read_req;
	    _client_tcp_port->threadId = 0;
	    tcpsvr_threads_returned_early++;
	    return 0;
	}
	if (read_reply->size > 0) {
	    if (sendn
		(_client_tcp_port->socket_fd, read_reply->data,
		    read_reply->size, 0, dtimeout) < 0) {
		_client_tcp_port->blocking = 0;
		_client_tcp_port->errors++;
		_client_tcp_port->blocking_read_req = NULL;
		delete blocking_read_req;
		_client_tcp_port->threadId = 0;
		tcpsvr_threads_returned_early++;
		return 0;
	    }
	}
    }
    _client_tcp_port->blocking_read_req = NULL;
    delete blocking_read_req;
    _client_tcp_port->threadId = 0;
    tcpsvr_threads_exited++;
#ifdef POSIX_THREADS
    pthread_exit(0);
#endif
#ifdef NO_THREADS
    exit(0);
#endif
}

#endif

void CMS_SERVER_REMOTE_TCP_PORT::handle_request(CLIENT_TCP_PORT *
    _client_tcp_port)
{
    CLIENT_TCP_PORT *client_port_to_check = NULL;
    pid_t pid = getpid();
    pid_t tid = 0;
    CMS_SERVER *server;
    server = find_server(pid, tid);
    if (NULL == server) {
	rcs_print_error
	    ("CMS_SERVER_REMOTE_TCP_PORT::handle_request() Cannot find server object for pid = %d.\n",
	    pid);
	return;
    }

    if (server->using_passwd_file) {
	current_user_info = get_connected_user(_client_tcp_port->socket_fd);
    }

    if (_client_tcp_port->errors >= _client_tcp_port->max_errors) {
	rcs_print_error("Too many errors - closing connection(%d)\n",
	    _client_tcp_port->socket_fd);
	client_port_to_check = (CLIENT_TCP_PORT *) client_ports->get_head();
	while (NULL != client_port_to_check) {
	    if (client_port_to_check->socket_fd ==
		_client_tcp_port->socket_fd) {
		delete client_port_to_check;
		client_ports->delete_current_node();
	    }
	    client_port_to_check =
		(CLIENT_TCP_PORT *) client_ports->get_next();
	}
	close(_client_tcp_port->socket_fd);
	current_clients--;
	FD_CLR(_client_tcp_port->socket_fd, &read_fd_set);
	_client_tcp_port->socket_fd = -1;
    }

    if (recvn(_client_tcp_port->socket_fd, temp_buffer, 20, 0, -1, NULL) < 0) {
	rcs_print_error("Can not read from client port (%d) from %s\n",
	    _client_tcp_port->socket_fd,
	    inet_ntoa(_client_tcp_port->address.sin_addr));
	_client_tcp_port->errors++;
	return;
    }
    long request_type, buffer_number, received_serial_number;
    received_serial_number = getbe32(temp_buffer);
    if (received_serial_number != _client_tcp_port->serial_number) {
	rcs_print_error
	    ("received_serial_number (%ld) does not equal expected serial number.(%ld)\n",
	    received_serial_number, _client_tcp_port->serial_number);
	_client_tcp_port->serial_number = received_serial_number;
	_client_tcp_port->errors++;
    }
    _client_tcp_port->serial_number++;
    request_type = ntohl(*((uint32_t *) temp_buffer + 1));
    buffer_number = ntohl(*((uint32_t *) temp_buffer + 2));

    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPSVR request received: fd = %d, serial_number=%ld, request_type=%ld, buffer_number=%ld\n",
	_client_tcp_port->socket_fd,
	_client_tcp_port->serial_number, request_type, buffer_number);

    if (NULL != _client_tcp_port->diag_info) {
	_client_tcp_port->diag_info->buffer_number = buffer_number;
	server->set_diag_info(_client_tcp_port->diag_info);
    } else if (server->diag_enabled) {
	server->reset_diag_info(buffer_number);
    }

    switch_function(_client_tcp_port,
	server, request_type, buffer_number, received_serial_number);

    if (NULL != _client_tcp_port->diag_info &&
	NULL != server->last_local_port_used && server->diag_enabled) {
	if (NULL != server->last_local_port_used->cms) {
	    if (NULL !=
		server->last_local_port_used->cms->handle_to_global_data) {
		_client_tcp_port->diag_info->bytes_moved =
		    server->last_local_port_used->cms->handle_to_global_data->
		    total_bytes_moved;
	    }
	}
    }
}

void CMS_SERVER_REMOTE_TCP_PORT::switch_function(CLIENT_TCP_PORT *
    _client_tcp_port,
    CMS_SERVER * server,
    long request_type, long buffer_number, long /*received_serial_number*/)
{
    int total_subdivisions = 1;
    CLIENT_TCP_PORT *client_port_to_check = NULL;
    switch (request_type) {
    case REMOTE_CMS_SET_DIAG_INFO_REQUEST_TYPE:
	{
	    if (NULL == _client_tcp_port->diag_info) {
		_client_tcp_port->diag_info =
		    new REMOTE_SET_DIAG_INFO_REQUEST();
	    }
	    if (recvn
		(_client_tcp_port->socket_fd, server->set_diag_info_buf, 68,
		    0, -1, NULL) < 0) {
		rcs_print_error
		    ("Can not read from client port (%d) from %s\n",
		    _client_tcp_port->socket_fd,
		    inet_ntoa(_client_tcp_port->address.sin_addr));
		_client_tcp_port->errors++;
		return;
	    }
	    _client_tcp_port->diag_info->bytes_moved = 0.0;
	    _client_tcp_port->diag_info->buffer_number = buffer_number;
	    memcpy(_client_tcp_port->diag_info->process_name,
		server->set_diag_info_buf, 16);
	    memcpy(_client_tcp_port->diag_info->host_sysinfo,
		server->set_diag_info_buf + 16, 32);
	    _client_tcp_port->diag_info->pid =
		htonl(*((uint32_t *) (server->set_diag_info_buf + 48)));
	    _client_tcp_port->diag_info->c_num =
		htonl(*((uint32_t *) (server->set_diag_info_buf + 52)));
	    memcpy(&(_client_tcp_port->diag_info->rcslib_ver),
		server->set_diag_info_buf + 56, 8);
	    _client_tcp_port->diag_info->reverse_flag =
		*((int *) ((char *) server->set_diag_info_buf + 64));
	    if (_client_tcp_port->diag_info->reverse_flag == 0x44332211) {
		_client_tcp_port->diag_info->rcslib_ver =
		    (double) tcp_svr_reverse_double((double)
		    _client_tcp_port->diag_info->rcslib_ver);
	    }
	}
	break;

    case REMOTE_CMS_GET_DIAG_INFO_REQUEST_TYPE:
	{
	    REMOTE_GET_DIAG_INFO_REQUEST diagreq;
	    diagreq.buffer_number = buffer_number;
	    REMOTE_GET_DIAG_INFO_REPLY *diagreply = NULL;
	    diagreply =
		(REMOTE_GET_DIAG_INFO_REPLY *) server->
		process_request(&diagreq);
	    if (NULL == diagreply) {
		putbe32(temp_buffer, _client_tcp_port->serial_number);
		putbe32(temp_buffer+4, CMS_SERVER_SIDE_ERROR);
		if (sendn
		    (_client_tcp_port->socket_fd, temp_buffer, 24, 0,
			dtimeout) < 0) {
		    _client_tcp_port->errors++;
		}
		return;
	    }
	    if (NULL == diagreply->cdi) {
		putbe32(temp_buffer, _client_tcp_port->serial_number);
		putbe32(temp_buffer + 4, CMS_SERVER_SIDE_ERROR);
		if (sendn
		    (_client_tcp_port->socket_fd, temp_buffer, 24, 0,
			dtimeout) < 0) {
		    _client_tcp_port->errors++;
		}
		return;
	    }
	    memset(temp_buffer, 0, 0x2000);
	    unsigned long dpi_offset = 32;
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    putbe32(temp_buffer + 4, diagreply->status);
	    putbe32(temp_buffer + 8, diagreply->cdi->last_writer);
	    putbe32(temp_buffer + 12, diagreply->cdi->last_reader);
	    double curtime = etime();
	    double reversed_temp = 0.0;
	    if (_client_tcp_port->diag_info->reverse_flag == 0x44332211) {
		reversed_temp =
		    (double) tcp_svr_reverse_double((double) curtime);
		memcpy(temp_buffer + 16, &reversed_temp, 8);
	    } else {
		memcpy(temp_buffer + 16, &(curtime), 8);
	    }
	    int dpi_count = 0;
	    if (NULL != diagreply->cdi->dpis) {
		CMS_DIAG_PROC_INFO *dpi =
		    (CMS_DIAG_PROC_INFO *) diagreply->cdi->dpis->get_head();
		while ((dpi_offset <
			((int) 0x2000 - sizeof(CMS_DIAG_PROC_INFO)))
		    && dpi != NULL) {
		    dpi_count++;
		    memcpy(temp_buffer + dpi_offset, dpi->name, 16);
		    dpi_offset += 16;
		    memcpy(temp_buffer + dpi_offset, dpi->host_sysinfo, 32);
		    dpi_offset += 32;
		    *((uint32_t *) ((char *) temp_buffer + dpi_offset)) =
			htonl(dpi->pid);
		    dpi_offset += 4;
		    if (_client_tcp_port->diag_info->reverse_flag ==
			0x44332211) {
			reversed_temp =
			    (double) tcp_svr_reverse_double((double)
			    dpi->rcslib_ver);
			memcpy(temp_buffer + dpi_offset, &reversed_temp, 8);
		    } else {
			memcpy(temp_buffer + dpi_offset, &(dpi->rcslib_ver),
			    8);
		    }
		    dpi_offset += 8;
		    *((uint32_t *) ((char *) temp_buffer + dpi_offset)) =
			htonl(dpi->access_type);
		    dpi_offset += 4;
		    *((uint32_t *) ((char *) temp_buffer + dpi_offset)) =
			htonl(dpi->msg_id);
		    dpi_offset += 4;
		    *((uint32_t *) ((char *) temp_buffer + dpi_offset)) =
			htonl(dpi->msg_size);
		    dpi_offset += 4;
		    *((uint32_t *) ((char *) temp_buffer + dpi_offset)) =
			htonl(dpi->msg_type);
		    dpi_offset += 4;
		    *((uint32_t *) ((char *) temp_buffer + dpi_offset)) =
			htonl(dpi->number_of_accesses);
		    dpi_offset += 4;
		    *((uint32_t *) ((char *) temp_buffer + dpi_offset)) =
			htonl(dpi->number_of_new_messages);
		    dpi_offset += 4;
		    if (_client_tcp_port->diag_info->reverse_flag ==
			0x44332211) {
			reversed_temp =
			    (double) tcp_svr_reverse_double((double)
			    dpi->bytes_moved);
			memcpy(temp_buffer + dpi_offset, &reversed_temp, 8);
		    } else {
			memcpy(temp_buffer + dpi_offset, &(dpi->bytes_moved),
			    8);
		    }
		    dpi_offset += 8;
		    if (_client_tcp_port->diag_info->reverse_flag ==
			0x44332211) {
			reversed_temp =
			    (double) tcp_svr_reverse_double((double)
			    dpi->bytes_moved_across_socket);
			memcpy(temp_buffer + dpi_offset, &reversed_temp, 8);
		    } else {
			memcpy(temp_buffer + dpi_offset,
			    &(dpi->bytes_moved_across_socket), 8);
		    }
		    dpi_offset += 8;
		    if (_client_tcp_port->diag_info->reverse_flag ==
			0x44332211) {
			reversed_temp =
			    (double) tcp_svr_reverse_double((double)
			    dpi->last_access_time);
			memcpy(temp_buffer + dpi_offset, &reversed_temp, 8);
		    } else {
			memcpy(temp_buffer + dpi_offset,
			    &(dpi->last_access_time), 8);
		    }
		    dpi_offset += 8;
		    if (_client_tcp_port->diag_info->reverse_flag ==
			0x44332211) {
			reversed_temp =
			    (double) tcp_svr_reverse_double((double)
			    dpi->first_access_time);
			memcpy(temp_buffer + dpi_offset, &reversed_temp, 8);
		    } else {
			memcpy(temp_buffer + dpi_offset,
			    &(dpi->first_access_time), 8);
		    }
		    dpi_offset += 8;
		    if (_client_tcp_port->diag_info->reverse_flag ==
			0x44332211) {
			reversed_temp =
			    (double) tcp_svr_reverse_double((double)
			    dpi->min_difference);
			memcpy(temp_buffer + dpi_offset, &reversed_temp, 8);
		    } else {
			memcpy(temp_buffer + dpi_offset,
			    &(dpi->min_difference), 8);
		    }
		    dpi_offset += 8;
		    if (_client_tcp_port->diag_info->reverse_flag ==
			0x44332211) {
			reversed_temp =
			    (double) tcp_svr_reverse_double((double)
			    dpi->max_difference);
			memcpy(temp_buffer + dpi_offset, &reversed_temp, 8);
		    } else {
			memcpy(temp_buffer + dpi_offset,
			    &(dpi->max_difference), 8);
		    }
		    dpi_offset += 8;
		    int is_last_writer =
			(dpi == diagreply->cdi->last_writer_dpi);
		    *((uint32_t *) ((char *) temp_buffer + dpi_offset)) =
			htonl(is_last_writer);
		    dpi_offset += 4;
		    int is_last_reader =
			(dpi == diagreply->cdi->last_reader_dpi);
		    *((uint32_t *) ((char *) temp_buffer + dpi_offset)) =
			htonl(is_last_reader);
		    dpi_offset += 4;
		    dpi =
			(CMS_DIAG_PROC_INFO *) diagreply->cdi->dpis->
			get_next();
		}
	    }
	    *((uint32_t *) temp_buffer + 6) = htonl(dpi_count);
	    *((uint32_t *) temp_buffer + 7) = htonl(dpi_offset);
	    if (sendn
		(_client_tcp_port->socket_fd, temp_buffer, dpi_offset, 0,
		    dtimeout) < 0) {
		_client_tcp_port->errors++;
		return;
	    }
	}
	break;

    case REMOTE_CMS_GET_BUF_NAME_REQUEST_TYPE:
	{
	    REMOTE_GET_BUF_NAME_REQUEST namereq;
	    namereq.buffer_number = buffer_number;
	    REMOTE_GET_BUF_NAME_REPLY *namereply = NULL;
	    namereply =
		(REMOTE_GET_BUF_NAME_REPLY *) server->
		process_request(&namereq);
	    memset(temp_buffer, 0, 40);
	    if (NULL != namereply) {
		putbe32(temp_buffer, _client_tcp_port->serial_number);
		putbe32(temp_buffer + 4, namereply->status);
		rtapi_strlcpy(temp_buffer + 8, namereply->name, 31);
		if (sendn
		    (_client_tcp_port->socket_fd, temp_buffer, 40, 0,
			dtimeout) < 0) {
		    _client_tcp_port->errors++;
		    return;
		}
	    } else {
		putbe32(temp_buffer, _client_tcp_port->serial_number);
		putbe32(temp_buffer + 4, CMS_SERVER_SIDE_ERROR);
		if (sendn
		    (_client_tcp_port->socket_fd, temp_buffer, 40, 0,
			dtimeout) < 0) {
		    _client_tcp_port->errors++;
		    return;
		}
	    }
	}
	break;

    case REMOTE_CMS_BLOCKING_READ_REQUEST_TYPE:
	{
	    TCPSVR_BLOCKING_READ_REQUEST *blocking_read_req;

#ifdef NO_THREADS
	    if (NULL == _client_tcp_port->blocking_read_req) {
		_client_tcp_port->blocking_read_req =
		    new TCPSVR_BLOCKING_READ_REQUEST();
	    }
	    blocking_read_req = _client_tcp_port->blocking_read_req;
#else
	    blocking_read_req;
	    = new TCPSVR_BLOCKING_READ_REQUEST();
#endif
	    blocking_read_req->buffer_number = buffer_number;
	    blocking_read_req->access_type =
		ntohl(*((uint32_t *) temp_buffer + 3));
	    blocking_read_req->last_id_read =
		ntohl(*((uint32_t *) temp_buffer + 4));
	    total_subdivisions = 1;
	    if (max_total_subdivisions > 1) {
		total_subdivisions =
		    server->get_total_subdivisions(buffer_number);
	    }
	    if (total_subdivisions > 1) {
		if (recvn
		    (_client_tcp_port->socket_fd,
			(char *) (((uint32_t *) temp_buffer) + 5), 8, 0, -1,
			NULL) < 0) {
		    rcs_print_error
			("Can not read from client port (%d) from %s\n",
			_client_tcp_port->socket_fd,
			inet_ntoa(_client_tcp_port->address.sin_addr));
		    _client_tcp_port->errors++;
		    return;
		}
		blocking_read_req->subdiv =
		    ntohl(*((uint32_t *) temp_buffer + 6));
	    } else {
		if (recvn
		    (_client_tcp_port->socket_fd,
			(char *) (((uint32_t *) temp_buffer) + 5), 4, 0, -1,
			NULL) < 0) {
		    rcs_print_error
			("Can not read from client port (%d) from %s\n",
			_client_tcp_port->socket_fd,
			inet_ntoa(_client_tcp_port->address.sin_addr));
		    _client_tcp_port->errors++;
		    return;
		}
	    }
	    blocking_read_req->timeout_millis =
		ntohl(*((uint32_t *) temp_buffer + 5));
	    blocking_read_req->server = server;
	    blocking_read_req->remport = this;
	    _client_tcp_port->blocking = 1;
	    blocking_read_req->_client_tcp_port = _client_tcp_port;
#ifdef POSIX_THREADS
	    int thr_retval = pthread_create(&(_client_tcp_port->threadId),	/* ptr to new-thread-id */
		NULL,		// pthread_attr_t *, ptr to attributes
		tcpsvr_handle_blocking_request,	// start_func
		blocking_read_req	// arg for start_func
		);
	    if (thr_retval != 0) {
		_client_tcp_port->blocking = 0;
		rcs_print_error("pthread_create error: thr_retval = %d\n",
		    thr_retval);
		rcs_print_error("pthread_create error: %d %s\n", errno,
		    strerror(errno));
		*((uint32_t *) temp_buffer) =
		    htonl(_client_tcp_port->serial_number);
		*((uint32_t *) temp_buffer + 1) =
		    htonl((unsigned long) CMS_SERVER_SIDE_ERROR);
		putbe32(temp_buffer + 8, 0);	/* size */
		putbe32(temp_buffer + 12, 0);	/* write_id */
		putbe32(temp_buffer + 16, 0);	/* was_read */
		sendn(_client_tcp_port->socket_fd, temp_buffer, 20, 0,
		    dtimeout);
		return;
	    }
#else
#ifdef NO_THREADS
	    int fork_ret = fork();
	    switch (fork_ret) {
	    case 0:		// child
		_client_tcp_port->threadId = getpid();
		tcpsvr_handle_blocking_request(blocking_read_req);
		exit(0);
		break;

	    case -1:		// Error
		rcs_print_error("fork error: %d %s\n", errno,
		    strerror(errno));
		_client_tcp_port->blocking = 0;
		putbe32(temp_buffer, _client_tcp_port->serial_number);
		putbe32(temp_buffer + 4, CMS_SERVER_SIDE_ERROR);
		putbe32(temp_buffer + 8, 0);
		putbe32(temp_buffer + 12, 0);
		putbe32(temp_buffer + 16, 0);
		sendn(_client_tcp_port->socket_fd, temp_buffer, 20, 0,
		    dtimeout);
		break;

	    default:		// parent;
		_client_tcp_port->threadId = fork_ret;
		break;
	    }
#else
	    rcs_print_error
		("Blocking read not supported on this platform.\n");
	    *((uint32_t *) temp_buffer) =
		htonl(_client_tcp_port->serial_number);
	    *((uint32_t *) temp_buffer + 1) =
		htonl((unsigned long) CMS_SERVER_SIDE_ERROR);
	    putbe32(temp_buffer + 8, 0);	/* size */
	    putbe32(temp_buffer + 12, 0);	/* write_id */
	    putbe32(temp_buffer + 16, 0);	/* was_read */
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 20, 0, dtimeout);
	    return;

#endif
#endif
	    tcpsvr_threads_created++;
	}
	break;

    case REMOTE_CMS_READ_REQUEST_TYPE:
	server->read_req.buffer_number = buffer_number;
	server->read_req.access_type = ntohl(*((uint32_t *) temp_buffer + 3));
	server->read_req.last_id_read = ntohl(*((uint32_t *) temp_buffer + 4));
	server->read_reply =
	    (REMOTE_READ_REPLY *) server->process_request(&server->read_req);
	if (max_total_subdivisions > 1) {
	    total_subdivisions =
		server->get_total_subdivisions(buffer_number);
	}
	if (total_subdivisions > 1) {
	    if (recvn
		(_client_tcp_port->socket_fd,
		    (char *) (((uint32_t *) temp_buffer) + 5), 4, 0, -1,
		    NULL) < 0) {
		rcs_print_error
		    ("Can not read from client port (%d) from %s\n",
		    _client_tcp_port->socket_fd,
		    inet_ntoa(_client_tcp_port->address.sin_addr));
		_client_tcp_port->errors++;
		return;
	    }
	    server->read_req.subdiv = ntohl(*((uint32_t *) temp_buffer + 5));
	} else {
	    server->read_req.subdiv = 0;
	}
	if (NULL == server->read_reply) {
	    rcs_print_error("Server could not process request.\n");
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    putbe32(temp_buffer + 4, CMS_SERVER_SIDE_ERROR);
	    putbe32(temp_buffer + 8, 0);
	    putbe32(temp_buffer + 12, 0);
	    putbe32(temp_buffer + 16, 0);
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 20, 0, dtimeout);
	    return;
	}
	putbe32(temp_buffer, _client_tcp_port->serial_number);
	putbe32(temp_buffer + 4, server->read_reply->status);
	putbe32(temp_buffer + 8, server->read_reply->size);
	putbe32(temp_buffer + 12, server->read_reply->write_id);
	putbe32(temp_buffer + 16, server->read_reply->was_read);
	if (server->read_reply->size < (0x2000 - 20)
	    && server->read_reply->size > 0) {
	    memcpy(temp_buffer + 20, server->read_reply->data,
		server->read_reply->size);
	    if (sendn
		(_client_tcp_port->socket_fd, temp_buffer,
		    20 + server->read_reply->size, 0, dtimeout) < 0) {
		_client_tcp_port->errors++;
		return;
	    }
	} else {
	    if (sendn
		(_client_tcp_port->socket_fd, temp_buffer, 20, 0,
		    dtimeout) < 0) {
		_client_tcp_port->errors++;
		return;
	    }
	    if (server->read_reply->size > 0) {
		if (sendn
		    (_client_tcp_port->socket_fd, server->read_reply->data,
			server->read_reply->size, 0, dtimeout) < 0) {
		    _client_tcp_port->errors++;
		    return;
		}
	    }
	}
	break;

    case REMOTE_CMS_WRITE_REQUEST_TYPE:
	server->write_req.buffer_number = buffer_number;
	server->write_req.access_type = ntohl(*((uint32_t *) temp_buffer + 3));
	server->write_req.size = ntohl(*((uint32_t *) temp_buffer + 4));
	total_subdivisions = 1;
	if (max_total_subdivisions > 1) {
	    total_subdivisions =
		server->get_total_subdivisions(buffer_number);
	}
	if (total_subdivisions > 1) {
	    if (recvn
		(_client_tcp_port->socket_fd,
		    (char *) (((uint32_t *) temp_buffer) + 5), 4, 0, -1,
		    NULL) < 0) {
		rcs_print_error
		    ("Can not read from client port (%d) from %s\n",
		    _client_tcp_port->socket_fd,
		    inet_ntoa(_client_tcp_port->address.sin_addr));
		_client_tcp_port->errors++;
		return;
	    }
	    server->write_req.subdiv = ntohl(*((uint32_t *) temp_buffer + 5));
	} else {
	    server->write_req.subdiv = 0;
	}
	if (server->write_req.size > 0) {
	    if (recvn
		(_client_tcp_port->socket_fd, server->write_req.data,
		    server->write_req.size, 0, -1, NULL) < 0) {
		_client_tcp_port->errors++;
		return;
	    }
	}
	REMOTE_WRITE_REPLY *reply;
	server->write_reply = reply =
	    (REMOTE_WRITE_REPLY *) server->process_request(&server->
	    write_req);
	if (((min_compatible_version < 2.58) && (min_compatible_version > 1e-6)) || server->write_reply->confirm_write) {
	    if (NULL == server->write_reply) {
		rcs_print_error("Server could not process request.\n");
	        putbe32(temp_buffer, reply->write_id);
		putbe32(temp_buffer + 4, CMS_SERVER_SIDE_ERROR);
		putbe32(temp_buffer + 8, 0);	/* was_read */
		sendn(_client_tcp_port->socket_fd, temp_buffer, 12, 0,
		    dtimeout);
		return;
	    }
	    putbe32(temp_buffer, reply->write_id);
	    putbe32(temp_buffer + 4, reply->status);
	    putbe32(temp_buffer + 8, reply->was_read);
	    if (sendn
		(_client_tcp_port->socket_fd, temp_buffer, 12, 0,
		    dtimeout) < 0) {
		_client_tcp_port->errors++;
	    }
	} else {
	    if (NULL == server->write_reply) {
		rcs_print_error("Server could not process request.\n");
	    }
	}
	break;

    case REMOTE_CMS_CHECK_IF_READ_REQUEST_TYPE:
	server->check_if_read_req.buffer_number = buffer_number;
	server->check_if_read_req.subdiv =
	    ntohl(*((uint32_t *) temp_buffer + 3));
	server->check_if_read_reply =
	    (REMOTE_CHECK_IF_READ_REPLY *) server->process_request(&server->
	    check_if_read_req);
	if (NULL == server->check_if_read_reply) {
	    rcs_print_error("Server could not process request.\n");
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    putbe32(temp_buffer + 4, CMS_SERVER_SIDE_ERROR);
	    putbe32(temp_buffer + 8, 0);	/* was_read */
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 12, 0, dtimeout);
	    return;
	}
	putbe32(temp_buffer, _client_tcp_port->serial_number);
	*((uint32_t *) temp_buffer + 1) =
	    htonl(server->check_if_read_reply->status);
	*((uint32_t *) temp_buffer + 2) =
	    htonl(server->check_if_read_reply->was_read);
	if (sendn(_client_tcp_port->socket_fd, temp_buffer, 12, 0, dtimeout) <
	    0) {
	    _client_tcp_port->errors++;
	}
	break;

    case REMOTE_CMS_GET_MSG_COUNT_REQUEST_TYPE:
	server->get_msg_count_req.buffer_number = buffer_number;
	server->get_msg_count_req.subdiv =
	    ntohl(*((uint32_t *) temp_buffer + 3));
	server->get_msg_count_reply =
	    (REMOTE_GET_MSG_COUNT_REPLY *) server->process_request(&server->
	    get_msg_count_req);
	if (NULL == server->get_msg_count_reply) {
	    rcs_print_error("Server could not process request.\n");
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    putbe32(temp_buffer + 4, CMS_SERVER_SIDE_ERROR);
	    putbe32(temp_buffer + 8, 0);	/* was_read */
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 12, 0, dtimeout);
	    return;
	}
	putbe32(temp_buffer, _client_tcp_port->serial_number);
	*((uint32_t *) temp_buffer + 1) =
	    htonl(server->get_msg_count_reply->status);
	*((uint32_t *) temp_buffer + 2) =
	    htonl(server->get_msg_count_reply->count);
	if (sendn(_client_tcp_port->socket_fd, temp_buffer, 12, 0, dtimeout) <
	    0) {
	    _client_tcp_port->errors++;
	}
	break;

    case REMOTE_CMS_GET_QUEUE_LENGTH_REQUEST_TYPE:
	server->get_queue_length_req.buffer_number = buffer_number;
	server->get_queue_length_req.subdiv =
	    ntohl(*((uint32_t *) temp_buffer + 3));
	server->get_queue_length_reply =
	    (REMOTE_GET_QUEUE_LENGTH_REPLY *) server->
	    process_request(&server->get_queue_length_req);
	if (NULL == server->get_queue_length_reply) {
	    rcs_print_error("Server could not process request.\n");
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    putbe32(temp_buffer + 4, CMS_SERVER_SIDE_ERROR);
	    putbe32(temp_buffer + 8, 0);	/* was_read */
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 12, 0, dtimeout);
	    return;
	}
	putbe32(temp_buffer, _client_tcp_port->serial_number);
	*((uint32_t *) temp_buffer + 1) =
	    htonl(server->get_queue_length_reply->status);
	*((uint32_t *) temp_buffer + 2) =
	    htonl(server->get_queue_length_reply->queue_length);
	if (sendn(_client_tcp_port->socket_fd, temp_buffer, 12, 0, dtimeout) <
	    0) {
	    _client_tcp_port->errors++;
	}
	break;

    case REMOTE_CMS_GET_SPACE_AVAILABLE_REQUEST_TYPE:
	server->get_space_available_req.buffer_number = buffer_number;
	server->get_space_available_req.subdiv =
	    ntohl(*((uint32_t *) temp_buffer + 3));
	server->get_space_available_reply =
	    (REMOTE_GET_SPACE_AVAILABLE_REPLY *) server->
	    process_request(&server->get_space_available_req);
	if (NULL == server->get_space_available_reply) {
	    rcs_print_error("Server could not process request.\n");
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    putbe32(temp_buffer + 4, CMS_SERVER_SIDE_ERROR);
	    putbe32(temp_buffer + 8, 0);	/* was_read */
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 12, 0, dtimeout);
	    return;
	}
	putbe32(temp_buffer, _client_tcp_port->serial_number);
	*((uint32_t *) temp_buffer + 1) =
	    htonl(server->get_space_available_reply->status);
	*((uint32_t *) temp_buffer + 2) =
	    htonl(server->get_space_available_reply->space_available);
	if (sendn(_client_tcp_port->socket_fd, temp_buffer, 12, 0, dtimeout) <
	    0) {
	    _client_tcp_port->errors++;
	}
	break;

    case REMOTE_CMS_CLEAR_REQUEST_TYPE:
	server->clear_req.buffer_number = buffer_number;
	server->clear_req.subdiv = ntohl(*((uint32_t *) temp_buffer + 3));
	server->clear_reply =
	    (REMOTE_CLEAR_REPLY *) server->process_request(&server->
	    clear_req);
	if (NULL == server->clear_reply) {
	    rcs_print_error("Server could not process request.\n");
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    putbe32(temp_buffer + 4, CMS_SERVER_SIDE_ERROR);
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 8, 0, dtimeout);
	    return;
	}
	putbe32(temp_buffer, _client_tcp_port->serial_number);
	putbe32(temp_buffer + 4, server->clear_reply->status);
	if (sendn(_client_tcp_port->socket_fd, temp_buffer, 8, 0, dtimeout) <
	    0) {
	    _client_tcp_port->errors++;
	}
	break;

    case REMOTE_CMS_CLEAN_REQUEST_TYPE:
	server->spawner_pid = server->server_pid;
	server->kill_server();
	break;

    case REMOTE_CMS_CLOSE_CHANNEL_REQUEST_TYPE:
	client_port_to_check = (CLIENT_TCP_PORT *) client_ports->get_head();
	while (NULL != client_port_to_check) {
	    if (client_port_to_check->socket_fd ==
		_client_tcp_port->socket_fd) {
		break;
	    }
	    client_port_to_check =
		(CLIENT_TCP_PORT *) client_ports->get_next();
	}
	FD_CLR(_client_tcp_port->socket_fd, &read_fd_set);
	close(_client_tcp_port->socket_fd);
	current_clients--;
	if (NULL != _client_tcp_port->subscriptions) {
	    remove_subscription_client(_client_tcp_port, buffer_number);
	}
	_client_tcp_port->socket_fd = -1;
	delete _client_tcp_port;
	client_ports->delete_current_node();
	break;

    case REMOTE_CMS_GET_KEYS_REQUEST_TYPE:
	server->get_keys_req.buffer_number = buffer_number;
	if (recvn(_client_tcp_port->socket_fd,
		server->get_keys_req.name, 16, 0, -1, NULL) < 0) {
	    _client_tcp_port->errors++;
	    return;
	}
	server->get_keys_reply =
	    (REMOTE_GET_KEYS_REPLY *) server->process_request(&server->
	    get_keys_req);
	if (NULL == server->get_keys_reply) {
	    rcs_print_error("Server could not process request.\n");
	    memset(temp_buffer, 0, 20);
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    server->gen_random_key(((char *) temp_buffer) + 4, 2);
	    server->gen_random_key(((char *) temp_buffer) + 12, 2);
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 20, 0, dtimeout);
	    return;
	} else {
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    memcpy(((char *) temp_buffer) + 4, server->get_keys_reply->key1,
		8);
	    memcpy(((char *) temp_buffer) + 12, server->get_keys_reply->key2,
		8);
	    /* successful ? */
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 20, 0, dtimeout);
	    return;
	}
	break;

    case REMOTE_CMS_LOGIN_REQUEST_TYPE:
	server->login_req.buffer_number = buffer_number;
	if (recvn(_client_tcp_port->socket_fd,
		server->login_req.name, 16, 0, -1, NULL) < 0) {
	    _client_tcp_port->errors++;
	    return;
	}
	if (recvn(_client_tcp_port->socket_fd,
		server->login_req.passwd, 16, 0, -1, NULL) < 0) {
	    _client_tcp_port->errors++;
	    return;
	}
	server->login_reply =
	    (REMOTE_LOGIN_REPLY *) server->process_request(&server->
	    login_req);
	if (NULL == server->login_reply) {
	    rcs_print_error("Server could not process request.\n");
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    putbe32(temp_buffer + 4, 0);	/* not successful */
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 8, 0, dtimeout);
	    return;
	} else {
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    putbe32(temp_buffer + 4, server->login_reply->success);
	    /* successful ? */
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 8, 0, dtimeout);
	    return;
	}
	break;

    case REMOTE_CMS_SET_SUBSCRIPTION_REQUEST_TYPE:
	server->set_subscription_req.buffer_number = buffer_number;
	server->set_subscription_req.subscription_type =
	    ntohl(*((uint32_t *) temp_buffer + 3));
	server->set_subscription_req.poll_interval_millis =
	    ntohl(*((uint32_t *) temp_buffer + 4));
	server->set_subscription_reply =
	    (REMOTE_SET_SUBSCRIPTION_REPLY *) server->
	    process_request(&server->set_subscription_req);
	if (NULL == server->set_subscription_reply) {
	    rcs_print_error("Server could not process request.\n");
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    putbe32(temp_buffer + 4, 0);	/* not successful */
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 8, 0, dtimeout);
	    return;
	} else {
	    if (server->set_subscription_reply->success) {
		if (server->set_subscription_req.subscription_type ==
		    CMS_POLLED_SUBSCRIPTION
		    || server->set_subscription_req.subscription_type ==
		    CMS_VARIABLE_SUBSCRIPTION) {
		    add_subscription_client(buffer_number,
			server->set_subscription_req.
			subscription_type,
			server->set_subscription_req.
			poll_interval_millis, _client_tcp_port);
		}
		if (server->set_subscription_req.subscription_type ==
		    CMS_NO_SUBSCRIPTION) {
		    remove_subscription_client(_client_tcp_port,
			buffer_number);
		}
	    }
	    putbe32(temp_buffer, _client_tcp_port->serial_number);
	    *((uint32_t *) temp_buffer + 1) =
		htonl(server->set_subscription_reply->success);
	    /* successful ? */
	    sendn(_client_tcp_port->socket_fd, temp_buffer, 8, 0, dtimeout);
	    return;
	}
	break;

    default:
	_client_tcp_port->errors++;
	rcs_print_error("Unrecognized request type received.(%ld)\n",
	    request_type);
	break;
    }
}

void CMS_SERVER_REMOTE_TCP_PORT::add_subscription_client(int buffer_number,
    int subscription_type, int poll_interval_millis, CLIENT_TCP_PORT * clnt)
{
    if (NULL == subscription_buffers) {
	subscription_buffers = new LinkedList();
    }
    if (NULL == subscription_buffers) {
	rcs_print_error("Can`t create subscription_buffers list.\n");
    }

    TCP_BUFFER_SUBSCRIPTION_INFO *buf_info =
	(TCP_BUFFER_SUBSCRIPTION_INFO *) subscription_buffers->get_head();
    while (NULL != buf_info) {
	if (buf_info->buffer_number == buffer_number) {
	    break;
	}
	buf_info =
	    (TCP_BUFFER_SUBSCRIPTION_INFO *) subscription_buffers->get_next();
    }
    if (NULL == buf_info) {
	buf_info = new TCP_BUFFER_SUBSCRIPTION_INFO();
	buf_info->buffer_number = buffer_number;
	buf_info->sub_clnt_info = new LinkedList();
	buf_info->list_id =
	    subscription_buffers->store_at_tail(buf_info, sizeof(*buf_info),
	    0);
    }
    buf_info->min_last_id = 0;
    if (NULL == clnt->subscriptions) {
	clnt->subscriptions = new LinkedList();
    }
    TCP_CLIENT_SUBSCRIPTION_INFO *temp_clnt_info =
	(TCP_CLIENT_SUBSCRIPTION_INFO *) clnt->subscriptions->get_head();
    while (temp_clnt_info != NULL) {
	if (temp_clnt_info->buffer_number == buffer_number) {
	    break;
	}
	temp_clnt_info =
	    (TCP_CLIENT_SUBSCRIPTION_INFO *) clnt->subscriptions->get_next();
    }
    if (NULL == temp_clnt_info) {
	temp_clnt_info = new TCP_CLIENT_SUBSCRIPTION_INFO();
	temp_clnt_info->last_sub_sent_time = 0.0;
	temp_clnt_info->buffer_number = buffer_number;
	temp_clnt_info->subscription_paused = 0;
	temp_clnt_info->last_id_read = 0;
	temp_clnt_info->sub_buf_info = buf_info;
	temp_clnt_info->clnt_port = clnt;
	temp_clnt_info->last_sub_sent_time = etime();
	temp_clnt_info->subscription_list_id =
	    clnt->subscriptions->store_at_tail(temp_clnt_info,
	    sizeof(*temp_clnt_info), 0);
	buf_info->sub_clnt_info->store_at_tail(temp_clnt_info,
	    sizeof(*temp_clnt_info), 0);
    }
    temp_clnt_info->subscription_type = subscription_type;
    temp_clnt_info->poll_interval_millis = poll_interval_millis;
    recalculate_polling_interval();
}

void CMS_SERVER_REMOTE_TCP_PORT::remove_subscription_client(CLIENT_TCP_PORT *
    clnt, int buffer_number)
{
    TCP_CLIENT_SUBSCRIPTION_INFO *temp_clnt_info =
	(TCP_CLIENT_SUBSCRIPTION_INFO *) clnt->subscriptions->get_head();
    while (temp_clnt_info != NULL) {
	if (temp_clnt_info->buffer_number == buffer_number) {
	    if (NULL != temp_clnt_info->sub_buf_info) {
		if (NULL != temp_clnt_info->sub_buf_info->sub_clnt_info) {
		    temp_clnt_info->sub_buf_info->sub_clnt_info->
			delete_node(temp_clnt_info->subscription_list_id);
		    if (temp_clnt_info->sub_buf_info->sub_clnt_info->
			list_size == 0) {
			subscription_buffers->delete_node(temp_clnt_info->
			    sub_buf_info->list_id);
			delete temp_clnt_info->sub_buf_info->sub_clnt_info;
			temp_clnt_info->sub_buf_info->sub_clnt_info = NULL;
			delete temp_clnt_info->sub_buf_info;
			temp_clnt_info->sub_buf_info = NULL;
		    }
		}
	    }
	    delete temp_clnt_info;
	    temp_clnt_info = NULL;
	    break;
	}
	temp_clnt_info =
	    (TCP_CLIENT_SUBSCRIPTION_INFO *) clnt->subscriptions->get_next();
    }
    recalculate_polling_interval();
}

void CMS_SERVER_REMOTE_TCP_PORT::recalculate_polling_interval()
{
    int min_poll_interval_millis = 30000;
    polling_enabled = 0;
    TCP_BUFFER_SUBSCRIPTION_INFO *buf_info =
	(TCP_BUFFER_SUBSCRIPTION_INFO *) subscription_buffers->get_head();
    while (NULL != buf_info) {
	TCP_CLIENT_SUBSCRIPTION_INFO *temp_clnt_info =
	    (TCP_CLIENT_SUBSCRIPTION_INFO *) buf_info->sub_clnt_info->
	    get_head();
	while (temp_clnt_info != NULL) {
	    if (temp_clnt_info->poll_interval_millis <
		min_poll_interval_millis
		&& temp_clnt_info->subscription_type ==
		CMS_POLLED_SUBSCRIPTION) {
		min_poll_interval_millis =
		    temp_clnt_info->poll_interval_millis;
		polling_enabled = 1;
	    }
	    temp_clnt_info = (TCP_CLIENT_SUBSCRIPTION_INFO *)
		buf_info->sub_clnt_info->get_next();
	}
	buf_info =
	    (TCP_BUFFER_SUBSCRIPTION_INFO *) subscription_buffers->get_next();
    }
    if (min_poll_interval_millis >= ((int) (clk_tck() * 1000.0))) {
	current_poll_interval_millis = min_poll_interval_millis;
    } else {
	current_poll_interval_millis = ((int) (clk_tck() * 1000.0));
    }
    select_timeout.tv_sec = current_poll_interval_millis / 1000;
    select_timeout.tv_usec = (current_poll_interval_millis % 1000) * 1000;
    dtimeout = (current_poll_interval_millis + 10) * 1000.0;
    if (dtimeout < 0.5) {
	dtimeout = 0.5;
    }
}

void CMS_SERVER_REMOTE_TCP_PORT::update_subscriptions()
{
    pid_t pid = getpid();
    pid_t tid = 0;
    CMS_SERVER *server;
    server = find_server(pid, tid);
    if (NULL == server) {
	rcs_print_error
	    ("CMS_SERVER_REMOTE_TCP_PORT::update_subscriptions Cannot find server object for pid = %d.\n",
	    pid);
	return;
    }
    if (NULL == subscription_buffers) {
	return;
    }
    double cur_time = etime();
    TCP_BUFFER_SUBSCRIPTION_INFO *buf_info =
	(TCP_BUFFER_SUBSCRIPTION_INFO *) subscription_buffers->get_head();
    while (NULL != buf_info) {
	server->read_req.buffer_number = buf_info->buffer_number;
	server->read_req.access_type = CMS_READ_ACCESS;
	server->read_req.last_id_read = buf_info->min_last_id;
	server->read_reply =
	    (REMOTE_READ_REPLY *) server->process_request(&server->read_req);
	if (NULL == server->read_reply) {
	    rcs_print_error("Server could not process request.\n");
	    buf_info = (TCP_BUFFER_SUBSCRIPTION_INFO *)
		subscription_buffers->get_next();
	    continue;
	}
	if (server->read_reply->write_id == buf_info->min_last_id ||
	    server->read_reply->size < 1) {
	    buf_info = (TCP_BUFFER_SUBSCRIPTION_INFO *)
		subscription_buffers->get_next();
	    continue;
	}
	putbe32(temp_buffer, 0);
	putbe32(temp_buffer + 4, server->read_reply->status);
	putbe32(temp_buffer + 8, server->read_reply->size);
	putbe32(temp_buffer + 12, server->read_reply->write_id);
	putbe32(temp_buffer + 16, server->read_reply->was_read);
	TCP_CLIENT_SUBSCRIPTION_INFO *temp_clnt_info =
	    (TCP_CLIENT_SUBSCRIPTION_INFO *) buf_info->sub_clnt_info->
	    get_head();
	buf_info->min_last_id = server->read_reply->write_id;
	while (temp_clnt_info != NULL) {
	    double time_diff = cur_time - temp_clnt_info->last_sub_sent_time;
	    int time_diff_millis = (int) ((double) time_diff * 1000.0);
	    rcs_print_debug(PRINT_SERVER_SUBSCRIPTION_ACTIVITY,
		"Subscription time_diff_millis=%d\n", time_diff_millis);
	    if (((temp_clnt_info->subscription_type == CMS_POLLED_SUBSCRIPTION
			&& time_diff_millis + 10 >=
			temp_clnt_info->poll_interval_millis)
		    || temp_clnt_info->subscription_type ==
		    CMS_VARIABLE_SUBSCRIPTION)
		&& temp_clnt_info->last_id_read !=
		server->read_reply->write_id) {
		temp_clnt_info->last_id_read = server->read_reply->write_id;
		temp_clnt_info->last_sub_sent_time = cur_time;
		temp_clnt_info->clnt_port->serial_number++;
		putbe32(temp_buffer, temp_clnt_info->clnt_port->serial_number);
		if (server->read_reply->size < 0x2000 - 20
		    && server->read_reply->size > 0) {
		    memcpy(temp_buffer + 20, server->read_reply->data,
			server->read_reply->size);
		    if (sendn
			(temp_clnt_info->clnt_port->socket_fd, temp_buffer,
			    20 + server->read_reply->size, 0, dtimeout) < 0) {
			temp_clnt_info->clnt_port->errors++;
			return;
		    }
		} else {
		    if (sendn(temp_clnt_info->clnt_port->socket_fd,
			    temp_buffer, 20, 0, dtimeout) < 0) {
			temp_clnt_info->clnt_port->errors++;
			return;
		    }
		    if (server->read_reply->size > 0) {
			if (sendn(temp_clnt_info->clnt_port->socket_fd,
				server->read_reply->data,
				server->read_reply->size, 0, dtimeout) < 0) {
			    temp_clnt_info->clnt_port->errors++;
			    return;
			}
		    }
		}
	    }
	    if (temp_clnt_info->last_id_read < buf_info->min_last_id) {
		buf_info->min_last_id = temp_clnt_info->last_id_read;
	    }
	    temp_clnt_info = (TCP_CLIENT_SUBSCRIPTION_INFO *)
		buf_info->sub_clnt_info->get_next();
	}
	buf_info =
	    (TCP_BUFFER_SUBSCRIPTION_INFO *) subscription_buffers->get_next();
    }
}

TCP_BUFFER_SUBSCRIPTION_INFO::TCP_BUFFER_SUBSCRIPTION_INFO()
{
    buffer_number = -1;
    min_last_id = 0;
    list_id = -1;
    sub_clnt_info = NULL;
}

TCP_BUFFER_SUBSCRIPTION_INFO::~TCP_BUFFER_SUBSCRIPTION_INFO()
{
    buffer_number = -1;
    min_last_id = 0;
    list_id = -1;
    if (NULL != sub_clnt_info) {
	delete sub_clnt_info;
	sub_clnt_info = NULL;
    }
}

TCP_CLIENT_SUBSCRIPTION_INFO::TCP_CLIENT_SUBSCRIPTION_INFO()
{
    subscription_type = CMS_NO_SUBSCRIPTION;
    poll_interval_millis = 30000;
    last_sub_sent_time = 0.0;
    subscription_list_id = -1;
    buffer_number = -1;
    subscription_paused = 0;
    last_id_read = 0;
    sub_buf_info = NULL;
    clnt_port = NULL;
}

TCP_CLIENT_SUBSCRIPTION_INFO::~TCP_CLIENT_SUBSCRIPTION_INFO()
{
    subscription_type = CMS_NO_SUBSCRIPTION;
    poll_interval_millis = 30000;
    last_sub_sent_time = 0.0;
    subscription_list_id = -1;
    buffer_number = -1;
    subscription_paused = 0;
    last_id_read = 0;
    sub_buf_info = NULL;
    clnt_port = NULL;
}

CLIENT_TCP_PORT::CLIENT_TCP_PORT()
  : serial_number(0),
    errors(0),
    max_errors(50),
    socket_fd(-1),
    subscriptions(NULL),
    tid(-1),
    pid(-1),
    blocking(0),
    threadId(0),
    blocking_read_req(NULL),
    diag_info(NULL)
{
    memset(&address, 0, sizeof(address));
    //address.sin_port = 0;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
}

CLIENT_TCP_PORT::~CLIENT_TCP_PORT()
{
    if (socket_fd > 0) {
	close(socket_fd);
	socket_fd = -1;
    }
    if (NULL != subscriptions) {
	TCP_CLIENT_SUBSCRIPTION_INFO *sub_info =
	    (TCP_CLIENT_SUBSCRIPTION_INFO *) subscriptions->get_head();
	while (NULL != sub_info) {
	    delete sub_info;
	    sub_info =
		(TCP_CLIENT_SUBSCRIPTION_INFO *) subscriptions->get_next();
	}
	delete subscriptions;
	subscriptions = NULL;
    }
#ifdef NO_THREADS
    if (NULL != blocking_read_req) {
	delete blocking_read_req;
	blocking_read_req = NULL;
    }
#endif
    if (NULL != diag_info) {
	delete diag_info;
	diag_info = NULL;
    }
}
