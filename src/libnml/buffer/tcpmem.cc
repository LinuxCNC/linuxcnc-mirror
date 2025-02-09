/********************************************************************
* Description: tcpmem.cc
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change: 
********************************************************************/

#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>		// strtol()
#include <unistd.h>
#include <string.h>		// strstr()
#include <errno.h>		// errno, strerror()
#include <signal.h>		// signal, SIG_ERR, SIGPIPE
#include <ctype.h>		// isdigit()
#include <arpa/inet.h>		/* inet_ntoa */
#include <sys/socket.h>
#include <sys/time.h>           /* struct timeval */
#include <netinet/in.h>         /* sockaddr_in */
#include <netdb.h>
#include <math.h>		/* fmod() */

#ifdef __cplusplus
}
#endif
#include "rem_msg.hh"		/* REMOTE_CMS_READ_REQUEST_TYPE, etc. */
#include "rcs_print.hh"		/* rcs_print_error() */
#include "cmsdiag.hh"
#define DEFAULT_MAX_CONSECUTIVE_TIMEOUTS (-1)
#include "timer.hh"		/* esleep() */
#include "tcpmem.hh"
#include "recvn.h"		/* recvn() */
#include "sendn.h"		/* sendn() */
#include "tcp_opts.hh"		/* SET_TCP_NODELAY */
#include "linklist.hh"          /* LinkedList */

int tcpmem_sigpipe_count = 0;
int last_sig = 0;

void tcpmem_sigpipe_handler(int sig)
{
    last_sig = sig;
    tcpmem_sigpipe_count++;
}

TCPMEM::TCPMEM(const char *_bufline, const char *_procline)
  : CMS(_bufline, _procline),
    diag_info_buf{},
    recvd_bytes(0),
    serial_number(0),
    returned_serial_number(0),
    subscription_type(CMS_NO_SUBSCRIPTION),
    poll_interval_millis(30000),
    server_host_entry(NULL),
    socket_fd(0),
    temp_buffer{},
    timedout_request(),
    bytes_to_throw_away(0),
    write_socket_fd(0),
    read_socket_fd(0),
    write_serial_number(0),
    read_serial_number(0),
    timedout_request_status(CMS_STATUS_NOT_SET),
    timedout_request_writeid(0),
    max_consecutive_timeouts(DEFAULT_MAX_CONSECUTIVE_TIMEOUTS),
    waiting_for_message(0),
    waiting_message_size(0),
    waiting_message_id(0),
    autoreconnect(1),
    reconnect_needed(0),
    sigpipe_count(0),
    old_handler((void (*)(int))SIG_ERR),
    subscription_count(0)
{
    char *max_consecutive_timeouts_string;
    max_consecutive_timeouts_string = strstr(ProcessLine, "max_timeouts=");
    polling = (NULL != strstr(proclineupper, "POLL"));
    if (NULL != max_consecutive_timeouts_string) {
	max_consecutive_timeouts_string += strlen("max_timeouts=");
	if (!strncmp(max_consecutive_timeouts_string, "INF", 3)) {
	    max_consecutive_timeouts = -1;
	} else {
	    max_consecutive_timeouts =
		strtol(max_consecutive_timeouts_string, (char **) NULL, 0);
	}
    }

    char *sub_info_string = NULL;
    sub_info_string = strstr(ProcessLine, "sub=");
    if (NULL != sub_info_string) {
	if (!strncmp(sub_info_string + 4, "none", 4)) {
	    subscription_type = CMS_NO_SUBSCRIPTION;
	} else if (!strncmp(sub_info_string + 4, "var", 3)) {
	    subscription_type = CMS_VARIABLE_SUBSCRIPTION;
	} else {
	    poll_interval_millis =
		((int) (atof(sub_info_string + 4) * 1000.0));
	    subscription_type = CMS_POLLED_SUBSCRIPTION;
	}
    }
    if (NULL != strstr(ProcessLine, "noreconnect")) {
	autoreconnect = 0;
    }

    /* Set up the socket address structure. */
    memset(&server_socket_address, 0, sizeof(server_socket_address));
    server_socket_address.sin_family = AF_INET;
    server_socket_address.sin_port = htons(((u_short) tcp_port_number));
    int hostname_was_address = 0;
    char bufferhost_first_char = BufferHost[0];
    if (bufferhost_first_char >= '0' && bufferhost_first_char <= '9') {
	server_socket_address.sin_addr.s_addr = inet_addr(BufferHost);
	if (server_socket_address.sin_addr.s_addr != INADDR_NONE) {
	    hostname_was_address = 1;
	}
    }

    if (!hostname_was_address) {
	/* Get the IP address of the server using it's BufferHost. */
	server_host_entry = gethostbyname(BufferHost);
	if (NULL == server_host_entry) {
	    status = CMS_CONFIG_ERROR;
	    autoreconnect = 0;
	    rcs_print_error("TCPMEM: Couldn't get host address for (%s).\n",
		BufferHost);
	    return;
	}
	server_socket_address.sin_addr.s_addr =
	    *((int *) server_host_entry->h_addr_list[0]);
	server_socket_address.sin_family = server_host_entry->h_addrtype;
    }
    rcs_print_debug(PRINT_CMS_CONFIG_INFO,
	"Using server on %s with IP address %s and port %d.\n",
	BufferHost,
	inet_ntoa(server_socket_address.sin_addr), tcp_port_number);

    reconnect();

    if (status >= 0 &&
	(min_compatible_version > 2.58 || min_compatible_version < 1e-6)) {
	verify_bufname();
	if (status < 0) {
	    rcs_print_error("TCPMEM: verify_bufname() failed.\n");
	}
    }

    if (status >= 0 && enable_diagnostics &&
	(min_compatible_version > 3.71 || min_compatible_version < 1e-6)) {
	send_diag_info();
    }
}

static void put32(char *addr, uint32_t val) {
    memcpy(addr, &val, sizeof(val));
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

void
  TCPMEM::send_diag_info()
{
    if (polling) {
	return;
    }
    if (NULL == dpi) {
	return;
    }
    disable_sigpipe();

    set_socket_fds(read_socket_fd);
    memset(diag_info_buf, 0, 88);
    putbe32(diag_info_buf, (uint32_t)serial_number);
    putbe32(diag_info_buf + 4, REMOTE_CMS_SET_DIAG_INFO_REQUEST_TYPE);
    putbe32(diag_info_buf + 8, buffer_number);
    strncpy(diag_info_buf + 20, dpi->name, 16);
    strncpy(diag_info_buf + 36, dpi->host_sysinfo, 32);
    putbe32(diag_info_buf + 68, (uint32_t) dpi->pid);
    putbe32(diag_info_buf + 72, (uint32_t) connection_number);
    memcpy(diag_info_buf + 76, &(dpi->rcslib_ver), 8);
    put32(diag_info_buf + 84, (uint32_t) 0x11223344);
    if (sendn(socket_fd, diag_info_buf, 88, 0, timeout) < 0) {
	reconnect_needed = 1;
	fatal_error_occurred = 1;
	reenable_sigpipe();
	status = CMS_MISC_ERROR;
	return;
    }
    serial_number++;
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM sending request: fd = %d, serial_number=%ld, request_type=%d, buffer_number=%ld\n",
	socket_fd, serial_number,
	ntohl(*((uint32_t *) diag_info_buf + 1)), buffer_number);
    reenable_sigpipe();

}

void TCPMEM::verify_bufname()
{
    if (polling) {
	return;
    }
    disable_sigpipe();

    set_socket_fds(read_socket_fd);

    putbe32(temp_buffer, (uint32_t) serial_number);
    putbe32(temp_buffer + 4, REMOTE_CMS_GET_BUF_NAME_REQUEST_TYPE);
    putbe32(temp_buffer + 8, buffer_number);
    if (sendn(socket_fd, temp_buffer, 20, 0, timeout) < 0) {
	reconnect_needed = 1;
	fatal_error_occurred = 1;
	reenable_sigpipe();
	status = CMS_MISC_ERROR;
	return;
    }
    serial_number++;
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM sending request: fd = %d, serial_number=%ld, request_type=%d, buffer_number=%ld\n",
	socket_fd, serial_number,
	ntohl(*((uint32_t *) temp_buffer + 1)), buffer_number);
    if (recvn(socket_fd, temp_buffer, 40, 0, timeout, &recvd_bytes) < 0) {
	if (recvn_timedout) {
	    bytes_to_throw_away = 40;
	    return;
	}
    }
    returned_serial_number = getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	socket_fd, returned_serial_number, buffer_number);
    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
	    returned_serial_number, serial_number);
	reconnect_needed = 1;
	fatal_error_occurred = 1;
	reenable_sigpipe();
	status = CMS_MISC_ERROR;
	return;
    }
    status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
    if (status < 0) {
	return;
    }
    if (strncmp(temp_buffer + 8, BufferName, 31)) {
	rcs_print_error
	    ("TCPMEM: The buffer (%s) is registered on TCP port %d of host %s with buffer number %ld.\n",
	    ((char *) temp_buffer + 8), tcp_port_number, BufferHost,
	    buffer_number);
	rcs_print_error
	    ("TCPMEM: However, this process (%s) is attempting to connect to the buffer %s at the same location.\n",
	    ProcessName, BufferName);
	status = CMS_RESOURCE_CONFLICT_ERROR;
	return;
    }
    reenable_sigpipe();
}

CMS_DIAGNOSTICS_INFO *TCPMEM::get_diagnostics_info()
{
    if (polling) {
	return (NULL);
    }
    disable_sigpipe();

    if (((int) handle_old_replies()) < 0) {
	reenable_sigpipe();
	return (NULL);
    }

    set_socket_fds(read_socket_fd);

    putbe32(temp_buffer, serial_number);
    putbe32(temp_buffer + 4, REMOTE_CMS_GET_DIAG_INFO_REQUEST_TYPE);
    putbe32(temp_buffer + 8, (uint32_t) buffer_number);
    if (sendn(socket_fd, temp_buffer, 20, 0, timeout) < 0) {
	reconnect_needed = 1;
	fatal_error_occurred = 1;
	reenable_sigpipe();
	status = CMS_MISC_ERROR;
	return (NULL);
    }
    memset(temp_buffer, 0, 0x2000);
    serial_number++;
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM sending request: fd = %d, serial_number=%ld, request_type=%d, buffer_number=%ld\n",
	socket_fd, serial_number,
	ntohl(*((uint32_t *) temp_buffer + 1)), buffer_number);
    if (recvn(socket_fd, temp_buffer, 32, 0, -1.0, &recvd_bytes) < 0) {
	if (recvn_timedout) {
	    bytes_to_throw_away = 32;
	}
	return (NULL);
    }
    recvd_bytes = 0;
    returned_serial_number = getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	socket_fd, returned_serial_number, buffer_number);
    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
	    returned_serial_number, serial_number);
	reconnect_needed = 1;
	fatal_error_occurred = 1;
	reenable_sigpipe();
	status = CMS_MISC_ERROR;
	return (NULL);
    }
    status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
    if (status < 0) {
	return (NULL);
    }
    if (NULL == di) {
	di = new CMS_DIAGNOSTICS_INFO();
	di->dpis = new LinkedList();
    } else {
	di->dpis->delete_members();
    }
    di->last_writer_dpi = NULL;
    di->last_reader_dpi = NULL;
    di->last_writer = ntohl(*((uint32_t *) temp_buffer + 2));
    di->last_reader = ntohl(*((uint32_t *) temp_buffer + 3));
    double server_time;
    memcpy(&server_time, temp_buffer + 16, 8);
    double local_time = etime();
    double diff_time = local_time - server_time;
    int dpi_count = ntohl(*((uint32_t *) temp_buffer + 6));
    int dpi_max_size = ntohl(*((uint32_t *) temp_buffer + 7));
    if (dpi_max_size > 32 && dpi_max_size < 0x2000) {
	if (recvn
	    (socket_fd, temp_buffer + 32, dpi_max_size - 32, 0, -1.0,
		&recvd_bytes) < 0) {
	    if (recvn_timedout) {
		bytes_to_throw_away = dpi_max_size - 32;
		return (NULL);
	    }
	}
	recvd_bytes = 0;
	int dpi_offset = 32;
	CMS_DIAG_PROC_INFO cms_dpi;
	for (int i = 0; i < dpi_count && dpi_offset < dpi_max_size; i++) {
	    // Cleared in constructor
	    // memset(&cms_dpi, 0, sizeof(CMS_DIAG_PROC_INFO));
	    memcpy(cms_dpi.name, temp_buffer + dpi_offset, 16);
	    dpi_offset += 16;
	    memcpy(cms_dpi.host_sysinfo, temp_buffer + dpi_offset, 32);
	    dpi_offset += 32;
	    cms_dpi.pid =
		ntohl(*((uint32_t *) ((char *) temp_buffer + dpi_offset)));
	    dpi_offset += 4;
	    memcpy(&(cms_dpi.rcslib_ver), temp_buffer + dpi_offset, 8);
	    dpi_offset += 8;
	    cms_dpi.access_type = (CMS_INTERNAL_ACCESS_TYPE)
		ntohl(*((uint32_t *) ((char *) temp_buffer + dpi_offset)));
	    dpi_offset += 4;
	    cms_dpi.msg_id =
		ntohl(*((uint32_t *) ((char *) temp_buffer + dpi_offset)));
	    dpi_offset += 4;
	    cms_dpi.msg_size =
		ntohl(*((uint32_t *) ((char *) temp_buffer + dpi_offset)));
	    dpi_offset += 4;
	    cms_dpi.msg_type =
		ntohl(*((uint32_t *) ((char *) temp_buffer + dpi_offset)));
	    dpi_offset += 4;
	    cms_dpi.number_of_accesses =
		ntohl(*((uint32_t *) ((char *) temp_buffer + dpi_offset)));
	    dpi_offset += 4;
	    cms_dpi.number_of_new_messages =
		ntohl(*((uint32_t *) ((char *) temp_buffer + dpi_offset)));
	    dpi_offset += 4;
	    memcpy(&(cms_dpi.bytes_moved), temp_buffer + dpi_offset, 8);
	    dpi_offset += 8;
	    memcpy(&(cms_dpi.bytes_moved_across_socket),
		temp_buffer + dpi_offset, 8);
	    dpi_offset += 8;
	    memcpy(&(cms_dpi.last_access_time), temp_buffer + dpi_offset, 8);
	    if (cmsdiag_timebias_set) {
		cms_dpi.last_access_time += diff_time - cmsdiag_timebias;
	    }
	    dpi_offset += 8;
	    memcpy(&(cms_dpi.first_access_time), temp_buffer + dpi_offset, 8);
	    if (cmsdiag_timebias_set) {
		cms_dpi.first_access_time += diff_time - cmsdiag_timebias;
	    }
	    dpi_offset += 8;
	    memcpy(&(cms_dpi.min_difference), temp_buffer + dpi_offset, 8);
	    dpi_offset += 8;
	    memcpy(&(cms_dpi.max_difference), temp_buffer + dpi_offset, 8);
	    dpi_offset += 8;
	    di->dpis->store_at_tail(&cms_dpi, sizeof(CMS_DIAG_PROC_INFO), 1);
	    int is_last_writer =
		ntohl(*((uint32_t *) ((char *) temp_buffer + dpi_offset)));
	    dpi_offset += 4;
	    if (is_last_writer) {
		di->last_writer_dpi =
		    (CMS_DIAG_PROC_INFO *) di->dpis->get_tail();
	    }
	    int is_last_reader =
		ntohl(*((uint32_t *) ((char *) temp_buffer + dpi_offset)));
	    dpi_offset += 4;
	    if (is_last_reader) {
		di->last_reader_dpi =
		    (CMS_DIAG_PROC_INFO *) di->dpis->get_tail();
	    }
	}
    }
    reenable_sigpipe();
    return di;
}

void TCPMEM::reconnect()
{
    if (socket_fd > 0) {
	disconnect();
    }
    subscription_count = 0;
    timedout_request = NO_REMOTE_CMS_REQUEST;
    bytes_to_throw_away = 0;
    recvd_bytes = 0;
    socket_fd = 0;
    waiting_for_message = 0;
    waiting_message_size = 0;
    waiting_message_id = 0;
    serial_number = 0;

    rcs_print_debug(PRINT_CMS_CONFIG_INFO, "Creating socket . . .\n");

    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
	rcs_print_error("TCPMEM: Error from socket() (errno = %d:%s)\n",
	    errno, strerror(errno));

	status = CMS_CREATE_ERROR;
	return;
    }
    rcs_print_debug(PRINT_CMS_CONFIG_INFO, "Setting socket options . . . \n");
    if (set_tcp_socket_options(socket_fd) < 0) {
	return;
    }
    struct timeval tm;
    int socket_ret;
    double start_time, current_time;
    fd_set fds;
    sockaddr_in cli_addr;
    cli_addr.sin_family = AF_INET;
    cli_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    cli_addr.sin_port = htons(0);
    rcs_print_debug(PRINT_CMS_CONFIG_INFO, "Binding . . . \n");
    if (bind(socket_fd, (struct sockaddr *) &cli_addr, sizeof(cli_addr)) < 0) {
	rcs_print_error("TCPMEM: bind error %d = %s\n", errno,
	    strerror(errno));
	status = CMS_CREATE_ERROR;
    }
    rcs_print_debug(PRINT_CMS_CONFIG_INFO, "Connecting . . .\n");
    if (connect(socket_fd, (struct sockaddr *) &server_socket_address,
	    sizeof(server_socket_address)) < 0) {
	if (EINPROGRESS == errno) {

	    tm.tv_sec = (long) timeout;
	    tm.tv_sec = (long) (fmod(timeout, 1.0) * 1e6);
	    FD_ZERO(&fds);
	    FD_SET(socket_fd, &fds);
	    start_time = etime();
	    while (!(socket_ret = select(socket_fd + 1,
			(fd_set *) NULL, &fds, (fd_set *) NULL, &tm))) {
		FD_SET(socket_fd, &fds);
		esleep(0.001);
		current_time = etime();
		double timeleft = start_time + timeout - current_time;
		if (timeleft <= 0.0 && timeout >= 0.0) {
		    if (!reconnect_needed) {
			rcs_print_error
			    ("TCPMEM: Timed out waiting for connection.\n");
		    }
		    status = CMS_NO_SERVER_ERROR;
		    return;
		}
		tm.tv_sec = (long) timeleft;
		tm.tv_sec = (long) (fmod(timeleft, 1.0) * 1e6);
	    }

	    if (-1 == socket_ret) {
		rcs_print_error("select error: %d -- %s\n", errno,
		    strerror(errno));
		rcs_print_error("TCPMEM: Couldn't connect.\n");
		status = CMS_NO_SERVER_ERROR;
		return;
	    }
	} else {
	    rcs_print_error("connect error: %d -- %s\n", errno,
		strerror(errno));
	    rcs_print_error
		("TCPMEM: Error trying to connect to TCP port %d of host %s(%s). sin_family=%d\n",
		ntohs(server_socket_address.sin_port), BufferHost,
		inet_ntoa(server_socket_address.sin_addr),
		server_socket_address.sin_family);
	    status = CMS_NO_SERVER_ERROR;
	    return;
	}
    }
    read_socket_fd = socket_fd;

    memset(temp_buffer, 0, 32);
    if (total_subdivisions > 1) {
	subscription_type = CMS_NO_SUBSCRIPTION;
    }

    if (subscription_type != CMS_NO_SUBSCRIPTION) {
	verify_bufname();
	if (status < 0) {
	    rcs_print_error("TCPMEM: verify_bufname() failed\n");
	    return;
	}
	putbe32(temp_buffer, (uint32_t) serial_number);
	putbe32(temp_buffer + 4, REMOTE_CMS_SET_SUBSCRIPTION_REQUEST_TYPE);
	putbe32(temp_buffer + 8, (uint32_t) buffer_number);
	putbe32(temp_buffer + 12, (uint32_t) subscription_type);
	putbe32(temp_buffer + 16, (uint32_t) poll_interval_millis);
	if (sendn(socket_fd, temp_buffer, 20, 0, 30) < 0) {
	    rcs_print_error("Can`t setup subscription.\n");
	    subscription_type = CMS_NO_SUBSCRIPTION;
	} else {
	    serial_number++;
	    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
		"TCPMEM sending request: fd = %d, serial_number=%ld, request_type=%d, buffer_number=%ld\n",
		socket_fd, serial_number,
		ntohl(*((uint32_t *) temp_buffer + 1)), buffer_number);
	    memset(temp_buffer, 0, 20);
	    recvd_bytes = 0;
	    if (recvn(socket_fd, temp_buffer, 8, 0, 30, &recvd_bytes) < 0) {
		rcs_print_error("Can`t setup subscription.\n");
		subscription_type = CMS_NO_SUBSCRIPTION;
	    }
	    if (!getbe32(temp_buffer+4)) {
		rcs_print_error("Can`t setup subscription.\n");
		subscription_type = CMS_NO_SUBSCRIPTION;
	    }

	    bytes_to_throw_away = 8 - recvd_bytes;
	    if (bytes_to_throw_away < 0 || bytes_to_throw_away > 8) {
		bytes_to_throw_away = 0;
	    }
	    recvd_bytes = 0;
	}
	memset(temp_buffer, 0, 20);
    }
    if (subscription_type != CMS_NO_SUBSCRIPTION) {
	polling = 1;
    }

    if (polling) {
	make_tcp_socket_nonblocking(socket_fd);
	write_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (write_socket_fd < 0) {
	    rcs_print_error("TCPMEM: Error from socket() (errno = %d:%s)\n",
		errno, strerror(errno));
	    status = CMS_CREATE_ERROR;
	    return;
	}
	rcs_print_debug(PRINT_CMS_CONFIG_INFO,
	    "Setting socket options . . . \n");
	if (set_tcp_socket_options(write_socket_fd) < 0) {
	    return;
	}
	rcs_print_debug(PRINT_CMS_CONFIG_INFO, "Binding . . . \n");
	if (bind
	    (write_socket_fd, (struct sockaddr *) &cli_addr,
		sizeof(cli_addr)) < 0) {
	    rcs_print_error("TCPMEM: bind error %d = %s\n", errno,
		strerror(errno));
	    status = CMS_CREATE_ERROR;
	}
	rcs_print_debug(PRINT_CMS_CONFIG_INFO, "Connecting . . .\n");
	if (connect
	    (write_socket_fd, (struct sockaddr *) &server_socket_address,
		sizeof(server_socket_address)) < 0) {
	    if (EINPROGRESS == errno) {
		FD_ZERO(&fds);
		FD_SET(write_socket_fd, &fds);
		start_time = etime();
		tm.tv_sec = (long) timeout;
		tm.tv_sec = (long) (fmod(timeout, 1.0) * 1e6);
		while (!(socket_ret = select(write_socket_fd + 1,
			    (fd_set *) NULL, &fds, (fd_set *) NULL, &tm))) {
		    FD_SET(write_socket_fd, &fds);
		    esleep(0.001);
		    current_time = etime();
		    double timeleft = start_time + timeout - current_time;
		    if (timeleft <= 0.0 && timeout >= 0.0) {
			rcs_print_error
			    ("TCPMEM: Timed out waiting for connection.\n");
			status = CMS_NO_SERVER_ERROR;
			return;
		    }
		    tm.tv_sec = (long) timeleft;
		    tm.tv_sec = (long) (fmod(timeleft, 1.0) * 1e6);
		}
		if (-1 == socket_ret) {
		    rcs_print_error("select error: %d -- %s\n", errno,
			strerror(errno));
		    rcs_print_error("TCPMEM: Couldn't connect.\n");
		    status = CMS_NO_SERVER_ERROR;
		    return;
		}
	    } else {
		rcs_print_error("connect error: %d -- %s\n", errno,
		    strerror(errno));
		rcs_print_error
		    ("TCPMEM: Error trying to connect to TCP port %d of host %s.\n",
		    ntohs(server_socket_address.sin_port), BufferHost);
	    }
	}
	timeout = 0;
    } else {
	write_socket_fd = read_socket_fd;
    }
    reconnect_needed = 0;
    fatal_error_occurred = 0;

}

TCPMEM::~TCPMEM()
{
    disconnect();
}

void TCPMEM::disconnect()
{
    if (write_socket_fd > 0 && write_socket_fd != socket_fd) {
	if (status != CMS_CONFIG_ERROR && status != CMS_CREATE_ERROR) {
	    if (delete_totally) {
		putbe32(temp_buffer, (uint32_t) serial_number);
		putbe32(temp_buffer + 4, REMOTE_CMS_CLEAN_REQUEST_TYPE);
		putbe32(temp_buffer + 8, (uint32_t) buffer_number);
		sendn(write_socket_fd, temp_buffer, 20, 0, -1);
	    }
	}
	close(write_socket_fd);
	write_socket_fd = 0;
    }

    if (socket_fd > 0) {
	if (status != CMS_CONFIG_ERROR && status != CMS_CREATE_ERROR) {
	    if (delete_totally) {
		putbe32(temp_buffer, (uint32_t) serial_number);
		putbe32(temp_buffer + 4, REMOTE_CMS_CLEAN_REQUEST_TYPE);
		putbe32(temp_buffer + 8, (uint32_t) buffer_number);
		sendn(socket_fd, temp_buffer, 20, 0, -1);
	    }
	}
	close(socket_fd);
	socket_fd = 0;
    }
}

CMS_STATUS TCPMEM::handle_old_replies()
{
    long message_size;

    timedout_request_writeid = 0;
    status = CMS_STATUS_NOT_SET;
    switch (timedout_request) {
    case REMOTE_CMS_READ_REQUEST_TYPE:
	if (!waiting_for_message) {
	    if (recvn(socket_fd, temp_buffer, 20, 0, timeout, &recvd_bytes) <
		0) {
		if (recvn_timedout) {
		    if (polling) {
			return status;
		    } else {
			consecutive_timeouts++;
			if (consecutive_timeouts > max_consecutive_timeouts &&
			    max_consecutive_timeouts > 0) {
			    rcs_print_error
				("CMS: %d consecutive timeouts have occurred. -- Stop trying.\n",
				consecutive_timeouts);
			    fatal_error_occurred = 1;
			    reconnect_needed = 1;
			}
			return (status = CMS_TIMED_OUT);
		    }
		} else {
		    recvd_bytes = 0;
		    fatal_error_occurred = 1;
		    return (status = CMS_MISC_ERROR);
		}
	    }
	    recvd_bytes = 0;
	    returned_serial_number =
		(CMS_STATUS) getbe32(temp_buffer);
	    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
		"TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
		socket_fd, returned_serial_number, buffer_number);
	    if (returned_serial_number != serial_number) {
		rcs_print_error
		    ("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
		    returned_serial_number, serial_number);
		if (subscription_type == CMS_NO_SUBSCRIPTION) {
		    fatal_error_occurred = 1;
		    reconnect_needed = 1;
		    return (status = CMS_MISC_ERROR);
		} else {
		    serial_number = returned_serial_number;
		}
	    }
	    message_size = ntohl(*((uint32_t *) temp_buffer + 2));
	    timedout_request_status =
		(CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
	    timedout_request_writeid = ntohl(*((uint32_t *) temp_buffer + 3));
	    header.was_read = ntohl(*((uint32_t *) temp_buffer + 4));
	    if (message_size > max_encoded_message_size) {
		rcs_print_error("Received message is too big. (%ld > %ld)\n",
		    message_size, max_encoded_message_size);
		fatal_error_occurred = 1;
		reconnect_needed = 1;
		return (status = CMS_INSUFFICIENT_SPACE_ERROR);
	    }
	} else {
	    message_size = waiting_message_size;
	}
	if (message_size > 0) {
	    if (recvn
		(socket_fd, encoded_data, message_size, 0, timeout,
		    &recvd_bytes) < 0) {
		if (recvn_timedout) {
		    if (!waiting_for_message) {
			waiting_message_id = timedout_request_writeid;
			waiting_message_size = message_size;
		    }
		    waiting_for_message = 1;
		    timedout_request_writeid = 0;
		    if (polling) {
			return status;
		    } else {
			consecutive_timeouts++;
			if (consecutive_timeouts > max_consecutive_timeouts &&
			    max_consecutive_timeouts > 0) {
			    rcs_print_error
				("CMS: %d consecutive timeouts have occurred. -- Stop trying.\n",
				consecutive_timeouts);
			    fatal_error_occurred = 1;
			    reconnect_needed = 1;
			}
			return (status = CMS_TIMED_OUT);
		    }
		} else {
		    recvd_bytes = 0;
		    fatal_error_occurred = 1;
		    reconnect_needed = 1;
		    return (status = CMS_MISC_ERROR);
		}
	    }
	    recvd_bytes = 0;
	    if (waiting_for_message) {
		timedout_request_writeid = waiting_message_id;
	    }
	}
	break;

    case REMOTE_CMS_WRITE_REQUEST_TYPE:
    case REMOTE_CMS_CHECK_IF_READ_REQUEST_TYPE:
    case REMOTE_CMS_GET_MSG_COUNT_REQUEST_TYPE:
    case REMOTE_CMS_GET_QUEUE_LENGTH_REQUEST_TYPE:
    case REMOTE_CMS_GET_SPACE_AVAILABLE_REQUEST_TYPE:
	if (timedout_request == REMOTE_CMS_WRITE_REQUEST_TYPE &&
	    (min_compatible_version > 2.58 || min_compatible_version < 1e-6 ||
		confirm_write)) {
	    break;
	}
	if (recvn(socket_fd, temp_buffer, 12, 0, timeout, &recvd_bytes) < 0) {
	    if (recvn_timedout) {
		consecutive_timeouts++;
		if (consecutive_timeouts > max_consecutive_timeouts &&
		    max_consecutive_timeouts > 0) {
		    rcs_print_error
			("CMS: %d consecutive timeouts have occurred. -- Stop trying.\n",
			consecutive_timeouts);
		    reconnect_needed = 1;
		    fatal_error_occurred = 1;
		}
		reconnect_needed = 1;
		return (status = CMS_TIMED_OUT);
	    } else {
		fatal_error_occurred = 1;
		reconnect_needed = 1;
		return (status = CMS_MISC_ERROR);
	    }
	}
	recvd_bytes = 0;
	returned_serial_number =
	    (CMS_STATUS) getbe32(temp_buffer);
	rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	    "TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	    socket_fd, returned_serial_number, buffer_number);
	if (returned_serial_number != serial_number) {
	    rcs_print_error
		("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
		returned_serial_number, serial_number);
	    reconnect_needed = 1;
	    if (subscription_type == CMS_NO_SUBSCRIPTION) {
		return (status = CMS_MISC_ERROR);
	    }
	}
	break;

    case REMOTE_CMS_CLEAR_REQUEST_TYPE:
	if (recvn(socket_fd, temp_buffer, 4, 0, timeout, &recvd_bytes) < 0) {
	    if (recvn_timedout) {
		consecutive_timeouts++;
		reconnect_needed = 1;
		if (consecutive_timeouts > max_consecutive_timeouts &&
		    max_consecutive_timeouts > 0) {
		    rcs_print_error
			("CMS: %d consecutive timeouts have occurred. -- Stop trying.\n",
			consecutive_timeouts);
		    fatal_error_occurred = 1;
		}
		return (status = CMS_TIMED_OUT);
	    } else {
		reconnect_needed = 1;
		fatal_error_occurred = 1;
		return (status = CMS_MISC_ERROR);
	    }
	}
	recvd_bytes = 0;
	returned_serial_number =
	    (CMS_STATUS) getbe32(temp_buffer);
	rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	    "TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	    socket_fd, returned_serial_number, buffer_number);
	if (returned_serial_number != serial_number) {
	    rcs_print_error
		("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
		returned_serial_number, serial_number);
	    reconnect_needed = 1;
	    if (subscription_type == CMS_NO_SUBSCRIPTION) {
		return (status = CMS_MISC_ERROR);
	    }
	}
	break;

    case NO_REMOTE_CMS_REQUEST:
    default:
	break;
    }
    if (bytes_to_throw_away > 0) {
	if (recvn
	    (socket_fd, encoded_data, bytes_to_throw_away, 0, timeout,
		&recvd_bytes) < 0) {
	    if (recvn_timedout) {
		consecutive_timeouts++;
		if (consecutive_timeouts > max_consecutive_timeouts &&
		    max_consecutive_timeouts > 0) {
		    rcs_print_error
			("CMS: %d consecutive timeouts have occurred. -- Stop trying.\n",
			consecutive_timeouts);
		    fatal_error_occurred = 1;
		    reconnect_needed = 1;
		}
		return (status = CMS_TIMED_OUT);
	    } else {
		recvd_bytes = 0;
		fatal_error_occurred = 1;
		reconnect_needed = 1;
		return (status = CMS_MISC_ERROR);
	    }
	}
	recvd_bytes = 0;
    }
    bytes_to_throw_away = 0;
    timedout_request = NO_REMOTE_CMS_REQUEST;
    consecutive_timeouts = 0;
    waiting_for_message = 0;
    waiting_message_size = 0;
    waiting_message_id = 0;
    recvd_bytes = 0;
    return status;
}

CMS_STATUS TCPMEM::read()
{
    long message_size, id;
    REMOTE_CMS_REQUEST_TYPE last_timedout_request;

    /* Produce error message if process does not have permission to read. */
    if (!read_permission_flag) {
	rcs_print_error("CMS: %s was not configured to read %s\n",
	    ProcessName, BufferName);
	return (status = CMS_PERMISSIONS_ERROR);
    }

    if (reconnect_needed && autoreconnect) {
	reconnect();
    }

    if (reconnect_needed) {
	return (status = CMS_MISC_ERROR);
    }
    disable_sigpipe();

    if (subscription_type != CMS_NO_SUBSCRIPTION) {
	set_socket_fds(read_socket_fd);
	timedout_request = REMOTE_CMS_READ_REQUEST_TYPE;
	if (subscription_count < 1) {
	    serial_number++;
	}
	handle_old_replies();
	check_id(timedout_request_writeid);
	if (status == CMS_READ_OK) {
	    serial_number++;
	}
	subscription_count++;
	reenable_sigpipe();
	return status;
    }

    if (timedout_request == NO_REMOTE_CMS_REQUEST) {
	set_socket_fds(read_socket_fd);
    }
    if (fatal_error_occurred) {
	if (status >= 0) {
	    status = CMS_MISC_ERROR;
	}
	reenable_sigpipe();
	return (status);
    }
    if (socket_fd <= 0) {
	rcs_print_error("TCPMEM::read: Invalid socket descriptor. (%d)\n",
	    socket_fd);
	fatal_error_occurred = 1;
	reconnect_needed = 1;
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    last_timedout_request = timedout_request;
    if (((int) handle_old_replies()) < 0) {
	reenable_sigpipe();
	return status;
    }
    if (polling && last_timedout_request == REMOTE_CMS_READ_REQUEST_TYPE) {
	check_id(timedout_request_writeid);
	reenable_sigpipe();
	return status;
    }
    set_socket_fds(read_socket_fd);

    putbe32(temp_buffer, (uint32_t) serial_number);
    putbe32(temp_buffer + 4, REMOTE_CMS_READ_REQUEST_TYPE);
    putbe32(temp_buffer + 8, (uint32_t) buffer_number);
    putbe32(temp_buffer + 12, CMS_READ_ACCESS);
    putbe32(temp_buffer + 16, in_buffer_id);

    int send_header_size = 20;
    if (total_subdivisions > 1) {
	*((uint32_t *) temp_buffer + 5) = htonl((uint32_t) current_subdivision);
	send_header_size = 24;
    }
    if (sendn(socket_fd, temp_buffer, send_header_size, 0, timeout) < 0) {
	rcs_print_error("TCPMEM: Can't send READ request to server.\n");
	reconnect_needed = 1;
	fatal_error_occurred = 1;
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    serial_number++;
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM sending request: fd = %d, serial_number=%ld, request_type=%d, buffer_number=%ld\n",
	socket_fd, serial_number,
	ntohl(*((uint32_t *) temp_buffer + 1)), buffer_number);

    if (recvn(socket_fd, temp_buffer, 20, 0, timeout, &recvd_bytes) < 20) {
	if (recvn_timedout) {
	    timedout_request = REMOTE_CMS_READ_REQUEST_TYPE;
	    if (polling) {
		return (status = CMS_READ_OLD);
	    } else {
		consecutive_timeouts = 1;
		reenable_sigpipe();
		return (status = CMS_TIMED_OUT);
	    }
	} else {
	    recvd_bytes = 0;
	    reconnect_needed = 1;
	    fatal_error_occurred = 1;
	    reenable_sigpipe();
	    return (status = CMS_MISC_ERROR);
	}
    }
    recvd_bytes = 0;
    returned_serial_number = (CMS_STATUS) getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	socket_fd, returned_serial_number, buffer_number);

    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
	    returned_serial_number, serial_number);
	reconnect_needed = 1;
	if (subscription_type == CMS_NO_SUBSCRIPTION) {
	    fatal_error_occurred = 1;
	    reenable_sigpipe();
	    return (status = CMS_MISC_ERROR);
	}
    }
    status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
    message_size = ntohl(*((uint32_t *) temp_buffer + 2));
    id = ntohl(*((uint32_t *) temp_buffer + 3));
    header.was_read = ntohl(*((uint32_t *) temp_buffer + 4));
    if (message_size > max_encoded_message_size) {
	rcs_print_error("Received message is too big. (%ld > %ld)\n",
	    message_size, max_encoded_message_size);
	fatal_error_occurred = 1;
	reconnect_needed = 1;
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    if (message_size > 0) {
	if (recvn
	    (socket_fd, encoded_data, message_size, 0, timeout,
		&recvd_bytes) < 0) {
	    if (recvn_timedout) {
		if (!waiting_for_message) {
		    waiting_message_id = id;
		    waiting_message_size = message_size;
		}
		waiting_for_message = 1;
		timedout_request = REMOTE_CMS_READ_REQUEST_TYPE;
		if (polling) {
		    reenable_sigpipe();
		    return (status = CMS_READ_OLD);
		} else {
		    reenable_sigpipe();
		    return (status = CMS_TIMED_OUT);
		}
	    } else {
		recvd_bytes = 0;
		fatal_error_occurred = 1;
		reconnect_needed = 1;
		reenable_sigpipe();
		return (status = CMS_MISC_ERROR);
	    }
	}
    }
    recvd_bytes = 0;
    check_id(id);
    reenable_sigpipe();
    return (status);
}

CMS_STATUS TCPMEM::blocking_read(double _blocking_timeout)
{
    blocking_timeout = _blocking_timeout;
    long message_size, id;
    REMOTE_CMS_REQUEST_TYPE last_timedout_request;
    long timeout_millis;
    int orig_print_recvn_timeout_errors = print_recvn_timeout_errors;
    print_recvn_timeout_errors = 0;

/* Produce error message if process does not have permission to read. */
    if (!read_permission_flag) {
	rcs_print_error("CMS: %s was not configured to read %s\n",
	    ProcessName, BufferName);
	return (status = CMS_PERMISSIONS_ERROR);
    }

    if (blocking_timeout < 0) {
	timeout_millis = -1;
    } else {
	timeout_millis = (uint32_t) (blocking_timeout * 1000.0);
    }

    if (reconnect_needed && autoreconnect) {
	reconnect();
    }

    if (reconnect_needed) {
	print_recvn_timeout_errors = orig_print_recvn_timeout_errors;
	return (status = CMS_MISC_ERROR);
    }
    disable_sigpipe();
    double orig_timeout = timeout;

    if (subscription_type != CMS_NO_SUBSCRIPTION) {
	if (blocking_timeout < -1e-6 || blocking_timeout > 1e-6) {
	    make_tcp_socket_blocking(read_socket_fd);
	    timeout = blocking_timeout;
	}
	set_socket_fds(read_socket_fd);
	if (subscription_count < 1) {
	    serial_number++;
	}
	timedout_request = REMOTE_CMS_READ_REQUEST_TYPE;
	handle_old_replies();
	check_id(timedout_request_writeid);
	if (status == CMS_READ_OK) {
	    serial_number++;
	}
	subscription_count++;
	reenable_sigpipe();
	if (blocking_timeout < -1e-6 || blocking_timeout > 1e-6) {
	    make_tcp_socket_nonblocking(read_socket_fd);
	    timeout = orig_timeout;
	}
	print_recvn_timeout_errors = orig_print_recvn_timeout_errors;
	return status;
    }

    if (timedout_request == NO_REMOTE_CMS_REQUEST) {
	set_socket_fds(read_socket_fd);
    }
    if (fatal_error_occurred) {
	if (status >= 0) {
	    status = CMS_MISC_ERROR;
	}
	reenable_sigpipe();
	print_recvn_timeout_errors = orig_print_recvn_timeout_errors;
	return (status);
    }
    if (socket_fd <= 0) {
	rcs_print_error("TCPMEM::read: Invalid socket descriptor. (%d)\n",
	    socket_fd);
	fatal_error_occurred = 1;
	reconnect_needed = 1;
	reenable_sigpipe();
	print_recvn_timeout_errors = orig_print_recvn_timeout_errors;
	return (status = CMS_MISC_ERROR);
    }
    last_timedout_request = timedout_request;
    if (((int) handle_old_replies()) < 0) {
	reenable_sigpipe();
	print_recvn_timeout_errors = orig_print_recvn_timeout_errors;
	return status;
    }
    if (polling && last_timedout_request == REMOTE_CMS_READ_REQUEST_TYPE) {
	check_id(timedout_request_writeid);
	reenable_sigpipe();
	print_recvn_timeout_errors = orig_print_recvn_timeout_errors;
	return status;
    }
    set_socket_fds(read_socket_fd);

    putbe32(temp_buffer, (uint32_t) serial_number);
    putbe32(temp_buffer + 4, REMOTE_CMS_BLOCKING_READ_REQUEST_TYPE);
    putbe32(temp_buffer + 8, (uint32_t) buffer_number);
    putbe32(temp_buffer + 12, CMS_READ_ACCESS);
    putbe32(temp_buffer + 16, (uint32_t) in_buffer_id);
    putbe32(temp_buffer + 20, (uint32_t) timeout_millis);

    int send_header_size = 24;
    if (total_subdivisions > 1) {
        putbe32(temp_buffer + 24, (uint32_t) current_subdivision);
	send_header_size = 28;
    }
    if (sendn(socket_fd, temp_buffer, send_header_size, 0, blocking_timeout) <
	0) {
	rcs_print_error
	    ("TCPMEM: Can't send BLOCKING_READ request to server.\n");
	reconnect_needed = 1;
	fatal_error_occurred = 1;
	reenable_sigpipe();
	print_recvn_timeout_errors = orig_print_recvn_timeout_errors;
	return (status = CMS_MISC_ERROR);
    }
    serial_number++;
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM sending request: fd = %d, serial_number=%ld, "
	"request_type=%d, buffer_number=%ld\n",
	socket_fd, serial_number,
	ntohl(*((uint32_t *) temp_buffer + 1)), buffer_number);
    if (recvn(socket_fd, temp_buffer, 20, 0, blocking_timeout, &recvd_bytes) <
	0) {
	print_recvn_timeout_errors = orig_print_recvn_timeout_errors;
	if (recvn_timedout) {
	    timedout_request = REMOTE_CMS_READ_REQUEST_TYPE;
	    if (polling) {
		return (status = CMS_READ_OLD);
	    } else {
		consecutive_timeouts = 1;
		reenable_sigpipe();
		return (status = CMS_TIMED_OUT);
	    }
	} else {
	    recvd_bytes = 0;
	    reconnect_needed = 1;
	    fatal_error_occurred = 1;
	    reenable_sigpipe();
	    return (status = CMS_MISC_ERROR);
	}
    }
    print_recvn_timeout_errors = orig_print_recvn_timeout_errors;
    recvd_bytes = 0;
    returned_serial_number = (CMS_STATUS) getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	socket_fd, returned_serial_number, buffer_number);

    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
	    returned_serial_number, serial_number);
	reconnect_needed = 1;
	if (subscription_type == CMS_NO_SUBSCRIPTION) {
	    fatal_error_occurred = 1;
	    reenable_sigpipe();
	    return (status = CMS_MISC_ERROR);
	}
    }
    status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
    message_size = ntohl(*((uint32_t *) temp_buffer + 2));
    id = ntohl(*((uint32_t *) temp_buffer + 3));
    header.was_read = ntohl(*((uint32_t *) temp_buffer + 4));
    if (message_size > max_encoded_message_size) {
	rcs_print_error("Received message is too big. (%ld > %ld)\n",
	    message_size, max_encoded_message_size);
	fatal_error_occurred = 1;
	reconnect_needed = 1;
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    if (message_size > 0) {
	if (recvn
	    (socket_fd, encoded_data, message_size, 0, blocking_timeout,
		&recvd_bytes) < 0) {
	    if (recvn_timedout) {
		if (!waiting_for_message) {
		    waiting_message_id = id;
		    waiting_message_size = message_size;
		}
		waiting_for_message = 1;
		timedout_request = REMOTE_CMS_READ_REQUEST_TYPE;
		if (polling) {
		    reenable_sigpipe();
		    return (status = CMS_READ_OLD);
		} else {
		    reenable_sigpipe();
		    return (status = CMS_TIMED_OUT);
		}
	    } else {
		recvd_bytes = 0;
		fatal_error_occurred = 1;
		reconnect_needed = 1;
		reenable_sigpipe();
		return (status = CMS_MISC_ERROR);
	    }
	}
    }
    recvd_bytes = 0;
    check_id(id);
    reenable_sigpipe();
    return (status);
}

void TCPMEM::reenable_sigpipe()
{
    if (old_handler != ((void (*)(int)) SIG_ERR)) {
	signal(SIGPIPE, old_handler);
    }
    old_handler = (void (*)(int)) SIG_ERR;
    if (tcpmem_sigpipe_count > sigpipe_count) {
	sigpipe_count = tcpmem_sigpipe_count;
	reconnect_needed = 1;
    }
}

void TCPMEM::disable_sigpipe()
{
    if (!autoreconnect) {
	return;
    }
    old_handler = signal(SIGPIPE, tcpmem_sigpipe_handler);
    if (tcpmem_sigpipe_count > sigpipe_count) {
	sigpipe_count = tcpmem_sigpipe_count;
    }
}

CMS_STATUS TCPMEM::peek()
{
    /* Produce error message if process does not have permission to read. */
    if (!read_permission_flag) {
	rcs_print_error("CMS: %s was not configured to read %s\n",
	    ProcessName, BufferName);
	return (status = CMS_PERMISSIONS_ERROR);
    }

    if (reconnect_needed && autoreconnect) {
	reconnect();
    }

    if (reconnect_needed) {
	return (status = CMS_MISC_ERROR);
    }
    disable_sigpipe();

    long message_size, id;
    REMOTE_CMS_REQUEST_TYPE last_timedout_request;
    if (subscription_type != CMS_NO_SUBSCRIPTION) {
	set_socket_fds(read_socket_fd);
	timedout_request = REMOTE_CMS_READ_REQUEST_TYPE;
	if (subscription_count < 1) {
	    serial_number++;
	}
	handle_old_replies();
	check_id(timedout_request_writeid);
	if (status == CMS_READ_OK) {
	    serial_number++;
	}
	reenable_sigpipe();
	subscription_count++;
	return status;
    }

    if (timedout_request == NO_REMOTE_CMS_REQUEST) {
	set_socket_fds(read_socket_fd);
    }

    if (fatal_error_occurred) {
	if (status >= 0) {
	    status = CMS_MISC_ERROR;
	}
	reenable_sigpipe();
	return (status);
    }
    if (socket_fd <= 0) {
	reconnect_needed = 1;
	rcs_print_error("TCPMEM::read: Invalid socket descriptor. (%d)\n",
	    socket_fd);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    last_timedout_request = timedout_request;
    if (((int) handle_old_replies()) < 0) {
	reenable_sigpipe();
	return status;
    }
    if (polling && last_timedout_request == REMOTE_CMS_READ_REQUEST_TYPE) {
	check_id(timedout_request_writeid);
	reenable_sigpipe();
	return status;
    }
    set_socket_fds(read_socket_fd);

    putbe32(temp_buffer, (uint32_t) serial_number);
    putbe32(temp_buffer + 4, REMOTE_CMS_READ_REQUEST_TYPE);
    putbe32(temp_buffer + 8, (uint32_t) buffer_number);
    putbe32(temp_buffer + 12, CMS_PEEK_ACCESS);
    putbe32(temp_buffer + 16, (uint32_t) in_buffer_id);
    int send_header_size = 20;
    if (total_subdivisions > 1) {
	*((uint32_t *) temp_buffer + 20) = htonl((uint32_t) current_subdivision);
	send_header_size = 24;
    }
    if (sendn(socket_fd, temp_buffer, send_header_size, 0, timeout) < 0) {
	rcs_print_error("TCPMEM: Can't send PEEK request to server.\n");
	reconnect_needed = 1;
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    serial_number++;
    if (recvn(socket_fd, temp_buffer, 20, 0, timeout, &recvd_bytes) < 0) {
	if (recvn_timedout) {
	    timedout_request = REMOTE_CMS_READ_REQUEST_TYPE;
	    if (polling) {
		reenable_sigpipe();
		return (status = CMS_READ_OLD);
	    } else {
		consecutive_timeouts = 1;
		reenable_sigpipe();
		return (status = CMS_TIMED_OUT);
	    }
	} else {
	    recvd_bytes = 0;
	    fatal_error_occurred = 1;
	    reconnect_needed = 1;
	    reenable_sigpipe();
	    return (status = CMS_MISC_ERROR);
	}
    }
    recvd_bytes = 0;
    returned_serial_number = (CMS_STATUS) getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	socket_fd, returned_serial_number, buffer_number);

    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
	    returned_serial_number, serial_number);
	reconnect_needed = 1;
	if (subscription_type == CMS_NO_SUBSCRIPTION) {
	    reenable_sigpipe();
	    return (status = CMS_MISC_ERROR);
	}
    }
    status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
    message_size = ntohl(*((uint32_t *) temp_buffer + 2));
    id = ntohl(*((uint32_t *) temp_buffer + 3));
    header.was_read = ntohl(*((uint32_t *) temp_buffer + 4));
    if (message_size > max_encoded_message_size) {
	reconnect_needed = 1;
	rcs_print_error("Received message is too big. (%ld > %ld)\n",
	    message_size, max_encoded_message_size);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    if (message_size > 0) {
	if (recvn
	    (socket_fd, encoded_data, message_size, 0, timeout,
		&recvd_bytes) < 0) {
	    if (recvn_timedout) {
		if (!waiting_for_message) {
		    waiting_message_id = id;
		    waiting_message_size = message_size;
		}
		waiting_for_message = 1;
		timedout_request = REMOTE_CMS_READ_REQUEST_TYPE;
		if (polling) {
		    reenable_sigpipe();
		    return (status = CMS_READ_OLD);
		} else {
		    reenable_sigpipe();
		    return (status = CMS_TIMED_OUT);
		}
	    } else {
		reconnect_needed = 1;
		recvd_bytes = 0;
		fatal_error_occurred = 1;
		reenable_sigpipe();
		return (status = CMS_MISC_ERROR);
	    }
	}
    }
    recvd_bytes = 0;
    check_id(id);
    reenable_sigpipe();
    return (status);
}

CMS_STATUS TCPMEM::write(void *user_data, int *serial_number_out)
{

    if (!write_permission_flag) {
	rcs_print_error("CMS: %s was not configured to write to %s\n",
	    ProcessName, BufferName);
	return (status = CMS_PERMISSIONS_ERROR);
    }

    if (reconnect_needed && autoreconnect) {
	reconnect();
    }

    if (!force_raw) {
	user_data = encoded_data;
    }

    if (reconnect_needed) {
	return (status = CMS_MISC_ERROR);
    }

    if (fatal_error_occurred) {
	if (status >= 0) {
	    status = CMS_MISC_ERROR;
	}
	return (status);
    }

    disable_sigpipe();

    if (socket_fd <= 0) {
	rcs_print_error("TCPMEM::write: Invalid socket descriptor. (%d)\n",
	    socket_fd);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    if (((int) handle_old_replies()) < 0) {
	reenable_sigpipe();
	return status;
    }
    set_socket_fds(write_socket_fd);

    putbe32(temp_buffer, serial_number);
    putbe32(temp_buffer + 4, REMOTE_CMS_WRITE_REQUEST_TYPE);
    putbe32(temp_buffer + 8, (uint32_t) buffer_number);
    putbe32(temp_buffer + 12, CMS_WRITE_ACCESS);
    putbe32(temp_buffer + 16, (uint32_t) header.in_buffer_size);
    int send_header_size = 20;
    if (total_subdivisions > 1) {
        putbe32(temp_buffer + 20,(uint32_t) current_subdivision);
	send_header_size = 24;
    }
    if (header.in_buffer_size < 0x2000 - 20 && header.in_buffer_size > 0) {
	memcpy(temp_buffer + send_header_size, user_data,
	    header.in_buffer_size);
	if (sendn
	    (socket_fd, temp_buffer, header.in_buffer_size + send_header_size,
		0, timeout) < 0) {
	    rcs_print_error
		("TCPMEM: Failed to send message of size %ld + header of size %d  to the server.\n",
		header.in_buffer_size, send_header_size);
	    reconnect_needed = 1;
	    reenable_sigpipe();
	    return (status = CMS_MISC_ERROR);
	}
    } else {
	if (sendn(socket_fd, temp_buffer, send_header_size, 0, timeout) < 0) {
	    rcs_print_error("TCPMEM: Failed to send header to server.\n");
	    reconnect_needed = 1;
	    reenable_sigpipe();
	    return (status = CMS_MISC_ERROR);
	}
	if (header.in_buffer_size > 0) {
	    if (sendn(socket_fd, user_data, header.in_buffer_size, 0, timeout)
		< 0) {
		reconnect_needed = 1;
		reenable_sigpipe();
		return (status = CMS_MISC_ERROR);
	    }
	}
    }
    serial_number++;
    if ((min_compatible_version < 2.58 && min_compatible_version > 1e-6)
	|| confirm_write) {
	if (recvn(socket_fd, temp_buffer, 12, 0, timeout, &recvd_bytes) < 0) {
	    if (recvn_timedout) {
		timedout_request = REMOTE_CMS_WRITE_REQUEST_TYPE;
		consecutive_timeouts = 1;
		reenable_sigpipe();
		return (status = CMS_TIMED_OUT);
	    } else {
		recvd_bytes = 0;
		reconnect_needed = 1;
		fatal_error_occurred = 1;
		reenable_sigpipe();
		return (status = CMS_MISC_ERROR);
	    }
	}
	recvd_bytes = 0;
	returned_serial_number =
	    (CMS_STATUS) getbe32(temp_buffer);
	if(serial_number_out) *serial_number_out = returned_serial_number;
	rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	    "TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	    socket_fd, returned_serial_number, buffer_number);

	status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
	header.was_read = ntohl(*((uint32_t *) temp_buffer + 2));
	header.write_id = returned_serial_number;
    } else {
	header.was_read = 0;
	status = CMS_WRITE_OK;
	returned_serial_number = serial_number;
    }
    reenable_sigpipe();
    return (status);
}

CMS_STATUS TCPMEM::write_if_read(void *user_data, int *serial_number_out)
{

    if (!write_permission_flag) {
	rcs_print_error("CMS: %s was not configured to write to %s\n",
	    ProcessName, BufferName);
	return (status = CMS_PERMISSIONS_ERROR);
    }

    if (reconnect_needed && autoreconnect) {
	reconnect();
    }
    if (!force_raw) {
	user_data = encoded_data;
    }

    if (reconnect_needed) {
	return (status = CMS_MISC_ERROR);
    }

    if (fatal_error_occurred) {
	if (status >= 0) {
	    status = CMS_MISC_ERROR;
	}
	return (status);
    }
    disable_sigpipe();

    if (socket_fd <= 0) {
	rcs_print_error("TCPMEM::write: Invalid socket descriptor. (%d)\n",
	    socket_fd);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    if (((int) handle_old_replies()) < 0) {
	reenable_sigpipe();
	return status;
    }

    set_socket_fds(write_socket_fd);

    putbe32( temp_buffer, (uint32_t) serial_number);
    putbe32( temp_buffer + 4, REMOTE_CMS_WRITE_REQUEST_TYPE);
    putbe32( temp_buffer + 8, (uint32_t) buffer_number);
    putbe32( temp_buffer + 12, CMS_WRITE_IF_READ_ACCESS);
    putbe32( temp_buffer + 16, (uint32_t) header.in_buffer_size);
    int send_header_size = 20;
    if (total_subdivisions > 1) {
        putbe32( temp_buffer + 20, (uint32_t) current_subdivision);
	send_header_size = 24;
    }
    if (header.in_buffer_size < 0x2000 - 20 && header.in_buffer_size > 0) {
	memcpy(temp_buffer + 20, user_data, header.in_buffer_size);
	if (sendn
	    (socket_fd, temp_buffer, header.in_buffer_size + send_header_size,
		0, timeout) < 0) {
	    reconnect_needed = 1;
	    reenable_sigpipe();
	    return (status = CMS_MISC_ERROR);
	}
    } else {
	if (sendn(socket_fd, temp_buffer, send_header_size, 0, timeout) < 0) {
	    reconnect_needed = 1;
	    reenable_sigpipe();
	    return (status = CMS_MISC_ERROR);
	}
	if (header.in_buffer_size > 0) {
	    if (sendn(socket_fd, user_data, header.in_buffer_size, 0, timeout)
		< 0) {
		reconnect_needed = 1;
		reenable_sigpipe();
		return (status = CMS_MISC_ERROR);
	    }
	}
    }
    serial_number++;
    if ((min_compatible_version < 2.58 && min_compatible_version > 1e-6) ||
	confirm_write) {
	if (recvn(socket_fd, temp_buffer, 12, 0, timeout, &recvd_bytes) < 0) {
	    if (recvn_timedout) {
		timedout_request = REMOTE_CMS_WRITE_REQUEST_TYPE;
		consecutive_timeouts = 1;
		reenable_sigpipe();
		return (status = CMS_TIMED_OUT);
	    } else {
		recvd_bytes = 0;
		fatal_error_occurred = 1;
		reconnect_needed = 1;
		reenable_sigpipe();
		return (status = CMS_MISC_ERROR);
	    }
	}
	recvd_bytes = 0;
	returned_serial_number =
	    (CMS_STATUS) getbe32(temp_buffer);
	if(serial_number_out) *serial_number_out = returned_serial_number;
	rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	    "TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	    socket_fd, returned_serial_number, buffer_number);
	status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
	header.was_read = ntohl(*((uint32_t *) temp_buffer + 2));
    } else {
	header.was_read = 0;
	status = CMS_WRITE_OK;
	returned_serial_number = 0;
    }
    reenable_sigpipe();
    return (status);
}

int TCPMEM::check_if_read()
{
    if (reconnect_needed && autoreconnect) {
	reconnect();
    }

    if (reconnect_needed) {
	return (status = CMS_MISC_ERROR);
    }

    if (fatal_error_occurred) {
	if (status >= 0) {
	    status = CMS_MISC_ERROR;
	}
	return (status);
    }

    disable_sigpipe();

    if (socket_fd <= 0) {
	rcs_print_error
	    ("TCPMEM::check_if_read: Invalid socket descriptor. (%d)\n",
	    socket_fd);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    if (((int) handle_old_replies()) < 0) {
	reenable_sigpipe();
	return 0;
    }

    set_socket_fds(write_socket_fd);

    putbe32(temp_buffer, (uint32_t) serial_number);
    putbe32(temp_buffer + 4, REMOTE_CMS_CHECK_IF_READ_REQUEST_TYPE);
    putbe32(temp_buffer + 8, (uint32_t) buffer_number);
    int send_header_size = 20;
    if (total_subdivisions > 1) {
        putbe32(temp_buffer + 12, (uint32_t) current_subdivision);
    }
    if (sendn(socket_fd, temp_buffer, send_header_size, 0, timeout) < 0) {
	status = CMS_MISC_ERROR;
	reconnect_needed = 1;
	reenable_sigpipe();
	return (0);
    }
    serial_number++;
    if (recvn(socket_fd, temp_buffer, 12, 0, timeout, &recvd_bytes) < 0) {
	if (recvn_timedout) {
	    timedout_request = REMOTE_CMS_CHECK_IF_READ_REQUEST_TYPE;
	    consecutive_timeouts = 1;
	    status = CMS_TIMED_OUT;
	    reenable_sigpipe();
	    return 0;
	} else {
	    recvd_bytes = 0;
	    fatal_error_occurred = 1;
	    status = CMS_MISC_ERROR;
	    reenable_sigpipe();
	    return 0;
	}
    }
    recvd_bytes = 0;
    returned_serial_number = (CMS_STATUS) getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	socket_fd, returned_serial_number, buffer_number);
    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
	    returned_serial_number, serial_number);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
    header.was_read = ntohl(*((uint32_t *) temp_buffer + 2));
    reenable_sigpipe();
    return (header.was_read);
}

int TCPMEM::get_queue_length()
{
    if (reconnect_needed && autoreconnect) {
	reconnect();
    }

    if (reconnect_needed) {
	return (status = CMS_MISC_ERROR);
    }

    if (fatal_error_occurred) {
	if (status >= 0) {
	    status = CMS_MISC_ERROR;
	}
	return (status);
    }

    disable_sigpipe();

    if (socket_fd <= 0) {
	rcs_print_error
	    ("TCPMEM::check_if_read: Invalid socket descriptor. (%d)\n",
	    socket_fd);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    if (((int) handle_old_replies()) < 0) {
	reenable_sigpipe();
	return 0;
    }

    set_socket_fds(write_socket_fd);

    putbe32(temp_buffer, (uint32_t) serial_number);
    putbe32(temp_buffer + 4, REMOTE_CMS_GET_QUEUE_LENGTH_REQUEST_TYPE);
    putbe32(temp_buffer + 8, (uint32_t) buffer_number);
    int send_header_size = 20;
    if (total_subdivisions > 1) {
        putbe32(temp_buffer + 16, (uint32_t) current_subdivision);
    }
    if (sendn(socket_fd, temp_buffer, send_header_size, 0, timeout) < 0) {
	status = CMS_MISC_ERROR;
	reconnect_needed = 1;
	reenable_sigpipe();
	return (0);
    }
    serial_number++;
    if (recvn(socket_fd, temp_buffer, 12, 0, timeout, &recvd_bytes) < 0) {
	if (recvn_timedout) {
	    timedout_request = REMOTE_CMS_GET_QUEUE_LENGTH_REQUEST_TYPE;
	    consecutive_timeouts = 1;
	    status = CMS_TIMED_OUT;
	    reenable_sigpipe();
	    return 0;
	} else {
	    recvd_bytes = 0;
	    fatal_error_occurred = 1;
	    status = CMS_MISC_ERROR;
	    reenable_sigpipe();
	    return 0;
	}
    }
    recvd_bytes = 0;
    returned_serial_number = (CMS_STATUS) getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	socket_fd, returned_serial_number, buffer_number);
    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
	    returned_serial_number, serial_number);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
    queuing_header.queue_length = ntohl(*((uint32_t *) temp_buffer + 2));
    reenable_sigpipe();
    return (queuing_header.queue_length);
}

int TCPMEM::get_msg_count()
{
    if (reconnect_needed && autoreconnect) {
	reconnect();
    }

    if (reconnect_needed) {
	return (status = CMS_MISC_ERROR);
    }

    if (fatal_error_occurred) {
	if (status >= 0) {
	    status = CMS_MISC_ERROR;
	}
	return (status);
    }

    disable_sigpipe();

    if (socket_fd <= 0) {
	rcs_print_error
	    ("TCPMEM::check_if_read: Invalid socket descriptor. (%d)\n",
	    socket_fd);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    if (((int) handle_old_replies()) < 0) {
	reenable_sigpipe();
	return 0;
    }

    set_socket_fds(write_socket_fd);

    putbe32(temp_buffer, (uint32_t) serial_number);
    putbe32(temp_buffer + 4, REMOTE_CMS_GET_MSG_COUNT_REQUEST_TYPE);
    putbe32(temp_buffer + 8, (uint32_t) buffer_number);
    int send_header_size = 20;
    if (total_subdivisions > 1) {
        putbe32(temp_buffer + 12, (uint32_t) current_subdivision);
    }
    if (sendn(socket_fd, temp_buffer, send_header_size, 0, timeout) < 0) {
	status = CMS_MISC_ERROR;
	reconnect_needed = 1;
	reenable_sigpipe();
	return (0);
    }
    serial_number++;
    if (recvn(socket_fd, temp_buffer, 12, 0, timeout, &recvd_bytes) < 0) {
	if (recvn_timedout) {
	    timedout_request = REMOTE_CMS_GET_MSG_COUNT_REQUEST_TYPE;
	    consecutive_timeouts = 1;
	    status = CMS_TIMED_OUT;
	    reenable_sigpipe();
	    return 0;
	} else {
	    recvd_bytes = 0;
	    fatal_error_occurred = 1;
	    status = CMS_MISC_ERROR;
	    reenable_sigpipe();
	    return 0;
	}
    }
    recvd_bytes = 0;
    returned_serial_number = (CMS_STATUS) getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	socket_fd, returned_serial_number, buffer_number);
    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
	    returned_serial_number, serial_number);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
    header.write_id = ntohl(*((uint32_t *) temp_buffer + 2));
    reenable_sigpipe();
    return (header.write_id);
}

int TCPMEM::get_space_available()
{
    if (reconnect_needed && autoreconnect) {
	reconnect();
    }

    if (reconnect_needed) {
	return (status = CMS_MISC_ERROR);
    }

    if (fatal_error_occurred) {
	if (status >= 0) {
	    status = CMS_MISC_ERROR;
	}
	return (status);
    }

    disable_sigpipe();

    if (socket_fd <= 0) {
	rcs_print_error
	    ("TCPMEM::check_if_read: Invalid socket descriptor. (%d)\n",
	    socket_fd);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    if (((int) handle_old_replies()) < 0) {
	reenable_sigpipe();
	return 0;
    }

    set_socket_fds(write_socket_fd);

    putbe32(temp_buffer, (uint32_t) serial_number);
    putbe32(temp_buffer + 4, REMOTE_CMS_GET_SPACE_AVAILABLE_REQUEST_TYPE);
    putbe32(temp_buffer + 8,buffer_number);
    int send_header_size = 20;
    if (total_subdivisions > 1) {
        putbe32(temp_buffer + 12, current_subdivision);
    }
    if (sendn(socket_fd, temp_buffer, send_header_size, 0, timeout) < 0) {
	status = CMS_MISC_ERROR;
	reconnect_needed = 1;
	reenable_sigpipe();
	return (0);
    }
    serial_number++;
    if (recvn(socket_fd, temp_buffer, 12, 0, timeout, &recvd_bytes) < 0) {
	if (recvn_timedout) {
	    timedout_request = REMOTE_CMS_GET_SPACE_AVAILABLE_REQUEST_TYPE;
	    consecutive_timeouts = 1;
	    status = CMS_TIMED_OUT;
	    reenable_sigpipe();
	    return 0;
	} else {
	    recvd_bytes = 0;
	    fatal_error_occurred = 1;
	    status = CMS_MISC_ERROR;
	    reenable_sigpipe();
	    return 0;
	}
    }
    recvd_bytes = 0;
    returned_serial_number = (CMS_STATUS) getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	socket_fd, returned_serial_number, buffer_number);
    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
	    returned_serial_number, serial_number);
	reenable_sigpipe();
	return (status = CMS_MISC_ERROR);
    }
    status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
    free_space = ntohl(*((uint32_t *) temp_buffer + 2));
    reenable_sigpipe();
    return (free_space);
}

CMS_STATUS TCPMEM::clear()
{
    if (reconnect_needed && autoreconnect) {
	reconnect();
    }

    if (reconnect_needed) {
	return (status = CMS_MISC_ERROR);
    }

    if (fatal_error_occurred) {
	if (status >= 0) {
	    status = CMS_MISC_ERROR;
	}
	return (status);
    }
    if (socket_fd <= 0) {
	rcs_print_error("TCPMEM::clear: Invalid socket descriptor. (%d)\n",
	    socket_fd);
	reconnect_needed = 1;
	return (status = CMS_MISC_ERROR);
    }
    if (((int) handle_old_replies()) < 0) {
	return status;
    }

    set_socket_fds(write_socket_fd);

    putbe32(temp_buffer, (uint32_t) serial_number);
    putbe32(temp_buffer + 4, REMOTE_CMS_CLEAR_REQUEST_TYPE);
    putbe32(temp_buffer + 8, buffer_number);
    putbe32(temp_buffer + 12, current_subdivision);

    if (sendn(socket_fd, temp_buffer, 20, 0, timeout) < 0) {
	reconnect_needed = 1;
	return (status = CMS_MISC_ERROR);
    }
    serial_number++;
    if (recvn(socket_fd, temp_buffer, 8, 0, timeout, &recvd_bytes) < 0) {
	if (recvn_timedout) {
	    timedout_request = REMOTE_CMS_CLEAR_REQUEST_TYPE;
	    consecutive_timeouts = 1;
	    return (status = CMS_TIMED_OUT);
	} else {
	    fatal_error_occurred = 1;
	    reconnect_needed = 1;
	    return (status = CMS_MISC_ERROR);
	}
    }
    returned_serial_number = (CMS_STATUS) getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%ld, buffer_number=%ld\n",
	socket_fd, returned_serial_number, buffer_number);

    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%ld) does not match expected serial number(%ld).\n",
	    returned_serial_number, serial_number);
	reconnect_needed = 1;
	return (status = CMS_MISC_ERROR);
    }
    status = (CMS_STATUS) ntohl(*((uint32_t *) temp_buffer + 1));
    header.was_read = ntohl(*((uint32_t *) temp_buffer + 2));
    return (status);
}
/*! \todo Another #if 0 */
#if 0
int TCPMEM::login(const char *name, const char *passwd)
{
    if (fatal_error_occurred) {
	if (status >= 0) {
	    status = CMS_MISC_ERROR;
	}
	return (status);
    }
    if (socket_fd <= 0) {
	rcs_print_error("TCPMEM::write: Invalid socket descriptor. (%d)\n",
	    socket_fd);
	return (status = CMS_MISC_ERROR);
    }
    int handle_old_reply_ret = 0;

    while (timedout_request != NO_REMOTE_CMS_REQUEST && !handle_old_reply_ret) {
	handle_old_reply_ret = handle_old_replies();
    }
    if (handle_old_reply_ret < 0) {
	return 0;
    }
    set_socket_fds(write_socket_fd);
    *((uint32_t *) temp_buffer) = htonl((uint32_t) serial_number);
    *((uint32_t *) temp_buffer + 1) =
	htonl((uint32_t) REMOTE_CMS_GET_KEYS_REQUEST_TYPE);
    *((uint32_t *) temp_buffer + 2) = htonl((uint32_t) buffer_number);
    if (sendn(socket_fd, temp_buffer, 20, 0, 30.0) < 0) {
	return 0;
    }
    memset(temp_buffer, 0, 20);
    strncpy(((char *) temp_buffer), name, 16);
    if (sendn(socket_fd, temp_buffer, 16, 0, 30.0) < 0) {
	return (status = CMS_MISC_ERROR);
    }
    serial_number++;
    if (recvn(socket_fd, temp_buffer, 20, 0, 30.0, &recvd_bytes) < 0) {
	return 0;
    }
    recvd_bytes = 0;
    returned_serial_number = (CMS_STATUS) getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%d, buffer_number=%d\n",
	socket_fd, returned_serial_number, buffer_number);
    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%d) does not match expected serial number(%d).\n",
	    returned_serial_number, serial_number);
	return (0);
    }
    char *crypt1_ret = crypt(passwd, ((char *) temp_buffer) + 4);
    if (NULL == crypt1_ret) {
	rcs_print_error("TCPMEM::login() crypt function failed.\n");
	return 0;
    }
    char passwd_pass1[16];
    strncpy(passwd_pass1, crypt1_ret, 16);
    char *crypt2_ret = crypt(passwd_pass1, ((char *) temp_buffer) + 12);
    if (NULL == crypt2_ret) {
	rcs_print_error("TCPMEM::login() crypt function failed.\n");
	return (0);
    }
    char passwd_pass2[16];
    strncpy(passwd_pass2, crypt2_ret, 16);

    *((uint32_t *) temp_buffer) = htonl((uint32_t) serial_number);
    *((uint32_t *) temp_buffer + 1) =
	htonl((uint32_t) REMOTE_CMS_LOGIN_REQUEST_TYPE);
    *((uint32_t *) temp_buffer + 2) = htonl((uint32_t) buffer_number);
    if (sendn(socket_fd, temp_buffer, 20, 0, 30.0) < 0) {
	return 0;
    }
    memset(temp_buffer, 0, 20);
    strncpy(((char *) temp_buffer), name, 16);
    if (sendn(socket_fd, temp_buffer, 16, 0, 30.0) < 0) {
	return (status = CMS_MISC_ERROR);
    }
    if (sendn(socket_fd, passwd_pass2, 16, 0, 30.0) < 0) {
	return (status = CMS_MISC_ERROR);
    }
    serial_number++;
    if (recvn(socket_fd, temp_buffer, 8, 0, 30.0, &recvd_bytes) < 0) {
	return 0;
    }
    recvd_bytes = 0;
    returned_serial_number = (CMS_STATUS) getbe32(temp_buffer);
    rcs_print_debug(PRINT_ALL_SOCKET_REQUESTS,
	"TCPMEM received_reply: fd = %d, serial_number=%d, buffer_number=%d\n",
	socket_fd, returned_serial_number, buffer_number);
    if (returned_serial_number != serial_number) {
	rcs_print_error
	    ("TCPMEM: Returned serial number(%d) does not match expected serial number(%d).\n",
	    returned_serial_number, serial_number);
	return (status = CMS_MISC_ERROR);
    }
    int success = ntohl(*((uint32_t *) temp_buffer + 1));
    return (success);
}
#endif

void TCPMEM::set_socket_fds(int new_fd)
{
    if (socket_fd == read_socket_fd) {
	read_serial_number = serial_number;
    }
    if (socket_fd == write_socket_fd) {
	write_serial_number = serial_number;
    }
    socket_fd = new_fd;
    if (socket_fd == read_socket_fd) {
	serial_number = read_serial_number;
    }
    if (socket_fd == write_socket_fd) {
	serial_number = write_serial_number;
    }
}
