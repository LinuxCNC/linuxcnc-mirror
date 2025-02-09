/********************************************************************
* Description: shmem.cc
*   C++ file for the Communication Management System (CMS).
*   Includes member Functions for class SHMEM.
*   Notes: The class SHMEM should be used by procedures accessing a
*   shared memory buffer on the same processor.
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

#include <stdio.h>		/* sscanf() */
#include <stddef.h>		/* size_t */
#include <sys/stat.h>		/* S_IRUSR, etc. */
#include <sys/types.h>		/* key_t */
#include <errno.h>		// errno
#include <string.h>		/* strchr(), memcpy(), memset() */
#include <stdlib.h>		/* strtod */
#include <physmem.hh>           /* PHYSMEM_HANDLE */
#include <math.h>

#ifdef __cplusplus
}
#endif
#include <rtapi_string.h>	/* rtapi_strlcpy */
#include "rcs_print.hh"		/* rcs_print_error() */
#include "cms.hh"		/* class CMS */
#include "shmem.hh"		/* class SHMEM */
#include "shm.hh"		/* class RCS_SHAREDMEM */
//#include "sem.hh"             /* class RCS_SEMAPHORE */
#include "memsem.hh"		/* mem_get_access(), mem_release_access() */
#include "timer.hh"		/* etime(), esleep() */
/* Common Definitions. */
//#include "autokey.h"
/* rw-rw-r-- permissions */
#define MODE (0700)

static inline bool not_zero(double x)
{
	return fabs(x) > 1e-6;
}

/* SHMEM Member Functions. */

/* Constructor for hard coded tests. */
SHMEM::SHMEM(const char * /*n*/, long s, int /*nt*/, key_t k, int m)
  : CMS(s),
    fast_mode(0),
    key(k),
    bsem_key(-1),
    second_read(0),
    shm(NULL),
    sem(NULL),
    master(m),
    sem_delay(0.00001),
    mao{},
    use_os_sem(1),
    use_os_sem_only(1),
    mutex_type(OS_SEM_MUTEX),
    shm_addr_offset(NULL),
    bsem(NULL),
    autokey_table_size(0)
{
    /* open the shared mem buffer and create mutual exclusion semaphore */
    open();
}

/* Constructor for use with cms_config. */
SHMEM::SHMEM(const char *bufline, const char *procline, int set_to_server, int set_to_master)
  : CMS(bufline, procline, set_to_server),
    fast_mode(0),
    bsem_key(-1),
    second_read(0),
    shm(NULL),
    sem(NULL),
    sem_delay(0.00001),
    mao{},
    use_os_sem(1),
    use_os_sem_only(1),
    mutex_type(OS_SEM_MUTEX),
    shm_addr_offset(NULL),
    bsem(NULL),
    autokey_table_size(0)
{
    /* Set pointers to null so only properly opened pointers are closed. */
    char *semdelay_equation;

    if (status < 0) {
	rcs_print_error("SHMEM: status = %d\n", status);
	return;
    }

    /* Save parameters from configuration file. */
    if (sscanf(bufline, "%*s %*s %*s %*s %*s %*s %*s %*s %*s %d", &key) != 1) {
	rcs_print_error("SHMEM: Invalid configuration file format.\n");
	return;
    }

    master = is_local_master;
    if (1 == set_to_master) {
	master = 1;
    } else if (-1 == set_to_master) {
	master = 0;
    }
    if (NULL != (semdelay_equation = strstr(proclineupper, "SEMDELAY="))) {
	sem_delay = strtod(semdelay_equation + 9, (char **) NULL);
    } else if (NULL != (semdelay_equation =
	    strstr(buflineupper, "SEMDELAY="))) {
	sem_delay = strtod(semdelay_equation + 9, (char **) NULL);
    }

    if (NULL != (semdelay_equation = strstr(buflineupper, "BSEM="))) {
	bsem_key = strtol(semdelay_equation + 5, (char **) NULL, 0);
    }

    if (NULL != strstr(buflineupper, "MUTEX=NONE")) {
	mutex_type = NO_MUTEX;
	use_os_sem = 0;
	use_os_sem_only = 0;
    }

    if (NULL != strstr(buflineupper, "MUTEX=OS_SEM")) {
	mutex_type = OS_SEM_MUTEX;
	use_os_sem = 1;
	use_os_sem_only = 1;
    }

    if (NULL != strstr(buflineupper, "MUTEX=NO_INTERRUPTS")) {
	mutex_type = NO_INTERRUPTS_MUTEX;
	use_os_sem = 0;
	use_os_sem_only = 0;
    }

    if (NULL != strstr(buflineupper, "MUTEX=NO_SWITCHING")) {
	mutex_type = NO_SWITCHING_MUTEX;
	use_os_sem = 0;
	use_os_sem_only = 0;
    }

    if (NULL != strstr(buflineupper, "MUTEX=MAO")) {
	mutex_type = MAO_MUTEX;
	use_os_sem = 0;
	use_os_sem_only = 0;
    }

    if (NULL != strstr(buflineupper, "MAO_W_OS_SEM")) {
	mutex_type = MAO_MUTEX_W_OS_SEM;
	use_os_sem = 1;
	use_os_sem_only = 0;
    }

    /* Open the shared memory buffer and create mutual exclusion semaphore. */
    open();
}

SHMEM::~SHMEM()
{
    /* detach from shared memory and semaphores */
    close();
}

/*
  Open the SHMEM buffer
  */
int SHMEM::open()
{
    /* Set pointers to NULL incase error occurs. */
    sem = NULL;
    shm = NULL;
    bsem = NULL;
    shm_addr_offset = NULL;
    second_read = 0;
    autokey_table_size = 0;
/*! \todo Another #if 0 */
#if 0				// PC Do we need to use autokey ?
    if (use_autokey_for_connection_number) {
	autokey_table_size = sizeof(AUTOKEY_TABLE_ENTRY) * total_connections;
    }
#endif
    /* set up the shared memory address and semaphore, in given state */
    if (master) {
	shm = new RCS_SHAREDMEM(key, size, RCS_SHAREDMEM_CREATE, (int) MODE);
	if (shm->addr == NULL) {
	    switch (shm->create_errno) {
	    case EACCES:
		status = CMS_PERMISSIONS_ERROR;
		break;

	    case EEXIST:
		status = CMS_RESOURCE_CONFLICT_ERROR;
		break;

	    case ENOMEM:
	    case ENOSPC:
		status = CMS_CREATE_ERROR;
		break;

	    default:
		status = CMS_MISC_ERROR;
	    }
	    delete shm;
	    shm = NULL;
	    return -1;
	}
	if (use_os_sem) {
	    sem =
		new RCS_SEMAPHORE(key, RCS_SEMAPHORE_CREATE, timeout,
		(int) MODE, (use_os_sem_only != 0));
	    if (NULL == sem) {
		rcs_print_error("CMS: couldn't create RCS_SEMAPHORE.\n");
		rcs_print_error(" Possibly out of memory?\n");
		status = CMS_CREATE_ERROR;
		return -1;
	    }
	    if (!sem->valid()) {
		rcs_print_error("CMS: RCS_SEMAPHORE is invalid.\n");
		status = CMS_MISC_ERROR;
		return -1;
	    }
	}
	if (bsem_key > 0) {
	    bsem = new RCS_SEMAPHORE(bsem_key, RCS_SEMAPHORE_CREATE,
		timeout, (int) MODE, 0);
	    if (NULL == bsem) {
		rcs_print_error("CMS: couldn't create RCS_SEMAPHORE.\n");
		rcs_print_error(" Possibly out of memory?\n");
		status = CMS_CREATE_ERROR;
		return -1;
	    }
	    if (!bsem->valid()) {
		rcs_print_error("CMS: RCS_SEMAPHORE is invalid.\n");
		status = CMS_MISC_ERROR;
		return -1;
	    }
	}
	in_buffer_id = 0;
    } else {
	shm = new RCS_SHAREDMEM(key, size, RCS_SHAREDMEM_NOCREATE);
	if (NULL == shm) {
	    rcs_print_error
		("CMS: couldn't create RCS_SHAREDMEM(%d(0x%X), %ld(0x%lX), RCS_SHAREDMEM_NOCREATE).\n",
		key, key, size, size);
	    status = CMS_CREATE_ERROR;
	    return -1;
	}
	if (shm->addr == NULL) {
	    switch (shm->create_errno) {
	    case EACCES:
		status = CMS_PERMISSIONS_ERROR;
		break;

	    case EEXIST:
		status = CMS_RESOURCE_CONFLICT_ERROR;
		break;

	    case ENOENT:
		status = CMS_NO_MASTER_ERROR;
		break;

	    case ENOMEM:
	    case ENOSPC:
		status = CMS_CREATE_ERROR;
		break;

	    default:
		status = CMS_MISC_ERROR;
	    }
	    delete shm;
	    shm = NULL;
	    return -1;
	}
	if (use_os_sem) {
	    sem = new RCS_SEMAPHORE(key, RCS_SEMAPHORE_NOCREATE, timeout);
	    if (NULL == sem) {
		rcs_print_error("CMS: couldn't create RCS_SEMAPHORE.\n");
		rcs_print_error(" Possibly out of memory?\n");
		status = CMS_CREATE_ERROR;
		return -1;
	    }
	    if (!sem->valid()) {
		rcs_print_error("CMS: RCS_SEMAPHORE is invalid.\n");
		status = CMS_MISC_ERROR;
		return -1;
	    }
	}
	if (bsem_key > 0) {
	    bsem =
		new RCS_SEMAPHORE(bsem_key, RCS_SEMAPHORE_NOCREATE, timeout);
	    if (NULL == bsem) {
		rcs_print_error("CMS: couldn't create RCS_SEMAPHORE.\n");
		rcs_print_error(" Possibly out of memory?\n");
		status = CMS_CREATE_ERROR;
		return -1;
	    }
	    if (!bsem->valid()) {
		rcs_print_error("CMS: RCS_SEMAPHORE is invalid.\n");
		status = CMS_MISC_ERROR;
		return -1;
	    }
	}
    }

    if (min_compatible_version < 3.44 && min_compatible_version > 0) {
	total_subdivisions = 1;
    }

    if (min_compatible_version > 2.57 || min_compatible_version <= 0) {
	if (!shm->created) {
	    char *cptr = (char *) shm->addr;
	    cptr[31] = 0;
	    if (strncmp(cptr, BufferName, 31)) {
		rcs_print_error
		    ("Shared memory buffers %s and %s may conflict. (key=%d(0x%X))\n",
		    BufferName, cptr, key, key);
		rtapi_strlcpy(cptr, BufferName, 32);
	    }
	}
	if (master) {
/*! \todo Another #if 0 */
#if 0				// PC Do we need to use autokey ?
	    if (use_autokey_for_connection_number) {
		void *autokey_table_end =
		    (void *) (((char *) shm->addr) + 32 + autokey_table_size);
		memset(autokey_table_end, 0, size - 32 - autokey_table_size);
	    }
#endif
	    rtapi_strlcpy((char *) shm->addr, BufferName, 32);
	}
/*! \todo Another #if 0 */
#if 0				// PC Do we need to use autokey ?
	if (use_autokey_for_connection_number) {
	    void *autokey_table = (void *) (((char *) shm->addr) + 32);
	    connection_number =
		autokey_getkey(autokey_table, total_connections, ProcessName);
	    shm_addr_offset =
		(void *) ((char *) (shm->addr) + 32 + autokey_table_size);
	    max_message_size -= (32 + autokey_table_size);	/* size of
								   cms buffer 
								   available
								   for user */
	} else {
#endif
	    shm_addr_offset = (void *) ((char *) (shm->addr) + 32);
	    max_message_size -= 32;	/* size of cms buffer available for
					   user */
/*! \todo Another #if 0 */
#if 0				// PC Do we need to use autokey ?
	}
#endif
	/* messages = size - CMS Header space */
	if (enc_max_size <= 0 || enc_max_size > size) {
	    if (neutral) {
		max_encoded_message_size -= 32;
	    } else {
		max_encoded_message_size -=
		    (cms_encoded_data_explosion_factor * 32);
	    }
	}
	/* Maximum size of message after being encoded. */
	guaranteed_message_space -= 32;	/* Largest size message before being
					   encoded that can be guaranteed to
					   fit after xdr. */
	size -= 32;
	size_without_diagnostics -= 32;
	subdiv_size =
	    (size_without_diagnostics -
	    total_connections) / total_subdivisions;
	subdiv_size -= (subdiv_size % 4);
    } else {
	if (master) {
	    memset(shm->addr, 0, size);
	}
	shm_addr_offset = shm->addr;
    }
    skip_area = 32 + total_connections + autokey_table_size;
    mao.data = shm_addr_offset;
    mao.timeout = timeout;
    mao.total_connections = total_connections;
    mao.sem_delay = sem_delay;
    mao.connection_number = connection_number;
    mao.split_buffer = split_buffer;
    mao.read_only = 0;
    mao.sem = sem;

    fast_mode = !queuing_enabled && !split_buffer && !neutral &&
	(mutex_type == NO_SWITCHING_MUTEX);
    handle_to_global_data = dummy_handle = new PHYSMEM_HANDLE;
    handle_to_global_data->set_to_ptr(shm_addr_offset, size);
    if ((connection_number < 0 || connection_number >= total_connections)
	&& (mutex_type == MAO_MUTEX || mutex_type == MAO_MUTEX_W_OS_SEM)) {
	rcs_print_error("Bad connection number %ld\n", connection_number);
	status = CMS_MISC_ERROR;
	return -1;
    }
    return 0;
}

/* Closes the  shared memory and mutual exclusion semaphore  descriptors. */
int SHMEM::close()
{
    int nattch = 0;
    second_read = 0;

/*! \todo Another #if 0 */
#if 0				// PC Do we need to use autokey ?
    if (use_autokey_for_connection_number) {
	void *autokey_table = (void *) (((char *) shm->addr) + 32);
	autokey_releasekey(autokey_table, total_connections, ProcessName,
	    connection_number);
    }
#endif
    if (NULL != shm) {
	/* see if we're the last one */
	nattch = shm->nattch();
	shm->delete_totally = delete_totally;
	delete shm;
	shm = NULL;
    }
    if (NULL != sem) {
	/* if we're the last one, then make us the master so that the
	   semaphore will go away */
	if ((nattch <= 1 && nattch > -1) || delete_totally) {
	    sem->setflag(RCS_SEMAPHORE_CREATE);
	} else {
	    sem->setflag(RCS_SEMAPHORE_NOCREATE);
	}
	delete sem;
    }
    if (NULL != bsem) {
	/* if we're the last one, then make us the master so that the
	   semaphore will go away */
	if ((nattch <= 1 && nattch > -1) || delete_totally) {
	    bsem->setflag(RCS_SEMAPHORE_CREATE);
	} else {
	    bsem->setflag(RCS_SEMAPHORE_NOCREATE);
	}
	delete bsem;
    }
#ifdef DEBUG
    printf("SHMEM(%s): nattch = %d\n", BufferName, nattch);
#endif

    return 0;
}

/* Access the shared memory buffer. */
CMS_STATUS SHMEM::main_access(void *_local, int *serial_number)
{

    /* Check pointers. */
    if (shm == NULL) {
	second_read = 0;
	return (status = CMS_MISC_ERROR);
    }

    if (bsem == NULL && not_zero(blocking_timeout)) {
	rcs_print_error
	    ("No blocking semaphore available. Can not call blocking_read(%f).\n",
	    blocking_timeout);
	second_read = 0;
	return (status = CMS_NO_BLOCKING_SEM_ERROR);
    }

    mao.read_only = ((internal_access_type == CMS_CHECK_IF_READ_ACCESS) ||
	(internal_access_type == CMS_PEEK_ACCESS) ||
	(internal_access_type == CMS_READ_ACCESS));

    switch (mutex_type) {
    case NO_MUTEX:
	break;

    case MAO_MUTEX:
    case MAO_MUTEX_W_OS_SEM:
	switch (mem_get_access(&mao)) {
	case -1:
	    rcs_print_error("SHMEM: Can't take semaphore\n");
	    second_read = 0;
	    return (status = CMS_MISC_ERROR);
	case -2:
	    if (timeout > 0) {
		rcs_print_error("SHMEM: Timed out waiting for semaphore.\n");
		rcs_print_error("buffer = %s, timeout = %lf sec.\n",
		    BufferName, timeout);
	    }
	    second_read = 0;
	    return (status = CMS_TIMED_OUT);
	default:
	    break;
	}
	toggle_bit = mao.toggle_bit;
	break;

    case OS_SEM_MUTEX:
	if (sem == NULL) {
	    second_read = 0;
	    return (status = CMS_MISC_ERROR);
	}
	switch (sem->wait()) {
	case -1:
	    rcs_print_error("SHMEM: Can't take semaphore\n");
	    second_read = 0;
	    return (status = CMS_MISC_ERROR);
	case -2:
	    if (timeout > 0) {
		rcs_print_error("SHMEM: Timed out waiting for semaphore.\n");
		rcs_print_error("buffer = %s, timeout = %lf sec.\n",
		    BufferName, timeout);
	    }
	    second_read = 0;
	    return (status = CMS_TIMED_OUT);
	default:
	    break;
	}
	break;

    case NO_INTERRUPTS_MUTEX:
	rcs_print_error("Interrupts can not be disabled.\n");
	second_read = 0;
	return (status = CMS_MISC_ERROR);
	break;

    case NO_SWITCHING_MUTEX:
	rcs_print_error("Interrupts can not be disabled.\n");
	return (status = CMS_MISC_ERROR);
	break;

    default:
	rcs_print_error("SHMEM: Invalid mutex type.(%d)\n", mutex_type);
	second_read = 0;
	return (status = CMS_MISC_ERROR);
	break;
    }

    if (second_read > 0 && enable_diagnostics) {
	disable_diag_store = 1;
    }

    /* Perform access function. */
    internal_access(shm->addr, size, _local, serial_number);

    disable_diag_store = 0;

    if (NULL != bsem &&
	(internal_access_type == CMS_WRITE_ACCESS
	    || internal_access_type == CMS_WRITE_IF_READ_ACCESS)) {
	bsem->flush();
    }
    switch (mutex_type) {
    case NO_MUTEX:
	break;

    case MAO_MUTEX:
    case MAO_MUTEX_W_OS_SEM:
	mem_release_access(&mao);
	break;

    case OS_SEM_MUTEX:
	sem->post();
	break;
    case NO_INTERRUPTS_MUTEX:
	rcs_print_error("Can not restore interrupts.\n");
	break;

    case NO_SWITCHING_MUTEX:
	rcs_print_error("Can not restore interrupts.\n");
	break;
    }

    switch (internal_access_type) {

    case CMS_READ_ACCESS:
	if (NULL != bsem && status == CMS_READ_OLD &&
	    not_zero(blocking_timeout)) {
	    if (second_read > 10 && total_subdivisions <= 1) {
		status = CMS_MISC_ERROR;
		rcs_print_error
		    ("CMS: Blocking semaphore error. The semaphore wait has returned %d times but there is still no new data.\n",
		    second_read);
		second_read = 0;
		return (status);
	    }
	    second_read++;
	    bsem->timeout = blocking_timeout;
	    int bsem_ret = bsem->wait();
	    if (bsem_ret == -2) {
		status = CMS_TIMED_OUT;
		second_read = 0;
		return (status);
	    }
	    if (bsem_ret == -1) {
		rcs_print_error("CMS: Blocking semaphore error.\n");
		status = CMS_MISC_ERROR;
		second_read = 0;
		return (status);
	    }
	    main_access(_local, serial_number);
	}
	break;

    case CMS_WRITE_ACCESS:
    case CMS_WRITE_IF_READ_ACCESS:
	break;

    default:
	break;

    }

    second_read = 0;
    return (status);
}
