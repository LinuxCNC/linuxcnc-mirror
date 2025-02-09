/********************************************************************
* Description: shm.cc
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

#include "shm.hh"
#include "_shm.h"		/* rcs_shm_open(), rcs_shm_close() */
#include "rcs_print.hh"		// rcs_print_error()

extern "C" {
#include <stdio.h>		/* fprintf(), stderr */
#include <sys/types.h>		/* key_t */
#include <stddef.h>		/* size_t */
#include <errno.h>		// errno
#include <sys/ipc.h>		// IPC_CREAT
}

RCS_SHAREDMEM::RCS_SHAREDMEM(key_t key, size_t size, int oflag, int mode)
{
    shm = NULL;
    addr = NULL;
    delete_totally = 0;
    create_errno = 0;
    created = 0;

    if (oflag & RCS_SHAREDMEM_CREATE) {
	/* create shared memory */
#ifdef USE_POSIX_SHAREDMEM
	shm = rcs_shm_open(key, size, O_CREAT, mode);
#else
	shm = rcs_shm_open(key, size, IPC_CREAT, mode);
#endif
	if (shm == NULL) {
	    create_errno = errno;
	    rcs_print_error("can't create shared memory\n");
	    return;
	}
    } else {
	/* attach to existing shared memory */
	shm = rcs_shm_open(key, size, 0);
	if (shm == NULL) {
	    create_errno = errno;
	    rcs_print_error
		("can't attach to shared memory-- is master started?\n");
	    return;
	}
    }
    create_errno = shm->create_errno;
    created = shm->created;
    /* duplicate the pointer, so users don't have to dig into shm->addr */
    addr = shm->addr;
}

RCS_SHAREDMEM::~RCS_SHAREDMEM()
{
    if (shm == NULL) {
	return;
    } else {
	if (delete_totally) {
	    rcs_shm_delete(shm);
	} else {
	    rcs_shm_close(shm);
	}
	shm = NULL;
    }
}

int
  RCS_SHAREDMEM::nattch()
{
    if (shm == NULL) {
	return -1;
    } else {
	return rcs_shm_nattch(shm);
    }
}
