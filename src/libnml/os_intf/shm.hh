/********************************************************************
* Description: shm.hh
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

#ifndef SHAREDMEM_HH
#define SHAREDMEM_HH

extern "C" {
#include <sys/types.h>		/* key_t */
#include <stddef.h>		/* size_t */
#include "_shm.h"		/* shm_t */
}
#define RCS_SHAREDMEM_NOCREATE 0x00	/* just attach to existing sharedmem */
#define RCS_SHAREDMEM_CREATE 0x01	/* create sharedmem */
class RCS_SHAREDMEM {
  public:
    RCS_SHAREDMEM(key_t key, size_t size, int oflag, int mode = 0);
     ~RCS_SHAREDMEM();

    // Don't copy me.
    RCS_SHAREDMEM(const RCS_SHAREDMEM & shm) = delete;
    RCS_SHAREDMEM& operator=(const RCS_SHAREDMEM & shm) = delete;

    int nattch();		/* how many processes are attached */
    int create_errno;		/* 0 or stored errno after shmget failed */
    void *addr;			/* pointer to shared memory */
    int delete_totally;		/* Flag to clean the sharedmem completely */

  private:
      shm_t * shm;
  public:
    int created;
};

#endif
