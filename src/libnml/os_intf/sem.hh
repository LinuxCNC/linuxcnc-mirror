/********************************************************************
* Description: sem.hh
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
#ifndef SEMAPHORE_HH
#define SEMAPHORE_HH

extern "C" {
#include "_sem.h"		/* rcs_sem_t */
#include <sys/stat.h>		/* S_IRUSR, etc. */
}
/* rw-rw-rw- permissions */
#define DEFAULT_SEM_MODE (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)
#define RCS_SEMAPHORE_NOCREATE 0x00	/* just attach to existing semaphore */
#define RCS_SEMAPHORE_CREATE 0x01	/* create semaphore */
class RCS_SEMAPHORE {
    /* RCS_SEMAPHORE objects can be used for mutual exclusion of resources
       shared by multiple processes on the same host.

       To use:

       Create or attach to the semaphore by initializing the RCS_SEMAPHORE
       object. You will need an id agreed on by the all the processes using
       the semaphore and you must specify which process is responsible for
       creating the semaphore. Surround accesses to the shared resource with */
  public:
    RCS_SEMAPHORE(key_t id, int oflag, double _timeout, int mode =
	DEFAULT_SEM_MODE, int state = 0);
    /* Initializes an RCS_SEMAPHORE object. If _oflag equals
       RCS_SEMAPHORE_CREATE a semaphore is created. If _oflag equals
       RCS_SEMAPHORE_NOCREATE the process will try to attach to a semaphore
       that must already have been created with the same _id. If _timeout is
       positive, then calls to RCS_SEMAPHORE::wait() will return -1 after
       timeout seconds. If _timeout is negative, then RCS_SEMAPHORE::wait()
       will wait indefinitely for the semaphore to be available. If _timeout
       is zero, then RCS_SEMAPHORE::wait() will return immediately with 0 if
       the semaphore was available or -1 if it was not. The _mode determines
       which users will have permission to use the semaphore. You can or
       together symbolic constants from sys/stat.h. The default value of
       _mode, DEFAULT_SEM_MODE, allows read and write access to everyone. The 
       _mode will be ignored if the process is not creating the semaphore.
       The _state should be 1 to make the semaphore immediately available.
       The _state will be ignored if the process is not creating the
       semaphore. */
     ~RCS_SEMAPHORE();

    // Don't copy me.
    RCS_SEMAPHORE(const RCS_SEMAPHORE & sem) = delete;
    RCS_SEMAPHORE& operator=(const RCS_SEMAPHORE & sem) = delete;

    int wait();
    /* Wait for the semaphore to be available and then take it. See the
       constructors parameters for several options affecting its behavior.
       Returns 0 for success or -1 for failure. */

    int trywait();
    /* If the semaphore is available take it. Returns 0 for success or -1 for 
       failure. */

    int post();
    /* Release the semaphore. Returns 0 for success or -1 for failure. */

    int flush();
    /* Test to see if the semaphore is available but don't take it even if it 
       is. Returns a positive integer if the semaphore is available or 0 if
       it is not. */

    /* additional non-POSIX functions */

    int setflag(int oflag);	/* change oflag-- one can toggle the state of 
				   this being the master, so that flexible
				   destruction of OS semaphore can be done */
    int valid();
    int clear();		// Make sure this semaphore will not
    // immediately be available.

    unsigned long int id;
    double timeout;
    int oflag;
    int mode;
    int state;
    rcs_sem_t *sem;
    unsigned int sval;
};

#endif
