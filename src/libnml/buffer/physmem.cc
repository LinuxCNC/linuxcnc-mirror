/********************************************************************
* Description: physmem.cc
*   Provides the member functions for the PHYSMEM_HANDLE class. This
*   is a C++ interface for portable access to physical memory.
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

#include <string.h>		/* memcpy(), memset() */
#include <stdio.h>		// sprintf()
#include <stdlib.h>		/* malloc() */

#ifdef __cplusplus
}
#endif
#include "physmem.hh"		/* class PHYSMEM_HANDLE */
#include "rcs_print.hh"
PHYSMEM_HANDLE::PHYSMEM_HANDLE()
  : offset(0),
    size(0),
    address_code(0),
    isvalid(1),
    temp_buf(NULL),
    physical_address(0),
    local_address(NULL),
    using_bit3(0),
    total_bytes_moved(0.0),
    enable_byte_counting(0)
{
}

PHYSMEM_HANDLE::PHYSMEM_HANDLE(unsigned long _physical_address, long _address_code, long _size)
  : offset(0),
    size(_size),
    address_code(_address_code),
    isvalid(1),
    temp_buf(NULL),
    physical_address(_physical_address),
    local_address(NULL),
    using_bit3(0),
    total_bytes_moved(0.0),
    enable_byte_counting(0)
{
    if (0 == physical_address) {
	local_address = (LOCAL_ADDRESS_TYPE) NULL;
	return;
    }
    local_address = (LOCAL_ADDRESS_TYPE) physical_address;
}

/* Destructor. */
PHYSMEM_HANDLE::~PHYSMEM_HANDLE()
{
}

/* Use an ordinary pointer to access memory. */
void
  PHYSMEM_HANDLE::set_to_ptr(void *_ptr, long _size)
{
    local_address = (LOCAL_ADDRESS_TYPE) _ptr;
    size = _size;
    offset = 0;
}

static int physmem_read_local_address_is_null_error_print_count = 0;

/***********************************************************
* Read _read_size bytes from physical memory to store at _to.
* Returns: 0 for success or -1 for failure.
**********************************************************/
int PHYSMEM_HANDLE::read(void *_to, long _read_size)
{
    if (NULL == _to) {
	rcs_print_error("PHYSMEM_HANDLE::read _to = NULL.\n");
	return (-1);
    }
    /* Check sizes. */
    if (_read_size + offset > size || offset < 0) {
	rcs_print_error
	    ("PHYSMEM_HANDLE: Can't read %ld bytes at offset %ld from buffer of size %ld.\n",
	    _read_size, offset, size);
	return (-1);
    }

    if (enable_byte_counting) {
	total_bytes_moved += _read_size;
    }

    /* If local_address has been initialized use it as an ordinary pointer. */
    if (NULL != local_address) {
	char *from;
	from = ((char *) local_address) + offset;
	if (_read_size == 2) {
	    short *sfrom = (short *) from;
	    short sval;
	    sval = *sfrom;
	    short *sto = (short *) _to;
	    *sto = sval;
	} else {
	    memcpy(_to, from, _read_size);
	}
	return (0);
    }

    /* include platform specific ways of accessing physical memory here. */
    if (!(physmem_read_local_address_is_null_error_print_count % 100000)) {
	rcs_print_error
	    ("PHYSMEM_HANDLE: Cannot read from physical memory when local address is NULL.\n");
	rcs_print_error("(This error has occurred %d times.)\n",
	    physmem_read_local_address_is_null_error_print_count + 1);
    }
    physmem_read_local_address_is_null_error_print_count++;
    return (-1);
}

static int physmem_write_local_address_is_null_error_print_count = 0;

/***********************************************************
* Write _write_size bytes from memory at _from to physical memory.
* Returns: 0 for success or -1 for failure.
**********************************************************/
int PHYSMEM_HANDLE::write(void *_from, long _write_size)
{
    if (NULL == _from) {
	rcs_print_error("PHYSMEM_HANDLE:write _from = NULL\n");
	return -1;
    }

    /* Check sizes. */
    if (_write_size + offset > size || offset < 0) {
	rcs_print_error
	    ("PHYSMEM_HANDLE: Can't write %ld bytes at offset %ld from buffer of size %ld.\n",
	    _write_size, offset, size);
	return (-1);
    }
    if (enable_byte_counting) {
	total_bytes_moved += _write_size;
    }
    /* If local_address has been initialized use it as an ordinary pointer. */
    if (NULL != local_address) {
	char *to;
	to = ((char *) local_address) + offset;
	if (_write_size == 2) {
	    short *sto = (short *) to;
	    short sval = *(short *) _from;
	    *sto = sval;
	} else {
	    memcpy(to, _from, _write_size);
	}
	return (0);
    }

    /* include platform specific ways of accessing physical memory here. */
    if (!(physmem_write_local_address_is_null_error_print_count % 100000)) {
	rcs_print_error
	    ("PHYSMEM_HANDLE: Cannot write to physical memory when local address is NULL.\n");
	rcs_print_error("(This error has occurred %d times.)\n",
	    physmem_write_local_address_is_null_error_print_count + 1);
    }
    physmem_write_local_address_is_null_error_print_count++;
    return (-1);
}

void PHYSMEM_HANDLE::memsetf(long _memset_offset, char _byte,
    long _memset_size)
{
    /* Check sizes. */
    if (_memset_size + _memset_offset > size) {
	return;
    }

    /* If local_address has been initialized use it as an ordinary pointer. */
    if (NULL != local_address) {
	char *temp_addr;
	temp_addr = ((char *) local_address) + _memset_offset;
	memset(temp_addr, _byte, _memset_size);
	return;
    } else {
	/* Since local address is not initialized use temp_buf and write to
	   access the physical memory in an platform specific way. */
	if (NULL == temp_buf) {
	    temp_buf = (char *) malloc(size);
	}
	if (NULL != temp_buf) {
	    if (_memset_size + _memset_offset <= size) {
		memset(temp_buf, _byte, _memset_size);
		unsigned long old_offset;
		old_offset = offset;
		offset = _memset_offset;
		write(temp_buf, _memset_size);
		offset = old_offset;
	    } else {
		memset(temp_buf, _byte, size - _memset_offset);
		unsigned long old_offset;
		old_offset = offset;
		offset = _memset_offset;
		write(temp_buf, size - offset);
		offset = old_offset;
	    }
	}
    }
}

int PHYSMEM_HANDLE::clear_memory()
{
    /* If local_address has been initialized use it as an ordinary pointer. */
    if (NULL != local_address) {
	memset(local_address, 0, size);
	return (0);
    } else {
	/* Since local address is not initialized use temp_buf and write to
	   access the physical memory in an platform specific way. */
	if (NULL == temp_buf) {
	    temp_buf = (char *) malloc(size);
	}
	if (NULL == temp_buf) {
	    return (-1);
	}
	memset(temp_buf, 0, size);
	unsigned long old_offset;
	old_offset = offset;
	offset = 0;
	if (-1 == write(temp_buf, size)) {
	    offset = old_offset;
	    return (-1);
	}
	offset = old_offset;
    }
    return (0);
}

int PHYSMEM_HANDLE::valid()
{
    return isvalid;
}
