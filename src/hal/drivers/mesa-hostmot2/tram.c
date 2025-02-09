
//
//    Copyright (C) 2007-2008 Sebastian Kuzminsky
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

#include <rtapi_slab.h>
#include <rtapi_list.h>

#include "rtapi.h"
#include "rtapi_string.h"
#include "rtapi_math.h"

#include "hal.h"

#include "hal/drivers/mesa-hostmot2/hostmot2.h"




//
// This function is called by the MD parsers.  It records that in the
// specified hm2 instance, the specified register address range should be
// copied to memory on the Linux computer, and *buffer should point to that
// memory.
//
// The hm2_allocate_tram_regions() function below actually allocates
// the memory and writes the buffer variable.
//
// in the future this function will inform the Translation RAM
//

int hm2_register_tram_read_region(hostmot2_t *hm2, rtapi_u16 addr, rtapi_u16 size, rtapi_u32 **buffer) {
    hm2_tram_entry_t *tram_entry;

    tram_entry = rtapi_kmalloc(sizeof(hm2_tram_entry_t), RTAPI_GFP_KERNEL);
    if (tram_entry == NULL) {
        HM2_ERR("out of memory!\n");
        return -ENOMEM;
    }

    tram_entry->addr = addr;
    tram_entry->size = size;
    tram_entry->buffer = buffer;

    rtapi_list_add_tail(&tram_entry->list, &hm2->tram_read_entries);

    // Not a memory leak. The 'tram_entry' is part of the list.
    // cppcheck-suppress memleak
    return 0;
}


int hm2_register_tram_write_region(hostmot2_t *hm2, rtapi_u16 addr, rtapi_u16 size, rtapi_u32 **buffer) {
    hm2_tram_entry_t *tram_entry;

    tram_entry = rtapi_kmalloc(sizeof(hm2_tram_entry_t), RTAPI_GFP_KERNEL);
    if (tram_entry == NULL) {
        HM2_ERR("out of memory!\n");
        return -ENOMEM;
    }

    tram_entry->addr = addr;
    tram_entry->size = size;
    tram_entry->buffer = buffer;

    rtapi_list_add_tail(&tram_entry->list, &hm2->tram_write_entries);

    // Not a memory leak. The 'tram_entry' is part of the list.
    // cppcheck-suppress memleak
    return 0;
}


int hm2_allocate_tram_regions(hostmot2_t *hm2) {
    struct rtapi_list_head *ptr;
    rtapi_u16 offset;
    
    int old_tram_read_size = hm2->tram_read_size;
    int old_tram_write_size = hm2->tram_write_size;

    hm2->tram_read_size = 0;
    rtapi_list_for_each(ptr, &hm2->tram_read_entries) {
        hm2_tram_entry_t *tram_entry = rtapi_list_entry(ptr, hm2_tram_entry_t, list);
        hm2->tram_read_size += tram_entry->size;
    }

    hm2->tram_write_size = 0;
    rtapi_list_for_each(ptr, &hm2->tram_write_entries) {
        hm2_tram_entry_t *tram_entry = rtapi_list_entry(ptr, hm2_tram_entry_t, list);
        hm2->tram_write_size += tram_entry->size;
    }

    HM2_DBG(
        "allocating Translation RAM buffers (reading %d bytes, writing %d bytes)\n",
        hm2->tram_read_size,
        hm2->tram_write_size
    );

    hm2->tram_read_buffer = (rtapi_u32 *)rtapi_krealloc(hm2->tram_read_buffer, hm2->tram_read_size, RTAPI_GFP_KERNEL);
    if (hm2->tram_read_buffer == NULL) {
        HM2_ERR("Error while (re)allocating Translation RAM read buffer (%d bytes)\n", hm2->tram_read_size);
        return -ENOMEM;
    }
    if(hm2->tram_read_size>old_tram_read_size)
        memset((char*)hm2->tram_read_buffer+old_tram_read_size, 0, hm2->tram_read_size-old_tram_read_size);
    
    hm2->tram_write_buffer = (rtapi_u32 *)rtapi_krealloc(hm2->tram_write_buffer, hm2->tram_write_size, RTAPI_GFP_KERNEL);
    if (hm2->tram_write_buffer == NULL) {
        HM2_ERR("Error while (re)allocating Translation RAM write buffer (%d bytes)\n", hm2->tram_write_size);
        return -ENOMEM;
    }
    if(hm2->tram_write_size>old_tram_write_size)
        memset((char*)hm2->tram_write_buffer+old_tram_write_size, 0, hm2->tram_write_size-old_tram_write_size);

    HM2_DBG("buffer address %p\n", &hm2->tram_write_buffer);
    HM2_DBG("Translation RAM read buffer:\n");
    offset = 0;
    rtapi_list_for_each(ptr, &hm2->tram_read_entries) {
        hm2_tram_entry_t *tram_entry = rtapi_list_entry(ptr, hm2_tram_entry_t, list);
        *tram_entry->buffer = (rtapi_u32*)((rtapi_u8*)hm2->tram_read_buffer + offset);
        offset += tram_entry->size;
        HM2_DBG("    addr=0x%04x, size=%d, buffer=%p\n", tram_entry->addr, tram_entry->size, *tram_entry->buffer);
    }

    HM2_DBG("Translation RAM write buffer:\n");
    offset = 0;
    rtapi_list_for_each(ptr, &hm2->tram_write_entries) {
        hm2_tram_entry_t *tram_entry = rtapi_list_entry(ptr, hm2_tram_entry_t, list);
        *tram_entry->buffer = (rtapi_u32*)((rtapi_u8*)hm2->tram_write_buffer + offset);
        offset += tram_entry->size;
        HM2_DBG("    addr=0x%04x, size=%d, buffer=%p\n", tram_entry->addr, tram_entry->size, *tram_entry->buffer);
    }
    
    return 0;
}


static rtapi_u32 tram_read_iteration = 0;
int hm2_tram_read(hostmot2_t *hm2) {
    struct rtapi_list_head *ptr;

    rtapi_list_for_each(ptr, &hm2->tram_read_entries) {
        hm2_tram_entry_t *tram_entry = rtapi_list_entry(ptr, hm2_tram_entry_t, list);

        if (!hm2->llio->queue_read(hm2->llio, tram_entry->addr, *tram_entry->buffer, tram_entry->size)) {
            HM2_ERR("TRAM read error! (addr=0x%04x, size=%d, iter=%u)\n", tram_entry->addr, tram_entry->size, tram_read_iteration);
            return -EIO;
        }
    }
    tram_read_iteration ++;

    return 0;
}

int hm2_queue_read(hostmot2_t *hm2) {
    if (!hm2->llio->send_queued_reads) return 0;
    if (!hm2->llio->send_queued_reads(hm2->llio)) {
        HM2_ERR("error queuing read! iter=%u)\n",
            tram_read_iteration);
        return -EIO;
    }
    return 0;
}

int hm2_finish_read(hostmot2_t *hm2) {
    if (!hm2->llio->receive_queued_reads) return 0;
    int r = hm2->llio->receive_queued_reads(hm2->llio);
    if (r < 0) return r;
    if (!r) {
        HM2_ERR("error finishing read! iter=%u\n",
            tram_read_iteration);
        return -EIO;
    }
    return 0;
}


static rtapi_u32 tram_write_iteration = 0;
int hm2_tram_write(hostmot2_t *hm2) {
    struct rtapi_list_head *ptr;

    rtapi_list_for_each(ptr, &hm2->tram_write_entries) {
        hm2_tram_entry_t *tram_entry = rtapi_list_entry(ptr, hm2_tram_entry_t, list);

        if (!hm2->llio->queue_write(hm2->llio, tram_entry->addr, *tram_entry->buffer, tram_entry->size)) {
            HM2_ERR("TRAM write error! (addr=0x%04x, size=%d, iter=%u)\n", tram_entry->addr, tram_entry->size, tram_write_iteration);
            return -EIO;
        }
    }
    tram_write_iteration ++;

    return 0;
}

int hm2_finish_write(hostmot2_t *hm2) {
    if (!hm2->llio->send_queued_writes) return 0;
    if (!hm2->llio->send_queued_writes(hm2->llio)) {
        HM2_ERR("error finishing write! iter=%u)\n",
            tram_write_iteration);
        return -EIO;
    }

    return 0;
}


void hm2_tram_cleanup(hostmot2_t *hm2) {
    while (hm2->tram_read_entries.next != &hm2->tram_read_entries) {
        struct rtapi_list_head *te_ptr = hm2->tram_read_entries.next;
        hm2_tram_entry_t *te = rtapi_list_entry(te_ptr, hm2_tram_entry_t, list);
        rtapi_list_del(te_ptr);
        rtapi_kfree(te);
    }
    while (hm2->tram_write_entries.next != &hm2->tram_write_entries) {
        struct rtapi_list_head *te_ptr = hm2->tram_write_entries.next;
        hm2_tram_entry_t *te = rtapi_list_entry(te_ptr, hm2_tram_entry_t, list);
        rtapi_list_del(te_ptr);
        rtapi_kfree(te);
    }

    // free the tram buffers
    if (hm2->tram_read_buffer != NULL) rtapi_kfree(hm2->tram_read_buffer);
    if (hm2->tram_write_buffer != NULL) rtapi_kfree(hm2->tram_write_buffer);
}

