//    Copyright 2014 Jeff Epler
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
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#ifndef RTAPI_DEVICE_H
#define RTAPI_DEVICE_H

#ifdef __KERNEL__
#include <linux/device.h>

// type
#ifndef rtapi_device // could be defined in first-included rtapi_firmware.h
#define rtapi_device device
#endif

// functions
#define rtapi_dev_set_name dev_set_name
#define rtapi_device_register device_register
#define rtapi_device_unregister device_unregister

#else

#include <rtapi.h>
#include <stdio.h>
#include <stdarg.h>

RTAPI_BEGIN_DECLS

#define RTAPI_DEVICE_NAME_LEN 47
struct rtapi_device {
    char    name[RTAPI_DEVICE_NAME_LEN + 1];
    void    (*release)(struct rtapi_device *dev);
};

static __inline__ int
rtapi_dev_set_name(struct rtapi_device *dev, const char *name, ...)
    __attribute__((format(printf,2,3)));
static __inline__ int
rtapi_dev_set_name(struct rtapi_device *dev, const char *name, ...)
{
    va_list ap;
    va_start(ap, name);
    int result = vsnprintf(dev->name, sizeof(dev->name), name, ap);
    va_end(ap);
    return result;
}


static __inline__ int rtapi_device_register(struct rtapi_device *dev) { (void)dev; return 0;}
static __inline__ void rtapi_device_unregister(struct rtapi_device *dev) { dev->release(dev); };

RTAPI_END_DECLS

#endif
#endif
