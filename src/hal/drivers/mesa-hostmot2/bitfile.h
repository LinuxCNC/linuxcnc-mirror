
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
//
//    The code in this file is based on bfload by John Kasunich and
//    m5i20cfg by Peter C. Wallace.  See src/hal/util/bitfile.h for some
//    good comments on the bitfile format.
//


#ifndef __BITFILE_H
#define __BITFILE_H


#include <rtapi_firmware.h>
#include <rtapi_stdint.h>




typedef struct {
    size_t size;
    const unsigned char *data;  // a pointer into the "parent" struct firmware
} bitfile_chunk_t;


typedef struct {
    bitfile_chunk_t a, b, c, d, e;
} bitfile_t;




int bitfile_parse_and_verify(const struct rtapi_firmware *fw, bitfile_t *bitfile);
rtapi_u8 bitfile_reverse_bits(rtapi_u8 data);




#endif  // __BITFILE_H

