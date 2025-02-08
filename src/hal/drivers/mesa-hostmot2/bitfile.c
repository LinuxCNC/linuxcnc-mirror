
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


#include <rtapi_firmware.h>

#include "rtapi.h"
#include "rtapi_string.h"
#include "rtapi_math.h"

#include "hal.h"

#include "hostmot2.h"
#include "bitfile.h"




static int bitfile_do_small_chunk(const struct rtapi_firmware *fw, bitfile_chunk_t *chunk, size_t *i) {
    if (*i + 2 > fw->size) {
        HM2_PRINT_NO_LL("bitfile chunk extends past end of firmware\n");
        return -EFAULT;
    }

    chunk->size = (fw->data[*i] * 256) + fw->data[*i + 1];
    (*i) += 2;

    if (*i + chunk->size > fw->size) {
        HM2_PRINT_NO_LL("bitfile chunk extends past end of firmware\n");
        return -EFAULT;
    }

    chunk->data = &fw->data[*i];

    if (chunk->data[chunk->size - 1] != '\0') {
        HM2_PRINT_NO_LL("bitfile small chunk is not NULL terminated\n");
        return -EINVAL;
    }

    (*i) += chunk->size;

    return 0;
}




static int bitfile_do_big_chunk(const struct rtapi_firmware *fw, bitfile_chunk_t *chunk, size_t *i) {
    if (*i + 4 > fw->size) {
        HM2_PRINT_NO_LL("bitfile chunk extends past end of firmware\n");
        return -EFAULT;
    }

    chunk->size = ((uint32_t)fw->data[*i] << 24) + ((uint32_t)fw->data[*i + 1] << 16) + ((uint32_t)fw->data[*i + 2] << 8) + fw->data[*i + 3];
    (*i) += 4;

    if (*i + chunk->size > fw->size) {
        HM2_PRINT_NO_LL("bitfile chunk extends past end of firmware\n");
        return -EFAULT;
    }

    chunk->data = &fw->data[*i];
    (*i) += chunk->size;

    return 0;
}




static int bitfile_parse_and_verify_chunk(const struct rtapi_firmware *fw, bitfile_t *bitfile, size_t *i) {
    char tag;

    tag = fw->data[*i];
    (*i) ++;

    if ((*i) > fw->size) {
        HM2_PRINT_NO_LL("bitfile chunk '%c' size fell off the end!\n", tag);
        return -EFAULT;
    }

    switch (tag) {
        case 'a':
            // Design name
            return bitfile_do_small_chunk(fw, &bitfile->a, i);

        case 'b':
            // Part ID
            return bitfile_do_small_chunk(fw, &bitfile->b, i);

        case 'c':
            // Design date
            return bitfile_do_small_chunk(fw, &bitfile->c, i);

        case 'd':
            // Design time
            return bitfile_do_small_chunk(fw, &bitfile->d, i);

        case 'e':
            return bitfile_do_big_chunk(fw, &bitfile->e, i);

        default: {
            HM2_PRINT_NO_LL("bitfile has unknown chunk '%c'\n", tag);
            break;
        }
    }

    // unknown chunks cause error, because we don't know if they're big or small
    return -EINVAL;
}




#define BITFILE_HEADERLEN 13

int bitfile_parse_and_verify(const struct rtapi_firmware *fw, bitfile_t *bitfile) {
    size_t i;
    int r;

    const unsigned char bitfile_header[BITFILE_HEADERLEN] = {
        0x00, 0x09,
        0x0f, 0xf0, 0x0f, 0xf0,
        0x0f, 0xf0, 0x0f, 0xf0,
        0x00, 0x00, 0x01
    };


    //
    // initialize all the bitfile chunks
    //

    bitfile->a.size = 0;
    bitfile->a.data = NULL;

    bitfile->b.size = 0;
    bitfile->b.data = NULL;

    bitfile->c.size = 0;
    bitfile->c.data = NULL;

    bitfile->d.size = 0;
    bitfile->d.data = NULL;

    bitfile->e.size = 0;
    bitfile->e.data = NULL;


    // 
    // verify the header
    //

    if (fw->size < BITFILE_HEADERLEN) {
        HM2_PRINT_NO_LL("bitfile is too short\n");
        return -EFAULT;
    }

    for (i = 0; i < BITFILE_HEADERLEN; i ++) {
        if (fw->data[i] != bitfile_header[i]) {
            HM2_PRINT_NO_LL("bitfile has invalid header\n");
            return -EINVAL;
        }
    }


    // 
    // parse and verify all the chunks
    //

    while (i < fw->size) {
        r = bitfile_parse_and_verify_chunk(fw, bitfile, &i);
        if (r != 0) return r;
    }


    // 
    // make sure we got all the required chunks
    //

    if (bitfile->b.data == NULL) {
        HM2_PRINT_NO_LL("bitfile lacks Part Name (chunk 'b')!\n");
        return -EINVAL;
    }

    if (bitfile->e.data == NULL) {
        HM2_PRINT_NO_LL("bitfile lacks FPGA Config (part 'e')!\n");
        return -EINVAL;
    }


    // looks like a good bitfile
    return 0;
}




//
// the fpga was originally designed to be programmed serially... even
// though we are doing it using a parallel interface, the bit ordering
// is based on the serial interface, and the data needs to be reversed
//

rtapi_u8 bitfile_reverse_bits(rtapi_u8 data) {
    static const rtapi_u8 swaptab[256] = {
	0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
	0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
	0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
	0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
	0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
	0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
	0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
	0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
	0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
	0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
	0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
	0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
	0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
	0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
	0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
	0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
    };
    return swaptab[data];
}

