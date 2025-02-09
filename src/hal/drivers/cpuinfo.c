/*
Copyright (c) 2023 Andy Pugh
Initially (c) 2012 Ben Croston
Rewritten according to advice at
https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#best-practice-for-revision-code-usage

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "rtapi.h"

int get_rpi_revision(void)
{
    FILE *fp;
    char buffer[1024];
    char *r;
    unsigned int revision = 0;

    if ((fp = fopen("/sys/firmware/devicetree/base/model", "r")) == NULL)
        return -1;

    r = fgets(buffer, sizeof(buffer) , fp);
    fclose(fp);

    if (!r) return -1;
    
    rtapi_print_msg(RTAPI_MSG_INFO, "%s found\n", buffer);
    
    if (strncmp(buffer, "Raspberry",9) != 0)
        return -1;

    if ((fp = fopen("/proc/cpuinfo", "r")) == NULL)
        return -1;

    while(!feof(fp)) {
        if (fgets(buffer, sizeof(buffer) , fp)){
            sscanf(buffer, "Revision  : %x", &revision);
        }
    }
    fclose(fp);

    if ( ! (revision & 0x800000)){ // old-style revision code
        if ((revision & 0xFF) <= 3) return 1;
        return 2;
    } else {
        return ((revision >> 4) & 0xFF);
    }
}
