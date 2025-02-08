
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
#include <rtapi_ctype.h>
#include <rtapi_list.h>

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "rtapi_math.h"

#include "hal.h"

#include "hostmot2.h"
#include "bitfile.h"




MODULE_INFO(linuxcnc, "component:hostmot2:RTAI driver for the HostMot2 firmware from Mesa Electronics.");
MODULE_INFO(linuxcnc, "funct:read:1:Read all registers.");
MODULE_INFO(linuxcnc, "funct:write:1:Write all registers, and pet the watchdog to keep it from biting.");
MODULE_INFO(linuxcnc, "license:GPL");

MODULE_LICENSE("GPL");




int debug_idrom = 0;
RTAPI_MP_INT(debug_idrom, "Developer/debug use only!  Enable debug logging of the HostMot2\nIDROM header.");

int debug_module_descriptors = 0;
RTAPI_MP_INT(debug_module_descriptors, "Developer/debug use only!  Enable debug logging of the HostMot2\nModule Descriptors.");

int debug_modules = 0;
RTAPI_MP_INT(debug_modules, "Developer/debug use only!  Enable debug logging of the HostMot2\nModules used.");

int use_serial_numbers = 0;
RTAPI_MP_INT(use_serial_numbers, "Name cards by serial number, not enumeration order (smart-serial only)");

int sserial_baudrate = -1;
RTAPI_MP_INT(sserial_baudrate, "Over-ride the standard smart-serial baud rate. For flashing remote firmware only.");


// this keeps track of all the hm2 instances that have been registered by
// the low-level drivers
struct rtapi_list_head hm2_list;


static int comp_id;




//
// functions exported to LinuxCNC
//

static void hm2_read_request(void *void_hm2, long period) {
    hostmot2_t *hm2 = void_hm2;
    hm2->llio->period = period;

    // if there are comm problems, wait for the user to fix it
    if ((*hm2->llio->io_error) != 0) return;

    hm2_tram_read(hm2);
    if ((*hm2->llio->io_error) != 0) return;
    hm2_raw_queue_read(hm2);
    hm2_tp_pwmgen_queue_read(hm2);
    if ((*hm2->llio->io_error) != 0) return;
    hm2_queue_read(hm2);
    hm2->llio->read_requested = true;
    hm2->llio->read_time = rtapi_get_time();
}

static void hm2_read(void *void_hm2, long period) {
    hostmot2_t *hm2 = void_hm2;

    if(!hm2->llio->read_requested) hm2_read_request(void_hm2, period);
    hm2->llio->read_requested = false;

    // if there are comm problems, wait for the user to fix it
    if ((*hm2->llio->io_error) != 0) return;
    // if there's a temporary read failure, don't sweat it
    if(hm2_finish_read(hm2) == -EAGAIN) return;
    if ((*hm2->llio->io_error) != 0) return;

    hm2_watchdog_process_tram_read(hm2);
    hm2_ioport_gpio_process_tram_read(hm2);
    hm2_encoder_process_tram_read(hm2, period);
    hm2_inmux_process_tram_read(hm2);
    hm2_inm_process_tram_read(hm2);
    hm2_xy2mod_process_tram_read(hm2);
    hm2_resolver_process_tram_read(hm2, period);
    hm2_stepgen_process_tram_read(hm2, period);
    hm2_sserial_process_tram_read(hm2, period);
    hm2_bspi_process_tram_read(hm2, period);
    hm2_absenc_process_tram_read(hm2, period);
    hm2_oneshot_process_tram_read(hm2);
    hm2_periodm_process_tram_read(hm2);
    //UARTS PktUARTS need to be explicitly handled by an external component

    hm2_tp_pwmgen_process_read(hm2); // check the status of the fault bit
    hm2_dpll_process_tram_read(hm2, period);
}


static void hm2_write(void *void_hm2, long period) {
    hostmot2_t *hm2 = void_hm2;

    // if there are comm problems, wait for the user to fix it
    if ((*hm2->llio->io_error) != 0) return;

    if (!hm2->ddr_initialized) {
        hm2_ioport_initialize_ddr(hm2);
        hm2->ddr_initialized = true;
    }

    hm2_watchdog_prepare_tram_write(hm2);
    hm2_ioport_gpio_prepare_tram_write(hm2);
    hm2_pwmgen_prepare_tram_write(hm2);
    hm2_oneshot_prepare_tram_write(hm2);
    hm2_periodm_prepare_tram_write(hm2);
    hm2_rcpwmgen_prepare_tram_write(hm2);
    hm2_inmux_prepare_tram_write(hm2);
    hm2_inm_prepare_tram_write(hm2);
    hm2_tp_pwmgen_prepare_tram_write(hm2);
    hm2_stepgen_prepare_tram_write(hm2, period);
    hm2_sserial_prepare_tram_write(hm2, period);
    hm2_bspi_prepare_tram_write(hm2, period);
    hm2_ssr_prepare_tram_write(hm2);
    hm2_outm_prepare_tram_write(hm2);

    //UARTS need to be explicitly handled by an external component
    hm2_tram_write(hm2);

    // these usually do nothing
    // they only write to the FPGA if certain pins & params have changed
    hm2_ioport_write(hm2);    // handles gpio.is_output but not gpio.out (that's done in tram_write() above)
    hm2_watchdog_write(hm2, period);  // in case the user has written to the watchdog.timeout_ns param
    hm2_pwmgen_write(hm2);    // update pwmgen registers if needed
    hm2_oneshot_write(hm2);   // update oneshot registers if needed
    hm2_periodm_write(hm2);   // update periodm registers if needed
    hm2_rcpwmgen_write(hm2);  // update rcpwmgen registers if needed
    hm2_inmux_write(hm2);     // update inmux control register if needed
    hm2_inm_write(hm2);       // update inm control register if needed
    hm2_xy2mod_write(hm2);    // update xy2mod motion registers if needed
    hm2_tp_pwmgen_write(hm2); // update Three Phase PWM registers if needed
    hm2_stepgen_write(hm2);   // update stepgen registers if needed
    hm2_encoder_write(hm2);   // update ctrl register if needed
    hm2_absenc_write(hm2);    // set bit-lengths and frequency
    hm2_resolver_write(hm2, period); // Update the excitation frequency
    hm2_dpll_write(hm2, period); // Update the timer phases
    hm2_led_write(hm2);	      // Update on-board LEDs
    hm2_ssr_write(hm2);

    hm2_raw_write(hm2);
    hm2_finish_write(hm2);
}


static void hm2_read_gpio(void *void_hm2, long period) {
    (void)period;
    hostmot2_t *hm2 = void_hm2;

    // if there are comm problems, wait for the user to fix it
    if ((*hm2->llio->io_error) != 0) return;

    hm2_ioport_gpio_read(hm2);
}


static void hm2_write_gpio(void *void_hm2, long period) {
    hostmot2_t *hm2 = void_hm2;

    // if there are comm problems, wait for the user to fix it
    if ((*hm2->llio->io_error) != 0) return;

    hm2_ioport_gpio_write(hm2);
    hm2_watchdog_write(hm2, period);
}




//
// misc little helper functions
//


// FIXME: the static automatic string makes this function non-reentrant
const char *hm2_hz_to_mhz(rtapi_u32 freq_hz) {
    static char mhz_str[20];
    int r;
    int freq_mhz, freq_mhz_fractional;

    freq_mhz = freq_hz / (1000*1000);
    freq_mhz_fractional = (freq_hz / 1000) % 1000;
    r = snprintf(mhz_str, sizeof(mhz_str), "%d.%03d", freq_mhz, freq_mhz_fractional);
    if (r >= (int)sizeof(mhz_str)) {
        HM2_ERR_NO_LL("too many MHz!\n");
        return "(unpresentable)";
    }

    return mhz_str;
}

// FIXME: It would be nice if this was more generic
EXPORT_SYMBOL_GPL(hm2_get_bspi);
int hm2_get_bspi(hostmot2_t** hm2, char *name){
    struct rtapi_list_head *ptr;
    int i;
    rtapi_list_for_each(ptr, &hm2_list) {
        *hm2 = rtapi_list_entry(ptr, hostmot2_t, list);
        if ((*hm2)->bspi.num_instances > 0) {
            for (i = 0; i < (*hm2)->bspi.num_instances ; i++) {
                if (!strcmp((*hm2)->bspi.instance[i].name, name)) {return i;}
            }
        }
    }
    return -1;
}

EXPORT_SYMBOL_GPL(hm2_get_uart);
int hm2_get_uart(hostmot2_t** hm2, char *name){
    struct rtapi_list_head *ptr;
    int i;
    rtapi_list_for_each(ptr, &hm2_list) {
        *hm2 = rtapi_list_entry(ptr, hostmot2_t, list);
        if ((*hm2)->uart.num_instances > 0) {
            for (i = 0; i < (*hm2)->uart.num_instances ; i++) {
                if (!strcmp((*hm2)->uart.instance[i].name, name)) {return i;}
            }
        }
    }
    return -1;
}
EXPORT_SYMBOL_GPL(hm2_get_pktuart);
int hm2_get_pktuart(hostmot2_t** hm2, char *name){
    struct rtapi_list_head *ptr;
    int i;
    rtapi_list_for_each(ptr, &hm2_list) {
        *hm2 = rtapi_list_entry(ptr, hostmot2_t, list);
        if ((*hm2)->pktuart.num_instances > 0) {
            for (i = 0; i < (*hm2)->pktuart.num_instances ; i++) {
                if (!strcmp((*hm2)->pktuart.instance[i].name, name)) {return i;}
            }
        }
    }
    return -1;
}
EXPORT_SYMBOL_GPL(hm2_get_sserial);
// returns a pointer to a remote struct
hm2_sserial_remote_t *hm2_get_sserial(hostmot2_t** hm2, char *name){
   // returns inst * 64 + remote index
    struct rtapi_list_head *ptr;
    int i, j;
    rtapi_list_for_each(ptr, &hm2_list) {
        *hm2 = rtapi_list_entry(ptr, hostmot2_t, list);
        if ((*hm2)->sserial.num_instances > 0) {
            for (i = 0; i < (*hm2)->sserial.num_instances ; i++) {
                for (j = 0; j < (*hm2)->sserial.instance[i].num_remotes; j++){
                    if (strstr(name, (*hm2)->sserial.instance[i].remotes[j].name)) {
                        return &((*hm2)->sserial.instance[i].remotes[j]);
                    }
                }
            }
        }
    }
    return NULL;
}






// FIXME: the static automatic string makes this function non-reentrant
const char *hm2_get_general_function_name(int gtag) {
    switch (gtag) {
        case HM2_GTAG_WATCHDOG:        return "Watchdog";
        case HM2_GTAG_IOPORT:          return "IOPort";
        case HM2_GTAG_ENCODER:         return "Encoder";
        case HM2_GTAG_SSI:             return "SSI Encoder";
        case HM2_GTAG_BISS:            return "BiSS Encoder";
        case HM2_GTAG_FABS:            return "Fanuc Abs Encoder";
        case HM2_GTAG_RESOLVER:        return "Resolver";    
        case HM2_GTAG_STEPGEN:         return "StepGen";
        case HM2_GTAG_PWMGEN:          return "PWMGen";
        case HM2_GTAG_RCPWMGEN:        return "RCPWMGen";
        case HM2_GTAG_TRANSLATIONRAM:  return "TranslationRAM";
        case HM2_GTAG_TPPWM:           return "ThreePhasePWM";
        case HM2_GTAG_LED:             return "LED";
        case HM2_GTAG_MUXED_ENCODER:   return "Muxed Encoder";
        case HM2_GTAG_MUXED_ENCODER_SEL: return "Muxed Encoder Select";
        case HM2_GTAG_SMARTSERIALB:
        case HM2_GTAG_SMARTSERIAL:     return "Smart Serial Interface";
        case HM2_GTAG_BSPI:            return "Buffered SPI Interface";
        case HM2_GTAG_UART_RX:         return "UART Receive Channel";
        case HM2_GTAG_UART_TX:         return "UART Transmit Channel";
        case HM2_GTAG_PKTUART_RX:      return "PktUART Receive Channel";
        case HM2_GTAG_PKTUART_TX:      return "PktUART Transmit Channel";
        case HM2_GTAG_HM2DPLL:         return "Hostmot2 DPLL";
        case HM2_GTAG_INMUX:           return "InMux Input Mux";
        case HM2_GTAG_INM:             return "InM Input Module";
        case HM2_GTAG_OUTM:             return "OutM Output Module";
        case HM2_GTAG_XY2MOD:          return "xy2mod Galvo interface";
        case HM2_GTAG_DPAINTER:        return "Data Painter";
        case HM2_GTAG_SSR:             return "SSR";
        case HM2_GTAG_ONESHOT:         return "OneShot";
        case HM2_GTAG_PERIODM:         return "PeriodM";

        default: {
            static char unknown[100];
            rtapi_snprintf(unknown, 100, "(unknown-gtag-%d)", gtag);
            HM2_ERR_NO_LL("Firmware contains unknown function (gtag-%d)\n", gtag);
            return unknown;
        }
    }
}

int hm2_fabs_parse(hostmot2_t *hm2, char *token, int gtag){ 
    //adds the absolute encoder format strings to a list
    hm2_absenc_format_t *def;
    struct rtapi_list_head *ptr;
    int i = simple_strtol(token, &token, 0);
    if (i >= MAX_ABSENCS){
        HM2_ERR("Currently only %i absolute encoders are supported"
                " and you referred to an index of %i\n", MAX_ABSENCS, i);
        return -1;
    }
    if (*token != '='){
        HM2_ERR("The absolute encoder tag must be in the form "
                "[ssi / biss / fanuc]_chan_N=abcdefg where N is a number"
                " less than %i and abcdefg is a string specifying the "
                "bit fields\n",
                MAX_ABSENCS);
        return -1;
    }
    rtapi_list_for_each(ptr, &hm2->config.absenc_formats){
        def = rtapi_list_entry(ptr, hm2_absenc_format_t, list);
        if (i == def->index && gtag == def->gtag){
            HM2_ERR("Duplicate SSI/BISS/Fanuc definition. {Index %i for GTAG %i)"
                    "exiting\n", i, gtag);
            return -1;
        }
    }
    def = rtapi_kzalloc(sizeof(hm2_absenc_format_t), RTAPI_GFP_KERNEL);
    if (def == NULL){
        HM2_ERR("out of memory!\n");
        return -ENOMEM;
    }
    def->gtag = gtag;
    def->index = i;
    strncpy(def->string, ++token, MAX_ABSENC_LEN);
    rtapi_list_add(&def->list, &hm2->config.absenc_formats);
    return 0;
}

static int hm2_parse_config_string(hostmot2_t *hm2, char *config_string) {
    char **argv;
    int argc;
    int i,j;

    // default is to enable everything in the firmware
    hm2->config.num_encoders = -1;
    hm2->config.num_absencs = -1;
    hm2->absenc.chans = NULL;
    hm2->absenc.num_chans = 0;
    RTAPI_INIT_LIST_HEAD(&hm2->config.absenc_formats);
    hm2->config.num_resolvers = -1;
    hm2->config.num_pwmgens = -1;
    hm2->config.num_rcpwmgens = -1;
    hm2->config.num_tp_pwmgens = -1;
    hm2->config.num_sserials = -1;
    for(i=0;i<4;i++) for(j=0;j<8;j++) hm2->config.sserial_modes[i][j]='0';
    hm2->sserial.instance = NULL;
    hm2->config.num_stepgens = -1;
    hm2->config.stepgen_width = 2; // To avoid nasty surprises with table mode
    hm2->config.num_bspis = -1;
    hm2->config.num_uarts = -1;
    hm2->config.num_pktuarts = -1;
    hm2->config.num_dplls = -1;
    hm2->config.num_inmuxs = -1;
    hm2->config.num_inms = -1;
    hm2->config.num_xy2mods = -1;
    hm2->config.num_leds = -1;
    hm2->config.num_ssrs = -1;
    hm2->config.num_outms = -1;
    hm2->config.num_oneshots = -1;
    hm2->config.num_periodms = -1;
    hm2->config.enable_raw = 0;
    hm2->config.firmware = NULL;

    if (config_string == NULL) return 0;

    HM2_DBG("parsing config string \"%s\"\n", config_string);

    argv = rtapi_argv_split(RTAPI_GFP_KERNEL, config_string, &argc);
    if (argv == NULL) {
        HM2_ERR("out of memory while parsing config string\n");
        return -ENOMEM;
    }

    for (i = 0; i < argc; i ++) {
        char *token = argv[i];

        if (token == NULL) break;

        if (strncmp(token, "num_encoders=", 13) == 0) {
            token += 13;
            hm2->config.num_encoders = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "ssi_chan_", 9) == 0) {
            token += 9;
            if (hm2_fabs_parse(hm2, token, HM2_GTAG_SSI) )goto fail;
            
        } else if (strncmp(token, "biss_chan_", 10) == 0) {
            token += 10;
            if (hm2_fabs_parse(hm2, token, HM2_GTAG_BISS)) goto fail;
            
        } else if (strncmp(token, "fanuc_chan_", 11) == 0) {
            token += 11;
            if (hm2_fabs_parse(hm2, token, HM2_GTAG_FABS)) goto fail;

        } else if (strncmp(token, "num_resolvers=", 14) == 0) {
            token += 14;
            hm2->config.num_resolvers = simple_strtol(token, NULL, 0);
            
        } else if (strncmp(token, "num_pwmgens=", 12) == 0) {
            token += 12;
            hm2->config.num_pwmgens = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_rcpwmgens=", 14) == 0) {
            token += 14;
            hm2->config.num_rcpwmgens = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_inmuxs=", 11) == 0) {
            token += 11;
            hm2->config.num_inmuxs = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_inms=", 9) == 0) {
            token += 9;
            hm2->config.num_inms = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_outms=", 10) == 0) {
            token += 10;
            hm2->config.num_outms = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_oneshots=", 13) == 0) {
            token += 13;
            hm2->config.num_oneshots = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_periodms=", 13) == 0) {
            token += 13;
            hm2->config.num_periodms = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_ssrs=", 9) == 0) {
            token += 9;
            hm2->config.num_ssrs = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_xy2mods=", 12) == 0) {
            token += 12;
            hm2->config.num_xy2mods = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_3pwmgens=", 13) == 0) {
            token += 13;
            hm2->config.num_tp_pwmgens = simple_strtol(token, NULL, 0);
            
        } else if (strncmp(token, "sserial_port_", 13) == 0) {
            int i;
            int c = 0;
            int flag = 0;
            token += 13;
            i = *token - '0';
            token += 1;
            if (i < 0 || i > 3 || *token != '='){
                HM2_ERR("sserial_port tag must be in the form "
                        """sserial_port_N=0123xx23"" where N may be 0 to 3\n");
                goto fail;
            }
            for (token += 1 ; *token != 0; token++) {
                if ((*token >= '0' && *token <= '9') && c < 8) {
                    hm2->config.sserial_modes[i][c++] = *token;
                    flag = 1;
                }
                else if (*token == 'x' && c < 8) {
                    hm2->config.sserial_modes[i][c++] = *token;
                }
            }
            
            if (hm2->config.num_sserials == -1){
                hm2->config.num_sserials = 0;
            }
                
            if (i >= hm2->config.num_sserials && flag){
                hm2->config.num_sserials = i + 1;
            }

        } else if (strncmp(token, "num_stepgens=", 13) == 0) {
            token += 13;
            hm2->config.num_stepgens = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "stepgen_width=", 14) == 0) {
            token += 14;
            hm2->config.stepgen_width = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_bspis=", 10) == 0) {
            token += 10;
            hm2->config.num_bspis = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_uarts=", 10) == 0) {
            token += 10;
            hm2->config.num_uarts = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_pktuarts=", 13) == 0) {
            token += 13;
            hm2->config.num_pktuarts = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "num_leds=", 9) == 0) {
            token += 9;
            hm2->config.num_leds = simple_strtol(token, NULL, 0);
            
        } else if (strncmp(token, "num_dplls=", 10) == 0) {
            token += 10;
            hm2->config.num_dplls = simple_strtol(token, NULL, 0);

        } else if (strncmp(token, "enable_raw", 10) == 0) {
            hm2->config.enable_raw = 1;

        } else if (strncmp(token, "firmware=", 9) == 0) {
            // FIXME: we leak this in hm2_register
            hm2->config.firmware = rtapi_kstrdup(token + 9, RTAPI_GFP_KERNEL);
            if (hm2->config.firmware == NULL) {
                goto fail;
            }

        } else {
            HM2_ERR("invalid token in config string: \"%s\"\n", token);
            goto fail;
        }
    }

    HM2_DBG("final config:\n");
    HM2_DBG("    num_encoders=%d\n", hm2->config.num_encoders);
    HM2_DBG("    num_absencs=%d\n", hm2->config.num_absencs);
    HM2_DBG("    num_resolvers=%d\n", hm2->config.num_resolvers);
    HM2_DBG("    num_pwmgens=%d\n",  hm2->config.num_pwmgens);
    HM2_DBG("    num_rcpwmgens=%d\n",  hm2->config.num_rcpwmgens);
    HM2_DBG("    num_inmuxs=%d\n",  hm2->config.num_inmuxs);
    HM2_DBG("    num_inms=%d\n",  hm2->config.num_inms);
    HM2_DBG("    num_outms=%d\n",  hm2->config.num_outms);
    HM2_DBG("    num_oneshots=%d\n",  hm2->config.num_oneshots);
    HM2_DBG("    num_periodms=%d\n",  hm2->config.num_periodms);
    HM2_DBG("    num_ssrs=%d\n",  hm2->config.num_ssrs);
    HM2_DBG("    num_xy2mods=%d\n",  hm2->config.num_xy2mods);
    HM2_DBG("    num_3pwmgens=%d\n", hm2->config.num_tp_pwmgens);
    HM2_DBG("    sserial_port_0=%8.8s\n"
            "                    sserial_port_1=%8.8s\n"
            "                    sserial_port_2=%8.8s\n"
            "                    sserial_port_3=%8.8s\n", 
            hm2->config.sserial_modes[0],
            hm2->config.sserial_modes[1], 
            hm2->config.sserial_modes[2],
            hm2->config.sserial_modes[3]);
    HM2_DBG("    num_stepgens=%d\n", hm2->config.num_stepgens);
    HM2_DBG("    num_bspis=%d\n", hm2->config.num_bspis);
    HM2_DBG("    num_uarts=%d\n", hm2->config.num_uarts);
    HM2_DBG("    num_pktuarts=%d\n", hm2->config.num_pktuarts);
    HM2_DBG("    enable_raw=%d\n",   hm2->config.enable_raw);
    HM2_DBG("    firmware=%s\n",   hm2->config.firmware ? hm2->config.firmware : "(NULL)");

    rtapi_argv_free(argv);
    return 0;

fail:
    rtapi_argv_free(argv);
    return -EINVAL;
}




//
// functions for dealing with the idrom
//

static void hm2_print_idrom(hostmot2_t *hm2) {
    HM2_PRINT("IDRom:\n");

    HM2_PRINT("    IDRom Type: 0x%08X\n", hm2->idrom.idrom_type);

    HM2_PRINT("    Offset to Modules: 0x%08X\n", hm2->idrom.offset_to_modules); 
    HM2_PRINT("    Offset to Pin Description: 0x%08X\n", hm2->idrom.offset_to_pin_desc); 

    HM2_PRINT(
        "    Board Name: %c%c%c%c%c%c%c%c\n",
        hm2->idrom.board_name[0],
        hm2->idrom.board_name[1],
        hm2->idrom.board_name[2],
        hm2->idrom.board_name[3],
        hm2->idrom.board_name[4],
        hm2->idrom.board_name[5],
        hm2->idrom.board_name[6],
        hm2->idrom.board_name[7]
    );

    HM2_PRINT("    FPGA Size: %u\n", hm2->idrom.fpga_size);
    HM2_PRINT("    FPGA Pins: %u\n", hm2->idrom.fpga_pins);
    HM2_PRINT("    Port Width: %u\n", hm2->idrom.port_width);

    HM2_PRINT("    IO Ports: %u\n", hm2->idrom.io_ports);
    HM2_PRINT("    IO Width: %u\n", hm2->idrom.io_width);

    HM2_PRINT(
        "    Clock Low: %d Hz (%d KHz, %d MHz)\n",
        hm2->idrom.clock_low,
        (hm2->idrom.clock_low / 1000),
        (hm2->idrom.clock_low / (1000 * 1000))
    );

    HM2_PRINT(
        "    Clock High: %d Hz (%d KHz, %d MHz)\n",
        hm2->idrom.clock_high,
        (hm2->idrom.clock_high / 1000),
        (hm2->idrom.clock_high / (1000 * 1000))
    );

    HM2_PRINT("    Instance Stride 0: 0x%08X\n", hm2->idrom.instance_stride_0);
    HM2_PRINT("    Instance Stride 1: 0x%08X\n", hm2->idrom.instance_stride_1);

    HM2_PRINT("    Register Stride 0: 0x%08X\n", hm2->idrom.register_stride_0);
    HM2_PRINT("    Register Stride 1: 0x%08X\n", hm2->idrom.register_stride_1);
}




static int hm2_read_idrom(hostmot2_t *hm2) {
    rtapi_u32 read_data;

    //
    // find the idrom offset
    //

    if (!hm2->llio->read(hm2->llio, HM2_ADDR_IDROM_OFFSET, &read_data, 4)) {
        HM2_ERR("error reading IDROM Offset\n");
        return -EIO;
    }
    hm2->idrom_offset = read_data & 0xFFFF;

    //
    // first read in the idrom type to make sure we know how to deal with it
    //


    if (!hm2->llio->read(hm2->llio, hm2->idrom_offset, &hm2->idrom.idrom_type, sizeof(hm2->idrom.idrom_type))) {
        HM2_ERR("error reading IDROM type\n");
        return -EIO;
    }
    if (
        (hm2->idrom.idrom_type != 2) 
        && (hm2->idrom.idrom_type != 3)
    ) {
        HM2_ERR("invalid IDROM type %d, expected 2 or 3, aborting load\n", hm2->idrom.idrom_type);
        return -EINVAL;
    }


    //
    // ok, read in the whole thing
    //


    if (!hm2->llio->read(hm2->llio, hm2->idrom_offset, &hm2->idrom, sizeof(hm2->idrom))) {
        HM2_ERR("error reading IDROM\n");
        return -EIO;
    }


    //
    // verify the idrom we read
    //

    if (hm2->idrom.port_width != hm2->llio->pins_per_connector) {
        HM2_ERR("invalid IDROM PortWidth %d, this board has %d pins per connector, aborting load\n", hm2->idrom.port_width, hm2->llio->pins_per_connector);
        hm2_print_idrom(hm2);
        return -EINVAL;
    }

    if (hm2->idrom.io_width != (hm2->idrom.io_ports * hm2->idrom.port_width)) {
        HM2_ERR(
            "IDROM IOWidth is %d, but IDROM IOPorts is %d and IDROM PortWidth is %d (inconsistent firmware), aborting driver load\n",
            hm2->idrom.io_width,
            hm2->idrom.io_ports,
            hm2->idrom.port_width
        );
        return -EINVAL;
    }

    if (hm2->idrom.io_ports != hm2->llio->num_ioport_connectors) {
        HM2_ERR(
            "IDROM IOPorts is %d but llio num_ioport_connectors is %d, driver and firmware are inconsistent, aborting driver load\n",
            hm2->idrom.io_ports,
            hm2->llio->num_ioport_connectors
        );
        return -EINVAL;
    }

    if (hm2->idrom.io_width > HM2_MAX_PIN_DESCRIPTORS) {
        HM2_ERR(
            "IDROM IOWidth is %d but max is %d, aborting driver load\n",
            hm2->idrom.io_width,
            HM2_MAX_PIN_DESCRIPTORS
        );
        return -EINVAL;
    }

    if (hm2->idrom.clock_low < 1e6) {
        HM2_ERR(
            "IDROM ClockLow is %d, that's too low, aborting driver load\n",
            hm2->idrom.clock_low
        );
        return -EINVAL;
    }

    if (hm2->idrom.clock_high < 1e6) {
        HM2_ERR(
            "IDROM ClockHigh is %d, that's too low, aborting driver load\n",
            hm2->idrom.clock_high
        );
        return -EINVAL;
    }

    if (debug_idrom) {
        hm2_print_idrom(hm2);
    }

    return 0;
}




// reads the Module Descriptors
// doesn't do any validation or parsing or anything, that's in hm2_parse_module_descriptors(), which comes next
static int hm2_read_module_descriptors(hostmot2_t *hm2) {
    int addr = hm2->idrom_offset + hm2->idrom.offset_to_modules;

    for (
        hm2->num_mds = 0;
        hm2->num_mds < HM2_MAX_MODULE_DESCRIPTORS;
        hm2->num_mds ++, addr += 12
    ) {
        rtapi_u32 d[3];
        hm2_module_descriptor_t *md = &hm2->md[hm2->num_mds];

        if (!hm2->llio->read(hm2->llio, addr, d, 12)) {
            HM2_ERR("error reading Module Descriptor %d (at 0x%04x)\n", hm2->num_mds, addr);
            return -EIO;
        }

        md->gtag = d[0] & 0x000000FF;
        if (md->gtag == 0) {
            // done
            return 0;
        }

        md->version   = (d[0] >>  8) & 0x000000FF;
        md->clock_tag = (d[0] >> 16) & 0x000000FF;
        md->instances = (d[0] >> 24) & 0x000000FF;

        if (md->clock_tag == 1) {
            md->clock_freq = hm2->idrom.clock_low;
        } else if (md->clock_tag == 2) {
            md->clock_freq = hm2->idrom.clock_high;
        } else {
            HM2_ERR("Module Descriptor %d (at 0x%04x) has invalid ClockTag %d\n", hm2->num_mds, addr, md->clock_tag);
            return -EINVAL;
        }

        md->base_address = (d[1] >> 00) & 0x0000FFFF;
        md->num_registers = (d[1] >> 16) & 0x000000FF;

        md->register_stride = (d[1] >> 24) & 0x0000000F;
        if (md->register_stride == 0) {
            md->register_stride = hm2->idrom.register_stride_0;
        } else if (md->register_stride == 1) {
            md->register_stride = hm2->idrom.register_stride_1;
        } else {
            HM2_ERR("Module Descriptor %d (at 0x%04x) has invalid RegisterStride %d\n", hm2->num_mds, addr, md->register_stride);
            return -EINVAL;
        }

        md->instance_stride = (d[1] >> 28) & 0x0000000F;
        if (md->instance_stride == 0) {
            md->instance_stride = hm2->idrom.instance_stride_0;
        } else if (md->instance_stride == 1) {
            md->instance_stride = hm2->idrom.instance_stride_1;
        } else {
            HM2_ERR("Module Descriptor %d (at 0x%04x) has invalid InstanceStride %d\n", hm2->num_mds, addr, md->instance_stride);
            return -EINVAL;
        }

        md->multiple_registers = d[2];

        if (debug_module_descriptors) {
            HM2_PRINT("Module Descriptor %d at 0x%04X:\n", hm2->num_mds, addr);
            HM2_PRINT("    General Function Tag: %d (%s)\n", md->gtag, hm2_get_general_function_name(md->gtag));
            HM2_PRINT("    Version: %d\n", md->version);
            HM2_PRINT("    Clock Tag: %d (%s MHz)\n", md->clock_tag, hm2_hz_to_mhz(md->clock_freq));
            HM2_PRINT("    Instances: %d\n", md->instances);
            HM2_PRINT("    Base Address: 0x%04X\n", md->base_address);
            HM2_PRINT("    -- Num Registers: %d\n", md->num_registers);
            HM2_PRINT("    Register Stride: 0x%08X\n", md->register_stride);
            HM2_PRINT("    -- Instance Stride: 0x%08X\n", md->instance_stride);
            HM2_PRINT("    -- Multiple Registers: 0x%08X\n", md->multiple_registers);
        }
    }

    return 0;
}




//
// Here comes the code to "parse" the Module Descriptors, turn them into
// internal-to-the-driver representations, and export those internal
// representations to the HAL.
//
// There's a general function that walks the MD list and tries to parse
// each MD in turn, and there's a special function to parse each GTag
// (aka Function)
//
// The per-Module parsers return the number of instances accepted, which
// may be less than the number of instances available, or even 0, if the
// user has disabled some instances using modparams.  The per-Module
// parsers return -1 on error, which causes the module load to fail.
//


int hm2_md_is_consistent_or_complain(
    hostmot2_t *hm2,
    int md_index,
    rtapi_u8 version,
    rtapi_u8 num_registers,
    rtapi_u32 instance_stride,
    rtapi_u32 multiple_registers
) {
    hm2_module_descriptor_t *md = &hm2->md[md_index];

    if (hm2_md_is_consistent(hm2, md_index, version, num_registers, instance_stride, multiple_registers)) return 1;

    HM2_ERR(
        "inconsistent Module Descriptor for %s, not loading driver\n",
        hm2_get_general_function_name(md->gtag)
    );

    HM2_ERR(
        "    Version = %d, expected %d\n",
        md->version,
        version
    );

    HM2_ERR(
        "    NumRegisters = %d, expected %d\n",
        md->num_registers,
        num_registers
    );

    HM2_ERR(
        "    InstanceStride = 0x%08X, expected 0x%08X\n",
        md->instance_stride,
        instance_stride
    );

    HM2_ERR(
        "    MultipleRegisters = 0x%08X, expected 0x%08X\n",
        md->multiple_registers,
        multiple_registers
    );

    return 0;
}


int hm2_md_is_consistent(
    hostmot2_t *hm2,
    int md_index,
    rtapi_u8 version,
    rtapi_u8 num_registers,
    rtapi_u32 instance_stride,
    rtapi_u32 multiple_registers
) {
    hm2_module_descriptor_t *md = &hm2->md[md_index];

    if (
        (md->num_registers == num_registers)
        && (md->version == version)
        && (md->instance_stride == instance_stride)
        && (md->multiple_registers == multiple_registers)
    ) {
        return 1;
    }

    return 0;
}




static int hm2_parse_module_descriptors(hostmot2_t *hm2) {
    int md_index, md_accepted;
    
    hm2->dpll_module_present = 0;
    for (md_index = 0; md_index < hm2->num_mds; md_index ++) {
        hm2_module_descriptor_t *md = &hm2->md[md_index];

        if (md->gtag == HM2_GTAG_HM2DPLL) {
            hm2->dpll_module_present = 1;
            break;
        } else if (md->gtag == 0) {
            break;
        }
    }

    // Run through once looking for IO Ports in case other modules
    // need them
    for (md_index = 0; md_index < hm2->num_mds; md_index ++) {
        hm2_module_descriptor_t *md = &hm2->md[md_index];

        if (md->gtag != HM2_GTAG_IOPORT) {
            continue;
        }

        md_accepted = hm2_ioport_parse_md(hm2, md_index);

        if ((*hm2->llio->io_error) != 0) {
            HM2_ERR("IO error while parsing Module Descriptor %d\n", md_index);
            return -EIO;
        }

        if (md_accepted >= 0)  {
            HM2_INFO(
                     "MD %d: %dx %s v%d: accepted, using %d\n",
                     md_index,
                     md->instances,
                     hm2_get_general_function_name(md->gtag),
                     md->version,
                     md_accepted
                     );
        } else {
            HM2_ERR("failed to parse Module Descriptor %d\n", md_index);
            return md_accepted;
        }
    }

    // Now look for the other modules. 
    for (md_index = 0; md_index < hm2->num_mds; md_index ++) {
        hm2_module_descriptor_t *md = &hm2->md[md_index];

        if (md->gtag == 0) {
            // done
            return 0;
        }

        md_accepted = 0;  // will be set by the switch

        switch (md->gtag) {

            case HM2_GTAG_IOPORT:
                // We silently ignore IOPorts here because we already
                // handled them above.
                continue;

            case HM2_GTAG_ENCODER:
            case HM2_GTAG_MUXED_ENCODER:
                md_accepted = hm2_encoder_parse_md(hm2, md_index);
                break;
            
            case HM2_GTAG_SSI:
            case HM2_GTAG_BISS:
            case HM2_GTAG_FABS:
                md_accepted = hm2_absenc_parse_md(hm2, md_index);
                break;
                
            case HM2_GTAG_RESOLVER:
                md_accepted = hm2_resolver_parse_md(hm2, md_index);
                break;

            case HM2_GTAG_PWMGEN:
                md_accepted = hm2_pwmgen_parse_md(hm2, md_index);
                break;

            case HM2_GTAG_STEPGEN:
                md_accepted = hm2_stepgen_parse_md(hm2, md_index);
                break;

            case HM2_GTAG_WATCHDOG:
                md_accepted = hm2_watchdog_parse_md(hm2, md_index);
                break;

            case HM2_GTAG_TPPWM:
                md_accepted = hm2_tp_pwmgen_parse_md(hm2, md_index);
                break;

            case HM2_GTAG_SMARTSERIALB:
            case HM2_GTAG_SMARTSERIAL:
                md_accepted = hm2_sserial_parse_md(hm2, md_index);
                break;
                
            case HM2_GTAG_BSPI:
                md_accepted = hm2_bspi_parse_md(hm2, md_index);
                break;
                
            case HM2_GTAG_UART_RX:
            case HM2_GTAG_UART_TX:
                md_accepted = hm2_uart_parse_md(hm2, md_index);
                break;

            case HM2_GTAG_PKTUART_RX:
            case HM2_GTAG_PKTUART_TX:
                md_accepted = hm2_pktuart_parse_md(hm2, md_index);
                break;

            case HM2_GTAG_HM2DPLL:
                md_accepted = hm2_dpll_parse_md(hm2, md_index);
                break;
 
           case HM2_GTAG_INMUX:
                md_accepted = hm2_inmux_parse_md(hm2, md_index);
                break;

           case HM2_GTAG_INM:
                md_accepted = hm2_inm_parse_md(hm2, md_index);
                break;
 
          case HM2_GTAG_XY2MOD:
                md_accepted = hm2_xy2mod_parse_md(hm2, md_index);
                break;
                
            case HM2_GTAG_LED:
                md_accepted = hm2_led_parse_md(hm2, md_index);
                break;

            case HM2_GTAG_SSR:
                md_accepted = hm2_ssr_parse_md(hm2, md_index);
                break;

            case HM2_GTAG_OUTM:
                md_accepted = hm2_outm_parse_md(hm2, md_index);
                break;

            case HM2_GTAG_ONESHOT:
                md_accepted = hm2_oneshot_parse_md(hm2, md_index);
                break;

            case HM2_GTAG_PERIODM:
                md_accepted = hm2_periodm_parse_md(hm2, md_index);
                break;

          case HM2_GTAG_RCPWMGEN:
                md_accepted = hm2_rcpwmgen_parse_md(hm2, md_index);
                break;

            default:
                HM2_WARN(
                    "MD %d: %dx %s v%d: ignored\n",
                    md_index,
                    md->instances,
                    hm2_get_general_function_name(md->gtag),
                    md->version
                );
                continue;

        }

        if ((*hm2->llio->io_error) != 0) {
            HM2_ERR("IO error while parsing Module Descriptor %d\n", md_index);
            return -EIO;
        }

        if (md_accepted >= 0)  {
            HM2_INFO(
                "MD %d: %dx %s v%d: accepted, using %d\n",
                md_index,
                md->instances,
                hm2_get_general_function_name(md->gtag),
                md->version,
                md_accepted
            );
        } else {
            HM2_ERR("failed to parse Module Descriptor %d\n", md_index);
            return md_accepted;
        }

    }    
    
    // on any one run through the absenc driver there is no way to know if 
    // it is the last time, so we need to  trigger this from somewhere that 
    // does know that it has stopped calling the sub-driver. 
    if (hm2->absenc.num_chans > 0){
         if (hm2_absenc_register_tram(hm2)){
             HM2_ERR("Failed to register TRAM for absolute encoders\n");
             return -EINVAL;
         }
    }
                                           
    return 0;  // success!
}




//
// These functions free all the memory rtapi_kmalloc'ed in hm2_parse_module_descriptors()
//

static void hm2_cleanup(hostmot2_t *hm2) {
    // clean up the Pins, if they're initialized
    if (hm2->pin != NULL) rtapi_kfree(hm2->pin);

    // clean up the Modules
    hm2_ioport_cleanup(hm2);
    hm2_encoder_cleanup(hm2);
    hm2_absenc_cleanup(hm2);
    hm2_resolver_cleanup(hm2);
    hm2_watchdog_cleanup(hm2);
    hm2_pwmgen_cleanup(hm2);
    hm2_inmux_cleanup(hm2);
    hm2_inm_cleanup(hm2);
    hm2_xy2mod_cleanup(hm2);
    hm2_tp_pwmgen_cleanup(hm2);
    hm2_led_cleanup(hm2);
    hm2_sserial_cleanup(hm2);
    hm2_bspi_cleanup(hm2);
    hm2_ssr_cleanup(hm2);
    hm2_outm_cleanup(hm2);
    hm2_oneshot_cleanup(hm2);
    hm2_periodm_cleanup(hm2);
    hm2_rcpwmgen_cleanup(hm2);

    // free all the tram entries
    hm2_tram_cleanup(hm2);
}




void hm2_print_modules(hostmot2_t *hm2) {
    hm2_encoder_print_module(hm2);
    hm2_absenc_print_module(hm2);
    hm2_resolver_print_module(hm2);
    hm2_pwmgen_print_module(hm2);
    hm2_tp_pwmgen_print_module(hm2);
    hm2_sserial_print_module(hm2);
    hm2_stepgen_print_module(hm2);
    hm2_bspi_print_module(hm2);
    hm2_ioport_print_module(hm2);
    hm2_ssr_print_module(hm2);
    hm2_outm_print_module(hm2);
    hm2_oneshot_print_module(hm2);
    hm2_periodm_print_module(hm2);
    hm2_watchdog_print_module(hm2);
    hm2_inmux_print_module(hm2);
    hm2_inm_print_module(hm2);
    hm2_xy2mod_print_module(hm2);
    hm2_rcpwmgen_print_module(hm2);
}




//
// register and unregister, for the low-level I/O drivers to add and remove boards to this hostmot2 driver
//


static void hm2_release_device(struct rtapi_device *dev) {
    (void)dev;
    // nothing to do here
}

static int dummy_queue_write(hm2_lowlevel_io_t *this, rtapi_u32 addr,
        const void *buffer, int size) {
    if(size >= 0) return this->write(this, addr, buffer, size);
    return 1; // success
}

static int dummy_queue_read(hm2_lowlevel_io_t *this, rtapi_u32 addr,
        void *buffer, int size) {
    if(size >= 0) return this->read(this, addr, buffer, size);
    return 1; // success
}

EXPORT_SYMBOL_GPL(hm2_register);

int hm2_register(hm2_lowlevel_io_t *llio, char *config_string) {
    int r;
    hostmot2_t *hm2;

    //
    // first a pile of sanity checks
    //

    if (llio == NULL) {
        HM2_ERR_NO_LL("NULL llio passed in\n");
        return -EINVAL;
    }


    //
    // verify llio->name
    //

    {
        int i;

        for (i = 0; i < HAL_NAME_LEN+1; i ++) {
            if (llio->name[i] == '\0') break;
            if (!isprint(llio->name[i])) {
                HM2_ERR_NO_LL("invalid llio name passed in (contains non-printable character)\n");
                return -EINVAL;
            }
        }
        if (i == HAL_NAME_LEN+1) {
            HM2_ERR_NO_LL("invalid llio name passed in (not NULL terminated)\n");
            return -EINVAL;
        }
        if (i == 0) {
            HM2_ERR_NO_LL("invalid llio name passed in (zero length)\n");
            return -EINVAL;
        }
    }


    //
    // verify llio ioport connector names
    //

    if ((llio->num_ioport_connectors < 1) || (llio->num_ioport_connectors > ANYIO_MAX_IOPORT_CONNECTORS)) {
        HM2_ERR_NO_LL("llio reports invalid number of I/O connectors (%d)\n", llio->num_ioport_connectors);
        return -EINVAL;
    }

    {
        for (unsigned port = 0; port < llio->num_ioport_connectors; port ++) {
            int i;

            if (llio->ioport_connector_name[port] == NULL) {
                HM2_ERR_NO_LL("llio ioport connector name %u is NULL\n", port);
                return -EINVAL;
            }

            for (i = 0; i < HAL_NAME_LEN+1; i ++) {
                if (llio->ioport_connector_name[port][i] == '\0') break;
                if (!isprint(llio->ioport_connector_name[port][i])) {
                    HM2_ERR_NO_LL("invalid llio ioport connector name %u passed in (contains non-printable character)\n", port);
                    return -EINVAL;
                }
            }
            if (i == HAL_NAME_LEN+1) {
                HM2_ERR_NO_LL("invalid llio ioport connector name %u passed in (not NULL terminated)\n", port);
                return -EINVAL;
            }
            if (i == 0) {
                HM2_ERR_NO_LL("invalid llio ioport connector name %u passed in (zero length)\n", port);
                return -EINVAL;
            }
        }
    }


    //
    // verify llio functions
    //

    if (llio->read == NULL) {
        HM2_ERR_NO_LL("NULL llio->read passed in\n");
        return -EINVAL;
    }

    if (llio->write == NULL) {
        HM2_ERR_NO_LL("NULL llio->write passed in\n");
        return -EINVAL;
    }

    if (!llio->queue_write) {
        llio->queue_write = dummy_queue_write;
    }

    if (!llio->queue_read) {
        llio->queue_read = dummy_queue_read;
    }

    //
    // make a hostmot2_t struct to represent this device
    //

    hm2 = rtapi_kmalloc(sizeof(hostmot2_t), RTAPI_GFP_KERNEL);
    if (hm2 == NULL) {
        HM2_PRINT_NO_LL("out of memory!\n");
        return -ENOMEM;
    }

    memset(hm2, 0, sizeof(hostmot2_t));

    hm2->llio = llio;
    hm2->use_serial_numbers = use_serial_numbers;
    hm2->sserial.baudrate = sserial_baudrate;

    RTAPI_INIT_LIST_HEAD(&hm2->tram_read_entries);
    RTAPI_INIT_LIST_HEAD(&hm2->tram_write_entries);

    // tentatively add it to the hm2 list
    rtapi_list_add_tail(&hm2->list, &hm2_list);


    //
    // parse the config string
    //

    r = hm2_parse_config_string(hm2, config_string);
    if (r != 0) {
        goto fail0;
    }



    // NOTE: program_fpga will be NULL for 6i25 and 5i25 (and future cards 
    // with EPROM firmware, probably. 

    if ((llio->program_fpga != NULL) && (hm2->config.firmware == NULL)) {
        HM2_PRINT_NO_LL("no firmware specified in config modparam!  the board had better have firmware configured already, or this won't work\n");
    }

    
    //
    // if programming of the fpga is supported by the board and the user
    // requested a firmware file, fetch it from userspace and program
    // the board
    //

    if ((llio->program_fpga != NULL) && (hm2->config.firmware != NULL)) {
        const struct rtapi_firmware *fw;
        bitfile_t bitfile;
        struct rtapi_device dev;

        // check firmware name length
        if (strlen(hm2->config.firmware) > FIRMWARE_NAME_MAX) {
            HM2_ERR("requested firmware name '%s' is too long (max length is %d)\n", hm2->config.firmware, FIRMWARE_NAME_MAX);
            r = -ENAMETOOLONG;
            goto fail0;
        }

        memset(&dev, '\0', sizeof(dev));
        rtapi_dev_set_name(&dev, "%s", hm2->llio->name);
        dev.release = hm2_release_device;
        r = rtapi_device_register(&dev);
        if (r != 0) {
            HM2_ERR("error with device_register\n");
            goto fail0;
        }

        r = rtapi_request_firmware(&fw, hm2->config.firmware, &dev);
        rtapi_device_unregister(&dev);
        if (r == -ENOENT) {
            HM2_ERR("firmware %s not found\n", hm2->config.firmware);
            HM2_ERR("install the package containing the firmware.\n");
            goto fail0;
        }
        if (r != 0) {
            HM2_ERR("request for firmware %s failed, aborting hm2_register (r=%d)\n", hm2->config.firmware, r);
            goto fail0;
        }

        r = bitfile_parse_and_verify(fw, &bitfile);
        if (r != 0) {
            HM2_ERR("firmware %s fails verification, aborting hm2_register\n", hm2->config.firmware);
            rtapi_release_firmware(fw);
            goto fail0;
        }

        HM2_INFO("firmware %s:\n", hm2->config.firmware);
        HM2_INFO("    %s %s %s\n", bitfile.a.data, bitfile.c.data, bitfile.d.data);
        HM2_INFO("    Part Name: %s\n", bitfile.b.data);
        HM2_INFO("    FPGA Config: %zu bytes\n", bitfile.e.size);

        if (llio->fpga_part_number == NULL) {
            HM2_ERR("llio did not provide an FPGA part number, cannot verify firmware part number\n");
        } else {
            if (strcmp(llio->fpga_part_number, (char*)bitfile.b.data) != 0) {
                HM2_ERR(
                    "board has FPGA '%s', but the firmware in %s is for FPGA '%s'\n",
                    llio->fpga_part_number,
                    hm2->config.firmware,
                    bitfile.b.data
                );
                rtapi_release_firmware(fw);
                r = -EINVAL;
                goto fail0;
            }
        }

        if (llio->reset != NULL) {
            r = llio->reset(llio);
            if (r != 0) {
                rtapi_release_firmware(fw);
                HM2_ERR("failed to reset fpga, aborting hm2_register\n");
                goto fail0;
            }
        }

        r = llio->program_fpga(llio, &bitfile);
        rtapi_release_firmware(fw);
        if (r != 0) {
            HM2_ERR("failed to program fpga, aborting hm2_register\n");
            goto fail0;
        }
    }


    //
    // export a parameter to deal with communication errors
    // NOTE: this is really only useful for EPP boards, PCI doesn't use it
    //

    {
        int r;
        char name[HAL_NAME_LEN + 1];

        llio->io_error = (hal_bit_t *)hal_malloc(sizeof(hal_bit_t));
        if (llio->io_error == NULL) {
            HM2_ERR("out of memory!\n");
            r = -ENOMEM;
            goto fail0;
        }

        (*llio->io_error) = 0;

        rtapi_snprintf(name, sizeof(name), "%s.io_error", llio->name);
        r = hal_param_bit_new(name, HAL_RW, llio->io_error, llio->comp_id);
        if (r < 0) {
            HM2_ERR("error adding param '%s', aborting\n", name);
            r = -EINVAL;
            goto fail0;
        }
    }

    HM2_PRINT("Low Level init %s\n", HM2_VERSION);

    //
    // read & verify FPGA firmware IOCookie
    //
 
    {
        uint32_t cookie;

        if (!llio->read(llio, HM2_ADDR_IOCOOKIE, &cookie, 4)) {
            HM2_ERR("error reading hm2 cookie\n");
            r = -EIO;
            goto fail0;
        }

        if (cookie != HM2_IOCOOKIE) {
            HM2_ERR("invalid cookie, got 0x%08X, expected 0x%08X\n", cookie, HM2_IOCOOKIE);
            HM2_ERR("FPGA failed to initialize, or unexpected firmware?\n");
            r = -EINVAL;
            goto fail0;
        }
    }


    //
    // read & verify FPGA firmware ConfigName
    //

    {
        char name[9];  // read 8, plus 1 for the NULL

        if (!llio->read(llio, HM2_ADDR_CONFIGNAME, name, 8)) {
            HM2_ERR("error reading HM2 Config Name\n");
            r = -EIO;
            goto fail0;
        }
        name[8] = '\0';

        if (strncmp(name, HM2_CONFIGNAME, 9) != 0) {
            int i;
            // Prevent non-printable characters messing up the terminal
            for (i = 0; i < 8; i++) {
                if (!isprint(name[i]))
                    name[i] = '?';
            }
            HM2_ERR("invalid config name, got '%s', expected '%s'\n", name, HM2_CONFIGNAME);
            r = -EINVAL;
            goto fail0;
        }
    }


    //
    // Looks like HostMot2 alright, go ahead an initialize it
    //


    //
    // read the IDROM Header, the Pin Descriptors, and the Module Descriptors
    //

    r = hm2_read_idrom(hm2);
    if (r != 0) {
        goto fail0;
    }

    r = hm2_read_pin_descriptors(hm2);
    if (r != 0) {
        goto fail0;
    }

    r = hm2_read_module_descriptors(hm2);
    if (r != 0) {
        goto fail0;
    }


    //
    // process the Module Descriptors and initialize the HostMot2 Modules found
    //

    r = hm2_parse_module_descriptors(hm2);
    if (r != 0) {
        goto fail1;
    }


    //
    // allocate memory for the PC's copy of the HostMot2's registers
    //

    r = hm2_allocate_tram_regions(hm2);
    if (r < 0) {
        HM2_ERR("error allocating memory for HostMot2 registers\n");
        goto fail1;
    }


    //
    // At this point, all register buffers have been allocated.
    // All non-TRAM register buffers except for the IOPort registers have
    // been initialized. All HAL objects except for the GPIOs have been
    // allocated & exported to HAL.
    //


    //
    // set IOPorts based on detected, enabled Modules
    // all in-use module instances get all their pins, the unused pins are left as GPIOs
    //

    hm2_configure_pins(hm2);

    r = hm2_ioport_gpio_export_hal(hm2);
    if (r != 0) {
        goto fail1;
    }


    //
    // the "raw" interface lets you peek and poke the HostMot2 registers from HAL
    //

    r = hm2_raw_setup(hm2);
    if (r != 0) {
        goto fail1;
    }


    //
    // At this point, all non-TRAM register buffers have been initialized
    // and all HAL objects have been allocated and exported to HAL.
    //


    //
    // Write out all the non-TRAM register buffers to the FPGA.
    //
    // This initializes the FPGA to the default load-time state chosen by
    // the hostmot2 driver.  Users can change the state later via HAL.
    //

    hm2_force_write(hm2);


    //
    // read the TRAM one first time
    //

    r = hm2_tram_read(hm2);
    if (r != 0) {
        goto fail1;
    }

    hm2_queue_read(hm2);
    if (r != 0) {
        goto fail1;
    }

    r = hm2_finish_read(hm2);
    if (r != 0) {
        goto fail1;
    }

    // set HAL gpio input pins based on FPGA pins
    hm2_ioport_gpio_process_tram_read(hm2);

    // initialize encoder count & pos to 0
    hm2_encoder_tram_init(hm2);
    hm2_encoder_process_tram_read(hm2, 1000);

    // initialize step accumulator, hal count & position to 0
    hm2_stepgen_tram_init(hm2);
    hm2_stepgen_process_tram_read(hm2, 1000);


    //
    // write the TRAM one first time
    //

    // set gpio output pins (tho there should be none yet) based on their HAL objects
    hm2_ioport_gpio_tram_write_init(hm2);
    hm2_ioport_gpio_prepare_tram_write(hm2);

    // tell stepgen not to move
    // NOTE: the 1000 is the fake "amount of time since function last ran"
    hm2_stepgen_prepare_tram_write(hm2, 1000);

    // tell pwmgen not to move
    hm2_pwmgen_prepare_tram_write(hm2);

    r = hm2_tram_write(hm2);
    if (r != 0) {
        goto fail1;
    }

    r = hm2_finish_write(hm2);
    if (r != 0) {
        goto fail1;
    }

    //
    // final check for comm errors
    //

    if ((*hm2->llio->io_error) != 0) {
        HM2_ERR("comm errors while initializing firmware!\n");
        goto fail1;
    }


    //
    // all initialized show what pins & modules we ended up with
    //

    hm2_print_pin_usage(hm2);

    if (debug_modules) {
        HM2_PRINT("HM2 Modules used:\n");
        hm2_print_modules(hm2);
    }


    //
    // export the main read/write functions
    //

    {
        if(hm2->llio->split_read) {
            r = hal_export_functf(hm2_read_request, hm2, 1, 0, hm2->llio->comp_id, "%s.read-request", hm2->llio->name);
            if (r != 0) {
                HM2_ERR("error %d exporting read function %s.read-request\n", r, hm2->llio->name);
                r = -EINVAL;
                goto fail1;
            }
        }
        r = hal_export_functf(hm2_read, hm2, 1, 0, hm2->llio->comp_id, "%s.read", hm2->llio->name);
        if (r != 0) {
            HM2_ERR("error %d exporting read function %s.read\n", r, hm2->llio->name);
            r = -EINVAL;
            goto fail1;
        }

        r = hal_export_functf(hm2_write, hm2, 1, 0, hm2->llio->comp_id, "%s.write", hm2->llio->name);
        if (r != 0) {
            HM2_ERR("error %d exporting write function %s.write\n", r, hm2->llio->name);
            r = -EINVAL;
            goto fail1;
        }
    }


    //
    // if the llio claims to be threadsafe, export the gpio read/write functions
    //

    if (hm2->llio->threadsafe) {
        r = hal_export_functf(hm2_read_gpio, hm2, 1, 0, hm2->llio->comp_id, "%s.read_gpio", hm2->llio->name);
        if (r != 0) {
            HM2_ERR("error %d exporting gpio_read function %s.read_gpio\n", r, hm2->llio->name);
            r = -EINVAL;
            goto fail1;
        }

        r = hal_export_functf(hm2_write_gpio, hm2, 1, 0, hm2->llio->comp_id, "%s.write_gpio", hm2->llio->name);
        if (r != 0) {
            HM2_ERR("error %d exporting gpio_write function %s.write_gpio\n", r, hm2->llio->name);
            r = -EINVAL;
            goto fail1;
        }
    }


    //
    // found one!
    //

    HM2_PRINT("registered\n");

    return 0;


fail1:
    hm2_cleanup(hm2);  // undoes the rtapi_kmallocs from hm2_parse_module_descriptors()

fail0:
    rtapi_list_del(&hm2->list);
    rtapi_kfree(hm2);
    return r;
}




EXPORT_SYMBOL_GPL(hm2_unregister);
void hm2_unregister(hm2_lowlevel_io_t *llio) {
    struct rtapi_list_head *ptr;

    rtapi_list_for_each(ptr, &hm2_list) {
        hostmot2_t *hm2 = rtapi_list_entry(ptr, hostmot2_t, list);
        if (hm2->llio != llio) continue;

        // if there's a watchdog, set it to safe the board right away
        if (hm2->watchdog.num_instances > 0) {
            hm2->watchdog.instance[0].enable = 1;
            hm2->watchdog.instance[0].hal.param.timeout_ns = 1;
            hm2_watchdog_force_write(hm2);
        }

        HM2_PRINT("unregistered\n");

        hm2_cleanup(hm2);

        rtapi_list_del(ptr);
        rtapi_kfree(hm2);

        return;
    }

    HM2_PRINT_NO_LL("ignoring request to unregister %s: not found\n", llio->name);
    return;
}




//
// setup and cleanup code
//

int rtapi_app_main(void) {
    HM2_PRINT_NO_LL("loading Mesa HostMot2 driver version %s\n", HM2_VERSION);

    comp_id = hal_init("hostmot2");
    if(comp_id < 0) return comp_id;
    RTAPI_INIT_LIST_HEAD(&hm2_list);

    hal_ready(comp_id);

    return 0;
}


void rtapi_app_exit(void) {
    HM2_PRINT_NO_LL("unloading\n");
    hal_exit(comp_id);
}




// this pushes our idea of what things are like into the FPGA's poor little mind
void hm2_force_write(hostmot2_t *hm2) {
    if (hm2->llio->set_force_enqueue != NULL)
        hm2->llio->set_force_enqueue(hm2->llio, 1);
    hm2_watchdog_force_write(hm2);
    hm2_ioport_force_write(hm2);
    hm2_encoder_force_write(hm2);
    hm2_pwmgen_force_write(hm2);
    hm2_rcpwmgen_force_write(hm2);
    hm2_stepgen_force_write(hm2);
    hm2_tp_pwmgen_force_write(hm2);
    hm2_sserial_force_write(hm2);
    hm2_bspi_force_write(hm2);
    hm2_inmux_force_write(hm2);
    hm2_inm_force_write(hm2);
    hm2_xy2mod_force_write(hm2);
    hm2_oneshot_force_write(hm2);
    hm2_periodm_force_write(hm2);

    // NOTE: It's important that the SSR is written *after* the
    // ioport is written.  Initialization of the SSR requires that
    // the IO Port pin directions is set appropriately.
    hm2_ssr_force_write(hm2);
    hm2_outm_force_write(hm2);
    if (hm2->llio->set_force_enqueue != NULL)
        hm2->llio->set_force_enqueue(hm2->llio, 0);
}

