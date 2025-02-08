/*
 * This is a component for hostmot2 board identification
 * Copyright (c) 2024 B.Stultiens <lcnc@vagrearg.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, see <https://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <ctype.h>

#include <rtapi.h>

#include "hostmot2.h"
#include "hostmot2-lowlevel.h"
#include "llio_info.h"

typedef struct __info_entry_t {
	char board_name[8];
	const char *base_name;
	int num_ioport_connectors;
	int pins_per_connector;
	const char *ioport_connector_name[ANYIO_MAX_IOPORT_CONNECTORS];
	const char **io_connector_pin_names;
	int num_leds;
	const char *fpga_part_number;
	int (*hook)(hm2_lowlevel_io_t *llio, const hm2_idrom_t *idrom);
} info_entry_t;

static const char *hm2_7c80_pin_names[] = {
	"TB07-02/TB07-03",	/* Step/Dir/Misc 5V out */
	"TB07-04/TB07-05",
	"TB08-02/TB08-03",
	"TB08-04/TB08-05",
	"TB09-02/TB09-03",
	"TB09-04/TB09-05",
	"TB10-02/TB10-03",
	"TB10-04/TB10-05",
	"TB11-02/TB11-03",
	"TB11-04/TB11-05",
	"TB12-02/TB12-03",
	"TB12-04/TB12-05",
	"TB03-03/TB04-04",	/* RS-422/RS-485 interface */
	"TB03-05/TB04-06",
	"TB03-05/TB03-06",
	"TB04-01/TB04-02",	/* Encoder */
	"TB04-04/TB04-05",
	"TB04-07/TB04-08",
	"TB05-02",		/* Spindle */
	"TB05-02",
	"TB05-05/TB05-06",
	"TB05-07/TB05-08",
	"Internal InMux0",	/* InMux */
	"Internal InMux1",
	"Internal InMux2",
	"Internal InMux3",
	"Internal InMux4",

	"Internal InMuxData",
	"TB13-01/TB13-02",	/* SSR */
	"TB13-03/TB13-04",
	"TB13-05/TB13-06",
	"TB13-07/TB13-08",
	"TB14-01/TB14-02",
	"TB14-03/TB14-04",
	"TB14-05/TB14-06",
	"TB14-07/TB14-08",
	"Internal SSR",
	"P1-01/DB25-01",   /* P1 parallel expansion */
	"P1-02/DB25-14",
	"P1-03/DB25-02",
	"P1-04/DB25-15",
	"P1-05/DB25-03",
	"P1-06/DB25-16",
	"P1-07/DB25-04",
	"P1-08/DB25-17",
	"P1-09/DB25-05",
	"P1-11/DB25-06",
	"P1-13/DB25-07",
	"P1-15/DB25-08",
	"P1-17/DB25-09",
	"P1-19/DB25-10",
	"P1-21/DB25-11",
	"P1-23/DB25-12",
	"P1-25/DB25-13",
};

static const char *hm2_7c81_pin_names[] = {
	"P1-01/DB25-01",
	"P1-02/DB25-14",
	"P1-03/DB25-02",
	"P1-04/DB25-15",
	"P1-05/DB25-03",
	"P1-06/DB25-16",
	"P1-07/DB25-04",
	"P1-08/DB25-17",
	"P1-09/DB25-05",
	"P1-11/DB25-06",
	"P1-13/DB25-07",
	"P1-15/DB25-08",
	"P1-17/DB25-09",
	"P1-19/DB25-10",
	"P1-21/DB25-11",
	"P1-23/DB25-12",
	"P1-25/DB25-13",
	"J5-TX0",
	"J6-TX1",

	"P2-01/DB25-01",
	"P2-02/DB25-14",
	"P2-03/DB25-02",
	"P2-04/DB25-15",
	"P2-05/DB25-03",
	"P2-06/DB25-16",
	"P2-07/DB25-04",
	"P2-08/DB25-17",
	"P2-09/DB25-05",
	"P2-11/DB25-06",
	"P2-13/DB25-07",
	"P2-15/DB25-08",
	"P2-17/DB25-09",
	"P2-19/DB25-10",
	"P2-21/DB25-11",
	"P2-23/DB25-12",
	"P2-25/DB25-13",
	"J5-TXEN0",
	"J6-TXEN1",

	"P7-01/DB25-01",
	"P7-02/DB25-14",
	"P7-03/DB25-02",
	"P7-04/DB25-15",
	"P7-05/DB25-03",
	"P7-06/DB25-16",
	"P7-07/DB25-04",
	"P7-08/DB25-17",
	"P7-09/DB25-05",
	"P7-11/DB25-06",
	"P7-13/DB25-07",
	"P7-15/DB25-08",
	"P7-17/DB25-09",
	"P7-19/DB25-10",
	"P7-21/DB25-11",
	"P7-23/DB25-12",
	"P7-25/DB25-13",
	"P5-RX0",
	"P6-RX1"
};

int hook_7i43(hm2_lowlevel_io_t *llio, const hm2_idrom_t *idrom)
{
	switch(idrom->fpga_size) {
	case 200: llio->fpga_part_number = "3s200tq144"; return 0;
	default:
		rtapi_print_msg(RTAPI_MSG_WARN, "hook_7i43(): Unknown fpga_size: %d\n", idrom->fpga_size);
		/* Fallthrough */
	case 400: llio->fpga_part_number = "3s400tq144"; return 0;
	}
}

static const info_entry_t spiboards[] = {
	{
		.board_name = "MESA7C80",
		.base_name = "hm2_7c80",
		.num_ioport_connectors = 2,
		.pins_per_connector = 27,
		.ioport_connector_name = { "Embedded I/O", "Embedded I/O + P1 expansion" },
		.io_connector_pin_names = hm2_7c80_pin_names,
		.num_leds = 4,
		.fpga_part_number = "xc6slx9tq144",
		.hook = NULL,
	},
	{
		.board_name = "MESA7C81",
		.base_name = "hm2_7c81",
		.num_ioport_connectors = 3,
		.pins_per_connector = 19,
		.ioport_connector_name = { "P1", "P2", "P7" },
		.io_connector_pin_names = hm2_7c81_pin_names,
		.num_leds = 4,
		.fpga_part_number = "xc6slx9tq144",
		.hook = NULL,
	},
	{
		.board_name = "MESA7I90",
		.base_name = "hm2_7i90",
		.num_ioport_connectors = 3,
		.pins_per_connector = 24,
		.ioport_connector_name = { "P1", "P2", "P3" },
		.io_connector_pin_names = NULL,
		.num_leds = 2,
		.fpga_part_number ="xc6slx9tq144",
		.hook = NULL,
	},
	{
		.board_name = "MESA7I43",
		.base_name = "hm2_7i43",
		.num_ioport_connectors = 2,
		.pins_per_connector = 24,
		.ioport_connector_name = { "P4", "P3" },
		.io_connector_pin_names = NULL,
		.num_leds = 8,
		.fpga_part_number = "3s400tq144",
		.hook = hook_7i43,
	},
};

#define NELEM(x)	(sizeof(x) / sizeof(*(x)))

const char *set_llio_info_spi(hm2_lowlevel_io_t *llio, const hm2_idrom_t *idrom)
{
	char buf[sizeof(idrom->board_name)+1];

	/* In the far future, when there are too many boards, use bsearch */
	/* With few boards, linear search is faster */
	for(unsigned i = 0; i < NELEM(spiboards); i++) {
		if(!memcmp(idrom->board_name, spiboards[i].board_name, sizeof(idrom->board_name))) {
			llio->num_ioport_connectors = spiboards[i].num_ioport_connectors;
			llio->pins_per_connector = spiboards[i].pins_per_connector;
			for(unsigned j = 0; j < ANYIO_MAX_IOPORT_CONNECTORS; j++)
				llio->ioport_connector_name[j] = spiboards[i].ioport_connector_name[j];
			llio->io_connector_pin_names = spiboards[i].io_connector_pin_names;
			llio->num_leds = spiboards[i].num_leds;
			llio->fpga_part_number = spiboards[i].fpga_part_number;
			/* Call the hook if defined */
			if(spiboards[i].hook) {
				/* Hooks should return zero on success */
				if(0 != spiboards[i].hook(llio, idrom))
					return NULL;
			}
			return spiboards[i].base_name;
		}
	}

	memcpy(buf, idrom->board_name, sizeof(idrom->board_name));
	buf[sizeof(idrom->board_name)] = 0;
	for(unsigned i = 0; i < sizeof(idrom->board_name); i++) {
		if(!isprint(buf[i]))
			buf[i] = '?';
	}
	rtapi_print_msg(RTAPI_MSG_ERR, "set_llio_info_spi(): Unknown hostmot2 board name: %.8s\n", buf);
	return NULL;
}

/* vim: ts=4
 */
