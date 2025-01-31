/*
 * This is a component for RaspberryPi 5 to hostmot2 over SPI for linuxcnc.
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

#ifndef HAL_HM2_RP1DEV_H
#define HAL_HM2_RP1DEV_H

#include "hwregaccess.h"

/*
 * RP1 peripheral description see:
 * - https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf
 *
 * *** Note:
 * Lots of RP1 stuff is not used by the driver and is omitted in the
 * defines in this header file.
 */

/* Bit and mask helpers */
#define RP1BIT(x)		(1 << (x))
#define RP1MASK(x,y)	((x) << (y))

/*
 * The RP1 is PCIe connected and memory mapped. There are several devices
 * attached to the RP1 PCIe connection spanning two "bars". Bar0 has 64k of
 * shared RAM attached. Bar1 is where the RP1 peripherals are mapped. The
 * translation address is visible in the device-tree if you look at:
 *   /proc/device-tree/axi/pcie@120000/ranges
 * Or, look at the linux source:
 *   linux-src-tree.../arch/arm64/boot/dts/broadcom/bcm2712.dts
 * Setting this address here at a fixed value should not give us problems
 * because of all the devices out there already (famous last words).
 */
#define RP1_PCIE_BAR1_ADDR		0x1f00000000	/* Base address in /dev/mem */
#define RP1_PCIE_BAR1_LEN		0x0000400000	/* Map length for access */

/* Register offsets */
#define RP1_IO_BANK0_OFFSET		0xd0000	/* 28 GPIOs from 40-pin header */
#define RP1_IO_BANK1_OFFSET		0xd4000
#define RP1_IO_BANK2_OFFSET		0xd8000

#define RP1_SYS_RIO0_OFFSET		0xe0000	/* GPIO access */
#define RP1_SYS_RIO1_OFFSET		0xe4000
#define RP1_SYS_RIO2_OFFSET		0xe8000

#define RP1_RW_OFFSET			0x0000	/* The address offset for bit manipulation */
#define RP1_XOR_OFFSET			0x1000
#define RP1_SET_OFFSET			0x2000
#define RP1_CLR_OFFSET			0x3000

#define RP1_PADS_BANK0_OFFSET	0xf0000	/* Pad control, drive strength, pull-up/down, etc. */
#define RP1_PADS_BANK1_OFFSET	0xf4000
#define RP1_PADS_BANK2_OFFSET	0xf8000

#define RP1_SPI0_OFFSET			0x50000
#define RP1_SPI1_OFFSET			0x54000
#define RP1_SPI2_OFFSET			0x58000
#define RP1_SPI3_OFFSET			0x5c000
#define RP1_SPI4_OFFSET			0x60000
#define RP1_SPI5_OFFSET			0x64000
/* SPI6, SPI7 and SPI8 are not mappable on the 40-pin header */

/*
 * SPI to GPIO pin mappings
 *  Port | CLK | MOSI | MISO | CE0 | CE1 | CE2 | CE3 |FuncSel
 * ------+-----+------+------+-----+-----+-----+-------------
 *  SPI0 |  11 |  10  |   9  |   8 |   7 |   3 |   2 | 0x00  *** SPI0_CE2 and SPI0_CE3 overlap I2C
 *  SPI1 |  21 |  20  |  19  |  18 |  17 |  16 |     | 0x00
 *  SPI2 |   3 |   2  |   1  |   0 |  24 |     |     | 0x08  *** SPI2_CE0 and SPI2_MISO overlap the ID EEPROM
 *  SPI3 |   7 |   6  |   5  |   4 |  25 |     |     | 0x08  *** SPI3_CLK overlaps SPI0_CE1
 *  SPI4 |  11 |   9  |  10  |   8 |     |     |     | 0x08  *** SPI4 is slave only and overlaps SPI0
 *  SPI5 |  15 |  14  |  13  |  12 |  26 |     |     | 0x08  *** SPI5_CLK and SPI5_MOSI overlap standard UART
 *
 * - SPI0 Dual mode could be supported using GPIO 0 and 1, but probably
 *   requires that probing the ID EEPROM be disabled in the kernel because the
 *   extra GPIOs are assigned to a dedicated I2C port for HAT probing.
 * - SPI0 has two extra CE lines that overlap the default I2C port of the RPi.
 *   These cannot be used if, for example, a touch display is attached.
 * - SPI2 has the problem of overlapping the ID EEPROM pins GPIO 0 and 1.
 *   Probably better not use it.
 * - SPI3 has a clock line overlap with SPI0's CE1. Otherwise it should be
 *   usable is you do not need the GPCLKx lines.
 * - SPI4 is a slave device and cannot be used in our context.
 * - SPI5 has clock and mosi overlap with the default UART (/dev/ttyAMA0). We
 *   should keep access to the TTY.
 *
 * All considering, just using the plain old assignments is far better than
 * the problems we could run into with the other ports. Therefore, the only
 * supported ports are: SPI0/CE[01] and SPI1/CE[012].
 */

/*
 * GPIO data and direction
 */
typedef struct __rp1_rio_regs_t {
	__O  uint32_t	out;			/* 0x00 read/write gpio pins */
	__IO uint32_t	oe;				/* 0x04 data direction (0=out, 1=in) */
	union {
		__I  uint32_t nosync_in;	/* 0x08 direct access to input pins */
		__I  uint32_t in;
	};
	__I  uint32_t	sync_in;		/* 0x0c input through 2-stage synchroniser */
} rp1_rio_regs_t;

typedef struct __rp1_rio_t {
	rp1_rio_regs_t	rw;
	__I uint32_t	reserved1[((RP1_XOR_OFFSET - RP1_RW_OFFSET  - sizeof(rp1_rio_regs_t)) / sizeof(uint32_t))];
	rp1_rio_regs_t	xor;
	__I uint32_t	reserved2[((RP1_SET_OFFSET - RP1_XOR_OFFSET - sizeof(rp1_rio_regs_t)) / sizeof(uint32_t))];
	rp1_rio_regs_t	set;
	__I uint32_t	reserved3[((RP1_CLR_OFFSET - RP1_SET_OFFSET - sizeof(rp1_rio_regs_t)) / sizeof(uint32_t))];
	rp1_rio_regs_t	clr;
	/* There is room below, but we never need to access it */
} rp1_rio_t;

/*
 * Alternate function specification see RP1 documentation
 */
#define RP1_FSEL_SYS_RIO			5	/* Pin as GPIO */
#define RP1_FSEL_ALT0				0	/* The RPi default alternate assignments */

typedef struct __rp1_gpio_stat_ctrl_t {
	__I  uint32_t	status;		/* GPIO status */
	__IO uint32_t	ctrl;		/* GPIO control */
} rp1_gpio_stat_ctrl_t;

#define RP1_GPIO_STAT_IRQTOPROC_BIT				29
#define RP1_GPIO_STAT_IRQCOMBINED_BIT			28
#define RP1_GPIO_STAT_EVENT_DB_LEVEL_HIGH_BIT	27
#define RP1_GPIO_STAT_EVENT_DB_LEVEL_LOW_BIT	26
#define RP1_GPIO_STAT_EVENT_F_EDGE_HIGH_BIT		25
#define RP1_GPIO_STAT_EVENT_F_EDGE_LOW_BIT		24
#define RP1_GPIO_STAT_EVENT_LEVEL_HIGH_BIT		23
#define RP1_GPIO_STAT_EVENT_LEVEL_LOW_BIT		22
#define RP1_GPIO_STAT_EVENT_EDGE_HIGH_BIT		21
#define RP1_GPIO_STAT_EVENT_EDGE_LOW_BIT		20
#define RP1_GPIO_STAT_INTOPERI_BIT				19
#define RP1_GPIO_STAT_INFILTERED_BIT			18
#define RP1_GPIO_STAT_INFROMPAD_BIT				17
#define RP1_GPIO_STAT_INISDIRECT_BIT			16
#define RP1_GPIO_STAT_OETOPAD_BIT				13
#define RP1_GPIO_STAT_OEFROMPERI_BIT			12
#define RP1_GPIO_STAT_OUTTOPAD_BIT				9
#define RP1_GPIO_STAT_OUTFROMPERI_BIT			8

#define RP1_GPIO_CTRL_IRQOVER_BIT				30
#define RP1_GPIO_CTRL_IRQRESET_BIT				28
#define RP1_GPIO_CTRL_IRQMASK_DB_LEVEL_HIGH_BIT	27
#define RP1_GPIO_CTRL_IRQMASK_DB_LEVEL_LOW_BIT	26
#define RP1_GPIO_CTRL_IRQMASK_F_EDGE_HIGH_BIT	25
#define RP1_GPIO_CTRL_IRQMASK_F_EDGE_LOW_BIT	24
#define RP1_GPIO_CTRL_IRQMASK_LEVEL_HIGH_BIT	23
#define RP1_GPIO_CTRL_IRQMASK_LEVEL_LOW_BIT		22
#define RP1_GPIO_CTRL_IRQMASK_EDGE_HIGH_BIT		21
#define RP1_GPIO_CTRL_IRQMASK_EDGE_LOW_BIT		20
#define RP1_GPIO_CTRL_INOVER_BIT				16
#define RP1_GPIO_CTRL_OEOVER_BIT				14
#define RP1_GPIO_CTRL_OUTOVER_BIT				12
#define RP1_GPIO_CTRL_F_M_BIT					5
#define RP1_GPIO_CTRL_FUNCSEL_BIT				0

#define RP1_GPIO_CTRL_INOVER_MASK	RP1MASK(0x03, RP1_GPIO_CTRL_INOVER_BIT)
#define RP1_GPIO_CTRL_OEOVER_MASK	RP1MASK(0x03, RP1_GPIO_CTRL_OEOVER_BIT)
#define RP1_GPIO_CTRL_OUTOVER_MASK	RP1MASK(0x03, RP1_GPIO_CTRL_OUTOVER_BIT)
#define RP1_GPIO_CTRL_F_M_MASK		RP1MASK(0x7f, RP1_GPIO_CTRL_F_M_BIT)
#define RP1_GPIO_CTRL_FUNCSEL_MASK	RP1MASK(0x1f, RP1_GPIO_CTRL_FUNCSEL_BIT)

#define RP1_GPIO_CTRL_INOVER(v)		RP1MASK((v)&0x03, RP1_GPIO_CTRL_INOVER_BIT)
#define RP1_GPIO_CTRL_OEOVER(v)		RP1MASK((v)&0x03, RP1_GPIO_CTRL_OEOVER_BIT)
#define RP1_GPIO_CTRL_OUTOVER(v)	RP1MASK((v)&0x03, RP1_GPIO_CTRL_OUTOVER_BIT)
#define RP1_GPIO_CTRL_F_M(v)		RP1MASK((v)&0x7f, RP1_GPIO_CTRL_F_M_BIT)
#define RP1_GPIO_CTRL_FUNCSEL(v)	RP1MASK((v)&0x1f, RP1_GPIO_CTRL_FUNCSEL_BIT)

#define RP1_GPIO_CTRL_FUNCSEL_ALT0		0
#define RP1_GPIO_CTRL_FUNCSEL_ALT1		1
#define RP1_GPIO_CTRL_FUNCSEL_ALT2		2
#define RP1_GPIO_CTRL_FUNCSEL_ALT3		3
#define RP1_GPIO_CTRL_FUNCSEL_ALT4		4
#define RP1_GPIO_CTRL_FUNCSEL_ALT5		5
#define RP1_GPIO_CTRL_FUNCSEL_ALT6		6
#define RP1_GPIO_CTRL_FUNCSEL_ALT7		7
#define RP1_GPIO_CTRL_FUNCSEL_ALT8		8
#define RP1_GPIO_CTRL_FUNCSEL_DPI		RP1_GPIO_CTRL_FUNCSEL_ALT1
#define RP1_GPIO_CTRL_FUNCSEL_SYS_RIO	RP1_GPIO_CTRL_FUNCSEL_ALT5
#define RP1_GPIO_CTRL_FUNCSEL_PROC_RIO	RP1_GPIO_CTRL_FUNCSEL_ALT6
#define RP1_GPIO_CTRL_FUNCSEL_PIO		RP1_GPIO_CTRL_FUNCSEL_ALT7
#define RP1_GPIO_CTRL_FUNCSEL_NULL		31

typedef struct __rp1_io_bank0_t {
	rp1_gpio_stat_ctrl_t	gpio[28];	/* 0x000 GPIO status and control */
	__I  uint32_t	reserved0e0[8];		/* 0x0e0 */
	__I  uint32_t	intr;				/* 0x100 raw interrupts */
	__IO uint32_t	proc0_inte;			/* 0x104 interrupt enable for proc0 */
	__IO uint32_t	proc0_intf;			/* 0x108 interrupt force for proc0 */
	__I  uint32_t	proc0_ints;			/* 0x10c interrupt status after masking and forcing for proc0 */
	__IO uint32_t	proc1_inte;			/* 0x110 interrupt enable for proc1 */
	__IO uint32_t	proc1_intf;			/* 0x114 interrupt force for proc1 */
	__I  uint32_t	proc1_ints;			/* 0x118 interrupt status after masking and forcing for proc1 */
	__IO uint32_t	pcie_inte;			/* 0x11c interrupt enable for pcie */
	__IO uint32_t	pcie_intf;			/* 0x120 interrupt force for pcie */
	__I  uint32_t	pcie_ints;			/* 0x124 interrupt status after masking and forcing for pcie */
} rp1_io_bank0_t;

typedef struct __rp1_pads_bank0_t {
	__IO uint32_t	voltage_select;		/* 0x00 bank voltage control */
	__IO uint32_t	gpio[28];			/* 0x04 pad control registers for each pin */
} rp1_pads_bank0_t;

#define RP1_PADS_VOLTAGE_CONTROL_3V3	0
#define RP1_PADS_VOLTAGE_CONTROL_1V8	1

#define RP1_PADS_OD_BIT			7	/* output disable */
#define RP1_PADS_IE_BIT			6	/* interrupt enable */
#define RP1_PADS_DRIVE_BIT		4	/* bits 5:4 drive strength */
#define RP1_PADS_PUE_BIT		3	/* pull up enable */
#define RP1_PADS_PDE_BIT		2	/* pull down enable */
#define RP1_PADS_SCHMITT_BIT	1	/* schmitt trigger enable */
#define RP1_PADS_SLEWFAST_BIT	0	/* slew rate control */
#define RP1_PADS_OD				RP1BIT(RP1_PADS_OD_BIT)
#define RP1_PADS_IE				RP1BIT(RP1_PADS_IE_BIT)
#define RP1_PADS_DRIVE_MASK		RP1MASK(0x3, RP1_PADS_DRIVE_BIT)
#define RP1_PADS_DRIVE(v)		RP1MASK((v) & 0x3, RP1_PADS_DRIVE_BIT)
#define RP1_PADS_PUE			RP1BIT(RP1_PADS_PUE_BIT)
#define RP1_PADS_PDE			RP1BIT(RP1_PADS_PDE_BIT)
#define RP1_PADS_SCHMITT		RP1BIT(RP1_PADS_SCHMITT_BIT)
#define RP1_PADS_SLEWFAST		RP1BIT(RP1_PADS_SLEWFAST_BIT)
#define RP1_PADS_DRIVE_2		0	// 2mA drive
#define RP1_PADS_DRIVE_4		1	// 4mA drive
#define RP1_PADS_DRIVE_8		2	// 8mA drive
#define RP1_PADS_DRIVE_12		3	// 12mA drive

/*
 * The Designware SSI
 *
 * The RP1 SSI implementation has:
 * - a fifo depth of 64.
 * - 32 bit max. transfer word size (uses CTRLR0_DFS_32)
 */

#define RP1_SPI_CLK			200000000	/* Apparently, master clock is at 200 MHz */
#define RP1_SPI_FIFO_LEN	64			/* Fifo len */

typedef struct __dw_ssi_t {
	__IO uint32_t	ctrlr0;			/* 0x00 serial data transfer control */
	__IO uint32_t	ctrlr1;			/* 0x04 data frame count (rx-only and eeprom modes) */
	__IO uint32_t	ssienr;			/* 0x08 ssi enable/disable */
	__IO uint32_t	mwcr;			/* 0x0c microwire control */
	__IO uint32_t	ser;			/* 0x10 slave enable bits */
	__IO uint32_t	baudr;			/* 0x14 baudrate divider (16 bit and must be even) */
	__IO uint32_t	txftlr;			/* 0x18 transmit fifo threshold level*/
	__IO uint32_t	rxftlr;			/* 0x1c receive fifo threshold level */
	__I  uint32_t	txflr;			/* 0x20 transmit fifo level */
	__I  uint32_t	rxflr;			/* 0x24 receive fifo level */
	__I  uint32_t	sr;				/* 0x28 transfer status */
	__IO uint32_t	imr;			/* 0x2c interrupt mask */
	__I  uint32_t	isr;			/* 0x30 interrupt status (masked by imr) */
	__I  uint32_t	risr;			/* 0x34 raw interrupt status (unmasked) */
	__I  uint32_t	txoicr;			/* 0x38 transmit fifo overflow interrupt clear */
	__I  uint32_t	rxoicr;			/* 0x3c receive fifo overflow interrupt clear */
	__I  uint32_t	rxuicr;			/* 0x40 receive fifo underflow interrupt clear */
	__I  uint32_t	msticr;			/* 0x44 multi-master interrupt clear */
	__I  uint32_t	icr;			/* 0x48 interrupt clear */
	__IO uint32_t	dmacr;			/* 0x4c dma control */
	__IO uint32_t	dmatdlr;		/* 0x50 dma transmit data level */
	__IO uint32_t	dmardlr;		/* 0x54 dma receive data level */
	__I  uint32_t	idr;			/* 0x58 peripheral identification code */
	__I  uint32_t	ssi_version_id;	/* 0x5c dw ssi hardware version ident */
	union {
		struct {
			__IO uint32_t dr0;		/* 0x60-0xec data register 16/32-bit */
			__I  uint32_t reserved_dr[35];
		};
		__IO uint32_t	drx[36];	/* All these act the same according to the docs, use dr0 */
	};
	__IO uint32_t	rx_sample_dly;	/* 0xf0 receive sample delay */
	__IO uint32_t	spi_ctrlr0;		/* 0xf4 multimode control*/
	__IO uint32_t	tx_drive_edge;	/* 0xf8 transmit drive edge */
	__I  uint32_t	rsvd;			/* 0xfc reserved */
} dw_ssi_t;

#define DW_SSI_CTRLR0_SSTE_BIT		24	/* slave select toggle enable */
#define DW_SSI_CTRLR0_SPI_FRF_BIT	21	/* SPI frame format */
#define DW_SSI_CTRLR0_DFS_32_BIT	16	/* data frame size */
#define DW_SSI_CTRLR0_CFS_BIT		12	/* control frame size */
#define DW_SSI_CTRLR0_SRL_BIT		11	/* shift register loop */
#define DW_SSI_CTRLR0_SLV_OE_BIT	10	/* slave output enable */
#define DW_SSI_CTRLR0_TMOD_BIT		8	/* transfer mode */
#define DW_SSI_CTRLR0_SCPOL_BIT		7	/* serial clock polarity */
#define DW_SSI_CTRLR0_SCPH_BIT		6	/* serial clock phase */
#define DW_SSI_CTRLR0_FRF_BIT		4	/* frame format */
#define DW_SSI_CTRLR0_DFS_BIT		0	/* data frame size */

#define DW_SSI_CTRLR0_SSTE			RP1BIT(DW_SSI_CTRLR0_SSTE_BIT)
#define DW_SSI_CTRLR0_SPI_FRF_MASK	RP1MASK(0x03, DW_SSI_CTRLR0_SPI_FRF_BIT)
#define DW_SSI_CTRLR0_DFS_32_MASK	RP1MASK(0x1f, DW_SSI_CTRLR0_DFS_32_BIT)
#define DW_SSI_CTRLR0_CFS_MASK		RP1MASK(0x0f, DW_SSI_CTRLR0_CFS_BIT)
#define DW_SSI_CTRLR0_SRL			RP1BIT(DW_SSI_CTRLR0_SRL_BIT)
#define DW_SSI_CTRLR0_SLV_OE		RP1BIT(DW_SSI_CTRLR0_SLV_OE_BIT)
#define DW_SSI_CTRLR0_TMOD_MASK		RP1MASK(0x03, DW_SSI_CTRLR0_TMOD_BIT)
#define DW_SSI_CTRLR0_SCPOL			RP1BIT(DW_SSI_CTRLR0_SCPOL_BIT)
#define DW_SSI_CTRLR0_SCPH			RP1BIT(DW_SSI_CTRLR0_SCPH_BIT)
#define DW_SSI_CTRLR0_FRF_MASK		RP1MASK(0x03, DW_SSI_CTRLR0_FRF_BIT)
#define DW_SSI_CTRLR0_DFS_MASK		RP1MASK(0x0f, DW_SSI_CTRLR0_DFS_BIT)

#define DW_SSI_CTRLR0_SPI_FRF(v)	RP1MASK((v)&0x03, DW_SSI_CTRLR0_SPI_FRF_BIT)
#define DW_SSI_CTRLR0_DFS_32(v)		RP1MASK((v)&0x1f, DW_SSI_CTRLR0_DFS_32_BIT)
#define DW_SSI_CTRLR0_CFS(v)		RP1MASK((v)&0x0f, DW_SSI_CTRLR0_CFS_BIT)
#define DW_SSI_CTRLR0_TMOD(v)		RP1MASK((v)&0x03, DW_SSI_CTRLR0_TMOD_BIT)
#define DW_SSI_CTRLR0_FRF(v)		RP1MASK((v)&0x03, DW_SSI_CTRLR0_FRF_BIT)
#define DW_SSI_CTRLR0_DFS(v)		RP1MASK((v)&0x0f, DW_SSI_CTRLR0_DFS_BIT)

#define DW_SSI_CTRLR0_TMOD_TXRX		0	/* transmit and receive */
#define DW_SSI_CTRLR0_TMOD_TX		1	/* transmit only */
#define DW_SSI_CTRLR0_TMOD_RX		2	/* receive only */
#define DW_SSI_CTRLR0_TMOD_EERD		3	/* EEPROM read */

#define DW_SSI_CTRLR1_NDF_BIT		0	/* number of data frames */
#define DW_SSI_CTRLR1_NDF_MASK		RP1MASK(0xffff, DW_SSI_CTRLR1_NDF_BIT)

#define DW_SSI_SSIENR_SSI_EN_BIT	0
#define DW_SSI_SSIENR_SSI_EN		RP1BIT(DW_SSI_SSIENR_SSI_EN_BIT)

#define DW_SSI_MWCR_MHS_BIT			2	/* microwire handshaking */
#define DW_SSI_MWCR_MDD_BIT			1	/* microwire direction control */
#define DW_SSI_MWCR_MWMOD_BIT		0	/* microwire transfer mode */

/*
 * There are max. 4 chip selects (SPI0(4), SPI1(3), SPI2(2), SPI3(2), SPI4(1),
 * SPI5(2), SPI6(3), SPI7(1), SPI8(2))
 */
#define DW_SSI_SER_CS0_BIT			0	/* chip select 0 */
#define DW_SSI_SER_CS1_BIT			1	/* chip select 1 */
#define DW_SSI_SER_CS2_BIT			2	/* chip select 2 */
#define DW_SSI_SER_CS3_BIT			3	/* chip select 3 */
#define DW_SSI_SER_CS0				RP1BIT(DW_SSI_SER_CS0_BIT)
#define DW_SSI_SER_CS1				RP1BIT(DW_SSI_SER_CS1_BIT)
#define DW_SSI_SER_CS2				RP1BIT(DW_SSI_SER_CS2_BIT)
#define DW_SSI_SER_CS3				RP1BIT(DW_SSI_SER_CS3_BIT)

#define DW_SSI_BAUDR_SCKDV_BIT		0	/* ssi clock divider (must be even) */
#define DW_SSI_BAUDR_SCKDV_MASK		RP1MASK(0xffff, DW_SSI_BAUDR_SCKDV_BIT)
#define DW_SSI_BAUDR_SCKDV(v)		RP1MASK((v)&0xfffe, DW_SSI_BAUDR_SCKDV_BIT)

#define DW_SSI_SR_DCOL_BIT		6	/* data collision error */
#define DW_SSI_SR_TXE_BIT		5	/* transmission error */
#define DW_SSI_SR_RFF_BIT		4	/* receive fifo full */
#define DW_SSI_SR_RFNE_BIT		3	/* receive fifo not empty */
#define DW_SSI_SR_TFE_BIT		2	/* transmit fifo empty */
#define DW_SSI_SR_TFNF_BIT		1	/* transmit fifo not full */
#define DW_SSI_SR_BUSY_BIT		0	/* ssi busy flag */
#define DW_SSI_SR_DCOL			RP1BIT(DW_SSI_SR_DCOL_BIT)
#define DW_SSI_SR_TXE			RP1BIT(DW_SSI_SR_TXE_BIT)
#define DW_SSI_SR_RFF			RP1BIT(DW_SSI_SR_RFF_BIT)
#define DW_SSI_SR_RFNE			RP1BIT(DW_SSI_SR_RFNE_BIT)
#define DW_SSI_SR_TFE			RP1BIT(DW_SSI_SR_TFE_BIT)
#define DW_SSI_SR_TFNF			RP1BIT(DW_SSI_SR_TFNF_BIT)
#define DW_SSI_SR_BUSY			RP1BIT(DW_SSI_SR_BUSY_BIT)

#define DW_SSI_IMR_MSTIM_BIT	5	/* multi-master contention interrupt mask */
#define DW_SSI_IMR_RXFIM_BIT	4	/* receive fifo full interrupt mask */
#define DW_SSI_IMR_RXOIM_BIT	3	/* receive fifi overflow interrupt mask */
#define DW_SSI_IMR_RXUIM_BIT	2	/* receive fifo underflow interrupt mask */
#define DW_SSI_IMR_TXOIM_BIT	1	/* transmit fifo overflow interrupt mask */
#define DW_SSI_IMR_TXEIM_BIT	0	/* transmit fifo empty interrupt mask */
#define DW_SSI_IMR_MSTIM		RP1BIT(DW_SSI_IMR_MSTIM_BIT)
#define DW_SSI_IMR_RXFIM		RP1BIT(DW_SSI_IMR_RXFIM_BIT)
#define DW_SSI_IMR_RXOIM		RP1BIT(DW_SSI_IMR_RXOIM_BIT)
#define DW_SSI_IMR_RXUIM		RP1BIT(DW_SSI_IMR_RXUIM_BIT)
#define DW_SSI_IMR_TXOIM		RP1BIT(DW_SSI_IMR_TXOIM_BIT)
#define DW_SSI_IMR_TXEIM		RP1BIT(DW_SSI_IMR_TXEIM_BIT)

#define DW_SSI_ISR_MSTIS_BIT	5	/* multi-master contention interrupt status */
#define DW_SSI_ISR_RXFIS_BIT	4	/* receive fifo full interrupt status */
#define DW_SSI_ISR_RXOIS_BIT	3	/* receive fifi overflow interrupt status */
#define DW_SSI_ISR_RXUIS_BIT	2	/* receive fifo underflow interrupt status */
#define DW_SSI_ISR_TXOIS_BIT	1	/* transmit fifo overflow interrupt status */
#define DW_SSI_ISR_TXEIS_BIT	0	/* transmit fifo empty interrupt status */
#define DW_SSI_ISR_MSTIS		RP1BIT(DW_SSI_ISR_MSTIS_BIT)
#define DW_SSI_ISR_RXFIS		RP1BIT(DW_SSI_ISR_RXFIS_BIT)
#define DW_SSI_ISR_RXOIS		RP1BIT(DW_SSI_ISR_RXOIS_BIT)
#define DW_SSI_ISR_RXUIS		RP1BIT(DW_SSI_ISR_RXUIS_BIT)
#define DW_SSI_ISR_TXOIS		RP1BIT(DW_SSI_ISR_TXOIS_BIT)
#define DW_SSI_ISR_TXEIS		RP1BIT(DW_SSI_ISR_TXEIS_BIT)

#define DW_SSI_RISR_MSTIR_BIT	5	/* multi-master contention interrupt raw status */
#define DW_SSI_RISR_RXFIR_BIT	4	/* receive fifo full interrupt raw status */
#define DW_SSI_RISR_RXOIR_BIT	3	/* receive fifi overflow interrupt raw status */
#define DW_SSI_RISR_RXUIR_BIT	2	/* receive fifo underflow interrupt raw status */
#define DW_SSI_RISR_TXOIR_BIT	1	/* transmit fifo overflow interrupt raw status */
#define DW_SSI_RISR_TXEIR_BIT	0	/* transmit fifo empty interrupt raw status */
#define DW_SSI_RISR_MSTIR		RP1BIT(DW_SSI_RISR_MSTIR_BIT)
#define DW_SSI_RISR_RXFIR		RP1BIT(DW_SSI_RISR_RXFIR_BIT)
#define DW_SSI_RISR_RXOIR		RP1BIT(DW_SSI_RISR_RXOIR_BIT)
#define DW_SSI_RISR_RXUIR		RP1BIT(DW_SSI_RISR_RXUIR_BIT)
#define DW_SSI_RISR_TXOIR		RP1BIT(DW_SSI_RISR_TXOIR_BIT)
#define DW_SSI_RISR_TXEIR		RP1BIT(DW_SSI_RISR_TXEIR_BIT)

#endif
/* vim: ts=4
 */
