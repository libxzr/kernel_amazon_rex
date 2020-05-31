/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 * http://www.ti.com/product/bq27411-g1
 * http://www.ti.com/product/bq27421-g1
 * http://www.ti.com/product/bq27425-g1
 * http://www.ti.com/product/bq27441-g1
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/notifier.h>

#include <linux/power/bq27x00_battery.h>


#define DRIVER_VERSION			"1.2.0"

#define INVALID_REG_ADDR		0xFF

enum bq27xxx_reg_index {
	BQ27XXX_REG_CTRL = 0,
	BQ27XXX_REG_TEMP,
	BQ27XXX_REG_INT_TEMP,
	BQ27XXX_REG_VOLT,
	BQ27XXX_REG_AI,
	BQ27XXX_REG_FLAGS,
	BQ27XXX_REG_TTE,
	BQ27XXX_REG_TTF,
	BQ27XXX_REG_TTES,
	BQ27XXX_REG_TTECP,
	BQ27XXX_REG_NAC,
	BQ27XXX_REG_FCC,
	BQ27XXX_REG_CYCT,
	BQ27XXX_REG_AE,
	BQ27XXX_REG_SOC,
	BQ27XXX_REG_DCAP,
	BQ27XXX_POWER_AVG,
	NUM_REGS
};

/* bq27500 registers */
static __initdata u8 bq27500_regs[NUM_REGS] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0xFF,	/* INT TEMP -NA	*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0A,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0x18,	/* TTF		*/
	0x1c,	/* TTES		*/
	0x26,	/* TTECP	*/
	0x0C,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2A,	/* CYCT		*/
	0x22,	/* AE		*/
	0x2C,	/* SOC(RSOC)	*/
	0x3C,	/* DCAP(ILMD)	*/
	0x24,	/* AP		*/
};

/* bq27520 registers */
static __initdata u8 bq27520_regs[] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0xFF,	/* INT TEMP - NA*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0A,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0x18,	/* TTF		*/
	0x1c,	/* TTES		*/
	0x26,	/* TTECP	*/
	0x0C,	/* NAC		*/
	0x12,	/* LMD		*/
	0xFF,	/* CYCT - NA	*/
	0x22,	/* AE		*/
	0x2C,	/* SOC(RSOC	*/
	0xFF,	/* DCAP(ILMD) - NA */
	0x24,	/* AP		*/
};

/* bq2753x registers */
static __initdata u8 bq2753x_regs[] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0xFF,	/* INT TEMP - NA*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0A,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0xFF,	/* TTF - NA	*/
	0xFF,	/* TTES - NA	*/
	0xFF,	/* TTECP - NA	*/
	0x0C,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2A,	/* CYCT		*/
	0xFF,	/* AE - NA	*/
	0x2C,	/* SOC(RSOC)	*/
	0xFF,	/* DCAP(ILMD) - NA */
	0x24,	/* AP		*/
};

/* bq2754x registers */
static __initdata u8 bq2754x_regs[NUM_REGS] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0x28,	/* INT TEMP - NA*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0A,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0xFF,	/* TTF - NA	*/
	0xFF,	/* TTES - NA	*/
	0xFF,	/* TTECP - NA	*/
	0x0C,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2A,	/* CYCT		*/
	0xFF,	/* AE - NA	*/
	0x2C,	/* SOC(RSOC)	*/
	0xFF,	/* DCAP(ILMD) - NA */
	0xFF,	/* AP		*/
};

/* bq27200 registers */
static __initdata u8 bq27200_regs[NUM_REGS] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0xFF,	/* INT TEMP - NA	*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0A,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0x18,	/* TTF		*/
	0x1c,	/* TTES		*/
	0x26,	/* TTECP	*/
	0x0C,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2A,	/* CYCT		*/
	0x22,	/* AE		*/
	0x0B,	/* SOC(RSOC)	*/
	0x76,	/* DCAP(ILMD)	*/
	0x24,	/* AP		*/
};

/* bq274xx registers */
static __initdata u8 bq274xx_regs[NUM_REGS] = {
	0x00,	/* CONTROL	*/
	0x02,	/* TEMP		*/
	0x1e,	/* INT TEMP	*/
	0x04,	/* VOLT		*/
	0x10,	/* AVG CURR	*/
	0x06,	/* FLAGS	*/
	0xFF,	/* TTE - NA	*/
	0xFF,	/* TTF - NA	*/
	0xFF,	/* TTES - NA	*/
	0xFF,	/* TTECP - NA	*/
	0x08,	/* NAC		*/
	0x0E,	/* FCC		*/
	0xFF,	/* CYCT - NA	*/
	0xFF,	/* AE - NA	*/
	0x1C,	/* SOC		*/
	0x3C,	/* DCAP - NA	*/
	0x18,	/* AP		*/
};

/* bq276xx registers - same as bq274xx except CYCT */
static __initdata u8 bq276xx_regs[NUM_REGS] = {
	0x00,	/* CONTROL	*/
	0x02,	/* TEMP		*/
	0x1e,	/* INT TEMP	*/
	0x04,	/* VOLT		*/
	0x10,	/* AVG CURR	*/
	0x06,	/* FLAGS	*/
	0xFF,	/* TTE - NA	*/
	0xFF,	/* TTF - NA	*/
	0xFF,	/* TTES - NA	*/
	0xFF,	/* TTECP - NA	*/
	0x08,	/* NAC		*/
	0x0E,	/* FCC		*/
	0x22,	/* CYCT		*/
	0xFF,	/* AE - NA	*/
	0x1C,	/* SOC		*/
	0x3C,	/* DCAP - NA	*/
	0x18,	/* AP		*/
};

/* for device that does not have a design capacity register */

#define HARDCODED_DCAP 1340
/*
 * SBS Commands for DF access - these are pretty standard
 * So, no need to go in the command array
 */
#define BLOCK_DATA_CLASS		0x3E
#define DATA_BLOCK			0x3F
#define BLOCK_DATA			0x40
#define BLOCK_DATA_CHECKSUM		0x60
#define BLOCK_DATA_CONTROL		0x61

/* bq274xx/bq276xx specific command information */
#define BQ274XX_UNSEAL_KEY		0x80008000
#define BQ274XX_RESET			0x41
#define BQ274XX_SOFT_RESET		0x42

#define BQ274XX_FLAG_ITPOR				0x20
#define BQ274XX_CTRL_STATUS_INITCOMP	0x80

#define BQ27XXX_FLAG_DSC		BIT(0)
#define BQ27XXX_FLAG_SOCF		BIT(1) /* State-of-Charge threshold final */
#define BQ27XXX_FLAG_SOC1		BIT(2) /* State-of-Charge threshold 1 */
#define BQ27XXX_FLAG_FC			BIT(9)
#define BQ27XXX_FLAG_OTD		BIT(14)
#define BQ27XXX_FLAG_OTC		BIT(15)

/* BQ27000 has different layout for Flags register */
#define BQ27200_FLAG_EDVF		BIT(0) /* Final End-of-Discharge-Voltage flag */
#define BQ27200_FLAG_EDV1		BIT(1) /* First End-of-Discharge-Voltage flag */
#define BQ27200_FLAG_CI			BIT(4) /* Capacity Inaccurate flag */
#define BQ27200_FLAG_FC			BIT(5)
#define BQ27200_FLAG_CHGS		BIT(7) /* Charge state flag */

#define BQ27200_RS			20 /* Resistor sense */
#define BQ27200_POWER_CONSTANT		(256 * 29200 / 1000)

/* Subcommands of Control() */
#define CONTROL_STATUS_SUBCMD		0x0000
#define DEV_TYPE_SUBCMD			0x0001
#define FW_VER_SUBCMD			0x0002
#define DF_VER_SUBCMD			0x001F
#define RESET_SUBCMD			0x0041
#define SET_CFGUPDATE_SUBCMD		0x0013
#define SEAL_SUBCMD			0x0020

/* Location of SEAL enable bit in bq276xx DM */
#define BQ276XX_OP_CFG_B_SUBCLASS	64
#define BQ276XX_OP_CFG_B_OFFSET		2
#define BQ276XX_OP_CFG_B_DEF_SEAL_BIT	(1 << 5)

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(struct bq27x00_device_info *di, u8 reg, bool single);
	int (*write)(struct bq27x00_device_info *di, u8 reg, int value,
			bool single);
	int (*blk_read)(struct bq27x00_device_info *di, u8 reg, u8 *data,
		u8 sz);
	int (*blk_write)(struct bq27x00_device_info *di, u8 reg, u8 *data,
		u8 sz);
};

enum bq27x00_chip { BQ27200, BQ27500, BQ27520, BQ274XX, BQ276XX, BQ2753X,
	BQ27542, BQ27545};

struct bq27x00_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int energy;
	int flags;
	int power_avg;
	int health;
};

struct dm_reg {
	u8 subclass;
	u8 offset;
	u8 len;
	u32 data;
};

struct bq27x00_device_info {
	struct device 		*dev;
	int			id;
	enum bq27x00_chip	chip;

	struct bq27x00_reg_cache cache;
	int charge_design_full;

	unsigned long last_update;
	struct delayed_work work;

	struct power_supply	*bat;

	struct bq27x00_access_methods bus;

	struct mutex lock;

	int fw_ver;
	int df_ver;
	u8 regs[NUM_REGS];
	struct dm_reg *dm_regs;
	u16 dm_regs_count;
};

static __initdata enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
};

static __initdata enum power_supply_property bq27520_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
};

static __initdata enum power_supply_property bq2753x_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

static __initdata enum power_supply_property bq27542_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

static __initdata enum power_supply_property bq27545_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_POWER_AVG,
};


//static __initdata enum power_supply_property bq274xx_battery_props[] = {
static enum power_supply_property bq274xx_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static __initdata enum power_supply_property bq276xx_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

/*
 * Ordering the parameters based on subclass and then offset will help in
 * having fewer flash writes while updating.
 * Customize these values and, if necessary, add more based on system needs.
 */
static struct dm_reg bq274xx_dm_regs[] = {
#if USE_DEFAULT
	{82, 0, 2, 16736},	/* Qmax */
	{82, 5, 1, 0x81},	/* Load Select */
	{82, 6, 2, 1070},	/* Design Capacity */
	{82, 8, 2, 4066},	/* Design Energy */
	{82, 10, 2, 3400},	/* Terminate Voltage */
	{82, 21, 2, 202},	/* Taper rate */
#else
	{0x02,   0, 1, 0x02},
	{0x02,   1, 1, 0x26},
	{0x02,   2, 1, 0x00},
	{0x02,   3, 1, 0x00},
	{0x02,   4, 1, 0x32},
	{0x02,   5, 1, 0x00},
	{0x02,   6, 1, 0x00},
	{0x02,   7, 1, 0x00},
	{0x02,   8, 1, 0x00},
	{0x02,   9, 1, 0x00},
	{0x02,  10, 1, 0x00},
	{0x02,  11, 1, 0x00},
	{0x02,  12, 1, 0x00},
	{0x02,  13, 1, 0x00},
	{0x02,  14, 1, 0x00},
	{0x02,  15, 1, 0x00},
	{0x02,  16, 1, 0x00},
	{0x02,  17, 1, 0x00},
	{0x02,  18, 1, 0x00},
	{0x02,  19, 1, 0x00},
	{0x02,  20, 1, 0x00},
	{0x02,  21, 1, 0x00},
	{0x02,  22, 1, 0x00},
	{0x02,  23, 1, 0x00},
	{0x02,  24, 1, 0x00},
	{0x02,  25, 1, 0x00},
	{0x02,  26, 1, 0x00},
	{0x02,  27, 1, 0x00},
	{0x02,  28, 1, 0x00},
	{0x02,  29, 1, 0x00},
	{0x02,  30, 1, 0x00},
	{0x02,  31, 1, 0x00},
	{0x24,   0, 1, 0x00},
	{0x24,   1, 1, 0x19},
	{0x24,   2, 1, 0x28},
	{0x24,   3, 1, 0x63},
	{0x24,   4, 1, 0x5F},
	{0x24,   5, 1, 0xFF},
	{0x24,   6, 1, 0x62},
	{0x24,   7, 1, 0x00},
	{0x24,   8, 1, 0x32},
	{0x24,   9, 1, 0x00},
	{0x24,  10, 1, 0x00},
	{0x24,  11, 1, 0x00},
	{0x24,  12, 1, 0x00},
	{0x24,  13, 1, 0x00},
	{0x24,  14, 1, 0x00},
	{0x24,  15, 1, 0x00},
	{0x24,  16, 1, 0x00},
	{0x24,  17, 1, 0x00},
	{0x24,  18, 1, 0x00},
	{0x24,  19, 1, 0x00},
	{0x24,  20, 1, 0x00},
	{0x24,  21, 1, 0x00},
	{0x24,  22, 1, 0x00},
	{0x24,  23, 1, 0x00},
	{0x24,  24, 1, 0x00},
	{0x24,  25, 1, 0x00},
	{0x24,  26, 1, 0x00},
	{0x24,  27, 1, 0x00},
	{0x24,  28, 1, 0x00},
	{0x24,  29, 1, 0x00},
	{0x24,  30, 1, 0x00},
	{0x24,  31, 1, 0x00},
	{0x31,   0, 1, 0x0A},
	{0x31,   1, 1, 0x0F},
	{0x31,   2, 1, 0x02},
	{0x31,   3, 1, 0x05},
	{0x31,   4, 1, 0x00},
	{0x31,   5, 1, 0x00},
	{0x31,   6, 1, 0x00},
	{0x31,   7, 1, 0x00},
	{0x31,   8, 1, 0x00},
	{0x31,   9, 1, 0x00},
	{0x31,  10, 1, 0x00},
	{0x31,  11, 1, 0x00},
	{0x31,  12, 1, 0x00},
	{0x31,  13, 1, 0x00},
	{0x31,  14, 1, 0x00},
	{0x31,  15, 1, 0x00},
	{0x31,  16, 1, 0x00},
	{0x31,  17, 1, 0x00},
	{0x31,  18, 1, 0x00},
	{0x31,  19, 1, 0x00},
	{0x31,  20, 1, 0x00},
	{0x31,  21, 1, 0x00},
	{0x31,  22, 1, 0x00},
	{0x31,  23, 1, 0x00},
	{0x31,  24, 1, 0x00},
	{0x31,  25, 1, 0x00},
	{0x31,  26, 1, 0x00},
	{0x31,  27, 1, 0x00},
	{0x31,  28, 1, 0x00},
	{0x31,  29, 1, 0x00},
	{0x31,  30, 1, 0x00},
	{0x31,  31, 1, 0x00},
	{0x40,   0, 1, 0x64},
	{0x40,   1, 1, 0x78},
	{0x40,   2, 1, 0x0F},
	{0x40,   3, 1, 0x9F},
	{0x40,   4, 1, 0x23},
	{0x40,   5, 1, 0x00},
	{0x40,   6, 1, 0x00},
	{0x40,   7, 1, 0x14},
	{0x40,   8, 1, 0x04},
	{0x40,   9, 1, 0x00},
	{0x40,  10, 1, 0x09},
	{0x40,  11, 1, 0x04},
	{0x40,  12, 1, 0x26},
	{0x40,  13, 1, 0x00},
	{0x40,  14, 1, 0x00},
	{0x40,  15, 1, 0x00},
	{0x40,  16, 1, 0x00},
	{0x40,  17, 1, 0x00},
	{0x40,  18, 1, 0x00},
	{0x40,  19, 1, 0x00},
	{0x40,  20, 1, 0x00},
	{0x40,  21, 1, 0x00},
	{0x40,  22, 1, 0x00},
	{0x40,  23, 1, 0x00},
	{0x40,  24, 1, 0x00},
	{0x40,  25, 1, 0x00},
	{0x40,  26, 1, 0x00},
	{0x40,  27, 1, 0x00},
	{0x40,  28, 1, 0x00},
	{0x40,  29, 1, 0x00},
	{0x40,  30, 1, 0x00},
	{0x40,  31, 1, 0x00},
	{0x44,   0, 1, 0x00},
	{0x44,   1, 1, 0x32},
	{0x44,   2, 1, 0x01},
	{0x44,   3, 1, 0xC2},
	{0x44,   4, 1, 0x30},
	{0x44,   5, 1, 0x00},
	{0x44,   6, 1, 0x03},
	{0x44,   7, 1, 0x08},
	{0x44,   8, 1, 0x98},
	{0x44,   9, 1, 0x01},
	{0x44,  10, 1, 0x00},
	{0x44,  11, 1, 0x3C},
	{0x44,  12, 1, 0x01},
	{0x44,  13, 1, 0x00},
	{0x44,  14, 1, 0x00},
	{0x44,  15, 1, 0x00},
	{0x44,  16, 1, 0x00},
	{0x44,  17, 1, 0x00},
	{0x44,  18, 1, 0x00},
	{0x44,  19, 1, 0x00},
	{0x44,  20, 1, 0x00},
	{0x44,  21, 1, 0x00},
	{0x44,  22, 1, 0x00},
	{0x44,  23, 1, 0x00},
	{0x44,  24, 1, 0x00},
	{0x44,  25, 1, 0x00},
	{0x44,  26, 1, 0x00},
	{0x44,  27, 1, 0x00},
	{0x44,  28, 1, 0x00},
	{0x44,  29, 1, 0x00},
	{0x44,  30, 1, 0x00},
	{0x44,  31, 1, 0x00},
	{0x50,   0, 1, 0x01},
	{0x50,   1, 1, 0xF4},
	{0x50,   2, 1, 0x00},
	{0x50,   3, 1, 0x1E},
	{0x50,   4, 1, 0xC8},
	{0x50,   5, 1, 0x14},
	{0x50,   6, 1, 0x08},
	{0x50,   7, 1, 0x00},
	{0x50,   8, 1, 0x3C},
	{0x50,   9, 1, 0x0E},
	{0x50,  10, 1, 0x10},
	{0x50,  11, 1, 0x00},
	{0x50,  12, 1, 0x0A},
	{0x50,  13, 1, 0x46},
	{0x50,  14, 1, 0x05},
	{0x50,  15, 1, 0x14},
	{0x50,  16, 1, 0x05},
	{0x50,  17, 1, 0x0F},
	{0x50,  18, 1, 0x03},
	{0x50,  19, 1, 0x20},
	{0x50,  20, 1, 0x7F},
	{0x50,  21, 1, 0xFF},
	{0x50,  22, 1, 0x00},
	{0x50,  23, 1, 0xF0},
	{0x50,  24, 1, 0x46},
	{0x50,  25, 1, 0x50},
	{0x50,  26, 1, 0x18},
	{0x50,  27, 1, 0x01},
	{0x50,  28, 1, 0x90},
	{0x50,  29, 1, 0x00},
	{0x50,  30, 1, 0x64},
	{0x50,  31, 1, 0x19},
	{0x50,  32, 1, 0xDC},
	{0x50,  33, 1, 0x5C},
	{0x50,  34, 1, 0x60},
	{0x50,  35, 1, 0x00},
	{0x50,  36, 1, 0x7D},
	{0x50,  37, 1, 0x00},
	{0x50,  38, 1, 0x04},
	{0x50,  39, 1, 0x03},
	{0x50,  40, 1, 0x19},
	{0x50,  41, 1, 0x25},
	{0x50,  42, 1, 0x0F},
	{0x50,  43, 1, 0x14},
	{0x50,  44, 1, 0x0A},
	{0x50,  45, 1, 0x78},
	{0x50,  46, 1, 0x60},
	{0x50,  47, 1, 0x28},
	{0x50,  48, 1, 0x01},
	{0x50,  49, 1, 0xF4},
	{0x50,  50, 1, 0x00},
	{0x50,  51, 1, 0x00},
	{0x50,  52, 1, 0x00},
	{0x50,  53, 1, 0x00},
	{0x50,  54, 1, 0x43},
	{0x50,  55, 1, 0x80},
	{0x50,  56, 1, 0x04},
	{0x50,  57, 1, 0x01},
	{0x50,  58, 1, 0x14},
	{0x50,  59, 1, 0x00},
	{0x50,  60, 1, 0x08},
	{0x50,  61, 1, 0x0B},
	{0x50,  62, 1, 0xB8},
	{0x50,  63, 1, 0x01},
	{0x50,  64, 1, 0x2C},
	{0x50,  65, 1, 0x0A},
	{0x50,  66, 1, 0x01},
	{0x50,  67, 1, 0x0A},
	{0x50,  68, 1, 0x00},
	{0x50,  69, 1, 0x00},
	{0x50,  70, 1, 0x00},
	{0x50,  71, 1, 0xC8},
	{0x50,  72, 1, 0x00},
	{0x50,  73, 1, 0x64},
	{0x50,  74, 1, 0x02},
	{0x50,  75, 1, 0x00},
	{0x50,  76, 1, 0x00},
	{0x50,  77, 1, 0x00},
	{0x50,  78, 1, 0x00},
	{0x50,  79, 1, 0x07},
	{0x50,  80, 1, 0xD0},
	{0x50,  81, 1, 0x01},
	{0x50,  82, 1, 0x03},
	{0x50,  83, 1, 0x5A},
	{0x50,  84, 1, 0x14},
	{0x50,  85, 1, 0x00},
	{0x50,  86, 1, 0x00},
	{0x50,  87, 1, 0x00},
	{0x50,  88, 1, 0x00},
	{0x50,  89, 1, 0x00},
	{0x50,  90, 1, 0x00},
	{0x50,  91, 1, 0x00},
	{0x50,  92, 1, 0x00},
	{0x50,  93, 1, 0x00},
	{0x50,  94, 1, 0x00},
	{0x50,  95, 1, 0x00},
	{0x51,   0, 1, 0x00},
	{0x51,   1, 1, 0xA7},
	{0x51,   2, 1, 0x00},
	{0x51,   3, 1, 0x64},
	{0x51,   4, 1, 0x00},
	{0x51,   5, 1, 0xFA},
	{0x51,   6, 1, 0x00},
	{0x51,   7, 1, 0x3C},
	{0x51,   8, 1, 0x3C},
	{0x51,   9, 1, 0x01},
	{0x51,  10, 1, 0xB3},
	{0x51,  11, 1, 0xB3},
	{0x51,  12, 1, 0x01},
	{0x51,  13, 1, 0x90},
	{0x51,  14, 1, 0x00},
	{0x51,  15, 1, 0x00},
	{0x51,  16, 1, 0x00},
	{0x51,  17, 1, 0x00},
	{0x51,  18, 1, 0x00},
	{0x51,  19, 1, 0x00},
	{0x51,  20, 1, 0x00},
	{0x51,  21, 1, 0x00},
	{0x51,  22, 1, 0x00},
	{0x51,  23, 1, 0x00},
	{0x51,  24, 1, 0x00},
	{0x51,  25, 1, 0x00},
	{0x51,  26, 1, 0x00},
	{0x51,  27, 1, 0x00},
	{0x51,  28, 1, 0x00},
	{0x51,  29, 1, 0x00},
	{0x51,  30, 1, 0x00},
	{0x51,  31, 1, 0x00},
	{0x52,   0, 1, 0x41},
	{0x52,   1, 1, 0x60},
	{0x52,   2, 1, 0x01},
	{0x52,   3, 1, 0x00},
	{0x52,   4, 1, 0x00},
	{0x52,   5, 1, 0x01},
	{0x52,   6, 1, 0x04},
	{0x52,   7, 1, 0x2E},
	{0x52,   8, 1, 0x0F},
	{0x52,   9, 1, 0x0C},
	{0x52,  10, 1, 0x0D},
	{0x52,  11, 1, 0x48},
	{0x52,  12, 1, 0x00},
	{0x52,  13, 1, 0xC8},
	{0x52,  14, 1, 0x00},
	{0x52,  15, 1, 0x32},
	{0x52,  16, 1, 0x00},
	{0x52,  17, 1, 0x14},
	{0x52,  18, 1, 0x03},
	{0x52,  19, 1, 0xE8},
	{0x52,  20, 1, 0x01},
	{0x52,  21, 1, 0x00},
	{0x52,  22, 1, 0xC8},
	{0x52,  23, 1, 0x00},
	{0x52,  24, 1, 0x0A},
	{0x52,  25, 1, 0xFF},
	{0x52,  26, 1, 0xDD},
	{0x52,  27, 1, 0xFF},
	{0x52,  28, 1, 0xDE},
	{0x52,  29, 1, 0x00},
	{0x52,  30, 1, 0x03},
	{0x52,  31, 1, 0x00},
	{0x53,   0, 1, 0x17},
	{0x53,   1, 1, 0x16},
	{0x53,   2, 1, 0x10},
	{0x53,   3, 1, 0xFD},
	{0x53,   4, 1, 0xE0},
	{0x53,   5, 1, 0xE0},
	{0x53,   6, 1, 0xE2},
	{0x53,   7, 1, 0xE2},
	{0x53,   8, 1, 0xE3},
	{0x53,   9, 1, 0xE3},
	{0x53,  10, 1, 0xE4},
	{0x53,  11, 1, 0xE5},
	{0x53,  12, 1, 0xE6},
	{0x53,  13, 1, 0xE7},
	{0x53,  14, 1, 0xE9},
	{0x53,  15, 1, 0xE9},
	{0x53,  16, 1, 0xEA},
	{0x53,  17, 1, 0xE9},
	{0x53,  18, 1, 0xEC},
	{0x53,  19, 1, 0xEA},
	{0x53,  20, 1, 0xDE},
	{0x53,  21, 1, 0xEE},
	{0x53,  22, 1, 0xEE},
	{0x53,  23, 1, 0xF2},
	{0x53,  24, 1, 0xF4},
	{0x53,  25, 1, 0xF6},
	{0x53,  26, 1, 0xF5},
	{0x53,  27, 1, 0xF8},
	{0x53,  28, 1, 0xF8},
	{0x53,  29, 1, 0xF9},
	{0x53,  30, 1, 0xFB},
	{0x53,  31, 1, 0xFA},
	{0x53,  32, 1, 0xF9},
	{0x53,  33, 1, 0xF5},
	{0x53,  34, 1, 0xF3},
	{0x53,  35, 1, 0xEF},
	{0x53,  36, 1, 0xF2},
	{0x53,  37, 1, 0xE4},
	{0x53,  38, 1, 0xFD},
	{0x53,  39, 1, 0xF3},
	{0x53,  40, 1, 0xF8},
	{0x53,  41, 1, 0xD4},
	{0x53,  42, 1, 0x80},
	{0x53,  43, 1, 0x00},
	{0x53,  44, 1, 0x00},
	{0x53,  45, 1, 0x00},
	{0x53,  46, 1, 0x00},
	{0x53,  47, 1, 0x00},
	{0x53,  48, 1, 0x00},
	{0x53,  49, 1, 0x00},
	{0x53,  50, 1, 0x00},
	{0x53,  51, 1, 0x00},
	{0x53,  52, 1, 0x00},
	{0x53,  53, 1, 0x00},
	{0x53,  54, 1, 0x00},
	{0x53,  55, 1, 0x00},
	{0x53,  56, 1, 0x00},
	{0x53,  57, 1, 0x00},
	{0x53,  58, 1, 0x00},
	{0x53,  59, 1, 0x00},
	{0x53,  60, 1, 0x00},
	{0x53,  61, 1, 0x00},
	{0x53,  62, 1, 0x00},
	{0x53,  63, 1, 0x00},
	{0x54,   0, 1, 0xFB},
	{0x54,   1, 1, 0xD3},
	{0x54,   2, 1, 0x04},
	{0x54,   3, 1, 0x0B},
	{0x54,   4, 1, 0x0C},
	{0x54,   5, 1, 0x03},
	{0x54,   6, 1, 0x04},
	{0x54,   7, 1, 0x03},
	{0x54,   8, 1, 0x04},
	{0x54,   9, 1, 0x04},
	{0x54,  10, 1, 0x00},
	{0x54,  11, 1, 0x04},
	{0x54,  12, 1, 0x00},
	{0x54,  13, 1, 0x00},
	{0x54,  14, 1, 0x01},
	{0x54,  15, 1, 0x0B},
	{0x54,  16, 1, 0x09},
	{0x54,  17, 1, 0x00},
	{0x54,  18, 1, 0xF9},
	{0x54,  19, 1, 0x0A},
	{0x54,  20, 1, 0xFF},
	{0x54,  21, 1, 0x0A},
	{0x54,  22, 1, 0x05},
	{0x54,  23, 1, 0xFD},
	{0x54,  24, 1, 0xFD},
	{0x54,  25, 1, 0x07},
	{0x54,  26, 1, 0xFB},
	{0x54,  27, 1, 0xFF},
	{0x54,  28, 1, 0xF4},
	{0x54,  29, 1, 0xEB},
	{0x54,  30, 1, 0xC9},
	{0x54,  31, 1, 0xCE},
	{0x54,  32, 1, 0x00},
	{0x54,  33, 1, 0x0F},
	{0x54,  34, 1, 0x0B},
	{0x54,  35, 1, 0xF2},
	{0x54,  36, 1, 0x56},
	{0x54,  37, 1, 0xE6},
	{0x54,  38, 1, 0x19},
	{0x54,  39, 1, 0x86},
	{0x54,  40, 1, 0x0F},
	{0x54,  41, 1, 0x81},
	{0x54,  42, 1, 0x00},
	{0x54,  43, 1, 0x00},
	{0x54,  44, 1, 0x00},
	{0x54,  45, 1, 0x00},
	{0x54,  46, 1, 0x00},
	{0x54,  47, 1, 0x00},
	{0x54,  48, 1, 0x00},
	{0x54,  49, 1, 0x00},
	{0x54,  50, 1, 0x00},
	{0x54,  51, 1, 0x00},
	{0x54,  52, 1, 0x00},
	{0x54,  53, 1, 0x00},
	{0x54,  54, 1, 0x00},
	{0x54,  55, 1, 0x00},
	{0x54,  56, 1, 0x00},
	{0x54,  57, 1, 0x00},
	{0x54,  58, 1, 0x00},
	{0x54,  59, 1, 0x00},
	{0x54,  60, 1, 0x00},
	{0x54,  61, 1, 0x00},
	{0x54,  62, 1, 0x00},
	{0x54,  63, 1, 0x00},
	{0x55,   0, 1, 0xFF},
	{0x55,   1, 1, 0x46},
	{0x55,   2, 1, 0x00},
	{0x55,   3, 1, 0x02},
	{0x55,   4, 1, 0xFD},
	{0x55,   5, 1, 0xFB},
	{0x55,   6, 1, 0x06},
	{0x55,   7, 1, 0x07},
	{0x55,   8, 1, 0xF1},
	{0x55,   9, 1, 0x04},
	{0x55,  10, 1, 0x01},
	{0x55,  11, 1, 0xFE},
	{0x55,  12, 1, 0xD1},
	{0x55,  13, 1, 0x33},
	{0x55,  14, 1, 0x45},
	{0x55,  15, 1, 0xF6},
	{0x55,  16, 1, 0x6B},
	{0x55,  17, 1, 0x00},
	{0x55,  18, 1, 0x00},
	{0x55,  19, 1, 0x00},
	{0x55,  20, 1, 0x00},
	{0x55,  21, 1, 0x00},
	{0x55,  22, 1, 0x00},
	{0x55,  23, 1, 0x00},
	{0x55,  24, 1, 0x00},
	{0x55,  25, 1, 0x00},
	{0x55,  26, 1, 0x00},
	{0x55,  27, 1, 0x00},
	{0x55,  28, 1, 0x00},
	{0x55,  29, 1, 0x00},
	{0x55,  30, 1, 0x00},
	{0x55,  31, 1, 0x00},
	{0x6C,   0, 1, 0xFE},
	{0x6C,   1, 1, 0x8F},
	{0x6C,   2, 1, 0x00},
	{0x6C,   3, 1, 0xFD},
	{0x6C,   4, 1, 0x0F},
	{0x6C,   5, 1, 0x47},
	{0x6C,   6, 1, 0xD2},
	{0x6C,   7, 1, 0xE5},
	{0x6C,   8, 1, 0x03},
	{0x6C,   9, 1, 0xBC},
	{0x6C,  10, 1, 0x00},
	{0x6C,  11, 1, 0xFA},
	{0x6C,  12, 1, 0x37},
	{0x6C,  13, 1, 0xA0},
	{0x6C,  14, 1, 0xEE},
	{0x6C,  15, 1, 0xCF},
	{0x6C,  16, 1, 0x6C},
	{0x6C,  17, 1, 0x00},
	{0x6C,  18, 1, 0x00},
	{0x6C,  19, 1, 0x00},
	{0x6C,  20, 1, 0x00},
	{0x6C,  21, 1, 0x00},
	{0x6C,  22, 1, 0x00},
	{0x6C,  23, 1, 0x00},
	{0x6C,  24, 1, 0x00},
	{0x6C,  25, 1, 0x00},
	{0x6C,  26, 1, 0x00},
	{0x6C,  27, 1, 0x00},
	{0x6C,  28, 1, 0x00},
	{0x6C,  29, 1, 0x00},
	{0x6C,  30, 1, 0x00},
	{0x6C,  31, 1, 0x00},
	{0x59,   0, 1, 0x00},
	{0x59,   1, 1, 0x4D},
	{0x59,   2, 1, 0x00},
	{0x59,   3, 1, 0x51},
	{0x59,   4, 1, 0x00},
	{0x59,   5, 1, 0x5D},
	{0x59,   6, 1, 0x00},
	{0x59,   7, 1, 0x6F},
	{0x59,   8, 1, 0x00},
	{0x59,   9, 1, 0x52},
	{0x59,  10, 1, 0x00},
	{0x59,  11, 1, 0x54},
	{0x59,  12, 1, 0x00},
	{0x59,  13, 1, 0x61},
	{0x59,  14, 1, 0x00},
	{0x59,  15, 1, 0x5D},
	{0x59,  16, 1, 0x00},
	{0x59,  17, 1, 0x62},
	{0x59,  18, 1, 0x00},
	{0x59,  19, 1, 0x69},
	{0x59,  20, 1, 0x00},
	{0x59,  21, 1, 0x76},
	{0x59,  22, 1, 0x00},
	{0x59,  23, 1, 0x71},
	{0x59,  24, 1, 0x00},
	{0x59,  25, 1, 0x96},
	{0x59,  26, 1, 0x00},
	{0x59,  27, 1, 0xEF},
	{0x59,  28, 1, 0x02},
	{0x59,  29, 1, 0x5B},
	{0x59,  30, 1, 0x00},
	{0x59,  31, 1, 0x00},
	{0x6D,   0, 1, 0x04},
	{0x6D,   1, 1, 0x37},
	{0x6D,   2, 1, 0x0E},
	{0x6D,   3, 1, 0xE7},
	{0x6D,   4, 1, 0x0E},
	{0x6D,   5, 1, 0xB6},
	{0x6D,   6, 1, 0x10},
	{0x6D,   7, 1, 0x5F},
	{0x6D,   8, 1, 0x10},
	{0x6D,   9, 1, 0x04},
	{0x6D,  10, 1, 0x00},
	{0x6D,  11, 1, 0x00},
	{0x6D,  12, 1, 0x00},
	{0x6D,  13, 1, 0x00},
	{0x6D,  14, 1, 0x00},
	{0x6D,  15, 1, 0x00},
	{0x6D,  16, 1, 0x00},
	{0x6D,  17, 1, 0x00},
	{0x6D,  18, 1, 0x00},
	{0x6D,  19, 1, 0x00},
	{0x6D,  20, 1, 0x00},
	{0x6D,  21, 1, 0x00},
	{0x6D,  22, 1, 0x00},
	{0x6D,  23, 1, 0x00},
	{0x6D,  24, 1, 0x00},
	{0x6D,  25, 1, 0x00},
	{0x6D,  26, 1, 0x00},
	{0x6D,  27, 1, 0x00},
	{0x6D,  28, 1, 0x00},
	{0x6D,  29, 1, 0x00},
	{0x6D,  30, 1, 0x00},
	{0x6D,  31, 1, 0x00},
	{0x70,   0, 1, 0x80},
	{0x70,   1, 1, 0x00},
	{0x70,   2, 1, 0x80},
	{0x70,   3, 1, 0x00},
	{0x70,   4, 1, 0x00},
	{0x70,   5, 1, 0x00},
	{0x70,   6, 1, 0x00},
	{0x70,   7, 1, 0x00},
	{0x70,   8, 1, 0x00},
	{0x70,   9, 1, 0x00},
	{0x70,  10, 1, 0x00},
	{0x70,  11, 1, 0x00},
	{0x70,  12, 1, 0x00},
	{0x70,  13, 1, 0x00},
	{0x70,  14, 1, 0x00},
	{0x70,  15, 1, 0x00},
	{0x70,  16, 1, 0x00},
	{0x70,  17, 1, 0x00},
	{0x70,  18, 1, 0x00},
	{0x70,  19, 1, 0x00},
	{0x70,  20, 1, 0x00},
	{0x70,  21, 1, 0x00},
	{0x70,  22, 1, 0x00},
	{0x70,  23, 1, 0x00},
	{0x70,  24, 1, 0x00},
	{0x70,  25, 1, 0x00},
	{0x70,  26, 1, 0x00},
	{0x70,  27, 1, 0x00},
	{0x70,  28, 1, 0x00},
	{0x70,  29, 1, 0x00},
	{0x70,  30, 1, 0x00},
	{0x70,  31, 1, 0x00},

#endif
};

static struct dm_reg bq276xx_dm_regs[] = {
	{64, 2, 1, 0x2C},	/* Op Config B */
	{82, 0, 2, 1000},	/* Qmax */
	{82, 2, 1, 0x81},	/* Load Select */
	{82, 3, 2, 1340},	/* Design Capacity */
	{82, 5, 2, 3700},	/* Design Energy */
	{82, 9, 2, 3250},	/* Terminate Voltage */
	{82, 20, 2, 110},	/* Taper rate */
};

static unsigned int poll_interval = 360;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");

/*
 * Forward Declarations
 */
static int read_dm_block(struct bq27x00_device_info *di, u8 subclass,
	u8 offset, u8 *data);


/*
 * Common code for BQ27x00 devices
 */

static inline int bq27xxx_read(struct bq27x00_device_info *di, int reg_index,
		bool single)
{
	int val;

	/* Reports 0 for invalid/missing registers */
	if (!di || di->regs[reg_index] == INVALID_REG_ADDR)
		return 0;

	val = di->bus.read(di, di->regs[reg_index], single);

	return val;
}

static inline int bq27xxx_write(struct bq27x00_device_info *di, int reg_index,
		int value, bool single)
{
	if (!di || di->regs[reg_index] == INVALID_REG_ADDR)
		return -1;

	return di->bus.write(di, di->regs[reg_index], value, single);
}

static int control_cmd_wr(struct bq27x00_device_info *di, u16 cmd)
{
	dev_dbg(di->dev, "%s: cmd - %04x\n", __func__, cmd);

	return di->bus.write(di, BQ27XXX_REG_CTRL, cmd, false);
}

static int control_cmd_read(struct bq27x00_device_info *di, u16 cmd)
{
	dev_dbg(di->dev, "%s: cmd - %04x\n", __func__, cmd);

	di->bus.write(di, BQ27XXX_REG_CTRL, cmd, false);

	msleep(5);

	return di->bus.read(di, BQ27XXX_REG_CTRL, false);
}
/*
 * It is assumed that the gauge is in unsealed mode when this function
 * is called
 */
static int bq276xx_seal_enabled(struct bq27x00_device_info *di)
{
	u8 buf[32];
	u8 op_cfg_b;

	if (!read_dm_block(di, BQ276XX_OP_CFG_B_SUBCLASS,
		BQ276XX_OP_CFG_B_OFFSET, buf)) {
		return 1; /* Err on the side of caution and try to seal */
	}

	op_cfg_b = buf[BQ276XX_OP_CFG_B_OFFSET & 0x1F];

	if (op_cfg_b & BQ276XX_OP_CFG_B_DEF_SEAL_BIT)
		return 1;

	return 0;
}

#define SEAL_UNSEAL_POLLING_RETRY_LIMIT	1000

static inline int sealed(struct bq27x00_device_info *di)
{
	return control_cmd_read(di, CONTROL_STATUS_SUBCMD) & (1 << 13);
}

static int unseal(struct bq27x00_device_info *di, u32 key)
{
	int i = 0;

	dev_dbg(di->dev, "%s: key - %08x\n", __func__, key);

	if (!sealed(di))
		goto out;

	di->bus.write(di, BQ27XXX_REG_CTRL, key & 0xFFFF, false);
	msleep(5);
	di->bus.write(di, BQ27XXX_REG_CTRL, (key & 0xFFFF0000) >> 16, false);
	msleep(5);

	while (i < SEAL_UNSEAL_POLLING_RETRY_LIMIT) {
		i++;
		if (!sealed(di))
			break;
		msleep(10);
	}

out:
	if (i == SEAL_UNSEAL_POLLING_RETRY_LIMIT) {
		dev_err(di->dev, "%s: failed\n", __func__);
		return 0;
	} else {
		return 1;
	}
}

static int seal(struct bq27x00_device_info *di)
{
	int i = 0;
	int is_sealed;

	dev_dbg(di->dev, "%s:\n", __func__);

	is_sealed = sealed(di);
	if (is_sealed)
		return is_sealed;

	if (di->chip == BQ276XX && !bq276xx_seal_enabled(di)) {
		dev_dbg(di->dev, "%s: sealing is not enabled\n", __func__);
		return is_sealed;
	}

	di->bus.write(di, BQ27XXX_REG_CTRL, SEAL_SUBCMD, false);

	while (i < SEAL_UNSEAL_POLLING_RETRY_LIMIT) {
		i++;
		is_sealed = sealed(di);
		if (is_sealed)
			break;
		msleep(10);
	}

	if (!is_sealed)
		dev_err(di->dev, "%s: failed\n", __func__);

	return is_sealed;
}

#define CFG_UPDATE_POLLING_RETRY_LIMIT 50
static int enter_cfg_update_mode(struct bq27x00_device_info *di)
{
	int i = 0;
	u16 flags;

	dev_dbg(di->dev, "%s:\n", __func__);

	if (!unseal(di, BQ274XX_UNSEAL_KEY))
		return 0;

	control_cmd_wr(di, SET_CFGUPDATE_SUBCMD);
	msleep(5);

	while (i < CFG_UPDATE_POLLING_RETRY_LIMIT) {
		i++;
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
		if (flags & (1 << 4))
			break;
		msleep(100);
	}

	if (i == CFG_UPDATE_POLLING_RETRY_LIMIT) {
		dev_err(di->dev, "%s: failed %04x\n", __func__, flags);
		return 0;
	}

	return 1;
}

static int exit_cfg_update_mode(struct bq27x00_device_info *di)
{
	int i = 0;
	u16 flags;

	dev_dbg(di->dev, "%s:\n", __func__);

	control_cmd_wr(di, BQ274XX_SOFT_RESET);

	while (i < CFG_UPDATE_POLLING_RETRY_LIMIT) {
		i++;
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
		if (!(flags & (1 << 4)))
			break;
		msleep(100);
	}

	if (i == CFG_UPDATE_POLLING_RETRY_LIMIT) {
		dev_err(di->dev, "%s: failed %04x\n", __func__, flags);
		return 0;
	}

	if (seal(di))
		return 1;
	else
		return 0;
}
static u8 checksum(u8 *data)
{
	u16 sum = 0;
	int i;

	for (i = 0; i < 32; i++)
		sum += data[i];

	sum &= 0xFF;

	return 0xFF - sum;
}

#ifdef DEBUG
static void print_buf(const char *msg, u8 *buf)
{
	int i;

	printk("\nbq: %s buf: ", msg);
	for (i = 0; i < 32; i++)
		printk("%02x ", buf[i]);

	printk("\n");
}
#else
#define print_buf(a, b)
#endif

static int update_dm_block(struct bq27x00_device_info *di, u8 subclass,
	u8 offset, u8 *data)
{
//	u8 buf[32];
	u8 buf[100]; //The largest dm_block found is subclass 0x50, which has 96 bytes
	u8 cksum;
	u8 blk_offset = offset >> 5;

	dev_info(di->dev, "%s: subclass %d offset %d\n",
		__func__, subclass, offset);

	di->bus.write(di, BLOCK_DATA_CONTROL, 0, true);
	msleep(5);

	di->bus.write(di, BLOCK_DATA_CLASS, subclass, true);
	msleep(5);

	di->bus.write(di, DATA_BLOCK, blk_offset, true);
	msleep(5);

	di->bus.blk_write(di, BLOCK_DATA, data, 32);
	msleep(5);
	print_buf(__func__, data);

	cksum = checksum(data);
	di->bus.write(di, BLOCK_DATA_CHECKSUM, cksum, true);
	msleep(5);

	/* Read back and compare to make sure write is successful */
	di->bus.write(di, DATA_BLOCK, blk_offset, true);
	msleep(5);
	di->bus.blk_read(di, BLOCK_DATA, buf, 32);
	if (memcmp(data, buf, 32)) {
		dev_err(di->dev, "%s: error updating subclass %d offset %d\n",
			__func__, subclass, offset);
		return 0;
	} else {
		return 1;
	}
}

static int read_dm_block(struct bq27x00_device_info *di, u8 subclass,
	u8 offset, u8 *data)
{
	u8 cksum_calc, cksum;
	u8 blk_offset = offset >> 5;

	dev_dbg(di->dev, "%s: subclass %d offset %d\n",
		__func__, subclass, offset);

	di->bus.write(di, BLOCK_DATA_CONTROL, 0, true);
	msleep(5);

	di->bus.write(di, BLOCK_DATA_CLASS, subclass, true);
	msleep(5);

	di->bus.write(di, DATA_BLOCK, blk_offset, true);
	msleep(5);

	di->bus.blk_read(di, BLOCK_DATA, data, 32);

	cksum_calc = checksum(data);
	cksum = di->bus.read(di, BLOCK_DATA_CHECKSUM, true);
	if (cksum != cksum_calc) {
		dev_err(di->dev, "%s: error reading subclass %d offset %d\n",
			__func__, subclass, offset);
		return 0;
	}

	print_buf(__func__, data);

	return 1;
}

/*
 * Return the battery State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_soc(struct bq27x00_device_info *di)
{
	int soc;

	soc = bq27xxx_read(di, BQ27XXX_REG_SOC, false);

	if (soc < 0)
		dev_dbg(di->dev, "error reading relative State-of-Charge\n");

	return soc;
}

/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_charge(struct bq27x00_device_info *di, u8 reg)
{
	int charge;

	charge = bq27xxx_read(di, reg, false);
	if (charge < 0) {
		dev_dbg(di->dev, "error reading charge register %02x: %d\n",
			reg, charge);
		return charge;
	}

	if (di->chip == BQ27200)
		charge = charge * 3570 / BQ27200_RS;
	else
		charge *= 1000;

	return charge;
}

/*
 * Return the battery Nominal available capaciy in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_nac(struct bq27x00_device_info *di)
{
	int flags;

	if (di->chip == BQ27200) {
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, true);
		if (flags >= 0 && (flags & BQ27200_FLAG_CI))
			return -ENODATA;
	}

	return bq27x00_battery_read_charge(di, BQ27XXX_REG_NAC);
}

/*
 * Return the battery Last measured discharge in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_fcc(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27XXX_REG_FCC);
}

/*
 * Return the Design Capacity in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_dcap(struct bq27x00_device_info *di)
{
	int dcap;

	dcap = bq27xxx_read(di, BQ27XXX_REG_DCAP, false);

	if (dcap < 0) {
		dev_dbg(di->dev, "error reading initial last measured discharge, returning hard-coded value\n");
		dcap = HARDCODED_DCAP;
	}

	if (di->chip == BQ27200)
		dcap = dcap * 256 * 3570 / BQ27200_RS;
	else
		dcap *= 1000;

	return dcap;
}

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_energy(struct bq27x00_device_info *di)
{
	int ae;

	ae = bq27xxx_read(di, BQ27XXX_REG_AE, false);
	if (ae < 0) {
		dev_dbg(di->dev, "error reading available energy\n");
		return ae;
	}

	if (di->chip == BQ27200)
		ae = ae * 29200 / BQ27200_RS;
	else
		ae *= 1000;

	return ae;
}

/*
 * Return the battery temperature in tenths of degree Kelvin
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_temperature(struct bq27x00_device_info *di)
{
	int temp;

	temp = bq27xxx_read(di, BQ27XXX_REG_TEMP, false);
	if (temp < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return temp;
	}

	if (di->chip == BQ27200)
		temp = 5 * temp / 2;

	return temp;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_cyct(struct bq27x00_device_info *di)
{
	int cyct;

	cyct = bq27xxx_read(di, BQ27XXX_REG_CYCT, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_time(struct bq27x00_device_info *di, u8 reg)
{
	int tval;

	tval = bq27xxx_read(di, reg, false);
	if (tval < 0) {
		dev_dbg(di->dev, "error reading time register %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}

/*
 * Read a power avg register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_pwr_avg(struct bq27x00_device_info *di, u8 reg)
{
	int tval;

	tval = bq27xxx_read(di, reg, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading power avg rgister  %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (di->chip == BQ27200)
		return (tval * BQ27200_POWER_CONSTANT) / BQ27200_RS;
	else
		return tval;
}

static int overtemperature(struct bq27x00_device_info *di, u16 flags)
{
	if (di->chip == BQ27520)
		return flags & (BQ27XXX_FLAG_OTC | BQ27XXX_FLAG_OTD);
	else
		return flags & BQ27XXX_FLAG_OTC;
}

/*
 * Read flag register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_health(struct bq27x00_device_info *di)
{
	u16 tval;

	tval = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading flag register:%d\n", tval);
		return tval;
	}

	if ((di->chip == BQ27200)) {
		if (tval & BQ27200_FLAG_EDV1)
			tval = POWER_SUPPLY_HEALTH_DEAD;
		else
			tval = POWER_SUPPLY_HEALTH_GOOD;
		return tval;
	} else {
		if (tval & BQ27XXX_FLAG_SOCF)
			tval = POWER_SUPPLY_HEALTH_DEAD;
		else if (overtemperature(di, tval))
			tval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			tval = POWER_SUPPLY_HEALTH_GOOD;
		return tval;
	}

	return -1;
}

static void bq27x00_update(struct bq27x00_device_info *di)
{
	struct bq27x00_reg_cache cache = {0, };
	bool is_bq27200 = di->chip == BQ27200;
	bool is_bq27500 = di->chip == BQ27500;
	bool is_bq274xx = di->chip == BQ274XX;
	bool is_bq276xx = di->chip == BQ276XX;

	cache.flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, !is_bq27500);
	if (cache.flags >= 0) {
		if (is_bq27200 && (cache.flags & BQ27200_FLAG_CI)) {
			dev_info(di->dev, "battery is not calibrated! ignoring capacity values\n");
			cache.capacity = -ENODATA;
			cache.energy = -ENODATA;
			cache.time_to_empty = -ENODATA;
			cache.time_to_empty_avg = -ENODATA;
			cache.time_to_full = -ENODATA;
			cache.charge_full = -ENODATA;
			cache.health = -ENODATA;
		} else {
			cache.capacity = bq27x00_battery_read_soc(di);
			if (!(is_bq274xx || is_bq276xx)) {
				cache.energy = bq27x00_battery_read_energy(di);
				cache.time_to_empty =
					bq27x00_battery_read_time(di,
							BQ27XXX_REG_TTE);
				cache.time_to_empty_avg =
					bq27x00_battery_read_time(di,
							BQ27XXX_REG_TTECP);
				cache.time_to_full =
					bq27x00_battery_read_time(di,
							BQ27XXX_REG_TTF);
			}
			cache.charge_full = bq27x00_battery_read_fcc(di);
			cache.health = bq27x00_battery_read_health(di);
		}
		cache.temperature = bq27x00_battery_read_temperature(di);
		if (!is_bq274xx)
			cache.cycle_count = bq27x00_battery_read_cyct(di);
		cache.power_avg =
			bq27x00_battery_read_pwr_avg(di, BQ27XXX_POWER_AVG);

		/* We only have to read charge design full once */
		if (di->charge_design_full <= 0)
			di->charge_design_full = bq27x00_battery_read_dcap(di);
	}

	if (memcmp(&di->cache, &cache, sizeof(cache)) != 0) {
		di->cache = cache;
		power_supply_changed(di->bat);
	}

	di->last_update = jiffies;
}

static void copy_to_dm_buf_big_endian(struct bq27x00_device_info *di,
	u8 *buf, u8 offset, u8 sz, u32 val)
{
	dev_info(di->dev, "%s: offset %d sz %d val %d\n",
		__func__, offset, sz, val);

	switch (sz) {
	case 1:
		buf[offset] = (u8) val;
		break;
	case 2:
		put_unaligned_be16((u16) val, &buf[offset]);
		break;
	case 4:
		put_unaligned_be32(val, &buf[offset]);
		break;
	default:
		dev_err(di->dev, "%s: bad size for dm parameter - %d",
			__func__, sz);
		break;
	}
}

static int rom_mode_gauge_init_completed(struct bq27x00_device_info *di)
{
	dev_dbg(di->dev, "%s:\n", __func__);

	return control_cmd_read(di, CONTROL_STATUS_SUBCMD) &
		BQ274XX_CTRL_STATUS_INITCOMP;
}

static bool rom_mode_gauge_dm_initialized(struct bq27x00_device_info *di)
{
#if 1 
	u16 flags;

	flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);

	dev_info(di->dev, "%s: flags - 0x%04x\n", __func__, flags);

	if (flags & BQ274XX_FLAG_ITPOR)
		return false;
	else
		return true;
#else
	static int oneshot_init = 0;
	if(oneshot_init == 0) {
		oneshot_init = 1;
		return false;
	}
	else
		return true;
#endif
}

#define INITCOMP_TIMEOUT_MS		10000
u8 dm_buf[1024];
static void rom_mode_gauge_dm_init(struct bq27x00_device_info *di)
{
	int i;
	int timeout = INITCOMP_TIMEOUT_MS;
	u8 subclass, offset;
	u32 blk_number;
	u32 blk_number_prev = 0;
//	u8 buf[1024];
	bool buf_valid = false;
	struct dm_reg *dm_reg;

	dev_dbg(di->dev, "%s:\n", __func__);

	while (!rom_mode_gauge_init_completed(di) && timeout > 0) {
		msleep(100);
		timeout -= 100;
	}

	if (timeout <= 0) {
		dev_err(di->dev, "%s: INITCOMP not set after %d seconds\n",
			__func__, INITCOMP_TIMEOUT_MS/100);
		return;
	}

	if (!di->dm_regs || !di->dm_regs_count) {
		dev_err(di->dev, "%s: Data not available for DM initialization\n",
			__func__);
		return;
	}
	dev_info(di->dev, "%s, dm_regs=0x%x, dm_regs_count=%d\n",__func__, di->dm_regs, di->dm_regs_count);
	enter_cfg_update_mode(di);
	for (i = 0; i < di->dm_regs_count; i++) {
		dm_reg = &di->dm_regs[i];
		subclass = dm_reg->subclass;
		offset = dm_reg->offset;

		/*
		 * Create a composite block number to see if the subsequent
		 * register also belongs to the same 32 btye block in the DM
		 */
		blk_number = subclass << 8;
		blk_number |= offset >> 5;

		if (blk_number == blk_number_prev) {
			copy_to_dm_buf_big_endian(di, dm_buf, offset,
				dm_reg->len, dm_reg->data);
		} else {

			if (buf_valid)
				update_dm_block(di, blk_number_prev >> 8,
					(blk_number_prev << 5) & 0xFF , dm_buf);
			else
				buf_valid = true;

			read_dm_block(di, dm_reg->subclass, dm_reg->offset,
				dm_buf);
			copy_to_dm_buf_big_endian(di, dm_buf, offset,
				dm_reg->len, dm_reg->data);
		}
		blk_number_prev = blk_number;
	}

	/* Last buffer to be written */
	if (buf_valid)
		update_dm_block(di, subclass, offset, dm_buf);

	exit_cfg_update_mode(di);
}

static void bq27x00_battery_poll(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, work.work);

	if (((di->chip == BQ274XX) || (di->chip == BQ276XX)) &&
		!rom_mode_gauge_dm_initialized(di)) {
		rom_mode_gauge_dm_init(di);
	}

	bq27x00_update(di);

	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
		set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&di->work, msecs_to_jiffies(poll_interval * HZ));
	}
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int curr;
	int flags;

	curr = bq27xxx_read(di, BQ27XXX_REG_AI, false);
	if (curr < 0) {
		dev_err(di->dev, "error reading current\n");
		return curr;
	}

	if (di->chip == BQ27200) {
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
		if (flags & BQ27200_FLAG_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}

		val->intval = curr * 3570 / BQ27200_RS;
	} else {
		/* Other gauges return signed value */
		val->intval = (int)((s16)curr) * 1000;
	}

	return 0;
}

static int bq27x00_battery_status(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int status;

	if (di->chip == BQ27200) {
		if (di->cache.flags & BQ27200_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27200_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else if (power_supply_am_i_supplied(di->bat))
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		if (di->cache.flags & BQ27XXX_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27XXX_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	}

	val->intval = status;

	return 0;
}

static int bq27x00_battery_capacity_level(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int level;

	if (di->chip == BQ27200) {
		if (di->cache.flags & BQ27200_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27200_FLAG_EDV1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27200_FLAG_EDVF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	} else {
		if (di->cache.flags & BQ27XXX_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27XXX_FLAG_SOC1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27XXX_FLAG_SOCF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}

	val->intval = level;

	return 0;
}

/*
 * Return the battery Voltage in millivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int volt;

	volt = bq27xxx_read(di, BQ27XXX_REG_VOLT, false);
	if (volt < 0) {
		dev_err(di->dev, "error reading voltage\n");
		return volt;
	}

	val->intval = volt * 1000;

	return 0;
}

static int bq27x00_simple_value(int value,
	union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di = psy->drv_data;
	
	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
		cancel_delayed_work_sync(&di->work);
		bq27x00_battery_poll(&di->work.work);
	}
	mutex_unlock(&di->lock);

	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27x00_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq27x00_battery_voltage(di, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->cache.flags < 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq27x00_battery_current(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = bq27x00_simple_value(di->cache.capacity, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = bq27x00_battery_capacity_level(di, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = bq27x00_simple_value(di->cache.temperature, val);
		if (ret == 0)
			val->intval -= 2731;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_empty, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27x00_simple_value(di->cache.time_to_empty_avg, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_full, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27x00_simple_value(bq27x00_battery_read_nac(di), val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27x00_simple_value(di->cache.charge_full, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = bq27x00_simple_value(di->charge_design_full, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq27x00_simple_value(di->cache.cycle_count, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27x00_simple_value(di->cache.energy, val);
		break;
	case POWER_SUPPLY_PROP_POWER_AVG:
		ret = bq27x00_simple_value(di->cache.power_avg, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq27x00_simple_value(di->cache.health, val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void bq27x00_external_power_changed(struct power_supply *psy)
{
//	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);
	struct bq27x00_device_info *di = psy->drv_data;

	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, msecs_to_jiffies(0));
}

#define BQBAT_NAME                                "bq27x00_bat"
static const struct power_supply_desc bd27x00_battery_desc = {
	.name		= BQBAT_NAME,
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties	= bq274xx_battery_props,
	.num_properties	= ARRAY_SIZE(bq274xx_battery_props),
	.get_property	= bq27x00_battery_get_property,
	.external_power_changed = bq27x00_external_power_changed,
};

#if 0
static void __init set_properties_array(struct bq27x00_device_info *di,
	enum power_supply_property *props, int num_props)
{
	int tot_sz = num_props * sizeof(enum power_supply_property);

	di->bat.desc->properties = devm_kzalloc(di->dev, tot_sz, GFP_KERNEL);

	if (di->bat.desc->properties) {
		memcpy(di->bat.desc->properties, props, tot_sz);
		di->bat.desc->num_properties = num_props;
	} else {
		di->bat.desc->.num_properties = 0;
	}
}
#endif
struct power_supply_config bdbat_cfg = {};
static int __init bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	int ret;

#if 0
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	if (di->chip == BQ274XX) {
		set_properties_array(di, bq274xx_battery_props,
			ARRAY_SIZE(bq274xx_battery_props));
	} else if (di->chip == BQ276XX) {
		set_properties_array(di, bq276xx_battery_props,
			ARRAY_SIZE(bq276xx_battery_props));
	} else if (di->chip == BQ27520) {
		set_properties_array(di, bq27520_battery_props,
			ARRAY_SIZE(bq27520_battery_props));
	} else if (di->chip == BQ2753X) {
		set_properties_array(di, bq2753x_battery_props,
			ARRAY_SIZE(bq2753x_battery_props));
	} else if (di->chip == BQ27542) {
		set_properties_array(di, bq27542_battery_props,
			ARRAY_SIZE(bq27542_battery_props));
	} else if (di->chip == BQ27545) {
		set_properties_array(di, bq27545_battery_props,
			ARRAY_SIZE(bq27545_battery_props));
	} else {
		set_properties_array(di, bq27x00_battery_props,
			ARRAY_SIZE(bq27x00_battery_props));
	}
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.external_power_changed = bq27x00_external_power_changed;
#endif

#if 0
	INIT_DELAYED_WORK(&di->work, bq27x00_battery_poll);
	mutex_init(&di->lock);
#endif
	//ret = power_supply_register(di->dev, di->bat);
	bdbat_cfg.drv_data = di; 
	di->bat = power_supply_register(di->dev, &bd27x00_battery_desc, &bdbat_cfg);
	dev_dbg(di->dev, "%s di->bat=0x%x\n", __func__, di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	bq27x00_update(di);

	return 0;
}

static void bq27x00_powersupply_unregister(struct bq27x00_device_info *di)
{
	/*
	 * power_supply_unregister call bq27x00_battery_get_property which
	 * call bq27x00_battery_poll.
	 * Make sure that bq27x00_battery_poll will not call
	 * schedule_delayed_work again after unregister (which cause OOPS).
	 */
	poll_interval = 0;

	cancel_delayed_work_sync(&di->work);

	power_supply_unregister(di->bat);
	di->bat = NULL;

	mutex_destroy(&di->lock);
}


/* i2c specific code */
#ifdef CONFIG_BATTERY_BQ27X00_I2C

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static int bq27xxx_read_i2c(struct bq27x00_device_info *di, u8 reg, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}

static int bq27xxx_write_i2c(struct bq27x00_device_info *di, u8 reg, int value, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg;
	unsigned char data[4];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	data[0] = reg;
	if (single) {
		data[1] = (unsigned char)value;
		msg.len = 2;
	} else {
		put_unaligned_le16(value, &data[1]);
		msg.len = 3;
	}

	msg.buf = data;
	msg.addr = client->addr;
	msg.flags = 0;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int bq27xxx_read_i2c_blk(struct bq27x00_device_info *di, u8 reg,
	u8 *data, u8 len)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = 1;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = len;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return ret;
}

static int bq27xxx_write_i2c_blk(struct bq27x00_device_info *di, u8 reg,
	u8 *data, u8 sz)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg;
	int ret;
	u8 buf[33];

	if (!client->adapter)
		return -ENODEV;

	buf[0] = reg;
	memcpy(&buf[1], data, sz);

	msg.buf = buf;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sz + 1;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int bq27x00_battery_reset(struct bq27x00_device_info *di)
{
	dev_info(di->dev, "Gas Gauge Reset\n");
#if 0
	bq27xxx_write(di, BQ27XXX_REG_CTRL, RESET_SUBCMD, false);

	msleep(10);
#else
	//It seems reset only works if chip is unsealed...
	if (!unseal(di, BQ274XX_UNSEAL_KEY))
		return 0;

	control_cmd_wr(di, BQ274XX_RESET);
	msleep(5);
	if (seal(di))
		return 1;
	else
		return 0;
#endif
	return bq27xxx_read(di, BQ27XXX_REG_CTRL, false);
}

static int bq27x00_battery_soft_reset(struct bq27x00_device_info *di)
{
	dev_info(di->dev, "Gas Gauge Soft Reset\n");

//	bq27xxx_write(di, BQ27XXX_REG_CTRL, SOFT_RESET_SUBCMD, false);

	msleep(10);

	return bq27xxx_read(di, BQ27XXX_REG_CTRL, false);
}

static int bq27x00_battery_read_fw_version(struct bq27x00_device_info *di)
{
	bq27xxx_write(di, BQ27XXX_REG_CTRL, FW_VER_SUBCMD, false);

	msleep(10);

	return bq27xxx_read(di, BQ27XXX_REG_CTRL, false);
}

static int bq27x00_battery_read_device_type(struct bq27x00_device_info *di)
{
	bq27xxx_write(di, BQ27XXX_REG_CTRL, DEV_TYPE_SUBCMD, false);

	msleep(10);

	return bq27xxx_read(di, BQ27XXX_REG_CTRL, false);
}

static int bq27x00_battery_read_dataflash_version(struct bq27x00_device_info *di)
{
	bq27xxx_write(di, BQ27XXX_REG_CTRL, DF_VER_SUBCMD, false);

	msleep(10);

	return bq27xxx_read(di, BQ27XXX_REG_CTRL, false);
}

static ssize_t show_firmware_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int ver;

	ver = bq27x00_battery_read_fw_version(di);

	return sprintf(buf, "%d\n", ver);
}

static DEVICE_ATTR(fw_version, S_IRUSR, show_firmware_version, NULL);
static ssize_t show_dataflash_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int ver;

	ver = bq27x00_battery_read_dataflash_version(di);

	return sprintf(buf, "%d\n", ver);
}
static DEVICE_ATTR(df_version, S_IRUSR, show_dataflash_version, NULL);

static ssize_t show_device_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int dev_type;

	dev_type = bq27x00_battery_read_device_type(di);

	return sprintf(buf, "%d\n", dev_type);
}
static DEVICE_ATTR(device_type, S_IRUSR, show_device_type, NULL);

static ssize_t store_reset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int ret;
	unsigned int val;

	ret = sscanf(buf,"%x",&val);
	if (ret == 1 && val == 0x55){
		bq27x00_battery_reset(di);	// reset chip
	}
	else if(ret == 1 && val == 0x56) {
		bq27x00_battery_soft_reset(di);	// soft reset chip
	}
	return count;
}
static DEVICE_ATTR(reset, S_IWUSR, NULL, store_reset);
#if 0
static DEVICE_ATTR(fw_version, S_IRUGO, show_firmware_version, NULL);
static DEVICE_ATTR(df_version, S_IRUGO, show_dataflash_version, NULL);
static DEVICE_ATTR(device_type, S_IRUGO, show_device_type, NULL);
static DEVICE_ATTR(reset, S_IWUGO, NULL, store_reset);
#endif

static struct attribute *bq27x00_attributes[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_df_version.attr,
	&dev_attr_device_type.attr,
	&dev_attr_reset.attr,
	NULL,
};
static const struct attribute_group bq27x00_attr_group = {
	.attrs = bq27x00_attributes,
};

struct i2c_client *bq27x00_i2c_client;

static int __init bq27x00_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	int num;
	int retval = 0;
	u8 *regs;

//	bq27x00_i2c_client = client;
//	bq27x00_i2c_client->addr = 0x55;

	/* Get new ID for the new battery device */
	idr_preload(GFP_KERNEL);
	mutex_lock(&battery_mutex);
	num = idr_alloc(&battery_id, client, 0, 0, GFP_KERNEL);
	mutex_unlock(&battery_mutex);
	idr_preload_end();
	if (num < 0) {
		dev_err(&client->dev, "failed to allocate pointer id\n");
		return num;
	}
#if 0
	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}
#endif
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	di->id = num;
	di->dev = &client->dev;
	di->chip = BQ274XX; //id->driver_data;
//	di->bat.name = name;
	di->bus.read = &bq27xxx_read_i2c;
	di->bus.write = &bq27xxx_write_i2c;
	di->bus.blk_read = bq27xxx_read_i2c_blk;
	di->bus.blk_write = bq27xxx_write_i2c_blk;
	di->dm_regs = NULL;
	di->dm_regs_count = 0;

	if (di->chip == BQ27200)
		regs = bq27200_regs;
	else if (di->chip == BQ27500)
		regs = bq27500_regs;
	else if (di->chip == BQ27520)
		regs = bq27520_regs;
	else if (di->chip == BQ2753X)
		regs = bq2753x_regs;
	else if (di->chip == BQ27542 || di->chip == BQ27545)
		regs = bq2754x_regs;
	else if (di->chip == BQ274XX) {
		dev_info(&client->dev, "%s, dm_regs set to bq274xx_regs\n", __func__);
		regs = bq274xx_regs;
		di->dm_regs = bq274xx_dm_regs;
		di->dm_regs_count = ARRAY_SIZE(bq274xx_dm_regs);
	} else if (di->chip == BQ276XX) {
		/* commands are same as bq274xx, only DM is different */
		regs = bq276xx_regs;
		di->dm_regs = bq276xx_dm_regs;
		di->dm_regs_count = ARRAY_SIZE(bq276xx_dm_regs);
	} else {
		dev_err(&client->dev,
			"Unexpected gas gague: %d\n", di->chip);
		regs = bq27520_regs;
	}

	memcpy(di->regs, regs, NUM_REGS);

	di->fw_ver = bq27x00_battery_read_fw_version(di);
	dev_info(&client->dev, "Gas Guage fw version is 0x%04x\n", di->fw_ver);


	INIT_DELAYED_WORK(&di->work, bq27x00_battery_poll);
	mutex_init(&di->lock);

	retval = bq27x00_powersupply_init(di);
	if (retval)
		goto batt_failed_3;

	/* Schedule a polling after about 1 min */
	schedule_delayed_work(&di->work, msecs_to_jiffies(60 * HZ));

	i2c_set_clientdata(client, di);

	retval = sysfs_create_group(&client->dev.kobj, &bq27x00_attr_group);
	if (retval)
		dev_err(&client->dev, "could not create sysfs files\n");
	return 0;

batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	bq27x00_powersupply_unregister(di);

//	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}

/*
static const struct i2c_device_id bq27x00_id[] = {
	{ "bq27200", BQ27200 },
	{ "bq27500", BQ27500 },
	{ "bq27520", BQ27520 },
	{ "bq274xx", BQ274XX },
	{ "bq276xx", BQ276XX },
	{ "bq2753x", BQ2753X },
	{ "bq27542", BQ27542 },
	{ "bq27545", BQ27545 },
	{},
};
*/

static const struct i2c_device_id bq27x00_id[] = {
	{ "bq274xx", BQ274XX },
	{}
};
MODULE_DEVICE_TABLE(i2c, bq27x00_id);

#define BQ27X00_OF_NODE_NAME	"ti,bq27x00"
static const struct of_device_id bq27x00_match_table[] = {
	{ .compatible = BQ27X00_OF_NODE_NAME, },
	{}
};
static struct i2c_driver __refdata bq27x00_battery_driver = {
	.driver = {
		.name = "bq27x00-battery",
		.of_match_table = bq27x00_match_table,
	},
	.probe = bq27x00_battery_probe,
	.remove = bq27x00_battery_remove,
	.id_table = bq27x00_id,
};

module_i2c_driver(bq27x00_battery_driver);

#if 0
static inline int __init bq27x00_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq27x00_battery_driver);
	printk(KERN_ERR "%s\n", __FUNCTION__);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27x00 i2c driver\n");

	return ret;
}

static inline void __exit bq27x00_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27x00_battery_driver);
}

#endif
#else

static inline int bq27x00_battery_i2c_init(void) { return 0; }
static inline void bq27x00_battery_i2c_exit(void) {};

#endif

/* platform specific code */
#ifdef CONFIG_BATTERY_BQ27X00_PLATFORM

static int bq27000_read_platform(struct bq27x00_device_info *di, u8 reg,
			bool single)
{
	struct device *dev = di->dev;
	struct bq27000_platform_data *pdata = dev->platform_data;
	unsigned int timeout = 3;
	int upper, lower;
	int temp;

	if (!single) {
		/* Make sure the value has not changed in between reading the
		 * lower and the upper part */
		upper = pdata->read(dev, reg + 1);
		do {
			temp = upper;
			if (upper < 0)
				return upper;

			lower = pdata->read(dev, reg);
			if (lower < 0)
				return lower;

			upper = pdata->read(dev, reg + 1);
		} while (temp != upper && --timeout);

		if (timeout == 0)
			return -EIO;

		return (upper << 8) | lower;
	}

	return pdata->read(dev, reg);
}

static int __init bq27000_battery_probe(struct platform_device *pdev)
{
	struct bq27x00_device_info *di;
	struct bq27000_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform_data supplied\n");
		return -EINVAL;
	}

	if (!pdata->read) {
		dev_err(&pdev->dev, "no hdq read callback supplied\n");
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	di->chip = BQ27200;

//	di->bat.name = pdata->name ?: dev_name(&pdev->dev);
	di->bus.read = &bq27000_read_platform;

	ret = bq27x00_powersupply_init(di);
	if (ret)
		goto err_free;

	return 0;

err_free:
	kfree(di);

	return ret;
}

static int __exit bq27000_battery_remove(struct platform_device *pdev)
{
	struct bq27x00_device_info *di = platform_get_drvdata(pdev);

	bq27x00_powersupply_unregister(di);

	kfree(di);

	return 0;
}

static struct platform_driver __initdata bq27000_battery_driver = {
	.probe	= bq27000_battery_probe,
	.remove = __exit_p(bq27000_battery_remove),
	.driver = {
		.name = "bq27000-battery",
		.owner = THIS_MODULE,
	},
};

static inline int bq27x00_battery_platform_init(void)
{
	int ret = platform_driver_register(&bq27000_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27200 platform driver\n");

	return ret;
}

static inline void bq27x00_battery_platform_exit(void)
{
	platform_driver_unregister(&bq27000_battery_driver);
}

#else

static inline int bq27x00_battery_platform_init(void) { return 0; }
static inline void bq27x00_battery_platform_exit(void) {};

#endif

/*
 * Module stuff
 */
/*
static int __init bq27x00_battery_init(void)
{
	int ret;

	printk(KERN_ERR "%s\n", __FUNCTION__);
	ret = bq27x00_battery_i2c_init();
	if (ret)
		return ret;

	ret = bq27x00_battery_platform_init();
	if (ret)
		bq27x00_battery_i2c_exit();

	return ret;
}
module_init(bq27x00_battery_init);

//subsys_initcall(bq27x00_battery_init);
static void __exit bq27x00_battery_exit(void)
{
	bq27x00_battery_platform_exit();
	bq27x00_battery_i2c_exit();
}
module_exit(bq27x00_battery_exit);
*/

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
