/**
 * @file bd71827.h  ROHM BD71827GW header file
 *
 * Copyright 2016
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 * @author cpham2403@gmail.com
 */

#ifndef __LINUX_MFD_BD71827_H
#define __LINUX_MFD_BD71827_H

/* Don't use Device Tree */
//#define BD71827_NONE_DTB	1

#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/delay.h>

/*
 * When GPIO2_MODE bits are set to 2'b10 (LDO5_VSEL), LDO5 output voltage is controlled by GPIO2 pin.
 * If GPIO2 = L, LDO5 output voltage corresponds to the setting of LDO5_L bits.
 * If GPIO2 = H, LDO5 output voltage corresponds to the setting of LDO5_H bits.
 * When GPIO2_MODE bits are not set to 2'b10, LDO5 output voltage corresponds to the setting of LDO5_L bits
 */
// LDO5VSEL_EQ_H
// define to 1 when LDO5VSEL connect to High
// define to 0 when LDO5VSEL connect to Low
// Default LDO5_SEL = 0, don't use GPIO2 select mode.
#define LDO5VSEL_EQ_H		0

#ifndef LDO5VSEL_EQ_H
	#error define LDO5VSEL_EQ_H to 1 when connect to High, to 0 when connect to Low
#else
	#if LDO5VSEL_EQ_H == 1
		#define BD71827_REG_LDO5_VOLT 	BD71827_REG_LDO5_VOLT_H
		#define LDO5_MASK				LDO5_H_MASK
	#elif LDO5VSEL_EQ_H == 0
		#define BD71827_REG_LDO5_VOLT 	BD71827_REG_LDO5_VOLT_L
		#define LDO5_MASK				LDO5_L_MASK
	#else
		#error  Define LDO5VSEL_EQ_H only to 0 or 1
	#endif
#endif

enum {
	BD71827_BUCK1	=	0,
	BD71827_BUCK2,
	BD71827_BUCK3,
	BD71827_BUCK4,
	BD71827_BUCK5,
	// General Purpose
	BD71827_LDO1,
	BD71827_LDO2,
	BD71827_LDO3,
	BD71827_LDO4,
	BD71827_LDO5,
	BD71827_LDO6,
	// LDO for Secure Non-Volatile Storage
	BD71827_LDOSNVS,
	BD71827_REGULATOR_CNT,
};

#define BD71827_SUPPLY_STATE_ENABLED    0x1

#define BD71827_BUCK1_VOLTAGE_NUM	0x3F
#define BD71827_BUCK2_VOLTAGE_NUM	0x3F
#define BD71827_BUCK3_VOLTAGE_NUM	0x3F
#define BD71827_BUCK4_VOLTAGE_NUM	0x1F
#define BD71827_BUCK5_VOLTAGE_NUM	0x1F
#define BD71827_LDO1_VOLTAGE_NUM	0x3F
#define BD71827_LDO2_VOLTAGE_NUM	0x3F
#define BD71827_LDO3_VOLTAGE_NUM	0x3F
#define BD71827_LDO4_VOLTAGE_NUM	0x3F
#define BD71827_LDO5_VOLTAGE_NUM	0x3F
#define BD71827_LDO6_VOLTAGE_NUM	0x1
#define BD71827_LDOSNVS_VOLTAGE_NUM	0x1

#define BD71827_GPIO_NUM			2	/* BD71827 have 2 GPO */

enum {
	BD71827_REG_DEVICE              = 0x00,
	BD71827_REG_PWRCTRL             = 0x01,
	BD71827_REG_BUCK1_MODE          = 0x02,
	BD71827_REG_BUCK2_MODE          = 0x03,
	BD71827_REG_BUCK3_MODE          = 0x04,
	BD71827_REG_BUCK4_MODE          = 0x05,
	BD71827_REG_BUCK5_MODE          = 0x06,
	BD71827_REG_BUCK1_VOLT_RUN      = 0x07,
	BD71827_REG_BUCK1_VOLT_SUSP     = 0x08,
	BD71827_REG_BUCK2_VOLT_RUN      = 0x09,
	BD71827_REG_BUCK2_VOLT_SUSP     = 0x0A,
	BD71827_REG_BUCK3_VOLT          = 0x0B,
	BD71827_REG_BUCK4_VOLT          = 0x0C,
	BD71827_REG_BUCK5_VOLT          = 0x0D,
	BD71827_REG_LED_CTRL            = 0x0E,
	BD71827_REG_reserved_0F         = 0x0F,
	BD71827_REG_LDO_MODE1           = 0x10,
	BD71827_REG_LDO_MODE2           = 0x11,
	BD71827_REG_LDO_MODE3           = 0x12,
	BD71827_REG_LDO_MODE4           = 0x13,
	BD71827_REG_LDO1_VOLT           = 0x14,
	BD71827_REG_LDO2_VOLT           = 0x15,
	BD71827_REG_LDO3_VOLT           = 0x16,
	BD71827_REG_LDO4_VOLT           = 0x17,
	BD71827_REG_LDO5_VOLT_H         = 0x18,
	BD71827_REG_LDO5_VOLT_L         = 0x19,
	BD71827_REG_BUCK_PD_DIS         = 0x1A,
	BD71827_REG_LDO_PD_DIS          = 0x1B,
	BD71827_REG_GPIO                = 0x1C,
	BD71827_REG_OUT32K              = 0x1D,
	BD71827_REG_SEC                 = 0x1E,
	BD71827_REG_MIN                 = 0x1F,
	BD71827_REG_HOUR                = 0x20,
	BD71827_REG_WEEK                = 0x21,
	BD71827_REG_DAY                 = 0x22,
	BD71827_REG_MONTH               = 0x23,
	BD71827_REG_YEAR                = 0x24,
	BD71827_REG_ALM0_SEC            = 0x25,
	BD71827_REG_ALM0_MIN            = 0x26,
	BD71827_REG_ALM0_HOUR           = 0x27,
	BD71827_REG_ALM0_WEEK           = 0x28,
	BD71827_REG_ALM0_DAY            = 0x29,
	BD71827_REG_ALM0_MONTH          = 0x2A,
	BD71827_REG_ALM0_YEAR           = 0x2B,
	BD71827_REG_ALM1_SEC            = 0x2C,
	BD71827_REG_ALM1_MIN            = 0x2D,
	BD71827_REG_ALM1_HOUR           = 0x2E,
	BD71827_REG_ALM1_WEEK           = 0x2F,
	BD71827_REG_ALM1_DAY            = 0x30,
	BD71827_REG_ALM1_MONTH          = 0x31,
	BD71827_REG_ALM1_YEAR           = 0x32,
	BD71827_REG_ALM0_MASK           = 0x33,
	BD71827_REG_ALM1_MASK           = 0x34,
	BD71827_REG_ALM2                = 0x35,
	BD71827_REG_TRIM                = 0x36,
	BD71827_REG_CONF                = 0x37,
	BD71827_REG_SYS_INIT            = 0x38,
	BD71827_REG_CHG_STATE           = 0x39,
	BD71827_REG_CHG_LAST_STATE      = 0x3A,
	BD71827_REG_BAT_STAT            = 0x3B,
	BD71827_REG_DCIN_STAT           = 0x3C,
	BD71827_REG_VSYS_STAT           = 0x3D,
	BD71827_REG_CHG_STAT            = 0x3E,
	BD71827_REG_CHG_WDT_STAT        = 0x3F,
	BD71827_REG_BAT_TEMP            = 0x40,
	BD71827_REG_ILIM_STAT           = 0x41,
	BD71827_REG_DCIN_SET            = 0x42,
	BD71827_REG_DCIN_CLPS           = 0x43,
	BD71827_REG_VSYS_REG            = 0x44,
	BD71827_REG_VSYS_MAX            = 0x45,
	BD71827_REG_VSYS_MIN            = 0x46,
	BD71827_REG_CHG_SET1            = 0x47,
	BD71827_REG_CHG_SET2            = 0x48,
	BD71827_REG_CHG_WDT_PRE         = 0x49,
	BD71827_REG_CHG_WDT_FST         = 0x4A,
	BD71827_REG_CHG_IPRE            = 0x4B,
	BD71827_REG_CHG_IFST            = 0x4C,
	BD71827_REG_CHG_IFST_TERM       = 0x4D,
	BD71827_REG_CHG_VPRE            = 0x4E,
	BD71827_REG_CHG_VBAT_1          = 0x4F,
	BD71827_REG_CHG_VBAT_2          = 0x50,
	BD71827_REG_CHG_VBAT_3          = 0x51,
	BD71827_REG_CHG_LED_1           = 0x52,
	BD71827_REG_VF_TH               = 0x53,
	BD71827_REG_BAT_SET_1           = 0x54,
	BD71827_REG_BAT_SET_2           = 0x55,
	BD71827_REG_BAT_SET_3           = 0x56,
	BD71827_REG_ALM_VBAT_TH_U       = 0x57,
	BD71827_REG_ALM_VBAT_TH_L       = 0x58,
	BD71827_REG_ALM_DCIN_TH         = 0x59,
	BD71827_REG_ALM_VSYS_TH         = 0x5A,
	BD71827_REG_reserved_5B         = 0x5B,
	BD71827_REG_reserved_5C         = 0x5C,
	BD71827_REG_VM_VBAT_U           = 0x5D,
	BD71827_REG_VM_VBAT_L           = 0x5E,
	BD71827_REG_VM_BTMP             = 0x5F,
	BD71827_REG_VM_VTH              = 0x60,
	BD71827_REG_VM_DCIN_U           = 0x61,
	BD71827_REG_VM_DCIN_L           = 0x62,
	BD71827_REG_reserved_63         = 0x63,
	BD71827_REG_VM_VF               = 0x64,
	BD71827_REG_reserved_65         = 0x65,
	BD71827_REG_reserved_66         = 0x66,
	BD71827_REG_VM_OCV_PRE_U        = 0x67,
	BD71827_REG_VM_OCV_PRE_L        = 0x68,
	BD71827_REG_reserved_69         = 0x69,
	BD71827_REG_reserved_6A         = 0x6A,
	BD71827_REG_VM_OCV_PST_U        = 0x6B,
	BD71827_REG_VM_OCV_PST_L        = 0x6C,
	BD71827_REG_VM_SA_VBAT_U        = 0x6D,
	BD71827_REG_VM_SA_VBAT_L        = 0x6E,
	BD71827_REG_reserved_6F         = 0x6F,
	BD71827_REG_reserved_70         = 0x70,
	BD71827_REG_CC_CTRL             = 0x71,
	BD71827_REG_CC_BATCAP1_TH_U     = 0x72,
	BD71827_REG_CC_BATCAP1_TH_L     = 0x73,
	BD71827_REG_CC_BATCAP2_TH_U     = 0x74,
	BD71827_REG_CC_BATCAP2_TH_L     = 0x75,
	BD71827_REG_CC_BATCAP3_TH_U     = 0x76,
	BD71827_REG_CC_BATCAP3_TH_L     = 0x77,
	BD71827_REG_CC_STAT             = 0x78,
	BD71827_REG_CC_CCNTD_3          = 0x79,
	BD71827_REG_CC_CCNTD_2          = 0x7A,
	BD71827_REG_CC_CCNTD_1          = 0x7B,
	BD71827_REG_CC_CCNTD_0          = 0x7C,
	BD71827_REG_CC_CURCD_U          = 0x7D,
	BD71827_REG_CC_CURCD_L          = 0x7E,
	BD71827_REG_CC_OCUR_THR_1       = 0x7F,
	BD71827_REG_CC_OCUR_DUR_1       = 0x80,
	BD71827_REG_CC_OCUR_THR_2       = 0x81,
	BD71827_REG_CC_OCUR_DUR_2       = 0x82,
	BD71827_REG_CC_OCUR_THR_3       = 0x83,
	BD71827_REG_CC_OCUR_DUR_3       = 0x84,
	BD71827_REG_CC_OCUR_MON         = 0x85,
	BD71827_REG_VM_BTMP_OV_THR      = 0x86,
	BD71827_REG_VM_BTMP_OV_DUR      = 0x87,
	BD71827_REG_VM_BTMP_LO_THR      = 0x88,
	BD71827_REG_VM_BTMP_LO_DUR      = 0x89,
	BD71827_REG_VM_BTMP_MON         = 0x8A,
	BD71827_REG_INT_EN_01           = 0x8B,
	BD71827_REG_INT_EN_02           = 0x8C,
	BD71827_REG_INT_EN_03           = 0x8D,
	BD71827_REG_INT_EN_04           = 0x8E,
	BD71827_REG_INT_EN_05           = 0x8F,
	BD71827_REG_INT_EN_06           = 0x90,
	BD71827_REG_INT_EN_07           = 0x91,
	BD71827_REG_INT_EN_08           = 0x92,
	BD71827_REG_INT_EN_09           = 0x93,
	BD71827_REG_INT_EN_10           = 0x94,
	BD71827_REG_INT_EN_11           = 0x95,
	BD71827_REG_INT_EN_12           = 0x96,
	BD71827_REG_INT_STAT            = 0x97,
	BD71827_REG_INT_STAT_01         = 0x98,
	BD71827_REG_INT_STAT_02         = 0x99,
	BD71827_REG_INT_STAT_03         = 0x9A,
	BD71827_REG_INT_STAT_04         = 0x9B,
	BD71827_REG_INT_STAT_05         = 0x9C,
	BD71827_REG_INT_STAT_06         = 0x9D,
	BD71827_REG_INT_STAT_07         = 0x9E,
	BD71827_REG_INT_STAT_08         = 0x9F,
	BD71827_REG_INT_STAT_09         = 0xA0,
	BD71827_REG_INT_STAT_10         = 0xA1,
	BD71827_REG_INT_STAT_11         = 0xA2,
	BD71827_REG_INT_STAT_12         = 0xA3,
	BD71827_REG_INT_UPDATE          = 0xA4,
	BD71827_REG_PWRCTRL2            = 0xA8,
	BD71827_REG_PWRCTRL3            = 0xA9,
	BD71827_REG_SWRESET             = 0xAA,
	BD71827_REG_BUCK1_VOLT_IDLE     = 0xAB,
	BD71827_REG_BUCK2_VOLT_IDLE     = 0xAC,
	BD71827_REG_ONEVNT_MODE_1       = 0xAD,
	BD71827_REG_ONEVNT_MODE_2       = 0xAE,
	BD71827_REG_RESERVE_0           = 0xB0,
	BD71827_REG_RESERVE_1           = 0xB1,
	BD71827_REG_RESERVE_2           = 0xB2,
	BD71827_REG_RESERVE_3           = 0xB3,
	BD71827_REG_RESERVE_4           = 0xB4,
	BD71827_REG_RESERVE_5           = 0xB5,
	BD71827_REG_RESERVE_6           = 0xB6,
	BD71827_REG_RESERVE_7           = 0xB7,
	BD71827_REG_RESERVE_8           = 0xB8,
	BD71827_REG_RESERVE_9           = 0xB9,
	BD71827_REG_RESERVE_A           = 0xBA,
	BD71827_REG_RESERVE_B           = 0xBB,
	BD71827_REG_RESERVE_C           = 0xBC,
	BD71827_REG_RESERVE_D           = 0xBD,
	BD71827_REG_RESERVE_E           = 0xBE,
	BD71827_REG_RESERVE_F           = 0xBF,
	BD71827_REG_VM_VSYS_U           = 0xC0,
	BD71827_REG_VM_VSYS_L           = 0xC1,
	BD71827_REG_VM_SA_VSYS_U        = 0xC2,
	BD71827_REG_VM_SA_VSYS_L        = 0xC3,
	BD71827_REG_CC_SA_CURCD_U       = 0xC4,
	BD71827_REG_CC_SA_CURCD_L       = 0xC5,
	BD71827_REG_BATID               = 0xC6,
	BD71827_REG_VM_SA_VBAT_MIN_U    = 0xD4,
	BD71827_REG_VM_SA_VBAT_MIN_L    = 0xD5,
	BD71827_REG_VM_SA_VBAT_MAX_U    = 0xD6,
	BD71827_REG_VM_SA_VBAT_MAX_L    = 0xD7,
	BD71827_REG_VM_SA_VSYS_MIN_U    = 0xD8,
	BD71827_REG_VM_SA_VSYS_MIN_L    = 0xD9,
	BD71827_REG_VM_SA_VSYS_MAX_U    = 0xDA,
	BD71827_REG_VM_SA_VSYS_MAX_L    = 0xDB,
	BD71827_REG_VM_SA_MINMAX_CLR    = 0xDC,
	BD71827_REG_VM_OCV_PWRON_U      = 0xDD,
	BD71827_REG_VM_OCV_PWRON_L      = 0xDE,
	BD71827_REG_REX_CCNTD_3         = 0xE0,
	BD71827_REG_REX_CCNTD_2         = 0xE1,
	BD71827_REG_REX_CCNTD_1         = 0xE2,
	BD71827_REG_REX_CCNTD_0         = 0xE3,
	BD71827_REG_REX_SA_VBAT_U       = 0xE4,
	BD71827_REG_REX_SA_VBAT_L       = 0xE5,
	BD71827_REG_REX_CTRL_1          = 0xE6,
	BD71827_REG_REX_CTRL_2          = 0xE7,
	BD71827_REG_FULL_CCNTD_3        = 0xE8,
	BD71827_REG_FULL_CCNTD_2        = 0xE9,
	BD71827_REG_FULL_CCNTD_1        = 0xEA,
	BD71827_REG_FULL_CCNTD_0        = 0xEB,
	BD71827_REG_FULL_CTRL           = 0xEC,
	BD71827_REG_CCNTD_CHG_3         = 0xF0,
	BD71827_REG_CCNTD_CHG_2         = 0xF1,
	BD71827_REG_INT_EN_13           = 0xF8,
	BD71827_REG_INT_STAT_13         = 0xF9,
	BD71827_REG_I2C_MAGIC           = 0xFE,
	BD71827_REG_PRODUCT             = 0xFF,
	BD71827_MAX_REGISTER			= 0x100,
};

/* BD71827_REG_TEST_REG */
#define BD71827_TEST_REG_ACCESS_AREA_1	0x76
#define BD71827_TEST_REG_ACCESS_AREA_2	0x66
#define BD71827_TEST_REG_ACCESS_AREA_3	0x56

#define BD71827_TEST_REG_POWER_SEQ_REG	0x42
#define BD71827_TEST_REG_DISABLE_OFF_SEQUENCE	0x4C
#define BD71827_TEST_REG_ENABLE_OFF_SEQUENCE	0x44

#define BD71827_TEST_REG_BUTTON_PRESS_COUNTER_REG	0x55
#define BD71827_TEST_REG_BUTTON_PRESS_COUNTER_CLEAR	0x0F
#define BD71827_TEST_REG_BUTTON_PRESS_COUNTER_NORMAL	0x07

#define BD71827_TEST_REG_BATFET_CFG_REG 	0x72
#define BD71827_TEST_REG_BATFET_ON		0x83
#define BD71827_TEST_REG_BATFET_OFF		0x0

#define BD71827_TEST_REG_TS_MONITOR_CFG_REG	0x82
#define BD71827_TEST_REG_TS_MONITOR_OFF		0x3E
#define BD71827_TEST_REG_TS_MONITOR_ON		0x3F

#define BD71827_TEST_REG_SARADC_REG		0x83
#define BD71827_TEST_REG_SARADC_VALUE_256	0x07

#define BD71827_TEST_REG_ACCESS_USER_AREA	0x00

/* BD71827_REG_BUCK1_MODE bits */
#define BUCK1_RAMPRATE_MASK		0xC0
#define BUCK1_RAMPRATE_10P00MV	0x0
#define BUCK1_RAMPRATE_5P00MV	0x1
#define BUCK1_RAMPRATE_2P50MV	0x2
#define BUCK1_RAMPRATE_1P25MV	0x3
#define BUCK1_RUN_ON			0x04

/* BD71827_REG_BUCK2_MODE bits */
#define BUCK2_RAMPRATE_MASK		0xC0
#define BUCK2_RAMPRATE_10P00MV	0x0
#define BUCK2_RAMPRATE_5P00MV	0x1
#define BUCK2_RAMPRATE_2P50MV	0x2
#define BUCK2_RAMPRATE_1P25MV	0x3
#define BUCK2_RUN_ON			0x04

/* BD71827_REG_BUCK3_MODE bits */
#define BUCK3_RUN_ON			0x04

/* BD71827_REG_BUCK4_MODE bits */
#define BUCK4_RUN_ON			0x04

/* BD71827_REG_BUCK5_MODE bits */
#define BUCK5_RUN_ON			0x04

/* BD71827_REG_BUCK1_VOLT_RUN bits */
#define BUCK1_RUN_MASK			0x3F
#define BUCK1_RUN_DEFAULT		0x0E

/* BD71827_REG_BUCK1_VOLT_SUSP bits */
#define BUCK1_SUSP_MASK			0x3F
#define BUCK1_SUSP_DEFAULT		0x04

/* BD71827_REG_BUCK1_VOLT_IDLE bits */
#define BUCK1_IDLE_MASK			0x3F
#define BUCK1_IDLE_DEFAULT		0x0E

/* BD71827_REG_BUCK2_VOLT_RUN bits */
#define BUCK2_RUN_MASK			0x3F
#define BUCK2_RUN_DEFAULT		0x0E

/* BD71827_REG_BUCK2_VOLT_SUSP bits */
#define BUCK2_SUSP_MASK			0x3F
#define BUCK2_SUSP_DEFAULT		0x0E

/* BD71827_REG_BUCK3_VOLT bits */
#define BUCK3_MASK			0x3F
#define BUCK3_DEFAULT		0x0E

/* BD71827_REG_BUCK4_VOLT bits */
#define BUCK4_MASK			0x3F
#define BUCK4_DEFAULT		0x0E

/* BD71827_REG_BUCK5_VOLT bits */
#define BUCK5_MASK			0x3F
#define BUCK5_DEFAULT		0x0E

/* BD71827_REG_BUCK2_VOLT_IDLE bits */
#define BUCK2_IDLE_MASK			0x3F
#define BUCK2_IDLE_DEFAULT		0x0E

/* BD71827_REG_OUT32K bits */
#define OUT32K_EN				0x01
#define OUT32K_MODE				0x02

/* BD71827_REG_BAT_STAT bits */
#define BAT_DET					0x20
#define BAT_DET_OFFSET			5
#define BAT_DET_DONE			0x10
#define VBAT_OV					0x08
#define DBAT_DET				0x01

/* BD71827_REG_DCIN_STAT bits */
#define DCIN_DET				0x01

/* BD71827_REG_ALM0_MASK bits */
#define A0_ONESEC				BIT(7)
#define A0_SEC					BIT(0)
#define A0_MIN					BIT(1)
#define A0_HOUR					BIT(2)
#define A0_WEEK					BIT(3)
#define A0_DAY					BIT(4)
#define A0_MON					BIT(5)
#define A0_YEAR					BIT(6)

/* BD71827_REG_CONF bits */
#define BD71827_REG_CONF_XSTB	0x02
#define BD71827_REG_CONF_PON	0x01

/*BD71827_CHG_VBAT_ registers values */

/* 3.72V to 4.60V range, 10mV step: 0x0-3.72, 0x1-3.73, ... */
#define BD71827_CHG_VBAT_4V 		0x1C //maps to 4.0V
#define BD71827_CHG_VBAT_CHG_1_DEFAULT	0x3F //maps to 4.35V == OTP value
#define BD71827_CHG_VBAT_CHG_2_DEFAULT	0x26 //maps to 4.1V, OTP value is 0x35, which maps to 4.25V
#define BD71827_CHG_VBAT_CHG_3_DEFAULT	0x3F //maps to 4.35v == OTP value

/* BD71827_REG_INT_STAT_03 bits */
#define DCIN_MON_DET				0x02
#define DCIN_MON_RES				0x01
#define POWERON_LONG				0x04
#define POWERON_MID					0x08
#define POWERON_SHORT				0x10
#define POWERON_PRESS				0x20

/* BD71827_REG_SYS_INIT bits */
#define CHGRST						BIT(1)

/* BD71827_REG_DCIN_SET bits */
#define CHG_DCIN_EN					BIT(7)
#define CHG_DCIN_ILIM_EN			BIT(6)
#define CHG_DCIN_SET_MASK			0x3F
#define CHG_DCIN_SET_STEP			  50
#define CHG_DCIN_UPPER_LIMIT		 900

/* BD71827_REG_CHG_IFST bits */
#define CHG_IFST_MASK				0x3F
#define CHG_IFST_STEP				  25
#define CHG_IFST_UPPER_LIMIT		 900

/* BD71805_REG_INT_STAT_08 bits */
#define VBAT_MON_DET				0x02
#define VBAT_MON_RES				0x01

/* BD71805_REG_INT_STAT_11 bits */
#define	INT_STAT_11_VF_DET			0x80
#define	INT_STAT_11_VF_RES			0x40
#define	INT_STAT_11_VF125_DET		0x20
#define	INT_STAT_11_VF125_RES		0x10
#define	INT_STAT_11_OVTMP_DET		0x08
#define	INT_STAT_11_OVTMP_RES		0x04
#define	INT_STAT_11_LOTMP_DET		0x02
#define	INT_STAT_11_LOTMP_RES		0x01

/* BD71827_REG_PWRCTRL bits */
#define RESTARTEN				0x01

/* BD71827_REG_GPIO bits */
#define GPIO2_MODE_MASK			0xC0
#define GPIO2_MODE_RD(x)		(((u8)x & 0xC0) >> 6)
#define GPIO2_LDO5_VSEL			2
#define GPIO2_PMIC_ON_REQ		3
#define GPIO1_MODE_MASK			0x30
#define GPO_DRV_MASK			0x0C
#define GPO1_DRV_MASK			0x04
#define GPO2_DRV_MASK			0x08

/* BD71827_REG_CHG_SET1 bits */
#define CHG_EN					0x01

/* BD71827_REG_PRODUCT */
#define PRODUCT_VERSION			0xF0

/* BD71827 interrupt masks */
enum {
	BD71827_INT_EN_01_BUCKAST_MASK	=	0x1F,
	BD71827_INT_EN_02_DCINAST_MASK	=	0x0F,
	BD71827_INT_EN_03_DCINAST_MASK	=	0x3F,
	BD71827_INT_EN_04_VSYSAST_MASK	=	0xCF,
	BD71827_INT_EN_05_CHGAST_MASK	=	0xFF,
	BD71827_INT_EN_06_BATAST_MASK	=	0xF3,
	BD71827_INT_EN_07_BMONAST_MASK	=	0xFE,
	BD71827_INT_EN_08_BMONAST_MASK	=	0x03,
	BD71827_INT_EN_09_BMONAST_MASK	=	0x07,
	BD71827_INT_EN_10_BMONAST_MASK	=	0x3F,
	BD71827_INT_EN_11_TMPAST_MASK	=	0xFF,
	BD71827_INT_EN_12_ALMAST_MASK	=	0x07,
};
/* BD71827 interrupt irqs */
enum {
	BD71827_IRQ_BUCK_01		=	0x0,
	BD71827_IRQ_DCIN_02,
	BD71827_IRQ_DCIN_03,
	BD71827_IRQ_VSYS_04,
	BD71827_IRQ_CHARGE_05,
	BD71827_IRQ_BAT_06,
	BD71827_IRQ_BAT_MON_07,
	BD71827_IRQ_BAT_MON_08,
	BD71827_IRQ_BAT_MON_09,
	BD71827_IRQ_BAT_MON_10,
	BD71827_IRQ_TEMPERATURE_11,
	BD71827_IRQ_ALARM_12,
	TOTAL_IRQ_STATUS_REGS,
};

/* BD71827_REG_INT_EN_12 bits */
#define ALM0_EN					0x1

/* BD71827_REG_CC_CTRL bits */
#define CCNTRST					0x80
#define CCNTENB					0x40
#define CCCALIB					0x20

/* BD71827_REG_CHG_SET1 bits */
#define WDT_AUTO				0x40
#define WDT_MANUAL				0x2F

/* BD71827_REG_CC_CURCD */
#define CURDIR_Discharging		0x8000

/* BD71827_REG_VM_SA_IBAT */
#define IBAT_SA_DIR_Discharging	0x8000

/* BD71827_REG_VM_SA_MINMAX_CLR bits */
#define VSYS_SA_MIN_CLR			0x10
#define VBAT_SA_MIN_CLR			0x01

/* BD71827_REG_REX_CTRL_1 bits */
#define REX_CLR					0x10
#define REX_PMU_STATE_MASK		0x04

/* BD71827_REG_FULL_CTRL bits */
#define FULL_CLR				0x10

/* BD71827_REG_LED_CTRL bits */
#define CHGDONE_LED_EN			0x10

/* BD71827_REG_LDO_MODE1 bits */
#define LDO1_RUN_ON				0x40
/*
 * Bit 3 : LDO4_REG_MODE
 * 0: LDO4 is controlled via external pin (GPIO1).
 * 1: LDO4 is controlled via register.
 */
#define LDO4_REG_MODE			0x08
/*
 * Bit 2 : LDO3_REG_MODE
 * 0: LDO3 starts when DCIN is supplied.
 * 1: LDO3 is controlled via register.
 */
#define LDO3_REG_MODE			0x04

/* BD71827_REG_LDO_MODE2 bits */
#define LDO2_RUN_ON				0x04
#define LDO3_RUN_ON				0xC0

/* BD71827_REG_LDO_MODE3 bits */
#define LDO4_RUN_ON				0x04
#define LDO5_RUN_ON				0x40

/* BD71827_REG_LDO_MODE4 bits */
#define LDO6_RUN_ON				0x04
#define SNVS_RUN_ON				0x40


/* BD71827_REG_LDO1_VOLT bits */
#define LDO1_MASK				0x3F

/* BD71827_REG_LDO2_VOLT bits */
#define LDO2_MASK				0x3F

/* BD71827_REG_LDO3_VOLT bits */
#define LDO3_MASK				0x3F

/* BD71827_REG_LDO4_VOLT bits */
#define LDO4_MASK				0x3F

/* BD71827_REG_LDO5_VOLT_H bits */
#define LDO5_H_MASK				0x3F

/* BD71827_REG_LDO5_VOLT_L bits */
#define LDO5_L_MASK				0x3F

/* BD71827_REG_SEC bits */
#define	SEC_MASK(x)		((u8)x & 0x7F)

/* BD71827_REG_MIN bits */
#define	MIN_MASK(x)		((u8)x & 0x7F)

/* BD71827_REG_HOUR bits */
#define	HOUR_MASK(x)	((u8)x & 0x3F)
#define HOUR_24HOUR				0x80

/* BD71827_REG_WEEK bits */
#define	WEEK_MASK(x)	((u8)x & 0x07)

/* BD71827_REG_DAY bits */
#define	DAY_MASK(x)		((u8)x & 0x3F)

/* BD71827_REG_MONTH bits */
#define	MONTH_MASK(x)	((u8)x & 0x1F)

/* BD71827_REG_YEAR bits */
#define	YEAR_MASK(x)	((u8)x & 0xFF)

/** @brief charge state enumuration */
enum CHG_STATE {
	CHG_STATE_SUSPEND = 0x0,		/**< suspend state */
	CHG_STATE_TRICKLE_CHARGE,		/**< trickle charge state */
	CHG_STATE_PRE_CHARGE,			/**< precharge state */
	CHG_STATE_FAST_CHARGE,			/**< fast charge state */
	CHG_STATE_TOP_OFF,			/**< top off state */
	CHG_STATE_DONE,				/**< charge complete */
};

/** @brief rtc or alarm registers structure */
struct bd71827_rtc_alarm {
	u8	sec;
	u8	min;
	u8	hour;
	u8	week;
	u8	day;
	u8	month;
	u8	year;
};

struct bd71827;

/**
 * @brief Board platform data may be used to initialize regulators.
 */

struct bd71827_board {
	struct regulator_init_data *init_data[BD71827_REGULATOR_CNT];
	/**< regulator initialize data */
	int	gpio_intr;		/**< gpio connected to bd71827 INTB */
	int	irq_base;		/**< bd71827 sub irqs base #  */
};

/**
 * @brief bd71827 sub-driver chip access routines
 */

struct bd71827 {
	struct device *dev;
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct mutex io_mutex;
	unsigned int id;

	/* IRQ Handling */
	int 	chip_irq;		/**< bd71827 irq to host cpu */
	struct regmap_irq_chip_data *irq_data;

	/* Client devices */
	struct bd71827_pmic *pmic;	/**< client device regulator */
	struct bd71827_power *power;	/**< client device battery */

	struct bd71827_board *of_plat_data;
	/**< Device node parsed board data */
	struct input_dev *input;
};

static inline int bd71827_chip_id(struct bd71827 *bd71827)
{
	return bd71827->id;
}


/*
 * helper i2c write register routine to set up an i2c msg and send it out
 * without locking intentionally.
 */
static inline int bd71827_write_i2c_reg(struct i2c_client *client, int reg, int val)
{
        u8 temp_buf[2] = { reg, val };
        int ret;
        struct i2c_msg msg = {
                .addr = client->addr,
                .flags = 0,
                .len = 2,
                .buf = temp_buf,
        };

        ret = __i2c_transfer(client->adapter, &msg, 1);
        return (ret == 1)? 0 : -EBUSY;
}

/*
 * helper i2c read register routine. Similar to the write reg. routine.
 * This is done without locking intentionally.
 */
static inline int bd71827_read_i2c_reg(struct i2c_client *client, u8 reg, u8 *val)
{
        int ret;
        struct i2c_msg msg1 = {
                .addr = client->addr,
                .flags = 0,
                .len = 1,
                .buf = &reg,
        };
        struct i2c_msg msg2 = {
                .addr = client->addr,
                .flags = I2C_M_RD,
                .len = 1,
                .buf = val,
        };
        struct i2c_msg msgs[] = { msg1, msg2 };

        ret = __i2c_transfer(client->adapter, msgs, 2);
        return (ret == 2)? 0 : -EBUSY;
}


/*
 * Function to enter PMIC test mode.
 * This requires writing 3 separate values to the
 * BD71827_REG_I2c_MAGIC register. If any of these failed,
 * we will not be in test mode. No clean up is required.
 */
static inline int bd71827_enter_test_mode(struct i2c_client *client)
{
	int ret = 0, loop_count=0;
	u8 val = 0;

	//first check register 0x00 bit 7 to make sure I2C FIFO is empty before entering test mode
	while (loop_count < 5) {
		ret = bd71827_read_i2c_reg(client, BD71827_REG_DEVICE, &val);
		if (ret)
			goto out;
		if(val & 0x8)
			loop_count++;
		else
			break;
	}
	if (loop_count >= 5) {
		ret = -1;
		goto out;
	}

	ret = bd71827_write_i2c_reg(client, BD71827_REG_I2C_MAGIC,
				    BD71827_TEST_REG_ACCESS_AREA_1);
	if (ret)
		goto out;
	ret = bd71827_write_i2c_reg(client, BD71827_REG_I2C_MAGIC,
				    BD71827_TEST_REG_ACCESS_AREA_2);
	if (ret)
		goto out;
	ret = bd71827_write_i2c_reg(client, BD71827_REG_I2C_MAGIC,
				    BD71827_TEST_REG_ACCESS_AREA_3);
	if (ret)
		goto out;

	//Now check if BD71827_REG_I2C_MAGIC value is 0xFF(test mode)
	ret = bd71827_read_i2c_reg(client, BD71827_REG_I2C_MAGIC, &val);
	if (ret)
		goto out;
	if(val != 0xFF) {
		WARN_ON(1);
		ret = -1;
		goto out;
	}
out:
	return ret;
}

/*
 * Function to exit PMIC test mode.
 * This requires writing one value to the
 * BD71827_REG_I2C_MAGIC register. If writing failed (unlikely), just warn and retry
 */
static inline int bd71827_exit_test_mode(struct i2c_client *client)
{
	int ret;
	udelay(200);
	ret = bd71827_write_i2c_reg(client, BD71827_REG_I2C_MAGIC,
				    BD71827_TEST_REG_ACCESS_USER_AREA);
	if(ret) {
		/* Fail to exit test mode, this should not happen.
		 * Not much we can do, warn it and let's try again.
		 */
		WARN_ON(1);
		ret = bd71827_write_i2c_reg(client, BD71827_REG_I2C_MAGIC,
				      BD71827_TEST_REG_ACCESS_USER_AREA);
	}
	return ret;
}
/**
 * @brief bd71827_reg_read_nocheck
 * read single register's value of bd71827
 * @param bd71827 device to read
 * @param reg register address
 * @return register value if success
 *         error number if fail
 * Note: this function will not check the I2C_MAGIC, register is accessed as is.
 */
static inline int bd71827_reg_read_nocheck(struct bd71827 *bd71827, u8 reg)
{
	int r, val;

	r = regmap_read(bd71827->regmap, reg, &val);
	if (r < 0) {
		return r;
	}
	return val;
}

/**
 * @brief bd71827_reg_write
 * write single register of bd71827
 * @param bd71827 device to write
 * @param reg register address
 * @param val value to write
 * @retval 0 if success
 * @retval negative error number if fail
 * Note: this function does not check the I2C_MAGIC, register is accessed as is.
 */

static inline int bd71827_reg_write_nocheck(struct bd71827 *bd71827, u8 reg,
		unsigned int val)
{
	return regmap_write(bd71827->regmap, reg, val);
}

/**
 * @brief bd71827_reg_read
 * read single register's value of bd71827
 * @param bd71827 device to read
 * @param reg register address
 * @return register value if success
 *         error number if fail
 * Note: this function is update to check the I2C_MAGIC to limit the access
 * to user registers only, panic if test register access is open
 */
static inline int bd71827_reg_read(struct bd71827 *bd71827, u8 reg)
{
	int r;
	unsigned int val;

	r = regmap_read(bd71827->regmap, BD71827_REG_I2C_MAGIC, &val);
	if (r < 0) {
		return r;
	}
	if (val != 0)
	{
		printk(KERN_ERR "unexpected PMIC test register read to reg 0x%x\n",reg);
		WARN_ON(1);
		return -1;
	}

	r = regmap_read(bd71827->regmap, reg, &val);
	if (r < 0) {
		return r;
	}
	return val;
}

/**
 * @brief bd71827_reg_write
 * write single register of bd71827
 * @param bd71827 device to write
 * @param reg register address
 * @param val value to write
 * @retval 0 if success
 * @retval negative error number if fail
 * Note: this function is updated to check the I2C_MAGIC to limit the access
 * to user registers only, panic if test register access is open
 */

static inline int bd71827_reg_write(struct bd71827 *bd71827, u8 reg,
		unsigned int val)
{
	int r, value;

	r = regmap_read(bd71827->regmap, BD71827_REG_I2C_MAGIC, &value);
	if (r < 0) {
		return r;
	}
	if (value != 0)
	{
		printk(KERN_ERR "unexpected PMIC test register write to reg 0x%x, val:0x%x\n",reg, val);
		WARN_ON(1);
		return -1;
	}
	return regmap_write(bd71827->regmap, reg, val);
}

/**
 * @brief bd71827_set_bits
 * set bits in one register of bd71827
 * @param bd71827 device to read
 * @param reg register address
 * @param mask mask bits
 * @retval 0 if success
 * @retval negative error number if fail
 */
static inline int bd71827_set_bits(struct bd71827 *bd71827, u8 reg,
		u8 mask)
{
	int r;
	unsigned int value;

	r = regmap_read(bd71827->regmap, BD71827_REG_I2C_MAGIC, &value);
	if (r < 0) {
		return r;
	}
	if (value != 0)
	{
		printk(KERN_ERR "unexpected PMIC test register set_bits to reg 0x%x\n",reg);
		WARN_ON(1);
		return -1;
	}

	return regmap_update_bits(bd71827->regmap, reg, mask, mask);
}

/**
 * @brief bd71827_clear_bits
 * clear bits in one register of bd71827
 * @param bd71827 device to read
 * @param reg register address
 * @param mask mask bits
 * @retval 0 if success
 * @retval negative error number if fail
 */

static inline int bd71827_clear_bits(struct bd71827 *bd71827, u8 reg,
		u8 mask)
{
	int r;
	unsigned int value;

	r = regmap_read(bd71827->regmap, BD71827_REG_I2C_MAGIC, &value);
	if (r < 0) {
		return r;
	}
	if (value != 0)
	{
		printk(KERN_ERR "unexpected PMIC test register clear bits to reg 0x%x\n",reg);
		WARN_ON(1);
		return -1;
	}

	return regmap_update_bits(bd71827->regmap, reg, mask, 0);
}

/**
 * @brief bd71827_update_bits
 * update bits in one register of bd71827
 * @param bd71827 device to read
 * @param reg register address
 * @param mask mask bits
 * @param val value to update
 * @retval 0 if success
 * @retval negative error number if fail
 */

static inline int bd71827_update_bits(struct bd71827 *bd71827, u8 reg,
					   u8 mask, u8 val)
{
	int r;
	unsigned int value;

	r = regmap_read(bd71827->regmap, BD71827_REG_I2C_MAGIC, &value);
	if (r < 0) {
		return r;
	}
	if (value != 0)
	{
		printk(KERN_ERR "unexpected PMIC test register update bits to reg 0x%x\n",reg);
		WARN_ON(1);
		return -1;
	}

	return regmap_update_bits(bd71827->regmap, reg, mask, val);
}

/**
 * @brief bd71827 platform data type
 */
struct bd71827_gpo_plat_data {
	u32 drv;		///< gpo output drv
	int gpio_base;		///< base gpio number in system
};

struct int_status_reg {
	int reg;
	struct mutex lock;
};

#define BD718xx_METRIC_BUFFER_SIZE  128

u8 ext_bd71827_reg_read8(u8 reg);
int ext_bd71827_reg_write8(int reg, u8 val);

#define BD71827_DBG0		0x0001
#define BD71827_DBG1		0x0002
#define BD71827_DBG2		0x0004
#define BD71827_DBG3		0x0008

enum ROHM_PMIC_STATE {
	SHIP = 0x1,		/**ship mode */
	HIBERNATION = 0x02,	/* hibernation mode */
	PMIC_LSPR = 0x04,	/* LSPR mode */
	PMIC_IDLE = 0x08,	/* IDLE */
};

extern unsigned int bd71827_debug_mask;
#define bd71827_debug(debug, fmt, arg...) do { if(debug & bd71827_debug_mask) printk("BD7181x:" fmt, ##arg);} while(0)

#endif /* __LINUX_MFD_BD71827_H */
