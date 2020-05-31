/*
 * include/linux/input/sx9310_platform_data.h
 *
 * SX9310 Platform Data
 * 2 cap differential
 *
 * Copyright 2012 Semtech Corp.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _SX9310_PLATFORM_DATA_H_
#define _SX9310_PLATFORM_DATA_H_
#define DIFFERENTIAL



struct smtc_reg_data {
  unsigned char reg;
  unsigned char val;
};
typedef struct smtc_reg_data smtc_reg_data_t;
typedef struct smtc_reg_data *psmtc_reg_data_t;


struct _buttonInfo {
  /*! The Key to send to the input */
  int keycode;
  /*! Mask to look for on Touch Status */
  int mask;
  /*! Current state of button. */
  int state;
};

struct _totalButtonInformation {
  struct _buttonInfo *buttons;
  int buttonSize;
  struct input_dev *input;
};

typedef struct _totalButtonInformation buttonInformation_t;
typedef struct _totalButtonInformation *pbuttonInformation_t;


struct sx9310_platform_data {
  int i2c_reg_num;
  struct smtc_reg_data *pi2c_reg;

  pbuttonInformation_t pbuttonInformation;

  int (*get_is_nirq_low)(void);

  int     (*init_platform_hw)(void);
  void    (*exit_platform_hw)(void);
};
typedef struct sx9310_platform_data sx9310_platform_data_t;
typedef struct sx9310_platform_data *psx9310_platform_data_t;

#ifdef CONFIG_OF
/* Define Registers that need to be initialized to values different than
 * default
 */
static struct smtc_reg_data sx9310_i2c_reg_setup[] = {
	{
		.reg = SX9310_IRQSTAT_REG,
		.val = 0x10,
	},
	{
		.reg = SX9310_IRQ_ENABLE_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_IRQFUNC_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL0_REG,
		.val = 0x70,
	},
	{
		.reg = SX9310_CPS_CTRL1_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL2_REG,
		.val = 0x8B,
	},
	{
		.reg = SX9310_CPS_CTRL3_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL4_REG,
		.val = 0xFF,
	},
	{
		.reg = SX9310_CPS_CTRL5_REG,
		.val = 0xC1,
	},
	{
		.reg = SX9310_CPS_CTRL6_REG,
		.val = 0xA0,
	},
	{
		.reg = SX9310_CPS_CTRL7_REG,
		.val = 0x0C,
	},
	{
		.reg = SX9310_CPS_CTRL8_REG,
		.val = 0x25,
	},
	{
		.reg = SX9310_CPS_CTRL9_REG,
		.val = 0x23,
	},
	{
		.reg = SX9310_CPS_CTRL10_REG,
		.val = 0x15,
	},
	/*
	{
		.reg = SX9310_CPS_CTRL11_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL12_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL13_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL14_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL15_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL16_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL17_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL18_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL19_REG,
		.val = 0x00,
	},*/
	{
		.reg = SX9310_SAR_CTRL0_REG,
		.val = 0xD0,
	},
	{
		.reg = SX9310_SAR_CTRL1_REG,
		.val = 0x8A,
	},
	{
		.reg = SX9310_SAR_CTRL2_REG,
		.val = 0x3C,
	},
};

static struct _buttonInfo psmtcButtons[] = {
  {
    .keycode = KEY_0,
    .mask = SX9310_TCHCMPSTAT_TCHSTAT0_FLAG,
  },
  {
    .keycode = KEY_1,
    .mask = SX9310_TCHCMPSTAT_TCHSTAT1_FLAG,
  },
  {
    .keycode = KEY_2,
    .mask = SX9310_TCHCMPSTAT_TCHSTAT2_FLAG,
  },
  {
    .keycode = KEY_3,
    .mask = SX9310_TCHCMPSTAT_TCHSTAT3_FLAG,
  },
};

static struct _totalButtonInformation smtcButtonInformation = {
  .buttons = psmtcButtons,
  .buttonSize = ARRAY_SIZE(psmtcButtons),
};
#endif

#endif
