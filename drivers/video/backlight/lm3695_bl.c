/*
 * TI LM3695 Backlight Driver
 *
 * Copyright 2015 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mfd/ti-lmu.h>
#include <linux/mfd/ti-lmu-register.h>
#include <linux/module.h>

#include "ti-lmu-backlight.h"

#define LM3695_FULL_STRINGS		(LMU_HVLED1 | LMU_HVLED2)
#define LM3695_MAX_BRIGHTNESS		2047
#define LM3695_CONFIGURE		(LM3695_BOOST_FREQ_SEL_MASK|LM3695_OVP_SEL_MASK|LM3695_RAMP_DISABLE_MASK)

static inline int lm3695_bl_config_value(struct ti_lmu_bl *lmu_bl)
{
	if (lmu_bl->bl_string == LM3695_FULL_STRINGS)
		return LM3695_CONFIGURE | LM3695_BRT_RW_MASK | LM3695_BL_TWO_STRINGS;
	else
		return LM3695_CONFIGURE | LM3695_BRT_RW_MASK | LM3695_BL_ONE_STRING;
}

static int lm3695_bl_enable(struct ti_lmu_bl *lmu_bl, int enable)
{
	int ret;
	u8 data, orig;

	ret = ti_lmu_read_byte(lmu_bl->chip->lmu, LM3695_REG_GP, &orig);
	if (ret)
		return ret;

	data = lm3695_bl_config_value(lmu_bl) & ~LM3695_BL_EN_MASK;
	data |= enable;

	if (data != orig) {
		ret = ti_lmu_write_byte(lmu_bl->chip->lmu, LM3695_REG_GP, data);
		if (ret)
			return ret;

		/* Wait time for brightness register wake up */
		usleep_range(600, 700);
	}

	return 0;
}

static int lm3695_bl_set_brightness(struct ti_lmu_bl *lmu_bl, int brightness)
{
	u8 data;
	int ret;

	data = brightness & LM3695_BRT_LSB_MASK;
	ret = ti_lmu_update_bits(lmu_bl->chip->lmu, LM3695_REG_BRT_LSB,
				 LM3695_BRT_LSB_MASK, data);
	if (ret)
		return ret;

	data = (brightness >> LM3695_BRT_MSB_SHIFT) & 0xFF;
	return ti_lmu_write_byte(lmu_bl->chip->lmu, LM3695_REG_BRT_MSB,
				 data);
}

static int lm3695_bl_get_brightness(struct ti_lmu_bl *lmu_bl)
{
	int brightness;
	u8 data;
	int ret;

	ret = ti_lmu_read_byte(lmu_bl->chip->lmu, LM3695_REG_BRT_MSB, &data);
	if (ret < 0)
		return 0;
	brightness = data;

	ret = ti_lmu_read_byte(lmu_bl->chip->lmu, LM3695_REG_BRT_LSB, &data);
	if (ret < 0)
		return 0;
	brightness = ((brightness & 0xFF) << LM3695_BRT_MSB_SHIFT) |
			(data & LM3695_BRT_LSB_MASK);

	return brightness;
}

static int lm3695_bl_init(struct ti_lmu_bl_chip *chip)
{
	return ti_lmu_update_bits(chip->lmu, LM3695_REG_GP,
				  LM3695_BRT_RW_MASK, LM3695_BRT_RW_MASK);
}

static int lm3695_bl_configure(struct ti_lmu_bl *lmu_bl)
{

	return ti_lmu_update_bits(lmu_bl->chip->lmu, LM3695_REG_GP,
		LM3695_BOOST_FREQ_SEL_MASK|LM3695_OVP_SEL_MASK|LM3695_BL_STRING_MASK|LM3695_RAMP_DISABLE_MASK,
		lm3695_bl_config_value(lmu_bl));
}

static const struct ti_lmu_bl_ops lm3695_lmu_ops = {
	.init			= lm3695_bl_init,
	.configure		= lm3695_bl_configure,
	.update_brightness	= lm3695_bl_set_brightness,
	.get_brightness	= lm3695_bl_get_brightness,
	.bl_enable		= lm3695_bl_enable,
	.max_brightness		= LM3695_MAX_BRIGHTNESS,
};

/* LM3695 backlight of_device_id */
TI_LMU_BL_OF_DEVICE(lm3695, "ti,lm3695-backlight");

/* LM3695 backlight platform driver */
TI_LMU_BL_PLATFORM_DRIVER(lm3695, "lm3695-backlight");

MODULE_DESCRIPTION("TI LM3695 Backlight Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:lm3695-backlight");
