/*
 * TI LMU(Lighting Management Unit) Backlight Common Driver
 *
 * Copyright 2015 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __TI_LMU_BACKLIGHT_H__
#define __TI_LMU_BACKLIGHT_H__

#include <linux/mfd/ti-lmu.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>

/* Macro for LMU backlight of_device_id */
#define TI_LMU_BL_OF_DEVICE(chip, compat)				\
static const struct of_device_id chip##_bl_of_match[] = {		\
	{ .compatible = compat, .data = &chip##_lmu_ops },		\
	{ }								\
};									\
MODULE_DEVICE_TABLE(of, chip##_bl_of_match)				\

/* Macro for LMU backlight platform driver */
#define TI_LMU_BL_PLATFORM_DRIVER(chip, _name)				\
static struct platform_driver chip##_bl_driver = {			\
	.probe  = ti_lmu_backlight_probe,				\
	.remove = ti_lmu_backlight_remove,				\
	.driver = {							\
		.name = _name,						\
		.of_match_table = chip##_bl_of_match,			\
	},								\
};									\
									\
module_platform_driver(chip##_bl_driver)

enum ti_lmu_bl_ctrl_mode {
	BL_REGISTER_BASED,
	BL_PWM_BASED,
};

enum ti_lmu_bl_ramp_mode {
	BL_RAMP_UP,
	BL_RAMP_DOWN,
};

struct ti_lmu_bl;
struct ti_lmu_bl_chip;

/**
 * struct ti_lmu_bl_ops
 *
 * @init:		Device initialization function
 * @configure:		Device string configuration function
 * @update_brightness:	Device brightness control function
 * @bl_enable:		Device backlight enable/disable control function
 * @hwmon_notifier_used:Set true if the device needs to handle LMU HWMON event
 * @max_brightness:	Max brightness value of backlight device
 * @ramp_table:		Ramp time table for lighting effect.
 *			It's used for searching appropriate register index.
 *			Please refer to ti_lmu_backlight_get_ramp_index().
 * @size_ramp:		Size of ramp table
 *
 * TI LMU backlight device should call ti_lmu_backlight_register() with
 * each device operation.
 */
struct ti_lmu_bl_ops {
	int (*init)(struct ti_lmu_bl_chip *lmu_chip);
	int (*configure)(struct ti_lmu_bl *lmu_bl);
	int (*update_brightness)(struct ti_lmu_bl *lmu_bl, int brightness);
	int (*get_brightness)(struct ti_lmu_bl *lmu_bl);
	int (*bl_enable)(struct ti_lmu_bl *lmu_bl, int enable);
	bool hwmon_notifier_used;
	const int max_brightness;
	const int *ramp_table;
	const int size_ramp;
};

/**
 * struct ti_lmu_bl_chip
 *
 * @dev:		Parent device pointer
 * @lmu:		LMU structure. Used for register R/W access.
 * @ops:		Device specific operation
 * @lmu_bl:		Multiple backlight strings
 * @num_backlights:	Number of backlight strings
 * @nb:			Notifier block for handling hwmon event
 *
 * One backlight chip can have multiple backlight strings, 'ti_lmu_bl'.
 */
struct ti_lmu_bl_chip {
	struct device *dev;
	struct ti_lmu *lmu;
	const struct ti_lmu_bl_ops *ops;
	struct ti_lmu_bl *lmu_bl;
	int num_backlights;
	struct notifier_block nb;
};

/**
 * struct ti_lmu_bl
 *
 * @chip:		Pointer to parent backlight device
 * @bl_dev:		Backlight subsystem device structure
 * @bank_id:		Backlight bank ID
 * @name:		Backlight channel name
 * @mode:		Backlight control mode
 * @bl_string:		Backlight string configuration.
 *			Bit mask is set on parsing DT.
 * @imax:		[Optional] Max current index.
 *			It's result of ti_lmu_get_current_code().
 * @init_brightness:	[Optional] Initial brightness value
 * @ramp_up_msec:	[Optional] Ramp up time
 * @ramp_down_msec:	[Optional] Ramp down time
 * @pwm_period:		[Optional] PWM period
 * @pwm:		[Optional] PWM subsystem structure
 *
 * Each backlight device has its own channel configuration.
 * For chip control, parent chip data structure is used.
 */
struct ti_lmu_bl {
	struct ti_lmu_bl_chip *chip;
	struct backlight_device *bl_dev;

	int bank_id;
	const char *name;
	enum ti_lmu_bl_ctrl_mode mode;
	unsigned long bl_string;	/* bit OR mask of LMU_HVLEDx */
	#define LMU_HVLED1	BIT(0)
	#define LMU_HVLED2	BIT(1)
	#define LMU_HVLED3	BIT(2)

	enum ti_lmu_max_current imax;
	unsigned int init_brightness;

	/* Used for lighting effect */
	unsigned int ramp_up_msec;
	unsigned int ramp_down_msec;

	/* Only valid in case of PWM mode */
	unsigned int pwm_period;
	struct pwm_device *pwm;
};

int ti_lmu_backlight_probe(struct platform_device *pdev);
int ti_lmu_backlight_remove(struct platform_device *pdev);
int ti_lmu_backlight_get_ramp_index(struct ti_lmu_bl *lmu_bl,
				    enum ti_lmu_bl_ramp_mode mode);
#endif
