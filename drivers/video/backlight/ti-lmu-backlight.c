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

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mfd/ti-lmu.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#ifdef CONFIG_FRONTLIGHT
#include <linux/frontlight.h>
#endif

#include "ti-lmu-backlight.h"

#ifdef CONFIG_LAB126
#ifdef CONFIG_FALCON
extern int in_falcon(void);
#endif
#endif /* CONFIG_LAB126 */

#define DEFAULT_BL_NAME			"lcd-backlight"
#define DEFAULT_PWM_NAME		"lmu-backlight"

static int ti_lmu_backlight_enable(struct ti_lmu_bl *lmu_bl, int enable)
{
	const struct ti_lmu_bl_ops *ops = lmu_bl->chip->ops;

	if (ops->bl_enable)
		return ops->bl_enable(lmu_bl, enable);

	return 0;
}

static void ti_lmu_backlight_pwm_ctrl(struct ti_lmu_bl *lmu_bl, int br,
				      int max_br)
{
	struct pwm_device *pwm;
	unsigned int duty, period;

	/* Request a PWM device with the consumer name */
	if (!lmu_bl->pwm) {
		pwm = devm_pwm_get(lmu_bl->chip->dev, DEFAULT_PWM_NAME);
		if (IS_ERR(pwm)) {
			dev_err(lmu_bl->chip->dev,
				"Can not get PWM device, err: %ld\n",
				PTR_ERR(pwm));
			return;
		}

		lmu_bl->pwm = pwm;
	}

	period = lmu_bl->pwm_period;
	duty = br * period / max_br;

	pwm_config(lmu_bl->pwm, duty, period);
	if (duty)
		pwm_enable(lmu_bl->pwm);
	else
		pwm_disable(lmu_bl->pwm);
}

static int ti_lmu_backlight_get_brightness(struct backlight_device *bl_dev)
{
	struct ti_lmu_bl *lmu_bl = bl_get_data(bl_dev);
	const struct ti_lmu_bl_ops *ops = lmu_bl->chip->ops;

	if (ops->get_brightness)
		return ops->get_brightness(lmu_bl);

	return 0;
}

static int ti_lmu_backlight_update_status(struct backlight_device *bl_dev)
{
	struct ti_lmu_bl *lmu_bl = bl_get_data(bl_dev);
	const struct ti_lmu_bl_ops *ops = lmu_bl->chip->ops;
	int brightness = bl_dev->props.brightness;
	int ret;
#ifdef CONFIG_LAB126
#ifdef CONFIG_FALCON
	static int falcon_resume = 0;
#endif
#endif /* CONFIG_LAB126 */

	if (bl_dev->props.brightness > bl_dev->props.max_brightness) {
		/* do not set brightness if larger than max */
		return 0;
	}

	if (bl_dev->props.state & BL_CORE_SUSPENDED)
#ifdef CONFIG_LAB126
#ifdef CONFIG_FALCON
	{
		if (in_falcon())
			falcon_resume = 1;
#endif
#endif /* CONFIG_LAB126 */
		brightness = 0;
#ifdef CONFIG_LAB126
#ifdef CONFIG_FALCON
	} else if (falcon_resume)	{
		falcon_resume = 0;

		/* get the brightness value from HW which was set in uboot */
		brightness = ti_lmu_backlight_get_brightness(bl_dev);
		bl_dev->props.brightness = brightness;
	}
#endif
#endif /* CONFIG_LAB126 */

	if (brightness > 0)
		ret = ti_lmu_backlight_enable(lmu_bl, 1);
	else
		ret = ti_lmu_backlight_enable(lmu_bl, 0);
	if (ret)
		return ret;

	if (lmu_bl->mode == BL_PWM_BASED)
		ti_lmu_backlight_pwm_ctrl(lmu_bl, brightness,
					  bl_dev->props.max_brightness);

	/*
	 * In some devices, additional handling is required after PWM control.
	 * So, just call device-specific brightness function.
	 */
	if (ops->update_brightness)
		return ops->update_brightness(lmu_bl, brightness);

	return 0;
}

static const struct backlight_ops lmu_bl_common_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = ti_lmu_backlight_update_status,
};

static int ti_lmu_backlight_of_create(struct ti_lmu_bl_chip *chip,
				      struct device_node *np)
{
	struct device_node *child;
	struct ti_lmu_bl *lmu_bl, *each;
	int num_backlights;
	int i = 0;
	u32 imax;

	num_backlights = of_get_child_count(np);
	if (num_backlights == 0) {
		dev_err(chip->dev, "No backlight strings\n");
		return -EINVAL;
	}

	/* One chip can have mulitple backlight strings */
	lmu_bl = devm_kzalloc(chip->dev, sizeof(*lmu_bl) * num_backlights,
			      GFP_KERNEL);
	if (!lmu_bl)
		return -ENOMEM;

	for_each_child_of_node(np, child) {
		each = lmu_bl + i;
		each->bank_id = i;
		each->chip = chip;

		of_property_read_string(child, "backlight-name",
					&each->name);

		/* Backlight string configuration */
		each->bl_string = 0;
		if (of_property_read_bool(child, "hvled1-used"))
			each->bl_string |= LMU_HVLED1;
		if (of_property_read_bool(child, "hvled2-used"))
			each->bl_string |= LMU_HVLED2;
		if (of_property_read_bool(child, "hvled3-used"))
			each->bl_string |= LMU_HVLED3;

		imax = 0;
		of_property_read_u32(child, "backlight-max-microamp", &imax);
		each->imax = ti_lmu_get_current_code(imax);

		of_property_read_u32(child, "initial-brightness",
				     &each->init_brightness);

		/* Lighting effect properties */
		of_property_read_u32(child, "ramp-up-msec",
				     &each->ramp_up_msec);
		of_property_read_u32(child, "ramp-down-msec",
				     &each->ramp_down_msec);

		/* PWM mode */
		of_property_read_u32(child, "pwm-period", &each->pwm_period);
		if (each->pwm_period > 0)
			each->mode = BL_PWM_BASED;
		else
			each->mode = BL_REGISTER_BASED;

		each->chip = chip;
		i++;
	}

	chip->lmu_bl = lmu_bl;
	chip->num_backlights = num_backlights;

	return 0;
}

static int ti_lmu_backlight_configure(struct ti_lmu_bl *lmu_bl)
{
	const struct ti_lmu_bl_ops *ops = lmu_bl->chip->ops;

	if (ops->configure)
		return ops->configure(lmu_bl);

	return 0;
}

static int ti_lmu_backlight_reload(struct ti_lmu_bl_chip *chip)
{
	struct ti_lmu_bl *each;
	int i, ret;

	if (chip->ops->init) {
		ret = chip->ops->init(chip);
		if (ret)
			return ret;
	}

	for (i = 0; i < chip->num_backlights; i++) {
		each = chip->lmu_bl + i;
		ret = ti_lmu_backlight_configure(each);
		if (ret)
			return ret;

		backlight_update_status(each->bl_dev);
	}

	return 0;
}

static int ti_lmu_backlight_hwmon_notifier(struct notifier_block *nb,
					   unsigned long action, void *unused)
{
	struct ti_lmu_bl_chip *chip = container_of(nb, struct ti_lmu_bl_chip,
						   nb);
	int ret;

	/*
	 * LMU HWMON driver reset the device, so backlight should be
	 * re-initialized after hwmon detection procedure is done.
	 */
	if (action == LMU_EVENT_HWMON_DONE) {
		ret = ti_lmu_backlight_reload(chip);
		if (ret)
			return NOTIFY_STOP;
	}

	return NOTIFY_OK;
}

static int ti_lmu_backlight_init(struct ti_lmu_bl_chip *chip,
				 struct device_node *np)
{
	int ret;

	ret = ti_lmu_backlight_of_create(chip, np);
	if (ret)
		return ret;

	if (chip->ops->init) {
		ret = chip->ops->init(chip);
		if (ret)
			return ret;
	}

	/* Register notifier for LMU HWMON event */
	if (chip->ops->hwmon_notifier_used) {
		chip->nb.notifier_call = ti_lmu_backlight_hwmon_notifier;
		ret = blocking_notifier_chain_register(&chip->lmu->notifier,
						       &chip->nb);
		if (ret)
			return ret;
	}

	return 0;
}

static int ti_lmu_backlight_add_device(struct device *dev,
				       struct ti_lmu_bl *lmu_bl)
{
	struct backlight_device *bl_dev;
	struct backlight_properties props;
	int max_brightness = lmu_bl->chip->ops->max_brightness;
	char name[20];

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = max_brightness;
	if (lmu_bl->chip->ops->get_brightness)
		props.brightness = lmu_bl->chip->ops->get_brightness(lmu_bl);
	else
		props.brightness = lmu_bl->init_brightness;

	/* Backlight device name */
	if (!lmu_bl->name)
		snprintf(name, sizeof(name), "%s:%d", DEFAULT_BL_NAME,
			 lmu_bl->bank_id);
	else
		snprintf(name, sizeof(name), "%s", lmu_bl->name);

	bl_dev = devm_backlight_device_register(dev, name, lmu_bl->chip->dev,
						lmu_bl, &lmu_bl_common_ops,
						&props);
	if (IS_ERR(bl_dev))
		return PTR_ERR(bl_dev);

	lmu_bl->bl_dev = bl_dev;

#ifdef CONFIG_FRONTLIGHT
	frontlight_register(bl_dev);
#endif

	return 0;
}

static struct ti_lmu_bl_chip *
ti_lmu_backlight_register(struct device *dev, struct ti_lmu *lmu,
			  const struct ti_lmu_bl_ops *ops)
{
	struct device_node *np = dev->of_node;
	struct ti_lmu_bl_chip *chip;
	struct ti_lmu_bl *each;
	int i, ret;

	if (!ops) {
		dev_err(dev, "Operation is not configured\n");
		return ERR_PTR(-EINVAL);
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return ERR_PTR(-ENOMEM);

	chip->dev = dev;
	chip->lmu = lmu;
	chip->ops = ops;

	ret = ti_lmu_backlight_init(chip, np);
	if (ret) {
		dev_err(dev, "Backlight init err: %d\n", ret);
		return ERR_PTR(ret);
	}

	/* Add backlight strings */
	for (i = 0; i < chip->num_backlights; i++) {
		each = chip->lmu_bl + i;

		ret = ti_lmu_backlight_configure(each);
		if (ret) {
			dev_err(dev, "Backlight config err: %d\n", ret);
			return ERR_PTR(ret);
		}

		ret = ti_lmu_backlight_add_device(dev, each);
		if (ret) {
			dev_err(dev, "Backlight device err: %d\n", ret);
			return ERR_PTR(ret);
		}

		backlight_update_status(each->bl_dev);
	}

	return chip;
}

static void ti_lmu_backlight_unregister(struct ti_lmu_bl_chip *chip)
{
	struct ti_lmu_bl *each;
	int i;

	for (i = 0; i < chip->num_backlights; i++) {
		each = chip->lmu_bl + i;
		each->bl_dev->props.brightness = 0;
		backlight_update_status(each->bl_dev);
	}
}

/* Common probe() function for LMU backlight drivers */
int ti_lmu_backlight_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ti_lmu *lmu = dev_get_drvdata(dev->parent);
	struct ti_lmu_bl_chip *chip;
	const struct of_device_id *match;
	const struct ti_lmu_bl_ops *ops;

	match = of_match_device(dev->driver->of_match_table, dev);
	if (!match) {
		dev_err(dev, "No matched device is found\n");
		return -ENODEV;
	}

	/*
	 * Get device specific data(ti_lmu_bl_ops) from of_match table.
	 * This data is defined by TI_LMU_BL_OF_DEVICE() of each device.
	 */
	ops = (struct ti_lmu_bl_ops *)match->data;
	chip = ti_lmu_backlight_register(&pdev->dev, lmu, ops);
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	platform_set_drvdata(pdev, chip);

	return 0;
}
EXPORT_SYMBOL_GPL(ti_lmu_backlight_probe);

/* Common remove() function for LMU backlight drivers */
int ti_lmu_backlight_remove(struct platform_device *pdev)
{
	struct ti_lmu_bl_chip *chip = platform_get_drvdata(pdev);

	if (chip->ops->hwmon_notifier_used)
		blocking_notifier_chain_unregister(&chip->lmu->notifier,
						   &chip->nb);

	ti_lmu_backlight_unregister(chip);

	return 0;
}
EXPORT_SYMBOL_GPL(ti_lmu_backlight_remove);

/*
 * Convert ramp up/down time into register index value.
 * Get appropriate index by comparing ramp time table.
 */
int ti_lmu_backlight_get_ramp_index(struct ti_lmu_bl *lmu_bl,
				    enum ti_lmu_bl_ramp_mode mode)
{
	const int *table = lmu_bl->chip->ops->ramp_table;
	const int size = lmu_bl->chip->ops->size_ramp;
	unsigned int ramp_time;
	int i;

	if (!table)
		return -EINVAL;

	if (mode == BL_RAMP_UP)
		ramp_time = lmu_bl->ramp_up_msec;
	else if (mode == BL_RAMP_DOWN)
		ramp_time = lmu_bl->ramp_down_msec;
	else
		return -EINVAL;

	if (ramp_time <= table[0])
		return 0;

	if (ramp_time >= table[size - 1])
		return size - 1;

	for (i = 1; i < size; i++)
		if (ramp_time >= table[i - 1] && ramp_time < table[i])
			return i - 1;

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(ti_lmu_backlight_get_ramp_index);

MODULE_DESCRIPTION("TI LMU Backlight Common Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL v2");
