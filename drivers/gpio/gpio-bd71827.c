/*
 * gpio-bd71827.c
 * @file Access to GPOs on ROHM BD71827GW chip
 *
 * @author: cpham2403@gmail.com
 * Copyright 2016.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define DEBUG
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/mfd/bd71827.h>

/** @brief bd71827 gpio chip core data */
static struct gpio_chip bd71827gpo_chip;


/** @brief GPO Driver type constants */
static const char* bd71827_gpo_driver_types[] = {"open_drain", "cmos"};

/** @brief GPO2 Mode select constants
 * 0: GPO
 * 1: GPO
 * 2: LDO5_VSEL (input)
 * 3: PMIC_ON_REQ (input)
 * */
static const char* bd71827_gpo2_mode_selects[] = {"gpo", "gpo", "ldo5_vsel", "pmic_on_req"};

/** @brief GPO1 Mode select constants
 * 0: GPO
 * 1: GPO
 * 2: LDO4_EN (input)
 * 3: GPO
 * */
static const char* bd71827_gpo1_mode_selects[] = {"gpo", "gpo", "ldo4_en", "gpo"};

/** @brief retrieve GPO Driver type */
static ssize_t bd71827_show_gpo_driver_type(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bd71827 *mfd = dev_get_drvdata(dev);
	int type, r = 0;

	if(mfd == NULL) {
		return -EIO;
	}
	type = bd71827_reg_read(mfd, BD71827_REG_GPIO);
	if(type < 0)
		return type;
	r += sprintf(buf+r, "GPO1: %s\n", bd71827_gpo_driver_types[!!(type & GPO1_DRV_MASK)]);
	r += sprintf(buf+r, "GPO2: %s\n", bd71827_gpo_driver_types[!!(type & GPO2_DRV_MASK)]);

	return r;
}

/** @brief set GPO driver type */
static ssize_t bd71827_set_gpo_driver_type(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bd71827 *mfd = dev_get_drvdata(dev);
	int gpo_index = 1, type = 0, reg = BD71827_REG_GPIO, ret = 0, i = 0;
	char gpo_type[40];

	if(mfd == NULL) {
		return -EIO;
	}
	memset(gpo_type, 0, sizeof(gpo_type));

	if (sscanf(buf, "%d %s", &gpo_index, &gpo_type[0]) < 1) {
		dev_err(dev, "Usage: <index:1|2> <type:open_drain|cmos>\n");
		return -EINVAL;
	}
	printk(KERN_INFO "GPO%d: type = %s\n", gpo_index, gpo_type);
	if((gpo_index < 1) || (gpo_index > BD71827_GPIO_NUM)) {
		dev_err(dev, "Usage: <index:1|2> <type:open_drain|cmos>\n");
		return -EINVAL;
	}

	for( i = 0; i < ARRAY_SIZE(bd71827_gpo_driver_types); i++) {
		if (strncmp(gpo_type, bd71827_gpo_driver_types[i], strlen(bd71827_gpo_driver_types[i])) == 0) {
			type = i;
			break;
		}
	}
	if(i >= ARRAY_SIZE(bd71827_gpo_driver_types)) {
		dev_err(dev, "Not found <type:open_drain|cmos>\n");
		return -EINVAL;
	}

	ret = bd71827_update_bits(mfd, reg, 1<<(1 + gpo_index), type<<(1 + gpo_index));
	if (ret < 0) {
		dev_err(dev, "Update bits error\n");
		return ret;
	}
	return count;
}

/** @brief retrieve GPO Mode */
static ssize_t bd71827_show_gpo_mode_select(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bd71827 *mfd = dev_get_drvdata(dev);
	int select, r = 0;

	if(mfd == NULL) {
		return -EIO;
	}
	select = bd71827_reg_read(mfd, BD71827_REG_GPIO);
	if(select < 0)
		return select;
	r += sprintf(buf+r, "GPO1: %s\n", bd71827_gpo1_mode_selects[((select & GPIO1_MODE_MASK) >> 4)&0x3]);
	r += sprintf(buf+r, "GPO2: %s\n", bd71827_gpo2_mode_selects[((select & GPIO2_MODE_MASK) >> 6)&0x3]);

	return r;
}

/** @brief set GPIO Mode value */
static ssize_t bd71827_set_gpo_mode_select(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bd71827 *mfd = dev_get_drvdata(dev);
	int gpo_index = 1, select = 0, reg = BD71827_REG_GPIO, ret = 0, i = 0;
	char gpo_select[40];

	if(mfd == NULL) {
		return -EIO;
	}
	memset(gpo_select, 0, sizeof(gpo_select));

	if (sscanf(buf, "%d %s", &gpo_index, &gpo_select[0]) < 1) {
		dev_err(dev, "Show usage: 'cat available_gpo_mode_select'\n");
		return -EINVAL;
	}
	printk(KERN_INFO "GPO%d: select = %s\n", gpo_index, gpo_select);
	if((gpo_index < 1) || (gpo_index > BD71827_GPIO_NUM)) {
		dev_err(dev, "Show usage: 'cat available_gpo_mode_select'\n");
		return -EINVAL;
	}
	if(gpo_index == 1) {
		for( i = 0; i < ARRAY_SIZE(bd71827_gpo1_mode_selects); i++) {
		if (strncmp(gpo_select, bd71827_gpo1_mode_selects[i], strlen(bd71827_gpo1_mode_selects[i])) == 0) {
			select = i;
			break;
		}
		}
		if(i >= ARRAY_SIZE(bd71827_gpo1_mode_selects)) {
			dev_err(dev, "Show usage: 'cat available_gpo_mode_select'\n");
			return -EINVAL;
		}
	} else if(gpo_index == 2) {
		for( i = 0; i < ARRAY_SIZE(bd71827_gpo2_mode_selects); i++) {
		if (strncmp(gpo_select, bd71827_gpo2_mode_selects[i], strlen(bd71827_gpo2_mode_selects[i])) == 0) {
			select = i;
			break;
		}
		}
		if(i >= ARRAY_SIZE(bd71827_gpo2_mode_selects)) {
			dev_err(dev, "Show usage: 'cat available_gpo_mode_select'\n");
			return -EINVAL;
		}
	}

	ret = bd71827_update_bits(mfd, reg, 0x3<<(4 + (gpo_index - 1)*2), select<<(4 + (gpo_index - 1)*2));
	if (ret < 0) {
		dev_err(dev, "Update bits error\n");
		return ret;
	}
	return count;
}

/** @brief list all supported GPO Driver types */
static ssize_t bd71827_available_gpo_driver_types(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, r;

	r = 0;
	for (i = 0; i < ARRAY_SIZE(bd71827_gpo_driver_types) && r >= 0; i++) {
		r += sprintf(buf + r, "%s ", bd71827_gpo_driver_types[i]);
	}
	r += sprintf(buf + r, "\n");

	return r;
}

/** @brief list all supported GPO Mode selects */
static ssize_t bd71827_available_gpo_mode_selects(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, r;

	r = 0;
	//GPIO1
	r += sprintf(buf+r, "GPO1: ");
	for (i = 0; i < ARRAY_SIZE(bd71827_gpo1_mode_selects) && r >= 0; i++) {
		r += sprintf(buf + r, "%s ", bd71827_gpo1_mode_selects[i]);
	}
	r += sprintf(buf + r, "\n");
	//GPIO2
	r += sprintf(buf+r, "GPO2: ");
	for (i = 0; i < ARRAY_SIZE(bd71827_gpo2_mode_selects) && r >= 0; i++) {
		r += sprintf(buf + r, "%s ", bd71827_gpo2_mode_selects[i]);
	}
	r += sprintf(buf + r, "\n");

	return r;
}

static DEVICE_ATTR(gpo_driver_type, S_IWUSR | S_IRUGO, bd71827_show_gpo_driver_type, bd71827_set_gpo_driver_type);
static DEVICE_ATTR(gpo_mode_select, S_IWUSR | S_IRUGO, bd71827_show_gpo_mode_select, bd71827_set_gpo_mode_select);
static DEVICE_ATTR(available_gpo_driver_type, S_IWUSR | S_IRUGO, bd71827_available_gpo_driver_types, NULL);
static DEVICE_ATTR(available_gpo_mode_select, S_IWUSR | S_IRUGO, bd71827_available_gpo_mode_selects, NULL);

/** @brief device sysfs attribute table, about o */
static struct attribute *gpo_attributes[] = {
	&dev_attr_gpo_driver_type.attr,
	&dev_attr_gpo_mode_select.attr,
	&dev_attr_available_gpo_driver_type.attr,
	&dev_attr_available_gpo_mode_select.attr,
	NULL
};

static const struct attribute_group gpo_attr_group = {
	.attrs	= gpo_attributes,
};

/** @brief get gpo output value
 * @param chip pointer to core data
 * @param offset gpo number, start from 0
 * @retval 0 success
 * @retval negative error number
 */
static int bd71827gpo_get(struct gpio_chip *chip, unsigned offset)
{
	struct bd71827 *bd71827 = dev_get_drvdata(chip->dev->parent);
	int ret = 0;

	ret = bd71827_reg_read(bd71827, BD71827_REG_GPIO);
	if (ret < 0)
		return ret;

	return (ret >> offset) & 1;
}

/** @brief set gpo direction as output
 * @param chip pointer to core data
 * @param offset gpo number, start from 0
 * @param value output value when set direction out
 * @retval 0 success
 */
static int bd71827gpo_direction_out(struct gpio_chip *chip, unsigned offset,
				    int value)
{
	/* This only drives GPOs, and can't change direction */
	return 0;
}

/** @brief set gpo output value
 * @param chip pointer to core data
 * @param offset gpo number, start from 0
 * @param value output value, not zero as high level, 0 as low level
 * @retval 0 success
 * @retval negative error number
 */
static void bd71827gpo_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct bd71827 *bd71827 = dev_get_drvdata(chip->dev->parent);
	int ret;
	u8 gpoctl;

	ret = bd71827_reg_read(bd71827, BD71827_REG_GPIO);
	if (ret < 0)
		return;

	if (value)
		gpoctl = ret | (1 << offset);
	else
		gpoctl = ret & ~(1 << offset);

	bd71827_reg_write(bd71827, BD71827_REG_GPIO, gpoctl);
}

/** @brief bd71827 gpio chip core data */
static struct gpio_chip bd71827gpo_chip = {
	.label				= "bd71827",		///< gpio chip name
	.owner				= THIS_MODULE,
	.get				= bd71827gpo_get,
	.direction_output	= bd71827gpo_direction_out,
	.set				= bd71827gpo_set,
	.can_sleep			= 1,
};

/*----------------------------------------------------------------------*/
#if defined(BD71827_NONE_DTB)
/** @brief retrive gpo platform data from device tree
 * @param pdev platfrom device pointer
 * @return pointer to platform data
 * @retval NULL error
 */
static struct bd71827_gpo_plat_data *gpio_bd71827_fix_table(
	struct platform_device *pdev)
{
	struct bd71827_gpo_plat_data *platform_data;

	platform_data = devm_kzalloc(&pdev->dev, sizeof(*platform_data), GFP_KERNEL);
	if (!platform_data) {
		return NULL;
	}

	platform_data->drv = 0x0C; /* 0b0000_1100 all gpos with cmos output mode */

	return platform_data;
}
#else /* BD71827_NONE_DTB */
#ifdef CONFIG_OF
/** @brief retrive gpo platform data from device tree
 * @param pdev platfrom device pointer
 * @return pointer to platform data
 * @retval NULL error
 */
static struct bd71827_gpo_plat_data *of_gpio_bd71827(
	struct platform_device *pdev)
{
	struct bd71827_gpo_plat_data *platform_data;
	struct device_node *np, *gpio_np;

	platform_data = devm_kzalloc(&pdev->dev, sizeof(*platform_data), GFP_KERNEL);
	if (!platform_data) {
		return NULL;
	}

	np = of_node_get(pdev->dev.parent->of_node);
	gpio_np = of_find_node_by_name(np, "gpo");
	if (!gpio_np) {
		dev_err(&pdev->dev, "gpio node not found\n");
		return NULL;
	}

	pdev->dev.of_node = gpio_np;
	
	if (of_property_read_u32(gpio_np, "rohm,drv", &platform_data->drv)) {
		platform_data->drv = -1;
	}
	
	return platform_data;
}
#endif
#endif /* BD71827_NONE_DTB */

/** @brief probe bd71827 gpo device
 * @param pdev platfrom device pointer
 * @retval 0 success
 * @retval negative error number
 */
static int gpo_bd71827_probe(struct platform_device *pdev)
{
	struct bd71827_gpo_plat_data *pdata = pdev->dev.platform_data;
	struct device *mfd_dev = pdev->dev.parent;
	struct bd71827 *bd71827 = dev_get_drvdata(mfd_dev);
	int ret;

	platform_set_drvdata(pdev, bd71827);

#if defined(BD71827_NONE_DTB)
	pdata = gpio_bd71827_fix_table(pdev);
#else
#ifdef CONFIG_OF
	pdata = of_gpio_bd71827(pdev);
#endif
#endif

	if (pdata && pdata->gpio_base > 0)
		bd71827gpo_chip.base = pdata->gpio_base;
	else
		bd71827gpo_chip.base = -1;

	bd71827gpo_chip.ngpio = BD71827_GPIO_NUM;

	bd71827gpo_chip.dev = &pdev->dev;

	ret = gpiochip_add(&bd71827gpo_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register gpiochip, %d\n", ret);
		bd71827gpo_chip.ngpio = 0;
		return ret;
	}

	if (pdata && pdata->drv != -1UL) {
		bd71827_update_bits(bd71827, BD71827_REG_GPIO, GPO_DRV_MASK, pdata->drv);
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &gpo_attr_group);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to create attribute group: %d\n", ret);
	}

	return ret;
}

/** @brief remove bd71827 gpo device
 * @param pdev platfrom device pointer
 * @retval 0 success
 * @retval negative error number
 */
static int gpo_bd71827_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &gpo_attr_group);

	gpiochip_remove(&bd71827gpo_chip);
	return 0;
}

/* Note:  this hardware lives inside an I2C-based multi-function device. */
MODULE_ALIAS("platform:bd71827-gpo");

/** @brief bd71827 gpo driver core data */
static struct platform_driver gpo_bd71827_driver = {
	.driver = {
		.name	= "bd71827-gpo",
		.owner	= THIS_MODULE,
	},
	.probe		= gpo_bd71827_probe,
	.remove		= gpo_bd71827_remove,
};

module_platform_driver(gpo_bd71827_driver);

MODULE_AUTHOR("Cong Pham <cpham2403@gmail.com>");
MODULE_DESCRIPTION("GPO interface for BD71827");
MODULE_LICENSE("GPL");
