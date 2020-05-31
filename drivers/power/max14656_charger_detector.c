/*
 * Maxim MAX14656 / AL32 USB Charger Detector driver
 *
 * Copyright (C) 2014 LG Electronics, Inc
 * Copyright (C) 2016 Alexander Kurz <akurz@blala.de>
 *
 * Components from Maxim AL32 Charger detection Driver for MX50 Yoshi Board
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Manish Lachwani (lachwani@lab126.com)
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/mutex.h>

#ifdef CONFIG_CPU_FREQ_OVERRIDE_LAB126
#include <linux/cpufreq.h>
#endif

#define MAX14656_MANUFACTURER	"Maxim Integrated"
#define MAX14656_NAME			"max14656"

#define MAX14656_OF_NODE_NAME	"maxim,max14656"

#define MAX14656_DEVICE_ID		0x00
#define MAX14656_INTERRUPT_1	0x01
#define MAX14656_INTERRUPT_2	0x02
#define MAX14656_STATUS_1		0x03
#define MAX14656_STATUS_2		0x04
#define MAX14656_INTMASK_1		0x05
#define MAX14656_INTMASK_2		0x06
#define MAX14656_CONTROL_1		0x07
#define MAX14656_CONTROL_2		0x08
#define MAX14656_CONTROL_3		0x09

#define DEVICE_VENDOR_MASK		0xf0
#define DEVICE_REV_MASK			0x0f
#define DCD_TMR_INT_MASK		BIT(7)
#define INT_EN_REG_MASK			BIT(4)
#define CHG_TYPE_INT_MASK		BIT(0)
#define STATUS1_OVP_MASK		BIT(5)
#define STATUS1_VB_VALID_MASK	BIT(4)
#define STATUS1_CHG_TYPE_MASK	0x0f
#define INT1_DCD_TIMEOUT_MASK	BIT(7)
#define CHG_TYPE_MAN_MASK		BIT(1)
#define CHG_DET_EN_MASK			BIT(0)

#define CONTROL1_DEFAULT		0x0d
#define CONTROL1_INT_EN			BIT(4)
#define CONTROL1_INT_HIGH		BIT(5)
#define CONTROL1_EDGE			BIT(7)
#define CONTROL2_DEFAULT		0x8e
#define CONTROL2_USB_CPL		BIT(2)
#define CONTROL2_ADC_EN			BIT(0)
#define CONTROL3_DEFAULT		0x8d
#define CONTROL3_DCD_EN			BIT(2)
#define CONTROL3_DET_MAN_EN		BIT(1)
#define CONTROL3_CHG_DET_EN		BIT(0)
#define INTMASK1_DCD_TMR_M		BIT(7)
#define INTMASK1_VB_VALID_M		BIT(1)
#define INTMASK1_CHG_TYP_M		BIT(0)
#define INTMASK2_ADC_M			BIT(0)

#define REG_TO_READ				5

/* Bits operation helper begin */
#define U8_FFS(_x) \
	((_x) & 0x0F ? \
		((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) : \
		               ((_x) & 0x04 ? 2 : 3)) \
		         : \
		((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) : \
		               ((_x) & 0x40 ? 6 : 7)) \
	)

#ifdef  BITS
#undef  BITS
#endif
#define BITS(_msb, _lsb) \
	(BIT(_msb) - BIT(_lsb) + BIT(_msb))

#define BITS_MASKED_GET(_val, _mask) \
	((_val) & (_mask)) >> U8_FFS(_mask)

#define BITS_MASKED_SET(_var, _mask, _val) \
	(_var) &= ~(_mask); \
	(_var) |= ((_val) << U8_FFS(_mask)) & (_mask);

#define U4_TO_BINARY_FORMAT "0b%c%c%c%c"
#define U4_TO_BINARY(_val) \
	((_val) & 0x08 ? '1' : '0'), \
	((_val) & 0x04 ? '1' : '0'), \
	((_val) & 0x02 ? '1' : '0'), \
	((_val) & 0x01 ? '1' : '0')

#define U2_TO_BINARY_FORMAT "0b%c%c"
#define U2_TO_BINARY(_val) \
	((_val) & 0x02 ? '1' : '0'), \
	((_val) & 0x01 ? '1' : '0')

#define U1_TO_BINARY_FORMAT "%c"
#define U1_TO_BINARY(_val) \
	((_val) & 0x01 ? '1' : '0')
/* Bits operation helper end */

#define USB_SWC_MASK			BITS(3,2)
#define USB_SWC_FORMAT			U2_TO_BINARY_FORMAT
#define USB_SWC_VALUE			U2_TO_BINARY

#define DETECT_WORK_DELAY		2000

/*
 * Changer types are defined by Maxim, don't insert or reorder
 */
enum max14656_chg_type {
	MAX14656_NO_CHARGER	= 0,
	MAX14656_SDP_CHARGER,
	MAX14656_CDP_CHARGER,
	MAX14656_DCP_CHARGER,
	MAX14656_APPLE_500MA_CHARGER,
	MAX14656_APPLE_1A_CHARGER,
	MAX14656_APPLE_2A_CHARGER,
	MAX14656_SPECIAL_500MA_CHARGER,

	MAX14656_APPLE_RFU_CHARGER = 0b1100,

	MAX14656_CHARGER_LAST
};


struct max14656_chg_type_props {
	enum power_supply_type type;
	int current_limit;
};

static const struct max14656_chg_type_props chg_type_props[] = {
	{ POWER_SUPPLY_TYPE_UNKNOWN,  500 },
	{ POWER_SUPPLY_TYPE_USB,      500 },
	{ POWER_SUPPLY_TYPE_USB_CDP, 1500 },
	{ POWER_SUPPLY_TYPE_USB_DCP, 1500 },
	{ POWER_SUPPLY_TYPE_USB_DCP,  500 },
	{ POWER_SUPPLY_TYPE_USB_DCP, 1000 },
	{ POWER_SUPPLY_TYPE_USB_DCP, 2000 },
	{ POWER_SUPPLY_TYPE_USB,      500 },

	{ POWER_SUPPLY_TYPE_UNKNOWN,  500 },
	{ POWER_SUPPLY_TYPE_UNKNOWN,  500 },
	{ POWER_SUPPLY_TYPE_UNKNOWN,  500 },
	{ POWER_SUPPLY_TYPE_UNKNOWN,  500 },

	/* According to test result, Apple RFU charger supports 2A charging current */
	{ POWER_SUPPLY_TYPE_USB_DCP, 2000 },
};

struct max14656_chip {
	struct i2c_client				*client;
	const struct attribute_group	*attrs_group;

	struct delayed_work				detect_work;
	struct mutex					work_lock;

	int								gpio_chg_det_int;
	int								irq_chg_det_int;
	int								gpio_chg_det_ce;

	struct kobject					*kobj;

#define ONLINE						1
#define FIRSTIME                    2
	unsigned long					bitflags;
};

/*
 * Read single register
 */
static int max14656_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		pr_err("%s:%d i2c read fail: can't read from %02x: %d\n", __func__, __LINE__, reg, ret);
		return ret;
	}
	*val = ret;
	return 0;
}

/*
 * Write single register
 */
static int max14656_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		pr_err("%s:%d i2c write fail: can't write %02x to %02x: %d\n", __func__, __LINE__, val, reg, ret);
		return ret;
	}
	return 0;
}

/*
 * Read contiguous registers
 */
static int max14656_read_block_reg(struct i2c_client *client, u8 reg, u8 length, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, length, val);
	if (ret < 0) {
		pr_err("%s:%d failed to block read reg 0x%x: %d\n", __func__, __LINE__, reg, ret);
		return ret;
	}

	return 0;
}

/*
 * Update register bits
 */
static int max14656_update_reg_bits(struct i2c_client *client, u8 reg, u8 mask, u8 val)
{
	u8 reg_val;

	if (max14656_read_reg(client, reg, &reg_val))
		return -ENODEV;

	BITS_MASKED_SET(reg_val, mask, val);

	if (max14656_write_reg(client, reg, reg_val))
		return -ENODEV;

	return 0;
}

#ifdef CONFIG_LAB126
extern int pmic_current_event_handler(unsigned long event, void *ptr);
#endif /* CONFIG_LAB126 */

#ifdef CONFIG_USB_REX
extern void usbotg_force_bsession(bool connected);
extern int  ci_hdrc_get_phy_vcc(unsigned controller_id);
extern void ci_hdrc_set_phy_vcc(unsigned controller_id, bool vcc_enable, bool vcc_force);

/* DONT reuse g_chip, it acts as an enclosure to max14656_set_ums_cb */
static struct max14656_chip *g_chip = NULL;
static void (*ums_cb)(bool connected);

/* This function is called after binding fsg */
void max14656_set_ums_cb(void (*func)(bool connected))
{
	struct max14656_chip *chip = g_chip;
	if (!g_chip) {
		return;
	}

	ums_cb = func;
}
EXPORT_SYMBOL(max14656_set_ums_cb);

void usb_gadget_chg_det_sync(int delay_ms)
{
	struct max14656_chip *chip = g_chip;
	if (!g_chip) {
	    pr_info("charger driver is ready \n");
		return;
	}
#ifdef CONFIG_CPU_FREQ_OVERRIDE_LAB126
    cpufreq_override(2);
#endif
    pr_info("%s\n", __func__);
    cancel_delayed_work(&chip->detect_work);
    schedule_delayed_work(&chip->detect_work, msecs_to_jiffies(delay_ms));
}
EXPORT_SYMBOL(usb_gadget_chg_det_sync);
#endif /* CONFIG_USB_REX */

static void max14656_detect_worker(struct work_struct *work)
{
	struct max14656_chip *chip =
		container_of(work, struct max14656_chip, detect_work.work);
    u8 buf[REG_TO_READ] = {0};
    u8 chg_type = 0;
    int ret = 0;

    mutex_lock(&chip->work_lock);

#ifdef CONFIG_CPU_FREQ_OVERRIDE_LAB126
    cpufreq_override(2);
#endif

    ret = max14656_read_block_reg(chip->client, MAX14656_DEVICE_ID,
            REG_TO_READ, buf);
    if (ret != 0) {
        pr_err("%s:%d failed to read registers\n", __func__, __LINE__);
        mutex_unlock(&chip->work_lock);
        return ret;
    }

    pr_info("%s:%d 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", __func__, __LINE__,
        buf[MAX14656_INTERRUPT_1], buf[MAX14656_INTERRUPT_2],
        buf[MAX14656_STATUS_1], buf[MAX14656_STATUS_2]);

    chg_type = BITS_MASKED_GET(buf[MAX14656_STATUS_1], STATUS1_CHG_TYPE_MASK);
    pr_info("%s:%d chg_type="U4_TO_BINARY_FORMAT"\n", __func__, __LINE__, U4_TO_BINARY(chg_type));

    if ((chg_type != MAX14656_NO_CHARGER) && (buf[MAX14656_STATUS_1] & STATUS1_VB_VALID_MASK)) {
#ifdef CONFIG_LAB126
        if (chg_type < MAX14656_CHARGER_LAST) {
            pmic_current_event_handler(chg_type_props[chg_type].type,
            (void *)chg_type_props[chg_type].current_limit);
        } else {
            pmic_current_event_handler(POWER_SUPPLY_TYPE_UNKNOWN,
            (void *)chg_type_props[POWER_SUPPLY_TYPE_UNKNOWN].current_limit);
        }
#endif /* CONFIG_LAB126 */
        pr_info("charger connected:%d bitflags: 0x%02lx\n", __LINE__, chip->bitflags);
        kobject_uevent(chip->kobj, KOBJ_ONLINE);
        if (!test_and_set_bit(ONLINE, &chip->bitflags) ||
            !test_and_set_bit(FIRSTIME, &chip->bitflags)) {
#ifdef CONFIG_USB_REX
            switch (chg_type) {
                case MAX14656_SDP_CHARGER:
                /* No break here, fallthough next statement */
                case MAX14656_CDP_CHARGER:
                    {
                        pr_info("turn on VBUS\n");
                        ci_hdrc_set_phy_vcc(0, true, false);
                        usbotg_force_bsession(1);
                    }
                }
            }
#endif /* CONFIG_USB_REX */
    } else {
#ifdef CONFIG_LAB126
        pmic_current_event_handler(POWER_SUPPLY_TYPE_UNKNOWN,
        (void *)chg_type_props[POWER_SUPPLY_TYPE_UNKNOWN].current_limit);
#endif /* CONFIG_LAB126 */
#ifdef CONFIG_USB_REX
        pr_info("no charger connected:%d bitflags: 0x%02lx\n", __LINE__, chip->bitflags);
        if (test_and_clear_bit(ONLINE, &chip->bitflags) ||
            !test_and_set_bit(FIRSTIME, &chip->bitflags)) {
            usbotg_force_bsession(0);
            ci_hdrc_set_phy_vcc(0, false, false);
            pr_info("turn off VBUS\n");
        }
         kobject_uevent(chip->kobj, KOBJ_OFFLINE);
#endif /* CONFIG_USB_REX */
    }
    mutex_unlock(&chip->work_lock);
    return ret;
}

static int max14656_hw_init(struct max14656_chip *chip)
{
	struct i2c_client *client = chip->client;
	u8 val = 0;
	u8 rev = 0;

	if (max14656_read_reg(client, MAX14656_DEVICE_ID, &val))
		return -ENODEV;

	if ((val & DEVICE_VENDOR_MASK) != 0x20) {
		pr_err("%s:%d wrong vendor ID %d\n", __func__, __LINE__, ((val & DEVICE_VENDOR_MASK) >> 4));
		return -ENODEV;
	}
	rev = val & DEVICE_REV_MASK;

	if (max14656_write_reg(client, MAX14656_INTMASK_1, INTMASK1_CHG_TYP_M /* | INTMASK1_VB_VALID_M */))
		return -EINVAL;

	if (max14656_write_reg(client, MAX14656_INTMASK_2, INTMASK2_ADC_M))
		return -EINVAL;

	if (max14656_write_reg(client, MAX14656_CONTROL_2, CONTROL2_DEFAULT /* CONTROL2_ADC_EN | CONTROL2_USB_CPL */))
		return -EINVAL;

	if (max14656_write_reg(client, MAX14656_CONTROL_3, CONTROL3_DEFAULT))
		return -EINVAL;

	dev_info(&client->dev, "detected revision %d\n", rev);
	return 0;
}

/*
 * Sys entries
 */
static ssize_t chg_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max14656_chip *chip = dev_get_drvdata(dev);
	u8 val = 0;

	if (max14656_read_reg(chip->client, MAX14656_STATUS_1, &val))
		return scnprintf(buf, PAGE_SIZE, "\n");

	val = BITS_MASKED_GET(val, STATUS1_CHG_TYPE_MASK);

	return scnprintf(buf, PAGE_SIZE, U4_TO_BINARY_FORMAT"\n", U4_TO_BINARY(val));
}

static ssize_t online_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max14656_chip *chip = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", test_bit(ONLINE, &chip->bitflags) ? 1 : 0);
}

static ssize_t ovp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max14656_chip *chip = dev_get_drvdata(dev);
	u8 val = 0;

	if (max14656_read_reg(chip->client, MAX14656_STATUS_1, &val))
		return scnprintf(buf, PAGE_SIZE, "\n");

	return scnprintf(buf, PAGE_SIZE, "%d\n", (val & STATUS1_OVP_MASK) ? 1 : 0);
}

static ssize_t usb_swc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max14656_chip *chip = dev_get_drvdata(dev);
	u8 val = 0;

	if (max14656_read_reg(chip->client, MAX14656_CONTROL_1, &val))
		return scnprintf(buf, PAGE_SIZE, "\n");

	val = BITS_MASKED_GET(val, USB_SWC_MASK);

	return scnprintf(buf, PAGE_SIZE, USB_SWC_FORMAT"\n", USB_SWC_VALUE(val));
}

static ssize_t usb_swc_store(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
	struct max14656_chip *chip = dev_get_drvdata(dev);
	u8 usb_swc = 0;

	usb_swc = (u8)simple_strtoul(buf, NULL, 2);

	if (max14656_update_reg_bits(chip->client, MAX14656_CONTROL_1, USB_SWC_MASK, usb_swc))
		return -ENODEV;

	return count;
}

static ssize_t chg_type_man_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max14656_chip *chip = dev_get_drvdata(dev);
	u8 val = 0;

	if (max14656_read_reg(chip->client, MAX14656_CONTROL_3, &val))
		return scnprintf(buf, PAGE_SIZE, "\n");

	return scnprintf(buf, PAGE_SIZE, "%d\n", (val & CHG_TYPE_MAN_MASK) ? 1 : 0);
}

static ssize_t chg_type_man_store(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
	struct max14656_chip *chip = dev_get_drvdata(dev);
	u8 chg_type_man = 0;

	chg_type_man = simple_strtoul(buf, NULL, 2) ? 1 : 0;

	/* Trigger a manual detection */
	if (max14656_update_reg_bits(chip->client, MAX14656_CONTROL_3, CHG_TYPE_MAN_MASK, chg_type_man))
		return -ENODEV;

	/* Schedule deferred detection in case of no interrupt for manual detection */
	cancel_delayed_work(&chip->detect_work);
	schedule_delayed_work(&chip->detect_work, msecs_to_jiffies(DETECT_WORK_DELAY));

	return count;
}

static ssize_t chg_det_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max14656_chip *chip = dev_get_drvdata(dev);
	u8 val = 0;

	if (max14656_read_reg(chip->client, MAX14656_CONTROL_3, &val))
		return scnprintf(buf, PAGE_SIZE, "\n");

	return scnprintf(buf, PAGE_SIZE, "%d\n", (val & CHG_DET_EN_MASK) ? 1 : 0);
}

static ssize_t chg_det_en_store(struct device *dev,
			struct device_attribute *devattr, const char *buf, size_t count)
{
	struct max14656_chip *chip = dev_get_drvdata(dev);
	u8 chg_det_en = 0;

	chg_det_en = simple_strtoul(buf, NULL, 2) ? 1 : 0;

	if (max14656_update_reg_bits(chip->client, MAX14656_CONTROL_3, CHG_DET_EN_MASK, chg_det_en))
		return -ENODEV;

	return count;
}

static DEVICE_ATTR_RO(chg_type);
static DEVICE_ATTR_RO(online);
static DEVICE_ATTR_RO(ovp);
static DEVICE_ATTR_RW(usb_swc);
static DEVICE_ATTR_RW(chg_type_man);
static DEVICE_ATTR_RW(chg_det_en);

static struct attribute *max14656_attrs[] = {
	&dev_attr_chg_type.attr,
	&dev_attr_online.attr,
	&dev_attr_ovp.attr,
	&dev_attr_usb_swc.attr,
	&dev_attr_chg_type_man.attr,
	&dev_attr_chg_det_en.attr,
	NULL
};

static const struct attribute_group max14656_attrs_group = {
    .attrs = max14656_attrs,
};

/*
 * Probe function
 */
static int max14656_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct i2c_adapter *adapter = to_i2c_adapter(dev->parent);
	struct max14656_chip *chip;
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s:%d No support for SMBUS_BYTE_DATA\n", __func__, __LINE__);
		return -ENODEV;
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	dev_set_drvdata(dev, chip);

	chip->client = client;
	chip->kobj = &dev->kobj;

	mutex_init(&chip->work_lock);
	INIT_DELAYED_WORK(&chip->detect_work, max14656_detect_worker);

	ret = max14656_hw_init(chip);
	if (ret) {
		ret = -ENODEV;
		goto err_free_chip;
	}

	chip->attrs_group = &max14656_attrs_group;
	ret = sysfs_create_group(chip->kobj, chip->attrs_group);
	if (unlikely(ret)) {
		pr_err("%s:%d failed to create attribute group ret=[%d]\n", __func__, __LINE__, ret);
		chip->attrs_group = NULL;
		goto err_free_chip;
	}

#ifdef CONFIG_USB_REX
	g_chip = chip;
#endif /* CONFIG_USB_REX */
	return 0;

err_free_chip:
    if (chip->attrs_group) {
        sysfs_remove_group(chip->kobj, chip->attrs_group);
    }
	if (chip->gpio_chg_det_int) {
		devm_free_irq(dev, gpio_to_irq(chip->gpio_chg_det_int), chip);
	}
	if (chip->gpio_chg_det_ce) {
		devm_free_irq(dev, gpio_to_irq(chip->gpio_chg_det_ce), chip);
	}
	dev_set_drvdata(dev, NULL);
	devm_kfree(dev, chip);
	return ret;
}

#ifdef CONFIG_PM_SLEEP

static int max14656_suspend(struct device *dev)
{
	struct max14656_chip *chip = dev_get_drvdata(dev);

	pr_info("%s:%d bitflags: 0x%02lx\n", __func__, __LINE__, chip->bitflags);

	return 0;
}

static int max14656_resume(struct device *dev)
{
	struct max14656_chip *chip = dev_get_drvdata(dev);

    pr_info("%s:%d bitflags: 0x%02lx\n", __func__, __LINE__, chip->bitflags);

    return 0;
}

static SIMPLE_DEV_PM_OPS(max14656_pm_ops, max14656_suspend, max14656_resume);
#define MAX14656_PM_OPS (&max14656_pm_ops)

#else

#define MAX14656_PM_OPS NULL

#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id max14656_id[] = {
	{ MAX14656_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, max14656_id);

static const struct of_device_id max14656_match_table[] = {
	{ .compatible = MAX14656_OF_NODE_NAME, },
	{}
};
MODULE_DEVICE_TABLE(of, max14656_match_table);

static struct i2c_driver max14656_i2c_driver = {
	.driver = {
		.name	= MAX14656_NAME,
		.of_match_table = max14656_match_table,
		.pm	= MAX14656_PM_OPS,
	},
	.probe		= max14656_probe,
	.id_table	= max14656_id,
};
module_i2c_driver(max14656_i2c_driver);

MODULE_DESCRIPTION("MAX14656 USB charger detector");
MODULE_LICENSE("GPL v2");

