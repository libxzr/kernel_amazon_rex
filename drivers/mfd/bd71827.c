/*
 * @file bd71827.c  --  RoHM BD71827 mfd driver
 * 
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 * @author: cpham2403@gmail.com
 * Copyright 2016.
 */
/* #define DEBUG */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/bd71827.h>
#include <linux/delay.h>

extern int bd71827_get_events_recorder(struct bd71827*);

/** @brief bd71827 irq resource */
static struct resource rtc_resources[] = {
	{
		.start  = BD71827_IRQ_ALARM_12,
		.end    = BD71827_IRQ_ALARM_12,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct resource power_resources[] = {
	// irq# 0
	{
		.start	= BD71827_IRQ_DCIN_03,
		.end	= BD71827_IRQ_DCIN_03,
		.flags	= IORESOURCE_IRQ,
	},
	// irq# 1
	{
		.start	= BD71827_IRQ_BAT_MON_08,
		.end	= BD71827_IRQ_BAT_MON_08,
		.flags	= IORESOURCE_IRQ,
	},
	// irq# 2
	{
		.start	= BD71827_IRQ_TEMPERATURE_11,
		.end	= BD71827_IRQ_TEMPERATURE_11,
		.flags	= IORESOURCE_IRQ,
	}
};

/** @brief bd71827 multi function cells */
static struct mfd_cell bd71827_mfd_cells[] = {
	{
		.name = "bd71827-pmic",
	},
	{
		.name = "bd71827-power",
		.num_resources = ARRAY_SIZE(power_resources),
		.resources = &power_resources[0],
	},
	{
		.name = "bd71827-gpo",
	},
	{
		.name = "bd71827-rtc",
		.num_resources = ARRAY_SIZE(rtc_resources),
		.resources = &rtc_resources[0],
	},
	{
		.name = "bd71827-led",
	},
};

/** @brief bd71827 irqs */
static const struct regmap_irq bd71827_irqs[] = {
	[BD71827_IRQ_BUCK_01] = {
		.mask = BD71827_INT_EN_01_BUCKAST_MASK,
		.reg_offset = 1,
	},
	[BD71827_IRQ_DCIN_02] = {
		.mask = BD71827_INT_EN_02_DCINAST_MASK,
		.reg_offset = 2,
	},
	[BD71827_IRQ_DCIN_03] = {
		.mask = BD71827_INT_EN_03_DCINAST_MASK,
		.reg_offset = 3,
	},
	[BD71827_IRQ_VSYS_04] = {
		.mask = BD71827_INT_EN_04_VSYSAST_MASK,
		.reg_offset = 4,
	},
	[BD71827_IRQ_CHARGE_05] = {
		.mask = BD71827_INT_EN_05_CHGAST_MASK,
		.reg_offset = 5,
	},
	[BD71827_IRQ_BAT_06] = {
		.mask = BD71827_INT_EN_06_BATAST_MASK,
		.reg_offset = 6,
	},
	[BD71827_IRQ_BAT_MON_07] = {
		.mask = BD71827_INT_EN_07_BMONAST_MASK,
		.reg_offset = 7,
	},
	[BD71827_IRQ_BAT_MON_08] = {
		.mask = BD71827_INT_EN_08_BMONAST_MASK,
		.reg_offset = 8,
	},
	[BD71827_IRQ_BAT_MON_09] = {
		.mask = BD71827_INT_EN_09_BMONAST_MASK,
		.reg_offset = 9,
	},
	[BD71827_IRQ_BAT_MON_10] = {
		.mask = BD71827_INT_EN_10_BMONAST_MASK,
		.reg_offset = 10,
	},
	[BD71827_IRQ_TEMPERATURE_11] = {
		.mask = BD71827_INT_EN_11_TMPAST_MASK,
		.reg_offset = 11,
	},
	[BD71827_IRQ_ALARM_12] = {
		.mask = BD71827_INT_EN_12_ALMAST_MASK,
		.reg_offset = 12,
	},
};

/** @brief bd71827 irq chip definition */
static struct regmap_irq_chip bd71827_irq_chip = {
	.name = "bd71827",
	.irqs = bd71827_irqs,
	.num_irqs = ARRAY_SIZE(bd71827_irqs),
	.num_regs = 13,
	.irq_reg_stride = 1,
	.status_base = BD71827_REG_INT_STAT,
	.mask_base = BD71827_REG_INT_EN_01 - 1,
	.mask_invert = true,
	// .ack_base = BD71827_REG_INT_STAT_00,
};

/** @brief bd71827 irq initialize
 *  @param bd71827 bd71827 device to init
 *  @param bdinfo platform init data
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd71827_irq_init(struct bd71827 *bd71827, struct bd71827_board* bdinfo) {
	int irq;
	int ret = 0;

	if (!bdinfo) {
		dev_warn(bd71827->dev, "No interrupt support, no pdata\n");
		return -EINVAL;
	}

	/* read and clear INT_STAT before requesting irq */
	ret = bd71827_get_events_recorder(bd71827);
	if (unlikely(ret)) {
		dev_err(bd71827->dev, "failed to clear INT_STAT [%d]\n", ret);
		return ret;
	}

	/* XXX: INT_13 is not handled in kernel driver, so keep it disabled */
	ret = bd71827_reg_write(bd71827, BD71827_REG_INT_EN_13, 0x0);
	if (unlikely(ret)) {
		dev_err(bd71827->dev, "failed to diable INT_13 [%d]\n", ret);
		return ret;
	}

	/* Request INTB gpio */
	ret = devm_gpio_request_one(bd71827->dev, (unsigned)bdinfo->gpio_intr,
	    GPIOF_IN, "bd71827-intb");

	if (unlikely(ret)) {
        dev_err(bd71827->dev, "failed to request INTB gpio [%d]\n", ret);
       return ret;
    }
	
	dev_info(bd71827->dev, "gpio_intr = %d \n", bdinfo->gpio_intr);
	irq = gpio_to_irq(bdinfo->gpio_intr);

	bd71827->chip_irq = irq;
	dev_info(bd71827->dev, "chip_irq=%d \n", bd71827->chip_irq);
	ret = regmap_add_irq_chip(bd71827->regmap, bd71827->chip_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_LOW, bdinfo->irq_base,
		&bd71827_irq_chip, &bd71827->irq_data);
	if (ret < 0) {
		dev_warn(bd71827->dev, "Failed to add irq_chip %d\n", ret);
	}
	/* Configure wakeup capable */
	device_set_wakeup_capable(bd71827->dev, 1);
	device_set_wakeup_enable(bd71827->dev , 1);
	return ret;
}

/** @brief bd71827 irq initialize
 *  @param bd71827 bd71827 device to init
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd71827_irq_exit(struct bd71827 *bd71827)
{
	if (bd71827->chip_irq > 0)
		regmap_del_irq_chip(bd71827->chip_irq, bd71827->irq_data);
	return 0;
}

/** @brief check whether volatile register 
 *  @param dev kernel device pointer
 *  @param reg register index
 */
static bool is_volatile_reg(struct device *dev, unsigned int reg)
{
	// struct bd71827 *bd71827 = dev_get_drvdata(dev);

	/*
	 * Caching all regulator registers.
	 */
	return true;
}

/** @brief regmap configures */
static const struct regmap_config bd71827_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = is_volatile_reg,
	.max_register = BD71827_MAX_REGISTER - 1,
	.cache_type = REGCACHE_RBTREE,
};

#if defined(BD71827_NONE_DTB)
static struct bd71827_board *bd71827_parse_fix_table(struct i2c_client *client,
						int *chip_id)
{
	struct bd71827_board *board_info;

	*chip_id  = 0;

	board_info = devm_kzalloc(&client->dev, sizeof(*board_info),
			GFP_KERNEL);
	if (!board_info) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	board_info->gpio_intr = 3*32+21 /* gpio3.GPIO[21] */;
	if (!gpio_is_valid(board_info->gpio_intr)) {
		dev_err(&client->dev, "no pmic intr pin available\n");
		goto err_intr;
	}

	board_info->irq_base = -1;

	return board_info;

err_intr:
	devm_kfree(&client->dev, board_info);
	return NULL;
}
#else /* BD71827_NONE_DTB */
#ifdef CONFIG_OF
static struct of_device_id bd71827_of_match[] = {
	{ .compatible = "rohm,bd71827", .data = (void *)0},
	{ },
};
MODULE_DEVICE_TABLE(of, bd71827_of_match);


/** @brief parse device tree data of bd71827
 *  @param client client object provided by system
 *  @param chip_id return chip id back to caller
 *  @return board initialize data
 */
static struct bd71827_board *bd71827_parse_dt(struct i2c_client *client,
						int *chip_id)
{
	struct device_node *np = client->dev.of_node;
	struct bd71827_board *board_info;
	unsigned int prop;
	const struct of_device_id *match;
	int r = 0;

	match = of_match_device(bd71827_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	*chip_id  = (int)match->data;

	board_info = devm_kzalloc(&client->dev, sizeof(*board_info),
			GFP_KERNEL);
	if (!board_info) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	board_info->gpio_intr = of_get_named_gpio(np, "gpio_intr", 0);
	if (!gpio_is_valid(board_info->gpio_intr)) {
		dev_err(&client->dev, "no pmic intr pin available\n");
		goto err_intr;
	}

	r = of_property_read_u32(np, "irq_base", &prop);
	if (!r) {
		board_info->irq_base = prop;
	} else {
		board_info->irq_base = -1;
	}

	return board_info;

err_intr:
	devm_kfree(&client->dev, board_info);
	return NULL;
}
#else
static inline
struct bd71827_board *bd71827_parse_dt(struct i2c_client *client,
					 int *chip_id)
{
	return NULL;
}
#endif
#endif /* BD71827_NONE_DTB */

/** @brief probe bd71827 device
 *  @param i2c client object provided by system
 *  @param id chip id
 *  @retval 0 probe success
 *  @retval negative error number
 */
static int bd71827_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct bd71827 *bd71827;
	struct bd71827_board *pmic_plat_data;
	struct bd71827_board *of_pmic_plat_data = NULL;
	int chip_id = id->driver_data;
	int ret = 0;

	pmic_plat_data = dev_get_platdata(&i2c->dev);

#if defined(BD71827_NONE_DTB)
	if (!pmic_plat_data) {
		pmic_plat_data = bd71827_parse_fix_table(i2c, &chip_id);
		of_pmic_plat_data = pmic_plat_data;
	}
#else
	if (!pmic_plat_data && i2c->dev.of_node) {
		pmic_plat_data = bd71827_parse_dt(i2c, &chip_id);
		of_pmic_plat_data = pmic_plat_data;
	}
#endif
	if (!pmic_plat_data)
		return -EINVAL;

	bd71827 = kzalloc(sizeof(struct bd71827), GFP_KERNEL);
	if (bd71827 == NULL)
		return -ENOMEM;

	bd71827->of_plat_data = of_pmic_plat_data;
	i2c_set_clientdata(i2c, bd71827);
	bd71827->dev = &i2c->dev;
	bd71827->i2c_client = i2c;
	bd71827->id = chip_id;
	mutex_init(&bd71827->io_mutex);

	bd71827->regmap = devm_regmap_init_i2c(i2c, &bd71827_regmap_config);
	if (IS_ERR(bd71827->regmap)) {
		ret = PTR_ERR(bd71827->regmap);
		dev_err(&i2c->dev, "regmap initialization failed: %d\n", ret);
		return ret;
	}

	ret = bd71827_reg_read(bd71827, BD71827_REG_DEVICE);
	if(ret < 0) {
		dev_err(bd71827->dev, "%s(): Read BD71827_REG_DEVICE failed!\n", __func__);
		goto err;
	}
	dev_info(bd71827->dev, "Device ID=0x%X\n", ret);

	bd71827_irq_init(bd71827, of_pmic_plat_data);

	ret = mfd_add_devices(bd71827->dev, -1,
			      bd71827_mfd_cells, ARRAY_SIZE(bd71827_mfd_cells),
			      NULL, 0,
			      regmap_irq_get_domain(bd71827->irq_data));
	if (ret < 0)
		goto err;

	return ret;

err:
	mfd_remove_devices(bd71827->dev);
	kfree(bd71827);
	return ret;
}

/** @brief remove bd71827 device
 *  @param i2c client object provided by system
 *  @return 0
 */
static int bd71827_i2c_remove(struct i2c_client *i2c)
{
	struct bd71827 *bd71827 = i2c_get_clientdata(i2c);

	bd71827_irq_exit(bd71827);
	mfd_remove_devices(bd71827->dev);
	kfree(bd71827);

	return 0;
}
#ifdef CONFIG_PM_SLEEP

/*
 * This function will be used to perform the workarounds suggested by ROHM
 * to be able to suspend and resume the SOC. The suggested sequence
 * requires 5+ I2C writes and must be conducted under PMIC test mode.
 * The sequence goes like this:
 * - Enter the PMIC test mode
 * - workaround 1: adjust the power sequence register
 * - workaround X...: if more
 * - (IMPORTANT) Exit test mode and back to PMIC normal mode.
 * During the duration of this process, all other I2C registers access
 * are invalid hence we need to do this "atomically".
 */
static int bd71827_suspend_resume_testmode_workaround(struct bd71827 *bd71827, int suspend)
{
	struct i2c_client *client = bd71827->i2c_client;
	int ret;
	u8 reg = 0;

	// skip for hibernation mode
	extern int in_falcon(void);
	if ( in_falcon() ) {
		return 0;
	}

	if (unlikely(!client))
		return -EINVAL;

	/*
	 * lock at the adapter level explicitly. If there are any other processes
	 * (including threaded IRQ handlers) who wish to access the i2c, it will
	 * need to wait.
	 * This is a poor man's way to ensure atomicity in executing a
	 * a sequence of i2c operations back to back.
	 */
	i2c_lock_adapter(client->adapter);

	/* Enter PMIC test mode */
	ret = bd71827_enter_test_mode(client);
	if (ret)
		goto out;

	if (suspend)
		ret = bd71827_write_i2c_reg(client, BD71827_TEST_REG_POWER_SEQ_REG,
					    BD71827_TEST_REG_DISABLE_OFF_SEQUENCE);
	else
		ret = bd71827_write_i2c_reg(client, BD71827_TEST_REG_POWER_SEQ_REG,
					    BD71827_TEST_REG_ENABLE_OFF_SEQUENCE);

	/* Exit PMIC test mode, back to normal */
	bd71827_exit_test_mode(client);

out:
	i2c_unlock_adapter(client->adapter);

	/* everything went ok, delay a bit for test setting to take effect */
	if (!ret)
		udelay(2000);

	return ret;
}

/**@brief suspend bd71827 i2c device
 * @param dev rtc device of system
 * @retval 0
 */
static int bd71827_i2c_suspend_late(struct device *dev)
{
	struct bd71827 *bd71827 = dev_get_drvdata(dev);
#if 0
	ext_bd71827_reg_write8(BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_1);
	ext_bd71827_reg_write8(BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_2);
	ext_bd71827_reg_write8(BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_3);
	ext_bd71827_reg_write8(BD71827_TEST_REG_POWER_SEQ_REG,BD71827_TEST_REG_DISABLE_OFF_SEQUENCE);
	ext_bd71827_reg_write8(BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_USER_AREA);
	msleep(1);
#endif
	bd71827_suspend_resume_testmode_workaround(bd71827, 1);
	/* keep 3.3V */
#if 0
	ext_bd71827_reg_write8(BD71827_REG_LDO1_VOLT, 0x14); /* 1.8V */
#endif
	enable_irq_wake(bd71827->chip_irq);
	return 0;
}

/**@brief resume bd71827 i2c device
 * @param dev rtc device of system
 * @retval 0
 */
static int bd71827_i2c_resume_early(struct device *dev)
{
	struct bd71827 *bd71827 = dev_get_drvdata(dev);
#if 0
	ext_bd71827_reg_write8(BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_1);
	ext_bd71827_reg_write8(BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_2);
	ext_bd71827_reg_write8(BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_AREA_3);
	ext_bd71827_reg_write8(BD71827_TEST_REG_POWER_SEQ_REG,BD71827_TEST_REG_ENABLE_OFF_SEQUENCE);
	ext_bd71827_reg_write8(BD71827_REG_I2C_MAGIC, BD71827_TEST_REG_ACCESS_USER_AREA);
	msleep(1);
#endif
	bd71827_suspend_resume_testmode_workaround(bd71827, 0);
	/* keep 3.3V */
#if 0
	ext_bd71827_reg_write8(BD71827_REG_LDO1_VOLT, 0x30); /* 3.2V */
#endif
	disable_irq_wake(bd71827->chip_irq);

	return 0;
}
#endif
//static SIMPLE_DEV_PM_OPS(bd71827_i2c_pm_ops, bd71827_i2c_suspend, bd71827_i2c_resume);
static struct dev_pm_ops bd71827_i2c_pm_ops = {
        .suspend_late = bd71827_i2c_suspend_late,
        .resume_early = bd71827_i2c_resume_early,
};
static const struct i2c_device_id bd71827_i2c_id[] = {
	{ "bd71827", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bd71827_i2c_id);

static struct i2c_driver bd71827_i2c_driver = {
	.driver = {
		.name = "bd71827",
		.owner = THIS_MODULE,
		.pm = &bd71827_i2c_pm_ops,
#if !defined(BD71827_NONE_DTB)
		.of_match_table = of_match_ptr(bd71827_of_match),
#endif
	},
	.probe = bd71827_i2c_probe,
	.remove = bd71827_i2c_remove,
	.id_table = bd71827_i2c_id,
};

static int __init bd71827_i2c_init(void)
{
	return i2c_add_driver(&bd71827_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(bd71827_i2c_init);

static void __exit bd71827_i2c_exit(void)
{
	i2c_del_driver(&bd71827_i2c_driver);
}
module_exit(bd71827_i2c_exit);

MODULE_AUTHOR("Cong Pham <cpham2403@gmail.com>");
MODULE_DESCRIPTION("BD71827 chip multi-function driver");
MODULE_LICENSE("GPL");
