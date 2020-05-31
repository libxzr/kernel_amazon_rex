/*
 * @file bd71827-regulator.c RoHM BD71827 regulator driver
 *
 * @author: cpham2403@gmail.com
 * Copyright 2016.
 *
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */
#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mfd/bd71827.h>
#include <linux/regulator/of_regulator.h>

#define BD71827_VOL_OFFSET			0
#define BD71827_STANDBY_OFFSET		0
//#define BD7181X_BUCK1_09V_SUSPEND   0x04
//#define BD7181X_WDT_MANUAL          0x2F
#define BD71827_DVS_BUCK_NUM		2
#define BD71827_DVS_RUN_IDLE_SUSP	3

struct bd71827_buck_dvs {
	u32 voltage[BD71827_DVS_RUN_IDLE_SUSP];
};

/** @brief bd71827 regulator type */
struct bd71827_pmic {
	struct regulator_desc descs[BD71827_REGULATOR_CNT];	/**< regulator description to system */
	struct bd71827 *mfd;									/**< parent device */
	struct device *dev;										/**< regulator kernel device */
	struct regulator_dev *rdev[BD71827_REGULATOR_CNT];		/**< regulator device of system */
	struct bd71827_buck_dvs buck_dvs[BD71827_DVS_BUCK_NUM];			/**< buck1/2 dvs */
};

/*
 * BUCK1/2
 * BUCK1RAMPRATE[1:0] BUCK1 DVS ramp rate setting
 * 00: 10.00mV/usec 10mV 1uS
 * 01: 5.00mV/usec	10mV 2uS
 * 10: 2.50mV/usec	10mV 4uS
 * 11: 1.25mV/usec	10mV 8uS
 */
static int bd71827_buck12_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	struct bd71827_pmic *pmic = rdev_get_drvdata(rdev);
	struct bd71827 *mfd = pmic->mfd;
	int id = rdev->desc->id;
	unsigned int ramp_value = BUCK1_RAMPRATE_10P00MV;

	//dev_dbg(pmic->dev, "Buck[%d] Set Ramp = %d\n", id + 1, ramp_delay);
	switch (ramp_delay) {
	case 1 ... 1250:
		ramp_value = BUCK1_RAMPRATE_1P25MV;
		break;
	case 1251 ... 2500:
		ramp_value = BUCK1_RAMPRATE_2P50MV;
		break;
	case 2501 ... 5000:
		ramp_value = BUCK1_RAMPRATE_5P00MV;
		break;
	case 5001 ... 10000:
		ramp_value = BUCK1_RAMPRATE_10P00MV;
		break;
	default:
		ramp_value = BUCK1_RAMPRATE_10P00MV;
		dev_err(pmic->dev, "%s: ramp_delay: %d not supported, setting 10000mV//us\n",
			rdev->desc->name, ramp_delay);
	}

	return bd71827_update_bits(mfd, BD71827_REG_BUCK1_MODE + id*0x1,
			BUCK1_RAMPRATE_MASK, ramp_value << 6);
}

static int bd71827_buck12_get_voltage_sel(struct regulator_dev *rdev)
{
	struct bd71827_pmic *pmic = rdev_get_drvdata(rdev);
	int rid = rdev_get_id(rdev);
	struct bd71827 *bd71827 = pmic->mfd;
	int ret;
	u8 reg = BD71827_REG_BUCK1_VOLT_RUN + rid*0x2;

	ret = bd71827_reg_read(bd71827, reg);
	if (ret < 0) {
		return ret;
	}

	//dev_dbg(pmic->dev, "Buck[%d] Get Vol = %d\n", rid + 1, ret & BUCK1_RUN_MASK);

	return (ret & BUCK1_RUN_MASK);
}

/*
 * For Buck 1/2.
 *
 */
static int bd71827_buck12_set_voltage_sel(struct regulator_dev *rdev, unsigned sel)
{
	struct bd71827_pmic *pmic = rdev_get_drvdata(rdev);
	int rid = rdev_get_id(rdev);
	struct bd71827 *bd71827 = pmic->mfd;
	int ret;
	u8 reg = BD71827_REG_BUCK1_VOLT_RUN + rid*0x2;

	//dev_dbg(pmic->dev, "Buck[%d] Set Vol Sel = %d\n", rid + 1, sel);

	ret = bd71827_reg_write(bd71827, reg, sel & BUCK1_RUN_MASK);

	return ret;
}

static struct regulator_ops bd71827_ldo_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static struct regulator_ops bd71827_fixed_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
};

static struct regulator_ops bd71827_buck_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

static struct regulator_ops bd71827_buck12_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = bd71827_buck12_set_voltage_sel,
	.get_voltage_sel = bd71827_buck12_get_voltage_sel,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay = bd71827_buck12_set_ramp_delay,
};

/*
 * BUCK1 PVIN1 1.3V 1000mA
 * 0.8 to 2.0V (25mV step) [DVS]
 */
static const struct regulator_linear_range bd71827_buck1_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000,  0x00, 0x2F, 25000),
	REGULATOR_LINEAR_RANGE(2000000, 0x30, 0x3F, 0),
};

/*
 * BUCK2 PVIN2 1.3V 1000mA
 * 0.8 to 2.0V (25mV step) [DVS]
 */
static const struct regulator_linear_range bd71827_buck2_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000,  0x00, 0x2F, 25000),
	REGULATOR_LINEAR_RANGE(2000000, 0x30, 0x3F, 0),
};

/*
 * BUCK3 PVIN3 1.8V 500mA
 * 1.2V to 2.0V (50mV step)
 */
static const struct regulator_linear_range bd71827_buck3_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1200000, 0x00, 0x0F, 50000),
	REGULATOR_LINEAR_RANGE(2000000, 0x10, 0x3F, 0),
};

/*
 * BUCK4 PVIN4 1.2V 1000mA
 * 1.1V to 1.8V (25mV step)
 */
static const struct regulator_linear_range bd71827_buck4_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1100000, 0x00, 0x1B, 25000),
	REGULATOR_LINEAR_RANGE(1800000, 0x1C, 0x1F, 0),
};

/*
 * BUCK5 PVIN5 3.2V 1000mA
 * 2.5V to 3.3V (50mV step)
 */
static const struct regulator_linear_range bd71827_buck5_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(2500000, 0x00, 0x0F, 50000),
	REGULATOR_LINEAR_RANGE(3300000, 0x10, 0x1F, 0),
};

/*
 * LDO1 VINL1 1.8V 100mA
 * 0.8 to 3.3V (50mV step)
 */
static const struct regulator_linear_range bd71827_ldo1_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000,  0x00, 0x31, 50000),
	REGULATOR_LINEAR_RANGE(3300000, 0x32, 0x3F, 0),
};

/*
 * LDO2 VINL1 3.2V 100mA
 * 0.8 to 3.3V (50mV step)
 */
static const struct regulator_linear_range bd71827_ldo2_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000,  0x00, 0x31, 50000),
	REGULATOR_LINEAR_RANGE(3300000, 0x32, 0x3F, 0),
};


/*
 * LDO3 VINL1 3.0V 50mA
 * 0.8 to 3.3V (50mV step)
 */
static const struct regulator_linear_range bd71827_ldo3_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000,  0x00, 0x31, 50000),
	REGULATOR_LINEAR_RANGE(3300000, 0x32, 0x3F, 0),
};

/*
 * LDO4 VINL2 3.3V 400mA
 * 0.8 to 3.3V (50mV step)
 */
static const struct regulator_linear_range bd71827_ldo4_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000,  0x00, 0x31, 50000),
	REGULATOR_LINEAR_RANGE(3300000, 0x32, 0x3F, 0),
};

/*
 * LDO5 VINL2 1.8V 250mA
 * 0.8 to 3.3V (50mV step)
 */
static const struct regulator_linear_range bd71827_ldo5_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000,  0x00, 0x31, 50000),
	REGULATOR_LINEAR_RANGE(3300000, 0x32, 0x3F, 0),
};

static const struct regulator_desc bd71827_regulators[] = {
	{
		.name = "BUCK1",
		.id = BD71827_BUCK1,
		.ops = &bd71827_buck12_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_BUCK1_VOLTAGE_NUM,
		.linear_ranges = bd71827_buck1_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71827_buck1_voltage_ranges),
		.vsel_reg = BD71827_REG_BUCK1_VOLT_RUN,
		.vsel_mask = BUCK1_RUN_MASK,
		.enable_reg = BD71827_REG_BUCK1_MODE,
		.enable_mask = BUCK1_RUN_ON,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK2",
		.id = BD71827_BUCK2,
		.ops = &bd71827_buck12_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_BUCK2_VOLTAGE_NUM,
		.linear_ranges = bd71827_buck2_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71827_buck2_voltage_ranges),
		.vsel_reg = BD71827_REG_BUCK2_VOLT_RUN,
		.vsel_mask = BUCK2_RUN_MASK,
		.enable_reg = BD71827_REG_BUCK2_MODE,
		.enable_mask = BUCK2_RUN_ON,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK3",
		.id = BD71827_BUCK3,
		.ops = &bd71827_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_BUCK3_VOLTAGE_NUM,
		.linear_ranges = bd71827_buck3_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71827_buck3_voltage_ranges),
		.vsel_reg = BD71827_REG_BUCK3_VOLT,
		.vsel_mask = BUCK3_MASK,
		.enable_reg = BD71827_REG_BUCK3_MODE,
		.enable_mask = BUCK3_RUN_ON,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK4",
		.id = BD71827_BUCK4,
		.ops = &bd71827_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_BUCK4_VOLTAGE_NUM,
		.linear_ranges = bd71827_buck4_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71827_buck4_voltage_ranges),
		.vsel_reg = BD71827_REG_BUCK4_VOLT,
		.vsel_mask = BUCK4_MASK,
		.enable_reg = BD71827_REG_BUCK4_MODE,
		.enable_mask = BUCK4_RUN_ON,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK5",
		.id = BD71827_BUCK5,
		.ops = &bd71827_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_BUCK5_VOLTAGE_NUM,
		.linear_ranges = bd71827_buck5_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71827_buck5_voltage_ranges),
		.vsel_reg = BD71827_REG_BUCK5_VOLT,
		.vsel_mask = BUCK5_MASK,
		.enable_reg = BD71827_REG_BUCK5_MODE,
		.enable_mask = BUCK5_RUN_ON,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO1",
		.id = BD71827_LDO1,
		.ops = &bd71827_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_LDO1_VOLTAGE_NUM,
		.linear_ranges = bd71827_ldo1_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71827_ldo1_voltage_ranges),
		.vsel_reg = BD71827_REG_LDO1_VOLT,
		.vsel_mask = LDO1_MASK,
		.enable_reg = BD71827_REG_LDO_MODE1,
		.enable_mask = LDO1_RUN_ON,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO2",
		.id = BD71827_LDO2,
		.ops = &bd71827_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_LDO2_VOLTAGE_NUM,
		.linear_ranges = bd71827_ldo2_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71827_ldo2_voltage_ranges),
		.vsel_reg = BD71827_REG_LDO2_VOLT,
		.vsel_mask = LDO2_MASK,
		.enable_reg = BD71827_REG_LDO_MODE2,
		.enable_mask = LDO2_RUN_ON,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO3",
		.id = BD71827_LDO3,
		.ops = &bd71827_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_LDO3_VOLTAGE_NUM,
		.linear_ranges = bd71827_ldo3_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71827_ldo3_voltage_ranges),
		.vsel_reg = BD71827_REG_LDO3_VOLT,
		.vsel_mask = LDO3_MASK,
		.enable_reg = BD71827_REG_LDO_MODE2,
		.enable_mask = LDO3_RUN_ON,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO4",
		.id = BD71827_LDO4,
		.ops = &bd71827_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_LDO4_VOLTAGE_NUM,
		.linear_ranges = bd71827_ldo4_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71827_ldo4_voltage_ranges),
		.vsel_reg = BD71827_REG_LDO4_VOLT,
		.vsel_mask = LDO4_MASK,
		.enable_reg = BD71827_REG_LDO_MODE3,
		.enable_mask = LDO4_RUN_ON,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO5",
		.id = BD71827_LDO5,
		.ops = &bd71827_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_LDO5_VOLTAGE_NUM,
		.linear_ranges = bd71827_ldo5_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(bd71827_ldo5_voltage_ranges),
		.vsel_reg = BD71827_REG_LDO5_VOLT,
		.vsel_mask = LDO5_MASK,
		.enable_reg = BD71827_REG_LDO_MODE3,
		.enable_mask = LDO5_RUN_ON,
		.owner = THIS_MODULE,
	},
	/*
	 * LDO6 VIN 1.8V 100mA
	 * Fixed voltage
	 */
	{
		.name = "LDO6",
		.id = BD71827_LDO6,
		.ops = &bd71827_fixed_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_LDO6_VOLTAGE_NUM,
		.min_uV = 1800000,
		.enable_reg = BD71827_REG_LDO_MODE4,
		.enable_mask = LDO6_RUN_ON,
		.owner = THIS_MODULE,
	},
	/*
	 * VOSNVS VIN 3.0V 10mA
	 * Fixed voltage
	 */
	{
		.name = "LDOSNVS",
		.id = BD71827_LDOSNVS,
		.ops = &bd71827_fixed_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = BD71827_LDOSNVS_VOLTAGE_NUM,
		.min_uV = 3000000,
		.enable_reg = BD71827_REG_LDO_MODE4,
		.enable_mask = SNVS_RUN_ON,
		.owner = THIS_MODULE,
	},
};

#ifdef CONFIG_OF

static struct of_regulator_match bd71827_matches[] = {
	{ .name = "buck1",	},
	{ .name = "buck2",	},
	{ .name = "buck3",	},
	{ .name = "buck4",	},
	{ .name = "buck5",	},
	{ .name = "ldo1",	},
	{ .name = "ldo2",	},
	{ .name = "ldo3",	},
	{ .name = "ldo4",	},
	{ .name = "ldo5",	},
	{ .name = "ldo6",	},
	{ .name = "ldosnvs",	},
};

/**@brief parse bd71827 regulator device tree
 * @param pdev platform device of bd71827 regulator
 * @param bd71827_reg_matches return regualtor matches
 * @retval 0 parse success
 * @retval NULL parse fail
 */
static int bd71827_parse_dt_reg_data(
		struct platform_device *pdev,
		struct of_regulator_match **reg_matches)
{
	// struct bd71827 *bd71827 = dev_get_drvdata(pdev->dev.parent);
	struct device_node *np, *regulators;
	struct of_regulator_match *matches;
	int ret, count;

	np = of_node_get(pdev->dev.parent->of_node);
	regulators = of_find_node_by_name(np, "regulators");
	if (!regulators) {
		dev_err(&pdev->dev, "regulator node not found\n");
		return -EINVAL;
	}

	count = ARRAY_SIZE(bd71827_matches);
	matches = bd71827_matches;

	ret = of_regulator_match(&pdev->dev, regulators, matches, count);
	of_node_put(regulators);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error parsing regulator init data: %d\n",
			ret);
		return ret;
	}

	*reg_matches = matches;

	return 0;
}
#else
static inline int bd71827_parse_dt_reg_data(
			struct platform_device *pdev,
			struct of_regulator_match **reg_matches)
{
	*reg_matches = NULL;
	return 0;
}
#endif

/** @brief out32k mode constants */
static const char* out32k_modes[] = {"open_drain", "cmos"};

/** @brief retrive out32k output mode */
static ssize_t show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bd71827_pmic *pmic = dev_get_drvdata(dev);
	int o;

	o = bd71827_reg_read(pmic->mfd, BD71827_REG_OUT32K);
	o = (o & OUT32K_MODE) != 0;

	return sprintf(buf, "%s\n", out32k_modes[o]);
}

/** @brief set out32k output mode */
static ssize_t set_mode(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bd71827_pmic *pmic = dev_get_drvdata(dev);
	int o, r;

	if (strncmp(buf, out32k_modes[0], strlen(out32k_modes[0])) == 0) {
		o = 0;
	} else {
		o = OUT32K_MODE;
	}

	r = bd71827_update_bits(pmic->mfd, BD71827_REG_OUT32K, OUT32K_MODE, o);
	if (r < 0) {
		return r;
	}
	return count;
}

/** @brief retrive out32k output value */
static ssize_t show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bd71827_pmic *pmic = dev_get_drvdata(dev);
	int o;

	o = bd71827_reg_read(pmic->mfd, BD71827_REG_OUT32K);
	o = (o & OUT32K_EN) != 0;

	return sprintf(buf, "%d\n", o);
}

/** @brief set o output value */
static ssize_t set_value(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct bd71827_pmic *pmic = dev_get_drvdata(dev);
	int o, r;

	if (sscanf(buf, "%d", &o) < 1) {
		return -EINVAL;
	}

	if (o != 0) {
		o = OUT32K_EN;
	}
	r = bd71827_update_bits(pmic->mfd, BD71827_REG_OUT32K, OUT32K_EN, o);
	if (r < 0) {
		return r;
	}
	return count;
}

/** @brief list all supported modes */
static ssize_t available_modes(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, r;

	r = 0;
	for (i = 0; i < ARRAY_SIZE(out32k_modes) && r >= 0; i++) {
		r += sprintf(buf + r, "%s ", out32k_modes[i]);
	}
	r += sprintf(buf + r, "\n");

	return r;
}

/** @brief list all supported values */
static ssize_t available_values(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0 1 \n");
}

static DEVICE_ATTR(out32k_mode, S_IWUSR | S_IRUGO, show_mode, set_mode);
static DEVICE_ATTR(out32k_value, S_IWUSR | S_IRUGO, show_value, set_value);
static DEVICE_ATTR(available_mode, S_IWUSR | S_IRUGO, available_modes, NULL);
static DEVICE_ATTR(available_value, S_IWUSR | S_IRUGO, available_values, NULL);

/** @brief device sysfs attribute table, about o */
static struct attribute *clk_attributes[] = {
	&dev_attr_out32k_mode.attr,
	&dev_attr_out32k_value.attr,
	&dev_attr_available_mode.attr,
	&dev_attr_available_value.attr,
	NULL
};

static const struct attribute_group clk_attr_group = {
	.attrs	= clk_attributes,
};

/*----------------------------------------------------------------------*/
#if defined(BD71827_NONE_DTB)
static void bd71827_buck_dvs_fix_table(struct platform_device *pdev, struct bd71827_buck_dvs *buck_dvs)
{
	/* VDD_ARM: Run-Idle-Suspend */
	buck_dvs[0].voltage[0] = 1100000;
	buck_dvs[0].voltage[1] = 900000;
	buck_dvs[0].voltage[2] = 900000;
	/* VDD_SOC: Run-Idle-Suspend */
	buck_dvs[1].voltage[0] = 1100000;
	buck_dvs[1].voltage[1] = 900000;
	buck_dvs[1].voltage[2] = 900000;
}
#else
#ifdef CONFIG_OF
/** @brief buck1/2 dvs enable/voltage from device tree
 * @param pdev platfrom device pointer
 * @param buck_dvs pointer
 * @return void
 */
static void of_bd71827_buck_dvs(struct platform_device *pdev, struct bd71827_buck_dvs *buck_dvs)
{
	struct device_node *pmic_np;

	pmic_np = of_node_get(pdev->dev.parent->of_node);
	if (!pmic_np) {
		dev_err(&pdev->dev, "could not find pmic sub-node\n");
		return;
	}

	if (of_get_property(pmic_np, "bd71827,pmic-buck1-uses-i2c-dvs", NULL)) {
		if (of_property_read_u32_array(pmic_np,
							"bd71827,pmic-buck1-dvs-voltage",
							&buck_dvs[0].voltage[0], BD71827_DVS_RUN_IDLE_SUSP)) {
			dev_err(&pdev->dev, "buck1 voltages not specified\n");
		}
	}

	if (of_get_property(pmic_np, "bd71827,pmic-buck2-uses-i2c-dvs", NULL)) {
		if (of_property_read_u32_array(pmic_np,
							"bd71827,pmic-buck2-dvs-voltage",
						&buck_dvs[1].voltage[0], BD71827_DVS_RUN_IDLE_SUSP)) {
			dev_err(&pdev->dev, "buck2 voltages not specified\n");
		}
	}
}
#else
static void of_bd71827_buck_dvs(struct platform_device *pdev, struct bd71827_buck_dvs *buck_dvs)
{
	buck_dvs[0].voltage[0] = BUCK1_RUN_DEFAULT;
	buck_dvs[0].voltage[1] = BUCK1_IDLE_DEFAULT;
	buck_dvs[0].voltage[2] = BUCK1_SUSP_DEFAULT;
	buck_dvs[1].voltage[0] = BUCK2_RUN_DEFAULT;
	buck_dvs[1].voltage[1] = BUCK2_IDLE_DEFAULT;
	buck_dvs[1].voltage[2] = BUCK2_SUSP_DEFAULT;
}
#endif
#endif

static int bd71827_buck12_dvs_init(struct bd71827_pmic *pmic)
{
	struct bd71827 *bd71827 = pmic->mfd;
	struct bd71827_buck_dvs *buck_dvs = &pmic->buck_dvs[0];
	int i, ret, val, selector = 0;
	u8 reg_run, reg_idle, reg_susp;

	for(i = 0; i < BD71827_DVS_BUCK_NUM; i++, buck_dvs++) {
		reg_run = BD71827_REG_BUCK1_VOLT_RUN + i*0x2;
		reg_idle = BD71827_REG_BUCK1_VOLT_IDLE + i*0x1;
		reg_susp = BD71827_REG_BUCK1_VOLT_SUSP + i*0x2;
		dev_info(pmic->dev, "Buck%d: DVS Run-Idle-Susp[%d - %d - %d].\n", i, buck_dvs->voltage[0], buck_dvs->voltage[1], buck_dvs->voltage[2]);
		selector = regulator_map_voltage_iterate(pmic->rdev[i], buck_dvs->voltage[0], buck_dvs->voltage[0]);
		if(selector < 0) {
			dev_err(pmic->dev, "%s(): not found selector for Run voltage [%d]\n", __func__, buck_dvs->voltage[0]);
		} else {
			val = (selector & BUCK1_RUN_MASK);
			ret = bd71827_reg_write(bd71827, reg_run, val);
			if(ret < 0)
				return ret;
		}
		selector = regulator_map_voltage_iterate(pmic->rdev[i], buck_dvs->voltage[1], buck_dvs->voltage[1]);
		if(selector < 0) {
			dev_err(pmic->dev, "%s(): not found selector for Idle voltage [%d]\n", __func__, buck_dvs->voltage[1]);
		} else {
			val = (selector & BUCK1_IDLE_MASK);
			ret = bd71827_reg_write(bd71827, reg_idle, val);
			if(ret < 0)
				return ret;
		}
		selector = regulator_map_voltage_iterate(pmic->rdev[i], buck_dvs->voltage[2], buck_dvs->voltage[2]);
		if(selector < 0) {
			dev_err(pmic->dev, "%s(): not found selector for Susp voltage [%d]\n", __func__, buck_dvs->voltage[2]);
		} else {
			val = (selector & BUCK1_SUSP_MASK);
			ret = bd71827_reg_write(bd71827, reg_susp, val);
			if(ret < 0)
				return ret;
		}
	}
	return 0;
}

/**@brief probe bd71827 regulator device
 @param pdev bd71827 regulator platform device
 @retval 0 success
 @retval negative fail
*/
static __init int bd71827_probe(struct platform_device *pdev)
{
	struct bd71827_pmic *pmic;
	struct bd71827_board *pdata;
	struct regulator_config config = {};
	struct bd71827 *bd71827 = dev_get_drvdata(pdev->dev.parent);
	struct of_regulator_match *matches = NULL;
	int i, err;

	pmic = kzalloc(sizeof(*pmic), GFP_KERNEL);
	if (!pmic) {
		dev_err(&pdev->dev, "Memory allocation failed for pmic\n");
		return -ENOMEM;
	}

	memcpy(pmic->descs, bd71827_regulators,	sizeof(pmic->descs));

	pmic->dev = &pdev->dev;
	pmic->mfd = bd71827;
	platform_set_drvdata(pdev, pmic);

	bd71827_reg_write(pmic->mfd, BD71827_REG_BUCK1_VOLT_SUSP, BUCK1_SUSP_DEFAULT); // Set BUCK_1 suspend voltage to 0.9V
	bd71827_reg_write(pmic->mfd, BD71827_REG_CHG_SET1, WDT_MANUAL); // Set WDT_AUTO to 0 so we can have manual control
	bd71827_clear_bits(pmic->mfd, BD71827_REG_PWRCTRL, RESTARTEN); // Disable to go to ship-mode
	bd71827_clear_bits(pmic->mfd, BD71827_REG_GPIO, RESTARTEN); // Turn OFF the green LED
	bd71827_set_bits(pmic->mfd, BD71827_REG_CHG_SET1, CHG_EN); // Enable charger

	pdata = dev_get_platdata(bd71827->dev);

	if (!pdata && bd71827->dev->of_node) {
		bd71827_parse_dt_reg_data(pdev,	&matches);
		if (matches == NULL) {
			dev_err(&pdev->dev, "Platform data not found\n");
			return -EINVAL;
		}
	}

	/* Get buck dvs parameters */
#if defined(BD71827_NONE_DTB)
	bd71827_buck_dvs_fix_table(pdev, &pmic->buck_dvs[0]);
#else
	of_bd71827_buck_dvs(pdev, &pmic->buck_dvs[0]);
#endif

	for (i = 0; i < BD71827_REGULATOR_CNT; i++) {
		struct regulator_init_data *init_data;
		struct regulator_desc *desc;
		struct regulator_dev *rdev;

		desc = &pmic->descs[i];
		desc->name = bd71827_matches[i].name;
		
		if (pdata) {
			init_data = pdata->init_data[i];
		} else {
			init_data = matches[i].init_data;
		}

		config.dev = pmic->dev;
		config.init_data = init_data;
		config.driver_data = pmic;
		config.regmap = bd71827->regmap;
		config.of_node = matches[i].of_node;
		dev_info(config.dev, "regulator register name '%s'\n", desc->name);

		rdev = regulator_register(desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(bd71827->dev,
				"failed to register %s regulator\n",
				desc->name);
			err = PTR_ERR(rdev);
			goto err;
		}
		pmic->rdev[i] = rdev;
	}

	err = sysfs_create_group(&pdev->dev.kobj, &clk_attr_group);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to create attribute group: %d\n", err);
		goto err;
	}

	/* Init buck12 dvs */
	err = bd71827_buck12_dvs_init(pmic);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to buck12 dvs: %d\n", err);
		goto err;
	}

	/*
	 * LDO3 is turned on when DCIN is supplied with default setting (LDO3_REG_MODE=0). LDO3 is not controlled by I2C
	 * registers shown in Table 11 with default setting. When LDO3_REG_MODE is set 1, LDO3 is controlled by I2C registers
	 * same as other power rails.
	 *
	 * --> Default use register to control LDO3
	 */
	err = bd71827_update_bits(pmic->mfd, BD71827_REG_LDO_MODE1, LDO3_REG_MODE, LDO3_REG_MODE);
	if (err < 0) {
		return err;
	}
	/*
	 * LDO4 can be controlled by GPIO1 terminal when LDO4_REG_MODE is set 0. With this settings, LDO4 is not
	 * controlled by I2C registers. And LDO4 is turned on when GPIO1 set to H.
	 *
	 * --> Default use register to control LDO4
	 *
	 */
	err = bd71827_update_bits(pmic->mfd, BD71827_REG_LDO_MODE1, LDO4_REG_MODE, LDO4_REG_MODE);
	if (err < 0) {
		return err;
	}

	return 0;

err:
	while (--i >= 0)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return err;
}

/**@brief remove bd71827 regulator device
 @param pdev bd71827 regulator platform device
 @return 0
*/
static int __exit bd71827_remove(struct platform_device *pdev)
{
	struct bd71827_pmic *pmic = platform_get_drvdata(pdev);
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &clk_attr_group);

	for (i = 0; i < BD71827_REGULATOR_CNT; i++)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return 0;
}

static struct platform_driver bd71827_driver = {
	.driver = {
		.name = "bd71827-pmic",
		.owner = THIS_MODULE,
	},
	.probe = bd71827_probe,
	.remove = bd71827_remove,
};

/**@brief module initialize function */
static int __init bd71827_init(void)
{
	return platform_driver_register(&bd71827_driver);
}
subsys_initcall(bd71827_init);

/**@brief module deinitialize function */
static void __exit bd71827_cleanup(void)
{
	platform_driver_unregister(&bd71827_driver);
}
module_exit(bd71827_cleanup);

MODULE_AUTHOR("Cong Pham <cpham2403@gmail.com>");
MODULE_DESCRIPTION("BD71827 voltage regulator driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bd71827-pmic");
