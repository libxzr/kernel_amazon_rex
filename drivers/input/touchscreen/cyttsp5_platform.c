/*
 * cyttsp5_platform.c
 * Parade TrueTouch(TM) Standard Product V5 Platform Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2015 Parade Technologies
 * Copyright (C) 2013-2015 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at www.paradetech.com <ttdrivers@paradetech.com>
 *
 */

#include "cyttsp5_regs.h"
#include <linux/cyttsp5_platform.h>
#ifdef CONFIG_CPU_FREQ_OVERRIDE_LAB126
#include <linux/busfreq-imx.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
/* FW for Panel ID = 0x00 */
#include "cyttsp5_fw_pid00.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid00 = {
	.img = cyttsp4_img_pid00,
	.size = ARRAY_SIZE(cyttsp4_img_pid00),
	.ver = cyttsp4_ver_pid00,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid00),
	.panel_id = 0x00,
};

/* FW for Panel ID = 0x01 */
#include "cyttsp5_fw_pid01.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid01 = {
	.img = cyttsp4_img_pid01,
	.size = ARRAY_SIZE(cyttsp4_img_pid01),
	.ver = cyttsp4_ver_pid01,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid01),
	.panel_id = 0x01,
};

/* FW for Panel ID not enabled (legacy) */
#include "cyttsp5_fw.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
/* FW for Panel ID not enabled (legacy) */
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
/* TT Config for Panel ID = 0x00 */
#include "cyttsp5_params_pid00.h"
static struct touch_settings cyttsp5_sett_param_regs_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid00),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid00),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid00 = {
	.param_regs = &cyttsp5_sett_param_regs_pid00,
	.param_size = &cyttsp5_sett_param_size_pid00,
	.fw_ver = ttconfig_fw_ver_pid00,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid00),
	.panel_id = 0x00,
};

/* TT Config for Panel ID = 0x01 */
#include "cyttsp5_params_pid01.h"
static struct touch_settings cyttsp5_sett_param_regs_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid01),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid01),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid01 = {
	.param_regs = &cyttsp5_sett_param_regs_pid01,
	.param_size = &cyttsp5_sett_param_size_pid01,
	.fw_ver = ttconfig_fw_ver_pid01,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid01),
	.panel_id = 0x01,
};

/* TT Config for Panel ID not enabled (legacy)*/
#include "cyttsp5_params.h"
static struct touch_settings cyttsp5_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = &cyttsp5_sett_param_regs,
	.param_size = &cyttsp5_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
/* TT Config for Panel ID not enabled (legacy)*/
static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

static struct cyttsp5_touch_firmware *cyttsp5_firmwares[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	&cyttsp5_firmware_pid00,
	&cyttsp5_firmware_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

static struct cyttsp5_touch_config *cyttsp5_ttconfigs[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
	&cyttsp5_ttconfig_pid00,
	&cyttsp5_ttconfig_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data = {
	.fw = &cyttsp5_firmware,
	.ttconfig = &cyttsp5_ttconfig,
	.fws = cyttsp5_firmwares,
	.ttconfigs = cyttsp5_ttconfigs,
	.flags = CY_LOADER_FLAG_NONE,
};

int cyttsp5_xres(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	static struct cyttsp5_core_commands *cmd;
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;
	int wd_enabled = cd->watchdog_enabled;		//WDT state variable

#ifdef CONFIG_CPU_FREQ_OVERRIDE_LAB126
	request_bus_freq(BUS_FREQ_HIGH);
#endif

	cmd = cyttsp5_get_commands();
	if (cmd) {
		cmd->request_stop_wd(dev);
	}

	/* power cycle and reset hardware */
	gpio_set_value(rst_gpio, 0);
	cyttsp5_power(pdata, 0, dev, 0);
	msleep(200);
	cyttsp5_power(pdata, 1, dev, 0);
	msleep(20);
	gpio_set_value(rst_gpio, 1);
	msleep(10);

	if (cmd && wd_enabled) {			//WDT state variable
		cmd->request_start_wd(dev);
	}

#ifdef CONFIG_CPU_FREQ_OVERRIDE_LAB126
	release_bus_freq(BUS_FREQ_HIGH);
#endif

	dev_info(dev,
		"%s: RESET CYTTSP gpio=%d r=%d\n", __func__,
		pdata->rst_gpio, rc);
	return rc;
}

int cyttsp5_init(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;
	struct regulator *vcc_vdd = pdata->vcc_vdd;
	struct regulator *vcc_ana = pdata->vcc_ana;

	if (on) {
		rc = gpio_request(rst_gpio, "cttysp5_rst");
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail request gpio=%d\n", __func__,
				rst_gpio);
				pdata->rst_gpio =  -1;
				pdata->irq_gpio =  -1;
		} else {
			rc = gpio_direction_output(rst_gpio, 0);
			if (rc < 0) {
				pr_err("%s: Fail set output gpio=%d\n",
					__func__, rst_gpio);
				gpio_free(rst_gpio);
			} else {
				rc = gpio_request(irq_gpio, "cttysp5_irq");
				if (rc < 0) {
					dev_err(dev,
						"%s: Fail request gpio=%d\n",
						__func__, irq_gpio);
					gpio_free(rst_gpio);
					pdata->rst_gpio =  -1;
					pdata->irq_gpio =  -1;
				} else {
					gpio_direction_input(irq_gpio);
				}
			}
		}

		// platform power
		//touch_1v8
		if (!vcc_vdd) {
			dev_err(dev,
				"%s: Fail request regulator of vcc_vdd\n", __func__);
			rc = -1;
		} else {
			if (regulator_is_enabled(vcc_vdd)) {
				pr_info("%s disabling vcc_vdd\n", __func__);
				regulator_disable(vcc_vdd);
			}
			//touch_3v3
			if (!vcc_ana) {
				dev_err(dev,
					"%s: Fail request regulator of vcc_ana\n", __func__);
				rc = -1;
			} else {
				if (regulator_is_enabled(vcc_ana)) {
					pr_info("%s disabling vcc_ana\n", __func__);
					regulator_disable(vcc_ana);
				}
			}
		}
	} else {
		if (gpio_is_valid(rst_gpio))
			gpio_free(rst_gpio);
		if (gpio_is_valid(irq_gpio))
			gpio_free(irq_gpio);
		//platform power
		if (vcc_vdd) {
			pr_info("%s put vcc_vdd\n", __func__);
			regulator_put(vcc_vdd);
		}
		if (vcc_ana) {
			pr_info("%s put vcc_ana\n", __func__);
			regulator_put(vcc_ana);
		}
	}

	dev_info(dev, "%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d r=%d\n",
		__func__, rst_gpio, irq_gpio, rc);
	return rc;
}

static int cyttsp5_wakeup(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	struct regulator *vcc_vdd = pdata->vcc_vdd;
	struct regulator *vcc_ana = pdata->vcc_ana;
	int rc = 0;

	if (vcc_vdd && vcc_ana) {
		//touch_3v3
		if (!regulator_is_enabled(vcc_ana)) {
			pr_info("%s enabling vcc_ana\n", __func__);
			rc = regulator_enable(vcc_ana);
		}
		//touch_1v8
		if (!regulator_is_enabled(vcc_vdd)) {
			pr_info("%s enabling vcc_vdd\n", __func__);
			rc = regulator_enable(vcc_vdd);
		}
		msleep(1);
	}

	return rc;
}

static int cyttsp5_sleep(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	struct regulator *vcc_vdd = pdata->vcc_vdd;
	struct regulator *vcc_ana = pdata->vcc_ana;

	if (vcc_vdd && vcc_ana) {
		//touch_1v8
		if (regulator_is_enabled(vcc_vdd)) {
			pr_info("%s disabling vcc_vdd\n", __FUNCTION__);
			regulator_disable(vcc_vdd);
		}
		//touch_3v3
		if (regulator_is_enabled(vcc_ana)) {
			pr_info("%s disabling vcc_ana\n", __FUNCTION__);
			regulator_disable(vcc_ana);
		}
	}

	return 0;
}

int cyttsp5_power(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp5_wakeup(pdata, dev, ignore_irq);

	return cyttsp5_sleep(pdata, dev, ignore_irq);
}

int cyttsp5_irq_stat(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

#ifdef CYTTSP5_DETECT_HW
int cyttsp5_detect(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, cyttsp5_platform_read read)
{
	int retry = 3;
	int rc;
	char buf[1];

	while (retry--) {
		/* Perform reset, wait for 100 ms and perform read */
		dev_vdbg(dev, "%s: Performing a reset\n", __func__);
		pdata->xres(pdata, dev);
		msleep(100);
		rc = read(dev, buf, 1);
		if (!rc)
			return 0;

		dev_vdbg(dev, "%s: Read unsuccessful, try=%d\n",
			__func__, 3 - retry);
	}

	return rc;
}
#endif
