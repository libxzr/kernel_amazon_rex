 /*
 * mwan_rex.c  --  Mario WAN hardware control driver for REX
 *
 * Copyright 2005-2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <net/mwan.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/busfreq-imx.h>
#include <linux/mutex.h>

#define ENABLE_FW_READY_IRQ
#define ENABLE_SOC_WAKEUP_IRQ
#define ENABLE_SMS_READY_IRQ
//#define ENABLE_DPDT_SNIFFER_IRQ

//#define DEBUG

#ifdef DEBUG
#define log_debug(format, arg...) printk("mwan: D %s:" format, __func__, ## arg)
#else
#define log_debug(format, arg...) do{}while(0)
#endif

#define log_info(format, arg...)  printk("mwan: I %s:" format, __func__, ## arg)
#define log_err(format, arg...)   printk("mwan: E %s:" format, __func__, ## arg)


// standard network deregistration time definition:
//   2s -- maximum time required between the start of power down and that of IMSI detach
//   5s -- maximum time required for the IMSI detach
//   5s -- maximum time required between the IMSI detach and power down finish (time
//         required to stop tasks, etc.)
//   3s -- recommended safety margin
#define NETWORK_DEREG_TIME	((12 + 3) * 1000)

// minimum allowed delay between TPH notifications
// set this lower but don't remove completely just in case we get spurious GPIO transitions on boot
#define WAKE_EVENT_INTERVAL	6

/*
 * Small snippet taken out of the implementation of msecs_to_jiffies
 * to optimize a little bit.
 * HZ is equal to or smaller than 1000, and 1000 is a nice
 * round multiple of HZ, divide with the factor between them,
 * but round upwards:
 */
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
#define MS_TO_JIFFIES(X)	((unsigned long)((X) + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ))
#else
#define MS_TO_JIFFIES(X)	msecs_to_jiffies(X)
#endif

#define MSEC_2_JIFFIES(X)	(__builtin_constant_p(X) ? MS_TO_JIFFIES(X) : msecs_to_jiffies(X))


#define rex_PWROFF_HOLD_TIME_MS            500
#define rex_PWROFF_DISCHARGE_TIME_MSEC     1000
#define rex_PWRON_TIME_SEC                 20
#define rex_PWROFF_TIME_SEC                7
#ifdef ENABLE_FW_READY_IRQ
#define rex_FW_READY_DBOUNCE_TIME_MSEC     30
#endif
#ifdef ENABLE_SMS_READY_IRQ
#define rex_SMS_READY_DBOUNCE_TIME_MSEC    30
#endif
#ifdef ENABLE_SOC_WAKEUP_IRQ
#define rex_USB_WAKEUP_DONE_TIME_MSEC      500
#define rex_USB_RESUME_RETRY_TIME_MSEC     60
#endif
#ifdef ENABLE_DPDT_SNIFFER_IRQ
#define rex_DPDT_SNIFFER_DBOUNCE_TIME_MSEC 5
#endif


#define VERSION               "6.1.4"

#define WAN_STRING_CLASS      "wan"
#define WAN_STRING_DEV        "mwan"

static struct file_operations mwan_ops = {
	.owner = THIS_MODULE,
};


#define PROC_WAN              "wan"
#define PROC_WAN_POWER        "power"
#define PROC_WAN_TYPE         "type"
#define PROC_WAN_SAR          "sar_detect"
#define PROC_WAN_FWRDY        "fw_ready"
#define PROC_WAN_USB_EN       "usb_en"
#define PROC_WAN_DPDT         "dpdt"
#define PROC_WAN_SPST         "spst"
#define PROC_WAN_DISABLE      "disable"
#define PROC_WAN_DPDT_SNIFFER "dpdt_sniffer"


enum mwan_type {
	MWAN_COMPATIBLE,           /* Determine device type base on the board id */
	MWAN_REX_V2,               /* REX V2, BANFF ES 2.0 */
	MWAN_MOONSHINE_PRE_PROTO,  /* Moonshine Pre-Proto, BANFF ES 2.0 */
	MWAN_MOONSHINE_PROTO,      /* Moonshine Proto, BANFF ES 2.2 */
	MWAN_MOONSHINE,            /* Moonshine Pre-HVT and beyond, BANFF ES 2.2 and beyond */
};

static const struct platform_device_id mwan_devtype[] = {
	{
		.name = "mwan-compatible",
		.driver_data = MWAN_COMPATIBLE,
	}, {
		.name = "mwan-rex-v2",
		.driver_data = MWAN_REX_V2,
	}, {
		.name = "mwan-moonshine-ppro",
		.driver_data = MWAN_MOONSHINE_PRE_PROTO,
	}, {
		.name = "mwan-moonshine-proto",
		.driver_data = MWAN_MOONSHINE_PROTO,
	}, {
		.name = "mwan-moonshine",
		.driver_data = MWAN_MOONSHINE,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, mwan_devtype);

static const struct of_device_id mwan_of_match[] = {
	{
		.compatible = "mwan-compatible",
		.data = &mwan_devtype[MWAN_COMPATIBLE],
	}, {
		.compatible = "mwan-rex-v2",
		.data = &mwan_devtype[MWAN_REX_V2],
	}, {
		.compatible = "mwan-moonshine-ppro",
		.data = &mwan_devtype[MWAN_MOONSHINE_PRE_PROTO],
	}, {
		.compatible = "mwan-moonshine-proto",
		.data = &mwan_devtype[MWAN_MOONSHINE_PROTO],
	}, {
		.compatible = "mwan-moonshine",
		.data = &mwan_devtype[MWAN_MOONSHINE],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, mwan_of_match);


struct mwan_gpio {
	unsigned    ready;
	unsigned    gpio;
};

struct mwan_platform_data {
	struct mwan_gpio    wan_vbat_en;
	struct mwan_gpio    wan_usb_en;
	struct mwan_gpio    wan_power_on;
	struct mwan_gpio    wan_dpdt_mux;
	struct mwan_gpio    wan_dpdt_ctl;
	struct mwan_gpio    wan_spst_ctl;
	struct mwan_gpio    wan_disable_n;
	struct mwan_gpio    wan_dpdt_sniffer;
	struct mwan_gpio    wan_fw_ready;
	struct mwan_gpio    wan_reset_n;
	struct mwan_gpio    wan_sar_det;
	struct mwan_gpio    wan_sms_ready;
	struct mwan_gpio    wan_wakeup;
	struct mwan_gpio    soc_wakeup;
};

struct mwan_data {
	enum mwan_type            devtype;
	struct platform_device    *pdev;
	struct mwan_platform_data *pdata;

	struct class        *wan_class;
	struct device       *wan_dev;
	int                 wan_major;

	wan_status_t        wan_status;
	int                 wan_sar_detect_state;
	wan_dpdt_state_t    wan_dpdt_state;
	int                 wan_spst_state;
	int                 wan_usb_enable_state;
	int                 wan_disable_state;
	int                 wan_dpdt_sniffer_state;
	int                 wan_on_off_state;
	int                 fw_ready_condition;
	wait_queue_head_t   fw_ready_wait_q;

	struct proc_dir_entry *proc_wan_parent;
	struct proc_dir_entry *proc_wan_power;
	struct proc_dir_entry *proc_wan_type;
	struct proc_dir_entry *proc_wan_sar;
	struct proc_dir_entry *proc_wan_fwrdy;
	struct proc_dir_entry *proc_wan_usb;
	struct proc_dir_entry *proc_wan_dpdt;
	struct proc_dir_entry *proc_wan_spst;
	struct proc_dir_entry *proc_wan_disable;
	struct proc_dir_entry *proc_wan_dpdt_sniffer;

#ifdef ENABLE_FW_READY_IRQ
	struct delayed_work fw_ready_work;
	int                 fw_ready_irq;
#endif
#ifdef ENABLE_SOC_WAKEUP_IRQ
	struct delayed_work soc_wakeup_done_work;
	int                 soc_wakeup_irq;
	int                 usb_resume_request;
	int                 usb_resume_status;
#endif
#ifdef ENABLE_SMS_READY_IRQ
	struct delayed_work sms_ready_work;
	int                 sms_ready_irq;
#endif
#ifdef ENABLE_DPDT_SNIFFER_IRQ
	struct delayed_work dpdt_sniffer_work;
	int                 dpdt_sniffer_irq;
#endif

	struct mutex        lock;
};


static int modem_type = MODEM_TYPE_UNKNOWN;

static void init_modem_type(int type)
{
	if (modem_type != type) {
		log_info("type=%d:setting modem type\n", type);

		modem_type = type;
	}
}

int wan_get_modem_type(void)
{
	return modem_type;
}
EXPORT_SYMBOL(wan_get_modem_type);


/****************************************************
static const char *hwid_map[] = {
	"Unknown",    // 0
	"Unknown",    // 1
	"PreHVT",     // 2
	"HVT",        // 3
	"PreEVT",     // 4
	"EVT",        // 5
	"EVT1.1",     // 6
	"EVT1.1B",    // 7
	"DVT",        // 8
	"Unknown",    // 9
	"Unknown",    // 10
	"RexV3",      // 11
	"Unknown",    // 12
	"Unknown",    // 13
	"Unknown",    // 14
	"Unknown",    // 15
};
****************************************************/
extern int idme_hwid_value;
static int get_hw_id(void)
{
	return idme_hwid_value;
}

extern char idme_board_id_value[];
static unsigned int get_board_rev(void)
{
	char board_rev[3] = { 0, };
	static unsigned int rs = 0;

	if (rs > 0)
		return rs;

	strlcpy(board_rev, (idme_board_id_value + 6), sizeof(board_rev));

	if (unlikely(kstrtouint(board_rev, 16, &rs)))
		log_err("get_board_rev kstrtouint failed!\n");

	log_debug("board_rev=%u\n", rs);
	return rs;
}


static int mwan_get_of_pdata(struct device *dev,
			    struct mwan_platform_data *pdata,
			    enum mwan_type devtype)
{
	struct device_node *node = dev->of_node;

	if (of_gpio_named_count(node, "wan_vbat_en") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_vbat_en", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_vbat_en.ready = 1;
		pdata->wan_vbat_en.gpio = val;
		log_debug("wan_vbat_en: gpio=%d, ready=%d\n", pdata->wan_vbat_en.gpio, pdata->wan_vbat_en.ready);
	}

	if (of_gpio_named_count(node, "wan_usb_en") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_usb_en", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_usb_en.ready = 1;
		pdata->wan_usb_en.gpio = val;
		log_debug("wan_usb_en: gpio=%d, ready=%d\n", pdata->wan_usb_en.gpio, pdata->wan_usb_en.ready);
	}

	if (of_gpio_named_count(node, "wan_power_on") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_power_on", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_power_on.ready = 1;
		pdata->wan_power_on.gpio = val;
		log_debug("wan_power_on: gpio=%d, ready=%d\n", pdata->wan_power_on.gpio, pdata->wan_power_on.ready);
	}

	if (of_gpio_named_count(node, "wan_dpdt_mux") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_dpdt_mux", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_dpdt_mux.ready = 1;
		pdata->wan_dpdt_mux.gpio = val;
		log_debug("wan_dpdt_mux: gpio=%d, ready=%d\n", pdata->wan_dpdt_mux.gpio, pdata->wan_dpdt_mux.ready);
	}

	if (of_gpio_named_count(node, "wan_dpdt_ctl") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_dpdt_ctl", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_dpdt_ctl.ready = 1;
		pdata->wan_dpdt_ctl.gpio = val;
		log_debug("wan_dpdt_ctl: gpio=%d, ready=%d\n", pdata->wan_dpdt_ctl.gpio, pdata->wan_dpdt_ctl.ready);
	}

	if (of_gpio_named_count(node, "wan_spst_ctl") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_spst_ctl", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_spst_ctl.ready = 1;
		pdata->wan_spst_ctl.gpio = val;
		log_debug("wan_spst_ctl: gpio=%d, ready=%d\n", pdata->wan_spst_ctl.gpio, pdata->wan_spst_ctl.ready);
	}

	if (of_gpio_named_count(node, "wan_disable_n") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_disable_n", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_disable_n.ready = 1;
		pdata->wan_disable_n.gpio = val;
		if (devtype == MWAN_MOONSHINE)
			pdata->wan_disable_n.ready = 0;
		log_debug("wan_disable_n: gpio=%d, ready=%d\n", pdata->wan_disable_n.gpio, pdata->wan_disable_n.ready);
	}

	if (of_gpio_named_count(node, "wan_dpdt_sniffer") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_dpdt_sniffer", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_dpdt_sniffer.ready = 1;
		pdata->wan_dpdt_sniffer.gpio = val;
		log_debug("wan_dpdt_sniffer: gpio=%d, ready=%d\n", pdata->wan_dpdt_sniffer.gpio, pdata->wan_dpdt_sniffer.ready);
	}

	if (of_gpio_named_count(node, "wan_fw_ready") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_fw_ready", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_fw_ready.ready = 1;
		pdata->wan_fw_ready.gpio = val;
		log_debug("wan_fw_ready: gpio=%d, ready=%d\n", pdata->wan_fw_ready.gpio, pdata->wan_fw_ready.ready);
	}

	if (of_gpio_named_count(node, "wan_reset_n") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_reset_n", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_reset_n.ready = 1;
		pdata->wan_reset_n.gpio = val;
		log_debug("wan_reset_n: gpio=%d, ready=%d\n", pdata->wan_reset_n.gpio, pdata->wan_reset_n.ready);
	}

	if (of_gpio_named_count(node, "wan_sar_det") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_sar_det", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_sar_det.ready = 1;
		pdata->wan_sar_det.gpio = val;
		log_debug("wan_sar_det: gpio=%d, ready=%d\n", pdata->wan_sar_det.gpio, pdata->wan_sar_det.ready);
	}

	if (of_gpio_named_count(node, "wan_sms_ready") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_sms_ready", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_sms_ready.ready = 1;
		pdata->wan_sms_ready.gpio = val;
		log_debug("wan_sms_ready: gpio=%d, ready=%d\n", pdata->wan_sms_ready.gpio, pdata->wan_sms_ready.ready);
	}

	if (of_gpio_named_count(node, "wan_wakeup") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "wan_wakeup", 0, NULL);
		if (val < 0)
			return val;
		pdata->wan_wakeup.ready = 1;
		pdata->wan_wakeup.gpio = val;
		log_debug("wan_wakeup: gpio=%d, ready=%d\n", pdata->wan_wakeup.gpio, pdata->wan_wakeup.ready);
	}

	if (of_gpio_named_count(node, "soc_wakeup") > 0) {
		int val;

		val = of_get_named_gpio_flags(node, "soc_wakeup", 0, NULL);
		if (val < 0)
			return val;
		pdata->soc_wakeup.ready = 1;
		pdata->soc_wakeup.gpio = val;
		log_debug("soc_wakeup: gpio=%d, ready=%d\n", pdata->soc_wakeup.gpio, pdata->soc_wakeup.ready);
	}

	return 0;
}


static int mwan_request_gpio(struct mwan_data *mwan_data)
{
	struct platform_device *pdev = mwan_data->pdev;
	struct mwan_platform_data *pdata = mwan_data->pdata;
	int err;

	if (pdata->wan_vbat_en.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_vbat_en.gpio, "wan_vbat_en");
		if (err) {
			log_err("Failed to request wan_vbat_en, err=%d\n", err);
			return err;
		}

		err = gpio_direction_output(pdata->wan_vbat_en.gpio, 0);
		if (err)
			return err;
	}

	if (pdata->wan_power_on.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_power_on.gpio, "wan_power_on");
		if (err) {
			log_err("Failed to request wan_power_on, err=%d\n", err);
			return err;
		}

		err = gpio_direction_output(pdata->wan_power_on.gpio, 0);
		if (err)
			return err;
	}

	if (pdata->wan_reset_n.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_reset_n.gpio, "wan_reset_n");
		if (err) {
			log_err("Failed to request wan_reset_n, err=%d\n", err);
			return err;
		}

		err = gpio_direction_output(pdata->wan_reset_n.gpio, 0);
		if (err)
			return err;
	}

	if (pdata->wan_usb_en.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_usb_en.gpio, "wan_usb_en");
		if (err) {
			log_err("Failed to request wan_usb_en, err=%d\n", err);
			return err;
		}

		err = gpio_direction_output(pdata->wan_usb_en.gpio, 0);
		if (err)
			return err;
	}

	if (pdata->wan_disable_n.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_disable_n.gpio, "wan_disable_n");
		if (err) {
			log_err("Failed to request wan_disable_n, err=%d\n", err);
			return err;
		}

		err = gpio_direction_output(pdata->wan_disable_n.gpio, 0);
		if (err)
			return err;
	}

	if (pdata->wan_sar_det.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_sar_det.gpio, "wan_sar_det");
		if (err) {
			log_err("Failed to request wan_sar_det, err=%d\n", err);
			return err;
		}

		err = gpio_direction_output(pdata->wan_sar_det.gpio, 0);
		if (err)
			return err;
	}

	if (pdata->wan_dpdt_mux.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_dpdt_mux.gpio, "wan_dpdt_mux");
		if (err) {
			log_err("Failed to request wan_dpdt_mux, err=%d\n", err);
			return err;
		}

		err = gpio_direction_output(pdata->wan_dpdt_mux.gpio, 1);
		if (err)
			return err;
	}

	if (pdata->wan_dpdt_ctl.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_dpdt_ctl.gpio, "wan_dpdt_ctl");
		if (err) {
			log_err("Failed to request wan_dpdt_ctl, err=%d\n", err);
			return err;
		}

		err = gpio_direction_output(pdata->wan_dpdt_ctl.gpio, 0);
		if (err)
			return err;
	}

	if (pdata->wan_spst_ctl.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_spst_ctl.gpio, "wan_spst_ctl");
		if (err) {
			log_err("Failed to request wan_spst_ctl, err=%d\n", err);
			return err;
		}

		err = gpio_direction_output(pdata->wan_spst_ctl.gpio, 0);
		if (err)
			return err;
	}

	if (pdata->wan_wakeup.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_wakeup.gpio, "wan_wakeup");
		if (err) {
			log_err("Failed to request wan_wakeup, err=%d\n", err);
			return err;
		}

		err = gpio_direction_input(pdata->wan_wakeup.gpio);
		if (err)
			return err;
	}

	if (pdata->soc_wakeup.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->soc_wakeup.gpio, "soc_wakeup");
		if (err) {
			log_err("Failed to request soc_wakeup, err=%d\n", err);
			return err;
		}

		err = gpio_direction_input(pdata->soc_wakeup.gpio);
		if (err)
			return err;
	}

	if (pdata->wan_sms_ready.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_sms_ready.gpio, "wan_sms_ready");
		if (err) {
			log_err("Failed to request wan_sms_ready, err=%d\n", err);
			return err;
		}

		err = gpio_direction_input(pdata->wan_sms_ready.gpio);
		if (err)
			return err;
	}

	if (pdata->wan_dpdt_sniffer.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_dpdt_sniffer.gpio, "wan_dpdt_sniffer");
		if (err) {
			log_err("Failed to request wan_dpdt_sniffer, err=%d\n", err);
			return err;
		}

		err = gpio_direction_input(pdata->wan_dpdt_sniffer.gpio);
		if (err)
			return err;
	}

	if (pdata->wan_fw_ready.ready) {
		err = devm_gpio_request(&pdev->dev, pdata->wan_fw_ready.gpio, "wan_fw_ready");
		if (err) {
			log_err("Failed to request wan_fw_ready, err=%d\n", err);
			return err;
		}

		err = gpio_direction_input(pdata->wan_fw_ready.gpio);
		if (err)
			return err;
	}

	return 0;
}

static inline void gpio_wan_vbat_enable(struct mwan_data *mwan_data, int enable)
{
	if (mwan_data->pdata->wan_vbat_en.ready) {
		gpio_set_value(mwan_data->pdata->wan_vbat_en.gpio, enable);
	}
}

static inline void gpio_wan_power_on(struct mwan_data *mwan_data, int enable)
{
	if (mwan_data->pdata->wan_power_on.ready) {
		if (mwan_data->pdata->wan_reset_n.ready) {
			gpio_set_value(mwan_data->pdata->wan_reset_n.gpio, enable);
			msleep(100); //minimum 100ms required
		}

		gpio_set_value(mwan_data->pdata->wan_power_on.gpio, enable);
	}
}

static inline void gpio_wan_sar_det_enable(struct mwan_data *mwan_data, int enable)
{
	if (mwan_data->pdata->wan_sar_det.ready) {
		gpio_set_value(mwan_data->pdata->wan_sar_det.gpio, enable);
	}
}

static inline void gpio_wan_usb_enable(struct mwan_data *mwan_data, int enable)
{
	if (mwan_data->pdata->wan_usb_en.ready)
		gpio_set_value(mwan_data->pdata->wan_usb_en.gpio, enable);
}

static inline void gpio_wan_dpdt(struct mwan_data *mwan_data, wan_dpdt_state_t dpdt_state)
{
	unsigned long flags;

	local_irq_save(flags);

	if (mwan_data->pdata->wan_dpdt_mux.ready &&
		mwan_data->pdata->wan_dpdt_ctl.ready &&
		mwan_data->pdata->wan_spst_ctl.ready) {
		if (mwan_data->wan_spst_state == WAN_SPST_RELEASE) {
			gpio_set_value(mwan_data->pdata->wan_spst_ctl.gpio, 1);
			/* Requires 10us SPST switch transition time for CXA4428GC */
			udelay(10);
		}

		if (mwan_data->wan_dpdt_state == WAN_DPDT_RELEASE && dpdt_state != WAN_DPDT_RELEASE) {
			gpio_set_value(mwan_data->pdata->wan_dpdt_mux.gpio, 0);
			gpio_set_value(mwan_data->pdata->wan_dpdt_ctl.gpio, ((dpdt_state==WAN_DPDT_DIV)?1:0));
		}
		else if (mwan_data->wan_dpdt_state != WAN_DPDT_RELEASE && dpdt_state == WAN_DPDT_RELEASE) {
			gpio_set_value(mwan_data->pdata->wan_dpdt_mux.gpio, 1);
		}
		else {
			gpio_set_value(mwan_data->pdata->wan_dpdt_ctl.gpio, ((dpdt_state==WAN_DPDT_DIV)?1:0));
		}
		/* Requires 6us DPDT switch transition time for CXM3636ER */
		udelay(6);

		if (mwan_data->wan_spst_state == WAN_SPST_RELEASE) {
			gpio_set_value(mwan_data->pdata->wan_spst_ctl.gpio, 0);
		}
	}
	else if (mwan_data->pdata->wan_dpdt_ctl.ready) {
		gpio_set_value(mwan_data->pdata->wan_dpdt_ctl.gpio, ((dpdt_state==WAN_DPDT_DIV)?1:0));
	}

	local_irq_restore(flags);

	log_debug("from:%d, to:%d\n", mwan_data->wan_dpdt_state, dpdt_state);
}

static inline void gpio_wan_spst(struct mwan_data *mwan_data, wan_spst_state_t spst_state)
{
	if (mwan_data->pdata->wan_spst_ctl.ready)
		gpio_set_value(mwan_data->pdata->wan_spst_ctl.gpio, ((spst_state==WAN_SPST_ENABLE)?1:0));
}

static inline void gpio_wan_disable(struct mwan_data *mwan_data, int disable)
{
	if (mwan_data->pdata->wan_disable_n.ready)
		gpio_set_value(mwan_data->pdata->wan_disable_n.gpio, (disable?0:1));
}


#ifdef CONFIG_USB_REX
extern void ci_hdrc_set_phy_vcc(unsigned controller_id, bool vcc_enable, bool vcc_force);
#endif

static void set_wan_on_off(struct mwan_data *mwan_data, int enable)
{
	static unsigned long modem_off_jiffies = INITIAL_JIFFIES;
	unsigned long current_jiffies;
	unsigned long time_delta;
	unsigned long wait_jiffies;

	enable = enable != 0; // (ensure that "enable" is a boolean)

	if (!enable) {
		modem_off_jiffies = jiffies;
	}
	else {
		current_jiffies = jiffies;
		time_delta = current_jiffies - modem_off_jiffies;
		// allow for supercap discharge so that the modem actually powers off
		if (time_delta < MSEC_2_JIFFIES(rex_PWROFF_DISCHARGE_TIME_MSEC)) {
			// only wait for the remaining time
			wait_jiffies = (MSEC_2_JIFFIES(rex_PWROFF_DISCHARGE_TIME_MSEC) - time_delta);
			log_info("wait=%lu jiffies:modem power on delay\n", wait_jiffies);
			while (wait_jiffies) {
				set_current_state(TASK_UNINTERRUPTIBLE);
				wait_jiffies = schedule_timeout(wait_jiffies);
			}
		}
	}

	log_info("enable=%d\n", enable);

	gpio_wan_spst(mwan_data, WAN_SPST_RELEASE);
	mwan_data->wan_spst_state = WAN_SPST_RELEASE;

	gpio_wan_dpdt(mwan_data, WAN_DPDT_RELEASE);
	mwan_data->wan_dpdt_state = WAN_DPDT_RELEASE;

	if (enable) {
		gpio_wan_vbat_enable(mwan_data, enable);
		msleep(200); //minimum 150ms required. To be safe, set 200ms.
		gpio_wan_power_on(mwan_data, enable);
		gpio_wan_disable(mwan_data, mwan_data->wan_disable_state);
#ifdef CONFIG_USB_REX
		ci_hdrc_set_phy_vcc(1, true, false);
#endif
	}
	else {
#ifdef CONFIG_USB_REX
		ci_hdrc_set_phy_vcc(1, false, false);
#endif
		gpio_wan_disable(mwan_data, 1);
		gpio_wan_power_on(mwan_data, enable);
		gpio_wan_vbat_enable(mwan_data, enable);
	}
	mwan_data->wan_on_off_state = enable;

}


static int set_wan_power(struct mwan_data *mwan_data, wan_status_t new_status)
{
	wan_status_t check_status;
	int err = 0;

	if (new_status == WAN_OFF_KILL)
		check_status = WAN_OFF;
	else if (new_status == WAN_ON_FORCE)
		check_status = WAN_ON;
	else
		check_status = new_status;

	mutex_lock(&mwan_data->lock);

	if (check_status == mwan_data->wan_status) {
		mutex_unlock(&mwan_data->lock);
		return 0;
	}

	switch (new_status) {
		case WAN_ON:
		case WAN_ON_FORCE:
			// bring up WAN_ON_OFF
			mwan_data->fw_ready_condition = 0;
#ifdef ENABLE_FW_READY_IRQ
			if (mwan_data->pdata->wan_fw_ready.ready) {
				irq_set_irq_type(mwan_data->fw_ready_irq, IRQF_TRIGGER_RISING);
				enable_irq(mwan_data->fw_ready_irq);
			}
#endif
			set_wan_on_off(mwan_data, 1);

			if (new_status != WAN_ON_FORCE) {
				log_debug("fw_ready_condition=%d:Waiting for fw_ready event\n", mwan_data->fw_ready_condition);
				// wait for firmware ready to high
				if ((err = wait_event_interruptible_timeout(
								mwan_data->fw_ready_wait_q,
								mwan_data->fw_ready_condition,
								rex_PWRON_TIME_SEC*HZ)) <= 0 ) {
					log_err("err=%d: fw_ready event %s\n", err,(err)?("received signal"):("timed out"));

#ifdef ENABLE_FW_READY_IRQ
					if (mwan_data->pdata->wan_fw_ready.ready) {
						disable_irq(mwan_data->fw_ready_irq);
						set_wan_on_off(mwan_data, 0);
						err = -EIO;
						mutex_unlock(&mwan_data->lock);
						return err;
					}
#endif
				}
				else {
					log_debug("fw_ready_condition=%d, ret=%d\n", mwan_data->fw_ready_condition, err);
					err = 0;
				}
			}

			new_status = WAN_ON;
			mwan_data->fw_ready_condition = 1;

#ifdef ENABLE_SOC_WAKEUP_IRQ
			if (mwan_data->pdata->soc_wakeup.ready)
				enable_irq(mwan_data->soc_wakeup_irq);
#endif
#ifdef ENABLE_SMS_READY_IRQ
			if (mwan_data->pdata->wan_sms_ready.ready)
				enable_irq(mwan_data->sms_ready_irq);
#endif
#ifdef ENABLE_DPDT_SNIFFER_IRQ
			if (mwan_data->pdata->wan_dpdt_sniffer.ready)
				enable_irq(mwan_data->dpdt_sniffer_irq);
#endif
			break;

		case WAN_OFF :
		case WAN_OFF_KILL :
#ifdef ENABLE_SOC_WAKEUP_IRQ
			if (mwan_data->pdata->soc_wakeup.ready) {
				disable_irq(mwan_data->soc_wakeup_irq);
				cancel_delayed_work_sync(&mwan_data->soc_wakeup_done_work);
				mwan_data->usb_resume_request = 0;
			}
#endif
#ifdef ENABLE_SMS_READY_IRQ
			if (mwan_data->pdata->wan_sms_ready.ready) {
				if (!cancel_delayed_work_sync(&mwan_data->sms_ready_work))
					disable_irq(mwan_data->sms_ready_irq);
			}
#endif
#ifdef ENABLE_DPDT_SNIFFER_IRQ
			if (mwan_data->pdata->wan_dpdt_sniffer.ready) {
				if (!cancel_delayed_work_sync(&mwan_data->dpdt_sniffer_work))
					disable_irq(mwan_data->dpdt_sniffer_irq);
			}
#endif

			if (new_status != WAN_OFF_KILL) {
				log_debug("fw_ready_condition=%d:Waiting for fw_ready event\n", mwan_data->fw_ready_condition);
				// wait for firmware ready to low
				if ((err = wait_event_interruptible_timeout(
								mwan_data->fw_ready_wait_q,
								!mwan_data->fw_ready_condition,
								rex_PWROFF_TIME_SEC*HZ)) <= 0 ) {
					log_err("err=%d: fw_ready event %s\n", err,(err)?("received signal"):("timed out"));
				}
				else {
					log_debug("fw_ready_condition=%d, ret=%d\n", mwan_data->fw_ready_condition, err);
					err = 0;
				}
			}

#ifdef ENABLE_FW_READY_IRQ
			if (mwan_data->pdata->wan_fw_ready.ready) {
				if (!cancel_delayed_work_sync(&mwan_data->fw_ready_work))
					disable_irq(mwan_data->fw_ready_irq);
			}
#endif

			msleep(rex_PWROFF_HOLD_TIME_MS);

			// bring down WAN_ON_OFF
			set_wan_on_off(mwan_data, 0);
			new_status = WAN_OFF;
			mwan_data->fw_ready_condition = 0;

			break;

		default :
			log_err("request=%d:unknown power request\n", new_status);
			err = -EINVAL;
			mutex_unlock(&mwan_data->lock);
			return err;

	}

	mwan_data->wan_status = new_status;
	wan_set_power_status(new_status);

	mutex_unlock(&mwan_data->lock);

	return err;
}


static int proc_power_show(struct seq_file *m, void *v)
{
	const struct mwan_data *mwan_data = m->private;

	seq_printf(m, "%d\n", mwan_data->wan_status == WAN_ON ? 1 : 0);
	return 0;
}

static int proc_power_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_power_show, PDE_DATA(inode));
}

static ssize_t proc_power_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct mwan_data *mwan_data = m->private;
	char lbuf[16];
	unsigned char op;

	memset(lbuf, 0, sizeof(lbuf));

	if (copy_from_user(lbuf, buf, 1)) return -EFAULT;

	op = lbuf[0];
	if (op >= '0' && op <= '9') {
		wan_status_t new_wan_status = (wan_status_t)(op - '0');

		switch (new_wan_status) {

			case WAN_OFF:
			case WAN_ON:
				// perform normal on/off power handling
				if( set_wan_power(mwan_data, new_wan_status) != 0 ) return -EIO;
				break;

			case WAN_OFF_KILL:
			case WAN_ON_FORCE:
				set_wan_power(mwan_data, new_wan_status);
				break;

			default :
				log_err("request=%d:unknown power request\n", new_wan_status);
				break;
		}

	}
	else {
		log_err("request='%c':unknown power request\n", op);
	}

	return count;
}


static int proc_type_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", modem_type);
	return 0;
}

static int proc_type_open(struct inode *inode, struct  file *file)
{
	return single_open(file, proc_type_show, PDE_DATA(inode));
}

static ssize_t proc_type_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char lbuf[16], ch;

	memset(lbuf, 0, sizeof(lbuf));

	if (copy_from_user(lbuf, buf, 1)) return -EFAULT;
	ch = lbuf[0];
	if (ch >= '0' && ch <= '9')
		init_modem_type(ch - '0');
	else
		log_err("type=%c:invalid type\n", ch);

	return count;
}


static int proc_sar_show(struct seq_file *m, void *v)
{
	const struct mwan_data *mwan_data = m->private;

	seq_printf(m, "%d\n", mwan_data->wan_sar_detect_state);
	return 0;
}

static int proc_sar_open(struct inode *inode, struct  file *file)
{
	return single_open(file, proc_sar_show, PDE_DATA(inode));
}

static ssize_t proc_sar_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct mwan_data *mwan_data = m->private;
	char lbuf[16];
	unsigned char op;

	memset(lbuf, 0, sizeof(lbuf));

	if (copy_from_user(lbuf, buf, 1)) return -EFAULT;

	op = lbuf[0];
	if (op >= '0' && op <= '1') {
		int enable = (int)(op - '0');

		gpio_wan_sar_det_enable(mwan_data, enable);
		mwan_data->wan_sar_detect_state = enable;

	} else {
		log_err("request='%c':unknown sar_detect request\n", op);
	}

	return count;
}


static int proc_usb_en_show(struct seq_file *m, void *v)
{
	const struct mwan_data *mwan_data = m->private;

	seq_printf(m, "%d\n", mwan_data->wan_usb_enable_state);
	return 0;
}

static int proc_usb_en_open(struct inode *inode, struct  file *file)
{
	return single_open(file, proc_usb_en_show, PDE_DATA(inode));
}

static ssize_t proc_usb_en_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct mwan_data *mwan_data = m->private;
	char lbuf[16];
	unsigned char op;

	memset(lbuf, 0, sizeof(lbuf));

	if (copy_from_user(lbuf, buf, 1)) return -EFAULT;

	op = lbuf[0];
	if (op >= '0' && op <= '1') {
		int enable = (int)(op - '0');

		if (enable != mwan_data->wan_usb_enable_state) {
			gpio_wan_usb_enable(mwan_data, enable);
			mwan_data->wan_usb_enable_state = enable;
		}
	} else {
		log_err("request='%c':unknown usb_en request\n", op);
	}

	return count;
}


static int proc_dpdt_show(struct seq_file *m, void *v)
{
	const struct mwan_data *mwan_data = m->private;

	seq_printf(m, "%d\n", mwan_data->wan_dpdt_state);
	return 0;
}

static int proc_dpdt_open(struct inode *inode, struct  file *file)
{
	return single_open(file, proc_dpdt_show, PDE_DATA(inode));
}

static ssize_t proc_dpdt_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct mwan_data *mwan_data = m->private;
	char lbuf[16];
	unsigned char op;

	memset(lbuf, 0, sizeof(lbuf));

	if (copy_from_user(lbuf, buf, 1)) return -EFAULT;

	op = lbuf[0];
	if (op >= '0' && op <= '2') {
		wan_dpdt_state_t dpdt_state = (wan_dpdt_state_t)(op - '0');
		if (dpdt_state != mwan_data->wan_dpdt_state) {
			gpio_wan_dpdt(mwan_data, dpdt_state);
			mwan_data->wan_dpdt_state = dpdt_state;
		}
	} else {
		log_err("request='%c':unknown dpdt request\n", op);
	}

	return count;
}


static int proc_spst_show(struct seq_file *m, void *v)
{
	const struct mwan_data *mwan_data = m->private;

	seq_printf(m, "%d\n", mwan_data->wan_spst_state);
	return 0;
}

static int proc_spst_open(struct inode *inode, struct  file *file)
{
	return single_open(file, proc_spst_show, PDE_DATA(inode));
}

static ssize_t proc_spst_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct mwan_data *mwan_data = m->private;
	char lbuf[16];
	unsigned char op;

	memset(lbuf, 0, sizeof(lbuf));

	if (copy_from_user(lbuf, buf, 1)) return -EFAULT;

	op = lbuf[0];
	if (op >= '0' && op <= '2') {
		wan_spst_state_t spst_state = (wan_spst_state_t)(op - '0');
		if (spst_state != mwan_data->wan_spst_state) {
			gpio_wan_spst(mwan_data, spst_state);
			mwan_data->wan_spst_state = spst_state;
		}
	} else {
		log_err("request='%c':unknown spst request\n", op);
	}

	return count;
}


static int proc_disable_show(struct seq_file *m, void *v)
{
	const struct mwan_data *mwan_data = m->private;

	seq_printf(m, "%d\n", mwan_data->wan_disable_state);
	return 0;
}

static int proc_disable_open(struct inode *inode, struct  file *file)
{
	return single_open(file, proc_disable_show, PDE_DATA(inode));
}

static ssize_t proc_disable_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct mwan_data *mwan_data = m->private;
	char lbuf[16];
	unsigned char op;

	memset(lbuf, 0, sizeof(lbuf));

	if (copy_from_user(lbuf, buf, 1)) return -EFAULT;

	op = lbuf[0];
	if (op >= '0' && op <= '1') {
		int disable = (int)(op - '0');

		mwan_data->wan_disable_state = disable;
		if (mwan_data->wan_status == WAN_ON)
			gpio_wan_disable(mwan_data, disable);
	} else {
		log_err("request='%c':unknown disable request\n", op);
	}

	return count;
}


static int proc_dpdt_sniffer_show(struct seq_file *m, void *v)
{
	struct mwan_data *mwan_data = m->private;

	if (mwan_data->pdata->wan_dpdt_sniffer.ready)
		mwan_data->wan_dpdt_sniffer_state = gpio_get_value(mwan_data->pdata->wan_dpdt_sniffer.gpio);

	seq_printf(m, "%d\n", mwan_data->wan_dpdt_sniffer_state);
	return 0;
}

static int proc_dpdt_sniffer_open(struct inode *inode, struct  file *file)
{
	return single_open(file, proc_dpdt_sniffer_show, PDE_DATA(inode));
}


static int proc_fwrdy_show(struct seq_file *m, void *v)
{
	const struct mwan_data *mwan_data = m->private;

	seq_printf(m, "%d\n", mwan_data->fw_ready_condition);
	return 0;
}

static int proc_fwrdy_open(struct inode *inode, struct  file *file)
{
	return single_open(file, proc_fwrdy_show, PDE_DATA(inode));
}


static const struct file_operations proc_fops_wan_power = {
	.owner		= THIS_MODULE,
	.open		= proc_power_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= proc_power_write,
};

static const struct file_operations proc_fops_wan_type = {
	.owner		= THIS_MODULE,
	.open		= proc_type_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= proc_type_write,
};

static const struct file_operations proc_fops_wan_sar = {
	.owner		= THIS_MODULE,
	.open		= proc_sar_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= proc_sar_write,
};

static const struct file_operations proc_fops_wan_fwrdy = {
	.owner		= THIS_MODULE,
	.open		= proc_fwrdy_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations proc_fops_wan_usb_en = {
	.owner		= THIS_MODULE,
	.open		= proc_usb_en_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= proc_usb_en_write,
};

static const struct file_operations proc_fops_wan_dpdt = {
	.owner		= THIS_MODULE,
	.open		= proc_dpdt_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= proc_dpdt_write,
};

static const struct file_operations proc_fops_wan_spst = {
	.owner		= THIS_MODULE,
	.open		= proc_spst_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= proc_spst_write,
};

static const struct file_operations proc_fops_wan_disable = {
	.owner		= THIS_MODULE,
	.open		= proc_disable_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= proc_disable_write,
};

static const struct file_operations proc_fops_wan_dpdt_sniffer = {
	.owner		= THIS_MODULE,
	.open		= proc_dpdt_sniffer_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int proc_init(struct mwan_data *mwan_data)
{
	int ret = 0;

	// create parent directory
	mwan_data->proc_wan_parent = proc_mkdir(PROC_WAN, NULL);

	if (mwan_data->proc_wan_parent) {
		mwan_data->proc_wan_power = proc_create_data(PROC_WAN_POWER, S_IWUSR | S_IRUGO,
										mwan_data->proc_wan_parent,	&proc_fops_wan_power, mwan_data);
		if (mwan_data->proc_wan_power == NULL) {
			log_err("can not create proc: %s \n", PROC_WAN_POWER);
		}

		mwan_data->proc_wan_type = proc_create_data(PROC_WAN_TYPE, S_IWUSR | S_IRUGO,
										mwan_data->proc_wan_parent,&proc_fops_wan_type, mwan_data);
		if (mwan_data->proc_wan_type == NULL) {
			log_err("can not create proc: %s \n", PROC_WAN_TYPE);
		}

		if (mwan_data->pdata->wan_sar_det.ready) {
			mwan_data->proc_wan_sar = proc_create_data(PROC_WAN_SAR, S_IWUSR | S_IRUGO,
										mwan_data->proc_wan_parent, &proc_fops_wan_sar, mwan_data);
			if (mwan_data->proc_wan_sar == NULL) {
				log_err("can not create proc: %s \n", PROC_WAN_SAR);
			}
		}

		mwan_data->proc_wan_fwrdy = proc_create_data(PROC_WAN_FWRDY, S_IRUGO,
										mwan_data->proc_wan_parent, &proc_fops_wan_fwrdy, mwan_data);
		if (mwan_data->proc_wan_fwrdy == NULL) {
			log_err("can not create proc: %s \n", PROC_WAN_FWRDY);
		}

		mwan_data->proc_wan_usb = proc_create_data(PROC_WAN_USB_EN, S_IWUSR | S_IRUGO,
										mwan_data->proc_wan_parent,&proc_fops_wan_usb_en, mwan_data);
		if (mwan_data->proc_wan_usb == NULL) {
			log_err("can not create proc: %s \n", PROC_WAN_USB_EN);
		}

		if (mwan_data->pdata->wan_dpdt_ctl.ready) {
			mwan_data->proc_wan_dpdt = proc_create_data(PROC_WAN_DPDT, S_IWUSR | S_IRUGO,
											mwan_data->proc_wan_parent,&proc_fops_wan_dpdt, mwan_data);
			if (mwan_data->proc_wan_dpdt == NULL) {
				log_err("can not create proc: %s \n", PROC_WAN_DPDT);
			}
		}

		if (mwan_data->pdata->wan_spst_ctl.ready) {
			mwan_data->proc_wan_spst = proc_create_data(PROC_WAN_SPST, S_IWUSR | S_IRUGO,
											mwan_data->proc_wan_parent,&proc_fops_wan_spst, mwan_data);
			if (mwan_data->proc_wan_spst == NULL) {
				log_err("can not create proc: %s \n", PROC_WAN_SPST);
			}
		}

		if (mwan_data->pdata->wan_disable_n.ready) {
			mwan_data->proc_wan_disable = proc_create_data(PROC_WAN_DISABLE, S_IWUSR | S_IRUGO,
												mwan_data->proc_wan_parent,&proc_fops_wan_disable, mwan_data);
			if (mwan_data->proc_wan_disable == NULL) {
				log_err("can not create proc: %s \n", PROC_WAN_DISABLE);
			}
		}

		if (mwan_data->pdata->wan_dpdt_sniffer.ready) {
			mwan_data->proc_wan_dpdt_sniffer = proc_create_data(PROC_WAN_DPDT_SNIFFER, S_IRUGO,
													mwan_data->proc_wan_parent,&proc_fops_wan_dpdt_sniffer, mwan_data);
			if (mwan_data->proc_wan_dpdt_sniffer == NULL) {
				log_err("can not create proc: %s \n", PROC_WAN_DPDT_SNIFFER);
			}
		}
	}
	else {
		log_err("can not create proc_wan_parent \n");
		ret = -1;
	}

	return ret;
}

static void proc_exit(struct mwan_data *mwan_data)
{
	if (mwan_data->proc_wan_parent) {
		if (mwan_data->proc_wan_power) {
			remove_proc_entry(PROC_WAN_POWER, mwan_data->proc_wan_parent);
			mwan_data->proc_wan_power = NULL;
		}
		if (mwan_data->proc_wan_type) {
			remove_proc_entry(PROC_WAN_TYPE, mwan_data->proc_wan_parent);
			mwan_data->proc_wan_type = NULL;
		}
		if (mwan_data->proc_wan_sar) {
			remove_proc_entry(PROC_WAN_SAR, mwan_data->proc_wan_parent);
			mwan_data->proc_wan_sar = NULL;
		}
		if (mwan_data->proc_wan_fwrdy) {
			remove_proc_entry(PROC_WAN_FWRDY, mwan_data->proc_wan_parent);
			mwan_data->proc_wan_fwrdy = NULL;
		}
		if (mwan_data->proc_wan_usb) {
			remove_proc_entry(PROC_WAN_USB_EN, mwan_data->proc_wan_parent);
			mwan_data->proc_wan_usb = NULL;
		}
		if (mwan_data->proc_wan_dpdt) {
			remove_proc_entry(PROC_WAN_DPDT, mwan_data->proc_wan_parent);
			mwan_data->proc_wan_dpdt = NULL;
		}
		if (mwan_data->proc_wan_spst) {
			remove_proc_entry(PROC_WAN_SPST, mwan_data->proc_wan_parent);
			mwan_data->proc_wan_spst = NULL;
		}
		if (mwan_data->proc_wan_disable) {
			remove_proc_entry(PROC_WAN_DISABLE, mwan_data->proc_wan_parent);
			mwan_data->proc_wan_disable = NULL;
		}
		if (mwan_data->proc_wan_dpdt_sniffer) {
			remove_proc_entry(PROC_WAN_DPDT_SNIFFER, mwan_data->proc_wan_parent);
			mwan_data->proc_wan_dpdt_sniffer = NULL;
		}

		remove_proc_entry(PROC_WAN, NULL);
		mwan_data->proc_wan_parent = NULL;
	}
}


#ifdef ENABLE_FW_READY_IRQ
static void do_fw_ready_work(struct work_struct *ws)
{
	struct mwan_data *mwan_data =
		container_of(ws, struct mwan_data, fw_ready_work.work);

	/* FW ready line is high until power off */
	if (gpio_get_value(mwan_data->pdata->wan_fw_ready.gpio)) {
		log_info("fw_ready changed low to high\n");
		if (!mwan_data->fw_ready_condition) {
			mwan_data->fw_ready_condition = 1;
			wake_up(&mwan_data->fw_ready_wait_q);
		}
		irq_set_irq_type(mwan_data->fw_ready_irq, IRQF_TRIGGER_FALLING);
	}
	else {
		log_info("fw_ready changed high to low\n");
		if (mwan_data->fw_ready_condition) {
			mwan_data->fw_ready_condition =  0;
			wake_up(&mwan_data->fw_ready_wait_q);
		}
		irq_set_irq_type(mwan_data->fw_ready_irq, IRQF_TRIGGER_RISING);
	}
	enable_irq(mwan_data->fw_ready_irq);
}

static irqreturn_t fw_ready_irq_handler(int irq, void *dev_id)
{
	struct mwan_data *mwan_data = dev_id;

	disable_irq_nosync(irq);
	schedule_delayed_work(&mwan_data->fw_ready_work, MSEC_2_JIFFIES(rex_FW_READY_DBOUNCE_TIME_MSEC));
	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_USB_REX_WAN
extern int ehci_host_wan_resume_device(void);
#else
static inline int ehci_host_wan_resume_device(void) { return 0; }
#endif

#ifdef ENABLE_SOC_WAKEUP_IRQ
static void do_soc_wakeup_done_work(struct work_struct *ws)
{
	struct mwan_data *mwan_data =
		container_of(ws, struct mwan_data, soc_wakeup_done_work.work);
	int ret;
	unsigned long delay;

	if (mwan_data->usb_resume_request && mwan_data->usb_resume_status == -EBUSY) {
		mwan_data->usb_resume_status = 0;
		ret = ehci_host_wan_resume_device();
		delay = MSEC_2_JIFFIES(rex_USB_WAKEUP_DONE_TIME_MSEC - rex_USB_RESUME_RETRY_TIME_MSEC);
		schedule_delayed_work(&mwan_data->soc_wakeup_done_work, delay);
		dev_dbg(mwan_data->wan_dev, "rescheduled, ret=%d\n", ret);
		return;
	}
	mwan_data->usb_resume_request = 0;

	dev_dbg(mwan_data->wan_dev, "handled\n");
}

static irqreturn_t soc_wakeup_irq_handler(int irq, void  *dev_id)
{
	struct mwan_data *mwan_data = dev_id;
	unsigned long delay;

	if (!mwan_data->usb_resume_request) {
		mwan_data->usb_resume_request = 1;
		mwan_data->usb_resume_status = ehci_host_wan_resume_device();
		if (mwan_data->usb_resume_status == -EBUSY)
			delay = MSEC_2_JIFFIES(rex_USB_RESUME_RETRY_TIME_MSEC);
		else
			delay = MSEC_2_JIFFIES(rex_USB_WAKEUP_DONE_TIME_MSEC);
		schedule_delayed_work(&mwan_data->soc_wakeup_done_work, delay);
	}
	return IRQ_HANDLED;
}
#endif

#ifdef ENABLE_SMS_READY_IRQ
static void do_sms_ready_work(struct work_struct *ws)
{
	struct mwan_data *mwan_data =
		container_of(ws, struct mwan_data, sms_ready_work.work);
	static unsigned long last_tph_sec = 0;
	unsigned long current_sec = CURRENT_TIME_SEC.tv_sec;

	if (gpio_get_value(mwan_data->pdata->wan_sms_ready.gpio)) {
		/* limit back-to-back interrupts.*/
		if( current_sec - last_tph_sec >= WAKE_EVENT_INTERVAL ) {
			last_tph_sec = current_sec;
			kobject_uevent(&mwan_data->wan_dev->kobj, KOBJ_CHANGE);
			log_info("tph::tph event occurred; notifying system of TPH\n");
		}
	}

	enable_irq(mwan_data->sms_ready_irq);
}

static irqreturn_t sms_ready_irq_handler(int irq, void  *dev_id)
{
	struct mwan_data *mwan_data = dev_id;

	disable_irq_nosync(irq);
	schedule_delayed_work(&mwan_data->sms_ready_work, MSEC_2_JIFFIES(rex_SMS_READY_DBOUNCE_TIME_MSEC));
	return IRQ_HANDLED;
}
#endif

#ifdef ENABLE_DPDT_SNIFFER_IRQ
static void do_dpdt_sniffer_work(struct work_struct *ws)
{
	struct mwan_data *mwan_data =
		container_of(ws, struct mwan_data, dpdt_sniffer_work.work);

	if (gpio_get_value(mwan_data->pdata->wan_dpdt_sniffer.gpio)) {
		log_info("BAS Tx Antenna: Div\n");
	}
	else {
		log_info("BAS Tx Antenna: Main\n");
	}

	enable_irq(mwan_data->dpdt_sniffer_irq);
}

static irqreturn_t dpdt_sniffer_irq_handler(int irq, void  *dev_id)
{
	struct mwan_data *mwan_data = dev_id;

	disable_irq_nosync(irq);
	schedule_delayed_work(&mwan_data->dpdt_sniffer_work, MSEC_2_JIFFIES(rex_DPDT_SNIFFER_DBOUNCE_TIME_MSEC));
	return IRQ_HANDLED;
}
#endif

static int rex_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mwan_data *mwan_data = platform_get_drvdata(pdev);

	dev_dbg(mwan_data->wan_dev, "disable irqs (%d)\n", mwan_data->wan_on_off_state);

	if (mwan_data->wan_on_off_state) {
#ifdef ENABLE_FW_READY_IRQ
		if (mwan_data->pdata->wan_fw_ready.ready) {
			if (!cancel_delayed_work_sync(&mwan_data->fw_ready_work))
				disable_irq(mwan_data->fw_ready_irq);
		}
#endif
#ifdef ENABLE_SOC_WAKEUP_IRQ
		if (mwan_data->pdata->soc_wakeup.ready) {
			disable_irq(mwan_data->soc_wakeup_irq);
			cancel_delayed_work_sync(&mwan_data->soc_wakeup_done_work);
			mwan_data->usb_resume_request = 0;
		}
#endif
#ifdef ENABLE_SMS_READY_IRQ
		if (mwan_data->pdata->wan_sms_ready.ready) {
			if (!cancel_delayed_work_sync(&mwan_data->sms_ready_work))
				disable_irq(mwan_data->sms_ready_irq);
		}
#endif
#ifdef ENABLE_DPDT_SNIFFER_IRQ
		if (mwan_data->pdata->wan_dpdt_sniffer.ready) {
			if (!cancel_delayed_work_sync(&mwan_data->dpdt_sniffer_work))
				disable_irq(mwan_data->dpdt_sniffer_irq);
		}
#endif
	}

	return 0;
}

static int rex_resume(struct platform_device *pdev)
{
	struct mwan_data *mwan_data = platform_get_drvdata(pdev);

	dev_dbg(mwan_data->wan_dev, "enable irqs (%d)\n", mwan_data->wan_on_off_state);

	if (mwan_data->wan_on_off_state) {
#ifdef ENABLE_SOC_WAKEUP_IRQ
		if (mwan_data->pdata->soc_wakeup.ready)
			enable_irq(mwan_data->soc_wakeup_irq);
#endif
#ifdef ENABLE_SMS_READY_IRQ
		if (mwan_data->pdata->wan_sms_ready.ready)
			enable_irq(mwan_data->sms_ready_irq);
#endif
#ifdef ENABLE_DPDT_SNIFFER_IRQ
		if (mwan_data->pdata->wan_dpdt_sniffer.ready)
			enable_irq(mwan_data->dpdt_sniffer_irq);
#endif
#ifdef ENABLE_FW_READY_IRQ
		if (mwan_data->pdata->wan_fw_ready.ready)
			enable_irq(mwan_data->fw_ready_irq);
#endif
	}

	return 0;
}

static int rex_probe(struct platform_device *pdev)
{
	int ret = -1;
	int err = 0;
	struct mwan_data *mwan_data;
	const struct of_device_id *of_id;
	enum mwan_type devtype;
	struct mwan_platform_data *pdata = dev_get_platdata(&pdev->dev);

	log_info("Mario WAN driver for REX " VERSION " HWID=%d REV=%d\n", get_hw_id(), get_board_rev());

	of_id = of_match_device(mwan_of_match, &pdev->dev);
	if (of_id) {
		const struct platform_device_id *id_entry = of_id->data;
		devtype = id_entry->driver_data;
		if (devtype == MWAN_COMPATIBLE) {
			switch (get_hw_id()) {
				case 0:
				case 1:
					devtype = MWAN_MOONSHINE_PROTO;
					break;
				default:
					devtype = MWAN_MOONSHINE;
					break;
			}
		}
	}
	else {
		log_err("No match device found\n");
		return -ENODEV;
	}

	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(struct mwan_platform_data), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		err = mwan_get_of_pdata(&pdev->dev, pdata, devtype);
		if (err)
			return err;
	}

	mwan_data = devm_kzalloc(&pdev->dev, sizeof(struct mwan_data), GFP_KERNEL);
	if (!mwan_data)
		return -ENOMEM;

	mwan_data->devtype = devtype;
	mwan_data->pdev = pdev;
	mwan_data->pdata = pdata;
	platform_set_drvdata(pdev, mwan_data);

	mwan_data->wan_status = WAN_OFF;
	mwan_data->wan_dpdt_state = WAN_DPDT_RELEASE;
	mwan_data->wan_spst_state = WAN_SPST_RELEASE;

	err = mwan_request_gpio(mwan_data);
	if (err) {
		log_err("Failed to request WAN GPIO, err=%d\n", err);
		return err;
	}

	mutex_init(&mwan_data->lock);

	err = proc_init(mwan_data);
	if (err < 0) {
		log_err("proc_init failed\n");
		goto exit0;
	}

	init_waitqueue_head(&mwan_data->fw_ready_wait_q);

#ifdef ENABLE_FW_READY_IRQ
	INIT_DELAYED_WORK(&mwan_data->fw_ready_work, do_fw_ready_work);

	if (pdata->wan_fw_ready.ready) {
		mwan_data->fw_ready_irq = gpio_to_irq(pdata->wan_fw_ready.gpio);
		err = devm_request_irq(&pdev->dev, mwan_data->fw_ready_irq, fw_ready_irq_handler,
						IRQF_TRIGGER_RISING, "wan_fw_ready_irq", mwan_data);
		if (err < 0) {
			log_err("irq=%d:Could not request fw_ready_irq\n", mwan_data->fw_ready_irq);
			goto exit1;
		}
		log_info("wan_fw_ready_irq: %d\n", mwan_data->fw_ready_irq);

		/* disable fw_ready_irq until WAN is powered up */
		disable_irq(mwan_data->fw_ready_irq);
	}
#endif

#ifdef ENABLE_SOC_WAKEUP_IRQ
	INIT_DELAYED_WORK(&mwan_data->soc_wakeup_done_work, do_soc_wakeup_done_work);

	if (pdata->soc_wakeup.ready) {
		mwan_data->soc_wakeup_irq = gpio_to_irq(pdata->soc_wakeup.gpio);
		err = devm_request_threaded_irq(&pdev->dev, mwan_data->soc_wakeup_irq, NULL, soc_wakeup_irq_handler,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT, "soc_wakeup_irq", mwan_data);
		if (err < 0) {
			log_err("irq=%d:Could not request soc_wakeup_irq\n", mwan_data->soc_wakeup_irq);
			goto exit1;
		}
		log_info("soc_wakeup_irq: %d\n", mwan_data->soc_wakeup_irq);

		/* disable soc_wakeup_irq until WAN is powered up */
		disable_irq(mwan_data->soc_wakeup_irq);
	}
#endif

#ifdef ENABLE_SMS_READY_IRQ
	INIT_DELAYED_WORK(&mwan_data->sms_ready_work, do_sms_ready_work);

	if (pdata->wan_sms_ready.ready) {
		mwan_data->sms_ready_irq = gpio_to_irq(pdata->wan_sms_ready.gpio);
		err = devm_request_irq(&pdev->dev, mwan_data->sms_ready_irq, sms_ready_irq_handler,
						IRQF_TRIGGER_RISING, "sms_ready_irq", mwan_data);
		if (err < 0) {
			log_err("irq=%d:Could not request sms_ready_irq\n", mwan_data->sms_ready_irq);
			goto exit1;
		}
		log_info("sms_ready_irq: %d\n", mwan_data->sms_ready_irq);

		/* disable sms_ready_irq until WAN is powered up */
		disable_irq(mwan_data->sms_ready_irq);
	}
#endif

#ifdef ENABLE_DPDT_SNIFFER_IRQ
	INIT_DELAYED_WORK(&mwan_data->dpdt_sniffer_work, do_dpdt_sniffer_work);

	if (pdata->wan_dpdt_sniffer.ready) {
		mwan_data->dpdt_sniffer_irq = gpio_to_irq(pdata->wan_dpdt_sniffer.gpio);
		err = devm_request_irq(&pdev->dev, mwan_data->dpdt_sniffer_irq, dpdt_sniffer_irq_handler,
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "dpdt_sniffer_irq", mwan_data);
		if (err < 0) {
			log_err("irq=%d:Could not request dpdt_sniffer_irq\n", mwan_data->dpdt_sniffer_irq);
			goto exit1;
		}
		log_info("dpdt_sniffer_irq: %d\n", mwan_data->dpdt_sniffer_irq);

		/* disable dpdt_sniffer_irq until WAN is powered up */
		disable_irq(mwan_data->dpdt_sniffer_irq);
	}
#endif

	mwan_data->wan_major = register_chrdev(0, WAN_STRING_DEV, &mwan_ops);
	if (mwan_data->wan_major < 0) {
		ret = mwan_data->wan_major;
		log_err("err=%d:could not register device\n", ret);
		goto exit1;
	}

	mwan_data->wan_class = class_create(THIS_MODULE, WAN_STRING_CLASS);
	if (IS_ERR(mwan_data->wan_class)) {
		ret = PTR_ERR(mwan_data->wan_class);
		log_err("err=%d:could not create class\n", ret);
		goto exit2;
	}

	mwan_data->wan_dev = device_create(mwan_data->wan_class, NULL, MKDEV(mwan_data->wan_major, 0), NULL, WAN_STRING_DEV);
	if (IS_ERR(mwan_data->wan_dev)) {
		ret = PTR_ERR(mwan_data->wan_dev);
		log_err("err=%d:could not create class device\n", ret);
		goto exit3;
	}

	wan_set_power_status(WAN_OFF);

	log_info("done (devtype=%d)\n", devtype);
	return 0;

exit3:
	class_destroy(mwan_data->wan_class);
	mwan_data->wan_class = NULL;

exit2:
	unregister_chrdev(mwan_data->wan_major, WAN_STRING_DEV);

exit1:
	proc_exit(mwan_data);

exit0:
	return ret;

}

static int rex_remove(struct platform_device *pdev)
{
	struct mwan_data *mwan_data = platform_get_drvdata(pdev);

	if (wan_get_power_status() == WAN_ON)
		set_wan_on_off(mwan_data, 0);

	wan_set_power_status(WAN_INVALID);

	if (mwan_data->wan_dev != NULL) {
		device_destroy(mwan_data->wan_class, MKDEV(mwan_data->wan_major, 0));
		mwan_data->wan_dev = NULL;
		class_destroy(mwan_data->wan_class);
		unregister_chrdev(mwan_data->wan_major, WAN_STRING_DEV);
	}

	proc_exit(mwan_data);

	log_info("done\n");
	return 0;
}

static struct platform_driver rex_driver = {
	.driver = {
		.name = "mwan",
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(mwan_of_match),
#endif
	},
	.suspend = rex_suspend,
	.resume = rex_resume,
	.probe = rex_probe,
	.remove = rex_remove,
};

module_platform_driver(rex_driver);

MODULE_DESCRIPTION("Mario WAN driver for REX");
MODULE_AUTHOR("Amazon Lab126");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION);
MODULE_ALIAS("platform:mwan");
