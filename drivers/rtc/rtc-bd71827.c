/*
 * @file RoHM BD71827 Real Time Clock interface
 *
 * Copyright (C) 2016.
 *
 * @author Cong Pham <cpham2403@gmail.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#define DEBUG
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/mfd/bd71827.h>

#if defined(CONFIG_AMAZON_METRICS_LOG)
#include <linux/metricslog.h>
char bd71827_rtc_metric_buf[BD718xx_METRIC_BUFFER_SIZE];
#endif
extern int bd71827_get_battery_mah(void);

/* do not need week for alarm mask */
#define ALM_MASK (A0_SEC|A0_MIN|A0_HOUR|A0_DAY|A0_MON|A0_YEAR)

/** @brief bd71827 rtc struct */
struct bd71827_rtc {
	struct rtc_device	*rtc;		/**< system rtc device */
	int irq;				/**< rtc irq */
};

unsigned long suspend_time, wakeup_time, total_suspend_time, total_wake_time, up_time;
int suspend_mah, wakeup_mah;
/* @brief Total number of RTC registers needed to set time*/
// #define NUM_TIME_REGS	(BD71827_REG_YEAR - BD71827_REG_SEC + 1)

/**@brief enable or disable rtc alarm irq
 * @param dev rtc device of system
 * @param enabled enable if non-zero
 * @retval 0 success
 * @retval negative error number
 */
static int bd71827_rtc_alarm_irq_enable(struct device *dev, unsigned enabled)
{
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);

	dev_info(dev, "%s: %s rtc alarm\n", __func__, enabled ? "enable" : "disable");
	return regmap_write(mfd->regmap, BD71827_REG_ALM0_MASK, enabled ? ALM_MASK : 0);
}

/**@brief bd71827 rtc time convert to linux time
 * @param tm linux rtc time
 * @param hw_rtc bd71827 rtc time
 * @return argument tm
 */
static struct rtc_time* hw_to_rtc_time(struct rtc_time* tm, const struct bd71827_rtc_alarm* hw_rtc) {
	u8 hour;

	tm->tm_sec = bcd2bin(SEC_MASK(hw_rtc->sec));
	tm->tm_min = bcd2bin(MIN_MASK(hw_rtc->min));

	hour = bcd2bin(HOUR_MASK(hw_rtc->hour));
	if ((hw_rtc->hour & HOUR_24HOUR) == 0) {
		if (hour == 12) {
			hour = 0;
		} else if (hour == 32) {
			hour = 12;
		} else if ((21 <= hour) && (hour <= 31)) {
			hour = hour - (21 - 13);
		}
	}
	tm->tm_hour = hour;

	tm->tm_mday = bcd2bin(DAY_MASK(hw_rtc->day));
	tm->tm_mon = bcd2bin(MONTH_MASK(hw_rtc->month)) - 1;
	tm->tm_year = bcd2bin(YEAR_MASK(hw_rtc->year)) + 100;
	return tm;
}

/**@brief linux time convert bd71827 rtc time
 * @param hw_rtc bd71827 rtc time
 * @param tm linux rtc time
 * @return argument hw_rtc
 */
static struct bd71827_rtc_alarm* rtc_time_to_hw(struct bd71827_rtc_alarm* hw_rtc, const struct rtc_time* tm) {
	hw_rtc->sec = bin2bcd(tm->tm_sec);
	hw_rtc->min = bin2bcd(tm->tm_min);
	hw_rtc->hour = HOUR_24HOUR | bin2bcd(tm->tm_hour);
	hw_rtc->day = bin2bcd(tm->tm_mday);
	hw_rtc->month = bin2bcd(tm->tm_mon + 1);
	hw_rtc->year = bin2bcd(tm->tm_year - 100);

	return hw_rtc;
}

/*
 * Gets current bd71827 RTC time and date parameters.
 *
 * The RTC's time/alarm representation is not what gmtime(3) requires
 * Linux to use:
 *
 *  - Months are 1..12 vs Linux 0-11
 *  - Years are 0..99 vs Linux 1900..N (we assume 21st century)
 */
/**@brief read date/time from bd71827 rtc
 * @param dev rtc device of system
 * @param tm date/time store target
 * @retval 0 success
 * @retval negative error number
 */
static int bd71827_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct bd71827_rtc_alarm rtc_data[1];
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	int ret;

	ret = regmap_bulk_read(mfd->regmap, BD71827_REG_SEC, rtc_data, sizeof rtc_data);
	if (ret < 0) {
		dev_err(dev, "reading from RTC failed with err:%d\n", ret);
		return ret;
	}

	hw_to_rtc_time(tm, rtc_data);

	return ret;
}

/**@brief write date/time to bd71827 rtc
 * @param dev rtc device of system
 * @param tm date/time source
 * @retval 0 success
 * @retval negative error number
 */
static int bd71827_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct bd71827_rtc_alarm rtc_data[1];
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	int ret;

	rtc_time_to_hw(rtc_data, tm);

	/* update all the time registers in one shot */
	ret = regmap_bulk_write(mfd->regmap, BD71827_REG_SEC, rtc_data, sizeof rtc_data);
	if (ret < 0) {
		dev_err(dev, "rtc_set_time error %d\n", ret);
		return ret;
	}
	dev_info(dev, "%s, rtc time set to 20%02d-%02d-%02d %02d:%02d:%02d\n",__func__,
			bcd2bin(rtc_data->year), bcd2bin(rtc_data->month), bcd2bin(rtc_data->day),
			bcd2bin(HOUR_MASK(rtc_data->hour)), bcd2bin(rtc_data->min), bcd2bin(rtc_data->sec));

	return ret;
}

/**@brief Gets current bd71827 RTC alarm time.
 * @param dev rtc device of system
 * @param alm alarm date/time store target
 * @retval 0 success
 * @retval negative error number
 */
static int bd71827_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct bd71827_rtc_alarm rtc_data[1];
	u32 int_val;
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	int ret;

	ret = regmap_bulk_read(mfd->regmap, BD71827_REG_ALM0_SEC, rtc_data, sizeof rtc_data);
	if (ret < 0) {
		dev_err(dev, "rtc_read_alarm error %d\n", ret);
		return ret;
	}
	
	hw_to_rtc_time(&alm->time, rtc_data);

	ret = regmap_read(mfd->regmap, BD71827_REG_ALM0_MASK, &int_val);
	if (ret < 0)
		return ret;

	if (int_val & ALM_MASK)
		alm->enabled = 1;

	return ret;
}

/**@brief Set current bd71827 RTC alarm time
 * @param dev rtc device of system
 * @param alm alarm date/time to set
 * @retval 0 success
 * @retval negative error number
 */
static int bd71827_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct bd71827_rtc_alarm rtc_data[1];
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	int ret;

	// printk("%s() L%d\n", __func__, __LINE__);

	/*
	 * Figure 39. RTC ALARM0 Interrupt Process
	 *
	 * Clear ALM0_MASK Register.
	 *
	 * Set Alarm Threshold Registers.
	 * (ALM0_SEC ~ ALM0_YEAR)
	 *
	 * Set ALM0_MASK Register.
	 *
	 */

	ret = bd71827_rtc_alarm_irq_enable(dev, 0);
	if (ret < 0)
		return ret;

	rtc_time_to_hw(rtc_data, &alm->time);

	/* update all the alarm registers in one shot */
	ret = regmap_bulk_write(mfd->regmap, BD71827_REG_ALM0_SEC, rtc_data, sizeof rtc_data);
	if (ret) {
		dev_err(dev, "rtc_set_alarm error %d\n", ret);
		return ret;
	}
	dev_info(dev, "%s, rtc alarm set to 20%02d-%02d-%02d %02d:%02d:%02d\n",__func__,
			bcd2bin(rtc_data->year), bcd2bin(rtc_data->month), bcd2bin(rtc_data->day),
			bcd2bin(HOUR_MASK(rtc_data->hour)), bcd2bin(rtc_data->min), bcd2bin(rtc_data->sec));

	ret = bd71827_rtc_alarm_irq_enable(dev, 1);
	if (ret < 0)
		return ret;

	return ret;
}

/**@brief bd71827 rtc alarm interrupt
 * @param irq system irq
 * @param rtc rtc device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 */
static irqreturn_t bd71827_rtc_interrupt(int irq, void *rtc)
{
	struct device *dev = rtc;
	unsigned long events = 0;
	struct bd71827 *mfd = dev_get_drvdata(dev->parent);
	struct bd71827_rtc *bd_rtc = dev_get_drvdata(dev);
	int ret;
	u32 rtc_reg;

	dev_info(mfd->dev, "bd71827_rtc_interrupt() in.\n");

	ret = regmap_read(mfd->regmap, BD71827_REG_INT_STAT_12, &rtc_reg);
	if (ret)
		return IRQ_NONE;

	dev_info(mfd->dev, "BD71827_REG_INT_STAT_12=0x%x\n", rtc_reg);

	if (rtc_reg & ALM0_EN)
		events = RTC_IRQF | RTC_AF;

	ret = regmap_write(mfd->regmap, BD71827_REG_INT_STAT_12, rtc_reg);
	if (ret)
		return IRQ_NONE;

	dev_info(mfd->dev, "\n~~~IRQ ALARM.\n");

	/* Notify RTC core on event */
	rtc_update_irq(bd_rtc->rtc, 1, events);

	return IRQ_HANDLED;
}

/** @brief function operations definition */
static struct rtc_class_ops bd71827_rtc_ops = {
	.read_time	= bd71827_rtc_read_time,
	.set_time	= bd71827_rtc_set_time,
	.read_alarm	= bd71827_rtc_read_alarm,
	.set_alarm	= bd71827_rtc_set_alarm,
	.alarm_irq_enable = bd71827_rtc_alarm_irq_enable,
};

/**@brief probe bd71827 rtc device
 @param pdev bd71827 rtc platform device
 @retval 0 success
 @retval negative fail
*/
static int bd71827_rtc_probe(struct platform_device *pdev)
{
	struct bd71827 *bd71827 = NULL;
	struct bd71827_rtc *bd_rtc = NULL;
	int ret;
	int irq;
	u32 rtc_reg;

	bd71827 = dev_get_drvdata(pdev->dev.parent);

	bd_rtc = devm_kzalloc(&pdev->dev, sizeof(struct bd71827_rtc),
			GFP_KERNEL);
	if (!bd_rtc)
		return -ENOMEM;

	/* Clear pending interrupts */
	ret = regmap_read(bd71827->regmap, BD71827_REG_INT_STAT_12, &rtc_reg);
	if (ret < 0)
		return ret;

	ret = regmap_write(bd71827->regmap, BD71827_REG_INT_STAT_12, rtc_reg);
	if (ret < 0)
		return ret;

	dev_info(&pdev->dev, "Enabling rtc-bd71827.\n");

	/*
	 * Figure 36. RTC Initialization
	 *
	 * PON=1? Yes
	 * CPU Power On
	 * Set SEC ~ YEAR2 register and TRIM register and
	 * ALM*_* register, etc.
	 * PON set to " 0"
	 */

	ret = regmap_read(bd71827->regmap, BD71827_REG_CONF, &rtc_reg);
	if (ret)
		return IRQ_NONE;

	/* Detect CPU Power On */
	if(rtc_reg & BD71827_REG_CONF_PON) {
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM0_SEC, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM0_MIN, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM0_HOUR, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM0_WEEK, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM0_DAY, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM0_MONTH, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM0_YEAR, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM1_SEC, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM1_MIN, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM1_HOUR, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM1_WEEK, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM1_DAY, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM1_MONTH, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM1_YEAR, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM0_MASK, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM1_MASK, 0);
		if (ret < 0)
			return ret;
		ret = regmap_write(bd71827->regmap, BD71827_REG_ALM2, 0);
		if (ret < 0)
			return ret;

		ret = regmap_write(bd71827->regmap, BD71827_REG_CONF, rtc_reg & ~BD71827_REG_CONF_PON);
		if (ret < 0)
			return ret;
	}

	irq  = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_warn(&pdev->dev, "Wake up is not possible as irq = %d\n", irq);
		return -ENXIO;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
		bd71827_rtc_interrupt, IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
		dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ is not free.\n");
		return ret;
	}
	bd_rtc->irq = irq;
	device_set_wakeup_capable(&pdev->dev, 1);

	bd_rtc->rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
		&bd71827_rtc_ops, THIS_MODULE);
	if (IS_ERR(bd_rtc->rtc)) {
		ret = PTR_ERR(bd_rtc->rtc);
		dev_err(&pdev->dev, "RTC device register: err %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, bd_rtc);

	{
		struct rtc_time tm;

		if(!bd71827_rtc_read_time(&pdev->dev, &tm))
			printk(KERN_INFO "bd71827-rtc probe time: %d:%d:%d-%d:%d:%d:\n", tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		else
			printk(KERN_INFO "bd71827-rtc probe read rtc time failed\n");

		rtc_tm_to_time(&tm, &wakeup_time);
	}
	return 0;
}

/*
 * Disable bd71827 RTC interrupts.
 * Sets status flag to free.
 */
/**@brief remove bd71827 rtc device
 @param pdev bd71827 rtc platform device
 @return 0
*/
static int bd71827_rtc_remove(struct platform_device *pdev)
{
	bd71827_rtc_alarm_irq_enable(&pdev->dev, 0);

	return 0;
}

/**@brief shutdown bd71827 rtc device
 @param pdev bd71827 rtc platform device
 @return void
*/
static void bd71827_rtc_shutdown(struct platform_device *pdev)
{
	/* mask timer interrupts, but leave alarm interrupts on to enable
	   power-on when alarm is triggered */
	bd71827_rtc_alarm_irq_enable(&pdev->dev, 0);
}

#ifdef CONFIG_PM_SLEEP
/**@brief suspend bd71827 rtc device
 * @param dev rtc device of system
 * @retval 0
 */
static int bd71827_rtc_suspend(struct device *dev)
{
	struct bd71827_rtc *bd_rtc = dev_get_drvdata(dev);
	struct rtc_time tm;

	if(!bd71827_rtc_read_time(dev, &tm))
	{
		suspend_mah = bd71827_get_battery_mah();

		rtc_tm_to_time(&tm, &suspend_time);
		if(suspend_time > wakeup_time && wakeup_time !=0)
		{
			total_wake_time += suspend_time-wakeup_time;
			printk(KERN_INFO "FRED total wake time is: %d \n", total_wake_time);
		}
	}
	else
		printk(KERN_ERR "FRED rtc suspend read rtc time failed\n");


	if (device_may_wakeup(dev))
		enable_irq_wake(bd_rtc->irq);
	return 0;
}

/**@brief resume bd71827 rtc device
 * @param dev rtc device of system
 * @retval 0
 */
static int bd71827_rtc_resume(struct device *dev)
{
	struct bd71827_rtc *bd_rtc = dev_get_drvdata(dev);
	struct rtc_time tm;

	if(!bd71827_rtc_read_time(dev, &tm))
	{
		rtc_tm_to_time(&tm, &wakeup_time);
		if(wakeup_time > suspend_time && suspend_time !=0)
		{
			total_suspend_time += wakeup_time - suspend_time;
			printk(KERN_INFO "FRED total suspend time is: %d \n", total_suspend_time);
		}
#ifdef CONFIG_AMAZON_METRICS_LOG
		wakeup_mah = bd71827_get_battery_mah();
		printk(KERN_ERR "FRED suspenddelta mah=%d\n", wakeup_mah-suspend_mah);
		memset(bd71827_rtc_metric_buf,0,sizeof(bd71827_rtc_metric_buf));
                snprintf(bd71827_rtc_metric_buf, sizeof(bd71827_rtc_metric_buf),
                        "kernel:pmic-bd71827:suspenddelta=%d;CT;1,suspendtime=%d;CT;1:NR",
                        wakeup_mah-suspend_mah, wakeup_time-suspend_time);

                log_to_metrics(ANDROID_LOG_INFO, "battery", bd71827_rtc_metric_buf);
#endif
	}
	else
		printk(KERN_ERR "FRED rtc wakeup read rtc time failed\n");

	if (device_may_wakeup(dev))
		disable_irq_wake(bd_rtc->irq);
	return 0;
}

unsigned long bd71827_total_suspend_time(void)
{
	return total_suspend_time;
}
EXPORT_SYMBOL(bd71827_total_suspend_time);

unsigned long bd71827_total_wake_time(void)
{
	return total_wake_time;
}
EXPORT_SYMBOL(bd71827_total_wake_time);


#endif

static SIMPLE_DEV_PM_OPS(bd71827_rtc_pm_ops, bd71827_rtc_suspend, bd71827_rtc_resume);

#if !defined(BD71827_NONE_DTB)
#ifdef CONFIG_OF
static const struct of_device_id bd71827_rtc_of_match[] = {
	{.compatible = "ti,bd71827-rtc", },
	{ },
};
MODULE_DEVICE_TABLE(of, bd71827_rtc_of_match);
#endif
#endif

static struct platform_driver bd71827rtc_driver = {
	.probe		= bd71827_rtc_probe,
	.remove		= bd71827_rtc_remove,
	.shutdown	= bd71827_rtc_shutdown,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "bd71827-rtc",
		.pm		= &bd71827_rtc_pm_ops,
#if !defined(BD71827_NONE_DTB)
		.of_match_table = of_match_ptr(bd71827_rtc_of_match),
#endif
	}
	,
};

/**@brief module initialize function */
static int __init bd71827_rtc_init(void)
{
	suspend_time = 0;
	wakeup_time = 0;
	total_suspend_time = 0;
	total_wake_time = 0;
	up_time = 0;
	suspend_mah = 0;
	wakeup_mah = 0;
	return platform_driver_register(&bd71827rtc_driver);
}
module_init(bd71827_rtc_init);

/**@brief module deinitialize function */
static void __exit bd71827_rtc_exit(void)
{
	platform_driver_unregister(&bd71827rtc_driver);
}
module_exit(bd71827_rtc_exit);

MODULE_AUTHOR("Cong Pham <cpham2403@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bd71827-rtc");
