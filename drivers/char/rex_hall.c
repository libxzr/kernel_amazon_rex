/*
 * Copyright (c) 2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/mmc/host.h>
#include <linux/device.h>
#include <linux/if.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/random.h>
#include <linux/lab126_hall.h>
#include <linux/sysctl.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>

#define HALL_DRIVER_NAME    "hall_sensor"


#define HALL_CLOSE_NORMAL_DELAY	(250)
#define HALL_CLOSE_MAX_DELAY	(3000)

#define HALL_EVENT_DEB      3

#define HALL_CTRL_DISABLE   0
#define HALL_CTRL_ENABLE    1

#define HALL_IRQ_TRIGGER (IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_EARLY_RESUME)

#undef HALL_INPUT_EVENT

/* Hall Sensor States */
typedef enum _hall_state {
	HALL_OPEN = 0,
	HALL_CLOSE
} hall_state_t;


struct hall_sysfs {
	unsigned int hall_timeout;
	unsigned int hall_dbg;
	unsigned int hall_enabled;
	unsigned int hall_test_trigger;
};

struct hall_drvdata {
	struct hall_platform_data* pdata;
	struct device *dev;
#ifdef HALL_INPUT_EVENT
	struct input_dev *input;
#endif
	struct work_struct hall_detwq;
	struct delayed_work hall_open_event_work;
	struct delayed_work hall_close_event_work;
	struct mutex hall_lock;
	hall_state_t hall_state;
	hall_state_t hall_event;
	struct hall_sysfs sysfs;
	unsigned int irq;
};

static int g_gpio_hall = -1;
static bool g_active_high = false;

#if defined(CONFIG_LAB126) && defined(CONFIG_TOI)
extern void cancel_toi_hibernation(void);
#endif

/*
 * hall sensor gpio value
 */
int gpio_hallsensor_detect(void)
{
	if (g_gpio_hall > 0) {
		return (g_active_high == !!gpio_get_value(g_gpio_hall));
	}

	return 0;
}

EXPORT_SYMBOL(gpio_hallsensor_detect);

static void hall_sensor_ctrl(int closed)
{
	if (g_gpio_hall > 0) {
		gpio_set_value(g_gpio_hall, closed);
	}
	return;
}

static ssize_t hall_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct hall_drvdata *drv = platform_get_drvdata(pdev);
	struct hall_sysfs *sysfs = &drv->sysfs;
	int value = 0;
	if (sscanf(buf, "%d", &value) <= 0) {
		dev_err(dev, "Could not update hall debug ctrl \n");
		return -EINVAL;
	}
	sysfs->hall_dbg = (value > 0) ? 1 : 0;
	return size;
}

static ssize_t hall_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct hall_drvdata *drv = platform_get_drvdata(pdev);
	struct hall_sysfs *sysfs = &drv->sysfs;
	return sprintf(buf, "%d\n", sysfs->hall_dbg);
}
static DEVICE_ATTR(hall_debug, 0644, hall_debug_show, hall_debug_store);

static ssize_t hall_detect_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct hall_drvdata *drv = platform_get_drvdata(pdev);
	return sprintf(buf, "%d\n", drv->hall_state);
}
static DEVICE_ATTR(hall_detect, 0444, hall_detect_show, NULL);

static ssize_t hall_hw_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_hallsensor_detect());
}
static DEVICE_ATTR(hall_hw_state, 0444, hall_hw_state_show, NULL);

static ssize_t hall_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct hall_drvdata *drv = platform_get_drvdata(pdev);
	struct hall_sysfs *sysfs = &drv->sysfs;
	int value = 0;
	if (sscanf(buf, "%d", &value) <= 0) {
		dev_err(dev, "Could not update hall enable ctrl \n");
		return -EINVAL;
	}

	if (value > 0 && !sysfs->hall_enabled) {
		sysfs->hall_enabled = HALL_CTRL_ENABLE;
	} else if(value <= 0 && sysfs->hall_enabled) {
		sysfs->hall_enabled = HALL_CTRL_DISABLE;
	}
	return size;
}
static ssize_t hall_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct hall_drvdata *drv = platform_get_drvdata(pdev);
	struct hall_sysfs *sysfs = &drv->sysfs;
	return sprintf(buf, "%d\n", sysfs->hall_enabled);
}
static DEVICE_ATTR(hall_enable, 0644, hall_enable_show, hall_enable_store);


static ssize_t hall_timeout_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct hall_drvdata *drv = platform_get_drvdata(pdev);
	struct hall_sysfs *sysfs = &drv->sysfs;
	return sprintf(buf, "%d secs\n", sysfs->hall_timeout);
}


static ssize_t hall_timeout_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct hall_drvdata *drv = platform_get_drvdata(pdev);
	int value = 0;
	if (sscanf(buf, "%d", &value) <= 0) {
		dev_err(dev, "Could not update hall timeout  ctrl \n");
		return -EINVAL;
	}

	drv->sysfs.hall_timeout = value;
	return size;
}

static DEVICE_ATTR(hall_timeout, 0644, hall_timeout_show, hall_timeout_store);


static ssize_t hall_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct hall_drvdata *drv = platform_get_drvdata(pdev);
	struct hall_sysfs *sysfs = &drv->sysfs;
	return sprintf(buf, "Triggered %s\n", (sysfs->hall_test_trigger ? "Close" : "Open"));
}


static ssize_t hall_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct hall_drvdata *drv = platform_get_drvdata(pdev);
	int value = 0;
	if (sscanf(buf, "%d", &value) <= 0) {
		dev_err(dev, "Could not update hall timeout  ctrl \n");
		return -EINVAL;
	}

	drv->sysfs.hall_test_trigger = value;
	hall_sensor_ctrl(value);
	return size;
}

static DEVICE_ATTR(hall_test_trigger, 0644, hall_test_show, hall_test_store);



static int hall_sysfs_create(struct platform_device *pdev)
{
	int ret = 0;
	struct hall_drvdata *drv = platform_get_drvdata(pdev);
	struct hall_sysfs *sysfs = &drv->sysfs;

	sysfs->hall_enabled = HALL_CTRL_ENABLE;
	sysfs->hall_timeout = HALL_EVENT_DEB;
	sysfs->hall_dbg = 0;

        if (device_create_file(&pdev->dev, &dev_attr_hall_detect))
	{
                dev_err(&pdev->dev, "Unable to create file: hall_detect\n");
		ret = -EFAULT;
		goto err_detect;
	}

        if (device_create_file(&pdev->dev, &dev_attr_hall_hw_state))
	{
                dev_err(&pdev->dev, "Unable to create file: hall_hw_state\n");
		ret = -EFAULT;
		goto err_detect;
	}

        if (device_create_file(&pdev->dev, &dev_attr_hall_enable))
	{
                dev_err(&pdev->dev, "Unable to create file: hall_enable\n");
		ret= -EFAULT;
		goto err_enable;
	}

        if (device_create_file(&pdev->dev, &dev_attr_hall_debug))
	{
                dev_err(&pdev->dev, "Unable to create file: hall_debug\n");
		ret = -EFAULT;
		goto err_debug;
	}

        if (device_create_file(&pdev->dev, &dev_attr_hall_timeout))
	{
                dev_err(&pdev->dev, "Unable to create file: hall_timeout\n");
		ret = -EFAULT;
		goto err_timeout;
	}

        if (device_create_file(&pdev->dev, &dev_attr_hall_test_trigger))
	{
                dev_err(&pdev->dev, "Unable to create file: hall_test_trigger\n");
		ret = -EFAULT;
		goto err_trigger;
	}

	return ret;
err_trigger:
	device_remove_file(&pdev->dev, &dev_attr_hall_timeout);
err_timeout:
	device_remove_file(&pdev->dev, &dev_attr_hall_debug);
err_debug:
	device_remove_file(&pdev->dev, &dev_attr_hall_enable);
err_enable:
	device_remove_file(&pdev->dev, &dev_attr_hall_detect);
err_detect:
	return ret;

}

static void hall_sysfs_destroy(struct platform_device *pdev)
{

	device_remove_file(&pdev->dev, &dev_attr_hall_detect);
	device_remove_file(&pdev->dev, &dev_attr_hall_timeout);
	device_remove_file(&pdev->dev, &dev_attr_hall_debug);
	device_remove_file(&pdev->dev, &dev_attr_hall_enable);
	return;
}

static void hall_send_close_event(struct hall_drvdata *drv)
{
	char *envp[] = {"HALLSENSOR=closed", NULL};

	drv->hall_state = HALL_CLOSE;
	dev_info(drv->dev, "KERNEL: send event: close(%d)\n", drv->hall_state);
	kobject_uevent_env(&drv->dev->kobj, KOBJ_ONLINE, envp);
}

static void hall_send_open_event(struct hall_drvdata *drv)
{
	char *envp[] = {"HALLSENSOR=opened", NULL};

#if defined(CONFIG_LAB126) && defined(CONFIG_TOI)
			cancel_toi_hibernation();
#endif
	drv->hall_state = HALL_OPEN;
	dev_info(drv->dev, "KERNEL: send event: open(%d)\n", drv->hall_state);
	kobject_uevent_env(&drv->dev->kobj, KOBJ_OFFLINE, envp);
}

static void hall_open_work_fun(struct work_struct *work)
{
	return;
}

static void hall_close_work_fun(struct work_struct *work)
{
	struct hall_drvdata *drv = container_of(work, struct hall_drvdata, hall_close_event_work.work);

	hall_send_close_event(drv);

	if (drv->sysfs.hall_dbg)
		dev_info(drv->dev, "KERNEL: I hall:close event:::current_state=%d\n", gpio_hallsensor_detect());
}

static void hall_detwq_handler(struct work_struct *work)
{
	struct hall_drvdata *drv = container_of(work, struct hall_drvdata, hall_detwq);
#ifdef HALL_INPUT_EVENT
	struct input_dev *input = drv->input;
#endif
	int irq = drv->irq;
	struct hall_sysfs *sysfs = &drv->sysfs;

	if (sysfs->hall_enabled) {
		drv->hall_event = gpio_hallsensor_detect();
		dev_dbg(drv->dev, "KERNEL: Sensor event : %d hall state %d\n", drv->hall_event, drv->hall_state);

#ifdef HALL_INPUT_EVENT
		input_report_switch(input, SW_LID, drv->hall_event);
		input_sync(input);
#endif

		cancel_delayed_work_sync(&drv->hall_close_event_work);
		if (drv->hall_event == HALL_OPEN) {
			/* hall open */
			if (sysfs->hall_dbg)
				dev_info(drv->dev, "KERNEL: I hall:opened:::current_state=%d\n", drv->hall_event);

			if (!cancel_delayed_work_sync(&drv->hall_open_event_work)) {
				hall_send_open_event(drv);

				if (sysfs->hall_dbg)
					dev_info(drv->dev, "KERNEL: I hall:open event:::current_state=%d\n", drv->hall_event);
			}
			/* schedule a delayed work to drop all events in HALL_CLOSE_MAX_DELAY ms */
			schedule_delayed_work(&drv->hall_open_event_work, msecs_to_jiffies(HALL_CLOSE_MAX_DELAY));
			goto done;
		} else {
			/* hall close */
			if (sysfs->hall_dbg)
				dev_info(drv->dev, "KERNEL: I hall:closed:::current_state=%d\n", drv->hall_event);

			if (delayed_work_pending(&drv->hall_open_event_work)) {
				schedule_delayed_work(&drv->hall_close_event_work, msecs_to_jiffies(HALL_CLOSE_MAX_DELAY));
			} else {
				schedule_delayed_work(&drv->hall_close_event_work, msecs_to_jiffies(HALL_CLOSE_NORMAL_DELAY));
			}
		}
	}
done:
	enable_irq(irq);

	return;
}

static irqreturn_t hall_isr(int irq, void *dev_id)
{
	struct hall_drvdata *drv = (struct hall_drvdata *)dev_id;
	disable_irq_nosync(irq);
	schedule_work(&drv->hall_detwq);

	return IRQ_HANDLED;
}

#ifdef CONFIG_PROC_FS
#define HALL_PROC_FILE "driver/hall_sensor"
static int hall_show(struct seq_file *m, void *v)
{
        seq_printf(m, "0x%x\n", !gpio_hallsensor_detect());
        return 0;
}

static int hall_open(struct inode *inode, struct file *file)
{
        return single_open(file, hall_show, PDE_DATA(inode));
}

static const struct file_operations proc_fops = {
        .owner = THIS_MODULE,
        .open = hall_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

static void create_hall_proc_file(struct hall_drvdata *priv)
{
        struct proc_dir_entry *entry;
        entry = proc_create_data(HALL_PROC_FILE, 0644, NULL, &proc_fops, priv);
        if (!entry)
                pr_err("%s: Error creating %s\n", __func__, HALL_PROC_FILE);
}

static void remove_hall_proc_file(void)
{
        remove_proc_entry(HALL_PROC_FILE, NULL);
}
#endif

static int rex_hall_probe(struct platform_device *pdev)
{
	int irq;
	int ret = 0;
	struct device_node *np;
	struct hall_drvdata* ddata;
	struct hall_platform_data *pdata = pdev->dev.platform_data;
#ifdef HALL_INPUT_EVENT
	struct input_dev *input;
#endif

	ddata = kzalloc(sizeof(struct hall_drvdata), GFP_KERNEL);
	if (!ddata) {
		dev_err(&pdev->dev, "ddata error\n");
		ret = -ENOMEM;
		goto err;
	}
	platform_set_drvdata(pdev, ddata);
	ddata->pdata = pdata;
	ddata->dev = &pdev->dev;

	INIT_WORK(&ddata->hall_detwq, hall_detwq_handler);
	INIT_DELAYED_WORK(&ddata->hall_open_event_work, hall_open_work_fun);
	INIT_DELAYED_WORK(&ddata->hall_close_event_work, hall_close_work_fun);
	mutex_init(&ddata->hall_lock);

	np = of_find_compatible_node(NULL, NULL, "amzn,rex_hall");
	if (!np) {
		dev_err(&pdev->dev, "%s: invalid pnode\n", __func__);
		ret = -EINVAL;
		goto err_free_ddata;
	}

	g_active_high = of_property_read_bool(np, "enable-active-high");

	ret = of_get_named_gpio(np, "hall_int_n", 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: get wl_host_wake GPIO failed err=%d\n", __func__, ret);
		ret = -EINVAL;
		goto err_free_ddata;
	}

	g_gpio_hall = ret;

	ret = gpio_request(g_gpio_hall, "hall_irq");
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to request gpio %d for hall err=%d\n", __func__, g_gpio_hall, ret);
		ret = -EINVAL;
		goto err_free_ddata;
	}

	ret = gpio_direction_input(g_gpio_hall);
	if (ret) {
		dev_err(&pdev->dev, "%s: configuration failure err=%d\n", __func__, ret);
		ret = -EINVAL;
		goto err_free_gpio;
	}

	/* Out Of Band interrupt */
	irq = gpio_to_irq(g_gpio_hall);
	if (irq < 0) {
		dev_err(&pdev->dev, "%s: irq failure\n", __func__);
		ret = -EINVAL;
		goto err_free_gpio;
	}

	ddata->irq = irq;

	ret = request_irq(irq, hall_isr, HALL_IRQ_TRIGGER, HALL_DRIVER_NAME, ddata);
	if (ret) {
		dev_err(&pdev->dev, "%s Failed to claim irq %d, error %d \n",__FUNCTION__, irq, ret);
		ret = -EFAULT;
		goto err_free_gpio;
	}

#ifdef HALL_INPUT_EVENT
	input = input_allocate_device();
	if (!input) {
		dev_err(&pdev->dev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_free_irq;
	}

	input->name = pdev->name;
	input->dev.parent = &pdev->dev;
	set_bit(EV_SW, input->evbit);
	set_bit(SW_LID, input->swbit);
	ret = input_register_device(input);

	if (ret) {
		dev_err(&pdev->dev, "Failed to register input dev, err=%d\n", ret);
		ret = -EFAULT;
		goto err_free_input;
	}

	ddata->input = input;
#endif

	/* Configure wakeup capable */
	device_set_wakeup_capable(&pdev->dev, true);
	device_set_wakeup_enable(&pdev->dev, true);
	enable_irq_wake(ddata->irq);

	if (hall_sysfs_create(pdev) < 0) {
                dev_err(&pdev->dev, "Unable to create file: hall_detect\n");
		ret = -EFAULT;
		goto err_dereg_input;
	}


#ifdef CONFIG_PROC_FS
        create_hall_proc_file(ddata);
#endif
	return 0;

err_dereg_input:
	device_set_wakeup_capable(&pdev->dev, false);
#ifdef HALL_INPUT_EVENT
	input_unregister_device(input);
err_free_input:
	input_free_device(input);
err_free_irq:
#endif
	disable_irq_wake(pdata->hall_irq);
	free_irq(pdata->hall_irq, pdev->dev.platform_data);
err_free_gpio:
	gpio_free(g_gpio_hall);
err_free_ddata:
	platform_set_drvdata(pdev, NULL);
	kfree(ddata);
err:
	return ret;
}

static int rex_hall_suspend(struct device *dev)
{
	struct hall_drvdata* ddata = dev_get_drvdata(dev);

	/* disable raising irq */
	irq_set_irq_type(ddata->irq, (HALL_IRQ_TRIGGER & ~(g_active_high ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING)));
	enable_irq_wake(ddata->irq);

	return 0;
}

static int rex_hall_resume(struct device *dev)
{
	struct hall_drvdata* ddata = dev_get_drvdata(dev);

	/* Update the hall_state based on the HW status after resume */
	ddata->hall_state = gpio_hallsensor_detect();
	irq_set_irq_type(ddata->irq, HALL_IRQ_TRIGGER);

	return 0;
}

static int rex_hall_remove(struct platform_device *pdev)
{
	struct hall_platform_data *pdata = pdev->dev.platform_data;
	struct hall_drvdata *priv = platform_get_drvdata(pdev);

	hall_sysfs_destroy(pdev);

//	device_remove_file(&pdev->dev, &dev_attr_hall_trig_wkup);
#ifdef CONFIG_PROC_FS
	remove_hall_proc_file();
#endif
	cancel_delayed_work_sync(&priv->hall_open_event_work);
	cancel_delayed_work_sync(&priv->hall_close_event_work);
	cancel_work_sync(&priv->hall_detwq);

#ifdef HALL_INPUT_EVENT
	input_unregister_device(priv->input);
	input_free_device(priv->input);
#endif
	device_set_wakeup_capable(&pdev->dev, false);
	disable_irq_wake(pdata->hall_irq);
	free_irq(pdata->hall_irq, pdev->dev.platform_data);
	platform_set_drvdata(pdev, NULL);
	kfree(priv);
	return 0;
}

static SIMPLE_DEV_PM_OPS(rex_all_pm_ops, rex_hall_suspend, rex_hall_resume);

static const struct of_device_id hall_of_match[] = {
        { .compatible = "amzn,rex_hall", },
        {},
};

static struct platform_driver rex_hall_driver = {
	.driver		= {
		.name	= HALL_DRIVER_NAME,
		.owner	= THIS_MODULE,
                .of_match_table = of_match_ptr(hall_of_match),
        .pm		= &rex_all_pm_ops,

	},
	.probe		= rex_hall_probe,
	.remove		= rex_hall_remove,
};

static int __init rex_hall_init(void)
{
	platform_driver_register(&rex_hall_driver);
	return 0;
}

static void __exit rex_hall_exit(void)
{
	platform_driver_unregister(&rex_hall_driver);
}

module_init(rex_hall_init);
module_exit(rex_hall_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("lab126");
MODULE_DESCRIPTION("Hall sensor driver for rex platform");
