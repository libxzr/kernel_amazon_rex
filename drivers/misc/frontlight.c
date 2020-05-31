/*
 * Copyright (c) 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/frontlight.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/mxcfb.h>
#include <linux/delay.h>

#define FRONTLIGHT_DRIVER_NAME	"frontlight"
#define FRONTLIGHT_MINOR	162

static DEFINE_MUTEX(core_lock);
struct backlight_device *backlight_device;
static struct miscdevice frontlight_misc_device;
static int ui_brightness = -1;

int frontlight_set_brightness(int brightness, int from_ui)
{
	int ret = -EINVAL;
	mutex_lock(&backlight_device->ops_lock);
	if (backlight_device->ops) {
		if (unlikely(brightness > backlight_device->props.max_brightness)) {
			dev_err(frontlight_misc_device.this_device, "brightness larger than max: %u\n", brightness);
			ret = -EINVAL;
		} else {
			dev_dbg(frontlight_misc_device.this_device, "set brightness to %u\n", brightness);
			backlight_device->props.brightness = brightness;
			backlight_update_status(backlight_device);
			ret = 0;
			if(from_ui)
				ui_brightness = backlight_device->props.brightness;
		}
	}
	mutex_unlock(&backlight_device->ops_lock);
	return ret;
}
EXPORT_SYMBOL(frontlight_set_brightness);

int get_ui_brightness(void)
{
	if(unlikely(ui_brightness == -1))
		return 0;
	return ui_brightness;
}
EXPORT_SYMBOL(get_ui_brightness);


static long frontlight_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int __user *argp = (int __user *)arg;
	int ret = -EINVAL;
	int brightness = 0;

	if (!unlikely(backlight_device))
		return -ENODEV;

	switch (cmd) {
		case FL_IOCTL_SET_INTENSITY_FORCED:
		case FL_IOCTL_SET_INTENSITY:
			if (get_user(brightness, argp))
				ret = -EFAULT;
			else {
				ret = frontlight_set_brightness(brightness, 1);
			}
			break;
		case FL_IOCTL_GET_INTENSITY:
			if (put_user(backlight_device->props.brightness, argp))
				ret = -EFAULT;
			else
				ret = 0;
			break;
		case FL_IOCTL_GET_RANGE_MAX:
			if (put_user(backlight_device->props.max_brightness, argp))
				ret = -EFAULT;
			else
				ret = 0;
			break;
		default:
			break;
	}
	return ret;
}

static const struct file_operations frontlight_misc_fops =
{
	.owner = THIS_MODULE,
	.unlocked_ioctl = frontlight_ioctl,
};

static struct miscdevice frontlight_misc_device =
{
	.minor = FRONTLIGHT_MINOR,
	.name  = FRONTLIGHT_DRIVER_NAME,
	.fops  = &frontlight_misc_fops,
};

int frontlight_register(struct backlight_device *device)
{
	int ret;

	ret = 0;

	if (!unlikely(device))
		return -EINVAL;

	mutex_lock(&core_lock);
	if (unlikely(backlight_device)) {
		dev_err(frontlight_misc_device.this_device, "another device (%s:%s) already registered.",
				dev_driver_string(&backlight_device->dev), dev_name(&backlight_device->dev));
		ret = -EBUSY;
	} else {
		backlight_device = device;
		dev_info(frontlight_misc_device.this_device, "new frontlight device (%s:%s) registered.",
				dev_driver_string(&backlight_device->dev), dev_name(&backlight_device->dev));
	}
	mutex_unlock(&core_lock);

	return ret;
}
EXPORT_SYMBOL(frontlight_register);

static int __init frontlight_dev_init(void)
{
	int res;

	printk(KERN_INFO "frontlight /dev entries driver\n");
	res = misc_register(&frontlight_misc_device);
	if (res) {
		pr_err("misc frontlight: Coulnd't register device %d.\n", frontlight_misc_device.minor);
		goto out;
	}

	return 0;
out:
	printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);
	return res;
}

static void __exit frontlight_dev_exit(void)
{
	misc_deregister(&frontlight_misc_device);
}

MODULE_AUTHOR("");
MODULE_DESCRIPTION("Frontlight /dev entries driver");
MODULE_LICENSE("GPL");

module_init(frontlight_dev_init);
module_exit(frontlight_dev_exit);
