/*
 * Copyright 2017 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include "util_driver.h"
#include <linux/busfreq-imx.h>

/* Module information */
MODULE_AUTHOR(AUTHOR);
MODULE_DESCRIPTION(DESCRIPTION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");

/* Device variables */
static struct class* util_class = NULL;
static struct device* util_device = NULL;
static int major;
/* A mutex will ensure that only one process accesses our device */
static DEFINE_MUTEX(device_mutex);

/* Module parameters that can be provided on insmod */
static bool debug = false;	/* print extra debug info */
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "enable debug info (default: false)");

static int device_open(struct inode* inode, struct file* filp)
{
	dbgprnt("");

	/*no write access */
	if ( ((filp->f_flags & O_ACCMODE) == O_WRONLY)
	  || ((filp->f_flags & O_ACCMODE) == O_RDWR) ) {
		warn("write access is prohibited\n");
		return -EACCES;
	}

	/* Ensure that only one process has access to our device at any one time
	 * For more info on concurrent accesses, see http://lwn.net/images/pdf/LDD3/ch05.pdf */
	if (!mutex_trylock(&device_mutex)) {
		warn("another process is accessing the device\n");
		return -EBUSY;
	}

	return 0;
}

static int device_close(struct inode* inode, struct file* filp)
{
	dbgprnt("");
	mutex_unlock(&device_mutex);
	return 0;
}


/*
  * The file_operation scructure
  */
static struct file_operations fops = {
	.open = device_open,
	.release = device_close
};

static long ibusfreq = 0;
static ssize_t show_busfreq(struct device *dev, struct device_attribute *attr,
                         char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "busfreq= %ld\n", ibusfreq);
}

static ssize_t store_busfreq(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    int ret = 0;
	ret = kstrtol(buf, 10, &ibusfreq);
        if(0 != ret) {
            dbgprnt("Error to call dbgprnt, re==%d", ret);
            return 0;
        }
	switch(ibusfreq) {
		case 4:
			request_bus_freq(BUS_FREQ_HIGH);
			break;
		case 3:
			release_bus_freq(BUS_FREQ_HIGH);
			break;
		case 2:
			request_bus_freq(BUS_FREQ_LOW);
			break;
		case 1:
			release_bus_freq(BUS_FREQ_LOW);
			break;

		default:
			break;
	}
//        snprintf(dev->name, sizeof(dev->name), "%.*s",
//                 (int)min(count, sizeof(dev->name) - 1), buf);

	return count;
}


static DEVICE_ATTR(busfreq, S_IWUSR | S_IRUGO, show_busfreq, store_busfreq);


/* Module initialization and release */
static int __init util_init(void)
{
	int retval;
	dbgprnt("");

	/* dynamically allocate a major for our device */
	major = register_chrdev(0, DEVICE_NAME, &fops);
	if (major < 0) {
		err("failed to register device: error %d\n", major);
		retval = major;
		goto failed_chrdevreg;
	}

	/* either tie our device to a bus (existing, or one that we create)
	 * or use a "virtual" device class. */
	util_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(util_class)) {
		err("failed to register device class '%s'\n", CLASS_NAME);
		retval = PTR_ERR(util_class);
		goto failed_classreg;
	}

	/* With a class, the easiest way to instantiate a device is to call device_create() */
	util_device = device_create(util_class, NULL, MKDEV(major, 0), NULL, CLASS_NAME "_" DEVICE_NAME);
	if (IS_ERR(util_device)) {
		err("failed to create device '%s_%s'\n", CLASS_NAME, DEVICE_NAME);
		retval = PTR_ERR(util_device);
		goto failed_devreg;
	}

	/* create the sysfs endpoints ). */
	retval = device_create_file(util_device, &dev_attr_busfreq);
	if (retval < 0) {
		warn("failed to create write /sys endpoint - continuing without\n");
	}

	mutex_init(&device_mutex);

	return 0;

failed_devreg:
	class_destroy(util_class);
failed_classreg:
	unregister_chrdev(major, DEVICE_NAME);
failed_chrdevreg:
	return -1;
}

static void __exit util_exit(void)
{
	dbgprnt("");
	device_remove_file(util_device, &dev_attr_busfreq);
	device_destroy(util_class, MKDEV(major, 0));
	class_destroy(util_class);
	unregister_chrdev(major, DEVICE_NAME);
}

/* Let the kernel know the calls for module init and exit */
module_init(util_init);
module_exit(util_exit);
