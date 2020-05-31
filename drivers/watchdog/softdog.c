/*
 *	SoftDog:	A Software Watchdog Device
 *
 *	(c) Copyright 1996 Alan Cox <alan@lxorguk.ukuu.org.uk>,
 *							All Rights Reserved.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	Neither Alan Cox nor CymruNet Ltd. admit liability nor provide
 *	warranty for any of this software. This material is provided
 *	"AS-IS" and at no charge.
 *
 *	(c) Copyright 1995    Alan Cox <alan@lxorguk.ukuu.org.uk>
 *
 *	Software only watchdog driver. Unlike its big brother the WDT501P
 *	driver this won't always recover a failed machine.
 *
 *  03/96: Angelo Haritsis <ah@doc.ic.ac.uk> :
 *	Modularised.
 *	Added soft_margin; use upon insmod to change the timer delay.
 *	NB: uses same minor as wdt (WATCHDOG_MINOR); we could use separate
 *	    minors.
 *
 *  19980911 Alan Cox
 *	Made SMP safe for 2.3.x
 *
 *  20011127 Joel Becker (jlbec@evilplan.org>
 *	Added soft_noboot; Allows testing the softdog trigger without
 *	requiring a recompile.
 *	Added WDIOC_GETTIMEOUT and WDIOC_SETTIMOUT.
 *
 *  20020530 Joel Becker <joel.becker@oracle.com>
 *	Added Matt Domsch's nowayout module option.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/watchdog.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>

#define TIMER_MARGIN	60		/* Default is 60 seconds */
static unsigned int soft_margin = TIMER_MARGIN;	/* in seconds */
module_param(soft_margin, uint, 0);
MODULE_PARM_DESC(soft_margin,
	"Watchdog soft_margin in seconds. (0 < soft_margin < 65536, default="
					__MODULE_STRING(TIMER_MARGIN) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		"Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int soft_noboot;
module_param(soft_noboot, int, 0);
MODULE_PARM_DESC(soft_noboot,
	"Softdog action, set to 1 to ignore reboots, 0 to reboot (default=0)");

static int soft_panic;
module_param(soft_panic, int, 0);
MODULE_PARM_DESC(soft_panic,
	"Softdog action, set to 1 to panic, 0 to reboot (default=0)");

/*
 *	Our timer
 */

static void watchdog_fire(unsigned long);

static struct timer_list watchdog_ticktock =
		TIMER_INITIALIZER(watchdog_fire, 0, 0);

/*
 *	If the timer expires..
 */


#define SYS_LOG_FILE "/var/log/messages"
#define SYS_LOG_DUMP_SIZE 5*1024
#define DUMP_BUF_SIZE 256
char buf[DUMP_BUF_SIZE+1];

static void dump_sys_log(void)
{
	struct file *filp, *filp_dump;
	long file_size, read_size, size_to_read, offset_to_read, offset_to_write, write_size;
	int i;
//	void *buf;
	filp = filp_open(SYS_LOG_FILE, O_RDONLY|O_LARGEFILE, 0);

        if (IS_ERR(filp))
        {
                int ret = PTR_ERR(filp);
                pr_crit("Unable to open'%s (err=%d)'.\n", SYS_LOG_FILE, ret);
                return;
        }
#ifdef SOFTDOG_DUMP_TO_FILE
#define SYS_LOG_SOFTDOG_DUMP "/var/local/softdog_syslog_dump"
	filp_dump = filp_open(SYS_LOG_SOFTDOG_DUMP, O_CREAT|O_WRONLY|O_LARGEFILE, 0);

        if (IS_ERR(filp_dump))
        {
                int ret = PTR_ERR(filp_dump);
                pr_crit("Unable to open'%s (err=%d)'.\n", SYS_LOG_SOFTDOG_DUMP, ret);
                return;
        }
#endif
        file_size = i_size_read(file_inode(filp));
        if ( file_size<=0 ) {
                pr_crit("%s: invalid file size %ld\n",__func__,file_size);
                filp_close(filp,0);
                return;
        }
	size_to_read = file_size > SYS_LOG_DUMP_SIZE? SYS_LOG_DUMP_SIZE:file_size;
	offset_to_read = file_size > SYS_LOG_DUMP_SIZE ? file_size - SYS_LOG_DUMP_SIZE : 0;

        pr_crit("%s: open file (%s) ok size=%ld\n",__func__,SYS_LOG_FILE,file_size);

/*
        buf = vmalloc(2048);
	memset(buf,0,DUMP_BUF_SIZE+1);
        if (buf == NULL) {
                pr_crit("%s: can't allocate memory\n",__func__);
                return;
        }
*/
	pr_crit("SYS_LOG_DUMP Begin======\n");
	offset_to_write = 0;
	for(i=0;i<SYS_LOG_DUMP_SIZE/DUMP_BUF_SIZE;i++) { //has to dump in small buffer, otherwise system would crash
		memset(buf,0,DUMP_BUF_SIZE+1);
		//read_size = kernel_read(filp, offset_to_read, buf, size_to_read);
		size_to_read=DUMP_BUF_SIZE;
		read_size = kernel_read(filp, offset_to_read+i*size_to_read, buf, size_to_read);
		if (read_size != size_to_read) {
			pr_crit( "%s: read error (got %ld)(expect %ld)\n",__func__,read_size,size_to_read);
			return;
		}
#ifdef SOFTDOG_DUMP_TO_FILE
		write_size = kernel_write(filp_dump, offset_to_write+i*size_to_read, buf, size_to_read);
		if (write_size != size_to_read) {
			pr_crit( "%s: write error (write %ld)(expect %ld)\n",__func__,write_size,size_to_read);
			return;
		}
#endif
		pr_crit("%s",buf);
	}
	pr_crit("SYS_LOG_DUMP Done======\n");
        //fput(filp);
        filp_close(filp, 0);
#ifdef SOFTDOG_DUMP_TO_FILE
        filp_close(filp_dump, 0);
#endif
//	vfree(buf);
	return;
}
static void watchdog_fire(unsigned long data)
{
	if (soft_noboot)
		pr_crit("Triggered - Reboot ignored\n");
	else if (soft_panic) {
		pr_crit("Initiating panic\n");
		panic("Software Watchdog Timer expired");
	} else {
		pr_crit("Initiating system reboot\n");
		dump_sys_log();
		emergency_restart();
		pr_crit("Reboot didn't ?????\n");
	}
}

/*
 *	Softdog operations
 */

static int softdog_ping(struct watchdog_device *w)
{
	mod_timer(&watchdog_ticktock, jiffies+(w->timeout*HZ));
	return 0;
}

static int softdog_stop(struct watchdog_device *w)
{
	del_timer(&watchdog_ticktock);
	return 0;
}

static int softdog_set_timeout(struct watchdog_device *w, unsigned int t)
{
	w->timeout = t;
	return 0;
}

/*
 *	Notifier for system down
 */

static int softdog_notify_sys(struct notifier_block *this, unsigned long code,
	void *unused)
{
	if (code == SYS_DOWN || code == SYS_HALT)
		/* Turn the WDT off */
		softdog_stop(NULL);
	return NOTIFY_DONE;
}

/*
 *	Kernel Interfaces
 */

static struct notifier_block softdog_notifier = {
	.notifier_call	= softdog_notify_sys,
};

static struct watchdog_info softdog_info = {
	.identity = "Software Watchdog",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static struct watchdog_ops softdog_ops = {
	.owner = THIS_MODULE,
	.start = softdog_ping,
	.stop = softdog_stop,
	.set_timeout = softdog_set_timeout,
};

static struct watchdog_device softdog_dev = {
	.info = &softdog_info,
	.ops = &softdog_ops,
	.min_timeout = 1,
	.max_timeout = 0xFFFF
};

static int __init watchdog_init(void)
{
	int ret;

	/* Check that the soft_margin value is within it's range;
	   if not reset to the default */
	if (soft_margin < 1 || soft_margin > 65535) {
		pr_info("soft_margin must be 0 < soft_margin < 65536, using %d\n",
			TIMER_MARGIN);
		return -EINVAL;
	}
	softdog_dev.timeout = soft_margin;

	watchdog_set_nowayout(&softdog_dev, nowayout);

	ret = register_reboot_notifier(&softdog_notifier);
	if (ret) {
		pr_err("cannot register reboot notifier (err=%d)\n", ret);
		return ret;
	}

	ret = watchdog_register_device(&softdog_dev);
	if (ret) {
		unregister_reboot_notifier(&softdog_notifier);
		return ret;
	}

	pr_info("Software Watchdog Timer: 0.08 initialized. soft_noboot=%d soft_margin=%d sec soft_panic=%d (nowayout=%d)\n",
		soft_noboot, soft_margin, soft_panic, nowayout);

	return 0;
}

static void __exit watchdog_exit(void)
{
	watchdog_unregister_device(&softdog_dev);
	unregister_reboot_notifier(&softdog_notifier);
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("Alan Cox");
MODULE_DESCRIPTION("Software Watchdog Device Driver");
MODULE_LICENSE("GPL");
