/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/version.h>

#ifdef CONFIG_FALCON_MTD_NOR
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/cfi.h>
#endif

#include <linux/bitops.h>

#include <asm/falcon_syscall.h>

static wait_queue_head_t *storage_wait_queue;
static int *wake_up_cond;

static void falcon_wakeup_task(void);

static unsigned long need_wakeup;

void init_storage_common(void)
{
}

static void falcon_wakeup_task(void)
{
	if (!storage_wait_queue || !wake_up_cond)
		return;

	if (*wake_up_cond == 1)
		return;

	if (preempt_count())

		test_and_set_bit(0, &need_wakeup);
	else {
		*wake_up_cond = 1;
		wake_up(storage_wait_queue);
	}
}

void falcon_delayed_wakeup(void)
{
	if (test_and_clear_bit(0, &need_wakeup) == 0)
		return;

	BUG_ON(!storage_wait_queue || !wake_up_cond);

	*wake_up_cond = 1;
	wake_up(storage_wait_queue);
}

void falcon_set_wait_queue(wait_queue_head_t *q, int *cond)
{
	storage_wait_queue = q;
	wake_up_cond = cond;

	if (q && cond)
		set_falcon_callback(falcon_wakeup_task);
	else
		set_falcon_callback(NULL);
}

int falcon_storage_suspend(void)
{
	return 0;
}

void falcon_storage_resume(void)
{
	if (storage_wait_queue != NULL)
		wake_up(storage_wait_queue);

	need_wakeup = 0;

}
