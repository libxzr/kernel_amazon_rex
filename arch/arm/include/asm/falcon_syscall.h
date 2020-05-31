/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _ASM_ARM_FALCON_SYSCALL_H
#define _ASM_ARM_FALCON_SYSCALL_H

#ifdef __KERNEL__

#include <linux/version.h>
#include <linux/suspend.h>

#define bios_svc(num, arg0, arg1, arg2, arg3, arg4)			\
	_illegal_inst(0xf7f9b0f0 | (((num) & 0xf0) << 4) | ((num) & 0xf), \
		      arg0, arg1, arg2, arg3, arg4)
#define sbios_svc(num, arg0, arg1, arg2, arg3, arg4)			\
	_illegal_inst(0xf7f9bff0 | (((num) & 0xf0) << 4) | ((num) & 0xf), \
				  arg0, arg1, arg2, arg3, arg4)
#define _illegal_inst(num, arg0, arg1, arg2, arg3, arg4)		\
	({								\
		int ret;						\
		__asm__ __volatile__(					\
			"mov r0, %1\n"					\
			"mov r1, %2\n"					\
			"mov r2, %3\n"					\
			"mov r3, %4\n"					\
			"mov r4, %5\n"					\
			".word "#num"\n"				\
			"mov %0, r0\n"					\
			: "=r"(ret)					\
			: "r"(arg0), "r"(arg1), "r"(arg2),		\
			  "r"(arg3), "r"(arg4)				\
			: "r0", "r1", "r2", "r3", "r4", "cc");		\
		ret;							\
	})

enum boot_status {
	BS_NORMAL = -3,
	BS_LE_ERR = -2,
	BS_ERR = -1,
	BS_BOOT = 0,
	BS_CANCEL = 1,
};

int can_use_falcon(void);
int falcon_storage_init(void *storage_info, int info_size);
#ifdef CONFIG_HAVE_MEMBLOCK
void falcon_mem_reserve(void);
#ifdef CONFIG_FALCON_PSEUDO_NMI
void falcon_pnmi_handler_reserve(void);
#endif
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
void remap_falcon_lowmem(void);
#endif
void falcon_mm_init(unsigned long start_pfn, unsigned long end_pfn);
int suspend_falcon_enter(struct platform_suspend_ops const *ops);
void suspend_falcon_exit(void);
int in_falcon(void);
enum boot_status is_falcon_boot_status(void);
bool is_falcon_loading(void);
void set_falcon_callback(void (*func)(void));
void call_falcon_callback(void);
void falcon_revert_process(void);
int falcon_register_page(void);

#endif

#endif
