/*
 * Hibernation support specific for ARM
 *
 * Derived from work on ARM hibernation support by:
 *
 * Ubuntu project, hibernation support for mach-dove
 * Copyright (C) 2010 Nokia Corporation (Hiroshi Doyu)
 * Copyright (C) 2010 Texas Instruments, Inc. (Teerth Reddy et al.)
 *  https://lkml.org/lkml/2010/6/18/4
 *  https://lists.linux-foundation.org/pipermail/linux-pm/2010-June/027422.html
 *  https://patchwork.kernel.org/patch/96442/
 *
 * Copyright (C) 2006 Rafael J. Wysocki <rjw@sisk.pl>
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <asm/system_misc.h>
#include <asm/idmap.h>
#include <asm/suspend.h>
#include <asm/memory.h>
#include <asm/sections.h>
#if defined(CONFIG_BD71827)
#include <linux/mfd/bd71827.h>
#endif
#include "reboot.h"

#if defined(CONFIG_LAB126) && defined(CONFIG_TOI)
extern const void __nosave_begin, __nosave_end;
extern void cpu_resume(void);
extern void cpu_resume_restore_nosave(void);
extern void get_block_chain_info(u32 *data);

u32 __nosave_backup_phys;
u32 __nosave_begin_phys;
u32 __nosave_end_phys;
u32 page_over_uboot_sp_cnt = 0;
u32 hole_page_cnt = 0;
u32 ps1_bmap[4096];
extern void cpu_save_state(void);
extern void cpu_restore_state(void);
#endif

int pfn_is_nosave(unsigned long pfn)
{
	unsigned long nosave_begin_pfn = virt_to_pfn(&__nosave_begin);
	unsigned long nosave_end_pfn = virt_to_pfn(&__nosave_end - 1);

	return (pfn >= nosave_begin_pfn) && (pfn <= nosave_end_pfn);
}

void notrace save_processor_state(void)
{
	WARN_ON(num_online_cpus() != 1);
	local_fiq_disable();
#if defined(CONFIG_LAB126) && defined(CONFIG_TOI)
	cpu_save_state();
#endif
}

void notrace restore_processor_state(void)
{
	local_fiq_enable();

	extern int in_suspend;

	if(in_suspend)
 	  return;
#if defined(CONFIG_LAB126) && defined(CONFIG_TOI)
	cpu_restore_state();
#endif
}

#if defined(CONFIG_LAB126) && defined(CONFIG_TOI)
void swsusp_arch_add_info(char *archdata, u32* over_sp_cnt, u32* free_page, u8 *pmicRegs, u32 *bmap, u32 *chain_info)
{
	*(u32 *) archdata = virt_to_phys(cpu_resume_restore_nosave);
	*over_sp_cnt = page_over_uboot_sp_cnt;
	*free_page = hole_page_cnt;

	memcpy((u32*) bmap, (u32*) ps1_bmap, 892*4);

	printk(KERN_INFO "cpu_resume address 0x%X\n", *(u32 *)archdata);
	printk(KERN_INFO "over_sp_cnt=(0x%X)%d\n", *over_sp_cnt, *over_sp_cnt);
	printk(KERN_INFO "free_page=(0x%X)%d\n", *free_page, *free_page);

#if defined(CONFIG_BD71827)
	pmicRegs[0] = ext_bd71827_reg_read8(0x12);
	printk(KERN_INFO "regadd:0x12 = %0X\n", pmicRegs[0]);
#endif
	memset(chain_info, 0x0, 5*sizeof(u32));
	get_block_chain_info(chain_info);
	if (chain_info[4] == 0) {
		chain_info[4] = 1;
	}
	
	printk(KERN_INFO "chain_info[0] = %d\n", chain_info[0]);
	printk(KERN_INFO "chain_info[1] = %d\n", chain_info[1]);
	printk(KERN_INFO "chain_info[2] = %d\n", chain_info[2]);
	printk(KERN_INFO "chain_info[3] = %d\n", chain_info[3]);
	printk(KERN_INFO "chain_info[4] = %d\n", chain_info[4]);
}

void toi_meta_add_info(u32 over_sp_cnt, u32 free_page_cnt, u32 *bmap)
{
	page_over_uboot_sp_cnt = over_sp_cnt;
	hole_page_cnt = free_page_cnt;
	memcpy((u32*) ps1_bmap, (u32*) bmap, 4096*4);
}
EXPORT_SYMBOL(toi_meta_add_info);
#endif

/*
 * Snapshot kernel memory and reset the system.
 *
 * swsusp_save() is executed in the suspend finisher so that the CPU
 * context pointer and memory are part of the saved image, which is
 * required by the resume kernel image to restart execution from
 * swsusp_arch_suspend().
 *
 * soft_restart is not technically needed, but is used to get success
 * returned from cpu_suspend.
 *
 * When soft reboot completes, the hibernation snapshot is written out.
 */
static int notrace arch_save_image(unsigned long unused)
{
	int ret;

	ret = swsusp_save();
	if (ret == 0)
		_soft_restart(virt_to_phys(cpu_resume), false);
	return ret;
}

/*
 * Save the current CPU state before suspend / poweroff.
 */
int notrace swsusp_arch_suspend(void)
{
	return cpu_suspend(0, arch_save_image);
}

/*
 * Restore page contents for physical pages that were in use during loading
 * hibernation image.  Switch to idmap_pgd so the physical page tables
 * are overwritten with the same contents.
 */
static void notrace arch_restore_image(void *unused)
{
	struct pbe *pbe;

	cpu_switch_mm(idmap_pgd, &init_mm);
	for (pbe = restore_pblist; pbe; pbe = pbe->next)
		copy_page(pbe->orig_address, pbe->address);

	_soft_restart(virt_to_phys(cpu_resume), false);
}

static u64 resume_stack[PAGE_SIZE/2/sizeof(u64)] __nosavedata;

/*
 * Resume from the hibernation image.
 * Due to the kernel heap / data restore, stack contents change underneath
 * and that would make function calls impossible; switch to a temporary
 * stack within the nosave region to avoid that problem.
 */
int swsusp_arch_resume(void)
{
	call_with_stack(arch_restore_image, 0,
		resume_stack + ARRAY_SIZE(resume_stack));
	return 0;
}

#if defined(CONFIG_LAB126) && defined(CONFIG_TOI)
static int __init swsusp_arch_init(void)
{
	char *backup;
	size_t len;

	len = &__nosave_end - &__nosave_begin;
	backup = kmalloc(len, GFP_KERNEL);
	printk("############## init ###");
	printk("Size of sector_t %d", sizeof(sector_t));
	if (backup) {
		pr_info("%s: Backed up %d byte nosave region\n", __func__, len);
		memcpy(backup, &__nosave_begin, len);
	}

	__nosave_backup_phys = virt_to_phys(backup);
	__nosave_begin_phys = virt_to_phys(&__nosave_begin);
	__nosave_end_phys = virt_to_phys(&__nosave_end);

	return 0;
}

late_initcall(swsusp_arch_init);
#endif