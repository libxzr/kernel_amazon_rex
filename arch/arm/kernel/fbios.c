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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/bootmem.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
#include <linux/sysdev.h>
#endif
#include <linux/falcon_storage.h>
#include <asm/suspend.h>
#include <asm/cacheflush.h>
#include <asm/falcon_syscall.h>
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 14, 0)
#include <asm/system.h>
#endif
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/ptrace.h>
#ifdef CONFIG_SMP
#include <asm/cpu.h>
#ifdef CONFIG_GENERIC_SMP_IDLE_THREAD
#include <linux/smpboot.h>
#include <linux/smp.h>
#endif
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
#include <asm/idmap.h>
#endif

#include <asm/tlbflush.h>

#ifdef CONFIG_HAVE_MEMBLOCK
#include <linux/memblock.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
#include <asm/memory.h>
#include <asm/sections.h>
#include <asm/mach/map.h>
#include <linux/bootmem.h>
#endif

#include <linux/falcon_revert.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 27)
#undef for_each_nodebank
#endif
#define for_each_nodebank(iter, mi, no)			\
	for (iter = 0; iter < (mi)->nr_banks; iter++)	\
		if ((mi)->bank[iter].node == no)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 28)
#define bank_pfn_start(bank)    __phys_to_pfn((bank)->start)
#define bank_pfn_end(bank)      __phys_to_pfn((bank)->start + (bank)->size)
#endif
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 6)
#define USABLE_AREA     0xe7fddef1
#else
#define USABLE_AREA     0x0
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 6)
#define WORK_OFFSET     0xc00
#else
#define WORK_OFFSET     0x200
#endif
#define MAX_USABLE_AREA	8
const u32 vector_work_offset = WORK_OFFSET;
EXPORT_SYMBOL(vector_work_offset);

struct bios {
	u32 marker;
};

early_param_on_off("falcon", "nofalcon", falcon_enabled, 1);
static int is_exist_bios;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
static int is_remap;
#endif
static int is_bios_initialized;
#ifdef CONFIG_FALCON_PSEUDO_NMI
static int is_exist_pnmi_handler;
#endif
static void (*falcon_callback_fn)(void);
static void *resume_storage_info;
static int resume_storage_info_size;

static int do_falcon_callback(unsigned long addr, unsigned int fsr,
			      struct pt_regs *regs);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
static int falcon_suspend(struct sys_device *dev, pm_message_t state);
static int falcon_resume(struct sys_device *dev);
#else
static int falcon_suspend(void);
static void falcon_resume(void);
#endif
int *falcon_boot_status;
EXPORT_SYMBOL(falcon_boot_status);
int *falcon_load_status = NULL;
EXPORT_SYMBOL(falcon_load_status);

int need_idleload = 0;
EXPORT_SYMBOL(need_idleload);

#ifdef CONFIG_FALCON_CMA
int need_revert = 1;
EXPORT_SYMBOL(need_revert);
#endif

#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
static struct wake_lock falcon_wakelock;

#define TIMEOUT_FALCON_WAKELOCK		(HZ * 7)
#endif

#ifdef CONFIG_FALCON_CMA

LIST_HEAD(falcon_revert_list);
EXPORT_SYMBOL(falcon_revert_list);
DEFINE_SPINLOCK(falcon_revert_lock);
EXPORT_SYMBOL(falcon_revert_lock);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
static struct sysdev_class falcon_sysclass = {
	.name = "falcon",
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
	.suspend = falcon_suspend,
	.resume = falcon_resume,
#endif
};
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
#include <linux/syscore_ops.h>
static struct syscore_ops falcon_syscore_ops = {
	.suspend = falcon_suspend,
	.resume = falcon_resume,
};
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
static struct sys_device device_falcon = {
	.id	= 0,
	.cls	= &falcon_sysclass,
};
#endif

static struct platform_suspend_ops const *suspend_ops_orig;

u32 storage_bios_addr;
u32 storage_bios_size;
u32 falcon_bios_addr;
u32 falcon_bios_size;

int can_use_falcon(void)
{
	return falcon_enabled && is_exist_bios && is_bios_initialized;
}
EXPORT_SYMBOL(can_use_falcon);

int falcon_storage_init(void *storage_info, int info_size)
{
	if (!is_exist_bios)
		return -1;

	if (bios_svc(0x12, storage_info, info_size, 0, 0, 0)) {
		pr_err("storage_init error\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(falcon_storage_init);

static void __init replace_ioaddr(void)
{
	int ret;
	u32 paddr, size, *vaddr;

	while ((ret = bios_svc(0x9, &paddr, &size, &vaddr, 0, 0)) == 0) {
		void *addr;

		addr = ioremap(paddr, size);
		if (addr == NULL)
			pr_err("ioremap error!! paddr=%x\n", paddr);

		pr_debug("IOTABLE phys:%08x size:%08x ptr:%p => %p\n",
			 paddr, size, vaddr, addr);

		*vaddr = (u32)addr;
	}
}

#ifndef CONFIG_ARM_LPAE

u32 saved_pte;

static void add_write_perm(u32 addr)
{
	u32 *l1_base = (u32 *) init_mm.pgd;
	u32 *l2_base  = (u32 *) phys_to_virt(l1_base[addr >> 20] & 0xfffffc00);
	u32 *ptep = &l2_base[(addr >> 12) & 0xff];

	saved_pte = *ptep;
	*ptep = *ptep & ~(PTE_EXT_APX);

	__cpuc_flush_kern_all();
	barrier();
	__cpu_flush_kern_tlb_range(addr & PAGE_MASK, PAGE_SIZE);

}
EXPORT_SYMBOL(add_write_perm);

static void restore_perm(u32 addr)
{
	u32 *l1_base = (u32 *) init_mm.pgd;
	u32 *l2_base  = (u32 *) phys_to_virt(l1_base[addr >> 20] & 0xfffffc00);
	u32 *ptep = &l2_base[(addr >> 12) & 0xff];

	*ptep = saved_pte;

	__cpuc_flush_kern_all();
	barrier();
	__cpu_flush_kern_tlb_range(addr & PAGE_MASK, PAGE_SIZE);

}
EXPORT_SYMBOL(restore_perm);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
static void add_exec_perm(u32 addr)
{
	u32 *l1_base = (u32 *) init_mm.pgd;
	u32 *l2_base = (u32 *) phys_to_virt(l1_base[addr >> 20] & 0xfffffc00);
	u32 *ptep = &l2_base[(addr >> 12) & 0xff];

	*ptep = *ptep & ~(PTE_EXT_XN);

	__cpuc_flush_kern_all();
	barrier();
	__cpu_flush_kern_tlb_range(addr & PAGE_MASK, PAGE_SIZE);
}
#endif

#else
static void modify_pte(u32 addr, u64 clr_bit, u64 set_bit)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset(&init_mm, addr);
	if (!pgd_present(*pgd))
		return;

	pud = pud_offset(pgd, addr);
	if (!pud_present(*pud))
		return;

	pmd = pmd_offset(pud, addr);
	if (!pmd_present(*pmd))
		return;

	pte = pte_offset_map(pmd, addr);
	if (pte_present(*pte)) {
		pte_t pteval = pte_val(*pte);

		pteval &= ~clr_bit;
		pteval |= set_bit;
		*pte = pteval;
	}
	pte_unmap(pte);

	__cpuc_flush_kern_all();
	barrier();
	__cpu_flush_kern_tlb_range(addr & PAGE_MASK, PAGE_SIZE);
}

static void add_write_perm(u32 addr)
{
	modify_pte(addr, L_PTE_RDONLY, 0);
}
EXPORT_SYMBOL(add_write_perm);

static void restore_perm(u32 addr)
{
	modify_pte(addr, 0, L_PTE_RDONLY);
}
EXPORT_SYMBOL(restore_perm);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
static void add_exec_perm(u32 addr)
{
	modify_pte(addr, L_PTE_XN, 0);
}
#endif

#endif

static int __init prepare_for_hook(void)
{
	volatile u32 *addr = (u32 *)(0xffff0000 + vector_work_offset);
	u32 vector = 0xffff0000;
	int count = 0;
	int i;

	if (!USABLE_AREA)
		return 0;

	add_write_perm(vector);

	for (i = 0; i < vector_work_offset / 4; i++) {
		addr--;
		if (*addr == USABLE_AREA) {
			*addr = 0;
			count++;
			if (count >= MAX_USABLE_AREA)
				break;
		}
	}

	restore_perm(vector);

	if (count < MAX_USABLE_AREA)
		return -EINVAL;

	return 0;
}

static int __init hook_vector(int offset)
{
	volatile u32 *vector = (u32 *)(0xffff0000 + offset);
	volatile u32 *addr = (u32 *)(0xffff0000 + vector_work_offset);
	u32 org_vector;
	int i;
	int ret;
	unsigned long flags;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	u32 tmp_addr;
#endif

	for (i = 0; i < vector_work_offset / 4; i++) {
		addr--;
		if (*addr == 0)
			break;
	}

	if (*addr) {
		pr_err("Fatal: FALCON workarea is not found.\n");
		return -EINVAL;
	}

	local_irq_save(flags);

	add_write_perm((u32)vector);

	*addr = falcon_bios_addr + offset;

	org_vector = *vector;

	*vector = 0xe59ff000 | ((u32)addr - (u32)vector - 8);
	__cpuc_flush_kern_all();

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	if (0 < is_remap) {
		for (tmp_addr = storage_bios_addr & PAGE_MASK;
		     tmp_addr <
			     falcon_bios_addr + falcon_bios_size;
		     tmp_addr += PAGE_SIZE)
			add_exec_perm(tmp_addr);
	}
#endif

	ret = bios_svc(0x1, org_vector, storage_bios_addr, 1, 0, 0);
	if (ret) {
		pr_err("Fatal: hooking failed.");
		*vector = org_vector;
		*addr = 0;
	}

	__cpuc_flush_kern_all();

	restore_perm((u32)vector);

	local_irq_restore(flags);

	if (ret) {
		return -EINVAL;
	}

	return 0;
}

static int __init check_bios(void)
{
	const struct bios *bios =
		(const struct bios *)(falcon_bios_addr + 0x30);
	const struct bios *sbios =
		(const struct bios *)(storage_bios_addr + 0x20);

	if (bios->marker != 0x51494255 || sbios->marker != 0x51494255)
		return -1;

	pr_info("Falcon BIOS found.\n");

	return 0;
}

#ifdef CONFIG_FALCON_PSEUDO_NMI
static int __init check_pnmi_handler(void)
{
	struct header {
		u32 marker;
		u32 revision;
	};
	const struct header *header = (const struct header *)
		(CONFIG_FALCON_PSEUDO_NMI_HANDLER_ADDR + 0x20);

	if (header->marker != 0x71696275)
		return -1;

	pr_info("Pseudo NMI handler found.\n");

	return 0;
}

#ifdef CONFIG_FALCON_PSEUDO_NMI_VECTOR_HOOK
int __init hook_irq_vector(int offset)
{
	volatile u32 *vector = (u32 *)(0xffff0000 + offset);
	volatile u32 *addr = (u32 *)(0xffff0000 + STUBS_OFFSET);

	int i;
	unsigned long flags;

	if (check_pnmi_handler())
		return -EINVAL;

	for (i = 0; i < STUBS_OFFSET / 4; i++) {
		addr--;
		if (*addr == 0)
			break;
	}

	if (*addr) {
		pr_err("Fatal: FALCON workarea is not found.\n");
		return -EINVAL;
	}

	local_irq_save(flags);

	add_write_perm((u32)vector);

	*addr = CONFIG_FALCON_PSEUDO_NMI_HANDLER_ADDR + 8;

	*vector = 0xe59ff000 | ((u32)addr - (u32)vector - 8);
	__cpuc_flush_kern_all();

	restore_perm((u32)vector);

	local_irq_restore(flags);

	return 0;
}
#endif
#endif

static int __init register_bank(void)
{
#ifdef CONFIG_HAVE_MEMBLOCK
	struct memblock_region *reg;

	for_each_memblock(memory, reg) {
		int ret;
		phys_addr_t addr = reg->base;
		phys_addr_t left = reg->size;

		while (left > 0) {
			u32 size = (left > 0xc0000000UL)
				? 0x10000000UL : (u32)left;

			ret = bios_svc(0x26, (u32)addr, size, (u32)__va(addr), (u32)((u64)addr >> 32), 0);
			if(ret) {
				pr_err("Failed to register bank(p=(%x:%x), v=%x, size=%x)\n",
					(u32)((u64)addr>>32), (u32)addr, (u32)__va(addr), size);
				return -1;
			}

			addr += size;
			left -= size;
		}
	}
#else
	int i;

	for_each_nodebank(i, &meminfo, 0) {
		int ret;
		struct membank *bank = &meminfo.bank[i];
		u32 vaddr = (u32)__va(bank->start);

		ret = bios_svc(0x26, bank->start, bank->size, vaddr, 0, 0);
		if(ret) {
			pr_err("Failed to register bank(p=%x, v=%x, size=%x)\n",
				bank->start, vaddr, bank->size);
			return -1;
		}
	}
#endif

	return 0;
}


static int __init falcon_init(void)
{
	if (is_exist_bios == 0)
		return 0;

	if (prepare_for_hook()) {
		pr_err("Hook preparation is failed.\n");
		return -EINVAL;
	}

	if (hook_vector(4)) {
		pr_err("BIOS initialization is failed.\n");
		return -EINVAL;
	}

#ifdef CONFIG_FALCON_PSEUDO_NMI_VECTOR_HOOK

	if (hook_irq_vector(0x18)) {
		pr_err("Hook IRQ vector failed.\n");
		return -EINVAL;
	}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
	hook_fault_code(0, do_falcon_callback, SIGSEGV, "vector exception");
#else
	hook_fault_code(0, do_falcon_callback, SIGSEGV, 0, "vector exception");
#endif

	replace_ioaddr();

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
	if (sysdev_class_register(&falcon_sysclass) ||
	    sysdev_register(&device_falcon))
		is_exist_bios = 0;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	register_syscore_ops(&falcon_syscore_ops);
#endif
	if(register_bank())
		return -EINVAL;

	is_bios_initialized = 1;

#ifdef CONFIG_WAKELOCK
	wake_lock_init(&falcon_wakelock, WAKE_LOCK_SUSPEND, "falcon");
#endif

	return 0;
}
core_initcall(falcon_init);

int falcon_workmem_size = CONFIG_FALCON_BIOS_WORK_SIZE;
EXPORT_SYMBOL(falcon_workmem_size);
void *falcon_workmem_addr;
EXPORT_SYMBOL(falcon_workmem_addr);

#ifdef CONFIG_HAVE_MEMBLOCK
#define BIOS_AREA_ADDR (CONFIG_FALCON_STORAGE_BIOS_ADDR)
#define BIOS_AREA_SIZE							\
	(CONFIG_FALCON_STORAGE_BIOS_SIZE + CONFIG_FALCON_BIOS_SIZE)

static int is_reserved_memblock;

#include <linux/of_fdt.h>
#ifdef CONFIG_OF_FLATTREE
int __init early_init_dt_scan_falcon(unsigned long node, const char *uname,
				     int depth, void *data)
{
	int l;
	const __be32 *reg;

	if (strcmp(uname, "falcon") != 0)
		return 0;

	reg = of_get_flat_dt_prop(node, "falcon-bios", &l);
	if (reg) {
		falcon_bios_addr = (u32)dt_mem_next_cell(1, &reg);
		falcon_bios_size = (u32)dt_mem_next_cell(1, &reg);
	}
	else {
		falcon_bios_addr = CONFIG_FALCON_BIOS_ADDR;
		falcon_bios_size = CONFIG_FALCON_BIOS_SIZE;
	}

	pr_info("dt: falcon bios 0x%x(0x%x)\n", falcon_bios_addr, falcon_bios_size);

	reg = of_get_flat_dt_prop(node, "storage-bios", &l);
	if (reg) {
		storage_bios_addr = (u32)dt_mem_next_cell(1, &reg);
		storage_bios_size = (u32)dt_mem_next_cell(1, &reg);
	} else {
		storage_bios_addr = CONFIG_FALCON_STORAGE_BIOS_ADDR;
		storage_bios_size = CONFIG_FALCON_STORAGE_BIOS_SIZE;
	}

	pr_info("dt: storage bios 0x%x(0x%x)\n", storage_bios_addr, storage_bios_size);

	reg = of_get_flat_dt_prop(node, "work-size", &l);
	if (reg)
		falcon_workmem_size = dt_mem_next_cell(1, &reg);
	else
		falcon_workmem_size = CONFIG_FALCON_BIOS_WORK_SIZE;

	pr_info("dt: falcon work 0x%x\n", falcon_workmem_size);

	return 1;
}
#endif

void __init falcon_mem_reserve(void)
{
#ifdef CONFIG_OF_FLATTREE
	of_scan_flat_dt(early_init_dt_scan_falcon, NULL);
#else
	falcon_bios_addr = CONFIG_FALCON_BIOS_ADDR;
	falcon_bios_size = CONFIG_FALCON_BIOS_ADDR;
	storage_bios_addr = CONFIG_FALCON_STORAGE_BIOS_ADDR;
	storage_bios_size = CONFIG_FALCON_STORAGE_BIOS_SIZE;
	falcon_workmem_size = CONFIG_FALCON_BIOS_WORK_SIZE;
#endif
	if (storage_bios_addr != falcon_bios_addr - storage_bios_size) {
		pr_err("#### Error!! BIOS area is not valid!!\n");
		return;
	}

	if (!memblock_is_region_memory(__virt_to_phys(storage_bios_addr),
				       storage_bios_size + falcon_bios_size)) {
		pr_err("#### Error!! BIOS area is not in a memory region!!\n");
		return;
	}

	if (memblock_is_region_reserved(__virt_to_phys(storage_bios_addr),
					storage_bios_size + falcon_bios_size)) {
		pr_err("#### Error!! BIOS area overlaps in-use memory region!!\n");
		return;
	}

	memblock_reserve(__virt_to_phys(storage_bios_addr), storage_bios_size + falcon_bios_size);

	is_reserved_memblock = 1;
}

#ifdef CONFIG_FALCON_PSEUDO_NMI
static int is_reserved_pnmi_handler;

void __init falcon_pnmi_handler_reserve(void)
{
	u32 handler_addr = CONFIG_FALCON_PSEUDO_NMI_HANDLER_ADDR;
	u32 handler_size = CONFIG_FALCON_PSEUDO_NMI_HANDLER_SIZE;

	if (!memblock_is_region_memory(__virt_to_phys(handler_addr),
				       handler_size)) {
		pr_err("#### Error!! Pseudo NMI handler area is not in a memory region!!\n");
		return;
	}

	if (memblock_is_region_reserved(__virt_to_phys(handler_addr),
					handler_size)) {
		pr_err("#### Error!! Ext Pseudo NMI handler area overlaps in-use memory region!!\n");
		return;
	}

	memblock_reserve(__virt_to_phys(handler_addr), handler_size);

	is_reserved_pnmi_handler = 1;
}
#endif

#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
void __init remap_falcon_lowmem(void)
{
	unsigned long kernel_x_start =
		round_down((unsigned long)_stext, SECTION_SIZE);
	unsigned long kernel_x_end =
		round_up((unsigned long)__init_end, SECTION_SIZE);

	unsigned long bios_pmd_start =
		round_down(storage_bios_addr, PMD_SIZE);
	unsigned long bios_pmd_end =
		round_up(falcon_bios_addr + falcon_bios_size, PMD_SIZE);

	int pmd_count = (bios_pmd_end - bios_pmd_start) / PMD_SIZE;
	int ealloc_count = 0;
	u32 *prv_alloc_p = NULL;
	u32 *alloc_p = NULL;

#ifdef CONFIG_HAVE_MEMBLOCK
	if (!is_reserved_memblock)
		return;
#endif

	if ((kernel_x_start <= storage_bios_addr) &&
	    (falcon_bios_addr + falcon_bios_size <=
	     kernel_x_end)) {
		is_remap = -1;
		return;
	}

	if (!((bios_pmd_end <= kernel_x_start) ||
	      (kernel_x_end <= bios_pmd_start))) {
		pr_err("#### Error!! SBIOS area is too low!!\n");
		return;
	}

	while (ealloc_count < pmd_count) {
		prv_alloc_p = alloc_p;
		alloc_p = (u32 *) __va(
			memblock_alloc(PTE_HWTABLE_OFF + PTE_HWTABLE_SIZE,
				       PTE_HWTABLE_OFF + PTE_HWTABLE_SIZE));
		*alloc_p = (u32) prv_alloc_p;

		if (((u32) alloc_p + (PTE_HWTABLE_OFF + PTE_HWTABLE_SIZE) <=
		     (u32) bios_pmd_start) ||
		    ((u32) bios_pmd_end <= (u32) alloc_p))
			ealloc_count++;
		else
			ealloc_count = 0;
	}

	while (ealloc_count) {
		prv_alloc_p = (u32 *) *alloc_p;
		memblock_free_early(__pa(alloc_p),
				    PTE_HWTABLE_OFF + PTE_HWTABLE_SIZE);
		alloc_p = prv_alloc_p;
		ealloc_count--;
	}

	falcon_remapping(bios_pmd_start, bios_pmd_end);

	while (alloc_p) {
		prv_alloc_p = (u32 *) *alloc_p;
		memblock_free_early(__pa(alloc_p),
				    PTE_HWTABLE_OFF + PTE_HWTABLE_SIZE);
		alloc_p = prv_alloc_p;
	}

	is_remap = 1;
}
#endif

void __init falcon_mm_init(unsigned long start_pfn, unsigned long end_pfn)
{
#ifndef CONFIG_HAVE_MEMBLOCK
	int ret;
	u32 paddr;
	u32 size;
#endif
	u32 end_addr = (u32)phys_to_virt(__pfn_to_phys(end_pfn));

	if (!falcon_enabled) {
		pr_info("Falcon Disabled.\n");
		return;
	}

	if (is_exist_bios)
		return;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	if (!is_remap)
		return;
#elif defined(CONFIG_HAVE_MEMBLOCK)
	if (!is_reserved_memblock)
		return;
#else

	if (storage_bios_addr != falcon_bios_addr - storage_bios_size) {
		pr_err("#### Error!! BIOS area is not valid!!\n");
		return;
	}

	if (storage_bios_addr < (u32)_end) {
		pr_err("#### Error!! SBIOS area is too low!!\n");
		return;
	}
#endif

	if (falcon_bios_addr + falcon_bios_size > end_addr) {
		pr_err("#### Error!! BIOS area exceeds lowmem region!!\n");
#ifdef CONFIG_HAVE_MEMBLOCK
		memblock_free(__virt_to_phys(storage_bios_addr), storage_bios_size + falcon_bios_size);
#endif
		return;
	}

#ifndef CONFIG_HAVE_MEMBLOCK
	paddr = __virt_to_phys(storage_bios_addr);
	size = falcon_bios_size + storage_bios_size;
	ret = reserve_bootmem(paddr, size, BOOTMEM_EXCLUSIVE);
	if (ret) {
		pr_err("#### Error!! BIOS area has been already reserved!!\n");
		return;
	}
#endif

	if (falcon_workmem_size) {
#ifndef CONFIG_HAVE_MEMBLOCK
		falcon_workmem_addr = __alloc_bootmem(falcon_workmem_size,
						      PAGE_SIZE,
						      __pa((u32)_end));
#else
		falcon_workmem_addr =
			__va(memblock_alloc(falcon_workmem_size, PAGE_SIZE));
#endif
		if (!falcon_workmem_addr)
			pr_err("#### Error!! Can't allocate BIOS work area\n");
		else
			memset(falcon_workmem_addr, 0, falcon_workmem_size);
	}

	if (check_bios()) {
		pr_warn("Falcon BIOS is not found.\n");
		return;
	}
	is_exist_bios = 1;

#ifdef CONFIG_FALCON_PSEUDO_NMI
	if (!is_reserved_pnmi_handler)
		return;

	if (check_pnmi_handler()) {
		pr_warn("Falcon Pseudo NMI handler is not found.\n");
		return;
	}
	is_exist_pnmi_handler = 1;
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
static int falcon_suspend(struct sys_device *dev, pm_message_t state)
#else
static int falcon_suspend(void)
#endif
{
	if (!is_exist_bios)
		return 0;

	return bios_svc(0x21, 0, 0, 0, 0, 0) ? -EINVAL : 0;
}

#ifdef CONFIG_WAKELOCK
static void falcon_resume_timeout(unsigned long data)
{
	wake_lock(&falcon_wakelock);
}
#endif

void set_resume_storage_info(void *storage_info, int storage_info_size)
{
	resume_storage_info = storage_info;
	resume_storage_info_size = storage_info_size;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
#define RES_RET_TRUE	(0)
#define RES_RET_ERROR	(-EINVAL)
static int falcon_resume(struct sys_device *dev)
#else
#define RES_RET_TRUE
#define RES_RET_ERROR
static void falcon_resume(void)
#endif
{
	if (!is_exist_bios)
		return RES_RET_TRUE;

	if (in_falcon()) {
#ifdef CONFIG_WAKELOCK
		static struct timer_list timer;

		init_timer_on_stack(&timer);
		timer.expires = jiffies + 1;
		timer.function = falcon_resume_timeout;
		add_timer(&timer);
#endif
		return RES_RET_TRUE;
	}

	if (bios_svc(0x12, resume_storage_info, resume_storage_info_size, 0, 0, 0)) {
		is_exist_bios = 0;
		pr_err("falcon_resume error\n");
		return RES_RET_ERROR;
	}

	return RES_RET_TRUE;
}

int suspend_falcon_enter(struct platform_suspend_ops const *ops)
{
	suspend_ops_orig = suspend_ops;
	suspend_set_ops(ops);

#ifdef CONFIG_EARLYSUSPEND
	request_suspend_state(PM_SUSPEND_MEM);
	return 0;
#else
	return pm_suspend(PM_SUSPEND_MEM);
#endif
}
EXPORT_SYMBOL(suspend_falcon_enter);

void suspend_falcon_exit(void)
{
	if (!suspend_ops_orig)
		return;

	suspend_set_ops(suspend_ops_orig);
	suspend_ops_orig = NULL;

#ifdef CONFIG_WAKELOCK

	wake_lock_timeout(&falcon_wakelock, TIMEOUT_FALCON_WAKELOCK);
#endif
}
EXPORT_SYMBOL(suspend_falcon_exit);

int in_falcon(void)
{
	return !!suspend_ops_orig;
}
EXPORT_SYMBOL(in_falcon);

enum boot_status is_falcon_boot_status(void)
{
	if (falcon_boot_status)
		return (enum boot_status) *falcon_boot_status;

	return BS_NORMAL;
}
EXPORT_SYMBOL(is_falcon_boot_status);

bool is_falcon_loading(void)
{
	if (falcon_load_status)
		return !!(*falcon_load_status);

	return false;
}
EXPORT_SYMBOL(is_falcon_loading);

void set_falcon_callback(void (*func)(void))
{
	falcon_callback_fn = func;
}

void call_falcon_callback(void)
{
	if (falcon_callback_fn)
		falcon_callback_fn();
}
EXPORT_SYMBOL(call_falcon_callback);

static int do_falcon_callback(unsigned long addr, unsigned int fsr,
			      struct pt_regs *regs)
{
	call_falcon_callback();
	asm volatile ("mcr p15, 0, %0, c5, c0, 0\n\t" : : "r"(0x40f));
	return 0;
}

#ifndef CONFIG_ARM_LPAE
static void falcon_revert_ownerless_mm(struct task_struct *p)
{
	if (p == NULL || p->active_mm == NULL)
		return;

	if (atomic_read(&p->active_mm->mm_users) < 1)
		bios_svc(0x16, p->pid, p->active_mm->pgd, 0, 0, 0);
}

void falcon_revert_process(void)
{
	struct task_struct *p;
#ifdef CONFIG_SMP
	int cpu;
#endif

	read_lock(&tasklist_lock);

	bios_svc(0x16, 0, init_mm.pgd, 0, 0, 0);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	bios_svc(0x16, 0, idmap_pgd, 0, 0, 0);
#endif

#ifndef CONFIG_SMP
	falcon_revert_ownerless_mm(current);
#else
	for_each_possible_cpu(cpu) {
#ifndef CONFIG_GENERIC_SMP_IDLE_THREAD
		p = (&per_cpu(cpu_data, cpu))->idle;
#else

		p = get_cpu_idle_thread(cpu);
		if (IS_ERR(p))
			continue;
#endif
		if (p != NULL)
			falcon_revert_ownerless_mm(p);
	}
#endif

	for_each_process(p)
		if (p->mm)
			bios_svc(0x16, p->pid, p->mm->pgd, 0, 0, 0);

	read_unlock(&tasklist_lock);

}
#else
void falcon_revert_process(void)
{
	bios_svc(0x16, 0, init_mm.pgd, 0, 0, 0);
}
#endif
EXPORT_SYMBOL(falcon_revert_process);

int falcon_register_page(void)
{
#ifdef CONFIG_HAVE_MEMBLOCK
	struct memblock_region *reg;

	for_each_memblock(memory, reg) {
		u32 start_pfn, end_pfn;

		start_pfn = memblock_region_memory_base_pfn(reg);
		end_pfn = memblock_region_memory_end_pfn(reg);

		bios_svc(0x5, pfn_to_page(start_pfn), (u32)reg->base, end_pfn - start_pfn, (u32)((u64)reg->base >> 32), 0);
	}
#else
	int bank;

	for (bank = 0; bank < meminfo.nr_banks; bank++) {
		u32 start_pfn = bank_pfn_start(&meminfo.bank[bank]);
		u32 end_pfn = bank_pfn_end(&meminfo.bank[bank]);

		bios_svc(0x5, pfn_to_page(start_pfn), __pfn_to_phys(start_pfn), end_pfn - start_pfn, 0, 0);
	}
#endif
	return 0;
}
EXPORT_SYMBOL(falcon_register_page);

struct pid *falcon_get_task_pid(struct task_struct *t, enum pid_type type)
{
	return get_task_pid(t, type);
}
EXPORT_SYMBOL(falcon_get_task_pid);

struct task_struct *falcon_pid_task(struct pid *p, enum pid_type type)
{
	return pid_task(p, type);
}
EXPORT_SYMBOL(falcon_pid_task);

void falcon_put_pid(struct pid *p)
{
	put_pid(p);
}
EXPORT_SYMBOL(falcon_put_pid);

#ifdef CONFIG_FALCON_PSEUDO_NMI

void handle_pseudo_nmi(u32 irqno, struct pt_regs *regs)
{
	if (is_exist_pnmi_handler) {
		void (*handler)(u32, struct pt_regs *) =
			(void *)CONFIG_FALCON_PSEUDO_NMI_HANDLER_ADDR;

		handler(irqno, regs);
	} else
		pr_warn("WARNING: Pseudo NMI handler is missing!!\n");
}

static int is_running_on_falcon(u32 ret_addr)
{
	u32 bios_area_start = storage_bios_addr;
	u32 bios_area_end = bios_area_start +
		storage_bios_size + falcon_bios_size;

	if (ret_addr >= bios_area_start && ret_addr < bios_area_end)
		return 1;

	return 0;
}
#endif

#ifdef CONFIG_FALCON_DRV
static int is_running_falcon_callback(void)
{
	u32 dfsr;

	asm volatile ("mrc p15, 0, %0, c5, c0, 0\n\t" : "=r"(dfsr));

	if ((dfsr & 0x40f) == 0)
		return 1;

	return 0;
}
#endif

bool falcon_needs_skipping_preempt(struct pt_regs *p_regs)
{
#ifdef CONFIG_FALCON_PSEUDO_NMI
	if (is_running_on_falcon(p_regs->ARM_lr))
		return 1;
#endif
#ifdef CONFIG_FALCON_DRV
	if (is_running_falcon_callback())
		return 1;
#endif
	return 0;
}
