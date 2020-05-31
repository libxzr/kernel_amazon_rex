/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/spinlock.h>
#include <linux/falconmem.h>
#include <linux/seq_file.h>

static DEFINE_SPINLOCK(lock);

LIST_HEAD(falcon_meminfo_head);
EXPORT_SYMBOL(falcon_meminfo_head);

int falcon_meminfo_pages;
EXPORT_SYMBOL(falcon_meminfo_pages);

DECLARE_BITMAP(falcon_bitmap, FALCON_MAX_PFN);
EXPORT_SYMBOL(falcon_bitmap);

void falcon_set_preload(phys_addr_t paddr, u32 num)
{
	unsigned int pfn = __phys_to_pfn(paddr);

	if ((paddr >> PAGE_SHIFT) > FALCON_MAX_PFN) {
		pr_err("%s paddr=%pa pfn=%x (%d)\n", __func__, &paddr, pfn, num);
		return;
	}

	bitmap_set(falcon_bitmap, pfn, num);

}
EXPORT_SYMBOL(falcon_set_preload);

void falcon_clr_preload(phys_addr_t paddr, u32 num)
{
	unsigned pfn = __phys_to_pfn(paddr);

	if ((paddr >> PAGE_SHIFT) > FALCON_MAX_PFN) {
		pr_err("%s paddr=%pa pfn=%x (%d)\n", __func__, &paddr, pfn, num);
		return;
	}

	bitmap_clear(falcon_bitmap, pfn, num);

}
EXPORT_SYMBOL(falcon_clr_preload);

void falcon_add_preload(phys_addr_t paddr, u32 order)
{
	unsigned long flags;
	struct falcon_meminfo_t *item;

	item = kmalloc(sizeof(struct falcon_meminfo_t),
		       GFP_KERNEL);
	item->paddr = paddr;
	item->order = order;

	spin_lock_irqsave(&lock, flags);
	list_add(&(item->mem_list), &falcon_meminfo_head);
	falcon_meminfo_pages += 1 << order;
	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL(falcon_add_preload);

void falcon_del_preload(phys_addr_t paddr, u32 order)
{
	unsigned long flags;
	struct falcon_meminfo_t *item;

	spin_lock_irqsave(&lock, flags);
	list_for_each_entry(item, &falcon_meminfo_head, mem_list) {
		if (item->paddr == paddr && item->order == order) {

			list_del(&item->mem_list);
			falcon_meminfo_pages -= 1 << item->order;
			spin_unlock_irqrestore(&lock, flags);
			kfree(item);
			return;
		}
	}
	spin_unlock_irqrestore(&lock, flags);
#ifdef CONFIG_ARM_LPAE
	pr_warn("%s: no entry phys=%llx order=%d\n", __func__, paddr, order);
#else
	pr_warn("%s: no entry phys=%x order=%d\n", __func__, paddr, order);
#endif
}
EXPORT_SYMBOL(falcon_del_preload);

LIST_HEAD(falcon_kernel_ranges_head);
EXPORT_SYMBOL(falcon_kernel_ranges_head);

LIST_HEAD(falcon_process_options);
EXPORT_SYMBOL(falcon_process_options);

void falcon_add_preload_kernel_range(u32 paddr, u32 size)
{
	unsigned long flags;
	falcon_meminfo *item;

//	printk("%s: p=%x size=%d\n", __func__, paddr, size);
	item = (falcon_meminfo *)kmalloc(sizeof(falcon_meminfo), GFP_ATOMIC);
	if (item == NULL) {
		printk(KERN_ERR "%s unable to allocate container for range\n", __func__);
		return;
	}
	item->paddr = paddr;
	item->size = size;

	spin_lock_irqsave(&lock, flags);
	list_add(&(item->mem_list), &falcon_kernel_ranges_head);
	falcon_meminfo_pages += PAGE_ALIGN(size) >> PAGE_SHIFT;
	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL(falcon_add_preload_kernel_range);

void falcon_del_preload_kernel_range(u32 paddr, u32 size)
{
	unsigned long flags;
	falcon_meminfo *item;

	spin_lock_irqsave(&lock, flags);
	list_for_each_entry(item, &falcon_kernel_ranges_head, mem_list) {
		if (item->paddr == paddr && item->size == size) {
//			printk("%s: p=%x\n size=%d", __func__, paddr, size);
			list_del(&item->mem_list);
			falcon_meminfo_pages -= PAGE_ALIGN(size) >> PAGE_SHIFT;
			spin_unlock_irqrestore(&lock, flags);
			kfree(item);
			return;
		}
	}
	spin_unlock_irqrestore(&lock, flags);
	printk("%s: no entry phys=%x size=%d\n", __func__, paddr, size);
}
EXPORT_SYMBOL(falcon_del_preload_kernel_range);


LIST_HEAD(falcon_physical_ranges_head);
EXPORT_SYMBOL(falcon_physical_ranges_head);

void falcon_add_preload_physical_range(phys_addr_t paddr, u32 size)
{
	unsigned long flags;
	falcon_meminfo *item;

//	printk("%s: p=%x size=%d\n", __func__, paddr, size);
	item = (falcon_meminfo *)kmalloc(sizeof(falcon_meminfo), GFP_ATOMIC);
	if (item == NULL) {
		printk(KERN_ERR "%s unable to allocate container for range\n", __func__);
		return;
	}
	item->paddr = paddr;
	item->size = size;

	spin_lock_irqsave(&lock, flags);
	list_add(&(item->mem_list), &falcon_physical_ranges_head);
	falcon_meminfo_pages += PAGE_ALIGN(size) >> PAGE_SHIFT;
	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL(falcon_add_preload_physical_range);

void falcon_del_preload_physical_range(phys_addr_t paddr, u32 size)
{
	unsigned long flags;
	falcon_meminfo *item;

	spin_lock_irqsave(&lock, flags);
	list_for_each_entry(item, &falcon_physical_ranges_head, mem_list) {
		if (item->paddr == paddr && item->size == size) {
//			printk("%s: p=%x\n size=%d", __func__, paddr, size);
			list_del(&item->mem_list);
			falcon_meminfo_pages -= PAGE_ALIGN(size) >> PAGE_SHIFT;
			spin_unlock_irqrestore(&lock, flags);
			kfree(item);
			return;
		}
	}
	spin_unlock_irqrestore(&lock, flags);
	printk("%s: no entry phys=%x size=%d\n", __func__, paddr, size);
}
EXPORT_SYMBOL(falcon_del_preload_physical_range);


LIST_HEAD(falcon_usermem_ranges_head);
EXPORT_SYMBOL(falcon_usermem_ranges_head);

void falcon_add_preload_user_range(struct mm_struct * mm, u32 paddr, u32 size)
{
	unsigned long flags;
	falcon_meminfo *item;

//	printk("%s: mm=%p, p=%x size=%d\n", __func__, mm, paddr, size);
	item = (falcon_meminfo *)kmalloc(sizeof(falcon_meminfo), GFP_ATOMIC);
	if (item == NULL) {
		printk(KERN_ERR "%s unable to allocate container for range\n", __func__);
		return;
	}
	item->paddr = paddr;
	item->size = size;
	item->mm = mm;

	spin_lock_irqsave(&lock, flags);
	list_add(&(item->mem_list), &falcon_usermem_ranges_head);
	falcon_meminfo_pages += PAGE_ALIGN(size) >> PAGE_SHIFT;
	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL(falcon_add_preload_user_range);

void falcon_del_preload_user_range(struct mm_struct *mm, u32 paddr, u32 size)
{
	unsigned long flags;
	falcon_meminfo *item;

	spin_lock_irqsave(&lock, flags);
	list_for_each_entry(item, &falcon_usermem_ranges_head, mem_list) {
		if (item->mm == mm && item->paddr == paddr && item->size == size) {
//			printk("%s: p=%x\n size=%d", __func__, paddr, size);
			list_del(&item->mem_list);
			falcon_meminfo_pages -= PAGE_ALIGN(size) >> PAGE_SHIFT;
			spin_unlock_irqrestore(&lock, flags);
			kfree(item);
			return;
		}
	}
	spin_unlock_irqrestore(&lock, flags);
	printk(KERN_INFO "%s: no entry mm=%p, addr=%x, size=%d\n", __func__, mm, paddr, size);
}
EXPORT_SYMBOL(falcon_del_preload_user_range);

void falcon_delete_all_preload_usermem(void)
{
	unsigned long flags;
	falcon_meminfo *item, *temp;

	spin_lock_irqsave(&lock, flags);
	list_for_each_entry_safe(item, temp, &falcon_usermem_ranges_head, mem_list) {
		list_del(&item->mem_list);
		falcon_meminfo_pages -= PAGE_ALIGN(item->size) >> PAGE_SHIFT;
		kfree(item);
	}
	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL(falcon_delete_all_preload_usermem);

/*
 * For every process with preload options, set the struct falcon_pidinfo
 * structure's "mm" field to the user process's mm field. This needs to be done
 * during the snapshot process after userspace has been frozen, instead of when
 * the options were written to the proc entry, because the process may have
 * died between writing and hibernate.
 */
void fixup_falcon_process_options(void) {
	struct falcon_pidinfo *info, *tmp;

	list_for_each_entry_safe(info, tmp, &falcon_process_options, info_list) {
		struct task_struct *tsk = find_task_by_vpid(info->pid);
		if (tsk) {
			info->mm = tsk->mm;
		} else {
			info->mm = NULL;
		}
	}
}
EXPORT_SYMBOL(fixup_falcon_process_options);


/*
 * Parse the null terminated string "buffer" into the pidinfo structure info.
 * info->regions can initially be set to null, and all other properties will be
 * set. This allows the caller to make two passes, the first to figure out how
 * large to make the regions array, and the second to fill it in.
 *
 * The format of the parsed string is:
 *  A numeric PID, followed by
 *  Any number of extra options after the PID:
 *    'A' - preload all anon pages
 *    'f' - preload file pages, but only active ones
 *    'F' - preload all file pages
 *    <#> - preload the virtual mapping containing address #.
 *
 * Only one PID with options can be parsed at a time. Multiple processes will
 * require multiple writes to the proc entry.
 *
 * Both the PID and the range address are allowed to be either decimal numbers,
 * or base 16 numbers prefixed with 0x.
 *
 * Returns 0 if input was valid, or -EINVAL if the string was invalid.
 */
static int parse_process_save(char *buffer, struct falcon_pidinfo *info) {
	unsigned long long tmp;

	tmp = simple_strtoull(buffer, &buffer, 0);

	info->pid = tmp;

	info->mm = NULL;
	info->num_regions = 0;
	info->flags = 0;

	while (*buffer) {
		if (*buffer == PROCESS_SAVE_CHAR_ALL_ANON) {
			info->flags |= PROCESS_SAVE_FLAG_ALL_ANON;
			buffer++;
		} else if (*buffer == PROCESS_SAVE_CHAR_ACT_FILE) {
			info->flags |= PROCESS_SAVE_FLAG_ACT_FILE;
			buffer++;
		} else if (*buffer == PROCESS_SAVE_CHAR_ALL_FILE) {
			info->flags |= PROCESS_SAVE_FLAG_ALL_FILE;
			buffer++;
		} else if (*buffer == PROCESS_SAVE_CHAR_REGION_START) {
			tmp = simple_strtoul(buffer +1, &buffer, 0);

			if (info->regions != NULL) {
				info->regions[info->num_regions] = tmp;
			}
			info->num_regions += 1;

			if (*buffer != PROCESS_SAVE_CHAR_REGION_END) {
				return -EINVAL;
			}
			buffer++;
		} else if (*buffer == '\n' || *buffer == '\t' || *buffer == ' ') {
			buffer++;
		} else {
			printk(KERN_CRIT "Unexpected chr: %d\n", (int)(*buffer));
			return -EINVAL;
		}
	}

	return 0;
}




/*static int falconmem_proc_options_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data) */

static int options_show(struct seq_file *s, void *p)
{
	struct falcon_pidinfo *info, *tmp;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&lock, flags);
	list_for_each_entry_safe(info, tmp, &falcon_process_options, info_list) {
		seq_printf(s, "PID %d - Saves %s%s%s%s%s\n", info->pid,
			(info->flags & PROCESS_SAVE_FLAG_ALL_ANON) ? "all anon, " : "",
			(info->flags & PROCESS_SAVE_FLAG_INACT_FILE) ? "inactive file, ": "",
			(info->flags & PROCESS_SAVE_FLAG_ACT_FILE) ? "active file, ": "",
			info->flags && info->num_regions ? "and" :"",
			(info->num_regions != 0) ? "regions containing:": "");
		for (i=0; i<info->num_regions; i++) {
			seq_printf(s, "\t0x%p\n", (void *)info->regions[i]);
		}
	}
	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

static ssize_t options_write(struct file *file, const char __user *buffer,
			   size_t count, loff_t *data) {
	char buffer_copy[256];
	int ret;
	unsigned long flags;
	struct falcon_pidinfo *info;

	if (count < ARRAY_SIZE(buffer_copy)) {
		if (copy_from_user(buffer_copy, buffer, count)) {
			return -EFAULT;
		}
		buffer_copy[count] = '\0';
	} else {
		if (copy_from_user(buffer_copy, buffer, ARRAY_SIZE(buffer_copy))) {
			return -EFAULT;
		}
		buffer_copy[ARRAY_SIZE(buffer_copy)-1] = '\0';
	}

	info = kmalloc(sizeof *info, GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}

	info->regions = NULL;

	ret = parse_process_save(buffer_copy, info);
	if (ret) {
		kfree(info);
		return ret;
	}

	if (info->num_regions > 0) {
		info->regions = kmalloc(info->num_regions * (sizeof(uintptr_t)), GFP_KERNEL);
		if (info->regions == NULL) {
			kfree(info);
			return -ENOMEM;
		} else {
			parse_process_save(buffer_copy, info);
		}
	}

	spin_lock_irqsave(&lock, flags);
	list_add(&(info->info_list), &falcon_process_options);
	spin_unlock_irqrestore(&lock, flags);

	return count;
}
static int options_open(struct inode *inode, struct file *file)
{
	return single_open(file, options_show, NULL);
}

static int reset_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, NULL);
}


static ssize_t reset_write(struct file *file, const char __user *buffer,
			   size_t count, loff_t *data) {
	struct falcon_pidinfo *info, *tmp;
	unsigned long flags;


	spin_lock_irqsave(&lock, flags);
	list_for_each_entry_safe(info, tmp, &falcon_process_options, info_list) {
		list_del(&(info->info_list));
		kfree(info);
	}
	spin_unlock_irqrestore(&lock, flags);

	return count;
}


static const struct file_operations options_fops = {
	.open 		= options_open,
	.read 		= seq_read,
	.write 		= options_write,
	.llseek 	= seq_lseek,
	.release 	= single_release,
};

static const struct file_operations reset_fops = {
	.open 		= reset_open,
	.read 		= seq_read,
	.write 		= reset_write,
	.llseek 	= seq_lseek,
	.release 	= single_release,
};


static struct proc_entry {
	char *name;
	umode_t mode;
	const struct file_operations *fops;
	struct proc_dir_entry *entry;
} proc_files[] = {
	{"process_options", 0600, &options_fops},
	{"reset", 0200, &reset_fops},
	{NULL}
};
/* *INDENT-ON* */

static struct proc_dir_entry *falconmem_proc __read_mostly;

static int create_proc_file(struct proc_entry *p)
{
	p->entry = proc_create(p->name, p->mode, falconmem_proc, p->fops);
	if (!p->entry)
		return -1;
	return 0;
}

static void delete_proc_files(void)
{
	struct proc_entry *p;

	if (falconmem_proc) {
		for (p = proc_files; p->name; p++)
			if (p->entry)
				remove_proc_entry(p->name, falconmem_proc);
		proc_remove(falconmem_proc);
	}
}

void falconmem_exit(void) {
    delete_proc_files();
}

int falconmem_init(void) {
	struct proc_entry *p;

	falconmem_proc = proc_mkdir("falconmem", NULL);
	if (falconmem_proc == NULL) {
		return -EINVAL;
	}

        for (p = proc_files; p->name; p++)
            if (create_proc_file(p))
                goto err;
        return 0;

err:
        delete_proc_files();
        return -EINVAL;
}

module_init(falconmem_init);
module_exit(falconmem_exit);
