/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef _LINUX_FALCONMEM_H_
#define _LINUX_FALCONMEM_H_
/*
struct falcon_meminfo_t {
	phys_addr_t paddr;
	u32 order;
	struct list_head mem_list;
};
*/
void falcon_add_preload(phys_addr_t paddr, u32 order);
void falcon_del_preload(phys_addr_t paddr, u32 order);

extern struct list_head falcon_meminfo_head;
extern int falcon_meminfo_pages;

typedef struct falcon_meminfo_t {
	u32 paddr;
	u32 order;
	u32 size;
	struct mm_struct *mm; //optional, only used for usermem ranges
	struct list_head mem_list;
} falcon_meminfo;

struct falcon_pidinfo {
	pid_t pid;
	struct mm_struct *mm;
	int flags;
	unsigned int num_regions;
	uintptr_t *regions;

	struct list_head info_list;
};


extern struct list_head falcon_kernel_ranges_head;
void falcon_add_preload_kernel_range(u32 paddr, u32 size);
void falcon_del_preload_kernel_range(u32 paddr, u32 range);

extern struct list_head falcon_physical_ranges_head;
void falcon_add_preload_physical_range(u32 paddr, u32 size);
void falcon_del_preload_physical_range(u32 paddr, u32 range);

extern struct list_head falcon_usermem_ranges_head;
void falcon_add_preload_user_range(struct mm_struct *mm, u32 paddr, u32 size);
void falcon_del_preload_user_range(struct mm_struct *mm, u32 paddr, u32 range);
void falcon_delete_all_preload_usermem(void);

extern struct list_head falcon_process_options;
void fixup_falcon_process_options(void);

#define PROCESS_SAVE_CHAR_ALL_ANON   'A'
#define PROCESS_SAVE_CHAR_ACT_FILE   'f'
#define PROCESS_SAVE_CHAR_ALL_FILE   'F'
#define PROCESS_SAVE_CHAR_REGION_START '<'
#define PROCESS_SAVE_CHAR_REGION_END   '>'

#define PROCESS_SAVE_FLAG_ALL_ANON   1
#define PROCESS_SAVE_FLAG_ACT_FILE   2
#define PROCESS_SAVE_FLAG_INACT_FILE 4
#define PROCESS_SAVE_FLAG_ALL_FILE   (PROCESS_SAVE_FLAG_INACT_FILE | PROCESS_SAVE_FLAG_ACT_FILE)

#define FALCON_MAX_PFN   ((1ULL << 32) >> PAGE_SHIFT)

extern unsigned long falcon_bitmap[];

#endif
