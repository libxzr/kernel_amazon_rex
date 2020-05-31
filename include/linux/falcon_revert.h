#ifndef _ASM_ARM_FALCON_REVERT_H
#define _ASM_ARM_FALCON_REVERT_H

#include <linux/list.h>

struct falcon_revert_entry {
	struct list_head head;
	struct mm_struct *mm;
};

extern struct list_head falcon_revert_list;
extern spinlock_t falcon_revert_lock;

#endif /* _FALCON_REVERT_H */
