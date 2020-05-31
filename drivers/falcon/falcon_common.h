/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _FALCON_COMMON_H
#define _FALCON_COMMON_H

enum storage_request_type {
	RT_READ,
	RT_WRITE,
	RT_READ_SG,
	RT_WRITE_SG,
	RT_ERASE,
	RT_FLUSH,
	RT_NAND_READID,
	RT_NAND_STATUS,
	RT_NAND_READ,
	RT_NAND_WRITE,
	RT_NAND_READ_SPARE,
	RT_NAND_WRITE_SPARE,
	RT_NAND_IS_BAD,
	RT_SBIOS_CALL,
	RT_CMD_CALL,
	RT_NOR_WRITE_WORD,
};

enum storage_wait_event {
	WAIT_POLLING = 1,
	WAIT_IRQ,
};

enum falcon_irqreturn {
	INT_NONE = 0,
	INT_DONE,
	INT_DONE_WAKEUP,
	INT_ERR,
};

enum storage_part_type {
	PT_DATA_PART = 0,
	PT_BOOT_PART1,
	PT_BOOT_PART2,
	PT_RPMB_PART,
	PT_MAX_PART,
};

enum flash_state_t {
	FLASH_STATE_READY,
	FLASH_STATE_STATUS,
	FLASH_STATE_CFI_QUERY,
	FLASH_STATE_JEDEC_QUERY,
	FLASH_STATE_ERASING,
	FLASH_STATE_ERASE_SUSPENDING,
	FLASH_STATE_ERASE_SUSPENDED,
	FLASH_STATE_WRITING,
	FLASH_STATE_WRITING_TO_BUFFER,
	FLASH_STATE_OTP_WRITE,
	FLASH_STATE_WRITE_SUSPENDING,
	FLASH_STATE_WRITE_SUSPENDED,
	FLASH_STATE_PM_SUSPENDED,
	FLASH_STATE_SYNCING,
	FLASH_STATE_UNLOADING,
	FLASH_STATE_LOCKING,
	FLASH_STATE_UNLOCKING,
	FLASH_STATE_POINT,
	FLASH_STATE_XIP_WHILE_ERASING,
	FLASH_STATE_XIP_WHILE_WRITING,
	FLASH_STATE_SHUTDOWN,
	FLASH_STATE_UNKNOWN
};

#define RC_ERR -1
#define RC_BUSY -2
#define RC_OK 0
#define RC_DONE 1

enum storage_type {
	S_SD,
	S_MMC,
	S_NAND,
	S_NOR,
	S_RAM,
};

struct erase_region {
	u16 count;
	u16 size;
};

struct storage_info {
	enum storage_type type;
	u32 erase_size;
	u32 capacity[PT_MAX_PART];
	u32 cache_en;
	union {
		struct {
			u32 blocks;
			u32 pages_per_block;
			u32 page_size;
			u32 spare_size;
		}nand;
		struct {
			struct erase_region erase_region[4];
		}nor;
	} private;
};


#ifndef CONFIG_FALCON_MTD_NOR
void falcon_set_wait_queue(wait_queue_head_t *q, int *cond);
#else
void falcon_set_wait_queue_mtd(struct mtd_info *mtdinfo);
#endif
void falcon_init_notice_info(void);
int falcon_storage_suspend(void);
void falcon_storage_resume(void);
void falcon_blk_platform_suspend(void);
void falcon_blk_platform_resume(void);
void init_storage_common(void);

#endif
