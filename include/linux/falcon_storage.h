/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _LINUX_FALCON_STORAGE_H
#define _LINUX_FALCON_STORAGE_H

#ifdef __KERNEL__
#include <linux/version.h>
#include <linux/mtd/mtd.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
#include <linux/of.h>
#endif

#include <uapi/linux/falcon_storage.h>

struct falcon_blk_host_param {
	const char      *name;
	unsigned int    max_seg_size;
	unsigned short  max_hw_segs;
	unsigned short  max_phys_segs;
	unsigned int    max_req_size;
	unsigned int    max_blk_size;
	unsigned int    max_blk_count;
	int             irq;
	u64             dma_mask;

	unsigned char	heads;
	unsigned char	sectors;
};

struct falcon_nand_host_param {
	char			*name;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	const struct of_device_id	*of_mtable;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	const char		* const *part_probes;
#else
	const char		**part_probes;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	int			ppdata_flag;
#endif
	u32			pagesize;
	u32			sparesize;
	int			ecc_strength;
	struct nand_ecclayout	*nand_oob_layout;
	int			irq;
};

struct falcon_nor_erase_region {
	u16 count;
	u16 size;
};

struct falcon_nor_host_param {
	char			*name;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	const struct of_device_id	*of_mtable;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	const char		* const *part_probes;
#else
	const char		**part_probes;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	int			ppdata_flag;
#endif
	u32			writebufsize;
	u32			erasesize;
	int			numeraseregions;
	struct mtd_erase_region_info eraseregions[4];

};

void falcon_blk_platform_init(void);
struct falcon_blk_host_param *falcon_blk_get_hostinfo(void);
struct falcon_nand_host_param *falcon_nand_get_hostinfo(void);
struct falcon_nor_host_param *falcon_nor_get_hostinfo(void);
void falcon_blk_platform_pre(void);
void falcon_blk_platform_post(void);

void falcon_delayed_wakeup(void);

#define GET_FALCON_PROP_U8(params, x) ({				\
			int ret = 0;					\
			u8 val = 0;					\
			ret = of_property_read_u8(np, #x, &val);	\
			if (0 != ret) {					\
				pr_err("of_get_property(" #x ") error\n"); \
			} else {					\
				params.x = val;				\
			}						\
		})

#define GET_FALCON_PROP_U16(params, x) ({				\
			int ret = 0;					\
			u16 val = 0;					\
			ret = of_property_read_u16(np, #x, &val);	\
			if (0 != ret) {					\
				pr_err("of_get_property(" #x ") error\n"); \
			} else {					\
				params.x = val;				\
			}						\
		})

#define GET_FALCON_PROP_U32(params, x) ({				\
			int ret = 0;					\
			u32 val = 0;					\
			ret = of_property_read_u32(np, #x, &val);	\
			if (0 != ret) {					\
				pr_err("of_get_property(" #x ") error\n"); \
			} else {					\
				params.x = val;				\
			}						\
		})

#define GET_FALCON_PROP_S32(params, x) ({				\
			int ret = 0;					\
			s32 val = 0;					\
			ret = of_property_read_s32(np, #x, &val);	\
			if (0 != ret) {					\
				pr_err("of_get_property(" #x ") error\n"); \
			} else {					\
				params.x = val;				\
			}						\
		})

#define GET_FALCON_PROP_U64(params, x) ({				\
			int ret = 0;					\
			u64 val = 0;					\
			ret = of_property_read_u64(np, #x, &val);	\
			if (0 != ret) {					\
				pr_err("of_get_property(" #x ") error\n"); \
			} else {					\
				params.x = val;				\
			}						\
		})

#endif

#endif
