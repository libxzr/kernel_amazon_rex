/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/falcon_storage.h>

#define SECTOR_SIZE	512

/*
 *  Block device Host controller parameters
 */
static struct falcon_blk_host_param falcon_blk_param = {
	.name = "falcon blk",
	.max_seg_size = 128 * 1024,
	.max_hw_segs = 1,		/* max_hw_segs (use Bounce Buffer) */
	/* .max_hw_segs = 32, */	/* max_hw_segs (use Multisector I/O) */
	.max_phys_segs = 32,
	.max_req_size = 128 * 1024,
	.max_blk_size = SECTOR_SIZE,
	.max_blk_count = 128 * 1024 / SECTOR_SIZE,
	/* .irq = 9, */			/* irq  */
	.irq = -1,			/* irq (doesn't use irq) */
};

/*
 *  NAND Host controller parameters
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
static const struct of_device_id falcon_nand_host_ids[] = {
	{ .compatible = "xxx", },
	{},
};
#endif

static const char * const part_probes[] = { "cmdlinepart", NULL };

/* OOB placement block for use with hardware ecc generation */
static struct nand_ecclayout nand_oob_layout = {
	.eccbytes = 24,
	.eccpos = {
		   40, 41, 42, 43, 44, 45, 46, 47,
		   48, 49, 50, 51, 52, 53, 54, 55,
		   56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = {
		{.offset = 2,
		 .length = 38} }
};

static struct falcon_nand_host_param falcon_nand_param = {
	.name = "falcon nand",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	.of_mtable = falcon_nand_host_ids,
#endif
	.part_probes = part_probes,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	/*
	 *   1 : set parser_data for mtd_device_parse_register()
	 *   0 : doesn't set parser_data for mtd_device_parse_register()
	 */
	.ppdata_flag = 1,
#endif
	.pagesize = 2048,
	.sparesize = 32,
	.ecc_strength = 1,
	.nand_oob_layout = &nand_oob_layout,
	/* .irq = 33, */	/* irq */
	.irq = -1,		/* irq (doesn't use irq) */
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
static const struct of_device_id falcon_nor_host_ids[] = {
	{ .compatible = "xxx", },
	{},
};
#endif

static struct falcon_nor_host_param falcon_nor_param = {
	.name			= "falcon nor",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	.of_mtable		= falcon_nor_host_ids,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	/*
	 *   1 : set parser_data for mtd_device_parse_register()
	 *   0 : doesn't set parser_data for mtd_device_parse_register()
	 */
	.ppdata_flag		= 1,
#endif
	.writebufsize		= 512,
	.erasesize		= 0x20000,
	.numeraseregions	= 1,
#if 0
	.eraseregions		= {},
#endif
};

/**
 * Get block dev host controller parameter
 *
 * @return    addr of host param structure
 */
struct falcon_blk_host_param *falcon_blk_get_hostinfo(void)
{
	return &falcon_blk_param;
}

/**
 * Do platform depending operations
 * This is called before real HW access is done.
 */
void falcon_blk_platform_pre(void)
{
}

/**
 * Do platform depending operations
 * This is called after real HW access is done.
 */
void falcon_blk_platform_post(void)
{
}

/**
 * Initialization for platform depending operations
 * This is called once when falcon block wrapper driver is initalized.
 */
void falcon_blk_platform_init(void)
{
}

/**
 * Do platform depended operations
 * This function is called in suspend
 */
void falcon_blk_platform_suspend(void)
{
}

/**
 * Do platform depended operations
 * This function is called in resume.
 */
void falcon_blk_platform_resume(void)
{
}

/**
 * Get NAND controller parameter
 *
 * @return    addr of NAND controller param structure
 */
struct falcon_nand_host_param *falcon_nand_get_hostinfo(void)
{
	return &falcon_nand_param;
}


/**
 * Get NOR device parameter
 *
 * @return    addr of NOR device param structure
 */
struct falcon_nor_host_param *falcon_nor_get_hostinfo(void)
{
	return &falcon_nor_param;
}

/*
 * Local Variables:
 * mode: c
 * c-file-style: "K&R"
 * tab-width: 8
 * indent-tabs-mode: t
 * c-basic-offset: 8
 * End:
 */
