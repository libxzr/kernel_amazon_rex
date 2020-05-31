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
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/err.h>
#include <linux/falcon_storage.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
#include <linux/of.h>
#endif
#include <asm/mach/flash.h>
#include <asm/falcon_syscall.h>
#include "falcon_common.h"

#define SECTOR_SIZE 512

struct falcon_mtd_s {
	struct mtd_info mtd;
	struct device *dev;
	struct mutex transaction_lock;
};

static struct falcon_mtd_s *falcon_nor_data;
static struct falcon_nor_host_param *falcon_host_param;

int do_exec(enum storage_request_type type,
	    u32 arg1, void *arg2, void *arg3, void *arg4)
{
	int ret;
	int count = 0;

	mutex_lock(&falcon_nor_data->transaction_lock);

	ret = bios_svc(0x13, type, arg1, arg2, arg3, arg4);
	if (ret != RC_OK) {
		mutex_unlock(&falcon_nor_data->transaction_lock);
		if (ret == RC_DONE)
			return 0;
		else
			return -1;
	}

	ret = bios_svc(0x14, WAIT_POLLING, 0, 0, 0, 0);
	while (ret == RC_OK) {
		ret = bios_svc(0x14, WAIT_POLLING, 0, 0, 0, 0);
		set_user_nice(current, 10);
		schedule();
		count++;
	}

	mutex_unlock(&falcon_nor_data->transaction_lock);

	if (ret == RC_DONE)
		return 0;
	else
		return -1;
}

static int falcon_nor_erase_varsize(struct mtd_info *mtd,
				    struct erase_info *instr)
{
	unsigned long ofs, len, size;

	int ret;

	ofs = instr->addr;
	len = instr->len;
	size = mtd->erasesize;

	while (len) {
		ret = do_exec(RT_ERASE, ofs/SECTOR_SIZE, NULL, NULL, NULL);

		if (ret)
			return ret;

		ofs += size;
		len -= size;
	}

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return ret;
}

#define NOR_ALIGN(x, size) ((x + size - 1) & ~(size - 1))

static int falcon_nor_write(struct mtd_info *mtd, loff_t to, size_t len,
				  size_t *retlen, const u_char *buf)
{
	int ret;

	if (len > 0) {
		u32 lower = to & 0xFFFFFFFF;
		u32 upper = to >> 32;

		ret = do_exec(RT_NOR_WRITE_WORD,
			      lower, (void *)buf, (void *)len, (void *)upper);

		if (ret)
			return ret;
	}

	*retlen += len;
	return ret;
}

static int falcon_nor_read(struct mtd_info *mtd, loff_t from,
			   size_t len, size_t *retlen, u_char *buf)
{
	int ret;

	void *temp_buf =
		kmalloc(NOR_ALIGN(len, SECTOR_SIZE) + SECTOR_SIZE, GFP_KERNEL);
	u32 ofs = from % SECTOR_SIZE;
	u32 reallen;
	u32 left = len;
	loff_t start_sector = 0;
	u32 read_sectors = 0;
	u32 lower, upper;

	start_sector = from/SECTOR_SIZE;

	if (ofs != 0) {
		read_sectors = 1;

		if (ofs + left > SECTOR_SIZE)
			reallen = SECTOR_SIZE - ofs;
		else
			reallen = left;

		left -= reallen;
	}

	if (left > SECTOR_SIZE) {
		read_sectors += left/SECTOR_SIZE;

		reallen = left/SECTOR_SIZE * SECTOR_SIZE;

		left -= reallen;
	}

	if (left > 0)
		read_sectors += 1;

	lower = start_sector & 0xFFFFFFFF;
	upper = start_sector >> 32;

	ret = do_exec(RT_READ,
		      lower, temp_buf, (void *)read_sectors, (void *)upper);
	if (ret) {
		kfree(temp_buf);
		return ret;
	}

	memcpy(buf, temp_buf + ofs, len);

	kfree(temp_buf);
	*retlen += len;
	return ret;
}

static int falcon_nor_probe(struct platform_device *pdev)
{
	struct flash_platform_data *flash = pdev->dev.platform_data;
	struct mtd_info *mtd;
	int i;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	struct mtd_part_parser_data *p_ppdata = NULL;
	struct mtd_part_parser_data ppdata = {
		.of_node = pdev->dev.of_node,
	};
#else
	int nr_parts = 0;
#endif

	int err = 0;

	if (!flash) {
		flash = kmalloc(sizeof(struct flash_platform_data), GFP_KERNEL);
		if (!flash) {
			err = -ENOMEM;
			goto out;
		}
	}

	falcon_nor_data = kmalloc(sizeof(struct falcon_mtd_s), GFP_KERNEL);
	if (!falcon_nor_data) {
		err = -ENOMEM;
		goto out;
	}
	memset(falcon_nor_data, 0, sizeof(struct falcon_mtd_s));

	falcon_nor_data->dev = &pdev->dev;

	mutex_init(&falcon_nor_data->transaction_lock);

	mtd = &falcon_nor_data->mtd;
	mtd->type = MTD_NORFLASH;

	mtd->owner = THIS_MODULE;
	mtd->name = falcon_host_param->name;
	mtd->size = bios_svc(0x15, 0, 0, 0, 0, 0) * 0x200;
	mtd->_erase = falcon_nor_erase_varsize;
	mtd->_write = falcon_nor_write;
	mtd->_read    = falcon_nor_read;

	mtd->flags   = MTD_CAP_NORFLASH;
	mtd->writesize = 1;
	mtd->writebufsize = falcon_host_param->writebufsize;
	mtd->erasesize = falcon_host_param->erasesize;
	mtd->numeraseregions = falcon_host_param->numeraseregions;
	mtd->eraseregions = kmalloc(sizeof(struct mtd_erase_region_info)
				    * mtd->numeraseregions, GFP_KERNEL);

	for (i = 0; i < mtd->numeraseregions; i++) {
		mtd->eraseregions[i].offset =
			falcon_host_param->eraseregions[i].offset;
		mtd->eraseregions[i].erasesize =
			falcon_host_param->eraseregions[i].erasesize;
		mtd->eraseregions[i].numblocks =
			falcon_host_param->eraseregions[i].numblocks;
	}

	if (falcon_host_param->ppdata_flag)
		p_ppdata = &ppdata;
	mtd_device_parse_register(mtd, falcon_host_param->part_probes,
				  p_ppdata, flash->parts, flash->nr_parts);

	if (!pdev->dev.platform_data)
		pdev->dev.platform_data = flash;

	pr_info("%s(%d): numeraseregions = %d\n",
		__func__, __LINE__, mtd->numeraseregions);

	platform_set_drvdata(pdev, mtd);
	return 0;

out:
	if (flash && !pdev->dev.platform_data)
		kfree(flash);

	return err;

}

static int __exit falcon_nor_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);

	kfree(falcon_nor_data);

	return 0;
}

#ifdef CONFIG_PM

static int falcon_nor_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mtd_info *info = platform_get_drvdata(pdev);
	int ret = 0;

	mutex_lock(&falcon_nor_data->transaction_lock);

	if (info) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
		ret = mtd_suspend(info);
#else
		ret = info->suspend(info);
#endif
	}

	if (!ret)
		ret = falcon_storage_suspend();

	return ret;
}

static int falcon_nor_resume(struct platform_device *pdev)
{
	struct mtd_info *info = platform_get_drvdata(pdev);
	int ret = 0;

	if (info) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
		mtd_resume(info);
#else
		info->resume(info);
#endif
	}

	falcon_storage_resume();

	mutex_unlock(&falcon_nor_data->transaction_lock);

	return ret;
}

#else
#define falcon_nor_suspend   NULL
#define falcon_nor_resume    NULL
#endif

static struct platform_driver falcon_nor_driver = {
	.driver = {
	},
	.probe = falcon_nor_probe,
	.remove = __exit_p(falcon_nor_remove),
	.suspend = falcon_nor_suspend,
	.resume = falcon_nor_resume,
};

static int __init falcon_nor_init(void)
{
	falcon_blk_platform_init();

	if (falcon_storage_init(NULL, 0)) {
		pr_err("falcon_storage_init error\n");
		return -EINVAL;
	}

	if (!can_use_falcon()) {
		pr_err("Falcon BIOS is not found.\n");
		return -EINVAL;
	}

	falcon_host_param = falcon_nor_get_hostinfo();
	if (!falcon_host_param) {
		pr_err("Falcon NOR host info is not got.\n");
		return -EINVAL;
	}
	falcon_nor_driver.driver.name = falcon_host_param->name;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	falcon_nor_driver.driver.of_match_table =
		of_match_ptr(falcon_host_param->of_mtable);
#endif

	init_storage_common();

	return platform_driver_register(&falcon_nor_driver);
}

static void __exit falcon_nor_cleanup(void)
{

	platform_driver_unregister(&falcon_nor_driver);
}

module_init(falcon_nor_init);
module_exit(falcon_nor_cleanup);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("Falcon NOR MTD driver");
MODULE_LICENSE("GPL");
