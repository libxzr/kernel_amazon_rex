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
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/falcon_storage.h>
#include <linux/version.h>
#include <linux/mutex.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
#include <linux/of.h>
#endif

#include <asm/mach/flash.h>

#include <asm/falcon_syscall.h>
#include "falcon_common.h"

#define MTD_DEBUG_LEVEL0 (0)
#define MTD_DEBUG_LEVEL1 (1)
#define MTD_DEBUG_LEVEL2 (2)
#define MTD_DEBUG_LEVEL3 (3)

#ifdef CONFIG_MTD_DEBUG
#define DEBUG(n, args...) {			\
	do {					\
		pr_info(args);			\
	} while (0);				\
}
#else
#define DEBUG(n, args...) {			\
	do {					\
		if (0)				\
			pr_info(args);		\
	} while (0);				\
}
#endif

#define SECTOR_SIZE 512

#define NAND_BUFF_ALIGN 64
#define NAND_BUFF_ALIGNMENT(x)						\
	(((u32)(x) + NAND_BUFF_ALIGN - 1) & ~(NAND_BUFF_ALIGN - 1))

#define EXEC_TIMEOUT	HZ

struct falcon_mtd_s {
	struct mtd_info mtd;
	struct nand_chip nand;
	struct mtd_partition *parts;
	struct device *dev;
	void *p_data_buf;
	char *data_buf;
	int result;
	u16 col_addr;
	u32 write_sector;
	bool write_oobonly;
	int irq_num;
	int irq_handled;
	wait_queue_head_t irq_waitq;
	struct mutex transaction_lock;
};

static struct falcon_mtd_s *falcon_nand_data;
static struct falcon_nand_host_param *falcon_host_param;

static inline u32 page_to_sector(int page_addr, struct nand_chip *chip)
{
	BUG_ON((1 << chip->page_shift) < SECTOR_SIZE);

	return ((u32) page_addr) * ((1 << chip->page_shift) / SECTOR_SIZE);
}

int do_exec(enum storage_request_type type, u32 arg1, void *arg2, void *arg3)
{
	int ret;
	int count = 0;
	int wait_stat;

	mutex_lock(&falcon_nand_data->transaction_lock);

	ret = bios_svc(0x13, type, arg1, arg2, arg3, NULL);
	if (ret != RC_OK) {
		mutex_unlock(&falcon_nand_data->transaction_lock);
		if (ret == RC_DONE)
			return 0;
		else
			return -1;
	}

	if (falcon_nand_data->irq_num != -1) {
		falcon_nand_data->irq_handled = 0;
		ret = bios_svc(0x14, WAIT_IRQ, 0, 0, 0, 0);

		while (ret == RC_OK) {
			wait_stat =
				wait_event_timeout(
					falcon_nand_data->irq_waitq,
					falcon_nand_data->irq_handled,
					EXEC_TIMEOUT);

			if (wait_stat < 0)
				pr_err("%s: ERROR: wait completion error\n",
				       __func__);

			falcon_nand_data->irq_handled = 0;
			ret = bios_svc(0x14, WAIT_IRQ, 0, 0, 0, 0);
		}
	} else {
		ret = bios_svc(0x14, WAIT_POLLING, 0, 0, 0, 0);
		while (ret == RC_OK) {
			ret = bios_svc(0x14, WAIT_POLLING, 0, 0, 0, 0);
			set_user_nice(current, 10);
			schedule();
			count++;
		}
	}

	mutex_unlock(&falcon_nand_data->transaction_lock);

	if (ret == RC_DONE)
		return 0;
	else
		return -1;
}

static int read_id(u8 *buf)
{
	return do_exec(RT_NAND_READID, 0, buf, 0);
}

static int read_status(void)
{
	u32 status;

	if (do_exec(RT_NAND_STATUS, 0, &status, 0))
		return -1;
	return status;
}

static int read_single_page(u32 sector, void *buff, void *spare)
{
	return do_exec(RT_NAND_READ, sector, buff, spare);
}

static int write_single_page(u32 sector, void *buff, void *spare)
{
	return do_exec(RT_NAND_WRITE, sector, buff, spare);
}

static int read_single_spare(u32 sector, void *spare)
{
	return do_exec(RT_NAND_READ_SPARE, sector, NULL, spare);
}

static int write_single_spare(u32 sector, void *spare)
{
	return do_exec(RT_NAND_WRITE_SPARE, sector, NULL, spare);
}

static int erase_block(u32 sector)
{
	return do_exec(RT_ERASE, sector, NULL, NULL);
}

static void wait_op_done(void)
{

}

static int falcon_nand_dev_ready(struct mtd_info *mtd)
{
	return 1;
}

static void falcon_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{

}

static int falcon_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				 u_char *read_ecc, u_char *calc_ecc)
{

	return 0;
}

static int falcon_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				  u_char *ecc_code)
{

	return 0;
}

static u_char falcon_nand_read_byte(struct mtd_info *mtd)
{
	u_char ret_val = 0;

	ret_val = falcon_nand_data->data_buf[falcon_nand_data->col_addr++];

	DEBUG(MTD_DEBUG_LEVEL3,
		  "%s: ret=0x%x\n", __func__, ret_val);

	return ret_val;
}

static u16 falcon_nand_read_word(struct mtd_info *mtd)
{
	u16 ret_val;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "falcon_nand_read_word(col = %d)\n", falcon_nand_data->col_addr);

	ret_val = falcon_nand_data->data_buf[falcon_nand_data->col_addr++];
	ret_val |= (u16)falcon_nand_data->data_buf[falcon_nand_data->col_addr++]
			<< 8;

	return ret_val;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
static int falcon_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_required, int pages)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
static int falcon_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int pages)
#else
static int falcon_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf)
#endif
{
	memcpy(buf, falcon_nand_data->data_buf, mtd->writesize);
	memcpy(chip->oob_poi,
	       falcon_nand_data->data_buf + mtd->writesize,
	       mtd->oobsize);

	if (falcon_nand_data->result)
		mtd->ecc_stats.failed++;

	return 0;
}

static void falcon_nand_write_buf(struct mtd_info *mtd,
			       const u_char *buf, int len)
{
	int n;
	int col;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "falcon_nand_write_buf(col = %d, len = %d)\n",
	      falcon_nand_data->col_addr, len);

	col = falcon_nand_data->col_addr;

	n = mtd->writesize + mtd->oobsize - col;
	n = min(len, n);

	DEBUG(MTD_DEBUG_LEVEL3,
	      "%s:%d: col = %d, n = %d\n", __func__, __LINE__, col, n);

	memcpy(falcon_nand_data->data_buf + falcon_nand_data->col_addr, buf, n);

	falcon_nand_data->col_addr += n;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
static int falcon_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf, int oob_required)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
static void falcon_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				   const uint8_t *buf, int oob_required)
#else
static void falcon_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf)
#endif
{

	memcpy(falcon_nand_data->data_buf, buf, mtd->writesize);
	memcpy(falcon_nand_data->data_buf + mtd->writesize, chip->oob_poi,
		   mtd->oobsize);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
	return 0;
#else
	return;
#endif
}

static void falcon_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{

	int n;
	int col;

	DEBUG(MTD_DEBUG_LEVEL3,
	      "falcon_nand_read_buf(col = %d, len = %d)\n",
	      falcon_nand_data->col_addr, len);

	col = falcon_nand_data->col_addr;

	n = mtd->writesize + mtd->oobsize - col;
	n = min(len, n);

	memcpy(buf, falcon_nand_data->data_buf + col, n);

	falcon_nand_data->col_addr = col + n;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 0)

static int
falcon_nand_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	pr_info("%s not implemented\n", __func__);
	return -EFAULT;
}
#endif

static void falcon_nand_select_chip(struct mtd_info *mtd, int chip)
{
}

static void falcon_nand_command(struct mtd_info *mtd, unsigned command,
			     int column, int page_addr)
{
	struct nand_chip *this = mtd->priv;

	if (command != NAND_CMD_READOOB)
		DEBUG(MTD_DEBUG_LEVEL3,
			  "falcon_nand_command (cmd = 0x%x, col = 0x%x, page = 0x%x)\n",
			  command, column, page_addr);

	switch (command) {

	case NAND_CMD_STATUS:
		falcon_nand_data->data_buf[0] = read_status();

		falcon_nand_data->data_buf[1] = 0;
		falcon_nand_data->col_addr = 0;
		return;

	case NAND_CMD_READ0:
		falcon_nand_data->result =
			read_single_page(page_to_sector(page_addr, this),
					 falcon_nand_data->data_buf,
					 falcon_nand_data->data_buf +
					 mtd->writesize);
		falcon_nand_data->col_addr = column;
		return;

	case NAND_CMD_READID:
		read_id(falcon_nand_data->data_buf);
		falcon_nand_data->col_addr = 0;
		return;

	case NAND_CMD_READOOB:
		read_single_spare(page_to_sector(page_addr, this),
				  falcon_nand_data->data_buf + mtd->writesize);
		falcon_nand_data->col_addr = mtd->writesize;
		return;

	case NAND_CMD_SEQIN:
		if (column >= mtd->writesize) {

			if (column != mtd->writesize)

				read_single_spare(
					page_to_sector(page_addr, this),
					falcon_nand_data->data_buf +
					mtd->writesize);
			falcon_nand_data->write_oobonly = 1;
		} else {
			if (column != 0)

				read_single_page(
					page_to_sector(page_addr, this),
					falcon_nand_data->data_buf,
					falcon_nand_data->data_buf +
					mtd->writesize);
			falcon_nand_data->write_oobonly = 0;
		}
		falcon_nand_data->col_addr = column;
		falcon_nand_data->write_sector =
			page_to_sector(page_addr, this);
		return;

	case NAND_CMD_PAGEPROG:
		if (!falcon_nand_data->write_oobonly)
			falcon_nand_data->result =
				write_single_page(
					falcon_nand_data->write_sector,
					falcon_nand_data->data_buf,
					falcon_nand_data->data_buf +
					mtd->writesize);
		else
			falcon_nand_data->result =
				write_single_spare(
					falcon_nand_data->write_sector,
					falcon_nand_data->data_buf +
					mtd->writesize);
		return;

	case NAND_CMD_ERASE1:
		falcon_nand_data->result =
			erase_block(page_to_sector(page_addr, this));

		wait_op_done();
		return;

	case NAND_CMD_ERASE2:
		return;

	case NAND_CMD_RESET:
		return;

	default:
		pr_err("unknown command %x\n", command);
		return;
	}

}

static int falcon_nand_wait(struct mtd_info *mtd, struct nand_chip *chip)
{
	return falcon_nand_data->result ? NAND_STATUS_FAIL : 0;
}

static int falcon_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
				 int page)
{
	struct nand_chip *this = mtd->priv;
	int ret = 0;
	unsigned long flags;

	memcpy(falcon_nand_data->data_buf + mtd->writesize,
	       chip->oob_poi, mtd->oobsize);
	local_irq_save(flags);
	ret = write_single_spare(page_to_sector(page, this),
				 falcon_nand_data->data_buf + mtd->writesize);
	local_irq_restore(flags);

	return ret ? -EIO : 0;
}

static irqreturn_t falcon_nand_irq(int irq, void *devid)
{
	int stat;
	unsigned long flags;
	struct falcon_mtd_s *dev = devid;

	local_irq_save(flags);
	stat = bios_svc(0x17, NULL, 0, 0, 0, 0);
	local_irq_restore(flags);

	if (stat == INT_DONE_WAKEUP || stat == INT_ERR) {
		dev->irq_handled = 1;
		wake_up(&dev->irq_waitq);
	}

	return IRQ_HANDLED;
}

static int falcon_nand_probe(struct platform_device *pdev)
{
	struct nand_chip *this;
	struct mtd_info *mtd;
	struct flash_platform_data *flash = pdev->dev.platform_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	struct mtd_part_parser_data *p_ppdata = NULL;
	struct mtd_part_parser_data ppdata = {
		.of_node = pdev->dev.of_node,
	};
#else
	int nr_parts = 0;
#endif
	int irq_num;

	int err = 0;

	DEBUG(MTD_DEBUG_LEVEL3, "FALCON_NAND: %s\n", __func__);

	if (!flash) {
		flash = kmalloc(sizeof(struct flash_platform_data), GFP_KERNEL);
		if (!flash) {
			err = -ENOMEM;
			goto out;
		}
	}

	falcon_nand_data = kmalloc(sizeof(struct falcon_mtd_s), GFP_KERNEL);
	if (!falcon_nand_data) {
		err = -ENOMEM;
		goto out;
	}
	memset(falcon_nand_data, 0, sizeof(struct falcon_mtd_s));

	falcon_nand_data->p_data_buf = kmalloc(falcon_host_param->pagesize +
					       falcon_host_param->sparesize +
					       NAND_BUFF_ALIGN, GFP_KERNEL);
	if (!falcon_nand_data->p_data_buf) {
		err = -ENOMEM;
		goto out_1;
	}
	falcon_nand_data->data_buf =
		(char *)NAND_BUFF_ALIGNMENT(falcon_nand_data->p_data_buf);

	falcon_nand_data->dev = &pdev->dev;

	this = &falcon_nand_data->nand;
	mtd = &falcon_nand_data->mtd;
	mtd->priv = this;
	mtd->owner = THIS_MODULE;

	this->chip_delay = 5;

	this->priv = falcon_nand_data;
	this->dev_ready = falcon_nand_dev_ready;
	this->cmdfunc = falcon_nand_command;
	this->select_chip = falcon_nand_select_chip;
	this->read_byte = falcon_nand_read_byte;
	this->read_word = falcon_nand_read_word;
	this->write_buf = falcon_nand_write_buf;
	this->read_buf = falcon_nand_read_buf;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 0)
	this->verify_buf = falcon_nand_verify_buf;
#endif
	this->waitfunc = falcon_nand_wait;

	this->ecc.calculate = falcon_nand_calculate_ecc;
	this->ecc.hwctl = falcon_nand_enable_hwecc;
	this->ecc.correct = falcon_nand_correct_data;
	this->ecc.mode = NAND_ECC_HW;
	this->ecc.size = falcon_host_param->pagesize;
	this->ecc.bytes = falcon_host_param->sparesize;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
	this->ecc.strength = falcon_host_param->ecc_strength;
#endif
	this->ecc.layout = falcon_host_param->nand_oob_layout;
	this->ecc.read_page = falcon_nand_read_page;
	this->ecc.write_page = falcon_nand_write_page;
	this->ecc.write_oob = falcon_nand_write_oob;
	irq_num = falcon_host_param->irq;

	if (irq_num > -1) {
		init_waitqueue_head(&falcon_nand_data->irq_waitq);
		falcon_set_wait_queue(&falcon_nand_data->irq_waitq,
				      &falcon_nand_data->irq_handled);

		err = request_irq(irq_num, falcon_nand_irq, 0, "falconnand",
				  falcon_nand_data);
		if (err) {
			pr_err("falcon nand: request_irq is failed.\n");
			falcon_nand_data->irq_num = -1;
			err = -EBUSY;
			goto out_2;
		}
		falcon_nand_data->irq_num = irq_num;

		pr_info("%s: Use interrupt : IRQ=%d\n",
			__func__, falcon_nand_data->irq_num);
	} else {
		falcon_nand_data->irq_num = -1;
		pr_info("%s: Not Use interrupt\n", __func__);
	}

	mutex_init(&falcon_nand_data->transaction_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	if (nand_scan_ident(mtd, 1, NULL)) {
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
	if (nand_scan_ident(mtd, 1)) {
#else
	if (nand_scan(mtd, 1)) {
#endif
		DEBUG(MTD_DEBUG_LEVEL0,
		      "FALCON_NAND: Unable to find any NAND device.\n");
		err = -ENXIO;
		goto out_2;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
	if (nand_scan_tail(mtd)) {
		DEBUG(MTD_DEBUG_LEVEL0,
		      "FALCON_NAND: Failed to finish NAND scan.\n");
		err = -ENXIO;
		goto out_2;
	}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	if (falcon_host_param->ppdata_flag)
		p_ppdata = &ppdata;
	mtd_device_parse_register(mtd, falcon_host_param->part_probes,
				  p_ppdata, flash->parts, flash->nr_parts);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	nr_parts = parse_mtd_partitions(mtd, falcon_host_param->part_probes,
					&falcon_nand_data->parts, 0);
	if (nr_parts > 0)
		mtd_device_register(mtd, falcon_nand_data->parts, nr_parts);
	else if (flash->parts)
		mtd_device_register(mtd, flash->parts, flash->nr_parts);
	else {
		pr_info("Registering %s as whole device\n", mtd->name);
		mtd_device_register(mtd, NULL, 0);
	}
#else
#ifdef CONFIG_MTD_PARTITIONS
	nr_parts =
	    parse_mtd_partitions(mtd, falcon_host_param->part_probes,
				 &falcon_nand_data->parts, 0);
	if (nr_parts > 0)
		add_mtd_partitions(mtd, falcon_nand_data->parts, nr_parts);
	else if (flash->parts)
		add_mtd_partitions(mtd, flash->parts, flash->nr_parts);
	else
#endif
	{
		pr_info("Registering %s as whole device\n", mtd->name);
		add_mtd_device(mtd);
	}
#endif

	if (!pdev->dev.platform_data)
		pdev->dev.platform_data = flash;

	platform_set_drvdata(pdev, mtd);
	return 0;

out_2:
	if (falcon_nand_data->irq_num > -1)
		free_irq(falcon_nand_data->irq_num, falcon_nand_data);

	kfree(falcon_nand_data->p_data_buf);
out_1:
	kfree(falcon_nand_data);
out:
	if (flash && !pdev->dev.platform_data)
		kfree(flash);

	return err;

}

static int __exit falcon_nand_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (falcon_nand_data) {
		nand_release(mtd);
		kfree(falcon_nand_data->p_data_buf);
		kfree(falcon_nand_data);
	}

	return 0;
}

#ifdef CONFIG_PM

static int falcon_nand_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mtd_info *info = platform_get_drvdata(pdev);
	int ret = 0;

	DEBUG(MTD_DEBUG_LEVEL0, "FALCON_NAND : NAND suspend\n");

	mutex_lock(&falcon_nand_data->transaction_lock);

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

static int falcon_nand_resume(struct platform_device *pdev)
{
	struct mtd_info *info = platform_get_drvdata(pdev);
	int ret = 0;

	DEBUG(MTD_DEBUG_LEVEL0, "FALCON_NAND : NAND resume\n");

	if (falcon_nand_data->irq_num != -1) {
		falcon_nand_data->irq_handled = 0;
		wait_event_timeout(falcon_nand_data->irq_waitq,
				   falcon_nand_data->irq_handled, 1);
	}

	if (info) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
		mtd_resume(info);
#else
		info->resume(info);
#endif
	}

	falcon_storage_resume();

	mutex_unlock(&falcon_nand_data->transaction_lock);

	return ret;
}

#else
#define falcon_nand_suspend   NULL
#define falcon_nand_resume    NULL
#endif

static struct platform_driver falcon_nand_driver = {
	.driver = {
	},
	.probe = falcon_nand_probe,
	.remove = __exit_p(falcon_nand_remove),
	.suspend = falcon_nand_suspend,
	.resume = falcon_nand_resume,
};

static int __init falcon_nand_init(void)
{
	falcon_blk_platform_init();

	if (falcon_storage_init(NULL. 0)) {
		pr_err("falcon_storage_init error\n");
		return -EINVAL;
	}

	if (!can_use_falcon()) {
		pr_err("Falcon BIOS is not found.\n");
		return -EINVAL;
	}

	falcon_host_param = falcon_nand_get_hostinfo();
	if (!falcon_host_param) {
		pr_err("Falcon NAND host info is not got.\n");
		return -EINVAL;
	}
	falcon_nand_driver.driver.name = falcon_host_param->name;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	falcon_nand_driver.driver.of_match_table =
		of_match_ptr(falcon_host_param->of_mtable);
#endif

	init_storage_common();

	return platform_driver_register(&falcon_nand_driver);
}

static void __exit falcon_nand_cleanup(void)
{

	platform_driver_unregister(&falcon_nand_driver);
}

module_init(falcon_nand_init);
module_exit(falcon_nand_cleanup);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("Falcon NAND MTD driver");
MODULE_LICENSE("GPL");
