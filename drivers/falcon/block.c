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
#include <linux/platform_device.h>
#include <linux/blkdev.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/slab.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#if defined(CONFIG_LAB126)
#include <linux/delay.h>
#endif

#include <asm/falcon_syscall.h>
#include <linux/falcon_storage.h>
#include "falcon_common.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
#include <linux/mmc/ioctl.h>
#include <linux/mmc/mmc.h>
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Falcon Block device driver");
MODULE_AUTHOR("");

#define THREADING_IDLELOAD

#if 0
#define dprintk(...) {				\
		pr_err("%s:\t", __func__);	\
		pr_err(__VA_ARGS__);		\
	}
#else
#define dprintk(...)
#endif

#define FALCON_BLK_DRV_NAME		"falconblk"
#define FALCON_BLK_DEV_NAME		"mmcblk1"
#define FALCON_BLK_BOOTDEV0_NAME	FALCON_BLK_DEV_NAME"boot0"
#define FALCON_BLK_BOOTDEV1_NAME	FALCON_BLK_DEV_NAME"boot1"
#define FALCON_BLK_RPMBDEV_NAME		FALCON_BLK_DEV_NAME"rpmb"
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0)
#ifdef CONFIG_FALCON_BLK_BOOTPART
#warning BootPartition is unsupported less than Linux 3.0
#endif
#define FALCON_BLK_DEV_MAX		1
#else
#ifdef CONFIG_FALCON_BLK_BOOTPART
#ifdef CONFIG_FALCON_BLK_RPMB
#define FALCON_BLK_DEV_MAX		4
#else
#define FALCON_BLK_DEV_MAX		3
#endif
#else
#ifdef CONFIG_FALCON_BLK_RPMB
#define FALCON_BLK_DEV_MAX		2
#else
#define FALCON_BLK_DEV_MAX		1
#endif
#endif
#endif
#define SECTOR_SIZE			512
#define FALCON_MINORS			16

#define EXEC_TIMEOUT			(HZ / 2)

#define FALCON_BLK_WORKQUEUE_NICE	-20

#define MMC_RSP_MASK				(0x1f << 0)
#define MMC_RSP_PRESENT				(1 << 0)
#define MMC_RSP_136				(1 << 1)
#define MMC_RSP_CRC				(1 << 2)
#define MMC_RSP_BUSY				(1 << 3)
#define MMC_RSP_OPCODE				(1 << 4)

#if defined(CONFIG_LAB126)
extern void usdhc2_clock_disable(void); 
extern void usdhc2_clock_enable(void);
#define CMD_FLAG_RESTYP_NONE 		(0x00000000)
#endif
#define CMD_FLAG_RESTYP_R1			(0x00000001)
#define CMD_FLAG_RESTYP_R1b			(0x00000002)
#define CMD_FLAG_RESTYP_R2			(0x00000003)
#define CMD_FLAG_RESTYP_R3			(0x00000004)
#define CMD_FLAG_RESTYP_R4			(0x00000005)
#define CMD_FLAG_RESTYP_R5			(0x00000006)
#define CMD_FLAG_RESTYP_R5b			(0x00000007)
#define CMD_FLAG_RESTYP_R6			(0x00000008)

#define CMD_FLAG_DATA_NONE			(0x00000000)
#define CMD_FLAG_DATA_READ			(0x00000010)
#define CMD_FLAG_DATA_WRITE			(0x00000020)

#define CMD_FLAG_DATA_SG			(0x00000040)

#define CMD_FLAG_RWR_REQ			(0x10000000)
#define CMD_FLAG_ACMD_REQ			(0x20000000)

const int falcon_blk_partnum[FALCON_BLK_DEV_MAX] = {
	PT_DATA_PART,
#ifdef CONFIG_FALCON_BLK_BOOTPART
	PT_BOOT_PART1,
	PT_BOOT_PART2,
#endif
#ifdef CONFIG_FALCON_BLK_RPMB
	PT_RPMB_PART,
#endif
};

#ifdef CONFIG_ARM_LPAE
typedef u64 fpaddr_t;
#else
typedef u32 fpaddr_t;
#endif

struct falcon_sglist {
	fpaddr_t		paddr;
	u32			length;
	u32			offset;
};

struct falcon_sginfo {
	u32			size;
	struct falcon_sglist	*list;
};

struct falcon_blkinfo {
	u32			part;
};

struct falcon_cmdinfo {
	u32 cmd;
	u32 arg;
	u32 res[4];
	u32 flag;
	void *data;
	u16 block_size;
	u16 block_count;
};

struct falcon_blk_work {
	struct work_struct	work;
	struct falcon_blk_req	*req;
};

struct falcon_blk_host {

	const char		*name;
	unsigned int		max_seg_size;
	unsigned short		max_hw_segs;
	unsigned short		max_phys_segs;
	unsigned int		max_req_size;
	unsigned int		max_blk_size;
	unsigned int		max_blk_count;

	int			irq;

	u64			dma_mask;

	unsigned char		heads;
	unsigned char		sectors;
};

struct falcon_blk_part {
	int			part_num;

	struct falcon_blk_dev	*dev;

	struct request_queue	*queue;
	struct gendisk		*gd;
	spinlock_t		lock;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	struct device_attribute	force_ro;
#endif
};

struct falcon_blk_req {
	struct falcon_blk_part	*part;

	struct request		*req;

	struct scatterlist	*sg;
	char			*bounce_buf;
	struct scatterlist	*bounce_sg;
	unsigned int		bounce_sg_len;

	struct falcon_sginfo	falcon_sg;

	int			count;
	int			size;
	int			error;

	struct falcon_blkinfo	*falcon_binfo;

	enum req_states {
		RS_NON = 0,
		RS_GET,
		RS_WAIT,
		RS_RUN,
		RS_RUNNING,
		RS_DONE,
	}			state;
};

struct falcon_blk_dev {
	struct device		*dev;
	struct storage_info	storage;
	struct falcon_blk_host	*host;

	unsigned int		bounce_sg_len;

	int			use_irq;
	int			irq_handled;
	struct semaphore	falcon_sem;
	wait_queue_head_t	irq_waitq;
	int			storage_active;

	struct task_struct	*thread;
	enum thread_states {
		TS_NON = 0,
		TS_SLEEP,
		TS_GET_REQ,
		TS_PRE_REQ,
		TS_RUN_REQ,
		TS_POST_REQ,
	}			thread_state;
	unsigned int		thread_flags;
	spinlock_t		thread_lock;

	struct workqueue_struct	*workq;
	struct falcon_blk_work	falcon_work;

	struct falcon_blk_req	falcon_req[2];
	struct falcon_blk_req	*forward_req;
	struct falcon_blk_req	*backward_req;

	struct falcon_blk_part	*part[FALCON_BLK_DEV_MAX];
};

static int do_request(struct falcon_blk_dev *dev)
{
	int ret;
	int wait_stat;

	if (dev->use_irq) {

		dev->irq_handled = 0;
		ret = bios_svc(0x14, WAIT_IRQ, 0, 0, 0, 0);

		while (ret == RC_OK) {

			wait_stat = wait_event_timeout(dev->irq_waitq,
						       dev->irq_handled,
						       EXEC_TIMEOUT);

			if (wait_stat < 0)
				pr_err("%s: ERROR: wait event error\n",
				       __func__);

			dev->irq_handled = 0;
			ret = bios_svc(0x14, WAIT_IRQ, 0, 0, 0, 0);

		}
	} else {
		ret = bios_svc(0x14, WAIT_POLLING, 0, 0, 0, 0);
		while (ret == RC_OK) {
			set_user_nice(current, 10);
			schedule();
			ret = bios_svc(0x14, WAIT_POLLING, 0, 0, 0, 0);
		}
	}

	return ret;
}

static unsigned int falcon_blk_flush(struct falcon_blk_dev *dev,
				     struct falcon_blk_req *req)
{
	int ret;

	falcon_blk_platform_pre();

	ret = bios_svc(0x13, RT_FLUSH, NULL, NULL, NULL, (req) ? req->falcon_binfo : NULL);

	if (ret)
		pr_err("falcon_blk: request error!! ret=%d\n", ret);
	else
		ret = do_request(dev);

	if (ret != RC_DONE) {
		pr_err("falcon_blk: flush error!! ret=%d\n", ret);
	}

	falcon_blk_platform_post();

	if (req)
		req->error = ret;

	return ret != RC_DONE;
}

#ifdef CONFIG_FALCON_BLK_MULTISEG

static unsigned int falcon_blk_queue_map_sg(struct falcon_blk_req *req)
{
	struct request *rq = req->req;
	struct falcon_blk_part *part = req->part;
	unsigned int sg_len;
	size_t buflen;
	struct scatterlist *sg;
	int i;

	if (!req->bounce_buf)
		return blk_rq_map_sg(part->queue, rq, req->sg);

	BUG_ON(!req->bounce_sg);

	sg_len = blk_rq_map_sg(part->queue, rq, req->bounce_sg);

	req->bounce_sg_len = sg_len;

	buflen = 0;
	for_each_sg(req->bounce_sg, sg, sg_len, i) {
		buflen += sg->length;
		dprintk("segment %d: %d\n", i, sg->length);
	}

	sg_init_one(req->sg, req->bounce_buf, buflen);

	return 1;
}

static void falcon_blk_queue_bounce_pre(struct falcon_blk_req *req)
{
	struct request *rq = req->req;
	unsigned long flags;

	if (!req->bounce_buf)
		return;

	if (rq_data_dir(rq) != WRITE)
		return;

	local_irq_save(flags);
	sg_copy_to_buffer(req->bounce_sg, req->bounce_sg_len,
			  req->bounce_buf, req->sg[0].length);
	local_irq_restore(flags);
}

static void falcon_blk_queue_bounce_post(struct falcon_blk_req *req)
{
	struct request *rq = req->req;

	unsigned long flags;

	if (!req->bounce_buf)
		return;

	if (rq_data_dir(rq) != READ)
		return;

	local_irq_save(flags);
	sg_copy_from_buffer(req->bounce_sg, req->bounce_sg_len,
			    req->bounce_buf, req->sg[0].length);
	local_irq_restore(flags);
}

static void falcon_map_sg(struct falcon_blk_req *req)
{
	struct request *rq = req->req;
	struct falcon_blk_part *part = req->part;
	struct falcon_blk_dev *dev = part->dev;

	int nents;

	nents = dma_map_sg(dev->dev, req->sg, req->count,
			   rq_data_dir(rq) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	BUG_ON(nents != req->count);
}

static void falcon_unmap_sg(struct falcon_blk_req *req)
{
	struct request *rq = req->req;
	struct falcon_blk_part *part = req->part;
	struct falcon_blk_dev *dev = part->dev;

	dma_unmap_sg(dev->dev, req->sg, req->count,
		     rq_data_dir(rq) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

}

static void prepare_falcon_sglist(struct falcon_blk_req *req)
{
	struct falcon_sglist *falcon_sg = req->falcon_sg.list;
	struct scatterlist *sg = NULL;
	int i;
	unsigned int size = 0;
	unsigned int count = req->count;

	for_each_sg(req->sg, sg, count, i) {

		BUG_ON(sg == NULL);

		falcon_sg->paddr = (fpaddr_t)sg_dma_address(sg);
		falcon_sg->offset = sg->offset;
		falcon_sg->length = sg_dma_len(sg);

		size += falcon_sg->length;

		falcon_sg++;
	}

	req->falcon_sg.size = size;
}

static void pre_request_sg(struct falcon_blk_req *req)
{
	struct falcon_blk_part *part = req->part;

#ifdef CONFIG_FALCON_BLK_MULTISEG
	req->count = falcon_blk_queue_map_sg(req);

	BUG_ON(req->count > part->dev->host->max_phys_segs);

	falcon_blk_queue_bounce_pre(req);

	falcon_map_sg(req);

	prepare_falcon_sglist(req);

	req->size = req->falcon_sg.size;
#endif

	req->falcon_binfo->part = falcon_blk_partnum[part->part_num];
}

static bool post_request_sg(struct falcon_blk_req *req)
{
	int is_left;
	unsigned long flags;

	struct falcon_blk_part *part = req->part;
	struct request *rq = req->req;

#ifdef CONFIG_FALCON_BLK_MULTISEG
	falcon_unmap_sg(req);

	falcon_blk_queue_bounce_post(req);
#endif

	spin_lock_irqsave(part->queue->queue_lock, flags);
	is_left = __blk_end_request(rq, req->error != RC_DONE, req->size);
	spin_unlock_irqrestore(part->queue->queue_lock, flags);

	return is_left;
}

static void process_request_sg(struct work_struct *w)
{
	struct falcon_blk_work *work = (struct falcon_blk_work *)w;
	struct falcon_blk_req *req =  work->req;
	struct falcon_blk_part *part = req->part;
	struct falcon_blk_dev *dev = part->dev;
	struct request *rq = req->req;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
	unsigned long pos = rq->sector;
#else
	unsigned long pos = blk_rq_pos(rq);
#endif
	int thread_wakeup_flag = 0;
	unsigned long flags;
	int i;

	down(&dev->falcon_sem);

	req->state = RS_RUNNING;

	dprintk("enter\n");

	dprintk("  %c: start sector:%lu, segment count:%u, size:0x%x\n",
		rq_data_dir(rq) ? 'W' : 'R',
		pos, req->count, req->falcon_sg.size);

	if (req->falcon_sg.size || !(rq->cmd_flags & REQ_FLUSH)) {
		falcon_blk_platform_pre();

		if (req->bounce_buf)
			req->error =
				bios_svc(0x13, rq_data_dir(rq) ?
						     RT_WRITE : RT_READ, pos, req->bounce_buf, sg_dma_len(req->sg) / 512, req->falcon_binfo);
		else
			req->error =
				bios_svc(0x13, rq_data_dir(rq) ?
						     RT_WRITE_SG : RT_READ_SG, pos, &req->falcon_sg, req->count, req->falcon_binfo);

		if (req->error)
			pr_err("falcon_blk: request error!! ret=%d\n", req->error);
		else
			req->error = do_request(dev);

		dprintk("ret = %d\n", req->error);

		falcon_blk_platform_post();
	}
	else
		req->error = RC_DONE;

	if ((rq->cmd_flags & REQ_FLUSH) && (req->error == RC_DONE))
		falcon_blk_flush(dev, req);

	spin_lock_irqsave(&dev->thread_lock, flags);
	req->state = RS_DONE;
	dev->storage_active = 0;

	if (dev->thread_state == TS_SLEEP)
		thread_wakeup_flag = 1;

	spin_unlock_irqrestore(&dev->thread_lock, flags);

	up(&dev->falcon_sem);

	for (i = FALCON_BLK_WORKQUEUE_NICE; i < 20; i++) {
		if (can_nice(current, i)) {
			set_user_nice(current, i);
			break;
		}
	}

	if (thread_wakeup_flag)
		wake_up_process(dev->thread);
}

#else

static int get_first_rq_size(struct request *rq)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
	unsigned long sectors = rq->hard_cur_sectors;
#else
	unsigned long sectors = blk_rq_cur_sectors(rq);
#endif

#if 1

	struct bio_vec *bv;
	int size = 0;
	struct req_iterator iter;
	unsigned int start_addr = (unsigned int)rq->buffer;

	rq_for_each_segment(bv, rq, iter) {

		if ((unsigned int)(page_address(bv->bv_page) + bv->bv_offset) ==
			start_addr + size) {
			size += bv->bv_len;
		} else

			goto end;
	}

end:
	if (size)
		return size / SECTOR_SIZE;
	else
		return sectors;
#else
	return rq->hard_cur_sectors;
#endif
}

static void pre_request(struct falcon_blk_req *req)
{
	struct falcon_blk_part *part = req->part;

	req->falcon_binfo->part = falcon_blk_partnum[part->part_num];
}

static bool post_request(struct falcon_blk_req *req)
{
	int is_left;
	unsigned long flags;

	struct falcon_blk_part *part = req->part;
	struct request *rq = req->req;

	spin_lock_irqsave(part->queue->queue_lock, flags);
	is_left = __blk_end_request(rq, req->error != RC_DONE, req->size);
	spin_unlock_irqrestore(part->queue->queue_lock, flags);

	return is_left;
}

static void process_request(struct work_struct *w)
{
	struct falcon_blk_work *work = (struct falcon_blk_work *)w;
	struct falcon_blk_req *req =  work->req;
	struct falcon_blk_part *part = req->part;
	struct falcon_blk_dev *dev = part->dev;
	struct request *rq = req->req;
	int size = get_first_rq_size(rq);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
	unsigned long pos = rq->sector;
#else
	unsigned long pos = blk_rq_pos(rq);
#endif
	int thread_wakeup_flag = 0;
	unsigned long flags;

	down(&dev->falcon_sem);

	req->state = RS_RUNNING;

	if (size || !(rq->cmd_flags & REQ_FLUSH)) {
		falcon_blk_platform_pre();

		dprintk("enter\n");

		dprintk("  %c: sector:%lu, size:%u, buff:%p\n",
			rq_data_dir(rq) ? 'W' : 'R',
			pos, size, rq->buffer);

		req->error = bios_svc(0x13, rq_data_dir(rq) ?
						  RT_WRITE : RT_READ, pos, rq->buffer, size, req->falcon_binfo);

		if (req->error)
			pr_err("falcon_blk: request error!!  ret=%d\n",
			       req->error);
		else
			req->error = do_request(dev);

		dprintk("ret = %d\n", req->error);

		falcon_blk_platform_post();
	}
	else
		req->error = RC_DONE

	if ((rq->cmd_flags & REQ_FLUSH) && (req->error == RC_DONE))
		falcon_blk_flush(dev, req);

	spin_lock_irqsave(&dev->thread_lock, flags);
	req->state = RS_DONE;
	dev->storage_active = 0;

	if (dev->thread_state == TS_SLEEP)
		thread_wakeup_flag = 1;

	spin_unlock_irqrestore(&dev->thread_lock, flags);

	up(&dev->falcon_sem);

	if (thread_wakeup_flag)
		wake_up_process(dev->thread);
}

#endif

static irqreturn_t falcon_blk_irq(int irq, void *devid)
{
	int stat;
	unsigned long flags;
	struct falcon_blk_dev *dev = devid;

	local_irq_save(flags);
	stat = bios_svc(0x17, NULL, 0, 0, 0, 0);
	local_irq_restore(flags);

	if (stat == INT_DONE_WAKEUP || stat == INT_ERR) {
		dev->irq_handled = 1;
		wake_up(&dev->irq_waitq);
	}

	return IRQ_HANDLED;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
static int falcon_blk_request_cmd_call(
	struct block_device *bdev,
	struct falcon_blk_part *part,
	struct mmc_ioc_cmd __user *p_ioc_cmd)
{
	int result = 0;
	struct falcon_blk_dev *dev = part->dev;

	struct mmc_ioc_cmd ioc_cmd;
	struct falcon_cmdinfo cmdinfo = {};
	int res_flags, flags;
	void *data = NULL;
	int data_size;

	struct falcon_blkinfo falcon_binfo = {
		falcon_blk_partnum[part->part_num],
	};

	if (down_timeout(&dev->falcon_sem, HZ))
		return -EINTR;

	if (copy_from_user(&ioc_cmd, p_ioc_cmd, sizeof(ioc_cmd))) {
		result = -EFAULT;
		goto err1;
	}

	res_flags = ioc_cmd.flags &
		(MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC |
		 MMC_RSP_BUSY | MMC_RSP_OPCODE);
	switch (res_flags) {
	case MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE:
		flags = CMD_FLAG_RESTYP_R1;
		break;
	case MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE | MMC_RSP_BUSY:
		flags = CMD_FLAG_RESTYP_R1b;
		break;
	case MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC:
		flags = CMD_FLAG_RESTYP_R2;
		break;
	case MMC_RSP_PRESENT:
		flags = CMD_FLAG_RESTYP_R3;
		break;
	default:
		pr_warn("falcon_blk: Unknown res type for mmc_ioc_cmd.\n");
		result = -EINVAL;
		goto err1;
	}

	data_size = ioc_cmd.blksz * ioc_cmd.blocks;
	if (data_size > MMC_IOC_MAX_BYTES) {
		result = -EOVERFLOW;
		goto err1;
	}

	if (data_size) {
		if (ioc_cmd.write_flag) {
			flags |= CMD_FLAG_DATA_WRITE;
			if (ioc_cmd.write_flag & 0x80000000)
				flags |= CMD_FLAG_RWR_REQ;
		} else
			flags |= CMD_FLAG_DATA_READ;

		data = kzalloc(data_size, GFP_KERNEL);
		if (!data) {
			result = -ENOMEM;
			goto err1;
		}

		if (copy_from_user(
			    data,
			    (void __user *)(unsigned long) ioc_cmd.data_ptr,
			    data_size)) {
			result = -EFAULT;
			goto err2;
		}
	}

	if (ioc_cmd.is_acmd)
		cmdinfo.flag = CMD_FLAG_ACMD_REQ;

	cmdinfo.cmd = ioc_cmd.opcode;
	cmdinfo.arg = ioc_cmd.arg;
	cmdinfo.flag = flags;
	cmdinfo.data = data;
	cmdinfo.block_size = ioc_cmd.blksz;
	cmdinfo.block_count = ioc_cmd.blocks;

	falcon_blk_platform_pre();

	result = bios_svc(0x13, RT_CMD_CALL, 0, &cmdinfo, 0, &falcon_binfo);
	if (result)
		pr_err("falcon_blk: request error!! ret=%d\n", result);
	else
		result = do_request(dev);

	falcon_blk_platform_post();

	if (result != RC_DONE) {
		result = -EIO;
		goto err2;
	}

	if (ioc_cmd.postsleep_min_us)
		pr_info("falcon_blk: Non support \"postsleep\"\n");

	if (copy_to_user(p_ioc_cmd->response, ioc_cmd.response,
			 sizeof(ioc_cmd.response))) {
		result = -EFAULT;
		goto err2;
	}

	if (data && !ioc_cmd.write_flag)
		if (copy_to_user(
			    (void __user *)(unsigned long) ioc_cmd.data_ptr,
			    data, data_size))
			result = -EFAULT;

err2:
	kfree(data);
err1:
	up(&dev->falcon_sem);

	return result;
}
#endif

#if defined(CONFIG_LAB126)
void __dump_lastk_to_mmc(char *buf, unsigned long start, unsigned int size)
{
	struct falcon_cmdinfo cmdinfo = {};
    int result = 0;
    int i = 0;
    int j = 0;
    struct falcon_blkinfo falcon_binfo = {
        falcon_blk_partnum[0],
    };

	if (size > 1)
		cmdinfo.cmd = MMC_WRITE_MULTIPLE_BLOCK;
	else
		cmdinfo.cmd = MMC_WRITE_BLOCK;

    cmdinfo.arg = start;
    cmdinfo.flag = CMD_FLAG_DATA_WRITE | CMD_FLAG_RESTYP_R1b;
    cmdinfo.data = buf;
    cmdinfo.block_size = SECTOR_SIZE;
    cmdinfo.block_count = size;

    falcon_blk_platform_pre();
    for (j = 0; j < 10; j++) {
	    result = bios_svc(0x13, RT_WRITE, cmdinfo.arg , cmdinfo.data, cmdinfo.block_count, &falcon_binfo);
	    if (result)
	        pr_info("falcon_blk: request FAILED! ret=%d\n", result);
	    else{
	        pr_info("falcon_blk: request OK! ret=%d at j=%d\n", result, j);
	        for (i = 0; i < 100; i++) {
	        	bios_svc(0x14, WAIT_POLLING, 0, 0, 0, 0);
	        }
	        break;
	    }
	}
    falcon_blk_platform_post();
}
EXPORT_SYMBOL(__dump_lastk_to_mmc);

static void __mmc_sleep_step_1(void)
{
	struct falcon_cmdinfo cmdinfo = {};
    int result = 0;
    int i = 0;
    int j = 0;
    struct falcon_blkinfo falcon_binfo = {
        falcon_blk_partnum[0],
    };

	cmdinfo.cmd = MMC_SELECT_CARD;
    cmdinfo.arg = 0;
    cmdinfo.flag = CMD_FLAG_RESTYP_NONE;

    falcon_blk_platform_pre();
    for (j = 0; j < 10; j++) {
	    result = bios_svc(0x13, RT_CMD_CALL, 0, &cmdinfo, 0, &falcon_binfo);
	    if (result)
	        pr_info("falcon_blk: request FAILED! ret=%d\n", result);
	    else{
	        pr_info("falcon_blk: request OK! ret=%d at j=%d\n", result, j);
	        for (i = 0; i < 10; i++) {
	        	bios_svc(0x14, WAIT_POLLING, 0, 0, 0, 0);
	        }
	        break;
	    }
	}
    falcon_blk_platform_post();
}

static void __mmc_sleep_step_0(void)
{
	struct falcon_cmdinfo cmdinfo = {};
    int result = 0;
    int i = 0;
    int j = 0;
    struct falcon_blkinfo falcon_binfo = {
        falcon_blk_partnum[0],
    };

	cmdinfo.cmd = MMC_SWITCH;
    cmdinfo.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) | (EXT_CSD_POWER_OFF_NOTIFICATION << 16) | (EXT_CSD_NOTIFICATION_SLEEP << 8);
    cmdinfo.flag = CMD_FLAG_RESTYP_R1b;

    falcon_blk_platform_pre();
    for (j = 0; j < 10; j++) {
	    result = bios_svc(0x13, RT_CMD_CALL, 0, &cmdinfo, 0, &falcon_binfo);
	    if (result)
	        pr_info("falcon_blk: request FAILED! ret=%d\n", result);
	    else{
	        pr_info("falcon_blk: request OK! ret=%d at j=%d\n", result, j);
	        for (i = 0; i < 10; i++) {
	        	bios_svc(0x14, WAIT_POLLING, 0, 0, 0, 0);
	        }
	        break;
	    }
	}
    falcon_blk_platform_post();
}

static void __mmc_sleep_step_2(void)
{
	struct falcon_cmdinfo cmdinfo = {};
    int result = 0;
    int i = 0;
    int j = 0;
    struct falcon_blkinfo falcon_binfo = {
        falcon_blk_partnum[0],
    };

	cmdinfo.cmd = MMC_SLEEP_AWAKE;
    cmdinfo.arg = (0x1 << 16) | (0x1 << 15);
    cmdinfo.flag = CMD_FLAG_RESTYP_R1b;
    falcon_blk_platform_pre();
    for (j = 0; j < 10; j++) {
	    result = bios_svc(0x13, RT_CMD_CALL, 0, &cmdinfo, 0, &falcon_binfo);
	    if (result)
	        pr_info("falcon_blk: request FAILED! ret=%d\n", result);
	    else{
	        pr_info("falcon_blk: request OK! ret=%d at j=%d\n", result, j);
	        for (i = 0; i < 10; i++) {
	        	bios_svc(0x14, WAIT_POLLING, 0, 0, 0, 0);
	        }
	        break;
	    }
	}
    falcon_blk_platform_post();
}

static void __mmc_suspend(void)
{
	__mmc_sleep_step_0();
	__mmc_sleep_step_1();
	__mmc_sleep_step_2();
	usdhc2_clock_disable();
}

static void __mmc_resume(void)
{
	usdhc2_clock_enable();
}

void mmc_shipmode(void)
{
	__mmc_sleep_step_0();
	__mmc_sleep_step_1();
	__mmc_sleep_step_2();
	//__mmc_suspend(); //do not disable clock otherwise system clock will not available and the last ship command may not get to pmic
}
EXPORT_SYMBOL(mmc_shipmode);
#endif


static int falcon_blk_request_sbios_call(
	struct falcon_blk_part *part,
	struct falcon_blk_sbios_call_container __user *arg)
{
	struct falcon_blk_dev *dev = part->dev;
	struct falcon_blk_sbios_call_container container;
	struct falcon_blkinfo falcon_binfo = {
		falcon_blk_partnum[part->part_num],
	};
	void *data = NULL;
	int result = -EINVAL;
	int ret;

	if (down_timeout(&dev->falcon_sem, HZ)) {
		result = -EINTR;
		return result;
	}

	if (copy_from_user(&container, (void *)arg, sizeof(container)))
		goto end;

	if (container.size == 0)
		goto end;

	data = kmalloc(container.size, GFP_KERNEL);
	if (!data) {
		result = -ENOMEM;
		goto end;
	}

	if (copy_from_user(data, container.ptr, container.size))
		goto end;

	falcon_blk_platform_pre();

	ret = bios_svc(0x13, RT_SBIOS_CALL, 0, data, container.size, &falcon_binfo);
	if (ret)
		pr_err("falcon_blk: request error!! ret=%d\n", ret);
	else
		ret = do_request(dev);

	falcon_blk_platform_post();

	if (ret != RC_DONE) {
		if (ret == RC_BUSY)
			result = -EBUSY;
		else
			result = -EIO;

		goto end;
	}

	result = -EINVAL;
	if (copy_to_user(container.ptr, data, container.size))
		goto end;

	result = 0;
end:
	kfree(data);

	up(&dev->falcon_sem);

	return result;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
static int _falcon_blk_ioctl(struct falcon_blk_part *part, unsigned int cmd,
			     unsigned long arg)
#else
static int _falcon_blk_ioctl(struct block_device *bdev,
			     struct falcon_blk_part *part,
			     unsigned int cmd, unsigned long arg)
#endif
{
	int ret = -EINVAL;

	switch (cmd) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	case MMC_IOC_CMD:
		ret = falcon_blk_request_cmd_call(bdev, part, (void *)arg);
		break;
#endif
	case F_SBIOS_CALL:
		ret = falcon_blk_request_sbios_call(part, (void *)arg);
		break;
	default:
		break;
	}
	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
static int falcon_blk_ioctl(struct inode *inode, struct file *filp,
			    unsigned int cmd, unsigned long arg)

{
	struct falcon_blk_part *part = filp->private_data;

	return _falcon_blk_ioctl(part, cmd, arg);
}
#else
static int falcon_blk_ioctl(struct block_device *bdev, fmode_t mode,
			    unsigned int cmd, unsigned long arg)
{
	struct falcon_blk_part *part = bdev->bd_disk->private_data;

	return _falcon_blk_ioctl(bdev, part, cmd, arg);
}
#endif

static int falcon_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct falcon_blk_part *part = bdev->bd_disk->private_data;
	unsigned int capacity = get_capacity(bdev->bd_disk);

	geo->heads = part->dev->host->heads;
	geo->sectors = part->dev->host->sectors;
	geo->cylinders = capacity / (geo->heads * geo->sectors);

	return 0;
}

static int get_queue_req(struct falcon_blk_dev *dev, struct falcon_blk_req *req)
{
	static int part_num = FALCON_BLK_DEV_MAX;

	int start_part = -1;
	unsigned int part_flag;
	unsigned long queue_flags, thread_flags;

	struct request_queue *q;

	do {
		part_num++;
		if (part_num >= FALCON_BLK_DEV_MAX)
			part_num = 0;

		if (start_part == -1)
			start_part = part_num;
		else if (part_num == start_part)
			break;

		part_flag = 1 << part_num;
		if (!(dev->thread_flags & part_flag))
			continue;

		req->part = dev->part[part_num];

		do {
			q = req->part->queue;
			spin_lock_irqsave(q->queue_lock, queue_flags);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
			if (!blk_queue_plugged(q)) {
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
				req->req = elv_next_request(q);
#else
				req->req = blk_fetch_request(q);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
			}
#endif

			if (req->req == NULL) {
				spin_lock_irqsave(&dev->thread_lock,
						  thread_flags);
				dev->thread_flags &= ~part_flag;
				spin_unlock_irqrestore(&dev->thread_lock,
						       thread_flags);
				spin_unlock_irqrestore(q->queue_lock,
						       queue_flags);
				break;
			}
			spin_unlock_irqrestore(q->queue_lock, queue_flags);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
			if (!blk_fs_request(req->req)) {
#else
			if (req->req->cmd_type != REQ_TYPE_FS) {
#endif
				pr_notice("Skip non-fs request\n");
				BUG_ON(req->req->cmd_flags & REQ_FLUSH);
				spin_lock_irqsave(q->queue_lock, queue_flags);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
				end_request(req->req, 0);
#else
				__blk_end_request_cur(req->req, -EIO);
#endif
				spin_unlock_irqrestore(q->queue_lock,
						       queue_flags);
				req->req = NULL;
				continue;
			}
		} while (!req->req);
	} while (!req->req);

	return !req->req;
}

static int falcon_blk_queue_thread(void *d)
{
	struct falcon_blk_dev *dev = d;

	schedule();

	do {
		struct falcon_blk_req *forward = dev->forward_req;
		struct falcon_blk_req *backward = dev->backward_req;
		struct falcon_blk_req *req = NULL;
		unsigned long flags;
		int break_flag = 0;

		spin_lock_irqsave(&dev->thread_lock, flags);

		if (((forward->state == RS_WAIT) ||
		     (backward->state == RS_WAIT)) &&
		    !((forward->state == RS_RUN) ||
		      (forward->state == RS_RUNNING) ||
		      (backward->state == RS_RUN) ||
		      (backward->state == RS_RUNNING))) {
			dev->thread_state = TS_RUN_REQ;

			if (forward->state == RS_WAIT)
				req = forward;
			else
				req = backward;
		} else if ((forward->state == RS_GET) ||
			   (backward->state == RS_GET)) {
			dev->thread_state = TS_PRE_REQ;

			if (forward->state == RS_GET)
				req = forward;
			else
				req = backward;
		} else if ((dev->thread_flags &
			    ((1 << FALCON_BLK_DEV_MAX) - 1)) &&
			   ((forward->state == RS_NON) ||
			    (backward->state == RS_NON)) &&
			   !kthread_should_stop()) {
			dev->thread_state = TS_GET_REQ;

			if (forward->state == RS_NON)
				req = forward;
			else
				req = backward;
		} else if (forward->state == RS_DONE) {
			dev->thread_state = TS_POST_REQ;
			req = forward;
		} else if (((forward->state == RS_NON) &&
			    (backward->state == RS_NON)) &&
			   kthread_should_stop()) {
			dev->thread_state = TS_SLEEP;
			break_flag = 1;
		} else {
			set_current_state(TASK_INTERRUPTIBLE);
			dev->thread_state = TS_SLEEP;
		}
		spin_unlock_irqrestore(&dev->thread_lock, flags);

		if (break_flag)
			break;

		switch (dev->thread_state) {
		case TS_SLEEP:
			schedule();
			set_current_state(TASK_RUNNING);
			break;

		case TS_GET_REQ:
			if (!get_queue_req(dev, req))
				req->state = RS_GET;
			break;

		case TS_PRE_REQ:
#ifdef CONFIG_FALCON_BLK_MULTISEG
			pre_request_sg(req);
#else
			pre_request(req);
#endif
			req->state = RS_WAIT;
			break;

		case TS_RUN_REQ:
			if (!dev->storage_active) {
				dev->storage_active = 1;
				req->state = RS_RUN;
				dev->falcon_work.req = req;
				queue_work(dev->workq, &dev->falcon_work.work);
				if (req->state == RS_RUN)
					schedule();
			}
			break;

		case TS_POST_REQ:
#ifdef CONFIG_FALCON_BLK_MULTISEG
			if (post_request_sg(req)) {
#else
			if (post_request(req)) {
#endif
				req->state = RS_GET;
			} else {
				req->state = RS_NON;
				req->req = NULL;

				dev->forward_req = backward;
				dev->backward_req = forward;
			}
			break;

		default:
			pr_err("falcon_blk: thread error!! %d\n",
			       dev->thread_state);
			dev->thread_state = TS_SLEEP;

			break;
		}
	} while (1);

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)) && \
	defined(CONFIG_FALCON_BLK_RPMB)
static int falcon_blk_prep_request(struct request_queue *q, struct request *req)
{
	struct falcon_blk_part *part = q->queuedata;

	if (falcon_blk_partnum[part->part_num] == PT_RPMB_PART)
		return BLKPREP_KILL;

	req->cmd_flags |= REQ_DONTPREP;

	return BLKPREP_OK;
}
#endif

static void falcon_blk_request(struct request_queue *q)
{
	struct falcon_blk_part *part = q->queuedata;
	struct falcon_blk_dev *dev = part->dev;
	struct request *req;
	int thread_wakeup_flag = 0;
	unsigned long flags;

	if (!part) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
		while ((req = elv_next_request(q)) != NULL) {
			int ret;

			do {
				ret = __blk_end_request(req, -EIO,
							blk_rq_cur_bytes(req));
			} while (ret);
		}
#else
		while ((req = blk_fetch_request(q)) != NULL) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
			req->cmd_flags |= REQ_QUIET;
#endif
			__blk_end_request_all(req, -EIO);
		}
#endif
		return;
	}

	spin_lock_irqsave(&dev->thread_lock, flags);
	dev->thread_flags |= (1 << part->part_num);

	if ((dev->thread_state == TS_SLEEP) &&
	    ((dev->forward_req->state == RS_NON) ||
	     (dev->backward_req->state == RS_NON)))
		thread_wakeup_flag = 1;
	spin_unlock_irqrestore(&dev->thread_lock, flags);

	if (thread_wakeup_flag)
		wake_up_process(dev->thread);

}

static int falcon_blk_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct falcon_blk_dev *dev = platform_get_drvdata(pdev);

	down(&dev->falcon_sem);
	if (dev->storage.cache_en)
		if (falcon_blk_flush(dev, NULL))
			return 1;

#if defined(CONFIG_LAB126)
	__mmc_suspend();
#endif
	falcon_blk_platform_suspend();
	pr_info("%s: done\n", __func__);
	return 0;
}

static int falcon_blk_resume(struct platform_device *pdev)
{
	struct falcon_blk_dev *dev = platform_get_drvdata(pdev);

#if defined(CONFIG_LAB126)
	__mmc_resume();
#endif
	if (dev->use_irq) {
		dev->irq_handled = 0;
		wait_event_timeout(dev->irq_waitq, dev->irq_handled, 1);
	}

	falcon_storage_resume();

	falcon_blk_platform_resume();
	up(&dev->falcon_sem);

	pr_info("%s: done\n", __func__);
	return 0;
}

static struct platform_device *falcon_blk_platform_device;

static struct platform_driver falcon_blk_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = FALCON_BLK_DRV_NAME,
		},
	.suspend = falcon_blk_suspend,
	.resume = falcon_blk_resume,
};

static const struct block_device_operations falcon_blk_ops = {
	.ioctl = falcon_blk_ioctl,
	.getgeo = falcon_getgeo,
	.owner = THIS_MODULE,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
static ssize_t force_ro_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "%d\n",
		       get_disk_ro(dev_to_disk(dev)));
	return ret;
}

static ssize_t force_ro_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	int ret;
	u32 set;

	if (kstrtou32(buf, 10, &set)) {
		ret = -EINVAL;
		goto out;
	}

	set_disk_ro(dev_to_disk(dev), set);
	ret = count;
out:
	return ret;
}
#endif

static struct falcon_blk_host * __init get_host_info(void)
{
	struct falcon_blk_host *host;
	struct falcon_blk_host_param *p;

	host = kmalloc(sizeof(struct falcon_blk_host), GFP_KERNEL);
	if (!host)
		return NULL;

	p = falcon_blk_get_hostinfo();

	if (p) {
		pr_info("%s: Found target host info\n", __func__);
		host->name = p->name;
		host->max_hw_segs = p->max_hw_segs;
		host->max_phys_segs = p->max_phys_segs;
		host->max_seg_size = p->max_seg_size;

		host->max_req_size = p->max_req_size;
		host->max_blk_size = p->max_blk_size;
		host->max_blk_count = p->max_blk_count;

		host->irq = p->irq;

		host->dma_mask = p->dma_mask;

		if (p->heads && p->sectors) {
			host->heads = p->heads;
			host->sectors = p->sectors;
		} else {

			host->heads = 4;
			host->sectors = 16;
		}
	} else {

		host->name = "falcon_blk default";
		host->max_hw_segs = 1;
		host->max_phys_segs = 32;
		host->max_seg_size = 128 * 1024;

		host->max_req_size = 128 * 1024;
		host->max_blk_size = SECTOR_SIZE;
		host->max_blk_count = host->max_req_size / SECTOR_SIZE;

		host->irq = 0;

		host->dma_mask = 0xffffffffULL;

		host->heads = 4;
		host->sectors = 16;
	}

	return host;
}

static int __init create_fb_req(struct falcon_blk_req *req,
				unsigned int max_phys_segs,
				unsigned int bouncesz)
{
	if (max_phys_segs == 0) {
		pr_err("falcon_blk: max_hw_segs is invalid value\n");
		return -EINVAL;
	}

	if (max_phys_segs == 1) {
		if (bouncesz > 512) {
			req->bounce_buf = kmalloc(bouncesz, GFP_DMA);
			if (!req->bounce_buf)
				return -ENOMEM;
		}

		req->sg = kmalloc(sizeof(struct scatterlist), GFP_KERNEL);
		if (!req->sg)
			return -ENOMEM;

		sg_init_table(req->sg, 1);

		req->bounce_sg =
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
			kmalloc(sizeof(struct scatterlist) * bouncesz / 512,
				GFP_KERNEL);
#else
			kmalloc_array(bouncesz / 512,
				      sizeof(struct scatterlist),
				      GFP_KERNEL);
#endif
		if (!req->bounce_sg)
			return -ENOMEM;
		sg_init_table(req->bounce_sg, bouncesz / 512);
	}

	if (!req->bounce_buf) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
		req->sg = kmalloc(sizeof(struct scatterlist) *  max_phys_segs,
				  GFP_KERNEL);
#else
		req->sg = kmalloc_array(max_phys_segs,
					sizeof(struct scatterlist),
					GFP_KERNEL);
#endif
		if (!req->sg)
			return -ENOMEM;
		sg_init_table(req->sg, max_phys_segs);
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
	req->falcon_sg.list =
		kmalloc(sizeof(struct falcon_sglist) * max_phys_segs,
			GFP_KERNEL);
#else
	req->falcon_sg.list = kmalloc_array(max_phys_segs,
					    sizeof(struct falcon_sglist),
					    GFP_KERNEL);
#endif
	if (!req->falcon_sg.list)
		return -ENOMEM;

	req->falcon_binfo = kmalloc(sizeof(struct falcon_blkinfo), GFP_KERNEL);
	if (!req->falcon_binfo)
		return -ENOMEM;

	return 0;
}

static void delete_fb_req(struct falcon_blk_req *req)
{
	kfree(req->bounce_buf);
	kfree(req->bounce_sg);
	kfree(req->sg);
	kfree(req->falcon_sg.list);
	kfree(req->falcon_binfo);
}

#define FALCON_BLK_QUEUE_BOUNCESZ	524288
static int __init create_queue(struct falcon_blk_dev *dev,
			       struct falcon_blk_part *part,
			       unsigned int bouncesz)
{
	struct falcon_blk_host *host = dev->host;
	u64 limit = BLK_BOUNCE_HIGH;

	if (host->dma_mask)
		limit = host->dma_mask;

	part->queue = blk_init_queue(falcon_blk_request, &part->lock);
	if (!part->queue) {
		pr_err("falcon_blk: blk_init_queue is failed.\n");
		return -ENOMEM;
	}

	part->queue->queuedata = part;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
	blk_queue_hardsect_size(part->queue, SECTOR_SIZE);
#else
	blk_queue_logical_block_size(part->queue, SECTOR_SIZE);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
	blk_queue_ordered(part->queue, QUEUE_ORDERED_DRAIN, NULL);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)) && \
	defined(CONFIG_FALCON_BLK_RPMB)
	blk_queue_prep_rq(part->queue, falcon_blk_prep_request);
#endif

	if (host->max_hw_segs == 1) {

		if (bouncesz > 512) {
			blk_queue_bounce_limit(part->queue, BLK_BOUNCE_ANY);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
			blk_queue_max_sectors(part->queue, bouncesz / 512);
			blk_queue_max_phys_segments(part->queue,
						    bouncesz / 512);
			blk_queue_max_hw_segments(part->queue, bouncesz / 512);
#else
			blk_queue_max_hw_sectors(part->queue, bouncesz / 512);
			blk_queue_max_segments(part->queue, bouncesz / 512);
#endif
			blk_queue_max_segment_size(part->queue, bouncesz);
		}
	}

	if (!(bouncesz > 512)) {

		blk_queue_bounce_limit(part->queue, limit);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
		blk_queue_max_sectors(part->queue,
				      min(host->max_blk_count,
					  host->max_req_size / 512));
		blk_queue_max_phys_segments(part->queue, host->max_phys_segs);
		blk_queue_max_hw_segments(part->queue, host->max_hw_segs);
#else
		blk_queue_max_hw_sectors(part->queue,
					 min(host->max_blk_count,
					     host->max_req_size / 512));
		blk_queue_max_segments(part->queue, host->max_phys_segs);
#endif
		blk_queue_max_segment_size(part->queue, host->max_seg_size);
	}
	if (dev->storage.cache_en)
		blk_queue_flush(part->queue, REQ_FLUSH | REQ_FUA);

	return 0;
}

static int __init part_setup(struct falcon_blk_dev *dev, int major,
			     int part_num, unsigned int bouncesz)
{
	struct falcon_blk_part *part;
	char const falcon_block_name[FALCON_BLK_DEV_MAX][32] = {
		FALCON_BLK_DEV_NAME,
#ifdef CONFIG_FALCON_BLK_BOOTPART
		FALCON_BLK_BOOTDEV0_NAME,
		FALCON_BLK_BOOTDEV1_NAME,
#endif
#ifdef CONFIG_FALCON_BLK_RPMB
		FALCON_BLK_RPMBDEV_NAME,
#endif
	};
	int ret;

	part = kmalloc(sizeof(*part), GFP_KERNEL);
	if (!part)
		return -ENOMEM;
	memset(part, 0, sizeof(struct falcon_blk_part));

	dev->part[part_num] = part;
	part->part_num = part_num;
	spin_lock_init(&part->lock);

	ret = create_queue(dev, part, bouncesz);
	if (ret) {
		ret = -ENOMEM;
		goto err1;
	}

	part->gd = alloc_disk(FALCON_MINORS);
	if (!part->gd) {
		pr_err("falcon_blk: alloc_disk is failed.\n");
		ret = -ENOMEM;
		goto err2;
	}

	part->gd->major = major;
	part->gd->first_minor = part_num * FALCON_MINORS;
	part->gd->minors = FALCON_MINORS;
	part->gd->fops = &falcon_blk_ops;
	part->gd->queue = part->queue;
	part->gd->private_data = part;

	part->dev = dev;

	strncpy(part->gd->disk_name, falcon_block_name[part_num], 32);

	set_capacity(part->gd, dev->storage.capacity[falcon_blk_partnum[part_num]]);
	if ((falcon_blk_partnum[part_num] == PT_BOOT_PART1) ||
	    (falcon_blk_partnum[part_num] == PT_BOOT_PART2))
		set_disk_ro(part->gd, 0);
	else
		set_disk_ro(part->gd, 0);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (part_num != 0)
		part->gd->flags |= GENHD_FL_NO_PART_SCAN;
#endif
	add_disk(part->gd);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	if ((falcon_blk_partnum[part_num] == PT_BOOT_PART1) ||
	    (falcon_blk_partnum[part_num] == PT_BOOT_PART2)) {

		part->force_ro.show = force_ro_show;
		part->force_ro.store = force_ro_store;
		sysfs_attr_init(&part->force_ro.attr);
		part->force_ro.attr.name = "force_ro";
		part->force_ro.attr.mode = S_IRUGO | S_IWUSR;
		ret = device_create_file(disk_to_dev(part->gd),
					 &part->force_ro);
		if (ret) {
			part->force_ro.show = NULL;
			pr_err("falcon_blk: device_create_file is failed.\n");
			return ret;
		}
	}
#endif

	return 0;

err2:
	blk_cleanup_queue(part->queue);
err1:
	kfree(part);
	dev->part[part_num] = NULL;

	return ret;
}

#ifdef THREADING_IDLELOAD
wait_queue_head_t *falconblk_irq_waitq;
EXPORT_SYMBOL(falconblk_irq_waitq);
int *falconblk_irq_handled;
EXPORT_SYMBOL(falconblk_irq_handled);
struct semaphore	*falconblk_sem;
EXPORT_SYMBOL(falconblk_sem);
#endif

static int __init init_falcon_blk(void)
{
	int ret;
	int major = -1;
	struct falcon_blk_dev *dev = NULL;
	struct falcon_blk_host *host = NULL;
	struct storage_info storage_info;
	int i;
	unsigned int bouncesz = 0;

	falcon_blk_platform_init();
	if (falcon_storage_init(&storage_info, sizeof(storage_info))) {
		pr_err("falcon_storage_init error\n");
		return -EINVAL;
	}

	if (!can_use_falcon()) {
		pr_err("unable to get Falcon storage driver information\n");
		return -EINVAL;
	}

	if (platform_driver_register(&falcon_blk_driver)) {
		pr_err("falcon_blk: unable to register driver\n");
		return -EBUSY;
	}

	falcon_blk_platform_device =
		platform_device_register_simple(FALCON_BLK_DRV_NAME,
						-1, NULL, 0);
	if (IS_ERR(falcon_blk_platform_device)) {
		pr_err("falcon_blk: unable to register platform device\n");
		ret = -EBUSY;
		goto err1;
	}

	major = register_blkdev(0, FALCON_BLK_DRV_NAME);
	if (major <= 0) {
		pr_err("falcon_blk: unable to register block device\n");
		ret = -EBUSY;
		goto err1;
	}

	dev = kmalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err1;
	}
	memset(dev, 0, sizeof(struct falcon_blk_dev));

	platform_set_drvdata(falcon_blk_platform_device, dev);

	dev->dev = &falcon_blk_platform_device->dev;
	memcpy(&dev->storage, &storage_info, sizeof(dev->storage));
	dev->host = get_host_info();
	host = dev->host;

	if (!dev->host) {
		ret = -ENOMEM;
		goto err1;
	}
	dev->dev->dma_mask = &host->dma_mask;

	if (host->irq > -1) {
		init_waitqueue_head(&dev->irq_waitq);
		falcon_set_wait_queue(&dev->irq_waitq, &dev->irq_handled);
#ifdef THREADING_IDLELOAD
		falconblk_irq_waitq = &dev->irq_waitq;
		falconblk_irq_handled = &dev->irq_handled;
#endif
		dev->use_irq = 1;

		ret = request_irq(host->irq, falcon_blk_irq,
				  0, "falcon_blk", dev);
		if (ret) {
			pr_err("falcon_blk: request_irq is failed.\n");
			ret = -EBUSY;
			goto err1;
		}

		pr_info("%s: Use interrupt : IRQ=%d\n", __func__, host->irq);
	} else {
		dev->use_irq = 0;
		pr_info("%s: Not Use interrupt\n", __func__);
	}

	sema_init(&dev->falcon_sem, 1);
	spin_lock_init(&dev->thread_lock);

#ifdef THREADING_IDLELOAD
	falconblk_sem = &dev->falcon_sem;
#endif

	if (host->max_hw_segs == 1) {
		bouncesz = FALCON_BLK_QUEUE_BOUNCESZ;

		if (bouncesz > host->max_req_size)
			bouncesz = host->max_req_size;
		if (bouncesz > host->max_seg_size)
			bouncesz = host->max_seg_size;
		if (bouncesz > (host->max_blk_count * 512))
			bouncesz = host->max_blk_count * 512;
	}

	for (i = 0;
	     i < (sizeof(dev->falcon_req)/sizeof(struct falcon_blk_req)); i++) {
		ret = create_fb_req(&dev->falcon_req[i],
				    host->max_hw_segs, bouncesz);
		if (ret)
			goto err2;

		if (!dev->forward_req)
			dev->forward_req = &dev->falcon_req[i];
		else if (!dev->backward_req)
			dev->backward_req = &dev->falcon_req[i];
	}

	dev->workq = create_singlethread_workqueue("falcon_blkw");
	if (!dev->workq) {
		pr_err("falcon_blk: create_singlethread_workqueue is failed.\n");
		ret = -ENOMEM;
		goto err2;
	}
#ifdef CONFIG_FALCON_BLK_MULTISEG
	INIT_WORK(&dev->falcon_work.work, process_request_sg);
#else
	INIT_WORK(&dev->falcon_work.work, process_request);
#endif

	dev->thread = kthread_run(falcon_blk_queue_thread, dev, "falcon_blkd");
	if (IS_ERR(dev->thread)) {
		pr_err("falcon_blk: kthread_run is failed.\n");
		ret = PTR_ERR(dev->thread);
		goto err3;
	}

	for (i = 0; i < FALCON_BLK_DEV_MAX; i++) {
		if (!dev->storage.capacity[falcon_blk_partnum[i]])
			break;

		ret = part_setup(dev, major, i, bouncesz);

		if (ret)
			goto err4;
	}
	init_storage_common();

	return 0;

err4:
	for (i = 0; i < FALCON_BLK_DEV_MAX; i++) {
		if (dev->part[i]) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
			if (dev->part[i]->force_ro.show)
				device_remove_file(
					disk_to_dev(dev->part[i]->gd),
					&dev->part[i]->force_ro);
#endif
			del_gendisk(dev->part[i]->gd);
		}
	}
	kthread_stop(dev->thread);
	for (i = 0; i < FALCON_BLK_DEV_MAX; i++) {
		if (dev->part[i]) {
			put_disk(dev->part[i]->gd);
			blk_cleanup_queue(dev->part[i]->queue);
			kfree(dev->part[i]);
		}
	}

err3:
	destroy_workqueue(dev->workq);

err2:
	for (i = 0;
	     i < (sizeof(dev->falcon_req)/sizeof(struct falcon_blk_req)); i++)
		delete_fb_req(&dev->falcon_req[i]);

	if (host->irq > -1)
		free_irq(host->irq, dev);

err1:
	if (dev) {
		kfree(dev->host);
		kfree(dev);
		dev = NULL;
	}

	if (major > 0)
		unregister_blkdev(major, FALCON_BLK_DRV_NAME);

	platform_driver_unregister(&falcon_blk_driver);

	pr_err("#### %s: error!!\n", __func__);

	return ret;
}

static void __exit exit_falcon_blk(void)
{
}

module_init(init_falcon_blk);
module_exit(exit_falcon_blk);
