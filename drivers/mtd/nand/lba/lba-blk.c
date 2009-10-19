/*
 * Freescale STMP37XX/STMP378X LBA/block driver
 *
 * Author: Dmitrij Frasenyak <sed@embeddedalley.com>
 *
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/sched.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/types.h>	/* size_t */
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/hdreg.h>	/* HDIO_GETGEO */
#include <linux/kdev_t.h>
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>	/* invalidate_bdev */
#include <linux/bio.h>
#include <linux/dma-mapping.h>
#include <linux/hdreg.h>
#include <linux/blkdev.h>
#include "lba.h"

static int lba_major;

#define LBA_NAME "lba"

#if 0
#define TAG() printk(KERNE_ERR "%s: %d\n", __func__, __LINE__)
#else
#define TAG()
#endif

/*
 * The internal representation of our device.
 */
struct lba_blk_dev {
	int size;                       /* Device size in sectors */
	spinlock_t lock;                /* For mutual exclusion */
	int users;
	struct request_queue *queue;    /* The device request queue */
	struct gendisk *gd;             /* The gendisk structure */
	struct lba_data *data;          /* pointer from lba core */

	struct task_struct	*thread;
	struct bio 		*bio_head;
	struct bio		*bio_tail;
	wait_queue_head_t	wait_q;
	struct semaphore        busy;

};

static struct lba_blk_dev *g_lba_blk;

static void blk_add_bio(struct lba_blk_dev *dev, struct bio *bio);


/*
 * Transfer a single BIO.
 */
static int lba_blk_xfer_bio(struct lba_blk_dev *dev, struct bio *bio)
{
	int i;
	struct bio_vec *bvec;
	sector_t sector = bio->bi_sector;
	enum dma_data_direction dir;
	int status = 0;
	int (*lba_xfer)(void *priv,
			unsigned int sector,
			unsigned int count,
			void *buffer,
			dma_addr_t handle);

	if  (bio_data_dir(bio) == WRITE) {
		lba_xfer = lba_write_sectors;
		dir = DMA_TO_DEVICE;
	} else {
		lba_xfer = lba_read_sectors;
		dir = DMA_FROM_DEVICE;
	}

	/* Fixme: merge segments */
	bio_for_each_segment(bvec, bio, i) {
		void *buffer = page_address(bvec->bv_page);
		dma_addr_t handle ;
		if (!buffer)
			BUG();
		buffer += bvec->bv_offset;
		handle = dma_map_single(&dev->data->nand->dev->dev,
					    buffer,
					    bvec->bv_len,
					    dir);
		status = lba_xfer(dev->data->nand, sector,
				  bvec->bv_len >> 9,
				  buffer,
				  handle);

		dma_unmap_single(&dev->data->nand->dev->dev,
				 handle,
				 bvec->bv_len,
				 dir);
		if (status)
			break;

		sector += bio_cur_bytes(bio) >> 9;
	}

	return status;
}


/*
 * The direct make request version.
 */
static int lba_make_request(struct request_queue *q, struct bio *bio)
{
	struct lba_blk_dev *dev = q->queuedata;

	blk_add_bio(dev, bio);
	return 0;
}

/*
 * Open and close.
 */

static int lba_blk_open(struct block_device *bdev, fmode_t mode)
{
	struct lba_blk_dev *dev = bdev->bd_disk->private_data;

	TAG();

	spin_lock_irq(&dev->lock);
	dev->users++;
	spin_unlock_irq(&dev->lock);
	return 0;
}


static int lba_blk_release(struct gendisk *gd, fmode_t mode)
{
	struct lba_blk_dev *dev = gd->private_data;

	spin_lock(&dev->lock);
	dev->users--;
	spin_unlock(&dev->lock);

	return 0;
}

static int lba_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	/*
	 * get geometry: we have to fake one...  trim the size to a
	 * multiple of 2048 (1M): tell we have 32 sectors, 64 heads,
	 * whatever cylinders.
	 */
	geo->heads = 1 << 6;
	geo->sectors = 1 << 5;
	geo->cylinders = get_capacity(bdev->bd_disk) >> 11;
	return 0;
}

/*
 * Add bio to back of pending list
 */
static void blk_add_bio(struct lba_blk_dev *dev, struct bio *bio)
{
	unsigned long flags;
	spin_lock_irqsave(&dev->lock, flags);
	if (dev->bio_tail) {
		dev->bio_tail->bi_next = bio;
		dev->bio_tail = bio;
	} else
		dev->bio_head = dev->bio_tail = bio;
	wake_up(&dev->wait_q);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/*
 * Grab first pending buffer
 */
static struct bio *blk_get_bio(struct lba_blk_dev *dev)
{
	struct bio *bio;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	bio = dev->bio_head;
	if (bio) {
		if (bio == dev->bio_tail) {
			dev->bio_tail = NULL;
			dev->bio_head = NULL;
		}
		dev->bio_head = bio->bi_next;
		bio->bi_next = NULL;
	}
	spin_unlock_irqrestore(&dev->lock, flags);

	return bio;
}

static int lba_thread(void *data)
{
	struct lba_blk_dev *dev = data;
	struct bio *bio;
	int status;

	set_user_nice(current, -20);

	while (!kthread_should_stop() || dev->bio_head) {

		wait_event_interruptible(dev->wait_q,
				dev->bio_head || kthread_should_stop());

		if (!dev->bio_head)
			continue;

		if (lba_core_lock_mode(dev->data, LBA_MODE_MDP))
			continue;

		bio = blk_get_bio(dev);
		status = lba_blk_xfer_bio(dev, bio);
		bio_endio(bio,  status);

		lba_core_unlock_mode(dev->data);
	}

	return 0;
}



/*
 * The device operations structure.
 */
static struct block_device_operations lba_blk_ops = {
	.owner           = THIS_MODULE,
	.open 	         = lba_blk_open,
	.release 	 = lba_blk_release,
	.getgeo		=  lba_getgeo,
};


int lba_blk_init(struct lba_data *data)
{

	struct lba_blk_dev *dev;
	int err;
	if (!data)
		BUG();

	printk(KERN_INFO "LBA block driver v0.1\n");
	lba_major = LBA_MAJOR;
	dev = g_lba_blk =  kzalloc(sizeof(struct lba_blk_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->data = data;
	register_blkdev(lba_major, "lba");

	spin_lock_init(&dev->lock);
	init_waitqueue_head(&dev->wait_q);
	sema_init(&dev->busy, 1);

	dev->queue = blk_alloc_queue(GFP_KERNEL);
	if (!dev->queue)
		goto out2;
	blk_queue_make_request(dev->queue, lba_make_request);
	/*dev->queue->unplug_fn = lba_unplug_device;*/
	blk_queue_logical_block_size(dev->queue, 512);

	dev->queue->queuedata = dev;
	dev->gd = alloc_disk(32);
	if (!dev->gd) {
		printk(KERN_ERR "failed to alloc disk\n");
		goto out3;
	}
	dev->size = data->mdp_size  ;
	printk(KERN_INFO "%s: set capacity of the device to 0x%x\n",
	       __func__, dev->size);
	dev->gd->major = lba_major;
	dev->gd->first_minor = 0;
	dev->gd->fops = &lba_blk_ops;
	dev->gd->queue = dev->queue;
	dev->gd->private_data = dev;
	snprintf(dev->gd->disk_name, 8, LBA_NAME);
	set_capacity(dev->gd, dev->size);

	dev->thread = kthread_create(lba_thread, dev, "lba-%d", 1);
	if (IS_ERR(dev->thread)) {
		err = PTR_ERR(dev->thread);
		goto out3;
	}
	wake_up_process(dev->thread);

	add_disk(dev->gd);


	TAG();

	return 0;
out3:
out2:
	unregister_blkdev(lba_major, "lba");
	return -ENOMEM;
}

int lba_blk_remove(struct lba_data *data)
{

	struct lba_blk_dev *dev = g_lba_blk;

	del_gendisk(dev->gd);
	kthread_stop(dev->thread);
	blk_cleanup_queue(dev->queue);
	put_disk(dev->gd);

	unregister_blkdev(lba_major, LBA_NAME);
	kfree(dev);
	return 0;
}

MODULE_LICENSE("GPL");
MODULE_ALIAS_BLOCKDEV_MAJOR(254);
