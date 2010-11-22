/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <mach/dmaengine.h>

#include "dma-apbh.h"

static void *mxs_dma_pool;
static int mxs_dma_alignment = MXS_DMA_ALIGNMENT;

/*
 * The mutex that arbitrates access to the array of structures that represent
 * all the DMA channels in the system (see mxs_dma_channels, below).
 */

static DEFINE_MUTEX(mxs_dma_mutex);

/*
 * The list of DMA drivers that manage various DMA channels. A DMA device
 * driver registers to manage DMA channels by calling mxs_dma_device_register().
 */

static LIST_HEAD(mxs_dma_devices);

/*
 * The array of struct mxs_dma_chan that represent every DMA channel in the
 * system. The index of the structure in the array indicates the specific DMA
 * hardware it represents (see mach-mx28/include/mach/dma.h).
 */

static struct mxs_dma_chan mxs_dma_channels[MXS_MAX_DMA_CHANNELS];

int mxs_dma_request(int channel, struct device *dev, const char *name)
{
	int ret = 0;
	struct mxs_dma_chan *pchan;

	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return -EINVAL;

	if (!dev || !name)
		return -EINVAL;
	pchan = mxs_dma_channels + channel;
	mutex_lock(&mxs_dma_mutex);
	if ((pchan->flags & MXS_DMA_FLAGS_VALID) != MXS_DMA_FLAGS_VALID) {
		ret = -ENODEV;
		goto out;
	}
	if (pchan->flags & MXS_DMA_FLAGS_ALLOCATED) {
		ret = -EBUSY;
		goto out;
	}
	pchan->flags |= MXS_DMA_FLAGS_ALLOCATED;
	pchan->name = name;
	pchan->dev = (unsigned long)dev;
	pchan->active_num = 0;
	pchan->pending_num = 0;
	spin_lock_init(&pchan->lock);
	INIT_LIST_HEAD(&pchan->active);
	INIT_LIST_HEAD(&pchan->done);
out:
	mutex_unlock(&mxs_dma_mutex);
	return ret;
}
EXPORT_SYMBOL(mxs_dma_request);

void mxs_dma_release(int channel, struct device *dev)
{
	struct mxs_dma_chan *pchan;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return;
	pchan = mxs_dma_channels + channel;

	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return;

	if (pchan->flags & MXS_DMA_FLAGS_BUSY)
		return;

	if (pchan->dev != (unsigned long)dev)
		return;

	mutex_lock(&mxs_dma_mutex);
	pchan->dev = 0;
	pchan->active_num = 0;
	pchan->pending_num = 0;
	pchan->flags &= ~MXS_DMA_FLAGS_ALLOCATED;
	mutex_unlock(&mxs_dma_mutex);
}
EXPORT_SYMBOL(mxs_dma_release);

int mxs_dma_enable(int channel)
{
	int ret = 0;
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return -EINVAL;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return -EINVAL;

	pdma = pchan->dma;
	spin_lock_irqsave(&pchan->lock, flags);
	if (pchan->pending_num && pdma->enable)
		ret = pdma->enable(pchan, channel - pdma->chan_base);
	pchan->flags |= MXS_DMA_FLAGS_BUSY;
	spin_unlock_irqrestore(&pchan->lock, flags);
	return ret;
}
EXPORT_SYMBOL(mxs_dma_enable);

void mxs_dma_disable(int channel)
{
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return;
	if (!(pchan->flags & MXS_DMA_FLAGS_BUSY))
		return;
	pdma = pchan->dma;
	spin_lock_irqsave(&pchan->lock, flags);
	if (pdma->disable)
		pdma->disable(pchan, channel - pdma->chan_base);
	pchan->flags &= ~MXS_DMA_FLAGS_BUSY;
	pchan->active_num = 0;
	pchan->pending_num = 0;
	list_splice_init(&pchan->active, &pchan->done);
	spin_unlock_irqrestore(&pchan->lock, flags);
}
EXPORT_SYMBOL(mxs_dma_disable);

int mxs_dma_get_info(int channel, struct mxs_dma_info *info)
{
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
#ifdef DUMPING_DMA_ALL
	struct list_head *pos;
	int value, i, offset;
#endif

	if (!info)
		return -EINVAL;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return -EINVAL;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return -EFAULT;
	pdma = pchan->dma;
	if (pdma->info)
		pdma->info(pdma, channel - pdma->chan_base, info);

#ifdef DUMPING_DMA_ALL
	printk(KERN_INFO "\n[ %s ] channel : %d ,active : %d, pending : %d\n",
		__func__, channel, pchan->active_num, pchan->pending_num);

	for (i = 0; i < 6; i++) {
		offset = i * 0x10 + channel * 0x70;
		value = __raw_readl(pdma->base + offset);
		printk(KERN_INFO "[ %s ] offset : 0x%.3x --  0x%.8x\n",
				__func__, offset, value);
	}
	for (i = 0; i < 7; i++) {
		offset = 0x100 + i * 0x10 + channel * 0x70;
		value = __raw_readl(pdma->base + offset);
		printk(KERN_INFO "[ %s ] offset : 0x%.3x --  0x%.8x\n",
				__func__, offset, value);
	}

	offset = 0;
	list_for_each(pos, &pchan->active) {
		struct mxs_dma_desc *pdesc;

		pdesc = list_entry(pos, struct mxs_dma_desc, node);
		printk(KERN_INFO "========================================\n");
		printk(KERN_INFO "The whole chain of CMD %d is :\n"
				"\tNEXT_COMMAND_ADDRESS	: 0x%.8lx\n"
				"\tCMD			: 0x%.8lx\n"
				"\tDMA Buffer		: 0x%.8x\n"
				"\taddress		: 0x%.8x\n",
				offset++,
				pdesc->cmd.next,
				pdesc->cmd.cmd.data,
				(int)pdesc->cmd.address,
				(int)pdesc->address);

		for (i = 0; i < pdesc->cmd.cmd.bits.pio_words; i++) {
			printk(KERN_INFO "PIO WORD [ %d ] --> 0x%.8lx\n",
				i, pdesc->cmd.pio_words[i]);
		}
		printk(KERN_INFO "==================================\n");
	}
#endif
	return 0;
}
EXPORT_SYMBOL(mxs_dma_get_info);

int mxs_dma_cooked(int channel, struct list_head *head)
{
	int sem;
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct list_head *p, *q;
	struct mxs_dma_desc *pdesc;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return -EINVAL;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return -EINVAL;

	sem = mxs_dma_read_semaphore(channel);
	if (sem < 0)
		return sem;
	if (sem == pchan->active_num)
		return 0;
	BUG_ON(sem > pchan->active_num);
	spin_lock_irqsave(&pchan->lock, flags);
	list_for_each_safe(p, q, &pchan->active) {
		if ((pchan->active_num) <= sem)
			break;
		pdesc = list_entry(p, struct mxs_dma_desc, node);
		pdesc->flags &= ~MXS_DMA_DESC_READY;
		if (head)
			list_move_tail(p, head);
		else
			list_move_tail(p, &pchan->done);
		if (pdesc->flags & MXS_DMA_DESC_LAST)
			pchan->active_num--;
	}
	if (sem == 0)
		pchan->flags &= ~MXS_DMA_FLAGS_BUSY;
	spin_unlock_irqrestore(&pchan->lock, flags);

	BUG_ON(sem != pchan->active_num);
	return 0;
}
EXPORT_SYMBOL(mxs_dma_cooked);

void mxs_dma_reset(int channel)
{
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return;
	pdma = pchan->dma;
	spin_lock_irqsave(&pchan->lock, flags);
	if (pdma->reset)
		pdma->reset(pdma, channel - pdma->chan_base);
	spin_unlock_irqrestore(&pchan->lock, flags);
}
EXPORT_SYMBOL(mxs_dma_reset);

void mxs_dma_freeze(int channel)
{
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return;
	pdma = pchan->dma;
	spin_lock_irqsave(&pchan->lock, flags);
	if (pdma->freeze)
		pdma->freeze(pdma, channel - pdma->chan_base);
	spin_unlock_irqrestore(&pchan->lock, flags);
}
EXPORT_SYMBOL(mxs_dma_freeze);

void mxs_dma_unfreeze(int channel)
{
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return;
	pdma = pchan->dma;
	spin_lock_irqsave(&pchan->lock, flags);
	if (pdma->unfreeze)
		pdma->unfreeze(pdma, channel - pdma->chan_base);
	spin_unlock_irqrestore(&pchan->lock, flags);
}
EXPORT_SYMBOL(mxs_dma_unfreeze);

int mxs_dma_read_semaphore(int channel)
{
	int ret = -EINVAL;
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return ret;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return ret;
	pdma = pchan->dma;
	spin_lock_irqsave(&pchan->lock, flags);
	if (pdma->read_semaphore)
		ret = pdma->read_semaphore(pdma, channel - pdma->chan_base);
	spin_unlock_irqrestore(&pchan->lock, flags);
	return ret;
}
EXPORT_SYMBOL(mxs_dma_read_semaphore);

void mxs_dma_enable_irq(int channel, int en)
{
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return;
	pdma = pchan->dma;
	spin_lock_irqsave(&pchan->lock, flags);
	if (pdma->enable_irq)
		pdma->enable_irq(pdma, channel - pdma->chan_base, en);
	spin_unlock_irqrestore(&pchan->lock, flags);
}
EXPORT_SYMBOL(mxs_dma_enable_irq);

int mxs_dma_irq_is_pending(int channel)
{
	int ret = 0;
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return ret;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return ret;
	pdma = pchan->dma;
	spin_lock_irqsave(&pchan->lock, flags);
	if (pdma->irq_is_pending)
		ret = pdma->irq_is_pending(pdma, channel - pdma->chan_base);
	spin_unlock_irqrestore(&pchan->lock, flags);
	return ret;
}
EXPORT_SYMBOL(mxs_dma_irq_is_pending);

void mxs_dma_ack_irq(int channel)
{
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return;
	pdma = pchan->dma;
	spin_lock_irqsave(&pchan->lock, flags);
	if (pdma->ack_irq)
		pdma->ack_irq(pdma, channel - pdma->chan_base);
	spin_unlock_irqrestore(&pchan->lock, flags);
}
EXPORT_SYMBOL(mxs_dma_ack_irq);

void mxs_dma_set_target(int channel, int target)
{
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return;
	if (pchan->flags & MXS_DMA_FLAGS_BUSY)
		return;
	pdma = pchan->dma;
	spin_lock_irqsave(&pchan->lock, flags);
	if (pdma->set_target)
		pdma->set_target(pdma, channel - pdma->chan_base, target);
	spin_unlock_irqrestore(&pchan->lock, flags);
}
EXPORT_SYMBOL(mxs_dma_set_target);

/* mxs dma utility function */
struct mxs_dma_desc *mxs_dma_alloc_desc(void)
{
	struct mxs_dma_desc *pdesc;
	unsigned int address;
	if (mxs_dma_pool == NULL)
		return NULL;

	pdesc = dma_pool_alloc(mxs_dma_pool, GFP_KERNEL, &address);
	if (pdesc == NULL)
		return NULL;
	memset(pdesc, 0, sizeof(*pdesc));
	pdesc->address = address;
	return pdesc;
};
EXPORT_SYMBOL(mxs_dma_alloc_desc);

void mxs_dma_free_desc(struct mxs_dma_desc *pdesc)
{
	if (pdesc == NULL)
		return;

	if (mxs_dma_pool == NULL)
		return;

	dma_pool_free(mxs_dma_pool, pdesc, pdesc->address);
}
EXPORT_SYMBOL(mxs_dma_free_desc);

int mxs_dma_desc_append(int channel, struct mxs_dma_desc *pdesc)
{
	int ret = 0;
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_desc *last;
	struct mxs_dma_device *pdma;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return -EINVAL;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return -EINVAL;
	pdma = pchan->dma;
	pdesc->cmd.next = mxs_dma_cmd_address(pdesc);
	pdesc->flags |= MXS_DMA_DESC_FIRST | MXS_DMA_DESC_LAST;
	spin_lock_irqsave(&pchan->lock, flags);
	if (!list_empty(&pchan->active)) {

		last = list_entry(pchan->active.prev,
				  struct mxs_dma_desc, node);

		pdesc->flags &= ~MXS_DMA_DESC_FIRST;
		last->flags &= ~MXS_DMA_DESC_LAST;

		last->cmd.next = mxs_dma_cmd_address(pdesc);
		last->cmd.cmd.bits.chain = 1;
	}
	pdesc->flags |= MXS_DMA_DESC_READY;
	if (pdesc->flags & MXS_DMA_DESC_FIRST)
		pchan->pending_num++;
	list_add_tail(&pdesc->node, &pchan->active);
	spin_unlock_irqrestore(&pchan->lock, flags);
	return ret;
}
EXPORT_SYMBOL(mxs_dma_desc_append);

int mxs_dma_desc_add_list(int channel, struct list_head *head)
{
	int ret = 0, size = 0;
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	struct mxs_dma_device *pdma;
	struct list_head *p;
	struct mxs_dma_desc *prev = NULL, *pcur;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return -EINVAL;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return -EINVAL;

	if (list_empty(head))
		return 0;

	pdma = pchan->dma;
	list_for_each(p, head) {
		pcur = list_entry(p, struct mxs_dma_desc, node);
		if (!(pcur->cmd.cmd.bits.dec_sem || pcur->cmd.cmd.bits.chain))
			return -EINVAL;
		if (prev)
			prev->cmd.next = mxs_dma_cmd_address(pcur);
		else
			pcur->flags |= MXS_DMA_DESC_FIRST;
		pcur->flags |= MXS_DMA_DESC_READY;
		prev = pcur;
		size++;
	}
	pcur = list_first_entry(head, struct mxs_dma_desc, node);
	prev->cmd.next = mxs_dma_cmd_address(pcur);
	prev->flags |= MXS_DMA_DESC_LAST;

	spin_lock_irqsave(&pchan->lock, flags);
	if (!list_empty(&pchan->active)) {
		pcur = list_entry(pchan->active.next,
				  struct mxs_dma_desc, node);
		if (pcur->cmd.cmd.bits.dec_sem != prev->cmd.cmd.bits.dec_sem) {
			ret = -EFAULT;
			goto out;
		}
		prev->cmd.next = mxs_dma_cmd_address(pcur);
		prev = list_entry(pchan->active.prev,
				  struct mxs_dma_desc, node);
		pcur = list_first_entry(head, struct mxs_dma_desc, node);
		pcur->flags &= ~MXS_DMA_DESC_FIRST;
		prev->flags &= ~MXS_DMA_DESC_LAST;
		prev->cmd.next = mxs_dma_cmd_address(pcur);
	}
	list_splice(head, &pchan->active);
	pchan->pending_num += size;
	if (!(pcur->cmd.cmd.bits.dec_sem) && (pcur->flags & MXS_DMA_DESC_FIRST))
		pchan->pending_num += 1;
	else
		pchan->pending_num += size;
out:
	spin_unlock_irqrestore(&pchan->lock, flags);
	return ret;
}
EXPORT_SYMBOL(mxs_dma_desc_add_list);

int mxs_dma_get_cooked(int channel, struct list_head *head)
{
	unsigned long flags;
	struct mxs_dma_chan *pchan;
	if ((channel < 0) || (channel >= MXS_MAX_DMA_CHANNELS))
		return -EINVAL;
	pchan = mxs_dma_channels + channel;
	if (!(pchan->flags & MXS_DMA_FLAGS_ALLOCATED))
		return -EINVAL;

	if (head == NULL)
		return 0;

	spin_lock_irqsave(&pchan->lock, flags);
	list_splice(&pchan->done, head);
	spin_unlock_irqrestore(&pchan->lock, flags);
	return 0;
}
EXPORT_SYMBOL(mxs_dma_get_cooked);

int mxs_dma_device_register(struct mxs_dma_device *pdev)
{
	int i;
	struct mxs_dma_chan *pchan;

	if (pdev == NULL || !pdev->chan_num)
		return -EINVAL;

	if ((pdev->chan_base >= MXS_MAX_DMA_CHANNELS) ||
	    ((pdev->chan_base + pdev->chan_num) > MXS_MAX_DMA_CHANNELS))
		return -EINVAL;

	mutex_lock(&mxs_dma_mutex);
	pchan = mxs_dma_channels + pdev->chan_base;
	for (i = 0; i < pdev->chan_num; i++, pchan++) {
		pchan->dma = pdev;
		pchan->flags = MXS_DMA_FLAGS_VALID;
	}
	list_add(&pdev->node, &mxs_dma_devices);
	mutex_unlock(&mxs_dma_mutex);
	return 0;
}
EXPORT_SYMBOL(mxs_dma_device_register);

static int __init mxs_dma_alignment_setup(char *line)
{
	get_option(&line, &mxs_dma_alignment);
	mxs_dma_alignment = (mxs_dma_alignment + 3) & (~3);
	mxs_dma_alignment = max(mxs_dma_alignment, MXS_DMA_ALIGNMENT);
	return 1;
};

__setup("mxs-dma-alignment=", mxs_dma_alignment_setup);

static int mxs_dmaengine_init(void)
{
	mxs_dma_pool = dma_pool_create("mxs_dma", NULL,
				       sizeof(struct mxs_dma_desc),
				       mxs_dma_alignment, PAGE_SIZE);
	if (mxs_dma_pool == NULL)
		return -ENOMEM;
	return 0;
}

subsys_initcall(mxs_dmaengine_init);

#ifdef CONFIG_PROC_FS

static void *mxs_dma_proc_seq_start(struct seq_file *file, loff_t * index)
{
	if (*index >= MXS_MAX_DMA_CHANNELS)
		return NULL;
	return mxs_dma_channels + *index;
}

static void *mxs_dma_proc_seq_next(struct seq_file *file, void *data,
				   loff_t *index)
{
	if (data == NULL)
		return NULL;

	if (*index >= MXS_MAX_DMA_CHANNELS)
		return NULL;

	return mxs_dma_channels + (*index)++;
}

static void mxs_dma_proc_seq_stop(struct seq_file *file, void *data)
{
}

static int mxs_dma_proc_seq_show(struct seq_file *file, void *data)
{
	int result;
	struct mxs_dma_chan *pchan = (struct mxs_dma_chan *)data;
	struct mxs_dma_device *pdev = pchan->dma;
	result = seq_printf(file, "%s-channel%-d	(%s)\n",
			    pdev->name,
			    pchan - mxs_dma_channels,
			    pchan->name ? pchan->name : "idle");
	return result;
}

static const struct seq_operations mxc_dma_proc_seq_ops = {
	.start = mxs_dma_proc_seq_start,
	.next = mxs_dma_proc_seq_next,
	.stop = mxs_dma_proc_seq_stop,
	.show = mxs_dma_proc_seq_show
};

static int mxs_dma_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &mxc_dma_proc_seq_ops);
}

static const struct file_operations mxs_dma_proc_info_ops = {
	.open = mxs_dma_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int __init mxs_dmaengine_info_init(void)
{
	struct proc_dir_entry *res;
	res = create_proc_entry("dma-engine", 0, NULL);
	if (!res) {
		printk(KERN_ERR "Failed to create dma info file \n");
		return -ENOMEM;
	}
	res->proc_fops = &mxs_dma_proc_info_ops;
	return 0;
}

late_initcall(mxs_dmaengine_info_init);
#endif
