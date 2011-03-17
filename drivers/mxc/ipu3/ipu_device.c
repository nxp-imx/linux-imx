/*
 * Copyright 2005-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file ipu_device.c
 *
 * @brief This file contains the IPUv3 driver device interface and fops functions.
 *
 * @ingroup IPU
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <asm/cacheflush.h>

#include "ipu_prv.h"
#include "ipu_regs.h"
#include "ipu_param_mem.h"

/* Strucutures and variables for exporting MXC IPU as device*/

static int mxc_ipu_major;
static struct class *mxc_ipu_class;

DEFINE_SPINLOCK(event_lock);

struct ipu_dev_irq_info {
	wait_queue_head_t waitq;
	int irq_pending;
} irq_info[480];

int register_ipu_device(void);

/* Static functions */

int get_events(ipu_event_info *p)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&event_lock, flags);
	if (irq_info[p->irq].irq_pending > 0)
		irq_info[p->irq].irq_pending--;
	else
		ret = -1;
	spin_unlock_irqrestore(&event_lock, flags);

	return ret;
}

static irqreturn_t mxc_ipu_generic_handler(int irq, void *dev_id)
{
	irq_info[irq].irq_pending++;

	/* Wakeup any blocking user context */
	wake_up_interruptible(&(irq_info[irq].waitq));
	return IRQ_HANDLED;
}

static int mxc_ipu_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	return ret;
}
static int mxc_ipu_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case IPU_INIT_CHANNEL:
		{
			ipu_channel_parm parm;

			if (copy_from_user
					(&parm, (ipu_channel_parm *) arg,
					 sizeof(ipu_channel_parm)))
				return -EFAULT;

			if (!parm.flag) {
				ret =
					ipu_init_channel(parm.channel,
							&parm.params);
			} else {
				ret = ipu_init_channel(parm.channel, NULL);
			}
		}
		break;
	case IPU_UNINIT_CHANNEL:
		{
		ipu_channel_t ch;
		int __user *argp = (void __user *)arg;
		if (get_user(ch, argp))
				return -EFAULT;
			ipu_uninit_channel(ch);
		}
		break;
	case IPU_INIT_CHANNEL_BUFFER:
		{
			ipu_channel_buf_parm parm;
			if (copy_from_user
				(&parm, (ipu_channel_buf_parm *) arg,
				sizeof(ipu_channel_buf_parm)))
				return -EFAULT;

			ret =
				ipu_init_channel_buffer(
						parm.channel, parm.type,
						parm.pixel_fmt,
						parm.width, parm.height,
						parm.stride,
						parm.rot_mode,
						parm.phyaddr_0,
						parm.phyaddr_1,
						parm.u_offset,
						parm.v_offset);

		}
		break;
	case IPU_UPDATE_CHANNEL_BUFFER:
		{
			ipu_channel_buf_parm parm;
			if (copy_from_user
				(&parm, (ipu_channel_buf_parm *) arg,
				sizeof(ipu_channel_buf_parm)))
				return -EFAULT;

			if ((parm.phyaddr_0 != (dma_addr_t) NULL)
				&& (parm.phyaddr_1 == (dma_addr_t) NULL)) {
				ret =
					ipu_update_channel_buffer(
							parm.channel,
							parm.type,
							parm.bufNum,
							parm.phyaddr_0);
			} else if ((parm.phyaddr_0 == (dma_addr_t) NULL)
				&& (parm.phyaddr_1 != (dma_addr_t) NULL)) {
				ret =
					ipu_update_channel_buffer(
							parm.channel,
							parm.type,
							parm.bufNum,
							parm.phyaddr_1);
			} else {
				ret = -1;
			}

		}
		break;
	case IPU_SELECT_CHANNEL_BUFFER:
		{
			ipu_channel_buf_parm parm;
			if (copy_from_user
				(&parm, (ipu_channel_buf_parm *) arg,
				sizeof(ipu_channel_buf_parm)))
				return -EFAULT;

			ret =
				ipu_select_buffer(parm.channel,
					parm.type, parm.bufNum);

		}
		break;
	case IPU_SELECT_MULTI_VDI_BUFFER:
		{
			uint32_t parm;
			if (copy_from_user
				(&parm, (uint32_t *) arg,
				sizeof(uint32_t)))
				return -EFAULT;

			ret = ipu_select_multi_vdi_buffer(parm);
		}
		break;
	case IPU_LINK_CHANNELS:
		{
			ipu_channel_link link;
			if (copy_from_user
				(&link, (ipu_channel_link *) arg,
				sizeof(ipu_channel_link)))
				return -EFAULT;

			ret = ipu_link_channels(link.src_ch,
				link.dest_ch);

		}
		break;
	case IPU_UNLINK_CHANNELS:
		{
			ipu_channel_link link;
			if (copy_from_user
				(&link, (ipu_channel_link *) arg,
				sizeof(ipu_channel_link)))
				return -EFAULT;

			ret = ipu_unlink_channels(link.src_ch,
				link.dest_ch);

		}
		break;
	case IPU_ENABLE_CHANNEL:
		{
			ipu_channel_t ch;
			int __user *argp = (void __user *)arg;
			if (get_user(ch, argp))
				return -EFAULT;
			ipu_enable_channel(ch);
		}
		break;
	case IPU_DISABLE_CHANNEL:
		{
			ipu_channel_info info;
			if (copy_from_user
				(&info, (ipu_channel_info *) arg,
				 sizeof(ipu_channel_info)))
				return -EFAULT;

			ret = ipu_disable_channel(info.channel,
				info.stop);
		}
		break;
	case IPU_ENABLE_IRQ:
		{
			uint32_t irq;
			int __user *argp = (void __user *)arg;
			if (get_user(irq, argp))
				return -EFAULT;
			ipu_enable_irq(irq);
		}
		break;
	case IPU_DISABLE_IRQ:
		{
			uint32_t irq;
			int __user *argp = (void __user *)arg;
			if (get_user(irq, argp))
				return -EFAULT;
			ipu_disable_irq(irq);
		}
		break;
	case IPU_CLEAR_IRQ:
		{
			uint32_t irq;
			int __user *argp = (void __user *)arg;
			if (get_user(irq, argp))
				return -EFAULT;
			ipu_clear_irq(irq);
		}
		break;
	case IPU_FREE_IRQ:
		{
			ipu_irq_info info;

			if (copy_from_user
					(&info, (ipu_irq_info *) arg,
					 sizeof(ipu_irq_info)))
				return -EFAULT;

			ipu_free_irq(info.irq, info.dev_id);
			irq_info[info.irq].irq_pending = 0;
		}
		break;
	case IPU_REQUEST_IRQ_STATUS:
		{
			uint32_t irq;
			int __user *argp = (void __user *)arg;
			if (get_user(irq, argp))
				return -EFAULT;
			ret = ipu_get_irq_status(irq);
		}
		break;
	case IPU_REGISTER_GENERIC_ISR:
		{
			ipu_event_info info;
			if (copy_from_user
					(&info, (ipu_event_info *) arg,
					 sizeof(ipu_event_info)))
				return -EFAULT;

			ret =
				ipu_request_irq(info.irq,
					mxc_ipu_generic_handler,
					0, "video_sink", info.dev);
			if (ret == 0)
				init_waitqueue_head(&(irq_info[info.irq].waitq));
		}
		break;
	case IPU_GET_EVENT:
		/* User will have to allocate event_type
		structure and pass the pointer in arg */
		{
			ipu_event_info info;
			int r = -1;

			if (copy_from_user
					(&info, (ipu_event_info *) arg,
					 sizeof(ipu_event_info)))
				return -EFAULT;

			r = get_events(&info);
			if (r == -1) {
				if ((file->f_flags & O_NONBLOCK) &&
					(irq_info[info.irq].irq_pending == 0))
					return -EAGAIN;
				wait_event_interruptible_timeout(irq_info[info.irq].waitq,
						(irq_info[info.irq].irq_pending != 0), 2 * HZ);
				r = get_events(&info);
			}
			ret = -1;
			if (r == 0) {
				if (!copy_to_user((ipu_event_info *) arg,
					&info, sizeof(ipu_event_info)))
					ret = 0;
			}
		}
		break;
	case IPU_ALOC_MEM:
		{
			ipu_mem_info info;
			if (copy_from_user
					(&info, (ipu_mem_info *) arg,
					 sizeof(ipu_mem_info)))
				return -EFAULT;

			info.vaddr = dma_alloc_coherent(0,
					PAGE_ALIGN(info.size),
					&info.paddr,
					GFP_DMA | GFP_KERNEL);
			if (info.vaddr == 0) {
				printk(KERN_ERR "dma alloc failed!\n");
				return -ENOBUFS;
			}
			if (copy_to_user((ipu_mem_info *) arg, &info,
					sizeof(ipu_mem_info)) > 0)
				return -EFAULT;
		}
		break;
	case IPU_FREE_MEM:
		{
			ipu_mem_info info;
			if (copy_from_user
					(&info, (ipu_mem_info *) arg,
					 sizeof(ipu_mem_info)))
				return -EFAULT;

			if (info.vaddr)
				dma_free_coherent(0, PAGE_ALIGN(info.size),
					info.vaddr, info.paddr);
			else
				return -EFAULT;
		}
		break;
	case IPU_IS_CHAN_BUSY:
		{
			ipu_channel_t chan;
			if (copy_from_user
					(&chan, (ipu_channel_t *)arg,
					 sizeof(ipu_channel_t)))
				return -EFAULT;

			if (ipu_is_channel_busy(chan))
				ret = 1;
			else
				ret = 0;
		}
		break;
	case IPU_CALC_STRIPES_SIZE:
		{
			ipu_stripe_parm stripe_parm;

			if (copy_from_user (&stripe_parm, (ipu_stripe_parm *)arg,
					 sizeof(ipu_stripe_parm)))
				return -EFAULT;
			ipu_calc_stripes_sizes(stripe_parm.input_width,
						stripe_parm.output_width,
						stripe_parm.maximal_stripe_width,
						stripe_parm.cirr,
						stripe_parm.equal_stripes,
						stripe_parm.input_pixelformat,
						stripe_parm.output_pixelformat,
						&stripe_parm.left,
						&stripe_parm.right);
			if (copy_to_user((ipu_stripe_parm *) arg, &stripe_parm,
					sizeof(ipu_stripe_parm)) > 0)
				return -EFAULT;
		}
		break;
	case IPU_UPDATE_BUF_OFFSET:
		{
			ipu_buf_offset_parm offset_parm;

			if (copy_from_user (&offset_parm, (ipu_buf_offset_parm *)arg,
					 sizeof(ipu_buf_offset_parm)))
				return -EFAULT;
			ret = ipu_update_channel_offset(offset_parm.channel,
							offset_parm.type,
							offset_parm.pixel_fmt,
							offset_parm.width,
							offset_parm.height,
							offset_parm.stride,
							offset_parm.u_offset,
							offset_parm.v_offset,
							offset_parm.vertical_offset,
							offset_parm.horizontal_offset);
		}
		break;
	case IPU_CSC_UPDATE:
		{
			int param[5][3];
			ipu_csc_update csc;
			if (copy_from_user(&csc, (void *) arg,
					   sizeof(ipu_csc_update)))
				return -EFAULT;
			if (copy_from_user(&param[0][0], (void *) csc.param,
					   sizeof(param)))
				return -EFAULT;
			ipu_set_csc_coefficients(csc.channel, param);
		}
		break;
	default:
		break;
	}
	return ret;
}

static int mxc_ipu_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_writethru(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot)) {
		printk(KERN_ERR
				"mmap failed!\n");
		return -ENOBUFS;
	}
	return 0;
}

static int mxc_ipu_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations mxc_ipu_fops = {
	.owner = THIS_MODULE,
	.open = mxc_ipu_open,
	.mmap = mxc_ipu_mmap,
	.release = mxc_ipu_release,
	.ioctl = mxc_ipu_ioctl,
};

int register_ipu_device()
{
	int ret = 0;
	struct device *temp;
	mxc_ipu_major = register_chrdev(0, "mxc_ipu", &mxc_ipu_fops);
	if (mxc_ipu_major < 0) {
		printk(KERN_ERR
			"Unable to register Mxc Ipu as a char device\n");
		return mxc_ipu_major;
	}

	mxc_ipu_class = class_create(THIS_MODULE, "mxc_ipu");
	if (IS_ERR(mxc_ipu_class)) {
		printk(KERN_ERR "Unable to create class for Mxc Ipu\n");
		ret = PTR_ERR(mxc_ipu_class);
		goto err1;
	}

	temp = device_create(mxc_ipu_class, NULL, MKDEV(mxc_ipu_major, 0),
			NULL, "mxc_ipu");

	if (IS_ERR(temp)) {
		printk(KERN_ERR "Unable to create class device for Mxc Ipu\n");
		ret = PTR_ERR(temp);
		goto err2;
	}
	spin_lock_init(&event_lock);

	return ret;

err2:
	class_destroy(mxc_ipu_class);
err1:
	unregister_chrdev(mxc_ipu_major, "mxc_ipu");
	return ret;

}
