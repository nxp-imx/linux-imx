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
 * @brief This file contains the IPU driver device interface and fops functions.
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

#include "ipu_prv.h"
#include "ipu_regs.h"
#include "ipu_param_mem.h"

/* Strucutures and variables for exporting MXC IPU as device*/

#define MAX_Q_SIZE 10

static int mxc_ipu_major;
static struct class *mxc_ipu_class;

DEFINE_SPINLOCK(queue_lock);
static DECLARE_MUTEX(user_mutex);

static wait_queue_head_t waitq;
static int pending_events;
int read_ptr;
int write_ptr;

ipu_event_info events[MAX_Q_SIZE];

int register_ipu_device(void);

/* Static functions */

int get_events(ipu_event_info *p)
{
	unsigned long flags;
	int ret = 0, i, cnt, found = 0;
	spin_lock_irqsave(&queue_lock, flags);
	if (pending_events != 0) {
		if (write_ptr > read_ptr)
			cnt = write_ptr - read_ptr;
		else
			cnt = MAX_Q_SIZE - read_ptr + write_ptr;
		for (i = 0; i < cnt; i++) {
			if (p->irq == events[read_ptr].irq) {
				*p = events[read_ptr];
				events[read_ptr].irq = 0;
				read_ptr++;
				if (read_ptr >= MAX_Q_SIZE)
					read_ptr = 0;
				found = 1;
				break;
			}

			if (events[read_ptr].irq) {
				events[write_ptr] = events[read_ptr];
				events[read_ptr].irq = 0;
				write_ptr++;
				if (write_ptr >= MAX_Q_SIZE)
					write_ptr = 0;
			} else
				pending_events--;

			read_ptr++;
			if (read_ptr >= MAX_Q_SIZE)
				read_ptr = 0;
		}
		if (found)
			pending_events--;
		else
			ret = -1;
	} else {
		ret = -1;
	}

	spin_unlock_irqrestore(&queue_lock, flags);
	return ret;
}

static irqreturn_t mxc_ipu_generic_handler(int irq, void *dev_id)
{
	ipu_event_info e;

	e.irq = irq;
	e.dev = dev_id;
	events[write_ptr] = e;
	write_ptr++;
	if (write_ptr >= MAX_Q_SIZE)
		write_ptr = 0;
	pending_events++;
	/* Wakeup any blocking user context */
	wake_up_interruptible(&waitq);
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
			     sizeof(ipu_channel_parm))) {
				return -EFAULT;
			}
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
			     sizeof(ipu_channel_buf_parm))) {
				return -EFAULT;
			}
			ret =
			    ipu_init_channel_buffer(parm.channel, parm.type,
						    parm.pixel_fmt,
						    parm.width, parm.height,
						    parm.stride,
						    parm.rot_mode,
						    parm.phyaddr_0,
						    parm.phyaddr_1,
						    0,
						    parm.u_offset,
						    parm.v_offset);

		}
		break;

	case IPU_UPDATE_CHANNEL_BUFFER:
		{
			ipu_channel_buf_parm parm;
			if (copy_from_user
			    (&parm, (ipu_channel_buf_parm *) arg,
			     sizeof(ipu_channel_buf_parm))) {
				return -EFAULT;
			}
			if ((parm.phyaddr_0 != (dma_addr_t) NULL)
			    && (parm.phyaddr_1 == (dma_addr_t) NULL)) {
				ret =
				    ipu_update_channel_buffer(parm.channel,
							      parm.type,
							      parm.bufNum,
							      parm.phyaddr_0);
			} else if ((parm.phyaddr_0 == (dma_addr_t) NULL)
				   && (parm.phyaddr_1 != (dma_addr_t) NULL)) {
				ret =
				    ipu_update_channel_buffer(parm.channel,
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
			     sizeof(ipu_channel_buf_parm))) {
				return -EFAULT;
			}
			ret =
			    ipu_select_buffer(parm.channel, parm.type,
					      parm.bufNum);

		}
		break;
	case IPU_LINK_CHANNELS:
		{
			ipu_channel_link link;
			if (copy_from_user
			    (&link, (ipu_channel_link *) arg,
			     sizeof(ipu_channel_link))) {
				return -EFAULT;
			}
			ret = ipu_link_channels(link.src_ch, link.dest_ch);

		}
		break;
	case IPU_UNLINK_CHANNELS:
		{
			ipu_channel_link link;
			if (copy_from_user
			    (&link, (ipu_channel_link *) arg,
			     sizeof(ipu_channel_link))) {
				return -EFAULT;
			}
			ret = ipu_unlink_channels(link.src_ch, link.dest_ch);

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
			     sizeof(ipu_channel_info))) {
				return -EFAULT;
			}
			ret = ipu_disable_channel(info.channel, info.stop);
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
			     sizeof(ipu_irq_info))) {
				return -EFAULT;
			}
			ipu_free_irq(info.irq, info.dev_id);
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
	case IPU_SDC_INIT_PANEL:
		{
			ipu_sdc_panel_info sinfo;
			if (copy_from_user
			    (&sinfo, (ipu_sdc_panel_info *) arg,
			     sizeof(ipu_sdc_panel_info))) {
				return -EFAULT;
			}
			ret =
			    ipu_sdc_init_panel(sinfo.panel, sinfo.pixel_clk,
					       sinfo.width, sinfo.height,
					       sinfo.pixel_fmt,
					       sinfo.hStartWidth,
					       sinfo.hSyncWidth,
					       sinfo.hEndWidth,
					       sinfo.vStartWidth,
					       sinfo.vSyncWidth,
					       sinfo.vEndWidth, sinfo.signal);
		}
		break;
	case IPU_SDC_SET_WIN_POS:
		{
			ipu_sdc_window_pos pos;
			if (copy_from_user
			    (&pos, (ipu_sdc_window_pos *) arg,
			     sizeof(ipu_sdc_window_pos))) {
				return -EFAULT;
			}
			ret =
			    ipu_disp_set_window_pos(pos.channel, pos.x_pos,
						   pos.y_pos);

		}
		break;
	case IPU_SDC_SET_GLOBAL_ALPHA:
		{
			ipu_sdc_global_alpha g;
			if (copy_from_user
			    (&g, (ipu_sdc_global_alpha *) arg,
			     sizeof(ipu_sdc_global_alpha))) {
				return -EFAULT;
			}
			ret = ipu_sdc_set_global_alpha(g.enable, g.alpha);
		}
		break;
	case IPU_SDC_SET_COLOR_KEY:
		{
			ipu_sdc_color_key c;
			if (copy_from_user
			    (&c, (ipu_sdc_color_key *) arg,
			     sizeof(ipu_sdc_color_key))) {
				return -EFAULT;
			}
			ret =
			    ipu_sdc_set_color_key(c.channel, c.enable,
						  c.colorKey);
		}
		break;
	case IPU_SDC_SET_BRIGHTNESS:
		{
			uint8_t b;
			int __user *argp = (void __user *)arg;
			if (get_user(b, argp))
				return -EFAULT;
			ret = ipu_sdc_set_brightness(b);

		}
		break;
	case IPU_REGISTER_GENERIC_ISR:
		{
			ipu_event_info info;
			if (copy_from_user
			    (&info, (ipu_event_info *) arg,
			     sizeof(ipu_event_info))) {
				return -EFAULT;
			}
			ret =
			    ipu_request_irq(info.irq, mxc_ipu_generic_handler,
					    0, "video_sink", info.dev);
		}
		break;
	case IPU_GET_EVENT:
		/* User will have to allocate event_type structure and pass the pointer in arg */
		{
			ipu_event_info info;
			int r = -1;

			if (copy_from_user
					(&info, (ipu_event_info *) arg,
					 sizeof(ipu_event_info)))
				return -EFAULT;

			r = get_events(&info);
			if (r == -1) {
				wait_event_interruptible_timeout(waitq,
						(pending_events != 0), 2 * HZ);
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
	case IPU_ADC_WRITE_TEMPLATE:
		{
			ipu_adc_template temp;
			if (copy_from_user
			    (&temp, (ipu_adc_template *) arg, sizeof(temp))) {
				return -EFAULT;
			}
			ret =
			    ipu_adc_write_template(temp.disp, temp.pCmd,
						   temp.write);
		}
		break;
	case IPU_ADC_UPDATE:
		{
			ipu_adc_update update;
			if (copy_from_user
			    (&update, (ipu_adc_update *) arg, sizeof(update))) {
				return -EFAULT;
			}
			ret =
			    ipu_adc_set_update_mode(update.channel, update.mode,
						    update.refresh_rate,
						    update.addr, update.size);
		}
		break;
	case IPU_ADC_SNOOP:
		{
			ipu_adc_snoop snoop;
			if (copy_from_user
			    (&snoop, (ipu_adc_snoop *) arg, sizeof(snoop))) {
				return -EFAULT;
			}
			ret =
			    ipu_adc_get_snooping_status(snoop.statl,
							snoop.stath);
		}
		break;
	case IPU_ADC_CMD:
		{
			ipu_adc_cmd cmd;
			if (copy_from_user
			    (&cmd, (ipu_adc_cmd *) arg, sizeof(cmd))) {
				return -EFAULT;
			}
			ret =
			    ipu_adc_write_cmd(cmd.disp, cmd.type, cmd.cmd,
					      cmd.params, cmd.numParams);
		}
		break;
	case IPU_ADC_INIT_PANEL:
		{
			ipu_adc_panel panel;
			if (copy_from_user
			    (&panel, (ipu_adc_panel *) arg, sizeof(panel))) {
				return -EFAULT;
			}
			ret =
			    ipu_adc_init_panel(panel.disp, panel.width,
					       panel.height, panel.pixel_fmt,
					       panel.stride, panel.signal,
					       panel.addr, panel.vsync_width,
					       panel.mode);
		}
		break;
	case IPU_ADC_IFC_TIMING:
		{
			ipu_adc_ifc_timing t;
			if (copy_from_user
			    (&t, (ipu_adc_ifc_timing *) arg, sizeof(t))) {
				return -EFAULT;
			}
			ret =
			    ipu_adc_init_ifc_timing(t.disp, t.read,
						    t.cycle_time, t.up_time,
						    t.down_time,
						    t.read_latch_time,
						    t.pixel_clk);
		}
		break;
	case IPU_CSI_INIT_INTERFACE:
		{
			ipu_csi_interface c;
			if (copy_from_user
			    (&c, (ipu_csi_interface *) arg, sizeof(c)))
				return -EFAULT;
			ret =
			    ipu_csi_init_interface(c.width, c.height,
						   c.pixel_fmt, c.signal);
		}
		break;
	case IPU_CSI_ENABLE_MCLK:
		{
			ipu_csi_mclk m;
			if (copy_from_user(&m, (ipu_csi_mclk *) arg, sizeof(m)))
				return -EFAULT;
			ret = ipu_csi_enable_mclk(m.src, m.flag, m.wait);
		}
		break;
	case IPU_CSI_READ_MCLK_FLAG:
		{
			ret = ipu_csi_read_mclk_flag();
		}
		break;
	case IPU_CSI_FLASH_STROBE:
		{
			bool strobe;
			int __user *argp = (void __user *)arg;
			if (get_user(strobe, argp))
				return -EFAULT;
			ipu_csi_flash_strobe(strobe);
		}
		break;
	case IPU_CSI_GET_WIN_SIZE:
		{
			ipu_csi_window_size w;
			int dummy = 0;
			ipu_csi_get_window_size(&w.width, &w.height, dummy);
			if (copy_to_user
			    ((ipu_csi_window_size *) arg, &w, sizeof(w)))
				return -EFAULT;
		}
		break;
	case IPU_CSI_SET_WIN_SIZE:
		{
			ipu_csi_window_size w;
			int dummy = 0;
			if (copy_from_user
			    (&w, (ipu_csi_window_size *) arg, sizeof(w)))
				return -EFAULT;
			ipu_csi_set_window_size(w.width, w.height, dummy);
		}
		break;
	case IPU_CSI_SET_WINDOW:
		{
			ipu_csi_window p;
			int dummy = 0;
			if (copy_from_user
			    (&p, (ipu_csi_window *) arg, sizeof(p)))
				return -EFAULT;
			ipu_csi_set_window_pos(p.left, p.top, dummy);
		}
		break;
	case IPU_PF_SET_PAUSE_ROW:
		{
			uint32_t p;
			int __user *argp = (void __user *)arg;
			if (get_user(p, argp))
				return -EFAULT;
			ret = ipu_pf_set_pause_row(p);
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

			if (info.vaddr != 0)
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
	.ioctl = mxc_ipu_ioctl
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

	temp = device_create(mxc_ipu_class, NULL, MKDEV(mxc_ipu_major, 0), NULL,
			     "mxc_ipu");

	if (IS_ERR(temp)) {
		printk(KERN_ERR "Unable to create class device for Mxc Ipu\n");
		ret = PTR_ERR(temp);
		goto err2;
	}
	spin_lock_init(&queue_lock);
	init_waitqueue_head(&waitq);
	return ret;

      err2:
	class_destroy(mxc_ipu_class);
      err1:
	unregister_chrdev(mxc_ipu_major, "mxc_ipu");
	return ret;

}
