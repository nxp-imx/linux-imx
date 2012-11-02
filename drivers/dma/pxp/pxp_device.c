/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/pxp_dma.h>

#include <linux/atomic.h>

#include <mach/dma.h>

static atomic_t open_count = ATOMIC_INIT(0);

static DEFINE_SPINLOCK(pxp_mem_lock);
static DEFINE_SPINLOCK(pxp_chan_lock);
static LIST_HEAD(head);
static LIST_HEAD(list);
static struct pxp_irq_info irq_info[NR_PXP_VIRT_CHANNEL];

struct pxp_chan_handle {
	int chan_id;
	int hist_status;
};

/* To track the allocated memory buffer */
struct memalloc_record {
	struct list_head list;
	struct pxp_mem_desc mem;
};

struct pxp_chan_info {
	int chan_id;
	struct dma_chan *dma_chan;
	struct list_head list;
};

static int pxp_alloc_dma_buffer(struct pxp_mem_desc *mem)
{
	mem->cpu_addr = (unsigned long)
	    dma_alloc_coherent(NULL, PAGE_ALIGN(mem->size),
			       (dma_addr_t *) (&mem->phys_addr),
			       GFP_DMA | GFP_KERNEL);
	pr_debug("[ALLOC] mem alloc phys_addr = 0x%x\n", mem->phys_addr);
	if ((void *)(mem->cpu_addr) == NULL) {
		printk(KERN_ERR "Physical memory allocation error!\n");
		return -1;
	}
	return 0;
}

static void pxp_free_dma_buffer(struct pxp_mem_desc *mem)
{
	if (mem->cpu_addr != 0) {
		dma_free_coherent(0, PAGE_ALIGN(mem->size),
				  (void *)mem->cpu_addr, mem->phys_addr);
	}
}

static int pxp_free_buffers(void)
{
	struct memalloc_record *rec, *n;
	struct pxp_mem_desc mem;

	list_for_each_entry_safe(rec, n, &head, list) {
		mem = rec->mem;
		if (mem.cpu_addr != 0) {
			pxp_free_dma_buffer(&mem);
			pr_debug("[FREE] freed paddr=0x%08X\n", mem.phys_addr);
			/* delete from list */
			list_del(&rec->list);
			kfree(rec);
		}
	}

	return 0;
}

/* Callback function triggered after PxP receives an EOF interrupt */
static void pxp_dma_done(void *arg)
{
	struct pxp_tx_desc *tx_desc = to_tx_desc(arg);
	struct dma_chan *chan = tx_desc->txd.chan;
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);
	int chan_id = pxp_chan->dma_chan.chan_id;

	pr_debug("DMA Done ISR, chan_id %d\n", chan_id);

	irq_info[chan_id].irq_pending++;
	irq_info[chan_id].hist_status = tx_desc->hist_status;

	wake_up_interruptible(&(irq_info[chan_id].waitq));
}

static int pxp_ioc_config_chan(unsigned long arg)
{
	struct scatterlist sg[3];
	struct pxp_tx_desc *desc;
	struct dma_async_tx_descriptor *txd;
	struct pxp_chan_info *info;
	struct pxp_config_data pxp_conf;
	dma_cookie_t cookie;
	int chan_id;
	int i, length, ret;

	ret = copy_from_user(&pxp_conf,
			     (struct pxp_config_data *)arg,
			     sizeof(struct pxp_config_data));
	if (ret)
		return -EFAULT;

	chan_id = pxp_conf.chan_id;
	if (chan_id < 0 || chan_id >= NR_PXP_VIRT_CHANNEL)
		return -ENODEV;

	init_waitqueue_head(&(irq_info[chan_id].waitq));

	/* find the channel */
	spin_lock(&pxp_chan_lock);
	list_for_each_entry(info, &list, list) {
		if (info->dma_chan->chan_id == chan_id)
			break;
	}
	spin_unlock(&pxp_chan_lock);

	sg_init_table(sg, 3);

	txd =
	    info->dma_chan->device->device_prep_slave_sg(info->dma_chan,
							 sg, 3,
							 DMA_TO_DEVICE,
							 DMA_PREP_INTERRUPT);
	if (!txd) {
		pr_err("Error preparing a DMA transaction descriptor.\n");
		return -EIO;
	}

	txd->callback_param = txd;
	txd->callback = pxp_dma_done;

	desc = to_tx_desc(txd);

	length = desc->len;
	for (i = 0; i < length; i++) {
		if (i == 0) {	/* S0 */
			memcpy(&desc->proc_data,
			       &pxp_conf.proc_data,
			       sizeof(struct pxp_proc_data));
			memcpy(&desc->layer_param.s0_param,
			       &pxp_conf.s0_param,
			       sizeof(struct pxp_layer_param));
		} else if (i == 1) {	/* Output */
			memcpy(&desc->layer_param.out_param,
			       &pxp_conf.out_param,
			       sizeof(struct pxp_layer_param));
		} else {
			/* OverLay */
			memcpy(&desc->layer_param.ol_param,
			       &pxp_conf.ol_param,
			       sizeof(struct pxp_layer_param));
		}

		desc = desc->next;
	}

	cookie = txd->tx_submit(txd);
	if (cookie < 0) {
		pr_err("Error tx_submit\n");
		return -EIO;
	}

	return 0;
}

static int pxp_device_open(struct inode *inode, struct file *filp)
{
	atomic_inc(&open_count);

	return 0;
}

static int pxp_device_release(struct inode *inode, struct file *filp)
{
	if (atomic_dec_and_test(&open_count))
		pxp_free_buffers();

	return 0;
}

static int pxp_device_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct memalloc_record *rec, *n;
	int request_size, found;

	request_size = vma->vm_end - vma->vm_start;
	found = 0;

	pr_debug("start=0x%x, pgoff=0x%x, size=0x%x\n",
		 (unsigned int)(vma->vm_start), (unsigned int)(vma->vm_pgoff),
		 request_size);

	spin_lock(&pxp_mem_lock);
	list_for_each_entry_safe(rec, n, &head, list) {
		if (rec->mem.phys_addr == (vma->vm_pgoff << PAGE_SHIFT) &&
			(rec->mem.size <= request_size)) {
			found = 1;
			break;
		}
	}
	spin_unlock(&pxp_mem_lock);

	if (found == 0)
		return -ENOMEM;

	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			       request_size, vma->vm_page_prot) ? -EAGAIN : 0;
}

static bool chan_filter(struct dma_chan *chan, void *arg)
{
	if (imx_dma_is_pxp(chan))
		return true;
	else
		return false;
}

static long pxp_device_ioctl(struct file *filp,
			    unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case PXP_IOC_GET_CHAN:
		{
			struct pxp_chan_info *info;
			dma_cap_mask_t mask;

			pr_debug("drv: PXP_IOC_GET_CHAN Line %d\n", __LINE__);
			info = kzalloc(sizeof(*info), GFP_KERNEL);
			if (!info) {
				pr_err("%d: alloc err\n", __LINE__);
				return -ENOMEM;
			}

			dma_cap_zero(mask);
			dma_cap_set(DMA_SLAVE, mask);
			dma_cap_set(DMA_PRIVATE, mask);
			info->dma_chan = dma_request_channel(mask, chan_filter, NULL);
			if (!info->dma_chan) {
				pr_err("Unsccessfully received channel!\n");
				kfree(info);
				return -EBUSY;
			}
			pr_debug("Successfully received channel."
				 "chan_id %d\n", info->dma_chan->chan_id);

			spin_lock(&pxp_chan_lock);
			list_add_tail(&info->list, &list);
			spin_unlock(&pxp_chan_lock);

			if (put_user
			    (info->dma_chan->chan_id, (u32 __user *) arg))
				return -EFAULT;

			break;
		}
	case PXP_IOC_PUT_CHAN:
		{
			int chan_id;
			struct pxp_chan_info *info;

			if (get_user(chan_id, (u32 __user *) arg))
				return -EFAULT;

			if (chan_id < 0 || chan_id >= NR_PXP_VIRT_CHANNEL)
				return -ENODEV;

			spin_lock(&pxp_chan_lock);
			list_for_each_entry(info, &list, list) {
				if (info->dma_chan->chan_id == chan_id)
					break;
			}
			spin_unlock(&pxp_chan_lock);

			pr_debug("%d release chan_id %d\n", __LINE__,
				 info->dma_chan->chan_id);
			/* REVISIT */
			dma_release_channel(info->dma_chan);
			spin_lock(&pxp_chan_lock);
			list_del_init(&info->list);
			spin_unlock(&pxp_chan_lock);
			kfree(info);

			break;
		}
	case PXP_IOC_CONFIG_CHAN:
		{

			int ret;

			ret = pxp_ioc_config_chan(arg);
			if (ret)
				return ret;

			break;
		}
	case PXP_IOC_START_CHAN:
		{
			struct pxp_chan_info *info;
			int chan_id;

			if (get_user(chan_id, (u32 __user *) arg))
				return -EFAULT;

			/* find the channel */
			spin_lock(&pxp_chan_lock);
			list_for_each_entry(info, &list, list) {
				if (info->dma_chan->chan_id == chan_id)
					break;
			}
			spin_unlock(&pxp_chan_lock);

			dma_async_issue_pending(info->dma_chan);

			break;
		}
	case PXP_IOC_GET_PHYMEM:
		{
			struct memalloc_record *rec;

			rec = kzalloc(sizeof(*rec), GFP_KERNEL);
			if (!rec)
				return -ENOMEM;

			ret = copy_from_user(&(rec->mem),
					     (struct pxp_mem_desc *)arg,
					     sizeof(struct pxp_mem_desc));
			if (ret) {
				kfree(rec);
				return -EFAULT;
			}

			pr_debug("[ALLOC] mem alloc size = 0x%x\n",
				 rec->mem.size);

			ret = pxp_alloc_dma_buffer(&(rec->mem));
			if (ret == -1) {
				kfree(rec);
				printk(KERN_ERR
				       "Physical memory allocation error!\n");
				break;
			}
			ret = copy_to_user((void __user *)arg, &(rec->mem),
					   sizeof(struct pxp_mem_desc));
			if (ret) {
				kfree(rec);
				ret = -EFAULT;
				break;
			}

			spin_lock(&pxp_mem_lock);
			list_add(&rec->list, &head);
			spin_unlock(&pxp_mem_lock);

			break;
		}
	case PXP_IOC_PUT_PHYMEM:
		{
			struct memalloc_record *rec, *n;
			struct pxp_mem_desc pxp_mem;

			ret = copy_from_user(&pxp_mem,
					     (struct pxp_mem_desc *)arg,
					     sizeof(struct pxp_mem_desc));
			if (ret)
				return -EACCES;

			pr_debug("[FREE] mem freed cpu_addr = 0x%x\n",
				 pxp_mem.cpu_addr);
			if ((void *)pxp_mem.cpu_addr != NULL)
				pxp_free_dma_buffer(&pxp_mem);

			spin_lock(&pxp_mem_lock);
			list_for_each_entry_safe(rec, n, &head, list) {
				if (rec->mem.cpu_addr == pxp_mem.cpu_addr) {
					/* delete from list */
					list_del(&rec->list);
					kfree(rec);
					break;
				}
			}
			spin_unlock(&pxp_mem_lock);

			break;
		}
	case PXP_IOC_WAIT4CMPLT:
		{
			struct pxp_chan_handle chan_handle;
			int ret, chan_id;

			ret = copy_from_user(&chan_handle,
					     (struct pxp_chan_handle *)arg,
					     sizeof(struct pxp_chan_handle));
			if (ret)
				return -EFAULT;

			chan_id = chan_handle.chan_id;
			if (chan_id < 0 || chan_id >= NR_PXP_VIRT_CHANNEL)
				return -ENODEV;

			if (!wait_event_interruptible_timeout
			    (irq_info[chan_id].waitq,
			     (irq_info[chan_id].irq_pending != 0), 2 * HZ)) {
				pr_warning("pxp blocking: timeout.\n");
				return -ETIME;
			} else if (signal_pending(current)) {
				printk(KERN_WARNING
				       "pxp interrupt received.\n");
				return -ERESTARTSYS;
			} else
				irq_info[chan_id].irq_pending--;

			chan_handle.hist_status = irq_info[chan_id].hist_status;
			ret = copy_to_user((struct pxp_chan_handle *)arg,
					   &chan_handle,
					   sizeof(struct pxp_chan_handle));
			if (ret)
				return -EFAULT;
			break;
		}
	default:
		break;
	}

	return 0;
}

static const struct file_operations pxp_device_fops = {
	.open = pxp_device_open,
	.release = pxp_device_release,
	.unlocked_ioctl = pxp_device_ioctl,
	.mmap = pxp_device_mmap,
};

static struct miscdevice pxp_device_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pxp_device",
	.fops = &pxp_device_fops,
};

static int __devinit pxp_device_probe(struct platform_device *pdev)
{
	int ret;

	ret = misc_register(&pxp_device_miscdev);
	if (ret)
		return ret;

	pr_debug("PxP_Device Probe Successfully\n");
	return 0;
}

static int __devexit pxp_device_remove(struct platform_device *pdev)
{
	misc_deregister(&pxp_device_miscdev);

	return 0;
}

static struct platform_driver pxp_client_driver = {
	.probe = pxp_device_probe,
	.remove = __exit_p(pxp_device_remove),
	.driver = {
		   .name = "imx-pxp-client",
		   .owner = THIS_MODULE,
		   },
};

static int __init pxp_device_init(void)
{
	return platform_driver_register(&pxp_client_driver);
}

static void __exit pxp_device_exit(void)
{
	platform_driver_unregister(&pxp_client_driver);
}

module_init(pxp_device_init);
module_exit(pxp_device_exit);

MODULE_DESCRIPTION("i.MX PxP client driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
