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

/*
 * Encoder device driver (kernel module)
 *
 * Copyright (C) 2005  Hantro Products Oy.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>		/*  __init,__exit directives */
#include <linux/mm.h>		/* remap_page_range / remap_pfn_range */
#include <linux/fs.h>		/* for struct file_operations */
#include <linux/errno.h>	/* standard error codes */
#include <linux/platform_device.h>	/* for device registeration for PM */
#include <linux/delay.h>	/* for msleep_interruptible */
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>	/* for dma_alloc_consistent */
#include <linux/clk.h>
#include <asm/uaccess.h>	/* for ioctl __get_user, __put_user */
#include <mach/hardware.h>
#include "mxc_hmp4e.h"		/* MPEG4 encoder specific */

/* here's all the must remember stuff */
typedef struct {
	ulong buffaddr;
	u32 buffsize;
	ulong iobaseaddr;
	u32 iosize;
	volatile u32 *hwregs;
	u32 irq;
	u16 hwid_offset;
	u16 intr_offset;
	u16 busy_offset;
	u16 type;		/* Encoder type, CIF = 0, VGA = 1 */
	u16 clk_gate;
	u16 busy_val;
	struct fasync_struct *async_queue;
#ifdef CONFIG_PM
	s32 suspend_state;
	wait_queue_head_t power_queue;
#endif
} hmp4e_t;

/* and this is our MAJOR; use 0 for dynamic allocation (recommended)*/
static s32 hmp4e_major;

static u32 hmp4e_phys;
static struct class *hmp4e_class;
static hmp4e_t hmp4e_data;

/*! MPEG4 enc clock handle. */
static struct clk *hmp4e_clk;

/*
 * avoid "enable_irq(x) unbalanced from ..."
 * error messages from the kernel, since {ena,dis}able_irq()
 * calls are stacked in kernel.
 */
static bool irq_enable;

ulong base_port = MPEG4_ENC_BASE_ADDR;
u32 irq = MXC_INT_MPEG4_ENCODER;

module_param(base_port, long, 000);
module_param(irq, int, 000);

/*!
 * These variables store the register values when HMP4E is in suspend mode.
 */
#ifdef CONFIG_PM
u32 io_regs[64];
#endif

static s32 hmp4e_map_buffer(struct file *filp, struct vm_area_struct *vma);
static s32 hmp4e_map_hwregs(struct file *filp, struct vm_area_struct *vma);
static void hmp4e_reset(hmp4e_t *dev);
irqreturn_t hmp4e_isr(s32 irq, void *dev_id);

/*!
 * This funtion is called to write h/w register.
 *
 * @param   val		value to be written into the register
 * @param   offset	register offset
 *
 */
static inline void hmp4e_write(u32 val, u32 offset)
{
	hmp4e_t *dev = &hmp4e_data;
	__raw_writel(val, (dev->hwregs + offset));
}

/*!
 * This funtion is called to read h/w register.
 *
 * @param   offset	register offset
 *
 * @return  This function returns the value read from the register.
 *
 */
static inline u32 hmp4e_read(u32 offset)
{
	hmp4e_t *dev = &hmp4e_data;
	u32 val;

	val = __raw_readl(dev->hwregs + offset);

	return val;
}

/*!
 * The device's mmap method. The VFS has kindly prepared the process's
 * vm_area_struct for us, so we examine this to see what was requested.
 *
 * @param   filp	pointer to struct file
 * @param   vma		pointer to struct vma_area_struct
 *
 * @return  This function returns 0 if successful or -ve value on error.
 *
 */
static s32 hmp4e_mmap(struct file *filp, struct vm_area_struct *vma)
{
	s32 result;
	ulong offset = vma->vm_pgoff << PAGE_SHIFT;

	pr_debug("hmp4e_mmap: size = %lu off = 0x%08lx\n",
		 (unsigned long)(vma->vm_end - vma->vm_start), offset);

	if (offset == 0) {
		result = hmp4e_map_buffer(filp, vma);
	} else if (offset == hmp4e_data.iobaseaddr) {
		result = hmp4e_map_hwregs(filp, vma);
	} else {
		pr_debug("hmp4e: mmap invalid value\n");
		result = -EINVAL;
	}

	return result;
}

/*!
 * This funtion is called to handle ioctls.
 *
 * @param   inode	pointer to struct inode
 * @param   filp	pointer to struct file
 * @param   cmd		ioctl command
 * @param   arg		user data
 *
 * @return  This function returns 0 if successful or -ve value on error.
 *
 */
static s32 hmp4e_ioctl(struct inode *inode, struct file *filp,
		       u32 cmd, ulong arg)
{
	s32 err = 0, retval = 0;
	ulong offset = 0;
	hmp4e_t *dev = &hmp4e_data;
	write_t bwrite;

#ifdef CONFIG_PM
	wait_event_interruptible(hmp4e_data.power_queue,
				 hmp4e_data.suspend_state == 0);
#endif

	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != HMP4E_IOC_MAGIC) {
		pr_debug("hmp4e: ioctl invalid magic\n");
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > HMP4E_IOC_MAXNR) {
		pr_debug("hmp4e: ioctl exceeds max ioctl\n");
		return -ENOTTY;
	}

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	}

	if (err) {
		pr_debug("hmp4e: ioctl invalid direction\n");
		return -EFAULT;
	}

	switch (cmd) {
	case HMP4E_IOCHARDRESET:
		break;

	case HMP4E_IOCGBUFBUSADDRESS:
		retval = __put_user((ulong) hmp4e_phys, (u32 *) arg);
		break;

	case HMP4E_IOCGBUFSIZE:
		retval = __put_user(hmp4e_data.buffsize, (u32 *) arg);
		break;

	case HMP4E_IOCSREGWRITE:
		if (dev->type != 1) {	/* This ioctl only for VGA */
			pr_debug("hmp4e: HMP4E_IOCSREGWRITE invalid\n");
			retval = -EINVAL;
			break;
		}

		retval = __copy_from_user(&bwrite, (u32 *) arg,
					  sizeof(write_t));

		if (bwrite.offset <= hmp4e_data.iosize - 4) {
			hmp4e_write(bwrite.data, (bwrite.offset / 4));
		} else {
			pr_debug("hmp4e: HMP4E_IOCSREGWRITE failed\n");
			retval = -EFAULT;
		}
		break;

	case HMP4E_IOCXREGREAD:
		if (dev->type != 1) {	/* This ioctl only for VGA */
			pr_debug("hmp4e: HMP4E_IOCSREGWRITE invalid\n");
			retval = -EINVAL;
			break;
		}

		retval = __get_user(offset, (ulong *) arg);
		if (offset <= hmp4e_data.iosize - 4) {
			__put_user(hmp4e_read((offset / 4)), (ulong *) arg);
		} else {
			pr_debug("hmp4e: HMP4E_IOCXREGREAD failed\n");
			retval = -EFAULT;
		}
		break;

	case HMP4E_IOCGHWOFFSET:
		__put_user(hmp4e_data.iobaseaddr, (ulong *) arg);
		break;

	case HMP4E_IOCGHWIOSIZE:
		__put_user(hmp4e_data.iosize, (u32 *) arg);
		break;

	case HMP4E_IOC_CLI:
		if (irq_enable == true) {
			disable_irq(hmp4e_data.irq);
			irq_enable = false;
		}
		break;

	case HMP4E_IOC_STI:
		if (irq_enable == false) {
			enable_irq(hmp4e_data.irq);
			irq_enable = true;
		}
		break;

	default:
		pr_debug("unknown case %x\n", cmd);
	}

	return retval;
}

/*!
 * This funtion is called when the device is opened.
 *
 * @param   inode	pointer to struct inode
 * @param   filp	pointer to struct file
 *
 * @return  This function returns 0 if successful or -ve value on error.
 *
 */
static s32 hmp4e_open(struct inode *inode, struct file *filp)
{
	hmp4e_t *dev = &hmp4e_data;

	filp->private_data = (void *)dev;

	if (request_irq(dev->irq, hmp4e_isr, 0, "mxc_hmp4e", dev) != 0) {
		pr_debug("hmp4e: request irq failed\n");
		return -EBUSY;
	}

	if (irq_enable == false) {
		irq_enable = true;
	}
	clk_enable(hmp4e_clk);
	return 0;
}

static s32 hmp4e_fasync(s32 fd, struct file *filp, s32 mode)
{
	hmp4e_t *dev = (hmp4e_t *) filp->private_data;
	return fasync_helper(fd, filp, mode, &dev->async_queue);
}

/*!
 * This funtion is called when the device is closed.
 *
 * @param   inode	pointer to struct inode
 * @param   filp	pointer to struct file
 *
 * @return  This function returns 0.
 *
 */
static s32 hmp4e_release(struct inode *inode, struct file *filp)
{
	hmp4e_t *dev = (hmp4e_t *) filp->private_data;

	/* this is necessary if user process exited asynchronously */
	if (irq_enable == true) {
		disable_irq(dev->irq);
		irq_enable = false;
	}

	/* reset hardware */
	hmp4e_reset(&hmp4e_data);

	/* free the encoder IRQ */
	free_irq(dev->irq, (void *)dev);

	/* remove this filp from the asynchronusly notified filp's */
	hmp4e_fasync(-1, filp, 0);
	clk_disable(hmp4e_clk);
	return 0;
}

/* VFS methods */
static struct file_operations hmp4e_fops = {
	.owner = THIS_MODULE,
	.open = hmp4e_open,
	.release = hmp4e_release,
	.ioctl = hmp4e_ioctl,
	.mmap = hmp4e_mmap,
	.fasync = hmp4e_fasync,
};

/*!
 * This funtion allocates physical contigous memory.
 *
 * @param   size 	size of memory to be allocated
 *
 * @return  This function returns 0 if successful or -ve value on error.
 *
 */
static s32 hmp4e_alloc(u32 size)
{
	hmp4e_data.buffsize = PAGE_ALIGN(size);
	hmp4e_data.buffaddr =
	    (ulong) dma_alloc_coherent(NULL, hmp4e_data.buffsize,
				       (dma_addr_t *) &hmp4e_phys,
				       GFP_DMA | GFP_KERNEL);

	if (hmp4e_data.buffaddr == 0) {
		printk(KERN_ERR "hmp4e: couldn't allocate data buffer\n");
		return -ENOMEM;
	}

	memset((s8 *) hmp4e_data.buffaddr, 0, hmp4e_data.buffsize);
	return 0;
}

/*!
 * This funtion frees the DMAed memory.
 */
static void hmp4e_free(void)
{
	if (hmp4e_data.buffaddr != 0) {
		dma_free_coherent(NULL, hmp4e_data.buffsize,
				  (void *)hmp4e_data.buffaddr, hmp4e_phys);
		hmp4e_data.buffaddr = 0;
	}
}

/*!
 * This funtion maps the shared buffer in memory.
 *
 * @param   filp 	pointer to struct file
 * @param   vma		pointer to struct vm_area_struct
 *
 * @return  This function returns 0 if successful or -ve value on error.
 *
 */
static s32 hmp4e_map_buffer(struct file *filp, struct vm_area_struct *vma)
{
	ulong phys;
	ulong start = (u32) vma->vm_start;
	ulong size = (u32) (vma->vm_end - vma->vm_start);

	/* if userspace tries to mmap beyond end of our buffer, fail */
	if (size > hmp4e_data.buffsize) {
		pr_debug("hmp4e: hmp4e_map_buffer, invalid size\n");
		return -EINVAL;
	}

	vma->vm_flags |= VM_RESERVED | VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	phys = hmp4e_phys;

	if (remap_pfn_range(vma, start, phys >> PAGE_SHIFT, size,
			    vma->vm_page_prot)) {
		pr_debug("hmp4e: failed mmapping shared buffer\n");
		return -EAGAIN;
	}

	return 0;
}

/*!
 * This funtion maps the h/w register space in memory.
 *
 * @param   filp 	pointer to struct file
 * @param   vma		pointer to struct vm_area_struct
 *
 * @return  This function returns 0 if successful or -ve value on error.
 *
 */
static s32 hmp4e_map_hwregs(struct file *filp, struct vm_area_struct *vma)
{
	ulong phys;
	ulong start = (unsigned long)vma->vm_start;
	ulong size = (unsigned long)(vma->vm_end - vma->vm_start);

	/* if userspace tries to mmap beyond end of our buffer, fail */
	if (size > PAGE_SIZE) {
		pr_debug("hmp4e: hmp4e_map_hwregs, invalid size\n");
		return -EINVAL;
	}

	vma->vm_flags |= VM_RESERVED | VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Remember this won't work for vmalloc()d memory ! */
	phys = hmp4e_data.iobaseaddr;

	if (remap_pfn_range(vma, start, phys >> PAGE_SHIFT, hmp4e_data.iosize,
			    vma->vm_page_prot)) {
		pr_debug("hmp4e: failed mmapping HW registers\n");
		return -EAGAIN;
	}

	return 0;
}

/*!
 * This function is the interrupt service routine.
 *
 * @param   irq		the irq number
 * @param   dev_id	driver data when ISR was regiatered
 *
 * @return  The return value is IRQ_HANDLED.
 *
 */
irqreturn_t hmp4e_isr(s32 irq, void *dev_id)
{
	hmp4e_t *dev = (hmp4e_t *) dev_id;
	u32 offset = dev->intr_offset;

	u32 irq_status = hmp4e_read(offset);

	/* clear enc IRQ */
	hmp4e_write(irq_status & (~0x01), offset);

	if (dev->async_queue)
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);

	return IRQ_HANDLED;
}

/*!
 * This function is called to reset the encoder.
 *
 * @param   dev	 pointer to struct hmp4e_data
 *
 */
static void hmp4e_reset(hmp4e_t *dev)
{
	s32 i;

	/* enable HCLK for register reset */
	hmp4e_write(dev->clk_gate, 0);

	/* Reset registers, except ECR0 (0x00) and ID (read-only) */
	for (i = 1; i < (dev->iosize / 4); i += 1) {
		if (i == dev->hwid_offset)	/* ID is read only */
			continue;

		/* Only for CIF, not used */
		if ((dev->type == 0) && (i == 14))
			continue;

		hmp4e_write(0, i);
	}

	/* disable HCLK */
	hmp4e_write(0, 0);
	return;
}

/*!
 * This function is called during the driver binding process. This function
 * does the hardware initialization.
 *
 * @param   dev   the device structure used to store device specific
 *                information that is used by the suspend, resume and remove
 *                functions
 *
 * @return  The function returns 0 if successful.
 */
static s32 hmp4e_probe(struct platform_device *pdev)
{
	s32 result;
	u32 hwid;
	struct device *temp_class;

	hmp4e_data.iobaseaddr = base_port;
	hmp4e_data.irq = irq;
	hmp4e_data.buffaddr = 0;

	/* map hw i/o registers into kernel space */
	hmp4e_data.hwregs = (volatile void *)IO_ADDRESS(hmp4e_data.iobaseaddr);

	hmp4e_clk = clk_get(&pdev->dev, "mpeg4_clk");
	if (IS_ERR(hmp4e_clk)) {
		printk(KERN_INFO "hmp4e: Unable to get clock\n");
		return -EIO;
	}

	clk_enable(hmp4e_clk);

	/* check hw id for encoder signature */
	hwid = hmp4e_read(7);
	if ((hwid & 0xffff) == 0x1882) {	/* CIF first */
		hmp4e_data.type = 0;
		hmp4e_data.iosize = (16 * 4);
		hmp4e_data.hwid_offset = 7;
		hmp4e_data.intr_offset = 5;
		hmp4e_data.clk_gate = (1 << 1);
		hmp4e_data.buffsize = 512000;
		hmp4e_data.busy_offset = 0;
		hmp4e_data.busy_val = 1;
	} else {
		hwid = hmp4e_read((0x88 / 4));
		if ((hwid & 0xffff0000) == 0x52510000) {	/* VGA */
			hmp4e_data.type = 1;
			hmp4e_data.iosize = (35 * 4);
			hmp4e_data.hwid_offset = (0x88 / 4);
			hmp4e_data.intr_offset = (0x10 / 4);
			hmp4e_data.clk_gate = (1 << 12);
			hmp4e_data.buffsize = 1048576;
			hmp4e_data.busy_offset = (0x10 / 4);
			hmp4e_data.busy_val = (1 << 1);
		} else {
			printk(KERN_INFO "hmp4e: HW ID not found\n");
			goto error1;
		}
	}

	/* Reset hardware */
	hmp4e_reset(&hmp4e_data);

	/* allocate memory shared with ewl */
	result = hmp4e_alloc(hmp4e_data.buffsize);
	if (result < 0)
		goto error1;

	result = register_chrdev(hmp4e_major, "hmp4e", &hmp4e_fops);
	if (result <= 0) {
		pr_debug("hmp4e: unable to get major %d\n", hmp4e_major);
		goto error2;
	}

	hmp4e_major = result;

	hmp4e_class = class_create(THIS_MODULE, "hmp4e");
	if (IS_ERR(hmp4e_class)) {
		pr_debug("Error creating hmp4e class.\n");
		goto error3;
	}

	temp_class = device_create(hmp4e_class, NULL, MKDEV(hmp4e_major, 0), NULL,
				   "hmp4e");
	if (IS_ERR(temp_class)) {
		pr_debug("Error creating hmp4e class device.\n");
		goto error4;
	}

	platform_set_drvdata(pdev, &hmp4e_data);

#ifdef CONFIG_PM
	hmp4e_data.async_queue = NULL;
	hmp4e_data.suspend_state = 0;
	init_waitqueue_head(&hmp4e_data.power_queue);
#endif

	printk(KERN_INFO "hmp4e: %s encoder initialized\n",
	       hmp4e_data.type ? "VGA" : "CIF");
	clk_disable(hmp4e_clk);
	return 0;

      error4:
	class_destroy(hmp4e_class);
      error3:
	unregister_chrdev(hmp4e_major, "hmp4e");
      error2:
	hmp4e_free();
      error1:
	clk_disable(hmp4e_clk);
	clk_put(hmp4e_clk);
	printk(KERN_INFO "hmp4e: module not inserted\n");
	return -EIO;
}

/*!
 * Dissociates the driver.
 *
 * @param   dev   the device structure
 *
 * @return  The function always returns 0.
 */
static s32 hmp4e_remove(struct platform_device *pdev)
{
	device_destroy(hmp4e_class, MKDEV(hmp4e_major, 0));
	class_destroy(hmp4e_class);
	unregister_chrdev(hmp4e_major, "hmp4e");
	hmp4e_free();
	clk_disable(hmp4e_clk);
	clk_put(hmp4e_clk);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
/*!
 * This is the suspend of power management for the Hantro MPEG4 module
 *
 * @param        dev            the device
 * @param        state          the state
 *
 * @return       This function always returns 0.
 */
static s32 hmp4e_suspend(struct platform_device *pdev, pm_message_t state)
{
	s32 i;
	hmp4e_t *pdata = &hmp4e_data;

	/*
	 * how many times msleep_interruptible will be called before
	 * giving up
	 */
	s32 timeout = 10;

	pr_debug("hmp4e: Suspend\n");
	hmp4e_data.suspend_state = 1;

	/* check if encoder is currently running */
	while ((hmp4e_read(pdata->busy_offset) & (pdata->busy_val)) &&
	       --timeout) {
		pr_debug("hmp4e: encoder is running, going to sleep\n");
		msleep_interruptible((unsigned int)30);
	}

	if (!timeout) {
		pr_debug("hmp4e: timeout suspending, resetting encoder\n");
		hmp4e_write(hmp4e_read(pdata->busy_offset) &
			    (~pdata->busy_val), pdata->busy_offset);
	}

	/* first read register 0 */
	io_regs[0] = hmp4e_read(0);

	/* then override HCLK to make sure other registers can be read */
	hmp4e_write(pdata->clk_gate, 0);

	/* read other registers */
	for (i = 1; i < (pdata->iosize / 4); i += 1) {

		/* Only for CIF, not used */
		if ((pdata->type == 0) && (i == 14))
			continue;

		io_regs[i] = hmp4e_read(i);
	}

	/* restore value of register 0 */
	hmp4e_write(io_regs[0], 0);

	/* stop HCLK */
	hmp4e_write(0, 0);
	clk_disable(hmp4e_clk);
	return 0;
};

/*!
 * This is the resume of power management for the Hantro MPEG4 module
 * It suports RESTORE state.
 *
 * @param        pdev            the platform device
 *
 * @return       This function always returns 0
 */
static s32 hmp4e_resume(struct platform_device *pdev)
{
	s32 i;
	u32 status;
	hmp4e_t *pdata = &hmp4e_data;

	pr_debug("hmp4e: Resume\n");
	clk_enable(hmp4e_clk);

	/* override HCLK to make sure registers can be written */
	hmp4e_write(pdata->clk_gate, 0x00);

	for (i = 1; i < (pdata->iosize / 4); i += 1) {
		if (i == pdata->hwid_offset)	/* Read only */
			continue;

		/* Only for CIF, not used */
		if ((pdata->type == 0) && (i == 14))
			continue;

		hmp4e_write(io_regs[i], i);
	}

	/* write register 0 last */
	hmp4e_write(io_regs[0], 0x00);

	/* Clear the suspend flag */
	hmp4e_data.suspend_state = 0;

	/* Unblock the wait queue */
	wake_up_interruptible(&hmp4e_data.power_queue);

	/* Continue operations */
	status = hmp4e_read(pdata->intr_offset);
	if (status & 0x1) {
		hmp4e_write(status & (~0x01), pdata->intr_offset);
		if (hmp4e_data.async_queue)
			kill_fasync(&hmp4e_data.async_queue, SIGIO, POLL_IN);
	}

	return 0;
};

#endif

static struct platform_driver hmp4e_driver = {
	.driver = {
		   .name = "mxc_hmp4e",
		   },
	.probe = hmp4e_probe,
	.remove = hmp4e_remove,
#ifdef CONFIG_PM
	.suspend = hmp4e_suspend,
	.resume = hmp4e_resume,
#endif
};

static s32 __init hmp4e_init(void)
{
	printk(KERN_INFO "hmp4e: init\n");
	platform_driver_register(&hmp4e_driver);
	return 0;
}

static void __exit hmp4e_cleanup(void)
{
	platform_driver_unregister(&hmp4e_driver);
	printk(KERN_INFO "hmp4e: module removed\n");
}

module_init(hmp4e_init);
module_exit(hmp4e_cleanup);

/* module description */
MODULE_AUTHOR("Hantro Products Oy");
MODULE_DESCRIPTION("Device driver for Hantro's hardware based MPEG4 encoder");
MODULE_SUPPORTED_DEVICE("5251/4251 MPEG4 Encoder");
MODULE_LICENSE("GPL");
