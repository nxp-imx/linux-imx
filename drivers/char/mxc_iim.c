/*
 * Copyright (C) 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <mach/mxc_iim.h>

static struct mxc_iim_data *iim_data;

#ifdef MXC_IIM_DEBUG
static inline void dump_reg(void)
{
	struct iim_regs *iim_reg_base =
		(struct iim_regs *)iim_data->virt_base;

	dev_dbg(iim_data->dev, "stat: 0x%08x\n",
			__raw_readl(&iim_reg_base->stat));
	dev_dbg(iim_data->dev, "statm: 0x%08x\n",
			__raw_readl(&iim_reg_base->statm));
	dev_dbg(iim_data->dev, "err: 0x%08x\n",
			__raw_readl(&iim_reg_base->err));
	dev_dbg(iim_data->dev, "emask: 0x%08x\n",
			__raw_readl(&iim_reg_base->emask));
	dev_dbg(iim_data->dev, "fctl: 0x%08x\n",
			__raw_readl(&iim_reg_base->fctl));
	dev_dbg(iim_data->dev, "ua: 0x%08x\n",
			__raw_readl(&iim_reg_base->ua));
	dev_dbg(iim_data->dev, "la: 0x%08x\n",
			__raw_readl(&iim_reg_base->la));
	dev_dbg(iim_data->dev, "sdat: 0x%08x\n",
			__raw_readl(&iim_reg_base->sdat));
	dev_dbg(iim_data->dev, "prev: 0x%08x\n",
			__raw_readl(&iim_reg_base->prev));
	dev_dbg(iim_data->dev, "srev: 0x%08x\n",
			__raw_readl(&iim_reg_base->srev));
	dev_dbg(iim_data->dev, "preg_p: 0x%08x\n",
			__raw_readl(&iim_reg_base->preg_p));
	dev_dbg(iim_data->dev, "scs0: 0x%08x\n",
			__raw_readl(&iim_reg_base->scs0));
	dev_dbg(iim_data->dev, "scs1: 0x%08x\n",
			__raw_readl(&iim_reg_base->scs1));
	dev_dbg(iim_data->dev, "scs2: 0x%08x\n",
			__raw_readl(&iim_reg_base->scs2));
	dev_dbg(iim_data->dev, "scs3: 0x%08x\n",
			__raw_readl(&iim_reg_base->scs3));
}
#endif

static inline void mxc_iim_disable_irq(void)
{
	struct iim_regs *iim_reg_base = (struct iim_regs *)iim_data->virt_base;

	dev_dbg(iim_data->dev, "=> %s\n", __func__);

	__raw_writel(0x0, &(iim_reg_base->statm));
	__raw_writel(0x0, &(iim_reg_base->emask));

	dev_dbg(iim_data->dev, "<= %s\n", __func__);
}

static inline void fuse_op_start(void)
{
	struct iim_regs *iim_reg_base = (struct iim_regs *)iim_data->virt_base;

	dev_dbg(iim_data->dev, "=> %s\n", __func__);

#ifdef MXC_IIM_DEBUG
	dump_reg();
#endif

	/* Generate interrupt */
	__raw_writel(0x3, &(iim_reg_base->statm));
	__raw_writel(0xfe, &(iim_reg_base->emask));
	/* Clear the status bits and error bits */
	__raw_writel(0x3, &(iim_reg_base->stat));
	__raw_writel(0xfe, &(iim_reg_base->err));

	dev_dbg(iim_data->dev, "<= %s\n", __func__);
}

static u32 sense_fuse(s32 bank, s32 row, s32 bit)
{
	s32 addr, addr_l, addr_h;
	s32 err = 0;
	struct iim_regs *iim_reg_base = (struct iim_regs *)iim_data->virt_base;

	dev_dbg(iim_data->dev, "=> %s\n", __func__);

	init_completion(&(iim_data->completion));

	iim_data->action = POLL_FUSE_SNSD;

	fuse_op_start();

	addr = ((bank << 11) | (row << 3) | (bit & 0x7));
	/* Set IIM Program Upper Address */
	addr_h = (addr >> 8) & 0x000000FF;
	/* Set IIM Program Lower Address */
	addr_l = (addr & 0x000000FF);

	dev_dbg(iim_data->dev, "%s: addr_h=0x%x, addr_l=0x%x\n",
			__func__, addr_h, addr_l);
	__raw_writel(addr_h, &(iim_reg_base->ua));
	__raw_writel(addr_l, &(iim_reg_base->la));

	/* Start sensing */
#ifdef MXC_IIM_DEBUG
	dump_reg();
#endif
	__raw_writel(0x8, &(iim_reg_base->fctl));

	err = wait_for_completion_timeout(&(iim_data->completion),
					msecs_to_jiffies(1000));
	err = (!err) ? -ETIMEDOUT : 0;
	if (err)
		dev_dbg(iim_data->dev, "Sense timeout!");

	dev_dbg(iim_data->dev, "<= %s\n", __func__);

	return __raw_readl(&(iim_reg_base->sdat));
}

/* Blow fuses based on the bank, row and bit positions (all 0-based)
*/
static s32 fuse_blow_bit(s32 bank, s32 row, s32 bit)
{
	int addr, addr_l, addr_h, err;
	struct iim_regs *iim_reg_base = (struct iim_regs *)iim_data->virt_base;

	dev_dbg(iim_data->dev, "=> %s\n", __func__);

	init_completion(&iim_data->completion);

	iim_data->action = POLL_FUSE_PRGD;

	fuse_op_start();

	/* Disable IIM Program Protect */
	__raw_writel(0xaa, &(iim_reg_base->preg_p));

	addr = ((bank << 11) | (row << 3) | (bit & 0x7));
	/* Set IIM Program Upper Address */
	addr_h = (addr >> 8) & 0x000000FF;
	/* Set IIM Program Lower Address */
	addr_l = (addr & 0x000000FF);

	dev_dbg(iim_data->dev, "blowing addr_h=0x%x, addr_l=0x%x\n",
		addr_h, addr_l);

	__raw_writel(addr_h, &(iim_reg_base->ua));
	__raw_writel(addr_l, &(iim_reg_base->la));

	/* Start Programming */
#ifdef MXC_IIM_DEBUG
	dump_reg();
#endif
	__raw_writel(0x31, &(iim_reg_base->fctl));
	err = wait_for_completion_timeout(&(iim_data->completion),
					msecs_to_jiffies(1000));
	err = (!err) ? -ETIMEDOUT : 0;
	if (err)
		dev_dbg(iim_data->dev, "Fuse timeout!\n");

	/* Enable IIM Program Protect */
	__raw_writel(0x0, &(iim_reg_base->preg_p));

	dev_dbg(iim_data->dev, "<= %s\n", __func__);

	return err;
}

static void fuse_blow_row(s32 bank, s32 row, s32 value)
{
	u32 i;

	dev_dbg(iim_data->dev, "=> %s\n", __func__);

	/* Enable fuse blown */
	if (iim_data->enable_fuse)
		iim_data->enable_fuse();

	for (i = 0; i < 8; i++) {
		if (((value >> i) & 0x1) == 0)
			continue;
	if (fuse_blow_bit(bank, row, i) != 0) {
		dev_dbg(iim_data->dev, "fuse_blow_bit(bank: %d, row: %d, "
				"bit: %d failed\n",
				bank, row, i);
		}
	}

	if (iim_data->disable_fuse)
		iim_data->disable_fuse();

	dev_dbg(iim_data->dev, "<= %s\n", __func__);
}

static irqreturn_t mxc_iim_irq(int irq, void *dev_id)
{
	u32 status, error, rtn = 0;
	ulong flags;
	struct iim_regs *iim_reg_base = (struct iim_regs *)iim_data->virt_base;

	dev_dbg(iim_data->dev, "=> %s\n", __func__);

	spin_lock_irqsave(&iim_data->lock, flags);

	mxc_iim_disable_irq();
#ifdef MXC_IIM_DEBUG
	dump_reg();
#endif
	if (iim_data->action != POLL_FUSE_PRGD &&
		iim_data->action != POLL_FUSE_SNSD) {
		dev_dbg(iim_data->dev, "%s(%d) invalid operation\n",
			__func__, iim_data->action);
		rtn = 1;
		goto out;
	}

	/* Test for successful write */
	status = readl(&(iim_reg_base->stat));
	error = readl(&(iim_reg_base->err));

	if ((status & iim_data->action) != 0 && \
		(error & (iim_data->action >> IIM_ERR_SHIFT)) == 0) {
		if (error) {
			printk(KERN_NOTICE "Even though the operation"
				"seems successful...\n");
			printk(KERN_NOTICE "There are some error(s) "
				"at addr=0x%x: 0x%x\n",
				(u32)&(iim_reg_base->err), error);
		}
		rtn = 0;
		goto out;
	}
	printk(KERN_NOTICE "%s(%d) failed\n", __func__, iim_data->action);
	printk(KERN_NOTICE "status address=0x%x, value=0x%x\n",
		(u32)&(iim_reg_base->stat), status);
	printk(KERN_NOTICE "There are some error(s) at addr=0x%x: 0x%x\n",
		(u32)&(iim_reg_base->err), error);
#ifdef MXC_IIM_DEBUG
	dump_reg();
#endif

out:
	complete(&(iim_data->completion));
	spin_unlock_irqrestore(&iim_data->lock, flags);

	dev_dbg(iim_data->dev, "<= %s\n", __func__);

	return IRQ_RETVAL(rtn);
}

static loff_t mxc_iim_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;

	dev_dbg(iim_data->dev, "=> %s\n", __func__);
	dev_dbg(iim_data->dev, "off: %lld, whence: %d\n", off, whence);

	if (off < iim_data->bank_start ||
		off > iim_data->bank_end) {
		dev_dbg(iim_data->dev, "invalid seek offset: %lld\n", off);
		goto invald_arg_out;
	}

	switch (whence) {
	case 0: /* SEEK_SET */
		newpos = off;
		break;
	case 1: /* SEEK_CUR */
		newpos = filp->f_pos + off;
		break;
	case 2: /* SEEK_END */
		newpos = iim_data->bank_end + off;
		break;
	default: /* can't happen */
		return -EINVAL;

	}

	if (newpos < 0)
		return -EINVAL;
	filp->f_pos = newpos;

	dev_dbg(iim_data->dev, "newpos: %lld\n", newpos);

	dev_dbg(iim_data->dev, "<= %s\n", __func__);

	return newpos;
invald_arg_out:
	return -EINVAL;
}

static ssize_t mxc_iim_read(struct file *filp, char __user *buf,
			size_t count, loff_t *f_pos)
{
	s32 bank;
	s8 row, fuse_val;
	ssize_t retval = 0;

	dev_dbg(iim_data->dev, "=> %s\n", __func__);
	dev_dbg(iim_data->dev, "count: %d f_pos: %lld\n", count, *f_pos);

	if (1 != count)
		goto invald_arg_out;
	if (*f_pos & 0x3)
		goto invald_arg_out;
	if (*f_pos < iim_data->bank_start ||
		*f_pos > iim_data->bank_end) {
		dev_dbg(iim_data->dev, "bank_start: 0x%08x, bank_end: 0x%08x\n",
			iim_data->bank_start, iim_data->bank_end);
		goto out;
	}

	bank = (*f_pos - iim_data->bank_start) >> 10;
	row  = ((*f_pos - iim_data->bank_start) & 0x3ff) >> 2;

	dev_dbg(iim_data->dev, "Read fuse at bank:%d row:%d\n",
		bank, row);
	mutex_lock(&iim_data->mutex);
	fuse_val = (s8)sense_fuse(bank, row, 0);
	mutex_unlock(&iim_data->mutex);
	dev_dbg(iim_data->dev, "fuses at (bank:%d, row:%d) = 0x%x\n",
		bank, row, fuse_val);
	if (copy_to_user(buf, &fuse_val, count)) {
		retval = -EFAULT;
		goto out;
	}

out:
	retval = count;
	dev_dbg(iim_data->dev, "<= %s\n", __func__);
	return retval;
invald_arg_out:
	retval = -EINVAL;
	return retval;
}

static ssize_t mxc_iim_write(struct file *filp, const char __user *buf,
			size_t count, loff_t *f_pos)
{
	s32 bank;
	ulong fuse_val;
	s8 row;
	s8 *tmp_buf = NULL;
	ssize_t retval = 0;

	dev_dbg(iim_data->dev, "=> %s\n", __func__);
	dev_dbg(iim_data->dev, "count: %d f_pos: %lld\n", count, *f_pos);

	if (*f_pos & 0x3)
		goto invald_arg_out;
	if (*f_pos < iim_data->bank_start ||
		*f_pos > iim_data->bank_end) {
		dev_dbg(iim_data->dev, "bank_start: 0x%08x, bank_end: 0x%08x\n",
			iim_data->bank_start, iim_data->bank_end);
		goto invald_arg_out;
	}

	tmp_buf = kmalloc(count + 1, GFP_KERNEL);
	if (unlikely(!tmp_buf)) {
		retval = -ENOMEM;
		goto out;
	}
	memset(tmp_buf, 0, count + 1);
	if (copy_from_user(tmp_buf, buf, count)) {
		retval = -EFAULT;
		goto out;
	}

	bank = (*f_pos - iim_data->bank_start) >> 10;
	row  = ((*f_pos - iim_data->bank_start) & 0x3ff) >> 2;
	if (strict_strtoul(tmp_buf, 16, &fuse_val)) {
		dev_dbg(iim_data->dev, "Invalid value parameter: %s\n",
				tmp_buf);
		goto invald_arg_out;
	}
	if (unlikely(fuse_val > 0xff)) {
		dev_dbg(iim_data->dev, "Invalid value for fuse: %d\n",
				(unsigned int)fuse_val);
		goto invald_arg_out;
	}

	dev_dbg(iim_data->dev, "Blowing fuse at bank:%d row:%d value:%d\n",
			bank, row, (int)fuse_val);
	mutex_lock(&iim_data->mutex);
	fuse_blow_row(bank, row, fuse_val);
	fuse_val = sense_fuse(bank, row, 0);
	mutex_unlock(&iim_data->mutex);
	dev_dbg(iim_data->dev, "fuses at (bank:%d, row:%d) = 0x%x\n",
		bank, row, (unsigned int)fuse_val);

	retval = count;
out:
	if (tmp_buf)
		kfree(tmp_buf);
	dev_dbg(iim_data->dev, "<= %s\n", __func__);
	return retval;
invald_arg_out:
	if (tmp_buf)
		kfree(tmp_buf);
	retval = -EINVAL;
	return retval;
}

/*!
 * MXC IIM interface - memory map function
 * This function maps IIM registers from IIM base address.
 *
 * @param file	     struct file *
 * @param vma	     structure vm_area_struct *
 *
 * @return	     Return 0 on success or negative error code on error
 */
static int mxc_iim_mmap(struct file *file, struct vm_area_struct *vma)
{
	dev_dbg(iim_data->dev, "=> %s\n", __func__);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    iim_data->reg_base >> PAGE_SHIFT,
			    vma->vm_end - vma->vm_start,
			    vma->vm_page_prot))
		return -EAGAIN;

	dev_dbg(iim_data->dev, "<= %s\n", __func__);

	return 0;
}

/*!
 * MXC IIM interface - open function
 *
 * @param inode	     struct inode *
 * @param filp	     struct file *
 *
 * @return	     Return 0 on success or negative error code on error
 */
static int mxc_iim_open(struct inode *inode, struct file *filp)
{
	int ret;

	dev_dbg(iim_data->dev, "=> %s\n", __func__);

	dev_dbg(iim_data->dev, "iim_data addr: 0x%08x\n", (u32)iim_data);

	iim_data->clk = clk_get(NULL, "iim_clk");
	if (IS_ERR(iim_data->clk)) {
		dev_err(iim_data->dev, "No IIM clock defined\n");
		return -ENODEV;
	}
	clk_enable(iim_data->clk);

	iim_data->virt_base =
		(u32)ioremap(iim_data->reg_base, iim_data->reg_size);

	mxc_iim_disable_irq();

	dev_dbg(iim_data->dev, "<= %s\n", __func__);

	return 0;
}

/*!
 * MXC IIM interface - release function
 *
 * @param inode	     struct inode *
 * @param filp	     struct file *
 *
 * @return	     Return 0 on success or negative error code on error
 */
static int mxc_iim_release(struct inode *inode, struct file *filp)
{
	clk_disable(iim_data->clk);
	clk_put(iim_data->clk);
	iounmap((void *)iim_data->virt_base);
	return 0;
}

static const struct file_operations mxc_iim_fops = {
	.read = mxc_iim_read,
	.write = mxc_iim_write,
	.llseek = mxc_iim_llseek,
	.mmap = mxc_iim_mmap,
	.open = mxc_iim_open,
	.release = mxc_iim_release,
};

static struct miscdevice mxc_iim_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mxc_iim",
	.fops = &mxc_iim_fops,
};

/*!
 * This function is called by the driver framework to get iim base/end address
 * and register iim misc device.
 *
 * @param	dev	The device structure for IIM passed in by the driver
 *			framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static __devinit int mxc_iim_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	iim_data = pdev->dev.platform_data;
	iim_data->dev = &pdev->dev;
	iim_data->name = mxc_iim_miscdev.name;

	dev_dbg(iim_data->dev, "=> %s\n", __func__);

	dev_dbg(iim_data->dev, "iim_data addr: 0x%08x "
		"bank_start: 0x%04x bank_end: 0x%04x "
		"enable_fuse: 0x%08x disable_fuse: 0x%08x\n",
		(u32)iim_data,
		iim_data->bank_start, iim_data->bank_end,
		(u32)iim_data->enable_fuse,
		(u32)iim_data->disable_fuse);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(res)) {
		dev_err(iim_data->dev, "Unable to get IIM resource\n");
		return -ENODEV;
	}

	iim_data->irq = platform_get_irq(pdev, 0);
	dev_dbg(iim_data->dev, "irq number: %d\n", iim_data->irq);
	if (!iim_data->irq) {
		ret = -ENOMEM;
		return ret;
	}

	ret = request_irq(iim_data->irq, mxc_iim_irq, IRQF_DISABLED,
			iim_data->name, iim_data);
	if (ret)
		return ret;

	iim_data->reg_base = res->start;
	iim_data->reg_end = res->end;
	iim_data->reg_size =
		iim_data->reg_end - iim_data->reg_base + 1;

	mutex_init(&(iim_data->mutex));
	spin_lock_init(&(iim_data->lock));

	ret = misc_register(&mxc_iim_miscdev);
	if (ret)
		return ret;

	dev_dbg(iim_data->dev, "<= %s\n", __func__);

	return 0;
}

static int __devexit mxc_iim_remove(struct platform_device *pdev)
{
	free_irq(iim_data->irq, iim_data);
	misc_deregister(&mxc_iim_miscdev);
	return 0;
}

static struct platform_driver mxc_iim_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mxc_iim",
		   },
	.probe = mxc_iim_probe,
	.remove = mxc_iim_remove,
};

static int __init mxc_iim_dev_init(void)
{
	return platform_driver_register(&mxc_iim_driver);
}

static void __exit mxc_iim_dev_cleanup(void)
{
	platform_driver_unregister(&mxc_iim_driver);
}

module_init(mxc_iim_dev_init);
module_exit(mxc_iim_dev_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC IIM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(MISC_DYNAMIC_MINOR);
