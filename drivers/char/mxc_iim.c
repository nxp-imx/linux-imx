/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>

static unsigned long iim_reg_base, iim_reg_end, iim_reg_size;
static struct clk *iim_clk;
static struct device *iim_dev;

/*!
 * MXC IIM interface - memory map function
 * This function maps 4KB IIM registers from IIM base address.
 *
 * @param file	     struct file *
 * @param vma	     structure vm_area_struct *
 *
 * @return	     Return 0 on success or negative error code on error
 */
static int mxc_iim_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    iim_reg_base >> PAGE_SHIFT,
			    iim_reg_size,
			    vma->vm_page_prot))
		return -EAGAIN;

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
	iim_clk = clk_get(NULL, "iim_clk");
	if (IS_ERR(iim_clk)) {
		dev_err(iim_dev, "No IIM clock defined\n");
		return -ENODEV;
	}
	clk_enable(iim_clk);

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
	clk_disable(iim_clk);
	clk_put(iim_clk);
	return 0;
}

static const struct file_operations mxc_iim_fops = {
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
static int mxc_iim_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	iim_dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(res)) {
		dev_err(iim_dev, "Unable to get IIM resource\n");
		return -ENODEV;
	}

	iim_reg_base = res->start;
	iim_reg_end = res->end;
	iim_reg_size = iim_reg_end - iim_reg_base + 1;

	ret = misc_register(&mxc_iim_miscdev);
	if (ret)
		return ret;

	return 0;
}

static int mxc_iim_remove(struct platform_device *pdev)
{
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
