/*
 * Copyright (C) 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/miscdevice.h>

static unsigned long iim_reg_base0, iim_reg_end0, iim_reg_size0;
static unsigned long iim_reg_base1, iim_reg_end1, iim_reg_size1;
static struct device *iim_dev;

/*!
 * MXS Virtual IIM interface - memory map function
 * This function maps one page size VIIM registers from VIIM base address0
 * if the size of the required virtual memory space is less than or equal to
 * one page size, otherwise this function will also map one page size VIIM
 * registers from VIIM base address1.
 *
 * @param file	     struct file *
 * @param vma	     structure vm_area_struct *
 *
 * @return	     Return 0 on success or negative error code on error
 */
static int mxs_viim_mmap(struct file *file, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    iim_reg_base0 >> PAGE_SHIFT,
			    iim_reg_size0,
			    vma->vm_page_prot))
		return -EAGAIN;

	if (size > iim_reg_size0) {
		if (remap_pfn_range(vma,
				    vma->vm_start + iim_reg_size0,
				    iim_reg_base1 >> PAGE_SHIFT,
				    iim_reg_size1,
				    vma->vm_page_prot))
			return -EAGAIN;
	}

	return 0;
}

/*!
 * MXS Virtual IIM interface - open function
 *
 * @param inode	     struct inode *
 * @param filp	     struct file *
 *
 * @return	     Return 0 on success or negative error code on error
 */
static int mxs_viim_open(struct inode *inode, struct file *filp)
{
	return 0;
}

/*!
 * MXS Virtual IIM interface - release function
 *
 * @param inode	     struct inode *
 * @param filp	     struct file *
 *
 * @return	     Return 0 on success or negative error code on error
 */
static int mxs_viim_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations mxs_viim_fops = {
	.mmap = mxs_viim_mmap,
	.open = mxs_viim_open,
	.release = mxs_viim_release,
};

static struct miscdevice mxs_viim_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mxs_viim",
	.fops = &mxs_viim_fops,
};

/*!
 * This function is called by the driver framework to get virtual iim base/end
 * address and register iim misc device.
 *
 * @param	dev	The device structure for Virtual IIM passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int mxs_viim_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	iim_dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(res)) {
		dev_err(iim_dev, "Unable to get Virtual IIM resource 0\n");
		return -ENODEV;
	}

	iim_reg_base0 = res->start;
	iim_reg_end0 = res->end;
	iim_reg_size0 = iim_reg_end0 - iim_reg_base0 + 1;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (IS_ERR(res)) {
		dev_err(iim_dev, "Unable to get Virtual IIM resource 1\n");
		return -ENODEV;
	}

	iim_reg_base1 = res->start;
	iim_reg_end1 = res->end;
	iim_reg_size1 = iim_reg_end1 - iim_reg_base1 + 1;

	ret = misc_register(&mxs_viim_miscdev);
	if (ret)
		return ret;

	return 0;
}

static int mxs_viim_remove(struct platform_device *pdev)
{
	misc_deregister(&mxs_viim_miscdev);
	return 0;
}

static struct platform_driver mxs_viim_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mxs_viim",
		   },
	.probe = mxs_viim_probe,
	.remove = mxs_viim_remove,
};

static int __init mxs_viim_dev_init(void)
{
	return platform_driver_register(&mxs_viim_driver);
}

static void __exit mxs_viim_dev_cleanup(void)
{
	platform_driver_unregister(&mxs_viim_driver);
}

module_init(mxs_viim_dev_init);
module_exit(mxs_viim_dev_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IMX Virtual IIM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(MISC_DYNAMIC_MINOR);
