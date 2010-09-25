/*
 * Freescale On-Chip OTP driver
 *
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fcntl.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include "fsl_otp.h"

static DEFINE_MUTEX(otp_mutex);
static struct kobject *otp_kobj;
static struct attribute **attrs;
static struct kobj_attribute *kattr;
static struct attribute_group attr_group;
static struct fsl_otp_data *otp_data;

static inline unsigned int get_reg_index(struct kobj_attribute *tmp)
{
	return tmp - kattr;
}

static int otp_wait_busy(u32 flags)
{
	int count;
	u32 c;

	for (count = 10000; count >= 0; count--) {
		c = __raw_readl(REGS_OCOTP_BASE + HW_OCOTP_CTRL);
		if (!(c & (BM_OCOTP_CTRL_BUSY | BM_OCOTP_CTRL_ERROR | flags)))
			break;
		cpu_relax();
	}
	if (count < 0)
		return -ETIMEDOUT;
	return 0;
}

static ssize_t otp_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	unsigned int index = get_reg_index(attr);
	u32 value = 0;

	/* sanity check */
	if (index >= otp_data->fuse_num)
		return 0;

	mutex_lock(&otp_mutex);

	if (otp_read_prepare()) {
		mutex_unlock(&otp_mutex);
		return 0;
	}
	value = __raw_readl(REGS_OCOTP_BASE + HW_OCOTP_CUSTn(index));
	otp_read_post();

	mutex_unlock(&otp_mutex);
	return sprintf(buf, "0x%x\n", value);
}

static int otp_write_bits(int addr, u32 data, u32 magic)
{
	u32 c; /* for control register */

	/* init the control register */
	c = __raw_readl(REGS_OCOTP_BASE + HW_OCOTP_CTRL);
	c &= ~BM_OCOTP_CTRL_ADDR;
	c |= BF(addr, OCOTP_CTRL_ADDR);
	c |= BF(magic, OCOTP_CTRL_WR_UNLOCK);
	__raw_writel(c, REGS_OCOTP_BASE + HW_OCOTP_CTRL);

	/* init the data register */
	__raw_writel(data, REGS_OCOTP_BASE + HW_OCOTP_DATA);
	otp_wait_busy(0);

	mdelay(2); /* Write Postamble */
	return 0;
}

static ssize_t otp_store(struct kobject *kobj, struct kobj_attribute *attr,
		       const char *buf, size_t count)
{
	unsigned int index = get_reg_index(attr);
	int value;

	/* sanity check */
	if (index >= otp_data->fuse_num)
		return 0;

	sscanf(buf, "0x%x", &value);

	mutex_lock(&otp_mutex);
	if (otp_write_prepare(otp_data)) {
		mutex_unlock(&otp_mutex);
		return 0;
	}
	otp_write_bits(index, value, 0x3e77);
	otp_write_post();
	mutex_unlock(&otp_mutex);
	return count;
}

static void free_otp_attr(void)
{
	kfree(attrs);
	attrs = NULL;

	kfree(kattr);
	kattr = NULL;
}

static int __init alloc_otp_attr(struct fsl_otp_data *pdata)
{
	int i;

	otp_data = pdata; /* get private data */

	/* The last one is NULL, which is used to detect the end */
	attrs = kzalloc((otp_data->fuse_num + 1) * sizeof(attrs[0]),
			GFP_KERNEL);
	kattr = kzalloc(otp_data->fuse_num * sizeof(struct kobj_attribute),
			GFP_KERNEL);

	if (!attrs || !kattr)
		goto error_out;

	for (i = 0; i < otp_data->fuse_num; i++) {
		kattr[i].attr.name = pdata->fuse_name[i];
		kattr[i].attr.mode = 0600;
		kattr[i].show  = otp_show;
		kattr[i].store = otp_store;

		attrs[i] = &kattr[i].attr;
	}
	memset(&attr_group, 0, sizeof(attr_group));
	attr_group.attrs = attrs;
	return 0;

error_out:
	free_otp_attr();
	return -ENOMEM;
}

static int __devinit fsl_otp_probe(struct platform_device *pdev)
{
	int retval;
	struct fsl_otp_data *pdata;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL)
		return -ENODEV;

	retval = alloc_otp_attr(pdata);
	if (retval)
		return retval;

	retval = map_ocotp_addr(pdev);
	if (retval)
		goto error;

	otp_kobj = kobject_create_and_add("fsl_otp", NULL);
	if (!otp_kobj) {
		retval = -ENOMEM;
		goto error;
	}

	retval = sysfs_create_group(otp_kobj, &attr_group);
	if (retval)
		goto error;

	mutex_init(&otp_mutex);
	return 0;
error:
	kobject_put(otp_kobj);
	otp_kobj = NULL;
	free_otp_attr();
	unmap_ocotp_addr();
	return retval;
}

static int otp_remove(struct platform_device *pdev)
{
	kobject_put(otp_kobj);
	otp_kobj = NULL;

	free_otp_attr();
	unmap_ocotp_addr();
	return 0;
}

static struct platform_driver ocotp_driver = {
	.probe		= fsl_otp_probe,
	.remove		= otp_remove,
	.driver		= {
		.name   = "ocotp",
		.owner	= THIS_MODULE,
	},
};

static int __init fsl_otp_init(void)
{
	return platform_driver_register(&ocotp_driver);
}

static void __exit fsl_otp_exit(void)
{
	platform_driver_unregister(&ocotp_driver);
}
module_init(fsl_otp_init);
module_exit(fsl_otp_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Huang Shijie <b32955@freescale.com>");
MODULE_DESCRIPTION("Common driver for OTP controller");
