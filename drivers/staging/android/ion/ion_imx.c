/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2021 NXP
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include "ion_imx.h"

#define ION_NUM_MINORS 256
static dev_t ion_dev_imx;
static struct class *ion_class;
static struct cdev cdev;

long ion_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case ION_GET_PHYS:
	{
		struct dma_buf *dmabuf;
		struct device dev;
		unsigned long phys = 0;
		struct dma_buf_attachment *attachment = NULL;
		struct sg_table *sgt = NULL;
		struct ion_imx_phys_data data;

		// copy ion_imx_phys_data from user space to kernel space
		if (copy_from_user(&data, (void __user *)arg,
			sizeof(struct ion_imx_phys_data)))
			return -EFAULT;

		// get the dmafd from user space.
		// dma_buf is from dma_buf_get according the dma fd.
		dmabuf = dma_buf_get(data.dmafd);
		if (!dmabuf || IS_ERR(dmabuf)) {
			return -EFAULT;
		}
		memset(&dev, 0, sizeof(dev));
		device_initialize(&dev);
		dev.coherent_dma_mask = DMA_BIT_MASK(64);
		dev.dma_mask = &dev.coherent_dma_mask;
		attachment = dma_buf_attach(dmabuf, &dev);
		if (!attachment || IS_ERR(attachment)) {
			return -EFAULT;
		}

		sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
		if (sgt && !IS_ERR(sgt)) {
			phys = sg_dma_address(sgt->sgl);
			dma_buf_unmap_attachment(attachment, sgt,
				DMA_BIDIRECTIONAL);
		}

		dma_buf_detach(dmabuf, attachment);

		data.phys = phys;
		dma_buf_put(dmabuf);

		// put the phys address to user space through ion_imx_phys_data
		if (copy_to_user((void __user *)arg, &data,
			sizeof(struct ion_imx_phys_data)))
			return -EFAULT;
		return 0;
	}
	default:
		return -ENOTTY;
	}
}

static const struct file_operations ion_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ion_ioctl,
};

static int ion_imx_init(void)
{
	int rc;
	struct device *dev;
	if ((rc = alloc_chrdev_region(&ion_dev_imx, 0, ION_NUM_MINORS, "ion_imx"))) {
		pr_err("Unable to allocate major number: %i\n", rc);
		goto err1;
	}

	cdev_init(&cdev, &ion_fops);
	cdev.owner = THIS_MODULE;
	rc = cdev_add(&cdev, ion_dev_imx, 1);

	ion_class = class_create(THIS_MODULE, "ion_imx");
	if (IS_ERR(ion_class)) {
		pr_err("Unable to create imx ion class\n");
		rc = PTR_ERR(ion_class);
		goto err2;
	}

	dev = device_create(ion_class, NULL, ion_dev_imx, NULL, "ion_imx");
	if (IS_ERR(dev)) {
		pr_err("Unable to create imx ion device\n");
		rc = PTR_ERR(dev);
		goto err3;
	}

	return 0;

err3:
	class_destroy(ion_class);
err2:
	unregister_chrdev_region(ion_dev_imx, ION_NUM_MINORS);
err1:
	return rc;
}

static ion_imx_exit(void)
{
	cdev_del(&cdev);
	device_destroy(ion_class, ion_dev_imx);
	class_destroy(ion_class);
	unregister_chrdev_region(ion_dev_imx, ION_NUM_MINORS);
}

module_init(ion_imx_init);
module_exit(ion_imx_exit);

MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_DESCRIPTION("imx ion driver");
MODULE_LICENSE("GPL");
