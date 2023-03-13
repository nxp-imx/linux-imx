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
#include <linux/dma-buf-imx.h>

#define DMABUF_NUM_MINORS 256
static dev_t dmabuf_dev_imx;
static struct class *dmabuf_class;
static struct cdev cdev;
static DEFINE_MUTEX(dev_lock);

static void dma_buf_ioc_phy_release(struct device *dev)
{
	return;
}

long ion_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case DMABUF_GET_PHYS:
	{
		struct dma_buf *dmabuf;
		struct device dev;
		unsigned long phys = 0;
		struct dma_buf_attachment *attachment = NULL;
		struct sg_table *sgt = NULL;
		struct dmabuf_imx_phys_data data;
		int ret;

		// copy dmabuf_imx_phys_data from user space to kernel space
		if (copy_from_user(&data, (void __user *)arg,
			sizeof(struct dmabuf_imx_phys_data)))
			return -EFAULT;

		// get the dmafd from user space.
		// dma_buf is from dma_buf_get according the dma fd.
		dmabuf = dma_buf_get(data.dmafd);
		if (!dmabuf || IS_ERR(dmabuf)) {
			return -EFAULT;
		}
		memset(&dev, 0, sizeof(dev));
		mutex_lock(&dev_lock);
		device_initialize(&dev);
		dev.coherent_dma_mask = DMA_BIT_MASK(64);
		dev.dma_mask = &dev.coherent_dma_mask;
		dev.parent = NULL;
		dev.release = dma_buf_ioc_phy_release;
		dev_set_name(&dev, "dma_phy");
		ret = device_add(&dev);
		if (ret < 0) {
			put_device(&dev);
			return ret;
		}
		attachment = dma_buf_attach(dmabuf, &dev);
		if (!attachment || IS_ERR(attachment)) {
			device_del(&dev);
			put_device(&dev);
			dma_buf_put(dmabuf);
			mutex_unlock(&dev_lock);
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

		device_del(&dev);
		put_device(&dev);
		mutex_unlock(&dev_lock);
		// put the phys address to user space through dmabuf_imx_phys_data
		if (copy_to_user((void __user *)arg, &data,
			sizeof(struct dmabuf_imx_phys_data)))
			return -EFAULT;
		return 0;
	}
	case DMABUF_GET_HEAP_NAME:{
		struct dma_buf *dmabuf;
		struct dmabuf_imx_heap_name data;
		// copy dmabuf_imx_heap_name from user space to kernel space
		if (copy_from_user(&data, (void __user *)arg,
			sizeof(struct dmabuf_imx_heap_name)))
			return -EFAULT;

		// get the dmafd from user space.
		// dma_buf is from dma_buf_get according the dma fd.
		dmabuf = dma_buf_get(data.dmafd);
		if (!dmabuf || IS_ERR(dmabuf)) {
			return -EFAULT;
		}

		strscpy(data.name, dmabuf->exp_name, sizeof(data.name));
		dma_buf_put(dmabuf);

		// put the phys address to user space through dmabuf_imx_heap_name
		if (copy_to_user((void __user *)arg, &data,
			sizeof(struct dmabuf_imx_heap_name)))
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
	.compat_ioctl = ion_ioctl,
};

static int dmabuf_imx_init(void)
{
	int rc;
	struct device *dev;
	if ((rc = alloc_chrdev_region(&dmabuf_dev_imx, 0, DMABUF_NUM_MINORS, "dmabuf_imx"))) {
		pr_err("Unable to allocate major number: %i\n", rc);
		goto err1;
	}

	cdev_init(&cdev, &ion_fops);
	cdev.owner = THIS_MODULE;
	rc = cdev_add(&cdev, dmabuf_dev_imx, 1);

	dmabuf_class = class_create(THIS_MODULE, "dmabuf_imx");
	if (IS_ERR(dmabuf_class)) {
		pr_err("Unable to create imx ion class\n");
		rc = PTR_ERR(dmabuf_class);
		goto err2;
	}

	dev = device_create(dmabuf_class, NULL, dmabuf_dev_imx, NULL, "dmabuf_imx");
	if (IS_ERR(dev)) {
		pr_err("Unable to create imx ion device\n");
		rc = PTR_ERR(dev);
		goto err3;
	}

	return 0;

err3:
	class_destroy(dmabuf_class);
err2:
	unregister_chrdev_region(dmabuf_dev_imx, DMABUF_NUM_MINORS);
err1:
	return rc;
}

static void dmabuf_imx_exit(void)
{
	cdev_del(&cdev);
	device_destroy(dmabuf_class, dmabuf_dev_imx);
	class_destroy(dmabuf_class);
	unregister_chrdev_region(dmabuf_dev_imx, DMABUF_NUM_MINORS);
}

module_init(dmabuf_imx_init);
module_exit(dmabuf_imx_exit);

MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_DESCRIPTION("imx dmabuf driver");
MODULE_LICENSE("GPL");
