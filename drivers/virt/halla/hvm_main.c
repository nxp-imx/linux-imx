// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Co., Ltd.
 *            http://www.samsung.com/
 */

#include <linux/device.h>
#include <linux/file.h>
#include <linux/kdev_t.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#include "exynos-hvc.h"
#include "hvm_drv.h"

static long hvm_dev_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long args)
{
	void __user *argp = (void __user *)args;

	switch (cmd) {
	case HVM_CREATE_VM:
		return hvm_dev_ioctl_create_vm(args);
	case HVM_CHECK_EXTENSION: {
		u64 cap;

		if (copy_from_user(&cap, argp, sizeof(__u64)))
			return -EFAULT;

		return hvm_check_extension(NULL, cap, argp);
	}
	default:
		break;
	}

	return -ENOTTY;
}

static const struct file_operations hvm_dev_ops = {
	.unlocked_ioctl = hvm_dev_ioctl,
};

static struct miscdevice hvm_dev = {
	.name = HALLA_KERNEL_DEV_NAME,
	.fops = &hvm_dev_ops,
};

static struct reserved_mem *hvm_get_vm_pgt_rmem(struct device *dev)
{
	struct device_node *np;
	struct reserved_mem *rmem;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "Failed to acquire memory region phandle\n");
		return NULL;
	}

	rmem = of_reserved_mem_lookup(np);
	if (!rmem) {
		dev_err(dev, "Failed to acquire memory region\n");
		return NULL;
	}

	dev_info(dev, "Reserved memory for VM pgtable: addr=%#llx, size=%#llx\n",
		 rmem->base, rmem->size);

	return rmem;
}

static int drv_init(struct device *dev)
{
	struct reserved_mem *rmem;
	unsigned long ret;

	rmem = hvm_get_vm_pgt_rmem(dev);
	if (!rmem)
		return -ENOENT;

	ret = exynos_hvc(HVC_FID_HVM_PROBE, rmem->base, rmem->size,
			 hvm_get_guest_vtimer_irq(), 0);
	if (ret) {
		dev_err(dev, "HVC for HVM probe failure (%#lx)\n", ret);
		return -ENXIO;
	}

	return 0;
}

static int hvm_drv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	ret = misc_register(&hvm_dev);
	if (ret)
		return ret;

	ret = hvm_drv_irqfd_init();
	if (ret)
		goto err_irqfd_init;

	ret = hvm_vtimer_init(dev);
	if (ret)
		goto err_vtimer_init;

	ret = hvm_drv_debug_init();
	if (ret)
		goto err_debug_init;

	ret = drv_init(dev);
	if (ret)
		goto err_drv_init;

	return 0;

err_drv_init:
	hvm_drv_debug_exit();
err_debug_init:
	hvm_vtimer_exit();
err_vtimer_init:
	hvm_drv_irqfd_exit();
err_irqfd_init:
	misc_deregister(&hvm_dev);

	return ret;
}

static void hvm_drv_remove(struct platform_device *pdev)
{
	hvm_drv_irqfd_exit();
	hvm_destroy_all_vms();
	misc_deregister(&hvm_dev);
}

static const struct of_device_id hvm_of_match[] = {
	{ .compatible = "samsung,exynos-hvm" },
	{ },
};

static struct platform_driver hvm_driver = {
	.probe = hvm_drv_probe,
	.remove = hvm_drv_remove,
	.driver = {
		.name = "exynos-hvm",
		.owner = THIS_MODULE,
		.of_match_table = hvm_of_match,
	},
};

module_platform_driver(hvm_driver);

MODULE_DEVICE_TABLE(of, hvm_of_match);
MODULE_DESCRIPTION("Halla VM management");
MODULE_AUTHOR("Samsung LSI");
MODULE_LICENSE("GPL");
