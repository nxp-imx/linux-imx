// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 NXP
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>

#define MU_BUF_OFFSET	0x8000

/*
 * This structure holds the private data associated with a UIO device,
 * typically used in a platform that exposes hardware to userspace via the UIO framework
 */
struct uio_fce_dev {
	struct uio_info *info;		// Ptr to  UIO info structure registered with the kernel
	void __iomem *fce_io_vaddr;	// Virtual address of the memory-mapped I/O region
	u32 irq;			// IRQ number assigned to the device
};

static int fce_probe(struct platform_device *pdev)
{
	struct uio_fce_dev *fce_dev;
	struct uio_info *info;
	struct resource *regs_fce_io;
	int ret, len;
	struct device *dev = &pdev->dev;

	fce_dev = devm_kzalloc(dev, sizeof(struct uio_fce_dev), GFP_KERNEL);
	if (!fce_dev)
		return -ENOMEM;

	fce_dev->info = devm_kzalloc(dev, sizeof(struct uio_info), GFP_KERNEL);
	if (!fce_dev->info)
		return -ENOMEM;

	regs_fce_io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs_fce_io || !regs_fce_io->start) {
		dev_err(dev, "NO FCE I/O reseource specified\n");
		return -ENODEV;
	}

	len = resource_size(regs_fce_io);
	fce_dev->fce_io_vaddr = devm_ioremap(dev, regs_fce_io->start, len);
	if (!fce_dev->fce_io_vaddr) {
		dev_err(dev, "Can't remap FCE I/O  address range\n");
		return -EIO;
	}

	/* Register UIO device */
	info = fce_dev->info;
	info->mem[0].name = "V2X FCE SHE0 MU";
	info->mem[0].addr = regs_fce_io->start;
	info->mem[0].offs = MU_BUF_OFFSET;
	info->mem[0].size = resource_size(regs_fce_io);
	info->mem[0].memtype = UIO_MEM_PHYS;
	info->mem[0].internal_addr = fce_dev->fce_io_vaddr;

	info->name = "FCE UIO";
	info->version = "UIO V2X FCE Driver 1.0";
	info->priv = fce_dev;

	ret = devm_uio_register_device(dev, info);
	if (ret) {
		dev_err(dev, "UIO V2X FCE register failed\n");
		return ret;
	}

	platform_set_drvdata(pdev, fce_dev);
	dev_info(dev, "%s initialized\n", info->name);

	return 0;
}

static const struct of_device_id uio_fce_ids[] = {
	{ .compatible = "fsl,imx94-mu-v2x-fce", },
	{},
};

MODULE_DEVICE_TABLE(of, uio_fce_ids);

static struct platform_driver uio_fce_driver = {
	.driver = {
		.name = "uio_v2x_fce",
		.of_match_table = uio_fce_ids,
	},
	.probe = fce_probe,
};

module_platform_driver(uio_fce_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("UIO V2X FCE Driver");
