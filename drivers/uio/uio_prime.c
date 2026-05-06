// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025-2026 NXP
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>

#include "atu.h"

#define GIER_OFFSET	0x110  /* Global Interrupt Enable Register offset */
#define GSR_OFFSET	0x118  /* Global Status Register offset */
#define IRQ_BIT		BIT(0) /* Status bit asserted on IRQ (example) */
#define GIER_ENABLE	BIT(0) /* Enable mask for global interrupt */

/*
 * This structure holds the private data associated with a UIO device,
 * typically used in a platform that exposes hardware to userspace via the UIO framework
 */
struct uio_prime {
	struct uio_info *uio;	// Ptr to  UIO info structure registered with the kernel
	void __iomem *regs;	// Virtual address of the memory-mapped I/O region
	u32 irq;		// IRQ number assigned to the device

	/* ATU parameters */
	struct atu_dev atu;

	/* Misc device for IOCTL interface */
	struct miscdevice miscdev;
};

/*
 * IOCTL handler for miscdevice
 */
static long prime_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct uio_prime *prime_priv = container_of(filp->private_data,
						    struct uio_prime, miscdev);
	return atu_ioctl(&prime_priv->atu, cmd, arg);
}

static int prime_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct uio_prime *prime_priv = container_of(filp->private_data,
						    struct uio_prime, miscdev);
	return atu_mmap(&prime_priv->atu, vma);
}

static const struct file_operations prime_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = prime_ioctl,
	.compat_ioctl = prime_ioctl,
	.mmap = prime_mmap,
};

static irqreturn_t prime_irqhandler(int irq, struct uio_info *uio)
{
	struct uio_prime *priv = uio->priv;
	u32 st = readl(priv->regs + GSR_OFFSET);

	/* IRQ guard: only handle if our bit is set */
	if (!(st & IRQ_BIT))
		return IRQ_NONE;

	/* MASK further device interrupts until userspace re-enables */
	writel(0, priv->regs + GIER_OFFSET);

	/* ACK interrupt */
	writel(IRQ_BIT, priv->regs + GSR_OFFSET);
	readl(priv->regs + GSR_OFFSET);  // flush

	return IRQ_HANDLED;
}

/* IRQ Control: called when user writes 0 or 1 to /dev/uioX */
static int prime_irqcontrol(struct uio_info *uio, s32 irq_on)
{
	struct uio_prime *priv = uio->priv;

	if (irq_on)
		writel(GIER_ENABLE, priv->regs + GIER_OFFSET);
	else
		writel(0, priv->regs + GIER_OFFSET);

	return 0;
}

static int prime_open(struct uio_info *uio, struct inode *inode)
{
	struct uio_prime *priv = uio->priv;

	/* Clear any stale status bits */
	u32 st = readl(priv->regs + GSR_OFFSET);

	if (st)
		writel(st, priv->regs + GSR_OFFSET);

	/* Enable interrupt */
	writel(GIER_ENABLE, priv->regs + GIER_OFFSET);

	return 0;
}

static int prime_release(struct uio_info *uio, struct inode *inode)
{
	struct uio_prime *priv = uio->priv;

	/* Disable interrupt */
	writel(0, priv->regs + GIER_OFFSET);

	return 0;
}

static int prime_probe(struct platform_device *pdev)
{
	struct uio_prime *prime_priv;
	struct uio_info *uio;
	struct resource *mu_res, *atu_res;
	struct device *dev = &pdev->dev;
	struct device_node *rmem_np;
	struct reserved_mem *rmem;
	int ret;

	/* Allocate driver private structure */
	prime_priv = devm_kzalloc(dev, sizeof(*prime_priv), GFP_KERNEL);
	if (!prime_priv)
		return -ENOMEM;

	uio = devm_kzalloc(dev, sizeof(struct uio_info), GFP_KERNEL);
	if (!uio)
		return -ENOMEM;

	/* Allow 64-bit DMA */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "cannot set DMA mask: %d\n", ret);
		return ret;
	}

	/* -----------------------------
	 * 1. Map PRIME MMIO (MU registers)
	 * ------------------------------
	 */
	mu_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mu_res || !mu_res->start) {
		dev_err(dev, "no MU MMIO resource\n");
		return -ENODEV;
	}

	prime_priv->regs = devm_ioremap_resource(dev, mu_res);
	if (IS_ERR(prime_priv->regs))
		return PTR_ERR(prime_priv->regs);

	/* Fetch IRQ from DT (or board init) */
	prime_priv->irq = platform_get_irq(pdev, 0);
	if (prime_priv->irq < 0) {
		dev_err(dev, "get irq failed\n");
		return prime_priv->irq;
	}

	/* -----------------------------
	 * 2. Fill UIO info
	 * -----------------------------
	 */
	uio->name = "PRIME UIO";
	uio->version = "PRIME UIO Driver 1.0";
	uio->priv = prime_priv;

	/* MU registers exposed to userspace */
	uio->mem[0].name = "PRIME MMIO";
	uio->mem[0].addr = mu_res->start;
	uio->mem[0].size = resource_size(mu_res);
	uio->mem[0].memtype = UIO_MEM_PHYS;
	uio->mem[0].internal_addr = prime_priv->regs;

	/* IRQ for UIO */
	uio->irq =  prime_priv->irq;
	uio->irq_flags = IRQF_SHARED;
	uio->handler = prime_irqhandler;
	uio->irqcontrol = prime_irqcontrol;
	uio->open = prime_open;
	uio->release = prime_release;

	ret = devm_uio_register_device(dev, uio);
	if (ret) {
		dev_err(dev, "UIO register failed: %d\n", ret);
		return -ENODEV;
	}

	prime_priv->uio = uio;

	/* ------------------------------
	 * 3. Map ATU MMIO (ATU registers)
	 * ------------------------------
	 */
	atu_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!atu_res) {
		dev_err(dev, "No ATU MMIO resource\n");
		goto skip_atu;
	}

	prime_priv->atu.atu_phys = atu_res->start;
	prime_priv->atu.atu_size = resource_size(atu_res);

	prime_priv->atu.atu_base = devm_ioremap_resource(dev, atu_res);
	if (IS_ERR(prime_priv->atu.atu_base)) {
		prime_priv->atu.atu_phys = 0;
		return PTR_ERR(prime_priv->atu.atu_base);
	}

	/* -----------------------------
	 * 4. Reserved-memory binding
	 * -----------------------------
	 */
	ret = of_reserved_mem_device_init(dev);
	if (ret) {
		dev_err(dev, "reserved mem init failed: %d\n", ret);
		goto no_resv_mem;
	}

	rmem_np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!rmem_np) {
		dev_err(dev, "memory-region phandle missing in DT\n");
		of_reserved_mem_device_release(dev);
		goto no_resv_mem;
	}

	rmem = of_reserved_mem_lookup(rmem_np);
	of_node_put(rmem_np);

	if (!rmem) {
		dev_err(dev, "failed to lookup reserved memory block\n");
		of_reserved_mem_device_release(dev);
		goto no_resv_mem;
	}

	if (!rmem->size) {
		dev_err(dev, "Reserved memory block size is zero!\n");
		of_reserved_mem_device_release(dev);
		goto no_resv_mem;
	}

	prime_priv->atu.resv_mem_phys_addr = rmem->base;
	prime_priv->atu.resv_mem_size      = rmem->size;

no_resv_mem:
	ret = atu_ll_init(&prime_priv->atu);
	if (ret)
		goto err_free_reserved;

	/* ---------------------------------
	 * 5. Register misc device for IOCTL
	 * ---------------------------------
	 */
	prime_priv->miscdev.minor = MISC_DYNAMIC_MINOR;
	prime_priv->miscdev.name = "prime";
	prime_priv->miscdev.fops = &prime_fops;
	prime_priv->miscdev.parent = dev;

	ret = misc_register(&prime_priv->miscdev);
	if (ret) {
		dev_err(dev, "misc device registration failed: %d\n", ret);
		goto err_free_reserved;
	}

	dev_info(dev, " ATU MMIO:      %pa (%u bytes)\n"
		 " Prime Reserved mem:  phys=0x%llx size=0x%x MB\n",
		 &prime_priv->atu.atu_phys, prime_priv->atu.atu_size,
		 prime_priv->atu.resv_mem_phys_addr,
		 prime_priv->atu.resv_mem_size / (1024 * 1024));
skip_atu:
	platform_set_drvdata(pdev, prime_priv);
	dev_info(dev, "%s initialized (irq=%d, mmio=%pa..%pa)\n",
		 uio->name, prime_priv->irq, &mu_res->start, &mu_res->end);

	return 0;

err_free_reserved:
	if (prime_priv->atu.resv_mem_phys_addr)
		of_reserved_mem_device_release(dev);

	return ret;
}

static void prime_remove(struct platform_device *pdev)
{
	struct uio_prime *prime_priv = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	if (prime_priv->atu.atu_phys) {
		misc_deregister(&prime_priv->miscdev);
		mutex_destroy(&prime_priv->atu.mem_lock);

		if (prime_priv->atu.resv_mem_phys_addr)
			of_reserved_mem_device_release(dev);
	}
}

static const struct of_device_id prime_of_match[] = {
	{ .compatible = "fsl,imx94-mu-prime", },
	{ .compatible = "fsl,imx952-mu-prime", },
	{},
};

MODULE_DEVICE_TABLE(of, prime_of_match);

static struct platform_driver prime_drv = {
	.driver = {
		.name = "prime-uio",
		.of_match_table = prime_of_match,
	},
	.probe = prime_probe,
	.remove = prime_remove,
};

module_platform_driver(prime_drv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("PRIME UIO Driver with IRQ support");
