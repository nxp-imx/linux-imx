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
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct device_node *rmem_np;
	int ret;

	prime_priv = devm_kzalloc(dev, sizeof(struct uio_prime), GFP_KERNEL);
	if (!prime_priv)
		return -ENOMEM;

	uio = devm_kzalloc(dev, sizeof(struct uio_info), GFP_KERNEL);
	if (!uio)
		return -ENOMEM;

	/* MMIO resource 0: device registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || !res->start) {
		dev_err(dev, "no MMIO resource\n");
		return -ENODEV;
	}

	prime_priv->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(prime_priv->regs))
		return PTR_ERR(prime_priv->regs);

	/* Fetch IRQ from DT (or board init) */
	prime_priv->irq = platform_get_irq(pdev, 0);
	if (prime_priv->irq < 0) {
		dev_err(dev, "get irq failed\n");
		return prime_priv->irq;
	}

	/* Fill UIO info */
	uio->name = "PRIME UIO";
	uio->version = "PRIME UIO Driver 1.0";
	uio->priv = prime_priv;

	/* Map the register window to userspace */
	uio->mem[0].name = "PRIME MMIO";
	uio->mem[0].addr = res->start;
	uio->mem[0].size = resource_size(res);
	uio->mem[0].memtype = UIO_MEM_PHYS;
	uio->mem[0].internal_addr = prime_priv->regs;

	/* IRQ for UIO */
	uio->irq =  prime_priv->irq;
	uio->irq_flags = IRQF_SHARED;
	uio->handler = prime_irqhandler;
	uio->irqcontrol = prime_irqcontrol;
	uio->open = prime_open;
	uio->release = prime_release;

	rmem_np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (rmem_np) {
		struct reserved_mem *rmem = of_reserved_mem_lookup(rmem_np);

		of_node_put(rmem_np);
		if (rmem) {
			uio->mem[1].name = "PRIME Reserved Memory";
			uio->mem[1].addr = rmem->base;
			uio->mem[1].size = rmem->size;
			uio->mem[1].memtype = UIO_MEM_PHYS;
			uio->mem[1].internal_addr = NULL;
		} else
			dev_warn(dev, "Reserved memory not found; map1 not created\n");
	}

	ret = devm_uio_register_device(dev, uio);
	if (ret) {
		dev_err(dev, "UIO register failed: %d\n", ret);
		return ret;
	}

	ret = of_reserved_mem_device_init(dev);
	if (ret) {
		dev_err(dev, "reserved mem init failed: %d\n", ret);
		return ret;
	}

	prime_priv->uio = uio;
	platform_set_drvdata(pdev, prime_priv);
	dev_info(dev, "%s initialized (irq=%d, mmio=%pa..%pa)\n",
		 uio->name, prime_priv->irq, &res->start, &res->end);

	return 0;
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
};

module_platform_driver(prime_drv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("PRIME UIO Driver with IRQ support");
