/*
 * Copyright (C) 2005-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 *  linux/arch/arm/plat-mxc/epit.c
 *
 *  Copyright (C) 2010 Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <mach/hardware.h>
#include <asm/mach/time.h>
#include <mach/common.h>
#include <mach/epit.h>

struct epit_device {
	struct list_head	node;
	struct platform_device *pdev;
	const char	*label;
	struct clk	*clk;
	int		clk_enabled;
	void __iomem	*mmio_base;
	unsigned int	use_count;
	unsigned int	id;
	unsigned int	irq;
	int				mode;
	void (*cb)(void *);
	void *cb_para;
};


static DEFINE_MUTEX(epit_lock);
static LIST_HEAD(epit_list);

static inline void epit_irq_disable(struct epit_device *epit)
{
	u32 val;

	val = readl(epit->mmio_base + EPITCR);
	val &= ~EPITCR_OCIEN;
	writel(val, epit->mmio_base + EPITCR);
}

static inline void epit_irq_enable(struct epit_device *epit)
{
	u32 val;

	val = readl(epit->mmio_base + EPITCR);
	val |= EPITCR_OCIEN;
	writel(val, epit->mmio_base + EPITCR);
}

static inline void epit_irq_acknowledge(struct epit_device *epit)
{
	writel(EPITSR_OCIF, epit->mmio_base + EPITSR);
}

static irqreturn_t epit_timer_interrupt(int irq, void *epit)
{
	struct epit_device *epit_dev = epit;
	u32 cr = 0;

    if (epit_dev) {
		/* stop EPIT timer */
		cr = readl(epit_dev->mmio_base + EPITCR);
		cr &= ~(EPITCR_EN);
		writel(cr, epit_dev->mmio_base + EPITCR);
		/* clear EPIT interrupt flag */
	    epit_irq_acknowledge(epit_dev);
		if (epit_dev->cb)
			epit_dev->cb(epit_dev->cb_para);
	}
	return IRQ_HANDLED;
}

int epit_config(struct epit_device *epit, int mode, void *cb, void *para)
{
	u32 cr;
	unsigned int rc;

	if (epit == NULL)
		return -EINVAL;

	epit->cb = cb;
	epit->cb_para = para;

	/*SW Reset EPIT */
	writel(EPITCR_SWR, epit->mmio_base + EPITCR);
	while (readl(epit->mmio_base + EPITCR) & EPITCR_SWR)
		;

	/* reset EPIT register */
	writel(0x0, epit->mmio_base + EPITCR);
	/* clear EPIT interrupt flag */
	writel(EPITSR_OCIF, epit->mmio_base + EPITSR);
	/* set EPIT mode and clk src */
	cr = (mode << 3) | EPITCR_CLKSRC_REF_HIGH;
	cr |= (EPITCR_WAITEN | EPITCR_OCIEN | EPITCR_RLD);
	writel(cr, epit->mmio_base + EPITCR);
	/* write load counter */
	writel(0xFFFFFFFF, epit->mmio_base + EPITLR);

	epit->mode = mode;

	if (!epit->clk_enabled) {
		rc = clk_enable(epit->clk);
		if (!rc)
			epit->clk_enabled = 1;
	}

	/* Enable EPIT IRQ */
	epit_irq_enable(epit);
	return 0;
}
EXPORT_SYMBOL(epit_config);

void epit_start(struct epit_device *epit, int time_ns)
{
	u32 compare_count = 0;
	unsigned long long int c;
	unsigned int cycles, prescale;
	u32 cr;

	c = clk_get_rate(epit->clk);
	c = c * time_ns;
	do_div(c, 1000000000);
	cycles = c;
	prescale = cycles / 0x10000 + 1;
	cycles /= (prescale + 1);
	/* select prescale */
	cr = readl(epit->mmio_base + EPITCR);
	cr &= ~(EPITCR_PRESC(0xFFF));
	cr |= EPITCR_PRESC(prescale) ;
	writel(cr, epit->mmio_base + EPITCR);
	/* write compare count */
	if (EPIT_FREE_RUN_MODE == epit->mode) {
		compare_count = (0xFFFFFFFF - cycles + 1); /* down counter */
	} else {
		cr = readl(epit->mmio_base + EPITLR);
		compare_count = cr - cycles + 1;
	}
	writel(compare_count, epit->mmio_base + EPITCMPR);
	/* set EPIT Timer Mode */
	cr = readl(epit->mmio_base + EPITCR);
	if (EPIT_FREE_RUN_MODE == epit->mode)
		cr |= EPITCR_ENMOD;

	writel(cr, epit->mmio_base + EPITCR);
	/* start EPIT timer */
	cr = readl(epit->mmio_base + EPITCR);
	cr |= EPITCR_EN;
	writel(cr, epit->mmio_base + EPITCR);
}
EXPORT_SYMBOL(epit_start);

void epit_stop(struct epit_device *epit)
{
	u32 cr = 0;

	epit_irq_disable(epit);
	cr = readl(epit->mmio_base + EPITCR);
	cr &= ~EPITCR_EN;
	writel(cr, epit->mmio_base + EPITCR);
	/* reset EPIT register */
	writel(EPITCR_SWR, epit->mmio_base + EPITCR);
	while (readl(epit->mmio_base + EPITCR) & EPITCR_SWR)
		;

	if (epit->clk_enabled) {
		clk_disable(epit->clk);
		epit->clk_enabled = 0;
	}
}
EXPORT_SYMBOL(epit_stop);

struct epit_device *epit_request(int epit_id, const char *label)
{
	struct epit_device *epit;
	int found = 0;

	mutex_lock(&epit_lock);
	list_for_each_entry(epit, &epit_list, node) {
		if (epit->id == epit_id) {
			found = 1;
			break;
		}
	}
	if (found) {
		if (epit->use_count == 0) {
			epit->use_count++;
			epit->label = label;
		} else {
			epit = ERR_PTR(-EBUSY);
		}
	} else {
		epit = ERR_PTR(-ENOENT);
	}

	mutex_unlock(&epit_lock);
	return epit;
}
EXPORT_SYMBOL(epit_request);

void epit_free(struct epit_device *epit)
{
	mutex_lock(&epit_lock);
	if (epit->use_count) {
		epit->use_count--;
		epit->label = NULL;
	} else {
		pr_warning("EPIT device already freed\n");
	}
	mutex_unlock(&epit_lock);
}
EXPORT_SYMBOL(epit_free);

static int __devinit mxc_epit_probe(struct platform_device *pdev)
{
	struct epit_device *epit;
	struct resource *r;
	struct mxc_epit_platform_data *plat_data = pdev->dev.platform_data;
	int ret = 0;

	epit = kzalloc(sizeof(struct epit_device), GFP_KERNEL);
	if (epit == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	epit->clk = clk_get(&pdev->dev, "epit");
	if (IS_ERR(epit->clk)) {
		ret = PTR_ERR(epit->clk);
		goto err_free;
	}

	epit->clk_enabled = 0;
	epit->use_count = 0;
	epit->id = pdev->id;
	epit->pdev = pdev;

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	epit->irq = (unsigned int)(r->start);
	if (!epit->irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		ret = -ENODEV;
		goto err_free_clk;
	}

	/* setup IRQ */
	if (request_irq(epit->irq, epit_timer_interrupt, IRQF_DISABLED, "epit",
					(void *)epit)) {
		printk(KERN_ERR "rtc: cannot register IRQ %d\n", epit->irq);
		ret = -EIO;
		goto err_free_clk;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}

	r = request_mem_region(r->start, r->end - r->start + 1, pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free;
	}
	epit->mmio_base = ioremap(r->start, r->end - r->start + 1);
	if (epit->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	mutex_lock(&epit_lock);
	list_add_tail(&epit->node, &epit_list);
	mutex_unlock(&epit_lock);
	platform_set_drvdata(pdev, epit);
	return 0;
err_free_mem:
	release_mem_region(r->start, r->end - r->start + 1);
err_free_clk:
	clk_put(epit->clk);
err_free:
	kfree(epit);
	return ret;
}

static int __devexit mxc_epit_remove(struct platform_device *pdev)
{
	struct epit_device *epit;
	struct resource *r;

	epit = platform_get_drvdata(pdev);
	if (epit == NULL)
		return -ENODEV;

	mutex_lock(&epit_lock);
	list_del(&epit->node);
	mutex_unlock(&epit_lock);

	iounmap(epit->mmio_base);
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, r->end - r->start + 1);
	clk_put(epit->clk);
	kfree(epit);
	return 0;
}

static struct platform_driver mxc_epit_driver = {
	.driver = {
		.name = "mxc_epit",
	},
	.probe = mxc_epit_probe,
	.remove = __devexit_p(mxc_epit_remove),
};

static int __init mxc_epit_init(void)
{
	return platform_driver_register(&mxc_epit_driver);
}
arch_initcall(mxc_epit_init);

static void __exit mxc_epit_exit(void)
{
	platform_driver_unregister(&mxc_epit_driver);
}
module_exit(mxc_epit_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Enhance PIT driver");
MODULE_LICENSE("GPL");
