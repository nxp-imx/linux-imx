/*
 * RNG driver for Freescale RNGA
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * Hardware driver for the Intel/AMD/VIA Random Number Generators (RNG)
 * (c) Copyright 2003 Red Hat Inc <jgarzik@redhat.com>
 *
 * derived from
 *
 * Hardware driver for the AMD 768 Random Number Generator (RNG)
 * (c) Copyright 2001 Red Hat Inc <alan@redhat.com>
 *
 * derived from
 *
 * Hardware driver for Intel i810 Random Number Generator (RNG)
 * Copyright 2000,2001 Jeff Garzik <jgarzik@pobox.com>
 * Copyright 2000,2001 Philipp Rumpf <prumpf@mandrakesoft.com>
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/hw_random.h>
#include <linux/io.h>

/* RNGA Registers */
#define RNGA_CONTROL			0x00
#define RNGA_STATUS			0x04
#define RNGA_ENTROPY			0x08
#define RNGA_OUTPUT_FIFO		0x0c
#define RNGA_MODE			0x10
#define RNGA_VERIFICATION_CONTROL	0x14
#define RNGA_OSC_CONTROL_COUNTER	0x18
#define RNGA_OSC1_COUNTER		0x1c
#define RNGA_OSC2_COUNTER		0x20
#define RNGA_OSC_COUNTER_STATUS		0x24

/* RNGA Registers Range */
#define RNG_ADDR_RANGE			0x28

/* RNGA Control Register */
#define RNGA_CONTROL_SLEEP		0x00000010
#define RNGA_CONTROL_CLEAR_INT		0x00000008
#define RNGA_CONTROL_MASK_INTS		0x00000004
#define RNGA_CONTROL_HIGH_ASSURANCE	0x00000002
#define RNGA_CONTROL_GO			0x00000001

#define RNGA_STATUS_LEVEL_MASK		0x0000ff00

/* RNGA Status Register */
#define RNGA_STATUS_OSC_DEAD		0x80000000
#define RNGA_STATUS_SLEEP		0x00000010
#define RNGA_STATUS_ERROR_INT		0x00000008
#define RNGA_STATUS_FIFO_UNDERFLOW	0x00000004
#define RNGA_STATUS_LAST_READ_STATUS	0x00000002
#define RNGA_STATUS_SECURITY_VIOLATION	0x00000001

static struct platform_device *rng_dev;

static int fsl_rnga_data_present(struct hwrng *rng)
{
	int level;
	u32 rng_base = (u32) rng->priv;

	/* how many random numbers is in FIFO? [0-16] */
	level = ((__raw_readl(rng_base + RNGA_STATUS) &
			RNGA_STATUS_LEVEL_MASK) >> 8);

	return level > 0 ? 1 : 0;
}

static int fsl_rnga_data_read(struct hwrng *rng, u32 * data)
{
	int err;
	u32 ctrl, rng_base = (u32) rng->priv;

	/* retrieve a random number from FIFO */
	*data = __raw_readl(rng_base + RNGA_OUTPUT_FIFO);

	/* some error while reading this random number? */
	err = __raw_readl(rng_base + RNGA_STATUS) & RNGA_STATUS_ERROR_INT;

	/* if error: clear error interrupt, but doesn't return random number */
	if (err) {
		dev_dbg(&rng_dev->dev, "Error while reading random number!\n");
		ctrl = __raw_readl(rng_base + RNGA_CONTROL);
		__raw_writel(ctrl | RNGA_CONTROL_CLEAR_INT,
					rng_base + RNGA_CONTROL);
		return 0;
	} else
		return 4;
}

static int fsl_rnga_init(struct hwrng *rng)
{
	u32 ctrl, osc, rng_base = (u32) rng->priv;

	/* wake up */
	ctrl = __raw_readl(rng_base + RNGA_CONTROL);
	__raw_writel(ctrl & ~RNGA_CONTROL_SLEEP, rng_base + RNGA_CONTROL);

	/* verify if oscillator is working */
	osc = __raw_readl(rng_base + RNGA_STATUS);
	if (osc & RNGA_STATUS_OSC_DEAD) {
		dev_err(&rng_dev->dev, "RNGA Oscillator is dead!\n");
		return -ENODEV;
	}

	/* go running */
	ctrl = __raw_readl(rng_base + RNGA_CONTROL);
	__raw_writel(ctrl | RNGA_CONTROL_GO, rng_base + RNGA_CONTROL);

	return 0;
}

static void fsl_rnga_cleanup(struct hwrng *rng)
{
	u32 ctrl, rng_base = (u32) rng->priv;

	ctrl = __raw_readl(rng_base + RNGA_CONTROL);

	/* stop rnga */
	__raw_writel(ctrl & ~RNGA_CONTROL_GO, rng_base + RNGA_CONTROL);
}

static struct hwrng fsl_rnga = {
	.name = "fsl-rnga",
	.init = fsl_rnga_init,
	.cleanup = fsl_rnga_cleanup,
	.data_present = fsl_rnga_data_present,
	.data_read = fsl_rnga_data_read
};

static int __init fsl_rnga_probe(struct platform_device *pdev)
{
	int err = -ENODEV;
	struct clk *clk;
	struct resource *res, *mem;
	void __iomem *rng_base = NULL;

	if (rng_dev)
		return -EBUSY;

	clk = clk_get(NULL, "rng_clk");

	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Could not get rng_clk!\n");
		err = PTR_ERR(clk);
		return err;
	}

	clk_enable(clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res)
		return -ENOENT;

	mem = request_mem_region(res->start, res->end - res->start, pdev->name);

	if (mem == NULL)
		return -EBUSY;

	dev_set_drvdata(&pdev->dev, mem);
	rng_base = ioremap(res->start, res->end - res->start);

	fsl_rnga.priv = (unsigned long)rng_base;

	err = hwrng_register(&fsl_rnga);
	if (err) {
		dev_err(&pdev->dev, "FSL RNGA registering failed (%d)\n", err);
		return err;
	}

	rng_dev = pdev;

	dev_info(&pdev->dev, "FSL RNGA Registered.\n");

	return 0;
}

static int __exit fsl_rnga_remove(struct platform_device *pdev)
{
	struct resource *mem = dev_get_drvdata(&pdev->dev);
	void __iomem *rng_base = (void __iomem *)fsl_rnga.priv;

	hwrng_unregister(&fsl_rnga);

	release_resource(mem);

	iounmap(rng_base);

	return 0;
}

static struct platform_driver fsl_rnga_driver = {
	.driver = {
		   .name = "fsl_rnga",
		   .owner = THIS_MODULE,
		   },
	.remove = __exit_p(fsl_rnga_remove),
};

static int __init mod_init(void)
{
	return platform_driver_probe(&fsl_rnga_driver, fsl_rnga_probe);
}

static void __exit mod_exit(void)
{
	platform_driver_unregister(&fsl_rnga_driver);
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("H/W RNGA driver for i.MX");
MODULE_LICENSE("GPL");
