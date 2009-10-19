/*
 * RNG driver for Freescale RNGC
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
#include <linux/interrupt.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <asm/hardware.h>

#define RNGC_VERSION_MAJOR3 3

#define RNGC_VERSION_ID				0x0000
#define RNGC_COMMAND				0x0004
#define RNGC_CONTROL				0x0008
#define RNGC_STATUS				0x000C
#define RNGC_ERROR				0x0010
#define RNGC_FIFO				0x0014
#define RNGC_VERIF_CTRL				0x0020
#define RNGC_OSC_CTRL_COUNT			0x0028
#define RNGC_OSC_COUNT				0x002C
#define RNGC_OSC_COUNT_STATUS			0x0030

#define RNGC_VERID_ZEROS_MASK			0x0f000000
#define RNGC_VERID_RNG_TYPE_MASK		0xf0000000
#define RNGC_VERID_RNG_TYPE_SHIFT		28
#define RNGC_VERID_CHIP_VERSION_MASK		0x00ff0000
#define RNGC_VERID_CHIP_VERSION_SHIFT		16
#define RNGC_VERID_VERSION_MAJOR_MASK		0x0000ff00
#define RNGC_VERID_VERSION_MAJOR_SHIFT		8
#define RNGC_VERID_VERSION_MINOR_MASK		0x000000ff
#define RNGC_VERID_VERSION_MINOR_SHIFT		0

#define RNGC_CMD_ZEROS_MASK			0xffffff8c
#define RNGC_CMD_SW_RST				0x00000040
#define RNGC_CMD_CLR_ERR			0x00000020
#define RNGC_CMD_CLR_INT			0x00000010
#define RNGC_CMD_SEED				0x00000002
#define RNGC_CMD_SELF_TEST			0x00000001

#define RNGC_CTRL_ZEROS_MASK			0xfffffc8c
#define RNGC_CTRL_CTL_ACC			0x00000200
#define RNGC_CTRL_VERIF_MODE			0x00000100
#define RNGC_CTRL_MASK_ERROR			0x00000040

#define RNGC_CTRL_MASK_DONE			0x00000020
#define RNGC_CTRL_AUTO_SEED			0x00000010
#define RNGC_CTRL_FIFO_UFLOW_MASK		0x00000003
#define RNGC_CTRL_FIFO_UFLOW_SHIFT		0

#define RNGC_CTRL_FIFO_UFLOW_ZEROS_ERROR	0
#define RNGC_CTRL_FIFO_UFLOW_ZEROS_ERROR2	1
#define RNGC_CTRL_FIFO_UFLOW_BUS_XFR		2
#define RNGC_CTRL_FIFO_UFLOW_ZEROS_INTR		3

#define RNGC_STATUS_ST_PF_MASK			0x00c00000
#define RNGC_STATUS_ST_PF_SHIFT			22
#define RNGC_STATUS_ST_PF_TRNG			0x00800000
#define RNGC_STATUS_ST_PF_PRNG			0x00400000
#define RNGC_STATUS_ERROR			0x00010000
#define RNGC_STATUS_FIFO_SIZE_MASK		0x0000f000
#define RNGC_STATUS_FIFO_SIZE_SHIFT		12
#define RNGC_STATUS_FIFO_LEVEL_MASK		0x00000f00
#define RNGC_STATUS_FIFO_LEVEL_SHIFT		8
#define RNGC_STATUS_NEXT_SEED_DONE		0x00000040
#define RNGC_STATUS_SEED_DONE			0x00000020
#define RNGC_STATUS_ST_DONE			0x00000010
#define RNGC_STATUS_RESEED			0x00000008
#define RNGC_STATUS_SLEEP			0x00000004
#define RNGC_STATUS_BUSY			0x00000002
#define RNGC_STATUS_SEC_STATE			0x00000001

#define RNGC_ERROR_STATUS_ZEROS_MASK		0xffffffc0
#define RNGC_ERROR_STATUS_BAD_KEY		0x00000040
#define RNGC_ERROR_STATUS_RAND_ERR		0x00000020
#define RNGC_ERROR_STATUS_FIFO_ERR		0x00000010
#define RNGC_ERROR_STATUS_STAT_ERR		0x00000008
#define RNGC_ERROR_STATUS_ST_ERR		0x00000004
#define RNGC_ERROR_STATUS_OSC_ERR		0x00000002
#define RNGC_ERROR_STATUS_LFSR_ERR		0x00000001

#define RNG_ADDR_RANGE				0x34

static DECLARE_COMPLETION(rng_self_testing);
static DECLARE_COMPLETION(rng_seed_done);

static struct platform_device *rng_dev;

int irq_rng;

static int fsl_rngc_data_present(struct hwrng *rng)
{
	int level;
	u32 rngc_base = (u32) rng->priv;

	/* how many random numbers are in FIFO? [0-16] */
	level = (__raw_readl(rngc_base + RNGC_STATUS) &
	    RNGC_STATUS_FIFO_LEVEL_MASK) >> RNGC_STATUS_FIFO_LEVEL_SHIFT;

	return level > 0 ? 1 : 0;
}

static int fsl_rngc_data_read(struct hwrng *rng, u32 * data)
{
	int err;
	u32 rngc_base = (u32) rng->priv;

	/* retrieve a random number from FIFO */
	*data = __raw_readl(rngc_base + RNGC_FIFO);

	/* is there some error while reading this random number? */
	err = __raw_readl(rngc_base + RNGC_STATUS) & RNGC_STATUS_ERROR;

	/* if error happened doesn't return random number */
	return err ? 0 : 4;
}

static irqreturn_t rngc_irq(int irq, void *dev)
{
	int handled = 0;
	u32 rngc_base = (u32) dev;

	/* is the seed creation done? */
	if (__raw_readl(rngc_base + RNGC_STATUS) & RNGC_STATUS_SEED_DONE) {
		complete(&rng_seed_done);
		__raw_writel(RNGC_CMD_CLR_INT | RNGC_CMD_CLR_ERR,
		       rngc_base + RNGC_COMMAND);
		handled = 1;
	}

	/* is the self test done? */
	if (__raw_readl(rngc_base + RNGC_STATUS) & RNGC_STATUS_ST_DONE) {
		complete(&rng_self_testing);
		__raw_writel(RNGC_CMD_CLR_INT | RNGC_CMD_CLR_ERR,
			rngc_base + RNGC_COMMAND);
		handled = 1;
	}

	/* is there any error? */
	if (__raw_readl(rngc_base + RNGC_STATUS) & RNGC_STATUS_ERROR) {
		/* clear interrupt */
		__raw_writel(RNGC_CMD_CLR_INT | RNGC_CMD_CLR_ERR,
		       rngc_base + RNGC_COMMAND);
		handled = 1;
	}

	return handled;
}

static int fsl_rngc_init(struct hwrng *rng)
{
	int err;
	u32 cmd, ctrl, osc, rngc_base = (u32) rng->priv;

	INIT_COMPLETION(rng_self_testing);
	INIT_COMPLETION(rng_seed_done);

	err = __raw_readl(rngc_base + RNGC_STATUS) & RNGC_STATUS_ERROR;
	if (err) {
		/* is this a bad keys error ? */
		if (__raw_readl(rngc_base + RNGC_ERROR) &
		   RNGC_ERROR_STATUS_BAD_KEY) {
			dev_err(&rng_dev->dev, "Can't start, Bad Keys.\n");
			return -EIO;
		}
	}

	/* mask all interrupts, will be unmasked soon */
	ctrl = __raw_readl(rngc_base + RNGC_CONTROL);
	__raw_writel(ctrl | RNGC_CTRL_MASK_DONE | RNGC_CTRL_MASK_ERROR,
	       rngc_base + RNGC_CONTROL);

	/* verify if oscillator is working */
	osc = __raw_readl(rngc_base + RNGC_ERROR);
	if (osc & RNGC_ERROR_STATUS_OSC_ERR) {
		dev_err(&rng_dev->dev, "RNGC Oscillator is dead!\n");
		return -EIO;
	}

	err = request_irq(irq_rng, rngc_irq, 0, "fsl_rngc", (void *)rng->priv);
	if (err) {
		dev_err(&rng_dev->dev, "Can't get interrupt working.\n");
		return -EIO;
	}

	/* do self test, repeat until get success */
	do {
		/* clear error */
		cmd = __raw_readl(rngc_base + RNGC_COMMAND);
		__raw_writel(cmd | RNGC_CMD_CLR_ERR, rngc_base + RNGC_COMMAND);

		/* unmask all interrupt */
		ctrl = __raw_readl(rngc_base + RNGC_CONTROL);
		__raw_writel(ctrl & ~(RNGC_CTRL_MASK_DONE |
			RNGC_CTRL_MASK_ERROR), rngc_base + RNGC_CONTROL);

		/* run self test */
		cmd = __raw_readl(rngc_base + RNGC_COMMAND);
		__raw_writel(cmd | RNGC_CMD_SELF_TEST,
			rngc_base + RNGC_COMMAND);

		wait_for_completion(&rng_self_testing);

	} while (__raw_readl(rngc_base + RNGC_ERROR) &
		RNGC_ERROR_STATUS_ST_ERR);

	/* clear interrupt. Is it really necessary here? */
	__raw_writel(RNGC_CMD_CLR_INT | RNGC_CMD_CLR_ERR,
		rngc_base + RNGC_COMMAND);

	/* create seed, repeat while there is some statistical error */
	do {
		/* clear error */
		cmd = __raw_readl(rngc_base + RNGC_COMMAND);
		__raw_writel(cmd | RNGC_CMD_CLR_ERR, rngc_base + RNGC_COMMAND);

		/* seed creation */
		cmd = __raw_readl(rngc_base + RNGC_COMMAND);
		__raw_writel(cmd | RNGC_CMD_SEED, rngc_base + RNGC_COMMAND);

		wait_for_completion(&rng_seed_done);

	} while (__raw_readl(rngc_base + RNGC_ERROR) &
		RNGC_ERROR_STATUS_STAT_ERR);

	err = __raw_readl(rngc_base + RNGC_ERROR) &
					(RNGC_ERROR_STATUS_STAT_ERR |
					RNGC_ERROR_STATUS_RAND_ERR |
					RNGC_ERROR_STATUS_FIFO_ERR |
					RNGC_ERROR_STATUS_ST_ERR |
					RNGC_ERROR_STATUS_OSC_ERR |
					RNGC_ERROR_STATUS_LFSR_ERR);

	if (err) {
		dev_err(&rng_dev->dev, "FSL RNGC appears inoperable.\n");
		return -EIO;
	}

	return 0;
}

static struct hwrng fsl_rngc = {
	.name = "fsl-rngc",
	.init = fsl_rngc_init,
	.data_present = fsl_rngc_data_present,
	.data_read = fsl_rngc_data_read
};

static int __init fsl_rngc_probe(struct platform_device *pdev)
{
	int err = -ENODEV;
	struct clk *clk;
	struct resource *res, *mem;
	void __iomem *rngc_base = NULL;

	if (rng_dev)
		return -EBUSY;

	clk = clk_get(NULL, "rng_clk");

	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Can not get rng_clk\n");
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
	rngc_base = ioremap(res->start, res->end - res->start);

	fsl_rngc.priv = (unsigned long)rngc_base;

	irq_rng = platform_get_irq(pdev, 0);

	err = hwrng_register(&fsl_rngc);
	if (err) {
		dev_err(&pdev->dev, "FSL RNGC registering failed (%d)\n", err);
		return err;
	}

	rng_dev = pdev;

	dev_info(&pdev->dev, "FSL RNGC Registered.\n");

	return 0;
}

static int __exit fsl_rngc_remove(struct platform_device *pdev)
{
	struct resource *mem = dev_get_drvdata(&pdev->dev);
	void __iomem *rngc_base = (void __iomem *)fsl_rngc.priv;

	hwrng_unregister(&fsl_rngc);

	release_resource(mem);

	iounmap(rngc_base);

	return 0;
}

static struct platform_driver fsl_rngc_driver = {
	.driver = {
		   .name = "fsl_rngc",
		   .owner = THIS_MODULE,
		   },
	.remove = __exit_p(fsl_rngc_remove),
};

static int __init mod_init(void)
{
	return platform_driver_probe(&fsl_rngc_driver, fsl_rngc_probe);
}

static void __exit mod_exit(void)
{
	platform_driver_unregister(&fsl_rngc_driver);
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("H/W RNGC driver for i.MX");
MODULE_LICENSE("GPL");
