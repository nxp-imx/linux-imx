/*
 * Copyright 2007-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_vl2cc.c
 *
 * @brief VL2CC initialization and flush operation implementation
 *
 * @ingroup VL2CC
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>

#define VL2CC_CTRL_OFFSET	(0x100)
#define VL2CC_AUXCTRL_OFFSET	(0x104)
#define VL2CC_INVWAY_OFFSET	(0x77C)
#define VL2CC_CLEANWAY_OFFSET	(0x7BC)

/*! VL2CC clock handle. */
static struct clk *vl2cc_clk;
static u32 *vl2cc_base;

/*!
 * Initialization function of VL2CC. Remap the VL2CC base address.
 *
 * @return status  0 success.
 */
int vl2cc_init(u32 vl2cc_hw_base)
{
	vl2cc_base = ioremap(vl2cc_hw_base, SZ_8K - 1);
	if (vl2cc_base == NULL) {
		printk(KERN_INFO "vl2cc: Unable to ioremap\n");
		return -ENOMEM;
	}

	vl2cc_clk = clk_get(NULL, "vl2cc_clk");
	if (IS_ERR(vl2cc_clk)) {
		printk(KERN_INFO "vl2cc: Unable to get clock\n");
		iounmap(vl2cc_base);
		return -EIO;
	}

	printk(KERN_INFO "VL2CC initialized\n");
	return 0;
}

/*!
 * Enable VL2CC hardware
 */
void vl2cc_enable(void)
{
	volatile u32 reg;

	clk_enable(vl2cc_clk);

	/* Disable VL2CC */
	reg = __raw_readl(vl2cc_base + VL2CC_CTRL_OFFSET);
	reg &= 0xFFFFFFFE;
	__raw_writel(reg, vl2cc_base + VL2CC_CTRL_OFFSET);

	/* Set the latency for data RAM reads, data RAM writes, tag RAM and
	 * dirty RAM to 1 cycle - write 0x0 to AUX CTRL [11:0] and also
	 * configure the number of ways to 8 - write 8 to AUX CTRL [16:13]
	 */
	reg = __raw_readl(vl2cc_base + VL2CC_AUXCTRL_OFFSET);
	reg &= 0xFFFE1000;	/* Clear [16:13] too */
	reg |= (0x8 << 13);	/* [16:13] = 8; */
	__raw_writel(reg, vl2cc_base + VL2CC_AUXCTRL_OFFSET);

	/* Invalidate the VL2CC ways - write 0xff to INV BY WAY and poll the
	 * register until its value is 0x0
	 */
	__raw_writel(0xff, vl2cc_base + VL2CC_INVWAY_OFFSET);
	while (__raw_readl(vl2cc_base + VL2CC_INVWAY_OFFSET) != 0x0)
		;

	/* Enable VL2CC */
	reg = __raw_readl(vl2cc_base + VL2CC_CTRL_OFFSET);
	reg |= 0x1;
	__raw_writel(reg, vl2cc_base + VL2CC_CTRL_OFFSET);
}

/*!
 * Flush VL2CC
 */
void vl2cc_flush(void)
{
	__raw_writel(0xff, vl2cc_base + VL2CC_CLEANWAY_OFFSET);
	while (__raw_readl(vl2cc_base + VL2CC_CLEANWAY_OFFSET) != 0x0)
		;
}

/*!
 * Disable VL2CC
 */
void vl2cc_disable(void)
{
	__raw_writel(0, vl2cc_base + VL2CC_CTRL_OFFSET);
	clk_disable(vl2cc_clk);
}

/*!
 * Cleanup VL2CC
 */
void vl2cc_cleanup(void)
{
	clk_put(vl2cc_clk);
	iounmap(vl2cc_base);
}
