/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mach-mx51/cpu.c
 *
 * @brief This file contains the CPU initialization code.
 *
 * @ingroup MSL_MX51
 */

#include <linux/types.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/iram_alloc.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include "crm_regs.h"

void __iomem *arm_plat_base;
void __iomem *gpc_base;

static void __init mipi_hsc_disable(void)
{
	void __iomem *reg_hsc_mcd = ioremap(MIPI_HSC_BASE_ADDR, SZ_4K);
	void __iomem *reg_hsc_mxt_conf = reg_hsc_mcd + 0x800;
	struct clk *clk;
	uint32_t temp;

	/* Temporarily setup MIPI module to legacy mode */
	clk = clk_get(NULL, "mipi_hsp_clk");
	if (!IS_ERR(clk)) {
		clk_enable(clk);

		/* Temporarily setup MIPI module to legacy mode */
		__raw_writel(0xF00, reg_hsc_mcd);

		/* CSI mode reserved*/
		temp = __raw_readl(reg_hsc_mxt_conf);
		__raw_writel(temp | 0x0FF, reg_hsc_mxt_conf);

		if (cpu_is_mx51_rev(CHIP_REV_2_0) > 0) {
			temp = __raw_readl(reg_hsc_mxt_conf);
			__raw_writel(temp | 0x10000, reg_hsc_mxt_conf);
		}

		clk_disable(clk);
		clk_put(clk);
	}
	iounmap(reg_hsc_mcd);
}

/*!
 * This function resets IPU
 */
void mx5_ipu_reset(void)
{
	u32 *reg;
	u32 value;
	reg = ioremap(MX53_BASE_ADDR(SRC_BASE_ADDR), PAGE_SIZE);
	value = __raw_readl(reg);
	value = value | 0x8;
	__raw_writel(value, reg);
	iounmap(reg);
}

void mx5_vpu_reset(void)
{
	u32 reg;
	void __iomem *src_base;

	src_base = ioremap(MX53_BASE_ADDR(SRC_BASE_ADDR), PAGE_SIZE);

	/* mask interrupt due to vpu passed reset */
	reg = __raw_readl(src_base + 0x18);
	reg |= 0x02;
	__raw_writel(reg, src_base + 0x18);

	reg = __raw_readl(src_base);
	reg |= 0x5;    /* warm reset vpu */
	__raw_writel(reg, src_base);
	while (__raw_readl(src_base) & 0x04)
		;

	iounmap(src_base);
}

static int __init post_cpu_init(void)
{
	void __iomem *base;
	unsigned int reg;
	int iram_size = IRAM_SIZE;

	if (cpu_is_mx51()) {
		mipi_hsc_disable();

#if defined(CONFIG_MXC_SECURITY_SCC) || defined(CONFIG_MXC_SECURITY_SCC_MODULE)
		iram_size -= SCC_RAM_SIZE;
#endif
		iram_init(MX51_IRAM_BASE_ADDR, iram_size);
	} else {
		iram_init(MX53_IRAM_BASE_ADDR, iram_size);
	}

	gpc_base = ioremap(MX53_BASE_ADDR(GPC_BASE_ADDR), SZ_4K);

	/* Set ALP bits to 000. Set ALP_EN bit in Arm Memory Controller reg. */
	arm_plat_base = ioremap(MX53_BASE_ADDR(ARM_BASE_ADDR), SZ_4K);
	reg = 0x8;
	__raw_writel(reg, MXC_CORTEXA8_PLAT_AMC);

	base = ioremap(MX53_BASE_ADDR(AIPS1_BASE_ADDR), SZ_4K);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);
	iounmap(base);

	base = ioremap(MX53_BASE_ADDR(AIPS2_BASE_ADDR), SZ_4K);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);
	iounmap(base);

	/*Allow for automatic gating of the EMI internal clock.
	 * If this is done, emi_intr CCGR bits should be set to 11.
	 */
	base = ioremap(MX53_BASE_ADDR(M4IF_BASE_ADDR), SZ_4K);
	reg = __raw_readl(base + 0x8c);
	reg &= ~0x1;
	__raw_writel(reg, base + 0x8c);
	iounmap(base);

	return 0;
}

postcore_initcall(post_cpu_init);
