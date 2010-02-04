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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/iram_alloc.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include "crm_regs.h"

void __iomem *arm_plat_base;

/*!
 * CPU initialization. It is called by fixup_mxc_board()
 */
void __init mxc_cpu_init(void)
{
	if (!system_rev)
		mxc_set_system_rev(0x51, CHIP_REV_1_0);
}

static int __init post_cpu_init(void)
{
	void __iomem *base;
	unsigned int reg;
	int iram_size = IRAM_SIZE;

#if defined(CONFIG_MXC_SECURITY_SCC) || defined(CONFIG_MXC_SECURITY_SCC_MODULE)
	if (cpu_is_mx51())
		iram_size -= SCC_RAM_SIZE;
#endif
	iram_init(IRAM_BASE_ADDR, iram_size);

	/* Set ALP bits to 000. Set ALP_EN bit in Arm Memory Controller reg. */
	arm_plat_base = ioremap(ARM_BASE_ADDR, SZ_4K);
	reg = 0x8;
	__raw_writel(reg, MXC_CORTEXA8_PLAT_AMC);

	base = ioremap(AIPS1_BASE_ADDR, SZ_4K);
	__raw_writel(0x0, base + 0x40);
	__raw_writel(0x0, base + 0x44);
	__raw_writel(0x0, base + 0x48);
	__raw_writel(0x0, base + 0x4C);
	reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
	__raw_writel(reg, base + 0x50);
	iounmap(base);

	base = ioremap(AIPS2_BASE_ADDR, SZ_4K);
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
	base = ioremap(M4IF_BASE_ADDR, SZ_4K);
	reg = __raw_readl(base + 0x8c);
	reg &= ~0x1;
	__raw_writel(reg, base + 0x8c);
	iounmap(base);

	return 0;
}

postcore_initcall(post_cpu_init);
