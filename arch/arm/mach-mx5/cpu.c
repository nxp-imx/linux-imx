/*
 * Copyright 2008-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * This file contains the CPU initialization code.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/iram_alloc.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include <mach/hardware.h>
#include <asm/io.h>

#define CORTEXA8_PLAT_AMC       0x18
#define SRPG_NEON_PUPSCR        0x284
#define SRPG_NEON_PDNSCR        0x288
#define SRPG_ARM_PUPSCR         0x2A4
#define SRPG_ARM_PDNSCR         0x2A8
#define SRPG_EMPGC0_PUPSCR      0x2E4
#define SRPG_EMPGC0_PDNSCR      0x2E8
#define SRPG_EMPGC1_PUPSCR      0x304
#define SRPG_EMPGC1_PDNSCR      0x308

void __iomem *arm_plat_base;
void __iomem *gpc_base;
void __iomem *ccm_base;
struct cpu_op *(*get_cpu_op)(int *op);

extern void init_ddr_settings(void);

static int cpu_silicon_rev = -1;

#define IIM_SREV 0x24
#define MX50_HW_ADADIG_DIGPROG	0xB0

static int get_mx51_srev(void)
{
	void __iomem *iim_base = MX51_IO_ADDRESS(MX51_IIM_BASE_ADDR);
	u32 rev = readl(iim_base + IIM_SREV) & 0xff;

	if (rev == 0x0)
		return IMX_CHIP_REVISION_2_0;
	else if (rev == 0x10)
		return IMX_CHIP_REVISION_3_0;
	return 0;
}

/*
 * Returns:
 *	the silicon revision of the cpu
 *	-EINVAL - not a mx51
 */
int mx51_revision(void)
{
	if (!cpu_is_mx51())
		return -EINVAL;

	if (cpu_silicon_rev == -1)
		cpu_silicon_rev = get_mx51_srev();

	return cpu_silicon_rev;
}
EXPORT_SYMBOL(mx51_revision);

void mx51_display_revision(void)
{
	int rev;
	char *srev;
	rev = mx51_revision();

	switch (rev) {
	case IMX_CHIP_REVISION_2_0:
		srev = IMX_CHIP_REVISION_2_0_STRING;
		break;
	case IMX_CHIP_REVISION_3_0:
		srev = IMX_CHIP_REVISION_3_0_STRING;
		break;
	default:
		srev = IMX_CHIP_REVISION_UNKNOWN_STRING;
	}
	printk(KERN_INFO "CPU identified as i.MX51, silicon rev %s\n", srev);
}
EXPORT_SYMBOL(mx51_display_revision);

#ifdef CONFIG_NEON

/*
 * All versions of the silicon before Rev. 3 have broken NEON implementations.
 * Dependent on link order - so the assumption is that vfp_init is called
 * before us.
 */
static int __init mx51_neon_fixup(void)
{
	if (!cpu_is_mx51())
		return 0;

	if (mx51_revision() < IMX_CHIP_REVISION_3_0 && (elf_hwcap & HWCAP_NEON)) {
		elf_hwcap &= ~HWCAP_NEON;
		pr_info("Turning off NEON support, detected broken NEON implementation\n");
	}
	return 0;
}

late_initcall(mx51_neon_fixup);
#endif

static int get_mx53_srev(void)
{
	void __iomem *iim_base = MX53_IO_ADDRESS(MX53_IIM_BASE_ADDR);
	u32 rev = readl(iim_base + IIM_SREV) & 0xff;

	switch (rev) {
	case 0x0:
		return IMX_CHIP_REVISION_1_0;
	case 0x2:
		return IMX_CHIP_REVISION_2_0;
	case 0x3:
		return IMX_CHIP_REVISION_2_1;
	default:
		return IMX_CHIP_REVISION_UNKNOWN;
	}
}

/*
 * Returns:
 *	the silicon revision of the cpu
 *	-EINVAL - not a mx53
 */
int mx53_revision(void)
{
	if (!cpu_is_mx53())
		return -EINVAL;

	if (cpu_silicon_rev == -1)
		cpu_silicon_rev = get_mx53_srev();

	return cpu_silicon_rev;
}
EXPORT_SYMBOL(mx53_revision);
#define MX50_HW_ADADIG_DIGPROG	0xB0

static int get_mx50_srev(void)
{
	void __iomem *anatop = ioremap(MX50_ANATOP_BASE_ADDR, SZ_8K);
	u32 rev;

	if (!anatop) {
		cpu_silicon_rev = -EINVAL;
		return 0;
	}

	rev = readl(anatop + MX50_HW_ADADIG_DIGPROG);
	rev &= 0xff;

	iounmap(anatop);
	if (rev == 0x0)
		return IMX_CHIP_REVISION_1_0;
	else if (rev == 0x1)
		return IMX_CHIP_REVISION_1_1;
	return 0;
}

/*
 * Returns:
 *	the silicon revision of the cpu
 *	-EINVAL - not a mx50
 */
int mx50_revision(void)
{
	if (!cpu_is_mx50())
		return -EINVAL;

	if (cpu_silicon_rev == -1)
		cpu_silicon_rev = get_mx50_srev();

	return cpu_silicon_rev;
}
EXPORT_SYMBOL(mx50_revision);

void mx53_display_revision(void)
{
	int rev;
	char *srev;
	rev = mx53_revision();

	switch (rev) {
	case IMX_CHIP_REVISION_1_0:
		srev = IMX_CHIP_REVISION_1_0_STRING;
		break;
	case IMX_CHIP_REVISION_2_0:
		srev = IMX_CHIP_REVISION_2_0_STRING;
		break;
	case IMX_CHIP_REVISION_2_1:
		srev = IMX_CHIP_REVISION_2_1_STRING;
		break;
	default:
		srev = IMX_CHIP_REVISION_UNKNOWN_STRING;
	}
	printk(KERN_INFO "CPU identified as i.MX53, silicon rev %s\n", srev);
}
EXPORT_SYMBOL(mx53_display_revision);

static int __init post_cpu_init(void)
{
	unsigned int reg;
	void __iomem *base;
	struct clk *gpcclk = clk_get(NULL, "gpc_dvfs_clk");

	if (cpu_is_mx51()) {
		ccm_base = MX51_IO_ADDRESS(MX51_CCM_BASE_ADDR);
		gpc_base = MX51_IO_ADDRESS(MX51_GPC_BASE_ADDR);
		arm_plat_base = MX51_IO_ADDRESS(MX51_ARM_BASE_ADDR);
		iram_init(MX51_IRAM_BASE_ADDR, MX51_IRAM_SIZE);
	} else if (cpu_is_mx53()) {
		ccm_base = MX53_IO_ADDRESS(MX53_CCM_BASE_ADDR);
		gpc_base = MX53_IO_ADDRESS(MX53_GPC_BASE_ADDR);
		arm_plat_base = MX53_IO_ADDRESS(MX53_ARM_BASE_ADDR);
		iram_init(MX53_IRAM_BASE_ADDR, MX53_IRAM_SIZE);
	} else {
		ccm_base = MX50_IO_ADDRESS(MX50_CCM_BASE_ADDR);
		gpc_base = MX50_IO_ADDRESS(MX50_GPC_BASE_ADDR);
		arm_plat_base = MX50_IO_ADDRESS(MX50_ARM_BASE_ADDR);
		iram_init(MX50_IRAM_BASE_ADDR, MX50_IRAM_SIZE);
	}

	if (cpu_is_mx51() || cpu_is_mx53()) {
		if (cpu_is_mx51()) {
			base = MX51_IO_ADDRESS(MX51_AIPS1_BASE_ADDR);
		} else {
			base = MX53_IO_ADDRESS(MX53_AIPS1_BASE_ADDR);
		}

		__raw_writel(0x0, base + 0x40);
		__raw_writel(0x0, base + 0x44);
		__raw_writel(0x0, base + 0x48);
		__raw_writel(0x0, base + 0x4C);
		reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
		__raw_writel(reg, base + 0x50);

		if (cpu_is_mx51())
			base = MX51_IO_ADDRESS(MX51_AIPS2_BASE_ADDR);
		else
			base = MX53_IO_ADDRESS(MX53_AIPS2_BASE_ADDR);

		__raw_writel(0x0, base + 0x40);
		__raw_writel(0x0, base + 0x44);
		__raw_writel(0x0, base + 0x48);
		__raw_writel(0x0, base + 0x4C);
		reg = __raw_readl(base + 0x50) & 0x00FFFFFF;
		__raw_writel(reg, base + 0x50);
	}
	clk_enable(gpcclk);

	/* Setup the number of clock cycles to wait for SRPG
	* power up and power down requests.
	*/
	__raw_writel(0x010F0201, gpc_base + SRPG_ARM_PUPSCR);
	__raw_writel(0x010F0201, gpc_base + SRPG_NEON_PUPSCR);
	__raw_writel(0x00000008, gpc_base + SRPG_EMPGC0_PUPSCR);
	__raw_writel(0x00000008, gpc_base + SRPG_EMPGC1_PUPSCR);

	__raw_writel(0x01010101, gpc_base + SRPG_ARM_PDNSCR);
	__raw_writel(0x01010101, gpc_base + SRPG_NEON_PDNSCR);
	__raw_writel(0x00000018, gpc_base + SRPG_EMPGC0_PDNSCR);
	__raw_writel(0x00000018, gpc_base + SRPG_EMPGC1_PDNSCR);

	clk_disable(gpcclk);
	clk_put(gpcclk);

	/* Set ALP bits to 000. Set ALP_EN bit in Arm Memory Controller reg. */
	reg = 0x8;
	__raw_writel(reg, arm_plat_base + CORTEXA8_PLAT_AMC);

	if (cpu_is_mx53()) {
		/*Allow for automatic gating of the EMI internal clock.
		 * If this is done, emi_intr CCGR bits should be set to 11.
		 */
		base = ioremap(MX53_M4IF_BASE_ADDR, SZ_4K);
		reg = __raw_readl(base + 0x8c);
		reg &= ~0x1;
		__raw_writel(reg, base + 0x8c);
		iounmap(base);
	}

	if (cpu_is_mx50())
		init_ddr_settings();

	return 0;
}

postcore_initcall(post_cpu_init);
