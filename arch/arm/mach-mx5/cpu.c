/*
 * Copyright 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/proc_fs.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/iram_alloc.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/mach/map.h>

#define CORTEXA8_PLAT_AMC	0x18
#define SRPG_NEON_PUPSCR	0x284
#define SRPG_NEON_PDNSCR	0x288
#define SRPG_ARM_PUPSCR		0x2A4
#define SRPG_ARM_PDNSCR		0x2A8
#define SRPG_EMPGC0_PUPSCR	0x2E4
#define SRPG_EMPGC0_PDNSCR	0x2E8
#define SRPG_EMPGC1_PUPSCR	0x304
#define SRPG_EMPGC1_PDNSCR	0x308

void __iomem *arm_plat_base;
void __iomem *gpc_base;
void __iomem *ccm_base;
extern void init_ddr_settings(void);

static int cpu_silicon_rev = -1;

#define IIM_SREV 0x24
#define MX50_HW_ADADIG_DIGPROG	0xB0

static int get_mx51_srev(void)
{
	void __iomem *iim_base = IO_ADDRESS(IIM_BASE_ADDR);
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

static int get_mx53_srev(void)
{
	void __iomem *iim_base = IO_ADDRESS(IIM_BASE_ADDR);
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

static int get_mx50_srev(void)
{
	void __iomem *anatop = ioremap(ANATOP_BASE_ADDR, SZ_8K);
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

struct cpu_wp *(*get_cpu_wp)(int *wp);
void (*set_num_cpu_wp)(int num);

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

		temp = __raw_readl(reg_hsc_mxt_conf);
		__raw_writel(temp | 0x10000, reg_hsc_mxt_conf);

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
	struct clk *gpcclk = clk_get(NULL, "gpc_dvfs_clk");
	int iram_size = IRAM_SIZE;

	if (!cpu_is_mx5())
		return 0;

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
	ccm_base = ioremap(MX53_BASE_ADDR(CCM_BASE_ADDR), SZ_4K);

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
	arm_plat_base = ioremap(MX53_BASE_ADDR(ARM_BASE_ADDR), SZ_4K);
	reg = 0x8;
	__raw_writel(reg, arm_plat_base + CORTEXA8_PLAT_AMC);

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

	if (cpu_is_mx51() || cpu_is_mx53()) {
		/*Allow for automatic gating of the EMI internal clock.
		 * If this is done, emi_intr CCGR bits should be set to 11.
		 */
		base = ioremap(MX53_BASE_ADDR(M4IF_BASE_ADDR), SZ_4K);
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
