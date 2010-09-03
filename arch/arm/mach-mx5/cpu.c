/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
void __iomem *databahn_base;
void *wait_in_iram_base;
void (*wait_in_iram)(void *ccm_addr, void *databahn_addr);

extern void mx50_wait(u32 ccm_base, u32 databahn_addr);

static int cpu_silicon_rev = -1;

#define SI_REV 0x48

static void query_silicon_parameter(void)
{
	void __iomem *rom = ioremap(IROM_BASE_ADDR, IROM_SIZE);
	u32 rev;

	if (!rom) {
		cpu_silicon_rev = -EINVAL;
		return;
	}

	rev = readl(rom + SI_REV);
	switch (rev) {
	case 0x1:
		cpu_silicon_rev = CHIP_REV_1_0;
		break;
	case 0x2:
		cpu_silicon_rev = CHIP_REV_1_1;
		break;
	case 0x10:
		cpu_silicon_rev = CHIP_REV_2_0;
		break;
	case 0x20:
		cpu_silicon_rev = CHIP_REV_3_0;
		break;
	default:
		cpu_silicon_rev = 0;
	}

	iounmap(rom);
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
		query_silicon_parameter();

	return cpu_silicon_rev;
}
EXPORT_SYMBOL(mx51_revision);

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

	databahn_base = ioremap(MX50_DATABAHN_BASE_ADDR, SZ_16K);

	if (cpu_is_mx50()) {
		struct clk *ddr_clk = clk_get(NULL, "ddr_clk");
		unsigned long iram_paddr;

		iram_alloc(SZ_4K, &iram_paddr);
		/* Need to remap the area here since we want the memory region
			 to be executable. */
		wait_in_iram_base = __arm_ioremap(iram_paddr,
							SZ_4K, MT_HIGH_VECTORS);
		memcpy(wait_in_iram_base, mx50_wait, SZ_4K);
		wait_in_iram = (void *)wait_in_iram_base;

		clk_enable(ddr_clk);

		/* Set the DDR to enter automatic self-refresh. */
		/* Set the DDR to automatically enter lower power mode 4. */
		reg = __raw_readl(databahn_base + DATABAHN_CTL_REG22);
		reg &= ~LOWPOWER_AUTOENABLE_MASK;
		reg |= 1 << 1;
		__raw_writel(reg, databahn_base + DATABAHN_CTL_REG22);

		/* set the counter for entering mode 4. */
		reg = __raw_readl(databahn_base + DATABAHN_CTL_REG21);
		reg &= ~LOWPOWER_EXTERNAL_CNT_MASK;
		reg = 128 << LOWPOWER_EXTERNAL_CNT_OFFSET;
		__raw_writel(reg, databahn_base + DATABAHN_CTL_REG21);

		/* Enable low power mode 4 */
		reg = __raw_readl(databahn_base + DATABAHN_CTL_REG20);
		reg &= ~LOWPOWER_CONTROL_MASK;
		reg |= 1 << 1;
		__raw_writel(reg, databahn_base + DATABAHN_CTL_REG20);
		clk_disable(ddr_clk);
	}
	return 0;
}

postcore_initcall(post_cpu_init);
