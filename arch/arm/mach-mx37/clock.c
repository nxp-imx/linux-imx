/*
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <asm/div64.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/clock.h>
#include <mach/mxc_dptc.h>
#include <mach/spba.h>
#include <mach/mxc_uart.h>

#include "crm_regs.h"
#include "iomux.h"

extern int mxc_jtag_enabled;
extern int cpufreq_trig_needed;
extern int dvfs_core_is_active;

static unsigned long pll_base[] = {
	(unsigned long)MXC_DPLL1_BASE,
	(unsigned long)MXC_DPLL2_BASE,
	(unsigned long)MXC_DPLL3_BASE,
};

static struct clk pll1_main_clk;
static struct clk pll1_sw_clk;
static struct clk pll2_sw_clk;
static struct clk pll3_sw_clk;
static struct clk lp_apm_clk;
static struct clk emi_core_clk;
static struct clk emi_fast_clk;
static struct clk emi_slow_clk;
static struct clk emi_intr_clk;
static struct clk ddr_clk;
static struct clk ipu_clk[];
static struct clk axi_a_clk;
static struct clk axi_b_clk;
static struct clk axi_c_clk;
static struct clk ahb_clk;

int cpu_wp_nr;
int lp_high_freq;
int lp_med_freq;
static int cpu_curr_wp;
static struct cpu_wp *cpu_wp_tbl;

extern void propagate_rate(struct clk *tclk);
extern void board_ref_clk_rate(unsigned long *ckil, unsigned long *osc,
			       unsigned long *ckih);
static int cpu_clk_set_wp(int wp);

static int _clk_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(clk->enable_reg);
	reg |= MXC_CCM_CCGR_CG_MASK << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	return 0;
}

static void _clk_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGR_CG_MASK << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);
}

static void _clk_disable_inwait(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGR_CG_MASK << clk->enable_shift);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);
}

/*
 * For the 4-to-1 muxed input clock
 */
static inline u32 _get_mux(struct clk *parent, struct clk *m0,
			   struct clk *m1, struct clk *m2, struct clk *m3)
{
	if (parent == m0) {
		return 0;
	} else if (parent == m1) {
		return 1;
	} else if (parent == m2) {
		return 2;
	} else if (parent == m3) {
		return 3;
	} else {
		BUG();
	}
	return 0;
}

static inline unsigned long _get_pll_base(struct clk *pll)
{
	if (pll == &pll1_main_clk) {
		return pll_base[0];
	} else if (pll == &pll2_sw_clk) {
		return pll_base[1];
	} else if (pll == &pll3_sw_clk) {
		return pll_base[2];
	} else {
		BUG();
	}
	return 0;
}

static struct clk ckih_clk = {
	.name = "ckih",
	.flags = RATE_PROPAGATES,
};

static struct clk osc_clk = {
	.name = "osc",
	.flags = RATE_PROPAGATES,
};

static struct clk ckil_clk = {
	.name = "ckil",
	.flags = RATE_PROPAGATES,
};

static void _fpm_recalc(struct clk *clk)
{
	clk->rate = ckil_clk.rate * 512;
	if ((__raw_readl(MXC_CCM_CCR) & MXC_CCM_CCR_FPM_MULT_MASK) != 0) {
		clk->rate *= 2;
	}
}

static int _fpm_enable(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CCR);
	reg |= MXC_CCM_CCR_FPM_EN;
	__raw_writel(reg, MXC_CCM_CCR);
	return 0;
}

static void _fpm_disable(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CCR);
	reg &= ~MXC_CCM_CCR_FPM_EN;
	__raw_writel(reg, MXC_CCM_CCR);
}

static struct clk fpm_clk = {
	.name = "fpm_clk",
	.parent = &ckil_clk,
	.recalc = _fpm_recalc,
	.enable = _fpm_enable,
	.disable = _fpm_disable,
	.flags = RATE_PROPAGATES,
};

static void _fpm_div2_recalc(struct clk *clk)
{
	clk->rate = clk->parent->rate / 2;
}

static struct clk fpm_div2_clk = {
	.name = "fpm_div2_clk",
	.parent = &fpm_clk,
	.recalc = _fpm_div2_recalc,
	.flags = RATE_PROPAGATES,
};

static int _clk_pll_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 pllbase;

	long mfi, pdf, mfn, mfd = 999999;
	s64 temp64;
	unsigned long quad_parent_rate;
	unsigned long pll_hfsm, dp_ctl;

	pllbase = _get_pll_base(clk);

	quad_parent_rate = 4*clk->parent->rate;
	pdf = mfi = -1;
	while (++pdf < 16 && mfi < 5)
		mfi = rate * (pdf+1) / quad_parent_rate;
	if (mfi > 15)
		return -1;
	pdf--;

	temp64 = rate*(pdf+1) - quad_parent_rate*mfi;
	do_div(temp64, quad_parent_rate/1000000);
	mfn = (long)temp64;

	dp_ctl = __raw_readl(pllbase + MXC_PLL_DP_CTL);
	/* use dpdck0_2 */
	__raw_writel(dp_ctl | 0x1000L, pllbase + MXC_PLL_DP_CTL);
	pll_hfsm = dp_ctl & MXC_PLL_DP_CTL_HFSM;
	if (pll_hfsm == 0) {
		reg = mfi<<4 | pdf;
		__raw_writel(reg, pllbase + MXC_PLL_DP_OP);
		__raw_writel(mfd, pllbase + MXC_PLL_DP_MFD);
		__raw_writel(mfn, pllbase + MXC_PLL_DP_MFN);
	} else {
		reg = mfi<<4 | pdf;
		__raw_writel(reg, pllbase + MXC_PLL_DP_HFS_OP);
		__raw_writel(mfd, pllbase + MXC_PLL_DP_HFS_MFD);
		__raw_writel(mfn, pllbase + MXC_PLL_DP_HFS_MFN);
	}

	return 0;
}

static void _clk_pll_recalc(struct clk *clk)
{
	long mfi, mfn, mfd, pdf, ref_clk, mfn_abs;
	unsigned long dp_op, dp_mfd, dp_mfn, dp_ctl, pll_hfsm, dbl;
	unsigned long pllbase;
	s64 temp;

	pllbase = _get_pll_base(clk);

	dp_ctl = __raw_readl(pllbase + MXC_PLL_DP_CTL);
	pll_hfsm = dp_ctl & MXC_PLL_DP_CTL_HFSM;
	dbl = dp_ctl & MXC_PLL_DP_CTL_DPDCK0_2_EN;

	if (pll_hfsm == 0) {
		dp_op = __raw_readl(pllbase + MXC_PLL_DP_OP);
		dp_mfd = __raw_readl(pllbase + MXC_PLL_DP_MFD);
		dp_mfn = __raw_readl(pllbase + MXC_PLL_DP_MFN);
	} else {
		dp_op = __raw_readl(pllbase + MXC_PLL_DP_HFS_OP);
		dp_mfd = __raw_readl(pllbase + MXC_PLL_DP_HFS_MFD);
		dp_mfn = __raw_readl(pllbase + MXC_PLL_DP_HFS_MFN);
	}
	pdf = dp_op & MXC_PLL_DP_OP_PDF_MASK;
	mfi = (dp_op & MXC_PLL_DP_OP_MFI_MASK) >> MXC_PLL_DP_OP_MFI_OFFSET;
	mfi = (mfi <= 5) ? 5 : mfi;
	mfd = dp_mfd & MXC_PLL_DP_MFD_MASK;
	mfn = mfn_abs = dp_mfn & MXC_PLL_DP_MFN_MASK;
	/* Sign extend to 32-bits */
	if (mfn >= 0x04000000) {
		mfn |= 0xFC000000;
		mfn_abs = -mfn;
	}

	ref_clk = 2 * clk->parent->rate;
	if (dbl != 0) {
		ref_clk *= 2;
	}
	ref_clk /= (pdf + 1);
	temp = (u64) ref_clk *mfn_abs;
	do_div(temp, mfd + 1);
	if (mfn < 0)
		temp = -temp;
	temp = (ref_clk * mfi) + temp;

	clk->rate = temp;
}

static int _clk_pll_enable(struct clk *clk)
{
	u32 reg;
	u32 pllbase;

	pllbase = _get_pll_base(clk);
	reg = __raw_readl(pllbase + MXC_PLL_DP_CTL) | MXC_PLL_DP_CTL_UPEN;
	__raw_writel(reg, pllbase + MXC_PLL_DP_CTL);

	/* Wait for lock */
	while (!(__raw_readl(pllbase + MXC_PLL_DP_CTL) & MXC_PLL_DP_CTL_LRF)) ;

	return 0;
}

static void _clk_pll_disable(struct clk *clk)
{
	u32 reg;
	u32 pllbase;

	pllbase = _get_pll_base(clk);
	reg = __raw_readl(pllbase + MXC_PLL_DP_CTL) & ~MXC_PLL_DP_CTL_UPEN;
	__raw_writel(reg, pllbase + MXC_PLL_DP_CTL);
}

static struct clk pll1_main_clk = {
	.name = "pll1_main_clk",
	.parent = &osc_clk,
	.recalc = _clk_pll_recalc,
	.set_rate = _clk_pll_set_rate,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.flags = RATE_PROPAGATES,
};

static int _clk_pll1_sw_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CCSR);

	if (parent == &pll1_main_clk) {
		reg &= ~MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
	} else {
		mux = _get_mux(parent, &lp_apm_clk, NULL, &pll2_sw_clk,
			       &pll3_sw_clk);
		reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
		    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
		__raw_writel(reg, MXC_CCM_CCSR);
		reg = __raw_readl(MXC_CCM_CCSR);
		reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
	}
	__raw_writel(reg, MXC_CCM_CCSR);
	return 0;
}

static void _clk_pll1_sw_recalc(struct clk *clk)
{
	u32 reg, div;
	div = 1;
	reg = __raw_readl(MXC_CCM_CCSR);

	if (clk->parent == &pll2_sw_clk) {
		div = ((reg & MXC_CCM_CCSR_PLL2_PODF_MASK) >>
		       MXC_CCM_CCSR_PLL2_PODF_OFFSET) + 1;
	} else if (clk->parent == &pll3_sw_clk) {
		div = ((reg & MXC_CCM_CCSR_PLL3_PODF_MASK) >>
		       MXC_CCM_CCSR_PLL3_PODF_OFFSET) + 1;
	}
	clk->rate = clk->parent->rate / div;
}

/* pll1 switch clock */
static struct clk pll1_sw_clk = {
	.name = "pll1_sw_clk",
	.parent = &pll1_main_clk,
	.set_parent = _clk_pll1_sw_set_parent,
	.recalc = _clk_pll1_sw_recalc,
	.flags = RATE_PROPAGATES,
};

/* same as pll2_main_clk. These two clocks should always be the same */
static struct clk pll2_sw_clk = {
	.name = "pll2",
	.parent = &osc_clk,
	.recalc = _clk_pll_recalc,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.flags = RATE_PROPAGATES,
};

/* same as pll3_main_clk. These two clocks should always be the same */
static struct clk pll3_sw_clk = {
	.name = "pll3",
	.parent = &osc_clk,
	.recalc = _clk_pll_recalc,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.flags = RATE_PROPAGATES,
};

static int _clk_lp_apm_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	if (parent == &osc_clk) {
		reg = __raw_readl(MXC_CCM_CCSR) & ~MXC_CCM_CCSR_LP_APM_SEL;
	} else if (parent == &fpm_clk) {
		reg = __raw_readl(MXC_CCM_CCSR) | MXC_CCM_CCSR_LP_APM_SEL;
	} else {
		return -EINVAL;
	}
	__raw_writel(reg, MXC_CCM_CCSR);

	return 0;
}

static struct clk lp_apm_clk = {
	.name = "lp_apm",
	.parent = &osc_clk,
	.set_parent = _clk_lp_apm_set_parent,
	.flags = RATE_PROPAGATES,
};

static void _clk_arm_recalc(struct clk *clk)
{
	u32 cacrr, div;

	cacrr = __raw_readl(MXC_CCM_CACRR);
	div = (cacrr & MXC_CCM_CACRR_ARM_PODF_MASK) + 1;
	clk->rate = clk->parent->rate / div;
}

static int _clk_cpu_set_rate(struct clk *clk, unsigned long rate)
{
	u32 i;
	for (i = 0; i < cpu_wp_nr; i++) {
		if (rate == cpu_wp_tbl[i].cpu_rate)
			break;
	}
	if (i > cpu_wp_nr)
		return -EINVAL;
	cpu_clk_set_wp(i);

	return 0;
}

static unsigned long _clk_cpu_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 i;
	u32 wp;

	for (i = 0; i < cpu_wp_nr; i++) {
		if (rate == cpu_wp_tbl[i].cpu_rate)
			break;
	}

	if (i > cpu_wp_nr)
		wp = 0;

	return cpu_wp_tbl[wp].cpu_rate;
}

static struct clk cpu_clk = {
	.name = "cpu_clk",
	.parent = &pll1_sw_clk,
	.recalc = _clk_arm_recalc,
	.set_rate = _clk_cpu_set_rate,
	.round_rate = _clk_cpu_round_rate,
};

static int _clk_periph_apm_set_parent(struct clk *clk,
						struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll3_sw_clk, &lp_apm_clk, NULL);

	reg = __raw_readl(MXC_CCM_CAMR) & ~MXC_CCM_CAMR_PERIPH_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CAMR_PERIPH_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CAMR);

	return 0;
}

static struct clk periph_apm_clk = {
	.name = "periph_apm_clk",
	.parent = &pll1_sw_clk,
	.set_parent = _clk_periph_apm_set_parent,
	.flags = RATE_PROPAGATES,
};

static void _clk_main_bus_recalc(struct clk *clk)
{
	clk->rate = clk->parent->rate;
}

static int _clk_main_bus_set_rate(struct clk *clk, unsigned long rate)
{
	u32 div = 0;

	clk->rate = clk->parent->rate/(div + 1);
	return 0;
}
static int _clk_main_bus_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, stat;

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);
	if (emi_intr_clk.usecount == 0)
		emi_intr_clk.enable(&emi_intr_clk);

	if (ipu_clk[0].usecount == 0)
		ipu_clk[0].enable(&ipu_clk[0]);

	if (parent == &pll2_sw_clk) {
		reg = __raw_readl(MXC_CCM_CBCDR6) &
		    ~MXC_CCM_CBCDR6_PERIPH_CLK_SEL;
	} else if (parent == &periph_apm_clk) {
		reg = __raw_readl(MXC_CCM_CBCDR6) |
		    MXC_CCM_CBCDR6_PERIPH_CLK_SEL;
	} else {
		return -EINVAL;
	}
	__raw_writel(reg, MXC_CCM_CBCDR6);

	/* Set the Load-dividers bit in CCM */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_LOAD_DIVIDERS;
	__raw_writel(reg, MXC_CCM_CCDR);

	do {
		stat = __raw_readl(MXC_CCM_CCDR) & MXC_CCM_CCDR_LOAD_DIVIDERS;
	} while (stat);

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);
	if (emi_intr_clk.usecount == 0)
		emi_intr_clk.disable(&emi_intr_clk);

	if (ipu_clk[0].usecount == 0)
		ipu_clk[0].enable(&ipu_clk[0]);

	return 0;
}

static struct clk main_bus_clk = {
	.name = "main_bus_clk",
	.parent = &pll2_sw_clk,
	.set_parent = _clk_main_bus_set_parent,
	.set_rate = _clk_main_bus_set_rate,
	.recalc = _clk_main_bus_recalc,
	.flags = RATE_PROPAGATES,
};

static int _clk_axi_a_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, stat;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	if (ddr_clk.parent == &axi_a_clk && emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);

	reg = __raw_readl(MXC_CCM_CBCDR3);
	reg &= ~MXC_CCM_CBCDR3_AXI_A_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR3_AXI_A_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR3);

	/* Set the Load-dividers bit in CCM */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_LOAD_DIVIDERS;
	__raw_writel(reg, MXC_CCM_CCDR);

	do {
		stat = __raw_readl(MXC_CCM_CCDR) & MXC_CCM_CCDR_LOAD_DIVIDERS;
	} while (stat);
	clk->rate = rate;

	if (ddr_clk.parent == &axi_a_clk && emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);

	return 0;
}

static void _clk_axi_a_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR3);
	div = ((reg & MXC_CCM_CBCDR3_AXI_A_PODF_MASK) >>
	       MXC_CCM_CBCDR3_AXI_A_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static unsigned long _clk_axi_a_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return clk->parent->rate / div;
}

static struct clk axi_a_clk = {
	.name = "axi_a_clk",
	.parent = &main_bus_clk,
	.recalc = _clk_axi_a_recalc,
	.set_rate = _clk_axi_a_set_rate,
	.round_rate = _clk_axi_a_round_rate,
	.flags = RATE_PROPAGATES,
};

static int _clk_axi_b_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, stat;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	if (ddr_clk.parent == &axi_b_clk && emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);

	reg = __raw_readl(MXC_CCM_CBCDR4);
	reg &= ~MXC_CCM_CBCDR4_AXI_B_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR4_AXI_B_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR4);

	/* Set the Load-dividers bit in CCM */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_LOAD_DIVIDERS;
	__raw_writel(reg, MXC_CCM_CCDR);

	do {
		stat = __raw_readl(MXC_CCM_CCDR) & MXC_CCM_CCDR_LOAD_DIVIDERS;
	} while (stat);
	clk->rate = rate;

	if (ddr_clk.parent == &axi_c_clk && emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);

	return 0;
}

static void _clk_axi_b_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR4);
	div = ((reg & MXC_CCM_CBCDR4_AXI_B_PODF_MASK) >>
	       MXC_CCM_CBCDR4_AXI_B_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static unsigned long _clk_axi_b_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return clk->parent->rate / div;
}

static struct clk axi_b_clk = {
	.name = "axi_b_clk",
	.parent = &main_bus_clk,
	.recalc = _clk_axi_b_recalc,
	.set_rate = _clk_axi_b_set_rate,
	.round_rate = _clk_axi_b_round_rate,
	.flags = RATE_PROPAGATES,
};

static int _clk_axi_c_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, stat;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	if (ddr_clk.parent == &axi_c_clk && emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);

	reg = __raw_readl(MXC_CCM_CBCDR5);
	reg &= ~MXC_CCM_CBCDR5_AXI_C_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR5_AXI_C_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR5);

	/* Set the Load-dividers bit in CCM */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_LOAD_DIVIDERS;
	__raw_writel(reg, MXC_CCM_CCDR);

	do {
		stat = __raw_readl(MXC_CCM_CCDR) & MXC_CCM_CCDR_LOAD_DIVIDERS;
	} while (stat);
	clk->rate = rate;

	if (ddr_clk.parent == &axi_c_clk && emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);

	return 0;
}

static void _clk_axi_c_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR5);
	div = ((reg & MXC_CCM_CBCDR5_AXI_C_PODF_MASK) >>
	       MXC_CCM_CBCDR5_AXI_C_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static unsigned long _clk_axi_c_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return clk->parent->rate / div;
}

static struct clk axi_c_clk = {
	.name = "axi_c_clk",
	.parent = &main_bus_clk,
	.recalc = _clk_axi_c_recalc,
	.set_rate = _clk_axi_c_set_rate,
	.round_rate = _clk_axi_c_round_rate,
	.flags = RATE_PROPAGATES,
};

static void _clk_ahb_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR2);
	div = ((reg & MXC_CCM_CBCDR2_AHB_PODF_MASK) >>
	       MXC_CCM_CBCDR2_AHB_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static int _clk_ahb_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR2);
	reg &= ~MXC_CCM_CBCDR2_AHB_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR2_AHB_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR2);

	/* Set the Load-dividers bit in CCM */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_LOAD_DIVIDERS;
	__raw_writel(reg, MXC_CCM_CCDR);
	clk->rate = rate;

	return 0;
}

static unsigned long _clk_ahb_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return clk->parent->rate / div;
}

static struct clk ahb_clk = {
	.name = "ahb_clk",
	.parent = &main_bus_clk,
	.recalc = _clk_ahb_recalc,
	.set_rate = _clk_ahb_set_rate,
	.round_rate = _clk_ahb_round_rate,
	.flags = RATE_PROPAGATES,
};

static struct clk ahb_max_clk = {
	.name = "max_clk",
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static int _clk_emi_core_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, stat;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);
	if (emi_intr_clk.usecount == 0)
		emi_intr_clk.enable(&emi_intr_clk);

	reg = __raw_readl(MXC_CCM_CBCDR6);
	reg &= ~MXC_CCM_CBCDR6_EMI_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR6_EMI_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR6);

	/* Set the Load-dividers bit in CCM */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_LOAD_DIVIDERS;
	__raw_writel(reg, MXC_CCM_CCDR);

	do {
		stat = __raw_readl(MXC_CCM_CCDR) & MXC_CCM_CCDR_LOAD_DIVIDERS;
	} while (stat);

	clk->rate = rate;

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);
	if (emi_intr_clk.usecount == 0)
		emi_intr_clk.disable(&emi_intr_clk);

	return 0;
}

static void _clk_emi_core_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR6);
	div = ((reg & MXC_CCM_CBCDR6_EMI_PODF_MASK) >>
	       MXC_CCM_CBCDR6_EMI_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static int _clk_emi_core_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	u32 stat;

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);
	if (emi_intr_clk.usecount == 0)
		emi_intr_clk.enable(&emi_intr_clk);

	if ((ipu_clk[0].parent == &emi_core_clk) &&
					(ipu_clk[0].usecount == 0))
		ipu_clk[0].enable(&ipu_clk[0]);

	reg = __raw_readl(MXC_CCM_CBCDR6);
	if (parent == &ahb_clk) {
		reg |= MXC_CCM_CBCDR6_EMI_CLK_SEL;
	} else if (parent == &main_bus_clk) {
		reg &= ~MXC_CCM_CBCDR6_EMI_CLK_SEL;
	} else {
		BUG();
	}
	__raw_writel(reg, MXC_CCM_CBCDR6);

	do {
		stat = __raw_readl(MXC_CCM_CCDR) & MXC_CCM_CCDR_LOAD_DIVIDERS;
	} while (stat);

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);
	if (emi_intr_clk.usecount == 0)
		emi_intr_clk.disable(&emi_intr_clk);
	if ((ipu_clk[0].parent == &emi_core_clk) &&
					(ipu_clk[0].usecount == 0))
		ipu_clk[0].disable(&ipu_clk[0]);

	return 0;
}

static unsigned long _clk_emi_core_round_rate(struct clk *clk,
					      unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return clk->parent->rate / div;
}

static struct clk emi_core_clk = {
	.name = "emi_core_clk",
	.set_parent = _clk_emi_core_set_parent,
	.recalc = _clk_emi_core_recalc,
	.set_rate = _clk_emi_core_set_rate,
	.round_rate = _clk_emi_core_round_rate,
	.flags = RATE_PROPAGATES,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG11_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk ahbmux1_clk = {
	.name = "ahbmux1_clk",
	.id = 0,
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG4_OFFSET,
	.disable = _clk_disable_inwait,
};

static struct clk ahbmux2_clk = {
	.name = "ahbmux2_clk",
	.id = 0,
	.parent = &emi_core_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG7_OFFSET,
	.disable = _clk_disable_inwait,
};

static struct clk emi_fast_clk = {
	.name = "emi_fast_clk",
	.parent = &emi_core_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG12_OFFSET,
	.disable = _clk_disable_inwait,
};

static struct clk emi_slow_clk = {
	.name = "emi_slow_clk",
	.parent = &emi_core_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG13_OFFSET,
	.disable = _clk_disable_inwait,
};

static int _clk_emi_intr_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, stat;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 4))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR7);
	reg &= ~MXC_CCM_CBCDR7_IPG_INIT_MEM_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR7_IPG_INT_MEM_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR7);

	/* Set the Load-dividers bit in CCM */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_LOAD_DIVIDERS;
	__raw_writel(reg, MXC_CCM_CCDR);

	do {
		stat = __raw_readl(MXC_CCM_CCDR) & MXC_CCM_CCDR_LOAD_DIVIDERS;
	} while (stat);

	clk->rate = rate;

	return 0;
}

static void _clk_emi_intr_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR7);
	div = ((reg & MXC_CCM_CBCDR7_IPG_INIT_MEM_PODF_MASK) >>
	       MXC_CCM_CBCDR7_IPG_INT_MEM_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static unsigned long _clk_emi_intr_round_rate(struct clk *clk,
					      unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;
	if (div > 4)
		div = 4;
	else if (div == 0)
		div++;
	return clk->parent->rate / div;
}

static struct clk emi_intr_clk = {
	.name = "emi_intr_clk",
	.parent = &emi_core_clk,
	.secondary = &ahbmux2_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG14_OFFSET,
	.disable = _clk_disable_inwait,
	.recalc = _clk_emi_intr_recalc,
	.set_rate = _clk_emi_intr_set_rate,
	.round_rate = _clk_emi_intr_round_rate,
};

static void _clk_ipg_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR2);
	div = ((reg & MXC_CCM_CBCDR2_IPG_PODF_MASK) >>
	       MXC_CCM_CBCDR2_IPG_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static struct clk ipg_clk = {
	.name = "ipg_clk",
	.parent = &ahb_clk,
	.recalc = _clk_ipg_recalc,
	.flags = RATE_PROPAGATES,
};

static void _clk_ipg_per_recalc(struct clk *clk)
{
	u32 reg, prediv1, prediv2, podf;

	if (clk->parent == &main_bus_clk || clk->parent == &lp_apm_clk) {
		/* the main_bus_clk is the one before the DVFS engine */
		reg = __raw_readl(MXC_CCM_CBCDR2);
		prediv1 = ((reg & MXC_CCM_CBCDR2_PERCLK_PRED1_MASK) >>
			   MXC_CCM_CBCDR2_PERCLK_PRED1_OFFSET) + 1;
		prediv2 = ((reg & MXC_CCM_CBCDR2_PERCLK_PRED2_MASK) >>
			   MXC_CCM_CBCDR2_PERCLK_PRED2_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CBCDR2_PERCLK_PODF_MASK) >>
			MXC_CCM_CBCDR2_PERCLK_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / (prediv1 * prediv2 * podf);
	} else if (clk->parent == &ipg_clk) {
		clk->rate = ipg_clk.rate;
	} else {
		BUG();
	}
}

static int _clk_ipg_per_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	mux = _get_mux(parent, &main_bus_clk, &lp_apm_clk, &ipg_clk, NULL);
	if (mux == 2) {
		reg |= MXC_CCM_CSCMR1_PERCLK_IPG_CLK_SEL;
	} else {
		reg &= ~MXC_CCM_CSCMR1_PERCLK_IPG_CLK_SEL;
		if (mux == 0) {
			reg &= ~MXC_CCM_CSCMR1_PERCLK_LP_APM_CLK_SEL;
		} else {
			reg |= MXC_CCM_CSCMR1_PERCLK_LP_APM_CLK_SEL;
		}
	}
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ipg_perclk = {
	.name = "ipg_perclk",
	.parent = &ipg_clk,
	.recalc = _clk_ipg_per_recalc,
	.set_parent = _clk_ipg_per_set_parent,
	.flags = RATE_PROPAGATES,
};

static struct clk aips_tz1_clk = {
	.name = "aips_tz1_clk",
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk aips_tz2_clk = {
	.name = "aips_tz2_clk",
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk gpc_dvfs_clk = {
	.name = "gpc_dvfs_clk",
	.parent = &aips_tz2_clk,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG15_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk sdma_clk[] = {
	{
	 .name = "sdma_ahb_clk",
	 .parent = &ahb_clk,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGR5_CG0_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "sdma_ipg_clk",
	 .parent = &ipg_clk,
#ifdef CONFIG_SDMA_IRAM
	 .secondary = &emi_intr_clk,
#endif
	 },
};

static int _clk_tve_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);

	if (parent == &pll3_sw_clk) {
		reg &= ~(MXC_CCM_CSCMR1_TVE_CLK_SEL);
	} else if (parent == &osc_clk) {
		reg |= MXC_CCM_CSCMR1_TVE_CLK_SEL;
		reg &= MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL;
	} else if (parent == &ckih_clk) {
		reg |= MXC_CCM_CSCMR1_TVE_CLK_SEL;
		reg |= MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL;
	} else {
		BUG();
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static void _clk_tve_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & MXC_CCM_CSCMR1_TVE_CLK_SEL) == 0) {
		reg = __raw_readl(MXC_CCM_CDCDR) &
		    MXC_CCM_CDCDR_TVE_CLK_PRED_MASK;
		div = (reg >> MXC_CCM_CDCDR_TVE_CLK_PRED_OFFSET) + 1;
		clk->rate = clk->parent->rate / div;
	} else {
		clk->rate = clk->parent->rate;
	}
}

static unsigned long _clk_tve_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (reg & MXC_CCM_CSCMR1_TVE_CLK_SEL) {
		return -EINVAL;
	}

	div = clk->parent->rate / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return clk->parent->rate / div;
}

static int _clk_tve_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (reg & MXC_CCM_CSCMR1_TVE_CLK_SEL) {
		return -EINVAL;
	}

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8)) {
		return -EINVAL;
	}

	div--;
	reg = __raw_readl(MXC_CCM_CDCDR) & ~MXC_CCM_CDCDR_TVE_CLK_PRED_MASK;
	reg |= div << MXC_CCM_CDCDR_TVE_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CDCDR);
	clk->rate = rate;
	return 0;
}

static struct clk tve_clk = {
	.name = "tve_clk",
	.parent = &pll3_sw_clk,
	.secondary = &aips_tz1_clk,
	.set_parent = _clk_tve_set_parent,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG10_OFFSET,
	.recalc = _clk_tve_recalc,
	.round_rate = _clk_tve_round_rate,
	.set_rate = _clk_tve_set_rate,
	.enable = _clk_enable,
	.disable = _clk_disable,
	 .flags = RATE_PROPAGATES,
};

static struct clk spba_clk = {
	.name = "spba_clk",
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG1_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static void _clk_uart_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_UART_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_UART_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_uart_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_UART_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_UART_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static int _clk_uart_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, post_div = 1;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 64) || (div == 1))
		return -EINVAL;

	if (div > 8) {
		int i = 1;
		while ((div / (2 * i)) > 8)
			i++;
		post_div = i * 2;
		div = div / post_div;
	}

	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_UART_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CSCDR1_UART_CLK_PRED_MASK;
	reg |= (div - 1) << MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET |
	    (post_div - 1) << MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);
	clk->rate = rate;

	return 0;
}

static unsigned long _clk_uart_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;
	if (div > 64)
		div = 64;
	else if (div == 0)
		div++;
	return clk->parent->rate / div;
}

static struct clk uart_main_clk = {
	.name = "uart_main_clk",
	.parent = &pll2_sw_clk,
	.secondary = &emi_fast_clk,
	.recalc = _clk_uart_recalc,
	.set_parent = _clk_uart_set_parent,
	.set_rate = _clk_uart_set_rate,
	.round_rate = _clk_uart_round_rate,
	.flags = RATE_PROPAGATES,
};

static struct clk uart1_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 0,
	 .parent = &uart_main_clk,
	 .secondary = &uart1_clk[1],
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG5_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &aips_tz2_clk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG4_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart2_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 1,
	 .parent = &uart_main_clk,
	 .secondary = &uart2_clk[1],
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG7_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &aips_tz2_clk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG6_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart3_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 2,
	 .parent = &uart_main_clk,
	 .secondary = &uart3_clk[1],
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk gpt_clk[] = {
	{
	 .name = "gpt_clk",
	 .id = 0,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &gpt_clk[1],
	 },
	{
	 .name = "gpt_ipg_clk",
	 .parent = &ipg_clk,
	 .id = 0,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG7_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "gpt_32k_clk",
	 .id = 0,
	 .parent = &ckil_clk,
	 },
};

static struct clk i2c_clk[] = {
	{
	 .name = "i2c_clk",
	 .id = 0,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG14_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "i2c_clk",
	 .id = 1,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG15_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "i2c_clk",
	 .id = 2,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG0_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static void _clk_cspi_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR2);
	prediv = ((reg & MXC_CCM_CSCDR2_CSPI_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR2_CSPI_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CSCDR2_CSPI_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR2_CSPI_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_cspi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_CSPI_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_CSPI_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static int _clk_cspi_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, post_div = 1;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 512) || (div == 1))
		return -EINVAL;

	if (div > 8) {
		int i = 1;
		while ((div / (2 * i)) > 8)
			i++;
		post_div = i * 2;
		div = div / post_div;
	}

	reg = __raw_readl(MXC_CCM_CSCDR2);
	reg &= ~MXC_CCM_CSCDR2_CSPI_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CSCDR2_CSPI_CLK_PRED_MASK;
	reg |= (div - 1) << MXC_CCM_CSCDR2_CSPI_CLK_PRED_OFFSET;
	reg |= (post_div - 1) << MXC_CCM_CSCDR2_CSPI_CLK_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR2);
	clk->rate = rate;

	return 0;
}

static unsigned long _clk_cspi_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;
	if (div > 512)
		div = 8;
	else if (div == 1)
		div = 2;
	else if (div == 0)
		div++;
	return clk->parent->rate / div;
}

static struct clk cspi_main_clk = {
	.name = "cspi_main_clk",
	.parent = &pll3_sw_clk,
	.recalc = _clk_cspi_recalc,
	.set_parent = _clk_cspi_set_parent,
	.set_rate = _clk_cspi_set_rate,
	.round_rate = _clk_cspi_round_rate,
	.flags = RATE_PROPAGATES,
};

static struct clk cspi1_clk[] = {
	{
	 .name = "cspi_clk",
	 .id = 0,
	 .parent = &cspi_main_clk,
	 .secondary = &cspi1_clk[1],
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "cspi_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &aips_tz2_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG7_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk cspi2_clk[] = {
	{
	 .name = "cspi_clk",
	 .id = 1,
	 .parent = &cspi_main_clk,
	 .secondary = &cspi2_clk[1],
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "cspi_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk cspi3_clk[] = {
	{
	 .name = "cspi_clk",
	 .id = 2,
	 .parent = &cspi_main_clk,
	 .secondary = &cspi3_clk[1],
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG12_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "cspi_ipg_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .secondary = &aips_tz2_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG11_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static int _clk_ssi_lp_apm_set_parent(struct clk *clk,
						struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);

	if (parent == &ckih_clk) {
		reg &= ~MXC_CCM_CSCMR1_SSI_APM_CLK_SEL;
	} else if (parent == &lp_apm_clk) {
		reg |= MXC_CCM_CSCMR1_SSI_APM_CLK_SEL;
	} else {
		BUG();
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static struct clk ssi_lp_apm_clk = {
	.name = "ssi_lp_apm_clk",
	.parent = &ckih_clk,
	.set_parent = _clk_ssi_lp_apm_set_parent,
};

static void _clk_ssi1_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CS1CDR);
	prediv = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK) >>
		  MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK) >>
		MXC_CCM_CS1CDR_SSI1_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}
static int _clk_ssi1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk,
		       &pll3_sw_clk, &ssi_lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_SSI1_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_SSI1_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi1_clk[] = {
	{
	 .name = "ssi_clk",
	 .id = 0,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_ssi1_set_parent,
	 .secondary = &ssi1_clk[1],
	 .recalc = _clk_ssi1_recalc,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &ssi1_clk[2],
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG7_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_dep_clk",
	 .id = 0,
	 .parent = &aips_tz2_clk,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	 .secondary = &emi_intr_clk,
#else
	 .secondary = &emi_fast_clk,
#endif
	 },
};

static void _clk_ssi2_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CS2CDR);
	prediv = ((reg & MXC_CCM_CS2CDR_SSI2_CLK_PRED_MASK) >>
		  MXC_CCM_CS2CDR_SSI2_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CS2CDR_SSI2_CLK_PODF_MASK) >>
		MXC_CCM_CS2CDR_SSI2_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_ssi2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk,
		       &pll3_sw_clk, &ssi_lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_SSI2_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_SSI2_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi2_clk[] = {
	{
	 .name = "ssi_clk",
	 .id = 1,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_ssi2_set_parent,
	 .secondary = &ssi2_clk[1],
	 .recalc = _clk_ssi2_recalc,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &ssi2_clk[2],
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_dep_clk",
	 .id = 1,
	 .parent = &spba_clk,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	 .secondary = &emi_intr_clk,
#else
	 .secondary = &emi_fast_clk,
#endif
	 },
};

static void _clk_ssi_ext1_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	clk->rate = clk->parent->rate;
	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & MXC_CCM_CSCMR1_SSI_EXT1_COM_CLK_SEL) == 0) {
		reg = __raw_readl(MXC_CCM_CSECDR1);
		prediv = ((reg & MXC_CCM_CSECDR1_SSI_EXT1_CLK_PRED_MASK) >>
			  MXC_CCM_CSECDR1_SSI_EXT1_CLK_PRED_OFFSET) + 1;
		if (prediv == 1)
			BUG();
		podf = ((reg & MXC_CCM_CSECDR1_SSI_EXT1_CLK_PODF_MASK) >>
			MXC_CCM_CSECDR1_SSI_EXT1_CLK_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / (prediv * podf);
	}
}

static int _clk_ssi_ext1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &ssi1_clk[0]) {
		reg |= MXC_CCM_CSCMR1_SSI_EXT1_COM_CLK_SEL;
	} else {
		reg &= ~MXC_CCM_CSCMR1_SSI_EXT1_COM_CLK_SEL;
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &ssi_lp_apm_clk);
		reg = (reg & ~MXC_CCM_CSCMR1_SSI_EXT1_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR1_SSI_EXT1_CLK_SEL_OFFSET);
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi_ext1_clk = {
	.name = "ssi_ext1_clk",
	.parent = &pll3_sw_clk,
	.set_parent = _clk_ssi_ext1_set_parent,
	.recalc = _clk_ssi_ext1_recalc,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGR3_CG11_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static void _clk_ssi_ext2_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	clk->rate = clk->parent->rate;
	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & MXC_CCM_CSCMR1_SSI_EXT2_COM_CLK_SEL) == 0) {
		reg = __raw_readl(MXC_CCM_CSECDR2);
		prediv = ((reg & MXC_CCM_CSECDR2_SSI_EXT2_CLK_PRED_MASK) >>
			  MXC_CCM_CSECDR2_SSI_EXT2_CLK_PRED_OFFSET) + 1;
		if (prediv == 1)
			BUG();
		podf = ((reg & MXC_CCM_CSECDR2_SSI_EXT2_CLK_PODF_MASK) >>
			MXC_CCM_CSECDR2_SSI_EXT2_CLK_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / (prediv * podf);
	}
}

static int _clk_ssi_ext2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &ssi2_clk[0]) {
		reg |= MXC_CCM_CSCMR1_SSI_EXT2_COM_CLK_SEL;
	} else {
		reg &= ~MXC_CCM_CSCMR1_SSI_EXT2_COM_CLK_SEL;
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &ssi_lp_apm_clk);
		reg = (reg & ~MXC_CCM_CSCMR1_SSI_EXT2_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR1_SSI_EXT2_CLK_SEL_OFFSET);
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi_ext2_clk = {
	.name = "ssi_ext2_clk",
	.parent = &pll3_sw_clk,
	.set_parent = _clk_ssi_ext2_set_parent,
	.recalc = _clk_ssi_ext2_recalc,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGR3_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk iim_clk = {
	.name = "iim_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG15_OFFSET,
	.disable = _clk_disable,
};

static struct clk tmax1_clk = {
	.name = "tmax1_clk",
	.id = 0,
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGR1_CG0_OFFSET,
	.disable = _clk_disable,
};

static struct clk tmax2_clk = {
	.name = "tmax2_clk",
	.id = 0,
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGR1_CG1_OFFSET,
	.disable = _clk_disable,
};

static void _clk_usboh2_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_USBOH2_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_USBOH2_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CSCDR1_USBOH2_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_USBOH2_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_usboh2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_USBOH2_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_USBOH2_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

/*
 * This is USB core clock.
 ** need access DDR/iram, TMAX
 */
static struct clk usb_core_clk[] = {
	{
	 .name = "usb_ahb_clk",
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG11_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &usb_core_clk[1],
	 },
	{
	 .name = "usb_tmax_clk",
	 .parent = &tmax1_clk,
	 .secondary = &usb_core_clk[2],
	 },
	{
	 .name = "usb_ddr_clk",
	 .parent = &emi_fast_clk,
#if defined CONFIG_USB_STATIC_IRAM_PPH || defined CONFIG_USB_STATIC_IRAM
	.secondary = &usb_core_clk[3],
#endif
	 },
	/* iram patch, need access internal ram */
	{
	 .name = "usb_iram_clk",
	 .parent = &emi_intr_clk,
	 },
};

/* used for connecting external PHY */
static struct clk usboh2_clk = {
	.name = "usboh2_clk",
	.parent = &pll3_sw_clk,
	.set_parent = _clk_usboh2_set_parent,
	.recalc = _clk_usboh2_recalc,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGR2_CG12_OFFSET,
	.disable = _clk_disable,
};

static void _clk_usb_phy_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	if (clk->parent == &pll3_sw_clk) {
		reg = __raw_readl(MXC_CCM_CDCDR);
		prediv = ((reg & MXC_CCM_CDCDR_USB_PHY_PRED_MASK) >>
			  MXC_CCM_CDCDR_USB_PHY_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CDCDR_USB_PHY_PODF_MASK) >>
			MXC_CCM_CDCDR_USB_PHY_PODF_OFFSET) + 1;

		clk->rate = clk->parent->rate / (prediv * podf);
	} else
		clk->rate = clk->parent->rate;
}

static int _clk_usb_phy_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &osc_clk) {
		reg &= ~MXC_CCM_CSCMR1_USB_PHY_CLK_SEL;
	} else if (parent == &pll3_sw_clk) {
		reg |= MXC_CCM_CSCMR1_USB_PHY_CLK_SEL;
	} else {
		BUG();
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static struct clk usb_phy_clk = {
	.name = "usb_phy_clk",
	.parent = &osc_clk,
	.set_parent = _clk_usb_phy_set_parent,
	.recalc = _clk_usb_phy_recalc,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG6_OFFSET,
	.disable = _clk_disable,
};

static struct clk esdhc_dep_clks = {
	 .name = "sd_dep_clk",
	 .parent = &spba_clk,
	 .secondary = &emi_fast_clk,
};


static void _clk_esdhc1_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_esdhc1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) &
	    ~MXC_CCM_CSCMR1_ESDHC1_MSHC1_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_ESDHC1_MSHC1_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk esdhc1_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 0,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_esdhc1_set_parent,
	 .recalc = _clk_esdhc1_recalc,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG14_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc1_clk[1],
	 },
	{
	 .name = "esdhc_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG13_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc1_clk[2],
	 },
	{
	.name = "esdhc1_sec_clk",
	.parent = &tmax2_clk,
	.secondary = &esdhc_dep_clks,
	},
};

static void _clk_esdhc2_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_esdhc2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) &
	    ~MXC_CCM_CSCMR1_ESDHC2_MSHC2_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_ESDHC2_MSHC2_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk esdhc2_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 1,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_esdhc2_set_parent,
	 .recalc = _clk_esdhc2_recalc,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG0_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc2_clk[1],
	 },
	{
	 .name = "esdhc_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG15_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc2_clk[2],
	 },
	{
	.name = "esdhc2_sec_clk",
	.parent = &ahb_max_clk,
	.secondary = &esdhc_dep_clks,
	},
};

static int _clk_esdhc3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &esdhc1_clk[0]) {
		reg &= ~MXC_CCM_CSCMR1_ESDHC3_CLK_SEL;
	} else if (parent == &esdhc2_clk[0]) {
		reg |= MXC_CCM_CSCMR1_ESDHC3_CLK_SEL;
	} else {
		BUG();
	}
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk esdhc3_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 2,
	 .parent = &esdhc1_clk[0],
	 .set_parent = _clk_esdhc3_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG2_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc3_clk[1],
	 },
	{
	 .name = "esdhc_ipg_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG1_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc3_clk[2],
	 },
	{
	.name = "esdhc3_sec_clk",
	.parent = &ahb_max_clk,
	.secondary = &esdhc_dep_clks,
	},

};

static void _clk_nfc_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR7);
	div = ((reg & MXC_CCM_CBCDR7_NFC_PODF_MASK) >>
	       MXC_CCM_CBCDR7_NFC_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static int _clk_nfc_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, stat;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);
	if (emi_intr_clk.usecount == 0)
		emi_intr_clk.enable(&emi_intr_clk);

	reg = __raw_readl(MXC_CCM_CBCDR7);
	reg &= ~MXC_CCM_CBCDR7_NFC_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR7_NFC_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR7);

	/* Set the Load-dividers bit in CCM */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_LOAD_DIVIDERS;
	__raw_writel(reg, MXC_CCM_CCDR);

	do {
		stat = __raw_readl(MXC_CCM_CCDR) & MXC_CCM_CCDR_LOAD_DIVIDERS;
	} while (stat);
	clk->rate = rate;

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);
	if (emi_intr_clk.usecount == 0)
		emi_intr_clk.disable(&emi_intr_clk);

	return 0;
}

static unsigned long _clk_nfc_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return clk->parent->rate / div;
}

static struct clk nfc_clk = {
	.name = "nfc_clk",
	.parent = &emi_core_clk,
	.secondary = &emi_slow_clk,
	.recalc = _clk_nfc_recalc,
	.set_rate = _clk_nfc_set_rate,
	.round_rate = _clk_nfc_round_rate,
};

static int _clk_spdif_xtal_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &osc_clk) {
		reg &= ~MXC_CCM_CSCMR1_SPDIF_CLK_SEL;
	} else if (parent == &ckih_clk) {
		reg |= MXC_CCM_CSCMR1_SPDIF_CLK_SEL;
	} else {
		BUG();
	}
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk spdif_xtal_clk = {
	.name = "spdif_xtal_clk",
	.parent = &osc_clk,
	.set_parent = _clk_spdif_xtal_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGR1_CG10_OFFSET,
	.disable = _clk_disable,
};

static int _clk_spdif0_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	reg |= MXC_CCM_CSCMR2_SPDIF0_COM;
	if (parent != &ssi1_clk[0]) {
		reg &= ~MXC_CCM_CSCMR2_SPDIF0_COM;
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &spdif_xtal_clk);
		reg = (reg & ~MXC_CCM_CSCMR2_SPDIF0_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR2_SPDIF0_CLK_SEL_OFFSET);
	}
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static void _clk_spdif0_recalc(struct clk *clk)
{
	u32 reg, pred, podf;

	if (clk->parent == &ssi1_clk[0]) {
		clk->rate = clk->parent->rate;
	} else {
		reg = __raw_readl(MXC_CCM_CDCDR);
		pred = ((reg & MXC_CCM_CDCDR_SPDIF0_CLK_PRED_MASK) >>
			MXC_CCM_CDCDR_SPDIF0_CLK_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CDCDR_SPDIF0_CLK_PODF_MASK) >>
			MXC_CCM_CDCDR_SPDIF0_CLK_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / (pred * podf);
	}
}

static struct clk spdif0_clk[] = {
	{
	 .name = "spdif_clk",
	 .id = 0,
	 .parent = &pll3_sw_clk,
	 .secondary = &spdif0_clk[1],
	 .set_parent = _clk_spdif0_set_parent,
	 .recalc = _clk_spdif0_recalc,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG11_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .name = "spdif_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG10_OFFSET,
	 .disable = _clk_disable,
	 },
};

static int _clk_spdif1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	reg |= MXC_CCM_CSCMR2_SPDIF1_COM;
	if (parent != &ssi2_clk[0]) {
		reg &= ~MXC_CCM_CSCMR2_SPDIF1_COM;
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &spdif_xtal_clk);
		reg = (reg & ~MXC_CCM_CSCMR2_SPDIF1_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR2_SPDIF1_CLK_SEL_OFFSET);
	}
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static void _clk_spdif1_recalc(struct clk *clk)
{
	u32 reg, pred, podf;

	if (clk->parent == &ssi2_clk[0]) {
		clk->rate = clk->parent->rate;
	} else {
		reg = __raw_readl(MXC_CCM_CDCDR);
		pred = ((reg & MXC_CCM_CDCDR_SPDIF1_CLK_PRED_MASK) >>
			MXC_CCM_CDCDR_SPDIF1_CLK_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CDCDR_SPDIF1_CLK_PODF_MASK) >>
			MXC_CCM_CDCDR_SPDIF1_CLK_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / (pred * podf);
	}
}

static struct clk spdif1_clk[] = {
	{
	 .name = "spdif_clk",
	 .id = 1,
	 .parent = &pll3_sw_clk,
	 .secondary = &spdif1_clk[1],
	 .set_parent = _clk_spdif1_set_parent,
	 .recalc = _clk_spdif1_recalc,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG12_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .name = "spdif_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG10_OFFSET,
	 .disable = _clk_disable,
	 },
};

static int _clk_ipu_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);
	/* Handshake with IPU when certain clock rates are changed. */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg &= ~MXC_CCM_CCDR_IPU_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	/* Handshake with IPU when LPM is entered as its enabled. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg &= ~MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static void _clk_ipu_disable(struct clk *clk)
{
	u32 reg;
	_clk_disable(clk);

	/* No handshake with IPU as its not enabled. */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_IPU_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	/* No handshake with IPU when LPM is entered as its not enabled. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}

static int _clk_ipu_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CAMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &axi_c_clk,
		       &emi_core_clk);
	reg = (reg & ~MXC_CCM_CAMR_IPU_HSP_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CAMR_IPU_HSP_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CAMR);

	return 0;
}

static struct clk ipu_clk[] = {
	{
	 .name = "ipu_clk",
	 .parent = &axi_a_clk,
	 .secondary = &ipu_clk[1],
	 .set_parent = _clk_ipu_set_parent,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG15_OFFSET,
	 .enable = _clk_ipu_enable,
	 .disable = _clk_ipu_disable,
	 .flags = CPU_FREQ_TRIG_UPDATE,
	 },
	{
	 .name = "ipu_sec_clk",
	 .parent = &emi_fast_clk,
	 .secondary = &ahbmux1_clk,
	 }
};

static int _clk_ipu_di_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	if (parent == &tve_clk) {
		mxc_iomux_set_gpr(MUX_IPUv3D_CAMP, false, 0);
	} else if (parent == &ckih_clk) {
		mxc_iomux_set_gpr(MUX_IPUv3D_CAMP, true, 0);
		reg = __raw_readl(MXC_CCM_CSCMR1);
		reg |= MXC_CCM_CSCMR1_DI_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CSCMR1);
	} else if (parent == &osc_clk) {
		mxc_iomux_set_gpr(MUX_IPUv3D_CAMP, true, 0);
		reg = __raw_readl(MXC_CCM_CSCMR1);
		reg &= ~MXC_CCM_CSCMR1_DI_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CSCMR1);
	} else {
		return -EINVAL;
	}

	return 0;
}

static void _clk_ipu_di_recalc(struct clk *clk)
{
	if (clk->parent == &tve_clk) {
		clk->rate = clk->parent->rate / 8;
	} else {
		clk->rate = clk->parent->rate;
	}
}

static struct clk ipu_di_clk = {
	.name = "ipu_di_clk",
	.parent = &tve_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG14_OFFSET,
	.recalc = _clk_ipu_di_recalc,
	.set_parent = _clk_ipu_di_set_parent,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static int _clk_ddr_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CAMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &axi_c_clk,
		       &emi_core_clk);
	reg = (reg & ~MXC_CCM_CAMR_DDR_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CAMR_DDR_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CAMR);

	return 0;
}

static struct clk ddr_clk = {
	.name = "ddr_clk",
	.parent = &axi_c_clk,
	.set_parent = _clk_ddr_set_parent,
};

static int _clk_arm_axi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CAMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &axi_c_clk,
		       &emi_core_clk);
	reg = (reg & ~MXC_CCM_CAMR_ARM_AXI_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CAMR_ARM_AXI_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CAMR);

	return 0;
}

static struct clk arm_axi_clk = {
	.name = "arm_axi_clk",
	.parent = &axi_a_clk,
	.set_parent = _clk_arm_axi_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG1_OFFSET,
	.disable = _clk_disable,
};

static int _clk_vpu_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CAMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &axi_c_clk,
		       &emi_core_clk);
	reg = (reg & ~MXC_CCM_CAMR_VPU_AXI_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CAMR_VPU_AXI_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CAMR);

	return 0;
}

static int _clk_vpu_core_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CAMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &axi_c_clk,
		       &emi_core_clk);
	reg = (reg & ~MXC_CCM_CAMR_VPU_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CAMR_VPU_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CAMR);

	return 0;
}

static struct clk vpu_clk[] = {
	{
	 .name = "vpu_clk",
	 .parent = &axi_a_clk,
	 .set_parent = _clk_vpu_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGR5_CG7_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &vpu_clk[1],
	 .flags = CPU_FREQ_TRIG_UPDATE,
	 },
	{
	 .name = "vpu_core_clk",
	 .parent = &axi_a_clk,
	 .secondary = &vpu_clk[2],
	 .set_parent = _clk_vpu_core_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGR5_CG6_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .name = "vpu_emi_clk",
	 .parent = &emi_fast_clk,
#ifdef CONFIG_MXC_VPU_IRAM
	 .secondary = &emi_intr_clk,
#endif
	 }
};

static int _clk_lpsr_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CLPCR);
	mux = _get_mux(parent, &ckil_clk, &fpm_clk, &fpm_div2_clk, NULL);
	reg = (reg & ~MXC_CCM_CLPCR_LPSR_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CLPCR_LPSR_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static struct clk lpsr_clk = {
	.name = "lpsr_clk",
	.parent = &ckil_clk,
	.set_parent = _clk_lpsr_set_parent,
};

static void _clk_pgc_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	div = (reg & MXC_CCM_CSCDR1_PGC_CLK_PODF_MASK) >>
	    MXC_CCM_CSCDR1_PGC_CLK_PODF_OFFSET;
	div = 1 >> div;
	clk->rate = clk->parent->rate / div;
}

static struct clk pgc_clk = {
	.name = "pgc_clk",
	.parent = &ipg_clk,
	.recalc = _clk_pgc_recalc,
};

/*usb OTG clock */
/*Notes: in mx37, usb clock get from UTMI PHY, always 60MHz*/

static struct clk usb_clk = {
	.name = "usb_clk",
	.rate = 60000000,
};

static struct clk rtc_clk = {
	.name = "rtc_clk",
	.parent = &ckil_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG13_OFFSET,
	.disable = _clk_disable,
};

static struct clk ata_clk = {
	.name = "ata_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGR3_CG14_OFFSET,
	.disable = _clk_disable,
};

static struct clk rng_clk = {
	.name = "rng_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGR3_CG5_OFFSET,
	.disable = _clk_disable,
};

static struct clk scc_clk = {
	.name = "scc_clk",
	.parent = &ipg_clk,
	.secondary = &emi_fast_clk,
};

static void cko1_recalc(struct clk *clk)
{
	unsigned long rate;
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= MXC_CCM_CCOSR_CKOL_DIV_MASK;
	reg = reg >> MXC_CCM_CCOSR_CKOL_DIV_OFFSET;
	rate = clk->parent->rate;
	clk->rate = rate / (reg + 1);
}

static int cko1_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg |= MXC_CCM_CCOSR_CKOL_EN;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}

static void cko1_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MXC_CCM_CCOSR_CKOL_EN;
	__raw_writel(reg, MXC_CCM_CCOSR);
}

static int cko1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;

	div = (clk->parent->rate/rate - 1) & 0x7;
	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MXC_CCM_CCOSR_CKOL_DIV_MASK;
	reg |= div << MXC_CCM_CCOSR_CKOL_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}

static unsigned long cko1_round_rate(struct clk *clk, unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;
	div = div < 1 ? 1 : div;
	div = div > 8 ? 8 : div;
	return clk->parent->rate / div;
}

static int cko1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 sel, reg;

	if (parent == &cpu_clk)
		sel = 0;
	else if (parent == &pll1_sw_clk)
		sel = 1;
	else if (parent == &pll2_sw_clk)
		sel = 2;
	else if (parent == &pll3_sw_clk)
		sel = 3;
	else if (parent == &emi_core_clk)
		sel = 4;
	else if (parent == &nfc_clk)
		sel = 6;
	else if (parent == &vpu_clk[1])
		sel = 7;
	else if (parent == &ipu_di_clk)
		sel = 8;
	else if (parent == &ahb_clk)
		sel = 11;
	else if (parent == &ipg_clk)
		sel = 12;
	else if (parent == &ipg_perclk)
		sel = 13;
	else
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MXC_CCM_CCOSR_CKOL_SEL_MASK;
	reg |= sel << MXC_CCM_CCOSR_CKOL_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}
static struct clk cko1_clk = {
	.name = "cko1_clk",
	.recalc = cko1_recalc,
	.enable = cko1_enable,
	.disable = cko1_disable,
	.set_rate = cko1_set_rate,
	.round_rate = cko1_round_rate,
	.set_parent = cko1_set_parent,
};

static struct clk *mxc_clks[] = {
	&osc_clk,
	&ckih_clk,
	&ckil_clk,
	&fpm_clk,
	&fpm_div2_clk,
	&pll1_main_clk,
	&pll1_sw_clk,
	&pll2_sw_clk,
	&pll3_sw_clk,
	&gpc_dvfs_clk,
	&lp_apm_clk,
	&cpu_clk,
	&periph_apm_clk,
	&main_bus_clk,
	&axi_a_clk,
	&axi_b_clk,
	&axi_c_clk,
	&ahb_clk,
	&ahb_max_clk,
	&aips_tz1_clk,
	&aips_tz2_clk,
	&ipg_clk,
	&ipg_perclk,
	&sdma_clk[0],
	&sdma_clk[1],
	&ipu_clk[0],
	&ipu_clk[1],
	&ipu_di_clk,
	&tve_clk,
	&uart_main_clk,
	&uart1_clk[0],
	&uart1_clk[1],
	&uart2_clk[0],
	&uart2_clk[1],
	&uart3_clk[0],
	&uart3_clk[1],
	&spba_clk,
	&i2c_clk[0],
	&i2c_clk[1],
	&i2c_clk[2],
	&gpt_clk[0],
	&gpt_clk[1],
	&gpt_clk[2],
	&cspi_main_clk,
	&cspi1_clk[0],
	&cspi1_clk[1],
	&cspi2_clk[0],
	&cspi2_clk[1],
	&cspi3_clk[0],
	&cspi3_clk[1],
	&ssi_lp_apm_clk,
	&ssi1_clk[0],
	&ssi1_clk[1],
	&ssi2_clk[0],
	&ssi2_clk[1],
	&ssi_ext1_clk,
	&ssi_ext2_clk,
	&iim_clk,
	&tmax1_clk,
	&tmax2_clk,
	&ahbmux1_clk,
	&ahbmux2_clk,
	&usb_core_clk[0],
	&usb_core_clk[1],
	&usb_core_clk[2],
	&usb_core_clk[3],
	&usboh2_clk,
	&usb_phy_clk,
	&usb_clk,
	&esdhc1_clk[0],
	&esdhc1_clk[1],
	&esdhc1_clk[2],
	&esdhc2_clk[0],
	&esdhc2_clk[1],
	&esdhc2_clk[2],
	&esdhc3_clk[0],
	&esdhc3_clk[1],
	&esdhc3_clk[2],
	&esdhc_dep_clks,
	&emi_core_clk,
	&emi_fast_clk,
	&emi_slow_clk,
	&emi_intr_clk,
	&nfc_clk,
	&spdif_xtal_clk,
	&spdif0_clk[0],
	&spdif0_clk[1],
	&spdif1_clk[0],
	&spdif1_clk[1],
	&ddr_clk,
	&arm_axi_clk,
	&vpu_clk[0],
	&vpu_clk[1],
	&vpu_clk[2],
	&lpsr_clk,
	&pgc_clk,
	&rtc_clk,
	&ata_clk,
	&rng_clk,
	&scc_clk,
	&cko1_clk,
};

static void clk_tree_init(void)
{
	u32 reg, dp_ctl;

	ipg_perclk.set_parent(&ipg_perclk, &lp_apm_clk);

	/*
	 *Initialise the IPG PER CLK dividers to 3. IPG_PER_CLK should be at
	 * 8MHz, its derived from lp_apm.
	 */
	reg = __raw_readl(MXC_CCM_CBCDR2);
	reg &= ~MXC_CCM_CBCDR2_PERCLK_PRED1_MASK;
	reg &= ~MXC_CCM_CBCDR2_PERCLK_PRED2_MASK;
	reg &= ~MXC_CCM_CBCDR2_PERCLK_PODF_MASK;
	reg |= (2 << MXC_CCM_CBCDR2_PERCLK_PRED1_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCDR2);

	/* set pll1_main_clk parent */
	pll1_main_clk.parent = &osc_clk;
	dp_ctl = __raw_readl(pll_base[0] + MXC_PLL_DP_CTL);
	if ((dp_ctl & MXC_PLL_DP_CTL_REF_CLK_SEL_MASK) == 0)
		pll1_main_clk.parent = &fpm_clk;
	/* set pll2_sw_clk parent */
	pll2_sw_clk.parent = &osc_clk;
	dp_ctl = __raw_readl(pll_base[1] + MXC_PLL_DP_CTL);
	if ((dp_ctl & MXC_PLL_DP_CTL_REF_CLK_SEL_MASK) == 0)
		pll2_sw_clk.parent = &fpm_clk;
	/* set pll3_clk parent */
	pll3_sw_clk.parent = &osc_clk;
	dp_ctl = __raw_readl(pll_base[2] + MXC_PLL_DP_CTL);
	if ((dp_ctl & MXC_PLL_DP_CTL_REF_CLK_SEL_MASK) == 0)
		pll3_sw_clk.parent = &fpm_clk;

	/* set emi_core_clk parent */
	emi_core_clk.parent = &main_bus_clk;
	reg = __raw_readl(MXC_CCM_CBCDR6);
	if ((reg & MXC_CCM_CBCDR6_EMI_CLK_SEL) != 0) {
		emi_core_clk.parent = &ahb_clk;
	}

	/* set ipg_perclk parent */
	ipg_perclk.parent = &lp_apm_clk;
	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & MXC_CCM_CSCMR1_PERCLK_IPG_CLK_SEL) != 0) {
		ipg_perclk.parent = &ipg_clk;
	} else {
		if ((reg & MXC_CCM_CSCMR1_PERCLK_LP_APM_CLK_SEL) == 0)
			ipg_perclk.parent = &main_bus_clk;
	}

	/* set DDR clock parent */
	reg = __raw_readl(MXC_CCM_CAMR) & MXC_CCM_CAMR_DDR_CLK_SEL_MASK;
	reg >>= MXC_CCM_CAMR_DDR_CLK_SEL_OFFSET;
	if (reg == 0) {
		ddr_clk.parent = &axi_a_clk;
	} else if (reg == 1) {
		ddr_clk.parent = &axi_b_clk;
	} else if (reg == 2) {
		ddr_clk.parent = &axi_c_clk;
	} else {
		ddr_clk.parent = &emi_core_clk;
	}
}

int __init mx37_clocks_init(unsigned long ckil, unsigned long osc, unsigned long ckih1, unsigned long ckih2)
{
	struct clk **clkp;
	u32 reg;
	int i;

	/* Turn off all possible clocks */
	if (mxc_jtag_enabled) {
		__raw_writel((1 << MXC_CCM_CCGR0_CG0_OFFSET) |
			     (1 << MXC_CCM_CCGR0_CG1_OFFSET) |
			     (1 << MXC_CCM_CCGR0_CG2_OFFSET) |
			     (1 << MXC_CCM_CCGR0_CG12_OFFSET) |
			     (1 << MXC_CCM_CCGR0_CG13_OFFSET) |
			     (1 << MXC_CCM_CCGR0_CG7_OFFSET) |
			     (1 << MXC_CCM_CCGR0_CG14_OFFSET), MXC_CCM_CCGR0);
	} else {
		__raw_writel((1 << MXC_CCM_CCGR0_CG0_OFFSET) |
			     (1 << MXC_CCM_CCGR0_CG1_OFFSET) |
			     (1 << MXC_CCM_CCGR0_CG12_OFFSET) |
			     (1 << MXC_CCM_CCGR0_CG13_OFFSET) |
			     (1 << MXC_CCM_CCGR0_CG7_OFFSET) |
			     (1 << MXC_CCM_CCGR0_CG14_OFFSET), MXC_CCM_CCGR0);
	}
	__raw_writel(0, MXC_CCM_CCGR1);

	/* TMAX clocks. */
	reg = __raw_readl(MXC_CCM_CCGR1);
	reg |= 1 << MXC_CCM_CCGR1_CG0_OFFSET;
	reg |= 1 << MXC_CCM_CCGR1_CG1_OFFSET;
	__raw_writel(reg, MXC_CCM_CCGR1);
	__raw_writel(0, MXC_CCM_CCGR2);
	__raw_writel(0, MXC_CCM_CCGR3);
	__raw_writel(0, MXC_CCM_CCGR4);
	/* Initialise the EMI clocks to be OFF when ARM is in WAIT mode. */
	__raw_writel((1 << MXC_CCM_CCGR5_CG4_OFFSET) |
		     (1 << MXC_CCM_CCGR5_CG12_OFFSET) |
		     (1 << MXC_CCM_CCGR5_CG13_OFFSET) |
		     (1 << MXC_CCM_CCGR5_CG14_OFFSET) |
		     MXC_CCM_CCGR5_CG11_MASK, MXC_CCM_CCGR5);

	ckil_clk.rate = ckil;
	ckih_clk.rate = ckih1;
	osc_clk.rate = osc;

	clk_tree_init();

	for (clkp = mxc_clks; clkp < mxc_clks + ARRAY_SIZE(mxc_clks); clkp++)
		clk_register(*clkp);

	reg = __raw_readl(MXC_CCM_CCSR);
	/*STEP_CLK	mxc_timer_init(&gpt_clk[0], IO_ADDRESS(GPT1_BASE_ADDR), MXC_INT_GPT);
 - make sure its source is lp_apm */
	reg &= ~MXC_CCM_CCSR_STEP_SEL_MASK;
	__raw_writel(reg, MXC_CCM_CCSR);

	/* This will propagate to all children and init all the clock rates */
	propagate_rate(&osc_clk);
	propagate_rate(&ckih_clk);
	propagate_rate(&ckil_clk);

	_clk_pll_disable(&pll3_sw_clk);

	clk_enable(&cpu_clk);
	clk_enable(&main_bus_clk);

	/* Move UART to run from pll2_sw_clk */
	clk_set_parent(&uart_main_clk, &pll2_sw_clk);

	/* Set the UART dividers to divide by 10, so the UART_CLK is 66.5MHz. */
	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_UART_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CSCDR1_UART_CLK_PRED_MASK;
	reg |= (4 << MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET) |
	    (1 << MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCDR1);

	/* move cspi to 24MHz */
	clk_set_parent(&cspi_main_clk, &lp_apm_clk);
	clk_set_rate(&cspi_main_clk, 12000000);

	/*move the spdif0 to spdif_xtal_ckl */
	clk_set_parent(&spdif0_clk[0], &spdif_xtal_clk);
	/*set the SPDIF dividers to 1 */
	reg = __raw_readl(MXC_CCM_CDCDR);
	reg &= ~MXC_CCM_CDCDR_SPDIF0_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CDCDR_SPDIF0_CLK_PRED_MASK;
	__raw_writel(reg, MXC_CCM_CDCDR);

	/* move the spdif1 to 24MHz */
	clk_set_parent(&spdif1_clk[0], &spdif_xtal_clk);
	/* set the spdif1 dividers to 1 */
	reg = __raw_readl(MXC_CCM_CDCDR);
	reg &= ~MXC_CCM_CDCDR_SPDIF1_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CDCDR_SPDIF1_CLK_PRED_MASK;
	__raw_writel(reg, MXC_CCM_CDCDR);

	/* Move SSI clocks to SSI_LP_APM clock */
	clk_set_parent(&ssi_lp_apm_clk, &lp_apm_clk);

	clk_set_parent(&ssi1_clk[0], &ssi_lp_apm_clk);
	/* set the SSI dividers to divide by 2 */
	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg &= ~MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK;
	reg |= 1 << MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CS1CDR);

	clk_set_parent(&ssi2_clk[0], &ssi_lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CS2CDR);
	reg &= ~MXC_CCM_CS2CDR_SSI2_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CS2CDR_SSI2_CLK_PRED_MASK;
	reg |= 1 << MXC_CCM_CS2CDR_SSI2_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CS2CDR);

	/* Change the SSI_EXT1_CLK to be sourced from SSI1_CLK_ROOT */
	clk_set_parent(&ssi_ext1_clk, &ssi1_clk[0]);
	clk_set_parent(&ssi_ext2_clk, &ssi2_clk[0]);

	propagate_rate(&ssi_lp_apm_clk);

	clk_set_parent(&arm_axi_clk, &emi_core_clk);

	clk_set_parent(&ipu_clk[0], &axi_a_clk);
	clk_set_parent(&vpu_clk[0], &axi_a_clk);
	clk_set_parent(&vpu_clk[1], &axi_a_clk);

	clk_set_parent(&emi_core_clk, &ahb_clk);
	clk_set_rate(&emi_core_clk, clk_round_rate(&emi_core_clk, 130000000));
	propagate_rate(&emi_core_clk);
	clk_set_rate(&emi_intr_clk, clk_round_rate(&emi_intr_clk, 66000000));
	/* Change the NFC clock rate to be 1:3 ratio with emi clock. */
	clk_set_rate(&nfc_clk, clk_round_rate(&nfc_clk,
			(clk_get_rate(&emi_slow_clk))/3));

	clk_set_parent(&usb_phy_clk, &osc_clk);

	clk_set_parent(&cko1_clk, &ipg_perclk);
	clk_set_rate(&cko1_clk, 8000000);
	/* Set the current working point. */
	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	for (i = 0; i < cpu_wp_nr; i++) {
		if (clk_get_rate(&cpu_clk) == cpu_wp_tbl[i].cpu_rate) {
			cpu_curr_wp = i;
			break;
		}
	}
	if (i > cpu_wp_nr)
		BUG();

	propagate_rate(&osc_clk);
	propagate_rate(&pll1_sw_clk);
	propagate_rate(&pll2_sw_clk);

#ifdef DVFS_SW_WORKAROUND
	clk_set_parent(&periph_apm_clk, &pll3_sw_clk);

	clk_set_parent(&main_bus_clk, &periph_apm_clk);
	clk_disable(&pll2_sw_clk);
	clk_set_rate(&pll2_sw_clk, 266000000);
	pll2_sw_clk.recalc(&pll2_sw_clk);
	clk_enable(&pll2_sw_clk);
	clk_set_parent(&main_bus_clk, &pll2_sw_clk);

	clk_set_rate(&ahb_clk, clk_round_rate(&ahb_clk, 130000000));
	clk_set_rate(&axi_b_clk, clk_round_rate(&axi_b_clk, 110000000));
	clk_set_rate(&axi_c_clk, clk_round_rate(&axi_c_clk, 166000000));
	clk_set_rate(&axi_a_clk, clk_round_rate(&axi_a_clk, 130000000));

	clk_set_parent(&emi_core_clk, &ahb_clk);
	clk_set_rate(&emi_core_clk, clk_round_rate(&emi_core_clk, 130000000));
	clk_set_rate(&emi_intr_clk, clk_round_rate(&emi_intr_clk, 66000000));

	clk_set_rate(&axi_c_clk, clk_round_rate(&axi_c_clk, 130000000));
	clk_set_parent(&ddr_clk, &axi_c_clk);
	/* Set the UART dividers to divide by 6, so the UART_CLK is 66.5MHz. */
	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_UART_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CSCDR1_UART_CLK_PRED_MASK;
	reg |= (3 << MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET) |
	    (0 << MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCDR1);
#endif

	mxc_timer_init(&gpt_clk[0], IO_ADDRESS(GPT1_BASE_ADDR), MXC_INT_GPT);

	return 0;
}

/*!
 * Setup cpu clock based on working point.
 * @param	wp	cpu freq working point
 * @return		0 on success or error code on failure.
 */
static int cpu_clk_set_wp(int wp)
{
	struct cpu_wp *p;
	u32 reg;
	u32 stat;

	if (wp == cpu_curr_wp)
		return 0;

	p = &cpu_wp_tbl[wp];

	if (!dvfs_core_is_active) {
		/* Change the ARM clock to requested frequency */
		/* First move the ARM clock to step clock */
		/* which is running at 24MHz. */

		/* Change the source of pll1_sw_clk to be the step_clk */
		reg = __raw_readl(MXC_CCM_CCSR);
		reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CCSR);

		/* Stop the PLL */
		reg = __raw_readl(MXC_DPLL1_BASE + MXC_PLL_DP_CTL);
		reg &= ~MXC_PLL_DP_CTL_UPEN;
		__raw_writel(reg, MXC_DPLL1_BASE + MXC_PLL_DP_CTL);

		/* PDF and MFI */
		reg = p->pdf | p->mfi << MXC_PLL_DP_OP_MFI_OFFSET;
		__raw_writel(reg, MXC_DPLL1_BASE + MXC_PLL_DP_OP);

		/* MFD */
		__raw_writel(p->mfd, MXC_DPLL1_BASE + MXC_PLL_DP_MFD);

		/* MFI */
		__raw_writel(p->mfn, MXC_DPLL1_BASE + MXC_PLL_DP_MFN);

		reg = __raw_readl(MXC_DPLL1_BASE + MXC_PLL_DP_CTL);
		reg |= MXC_PLL_DP_CTL_UPEN;
		/* Set the UPEN bits */
		__raw_writel(reg, MXC_DPLL1_BASE + MXC_PLL_DP_CTL);
		/* Forcefully restart the PLL */
		reg |= MXC_PLL_DP_CTL_RST;
		__raw_writel(reg, MXC_DPLL1_BASE + MXC_PLL_DP_CTL);

		/* Wait for the PLL to lock */
		do {
			stat = __raw_readl(MXC_DPLL1_BASE + MXC_PLL_DP_CTL) &
			       MXC_PLL_DP_CTL_LRF;
		} while (!stat);

		reg = __raw_readl(MXC_CCM_CCSR);
		/* Move the PLL1 back to the pll1_main_clk */
		reg &= ~MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CCSR);
	}
	cpu_curr_wp = wp;

	pll1_sw_clk.rate = cpu_wp_tbl[wp].cpu_rate;
	pll1_main_clk.rate = pll1_sw_clk.rate;
	cpu_clk.rate = pll1_sw_clk.rate;

#if defined(CONFIG_CPU_FREQ)
	cpufreq_trig_needed = 1;
#endif

	if (wp == 0)
		dptc_resume(DPTC_GP_ID);
	else
		dptc_suspend(DPTC_GP_ID);

	return 0;
}
