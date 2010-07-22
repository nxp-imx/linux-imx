/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <asm/div64.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/clock.h>
#include <mach/mxc_dvfs.h>
#include <mach/sdram_autogating.h>

#include "crm_regs.h"

static struct clk pll1_main_clk;
static struct clk pll1_sw_clk;
static struct clk pll2_sw_clk;
static struct clk pll3_sw_clk;
static struct clk apll_clk;
static struct clk pfd0_clk;
static struct clk pfd1_clk;
static struct clk pfd2_clk;
static struct clk pfd3_clk;
static struct clk pfd4_clk;
static struct clk pfd5_clk;
static struct clk pfd6_clk;
static struct clk pfd7_clk;
static struct clk lp_apm_clk;
static struct clk weim_clk;
static struct clk ddr_clk;
static struct clk axi_a_clk;
static struct clk axi_b_clk;
static struct clk gpu2d_clk;
static int cpu_curr_wp;
static struct cpu_wp *cpu_wp_tbl;

static void __iomem *pll1_base;
static void __iomem *pll2_base;
static void __iomem *pll3_base;
static void __iomem *apll_base;

extern int cpu_wp_nr;
extern int lp_high_freq;
extern int lp_med_freq;

#define SPIN_DELAY	1000000 /* in nanoseconds */

extern int mxc_jtag_enabled;
extern int uart_at_24;
extern int cpufreq_trig_needed;
extern int low_bus_freq_mode;

static int cpu_clk_set_wp(int wp);
extern void propagate_rate(struct clk *tclk);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);

static struct clk esdhc3_clk[];

static void __calc_pre_post_dividers(u32 div, u32 *pre, u32 *post)
{
	u32 min_pre, temp_pre, old_err, err;

	if (div >= 512) {
		*pre = 8;
		*post = 64;
	} else if (div >= 8) {
		min_pre = (div - 1) / 64 + 1;
		old_err = 8;
		for (temp_pre = 8; temp_pre >= min_pre; temp_pre--) {
			err = div % temp_pre;
			if (err == 0) {
				*pre = temp_pre;
				break;
			}
			err = temp_pre - err;
			if (err < old_err) {
				old_err = err;
				*pre = temp_pre;
			}
		}
		*post = (div + *pre - 1) / *pre;
	} else if (div < 8) {
		*pre = div;
		*post = 1;
	}
}

static int _clk_enable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg |= MXC_CCM_CCGR_CG_MASK << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	if (clk->flags & AHB_HIGH_SET_POINT)
		lp_high_freq++;
	else if (clk->flags & AHB_MED_SET_POINT)
		lp_med_freq++;

	return 0;
}

static int _clk_enable_inrun(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGR_CG_MASK << clk->enable_shift);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);
	return 0;
}

static void _clk_disable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGR_CG_MASK << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);

	if (clk->flags & AHB_HIGH_SET_POINT)
		lp_high_freq--;
	else if (clk->flags & AHB_MED_SET_POINT)
		lp_med_freq--;
}

static void _clk_disable_inwait(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGR_CG_MASK << clk->enable_shift);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);
}

static unsigned long _clk_round_rate_div(struct clk *clk,
						unsigned long rate,
						u32 max_div,
						u32 *new_div)
{
	u32 div;

	div = DIV_ROUND_UP(clk->parent->rate, rate);
	if (div > max_div)
		div = max_div;
	else if (div == 0)
		div++;
	if (new_div != NULL)
		*new_div = div;

	return clk->parent->rate / div;
}
/*
 * For the 4-to-1 muxed input clock
 */
static inline u32 _get_mux(struct clk *parent, struct clk *m0,
			   struct clk *m1, struct clk *m2, struct clk *m3)
{
	if (parent == m0)
		return 0;
	else if (parent == m1)
		return 1;
	else if (parent == m2)
		return 2;
	else if (parent == m3)
		return 3;
	else
		BUG();

	return 0;
}

/*
 * For the 4-to-1 muxed input clock
 */
static inline u32 _get_mux8(struct clk *parent, struct clk *m0, struct clk *m1,
			    struct clk *m2, struct clk *m3, struct clk *m4,
			    struct clk *m5, struct clk *m6, struct clk *m7)
{
	if (parent == m0)
		return 0;
	else if (parent == m1)
		return 1;
	else if (parent == m2)
		return 2;
	else if (parent == m3)
		return 3;
	else if (parent == m4)
		return 4;
	else if (parent == m5)
		return 5;
	else if (parent == m6)
		return 6;
	else if (parent == m7)
		return 7;
	else
		BUG();

	return 0;
}

static inline void __iomem *_get_pll_base(struct clk *pll)
{
	if (pll == &pll1_main_clk)
		return pll1_base;
	else if (pll == &pll2_sw_clk)
		return pll2_base;
	else if (pll == &pll3_sw_clk)
		return pll3_base;
	else
		BUG();

	return NULL;
}

static struct clk ckih_clk = {
	.name = "ckih",
	.flags = RATE_PROPAGATES,
};

static struct clk ckih2_clk = {
	.name = "ckih2",
	.flags = RATE_PROPAGATES,
};

static struct clk osc_clk = {
	.name = "osc",
	.flags = RATE_PROPAGATES,
};

static int apll_enable(struct clk *clk)
{
	__raw_writel(1, apll_base + MXC_ANADIG_MISC_SET);
	udelay(10);
	return 0;
}

static void apll_disable(struct clk *clk)
{
	__raw_writel(1, apll_base + MXC_ANADIG_MISC_CLR);
}

static struct clk apll_clk = {
	.name = "apll",
	.rate = 480000000,
	.enable = apll_enable,
	.disable = apll_disable,
	.flags = RATE_PROPAGATES,
};

static void pfd_recalc(struct clk *clk)
{
	u32 frac;
	u64 rate;
	frac = __raw_readl(apll_base +
		(int)clk->enable_reg) >> clk->enable_shift;
	frac &= MXC_ANADIG_PFD_FRAC_MASK;
	rate = (u64)clk->parent->rate * 18;
	do_div(rate, frac);
	clk->rate = rate;
}

static unsigned long pfd_round_rate(struct clk *clk, unsigned long rate)
{
	u32 frac;
	u64 tmp;
	tmp = (u64)clk->parent->rate * 18;
	do_div(tmp, rate);
	frac = tmp;
	frac = frac < 18 ? 18 : frac;
	frac = frac > 35 ? 35 : frac;
	do_div(tmp, frac);
	return tmp;
}

static int pfd_set_rate(struct clk *clk, unsigned long rate)
{
	u32 frac;
	u64 tmp;
	tmp = (u64)clk->parent->rate * 18;
	do_div(tmp, rate);
	frac = tmp;
	frac = frac < 18 ? 18 : frac;
	frac = frac > 35 ? 35 : frac;
	/* clear clk frac bits */
	__raw_writel(MXC_ANADIG_PFD_FRAC_MASK << clk->enable_shift,
			apll_base + (int)clk->enable_reg + 8);
	/* set clk frac bits */
	__raw_writel(frac << clk->enable_shift,
			apll_base + (int)clk->enable_reg + 4);

	tmp = (u64)clk->parent->rate * 18;
	do_div(tmp, frac);
	clk->rate = tmp;
	return 0;
}

static int pfd_enable(struct clk *clk)
{
	int index;
	index = _get_mux8(clk, &pfd0_clk, &pfd1_clk, &pfd2_clk, &pfd3_clk,
			&pfd4_clk, &pfd5_clk, &pfd6_clk, &pfd7_clk);
	__raw_writel(1 << index, apll_base + MXC_ANADIG_PLLCTRL_CLR);
	/* clear clk gate bit */
	__raw_writel((1 << (clk->enable_shift + 7)),
			apll_base + (int)clk->enable_reg + 8);
	return 0;
}

static void pfd_disable(struct clk *clk)
{
	int index;
	index = _get_mux8(clk, &pfd0_clk, &pfd1_clk, &pfd2_clk, &pfd3_clk,
			&pfd4_clk, &pfd5_clk, &pfd6_clk, &pfd7_clk);
	/* set clk gate bit */
	__raw_writel((1 << (clk->enable_shift + 7)),
			apll_base + (int)clk->enable_reg + 4);
	__raw_writel(1 << index, apll_base + MXC_ANADIG_PLLCTRL_SET);
}

static struct clk pfd0_clk = {
	.name = "pfd0",
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC0,
	.enable_shift = MXC_ANADIG_PFD0_FRAC_OFFSET,
	.recalc = pfd_recalc,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES,
};

static struct clk pfd1_clk = {
	.name = "pfd1",
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC0,
	.enable_shift = MXC_ANADIG_PFD1_FRAC_OFFSET,
	.recalc = pfd_recalc,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES,
};

static struct clk pfd2_clk = {
	.name = "pfd2",
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC0,
	.enable_shift = MXC_ANADIG_PFD2_FRAC_OFFSET,
	.recalc = pfd_recalc,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES,
};

static struct clk pfd3_clk = {
	.name = "pfd3",
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC0,
	.enable_shift = MXC_ANADIG_PFD3_FRAC_OFFSET,
	.recalc = pfd_recalc,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES,
};

static struct clk pfd4_clk = {
	.name = "pfd4",
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC1,
	.enable_shift = MXC_ANADIG_PFD4_FRAC_OFFSET,
	.recalc = pfd_recalc,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES,
};

static struct clk pfd5_clk = {
	.name = "pfd5",
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC1,
	.enable_shift = MXC_ANADIG_PFD5_FRAC_OFFSET,
	.recalc = pfd_recalc,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES,
};

static struct clk pfd6_clk = {
	.name = "pfd6",
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC1,
	.enable_shift = MXC_ANADIG_PFD6_FRAC_OFFSET,
	.recalc = pfd_recalc,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES,
};

static struct clk pfd7_clk = {
	.name = "pfd7",
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC1,
	.enable_shift = MXC_ANADIG_PFD7_FRAC_OFFSET,
	.recalc = pfd_recalc,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES,
};

static struct clk ckil_clk = {
	.name = "ckil",
	.flags = RATE_PROPAGATES,
};

static void _clk_pll_recalc(struct clk *clk)
{
	long mfi, mfn, mfd, pdf, ref_clk, mfn_abs;
	unsigned long dp_op, dp_mfd, dp_mfn, dp_ctl, pll_hfsm, dbl;
	void __iomem *pllbase;
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
	if (dbl != 0)
		ref_clk *= 2;

	ref_clk /= (pdf + 1);
	temp = (u64) ref_clk * mfn_abs;
	do_div(temp, mfd + 1);
	if (mfn < 0)
		temp = -temp;
	temp = (ref_clk * mfi) + temp;

	clk->rate = temp;
}

static int _clk_pll_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, reg1;
	void __iomem *pllbase;
	struct timespec nstimeofday;
	struct timespec curtime;

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
	/* If auto restart is disabled, restart the PLL and
	  * wait for it to lock.
	  */
	reg = __raw_readl(pllbase + MXC_PLL_DP_CTL);
	if (reg & MXC_PLL_DP_CTL_UPEN) {
		reg = __raw_readl(pllbase + MXC_PLL_DP_CONFIG);
		if (!(reg & MXC_PLL_DP_CONFIG_AREN)) {
			reg1 = __raw_readl(pllbase + MXC_PLL_DP_CTL);
			reg1 |= MXC_PLL_DP_CTL_RST;
			__raw_writel(reg1, pllbase + MXC_PLL_DP_CTL);
		}
		/* Wait for lock */
		getnstimeofday(&nstimeofday);
		while (!(__raw_readl(pllbase + MXC_PLL_DP_CTL)
					& MXC_PLL_DP_CTL_LRF)) {
			getnstimeofday(&curtime);
			if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
				panic("pll_set_rate: pll relock failed\n");
		}
	}
	clk->rate = rate;
	return 0;
}

static int _clk_pll_enable(struct clk *clk)
{
	u32 reg;
	void __iomem *pllbase;
	struct timespec nstimeofday;
	struct timespec curtime;

	pllbase = _get_pll_base(clk);
	reg = __raw_readl(pllbase + MXC_PLL_DP_CTL) | MXC_PLL_DP_CTL_UPEN;
	__raw_writel(reg, pllbase + MXC_PLL_DP_CTL);

	/* Wait for lock */
	getnstimeofday(&nstimeofday);
	while (!(__raw_readl(pllbase + MXC_PLL_DP_CTL) & MXC_PLL_DP_CTL_LRF)) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("pll relock failed\n");
	}
	return 0;
}

static void _clk_pll_disable(struct clk *clk)
{
	u32 reg;
	void __iomem *pllbase;

	pllbase = _get_pll_base(clk);
	reg = __raw_readl(pllbase + MXC_PLL_DP_CTL) & ~MXC_PLL_DP_CTL_UPEN;
	__raw_writel(reg, pllbase + MXC_PLL_DP_CTL);
}

static struct clk pll1_main_clk = {
	.name = "pll1_main_clk",
	.parent = &osc_clk,
	.recalc = _clk_pll_recalc,
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
		__raw_writel(reg, MXC_CCM_CCSR);
		/* Set the step_clk parent to be lp_apm, to save power. */
		mux = _get_mux(&lp_apm_clk, &lp_apm_clk, NULL, &pll2_sw_clk,
			       &pll3_sw_clk);
		reg = __raw_readl(MXC_CCM_CCSR);
		reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
		    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
	} else {
		if (parent == &lp_apm_clk) {
			reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
			reg = __raw_readl(MXC_CCM_CCSR);
			mux = _get_mux(parent, &lp_apm_clk, NULL, &pll2_sw_clk,
				       &pll3_sw_clk);
			reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
			    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
		} else {
			mux = _get_mux(parent, &lp_apm_clk, NULL, &pll2_sw_clk,
				       &pll3_sw_clk);
			reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
			    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
			__raw_writel(reg, MXC_CCM_CCSR);
			reg = __raw_readl(MXC_CCM_CCSR);
			reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;

		}
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

static int _clk_pll2_sw_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCSR);

	if (parent == &pll2_sw_clk) {
		reg &= ~MXC_CCM_CCSR_PLL2_SW_CLK_SEL;
	} else {
		reg = (reg & ~MXC_CCM_CCSR_PLL2_SW_CLK_SEL);
		reg |= MXC_CCM_CCSR_PLL2_SW_CLK_SEL;
	}
	__raw_writel(reg, MXC_CCM_CCSR);
	return 0;
}

/* same as pll2_main_clk. These two clocks should always be the same */
static struct clk pll2_sw_clk = {
	.name = "pll2",
	.parent = &osc_clk,
	.recalc = _clk_pll_recalc,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_rate = _clk_pll_set_rate,
	.set_parent = _clk_pll2_sw_set_parent,
	.flags = RATE_PROPAGATES,
};

/* same as pll3_main_clk. These two clocks should always be the same */
static struct clk pll3_sw_clk = {
	.name = "pll3",
	.parent = &osc_clk,
	.set_rate = _clk_pll_set_rate,
	.recalc = _clk_pll_recalc,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.flags = RATE_PROPAGATES,
};

static int _clk_lp_apm_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	if (parent == &osc_clk)
		reg = __raw_readl(MXC_CCM_CCSR) & ~MXC_CCM_CCSR_LP_APM_SEL;
	else if (parent == &apll_clk)
		reg = __raw_readl(MXC_CCM_CCSR) | MXC_CCM_CCSR_LP_APM_SEL;
	else
		return -EINVAL;

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
	if (i >= cpu_wp_nr)
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

/* TODO: Need to sync with GPC to determine if DVFS is in place so that
 * the DVFS_PODF divider can be applied in CDCR register.
 */
static void _clk_main_bus_recalc(struct clk *clk)
{
	u32 div = 0;

	if (dvfs_per_divider_active() || low_bus_freq_mode)
		div  = (__raw_readl(MXC_CCM_CDCR) & 0x3);
	clk->rate = clk->parent->rate / (div + 1);
}

static int _clk_main_bus_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			&lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CBCDR) & ~MX50_CCM_CBCDR_PERIPH_CLK_SEL_MASK;
	reg |= mux;
	__raw_writel(reg, MXC_CCM_CBCDR);

	return 0;
}

static struct clk main_bus_clk = {
	.name = "main_bus_clk",
	.parent = &pll2_sw_clk,
	.set_parent = _clk_main_bus_set_parent,
	.recalc = _clk_main_bus_recalc,
	.flags = RATE_PROPAGATES,
};

static void _clk_axi_a_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AXI_A_PODF_MASK) >>
	       MXC_CCM_CBCDR_AXI_A_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static int _clk_axi_a_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AXI_A_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AXI_A_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_AXI_A_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("pll _clk_axi_a_set_rate failed\n");
	}
	clk->rate = rate;

	return 0;
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

static void _clk_axi_b_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AXI_B_PODF_MASK) >>
	       MXC_CCM_CBCDR_AXI_B_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static int _clk_axi_b_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AXI_B_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AXI_B_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_AXI_B_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("_clk_axi_b_set_rate failed\n");
	}

	clk->rate = rate;

	return 0;
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

static void _clk_ahb_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AHB_PODF_MASK) >>
	       MXC_CCM_CBCDR_AHB_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}


static int _clk_ahb_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AHB_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AHB_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_AHB_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("_clk_ahb_set_rate failed\n");
	}
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

static int _clk_max_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);

	/* Handshake with MAX when LPM is entered. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg &= ~MXC_CCM_CLPCR_BYPASS_MAX_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}


static void _clk_max_disable(struct clk *clk)
{
	u32 reg;

	_clk_disable_inwait(clk);

	/* No Handshake with MAX when LPM is entered as its disabled. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_MAX_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}


static struct clk ahb_max_clk = {
	.name = "max_clk",
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG14_OFFSET,
	.enable = _clk_max_enable,
	.disable = _clk_max_disable,
};

static int _clk_weim_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CBCDR);
	if (parent == &ahb_clk)
		reg |= MX50_CCM_CBCDR_WEIM_CLK_SEL;
	else if (parent == &main_bus_clk)
		reg &= ~MX50_CCM_CBCDR_WEIM_CLK_SEL;
	else
		BUG();
	__raw_writel(reg, MXC_CCM_CBCDR);

	return 0;
}

static void _clk_weim_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_EMI_PODF_MASK) >>
	       MXC_CCM_CBCDR_EMI_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static int _clk_weim_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;
	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_EMI_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_EMI_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);
	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_EMI_PODF_BUSY) {
		getnstimeofday(&curtime);
		if ((curtime.tv_nsec - nstimeofday.tv_nsec) > SPIN_DELAY)
			panic("_clk_emi_slow_set_rate failed\n");
	}
	clk->rate = rate;

	return 0;
}

static unsigned long _clk_weim_round_rate(struct clk *clk,
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


static struct clk weim_clk = {
	.name = "weim_clk",
	.parent = &main_bus_clk,
	.set_parent = _clk_weim_set_parent,
	.recalc = _clk_weim_recalc,
	.set_rate = _clk_weim_set_rate,
	.round_rate = _clk_weim_round_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG8_OFFSET,
	.disable = _clk_disable_inwait,
	.flags = RATE_PROPAGATES,
};

static struct clk ahbmux1_clk = {
	.name = "ahbmux1_clk",
	.id = 0,
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG8_OFFSET,
	.disable = _clk_disable_inwait,
};

static void _clk_ipg_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_IPG_PODF_MASK) >>
	       MXC_CCM_CBCDR_IPG_PODF_OFFSET) + 1;
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
		reg = __raw_readl(MXC_CCM_CBCDR);
		prediv1 = ((reg & MXC_CCM_CBCDR_PERCLK_PRED1_MASK) >>
			   MXC_CCM_CBCDR_PERCLK_PRED1_OFFSET) + 1;
		prediv2 = ((reg & MXC_CCM_CBCDR_PERCLK_PRED2_MASK) >>
			   MXC_CCM_CBCDR_PERCLK_PRED2_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CBCDR_PERCLK_PODF_MASK) >>
			MXC_CCM_CBCDR_PERCLK_PODF_OFFSET) + 1;
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

	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &main_bus_clk, &lp_apm_clk, &ipg_clk, NULL);
	if (mux == 2) {
		reg |= MXC_CCM_CBCMR_PERCLK_IPG_CLK_SEL;
	} else {
		reg &= ~MXC_CCM_CBCMR_PERCLK_IPG_CLK_SEL;
		if (mux == 0)
			reg &= ~MXC_CCM_CBCMR_PERCLK_LP_APM_CLK_SEL;
		else
			reg |= MXC_CCM_CBCMR_PERCLK_LP_APM_CLK_SEL;
	}
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk ipg_perclk = {
	.name = "ipg_perclk",
	.parent = &lp_apm_clk,
	.recalc = _clk_ipg_per_recalc,
	.set_parent = _clk_ipg_per_set_parent,
	.flags = RATE_PROPAGATES,
};

static struct clk ipmux1_clk = {
	.name = "ipmux1",
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG6_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk ipmux2_clk = {
	.name = "ipmux2",
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static int _clk_ocram_enable(struct clk *clk)
{
	return 0;
}

static void _clk_ocram_disable(struct clk *clk)
{
}

static struct clk ocram_clk = {
	.name = "ocram_clk",
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG1_OFFSET,
	.enable = _clk_ocram_enable,
	.disable = _clk_ocram_disable,
};


static struct clk aips_tz1_clk = {
	.name = "aips_tz1_clk",
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk aips_tz2_clk = {
	.name = "aips_tz2_clk",
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk gpc_dvfs_clk = {
	.name = "gpc_dvfs_clk",
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static int _clk_sdma_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);

	/* Handshake with SDMA when LPM is entered. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg &= ~MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static void _clk_sdma_disable(struct clk *clk)
{
	u32 reg;

	_clk_disable(clk);
	/* No handshake with SDMA as its not enabled. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}


static struct clk sdma_clk[] = {
	{
	 .name = "sdma_ahb_clk",
	 .parent = &ahb_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG15_OFFSET,
	 .enable = _clk_sdma_enable,
	 .disable = _clk_sdma_disable,
	 },
	{
	 .name = "sdma_ipg_clk",
	 .parent = &ipg_clk,
	 },
};

static struct clk spba_clk = {
	.name = "spba_clk",
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG0_OFFSET,
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

static struct clk uart_main_clk = {
	.name = "uart_main_clk",
	.parent = &pll2_sw_clk,
	.recalc = _clk_uart_recalc,
	.set_parent = _clk_uart_set_parent,
	.flags = RATE_PROPAGATES,
};

static struct clk uart1_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 0,
	 .parent = &uart_main_clk,
	 .secondary = &uart1_clk[1],
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG4_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#ifdef UART1_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
#ifdef UART1_DMA_ENABLE
	 .secondary = &aips_tz1_clk,
#endif
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG3_OFFSET,
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
	 .enable_shift = MXC_CCM_CCGR1_CG6_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#ifdef UART2_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
#ifdef UART2_DMA_ENABLE
	 .secondary = &aips_tz1_clk,
#endif
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG5_OFFSET,
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
	 .enable_shift = MXC_CCM_CCGR1_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#ifdef UART3_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG7_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart4_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 3,
	 .parent = &uart_main_clk,
	 .secondary = &uart4_clk[1],
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGR7_CG5_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#ifdef UART4_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 3,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGR7_CG4_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart5_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 4,
	 .parent = &uart_main_clk,
	 .secondary = &uart5_clk[1],
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGR7_CG7_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#ifdef UART5_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 4,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGR7_CG6_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk gpt_clk[] = {
	{
	 .name = "gpt_clk",
	 .parent = &ipg_perclk,
	 .id = 0,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &gpt_clk[1],
	 },
	{
	 .name = "gpt_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "gpt_32k_clk",
	 .id = 0,
	 .parent = &ckil_clk,
	 },
};

static struct clk pwm1_clk[] = {
	{
	 .name = "pwm",
	 .parent = &ipg_perclk,
	 .id = 0,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG6_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &pwm1_clk[1],
	 },
	{
	 .name = "pwm_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG5_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
	 .disable = _clk_disable,
	 },
	{
	 .name = "pwm_32k_clk",
	 .id = 0,
	 .parent = &ckil_clk,
	 },
};

static struct clk pwm2_clk[] = {
	{
	 .name = "pwm",
	 .parent = &ipg_perclk,
	 .id = 1,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &pwm2_clk[1],
	 },
	{
	 .name = "pwm_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG7_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
	 .disable = _clk_disable,
	 },
	{
	 .name = "pwm_32k_clk",
	 .id = 1,
	 .parent = &ckil_clk,
	 },
};

static struct clk i2c_clk[] = {
	{
	 .name = "i2c_clk",
	 .id = 0,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "i2c_clk",
	 .id = 1,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "i2c_clk",
	 .id = 2,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG11_OFFSET,
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

static struct clk cspi_main_clk = {
	.name = "cspi_main_clk",
	.parent = &pll3_sw_clk,
	.recalc = _clk_cspi_recalc,
	.set_parent = _clk_cspi_set_parent,
	.flags = RATE_PROPAGATES,
};

static struct clk cspi1_clk[] = {
	{
	 .name = "cspi_clk",
	 .id = 0,
	 .parent = &cspi_main_clk,
	 .secondary = &cspi1_clk[1],
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "cspi_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG9_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
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
	 .enable_shift = MXC_CCM_CCGR4_CG12_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "cspi_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &aips_tz2_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG11_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
	 .disable = _clk_disable,
	 },
};

static struct clk cspi3_clk = {
	 .name = "cspi_clk",
	 .id = 2,
	.parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG13_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &aips_tz2_clk,
};

static int _clk_ssi_lp_apm_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &ckih_clk, &lp_apm_clk, &ckih2_clk, NULL);
	reg = __raw_readl(MXC_CCM_CSCMR1) &
	    ~MXC_CCM_CSCMR1_SSI_APM_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_SSI_APM_CLK_SEL_OFFSET;
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
	 .enable_shift = MXC_CCM_CCGR3_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &ssi1_clk[2],
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_dep_clk",
	 .id = 0,
	 .parent = &aips_tz2_clk,
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
	 .enable_shift = MXC_CCM_CCGR3_CG11_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &ssi2_clk[2],
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_dep_clk",
	 .id = 1,
	 .parent = &spba_clk,
	 },
};

static void _clk_ssi_ext1_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	clk->rate = clk->parent->rate;
	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & MXC_CCM_CSCMR1_SSI_EXT1_COM_CLK_SEL) == 0) {
		reg = __raw_readl(MXC_CCM_CS1CDR);
		prediv = ((reg & MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_MASK) >>
			  MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_OFFSET) + 1;
		if (prediv == 1)
			BUG();
		podf = ((reg & MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_MASK) >>
			MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / (prediv * podf);
	}
}

static int _clk_ssi_ext1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || div > 512)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg &= ~(MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_MASK |
		 MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CS1CDR);

	clk->rate = rate;

	return 0;
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

static unsigned long _clk_ssi_ext1_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 pre, post;
	u32 div = clk->parent->rate / rate;

	if (clk->parent->rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return clk->parent->rate / (pre * post);
}

static struct clk ssi_ext1_clk = {
	.name = "ssi_ext1_clk",
	.parent = &pll3_sw_clk,
	.set_parent = _clk_ssi_ext1_set_parent,
	.set_rate = _clk_ssi_ext1_set_rate,
	.round_rate = _clk_ssi_ext1_round_rate,
	.recalc = _clk_ssi_ext1_recalc,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGR3_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static void _clk_ssi_ext2_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	clk->rate = clk->parent->rate;
	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & MXC_CCM_CSCMR1_SSI_EXT2_COM_CLK_SEL) == 0) {
		reg = __raw_readl(MXC_CCM_CS2CDR);
		prediv = ((reg & MXC_CCM_CS2CDR_SSI_EXT2_CLK_PRED_MASK) >>
			  MXC_CCM_CS2CDR_SSI_EXT2_CLK_PRED_OFFSET) + 1;
		if (prediv == 1)
			BUG();
		podf = ((reg & MXC_CCM_CS2CDR_SSI_EXT2_CLK_PODF_MASK) >>
			MXC_CCM_CS2CDR_SSI_EXT2_CLK_PODF_OFFSET) + 1;
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
	.enable_shift = MXC_CCM_CCGR3_CG15_OFFSET,
	.enable = _clk_enable,
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

static struct clk usb_ahb_clk = {
	 .name = "usb_ahb_clk",
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG13_OFFSET,
	 .disable = _clk_disable,
};

static struct clk usb_phy_clk[] = {
	{
	.name = "usb_phy1_clk",
	.id = 0,
	.parent = &osc_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG5_OFFSET,
	.disable = _clk_disable,
	},
	{
	.name = "usb_phy2_clk",
	.id = 1,
	.parent = &osc_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG6_OFFSET,
	.disable = _clk_disable,
	}
};

static struct clk esdhc_dep_clks = {
	 .name = "sd_dep_clk",
	 .parent = &spba_clk,
};

static void _clk_esdhc1_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_esdhc1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = reg & ~MXC_CCM_CSCMR1_ESDHC1_MSHC2_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_ESDHC1_MSHC2_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}


static int _clk_esdhc1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	 __calc_pre_post_dividers(div, &pre, &post);

	/* Set sdhc1 clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR1) &
		~(MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_MASK |
		MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	clk->rate = rate;
	return 0;
}

static struct clk esdhc1_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 0,
	 .parent = &pll2_sw_clk,
	 .set_parent = _clk_esdhc1_set_parent,
	 .recalc = _clk_esdhc1_recalc,
	 .set_rate = _clk_esdhc1_set_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG1_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc1_clk[1],
	 },
	{
	 .name = "esdhc_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &esdhc1_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG0_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .name = "esdhc_sec_clk",
	 .id = 0,
	 .parent = &tmax2_clk,
	 .secondary = &esdhc_dep_clks,
	 },

};

static int _clk_esdhc2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &esdhc1_clk[0])
		reg &= ~MXC_CCM_CSCMR1_ESDHC2_CLK_SEL;
	else if (parent == &esdhc3_clk[0])
		reg |= MXC_CCM_CSCMR1_ESDHC2_CLK_SEL;
	else
		BUG();
	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static struct clk esdhc2_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 1,
	 .parent = &esdhc1_clk[0],
	 .set_parent = _clk_esdhc2_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG3_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc2_clk[1],
	 },
	{
	 .name = "esdhc_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &esdhc2_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG2_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .name = "esdhc_sec_clk",
	 .id = 0,
	 .parent = &tmax2_clk,
	 .secondary = &esdhc_dep_clks,
	 },
};

static int _clk_esdhc3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	mux = _get_mux8(parent, &pll1_sw_clk, &pll2_sw_clk,
			&pll3_sw_clk, &lp_apm_clk, &pfd0_clk,
			&pfd1_clk, &pfd4_clk, &osc_clk);
	reg = reg & ~MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static void _clk_esdhc3_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_esdhc3_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	/* Set sdhc1 clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR1) &
		~(MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_MASK |
		MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_OFFSET;
	 __raw_writel(reg, MXC_CCM_CSCDR1);

	clk->rate = rate;
	return 0;
}


static struct clk esdhc3_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 2,
	 .parent = &pll2_sw_clk,
	 .set_parent = _clk_esdhc3_set_parent,
	 .recalc = _clk_esdhc3_recalc,
	 .set_rate = _clk_esdhc3_set_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG5_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc3_clk[1],
	 },
	{
	 .name = "esdhc_ipg_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .secondary = &esdhc3_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG4_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .name = "esdhc_sec_clk",
	 .id = 0,
	 .parent = &ahb_max_clk,
	 .secondary = &esdhc_dep_clks,
	 },
};

static int _clk_esdhc4_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &esdhc1_clk[0])
		reg &= ~MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
	else if (parent == &esdhc3_clk[0])
		reg |= MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
	else
		BUG();
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk esdhc4_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 3,
	 .parent = &esdhc1_clk[0],
	 .set_parent = _clk_esdhc4_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG7_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc4_clk[1],
	 },
	{
	 .name = "esdhc_ipg_clk",
	 .id = 3,
	 .parent = &ipg_clk,
	 .secondary = &esdhc4_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG6_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .name = "esdhc_sec_clk",
	 .id = 0,
	 .parent = &tmax2_clk,
	 .secondary = &esdhc_dep_clks,
	 },
};

static int _clk_ddr_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CLK_DDR);
	if (parent == &pfd0_clk)
		reg |= MXC_CCM_CLK_DDR_DDR_PFD_SEL;
	else if (parent == &pll1_sw_clk)
		reg &= ~MXC_CCM_CLK_DDR_DDR_PFD_SEL;
	else
		return -EINVAL;
	return 0;
}

static void _clk_ddr_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CLK_DDR);
	div = (reg & MXC_CCM_CLK_DDR_DDR_DIV_PLL_MASK) >>
		MXC_CCM_CLK_DDR_DDR_DIV_PLL_OFFSET;
	if (div)
		clk->rate = clk->parent->rate / div;
	else
		clk->rate = 0;
}

static struct clk ddr_clk = {
	.name = "ddr_clk",
	.parent = &pll1_sw_clk,
	.set_parent = _clk_ddr_set_parent,
	.recalc = _clk_ddr_recalc,
	.flags = RATE_PROPAGATES,
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

static struct clk usb_clk = {
	.name = "usb_clk",
	.rate = 60000000,
};

static struct clk rtc_clk = {
	.name = "rtc_clk",
	.parent = &ckil_clk,
	.secondary = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG14_OFFSET,
	.disable = _clk_disable,
};

static struct clk owire_clk = {
	/* 1w driver come from upstream and use owire as clock name*/
	.name = "owire",
	.parent = &ipg_perclk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGR2_CG11_OFFSET,
	.disable = _clk_disable,
};


static struct clk fec_clk[] = {
	{
	.name = "fec_clk",
	.parent = &ipg_clk,
	.secondary = &fec_clk[1],
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGR2_CG12_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	 .name = "fec_sec1_clk",
	 .parent = &tmax2_clk,
	 .secondary = &fec_clk[2],
	},
	{
	 .name = "fec_sec2_clk",
	 .parent = &aips_tz2_clk,
	},
};

static int _clk_gpu2d_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &weim_clk, &ahb_clk);
	reg = (reg & ~MXC_CCM_CBCMR_GPU2D_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_GPU2D_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk gpu2d_clk = {
	.name = "gpu2d_clk",
	.parent = &axi_a_clk,
	.set_parent = _clk_gpu2d_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG7_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk apbh_dma_clk = {
	.name = "apbh_dma_clk",
	.parent = &pll1_sw_clk,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGR7_CG10_OFFSET,
};

static int _clk_display_axi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CLKSEQ_BYPASS);
	mux = _get_mux(parent, &osc_clk, &pfd2_clk, &pll1_sw_clk, NULL);
	reg = (reg & ~MXC_CCM_CLKSEQ_BYPASS_BYPASS_DISPLAY_AXI_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CLKSEQ_BYPASS_BYPASS_DISPLAY_AXI_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CLKSEQ_BYPASS);

	return 0;
}

static void _clk_display_axi_recalc(struct clk *clk)
{
	u32 div;

	div = __raw_readl(MXC_CCM_DISPLAY_AXI);
	div &= MXC_CCM_DISPLAY_AXI_DIV_MASK;
	if (div == 0) { /* gated off */
		clk->rate = clk->parent->rate;
	} else {
		clk->rate = clk->parent->rate / div;
	}
}

static unsigned long _clk_display_axi_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 max_div = (2 << 6) - 1;
	return _clk_round_rate_div(clk, rate, max_div, NULL);
}

static int _clk_display_axi_set_rate(struct clk *clk, unsigned long rate)
{
	u32 new_div, max_div;
	u32 reg;

	max_div = (2 << 6) - 1;
	clk->rate = _clk_round_rate_div(clk, rate, max_div, &new_div);

	reg = __raw_readl(MXC_CCM_DISPLAY_AXI);
	reg &= ~MXC_CCM_DISPLAY_AXI_DIV_MASK;
	reg |= new_div << MXC_CCM_DISPLAY_AXI_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_DISPLAY_AXI);

#if 0
	while (__raw_readl(MXC_CCM_CSR2) & MXC_CCM_CSR2_DISPLAY_AXI_BUSY)
		;
#endif

	return 0;
}

static struct clk display_axi_clk = {
	.name = "display_axi",
	.parent = &osc_clk,
	.set_parent = _clk_display_axi_set_parent,
	.recalc = _clk_display_axi_recalc,
	.set_rate = _clk_display_axi_set_rate,
	.round_rate = _clk_display_axi_round_rate,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_DISPLAY_AXI,
	.enable_shift = MXC_CCM_DISPLAY_AXI_CLKGATE_OFFSET,
};

/* TODO: check Auto-Slow Mode */
static struct clk pxp_axi_clk = {
	.name = "pxp_axi",
	.parent = &display_axi_clk,
	.secondary = &apbh_dma_clk,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG9_OFFSET,
};

static struct clk elcdif_axi_clk = {
	.name = "elcdif_axi",
	.parent = &display_axi_clk,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG10_OFFSET,
};

static int _clk_elcdif_pix_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CLKSEQ_BYPASS);
	mux = _get_mux(parent, &osc_clk, &pfd6_clk, &pll1_sw_clk, &ckih_clk);
	reg = (reg & ~MXC_CCM_CLKSEQ_BYPASS_BYPASS_ELCDIF_PIX_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CLKSEQ_BYPASS_BYPASS_ELCDIF_PIX_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CLKSEQ_BYPASS);

	return 0;
}

static void _clk_elcdif_pix_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_ELCDIFPIX);
	prediv = ((reg & MXC_CCM_ELCDIFPIX_CLK_PRED_MASK) >>
		  MXC_CCM_ELCDIFPIX_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_ELCDIFPIX_CLK_PODF_MASK) >>
		MXC_CCM_ELCDIFPIX_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static unsigned long _clk_elcdif_pix_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 max_div = (2 << 12) - 1;
	return _clk_round_rate_div(clk, rate, max_div, NULL);
}

static int _clk_elcdif_pix_set_rate(struct clk *clk, unsigned long rate)
{
	u32 new_div, max_div;
	u32 reg;

	max_div = (2 << 12) - 1;
	clk->rate = _clk_round_rate_div(clk, rate, max_div, &new_div);

	reg = __raw_readl(MXC_CCM_ELCDIFPIX);
	/* Pre-divider set to 1 - only use PODF for clk dividing */
	reg &= ~MXC_CCM_ELCDIFPIX_CLK_PRED_MASK;
	reg |= 1 << MXC_CCM_ELCDIFPIX_CLK_PRED_OFFSET;
	reg &= ~MXC_CCM_ELCDIFPIX_CLK_PODF_MASK;
	reg |= new_div << MXC_CCM_ELCDIFPIX_CLK_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_ELCDIFPIX);

	return 0;
}

static struct clk elcdif_pix_clk = {
	.name = "elcdif_pix",
	.parent = &osc_clk,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG6_OFFSET,
	.set_parent = _clk_elcdif_pix_set_parent,
	.recalc = _clk_elcdif_pix_recalc,
	.round_rate = _clk_elcdif_pix_round_rate,
	.set_rate = _clk_elcdif_pix_set_rate,
};

static int _clk_epdc_axi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CLKSEQ_BYPASS);
	mux = _get_mux(parent, &osc_clk, &pfd3_clk, &pll1_sw_clk, NULL);
	reg = (reg & ~MXC_CCM_CLKSEQ_BYPASS_BYPASS_EPDC_AXI_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CLKSEQ_BYPASS_BYPASS_EPDC_AXI_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CLKSEQ_BYPASS);

	return 0;
}

static void _clk_epdc_axi_recalc(struct clk *clk)
{
	u32 div;

	div = __raw_readl(MXC_CCM_EPDC_AXI);
	div &= MXC_CCM_EPDC_AXI_DIV_MASK;
	if (div == 0) { /* gated off */
		clk->rate = clk->parent->rate;
	} else {
		clk->rate = clk->parent->rate / div;
	}
}

static unsigned long _clk_epdc_axi_round_rate_div(struct clk *clk,
						unsigned long rate,
						u32 *new_div)
{
	u32 div, max_div;

	max_div = (2 << 6) - 1;
	div = DIV_ROUND_UP(clk->parent->rate, rate);
	if (div > max_div)
		div = max_div;
	else if (div == 0)
		div++;
	if (new_div != NULL)
		*new_div = div;
	return clk->parent->rate / div;
}

static unsigned long _clk_epdc_axi_round_rate(struct clk *clk,
						unsigned long rate)
{
	return _clk_epdc_axi_round_rate_div(clk, rate, NULL);
}

static int _clk_epdc_axi_set_rate(struct clk *clk, unsigned long rate)
{
	u32 new_div;
	u32 reg;

	clk->rate = _clk_epdc_axi_round_rate_div(clk, rate, &new_div);

	reg = __raw_readl(MXC_CCM_EPDC_AXI);
	reg &= ~MXC_CCM_EPDC_AXI_DIV_MASK;
	reg |= new_div << MXC_CCM_EPDC_AXI_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_EPDC_AXI);

	while (__raw_readl(MXC_CCM_CSR2) & MXC_CCM_CSR2_EPDC_AXI_BUSY)
		;

	return 0;
}

static int _clk_epdc_axi_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCGR6);
	reg |= MXC_CCM_CCGR6_CG8_MASK;
	__raw_writel(reg, MXC_CCM_CCGR6);

	reg = __raw_readl(MXC_CCM_EPDC_AXI);
	reg |= MXC_CCM_EPDC_AXI_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_EPDC_AXI);

	return 0;
}

static void _clk_epdc_axi_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCGR6);
	reg &= ~MXC_CCM_CCGR6_CG8_MASK;
	__raw_writel(reg, MXC_CCM_CCGR6);

	reg = __raw_readl(MXC_CCM_EPDC_AXI);
	reg &= ~MXC_CCM_EPDC_AXI_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_EPDC_AXI);
}

/* TODO: check Auto-Slow Mode */
static struct clk epdc_axi_clk = {
	.name = "epdc_axi",
	.parent = &apbh_dma_clk,
	.set_parent = _clk_epdc_axi_set_parent,
	.recalc = _clk_epdc_axi_recalc,
	.set_rate = _clk_epdc_axi_set_rate,
	.round_rate = _clk_epdc_axi_round_rate,
	.enable = _clk_epdc_axi_enable,
	.disable = _clk_epdc_axi_disable,
};


static int _clk_epdc_pix_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CLKSEQ_BYPASS);
	mux = _get_mux(parent, &osc_clk, &pfd5_clk, &pll1_sw_clk, &ckih_clk);
	reg = (reg & ~MXC_CCM_CLKSEQ_BYPASS_BYPASS_EPDC_PIX_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CLKSEQ_BYPASS_BYPASS_EPDC_PIX_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CLKSEQ_BYPASS);

	return 0;
}

static void _clk_epdc_pix_recalc(struct clk *clk)
{
	u32 div;

	div = __raw_readl(MXC_CCM_EPDCPIX);
	div &= MXC_CCM_EPDC_PIX_CLK_PODF_MASK;
	if (div == 0) { /* gated off */
		clk->rate = clk->parent->rate;
	} else {
		clk->rate = clk->parent->rate / div;
	}
}

static unsigned long _clk_epdc_pix_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 max_div = (2 << 12) - 1;
	return _clk_round_rate_div(clk, rate, max_div, NULL);
}

static int _clk_epdc_pix_set_rate(struct clk *clk, unsigned long rate)
{
	u32 new_div, max_div;
	u32 reg;

	max_div = (2 << 12) - 1;
	clk->rate = _clk_round_rate_div(clk, rate, max_div, &new_div);

	reg = __raw_readl(MXC_CCM_EPDCPIX);
	/* Pre-divider set to 1 - only use PODF for clk dividing */
	reg &= ~MXC_CCM_EPDC_PIX_CLK_PRED_MASK;
	reg |= 1 << MXC_CCM_EPDC_PIX_CLK_PRED_OFFSET;
	reg &= ~MXC_CCM_EPDC_PIX_CLK_PODF_MASK;
	reg |= new_div << MXC_CCM_EPDC_PIX_CLK_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_EPDCPIX);

	while (__raw_readl(MXC_CCM_CSR2) & MXC_CCM_CSR2_EPDC_PIX_BUSY)
		;

	return 0;
}

static int _clk_epdc_pix_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCGR6);
	reg |= MXC_CCM_CCGR6_CG5_MASK;
	__raw_writel(reg, MXC_CCM_CCGR6);

	reg = __raw_readl(MXC_CCM_EPDCPIX);
	reg |= MXC_CCM_EPDC_PIX_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_EPDCPIX);

	return 0;
}

static void _clk_epdc_pix_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCGR6);
	reg &= ~MXC_CCM_CCGR6_CG5_MASK;
	__raw_writel(reg, MXC_CCM_CCGR6);

	reg = __raw_readl(MXC_CCM_EPDCPIX);
	reg &= ~MXC_CCM_EPDC_PIX_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_EPDCPIX);
}

/* TODO: check Auto-Slow Mode */
static struct clk epdc_pix_clk = {
	.name = "epdc_pix",
	.parent = &osc_clk,
	.set_parent = _clk_epdc_pix_set_parent,
	.recalc = _clk_epdc_pix_recalc,
	.set_rate = _clk_epdc_pix_set_rate,
	.round_rate = _clk_epdc_pix_round_rate,
	.enable = _clk_epdc_pix_enable,
	.disable = _clk_epdc_pix_disable,
};

static void cko1_recalc(struct clk *clk)
{
	unsigned long rate;
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= MX50_CCM_CCOSR_CKO1_DIV_MASK;
	reg = reg >> MX50_CCM_CCOSR_CKO1_DIV_OFFSET;
	rate = clk->parent->rate;
	clk->rate = rate / (reg + 1);
}

static int cko1_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg |= MX50_CCM_CCOSR_CKO1_EN;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}

static void cko1_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MX50_CCM_CCOSR_CKO1_EN;
	__raw_writel(reg, MXC_CCM_CCOSR);
}

static int cko1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;

	div = (clk->parent->rate/rate - 1) & 0x7;
	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MX50_CCM_CCOSR_CKO1_DIV_MASK;
	reg |= div << MX50_CCM_CCOSR_CKO1_DIV_OFFSET;
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
	u32 sel, reg, fast;

	if (parent == &cpu_clk) {
		sel = 0;
		fast = 1;
	} else if (parent == &pll1_sw_clk) {
		sel = 1;
		fast = 1;
	} else if (parent == &pll2_sw_clk) {
		sel = 2;
		fast = 1;
	} else if (parent == &pll3_sw_clk) {
		sel = 3;
		fast = 1;
	} else if (parent == &apll_clk) {
		sel = 0;
		fast = 0;
	} else if (parent == &pfd0_clk) {
		sel = 1;
		fast = 0;
	} else if (parent == &pfd1_clk) {
		sel = 2;
		fast = 0;
	} else if (parent == &pfd2_clk) {
		sel = 3;
		fast = 0;
	} else if (parent == &pfd3_clk) {
		sel = 4;
		fast = 0;
	} else if (parent == &pfd4_clk) {
		sel = 5;
		fast = 0;
	} else if (parent == &pfd5_clk) {
		sel = 6;
		fast = 0;
	} else if (parent == &pfd6_clk) {
		sel = 7;
		fast = 0;
	} else if (parent == &weim_clk) {
		sel = 10;
		fast = 0;
	} else if (parent == &ahb_clk) {
		sel = 11;
		fast = 0;
	} else if (parent == &ipg_clk) {
		sel = 12;
		fast = 0;
	} else if (parent == &ipg_perclk) {
		sel = 13;
		fast = 0;
	} else if (parent == &pfd7_clk) {
		sel = 15;
		fast = 0;
	} else
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MX50_CCM_CCOSR_CKO1_SEL_MASK;
	reg |= sel << MX50_CCM_CCOSR_CKO1_SEL_OFFSET;
	if (fast)
		reg &= ~MX50_CCM_CCOSR_CKO1_SLOW_SEL;
	else
		reg |= MX50_CCM_CCOSR_CKO1_SLOW_SEL;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}
static struct clk cko1_clk = {
	.name = "cko1_clk",
	.parent = &pll1_sw_clk,
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
	&ckih2_clk,
	&ckil_clk,
	&pll1_main_clk,
	&pll1_sw_clk,
	&pll2_sw_clk,
	&pll3_sw_clk,
	&apll_clk,
	&pfd0_clk,
	&pfd1_clk,
	&pfd2_clk,
	&pfd3_clk,
	&pfd4_clk,
	&pfd5_clk,
	&pfd6_clk,
	&pfd7_clk,
	&ipmux1_clk,
	&ipmux2_clk,
	&gpc_dvfs_clk,
	&lp_apm_clk,
	&cpu_clk,
	&main_bus_clk,
	&axi_a_clk,
	&axi_b_clk,
	&ahb_clk,
	&ahb_max_clk,
	&ipg_clk,
	&ipg_perclk,
	&ahbmux1_clk,
	&aips_tz1_clk,
	&aips_tz2_clk,
	&sdma_clk[0],
	&sdma_clk[1],
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
	&gpt_clk[0],
	&gpt_clk[1],
	&gpt_clk[2],
	&pwm1_clk[0],
	&pwm1_clk[1],
	&pwm1_clk[2],
	&pwm2_clk[0],
	&pwm2_clk[1],
	&pwm2_clk[2],
	&cspi_main_clk,
	&cspi1_clk[0],
	&cspi1_clk[1],
	&cspi2_clk[0],
	&cspi2_clk[1],
	&cspi3_clk,
	&ssi_lp_apm_clk,
	&ssi1_clk[0],
	&ssi1_clk[1],
	&ssi1_clk[2],
	&ssi2_clk[0],
	&ssi2_clk[1],
	&ssi2_clk[2],
	&ssi_ext1_clk,
	&ssi_ext2_clk,
	&tmax2_clk,
	&usb_ahb_clk,
	&usb_phy_clk[0],
	&usb_clk,
	&esdhc1_clk[0],
	&esdhc1_clk[1],
	&esdhc2_clk[0],
	&esdhc2_clk[1],
	&esdhc3_clk[0],
	&esdhc3_clk[1],
	&esdhc4_clk[0],
	&esdhc4_clk[1],
	&esdhc_dep_clks,
	&weim_clk,
	&ddr_clk,
	&pgc_clk,
	&rtc_clk,
	&owire_clk,
	&fec_clk[0],
	&fec_clk[1],
	&fec_clk[2],
	&gpu2d_clk,
	&cko1_clk,
	&display_axi_clk,
	&pxp_axi_clk,
	&elcdif_axi_clk,
	&epdc_axi_clk,
	&epdc_pix_clk,
	&elcdif_pix_clk,
};

static void clk_tree_init(void)
{
	u32 reg;

	ipg_perclk.set_parent(&ipg_perclk, &lp_apm_clk);

	/*
	 *Initialise the IPG PER CLK dividers to 3. IPG_PER_CLK should be at
	 * 8MHz, its derived from lp_apm.
	 */
	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_PERCLK_PRED1_MASK;
	reg &= ~MXC_CCM_CBCDR_PERCLK_PRED2_MASK;
	reg &= ~MXC_CCM_CBCDR_PERCLK_PODF_MASK;
	reg |= (2 << MXC_CCM_CBCDR_PERCLK_PRED1_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCDR);

	/* set pll1_main_clk parent */
	pll1_main_clk.parent = &osc_clk;

	/* set pll2_sw_clk parent */
	pll2_sw_clk.parent = &osc_clk;

	/* set pll3_clk parent */
	pll3_sw_clk.parent = &osc_clk;

	/* set weim_clk parent */
	weim_clk.parent = &main_bus_clk;
	reg = __raw_readl(MXC_CCM_CBCDR);
	if ((reg & MX50_CCM_CBCDR_WEIM_CLK_SEL) != 0)
		weim_clk.parent = &ahb_clk;

	/* set ipg_perclk parent */
	ipg_perclk.parent = &lp_apm_clk;
	reg = __raw_readl(MXC_CCM_CBCMR);
	if ((reg & MXC_CCM_CBCMR_PERCLK_IPG_CLK_SEL) != 0) {
		ipg_perclk.parent = &ipg_clk;
	} else {
		if ((reg & MXC_CCM_CBCMR_PERCLK_LP_APM_CLK_SEL) == 0)
			ipg_perclk.parent = &main_bus_clk;
	}
}

int __init mx50_clocks_init(unsigned long ckil, unsigned long osc, unsigned long ckih1)
{
	__iomem void *base;
	struct clk **clkp;
	int i = 0, j = 0, reg;
	int wp_cnt = 0;

	pll1_base = ioremap(MX53_BASE_ADDR(PLL1_BASE_ADDR), SZ_4K);
	pll2_base = ioremap(MX53_BASE_ADDR(PLL2_BASE_ADDR), SZ_4K);
	pll3_base = ioremap(MX53_BASE_ADDR(PLL3_BASE_ADDR), SZ_4K);
	apll_base = ioremap(ANATOP_BASE_ADDR, SZ_4K);

	/* Turn off all possible clocks */
	if (mxc_jtag_enabled) {
		__raw_writel(1 << MXC_CCM_CCGR0_CG0_OFFSET |
			      1 << MXC_CCM_CCGR0_CG2_OFFSET |
			      3 << MXC_CCM_CCGR0_CG3_OFFSET |
			      3 << MXC_CCM_CCGR0_CG4_OFFSET |
			      3 << MXC_CCM_CCGR0_CG8_OFFSET |
			      1 << MXC_CCM_CCGR0_CG12_OFFSET |
			      1 << MXC_CCM_CCGR0_CG13_OFFSET |
			      1 << MXC_CCM_CCGR0_CG14_OFFSET, MXC_CCM_CCGR0);
	} else {
		__raw_writel(1 << MXC_CCM_CCGR0_CG0_OFFSET |
			      3 << MXC_CCM_CCGR0_CG3_OFFSET |
			      3 << MXC_CCM_CCGR0_CG8_OFFSET |
			      1 << MXC_CCM_CCGR0_CG12_OFFSET |
			      1 << MXC_CCM_CCGR0_CG13_OFFSET |
			      3 << MXC_CCM_CCGR0_CG14_OFFSET, MXC_CCM_CCGR0);
	}

	__raw_writel(0, MXC_CCM_CCGR1);
	__raw_writel(0, MXC_CCM_CCGR2);
	__raw_writel(0, MXC_CCM_CCGR3);
	__raw_writel(0, MXC_CCM_CCGR4);

	__raw_writel(1 << MXC_CCM_CCGR5_CG6_OFFSET |
		     1 << MXC_CCM_CCGR5_CG8_OFFSET |
		     3 << MXC_CCM_CCGR5_CG9_OFFSET, MXC_CCM_CCGR5);

	__raw_writel(3 << MXC_CCM_CCGR6_CG0_OFFSET |
				3 << MXC_CCM_CCGR6_CG1_OFFSET |
				3 << MXC_CCM_CCGR6_CG4_OFFSET |
				3 << MXC_CCM_CCGR6_CG8_OFFSET |
				3 << MXC_CCM_CCGR6_CG9_OFFSET |
				3 << MXC_CCM_CCGR6_CG12_OFFSET |
				3 << MXC_CCM_CCGR6_CG13_OFFSET |
				2 << MXC_CCM_CCGR6_CG14_OFFSET, MXC_CCM_CCGR6);

	__raw_writel(0, MXC_CCM_CCGR7);

	ckil_clk.rate = ckil;
	osc_clk.rate = osc;
	ckih_clk.rate = ckih1;

	usb_phy_clk[0].enable_reg = MXC_CCM_CCGR4;
	usb_phy_clk[0].enable_shift = MXC_CCM_CCGR4_CG5_OFFSET;

	clk_tree_init();

	for (clkp = mxc_clks; clkp < mxc_clks + ARRAY_SIZE(mxc_clks); clkp++)
		clk_register(*clkp);

	clk_register(&uart4_clk[0]);
	clk_register(&uart4_clk[1]);
	clk_register(&uart5_clk[0]);
	clk_register(&uart5_clk[1]);
	clk_register(&i2c_clk[2]);
	clk_register(&usb_phy_clk[1]);
	clk_register(&ocram_clk);

	/* set DDR clock parent */
	reg = __raw_readl(MXC_CCM_CLK_DDR) &
				MXC_CCM_CLK_DDR_DDR_PFD_SEL;
	if (reg)
		clk_set_parent(&ddr_clk, &pfd0_clk);
	else
		clk_set_parent(&ddr_clk, &pll1_sw_clk);

	clk_set_parent(&esdhc1_clk[0], &pll2_sw_clk);
	clk_set_parent(&esdhc1_clk[2], &tmax2_clk);
	clk_set_parent(&esdhc2_clk[0], &esdhc1_clk[0]);
	clk_set_parent(&esdhc3_clk[0], &pll2_sw_clk);

	clk_register(&apbh_dma_clk);

	clk_set_parent(&epdc_axi_clk, &pll1_sw_clk);
	/* Set EPDC AXI to 200MHz */
	/*
	clk_set_rate(&epdc_axi_clk, 200000000);
	*/
	__raw_writel(0xC0000008, MXC_CCM_EPDC_AXI);
	clk_set_parent(&epdc_pix_clk, &pll1_sw_clk);

	reg = __raw_readl(MXC_CCM_ELCDIFPIX);
	reg &= ~MXC_CCM_ELCDIFPIX_CLKGATE_MASK;
	reg = 0x3 << MXC_CCM_ELCDIFPIX_CLKGATE_OFFSET;
	__raw_writel(reg, MXC_CCM_ELCDIFPIX);
	clk_set_parent(&elcdif_pix_clk, &pll1_sw_clk);

	/* This will propagate to all children and init all the clock rates */
	propagate_rate(&osc_clk);
	propagate_rate(&ckih_clk);
	propagate_rate(&ckil_clk);
	propagate_rate(&pll1_sw_clk);
	propagate_rate(&pll2_sw_clk);
	propagate_rate(&pll3_sw_clk);

	clk_enable(&cpu_clk);

	clk_enable(&main_bus_clk);

	clk_enable(&apbh_dma_clk);

	propagate_rate(&apll_clk);

	/* Initialise the parents to be axi_b, parents are set to
	 * axi_a when the clocks are enabled.
	 */

	clk_set_parent(&gpu2d_clk, &axi_a_clk);

	/* move cspi to 24MHz */
	clk_set_parent(&cspi_main_clk, &lp_apm_clk);
	clk_set_rate(&cspi_main_clk, 12000000);

	/* set DISPLAY_AXI to 200Mhz */
	clk_set_parent(&display_axi_clk, &pll1_sw_clk);
	clk_set_rate(&display_axi_clk, 200000000);

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

	/* Change the SSI_EXT1_CLK to be sourced from PLL2 for camera */
	clk_disable(&ssi_ext1_clk);
	clk_set_parent(&ssi_ext1_clk, &pll2_sw_clk);
	clk_set_rate(&ssi_ext1_clk, 24000000);
	clk_enable(&ssi_ext1_clk);
	clk_set_parent(&ssi_ext2_clk, &ssi2_clk[0]);

	/* move usb_phy_clk to 24MHz */
	clk_set_parent(&usb_phy_clk[0], &osc_clk);
	clk_set_parent(&usb_phy_clk[1], &osc_clk);

	/* set SDHC root clock as 200MHZ*/
	clk_set_rate(&esdhc1_clk[0], 200000000);
	clk_set_rate(&esdhc3_clk[0], 200000000);

	/* Set the current working point. */
	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	/* Update the cpu working point table based on the PLL1 freq
	 * at boot time
	 */
	if (pll1_main_clk.rate <= cpu_wp_tbl[cpu_wp_nr - 1].cpu_rate)
		wp_cnt = 1;
	else if (pll1_main_clk.rate <= cpu_wp_tbl[1].cpu_rate &&
				pll1_main_clk.rate > cpu_wp_tbl[2].cpu_rate)
		wp_cnt = cpu_wp_nr - 1;
	else
		wp_cnt = cpu_wp_nr;

	cpu_wp_tbl[0].cpu_rate = pll1_main_clk.rate;

	if (wp_cnt == 1) {
		cpu_wp_tbl[0] = cpu_wp_tbl[cpu_wp_nr - 1];
		memset(&cpu_wp_tbl[cpu_wp_nr - 1], 0, sizeof(struct cpu_wp));
		memset(&cpu_wp_tbl[cpu_wp_nr - 2], 0, sizeof(struct cpu_wp));
	} else if (wp_cnt < cpu_wp_nr) {
		for (i = 0; i < wp_cnt; i++)
			cpu_wp_tbl[i] = cpu_wp_tbl[i+1];
		memset(&cpu_wp_tbl[i], 0, sizeof(struct cpu_wp));
	}

	if (wp_cnt < cpu_wp_nr) {
		set_num_cpu_wp(wp_cnt);
		cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	}


	for (j = 0; j < cpu_wp_nr; j++) {
		/* Change the CPU podf divider based on the boot up
		 * pll1 rate.
		 */
		cpu_wp_tbl[j].cpu_podf = max(
			(int)((pll1_main_clk.rate / cpu_wp_tbl[j].cpu_rate)
			- 1), 0);
		if (pll1_main_clk.rate/(cpu_wp_tbl[j].cpu_podf + 1) >
				cpu_wp_tbl[j].cpu_rate) {
			cpu_wp_tbl[j].cpu_podf++;
			cpu_wp_tbl[j].cpu_rate =
				 pll1_main_clk.rate/
				 (1000 * (cpu_wp_tbl[j].cpu_podf + 1));
			cpu_wp_tbl[j].cpu_rate *= 1000;
		}
		if (pll1_main_clk.rate/(cpu_wp_tbl[j].cpu_podf + 1) <
					cpu_wp_tbl[j].cpu_rate) {
			cpu_wp_tbl[j].cpu_rate = pll1_main_clk.rate;
		}
		cpu_wp_tbl[j].pll_rate = pll1_main_clk.rate;
	}
	/* Set the current working point. */
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
	propagate_rate(&pll3_sw_clk);

	clk_set_parent(&uart_main_clk, &lp_apm_clk);
	clk_set_parent(&gpu2d_clk, &axi_b_clk);

	clk_set_parent(&weim_clk, &ahb_clk);
	clk_set_rate(&weim_clk, clk_round_rate(&weim_clk, 130000000));

	base = ioremap(MX53_BASE_ADDR(GPT1_BASE_ADDR), SZ_4K);
	mxc_timer_init(&gpt_clk[0], base, MXC_INT_GPT);
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

	if (wp == cpu_curr_wp)
		return 0;

	p = &cpu_wp_tbl[wp];

	/*
	 * leave the PLL1 freq unchanged.
	 */
	reg = __raw_readl(MXC_CCM_CACRR);
	reg &= ~MXC_CCM_CACRR_ARM_PODF_MASK;
	reg |= cpu_wp_tbl[wp].cpu_podf << MXC_CCM_CACRR_ARM_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CACRR);
	cpu_curr_wp = wp;
	cpu_clk.rate = cpu_wp_tbl[wp].cpu_rate;

#if defined(CONFIG_CPU_FREQ)
	cpufreq_trig_needed = 1;
#endif
	return 0;
}
