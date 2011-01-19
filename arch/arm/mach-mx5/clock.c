/*
 * Copyright (C) 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/types.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <asm/clkdev.h>
#include <asm/div64.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/clock.h>
#include <mach/mxc_dvfs.h>
#include <mach/sdram_autogating.h>

#include "crm_regs.h"
#include "serial.h"
#include "mx53_wp.h"

/* External clock values passed-in by the board code */
static unsigned long external_high_reference, external_low_reference;
static unsigned long oscillator_reference, ckih2_reference;

static struct clk pll1_main_clk;
static struct clk pll1_sw_clk;
static struct clk pll2_sw_clk;
static struct clk pll3_sw_clk;
static struct clk pll4_sw_clk;
static struct clk lp_apm_clk;
static struct clk tve_clk;
static struct clk emi_fast_clk;
static struct clk emi_slow_clk;
static struct clk emi_intr_clk[];
static struct clk ddr_clk;
static struct clk ipu_clk[];
static struct clk ldb_di_clk[];
static struct clk axi_a_clk;
static struct clk axi_b_clk;
static struct clk ddr_hf_clk;
static struct clk mipi_hsp_clk;
static struct clk gpu3d_clk;
static struct clk gpu2d_clk;
static struct clk vpu_clk[];
static int cpu_curr_wp;
static struct cpu_wp *cpu_wp_tbl;

static void __iomem *pll1_base;
static void __iomem *pll2_base;
static void __iomem *pll3_base;
static void __iomem *pll4_base;

extern int cpu_wp_nr;
extern int lp_high_freq;
extern int lp_med_freq;
static int max_axi_a_clk;
static int max_axi_b_clk;
static int max_ahb_clk;
static int max_emi_slow_clk;
extern int dvfs_core_is_active;

#define SPIN_DELAY	1000000 /* in nanoseconds */
#define MAX_AXI_A_CLK_MX51 	166250000
#define MAX_AXI_A_CLK_MX53 	400000000
#define MAX_AXI_B_CLK_MX51 	133000000
#define MAX_AXI_B_CLK_MX53 	200000000
#define MAX_AHB_CLK_MX51	133000000
#define MAX_EMI_SLOW_CLK_MX51	133000000
#define MAX_AHB_CLK_MX53	133333333
#define MAX_EMI_SLOW_CLK_MX53	133333333
#define MAX_DDR_HF_RATE		200000000
/* To keep compatible with some NAND flash, limit
 * max NAND clk to 34MHZ. The user can modify it for
 * dedicate NAND flash */
#define MAX_NFC_CLK		34000000

extern int mxc_jtag_enabled;
extern int uart_at_24;
extern int cpufreq_trig_needed;
extern int low_bus_freq_mode;

static int cpu_clk_set_wp(int wp);
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
	reg |= MXC_CCM_CCGRx_CG_MASK << clk->enable_shift;
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
	reg &= ~(MXC_CCM_CCGRx_CG_MASK << clk->enable_shift);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);
	return 0;
}

static void _clk_disable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGRx_CG_MASK << clk->enable_shift);
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
	reg &= ~(MXC_CCM_CCGRx_CG_MASK << clk->enable_shift);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);
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
 * For the ddr muxed input clock
 */
static inline u32 _get_mux_ddr(struct clk *parent, struct clk *m0,
			   struct clk *m1, struct clk *m2, struct clk *m3, struct clk *m4)
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
	else if (pll == &pll4_sw_clk)
		return pll4_base;
	else
		BUG();

	return NULL;
}

static unsigned long get_high_reference_clock_rate(struct clk *clk)
{
	return external_high_reference;
}

static unsigned long get_low_reference_clock_rate(struct clk *clk)
{
	return external_low_reference;
}

static unsigned long get_oscillator_reference_clock_rate(struct clk *clk)
{
	return oscillator_reference;
}

static unsigned long get_ckih2_reference_clock_rate(struct clk *clk)
{
	return ckih2_reference;
}

/* External high frequency clock */
static struct clk ckih_clk = {
	.get_rate = get_high_reference_clock_rate,
};

static struct clk ckih2_clk = {
	.get_rate = get_ckih2_reference_clock_rate,
};

static struct clk osc_clk = {
	.get_rate = get_oscillator_reference_clock_rate,
};

/* External low frequency (32kHz) clock */
static struct clk ckil_clk = {
	.get_rate = get_low_reference_clock_rate,
};

static unsigned long _fpm_get_rate(struct clk *clk)
{
	u32 rate = ckil_clk.get_rate(&ckil_clk) * 512;
	if ((__raw_readl(MXC_CCM_CCR) & MXC_CCM_CCR_FPM_MULT_MASK) != 0)
		rate *= 2;
	return rate;
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
	.parent = &ckil_clk,
	.get_rate = _fpm_get_rate,
	.enable = _fpm_enable,
	.disable = _fpm_disable,
	.flags = RATE_PROPAGATES,
};

static unsigned long _fpm_div2_get_rate(struct clk *clk)
{
	return  clk_get_rate(clk->parent) / 2;
}

static struct clk fpm_div2_clk = {
	.parent = &fpm_clk,
	.get_rate = _fpm_div2_get_rate,
	.flags = RATE_PROPAGATES,
};

static unsigned long _clk_pll_get_rate(struct clk *clk)
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

	ref_clk = 2 * clk_get_rate(clk->parent);
	if (dbl != 0)
		ref_clk *= 2;

	ref_clk /= (pdf + 1);
	temp = (u64) ref_clk * mfn_abs;
	do_div(temp, mfd + 1);
	if (mfn < 0)
		temp = -temp;
	temp = (ref_clk * mfi) + temp;

	return temp;
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

	quad_parent_rate = 4 * clk_get_rate(clk->parent);
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
	return 0;
}

static int _clk_pll_enable(struct clk *clk)
{
	u32 reg;
	void __iomem *pllbase;
	struct timespec nstimeofday;
	struct timespec curtime;

	pllbase = _get_pll_base(clk);
	reg = __raw_readl(pllbase + MXC_PLL_DP_CTL);

	if (reg & MXC_PLL_DP_CTL_UPEN)
		return 0;

	reg |=  MXC_PLL_DP_CTL_UPEN;
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
	.parent = &osc_clk,
	.get_rate = _clk_pll_get_rate,
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

static unsigned long _clk_pll1_sw_get_rate(struct clk *clk)
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
	return clk_get_rate(clk->parent) / div;
}

/* pll1 switch clock */
static struct clk pll1_sw_clk = {
	.parent = &pll1_main_clk,
	.set_parent = _clk_pll1_sw_set_parent,
	.get_rate = _clk_pll1_sw_get_rate,
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
	.parent = &osc_clk,
	.get_rate = _clk_pll_get_rate,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_rate = _clk_pll_set_rate,
	.set_parent = _clk_pll2_sw_set_parent,
	.flags = RATE_PROPAGATES,
};

/* same as pll3_main_clk. These two clocks should always be the same */
static struct clk pll3_sw_clk = {
	.parent = &osc_clk,
	.set_rate = _clk_pll_set_rate,
	.get_rate = _clk_pll_get_rate,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.flags = RATE_PROPAGATES,
};

/* same as pll4_main_clk. These two clocks should always be the same */
static struct clk pll4_sw_clk = {
	.parent = &osc_clk,
	.set_rate = _clk_pll_set_rate,
	.get_rate = _clk_pll_get_rate,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.flags = RATE_PROPAGATES,
};

static int _clk_lp_apm_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	if (parent == &osc_clk)
		reg = __raw_readl(MXC_CCM_CCSR) & ~MXC_CCM_CCSR_LP_APM_SEL;
	else if (parent == &fpm_clk)
		reg = __raw_readl(MXC_CCM_CCSR) | MXC_CCM_CCSR_LP_APM_SEL;
	else
		return -EINVAL;

	__raw_writel(reg, MXC_CCM_CCSR);

	return 0;
}

static struct clk lp_apm_clk = {
	.parent = &osc_clk,
	.set_parent = _clk_lp_apm_set_parent,
	.flags = RATE_PROPAGATES,
};

static unsigned long _clk_arm_get_rate(struct clk *clk)
{
	u32 cacrr, div;

	cacrr = __raw_readl(MXC_CCM_CACRR);
	div = (cacrr & MXC_CCM_CACRR_ARM_PODF_MASK) + 1;
	return clk_get_rate(clk->parent) / div;
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
	.parent = &pll1_sw_clk,
	.get_rate = _clk_arm_get_rate,
	.set_rate = _clk_cpu_set_rate,
	.round_rate = _clk_cpu_round_rate,
};

static int _clk_periph_apm_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	struct timespec nstimeofday;
	struct timespec curtime;

	mux = _get_mux(parent, &pll1_sw_clk, &pll3_sw_clk, &lp_apm_clk, NULL);

	reg = __raw_readl(MXC_CCM_CBCMR) & ~MXC_CCM_CBCMR_PERIPH_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CBCMR_PERIPH_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCMR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) &
			MXC_CCM_CDHIPR_PERIPH_CLK_SEL_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("pll _clk_periph_apm_set_parent failed\n");
	}
	return 0;
}

static struct clk periph_apm_clk = {
	.parent = &pll1_sw_clk,
	.set_parent = _clk_periph_apm_set_parent,
	.flags = RATE_PROPAGATES,
};

/* TODO: Need to sync with GPC to determine if DVFS is in place so that
 * the DVFS_PODF divider can be applied in CDCR register.
 */
static unsigned long _clk_main_bus_get_rate(struct clk *clk)
{
	u32 div = 0;

	if (dvfs_per_divider_active() || low_bus_freq_mode)
		div  = (__raw_readl(MXC_CCM_CDCR) & 0x3);
	return clk_get_rate(clk->parent) / (div + 1);
}

static int _clk_main_bus_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

	if (parent == &pll2_sw_clk) {
		reg = __raw_readl(MXC_CCM_CBCDR) &
		    ~MXC_CCM_CBCDR_PERIPH_CLK_SEL;
	} else if (parent == &periph_apm_clk) {
		reg = __raw_readl(MXC_CCM_CBCDR) | MXC_CCM_CBCDR_PERIPH_CLK_SEL;
	} else {
		return -EINVAL;
	}
	__raw_writel(reg, MXC_CCM_CBCDR);
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static struct clk main_bus_clk = {
	.parent = &pll2_sw_clk,
	.set_parent = _clk_main_bus_set_parent,
	.get_rate = _clk_main_bus_get_rate,
	.flags = RATE_PROPAGATES,
};

static unsigned long _clk_axi_a_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AXI_A_PODF_MASK) >>
	       MXC_CCM_CBCDR_AXI_A_PODF_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static int _clk_axi_a_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

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
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static unsigned long _clk_axi_a_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;
	if (parent_rate / div > max_axi_a_clk)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}


static struct clk axi_a_clk = {
	.parent = &main_bus_clk,
	.get_rate = _clk_axi_a_get_rate,
	.set_rate = _clk_axi_a_set_rate,
	.round_rate = _clk_axi_a_round_rate,
	.flags = RATE_PROPAGATES,
};

static unsigned long _clk_ddr_hf_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_DDR_PODF_MASK) >>
	       MXC_CCM_CBCDR_DDR_PODF_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static unsigned long _clk_ddr_hf_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;
	if (parent_rate / div > MAX_DDR_HF_RATE)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static int _clk_ddr_hf_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_DDR_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_DDR_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_DDR_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("clk_ddr_hf_set_rate failed\n");
	}
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static struct clk ddr_hf_clk = {
	.parent = &pll1_sw_clk,
	.get_rate = _clk_ddr_hf_get_rate,
	.round_rate = _clk_ddr_hf_round_rate,
	.set_rate = _clk_ddr_hf_set_rate,
	.flags = RATE_PROPAGATES,
};

static unsigned long _clk_axi_b_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AXI_B_PODF_MASK) >>
	       MXC_CCM_CBCDR_AXI_B_PODF_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static int _clk_axi_b_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

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

	emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static unsigned long _clk_axi_b_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;
	if (parent_rate / div > max_axi_b_clk)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}


static struct clk axi_b_clk = {
	.parent = &main_bus_clk,
	.get_rate = _clk_axi_b_get_rate,
	.set_rate = _clk_axi_b_set_rate,
	.round_rate = _clk_axi_b_round_rate,
	.flags = RATE_PROPAGATES,
};

static unsigned long _clk_ahb_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AHB_PODF_MASK) >>
	       MXC_CCM_CBCDR_AHB_PODF_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}


static int _clk_ahb_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

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
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static unsigned long _clk_ahb_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;
	if (parent_rate / div > max_ahb_clk)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}


static struct clk ahb_clk = {
	.parent = &main_bus_clk,
	.get_rate = _clk_ahb_get_rate,
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
	if (cpu_is_mx51())
		reg &= ~MXC_CCM_CLPCR_BYPASS_MAX_LPM_HS_MX51;
	else
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
	if (cpu_is_mx51())
		reg |= MXC_CCM_CLPCR_BYPASS_MAX_LPM_HS_MX51;
	else
		reg |= MXC_CCM_CLPCR_BYPASS_MAX_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}


static struct clk ahb_max_clk = {
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.enable = _clk_max_enable,
	.disable = _clk_max_disable,
};

static int _clk_emi_slow_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	if (parent == &ahb_clk) {
		reg |= MXC_CCM_CBCDR_EMI_CLK_SEL;
	} else if (parent == &main_bus_clk) {
		reg &= ~MXC_CCM_CBCDR_EMI_CLK_SEL;
	} else {
		BUG();
	}
	__raw_writel(reg, MXC_CCM_CBCDR);

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static unsigned long _clk_emi_slow_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_EMI_PODF_MASK) >>
	       MXC_CCM_CBCDR_EMI_PODF_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static int _clk_emi_slow_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
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

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);

	return 0;
}

static unsigned long _clk_emi_slow_round_rate(struct clk *clk,
					      unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;
	if (parent_rate / div > max_emi_slow_clk)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}


static struct clk emi_slow_clk = {
	.parent = &main_bus_clk,
	.set_parent = _clk_emi_slow_set_parent,
	.get_rate = _clk_emi_slow_get_rate,
	.set_rate = _clk_emi_slow_set_rate,
	.round_rate = _clk_emi_slow_round_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.disable = _clk_disable_inwait,
	.flags = RATE_PROPAGATES,
};

static struct clk ahbmux1_clk = {
	.id = 0,
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.disable = _clk_disable_inwait,
};

static struct clk ahbmux2_clk = {
	.id = 0,
	.parent = &ahb_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.disable = _clk_disable_inwait,
};


static struct clk emi_fast_clk = {
	.parent = &ddr_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.disable = _clk_disable_inwait,
};

static struct clk emi_intr_clk[] = {
	{
	.id = 0,
	.parent = &ahb_clk,
	.secondary = &ahbmux2_clk,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
	},
	{
	/* On MX51 - this clock is name emi_garb_clk, and controls the
	 * access of ARM to GARB.
	 */
	.id = 1,
	.parent = &ahb_clk,
	.secondary = &ahbmux2_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
	}
};

static unsigned long _clk_ipg_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_IPG_PODF_MASK) >>
	       MXC_CCM_CBCDR_IPG_PODF_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static struct clk ipg_clk = {
	.parent = &ahb_clk,
	.get_rate = _clk_ipg_get_rate,
	.flags = RATE_PROPAGATES,
};

static unsigned long _clk_ipg_per_get_rate(struct clk *clk)
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
		return clk_get_rate(clk->parent) / (prediv1 * prediv2 * podf);
	} else if (clk->parent == &ipg_clk) {
		return clk_get_rate(&ipg_clk);
	}
	BUG();
	return 0;
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
	.parent = &lp_apm_clk,
	.get_rate = _clk_ipg_per_get_rate,
	.set_parent = _clk_ipg_per_set_parent,
	.flags = RATE_PROPAGATES,
};

static int _clk_ipmux_enable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg |= 1  << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	return 0;
}

static void _clk_ipmux_disable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(0x1 << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);
}

static struct clk ipumux1_clk = {
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG6_1_OFFSET,
	.enable = _clk_ipmux_enable,
	.disable = _clk_ipmux_disable,
};

static struct clk ipumux2_clk = {
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG6_2_OFFSET,
	.enable = _clk_ipmux_enable,
	.disable = _clk_ipmux_disable,
};

static int _clk_ocram_enable(struct clk *clk)
{
	return 0;
}

static void _clk_ocram_disable(struct clk *clk)
{
}

static struct clk ocram_clk = {
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.enable = _clk_ocram_enable,
	.disable = _clk_ocram_disable,
};


static struct clk aips_tz1_clk = {
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk aips_tz2_clk = {
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk gpc_dvfs_clk = {
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static int _clk_sdma_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);

	/* Handshake with SDMA when LPM is entered. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	if (cpu_is_mx51())
		reg &= ~MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS_MX51;
	else
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
	if (cpu_is_mx51())
		reg |= MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS_MX51;
	else
		reg |= MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}


static struct clk sdma_clk[] = {
	{
	 .parent = &ahb_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	 .enable = _clk_sdma_enable,
	 .disable = _clk_sdma_disable,
	 },
	{
	 .parent = &ipg_clk,
#ifdef CONFIG_SDMA_IRAM
	 .secondary = &emi_intr_clk[0],
#endif
	 },
};

static int _clk_ipu_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);
	/* Handshake with IPU when certain clock rates are changed. */
	reg = __raw_readl(MXC_CCM_CCDR);
	if (cpu_is_mx51())
		reg &= ~MXC_CCM_CCDR_IPU_HS_MASK;
	else
		reg &= ~MXC_CCM_CCDR_IPU_HS_MX53_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	/* Handshake with IPU when LPM is entered as its enabled. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg &= ~MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	start_sdram_autogating();

	return 0;
}

static void _clk_ipu_disable(struct clk *clk)
{
	u32 reg;

	if (sdram_autogating_active())
		stop_sdram_autogating();

	_clk_disable(clk);

	/* No handshake with IPU whe dividers are changed
	 * as its not enabled. */
	reg = __raw_readl(MXC_CCM_CCDR);
	if (cpu_is_mx51())
		reg |= MXC_CCM_CCDR_IPU_HS_MASK;
	else
		reg |= MXC_CCM_CCDR_IPU_HS_MX53_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	/* No handshake with IPU when LPM is entered as its not enabled. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}

static int _clk_ipu_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &ahb_clk,
		       &emi_slow_clk);
	reg = (reg & ~MXC_CCM_CBCMR_IPU_HSP_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_IPU_HSP_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}


static struct clk ipu_clk[] = {
	{
	.parent = &ahb_clk,
	.secondary = &ipu_clk[1],
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	.enable = _clk_ipu_enable,
	.disable = _clk_ipu_disable,
	.set_parent = _clk_ipu_set_parent,
	 .flags = CPU_FREQ_TRIG_UPDATE | AHB_MED_SET_POINT | RATE_PROPAGATES,
	},
	{
	 .parent = &emi_fast_clk,
	 .secondary = &ahbmux1_clk,
	}
};

static int _clk_ipu_di_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	reg &= ~MXC_CCM_CSCMR2_DI_CLK_SEL_MASK(clk->id);
	if (parent == &pll3_sw_clk)
		;
	else if (parent == &osc_clk)
		reg |= 1 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	else if (parent == &ckih_clk)
		reg |= 2 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	else if ((parent == &pll4_sw_clk) && (clk->id == 0)) {
		if (cpu_is_mx51())
			return -EINVAL;
		reg |= 3 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	} else if ((parent == &tve_clk) && (clk->id == 1))
		reg |= 3 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	else if ((parent == &ldb_di_clk[clk->id]) && cpu_is_mx53())
		reg |= 5 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	else		/* Assume any other clock is external clock pin */
		reg |= 4 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static int priv_div;
static unsigned long _clk_ipu_di_get_rate(struct clk *clk)
{
	u32 reg, mux;
	u32 div = 1;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	mux = (reg & MXC_CCM_CSCMR2_DI_CLK_SEL_MASK(clk->id)) >>
		MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	if (mux == 0) {
		reg = __raw_readl(MXC_CCM_CDCDR) &
		    MXC_CCM_CDCDR_DI1_CLK_PRED_MASK;
		div = (reg >> MXC_CCM_CDCDR_DI1_CLK_PRED_OFFSET) + 1;
	} else if ((mux == 3) && (clk->id == 1)) {
		if (priv_div)
			div = priv_div;
	} else if ((mux == 3) && (clk->id == 0)) {
		reg = __raw_readl(MXC_CCM_CDCDR) &
			MXC_CCM_CDCDR_DI_PLL4_PODF_MASK;
		div = (reg >> MXC_CCM_CDCDR_DI_PLL4_PODF_OFFSET) + 1;
	}
	return clk_get_rate(clk->parent) / div;
}

static int _clk_ipu_di_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	if ((clk->parent == &pll4_sw_clk) && (clk->id == 0)) {
		reg = __raw_readl(MXC_CCM_CDCDR);
		reg &= ~MXC_CCM_CDCDR_DI_PLL4_PODF_MASK;
		reg |= (div - 1) << MXC_CCM_CDCDR_DI_PLL4_PODF_OFFSET;
		__raw_writel(reg, MXC_CCM_CDCDR);
	} else if (clk->parent == &pll3_sw_clk) {
		reg = __raw_readl(MXC_CCM_CDCDR);
		reg &= ~MXC_CCM_CDCDR_DI1_CLK_PRED_MASK;
		reg |= (div - 1) << MXC_CCM_CDCDR_DI1_CLK_PRED_OFFSET;
		__raw_writel(reg, MXC_CCM_CDCDR);
	} else if (((clk->parent == &tve_clk) && (clk->id == 1)) ||
		((clk->parent == &ldb_di_clk[clk->id]) && cpu_is_mx53())) {
		priv_div = div;
		return 0;
	} else
		return -EINVAL;

	return 0;
}

static unsigned long _clk_ipu_di_round_rate(struct clk *clk,
					    unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	if ((clk->parent == &ldb_di_clk[clk->id]) && cpu_is_mx53())
		return parent_rate;
	else {
		div = (parent_rate + rate/2) / rate;
		if (div > 8)
			div = 8;
		else if (div == 0)
			div++;
		return parent_rate / div;
	}
}

static struct clk ipu_di_clk[] = {
	{
	.id = 0,
	.parent = &pll3_sw_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	.get_rate = _clk_ipu_di_get_rate,
	.set_parent = _clk_ipu_di_set_parent,
	.round_rate = _clk_ipu_di_round_rate,
	.set_rate = _clk_ipu_di_set_rate,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.flags = RATE_PROPAGATES,
	},
	{
	.id = 1,
	.parent = &pll3_sw_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.get_rate = _clk_ipu_di_get_rate,
	.set_parent = _clk_ipu_di_set_parent,
	.round_rate = _clk_ipu_di_round_rate,
	.set_rate = _clk_ipu_di_set_rate,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.flags = RATE_PROPAGATES,
	},
};

static int _clk_ldb_di_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR2);

	if ((parent == &pll3_sw_clk)) {
		if (clk->id == 0)
			reg &= ~(MXC_CCM_CSCMR2_LDB_DI0_CLK_SEL);
		else
			reg &= ~(MXC_CCM_CSCMR2_LDB_DI1_CLK_SEL);
	} else if ((parent == &pll4_sw_clk)) {
		if (clk->id == 0)
			reg |= MXC_CCM_CSCMR2_LDB_DI0_CLK_SEL;
		else
			reg |= MXC_CCM_CSCMR2_LDB_DI1_CLK_SEL;
	} else {
		BUG();
	}

	__raw_writel(reg, MXC_CCM_CSCMR2);
	return 0;
}

static unsigned long _clk_ldb_di_get_rate(struct clk *clk)
{
	u32 div;

	if (clk->id == 0)
		div = __raw_readl(MXC_CCM_CSCMR2) &
			MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	else
		div = __raw_readl(MXC_CCM_CSCMR2) &
			MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;

	if (div)
		return clk_get_rate(clk->parent) / 7;

	return (2 * clk_get_rate(clk->parent)) / 7;
}

static unsigned long _clk_ldb_di_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 parent_rate = clk_get_rate(clk->parent);

	if (rate * 7 <= parent_rate + parent_rate/20)
		return parent_rate / 7;
	else
		return 2 * parent_rate / 7;
}

static int _clk_ldb_di_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div = 0;
	u32 parent_rate = clk_get_rate(clk->parent);

	if (rate * 7 <= parent_rate + parent_rate/20) {
		div = 7;
		rate = parent_rate / 7;
	} else
		rate = 2 * parent_rate / 7;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	if (div == 7)
		reg |= (clk->id ? MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV :
			MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV);
	else
		reg &= ~(clk->id ? MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV :
			MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV);
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static int _clk_ldb_di_enable(struct clk *clk)
{
	_clk_enable(clk);
	ipu_di_clk[clk->id].set_parent(&ipu_di_clk[clk->id], clk);
	ipu_di_clk[clk->id].parent = clk;
	ipu_di_clk[clk->id].enable(&ipu_di_clk[clk->id]);
	ipu_di_clk[clk->id].usecount++;
	return 0;
}

static void _clk_ldb_di_disable(struct clk *clk)
{
	_clk_disable(clk);
	ipu_di_clk[clk->id].disable(&ipu_di_clk[clk->id]);
	ipu_di_clk[clk->id].usecount--;
}

static struct clk ldb_di_clk[] = {
	{
	.id = 0,
	.parent = &pll4_sw_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.get_rate = _clk_ldb_di_get_rate,
	.set_parent = _clk_ldb_di_set_parent,
	.round_rate = _clk_ldb_di_round_rate,
	.set_rate = _clk_ldb_di_set_rate,
	.enable = _clk_ldb_di_enable,
	.disable = _clk_ldb_di_disable,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT,
	},
	{
	.id = 1,
	.parent = &pll4_sw_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.get_rate = _clk_ldb_di_get_rate,
	.set_parent = _clk_ldb_di_set_parent,
	.round_rate = _clk_ldb_di_round_rate,
	.set_rate = _clk_ldb_di_set_rate,
	.enable = _clk_ldb_di_enable,
	.disable = _clk_ldb_di_disable,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT,
	},
};

static int _clk_csi0_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk, NULL);
	reg = (reg & ~MXC_CCM_CSCMR2_CSI_MCLK1_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR2_CSI_MCLK1_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static unsigned long _clk_csi0_get_rate(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CSCDR4);
	pred = ((reg & MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_OFFSET) + 1;
	return clk_get_rate(clk->parent)/(pred * podf);
}

static unsigned long _clk_csi0_round_rate(struct clk *clk, unsigned long rate)
{
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;
	if (parent_rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return parent_rate / (pre * post);
}

static int _clk_csi0_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	if ((parent_rate / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	/* Set CSI clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR4) &
	    ~(MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_MASK |
		MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR4);

	return 0;
}

static struct clk csi0_clk = {
	.parent = &pll3_sw_clk,
	.set_parent = _clk_csi0_set_parent,
	.get_rate = _clk_csi0_get_rate,
	.round_rate = _clk_csi0_round_rate,
	.set_rate = _clk_csi0_set_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.disable = _clk_disable,
};

static int _clk_csi1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk, NULL);
	reg = (reg & ~MXC_CCM_CSCMR2_CSI_MCLK2_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR2_CSI_MCLK2_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static unsigned long _clk_csi1_get_rate(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CSCDR4);
	pred = ((reg & MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_OFFSET) + 1;
	return clk_get_rate(clk->parent)/(pred * podf) ;
}

static unsigned long _clk_csi1_round_rate(struct clk *clk, unsigned long rate)
{
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;
	if (parent_rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return parent_rate / (pre * post);
}

static int _clk_csi1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	if ((parent_rate / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	/* Set CSI clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR4) &
	    ~(MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_MASK |
		MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR4);

	return 0;
}

static struct clk csi1_clk = {
	.parent = &pll3_sw_clk,
	.set_parent = _clk_csi1_set_parent,
	.get_rate = _clk_csi1_get_rate,
	.round_rate = _clk_csi1_round_rate,
	.set_rate = _clk_csi1_set_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.disable = _clk_disable,
};


static int _clk_hsc_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);
	/* Handshake with IPU when certain clock rates are changed. */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg &= ~MXC_CCM_CCDR_HSC_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	reg = __raw_readl(MXC_CCM_CLPCR);
	reg &= ~MXC_CCM_CLPCR_BYPASS_HSC_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static void _clk_hsc_disable(struct clk *clk)
{
	u32 reg;

	_clk_disable(clk);
	/* No handshake with HSC as its not enabled. */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_HSC_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_HSC_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}

static struct clk mipi_esc_clk = {
	.parent = &pll2_sw_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
};

static struct clk mipi_hsc2_clk = {
	.parent = &pll2_sw_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.secondary = &mipi_esc_clk,
};

static struct clk mipi_hsc1_clk = {
	.parent = &pll2_sw_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.secondary = &mipi_hsc2_clk,
};

static struct clk mipi_hsp_clk = {
	.parent = &ipu_clk[0],
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.enable = _clk_hsc_enable,
	.disable = _clk_hsc_disable,
	.secondary = &mipi_hsc1_clk,
};

static int _clk_tve_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);

	if ((parent == &pll3_sw_clk) && cpu_is_mx51()) {
		reg &= ~(MXC_CCM_CSCMR1_TVE_CLK_SEL);
	} else if ((parent == &pll4_sw_clk) && cpu_is_mx53()) {
		reg &= ~(MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL);
	} else if ((parent == &osc_clk) && cpu_is_mx51()) {
		reg |= MXC_CCM_CSCMR1_TVE_CLK_SEL;
		reg &= ~MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL;
	} else if (parent == &ckih_clk) {
		reg |= MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL;
		reg |= MXC_CCM_CSCMR1_TVE_CLK_SEL; /* Reserved on MX53 */
	} else {
		BUG();
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static unsigned long _clk_tve_get_rate(struct clk *clk)
{
	u32 reg, div = 1;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & (MXC_CCM_CSCMR1_TVE_CLK_SEL | MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL)) == 0) {
		reg = __raw_readl(MXC_CCM_CDCDR) &
		    MXC_CCM_CDCDR_TVE_CLK_PRED_MASK;
		div = (reg >> MXC_CCM_CDCDR_TVE_CLK_PRED_OFFSET) + 1;
	}
	return clk_get_rate(clk->parent) / div;
}

static unsigned long _clk_tve_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (cpu_is_mx51() && (reg & MXC_CCM_CSCMR1_TVE_CLK_SEL))
		return -EINVAL;
	if (cpu_is_mx53() && (reg & MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL))
		return -EINVAL;

	div = (parent_rate + rate/2) / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return parent_rate / div;
}

static int _clk_tve_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (cpu_is_mx51() && (reg & MXC_CCM_CSCMR1_TVE_CLK_SEL))
		return -EINVAL;
	if (cpu_is_mx53() && (reg & MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL))
		return -EINVAL;

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	div--;
	reg = __raw_readl(MXC_CCM_CDCDR) & ~MXC_CCM_CDCDR_TVE_CLK_PRED_MASK;
	reg |= div << MXC_CCM_CDCDR_TVE_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CDCDR);
	return 0;
}

static struct clk tve_clk = {
	.parent = &pll3_sw_clk,
	.set_parent = _clk_tve_set_parent,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.get_rate = _clk_tve_get_rate,
	.round_rate = _clk_tve_round_rate,
	.set_rate = _clk_tve_set_rate,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk spba_clk = {
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static unsigned long _clk_uart_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_UART_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_UART_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent)/(prediv * podf) ;
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
	.parent = &pll2_sw_clk,
	.get_rate = _clk_uart_get_rate,
	.set_parent = _clk_uart_set_parent,
	.flags = RATE_PROPAGATES,
};

static struct clk uart1_clk[] = {
	{
	 .id = 0,
	 .parent = &uart_main_clk,
	 .secondary = &uart1_clk[1],
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#if UART1_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .id = 0,
	 .parent = &ipg_clk,
#if UART1_DMA_ENABLE
	 .secondary = &aips_tz1_clk,
#endif
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart2_clk[] = {
	{
	 .id = 1,
	 .parent = &uart_main_clk,
	 .secondary = &uart2_clk[1],
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#if UART2_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .id = 1,
	 .parent = &ipg_clk,
#if UART2_DMA_ENABLE
	 .secondary = &aips_tz1_clk,
#endif
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart3_clk[] = {
	{
	 .id = 2,
	 .parent = &uart_main_clk,
	 .secondary = &uart3_clk[1],
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#if UART3_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .id = 2,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart4_clk[] = {
	{
	 .id = 3,
	 .parent = &uart_main_clk,
	 .secondary = &uart4_clk[1],
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#if UART4_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .id = 3,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart5_clk[] = {
	{
	 .id = 4,
	 .parent = &uart_main_clk,
	 .secondary = &uart5_clk[1],
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#if UART5_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .id = 4,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk gpt_clk[] = {
	{
	 .parent = &ipg_perclk,
	 .id = 0,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &gpt_clk[1],
	 },
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 0,
	 .parent = &ckil_clk,
	 },
};

static struct clk pwm1_clk[] = {
	{
	 .parent = &ipg_perclk,
	 .id = 0,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &pwm1_clk[1],
	 },
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
	 .disable = _clk_disable,
	 },
	{
	 .id = 0,
	 .parent = &ckil_clk,
	 },
};

static struct clk pwm2_clk[] = {
	{
	 .parent = &ipg_perclk,
	 .id = 1,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &pwm2_clk[1],
	 },
	{
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
	 .disable = _clk_disable,
	 },
	{
	 .id = 1,
	 .parent = &ckil_clk,
	 },
};

static struct clk i2c_clk[] = {
	{
	 .id = 0,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 1,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 2,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static unsigned long _clk_hsi2c_serial_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR3);
	prediv = ((reg & MXC_CCM_CSCDR3_HSI2C_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR3_HSI2C_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR3_HSI2C_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR3_HSI2C_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
}

static struct clk hsi2c_serial_clk = {
	.id = 0,
	.parent = &pll3_sw_clk,
	.secondary = &spba_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.get_rate = _clk_hsi2c_serial_get_rate,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk hsi2c_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static unsigned long _clk_cspi_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR2);
	prediv = ((reg & MXC_CCM_CSCDR2_CSPI_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR2_CSPI_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CSCDR2_CSPI_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR2_CSPI_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
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
	.parent = &pll3_sw_clk,
	.get_rate = _clk_cspi_get_rate,
	.set_parent = _clk_cspi_set_parent,
	.flags = RATE_PROPAGATES,
};

static struct clk cspi1_clk[] = {
	{
	 .id = 0,
	 .parent = &cspi_main_clk,
	 .secondary = &cspi1_clk[1],
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
	 .disable = _clk_disable,
	 },
};

static struct clk cspi2_clk[] = {
	{
	 .id = 1,
	 .parent = &cspi_main_clk,
	 .secondary = &cspi2_clk[1],
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &aips_tz2_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
	 .disable = _clk_disable,
	 },
};

static struct clk cspi3_clk = {
	.id = 2,
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &aips_tz2_clk,
};

static unsigned long _clk_ieee_rtc_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR2);
	prediv = ((reg & MXC_CCM_CSCDR2_IEEE_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR2_IEEE_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CSCDR2_IEEE_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR2_IEEE_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
}

static int _clk_ieee_rtc_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || div > 512)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CSCDR2);
	reg &= ~(MXC_CCM_CSCDR2_IEEE_CLK_PRED_MASK |
		 MXC_CCM_CSCDR2_IEEE_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR2_IEEE_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR2_IEEE_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	return 0;
}

static unsigned long _clk_ieee_rtc_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (parent_rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return parent_rate / (pre * post);
}

static int _clk_ieee_rtc_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll3_sw_clk, &pll4_sw_clk,
		       NULL, NULL);
	reg = __raw_readl(MXC_CCM_CSCMR2) & ~MXC_CCM_CSCMR2_IEEE_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR2_IEEE_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static struct clk ieee_rtc_clk = {
	.id = 0,
	.parent = &pll3_sw_clk,
	.set_parent = _clk_ieee_rtc_set_parent,
	.set_rate = _clk_ieee_rtc_set_rate,
	.round_rate = _clk_ieee_rtc_round_rate,
	.get_rate = _clk_ieee_rtc_get_rate,
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
	.parent = &ckih_clk,
	.set_parent = _clk_ssi_lp_apm_set_parent,
};

static unsigned long _clk_ssi1_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CS1CDR);
	prediv = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK) >>
		  MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK) >>
		MXC_CCM_CS1CDR_SSI1_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
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
	 .id = 0,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_ssi1_set_parent,
	 .secondary = &ssi1_clk[1],
	 .get_rate = _clk_ssi1_get_rate,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &ssi1_clk[2],
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 0,
	 .parent = &aips_tz2_clk,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	 .secondary = &emi_intr_clk[0],
#else
	 .secondary = &emi_fast_clk,
#endif
	 },
};

static unsigned long _clk_ssi2_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CS2CDR);
	prediv = ((reg & MXC_CCM_CS2CDR_SSI2_CLK_PRED_MASK) >>
		  MXC_CCM_CS2CDR_SSI2_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CS2CDR_SSI2_CLK_PODF_MASK) >>
		MXC_CCM_CS2CDR_SSI2_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
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
	 .id = 1,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_ssi2_set_parent,
	 .secondary = &ssi2_clk[1],
	 .get_rate = _clk_ssi2_get_rate,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &ssi2_clk[2],
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 1,
	 .parent = &spba_clk,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	 .secondary = &emi_intr_clk[0],
#else
	 .secondary = &emi_fast_clk,
#endif
	 },
};

static int _clk_ssi3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_SSI3_CLK_SEL;

	if (parent == &ssi1_clk[0])
		reg &= ~MXC_CCM_CSCMR1_SSI3_CLK_SEL;
	else if (parent == &ssi2_clk[0])
		reg |= MXC_CCM_CSCMR1_SSI3_CLK_SEL;
	else {
		printk(KERN_ERR"Set ssi3 clock parent failed!\n");
		printk(KERN_ERR"ssi3 only support");
		printk(KERN_ERR"ssi1 and ssi2 as parent clock\n");
		return -1;
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static struct clk ssi3_clk[] = {
	{
	 .id = 2,
	 .parent = &ssi1_clk[0],
	 .set_parent = _clk_ssi3_set_parent,
	 .secondary = &ssi3_clk[1],
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 2,
	 .parent = &ipg_clk,
	 .secondary = &ssi3_clk[2],
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 2,
	 .parent = &aips_tz2_clk,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	 .secondary = &emi_intr_clk,
#else
	 .secondary = &emi_fast_clk,
#endif
	 },
};

static unsigned long _clk_ssi_ext1_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;
	u32 div = 1;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & MXC_CCM_CSCMR1_SSI_EXT1_COM_CLK_SEL) == 0) {
		reg = __raw_readl(MXC_CCM_CS1CDR);
		prediv = ((reg & MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_MASK) >>
			  MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_MASK) >>
			MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_OFFSET) + 1;
		div = prediv * podf;
	}
	return clk_get_rate(clk->parent) / div;
}

static int _clk_ssi_ext1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || div > 512)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg &= ~(MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_MASK |
		 MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CS1CDR);

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
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (parent_rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return parent_rate / (pre * post);
}

static struct clk ssi_ext1_clk = {
	.parent = &pll3_sw_clk,
	.set_parent = _clk_ssi_ext1_set_parent,
	.set_rate = _clk_ssi_ext1_set_rate,
	.round_rate = _clk_ssi_ext1_round_rate,
	.get_rate = _clk_ssi_ext1_get_rate,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static unsigned long _clk_ssi_ext2_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;
	u32 div = 1;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & MXC_CCM_CSCMR1_SSI_EXT2_COM_CLK_SEL) == 0) {
		reg = __raw_readl(MXC_CCM_CS2CDR);
		prediv = ((reg & MXC_CCM_CS2CDR_SSI_EXT2_CLK_PRED_MASK) >>
			  MXC_CCM_CS2CDR_SSI_EXT2_CLK_PRED_OFFSET) + 1;
		if (prediv == 1)
			BUG();
		podf = ((reg & MXC_CCM_CS2CDR_SSI_EXT2_CLK_PODF_MASK) >>
			MXC_CCM_CS2CDR_SSI_EXT2_CLK_PODF_OFFSET) + 1;
		div = prediv * podf;
	}
	return clk_get_rate(clk->parent) / div;
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
	.parent = &pll3_sw_clk,
	.set_parent = _clk_ssi_ext2_set_parent,
	.get_rate = _clk_ssi_ext2_get_rate,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static int _clk_esai_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	if (parent ==  &pll1_sw_clk || parent ==  &pll2_sw_clk ||
		    parent ==  &pll3_sw_clk) {
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			NULL);
		reg &= ~MXC_CCM_CSCMR2_ESAI_PRE_SEL_MASK;
		reg |= mux << MXC_CCM_CSCMR2_ESAI_PRE_SEL_OFFSET;
		reg &= ~MXC_CCM_CSCMR2_ESAI_POST_SEL_MASK;
		reg |= 0 << MXC_CCM_CSCMR2_ESAI_POST_SEL_OFFSET;
		/* divider setting */
	} else {
		mux = _get_mux(parent, &ssi1_clk[0], &ssi2_clk[0], &ckih_clk,
			&ckih2_clk);
		reg &= ~MXC_CCM_CSCMR2_ESAI_POST_SEL_MASK;
		reg |= (mux + 1) << MXC_CCM_CSCMR2_ESAI_POST_SEL_OFFSET;
		/* divider setting */
	}

	__raw_writel(reg, MXC_CCM_CSCMR2);

	/* set podf = 0 */
	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg &= ~MXC_CCM_CS1CDR_ESAI_CLK_PODF_MASK;
	__raw_writel(reg, MXC_CCM_CS1CDR);

	return 0;
}

static unsigned long _clk_esai_get_rate(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CS1CDR);
	if (clk->parent ==  &pll1_sw_clk || clk->parent ==  &pll2_sw_clk ||
		    clk->parent ==  &pll3_sw_clk) {
		pred = ((reg & MXC_CCM_CS1CDR_ESAI_CLK_PRED_MASK) >>
			MXC_CCM_CS1CDR_ESAI_CLK_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CS1CDR_ESAI_CLK_PODF_MASK) >>
			MXC_CCM_CS1CDR_ESAI_CLK_PODF_OFFSET) + 1;

		return clk_get_rate(clk->parent) / (pred * podf);
	} else {
		podf = ((reg & MXC_CCM_CS1CDR_ESAI_CLK_PODF_MASK) >>
			MXC_CCM_CS1CDR_ESAI_CLK_PODF_OFFSET) + 1;

		return clk_get_rate(clk->parent) / podf;
	}
}

static struct clk esai_clk[] = {
	{
	 .id = 0,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_esai_set_parent,
	 .get_rate = _clk_esai_get_rate,
	 .secondary = &esai_clk[1],
	 .enable_reg = MXC_CCM_CCGR6,
	 .enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR6,
	 .enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk iim_clk = {
	.parent = &ipg_clk,
	.secondary = &aips_tz2_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.disable = _clk_disable,
};

static struct clk tmax1_clk = {
	 .id = 0,
	 .parent = &ahb_clk,
	 .secondary = &ahb_max_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	 .disable = _clk_disable,
	 };

static struct clk tmax2_clk = {
	 .id = 0,
	 .parent = &ahb_clk,
	 .secondary = &ahb_max_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	 .disable = _clk_disable,
};

static struct clk tmax3_clk = {
	 .id = 0,
	 .parent = &ahb_clk,
	 .secondary = &ahb_max_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	 .disable = _clk_disable,
};

static unsigned long _clk_usboh3_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_USBOH3_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_USBOH3_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CSCDR1_USBOH3_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_USBOH3_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
}

static int _clk_usboh3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_USBOH3_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_USBOH3_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk usboh3_clk[] = {
	{
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_usboh3_set_parent,
	 .get_rate = _clk_usboh3_get_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &usboh3_clk[1],
	 .flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	 },
	{
	 .parent = &tmax2_clk,
#if defined(CONFIG_USB_STATIC_IRAM) \
    || defined(CONFIG_USB_STATIC_IRAM_PPH)
	 .secondary = &emi_intr_clk[0],
#else
	 .secondary = &emi_fast_clk,
#endif
	 },
};

static struct clk usb_ahb_clk = {
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	 .disable = _clk_disable,
};

static unsigned long _clk_usb_phy_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;
	u32 div = 1;

	if (clk->parent == &pll3_sw_clk) {
		reg = __raw_readl(MXC_CCM_CDCDR);
		prediv = ((reg & MXC_CCM_CDCDR_USB_PHY_PRED_MASK) >>
			  MXC_CCM_CDCDR_USB_PHY_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CDCDR_USB_PHY_PODF_MASK) >>
			MXC_CCM_CDCDR_USB_PHY_PODF_OFFSET) + 1;

		div = (prediv * podf);
	}

	return clk_get_rate(clk->parent) / div;
}

static int _clk_usb_phy_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &osc_clk)
		reg &= ~MXC_CCM_CSCMR1_USB_PHY_CLK_SEL;
	else if (parent == &pll3_sw_clk)
		reg |= MXC_CCM_CSCMR1_USB_PHY_CLK_SEL;
	else
		BUG();

	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static struct clk usb_phy_clk[] = {
	{
	.id = 0,
	.parent = &pll3_sw_clk,
	.secondary = &tmax3_clk,
	.set_parent = _clk_usb_phy_set_parent,
	.get_rate = _clk_usb_phy_get_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.disable = _clk_disable,
	},
	{
	.id = 1,
	.parent = &pll3_sw_clk,
	.secondary = &tmax3_clk,
	.set_parent = _clk_usb_phy_set_parent,
	.get_rate = _clk_usb_phy_get_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.disable = _clk_disable,
	}
};

static struct clk esdhc_dep_clks = {
	 .parent = &spba_clk,
	 .secondary = &emi_fast_clk,
};

static unsigned long _clk_esdhc1_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
}

static int _clk_esdhc1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) &
	    ~MXC_CCM_CSCMR1_ESDHC1_MSHC2_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_ESDHC1_MSHC2_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}


static int _clk_sdhc1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	if ((parent_rate / div) != rate)
		return -EINVAL;

	 __calc_pre_post_dividers(div, &pre, &post);

	/* Set sdhc1 clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR1) &
		~(MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_MASK |
		MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}

static struct clk esdhc1_clk[] = {
	{
	 .id = 0,
	 .parent = &pll2_sw_clk,
	 .set_parent = _clk_esdhc1_set_parent,
	 .get_rate = _clk_esdhc1_get_rate,
	 .set_rate = _clk_sdhc1_set_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc1_clk[1],
	 },
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &esdhc1_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .id = 0,
	 .parent = &tmax3_clk,
	 .secondary = &esdhc_dep_clks,
	 },

};

static unsigned long _clk_esdhc2_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	if (cpu_is_mx51()) {
		reg = __raw_readl(MXC_CCM_CSCDR1);
		prediv = ((reg & MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_MASK) >>
			MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_MASK) >>
			MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_OFFSET) + 1;

		return clk_get_rate(clk->parent) / (prediv * podf);
	}
	return clk_get_rate(clk->parent);
}

static int _clk_esdhc2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	if (cpu_is_mx51()) {
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &lp_apm_clk);
		reg = __raw_readl(MXC_CCM_CSCMR1) &
		    ~MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_MASK;
		reg |= mux << MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_OFFSET;
	} else { /* MX53  */
		reg = __raw_readl(MXC_CCM_CSCMR1);
		if (parent == &esdhc1_clk[0])
			reg &= ~MXC_CCM_CSCMR1_ESDHC2_CLK_SEL;
		else if (parent == &esdhc3_clk[0])
			reg |= MXC_CCM_CSCMR1_ESDHC2_CLK_SEL;
		else
			BUG();
	}
	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static int _clk_esdhc2_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	if (cpu_is_mx51()) {
		div = parent_rate / rate;

		if ((parent_rate / div) != rate)
			return -EINVAL;

		 __calc_pre_post_dividers(div, &pre, &post);

		/* Set sdhc1 clock divider */
		reg = __raw_readl(MXC_CCM_CSCDR1) &
			~(MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_MASK |
			MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_MASK);
		reg |= (post - 1) <<
				MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_OFFSET;
		reg |= (pre - 1) <<
				MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_OFFSET;
		__raw_writel(reg, MXC_CCM_CSCDR1);
	}
	return 0;
}

static struct clk esdhc2_clk[] = {
	{
	 .id = 1,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_esdhc2_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc2_clk[1],
	 },
	{
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &esdhc2_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .id = 0,
	 .parent = &tmax2_clk,
	 .secondary = &esdhc_dep_clks,
	 },
};

static int _clk_esdhc3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	if (cpu_is_mx51()) {
		reg = __raw_readl(MXC_CCM_CSCMR1);
		if (parent == &esdhc1_clk[0])
			reg &= ~MXC_CCM_CSCMR1_ESDHC3_CLK_SEL_MX51;
		else if (parent == &esdhc2_clk[0])
			reg |= MXC_CCM_CSCMR1_ESDHC3_CLK_SEL_MX51;
		else
			BUG();
	} else { /* MX53 */
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &lp_apm_clk);
		reg = __raw_readl(MXC_CCM_CSCMR1) &
		    ~MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_MASK;
		reg |= mux << MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_OFFSET;
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static unsigned long _clk_esdhc3_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
}

static int _clk_sdhc3_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	if (cpu_is_mx53()) {
		div = parent_rate / rate;

		if ((parent_rate / div) != rate)
			return -EINVAL;

		__calc_pre_post_dividers(div, &pre, &post);

		/* Set sdhc1 clock divider */
		reg = __raw_readl(MXC_CCM_CSCDR1) &
			~(MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_MASK |
			MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_MASK);
		reg |= (post - 1) << MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_OFFSET;
		reg |= (pre - 1) << MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_OFFSET;
		 __raw_writel(reg, MXC_CCM_CSCDR1);
	}
	return 0;
}


static struct clk esdhc3_clk[] = {
	{
	 .id = 2,
	 .parent = &esdhc1_clk[0],
	 .set_parent = _clk_esdhc3_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc3_clk[1],
	 },
	{
	 .id = 2,
	 .parent = &ipg_clk,
	 .secondary = &esdhc3_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .id = 0,
	 .parent = &ahb_max_clk,
	 .secondary = &esdhc_dep_clks,
	 },
};

static int _clk_esdhc4_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	if (cpu_is_mx51()) {
		reg = __raw_readl(MXC_CCM_CSCMR1);
		if (parent == &esdhc1_clk[0])
			reg &= ~MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
		else if (parent == &esdhc2_clk[0])
			reg |= MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
		else
			BUG();
	} else {/*MX53 */
		reg = __raw_readl(MXC_CCM_CSCMR1);
		if (parent == &esdhc1_clk[0])
			reg &= ~MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
		else if (parent == &esdhc3_clk[0])
			reg |= MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
		else
			BUG();
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk esdhc4_clk[] = {
	{
	 .id = 3,
	 .parent = &esdhc1_clk[0],
	 .set_parent = _clk_esdhc4_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc4_clk[1],
	 },
	{
	 .id = 3,
	 .parent = &ipg_clk,
	 .secondary = &esdhc4_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .id = 0,
	 .parent = &tmax3_clk,
	 .secondary = &esdhc_dep_clks,
	 },
};

static struct clk sata_clk = {
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.disable = _clk_disable,
};

static struct clk ieee_1588_clk = {
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.disable = _clk_disable,
};

static struct clk mlb_clk[] = {
	{
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.disable = _clk_disable,
	.secondary = &mlb_clk[1],
	},
	{
	.parent = &emi_fast_clk,
	.secondary = &emi_intr_clk[1],
	},
};

static int _can_root_clk_set(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &ipg_clk, &ckih_clk, &ckih2_clk, &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR2) & ~MXC_CCM_CSCMR2_CAN_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR2_CAN_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static struct clk can1_clk[] = {
	{
	.id = 0,
	.parent = &lp_apm_clk,
	.set_parent = _can_root_clk_set,
	.enable = _clk_enable,
	.secondary = &can1_clk[1],
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.disable = _clk_disable,
	 },
	{
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.disable = _clk_disable,
	 },
};

static struct clk can2_clk[] = {
	{
	.id = 1,
	.parent = &lp_apm_clk,
	.set_parent = _can_root_clk_set,
	.enable = _clk_enable,
	.secondary = &can2_clk[1],
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.disable = _clk_disable,
	 },
	{
	.id = 1,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.disable = _clk_disable,
	 },
};

static int _clk_sim_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk, NULL);
	reg = __raw_readl(MXC_CCM_CSCMR2) & ~MXC_CCM_CSCMR2_SIM_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR2_SIM_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static unsigned long _clk_sim_get_rate(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CSCDR2);
	pred = ((reg & MXC_CCM_CSCDR2_SIM_CLK_PRED_MASK) >>
		MXC_CCM_CSCDR2_SIM_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR2_SIM_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR2_SIM_CLK_PODF_OFFSET) + 1;
	return clk_get_rate(clk->parent) / (pred * podf);
}

static unsigned long _clk_sim_round_rate(struct clk *clk, unsigned long rate)
{
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;
	if (parent_rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return parent_rate / (pre * post);
}

static int _clk_sim_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	if ((parent_rate / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	/* Set SIM clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR2) &
	    ~(MXC_CCM_CSCDR2_SIM_CLK_PRED_MASK |
	      MXC_CCM_CSCDR2_SIM_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR2_SIM_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR2_SIM_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	return 0;

}

static struct clk sim_clk[] = {
	{
	.parent = &pll3_sw_clk,
	.set_parent = _clk_sim_set_parent,
	.secondary = &sim_clk[1],
	.get_rate = _clk_sim_get_rate,
	.round_rate = _clk_sim_round_rate,
	.set_rate = _clk_sim_set_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	 },
	{
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.disable = _clk_disable,
	 },
};

static unsigned long _clk_nfc_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_NFC_PODF_MASK) >>
	       MXC_CCM_CBCDR_NFC_PODF_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static unsigned long _clk_nfc_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	/*
	 * Compute the divider we'd have to use to reach the target rate.
	 */

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (parent_rate / div > MAX_NFC_CLK)
		div++;

	/*
	 * The divider for this clock is 3 bits wide, so we can't possibly
	 * divide the parent by more than eight.
	 */

	if (div > 8)
		return -EINVAL;

	return parent_rate / div;

}

static int _clk_nfc_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);


	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_NFC_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_NFC_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);
	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) &
			MXC_CCM_CDHIPR_NFC_IPG_INT_MEM_PODF_BUSY){
		getnstimeofday(&curtime);
		if ((curtime.tv_nsec - nstimeofday.tv_nsec) > SPIN_DELAY)
			panic("_clk_nfc_set_rate failed\n");
	}
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static struct clk emi_enfc_clk = {
	.parent = &emi_slow_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.disable = _clk_disable_inwait,
	.get_rate = _clk_nfc_get_rate,
	.round_rate = _clk_nfc_round_rate,
	.set_rate = _clk_nfc_set_rate,
};

static int _clk_spdif_xtal_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &osc_clk, &ckih_clk, &ckih2_clk, NULL);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_SPDIF_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_SPDIF_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk spdif_xtal_clk = {
	.parent = &osc_clk,
	.set_parent = _clk_spdif_xtal_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
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

static unsigned long _clk_spdif0_get_rate(struct clk *clk)
{
	u32 reg, pred, podf;
	u32 div = 1;

	if (clk->parent != &ssi1_clk[0]) {
		reg = __raw_readl(MXC_CCM_CDCDR);
		pred = ((reg & MXC_CCM_CDCDR_SPDIF0_CLK_PRED_MASK) >>
			MXC_CCM_CDCDR_SPDIF0_CLK_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CDCDR_SPDIF0_CLK_PODF_MASK) >>
			MXC_CCM_CDCDR_SPDIF0_CLK_PODF_OFFSET) + 1;
		div = (pred * podf);
	}
	return clk_get_rate(clk->parent) / div;
}

static struct clk spdif0_clk[] = {
	{
	.id = 0,
	.parent = &pll3_sw_clk,
	.set_parent = _clk_spdif0_set_parent,
	.get_rate = _clk_spdif0_get_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
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

static unsigned long _clk_spdif1_get_rate(struct clk *clk)
{
	u32 reg, pred, podf;
	u32 div = 1;

	if (clk->parent != &ssi2_clk[0]) {
		reg = __raw_readl(MXC_CCM_CDCDR);
		pred = ((reg & MXC_CCM_CDCDR_SPDIF1_CLK_PRED_MASK) >>
			MXC_CCM_CDCDR_SPDIF1_CLK_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CDCDR_SPDIF1_CLK_PODF_MASK) >>
			MXC_CCM_CDCDR_SPDIF1_CLK_PODF_OFFSET) + 1;
		div = (pred * podf);
	}
	return clk_get_rate(clk->parent) / div;
}

static struct clk spdif1_clk[] = {
	{
	.id = 1,
	.parent = &pll3_sw_clk,
	.set_parent = _clk_spdif1_set_parent,
	.get_rate = _clk_spdif1_get_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	 .disable = _clk_disable,
	 },
};

static int _clk_ddr_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, reg2, mux;
	struct timespec nstimeofday;
	struct timespec curtime;

	reg = __raw_readl(MXC_CCM_CBCMR);
	reg2 = __raw_readl(MXC_CCM_CBCDR);
	if (cpu_is_mx51()) {
		clk->parent = &ddr_hf_clk;
		mux = _get_mux_ddr(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk, &ddr_hf_clk);
	} else {
		clk->parent = &axi_a_clk;
		mux = _get_mux_ddr(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk, NULL);
	}
	if (mux < 4) {
		reg = (reg & ~MXC_CCM_CBCMR_DDR_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CBCMR_DDR_CLK_SEL_OFFSET);
		__raw_writel(reg, MXC_CCM_CBCMR);
		if (cpu_is_mx51())
			reg2 = (reg2 & ~MXC_CCM_CBCDR_DDR_HF_SEL);
	} else {
		reg2 = (reg2 & ~MXC_CCM_CBCDR_DDR_HF_SEL) |
			(MXC_CCM_CBCDR_DDR_HF_SEL);
	}
	if (cpu_is_mx51()) {
		__raw_writel(reg2, MXC_CCM_CBCDR);
		getnstimeofday(&nstimeofday);
		while (__raw_readl(MXC_CCM_CDHIPR) &
			MXC_CCM_CDHIPR_DDR_HF_CLK_SEL_BUSY){
			getnstimeofday(&curtime);
			if ((curtime.tv_nsec - nstimeofday.tv_nsec) > SPIN_DELAY)
				panic("_clk_ddr_set_parent failed\n");
		}
	}
	return 0;
}

static struct clk ddr_clk = {
	.parent = &axi_b_clk,
	.set_parent = _clk_ddr_set_parent,
	.flags = RATE_PROPAGATES,
};

static int _clk_arm_axi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk);
	reg = (reg & ~MXC_CCM_CBCMR_ARM_AXI_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_ARM_AXI_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk arm_axi_clk = {
	.parent = &axi_a_clk,
	.set_parent = _clk_arm_axi_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.disable = _clk_disable,
};

static int _clk_vpu_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk);
	reg = (reg & ~MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static int _clk_vpu_enable(struct clk *clk)
{
	/* Set VPU's parent to be axi_a or ahb when its enabled. */
	if (cpu_is_mx51_rev(CHIP_REV_2_0) < 0) {
		clk_set_parent(&vpu_clk[0], &ahb_clk);
		clk_set_parent(&vpu_clk[1], &ahb_clk);
	} else if (cpu_is_mx51()) {
		clk_set_parent(&vpu_clk[0], &axi_a_clk);
		clk_set_parent(&vpu_clk[1], &axi_a_clk);
	}

	return _clk_enable(clk);

}

static void _clk_vpu_disable(struct clk *clk)
{
	_clk_disable(clk);

	/* Set VPU's parent to be axi_b when its disabled. */
	if (cpu_is_mx51()) {
		clk_set_parent(&vpu_clk[0], &axi_b_clk);
		clk_set_parent(&vpu_clk[1], &axi_b_clk);
	}
}

static struct clk vpu_clk[] = {
	{
	 .set_parent = _clk_vpu_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &vpu_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	 },
	{
	 .set_parent = _clk_vpu_set_parent,
	 .enable = _clk_vpu_enable,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	 .disable = _clk_vpu_disable,
	 .secondary = &vpu_clk[2],
	 },
	{
	 .parent = &emi_fast_clk,
#ifdef CONFIG_MXC_VPU_IRAM
	 .secondary = &emi_intr_clk[0],
#endif
	 }
};

static int _clk_lpsr_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CLPCR);
	mux = _get_mux(parent, &ckil_clk, &osc_clk, NULL, NULL);
	reg = (reg & ~MXC_CCM_CLPCR_LPSR_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CLPCR_LPSR_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static struct clk lpsr_clk = {
	.parent = &ckil_clk,
	.set_parent = _clk_lpsr_set_parent,
};

static unsigned long _clk_pgc_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	div = (reg & MXC_CCM_CSCDR1_PGC_CLK_PODF_MASK) >>
	    MXC_CCM_CSCDR1_PGC_CLK_PODF_OFFSET;
	div = 1 >> div;
	return clk_get_rate(clk->parent) / div;
}

static struct clk pgc_clk = {
	.parent = &ipg_clk,
	.get_rate = _clk_pgc_get_rate,
};

static unsigned long _clk_usb_get_rate(struct clk *clk)
{
	return 60000000;
}

/*usb OTG clock */
static struct clk usb_clk = {
	.get_rate = _clk_usb_get_rate,
};

static struct clk usb_utmi_clk = {
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CSCMR1,
	.enable_shift = MXC_CCM_CSCMR1_USB_PHY_CLK_SEL_OFFSET,
	.disable = _clk_disable,
};

static struct clk rtc_clk = {
	.parent = &ckil_clk,
	.secondary = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.disable = _clk_disable,
};

static struct clk ata_clk = {
	.parent = &ipg_clk,
	.secondary = &spba_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk owire_clk = {
	.parent = &ipg_perclk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.disable = _clk_disable,
};


static struct clk fec_clk[] = {
	{
	.parent = &ipg_clk,
	.secondary = &fec_clk[1],
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	 .parent = &tmax2_clk,
	 .secondary = &fec_clk[2],
	},
	{
	 .parent = &aips_tz2_clk,
	 .secondary = &emi_fast_clk,
	},
};

static struct clk sahara_clk[] = {
	{
	.parent = &ahb_clk,
	.secondary = &sahara_clk[1],
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
	{
	.parent = &tmax1_clk,
	.secondary = &emi_fast_clk,
	}
};

static struct clk scc_clk[] = {
	{
	.parent = &ahb_clk,
	.secondary = &scc_clk[1],
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
	{
	.parent = &tmax1_clk,
	.secondary = &emi_fast_clk,
	}
};


static int _clk_gpu3d_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk);
	reg = (reg & ~MXC_CCM_CBCMR_GPU_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_GPU_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}


static struct clk garb_clk = {
	.parent = &axi_a_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.disable = _clk_disable,
};

static struct clk gpu3d_clk = {
	.parent = &axi_a_clk,
	.set_parent = _clk_gpu3d_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	.secondary = &garb_clk,
};

static int _clk_gpu2d_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk);
	reg = (reg & ~MXC_CCM_CBCMR_GPU2D_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_GPU2D_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk gpu2d_clk = {
	.parent = &axi_a_clk,
	.set_parent = _clk_gpu2d_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static unsigned long cko1_get_rate(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= MXC_CCM_CCOSR_CKOL_DIV_MASK;
	reg = reg >> MXC_CCM_CCOSR_CKOL_DIV_OFFSET;
	return clk_get_rate(clk->parent) / (reg + 1);
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
	u32 parent_rate = clk_get_rate(clk->parent);

	div = (parent_rate/rate - 1) & 0x7;
	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MXC_CCM_CCOSR_CKOL_DIV_MASK;
	reg |= div << MXC_CCM_CCOSR_CKOL_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}

static unsigned long cko1_round_rate(struct clk *clk, unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	div = div < 1 ? 1 : div;
	div = div > 8 ? 8 : div;
	return parent_rate / div;
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
	else if (parent == &emi_slow_clk)
		sel = 4;
	else if (parent == &pll4_sw_clk)
		sel = 5;
	else if (parent == &emi_enfc_clk)
		sel = 6;
	else if (parent == &ipu_di_clk[0])
		sel = 8;
	else if (parent == &ahb_clk)
		sel = 11;
	else if (parent == &ipg_clk)
		sel = 12;
	else if (parent == &ipg_perclk)
		sel = 13;
	else if (parent == &ckil_clk)
		sel = 14;
	else
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MXC_CCM_CCOSR_CKOL_SEL_MASK;
	reg |= sel << MXC_CCM_CCOSR_CKOL_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}
static struct clk cko1_clk = {
	.get_rate = cko1_get_rate,
	.enable = cko1_enable,
	.disable = cko1_disable,
	.set_rate = cko1_set_rate,
	.round_rate = cko1_round_rate,
	.set_parent = cko1_set_parent,
};
static int _clk_asrc_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	if (parent == &pll4_sw_clk)
		reg |= MXC_CCM_CSCMR2_ASRC_CLK_SEL;
	else
		reg &= ~MXC_CCM_CSCMR2_ASRC_CLK_SEL;
	 __raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static unsigned long _clk_asrc_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR2);
	prediv = ((reg & MXC_CCM_CSCDR2_ASRC_CLK_PRED_MASK) >>
		MXC_CCM_CSCDR2_ASRC_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CSCDR2_ASRC_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR2_ASRC_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
}

static int _clk_asrc_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	if ((parent_rate / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CSCDR2) &
		~(MXC_CCM_CSCDR2_ASRC_CLK_PRED_MASK |
		MXC_CCM_CSCDR2_ASRC_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR2_ASRC_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR2_ASRC_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	return 0;
}

static unsigned long _clk_asrc_round_rate(struct clk *clk,
			unsigned long rate)
{
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (parent_rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return parent_rate / (pre * post);
}

static struct clk asrc_clk[] = {
	{
	.id = 0,
	.parent = &pll4_sw_clk,
	.set_parent = _clk_asrc_set_parent,
	.get_rate = _clk_asrc_get_rate,
	.set_rate = _clk_asrc_set_rate,
	.round_rate = _clk_asrc_round_rate,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
	{
	.id = 0,
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};

#define _REGISTER_CLOCK(d, n, c) \
	{ \
		.dev_id = d, \
		.con_id = n, \
		.clk = &c, \
	}

static struct clk_lookup lookups[] = {
	_REGISTER_CLOCK(NULL, "osc", osc_clk),
	_REGISTER_CLOCK(NULL, "ckih", ckih_clk),
	_REGISTER_CLOCK(NULL, "ckih2", ckih2_clk),
	_REGISTER_CLOCK(NULL, "ckil", ckil_clk),
	_REGISTER_CLOCK(NULL, "pll1_main_clk", pll1_main_clk),
	_REGISTER_CLOCK(NULL, "pll1_sw_clk", pll1_sw_clk),
	_REGISTER_CLOCK(NULL, "pll2", pll2_sw_clk),
	_REGISTER_CLOCK(NULL, "pll3", pll3_sw_clk),
	_REGISTER_CLOCK(NULL, "gpc_dvfs_clk", gpc_dvfs_clk),
	_REGISTER_CLOCK(NULL, "lp_apm", lp_apm_clk),
	_REGISTER_CLOCK(NULL, "cpu_clk", cpu_clk),
	_REGISTER_CLOCK(NULL, "periph_apm_clk", periph_apm_clk),
	_REGISTER_CLOCK(NULL, "main_bus_clk", main_bus_clk),
	_REGISTER_CLOCK(NULL, "axi_a_clk", axi_a_clk),
	_REGISTER_CLOCK(NULL, "axi_b_clk", axi_b_clk),
	_REGISTER_CLOCK(NULL, "ahb_clk", ahb_clk),
	_REGISTER_CLOCK(NULL, "ahb_max_clk", ahb_max_clk),
	_REGISTER_CLOCK(NULL, "vpu_clk", vpu_clk[0]),
	_REGISTER_CLOCK(NULL, "vpu_core_clk", vpu_clk[1]),
	_REGISTER_CLOCK(NULL, "nfc_clk", emi_enfc_clk),
	_REGISTER_CLOCK("mxc_sdma", "sdma_ahb_clk", sdma_clk[0]),
	_REGISTER_CLOCK("mxc_sdma", "sdma_ipg_clk", sdma_clk[1]),
	_REGISTER_CLOCK(NULL, "ipu_clk", ipu_clk[0]),
	_REGISTER_CLOCK(NULL, "ipu_di0_clk", ipu_di_clk[0]),
	_REGISTER_CLOCK(NULL, "ipu_di1_clk", ipu_di_clk[1]),
	_REGISTER_CLOCK(NULL, "csi_mclk1", csi0_clk),
	_REGISTER_CLOCK(NULL, "csi_mclk2", csi1_clk),
	_REGISTER_CLOCK(NULL, "tve_clk", tve_clk),
	_REGISTER_CLOCK("mxcintuart.0", NULL, uart1_clk[0]),
	_REGISTER_CLOCK("mxcintuart.1", NULL, uart2_clk[0]),
	_REGISTER_CLOCK("mxcintuart.2", NULL, uart3_clk[0]),
	_REGISTER_CLOCK(NULL, "i2c_clk", i2c_clk[0]),
	_REGISTER_CLOCK("imx-i2c.1", NULL, i2c_clk[1]),
	_REGISTER_CLOCK("mxc_pwm.0", NULL, pwm1_clk[0]),
	_REGISTER_CLOCK("mxc_pwm.1", NULL, pwm2_clk[0]),
	_REGISTER_CLOCK("mxc_spi.0", NULL, cspi1_clk[0]),
	_REGISTER_CLOCK("mxc_spi.1", NULL, cspi2_clk[0]),
	_REGISTER_CLOCK("mxc_spi.2", NULL, cspi3_clk),
	_REGISTER_CLOCK(NULL, "ssi_lp_apm_clk", ssi_lp_apm_clk),
	_REGISTER_CLOCK("mxc_ssi.0", NULL, ssi1_clk[0]),
	_REGISTER_CLOCK("mxc_ssi.1", NULL, ssi2_clk[0]),
	_REGISTER_CLOCK("mxc_ssi.2", NULL, ssi3_clk[0]),
	_REGISTER_CLOCK(NULL, "ssi_ext1_clk", ssi_ext1_clk),
	_REGISTER_CLOCK(NULL, "ssi_ext2_clk", ssi_ext2_clk),
	_REGISTER_CLOCK(NULL, "iim_clk", iim_clk),
	_REGISTER_CLOCK(NULL, "usboh3_clk", usboh3_clk[0]),
	_REGISTER_CLOCK(NULL, "usb_ahb_clk", usb_ahb_clk),
	_REGISTER_CLOCK(NULL, "usb_phy1_clk", usb_phy_clk[0]),
	_REGISTER_CLOCK(NULL, "usb_utmi_clk", usb_utmi_clk),
	_REGISTER_CLOCK(NULL, "usb_clk", usb_clk),
	_REGISTER_CLOCK("mxsdhci.0", NULL, esdhc1_clk[0]),
	_REGISTER_CLOCK("mxsdhci.1", NULL, esdhc2_clk[0]),
	_REGISTER_CLOCK("mxsdhci.2", NULL, esdhc3_clk[0]),
	_REGISTER_CLOCK("mxsdhci.3", NULL, esdhc4_clk[0]),
	_REGISTER_CLOCK(NULL, "emi_slow_clk", emi_slow_clk),
	_REGISTER_CLOCK(NULL, "ddr_clk", ddr_clk),
	_REGISTER_CLOCK(NULL, "emi_enfc_clk", emi_enfc_clk),
	_REGISTER_CLOCK(NULL, "emi_fast_clk", emi_fast_clk),
	_REGISTER_CLOCK(NULL, "emi_intr_clk.0", emi_intr_clk[0]),
	_REGISTER_CLOCK(NULL, "emi_intr_clk.1", emi_intr_clk[1]),
	_REGISTER_CLOCK(NULL, "spdif_xtal_clk", spdif_xtal_clk),
	_REGISTER_CLOCK("mxc_alsa_spdif.0", NULL, spdif0_clk[0]),
	_REGISTER_CLOCK("mxc_vpu.0", NULL, vpu_clk[0]),
	_REGISTER_CLOCK(NULL, "lpsr_clk", lpsr_clk),
	_REGISTER_CLOCK("mxc_rtc.0", NULL, rtc_clk),
	_REGISTER_CLOCK("pata_fsl", NULL, ata_clk),
	_REGISTER_CLOCK("mxc_w1.0", NULL, owire_clk),
	_REGISTER_CLOCK(NULL, "sahara_clk", sahara_clk[0]),
	_REGISTER_CLOCK(NULL, "gpu3d_clk", gpu3d_clk),
	_REGISTER_CLOCK(NULL, "garb_clk", garb_clk),
	_REGISTER_CLOCK(NULL, "gpu2d_clk", gpu2d_clk),
	_REGISTER_CLOCK("mxc_scc.0", NULL, scc_clk[0]),
	_REGISTER_CLOCK(NULL, "cko1", cko1_clk),
	_REGISTER_CLOCK(NULL, "gpt", gpt_clk[0]),
	_REGISTER_CLOCK("fec.0", NULL, fec_clk[0]),
	_REGISTER_CLOCK("mxc_w1.0", NULL, owire_clk),
};

static struct clk_lookup mx51_lookups[] = {
	_REGISTER_CLOCK("mxc_i2c_hs.3", NULL, hsi2c_serial_clk),
	_REGISTER_CLOCK("mxc_sim.0", NULL, sim_clk[0]),
	_REGISTER_CLOCK("mxc_alsa_spdif.0", NULL, spdif1_clk[0]),
	_REGISTER_CLOCK(NULL, "mipi_hsp_clk", mipi_hsp_clk),
	_REGISTER_CLOCK(NULL, "ddr_hf_clk", ddr_hf_clk),
};

static struct clk_lookup mx53_lookups[] = {
	_REGISTER_CLOCK(NULL, "pll4", pll4_sw_clk),
	_REGISTER_CLOCK("mxcintuart.3", NULL, uart4_clk[0]),
	_REGISTER_CLOCK("mxcintuart.4", NULL, uart5_clk[0]),
	_REGISTER_CLOCK("imx-i2c.2", NULL, i2c_clk[2]),
	_REGISTER_CLOCK(NULL, "usb_phy2_clk", usb_phy_clk[1]),
	_REGISTER_CLOCK(NULL, "ocram_clk", ocram_clk),
	_REGISTER_CLOCK(NULL, "imx_sata_clk", sata_clk),
	_REGISTER_CLOCK(NULL, "ieee_1588_clk", ieee_1588_clk),
	_REGISTER_CLOCK(NULL, "ieee_rtc_clk", ieee_rtc_clk),
	_REGISTER_CLOCK("mxc_mlb.0", NULL, mlb_clk[0]),
	_REGISTER_CLOCK("FlexCAN.0", "can_clk", can1_clk[0]),
	_REGISTER_CLOCK("FlexCAN.1", "can_clk", can2_clk[0]),
	_REGISTER_CLOCK(NULL, "ldb_di0_clk", ldb_di_clk[0]),
	_REGISTER_CLOCK(NULL, "ldb_di1_clk", ldb_di_clk[1]),
	_REGISTER_CLOCK(NULL, "esai_clk", esai_clk[0]),
	_REGISTER_CLOCK(NULL, "esai_ipg_clk", esai_clk[1]),
	_REGISTER_CLOCK(NULL, "asrc_clk", asrc_clk[1]),
	_REGISTER_CLOCK(NULL, "asrc_serial_clk", asrc_clk[0]),
};

static struct mxc_clk mx51_clks[ARRAY_SIZE(lookups) + ARRAY_SIZE(mx51_lookups)];
static struct mxc_clk mx53_clks[ARRAY_SIZE(lookups) + ARRAY_SIZE(mx53_lookups)];

static void clk_tree_init(void)
{
	u32 reg, dp_ctl;

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

	if (cpu_is_mx51()) {
		dp_ctl = __raw_readl(pll1_base + MXC_PLL_DP_CTL);
		if ((dp_ctl & MXC_PLL_DP_CTL_REF_CLK_SEL_MASK) == 0)
			pll1_main_clk.parent = &fpm_clk;

		dp_ctl = __raw_readl(pll2_base + MXC_PLL_DP_CTL);
		if ((dp_ctl & MXC_PLL_DP_CTL_REF_CLK_SEL_MASK) == 0)
			pll2_sw_clk.parent = &fpm_clk;

		dp_ctl = __raw_readl(pll3_base + MXC_PLL_DP_CTL);
		if ((dp_ctl & MXC_PLL_DP_CTL_REF_CLK_SEL_MASK) == 0)
			pll3_sw_clk.parent = &fpm_clk;
	} else {
		/* set pll4_clk parent */
		pll4_sw_clk.parent = &osc_clk;
	}

	/* set emi_slow_clk parent */
	emi_slow_clk.parent = &main_bus_clk;
	reg = __raw_readl(MXC_CCM_CBCDR);
	if ((reg & MXC_CCM_CBCDR_EMI_CLK_SEL) != 0)
		emi_slow_clk.parent = &ahb_clk;

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


int __init mx51_clocks_init(unsigned long ckil, unsigned long osc, unsigned long ckih1, unsigned long ckih2)
{
	__iomem void *base;
	struct clk *tclk;
	int i = 0, j = 0, reg;
	int wp_cnt = 0;
	u32 pll1_rate;

	pll1_base = ioremap(PLL1_BASE_ADDR, SZ_4K);
	pll2_base = ioremap(PLL2_BASE_ADDR, SZ_4K);
	pll3_base = ioremap(PLL3_BASE_ADDR, SZ_4K);

	/* Turn off all possible clocks */
	if (mxc_jtag_enabled) {
		__raw_writel(1 << MXC_CCM_CCGRx_CG0_OFFSET |
			      1 << MXC_CCM_CCGRx_CG1_OFFSET |
			      1 << MXC_CCM_CCGRx_CG2_OFFSET |
			      3 << MXC_CCM_CCGRx_CG3_OFFSET |
			      3 << MXC_CCM_CCGRx_CG4_OFFSET |
			      3 << MXC_CCM_CCGRx_CG8_OFFSET |
			      3 << MXC_CCM_CCGRx_CG9_OFFSET |
			      1 << MXC_CCM_CCGRx_CG12_OFFSET |
			      1 << MXC_CCM_CCGRx_CG13_OFFSET |
			      1 << MXC_CCM_CCGRx_CG14_OFFSET, MXC_CCM_CCGR0);
	} else {
		__raw_writel(1 << MXC_CCM_CCGRx_CG0_OFFSET |
			      1 << MXC_CCM_CCGRx_CG1_OFFSET |
			      1 << MXC_CCM_CCGRx_CG2_OFFSET |
			      3 << MXC_CCM_CCGRx_CG3_OFFSET |
			      3 << MXC_CCM_CCGRx_CG8_OFFSET |
			      3 << MXC_CCM_CCGRx_CG9_OFFSET |
			      1 << MXC_CCM_CCGRx_CG12_OFFSET |
			      1 << MXC_CCM_CCGRx_CG13_OFFSET |
			      3 << MXC_CCM_CCGRx_CG14_OFFSET, MXC_CCM_CCGR0);
	}
	__raw_writel(0, MXC_CCM_CCGR1);
	__raw_writel(0, MXC_CCM_CCGR2);
	__raw_writel(0, MXC_CCM_CCGR3);
	__raw_writel(1 << MXC_CCM_CCGRx_CG8_OFFSET, MXC_CCM_CCGR4);

	__raw_writel(1 << MXC_CCM_CCGRx_CG2_OFFSET |
		     1 << MXC_CCM_CCGR5_CG6_1_OFFSET |
		     1 << MXC_CCM_CCGR5_CG6_2_OFFSET |
		     3 << MXC_CCM_CCGRx_CG7_OFFSET |
		     1 << MXC_CCM_CCGRx_CG8_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET |
		     1 << MXC_CCM_CCGRx_CG10_OFFSET |
		     3 << MXC_CCM_CCGRx_CG11_OFFSET, MXC_CCM_CCGR5);

	__raw_writel(1 << MXC_CCM_CCGRx_CG4_OFFSET, MXC_CCM_CCGR6);

	external_low_reference = ckil;
	external_high_reference = ckih1;
	ckih2_reference = ckih2;
	oscillator_reference = osc;

	/* Fix up clocks unique to MX51. */
	esdhc2_clk[0].get_rate = _clk_esdhc2_get_rate;
	esdhc2_clk[0].set_rate = _clk_esdhc2_set_rate;

	clk_tree_init();

	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		clkdev_add(&lookups[i]);
		mx51_clks[i].reg_clk = lookups[i].clk;
		if (lookups[i].con_id != NULL)
			strcpy(mx51_clks[i].name, lookups[i].con_id);
		else
			strcpy(mx51_clks[i].name, lookups[i].dev_id);
		clk_register(&mx51_clks[i]);
	}

	j = ARRAY_SIZE(lookups);
	for (i = 0; i < ARRAY_SIZE(mx51_lookups); i++) {
		clkdev_add(&mx51_lookups[i]);
		mx51_clks[i+j].reg_clk = mx51_lookups[i].clk;
		if (mx51_lookups[i].con_id != NULL)
			strcpy(mx51_clks[i+j].name, mx51_lookups[i].con_id);
		else
			strcpy(mx51_clks[i+j].name, mx51_lookups[i].dev_id);
		clk_register(&mx51_clks[i+j]);
	}

	max_axi_a_clk = MAX_AXI_A_CLK_MX51;
	max_axi_b_clk = MAX_AXI_B_CLK_MX51;
	max_ahb_clk = MAX_AHB_CLK_MX51;
	max_emi_slow_clk = MAX_AHB_CLK_MX51;

	/* set DDR clock parent */
	reg = 0;
	if (cpu_is_mx51_rev(CHIP_REV_2_0) >= 1) {
		reg = __raw_readl(MXC_CCM_CBCDR) & MXC_CCM_CBCDR_DDR_HF_SEL;
		reg >>= MXC_CCM_CBCDR_DDR_HF_SEL_OFFSET;

		if (reg)
			tclk = &ddr_hf_clk;
	}
	if (reg == 0) {
		reg = __raw_readl(MXC_CCM_CBCMR) &
					MXC_CCM_CBCMR_DDR_CLK_SEL_MASK;
		reg >>= MXC_CCM_CBCMR_DDR_CLK_SEL_OFFSET;

		if (reg == 0) {
			tclk = &axi_a_clk;
		} else if (reg == 1) {
			tclk = &axi_b_clk;
		} else if (reg == 2) {
			tclk = &emi_slow_clk;
		} else {
			tclk = &ahb_clk;
		}
	}
	clk_set_parent(&ddr_clk, tclk);

	/*Setup the LPM bypass bits */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_HSC_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_RTIC_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_SCC_LPM_HS_MX51
		| MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS_MX51;
	__raw_writel(reg, MXC_CCM_CLPCR);

	/* Disable the handshake with HSC block as its not
	  * initialised right now.
	  */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_HSC_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	clk_enable(&cpu_clk);

	/* Set SDHC parents to be PLL2 */
	clk_set_parent(&esdhc1_clk[0], &pll2_sw_clk);
	clk_set_parent(&esdhc2_clk[0], &pll2_sw_clk);

	/* set SDHC root clock as 166.25MHZ*/
	clk_set_rate(&esdhc1_clk[0], 166250000);
	clk_set_rate(&esdhc2_clk[0], 166250000);

	/* Initialise the parents to be axi_b, parents are set to
	 * axi_a when the clocks are enabled.
	 */
	clk_set_parent(&vpu_clk[0], &axi_b_clk);
	clk_set_parent(&vpu_clk[1], &axi_b_clk);
	clk_set_parent(&gpu3d_clk, &axi_a_clk);
	clk_set_parent(&gpu2d_clk, &axi_a_clk);

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

	/*
	 * SSI3 has no clock divide register,
	 * we always set SSI3 parent clock to SSI1 and freq same to SSI1
	 */
	clk_set_parent(&ssi3_clk[0], &ssi1_clk[0]);

	/* Change the SSI_EXT1_CLK to be sourced from SSI1_CLK_ROOT */
	clk_set_parent(&ssi_ext1_clk, &ssi1_clk[0]);
	clk_set_parent(&ssi_ext2_clk, &ssi2_clk[0]);

	/* move usb_phy_clk to 24MHz */
	clk_set_parent(&usb_phy_clk[0], &osc_clk);

	/* set usboh3_clk to pll2 */
	clk_set_parent(&usboh3_clk[0], &pll2_sw_clk);
	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_USBOH3_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CSCDR1_USBOH3_CLK_PRED_MASK;
	reg |= 4 << MXC_CCM_CSCDR1_USBOH3_CLK_PRED_OFFSET;
	reg |= 1 << MXC_CCM_CSCDR1_USBOH3_CLK_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	/* Set the current working point. */
	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	/* Update the cpu working point table based on the PLL1 freq
	 * at boot time
	 */
	pll1_rate = clk_get_rate(&pll1_main_clk);
	if (pll1_rate <= cpu_wp_tbl[cpu_wp_nr - 1].cpu_rate)
		wp_cnt = 1;
	else if (pll1_rate <= cpu_wp_tbl[1].cpu_rate &&
				pll1_rate > cpu_wp_tbl[2].cpu_rate)
		wp_cnt = cpu_wp_nr - 1;
	else
		wp_cnt = cpu_wp_nr;

	cpu_wp_tbl[0].cpu_rate = pll1_rate;

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

	pll1_rate = clk_get_rate(&pll1_main_clk);
	for (j = 0; j < cpu_wp_nr; j++) {
		if ((ddr_clk.parent == &ddr_hf_clk)) {
			/* Change the CPU podf divider based on the boot up
			 * pll1 rate.
			 */
			cpu_wp_tbl[j].cpu_podf =
				(pll1_rate / cpu_wp_tbl[j].cpu_rate)
				- 1;
			if (pll1_rate / (cpu_wp_tbl[j].cpu_podf + 1) >
					cpu_wp_tbl[j].cpu_rate) {
				cpu_wp_tbl[j].cpu_podf++;
				cpu_wp_tbl[j].cpu_rate =
					 pll1_rate /
					 (1000 * (cpu_wp_tbl[j].cpu_podf + 1));
				cpu_wp_tbl[j].cpu_rate *= 1000;
			}
			if (pll1_rate / (cpu_wp_tbl[j].cpu_podf + 1) <
						cpu_wp_tbl[j].cpu_rate) {
				cpu_wp_tbl[j].cpu_rate = pll1_rate;
			}
		}
	cpu_wp_tbl[j].pll_rate = pll1_rate;
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

	clk_set_parent(&arm_axi_clk, &axi_a_clk);
	clk_set_parent(&ipu_clk[0], &axi_b_clk);

	if (uart_at_24) {
		/* Move UART to run from lp_apm */
		clk_set_parent(&uart_main_clk, &lp_apm_clk);

		/* Set the UART dividers to divide, so the UART_CLK is 24MHz. */
		reg = __raw_readl(MXC_CCM_CSCDR1);
		reg &= ~MXC_CCM_CSCDR1_UART_CLK_PODF_MASK;
		reg &= ~MXC_CCM_CSCDR1_UART_CLK_PRED_MASK;
		reg |= (0 << MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET) |
		    (0 << MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET);
		__raw_writel(reg, MXC_CCM_CSCDR1);
	} else {
		clk_set_parent(&uart_main_clk, &pll2_sw_clk);

		/* Set the UART dividers to divide, so the UART_CLK is 66.5MHz. */
		reg = __raw_readl(MXC_CCM_CSCDR1);
		reg &= ~MXC_CCM_CSCDR1_UART_CLK_PODF_MASK;
		reg &= ~MXC_CCM_CSCDR1_UART_CLK_PRED_MASK;
		reg |= (4 << MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET) |
		    (1 << MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET);
		__raw_writel(reg, MXC_CCM_CSCDR1);
	}

	clk_set_parent(&emi_slow_clk, &ahb_clk);
	clk_set_rate(&emi_slow_clk, clk_round_rate(&emi_slow_clk, 130000000));

	/* Change the NFC clock rate to be 1:4 ratio with emi clock. */
	clk_set_rate(&emi_enfc_clk, clk_round_rate(&emi_enfc_clk,
			(clk_get_rate(&emi_slow_clk))/4));

	base = ioremap(GPT1_BASE_ADDR, SZ_4K);
	mxc_timer_init(&gpt_clk[0], base, MXC_INT_GPT);
	return 0;
}

int __init mx53_clocks_init(unsigned long ckil, unsigned long osc, unsigned long ckih1, unsigned long ckih2)
{
	__iomem void *base;
	struct clk *tclk;
	int i = 0, j = 0, reg;
	u32 pll1_rate;

	pll1_base = ioremap(MX53_BASE_ADDR(PLL1_BASE_ADDR), SZ_4K);
	pll2_base = ioremap(MX53_BASE_ADDR(PLL2_BASE_ADDR), SZ_4K);
	pll3_base = ioremap(MX53_BASE_ADDR(PLL3_BASE_ADDR), SZ_4K);
	pll4_base = ioremap(MX53_BASE_ADDR(PLL4_BASE_ADDR), SZ_4K);

	/* Turn off all possible clocks */
	if (mxc_jtag_enabled) {
		__raw_writel(1 << MXC_CCM_CCGRx_CG0_OFFSET |
			      1 << MXC_CCM_CCGRx_CG1_OFFSET |
			      1 << MXC_CCM_CCGRx_CG2_OFFSET |
			      3 << MXC_CCM_CCGRx_CG3_OFFSET |
			      3 << MXC_CCM_CCGRx_CG4_OFFSET |
			      3 << MXC_CCM_CCGRx_CG8_OFFSET |
			      3 << MXC_CCM_CCGRx_CG9_OFFSET |
			      1 << MXC_CCM_CCGRx_CG12_OFFSET |
			      1 << MXC_CCM_CCGRx_CG13_OFFSET |
			      1 << MXC_CCM_CCGRx_CG14_OFFSET, MXC_CCM_CCGR0);
	} else {
		__raw_writel(1 << MXC_CCM_CCGRx_CG0_OFFSET |
			      1 << MXC_CCM_CCGRx_CG1_OFFSET |
			      3 << MXC_CCM_CCGRx_CG3_OFFSET |
			      3 << MXC_CCM_CCGRx_CG8_OFFSET |
			      3 << MXC_CCM_CCGRx_CG9_OFFSET |
			      1 << MXC_CCM_CCGRx_CG12_OFFSET |
			      1 << MXC_CCM_CCGRx_CG13_OFFSET |
			      3 << MXC_CCM_CCGRx_CG14_OFFSET, MXC_CCM_CCGR0);
	}

	__raw_writel(0, MXC_CCM_CCGR1);
	__raw_writel(0, MXC_CCM_CCGR2);
	__raw_writel(0, MXC_CCM_CCGR3);
	__raw_writel(1 << MXC_CCM_CCGRx_CG8_OFFSET, MXC_CCM_CCGR4);

	__raw_writel(1 << MXC_CCM_CCGRx_CG2_OFFSET |
		     1 << MXC_CCM_CCGRx_CG6_OFFSET |
		     3 << MXC_CCM_CCGRx_CG7_OFFSET |
		     1 << MXC_CCM_CCGRx_CG8_OFFSET |
		     1 << MXC_CCM_CCGRx_CG9_OFFSET |
		     3 << MXC_CCM_CCGRx_CG11_OFFSET, MXC_CCM_CCGR5);

	__raw_writel(1 << MXC_CCM_CCGRx_CG0_OFFSET |
				3 << MXC_CCM_CCGRx_CG1_OFFSET |
				1 << MXC_CCM_CCGRx_CG4_OFFSET |
				3 << MXC_CCM_CCGRx_CG12_OFFSET |
				3 << MXC_CCM_CCGRx_CG13_OFFSET , MXC_CCM_CCGR6);

	__raw_writel(0, MXC_CCM_CCGR7);

	external_low_reference = ckil;
	external_high_reference = ckih1;
	ckih2_reference = ckih2;
	oscillator_reference = osc;

	usb_phy_clk[0].enable_reg = MXC_CCM_CCGR4;
	usb_phy_clk[0].enable_shift = MXC_CCM_CCGRx_CG5_OFFSET;

	ipumux1_clk.enable_reg = MXC_CCM_CCGR5;
	ipumux1_clk.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET;
	ipumux2_clk.enable_reg = MXC_CCM_CCGR6;
	ipumux2_clk.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET;

	esdhc3_clk[0].get_rate = _clk_esdhc3_get_rate;
	esdhc3_clk[0].set_rate = _clk_sdhc3_set_rate;

#if defined(CONFIG_USB_STATIC_IRAM) \
    || defined(CONFIG_USB_STATIC_IRAM_PPH)
	usboh3_clk[1].secondary = &emi_intr_clk[1];
#endif
#ifdef CONFIG_SND_MXC_SOC_IRAM
	ssi2_clk[2].secondary = &emi_intr_clk[1];
	ssi1_clk[2].secondary = &emi_intr_clk[1];
#endif
#ifdef CONFIG_SDMA_IRAM
	sdma_clk[1].secondary = &emi_intr_clk[1];
#endif

	clk_tree_init();

	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		clkdev_add(&lookups[i]);
		mx53_clks[i].reg_clk = lookups[i].clk;
		if (lookups[i].con_id != NULL)
			strcpy(mx53_clks[i].name, lookups[i].con_id);
		else
			strcpy(mx53_clks[i].name, lookups[i].dev_id);
		clk_register(&mx53_clks[i]);
	}

	j = ARRAY_SIZE(lookups);
	for (i = 0; i < ARRAY_SIZE(mx53_lookups); i++) {
		clkdev_add(&mx53_lookups[i]);
		mx53_clks[i+j].reg_clk = mx53_lookups[i].clk;
		if (mx53_lookups[i].con_id != NULL)
			strcpy(mx53_clks[i+j].name, mx53_lookups[i].con_id);
		else
			strcpy(mx53_clks[i+j].name, mx53_lookups[i].dev_id);
		clk_register(&mx53_clks[i+j]);
	}

	clk_set_parent(&esai_clk[0], &ckih_clk);

	ldb_di_clk[0].parent = ldb_di_clk[1].parent =
	tve_clk.parent = &pll4_sw_clk;

	max_axi_a_clk = MAX_AXI_A_CLK_MX53;
	max_axi_b_clk = MAX_AXI_B_CLK_MX53;
	max_ahb_clk = MAX_AHB_CLK_MX53;
	max_emi_slow_clk = MAX_AHB_CLK_MX53;


	/* set DDR clock parent */
	reg = __raw_readl(MXC_CCM_CBCMR) &
				MXC_CCM_CBCMR_DDR_CLK_SEL_MASK;
	reg >>= MXC_CCM_CBCMR_DDR_CLK_SEL_OFFSET;
	if (reg == 0) {
		tclk = &axi_a_clk;
	} else if (reg == 1) {
		tclk = &axi_b_clk;
	} else if (reg == 2) {
		tclk = &emi_slow_clk;
	} else {
		tclk = &ahb_clk;
	}
	clk_set_parent(&ddr_clk, tclk);

	clk_set_parent(&esdhc1_clk[2], &tmax2_clk);
	clk_set_parent(&esdhc2_clk[0], &esdhc1_clk[0]);
	clk_set_parent(&esdhc3_clk[0], &pll2_sw_clk);

	clk_set_parent(&ipu_di_clk[0], &pll4_sw_clk);

#if 0
	/*Setup the LPM bypass bits */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_RTIC_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_SCC_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
#endif

	clk_enable(&cpu_clk);

	clk_enable(&main_bus_clk);

	/* Set AXI_B_CLK to be 200MHz */
	clk_set_rate(&axi_b_clk, 200000000);

	/* Initialise the parents to be axi_b, parents are set to
	 * axi_a when the clocks are enabled.
	 */

	clk_set_parent(&vpu_clk[0], &axi_b_clk);
	clk_set_parent(&vpu_clk[1], &axi_b_clk);

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

	/* set usboh3_clk to pll2 */
	clk_set_parent(&usboh3_clk[0], &pll2_sw_clk);
	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_USBOH3_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CSCDR1_USBOH3_CLK_PRED_MASK;
	reg |= 4 << MXC_CCM_CSCDR1_USBOH3_CLK_PRED_OFFSET;
	reg |= 1 << MXC_CCM_CSCDR1_USBOH3_CLK_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	/* set SDHC root clock as 200MHZ*/
	clk_set_rate(&esdhc1_clk[0], 200000000);
	clk_set_rate(&esdhc3_clk[0], 200000000);

	 /* Set the 1588 RTC input clocks as 108MHZ */
	clk_set_parent(&ieee_rtc_clk, &pll3_sw_clk);
	clk_set_rate(&ieee_rtc_clk, 108000000);

	/* The CPU working point should be set according to part number
	 * information. But part number information is not clear now.
	 * So update the cpu working point table based on the PLL1 freq
	 * at boot time
	 */
	pll1_rate = clk_get_rate(&pll1_main_clk);

	if (pll1_rate > 1000000000)
		mx53_set_cpu_part_number(IMX53_CEC_1_2G);
	else if (pll1_rate > 800000000)
		mx53_set_cpu_part_number(IMX53_CEC);
	else
		mx53_set_cpu_part_number(IMX53_AEC);

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

	clk_set_parent(&arm_axi_clk, &axi_b_clk);
	clk_set_parent(&ipu_clk[0], &axi_b_clk);
	clk_set_parent(&uart_main_clk, &pll3_sw_clk);
	clk_set_parent(&gpu3d_clk, &axi_b_clk);
	clk_set_parent(&gpu2d_clk, &axi_b_clk);

	clk_set_parent(&emi_slow_clk, &ahb_clk);
	clk_set_rate(&emi_slow_clk, clk_round_rate(&emi_slow_clk, 133333333));

	clk_set_rate(&emi_enfc_clk, clk_round_rate(&emi_enfc_clk,
			MAX_NFC_CLK));

	/* set the freq of asrc_serial_clk */
	clk_set_rate(&asrc_clk[0], clk_round_rate(&asrc_clk[0],
			1190000));
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
	u32 reg, pll_hfsm;
	u32 stat;

	if (wp == cpu_curr_wp)
		return 0;

	p = &cpu_wp_tbl[wp];

	/*
	 * If DDR clock is sourced from PLL1, we cannot drop PLL1 freq.
	 * Use the ARM_PODF to change the freq of the core, leave the PLL1
	 * freq unchanged. Meanwhile, if pll_rate is same, use the ARM_PODF
	 * to change the freq of core
	 */
	if ((ddr_clk.parent == &ddr_hf_clk) ||
		(p->pll_rate == cpu_wp_tbl[cpu_curr_wp].pll_rate)) {
		reg = __raw_readl(MXC_CCM_CACRR);
		reg &= ~MXC_CCM_CACRR_ARM_PODF_MASK;
		reg |= cpu_wp_tbl[wp].cpu_podf << MXC_CCM_CACRR_ARM_PODF_OFFSET;
		__raw_writel(reg, MXC_CCM_CACRR);
		cpu_curr_wp = wp;
	} else {
		struct timespec nstimeofday;
		struct timespec curtime;

		/* Change the ARM clock to requested frequency */
		/* First move the ARM clock to step clock which is running
		 * at 24MHz.
		 */

		/* Change the source of pll1_sw_clk to be the step_clk */
		reg = __raw_readl(MXC_CCM_CCSR);
		reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CCSR);

		/* Stop the PLL */
		reg = __raw_readl(pll1_base + MXC_PLL_DP_CTL);
		reg &= ~MXC_PLL_DP_CTL_UPEN;
		__raw_writel(reg, pll1_base + MXC_PLL_DP_CTL);

		reg = __raw_readl(MXC_CCM_CACRR);
		reg = (reg & ~MXC_CCM_CACRR_ARM_PODF_MASK)
			| p->cpu_podf;
		__raw_writel(reg, MXC_CCM_CACRR);

		reg = __raw_readl(pll1_base + MXC_PLL_DP_CTL);
		pll_hfsm = reg & MXC_PLL_DP_CTL_HFSM;
		/* PDF and MFI */
		reg = p->pdf | p->mfi << MXC_PLL_DP_OP_MFI_OFFSET;
		if (pll_hfsm == 0) {
			__raw_writel(reg, pll1_base + MXC_PLL_DP_OP);
			__raw_writel(p->mfd, pll1_base + MXC_PLL_DP_MFD);
			__raw_writel(p->mfn, pll1_base + MXC_PLL_DP_MFN);
		} else {
			__raw_writel(reg, pll1_base + MXC_PLL_DP_HFS_OP);
			__raw_writel(p->mfd, pll1_base + MXC_PLL_DP_HFS_MFD);
			__raw_writel(p->mfn, pll1_base + MXC_PLL_DP_HFS_MFN);
		}

		reg = __raw_readl(pll1_base + MXC_PLL_DP_CTL);
		reg |= MXC_PLL_DP_CTL_UPEN;
		/* Set the UPEN bits */
		__raw_writel(reg, pll1_base + MXC_PLL_DP_CTL);
		/* Forcefully restart the PLL */
		reg |= MXC_PLL_DP_CTL_RST;
		__raw_writel(reg, pll1_base + MXC_PLL_DP_CTL);

		/* Wait for the PLL to lock */
		getnstimeofday(&nstimeofday);
		do {
			getnstimeofday(&curtime);
			if ((curtime.tv_nsec - nstimeofday.tv_nsec) > SPIN_DELAY)
				panic("pll1 relock failed\n");
			stat = __raw_readl(pll1_base + MXC_PLL_DP_CTL) &
			    MXC_PLL_DP_CTL_LRF;
		} while (!stat);

		reg = __raw_readl(MXC_CCM_CCSR);
		/* Move the PLL1 back to the pll1_main_clk */
		reg &= ~MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CCSR);

		cpu_curr_wp = wp;
	}

#if defined(CONFIG_CPU_FREQ)
	cpufreq_trig_needed = 1;
#endif
	return 0;
}
