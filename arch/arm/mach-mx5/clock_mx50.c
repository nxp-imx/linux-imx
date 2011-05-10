/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

/* External clock values passed-in by the board code */
static unsigned long external_high_reference, external_low_reference;
static unsigned long oscillator_reference, ckih2_reference;

static struct clk pll1_main_clk;
static struct clk pll1_sw_clk;
static struct clk pll2_sw_clk;
static struct clk pll3_sw_clk;
static struct clk apbh_dma_clk;
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
static struct clk weim_clk[];
static struct clk ddr_clk;
static struct clk axi_a_clk;
static struct clk axi_b_clk;
static struct clk gpu2d_clk;
static int cpu_curr_wp;
static struct cpu_wp *cpu_wp_tbl;

static void __iomem *pll1_base;
static void __iomem *pll2_base;
static void __iomem *pll3_base;
void __iomem *apll_base;

extern int cpu_wp_nr;
extern int lp_high_freq;
extern int lp_med_freq;

void __iomem *databahn;

#define DDR_SYNC_MODE		0x30000
#define SPIN_DELAY	1000000 /* in nanoseconds */
#define WAIT(exp, timeout) \
({ \
	struct timespec nstimeofday; \
	struct timespec curtime; \
	int result = 1; \
	getnstimeofday(&nstimeofday); \
	while (!(exp)) { \
		getnstimeofday(&curtime); \
		if ((curtime.tv_nsec - nstimeofday.tv_nsec) > (timeout)) { \
			result = 0; \
			break; \
		} \
	} \
	result; \
})

#define MAX_AXI_A_CLK_MX50 	400000000
#define MAX_AXI_B_CLK_MX50 	200000000
#define MAX_AHB_CLK		133333333
#define MAX_EMI_SLOW_CLK	133000000

extern int mxc_jtag_enabled;
extern int uart_at_24;
extern int cpufreq_trig_needed;
extern int low_bus_freq_mode;
extern int med_bus_freq_mode;

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

static unsigned long _clk_round_rate_div(struct clk *clk,
						unsigned long rate,
						u32 max_div,
						u32 *new_div)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = DIV_ROUND_UP(parent_rate, rate);
	if (div > max_div)
		div = max_div;
	else if (div == 0)
		div++;
	if (new_div != NULL)
		*new_div = div;

	return parent_rate / div;
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

static int apll_enable(struct clk *clk)
{
	/* Set bit to flush multiple edges out of PLL vco */
	__raw_writel(MXC_ANADIG_PLL_HOLD_RING_OFF,
		apll_base + MXC_ANADIG_MISC_SET);

	__raw_writel(MXC_ANADIG_PLL_POWERUP, apll_base + MXC_ANADIG_MISC_SET);

	if (!WAIT(__raw_readl(apll_base + MXC_ANADIG_PLLCTRL)
		  & MXC_ANADIG_APLL_LOCK, 80000))
		panic("apll_enable failed!\n");

	/* Clear after relocking, then wait 10 us */
	__raw_writel(MXC_ANADIG_PLL_HOLD_RING_OFF,
		apll_base + MXC_ANADIG_MISC_CLR);

	udelay(10);

	return 0;
}

static void apll_disable(struct clk *clk)
{
	__raw_writel(MXC_ANADIG_PLL_POWERUP, apll_base + MXC_ANADIG_MISC_CLR);
}

static unsigned long apll_get_rate(struct clk *clk)
{
	return 480000000;
}

static struct clk apll_clk = {
	.get_rate = apll_get_rate,
	.enable = apll_enable,
	.disable = apll_disable,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static unsigned long pfd_round_rate(struct clk *clk, unsigned long rate)
{
	u32 frac;
	u64 tmp;
	tmp = (u64)clk_get_rate(clk->parent) * 18;
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
	tmp = (u64)clk_get_rate(clk->parent) * 18;

	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.enable(&apbh_dma_clk);

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

	tmp = (u64)clk_get_rate(clk->parent) * 18;
	do_div(tmp, frac);

	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.disable(&apbh_dma_clk);
	return 0;
}

static int pfd_enable(struct clk *clk)
{
	int index;

	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.enable(&apbh_dma_clk);
	index = _get_mux8(clk, &pfd0_clk, &pfd1_clk, &pfd2_clk, &pfd3_clk,
			&pfd4_clk, &pfd5_clk, &pfd6_clk, &pfd7_clk);
	/* clear clk gate bit */
	__raw_writel((1 << (clk->enable_shift + 7)),
			apll_base + (int)clk->enable_reg + 8);

	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.disable(&apbh_dma_clk);
	return 0;
}

static void pfd_disable(struct clk *clk)
{
	int index;

	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.enable(&apbh_dma_clk);
	index = _get_mux8(clk, &pfd0_clk, &pfd1_clk, &pfd2_clk, &pfd3_clk,
			&pfd4_clk, &pfd5_clk, &pfd6_clk, &pfd7_clk);
	/* set clk gate bit */
	__raw_writel((1 << (clk->enable_shift + 7)),
			apll_base + (int)clk->enable_reg + 4);
	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.disable(&apbh_dma_clk);
}

static struct clk pfd0_clk = {
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC0,
	.enable_shift = MXC_ANADIG_PFD0_FRAC_OFFSET,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk pfd1_clk = {
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC0,
	.enable_shift = MXC_ANADIG_PFD1_FRAC_OFFSET,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk pfd2_clk = {
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC0,
	.enable_shift = MXC_ANADIG_PFD2_FRAC_OFFSET,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk pfd3_clk = {
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC0,
	.enable_shift = MXC_ANADIG_PFD3_FRAC_OFFSET,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk pfd4_clk = {
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC1,
	.enable_shift = MXC_ANADIG_PFD4_FRAC_OFFSET,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk pfd5_clk = {
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC1,
	.enable_shift = MXC_ANADIG_PFD5_FRAC_OFFSET,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk pfd6_clk = {
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC1,
	.enable_shift = MXC_ANADIG_PFD6_FRAC_OFFSET,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk pfd7_clk = {
	.parent = &apll_clk,
	.enable_reg = (void *)MXC_ANADIG_FRAC1,
	.enable_shift = MXC_ANADIG_PFD7_FRAC_OFFSET,
	.set_rate = pfd_set_rate,
	.round_rate = pfd_round_rate,
	.enable = pfd_enable,
	.disable = pfd_disable,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
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
		if (!WAIT(__raw_readl(pllbase + MXC_PLL_DP_CTL)
			  & MXC_PLL_DP_CTL_LRF, SPIN_DELAY))
			panic("pll_set_rate: pll relock failed\n");
	}
	return 0;
}

static int _clk_pll_enable(struct clk *clk)
{
	u32 reg;
	void __iomem *pllbase;

	pllbase = _get_pll_base(clk);
	reg = __raw_readl(pllbase + MXC_PLL_DP_CTL);

	if (reg & MXC_PLL_DP_CTL_UPEN)
		return 0;

	reg |=  MXC_PLL_DP_CTL_UPEN;
	__raw_writel(reg, pllbase + MXC_PLL_DP_CTL);

	/* Wait for lock */
	if (!WAIT(__raw_readl(pllbase + MXC_PLL_DP_CTL) & MXC_PLL_DP_CTL_LRF,
				SPIN_DELAY))
		panic("pll relock failed\n");
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
		__raw_writel(reg, MXC_CCM_CCSR);
		/* Set the step_clk parent to be lp_apm, to save power. */
		mux = _get_mux(&lp_apm_clk, &lp_apm_clk, NULL, &pll2_sw_clk,
			       &pll3_sw_clk);
		reg = __raw_readl(MXC_CCM_CCSR);
		reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
		    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
	} else {
		if (parent == &lp_apm_clk) {
			mux = _get_mux(parent, &lp_apm_clk, NULL, &pll2_sw_clk,
				       &pll3_sw_clk);
			reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
			    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
			__raw_writel(reg, MXC_CCM_CCSR);
			reg = __raw_readl(MXC_CCM_CCSR);
			reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
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

/* TODO: Need to sync with GPC to determine if DVFS is in place so that
 * the DVFS_PODF divider can be applied in CDCR register.
 */
static unsigned long _clk_main_bus_get_rate(struct clk *clk)
{
	u32 div = 0;

	if (med_bus_freq_mode)
		div  = (__raw_readl(MXC_CCM_CDCR) & 0x3);
	return clk_get_rate(clk->parent) / (div + 1);
}

static int _clk_main_bus_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	struct timespec nstimeofday;
	struct timespec curtime;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			&lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CBCDR) & ~MX50_CCM_CBCDR_PERIPH_CLK_SEL_MASK;
	reg |= (mux << MX50_CCM_CBCDR_PERIPH_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) &
			MXC_CCM_CDHIPR_PERIPH_CLK_SEL_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("_clk_main_bus_set_parent failed\n");
	}

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
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AXI_A_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AXI_A_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	if (!WAIT(!(__raw_readl(MXC_CCM_CDHIPR)
	     & MXC_CCM_CDHIPR_AXI_A_PODF_BUSY), SPIN_DELAY))
		panic("pll _clk_axi_a_set_rate failed\n");

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

	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;

	return parent_rate / div;
}


static struct clk axi_a_clk = {
	.parent = &main_bus_clk,
	.get_rate = _clk_axi_a_get_rate,
	.set_rate = _clk_axi_a_set_rate,
	.round_rate = _clk_axi_a_round_rate,
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
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AXI_B_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AXI_B_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	if (!WAIT(!(__raw_readl(MXC_CCM_CDHIPR)
	     & MXC_CCM_CDHIPR_AXI_B_PODF_BUSY), SPIN_DELAY))
		panic("_clk_axi_b_set_rate failed\n");

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

	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;

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
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AHB_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AHB_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	if (!WAIT(!(__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_AHB_PODF_BUSY),
				SPIN_DELAY))
			panic("_clk_ahb_set_rate failed\n");

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
	if (parent_rate / div > MAX_AHB_CLK)
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
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.enable = _clk_max_enable,
	.disable = _clk_max_disable,
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

static int _clk_sys_clk_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CLK_SYS);
	reg &= ~(MXC_CCM_CLK_SYS_SYS_XTAL_CLKGATE_MASK |
				MXC_CCM_CLK_SYS_SYS_PLL_CLKGATE_MASK);
	if (__raw_readl(MXC_CCM_CLKSEQ_BYPASS) & 0x1)
		reg |= MXC_CCM_CLK_SYS_SYS_PLL_CLKGATE_MASK;
	else
		reg |= MXC_CCM_CLK_SYS_SYS_XTAL_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_CLK_SYS);
	return 0;
}

static void _clk_sys_clk_disable(struct clk *clk)
{
	u32 reg, reg1;

	reg1 = (__raw_readl(databahn + DATABAHN_CTL_REG55))
					& DDR_SYNC_MODE;
	reg = __raw_readl(MXC_CCM_CLK_SYS);
	reg &= ~(MXC_CCM_CLK_SYS_SYS_XTAL_CLKGATE_MASK |
				MXC_CCM_CLK_SYS_SYS_PLL_CLKGATE_MASK);
	if (__raw_readl(MXC_CCM_CLKSEQ_BYPASS) & 0x1)
		reg |= 1 << MXC_CCM_CLK_SYS_SYS_PLL_CLKGATE_OFFSET;
	else {
		/* If DDR is sourced from SYS_CLK (in Sync mode), we cannot
		 * gate its clock when ARM is in wait if the DDR is not in
		 * self refresh.
		 */
		if (reg1 == DDR_SYNC_MODE)
			reg |= 3 << MXC_CCM_CLK_SYS_SYS_XTAL_CLKGATE_OFFSET;
		else
			reg |= 1 << MXC_CCM_CLK_SYS_SYS_XTAL_CLKGATE_OFFSET;
	}
	__raw_writel(reg, MXC_CCM_CLK_SYS);
}

static struct clk sys_clk = {
	.enable = _clk_sys_clk_enable,
	.disable = _clk_sys_clk_disable,
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

static int _clk_weim_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;
	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_EMI_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_EMI_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);
	if (!WAIT(!(__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_EMI_PODF_BUSY),
		  SPIN_DELAY))
		panic("_clk_emi_slow_set_rate failed\n");

	return 0;
}

static unsigned long _clk_weim_round_rate(struct clk *clk,
					      unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return parent_rate / div;
}

static struct clk weim_clk[] = {
	{
	.parent = &main_bus_clk,
	.set_parent = _clk_weim_set_parent,
	.set_rate = _clk_weim_set_rate,
	.round_rate = _clk_weim_round_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.disable = _clk_disable_inwait,
	.flags = RATE_PROPAGATES,
	.secondary = &weim_clk[1],
	},
	{
	.parent = &ipg_clk,
	.secondary = &sys_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.disable = _clk_disable_inwait,
	}
};

static int _clk_ocram_enable(struct clk *clk)
{
	return 0;
}

static void _clk_ocram_disable(struct clk *clk)
{
}

static struct clk ocram_clk = {
	.parent = &sys_clk,
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
	 .parent = &ahb_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	 .enable = _clk_sdma_enable,
	 .disable = _clk_sdma_disable,
	 },
	{
	 .parent = &ipg_clk,
#ifdef CONFIG_SDMA_IRAM
	 .secondary = &ocram_clk,
#else
	 .secondary = &ddr_clk,
#endif
	 },
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
	 .enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
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
	 .secondary = &ocram_clk,
#else
	 .secondary = &ddr_clk,
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
	 .secondary = &ocram_clk,
#else
	 .secondary = &ddr_clk,
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
		if (prediv == 1)
			BUG();
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

static struct clk tmax2_clk = {
	 .id = 0,
	 .parent = &ahb_clk,
	 .secondary = &ahb_max_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	 .disable = _clk_disable,
};

static struct clk usb_ahb_clk = {
	 .parent = &ipg_clk,
	 .secondary = &ddr_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	 .disable = _clk_disable,
	.flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk usb_phy_clk[] = {
	{
	.id = 0,
	.parent = &osc_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	.disable = _clk_disable,
	},
	{
	.id = 1,
	.parent = &osc_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.disable = _clk_disable,
	}
};

static struct clk digctl_clk = {
	.id = 0,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.disable = _clk_disable,
};

static struct clk esdhc_dep_clks = {
	 .parent = &spba_clk,
	 .secondary = &ddr_clk,
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

	reg = __raw_readl(MXC_CCM_CSCMR1);
	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = reg & ~MX50_CCM_CSCMR1_ESDHC1_CLK_SEL_MASK;
	reg |= mux << MX50_CCM_CSCMR1_ESDHC1_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}


static int _clk_esdhc1_set_rate(struct clk *clk, unsigned long rate)
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
	 .set_rate = _clk_esdhc1_set_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc1_clk[1],
	.flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
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
	 .parent = &tmax2_clk,
	 .secondary = &esdhc_dep_clks,
	 },

};

static int _clk_esdhc2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &esdhc1_clk[0])
		reg &= ~MX50_CCM_CSCMR1_ESDHC2_CLK_SEL;
	else if (parent == &esdhc3_clk[0])
		reg |= MX50_CCM_CSCMR1_ESDHC2_CLK_SEL;
	else
		BUG();
	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static struct clk esdhc2_clk[] = {
	{
	 .id = 1,
	 .parent = &esdhc1_clk[0],
	 .set_parent = _clk_esdhc2_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc2_clk[1],
	.flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
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

	reg = __raw_readl(MXC_CCM_CSCMR1);
	mux = _get_mux8(parent, &pll1_sw_clk, &pll2_sw_clk,
			&pll3_sw_clk, &lp_apm_clk, &pfd0_clk,
			&pfd1_clk, &pfd4_clk, &osc_clk);
	reg = reg & ~MX50_CCM_CSCMR1_ESDHC3_CLK_SEL_MASK;
	reg |= mux << MX50_CCM_CSCMR1_ESDHC3_CLK_SEL_OFFSET;
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

static int _clk_esdhc3_set_rate(struct clk *clk, unsigned long rate)
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
		~(MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_MASK |
		MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_OFFSET;
	 __raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}


static struct clk esdhc3_clk[] = {
	{
	 .id = 2,
	 .parent = &pll2_sw_clk,
	 .set_parent = _clk_esdhc3_set_parent,
	 .get_rate = _clk_esdhc3_get_rate,
	 .set_rate = _clk_esdhc3_set_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc3_clk[1],
	.flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
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

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &esdhc1_clk[0])
		reg &= ~MX50_CCM_CSCMR1_ESDHC4_CLK_SEL;
	else if (parent == &esdhc3_clk[0])
		reg |= MX50_CCM_CSCMR1_ESDHC4_CLK_SEL;
	else
		BUG();
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
	.flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
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
	__raw_writel(reg, MXC_CCM_CLK_DDR);
	return 0;
}

static unsigned long _clk_ddr_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CLK_DDR);
	div = (reg & MXC_CCM_CLK_DDR_DDR_DIV_PLL_MASK) >>
		MXC_CCM_CLK_DDR_DDR_DIV_PLL_OFFSET;
	if (div)
		return clk_get_rate(clk->parent) / div;

	return 0;
}

static int _clk_ddr_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);
	reg = (__raw_readl(databahn + DATABAHN_CTL_REG55)) &
				DDR_SYNC_MODE;
	if (reg != DDR_SYNC_MODE) {
		reg = __raw_readl(MXC_CCM_CLK_DDR);
		reg |= MXC_CCM_CLK_DDR_DDR_CLKGATE_MASK;
		__raw_writel(reg, MXC_CCM_CLK_DDR);
	}
	return 0;
}

static void _clk_ddr_disable(struct clk *clk)
{
	_clk_disable_inwait(clk);
}


static struct clk ddr_clk = {
	.parent = &pll1_sw_clk,
	.secondary = &sys_clk,
	.set_parent = _clk_ddr_set_parent,
	.get_rate = _clk_ddr_get_rate,
	.enable = _clk_ddr_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.disable = _clk_ddr_disable,
};

static unsigned long _clk_usb_get_rate(struct clk *clk)
{
	return 60000000;
}

/*usb OTG clock */
static struct clk usb_clk = {
	.get_rate = _clk_usb_get_rate,
};

static struct clk rtc_clk = {
	.parent = &ckil_clk,
	.secondary = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.disable = _clk_disable,
};

struct clk rng_clk = {
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.disable = _clk_disable,
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
	 .parent = &aips_tz2_clk,
	 .secondary = &ddr_clk,
	},
};

static int gpmi_clk_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_GPMI);
	reg |= MXC_CCM_GPMI_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_GPMI);
	_clk_enable(clk);
	return 0;
}

static void gpmi_clk_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_GPMI);
	reg &= ~MXC_CCM_GPMI_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_GPMI);
	_clk_disable(clk);
}

static int bch_clk_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_BCH);
	reg |= MXC_CCM_BCH_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_BCH);
	_clk_enable(clk);
	return 0;
}

static void bch_clk_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_BCH);
	reg &= ~MXC_CCM_BCH_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_BCH);
	_clk_disable(clk);
}

static int gpmi_set_parent(struct clk *clk, struct clk *parent)
{
	/* Setting for ONFI nand which need PLL1(800MHZ) */
	if (parent == &pll1_main_clk) {
		u32 reg = __raw_readl(MXC_CCM_CLKSEQ_BYPASS);

		reg = (reg & ~MXC_CCM_CLKSEQ_BYPASS_BYPASS_GPMI_CLK_SEL_MASK) |
		   (0x2 << MXC_CCM_CLKSEQ_BYPASS_BYPASS_GPMI_CLK_SEL_OFFSET);
		reg = (reg & ~MXC_CCM_CLKSEQ_BYPASS_BYPASS_BCH_CLK_SEL_MASK) |
		   (0x2 << MXC_CCM_CLKSEQ_BYPASS_BYPASS_BCH_CLK_SEL_OFFSET);

		__raw_writel(reg, MXC_CCM_CLKSEQ_BYPASS);

		/* change to the new Parent */
		clk->parent = parent;
	} else
		printk(KERN_WARNING "You should not call the %s\n", __func__);
	return 0;
}

static int gpmi_set_rate(struct clk *clk, unsigned long rate)
{
	/* Setting for ONFI nand which in different mode */
	if (clk->parent == &pll1_main_clk) {
		u32 value;
		u32 reg;

		value = clk_get_rate(clk->parent);
		value /= rate;
		value /= 2; /* HW_GPMI_CTRL1's GPMI_CLK_DIV2_EN will be set */

		/* set GPMI clock */
		reg = __raw_readl(MXC_CCM_GPMI);
		reg = (reg & ~MXC_CCM_GPMI_CLK_DIV_MASK) | value;
		__raw_writel(reg, MXC_CCM_GPMI);

		/* set BCH clock */
		reg = __raw_readl(MXC_CCM_BCH);
		reg = (reg & ~MXC_CCM_BCH_CLK_DIV_MASK) | value;
		__raw_writel(reg, MXC_CCM_BCH);
	} else
		printk(KERN_WARNING "You should not call the %s\n", __func__);
	return 0;
}

static struct clk gpmi_nfc_clk[] = {
	{	/* gpmi_io_clk */
	.parent = &osc_clk,
	.secondary = &gpmi_nfc_clk[1],
	.set_parent = gpmi_set_parent,
	.set_rate   = gpmi_set_rate,
	.enable = gpmi_clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.disable = gpmi_clk_disable,
	},
	{	/* gpmi_apb_clk */
	.parent = &apbh_dma_clk,
	.secondary = &gpmi_nfc_clk[2],
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.disable = _clk_disable,
	},
	{	/* bch_clk */
	.parent = &osc_clk,
	.secondary = &gpmi_nfc_clk[3],
	.enable = bch_clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.disable = bch_clk_disable,
	},
	{	/* bch_apb_clk */
	.parent = &apbh_dma_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.disable = _clk_disable,
	},
};

static struct clk ocotp_clk = {
	.parent = &ahb_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.disable = _clk_disable,
};

static int _clk_gpu2d_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &weim_clk[0], &ahb_clk);
	reg = (reg & ~MXC_CCM_CBCMR_GPU2D_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_GPU2D_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk gpu2d_clk = {
	.parent = &axi_a_clk,
	.secondary = &ddr_clk,
	.set_parent = _clk_gpu2d_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk apbh_dma_clk = {
	.parent = &ahb_clk,
	.secondary = &ddr_clk,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
};

struct clk dcp_clk = {
	.parent = &ahb_clk,
	.secondary = &apbh_dma_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.disable = _clk_disable,
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

static unsigned long _clk_display_axi_get_rate(struct clk *clk)
{
	u32 div;

	div = __raw_readl(MXC_CCM_DISPLAY_AXI);
	div &= MXC_CCM_DISPLAY_AXI_DIV_MASK;
	if (div == 0) { /* gated off */
		return clk_get_rate(clk->parent);
	} else {
		return clk_get_rate(clk->parent) / div;
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
	_clk_round_rate_div(clk, rate, max_div, &new_div);

	reg = __raw_readl(MXC_CCM_DISPLAY_AXI);
	reg &= ~MXC_CCM_DISPLAY_AXI_DIV_MASK;
	reg |= new_div << MXC_CCM_DISPLAY_AXI_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_DISPLAY_AXI);

	while (__raw_readl(MXC_CCM_CSR2) & MXC_CCM_CSR2_DISPLAY_AXI_BUSY)
		;
	return 0;
}

static struct clk display_axi_clk = {
	.parent = &osc_clk,
	.secondary = &apbh_dma_clk,
	.set_parent = _clk_display_axi_set_parent,
	.get_rate = _clk_display_axi_get_rate,
	.set_rate = _clk_display_axi_set_rate,
	.round_rate = _clk_display_axi_round_rate,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_DISPLAY_AXI,
	.enable_shift = MXC_CCM_DISPLAY_AXI_CLKGATE_OFFSET,
	.flags = RATE_PROPAGATES | AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static int _clk_pxp_axi_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);

	/* Set the auto-slow bits */
	reg = __raw_readl(MXC_CCM_DISPLAY_AXI);
	reg |= (MXC_CCM_DISPLAY_AXI_PXP_ASM_EN);
	reg |= (5 << MXC_CCM_DISPLAY_AXI_PXP_ASM_DIV_OFFSET);
	__raw_writel(reg, MXC_CCM_DISPLAY_AXI);

	return 0;
}

static void _clk_pxp_axi_disable(struct clk *clk)
{
	u32 reg;

	/* clear the auto-slow bits */
	reg = __raw_readl(MXC_CCM_DISPLAY_AXI);
	reg &= ~MXC_CCM_DISPLAY_AXI_PXP_ASM_EN;
	__raw_writel(reg, MXC_CCM_DISPLAY_AXI);

	_clk_disable(clk);
}


/* TODO: check Auto-Slow Mode */
static struct clk pxp_axi_clk = {
	.parent = &display_axi_clk,
	.enable = _clk_pxp_axi_enable,
	.disable = _clk_pxp_axi_disable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk elcdif_axi_clk = {
	.parent = &display_axi_clk,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
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

static unsigned long _clk_elcdif_pix_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_ELCDIFPIX);
	prediv = ((reg & MXC_CCM_ELCDIFPIX_CLK_PRED_MASK) >>
		  MXC_CCM_ELCDIFPIX_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_ELCDIFPIX_CLK_PODF_MASK) >>
		MXC_CCM_ELCDIFPIX_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
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
	_clk_round_rate_div(clk, rate, max_div, &new_div);

	reg = __raw_readl(MXC_CCM_ELCDIFPIX);
	/* Pre-divider set to 1 - only use PODF for clk dividing */
	reg &= ~MXC_CCM_ELCDIFPIX_CLK_PRED_MASK;
	reg |= 1 << MXC_CCM_ELCDIFPIX_CLK_PRED_OFFSET;
	reg &= ~MXC_CCM_ELCDIFPIX_CLK_PODF_MASK;
	reg |= new_div << MXC_CCM_ELCDIFPIX_CLK_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_ELCDIFPIX);

	return 0;
}

static int _clk_elcdif_pix_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);
	reg = __raw_readl(MXC_CCM_ELCDIFPIX);
	reg |= 0x3 << MXC_CCM_ELCDIFPIX_CLKGATE_OFFSET;
	__raw_writel(reg, MXC_CCM_ELCDIFPIX);
	return 0;
}

static void _clk_elcdif_pix_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_ELCDIFPIX);
	reg &= ~MXC_CCM_ELCDIFPIX_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_ELCDIFPIX);
	_clk_disable(clk);
}

static struct clk elcdif_pix_clk = {
	.parent = &osc_clk,
	.secondary = &ddr_clk,
	.enable = _clk_elcdif_pix_enable,
	.disable = _clk_elcdif_pix_disable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.set_parent = _clk_elcdif_pix_set_parent,
	.get_rate = _clk_elcdif_pix_get_rate,
	.round_rate = _clk_elcdif_pix_round_rate,
	.set_rate = _clk_elcdif_pix_set_rate,
	.flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
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

static unsigned long _clk_epdc_axi_get_rate(struct clk *clk)
{
	u32 div;

	div = __raw_readl(MXC_CCM_EPDC_AXI);
	div &= MXC_CCM_EPDC_AXI_DIV_MASK;
	if (div == 0) { /* gated off */
		return clk_get_rate(clk->parent);
	} else {
		return clk_get_rate(clk->parent) / div;
	}
}

static unsigned long _clk_epdc_axi_round_rate_div(struct clk *clk,
						unsigned long rate,
						u32 *new_div)
{
	u32 div, max_div;

	max_div = (2 << 6) - 1;
	div = DIV_ROUND_UP(clk_get_rate(clk->parent), rate);
	if (div > max_div)
		div = max_div;
	else if (div == 0)
		div++;
	if (new_div != NULL)
		*new_div = div;
	return clk_get_rate(clk->parent) / div;
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

	_clk_epdc_axi_round_rate_div(clk, rate, &new_div);

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

	_clk_enable(clk);

	reg = __raw_readl(MXC_CCM_EPDC_AXI);
	reg |= MXC_CCM_EPDC_AXI_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_EPDC_AXI);

	/* Set the auto-slow bits */
	reg = __raw_readl(MXC_CCM_EPDC_AXI);
	reg |= (MXC_CCM_EPDC_AXI_ASM_EN);
	reg |= (5 << MXC_CCM_EPDC_AXI_ASM_DIV_OFFSET);
	__raw_writel(reg, MXC_CCM_EPDC_AXI);

	return 0;
}

static void _clk_epdc_axi_disable(struct clk *clk)
{
	u32 reg;

	/* clear the auto-slow bits */
	reg = __raw_readl(MXC_CCM_EPDC_AXI);
	reg &= ~MXC_CCM_EPDC_AXI_ASM_EN;
	__raw_writel(reg, MXC_CCM_EPDC_AXI);

	reg = __raw_readl(MXC_CCM_EPDC_AXI);
	reg &= ~MXC_CCM_EPDC_AXI_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_EPDC_AXI);
	_clk_disable(clk);
}

/* TODO: check Auto-Slow Mode */
static struct clk epdc_axi_clk = {
	.parent = &osc_clk,
	.secondary = &apbh_dma_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.set_parent = _clk_epdc_axi_set_parent,
	.get_rate = _clk_epdc_axi_get_rate,
	.set_rate = _clk_epdc_axi_set_rate,
	.round_rate = _clk_epdc_axi_round_rate,
	.enable = _clk_epdc_axi_enable,
	.disable = _clk_epdc_axi_disable,
	.flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
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

static unsigned long _clk_epdc_pix_get_rate(struct clk *clk)
{
	u32 div;

	div = __raw_readl(MXC_CCM_EPDCPIX);
	div &= MXC_CCM_EPDC_PIX_CLK_PODF_MASK;
	if (div == 0) { /* gated off */
		return clk_get_rate(clk->parent);
	} else {
		return clk_get_rate(clk->parent) / div;
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
	_clk_round_rate_div(clk, rate, max_div, &new_div);

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

	_clk_enable(clk);
	reg = __raw_readl(MXC_CCM_EPDCPIX);
	reg |= MXC_CCM_EPDC_PIX_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_EPDCPIX);

	return 0;
}

static void _clk_epdc_pix_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_EPDCPIX);
	reg &= ~MXC_CCM_EPDC_PIX_CLKGATE_MASK;
	__raw_writel(reg, MXC_CCM_EPDCPIX);
	_clk_disable(clk);
}

/* TODO: check Auto-Slow Mode */
static struct clk epdc_pix_clk = {
	.parent = &osc_clk,
	.secondary = &apbh_dma_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	.set_parent = _clk_epdc_pix_set_parent,
	.get_rate = _clk_epdc_pix_get_rate,
	.set_rate = _clk_epdc_pix_set_rate,
	.round_rate = _clk_epdc_pix_round_rate,
	.enable = _clk_epdc_pix_enable,
	.disable = _clk_epdc_pix_disable,
	.flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static unsigned long cko1_get_rate(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= MX50_CCM_CCOSR_CKO1_DIV_MASK;
	reg = reg >> MX50_CCM_CCOSR_CKO1_DIV_OFFSET;
	return clk_get_rate(clk->parent) / (reg + 1);
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
	u32 parent_rate = clk_get_rate(clk->parent);

	div = (parent_rate/rate - 1) & 0x7;
	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MX50_CCM_CCOSR_CKO1_DIV_MASK;
	reg |= div << MX50_CCM_CCOSR_CKO1_DIV_OFFSET;
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
	} else if (parent == &weim_clk[0]) {
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
	.parent = &pll1_sw_clk,
	.get_rate = cko1_get_rate,
	.enable = cko1_enable,
	.disable = cko1_disable,
	.set_rate = cko1_set_rate,
	.round_rate = cko1_round_rate,
	.set_parent = cko1_set_parent,
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
	_REGISTER_CLOCK(NULL, "apll", apll_clk),
	_REGISTER_CLOCK(NULL, "pfd0", pfd0_clk),
	_REGISTER_CLOCK(NULL, "pfd1", pfd1_clk),
	_REGISTER_CLOCK(NULL, "pfd2", pfd2_clk),
	_REGISTER_CLOCK(NULL, "pfd3", pfd3_clk),
	_REGISTER_CLOCK(NULL, "pfd4", pfd4_clk),
	_REGISTER_CLOCK(NULL, "pfd5", pfd5_clk),
	_REGISTER_CLOCK(NULL, "pfd6", pfd6_clk),
	_REGISTER_CLOCK(NULL, "pfd7", pfd7_clk),
	_REGISTER_CLOCK(NULL, "gpc_dvfs_clk", gpc_dvfs_clk),
	_REGISTER_CLOCK(NULL, "lp_apm", lp_apm_clk),
	_REGISTER_CLOCK(NULL, "cpu_clk", cpu_clk),
	_REGISTER_CLOCK(NULL, "main_bus_clk", main_bus_clk),
	_REGISTER_CLOCK(NULL, "axi_a_clk", axi_a_clk),
	_REGISTER_CLOCK(NULL, "axi_b_clk", axi_b_clk),
	_REGISTER_CLOCK(NULL, "ahb_clk", ahb_clk),
	_REGISTER_CLOCK(NULL, "ahb_max_clk", ahb_max_clk),
	_REGISTER_CLOCK("mxc_sdma", "sdma_ahb_clk", sdma_clk[0]),
	_REGISTER_CLOCK("mxc_sdma", "sdma_ipg_clk", sdma_clk[1]),
	_REGISTER_CLOCK("mxcintuart.0", NULL, uart1_clk[0]),
	_REGISTER_CLOCK("mxcintuart.1", NULL, uart2_clk[0]),
	_REGISTER_CLOCK("mxcintuart.2", NULL, uart3_clk[0]),
	_REGISTER_CLOCK("mxcintuart.3", NULL, uart4_clk[0]),
	_REGISTER_CLOCK("mxcintuart.4", NULL, uart5_clk[0]),
	_REGISTER_CLOCK("imx-i2c.0", NULL, i2c_clk[0]),
	_REGISTER_CLOCK("imx-i2c.1", NULL, i2c_clk[1]),
	_REGISTER_CLOCK("imx-i2c.2", NULL, i2c_clk[2]),
	_REGISTER_CLOCK("mxc_pwm.0", NULL, pwm1_clk[0]),
	_REGISTER_CLOCK("mxc_pwm.1", NULL, pwm2_clk[0]),
	_REGISTER_CLOCK("mxc_spi.0", NULL, cspi1_clk[0]),
	_REGISTER_CLOCK("mxc_spi.1", NULL, cspi2_clk[0]),
	_REGISTER_CLOCK("mxc_spi.2", NULL, cspi3_clk),
	_REGISTER_CLOCK(NULL, "ssi_lp_apm_clk", ssi_lp_apm_clk),
	_REGISTER_CLOCK("mxc_ssi.0", NULL, ssi1_clk[0]),
	_REGISTER_CLOCK("mxc_ssi.1", NULL, ssi2_clk[0]),
	_REGISTER_CLOCK(NULL, "ssi_ext1_clk", ssi_ext1_clk),
	_REGISTER_CLOCK(NULL, "ssi_ext2_clk", ssi_ext2_clk),
	_REGISTER_CLOCK(NULL, "usb_ahb_clk", usb_ahb_clk),
	_REGISTER_CLOCK(NULL, "usb_phy1_clk", usb_phy_clk[0]),
	_REGISTER_CLOCK(NULL, "usb_phy2_clk", usb_phy_clk[1]),
	_REGISTER_CLOCK(NULL, "usb_clk", usb_clk),
	_REGISTER_CLOCK("mxsdhci.0", NULL, esdhc1_clk[0]),
	_REGISTER_CLOCK("mxsdhci.1", NULL, esdhc2_clk[0]),
	_REGISTER_CLOCK("mxsdhci.2", NULL, esdhc3_clk[0]),
	_REGISTER_CLOCK("mxsdhci.3", NULL, esdhc4_clk[0]),
	_REGISTER_CLOCK(NULL, "digctl_clk", digctl_clk),
	_REGISTER_CLOCK(NULL, "ddr_clk", ddr_clk),
	_REGISTER_CLOCK("mxc_rtc.0", NULL, rtc_clk),
	_REGISTER_CLOCK("mxc_w1.0", NULL, owire_clk),
	_REGISTER_CLOCK(NULL, "gpu2d_clk", gpu2d_clk),
	_REGISTER_CLOCK(NULL, "cko1", cko1_clk),
	_REGISTER_CLOCK(NULL, "gpt", gpt_clk[0]),
	_REGISTER_CLOCK("fec.0", NULL, fec_clk[0]),
	_REGISTER_CLOCK(NULL, "fec_sec1_clk", fec_clk[1]),
	_REGISTER_CLOCK("mxc_w1.0", NULL, owire_clk),
	_REGISTER_CLOCK(NULL, "gpmi-nfc", gpmi_nfc_clk[0]),
	_REGISTER_CLOCK(NULL, "gpmi-apb", gpmi_nfc_clk[1]),
	_REGISTER_CLOCK(NULL, "bch", gpmi_nfc_clk[2]),
	_REGISTER_CLOCK(NULL, "bch-apb", gpmi_nfc_clk[3]),
	_REGISTER_CLOCK(NULL, "rng_clk", rng_clk),
	_REGISTER_CLOCK(NULL, "dcp_clk", dcp_clk),
	_REGISTER_CLOCK(NULL, "ocotp_ctrl_apb", ocotp_clk),
	_REGISTER_CLOCK(NULL, "ocram_clk", ocram_clk),
	_REGISTER_CLOCK(NULL, "apbh_dma_clk", apbh_dma_clk),
	_REGISTER_CLOCK(NULL, "sys_clk", sys_clk),
	_REGISTER_CLOCK(NULL, "elcdif_pix", elcdif_pix_clk),
	_REGISTER_CLOCK(NULL, "display_axi", display_axi_clk),
	_REGISTER_CLOCK(NULL, "elcdif_axi", elcdif_axi_clk),
	_REGISTER_CLOCK(NULL, "pxp_axi", pxp_axi_clk),
	_REGISTER_CLOCK(NULL, "epdc_axi", epdc_axi_clk),
	_REGISTER_CLOCK(NULL, "epdc_pix", epdc_pix_clk),
};

static struct mxc_clk mxc_clks[ARRAY_SIZE(lookups)];

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
	weim_clk[0].parent = &main_bus_clk;
	reg = __raw_readl(MXC_CCM_CBCDR);
	if ((reg & MX50_CCM_CBCDR_WEIM_CLK_SEL) != 0)
		weim_clk[0].parent = &ahb_clk;

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
	int i = 0, j = 0, reg;
	int wp_cnt = 0;
	u32 pll1_rate;

	pll1_base = ioremap(MX53_BASE_ADDR(PLL1_BASE_ADDR), SZ_4K);
	pll2_base = ioremap(MX53_BASE_ADDR(PLL2_BASE_ADDR), SZ_4K);
	pll3_base = ioremap(MX53_BASE_ADDR(PLL3_BASE_ADDR), SZ_4K);
	apll_base = ioremap(ANATOP_BASE_ADDR, SZ_4K);

	/* Turn off all possible clocks */
	if (mxc_jtag_enabled) {
		__raw_writel(1 << MXC_CCM_CCGRx_CG0_OFFSET |
			      3 << MXC_CCM_CCGRx_CG2_OFFSET |
			      3 << MXC_CCM_CCGRx_CG3_OFFSET |
			      3 << MXC_CCM_CCGRx_CG4_OFFSET |
			      3 << MXC_CCM_CCGRx_CG8_OFFSET |
			      1 << MXC_CCM_CCGRx_CG12_OFFSET |
			      1 << MXC_CCM_CCGRx_CG13_OFFSET |
			      1 << MXC_CCM_CCGRx_CG14_OFFSET, MXC_CCM_CCGR0);
	} else {
		__raw_writel(1 << MXC_CCM_CCGRx_CG0_OFFSET |
			      3 << MXC_CCM_CCGRx_CG3_OFFSET |
			      3 << MXC_CCM_CCGRx_CG8_OFFSET |
			      1 << MXC_CCM_CCGRx_CG12_OFFSET |
			      1 << MXC_CCM_CCGRx_CG13_OFFSET |
			      3 << MXC_CCM_CCGRx_CG14_OFFSET, MXC_CCM_CCGR0);
	}

	__raw_writel(0, MXC_CCM_CCGR1);
	__raw_writel(0, MXC_CCM_CCGR2);
	__raw_writel(0, MXC_CCM_CCGR3);
	__raw_writel(0, MXC_CCM_CCGR4);

	__raw_writel(3 << MXC_CCM_CCGRx_CG6_OFFSET |
		     1 << MXC_CCM_CCGRx_CG8_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET, MXC_CCM_CCGR5);

	__raw_writel(3 << MXC_CCM_CCGRx_CG0_OFFSET |
				3 << MXC_CCM_CCGRx_CG1_OFFSET |
				2 << MXC_CCM_CCGRx_CG14_OFFSET |
				3 << MXC_CCM_CCGRx_CG15_OFFSET, MXC_CCM_CCGR6);

	external_low_reference = ckil;
	external_high_reference = ckih1;
	oscillator_reference = osc;

	usb_phy_clk[0].enable_reg = MXC_CCM_CCGR4;
	usb_phy_clk[0].enable_shift = MXC_CCM_CCGRx_CG5_OFFSET;

	clk_tree_init();

	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		clkdev_add(&lookups[i]);
		mxc_clks[i].reg_clk = lookups[i].clk;
		if (lookups[i].con_id != NULL)
			strcpy(mxc_clks[i].name, lookups[i].con_id);
		else
			strcpy(mxc_clks[i].name, lookups[i].dev_id);
		clk_register(&mxc_clks[i]);
	}

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

	clk_enable(&cpu_clk);

	clk_enable(&main_bus_clk);

	clk_enable(&ocotp_clk);

	databahn = ioremap(MX50_DATABAHN_BASE_ADDR, SZ_16K);

	/* Initialise the parents to be axi_b, parents are set to
	 * axi_a when the clocks are enabled.
	 */

	clk_set_parent(&gpu2d_clk, &axi_a_clk);

	/* move cspi to 24MHz */
	clk_set_parent(&cspi_main_clk, &lp_apm_clk);
	clk_set_rate(&cspi_main_clk, 12000000);

	/*
	 * Set DISPLAY_AXI to 200Mhz
	 * For Display AXI, source clocks must be
	 * enabled before dividers can be changed
	 */
	clk_enable(&display_axi_clk);
	clk_enable(&elcdif_axi_clk);
	clk_enable(&pxp_axi_clk);
	clk_set_parent(&display_axi_clk, &pfd2_clk);
	clk_set_rate(&display_axi_clk, 200000000);
	clk_disable(&display_axi_clk);
	clk_disable(&pxp_axi_clk);
	clk_disable(&elcdif_axi_clk);

	/* Set the SELF-BIAS bit. */
	__raw_writel(MXC_ANADIG_REF_SELFBIAS_OFF,
					apll_base + MXC_ANADIG_MISC_SET);

	clk_enable(&elcdif_pix_clk);
	clk_set_parent(&elcdif_pix_clk, &pll1_sw_clk);
	clk_disable(&elcdif_pix_clk);

	/*
	 * Enable and set EPDC AXI to 200MHz
	 * For EPDC AXI, source clocks must be
	 * enabled before dividers can be changed
	 */
	clk_enable(&epdc_axi_clk);
	clk_set_parent(&epdc_axi_clk, &pfd3_clk);
	clk_set_rate(&epdc_axi_clk, 200000000);
	clk_disable(&epdc_axi_clk);

	clk_set_parent(&epdc_pix_clk, &pfd5_clk);

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

	/* move usb_phy_clk to 24MHz */
	clk_set_parent(&usb_phy_clk[0], &osc_clk);
	clk_set_parent(&usb_phy_clk[1], &osc_clk);

	/* move gpmi-nfc to 24MHz */
	clk_set_parent(&gpmi_nfc_clk[0], &osc_clk);

	/* set SDHC root clock as 200MHZ*/
	clk_set_rate(&esdhc1_clk[0], 200000000);
	clk_set_rate(&esdhc3_clk[0], 200000000);

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
		/* Change the CPU podf divider based on the boot up
		 * pll1 rate.
		 */
		cpu_wp_tbl[j].cpu_podf = max(
			(int)((pll1_rate / cpu_wp_tbl[j].cpu_rate)
			- 1), 0);
		if (pll1_rate/(cpu_wp_tbl[j].cpu_podf + 1) >
				cpu_wp_tbl[j].cpu_rate) {
			cpu_wp_tbl[j].cpu_podf++;
			cpu_wp_tbl[j].cpu_rate =
				 pll1_rate/
				 (1000 * (cpu_wp_tbl[j].cpu_podf + 1));
			cpu_wp_tbl[j].cpu_rate *= 1000;
		}
		if (pll1_rate/(cpu_wp_tbl[j].cpu_podf + 1) <
					cpu_wp_tbl[j].cpu_rate) {
			cpu_wp_tbl[j].cpu_rate = pll1_rate;
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

	clk_set_parent(&uart_main_clk, &lp_apm_clk);
	clk_set_parent(&gpu2d_clk, &axi_b_clk);

	clk_set_parent(&weim_clk[0], &ahb_clk);
	clk_set_rate(&weim_clk[0], clk_round_rate(&weim_clk[0], 130000000));

	/* Do the following just to disable the PLL since its not used */
	clk_enable(&pll3_sw_clk);
	clk_disable(&pll3_sw_clk);

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

#if defined(CONFIG_CPU_FREQ)
	cpufreq_trig_needed = 1;
#endif
	return 0;
}
