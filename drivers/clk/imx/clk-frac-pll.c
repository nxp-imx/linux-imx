// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 NXP.
 *
 * This driver supports the fractional plls found in the imx8m SOCs
 *
 * Documentation for this fractional pll can be found at:
 *   https://www.nxp.com/docs/en/reference-manual/IMX8MDQLQRM.pdf#page=834
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/slab.h>
#include <linux/bitfield.h>

#include "clk.h"

#define PLL_CFG0		0x0
#define PLL_CFG1		0x4

#define PLL_LOCK_STATUS		BIT(31)
#define PLL_PD_MASK		BIT(19)
#define PLL_BYPASS_MASK		BIT(14)
#define PLL_NEWDIV_VAL		BIT(12)
#define PLL_NEWDIV_ACK		BIT(11)
#define PLL_FRAC_DIV_MASK	GENMASK(30, 7)
#define PLL_INT_DIV_MASK	GENMASK(6, 0)
#define PLL_OUTPUT_DIV_MASK	GENMASK(4, 0)
#define PLL_FRAC_DENOM		0x1000000

#define PLL_FRAC_LOCK_TIMEOUT	10000
#define PLL_FRAC_ACK_TIMEOUT	500000

struct clk_frac_pll {
	struct clk_hw	hw;
	void __iomem	*base;
};

#define to_clk_frac_pll(_hw) container_of(_hw, struct clk_frac_pll, hw)

static int clk_wait_lock(struct clk_frac_pll *pll)
{
	u32 val;

	return readl_poll_timeout(pll->base, val, val & PLL_LOCK_STATUS, 0,
					PLL_FRAC_LOCK_TIMEOUT);
}

static int clk_wait_ack(struct clk_frac_pll *pll)
{
	u32 val;

	/* return directly if the pll is in powerdown or in bypass */
	if (readl_relaxed(pll->base) & (PLL_PD_MASK | PLL_BYPASS_MASK))
		return 0;

	/* Wait for the pll's divfi and divff to be reloaded */
	return readl_poll_timeout(pll->base, val, val & PLL_NEWDIV_ACK, 0,
					PLL_FRAC_ACK_TIMEOUT);
}

static int clk_pll_prepare(struct clk_hw *hw)
{
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);
	u32 val;

	val = readl_relaxed(pll->base + PLL_CFG0);
	val &= ~PLL_PD_MASK;
	writel_relaxed(val, pll->base + PLL_CFG0);

	return clk_wait_lock(pll);
}

static void clk_pll_unprepare(struct clk_hw *hw)
{
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);
	u32 val;

	val = readl_relaxed(pll->base + PLL_CFG0);
	val |= PLL_PD_MASK;
	writel_relaxed(val, pll->base + PLL_CFG0);
}

static int clk_pll_is_prepared(struct clk_hw *hw)
{
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);
	u32 val;

	val = readl_relaxed(pll->base + PLL_CFG0);
	return (val & PLL_PD_MASK) ? 0 : 1;
}

static unsigned long clk_pll_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);
	u32 val, divff, divfi, divq;
	u64 temp64 = parent_rate;
	u64 rate;

	val = readl_relaxed(pll->base + PLL_CFG0);
	divq = (FIELD_GET(PLL_OUTPUT_DIV_MASK, val) + 1) * 2;
	val = readl_relaxed(pll->base + PLL_CFG1);
	divff = FIELD_GET(PLL_FRAC_DIV_MASK, val);
	divfi = FIELD_GET(PLL_INT_DIV_MASK, val);

	temp64 *= 8;
	temp64 *= divff;
	do_div(temp64, PLL_FRAC_DENOM);
	do_div(temp64, divq);

	rate = parent_rate * 8 * (divfi + 1);
	do_div(rate, divq);
	rate += temp64;

	return rate;
}

/*
 * Fractional-N PLL operating limits (from PLL block guide, Table 1):
 *
 *   REF input frequency:          10 MHz to 300 MHz  (before DIVR)
 *   Post-DIVR reference (parent): 5 MHz to 7.5 MHz
 *   VCO frequency:                2000 MHz to 4000 MHz
 *   PLLOUT output frequency:      31.25 MHz to 2000 MHz
 *
 * In the Linux driver the parent_rate passed to set_rate/determine_rate
 * is already the post-DIVR reference (output of video_pll1_ref_div),
 * so it must be in [PLL_REF_MIN_FREQ, PLL_REF_MAX_FREQ].
 */
#define PLL_VCO_MIN_FREQ	2000000000ULL
#define PLL_VCO_MAX_FREQ	4000000000ULL
/* DIVQ range is 2..64 (even), so output range is [VCO_MIN/64 .. VCO_MAX/2]. */
#define PLL_OUT_MIN_FREQ	(PLL_VCO_MIN_FREQ / 64)	/* 31.25 MHz */
#define PLL_OUT_MAX_FREQ	(PLL_VCO_MAX_FREQ / 2)	/* 2000 MHz */
#define PLL_REF_MIN_FREQ	   5000000ULL
#define PLL_REF_MAX_FREQ	   7500000ULL
/*
 * Select the OUTPUT_DIV register value (divq_reg) such that the internal VCO
 * frequency (pllout * divq) falls inside [PLL_VCO_MIN_FREQ, PLL_VCO_MAX_FREQ].
 * divq = (divq_reg + 1) * 2, range [2..64].
 * Iterate from the smallest divq upward; take the first value that brings
 * VCO >= VCO_MIN. If even divq=64 leaves VCO below VCO_MIN (very low rate),
 * return 31 (divq=64) as the best effort and let the caller warn.
 */
static u32 clk_pll_calc_divq_reg(unsigned long rate)
{
	u32 divq_reg;

	for (divq_reg = 0; divq_reg <= 31; divq_reg++) {
		u64 vco = (u64)rate * ((divq_reg + 1) * 2);

		if (vco >= PLL_VCO_MIN_FREQ)
			break;
	}
	return divq_reg;
}

static int clk_pll_determine_rate(struct clk_hw *hw,
				  struct clk_rate_request *req)
{
	u64 parent_rate = req->best_parent_rate;
	u32 divff, divfi, divq_reg;
	u64 temp64, prate8, divq;

	divq_reg = clk_pll_calc_divq_reg(req->rate);
	divq = (u64)(divq_reg + 1) * 2;

	prate8 = parent_rate * 8;
	temp64 = (u64)req->rate * divq;
	do_div(temp64, prate8);
	divfi = temp64;
	temp64 = (u64)req->rate * divq - (u64)divfi * prate8;
	temp64 *= PLL_FRAC_DENOM;
	do_div(temp64, prate8);
	divff = temp64;

	temp64 = prate8 * divff;
	do_div(temp64, PLL_FRAC_DENOM);

	temp64 += prate8 * divfi;
	do_div(temp64, divq);
	req->rate = temp64;

	return 0;
}

/*
 * PLL output formula:
 * pllout = parent_rate * 8 * DIVF_VAL / divq
 * where DIVF_VAL = divfi + divff / 2^24
 * and divq = (OUTPUT_DIV + 1) * 2.
 * Choose the smallest OUTPUT_DIV that keeps VCO >= PLL_VCO_MIN_FREQ.
 */
static int clk_pll_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);
	u32 val, divfi, divff, divq_reg;
	u64 prate8, temp64, divq, vco;
	int ret;

	divq_reg = clk_pll_calc_divq_reg(rate);
	divq = (u64)(divq_reg + 1) * 2;

	prate8 = (u64)parent_rate * 8;
	temp64 = (u64)rate * divq;
	do_div(temp64, prate8);
	divfi = temp64;
	temp64 = (u64)rate * divq - (u64)divfi * prate8;
	temp64 *= PLL_FRAC_DENOM;
	do_div(temp64, prate8);
	divff = temp64;

	vco = (u64)rate * divq;

	/* Enforce PLL operating limits (PLL block guide, Table 1). */
	if (rate < PLL_OUT_MIN_FREQ || rate > PLL_OUT_MAX_FREQ)
		pr_warn("%s: output rate %lu Hz out of range [%llu, %llu]\n",
			clk_hw_get_name(hw), rate,
			PLL_OUT_MIN_FREQ, PLL_OUT_MAX_FREQ);

	if (parent_rate < PLL_REF_MIN_FREQ || parent_rate > PLL_REF_MAX_FREQ)
		pr_warn("%s: post-DIVR ref rate %lu Hz out of range [%llu, %llu]\n",
			clk_hw_get_name(hw), parent_rate,
			PLL_REF_MIN_FREQ, PLL_REF_MAX_FREQ);

	if (vco < PLL_VCO_MIN_FREQ || vco > PLL_VCO_MAX_FREQ)
		pr_warn("%s: VCO %llu Hz out of range [%llu, %llu]\n",
			clk_hw_get_name(hw), vco,
			PLL_VCO_MIN_FREQ, PLL_VCO_MAX_FREQ);

	pr_debug("%s: rate=%lu parent=%lu divq_reg=%u divq=%llu divfi=%u divff=%u vco=%llu\n",
		 clk_hw_get_name(hw), rate, parent_rate,
		 divq_reg, divq, divfi, divff, vco);

	val = readl_relaxed(pll->base + PLL_CFG1);
	val &= ~(PLL_FRAC_DIV_MASK | PLL_INT_DIV_MASK);
	val |= (divff << 7) | (divfi - 1);
	writel_relaxed(val, pll->base + PLL_CFG1);

	val = readl_relaxed(pll->base + PLL_CFG0);
	val &= ~PLL_OUTPUT_DIV_MASK;
	val |= divq_reg;
	writel_relaxed(val, pll->base + PLL_CFG0);

	/* Set the PLL_NEWDIV_VAL to reload the DIVFI and DIVFF */
	val = readl_relaxed(pll->base + PLL_CFG0);
	val |= PLL_NEWDIV_VAL;
	writel_relaxed(val, pll->base + PLL_CFG0);

	ret = clk_wait_ack(pll);

	/* clear the PLL_NEWDIV_VAL */
	val = readl_relaxed(pll->base + PLL_CFG0);
	val &= ~PLL_NEWDIV_VAL;
	writel_relaxed(val, pll->base + PLL_CFG0);

	return ret;
}

static const struct clk_ops clk_frac_pll_ops = {
	.prepare	= clk_pll_prepare,
	.unprepare	= clk_pll_unprepare,
	.is_prepared	= clk_pll_is_prepared,
	.recalc_rate	= clk_pll_recalc_rate,
	.determine_rate = clk_pll_determine_rate,
	.set_rate	= clk_pll_set_rate,
};

struct clk_hw *imx_clk_hw_frac_pll(const char *name,
				   const char *parent_name,
				   void __iomem *base)
{
	struct clk_init_data init;
	struct clk_frac_pll *pll;
	struct clk_hw *hw;
	int ret;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_frac_pll_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->base = base;
	pll->hw.init = &init;

	hw = &pll->hw;

	ret = clk_hw_register(NULL, hw);
	if (ret) {
		kfree(pll);
		return ERR_PTR(ret);
	}

	return hw;
}
EXPORT_SYMBOL_GPL(imx_clk_hw_frac_pll);
