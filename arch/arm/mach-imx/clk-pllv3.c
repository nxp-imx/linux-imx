/*
 * Copyright 2012-2013 Freescale Semiconductor, Inc.
 * Copyright 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include "clk.h"

#define PLL_NUM_OFFSET		0x10
#define PLL_DENOM_OFFSET	0x20

#define BM_PLL_POWER		(0x1 << 12)
#define BM_PLL_ENABLE		(0x1 << 13)
#define BM_PLL_BYPASS		(0x1 << 16)
#define BM_PLL_LOCK		(0x1 << 31)
#define BYPASS_RATE		24000000
#define BYPASS_MASK	0x10000

/**
 * struct clk_pllv3 - IMX PLL clock version 3
 * @clk_hw:	 clock source
 * @base:	 base address of PLL registers
 * @powerup_set: set POWER bit to power up the PLL
 * @always_on : Leave the PLL powered up all the time.
 * @div_mask:	 mask of divider bits
 *
 * IMX PLL clock version 3, found on i.MX6 series.  Divider for pllv3
 * is actually a multiplier, and always sits at bit 0.
 */
struct clk_pllv3 {
	struct clk_hw	hw;
	void __iomem	*base;
	bool		powerup_set;
	bool		always_on;
	u32		div_mask;
	u32		rate_req;
};

#define to_clk_pllv3(_hw) container_of(_hw, struct clk_pllv3, hw)

static int clk_pllv3_wait_for_lock(struct clk_pllv3 *pll, u32 timeout_ms)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);
	u32 val = readl_relaxed(pll->base) & BM_PLL_POWER;

	/* No need to wait for lock when pll is power down */
	if ((pll->powerup_set && !val) || (!pll->powerup_set && val))
		return 0;

	/* Wait for PLL to lock */
	do {
		if (readl_relaxed(pll->base) & BM_PLL_LOCK)
			break;
		if (time_after(jiffies, timeout))
			break;
	} while (1);

	if (readl_relaxed(pll->base) & BM_PLL_LOCK)
		return 0;
	else
		return -ETIMEDOUT;
}

static int clk_pllv3_power_up_down(struct clk_hw *hw, bool enable)
{
	struct clk_pllv3 *pll = to_clk_pllv3(hw);
	u32 val, ret = 0;

	if (enable) {
		val = readl_relaxed(pll->base);
		val &= ~BM_PLL_BYPASS;
		if (pll->powerup_set)
			val |= BM_PLL_POWER;
		else
			val &= ~BM_PLL_POWER;
		writel_relaxed(val, pll->base);

		ret = clk_pllv3_wait_for_lock(pll, 10);
	} else {
		val = readl_relaxed(pll->base);
		val |= BM_PLL_BYPASS;
		if (pll->powerup_set)
			val &= ~BM_PLL_POWER;
		else
			val |= BM_PLL_POWER;
		writel_relaxed(val, pll->base);
	}
	return ret;
}


static int clk_pllv3_enable(struct clk_hw *hw)
{
	struct clk_pllv3 *pll = to_clk_pllv3(hw);
	u32 val;

	if (pll->rate_req != BYPASS_RATE)
		clk_pllv3_power_up_down(hw, true);

	val = readl_relaxed(pll->base);
	val |= BM_PLL_ENABLE;
	writel_relaxed(val, pll->base);

	return 0;
}

static void clk_pllv3_disable(struct clk_hw *hw)
{
	struct clk_pllv3 *pll = to_clk_pllv3(hw);
	u32 val;

	val = readl_relaxed(pll->base);
	if (!pll->always_on)
		val &= ~BM_PLL_ENABLE;
	writel_relaxed(val, pll->base);

	if (pll->rate_req != BYPASS_RATE)
		clk_pllv3_power_up_down(hw, false);
}

static unsigned long clk_pllv3_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	struct clk_pllv3 *pll = to_clk_pllv3(hw);
	u32 div = readl_relaxed(pll->base)  & pll->div_mask;
	u32 bypass = readl_relaxed(pll->base) & BYPASS_MASK;
	u32 rate;

	if (pll->rate_req == BYPASS_RATE && bypass)
		rate = BYPASS_RATE;
	else
		rate = (div == 1) ? parent_rate * 22 : parent_rate * 20;

	return rate;
}

static long clk_pllv3_round_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long *prate)
{
	unsigned long parent_rate = *prate;

	/* If the PLL is bypassed, its rate is 24MHz. */
	if (rate == BYPASS_RATE)
		return BYPASS_RATE;

	return (rate >= parent_rate * 22) ? parent_rate * 22 :
					    parent_rate * 20;
}

static int clk_pllv3_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_pllv3 *pll = to_clk_pllv3(hw);
	u32 val, div;

	pll->rate_req = rate;
	val = readl_relaxed(pll->base);

	/* If the PLL is bypassed, its rate is 24MHz. */
	if (rate == BYPASS_RATE) {
		/* Set the bypass bit. */
		val |= BM_PLL_BYPASS;
		if (pll->powerup_set)
			val |= BM_PLL_POWER;
		else
			val &= ~BM_PLL_POWER;
		writel_relaxed(val, pll->base);

		return 0;
	}
	if (rate == parent_rate * 22)
		div = 1;
	else if (rate == parent_rate * 20)
		div = 0;
	else
		return -EINVAL;

	val = readl_relaxed(pll->base);
	val &= ~pll->div_mask;
	val |= div;
	writel_relaxed(val, pll->base);

	return clk_pllv3_wait_for_lock(pll, 10);
}

static const struct clk_ops clk_pllv3_ops = {
	.enable		= clk_pllv3_enable,
	.disable	= clk_pllv3_disable,
	.recalc_rate	= clk_pllv3_recalc_rate,
	.round_rate	= clk_pllv3_round_rate,
	.set_rate	= clk_pllv3_set_rate,
};

static unsigned long clk_pllv3_sys_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct clk_pllv3 *pll = to_clk_pllv3(hw);
	u32 div = readl_relaxed(pll->base) & pll->div_mask;
	u32 bypass = readl_relaxed(pll->base) & BYPASS_MASK;

	if (pll->rate_req == BYPASS_RATE && bypass)
		return BYPASS_RATE;

	return parent_rate * div / 2;
}

static long clk_pllv3_sys_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *prate)
{
	unsigned long parent_rate = *prate;
	unsigned long min_rate = parent_rate * 54 / 2;
	unsigned long max_rate = parent_rate * 108 / 2;
	u32 div;

	if (rate == BYPASS_RATE)
		return BYPASS_RATE;

	if (rate > max_rate)
		rate = max_rate;
	else if (rate < min_rate)
		rate = min_rate;
	div = rate * 2 / parent_rate;

	return parent_rate * div / 2;
}

static int clk_pllv3_sys_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_pllv3 *pll = to_clk_pllv3(hw);
	unsigned long min_rate = parent_rate * 54 / 2;
	unsigned long max_rate = parent_rate * 108 / 2;
	u32 val, div;

	if (rate != BYPASS_RATE && (rate < min_rate || rate > max_rate))
		return -EINVAL;

	pll->rate_req = rate;
	val = readl_relaxed(pll->base);

	if (rate == BYPASS_RATE) {
		/*
		 * Set the PLL in bypass mode if rate requested is
		 * BYPASS_RATE.
		 */
		val |= BM_PLL_BYPASS;
		/* Power down the PLL. */
		if (pll->powerup_set)
			val &= ~BM_PLL_POWER;
		else
			val |= BM_PLL_POWER;
		writel_relaxed(val, pll->base);
		return 0;
	}
	div = rate * 2 / parent_rate;
	val = readl_relaxed(pll->base);
	val &= ~pll->div_mask;
	val |= div;
	writel_relaxed(val, pll->base);

	return clk_pllv3_wait_for_lock(pll, 10);
}

static const struct clk_ops clk_pllv3_sys_ops = {
	.enable		= clk_pllv3_enable,
	.disable	= clk_pllv3_disable,
	.recalc_rate	= clk_pllv3_sys_recalc_rate,
	.round_rate	= clk_pllv3_sys_round_rate,
	.set_rate	= clk_pllv3_sys_set_rate,
};

static unsigned long clk_pllv3_av_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct clk_pllv3 *pll = to_clk_pllv3(hw);
	u32 mfn = readl_relaxed(pll->base + PLL_NUM_OFFSET);
	u32 mfd = readl_relaxed(pll->base + PLL_DENOM_OFFSET);
	u32 div = readl_relaxed(pll->base) & pll->div_mask;
	u32 bypass = readl_relaxed(pll->base) & BYPASS_MASK;

	if (pll->rate_req == BYPASS_RATE && bypass)
		return BYPASS_RATE;

	return (parent_rate * div) + ((parent_rate / mfd) * mfn);
}

static long clk_pllv3_av_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *prate)
{
	unsigned long parent_rate = *prate;
	unsigned long min_rate = parent_rate * 27;
	unsigned long max_rate = parent_rate * 54;
	u32 div;
	u32 mfn, mfd = 1000000;
	s64 temp64;

	if (rate == BYPASS_RATE)
		return BYPASS_RATE;

	if (rate > max_rate)
		rate = max_rate;
	else if (rate < min_rate)
		rate = min_rate;

	div = rate / parent_rate;
	temp64 = (u64) (rate - div * parent_rate);
	temp64 *= mfd;
	do_div(temp64, parent_rate);
	mfn = temp64;

	return parent_rate * div + parent_rate / mfd * mfn;
}

static int clk_pllv3_av_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_pllv3 *pll = to_clk_pllv3(hw);
	unsigned long min_rate = parent_rate * 27;
	unsigned long max_rate = parent_rate * 54;
	u32 val, div;
	u32 mfn, mfd = 1000000;
	s64 temp64;

	if (rate != BYPASS_RATE && (rate < min_rate || rate > max_rate))
		return -EINVAL;

	pll->rate_req = rate;
	val = readl_relaxed(pll->base);

	if (rate == BYPASS_RATE) {
		/*
		 * Set the PLL in bypass mode if rate requested is
		 * BYPASS_RATE.
		 */
		/* Bypass the PLL */
		val |= BM_PLL_BYPASS;
		/* Power down the PLL. */
		if (pll->powerup_set)
			val &= ~BM_PLL_POWER;
		else
			val |= BM_PLL_POWER;
		writel_relaxed(val, pll->base);
		return 0;
	}
	/* Else clear the bypass bit. */
	val &= ~BM_PLL_BYPASS;
	writel_relaxed(val, pll->base);

	div = rate / parent_rate;
	temp64 = (u64) (rate - div * parent_rate);
	temp64 *= mfd;
	do_div(temp64, parent_rate);
	mfn = temp64;

	val = readl_relaxed(pll->base);
	val &= ~pll->div_mask;
	val |= div;
	writel_relaxed(val, pll->base);
	writel_relaxed(mfn, pll->base + PLL_NUM_OFFSET);
	writel_relaxed(mfd, pll->base + PLL_DENOM_OFFSET);

	return clk_pllv3_wait_for_lock(pll, 10);
}

static const struct clk_ops clk_pllv3_av_ops = {
	.enable		= clk_pllv3_enable,
	.disable	= clk_pllv3_disable,
	.recalc_rate	= clk_pllv3_av_recalc_rate,
	.round_rate	= clk_pllv3_av_round_rate,
	.set_rate	= clk_pllv3_av_set_rate,
};

static unsigned long clk_pllv3_enet_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	return 500000000;
}

static const struct clk_ops clk_pllv3_enet_ops = {
	.enable		= clk_pllv3_enable,
	.disable	= clk_pllv3_disable,
	.recalc_rate	= clk_pllv3_enet_recalc_rate,
};

static const struct clk_ops clk_pllv3_mlb_ops = {
	.enable		= clk_pllv3_enable,
	.disable	= clk_pllv3_disable,
};

struct clk *imx_clk_pllv3(enum imx_pllv3_type type, const char *name,
			  const char *parent_name, void __iomem *base,
			  u32 div_mask, bool always_on)
{
	struct clk_pllv3 *pll;
	const struct clk_ops *ops;
	struct clk *clk;
	struct clk_init_data init;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	switch (type) {
	case IMX_PLLV3_SYS:
		ops = &clk_pllv3_sys_ops;
		break;
	case IMX_PLLV3_USB:
		ops = &clk_pllv3_ops;
		pll->powerup_set = true;
		break;
	case IMX_PLLV3_AV:
		ops = &clk_pllv3_av_ops;
		break;
	case IMX_PLLV3_ENET:
		ops = &clk_pllv3_enet_ops;
		break;
	case IMX_PLLV3_MLB:
		ops = &clk_pllv3_mlb_ops;
		break;
	default:
		ops = &clk_pllv3_ops;
	}
	pll->base = base;
	pll->div_mask = div_mask;
	pll->always_on = always_on;

	init.name = name;
	init.ops = ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
