// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(C) 2023, Emcraft Systems
 * Author(s): Vladimir Skvortsov <vskvortsov@emcraft.com>
 */

#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/slab.h>

#include "clk.h"

#define DIV_SHIFT	0x0
#define DIV_MASK	0xff
#define PWRUP_MASK	BIT(13)
#define CLKE_MASK	BIT(14)
#define PDIV_SHIFT	15
#define PDIV_MASK	GENMASK(16, 15)
#define STABLE_MASK	BIT(29)

#define LOCK_TIMEOUT_US		50

static const unsigned int pdiv_table[] = { 2, 4, 8, 1 };

struct clk_pllarm {
	struct clk_hw	hw;
	void __iomem	*base;
	u32		powerup_mask;
	u32		enable_mask;
	u32		stable_mask;
	u32		div_mask;
	u32		div_shift;
	u32		pdiv_mask;
	u32		pdiv_shift;
};

#define to_clk_pllarm(_hw) container_of(_hw, struct clk_pllarm, hw)

static ulong clk_pllarm_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct clk_pllarm *pll = to_clk_pllarm(hw);
	u32 pll_ctrl = readl_relaxed(pll->base);

	u32 div = (pll_ctrl & pll->div_mask) >> pll->div_shift;
	u32 pdiv_idx = (pll_ctrl & pll->pdiv_mask) >> pll->pdiv_shift;

	if (pdiv_idx >= ARRAY_SIZE(pdiv_table)) {
		return 0;
	}

	return (parent_rate * (div / 2)) / pdiv_table[pdiv_idx];
}

static int clk_pllarm_prepare(struct clk_hw *hw)
{
	struct clk_pllarm *pll = to_clk_pllarm(hw);
	u32 pll_ctrl = readl_relaxed(pll->base) & ~(STABLE_MASK);

	if (pll_ctrl & pll->enable_mask) {
		return 0;
	}

	pll_ctrl |= pll->powerup_mask;
	writel_relaxed(pll_ctrl, pll->base);

	readl_relaxed_poll_timeout(pll->base, pll_ctrl, pll_ctrl & STABLE_MASK, 10, 0);

	pll_ctrl |= pll->enable_mask;
	writel_relaxed(pll_ctrl, pll->base);

	return 0;
}

static void clk_pllarm_unprepare(struct clk_hw *hw)
{
	struct clk_pllarm *pll = to_clk_pllarm(hw);
	u32 pll_ctrl = readl_relaxed(pll->base) & ~(STABLE_MASK);

	pll_ctrl &= ~(pll->enable_mask | pll->powerup_mask);
	writel_relaxed(pll_ctrl, pll->base);
}

static const struct clk_ops clk_pllarm_ops = {
	.prepare	= clk_pllarm_prepare,
	.unprepare	= clk_pllarm_unprepare,
	.recalc_rate	= clk_pllarm_recalc_rate,
};

struct clk_hw *imx_clk_hw_pll_arm_rt1170(const char *name,
					 const char *parent_name, void __iomem *base)
{
	struct clk_pllarm *pll;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->powerup_mask = PWRUP_MASK;
	pll->enable_mask = CLKE_MASK;
	pll->stable_mask = STABLE_MASK;
	pll->div_mask = DIV_MASK;
	pll->div_shift = DIV_SHIFT;
	pll->pdiv_mask = PDIV_MASK;
	pll->pdiv_shift = PDIV_SHIFT;

	pll->base = base;

	init.name = name;
	init.ops = &clk_pllarm_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->hw.init = &init;

	hw = &pll->hw;
	ret = clk_hw_register(NULL, hw);
	if (ret) {
		kfree(pll);
		return ERR_PTR(ret);
	}

	return hw;
}
