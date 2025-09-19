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

/* ARMPLL specific bits */
#define ARMPLL_DIV_SHIFT	0x0
#define ARMPLL_DIV_MASK		0xff
#define ARMPLL_PDIV_SHIFT	15
#define ARMPLL_PDIV_MASK	GENMASK(16, 15)

/* mixed ARM and SYSn PLLs bits */
#define ARMPLL_HOLD_RING_OFF_MASK BIT(12)
#define SYSPLL_HOLD_RING_OFF_MASK BIT(11)

#define ARMPLL_PWRUP_MASK	BIT(13)
#define SYSPLL2_PWRUP_MASK	BIT(23)
#define SYSPLL3_PWRUP_MASK	BIT(21)

#define ARMPLL_CLKE_MASK	BIT(14)
#define SYSPLL_CLKE_MASK	BIT(13)

#define PLL1_GATE_MASK	BIT(14)

/* common bits for all ARM, SYS2 and SYS3 PLLs*/
#define STABLE_MASK	BIT(29)
#define GATE_MASK	BIT(30)

/* PLLs lock timeouts after powerup bit is set */
#define ARMPLL_LOCK_TIME	60
#define SYSPLL2_LOCK_TIME	500
#define SYSPLL3_LOCK_TIME	60

static const unsigned int pdiv_table[] = { 2, 4, 8, 1 };

struct clk_pll_imxrt1170 {
	struct clk_hw	hw;
	void __iomem	*base;
	u32		powerup_mask;
	u32		enable_mask;
	u32		stable_mask;
	u32		gate_mask;
	u32		hold_ring_off_mask;
	u32		div_mask;
	u32		div_shift;
	u32		pdiv_mask;
	u32		pdiv_shift;
	enum imxrt1170_pll_type type;
};

#define to_clk_pll_imxrt1170(_hw) container_of(_hw, struct clk_pll_imxrt1170, hw)

static int clk_pllimxrt1170_wait_stable(struct clk_pll_imxrt1170 *pll)
{
	u32 pll_ctrl = readl_relaxed(pll->base);
	int tmout;
	if (pll->type == IMXRT1170_PLL2)
		tmout = SYSPLL2_LOCK_TIME;
	else if (pll->type == IMXRT1170_PLL3)
		tmout = SYSPLL3_LOCK_TIME;
	else if (pll->type == IMXRT1170_PLLARM)
		tmout = ARMPLL_LOCK_TIME;
	return readl_relaxed_poll_timeout(pll->base, pll_ctrl, pll_ctrl & pll->stable_mask, 10, tmout);
}

static int clk_pllimxrt1170_is_prepared(struct clk_hw *hw)
{
	struct clk_pll_imxrt1170 *pll = to_clk_pll_imxrt1170(hw);
	u32 val;

	val = readl_relaxed(pll->base);

	if (!(val & pll->stable_mask) ||
	    !(val & pll->powerup_mask)) {
		return 0;
	}

	return 1;
}

static int clk_pllimxrt1170_prepare(struct clk_hw *hw)
{
	struct clk_pll_imxrt1170 *pll = to_clk_pll_imxrt1170(hw);
	u32 val;
	int tmout;

	val = readl_relaxed(pll->base);
	if (val & pll->powerup_mask) {
		return 0;
	}

	val &= ~pll->stable_mask;
	val |= pll->gate_mask;
	val &= ~pll->enable_mask;
	writel_relaxed(val, pll->base);

	udelay(30);

	val |= (pll->powerup_mask | pll->hold_ring_off_mask);
	writel_relaxed(val, pll->base);

	if (pll->type == IMXRT1170_PLL2)
		tmout = SYSPLL2_LOCK_TIME;
	else if (pll->type == IMXRT1170_PLL3)
		tmout = SYSPLL3_LOCK_TIME;
	else if (pll->type == IMXRT1170_PLLARM)
		tmout = ARMPLL_LOCK_TIME;

	udelay(tmout/2);

	val &= ~(pll->hold_ring_off_mask);
	writel_relaxed(val, pll->base);

	return clk_pllimxrt1170_wait_stable(pll);
}

static void clk_pllimxrt1170_unprepare(struct clk_hw *hw)
{
	struct clk_pll_imxrt1170 *pll = to_clk_pll_imxrt1170(hw);
	u32 pll_ctrl = readl_relaxed(pll->base) & ~(pll->stable_mask);

	pll_ctrl |= pll->gate_mask;
	pll_ctrl &= ~(pll->enable_mask | pll->powerup_mask);
	writel_relaxed(pll_ctrl, pll->base);
}

static int clk_pllimxrt1170_is_enabled(struct clk_hw *hw)
{
	struct clk_pll_imxrt1170 *pll = to_clk_pll_imxrt1170(hw);
	u32 val;

	val = readl_relaxed(pll->base);

	if ((val & pll->gate_mask) ||
	    !(val & pll->stable_mask) ||
	    !(val & pll->enable_mask) ||
	    !(val & pll->powerup_mask)) {
		return 0;
	}

	return 1;
}

static int clk_pllimxrt1170_enable(struct clk_hw *hw)
{
	struct clk_pll_imxrt1170 *pll = to_clk_pll_imxrt1170(hw);
	u32 val;

	val = readl_relaxed(pll->base);

	if ((pll->type != IMXRT1170_PLL1) && !(val & pll->powerup_mask)) {
		return -EINVAL;
	}

	if (!(val & pll->enable_mask)) {
		val |= pll->enable_mask;
		writel_relaxed(val, pll->base);
	}
	if (val & pll->gate_mask) {
		val &= ~(pll->gate_mask);
		writel_relaxed(val, pll->base);
	}

	return 0;
}

static void clk_pllimxrt1170_disable(struct clk_hw *hw)
{
	struct clk_pll_imxrt1170 *pll = to_clk_pll_imxrt1170(hw);
	u32 val;
	val = readl_relaxed(pll->base);

	val &= ~(pll->enable_mask);
	val |= pll->gate_mask;

	writel_relaxed(val, pll->base);
}

static ulong clk_pllarm_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct clk_pll_imxrt1170 *pll = to_clk_pll_imxrt1170(hw);
	u32 pll_ctrl = readl_relaxed(pll->base);

	u32 div = (pll_ctrl & pll->div_mask) >> pll->div_shift;
	u32 pdiv_idx = (pll_ctrl & pll->pdiv_mask) >> pll->pdiv_shift;

	if (pdiv_idx >= ARRAY_SIZE(pdiv_table)) {
		return 0;
	}

	return (parent_rate * (div / 2)) / pdiv_table[pdiv_idx];
}

static const struct clk_ops clk_pllarm_ops = {
	.prepare	= clk_pllimxrt1170_prepare,
	.unprepare	= clk_pllimxrt1170_unprepare,
	.disable	= clk_pllimxrt1170_disable,
	.enable		= clk_pllimxrt1170_enable,
	.is_enabled     = clk_pllimxrt1170_is_enabled,
	.is_prepared	= clk_pllimxrt1170_is_prepared,
	.recalc_rate	= clk_pllarm_recalc_rate,
};

static unsigned long clk_pllsys_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct clk_pll_imxrt1170 *pll = to_clk_pll_imxrt1170(hw);
	u32 factor = (pll->type == IMXRT1170_PLL2) ? 22 : 20;

	return parent_rate * factor;
}

static int clk_pllsys_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_pll_imxrt1170 *pll = to_clk_pll_imxrt1170(hw);
	u32 factor = (pll->type == IMXRT1170_PLL2) ? 22 : 20;
	u32 val =  parent_rate * factor;

	if (rate == val)
		return 0;

	return -EINVAL;
}

static long clk_pllsys_round_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long *prate)
{
	unsigned long parent_rate = *prate;

	return clk_pllsys_recalc_rate(hw, parent_rate);
}

static const struct clk_ops clk_pllsys_ops = {
	.prepare	= clk_pllimxrt1170_prepare,
	.unprepare	= clk_pllimxrt1170_unprepare,
	.disable	= clk_pllimxrt1170_disable,
	.enable		= clk_pllimxrt1170_enable,
	.is_enabled     = clk_pllimxrt1170_is_enabled,
	.is_prepared	= clk_pllimxrt1170_is_prepared,
	.round_rate	= clk_pllsys_round_rate,
	.recalc_rate	= clk_pllsys_recalc_rate,
	.set_rate	= clk_pllsys_set_rate,
};

static ulong clk_pll1_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	return 1000000000;
}

static const struct clk_ops clk_pll1_ops = {
	.disable	= clk_pllimxrt1170_disable,
	.enable		= clk_pllimxrt1170_enable,
	.is_enabled     = clk_pllimxrt1170_is_enabled,
	.recalc_rate	= clk_pll1_recalc_rate,
};

struct clk_hw *imx_clk_hw_pll_rt1170(enum imxrt1170_pll_type type, const char *name,
					 const char *parent_name, void __iomem *base)
{
	struct clk_pll_imxrt1170 *pll;
	struct clk_hw *hw;
	struct clk_init_data init;
	const struct clk_ops *ops;
	int ret;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->type = type;
	pll->stable_mask = STABLE_MASK;
	pll->gate_mask = GATE_MASK;
	pll->base = base;

	switch (type) {
	case IMXRT1170_PLLARM:
		pll->powerup_mask = ARMPLL_PWRUP_MASK;
		pll->enable_mask = ARMPLL_CLKE_MASK;
		pll->div_mask = ARMPLL_DIV_MASK;
		pll->div_shift = ARMPLL_DIV_SHIFT;
		pll->pdiv_mask = ARMPLL_PDIV_MASK;
		pll->pdiv_shift = ARMPLL_PDIV_SHIFT;
		ops = &clk_pllarm_ops;
		break;
	case IMXRT1170_PLL3:
		pll->powerup_mask = SYSPLL3_PWRUP_MASK;
		pll->enable_mask = SYSPLL_CLKE_MASK;
		ops = &clk_pllsys_ops;
		break;
	case IMXRT1170_PLL2:
		pll->powerup_mask = SYSPLL2_PWRUP_MASK;
		pll->enable_mask = SYSPLL_CLKE_MASK;
		ops = &clk_pllsys_ops;
		break;
	case IMXRT1170_PLL1:
		pll->gate_mask = PLL1_GATE_MASK;
		pll->enable_mask = SYSPLL_CLKE_MASK;
		ops = &clk_pll1_ops;
		break;
	default:
		ops = NULL;
		break;
	}

	init.name = name;
	init.ops = ops;
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
