// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 * Copyright 2012 Linaro Ltd.
 */

#include <linux/clk-provider.h>
#include <linux/export.h>
#include <linux/imx_sema4.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <soc/imx/src.h>
#include "clk.h"

/**
 * struct clk_pfd - IMX PFD clock
 * @hw:		clock source
 * @reg:	PFD register address
 * @idx:	the index of PFD encoded in the register
 * @rmw_lock:	spinlock for read-modify-write mode, or NULL for SET/CLR mode
 *
 * PFD clock found on i.MX6 series and i.MX RT series.  Each register for
 * PFD has 4 clk_pfd data encoded, and member idx is used to specify the one.
 * On i.MX6, each register has SET, CLR and TOG registers at offset 0x4 0x8
 * and 0xc.  On i.MX RT1170, the ANATOP module does not have SET/CLR/TOG
 * registers, so direct read-modify-write is used instead.  In RMW mode,
 * a shared spinlock protects against concurrent access from multiple PFDs
 * within the same register.
 */
struct clk_pfd {
	struct clk_hw	hw;
	void __iomem	*reg;
	u8		idx;
	spinlock_t	*rmw_lock;
};

#define to_clk_pfd(_hw) container_of(_hw, struct clk_pfd, hw)

#define SET	0x4
#define CLR	0x8
#define OTG	0xc

static void clk_pfd_do_hardware(struct clk_pfd *pfd, bool enable)
{
	u32 gate_bit = 1 << ((pfd->idx + 1) * 8 - 1);

	if (pfd->rmw_lock) {
		unsigned long flags;
		u32 val;

		spin_lock_irqsave(pfd->rmw_lock, flags);
		val = readl_relaxed(pfd->reg);
		if (enable)
			val &= ~gate_bit;
		else
			val |= gate_bit;
		writel_relaxed(val, pfd->reg);
		spin_unlock_irqrestore(pfd->rmw_lock, flags);
	} else {
		if (enable)
			writel_relaxed(gate_bit, pfd->reg + CLR);
		else
			writel_relaxed(gate_bit, pfd->reg + SET);
	}
}

static void clk_pfd_do_shared_clks(struct clk_hw *hw, bool enable)
{
	struct clk_pfd *pfd = to_clk_pfd(hw);

	if (imx_src_is_m4_enabled() && clk_on_imx6sx()) {
#ifdef CONFIG_SOC_IMX6SX
		if (!amp_power_mutex || !shared_mem) {
			if (enable)
				clk_pfd_do_hardware(pfd, enable);
			return;
		}

		imx_sema4_mutex_lock(amp_power_mutex);
		if (shared_mem->ca9_valid != SHARED_MEM_MAGIC_NUMBER ||
			shared_mem->cm4_valid != SHARED_MEM_MAGIC_NUMBER) {
			imx_sema4_mutex_unlock(amp_power_mutex);
			return;
		}

		if (!imx_update_shared_mem(hw, enable)) {
			imx_sema4_mutex_unlock(amp_power_mutex);
			return;
		}

		clk_pfd_do_hardware(pfd, enable);

		imx_sema4_mutex_unlock(amp_power_mutex);
#endif
	} else {
		clk_pfd_do_hardware(pfd, enable);
	}
}

static int clk_pfd_enable(struct clk_hw *hw)
{
	clk_pfd_do_shared_clks(hw, true);

	return 0;
}

static void clk_pfd_disable(struct clk_hw *hw)
{
	clk_pfd_do_shared_clks(hw, false);
}

static unsigned long clk_pfd_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_pfd *pfd = to_clk_pfd(hw);
	u64 tmp = parent_rate;
	u8 frac = (readl_relaxed(pfd->reg) >> (pfd->idx * 8)) & 0x3f;

	tmp *= 18;
	do_div(tmp, frac);

	return tmp;
}

static long clk_pfd_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	u64 tmp = *prate;
	u8 frac;

	tmp = tmp * 18 + rate / 2;
	do_div(tmp, rate);
	frac = tmp;
	if (frac < 12)
		frac = 12;
	else if (frac > 35)
		frac = 35;
	tmp = *prate;
	tmp *= 18;
	do_div(tmp, frac);

	return tmp;
}

static int clk_pfd_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_pfd *pfd = to_clk_pfd(hw);
	u64 tmp = parent_rate;
	u8 frac;

	tmp = tmp * 18 + rate / 2;
	do_div(tmp, rate);
	frac = tmp;
	if (frac < 12)
		frac = 12;
	else if (frac > 35)
		frac = 35;

	if (pfd->rmw_lock) {
		unsigned long flags;
		u32 val;

		spin_lock_irqsave(pfd->rmw_lock, flags);
		val = readl_relaxed(pfd->reg);
		val &= ~(0x3f << (pfd->idx * 8));
		val |= frac << (pfd->idx * 8);
		writel_relaxed(val, pfd->reg);
		spin_unlock_irqrestore(pfd->rmw_lock, flags);
	} else {
		writel_relaxed(0x3f << (pfd->idx * 8), pfd->reg + CLR);
		writel_relaxed(frac << (pfd->idx * 8), pfd->reg + SET);
	}

	return 0;
}

static int clk_pfd_is_enabled(struct clk_hw *hw)
{
	struct clk_pfd *pfd = to_clk_pfd(hw);

	if (readl_relaxed(pfd->reg) & (1 << ((pfd->idx + 1) * 8 - 1)))
		return 0;

	return 1;
}

static const struct clk_ops clk_pfd_ops = {
	.enable		= clk_pfd_enable,
	.disable	= clk_pfd_disable,
	.recalc_rate	= clk_pfd_recalc_rate,
	.round_rate	= clk_pfd_round_rate,
	.set_rate	= clk_pfd_set_rate,
	.is_enabled     = clk_pfd_is_enabled,
};

struct clk_hw *imx_clk_hw_pfd(const char *name, const char *parent_name,
			void __iomem *reg, u8 idx)
{
	return imx_clk_hw_pfd_rmw(name, parent_name, reg, idx, NULL);
}
EXPORT_SYMBOL_GPL(imx_clk_hw_pfd);

/**
 * imx_clk_hw_pfd_rmw - register a PFD clock using read-modify-write
 * @name:	clock name
 * @parent_name: parent clock name
 * @reg:	PFD register address (shared by 4 PFDs)
 * @idx:	PFD index within the register (0-3)
 * @lock:	shared spinlock protecting RMW on this register
 *
 * For SoCs (like i.MX RT1170) where the ANATOP PFD register does not
 * have SET/CLR/TOG companion registers.  All PFDs sharing the same
 * register must use the same @lock to prevent concurrent RMW races.
 */
struct clk_hw *imx_clk_hw_pfd_rmw(const char *name, const char *parent_name,
			void __iomem *reg, u8 idx, spinlock_t *lock)
{
	struct clk_pfd *pfd;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	pfd = kzalloc(sizeof(*pfd), GFP_KERNEL);
	if (!pfd)
		return ERR_PTR(-ENOMEM);

	pfd->reg = reg;
	pfd->idx = idx;
	pfd->rmw_lock = lock;

	init.name = name;
	init.ops = &clk_pfd_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pfd->hw.init = &init;
	hw = &pfd->hw;

	ret = clk_hw_register(NULL, hw);
	if (ret) {
		kfree(pfd);
		return ERR_PTR(ret);
	}

	return hw;
}
EXPORT_SYMBOL_GPL(imx_clk_hw_pfd_rmw);
