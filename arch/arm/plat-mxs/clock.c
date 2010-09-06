/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/cpufreq.h>

#include <mach/clock.h>

extern int cpufreq_trig_needed;
static bool (*mxs_enable_h_autoslow)(bool enable);
static void (*mxs_set_h_autoslow_flags)(u16 flags);

static DEFINE_SPINLOCK(clockfw_lock);

/*
 *-------------------------------------------------------------------------
 * Standard clock functions defined in include/linux/clk.h
 *-------------------------------------------------------------------------
 */
int __clk_get(struct clk *clk)
{
	if (clk->ref < CLK_REF_LIMIT)
		clk->ref += CLK_REF_UNIT;
	return clk->ref < CLK_REF_LIMIT;
}

void __clk_put(struct clk *clk)
{
	if (clk->ref & CLK_REF_LIMIT)
		clk->ref -= CLK_REF_UNIT;
}

static void default_clk_disable(struct clk *clk)
{
	if (clk->enable_reg)
		__raw_writel(clk->enable_bits, clk->enable_reg + SET_REGISTER);
}

static int default_clk_enable(struct clk *clk)
{
	if (clk->enable_reg)
		__raw_writel(clk->enable_bits, clk->enable_reg + CLR_REGISTER);
	return 0;
}

static unsigned long default_get_rate(struct clk *clk)
{
	if (clk->parent && clk->parent->get_rate)
		return clk->parent->get_rate(clk->parent);
	return 0L;
}

static void __clk_disable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk) || !clk->ref)
		return;

	if ((--clk->ref) & CLK_EN_MASK)
		return;

	if (clk->disable)
		clk->disable(clk);
	__clk_disable(clk->secondary);
	__clk_disable(clk->parent);
}

static int __clk_enable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	if ((clk->ref++) & CLK_EN_MASK)
		return 0;
	if (clk->parent)
		__clk_enable(clk->parent);
	if (clk->secondary)
		__clk_enable(clk->secondary);
	if (clk->enable)
		clk->enable(clk);
	return 0;
}

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	int ret = 0;
	int pre_usage;

	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	spin_lock_irqsave(&clockfw_lock, flags);
	pre_usage = (clk->ref & CLK_EN_MASK);

	if (clk->set_sys_dependent_parent)
		clk->set_sys_dependent_parent(clk);

	ret = __clk_enable(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
	if ((clk->flags & CPU_FREQ_TRIG_UPDATE)
	    && (pre_usage == 0)) {
		cpufreq_trig_needed = 1;
		cpufreq_update_policy(0);
	}
	return ret;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	if (clk == NULL || IS_ERR(clk))
		return;
	if (clk->flags & ALWAYS_ENABLED)
		return;
	spin_lock_irqsave(&clockfw_lock, flags);
	__clk_disable(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
	if ((clk->flags & CPU_FREQ_TRIG_UPDATE)
			&& ((clk->ref & CLK_EN_MASK) == 0)) {
		cpufreq_trig_needed = 1;
		cpufreq_update_policy(0);
	}
}
EXPORT_SYMBOL(clk_disable);

int clk_get_usecount(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return 0;

	return clk->ref & CLK_EN_MASK;
}
EXPORT_SYMBOL(clk_get_usecount);

unsigned long clk_get_rate(struct clk *clk)
{
	unsigned long flags, rate;
	if (clk == NULL || IS_ERR(clk) || clk->get_rate == NULL)
		return 0UL;

	spin_lock_irqsave(&clockfw_lock, flags);
	rate = clk->get_rate(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
	return rate;
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (clk == NULL || IS_ERR(clk) || !clk->round_rate)
		return 0;

	if (clk->flags & RATE_FIXED)
		return 0;

	if (clk->round_rate)
		return clk->round_rate(clk, rate);
	return 0;
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long flags;
	int ret = -EINVAL;

	if (clk == NULL || IS_ERR(clk) || clk->set_rate == NULL || rate == 0)
		return ret;

	if (clk->flags & RATE_FIXED)
		return ret;

	spin_lock_irqsave(&clockfw_lock, flags);
	ret = clk->set_rate(clk, rate);
	spin_unlock_irqrestore(&clockfw_lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	unsigned long flags;
	int ret = -EINVAL;
	struct clk *prev_parent;

	if (clk == NULL || IS_ERR(clk) || parent == NULL ||
	    IS_ERR(parent) || clk->set_parent == NULL ||
	    parent->get_rate == NULL)
		return ret;

	if (clk->ref & CLK_EN_MASK)
		clk_enable(parent);

	spin_lock_irqsave(&clockfw_lock, flags);
	prev_parent = clk->parent;
	ret = clk->set_parent(clk, parent);
	if (ret) {
		spin_unlock_irqrestore(&clockfw_lock, flags);
		if (clk->ref & CLK_EN_MASK)
			clk_disable(parent);
		return ret;
	}

	clk->parent = parent;

	spin_unlock_irqrestore(&clockfw_lock, flags);

	if (clk->ref & CLK_EN_MASK)
		clk_disable(prev_parent);
	return 0;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	struct clk *ret = NULL;

	if (clk == NULL || IS_ERR(clk))
		return ret;

	return clk->parent;
}
EXPORT_SYMBOL(clk_get_parent);

int clk_register(struct clk_lookup *lookup)
{
	if (lookup == NULL || IS_ERR(lookup) ||
	    lookup->clk == NULL || IS_ERR(lookup->clk))
		return -EINVAL;

	if (lookup->clk->ref & CLK_REF_LIMIT)
		return -EEXIST;

	if (!(lookup->clk->enable))
		lookup->clk->enable = default_clk_enable;
	if (!(lookup->clk->disable))
		lookup->clk->disable = default_clk_disable;
	if (!(lookup->clk->get_rate))
		lookup->clk->get_rate = default_get_rate;

	clkdev_add(lookup);

	return 0;
}
EXPORT_SYMBOL(clk_register);

void clk_unregister(struct clk_lookup *lookup)
{
	if (lookup == NULL || IS_ERR(lookup) ||
	    lookup->clk == NULL || IS_ERR(lookup->clk))
		return;

	if (lookup->clk->ref & CLK_REF_LIMIT)
		return;

	clkdev_drop(lookup);
	if (lookup->clk->enable == default_clk_enable)
		lookup->clk->enable = NULL;
	if (lookup->clk->disable == default_clk_disable)
		lookup->clk->disable = NULL;
	if (lookup->clk->get_rate == default_get_rate)
		lookup->clk->get_rate = NULL;
}
EXPORT_SYMBOL(clk_unregister);

bool clk_enable_h_autoslow(bool enable)
{
	unsigned long flags;
	bool ret = false;

	if (mxs_enable_h_autoslow == NULL)
		return ret;

	spin_lock_irqsave(&clockfw_lock, flags);
	ret = mxs_enable_h_autoslow(enable);
	spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_enable_h_autoslow);

void clk_set_h_autoslow_flags(u16 mask)
{
	unsigned long flags;

	if (mxs_set_h_autoslow_flags == NULL)
		return;

	spin_lock_irqsave(&clockfw_lock, flags);
	mxs_set_h_autoslow_flags(mask);
	spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_set_h_autoslow_flags);

void clk_en_public_h_asm_ctrl(bool (*enable_func)(bool),
	void (*set_func)(u16))
{
	mxs_enable_h_autoslow = enable_func;
	mxs_set_h_autoslow_flags = set_func;
}
EXPORT_SYMBOL(clk_en_public_h_asm_ctrl);
