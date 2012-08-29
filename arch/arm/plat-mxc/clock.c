/*
 * Based on arch/arm/plat-omap/clock.c
 *
 * Copyright (C) 2004 - 2005 Nokia corporation
 * Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 * Modified for omap shared clock framework by Tony Lindgren <tony@atomide.com>
 * Copyright 2007-2012 Freescale Semiconductor, Inc.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

/* #define DEBUG */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/semaphore.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/hardirq.h>

#include <mach/clock.h>
#include <mach/hardware.h>

extern int dvfs_core_is_active;
extern void bus_freq_update(struct clk *clk, bool flag);

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);

/*-------------------------------------------------------------------------
 * Standard clock functions defined in include/linux/clk.h
 *-------------------------------------------------------------------------*/

static void __clk_disable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	if (!clk->usecount) {
		WARN(1, "clock enable/disable mismatch! clk  %s\n", clk->name);
		return;
	}

	if (!(--clk->usecount)) {
		if (clk->disable)
			clk->disable(clk);
		__clk_disable(clk->parent);
		__clk_disable(clk->secondary);
	}
}

static int __clk_enable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	if (clk->usecount++ == 0) {
		__clk_enable(clk->parent);
		__clk_enable(clk->secondary);

		if (clk->enable)
			clk->enable(clk);
	}
	return 0;
}

/* This function increments the reference count on the clock and enables the
 * clock if not already enabled. The parent clock tree is recursively enabled
 */
int clk_enable(struct clk *clk)
{
	/* unsigned long flags; */
	int ret = 0;

	if (in_interrupt()) {
		printk(KERN_ERR " clk_enable cannot be called in an interrupt context\n");
		dump_stack();
		BUG();
	}

	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	if ((clk->flags & AHB_HIGH_SET_POINT) ||
		(clk->flags & AHB_MED_SET_POINT) ||
		(clk->flags & AHB_AUDIO_SET_POINT) ||
		(clk->flags & CPU_FREQ_TRIG_UPDATE))
		bus_freq_update(clk, true);

	mutex_lock(&clocks_mutex);
	ret = __clk_enable(clk);
	mutex_unlock(&clocks_mutex);

	return ret;
}
EXPORT_SYMBOL(clk_enable);

/* This function decrements the reference count on the clock and disables
 * the clock when reference count is 0. The parent clock tree is
 * recursively disabled
 */
void clk_disable(struct clk *clk)
{
	/* unsigned long flags; */

	if (in_interrupt()) {
		printk(KERN_ERR " clk_disable cannot be called in an interrupt context\n");
		dump_stack();
		BUG();
	}

	if (clk == NULL || IS_ERR(clk))
		return;

	mutex_lock(&clocks_mutex);
	__clk_disable(clk);
	mutex_unlock(&clocks_mutex);

	if ((clk->flags & AHB_HIGH_SET_POINT) ||
		(clk->flags & AHB_MED_SET_POINT) ||
		(clk->flags & AHB_AUDIO_SET_POINT) ||
		(clk->flags & CPU_FREQ_TRIG_UPDATE))
		bus_freq_update(clk, false);
}

EXPORT_SYMBOL(clk_disable);

/*!
 * @brief Function to get the usage count for the requested clock.
 *
 * This function returns the reference count for the clock.
 *
 * @param clk 	Handle to clock to disable.
 *
 * @return Returns the usage count for the requested clock.
 */
int clk_get_usecount(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return 0;

	return clk->usecount;
}

EXPORT_SYMBOL(clk_get_usecount);

/* Retrieve the *current* clock rate. If the clock itself
 * does not provide a special calculation routine, ask
 * its parent and so on, until one is able to return
 * a valid clock rate
 */
unsigned long clk_get_rate(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return 0UL;

	if (clk->get_rate)
		return clk->get_rate(clk);

	return clk_get_rate(clk->parent);
}
EXPORT_SYMBOL(clk_get_rate);

/* Round the requested clock rate to the nearest supported
 * rate that is less than or equal to the requested rate.
 * This is dependent on the clock's current parent.
 */
long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (clk == NULL || IS_ERR(clk) || !clk->round_rate)
		return 0;

	return clk->round_rate(clk, rate);
}
EXPORT_SYMBOL(clk_round_rate);

/* Set the clock to the requested clock rate. The rate must
 * match a supported rate exactly based on what clk_round_rate returns
 */
int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EINVAL;

	if (clk == NULL || IS_ERR(clk) || clk->set_rate == NULL || rate == 0)
		return ret;

	mutex_lock(&clocks_mutex);
	ret = clk->set_rate(clk, rate);
	mutex_unlock(&clocks_mutex);

	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

/* Set the clock's parent to another clock source */
int clk_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;
	struct clk *old;

	if (clk == NULL || IS_ERR(clk) || parent == NULL ||
	    IS_ERR(parent) || clk->set_parent == NULL)
		return ret;

	mutex_lock(&clocks_mutex);

	if (clk->usecount) {
		if (in_interrupt()) {
			printk(KERN_ERR " clk_enable cannot be called in an interrupt context\n");
			dump_stack();
			mutex_unlock(&clocks_mutex);
			BUG();
		}
		__clk_enable(parent);
	}

	ret = clk->set_parent(clk, parent);
	if (ret == 0) {
		old = clk->parent;
		clk->parent = parent;
	} else {
		old = parent;
	}
	if (clk->usecount)
		__clk_disable(old);

	mutex_unlock(&clocks_mutex);

	return ret;
}
EXPORT_SYMBOL(clk_set_parent);

/* Retrieve the clock's parent clock source */
struct clk *clk_get_parent(struct clk *clk)
{
	struct clk *ret = NULL;

	if (clk == NULL || IS_ERR(clk))
		return ret;

	return clk->parent;
}
EXPORT_SYMBOL(clk_get_parent);

/*
 * Get the resulting clock rate from a PLL register value and the input
 * frequency. PLLs with this register layout can at least be found on
 * MX1, MX21, MX27 and MX31
 *
 *                  mfi + mfn / (mfd + 1)
 *  f = 2 * f_ref * --------------------
 *                        pd + 1
 */
unsigned long mxc_decode_pll(unsigned int reg_val, u32 freq)
{
	long long ll;
	int mfn_abs;
	unsigned int mfi, mfn, mfd, pd;

	mfi = (reg_val >> 10) & 0xf;
	mfn = reg_val & 0x3ff;
	mfd = (reg_val >> 16) & 0x3ff;
	pd =  (reg_val >> 26) & 0xf;

	mfi = mfi <= 5 ? 5 : mfi;

	mfn_abs = mfn;

	/* On all i.MXs except i.MX1 and i.MX21 mfn is a 10bit
	 * 2's complements number
	 */
	if (!cpu_is_mx1() && !cpu_is_mx21() && mfn >= 0x200)
		mfn_abs = 0x400 - mfn;

	freq *= 2;
	freq /= pd + 1;

	ll = (unsigned long long)freq * mfn_abs;

	do_div(ll, mfd + 1);

	if (!cpu_is_mx1() && !cpu_is_mx21() && mfn >= 0x200)
		ll = -ll;

	ll = (freq * mfi) + ll;

	return ll;
}

#ifdef CONFIG_CLK_DEBUG
/*
 *	debugfs support to trace clock tree hierarchy and attributes
 */
static int clk_debug_rate_get(void *data, u64 *val)
{
	struct clk *clk = data;

	*val = (u64)clk_get_rate(clk);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(clk_debug_rate_fops, clk_debug_rate_get, NULL,
		"%llu\n");


static struct dentry *clk_root;
static int clk_debug_register_one(struct clk *clk)
{
	int err;
	struct dentry *d, *child, *child_tmp;
	struct clk *pa = clk_get_parent(clk);

	if (pa && !IS_ERR(pa))
		d = debugfs_create_dir(clk->name, pa->dentry);
	else {
		if (!clk_root)
			clk_root = debugfs_create_dir("clock", NULL);
		if (!clk_root)
			return -ENOMEM;
		d = debugfs_create_dir(clk->name, clk_root);
	}

	if (!d)
		return -ENOMEM;

	clk->dentry = d;

	d = debugfs_create_u8("usecount", S_IRUGO, clk->dentry,
			(u8 *)&clk->usecount);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}

	d = debugfs_create_file("rate", S_IRUGO, clk->dentry, (void *)clk,
			&clk_debug_rate_fops);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}

	d = debugfs_create_u32("flags", S_IRUGO, clk->dentry,
			(u32 *)&clk->flags);
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}

	return 0;

err_out:
	d = clk->dentry;
	list_for_each_entry_safe(child, child_tmp, &d->d_subdirs, d_u.d_child)
		debugfs_remove(child);
	debugfs_remove(clk->dentry);
	return err;
}

struct preinit_clk {
	struct list_head list;
	struct clk *clk;
};
static LIST_HEAD(preinit_clks);
static DEFINE_MUTEX(preinit_lock);
static int init_done;

void clk_debug_register(struct clk *clk)
{
	int err;
	struct clk *pa;

	if (init_done) {
		pa = clk_get_parent(clk);

		if (pa && !IS_ERR(pa) && !pa->dentry)
			clk_debug_register(pa);

		if (!clk->dentry) {
			err = clk_debug_register_one(clk);
			if (err)
				return;
		}
	} else {
		struct preinit_clk *p;
		mutex_lock(&preinit_lock);
		p = kmalloc(sizeof(*p), GFP_KERNEL);
		if (p) {
			p->clk = clk;
			list_add(&p->list, &preinit_clks);
		}
		mutex_unlock(&preinit_lock);
	}
}
EXPORT_SYMBOL_GPL(clk_debug_register);

static int __init clk_debugfs_init(void)
{
	struct preinit_clk *pclk, *tmp;

	init_done = 1;

	mutex_lock(&preinit_lock);
	list_for_each_entry(pclk, &preinit_clks, list) {
		clk_debug_register(pclk->clk);
	}

	list_for_each_entry_safe(pclk, tmp, &preinit_clks, list) {
		list_del(&pclk->list);
		kfree(pclk);
	}
	mutex_unlock(&preinit_lock);
	return 0;
}
late_initcall(clk_debugfs_init);
#endif
