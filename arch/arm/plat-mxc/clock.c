/*
 * Based on arch/arm/plat-omap/clock.c
 *
 * Copyright (C) 2004 - 2005 Nokia corporation
 * Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 * Modified for omap shared clock framework by Tony Lindgren <tony@atomide.com>
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/cpufreq.h>
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
#include <linux/seq_file.h>
#include <linux/semaphore.h>
#include <linux/string.h>

#include <mach/clock.h>

#if (defined(CONFIG_ARCH_MX51) || defined(CONFIG_ARCH_MX37))
extern int dvfs_core_is_active;
extern int lp_high_freq;
extern int lp_med_freq;
extern void dvfs_core_set_bus_freq(void);
#else
int dvfs_core_is_active;
void dvfs_core_set_bus_freq(void)
{
};
#endif

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clockfw_lock);

/*-------------------------------------------------------------------------
 * Standard clock functions defined in include/linux/clk.h
 *-------------------------------------------------------------------------*/

/*
 * All the code inside #ifndef CONFIG_COMMON_CLKDEV can be removed once all
 * MXC architectures have switched to using clkdev.
 */
#ifndef CONFIG_COMMON_CLKDEV
/*
 * Retrieve a clock by name.
 *
 * Note that we first try to use device id on the bus
 * and clock name. If this fails, we try to use "<name>.<id>". If this fails,
 * we try to use clock name only.
 * The reference count to the clock's module owner ref count is incremented.
 */
struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *p, *clk = ERR_PTR(-ENOENT);
	int idno;
	const char *str;

	if (id == NULL)
		return clk;

	if (dev == NULL || dev->bus != &platform_bus_type)
		idno = -1;
	else
		idno = to_platform_device(dev)->id;

	mutex_lock(&clocks_mutex);

	list_for_each_entry(p, &clocks, node) {
		if (p->id == idno &&
		    strcmp(id, p->name) == 0 && try_module_get(p->owner)) {
			clk = p;
			goto found;
		}
	}

	str = strrchr(id, '.');
	if (str) {
		int cnt = str - id;
		str++;
		idno = simple_strtol(str, NULL, 10);
		list_for_each_entry(p, &clocks, node) {
			if (p->id == idno &&
			    strlen(p->name) == cnt &&
			    strncmp(id, p->name, cnt) == 0 &&
			    try_module_get(p->owner)) {
				clk = p;
				goto found;
			}
		}
	}

	list_for_each_entry(p, &clocks, node) {
		if (strcmp(id, p->name) == 0 && try_module_get(p->owner)) {
			clk = p;
			goto found;
		}
	}

	printk(KERN_WARNING "clk: Unable to get requested clock: %s\n", id);

found:
	mutex_unlock(&clocks_mutex);

	return clk;
}
EXPORT_SYMBOL(clk_get);
#endif

static void __clk_disable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk) || !clk->usecount)
		return;

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
	unsigned long flags;
	int ret = 0;

	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	spin_lock_irqsave(&clockfw_lock, flags);

	ret = __clk_enable(clk);

	spin_unlock_irqrestore(&clockfw_lock, flags);

	if ((clk->flags & CPU_FREQ_TRIG_UPDATE)
	    && (clk_get_usecount(clk) == 1)) {
#if defined(CONFIG_CPU_FREQ)
		if (dvfs_core_is_active)
			dvfs_core_set_bus_freq();
#if (defined(CONFIG_ARCH_MX51) || defined(CONFIG_ARCH_MX37))
		else if ((lp_high_freq == 0 && lp_med_freq == 0) ||
			(lp_high_freq == 1) ||
			(lp_high_freq == 0 && lp_med_freq == 1))
#else
		else
#endif
			cpufreq_update_policy(0);
#else
		if (dvfs_core_is_active)
			dvfs_core_set_bus_freq();
#endif
	}
	return ret;
}
EXPORT_SYMBOL(clk_enable);

/* This function decrements the reference count on the clock and disables
 * the clock when reference count is 0. The parent clock tree is
 * recursively disabled
 */
void clk_disable(struct clk *clk)
{
	unsigned long flags;

	if (clk == NULL || IS_ERR(clk))
		return;

	spin_lock_irqsave(&clockfw_lock, flags);

	__clk_disable(clk);

	spin_unlock_irqrestore(&clockfw_lock, flags);

	if ((clk->flags & CPU_FREQ_TRIG_UPDATE)
	    && (clk_get_usecount(clk) == 0)) {
#if defined(CONFIG_CPU_FREQ)
		if (dvfs_core_is_active)
			dvfs_core_set_bus_freq();
#if (defined(CONFIG_ARCH_MX51) || defined(CONFIG_ARCH_MX37))
		else if (lp_high_freq == 0)
#else
		else
#endif
			cpufreq_update_policy(0);
#else
		if (dvfs_core_is_active)
			dvfs_core_set_bus_freq();
#endif
	}
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

	return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

#ifndef CONFIG_COMMON_CLKDEV
/* Decrement the clock's module reference count */
void clk_put(struct clk *clk)
{
	if (clk && !IS_ERR(clk))
		module_put(clk->owner);
}
EXPORT_SYMBOL(clk_put);
#endif

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

/* Propagate rate to children */
void propagate_rate(struct clk *tclk)
{
	struct clk *clkp;

	if (tclk == NULL || IS_ERR(tclk))
		return;

	pr_debug("mxc clock: finding children of %s-%d\n", tclk->name,
		 tclk->id);
	list_for_each_entry(clkp, &clocks, node) {
		if (likely(clkp->parent != tclk))
			continue;
		pr_debug("mxc clock: %s-%d: recalculating rate: old = %lu, ",
			 clkp->name, clkp->id, clkp->rate);
		if (likely((u32) clkp->recalc))
			clkp->recalc(clkp);
		else
			clkp->rate = tclk->rate;
		pr_debug("new = %lu\n", clkp->rate);
		propagate_rate(clkp);
	}
}

/* Set the clock to the requested clock rate. The rate must
 * match a supported rate exactly based on what clk_round_rate returns
 */
int clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long flags;
	int ret = -EINVAL;

	if (clk == NULL || IS_ERR(clk) || clk->set_rate == NULL || rate == 0)
		return ret;

	spin_lock_irqsave(&clockfw_lock, flags);

	ret = clk->set_rate(clk, rate);
	if (unlikely((ret == 0) && (clk->flags & RATE_PROPAGATES)))
		propagate_rate(clk);

	spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

/* Set the clock's parent to another clock source */
int clk_set_parent(struct clk *clk, struct clk *parent)
{
	unsigned long flags;
	int ret = -EINVAL;
	struct clk *prev_parent = clk->parent;

	if (clk == NULL || IS_ERR(clk) || parent == NULL ||
	    IS_ERR(parent) || clk->set_parent == NULL)
		return ret;

	if (clk->usecount != 0) {
		clk_enable(parent);
	}

	spin_lock_irqsave(&clockfw_lock, flags);
	ret = clk->set_parent(clk, parent);
	if (ret == 0) {
		clk->parent = parent;
		if (clk->recalc) {
			clk->recalc(clk);
		} else {
			clk->rate = parent->rate;
		}
		if (unlikely(clk->flags & RATE_PROPAGATES))
			propagate_rate(clk);
	}
	spin_unlock_irqrestore(&clockfw_lock, flags);

	if (clk->usecount != 0) {
		clk_disable(prev_parent);
	}

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

#ifndef CONFIG_COMMON_CLKDEV
/*
 * Add a new clock to the clock tree.
 */
int clk_register(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	mutex_lock(&clocks_mutex);
	list_add(&clk->node, &clocks);
	mutex_unlock(&clocks_mutex);

	return 0;
}
EXPORT_SYMBOL(clk_register);

/* Remove a clock from the clock tree */
void clk_unregister(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	mutex_lock(&clocks_mutex);
	list_del(&clk->node);
	mutex_unlock(&clocks_mutex);
}
EXPORT_SYMBOL(clk_unregister);

#ifdef CONFIG_PROC_FS

static void *mxc_proc_clocks_seq_start(struct seq_file *file, loff_t *index)
{
	unsigned int  i;
	unsigned int  name_length;
	unsigned int  longest_length = 0;
	struct clk    *current_clock = 0;
	struct clk    *clock;

	/* Examine the clock list. */

	i = 0;

	list_for_each_entry(clock, &clocks, node) {
		if (i++ == *index)
			current_clock = clock;
		name_length = strlen(clock->name);
		if (name_length > longest_length)
			longest_length = name_length;
	}

	/* Check if we found the indicated clock. */

	if (!current_clock)
		return NULL;

	/* Stash the length of the longest clock name for later use. */

	file->private = (void *) longest_length;

	/* Return success. */

	return current_clock;
}

static void *mxc_proc_clocks_seq_next(struct seq_file *file, void *data,
								loff_t *index)
{
	struct clk  *current_clock = (struct clk *) data;

	/* Check for nonsense. */

	if (!current_clock)
		return NULL;

	/* Check if the current clock is the last. */

	if (list_is_last(&current_clock->node, &clocks))
		return NULL;

	/* Move to the next clock structure. */

	current_clock = list_entry(current_clock->node.next,
						typeof(*current_clock), node);

	(*index)++;

	/* Return the new current clock. */

	return current_clock;

}

static void mxc_proc_clocks_seq_stop(struct seq_file *file, void *data)
{
}

static int mxc_proc_clocks_seq_show(struct seq_file *file, void *data)
{
	int            result;
	struct clk     *clock = (struct clk *) data;
	struct clk     *parent = clock->parent;
	unsigned int   longest_length = (unsigned int) file->private;
	unsigned long  range_divisor;
	const char     *range_units;

	if (clock->rate >= 1000000) {
		range_divisor = 1000000;
		range_units   = "MHz";
	} else if (clock->rate >= 1000) {
		range_divisor = 1000;
		range_units   = "KHz";
	} else {
		range_divisor = 1;
		range_units   = "Hz";
	}

	if (parent)
		result = seq_printf(file,
			"%s-%-d%*s  %s-%-d%*s  %c%c%c%c%c%c  %3d",
			clock->name,
			clock->id,
			longest_length - strlen(clock->name), "",
			parent->name,
			parent->id,
			longest_length - strlen(parent->name), "",
			(clock->flags & RATE_PROPAGATES)      ? 'P' : '_',
			(clock->flags & ALWAYS_ENABLED)       ? 'A' : '_',
			(clock->flags & RATE_FIXED)           ? 'F' : '_',
			(clock->flags & CPU_FREQ_TRIG_UPDATE) ? 'T' : '_',
			(clock->flags & AHB_HIGH_SET_POINT)   ? 'H' : '_',
			(clock->flags & AHB_MED_SET_POINT)    ? 'M' : '_',
			clock->usecount);
	else
		result = seq_printf(file,
			"%s-%-d%*s  %*s  %c%c%c%c%c%c  %3d",
			clock->name,
			clock->id,
			longest_length - strlen(clock->name), "",
			longest_length + 2, "",
			(clock->flags & RATE_PROPAGATES)      ? 'P' : '_',
			(clock->flags & ALWAYS_ENABLED)       ? 'A' : '_',
			(clock->flags & RATE_FIXED)           ? 'F' : '_',
			(clock->flags & CPU_FREQ_TRIG_UPDATE) ? 'T' : '_',
			(clock->flags & AHB_HIGH_SET_POINT)   ? 'H' : '_',
			(clock->flags & AHB_MED_SET_POINT)    ? 'M' : '_',
			clock->usecount);

	if (result)
		return result;

	result = seq_printf(file, "  %10lu (%lu%s)\n",
		clock->rate,
		clock->rate / range_divisor, range_units);

	return result;

}

static const struct seq_operations mxc_proc_clocks_seq_ops = {
	.start = mxc_proc_clocks_seq_start,
	.next  = mxc_proc_clocks_seq_next,
	.stop  = mxc_proc_clocks_seq_stop,
	.show  = mxc_proc_clocks_seq_show
};

static int mxc_proc_clocks_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &mxc_proc_clocks_seq_ops);
}

static const struct file_operations mxc_proc_clocks_ops = {
	.open    = mxc_proc_clocks_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release,
};

static int __init mxc_setup_proc_entry(void)
{
	struct proc_dir_entry *res;

	res = create_proc_entry("cpu/clocks", 0, NULL);
	if (!res) {
		printk(KERN_ERR "Failed to create proc/cpu/clocks\n");
		return -ENOMEM;
	}
	res->proc_fops = &mxc_proc_clocks_ops;
	return 0;
}

late_initcall(mxc_setup_proc_entry);
#endif /* CONFIG_PROC_FS */
#endif

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

#if !defined CONFIG_ARCH_MX1 && !defined CONFIG_ARCH_MX21
	if (mfn >= 0x200) {
		mfn |= 0xFFFFFE00;
		mfn_abs = -mfn;
	}
#endif

	freq *= 2;
	freq /= pd + 1;

	ll = (unsigned long long)freq * mfn_abs;

	do_div(ll, mfd + 1);
	if (mfn < 0)
		ll = -ll;
	ll = (freq * mfi) + ll;

	return ll;
}
