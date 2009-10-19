/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/spba.h>

/*!
 * @file plat-mxc/spba.c
 *
 * @brief This file contains the SPBA API implementation details.
 *
 * @ingroup SPBA
 */

static DEFINE_SPINLOCK(spba_lock);

#define SPBA_MASTER_MIN                 1
#define SPBA_MASTER_MAX                 7

/*!
 * the base addresses for the SPBA modules
 */
static unsigned long spba_base = (unsigned long)IO_ADDRESS(SPBA_CTRL_BASE_ADDR);

/*!
 * SPBA clock
 */
static struct clk *spba_clk;
/*!
 * This function allows the three masters (A, B, C) to take ownership of a
 * shared peripheral.
 *
 * @param  mod          specified module as defined in \b enum \b #spba_module
 * @param  master       one of more (or-ed together) masters as defined in \b enum \b #spba_masters
 *
 * @return 0 if successful; -1 otherwise.
 */
int spba_take_ownership(int mod, int master)
{
	unsigned long spba_flags;
	__u32 rtn_val = -1;

	if (master < SPBA_MASTER_MIN || master > SPBA_MASTER_MAX) {
		printk("spba_take_ownership() invalide master= %d\n", master);
		BUG();		/* oops */
	}

	if (spba_clk == NULL)
		spba_clk = clk_get(NULL, "spba_clk");

	clk_enable(spba_clk);

	spin_lock_irqsave(&spba_lock, spba_flags);
	__raw_writel(master, spba_base + mod);

	if ((__raw_readl(spba_base + mod) & MXC_SPBA_RAR_MASK) == master) {
		rtn_val = 0;
	}

	spin_unlock_irqrestore(&spba_lock, spba_flags);

	clk_disable(spba_clk);
	return rtn_val;
}

/*!
 * This function releases the ownership for a shared peripheral.
 *
 * @param  mod          specified module as defined in \b enum \b #spba_module
 * @param  master       one of more (or-ed together) masters as defined in \b enum \b #spba_masters
 *
 * @return 0 if successful; -1 otherwise.
 */
int spba_rel_ownership(int mod, int master)
{
	unsigned long spba_flags;
	volatile unsigned long rar;

	if (master < SPBA_MASTER_MIN || master > SPBA_MASTER_MAX) {
		printk("spba_take_ownership() invalide master= %d\n", master);
		BUG();		/* oops */
	}

	if (spba_clk == NULL)
		spba_clk = clk_get(NULL, "spba_clk");

	clk_enable(spba_clk);

	if ((__raw_readl(spba_base + mod) & master) == 0) {
		clk_disable(spba_clk);
		return 0;	/* does not own it */
	}

	spin_lock_irqsave(&spba_lock, spba_flags);

	/* Since only the last 3 bits are writeable, doesn't need to mask off
	   bits 31-3 */
	rar = __raw_readl(spba_base + mod) & (~master);
	__raw_writel(rar, spba_base + mod);

	if ((__raw_readl(spba_base + mod) & master) != 0) {
		spin_unlock_irqrestore(&spba_lock, spba_flags);
		clk_disable(spba_clk);
		return -1;
	}

	spin_unlock_irqrestore(&spba_lock, spba_flags);

	clk_disable(spba_clk);

	return 0;
}

EXPORT_SYMBOL(spba_take_ownership);
EXPORT_SYMBOL(spba_rel_ownership);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SPBA");
MODULE_LICENSE("GPL");
