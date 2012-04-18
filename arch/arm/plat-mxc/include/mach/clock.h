/*
 * Copyright 2005-2012 Freescale Semiconductor, Inc.
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

#ifndef __ASM_ARCH_MXC_CLOCK_H__
#define __ASM_ARCH_MXC_CLOCK_H__

#ifndef __ASSEMBLY__
#include <linux/list.h>

#define CLK_NAME_LEN 32
struct module;

struct clk {
#ifdef CONFIG_CLK_DEBUG
	char name[CLK_NAME_LEN];
	struct dentry           *dentry;
#endif
	int id;
	/* Source clock this clk depends on */
	struct clk *parent;
	/* Secondary clock to enable/disable with this clock */
	struct clk *secondary;
	/* Reference count of clock enable/disable */
	__s8 usecount;
	/* Register bit position for clock's enable/disable control. */
	u8 enable_shift;
	/* Register address for clock's enable/disable control. */
	void __iomem *enable_reg;
	u32 flags;
	/* get the current clock rate (always a fresh value) */
	unsigned long (*get_rate) (struct clk *);
	/* Function ptr to set the clock to a new rate. The rate must match a
	   supported rate returned from round_rate. Leave blank if clock is not
	   programmable */
	int (*set_rate) (struct clk *, unsigned long);
	/* Function ptr to round the requested clock rate to the nearest
	   supported rate that is less than or equal to the requested rate. */
	unsigned long (*round_rate) (struct clk *, unsigned long);
	/* Function ptr to enable the clock. Leave blank if clock can not
	   be gated. */
	int (*enable) (struct clk *);
	/* Function ptr to disable the clock. Leave blank if clock can not
	   be gated. */
	void (*disable) (struct clk *);
	/* Function ptr to set the parent clock of the clock. */
	int (*set_parent) (struct clk *, struct clk *);
};

int clk_get_usecount(struct clk *clk);

/* Clock flags */
#define RATE_PROPAGATES		(1 << 0)	/* Program children too */
#define ALWAYS_ENABLED		(1 << 1)	/* Clock cannot be disabled */
#define RATE_FIXED		(1 << 2)	/* Fixed clock rate */
#define CPU_FREQ_TRIG_UPDATE	(1 << 3)	/* CPUFREQ trig update */
#define AHB_HIGH_SET_POINT	(1 << 4)	/* Requires max AHB clock */
#define AHB_MED_SET_POINT	(1 << 5)	/* Requires med AHB clock */
#define AHB_AUDIO_SET_POINT	(1 << 6)	/* Requires LOW AHB, but higher DDR clock */

unsigned long mxc_decode_pll(unsigned int pll, u32 f_ref);

#ifdef CONFIG_CLK_DEBUG
void clk_debug_register(struct clk *clk);
#else
static inline void clk_debug_register(struct clk *clk) {}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ASM_ARCH_MXC_CLOCK_H__ */
