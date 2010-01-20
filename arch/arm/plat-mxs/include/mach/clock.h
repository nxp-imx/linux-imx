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

#ifndef __ASM_ARM_ARCH_CLOCK_H
#define __ASM_ARM_ARCH_CLOCK_H

#ifndef __ASSEMBLER__

#include <linux/list.h>
#include <asm/clkdev.h>

struct clk {
	int id;
	struct clk *parent;
	struct clk *secondary;
	unsigned long flags;

	unsigned int ref;
	unsigned int scale_bits;
	unsigned int enable_bits;
	unsigned int bypass_bits;
	unsigned int busy_bits;

	unsigned int wait:1;
	unsigned int invert:1;

	void __iomem *enable_reg;
	void __iomem *scale_reg;
	void __iomem *bypass_reg;
	void __iomem *busy_reg;

	/*
	 * Function ptr to set the clock to a new rate. The rate must match a
	 * supported rate returned from round_rate. Leave blank if clock is not
	 * programmable
	 */
	int (*set_rate) (struct clk *, unsigned long);
	/*
	 * Function ptr to get the clock rate.
	 */
	unsigned long (*get_rate) (struct clk *);
	/*
	 * Function ptr to round the requested clock rate to the nearest
	 * supported rate that is less than or equal to the requested rate.
	 */
	unsigned long (*round_rate) (struct clk *, unsigned long);
	/*
	 * Function ptr to enable the clock. Leave blank if clock can not
	 * be gated.
	 */
	int (*enable) (struct clk *);
	/*
	 * Function ptr to disable the clock. Leave blank if clock can not
	 * be gated.
	 */
	void (*disable) (struct clk *);
	/* Function ptr to set the parent clock of the clock. */
	int (*set_parent) (struct clk *, struct clk *);
};

int clk_get_usecount(struct clk *clk);
extern int clk_register(struct clk_lookup *lookup);
extern void clk_unregister(struct clk_lookup *lookup);

static inline int clk_is_busy(struct clk *clk)
{
	return __raw_readl(clk->busy_reg) & (1 << clk->busy_bits);
}

/* Clock flags */
/* 0 ~ 16 attribute flags */
#define ALWAYS_ENABLED		(1 << 0)	/* Clock cannot be disabled */
#define RATE_FIXED		(1 << 1)	/* Fixed clock rate */

/* 16 ~ 23 reservied */
/* 24 ~ 31 run time flags */

#define CLK_REF_UNIT		0x00010000
#define CLK_REF_LIMIT		0xFFFF0000
#define CLK_EN_MASK		0x0000FFFF
#endif /* __ASSEMBLER__ */

#endif
