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

	int ref;
	unsigned int scale_bits;
	unsigned int enable_bits;
	unsigned int bypass_bits;
	unsigned int busy_bits;
	unsigned int xtal_busy_bits;

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

	/* Function ptr to change the parent clock depending
	 * the system configuration at that time.  Will only
	 * change the parent clock if the ref count is 0 (the clock
	 * is not being used)
	 */
	int (*set_sys_dependent_parent) (struct clk *);

};

int clk_get_usecount(struct clk *clk);
extern int clk_register(struct clk_lookup *lookup);
extern void clk_unregister(struct clk_lookup *lookup);

bool clk_enable_h_autoslow(bool enable);
void clk_set_h_autoslow_flags(u16 mask);
void clk_en_public_h_asm_ctrl(bool (*enable_func)(bool),
	void (*set_func)(u16));

struct mxs_emi_scaling_data {
	u32 emi_div;
	u32 frac_div;
	u32 cur_freq;
	u32 new_freq;
};



#ifdef CONFIG_MXS_RAM_FREQ_SCALING
extern int mxs_ram_freq_scale(struct mxs_emi_scaling_data *);
extern u32 mxs_ram_funcs_sz;
#else
static inline int mxs_ram_freq_scale(struct mxs_emi_scaling_data *p)
{
}
static u32 mxs_ram_funcs_sz;
#endif

/* Clock flags */
/* 0 ~ 16 attribute flags */
#define ALWAYS_ENABLED		(1 << 0)	/* Clock cannot be disabled */
#define RATE_FIXED		(1 << 1)	/* Fixed clock rate */
#define CPU_FREQ_TRIG_UPDATE	(1 << 2)	/* CPUFREQ trig update */

/* 16 ~ 23 reservied */
/* 24 ~ 31 run time flags */

#define CLK_REF_UNIT		0x00010000
#define CLK_REF_LIMIT		0xFFFF0000
#define CLK_EN_MASK		0x0000FFFF
#endif /* __ASSEMBLER__ */

#endif
