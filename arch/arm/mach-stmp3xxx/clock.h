/*
 * Clock control driver for Freescale STMP37XX/STMP378X - internal header file
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __ARCH_ARM_STMX3XXX_CLOCK_H__
#define __ARCH_ARM_STMX3XXX_CLOCK_H__

#ifndef __ASSEMBLER__
#include <linux/list.h>

struct clk {
	struct list_head node;
	struct module *owner;
	const char *name;
	struct clk *parent;
	u32 rate;
	s8 usage;
	u32 flags;
	u32 scale_reg;
	u8 scale_shift;
	u8 enable_shift;
	u32 enable_reg;
	int enable_wait;
	int enable_negate;
	u8 bypass_shift;
	u32 bypass_reg;
	u8 busy_bit;
	u32 busy_reg;
	u32 saved_div;
	int (*enable) (struct clk *);
	int (*disable) (struct clk *);
	long (*get_rate) (struct clk *);
	long (*round_rate) (struct clk *, u32);
	int (*set_rate) (struct clk *, u32);
	int (*set_parent) (struct clk *, struct clk *);
	int (*propagate_rate) (struct clk *);
};

struct stmp3xxx_emi_scaling_data {
	u32 emi_div;
	u32 frac_div;
	u32 cur_freq;
	u32 new_freq;
};

#ifdef CONFIG_STMP378X_RAM_FREQ_SCALING
extern void stmp3xxx_ram_freq_scale(struct stmp3xxx_emi_scaling_data *);
extern u32 stmp3xxx_ram_funcs_sz;
#else
static inline void stmp3xxx_ram_freq_scale(struct stmp3xxx_emi_scaling_data *p)
{
}
static u32 stmp3xxx_ram_funcs_sz;
#endif

#endif /* __ASSEMBLER__ */

/* Flags */
#define RATE_PROPAGATES      (1<<0)
#define NEEDS_INITIALIZATION (1<<1)
#define PARENT_SET_RATE      (1<<2)
#define FIXED_RATE           (1<<3)
#define ENABLED	             (1<<4)
#define NEEDS_SET_PARENT     (1<<5)
#define PRESENT_ON_STMP37XX  (1<<8)
#define PRESENT_ON_STMP378X  (1<<9)

#define SCALING_DATA_EMI_DIV_OFFSET	0
#define SCALING_DATA_FRAC_DIV_OFFSET	4
#define SCALING_DATA_CUR_FREQ_OFFSET	8
#define SCALING_DATA_NEW_FREQ_OFFSET	12

#endif
