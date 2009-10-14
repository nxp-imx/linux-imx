/*
 * Clock manipulation routines for Freescale STMP37XX/STMP378X
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
/* #define DEBUG */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <mach/cpu.h>
#include <mach/regs-clkctrl.h>

#include "clock.h"

static DEFINE_SPINLOCK(clocks_lock);

static struct clk osc_24M;
static struct clk pll_clk;
static struct clk cpu_clk;
static struct clk hclk;

static int std_propagate_rate(struct clk *);

static inline int clk_is_busy(struct clk *clk)
{
	return __raw_readl(clk->busy_reg) & (1 << clk->busy_bit);
}

static int std_clk_enable(struct clk *clk)
{
	if (clk->enable_reg) {
		u32 clk_reg = __raw_readl(clk->enable_reg);
		pr_debug("%s: clock '%s'\n", __func__, clk->name);
		if (clk->enable_negate)
			clk_reg &= ~(1 << clk->enable_shift);
		else
			clk_reg |= (1 << clk->enable_shift);
		__raw_writel(clk_reg, clk->enable_reg);
		if (clk->enable_wait)
			udelay(clk->enable_wait);
		return 0;
	} else
		return -EINVAL;
}

static int std_clk_disable(struct clk *clk)
{
	if (clk->enable_reg) {
		u32 clk_reg = __raw_readl(clk->enable_reg);
		pr_debug("%s: clock '%s'\n", __func__, clk->name);
		if (clk->enable_negate)
			clk_reg |= (1 << clk->enable_shift);
		else
			clk_reg &= ~(1 << clk->enable_shift);
		__raw_writel(clk_reg, clk->enable_reg);
		return 0;
	} else
		return -EINVAL;
}

static int io_set_rate(struct clk *clk, u32 rate)
{
	u32 reg_frac, clkctrl_frac;
	int i, ret = 0, mask = 0x1f;

	clkctrl_frac = (clk->parent->rate * 18 + rate - 1) / rate;

	if (clkctrl_frac < 18 || clkctrl_frac > 35) {
		ret = -EINVAL;
		goto out;
	}

	reg_frac = __raw_readl(clk->scale_reg);
	reg_frac &= ~(mask << clk->scale_shift);
	__raw_writel(reg_frac | (clkctrl_frac << clk->scale_shift),
				clk->scale_reg);
	if (clk->busy_reg) {
		for (i = 10000; i; i--)
			if (!clk_is_busy(clk))
				break;
		if (!i)
			ret = -ETIMEDOUT;
		else
			ret = 0;
	}
out:
	return ret;
}

static long io_get_rate(struct clk *clk)
{
	long rate = clk->parent->rate * 18;
	int mask = 0x1f;

	rate /= (__raw_readl(clk->scale_reg) >> clk->scale_shift) & mask;
	pr_debug("rate now %ld\n", rate);
	clk->rate = rate;

	return rate;
}

static long per_get_rate(struct clk *clk)
{
	long rate = clk->parent->rate;
	long div;
	const int mask = 0xff;

	pr_debug("%s: clock name %s\n", __func__, clk->name);
	if (clk->enable_reg &&
			!(__raw_readl(clk->enable_reg) & clk->enable_shift))
		clk->rate = 0;
	else {
		div = (__raw_readl(clk->scale_reg) >> clk->scale_shift) & mask;
		if (div)
			rate /= div;
		else
			printk(KERN_WARNING "clock '%s' has divisor 0!\n",
				clk->name);
		clk->rate = rate;
	}

	return 0;
}

static int per_set_rate(struct clk *clk, u32 rate)
{
	int ret = -EINVAL;
	int div = (clk->parent->rate + rate - 1) / rate;
	u32 reg_frac;
	const int mask = 0xff;
	int try = 10;
	int i = -1;

	if (div == 0 || div > mask)
		goto out;

	reg_frac = __raw_readl(clk->scale_reg);
	reg_frac &= ~(mask << clk->scale_shift);

	while (try--) {
		__raw_writel(reg_frac | (div << clk->scale_shift),
				clk->scale_reg);

		if (clk->busy_reg) {
			for (i = 10000; i; i--)
				if (!clk_is_busy(clk))
					break;
		}
		if (i)
			break;
	}

	if (!i)
		ret = -ETIMEDOUT;
	else
		ret = 0;

out:
	BUG_ON(ret != 0);
	return ret;
}

static long lcdif_get_rate(struct clk *clk)
{
	long rate = clk->parent->rate;
	long div;
	const int mask = 0xff;

	pr_debug("%s: clock name %s\n", __func__, clk->name);
	div = (__raw_readl(clk->scale_reg) >> clk->scale_shift) & mask;
	if (div) {
		rate /= div;
		div = (HW_CLKCTRL_FRAC_RD() & BM_CLKCTRL_FRAC_PIXFRAC) >>
				BP_CLKCTRL_FRAC_PIXFRAC;
		rate /= div;
	} else
		printk(KERN_WARNING "clock '%s' has divisor 0!\n", clk->name);
	clk->rate = rate;

	return 0;
}

static int lcdif_set_rate(struct clk *clk, u32 rate)
{
	int ret = 0;
	/*
	 * On 3700, we can get most timings exact by modifying ref_pix
	 * and the divider, but keeping the phase timings at 1 (2
	 * phases per cycle).
	 *
	 * ref_pix can be between 480e6*18/35=246.9MHz and 480e6*18/18=480MHz,
	 * which is between 18/(18*480e6)=2.084ns and 35/(18*480e6)=4.050ns.
	 *
	 * ns_cycle >= 2*18e3/(18*480) = 25/6
	 * ns_cycle <= 2*35e3/(18*480) = 875/108
	 *
	 * Multiply the ns_cycle by 'div' to lengthen it until it fits the
	 * bounds. This is the divider we'll use after ref_pix.
	 *
	 * 6 * ns_cycle >= 25 * div
	 * 108 * ns_cycle <= 875 * div
	 */
	u32 ns_cycle = 1000000 / rate;
	ns_cycle *= 2; /* Fix calculate double frequency */
	u32 div, reg_val;
	u32 lowest_result = (u32) -1;
	u32 lowest_div = 0, lowest_fracdiv = 0;

	for (div = 1; div < 256; ++div) {
		u32 fracdiv;
		u32 ps_result;
		int lower_bound = 6 * ns_cycle >= 25 * div;
		int upper_bound = 108 * ns_cycle <= 875 * div;
		if (!lower_bound)
			break;
		if (!upper_bound)
			continue;
		/*
		 * Found a matching div. Calculate fractional divider needed,
		 * rounded up.
		 */
		fracdiv = ((clk->parent->rate / 1000 * 18 / 2) *
				ns_cycle + 1000 * div - 1) /
				(1000 * div);
		if (fracdiv < 18 || fracdiv > 35) {
			ret = -EINVAL;
			goto out;
		}
		/* Calculate the actual cycle time this results in */
		ps_result = 6250 * div * fracdiv / 27;

		/* Use the fastest result that doesn't break ns_cycle */
		if (ps_result <= lowest_result) {
			lowest_result = ps_result;
			lowest_div = div;
			lowest_fracdiv = fracdiv;
		}
	}

	if (div >= 256 || lowest_result == (u32) -1) {
		ret = -EINVAL;
		goto out;
	}
	pr_debug("Programming PFD=%u,DIV=%u ref_pix=%uMHz "
			"PIXCLK=%uMHz cycle=%u.%03uns\n",
			lowest_fracdiv, lowest_div,
			480*18/lowest_fracdiv, 480*18/lowest_fracdiv/lowest_div,
			lowest_result / 1000, lowest_result % 1000);

	/* Program ref_pix phase fractional divider */
	HW_CLKCTRL_FRAC_WR((HW_CLKCTRL_FRAC_RD() & ~BM_CLKCTRL_FRAC_PIXFRAC) |
			   BF_CLKCTRL_FRAC_PIXFRAC(lowest_fracdiv));
	/* Ungate PFD */
	HW_CLKCTRL_FRAC_CLR(BM_CLKCTRL_FRAC_CLKGATEPIX);

	/* Program pix divider */
	reg_val = __raw_readl(clk->scale_reg);
	reg_val &= ~(BM_CLKCTRL_PIX_DIV | BM_CLKCTRL_PIX_CLKGATE);
	reg_val |= BF_CLKCTRL_PIX_DIV(lowest_div);
	__raw_writel(reg_val, clk->scale_reg);

	/* Wait for divider update */
	if (clk->busy_reg) {
		int i;
		for (i = 10000; i; i--)
			if (!clk_is_busy(clk))
				break;
		if (!i) {
			ret = -ETIMEDOUT;
			goto out;
		}
	}

	/* Switch to ref_pix source */
	HW_CLKCTRL_CLKSEQ_CLR(BM_CLKCTRL_CLKSEQ_BYPASS_PIX);

out:
	return ret;
}


static int cpu_set_rate(struct clk *clk, u32 rate)
{
	if (rate < 24000)
		return -EINVAL;
	else if (rate == 24000) {
		/* switch to the 24M source */
		clk_set_parent(clk, &osc_24M);
	} else {
		int i;
		u32 clkctrl_cpu = 1;
		u32 c = clkctrl_cpu;
		u32 clkctrl_frac = 1;
		u32 val;
		for ( ; c < 0x40; c++) {
			u32 f = (pll_clk.rate*18/c + rate/2) / rate;
			int s1, s2;

			if (f < 18 || f > 35)
				continue;
			s1 = pll_clk.rate*18/clkctrl_frac/clkctrl_cpu - rate;
			s2 = pll_clk.rate*18/c/f - rate;
			pr_debug("%s: s1 %d, s2 %d\n", __func__, s1, s2);
			if (abs(s1) > abs(s2)) {
				clkctrl_cpu = c;
				clkctrl_frac = f;
			}
			if (s2 == 0)
				break;
		};
		pr_debug("%s: clkctrl_cpu %d, clkctrl_frac %d\n", __func__,
				clkctrl_cpu, clkctrl_frac);
		if (c == 0x40) {
			int  d = pll_clk.rate*18/clkctrl_frac/clkctrl_cpu -
				rate;
			if (abs(d) > 100 ||
			    clkctrl_frac < 18 || clkctrl_frac > 35)
				return -EINVAL;
		}

		/* 4.6.2 */
		val = __raw_readl(clk->scale_reg);
		val &= ~(0x3f << clk->scale_shift);
		val |= clkctrl_frac;
		clk_set_parent(clk, &osc_24M);
		udelay(10);
		__raw_writel(val, clk->scale_reg);
		/* ungate */
		__raw_writel(1<<7, clk->scale_reg + 8);
		/* write clkctrl_cpu */
		clk->saved_div = clkctrl_cpu;
		HW_CLKCTRL_CPU_WR((HW_CLKCTRL_CPU_RD() & ~0x3f) | clkctrl_cpu);
		for (i = 10000; i; i--)
			if (!clk_is_busy(clk))
				break;
		if (!i) {
			printk(KERN_ERR "couldn't set up CPU divisor\n");
			return -ETIMEDOUT;
		}
		clk_set_parent(clk, &pll_clk);
		clk->saved_div = 0;
		udelay(10);
	}
	return 0;
}

static long cpu_get_rate(struct clk *clk)
{
	long rate = clk->parent->rate * 18;

	rate /= (__raw_readl(clk->scale_reg) >> clk->scale_shift) & 0x3f;
	rate /= HW_CLKCTRL_CPU_RD() & 0x3f;
	rate = ((rate + 9) / 10) * 10;
	clk->rate = rate;

	return rate;
}

static long cpu_round_rate(struct clk *clk, u32 rate)
{
	unsigned long r = 0;

	if (rate <= 24000)
		r = 24000;
	else {
		u32 clkctrl_cpu = 1;
		u32 clkctrl_frac;
		do {
			clkctrl_frac =
				(pll_clk.rate*18 / clkctrl_cpu + rate/2) / rate;
			if (clkctrl_frac > 35)
				continue;
			if (pll_clk.rate*18 / clkctrl_frac / clkctrl_cpu/10 ==
			    rate / 10)
				break;
		} while (pll_clk.rate / 2  >= clkctrl_cpu++ * rate);
		if (pll_clk.rate / 2 < (clkctrl_cpu - 1) * rate)
			clkctrl_cpu--;
		pr_debug("%s: clkctrl_cpu %d, clkctrl_frac %d\n", __func__,
				clkctrl_cpu, clkctrl_frac);
		if (clkctrl_frac < 18)
			clkctrl_frac = 18;
		if (clkctrl_frac > 35)
			clkctrl_frac = 35;

		r = pll_clk.rate * 18;
		r /= clkctrl_frac;
		r /= clkctrl_cpu;
		r = 10 * ((r + 9) / 10);
	}
	return r;
}

static int emi_set_rate(struct clk *clk, u32 rate)
{
	int ret = 0;

	if (rate < 24000)
		return -EINVAL;
	else {
		int i;
		struct stmp3xxx_emi_scaling_data sc_data;
		int (*scale)(struct stmp3xxx_emi_scaling_data *) =
			(void *)STMP3XXX_OCRAM_VA_BASE;
		void *saved_ocram;
		u32 clkctrl_emi;
		u32 clkctrl_frac;
		int div = 1;
		/*
		 * We've been setting div to HW_CLKCTRL_CPU_RD() & 0x3f so far.
		 * TODO: verify 1 is still valid.
		 */

		if (!stmp3xxx_ram_funcs_sz)
			goto out;

		for (clkctrl_emi = div; clkctrl_emi < 0x3f;
					clkctrl_emi += div) {
			clkctrl_frac =
				(pll_clk.rate * 18 + rate * clkctrl_emi / 2) /
					(rate * clkctrl_emi);
			if (clkctrl_frac >= 18 && clkctrl_frac <= 35) {
				pr_debug("%s: clkctrl_frac found %d for %d\n",
					__func__, clkctrl_frac, clkctrl_emi);
				if (pll_clk.rate * 18 /
					clkctrl_frac / clkctrl_emi / 100 ==
					rate / 100)
					break;
			}
		}
		if (clkctrl_emi >= 0x3f)
			return -EINVAL;
		pr_debug("%s: clkctrl_emi %d, clkctrl_frac %d\n",
			__func__, clkctrl_emi, clkctrl_frac);

		saved_ocram = kmalloc(stmp3xxx_ram_funcs_sz, GFP_KERNEL);
		if (!saved_ocram)
			return -ENOMEM;
		memcpy(saved_ocram, scale, stmp3xxx_ram_funcs_sz);
		memcpy(scale, stmp3xxx_ram_freq_scale, stmp3xxx_ram_funcs_sz);

		sc_data.emi_div = clkctrl_emi;
		sc_data.frac_div = clkctrl_frac;
		sc_data.cur_freq = clk->rate / 1000;
		sc_data.new_freq = rate / 1000;

		local_irq_disable();
		local_fiq_disable();

		scale(&sc_data);

		local_fiq_enable();
		local_irq_enable();

		for (i = 10000; i; i--)
			if (!clk_is_busy(clk))
				break;
		memcpy(scale, saved_ocram, stmp3xxx_ram_funcs_sz);
		kfree(saved_ocram);

		if (!i) {
			printk(KERN_ERR "couldn't set up EMI divisor\n");
			ret = -ETIMEDOUT;
			goto out;
		}
	}
out:
	return ret;
}

static long emi_get_rate(struct clk *clk)
{
	long rate = clk->parent->rate * 18;

	rate /= (__raw_readl(clk->scale_reg) >> clk->scale_shift) & 0x3f;
	rate /= HW_CLKCTRL_EMI_RD() & 0x3f;
	clk->rate = rate;

	return rate;
}

static int clkseq_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;
	int shift = 8;

	/* bypass? */
	if (parent == &osc_24M)
		shift = 4;

	if (clk->bypass_reg) {
		u32 hbus_mask = BM_CLKCTRL_HBUS_DIV_FRAC_EN |
				BM_CLKCTRL_HBUS_DIV;

		if (clk == &cpu_clk && shift == 4) {
			u32 hbus_val = HW_CLKCTRL_HBUS_RD();
			u32 cpu_val = HW_CLKCTRL_CPU_RD();
			hbus_val &= ~hbus_mask;
			hbus_val |= 1;
			clk->saved_div = cpu_val & BM_CLKCTRL_CPU_DIV_CPU;
			cpu_val &= ~BM_CLKCTRL_CPU_DIV_CPU;
			cpu_val |= 1;
			__raw_writel(1 << clk->bypass_shift,
					clk->bypass_reg + shift);
			if (machine_is_stmp378x()) {
				HW_CLKCTRL_HBUS_WR(hbus_val);
				HW_CLKCTRL_CPU_WR(cpu_val);
				hclk.rate = 0;
			}
		} else if (clk == &cpu_clk && shift == 8) {
			u32 hbus_val = HW_CLKCTRL_HBUS_RD();
			u32 cpu_val = HW_CLKCTRL_CPU_RD();
			hbus_val &= ~hbus_mask;
			hbus_val |= 2;
			cpu_val &= ~BM_CLKCTRL_CPU_DIV_CPU;
			if (clk->saved_div)
				cpu_val |= clk->saved_div;
			else
				cpu_val |= 2;
			if (machine_is_stmp378x()) {
				HW_CLKCTRL_HBUS_WR(hbus_val);
				HW_CLKCTRL_CPU_WR(cpu_val);
				hclk.rate = 0;
			}
			__raw_writel(1 << clk->bypass_shift,
					clk->bypass_reg + shift);
		} else
			__raw_writel(1 << clk->bypass_shift,
					clk->bypass_reg + shift);

		ret = 0;
	}

	return ret;
}

static int hbus_set_rate(struct clk *clk, u32 rate)
{
	u8 div = 0;
	int is_frac = 0;
	u32 clkctrl_hbus;
	struct clk *parent = clk->parent;

	pr_debug("%s: rate %d, parent rate %d\n", __func__, rate,
			parent->rate);

	if (rate > parent->rate)
		return -EINVAL;

	if (((parent->rate + rate/2) / rate) * rate != parent->rate &&
	    parent->rate / rate < 32) {
		pr_debug("%s: switching to fractional mode\n", __func__);
		is_frac = 1;
	}

	if (is_frac)
		div = (32 * rate + parent->rate / 2) / parent->rate;
	else
		div = (parent->rate + rate - 1) / rate;
	pr_debug("%s: div calculated is %d\n", __func__, div);
	if (!div || div > 0x1f)
		return -EINVAL;

	clk_set_parent(&cpu_clk, &osc_24M);
	udelay(10);
	clkctrl_hbus = __raw_readl(clk->scale_reg);
	clkctrl_hbus &= ~0x3f;
	clkctrl_hbus |= div;
	clkctrl_hbus |= (is_frac << 5);

	__raw_writel(clkctrl_hbus, clk->scale_reg);
	if (clk->busy_reg) {
		int i;
		for (i = 10000; i; i--)
			if (!clk_is_busy(clk))
				break;
		if (!i) {
			printk(KERN_ERR "couldn't set up CPU divisor\n");
			return -ETIMEDOUT;
		}
	}
	clk_set_parent(&cpu_clk, &pll_clk);
	__raw_writel(clkctrl_hbus, clk->scale_reg);
	udelay(10);
	return 0;
}

static long hbus_get_rate(struct clk *clk)
{
	long rate = clk->parent->rate;

	if (__raw_readl(clk->scale_reg) & 0x20) {
		rate *= __raw_readl(clk->scale_reg) & 0x1f;
		rate /= 32;
	} else
		rate /= __raw_readl(clk->scale_reg) & 0x1f;
	clk->rate = rate;

	return rate;
}

static int xbus_set_rate(struct clk *clk, u32 rate)
{
	u16 div = 0;
	u32 clkctrl_xbus;

	pr_debug("%s: rate %d, parent rate %d\n", __func__, rate,
			clk->parent->rate);

	div = (clk->parent->rate + rate - 1) / rate;
	pr_debug("%s: div calculated is %d\n", __func__, div);
	if (!div || div > 0x3ff)
		return -EINVAL;

	clkctrl_xbus = __raw_readl(clk->scale_reg);
	clkctrl_xbus &= ~0x3ff;
	clkctrl_xbus |= div;
	__raw_writel(clkctrl_xbus, clk->scale_reg);
	if (clk->busy_reg) {
		int i;
		for (i = 10000; i; i--)
			if (!clk_is_busy(clk))
				break;
		if (!i) {
			printk(KERN_ERR "couldn't set up xbus divisor\n");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static long xbus_get_rate(struct clk *clk)
{
	long rate = clk->parent->rate;

	rate /= __raw_readl(clk->scale_reg) & 0x3ff;
	clk->rate = rate;

	return rate;
}


static int etm_set_rate(struct clk *clk, u32 rate)
{
	u16 div = 0;
	u32 clkctrl_etm;

	pr_debug("%s: rate %d, parent rate %d\n", __func__, rate,
			clk->parent->rate);

	div = (clk->parent->rate + rate - 1) / rate;
	pr_debug("%s: div calculated is %d\n", __func__, div);
	if (!div || div > 0x3f)
		return -EINVAL;

	clkctrl_etm = __raw_readl(clk->scale_reg);
	clkctrl_etm &= ~0x3f;
	clkctrl_etm |= div;
	__raw_writel(clkctrl_etm, clk->scale_reg);
	if (clk->busy_reg) {
		int i;
		for (i = 10000; i; i--)
			if (!clk_is_busy(clk))
				break;
		if (!i) {
			printk(KERN_ERR "couldn't set up ETM clock divisor\n");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static long etm_get_rate(struct clk *clk)
{
	long rate = clk->parent->rate;
	rate /= (__raw_readl(clk->scale_reg) >> clk->scale_shift) & 0x3f;
	clk->rate = rate;

	return rate;
}

/* List of on-chip clocks */

static struct clk osc_24M = {
	.name		= "osc_24M",
	.flags		= FIXED_RATE | ENABLED |
			PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
	.rate		= 24000,
	.propagate_rate = std_propagate_rate,
};

static struct clk pll_clk = {
	.name		= "pll",
	.parent		= &osc_24M,
	.enable_reg	= HW_CLKCTRL_PLLCTRL0_ADDR,
	.enable_shift	= 16,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_wait	= 10,
	.flags		= FIXED_RATE | ENABLED |
			PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
	.rate		= 480000,
	.propagate_rate = std_propagate_rate,
};

static struct clk cpu_clk = {
	.name		= "cpu",
	.parent		= &pll_clk,
	.get_rate	= cpu_get_rate,
	.set_rate	= cpu_set_rate,
	.round_rate	= cpu_round_rate,
	.scale_reg	= HW_CLKCTRL_FRAC_ADDR,
	.scale_shift	= 0,
	.bypass_reg	= HW_CLKCTRL_CLKSEQ_ADDR,
	.bypass_shift	= 7,
	.busy_reg	= HW_CLKCTRL_CPU_ADDR,
	.busy_bit	= 28,
	.set_parent	= clkseq_set_parent,
	.flags		= RATE_PROPAGATES | ENABLED |
			PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
	.propagate_rate = std_propagate_rate,
};

static struct clk io_clk = {
	.name		= "io",
	.parent		= &pll_clk,
	.enable_reg	= HW_CLKCTRL_FRAC_ADDR,
	.enable_shift	= 31,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.get_rate	= io_get_rate,
	.set_rate	= io_set_rate,
	.scale_reg	= HW_CLKCTRL_FRAC_ADDR,
	.scale_shift	= 24,
	.flags		= RATE_PROPAGATES | ENABLED |
			PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
	.propagate_rate = std_propagate_rate,
};

static struct clk hclk = {
	.name		= "hclk",
	.parent		= &cpu_clk,
	.get_rate	= hbus_get_rate,
	.set_rate	= hbus_set_rate,
	.scale_reg	= HW_CLKCTRL_HBUS_ADDR,
	.bypass_reg	= HW_CLKCTRL_CLKSEQ_ADDR,
	.bypass_shift	= 7,
	.busy_reg	= HW_CLKCTRL_HBUS_ADDR,
	.busy_bit	= 29,
	.flags		= RATE_PROPAGATES | ENABLED |
			PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
	.propagate_rate = std_propagate_rate,
};

static struct clk xclk = {
	.name		= "xclk",
	.parent		= &osc_24M,
	.get_rate	= xbus_get_rate,
	.set_rate	= xbus_set_rate,
	.scale_reg	= HW_CLKCTRL_XBUS_ADDR,
	.busy_reg	= HW_CLKCTRL_XBUS_ADDR,
	.busy_bit	= 31,
	.flags		= RATE_PROPAGATES | ENABLED |
			PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
	.propagate_rate = std_propagate_rate,
};

static struct clk uart_clk = {
	.name		= "uart",
	.parent		= &xclk,
	.enable_reg	= HW_CLKCTRL_XTAL_ADDR,
	.enable_shift	= 31,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.flags		= ENABLED | PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk audio_clk = {
	.name		= "audio",
	.parent		= &xclk,
	.enable_reg	= HW_CLKCTRL_XTAL_ADDR,
	.enable_shift	= 30,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.flags		= PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk pwm_clk = {
	.name		= "pwm",
	.parent		= &xclk,
	.enable_reg	= HW_CLKCTRL_XTAL_ADDR,
	.enable_shift	= 29,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.flags		= PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk dri_clk = {
	.name		= "dri",
	.parent		= &xclk,
	.enable_reg	= HW_CLKCTRL_XTAL_ADDR,
	.enable_shift	= 28,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.flags		= PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk digctl_clk = {
	.name		= "digctl",
	.parent		= &xclk, /* XXX? */
	.enable_reg	= HW_CLKCTRL_XTAL_ADDR,
	.enable_shift	= 27,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.flags		= PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk timer_clk = {
	.name		= "timer",
	.parent		= &xclk, /* XXX? */
	.enable_reg	= HW_CLKCTRL_XTAL_ADDR,
	.enable_shift	= 26,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.flags		= ENABLED | PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk lcdif_clk = {
	.name		= "lcdif",
	.parent		= &pll_clk,
	.get_rate	= lcdif_get_rate,
	.set_rate	= lcdif_set_rate,
	.scale_reg	= HW_CLKCTRL_PIX_ADDR,
	.busy_reg	= HW_CLKCTRL_PIX_ADDR,
	.busy_bit	= 29,
	.enable_reg	= HW_CLKCTRL_PIX_ADDR,
	.enable_shift	= 31,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.bypass_reg	= HW_CLKCTRL_CLKSEQ_ADDR,
	.bypass_shift	= 1,
	.set_parent	= clkseq_set_parent,
	.flags		= PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X |
			  NEEDS_SET_PARENT,
};

static struct clk ssp_clk = {
	.name		= "ssp",
	.parent		= &io_clk,
	.get_rate	= per_get_rate,
	.set_rate	= per_set_rate,
	.scale_reg	= HW_CLKCTRL_SSP_ADDR,
	.busy_reg	= HW_CLKCTRL_SSP_ADDR,
	.busy_bit	= 29,
	.enable_reg	= HW_CLKCTRL_SSP_ADDR,
	.enable_shift	= 31,
	.bypass_reg	= HW_CLKCTRL_CLKSEQ_ADDR,
	.bypass_shift	= 5,
	.set_parent	= clkseq_set_parent,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.flags		= NEEDS_SET_PARENT |
			PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk gpmi_clk = {
	.name		= "gpmi",
	.parent		= &io_clk,
	.get_rate	= per_get_rate,
	.set_rate	= per_set_rate,
	.scale_reg	= HW_CLKCTRL_GPMI_ADDR,
	.busy_reg	= HW_CLKCTRL_GPMI_ADDR,
	.busy_bit	= 29,
	.enable_reg	= HW_CLKCTRL_GPMI_ADDR,
	.enable_shift	= 31,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.bypass_reg	= HW_CLKCTRL_CLKSEQ_ADDR,
	.bypass_shift	= 4,
	.set_parent	= clkseq_set_parent,
	.flags		= NEEDS_SET_PARENT |
			PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk spdif_clk = {
	.name		= "spdif",
	.parent		= &pll_clk,
	.enable_reg	= HW_CLKCTRL_SPDIF_ADDR,
	.enable_shift	= 31,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.flags		= PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk emi_clk = {
	.name		= "emi",
	.parent		= &pll_clk,
	.enable_reg	= HW_CLKCTRL_EMI_ADDR,
	.enable_shift	= 31,
	.enable_negate	= 1,
	.scale_reg	= HW_CLKCTRL_FRAC_ADDR,
	.scale_shift	= 8,
	.get_rate	= emi_get_rate,
	.set_rate	= emi_set_rate,
	.busy_reg	= HW_CLKCTRL_EMI_ADDR,
	.busy_bit	= 28,
	.bypass_reg	= HW_CLKCTRL_CLKSEQ_ADDR,
	.bypass_shift	= 6,
	.set_parent	= clkseq_set_parent,
	.flags		= ENABLED | PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk ir_clk = {
	.name		= "ir",
	.parent		= &io_clk,
	.enable_reg	= HW_CLKCTRL_IR_ADDR,
	.enable_shift	= 31,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.bypass_reg	= HW_CLKCTRL_CLKSEQ_ADDR,
	.bypass_shift	= 3,
	.set_parent	= clkseq_set_parent,
	.flags		= NEEDS_SET_PARENT |
			PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk saif_clk = {
	.name		= "saif",
	.parent		= &pll_clk,
	.get_rate	= per_get_rate,
	.set_rate	= per_set_rate,
	.scale_reg	= HW_CLKCTRL_SAIF_ADDR,
	.busy_reg	= HW_CLKCTRL_SAIF_ADDR,
	.busy_bit	= 29,
	.enable_reg	= HW_CLKCTRL_SAIF_ADDR,
	.enable_shift	= 31,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.bypass_reg	= HW_CLKCTRL_CLKSEQ_ADDR,
	.bypass_shift	= 0,
	.set_parent	= clkseq_set_parent,
	.flags		= NEEDS_SET_PARENT |
			PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk usb_clk = {
	.name		= "usb",
	.parent		= &pll_clk,
	.enable_reg	= HW_CLKCTRL_PLLCTRL0_ADDR,
	.enable_shift	= 18,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.flags		= PRESENT_ON_STMP37XX | PRESENT_ON_STMP378X,
};

static struct clk vid_clk = {
	.name		= "ref_vid",
	.parent		= &osc_24M,
#ifdef CONFIG_MACH_STMP378X
	.enable_reg	= HW_CLKCTRL_FRAC1_ADDR,
	.enable_shift	= 31,
	.enable_negate	= 1,
#endif
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.rate		= 432000,
	.flags		= FIXED_RATE | PRESENT_ON_STMP378X,
};

static struct clk clk_tv108M_ng = {
	.name		= "tv108M_ng",
	.parent		= &vid_clk,
#ifdef CONFIG_MACH_STMP378X
	.enable_reg	= HW_CLKCTRL_TV_ADDR,
	.enable_shift	= 31,
	.enable_negate	= 1,
#endif
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.rate		= 108000,
	.flags		= FIXED_RATE | PRESENT_ON_STMP378X,
};

static struct clk clk_tv54M = {
	.name		= "tv54M",
	.parent		= &vid_clk,
#ifdef CONFIG_MACH_STMP378X
	.enable_reg	= HW_CLKCTRL_TV_ADDR,
	.enable_shift	= 30,
	.enable_negate	= 1,
#endif
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.rate		= 54000,
	.flags		= FIXED_RATE | PRESENT_ON_STMP378X,
};

static struct clk clk_tv27M = {
	.name		= "tv27M",
	.parent		= &vid_clk,
#ifdef CONFIG_MACH_STMP378X
	.enable_reg	= HW_CLKCTRL_TV_ADDR,
	.enable_shift	= 30,
	.enable_negate	= 1,
#endif
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.rate		= 27000,
	.flags		= FIXED_RATE | PRESENT_ON_STMP378X,
};

static struct clk clk_tvenc_fifo = {
	.name		= "tvenc_fifo",
	.parent		= &vid_clk,
	.flags		= PRESENT_ON_STMP378X,
};

static struct clk clk_etm = {
	.name		= "etm",
	.parent		= &pll_clk,
#ifdef CONFIG_MACH_STMP378X
	.enable_reg	= HW_CLKCTRL_ETM_ADDR,
	.enable_shift	= 31,
	.enable		= std_clk_enable,
	.disable	= std_clk_disable,
	.enable_negate	= 1,
	.scale_reg	= HW_CLKCTRL_ETM_ADDR,
	.scale_shift	= 0,
	.get_rate	= etm_get_rate,
	.set_rate	= etm_set_rate,
	.bypass_reg	= HW_CLKCTRL_CLKSEQ_ADDR,
	.bypass_shift	= 8,
	.busy_reg	= HW_CLKCTRL_ETM_ADDR,
	.busy_bit	= 29,
#endif
	.set_parent	= clkseq_set_parent,
	.flags		= PRESENT_ON_STMP378X,
};


/* list of all the clocks */
static struct clk *onchip_clks[] = {
	&osc_24M,
	&pll_clk,
	&cpu_clk,
	&hclk,
	&xclk,
	&io_clk,
	&uart_clk,
	&audio_clk,
	&pwm_clk,
	&dri_clk,
	&digctl_clk,
	&timer_clk,
	&lcdif_clk,
	&ssp_clk,
	&gpmi_clk,
	&spdif_clk,
	&emi_clk,
	&ir_clk,
	&saif_clk,
	&usb_clk,
	&vid_clk,
	&clk_tv108M_ng,
	&clk_tv54M,
	&clk_tv27M,
	&clk_etm,
};

static int std_propagate_rate(struct clk *clk)
{
	struct clk **clkp = onchip_clks;

	for (clkp = onchip_clks; clkp < onchip_clks + ARRAY_SIZE(onchip_clks);
	     clkp++) {
		if (!((*clkp)->flags & PRESENT_ON_STMP37XX &&
				cpu_is_stmp37xx()) &&
			!((*clkp)->flags & PRESENT_ON_STMP378X &&
				cpu_is_stmp378x()))
			continue;

		if ((*clkp)->parent == clk && (*clkp)->get_rate) {
			((*clkp)->get_rate)(*clkp);
			if ((*clkp)->flags & RATE_PROPAGATES)
				((*clkp)->propagate_rate)(*clkp);
		}
	}

	return 0;
}

/* Exported API */
unsigned long clk_get_rate(struct clk *clk)
{
	if (IS_ERR(clk))
		return 0;

	if (clk->rate != 0)
		return clk->rate;

	if (clk->get_rate != NULL)
		return clk->get_rate(clk);

	return clk->parent ? clk->parent->rate : 0;
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (!IS_ERR(clk) && clk->round_rate)
		return clk->round_rate(clk, rate);

	return 0;
}
EXPORT_SYMBOL(clk_round_rate);

static inline int close_enough(long rate1, long rate2)
{
	return rate1 && !((rate2 - rate1) * 1000 / rate1);
}

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EINVAL;

	pr_debug("%s: clock %s rate %ld:%ld\n", __func__, clk->name,
			(unsigned long)clk->rate, rate);
	if (clk->flags & FIXED_RATE || !clk->set_rate)
		goto out;
	else if (!close_enough(clk->rate, rate)) {
		ret = clk->set_rate(clk, rate);
		if (ret < 0) {
			printk(KERN_ERR "couldn't set rate for clock '%s': "
					"error %d\n", clk->name, ret);
			goto out;
		}
		clk->rate = rate;
		if (clk->flags & RATE_PROPAGATES)
			clk->propagate_rate(clk);
	} else
		ret = 0;

out:
	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk **p;
	struct clk *clk = ERR_PTR(-ENOENT);
	int idno;
	unsigned long clocks_flags;

	if (dev == NULL || dev->bus != &platform_bus_type)
		idno = -1;
	else
		idno = to_platform_device(dev)->id;

	spin_lock_irqsave(&clocks_lock, clocks_flags);
	for (p = onchip_clks; p < onchip_clks + ARRAY_SIZE(onchip_clks); p++) {
		if (!((*p)->flags & PRESENT_ON_STMP37XX &&
				cpu_is_stmp37xx()) &&
			!((*p)->flags & PRESENT_ON_STMP378X &&
				cpu_is_stmp378x()))
			continue;

		if (strcmp(id, (*p)->name) == 0 &&
			   try_module_get((*p)->owner)) {
			clk = *p;
			break;
		}
	}
	spin_unlock_irqrestore(&clocks_lock, clocks_flags);
	return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
	if (clk->owner)
		module_put(clk->owner);
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	unsigned long clocks_flags;

	if (IS_ERR(clk) || clk == NULL)
		return -EINVAL;
	pr_debug("%s: clock '%s'\n", __func__, clk->name);

	if (clk->parent)
		clk_enable(clk->parent);

	spin_lock_irqsave(&clocks_lock, clocks_flags);

	clk->usage++;
	pr_debug("clock '%s': use count increased to %d\n",
		clk->name, clk->usage);
	if (clk->enable)
	       (clk->enable)(clk);

	spin_unlock_irqrestore(&clocks_lock, clocks_flags);
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void local_clk_disable(struct clk *clk)
{
	unsigned long clocks_flags;

	if (IS_ERR(clk) || clk == NULL)
		return;

	spin_lock_irqsave(&clocks_lock, clocks_flags);

	if (clk->usage == 0 && clk->disable) {
		pr_debug("%s: disabling clock '%s'\n", __func__, clk->name);
		(clk->disable)(clk);
	}

	spin_unlock_irqrestore(&clocks_lock, clocks_flags);
}

void clk_disable(struct clk *clk)
{
	unsigned long clocks_flags;

	if (IS_ERR(clk) || clk == NULL)
		return;
	pr_debug("%s: clock '%s'\n", __func__, clk->name);

	spin_lock_irqsave(&clocks_lock, clocks_flags);

	if ((--clk->usage) == 0 && clk->disable)
		(clk->disable)(clk);
	pr_debug("clock '%s': use count decreased to %d\n",
		clk->name, clk->usage);

	spin_unlock_irqrestore(&clocks_lock, clocks_flags);
	if (clk->parent)
		clk_disable(clk->parent);
}
EXPORT_SYMBOL(clk_disable);

/* Some additional API */
int clk_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -ENODEV;
	unsigned long clocks_flags;

	if (!clk->set_parent)
		goto out;

	spin_lock_irqsave(&clocks_lock, clocks_flags);

	if (clk->set_parent)
		ret = clk->set_parent(clk, parent);
	if (!ret) {
		pr_debug("%s: '%s': parent usage %d, clk->parent usage %d, "
			"clk usage %d\n", __func__, clk->name,
			parent->usage, clk->parent->usage, clk->usage);
		if (parent->usage == 0 && parent->disable)
			(parent->disable)(parent);
		parent->usage += clk->usage;
		clk->parent->usage -= clk->usage;
		if (clk->parent->usage == 0 && clk->parent->disable)
			(clk->parent->disable)(clk->parent);
		clk->parent = parent;
	}
	spin_unlock_irqrestore(&clocks_lock, clocks_flags);

out:
	return ret;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	return clk->parent;
}
EXPORT_SYMBOL(clk_get_parent);

static void clkctrl_enable_powersavings(void)
{
	HW_CLKCTRL_HBUS_SET(BM_CLKCTRL_HBUS_APBHDMA_AS_ENABLE);
	HW_CLKCTRL_HBUS_SET(BM_CLKCTRL_HBUS_APBXDMA_AS_ENABLE);
	HW_CLKCTRL_HBUS_SET(BM_CLKCTRL_HBUS_TRAFFIC_AS_ENABLE);
	HW_CLKCTRL_HBUS_SET(BM_CLKCTRL_HBUS_TRAFFIC_JAM_AS_ENABLE);
	HW_CLKCTRL_HBUS_SET(BM_CLKCTRL_HBUS_CPU_DATA_AS_ENABLE);
	HW_CLKCTRL_HBUS_SET(BM_CLKCTRL_HBUS_CPU_INSTR_AS_ENABLE);
	HW_CLKCTRL_HBUS_SET(BM_CLKCTRL_HBUS_DCP_AS_ENABLE);
	HW_CLKCTRL_HBUS_SET(BM_CLKCTRL_HBUS_PXP_AS_ENABLE);

	HW_CLKCTRL_HBUS_SET(BF_CLKCTRL_HBUS_SLOW_DIV(
				BV_CLKCTRL_HBUS_SLOW_DIV__BY32));
	HW_CLKCTRL_HBUS_SET(BM_CLKCTRL_HBUS_AUTO_SLOW_MODE);
}

static int __init clk_init(void)
{
	struct clk **clkp;

	spin_lock_init(&clocks_lock);
	pr_debug("%s!\n", __func__);
	for (clkp = onchip_clks; clkp < onchip_clks + ARRAY_SIZE(onchip_clks);
	     clkp++) {
		if (!((*clkp)->flags & PRESENT_ON_STMP37XX &&
				cpu_is_stmp37xx()) &&
			!((*clkp)->flags & PRESENT_ON_STMP378X &&
				cpu_is_stmp378x()))
			continue;

		(*clkp)->owner = THIS_MODULE;
		if ((*clkp)->flags & ENABLED)
			clk_enable(*clkp);
		else
			local_clk_disable(*clkp);
		if (((*clkp)->flags & NEEDS_INITIALIZATION) &&
				(*clkp)->set_rate)
			(*clkp)->set_rate((*clkp), (*clkp)->rate);
		if (!((*clkp)->flags & FIXED_RATE) && ((*clkp)->get_rate))
			(*clkp)->get_rate(*clkp);
		if (((*clkp)->flags & FIXED_RATE) &&
				((*clkp)->flags & RATE_PROPAGATES))
			(*clkp)->propagate_rate(*clkp);
		if ((*clkp)->flags & NEEDS_SET_PARENT)
			(*clkp)->set_parent(*clkp, (*clkp)->parent);
		pr_debug("%s: clock %s, rate %d\n",
			__func__, (*clkp)->name, (*clkp)->rate);
	}
	clkctrl_enable_powersavings();

	return 0;
}

arch_initcall(clk_init);
