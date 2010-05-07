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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iram_alloc.h>
#include <linux/platform_device.h>

#include <mach/clock.h>

#include "regs-clkctrl.h"
#include "regs-digctl.h"

#include <mach/regs-rtc.h>
#include <mach/mx23.h>

#define CLKCTRL_BASE_ADDR IO_ADDRESS(CLKCTRL_PHYS_ADDR)
#define DIGCTRL_BASE_ADDR IO_ADDRESS(DIGCTL_PHYS_ADDR)
#define RTC_BASE_ADDR IO_ADDRESS(RTC_PHYS_ADDR)

/* these are the maximum clock speeds that have been
 * validated to run at the minumum VddD target voltage level for cpu operation
 * (presently 1.05V target, .975V Brownout).  Higher clock speeds for GPMI and
 * SSP have not been validated.
 */
#define PLL_ENABLED_MAX_CLK_SSP 96000000
#define PLL_ENABLED_MAX_CLK_GPMI 96000000


/* external clock input */
static struct clk pll_clk;
static struct clk ref_xtal_clk;

#ifdef DEBUG
static void print_ref_counts(void);
#endif

static unsigned long enet_mii_phy_rate;

static inline int clk_is_busy(struct clk *clk)
{
	if ((clk->parent == &ref_xtal_clk) && (clk->xtal_busy_bits))
		return __raw_readl(clk->busy_reg) & (1 << clk->xtal_busy_bits);
	else if (clk->busy_bits && clk->busy_reg)
		return __raw_readl(clk->busy_reg) & (1 << clk->busy_bits);
	else {
		printk(KERN_ERR "WARNING: clock has no assigned busy \
			register or bits\n");
		udelay(10);
		return 0;
	}
}

static inline int clk_busy_wait(struct clk *clk)
{
	int i;

	for (i = 10000000; i; i--)
		if (!clk_is_busy(clk))
			break;
	if (!i)
		return -ETIMEDOUT;
	else
		return 0;
}

static bool mx23_enable_h_autoslow(bool enable)
{
	bool currently_enabled;

	if (__raw_readl(CLKCTRL_BASE_ADDR+HW_CLKCTRL_HBUS) &
		BM_CLKCTRL_HBUS_AUTO_SLOW_MODE)
		currently_enabled = true;
	else
		currently_enabled = false;

	if (enable)
		__raw_writel(BM_CLKCTRL_HBUS_AUTO_SLOW_MODE,
			CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS_SET);
	else
		__raw_writel(BM_CLKCTRL_HBUS_AUTO_SLOW_MODE,
			CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS_CLR);
	return currently_enabled;
}


static void mx23_set_hbus_autoslow_flags(u16 mask)
{
	u32 reg;

	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);
	reg &= 0xFFFF;
	reg |= mask << 16;
	__raw_writel(reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);
}

static void local_clk_disable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk) || !clk->ref)
		return;

	if ((--clk->ref) & CLK_EN_MASK)
		return;

	if (clk->disable)
		clk->disable(clk);
	local_clk_disable(clk->secondary);
	local_clk_disable(clk->parent);
}

static int local_clk_enable(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	if ((clk->ref++) & CLK_EN_MASK)
		return 0;
	if (clk->parent)
		local_clk_enable(clk->parent);
	if (clk->secondary)
		local_clk_enable(clk->secondary);
	if (clk->enable)
		clk->enable(clk);
	return 0;
}


static bool mx23_is_clk_enabled(struct clk *clk)
{
	if (clk->enable_reg)
		return (__raw_readl(clk->enable_reg) &
				clk->enable_bits) ? 0 : 1;
	else
		return (clk->ref & CLK_EN_MASK) ? 1 : 0;
}


static int mx23_raw_enable(struct clk *clk)
{
	unsigned int reg;
	if (clk->enable_reg) {
		reg = __raw_readl(clk->enable_reg);
		reg &= ~clk->enable_bits;
		__raw_writel(reg, clk->enable_reg);
	}
	if (clk->busy_reg)
		clk_busy_wait(clk);

	return 0;
}

static void mx23_raw_disable(struct clk *clk)
{
	unsigned int reg;
	if (clk->enable_reg) {
		reg = __raw_readl(clk->enable_reg);
		reg |= clk->enable_bits;
		__raw_writel(reg, clk->enable_reg);
	}
}

static unsigned long ref_xtal_get_rate(struct clk *clk)
{
	return 24000000;
}

static struct clk ref_xtal_clk = {
	.flags = RATE_FIXED,
	.get_rate = ref_xtal_get_rate,
};

static unsigned long pll_get_rate(struct clk *clk);
static int pll_enable(struct clk *clk);
static void pll_disable(struct clk *clk);

static struct clk pll_clk = {

	 .parent = &ref_xtal_clk,
	 .flags = RATE_FIXED,
	 .get_rate = pll_get_rate,
	 .enable = pll_enable,
	 .disable = pll_disable,

};

static unsigned long pll_get_rate(struct clk *clk)
{
	return 480000000;
}

static int pll_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLLCTRL0);

	if ((reg & BM_CLKCTRL_PLLCTRL0_POWER) &&
		(reg & BM_CLKCTRL_PLLCTRL0_EN_USB_CLKS))
		return 0;

	__raw_writel(BM_CLKCTRL_PLLCTRL0_POWER |
			     BM_CLKCTRL_PLLCTRL0_EN_USB_CLKS,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLLCTRL0_SET);
	/* only a 10us delay is need.  PLLCTRL1 LOCK bitfied is only a timer
	 * and is incorrect (excessive).  Per definition of the PLLCTRL0
	 * POWER field, waiting at least 10us.
	 */
	udelay(10);

	return 0;
}

static void pll_disable(struct clk *clk)
{
	__raw_writel(BM_CLKCTRL_PLLCTRL0_POWER |
			     BM_CLKCTRL_PLLCTRL0_EN_USB_CLKS,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLLCTRL0_CLR);
	return;
}

static inline unsigned long
ref_clk_get_rate(unsigned long base, unsigned int div)
{
	unsigned long rate = base / 1000;
	return 1000 * ((rate * 18) / div);
}

static unsigned long ref_clk_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long base = clk->parent->get_rate(clk->parent);
	unsigned long div = (base  * 18) / rate;
	return (base / div) * 18;
}

static int ref_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long base = clk->parent->get_rate(clk->parent);
	unsigned long div = ((base/1000)  * 18) / (rate/1000);
	if (rate != ((base / div) * 18))
		return -EINVAL;
	if (clk->scale_reg == 0)
		return -EINVAL;
	base = __raw_readl(clk->scale_reg);
	base &= ~(0x3F << clk->scale_bits);
	base |= (div << clk->scale_bits);
	__raw_writel(base, clk->scale_reg);
	return 0;
}

static unsigned long ref_cpu_get_rate(struct clk *clk)
{
	unsigned int reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC) &
	    BM_CLKCTRL_FRAC_CPUFRAC;
	return ref_clk_get_rate(clk->parent->get_rate(clk->parent), reg);
}

static struct clk ref_cpu_clk = {
	.parent = &pll_clk,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.get_rate = ref_cpu_get_rate,
	.round_rate = ref_clk_round_rate,
	.set_rate = ref_clk_set_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.enable_bits = BM_CLKCTRL_FRAC_CLKGATECPU,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.scale_bits = BP_CLKCTRL_FRAC_CPUFRAC,
	.busy_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_CPU,
	.busy_bits	= 28,
};

static unsigned long ref_emi_get_rate(struct clk *clk)
{
	unsigned int reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC) &
	    BM_CLKCTRL_FRAC_EMIFRAC;
	reg >>= BP_CLKCTRL_FRAC_EMIFRAC;
	return ref_clk_get_rate(clk->parent->get_rate(clk->parent), reg);
}

static struct clk ref_emi_clk = {
	.parent = &pll_clk,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.get_rate = ref_emi_get_rate,
	.set_rate = ref_clk_set_rate,
	.round_rate = ref_clk_round_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.enable_bits = BM_CLKCTRL_FRAC_CLKGATEEMI,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.scale_bits = BP_CLKCTRL_FRAC_EMIFRAC,
};

static unsigned long ref_io_get_rate(struct clk *clk);
static struct clk ref_io_clk = {
	.parent = &pll_clk,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.get_rate = ref_io_get_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.enable_bits = BM_CLKCTRL_FRAC_CLKGATEIO,
};

static unsigned long ref_io_get_rate(struct clk *clk)
{
	unsigned int reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC) &
			BM_CLKCTRL_FRAC_IOFRAC;
	reg >>= BP_CLKCTRL_FRAC_IOFRAC;

	return ref_clk_get_rate(clk->parent->get_rate(clk->parent), reg);
}

static unsigned long ref_pix_get_rate(struct clk *clk)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC) &
	    BM_CLKCTRL_FRAC_PIXFRAC;
	reg >>= BP_CLKCTRL_FRAC_PIXFRAC;
	return ref_clk_get_rate(clk->parent->get_rate(clk->parent), reg);
}

static struct clk ref_pix_clk = {
	.parent = &pll_clk,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.get_rate = ref_pix_get_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.enable_bits = BM_CLKCTRL_FRAC_CLKGATEPIX,
};

static struct clk cpu_clk, h_clk;
static int clkseq_set_parent(struct clk *clk, struct clk *parent)
{
	int shift;

	if (clk->parent == parent)
		return 0;  /* clock parent already at target.  nothing to do */
	/* bypass? */
	if (parent == &ref_xtal_clk)
		shift = 4;
	else
		shift = 8;

	if (clk->bypass_reg)
		__raw_writel(1 << clk->bypass_bits, clk->bypass_reg + shift);

	return 0;
}

static unsigned long lcdif_get_rate(struct clk *clk)
{
	long rate = clk->parent->get_rate(clk->parent);
	long div;
	const int mask = 0xff;

	div = (__raw_readl(clk->scale_reg) >> clk->scale_bits) & mask;
	if (div) {
		rate /= div;
		div = (__raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC) &
			BM_CLKCTRL_FRAC_PIXFRAC) >> BP_CLKCTRL_FRAC_PIXFRAC;
		rate /= div;
	}

	return rate;
}

static int lcdif_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = 0;
	/*
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
	u32 ns_cycle = 1000000000 / rate;
	u32 div, reg_val;
	u32 lowest_result = (u32) -1;
	u32 lowest_div = 0, lowest_fracdiv = 0;

	ns_cycle *= 2; /* Fix calculate double frequency */



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
		fracdiv = ((clk->parent->get_rate(clk->parent) / 1000000 * 18 / 2) *
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
	reg_val = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC);
	reg_val &= ~BM_CLKCTRL_FRAC_PIXFRAC;
	reg_val |= BF_CLKCTRL_FRAC_PIXFRAC(lowest_fracdiv);
	__raw_writel(reg_val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC);

	/* Ungate PFD */
	__raw_writel(BM_CLKCTRL_FRAC_CLKGATEPIX,
			CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC_CLR);

	/* Program pix divider */
	reg_val = __raw_readl(clk->scale_reg);
	reg_val &= ~(BM_CLKCTRL_PIX_DIV | BM_CLKCTRL_PIX_CLKGATE);
	reg_val |= BF_CLKCTRL_PIX_DIV(lowest_div);
	__raw_writel(reg_val, clk->scale_reg);

	/* Wait for divider update */
	ret = clk_busy_wait(clk);
	if (ret)
		goto out;

	/* Switch to ref_pix source */
	reg_val = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);
	reg_val &= ~BM_CLKCTRL_CLKSEQ_BYPASS_PIX;
	__raw_writel(reg_val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);

out:
	return ret;
}

/*
 * We set lcdif_clk's parent as &pll_clk deliberately, although
 * in IC spec lcdif_clk(CLK_PIX) is derived from ref_pix which in turn
 * is derived from PLL. By doing so, users just need to set/get clock rate
 * for lcdif_clk, without need to take care of ref_pix, because the clock
 * driver will automatically calculate the fracdivider for HW_CLKCTRL_FRAC
 * and the divider for HW_CLKCTRL_PIX conjointly.
 */
static struct clk lcdif_clk = {
	.parent		= &pll_clk,
	.enable 	= mx23_raw_enable,
	.disable 	= mx23_raw_disable,
	.scale_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_PIX,
	.busy_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_PIX,
	.busy_bits	= 29,
	.enable_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_PIX,
	.enable_bits	= 31,
	.bypass_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	.bypass_bits	= 1,
	.get_rate	= lcdif_get_rate,
	.set_rate	= lcdif_set_rate,
	.set_parent = clkseq_set_parent,
	.flags		= CPU_FREQ_TRIG_UPDATE,
};

static unsigned long cpu_get_rate(struct clk *clk)
{
	unsigned long rate, div;
	rate = (clk->parent->get_rate(clk->parent));
	div = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_CPU) &
			  BM_CLKCTRL_CPU_DIV_CPU;
	rate = rate/div;
	return rate;
}

static unsigned long cpu_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long frac_rate, root_rate = clk->parent->get_rate(clk->parent);
	unsigned int div = root_rate / rate;
	if (div == 0)
		return root_rate;
	if (clk->parent == &ref_cpu_clk) {
		if (div > 0x3F)
			div = 0x3F;
		return root_rate / div;
	}

	frac_rate = root_rate % rate;
	div = root_rate / rate;
	if ((div == 0) || (div >= 0x400))
		return root_rate;
	if (frac_rate == 0)
		return rate;
	return rate;
}

static int cpu_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long root_rate = pll_clk.get_rate(&pll_clk);
	int ret = -EINVAL;
	u32 clkctrl_cpu = 1;
	u32 c = clkctrl_cpu;
	u32 clkctrl_frac = 1;
	u32 val;
	u32 reg_val, hclk_reg;
	bool h_autoslow;

	/* make sure the cpu div_xtal is 1 */
	reg_val = __raw_readl(CLKCTRL_BASE_ADDR+HW_CLKCTRL_CPU);
	reg_val &= ~(BM_CLKCTRL_CPU_DIV_XTAL);
	reg_val |= (1 << BP_CLKCTRL_CPU_DIV_XTAL);
	__raw_writel(reg_val, CLKCTRL_BASE_ADDR+HW_CLKCTRL_CPU);

	if (rate < ref_xtal_get_rate(&ref_xtal_clk))
		return -EINVAL;

	if (rate == clk_get_rate(clk))
		return 0;
	/* temporaily disable h autoslow to avoid
	 * hclk getting too slow while temporarily
	 * changing clocks
	 */
	h_autoslow = mx23_enable_h_autoslow(false);

	if (rate == ref_xtal_get_rate(&ref_xtal_clk)) {

		/* switch to the 24M source */
		clk_set_parent(clk, &ref_xtal_clk);

		/* to avoid bus starvation issues, we'll go ahead
		 * and change hbus clock divider to 1 now.  Cpufreq
		 * or other clock management can lower it later if
		 * desired for power savings or other reasons, but
		 * there should be no need to with hbus autoslow
		 * functionality enabled.
		 */

		ret = clk_busy_wait(&cpu_clk);
		if (ret) {
			printk(KERN_ERR "* couldn't set\
				up CPU divisor\n");
			return ret;
		}

		ret = clk_busy_wait(&h_clk);
		if (ret) {
			printk(KERN_ERR "* H_CLK busy timeout\n");
			return ret;
		}

		hclk_reg = __raw_readl(CLKCTRL_BASE_ADDR+HW_CLKCTRL_HBUS);
		hclk_reg &= ~(BM_CLKCTRL_HBUS_DIV);
		hclk_reg |= (1 << BP_CLKCTRL_HBUS_DIV);

		__raw_writel(hclk_reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS_SET);

		ret = clk_busy_wait(&cpu_clk);
		if (ret) {
			printk(KERN_ERR "** couldn't set\
				up CPU divisor\n");
			return ret;
		}

		ret = clk_busy_wait(&h_clk);
		if (ret) {
			printk(KERN_ERR "** CLK busy timeout\n");
			return ret;
		}

	} else {
		for ( ; c < 0x40; c++) {
			u32 f = ((root_rate/1000)*18/c + (rate/1000)/2) /
				(rate/1000);
			int s1, s2;

			if (f < 18 || f > 35)
				continue;
			s1 = (root_rate/1000)*18/clkctrl_frac/clkctrl_cpu -
			     (rate/1000);
			s2 = (root_rate/1000)*18/c/f - (rate/1000);
			if (abs(s1) > abs(s2)) {
				clkctrl_cpu = c;
				clkctrl_frac = f;
			}
			if (s2 == 0)
				break;
		};
		if (c == 0x40) {
			int  d = (root_rate/1000)*18/clkctrl_frac/clkctrl_cpu -
				(rate/1000);
			if ((abs(d) > 100) || (clkctrl_frac < 18) ||
				(clkctrl_frac > 35))
				return -EINVAL;
		}

		/* prepare Frac div */
		val = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC);
		val &= ~(BM_CLKCTRL_FRAC_CPUFRAC);
		val |= (clkctrl_frac << BP_CLKCTRL_FRAC_CPUFRAC);

		/* prepare clkctrl_cpu div*/
		reg_val = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_CPU);
		reg_val &= ~0x3F;
		reg_val |= clkctrl_cpu;

		/* set safe hbus clock divider. A divider of 3 ensure that
		 * the Vddd voltage required for the cpuclk is sufficiently
		 * high for the hbus clock and under 24MHz cpuclk conditions,
		 * a divider of at least 3 ensures hbusclk doesn't remain
		 * uneccesarily low which hurts performance
		 */
		hclk_reg = __raw_readl(CLKCTRL_BASE_ADDR+HW_CLKCTRL_HBUS);
		hclk_reg &= ~(BM_CLKCTRL_HBUS_DIV);
		hclk_reg |= (3 << BP_CLKCTRL_HBUS_DIV);

		/* if the pll was OFF, we need to turn it ON.
		 * Even if it was ON, we want to temporarily
		 * increment it by 1 to avoid turning off
		 * in the upcoming parent clock change to xtal.  This
		 * avoids waiting an extra 10us for every cpu clock
		 * change between ref_cpu sourced frequencies.
		 */
		pll_enable(&pll_clk);
		pll_clk.ref++;

		/* switch to XTAL CLK source temparily while
		 * we manipulate ref_cpu frequency */
		clk_set_parent(clk, &ref_xtal_clk);

		ret = clk_busy_wait(&h_clk);

		if (ret) {
			printk(KERN_ERR "-* HCLK busy wait timeout\n");
			return ret;
		}

		ret = clk_busy_wait(clk);

		if (ret) {
			printk(KERN_ERR "-* couldn't set\
				up CPU divisor\n");
			return ret;
		}

		__raw_writel(val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC);

		/* clear the gate */
		__raw_writel(BM_CLKCTRL_FRAC_CLKGATECPU, CLKCTRL_BASE_ADDR +
			     HW_CLKCTRL_FRAC_CLR);

		/* set the ref_cpu integer divider */
		__raw_writel(reg_val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_CPU);

		/* wait for the ref_cpu path to become stable before
		 * switching over to it
		 */

		ret = clk_busy_wait(&ref_cpu_clk);

		if (ret) {
			printk(KERN_ERR "-** couldn't set\
				up CPU divisor\n");
			return ret;
		}

		/* change hclk divider to safe value for any ref_cpu
		 * value.
		 */
		__raw_writel(hclk_reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);

		ret = clk_busy_wait(&h_clk);

		if (ret) {
			printk(KERN_ERR "-** HCLK busy wait timeout\n");
			return ret;
		}

		clk_set_parent(clk, &ref_cpu_clk);

		/* decrement the pll_clk ref count because
		 * we temporarily enabled/incremented the count
		 * above.
		 */
		pll_clk.ref--;

		ret = clk_busy_wait(&cpu_clk);

		if (ret) {
			printk(KERN_ERR "-*** Couldn't set\
				up CPU divisor\n");
			return ret;
		}

		ret = clk_busy_wait(&h_clk);

		if (ret) {
			printk(KERN_ERR "-*** HCLK busy wait timeout\n");
			return ret;
		}

	}
	mx23_enable_h_autoslow(h_autoslow);
	return ret;
}

static struct clk cpu_clk = {
	.parent = &ref_cpu_clk,
	.get_rate = cpu_get_rate,
	.round_rate = cpu_round_rate,
	.set_rate = cpu_set_rate,
	.set_parent = clkseq_set_parent,
	.scale_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.scale_bits	= 0,
	.bypass_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	.bypass_bits	= 7,
	.busy_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_CPU,
	.busy_bits	= 28,
	.xtal_busy_bits = 29,
};

static unsigned long uart_get_rate(struct clk *clk)
{
	unsigned int div;
	div = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_XTAL) &
	    BM_CLKCTRL_XTAL_DIV_UART;
	return clk->parent->get_rate(clk->parent) / div;
}

static struct clk uart_clk = {
	.parent = &ref_xtal_clk,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_XTAL,
	.enable_bits = BM_CLKCTRL_XTAL_UART_CLK_GATE,
	.get_rate = uart_get_rate,
};

static struct clk pwm_clk = {
	.parent = &ref_xtal_clk,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_XTAL,
	.enable_bits = BM_CLKCTRL_XTAL_PWM_CLK24M_GATE,
};

static unsigned long clk_32k_get_rate(struct clk *clk)
{
	return clk->parent->get_rate(clk->parent) / 750;
}

static struct clk clk_32k = {
	.parent = &ref_xtal_clk,
	.flags = RATE_FIXED,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_XTAL,
	.enable_bits = BM_CLKCTRL_XTAL_TIMROT_CLK32K_GATE,
	.get_rate = clk_32k_get_rate,
};

static unsigned long lradc_get_rate(struct clk *clk)
{
	return clk->parent->get_rate(clk->parent) / 16;
}

static struct clk lradc_clk = {
	.parent = &clk_32k,
	.flags = RATE_FIXED,
	.get_rate = lradc_get_rate,
};

static unsigned long x_get_rate(struct clk *clk)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_XBUS) &
	    BM_CLKCTRL_XBUS_DIV;
	return clk->parent->get_rate(clk->parent) / reg;
}

static unsigned long x_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned int root_rate, frac_rate;
	unsigned int div;
	root_rate = clk->parent->get_rate(clk->parent);
	frac_rate = root_rate % rate;
	div = root_rate / rate;
	/* while the reference manual specifies that divider
	 * values up to 1023 are aloud, other critial SoC compents
	 * require higher x clock values at all times.  Through
	 * limited testing, the lradc functionality to measure
	 * the battery voltage and copy this value to the
	 * power supply requires at least a 64kHz xclk.
	 * so the divider will be limited to 375.
	 */
	if ((div == 0) || (div > 375))
		return root_rate;
	if (frac_rate == 0)
		return rate;
	else
		return root_rate / (div + 1);
}

static int x_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long root_rate;
	unsigned long round_rate;
	unsigned int reg, div;
	root_rate = clk->parent->get_rate(clk->parent);

	if ((!clk->round_rate) || !(clk->scale_reg))
		return -EINVAL;

	round_rate =  clk->round_rate(clk, rate);
	div = root_rate / round_rate;

	if (root_rate % round_rate)
			return -EINVAL;

	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_XBUS);
	reg &= ~(BM_CLKCTRL_XBUS_DIV_FRAC_EN | BM_CLKCTRL_XBUS_DIV);
	reg |= BF_CLKCTRL_XBUS_DIV(div);
	__raw_writel(reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_XBUS);

	return clk_busy_wait(clk);

}

static struct clk x_clk = {
	.parent = &ref_xtal_clk,
	.get_rate = x_get_rate,
	.set_rate = x_set_rate,
	.round_rate = x_round_rate,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_XBUS,
	.busy_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_XBUS,
	.busy_bits	= 31,
};



static struct clk ana_clk = {
	.parent = &ref_xtal_clk,
};



static unsigned long xtal_clock32k_get_rate(struct clk *clk)
{
	if (__raw_readl(RTC_BASE_ADDR + HW_RTC_PERSISTENT0) &
		BM_RTC_PERSISTENT0_XTAL32_FREQ)
		return 32000;
	else
		return 32768;
}

static struct clk xtal_clock32k_clk = {
	.get_rate = xtal_clock32k_get_rate,
};

static unsigned long rtc32k_get_rate(struct clk *clk)
{
	if (clk->parent == &ref_xtal_clk)
		/* mx23 reference manual had error.
		 * fixed divider is 750 not 768
		 */
		return clk->parent->get_rate(clk->parent) / 750;
	else
		return xtal_clock32k_get_rate(clk);
}

static struct clk rtc32k_clk = {
	.parent = &xtal_clock32k_clk,
	.get_rate = rtc32k_get_rate,
};

static unsigned long h_get_rate(struct clk *clk)
{
	unsigned long reg, div;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);
	div = reg & BM_CLKCTRL_HBUS_DIV;
		return clk->parent->get_rate(clk->parent) / div;
}

static unsigned long h_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned int root_rate, frac_rate;
	unsigned int div;
	root_rate = clk->parent->get_rate(clk->parent);
	frac_rate = root_rate % rate;
	div = root_rate / rate;
	if ((div == 0) || (div >= 0x20))
		return root_rate;
	if (frac_rate == 0)
		return rate;
	else
		return root_rate / (div + 1);
}

static int h_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long root_rate;
	unsigned long round_rate;
	unsigned int reg, div;
	root_rate = clk->parent->get_rate(clk->parent);
	round_rate =  h_round_rate(clk, rate);
	div = root_rate / round_rate;
	if ((div == 0) || (div >= 0x20))
		return -EINVAL;

	if (root_rate % round_rate)
			return -EINVAL;

	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);
	reg &= ~(BM_CLKCTRL_HBUS_DIV_FRAC_EN | BM_CLKCTRL_HBUS_DIV);
	reg |= BF_CLKCTRL_HBUS_DIV(div);
	__raw_writel(reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);

	if (clk_busy_wait(clk)) {
		printk(KERN_ERR "couldn't set up AHB divisor\n");
		return -EINVAL;
	}

	return 0;
}

static struct clk h_clk = {
	.parent = &cpu_clk,
	.get_rate = h_get_rate,
	.set_rate = h_set_rate,
	.round_rate = h_round_rate,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS,
	.busy_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS,
	.busy_bits	= 31,
};

static struct clk ocrom_clk = {
	.parent = &h_clk,
};

static unsigned long emi_get_rate(struct clk *clk)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_EMI);
	if (clk->parent == &ref_emi_clk)
		reg = (reg & BM_CLKCTRL_EMI_DIV_EMI);
	else
		reg = (reg & BM_CLKCTRL_EMI_DIV_XTAL) >>
		    BP_CLKCTRL_EMI_DIV_XTAL;
	return clk->parent->get_rate(clk->parent) / reg;
}

static unsigned long emi_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long root_rate = clk->parent->get_rate(clk->parent);
	unsigned int div = root_rate / rate;
	if (div == 0)
		return root_rate;
	if (clk->parent == &ref_emi_clk) {
		if (div > 0x3F)
			div = 0x3F;
		return root_rate / div;
	}
	if (div > 0xF)
		div = 0xF;
	return root_rate / div;
}

/* when changing the emi clock, dram access must be
 * disabled.  Special handling is needed to perform
 * the emi clock change without touching sdram.
 */
static int emi_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = 0;

	struct mxs_emi_scaling_data sc_data;

	unsigned long clkctrl_emi;
	unsigned long clkctrl_frac;
	int div = 1;
	unsigned long root_rate, cur_emi_div, cur_emi_frac;
	struct clk *target_parent_p = &ref_xtal_clk;

	if (rate < ref_xtal_get_rate(&ref_xtal_clk))
		return -EINVAL;

	if (!mxs_ram_funcs_sz)
		goto out;

	sc_data.cur_freq = (clk->get_rate(clk)) / 1000 / 1000;
	sc_data.new_freq = rate / 1000 / 1000;

	if (sc_data.cur_freq == sc_data.new_freq)
		goto out;

	if (rate != ref_xtal_get_rate(&ref_xtal_clk)) {
		target_parent_p = &ref_emi_clk;
		pll_enable(&pll_clk);

		root_rate = pll_clk.get_rate(&pll_clk);

		for (clkctrl_emi = div; clkctrl_emi < 0x3f;
					clkctrl_emi += div) {
			clkctrl_frac = ((root_rate / 1000) * 18 +
					(rate / 1000) * clkctrl_emi / 2) /
					((rate / 1000) * clkctrl_emi);
			if (clkctrl_frac >= 18 && clkctrl_frac <= 35) {
				pr_debug("%s: clkctrl_frac found %ld for %ld\n",
					__func__, clkctrl_frac, clkctrl_emi);
				if (((root_rate / 1000) * 18 /
					clkctrl_frac / clkctrl_emi) / 1000 ==
					rate / 1000 / 1000)
					break;
			}
		}

		if (clkctrl_emi >= 0x3f)
			return -EINVAL;
		pr_debug("%s: clkctrl_emi %ld, clkctrl_frac %ld\n",
			__func__, clkctrl_emi, clkctrl_frac);

		sc_data.emi_div = clkctrl_emi;
		sc_data.frac_div = clkctrl_frac;
	}


	cur_emi_div = ((__raw_readl(CLKCTRL_BASE_ADDR+HW_CLKCTRL_EMI) &
		BM_CLKCTRL_EMI_DIV_EMI) >> BP_CLKCTRL_EMI_DIV_EMI);
	cur_emi_frac = ((__raw_readl(CLKCTRL_BASE_ADDR+HW_CLKCTRL_FRAC) &
		BM_CLKCTRL_EMI_DIV_EMI) >> BP_CLKCTRL_FRAC_EMIFRAC);

	if ((cur_emi_div == sc_data.emi_div) &&
		(cur_emi_frac == sc_data.frac_div))
		goto out;
	{
		unsigned long iram_phy;
		bool h_autoslow;
		int (*scale)(struct mxs_emi_scaling_data *) =
			iram_alloc(mxs_ram_funcs_sz, &iram_phy);

		if (NULL == scale) {
			pr_err("%s Not enough iram\n", __func__);
			return -ENOMEM;
		}

		/* temporaily disable h autoslow to maximize
		 * performance/minimize time spent with no
		 * sdram access
		 */
		h_autoslow = mx23_enable_h_autoslow(false);

		memcpy(scale, mxs_ram_freq_scale, mxs_ram_funcs_sz);

		local_irq_disable();
		local_fiq_disable();

		scale(&sc_data);

		iram_free(iram_phy, mxs_ram_funcs_sz);

		local_fiq_enable();
		local_irq_enable();

		/* temporaily disable h autoslow to avoid
		 * hclk getting too slow while temporarily
		 * changing clocks
		 */
		mx23_enable_h_autoslow(h_autoslow);
	}

	/* this code is for keeping track of ref counts.
	 * and disabling previous parent if necessary
	 * actual clkseq changes have already
	 * been made.
	 */
	clk_set_parent(clk, target_parent_p);

out:
	return ret;
}

static struct clk emi_clk = {
	.parent = &ref_emi_clk,
	.get_rate = emi_get_rate,
	.set_rate = emi_set_rate,
	.round_rate = emi_round_rate,
	.set_parent = clkseq_set_parent,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_EMI,
	.enable_bits = BM_CLKCTRL_EMI_CLKGATE,
	.scale_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.busy_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_EMI,
	.busy_bits	= 28,
	.xtal_busy_bits = 29,
	.bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	.bypass_bits = 6,
};

static unsigned long ssp_get_rate(struct clk *clk);

static int ssp_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EINVAL;
	u32 reg, div;
	bool is_clk_enable;

	is_clk_enable = mx23_is_clk_enabled(clk);
	if (!is_clk_enable)
		local_clk_enable(clk);

	/* if the desired clock can be sourced from ref_xtal,
	 * use ref_xtal to save power
	 */
	if ((rate <= ref_xtal_get_rate(&ref_xtal_clk)) &&
		((ref_xtal_get_rate(&ref_xtal_clk) % rate) == 0))
		clk_set_parent(clk, &ref_xtal_clk);
	else
		clk_set_parent(clk, &ref_io_clk);

	if (rate > PLL_ENABLED_MAX_CLK_SSP)
		rate = PLL_ENABLED_MAX_CLK_SSP;

	div = (clk_get_rate(clk->parent) + rate - 1) / rate;

	if (div == 0 || div > BM_CLKCTRL_SSP_DIV)
		goto out;

	reg = __raw_readl(clk->scale_reg);
	reg &= ~(BM_CLKCTRL_SSP_DIV | BM_CLKCTRL_SSP_DIV_FRAC_EN);
	reg |= div << clk->scale_bits;
	__raw_writel(reg, clk->scale_reg);

	ret = clk_busy_wait(clk);
out:
	if (!is_clk_enable)
		local_clk_disable(clk);

	if (ret != 0)
		printk(KERN_ERR "%s: error %d\n", __func__, ret);
	return ret;
}

static int ssp_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;

	if (clk->bypass_reg) {
		if (clk->parent == parent)
			return 0;
		if (parent == &ref_io_clk)
			__raw_writel(1 << clk->bypass_bits,
					clk->bypass_reg + CLR_REGISTER);
		else
			__raw_writel(1 << clk->bypass_bits,
					clk->bypass_reg + SET_REGISTER);
		clk->parent = parent;
		ret = 0;
	}

	return ret;
}

/* handle peripheral clocks whose optimal parent dependent on
 * system parameters such as cpu_clk rate.  For now, this optimization
 * only occurs to the peripheral clock when it's not in use to avoid
 * handling more complex system clock coordination issues.
 */
static int ssp_set_sys_dependent_parent(struct clk *clk)
{
	if ((clk->ref & CLK_EN_MASK) == 0) {
		if (clk_get_rate(&cpu_clk) > ref_xtal_get_rate(&ref_xtal_clk)) {
			clk_set_parent(clk, &ref_io_clk);
			clk_set_rate(clk, PLL_ENABLED_MAX_CLK_SSP);
		} else {
			clk_set_parent(clk, &ref_xtal_clk);
			clk_set_rate(clk, ref_xtal_get_rate(&ref_xtal_clk));
		}
	}

	return 0;
}

static struct clk ssp_clk = {
	 .parent = &ref_io_clk,
	 .get_rate = ssp_get_rate,
	 .enable = mx23_raw_enable,
	 .disable = mx23_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP,
	 .enable_bits = BM_CLKCTRL_SSP_CLKGATE,
	 .busy_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP,
	 .busy_bits = 29,
	 .scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP,
	 .scale_bits = 0,
	 .bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	 .bypass_bits = 5,
	 .set_rate = ssp_set_rate,
	 .set_parent = ssp_set_parent,
	 .set_sys_dependent_parent = ssp_set_sys_dependent_parent,
};

static unsigned long ssp_get_rate(struct clk *clk)
{
	unsigned int reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP) &
		    BM_CLKCTRL_SSP_DIV;

	return clk->parent->get_rate(clk->parent) / reg;
}

static unsigned long gpmi_get_rate(struct clk *clk)
{
	unsigned int reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_GPMI) &
		    BM_CLKCTRL_GPMI_DIV;

	return clk->parent->get_rate(clk->parent) / reg;
}

static int gpmi_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EINVAL;
	u32 reg, div;

	/* Make absolutely certain the clock is enabled. */
	local_clk_enable(clk);

	/* if the desired clock can be sourced from ref_xtal,
	 * use ref_xtal to save power
	 */
	if ((rate <= ref_xtal_get_rate(&ref_xtal_clk)) &&
		((ref_xtal_get_rate(&ref_xtal_clk) % rate) == 0))
		clk_set_parent(clk, &ref_xtal_clk);
	else
		clk_set_parent(clk, &ref_io_clk);

	if (rate > PLL_ENABLED_MAX_CLK_SSP)
		rate = PLL_ENABLED_MAX_CLK_GPMI;

	div = (clk_get_rate(clk->parent) + rate - 1) / rate;

	if (div == 0 || div > BM_CLKCTRL_GPMI_DIV)
		goto out;

	reg = __raw_readl(clk->scale_reg);
	reg &= ~(BM_CLKCTRL_GPMI_DIV | BM_CLKCTRL_GPMI_DIV_FRAC_EN);
	reg |= div << clk->scale_bits;
	__raw_writel(reg, clk->scale_reg);

	ret = clk_busy_wait(clk);

out:

	/* Undo the enable above. */
	local_clk_disable(clk);

	if (ret != 0)
		printk(KERN_ERR "%s: error %d\n", __func__, ret);
	return ret;
}

static int gpmi_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;

	if (clk->bypass_reg) {
		if (clk->parent == parent)
			return 0;
		if (parent == &ref_io_clk)
			__raw_writel(1 << clk->bypass_bits,
					clk->bypass_reg + CLR_REGISTER);
		else
			__raw_writel(1 << clk->bypass_bits,
					clk->bypass_reg + SET_REGISTER);
		clk->parent = parent;
		ret = 0;
	}

	return ret;
}

/* handle peripheral clocks whose optimal parent dependent on
 * system parameters such as cpu_clk rate.  For now, this optimization
 * only occurs to the peripheral clock when it's not in use to avoid
 * handling more complex system clock coordination issues.
 */
static int gpmi_set_sys_dependent_parent(struct clk *clk)
{

	if ((clk->ref & CLK_EN_MASK) == 0) {
		if (clk_get_rate(&cpu_clk) > ref_xtal_get_rate(&ref_xtal_clk)) {
			clk_set_parent(clk, &ref_io_clk);
			clk_set_rate(clk, PLL_ENABLED_MAX_CLK_GPMI);
		} else {
			clk_set_parent(clk, &ref_xtal_clk);
			clk_set_rate(clk, ref_xtal_get_rate(&ref_xtal_clk));
		}
	}

	return 0;
}

static struct clk gpmi_clk = {
	.parent		= &ref_io_clk,
	.secondary      = 0,
	.flags          = 0,
	.set_parent     = gpmi_set_parent,
	.set_sys_dependent_parent = gpmi_set_sys_dependent_parent,

	.enable_reg     = CLKCTRL_BASE_ADDR + HW_CLKCTRL_GPMI,
	.enable_bits    = BM_CLKCTRL_GPMI_CLKGATE,
	.enable         = mx23_raw_enable,
	.disable        = mx23_raw_disable,

	.scale_reg      = CLKCTRL_BASE_ADDR + HW_CLKCTRL_GPMI,
	.scale_bits     = 0,
	.round_rate     = 0,
	.set_rate       = gpmi_set_rate,
	.get_rate       = gpmi_get_rate,

	.bypass_reg     = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	.bypass_bits    = 4,

	.busy_reg       = CLKCTRL_BASE_ADDR + HW_CLKCTRL_GPMI,
	.busy_bits      = 29,
};

static unsigned long pcmspdif_get_rate(struct clk *clk)
{
	return clk->parent->get_rate(clk->parent) / 4;
}

static struct clk pcmspdif_clk = {
	.parent = &pll_clk,
	.get_rate = pcmspdif_get_rate,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SPDIF,
	.enable_bits = BM_CLKCTRL_SPDIF_CLKGATE,
};

/* usb_clk for usb0 */
static struct clk usb_clk = {
	.parent = &pll_clk,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.enable_reg = DIGCTRL_BASE_ADDR + HW_DIGCTL_CTRL,
	.enable_bits = BM_DIGCTL_CTRL_USB_CLKGATE,
	.flags		= CPU_FREQ_TRIG_UPDATE,
};

static struct clk audio_clk = {
	.parent = &ref_xtal_clk,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_XTAL,
	.enable_bits = BM_CLKCTRL_XTAL_FILT_CLK24M_GATE,
};

static struct clk vid_clk = {
	.parent		= &ref_xtal_clk,
	.enable         = mx23_raw_enable,
	.disable        = mx23_raw_disable,
	.enable_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1,
	.enable_bits	= BM_CLKCTRL_FRAC1_CLKGATEVID,
};

static struct clk tv108M_ng_clk = {
	.parent		= &vid_clk,
	.enable         = mx23_raw_enable,
	.disable        = mx23_raw_disable,
	.enable_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_TV,
	.enable_bits	= BM_CLKCTRL_TV_CLK_TV108M_GATE,
	.flags		= RATE_FIXED,
};

static struct clk tv27M_clk = {
	.parent		= &vid_clk,
	.enable         = mx23_raw_enable,
	.disable        = mx23_raw_disable,
	.enable_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_TV,
	.enable_bits	= BM_CLKCTRL_TV_CLK_TV_GATE,
	.flags		= RATE_FIXED,
};

static struct clk_lookup onchip_clocks[] = {
	{
	 .con_id = "pll.0",
	 .clk = &pll_clk,
	 },
	{
	 .con_id = "ref_xtal",
	 .clk = &ref_xtal_clk,
	 },
	{
	 .con_id = "ref_cpu",
	 .clk = &ref_cpu_clk,
	 },
	{
	 .con_id = "ref_emi",
	 .clk = &ref_emi_clk,
	 },
	{
	 .con_id = "ref_io.0",
	 .clk = &ref_io_clk,
	 },
	{
	 .con_id = "ref_pix",
	 .clk = &ref_pix_clk,
	 },
	{
	 .con_id = "lcdif",
	 .clk = &lcdif_clk,
	 },
	{
	 .con_id = "xtal_clock32k",
	 .clk = &xtal_clock32k_clk,
	 },
	{
	 .con_id = "rtc",
	 .clk = &rtc32k_clk,
	 },
	{
	 .con_id = "cpu",
	 .clk = &cpu_clk,
	 },
	{
	 .con_id = "h",
	 .clk = &h_clk,
	 },
	{
	 .con_id = "x",
	 .clk = &x_clk,
	 },
	{
	 .con_id = "ocrom",
	 .clk = &ocrom_clk,
	 },
	{
	 .con_id = "clk_32k",
	 .clk = &clk_32k,
	 },
	{
	 .con_id = "uart",
	 .clk = &uart_clk,
	 },
	{
	 .con_id = "pwm",
	 .clk = &pwm_clk,
	 },
	{
	 .con_id = "lradc",
	 .clk = &lradc_clk,
	 },
	{
	 .con_id = "ssp.0",
	 .clk = &ssp_clk,
	 },
	{
	 .con_id = "emi",
	 .clk = &emi_clk,
	 },
	{
	.con_id = "usb_clk0",
	.clk = &usb_clk,
	},
	{
	.con_id = "audio",
	.clk = &audio_clk,
	},
	{
	 .con_id = "spdif",
	 .clk = &pcmspdif_clk,
	},
	{
	 .con_id = "ref_vid",
	 .clk = &vid_clk,
	},
	{
	 .con_id = "tv108M_ng",
	 .clk = &tv108M_ng_clk,
	},
	{
	 .con_id = "tv27M",
	 .clk = &tv27M_clk,
	},
	{
	 .con_id = "gpmi",
	 .clk = &gpmi_clk,
	},
};

/* for debugging */
#ifdef DEBUG
static void print_ref_counts(void)
{

	printk(KERN_INFO "pll_clk ref count: %i\n",
		pll_clk.ref & CLK_EN_MASK);

	printk(KERN_INFO "ref_cpu_clk ref count: %i\n",
		ref_cpu_clk.ref & CLK_EN_MASK);

	printk(KERN_INFO "ref_emi_clk ref count: %i\n",
		ref_emi_clk.ref & CLK_EN_MASK);

	printk(KERN_INFO "lcdif_clk ref count: %i\n",
		lcdif_clk.ref & CLK_EN_MASK);

	printk(KERN_INFO "ref_io_clk ref count: %i\n",
		ref_io_clk.ref & CLK_EN_MASK);

	printk(KERN_INFO "ssp_clk ref count: %i\n",
		ssp_clk.ref & CLK_EN_MASK);

	printk(KERN_INFO "gpmi_clk ref count: %i\n",
		gpmi_clk.ref & CLK_EN_MASK);

}
#endif

static void mx23_clock_scan(void)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_CPU)
		cpu_clk.parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_EMI)
		emi_clk.parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_SSP)
		ssp_clk.parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_GPMI)
		gpmi_clk.parent = &ref_xtal_clk;

	reg = __raw_readl(RTC_BASE_ADDR + HW_RTC_PERSISTENT0);
	if (!(reg & BM_RTC_PERSISTENT0_CLOCKSOURCE))
		rtc32k_clk.parent = &ref_xtal_clk;
};

void __init mx23_set_input_clk(unsigned long xtal0,
			       unsigned long xtal1,
			       unsigned long xtal2, unsigned long enet)
{

}

void __init mx23_clock_init(void)
{
	int i;
	mx23_clock_scan();
	for (i = 0; i < ARRAY_SIZE(onchip_clocks); i++)
		clk_register(&onchip_clocks[i]);

	clk_enable(&cpu_clk);
	clk_enable(&emi_clk);

	clk_en_public_h_asm_ctrl(mx23_enable_h_autoslow,
		mx23_set_hbus_autoslow_flags);
}
