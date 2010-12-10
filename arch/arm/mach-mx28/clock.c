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
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iram_alloc.h>
#include <linux/platform_device.h>

#include <mach/clock.h>

#include "regs-clkctrl.h"
#include "regs-digctl.h"
#include "emi_settings.h"

#define HW_SAIF_CTRL    (0x00000000)
#define HW_SAIF_STAT    (0x00000010)
#define SAIF0_CTRL (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_CTRL)
#define SAIF0_STAT (IO_ADDRESS(SAIF0_PHYS_ADDR) + HW_SAIF_STAT)
#define SAIF1_CTRL (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_CTRL)
#define SAIF1_STAT (IO_ADDRESS(SAIF1_PHYS_ADDR) + HW_SAIF_STAT)
#define BM_SAIF_CTRL_RUN        0x00000001
#define BM_SAIF_STAT_BUSY       0x00000001
#define CLKCTRL_BASE_ADDR IO_ADDRESS(CLKCTRL_PHYS_ADDR)
#define DIGCTRL_BASE_ADDR IO_ADDRESS(DIGCTL_PHYS_ADDR)

/* external clock input */
static struct clk xtal_clk[];
static unsigned long xtal_clk_rate[3] = { 24000000, 24000000, 32000 };

static unsigned long enet_mii_phy_rate;

static inline int clk_is_busy(struct clk *clk)
{
	return __raw_readl(clk->busy_reg) & (1 << clk->busy_bits);
}

static bool mx28_enable_h_autoslow(bool enable)
{
	bool currently_enabled;

	if (__raw_readl(CLKCTRL_BASE_ADDR+HW_CLKCTRL_HBUS) &
		BM_CLKCTRL_HBUS_ASM_ENABLE)
		currently_enabled = true;
	else
		currently_enabled = false;

	if (enable)
		__raw_writel(BM_CLKCTRL_HBUS_ASM_ENABLE,
			CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS_SET);
	else
		__raw_writel(BM_CLKCTRL_HBUS_ASM_ENABLE,
			CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS_CLR);
	return currently_enabled;
}


static void mx28_set_hbus_autoslow_flags(u16 mask)
{
	u32 reg;

	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);
	reg &= 0xFFFF;
	reg |= mask << 16;
	__raw_writel(reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);
}

static int mx28_raw_enable(struct clk *clk)
{
	unsigned int reg;
	if (clk->enable_reg) {
		reg = __raw_readl(clk->enable_reg);
		reg &= ~clk->enable_bits;
		__raw_writel(reg, clk->enable_reg);
	}
	return 0;
}

static void mx28_raw_disable(struct clk *clk)
{
	unsigned int reg;
	if (clk->enable_reg) {
		reg = __raw_readl(clk->enable_reg);
		reg |= clk->enable_bits;
		__raw_writel(reg, clk->enable_reg);
	}
}

static unsigned int
mx28_get_frac_div(unsigned long root_rate, unsigned long rate, unsigned mask)
{
	unsigned long mult_rate;
	unsigned int div;
	mult_rate = rate * (mask + 1);
	div = mult_rate / root_rate;
	if ((mult_rate % root_rate) && (div < mask))
		div--;
	return div;
}

static unsigned long xtal_get_rate(struct clk *clk)
{
	int id = clk - xtal_clk;
	return xtal_clk_rate[id];
}

static struct clk xtal_clk[] = {
	{
	 .flags = RATE_FIXED,
	 .get_rate = xtal_get_rate,
	 },
	{
	 .flags = RATE_FIXED,
	 .get_rate = xtal_get_rate,
	 },
	{
	 .flags = RATE_FIXED,
	 .get_rate = xtal_get_rate,
	 },
};

static struct clk ref_xtal_clk = {
	.parent = &xtal_clk[0],
};

static unsigned long pll_get_rate(struct clk *clk);
static int pll_enable(struct clk *clk);
static void pll_disable(struct clk *clk);
static int pll_set_rate(struct clk *clk, unsigned long rate);
static struct clk pll_clk[] = {
	{
	 .parent = &ref_xtal_clk,
	 .flags = RATE_FIXED,
	 .get_rate = pll_get_rate,
	 .set_rate = pll_set_rate,
	 .enable = pll_enable,
	 .disable = pll_disable,
	 },
	{
	 .parent = &ref_xtal_clk,
	 .flags = RATE_FIXED,
	 .get_rate = pll_get_rate,
	 .set_rate = pll_set_rate,
	 .enable = pll_enable,
	 .disable = pll_disable,
	 },
	{
	 .parent = &ref_xtal_clk,
	 .flags = RATE_FIXED,
	 .get_rate = pll_get_rate,
	 .set_rate = pll_set_rate,
	 .enable = pll_enable,
	 .disable = pll_disable,
	 }
};

static unsigned long pll_get_rate(struct clk *clk)
{
	unsigned int reg;
	if (clk == (pll_clk + 2))
		return 50000000;
	if (clk == pll_clk) {
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL0CTRL1);
		reg = (reg & BM_CLKCTRL_PLL0CTRL0_DIV_SEL) >>
			BP_CLKCTRL_PLL0CTRL0_DIV_SEL;
	} else {
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL1CTRL1);
		reg = (reg & BM_CLKCTRL_PLL1CTRL0_DIV_SEL) >>
			BP_CLKCTRL_PLL1CTRL0_DIV_SEL;
	}
	switch (reg) {
	case 0:
		return 480000000;
	case 1:
		return 384000000;
	case 2:
		return 288000000;
	default:
		return -EINVAL;
	}
}

static int pll_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int div, reg;

	if (clk == pll_clk + 2)
		return -EINVAL;

	switch (rate) {
	case 480000000:
		div = 0;
		break;
	case 384000000:
		div = 1;
		break;
	case 288000000:
		div = 2;
		break;
	default:
		return -EINVAL;
	}
	if (clk == pll_clk) {
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL0CTRL1);
		reg &= ~BM_CLKCTRL_PLL0CTRL0_DIV_SEL;
		reg |= BF_CLKCTRL_PLL0CTRL0_DIV_SEL(div);
		__raw_writel(reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL0CTRL1);
	} else {
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL1CTRL1);
		reg &= ~BM_CLKCTRL_PLL1CTRL0_DIV_SEL;
		reg |= BF_CLKCTRL_PLL1CTRL0_DIV_SEL(div);
		__raw_writel(reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL1CTRL1);
	}
	return 0;
}

static int pll_enable(struct clk *clk)
{
	int timeout = 100;
	unsigned long reg;
	switch (clk - pll_clk) {
	case 0:
		__raw_writel(BM_CLKCTRL_PLL0CTRL0_POWER,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL0CTRL0_SET);
		do {
			udelay(10);
			reg = __raw_readl(CLKCTRL_BASE_ADDR +
					  HW_CLKCTRL_PLL0CTRL1);
			timeout--;
		} while ((timeout > 0) && !(reg & BM_CLKCTRL_PLL0CTRL1_LOCK));
		if (timeout <= 0)
			return -EFAULT;
		return 0;
	case 1:
		__raw_writel(BM_CLKCTRL_PLL1CTRL0_POWER,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL1CTRL0_SET);
		do {
			udelay(10);
			reg = __raw_readl(CLKCTRL_BASE_ADDR +
					  HW_CLKCTRL_PLL1CTRL1);
			timeout--;
		} while ((timeout > 0) && !(reg & BM_CLKCTRL_PLL1CTRL1_LOCK));
		if (timeout <= 0)
			return -EFAULT;
		return 0;
	case 2:
		__raw_writel(BM_CLKCTRL_PLL2CTRL0_POWER,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL2CTRL0_SET);
		udelay(10);
		__raw_writel(BM_CLKCTRL_PLL2CTRL0_CLKGATE,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL2CTRL0_CLR);
		break;
	}
	return -ENODEV;
}

static void pll_disable(struct clk *clk)
{
	switch (clk - pll_clk) {
	case 0:
		__raw_writel(BM_CLKCTRL_PLL0CTRL0_POWER,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL0CTRL0_CLR);
		return;
	case 1:
		__raw_writel(BM_CLKCTRL_PLL1CTRL0_POWER,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL1CTRL0_CLR);
		return;
	case 2:
		__raw_writel(BM_CLKCTRL_PLL2CTRL0_CLKGATE,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL2CTRL0_SET);
		__raw_writel(BM_CLKCTRL_PLL2CTRL0_POWER,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL2CTRL0_CLR);
		break;
	}
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
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0) &
	    BM_CLKCTRL_FRAC0_CPUFRAC;
	return ref_clk_get_rate(clk->parent->get_rate(clk->parent), reg);
}


static struct clk ref_cpu_clk = {
	.parent = &pll_clk[0],
	.get_rate = ref_cpu_get_rate,
	.round_rate = ref_clk_round_rate,
	.set_rate = ref_clk_set_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	.enable_bits = BM_CLKCTRL_FRAC0_CLKGATECPU,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	.scale_bits = BP_CLKCTRL_FRAC0_CPUFRAC,
};

static unsigned long ref_emi_get_rate(struct clk *clk)
{
	unsigned int reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0) &
	    BM_CLKCTRL_FRAC0_EMIFRAC;
	reg >>= BP_CLKCTRL_FRAC0_EMIFRAC;
	return ref_clk_get_rate(clk->parent->get_rate(clk->parent), reg);
}

static struct clk ref_emi_clk = {
	.parent = &pll_clk[0],
	.get_rate = ref_emi_get_rate,
	.set_rate = ref_clk_set_rate,
	.round_rate = ref_clk_round_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	.enable_bits = BM_CLKCTRL_FRAC0_CLKGATEEMI,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	.scale_bits = BP_CLKCTRL_FRAC0_EMIFRAC,
};

static unsigned long ref_io_get_rate(struct clk *clk);
static struct clk ref_io_clk[] = {
	{
	 .parent = &pll_clk[0],
	 .get_rate = ref_io_get_rate,
	 .set_rate = ref_clk_set_rate,
	 .round_rate = ref_clk_round_rate,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	 .enable_bits = BM_CLKCTRL_FRAC0_CLKGATEIO0,
	 .scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	 .scale_bits = BP_CLKCTRL_FRAC0_IO0FRAC,
	 },
	{
	 .parent = &pll_clk[0],
	 .get_rate = ref_io_get_rate,
	 .set_rate = ref_clk_set_rate,
	 .round_rate = ref_clk_round_rate,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	 .enable_bits = BM_CLKCTRL_FRAC0_CLKGATEIO1,
	 .scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	 .scale_bits = BP_CLKCTRL_FRAC0_IO1FRAC,
	 },
};

static unsigned long ref_io_get_rate(struct clk *clk)
{
	unsigned int reg;
	if (clk == ref_io_clk) {
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0) &
		    BM_CLKCTRL_FRAC0_IO0FRAC;
		reg >>= BP_CLKCTRL_FRAC0_IO0FRAC;
	} else {
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0) &
		    BM_CLKCTRL_FRAC0_IO1FRAC;
		reg >>= BP_CLKCTRL_FRAC0_IO1FRAC;
	}
	return ref_clk_get_rate(clk->parent->get_rate(clk->parent), reg);
}

static unsigned long ref_pix_get_rate(struct clk *clk)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1) &
	    BM_CLKCTRL_FRAC1_PIXFRAC;
	reg >>= BP_CLKCTRL_FRAC1_PIXFRAC;
	return ref_clk_get_rate(clk->parent->get_rate(clk->parent), reg);
}

static struct clk ref_pix_clk = {
	.parent = &pll_clk[0],
	.get_rate = ref_pix_get_rate,
	.set_rate = ref_clk_set_rate,
	.round_rate = ref_clk_round_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1,
	.enable_bits = BM_CLKCTRL_FRAC1_CLKGATEPIX,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1,
	.scale_bits = BP_CLKCTRL_FRAC1_PIXFRAC,
};

static unsigned long ref_hsadc_get_rate(struct clk *clk)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1) &
	    BM_CLKCTRL_FRAC1_HSADCFRAC;
	reg >>= BP_CLKCTRL_FRAC1_HSADCFRAC;
	return ref_clk_get_rate(clk->parent->get_rate(clk->parent), reg);
}

static struct clk ref_hsadc_clk = {
	.parent = &pll_clk[0],
	.get_rate = ref_hsadc_get_rate,
	.set_rate = ref_clk_set_rate,
	.round_rate = ref_clk_round_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1,
	.enable_bits = BM_CLKCTRL_FRAC1_CLKGATEHSADC,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1,
	.scale_bits = BP_CLKCTRL_FRAC1_HSADCFRAC,
};

static unsigned long ref_gpmi_get_rate(struct clk *clk)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1) &
	    BM_CLKCTRL_FRAC1_GPMIFRAC;
	reg >>= BP_CLKCTRL_FRAC1_GPMIFRAC;
	return ref_clk_get_rate(clk->parent->get_rate(clk->parent), reg);
}

static struct clk ref_gpmi_clk = {
	.parent = &pll_clk[0],
	.get_rate = ref_gpmi_get_rate,
	.set_rate = ref_clk_set_rate,
	.round_rate = ref_clk_round_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1,
	.enable_bits = BM_CLKCTRL_FRAC1_CLKGATEGPMI,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1,
	.scale_bits = BP_CLKCTRL_FRAC1_GPMIFRAC,
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

static struct clk  h_clk;
static int cpu_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long root_rate =
			clk->parent->parent->get_rate(clk->parent->parent);
	int i;
	u32 clkctrl_cpu = 1;
	u32 c = clkctrl_cpu;
	u32 clkctrl_frac = 1;
	u32 val;
	u32 reg_val, hclk_reg;

	if (rate < 24000)
		return -EINVAL;
	else if (rate == 24000) {
		/* switch to the 24M source */
		clk_set_parent(clk, &ref_xtal_clk);
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

		/* Set safe hbus clock divider. A divider of 3 ensure that
		 * the Vddd voltage required for the cpuclk is sufficiently
		 * high for the hbus clock.
		 */
		hclk_reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);
		if ((hclk_reg & BP_CLKCTRL_HBUS_DIV) != 3) {
			hclk_reg &= ~(BM_CLKCTRL_HBUS_DIV);
			hclk_reg |= BF_CLKCTRL_HBUS_DIV(3);

			/* change hclk divider to safe value for any ref_cpu
			 * value.
			 */
			__raw_writel(hclk_reg, CLKCTRL_BASE_ADDR +
				     HW_CLKCTRL_HBUS);
		}

		for (i = 10000; i; i--)
			if (!clk_is_busy(&h_clk))
				break;
		if (!i) {
			printk(KERN_ERR "couldn't set up HCLK divisor\n");
			return -ETIMEDOUT;
		}

		/* Set Frac div */
		val = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0);
		val &= ~(BM_CLKCTRL_FRAC0_CPUFRAC << BP_CLKCTRL_FRAC0_CPUFRAC);
		val |= clkctrl_frac;
		__raw_writel(val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0);
		/* Do not gate */
		__raw_writel(BM_CLKCTRL_FRAC0_CLKGATECPU, CLKCTRL_BASE_ADDR +
			     HW_CLKCTRL_FRAC0_CLR);

		/* write clkctrl_cpu */
		reg_val = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_CPU);
		reg_val &= ~0x3F;
		reg_val |= clkctrl_cpu;

		__raw_writel(reg_val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_CPU);

		for (i = 10000; i; i--)
			if (!clk_is_busy(clk))
				break;
		if (!i) {
			printk(KERN_ERR "couldn't set up CPU divisor\n");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static int cpu_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;

	if (clk->bypass_reg) {
		if (parent == clk->parent)
			return 0;
		if (parent == &ref_xtal_clk) {
			__raw_writel(1 << clk->bypass_bits,
				clk->bypass_reg + SET_REGISTER);
			ret = 0;
		}
		if (ret && (parent == &ref_cpu_clk)) {
			__raw_writel(1 << clk->bypass_bits,
				clk->bypass_reg + CLR_REGISTER);
			ret = 0;
		}
		if (!ret)
			clk->parent = parent;
	}
	return ret;
}

static struct clk cpu_clk = {
	.parent = &ref_cpu_clk,
	.get_rate = cpu_get_rate,
	.round_rate = cpu_round_rate,
	.set_rate = cpu_set_rate,
	.set_parent = cpu_set_parent,
	.bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	.bypass_bits = 18,
	.busy_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	.busy_bits = 28,
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
	unsigned long reg, div;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_XBUS);
	div = reg & BM_CLKCTRL_XBUS_DIV;
	if (!(reg & BM_CLKCTRL_XBUS_DIV_FRAC_EN))
		return clk->parent->get_rate(clk->parent) / div;
	return (clk->parent->get_rate(clk->parent) / 0x400) * div;
}

static unsigned long x_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned int root_rate, frac_rate;
	unsigned int div;
	root_rate = clk->parent->get_rate(clk->parent);
	frac_rate = root_rate % rate;
	div = root_rate / rate;
	if ((div == 0) || (div >= 0x400))
		return root_rate;
	if (frac_rate == 0)
		return rate;
	return rate;
}

static int x_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long root_rate;
	unsigned int reg, div;
	root_rate = clk->parent->get_rate(clk->parent);
	div = root_rate / rate;
	if ((div == 0) || (div >= 0x400))
		return -EINVAL;

	if (root_rate % rate) {
		div = mx28_get_frac_div(root_rate / 1000, rate / 1000, 0x3FF);
		if (((root_rate / 0x400) * div) > rate)
			return -EINVAL;
	}
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_XBUS);
	reg &= ~(BM_CLKCTRL_XBUS_DIV | BM_CLKCTRL_XBUS_DIV_FRAC_EN);
	if (root_rate % rate)
		reg |= BM_CLKCTRL_XBUS_DIV_FRAC_EN;
	reg |= BF_CLKCTRL_XBUS_DIV(div);
	__raw_writel(reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_XBUS);
	return 0;
}

static struct clk x_clk = {
	.parent = &ref_xtal_clk,
	.get_rate = x_get_rate,
	.set_rate = x_set_rate,
	.round_rate = x_round_rate,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_XBUS,
	.scale_bits = BM_CLKCTRL_XBUS_BUSY,
};

static struct clk ana_clk = {
	.parent = &ref_xtal_clk,
};

static unsigned long rtc_get_rate(struct clk *clk)
{
	if (clk->parent == &xtal_clk[2])
		return clk->parent->get_rate(clk->parent);
	return clk->parent->get_rate(clk->parent) / 768;
}

static struct clk rtc_clk = {
	.parent = &ref_xtal_clk,
	.get_rate = rtc_get_rate,
};

static struct clk flexcan_clk[] = {
	{
	 .parent = &ref_xtal_clk,
	 .enable = mx28_raw_enable,
	 .disable = mx28_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FLEXCAN,
	 .enable_bits = BM_CLKCTRL_FLEXCAN_STOP_CAN0,
	 },
	{
	 .parent = &ref_xtal_clk,
	 .enable = mx28_raw_enable,
	 .disable = mx28_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FLEXCAN,
	 .enable_bits = BM_CLKCTRL_FLEXCAN_STOP_CAN1,
	 },
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
	if (frac_rate < (rate / 2))
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

	if ((root_rate < rate) && (root_rate == 64000000))
		div = 3;

	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);
	reg &= ~(BM_CLKCTRL_HBUS_DIV_FRAC_EN | BM_CLKCTRL_HBUS_DIV);
	reg |= BF_CLKCTRL_HBUS_DIV(div);
	__raw_writel(reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);

	if (clk->busy_reg) {
		int i;
		for (i = 10000; i; i--)
			if (!clk_is_busy(clk))
				break;
		if (!i) {
			printk(KERN_ERR "couldn't set up AHB divisor\n");
			return -ETIMEDOUT;
		}
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

static int emi_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;
	if (clk->bypass_reg) {
		if (parent == clk->parent)
			return 0;
		if (parent == &ref_xtal_clk) {
			__raw_writel(1 << clk->bypass_bits,
				clk->bypass_reg + SET_REGISTER);
			ret = 0;
		}
		if (ret && (parent == &ref_emi_clk)) {
			__raw_writel(0 << clk->bypass_bits,
				clk->bypass_reg + CLR_REGISTER);
			ret = 0;
		}
		if (!ret)
			clk->parent = parent;
	}
	return ret;
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

static int emi_set_rate(struct clk *clk, unsigned long rate)
{
	int i;
	struct mxs_emi_scaling_data emi;
	unsigned long iram_phy;
	void (*f) (struct mxs_emi_scaling_data *, unsigned int *);
	f = iram_alloc((unsigned int)mxs_ram_freq_scale_end -
		(unsigned int)mxs_ram_freq_scale, &iram_phy);
	if (NULL == f) {
		pr_err("%s Not enough iram\n", __func__);
		return -ENOMEM;
	}
	memcpy(f, mxs_ram_freq_scale,
	       (unsigned int)mxs_ram_freq_scale_end -
	       (unsigned int)mxs_ram_freq_scale);
#ifdef CONFIG_MEM_mDDR
	if (rate <= 24000000) {
		emi.emi_div = 20;
		emi.frac_div = 18;
		emi.new_freq = 24;
		mDDREmiController_24MHz();
	} else if (rate <= 133000000) {
		emi.emi_div = 3;
		emi.frac_div = 22;
		emi.new_freq = 133;
		mDDREmiController_133MHz();
	} else {
		emi.emi_div = 2;
		emi.frac_div = 22;
		emi.new_freq = 200;
		mDDREmiController_200MHz();
		}
#else
	if (rate <= 133000000) {
		emi.emi_div = 3;
		emi.frac_div = 22;
		emi.new_freq = 133;
		DDR2EmiController_EDE1116_133MHz();
	} else if (rate <= 166000000) {
		emi.emi_div = 2;
		emi.frac_div = 27;
		emi.new_freq = 166;
		DDR2EmiController_EDE1116_166MHz();
	} else {
		emi.emi_div = 2;
		emi.frac_div = 22;
		emi.new_freq = 200;
		DDR2EmiController_EDE1116_200MHz();
		}
#endif

	local_irq_disable();
	local_fiq_disable();
	f(&emi, get_current_emidata());
	local_fiq_enable();
	local_irq_enable();
	iram_free(iram_phy,
		(unsigned int)mxs_ram_freq_scale_end -
	       (unsigned int)mxs_ram_freq_scale);

	for (i = 10000; i; i--)
		if (!clk_is_busy(clk))
			break;

	if (!i) {
		printk(KERN_ERR "couldn't set up EMI divisor\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static struct clk emi_clk = {
	.parent = &ref_emi_clk,
	.get_rate = emi_get_rate,
	.set_rate = emi_set_rate,
	.round_rate = emi_round_rate,
	.set_parent = emi_set_parent,
	.enable = mx28_raw_enable,
	.disable = mx28_raw_disable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_EMI,
	.enable_bits = BM_CLKCTRL_EMI_CLKGATE,
	.scale_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	.busy_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_EMI,
	.busy_bits	= 28,
	.bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	.bypass_bits = 7,
};

static unsigned long ssp_get_rate(struct clk *clk);

static int ssp_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EINVAL;
	int div = (clk_get_rate(clk->parent) + rate - 1) / rate;
	u32 reg_frac;
	const int mask = 0x1FF;
	int try = 10;
	int i = -1;

	if (div == 0 || div > mask)
		goto out;

	reg_frac = __raw_readl(clk->scale_reg);
	reg_frac &= ~(mask << clk->scale_bits);

	while (try--) {
		__raw_writel(reg_frac | (div << clk->scale_bits),
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
	if (ret != 0)
		pr_err("%s: error %d\n", __func__, ret);
	return ret;
}

static int ssp_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;

	if (clk->bypass_reg) {
		if (clk->parent == parent)
			return 0;
		if (parent == &ref_io_clk[0] || parent == &ref_io_clk[1])
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

static struct clk ssp_clk[] = {
	{
	 .parent = &ref_io_clk[0],
	 .get_rate = ssp_get_rate,
	 .enable = mx28_raw_enable,
	 .disable = mx28_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP0,
	 .enable_bits = BM_CLKCTRL_SSP0_CLKGATE,
	 .busy_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP0,
	 .busy_bits = 29,
	 .scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP0,
	 .scale_bits = 0,
	 .bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	 .bypass_bits = 3,
	 .set_rate = ssp_set_rate,
	 .set_parent = ssp_set_parent,
	 },
	{
	 .parent = &ref_io_clk[0],
	 .get_rate = ssp_get_rate,
	 .enable = mx28_raw_enable,
	 .disable = mx28_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP1,
	 .enable_bits = BM_CLKCTRL_SSP1_CLKGATE,
	 .busy_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP1,
	 .busy_bits = 29,
	 .scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP1,
	 .scale_bits = 0,
	 .bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	 .bypass_bits = 4,
	 .set_rate = ssp_set_rate,
	 .set_parent = ssp_set_parent,
	 },
	{
	 .parent = &ref_io_clk[1],
	 .get_rate = ssp_get_rate,
	 .enable = mx28_raw_enable,
	 .disable = mx28_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP2,
	 .enable_bits = BM_CLKCTRL_SSP2_CLKGATE,
	 .busy_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP2,
	 .busy_bits = 29,
	 .scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP2,
	 .scale_bits = 0,
	 .bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	 .bypass_bits = 5,
	 .set_rate = ssp_set_rate,
	 .set_parent = ssp_set_parent,
	 },
	{
	 .parent = &ref_io_clk[1],
	 .get_rate = ssp_get_rate,
	 .enable = mx28_raw_enable,
	 .disable = mx28_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP3,
	 .enable_bits = BM_CLKCTRL_SSP3_CLKGATE,
	 .busy_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP3,
	 .busy_bits = 29,
	 .scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP3,
	 .scale_bits = 0,
	 .bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	 .bypass_bits = 6,
	 .set_rate = ssp_set_rate,
	 .set_parent = ssp_set_parent,
	 },
};

static unsigned long ssp_get_rate(struct clk *clk)
{
	unsigned int reg, div;
	switch (clk - ssp_clk) {
	case 0:
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP0);
		div = reg & BM_CLKCTRL_SSP0_DIV;
		reg &= BM_CLKCTRL_SSP0_DIV_FRAC_EN;
		break;
	case 1:
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP1);
		div = reg & BM_CLKCTRL_SSP1_DIV;
		reg &= BM_CLKCTRL_SSP1_DIV_FRAC_EN;
		break;
	case 2:
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP2);
		div = reg & BM_CLKCTRL_SSP2_DIV;
		reg &= BM_CLKCTRL_SSP2_DIV_FRAC_EN;
		break;
	case 3:
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP3);
		div = reg & BM_CLKCTRL_SSP3_DIV;
		reg &= BM_CLKCTRL_SSP3_DIV_FRAC_EN;
		break;
	default:
		return 0;
	}
	if (!reg)
		return clk->parent->get_rate(clk->parent) / div;
	return (clk->parent->get_rate(clk->parent) / 0x200) / div;
}

static unsigned long lcdif_get_rate(struct clk *clk)
{
	long rate = clk->parent->get_rate(clk->parent);
	long div;

	div = __raw_readl(clk->scale_reg);
	if (!(div & BM_CLKCTRL_DIS_LCDIF_DIV_FRAC_EN)) {
		div = (div >> clk->scale_bits) & BM_CLKCTRL_DIS_LCDIF_DIV;
		return rate / (div ? div : 1);
	}

	div = (div >> clk->scale_bits) & BM_CLKCTRL_DIS_LCDIF_DIV;
	rate /= (BM_CLKCTRL_DIS_LCDIF_DIV >> clk->scale_bits) + 1;
	rate *= div;
	return rate;
}

static int lcdif_set_rate(struct clk *clk, unsigned long rate)
{
	int reg_val;

	reg_val = __raw_readl(clk->scale_reg);
	reg_val &= ~(BM_CLKCTRL_DIS_LCDIF_DIV | BM_CLKCTRL_DIS_LCDIF_CLKGATE);
	reg_val |= (1 << BP_CLKCTRL_DIS_LCDIF_DIV) & BM_CLKCTRL_DIS_LCDIF_DIV;
	__raw_writel(reg_val, clk->scale_reg);
	if (clk->busy_reg) {
		int i;
		for (i = 10000; i; i--)
			if (!clk_is_busy(clk))
				break;
		if (!i)
			return -ETIMEDOUT;
	}

	reg_val = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);
	reg_val |= BM_CLKCTRL_CLKSEQ_BYPASS_DIS_LCDIF;
	__raw_writel(reg_val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);

	return 0;
}

static int lcdif_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;
	if (clk->bypass_reg) {
		if (parent == clk->parent)
			return 0;
		if (parent == &ref_xtal_clk) {
			__raw_writel(1 << clk->bypass_bits,
				clk->bypass_reg + SET_REGISTER);
			ret = 0;
		}
		if (ret && (parent == &ref_pix_clk)) {
			__raw_writel(0 << clk->bypass_bits,
				clk->bypass_reg + CLR_REGISTER);
			ret = 0;
		}
		if (!ret)
			clk->parent = parent;
	}
	return ret;
}

static struct clk dis_lcdif_clk = {
	.parent = &pll_clk[0],
	.enable = mx28_raw_enable,
	.disable = mx28_raw_disable,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_DIS_LCDIF,
	.scale_bits = 0,
	.busy_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_DIS_LCDIF,
	.busy_bits = 29,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_DIS_LCDIF,
	.enable_bits = 31,
	.bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	.bypass_bits = 14,
	.get_rate = lcdif_get_rate,
	.set_rate = lcdif_set_rate,
	.set_parent = lcdif_set_parent,
	.flags = CPU_FREQ_TRIG_UPDATE,
};

static unsigned long hsadc_get_rate(struct clk *clk)
{
	unsigned int reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_HSADC);
	reg = (reg & BM_CLKCTRL_HSADC_FREQDIV) >> BP_CLKCTRL_HSADC_FREQDIV;
	return clk->parent->get_rate(clk->parent) / ((1 << reg) * 9);
}

static int hsadc_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg = clk->parent->get_rate(clk->parent);
	if ((reg / rate) % 9)
		return -EINVAL;
	reg = reg / 9;
	switch (reg) {
	case 1:
		reg = BM_CLKCTRL_HSADC_RESETB;
		break;
	case 2:
		reg = 1 | BM_CLKCTRL_HSADC_RESETB;
		break;
	case 4:
		reg = 2 | BM_CLKCTRL_HSADC_RESETB;
		break;
	case 8:
		reg = 3 | BM_CLKCTRL_HSADC_RESETB;
		break;
	default:
		return -EINVAL;
	}
	__raw_writel(reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_HSADC);
	return 0;
}

static unsigned long hsadc_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned int div;
	unsigned int reg = clk->parent->get_rate(clk->parent);
	div = ((reg / rate) + 8) / 9;
	if (div <= 1)
		return reg;
	if (div > 4)
		return reg >> 3;
	if (div > 2)
		return reg >> 2;
	return reg >> 1;
}

static struct clk hsadc_clk = {
	.parent = &ref_hsadc_clk,
	.get_rate = hsadc_get_rate,
	.set_rate = hsadc_set_rate,
	.round_rate = hsadc_round_rate,
};

static unsigned long gpmi_get_rate(struct clk *clk)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_GPMI) &
	    BM_CLKCTRL_GPMI_DIV;
	return clk->parent->get_rate(clk->parent) / reg;
}

static int gpmi_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;
	if (clk->bypass_reg) {
		if (parent == clk->parent)
			return 0;
		if (parent == &ref_xtal_clk) {
			__raw_writel(1 << clk->bypass_bits,
				clk->bypass_reg + SET_REGISTER);
			ret = 0;
		}
		if (ret && (parent == &ref_gpmi_clk)) {
			__raw_writel(0 << clk->bypass_bits,
				clk->bypass_reg + CLR_REGISTER);
			ret = 0;
		}
		if (!ret)
			clk->parent = parent;
	}
	return ret;
}

static unsigned long gpmi_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned int root_rate, frac_rate;
	unsigned int div;
	root_rate = clk->parent->get_rate(clk->parent);
	frac_rate = root_rate % rate;
	div = root_rate / rate;
	if ((div == 0) || (div >= 0x400))
		return root_rate;
	if (frac_rate == 0)
		return rate;
	return rate;
}

static int gpmi_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long root_rate;
	unsigned int reg, div;
	root_rate = clk->parent->get_rate(clk->parent);
	div = root_rate / rate;
	if ((div == 0) || (div >= 0x400))
		return -EINVAL;

	if (root_rate % rate) {
		div = mx28_get_frac_div(root_rate / 1000, rate / 1000, 0x3FF);
		if (((root_rate / 0x400) * div) > rate)
			return -EINVAL;
	}
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_GPMI);
	reg &= ~(BM_CLKCTRL_GPMI_DIV | BM_CLKCTRL_GPMI_DIV_FRAC_EN);
	if (root_rate % rate)
		reg |= BM_CLKCTRL_GPMI_DIV_FRAC_EN;
	reg |= BF_CLKCTRL_GPMI_DIV(div);
	__raw_writel(reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_GPMI);

	do {
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_GPMI);
	} while (reg & BM_CLKCTRL_GPMI_BUSY);
	return 0;
}

static struct clk gpmi_clk = {
	.parent = &ref_gpmi_clk,
	.set_parent = gpmi_set_parent,
	.get_rate = gpmi_get_rate,
	.set_rate = gpmi_set_rate,
	.round_rate = gpmi_round_rate,
	.enable = mx28_raw_enable,
	.disable = mx28_raw_disable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_GPMI,
	.enable_bits = BM_CLKCTRL_GPMI_CLKGATE,
	.bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	.bypass_bits = 2,
};

static unsigned long saif_get_rate(struct clk *clk);
static unsigned long saif_set_rate(struct clk *clk, unsigned int rate);
static unsigned long saif_set_parent(struct clk *clk, struct clk *parent);

static struct clk saif_clk[] = {
	{
	 .parent = &pll_clk[0],
	 .get_rate = saif_get_rate,
	 .enable = mx28_raw_enable,
	 .disable = mx28_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF0,
	 .enable_bits = BM_CLKCTRL_SAIF0_CLKGATE,
	 .scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF0,
	 .scale_bits = 0,
	 .busy_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF0,
	 .busy_bits = 29,
	 .bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	 .bypass_bits = 0,
	 .set_rate = saif_set_rate,
	 .set_parent = saif_set_parent,
	 },
	{
	 .parent = &pll_clk[0],
	 .get_rate = saif_get_rate,
	 .enable = mx28_raw_enable,
	 .disable = mx28_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF1,
	 .enable_bits = BM_CLKCTRL_SAIF1_CLKGATE,
	 .scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF1,
	 .scale_bits = 0,
	 .busy_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF1,
	 .busy_bits = 29,
	 .bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	 .bypass_bits = 1,
	 .set_rate = saif_set_rate,
	 .set_parent = saif_set_parent,
	 },
};

static unsigned long saif_get_rate(struct clk *clk)
{
	unsigned long reg, div;
	if (clk == saif_clk) {
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF0);
		div = reg & BM_CLKCTRL_SAIF0_DIV;
		reg &= BM_CLKCTRL_SAIF0_DIV_FRAC_EN;
	} else {
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF1);
		div = reg & BM_CLKCTRL_SAIF1_DIV;
		reg &= BM_CLKCTRL_SAIF1_DIV_FRAC_EN;
	}
	if (!reg)
		return clk->parent->get_rate(clk->parent) / div;
	return (clk->parent->get_rate(clk->parent) / 0x10000) * div;
}

static unsigned long saif_set_rate(struct clk *clk, unsigned int rate)
{
	u16 div = 0;
	u32 clkctrl_saif;
	u64 rates;
	struct clk *parent = clk->parent;

	pr_debug("%s: rate %d, parent rate %d\n", __func__, rate,
			clk_get_rate(parent));

	if (rate > clk_get_rate(parent))
		return -EINVAL;
	/*saif clock always use frac div*/
	rates = 65536 * (u64)rate;
	rates = rates + (u64)(clk_get_rate(parent) / 2);
	do_div(rates, clk_get_rate(parent));
	div = rates;

	pr_debug("%s: div calculated is %d\n", __func__, div);
	if (!div)
		return -EINVAL;

	clkctrl_saif = __raw_readl(clk->scale_reg);
	clkctrl_saif &= ~BM_CLKCTRL_SAIF0_DIV_FRAC_EN;
	clkctrl_saif &= ~BM_CLKCTRL_SAIF0_DIV;
	clkctrl_saif |= div;
	clkctrl_saif |= BM_CLKCTRL_SAIF0_DIV_FRAC_EN;
	clkctrl_saif &= ~BM_CLKCTRL_SAIF0_CLKGATE;
	__raw_writel(clkctrl_saif, clk->scale_reg);
	if (clk->busy_reg) {
		int i;
		for (i = 10000; i; i--)
			if (!clk_is_busy(clk))
				break;
		if (!i) {
			pr_err("couldn't set up SAIF clk divisor\n");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static unsigned long saif_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;
	int shift = 4;
	/*bypass*/
	if (parent == &pll_clk[0])
		shift = 8;
	if (clk->bypass_reg) {
		__raw_writel(1 << clk->bypass_bits, clk->bypass_reg + shift);
		ret = 0;
	}
	return ret;
}

static int saif_mclk_enable(struct clk *clk)
{
	/*Check if enabled already*/
	if (__raw_readl(clk->busy_reg) & clk->busy_bits)
		return 0;
	 /*Enable saif to enable mclk*/
	__raw_writel(0x1, clk->enable_reg);
	mdelay(1);
	__raw_writel(0x1, clk->enable_reg);
	mdelay(1);
	return 0;
}

static int saif_mclk_disable(struct clk *clk)
{
	/*Check if disabled already*/
	if (!(__raw_readl(clk->busy_reg) & clk->busy_bits))
		return 0;
	 /*Disable saif to disable mclk*/
	__raw_writel(0x0, clk->enable_reg);
	mdelay(1);
	__raw_writel(0x0, clk->enable_reg);
	mdelay(1);
	return 0;
}

static struct clk saif_mclk[] = {
	{
	 .parent = &saif_clk[0],
	 .enable = saif_mclk_enable,
	 .disable = saif_mclk_disable,
	 .enable_reg = SAIF0_CTRL,
	 .enable_bits = BM_SAIF_CTRL_RUN,
	 .busy_reg = SAIF0_STAT,
	 .busy_bits = BM_SAIF_STAT_BUSY,
	 },
	{
	 .parent = &saif_clk[1],
	 .enable = saif_mclk_enable,
	 .disable = saif_mclk_disable,
	 .enable_reg = SAIF1_CTRL,
	 .enable_bits = BM_SAIF_CTRL_RUN,
	 .busy_reg = SAIF1_STAT,
	 .busy_bits = BM_SAIF_STAT_BUSY,
	 },
};

static unsigned long pcmspdif_get_rate(struct clk *clk)
{
	return clk->parent->get_rate(clk->parent) / 4;
}

static struct clk pcmspdif_clk = {
	.parent = &pll_clk[0],
	.get_rate = pcmspdif_get_rate,
	.enable = mx28_raw_enable,
	.disable = mx28_raw_disable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SPDIF,
	.enable_bits = BM_CLKCTRL_SPDIF_CLKGATE,
};

/* usb_clk for usb0 */
static struct clk usb_clk0 = {
	.parent = &pll_clk[0],
	.enable = mx28_raw_enable,
	.disable = mx28_raw_disable,
	.enable_reg = DIGCTRL_BASE_ADDR + HW_DIGCTL_CTRL,
	.enable_bits = BM_DIGCTL_CTRL_USB0_CLKGATE,
	.flags = CPU_FREQ_TRIG_UPDATE,
};

/* usb_clk for usb1 */
static struct clk usb_clk1 = {
	.parent = &pll_clk[1],
	.enable = mx28_raw_enable,
	.disable = mx28_raw_disable,
	.enable_reg = DIGCTRL_BASE_ADDR + HW_DIGCTL_CTRL,
	.enable_bits = BM_DIGCTL_CTRL_USB1_CLKGATE,
	.flags = CPU_FREQ_TRIG_UPDATE,
};

/* usb phy clock for usb0 */
static struct clk usb_phy_clk0 = {
	.parent = &pll_clk[0],
	.enable = mx28_raw_disable, /* EN_USB_CLKS = 1 means ON */
	.disable = mx28_raw_enable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL0CTRL0_SET,
	.enable_bits = BM_CLKCTRL_PLL0CTRL0_EN_USB_CLKS,
	.flags = CPU_FREQ_TRIG_UPDATE,
};

/* usb phy clock for usb1 */
static struct clk usb_phy_clk1 = {
	.parent = &pll_clk[1],
	.enable = mx28_raw_disable,
	.disable = mx28_raw_enable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL1CTRL0_SET,
	.enable_bits = BM_CLKCTRL_PLL0CTRL0_EN_USB_CLKS,
	.flags = CPU_FREQ_TRIG_UPDATE,
};

static struct clk enet_out_clk = {
	.parent = &pll_clk[2],
	.enable = mx28_raw_enable,
	.disable = mx28_raw_disable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_ENET,
	.enable_bits = BM_CLKCTRL_ENET_DISABLE,
};

static struct clk_lookup onchip_clocks[] = {
	{
	 .con_id = "xtal.0",
	 .clk = &xtal_clk[0],
	 },
	{
	 .con_id = "xtal.1",
	 .clk = &xtal_clk[1],
	 },
	{
	 .con_id = "xtal.2",
	 .clk = &xtal_clk[2],
	 },
	{
	 .con_id = "pll.0",
	 .clk = &pll_clk[0],
	 },
	{
	 .con_id = "pll.1",
	 .clk = &pll_clk[1],
	 },
	{
	 .con_id = "pll.2",
	 .clk = &pll_clk[2],
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
	 .clk = &ref_io_clk[0],
	 },
	{
	 .con_id = "ref_io.1",
	 .clk = &ref_io_clk[1],
	 },
	{
	 .con_id = "ref_pix",
	 .clk = &ref_pix_clk,
	 },
	{
	 .con_id = "ref_hsadc",
	 .clk = &ref_hsadc_clk,
	 },
	{
	 .con_id = "ref_gpmi",
	 .clk = &ref_gpmi_clk,
	 },
	{
	 .con_id = "ana",
	 .clk = &ana_clk,
	 },
	{
	 .con_id = "rtc",
	 .clk = &rtc_clk,
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
	 .clk = &ssp_clk[0],
	 },
	{
	 .con_id = "ssp.1",
	 .clk = &ssp_clk[1],
	 },
	{
	 .con_id = "ssp.2",
	 .clk = &ssp_clk[2],
	 },
	{
	 .con_id = "ssp.3",
	 .clk = &ssp_clk[3],
	 },
	{
	 .con_id = "gpmi",
	 .clk = &gpmi_clk,
	 },
	{
	 .con_id = "spdif",
	 .clk = &pcmspdif_clk,
	 },
	{
	 .con_id = "saif.0",
	 .clk = &saif_clk[0],
	 },
	{
	 .con_id = "saif.1",
	 .clk = &saif_clk[1],
	 },
	{
	 .con_id = "emi",
	 .clk = &emi_clk,
	 },
	{
	 .con_id = "dis_lcdif",
	 .clk = &dis_lcdif_clk,
	 },
	{
	 .con_id = "hsadc",
	 .clk = &hsadc_clk,
	 },
	{
	 .con_id = "can_clk",
	 .dev_id = "FlexCAN.0",
	 .clk = &flexcan_clk[0],
	 },
	{
	 .con_id = "can_clk",
	 .dev_id = "FlexCAN.1",
	 .clk = &flexcan_clk[1],
	 },
	{
	.con_id = "usb_clk0",
	.clk = &usb_clk0,
	},
	{
	.con_id = "usb_clk1",
	.clk = &usb_clk1,
	},
	{
	.con_id = "usb_phy_clk0",
	.clk = &usb_phy_clk0,
	},
	{
	.con_id = "usb_phy_clk1",
	.clk = &usb_phy_clk1,
	},
	{
	.con_id = "fec_clk",
	.clk = &enet_out_clk,
	},
	{
	.con_id = "saif_mclk.0",
	.clk = &saif_mclk[0],
	},
	{
	.con_id = "saif_mclk.1",
	.clk = &saif_mclk[1],
	}
};

static void mx28_clock_scan(void)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_CPU)
		cpu_clk.parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_DIS_LCDIF)
		dis_lcdif_clk.parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_EMI)
		emi_clk.parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_SSP3)
		ssp_clk[3].parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_SSP2)
		ssp_clk[2].parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_SSP1)
		ssp_clk[1].parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_SSP0)
		ssp_clk[0].parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_GPMI)
		gpmi_clk.parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_SAIF1)
		saif_clk[1].parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_SAIF0)
		saif_clk[0].parent = &ref_xtal_clk;
};

void __init mx28_set_input_clk(unsigned long xtal0,
			       unsigned long xtal1,
			       unsigned long xtal2, unsigned long enet)
{
	xtal_clk_rate[0] = xtal0;
	xtal_clk_rate[1] = xtal1;
	xtal_clk_rate[2] = xtal2;
	enet_mii_phy_rate = enet;
}

void  mx28_enet_clk_hook(void)
{
	unsigned long reg;

	reg =  __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_ENET);

	reg &= ~BM_CLKCTRL_ENET_SLEEP;
	reg |= BM_CLKCTRL_ENET_CLK_OUT_EN;
	/* select clock for 1588 module */
	reg &= ~(BM_CLKCTRL_ENET_DIV_TIME | BM_CLKCTRL_ENET_TIME_SEL);
	reg |= BF_CLKCTRL_ENET_TIME_SEL(BV_CLKCTRL_ENET_TIME_SEL__PLL)
		| BF_CLKCTRL_ENET_DIV_TIME(12);

	__raw_writel(reg, CLKCTRL_BASE_ADDR + HW_CLKCTRL_ENET);
}

void __init mx28_clock_init(void)
{
	int i;
	mx28_clock_scan();
	mx28_enet_clk_hook();
	for (i = 0; i < ARRAY_SIZE(onchip_clocks); i++)
		clk_register(&onchip_clocks[i]);

	clk_enable(&cpu_clk);
	clk_enable(&emi_clk);

	clk_en_public_h_asm_ctrl(mx28_enable_h_autoslow,
		mx28_set_hbus_autoslow_flags);
}
