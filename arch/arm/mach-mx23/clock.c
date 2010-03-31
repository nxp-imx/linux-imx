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
#include <linux/platform_device.h>

#include <mach/clock.h>

#include "regs-clkctrl.h"
#include "regs-digctl.h"

#include "mach/mx23.h"

#define CLKCTRL_BASE_ADDR IO_ADDRESS(CLKCTRL_PHYS_ADDR)
#define DIGCTRL_BASE_ADDR IO_ADDRESS(DIGCTL_PHYS_ADDR)

/* external clock input */
static struct clk xtal_clk[];
static unsigned long xtal_clk_rate[3] = { 24000000, 24000000, 32000 };

static unsigned long enet_mii_phy_rate;

static int mx23_raw_enable(struct clk *clk)
{
	unsigned int reg;
	if (clk->enable_reg) {
		reg = __raw_readl(clk->enable_reg);
		reg &= ~clk->enable_bits;
		__raw_writel(reg, clk->enable_reg);
	}
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
	int timeout = 100;
	unsigned long reg;

	__raw_writel(BM_CLKCTRL_PLLCTRL0_POWER |
			     BM_CLKCTRL_PLLCTRL0_EN_USB_CLKS,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLLCTRL0_SET);
	do {
		udelay(10);
		reg = __raw_readl(CLKCTRL_BASE_ADDR +
				  HW_CLKCTRL_PLLCTRL1);
		timeout--;
	} while ((timeout > 0) && !(reg & BM_CLKCTRL_PLLCTRL1_LOCK));
	if (timeout <= 0)
		return -EFAULT;
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

static unsigned long ref_cpu_get_rate(struct clk *clk)
{
	unsigned int reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC) &
	    BM_CLKCTRL_FRAC_CPUFRAC;
	return ref_clk_get_rate(clk->parent->get_rate(clk->parent), reg);
}

static struct clk ref_cpu_clk = {
	.parent = &pll_clk,
	.get_rate = ref_cpu_get_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.enable_bits = BM_CLKCTRL_FRAC_CLKGATECPU,
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
	.get_rate = ref_emi_get_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.enable_bits = BM_CLKCTRL_FRAC_CLKGATEEMI,
};

static unsigned long ref_io_get_rate(struct clk *clk);
static struct clk ref_io_clk = {
	 .parent = &pll_clk,
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
	.get_rate = ref_pix_get_rate,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.enable_bits = BM_CLKCTRL_FRAC_CLKGATEPIX,
};

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

static int lcdif_set_rate(struct clk *clk, u32 rate)
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
	reg_val = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);
	reg_val &= ~BM_CLKCTRL_CLKSEQ_BYPASS_PIX;
	__raw_writel(reg_val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);

out:
	return ret;
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
			__raw_writel(1 << clk->bypass_bits,
				clk->bypass_reg + CLR_REGISTER);
			ret = 0;
		}
		if (!ret)
			clk->parent = parent;
	}
	return ret;
}

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
	.set_parent	= lcdif_set_parent,
};

static struct clk cpu_clk, h_clk;
static int clkseq_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = -EINVAL;
	int shift = 8;

	/* bypass? */
	if (parent == &ref_xtal_clk)
		shift = 4;

	if (clk->bypass_reg) {
		u32 hbus_val, cpu_val;

		if (clk == &cpu_clk && shift == 4) {
			hbus_val = __raw_readl(CLKCTRL_BASE_ADDR +
					HW_CLKCTRL_HBUS);
			cpu_val = __raw_readl(CLKCTRL_BASE_ADDR +
					HW_CLKCTRL_CPU);

			hbus_val &= ~(BM_CLKCTRL_HBUS_DIV_FRAC_EN |
				      BM_CLKCTRL_HBUS_DIV);
			hbus_val |= 1;

			cpu_val &= ~BM_CLKCTRL_CPU_DIV_CPU;
			cpu_val |= 1;

			__raw_writel(1 << clk->bypass_bits,
					clk->bypass_reg + shift);

			__raw_writel(hbus_val,
					CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);
			__raw_writel(cpu_val,
					CLKCTRL_BASE_ADDR + HW_CLKCTRL_CPU);
			/* h_clk.rate = 0; */
		} else if (clk == &cpu_clk && shift == 8) {
			hbus_val = __raw_readl(CLKCTRL_BASE_ADDR +
							HW_CLKCTRL_HBUS);
			cpu_val = __raw_readl(CLKCTRL_BASE_ADDR +
							HW_CLKCTRL_CPU);
			hbus_val &= ~(BM_CLKCTRL_HBUS_DIV_FRAC_EN |
				      BM_CLKCTRL_HBUS_DIV);
			hbus_val |= 2;
			cpu_val &= ~BM_CLKCTRL_CPU_DIV_CPU;
			cpu_val |= 2;

			__raw_writel(hbus_val,
				CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS);
			__raw_writel(cpu_val,
				CLKCTRL_BASE_ADDR + HW_CLKCTRL_CPU);
			/*	h_clk.rate = 0; */

			__raw_writel(1 << clk->bypass_bits,
					clk->bypass_reg + shift);
		} else
			__raw_writel(1 << clk->bypass_bits,
					clk->bypass_reg + shift);
		ret = 0;
	}

	return ret;
}

static unsigned long cpu_get_rate(struct clk *clk)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_CPU);
	if (clk->parent == &ref_cpu_clk)
		reg = (reg & BM_CLKCTRL_CPU_DIV_CPU) >> BP_CLKCTRL_CPU_DIV_CPU;
	else
		reg = (reg & BM_CLKCTRL_CPU_DIV_XTAL) >>
		    BP_CLKCTRL_CPU_DIV_XTAL;
	return clk->parent->get_rate(clk->parent) / reg;
}

static struct clk cpu_clk = {
	.parent = &ref_cpu_clk,
	.scale_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC,
	.scale_bits	= 0,
	.bypass_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	.bypass_bits	= 7,
	.busy_reg	= CLKCTRL_BASE_ADDR + HW_CLKCTRL_CPU,
	.busy_bits	= 28,
/*	.flags		= RATE_PROPAGATES | ENABLED,*/
	.get_rate = cpu_get_rate,
	.set_parent = clkseq_set_parent,
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

static struct clk x_clk = {
	.parent = &ref_xtal_clk,
	.get_rate = x_get_rate,
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

static unsigned long h_get_rate(struct clk *clk)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_HBUS) &
	    BM_CLKCTRL_HBUS_DIV;
	return clk->parent->get_rate(clk->parent) / reg;
}

static struct clk h_clk = {
	.parent = &cpu_clk,
	.get_rate = h_get_rate,
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

static struct clk emi_clk = {
	.parent = &ref_emi_clk,
	.get_rate = emi_get_rate,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_EMI,
	.enable_bits = BM_CLKCTRL_EMI_CLKGATE,
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
	 .bypass_bits = 3,
	 .set_rate = ssp_set_rate,
	 .set_parent = ssp_set_parent,
};

static unsigned long ssp_get_rate(struct clk *clk)
{
	unsigned int reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP) &
		    BM_CLKCTRL_SSP_DIV;

	return clk->parent->get_rate(clk->parent) / reg;
}

/* usb_clk for usb0 */
static struct clk usb_clk = {
	.parent = &pll_clk,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.enable_reg = DIGCTRL_BASE_ADDR + HW_DIGCTL_CTRL,
	.enable_bits = BM_DIGCTL_CTRL_USB_CLKGATE,
};

static struct clk audio_clk = {
	.parent = &ref_xtal_clk,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_XTAL,
	.enable_bits = BM_CLKCTRL_XTAL_FILT_CLK24M_GATE,
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
	}
};


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
};


void __init mx23_set_input_clk(unsigned long xtal0,
			       unsigned long xtal1,
			       unsigned long xtal2, unsigned long enet)
{
	xtal_clk_rate[0] = xtal0;
	xtal_clk_rate[1] = xtal1;
	xtal_clk_rate[2] = xtal2;
}

void __init mx23_clock_init(void)
{
	int i;
	mx23_clock_scan();
	for (i = 0; i < ARRAY_SIZE(onchip_clocks); i++)
		clk_register(&onchip_clocks[i]);

	clk_enable(&cpu_clk);
	clk_enable(&emi_clk);
}
