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
	.get_rate = cpu_get_rate,
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

static unsigned long lcdif_get_rate(struct clk *clk)
{
	long rate = clk->parent->get_rate(clk->parent);
	long div;

	div = (__raw_readl(clk->scale_reg) >> clk->scale_bits) &
	    BM_CLKCTRL_PIX_DIV;
	if (div)
		rate /= div;

	return rate;
}

static int lcdif_set_rate(struct clk *clk, unsigned long rate)
{
	int reg_val;

	reg_val = __raw_readl(clk->scale_reg);
	reg_val &= ~(BM_CLKCTRL_PIX_DIV | BM_CLKCTRL_PIX_CLKGATE);
	reg_val |= (1 << BP_CLKCTRL_PIX_DIV) & BM_CLKCTRL_PIX_DIV;
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
	reg_val |= BM_CLKCTRL_CLKSEQ_BYPASS_PIX;
	__raw_writel(reg_val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);

	return 0;
}

static struct clk dis_lcdif_clk = {
	.parent = &pll_clk,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.scale_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_PIX,
	.scale_bits = 0,
	.busy_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_PIX,
	.busy_bits = 29,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_PIX,
	.enable_bits = 31,
	.bypass_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ,
	.bypass_bits = 14,
	.get_rate = lcdif_get_rate,
	.set_rate = lcdif_set_rate,
};

/* usb_clk for usb0 */
static struct clk usb_clk = {
	.parent = &pll_clk,
	.enable = mx23_raw_enable,
	.disable = mx23_raw_disable,
	.enable_reg = DIGCTRL_BASE_ADDR + HW_DIGCTL_CTRL,
	.enable_bits = BM_DIGCTL_CTRL_USB_CLKGATE,
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
	}
};


static void mx23_clock_scan(void)
{
	unsigned long reg;
	reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_CPU)
		cpu_clk.parent = &ref_xtal_clk;
	if (reg & BM_CLKCTRL_CLKSEQ_BYPASS_PIX)
		dis_lcdif_clk.parent = &ref_xtal_clk;
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
