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

#define CLKCTRL_BASE_ADDR IO_ADDRESS(CLKCTRL_PHYS_ADDR)

/* external clock input */
static struct clk xtal_clk[];
static unsigned long xtal_clk_rate[3] = { 24000000, 24000000, 32000 };

static unsigned long enet_mii_phy_rate;

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
static struct clk pll_clk[] = {
	{
	 .parent = &ref_xtal_clk,
	 .flags = RATE_FIXED,
	 .get_rate = pll_get_rate,
	 .enable = pll_enable,
	 .disable = pll_disable,
	 },
	{
	 .parent = &ref_xtal_clk,
	 .flags = RATE_FIXED,
	 .get_rate = pll_get_rate,
	 .enable = pll_enable,
	 .disable = pll_disable,
	 },
	{
	 .parent = &ref_xtal_clk,
	 .flags = RATE_FIXED,
	 .get_rate = pll_get_rate,
	 .enable = pll_enable,
	 .disable = pll_disable,
	 }
};

static unsigned long pll_get_rate(struct clk *clk)
{
	if (clk == (pll_clk + 2))
		return 50000000;
	return 480000000;
}

static int pll_enable(struct clk *clk)
{
	int timeout = 100;
	unsigned long reg;
	switch (clk - pll_clk) {
	case 0:
		__raw_writel(BM_CLKCTRL_PLL0CTRL0_POWER |
			     BM_CLKCTRL_PLL0CTRL0_EN_USB_CLKS,
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
		__raw_writel(BM_CLKCTRL_PLL1CTRL0_POWER |
			     BM_CLKCTRL_PLL1CTRL0_EN_USB_CLKS,
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
		__raw_writel(BM_CLKCTRL_PLL0CTRL0_POWER |
			     BM_CLKCTRL_PLL0CTRL0_EN_USB_CLKS,
			     CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL0CTRL0_CLR);
		return;
	case 1:
		__raw_writel(BM_CLKCTRL_PLL1CTRL0_POWER |
			     BM_CLKCTRL_PLL1CTRL0_EN_USB_CLKS,
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
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	.enable_bits = BM_CLKCTRL_FRAC0_CLKGATECPU,
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
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	.enable_bits = BM_CLKCTRL_FRAC0_CLKGATEEMI,
};

static unsigned long ref_io_get_rate(struct clk *clk);
static struct clk ref_io_clk[] = {
	{
	 .parent = &pll_clk[0],
	 .get_rate = ref_io_get_rate,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	 .enable_bits = BM_CLKCTRL_FRAC0_CLKGATEIO0,
	 },
	{
	 .parent = &pll_clk[0],
	 .get_rate = ref_io_get_rate,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC0,
	 .enable_bits = BM_CLKCTRL_FRAC0_CLKGATEIO1,
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
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1,
	.enable_bits = BM_CLKCTRL_FRAC1_CLKGATEPIX,
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
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1,
	.enable_bits = BM_CLKCTRL_FRAC1_CLKGATEHSADC,
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
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1,
	.enable_bits = BM_CLKCTRL_FRAC1_CLKGATEGPMI,
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
	.enable = mx28_raw_enable,
	.disable = mx28_raw_disable,
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
			__raw_writel(1 << clk->bypass_bits,
					clk->bypass_reg + SET_REGISTER);
		else
			__raw_writel(0 << clk->bypass_bits,
					clk->bypass_reg + CLR_REGISTER);

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
	 },
	{
	 .parent = &ref_io_clk[1],
	 .get_rate = ssp_get_rate,
	 .enable = mx28_raw_enable,
	 .disable = mx28_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP3,
	 .enable_bits = BM_CLKCTRL_SSP3_CLKGATE,
	 },
};

static unsigned long ssp_get_rate(struct clk *clk)
{
	unsigned int reg;
	switch (clk - ssp_clk) {
	case 0:
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP0) &
		    BM_CLKCTRL_SSP0_DIV;
		break;
	case 1:
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP1) &
		    BM_CLKCTRL_SSP1_DIV;
		break;
	case 2:
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP2) &
		    BM_CLKCTRL_SSP2_DIV;
		break;
	case 3:
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SSP3) &
		    BM_CLKCTRL_SSP3_DIV;
		break;
	default:
		return 0;
	}
	return clk->parent->get_rate(clk->parent) / reg;
}

static unsigned long lcdif_get_rate(struct clk *clk)
{
	long rate = clk->parent->get_rate(clk->parent);
	long div;

	div = (__raw_readl(clk->scale_reg) >> clk->scale_bits) &
	    BM_CLKCTRL_DIS_LCDIF_DIV;
	if (div)
		rate /= div;

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

static struct clk gpmi_clk = {
	.parent = &ref_gpmi_clk,
	.get_rate = gpmi_get_rate,
	.enable = mx28_raw_enable,
	.disable = mx28_raw_disable,
	.enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_GPMI,
	.enable_bits = BM_CLKCTRL_GPMI_CLKGATE,
};

static unsigned long saif_get_rate(struct clk *clk);
static struct clk saif_clk[] = {
	{
	 .parent = &pll_clk[0],
	 .get_rate = saif_get_rate,
	 .enable = mx28_raw_enable,
	 .disable = mx28_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF0,
	 .enable_bits = BM_CLKCTRL_SAIF0_CLKGATE,
	 },
	{
	 .parent = &pll_clk[0],
	 .get_rate = saif_get_rate,
	 .enable = mx28_raw_enable,
	 .disable = mx28_raw_disable,
	 .enable_reg = CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF1,
	 .enable_bits = BM_CLKCTRL_SAIF1_CLKGATE,
	 },
};

static unsigned long saif_get_rate(struct clk *clk)
{
	unsigned long reg;
	if (clk == saif_clk)
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF0) &
		    BM_CLKCTRL_SAIF0_DIV;
	else
		reg = __raw_readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_SAIF1) &
		    BM_CLKCTRL_SAIF1_DIV;
	return clk->parent->get_rate(clk->parent) / reg;
}

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
	 .con_id = "flexcan.0",
	 .clk = &flexcan_clk[0],
	 },
	{
	 .con_id = "flexcan.1",
	 .clk = &flexcan_clk[1],
	 },
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

void __init mx28_clock_init(void)
{
	int i;
	mx28_clock_scan();
	for (i = 0; i < ARRAY_SIZE(onchip_clocks); i++)
		clk_register(&onchip_clocks[i]);

	clk_enable(&cpu_clk);
	clk_enable(&emi_clk);
}
