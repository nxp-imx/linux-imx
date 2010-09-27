/*
 * Copyright (C) 2009 by Sascha Hauer, Pengutronix
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc.
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
 * MA 02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/clkdev.h>

#include <mach/clock.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/mx25.h>
#include "crm_regs.h"

#define CRM_BASE	MX25_IO_ADDRESS(MX25_CRM_BASE_ADDR)

#define CCM_MPCTL	0x00
#define CCM_UPCTL	0x04
#define CCM_CCTL	0x08
#define CCM_CGCR0	0x0C
#define CCM_CGCR1	0x10
#define CCM_CGCR2	0x14
#define CCM_PCDR0	0x18
#define CCM_PCDR1	0x1C
#define CCM_PCDR2	0x20
#define CCM_PCDR3	0x24
#define CCM_RCSR	0x28
#define CCM_CRDR	0x2C
#define CCM_DCVR0	0x30
#define CCM_DCVR1	0x34
#define CCM_DCVR2	0x38
#define CCM_DCVR3	0x3c
#define CCM_LTR0	0x40
#define CCM_LTR1	0x44
#define CCM_LTR2	0x48
#define CCM_LTR3	0x4c

#define OSC24M_CLK_FREQ     24000000	/* 24M reference clk */
#define OSC32K_CLK_FREQ     32768	/* 32.768k oscillator in */

#if defined CONFIG_CPU_FREQ_IMX
#define AHB_CLK_DEFAULT 133000000
#define ARM_SRC_DEFAULT 532000000
#endif

static struct clk mpll_clk;
static struct clk upll_clk;
static struct clk ahb_clk;
static struct clk upll_24610k_clk;
int cpu_wp_nr;

static int clk_cgcr_enable(struct clk *clk);
static void clk_cgcr_disable(struct clk *clk);
static unsigned long get_rate_ipg(struct clk *clk);

static int _clk_upll_enable(struct clk *clk)
{
	unsigned long reg;

	reg = __raw_readl(MXC_CCM_CCTL);
	reg &= ~MXC_CCM_CCTL_UPLL_DISABLE;
	__raw_writel(reg, MXC_CCM_CCTL);

	while ((__raw_readl(MXC_CCM_UPCTL) & MXC_CCM_UPCTL_LF) == 0)
		;

	return 0;
}

static void _clk_upll_disable(struct clk *clk)
{
	unsigned long reg;

	reg = __raw_readl(MXC_CCM_CCTL);
	reg |= MXC_CCM_CCTL_UPLL_DISABLE;
	__raw_writel(reg, MXC_CCM_CCTL);
}

static int _perclk_enable(struct clk *clk)
{
	unsigned long reg;

	reg = __raw_readl(MXC_CCM_CGCR0);
	reg |= 1 << clk->id;
	__raw_writel(reg, MXC_CCM_CGCR0);

	return 0;
}

static void _perclk_disable(struct clk *clk)
{
	unsigned long reg;

	reg = __raw_readl(MXC_CCM_CGCR0);
	reg &= ~(1 << clk->id);
	__raw_writel(reg, MXC_CCM_CGCR0);
}

static unsigned long _clk_osc24m_get_rate(struct clk *clk)
{
	return OSC24M_CLK_FREQ;
}

static unsigned long _clk_osc32k_get_rate(struct clk *clk)
{
	return OSC32K_CLK_FREQ;
}

static unsigned long _clk_pll_get_rate(struct clk *clk)
{
	unsigned long mfi = 0, mfn = 0, mfd = 0, pdf = 0;
	unsigned long ref_clk;
	unsigned long reg;
	unsigned long long temp;

	ref_clk = clk_get_rate(clk->parent);

	if (clk == &mpll_clk) {
		reg = __raw_readl(MXC_CCM_MPCTL);
		pdf = (reg & MXC_CCM_MPCTL_PD_MASK) >> MXC_CCM_MPCTL_PD_OFFSET;
		mfd =
		    (reg & MXC_CCM_MPCTL_MFD_MASK) >> MXC_CCM_MPCTL_MFD_OFFSET;
		mfi =
		    (reg & MXC_CCM_MPCTL_MFI_MASK) >> MXC_CCM_MPCTL_MFI_OFFSET;
		mfn =
		    (reg & MXC_CCM_MPCTL_MFN_MASK) >> MXC_CCM_MPCTL_MFN_OFFSET;
	} else if (clk == &upll_clk) {
		reg = __raw_readl(MXC_CCM_UPCTL);
		pdf = (reg & MXC_CCM_UPCTL_PD_MASK) >> MXC_CCM_UPCTL_PD_OFFSET;
		mfd =
		    (reg & MXC_CCM_UPCTL_MFD_MASK) >> MXC_CCM_UPCTL_MFD_OFFSET;
		mfi =
		    (reg & MXC_CCM_UPCTL_MFI_MASK) >> MXC_CCM_UPCTL_MFI_OFFSET;
		mfn =
		    (reg & MXC_CCM_UPCTL_MFN_MASK) >> MXC_CCM_UPCTL_MFN_OFFSET;
	} else {
		BUG();		/* oops */
	}

	mfi = (mfi <= 5) ? 5 : mfi;
	temp = 2LL * ref_clk * mfn;
	do_div(temp, mfd + 1);
	temp = 2LL * ref_clk * mfi + temp;
	do_div(temp, pdf + 1);

	return temp;
}

static unsigned long _clk_cpu_round_rate(struct clk *clk, unsigned long rate)
{
	int div = clk_get_rate(clk->parent) / rate;

	if (clk_get_rate(clk->parent) % rate)
		div++;

	if (div > 4)
		div = 4;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_cpu_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long div = 0x0, reg = 0x0;
	unsigned long cctl = __raw_readl(MXC_CCM_CCTL);

#if defined CONFIG_CPU_FREQ_IMX
	struct cpu_wp *cpu_wp;
	unsigned long ahb_clk_div = 0;
	unsigned long arm_src = 0;
	int i;

	cpu_wp = get_cpu_wp(&cpu_wp_nr);
	for (i = 0; i < cpu_wp_nr; i++) {
		if (cpu_wp[i].cpu_rate == rate) {
			div = cpu_wp[i].cpu_podf;
			ahb_clk_div = cpu_wp[i].cpu_rate / AHB_CLK_DEFAULT - 1;
			arm_src =
			    (cpu_wp[i].pll_rate == ARM_SRC_DEFAULT) ? 0 : 1;
			break;
		}
	}
	if (i == cpu_wp_nr)
		return -EINVAL;
	reg = (cctl & ~MXC_CCM_CCTL_ARM_MASK) |
	    (div << MXC_CCM_CCTL_ARM_OFFSET);
	reg = (reg & ~MXC_CCM_CCTL_AHB_MASK) |
	    (ahb_clk_div << MXC_CCM_CCTL_AHB_OFFSET);
	reg = (reg & ~MXC_CCM_CCTL_ARM_SRC) |
	    (arm_src << MXC_CCM_CCTL_ARM_SRC_OFFSET);
	__raw_writel(reg, MXC_CCM_CCTL);
#else
	div = clk_get_rate(clk->parent) / rate;

	if (div > 4 || div < 1 || ((clk_get_rate(clk->parent) / div) != rate))
		return -EINVAL;
	div--;

	reg =
	    (cctl & ~MXC_CCM_CCTL_ARM_MASK) | (div << MXC_CCM_CCTL_ARM_OFFSET);
	__raw_writel(reg, MXC_CCM_CCTL);
#endif

	return 0;
}

static unsigned long _clk_cpu_get_rate(struct clk *clk)
{
	unsigned long div;
	unsigned long cctl = __raw_readl(MXC_CCM_CCTL);
	unsigned long rate;

	div = (cctl & MXC_CCM_CCTL_ARM_MASK) >> MXC_CCM_CCTL_ARM_OFFSET;

	rate = clk_get_rate(clk->parent) / (div + 1);

	if (cctl & MXC_CCM_CCTL_ARM_SRC) {
		rate *= 3;
		rate /= 4;
	}

	return rate;
}

static unsigned long _clk_ahb_get_rate(struct clk *clk)
{
	unsigned long div;
	unsigned long cctl = __raw_readl(MXC_CCM_CCTL);

	div = (cctl & MXC_CCM_CCTL_AHB_MASK) >> MXC_CCM_CCTL_AHB_OFFSET;

	return clk_get_rate(clk->parent) / (div + 1);
}

static void *pcdr_a[4] = {
	MXC_CCM_PCDR0, MXC_CCM_PCDR1, MXC_CCM_PCDR2, MXC_CCM_PCDR3
};
static unsigned long _clk_perclkx_get_rate(struct clk *clk)
{
	unsigned long perclk_pdf;
	unsigned long pcdr;

	pcdr = __raw_readl(pcdr_a[clk->id >> 2]);

	perclk_pdf =
	    (pcdr >> ((clk->id & 3) << 3)) & MXC_CCM_PCDR1_PERDIV1_MASK;

	return clk_get_rate(clk->parent) / (perclk_pdf + 1);
}

static unsigned long _clk_perclkx_round_rate(struct clk *clk,
					     unsigned long rate)
{
	unsigned long div;
	unsigned long parent_rate;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;
	if (parent_rate % rate)
		div++;

	if (div > 64)
		div = 64;

	return parent_rate / div;
}

static int _clk_perclkx_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long reg;
	unsigned long div;
	unsigned long parent_rate;

	if (clk->id < 0 || clk->id > 15)
		return -EINVAL;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;
	if (div > 64 || div < 1 || ((parent_rate / div) != rate))
		return -EINVAL;
	div--;

	reg =
	    __raw_readl(pcdr_a[clk->id >> 2]) & ~(MXC_CCM_PCDR1_PERDIV1_MASK <<
						  ((clk->id & 3) << 3));
	reg |= div << ((clk->id & 3) << 3);
	__raw_writel(reg, pcdr_a[clk->id >> 2]);

	return 0;
}

static int _clk_perclkx_set_parent(struct clk *clk, struct clk *parent)
{
	unsigned long mcr;

	if (parent != &upll_clk && parent != &ahb_clk)
		return -EINVAL;

	clk->parent = parent;
	mcr = __raw_readl(MXC_CCM_MCR);
	if (parent == &upll_clk)
		mcr |= (1 << clk->id);
	else
		mcr &= ~(1 << clk->id);

	__raw_writel(mcr, MXC_CCM_MCR);

	return 0;
}

static int _clk_perclkx_set_parent3(struct clk *clk, struct clk *parent)
{
	unsigned long mcr = __raw_readl(MXC_CCM_MCR);
	int bit;

	if (parent != &upll_clk && parent != &ahb_clk &&
	    parent != &upll_24610k_clk)
		return -EINVAL;

	switch (clk->id) {
	case 2:
		bit = MXC_CCM_MCR_ESAI_CLK_MUX_OFFSET;
		break;
	case 13:
		bit = MXC_CCM_MCR_SSI1_CLK_MUX_OFFSET;
		break;
	case 14:
		bit = MXC_CCM_MCR_SSI2_CLK_MUX_OFFSET;
		break;
	default:
		return -EINVAL;
	}

	if (parent == &upll_24610k_clk) {
		mcr |= 1 << bit;
		__raw_writel(mcr, MXC_CCM_MCR);
		clk->parent = parent;
	} else {
		mcr &= ~(1 << bit);
		__raw_writel(mcr, MXC_CCM_MCR);
		return _clk_perclkx_set_parent(clk, parent);
	}

	return 0;
}

static unsigned long _clk_ipg_get_rate(struct clk *clk)
{
	return clk_get_rate(clk->parent) / 2;	/* Always AHB / 2 */
}

static unsigned long _clk_parent_round_rate(struct clk *clk, unsigned long rate)
{
	return clk->parent->round_rate(clk->parent, rate);
}

static int _clk_parent_set_rate(struct clk *clk, unsigned long rate)
{
	return clk->parent->set_rate(clk->parent, rate);
}

/* Top-level clocks */

static struct clk osc24m_clk = {
	.get_rate = _clk_osc24m_get_rate,
	.flags = RATE_PROPAGATES,
};

static struct clk osc32k_clk = {
	.get_rate = _clk_osc32k_get_rate,
	.flags = RATE_PROPAGATES,
};

static struct clk mpll_clk = {
	.parent = &osc24m_clk,
	.get_rate = _clk_pll_get_rate,
	.flags = RATE_PROPAGATES,
};

static struct clk upll_clk = {
	.parent = &osc24m_clk,
	.get_rate = _clk_pll_get_rate,
	.enable = _clk_upll_enable,
	.disable = _clk_upll_disable,
	.flags = RATE_PROPAGATES,
};

static unsigned long _clk_24610k_get_rate(struct clk *clk)
{
	long long temp = clk_get_rate(clk->parent) * 2461LL;

	do_div(temp, 24000);

	return temp;	/* Always (UPLL * 24.61 / 240) */
}

static struct clk upll_24610k_clk = {
	.parent = &upll_clk,
	.get_rate = _clk_24610k_get_rate,
	.flags = RATE_PROPAGATES,
};

/* Mid-level clocks */

static struct clk cpu_clk = {	/* ARM clock */
	.parent = &mpll_clk,
	.set_rate = _clk_cpu_set_rate,
	.get_rate = _clk_cpu_get_rate,
	.round_rate = _clk_cpu_round_rate,
	.flags = RATE_PROPAGATES,
};

static struct clk ahb_clk = {	/* a.k.a. HCLK */
	.parent = &cpu_clk,
	.get_rate = _clk_ahb_get_rate,
	.flags = RATE_PROPAGATES,
};

static struct clk ipg_clk = {
	.parent = &ahb_clk,
	.get_rate = _clk_ipg_get_rate,
	.flags = RATE_PROPAGATES,
};


/* Bottom-level clocks */

struct clk usb_ahb_clk = {
	.id = 0,
	.parent = &ahb_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR0,
	.enable_shift = MXC_CCM_CGCR0_HCLK_USBOTG_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk rtic_clk = {
	.id = 0,
	.parent = &ahb_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR0,
	.enable_shift = MXC_CCM_CGCR0_HCLK_RTIC_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk emi_clk = {
	.id = 0,
	.parent = &ahb_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR0,
	.enable_shift = MXC_CCM_CGCR0_HCLK_EMI_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk brom_clk = {
	.id = 0,
	.parent = &ahb_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR0,
	.enable_shift = MXC_CCM_CGCR0_HCLK_BROM_OFFSET,
	.disable = clk_cgcr_disable,
};

static struct clk per_clk[] = {
	/* per_csi_clk */
	{
	 .id = 0,
	 .parent = &upll_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_epit_clk */
	{
	 .id = 1,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_esai_clk */
	{
	 .id = 2,
	 .parent = &ahb_clk,	/* can be AHB or UPLL or 24.61MHz */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent3,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_esdhc1_clk */
	{
	 .id = 3,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_esdhc2_clk */
	{
	 .id = 4,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_gpt_clk */
	{
	 .id = 5,
	 .parent = &ahb_clk,	/* Must be AHB */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_i2c_clk */
	{
	 .id = 6,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_lcdc_clk */
	{
	 .id = 7,
	 .parent = &upll_clk,	/* Must be UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_nfc_clk */
	{
	 .id = 8,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_owire_clk */
	{
	 .id = 9,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_pwm_clk */
	{
	 .id = 10,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_sim1_clk */
	{
	 .id = 11,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_sim2_clk */
	{
	 .id = 12,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_ssi1_clk */
	{
	 .id = 13,
	 .parent = &ahb_clk,	/* can be AHB or UPLL or 24.61MHz */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent3,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_ssi2_clk */
	{
	 .id = 14,
	 .parent = &ahb_clk,	/* can be AHB or UPLL or 24.61MHz */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent3,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,
	},
	/* per_uart_clk */
	{
	 .id = 15,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .get_rate = _clk_perclkx_get_rate,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
};

struct clk nfc_clk = {
	.id = 0,
	.parent = &per_clk[8],
};

struct clk audmux_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_AUDMUX_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk ata_clk[] = {
	{
	/* ata_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_ATA_OFFSET,
	 .disable = clk_cgcr_disable,
	 .secondary = &ata_clk[1],},
	{
	/* ata_ahb_clk */
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_ATA_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk can_clk[] = {
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CAN1_OFFSET,
	 .disable = clk_cgcr_disable,},
	{
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CAN2_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk csi_clk[] = {
	{
	/* csi_clk */
	 .id = 0,
	 .parent = &per_clk[0],
	 .secondary = &csi_clk[1],},
	{
	/* csi_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CSI_OFFSET,
	 .disable = clk_cgcr_disable,
	 .secondary = &csi_clk[2],},
	{
	/* csi_ahb_clk */
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_CSI_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk cspi_clk[] = {
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CSPI1_OFFSET,
	 .get_rate = get_rate_ipg,
	 .disable = clk_cgcr_disable,},
	{
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CSPI2_OFFSET,
	 .get_rate = get_rate_ipg,
	 .disable = clk_cgcr_disable,},
	{
	 .id = 2,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CSPI3_OFFSET,
	 .get_rate = get_rate_ipg,
	 .disable = clk_cgcr_disable,},
};

struct clk dryice_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_DRYICE_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk ect_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_ECT_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk epit1_clk[] = {
	{
	/* epit_clk */
	 .id = 0,
	 .parent = &per_clk[1],
	 .secondary = &epit1_clk[1],},
	{
	/* epit_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_EPIT1_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk epit2_clk[] = {
	{
	/* epit_clk */
	 .id = 1,
	 .parent = &per_clk[1],
	 .secondary = &epit2_clk[1],},
	{
	/* epit_ipg_clk */
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_EPIT2_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk esai_clk[] = {
	{
	/* esai_clk */
	 .id = 0,
	 .parent = &per_clk[2],
	 .secondary = &esai_clk[1],},
	{
	/* esai_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_ESAI_OFFSET,
	 .disable = clk_cgcr_disable,
	 .secondary = &esai_clk[2],},
	{
	/* esai_ahb_clk */
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_ESAI_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk esdhc1_clk[] = {
	{
	/* esdhc_clk */
	 .id = 0,
	 .parent = &per_clk[3],
	 .secondary = &esdhc1_clk[1],},
	{
	/* esdhc_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_ESDHC1_OFFSET,
	 .disable = clk_cgcr_disable,
	 .secondary = &esdhc1_clk[2],},
	{
	/* esdhc_ahb_clk */
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_ESDHC1_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk esdhc2_clk[] = {
	{
	/* esdhc_clk */
	 .id = 1,
	 .parent = &per_clk[4],
	 .secondary = &esdhc2_clk[1],},
	{
	/* esdhc_ipg_clk */
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_ESDHC2_OFFSET,
	 .disable = clk_cgcr_disable,
	 .secondary = &esdhc2_clk[2],},
	{
	/* esdhc_ahb_clk */
	 .id = 1,
	 .parent = &ahb_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_ESDHC2_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk fec_clk[] = {
	{
	/* fec_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_FEC_OFFSET,
	 .disable = clk_cgcr_disable,
	 .secondary = &fec_clk[1],},
	{
	/* fec_ahb_clk */
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_FEC_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk gpio_clk[] = {
	{
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPIO1_OFFSET,
	 .disable = clk_cgcr_disable,},
	{
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPIO2_OFFSET,
	 .disable = clk_cgcr_disable,},
	{
	 .id = 2,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPIO3_OFFSET,
	 .disable = clk_cgcr_disable,},
};

static struct clk gpt1_clk[] = {
	{
	/* gpt_clk */
	 .id = 0,
	 .parent = &per_clk[5],
	 .secondary = &gpt1_clk[1],},
	{
	/* gpt_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPT1_OFFSET,
	 .disable = clk_cgcr_disable,},
};

static struct clk gpt2_clk[] = {
	{
	/* gpt_clk */
	 .id = 1,
	 .parent = &per_clk[5],
	 .secondary = &gpt1_clk[1],},
	{
	/* gpt_ipg_clk */
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPT2_OFFSET,
	 .disable = clk_cgcr_disable,},
};

static struct clk gpt3_clk[] = {
	{
	/* gpt_clk */
	 .id = 2,
	 .parent = &per_clk[5],
	 .secondary = &gpt1_clk[1],},
	{
	/* gpt_ipg_clk */
	 .id = 2,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPT3_OFFSET,
	 .disable = clk_cgcr_disable,},
};

static struct clk gpt4_clk[] = {
	{
	/* gpt_clk */
	 .id = 3,
	 .parent = &per_clk[5],
	 .secondary = &gpt1_clk[1],},
	{
	/* gpt_ipg_clk */
	 .id = 3,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPT4_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk i2c_clk[] = {
	{
	 .id = 0,
	 .parent = &per_clk[6],},
	{
	 .id = 1,
	 .parent = &per_clk[6],},
	{
	 .id = 2,
	 .parent = &per_clk[6],},
};

struct clk iim_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_IIM_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk iomuxc_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_IOMUXC_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk kpp_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_KPP_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk lcdc_clk[] = {
	{
	/* lcdc_clk */
	 .id = 0,
	 .parent = &per_clk[7],
	 .secondary = &lcdc_clk[1],
	 .round_rate = _clk_parent_round_rate,
	 .set_rate = _clk_parent_set_rate,},
	{
	/* lcdc_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_LCDC_OFFSET,
	 .disable = clk_cgcr_disable,
	 .secondary = &lcdc_clk[2],},
	{
	/* lcdc_ahb_clk */
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_LCDC_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk owire_clk[] = {
	{
	/* owire_clk */
	 .id = 0,
	 .parent = &per_clk[9],
	 .secondary = &owire_clk[1],},
	{
	/* owire_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_OWIRE_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk pwm1_clk[] = {
	{
	/* pwm_clk */
	 .id = 0,
	 .parent = &per_clk[10],
	 .secondary = &pwm1_clk[1],},
	{
	/* pwm_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_PWM1_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk pwm2_clk[] = {
	{
	/* pwm_clk */
	 .id = 1,
	 .parent = &per_clk[10],
	 .secondary = &pwm2_clk[1],},
	{
	/* pwm_ipg_clk */
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_PWM2_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk pwm3_clk[] = {
	{
	/* pwm_clk */
	 .id = 2,
	 .parent = &per_clk[10],
	 .secondary = &pwm3_clk[1],},
	{
	/* pwm_ipg_clk */
	 .id = 2,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_PWM3_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk pwm4_clk[] = {
	{
	/* pwm_clk */
	 .id = 3,
	 .parent = &per_clk[10],
	 .secondary = &pwm4_clk[1],},
	{
	/* pwm_ipg_clk */
	 .id = 3,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_PWM3_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk rng_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR2,
	.enable_shift = MXC_CCM_CGCR2_RNGB_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk scc_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR2,
	.enable_shift = MXC_CCM_CGCR2_SCC_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk sdma_clk[] = {
	{
	/* sdma_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SDMA_OFFSET,
	 .disable = clk_cgcr_disable,
	 .secondary = &sdma_clk[1],},
	{
	/* sdma_ahb_clk */
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_SDMA_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk sim1_clk[] = {
	{
	/* sim1_clk */
	 .id = 0,
	 .parent = &per_clk[11],
	 .secondary = &sim1_clk[1],},
	{
	/* sim_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SIM1_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk sim2_clk[] = {
	{
	/* sim2_clk */
	 .id = 1,
	 .parent = &per_clk[12],
	 .secondary = &sim2_clk[1],},
	{
	/* sim_ipg_clk */
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SIM2_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk slcdc_clk[] = {
	{
	/* slcdc_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SLCDC_OFFSET,
	 .disable = clk_cgcr_disable,
	 .secondary = &slcdc_clk[1],},
	{
	/* slcdc_ahb_clk */
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_SLCDC_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk spba_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR2,
	.enable_shift = MXC_CCM_CGCR2_SPBA_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk ssi1_clk[] = {
	{
	/* ssi_clk */
	 .id = 0,
	 .parent = &per_clk[13],
	 .secondary = &ssi1_clk[1],},
	{
	/* ssi_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SSI1_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk ssi2_clk[] = {
	{
	/* ssi_clk */
	 .id = 1,
	 .parent = &per_clk[14],
	 .secondary = &ssi2_clk[1],},
	{
	/* ssi_ipg_clk */
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SSI2_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk tsc_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR2,
	.enable_shift = MXC_CCM_CGCR2_TCHSCRN_OFFSET,
	.disable = clk_cgcr_disable,
};

struct clk uart1_clk[] = {
	{
	/* uart_clk */
	 .id = 0,
	 .parent = &per_clk[15],
	 .secondary = &uart1_clk[1],},
	{
	/* uart_ipg_clk */
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_UART1_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk uart2_clk[] = {
	{
	/* uart_clk */
	 .id = 1,
	 .parent = &per_clk[15],
	 .secondary = &uart2_clk[1],},
	{
	/* uart_ipg_clk */
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_UART2_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk uart3_clk[] = {
	{
	/* uart_clk */
	 .id = 2,
	 .parent = &per_clk[15],
	 .secondary = &uart3_clk[1],},
	{
	/* uart_ipg_clk */
	 .id = 2,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_UART3_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk uart4_clk[] = {
	{
	/* uart_clk */
	 .id = 3,
	 .parent = &per_clk[15],
	 .secondary = &uart4_clk[1],},
	{
	/* uart_ipg_clk */
	 .id = 3,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_UART4_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk uart5_clk[] = {
	{
	/* uart_clk */
	 .id = 4,
	 .parent = &per_clk[15],
	 .secondary = &uart5_clk[1],},
	{
	/* uart_ipg_clk */
	 .id = 4,
	 .parent = &ipg_clk,
	 .enable = clk_cgcr_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_UART5_OFFSET,
	 .disable = clk_cgcr_disable,},
};

struct clk wdog_clk = {
	.id = 0,
	.parent = &ipg_clk,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_CGCR2,
	.enable_shift = MXC_CCM_CGCR2_WDOG_OFFSET,
	.disable = clk_cgcr_disable,
};

static unsigned long _clk_usb_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long div;
	unsigned long parent_rate;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;
	if (parent_rate % rate)
		div++;

	if (div > 64)
		return -EINVAL;

	return parent_rate / div;
}

static int _clk_usb_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long reg;
	unsigned long div;
	unsigned long parent_rate;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;

	if (parent_rate / div != rate)
		return -EINVAL;
	if (div > 64)
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CCTL) & ~MXC_CCM_CCTL_USB_DIV_MASK;
	reg |= (div - 1) << MXC_CCM_CCTL_USB_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CCTL);

	return 0;
}

static unsigned long _clk_usb_get_rate(struct clk *clk)
{
	unsigned long div =
	    __raw_readl(MXC_CCM_CCTL) & MXC_CCM_CCTL_USB_DIV_MASK;

	div >>= MXC_CCM_CCTL_USB_DIV_OFFSET;

	return clk_get_rate(clk->parent) / (div + 1);
}

static int _clk_usb_set_parent(struct clk *clk, struct clk *parent)
{
	unsigned long mcr;

	if (parent != &upll_clk && parent != &ahb_clk)
		return -EINVAL;

	clk->parent = parent;
	mcr = __raw_readl(MXC_CCM_MCR);
	if (parent == &ahb_clk)
		mcr |= (1 << MXC_CCM_MCR_USB_CLK_MUX_OFFSET);
	else
		mcr &= ~(1 << MXC_CCM_MCR_USB_CLK_MUX_OFFSET);

	__raw_writel(mcr, MXC_CCM_MCR);

	return 0;
}

static struct clk usb_clk = {
	.parent = &upll_clk,
	.get_rate = _clk_usb_get_rate,
	.set_rate = _clk_usb_set_rate,
	.round_rate = _clk_usb_round_rate,
	.set_parent = _clk_usb_set_parent,
};

/* CLKO */

static unsigned long _clk_clko_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long div;
	unsigned long parent_rate;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;
	if (parent_rate % rate)
		div++;

	if (div > 64)
		return -EINVAL;

	return parent_rate / div;
}

static int _clk_clko_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long reg;
	unsigned long div;
	unsigned long parent_rate;

	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;

	if ((parent_rate / div) != rate)
		return -EINVAL;
	if (div > 64)
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_MCR) & ~MXC_CCM_MCR_CLKO_DIV_MASK;
	reg |= (div - 1) << MXC_CCM_MCR_CLKO_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_MCR);

	return 0;
}

static unsigned long _clk_clko_get_rate(struct clk *clk)
{
	unsigned long div =
	    __raw_readl(MXC_CCM_MCR) & MXC_CCM_MCR_CLKO_DIV_MASK;

	div >>= MXC_CCM_MCR_CLKO_DIV_OFFSET;

	return clk_get_rate(clk->parent) / (div + 1);
}

static struct clk *clko_sources[] = {
	&osc32k_clk,		/* 0x0 */
	&osc24m_clk,		/* 0x1 */
	&cpu_clk,		/* 0x2 */
	&ahb_clk,		/* 0x3 */
	&ipg_clk,		/* 0x4 */
	NULL,			/* 0x5 */
	NULL,			/* 0x6 */
	NULL,			/* 0x7 */
	NULL,			/* 0x8 */
	NULL,			/* 0x9 */
	&per_clk[0],		/* 0xA */
	&per_clk[2],		/* 0xB */
	&per_clk[13],		/* 0xC */
	&per_clk[14],		/* 0xD */
	&usb_clk,		/* 0xE */
	NULL,			/* 0xF */
};

#define NR_CLKO_SOURCES (sizeof(clko_sources) / sizeof(struct clk *))

static int _clk_clko_set_parent(struct clk *clk, struct clk *parent)
{
	unsigned long reg =
	    __raw_readl(MXC_CCM_MCR) & ~MXC_CCM_MCR_CLKO_SEL_MASK;
	struct clk **src;
	int i;

	for (i = 0, src = clko_sources; i < NR_CLKO_SOURCES; i++, src++)
		if (*src == parent)
			break;

	if (i == NR_CLKO_SOURCES)
		return -EINVAL;

	clk->parent = parent;

	reg |= i << MXC_CCM_MCR_CLKO_SEL_OFFSET;

	__raw_writel(reg, MXC_CCM_MCR);

	return 0;
}

static struct clk clko_clk = {
	.get_rate = _clk_clko_get_rate,
	.set_rate = _clk_clko_set_rate,
	.round_rate = _clk_clko_round_rate,
	.set_parent = _clk_clko_set_parent,
	.enable = clk_cgcr_enable,
	.enable_reg = MXC_CCM_MCR,
	.enable_shift = MXC_CCM_MCR_CLKO_EN_OFFSET,
	.disable = clk_cgcr_disable,
};

static unsigned long get_rate_mpll(void)
{
	ulong mpctl = __raw_readl(CRM_BASE + CCM_MPCTL);

	return mxc_decode_pll(mpctl, 24000000);
}

# if 0
static unsigned long get_rate_upll(void)
{
	ulong mpctl = __raw_readl(CRM_BASE + CCM_UPCTL);

	return mxc_decode_pll(mpctl, 24000000);
}
#endif

unsigned long get_rate_arm(struct clk *clk)
{
	unsigned long cctl = readl(CRM_BASE + CCM_CCTL);
	unsigned long rate = get_rate_mpll();

	if (cctl & (1 << 14))
		rate = (rate * 3) >> 1;

	return rate / ((cctl >> 30) + 1);
}

static unsigned long get_rate_ahb(struct clk *clk)
{
	unsigned long cctl = readl(CRM_BASE + CCM_CCTL);

	return get_rate_arm(NULL) / (((cctl >> 28) & 0x3) + 1);
}

static unsigned long get_rate_ipg(struct clk *clk)
{
	return get_rate_ahb(NULL) >> 1;
}

#if 0
static unsigned long get_rate_per(int per)
{
	unsigned long ofs = (per & 0x3) * 8;
	unsigned long reg = per & ~0x3;
	unsigned long val = (readl(CRM_BASE + CCM_PCDR0 + reg) >> ofs) & 0x3f;
	unsigned long fref;

	if (readl(CRM_BASE + 0x64) & (1 << per))
		fref = get_rate_upll();
	else
		fref = get_rate_ipg(NULL);

	return fref / (val + 1);
}

static unsigned long get_rate_uart(struct clk *clk)
{
	return get_rate_per(15);
}

static unsigned long get_rate_i2c(struct clk *clk)
{
	return get_rate_per(6);
}

static unsigned long get_rate_nfc(struct clk *clk)
{
	return get_rate_per(8);
}

static unsigned long get_rate_gpt(struct clk *clk)
{
	return get_rate_per(5);
}

static unsigned long get_rate_lcdc(struct clk *clk)
{
	return get_rate_per(7);
}

static unsigned long get_rate_otg(struct clk *clk)
{
	return 48000000; /* FIXME */
}
#endif

static int clk_cgcr_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(clk->enable_reg);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	return 0;
}

static void clk_cgcr_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(clk->enable_reg);
	reg &= ~(1 << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);
}

#if 0
#define DEFINE_CLOCK(name, i, er, es, gr, sr, s)	\
	static struct clk name = {			\
		.id		= i,			\
		.enable_reg	= CRM_BASE + er,	\
		.enable_shift	= es,			\
		.get_rate	= gr,			\
		.set_rate	= sr,			\
		.enable		= clk_cgcr_enable,	\
		.disable	= clk_cgcr_disable,	\
		.secondary	= s,			\
	}

DEFINE_CLOCK(gpt_clk,    0, CCM_CGCR0,  5, get_rate_gpt, NULL, NULL);
DEFINE_CLOCK(uart_per_clk, 0, CCM_CGCR0, 15, get_rate_uart, NULL, NULL);
DEFINE_CLOCK(cspi1_clk,  0, CCM_CGCR1,  5, get_rate_ipg, NULL, NULL);
DEFINE_CLOCK(cspi2_clk,  0, CCM_CGCR1,  6, get_rate_ipg, NULL, NULL);
DEFINE_CLOCK(cspi3_clk,  0, CCM_CGCR1,  7, get_rate_ipg, NULL, NULL);
DEFINE_CLOCK(fec_ahb_clk, 0, CCM_CGCR0, 23, NULL,	 NULL, NULL);
DEFINE_CLOCK(lcdc_ahb_clk, 0, CCM_CGCR0, 24, NULL,	 NULL, NULL);
DEFINE_CLOCK(lcdc_per_clk, 0, CCM_CGCR0,  7, NULL,	 NULL, &lcdc_ahb_clk);
DEFINE_CLOCK(uart1_clk,  0, CCM_CGCR2, 14, get_rate_uart, NULL, &uart_per_clk);
DEFINE_CLOCK(uart2_clk,  0, CCM_CGCR2, 15, get_rate_uart, NULL, &uart_per_clk);
DEFINE_CLOCK(uart3_clk,  0, CCM_CGCR2, 16, get_rate_uart, NULL, &uart_per_clk);
DEFINE_CLOCK(uart4_clk,  0, CCM_CGCR2, 17, get_rate_uart, NULL, &uart_per_clk);
DEFINE_CLOCK(uart5_clk,  0, CCM_CGCR2, 18, get_rate_uart, NULL, &uart_per_clk);
DEFINE_CLOCK(nfc_clk,    0, CCM_CGCR0,  8, get_rate_nfc, NULL, NULL);
DEFINE_CLOCK(usbotg_clk, 0, CCM_CGCR0, 28, get_rate_otg, NULL, NULL);
DEFINE_CLOCK(pwm1_clk,	 0, CCM_CGCR1, 31, get_rate_ipg, NULL, NULL);
DEFINE_CLOCK(pwm2_clk,	 0, CCM_CGCR2,  0, get_rate_ipg, NULL, NULL);
DEFINE_CLOCK(pwm3_clk,	 0, CCM_CGCR2,  1, get_rate_ipg, NULL, NULL);
DEFINE_CLOCK(pwm4_clk,	 0, CCM_CGCR2,  2, get_rate_ipg, NULL, NULL);
DEFINE_CLOCK(kpp_clk,	 0, CCM_CGCR1, 28, get_rate_ipg, NULL, NULL);
DEFINE_CLOCK(tsc_clk,	 0, CCM_CGCR2, 13, get_rate_ipg, NULL, NULL);
DEFINE_CLOCK(i2c_clk,	 0, CCM_CGCR0,  6, get_rate_i2c, NULL, NULL);
DEFINE_CLOCK(fec_clk,	 0, CCM_CGCR1, 15, get_rate_ipg, NULL, &fec_ahb_clk);
DEFINE_CLOCK(dryice_clk, 0, CCM_CGCR1,  8, get_rate_ipg, NULL, NULL);
DEFINE_CLOCK(lcdc_clk,	 0, CCM_CGCR1, 29, get_rate_lcdc, NULL, &lcdc_per_clk);
#endif

#define _REGISTER_CLOCK(d, n, c)	\
	{				\
		.dev_id = d,		\
		.con_id = n,		\
		.clk = &c,		\
	},

static struct clk_lookup lookups[] = {
#if 0
	_REGISTER_CLOCK("imx-uart.0", NULL, uart1_clk)
	_REGISTER_CLOCK("imx-uart.1", NULL, uart2_clk)
	_REGISTER_CLOCK("imx-uart.2", NULL, uart3_clk)
	_REGISTER_CLOCK("imx-uart.3", NULL, uart4_clk)
	_REGISTER_CLOCK("imx-uart.4", NULL, uart5_clk)
	_REGISTER_CLOCK("mxc-ehci.0", "usb", usbotg_clk)
	_REGISTER_CLOCK("mxc-ehci.1", "usb", usbotg_clk)
	_REGISTER_CLOCK("mxc-ehci.2", "usb", usbotg_clk)
	_REGISTER_CLOCK("fsl-usb2-udc", "usb", usbotg_clk)
	_REGISTER_CLOCK("mxc_nand.0", NULL, nfc_clk)
	_REGISTER_CLOCK("spi_imx.0", NULL, cspi1_clk)
	_REGISTER_CLOCK("spi_imx.1", NULL, cspi2_clk)
	_REGISTER_CLOCK("spi_imx.2", NULL, cspi3_clk)
	_REGISTER_CLOCK("mxc_pwm.0", NULL, pwm1_clk)
	_REGISTER_CLOCK("mxc_pwm.1", NULL, pwm2_clk)
	_REGISTER_CLOCK("mxc_pwm.2", NULL, pwm3_clk)
	_REGISTER_CLOCK("mxc_pwm.3", NULL, pwm4_clk)
	_REGISTER_CLOCK("mxc-keypad", NULL, kpp_clk)
	_REGISTER_CLOCK("mx25-adc", NULL, tsc_clk)
	_REGISTER_CLOCK("imx-i2c.0", NULL, i2c_clk)
	_REGISTER_CLOCK("imx-i2c.1", NULL, i2c_clk)
	_REGISTER_CLOCK("imx-i2c.2", NULL, i2c_clk)
	_REGISTER_CLOCK("fec.0", NULL, fec_clk)
	_REGISTER_CLOCK("imxdi_rtc.0", NULL, dryice_clk)
	_REGISTER_CLOCK("imx-fb.0", NULL, lcdc_clk)
#endif
	_REGISTER_CLOCK(NULL, "osc24m_clk", osc24m_clk)
	_REGISTER_CLOCK(NULL, "osc32k_clk", osc32k_clk)
	_REGISTER_CLOCK(NULL, "mpll_clk", mpll_clk)
	_REGISTER_CLOCK(NULL, "upll_clk", upll_clk)
	_REGISTER_CLOCK(NULL, "cpu_clk", cpu_clk)
	_REGISTER_CLOCK(NULL, "ahb_clk", ahb_clk)
	_REGISTER_CLOCK(NULL, "ipg_clk", ipg_clk)
	_REGISTER_CLOCK(NULL, "usb_ahb_clk", usb_ahb_clk)
	_REGISTER_CLOCK("mxcintuart.0", NULL, uart1_clk[0])
	_REGISTER_CLOCK("mxcintuart.1", NULL, uart2_clk[0])
	_REGISTER_CLOCK("mxcintuart.2", NULL, uart3_clk[0])
	_REGISTER_CLOCK("mxcintuart.3", NULL, uart4_clk[0])
	_REGISTER_CLOCK("mxcintuart.4", NULL, uart5_clk[0])
	_REGISTER_CLOCK(NULL, "usb_ahb_clk", usb_ahb_clk)
	_REGISTER_CLOCK(NULL, "usb_clk", usb_clk)
	_REGISTER_CLOCK("mxc_nandv2_flash.0", NULL, nfc_clk)
	_REGISTER_CLOCK("spi_imx.0", NULL, cspi_clk[0])
	_REGISTER_CLOCK("spi_imx.1", NULL, cspi_clk[1])
	_REGISTER_CLOCK("spi_imx.2", NULL, cspi_clk[2])
	_REGISTER_CLOCK("mxc_pwm.0", NULL, pwm1_clk[0])
	_REGISTER_CLOCK("mxc_pwm.1", NULL, pwm2_clk[0])
	_REGISTER_CLOCK("mxc_pwm.2", NULL, pwm3_clk[0])
	_REGISTER_CLOCK("mxc_pwm.3", NULL, pwm4_clk[0])
	_REGISTER_CLOCK("mxc_keypad.0", NULL, kpp_clk)
	_REGISTER_CLOCK("imx_adc.0", NULL, tsc_clk)
	_REGISTER_CLOCK("imx-i2c.0", NULL, i2c_clk[0])
	_REGISTER_CLOCK("imx-i2c.1", NULL, i2c_clk[1])
	_REGISTER_CLOCK("imx-i2c.2", NULL, i2c_clk[2])
	_REGISTER_CLOCK("fec.0", NULL, fec_clk[0])
	_REGISTER_CLOCK(NULL, "dryice_clk", dryice_clk)
	_REGISTER_CLOCK("imx-fb.0", NULL, lcdc_clk[0])
	_REGISTER_CLOCK("mxc_sdma.0", "sdma_ipg_clk", sdma_clk[0])
	_REGISTER_CLOCK("mxc_sdma.0", "sdma_ahb_clk", sdma_clk[1])
	_REGISTER_CLOCK("mxsdhci.0", NULL, esdhc1_clk[0])
	_REGISTER_CLOCK("mxsdhci.1", NULL, esdhc2_clk[0])
	_REGISTER_CLOCK(NULL, "gpt", gpt1_clk[0])
	_REGISTER_CLOCK(NULL, "gpt", gpt2_clk[0])
	_REGISTER_CLOCK(NULL, "gpt", gpt3_clk[0])
	_REGISTER_CLOCK(NULL, "gpt", gpt4_clk[0])
	_REGISTER_CLOCK(NULL, "clko_clk", clko_clk)
	_REGISTER_CLOCK("pata_fsl", NULL, ata_clk[0])
	_REGISTER_CLOCK("FlexCAN.0", NULL, can_clk[0])
	_REGISTER_CLOCK("mxc_esai.0", NULL, esai_clk[0])
	_REGISTER_CLOCK("mxc_iim.0", NULL, iim_clk)
	_REGISTER_CLOCK("mxc_w1.0", NULL, owire_clk[0])
	_REGISTER_CLOCK(NULL, "scc_clk", scc_clk)
	_REGISTER_CLOCK("mxc_sim.0", NULL, sim1_clk[0])
	_REGISTER_CLOCK("mxc_sim.0", NULL, sim2_clk[0])
	_REGISTER_CLOCK("mxc_ssi.0", NULL, ssi1_clk[0])
	_REGISTER_CLOCK("mxc_ssi.0", NULL, ssi2_clk[0])
	_REGISTER_CLOCK("mxc_ipu", "csi_clk", csi_clk[0])
	_REGISTER_CLOCK(NULL, "lcdc_clk", lcdc_clk[0])
	_REGISTER_CLOCK(NULL, "slcdc_clk", lcdc_clk[0])
	_REGISTER_CLOCK(NULL, "rng_clk", rng_clk)
	_REGISTER_CLOCK(NULL, "audmux_clk", audmux_clk)
	_REGISTER_CLOCK(NULL, "ect_clk", ect_clk)
	_REGISTER_CLOCK(NULL, "epit1_clk", epit1_clk[0])
	_REGISTER_CLOCK(NULL, "epit2_clk", epit2_clk[0])
	_REGISTER_CLOCK(NULL, "gpio_clk", gpio_clk[0])
	_REGISTER_CLOCK(NULL, "iomuxc_clk", iomuxc_clk)
	_REGISTER_CLOCK(NULL, "spba_clk", spba_clk)
	_REGISTER_CLOCK(NULL, "wdog_clk", wdog_clk)
	_REGISTER_CLOCK(NULL, "per_csi_clk", per_clk[0])
	_REGISTER_CLOCK(NULL, "per_epit_clk", per_clk[1])
	_REGISTER_CLOCK(NULL, "per_esai_clk", per_clk[2])
	_REGISTER_CLOCK(NULL, "per_esdhc1_clk", per_clk[3])
	_REGISTER_CLOCK(NULL, "per_esdhc2_clk", per_clk[4])
	_REGISTER_CLOCK(NULL, "per_gpt_clk", per_clk[5])
	_REGISTER_CLOCK(NULL, "per_i2c_clk", per_clk[6])
	_REGISTER_CLOCK(NULL, "per_lcdc_clk", per_clk[7])
	_REGISTER_CLOCK(NULL, "per_nfc_clk", per_clk[8])
	_REGISTER_CLOCK(NULL, "per_owire_clk", per_clk[9])
	_REGISTER_CLOCK(NULL, "per_pwm_clk", per_clk[10])
	_REGISTER_CLOCK(NULL, "per_sim1_clk", per_clk[11])
	_REGISTER_CLOCK(NULL, "per_sim2_clk", per_clk[12])
	_REGISTER_CLOCK(NULL, "per_ssi1_clk", per_clk[13])
	_REGISTER_CLOCK(NULL, "per_ssi2_clk", per_clk[14])
	_REGISTER_CLOCK(NULL, "per_uart_clk", per_clk[15])
};

static struct mxc_clk mxc_clks[ARRAY_SIZE(lookups)];

/*!
 * Function to get timer clock rate early in boot process before clock tree is
 * initialized.
 *
 * @return	Clock rate for timer
 */
unsigned long __init clk_early_get_timer_rate(void)
{
	upll_clk.get_rate(&upll_clk);
	per_clk[5].get_rate(&per_clk[5]);
	per_clk[5].enable(&per_clk[5]);

	return clk_get_rate(&per_clk[5]);
}

int __init mx25_clocks_init(void)
{
	int i;
	unsigned long upll_rate;
	unsigned long ahb_rate;

	clkdev_add_table(lookups, ARRAY_SIZE(lookups));

	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		clkdev_add(&lookups[i]);
		mxc_clks[i].reg_clk = lookups[i].clk;
		if (lookups[i].con_id != NULL)
			strcpy(mxc_clks[i].name, lookups[i].con_id);
		else
			strcpy(mxc_clks[i].name, lookups[i].dev_id);
		clk_register(&mxc_clks[i]);
	}

	/* Turn off all clocks except the ones we need to survive, namely:
	 * EMI, GPIO1-3 (CCM_CGCR1[18:16]), GPT1, IOMUXC (CCM_CGCR1[27]), IIM,
	 * SCC
	 */
	__raw_writel((1 << 19), CRM_BASE + CCM_CGCR0);

	__raw_writel((1 << MXC_CCM_CGCR1_GPT1_OFFSET) |
		     (1 << MXC_CCM_CGCR1_IIM_OFFSET),
		     CRM_BASE + CCM_CGCR1);

	__raw_writel((1 << 5), CRM_BASE + CCM_CGCR2);

	/* Clock source for lcdc is upll */
	__raw_writel(__raw_readl(CRM_BASE+0x64) | (1 << 7), CRM_BASE + 0x64);

	/* Init all perclk sources to ahb clock*/
	for (i = 0; i < (sizeof(per_clk) / sizeof(struct clk)); i++)
		per_clk[i].set_parent(&per_clk[i], &ahb_clk);

	/* GPT clock must be derived from AHB clock */
	ahb_rate = clk_get_rate(&ahb_clk);
	clk_set_rate(&per_clk[5], ahb_rate / 10);

	/* LCDC clock must be derived from UPLL clock */
	upll_rate = clk_get_rate(&upll_clk);
	clk_set_parent(&per_clk[7], &upll_clk);
	clk_set_rate(&per_clk[7], upll_rate);

	/* the NFC clock must be derived from AHB clock */
	clk_set_parent(&per_clk[8], &ahb_clk);
	clk_set_rate(&per_clk[8], ahb_rate / 6);

	/* sim clock */
	clk_set_rate(&per_clk[11], ahb_rate / 2);

	/* the csi clock must be derived from UPLL clock */
	clk_set_parent(&per_clk[0], &upll_clk);
	clk_set_rate(&per_clk[0], upll_rate / 5);

	pr_info("Clock input source is %ld\n", clk_get_rate(&osc24m_clk));

	clk_enable(&emi_clk);
	clk_enable(&gpio_clk[0]);
	clk_enable(&gpio_clk[1]);
	clk_enable(&gpio_clk[2]);
	clk_enable(&iim_clk);
	clk_enable(&gpt1_clk[0]);
	clk_enable(&iomuxc_clk);
	clk_enable(&scc_clk);

	mxc_timer_init(&gpt1_clk[0], IO_ADDRESS(MX25_GPT1_BASE_ADDR),
		       MX25_INT_GPT1);

	return 0;
}
