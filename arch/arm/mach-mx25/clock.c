/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/* based on mach-mx27/clock.c */

#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <mach/clock.h>
#include <mach/common.h>
#include "crm_regs.h"

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

static int _clk_enable(struct clk *clk)
{
	unsigned long reg;

	reg = __raw_readl(clk->enable_reg);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	return 0;
}

static void _clk_disable(struct clk *clk)
{
	unsigned long reg;

	reg = __raw_readl(clk->enable_reg);
	reg &= ~(1 << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);
}

static int _clk_upll_enable(struct clk *clk)
{
	unsigned long reg;

	reg = __raw_readl(MXC_CCM_CCTL);
	reg &= ~MXC_CCM_CCTL_UPLL_DISABLE;
	__raw_writel(reg, MXC_CCM_CCTL);

	while ((__raw_readl(MXC_CCM_UPCTL) & MXC_CCM_UPCTL_LF) == 0) ;

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

static void _clk_pll_recalc(struct clk *clk)
{
	unsigned long mfi = 0, mfn = 0, mfd = 0, pdf = 0;
	unsigned long ref_clk;
	unsigned long reg;
	unsigned long long temp;

	ref_clk = clk->parent->rate;

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

	clk->rate = temp;
}

static unsigned long _clk_cpu_round_rate(struct clk *clk, unsigned long rate)
{
	int div = clk->parent->rate / rate;

	if (clk->parent->rate % rate)
		div++;

	if (div > 4)
		div = 4;

	return clk->parent->rate / div;
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
	clk->rate = rate;
#else
	div = clk->parent->rate / rate;

	if (div > 4 || div < 1 || ((clk->parent->rate / div) != rate))
		return -EINVAL;
	div--;

	reg =
	    (cctl & ~MXC_CCM_CCTL_ARM_MASK) | (div << MXC_CCM_CCTL_ARM_OFFSET);
	__raw_writel(reg, MXC_CCM_CCTL);
	clk->rate = rate;
#endif

	return 0;
}

static void _clk_cpu_recalc(struct clk *clk)
{
	unsigned long div;
	unsigned long cctl = __raw_readl(MXC_CCM_CCTL);

	div = (cctl & MXC_CCM_CCTL_ARM_MASK) >> MXC_CCM_CCTL_ARM_OFFSET;

	clk->rate = clk->parent->rate / (div + 1);

	if (cctl & MXC_CCM_CCTL_ARM_SRC) {
		clk->rate *= 3;
		clk->rate /= 4;
	}
}

static void _clk_ahb_recalc(struct clk *clk)
{
	unsigned long div;
	unsigned long cctl = __raw_readl(MXC_CCM_CCTL);

	div = (cctl & MXC_CCM_CCTL_AHB_MASK) >> MXC_CCM_CCTL_AHB_OFFSET;

	clk->rate = clk->parent->rate / (div + 1);
}

static void *pcdr_a[4] = {
	MXC_CCM_PCDR0, MXC_CCM_PCDR1, MXC_CCM_PCDR2, MXC_CCM_PCDR3
};
static void _clk_perclkx_recalc(struct clk *clk)
{
	unsigned long perclk_pdf;
	unsigned long pcdr;

	if (clk->id < 0 || clk->id > 15)
		return;

	pcdr = __raw_readl(pcdr_a[clk->id >> 2]);

	perclk_pdf =
	    (pcdr >> ((clk->id & 3) << 3)) & MXC_CCM_PCDR1_PERDIV1_MASK;

	clk->rate = clk->parent->rate / (perclk_pdf + 1);
}

static unsigned long _clk_perclkx_round_rate(struct clk *clk,
					     unsigned long rate)
{
	unsigned long div;

	div = clk->parent->rate / rate;
	if (clk->parent->rate % rate)
		div++;

	if (div > 64)
		div = 64;

	return clk->parent->rate / div;
}

static int _clk_perclkx_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long reg;
	unsigned long div;

	if (clk->id < 0 || clk->id > 15)
		return -EINVAL;

	div = clk->parent->rate / rate;
	if (div > 64 || div < 1 || ((clk->parent->rate / div) != rate))
		return -EINVAL;
	div--;

	reg =
	    __raw_readl(pcdr_a[clk->id >> 2]) & ~(MXC_CCM_PCDR1_PERDIV1_MASK <<
						  ((clk->id & 3) << 3));
	reg |= div << ((clk->id & 3) << 3);
	__raw_writel(reg, pcdr_a[clk->id >> 2]);

	clk->rate = rate;

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

static void _clk_ipg_recalc(struct clk *clk)
{
	clk->rate = clk->parent->rate / 2;	/* Always AHB / 2 */
}

static unsigned long _clk_parent_round_rate(struct clk *clk, unsigned long rate)
{
	return clk->parent->round_rate(clk->parent, rate);
}

static int _clk_parent_set_rate(struct clk *clk, unsigned long rate)
{
	int ret;

	ret = clk->parent->set_rate(clk->parent, rate);
	if (ret == 0)
		clk->rate = rate;
	return ret;
}

/* Top-level clocks */

static struct clk osc24m_clk = {
	.name = "osc24m",
	.rate = OSC24M_CLK_FREQ,
	.flags = RATE_PROPAGATES,
};

static struct clk osc32k_clk = {
	.name = "osc32k",
	.rate = OSC32K_CLK_FREQ,
	.flags = RATE_PROPAGATES,
};

static struct clk mpll_clk = {
	.name = "mpll",
	.parent = &osc24m_clk,
	.recalc = _clk_pll_recalc,
	.flags = RATE_PROPAGATES,
};

static struct clk upll_clk = {
	.name = "upll",
	.parent = &osc24m_clk,
	.recalc = _clk_pll_recalc,
	.enable = _clk_upll_enable,
	.disable = _clk_upll_disable,
	.flags = RATE_PROPAGATES,
};

static void _clk_24610k_recalc(struct clk *clk)
{
	long long temp = clk->parent->rate * 2461LL;

	do_div(temp, 24000);

	clk->rate = temp;	/* Always (UPLL * 24.61 / 240) */
}

static struct clk upll_24610k_clk = {
	.name = "upll_24610k",
	.parent = &upll_clk,
	.recalc = _clk_24610k_recalc,
	.flags = RATE_PROPAGATES,
};

/* Mid-level clocks */

static struct clk cpu_clk = {	/* ARM clock */
	.name = "cpu_clk",
	.parent = &mpll_clk,
	.set_rate = _clk_cpu_set_rate,
	.recalc = _clk_cpu_recalc,
	.round_rate = _clk_cpu_round_rate,
	.flags = RATE_PROPAGATES,
};

static struct clk ahb_clk = {	/* a.k.a. HCLK */
	.name = "ahb_clk",
	.parent = &cpu_clk,
	.recalc = _clk_ahb_recalc,
	.flags = RATE_PROPAGATES,
};

static struct clk ipg_clk = {
	.name = "ipg_clk",
	.parent = &ahb_clk,
	.recalc = _clk_ipg_recalc,
	.flags = RATE_PROPAGATES,
};

/* Bottom-level clocks */

struct clk usb_ahb_clk = {
	.name = "usb_ahb_clk",
	.id = 0,
	.parent = &ahb_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR0,
	.enable_shift = MXC_CCM_CGCR0_HCLK_USBOTG_OFFSET,
	.disable = _clk_disable,
};

struct clk rtic_clk = {
	.name = "rtic_clk",
	.id = 0,
	.parent = &ahb_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR0,
	.enable_shift = MXC_CCM_CGCR0_HCLK_RTIC_OFFSET,
	.disable = _clk_disable,
};

struct clk emi_clk = {
	.name = "emi_clk",
	.id = 0,
	.parent = &ahb_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR0,
	.enable_shift = MXC_CCM_CGCR0_HCLK_EMI_OFFSET,
	.disable = _clk_disable,
};

struct clk brom_clk = {
	.name = "brom_clk",
	.id = 0,
	.parent = &ahb_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR0,
	.enable_shift = MXC_CCM_CGCR0_HCLK_BROM_OFFSET,
	.disable = _clk_disable,
};

static struct clk per_clk[] = {
	{
	 .name = "per_csi_clk",
	 .id = 0,
	 .parent = &upll_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_epit_clk",
	 .id = 1,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_esai_clk",
	 .id = 2,
	 .parent = &ahb_clk,	/* can be AHB or UPLL or 24.61MHz */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent3,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_esdhc1_clk",
	 .id = 3,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_esdhc2_clk",
	 .id = 4,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_gpt_clk",
	 .id = 5,
	 .parent = &ahb_clk,	/* Must be AHB */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_i2c_clk",
	 .id = 6,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_lcdc_clk",
	 .id = 7,
	 .parent = &upll_clk,	/* Must be UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_nfc_clk",
	 .id = 8,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_owire_clk",
	 .id = 9,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_pwm_clk",
	 .id = 10,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_sim1_clk",
	 .id = 11,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_sim2_clk",
	 .id = 12,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_ssi1_clk",
	 .id = 13,
	 .parent = &ahb_clk,	/* can be AHB or UPLL or 24.61MHz */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent3,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_ssi2_clk",
	 .id = 14,
	 .parent = &ahb_clk,	/* can be AHB or UPLL or 24.61MHz */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent3,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
	{
	 .name = "per_uart_clk",
	 .id = 15,
	 .parent = &ahb_clk,	/* can be AHB or UPLL */
	 .round_rate = _clk_perclkx_round_rate,
	 .set_rate = _clk_perclkx_set_rate,
	 .set_parent = _clk_perclkx_set_parent,
	 .recalc = _clk_perclkx_recalc,
	 .enable = _perclk_enable,
	 .disable = _perclk_disable,
	 .flags = RATE_PROPAGATES,},
};

struct clk nfc_clk = {
	.name = "nfc_clk",
	.id = 0,
	.parent = &per_clk[8],
};

struct clk audmux_clk = {
	.name = "audmux_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_AUDMUX_OFFSET,
	.disable = _clk_disable,
};

struct clk ata_clk[] = {
	{
	 .name = "ata_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_ATA_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &ata_clk[1],},
	{
	 .name = "ata_ahb_clk",
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_ATA_OFFSET,
	 .disable = _clk_disable,},
};

struct clk can_clk[] = {
	{
	 .name = "can_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CAN1_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "can_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CAN2_OFFSET,
	 .disable = _clk_disable,},
};

struct clk csi_clk[] = {
	{
	 .name = "csi_clk",
	 .id = 0,
	 .parent = &per_clk[0],
	 .secondary = &csi_clk[1],},
	{
	 .name = "csi_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CSI_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &csi_clk[2],},
	{
	 .name = "csi_ahb_clk",
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_CSI_OFFSET,
	 .disable = _clk_disable,},
};

struct clk cspi_clk[] = {
	{
	 .name = "cspi_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CSPI1_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "cspi_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CSPI2_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "cspi_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_CSPI3_OFFSET,
	 .disable = _clk_disable,},
};

struct clk dryice_clk = {
	.name = "dryice_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_DRYICE_OFFSET,
	.disable = _clk_disable,
};

struct clk ect_clk = {
	.name = "ect_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_ECT_OFFSET,
	.disable = _clk_disable,
};

struct clk epit1_clk[] = {
	{
	 .name = "epit_clk",
	 .id = 0,
	 .parent = &per_clk[1],
	 .secondary = &epit1_clk[1],},
	{
	 .name = "epit_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_EPIT1_OFFSET,
	 .disable = _clk_disable,},
};

struct clk epit2_clk[] = {
	{
	 .name = "epit_clk",
	 .id = 1,
	 .parent = &per_clk[1],
	 .secondary = &epit2_clk[1],},
	{
	 .name = "epit_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_EPIT2_OFFSET,
	 .disable = _clk_disable,},
};

struct clk esai_clk[] = {
	{
	 .name = "esai_clk",
	 .id = 0,
	 .parent = &per_clk[2],
	 .secondary = &esai_clk[1],},
	{
	 .name = "esai_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_ESAI_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esai_clk[2],},
	{
	 .name = "esai_ahb_clk",
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_ESAI_OFFSET,
	 .disable = _clk_disable,},
};

struct clk esdhc1_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 0,
	 .parent = &per_clk[3],
	 .secondary = &esdhc1_clk[1],},
	{
	 .name = "esdhc_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_ESDHC1_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc1_clk[2],},
	{
	 .name = "esdhc_ahb_clk",
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_ESDHC1_OFFSET,
	 .disable = _clk_disable,},
};

struct clk esdhc2_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 1,
	 .parent = &per_clk[4],
	 .secondary = &esdhc2_clk[1],},
	{
	 .name = "esdhc_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_ESDHC2_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc2_clk[2],},
	{
	 .name = "esdhc_ahb_clk",
	 .id = 1,
	 .parent = &ahb_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_ESDHC2_OFFSET,
	 .disable = _clk_disable,},
};

struct clk fec_clk[] = {
	{
	 .name = "fec_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_FEC_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &fec_clk[1],},
	{
	 .name = "fec_ahb_clk",
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_FEC_OFFSET,
	 .disable = _clk_disable,},
};

struct clk gpio_clk[] = {
	{
	 .name = "gpio_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPIO1_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "gpio_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPIO2_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "gpio_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPIO3_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk gpt1_clk[] = {
	{
	 .name = "gpt_clk",
	 .id = 0,
	 .parent = &per_clk[5],
	 .secondary = &gpt1_clk[1],},
	{
	 .name = "gpt_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPT1_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk gpt2_clk[] = {
	{
	 .name = "gpt_clk",
	 .id = 1,
	 .parent = &per_clk[5],
	 .secondary = &gpt1_clk[1],},
	{
	 .name = "gpt_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPT2_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk gpt3_clk[] = {
	{
	 .name = "gpt_clk",
	 .id = 2,
	 .parent = &per_clk[5],
	 .secondary = &gpt1_clk[1],},
	{
	 .name = "gpt_ipg_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPT3_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk gpt4_clk[] = {
	{
	 .name = "gpt_clk",
	 .id = 3,
	 .parent = &per_clk[5],
	 .secondary = &gpt1_clk[1],},
	{
	 .name = "gpt_ipg_clk",
	 .id = 3,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_GPT4_OFFSET,
	 .disable = _clk_disable,},
};

struct clk i2c_clk[] = {
	{
	 .name = "i2c_clk",
	 .id = 0,
	 .parent = &per_clk[6],},
	{
	 .name = "i2c_clk",
	 .id = 1,
	 .parent = &per_clk[6],},
	{
	 .name = "i2c_clk",
	 .id = 2,
	 .parent = &per_clk[6],},
};

struct clk iim_clk = {
	.name = "iim_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_IIM_OFFSET,
	.disable = _clk_disable,
};

struct clk iomuxc_clk = {
	.name = "iomuxc_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_IOMUXC_OFFSET,
	.disable = _clk_disable,
};

struct clk kpp_clk = {
	.name = "kpp_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR1,
	.enable_shift = MXC_CCM_CGCR1_KPP_OFFSET,
	.disable = _clk_disable,
};

struct clk lcdc_clk[] = {
	{
	 .name = "lcdc_clk",
	 .id = 0,
	 .parent = &per_clk[7],
	 .secondary = &lcdc_clk[1],
	 .round_rate = _clk_parent_round_rate,
	 .set_rate = _clk_parent_set_rate,},
	{
	 .name = "lcdc_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_LCDC_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &lcdc_clk[2],},
	{
	 .name = "lcdc_ahb_clk",
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_LCDC_OFFSET,
	 .disable = _clk_disable,},
};

struct clk owire_clk[] = {
	{
	 .name = "owire_clk",
	 .id = 0,
	 .parent = &per_clk[9],
	 .secondary = &owire_clk[1],},
	{
	 .name = "owire_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_OWIRE_OFFSET,
	 .disable = _clk_disable,},
};

struct clk pwm1_clk[] = {
	{
	 .name = "pwm_clk",
	 .id = 0,
	 .parent = &per_clk[10],
	 .secondary = &pwm1_clk[1],},
	{
	 .name = "pwm_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR1,
	 .enable_shift = MXC_CCM_CGCR1_PWM1_OFFSET,
	 .disable = _clk_disable,},
};

struct clk pwm2_clk[] = {
	{
	 .name = "pwm_clk",
	 .id = 1,
	 .parent = &per_clk[10],
	 .secondary = &pwm2_clk[1],},
	{
	 .name = "pwm_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_PWM2_OFFSET,
	 .disable = _clk_disable,},
};

struct clk pwm3_clk[] = {
	{
	 .name = "pwm_clk",
	 .id = 2,
	 .parent = &per_clk[10],
	 .secondary = &pwm3_clk[1],},
	{
	 .name = "pwm_ipg_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_PWM3_OFFSET,
	 .disable = _clk_disable,},
};

struct clk pwm4_clk[] = {
	{
	 .name = "pwm_clk",
	 .id = 3,
	 .parent = &per_clk[10],
	 .secondary = &pwm4_clk[1],},
	{
	 .name = "pwm_ipg_clk",
	 .id = 3,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_PWM3_OFFSET,
	 .disable = _clk_disable,},
};

struct clk rng_clk = {
	.name = "rng_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR2,
	.enable_shift = MXC_CCM_CGCR2_RNGB_OFFSET,
	.disable = _clk_disable,
};

struct clk scc_clk = {
	.name = "scc_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR2,
	.enable_shift = MXC_CCM_CGCR2_SCC_OFFSET,
	.disable = _clk_disable,
};

struct clk sdma_clk[] = {
	{
	 .name = "sdma_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SDMA_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &sdma_clk[1],},
	{
	 .name = "sdma_ahb_clk",
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_SDMA_OFFSET,
	 .disable = _clk_disable,},
};

struct clk sim1_clk[] = {
	{
	 .name = "sim1_clk",
	 .id = 0,
	 .parent = &per_clk[11],
	 .secondary = &sim1_clk[1],},
	{
	 .name = "sim_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SIM1_OFFSET,
	 .disable = _clk_disable,},
};

struct clk sim2_clk[] = {
	{
	 .name = "sim2_clk",
	 .id = 1,
	 .parent = &per_clk[12],
	 .secondary = &sim2_clk[1],},
	{
	 .name = "sim_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SIM2_OFFSET,
	 .disable = _clk_disable,},
};

struct clk slcdc_clk[] = {
	{
	 .name = "slcdc_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SLCDC_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &slcdc_clk[1],},
	{
	 .name = "slcdc_ahb_clk",
	 .id = 0,
	 .parent = &ahb_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR0,
	 .enable_shift = MXC_CCM_CGCR0_HCLK_SLCDC_OFFSET,
	 .disable = _clk_disable,},
};

struct clk spba_clk = {
	.name = "spba_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR2,
	.enable_shift = MXC_CCM_CGCR2_SPBA_OFFSET,
	.disable = _clk_disable,
};

struct clk ssi1_clk[] = {
	{
	 .name = "ssi_clk",
	 .id = 0,
	 .parent = &per_clk[13],
	 .secondary = &ssi1_clk[1],},
	{
	 .name = "ssi_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SSI1_OFFSET,
	 .disable = _clk_disable,},
};

struct clk ssi2_clk[] = {
	{
	 .name = "ssi_clk",
	 .id = 1,
	 .parent = &per_clk[14],
	 .secondary = &ssi2_clk[1],},
	{
	 .name = "ssi_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_SSI2_OFFSET,
	 .disable = _clk_disable,},
};

struct clk tchscrn_clk = {
	.name = "tchscrn_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR2,
	.enable_shift = MXC_CCM_CGCR2_TCHSCRN_OFFSET,
	.disable = _clk_disable,
};

struct clk uart1_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 0,
	 .parent = &per_clk[15],
	 .secondary = &uart1_clk[1],},
	{
	 .name = "uart_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_UART1_OFFSET,
	 .disable = _clk_disable,},
};

struct clk uart2_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 1,
	 .parent = &per_clk[15],
	 .secondary = &uart2_clk[1],},
	{
	 .name = "uart_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_UART2_OFFSET,
	 .disable = _clk_disable,},
};

struct clk uart3_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 2,
	 .parent = &per_clk[15],
	 .secondary = &uart3_clk[1],},
	{
	 .name = "uart_ipg_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_UART3_OFFSET,
	 .disable = _clk_disable,},
};

struct clk uart4_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 3,
	 .parent = &per_clk[15],
	 .secondary = &uart4_clk[1],},
	{
	 .name = "uart_ipg_clk",
	 .id = 3,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_UART4_OFFSET,
	 .disable = _clk_disable,},
};

struct clk uart5_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 4,
	 .parent = &per_clk[15],
	 .secondary = &uart5_clk[1],},
	{
	 .name = "uart_ipg_clk",
	 .id = 4,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGCR2,
	 .enable_shift = MXC_CCM_CGCR2_UART5_OFFSET,
	 .disable = _clk_disable,},
};

struct clk wdog_clk = {
	.name = "wdog_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGCR2,
	.enable_shift = MXC_CCM_CGCR2_WDOG_OFFSET,
	.disable = _clk_disable,
};

static unsigned long _clk_usb_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long div;

	div = clk->parent->rate / rate;
	if (clk->parent->rate % rate)
		div++;

	if (div > 64)
		return -EINVAL;

	return clk->parent->rate / div;
}

static int _clk_usb_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long reg;
	unsigned long div;

	div = clk->parent->rate / rate;

	if (clk->parent->rate / div != rate)
		return -EINVAL;
	if (div > 64)
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CCTL) & ~MXC_CCM_CCTL_USB_DIV_MASK;
	reg |= (div - 1) << MXC_CCM_CCTL_USB_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CCTL);

	return 0;
}

static void _clk_usb_recalc(struct clk *clk)
{
	unsigned long div =
	    __raw_readl(MXC_CCM_CCTL) & MXC_CCM_CCTL_USB_DIV_MASK;

	div >>= MXC_CCM_CCTL_USB_DIV_OFFSET;

	clk->rate = clk->parent->rate / (div + 1);
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
	.name = "usb_clk",
	.parent = &upll_clk,
	.recalc = _clk_usb_recalc,
	.set_rate = _clk_usb_set_rate,
	.round_rate = _clk_usb_round_rate,
	.set_parent = _clk_usb_set_parent,
};

/* CLKO */

static unsigned long _clk_clko_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long div;

	div = clk->parent->rate / rate;
	if (clk->parent->rate % rate)
		div++;

	if (div > 64)
		return -EINVAL;

	return clk->parent->rate / div;
}

static int _clk_clko_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long reg;
	unsigned long div;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;
	if (div > 64)
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_MCR) & ~MXC_CCM_MCR_CLKO_DIV_MASK;
	reg |= (div - 1) << MXC_CCM_MCR_CLKO_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_MCR);

	return 0;
}

static void _clk_clko_recalc(struct clk *clk)
{
	unsigned long div =
	    __raw_readl(MXC_CCM_MCR) & MXC_CCM_MCR_CLKO_DIV_MASK;

	div >>= MXC_CCM_MCR_CLKO_DIV_OFFSET;

	clk->rate = clk->parent->rate / (div + 1);
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
	.name = "clko_clk",
	.recalc = _clk_clko_recalc,
	.set_rate = _clk_clko_set_rate,
	.round_rate = _clk_clko_round_rate,
	.set_parent = _clk_clko_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_MCR,
	.enable_shift = MXC_CCM_MCR_CLKO_EN_OFFSET,
	.disable = _clk_disable,
};

static struct clk *mxc_clks[] = {
	&osc24m_clk,
	&osc32k_clk,
	&mpll_clk,
	&upll_clk,
	&cpu_clk,
	&ahb_clk,
	&ipg_clk,
	&usb_ahb_clk,
	&per_clk[0],
	&per_clk[1],
	&per_clk[2],
	&per_clk[3],
	&per_clk[4],
	&per_clk[5],
	&per_clk[6],
	&per_clk[7],
	&per_clk[8],
	&per_clk[9],
	&per_clk[10],
	&per_clk[11],
	&per_clk[12],
	&per_clk[13],
	&per_clk[14],
	&per_clk[15],
	&nfc_clk,
	&audmux_clk,
	&ata_clk[0],
	&ata_clk[1],
	&can_clk[0],
	&can_clk[1],
	&csi_clk[0],
	&csi_clk[1],
	&csi_clk[2],
	&cspi_clk[0],
	&cspi_clk[1],
	&cspi_clk[2],
	&dryice_clk,
	&ect_clk,
	&epit1_clk[0],
	&epit1_clk[1],
	&epit2_clk[0],
	&epit2_clk[1],
	&esai_clk[0],
	&esai_clk[1],
	&esai_clk[2],
	&esdhc1_clk[0],
	&esdhc1_clk[1],
	&esdhc1_clk[2],
	&esdhc2_clk[0],
	&esdhc2_clk[1],
	&esdhc2_clk[2],
	&fec_clk[0],
	&fec_clk[1],
	&gpio_clk[0],
	&gpio_clk[1],
	&gpio_clk[2],
	&gpt1_clk[0],
	&gpt1_clk[1],
	&gpt2_clk[0],
	&gpt2_clk[1],
	&gpt3_clk[0],
	&gpt3_clk[1],
	&gpt4_clk[0],
	&gpt4_clk[1],
	&i2c_clk[0],
	&i2c_clk[1],
	&i2c_clk[2],
	&iim_clk,
	&iomuxc_clk,
	&kpp_clk,
	&lcdc_clk[0],
	&lcdc_clk[1],
	&lcdc_clk[2],
	&owire_clk[0],
	&owire_clk[1],
	&pwm1_clk[0],
	&pwm1_clk[1],
	&pwm2_clk[0],
	&pwm2_clk[1],
	&pwm3_clk[0],
	&pwm3_clk[1],
	&pwm4_clk[0],
	&pwm4_clk[1],
	&rng_clk,
	&scc_clk,
	&sdma_clk[0],
	&sdma_clk[1],
	&sim1_clk[0],
	&sim1_clk[1],
	&sim2_clk[0],
	&sim2_clk[1],
	&slcdc_clk[0],
	&slcdc_clk[1],
	&spba_clk,
	&ssi1_clk[0],
	&ssi1_clk[1],
	&ssi2_clk[0],
	&ssi2_clk[1],
	&tchscrn_clk,
	&uart1_clk[0],
	&uart1_clk[1],
	&uart2_clk[0],
	&uart2_clk[1],
	&uart3_clk[0],
	&uart3_clk[1],
	&uart4_clk[0],
	&uart4_clk[1],
	&uart5_clk[0],
	&uart5_clk[1],
	&wdog_clk,
	&usb_clk,
	&clko_clk,
};

/*!
 * Function to get timer clock rate early in boot process before clock tree is
 * initialized.
 *
 * @return	Clock rate for timer
 */
unsigned long __init clk_early_get_timer_rate(void)
{
	upll_clk.recalc(&upll_clk);
	per_clk[5].recalc(&per_clk[5]);
	per_clk[5].enable(&per_clk[5]);

	return per_clk[5].rate;
}

extern void propagate_rate(struct clk *tclk);

int __init mx25_clocks_init(unsigned long fref)
{
	int i;
	struct clk **clkp;

	for (clkp = mxc_clks; clkp < mxc_clks + ARRAY_SIZE(mxc_clks); clkp++)
		clk_register(*clkp);

	/* Turn off all possible clocks */
	__raw_writel((1 << MXC_CCM_CGCR0_HCLK_EMI_OFFSET), MXC_CCM_CGCR0);

	__raw_writel((1 << MXC_CCM_CGCR1_GPT1_OFFSET) |
		     (1 << MXC_CCM_CGCR1_IIM_OFFSET), MXC_CCM_CGCR1);
	__raw_writel(1 << MXC_CCM_CGCR2_SCC_OFFSET, MXC_CCM_CGCR2);

	/* Init all perclk sources to ahb clock*/
	for (i = 0; i < (sizeof(per_clk) / sizeof(struct clk)); i++)
		per_clk[i].set_parent(&per_clk[i], &ahb_clk);

	/* This will propagate to all children and init all the clock rates */
	propagate_rate(&osc24m_clk);
	propagate_rate(&osc32k_clk);

	/* GPT clock must be derived from AHB clock */
	clk_set_rate(&per_clk[5], ahb_clk.rate / 10);

	/* LCDC clock must be derived from UPLL clock */
	clk_set_parent(&per_clk[7], &upll_clk);
	clk_set_rate(&per_clk[7], upll_clk.rate);

	/* the NFC clock must be derived from AHB clock */
	clk_set_parent(&per_clk[8], &ahb_clk);
	clk_set_rate(&per_clk[8], ahb_clk.rate / 6);

	/* sim clock */
	clk_set_rate(&per_clk[11], ahb_clk.rate / 2);

	/* the csi clock must be derived from UPLL clock */
	clk_set_parent(&per_clk[0], &upll_clk);
	clk_set_rate(&per_clk[0], upll_clk.rate / 5);

	pr_info("Clock input source is %ld\n", osc24m_clk.rate);

	clk_enable(&emi_clk);
	clk_enable(&gpio_clk[0]);
	clk_enable(&gpio_clk[1]);
	clk_enable(&gpio_clk[2]);
	clk_enable(&iim_clk);
	clk_enable(&gpt1_clk[0]);
	clk_enable(&iomuxc_clk);
	clk_enable(&scc_clk);

	mxc_timer_init(&gpt1_clk[0], IO_ADDRESS(GPT1_BASE_ADDR), MXC_INT_GPT1);
	return 0;
}
