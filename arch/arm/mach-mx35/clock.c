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

#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <asm/div64.h>

#include "crm_regs.h"

#define PRE_DIV_MIN_FREQ    10000000	/* Minimum Frequency after Predivider */
#define PROPAGATE_RATE_DIS  2

struct timer_list dptcen_timer;
static int cpu_curr_wp;
static struct cpu_wp *cpu_wp_tbl;
static int cpu_wp_nr;
static int cpu_wp_offset;

static struct clk mcu_pll_clk;
static struct clk peri_pll_clk;
static struct clk ipg_clk;
static struct clk ckih_clk;
static struct clk ckie_clk;
static struct clk ahb_clk;
static struct clk cpu_clk;

#define CLK_CODE(arm, ahb, sel) (((arm) << 16) + ((ahb) << 8) + (sel))
#define CLK_CODE_ARM(c)		(((c) >> 16) & 0xFF)
#define CLK_CODE_AHB(c) 	(((c) >>  8) & 0xFF)
#define CLK_CODE_PATH(c) 	((c) & 0xFF)

static int __get_arm_div(unsigned long pdr0, int *fi, int *fd);

static int g_clk_mux_auto[8] = {
	CLK_CODE(1, 3, 0), CLK_CODE(1, 2, 1), CLK_CODE(2, 1, 1), -1,
	CLK_CODE(1, 6, 0), CLK_CODE(1, 4, 1), CLK_CODE(2, 2, 1), -1,
};

static int g_clk_mux_consumer[16] = {
	CLK_CODE(1, 4, 0), CLK_CODE(1, 3, 1), CLK_CODE(2, 2, 0), -1,
	-1, -1, CLK_CODE(4, 1, 0), CLK_CODE(1, 5, 0),
	CLK_CODE(1, 8, 0), CLK_CODE(1, 6, 1), CLK_CODE(2, 4, 0), -1,
	-1, -1, CLK_CODE(4, 2, 0), -1,
};

static int g_hsp_div_table[3][16] = {
	{4, 3, 2, -1, -1, -1, 1, 5, 4, 3, 2, -1, -1, -1, 1, -1},
	{-1, -1, -1, -1, -1, -1, -1, -1, 8, 6, 4, -1, -1, -1, 2, -1},
	{3, -1, -1, -1, -1, -1, -1, -1, 3, -1, -1, -1, -1, -1, -1, -1},
};

static void __calc_dividers(u32 div, u32 *pre, u32 *post, u32 base)
{
	u32 min_pre, temp_pre, old_err, err;
	min_pre = (div - 1) / base + 1;
	old_err = 8;
	for (temp_pre = 8; temp_pre >= min_pre; temp_pre--) {
		if (div > (temp_pre * base))
			break;
		if (div < (temp_pre * temp_pre))
			continue;
		err = div % temp_pre;
		if (err == 0) {
			*pre = temp_pre;
			break;
		}
		err = temp_pre - err;
		if (err < old_err) {
			old_err = err;
			*pre = temp_pre;
		}
	}
	*post = (div + *pre - 1) / *pre;
}

static void __calc_pre_post_dividers(u32 div, u32 *pre, u32 *post)
{
	if (div >= 512) {
		*pre = 8;
		*post = 64;
	} else if (div >= 64) {
		__calc_dividers(div, pre, post, 64);
	} else if (div <= 8) {
		*pre = div;
		*post = 1;
	} else {
		*pre = 1;
		*post = div;
	}
}

static void __calc_two_dividers(u32 div, u32 *pre, u32 *post)
{
	if (div >= 64) {
		*pre = *post = 8;
	} else if (div > 8) {
		__calc_dividers(div, pre, post, 8);
	} else {
		*pre = 1;
		*post = div;
	}
}

static unsigned long _clk_per_post_round_rate(struct clk *clk,
					      unsigned long rate)
{
	u32 pre, post;
	u32 div = clk->parent->rate / rate;
	if (clk->parent->rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return clk->parent->rate / (pre * post);
}

static unsigned long _clk_round_rate(struct clk *clk, unsigned long rate)
{
	u32 pre, post;
	u32 div = clk->parent->rate / rate;
	if (clk->parent->rate % rate)
		div++;

	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		__calc_two_dividers(div, &pre, &post);
		return clk->parent->rate / (pre * post);
	} else
		return clk->parent->rate / div;
}

static int __switch_cpu_wp(struct clk *clk, unsigned long rate)
{
	int i;
	u32 reg_value;
	if (cpu_wp_tbl[cpu_curr_wp].cpu_rate < rate) {
		for (i = cpu_curr_wp + 2; i < cpu_wp_nr; i += 2) {
			if (rate == cpu_wp_tbl[i].cpu_rate)
				goto found;
		}
		return -EINVAL;
	} else {
		for (i = cpu_curr_wp - 2; i >= 0; i -= 2) {
			if (rate == cpu_wp_tbl[i].cpu_rate)
				goto found;
		}
		return -EINVAL;
	}
      found:
	reg_value = __raw_readl(MXC_CCM_PDR0);
	reg_value = (reg_value & ~(MXC_CCM_PDR0_CON_MUX_DIV_MASK |
				   MXC_CCM_PDR0_AUTO_MUX_DIV_MASK)) |
	    cpu_wp_tbl[i].pdr0_reg;
	__raw_writel(reg_value, MXC_CCM_PDR0);

	if (cpu_wp_tbl[i].pll_rate != cpu_wp_tbl[cpu_curr_wp].pll_rate)
		clk_set_rate(clk->parent, cpu_wp_tbl[i].pll_rate);
	cpu_curr_wp = i;
	clk->rate = rate;
	return 0;
}

static int __switch_cpu_rate(struct clk *clk, unsigned long rate)
{
	int prev;
	unsigned long tmp;
	int arm_div, fi, fd, start, end;
	u32 reg_value;

	if (cpu_wp_tbl[cpu_curr_wp].cpu_rate < rate) {
		start = cpu_curr_wp + 2;
		end = cpu_wp_nr;
		prev = cpu_curr_wp;
	} else {
		start = cpu_wp_offset + 2;
		end = cpu_curr_wp;
		prev = cpu_wp_offset;
	}
	while (start < end) {
		arm_div = __get_arm_div(cpu_wp_tbl[start].pdr0_reg, &fi, &fd);
		tmp = (mcu_pll_clk.rate * fi) / (arm_div * fd);
		if (tmp == rate) {
			prev = start;
			break;
		}
		if (tmp < rate) {
			if (prev < start)
				prev = start;
		} else {
			break;
		}
		start += 2;
	}
	if (start >= end)
		return -EINVAL;

	if (prev == cpu_curr_wp)
		return 0;

	reg_value = __raw_readl(MXC_CCM_PDR0);
	reg_value = (reg_value & ~(MXC_CCM_PDR0_CON_MUX_DIV_MASK |
				   MXC_CCM_PDR0_AUTO_MUX_DIV_MASK)) |
	    cpu_wp_tbl[prev].pdr0_reg;
	__raw_writel(reg_value, MXC_CCM_PDR0);

	cpu_curr_wp = prev;
	clk->rate = rate;
	return 0;
}

static int __get_arm_div(unsigned long pdr0, int *fi, int *fd)
{
	int *pclk_mux;
	if ((pdr0 & MXC_CCM_PDR0_AUTO_CON)
	    || (cpu_is_mx35_rev(CHIP_REV_2_0) >= 1))
		pclk_mux =
		    g_clk_mux_consumer +
		    ((pdr0 & MXC_CCM_PDR0_CON_MUX_DIV_MASK) >>
		     MXC_CCM_PDR0_CON_MUX_DIV_OFFSET);
	else {
		pclk_mux = g_clk_mux_auto +
		    ((pdr0 & MXC_CCM_PDR0_AUTO_MUX_DIV_MASK) >>
		     MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET);
	}

	if ((*pclk_mux) == -1) {
		BUG();
		return -EINVAL;
	}

	if (fi && fd) {
		if (!CLK_CODE_PATH(*pclk_mux)) {
			*fi = *fd = 1;
			return CLK_CODE_ARM(*pclk_mux);
		}
		if ((pdr0 & MXC_CCM_PDR0_AUTO_CON)
		    || (cpu_is_mx35_rev(CHIP_REV_2_0) >= 1)) {
			*fi = 3;
			*fd = 4;
		} else {
			*fi = 2;
			*fd = 3;
		}
	}
	return CLK_CODE_ARM(*pclk_mux);
}

static int __get_ahb_div(unsigned long pdr0)
{
	int *pclk_mux;
	if ((pdr0 & MXC_CCM_PDR0_AUTO_CON)
	    || (cpu_is_mx35_rev(CHIP_REV_2_0) >= 1)) {
		pclk_mux =
		    g_clk_mux_consumer +
		    ((pdr0 & MXC_CCM_PDR0_CON_MUX_DIV_MASK) >>
		     MXC_CCM_PDR0_CON_MUX_DIV_OFFSET);
	} else {
		pclk_mux = g_clk_mux_auto +
		    ((pdr0 & MXC_CCM_PDR0_AUTO_MUX_DIV_MASK) >>
		     MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET);
	}

	if ((*pclk_mux) == -1) {
		BUG();
		return -EINVAL;
	}
	return CLK_CODE_AHB(*pclk_mux);
}

static void sync_cpu_wb(void)
{
	int i;
	struct cpu_wp *p;
	unsigned long reg = __raw_readl(MXC_CCM_PDR0);
	if ((reg & MXC_CCM_PDR0_AUTO_CON)
	    || (cpu_is_mx35_rev(CHIP_REV_2_0) >= 1)) {
		reg &= MXC_CCM_PDR0_CON_MUX_DIV_MASK;
	} else {
		reg &= MXC_CCM_PDR0_AUTO_MUX_DIV_MASK;
	}
	for (i = 0; i < cpu_wp_nr; i++) {
		p = cpu_wp_tbl + cpu_curr_wp;
		if (p->pdr0_reg == (reg & 0xF0E00))
			break;
		cpu_curr_wp = (cpu_curr_wp + 1) % cpu_wp_nr;
	}
	cpu_wp_offset = cpu_curr_wp & 1;
}

static int _clk_enable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg |= 3 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	return 0;
}

static void _clk_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(clk->enable_reg);
	reg &= ~(3 << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);
}

static void _clk_emi_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(clk->enable_reg);
	reg &= ~(3 << clk->enable_shift);
	reg |= (1 << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);
}

static int _clk_asrc_enable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(MXC_CCM_COSR);
	__raw_writel(reg | MXC_CCM_COSR_ASRC_AUDIO_EN, MXC_CCM_COSR);
	return 0;
}

static void _clk_asrc_disable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(MXC_CCM_COSR);
	__raw_writel(reg & (~MXC_CCM_COSR_ASRC_AUDIO_EN), MXC_CCM_COSR);
}

static int _clk_pll_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	signed long pd = 1;	/* Pre-divider */
	signed long mfi;	/* Multiplication Factor (Integer part) */
	signed long mfn;	/* Multiplication Factor (Integer part) */
	signed long mfd;	/* Multiplication Factor (Denominator Part) */
	signed long tmp;
	u32 ref_freq = clk->parent->rate;

	if ((clk == &mcu_pll_clk)
	    && (clk->parent->rate == cpu_wp_tbl[cpu_curr_wp].pll_rate)) {
		__raw_writel(cpu_wp_tbl[cpu_curr_wp].pll_reg, MXC_CCM_MPCTL);
		clk->rate = rate;
		return 0;
	}

	while (((ref_freq / pd) * 10) > rate)
		pd++;

	if ((ref_freq / pd) < PRE_DIV_MIN_FREQ)
		return -EINVAL;

	/* the ref_freq/2 in the following is to round up */
	mfi = (((rate / 2) * pd) + (ref_freq / 2)) / ref_freq;
	if (mfi < 5 || mfi > 15)
		return -EINVAL;

	/* pick a mfd value that will work
	 * then solve for mfn */
	mfd = ref_freq / 50000;

	/*
	 *          pll_freq * pd * mfd
	 *   mfn = --------------------  -  (mfi * mfd)
	 *           2 * ref_freq
	 */
	/* the tmp/2 is for rounding */
	tmp = ref_freq / 10000;
	mfn =
	    ((((((rate / 2) + (tmp / 2)) / tmp) * pd) * mfd) / 10000) -
	    (mfi * mfd);

	mfn = mfn & 0x3ff;
	pd--;
	mfd--;

	/* Change the Pll value */
	reg = (mfi << MXC_CCM_PCTL_MFI_OFFSET) |
	    (mfn << MXC_CCM_PCTL_MFN_OFFSET) |
	    (mfd << MXC_CCM_PCTL_MFD_OFFSET) | (pd << MXC_CCM_PCTL_PD_OFFSET);

	if (clk == &mcu_pll_clk)
		__raw_writel(reg, MXC_CCM_MPCTL);
	else if (clk == &peri_pll_clk)
		__raw_writel(reg, MXC_CCM_PPCTL);

	clk->rate = rate;
	return 0;
}

static int _clk_cpu_set_rate(struct clk *clk, unsigned long rate)
{
	if ((rate < ahb_clk.rate) || (rate % ahb_clk.rate != 0)) {
		printk(KERN_ERR "Wrong rate %lu in _clk_cpu_set_rate\n", rate);
		return -EINVAL;
	}

	if (clk->rate == rate)
		return 0;

	if (clk->parent->rate == cpu_wp_tbl[cpu_curr_wp].pll_rate)
		return __switch_cpu_wp(clk, rate);
	return __switch_cpu_rate(clk, rate);
}

static void _clk_pll_recalc(struct clk *clk)
{
	long mfi, mfn, mfd, pdf, ref_clk, mfn_abs;
	unsigned long reg = 0;
	s64 temp;

	ref_clk = ckih_clk.rate;

	if (clk == &mcu_pll_clk)
		reg = __raw_readl(MXC_CCM_MPCTL);
	else if (clk == &peri_pll_clk)
		reg = __raw_readl(MXC_CCM_PPCTL);
	else
		BUG();

	pdf = (reg & MXC_CCM_PCTL_PD_MASK) >> MXC_CCM_PCTL_PD_OFFSET;
	mfd = (reg & MXC_CCM_PCTL_MFD_MASK) >> MXC_CCM_PCTL_MFD_OFFSET;
	mfi = (reg & MXC_CCM_PCTL_MFI_MASK) >> MXC_CCM_PCTL_MFI_OFFSET;
	mfi = (mfi <= 5) ? 5 : mfi;
	mfn = mfn_abs = reg & MXC_CCM_PCTL_MFN_MASK;

	if (mfn >= 0x200) {
		mfn |= 0xFFFFFE00;
		mfn_abs = -mfn;
	}

	ref_clk *= 2;
	ref_clk /= pdf + 1;

	temp = (u64) ref_clk * mfn_abs;
	do_div(temp, mfd + 1);
	if (mfn < 0)
		temp = -temp;
	temp = (ref_clk * mfi) + temp;

	clk->rate = temp;
}

static int _clk_peri_pll_enable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(MXC_CCM_CCMR);
	reg |= MXC_CCM_CCMR_UPE;
	__raw_writel(reg, MXC_CCM_CCMR);

	/* No lock bit on MX31, so using max time from spec */
	udelay(80);

	return 0;
}

static void _clk_peri_pll_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCMR);
	reg &= ~MXC_CCM_CCMR_UPE;
	__raw_writel(reg, MXC_CCM_CCMR);
}

#define PDR0(mask, off) ((__raw_readl(MXC_CCM_PDR0) & mask) >> off)
#define PDR1(mask, off) ((__raw_readl(MXC_CCM_PDR1) & mask) >> off)
#define PDR2(mask, off) ((__raw_readl(MXC_CCM_PDR2) & mask) >> off)
#define PDR3(mask, off) ((__raw_readl(MXC_CCM_PDR3) & mask) >> off)
#define PDR4(mask, off) ((__raw_readl(MXC_CCM_PDR4) & mask) >> off)

static void _clk_cpu_recalc(struct clk *clk)
{
	unsigned long pdr0 = __raw_readl(MXC_CCM_PDR0);
	int arm_div, fi, fd;
	if (clk->parent->rate == cpu_wp_tbl[cpu_curr_wp].pll_rate) {
		clk->rate = cpu_wp_tbl[cpu_curr_wp].cpu_rate;
	} else {
		arm_div = __get_arm_div(pdr0, &fi, &fd);
		clk->rate = (clk->parent->rate * fi) / (arm_div * fd);
	}
}

static void _clk_hclk_recalc(struct clk *clk)
{
	unsigned long ahb_div, pdr0 = __raw_readl(MXC_CCM_PDR0);
	ahb_div = __get_ahb_div(pdr0);
	clk->rate = clk->parent->rate / ahb_div;
}

static void _clk_ipg_recalc(struct clk *clk)
{
	clk->rate = clk->parent->rate / 2;
}

static void _clk_nfc_recalc(struct clk *clk)
{
	unsigned long nfc_pdf;

	nfc_pdf = PDR4(MXC_CCM_PDR4_NFC_PODF_MASK,
		       MXC_CCM_PDR4_NFC_PODF_OFFSET);
	clk->rate = clk->parent->rate / (nfc_pdf + 1);
}

static void _clk_hsp_recalc(struct clk *clk)
{
	int hsp_pdf;
	unsigned long reg;
	reg = __raw_readl(MXC_CCM_PDR0);

	if ((reg & MXC_CCM_PDR0_AUTO_CON)
	    || (cpu_is_mx35_rev(CHIP_REV_2_0) >= 1)) {
		hsp_pdf =
		    (reg & MXC_CCM_PDR0_HSP_PODF_MASK) >>
		    MXC_CCM_PDR0_HSP_PODF_OFFSET;
		reg =
		    (reg & MXC_CCM_PDR0_CON_MUX_DIV_MASK) >>
		    MXC_CCM_PDR0_CON_MUX_DIV_OFFSET;
		if (hsp_pdf < 3) {
			hsp_pdf = g_hsp_div_table[hsp_pdf][reg];
			if (hsp_pdf > 0)
				clk->rate = clk->parent->rate / hsp_pdf;
		}
	} else {
		clk->rate = clk->parent->rate;
	}
}

static void _clk_mlb_recalc(struct clk *clk)
{
	clk->rate = clk->parent->rate * 2;
}

static void _clk_usb_recalc(struct clk *clk)
{
	unsigned long usb_podf, usb_prdf;
	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		usb_podf = PDR4(MXC_CCM_PDR4_USB_PODF_MASK,
				MXC_CCM_PDR4_USB_PODF_OFFSET);
		usb_prdf = PDR4(MXC_CCM_PDR4_USB_PRDF_MASK,
				MXC_CCM_PDR4_USB_PRDF_OFFSET);
		clk->rate =
		    clk->parent->rate / ((usb_prdf + 1) * (usb_podf + 1));
	} else {
		usb_podf = PDR4(MXC_CCM_PDR4_USB_PODF_MASK_V2,
				MXC_CCM_PDR4_USB_PODF_OFFSET);
		clk->rate = clk->parent->rate / (usb_podf + 1);
	}
}

static int _clk_usb_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 podf, prdf;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		__calc_two_dividers(div, &prdf, &podf);
		reg = __raw_readl(MXC_CCM_PDR4) &
		    ~(MXC_CCM_PDR4_USB_PODF_MASK | MXC_CCM_PDR4_USB_PRDF_MASK);
		reg |= (podf - 1) << MXC_CCM_PDR4_USB_PODF_OFFSET;
		reg |= (prdf - 1) << MXC_CCM_PDR4_USB_PRDF_OFFSET;
	} else {
		podf = div - 1;
		reg =
		    __raw_readl(MXC_CCM_PDR4) & ~MXC_CCM_PDR4_USB_PODF_MASK_V2;
		reg |= (podf - 1) << MXC_CCM_PDR4_USB_PODF_OFFSET;
	}
	__raw_writel(reg, MXC_CCM_PDR4);
	clk->rate = rate;
	return 0;
}

static void _clk_csi_recalc(struct clk *clk)
{
	u32 podf, prdf;

	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		prdf = PDR2(MXC_CCM_PDR2_CSI_PRDF_MASK,
			    MXC_CCM_PDR2_CSI_PRDF_OFFSET);
		podf =
		    PDR2(MXC_CCM_PDR2_CSI_PODF_MASK,
			 MXC_CCM_PDR2_CSI_PODF_OFFSET);
		clk->rate = clk->parent->rate / ((prdf + 1) * (podf + 1));
	} else {
		podf =
		    PDR2(MXC_CCM_PDR2_CSI_PODF_MASK_V2,
			 MXC_CCM_PDR2_CSI_PODF_OFFSET);
		clk->rate = clk->parent->rate / (podf + 1);
	}
}

static int _clk_csi_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 prdf, podf;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		__calc_two_dividers(div, &prdf, &podf);
		reg = __raw_readl(MXC_CCM_PDR2) &
		    ~(MXC_CCM_PDR2_CSI_PRDF_MASK | MXC_CCM_PDR2_CSI_PODF_MASK);
		reg |= (podf - 1) << MXC_CCM_PDR2_CSI_PODF_OFFSET;
		reg |= (prdf - 1) << MXC_CCM_PDR2_CSI_PRDF_OFFSET;
	} else {
		reg =
		    __raw_readl(MXC_CCM_PDR2) & ~MXC_CCM_PDR2_CSI_PODF_MASK_V2;
		reg |= (div - 1) << MXC_CCM_PDR2_CSI_PODF_OFFSET;
	}

	/* Set CSI clock divider */
	__raw_writel(reg, MXC_CCM_PDR2);
	clk->rate = rate;
	return 0;
}

static int _clk_csi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	if (parent == &cpu_clk)
		reg = __raw_readl(MXC_CCM_PDR2) | MXC_CCM_PDR2_CSI_M_U;
	else if (parent == &peri_pll_clk)
		reg = __raw_readl(MXC_CCM_PDR2) & (~MXC_CCM_PDR2_CSI_M_U);
	else
		return -EINVAL;
	__raw_writel(reg, MXC_CCM_PDR2);
	return 0;
}

static void _clk_per_recalc(struct clk *clk)
{
	u32 podf = 0, prdf = 0;

	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		if (clk->parent == &cpu_clk) {
			prdf = PDR4(MXC_CCM_PDR4_PER0_PRDF_MASK,
				    MXC_CCM_PDR4_PER0_PRDF_OFFSET);
			podf = PDR4(MXC_CCM_PDR4_PER0_PODF_MASK,
				    MXC_CCM_PDR4_PER0_PODF_OFFSET);
		} else {
			podf = PDR0(MXC_CCM_PDR0_PER_PODF_MASK,
				    MXC_CCM_PDR0_PER_PODF_OFFSET);
		}
		clk->rate = clk->parent->rate / ((podf + 1) * (prdf + 1));
	} else {
		if (clk->parent == &ahb_clk)
			podf = PDR0(MXC_CCM_PDR0_PER_PODF_MASK,
				    MXC_CCM_PDR0_PER_PODF_OFFSET);
		else if (clk->parent == &cpu_clk) {
			podf = PDR4(MXC_CCM_PDR4_PER0_PODF_MASK_V2,
				    MXC_CCM_PDR4_PER0_PODF_OFFSET);
		}
		clk->rate = clk->parent->rate / (podf + 1);
	}
}

static void _clk_uart_per_recalc(struct clk *clk)
{
	unsigned long podf, prdf;
	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		prdf = PDR4(MXC_CCM_PDR4_UART_PRDF_MASK,
			    MXC_CCM_PDR4_UART_PRDF_OFFSET);
		podf = PDR4(MXC_CCM_PDR4_UART_PODF_MASK,
			    MXC_CCM_PDR4_UART_PODF_OFFSET);
		clk->rate = clk->parent->rate / ((prdf + 1) * (podf + 1));
	} else {
		podf =
		    PDR4(MXC_CCM_PDR4_UART_PODF_MASK_V2,
			 MXC_CCM_PDR4_UART_PODF_OFFSET);
		clk->rate = clk->parent->rate / (podf + 1);
	}

}

static int _clk_uart_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 prdf, podf;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	/* Set UART clock divider */
	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		__calc_two_dividers(div, &prdf, &podf);
		reg = __raw_readl(MXC_CCM_PDR4) &
		    ~(MXC_CCM_PDR4_UART_PRDF_MASK |
		      MXC_CCM_PDR4_UART_PODF_MASK);
		reg |= (podf - 1) << MXC_CCM_PDR4_UART_PODF_OFFSET;
		reg |= (prdf - 1) << MXC_CCM_PDR4_UART_PRDF_OFFSET;
	} else {
		reg =
		    __raw_readl(MXC_CCM_PDR4) & ~MXC_CCM_PDR4_UART_PODF_MASK_V2;
		reg |= (div - 1) << MXC_CCM_PDR4_UART_PODF_OFFSET;
	}
	__raw_writel(reg, MXC_CCM_PDR4);
	clk->rate = rate;
	return 0;
}

static void _clk_ssi_recalc(struct clk *clk)
{
	unsigned long ssi_pdf, ssi_prepdf;

	if (clk->id == 1) {
		ssi_pdf = PDR2(MXC_CCM_PDR2_SSI2_PODF_MASK,
			       MXC_CCM_PDR2_SSI2_PODF_OFFSET);
		ssi_prepdf = PDR2(MXC_CCM_PDR2_SSI2_PRDF_MASK,
				  MXC_CCM_PDR2_SSI2_PRDF_OFFSET);
	} else {
		ssi_pdf = PDR2(MXC_CCM_PDR2_SSI1_PODF_MASK,
			       MXC_CCM_PDR2_SSI1_PODF_OFFSET);
		ssi_prepdf = PDR2(MXC_CCM_PDR2_SSI1_PRDF_MASK,
				  MXC_CCM_PDR2_SSI1_PRDF_OFFSET);
	}
	clk->rate = clk->parent->rate / ((ssi_prepdf + 1) * (ssi_pdf + 1));
}

static int _clk_ssi_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	if (clk->id == 1) {
		reg = __raw_readl(MXC_CCM_PDR2) &
		    ~(MXC_CCM_PDR2_SSI2_PRDF_MASK |
		      MXC_CCM_PDR2_SSI2_PODF_MASK);
		reg |= (post - 1) << MXC_CCM_PDR2_SSI2_PODF_OFFSET;
		reg |= (pre - 1) << MXC_CCM_PDR2_SSI2_PRDF_OFFSET;
	} else {
		reg = __raw_readl(MXC_CCM_PDR2) &
		    ~(MXC_CCM_PDR2_SSI1_PRDF_MASK |
		      MXC_CCM_PDR2_SSI1_PODF_MASK);
		reg |= (post - 1) << MXC_CCM_PDR2_SSI1_PODF_OFFSET;
		reg |= (pre - 1) << MXC_CCM_PDR2_SSI1_PRDF_OFFSET;
	}
	__raw_writel(reg, MXC_CCM_PDR2);

	clk->rate = rate;
	return 0;
}

static void _clk_mstick1_recalc(struct clk *clk)
{
	unsigned long prdf, podf;
	prdf = PDR1(MXC_CCM_PDR1_MSHC_PRDF_MASK, MXC_CCM_PDR1_MSHC_PRDF_OFFSET);
	podf = PDR1(MXC_CCM_PDR1_MSHC_PODF_MASK, MXC_CCM_PDR1_MSHC_PODF_OFFSET);
	clk->rate = clk->parent->rate / ((prdf + 1) * (podf + 1));
}

static int _clk_mstick1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	reg = __raw_readl(MXC_CCM_PDR1) &
	    ~(MXC_CCM_PDR1_MSHC_PRDF_MASK | MXC_CCM_PDR1_MSHC_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_PDR1_MSHC_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_PDR1_MSHC_PRDF_OFFSET;
	__raw_writel(reg, MXC_CCM_PDR1);

	clk->rate = rate;
	return 0;
}

static int _clk_mstick1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	if (parent == &cpu_clk)
		reg = __raw_readl(MXC_CCM_PDR1) | MXC_CCM_PDR1_MSHC_M_U;
	else if (parent == &peri_pll_clk)
		reg = __raw_readl(MXC_CCM_PDR1) & (~MXC_CCM_PDR1_MSHC_M_U);
	else
		return -EINVAL;
	__raw_writel(reg, MXC_CCM_PDR1);
	return 0;
}

static void _clk_spdif_recalc(struct clk *clk)
{
	unsigned long prdf, podf;
	prdf =
	    PDR3(MXC_CCM_PDR3_SPDIF_PRDF_MASK, MXC_CCM_PDR3_SPDIF_PRDF_OFFSET);
	podf =
	    PDR3(MXC_CCM_PDR3_SPDIF_PODF_MASK, MXC_CCM_PDR3_SPDIF_PODF_OFFSET);
	clk->rate = clk->parent->rate / ((prdf + 1) * (podf + 1));
}

static int _clk_spdif_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	reg = __raw_readl(MXC_CCM_PDR3) &
	    ~(MXC_CCM_PDR3_SPDIF_PRDF_MASK | MXC_CCM_PDR3_SPDIF_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_PDR3_SPDIF_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_PDR3_SPDIF_PRDF_OFFSET;
	__raw_writel(reg, MXC_CCM_PDR3);

	clk->rate = rate;
	return 0;
}

static int _clk_spdif_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	if (parent == &cpu_clk)
		reg = __raw_readl(MXC_CCM_PDR3) | MXC_CCM_PDR3_SPDIF_M_U;
	else if (parent == &peri_pll_clk)
		reg = __raw_readl(MXC_CCM_PDR3) & (~MXC_CCM_PDR3_SPDIF_M_U);
	else
		return -EINVAL;
	__raw_writel(reg, MXC_CCM_PDR3);
	return 0;
}

static void _clk_asrc_recalc(struct clk *clk)
{
	unsigned long div;
	div = __raw_readl(MXC_CCM_COSR) & MXC_CCM_COSR_ASRC_AUDIO_PODF_MASK;
	div = div >> MXC_CCM_COSR_ASRC_AUDIO_PODF_OFFSET;
	clk->rate = clk->parent->rate / (div + 1);
}

static int _clk_asrc_set_rate(struct clk *clk, unsigned long rate)
{
	int div;
	unsigned long reg;
	if (clk->parent->rate % rate)
		return -EINVAL;

	div = clk->parent->rate / rate;
	reg = __raw_readl(MXC_CCM_COSR) & (~MXC_CCM_COSR_ASRC_AUDIO_PODF_MASK);
	reg |= (div - 1) << MXC_CCM_COSR_ASRC_AUDIO_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_COSR);
	clk->rate = rate;
	return 0;
}

static void _clk_sdhc_recalc(struct clk *clk)
{
	u32 podf = 0, prdf = 0;

	switch (clk->id) {
	case 0:
		if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
			prdf = PDR3(MXC_CCM_PDR3_ESDHC1_PRDF_MASK,
				    MXC_CCM_PDR3_ESDHC1_PRDF_OFFSET);
			podf = PDR3(MXC_CCM_PDR3_ESDHC1_PODF_MASK,
				    MXC_CCM_PDR3_ESDHC1_PODF_OFFSET);
		} else
			podf = PDR3(MXC_CCM_PDR3_ESDHC1_PODF_MASK_V2,
				    MXC_CCM_PDR3_ESDHC1_PODF_OFFSET);
		break;
	case 1:
		if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
			prdf = PDR3(MXC_CCM_PDR3_ESDHC2_PRDF_MASK,
				    MXC_CCM_PDR3_ESDHC2_PRDF_OFFSET);
			podf = PDR3(MXC_CCM_PDR3_ESDHC2_PODF_MASK,
				    MXC_CCM_PDR3_ESDHC2_PODF_OFFSET);
		} else
			podf = PDR3(MXC_CCM_PDR3_ESDHC2_PODF_MASK_V2,
				    MXC_CCM_PDR3_ESDHC2_PODF_OFFSET);
		break;
	case 2:
		if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
			prdf = PDR3(MXC_CCM_PDR3_ESDHC3_PRDF_MASK,
				    MXC_CCM_PDR3_ESDHC3_PRDF_OFFSET);
			podf = PDR3(MXC_CCM_PDR3_ESDHC3_PODF_MASK,
				    MXC_CCM_PDR3_ESDHC3_PODF_OFFSET);
		} else
			podf = PDR3(MXC_CCM_PDR3_ESDHC3_PODF_MASK_V2,
				    MXC_CCM_PDR3_ESDHC3_PODF_OFFSET);
		break;
	default:
		return;
	}
	clk->rate = clk->parent->rate / ((podf + 1) * (prdf + 1));
}

static int _clk_sdhc_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 prdf, podf;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1)
		__calc_pre_post_dividers(div, &prdf, &podf);

	switch (clk->id) {
	case 0:
		if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
			reg = __raw_readl(MXC_CCM_PDR3) &
			    ~(MXC_CCM_PDR3_ESDHC1_PRDF_MASK |
			      MXC_CCM_PDR3_ESDHC1_PODF_MASK);
			reg |= (podf - 1) << MXC_CCM_PDR3_ESDHC1_PODF_OFFSET;
			reg |= (prdf - 1) << MXC_CCM_PDR3_ESDHC1_PRDF_OFFSET;
		} else {
			reg = __raw_readl(MXC_CCM_PDR3) &
			    ~MXC_CCM_PDR3_ESDHC1_PODF_MASK_V2;
			reg |= (div - 1) << MXC_CCM_PDR3_ESDHC1_PODF_OFFSET;
		}
		break;
	case 1:
		if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
			reg = __raw_readl(MXC_CCM_PDR3) &
			    ~(MXC_CCM_PDR3_ESDHC2_PRDF_MASK |
			      MXC_CCM_PDR3_ESDHC2_PODF_MASK);
			reg |= (podf - 1) << MXC_CCM_PDR3_ESDHC2_PODF_OFFSET;
			reg |= (prdf - 1) << MXC_CCM_PDR3_ESDHC2_PRDF_OFFSET;
		} else {
			reg = __raw_readl(MXC_CCM_PDR3) &
			    ~MXC_CCM_PDR3_ESDHC2_PODF_MASK_V2;
			reg |= (div - 1) << MXC_CCM_PDR3_ESDHC2_PODF_OFFSET;
		}
		break;
	case 2:
		if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
			reg = __raw_readl(MXC_CCM_PDR3) &
			    ~(MXC_CCM_PDR3_ESDHC3_PRDF_MASK |
			      MXC_CCM_PDR3_ESDHC3_PODF_MASK);
			reg |= (podf - 1) << MXC_CCM_PDR3_ESDHC3_PODF_OFFSET;
			reg |= (prdf - 1) << MXC_CCM_PDR3_ESDHC3_PRDF_OFFSET;
		} else {
			reg = __raw_readl(MXC_CCM_PDR3) &
			    ~MXC_CCM_PDR3_ESDHC3_PODF_MASK_V2;
			reg |= (div - 1) << MXC_CCM_PDR3_ESDHC3_PODF_OFFSET;
		}
		break;
	default:
		return -EINVAL;
	}
	__raw_writel(reg, MXC_CCM_PDR3);

	clk->rate = rate;
	return 0;
}

static struct clk ckih_clk = {
	.name = "ckih",
	.rate = CKIH_CLK_FREQ,
	.flags = RATE_FIXED,
};

static struct clk int_32k_clk = {
	.name = "int_32k",
	.rate = CKIL_CLK_FREQ,
	.flags = RATE_FIXED,
};

static struct clk ext_32k_clk = {
	.name = "ext_32k",
	.rate = CKIL_EXT_FREQ,
	.flags = RATE_FIXED,
};

static int _clk_ckil_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	if (parent == &int_32k_clk) {
		reg = __raw_readl(MXC_CCM_PDR0) & (~MXC_CCM_PDR0_CKIL_SEL);
		clk->rate = parent->rate;
	} else if (parent == &ext_32k_clk) {
		reg = __raw_readl(MXC_CCM_PDR0) | MXC_CCM_PDR0_CKIL_SEL;
		clk->rate = parent->rate;
	} else
		return -EINVAL;
	__raw_writel(reg, MXC_CCM_PDR0);
	return 0;
}

static int _clk_ckil_set_rate(struct clk *clk, unsigned long rate)
{
	clk->rate = clk->parent->rate;
	return 0;
}

static struct clk ckil_clk = {
	.name = "ckil",
	.parent = &ext_32k_clk,
	.set_parent = _clk_ckil_set_parent,
	.set_rate = _clk_ckil_set_rate,
};

static int _clk_ckie_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_PMCR2) & ~MXC_CCM_PMCR2_OSC_AUDIO_DOWN;
	__raw_writel(reg, MXC_CCM_PMCR2);

	return 0;
}

static void _clk_ckie_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_PMCR2) | MXC_CCM_PMCR2_OSC_AUDIO_DOWN;
	__raw_writel(reg, MXC_CCM_PMCR2);
}

static struct clk ckie_clk = {
	.name = "ckie",
	.rate = CKIE_CLK_FREQ,
	.flags = RATE_FIXED,
	.enable = _clk_ckie_enable,
	.disable = _clk_ckie_disable,
};

static struct clk mcu_pll_clk = {
	.name = "mcu_pll",
	.parent = &ckih_clk,
	.set_rate = _clk_pll_set_rate,
	.recalc = _clk_pll_recalc,
	.flags = RATE_PROPAGATES,
};

static struct clk peri_pll_clk = {
	.name = "peri_pll",
	.parent = &ckih_clk,
	.set_rate = _clk_pll_set_rate,
	.recalc = _clk_pll_recalc,
	.enable = _clk_peri_pll_enable,
	.disable = _clk_peri_pll_disable,
	.flags = RATE_PROPAGATES,
};

static struct clk cpu_clk = {
	.name = "cpu_clk",
	.parent = &mcu_pll_clk,
	.recalc = _clk_cpu_recalc,
	.set_rate = _clk_cpu_set_rate,
};

static struct clk ahb_clk = {
	.name = "ahb_clk",
	.parent = &cpu_clk,
	.recalc = _clk_hclk_recalc,
	.flags = RATE_PROPAGATES,
};

static struct clk ipg_clk = {
	.name = "ipg_clk",
	.parent = &ahb_clk,
	.recalc = _clk_ipg_recalc,
	.flags = RATE_PROPAGATES,
};

static struct clk perclk_clk = {
	.name = "perclk_clk",
	.parent = &ahb_clk,
	.recalc = _clk_per_recalc,
	.flags = RATE_PROPAGATES,
};

static struct clk uart_per_clk = {
	.name = "uart_per_clk",
	.parent = &peri_pll_clk,
	.recalc = _clk_uart_per_recalc,
	.round_rate = _clk_round_rate,
	.set_rate = _clk_uart_set_rate,
	.flags = RATE_PROPAGATES,
};

static struct clk asrc_clk[] = {
	{
	 .name = "asrc_clk",
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR0,
	 .enable_shift = MXC_CCM_CGR0_ASRC_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "asrc_audio_clk",
	 .parent = &ckie_clk,
	 .recalc = _clk_asrc_recalc,
	 .round_rate = _clk_round_rate,
	 .set_rate = _clk_asrc_set_rate,
	 .enable = _clk_asrc_enable,
	 .disable = _clk_asrc_disable,},
};

static struct clk ata_clk = {
	.name = "ata_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR0,
	.enable_shift = MXC_CCM_CGR0_ATA_OFFSET,
	.disable = _clk_disable,
};

static struct clk can_clk[] = {
	{
	 .name = "can_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR0,
	 .enable_shift = MXC_CCM_CGR0_CAN1_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "can_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR0,
	 .enable_shift = MXC_CCM_CGR0_CAN2_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk cspi_clk[] = {
	{
	 .name = "cspi_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR0,
	 .enable_shift = MXC_CCM_CGR0_CSPI1_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "cspi_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR0,
	 .enable_shift = MXC_CCM_CGR0_CSPI2_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk ect_clk = {
	.name = "ect_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR0,
	.enable_shift = MXC_CCM_CGR0_ECT_OFFSET,
	.disable = _clk_disable,
};

static struct clk emi_clk = {
	.name = "emi_clk",
	.parent = &ahb_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR0,
	.enable_shift = MXC_CCM_CGR0_EMI_OFFSET,
	.disable = _clk_emi_disable,
};

static struct clk epit_clk[] = {
	{
	 .name = "epit_clk",
	 .id = 0,
	 .parent = &perclk_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR0,
	 .enable_shift = MXC_CCM_CGR0_EPIT1_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "epit_clk",
	 .id = 1,
	 .parent = &perclk_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR0,
	 .enable_shift = MXC_CCM_CGR0_EPIT2_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk esai_clk = {
	.name = "esai_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR0,
	.enable_shift = MXC_CCM_CGR0_ESAI_OFFSET,
	.disable = _clk_disable,
};

static struct clk sdhc_clk[] = {
	{
	 .name = "sdhc_clk",
	 .id = 0,
	 .parent = &peri_pll_clk,
	 .recalc = _clk_sdhc_recalc,
	 .set_rate = _clk_sdhc_set_rate,
	 .round_rate = _clk_round_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR0,
	 .enable_shift = MXC_CCM_CGR0_ESDHC1_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "sdhc_clk",
	 .id = 1,
	 .parent = &peri_pll_clk,
	 .recalc = _clk_sdhc_recalc,
	 .set_rate = _clk_sdhc_set_rate,
	 .round_rate = _clk_round_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR0,
	 .enable_shift = MXC_CCM_CGR0_ESDHC2_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "sdhc_clk",
	 .id = 2,
	 .parent = &peri_pll_clk,
	 .recalc = _clk_sdhc_recalc,
	 .set_rate = _clk_sdhc_set_rate,
	 .round_rate = _clk_round_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR0,
	 .enable_shift = MXC_CCM_CGR0_ESDHC3_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk fec_clk = {
	.name = "fec_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR1,
	.enable_shift = MXC_CCM_CGR1_FEC_OFFSET,
	.disable = _clk_disable,
};

static struct clk gpt_clk = {
	.name = "gpt_clk",
	.parent = &perclk_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR1,
	.enable_shift = MXC_CCM_CGR1_GPT_OFFSET,
	.disable = _clk_disable,
};

static struct clk i2c_clk[] = {
	{
	 .name = "i2c_clk",
	 .id = 0,
	 .parent = &perclk_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR1,
	 .enable_shift = MXC_CCM_CGR1_I2C1_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "i2c_clk",
	 .id = 1,
	 .parent = &perclk_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR1,
	 .enable_shift = MXC_CCM_CGR1_I2C2_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "i2c_clk",
	 .id = 2,
	 .parent = &perclk_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR1,
	 .enable_shift = MXC_CCM_CGR1_I2C3_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk ipu_clk = {
	.name = "ipu_clk",
	.parent = &cpu_clk,
	.recalc = _clk_hsp_recalc,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR1,
	.enable_shift = MXC_CCM_CGR1_IPU_OFFSET,
	.disable = _clk_disable,
};

static struct clk kpp_clk = {
	.name = "kpp_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR1,
	.enable_shift = MXC_CCM_CGR1_KPP_OFFSET,
	.disable = _clk_disable,
};

static struct clk mlb_clk = {
	.name = "mlb_clk",
	.parent = &ahb_clk,
	.recalc = _clk_mlb_recalc,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR1,
	.enable_shift = MXC_CCM_CGR1_MLB_OFFSET,
	.disable = _clk_disable,
};

static struct clk mstick_clk = {
	.name = "mstick_clk",
	.id = 0,
	.parent = &peri_pll_clk,
	.recalc = _clk_mstick1_recalc,
	.set_rate = _clk_mstick1_set_rate,
	.round_rate = _clk_per_post_round_rate,
	.set_parent = _clk_mstick1_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR1,
	.enable_shift = MXC_CCM_CGR1_MSHC_OFFSET,
	.disable = _clk_disable,
};

static struct clk owire_clk = {
	.name = "owire_clk",
	.parent = &perclk_clk,
	.enable_reg = MXC_CCM_CGR1,
	.enable_shift = MXC_CCM_CGR1_OWIRE_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk pwm_clk = {
	.name = "pwm_clk",
	.parent = &perclk_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR1,
	.enable_shift = MXC_CCM_CGR1_PWM_OFFSET,
	.disable = _clk_disable,
};

static struct clk rng_clk = {
	.name = "rng_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR1,
	.enable_shift = MXC_CCM_CGR1_RNGC_OFFSET,
	.disable = _clk_disable,
};

static struct clk rtc_clk = {
	.name = "rtc_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR2,
	.enable_shift = MXC_CCM_CGR2_RTC_OFFSET,
	.disable = _clk_disable,
};

static struct clk rtic_clk = {
	.name = "rtic_clk",
	.parent = &ahb_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR2,
	.enable_shift = MXC_CCM_CGR2_RTIC_OFFSET,
	.disable = _clk_disable,
};

static struct clk scc_clk = {
	.name = "scc_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR2,
	.enable_shift = MXC_CCM_CGR2_SCC_OFFSET,
	.disable = _clk_disable,
};

static struct clk sdma_clk[] = {
	{
	 .name = "sdma_ahb_clk",
	 .parent = &ahb_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR2,
	 .enable_shift = MXC_CCM_CGR2_SDMA_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "sdma_ipg_clk",
	 .parent = &ipg_clk,}
};

static struct clk spba_clk = {
	.name = "spba_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR2,
	.enable_shift = MXC_CCM_CGR2_SPBA_OFFSET,
	.disable = _clk_disable,
};

static struct clk spdif_clk[] = {
	{
	 .name = "spdif_clk",
	 .parent = &peri_pll_clk,
	 .recalc = _clk_spdif_recalc,
	 .set_rate = _clk_spdif_set_rate,
	 .round_rate = _clk_per_post_round_rate,
	 .set_parent = _clk_spdif_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR2,
	 .enable_shift = MXC_CCM_CGR2_SPDIF_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "spdif_audio_clk",
	 .parent = &ckie_clk,},
	{
	 .name = "spdif_ipg_clk",
	 .parent = &ipg_clk,},
};

static struct clk ssi_clk[] = {
	{
	 .name = "ssi_clk",
	 .parent = &peri_pll_clk,
	 .recalc = _clk_ssi_recalc,
	 .set_rate = _clk_ssi_set_rate,
	 .round_rate = _clk_per_post_round_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR2,
	 .enable_shift = MXC_CCM_CGR2_SSI1_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "ssi_clk",
	 .id = 1,
	 .parent = &peri_pll_clk,
	 .recalc = _clk_ssi_recalc,
	 .set_rate = _clk_ssi_set_rate,
	 .round_rate = _clk_per_post_round_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR2,
	 .enable_shift = MXC_CCM_CGR2_SSI2_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk uart_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 0,
	 .parent = &uart_per_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR2,
	 .enable_shift = MXC_CCM_CGR2_UART1_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "uart_clk",
	 .id = 1,
	 .parent = &uart_per_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR2,
	 .enable_shift = MXC_CCM_CGR2_UART2_OFFSET,
	 .disable = _clk_disable,},
	{
	 .name = "uart_clk",
	 .id = 2,
	 .parent = &uart_per_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR2,
	 .enable_shift = MXC_CCM_CGR2_UART3_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk usb_clk[] = {
	{
	 .name = "usb_clk",
	 .parent = &peri_pll_clk,
	 .recalc = _clk_usb_recalc,
	 .round_rate = _clk_round_rate,
	 .set_rate = _clk_usb_set_rate,},
	{
	 .name = "usb_ahb_clk",
	 .parent = &ahb_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CGR2,
	 .enable_shift = MXC_CCM_CGR2_USBOTG_OFFSET,
	 .disable = _clk_disable,},
};

static struct clk wdog_clk = {
	.name = "wdog_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR2,
	.enable_shift = MXC_CCM_CGR2_WDOG_OFFSET,
	.disable = _clk_disable,
};

static struct clk csi_clk = {
	.name = "csi_clk",
	.parent = &peri_pll_clk,
	.recalc = _clk_csi_recalc,
	.round_rate = _clk_round_rate,
	.set_rate = _clk_csi_set_rate,
	.set_parent = _clk_csi_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR3,
	.enable_shift = MXC_CCM_CGR3_CSI_OFFSET,
	.disable = _clk_disable,
};

static struct clk iim_clk = {
	.name = "iim_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR3,
	.enable_shift = MXC_CCM_CGR3_IIM_OFFSET,
	.disable = _clk_disable,
};

static struct clk nfc_clk = {
	.name = "nfc_clk",
	.parent = &ahb_clk,
	.recalc = _clk_nfc_recalc,
};

static unsigned long _clk_cko1_round_rate(struct clk *clk, unsigned long rate)
{
	u32 div = 0, div1 = 1;

	div = clk->parent->rate / rate;
	if (clk->parent->rate % rate)
		div++;

	if (div > 64) {
		div = (div + 1) >> 1;
		div1++;
	}

	if (div > 128)
		div = 64;
	return clk->parent->rate / (div * div1);
}

static int _clk_cko1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div, div1 = 0;
	u32 prdf, podf;

	div = clk->parent->rate / rate;
	if ((clk->parent->rate / div) != rate)
		return -EINVAL;
	if (div > 64) {
		div1 = MXC_CCM_COSR_CLKOUTDIV_1;
		div >>= 1;
	} else {
		div1 = 0;
	}

	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		__calc_two_dividers(div, &prdf, &podf);
		reg = __raw_readl(MXC_CCM_COSR) &
		    ~(MXC_CCM_COSR_CLKOUT_PREDIV_MASK |
		      MXC_CCM_COSR_CLKOUT_PRODIV_MASK |
		      MXC_CCM_COSR_CLKOUTDIV_1);
		reg |= ((prdf - 1) << MXC_CCM_COSR_CLKOUT_PREDIV_OFFSET)
		    | ((podf - 1) << MXC_CCM_COSR_CLKOUT_PRODIV_OFFSET)
		    | div1;
	} else {
		reg = __raw_readl(MXC_CCM_COSR) &
		    ~(MXC_CCM_COSR_CLKOUT_PRODIV_MASK_V2 |
		      MXC_CCM_COSR_CLKOUTDIV_1);
		reg |= ((div - 1) << MXC_CCM_COSR_CLKOUT_PRODIV_OFFSET) | div1;
	}
	__raw_writel(reg, MXC_CCM_COSR);

	return 0;
}

static void _clk_cko1_recalc(struct clk *clk)
{
	u32 prdf = 1;
	u32 podf, div1;
	u32 reg = __raw_readl(MXC_CCM_COSR);

	div1 = 1 << ((reg & MXC_CCM_COSR_CLKOUTDIV_1) != 0);
	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		prdf = (reg & MXC_CCM_COSR_CLKOUT_PREDIV_MASK) >>
		    MXC_CCM_COSR_CLKOUT_PREDIV_OFFSET;
		podf = (reg & MXC_CCM_COSR_CLKOUT_PRODIV_MASK) >>
		    MXC_CCM_COSR_CLKOUT_PRODIV_OFFSET;
	} else
		podf = (reg & MXC_CCM_COSR_CLKOUT_PRODIV_MASK_V2) >>
		    MXC_CCM_COSR_CLKOUT_PRODIV_OFFSET;

	clk->rate = clk->parent->rate / (div1 * (podf + 1) * (prdf + 1));
}

static int _clk_cko1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	reg = __raw_readl(MXC_CCM_COSR) & ~MXC_CCM_COSR_CLKOSEL_MASK;

	if (parent == &ckil_clk) {
		reg &= ~MXC_CCM_COSR_CKIL_CKIH_MASK;
		reg |= 0 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	} else if (parent == &ckih_clk) {
		reg |= 1 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	} else if (parent == &ckie_clk)
		reg |= 2 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &peri_pll_clk)
		reg |= 6 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &cpu_clk)
		reg |= 7 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &ahb_clk)
		reg |= 8 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &ipg_clk)
		reg |= 9 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &usb_clk[1])
		reg |= 0xB << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &sdhc_clk[1])
		reg |= 0xC << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &ssi_clk[1])
		reg |= 0xD << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &mlb_clk)
		reg |= 0xE << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &csi_clk)
		reg |= 0x11 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &spdif_clk[0])
		reg |= 0x12 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &uart_clk[0])
		reg |= 0x13 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if (parent == &asrc_clk[1])
		reg |= 0x14 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if ((parent == &nfc_clk) && (cpu_is_mx35_rev(CHIP_REV_2_0) >= 1))
		reg |= 0x17 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else if ((parent == &ipu_clk) && (cpu_is_mx35_rev(CHIP_REV_2_0) >= 1))
		reg |= 0x18 << MXC_CCM_COSR_CLKOSEL_OFFSET;
	else
		return -EINVAL;

	__raw_writel(reg, MXC_CCM_COSR);
	return 0;
}

static int _clk_cko1_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_COSR) | MXC_CCM_COSR_CLKOEN;
	__raw_writel(reg, MXC_CCM_COSR);

	return 0;
}

static void _clk_cko1_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_COSR) & ~MXC_CCM_COSR_CLKOEN;
	__raw_writel(reg, MXC_CCM_COSR);
}

static struct clk cko1_clk = {
	.name = "cko1_clk",
	.recalc = _clk_cko1_recalc,
	.set_rate = _clk_cko1_set_rate,
	.round_rate = _clk_cko1_round_rate,
	.set_parent = _clk_cko1_set_parent,
	.enable = _clk_cko1_enable,
	.disable = _clk_cko1_disable,
};

static struct clk gpu2d_clk = {
	.name = "gpu2d_clk",
	.parent = &ahb_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CGR3,
	.enable_shift = MXC_CCM_CGR3_GPU2D_OFFSET,
	.disable = _clk_disable,
};

static struct clk *mxc_clks[] = {
	&int_32k_clk,
	&ext_32k_clk,
	&ckih_clk,
	&ckil_clk,
	&ckie_clk,
	&mcu_pll_clk,
	&peri_pll_clk,
	&cpu_clk,
	&ahb_clk,
	&ipg_clk,
	&perclk_clk,
	&uart_per_clk,
	&asrc_clk[0],
	&asrc_clk[1],
	&ata_clk,
	&can_clk[0],
	&can_clk[1],
	&cspi_clk[0],
	&cspi_clk[1],
	&ect_clk,
	&emi_clk,
	&epit_clk[0],
	&epit_clk[1],
	&esai_clk,
	&sdhc_clk[0],
	&sdhc_clk[1],
	&sdhc_clk[2],
	&fec_clk,
	&gpt_clk,
	&i2c_clk[0],
	&i2c_clk[1],
	&i2c_clk[2],
	&ipu_clk,
	&kpp_clk,
	&mlb_clk,
	&mstick_clk,
	&owire_clk,
	&rng_clk,
	&pwm_clk,
	&rtc_clk,
	&rtic_clk,
	&scc_clk,
	&sdma_clk[0],
	&sdma_clk[1],
	&spba_clk,
	&spdif_clk[0],
	&spdif_clk[1],
	&spdif_clk[2],
	&ssi_clk[0],
	&ssi_clk[1],
	&uart_clk[0],
	&uart_clk[1],
	&uart_clk[2],
	&usb_clk[0],
	&usb_clk[1],
	&wdog_clk,
	&csi_clk,
	&iim_clk,
	&nfc_clk,
	&cko1_clk,
	&gpu2d_clk,
};

extern void propagate_rate(struct clk *tclk);

static void mxc_clockout_scan(void)
{
	u32 reg = __raw_readl(MXC_CCM_COSR) & MXC_CCM_COSR_CLKOSEL_MASK;
	reg >>= MXC_CCM_COSR_CLKOSEL_OFFSET;
	switch (reg) {
	case 0:
		cko1_clk.parent = &ckil_clk;
		break;
	case 1:
		cko1_clk.parent = &ckih_clk;
		break;
	case 2:
		cko1_clk.parent = &ckie_clk;
		break;
	case 6:
		cko1_clk.parent = &peri_pll_clk;
		break;
	case 7:
		cko1_clk.parent = &cpu_clk;
		break;
	case 8:
		cko1_clk.parent = &ahb_clk;
		break;
	case 9:
		cko1_clk.parent = &ipg_clk;
		break;
	case 0xB:
		cko1_clk.parent = &usb_clk[1];
		break;
	case 0xC:
		cko1_clk.parent = &sdhc_clk[1];
		break;
	case 0xD:
		cko1_clk.parent = &ssi_clk[1];
		break;
	case 0xE:
		cko1_clk.parent = &mlb_clk;
		break;
	case 0x11:
		cko1_clk.parent = &csi_clk;
		break;
	case 0x12:
		cko1_clk.parent = &spdif_clk[0];
		break;
	case 0x13:
		cko1_clk.parent = &uart_clk[0];
		break;
	case 0x14:
		cko1_clk.parent = &asrc_clk[1];
		break;
	case 0x17:
		cko1_clk.parent = &nfc_clk;
		break;
	case 0x18:
		cko1_clk.parent = &ipu_clk;
		break;
	}
}

static void mxc_update_clocks(void)
{
	unsigned long reg;
	reg = __raw_readl(MXC_CCM_PDR0);
	if ((!(reg & MXC_CCM_PDR0_AUTO_CON))
	    && (cpu_is_mx35_rev(CHIP_REV_2_0) < 1))
		ipu_clk.parent = &ahb_clk;

	if (reg & MXC_CCM_PDR0_PER_SEL)
		perclk_clk.parent = &cpu_clk;

	reg = __raw_readl(MXC_CCM_PDR1);
	if (reg & MXC_CCM_PDR1_MSHC_M_U)
		mstick_clk.parent = &cpu_clk;

	reg = __raw_readl(MXC_CCM_PDR2);
	if (reg & MXC_CCM_PDR2_CSI_M_U)
		csi_clk.parent = &cpu_clk;
	if (reg & MXC_CCM_PDR2_SSI_M_U) {
		ssi_clk[0].parent = &cpu_clk;
		ssi_clk[1].parent = &cpu_clk;
	}

	reg = __raw_readl(MXC_CCM_PDR3);
	if (reg & MXC_CCM_PDR3_SPDIF_M_U)
		spdif_clk[0].parent = &cpu_clk;

	if (reg & MXC_CCM_PDR3_UART_M_U)
		uart_per_clk.parent = &cpu_clk;

	if (reg & MXC_CCM_PDR3_ESDHC_M_U) {
		sdhc_clk[0].parent = &cpu_clk;
		sdhc_clk[1].parent = &cpu_clk;
		sdhc_clk[2].parent = &cpu_clk;
	}

	reg = __raw_readl(MXC_CCM_PDR4);
	if (reg & MXC_CCM_PDR4_USB_M_U)
		usb_clk[0].parent = &cpu_clk;

	mxc_clockout_scan();
}

int __init mx35_clocks_init(void)
{
	struct clk **clkp;
	for (clkp = mxc_clks; clkp < mxc_clks + ARRAY_SIZE(mxc_clks); clkp++)
		clk_register(*clkp);

	/* Turn off all possible clocks */
	__raw_writel(MXC_CCM_CGR0_ECT_MASK | MXC_CCM_CGR0_EMI_MASK |
		     MXC_CCM_CGR0_ESDHC1_MASK | MXC_CCM_CGR0_ESDHC2_MASK |
		     MXC_CCM_CGR0_ESDHC3_MASK,
		     MXC_CCM_CGR0);
	__raw_writel(MXC_CCM_CGR1_GPIO1_MASK | MXC_CCM_CGR1_GPIO2_MASK |
		     MXC_CCM_CGR1_GPIO3_MASK | MXC_CCM_CGR1_GPT_MASK |
		     MXC_CCM_CGR1_IOMUXC_MASK, MXC_CCM_CGR1);
	__raw_writel(MXC_CCM_CGR2_MAX_MASK | MXC_CCM_CGR2_SPBA_MASK |
		     MXC_CCM_CGR2_AUDMUX_MASK | MXC_CCM_CGR2_MAX_ENABLE,
		     MXC_CCM_CGR2);
	__raw_writel(MXC_CCM_CGR3_IIM_MASK, MXC_CCM_CGR3);
	__raw_writel((__raw_readl(MXC_CCM_PMCR2) |
		      MXC_CCM_PMCR2_OSC24M_DOWN |
		      MXC_CCM_PMCR2_OSC_AUDIO_DOWN), MXC_CCM_PMCR2);
	mxc_update_clocks();
	pr_info("Clock input source is %ld\n", ckih_clk.rate);

	/* Determine which high frequency clock source is coming in */
	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	sync_cpu_wb();

	/* This will propagate to all children and init all the clock rates */
	propagate_rate(&ckih_clk);
	propagate_rate(&ext_32k_clk);
	propagate_rate(&ckie_clk);

	clk_enable(&mcu_pll_clk);
	clk_enable(&gpt_clk);
	clk_enable(&emi_clk);
	clk_enable(&iim_clk);
	clk_enable(&spba_clk);

	/* Init serial PLL according */
	clk_set_rate(&peri_pll_clk, 300000000);

	clk_enable(&peri_pll_clk);

	mxc_timer_init(&gpt_clk, IO_ADDRESS(GPT1_BASE_ADDR), MXC_INT_GPT);

	return 0;
}
