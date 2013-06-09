
/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/clkdev.h>
#include <linux/regulator/consumer.h>
#include <asm/div64.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/clock.h>
#include <mach/mxc_dvfs.h>
#include <mach/ahci_sata.h>
#include <mach/mxc_hdmi.h>
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "regs-anadig.h"

#ifdef CONFIG_CLK_DEBUG
#define __INIT_CLK_DEBUG(n)	.name = #n,
#else
#define __INIT_CLK_DEBUG(n)
#endif

extern u32 arm_max_freq;
extern int mxc_jtag_enabled;
extern struct regulator *cpu_regulator;
extern struct cpu_op *(*get_cpu_op)(int *op);
extern int lp_high_freq;
extern int lp_med_freq;
extern int wait_mode_arm_podf;
extern int lp_audio_freq;
extern int cur_arm_podf;
extern bool enet_is_active;

void __iomem *apll_base;

static void __iomem *timer_base;
static struct clk ipu1_clk;
static struct clk ipu2_clk;
static struct clk axi_clk;
static struct clk pll1_sys_main_clk;
static struct clk pll2_528_bus_main_clk;
static struct clk pll2_pfd_400M;
static struct clk pll2_pfd_352M;
static struct clk pll3_pfd_540M;
static struct clk pll2_pfd_594M;
static struct clk pll3_usb_otg_main_clk;
static struct clk pll4_audio_main_clk;
static struct clk pll5_video_main_clk;
static struct clk pll6_mlb150_main_clk;
static struct clk pll7_usb_host_main_clk;
static struct clk pll8_enet_main_clk;
static struct clk apbh_dma_clk;
static struct clk openvg_axi_clk;
static struct clk enfc_clk;
static struct clk usdhc3_clk;
static struct clk ipg_clk;
static struct clk gpt_clk[];
static struct clk clko2_clk;

static struct cpu_op *cpu_op_tbl;
static int cpu_op_nr;
static bool pll1_enabled;
static bool arm_needs_pll2_400;

#define SPIN_DELAY	1200000 /* in nanoseconds */

#define AUDIO_VIDEO_MIN_CLK_FREQ	650000000
#define AUDIO_VIDEO_MAX_CLK_FREQ	1300000000
DEFINE_SPINLOCK(clk_lock);
#define V2_TCN			0x24
#define V2_TSTAT		0x08
#define V2_TSTAT_ROV	(1 << 5)
#define V2_TCTL_CLK_OSC_DIV8	 (5 << 6)
#define MXC_TCTL		0x00
#define MXC_TPRER		0x04
#define V2_TPRER_PRE24M_OFFSET	12
#define V2_TPRER_PRE24M_MASK	0xF

/* We need to check the exp status again after timer expiration,
 * as there might be interrupt coming between the first time exp
 * and the time reading, then the time reading may be several ms
 * after the exp checking due to the irq handle, so we need to
 * check it to make sure the exp return the right value after
 * timer expiration. */
#define WAIT(exp, timeout) \
({ \
	u32 gpt_rate; \
	u32 gpt_ticks; \
	u32 gpt_cnt; \
	u32 reg; \
	int result = 1; \
	gpt_rate = clk_get_rate(&gpt_clk[0]); \
	gpt_ticks = timeout / (1000000000 / gpt_rate); \
	reg = __raw_readl(timer_base + V2_TSTAT);\
	/* Clear the GPT roll over interrupt. */ \
	if (reg & V2_TSTAT_ROV) { \
		reg |= V2_TSTAT_ROV;\
		__raw_writel(reg, timer_base + V2_TSTAT);\
	} \
	gpt_cnt = __raw_readl(timer_base + V2_TCN); \
	while (!(exp)) { \
		if ((__raw_readl(timer_base + V2_TCN) - gpt_cnt) > gpt_ticks) { \
			if (!exp) \
				result = 0; \
			break; \
		} else { \
			reg = __raw_readl(timer_base + V2_TSTAT);\
			if (reg & V2_TSTAT_ROV) { \
				u32 old_cnt = gpt_cnt; \
				/* Timer has rolled over. \
				  * Calculate the new tcik count. \
				  */ \
				gpt_cnt = __raw_readl(timer_base + V2_TCN); \
				gpt_ticks -= (0xFFFFFFFF - old_cnt + gpt_cnt); \
				/* Clear the roll over interrupt. */ \
				reg |= V2_TSTAT_ROV;\
				__raw_writel(reg, timer_base + V2_TSTAT);\
			} \
		} \
	} \
	result; \
})

/* External clock values passed-in by the board code */
static unsigned long external_high_reference, external_low_reference;
static unsigned long oscillator_reference, ckih2_reference;
static unsigned long anaclk_1_reference, anaclk_2_reference;

/* For MX 6DL/S, Video PLL may be used by synchronous display devices,
 * such as HDMI or LVDS, and also by the EPDC.  If EPDC is in use,
 * it must use the Video PLL to achieve the clock frequencies it needs.
 * So if EPDC is in use, the "epdc" string should be added to kernel
 * parameters, in order to set the EPDC parent clock to the Video PLL.
 * This will have an impact on the behavior of HDMI and LVDS.
 */
int epdc_enabled;
static int __init epdc_setup(char *__unused)
{
	epdc_enabled = 1;
	return 1;
}
__setup("epdc", epdc_setup);

static void __calc_pre_post_dividers(u32 max_podf, u32 div, u32 *pre, u32 *post)
{
	u32 min_pre, temp_pre, old_err, err;

	/* Some of the podfs are 3 bits while others are 6 bits.
	  * Handle both cases here.
	  */
	if (div >= 512 && (max_podf == 64)) {
		/* For pre = 3bits and podf = 6 bits, max divider is 512. */
		*pre = 8;
		*post = 64;
	} else if (div >= 64 && (max_podf == 8)) {
		/* For pre = 3bits and podf = 3 bits, max divider is 64. */
		*pre = 8;
		*post = 8;
	} else if (div >= 8) {
		/* Find the minimum pre-divider for a max podf */
		if (max_podf == 64)
			min_pre = (div - 1) / (1 << 6) + 1;
		else
			min_pre = (div - 1) / (1 << 3) + 1;
		old_err = 8;
		/* Now loop through to find the max pre-divider. */
		for (temp_pre = 8; temp_pre >= min_pre; temp_pre--) {
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
	} else if (div < 8) {
		*pre = div;
		*post = 1;
	}
}

static int _clk_enable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg |= MXC_CCM_CCGRx_CG_MASK << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	return 0;
}

static void _clk_disable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGRx_CG_MASK << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);
}

static void _clk_disable_inwait(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGRx_CG_MASK << clk->enable_shift);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);
}


/*
 * For the 4-to-1 muxed input clock
 */
static inline u32 _get_mux(struct clk *parent, struct clk *m0,
			   struct clk *m1, struct clk *m2, struct clk *m3)
{
	if (parent == m0)
		return 0;
	else if (parent == m1)
		return 1;
	else if (parent == m2)
		return 2;
	else if (parent == m3)
		return 3;
	else
		BUG();

	return 0;
}

static inline void __iomem *_get_pll_base(struct clk *pll)
{
	if (pll == &pll1_sys_main_clk)
		return PLL1_SYS_BASE_ADDR;
	else if (pll == &pll2_528_bus_main_clk)
		return PLL2_528_BASE_ADDR;
	else if (pll == &pll3_usb_otg_main_clk)
		return PLL3_480_USB1_BASE_ADDR;
	else if (pll == &pll4_audio_main_clk)
		return PLL4_AUDIO_BASE_ADDR;
	else if (pll == &pll5_video_main_clk)
		return PLL5_VIDEO_BASE_ADDR;
	else if (pll == &pll6_mlb150_main_clk)
		return PLL6_MLB_BASE_ADDR;
	else if (pll == &pll7_usb_host_main_clk)
		return PLL7_480_USB2_BASE_ADDR;
	else if (pll == &pll8_enet_main_clk)
		return PLL8_ENET_BASE_ADDR;
	else
		BUG();
	return NULL;
}


/*
 * For the 6-to-1 muxed input clock
 */
static inline u32 _get_mux6(struct clk *parent, struct clk *m0, struct clk *m1,
			    struct clk *m2, struct clk *m3, struct clk *m4,
			    struct clk *m5)
{
	if (parent == m0)
		return 0;
	else if (parent == m1)
		return 1;
	else if (parent == m2)
		return 2;
	else if (parent == m3)
		return 3;
	else if (parent == m4)
		return 4;
	else if (parent == m5)
		return 5;
	else
		BUG();

	return 0;
}
static unsigned long get_high_reference_clock_rate(struct clk *clk)
{
	return external_high_reference;
}

static unsigned long get_low_reference_clock_rate(struct clk *clk)
{
	return external_low_reference;
}

static unsigned long get_oscillator_reference_clock_rate(struct clk *clk)
{
	return oscillator_reference;
}

static unsigned long get_ckih2_reference_clock_rate(struct clk *clk)
{
	return ckih2_reference;
}

static unsigned long _clk_anaclk_1_get_rate(struct clk *clk)
{
	return anaclk_1_reference;
}

static int _clk_anaclk_1_set_rate(struct clk *clk, unsigned long rate)
{
	anaclk_1_reference = rate;
	return 0;
}

static unsigned long _clk_anaclk_2_get_rate(struct clk *clk)
{
	return anaclk_2_reference;
}

static int _clk_anaclk_2_set_rate(struct clk *clk, unsigned long rate)
{
	anaclk_2_reference = rate;
	return 0;
}

/* External high frequency clock */
static struct clk ckih_clk = {
	__INIT_CLK_DEBUG(ckih_clk)
	.get_rate = get_high_reference_clock_rate,
};

static struct clk ckih2_clk = {
	__INIT_CLK_DEBUG(ckih2_clk)
	.get_rate = get_ckih2_reference_clock_rate,
};

static struct clk osc_clk = {
	__INIT_CLK_DEBUG(osc_clk)
	.get_rate = get_oscillator_reference_clock_rate,
};

/* External low frequency (32kHz) clock */
static struct clk ckil_clk = {
	__INIT_CLK_DEBUG(ckil_clk)
	.get_rate = get_low_reference_clock_rate,
};

static struct clk anaclk_1 = {
	__INIT_CLK_DEBUG(anaclk_1)
	.get_rate = _clk_anaclk_1_get_rate,
	.set_rate = _clk_anaclk_1_set_rate,
};

static struct clk anaclk_2 = {
	__INIT_CLK_DEBUG(anaclk_2)
	.get_rate = _clk_anaclk_2_get_rate,
	.set_rate = _clk_anaclk_2_set_rate,
};

static unsigned long pfd_round_rate(struct clk *clk, unsigned long rate)
{
	u32 frac;
	u64 tmp;

	tmp = (u64)clk_get_rate(clk->parent) * 18;
	tmp += rate/2;
	do_div(tmp, rate);
	frac = tmp;
	frac = frac < 12 ? 12 : frac;
	frac = frac > 35 ? 35 : frac;
	tmp = (u64)clk_get_rate(clk->parent) * 18;
	do_div(tmp, frac);
	return tmp;
}

static unsigned long pfd_get_rate(struct clk *clk)
{
	u32 frac;
	u64 tmp;
	tmp = (u64)clk_get_rate(clk->parent) * 18;

	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.enable(&apbh_dma_clk);

	frac = (__raw_readl(clk->enable_reg) >> clk->enable_shift) &
			ANADIG_PFD_FRAC_MASK;

	do_div(tmp, frac);

	return tmp;
}

static int pfd_set_rate(struct clk *clk, unsigned long rate)
{
	u32 frac;
	u64 tmp;
	tmp = (u64)clk_get_rate(clk->parent) * 18;

	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.enable(&apbh_dma_clk);

	/* Round up the divider so that we don't set a rate
	  * higher than what is requested. */
	tmp += rate/2;
	do_div(tmp, rate);
	frac = tmp;
	frac = frac < 12 ? 12 : frac;
	frac = frac > 35 ? 35 : frac;
	/* clear clk frac bits */
	__raw_writel(ANADIG_PFD_FRAC_MASK << clk->enable_shift,
			(int)clk->enable_reg + 8);
	/* set clk frac bits */
	__raw_writel(frac << clk->enable_shift,
			(int)clk->enable_reg + 4);

	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.disable(&apbh_dma_clk);
	return 0;
}

static int _clk_pfd_enable(struct clk *clk)
{
	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.enable(&apbh_dma_clk);

	/* clear clk gate bit */
	__raw_writel((1 << (clk->enable_shift + 7)),
			(int)clk->enable_reg + 8);

	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.disable(&apbh_dma_clk);

	return 0;
}

static void _clk_pfd_disable(struct clk *clk)
{
	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.enable(&apbh_dma_clk);

	/* set clk gate bit */
	__raw_writel((1 << (clk->enable_shift + 7)),
			(int)clk->enable_reg + 4);

	if (apbh_dma_clk.usecount == 0)
		apbh_dma_clk.disable(&apbh_dma_clk);
}

static int _clk_pll_enable(struct clk *clk)
{
	unsigned int reg;
	void __iomem *pllbase;

	pllbase = _get_pll_base(clk);

	reg = __raw_readl(pllbase);
	reg &= ~ANADIG_PLL_POWER_DOWN;

	/* The 480MHz PLLs have the opposite definition for power bit. */
	if (clk == &pll3_usb_otg_main_clk || clk == &pll7_usb_host_main_clk)
		reg |= ANADIG_PLL_POWER_DOWN;

	__raw_writel(reg, pllbase);

	/* It will power on pll3 */
	if (clk == &pll3_usb_otg_main_clk)
		__raw_writel(BM_ANADIG_ANA_MISC2_CONTROL0, apll_base + HW_ANADIG_ANA_MISC2_CLR);

	/* Wait for PLL to lock */
	if (!WAIT((__raw_readl(pllbase) & ANADIG_PLL_LOCK),
				SPIN_DELAY))
		panic("pll enable failed\n");

	/* Enable the PLL output now*/
	reg = __raw_readl(pllbase);
	reg &= ~ANADIG_PLL_BYPASS;
	reg |= ANADIG_PLL_ENABLE;
	__raw_writel(reg, pllbase);

	return 0;
}

static void _clk_pll_disable(struct clk *clk)
{
	unsigned int reg;
	void __iomem *pllbase;

	if ((arm_needs_pll2_400) && (clk == &pll2_528_bus_main_clk))
		return;
	/*
	 * To support USB remote wake up, need always keep power and enable bit
	 * BM_ANADIG_ANA_MISC2_CONTROL0 will power off PLL3's power
	 * Please see TKT064178 for detail.
	 */
	if (clk == &pll3_usb_otg_main_clk) {
		__raw_writel(BM_ANADIG_ANA_MISC2_CONTROL0, apll_base + HW_ANADIG_ANA_MISC2_SET);
		return;
	}

	pllbase = _get_pll_base(clk);

	reg = __raw_readl(pllbase);
	reg |= ANADIG_PLL_BYPASS;
	reg &= ~ANADIG_PLL_ENABLE;

	__raw_writel(reg, pllbase);

}

static unsigned long  _clk_pll1_main_get_rate(struct clk *clk)
{
	unsigned int div;
	unsigned long val;

	/* If PLL1 is bypassed, its rate will be from OSC directly */
	if (__raw_readl(PLL1_SYS_BASE_ADDR) & ANADIG_PLL_SYS_BYPASS_MASK)
		return clk_get_rate(clk->parent);

	div = __raw_readl(PLL1_SYS_BASE_ADDR) & ANADIG_PLL_SYS_DIV_SELECT_MASK;
	val = (clk_get_rate(clk->parent) * div) / 2;
	return val;
}

static int _clk_pll1_main_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg, div;

	if (rate < AUDIO_VIDEO_MIN_CLK_FREQ || rate > AUDIO_VIDEO_MAX_CLK_FREQ)
		return -EINVAL;

	div = (rate * 2) / clk_get_rate(clk->parent);

	/* Update div */
	reg = __raw_readl(PLL1_SYS_BASE_ADDR) & ~ANADIG_PLL_SYS_DIV_SELECT_MASK;
	reg |= div;
	__raw_writel(reg, PLL1_SYS_BASE_ADDR);

	/* Wait for PLL1 to lock */
	if (!WAIT((__raw_readl(PLL1_SYS_BASE_ADDR) & ANADIG_PLL_LOCK),
				SPIN_DELAY))
		panic("pll1 enable failed\n");

	return 0;
}

static void _clk_pll1_disable(struct clk *clk)
{
	void __iomem *pllbase;
	u32 reg;

	pll1_enabled = false;

	/* Set PLL1 in bypass mode only. */
	/* We need to be able to set the ARM-PODF bit
	  * when the system enters WAIT mode. And setting
	  * this bit requires PLL1_main to be enabled.
	  */
	pllbase = _get_pll_base(clk);

	reg = __raw_readl(pllbase);
	reg |= ANADIG_PLL_BYPASS;
	__raw_writel(reg, pllbase);
}

static int _clk_pll1_enable(struct clk *clk)
{
	_clk_pll_enable(clk);
	pll1_enabled = true;
	return 0;
}

static struct clk pll1_sys_main_clk = {
	__INIT_CLK_DEBUG(pll1_sys_main_clk)
	.parent = &osc_clk,
	.get_rate = _clk_pll1_main_get_rate,
	.set_rate = _clk_pll1_main_set_rate,
	.enable = _clk_pll1_enable,
	.disable = _clk_pll1_disable,
};

static int _clk_pll1_sw_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCSR);

	if (parent == &pll1_sys_main_clk) {
		reg &= ~MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CCSR);
		/* Set the step_clk parent to be lp_apm, to save power. */
		reg = __raw_readl(MXC_CCM_CCSR);
		reg = (reg & ~MXC_CCM_CCSR_STEP_SEL);
	} else {
		/* Set STEP_CLK to be the parent*/
		if (parent == &osc_clk) {
			/* Set STEP_CLK to be sourced from LPAPM. */
			reg = __raw_readl(MXC_CCM_CCSR);
			reg = (reg & ~MXC_CCM_CCSR_STEP_SEL);
			__raw_writel(reg, MXC_CCM_CCSR);
		} else {
			/* Set STEP_CLK to be sourced from PLL2-PDF (400MHz). */
			reg = __raw_readl(MXC_CCM_CCSR);
			reg |= MXC_CCM_CCSR_STEP_SEL;
			__raw_writel(reg, MXC_CCM_CCSR);
		}
		reg = __raw_readl(MXC_CCM_CCSR);
		reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
	}
	__raw_writel(reg, MXC_CCM_CCSR);
	return 0;
}

static unsigned long _clk_pll1_sw_get_rate(struct clk *clk)
{
	return clk_get_rate(clk->parent);
}

static struct clk pll1_sw_clk = {
	__INIT_CLK_DEBUG(pll1_sw_clk)
	.parent = &pll1_sys_main_clk,
	.set_parent = _clk_pll1_sw_set_parent,
	.get_rate = _clk_pll1_sw_get_rate,
};

static unsigned long _clk_pll2_main_get_rate(struct clk *clk)
{
	unsigned int div;
	unsigned long val;

	div = __raw_readl(PLL2_528_BASE_ADDR) & ANADIG_PLL_528_DIV_SELECT;

	if (div == 1)
		val = clk_get_rate(clk->parent) * 22;

	else
		val = clk_get_rate(clk->parent) * 20;

	return val;
}

static int _clk_pll2_main_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg,  div;

	if (rate == 528000000)
		div = 1;
	else if (rate == 480000000)
		div = 0;
	else
		return -EINVAL;

	reg = __raw_readl(PLL2_528_BASE_ADDR);
	reg &= ~ANADIG_PLL_528_DIV_SELECT;
	reg |= div;
	__raw_writel(reg, PLL2_528_BASE_ADDR);

	return 0;
}

static struct clk pll2_528_bus_main_clk = {
	__INIT_CLK_DEBUG(pll2_528_bus_main_clk)
	.parent = &osc_clk,
	.get_rate = _clk_pll2_main_get_rate,
	.set_rate = _clk_pll2_main_set_rate,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
};

static void _clk_pll2_pfd_400M_disable(struct clk *clk)
{
	if (!arm_needs_pll2_400)
		_clk_pfd_disable(clk);
}

static struct clk pll2_pfd_400M = {
	__INIT_CLK_DEBUG(pll2_pfd_400M)
	.parent = &pll2_528_bus_main_clk,
	.enable_reg = (void *)PFD_528_BASE_ADDR,
	.enable_shift = ANADIG_PFD2_FRAC_OFFSET,
	.enable = _clk_pfd_enable,
	.disable = _clk_pll2_pfd_400M_disable,
	.get_rate = pfd_get_rate,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static struct clk pll2_pfd_352M = {
	__INIT_CLK_DEBUG(pll2_pfd_352M)
	.parent = &pll2_528_bus_main_clk,
	.enable_reg = (void *)PFD_528_BASE_ADDR,
	.enable_shift = ANADIG_PFD0_FRAC_OFFSET,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static struct clk pll2_pfd_594M = {
	__INIT_CLK_DEBUG(pll2_pfd_594M)
	.parent = &pll2_528_bus_main_clk,
	.enable_reg = (void *)PFD_528_BASE_ADDR,
	.enable_shift = ANADIG_PFD1_FRAC_OFFSET,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static unsigned long _clk_pll2_200M_get_rate(struct clk *clk)
{
	return clk_get_rate(clk->parent) / 2;
}

static struct clk pll2_200M = {
	__INIT_CLK_DEBUG(pll2_200M)
	.parent = &pll2_pfd_400M,
	.get_rate = _clk_pll2_200M_get_rate,
};

static unsigned long _clk_pll3_usb_otg_get_rate(struct clk *clk)
{
	unsigned int div;
	unsigned long val;

	div = __raw_readl(PLL3_480_USB1_BASE_ADDR)
		& ANADIG_PLL_480_DIV_SELECT_MASK;

	if (div == 1)
		val = clk_get_rate(clk->parent) * 22;
	else
		val = clk_get_rate(clk->parent) * 20;
	return val;
}

static int _clk_pll3_usb_otg_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg,  div;

	if (rate == 528000000)
		div = 1;
	else if (rate == 480000000)
		div = 0;
	else
		return -EINVAL;

	reg = __raw_readl(PLL3_480_USB1_BASE_ADDR);
	reg &= ~ANADIG_PLL_480_DIV_SELECT_MASK;
	reg |= div;
	__raw_writel(reg, PLL3_480_USB1_BASE_ADDR);

	return 0;
}


/* same as pll3_main_clk. These two clocks should always be the same */
static struct clk pll3_usb_otg_main_clk = {
	__INIT_CLK_DEBUG(pll3_usb_otg_main_clk)
	.parent = &osc_clk,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_rate = _clk_pll3_usb_otg_set_rate,
	.get_rate = _clk_pll3_usb_otg_get_rate,
};

/* for USB OTG */
static struct clk usb_phy1_clk = {
	__INIT_CLK_DEBUG(usb_phy1_clk)
	.parent = &pll3_usb_otg_main_clk,
	.set_rate = _clk_pll3_usb_otg_set_rate,
	.get_rate = _clk_pll3_usb_otg_get_rate,
};

/* For HSIC port 1 */
static struct clk usb_phy3_clk = {
	__INIT_CLK_DEBUG(usb_phy3_clk)
	.parent = &pll3_usb_otg_main_clk,
	.set_rate = _clk_pll3_usb_otg_set_rate,
	.get_rate = _clk_pll3_usb_otg_get_rate,
};

/* For HSIC port 2 */
static struct clk usb_phy4_clk = {
	__INIT_CLK_DEBUG(usb_phy4_clk)
	.parent = &pll3_usb_otg_main_clk,
	.set_rate = _clk_pll3_usb_otg_set_rate,
	.get_rate = _clk_pll3_usb_otg_get_rate,
};

static struct clk pll3_pfd_508M = {
	__INIT_CLK_DEBUG(pll3_pfd_508M)
	.parent = &pll3_usb_otg_main_clk,
	.enable_reg = (void *)PFD_480_BASE_ADDR,
	.enable_shift = ANADIG_PFD2_FRAC_OFFSET,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static struct clk pll3_pfd_454M = {
	__INIT_CLK_DEBUG(pll3_pfd_454M)
	.parent = &pll3_usb_otg_main_clk,
	.enable_reg = (void *)PFD_480_BASE_ADDR,
	.enable_shift = ANADIG_PFD3_FRAC_OFFSET,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static struct clk pll3_pfd_720M = {
	__INIT_CLK_DEBUG(pll3_pfd_720M)
	.parent = &pll3_usb_otg_main_clk,
	.enable_reg = (void *)PFD_480_BASE_ADDR,
	.enable_shift = ANADIG_PFD0_FRAC_OFFSET,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.set_rate = pfd_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
};

static int pfd_540M_set_rate(struct clk *clk,  unsigned long rate)
{
	if ((clk_get_parent(&ipu1_clk) == clk) ||
		(clk_get_parent(&ipu2_clk) == clk) ||
		(clk_get_parent(&axi_clk) == clk))
		WARN(1, "CHANGING rate of 540M PFD when IPU and \
					AXI is sourced from it \n");

	return pfd_set_rate(clk, rate);
}

static struct clk pll3_pfd_540M = {
	__INIT_CLK_DEBUG(pll3_pfd_540M)
	.parent = &pll3_usb_otg_main_clk,
	.enable_reg = (void *)PFD_480_BASE_ADDR,
	.enable_shift = ANADIG_PFD1_FRAC_OFFSET,
	.enable = _clk_pfd_enable,
	.disable = _clk_pfd_disable,
	.set_rate = pfd_540M_set_rate,
	.get_rate = pfd_get_rate,
	.round_rate = pfd_round_rate,
	.get_rate = pfd_get_rate,
};

static unsigned long _clk_pll3_sw_get_rate(struct clk *clk)
{
	return clk_get_rate(clk->parent);
}

/* same as pll3_main_clk. These two clocks should always be the same */
static struct clk pll3_sw_clk = {
	__INIT_CLK_DEBUG(pll3_sw_clk)
	.parent = &pll3_usb_otg_main_clk,
	.get_rate = _clk_pll3_sw_get_rate,
};

static unsigned long _clk_pll3_120M_get_rate(struct clk *clk)
{
	return clk_get_rate(clk->parent) / 4;
}

static struct clk pll3_120M = {
	__INIT_CLK_DEBUG(pll3_120M)
	.parent = &pll3_sw_clk,
	.get_rate = _clk_pll3_120M_get_rate,
};

static unsigned long _clk_pll3_80M_get_rate(struct clk *clk)
{
	return clk_get_rate(clk->parent) / 6;
}

static struct clk pll3_80M = {
	__INIT_CLK_DEBUG(pll3_80M)
	.parent = &pll3_sw_clk,
	.get_rate = _clk_pll3_80M_get_rate,
};

static unsigned long _clk_pll3_60M_get_rate(struct clk *clk)
{
	return clk_get_rate(clk->parent) / 8;
}

static struct clk pll3_60M = {
	__INIT_CLK_DEBUG(pll3_60M)
	.parent = &pll3_sw_clk,
	.get_rate = _clk_pll3_60M_get_rate,
};

static unsigned long  _clk_audio_video_get_rate(struct clk *clk)
{
	unsigned int div, mfn, mfd;
	unsigned long rate;
	unsigned int parent_rate = clk_get_rate(clk->parent);
	void __iomem *pllbase;
	int rev = mx6q_revision();
	unsigned int test_div_sel, control3, post_div = 1;

	if (clk == &pll4_audio_main_clk)
		pllbase = PLL4_AUDIO_BASE_ADDR;
	else
		pllbase = PLL5_VIDEO_BASE_ADDR;

	if ((rev >= IMX_CHIP_REVISION_1_1) || cpu_is_mx6dl()) {
		test_div_sel = (__raw_readl(pllbase)
			& ANADIG_PLL_AV_TEST_DIV_SEL_MASK)
			>> ANADIG_PLL_AV_TEST_DIV_SEL_OFFSET;
		if (test_div_sel == 0)
			post_div = 4;
		else if (test_div_sel == 1)
			post_div = 2;

		if (clk == &pll5_video_main_clk) {
			control3 = (__raw_readl(ANA_MISC2_BASE_ADDR)
				& ANADIG_ANA_MISC2_CONTROL3_MASK)
				>> ANADIG_ANA_MISC2_CONTROL3_OFFSET;
			if (control3 == 1)
				post_div *= 2;
			else if (control3 == 3)
				post_div *= 4;
		}
	}

	div = __raw_readl(pllbase) & ANADIG_PLL_SYS_DIV_SELECT_MASK;
	mfn = __raw_readl(pllbase + PLL_NUM_DIV_OFFSET);
	mfd = __raw_readl(pllbase + PLL_DENOM_DIV_OFFSET);

	rate = (parent_rate * div) + ((parent_rate / mfd) * mfn);
	rate = rate / post_div;

	return rate;
}

static int _clk_audio_video_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg,  div;
	unsigned int mfn, mfd = 1000000;
	s64 temp64;
	unsigned int parent_rate = clk_get_rate(clk->parent);
	void __iomem *pllbase;
	unsigned long min_clk_rate, pre_div_rate;
	int rev = mx6q_revision();
	u32 test_div_sel = 2;
	u32 control3 = 0;

	if ((rev < IMX_CHIP_REVISION_1_1) && !cpu_is_mx6dl())
		min_clk_rate = AUDIO_VIDEO_MIN_CLK_FREQ;
	else if (clk == &pll4_audio_main_clk)
		min_clk_rate = AUDIO_VIDEO_MIN_CLK_FREQ / 4;
	else
		min_clk_rate = AUDIO_VIDEO_MIN_CLK_FREQ / 16;

	if ((rate < min_clk_rate) || (rate > AUDIO_VIDEO_MAX_CLK_FREQ))
		return -EINVAL;

	if (clk == &pll4_audio_main_clk)
		pllbase = PLL4_AUDIO_BASE_ADDR;
	else
		pllbase = PLL5_VIDEO_BASE_ADDR;

	pre_div_rate = rate;
	if ((rev >= IMX_CHIP_REVISION_1_1) || cpu_is_mx6dl()) {
		while (pre_div_rate < AUDIO_VIDEO_MIN_CLK_FREQ) {
			pre_div_rate *= 2;
			/*
			 * test_div_sel field values:
			 * 2 -> Divide by 1
			 * 1 -> Divide by 2
			 * 0 -> Divide by 4
			 *
			 * control3 field values:
			 * 0 -> Divide by 1
			 * 1 -> Divide by 2
			 * 3 -> Divide by 4
			 */
			if (test_div_sel != 0)
				test_div_sel--;
			else {
				control3++;
				if (control3 == 2)
					control3++;
			}
		}
	}

	div = pre_div_rate / parent_rate;
	temp64 = (u64) (pre_div_rate - (div * parent_rate));
	temp64 *= mfd;
	do_div(temp64, parent_rate);
	mfn = temp64;

	reg = __raw_readl(pllbase)
			& ~ANADIG_PLL_SYS_DIV_SELECT_MASK
			& ~ANADIG_PLL_AV_TEST_DIV_SEL_MASK;
	reg |= div |
		(test_div_sel << ANADIG_PLL_AV_TEST_DIV_SEL_OFFSET);
	__raw_writel(reg, pllbase);
	__raw_writel(mfn, pllbase + PLL_NUM_DIV_OFFSET);
	__raw_writel(mfd, pllbase + PLL_DENOM_DIV_OFFSET);

	if (rev >= IMX_CHIP_REVISION_1_1) {
		reg = __raw_readl(ANA_MISC2_BASE_ADDR)
			& ~ANADIG_ANA_MISC2_CONTROL3_MASK;
		reg |= control3 << ANADIG_ANA_MISC2_CONTROL3_OFFSET;
		__raw_writel(reg, ANA_MISC2_BASE_ADDR);
	}

	return 0;
}

static unsigned long _clk_audio_video_round_rate(struct clk *clk,
						unsigned long rate)
{
	unsigned long min_clk_rate;
	unsigned int div, post_div = 1;
	unsigned int mfn, mfd = 1000000;
	s64 temp64;
	unsigned int parent_rate = clk_get_rate(clk->parent);
	unsigned long pre_div_rate;
	u32 test_div_sel = 2;
	u32 control3 = 0;
	unsigned long final_rate;
	int rev = mx6q_revision();

	if ((rev < IMX_CHIP_REVISION_1_1) && !cpu_is_mx6dl())
		min_clk_rate = AUDIO_VIDEO_MIN_CLK_FREQ;
	else if (clk == &pll4_audio_main_clk)
		min_clk_rate = AUDIO_VIDEO_MIN_CLK_FREQ / 4;
	else
		min_clk_rate = AUDIO_VIDEO_MIN_CLK_FREQ / 16;

	if (rate < min_clk_rate)
		return min_clk_rate;

	if (rate > AUDIO_VIDEO_MAX_CLK_FREQ)
		return AUDIO_VIDEO_MAX_CLK_FREQ;

	pre_div_rate = rate;
	if ((rev >= IMX_CHIP_REVISION_1_1) || cpu_is_mx6dl()) {
		while (pre_div_rate < AUDIO_VIDEO_MIN_CLK_FREQ) {
			pre_div_rate *= 2;
			post_div *= 2;
			if (test_div_sel != 0)
				test_div_sel--;
			else {
				control3++;
				if (control3 == 2)
					control3++;
			}
		}
	}

	div = pre_div_rate / parent_rate;
	temp64 = (u64) (pre_div_rate - (div * parent_rate));
	temp64 *= mfd;
	do_div(temp64, parent_rate);
	mfn = temp64;

	final_rate = (parent_rate * div) + ((parent_rate / mfd) * mfn);
	final_rate = final_rate / post_div;

	return final_rate;
}

static int _clk_audio_video_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;
	void __iomem *pllbase;

	if (clk == &pll4_audio_main_clk)
		pllbase = PLL4_AUDIO_BASE_ADDR;
	else
		pllbase = PLL5_VIDEO_BASE_ADDR;

	reg = __raw_readl(pllbase) & ~ANADIG_PLL_BYPASS_CLK_SRC_MASK;
	mux = _get_mux6(parent, &osc_clk, &anaclk_1, &anaclk_2,
				NULL, NULL, NULL);
	reg |= mux << ANADIG_PLL_BYPASS_CLK_SRC_OFFSET;
	__raw_writel(reg, pllbase);

	/* Set anaclk_x as input */
	if (parent == &anaclk_1) {
		reg = __raw_readl(ANADIG_MISC1_REG);
		reg |= (ANATOP_LVDS_CLK1_IBEN_MASK &
				~ANATOP_LVDS_CLK1_OBEN_MASK);
		__raw_writel(reg, ANADIG_MISC1_REG);
	} else if (parent == &anaclk_2) {
		reg = __raw_readl(ANADIG_MISC1_REG);
		reg |= (ANATOP_LVDS_CLK2_IBEN_MASK &
				~ANATOP_LVDS_CLK2_OBEN_MASK);
		__raw_writel(reg, ANADIG_MISC1_REG);
	}

	return 0;
}

static struct clk pll4_audio_main_clk = {
	__INIT_CLK_DEBUG(pll4_audio_main_clk)
	.parent = &osc_clk,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_rate = _clk_audio_video_set_rate,
	.get_rate = _clk_audio_video_get_rate,
	.round_rate = _clk_audio_video_round_rate,
	.set_parent = _clk_audio_video_set_parent,
};

static struct clk pll5_video_main_clk = {
	__INIT_CLK_DEBUG(pll5_video_main_clk)
	.parent = &osc_clk,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_rate = _clk_audio_video_set_rate,
	.get_rate = _clk_audio_video_get_rate,
	.round_rate = _clk_audio_video_round_rate,
	.set_parent = _clk_audio_video_set_parent,
};

static int _clk_pll_mlb_main_enable(struct clk *clk)
{
	unsigned int reg;
	void __iomem *pllbase;

	pllbase = _get_pll_base(clk);

	reg = __raw_readl(pllbase);
	reg &= ~ANADIG_PLL_BYPASS;

	reg = (0x3 << ANADIG_PLL_MLB_FLT_RES_CFG_OFFSET) |
		(0x3 << ANADIG_PLL_MLB_RX_CLK_DELAY_CFG_OFFSET) |
		(0x2 << ANADIG_PLL_MLB_VDDD_DELAY_CFG_OFFSET) |
		(0x1 << ANADIG_PLL_MLB_VDDA_DELAY_CFG_OFFSET) |
		(ANADIG_PLL_HOLD_RING_OFF);
	__raw_writel(reg, pllbase);

	return 0;
}

static void _clk_pll_mlb_main_disable(struct clk *clk)
{
	unsigned int reg;
	void __iomem *pllbase;

	pllbase = _get_pll_base(clk);

	reg = __raw_readl(pllbase);

	reg |= ANADIG_PLL_BYPASS;

	__raw_writel(reg, pllbase);
}

static struct clk pll6_mlb150_main_clk = {
	__INIT_CLK_DEBUG(pll6_mlb150_main_clk)
	.parent = &osc_clk,
	.enable = _clk_pll_mlb_main_enable,
	.disable = _clk_pll_mlb_main_disable,
};

static unsigned long _clk_pll7_usb_otg_get_rate(struct clk *clk)
{
	unsigned int div;
	unsigned long val;

	div = __raw_readl(PLL7_480_USB2_BASE_ADDR)
		& ANADIG_PLL_480_DIV_SELECT_MASK;

	if (div == 1)
		val = clk_get_rate(clk->parent) * 22;
	else
		val = clk_get_rate(clk->parent) * 20;
	return val;
}

static int _clk_pll7_usb_otg_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg,  div;

	if (rate == 528000000)
		div = 1;
	else if (rate == 480000000)
		div = 0;
	else
		return -EINVAL;

	reg = __raw_readl(PLL7_480_USB2_BASE_ADDR);
	reg &= ~ANADIG_PLL_480_DIV_SELECT_MASK;
	reg |= div;
	__raw_writel(reg, PLL7_480_USB2_BASE_ADDR);

	return 0;
}

static struct clk pll7_usb_host_main_clk = {
	__INIT_CLK_DEBUG(pll7_usb_host_main_clk)
	.parent = &osc_clk,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_rate = _clk_pll7_usb_otg_set_rate,
	.get_rate = _clk_pll7_usb_otg_get_rate,

};

static struct clk pll8_enet_main_clk = {
	__INIT_CLK_DEBUG(pll8_enet_main_clk)
	.parent = &osc_clk,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
};

static unsigned long _clk_arm_get_rate(struct clk *clk)
{
	u32 cacrr, div;

	cacrr = __raw_readl(MXC_CCM_CACRR);
	div = (cacrr & MXC_CCM_CACRR_ARM_PODF_MASK) + 1;
	return clk_get_rate(clk->parent) / div;
}

static int _clk_arm_set_rate(struct clk *clk, unsigned long rate)
{
	int i;
	u32 div;
	unsigned long parent_rate;
	unsigned long flags;
	unsigned long ipg_clk_rate, max_arm_wait_clk;

	for (i = 0; i < cpu_op_nr; i++) {
		if (rate == cpu_op_tbl[i].cpu_rate)
			break;
	}
	if (i >= cpu_op_nr)
		return -EINVAL;

	spin_lock_irqsave(&clk_lock, flags);

	if (rate <= clk_get_rate(&pll2_pfd_400M)) {
		/* Source pll1_sw_clk from step_clk which is sourced from
		  * PLL2_PFD_400M.
		  */
		if (pll1_sw_clk.parent != &pll2_pfd_400M) {
			if (pll2_pfd_400M.usecount == 0) {
				/* Check if PLL2 needs to be enabled also. */
				if (pll2_528_bus_main_clk.usecount == 0)
					pll2_528_bus_main_clk.enable(&pll2_528_bus_main_clk);
				/* Ensure parent usecount is
				  * also incremented.
				  */
				pll2_528_bus_main_clk.usecount++;
				pll2_pfd_400M.enable(&pll2_pfd_400M);
			}
			pll2_pfd_400M.usecount++;
			arm_needs_pll2_400 = true;
			pll1_sw_clk.set_parent(&pll1_sw_clk, &pll2_pfd_400M);
			pll1_sw_clk.parent = &pll2_pfd_400M;
		}
	} else {
		/* Make sure PLL1 is enabled */
		if (!pll1_enabled) {
			pll1_sys_main_clk.enable(&pll1_sys_main_clk);
			pll1_sys_main_clk.usecount = 1;
		}
		/* Make sure PLL1 rate is what we want */
		if (cpu_op_tbl[i].pll_rate != clk_get_rate(&pll1_sys_main_clk)) {
			/* If pll1_sw_clk is from pll1_sys_main_clk, switch it */
			if (pll1_sw_clk.parent == &pll1_sys_main_clk) {
				/* Change the PLL1 rate. */
				if (pll2_pfd_400M.usecount != 0)
					pll1_sw_clk.set_parent(&pll1_sw_clk, &pll2_pfd_400M);
				else
					pll1_sw_clk.set_parent(&pll1_sw_clk, &osc_clk);
			}
			pll1_sys_main_clk.set_rate(&pll1_sys_main_clk, cpu_op_tbl[i].pll_rate);
		}
		/* Make sure pll1_sw_clk is from pll1_sys_main_clk */
		pll1_sw_clk.set_parent(&pll1_sw_clk, &pll1_sys_main_clk);
		pll1_sw_clk.parent = &pll1_sys_main_clk;
		if (arm_needs_pll2_400) {
			pll2_pfd_400M.usecount--;
			if (pll2_pfd_400M.usecount == 0) {
				pll2_pfd_400M.disable(&pll2_pfd_400M);
				/* Ensure parent usecount is
				  * also decremented.
				  */
				pll2_528_bus_main_clk.usecount--;
				if (pll2_528_bus_main_clk.usecount == 0)
					pll2_528_bus_main_clk.disable(&pll2_528_bus_main_clk);
			}
		}
		arm_needs_pll2_400 = false;
	}
	parent_rate = clk_get_rate(clk->parent);
	div = parent_rate / rate;
	/* Calculate the ARM_PODF to be applied when the system
	  * enters WAIT state. The max ARM clk is decided by the
	  * ipg_clk and has to follow the ratio of ARM_CLK:IPG_CLK of 12:5.
	  * For ex, when IPG is at 66MHz, ARM_CLK cannot be greater
	  * than 158MHz.
	  * Pre-calculate the optimal divider now.
	  */
	ipg_clk_rate = clk_get_rate(&ipg_clk);
	max_arm_wait_clk = (12 * ipg_clk_rate) / 5;
	wait_mode_arm_podf = parent_rate / max_arm_wait_clk;

	if (div == 0)
		div = 1;

	if ((parent_rate / div) > rate)
		div++;

	if (div > 8) {
		spin_unlock_irqrestore(&clk_lock, flags);
		return -1;
	}

	cur_arm_podf = div;

	__raw_writel(div - 1, MXC_CCM_CACRR);

	while (__raw_readl(MXC_CCM_CDHIPR))
		;

	if (pll1_sys_main_clk.usecount == 1 && arm_needs_pll2_400) {
		pll1_sys_main_clk.disable(&pll1_sys_main_clk);
		pll1_sys_main_clk.usecount = 0;
	}

	spin_unlock_irqrestore(&clk_lock, flags);

	return 0;
}

static struct clk cpu_clk = {
	__INIT_CLK_DEBUG(cpu_clk)
	.parent = &pll1_sw_clk,
	.set_rate = _clk_arm_set_rate,
	.get_rate = _clk_arm_get_rate,
};

static unsigned long _clk_twd_get_rate(struct clk *clk)
{
	return clk_get_rate(clk->parent) / 2;
}

static struct clk twd_clk = {
	__INIT_CLK_DEBUG(twd_clk)
	.parent = &cpu_clk,
	.get_rate = _clk_twd_get_rate,
};

static int _clk_periph_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux6(parent, &pll2_528_bus_main_clk, &pll2_pfd_400M,
		&pll2_pfd_352M, &pll2_200M, &pll3_sw_clk, &osc_clk);

	if (mux <= 3) {
		/* Set the pre_periph_clk multiplexer */
		reg = __raw_readl(MXC_CCM_CBCMR);
		reg &= ~MXC_CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK;
		reg |= mux << MXC_CCM_CBCMR_PRE_PERIPH_CLK_SEL_OFFSET;
		__raw_writel(reg, MXC_CCM_CBCMR);

		/* Set the periph_clk_sel multiplexer. */
		reg = __raw_readl(MXC_CCM_CBCDR);
		reg &= ~MXC_CCM_CBCDR_PERIPH_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CBCDR);
	} else {
		reg = __raw_readl(MXC_CCM_CBCDR);
		/* Set the periph_clk2_podf divider to divide by 1. */
		reg &= ~MXC_CCM_CBCDR_PERIPH_CLK2_PODF_MASK;
		__raw_writel(reg, MXC_CCM_CBCDR);

		/* Set the periph_clk2_sel mux. */
		reg = __raw_readl(MXC_CCM_CBCMR);
		reg &= ~MXC_CCM_CBCMR_PERIPH_CLK2_SEL_MASK;
		reg |= ((mux - 4) << MXC_CCM_CBCMR_PERIPH_CLK2_SEL_OFFSET);
		__raw_writel(reg, MXC_CCM_CBCMR);

		while (__raw_readl(MXC_CCM_CDHIPR))
			;

		reg = __raw_readl(MXC_CCM_CBCDR);
		/* Set periph_clk_sel to select periph_clk2. */
		reg |= MXC_CCM_CBCDR_PERIPH_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CBCDR);
	}

	if (!WAIT(!(__raw_readl(MXC_CCM_CDHIPR)
	     & MXC_CCM_CDHIPR_PERIPH_CLK_SEL_BUSY), SPIN_DELAY))
		panic("_clk_periph_set_parent failed\n");

	return 0;
}

static unsigned long _clk_periph_get_rate(struct clk *clk)
{
	u32 div = 1;
	u32 reg;
	unsigned long val;

	if ((clk->parent == &pll3_sw_clk) || (clk->parent == &osc_clk)) {
		reg = __raw_readl(MXC_CCM_CBCDR)
			& MXC_CCM_CBCDR_PERIPH_CLK2_PODF_MASK;
		div = (reg >> MXC_CCM_CBCDR_PERIPH_CLK2_PODF_OFFSET) + 1;
	}
	val = clk_get_rate(clk->parent) / div;
	return val;
}

static struct clk periph_clk = {
	__INIT_CLK_DEBUG(periph_clk)
	.parent = &pll2_528_bus_main_clk,
	.set_parent = _clk_periph_set_parent,
	.get_rate = _clk_periph_get_rate,
};

static unsigned long _clk_axi_get_rate(struct clk *clk)
{
	u32 div, reg;
	unsigned long val;

	reg = __raw_readl(MXC_CCM_CBCDR) & MXC_CCM_CBCDR_AXI_PODF_MASK;
	div = (reg >> MXC_CCM_CBCDR_AXI_PODF_OFFSET);

	val = clk_get_rate(clk->parent) / (div + 1);
	return val;
}

static int _clk_axi_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AXI_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AXI_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	if (!WAIT(!(__raw_readl(MXC_CCM_CDHIPR)
	     & MXC_CCM_CDHIPR_AXI_PODF_BUSY), SPIN_DELAY))
		panic("pll _clk_axi_a_set_rate failed\n");

	return 0;
}

static unsigned long _clk_axi_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum
	 * value for the clock.
	 * Also prevent a div of 0.
	 */

	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;

	return parent_rate / div;
}

static int _clk_axi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	int mux;

	mux = _get_mux6(parent, &periph_clk, &pll2_pfd_400M,
				&pll3_pfd_540M, NULL, NULL, NULL);

	if (mux == 0) {
		/* Set the AXI_SEL mux */
		reg = __raw_readl(MXC_CCM_CBCDR) & ~MXC_CCM_CBCDR_AXI_SEL;
		__raw_writel(reg, MXC_CCM_CBCDR);
	} else {
		/* Set the AXI_ALT_SEL mux. */
		reg = __raw_readl(MXC_CCM_CBCDR)
			& ~MXC_CCM_CBCDR_AXI_ALT_SEL_MASK;
		reg |= ((mux - 1) << MXC_CCM_CBCDR_AXI_ALT_SEL_OFFSET);
		__raw_writel(reg, MXC_CCM_CBCDR);

		/* Set the AXI_SEL mux */
		reg = __raw_readl(MXC_CCM_CBCDR) & ~MXC_CCM_CBCDR_AXI_SEL;
		reg |= MXC_CCM_CBCDR_AXI_SEL;
		__raw_writel(reg, MXC_CCM_CBCDR);
	}
	return 0;
}

static struct clk axi_clk = {
	__INIT_CLK_DEBUG(axi_clk)
	.parent = &periph_clk,
	.set_parent = _clk_axi_set_parent,
	.set_rate = _clk_axi_set_rate,
	.get_rate = _clk_axi_get_rate,
	.round_rate = _clk_axi_round_rate,
};

static unsigned long _clk_ahb_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AHB_PODF_MASK) >>
	       MXC_CCM_CBCDR_AHB_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_ahb_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AHB_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AHB_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	if (!WAIT(!(__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_AHB_PODF_BUSY),
				SPIN_DELAY))
			panic("_clk_ahb_set_rate failed\n");

	return 0;
}

static unsigned long _clk_ahb_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static struct clk ahb_clk = {
	__INIT_CLK_DEBUG(ahb_clk)
	.parent = &periph_clk,
	.get_rate = _clk_ahb_get_rate,
	.set_rate = _clk_ahb_set_rate,
	.round_rate = _clk_ahb_round_rate,
};

static unsigned long _clk_ipg_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_IPG_PODF_MASK) >>
		MXC_CCM_CBCDR_IPG_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}


static struct clk ipg_clk = {
	__INIT_CLK_DEBUG(ipg_clk)
	.parent = &ahb_clk,
	.get_rate = _clk_ipg_get_rate,
};

static struct clk tzasc1_clk = {
	__INIT_CLK_DEBUG(tzasc1_clk)
	.id = 0,
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk tzasc2_clk = {
	__INIT_CLK_DEBUG(tzasc2_clk)
	.id = 0,
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk mx6fast1_clk = {
	__INIT_CLK_DEBUG(mx6fast1_clk)
	.id = 0,
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk mx6per1_clk = {
	__INIT_CLK_DEBUG(mx6per1_clk)
	.id = 0,
	.parent = &ahb_clk,
	.secondary = &mx6fast1_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk mx6per2_clk = {
	__INIT_CLK_DEBUG(mx6per2_clk)
	.id = 0,
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static unsigned long _clk_mmdc_ch0_axi_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_MMDC_CH0_PODF_MASK) >>
			MXC_CCM_CBCDR_MMDC_CH0_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_mmdc_ch0_axi_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_MMDC_CH0_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_MMDC_CH0_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	if (!WAIT(!(__raw_readl(MXC_CCM_CDHIPR)
		& MXC_CCM_CDHIPR_MMDC_CH0_PODF_BUSY),
				SPIN_DELAY))
			panic("_clk_mmdc_ch0_axi_set_rate failed\n");

	return 0;
}

static unsigned long _clk_mmdc_ch0_axi_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static struct clk mmdc_ch0_axi_clk[] = {
	{
	__INIT_CLK_DEBUG(mmdc_ch0_axi_clk)
	.id = 0,
	.parent = &periph_clk,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.secondary = &mmdc_ch0_axi_clk[1],
	.get_rate = _clk_mmdc_ch0_axi_get_rate,
	.set_rate = _clk_mmdc_ch0_axi_set_rate,
	.round_rate = _clk_mmdc_ch0_axi_round_rate,
	},
	{
	__INIT_CLK_DEBUG(mmdc_ch0_ipg_clk)
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.secondary = &tzasc1_clk,
	},
};

static unsigned long _clk_mmdc_ch1_axi_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_MMDC_CH1_PODF_MASK) >>
			MXC_CCM_CBCDR_MMDC_CH1_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_mmdc_ch1_axi_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_MMDC_CH1_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_MMDC_CH1_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	if (!WAIT(!(__raw_readl(MXC_CCM_CDHIPR)
		& MXC_CCM_CDHIPR_MMDC_CH1_PODF_BUSY), SPIN_DELAY))
			panic("_clk_mmdc_ch1_axi_set_rate failed\n");

	return 0;
}

static unsigned long _clk_mmdc_ch1_axi_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static struct clk mmdc_ch1_axi_clk[] = {
	{
	__INIT_CLK_DEBUG(mmdc_ch1_axi_clk)
	.id = 0,
	.parent = &pll2_pfd_400M,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.secondary = &mmdc_ch1_axi_clk[1],
	.get_rate = _clk_mmdc_ch1_axi_get_rate,
	.set_rate = _clk_mmdc_ch1_axi_set_rate,
	.round_rate = _clk_mmdc_ch1_axi_round_rate,
	},
	{
	.id = 1,
	__INIT_CLK_DEBUG(mmdc_ch1_ipg_clk)
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.secondary = &tzasc2_clk,
	},
};

static struct clk ocram_clk = {
	__INIT_CLK_DEBUG(ocram_clk)
	.id = 0,
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static unsigned long _clk_ipg_perclk_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	div = ((reg & MXC_CCM_CSCMR1_PERCLK_PODF_MASK) >>
			MXC_CCM_CSCMR1_PERCLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_ipg_perclk_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 64))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	reg &= ~MXC_CCM_CSCMR1_PERCLK_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCMR1_PERCLK_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}


static unsigned long _clk_ipg_perclk_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 64)
		div = 64;

	return parent_rate / div;
}

static struct clk ipg_perclk = {
	__INIT_CLK_DEBUG(ipg_perclk)
	.parent = &ipg_clk,
	.get_rate = _clk_ipg_perclk_get_rate,
	.set_rate = _clk_ipg_perclk_set_rate,
	.round_rate = _clk_ipg_perclk_round_rate,
};

static struct clk spba_clk = {
	__INIT_CLK_DEBUG(spba_clk)
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk sdma_clk[] = {
	{
	 __INIT_CLK_DEBUG(sdma_clk)
	 .parent = &ahb_clk,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &sdma_clk[1],
	},
	{
	 .parent = &mx6per1_clk,
#ifdef CONFIG_SDMA_IRAM
	 .secondary = &ocram_clk,
#else
	.secondary = &mmdc_ch0_axi_clk[0],
#endif
	},
};

static int _clk_gpu2d_axi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg = __raw_readl(MXC_CCM_CBCMR) & ~MXC_CCM_CBCMR_GPU2D_AXI_CLK_SEL;

	if (parent == &ahb_clk)
		reg |= MXC_CCM_CBCMR_GPU2D_AXI_CLK_SEL;

	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk gpu2d_axi_clk = {
	__INIT_CLK_DEBUG(gpu2d_axi_clk)
	.parent = &axi_clk,
	.set_parent = _clk_gpu2d_axi_set_parent,
};

static int _clk_gpu3d_axi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg = __raw_readl(MXC_CCM_CBCMR) & ~MXC_CCM_CBCMR_GPU3D_AXI_CLK_SEL;

	if (parent == &ahb_clk)
		reg |= MXC_CCM_CBCMR_GPU3D_AXI_CLK_SEL;

	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk gpu3d_axi_clk = {
	__INIT_CLK_DEBUG(gpu3d_axi_clk)
	.parent = &axi_clk,
	.secondary = &mmdc_ch0_axi_clk[0],
	.set_parent = _clk_gpu3d_axi_set_parent,
};

static int _clk_pcie_axi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg = __raw_readl(MXC_CCM_CBCMR) & ~MXC_CCM_CBCMR_PCIE_AXI_CLK_SEL;

	if (parent == &ahb_clk)
		reg |= MXC_CCM_CBCMR_PCIE_AXI_CLK_SEL;

	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk pcie_axi_clk = {
	__INIT_CLK_DEBUG(pcie_axi_clk)
	.parent = &axi_clk,
	.set_parent = _clk_pcie_axi_set_parent,
};

static int _clk_vdo_axi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg = __raw_readl(MXC_CCM_CBCMR) & ~MXC_CCM_CBCMR_VDOAXI_CLK_SEL;

	if (parent == &ahb_clk)
		reg |= MXC_CCM_CBCMR_VDOAXI_CLK_SEL;

	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk vdo_axi_clk = {
	__INIT_CLK_DEBUG(vdo_axi_clk)
	.parent = &axi_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_vdo_axi_set_parent,
};

static struct clk vdoa_clk[] = {
	{
	__INIT_CLK_DEBUG(vdoa_clk)
	.id = 0,
	.parent = &vdo_axi_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &vdoa_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	.parent =  &mmdc_ch0_axi_clk[0],
	.secondary = &vdoa_clk[2],
	},
	{
	.parent =  &mx6fast1_clk,
	.secondary = &ocram_clk,
	},
};

static unsigned long mx6_timer_rate(void)
{
	u32 parent_rate = clk_get_rate(&osc_clk);

	u32 reg = __raw_readl(timer_base + MXC_TCTL);
	u32 div;

	if ((reg & V2_TCTL_CLK_OSC_DIV8) == V2_TCTL_CLK_OSC_DIV8) {
		if (cpu_is_mx6q())
			/* For MX6Q, only options are 24MHz or 24MHz/8*/
			return parent_rate / 8;
		else {
			/* For MX6DLS and MX6Solo, the rate is based on the
			  * divider value set in prescalar register. */
			div = __raw_readl(timer_base + MXC_TPRER);
			div = (div >> V2_TPRER_PRE24M_OFFSET) &
					V2_TPRER_PRE24M_MASK;
			return parent_rate / (div + 1);
		}
	}
	return 0;
}

static unsigned long _clk_gpt_get_rate(struct clk *clk)
{
	unsigned long rate;

	if (mx6q_revision() == IMX_CHIP_REVISION_1_0)
		return clk_get_rate(clk->parent);

	rate = mx6_timer_rate();
	if (!rate)
		return clk_get_rate(clk->parent);

	return rate;
}

static struct clk gpt_clk[] = {
	{
	__INIT_CLK_DEBUG(gpt_clk)
	 .parent = &osc_clk,
	 .id = 0,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .get_rate = _clk_gpt_get_rate,
	 .secondary = &gpt_clk[1],
	 },
	{
	__INIT_CLK_DEBUG(gpt_serial_clk)
	 .id = 0,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static unsigned long _clk_iim_get_rate(struct clk *clk)
{
	return clk_get_rate(clk->parent);
}

static struct clk iim_clk = {
	__INIT_CLK_DEBUG(iim_clk)
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.disable = _clk_disable,
	.get_rate = _clk_iim_get_rate,
};

static struct clk i2c_clk[] = {
	{
	__INIT_CLK_DEBUG(i2c_clk_0)
	 .id = 0,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	__INIT_CLK_DEBUG(i2c_clk_1)
	 .id = 1,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	__INIT_CLK_DEBUG(i2c_clk_2)
	 .id = 2,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static int _clk_vpu_axi_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CBCMR)
		& ~MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_MASK;

	mux = _get_mux6(parent, &axi_clk, &pll2_pfd_400M,
		&pll2_pfd_352M, NULL, NULL, NULL);

	reg |= (mux << MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_OFFSET);

	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static unsigned long _clk_vpu_axi_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	div = ((reg & MXC_CCM_CSCDR1_VPU_AXI_PODF_MASK) >>
			MXC_CCM_CSCDR1_VPU_AXI_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_vpu_axi_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_VPU_AXI_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCDR1_VPU_AXI_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}

static unsigned long _clk_vpu_axi_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static struct clk vpu_clk[] = {
	{
	__INIT_CLK_DEBUG(vpu_clk)
	.parent = &axi_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_vpu_axi_set_parent,
	.round_rate = _clk_vpu_axi_round_rate,
	.set_rate = _clk_vpu_axi_set_rate,
	.get_rate = _clk_vpu_axi_get_rate,
	.secondary = &vpu_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	.parent =  &mmdc_ch0_axi_clk[0],
	.secondary = &vpu_clk[2],
	},
	{
	.parent =  &mx6fast1_clk,
	.secondary = &ocram_clk,
	},
};

static int _clk_ipu1_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CSCDR3)
		& ~MXC_CCM_CSCDR3_IPU1_HSP_CLK_SEL_MASK;

	mux = _get_mux6(parent, &mmdc_ch0_axi_clk[0],
		&pll2_pfd_400M, &pll3_120M, &pll3_pfd_540M, NULL, NULL);

	reg |= (mux << MXC_CCM_CSCDR3_IPU1_HSP_CLK_SEL_OFFSET);

	__raw_writel(reg, MXC_CCM_CSCDR3);

	return 0;
}

static unsigned long _clk_ipu1_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR3);
	div = ((reg & MXC_CCM_CSCDR3_IPU1_HSP_PODF_MASK) >>
			MXC_CCM_CSCDR3_IPU1_HSP_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_ipu1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR3);
	reg &= ~MXC_CCM_CSCDR3_IPU1_HSP_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCDR3_IPU1_HSP_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR3);

	return 0;
}

static unsigned long _clk_ipu_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static struct clk ipu1_clk = {
	__INIT_CLK_DEBUG(ipu1_clk)
	.parent = &mmdc_ch0_axi_clk[0],
	.secondary = &mmdc_ch0_axi_clk[0],
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ipu1_set_parent,
	.round_rate = _clk_ipu_round_rate,
	.set_rate = _clk_ipu1_set_rate,
	.get_rate = _clk_ipu1_get_rate,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static int _clk_ipu2_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CSCDR3)
		& ~MXC_CCM_CSCDR3_IPU2_HSP_CLK_SEL_MASK;

	mux = _get_mux6(parent, &mmdc_ch0_axi_clk[0],
		&pll2_pfd_400M, &pll3_120M, &pll3_pfd_540M, NULL, NULL);

	reg |= (mux << MXC_CCM_CSCDR3_IPU2_HSP_CLK_SEL_OFFSET);

	__raw_writel(reg, MXC_CCM_CSCDR3);

	return 0;
}

static unsigned long _clk_ipu2_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR3);
	div = ((reg & MXC_CCM_CSCDR3_IPU2_HSP_PODF_MASK) >>
			MXC_CCM_CSCDR3_IPU2_HSP_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_ipu2_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR3);
	reg &= ~MXC_CCM_CSCDR3_IPU2_HSP_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCDR3_IPU2_HSP_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR3);

	return 0;
}

static struct clk ipu2_clk = {
	__INIT_CLK_DEBUG(ipu2_clk)
	.parent = &mmdc_ch0_axi_clk[0],
	.secondary = &mmdc_ch0_axi_clk[0],
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ipu2_set_parent,
	.round_rate = _clk_ipu_round_rate,
	.set_rate = _clk_ipu2_set_rate,
	.get_rate = _clk_ipu2_get_rate,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk usdhc_dep_clk = {
	.parent = &mmdc_ch0_axi_clk[0],
	.secondary = &mx6per1_clk,
	};

static unsigned long _clk_usdhc_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static int _clk_usdhc1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_USDHC1_CLK_SEL;

	if (parent == &pll2_pfd_352M)
		reg |= (MXC_CCM_CSCMR1_USDHC1_CLK_SEL);

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static unsigned long _clk_usdhc1_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	div = ((reg & MXC_CCM_CSCDR1_USDHC1_PODF_MASK) >>
			MXC_CCM_CSCDR1_USDHC1_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_usdhc1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_USDHC1_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCDR1_USDHC1_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}

static struct clk usdhc1_clk = {
	__INIT_CLK_DEBUG(usdhc1_clk)
	.id = 0,
	.parent = &pll2_pfd_400M,
	.secondary = &usdhc_dep_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_usdhc1_set_parent,
	.round_rate = _clk_usdhc_round_rate,
	.set_rate = _clk_usdhc1_set_rate,
	.get_rate = _clk_usdhc1_get_rate,
};

static int _clk_usdhc2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_USDHC2_CLK_SEL;

	if (parent == &pll2_pfd_352M)
		reg |= (MXC_CCM_CSCMR1_USDHC2_CLK_SEL);

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static unsigned long _clk_usdhc2_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	div = ((reg & MXC_CCM_CSCDR1_USDHC2_PODF_MASK) >>
			MXC_CCM_CSCDR1_USDHC2_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_usdhc2_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_USDHC2_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCDR1_USDHC2_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}

static struct clk usdhc2_clk = {
	__INIT_CLK_DEBUG(usdhc2_clk)
	.id = 1,
	.parent = &pll2_pfd_400M,
	.secondary = &usdhc_dep_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_usdhc2_set_parent,
	.round_rate = _clk_usdhc_round_rate,
	.set_rate = _clk_usdhc2_set_rate,
	.get_rate = _clk_usdhc2_get_rate,
};

static int _clk_usdhc3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_USDHC3_CLK_SEL;

	if (parent == &pll2_pfd_352M)
		reg |= (MXC_CCM_CSCMR1_USDHC3_CLK_SEL);

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static unsigned long _clk_usdhc3_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	div = ((reg & MXC_CCM_CSCDR1_USDHC3_PODF_MASK) >>
			MXC_CCM_CSCDR1_USDHC3_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_usdhc3_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_USDHC3_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCDR1_USDHC3_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}


static struct clk usdhc3_clk = {
	__INIT_CLK_DEBUG(usdhc3_clk)
	.id = 2,
	.parent = &pll2_pfd_400M,
	.secondary = &usdhc_dep_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_usdhc3_set_parent,
	.round_rate = _clk_usdhc_round_rate,
	.set_rate = _clk_usdhc3_set_rate,
	.get_rate = _clk_usdhc3_get_rate,
};

static int _clk_usdhc4_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_USDHC4_CLK_SEL;

	if (parent == &pll2_pfd_352M)
		reg |= (MXC_CCM_CSCMR1_USDHC4_CLK_SEL);

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static unsigned long _clk_usdhc4_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	div = ((reg & MXC_CCM_CSCDR1_USDHC4_PODF_MASK) >>
			MXC_CCM_CSCDR1_USDHC4_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_usdhc4_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_USDHC4_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCDR1_USDHC4_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}


static struct clk usdhc4_clk = {
	__INIT_CLK_DEBUG(usdhc4_clk)
	.id = 3,
	.parent = &pll2_pfd_400M,
	.secondary = &usdhc_dep_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_usdhc4_set_parent,
	.round_rate = _clk_usdhc_round_rate,
	.set_rate = _clk_usdhc4_set_rate,
	.get_rate = _clk_usdhc4_get_rate,
};

static unsigned long _clk_ssi_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (parent_rate % rate)
		div++;

	__calc_pre_post_dividers(1 << 6, div, &pre, &post);

	return parent_rate / (pre * post);
}

static unsigned long _clk_ssi1_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CS1CDR);

	prediv = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK)
		>> MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK)
		>> MXC_CCM_CS1CDR_SSI1_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
}

static int _clk_ssi1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || div > 512)
		return -EINVAL;

	__calc_pre_post_dividers(1 << 6, div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg &= ~(MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK |
		 MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CS1CDR_SSI1_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET;

	__raw_writel(reg, MXC_CCM_CS1CDR);

	return 0;
}


static int _clk_ssi1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR1)
		& ~MXC_CCM_CSCMR1_SSI1_CLK_SEL_MASK;

	mux = _get_mux6(parent, &pll3_pfd_508M, &pll3_pfd_454M,
			&pll4_audio_main_clk, NULL, NULL, NULL);
	reg |= (mux << MXC_CCM_CSCMR1_SSI1_CLK_SEL_OFFSET);

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi1_clk = {
	__INIT_CLK_DEBUG(ssi1_clk)
	.parent = &pll3_pfd_508M,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ssi1_set_parent,
	.set_rate = _clk_ssi1_set_rate,
	.round_rate = _clk_ssi_round_rate,
	.get_rate = _clk_ssi1_get_rate,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	 .secondary = &ocram_clk,
#else
	 .secondary = &mmdc_ch0_axi_clk[0],
#endif
	.flags  = AHB_AUDIO_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static unsigned long _clk_ssi2_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CS2CDR);

	prediv = ((reg & MXC_CCM_CS2CDR_SSI2_CLK_PRED_MASK)
		>> MXC_CCM_CS2CDR_SSI2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CS2CDR_SSI2_CLK_PODF_MASK)
		>> MXC_CCM_CS2CDR_SSI2_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
}

static int _clk_ssi2_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || div > 512)
		return -EINVAL;

	__calc_pre_post_dividers(1 << 6, div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CS2CDR);
	reg &= ~(MXC_CCM_CS2CDR_SSI2_CLK_PRED_MASK |
		 MXC_CCM_CS2CDR_SSI2_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CS2CDR_SSI2_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CS2CDR_SSI2_CLK_PRED_OFFSET;

	__raw_writel(reg, MXC_CCM_CS2CDR);

	return 0;
}


static int _clk_ssi2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR1)
		& ~MXC_CCM_CSCMR1_SSI2_CLK_SEL_MASK;

	mux = _get_mux6(parent, &pll3_pfd_508M, &pll3_pfd_454M,
			&pll4_audio_main_clk, NULL, NULL, NULL);
	reg |= (mux << MXC_CCM_CSCMR1_SSI2_CLK_SEL_OFFSET);

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi2_clk = {
	__INIT_CLK_DEBUG(ssi2_clk)
	.parent = &pll3_pfd_508M,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ssi2_set_parent,
	.set_rate = _clk_ssi2_set_rate,
	.round_rate = _clk_ssi_round_rate,
	.get_rate = _clk_ssi2_get_rate,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	 .secondary = &ocram_clk,
#else
	 .secondary = &mmdc_ch0_axi_clk[0],
#endif
	.flags  = AHB_AUDIO_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static unsigned long _clk_ssi3_get_rate(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CS1CDR);

	prediv = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK)
		>> MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK)
		>> MXC_CCM_CS1CDR_SSI1_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (prediv * podf);
}

static int _clk_ssi3_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || div > 512)
		return -EINVAL;

	__calc_pre_post_dividers(1 << 6, div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg &= ~(MXC_CCM_CS1CDR_SSI3_CLK_PODF_MASK|
		 MXC_CCM_CS1CDR_SSI3_CLK_PRED_MASK);
	reg |= (post - 1) << MXC_CCM_CS1CDR_SSI3_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CS1CDR_SSI3_CLK_PRED_OFFSET;

	__raw_writel(reg, MXC_CCM_CS1CDR);

	return 0;
}


static int _clk_ssi3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_SSI3_CLK_SEL_MASK;

	mux = _get_mux6(parent, &pll3_pfd_508M, &pll3_pfd_454M,
				&pll4_audio_main_clk, NULL, NULL, NULL);
	reg |= (mux << MXC_CCM_CSCMR1_SSI3_CLK_SEL_OFFSET);

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi3_clk = {
	__INIT_CLK_DEBUG(ssi3_clk)
	.parent = &pll3_pfd_508M,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ssi3_set_parent,
	.set_rate = _clk_ssi3_set_rate,
	.round_rate = _clk_ssi_round_rate,
	.get_rate = _clk_ssi3_get_rate,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	 .secondary = &ocram_clk,
#else
	 .secondary = &mmdc_ch0_axi_clk[0],
#endif
	.flags  = AHB_AUDIO_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static unsigned long _clk_ldb_di_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 parent_rate = clk_get_rate(clk->parent);

	if (rate * 7 <= parent_rate + parent_rate/20)
		return parent_rate / 7;
	else
		return 2 * parent_rate / 7;
}

static unsigned long _clk_ldb_di0_get_rate(struct clk *clk)
{
	u32 div;

	div = __raw_readl(MXC_CCM_CSCMR2) &
		MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;

	if (div)
		return clk_get_rate(clk->parent) / 7;

	return (2 * clk_get_rate(clk->parent)) / 7;
}

static int _clk_ldb_di0_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div = 0;
	u32 parent_rate = clk_get_rate(clk->parent);

	if (rate * 7 <= parent_rate + parent_rate/20) {
		div = 7;
		rate = parent_rate / 7;
	} else
		rate = 2 * parent_rate / 7;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	if (div == 7)
		reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	else
		reg &= ~MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;

	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static int _clk_ldb_di0_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	int rev = mx6q_revision();

	reg = __raw_readl(MXC_CCM_CS2CDR)
		& ~MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK;

	mux = _get_mux6(parent, &pll5_video_main_clk,
		&pll2_pfd_352M, &pll2_pfd_400M,
		(rev == IMX_CHIP_REVISION_1_0) ?
		 &pll3_pfd_540M :	/* MX6Q TO1.0 */
		 &mmdc_ch1_axi_clk[0],	/* MX6Q TO1.1 and MX6DL */
		&pll3_usb_otg_main_clk, NULL);
	reg |= (mux << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET);

	__raw_writel(reg, MXC_CCM_CS2CDR);

	return 0;
}

static struct clk ldb_di0_clk = {
	 __INIT_CLK_DEBUG(ldb_di0_clk)
	.id = 0,
	.parent = &pll2_pfd_352M,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ldb_di0_set_parent,
	.set_rate = _clk_ldb_di0_set_rate,
	.round_rate = _clk_ldb_di_round_rate,
	.get_rate = _clk_ldb_di0_get_rate,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static unsigned long _clk_ldb_di1_get_rate(struct clk *clk)
{
	u32 div;

	div = __raw_readl(MXC_CCM_CSCMR2) &
		MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;

	if (div)
		return clk_get_rate(clk->parent) / 7;

	return (2 * clk_get_rate(clk->parent)) / 7;
}

static int _clk_ldb_di1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div = 0;
	u32 parent_rate = clk_get_rate(clk->parent);

	if (rate * 7 <= parent_rate + parent_rate/20) {
		div = 7;
		rate = parent_rate / 7;
	} else
		rate = 2 * parent_rate / 7;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	if (div == 7)
		reg |= MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	else
		reg &= ~MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;

	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static int _clk_ldb_di1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	int rev = mx6q_revision();

	reg = __raw_readl(MXC_CCM_CS2CDR)
		& ~MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK;

	mux = _get_mux6(parent, &pll5_video_main_clk,
		&pll2_pfd_352M, &pll2_pfd_400M,
		(rev == IMX_CHIP_REVISION_1_0) ?
		 &pll3_pfd_540M :	/* MX6Q TO1.0 */
		 &mmdc_ch1_axi_clk[0],	/* MX6Q TO1.1 and MX6DL */
		&pll3_usb_otg_main_clk, NULL);
	reg |= (mux << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);

	__raw_writel(reg, MXC_CCM_CS2CDR);

	return 0;
}

static struct clk ldb_di1_clk = {
	 __INIT_CLK_DEBUG(ldb_di1_clk)
	.id = 0,
	.parent = &pll2_pfd_352M,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ldb_di1_set_parent,
	.set_rate = _clk_ldb_di1_set_rate,
	.round_rate = _clk_ldb_di_round_rate,
	.get_rate = _clk_ldb_di1_get_rate,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};


static unsigned long _clk_ipu_di_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	if ((clk->parent == &ldb_di0_clk) ||
		(clk->parent == &ldb_di1_clk))
		return parent_rate;

	div = parent_rate / rate;
	/* Round to closest divisor */
	if ((parent_rate % rate) > (rate / 2))
		div++;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static unsigned long _clk_ipu1_di0_get_rate(struct clk *clk)
{
	u32 reg, div;

	if ((clk->parent == &ldb_di0_clk) ||
		(clk->parent == &ldb_di1_clk))
		return clk_get_rate(clk->parent);

	reg = __raw_readl(MXC_CCM_CHSCCDR);

	div = ((reg & MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK) >>
			 MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_ipu1_di0_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	if ((clk->parent == &ldb_di0_clk) ||
		(clk->parent == &ldb_di1_clk)) {
		if (parent_rate == rate)
			return 0;
		else
			return -EINVAL;
	}

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CHSCCDR);
	reg &= ~MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CHSCCDR);

	return 0;
}


static int _clk_ipu1_di0_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	if (parent == &ldb_di0_clk)
		mux = 0x3;
	else if (parent == &ldb_di1_clk)
		mux = 0x4;
	else {
		reg = __raw_readl(MXC_CCM_CHSCCDR)
			& ~MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK;

		mux = _get_mux6(parent, &mmdc_ch0_axi_clk[0],
				&pll3_usb_otg_main_clk, &pll5_video_main_clk,
				&pll2_pfd_352M, &pll2_pfd_400M, &pll3_pfd_540M);
		reg |= (mux << MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET);

		__raw_writel(reg, MXC_CCM_CHSCCDR);

		/* Derive clock from divided pre-muxed ipu1_di0 clock.*/
		mux = 0;
	}

	reg = __raw_readl(MXC_CCM_CHSCCDR)
		& ~MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK;
	__raw_writel(reg | (mux << MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET),
		MXC_CCM_CHSCCDR);

	return 0;
}

static unsigned long _clk_ipu1_di1_get_rate(struct clk *clk)
{
	u32 reg, div;

	if ((clk->parent == &ldb_di0_clk) ||
		(clk->parent == &ldb_di1_clk))
		return clk_get_rate(clk->parent);

	reg = __raw_readl(MXC_CCM_CHSCCDR);

	div = ((reg & MXC_CCM_CHSCCDR_IPU1_DI1_PODF_MASK)
			>> MXC_CCM_CHSCCDR_IPU1_DI1_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_ipu1_di1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	if ((clk->parent == &ldb_di0_clk) ||
		(clk->parent == &ldb_di1_clk)) {
		if (parent_rate == rate)
			return 0;
		else
			return -EINVAL;
	}

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CHSCCDR);
	reg &= ~MXC_CCM_CHSCCDR_IPU1_DI1_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CHSCCDR_IPU1_DI1_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CHSCCDR);

	return 0;
}


static int _clk_ipu1_di1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	if (parent == &ldb_di0_clk)
		mux = 0x3;
	else if (parent == &ldb_di1_clk)
		mux = 0x4;
	else {
		reg = __raw_readl(MXC_CCM_CHSCCDR)
			& ~MXC_CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_MASK;

		mux = _get_mux6(parent, &mmdc_ch0_axi_clk[0],
				&pll3_usb_otg_main_clk, &pll5_video_main_clk,
				&pll2_pfd_352M, &pll2_pfd_400M, &pll3_pfd_540M);
		reg |= (mux << MXC_CCM_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_OFFSET);

		__raw_writel(reg, MXC_CCM_CHSCCDR);

		/* Derive clock from divided pre-muxed ipu1_di0 clock.*/
		mux = 0;
	}
	reg = __raw_readl(MXC_CCM_CHSCCDR)
		& ~MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_MASK;
	__raw_writel(reg | (mux << MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET),
		MXC_CCM_CHSCCDR);

	return 0;
}

static struct clk ipu1_di_clk[] = {
	{
	 __INIT_CLK_DEBUG(ipu1_di_clk_0)
	.id = 0,
	.parent = &pll5_video_main_clk,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ipu1_di0_set_parent,
	.set_rate = _clk_ipu1_di0_set_rate,
	.round_rate = _clk_ipu_di_round_rate,
	.get_rate = _clk_ipu1_di0_get_rate,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	 __INIT_CLK_DEBUG(ipu1_di_clk_1)
	.id = 1,
	.parent = &pll5_video_main_clk,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ipu1_di1_set_parent,
	.set_rate = _clk_ipu1_di1_set_rate,
	.round_rate = _clk_ipu_di_round_rate,
	.get_rate = _clk_ipu1_di1_get_rate,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
};

static unsigned long _clk_ipu2_di0_get_rate(struct clk *clk)
{
	u32 reg, div;

	if ((clk->parent == &ldb_di0_clk) ||
		(clk->parent == &ldb_di1_clk))
		return clk_get_rate(clk->parent);

	reg = __raw_readl(MXC_CCM_CSCDR2);

	div = ((reg & MXC_CCM_CSCDR2_IPU2_DI0_PODF_MASK) >>
			MXC_CCM_CSCDR2_IPU2_DI0_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_ipu2_di0_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	if ((clk->parent == &ldb_di0_clk) ||
		(clk->parent == &ldb_di1_clk)) {
		if (parent_rate == rate)
			return 0;
		else
			return -EINVAL;
	}

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR2);
	reg &= ~MXC_CCM_CSCDR2_IPU2_DI0_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCDR2_IPU2_DI0_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	return 0;
}

static int _clk_ipu2_di0_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	if (parent == &ldb_di0_clk)
		mux = 0x3;
	else if (parent == &ldb_di1_clk)
		mux = 0x4;
	else {
		reg = __raw_readl(MXC_CCM_CSCDR2)
			& ~MXC_CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_MASK;

		mux = _get_mux6(parent, &mmdc_ch0_axi_clk[0],
				&pll3_usb_otg_main_clk, &pll5_video_main_clk,
				&pll2_pfd_352M, &pll2_pfd_400M, &pll3_pfd_540M);
		reg |= (mux << MXC_CCM_CSCDR2_IPU2_DI0_PRE_CLK_SEL_OFFSET);

		__raw_writel(reg, MXC_CCM_CSCDR2);

		/* Derive clock from divided pre-muxed ipu2_di0 clock.*/
		mux = 0;
	}
	reg = __raw_readl(MXC_CCM_CSCDR2)
		& ~MXC_CCM_CSCDR2_IPU2_DI0_CLK_SEL_MASK;
	__raw_writel(reg | (mux << MXC_CCM_CSCDR2_IPU2_DI0_CLK_SEL_OFFSET),
		MXC_CCM_CSCDR2);

	return 0;
}

static unsigned long _clk_ipu2_di1_get_rate(struct clk *clk)
{
	u32 reg, div;

	if ((clk->parent == &ldb_di0_clk) ||
		(clk->parent == &ldb_di1_clk))
		return clk_get_rate(clk->parent);

	reg = __raw_readl(MXC_CCM_CSCDR2);

	div = ((reg & MXC_CCM_CSCDR2_IPU2_DI1_PODF_MASK)
		>> MXC_CCM_CSCDR2_IPU2_DI1_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_ipu2_di1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	if ((clk->parent == &ldb_di0_clk) ||
		(clk->parent == &ldb_di1_clk)) {
		if (parent_rate == rate)
			return 0;
		else
			return -EINVAL;
	}

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR2);
	reg &= ~MXC_CCM_CSCDR2_IPU2_DI1_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCDR2_IPU2_DI1_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	return 0;
}

static int _clk_ipu2_di1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	if (parent == &ldb_di0_clk)
		mux = 0x3;
	else if (parent == &ldb_di1_clk)
		mux = 0x4;
	else {
		reg = __raw_readl(MXC_CCM_CSCDR2)
			& ~MXC_CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_MASK;

		mux = _get_mux6(parent, &mmdc_ch0_axi_clk[0],
				&pll3_usb_otg_main_clk, &pll5_video_main_clk,
				&pll2_pfd_352M, &pll2_pfd_400M, &pll3_pfd_540M);
		reg |= (mux << MXC_CCM_CSCDR2_IPU2_DI1_PRE_CLK_SEL_OFFSET);

		__raw_writel(reg, MXC_CCM_CSCDR2);

		/* Derive clock from divided pre-muxed ipu1_di0 clock.*/
		mux = 0;
	}
	reg = __raw_readl(MXC_CCM_CSCDR2)
		& ~MXC_CCM_CSCDR2_IPU2_DI1_CLK_SEL_MASK;
	__raw_writel(reg | (mux << MXC_CCM_CSCDR2_IPU2_DI1_CLK_SEL_OFFSET),
		MXC_CCM_CSCDR2);

	return 0;
}

static struct clk ipu2_di_clk[] = {
	{
	 __INIT_CLK_DEBUG(ipu2_di_clk_0)
	.id = 0,
	.parent = &pll5_video_main_clk,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ipu2_di0_set_parent,
	.set_rate = _clk_ipu2_di0_set_rate,
	.round_rate = _clk_ipu_di_round_rate,
	.get_rate = _clk_ipu2_di0_get_rate,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	 __INIT_CLK_DEBUG(ipu2_di_clk_1)
	.id = 1,
	.parent = &pll5_video_main_clk,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_ipu2_di1_set_parent,
	.set_rate = _clk_ipu2_di1_set_rate,
	.round_rate = _clk_ipu_di_round_rate,
	.get_rate = _clk_ipu2_di1_get_rate,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
};

static unsigned long _clk_can_root_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 64)
		div = 64;

	return parent_rate / div;
}

static int _clk_can_root_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 64))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCMR2) & MXC_CCM_CSCMR2_CAN_CLK_PODF_MASK;
	reg |= ((div - 1) << MXC_CCM_CSCMR2_CAN_CLK_PODF_OFFSET);

	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static unsigned long _clk_can_root_get_rate(struct clk *clk)
{
	u32 reg, div;
	unsigned long val;

	reg = __raw_readl(MXC_CCM_CSCMR2) & MXC_CCM_CSCMR2_CAN_CLK_PODF_MASK;
	div = (reg >> MXC_CCM_CSCMR2_CAN_CLK_PODF_OFFSET) + 1;
	val = clk_get_rate(clk->parent) / div;

	return val;
}

static struct clk can_clk_root = {
	 __INIT_CLK_DEBUG(can_clk_root)
	.parent = &pll3_60M,
	.set_rate = _clk_can_root_set_rate,
	.get_rate = _clk_can_root_get_rate,
	.round_rate = _clk_can_root_round_rate,
};

static struct clk can2_clk[] = {
	{
	 __INIT_CLK_DEBUG(can2_module_clk)
	.id = 0,
	.parent = &can_clk_root,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &can2_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	 __INIT_CLK_DEBUG(can2_serial_clk)
	.id = 1,
	.parent = &can_clk_root,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};


static struct clk can1_clk[] = {
	{
	 __INIT_CLK_DEBUG(can1_module_clk)
	.id = 0,
	.parent = &can_clk_root,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &can1_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	 __INIT_CLK_DEBUG(can1_serial_clk)
	.id = 1,
	.parent = &can_clk_root,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};

static unsigned long _clk_spdif_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (parent_rate % rate)
		div++;

	__calc_pre_post_dividers(1 << 3, div, &pre, &post);

	return parent_rate / (pre * post);
}

static int _clk_spdif0_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CDCDR)
		& ~MXC_CCM_CDCDR_SPDIF0_CLK_SEL_MASK;

	mux = _get_mux6(parent, &pll4_audio_main_clk,
		&pll3_pfd_508M, &pll3_pfd_454M,
		&pll3_sw_clk, NULL, NULL);
	reg |= mux << MXC_CCM_CDCDR_SPDIF0_CLK_SEL_OFFSET;

	__raw_writel(reg, MXC_CCM_CDCDR);

	return 0;
}

static unsigned long _clk_spdif0_get_rate(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CDCDR);

	pred = ((reg & MXC_CCM_CDCDR_SPDIF0_CLK_PRED_MASK)
		>> MXC_CCM_CDCDR_SPDIF0_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CDCDR_SPDIF0_CLK_PODF_MASK)
		>> MXC_CCM_CDCDR_SPDIF0_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (pred * podf);
}

static int _clk_spdif0_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || div > 64)
		return -EINVAL;

	__calc_pre_post_dividers(1 << 3, div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CDCDR);
	reg &= ~(MXC_CCM_CDCDR_SPDIF0_CLK_PRED_MASK|
		 MXC_CCM_CDCDR_SPDIF0_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CDCDR_SPDIF0_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CDCDR_SPDIF0_CLK_PRED_OFFSET;

	__raw_writel(reg, MXC_CCM_CDCDR);

	return 0;
}

static struct clk spdif0_clk[] = {
	{
	__INIT_CLK_DEBUG(spdif0_clk_0)
	.id = 0,
	.parent = &pll3_sw_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &spdif0_clk[1],
	 .set_rate = _clk_spdif0_set_rate,
	 .get_rate = _clk_spdif0_get_rate,
	 .set_parent = _clk_spdif0_set_parent,
	 .round_rate = _clk_spdif_round_rate,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	__INIT_CLK_DEBUG(spdif0_clk_1)
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 },
};

static unsigned long _clk_esai_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (parent_rate % rate)
		div++;

	__calc_pre_post_dividers(1 << 3, div, &pre, &post);

	return parent_rate / (pre * post);
}

static int _clk_esai_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2) & ~MXC_CCM_CSCMR2_ESAI_CLK_SEL_MASK;

	mux = _get_mux6(parent, &pll4_audio_main_clk, &pll3_pfd_508M,
			&pll3_pfd_454M,	&pll3_sw_clk, NULL, NULL);
	reg |= mux << MXC_CCM_CSCMR2_ESAI_CLK_SEL_OFFSET;

	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static unsigned long _clk_esai_get_rate(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CS1CDR);

	pred = ((reg & MXC_CCM_CS1CDR_ESAI_CLK_PRED_MASK)
		>> MXC_CCM_CS1CDR_ESAI_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CS1CDR_ESAI_CLK_PODF_MASK)
		>> MXC_CCM_CS1CDR_ESAI_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (pred * podf);
}

static int _clk_esai_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || div > 64)
		return -EINVAL;

	__calc_pre_post_dividers(1 << 3, div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg &= ~(MXC_CCM_CS1CDR_ESAI_CLK_PRED_MASK|
		 MXC_CCM_CS1CDR_ESAI_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CS1CDR_ESAI_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CS1CDR_ESAI_CLK_PRED_OFFSET;

	__raw_writel(reg, MXC_CCM_CS1CDR);

	return 0;
}

static struct clk esai_clk = {
	__INIT_CLK_DEBUG(esai_clk)
	 .id = 0,
	 .parent = &pll3_sw_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .set_rate = _clk_esai_set_rate,
	 .get_rate = _clk_esai_get_rate,
	 .set_parent = _clk_esai_set_parent,
	 .round_rate = _clk_esai_round_rate,
};

static int _clk_enet_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int reg, div = 1;

	switch (rate) {
	case 25000000:
		div = 0;
		break;
	case 50000000:
		div = 1;
		break;
	case 100000000:
		div = 2;
		break;
	case 125000000:
		div = 3;
		break;
	default:
		return -EINVAL;
	}
	reg = __raw_readl(PLL8_ENET_BASE_ADDR);
	reg &= ~ANADIG_PLL_ENET_DIV_SELECT_MASK;
	reg |= (div << ANADIG_PLL_ENET_DIV_SELECT_OFFSET);
	__raw_writel(reg, PLL8_ENET_BASE_ADDR);

	return 0;
}

static unsigned long _clk_enet_get_rate(struct clk *clk)
{
	unsigned int div;

	div = (__raw_readl(PLL8_ENET_BASE_ADDR))
		& ANADIG_PLL_ENET_DIV_SELECT_MASK;

	switch (div) {
	case 0:
		div = 20;
		break;
	case 1:
		div = 10;
		break;
	case 3:
		div = 5;
		break;
	case 4:
		div = 4;
		break;
	}

	return 500000000 / div;
}

static int _clk_enet_enable(struct clk *clk)
{
#ifndef CONFIG_MX6_ENET_IRQ_TO_GPIO
	enet_is_active = true;
#endif
	_clk_enable(clk);
	return 0;
}

static void _clk_enet_disable(struct clk *clk)
{
	_clk_disable(clk);
#ifndef CONFIG_MX6_ENET_IRQ_TO_GPIO
	enet_is_active = false;
#endif
}

static struct clk enet_clk[] = {
	{
	__INIT_CLK_DEBUG(enet_clk)
	 .id = 0,
	 .parent = &pll8_enet_main_clk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	 .enable = _clk_enet_enable,
	 .disable = _clk_enet_disable,
	 .set_rate = _clk_enet_set_rate,
	 .get_rate = _clk_enet_get_rate,
	.secondary = &enet_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	.parent = &mmdc_ch0_axi_clk[0],
	.secondary = &mx6per1_clk,
	},
};

static struct clk ecspi_clk[] = {
	{
	__INIT_CLK_DEBUG(ecspi0_clk)
	.id = 0,
	.parent = &pll3_60M,
	.secondary = &spba_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
	{
	__INIT_CLK_DEBUG(ecspi1_clk)
	.id = 1,
	.parent = &pll3_60M,
	.secondary = &spba_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
	{
	__INIT_CLK_DEBUG(ecspi2_clk)
	.id = 2,
	.parent = &pll3_60M,
	.secondary = &spba_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
	{
	__INIT_CLK_DEBUG(ecspi3_clk)
	.id = 3,
	.parent = &pll3_60M,
	.secondary = &spba_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
	{
	__INIT_CLK_DEBUG(ecspi4_clk)
	.id = 4,
	.parent = &pll3_60M,
	.secondary = &spba_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};

static unsigned long _clk_emi_slow_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static int _clk_emi_slow_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CSCMR1)
		& ~MXC_CCM_CSCMR1_ACLK_EMI_SLOW_MASK;

	mux = _get_mux6(parent, &axi_clk, &pll3_usb_otg_main_clk,
				&pll2_pfd_400M, &pll2_pfd_352M, NULL, NULL);
	reg |= (mux << MXC_CCM_CSCMR1_ACLK_EMI_SLOW_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static unsigned long _clk_emi_slow_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	div = ((reg & MXC_CCM_CSCMR1_ACLK_EMI_SLOW_PODF_MASK) >>
			MXC_CCM_CSCMR1_ACLK_EMI_SLOW_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_emi_slow_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	reg &= ~MXC_CCM_CSCMR1_ACLK_EMI_SLOW_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCMR1_ACLK_EMI_SLOW_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk emi_slow_clk = {
	__INIT_CLK_DEBUG(emi_slow_clk)
	 .id = 0,
	 .parent = &axi_clk,
	 .enable_reg = MXC_CCM_CCGR6,
	 .enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .set_rate = _clk_emi_slow_set_rate,
	 .get_rate = _clk_emi_slow_get_rate,
	 .round_rate = _clk_emi_slow_round_rate,
	 .set_parent = _clk_emi_slow_set_parent,
};

static unsigned long _clk_emi_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static int _clk_emi_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_ACLK_EMI_MASK;

	mux = _get_mux6(parent, &axi_clk, &pll3_usb_otg_main_clk,
			&pll2_pfd_400M, &pll2_pfd_352M, NULL, NULL);
	reg |= (mux << MXC_CCM_CSCMR1_ACLK_EMI_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static unsigned long _clk_emi_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	div = ((reg & MXC_CCM_CSCMR1_ACLK_EMI_PODF_MASK) >>
			MXC_CCM_CSCMR1_ACLK_EMI_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_emi_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	reg &= ~MXC_CCM_CSCMR1_ACLK_EMI_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CSCMR1_ACLK_EMI_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk emi_clk = {
	__INIT_CLK_DEBUG(emi_clk)
	 .id = 0,
	 .parent = &axi_clk,
	 .set_rate = _clk_emi_set_rate,
	 .get_rate = _clk_emi_get_rate,
	 .round_rate = _clk_emi_round_rate,
	 .set_parent = _clk_emi_set_parent,
};

static unsigned long _clk_enfc_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (parent_rate % rate)
		div++;

	__calc_pre_post_dividers(1 << 6, div, &pre, &post);

	return parent_rate / (pre * post);
}

static int _clk_enfc_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CS2CDR)
		& ~MXC_CCM_CS2CDR_ENFC_CLK_SEL_MASK;

	mux = _get_mux6(parent, &pll2_pfd_352M,
		&pll2_528_bus_main_clk, &pll3_usb_otg_main_clk,
		&pll2_pfd_400M, NULL, NULL);
	reg |= mux << MXC_CCM_CS2CDR_ENFC_CLK_SEL_OFFSET;

	__raw_writel(reg, MXC_CCM_CS2CDR);

	return 0;
}

static unsigned long _clk_enfc_get_rate(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CS2CDR);

	pred = ((reg & MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK)
		>> MXC_CCM_CS2CDR_ENFC_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK)
		>> MXC_CCM_CS2CDR_ENFC_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (pred * podf);
}

static int _clk_enfc_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || div > 512)
		return -EINVAL;

	__calc_pre_post_dividers(1 << 6, div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CS2CDR);
	reg &= ~(MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK|
		 MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CS2CDR_ENFC_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CS2CDR_ENFC_CLK_PRED_OFFSET;

	__raw_writel(reg, MXC_CCM_CS2CDR);

	return 0;
}

static struct clk enfc_clk = {
	__INIT_CLK_DEBUG(enfc_clk)
	 .id = 0,
	 .parent = &pll2_pfd_352M,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGRx_CG7_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .set_rate = _clk_enfc_set_rate,
	 .get_rate = _clk_enfc_get_rate,
	 .round_rate = _clk_enfc_round_rate,
	 .set_parent = _clk_enfc_set_parent,
};

static unsigned long _clk_uart_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 64)
		div = 64;

	return parent_rate / div;
}

static int _clk_uart_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 64))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CSCDR1) & MXC_CCM_CSCDR1_UART_CLK_PODF_MASK;
	reg |= ((div - 1) << MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET);

	__raw_writel(reg, MXC_CCM_CSCDR1);

	return 0;
}

static unsigned long _clk_uart_get_rate(struct clk *clk)
{
	u32 reg, div;
	unsigned long val;

	reg = __raw_readl(MXC_CCM_CSCDR1) & MXC_CCM_CSCDR1_UART_CLK_PODF_MASK;
	div = (reg >> MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET) + 1;
	val = clk_get_rate(clk->parent) / div;

	return val;
}

static struct clk uart_clk[] = {
	{
	__INIT_CLK_DEBUG(uart_clk)
	 .id = 0,
	 .parent = &pll3_80M,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &uart_clk[1],
	 .set_rate = _clk_uart_set_rate,
	 .get_rate = _clk_uart_get_rate,
	 .round_rate = _clk_uart_round_rate,
	},
	{
	__INIT_CLK_DEBUG(uart_serial_clk)
	 .id = 1,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	},
};

static unsigned long _clk_hsi_tx_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static int _clk_hsi_tx_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg = __raw_readl(MXC_CCM_CDCDR) & ~MXC_CCM_CDCDR_HSI_TX_CLK_SEL;

	if (parent == &pll2_pfd_400M)
		reg |= (MXC_CCM_CDCDR_HSI_TX_CLK_SEL);

	__raw_writel(reg, MXC_CCM_CDCDR);

	return 0;
}

static unsigned long _clk_hsi_tx_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CDCDR);
	div = ((reg & MXC_CCM_CDCDR_HSI_TX_PODF_MASK) >>
			MXC_CCM_CDCDR_HSI_TX_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_hsi_tx_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CDCDR);
	reg &= ~MXC_CCM_CDCDR_HSI_TX_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CDCDR_HSI_TX_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CDCDR);

	return 0;
}

static struct clk hsi_tx_clk[] = {
	{
	 __INIT_CLK_DEBUG(hsi_tx_clk)
	.id = 0,
	 .parent = &pll2_pfd_400M,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.set_parent = _clk_hsi_tx_set_parent,
	.round_rate = _clk_hsi_tx_round_rate,
	.set_rate = _clk_hsi_tx_set_rate,
	.get_rate = _clk_hsi_tx_get_rate,
	 .secondary = &hsi_tx_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	.parent = &mx6per1_clk,
	.secondary = &mx6per2_clk,
	 },
};

static struct clk mipi_pllref_clk = {
	 __INIT_CLK_DEBUG(mipi_pllref_clk)
	.id = 0,
	.parent = &pll3_pfd_540M,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk hdmi_clk[] = {
	{
	 __INIT_CLK_DEBUG(hdmi_isfr_clk)
	.id = 0,
	.parent = &pll3_pfd_540M,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
	{
	 __INIT_CLK_DEBUG(hdmi_iahb_clk)
	.id = 1,
	 .parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
};

static struct clk caam_clk[] = {
	{
	 __INIT_CLK_DEBUG(caam_mem_clk)
	.id = 0,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &caam_clk[1],
	},
	{
	 __INIT_CLK_DEBUG(caam_aclk_clk)
	.id = 1,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG5_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &caam_clk[2],
	},
	{
	 __INIT_CLK_DEBUG(caam_ipg_clk)
	.id = 2,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG6_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.parent = &mmdc_ch0_axi_clk[0],
	.secondary = &mx6per1_clk,
	},
};

static int _clk_asrc_serial_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CDCDR) & ~MXC_CCM_CDCDR_SPDIF1_CLK_SEL_MASK;

	mux = _get_mux6(parent, &pll4_audio_main_clk, &pll3_pfd_508M,
			&pll3_pfd_454M,	&pll3_sw_clk, NULL, NULL);
	reg |= mux << MXC_CCM_CDCDR_SPDIF1_CLK_SEL_OFFSET;

	__raw_writel(reg, MXC_CCM_CDCDR);

	return 0;
}

static unsigned long _clk_asrc_serial_get_rate(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CDCDR);

	pred = ((reg & MXC_CCM_CDCDR_SPDIF1_CLK_PRED_MASK)
		>> MXC_CCM_CDCDR_SPDIF1_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CDCDR_SPDIF1_CLK_PODF_MASK)
		>> MXC_CCM_CDCDR_SPDIF1_CLK_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / (pred * podf);
}

static int _clk_asrc_serial_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || div > 64)
		return -EINVAL;

	__calc_pre_post_dividers(1 << 3, div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CDCDR);
	reg &= ~(MXC_CCM_CDCDR_SPDIF1_CLK_PRED_MASK|
		 MXC_CCM_CDCDR_SPDIF1_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CDCDR_SPDIF1_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CDCDR_SPDIF1_CLK_PRED_OFFSET;

	__raw_writel(reg, MXC_CCM_CDCDR);

	return 0;
}

static unsigned long _clk_asrc_serial_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 pre, post;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (parent_rate % rate)
		div++;

	__calc_pre_post_dividers(1 << 3, div, &pre, &post);

	return parent_rate / (pre * post);
}

static struct clk asrc_clk[] = {
	{
	__INIT_CLK_DEBUG(asrc_clk)
	.id = 0,
	.parent = &pll4_audio_main_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &spba_clk,
	},
	{
	/*In the MX6 spec, asrc_serial_clk is listed as SPDIF1 clk
	 * This clock can never be gated and does  not have any
	 * CCGR bits associated with it.
	 */
	__INIT_CLK_DEBUG(asrc_serial_clk)
	.id = 1,
	.parent = &pll3_sw_clk,
	 .set_rate = _clk_asrc_serial_set_rate,
	 .get_rate = _clk_asrc_serial_get_rate,
	 .set_parent = _clk_asrc_serial_set_parent,
	 .round_rate = _clk_asrc_serial_round_rate,
	},
};

static struct clk apbh_dma_clk = {
	__INIT_CLK_DEBUG(apbh_dma_clk)
	.parent = &usdhc3_clk,
	.secondary = &mx6per1_clk,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
};

static struct clk aips_tz2_clk = {
	__INIT_CLK_DEBUG(aips_tz2_clk)
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk aips_tz1_clk = {
	__INIT_CLK_DEBUG(aips_tz1_clk)
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};


static struct clk openvg_axi_clk = {
	__INIT_CLK_DEBUG(openvg_axi_clk)
	.parent = &gpu2d_axi_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static unsigned long _clk_gpu3d_core_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static int _clk_gpu3d_core_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CBCMR)
		& ~MXC_CCM_CBCMR_GPU3D_CORE_CLK_SEL_MASK;

	mux = _get_mux6(parent, &mmdc_ch0_axi_clk[0],
		&pll3_usb_otg_main_clk,
		&pll2_pfd_594M, &pll2_pfd_400M, NULL, NULL);
	reg |= (mux << MXC_CCM_CBCMR_GPU3D_CORE_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static unsigned long _clk_gpu3d_core_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCMR);
	div = ((reg & MXC_CCM_CBCMR_GPU3D_CORE_PODF_MASK) >>
			MXC_CCM_CBCMR_GPU3D_CORE_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_gpu3d_core_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (div > 8)
		div = 8;

	reg = __raw_readl(MXC_CCM_CBCMR);
	reg &= ~MXC_CCM_CBCMR_GPU3D_CORE_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCMR_GPU3D_CORE_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk gpu3d_core_clk[] = {
	{
	__INIT_CLK_DEBUG(gpu3d_core_clk)
	.parent = &pll2_pfd_594M,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.set_parent = _clk_gpu3d_core_set_parent,
	.set_rate = _clk_gpu3d_core_set_rate,
	.get_rate = _clk_gpu3d_core_get_rate,
	.round_rate = _clk_gpu3d_core_round_rate,
	.secondary = &gpu3d_core_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	.parent = &gpu3d_axi_clk,
	.secondary = &mx6fast1_clk,
	},
};

static unsigned long _clk_gpu2d_core_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static int _clk_gpu2d_core_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CBCMR) &
				~MXC_CCM_CBCMR_GPU2D_CLK_SEL_MASK;

	/*on mx6dl, 2d core clock sources from 3d shader core clock*/
	if (!cpu_is_mx6dl()) {
		mux = _get_mux6(parent, &axi_clk, &pll3_usb_otg_main_clk,
			&pll2_pfd_352M, &pll2_pfd_400M, NULL, NULL);
		reg |= (mux << MXC_CCM_CBCMR_GPU2D_CLK_SEL_OFFSET);
		__raw_writel(reg, MXC_CCM_CBCMR);
	}

	return 0;
}

static unsigned long _clk_gpu2d_core_get_rate(struct clk *clk)
{
	u32 reg, div = 1;

	reg = __raw_readl(MXC_CCM_CBCMR);
	if (cpu_is_mx6q())
		div = ((reg & MXC_CCM_CBCMR_GPU2D_CORE_PODF_MASK) >>
				MXC_CCM_CBCMR_GPU2D_CORE_PODF_OFFSET) + 1;
	else if (cpu_is_mx6dl())
		/* on i.mx6dl, gpu2d_core_clk source from gpu3d_shader_clk */
		return clk_get_rate(clk->parent);

	return clk_get_rate(clk->parent) / div;
}

static int _clk_gpu2d_core_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CBCMR);
	reg &= ~MXC_CCM_CBCMR_GPU2D_CORE_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCMR_GPU2D_CORE_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}
static struct clk gpu2d_core_clk[] = {
	{
	__INIT_CLK_DEBUG(gpu2d_core_clk)
	.parent = &pll2_pfd_352M,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.disable = _clk_disable,
	.set_parent = _clk_gpu2d_core_set_parent,
	.set_rate = _clk_gpu2d_core_set_rate,
	.get_rate = _clk_gpu2d_core_get_rate,
	.round_rate = _clk_gpu2d_core_round_rate,
	.secondary = &mx6fast1_clk,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
};

static unsigned long _clk_gpu3d_shader_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;

	if (div > 8)
		div = 8;

	return parent_rate / div;
}

static int _clk_gpu3d_shader_set_parent(struct clk *clk, struct clk *parent)
{
	int mux;
	u32 reg = __raw_readl(MXC_CCM_CBCMR)
		& ~MXC_CCM_CBCMR_GPU3D_SHADER_CLK_SEL_MASK;

	mux = _get_mux6(parent, &mmdc_ch0_axi_clk[0],
		&pll3_usb_otg_main_clk,
		&pll2_pfd_594M, &pll3_pfd_720M, NULL, NULL);
	reg |= (mux << MXC_CCM_CBCMR_GPU3D_SHADER_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static unsigned long _clk_gpu3d_shader_get_rate(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCMR);
	div = ((reg & MXC_CCM_CBCMR_GPU3D_SHADER_PODF_MASK) >>
			MXC_CCM_CBCMR_GPU3D_SHADER_PODF_OFFSET) + 1;

	return clk_get_rate(clk->parent) / div;
}

static int _clk_gpu3d_shader_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	u32 parent_rate = clk_get_rate(clk->parent);

	div = parent_rate / rate;
	if (div == 0)
		div++;
	if (div > 8)
		div = 8;

	reg = __raw_readl(MXC_CCM_CBCMR);
	reg &= ~MXC_CCM_CBCMR_GPU3D_SHADER_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCMR_GPU3D_SHADER_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}


static struct clk gpu3d_shader_clk = {
	__INIT_CLK_DEBUG(gpu3d_shader_clk)
	.parent = &pll3_pfd_720M,
	.secondary = &mmdc_ch0_axi_clk[0],
	.set_parent = _clk_gpu3d_shader_set_parent,
	.set_rate = _clk_gpu3d_shader_set_rate,
	.get_rate = _clk_gpu3d_shader_get_rate,
	.round_rate = _clk_gpu3d_shader_round_rate,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

/* set the parent by the ipcg table */
static struct clk gpmi_nand_clk[] = {
	{	/* gpmi_io_clk */
	__INIT_CLK_DEBUG(gpmi_io_clk)
	.parent = &enfc_clk,
	.secondary = &gpmi_nand_clk[1],
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG14_OFFSET,
	.disable = _clk_disable,
	},
	{	/* gpmi_apb_clk */
	__INIT_CLK_DEBUG(gpmi_apb_clk)
	.parent = &usdhc3_clk,
	.secondary = &gpmi_nand_clk[2],
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG15_OFFSET,
	.disable = _clk_disable,
	},
	{	/* bch_clk */
	__INIT_CLK_DEBUG(gpmi_bch_clk)
	.parent = &usdhc4_clk,
	.secondary = &gpmi_nand_clk[3],
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG13_OFFSET,
	.disable = _clk_disable,
	},
	{	/* bch_apb_clk */
	__INIT_CLK_DEBUG(gpmi_bch_apb_clk)
	.parent = &usdhc3_clk,
	.secondary = &gpmi_nand_clk[4],
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG12_OFFSET,
	.disable = _clk_disable,
	},
	{	/* bch relative clk */
	__INIT_CLK_DEBUG(pl301_mx6qperl_bch)
	.parent = &mx6per1_clk,
	.secondary = &mmdc_ch0_axi_clk[0],
	},
};

static struct clk pwm_clk[] = {
	{
	__INIT_CLK_DEBUG(pwm_clk_0)
	 .parent = &ipg_perclk,
	 .id = 0,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGRx_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	__INIT_CLK_DEBUG(pwm_clk_1)
	 .parent = &ipg_perclk,
	 .id = 1,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	__INIT_CLK_DEBUG(pwm_clk_2)
	 .parent = &ipg_perclk,
	 .id = 2,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGRx_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	__INIT_CLK_DEBUG(pwm_clk_3)
	 .parent = &ipg_perclk,
	 .id = 3,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGRx_CG11_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static int _clk_sata_enable(struct clk *clk)
{
	unsigned int reg;

	/* Enable SATA ref clock */
	reg = __raw_readl(PLL8_ENET_BASE_ADDR);
	reg |= ANADIG_PLL_ENET_EN_SATA;
	__raw_writel(reg, PLL8_ENET_BASE_ADDR);

	_clk_enable(clk);

	return 0;
}

static void _clk_sata_disable(struct clk *clk)
{
	unsigned int reg;

	_clk_disable(clk);

	/* Disable SATA ref clock */
	reg = __raw_readl(PLL8_ENET_BASE_ADDR);
	reg &= ~ANADIG_PLL_ENET_EN_SATA;
	__raw_writel(reg, PLL8_ENET_BASE_ADDR);
}

static struct clk sata_clk[] = {
	{
	__INIT_CLK_DEBUG(sata_clk)
	.parent = &pll8_enet_main_clk,
	.enable = _clk_sata_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.disable = _clk_sata_disable,
	.secondary = &sata_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	.parent = &ipg_clk,
	.secondary = &sata_clk[2],
	},
	{
	.parent = &mmdc_ch0_axi_clk[0],
	.secondary = &mx6per1_clk,
	},
};

static int _clk_pcie_enable(struct clk *clk)
{
	unsigned int reg;

#ifndef CONFIG_IMX_PCIE_RC_MODE_IN_EP_RC_SYS
	/* Activate LVDS CLK1 (the MiniPCIe slot clock input) */
	reg = __raw_readl(ANADIG_MISC1_REG);
	reg &= ~ANATOP_LVDS_CLK1_IBEN_MASK;
	__raw_writel(reg, ANADIG_MISC1_REG);

	reg = __raw_readl(ANADIG_MISC1_REG);
	reg |= ANATOP_LVDS_CLK1_SRC_SATA;
	__raw_writel(reg, ANADIG_MISC1_REG);

	reg = __raw_readl(ANADIG_MISC1_REG);
	reg |= ANATOP_LVDS_CLK1_OBEN_MASK;
	__raw_writel(reg, ANADIG_MISC1_REG);
#endif

	/* Enable PCIE ref clock */
	reg = __raw_readl(PLL8_ENET_BASE_ADDR);
	reg |= ANADIG_PLL_ENET_EN_PCIE;
	__raw_writel(reg, PLL8_ENET_BASE_ADDR);

	_clk_enable(clk);

	return 0;
}

static void _clk_pcie_disable(struct clk *clk)
{
	unsigned int reg;

	_clk_disable(clk);

#ifndef CONFIG_IMX_PCIE_RC_MODE_IN_EP_RC_SYS
	/* De-activate LVDS CLK1 (the MiniPCIe slot clock input) */
	reg = __raw_readl(ANADIG_MISC1_REG);
	reg |= ANATOP_LVDS_CLK1_IBEN_MASK;
	__raw_writel(reg, ANADIG_MISC1_REG);

	reg = __raw_readl(ANADIG_MISC1_REG);
	reg &= ~ANATOP_LVDS_CLK1_SRC_SATA;
	__raw_writel(reg, ANADIG_MISC1_REG);

	reg = __raw_readl(ANADIG_MISC1_REG);
	reg &= ~ANATOP_LVDS_CLK1_OBEN_MASK;
	__raw_writel(reg, ANADIG_MISC1_REG);
#endif

	/* Disable PCIE ref clock */
	reg = __raw_readl(PLL8_ENET_BASE_ADDR);
	reg &= ~ANADIG_PLL_ENET_EN_PCIE;
	__raw_writel(reg, PLL8_ENET_BASE_ADDR);
}

static struct clk pcie_clk[] = {
	{
	__INIT_CLK_DEBUG(pcie_clk)
	.parent = &pcie_axi_clk,
	.enable = _clk_pcie_enable,
	.disable = _clk_pcie_disable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.secondary = &pcie_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	/*
	 * Enable SATA ref clock.
	 * PCIe needs both sides to have the same source of refernce clock,
	 * The SATA reference clock is taken out to link partner.
	 */
	.parent = &sata_clk[0],
	.secondary = &pcie_clk[2],
	},
	{
	.parent = &mmdc_ch0_axi_clk[0],
	.secondary = &mx6fast1_clk,
	},
};

static int _clk_pcie_ep_enable(struct clk *clk)
{
	unsigned int reg;

	/* Enable PCIE ref clock */
	reg = __raw_readl(PLL8_ENET_BASE_ADDR);
	reg |= ANADIG_PLL_ENET_EN_PCIE;
	__raw_writel(reg, PLL8_ENET_BASE_ADDR);

	_clk_enable(clk);

	return 0;
}

static void _clk_pcie_ep_disable(struct clk *clk)
{
	unsigned int reg;

	_clk_disable(clk);

	/* Disable PCIE ref clock */
	reg = __raw_readl(PLL8_ENET_BASE_ADDR);
	reg &= ~ANADIG_PLL_ENET_EN_PCIE;
	__raw_writel(reg, PLL8_ENET_BASE_ADDR);
}

static struct clk pcie_ep_clk[] = {
	{
	__INIT_CLK_DEBUG(pcie_ep_clk)
	.parent = &pcie_axi_clk,
	.enable = _clk_pcie_ep_enable,
	.disable = _clk_pcie_ep_disable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.secondary = &pcie_ep_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	.parent = &pll8_enet_main_clk,
	.secondary = &pcie_ep_clk[2],
	},
	{
	.parent = &mmdc_ch0_axi_clk[0],
	.secondary = &mx6fast1_clk,
	},
};

static struct clk usboh3_clk[] = {
	{
	__INIT_CLK_DEBUG(usboh3_clk)
	.parent = &ahb_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGRx_CG0_OFFSET,
	.disable = _clk_disable,
	.secondary = &usboh3_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	.parent = &mmdc_ch0_axi_clk[0],
	.secondary = &mx6per1_clk,
	},
};

static int _clk_mlb_set_parent(struct clk *clk, struct clk *parent)
{
	u32 sel, cbcmr = __raw_readl(MXC_CCM_CBCMR);

	/*
	 * In Rigel validatioin, the MLB sys_clock isn't using the
	 * right frequency after boot.
	 * In arik, the register CBCMR controls gpu2d clock, not mlb clock,
	 * mlb is sourced from axi clock.
	 * But In rigel, the axi clock is lower than in mx6q, so mlb need to
	 * find a new clock root.
	 * The gpu2d clock is the root of mlb clock in rigel.
	 * Thus we need to add below code in mx6dl.
	 * */
	sel = _get_mux(parent, &axi_clk, &pll3_sw_clk,
			&pll2_pfd_352M, &pll2_pfd_400M);

	cbcmr &= ~MXC_CCM_CBCMR_MLB_CLK_SEL_MASK;
	cbcmr |= sel << MXC_CCM_CBCMR_MLB_CLK_SEL_OFFSET;
	__raw_writel(cbcmr, MXC_CCM_CBCMR);

	return 0;
}

static struct clk mlb150_clk = {
	__INIT_CLK_DEBUG(mlb150_clk)
	.id = 0,
	.secondary = &ocram_clk,
	.set_parent = _clk_mlb_set_parent,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGRx_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static int _clk_enable1(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	return 0;
}

static void _clk_disable1(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(1 << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);
}

static int _clk_clko_set_parent(struct clk *clk, struct clk *parent)
{
	u32 sel, reg;

	if (parent == &pll3_usb_otg_main_clk)
		sel = 0;
	else if (parent == &pll2_528_bus_main_clk)
		sel = 1;
	else if (parent == &pll1_sys_main_clk)
		sel = 2;
	else if (parent == &pll5_video_main_clk)
		sel = 3;
	else if (parent == &axi_clk)
		sel = 5;
	else if (parent == &enfc_clk)
		sel = 6;
	else if (parent == &ipu1_di_clk[0])
		sel = 7;
	else if (parent == &ipu1_di_clk[1])
		sel = 8;
	else if (parent == &ipu2_di_clk[0])
		sel = 9;
	else if (parent == &ipu2_di_clk[1])
		sel = 10;
	else if (parent == &ahb_clk)
		sel = 11;
	else if (parent == &ipg_clk)
		sel = 12;
	else if (parent == &ipg_perclk)
		sel = 13;
	else if (parent == &ckil_clk)
		sel = 14;
	else if (parent == &pll4_audio_main_clk)
		sel = 15;
	else if (parent == &clko2_clk) {
		reg = __raw_readl(MXC_CCM_CCOSR);
		reg |= MXC_CCM_CCOSR_CKOL_MIRROR_CKO2_MASK;
		__raw_writel(reg, MXC_CCM_CCOSR);
		return 0;
	} else
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~(MXC_CCM_CCOSR_CKOL_MIRROR_CKO2_MASK |
		 MXC_CCM_CCOSR_CKOL_SEL_MASK);
	reg |= sel << MXC_CCM_CCOSR_CKOL_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}

static unsigned long _clk_clko_get_rate(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CCOSR);
	u32 div = ((reg & MXC_CCM_CCOSR_CKOL_DIV_MASK) >>
			MXC_CCM_CCOSR_CKOL_DIV_OFFSET) + 1;

	if (clk->parent == &clko2_clk)
		/* clko may output clko2 without divider */
		return clk_get_rate(clk->parent);
	else
		return clk_get_rate(clk->parent) / div;
}

static int _clk_clko_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	/* clko may output clko2 without divider */
	if (clk->parent == &clko2_clk)
		return 0;

	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MXC_CCM_CCOSR_CKOL_DIV_MASK;
	reg |= (div - 1) << MXC_CCM_CCOSR_CKOL_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}

static unsigned long _clk_clko_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	/* clko may output clko2 without divider */
	if (clk->parent == &clko2_clk)
		return parent_rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;
	else if (parent_rate % rate)
		div++;

	if (div > 8)
		div = 8;
	return parent_rate / div;
}

static int _clk_clko2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 sel, reg;

	if (parent == &mmdc_ch0_axi_clk[0])
		sel = 0;
	else if (parent == &mmdc_ch1_axi_clk[0])
		sel = 1;
	else if (parent == &usdhc4_clk)
		sel = 2;
	else if (parent == &usdhc1_clk)
		sel = 3;
	else if (parent == &gpu2d_axi_clk)
		sel = 4;
	else if (parent == &ecspi_clk[0])
		sel = 6;
	else if (parent == &gpu3d_axi_clk)
		sel = 7;
	else if (parent == &usdhc3_clk)
		sel = 8;
	else if (parent == &pcie_clk[0])
		sel = 9;
	else if (parent == &ipu1_clk)
		sel = 11;
	else if (parent == &ipu2_clk)
		sel = 12;
	else if (parent == &vdo_axi_clk)
		sel = 13;
	else if (parent == &osc_clk)
		sel = 14;
	else if (parent == &gpu2d_core_clk[0])
		sel = 15;
	else if (parent == &gpu3d_core_clk[0])
		sel = 16;
	else if (parent == &usdhc2_clk)
		sel = 17;
	else if (parent == &ssi1_clk)
		sel = 18;
	else if (parent == &ssi2_clk)
		sel = 19;
	else if (parent == &ssi3_clk)
		sel = 20;
	else if (parent == &gpu3d_shader_clk)
		sel = 21;
	else if (parent == &can_clk_root)
		sel = 23;
	else if (parent == &ldb_di0_clk)
		sel = 24;
	else if (parent == &ldb_di1_clk)
		sel = 25;
	else if (parent == &esai_clk)
		sel = 26;
	else if (parent == &uart_clk[0])
		sel = 28;
	else if (parent == &spdif0_clk[0])
		sel = 29;
	else if (parent == &hsi_tx_clk[0])
		sel = 31;
	else
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MXC_CCM_CCOSR_CKO2_SEL_MASK;
	reg |= sel << MXC_CCM_CCOSR_CKO2_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}

static unsigned long _clk_clko2_get_rate(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CCOSR);
	u32 div = ((reg & MXC_CCM_CCOSR_CKO2_DIV_MASK) >>
			MXC_CCM_CCOSR_CKO2_DIV_OFFSET) + 1;
	return clk_get_rate(clk->parent) / div;
}

static int _clk_clko2_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 parent_rate = clk_get_rate(clk->parent);
	u32 div = parent_rate / rate;

	if (div == 0)
		div++;
	if (((parent_rate / div) != rate) || (div > 8))
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MXC_CCM_CCOSR_CKO2_DIV_MASK;
	reg |= (div - 1) << MXC_CCM_CCOSR_CKO2_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}

static struct clk clko_clk = {
	__INIT_CLK_DEBUG(clko_clk)
	.parent = &pll2_528_bus_main_clk,
	.enable = _clk_enable1,
	.enable_reg = MXC_CCM_CCOSR,
	.enable_shift = MXC_CCM_CCOSR_CKOL_EN_OFFSET,
	.disable = _clk_disable1,
	.set_parent = _clk_clko_set_parent,
	.set_rate = _clk_clko_set_rate,
	.get_rate = _clk_clko_get_rate,
	.round_rate = _clk_clko_round_rate,
};

static struct clk clko2_clk = {
	__INIT_CLK_DEBUG(clko2_clk)
	.parent = &usdhc4_clk,
	.enable = _clk_enable1,
	.enable_reg = MXC_CCM_CCOSR,
	.enable_shift = MXC_CCM_CCOSR_CKO2_EN_OFFSET,
	.disable = _clk_disable1,
	.set_parent = _clk_clko2_set_parent,
	.set_rate = _clk_clko2_set_rate,
	.get_rate = _clk_clko2_get_rate,
	.round_rate = _clk_clko_round_rate,
};

static struct clk perfmon0_clk = {
	__INIT_CLK_DEBUG(perfmon0_clk)
	.parent = &mmdc_ch0_axi_clk[0],
	.enable = _clk_enable1,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG1_OFFSET,
	.disable = _clk_disable1,
};

static struct clk perfmon1_clk = {
	__INIT_CLK_DEBUG(perfmon1_clk)
	.parent = &ipu1_clk,
	.enable = _clk_enable1,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG2_OFFSET,
	.disable = _clk_disable1,
};

static struct clk perfmon2_clk = {
	__INIT_CLK_DEBUG(perfmon2_clk)
	.parent = &mmdc_ch0_axi_clk[0],
	.enable = _clk_enable1,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGRx_CG3_OFFSET,
	.disable = _clk_disable1,
};

static struct clk dummy_clk = {
	.id = 0,
};

#define _REGISTER_CLOCK(d, n, c) \
	{ \
		.dev_id = d, \
		.con_id = n, \
		.clk = &c, \
	}


static struct clk_lookup lookups[] = {
	_REGISTER_CLOCK(NULL, "osc", osc_clk),
	_REGISTER_CLOCK(NULL, "ckih", ckih_clk),
	_REGISTER_CLOCK(NULL, "ckih2", ckih2_clk),
	_REGISTER_CLOCK(NULL, "ckil", ckil_clk),
	_REGISTER_CLOCK(NULL, "pll1_main_clk", pll1_sys_main_clk),
	_REGISTER_CLOCK(NULL, "pll1_sw_clk", pll1_sw_clk),
	_REGISTER_CLOCK(NULL, "pll2", pll2_528_bus_main_clk),
	_REGISTER_CLOCK(NULL, "pll2_pfd_400M", pll2_pfd_400M),
	_REGISTER_CLOCK(NULL, "pll2_pfd_352M", pll2_pfd_352M),
	_REGISTER_CLOCK(NULL, "pll2_pfd_594M", pll2_pfd_594M),
	_REGISTER_CLOCK(NULL, "pll2_200M", pll2_200M),
	_REGISTER_CLOCK(NULL, "pll3_main_clk", pll3_usb_otg_main_clk),
	_REGISTER_CLOCK(NULL, "pll3_pfd_508M", pll3_pfd_508M),
	_REGISTER_CLOCK(NULL, "pll3_pfd_454M", pll3_pfd_454M),
	_REGISTER_CLOCK(NULL, "pll3_pfd_720M", pll3_pfd_720M),
	_REGISTER_CLOCK(NULL, "pll3_pfd_540M", pll3_pfd_540M),
	_REGISTER_CLOCK(NULL, "pll3_sw_clk", pll3_sw_clk),
	_REGISTER_CLOCK(NULL, "pll3_120M", pll3_120M),
	_REGISTER_CLOCK(NULL, "pll3_120M", pll3_80M),
	_REGISTER_CLOCK(NULL, "pll3_120M", pll3_60M),
	_REGISTER_CLOCK(NULL, "pll4", pll4_audio_main_clk),
	_REGISTER_CLOCK(NULL, "pll5", pll5_video_main_clk),
	_REGISTER_CLOCK(NULL, "pll6", pll6_mlb150_main_clk),
	_REGISTER_CLOCK(NULL, "pll3", pll7_usb_host_main_clk),
	_REGISTER_CLOCK(NULL, "pll4", pll8_enet_main_clk),
	_REGISTER_CLOCK(NULL, "cpu_clk", cpu_clk),
	_REGISTER_CLOCK("smp_twd", NULL, twd_clk),
	_REGISTER_CLOCK(NULL, "periph_clk", periph_clk),
	_REGISTER_CLOCK(NULL, "axi_clk", axi_clk),
	_REGISTER_CLOCK(NULL, "mmdc_ch0_axi", mmdc_ch0_axi_clk[0]),
	_REGISTER_CLOCK(NULL, "mmdc_ch1_axi", mmdc_ch1_axi_clk[0]),
	_REGISTER_CLOCK(NULL, "ahb", ahb_clk),
	_REGISTER_CLOCK(NULL, "ipg_clk", ipg_clk),
	_REGISTER_CLOCK(NULL, "ipg_perclk", ipg_perclk),
	_REGISTER_CLOCK(NULL, "spba", spba_clk),
	_REGISTER_CLOCK("imx-sdma", NULL, sdma_clk[0]),
	_REGISTER_CLOCK(NULL, "gpu2d_axi_clk", gpu2d_axi_clk),
	_REGISTER_CLOCK(NULL, "gpu3d_axi_clk", gpu3d_axi_clk),
	_REGISTER_CLOCK(NULL, "pcie_axi_clk", pcie_axi_clk),
	_REGISTER_CLOCK(NULL, "vdo_axi_clk", vdo_axi_clk),
	_REGISTER_CLOCK(NULL, "iim_clk", iim_clk),
	_REGISTER_CLOCK(NULL, "i2c_clk", i2c_clk[0]),
	_REGISTER_CLOCK("imx-i2c.1", NULL, i2c_clk[1]),
	_REGISTER_CLOCK("imx-i2c.2", NULL, i2c_clk[2]),
	_REGISTER_CLOCK(NULL, "vpu_clk", vpu_clk[0]),
	_REGISTER_CLOCK(NULL, "ipu1_clk", ipu1_clk),
	_REGISTER_CLOCK(NULL, "ipu2_clk", ipu2_clk),
	_REGISTER_CLOCK("sdhci-esdhc-imx.0", NULL, usdhc1_clk),
	_REGISTER_CLOCK("sdhci-esdhc-imx.1", NULL, usdhc2_clk),
	_REGISTER_CLOCK("sdhci-esdhc-imx.2", NULL, usdhc3_clk),
	_REGISTER_CLOCK("sdhci-esdhc-imx.3", NULL, usdhc4_clk),
	_REGISTER_CLOCK("imx-ssi.0", NULL, ssi1_clk),
	_REGISTER_CLOCK("imx-ssi.1", NULL, ssi2_clk),
	_REGISTER_CLOCK("imx-ssi.2", NULL, ssi3_clk),
	_REGISTER_CLOCK(NULL, "ipu1_di0_clk", ipu1_di_clk[0]),
	_REGISTER_CLOCK(NULL, "ipu1_di1_clk", ipu1_di_clk[1]),
	_REGISTER_CLOCK(NULL, "ipu2_di0_clk", ipu2_di_clk[0]),
	_REGISTER_CLOCK(NULL, "ipu2_di1_clk", ipu2_di_clk[1]),
	_REGISTER_CLOCK(NULL, "can_root_clk", can_clk_root),
	_REGISTER_CLOCK("imx6q-flexcan.0", NULL, can1_clk[0]),
	_REGISTER_CLOCK("imx6q-flexcan.1", NULL, can2_clk[0]),
	_REGISTER_CLOCK(NULL, "ldb_di0_clk", ldb_di0_clk),
	_REGISTER_CLOCK(NULL, "ldb_di1_clk", ldb_di1_clk),
	_REGISTER_CLOCK("mxc_spdif.0", NULL, spdif0_clk[0]),
	_REGISTER_CLOCK(NULL, "esai_clk", esai_clk),
	_REGISTER_CLOCK("imx6q-ecspi.0", NULL, ecspi_clk[0]),
	_REGISTER_CLOCK("imx6q-ecspi.1", NULL, ecspi_clk[1]),
	_REGISTER_CLOCK("imx6q-ecspi.2", NULL, ecspi_clk[2]),
	_REGISTER_CLOCK("imx6q-ecspi.3", NULL, ecspi_clk[3]),
	_REGISTER_CLOCK(NULL, "emi_slow_clk", emi_slow_clk),
	_REGISTER_CLOCK(NULL, "emi_clk", emi_clk),
	_REGISTER_CLOCK(NULL, "enfc_clk", enfc_clk),
	_REGISTER_CLOCK("imx-uart.0", NULL, uart_clk[0]),
	_REGISTER_CLOCK("imx-uart.1", NULL, uart_clk[0]),
	_REGISTER_CLOCK("imx-uart.2", NULL, uart_clk[0]),
	_REGISTER_CLOCK("imx-uart.3", NULL, uart_clk[0]),
	_REGISTER_CLOCK("imx-uart.4", NULL, uart_clk[0]),
	_REGISTER_CLOCK(NULL, "hsi_tx", hsi_tx_clk[0]),
	_REGISTER_CLOCK(NULL, "caam_clk", caam_clk[0]),
	_REGISTER_CLOCK(NULL, "asrc_clk", asrc_clk[0]),
	_REGISTER_CLOCK(NULL, "asrc_serial_clk", asrc_clk[1]),
	_REGISTER_CLOCK(NULL, "mxs-dma-apbh", apbh_dma_clk),
	_REGISTER_CLOCK(NULL, "openvg_axi_clk", openvg_axi_clk),
	_REGISTER_CLOCK(NULL, "gpu3d_clk", gpu3d_core_clk[0]),
	_REGISTER_CLOCK(NULL, "gpu2d_clk", gpu2d_core_clk[0]),
	_REGISTER_CLOCK(NULL, "gpu3d_shader_clk", gpu3d_shader_clk),
	_REGISTER_CLOCK(NULL, "gpt", gpt_clk[0]),
	_REGISTER_CLOCK("imx6q-gpmi-nand.0", NULL, gpmi_nand_clk[0]),
	_REGISTER_CLOCK(NULL, "gpmi-apb", gpmi_nand_clk[1]),
	_REGISTER_CLOCK(NULL, "bch", gpmi_nand_clk[2]),
	_REGISTER_CLOCK(NULL, "bch-apb", gpmi_nand_clk[3]),
	_REGISTER_CLOCK(NULL, "pl301_mx6qperl-bch", gpmi_nand_clk[4]),
	_REGISTER_CLOCK("mxc_pwm.0", NULL, pwm_clk[0]),
	_REGISTER_CLOCK("mxc_pwm.1", NULL, pwm_clk[1]),
	_REGISTER_CLOCK("mxc_pwm.2", NULL, pwm_clk[2]),
	_REGISTER_CLOCK("mxc_pwm.3", NULL, pwm_clk[3]),
	_REGISTER_CLOCK(NULL, "pcie_clk", pcie_clk[0]),
	_REGISTER_CLOCK(NULL, "pcie_ep_clk", pcie_ep_clk[0]),
	_REGISTER_CLOCK("enet.0", NULL, enet_clk[0]),
	_REGISTER_CLOCK(NULL, "imx_sata_clk", sata_clk[0]),
	_REGISTER_CLOCK(NULL, "usboh3_clk", usboh3_clk[0]),
	_REGISTER_CLOCK(NULL, "usb_phy1_clk", usb_phy1_clk),
	_REGISTER_CLOCK(NULL, "usb_phy3_clk", usb_phy3_clk),
	_REGISTER_CLOCK(NULL, "usb_phy4_clk", usb_phy4_clk),
	_REGISTER_CLOCK("imx2-wdt.0", NULL, dummy_clk),
	_REGISTER_CLOCK("imx2-wdt.1", NULL, dummy_clk),
	_REGISTER_CLOCK(NULL, "hdmi_isfr_clk", hdmi_clk[0]),
	_REGISTER_CLOCK(NULL, "hdmi_iahb_clk", hdmi_clk[1]),
	_REGISTER_CLOCK(NULL, "mipi_pllref_clk", mipi_pllref_clk),
	_REGISTER_CLOCK(NULL, "vdoa", vdoa_clk[0]),
	_REGISTER_CLOCK(NULL, NULL, aips_tz2_clk),
	_REGISTER_CLOCK(NULL, NULL, aips_tz1_clk),
	_REGISTER_CLOCK(NULL, "clko_clk", clko_clk),
	_REGISTER_CLOCK(NULL, "clko2_clk", clko2_clk),
	_REGISTER_CLOCK(NULL, "pxp_axi", ipu2_clk),
	_REGISTER_CLOCK(NULL, "epdc_axi", ipu2_clk),
	_REGISTER_CLOCK(NULL, "epdc_pix", ipu2_di_clk[1]),
	_REGISTER_CLOCK("mxs-perfmon.0", "perfmon", perfmon0_clk),
	_REGISTER_CLOCK("mxs-perfmon.1", "perfmon", perfmon1_clk),
	_REGISTER_CLOCK("mxs-perfmon.2", "perfmon", perfmon2_clk),
	_REGISTER_CLOCK(NULL, "mlb150_clk", mlb150_clk),
	_REGISTER_CLOCK(NULL, "anaclk_1", anaclk_1),
	_REGISTER_CLOCK(NULL, "anaclk_2", anaclk_2),
	_REGISTER_CLOCK(NULL, "apb_pclk", dummy_clk),
};

static struct
clk_lookup imx6dl_i2c4 = _REGISTER_CLOCK("imx-i2c.3", NULL, ecspi_clk[4]);
static struct
clk_lookup imx6q_ecspi5 = _REGISTER_CLOCK("imx6q-ecspi.4", NULL, ecspi_clk[4]);

static void clk_tree_init(void)

{
	unsigned int reg;

	reg = __raw_readl(MMDC_MDMISC_OFFSET);
	if ((reg & MMDC_MDMISC_DDR_TYPE_MASK) ==
		(0x1 << MMDC_MDMISC_DDR_TYPE_OFFSET) ||
		cpu_is_mx6dl()) {
		clk_set_parent(&periph_clk, &pll2_pfd_400M);
		printk(KERN_INFO "Set periph_clk's parent to pll2_pfd_400M!\n");
	}
}


int __init mx6_clocks_init(unsigned long ckil, unsigned long osc,
	unsigned long ckih1, unsigned long ckih2)
{
	__iomem void *base;
	int i, reg;
	u32 parent_rate, rate;
	unsigned long ipg_clk_rate, max_arm_wait_clk;

	external_low_reference = ckil;
	external_high_reference = ckih1;
	ckih2_reference = ckih2;
	oscillator_reference = osc;

	timer_base = ioremap(GPT_BASE_ADDR, SZ_4K);

	apll_base = ioremap(ANATOP_BASE_ADDR, SZ_4K);

	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		clkdev_add(&lookups[i]);
		clk_debug_register(lookups[i].clk);
	}

	/*
	 * imx6q have 5 ecspi and 3 i2c
	 * imx6dl have 4 ecspi and 4 i2c
	 * imx6dl i2c4 use the imx6q ecspi5 clock source
	 */
	if (cpu_is_mx6dl()) {
		clkdev_add(&imx6dl_i2c4);
		clk_debug_register(imx6dl_i2c4.clk);
	} else {
		clkdev_add(&imx6q_ecspi5);
		clk_debug_register(imx6q_ecspi5.clk);
	}

	/* Lower the ipg_perclk frequency to 22MHz.
	  * I2C needs a minimum of 12.8MHz as its source
	  * to acheive 400KHz speed. IPG_PERCLK sources
	  * I2C. 22MHz when divided by the I2C divider gives the
	  * freq closest to 400KHz.
	  */
	clk_set_rate(&ipg_perclk, 22000000);

	/* Timer needs to be initialized first as the
	  * the WAIT routines use GPT counter as
	  * a delay.
	  */
	if (mx6q_revision() == IMX_CHIP_REVISION_1_0) {
		gpt_clk[0].parent = &ipg_perclk;
		gpt_clk[0].get_rate = NULL;
	} else {
		/* Here we use OSC 24M as GPT's clock source, no need to
		enable gpt serial clock*/
		gpt_clk[0].secondary = NULL;
	}

	mxc_timer_init(&gpt_clk[0], timer_base, MXC_INT_GPT);

	clk_tree_init();

#ifdef CONFIG_MX6_VPU_352M
	if (cpu_is_mx6q()) {
		clk_set_rate(&pll2_pfd_400M, 352000000);
		clk_set_parent(&vpu_clk[0], &pll2_pfd_400M);
	}
#endif
	/* keep correct count. */
	clk_enable(&cpu_clk);
	clk_enable(&periph_clk);
	/* Disable un-necessary PFDs & PLLs */
	if (pll2_pfd_400M.usecount == 0 && cpu_is_mx6q())
		pll2_pfd_400M.disable(&pll2_pfd_400M);
	pll2_pfd_352M.disable(&pll2_pfd_352M);
	pll2_pfd_594M.disable(&pll2_pfd_594M);

#if !defined(CONFIG_FEC_1588)
	pll3_pfd_454M.disable(&pll3_pfd_454M);
	pll3_pfd_508M.disable(&pll3_pfd_508M);
	pll3_pfd_540M.disable(&pll3_pfd_540M);
	pll3_pfd_720M.disable(&pll3_pfd_720M);

	pll3_usb_otg_main_clk.disable(&pll3_usb_otg_main_clk);
#endif
	pll4_audio_main_clk.disable(&pll4_audio_main_clk);
	pll5_video_main_clk.disable(&pll5_video_main_clk);
	pll6_mlb150_main_clk.disable(&pll6_mlb150_main_clk);
	pll7_usb_host_main_clk.disable(&pll7_usb_host_main_clk);
	pll8_enet_main_clk.disable(&pll8_enet_main_clk);

	sata_clk[0].disable(&sata_clk[0]);
	pcie_clk[0].disable(&pcie_clk[0]);

	/* Initialize Audio and Video PLLs to valid frequency. */
	clk_set_rate(&pll4_audio_main_clk, 176000000);
	clk_set_rate(&pll5_video_main_clk, 650000000);

	clk_set_parent(&ipu1_di_clk[0], &pll5_video_main_clk);
	clk_set_parent(&ipu1_di_clk[1], &pll5_video_main_clk);
	clk_set_parent(&ipu2_di_clk[0], &pll5_video_main_clk);
	clk_set_parent(&ipu2_di_clk[1], &pll5_video_main_clk);

	clk_set_parent(&emi_clk, &pll2_pfd_400M);
	clk_set_rate(&emi_clk, 200000000);

	/*
	* on mx6dl, 2d core clock sources from 3d shader core clock,
	* but 3d shader clock multiplexer of mx6dl is different from
	* mx6q. For instance the equivalent of pll2_pfd_594M on mc6q
	* is pll2_pfd_528M on mx6dl. Make a note here.
	*/
	clk_set_parent(&gpu3d_shader_clk, &pll2_pfd_594M);
	clk_set_rate(&gpu3d_shader_clk, 594000000);
	if (cpu_is_mx6dl()) {
		/*for mx6dl, change gpu3d core clk parant to 594_PFD */
		clk_set_parent(&gpu3d_core_clk[0], &pll2_pfd_594M);
		clk_set_rate(&gpu3d_core_clk[0], 594000000);

		/*on mx6dl, 2d core clock sources from 3d shader core clock*/
		clk_set_parent(&gpu2d_core_clk[0], &gpu3d_shader_clk);
		/* on mx6dl gpu3d_axi_clk source from mmdc0 directly */
		clk_set_parent(&gpu3d_axi_clk, &mmdc_ch0_axi_clk[0]);
		/* on mx6dl gpu2d_axi_clk source from mmdc0 directly */
		clk_set_parent(&gpu2d_axi_clk, &mmdc_ch0_axi_clk[0]);

		clk_set_rate(&pll3_pfd_540M, 540000000);

		clk_set_parent(&ipu1_clk, &pll3_pfd_540M);
		/* pxp & epdc */
		clk_set_parent(&ipu2_clk, &pll2_pfd_400M);
		clk_set_rate(&ipu2_clk, 200000000);
		clk_set_parent(&axi_clk, &pll3_pfd_540M);
	} else if (cpu_is_mx6q()) {
		clk_set_parent(&gpu3d_core_clk[0], &mmdc_ch0_axi_clk[0]);
		clk_set_rate(&gpu3d_core_clk[0], 528000000);
		clk_set_parent(&ipu2_clk, &mmdc_ch0_axi_clk[0]);
		clk_set_parent(&ipu1_clk, &mmdc_ch0_axi_clk[0]);
		clk_set_parent(&axi_clk, &periph_clk);
	}

	/* Need to keep PLL3_PFD_540M enabled until AXI is sourced from it. */
	clk_enable(&axi_clk);

	if (cpu_is_mx6q() && (mx6q_revision() > IMX_CHIP_REVISION_1_1))
		clk_set_parent(&gpu2d_core_clk[0], &pll3_usb_otg_main_clk);

	clk_set_parent(&ldb_di0_clk, &pll2_pfd_352M);
	clk_set_parent(&ldb_di1_clk, &pll2_pfd_352M);

	/* PCLK camera - J5 */
	clk_set_parent(&clko2_clk, &osc_clk);
	clk_set_rate(&clko2_clk, 2400000);

	clk_set_parent(&clko_clk, &pll4_audio_main_clk);
	/*
	 * FIXME: asrc needs to use asrc_serial(spdif1) clock to do sample
	 * rate convertion and this clock frequency can not be too high, set
	 * it to the minimum value 7.5Mhz to make asrc work properly.
	 */
	clk_set_parent(&asrc_clk[1], &pll3_sw_clk);
	clk_set_rate(&asrc_clk[1], 7500000);

	/* set the GPMI clock to default frequency : 20MHz */
	clk_set_parent(&enfc_clk, &pll2_pfd_400M);
	clk_set_rate(&enfc_clk, enfc_clk.round_rate(&enfc_clk, 20000000));

	mx6_cpu_op_init();
	cpu_op_tbl = get_cpu_op(&cpu_op_nr);

	/* Gate off all possible clocks */
	if (mxc_jtag_enabled) {
		__raw_writel(3 << MXC_CCM_CCGRx_CG11_OFFSET |
			     3 << MXC_CCM_CCGRx_CG2_OFFSET |
			     3 << MXC_CCM_CCGRx_CG1_OFFSET |
			     3 << MXC_CCM_CCGRx_CG0_OFFSET, MXC_CCM_CCGR0);
	} else {
		__raw_writel(1 << MXC_CCM_CCGRx_CG11_OFFSET |
			     3 << MXC_CCM_CCGRx_CG2_OFFSET |
			     3 << MXC_CCM_CCGRx_CG1_OFFSET |
			     3 << MXC_CCM_CCGRx_CG0_OFFSET, MXC_CCM_CCGR0);
	}
	if (mx6q_revision() == IMX_CHIP_REVISION_1_0)
		/* If GPT use ipg_perclk, we need to enable gpt serial clock */
		__raw_writel(3 << MXC_CCM_CCGRx_CG10_OFFSET | 3 << MXC_CCM_CCGRx_CG11_OFFSET, MXC_CCM_CCGR1);
	else
		__raw_writel(3 << MXC_CCM_CCGRx_CG10_OFFSET, MXC_CCM_CCGR1);
	__raw_writel(1 << MXC_CCM_CCGRx_CG12_OFFSET |
		     1 << MXC_CCM_CCGRx_CG11_OFFSET |
		     3 << MXC_CCM_CCGRx_CG10_OFFSET |
		     3 << MXC_CCM_CCGRx_CG9_OFFSET |
		     3 << MXC_CCM_CCGRx_CG8_OFFSET, MXC_CCM_CCGR2);
	__raw_writel(1 << MXC_CCM_CCGRx_CG14_OFFSET |
		     1 << MXC_CCM_CCGRx_CG13_OFFSET |
		     3 << MXC_CCM_CCGRx_CG12_OFFSET |
		     1 << MXC_CCM_CCGRx_CG11_OFFSET |
		     3 << MXC_CCM_CCGRx_CG10_OFFSET, MXC_CCM_CCGR3);
	__raw_writel(3 << MXC_CCM_CCGRx_CG7_OFFSET |
			1 << MXC_CCM_CCGRx_CG6_OFFSET |
			1 << MXC_CCM_CCGRx_CG4_OFFSET, MXC_CCM_CCGR4);
	__raw_writel(1 << MXC_CCM_CCGRx_CG0_OFFSET, MXC_CCM_CCGR5);

	__raw_writel(0, MXC_CCM_CCGR6);

	/* S/PDIF */
	clk_set_parent(&spdif0_clk[0], &pll3_pfd_454M);

	/* MLB150 SYS Clock */
	/*
	 * In Rigel validatioin, the MLB sys_clock isn't using the
	 * right frequency after boot.
	 * In arik, the register CBCMR controls gpu2d clock, not mlb clock,
	 * mlb is sourced from axi clock.
	 * But In rigel, the axi clock is lower than in mx6q, so mlb need to
	 * find a new clock root.
	 * The gpu2d clock is the root of mlb clock in rigel.
	 * Thus we need to add below code in mx6dl.
	 * */
	if (cpu_is_mx6dl())
		clk_set_parent(&mlb150_clk, &pll3_sw_clk);


	if (cpu_is_mx6dl()) {
		/* pxp & epdc */
		clk_set_parent(&ipu2_clk, &pll2_pfd_400M);
		clk_set_rate(&ipu2_clk, 200000000);
		if (epdc_enabled)
			clk_set_parent(&ipu2_di_clk[1], &pll5_video_main_clk);
		else
			clk_set_parent(&ipu2_di_clk[1], &pll3_pfd_540M);
	}

	lp_high_freq = 0;
	lp_med_freq = 0;
	lp_audio_freq = 0;

	/* Get current ARM_PODF value */
	rate = clk_get_rate(&cpu_clk);
	parent_rate = clk_get_rate(&pll1_sw_clk);
	cur_arm_podf = parent_rate / rate;

	/* Calculate the ARM_PODF to be applied when the system
	  * enters WAIT state.
	  * The max ARM clk is decided by the ipg_clk and has to
	  * follow the ratio of ARM_CLK:IPG_CLK of 12:5.
	  */
	ipg_clk_rate = clk_get_rate(&ipg_clk);
	max_arm_wait_clk = (12 * ipg_clk_rate) / 5;
	wait_mode_arm_podf = parent_rate / max_arm_wait_clk;

	/* Turn OFF all unnecessary PHYs. */
	if (cpu_is_mx6q()) {
		/* Turn off SATA PHY. */
		base = ioremap(MX6Q_SATA_BASE_ADDR, SZ_8K);
		reg = __raw_readl(base + PORT_PHY_CTL);
		__raw_writel(reg | PORT_PHY_CTL_PDDQ_LOC, base + PORT_PHY_CTL);
	}

	/* Turn off HDMI PHY. */
	base = ioremap(MX6Q_HDMI_ARB_BASE_ADDR, SZ_128K);
	reg = __raw_readb(base + HDMI_PHY_CONF0);
	__raw_writeb(reg | HDMI_PHY_CONF0_GEN2_PDDQ_MASK, base + HDMI_PHY_CONF0);

	reg = __raw_readb(base + HDMI_MC_PHYRSTZ);
	__raw_writeb(reg | HDMI_MC_PHYRSTZ_DEASSERT, base + HDMI_MC_PHYRSTZ);

	iounmap(base);

	base = ioremap(MX6Q_IOMUXC_BASE_ADDR, SZ_4K);
	/* Close PLL inside SATA PHY. */
	reg = __raw_readl(base + 0x34);
	__raw_writel(reg | (1 << 1), base + 0x34);

	/* Close PCIE PHY. */
	reg = __raw_readl(base + 0x04);
	reg |= (1 << 18);
	__raw_writel(reg, base + 0x04);
	iounmap(base);

	return 0;

}
