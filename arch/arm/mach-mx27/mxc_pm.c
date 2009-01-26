/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup DPM_MX27 Power Management
 * @ingroup MSL_MX27
 */
/*!
 * @file mach-mx27/mxc_pm.c
 *
 * @brief This file contains the implementation of the Low-level power
 * management driver. It modifies the registers of the PLL and clock module
 * of the i.MX27.
 *
 * @ingroup DPM_MX27
 */

/*
 * Include Files
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <mach/mxc_pm.h>
#include <mach/mxc.h>
#include <mach/system.h>
#include <asm/irq.h>
#include "crm_regs.h"

/* Local defines */
#define MAX_ARM_FREQ        400000000
#define MAX_AHB_FREQ        133000000
#define MAX_IPG_FREQ        66500000
#define FREQ_COMP_TOLERANCE      100	/* tolerance percentage times 100 */
#define MX27_LLPM_DEBUG	    0

/*
 * Global variables
 */
#if 0
/*!
 * These variables hold the various clock values when the module is loaded.
 * This is needed because these clocks are derived from MPLL and when MPLL
 * output changes, these clocks need to be adjusted.
 */
static u32 perclk1, perclk2, perclk3, perclk4, nfcclk, cpuclk;

/*!
 * Compare two frequences using allowable tolerance
 *
 * The MX27 PLL can generate many frequencies. This function
 * compares the generated frequency to the requested frequency
 * and determines it they are within and acceptable tolerance.
 *
 * @param   freq1  desired frequency
 * @param   freq2  generated frequency
 *
 * @return       Returns 0 is frequencies are within talerance
 *               and non-zero is they are not.
 */
static s32 freq_equal(u32 freq1, u32 freq2)
{
	if (freq1 > freq2) {
		return (freq1 - freq2) <= (freq1 / FREQ_COMP_TOLERANCE);
	}
	return (freq2 - freq1) <= (freq1 / FREQ_COMP_TOLERANCE);
}

/*!
 * Select the PLL frequency based on the desired ARM frequency.
 *
 * The MPLL will be configured to output three frequencies, 400/333/266 MHz.
 *
 * @param       armfreq         Desired ARM frequency
 *
 * @return      Returns one of the selected PLL frequency (400/333/266 MHz).
 *              Returns -1 on error.
 *
 */
static s32 select_freq_pll(u32 armfreq)
{
	u32 div;

	div = 266000000 / armfreq;
	if ((div == 0) || (!freq_equal(armfreq, 266000000 / div))) {
		div = 400000000 / armfreq;
		if ((div == 0) || (!freq_equal(armfreq, 400000000 / div))) {
			return -1;
		}

		return 400000000;
	}

	return 266000000;
}

/*!
 * Check whether the desired ARM and AHB frequencies are valid.
 *
 * @param       armfreq         Desired ARM frequency
 * @param       ahbfreq         Desired AHB frequency
 *
 * @return      Returns 0 on success
 *              Return -1 on error
 */
static s32 mx27_pm_check_parameters(u32 armfreq, u32 ahbfreq)
{
	u32 ahbdiv;

	/* No idea about minimum frequencies.. just a guess! */
	if ((armfreq < 1000000) || (ahbfreq < 1000000)) {
		printk("arm or ahb frequencies are less\n");
		return -1;
	}

	if ((armfreq > MAX_ARM_FREQ) || (ahbfreq > MAX_AHB_FREQ)) {
		printk("arm or ahb freq. are too much\n");
		return -1;
	}

	/* AHB divider value is restricted to less than 8 */
	ahbdiv = armfreq / ahbfreq;
	if ((ahbdiv == 0) || (ahbdiv > 8)) {
		printk("Invalid ahb frequency\n");
		return -1;
	}

	return 0;
}

/*!
 * Integer clock scaling
 *
 * Change the main ARM clock frequencies without changing the MPLL.
 * The integer dividers (PRESC and BCLKDIV) are changed to obtain the
 * desired frequency. Since NFC clock is derived from ARM frequency,
 * NFCDIV is also adjusted.
 *
 * @param       arm_freq        Desired ARM frequency
 * @param       ahb_freq        Desired AHB frequency
 * @param       pll_freq        Current PLL frequency
 *
 * @return      Returns 0
 */
static s32 mx27_pm_intscale(u32 arm_freq, u32 ahb_freq, s32 pll_freq)
{
	u32 pre_div, bclk_div, nfc_div;

	/* Calculate ARM divider */
	pre_div = pll_freq / arm_freq;
	if (pre_div == 0)
		pre_div = 1;

	/* Calculate AHB divider */
	bclk_div = arm_freq / ahb_freq;
	if (bclk_div == 0)
		bclk_div = 1;

	if ((arm_freq / bclk_div) > ahb_freq)
		bclk_div++;

	/* NFC clock is dependent on ARM clock */
	nfc_div = arm_freq / nfcclk;
	if ((arm_freq / nfc_div) > nfcclk)
		nfc_div++;

	/* Adjust NFC divider */
	mxc_set_clocks_div(NFC_CLK, nfc_div);

#if MX27_LLPM_DEBUG
	printk("DIVIDERS: PreDiv = %d BCLKDIV = %d \n", pre_div, bclk_div);
	printk("Integer scaling\n");
	printk("PLL = %d : ARM = %d: AHB = %d\n", pll_freq, arm_freq, ahb_freq);
#endif

	/*
	 * This part is tricky. What to adjust first (PRESC or BCLKDIV)?
	 * After trial and error, if current ARM frequency is greater than
	 * desired ARM frequency, then adjust PRESC first, else if current
	 * ARM frequency is less than desired ARM frequency, then adjust
	 * BCLKDIV first.
	 */
	if (cpuclk > arm_freq) {
		mxc_set_clocks_div(CPU_CLK, pre_div);
		mxc_set_clocks_div(AHB_CLK, bclk_div);
	} else {
		mxc_set_clocks_div(AHB_CLK, bclk_div);
		mxc_set_clocks_div(CPU_CLK, pre_div);
	}

	cpuclk = arm_freq;
	mdelay(50);
	return 0;
}

/*!
 * Set dividers for various peripheral clocks.
 *
 * PERCLK1, PERCLK2, PERCLK3 and PERCLK4 are adjusted based on the MPLL
 * output frequency.
 *
 * @param       pll_freq        Desired MPLL output frequency
 */
static void mx27_set_dividers(u32 pll_freq)
{
	s32 perdiv1, perdiv2, perdiv3, perdiv4;

	perdiv1 = pll_freq / perclk1;
	if ((pll_freq / perdiv1) > perclk1)
		perdiv1++;

	perdiv2 = pll_freq / perclk2;
	if ((pll_freq / perdiv2) > perclk2)
		perdiv2++;

	perdiv3 = pll_freq / perclk3;
	if ((pll_freq / perdiv3) > perclk3)
		perdiv3++;

	perdiv4 = pll_freq / perclk4;
	if ((pll_freq / perdiv4) > perclk4)
		perdiv4++;

	mxc_set_clocks_div(PERCLK1, perdiv1);
	mxc_set_clocks_div(PERCLK2, perdiv2);
	mxc_set_clocks_div(PERCLK3, perdiv3);
	mxc_set_clocks_div(PERCLK4, perdiv4);
}

/*!
 * Change MPLL output frequency and adjust derived clocks to produce the
 * desired frequencies.
 *
 * @param       arm_freq        Desired ARM frequency
 * @param       ahb_freq        Desired AHB frequency
 * @param       org_pll         Current PLL frequency
 *
 * @return      Returns 0 on success
 *              Returns -1 on error
 */
static s32 mx27_pm_pllscale(u32 arm_freq, u32 ahb_freq, s32 org_pll)
{
	u32 mfi, mfn, mfd, pd = 1, cscr;
	s32 pll_freq;

	/* Obtain the PLL frequency for the desired ARM frequency */
	pll_freq = select_freq_pll(arm_freq);
	if (pll_freq == -1) {
		return -1;
	}

	/* The MPCTL0 register values are programmed based on the oscillator */
	cscr = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
	if ((cscr & CCM_CSCR_OSC26M) == 0) {
		/* MPCTL0 register values are programmed for 400/266 MHz */
		switch (pll_freq) {
		case 400000000:
			mfi = 7;
			mfn = 9;
			mfd = 12;
			pd = 0;
			break;

		case 266000000:
			mfi = 10;
			mfn = 6;
			mfd = 25;
			break;

		default:
			return -1;
		}
	} else {
		/* MPCTL0 register values are programmed for 400/266 MHz */
		switch (pll_freq) {
		case 400000000:
			mfi = 12;
			mfn = 2;
			mfd = 3;
			break;

		case 266000000:
			mfi = 8;
			mfn = 10;
			mfd = 31;
			break;

		default:
			return -1;
		}
	}

#if MX27_LLPM_DEBUG
	printk("PLL scaling\n");
	printk("PLL = %d : ARM = %d: AHB = %d\n", pll_freq, arm_freq, ahb_freq);
#endif

	/* Adjust the peripheral clock dividers for new PLL frequency */
	mx27_set_dividers(pll_freq);

	if (pll_freq > org_pll) {
		/* Set the dividers first */
		mx27_pm_intscale(arm_freq, ahb_freq, pll_freq);

		/* Set the PLL */
		mxc_pll_set(MCUPLL, mfi, pd, mfd, mfn);
		mdelay(50);
	} else {
		/* Set the PLL first */
		mxc_pll_set(MCUPLL, mfi, pd, mfd, mfn);
		mdelay(50);

		/* Set the dividers later */
		mx27_pm_intscale(arm_freq, ahb_freq, pll_freq);
	}

	return 0;
}
#endif
/*!
 * Implement steps required to transition to low-power modes.
 *
 * @param       mode    The desired low-power mode. Possible values are,
 *                      DOZE_MODE
 *                      WAIT_MODE
 *                      STOP_MODE
 *                      DSM_MODE
 */
void mxc_pm_lowpower(s32 mode)
{
	u32 cscr;

	local_irq_disable();

	/* WAIT and DOZE execute WFI only */
	switch (mode) {
	case STOP_MODE:
	case DSM_MODE:
		/* Clear MPEN and SPEN to disable MPLL/SPLL */
		cscr = __raw_readl(CCM_CSCR);
		cscr &= 0xFFFFFFFC;
		__raw_writel(cscr, CCM_CSCR);
		break;
	}

	/* Executes WFI */
	arch_idle();

	local_irq_enable();
}

#if 0
/*!
 * Called to change the core frequency. This function internally decides
 * whether to do integer scaling or pll scaling.
 *
 * @param       arm_freq        Desired ARM frequency
 * @param       ahb_freq        Desired AHB frequency
 * @param       ipg_freq        Desired IP frequency, constant AHB / 2 always.
 *
 * @return      Returns 0 on success
 *              Returns -1 on error
 */
int mxc_pm_dvfs(unsigned long arm_freq, long ahb_freq, long ipg_freq)
{
	u32 divider;
	s32 pll_freq, ret;
	unsigned long flags;

	if (mx27_pm_check_parameters(arm_freq, ahb_freq) != 0) {
		return -1;
	}

	local_irq_save(flags);

	/* Get the current PLL frequency */
	pll_freq = mxc_pll_clock(MCUPLL);

#if MX27_LLPM_DEBUG
	printk("MCU PLL frequency is %d\n", pll_freq);
#endif

	/* Decide whether to do integer scaling or pll scaling */
	if (arm_freq > pll_freq) {
		/* Do PLL scaling */
		ret = mx27_pm_pllscale(arm_freq, ahb_freq, pll_freq);
	} else {
		/* We need integer divider values */
		divider = pll_freq / arm_freq;
		if (!freq_equal(arm_freq, pll_freq / divider)) {
			/* Do PLL scaling */
			ret = mx27_pm_pllscale(arm_freq, ahb_freq, pll_freq);
		} else {
			/* Do integer scaling */
			ret = mx27_pm_intscale(arm_freq, ahb_freq, pll_freq);
		}
	}

	local_irq_restore(flags);
	return ret;
}
#endif
/*
 * This API is not supported on i.MX27
 */
int mxc_pm_intscale(long armfreq, long ahbfreq, long ipfreq)
{
	return -MXC_PM_API_NOT_SUPPORTED;
}

/*
 * This API is not supported on i.MX27
 */
int mxc_pm_pllscale(long armfreq, long ahbfreq, long ipfreq)
{
	return -MXC_PM_API_NOT_SUPPORTED;
}

/*!
 * This function is used to load the module.
 *
 * @return   Returns an Integer on success
 */
static int __init mxc_pm_init_module(void)
{
	printk(KERN_INFO "MX27: Power management module initialized\n");
	return 0;
}

/*!
 * This function is used to unload the module
 */
static void __exit mxc_pm_cleanup_module(void)
{
	printk(KERN_INFO "MX27: Power management module exit\n");
}

module_init(mxc_pm_init_module);
module_exit(mxc_pm_cleanup_module);

EXPORT_SYMBOL(mxc_pm_lowpower);
//EXPORT_SYMBOL(mxc_pm_dvfs);
EXPORT_SYMBOL(mxc_pm_pllscale);
EXPORT_SYMBOL(mxc_pm_intscale);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("i.MX27 low level PM driver");
MODULE_LICENSE("GPL");
