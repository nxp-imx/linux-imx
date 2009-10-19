/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup DPM_MX31 Power Management
 * @ingroup MSL_MX31
 */
/*!
 * @file mach-mx3/mxc_pm.c
 *
 * @brief This file provides all the kernel level and user level API
 * definitions for the CRM_MCU and DPLL in mx3.
 *
 * @ingroup DPM_MX31
 */

/*
 * Include Files
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/mxc_pm.h>
#include <asm/cacheflush.h>
#include <asm/irq.h>
#include <mach/common.h>
#include <asm/hardware/cache-l2x0.h>

#include "crm_regs.h"

/* Local defines */
#define FREQ_COMP_TOLERANCE      200	/* tolerance percentage times 100 */
#define MCU_PLL_MAX_FREQ   600000000	/* Maximum frequency MCU PLL clock */
#define MCU_PLL_MIN_FREQ   160000000	/* Minimum frequency MCU PLL clock */
#define NFC_MAX_FREQ        20000000	/* Maximum frequency NFC clock */
#define PRE_DIV_MIN_FREQ    10000000	/* Minimum Frequency after Predivider */

static struct clk *mcu_pll_clk;
static struct clk *cpu_clk;
static struct clk *ahb_clk;
static struct clk *ipg_clk;

/*!
 * Spinlock to protect CRM register accesses
 */
static DEFINE_SPINLOCK(mxc_crm_lock);

/*!
 * This function is called to modify the contents of a CCM_MCU register
 *
 * @param reg_offset the CCM_MCU register that will read
 * @param mask       the mask to be used to clear the bits that are to be modified
 * @param data       the data that should be written to the register
 */
void mxc_ccm_modify_reg(void *reg_offset, unsigned int mask,
			unsigned int data)
{
	unsigned long flags;
	unsigned long reg;

	spin_lock_irqsave(&mxc_crm_lock, flags);
	reg = __raw_readl(reg_offset);
	reg = (reg & (~mask)) | data;
	__raw_writel(reg, reg_offset);
	spin_unlock_irqrestore(&mxc_crm_lock, flags);
}

/*!
 * Compare two frequences using allowable tolerance
 *
 * The MX3 PLL can generate many frequencies. This function
 * compares the generated frequency to the requested frequency
 * and determines it they are within and acceptable tolerance.
 *
 * @param   freq1  desired frequency
 * @param   freq2  generated frequency
 *
 * @return       Returns 0 is frequencies are within talerance
 *               and non-zero is they are not.
 */
static int freq_equal(unsigned long freq1, unsigned long freq2)
{
	if (freq1 > freq2) {
		return (freq1 - freq2) <= (freq1 / FREQ_COMP_TOLERANCE);
	}
	return (freq2 - freq1) <= (freq1 / FREQ_COMP_TOLERANCE);
}

/*!
 * Calculate new MCU clock dividers for the PDR0 regiser.
 *
 * @param   mcu_main_clk PLL output frequency (Hz)
 * @param   arm_freq     desired ARM frequency (Hz)
 * @param   max_freq     desired MAX frequency (Hz)
 * @param   ip_freq      desired IP frequency (Hz)
 * @param   mask         were to return PDR0 mask
 * @param   value        were to return PDR0 value
 *
 * @return             Returns 0 on success or
 *                     Returns non zero if error
 *                       PLL_LESS_ARM_ERR if pll frequency is less than
 *                       desired core frequency
 *                       FREQ_OUT_OF_RANGE if desided frequencies ar not
 *                       possible with the current mcu pll frequency.
 */
static int
cal_pdr0_value(unsigned long mcu_main_clk,
	       long arm_freq,
	       long max_freq,
	       long ip_freq, unsigned long *mask, unsigned long *value)
{
	unsigned long arm_div;	/* ARM core clock divider */
	unsigned long max_div;	/* MAX clock divider */
	unsigned long ipg_div;	/* IPG clock divider */
	unsigned long nfc_div;	/* NFC (Nand Flash Controller) clock divider */
	unsigned long hsp_div;	/* HSP clock divider */

	if (arm_freq > mcu_main_clk) {
		return -PLL_LESS_ARM_ERR;
	}

	arm_div = mcu_main_clk / arm_freq;
	if ((arm_div == 0) || !freq_equal(arm_freq, mcu_main_clk / arm_div)) {
		return FREQ_OUT_OF_RANGE;
	}
	max_div = mcu_main_clk / max_freq;
	if ((max_div == 0) || !freq_equal(max_freq, mcu_main_clk / max_div)) {
		return FREQ_OUT_OF_RANGE;
	}
	hsp_div = max_div;

	ipg_div = max_freq / ip_freq;
	if ((ipg_div == 0) || !freq_equal(ip_freq, max_freq / ipg_div)) {
		return FREQ_OUT_OF_RANGE;
	}

	nfc_div = ((max_freq - 1000000) / NFC_MAX_FREQ) + 1;

	/* All of the divider values have been calculated.
	 * Now change the hardware register. */

	*mask = MXC_CCM_PDR0_HSP_PODF_MASK |
	    MXC_CCM_PDR0_NFC_PODF_MASK |
	    MXC_CCM_PDR0_IPG_PODF_MASK |
	    MXC_CCM_PDR0_MAX_PODF_MASK | MXC_CCM_PDR0_MCU_PODF_MASK;

	*value = ((hsp_div - 1) << MXC_CCM_PDR0_HSP_PODF_OFFSET) |
	    ((nfc_div - 1) << MXC_CCM_PDR0_NFC_PODF_OFFSET) |
	    ((ipg_div - 1) << MXC_CCM_PDR0_IPG_PODF_OFFSET) |
	    ((max_div - 1) << MXC_CCM_PDR0_MAX_PODF_OFFSET) |
	    ((arm_div - 1) << MXC_CCM_PDR0_MCU_PODF_OFFSET);

	return 0;
}

/*!
 * Integer clock scaling
 *
 * Change main arm clock frequencies without changing the PLL.
 * The integer dividers are changed to produce the desired
 * frequencies. The number of valid frequency are limited and
 * are determined by the current MCU PLL frequency
 *
 * @param   arm_freq    desired ARM frequency (Hz)
 * @param   max_freq    desired MAX frequency (Hz)
 * @param   ip_freq     desired IP frequency (Hz)
 *
 * @return             Returns 0 on success or
 *                     Returns non zero if error
 *                       PLL_LESS_ARM_ERR if pll frequency is less than
 *                       desired core frequency
 *                       FREQ_OUT_OF_RANGE if desided frequencies ar not
 *                       possible with the current mcu pll frequency.
 */
int mxc_pm_intscale(long arm_freq, long max_freq, long ip_freq)
{
	unsigned long mcu_main_clk;	/* mcu clock domain main clock */
	unsigned long mask;
	unsigned long value;
	int ret_value;

	printk(KERN_INFO "arm_freq=%ld, max_freq=%ld, ip_freq=%ld\n",
	       arm_freq, max_freq, ip_freq);
	//print_frequencies();  /* debug */

	mcu_main_clk = clk_get_rate(mcu_pll_clk);
	ret_value = cal_pdr0_value(mcu_main_clk, arm_freq, max_freq, ip_freq,
				   &mask, &value);
	if ((arm_freq != clk_round_rate(cpu_clk, arm_freq)) ||
	    (max_freq != clk_round_rate(ahb_clk, max_freq)) ||
	    (ip_freq != clk_round_rate(ipg_clk, ip_freq))) {
		return -EINVAL;
	}

	if ((max_freq != clk_get_rate(ahb_clk)) ||
	    (ip_freq != clk_get_rate(ipg_clk))) {
		return -EINVAL;
	}

	if (arm_freq != clk_get_rate(cpu_clk)) {
		ret_value = clk_set_rate(cpu_clk, arm_freq);
	}
	return ret_value;
}

/*!
 * PLL clock scaling
 *
 * Change MCU PLL frequency and adjust derived clocks. Integer
 * dividers are used generate the derived clocks so changed to produce
 * the desired the valid frequencies are limited by the desired ARM
 * frequency.
 *
 * The clock source for the MCU is set to the MCU PLL.
 *
 * @param   arm_freq    desired ARM frequency (Hz)
 * @param   max_freq    desired MAX frequency (Hz)
 * @param   ip_freq     desired IP frequency (Hz)
 *
 * @return             Returns 0 on success or
 *                     Returns non zero if error
 *                       PLL_LESS_ARM_ERR if pll frequency is less than
 *                       desired core frequency
 *                       FREQ_OUT_OF_RANGE if desided frequencies ar not
 *                       possible with the current mcu pll frequency.
 */
int mxc_pm_pllscale(long arm_freq, long max_freq, long ip_freq)
{
	signed long pll_freq = 0;	/* target pll frequency */
	unsigned long old_pll;
	unsigned long mask;
	unsigned long value;
	int ret_value;

	printk(KERN_INFO "arm_freq=%ld, max_freq=%ld, ip_freq=%ld\n",
	       arm_freq, max_freq, ip_freq);
	//print_frequencies();

	do {
		pll_freq += arm_freq;
		if ((pll_freq > MCU_PLL_MAX_FREQ) || (pll_freq / 8 > arm_freq)) {
			return FREQ_OUT_OF_RANGE;
		}
		if (pll_freq < MCU_PLL_MIN_FREQ) {
			ret_value = 111;
		} else {
			ret_value =
			    cal_pdr0_value(pll_freq, arm_freq, max_freq,
					   ip_freq, &mask, &value);
		}
	} while (ret_value != 0);

	old_pll = clk_get_rate(mcu_pll_clk);
	if (pll_freq > old_pll) {
		/* if pll freq is increasing then change dividers first */
		mxc_ccm_modify_reg(MXC_CCM_PDR0, mask, value);
		ret_value = clk_set_rate(mcu_pll_clk, pll_freq);
	} else {
		/* if pll freq is decreasing then change pll first */
		ret_value = clk_set_rate(mcu_pll_clk, pll_freq);
		mxc_ccm_modify_reg(MXC_CCM_PDR0, mask, value);
	}
	//print_frequencies();
	return ret_value;
}

/*!
 * Implementing steps required to transition to low-power modes
 *
 * @param   mode    The desired low-power mode. Possible values are,
 *                  WAIT_MODE, DOZE_MODE, STOP_MODE or DSM_MODE
 *
 */
void mxc_pm_lowpower(int mode)
{
	unsigned int lpm;
	int enable_flag;
	unsigned long reg;

	local_irq_disable();
	enable_flag = 0;

	switch (mode) {
	case STOP_MODE:
		/* State Retention mode */
		lpm = 2;
		/* Disable timer interrupt */
		disable_irq(MXC_INT_GPT);
		enable_flag = 1;

		/* Enable Well Bias and set VSTBY
		 * VSTBY pin will be asserted during SR mode. This asks the
		 * PM IC to set the core voltage to the standby voltage
		 * Must clear the MXC_CCM_CCMR_SBYCS bit as well  */
		mxc_ccm_modify_reg(MXC_CCM_CCMR,
				   MXC_CCM_CCMR_WBEN | MXC_CCM_CCMR_VSTBY |
				   MXC_CCM_CCMR_SBYCS,
				   MXC_CCM_CCMR_WBEN | MXC_CCM_CCMR_VSTBY |
				   MXC_CCM_CCMR_SBYCS);

		mxc_ccm_modify_reg(MXC_CCM_CCMR,
				   MXC_CCM_CCMR_LPM_MASK,
				   lpm << MXC_CCM_CCMR_LPM_OFFSET);
		cpu_do_idle();
		break;

	case DSM_MODE:
		/* Deep Sleep Mode */
		lpm = 3;
		/* Disable timer interrupt */
		disable_irq(MXC_INT_GPT);
		enable_flag = 1;
		/* Enabled Well Bias
		 * SBYCS = 0, MCU clock source is disabled*/
		mxc_ccm_modify_reg(MXC_CCM_CCMR,
				   MXC_CCM_CCMR_WBEN | MXC_CCM_CCMR_VSTBY |
				   MXC_CCM_CCMR_SBYCS | MXC_CCM_CCMR_LPM_MASK,
				   MXC_CCM_CCMR_WBEN | MXC_CCM_CCMR_VSTBY |
				   MXC_CCM_CCMR_SBYCS |
				   (lpm << MXC_CCM_CCMR_LPM_OFFSET));

		/* wake up by keypad */
		reg = __raw_readl(MXC_CCM_WIMR);
		reg &= ~(1 << 18);
		__raw_writel(reg, MXC_CCM_WIMR);

		flush_cache_all();
		l2x0_disable();

		mxc_pm_arch_entry(IO_ADDRESS(MX31_NFC_BASE_ADDR), 2048);
		printk(KERN_INFO "Resume from DSM\n");

		l2x0_enable();
		mxc_init_irq();

		break;
	default:
	case WAIT_MODE:
		/* Wait is the default mode used when idle. */
		reg = __raw_readl(MXC_CCM_CCMR);
		reg &= ~MXC_CCM_CCMR_LPM_MASK;
		__raw_writel(reg, MXC_CCM_CCMR);
		break;
	}

	if (enable_flag) {
		/* Enable timer interrupt */
		enable_irq(MXC_INT_GPT);
	}
	local_irq_enable();
}

#ifdef CONFIG_MXC_DVFS
/*!
 * Changes MCU frequencies using dvfs.
 *
 * @param       armfreq       desired ARM frequency in Hz
 * @param       ahbfreq       desired AHB frequency in Hz
 * @param       ipfreq        desired IP frequency in Hz
 *
 * @return             Returns 0 on success, non-zero on error
 */
int mxc_pm_dvfs(unsigned long armfreq, long ahbfreq, long ipfreq)
{
	int ret_value;
	int i;

	if (ahbfreq != 133000000) {
		return FREQ_OUT_OF_RANGE;
	}
	if (ipfreq != 66500000) {
		return FREQ_OUT_OF_RANGE;
	}
	ret_value = FREQ_OUT_OF_RANGE;
	for (i = 0; i < dvfs_states_tbl->num_of_states; i++) {
		if (dvfs_states_tbl->freqs[i] == armfreq) {
			ret_value = dvfs_set_state(i);
			break;
		}
	}

	return ret_value;
}
#endif				/* CONFIG_MXC_DVFS */

/*!
 * This function is used to load the module.
 *
 * @return   Returns an Integer on success
 */
static int __init mxc_pm_init_module(void)
{
	printk(KERN_INFO "Low-Level PM Driver module loaded\n");

	mcu_pll_clk = clk_get(NULL, "mcu_pll");
	cpu_clk = clk_get(NULL, "cpu_clk");
	ahb_clk = clk_get(NULL, "ahb_clk");
	ipg_clk = clk_get(NULL, "ipg_clk");
	return 0;
}

/*!
 * This function is used to unload the module
 */
static void __exit mxc_pm_cleanup_module(void)
{
	clk_put(mcu_pll_clk);
	clk_put(cpu_clk);
	clk_put(ahb_clk);
	clk_put(ipg_clk);
	printk(KERN_INFO "Low-Level PM Driver module Unloaded\n");
}

module_init(mxc_pm_init_module);
module_exit(mxc_pm_cleanup_module);

EXPORT_SYMBOL(mxc_pm_intscale);
EXPORT_SYMBOL(mxc_pm_pllscale);
EXPORT_SYMBOL(mxc_pm_lowpower);
#ifdef CONFIG_MXC_DVFS
EXPORT_SYMBOL(mxc_pm_dvfs);
#endif

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MX3 Low-level Power Management Driver");
MODULE_LICENSE("GPL");
