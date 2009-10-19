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

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <asm/proc-fns.h>
#include <asm/system.h>
#include <mach/clock.h>
#include <mach/hardware.h>
#include "crm_regs.h"

/*!
 * @defgroup MSL_MX35 i.MX35 Machine Specific Layer (MSL)
 */

/*!
 * @file mach-mx35/system.c
 * @brief This file contains idle and reset functions.
 *
 * @ingroup MSL_MX35
 */

/*!
* MX35 low-power mode
*/
enum mx35_low_pwr_mode {
	MX35_RUN_MODE,
	MX35_WAIT_MODE,
	MX35_DOZE_MODE,
	MX35_STOP_MODE
};

extern int mxc_jtag_enabled;

/*!
 * This function is used to set cpu low power mode before WFI instruction
 *
 * @param  mode         indicates different kinds of power modes
 */
void mxc_cpu_lp_set(enum mxc_cpu_pwr_mode mode)
{
	unsigned int lpm;
	unsigned long reg;

	/*read CCMR value */
	reg = __raw_readl(MXC_CCM_CCMR);

	switch (mode) {
	case WAIT_UNCLOCKED_POWER_OFF:
		lpm = MX35_DOZE_MODE;
		break;

	case STOP_POWER_ON:
	case STOP_POWER_OFF:
		lpm = MX35_STOP_MODE;
		/* Enabled Well Bias */
		reg |= MXC_CCM_CCMR_WBEN;
		if (!board_is_rev(BOARD_REV_1))
			reg |= MXC_CCM_CCMR_VSTBY;
		break;

	case WAIT_CLOCKED:
	case WAIT_UNCLOCKED:
	default:
		/* Wait is the default mode used when idle. */
		lpm = MX35_WAIT_MODE;
		break;
	}

	/* program LPM bit */
	reg = (reg & (~MXC_CCM_CCMR_LPM_MASK)) | lpm << MXC_CCM_CCMR_LPM_OFFSET;
	/* program Interrupt holdoff bit */
	reg = reg | MXC_CCM_CCMR_WFI;
	/* TBD: PMIC has put the voltage back to Normal if the voltage ready */
	/* counter finished */
	reg = reg | MXC_CCM_CCMR_STBY_EXIT_SRC;

	__raw_writel(reg, MXC_CCM_CCMR);
}

EXPORT_SYMBOL(mxc_cpu_lp_set);

/*!
 * This function puts the CPU into idle mode. It is called by default_idle()
 * in process.c file.
 */
void arch_idle(void)
{
	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks.
	 */
	if (!mxc_jtag_enabled) {
#ifdef CONFIG_MX35_DOZE_DURING_IDLE
		/*set as Doze mode */
		mxc_cpu_lp_set(WAIT_UNCLOCKED_POWER_OFF);
#else
		/* set as Wait mode */
		mxc_cpu_lp_set(WAIT_UNCLOCKED);
#endif
		cpu_do_idle();
	}
}

/*
 * This function resets the system. It is called by machine_restart().
 *
 * @param  mode         indicates different kinds of resets
 */
void arch_reset(char mode)
{
	/* Assert SRS signal */
	mxc_wd_reset();
}
