/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <mach/hardware.h>
#include <asm/proc-fns.h>
#include <asm/system.h>
#include <mach/clock.h>
#include "crm_regs.h"

/*!
 * @defgroup MSL_MX25 i.MX25 Machine Specific Layer (MSL)
 */

/*!
 * @file mach-mx25/system.c
 * @brief This file contains idle and reset functions.
 *
 * @ingroup MSL_MX25
 */

/*!
 * MX25 low-power mode
 */
enum mx25_low_pwr_mode {
	MX25_RUN_MODE,
	MX25_WAIT_MODE,
	MX25_DOZE_MODE,
	MX25_STOP_MODE
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
	unsigned int pmcr1, pmcr2, lpimr;
	unsigned int cgcr0, cgcr1, cgcr2;
	struct irq_desc *desc;
	int i;

	/*read CCTL value */
	reg = __raw_readl(MXC_CCM_CCTL);

	switch (mode) {
	case WAIT_UNCLOCKED_POWER_OFF:
		lpm = MX25_DOZE_MODE;
		break;

	case STOP_POWER_ON:
	case STOP_POWER_OFF:
		lpm = MX25_STOP_MODE;
		/* The clock of LCDC/SLCDC, SDMA, RTIC, RNGC, MAX, CAN
		   and EMI needs to be gated on when entering Stop mode.
		 */
		cgcr0 = __raw_readl(MXC_CCM_CGCR0);
		cgcr1 = __raw_readl(MXC_CCM_CGCR1);
		cgcr2 = __raw_readl(MXC_CCM_CGCR2);
		__raw_writel(cgcr0 | MXC_CCM_CGCR0_STOP_MODE_MASK,
			     MXC_CCM_CGCR0);
		__raw_writel(cgcr1 | MXC_CCM_CGCR1_STOP_MODE_MASK,
			     MXC_CCM_CGCR1);
		__raw_writel(cgcr2 | MXC_CCM_CGCR2_STOP_MODE_MASK,
			     MXC_CCM_CGCR2);
		/* The interrupts which are not wake-up sources need
		   be mask when entering Stop mode.
		 */
		lpimr = MXC_CCM_LPIMR0_MASK;
		for (i = 0; i < 32; i++) {
			desc = irq_desc + i;
			if ((desc->status & IRQ_WAKEUP) != 0)
				lpimr &= ~(1 << i);
		}
		__raw_writel(lpimr, MXC_CCM_LPIMR0);
		lpimr = MXC_CCM_LPIMR1_MASK;
		for (i = 32; i < 64; i++) {
			desc = irq_desc + i;
			if ((desc->status & IRQ_WAKEUP) != 0)
				lpimr &= ~(1 << (i - 32));
		}
		__raw_writel(lpimr, MXC_CCM_LPIMR1);

		if (mode == STOP_POWER_OFF) {
			pmcr2 = __raw_readl(MXC_CCM_PMCR2);
			pmcr2 |= (MXC_CCM_PMCR2_OSC24M_DOWN |
				  MXC_CCM_PMCR2_VSTBY);
			__raw_writel(pmcr2, MXC_CCM_PMCR2);
			pmcr1 = __raw_readl(MXC_CCM_PMCR1);
			pmcr1 &= ~(MXC_CCM_PMCR1_WBCN_MASK |
				   MXC_CCM_PMCR1_CSPAEM_MASK |
				   MXC_CCM_PMCR1_CSPA_MASK);
			pmcr1 |= MXC_CCM_PMCR1_AWB_DEFAULT;
			__raw_writel(pmcr1, MXC_CCM_PMCR1);
		}
		break;

	case WAIT_CLOCKED:
	case WAIT_UNCLOCKED:
	default:
		/* Wait is the default mode used when idle. */
		lpm = MX25_WAIT_MODE;
		break;
	}

	/* program LP CTL bit */
	reg = ((reg & (~MXC_CCM_CCTL_LP_CTL_MASK)) |
	       lpm << MXC_CCM_CCTL_LP_CTL_OFFSET);

	__raw_writel(reg, MXC_CCM_CCTL);
}

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
		/* set as Wait mode */
		mxc_cpu_lp_set(WAIT_UNCLOCKED);
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
