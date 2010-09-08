/*
 * linux/arch/arm/mach-mx3/pm.c
 *
 * MX3 Power Management Routines
 *
 * Original code for the SA11x0:
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 *
 * Modified for the PXA250 by Nicolas Pitre:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * Modified for the OMAP1510 by David Singleton:
 * Copyright (c) 2002 Monta Vista Software, Inc.
 *
 * Cleanup 2004 for OMAP1510/1610 by Dirk Behme <dirk.behme@de.bosch.com>
 *
 * Modified for the MX31
 * Copyright 2005-2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <mach/hardware.h>
#include "crm_regs.h"

extern void mxc_pm_arch_entry(void *entry, u32 size);
extern void mxc_init_irq(void __iomem *irqbase);

int suspend_ops_started;
int is_suspend_ops_started(void)
{
	return suspend_ops_started;
}

static int mx31_suspend_enter(suspend_state_t state)
{
	unsigned long reg;

	/* Enable Well Bias and set VSTBY
	* VSTBY pin will be asserted during SR mode. This asks the
	* PM IC to set the core voltage to the standby voltage
	* Must clear the MXC_CCM_CCMR_SBYCS bit as well??  */
	reg = __raw_readl(MXC_CCM_CCMR);
	reg &= ~MXC_CCM_CCMR_LPM_MASK;
	reg |= MXC_CCM_CCMR_WBEN | MXC_CCM_CCMR_VSTBY | MXC_CCM_CCMR_SBYCS;

	switch (state) {
	case PM_SUSPEND_MEM:
		/* State Retention mode */
		reg |= 2 << MXC_CCM_CCMR_LPM_OFFSET;
		__raw_writel(reg, MXC_CCM_CCMR);

		/* Executing CP15 (Wait-for-Interrupt) Instruction */
		cpu_do_idle();
		break;
	case PM_SUSPEND_STANDBY:
		/* Deep Sleep Mode */
		reg |= 3 << MXC_CCM_CCMR_LPM_OFFSET;
		__raw_writel(reg, MXC_CCM_CCMR);

		/* wake up by keypad */
		reg = __raw_readl(MXC_CCM_WIMR);
		reg &= ~(1 << 18);
		__raw_writel(reg, MXC_CCM_WIMR);

		flush_cache_all();
		l2x0_disable();

		mxc_pm_arch_entry(MX31_IO_ADDRESS(MX31_NFC_BASE_ADDR), 2048);
		printk(KERN_INFO "Resume from DSM\n");

		l2x0_enable();
		mxc_init_irq(MX31_IO_ADDRESS(MX31_AVIC_BASE_ADDR));

		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mx35_suspend_enter(suspend_state_t state)
{
	unsigned long reg;

	reg = __raw_readl(MXC_CCM_CCMR);
	reg &= ~MXC_CCM_CCMR_LPM_MASK;

	switch (state) {
	case PM_SUSPEND_MEM:
	case PM_SUSPEND_STANDBY:
		/* Enabled Well Bias */
		reg |= MXC_CCM_CCMR_WBEN | MXC_CCM_CCMR_VSTBY;
		/* program LPM bit */
		reg |= 3 << MXC_CCM_CCMR_LPM_OFFSET;
		/* program Interrupt holdoff bit */
		reg |= MX35_CCM_CCMR_WFI;
		/* TBD: PMIC has put the voltage back to
		 * Normal if the voltage ready
		 */
		/* counter finished */
		reg |= MX35_CCM_CCMR_STBY_EXIT_SRC;
		break;
	default:
		return -EINVAL;
	}

	__raw_writel(reg, MXC_CCM_CCMR);

	/* Executing CP15 (Wait-for-Interrupt) Instruction */
	cpu_do_idle();
	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx3_suspend_prepare(void)
{
	suspend_ops_started = 1;
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mx3_suspend_finish(void)
{
	suspend_ops_started = 0;
	return;
}

static int mx3_pm_valid(suspend_state_t state)
{
	return (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX);
}

struct platform_suspend_ops mx3_suspend_ops = {
	.valid = mx3_pm_valid,
	.prepare = mx3_suspend_prepare,
	.enter = mx31_suspend_enter,
	.finish = mx3_suspend_finish,
};

static int __init mx3_pm_init(void)
{
	printk(KERN_INFO "Power Management for Freescale MX3x\n");
	if (cpu_is_mx35())
		mx3_suspend_ops.enter = mx35_suspend_enter;
	suspend_set_ops(&mx3_suspend_ops);

	return 0;
}

late_initcall(mx3_pm_init);
