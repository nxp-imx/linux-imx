/*
 *  Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/suspend.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include "crm_regs.h"

/*!
 * @defgroup MSL_MX25 i.MX25 Machine Specific Layer (MSL)
 */

/*!
 * @file mach-mx25/pm.c
 * @brief This file contains suspend operations
 *
 * @ingroup MSL_MX25
 */
static unsigned int cgcr0, cgcr1, cgcr2;

static int mx25_suspend_enter(suspend_state_t state)
{
	unsigned int reg;

	switch (state) {
	case PM_SUSPEND_MEM:
		mxc_cpu_lp_set(STOP_POWER_OFF);
		break;
	case PM_SUSPEND_STANDBY:
		mxc_cpu_lp_set(WAIT_UNCLOCKED_POWER_OFF);
		break;
	default:
		return -EINVAL;
	}
	/* Executing CP15 (Wait-for-Interrupt) Instruction */
	cpu_do_idle();

	reg = (__raw_readl(MXC_CCM_CGCR0) & ~MXC_CCM_CGCR0_STOP_MODE_MASK) |
	    cgcr0;
	__raw_writel(reg, MXC_CCM_CGCR0);

	reg = (__raw_readl(MXC_CCM_CGCR1) & ~MXC_CCM_CGCR1_STOP_MODE_MASK) |
	    cgcr1;
	__raw_writel(reg, MXC_CCM_CGCR1);

	reg = (__raw_readl(MXC_CCM_CGCR2) & ~MXC_CCM_CGCR2_STOP_MODE_MASK) |
	    cgcr2;
	__raw_writel(reg, MXC_CCM_CGCR2);

	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx25_suspend_prepare(void)
{
	cgcr0 = __raw_readl(MXC_CCM_CGCR0) & MXC_CCM_CGCR0_STOP_MODE_MASK;
	cgcr1 = __raw_readl(MXC_CCM_CGCR1) & MXC_CCM_CGCR1_STOP_MODE_MASK;
	cgcr2 = __raw_readl(MXC_CCM_CGCR2) & MXC_CCM_CGCR2_STOP_MODE_MASK;

	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mx25_suspend_finish(void)
{
}

static int mx25_pm_valid(suspend_state_t state)
{
	return state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX;
}

struct platform_suspend_ops mx25_suspend_ops = {
	.valid = mx25_pm_valid,
	.prepare = mx25_suspend_prepare,
	.enter = mx25_suspend_enter,
	.finish = mx25_suspend_finish,
};

static int __init mx25_pm_init(void)
{
	pr_info("Static Power Management for Freescale i.MX25\n");
	suspend_set_ops(&mx25_suspend_ops);

	return 0;
}

late_initcall(mx25_pm_init);
