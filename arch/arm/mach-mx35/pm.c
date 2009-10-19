/*
 *  Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <mach/hardware.h>

/*!
 * @defgroup MSL_MX35 i.MX35 Machine Specific Layer (MSL)
 */

/*!
 * @file mach-mx35/pm.c
 * @brief This file contains suspend operations
 *
 * @ingroup MSL_MX35
 */
static int mx35_suspend_enter(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_MEM:
		mxc_cpu_lp_set(STOP_POWER_OFF);
		break;
	case PM_SUSPEND_STANDBY:
		mxc_cpu_lp_set(STOP_POWER_ON);
		break;
	default:
		return -EINVAL;
	}
	/* Executing CP15 (Wait-for-Interrupt) Instruction */
	cpu_do_idle();
	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx35_suspend_prepare(void)
{
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mx35_suspend_finish(void)
{
}

static int mx35_pm_valid(suspend_state_t state)
{
	return (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX);
}

struct platform_suspend_ops mx35_suspend_ops = {
	.valid = mx35_pm_valid,
	.prepare = mx35_suspend_prepare,
	.enter = mx35_suspend_enter,
	.finish = mx35_suspend_finish,
};

static int __init mx35_pm_init(void)
{
	pr_info("Static Power Management for Freescale i.MX35\n");
	suspend_set_ops(&mx35_suspend_ops);

	return 0;
}

late_initcall(mx35_pm_init);
