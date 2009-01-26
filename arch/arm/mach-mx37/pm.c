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

static int mx37_suspend_enter(suspend_state_t state)
{
	if (tzic_enable_wake(0) != 0)
		return -EAGAIN;

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
	cpu_do_idle();

	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx37_suspend_prepare(void)
{
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mx37_suspend_finish(void)
{
}

static int mx37_pm_valid(suspend_state_t state)
{
	return (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX);
}

struct platform_suspend_ops mx37_suspend_ops = {
	.valid = mx37_pm_valid,
	.prepare = mx37_suspend_prepare,
	.enter = mx37_suspend_enter,
	.finish = mx37_suspend_finish,
};

static int __init mx37_pm_init(void)
{
	pr_info("Static Power Management for Freescale i.MX37\n");
	suspend_set_ops(&mx37_suspend_ops);

	return 0;
}

late_initcall(mx37_pm_init);
