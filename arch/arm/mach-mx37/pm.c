/*
 *  Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <mach/hardware.h>

static struct cpu_wp *cpu_wp_tbl;
static struct clk *cpu_clk;

#if defined(CONFIG_CPU_FREQ)
static int org_freq;
extern int cpufreq_suspended;
extern int set_cpu_freq(int wp);
#endif

static int mx37_suspend_enter(suspend_state_t state)
{
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

	if (tzic_enable_wake(0) != 0)
		return -EAGAIN;

	cpu_do_idle();

	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx37_suspend_prepare(void)
{
#if defined(CONFIG_CPU_FREQ)
	struct cpufreq_freqs freqs;
	org_freq = clk_get_rate(cpu_clk);
	freqs.old = org_freq / 1000;
	freqs.new = cpu_wp_tbl[0].cpu_rate / 1000;
	freqs.cpu = 0;
	freqs.flags = 0;

	cpufreq_suspended = 1;
	if (clk_get_rate(cpu_clk) != cpu_wp_tbl[0].cpu_rate) {
		set_cpu_freq(cpu_wp_tbl[0].cpu_rate);
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
#endif
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mx37_suspend_finish(void)
{
#if defined(CONFIG_CPU_FREQ)
	struct cpufreq_freqs freqs;

	freqs.old = clk_get_rate(cpu_clk) / 1000;
	freqs.new = org_freq / 1000;
	freqs.cpu = 0;
	freqs.flags = 0;

	cpufreq_suspended = 0;

	if (org_freq != clk_get_rate(cpu_clk)) {
		set_cpu_freq(org_freq);
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
#endif
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
	int cpu_wp_nr;

	pr_info("Static Power Management for Freescale i.MX37\n");
	suspend_set_ops(&mx37_suspend_ops);

	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get cpu_clk\n", __func__);
		return PTR_ERR(cpu_clk);
	}
	return 0;
}

late_initcall(mx37_pm_init);
