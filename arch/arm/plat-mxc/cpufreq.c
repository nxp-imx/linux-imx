/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file cpufreq.c
 *
 * @brief A driver for the Freescale Semiconductor i.MXC CPUfreq module.
 *
 * The CPUFREQ driver is for controling CPU frequency. It allows you to change
 * the CPU clock speed on the fly.
 *
 * @ingroup PM
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <asm/setup.h>
#include <mach/clock.h>
#include <asm/cacheflush.h>
#include <linux/hrtimer.h>

int cpu_freq_khz_min;
int cpu_freq_khz_max;
int arm_lpm_clk;
int arm_normal_clk;
int cpufreq_suspended;
int cpufreq_trig_needed;

static struct clk *cpu_clk;
static struct regulator *gp_regulator;
static struct cpu_wp *cpu_wp_tbl;
static struct cpufreq_frequency_table imx_freq_table[6];
extern int low_bus_freq_mode;
extern int high_bus_freq_mode;
extern int dvfs_core_is_active;
extern int cpu_wp_nr;
extern char *gp_reg_id;

extern int set_low_bus_freq(void);
extern int set_high_bus_freq(int high_bus_speed);
extern int low_freq_bus_used(void);

#ifdef CONFIG_ARCH_MX5
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
#endif

int set_cpu_freq(int freq)
{
	int ret = 0;
	int org_cpu_rate;
	int gp_volt = 0;
	int i;

	org_cpu_rate = clk_get_rate(cpu_clk);
	if (org_cpu_rate == freq)
		return ret;

	for (i = 0; i < cpu_wp_nr; i++) {
		if (freq == cpu_wp_tbl[i].cpu_rate)
			gp_volt = cpu_wp_tbl[i].cpu_voltage;
	}

	if (gp_volt == 0)
		return ret;

	/*Set the voltage for the GP domain. */
	if (freq > org_cpu_rate) {
		ret = regulator_set_voltage(gp_regulator, gp_volt, gp_volt);
		if (ret < 0) {
			printk(KERN_DEBUG "COULD NOT SET GP VOLTAGE!!!!\n");
			return ret;
		}
	}

	ret = clk_set_rate(cpu_clk, freq);
	if (ret != 0) {
		printk(KERN_DEBUG "cannot set CPU clock rate\n");
		return ret;
	}

	if (freq < org_cpu_rate) {
		ret = regulator_set_voltage(gp_regulator, gp_volt, gp_volt);
		if (ret < 0) {
			printk(KERN_DEBUG "COULD NOT SET GP VOLTAGE!!!!\n");
			return ret;
		}
	}

	return ret;
}

static int mxc_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, imx_freq_table);
}

static unsigned int mxc_get_speed(unsigned int cpu)
{
	if (cpu)
		return 0;

	return clk_get_rate(cpu_clk) / 1000;
}

static int calc_frequency_khz(int target, unsigned int relation)
{
	int i;

	if ((target * 1000) == clk_get_rate(cpu_clk))
		return target;

	if (relation == CPUFREQ_RELATION_H) {
		for (i = cpu_wp_nr - 1; i >= 0; i--) {
			if (imx_freq_table[i].frequency <= target)
				return imx_freq_table[i].frequency;
		}
	} else if (relation == CPUFREQ_RELATION_L) {
		for (i = 0; i < cpu_wp_nr; i++) {
			if (imx_freq_table[i].frequency >= target)
				return imx_freq_table[i].frequency;
		}
	}
	printk(KERN_ERR "Error: No valid cpufreq relation\n");
	return cpu_freq_khz_max;
}

static int mxc_set_target(struct cpufreq_policy *policy,
			  unsigned int target_freq, unsigned int relation)
{
	struct cpufreq_freqs freqs;
	int freq_Hz;
	int low_freq_bus_ready = 0;
	int ret = 0;

	if (dvfs_core_is_active || cpufreq_suspended) {
		target_freq = clk_get_rate(cpu_clk) / 1000;
		freq_Hz = calc_frequency_khz(target_freq, relation) * 1000;
		if (freq_Hz == arm_lpm_clk)
			freqs.old = cpu_wp_tbl[cpu_wp_nr - 2].cpu_rate / 1000;
		else
			freqs.old = arm_lpm_clk / 1000;

		freqs.new = freq_Hz / 1000;
		freqs.cpu = 0;
		freqs.flags = 0;
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
		return ret;
	}
	/*
	 * Some governors do not respects CPU and policy lower limits
	 * which leads to bad things (division by zero etc), ensure
	 * that such things do not happen.
	 */
	if (target_freq < policy->cpuinfo.min_freq)
		target_freq = policy->cpuinfo.min_freq;

	if (target_freq < policy->min)
		target_freq = policy->min;

	freq_Hz = calc_frequency_khz(target_freq, relation) * 1000;

	freqs.old = clk_get_rate(cpu_clk) / 1000;
	freqs.new = freq_Hz / 1000;
	freqs.cpu = 0;
	freqs.flags = 0;
	low_freq_bus_ready = low_freq_bus_used();
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	if (!dvfs_core_is_active) {
		if ((freq_Hz == arm_lpm_clk) && (!low_bus_freq_mode)
		    && (low_freq_bus_ready)) {
			if (freqs.old != freqs.new)
				ret = set_cpu_freq(freq_Hz);
			set_low_bus_freq();

		} else {
			set_high_bus_freq(0);
			ret = set_cpu_freq(freq_Hz);
		}
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

static int __init mxc_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	int ret;
	int i;

	printk(KERN_INFO "i.MXC CPU frequency driver\n");

	if (policy->cpu != 0)
		return -EINVAL;

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_ERR "%s: failed to get cpu clock\n", __func__);
		return PTR_ERR(cpu_clk);
	}

	gp_regulator = regulator_get(NULL, gp_reg_id);
	if (IS_ERR(gp_regulator)) {
		clk_put(cpu_clk);
		printk(KERN_ERR "%s: failed to get gp regulator\n", __func__);
		return PTR_ERR(gp_regulator);
	}

	/* Set the current working point. */
	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);

	cpu_freq_khz_min = cpu_wp_tbl[0].cpu_rate / 1000;
	cpu_freq_khz_max = cpu_wp_tbl[0].cpu_rate / 1000;

	for (i = 0; i < cpu_wp_nr; i++) {
		imx_freq_table[cpu_wp_nr - 1 - i].index = cpu_wp_nr - i;
		imx_freq_table[cpu_wp_nr - 1 - i].frequency =
		    cpu_wp_tbl[i].cpu_rate / 1000;

		if ((cpu_wp_tbl[i].cpu_rate / 1000) < cpu_freq_khz_min)
			cpu_freq_khz_min = cpu_wp_tbl[i].cpu_rate / 1000;

		if ((cpu_wp_tbl[i].cpu_rate / 1000) > cpu_freq_khz_max)
			cpu_freq_khz_max = cpu_wp_tbl[i].cpu_rate / 1000;
	}

	imx_freq_table[i].index = 0;
	imx_freq_table[i].frequency = CPUFREQ_TABLE_END;

	policy->cur = clk_get_rate(cpu_clk) / 1000;
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	policy->min = policy->cpuinfo.min_freq = cpu_freq_khz_min;
	policy->max = policy->cpuinfo.max_freq = cpu_freq_khz_max;

	arm_lpm_clk = cpu_freq_khz_min * 1000;
	arm_normal_clk = cpu_freq_khz_max * 1000;

	/* Manual states, that PLL stabilizes in two CLK32 periods */
	policy->cpuinfo.transition_latency = 10;

	ret = cpufreq_frequency_table_cpuinfo(policy, imx_freq_table);

	if (ret < 0) {
		clk_put(cpu_clk);
		regulator_put(gp_regulator);
		printk(KERN_ERR "%s: failed to register i.MXC CPUfreq\n",
		       __func__);
		return ret;
	}

	cpufreq_frequency_table_get_attr(imx_freq_table, policy->cpu);
	return 0;
}

static int mxc_cpufreq_suspend(struct cpufreq_policy *policy,
				     pm_message_t state)
{
	return 0;
}

static int mxc_cpufreq_resume(struct cpufreq_policy *policy)
{
	return 0;
}

static int mxc_cpufreq_driver_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);

	/* Reset CPU to 665MHz */
	if (!dvfs_core_is_active)
		set_cpu_freq(arm_normal_clk);
	if (!high_bus_freq_mode)
		set_high_bus_freq(1);

	clk_put(cpu_clk);
	regulator_put(gp_regulator);
	return 0;
}

static struct cpufreq_driver mxc_driver = {
	.flags = CPUFREQ_STICKY,
	.verify = mxc_verify_speed,
	.target = mxc_set_target,
	.get = mxc_get_speed,
	.init = mxc_cpufreq_driver_init,
	.exit = mxc_cpufreq_driver_exit,
	.suspend = mxc_cpufreq_suspend,
	.resume = mxc_cpufreq_resume,
	.name = "imx",
};

static int __devinit mxc_cpufreq_init(void)
{
	return cpufreq_register_driver(&mxc_driver);
}

static void mxc_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&mxc_driver);
}

module_init(mxc_cpufreq_init);
module_exit(mxc_cpufreq_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("CPUfreq driver for i.MX");
MODULE_LICENSE("GPL");
