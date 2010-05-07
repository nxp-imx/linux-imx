/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/notifier.h>

#include <mach/hardware.h>
#include <linux/io.h>
#include <asm/system.h>
#include <mach/regulator.h>
#include <mach/power.h>
#include <mach/clock.h>
#include <mach/bus_freq.h>

static struct regulator *cpu_regulator;
static struct clk *cpu_clk;
static struct clk *ahb_clk;
static struct clk *x_clk;
static struct clk *emi_clk;
static struct regulator *vddd;
static struct regulator *vdddbo;
static struct regulator *vddio;
static struct regulator *vdda;
static struct cpufreq_frequency_table imx_freq_table[7];
int cpu_freq_khz_min;
int cpu_freq_khz_max;
int cpufreq_trig_needed;
int cur_freq_table_size;
int lcd_on_freq_table_size;
int lcd_off_freq_table_size;
int high_freq_needed;

extern char *ahb_clk_id;
extern struct profile profiles[OPERATION_WP_SUPPORTED];
extern int low_freq_used(void);

static int set_freq_table(struct cpufreq_policy *policy, int end_index)
{
	int ret = 0;
	int i;
	int zero_no = 0;

	for (i = 0; i < end_index; i++) {
		if (profiles[i].cpu == 0)
			zero_no++;
	}

	end_index -= zero_no;

	cpu_freq_khz_min = profiles[0].cpu;
	cpu_freq_khz_max = profiles[0].cpu;
	for (i = 0; i < end_index; i++) {
		imx_freq_table[end_index - 1 - i].index = end_index - i;
		imx_freq_table[end_index - 1 - i].frequency =
						profiles[i].cpu;

		if ((profiles[i].cpu) < cpu_freq_khz_min)
			cpu_freq_khz_min = profiles[i].cpu;

		if ((profiles[i].cpu) > cpu_freq_khz_max)
			cpu_freq_khz_max = profiles[i].cpu;
	}

	imx_freq_table[i].index = 0;
	imx_freq_table[i].frequency = CPUFREQ_TABLE_END;

	policy->cur = clk_get_rate(cpu_clk) / 1000;
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	policy->min = policy->cpuinfo.min_freq = cpu_freq_khz_min;
	policy->max = policy->cpuinfo.max_freq = cpu_freq_khz_max;

	/* Manual states, that PLL stabilizes in two CLK32 periods */
	policy->cpuinfo.transition_latency = 1000;

	ret = cpufreq_frequency_table_cpuinfo(policy, imx_freq_table);

	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register i.MXC CPUfreq\n",
		       __func__);
		return ret;
	}

	cpufreq_frequency_table_get_attr(imx_freq_table, policy->cpu);

	return ret;
}

static int set_op(struct cpufreq_policy *policy, unsigned int target_freq)
{
	struct cpufreq_freqs freqs;
	int ret = 0, i;

	freqs.old = clk_get_rate(cpu_clk) / 1000;
	freqs.cpu = 0;

/* work around usb problem when in updater firmare  mode*/
#ifdef CONFIG_MXS_UTP
	return 0;
#endif
	for (i = cur_freq_table_size - 1; i > 0; i--) {
		if (profiles[i].cpu <= target_freq &&
		    target_freq < profiles[i - 1].cpu) {
			freqs.new = profiles[i].cpu;
			break;
		}

		if (!vddd && profiles[i].cpu > freqs.old) {
			/* can't safely set more than now */
			freqs.new = profiles[i + 1].cpu;
			break;
		}
	}

	if (i == 0)
		freqs.new = profiles[i].cpu;

	if ((freqs.old / 1000) == (freqs.new / 1000)) {
		if (regulator_get_voltage(vddd) == profiles[i].vddd)
			return 0;
	}

	if (cpu_regulator && (freqs.old < freqs.new)) {
		ret = regulator_set_current_limit(cpu_regulator,
			profiles[i].cur, profiles[i].cur);
		if (ret)
			return ret;
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	if (freqs.old > freqs.new) {
		int ss = profiles[i].ss;

		/* change emi while cpu is fastest to minimize
		 * time spent changing emiclk
		 */
		clk_set_rate(emi_clk, (profiles[i].emi) * 1000);
		clk_set_rate(cpu_clk, (profiles[i].cpu) * 1000);
		clk_set_rate(ahb_clk, (profiles[i].ahb) * 1000);
		/* x_clk order doesn't really matter */
		clk_set_rate(x_clk, (profiles[i].xbus) * 1000);
		timing_ctrl_rams(ss);

		if (vddd && vdddbo && vddio && vdda) {
			ret = regulator_set_voltage(vddd,
						    profiles[i].vddd,
						    profiles[i].vddd);
			if (ret)
				ret = regulator_set_voltage(vddd,
							    profiles[i].vddd,
							    profiles[i].vddd);
			regulator_set_voltage(vdddbo,
					      profiles[i].vddd_bo,
					      profiles[i].vddd_bo);

			ret = regulator_set_voltage(vddio,
						    profiles[i].vddio,
						    profiles[i].vddio);
			if (ret)
				ret = regulator_set_voltage(vddio,
							    profiles[i].vddio,
							    profiles[i].vddio);
			ret = regulator_set_voltage(vdda,
						    profiles[i].vdda,
						    profiles[i].vdda);
			if (ret)
				ret = regulator_set_voltage(vdda,
							    profiles[i].vdda,
							    profiles[i].vdda);
		}
	} else {
		int ss = profiles[i].ss;
		if (vddd && vdddbo && vddio && vdda) {
			ret = regulator_set_voltage(vddd,
						    profiles[i].vddd,
						    profiles[i].vddd);
			if (ret)
				ret = regulator_set_voltage(vddd,
							    profiles[i].vddd,
							    profiles[i].vddd);
			regulator_set_voltage(vdddbo,
					      profiles[i].vddd_bo,
					      profiles[i].vddd_bo);
			ret = regulator_set_voltage(vddio,
						    profiles[i].vddio,
						    profiles[i].vddio);
			if (ret)
				ret = regulator_set_voltage(vddio,
							    profiles[i].vddio,
							    profiles[i].vddio);
			ret = regulator_set_voltage(vdda,
						    profiles[i].vdda,
						    profiles[i].vdda);
			if (ret)
				ret = regulator_set_voltage(vdda,
							    profiles[i].vdda,
							    profiles[i].vdda);
		}
		/* x_clk order doesn't really matter */
		clk_set_rate(x_clk, (profiles[i].xbus) * 1000);
		timing_ctrl_rams(ss);
		clk_set_rate(cpu_clk, (profiles[i].cpu) * 1000);
		clk_set_rate(ahb_clk, (profiles[i].ahb) * 1000);
		clk_set_rate(emi_clk, (profiles[i].emi) * 1000);
	}

	if (is_hclk_autoslow_ok())
		clk_set_h_autoslow_flags(profiles[i].h_autoslow_flags);
	else
		clk_enable_h_autoslow(false);

	if (high_freq_needed == 0)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	if (cpu_regulator && (freqs.old > freqs.new))   /* will not fail */
		regulator_set_current_limit(cpu_regulator,
					    profiles[i].cur,
					    profiles[i].cur);

	if (high_freq_needed == 1) {
		high_freq_needed = 0;
		cur_freq_table_size = lcd_on_freq_table_size;
		set_freq_table(policy, cur_freq_table_size);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}

	return ret;
}

static int calc_frequency_khz(int target, unsigned int relation)
{
	int i;

	if (target * 1000 == clk_get_rate(cpu_clk))
		return target;

	if (relation == CPUFREQ_RELATION_H) {
		for (i = cur_freq_table_size - 1; i >= 0; i--) {
			if (imx_freq_table[i].frequency <= target)
				return imx_freq_table[i].frequency;
		}
	} else if (relation == CPUFREQ_RELATION_L) {
		for (i = 0; i < cur_freq_table_size; i++) {
			if (imx_freq_table[i].frequency >= target)
				return imx_freq_table[i].frequency;
		}
}

	printk(KERN_ERR "Error: No valid cpufreq relation\n");
	return cpu_freq_khz_max;
}

static int mxs_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	int freq_KHz;
	struct cpufreq_freqs freqs;
	int low_freq_bus_ready = 0;

	if (cpufreq_trig_needed  == 1) {
		/* Set the current working point. */
		cpufreq_trig_needed = 0;
		target_freq = clk_get_rate(cpu_clk) / 1000;
		low_freq_bus_ready = low_freq_used();

		if ((target_freq < LCD_ON_CPU_FREQ_KHZ) &&
		    (low_freq_bus_ready == 0)) {
			high_freq_needed = 1;
			target_freq = LCD_ON_CPU_FREQ_KHZ;
			goto change_freq;
		}

		target_freq = clk_get_rate(cpu_clk) / 1000;
		freq_KHz = calc_frequency_khz(target_freq, relation);

		freqs.old = target_freq;
		freqs.new = freq_KHz;
		freqs.cpu = 0;
		freqs.flags = 0;
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		low_freq_bus_ready = low_freq_used();
		if (low_freq_bus_ready) {
			int i;
			cur_freq_table_size = lcd_off_freq_table_size;
			/* find current table index to get
			 * hbus autoslow flags and enable hbus autoslow.
			 */
			for (i = cur_freq_table_size - 1; i > 0; i--) {
				if (profiles[i].cpu <= target_freq &&
					target_freq < profiles[i - 1].cpu) {
					clk_set_h_autoslow_flags(
					profiles[i].h_autoslow_flags);
					break;
				}
			}
		} else {
			cur_freq_table_size = lcd_on_freq_table_size;
			clk_enable_h_autoslow(false);
		}

		set_freq_table(policy, cur_freq_table_size);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
		return 0;
}

	/*
	 * Some governors do not respects CPU and policy lower limits
	 * which leads to bad things (division by zero etc), ensure
	 * that such things do not happen.
	 */
change_freq:	if (target_freq < policy->cpuinfo.min_freq)
		target_freq = policy->cpuinfo.min_freq;

	if (target_freq < policy->min)
		target_freq = policy->min;

	freq_KHz = calc_frequency_khz(target_freq, relation);
	return set_op(policy, freq_KHz);
	}

static unsigned int mxs_getspeed(unsigned int cpu)
{
	if (cpu)
		return 0;

	return clk_get_rate(cpu_clk) / 1000;
}


static int mxs_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, imx_freq_table);
}

static int __init mxs_cpu_init(struct cpufreq_policy *policy)
{
	int ret = 0;
	int i;

	cpu_clk = clk_get(NULL, "cpu");
	if (IS_ERR(cpu_clk)) {
		ret = PTR_ERR(cpu_clk);
		goto out_cpu;
	}

	ahb_clk = clk_get(NULL, "h");
	if (IS_ERR(ahb_clk)) {
		ret = PTR_ERR(ahb_clk);
		goto out_ahb;
	}

	x_clk = clk_get(NULL, "x");
	if (IS_ERR(ahb_clk)) {
		ret = PTR_ERR(x_clk);
		goto out_x;
	}

	emi_clk = clk_get(NULL, "emi");
	if (IS_ERR(emi_clk)) {
		ret = PTR_ERR(emi_clk);
		goto out_emi;
	}

	if (policy->cpu != 0)
		return -EINVAL;

	cpu_regulator = regulator_get(NULL, "cpufreq-1");
	if (IS_ERR(cpu_regulator)) {
		printk(KERN_ERR "%s: failed to get CPU regulator\n", __func__);
		cpu_regulator = NULL;
		ret = PTR_ERR(cpu_regulator);
		goto out_cur;
	}

	vddd = regulator_get(NULL, "vddd");
	if (IS_ERR(vddd)) {
		printk(KERN_ERR "%s: failed to get vddd regulator\n", __func__);
		vddd = NULL;
		ret = PTR_ERR(vddd);
		goto out_cur;
	}

	vdddbo = regulator_get(NULL, "vddd_bo");
	if (IS_ERR(vdddbo)) {
		vdddbo = NULL;
		pr_warning("unable to get vdddbo");
		ret = PTR_ERR(vdddbo);
		goto out_cur;
	}

	vddio = regulator_get(NULL, "vddio");
	if (IS_ERR(vddio)) {
		vddio = NULL;
		pr_warning("unable to get vddio");
		ret = PTR_ERR(vddio);
		goto out_cur;
	}

	vdda = regulator_get(NULL, "vdda");
	if (IS_ERR(vdda)) {
		vdda = NULL;
		pr_warning("unable to get vdda");
		ret = PTR_ERR(vdda);
		goto out_cur;
	}

	for (i = 0; i < ARRAY_SIZE(profiles); i++) {
		if ((profiles[i].cpu) == LCD_ON_CPU_FREQ_KHZ) {
			lcd_on_freq_table_size = i + 1;
			break;
		}
	}

	if (i == ARRAY_SIZE(profiles)) {
		pr_warning("unable to find frequency for LCD on");
		printk(KERN_ERR "lcd_on_freq_table_size=%d\n",
			lcd_on_freq_table_size);
		goto out_cur;
	}

	for (i = 0; i < ARRAY_SIZE(profiles); i++) {
		if ((profiles[i].cpu) == 0) {
			lcd_off_freq_table_size = i;
			break;
		}
	}

	if (i == ARRAY_SIZE(profiles))
		lcd_off_freq_table_size = i;

	/* Set the current working point. */
	set_freq_table(policy, lcd_on_freq_table_size);
	cpufreq_trig_needed = 0;
	high_freq_needed = 0;
	cur_freq_table_size = lcd_on_freq_table_size;

	printk(KERN_INFO "%s: cpufreq init finished\n", __func__);
	return 0;
out_cur:
	if (cpu_regulator)
		regulator_put(cpu_regulator);
	if (vddd)
		regulator_put(vddd);
	if (vddio)
		regulator_put(vddio);
	if (vdda)
		regulator_put(vdda);

	clk_put(emi_clk);
out_emi:
	clk_put(x_clk);
out_x:
	clk_put(ahb_clk);
out_ahb:
	clk_put(cpu_clk);
out_cpu:
	return ret;
}

static int mxs_cpu_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);

	/* Reset CPU to 392MHz */
	set_op(policy, profiles[1].cpu);

	clk_put(cpu_clk);
	regulator_put(cpu_regulator);
	return 0;
}

static struct cpufreq_driver mxs_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= mxs_verify_speed,
	.target		= mxs_target,
	.get		= mxs_getspeed,
	.init		= mxs_cpu_init,
	.exit		= mxs_cpu_exit,
	.name		= "mxs",
};

static int __devinit mxs_cpufreq_init(void)
{
	return cpufreq_register_driver(&mxs_driver);
}

static void mxs_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&mxs_driver);
}

module_init(mxs_cpufreq_init);
module_exit(mxs_cpufreq_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("CPUfreq driver for i.MX");
MODULE_LICENSE("GPL");

