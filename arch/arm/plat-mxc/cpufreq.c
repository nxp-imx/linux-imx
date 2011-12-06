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

/*
 * A driver for the Freescale Semiconductor i.MXC CPUfreq module.
 * The CPUFREQ driver is for controlling CPU frequency. It allows you to change
 * the CPU clock speed on the fly.
 */
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#include <asm/smp_plat.h>
#include <asm/cpu.h>

#include <mach/hardware.h>
#include <mach/clock.h>

#define CLK32_FREQ	32768
#define NANOSECOND	(1000 * 1000 * 1000)

static int cpu_freq_khz_min;
static int cpu_freq_khz_max;

static struct clk *cpu_clk;
static struct cpufreq_frequency_table *imx_freq_table;

static int cpu_op_nr;
static struct cpu_op *cpu_op_tbl;
static u32 pre_suspend_rate;

extern int dvfs_core_is_active;
extern struct cpu_op *(*get_cpu_op)(int *op);
extern int (*set_cpu_voltage)(u32 cpu_volt);

int set_cpu_freq(int freq)
{
	int i, ret = 0;
	int org_cpu_rate;
	int gp_volt = 0;

	org_cpu_rate = clk_get_rate(cpu_clk);
	if (org_cpu_rate == freq)
		return ret;

	for (i = 0; i < cpu_op_nr; i++) {
		if (freq == cpu_op_tbl[i].cpu_rate)
			gp_volt = cpu_op_tbl[i].cpu_voltage;
	}

	if (gp_volt == 0)
		return ret;

	/*Set the voltage for the GP domain. */
	if (freq > org_cpu_rate) {
		ret = set_cpu_voltage(gp_volt);
		if (ret < 0) {
			printk(KERN_DEBUG "COULD NOT SET GP VOLTAGE!!!!\n");
			return ret;
		}
		udelay(50);
	}

	ret = clk_set_rate(cpu_clk, freq);
	if (ret != 0) {
		printk(KERN_DEBUG "cannot set CPU clock rate\n");
		return ret;
	}

	if (freq < org_cpu_rate) {
		ret = set_cpu_voltage(gp_volt);
		if (ret < 0) {
			printk(KERN_DEBUG "COULD NOT SET GP VOLTAGE!!!!\n");
			return ret;
		}
	}

	return ret;
}

static int mxc_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu > num_possible_cpus())
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, imx_freq_table);
}

static unsigned int mxc_get_speed(unsigned int cpu)
{
	if (cpu > num_possible_cpus())
		return 0;

	return clk_get_rate(cpu_clk) / 1000;
}

static int mxc_set_target(struct cpufreq_policy *policy,
			  unsigned int target_freq, unsigned int relation)
{
	struct cpufreq_freqs freqs;
	int freq_Hz;
	int ret = 0;
	unsigned int index;
	int i, num_cpus;

	num_cpus = num_possible_cpus();
	if (policy->cpu > num_cpus)
		return 0;

	if (dvfs_core_is_active) {
		printk(KERN_DEBUG"DVFS-CORE is active, cannot change frequency using CPUFREQ\n");
		return ret;
	}

	cpufreq_frequency_table_target(policy, imx_freq_table,
			target_freq, relation, &index);
	freq_Hz = imx_freq_table[index].frequency * 1000;

	freqs.old = clk_get_rate(cpu_clk) / 1000;
	freqs.new = freq_Hz / 1000;
	freqs.cpu = policy->cpu;
	freqs.flags = 0;

	for (i = 0; i < num_cpus; i++) {
		freqs.cpu = i;
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	}

	ret = set_cpu_freq(freq_Hz);

#ifdef CONFIG_SMP
	/* Loops per jiffy is not updated by the CPUFREQ driver for SMP systems.
	  * So update it for all CPUs.
	  */

	for_each_cpu(i, policy->cpus)
		per_cpu(cpu_data, i).loops_per_jiffy =
		cpufreq_scale(per_cpu(cpu_data, i).loops_per_jiffy,
					freqs.old, freqs.new);
#endif
	for (i = 0; i < num_cpus; i++) {
		freqs.cpu = i;
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}


	return ret;
}

static int mxc_cpufreq_suspend(struct cpufreq_policy *policy,
				 pm_message_t state)
{
	pre_suspend_rate = clk_get_rate(cpu_clk);
	/* Set to max freq and voltage */
	if (pre_suspend_rate != (imx_freq_table[0].frequency * 1000))
		set_cpu_freq(imx_freq_table[0].frequency);

	return 0;
}

static int mxc_cpufreq_resume(struct cpufreq_policy *policy)
{
	if (clk_get_rate(cpu_clk) != pre_suspend_rate)
		set_cpu_freq(pre_suspend_rate);

	return 0;
}

static int __devinit mxc_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret;
	int i;

	printk(KERN_INFO "i.MXC CPU frequency driver\n");

	if (policy->cpu >= num_possible_cpus())
		return -EINVAL;

	if (!get_cpu_op)
		return -EINVAL;

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_ERR "%s: failed to get cpu clock\n", __func__);
		return PTR_ERR(cpu_clk);
	}

	cpu_op_tbl = get_cpu_op(&cpu_op_nr);

	cpu_freq_khz_min = cpu_op_tbl[0].cpu_rate / 1000;
	cpu_freq_khz_max = cpu_op_tbl[0].cpu_rate / 1000;

	if (imx_freq_table == NULL) {
		imx_freq_table = kmalloc(
			sizeof(struct cpufreq_frequency_table) * (cpu_op_nr + 1),
				GFP_KERNEL);
		if (!imx_freq_table) {
			ret = -ENOMEM;
			goto err1;
		}

		for (i = 0; i < cpu_op_nr; i++) {
			imx_freq_table[i].index = i;
			imx_freq_table[i].frequency = cpu_op_tbl[i].cpu_rate / 1000;

			if ((cpu_op_tbl[i].cpu_rate / 1000) < cpu_freq_khz_min)
				cpu_freq_khz_min = cpu_op_tbl[i].cpu_rate / 1000;

			if ((cpu_op_tbl[i].cpu_rate / 1000) > cpu_freq_khz_max)
				cpu_freq_khz_max = cpu_op_tbl[i].cpu_rate / 1000;
		}

		imx_freq_table[i].index = i;
		imx_freq_table[i].frequency = CPUFREQ_TABLE_END;
	}
	policy->cur = clk_get_rate(cpu_clk) / 1000;
	policy->min = policy->cpuinfo.min_freq = cpu_freq_khz_min;
	policy->max = policy->cpuinfo.max_freq = cpu_freq_khz_max;

	/* All processors share the same frequency and voltage.
	  * So all frequencies need to be scaled together.
	  */
	 if (is_smp()) {
		policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
		cpumask_setall(policy->cpus);
	}

	/* Manual states, that PLL stabilizes in two CLK32 periods */
	policy->cpuinfo.transition_latency = 2 * NANOSECOND / CLK32_FREQ;

	ret = cpufreq_frequency_table_cpuinfo(policy, imx_freq_table);

	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register i.MXC CPUfreq with error code %d\n",
		       __func__, ret);
		goto err;
	}

	cpufreq_frequency_table_get_attr(imx_freq_table, policy->cpu);
	return 0;
err:
	kfree(imx_freq_table);
err1:
	clk_put(cpu_clk);
	return ret;
}

static int mxc_cpufreq_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);

	if (policy->cpu == 0) {
		set_cpu_freq(cpu_freq_khz_max * 1000);
		clk_put(cpu_clk);
		kfree(imx_freq_table);
	}
	return 0;
}

static struct cpufreq_driver mxc_driver = {
	.flags = CPUFREQ_STICKY,
	.verify = mxc_verify_speed,
	.target = mxc_set_target,
	.get = mxc_get_speed,
	.init = mxc_cpufreq_init,
	.exit = mxc_cpufreq_exit,
	.suspend = mxc_cpufreq_suspend,
	.resume = mxc_cpufreq_resume,
	.name = "imx",
};

static int __init mxc_cpufreq_driver_init(void)
{
	return cpufreq_register_driver(&mxc_driver);
}

static void mxc_cpufreq_driver_exit(void)
{
	cpufreq_unregister_driver(&mxc_driver);
}

module_init(mxc_cpufreq_driver_init);
module_exit(mxc_cpufreq_driver_exit);

MODULE_AUTHOR("Freescale Semiconductor Inc. Yong Shen <yong.shen@linaro.org>");
MODULE_DESCRIPTION("CPUfreq driver for i.MX");
MODULE_LICENSE("GPL");
