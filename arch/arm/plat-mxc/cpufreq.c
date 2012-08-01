/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/io.h>
#include <asm/smp_plat.h>
#include <asm/cpu.h>

#include <mach/hardware.h>
#include <mach/clock.h>

#define CLK32_FREQ	32768
#define NANOSECOND	(1000 * 1000 * 1000)

/*If using cpu internal ldo bypass,we need config pmic by I2C in suspend
interface, but cpufreq driver as sys_dev is more later to suspend than I2C
driver, so we should implement another I2C operate function which isolated
with kernel I2C driver, these code is copied from u-boot*/
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
/*0:normal; 1: in the middle of suspend or resume; 2: suspended*/
static int cpu_freq_suspend_in;
static struct mutex set_cpufreq_lock;
#endif

static int cpu_freq_khz_min;
static int cpu_freq_khz_max;

static struct clk *cpu_clk;
static struct cpufreq_frequency_table *imx_freq_table;

static int cpu_op_nr;
static struct cpu_op *cpu_op_tbl;
static u32 pre_suspend_rate;

extern struct regulator *cpu_regulator;
extern int dvfs_core_is_active;
extern struct cpu_op *(*get_cpu_op)(int *op);
extern int low_bus_freq_mode;
extern int audio_bus_freq_mode;
extern int high_bus_freq_mode;
extern int set_low_bus_freq(void);
extern int set_high_bus_freq(int high_bus_speed);
extern int low_freq_bus_used(void);

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
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	/*Do not change cpufreq if system enter suspend flow*/
	if (cpu_freq_suspend_in == 2)
		return -1;
#endif
	/*Set the voltage for the GP domain. */
	if (freq > org_cpu_rate) {
		if (low_bus_freq_mode || audio_bus_freq_mode)
			set_high_bus_freq(0);
		ret = regulator_set_voltage(cpu_regulator, gp_volt,
					    gp_volt);
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
		ret = regulator_set_voltage(cpu_regulator, gp_volt,
					    gp_volt);
		if (ret < 0) {
			printk(KERN_DEBUG "COULD NOT SET GP VOLTAGE!!!!\n");
			return ret;
		}
		if (low_freq_bus_used() &&
			!(low_bus_freq_mode || audio_bus_freq_mode))
			set_low_bus_freq();
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
		struct cpufreq_freqs freqs;

		freqs.old = policy->cur;
		freqs.new = clk_get_rate(cpu_clk) / 1000;
		freqs.cpu = policy->cpu;
		freqs.flags = 0;
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

		pr_debug("DVFS core is active, cannot change FREQ using CPUFREQ\n");
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
	#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	mutex_lock(&set_cpufreq_lock);
	ret = set_cpu_freq(freq_Hz);
	mutex_unlock(&set_cpufreq_lock);
	#else
	ret = set_cpu_freq(freq_Hz);
	#endif
#ifdef CONFIG_SMP
	/* Loops per jiffy is not updated by the CPUFREQ driver for SMP systems.
	  * So update it for all CPUs.
	  */

	for_each_cpu(i, policy->cpus)
		per_cpu(cpu_data, i).loops_per_jiffy =
		cpufreq_scale(per_cpu(cpu_data, i).loops_per_jiffy,
					freqs.old, freqs.new);
	/* Update global loops_per_jiffy to cpu0's loops_per_jiffy,
	 * as all CPUs are running at same freq */
	loops_per_jiffy = per_cpu(cpu_data, 0).loops_per_jiffy;
#endif
	for (i = 0; i < num_cpus; i++) {
		freqs.cpu = i;
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}

	return ret;
}
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
void mxc_cpufreq_suspend(void)
{
	mutex_lock(&set_cpufreq_lock);
	pre_suspend_rate = clk_get_rate(cpu_clk);
	/*set flag and raise up cpu frequency if needed*/
	cpu_freq_suspend_in = 1;
	if (pre_suspend_rate != (imx_freq_table[0].frequency * 1000))
			set_cpu_freq(imx_freq_table[0].frequency * 1000);
	cpu_freq_suspend_in = 2;
	mutex_unlock(&set_cpufreq_lock);

}

void mxc_cpufreq_resume(void)
{
	mutex_lock(&set_cpufreq_lock);
	cpu_freq_suspend_in = 1;
	if (clk_get_rate(cpu_clk) != pre_suspend_rate)
		set_cpu_freq(pre_suspend_rate);
	cpu_freq_suspend_in = 0;
	mutex_unlock(&set_cpufreq_lock);
}


#else
static int mxc_cpufreq_suspend(struct cpufreq_policy *policy)
{
	pre_suspend_rate = clk_get_rate(cpu_clk);
	if (pre_suspend_rate != (imx_freq_table[0].frequency * 1000))
		set_cpu_freq(imx_freq_table[0].frequency * 1000);

	return 0;
}

static int mxc_cpufreq_resume(struct cpufreq_policy *policy)
{
	if (clk_get_rate(cpu_clk) != pre_suspend_rate)
		set_cpu_freq(pre_suspend_rate);
	return 0;
}
#endif

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
#ifndef CONFIG_MX6_INTER_LDO_BYPASS
	.suspend = mxc_cpufreq_suspend,
	.resume = mxc_cpufreq_resume,
#endif
	.name = "imx",
};

extern void mx6_cpu_regulator_init(void);
static int __init mxc_cpufreq_driver_init(void)
{
	#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	mx6_cpu_regulator_init();
	mutex_init(&set_cpufreq_lock);
	#endif
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
