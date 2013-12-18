/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/suspend.h>
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
#ifdef CONFIG_ARCH_MX5
int cpufreq_suspended;
#endif
static bool cpufreq_suspend;
struct mutex set_cpufreq_lock;

extern struct regulator *cpu_regulator;
extern struct regulator *soc_regulator;
extern struct regulator *pu_regulator;
extern int dvfs_core_is_active;
extern struct cpu_op *(*get_cpu_op)(int *op);
extern void bus_freq_update(struct clk *clk, bool flag);
extern void mx6_arm_regulator_bypass(void);
extern void mx6_soc_pu_regulator_bypass(void);

int set_cpu_freq(int freq)
{
	int i, ret = 0;
	int org_cpu_rate;
	int gp_volt = 0, org_gp_volt = 0;
	int soc_volt = 0, org_soc_volt = 0;
	int pu_volt = 0, org_pu_volt = 0;

	org_cpu_rate = clk_get_rate(cpu_clk);
	if (org_cpu_rate == freq)
		return ret;

	for (i = 0; i < cpu_op_nr; i++) {
		if (freq == cpu_op_tbl[i].cpu_rate) {
			gp_volt = cpu_op_tbl[i].cpu_voltage;
			soc_volt = cpu_op_tbl[i].soc_voltage;
			pu_volt = cpu_op_tbl[i].pu_voltage;
		}
		if (org_cpu_rate == cpu_op_tbl[i].cpu_rate) {
			org_gp_volt = cpu_op_tbl[i].cpu_voltage;
			org_soc_volt = cpu_op_tbl[i].soc_voltage;
			org_pu_volt = cpu_op_tbl[i].pu_voltage;
		}
	}
	if (gp_volt == 0)
		return ret;
	/*Set the voltage for the GP domain. */
	if (freq > org_cpu_rate) {
		/* Check if the bus freq needs to be increased first */
		bus_freq_update(cpu_clk, true);

		if (!IS_ERR(soc_regulator)) {
			ret = regulator_set_voltage(soc_regulator, soc_volt,
							soc_volt);
			if (ret < 0) {
				printk(KERN_ERR
					"COULD NOT SET SOC VOLTAGE!!!!\n");
				goto err1;
			}
		}
		/*if pu_regulator is enabled, it will be tracked with VDDARM*/
		if (!IS_ERR(pu_regulator) &&
			regulator_is_enabled(pu_regulator)) {
			ret = regulator_set_voltage(pu_regulator, pu_volt,
							pu_volt);
			if (ret < 0) {
				printk(KERN_ERR
					"COULD NOT SET PU VOLTAGE!!!!\n");
				goto err2;
			}
			/* 
			 * Force sync setting PU since regulator mabye maintain
			 * the old volatage setting before disable PU, then will
			 * miss this time setting if the two setting is same.
			 * So do force sync again here. 
			 */
			regulator_sync_voltage(pu_regulator);
		}
		ret = regulator_set_voltage(cpu_regulator, gp_volt,
					    gp_volt);
		if (ret < 0) {
			printk(KERN_ERR "COULD NOT SET GP VOLTAGE!!!!\n");
			goto err3;
		}
	}
	ret = clk_set_rate(cpu_clk, freq);
	if (ret != 0) {
		printk(KERN_ERR "cannot set CPU clock rate\n");
		goto err4;
	}

	if (freq < org_cpu_rate) {
		ret = regulator_set_voltage(cpu_regulator, gp_volt,
					    gp_volt);
		if (ret < 0) {
			printk(KERN_ERR "COULD NOT SET GP VOLTAGE!!!!\n");
			goto err5;
		}
		if (!IS_ERR(soc_regulator)) {
			ret = regulator_set_voltage(soc_regulator, soc_volt,
							soc_volt);
			if (ret < 0) {
				printk(KERN_ERR
					"COULD NOT SET SOC VOLTAGE BACK!!!!\n");
				goto err6;
			}
		}
		/*if pu_regulator is enabled, it will be tracked with VDDARM*/
		if (!IS_ERR(pu_regulator) &&
			regulator_is_enabled(pu_regulator)) {
			ret = regulator_set_voltage(pu_regulator, pu_volt,
							pu_volt);
			if (ret < 0) {
				printk(KERN_ERR
					"COULD NOT SET PU VOLTAGE!!!!\n");
				goto err7;
			}
			/* 
			 * Force sync setting PU since regulator mabye maintain
			 * the old volatage setting before disable PU, then will
			 * miss this time setting if the two setting is same.
			 * So do force sync again here. 
			 */
			regulator_sync_voltage(pu_regulator);
		}
		/* Check if the bus freq can be decreased.*/
		bus_freq_update(cpu_clk, false);
	}

	return ret;
	/*Restore back if any regulator or clock rate set fail.*/
err7:
	ret = regulator_set_voltage(soc_regulator, org_soc_volt,
							org_soc_volt);
	if (ret < 0) {
		printk(KERN_ERR "COULD NOT RESTORE SOC VOLTAGE!!!!\n");
		goto err7;
	}

err6:
	ret = regulator_set_voltage(cpu_regulator, org_gp_volt, org_gp_volt);
	if (ret < 0) {
		printk(KERN_ERR "COULD NOT RESTORE GP VOLTAGE!!!!\n");
		goto err6;
	}

err5:
	ret = clk_set_rate(cpu_clk, org_cpu_rate);
	if (ret != 0) {
		printk(KERN_ERR "cannot restore CPU clock rate\n");
		goto err5;
	}
	return -1;

err4:
	ret = regulator_set_voltage(cpu_regulator, org_gp_volt, org_gp_volt);
	if (ret < 0) {
		printk(KERN_ERR "COULD NOT RESTORE GP VOLTAGE!!!!\n");
		goto err4;
	}

err3:
	if (!IS_ERR(pu_regulator) &&
		regulator_is_enabled(pu_regulator)) {
		ret = regulator_set_voltage(pu_regulator, org_pu_volt, org_pu_volt);
		if (ret < 0) {
			printk(KERN_ERR "COULD NOT RESTORE PU VOLTAGE!!!!\n");
			goto err3;
		}
	}
err2:
	ret = regulator_set_voltage(soc_regulator, org_soc_volt,
							org_soc_volt);
	if (ret < 0) {
		printk(KERN_ERR "COULD NOT RESTORE SOC VOLTAGE!!!!\n");
		goto err2;
	}
err1:
	return -1;

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

	if (dvfs_core_is_active
#ifdef CONFIG_ARCH_MX5
		|| cpufreq_suspended
#endif
		) {
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

	mutex_lock(&set_cpufreq_lock);
	if (cpufreq_suspend) {
		mutex_unlock(&set_cpufreq_lock);
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
	if (ret) {
		/*restore cpufreq and tell cpufreq core if set fail*/
		freqs.old = clk_get_rate(cpu_clk) / 1000;
		freqs.new = freqs.old;
		freqs.cpu = policy->cpu;
		freqs.flags = 0;
		for (i = 0; i < num_cpus; i++) {
			freqs.cpu = i;
			cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		}
		goto  Set_finish;
	}
#ifdef CONFIG_SMP
	/* Loops per jiffy is not updated by the CPUFREQ driver for SMP systems.
	  * So update it for all CPUs.
	  */

	for_each_possible_cpu(i)
		per_cpu(cpu_data, i).loops_per_jiffy =
		cpufreq_scale(per_cpu(cpu_data, i).loops_per_jiffy,
					freqs.old, freqs.new);
	/* Update global loops_per_jiffy to cpu0's loops_per_jiffy,
	 * as all CPUs are running at same freq */
	loops_per_jiffy = per_cpu(cpu_data, 0).loops_per_jiffy;
#endif
Set_finish:
	for (i = 0; i < num_cpus; i++) {
		freqs.cpu = i;
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
	mutex_unlock(&set_cpufreq_lock);

	return ret;
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

static struct freq_attr *imx_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};
static struct cpufreq_driver mxc_driver = {
	.flags = CPUFREQ_STICKY,
	.verify = mxc_verify_speed,
	.target = mxc_set_target,
	.get = mxc_get_speed,
	.init = mxc_cpufreq_init,
	.exit = mxc_cpufreq_exit,
	.name = "imx",
	.attr = imx_cpufreq_attr,
};

static int cpufreq_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	unsigned int i;
	int num_cpus;
	int ret;
	struct cpufreq_freqs freqs;

	num_cpus = num_possible_cpus();
	mutex_lock(&set_cpufreq_lock);
	if (event == PM_SUSPEND_PREPARE) {
		pre_suspend_rate = clk_get_rate(cpu_clk);
		if (pre_suspend_rate != (imx_freq_table[0].frequency * 1000)) {
			/*notify cpufreq core will raise up cpufreq to highest*/
			freqs.old = pre_suspend_rate / 1000;
			freqs.new = imx_freq_table[0].frequency;
			freqs.flags = 0;
			for (i = 0; i < num_cpus; i++) {
				freqs.cpu = i;
				cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
			}
			ret = set_cpu_freq(imx_freq_table[0].frequency * 1000);
			/*restore cpufreq and tell cpufreq core if set fail*/
			if (ret) {
				freqs.old =  clk_get_rate(cpu_clk)/1000;
				freqs.new = freqs.old;
				freqs.flags = 0;
				for (i = 0; i < num_cpus; i++) {
					freqs.cpu = i;
					cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
				}
				goto Notify_finish;/*if update freq error,return*/
			}
#ifdef CONFIG_SMP
			for_each_possible_cpu(i)
				per_cpu(cpu_data, i).loops_per_jiffy =
					cpufreq_scale(per_cpu(cpu_data, i).loops_per_jiffy,
					pre_suspend_rate / 1000, imx_freq_table[0].frequency);
			loops_per_jiffy = per_cpu(cpu_data, 0).loops_per_jiffy;
#else
			loops_per_jiffy = cpufreq_scale(loops_per_jiffy,
				pre_suspend_rate / 1000, imx_freq_table[0].frequency);
#endif
			for (i = 0; i < num_cpus; i++) {
				freqs.cpu = i;
				cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
			}
		}
		cpufreq_suspend = true;
	} else if (event == PM_POST_SUSPEND) {
		if (clk_get_rate(cpu_clk) != pre_suspend_rate) {
			/*notify cpufreq core will restore rate before suspend*/
			freqs.old = imx_freq_table[0].frequency;
			freqs.new = pre_suspend_rate / 1000;
			freqs.flags = 0;
			for (i = 0; i < num_cpus; i++) {
				freqs.cpu = i;
				cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
			}
			ret = set_cpu_freq(pre_suspend_rate);
			/*restore cpufreq and tell cpufreq core if set fail*/
			if (ret) {
				freqs.old =  clk_get_rate(cpu_clk)/1000;
				freqs.new = freqs.old;
				freqs.flags = 0;
				for (i = 0; i < num_cpus; i++) {
					freqs.cpu = i;
					cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
				}
				goto Notify_finish;/*if update freq error,return*/
			}
#ifdef CONFIG_SMP
			for_each_possible_cpu(i)
				per_cpu(cpu_data, i).loops_per_jiffy =
					cpufreq_scale(per_cpu(cpu_data, i).loops_per_jiffy,
					imx_freq_table[0].frequency, pre_suspend_rate / 1000);
			loops_per_jiffy = per_cpu(cpu_data, 0).loops_per_jiffy;
#else
			loops_per_jiffy = cpufreq_scale(loops_per_jiffy,
				imx_freq_table[0].frequency, pre_suspend_rate / 1000);
#endif
			for (i = 0; i < num_cpus; i++) {
				freqs.cpu = i;
				cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
			}
		}
		cpufreq_suspend = false;
	}
	mutex_unlock(&set_cpufreq_lock);
	return NOTIFY_OK;

Notify_finish:
	for (i = 0; i < num_cpus; i++) {
		freqs.cpu = i;
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
	mutex_unlock(&set_cpufreq_lock);
	return NOTIFY_OK;
}
static struct notifier_block imx_cpufreq_pm_notifier = {
	.notifier_call = cpufreq_pm_notify,
};
#ifdef CONFIG_ARCH_MX6
extern void mx6_cpu_regulator_init(void);
#endif
static int __init mxc_cpufreq_driver_init(void)
{
#ifdef CONFIG_ARCH_MX6
	mx6_cpu_regulator_init();
#endif
	mutex_init(&set_cpufreq_lock);
	register_pm_notifier(&imx_cpufreq_pm_notifier);
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
