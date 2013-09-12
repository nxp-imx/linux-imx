/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/busfreq-imx6.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/opp.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/suspend.h>

static struct regulator *arm_reg;
static struct regulator *pu_reg;
static struct regulator *soc_reg;

static struct clk *arm_clk;
static struct clk *pll1_sys_clk;
static struct clk *pll1_sw_clk;
static struct clk *step_clk;
static struct clk *pll2_pfd2_396m_clk;

static struct device *cpu_dev;
static struct cpufreq_frequency_table *freq_table;
static unsigned int transition_latency;
static struct mutex set_cpufreq_lock;

struct soc_opp {
	u32 arm_freq;
	u32 soc_volt;
};

static struct soc_opp *imx6_soc_opp;
static u32 soc_opp_count;

static int imx6_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, freq_table);
}

static unsigned int imx6_get_speed(unsigned int cpu)
{
	return clk_get_rate(arm_clk) / 1000;
}

static int imx6_set_target(struct cpufreq_policy *policy,
			    unsigned int target_freq, unsigned int relation)
{
	struct cpufreq_freqs freqs;
	struct opp *opp;
	unsigned long freq_hz, volt, volt_old;
	unsigned int index, soc_opp_index = 0;
	int ret;

	mutex_lock(&set_cpufreq_lock);

	ret = cpufreq_frequency_table_target(policy, freq_table, target_freq,
					     relation, &index);
	if (ret) {
		dev_err(cpu_dev, "failed to match target frequency %d: %d\n",
			target_freq, ret);
		mutex_unlock(&set_cpufreq_lock);
		return ret;
	}

	freqs.new = freq_table[index].frequency;
	freq_hz = freqs.new * 1000;
	freqs.old = clk_get_rate(arm_clk) / 1000;

	if (freqs.old == freqs.new) {
		mutex_unlock(&set_cpufreq_lock);
		return 0;
	}

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);

	rcu_read_lock();
	opp = opp_find_freq_ceil(cpu_dev, &freq_hz);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		dev_err(cpu_dev, "failed to find OPP for %ld\n", freq_hz);
		mutex_unlock(&set_cpufreq_lock);
		return PTR_ERR(opp);
	}

	volt = opp_get_voltage(opp);
	rcu_read_unlock();
	volt_old = regulator_get_voltage(arm_reg);

	/* Find the matching VDDSOC/VDDPU operating voltage */
	while (soc_opp_index < soc_opp_count) {
		if (freqs.new == imx6_soc_opp[soc_opp_index].arm_freq)
			break;
		soc_opp_index++;
	}
	if (soc_opp_index >= soc_opp_count) {
		dev_err(cpu_dev,
			"Cannot find matching imx6_soc_opp voltage\n");
			mutex_unlock(&set_cpufreq_lock);
			return -EINVAL;
	}

	dev_dbg(cpu_dev, "%u MHz, %ld mV --> %u MHz, %ld mV\n",
		freqs.old / 1000, volt_old / 1000,
		freqs.new / 1000, volt / 1000);

	/*
	  * CPU freq is increasing, so need to ensure
	  * that bus frequency is increased too.
	  */
	if (freqs.old == freq_table[0].frequency)
		request_bus_freq(BUS_FREQ_HIGH);

	/* scaling up?  scale voltage before frequency */
	if (freqs.new > freqs.old) {
		if (regulator_is_enabled(pu_reg)) {
			ret = regulator_set_voltage_tol(pu_reg,
					imx6_soc_opp[soc_opp_index].soc_volt,
					0);
			if (ret) {
				dev_err(cpu_dev,
					"failed to scale vddpu up: %d\n", ret);
				goto err1;
			}
		}
		ret = regulator_set_voltage_tol(soc_reg,
				imx6_soc_opp[soc_opp_index].soc_volt, 0);
		if (ret) {
			dev_err(cpu_dev,
				"failed to scale vddsoc up: %d\n", ret);
			goto err1;
		}
		ret = regulator_set_voltage_tol(arm_reg, volt, 0);
		if (ret) {
			dev_err(cpu_dev,
				"failed to scale vddarm up: %d\n", ret);
			goto err1;
		}
	}

	/*
	 * The setpoints are selected per PLL/PDF frequencies, so we need to
	 * reprogram PLL for frequency scaling.  The procedure of reprogramming
	 * PLL1 is as below.
	 *
	 *  - Enable pll2_pfd2_396m_clk and reparent pll1_sw_clk to it
	 *  - Reprogram pll1_sys_clk and reparent pll1_sw_clk back to it
	 *  - Disable pll2_pfd2_396m_clk
	 */
	clk_set_parent(step_clk, pll2_pfd2_396m_clk);
	clk_set_parent(pll1_sw_clk, step_clk);
	if (freq_hz > clk_get_rate(pll2_pfd2_396m_clk)) {
		clk_set_rate(pll1_sys_clk, freqs.new * 1000);
		clk_set_parent(pll1_sw_clk, pll1_sys_clk);
	}

	/* Ensure the arm clock divider is what we expect */
	ret = clk_set_rate(arm_clk, freqs.new * 1000);
	if (ret) {
		dev_err(cpu_dev, "failed to set clock rate: %d\n", ret);
		goto err1;
	}

	/* scaling down?  scale voltage after frequency */
	if (freqs.new < freqs.old) {
		ret = regulator_set_voltage_tol(arm_reg, volt, 0);
		if (ret) {
			dev_warn(cpu_dev,
				 "failed to scale vddarm down: %d\n", ret);
			goto err1;
		}

		ret = regulator_set_voltage_tol(soc_reg,
				imx6_soc_opp[soc_opp_index].soc_volt, 0);
		if (ret) {
			dev_err(cpu_dev,
				"failed to scale vddsoc down: %d\n", ret);
			goto err1;
		}

		if (regulator_is_enabled(pu_reg)) {
			ret = regulator_set_voltage_tol(pu_reg,
					imx6_soc_opp[soc_opp_index].soc_volt,
					0);
			if (ret) {
				dev_err(cpu_dev,
				"failed to scale vddpu down: %d\n", ret);
				goto err1;
			}
		}
	}
	/*
	  * If CPU is dropped to the lowest level, release the need
	  * for a high bus frequency.
	  */
	if (freqs.new == freq_table[0].frequency)
		release_bus_freq(BUS_FREQ_HIGH);

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);

	mutex_unlock(&set_cpufreq_lock);
	return 0;
err1:
	mutex_unlock(&set_cpufreq_lock);
	return -1;
}

static int imx6_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret;

	ret = cpufreq_frequency_table_cpuinfo(policy, freq_table);
	if (ret) {
		dev_err(cpu_dev, "invalid frequency table: %d\n", ret);
		return ret;
	}

	policy->cpuinfo.transition_latency = transition_latency;
	policy->cur = clk_get_rate(arm_clk) / 1000;
	cpumask_setall(policy->cpus);
	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);

	if (policy->cur > freq_table[0].frequency)
		request_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

static int imx6_cpufreq_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);
	return 0;
}

static struct freq_attr *imx6_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver imx6_cpufreq_driver = {
	.verify = imx6_verify_speed,
	.target = imx6_set_target,
	.get = imx6_get_speed,
	.init = imx6_cpufreq_init,
	.exit = imx6_cpufreq_exit,
	.name = "imx6-cpufreq",
	.attr = imx6_cpufreq_attr,
};

static int imx6_cpufreq_pm_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	struct cpufreq_policy *data = cpufreq_cpu_get(0);
	static u32 cpufreq_policy_min_pre_suspend;

	/*
	 * During suspend/resume, When cpufreq driver try to increase
	 * voltage/freq, it needs to control I2C/SPI to communicate
	 * with external PMIC to adjust voltage, but these I2C/SPI
	 * devices may be already suspended, to avoid such scenario,
	 * we just increase cpufreq to highest setpoint before suspend.
	 */
	switch (event) {
	case PM_SUSPEND_PREPARE:
		cpufreq_policy_min_pre_suspend = data->user_policy.min;
		data->user_policy.min = data->user_policy.max;
		break;
	case PM_POST_SUSPEND:
		data->user_policy.min = cpufreq_policy_min_pre_suspend;
		break;
	default:
		break;
	}

	cpufreq_update_policy(0);

	return NOTIFY_OK;
}

static struct notifier_block imx6_cpufreq_pm_notifier = {
	.notifier_call = imx6_cpufreq_pm_notify,
};

static int imx6_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np;
	struct opp *opp;
	unsigned long min_volt = 0, max_volt = 0;
	int num, ret;
	const struct property *prop;
	const __be32 *val;
	u32 nr, i;

	cpu_dev = &pdev->dev;

	np = of_find_node_by_path("/cpus/cpu@0");
	if (!np) {
		dev_err(cpu_dev, "failed to find cpu0 node\n");
		return -ENOENT;
	}

	cpu_dev->of_node = np;

	arm_clk = devm_clk_get(cpu_dev, "arm");
	pll1_sys_clk = devm_clk_get(cpu_dev, "pll1_sys");
	pll1_sw_clk = devm_clk_get(cpu_dev, "pll1_sw");
	step_clk = devm_clk_get(cpu_dev, "step");
	pll2_pfd2_396m_clk = devm_clk_get(cpu_dev, "pll2_pfd2_396m");
	if (IS_ERR(arm_clk) || IS_ERR(pll1_sys_clk) || IS_ERR(pll1_sw_clk) ||
	    IS_ERR(step_clk) || IS_ERR(pll2_pfd2_396m_clk)) {
		dev_err(cpu_dev, "failed to get clocks\n");
		ret = -ENOENT;
		goto put_node;
	}

	arm_reg = devm_regulator_get(cpu_dev, "arm");
	pu_reg = devm_regulator_get(cpu_dev, "pu");
	soc_reg = devm_regulator_get(cpu_dev, "soc");
	if (IS_ERR(arm_reg) || IS_ERR(pu_reg) || IS_ERR(soc_reg)) {
		dev_err(cpu_dev, "failed to get regulators\n");
		ret = -ENOENT;
		goto put_node;
	}

	/* We expect an OPP table supplied by platform */
	num = opp_get_opp_count(cpu_dev);
	if (num < 0) {
		ret = num;
		dev_err(cpu_dev, "no OPP table is found: %d\n", ret);
		goto put_node;
	}

	ret = opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto put_node;
	}

	prop = of_find_property(np, "fsl,soc-operating-points", NULL);
	if (!prop) {
		dev_err(cpu_dev,
			"fsl,soc-operating-points node not found\n");
		goto free_freq_table;
	}
	if (!prop->value) {
		dev_err(cpu_dev,
			"No entries in fsl-soc-operating-points node.\n");
		goto free_freq_table;
	}

	/*
	 * Each OPP is a set of tuples consisting of frequency and
	 * voltage like <freq-kHz vol-uV>.
	 */
	nr = prop->length / sizeof(u32);
	if (nr % 2) {
		dev_err(cpu_dev, "Invalid fsl-soc-operating-points  list\n");
		goto free_freq_table;
	}

	/* Get the VDDSOC/VDDPU voltages that need to track the CPU voltages. */
	imx6_soc_opp = devm_kzalloc(cpu_dev,
					sizeof(struct soc_opp) * (nr / 2),
					GFP_KERNEL);

	if (imx6_soc_opp == NULL) {
		dev_err(cpu_dev, "No Memory for VDDSOC/PU table\n");
		goto free_freq_table;
	}

	rcu_read_lock();
	val = prop->value;

	for (i = 0; i < nr / 2; i++) {
		unsigned long freq = be32_to_cpup(val++);
		unsigned long volt = be32_to_cpup(val++);

		if (i == 0)
			min_volt = max_volt = volt;
		if (volt < min_volt)
			min_volt = volt;
		if (volt > max_volt)
			max_volt = volt;
		opp = opp_find_freq_exact(cpu_dev,
					  freq * 1000, true);
		if (IS_ERR(opp)) {
			opp = opp_find_freq_exact(cpu_dev,
						  freq * 1000, false);
			if (IS_ERR(opp)) {
				dev_err(cpu_dev,
					"freq in soc-operating-points does not \
					match cpufreq table\n");
				rcu_read_unlock();
				goto free_freq_table;
			}
		}
		imx6_soc_opp[i].arm_freq = freq;
		imx6_soc_opp[i].soc_volt = volt;
		soc_opp_count++;
	}
	rcu_read_unlock();

	if (of_property_read_u32(np, "clock-latency", &transition_latency))
		transition_latency = CPUFREQ_ETERNAL;

	/*
	  * Calculate the ramp time for max voltage change in the
	  * VDDSOC and VDDPU regulators.
	  */
	ret = regulator_set_voltage_time(soc_reg, min_volt, max_volt);
	if (ret > 0)
		transition_latency += ret * 1000;

	ret = regulator_set_voltage_time(pu_reg, min_volt, max_volt);
	if (ret > 0)
		transition_latency += ret * 1000;

	/*
	 * OPP is maintained in order of increasing frequency, and
	 * freq_table initialised from OPP is therefore sorted in the
	 * same order.
	 */
	rcu_read_lock();
	opp = opp_find_freq_exact(cpu_dev,
				  freq_table[0].frequency * 1000, true);
	min_volt = opp_get_voltage(opp);
	opp = opp_find_freq_exact(cpu_dev,
				  freq_table[num - 1].frequency * 1000, true);
	max_volt = opp_get_voltage(opp);
	rcu_read_unlock();
	ret = regulator_set_voltage_time(arm_reg, min_volt, max_volt);
	if (ret > 0)
		transition_latency += ret * 1000;

	mutex_init(&set_cpufreq_lock);
	register_pm_notifier(&imx6_cpufreq_pm_notifier);

	ret = cpufreq_register_driver(&imx6_cpufreq_driver);
	if (ret) {
		dev_err(cpu_dev, "failed register driver: %d\n", ret);
		goto free_freq_table;
	}

	of_node_put(np);
	return 0;

free_freq_table:
	opp_free_cpufreq_table(cpu_dev, &freq_table);
put_node:
	of_node_put(np);
	return ret;
}

static int imx6_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&imx6_cpufreq_driver);
	opp_free_cpufreq_table(cpu_dev, &freq_table);

	return 0;
}

static struct platform_driver imx6_cpufreq_platdrv = {
	.driver = {
		.name	= "imx6-cpufreq",
		.owner	= THIS_MODULE,
	},
	.probe		= imx6_cpufreq_probe,
	.remove		= imx6_cpufreq_remove,
};
module_platform_driver(imx6_cpufreq_platdrv);

MODULE_AUTHOR("Shawn Guo <shawn.guo@linaro.org>");
MODULE_DESCRIPTION("Freescale i.MX6 cpufreq driver");
MODULE_LICENSE("GPL");
