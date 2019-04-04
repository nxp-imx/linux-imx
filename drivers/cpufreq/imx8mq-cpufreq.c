/*
 * Copyright (C) 2017-2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpu_cooling.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/device_cooling.h>

#define DC_VOLTAGE_MIN		900000
#define DC_VOLTAGE_MAX		1000000

static struct device *cpu_dev;
static bool free_opp;
static struct cpufreq_frequency_table *freq_table;
static unsigned int transition_latency;
static struct clk *a53_clk;
static struct clk *arm_a53_src_clk;
static struct clk *arm_pll_clk;
static struct clk *arm_pll_out_clk;
static struct clk *sys1_pll_800m_clk;
struct thermal_cooling_device *cdev;
static struct regulator *dc_reg;
static struct regulator *arm_reg;

static int imx8mq_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct dev_pm_opp *opp;
	unsigned long freq_hz, volt;
	unsigned int old_freq, new_freq;
	int ret;

	new_freq = freq_table[index].frequency;
	freq_hz = new_freq * 1000;
	old_freq = policy->cur;

	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_hz);
	if (IS_ERR(opp)) {
		dev_err(cpu_dev, "failed to find OPP for %ld\n", freq_hz);
		return PTR_ERR(opp);
	}
	volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	dev_dbg(cpu_dev, "%u MHz --> %u MHz\n",
		old_freq / 1000, new_freq / 1000);

	if (new_freq == policy->max) {
		if (!IS_ERR(dc_reg)) {
			ret = regulator_set_voltage_tol(dc_reg, DC_VOLTAGE_MAX, 0);
			if (ret) {
				dev_err(cpu_dev, "failed to scale dc_reg up: %d\n", ret);
				return ret;
			}
		}
	}

	if (new_freq > old_freq) {
		if (!IS_ERR(arm_reg)) {
			ret = regulator_set_voltage_tol(arm_reg, volt, 0);
			if (ret) {
				dev_err(cpu_dev, "failed to scale arm_reg up: %d\n", ret);
				return ret;
			}
		}
	}

	clk_set_parent(arm_a53_src_clk, sys1_pll_800m_clk);
	clk_set_rate(arm_pll_clk, new_freq * 1000);
	clk_set_parent(arm_a53_src_clk, arm_pll_out_clk);

	if (old_freq == policy->max) {
		if (!IS_ERR(dc_reg)) {
			ret = regulator_set_voltage_tol(dc_reg, DC_VOLTAGE_MIN, 0);
			if (ret) {
				dev_err(cpu_dev, "failed to scale dc_reg down: %d\n", ret);
				return ret;
			}
		}
	}

	if (new_freq < old_freq) {
		if (!IS_ERR(arm_reg)) {
			ret = regulator_set_voltage_tol(arm_reg, volt, 0);
			if (ret) {
				dev_err(cpu_dev, "failed to scale arm_reg down: %d\n", ret);
				return ret;
			}
		}
	}

	/* Ensure the arm clock divider is what we expect */
	ret = clk_set_rate(a53_clk, new_freq * 1000);
	if (ret)
		dev_err(cpu_dev, "failed to set clock rate: %d\n", ret);

	return ret;
}

static void imx8mq_cpufreq_ready(struct cpufreq_policy *policy)
{
	struct device_node *np = of_get_cpu_node(policy->cpu, NULL);

	if (of_find_property(np, "#cooling-cells", NULL)) {
		cdev = of_cpufreq_cooling_register(np, policy);

		if (IS_ERR(cdev) && PTR_ERR(cdev) != -ENOSYS) {
			pr_err("cpu%d is not running as cooling device: %ld\n",
					policy->cpu, PTR_ERR(cdev));

			cdev = NULL;
		}
	}

	of_node_put(np);
}

#define cpu_cooling_core_mask ((1 << 2) | (1 << 3))   /* offline cpu2 and cpu3 if needed*/
static int thermal_hot_pm_notify(struct notifier_block *nb, unsigned long event,
       void *dummy)
{
	static unsigned long prev_event = 0xffffffff;
	struct device *cpu_dev = NULL;
	static int cpus_offlined = 0;
	int i = 0, ret = 0;

	if (event == prev_event)
		return NOTIFY_OK;

	prev_event = event;

	switch (event) {
	case 0: /* default state, no trip point reached*/
	case 1: /* trip1 temperature are lower than trip2, we can
		   online the cpu2 and cpu3 to get better performance */
		for (i = 0; i < num_possible_cpus(); i++) {
			if (!(cpu_cooling_core_mask & BIT(i)))
				continue;
			if (!(cpus_offlined & BIT(i)))
				continue;
			cpus_offlined &= ~BIT(i);
			pr_info("Allow Online CPU%d, devfreq state: %d\n",
					i, event);

			lock_device_hotplug();
			if (cpu_online(i)) {
				unlock_device_hotplug();
				continue;
			}
			cpu_dev = get_cpu_device(i);
			ret = device_online(cpu_dev);
			if (ret)
				pr_err("Error %d online core %d\n",
						ret, i);
			unlock_device_hotplug();
		}
		break;
	case 2: /* rise above trip2 temperature, offline cpu2 and cpu3 to
		   to limit the max online cpu cores */
		for (i = num_possible_cpus() - 1; i >= 0; i--) {
			if (!(cpu_cooling_core_mask & BIT(i)))
				continue;
			if (cpus_offlined & BIT(i) && !cpu_online(i))
				continue;
			pr_info("Set Offline: CPU%d, devfreq state: %d\n",
					i, event);
			lock_device_hotplug();
			if (cpu_online(i)) {
				cpu_dev = get_cpu_device(i);
				ret = device_offline(cpu_dev);
				if (ret < 0)
					pr_err("Error %d offline core %d\n",
					       ret, i);
			}
			unlock_device_hotplug();
			cpus_offlined |= BIT(i);
		}
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block thermal_hot_pm_notifier =
{
	.notifier_call = thermal_hot_pm_notify,
};

static int imx8mq_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret;

	policy->clk = a53_clk;
	policy->cur = clk_get_rate(a53_clk) / 1000;

	ret = cpufreq_generic_init(policy, freq_table, transition_latency);
	if (ret) {
		dev_err(cpu_dev, "imx8mq cpufreq init failed!\n");
		return ret;
	}

	policy->suspend_freq = policy->max;

	return 0;
}

static struct cpufreq_driver imx8mq_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = imx8mq_set_target,
	.get = cpufreq_generic_get,
	.init = imx8mq_cpufreq_init,
	.name = "imx8mq-cpufreq",
	.ready = imx8mq_cpufreq_ready,
	.attr = cpufreq_generic_attr,
#ifdef CONFIG_PM
	.suspend = cpufreq_generic_suspend,
#endif
};

static int imx8mq_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np;
	int ret, num;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev) {
		pr_err("failed to get cpu0 device\n");
		return -ENODEV;
	}

	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		dev_err(cpu_dev, "failed to find cpu0 node\n");
		return -ENOENT;
	}

	a53_clk = clk_get(cpu_dev, "a53");
	arm_a53_src_clk = clk_get(cpu_dev, "arm_a53_src");
	arm_pll_clk = clk_get(cpu_dev, "arm_pll");
	arm_pll_out_clk = clk_get(cpu_dev, "arm_pll_out");
	sys1_pll_800m_clk = clk_get(cpu_dev, "sys1_pll_800m");
	if (IS_ERR(a53_clk) || IS_ERR(arm_a53_src_clk)
		|| IS_ERR(arm_pll_out_clk) || IS_ERR(arm_pll_clk)
		|| IS_ERR(sys1_pll_800m_clk)) {
		dev_err(cpu_dev, "failed to get clocks\n");
		ret = -ENOENT;
		goto put_clk;
	}

	dc_reg = regulator_get_optional(cpu_dev, "dc");
	arm_reg = regulator_get_optional(cpu_dev, "arm");

	/*
	 * We expect an OPP table supplied by platform.
	 * Just, incase the platform did not supply the OPP
	 * table, it will try to get it.
	 */
	num = dev_pm_opp_get_opp_count(cpu_dev);
	if (num < 0) {
		ret = dev_pm_opp_of_add_table(cpu_dev);
		if (ret < 0) {
			dev_err(cpu_dev, "failed to init OPP table: %d\n", ret);
			goto put_clk;
		}
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto out_free_opp;
	}

	if (of_property_read_u32(np, "clock-latency", &transition_latency))
		transition_latency = CPUFREQ_ETERNAL;

	ret = cpufreq_register_driver(&imx8mq_cpufreq_driver);
	if (ret) {
		dev_err(cpu_dev, "failed register driver: %d\n", ret);
		goto free_freq_table;
	}

	of_node_put(np);
	dev_info(cpu_dev, "registered imx8mq-cpufreq\n");

	register_devfreq_cooling_notifier(&thermal_hot_pm_notifier);

	return 0;

free_freq_table:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
out_free_opp:
	dev_pm_opp_of_remove_table(cpu_dev);
put_clk:
	if (!IS_ERR(a53_clk))
		clk_put(a53_clk);
	if (!IS_ERR(arm_a53_src_clk))
		clk_put(arm_a53_src_clk);
	if (!IS_ERR(arm_pll_clk))
		clk_put(arm_pll_clk);
	if (!IS_ERR(arm_pll_out_clk))
		clk_put(arm_pll_out_clk);
	if (!IS_ERR(sys1_pll_800m_clk))
		clk_put(sys1_pll_800m_clk);
	of_node_put(np);
	return ret;
}

static int imx8mq_cpufreq_remove(struct platform_device *pdev)
{
	unregister_devfreq_cooling_notifier(&thermal_hot_pm_notifier);
	cpufreq_cooling_unregister(cdev);
	cpufreq_unregister_driver(&imx8mq_cpufreq_driver);
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
	if (free_opp)
		dev_pm_opp_of_remove_table(cpu_dev);
	clk_put(a53_clk);
	clk_put(arm_a53_src_clk);
	clk_put(arm_pll_clk);
	clk_put(arm_pll_out_clk);
	clk_put(sys1_pll_800m_clk);

	return 0;
}

static struct platform_driver imx8mq_cpufreq_platdrv = {
	.driver = {
		.name	= "imx8mq-cpufreq",
	},
	.probe		= imx8mq_cpufreq_probe,
	.remove		= imx8mq_cpufreq_remove,
};
module_platform_driver(imx8mq_cpufreq_platdrv);

MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_DESCRIPTION("Freescale i.MX8MQ cpufreq driver");
MODULE_LICENSE("GPL");
