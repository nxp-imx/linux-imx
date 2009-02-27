/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
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

int low_bus_freq_mode;
int high_bus_freq_mode;
int cpu_freq_khz_min;
int cpu_freq_khz_max;
int arm_lpm_clk;
int arm_normal_clk;
char *gp_reg_id = "SW1";
char *lp_reg_id = "SW2";
int axi_c_clk_support;

static struct clk *cpu_clk;
static struct clk *main_bus_clk;
static struct clk *pll2;
static struct clk *axi_a_clk;
static struct clk *axi_b_clk;
static struct clk *axi_c_clk;
static struct clk *emi_core_clk;
static struct clk *nfc_clk;
static struct clk *ahb_clk;
static struct clk *vpu_clk;
static struct clk *vpu_core_clk;
static struct clk *arm_axi_clk;
static struct clk *ddr_clk;
static struct clk *ipu_clk;
static struct clk *periph_apm_clk;
static struct clk *lp_apm;
static struct clk *osc;
static struct regulator *gp_regulator;
static struct regulator *lp_regulator;
static struct cpu_wp *cpu_wp_tbl;
static struct cpufreq_frequency_table imx_freq_table[4];

extern int dvfs_core_is_active;
extern int cpu_wp_nr;
#ifdef CONFIG_ARCH_MX51
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
#endif

static int set_cpu_freq(int freq)
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

static int set_low_bus_freq(void)
{
	int ret = 0;
	unsigned long lp_lpm_clk;

	struct clk *p_clk;
	struct clk *amode_parent_clk;

	if (axi_c_clk_support == 0)
		return 0;

	lp_lpm_clk = clk_get_rate(lp_apm);
	amode_parent_clk = lp_apm;
	p_clk = clk_get_parent(periph_apm_clk);

	/* Make sure osc_clk is the parent of lp_apm. */
	if (clk_get_parent(amode_parent_clk) != osc)
		clk_set_parent(amode_parent_clk, osc);

	/* Set the parent of periph_apm_clk to be lp_apm */
	clk_set_parent(periph_apm_clk, amode_parent_clk);
	amode_parent_clk = periph_apm_clk;

	p_clk = clk_get_parent(main_bus_clk);
	/* Set the parent of main_bus_clk to be periph_apm_clk */
	clk_set_parent(main_bus_clk, amode_parent_clk);

	clk_set_rate(axi_a_clk, lp_lpm_clk);
	clk_set_rate(axi_b_clk, lp_lpm_clk);
	clk_set_rate(axi_c_clk, lp_lpm_clk);
	clk_set_rate(emi_core_clk, lp_lpm_clk);
	clk_set_rate(nfc_clk, 4800000);
	clk_set_rate(ahb_clk, lp_lpm_clk);

	amode_parent_clk = emi_core_clk;

	p_clk = clk_get_parent(arm_axi_clk);
	if (p_clk != amode_parent_clk)
		clk_set_parent(arm_axi_clk, amode_parent_clk);

	p_clk = clk_get_parent(vpu_clk);
	if (p_clk != amode_parent_clk)
		clk_set_parent(vpu_clk, amode_parent_clk);

	p_clk = clk_get_parent(vpu_core_clk);
	if (p_clk != amode_parent_clk)
		clk_set_parent(vpu_core_clk, amode_parent_clk);

	/* Set the voltage to 1.0v for the LP domain. */
	ret = regulator_set_voltage(lp_regulator, 1000000, 1000000);
	if (ret < 0) {
		printk(KERN_DEBUG "COULD NOT SET GP VOLTAGE!!!!!!\n");
		return ret;
	}

	low_bus_freq_mode = 1;
	high_bus_freq_mode = 0;
	return ret;
}

static int set_high_bus_freq(void)
{
	struct clk *p_clk;
	struct clk *rmode_parent_clk;
	int ret = 0;

	if (axi_c_clk_support == 0)
		return 0;

	if (!low_bus_freq_mode)
		return ret;

	low_bus_freq_mode = 0;

	/* Set the voltage to 1.2v for the LP domain. */
	ret = regulator_set_voltage(lp_regulator, 1200000, 1200000);
	if (ret < 0) {
		printk(KERN_DEBUG "COULD NOT SET LP VOLTAGE!!!!!!\n");
		return ret;
	}

	rmode_parent_clk = pll2;

	/* Set the dividers before setting the parent clock. */
	clk_set_rate(axi_a_clk, 4800000);
	clk_set_rate(axi_b_clk, 4000000);
	clk_set_rate(axi_c_clk, 6000000);

	clk_set_rate(emi_core_clk, 4800000);
	clk_set_rate(ahb_clk, 4800000);

	/* Set the parent of main_bus_clk to be pll2 */
	p_clk = clk_get_parent(main_bus_clk);
	clk_set_parent(main_bus_clk, rmode_parent_clk);
	udelay(5);
	high_bus_freq_mode = 1;
	return ret;
}

static int low_freq_bus_used(void)
{
	if (axi_c_clk_support == 0)
		return 0;

	if ((clk_get_usecount(ipu_clk) == 0)
	    && (clk_get_usecount(vpu_clk) == 0))
		return 1;
	else
		return 0;
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

	if (relation == CPUFREQ_RELATION_H) {
		for (i = ARRAY_SIZE(imx_freq_table) - 1; i > 0; i--) {
			if (imx_freq_table[i].frequency <= target)
				return imx_freq_table[i].frequency;
		}
	} else if (relation == CPUFREQ_RELATION_L) {
		for (i = 0; i < ARRAY_SIZE(imx_freq_table) - 1; i++) {
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
	long freq_Hz;
	int low_freq_bus_ready = 0;
	int ret = 0;

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

	if ((freqs.old == freqs.new) && (freqs.new != cpu_freq_khz_min))
		return 0;

	low_freq_bus_ready = low_freq_bus_used();

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	if ((freq_Hz == arm_lpm_clk) && (!low_bus_freq_mode)
	    && (low_freq_bus_ready)) {
		set_low_bus_freq();
		if (!dvfs_core_is_active)
			ret = set_cpu_freq(freq_Hz);
	} else {
		if (!high_bus_freq_mode)
			set_high_bus_freq();

		if (!dvfs_core_is_active)
			ret = set_cpu_freq(freq_Hz);
		if (low_bus_freq_mode) {
			if (ret == 0)
				set_high_bus_freq();
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

	axi_c_clk = clk_get(NULL, "axi_c_clk");
	if (IS_ERR(axi_c_clk)) {
		axi_c_clk_support = 0;
		printk(KERN_ERR "%s: failed to get axi_c_clk\n", __func__);
	} else {
		axi_c_clk_support = 1;
		main_bus_clk = clk_get(NULL, "main_bus_clk");
		if (IS_ERR(main_bus_clk)) {
			printk(KERN_ERR "%s: failed to get main_bus_clk\n",
			       __func__);
			return PTR_ERR(main_bus_clk);
		}

		pll2 = clk_get(NULL, "pll2");
		if (IS_ERR(pll2)) {
			printk(KERN_ERR "%s: failed to get pll2\n", __func__);
			return PTR_ERR(pll2);
		}

		axi_a_clk = clk_get(NULL, "axi_a_clk");
		if (IS_ERR(axi_a_clk)) {
			printk(KERN_ERR "%s: failed to get axi_a_clk\n",
			       __func__);
			return PTR_ERR(axi_a_clk);
		}

		axi_b_clk = clk_get(NULL, "axi_b_clk");
		if (IS_ERR(axi_b_clk)) {
			printk(KERN_ERR "%s: failed to get axi_b_clk\n",
			       __func__);
			return PTR_ERR(axi_b_clk);
		}

		emi_core_clk = clk_get(NULL, "emi_core_clk");
		if (IS_ERR(emi_core_clk)) {
			printk(KERN_ERR "%s: failed to get emi_core_clk\n",
			       __func__);
			return PTR_ERR(emi_core_clk);
		}

		nfc_clk = clk_get(NULL, "nfc_clk");
		if (IS_ERR(nfc_clk)) {
			printk(KERN_ERR "%s: failed to get nfc_clk\n",
			       __func__);
			return PTR_ERR(nfc_clk);
		}

		ahb_clk = clk_get(NULL, "ahb_clk");
		if (IS_ERR(ahb_clk)) {
			printk(KERN_ERR "%s: failed to get ahb_clk\n",
			       __func__);
			return PTR_ERR(ahb_clk);
		}

		vpu_core_clk = clk_get(NULL, "vpu_core_clk");
		if (IS_ERR(vpu_core_clk)) {
			printk(KERN_ERR "%s: failed to get vpu_core_clk\n",
			       __func__);
			return PTR_ERR(vpu_core_clk);
		}

		arm_axi_clk = clk_get(NULL, "arm_axi_clk");
		if (IS_ERR(arm_axi_clk)) {
			printk(KERN_ERR "%s: failed to get arm_axi_clk\n",
			       __func__);
			return PTR_ERR(arm_axi_clk);
		}

		ddr_clk = clk_get(NULL, "ddr_clk");
		if (IS_ERR(ddr_clk)) {
			printk(KERN_ERR "%s: failed to get ddr_clk\n",
			       __func__);
			return PTR_ERR(ddr_clk);
		}

		ipu_clk = clk_get(NULL, "ipu_clk");
		if (IS_ERR(ipu_clk)) {
			printk(KERN_ERR "%s: failed to get ipu_clk\n",
			       __func__);
			return PTR_ERR(ipu_clk);
		}

		vpu_clk = clk_get(NULL, "vpu_clk");
		if (IS_ERR(vpu_clk)) {
			printk(KERN_ERR "%s: failed to get vpu_clk\n",
			       __func__);
			return PTR_ERR(vpu_clk);
		}

		periph_apm_clk = clk_get(NULL, "periph_apm_clk");
		if (IS_ERR(periph_apm_clk)) {
			printk(KERN_ERR "%s: failed to get periph_apm_clk\n",
			       __func__);
			return PTR_ERR(periph_apm_clk);
		}

		lp_apm = clk_get(NULL, "lp_apm");
		if (IS_ERR(lp_apm)) {
			printk(KERN_ERR "%s: failed to get lp_apm\n", __func__);
			return PTR_ERR(lp_apm);
		}

		osc = clk_get(NULL, "osc");
		if (IS_ERR(osc)) {
			printk(KERN_ERR "%s: failed to get osc\n", __func__);
			return PTR_ERR(osc);
		}
	}

	gp_regulator = regulator_get(NULL, gp_reg_id);
	if (IS_ERR(gp_regulator)) {
		clk_put(cpu_clk);
		printk(KERN_ERR "%s: failed to get gp regulator\n", __func__);
		return PTR_ERR(gp_regulator);
	}

	lp_regulator = regulator_get(NULL, lp_reg_id);
	if (IS_ERR(lp_regulator)) {
		clk_put(ahb_clk);
		printk(KERN_ERR "%s: failed to get lp regulator\n", __func__);
		return PTR_ERR(lp_regulator);
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

	imx_freq_table[i].index = i + 1;
	imx_freq_table[i].frequency = cpu_wp_tbl[i].cpu_rate / 1000;

	if ((cpu_wp_tbl[i].cpu_rate / 1000) < cpu_freq_khz_min)
		cpu_freq_khz_min = cpu_wp_tbl[i].cpu_rate / 1000;

	if ((cpu_wp_tbl[i].cpu_rate / 1000) > cpu_freq_khz_max)
		cpu_freq_khz_max = cpu_wp_tbl[i].cpu_rate / 1000;

	imx_freq_table[i].index = 0;
	imx_freq_table[i].frequency = CPUFREQ_TABLE_END;

	policy->cur = policy->min = policy->max = clk_get_rate(cpu_clk) / 1000;
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	policy->cpuinfo.min_freq = cpu_freq_khz_min;
	policy->cpuinfo.max_freq = cpu_freq_khz_max;

	arm_lpm_clk = cpu_freq_khz_min * 1000;
	arm_normal_clk = cpu_freq_khz_max * 1000;

	/* Manual states, that PLL stabilizes in two CLK32 periods */
	policy->cpuinfo.transition_latency = 10;

	ret = cpufreq_frequency_table_cpuinfo(policy, imx_freq_table);
	if (ret < 0) {
		clk_put(cpu_clk);
		if (axi_c_clk_support != 0) {
			clk_put(main_bus_clk);
			clk_put(pll2);
			clk_put(axi_a_clk);
			clk_put(axi_b_clk);
			clk_put(axi_c_clk);
			clk_put(emi_core_clk);
			clk_put(nfc_clk);
			clk_put(ahb_clk);
			clk_put(vpu_core_clk);
			clk_put(arm_axi_clk);
			clk_put(ddr_clk);
			clk_put(ipu_clk);
			clk_put(vpu_clk);
			clk_put(periph_apm_clk);
			clk_put(lp_apm);
			clk_put(osc);
		}

		regulator_put(gp_regulator);
		regulator_put(lp_regulator);
		printk(KERN_ERR "%s: failed to register i.MXC CPUfreq\n",
		       __func__);
		return ret;
	}
	cpufreq_frequency_table_get_attr(imx_freq_table, policy->cpu);

	low_bus_freq_mode = 0;
	high_bus_freq_mode = 0;
	return 0;
}

static int mxc_cpufreq_driver_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);

	/* Reset CPU to 665MHz */
	if (!dvfs_core_is_active)
		set_cpu_freq(arm_normal_clk);

	if (low_bus_freq_mode)
		set_high_bus_freq();

	clk_put(cpu_clk);
	if (axi_c_clk_support != 0) {
		clk_put(main_bus_clk);
		clk_put(pll2);
		clk_put(axi_a_clk);
		clk_put(axi_b_clk);
		clk_put(axi_c_clk);
		clk_put(emi_core_clk);
		clk_put(nfc_clk);
		clk_put(ahb_clk);
		clk_put(vpu_core_clk);
		clk_put(arm_axi_clk);
		clk_put(ddr_clk);
		clk_put(ipu_clk);
		clk_put(periph_apm_clk);
		clk_put(lp_apm);
		clk_put(osc);
	}
	regulator_put(gp_regulator);
	regulator_put(lp_regulator);
	return 0;
}

static struct cpufreq_driver mxc_driver = {
	.flags = CPUFREQ_STICKY,
	.verify = mxc_verify_speed,
	.target = mxc_set_target,
	.get = mxc_get_speed,
	.init = mxc_cpufreq_driver_init,
	.exit = mxc_cpufreq_driver_exit,
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
