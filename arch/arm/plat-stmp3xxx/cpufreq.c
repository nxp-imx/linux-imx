/*
 *  CPU frequency scaling for Freescale STMP37XX/STMP378X
 *
 *  Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
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
#include <mach/regs-digctl.h>
#include <mach/regs-clkctrl.h>
#include <mach/platform.h>
#include "clock.h"

#define VERY_HI_RATE		2000000000
#define CLKCTRL_PLL_PWD_BIT 16
#define CLKCTRL_PLL_BYPASS 0x1ff
#define CLKCTRL_HBUS_AUTO_SLOW_MODE_BIT 20
#define LCD_ON_CPU_FREQ_KHZ 261820

static struct profile {
	int cpu;
	int ahb;
	int emi;
	int ss;
	int vddd;
	int vddd_bo;
	int cur;
	int vddio;
	int vdda;
	int pll_off;
} profiles[] = {
	{ 454740, 151580, 130910, 0, 1550000,
	1450000, 355000, 3300000, 1750000, 0 },
	{ 392730, 130910, 130910, 0, 1475000,
	1375000, 225000, 3300000, 1750000, 0 },
	{ 360000, 120000, 120000, 0, 13750000,
	1275000, 200000, 3300000, 1750000, 0 },
	{ 261820, 130910, 130910, 0, 1275000,
	1175000, 173000, 3300000, 1750000, 0 },
#ifdef CONFIG_STMP378X_RAM_MDDR
	{  64000,  64000,  48000, 3, 1050000,
	975000, 150000, 3300000, 1750000, 0 },
	{  24000,  24000,  24000, 3, 1050000,
	975000, 150000, 3075000, 1725000, 1 },
#else
	{  64000,  64000,  96000, 3, 1050000,
	975000, 150000, 3300000, 1750000, 0 },
#endif
};

static u32 clkseq_setting;
static struct regulator *cpu_regulator;
static struct clk *cpu_clk;
static struct clk *ahb_clk;
static struct clk *emi_clk;
static struct clk *usb_clk;
static struct clk *lcdif_clk;
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
extern int clk_get_usage(struct clk *clk);

static void hbus_auto_slow_mode_enable(void)
{
	__raw_writel(CLKCTRL_HBUS_AUTO_SLOW_MODE_BIT,
			REGS_CLKCTRL_BASE + HW_CLKCTRL_HBUS_SET);
}

static void hbus_auto_slow_mode_disable(void)
{
	__raw_writel(CLKCTRL_HBUS_AUTO_SLOW_MODE_BIT,
			  REGS_CLKCTRL_BASE + HW_CLKCTRL_HBUS_CLR);
}

int low_freq_used(void)
{
	if ((clk_get_usage(usb_clk) == 0)
	    && (clk_get_usage(lcdif_clk) == 0))
		return 1;
	else
		return 0;
	}

static int set_freq_table(struct cpufreq_policy *policy, int end_index)
{
	int ret = 0;
	int i;

	cpu_freq_khz_min = profiles[0].cpu;
	cpu_freq_khz_max = profiles[0].cpu;
	for (i = 0; i < end_index; i++) {
		imx_freq_table[end_index - 1 - i].index = end_index  - i;
		imx_freq_table[end_index - 1 - i].frequency =
						profiles[i].cpu;

		if ((profiles[i].cpu) < cpu_freq_khz_min)
			cpu_freq_khz_min = profiles[i].cpu;

		if ((profiles[i].cpu) > cpu_freq_khz_max)
			cpu_freq_khz_max = profiles[i].cpu;
	}

	imx_freq_table[i].index = 0;
	imx_freq_table[i].frequency = CPUFREQ_TABLE_END;

	policy->cur = clk_get_rate(cpu_clk);
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

static int set_op(unsigned int target_freq)
{
	struct cpufreq_freqs freqs;
	int ret = 0, i;

	freqs.old = clk_get_rate(cpu_clk);
	freqs.cpu = 0;

/* work around usb problem when in updater firmare  mode*/
#ifdef CONFIG_STMP_UTP
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

	if (freqs.old == freqs.new) {
		if (regulator_get_voltage(vddd) == profiles[i].vddd)
			return 0;
	}

	if (freqs.old == 24000 && freqs.new > 24000) {
		/* turn pll on */
		__raw_writel(CLKCTRL_PLL_PWD_BIT, REGS_CLKCTRL_BASE +
			      HW_CLKCTRL_PLLCTRL0_SET);
		udelay(10);
	} else if (freqs.old > 24000 && freqs.new == 24000)
		clkseq_setting = __raw_readl(REGS_CLKCTRL_BASE +
						 HW_CLKCTRL_CLKSEQ);

	if (cpu_regulator && (freqs.old < freqs.new)) {
		ret = regulator_set_current_limit(cpu_regulator,
			profiles[i].cur, profiles[i].cur);
		if (ret)
			return ret;
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	if (freqs.old > freqs.new) {
		int ss = profiles[i].ss;
		clk_set_rate(cpu_clk, profiles[i].cpu);
		clk_set_rate(ahb_clk, profiles[i].ahb);
		clk_set_rate(emi_clk, profiles[i].emi);
		__raw_writel(BF(ss, DIGCTL_ARMCACHE_VALID_SS) |
				      BF(ss, DIGCTL_ARMCACHE_DRTY_SS) |
				      BF(ss, DIGCTL_ARMCACHE_CACHE_SS) |
				      BF(ss, DIGCTL_ARMCACHE_DTAG_SS) |
				      BF(ss, DIGCTL_ARMCACHE_ITAG_SS),
				      REGS_DIGCTL_BASE + HW_DIGCTL_ARMCACHE);
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
		__raw_writel(BF(ss, DIGCTL_ARMCACHE_VALID_SS) |
				      BF(ss, DIGCTL_ARMCACHE_DRTY_SS) |
				      BF(ss, DIGCTL_ARMCACHE_CACHE_SS) |
				      BF(ss, DIGCTL_ARMCACHE_DTAG_SS) |
				      BF(ss, DIGCTL_ARMCACHE_ITAG_SS),
				      REGS_DIGCTL_BASE + HW_DIGCTL_ARMCACHE);
		clk_set_rate(cpu_clk, profiles[i].cpu);
		clk_set_rate(ahb_clk, profiles[i].ahb);
		clk_set_rate(emi_clk, profiles[i].emi);
	}
	udelay(100);

	if (freqs.old > 24000 && freqs.new == 24000) {
		/* turn pll off */
		__raw_writel(CLKCTRL_PLL_PWD_BIT, REGS_CLKCTRL_BASE +
			      HW_CLKCTRL_PLLCTRL0_CLR);
		__raw_writel(CLKCTRL_PLL_BYPASS, REGS_CLKCTRL_BASE +
			      HW_CLKCTRL_CLKSEQ);
	} else if (freqs.old == 24000 && freqs.new > 24000)
		__raw_writel(clkseq_setting, REGS_CLKCTRL_BASE +
				HW_CLKCTRL_CLKSEQ);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	if (cpu_regulator && (freqs.old > freqs.new))   /* will not fail */
		regulator_set_current_limit(cpu_regulator,
						profiles[i].cur,
						profiles[i].cur);

	return ret;
}

static int calc_frequency_khz(int target, unsigned int relation)
{
	int i;

	if (target == clk_get_rate(cpu_clk))
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

static int stmp3xxx_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	int freq_KHz;
	struct cpufreq_freqs freqs;
	int low_freq_bus_ready = 0;

	if (cpufreq_trig_needed  == 1) {
		/* Set the current working point. */
		cpufreq_trig_needed = 0;
		target_freq = clk_get_rate(cpu_clk);
		freq_KHz = calc_frequency_khz(target_freq, relation);

		freqs.old = target_freq;
		freqs.new = freq_KHz;
		freqs.cpu = 0;
		freqs.flags = 0;

		low_freq_bus_ready = low_freq_used();
		if (low_freq_bus_ready) {
			cur_freq_table_size = ARRAY_SIZE(profiles);
			hbus_auto_slow_mode_enable();
		} else {
			cur_freq_table_size = lcd_on_freq_table_size;
			hbus_auto_slow_mode_disable();
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
	if (target_freq < policy->cpuinfo.min_freq)
		target_freq = policy->cpuinfo.min_freq;

	if (target_freq < policy->min)
		target_freq = policy->min;

	freq_KHz = calc_frequency_khz(target_freq, relation);
	return set_op(freq_KHz);
	}

static unsigned int stmp3xxx_getspeed(unsigned int cpu)
{
	struct cpufreq_freqs freqs;
	int freq_KHz;
	unsigned int target_freq;

	if (cpu)
		return 0;

	if (cpufreq_trig_needed  == 1) {
		target_freq = clk_get_rate(cpu_clk);
		freq_KHz = calc_frequency_khz(target_freq, CPUFREQ_RELATION_L);

		freqs.old = target_freq;
		freqs.new = freq_KHz;
		freqs.cpu = 0;
		freqs.flags = 0;

		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	}

	return clk_get_rate(cpu_clk);
}


static int stmp3xxx_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, imx_freq_table);
}

static int __init stmp3xxx_cpu_init(struct cpufreq_policy *policy)
{
	int ret = 0;
	int i;

	cpu_clk = clk_get(NULL, "cpu");
	if (IS_ERR(cpu_clk)) {
		ret = PTR_ERR(cpu_clk);
		goto out_cpu;
	}

	ahb_clk = clk_get(NULL, "hclk");
	if (IS_ERR(ahb_clk)) {
		ret = PTR_ERR(ahb_clk);
		goto out_ahb;
	}

	emi_clk = clk_get(NULL, "emi");
	if (IS_ERR(emi_clk)) {
		ret = PTR_ERR(emi_clk);
		goto out_emi;
	}

	usb_clk = clk_get(NULL, "usb");
	if (IS_ERR(usb_clk)) {
		ret = PTR_ERR(usb_clk);
		goto out_usb;
	}

	lcdif_clk = clk_get(NULL, "lcdif");
	if (IS_ERR(lcdif_clk)) {
		ret = PTR_ERR(lcdif_clk);
		goto out_lcd;
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

	/* Set the current working point. */
	set_freq_table(policy, lcd_on_freq_table_size);
	cpufreq_trig_needed = 0;
	cur_freq_table_size = lcd_on_freq_table_size;
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

	clk_put(lcdif_clk);
out_lcd:
	clk_put(usb_clk);
out_usb:
	clk_put(emi_clk);
out_emi:
	clk_put(ahb_clk);
out_ahb:
	clk_put(cpu_clk);
out_cpu:
	return ret;
}

static int stmp3xxx_cpu_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);

	/* Reset CPU to 392MHz */
	set_op(profiles[1].cpu);

	clk_put(cpu_clk);
	regulator_put(cpu_regulator);
	return 0;
}

static struct cpufreq_driver stmp3xxx_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= stmp3xxx_verify_speed,
	.target		= stmp3xxx_target,
	.get		= stmp3xxx_getspeed,
	.init		= stmp3xxx_cpu_init,
	.exit		= stmp3xxx_cpu_exit,
	.name		= "stmp3xxx",
};

static int __devinit stmp3xxx_cpufreq_init(void)
{
	return cpufreq_register_driver(&stmp3xxx_driver);
}

static void stmp3xxx_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&stmp3xxx_driver);
}

module_init(stmp3xxx_cpufreq_init);
module_exit(stmp3xxx_cpufreq_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("CPUfreq driver for i.MX");
MODULE_LICENSE("GPL");

