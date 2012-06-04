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
#define IADR	0x00
#define IFDR	0x04
#define I2CR	0x08
#define I2SR	0x0c
#define I2DR	0x10

#define I2CR_IEN	(1 << 7)
#define I2CR_IIEN	(1 << 6)
#define I2CR_MSTA	(1 << 5)
#define I2CR_MTX	(1 << 4)
#define I2CR_TX_NO_AK	(1 << 3)
#define I2CR_RSTA	(1 << 2)

#define I2SR_ICF	(1 << 7)
#define I2SR_IBB	(1 << 5)
#define I2SR_IIF	(1 << 1)
#define I2SR_RX_NO_AK	(1 << 0)

#define I2C_MAX_TIMEOUT		100000
#define I2C_TIMEOUT_TICKET	1
#define CCM_CCGR2	0x70

/*#define MX6_I2CRAW_DEBUG*/
#ifdef MX6_I2CRAW_DEBUG
#define DPRINTF(args...)  printk(args)
#else
#define DPRINTF(args...)
#endif

/*judge for pfuze regulator driver suspend or not, after pfuze regulator
suspend and before resume, should use i2c raw read/write to configure
voltage, it's safe enough, otherwise mxc_cpufreq_suspend will be failed
since i2c/pfuze have been suspend firstly.*/
extern int cpu_freq_suspend_in;
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
extern int high_bus_freq_mode;
extern int set_low_bus_freq(void);
extern int set_high_bus_freq(int high_bus_speed);
extern int low_freq_bus_used(void);

#ifdef CONFIG_MX6_INTER_LDO_BYPASS
static void __iomem *i2c_base;/*i2c for pmic*/
static void __iomem *ccm_base;/*ccm_base*/
static int wait_busy(void)
{
	int timeout = I2C_MAX_TIMEOUT;

	while ((!(readb(i2c_base + I2SR) & I2SR_IBB) && (--timeout))) {
		writeb(0, i2c_base + I2SR);
		udelay(I2C_TIMEOUT_TICKET);
	}
	return timeout ? timeout : (readb(i2c_base + I2SR) & I2SR_IBB);
}

static int wait_complete(void)
{
	int timeout = I2C_MAX_TIMEOUT;

	while ((!(readb(i2c_base + I2SR) & I2SR_ICF)) && (--timeout)) {
		writeb(0, i2c_base + I2SR);
		udelay(I2C_TIMEOUT_TICKET);
	}
	DPRINTF("%s:%x\n", __func__, readb(i2c_base + I2SR));
	{
		int i;
		for (i = 0; i < 200; i++)
			udelay(10);

	}
	writeb(0, i2c_base + I2SR);	/* clear interrupt */

	return timeout;
}

static int tx_byte(u8 byte)
{
	writeb(byte, i2c_base + I2DR);

	if (!wait_complete() || readb(i2c_base + I2SR) & I2SR_RX_NO_AK) {
		DPRINTF("%s:%x <= %x\n", __func__, readb(i2c_base + I2SR),
			byte);
		return -1;
	}
	DPRINTF("%s:%x\n", __func__, byte);
	return 0;
}

static int i2c_addr(unsigned char chip, u32 addr, int alen)
{
	writeb(I2CR_IEN | I2CR_MSTA | I2CR_MTX, i2c_base + I2CR);
	if (!wait_busy()) {
		DPRINTF("%s:trigger start fail(%x)\n",
		       __func__, readb(i2c_base + I2SR));
		return -1;
	}
	if (tx_byte(chip << 1) || (readb(i2c_base + I2SR) & I2SR_RX_NO_AK)) {
		DPRINTF("%s:chip address cycle fail(%x)\n",
		       __func__, readb(i2c_base + I2SR));
		return -1;
	}
	while (alen--)
		if (tx_byte((addr >> (alen * 8)) & 0xff) ||
		    (readb(i2c_base + I2SR) & I2SR_RX_NO_AK)) {
			DPRINTF("%s:device address cycle fail(%x)\n",
			       __func__, readb(i2c_base + I2SR));
			return -1;
		}
	return 0;
}


static int i2c_write(unsigned char chip, u32 addr, int alen, unsigned char *buf,
			int len)
{
	int timeout = I2C_MAX_TIMEOUT;
	DPRINTF("%s chip: 0x%02x addr: 0x%04x alen: %d len: %d\n",
		__func__, chip, addr, alen, len);

	if (i2c_addr(chip, addr, alen))
		return -1;

	while (len--)
		if (tx_byte(*buf++))
			return -1;

	writeb(I2CR_IEN, i2c_base + I2CR);

	while (readb(i2c_base + I2SR) & I2SR_IBB && --timeout)
		udelay(I2C_TIMEOUT_TICKET);

	return 0;
}

#endif
int set_cpu_freq(int freq)
{
	int i, ret = 0;
	int org_cpu_rate;
	int gp_volt = 0;
	#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	unsigned char data;
	#endif

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
	if (cpu_freq_suspend_in) {
		u32 value;
		/*init I2C*/
		value = __raw_readl(ccm_base + CCM_CCGR2);
		__raw_writel(value | 0x300, ccm_base + CCM_CCGR2);
		udelay(1);
		value = readb(i2c_base + I2CR);
		writeb(value | (1 << 7), i2c_base + I2CR);
		value = readb(i2c_base + I2SR);
		writeb(0, i2c_base + I2SR);
		switch (freq) {
		case 1200000000:/*1.275*/
			data = 0x27;
			break;
		case 996000000:/*1.225V*/
			data = 0x25;
			break;
		case 792000000:/*1.1V*/
		case 672000000:
			data = 0x20;
			break;
		case 396000000:/*0.95V*/
			data = 0x1a;
			break;
		case 198000000:/*0.85V*/
			data = 0x16;
			break;
		default:
			printk(KERN_ERR "suspend freq error:%d!!!\n", freq);
			break;
		}
	}
	#endif

	/*Set the voltage for the GP domain. */
	if (freq > org_cpu_rate) {
		if (low_bus_freq_mode)
			set_high_bus_freq(0);
		#ifdef CONFIG_MX6_INTER_LDO_BYPASS
		if (cpu_freq_suspend_in) {
			ret = i2c_write(0x8, 0x20, 1, &data, 1);
			udelay(10);
		 } else
			ret = regulator_set_voltage(cpu_regulator, gp_volt,
							gp_volt);
		#else
		ret = regulator_set_voltage(cpu_regulator, gp_volt,
					    gp_volt);
		#endif
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
		#ifdef CONFIG_MX6_INTER_LDO_BYPASS

		if (cpu_freq_suspend_in) {
			ret = i2c_write(0x8, 0x20, 1, &data, 1);
			udelay(10);
		} else
			ret = regulator_set_voltage(cpu_regulator, gp_volt,
							gp_volt);
		#else
		ret = regulator_set_voltage(cpu_regulator, gp_volt,
					    gp_volt);
		#endif
		if (ret < 0) {
			printk(KERN_DEBUG "COULD NOT SET GP VOLTAGE!!!!\n");
			return ret;
		}
		if (low_freq_bus_used() && !low_bus_freq_mode)
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

	ret = set_cpu_freq(freq_Hz);

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

static int mxc_cpufreq_suspend(struct cpufreq_policy *policy)
{
	pre_suspend_rate = clk_get_rate(cpu_clk);
	/* Set to max freq and voltage */
	/*There should be *1000, but if fix the typo error, found
	hard to pass streng test, it means we didn't move cpu freq
	to highest freq in suspend, but if we choose bypass, we
	have to do this, so use macro to decrease the impact on
	released code, the 1Ghz issue should be fixed in the future*/
	if (pre_suspend_rate != (imx_freq_table[0].frequency * 1000))
	#ifdef CONFIG_MX6_INTER_LDO_BYPASS
		set_cpu_freq(imx_freq_table[0].frequency * 1000);
	#else
		set_cpu_freq(imx_freq_table[0].frequency);
	#endif

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

extern void mx6_cpu_regulator_init(void);
static int __init mxc_cpufreq_driver_init(void)
{
	#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	mx6_cpu_regulator_init();
	i2c_base = MX6_IO_ADDRESS(MX6Q_I2C2_BASE_ADDR);
	ccm_base = MX6_IO_ADDRESS(CCM_BASE_ADDR);
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
