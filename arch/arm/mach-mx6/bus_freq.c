/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file bus_freq.c
 *
 * @brief A common API for the Freescale Semiconductor i.MXC CPUfreq module
 * and DVFS CORE module.
 *
 * The APIs are for setting bus frequency to low or high.
 *
 * @ingroup PM
 */
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <mach/iram.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <mach/mxc_dvfs.h>
#include <mach/sdram_autogating.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include "crm_regs.h"


#define LPAPM_CLK	24000000
#define DDR_MED_CLK	400000000
#define DDR3_NORMAL_CLK	528000000
#define AXI_CLK_RATE		270000000
#define GPC_PGC_GPU_PGCR_OFFSET		0x260
#define GPC_CNTR_OFFSET				0x0



DEFINE_SPINLOCK(ddr_freq_lock);

int low_bus_freq_mode;
int audio_bus_freq_mode;
int high_bus_freq_mode;
int med_bus_freq_mode;

int bus_freq_scaling_initialized;
static struct device *busfreq_dev;
static int busfreq_suspended;

/* True if bus_frequency is scaled not using DVFS-PER */
int bus_freq_scaling_is_active;

int lp_high_freq;
int lp_med_freq;
int lp_audio_freq;
unsigned int ddr_low_rate;
unsigned int ddr_med_rate;
unsigned int ddr_normal_rate;

int low_freq_bus_used(void);
void set_ddr_freq(int ddr_freq);

extern int init_mmdc_settings(void);
extern struct cpu_op *(*get_cpu_op)(int *op);
extern int update_ddr_freq(int ddr_rate);
extern void __iomem *gpc_base;

struct mutex bus_freq_mutex;

struct timeval start_time;
struct timeval end_time;

static int cpu_op_nr;
static struct cpu_op *cpu_op_tbl;
static struct clk *pll2_400;
static struct clk *axi_clk;
static struct clk *periph_clk;
static struct clk *pll3_540;
static struct clk *cpu_clk;
static unsigned int org_ldo;
static struct clk *pll3;

static struct delayed_work low_bus_freq_handler;

static void reduce_bus_freq_handler(struct work_struct *work)
{
	unsigned long reg;

	if (low_bus_freq_mode || !low_freq_bus_used())
		return;

	if (audio_bus_freq_mode && lp_audio_freq)
		return;

	while (!mutex_trylock(&bus_freq_mutex))
		msleep(1);

	/* PLL3 is used in the DDR freq change process, enable it. */

	if (low_bus_freq_mode || !low_freq_bus_used()) {
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	if (audio_bus_freq_mode && lp_audio_freq) {
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	/* Set the axi_clk to be sourced from the periph_clk.
	  * So that its frequency can be lowered down to 50MHz
	  * or 24MHz as the case may be. */
	clk_set_parent(axi_clk, periph_clk);

	clk_enable(pll3);

	if (lp_audio_freq) {
		/* Need to ensure that PLL2_PFD_400M is kept ON. */
		clk_enable(pll2_400);
		update_ddr_freq(50000000);
		audio_bus_freq_mode = 1;
		low_bus_freq_mode = 0;
	} else {
		update_ddr_freq(24000000);
		if (audio_bus_freq_mode)
			clk_disable(pll2_400);
		low_bus_freq_mode = 1;
		audio_bus_freq_mode = 0;
	}

	if (med_bus_freq_mode)
		clk_disable(pll2_400);

	high_bus_freq_mode = 0;
	med_bus_freq_mode = 0;

	if (cpu_is_mx6q()) {
		/* Power gate the PU LDO. */
		/* Power gate the PU domain first. */
		/* enable power down request */
		reg = __raw_readl(gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
		__raw_writel(reg | 0x1, gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
		/* power down request */
		reg = __raw_readl(gpc_base + GPC_CNTR_OFFSET);
		__raw_writel(reg | 0x1, gpc_base + GPC_CNTR_OFFSET);
		/* Wait for power down to complete. */
		while (__raw_readl(gpc_base + GPC_CNTR_OFFSET) & 0x1)
			;

		org_ldo = reg = __raw_readl(ANADIG_REG_CORE);
		reg &= ~(ANADIG_REG_TARGET_MASK << ANADIG_REG1_PU_TARGET_OFFSET);
		__raw_writel(reg, ANADIG_REG_CORE);
	}
	clk_disable(pll3);
	mutex_unlock(&bus_freq_mutex);
}

/* Set the DDR, AHB to 24MHz.
  * This mode will be activated only when none of the modules that
  * need a higher DDR or AHB frequency are active.
  */
int set_low_bus_freq(void)
{
	if (busfreq_suspended)
		return 0;

	if (!bus_freq_scaling_initialized || !bus_freq_scaling_is_active)
		return 0;

	/* Don't lower the frequency immediately. Instead scheduled a delayed work
	  * and drop the freq if the conditions still remain the same.
	  */
	schedule_delayed_work(&low_bus_freq_handler, usecs_to_jiffies(3000000));
	return 0;
}

/* Set the DDR to either 528MHz or 400MHz for MX6q
 * or 400MHz for MX6DL.
 */
int set_high_bus_freq(int high_bus_freq)
{
	unsigned long reg;

	if (busfreq_suspended)
		return 0;

	if (!bus_freq_scaling_initialized || !bus_freq_scaling_is_active)
		return 0;

	if (high_bus_freq_mode && high_bus_freq)
		return 0;

	if (med_bus_freq_mode && !high_bus_freq)
		return 0;

	while (!mutex_trylock(&bus_freq_mutex))
		msleep(1);

	if ((high_bus_freq_mode && (high_bus_freq || lp_high_freq)) ||
		(med_bus_freq_mode && !high_bus_freq && lp_med_freq && !lp_high_freq)) {
		mutex_unlock(&bus_freq_mutex);
		return 0;
	}
	clk_enable(pll3);

	if (clk_get_parent(axi_clk) != pll3_540) {
		/* We need to set axi_clk to be sourced from PLL3_540MHz. */
		/* Ensure the divider is set to divide by 2 before changing the
		  * parent. */
		if (low_bus_freq_mode)
			clk_set_rate(axi_clk, clk_get_rate(axi_clk) / 2);
		clk_set_parent(axi_clk, pll3_540);
		clk_set_rate(axi_clk, AXI_CLK_RATE);
	}

	/* Enable the PU LDO */
	if (cpu_is_mx6q() && low_bus_freq_mode) {
		__raw_writel(org_ldo, ANADIG_REG_CORE);

		/* enable power up request */
		reg = __raw_readl(gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
		__raw_writel(reg | 0x1, gpc_base + GPC_PGC_GPU_PGCR_OFFSET);
		/* power up request */
		reg = __raw_readl(gpc_base + GPC_CNTR_OFFSET);
		__raw_writel(reg | 0x2, gpc_base + GPC_CNTR_OFFSET);
		/* Wait for the power up bit to clear */
		while (__raw_readl(gpc_base + GPC_CNTR_OFFSET) & 0x2)
			;
	}

	if (high_bus_freq) {
		update_ddr_freq(ddr_normal_rate);
		if (med_bus_freq_mode)
			clk_disable(pll2_400);
		high_bus_freq_mode = 1;
		med_bus_freq_mode = 0;
	} else {
		clk_enable(pll2_400);
		update_ddr_freq(ddr_med_rate);
		high_bus_freq_mode = 0;
		med_bus_freq_mode = 1;
	}
	if (audio_bus_freq_mode)
		clk_disable(pll2_400);
	low_bus_freq_mode = 0;
	audio_bus_freq_mode = 0;

	low_bus_freq_mode = 0;

	clk_disable(pll3);
	mutex_unlock(&bus_freq_mutex);

	return 0;
}


int low_freq_bus_used(void)
{
	if (!bus_freq_scaling_initialized)
		return 0;

	/* We only go the lowest setpoint if ARM is also
	 * at the lowest setpoint.
	 */
	if ((clk_get_rate(cpu_clk) >
			cpu_op_tbl[cpu_op_nr - 1].cpu_rate)
		|| (cpu_op_nr == 1)) {
		return 0;
	}

	if ((lp_high_freq == 0)
	    && (lp_med_freq == 0))
		return 1;
	else
		return 0;
}

void setup_pll(void)
{
}

static ssize_t bus_freq_scaling_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (bus_freq_scaling_is_active)
		return sprintf(buf, "Bus frequency scaling is enabled\n");
	else
		return sprintf(buf, "Bus frequency scaling is disabled\n");
}

static ssize_t bus_freq_scaling_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (strncmp(buf, "1", 1) == 0) {
		bus_freq_scaling_is_active = 1;
		set_high_bus_freq(0);
	} else if (strncmp(buf, "0", 1) == 0) {
		if (bus_freq_scaling_is_active)
			set_high_bus_freq(1);
		bus_freq_scaling_is_active = 0;
	}
	return size;
}

static int busfreq_suspend(struct platform_device *pdev, pm_message_t message)
{
	if (low_bus_freq_mode)
		set_high_bus_freq(1);
	busfreq_suspended = 1;
	return 0;
}

static int busfreq_resume(struct platform_device *pdev)
{
	busfreq_suspended = 0;
	return  0;
}

static DEVICE_ATTR(enable, 0644, bus_freq_scaling_enable_show,
			bus_freq_scaling_enable_store);

/*!
 * This is the probe routine for the bus frequency driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 *
 */
static int __devinit busfreq_probe(struct platform_device *pdev)
{
	u32 err;

	busfreq_dev = &pdev->dev;

	pll2_400 = clk_get(NULL, "pll2_pfd_400M");
	if (IS_ERR(pll2_400)) {
		printk(KERN_DEBUG "%s: failed to get pll2_pfd_400M\n",
		       __func__);
		return PTR_ERR(pll2_400);
	}

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get cpu_clk\n",
		       __func__);
		return PTR_ERR(cpu_clk);
	}

	pll3 = clk_get(NULL, "pll3_main_clk");
	if (IS_ERR(pll3)) {
		printk(KERN_DEBUG "%s: failed to get pll3\n",
		       __func__);
		return PTR_ERR(cpu_clk);
	}

	axi_clk = clk_get(NULL, "axi_clk");
	if (IS_ERR(axi_clk)) {
		printk(KERN_DEBUG "%s: failed to get axi_clk\n",
		       __func__);
		return PTR_ERR(axi_clk);
	}

	periph_clk = clk_get(NULL, "periph_clk");
	if (IS_ERR(periph_clk)) {
		printk(KERN_DEBUG "%s: failed to get periph_clk\n",
		       __func__);
		return PTR_ERR(periph_clk);
	}

	pll3_540 = clk_get(NULL, "pll3_pfd_540M");
	if (IS_ERR(pll3_540)) {
		printk(KERN_DEBUG "%s: failed to get periph_clk\n",
		       __func__);
		return PTR_ERR(pll3_540);
	}

	err = sysfs_create_file(&busfreq_dev->kobj, &dev_attr_enable.attr);
	if (err) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for BUSFREQ");
		return err;
	}

	cpu_op_tbl = get_cpu_op(&cpu_op_nr);
	low_bus_freq_mode = 0;
	high_bus_freq_mode = 1;
	med_bus_freq_mode = 0;
	bus_freq_scaling_is_active = 0;
	bus_freq_scaling_initialized = 1;

	if (cpu_is_mx6q()) {
		ddr_low_rate = LPAPM_CLK;
		ddr_med_rate = DDR_MED_CLK;
		ddr_normal_rate = DDR3_NORMAL_CLK;
	}
	if (cpu_is_mx6dl()) {
		ddr_low_rate = LPAPM_CLK;
		ddr_normal_rate = ddr_med_rate = DDR_MED_CLK;
	}

	INIT_DELAYED_WORK(&low_bus_freq_handler, reduce_bus_freq_handler);

	mutex_init(&bus_freq_mutex);

	init_mmdc_settings();

	return 0;
}

static struct platform_driver busfreq_driver = {
	.driver = {
		   .name = "imx_busfreq",
		},
	.probe = busfreq_probe,
	.suspend = busfreq_suspend,
	.resume = busfreq_resume,
};

/*!
 * Initialise the busfreq_driver.
 *
 * @return  The function always returns 0.
 */

static int __init busfreq_init(void)
{
	if (platform_driver_register(&busfreq_driver) != 0) {
		printk(KERN_ERR "busfreq_driver register failed\n");
		return -ENODEV;
	}

	printk(KERN_INFO "Bus freq driver module loaded\n");
	return 0;
}

static void __exit busfreq_cleanup(void)
{
	sysfs_remove_file(&busfreq_dev->kobj, &dev_attr_enable.attr);

	/* Unregister the device structure */
	platform_driver_unregister(&busfreq_driver);
	bus_freq_scaling_initialized = 0;
}

late_initcall(busfreq_init);
module_exit(busfreq_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("BusFreq driver");
MODULE_LICENSE("GPL");
