/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#define LP_LOW_VOLTAGE		1050000
#define LP_NORMAL_VOLTAGE		1250000
#define LP_APM_CLK			24000000
#define NAND_LP_APM_CLK			12000000
#define AXI_A_NORMAL_CLK		166250000
#define AXI_A_CLK_NORMAL_DIV		4
#define AXI_B_CLK_NORMAL_DIV		5
#define AHB_CLK_NORMAL_DIV		AXI_B_CLK_NORMAL_DIV
#define EMI_SLOW_CLK_NORMAL_DIV		AXI_B_CLK_NORMAL_DIV
#define NFC_CLK_NORMAL_DIV		4
#define SPIN_DELAY	1000000 /* in nanoseconds */
#define DDR_TYPE_DDR3		0x0
#define DDR_TYPE_DDR2		0x1

DEFINE_SPINLOCK(ddr_freq_lock);

unsigned long lp_normal_rate;
unsigned long lp_med_rate;
unsigned long ddr_normal_rate;
unsigned long ddr_med_rate;
unsigned long ddr_low_rate;

struct regulator *pll_regulator;

struct regulator *lp_regulator;
int low_bus_freq_mode;
int high_bus_freq_mode;
int med_bus_freq_mode;

int bus_freq_scaling_initialized;
char *lp_reg_id;

static struct device *busfreq_dev;
static int busfreq_suspended;

/* True if bus_frequency is scaled not using DVFS-PER */
int bus_freq_scaling_is_active;

int cpu_op_nr;
int lp_high_freq;
int lp_med_freq;

struct workqueue_struct *voltage_wq;
struct completion voltage_change_cmpl;

int low_freq_bus_used(void);
void set_ddr_freq(int ddr_freq);

extern struct cpu_op *(*get_cpu_op)(int *op);
extern void __iomem *ccm_base;
extern void __iomem *databahn_base;
extern int update_ddr_freq(int ddr_rate);


struct mutex bus_freq_mutex;

struct timeval start_time;
struct timeval end_time;

int set_low_bus_freq(void)
{
	return 0;
}

int set_high_bus_freq(int high_bus_freq)
{
	return 0;
}

void exit_lpapm_mode_mx6q(int high_bus_freq)
{

}


void set_ddr_freq(int ddr_rate)
{

}

int low_freq_bus_used(void)
{
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

module_init(busfreq_init);
module_exit(busfreq_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("BusFreq driver");
MODULE_LICENSE("GPL");
