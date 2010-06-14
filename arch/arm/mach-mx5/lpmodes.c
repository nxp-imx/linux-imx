/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mx51_lpmodes.c
 *
 * @brief Driver for the Freescale Semiconductor MXC low power modes setup.
 *
 * MX51 is designed to play and video with minimal power consumption.
 * This driver enables the platform to enter and exit audio and video low
 * power modes.
 *
 * @ingroup PM
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <mach/clock.h>
#include <mach/hardware.h>
#include <linux/regulator/machine.h>

#define ARM_LP_CLK  166250000
#define GP_LPM_VOLTAGE 775000
#define GP_NORMAL_VOLTAGE 1050000

static int org_cpu_rate;
int lp_video_mode;
int lp_audio_mode;
static struct device *lpmode_dev;
struct regulator *gp_core;

void enter_lp_video_mode(void)
{
}

void exit_lp_video_mode(void)
{
}

void enter_lp_audio_mode(void)
{
	struct clk *tclk;
	int ret;

	struct clk *p_clk;
	struct clk *amode_parent_clk;


	tclk = clk_get(NULL, "ipu_clk");
	if (clk_get_usecount(tclk) != 0) {
		printk(KERN_INFO
		"Cannot enter AUDIO LPM mode - display is still active\n");
		return;
	}

	tclk = clk_get(NULL, "cpu_clk");
	org_cpu_rate = clk_get_rate(tclk);

#ifdef CHANGE_DDR2_TO_PLL2
	tclk = clk_get(NULL, "ddr_clk");
	clk_set_parent(tclk, clk_get(NULL, "axi_a_clk"));

	/* Set CPU clock to be derived from PLL2 instead of PLL1 */
	tclk = clk_get(NULL, "pll1_sw_clk");
	clk_set_parent(tclk, clk_get(NULL, "pll2"));
	clk_enable(tclk);

	tclk = clk_get(NULL, "ddr_clk");
	clk_set_parent(tclk, clk_get(NULL, "ddr_hf_clk"));
#endif

	/*Change the DDR freq to 133Mhz. */
	tclk = clk_get(NULL, "ddr_hf_clk");
	clk_set_rate(tclk, clk_round_rate(tclk, 133000000));

	tclk = clk_get(NULL, "cpu_clk");
	ret = clk_set_rate(tclk, ARM_LP_CLK);
	if (ret != 0)
		printk(KERN_DEBUG "cannot set CPU clock rate\n");
	clk_put(tclk);
	/* Set the voltage to 0.775v for the GP domain. */
	ret = regulator_set_voltage(gp_core, GP_LPM_VOLTAGE, GP_LPM_VOLTAGE);
	if (ret < 0)
		printk(KERN_DEBUG "COULD NOT SET GP VOLTAGE!!!\n");

	tclk = clk_get(NULL, "periph_apm_clk");
	amode_parent_clk = clk_get(NULL, "lp_apm");
	p_clk = clk_get_parent(tclk);
	/* Make sure osc_clk is the parent of lp_apm. */
	clk_set_parent(amode_parent_clk, clk_get(NULL, "osc"));
	/* Set the parent of periph_apm_clk to be lp_apm */
	clk_set_parent(tclk, amode_parent_clk);

	amode_parent_clk = tclk;

	tclk = clk_get(NULL, "main_bus_clk");
	p_clk = clk_get_parent(tclk);
	/* Set the parent of main_bus_clk to be periph_apm_clk */
	clk_set_parent(tclk, amode_parent_clk);

	clk_set_rate(clk_get(NULL, "axi_a_clk"), 24000000);
	clk_set_rate(clk_get(NULL, "axi_b_clk"), 24000000);
	clk_set_rate(clk_get(NULL, "ahb_clk"), 24000000);
	clk_set_rate(clk_get(NULL, "emi_slow_clk"), 24000000);
	clk_set_rate(clk_get(NULL, "nfc_clk"), 12000000);

	/* disable PLL3 */
	tclk = clk_get(NULL, "pll3");
	if (tclk->usecount == 1)
		clk_disable(tclk);

	/* disable PLL2 */
	tclk = clk_get(NULL, "pll2");
	if (tclk->usecount == 1)
		clk_disable(tclk);

	/* disable PLL1 */
	tclk = clk_get(NULL, "pll1_main_clk");
	if (tclk->usecount == 1)
		clk_disable(tclk);

	lp_audio_mode = 1;
}

void exit_lp_audio_mode(void)
{
	struct clk *tclk;
	struct clk *p_clk;
	struct clk *rmode_parent_clk;
	int ret;

	/* Set the voltage to 1.05v for the GP domain. */
	ret = regulator_set_voltage(gp_core,
				    GP_NORMAL_VOLTAGE, GP_NORMAL_VOLTAGE);
	if (ret < 0)
		printk(KERN_DEBUG "COULD NOT SET GP VOLTAGE!!!\n");

	rmode_parent_clk = clk_get(NULL, "pll2");
	clk_enable(rmode_parent_clk);

	tclk = clk_get(NULL, "main_bus_clk");
	p_clk = clk_get_parent(tclk);

	/* Set the dividers before setting the parent clock. */
	clk_set_rate(clk_get(NULL, "axi_a_clk"), 6000000);
	clk_set_rate(clk_get(NULL, "axi_b_clk"), 4800000);
	clk_set_rate(clk_get(NULL, "ahb_clk"), 4800000);
	clk_set_rate(clk_get(NULL, "emi_slow_clk"), 4800000);
	clk_set_rate(clk_get(NULL, "nfc_clk"), 1200000);
	/* Set the parent of main_bus_clk to be pll2 */
	clk_set_parent(tclk, rmode_parent_clk);

#ifdef CHANGE_DDR2_TO_PLL2
	tclk = clk_get(NULL, "ddr_clk");
	clk_set_parent(tclk, clk_get(NULL, "axi_a_clk"));

	/* Set CPU clock to be derived from PLL1 instead of PLL2 */
	tclk = clk_get(NULL, "pll1_sw_clk");
	clk_set_parent(tclk, clk_get(NULL, "pll1_main_clk"));
	clk_disable(tclk);

	tclk = clk_get(NULL, "ddr_clk");
	clk_set_parent(tclk, clk_get(NULL, "ddr_hf_clk"));
#endif

	tclk = clk_get(NULL, "cpu_clk");
	ret = clk_set_rate(tclk, org_cpu_rate);
	if (ret != 0)
		printk(KERN_DEBUG "cannot set CPU clock rate\n");
	clk_put(tclk);

	tclk = clk_get(NULL, "cpu_clk");

	ret = clk_set_rate(tclk, org_cpu_rate);
	if (ret != 0)
		printk(KERN_DEBUG "cannot set CPU clock rate\n");


	/*Change the DDR freq to 200MHz*/
	tclk = clk_get(NULL, "ddr_hf_clk");
	clk_set_rate(tclk, clk_round_rate(tclk, 200000000));
	lp_audio_mode = 0;

}

static ssize_t lp_curr_mode(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	if (lp_video_mode)
		return sprintf(buf, "in lp_video_mode\n");
	else if (lp_audio_mode)
		return sprintf(buf, "in lp_audio_mode\n");
	else
		return sprintf(buf, "in normal mode\n");
}

static ssize_t set_lp_mode(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	printk(KERN_DEBUG "In set_lp_mode() \n");

	if (strstr(buf, "enable_lp_video") != NULL) {
		if (!lp_video_mode)
			enter_lp_video_mode();
	} else if (strstr(buf, "disable_lp_video") != NULL) {
		if (lp_video_mode)
			exit_lp_video_mode();
	} else if (strstr(buf, "enable_lp_audio") != NULL) {
		if (!lp_audio_mode)
			enter_lp_audio_mode();
	} else if (strstr(buf, "disable_lp_audio") != NULL) {
		if (lp_audio_mode)
			exit_lp_audio_mode();
	}
	return size;
}

static DEVICE_ATTR(lp_modes, 0644, lp_curr_mode, set_lp_mode);

/*!
 * This is the probe routine for the lp_mode driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 *
 */
static int __devinit mx51_lpmode_probe(struct platform_device *pdev)
{
	u32 res = 0;
	lpmode_dev = &pdev->dev;

	res = sysfs_create_file(&lpmode_dev->kobj, &dev_attr_lp_modes.attr);
	if (res) {
		printk(KERN_ERR
		"lpmode_dev: Unable to register sysdev entry for lpmode_dev");
		return res;
	}

	if (res != 0) {
		printk(KERN_ERR "lpmode_dev: Unable to start");
		return res;
	}
	gp_core = regulator_get(NULL, "SW1");
	lp_video_mode = 0;
	lp_audio_mode = 0;

	return 0;
}

static struct platform_driver mx51_lpmode_driver = {
	.driver = {
		   .name = "mx51_lpmode",
		   },
	.probe = mx51_lpmode_probe,
};

/*!
 * Initialise the mx51_lpmode_driver.
 *
 * @return  The function always returns 0.
 */

static int __init lpmode_init(void)
{
	if (platform_driver_register(&mx51_lpmode_driver) != 0) {
		printk(KERN_ERR "mx37_lpmode_driver register failed\n");
		return -ENODEV;
	}

	printk(KERN_INFO "LPMode driver module loaded\n");
	return 0;
}

static void __exit lpmode_cleanup(void)
{
	sysfs_remove_file(&lpmode_dev->kobj, &dev_attr_lp_modes.attr);

	/* Unregister the device structure */
	platform_driver_unregister(&mx51_lpmode_driver);
}

module_init(lpmode_init);
module_exit(lpmode_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("LPMode driver");
MODULE_LICENSE("GPL");
