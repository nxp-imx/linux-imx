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
 * @file bus_freq.c
 *
 * @brief A common API for the Freescale Semiconductor i.MXC CPUfreq module
 * and DVFS CORE module.
 *
 * The APIs are for setting bus frequency to low or high.
 *
 * @ingroup PM
 */

#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <mach/clock.h>
#include <mach/hardware.h>
#include <linux/regulator/consumer.h>

struct clk *main_bus_clk;
struct clk *pll2;
struct clk *axi_a_clk;
struct clk *axi_b_clk;
struct clk *axi_c_clk;
struct clk *emi_core_clk;
struct clk *nfc_clk;
struct clk *ahb_clk;
struct clk *vpu_clk;
struct clk *vpu_core_clk;
struct clk *arm_axi_clk;
struct clk *ddr_clk;
struct clk *ipu_clk;
struct clk *periph_apm_clk;
struct clk *lp_apm;
struct clk *osc;
struct regulator *lp_regulator;
int low_bus_freq_mode;
int high_bus_freq_mode;
char *gp_reg_id = "SW1";
char *lp_reg_id = "SW2";

int set_low_bus_freq(void)
{
	return 0;
}

int set_high_bus_freq(void)
{
	return 0;
}

int low_freq_bus_used(void)
{
	return 0;
}

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
	low_bus_freq_mode = 0;
	high_bus_freq_mode = 0;

	return 0;
}

static struct platform_driver busfreq_driver = {
	.driver = {
		   .name = "busfreq",
		   },
	.probe = busfreq_probe,
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
	/* Unregister the device structure */
	platform_driver_unregister(&busfreq_driver);
}

module_init(busfreq_init);
module_exit(busfreq_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("BusFreq driver");
MODULE_LICENSE("GPL");

