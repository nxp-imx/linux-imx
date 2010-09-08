/*
 * mx35-3stack-pmic-mc9s08dz60.c -- i.MX35 3STACK Driver for MCU PMIC
 */
 /*
  * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
  */

 /*
  * The code contained herein is licensed under the GNU General Public
  * License. You may obtain a copy of the GNU General Public License
  * Version 2 or later at the following locations:
  *
  * http://www.opensource.org/licenses/gpl-license.html
  * http://www.gnu.org/copyleft/gpl.html
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/pmic_external.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/mc9s08dz60/core.h>

#include <mach/board-mx35_3ds.h>
#include <mach/hardware.h>

static struct regulator_init_data lcd_init = {
	.constraints = {
		.name = "LCD",
	}
};

static struct regulator_init_data wifi_init = {
	.constraints = {
		.name = "WIFI",
	}
};

static struct regulator_init_data hdd_init = {
	.constraints = {
		.name = "HDD",
	}
};

static struct regulator_init_data gps_init = {
	.constraints = {
		.name = "GPS",
	}
};

static struct regulator_init_data spkr_init = {
	.constraints = {
		.name = "SPKR",
	}
};

static int mc9s08dz60_regulator_init(struct mc9s08dz60 *mc9s08dz60)
{
	if (!board_is_rev(BOARD_REV_2))
		return 0;

	mc9s08dz60_register_regulator(
			mc9s08dz60, MC9S08DZ60_LCD, &lcd_init);
	mc9s08dz60_register_regulator(mc9s08dz60,
			MC9S08DZ60_WIFI, &wifi_init);
	mc9s08dz60_register_regulator(
			mc9s08dz60, MC9S08DZ60_HDD, &hdd_init);
	mc9s08dz60_register_regulator(
			mc9s08dz60, MC9S08DZ60_GPS, &gps_init);
	mc9s08dz60_register_regulator(mc9s08dz60,
				MC9S08DZ60_SPKR, &spkr_init);
	return 0;
}

static struct mc9s08dz60_platform_data mc9s08dz60_plat = {
	.init = mc9s08dz60_regulator_init,
};

static struct i2c_board_info __initdata mc9s08dz60_i2c_device = {
	I2C_BOARD_INFO("mc9s08dz60", 0x69),
	.platform_data = &mc9s08dz60_plat,
};

static struct resource mc9s08dz60_keypad_resource = {
	.start = MXC_PSEUDO_IRQ_KEYPAD,
	.end = MXC_PSEUDO_IRQ_KEYPAD,
	.flags = IORESOURCE_IRQ,
};

static struct platform_device mc9s08dz60_keypad_dev = {
	.name = "mc9s08dz60keypad",
	.num_resources = 1,
	.resource = &mc9s08dz60_keypad_resource,
};

int __init mx35_3stack_init_mc9s08dz60(void)
{
	int retval = 0;
	retval = i2c_register_board_info(0, &mc9s08dz60_i2c_device, 1);
	if (retval == 0)
		platform_device_register(&mc9s08dz60_keypad_dev);
	return retval;
}
