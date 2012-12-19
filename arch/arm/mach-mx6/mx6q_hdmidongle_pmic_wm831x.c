/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/io.h>

#ifdef CONFIG_MFD_WM831X
#include <linux/mfd/wm831x/core.h>
#include <linux/mfd/wm831x/pdata.h>
#include <linux/mfd/wm831x/regulator.h>
#include <linux/mfd/wm831x/gpio.h>
#include <linux/mfd/wm831x/status.h>
#include <mach/system.h>

extern u32 enable_ldo_mode;

#ifdef CONFIG_MX6_INTER_LDO_BYPASS
/* 1.3, 1.3 1.5 */
#define WM831X_DC1_ON_CONFIG_VAL            (0x40<<WM831X_DC1_ON_VSEL_SHIFT)
#define WM831X_DC2_ON_CONFIG_VAL            (0x40<<WM831X_DC2_ON_VSEL_SHIFT)
#define WM831X_DC3_ON_CONFIG_VAL            (0x1A<<WM831X_DC3_ON_VSEL_SHIFT)
#else
/* 1.375, 1.375. 1.5 */

#define WM831X_DC1_ON_CONFIG_VAL            (0x44<<WM831X_DC1_ON_VSEL_SHIFT)
#define WM831X_DC2_ON_CONFIG_VAL            (0x44<<WM831X_DC2_ON_VSEL_SHIFT)
#define WM831X_DC3_ON_CONFIG_VAL            (0x1A<<WM831X_DC3_ON_VSEL_SHIFT)

#endif

#define WM831X_DC1_DVS_MODE_VAL	(0x02<<WM831X_DC1_DVS_SRC_SHIFT)
#define WM831X_DC2_DVS_MODE_VAL	(0x02<<WM831X_DC2_DVS_SRC_SHIFT)

#define WM831X_DC1_DVS_CONTROL_VAL	(0x20<<WM831X_DC1_DVS_VSEL_SHIFT)
#define WM831X_DC2_DVS_CONTROL_VAL	(0x20<<WM831X_DC2_DVS_VSEL_SHIFT)

#define WM831X_DC1_DVS_MASK	(WM831X_DC1_DVS_SRC_MASK|WM831X_DC1_DVS_VSEL_MASK)
#define WM831X_DC2_DVS_MASK	(WM831X_DC2_DVS_SRC_MASK|WM831X_DC1_DVS_VSEL_MASK)

#define WM831X_DC1_DVS_VAL	(WM831X_DC1_DVS_MODE_VAL|WM831X_DC1_DVS_CONTROL_VAL)
#define WM831X_DC2_DVS_VAL                   (WM831X_DC2_DVS_MODE_VAL|WM831X_DC2_DVS_CONTROL_VAL)

#define WM831X_GPN_FN_VAL_HW_EN	(0x0A<<WM831X_GPN_FN_SHIFT)
#define WM831X_GPN_FN_VAL_HW_CTL	(0x0C<<WM831X_GPN_FN_SHIFT)
#define WM831X_GPN_FN_VAL_DVS1	(0x08<<WM831X_GPN_FN_SHIFT)

#define WM831X_GPN_DIR_VAL	(0x1<<WM831X_GPN_DIR_SHIFT)
#define WM831X_GPN_PULL_VAL	(0x3<<WM831X_GPN_PULL_SHIFT)
#define WM831X_GPN_INT_MODE_VAL	(0x1<<WM831X_GPN_INT_MODE_SHIFT)
#define WM831X_GPN_POL_VAL	(0x1<<WM831X_GPN_POL_SHIFT)
#define WM831X_GPN_ENA_VAL	(0x1<<WM831X_GPN_ENA_SHIFT)

#define WM831X_GPIO7_8_9_MASK	(WM831X_GPN_DIR_MASK|WM831X_GPN_INT_MODE_MASK| \
						WM831X_GPN_PULL_MASK|WM831X_GPN_POL_MASK|WM831X_GPN_FN_MASK)


#define WM831X_GPIO7_VAL	(WM831X_GPN_DIR_VAL|WM831X_GPN_PULL_VAL|WM831X_GPN_INT_MODE_VAL| \
						WM831X_GPN_POL_VAL|WM831X_GPN_ENA_VAL|WM831X_GPN_FN_VAL_HW_EN)
#define WM831X_GPIO8_VAL	(WM831X_GPN_DIR_VAL|WM831X_GPN_PULL_VAL|WM831X_GPN_INT_MODE_VAL| \
						WM831X_GPN_POL_VAL|WM831X_GPN_ENA_VAL|WM831X_GPN_FN_VAL_HW_CTL)
#define WM831X_GPIO9_VAL	(WM831X_GPN_DIR_VAL|WM831X_GPN_PULL_VAL|WM831X_GPN_INT_MODE_VAL| \
						WM831X_GPN_POL_VAL|WM831X_GPN_ENA_VAL|WM831X_GPN_FN_VAL_DVS1)

#define WM831X_STATUS_LED_ON                  (0x1 << WM831X_LED_SRC_SHIFT)
#define WM831X_STATUS_LED_OFF                 (0x0 << WM831X_LED_SRC_SHIFT)

static int wm8326_post_init(struct wm831x *wm831x)
{
	wm831x_set_bits(wm831x, WM831X_DC1_ON_CONFIG, WM831X_DC1_ON_VSEL_MASK, WM831X_DC1_ON_CONFIG_VAL);
	wm831x_set_bits(wm831x, WM831X_DC2_ON_CONFIG, WM831X_DC2_ON_VSEL_MASK, WM831X_DC2_ON_CONFIG_VAL);
	wm831x_set_bits(wm831x, WM831X_DC3_ON_CONFIG, WM831X_DC3_ON_VSEL_MASK, WM831X_DC3_ON_CONFIG_VAL);

	wm831x_set_bits(wm831x, WM831X_DC1_DVS_CONTROL, WM831X_DC1_DVS_MASK, WM831X_DC1_DVS_VAL);
	wm831x_set_bits(wm831x, WM831X_DC2_DVS_CONTROL, WM831X_DC2_DVS_MASK, WM831X_DC2_DVS_VAL);

	wm831x_set_bits(wm831x, WM831X_GPIO7_CONTROL, WM831X_GPIO7_8_9_MASK, WM831X_GPIO7_VAL);
	wm831x_set_bits(wm831x, WM831X_GPIO8_CONTROL, WM831X_GPIO7_8_9_MASK, WM831X_GPIO8_VAL);
	wm831x_set_bits(wm831x, WM831X_GPIO9_CONTROL, WM831X_GPIO7_8_9_MASK, WM831X_GPIO9_VAL);

	wm831x_set_bits(wm831x, WM831X_STATUS_LED_1 , WM831X_LED_SRC_MASK, WM831X_STATUS_LED_OFF);
	wm831x_set_bits(wm831x, WM831X_STATUS_LED_2 , WM831X_LED_SRC_MASK, WM831X_STATUS_LED_ON);

#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	if (enable_ldo_mode == LDO_MODE_DEFAULT)
		enable_ldo_mode = LDO_MODE_BYPASSED;
#endif
	return 0;
}

#ifdef CONFIG_REGULATOR
/* ARM core */
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
static struct regulator_consumer_supply hdmidongle_vddarm_consumers[] = {
	{
		.supply     = "VDDCORE_DCDC1",
	}
};

static struct regulator_consumer_supply hdmidongle_vddsoc_consumers[] = {
	{
		.supply     = "VDDSOC_DCDC2",
	}
};
#endif

static struct regulator_init_data hdmidongle_vddarm_dcdc1 = {
	.constraints = {
		.name = "vdd_arm",
		.min_uV = 100000,
		.max_uV = 1500000,
		.min_uA = 0,
		.max_uA = 4000000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = 0,
		.always_on = 1,
		.boot_on = 1,
	},
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	.num_consumer_supplies = ARRAY_SIZE(hdmidongle_vddarm_consumers),
	.consumer_supplies = hdmidongle_vddarm_consumers,
#endif
};


static struct regulator_init_data hdmidongle_vddsoc_dcdc2 = {
	.constraints = {
		.name = "vdd_soc",
		.min_uV = 100000,
		.max_uV = 1500000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = 0,
		.always_on = 1,
		.boot_on = 1,
	},
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	.num_consumer_supplies = ARRAY_SIZE(hdmidongle_vddsoc_consumers),
	.consumer_supplies = hdmidongle_vddsoc_consumers,
#endif
};



static struct regulator_init_data hdmidongle_vddmem_1v5_dcdc3 = {
	.constraints = {
		.name = "vdd_mem_1v5",
		.min_uV = 1400000,
		.max_uV = 1500000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		.always_on = 1,
		.boot_on = 1,
	},
};
#endif

static struct wm831x_pdata hdmidongle_wm8326_pdata = {
#ifdef CONFIG_REGULATOR
	.dcdc = {
		&hdmidongle_vddarm_dcdc1,  /* DCDC1 */
		&hdmidongle_vddsoc_dcdc2,  /* DCDC2 */
	},
#endif
	.post_init = wm8326_post_init,
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
	I2C_BOARD_INFO("wm8326", 0x34),
	.platform_data = &hdmidongle_wm8326_pdata,
	},
};

int __init mx6q_hdmidongle_init_wm8326(void)
{
	return i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
}
#else
int __init mx6q_hdmidongle_init_wm8326(void) {};
#endif

