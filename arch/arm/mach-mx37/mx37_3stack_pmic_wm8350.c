/*
 * mx37-3stack-pmic-wm8350.c  --  i.MX37 3STACK Driver for Wolfson WM8350 PMIC
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Copyright 2008-2009 Freescale Semiconductor Inc.
 *
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/wm8350/audio.h>
#include <linux/mfd/wm8350/core.h>
#include <linux/mfd/wm8350/pmic.h>
#include <linux/mfd/wm8350/gpio.h>
#include <linux/mfd/wm8350/bl.h>
#include <mach/irqs.h>

#include "iomux.h"

/* CPU */
static struct regulator_consumer_supply dcdc1_consumers[] = {
	{
	 .supply = "cpu_vcc",
	 }
};

static struct regulator_consumer_supply dcdc3_consumers[] = {
	{
	 .supply = "AVDD",
	 .dev_name = "1-001a",
	 },
	{
	 .supply = "HPVDD",
	 .dev_name = "1-001a",
	 },
};

static struct regulator_init_data dcdc1_data = {
	.constraints = {
			.name = "DCDC1",
			.min_uV = 850000,
			.max_uV = 1200000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_FAST,
			.state_mem = {
				      .uV = 1050000,
				      .mode = REGULATOR_MODE_NORMAL,
				      .enabled = 1,
				      },
			.initial_state = PM_SUSPEND_MEM,
			.always_on = 1,
			.boot_on = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(dcdc1_consumers),
	.consumer_supplies = dcdc1_consumers,
};

/* MX37 LP */
static struct regulator_init_data dcdc4_data = {
	.constraints = {
			.name = "DCDC4",
			.min_uV = 1000000,
			.max_uV = 1250000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
			REGULATOR_MODE_FAST,
			.state_mem = {
				      .uV = 1250000,
				      .mode = REGULATOR_MODE_NORMAL,
				      .enabled = 1,
				      },
			.initial_state = PM_SUSPEND_MEM,
			.always_on = 1,
			.boot_on = 1,
			},
};

/* DDR RAM */
static struct regulator_init_data dcdc6_data = {
	.constraints = {
			.name = "DCDC6",
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.state_mem = {
				      .uV = 1800000,
				      .mode = REGULATOR_MODE_NORMAL,
				      .enabled = 1,
				      },
			.state_disk = {
				       .mode = REGULATOR_MODE_NORMAL,
				       .enabled = 0,
				       },
			.always_on = 1,
			.boot_on = 1,
			.initial_state = PM_SUSPEND_MEM,
			},
};

static struct regulator_init_data dcdc3_data = {
	.constraints = {
			.name = "DCDC3",
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.apply_uV = 1,
			},
	.num_consumer_supplies = ARRAY_SIZE(dcdc3_consumers),
	.consumer_supplies = dcdc3_consumers,
};

static struct regulator_init_data ldo1_data = {
	.constraints = {
			.name = "LDO1",
			.min_uV = 2800000,
			.max_uV = 2800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.apply_uV = 1,
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data ldo2_data = {
	.constraints = {
			.name = "LDO2",
			.min_uV = 2500000,
			.max_uV = 2500000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.apply_uV = 1,
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data ldo3_data = {
	.constraints = {
			.name = "LDO3",
			.min_uV = 1200000,
			.max_uV = 1200000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.apply_uV = 1,
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data ldo4_data = {
	.constraints = {
			.name = "LDO4",
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.apply_uV = 1,
			.always_on = 1,
			.boot_on = 1,
			},
};

static struct regulator_init_data isinka_data = {
	.constraints = {
			.name = "ISINKA",
			.min_uA = 0,
			.max_uA = 225000,
			.valid_ops_mask = REGULATOR_CHANGE_CURRENT,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			},
};

static struct regulator_init_data dcdc5_data = {
	.constraints = {
			.name = "DCDC5",
			.min_uV = 0,
			.max_uV = 5000000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			},
};

static struct regulator_init_data dcdc2_data = {
	.constraints = {
			.name = "DCDC2",
			.min_uV = 0,
			.max_uV = 5000000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			},
};

static void wm8350_nop_release(struct device *dev)
{
	/* Nothing */
}

static int wm8350_check_fb(struct fb_info *info)
{
	return (to_platform_device(info->device)->id == 0);
}

struct wm8350_bl_platform_data wm8350_bl_data = {
	.isink = WM8350_ISINK_A,
	.dcdc = WM8350_DCDC_5,
	.voltage_ramp = WM8350_DC5_RMP_20V,
	.retries = 5,
	.max_brightness = 63,
	.power = FB_BLANK_UNBLANK,
	.brightness = 50,
	.check_fb = wm8350_check_fb,
};

static struct platform_device mxc_wm8350_devices[] = {
	{
	 .name = "wm8350-bl",
	 .id = 2,
	 .dev = {
		 .release = wm8350_nop_release,
		 .platform_data = &wm8350_bl_data,
		 },
	 },
};

static struct wm8350_audio_platform_data imx_3stack_wm8350_setup = {
	.vmid_discharge_msecs = 1000,
	.drain_msecs = 30,
	.cap_discharge_msecs = 700,
	.vmid_charge_msecs = 700,
	.vmid_s_curve = WM8350_S_CURVE_SLOW,
	.dis_out4 = WM8350_DISCHARGE_SLOW,
	.dis_out3 = WM8350_DISCHARGE_SLOW,
	.dis_out2 = WM8350_DISCHARGE_SLOW,
	.dis_out1 = WM8350_DISCHARGE_SLOW,
	.vroi_out4 = WM8350_TIE_OFF_500R,
	.vroi_out3 = WM8350_TIE_OFF_500R,
	.vroi_out2 = WM8350_TIE_OFF_500R,
	.vroi_out1 = WM8350_TIE_OFF_500R,
	.vroi_enable = 0,
	.codec_current_on = WM8350_CODEC_ISEL_1_0,
	.codec_current_standby = WM8350_CODEC_ISEL_0_5,
	.codec_current_charge = WM8350_CODEC_ISEL_1_5,
};

struct mxc_audio_platform_data imx_3stack_audio_platform_data = {
	.ssi_num = 2,
	.src_port = 2,
	.ext_port = 5,
};

static struct platform_device *imx_snd_device;

static int mx37_wm8350_init(struct wm8350 *wm8350)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(mxc_wm8350_devices); i++) {
		if (platform_device_register(&mxc_wm8350_devices[i]) < 0)
			dev_err(&mxc_wm8350_devices[i].dev,
				"Unable to register WM8350 device\n");
	}

	wm8350->pmic.isink_A_dcdc = WM8350_DCDC_5;

	/*Note: Needs to be moved into a regulator function. */
	/* Configuring -- GPIO 7 pin */
	if (wm8350_gpio_config(wm8350, 7, WM8350_GPIO_DIR_OUT, 0,
			       WM8350_GPIO_ACTIVE_LOW, WM8350_GPIO_PULL_NONE,
			       WM8350_GPIO_INVERT_OFF,
			       WM8350_GPIO_DEBOUNCE_OFF) == 0)
		wm8350_set_bits(wm8350, WM8350_GPIO_PIN_STATUS, 1 << 7);
	else
		printk(KERN_ERR "Error in setting Wolfson GPIO pin 7 \n");
	/* enable gpio4:USB_VBUS_EN */
	ret =
	    wm8350_gpio_config(wm8350, 4, WM8350_GPIO_DIR_IN,
			       WM8350_GPIO4_MR_IN, WM8350_GPIO_ACTIVE_HIGH,
			       WM8350_GPIO_PULL_UP, WM8350_GPIO_INVERT_OFF,
			       WM8350_GPIO_DEBOUNCE_OFF);
	if (ret)
		printk(KERN_ERR "Error in setting USB VBUS enable pin\n");

	wm8350_register_regulator(wm8350, WM8350_DCDC_1, &dcdc1_data);
	wm8350_register_regulator(wm8350, WM8350_DCDC_2, &dcdc2_data);
	wm8350_register_regulator(wm8350, WM8350_DCDC_3, &dcdc3_data);
	wm8350_register_regulator(wm8350, WM8350_DCDC_4, &dcdc4_data);
	wm8350_register_regulator(wm8350, WM8350_DCDC_5, &dcdc5_data);
	wm8350_register_regulator(wm8350, WM8350_DCDC_6, &dcdc6_data);
	wm8350_register_regulator(wm8350, WM8350_LDO_1, &ldo1_data);
	wm8350_register_regulator(wm8350, WM8350_LDO_2, &ldo2_data);
	wm8350_register_regulator(wm8350, WM8350_LDO_3, &ldo3_data);
	wm8350_register_regulator(wm8350, WM8350_LDO_4, &ldo4_data);
	wm8350_register_regulator(wm8350, WM8350_ISINK_A, &isinka_data);

	/* register sound */
	pr_info("Registering imx37_snd_device");
	wm8350->codec.platform_data = &imx_3stack_wm8350_setup;

	imx_snd_device = platform_device_alloc("wm8350-imx-3stack-audio", -1);
	if (!imx_snd_device) {
		ret = -ENOMEM;
		goto err;
	}
	imx_3stack_audio_platform_data.priv = wm8350;

	imx_snd_device->dev.platform_data = &imx_3stack_audio_platform_data;
	ret = platform_device_add(imx_snd_device);
	if (ret)
		goto snd_err;

	return 0;

snd_err:
	platform_device_put(imx_snd_device);

err:
	kfree(wm8350->reg_cache);
	return ret;
}

struct wm8350_platform_data __initdata mx37_wm8350_pdata = {
	.init = mx37_wm8350_init,
};

static struct i2c_board_info __initdata wm8350_i2c_device = {
	I2C_BOARD_INFO("wm8350", 0x1a),
	.platform_data = &mx37_wm8350_pdata,
	.irq = IOMUX_TO_IRQ(MX37_PIN_GPIO1_4),
};

static __init int mxc_init_i2c(void)
{
	i2c_register_board_info(1, &wm8350_i2c_device, 1);
	return 0;
}

subsys_initcall(mxc_init_i2c);

static __init int wm8350_regulator_init(void)
{
	int i = 0;
	int ret = 0;
	struct regulator *regulator;
	char *wm8350_global_regulator[] = {
		"DCDC1",
		"DCDC3",
		"DCDC4",
		"DCDC6",
		"LDO3",
	};

	/* for board v2.0 later, do nothing here */
	if (board_is_rev(BOARD_REV_2))
		return 0;
	while ((i < ARRAY_SIZE(wm8350_global_regulator)) &&
		!IS_ERR_VALUE(
			(unsigned long)(regulator =
					regulator_get(NULL,
						wm8350_global_regulator
						[i])))) {
		regulator_enable(regulator);
		if (strcmp(wm8350_global_regulator[i], "DCDC4") == 0)
			ret =
			    regulator_set_voltage(regulator, 1250000, 1250000);
		else if (strcmp(wm8350_global_regulator[i], "DCDC1") == 0) {
			ret =
			    regulator_set_voltage(regulator, 1050000, 1050000);
			regulator_set_mode(regulator, REGULATOR_MODE_FAST);
		}
		i++;
	}
	return ret;
}

late_initcall(wm8350_regulator_init);
