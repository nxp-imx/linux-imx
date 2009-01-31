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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/wm8350/wm8350.h>
#include <linux/regulator/wm8350/wm8350-pmic.h>
#include <linux/regulator/wm8350/wm8350-gpio.h>
#include <linux/regulator/wm8350/wm8350-bus.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/fb.h>

#include <mach/dma.h>
#include <mach/spba.h>
#include <mach/clock.h>
#include <mach/mxc.h>
#include "board-mx37_3stack.h"

static void wm8350_regulator_init(void)
{
	int i = 0;
	struct regulator *regulator;
	char *wm8350_global_regulator[] = {
		"DCDC1",
		"DCDC3",
		"DCDC4",
		"DCDC6",
		"LDO3",
	};

	/* for board v2.0 later, do nothing here*/
	if (board_is_mx37(BOARD_REV_2))
		return;
	while (!IS_ERR_VALUE((unsigned long)(regulator = regulator_get(NULL,
		wm8350_global_regulator[i])))) {
		regulator_enable(regulator);
		i++;
	}
}
late_initcall(wm8350_regulator_init);

/*
 * Set to 1 when testing battery that is connected otherwise spuriuos debug
 */
#define BATTERY 0

/* extern const char imx_3stack_audio[32]; */

struct mxc_audio_platform_data imx_3stack_audio_platform_data = {
	.ssi_num = 2,
	.src_port = 2,
	.ext_port = 5,
	.regulator1 = "DCDC6",
	.regulator2 = "DCDC3"
};

#ifdef NOT_PORTED_TO_IMX37_3STACK_YET

static void imx37_3stack_switch_handler(struct wm8350 *wm8350, int irq)
{
	printk("switch pressed %d\n", irq);
}
#endif
static struct platform_device *imx_snd_device;

void wm8350_free(struct wm8350 *wm8350)
{
#if BATTERY
	struct wm8350_power *power = &wm8350->power;
#endif

	wm8350_mask_irq(wm8350, WM8350_IRQ_GPIO(7));
	wm8350_free_irq(wm8350, WM8350_IRQ_GPIO(7));
	wm8350_mask_irq(wm8350, WM8350_IRQ_WKUP_ONKEY);
	wm8350_free_irq(wm8350, WM8350_IRQ_WKUP_ONKEY);

#if BATTERY
	wm8350_charger_enable(power, 0);
	wm8350_fast_charger_enable(power, 0);
#endif
	if (wm8350->nirq)
		free_irq(wm8350->nirq, wm8350);

	flush_scheduled_work();

	if (device_is_registered(&wm8350->pmic.dev))
		device_unregister(&wm8350->pmic.dev);
	if (device_is_registered(&wm8350->rtc.dev))
		device_unregister(&wm8350->rtc.dev);
	if (device_is_registered(&wm8350->wdg.dev))
		device_unregister(&wm8350->wdg.dev);
	if (device_is_registered(&wm8350->power.dev))
		device_unregister(&wm8350->power.dev);

	platform_device_unregister(imx_snd_device);
}

#if BATTERY
static int wm8350_init_battery(struct wm8350 *wm8350)
{
	struct wm8350_power *power = &wm8350->power;
	struct wm8350_charger_policy *policy = &power->policy;

	policy->eoc_mA = WM8350_CHG_EOC_mA(10);
	policy->charge_mV = WM8350_CHG_4_05V;
	policy->fast_limit_mA = WM8350_CHG_FAST_LIMIT_mA(400);
	policy->charge_timeout = WM8350_CHG_TIME_MIN(60);
	policy->trickle_start_mV = WM8350_CHG_TRICKLE_3_1V;
	policy->trickle_charge_mA = WM8350_CHG_TRICKLE_50mA;

	wm8350_charger_enable(power, 1);
	wm8350_fast_charger_enable(power, 1);
	return 0;
}
#endif

#ifdef NOT_PORTED_TO_IMX37_3STACK_YET
static int config_gpios(struct wm8350 *wm8350)
{
	/* power on */
	wm8350_gpio_config(wm8350, 0, WM8350_GPIO_DIR_IN,
			   WM8350_GPIO0_PWR_ON_IN, WM8350_GPIO_ACTIVE_LOW,
			   WM8350_GPIO_PULL_UP, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_ON);

	/* Sw3 --> PWR_OFF_GPIO3 */
	/* lg - TODO: GPIO1_0 to be pulled down */
	wm8350_gpio_config(wm8350, 3, WM8350_GPIO_DIR_IN,
			   WM8350_GPIO3_PWR_OFF_IN, WM8350_GPIO_ACTIVE_HIGH,
			   WM8350_GPIO_PULL_DOWN, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_ON);

	/* MR or MEMRST ????? */
	wm8350_gpio_config(wm8350, 4, WM8350_GPIO_DIR_IN,
			   WM8350_GPIO4_MR_IN, WM8350_GPIO_ACTIVE_HIGH,
			   WM8350_GPIO_PULL_DOWN, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_OFF);

	/* Hibernate -- GPIO 7 */
	wm8350_gpio_config(wm8350, 7, WM8350_GPIO_DIR_IN,
			   WM8350_GPIO7_HIBERNATE_IN, WM8350_GPIO_ACTIVE_HIGH,
			   WM8350_GPIO_PULL_DOWN, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_OFF);

	/* SDOUT */
	wm8350_gpio_config(wm8350, 6, WM8350_GPIO_DIR_OUT,
			   WM8350_GPIO6_SDOUT_OUT, WM8350_GPIO_ACTIVE_HIGH,
			   WM8350_GPIO_PULL_NONE, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_OFF);

	/* GPIO switch SW2 */
	wm8350_gpio_config(wm8350, 7, WM8350_GPIO_DIR_IN, WM8350_GPIO7_GPIO_IN,
			   WM8350_GPIO_ACTIVE_HIGH, WM8350_GPIO_PULL_DOWN,
			   WM8350_GPIO_INVERT_OFF, WM8350_GPIO_DEBOUNCE_ON);
	wm8350_register_irq(wm8350, WM8350_IRQ_GPIO(7),
			    imx37_3stack_switch_handler, NULL);
	wm8350_unmask_irq(wm8350, WM8350_IRQ_GPIO(7));

	/* PWR_FAIL */
	wm8350_gpio_config(wm8350, 8, WM8350_GPIO_DIR_OUT,
			   WM8350_GPIO8_VCC_FAULT_OUT, WM8350_GPIO_ACTIVE_LOW,
			   WM8350_GPIO_PULL_NONE, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_OFF);

	/* BATT Fault */
	wm8350_gpio_config(wm8350, 9, WM8350_GPIO_DIR_OUT,
			   WM8350_GPIO9_BATT_FAULT_OUT, WM8350_GPIO_ACTIVE_LOW,
			   WM8350_GPIO_PULL_NONE, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_OFF);

	return 0;
}

static int config_hibernate(struct wm8350 *wm8350)
{
	struct wm8350_pmic *pmic = &wm8350->pmic;

	/* dont assert RTS when hibernating */
	wm8350_set_bits(wm8350, WM8350_SYSTEM_HIBERNATE, WM8350_RST_HIB_MODE);

	/* set up hibernate voltages -- needs refining */
	wm8350_dcdc_set_image_voltage(pmic, WM8350_DCDC_1, 1400,	/*1200, *//* 1.0v for mx32 */
				      WM8350_DCDC_HIB_MODE_IMAGE,
				      WM8350_DCDC_HIB_SIG_REG);
	wm8350_dcdc_set_image_voltage(pmic, WM8350_DCDC_3, 2800,
				      WM8350_DCDC_HIB_MODE_IMAGE,
				      WM8350_DCDC_HIB_SIG_REG);
	wm8350_dcdc_set_image_voltage(pmic, WM8350_DCDC_4, 1800,
				      WM8350_DCDC_HIB_MODE_IMAGE,
				      WM8350_DCDC_HIB_SIG_REG);
	wm8350_dcdc_set_image_voltage(pmic, WM8350_DCDC_6, 1800,
				      WM8350_DCDC_HIB_MODE_IMAGE,
				      WM8350_DCDC_HIB_SIG_REG);
	wm8350_ldo_set_image_voltage(pmic, WM8350_LDO_1, 2800,
				     WM8350_LDO_HIB_MODE_IMAGE,
				     WM8350_LDO_HIB_SIG_REG);
	wm8350_ldo_set_image_voltage(pmic, WM8350_LDO_2, 3300,
				     WM8350_LDO_HIB_MODE_IMAGE,
				     WM8350_LDO_HIB_SIG_REG);
	wm8350_ldo_set_image_voltage(pmic, WM8350_LDO_3, 1500,
				     WM8350_LDO_HIB_MODE_IMAGE,
				     WM8350_LDO_HIB_SIG_REG);
	wm8350_ldo_set_image_voltage(pmic, WM8350_LDO_4, 2500,
				     WM8350_LDO_HIB_MODE_IMAGE,
				     WM8350_LDO_HIB_MODE_DIS);
	return 0;
}
#endif

struct regulation_constraints led_regulation_constraints = {
	.min_uA = 0,
	.max_uA = 230000,
	.valid_ops_mask = REGULATOR_CHANGE_CURRENT,
};
struct regulation_constraints dcdc1_regulation_constraints = {
	.min_uV = mV_to_uV(850),
	.max_uV = mV_to_uV(1200),
	.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
};
struct regulation_constraints dcdc4_regulation_constraints = {
	.min_uV = mV_to_uV(1000),
	.max_uV = mV_to_uV(1200),
	.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
};

static void set_regulator_constraints(struct wm8350 *wm8350)
{
	regulator_set_platform_constraints("DCDC1",
					   &dcdc1_regulation_constraints);
	regulator_set_platform_constraints("DCDC4",
					   &dcdc4_regulation_constraints);
	regulator_set_platform_constraints("ISINKA",
					   &led_regulation_constraints);
}

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

static inline void mxc_init_wm8350(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mxc_wm8350_devices); i++) {
		if (platform_device_register(&mxc_wm8350_devices[i]) < 0)
			dev_err(&mxc_wm8350_devices[i].dev,
				"Unable to register WM8350 device\n");
	}
}

int wm8350_init(struct wm8350 *wm8350)
{
	int ret = 0;

	/* register regulator and set constraints */
	wm8350_device_register_pmic(wm8350);
	set_regulator_constraints(wm8350);
#ifdef NOT_PORTED_TO_IMX37
	wm8350_device_register_rtc(wm8350);
	wm8350_device_register_wdg(wm8350);
	wm8350_device_register_power(wm8350);
#endif
	mxc_init_wm8350();

	/*Note: Needs to be moved into a regulator function. */
	/* Configuring -- GPIO 7 pin */
	if (wm8350_gpio_config(wm8350, 7, WM8350_GPIO_DIR_OUT, 0,
			       WM8350_GPIO_ACTIVE_LOW, WM8350_GPIO_PULL_NONE,
			       WM8350_GPIO_INVERT_OFF,
			       WM8350_GPIO_DEBOUNCE_OFF) == 0)
		wm8350_gpio_set_status(wm8350, 7, 1);
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

	/* register sound */
	printk("Registering imx37_snd_device");
	imx_snd_device = platform_device_alloc("wm8350-imx-3stack-audio", -1);
	if (!imx_snd_device) {
		ret = -ENOMEM;
		goto err;
	}
	imx_snd_device->dev.platform_data = &imx_3stack_audio_platform_data;
	platform_set_drvdata(imx_snd_device, &wm8350->audio);
	ret = platform_device_add(imx_snd_device);
	if (ret)
		goto snd_err;

	/* set up PMIC IRQ (active high) to i.MX32ADS */
#ifdef NOT_PORTED_TO_IMX37
	printk("Registering PMIC INT");
	INIT_WORK(&wm8350->work, wm8350_irq_work);
	wm8350_reg_unlock(wm8350);
	wm8350_set_bits(wm8350, WM8350_SYSTEM_CONTROL_1, WM8350_IRQ_POL);
	wm8350_reg_lock(wm8350);
	set_irq_type(MXC_PMIC_INT_LINE, IRQF_TRIGGER_RISING);
	ret = request_irq(MXC_PMIC_INT_LINE, wm8350_irq_handler,
			  IRQF_DISABLED, "wm8350-pmic", wm8350);
	if (ret != 0) {
		printk(KERN_ERR "wm8350: cant request irq %d\n",
		       MXC_PMIC_INT_LINE);
		goto err;
	}
	wm8350->nirq = MXC_PMIC_INT_LINE;
	printk("Configuring WM8350 GPIOS");
	config_gpios(wm8350);
	config_hibernate(wm8350);

	/* Sw1 --> PWR_ON */
	printk("Registering and unmasking the WM8350 wakeup key");
	wm8350_register_irq(wm8350, WM8350_IRQ_WKUP_ONKEY,
			    imx37_3stack_switch_handler);
	wm8350_unmask_irq(wm8350, WM8350_IRQ_WKUP_ONKEY);

	/* unmask all & clear sticky */
	printk("Unmasking WM8350 local interrupts");
	wm8350_reg_write(wm8350, WM8350_SYSTEM_INTERRUPTS_MASK, 0x0);
	schedule_work(&wm8350->work);
#endif

#if BATTERY
	/* not much use without a battery atm */
	wm8350_init_battery(wm8350);
#endif

	printk("Exiting normally from wm8350_init()");
	return ret;
      snd_err:
	platform_device_put(imx_snd_device);

      err:
	printk("wm8350_init() FAILED");
	kfree(wm8350->reg_cache);
	return ret;
}
