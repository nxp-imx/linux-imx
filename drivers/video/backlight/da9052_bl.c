/*
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * da9052_bl.c: Backlight driver for DA9052
 */

#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#include <linux/delay.h>

#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/bl.h>


#define DRIVER_NAME		"da9052-backlight"
#define DRIVER_NAME1		"WLED-1"
#define DRIVER_NAME2		"WLED-2"
#define DRIVER_NAME3		"WLED-3"

/* These flags define if Backlight LEDs are present */
/* Set the following macros to 1, if LEDs are present. Otherwise set to 0 */
#define DA9052_LED1_PRESENT	1
#define DA9052_LED2_PRESENT	1
#define DA9052_LED3_PRESENT	1

#define DA9052_MAX_BRIGHTNESS	0xFF

struct da9052_backlight_data {
	struct device *da9052_dev;
	int current_brightness;
	struct da9052 *da9052;

	int is_led1_present;
	int is_led2_present;
	int is_led3_present;
};

enum da9052_led_number {
	LED1 = 1,
	LED2,
	LED3,
};

static int da9052_backlight_brightness_set(struct da9052_backlight_data *data,
			int brightness, enum da9052_led_number led)
{
	/*
	 * Mechanism for brightness control:
	 * For brightness control, current is used.
	 * PWM feature is not used.
	 * To use PWM feature, a fixed value of current should be defined.
	 */

	int ret = 0;
	unsigned int led_ramp_bit;
	unsigned int led_current_register;
	unsigned int led_current_sink_bit;
	unsigned int led_boost_en_bit;
	struct da9052_ssc_msg msg;

	switch (led) {
	case LED1:
		led_ramp_bit = DA9052_LEDCONT_LED1RAMP;
		led_current_register = DA9052_LED1CONF_REG;
		led_current_sink_bit = DA9052_LEDCONT_LED1EN;
		led_boost_en_bit = DA9052_BOOST_LED1INEN;
	break;
	case LED2:
		led_ramp_bit = DA9052_LEDCONT_LED2RAMP;
		led_current_register = DA9052_LED2CONF_REG;
		led_current_sink_bit = DA9052_LEDCONT_LED2EN;
		led_boost_en_bit = DA9052_BOOST_LED2INEN;
	break;
	case LED3:
		led_ramp_bit = DA9052_LEDCONT_LED3RAMP;
		led_current_register = DA9052_LED3CONF_REG;
		led_current_sink_bit = DA9052_LEDCONT_LED3EN;
		led_boost_en_bit = DA9052_BOOST_LED3INEN;
	break;
	default:
		return -EIO;
	}

	/*
		1. Configure the boost register
		2. Configure the LED _CONT register
		3. Configure the LEDx_CONF registers to the brightness value.
	*/
	msg.addr = DA9052_BOOST_REG;
	msg.data = 0x3F;
	if (brightness) {
		da9052_lock(data->da9052);
		ret = data->da9052->write(data->da9052, &msg);
		if (ret) {
			da9052_unlock(data->da9052);
			return ret;
		}
		da9052_unlock(data->da9052);
	}

	msg.addr = DA9052_LEDCONT_REG;
	msg.data = 0xFF;
	if (brightness) {
		da9052_lock(data->da9052);
		ret = data->da9052->write(data->da9052, &msg);
		if (ret) {
			da9052_unlock(data->da9052);
			return ret;
		}
		da9052_unlock(data->da9052);
	}

	msg.addr = led_current_register;
	msg.data = 0;
	/* Write to the DA9052 register */
	da9052_lock(data->da9052);
	ret = data->da9052->write(data->da9052, &msg);
	if (ret) {
		da9052_unlock(data->da9052);
		return ret;
	}
	da9052_unlock(data->da9052);
	msleep(20);
	msg.data = brightness;
	/* Write to the DA9052 register */
	da9052_lock(data->da9052);
	ret = data->da9052->write(data->da9052, &msg);
	if (ret) {
		da9052_unlock(data->da9052);
		return ret;
	}
	da9052_unlock(data->da9052);

	return 0;
}

static int da9052_backlight_set(struct backlight_device *bl, int brightness)
{
	struct da9052_backlight_data *data = bl_get_data(bl);
	int ret = 0;
	/* Check for LED1 */
	if (1 == data->is_led1_present) {
		ret = da9052_backlight_brightness_set(data, brightness, LED1);
		if (ret)
			return ret;
	}
	/* Check for LED2 */
	if (1 == data->is_led2_present) {
		ret = da9052_backlight_brightness_set(data, brightness, LED2);
		if (ret)
			return ret;
	}
	/* Check for LED3 */
	if (1 == data->is_led3_present) {
		ret = da9052_backlight_brightness_set(data, brightness, LED3);
		if (ret)
			return ret;
	}

	data->current_brightness = brightness;
	return 0;
}

static int da9052_init_WLED(struct da9052_backlight_data *data,
			enum da9052_led_number led)
{
	int ret = 0;
	unsigned int led_current_register;
	struct da9052_ssc_msg msg;

	switch (led) {
	case LED1:
		led_current_register = DA9052_LED1CONF_REG;
	break;
	case LED2:
		led_current_register = DA9052_LED2CONF_REG;
	break;
	case LED3:
		led_current_register = DA9052_LED3CONF_REG;
	break;
	default:
		return -EIO;
	}

	msg.addr = DA9052_BOOST_REG;
	msg.data = 0x00;
	da9052_lock(data->da9052);
	ret = data->da9052->write(data->da9052, &msg);
	if (ret) {
		da9052_unlock(data->da9052);
		return ret;
	}
	da9052_unlock(data->da9052);

	msg.addr = DA9052_LEDCONT_REG;
	msg.data = 0x00;
	da9052_lock(data->da9052);
	ret = data->da9052->write(data->da9052, &msg);
	if (ret) {
		da9052_unlock(data->da9052);
		return ret;
	}
	da9052_unlock(data->da9052);

	msg.addr = led_current_register;
	msg.data = 0;
	da9052_lock(data->da9052);
	ret = data->da9052->write(data->da9052, &msg);
	da9052_unlock(data->da9052);
	return ret;
}

static int da9052_backlight_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;
	return da9052_backlight_set(bl, brightness);
}

static int da9052_backlight_get_brightness(struct backlight_device *bl)
{
	struct da9052_backlight_data *data = bl_get_data(bl);
	return data->current_brightness;
}

struct backlight_ops da9052_backlight_ops = {
	.update_status	= da9052_backlight_update_status,
	.get_brightness	= da9052_backlight_get_brightness,
};

static int da9052_backlight_probe1(struct platform_device *pdev)
{
	struct da9052_backlight_data *data;
	struct backlight_device *bl;
	int ret = 0;
	struct da9052 *da9052 = dev_get_drvdata(pdev->dev.parent);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;
	data->da9052_dev = pdev->dev.parent;
	data->da9052	= da9052;
	data->current_brightness = 0;
	data->is_led1_present = DA9052_LED1_PRESENT;

	/* Init the WLED-1 bank */
	ret = da9052_init_WLED(data, LED1);
	if (ret)
		return ret;

	bl = backlight_device_register(pdev->name, data->da9052_dev,
			data, &da9052_backlight_ops);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		kfree(data);
		return PTR_ERR(bl);
	}

	bl->props.max_brightness = DA9052_MAX_BRIGHTNESS;
	bl->props.brightness = 0;

	platform_set_drvdata(pdev, bl);
	backlight_update_status(bl);

	return 0;
}
static int da9052_backlight_probe2(struct platform_device *pdev)
{
	struct da9052_backlight_data *data;
	struct backlight_device *bl;
	struct da9052 *da9052 = dev_get_drvdata(pdev->dev.parent);
	int ret = 0;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;
	data->da9052_dev = pdev->dev.parent;
	data->da9052	= da9052;
	data->current_brightness = 0;
	data->is_led2_present = DA9052_LED2_PRESENT;

	/* Init the WLED-2 bank */
	ret = da9052_init_WLED(data, LED2);
	if (ret)
		return ret;

	bl = backlight_device_register(pdev->name, data->da9052_dev,
			data, &da9052_backlight_ops);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		kfree(data);
		return PTR_ERR(bl);
	}

	bl->props.max_brightness = DA9052_MAX_BRIGHTNESS;
	bl->props.brightness = 0;

	platform_set_drvdata(pdev, bl);
	backlight_update_status(bl);

	return 0;
}
static int da9052_backlight_probe3(struct platform_device *pdev)
{
	struct da9052_backlight_data *data;
	struct backlight_device *bl;
	struct da9052 *da9052 = dev_get_drvdata(pdev->dev.parent);
	int ret = 0;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;
	data->da9052_dev = pdev->dev.parent;
	data->da9052	= da9052;
	data->current_brightness = 0;
	data->is_led3_present = DA9052_LED3_PRESENT;

	/* Init the WLED-2 bank */
	ret = da9052_init_WLED(data, LED3);
	if (ret)
		return ret;

	bl = backlight_device_register(pdev->name, data->da9052_dev,
			data, &da9052_backlight_ops);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		kfree(data);
		return PTR_ERR(bl);
	}

	bl->props.max_brightness = DA9052_MAX_BRIGHTNESS;
	bl->props.brightness = 0;

	platform_set_drvdata(pdev, bl);

	backlight_update_status(bl);
	return 0;
}

static int da9052_backlight_remove1(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct da9052_backlight_data *data = bl_get_data(bl);
	int ret = 0;

	/* Switch off the WLED-1 */
	ret = da9052_init_WLED(data, LED1);
	if (ret)
		return ret;

	backlight_device_unregister(bl);
	kfree(data);
	return 0;
}

static int da9052_backlight_remove2(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct da9052_backlight_data *data = bl_get_data(bl);
	int ret = 0;

	/* Switch off the WLED-2 */
	ret = da9052_init_WLED(data, LED2);
	if (ret)
		return ret;

	backlight_device_unregister(bl);
	kfree(data);
	return 0;
}
static int da9052_backlight_remove3(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct da9052_backlight_data *data = bl_get_data(bl);
	int ret;

	/* Switch off the WLED-3 */
	ret = da9052_init_WLED(data, LED3);
	if (ret)
		return ret;

	backlight_device_unregister(bl);
	kfree(data);
	return 0;
}

static struct platform_driver da9052_backlight_driver1 = {
	.driver		= {
		.name	= DRIVER_NAME1,
		.owner	= THIS_MODULE,
	},
	.probe		= da9052_backlight_probe1,
	.remove		= da9052_backlight_remove1,
};
static struct platform_driver da9052_backlight_driver2 = {
	.driver		= {
		.name	= DRIVER_NAME2,
		.owner	= THIS_MODULE,
	},
	.probe		= da9052_backlight_probe2,
	.remove		= da9052_backlight_remove2,
};
static struct platform_driver da9052_backlight_driver3 = {
	.driver		= {
		.name	= DRIVER_NAME3,
		.owner	= THIS_MODULE,
	},
	.probe		= da9052_backlight_probe3,
	.remove		= da9052_backlight_remove3,
};

static int __init da9052_backlight_init(void)
{
	s32 ret;
	ret = platform_driver_register(&da9052_backlight_driver1);
	if (ret)
		return ret;
	ret = platform_driver_register(&da9052_backlight_driver2);
	if (ret)
		return ret;

	ret = platform_driver_register(&da9052_backlight_driver3);
	if (ret)
		return ret;

	return ret;
}
module_init(da9052_backlight_init);

static void __exit da9052_backlight_exit(void)
{
	platform_driver_unregister(&da9052_backlight_driver1);
	platform_driver_unregister(&da9052_backlight_driver2);
	platform_driver_unregister(&da9052_backlight_driver3);
}
module_exit(da9052_backlight_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <dchen@diasemi.com>");
MODULE_DESCRIPTION("Backlight driver for Dialog DA9052 PMIC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);

