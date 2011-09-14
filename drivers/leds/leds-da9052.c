/*
 * leds-da9052.c  --  LED Driver for Dialog DA9052
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * Author: Dialog Semiconductor Ltd <dchen@diasemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/led.h>

#define DRIVER_NAME		"da9052-leds"

#define DA9052_LED4_PRESENT	1
#define DA9052_LED5_PRESENT	1


struct da9052_led_data {
	struct led_classdev cdev;
	struct work_struct work;
	struct da9052 *da9052;
	int id;
	int new_brightness;
	int is_led4_present;
	int is_led5_present;
};

#define GPIO14_PIN		2 /* GPO Open Drain */
#define GPIO14_TYPE		0 /* VDD_IO1 */
#define GPIO14_MODE		1 /* Output High */

#define GPIO15_PIN		2 /* GPO Open Drain */
#define GPIO15_TYPE		0 /* VDD_IO1 */
#define GPIO15_MODE		1 /* Output High */

#define MAXIMUM_PWM		95
#define MASK_GPIO14		0x0F
#define MASK_GPIO15		0xF0
#define GPIO15_PIN_BIT_POSITION	4

static void da9052_led_work(struct work_struct *work)
{
	struct da9052_led_data *led = container_of(work,
				 struct da9052_led_data, work);
	int reg = 0;
	int led_dim_bit = 0;
	struct da9052_ssc_msg msg;
	int ret = 0;

	switch (led->id) {
	case DA9052_LED_4:
		reg = DA9052_LED4CONT_REG;
		led_dim_bit = DA9052_LED4CONT_LED4DIM;
	break;
	case DA9052_LED_5:
		reg = DA9052_LED5CONT_REG;
		led_dim_bit = DA9052_LED5CONT_LED5DIM;
	break;
	}

	if (led->new_brightness > MAXIMUM_PWM)
		led->new_brightness = MAXIMUM_PWM;

	/* Always enable DIM feature
	 * This feature can be disabled if required
	 */
	msg.addr = reg;
	msg.data = led->new_brightness | led_dim_bit;
	da9052_lock(led->da9052);
	ret = led->da9052->write(led->da9052, &msg);
	if (ret) {
		da9052_unlock(led->da9052);
		return;
	}
	da9052_unlock(led->da9052);
}

static void da9052_led_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	struct da9052_led_data *led;

	led = container_of(led_cdev, struct da9052_led_data, cdev);
	led->new_brightness = value;
	schedule_work(&led->work);
}

static int __devinit da9052_led_setup(struct da9052_led_data *led)
{
	int reg = 0;
	int ret = 0;

	struct da9052_ssc_msg msg;

	switch (led->id) {
	case DA9052_LED_4:
		reg = DA9052_LED4CONT_REG;
	break;
	case DA9052_LED_5:
		reg = DA9052_LED5CONT_REG;
	break;
	}

	msg.addr = reg;
	msg.data = 0;

	da9052_lock(led->da9052);
	ret = led->da9052->write(led->da9052, &msg);
	if (ret) {
		da9052_unlock(led->da9052);
		return ret;
	}
	da9052_unlock(led->da9052);
	return ret;
}

static int da9052_leds_prepare(struct da9052_led_data *led)
{
	int ret = 0;
	struct da9052_ssc_msg msg;

	da9052_lock(led->da9052);

	if (1 == led->is_led4_present) {
		msg.addr = DA9052_GPIO1415_REG;
		msg.data = 0;

		ret = led->da9052->read(led->da9052, &msg);
		if (ret)
			goto out;
		msg.data = msg.data & ~(MASK_GPIO14);
		msg.data = msg.data | (
				GPIO14_PIN |
				(GPIO14_TYPE ? DA9052_GPIO1415_GPIO14TYPE : 0) |
				(GPIO14_MODE ? DA9052_GPIO1415_GPIO14MODE : 0));

		ret = led->da9052->write(led->da9052, &msg);
		if (ret)
			goto out;
	}

	if (1 == led->is_led5_present) {
		msg.addr = DA9052_GPIO1415_REG;
		msg.data = 0;

		ret = led->da9052->read(led->da9052, &msg);
		if (ret)
			goto out;
		msg.data = msg.data & ~(MASK_GPIO15);
		msg.data = msg.data |
			(((GPIO15_PIN << GPIO15_PIN_BIT_POSITION) |
				(GPIO15_TYPE ? DA9052_GPIO1415_GPIO15TYPE : 0) |
				(GPIO15_MODE ? DA9052_GPIO1415_GPIO15MODE : 0))
				);
		ret = led->da9052->write(led->da9052, &msg);
		if (ret)
			goto out;
	}

	da9052_unlock(led->da9052);
	return ret;
out:
	da9052_unlock(led->da9052);
	return ret;
}

static int __devinit da9052_led_probe(struct platform_device *pdev)
{
	struct da9052_leds_platform_data *pdata = (pdev->dev.platform_data);
	struct da9052_led_platform_data *led_cur;
	struct da9052_led_data *led, *led_dat;
	int ret, i;
	int init_led = 0;

	if (pdata->num_leds < 1 || pdata->num_leds > DA9052_LED_MAX) {
		dev_err(&pdev->dev, "Invalid led count %d\n", pdata->num_leds);
		return -EINVAL;
	}

	led = kzalloc(sizeof(*led) * pdata->num_leds, GFP_KERNEL);
	if (led == NULL) {
		dev_err(&pdev->dev, "failed to alloc memory\n");
		return -ENOMEM;
	}

	led->is_led4_present = DA9052_LED4_PRESENT;
	led->is_led5_present = DA9052_LED5_PRESENT;

	for (i = 0; i < pdata->num_leds; i++) {
		led_dat = &led[i];
		led_cur = &pdata->led[i];
		if (led_cur->id < 0) {
			dev_err(&pdev->dev, "invalid id %d\n", led_cur->id);
			ret = -EINVAL;
			goto err_register;
		}

		if (init_led & (1 << led_cur->id)) {
			dev_err(&pdev->dev, "led %d already initialized\n",
					led_cur->id);
			ret = -EINVAL;
			goto err_register;
		}

		init_led |= 1 << led_cur->id;
		
		led_dat->cdev.name = led_cur->name;
		// led_dat->cdev.default_trigger = led_cur[i]->default_trigger;
		led_dat->cdev.brightness_set = da9052_led_set;
		led_dat->cdev.brightness = LED_OFF;
		led_dat->id = led_cur->id;
		led_dat->da9052 = dev_get_drvdata(pdev->dev.parent);

		INIT_WORK(&led_dat->work, da9052_led_work);

		ret = led_classdev_register(pdev->dev.parent, &led_dat->cdev);
		if (ret) {
			dev_err(&pdev->dev, "failed to register led %d\n",
					led_dat->id);
			goto err_register;

		}
		ret = da9052_led_setup(led_dat);
		if (ret) {
			dev_err(&pdev->dev, "unable to init led %d\n",
					led_dat->id);
			i++;
			goto err_register;
		}
	}
	ret = da9052_leds_prepare(led);
	if (ret) {
		dev_err(&pdev->dev, "unable to init led driver\n");
		goto err_free;
	}

	platform_set_drvdata(pdev, led);
	return 0;

err_register:
	for (i = i - 1; i >= 0; i--) {
		led_classdev_unregister(&led[i].cdev);
		cancel_work_sync(&led[i].work);
	}

err_free:
	kfree(led);
	return ret;
}

static int __devexit da9052_led_remove(struct platform_device *pdev)
{
	struct da9052_leds_platform_data *pdata =
		(struct da9052_leds_platform_data *)pdev->dev.platform_data;
	struct da9052_led_data *led = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < pdata->num_leds; i++) {
		da9052_led_setup(&led[i]);
		led_classdev_unregister(&led[i].cdev);
		cancel_work_sync(&led[i].work);
	}

	kfree(led);
	return 0;
}

static struct platform_driver da9052_led_driver = {
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= da9052_led_probe,
	.remove		= __devexit_p(da9052_led_remove),
};

static int __init da9052_led_init(void)
{
	return platform_driver_register(&da9052_led_driver);
}
module_init(da9052_led_init);

static void __exit da9052_led_exit(void)
{
	platform_driver_unregister(&da9052_led_driver);
}
module_exit(da9052_led_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <dchen@diasemi.com>	");
MODULE_DESCRIPTION("LED driver for Dialog DA9052 PMIC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
