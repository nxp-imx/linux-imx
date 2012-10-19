/*
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * da9052_onkey.c: Onkey driver for DA9052
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>

#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/reg.h>

#define DRIVER_NAME "da9052-onkey"

struct da9052_onkey_data {
	struct da9052 *da9052;
	struct da9052_eh_nb eh_data;
	struct input_dev *input;
};

/* Flag to enable key events during suspend */
static bool enable_onkey_events;

static void da9052_onkey_report_event(struct da9052_eh_nb *eh_data,
				unsigned int event)
{
	struct da9052_onkey_data *da9052_onkey =
		container_of(eh_data, struct da9052_onkey_data, eh_data);
	struct da9052_ssc_msg msg;
	unsigned int ret;

	/* Read the Evnet Register */
	msg.addr = DA9052_EVENTB_REG;
	da9052_lock(da9052_onkey->da9052);
	ret = da9052_onkey->da9052->read(da9052_onkey->da9052, &msg);
	if (ret) {
		da9052_unlock(da9052_onkey->da9052);
		return;
	}
	da9052_unlock(da9052_onkey->da9052);
	msg.data = msg.data & DA9052_EVENTB_ENONKEY;

	/* We need onkey events only in suspend mode */
	if (enable_onkey_events) {
		input_report_key(da9052_onkey->input, KEY_POWER, msg.data);
		input_sync(da9052_onkey->input);
	}
	pr_debug("DA9052 ONKEY EVENT REPORTED\n");
}

static int __devinit da9052_onkey_probe(struct platform_device *pdev)
{
	struct da9052_onkey_data *da9052_onkey;
	int error;

	da9052_onkey = kzalloc(sizeof(*da9052_onkey), GFP_KERNEL);
	da9052_onkey->input = input_allocate_device();
	if (!da9052_onkey->input) {
		dev_err(&pdev->dev, "failed to allocate data device\n");
		error = -ENOMEM;
		goto fail1;
	}
	da9052_onkey->da9052 = dev_get_drvdata(pdev->dev.parent);

	if (!da9052_onkey->input) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		error = -ENOMEM;
		goto fail2;
	}

	da9052_onkey->input->evbit[0] = BIT_MASK(EV_KEY);
	da9052_onkey->input->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	da9052_onkey->input->name = "da9052-onkey";
	da9052_onkey->input->phys = "da9052-onkey/input0";
	da9052_onkey->input->dev.parent = &pdev->dev;

	/* Set the EH structure */
	da9052_onkey->eh_data.eve_type = ONKEY_EVE;
	da9052_onkey->eh_data.call_back = &da9052_onkey_report_event;
	error = da9052_onkey->da9052->register_event_notifier(
				da9052_onkey->da9052,
				&da9052_onkey->eh_data);
	if (error)
		goto fail2;

	error = input_register_device(da9052_onkey->input);
	if (error) {
		dev_err(&pdev->dev, "Unable to register input\
				device,error: %d\n", error);
		goto fail3;
	}

	platform_set_drvdata(pdev, da9052_onkey);

	return 0;

fail3:
	da9052_onkey->da9052->unregister_event_notifier(da9052_onkey->da9052,
					&da9052_onkey->eh_data);
fail2:
	input_free_device(da9052_onkey->input);
fail1:
	kfree(da9052_onkey);
	return error;
}

static int __devexit da9052_onkey_remove(struct platform_device *pdev)
{
	struct da9052_onkey_data *da9052_onkey = pdev->dev.platform_data;
	da9052_onkey->da9052->unregister_event_notifier(da9052_onkey->da9052,
					&da9052_onkey->eh_data);
	input_unregister_device(da9052_onkey->input);
	kfree(da9052_onkey);

	return 0;
}

#ifdef CONFIG_PM
static int da9052_onkey_suspend(struct device *dev)
{
	enable_onkey_events = true;
	return 0;
}

static int da9052_onkey_resume(struct device *dev)
{
	enable_onkey_events = false;
	return 0;
}

static const struct dev_pm_ops da9052_onkey_pm_ops = {
	.suspend	= da9052_onkey_suspend,
	.resume		= da9052_onkey_resume,
};
#else
static const struct dev_pm_ops da9052_onkey_pm_ops = {};
#endif

static struct platform_driver da9052_onkey_driver = {
	.probe		= da9052_onkey_probe,
	.remove		= __devexit_p(da9052_onkey_remove),
	.driver		= {
		.name	= "da9052-onkey",
		.owner	= THIS_MODULE,
		.pm	= &da9052_onkey_pm_ops,
	}
};

static int __init da9052_onkey_init(void)
{
	return platform_driver_register(&da9052_onkey_driver);
}

static void __exit da9052_onkey_exit(void)
{
	platform_driver_unregister(&da9052_onkey_driver);
}

module_init(da9052_onkey_init);
module_exit(da9052_onkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Dajun Chen <dchen@diasemi.com>");
MODULE_DESCRIPTION("Onkey driver for DA9052");
MODULE_ALIAS("platform:" DRIVER_NAME);
