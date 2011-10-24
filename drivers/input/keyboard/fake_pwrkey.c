/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
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
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/kd.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>

static struct input_dev *input;
static struct wake_lock wakelock;
static struct hrtimer timer;

static enum hrtimer_restart fake_timer_func(struct hrtimer *timer)
{
	input_event(input, EV_KEY, KEY_MENU, 0);
	return HRTIMER_NORESTART;
}

static int fake_pwrkey_probe(struct platform_device *pdev)
{
	int retval;

	input = input_allocate_device();
	if (!input) {
		dev_err(&pdev->dev, "no memory for input device\n");
		retval = -ENOMEM;
		goto err1;
	}

	input->name = "fake_power_key";
	input->phys = "fakepwrkey/input0";
	input->id.bustype = BUS_HOST;
	input->evbit[0] = BIT_MASK(EV_KEY);

	input_set_capability(input, EV_KEY, KEY_MENU);

	retval = input_register_device(input);
	if (retval < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err2;
	}

	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer.function = fake_timer_func;
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND,
			"pwrkey_lock");

	printk(KERN_INFO "PMIC powerkey probe\n");

	return 0;

err2:
	input_free_device(input);
err1:
	return retval;
}

static int fake_pwrkey_remove(struct platform_device *pdev)
{
	input_unregister_device(input);
	input_free_device(input);

	return 0;
}

static int fake_pwrkey_suspend(struct device *dev)
{
	return 0;
}

static int fake_pwrkey_resume(struct device *dev)
{
	input_event(input, EV_KEY, KEY_MENU, 1);
	hrtimer_start(&timer, ktime_set(0, 500000000), HRTIMER_MODE_REL);
	return 0;
}
static SIMPLE_DEV_PM_OPS(fake_pwrkey_pm_ops, fake_pwrkey_suspend, fake_pwrkey_resume);

static struct platform_driver fake_pwrkey_driver = {
	.driver = {
		.name = "fake_pwrkey",
		.pm	= &fake_pwrkey_pm_ops,
	},
	.probe = fake_pwrkey_probe,
	.remove = fake_pwrkey_remove,
};

static int __init fake_pwrkey_init(void)
{
	return platform_driver_register(&fake_pwrkey_driver);
}

static void __exit fake_pwrkey_exit(void)
{
	platform_driver_unregister(&fake_pwrkey_driver);
}

module_init(fake_pwrkey_init);
module_exit(fake_pwrkey_exit);


MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("MXC fake power key Driver");
MODULE_LICENSE("GPL");
