/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/powerkey.h>

#define KEYCODE_ONOFF            KEY_F4

/*! Input device structure. */
static struct input_dev *mxcpwrkey_dev;
static bool key_press;
static void power_key_event_handler(void *param)
{
	if (!key_press) {
		key_press = true;
		input_report_key(mxcpwrkey_dev, KEYCODE_ONOFF, 1);
		pr_info("%s_KeyUP\n", __func__);
	} else {
		if (key_press) {
			key_press = false;
			input_report_key(mxcpwrkey_dev, KEYCODE_ONOFF, 0);
			pr_info("%s_KeyDown\n", __func__);
		}
	}
}
static int mxcpwrkey_probe(struct platform_device *pdev)
{
	int retval, err;
	struct power_key_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		printk(KERN_DEBUG "power key platform data is NULL, there is no power key\n");
		return 0;
	}
	printk(KERN_INFO "PMIC powerkey probe\n");

	key_press = false;
	mxcpwrkey_dev = input_allocate_device();
	if (!mxcpwrkey_dev) {
		printk(KERN_ERR "mxcpwrkey_dev: not enough memory for input device\n");
		retval = -ENOMEM;
		goto err1;
	}

	mxcpwrkey_dev->name = "mxc_power_key";
	mxcpwrkey_dev->phys = "mxcpwrkey/input0";
	mxcpwrkey_dev->id.bustype = BUS_HOST;
	mxcpwrkey_dev->evbit[0] = BIT_MASK(EV_KEY);

	pdata->register_key_press_handler(power_key_event_handler, NULL);

	input_set_capability(mxcpwrkey_dev, EV_KEY, KEYCODE_ONOFF);

	retval = input_register_device(mxcpwrkey_dev);
	if (retval < 0) {
		printk(KERN_ERR "mxcpwrkey_dev: failed to register input device\n");
		goto err2;
	}
	return 0;
err2:
	input_free_device(mxcpwrkey_dev);
err1:
	return retval;
}
static int mxcpwrkey_remove(struct platform_device *pdev)
{
	return 0;
}
static int mxc_pwrkey_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}
static int mxc_pwrkey_resume(struct platform_device *pdev)
{
	return 0;
}
static struct platform_driver mxcpwrkey_driver = {
	.driver = {
		.name = "mxcpwrkey",
	},
	.probe = mxcpwrkey_probe,
	.remove = mxcpwrkey_remove,
	.suspend = mxc_pwrkey_suspend,
	.resume = mxc_pwrkey_resume,
};
static int __init mxcpwrkey_init(void)
{
	platform_driver_register(&mxcpwrkey_driver);
	return 0;
}

static void __exit mxcpwrkey_exit(void)
{
	platform_driver_unregister(&mxcpwrkey_driver);
}
module_init(mxcpwrkey_init);
module_exit(mxcpwrkey_exit);


MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("MXC board power key Driver");
MODULE_LICENSE("GPL");
