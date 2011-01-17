/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/powerkey.h>

struct mxc_pwrkey_priv {

	struct input_dev *input;
	int value;
	int (*get_status) (int);
};

static struct mxc_pwrkey_priv *mxc_pwrkey;

static void pwrkey_event_handler(void *param)
{
	int pressed;

	pressed = mxc_pwrkey->get_status((int)param);

	if (pressed) {
		input_report_key(
			mxc_pwrkey->input, mxc_pwrkey->value, 1);
		pr_info("%s_Keydown\n", __func__);
	} else {
		input_report_key(
			mxc_pwrkey->input, mxc_pwrkey->value, 0);
		pr_info("%s_Keyup\n", __func__);
	}
}

static int mxcpwrkey_probe(struct platform_device *pdev)
{
	int retval;
	struct input_dev *input;
	struct power_key_platform_data *pdata = pdev->dev.platform_data;

	if (mxc_pwrkey) {
		dev_warn(&pdev->dev, "two power key??\n");
		return -EBUSY;
	}

	if (!pdata || !pdata->get_key_status) {
		dev_err(&pdev->dev, "can not get platform data\n");
		return -EINVAL;
	}

	mxc_pwrkey = kmalloc(sizeof(struct mxc_pwrkey_priv), GFP_KERNEL);
	if (!mxc_pwrkey) {
		dev_err(&pdev->dev, "can not allocate private data");
		return -ENOMEM;
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(&pdev->dev, "no memory for input device\n");
		retval = -ENOMEM;
		goto err1;
	}

	input->name = "mxc_power_key";
	input->phys = "mxcpwrkey/input0";
	input->id.bustype = BUS_HOST;
	input->evbit[0] = BIT_MASK(EV_KEY);

	mxc_pwrkey->value = pdata->key_value;
	mxc_pwrkey->get_status = pdata->get_key_status;
	mxc_pwrkey->input = input;
	pdata->register_pwrkey(pwrkey_event_handler);

	input_set_capability(input, EV_KEY, mxc_pwrkey->value);

	retval = input_register_device(input);
	if (retval < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err2;
	}

	printk(KERN_INFO "PMIC powerkey probe\n");

	return 0;

err2:
	input_free_device(input);
err1:
	kfree(mxc_pwrkey);
	mxc_pwrkey = NULL;

	return retval;
}

static int mxcpwrkey_remove(struct platform_device *pdev)
{
	input_unregister_device(mxc_pwrkey->input);
	input_free_device(mxc_pwrkey->input);
	kfree(mxc_pwrkey);
	mxc_pwrkey = NULL;

	return 0;
}

static struct platform_driver mxcpwrkey_driver = {
	.driver = {
		.name = "mxcpwrkey",
	},
	.probe = mxcpwrkey_probe,
	.remove = mxcpwrkey_remove,
};

static int __init mxcpwrkey_init(void)
{
	return platform_driver_register(&mxcpwrkey_driver);
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
