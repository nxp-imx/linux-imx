/*
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_ts.c
 *
 * @brief Driver for the Freescale Semiconductor MXC touchscreen.
 *
 * The touchscreen driver is designed as a standard input driver which is a
 * wrapper over low level PMIC driver. Most of the hardware configuration and
 * touchscreen functionality is implemented in the low level PMIC driver. During
 * initialization, this driver creates a kernel thread. This thread then calls
 * PMIC driver to obtain touchscreen values continously. These values are then
 * passed to the input susbsystem.
 *
 * @ingroup touchscreen
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_adc.h>

#define MXC_TS_NAME	"mxc_ts"

static struct input_dev *mxc_inputdev = NULL;
static u32 input_ts_installed;

static int ts_thread(void *arg)
{
	t_touch_screen ts_sample;
	s32 wait = 0;

	daemonize("mxc_ts");
	while (input_ts_installed) {
		try_to_freeze();
		memset(&ts_sample, 0, sizeof(t_touch_screen));
		if (0 != pmic_adc_get_touch_sample(&ts_sample, !wait))
			continue;
		if (!(ts_sample.contact_resistance || wait))
			continue;

		input_report_abs(mxc_inputdev, ABS_X, ts_sample.x_position);
		input_report_abs(mxc_inputdev, ABS_Y, ts_sample.y_position);
		input_report_abs(mxc_inputdev, ABS_PRESSURE,
				 ts_sample.contact_resistance);
		input_sync(mxc_inputdev);

		wait = ts_sample.contact_resistance;
		msleep(20);
	}

	return 0;
}

static int __init mxc_ts_init(void)
{
	int retval;

	if (!is_pmic_adc_ready())
		return -ENODEV;

	mxc_inputdev = input_allocate_device();
	if (!mxc_inputdev) {
		printk(KERN_ERR
		       "mxc_ts_init: not enough memory for input device\n");
		return -ENOMEM;
	}

	mxc_inputdev->name = MXC_TS_NAME;
	mxc_inputdev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	mxc_inputdev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
	mxc_inputdev->absbit[0] =
	    BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) | BIT_MASK(ABS_PRESSURE);
	retval = input_register_device(mxc_inputdev);
	if (retval < 0) {
		input_free_device(mxc_inputdev);
		return retval;
	}

	input_ts_installed = 1;
	kernel_thread(ts_thread, NULL, CLONE_VM | CLONE_FS);
	printk("mxc input touchscreen loaded\n");
	return 0;
}

static void __exit mxc_ts_exit(void)
{
	input_ts_installed = 0;
	input_unregister_device(mxc_inputdev);

	if (mxc_inputdev) {
		input_free_device(mxc_inputdev);
		mxc_inputdev = NULL;
	}
}

late_initcall(mxc_ts_init);
module_exit(mxc_ts_exit);

MODULE_DESCRIPTION("MXC input touchscreen driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
