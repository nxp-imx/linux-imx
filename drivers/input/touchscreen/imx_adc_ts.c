/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file imx_adc_ts.c
 *
 * @brief Driver for the Freescale Semiconductor i.MX ADC touchscreen.
 *
 * This touchscreen driver is designed as a standard input driver.  It is a
 * wrapper around the low level ADC driver. Much of the hardware configuration
 * and touchscreen functionality is implemented in the low level ADC driver.
 * During initialization, this driver creates a kernel thread.  This thread
 * then calls the ADC driver to obtain touchscreen values continously. These
 * values are then passed to the input susbsystem.
 *
 * @ingroup touchscreen
 */

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/imx_adc.h>

#define IMX_ADC_TS_NAME	"imx_adc_ts"

static struct input_dev *imx_inputdev;
static u32 input_ts_installed;

static int ts_thread(void *arg)
{
	struct t_touch_screen ts_sample;
	int wait = 0;
	daemonize("imx_adc_ts");
	while (input_ts_installed) {
		try_to_freeze();

		memset(&ts_sample, 0, sizeof(ts_sample));
		if (0 != imx_adc_get_touch_sample(&ts_sample, !wait))
			continue;

		input_report_abs(imx_inputdev, ABS_X, ts_sample.x_position);
		input_report_abs(imx_inputdev, ABS_Y, ts_sample.y_position);
		input_report_abs(imx_inputdev, ABS_PRESSURE,
				 ts_sample.contact_resistance);
		input_sync(imx_inputdev);
		wait = ts_sample.contact_resistance;
		msleep(10);
	}

	return 0;
}

static int __init imx_adc_ts_init(void)
{
	int retval;

	if (!is_imx_adc_ready())
		return -ENODEV;

	imx_inputdev = input_allocate_device();
	if (!imx_inputdev) {
		pr_err("imx_ts_init: not enough memory for input device\n");
		return -ENOMEM;
	}

	imx_inputdev->name = IMX_ADC_TS_NAME;
	imx_inputdev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	imx_inputdev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
	imx_inputdev->absbit[0] =
	    BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) | BIT_MASK(ABS_PRESSURE);
	retval = input_register_device(imx_inputdev);
	if (retval < 0) {
		input_free_device(imx_inputdev);
		return retval;
	}

	input_ts_installed = 1;
	kthread_run(ts_thread, NULL, "ts_thread");
	pr_info("i.MX ADC input touchscreen loaded.\n");
	return 0;
}

static void __exit imx_adc_ts_exit(void)
{
	input_ts_installed = 0;
	input_unregister_device(imx_inputdev);

	if (imx_inputdev) {
		input_free_device(imx_inputdev);
		imx_inputdev = NULL;
	}
}

late_initcall(imx_adc_ts_init);
module_exit(imx_adc_ts_exit);

MODULE_DESCRIPTION("i.MX ADC input touchscreen driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
