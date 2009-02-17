/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file tsc2007.c
 *
 * @brief Driver for TI's tsc2007 I2C Touch Screen Controller.
 *
 * This driver is based on the driver written by Bill Gatliff
 *  Copyright (C) 2005 Bill Gatliff <bgat at billgatliff.com>
 *  Changes for 2.6.20 kernel by Nicholas Chen <nchen at cs.umd.edu>
 *
 * @ingroup touchscreen
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/bcd.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <asm/mach/irq.h>

#define DRIVER_NAME "tsc2007"

enum tsc2007_pd {
	PD_POWERDOWN = 0,	/* penirq */
	PD_IREFOFF_ADCON = 1,	/* no penirq */
	PD_IREFON_ADCOFF = 2,	/* penirq */
	PD_IREFON_ADCON = 3,	/* no penirq */
	PD_PENIRQ_ARM = PD_IREFON_ADCOFF,
	PD_PENIRQ_DISARM = PD_IREFON_ADCON,
};

enum tsc2007_m {
	M_12BIT = 0,
	M_8BIT = 1
};

enum tsc2007_cmd {
	MEAS_TEMP0 = 0,
	MEAS_IN1 = 2,
	MEAS_XPOS = 12,
	MEAS_YPOS = 13,
	MEAS_Z1POS = 14,
	MEAS_Z2POS = 15
};

#define tsc2007_CMD(cn, pdn, m) (((cn) << 4) | ((pdn) << 2) | ((m) << 1))

#define ADC_MAX ((1 << 12) - 1)

struct tsc2007_data {
	struct i2c_client *client;
	struct input_dev *idev;
	struct timer_list penirq_timer;
	struct task_struct *tstask;
	u32 ts_thread_cnt;
	struct completion penirq_completion;
	struct completion penup_completion;
	enum tsc2007_m m;
	int penirq;
	int penup_threshold;
	struct regulator *vdd_reg;
	int opened;
};

static int tsc2007_read(struct tsc2007_data *data,
			enum tsc2007_cmd cmd, enum tsc2007_pd pd, int *val)
{
	unsigned char c;
	unsigned char d[2];
	int ret;

	c = tsc2007_CMD(cmd, pd, data->m);

	ret = i2c_master_send(data->client, &c, 1);

	if (ret < 0)
		goto err;

	udelay(20);
	ret = i2c_master_recv(data->client, d, data->m == M_12BIT ? 2 : 1);
	if (ret < 0)
		goto err;

	if (val) {
		*val = d[0];
		*val <<= 4;
		if (data->m == M_12BIT)
			*val += (d[1] >> 4);
	}

	return 0;
      err:
	return -ENODEV;
}

static inline int tsc2007_read_xpos(struct tsc2007_data *d, enum
				    tsc2007_pd pd, int *x)
{
	return tsc2007_read(d, MEAS_XPOS, pd, x);
}

static inline int tsc2007_read_ypos(struct tsc2007_data *d, enum
				    tsc2007_pd pd, int *y)
{
	return tsc2007_read(d, MEAS_YPOS, pd, y);
}

static inline int tsc2007_read_pressure(struct tsc2007_data *d, enum
					tsc2007_pd pd, int *p)
{
	return tsc2007_read(d, MEAS_Z1POS, pd, p);
}

static inline int tsc2007_powerdown(struct tsc2007_data *d)
{
	/* we don't have a distinct powerdown command,
	   so do a benign read with the PD bits cleared */
	return tsc2007_read(d, MEAS_IN1, PD_POWERDOWN, 0);
}

#define PENUP_TIMEOUT		10

static irqreturn_t tsc2007_penirq(int irq, void *v)
{
	struct tsc2007_data *d = v;

	disable_irq(d->penirq);
	complete(&d->penirq_completion);
	return IRQ_HANDLED;
}

static void tsc2007_pen_up(unsigned long v)
{
	struct tsc2007_data *d = (struct tsc2007_data *)v;

	complete(&d->penup_completion);
	return;
}

static inline void tsc2007_restart_pen_up_timer(struct tsc2007_data *d)
{
	mod_timer(&d->penirq_timer, jiffies + (PENUP_TIMEOUT * HZ) / 1000);
}

static int tsc2007ts_thread(void *v)
{
	struct tsc2007_data *d = v;

	if (d->ts_thread_cnt)
		return -EINVAL;
	d->ts_thread_cnt = 1;

	while (1) {
		unsigned int x = 0, y = 0, p = 0;

		if (kthread_should_stop())
			break;
		/* Wait for an Pen down interrupt */
		if (wait_for_completion_interruptible_timeout
		    (&d->penirq_completion, HZ) <= 0)
			continue;

		tsc2007_read_xpos(d, PD_PENIRQ_DISARM, &x);
		tsc2007_read_ypos(d, PD_PENIRQ_DISARM, &y);
		tsc2007_read_pressure(d, PD_PENIRQ_DISARM, &p);
		input_report_abs(d->idev, ABS_X, 4096 - x);
		input_report_abs(d->idev, ABS_Y, 4096 - y);
		input_report_abs(d->idev, ABS_PRESSURE, p);
		input_sync(d->idev);

		while (p > d->penup_threshold) {
			tsc2007_restart_pen_up_timer(d);
			wait_for_completion_interruptible(&d->penup_completion);
			/* Pen Down */
			tsc2007_read_xpos(d, PD_PENIRQ_DISARM, &x);
			tsc2007_read_ypos(d, PD_PENIRQ_DISARM, &y);
			tsc2007_read_pressure(d, PD_PENIRQ_DISARM, &p);
			if (p <= d->penup_threshold)
				break;

			input_report_abs(d->idev, ABS_X, 4096 - x);
			input_report_abs(d->idev, ABS_Y, 4096 - y);
			input_report_abs(d->idev, ABS_PRESSURE, p);
			input_sync(d->idev);
		};

		/* Pen Up */
		input_report_abs(d->idev, ABS_X, 4096 - x);
		input_report_abs(d->idev, ABS_Y, 4096 - y);
		input_report_abs(d->idev, ABS_PRESSURE, 0);
		input_sync(d->idev);

		tsc2007_read(d, MEAS_TEMP0, PD_PENIRQ_ARM, 0);
		enable_irq(d->penirq);

	}

	d->ts_thread_cnt = 0;
	return 0;
}

/*!
 * This function puts the touch screen controller in low-power mode/state.
 *
 * @param   pdev  the device structure used to give information on touch screen
 *                to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int tsc2007_suspend(struct i2c_client *client, pm_message_t state)
{
	struct tsc2007_data *d = i2c_get_clientdata(client);

	if (!IS_ERR(d->tstask) && d->opened)
		kthread_stop(d->tstask);

	return 0;
}

/*!
 * This function brings the touch screen controller back from low-power state.
 *
 * @param   pdev  the device structure used to give information on touch screen
 *                to resume
 *
 * @return  The function always returns 0.
 */
static int tsc2007_resume(struct i2c_client *client)
{
	struct tsc2007_data *d = i2c_get_clientdata(client);

	if (d->opened)
		d->tstask = kthread_run(tsc2007ts_thread, d, DRIVER_NAME "tsd");

	return 0;
}

static int tsc2007_idev_open(struct input_dev *idev)
{
	struct tsc2007_data *d = input_get_drvdata(idev);
	int ret = 0;

	d->penirq_timer.data = (unsigned long)d;
	d->penirq_timer.function = tsc2007_pen_up;

	init_completion(&d->penup_completion);

	d->tstask = kthread_run(tsc2007ts_thread, d, DRIVER_NAME "tsd");
	if (IS_ERR(d->tstask))
		ret = PTR_ERR(d->tstask);
	else
		d->opened++;

	return ret;
}

static void tsc2007_idev_close(struct input_dev *idev)
{
	struct tsc2007_data *d = input_get_drvdata(idev);
	if (!IS_ERR(d->tstask))
		kthread_stop(d->tstask);

	del_timer_sync(&d->penirq_timer);

	if (d->opened > 0)
		d->opened--;
}

static int tsc2007_driver_register(struct tsc2007_data *data)
{
	struct input_dev *idev;
	int ret = 0;

	init_timer(&data->penirq_timer);
	data->penirq_timer.data = (unsigned long)data;
	data->penirq_timer.function = tsc2007_pen_up;

	init_completion(&data->penirq_completion);

	if (data->penirq) {
		ret =
		    request_irq(data->penirq, tsc2007_penirq, IRQF_TRIGGER_LOW,
				DRIVER_NAME, data);
		if (!ret) {
			printk(KERN_INFO "%s: Registering Touchscreen device\n",
			       __func__);
			set_irq_wake(data->penirq, 1);
		} else {
			printk(KERN_ERR "%s: Cannot grab irq %d\n",
			       __func__, data->penirq);
		}
	}
	idev = input_allocate_device();
	data->idev = idev;
	input_set_drvdata(idev, data);
	idev->name = DRIVER_NAME;
	idev->evbit[0] = BIT(EV_ABS);
	idev->open = tsc2007_idev_open;
	idev->close = tsc2007_idev_close;
	idev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	input_set_abs_params(idev, ABS_X, 0, ADC_MAX, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, ADC_MAX, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE, 0, 0, 0, 0);

	if (!ret)
		ret = input_register_device(idev);

	return ret;
}

static int tsc2007_i2c_remove(struct i2c_client *client)
{
	int err;
	struct tsc2007_data *d = i2c_get_clientdata(client);
	struct mxc_tsc_platform_data *tsc_data;

	free_irq(d->penirq, d);
	input_unregister_device(d->idev);

	err = i2c_detach_client(client);
	if (err) {
		dev_err(&client->dev, "Client deregistration failed, "
			"client not detached.\n");
		return err;
	}

	tsc_data = (struct mxc_tsc_platform_data *)(client->dev).platform_data;
	if (tsc_data && tsc_data->inactive)
		tsc_data->inactive();

	if (d->vdd_reg) {
		regulator_disable(d->vdd_reg);
		regulator_put(d->vdd_reg);
		d->vdd_reg = NULL;
	}
	return 0;
}

static int tsc2007_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tsc2007_data *data;
	struct mxc_tsc_platform_data *tsc_data;
	int err = 0;

	data = kzalloc(sizeof(struct tsc2007_data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	data->client = client;
	data->penirq = client->irq;

	tsc_data = (struct mxc_tsc_platform_data *)(client->dev).platform_data;
	if (tsc_data && tsc_data->vdd_reg) {
		if (tsc_data->penup_threshold > (ADC_MAX >> 3))
			data->penup_threshold = (ADC_MAX >> 3);
		else if (tsc_data->penup_threshold > 0)
			data->penup_threshold = tsc_data->penup_threshold;
		else
			data->penup_threshold = 10;

		data->vdd_reg = regulator_get(&client->dev, tsc_data->vdd_reg);
		if (!IS_ERR(data->vdd_reg))
			regulator_enable(data->vdd_reg);
		else
			data->vdd_reg = NULL;
		if (tsc_data->active)
			tsc_data->active();
	} else {
		data->vdd_reg = NULL;
		data->penup_threshold = 10;
	}

	err = tsc2007_powerdown(data);
	if (err >= 0) {
		data->m = M_12BIT;

		err = tsc2007_driver_register(data);
		if (err < 0)
			goto exit;

		return 0;
	}

      exit:
	return err;
}

static const struct i2c_device_id tsc2007_id[] = {
	{ "tsc2007", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, tsc2007_id);

static struct i2c_driver tsc2007_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   },
	.probe = tsc2007_i2c_probe,
	.remove = tsc2007_i2c_remove,
	.suspend = tsc2007_suspend,
	.resume = tsc2007_resume,
	.command = NULL,
	.id_table = tsc2007_id,
};

static int __init tsc2007_init(void)
{
	return i2c_add_driver(&tsc2007_driver);
}

static void __exit tsc2007_exit(void)
{
	i2c_del_driver(&tsc2007_driver);
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("tsc2007 Touch Screen Controller driver");
MODULE_LICENSE("GPL");

module_init(tsc2007_init);
module_exit(tsc2007_exit);
