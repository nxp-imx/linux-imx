/*
 * Driver for EETI eGalax Multiple Touch Controller
 *
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * based on max11801_ts.c
 *
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

/* EETI eGalax serial touch screen controller is a I2C based multiple
 * touch screen controller, it supports 5 point multiple touch. */

/* TODO:
  - auto idle mode support
  - early suspend support for android
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/input/mt.h>
#include <linux/earlysuspend.h>

/* Mouse Mode: some panel may configure the controller to mouse mode,
 * which can only report one point at a given time.
 * This driver will ignore events in this mode.
 */
#define REPORT_MODE_MOUSE		0x1
/* Vendor Mode: this mode is used to transfer some vendor specific
 * messages.
 * This driver will ignore events in this mode.
 */
#define REPORT_MODE_VENDOR		0x3
/* Multiple Touch Mode */
#define REPORT_MODE_MTTOUCH		0x4

#define MAX_SUPPORT_POINTS		2

#define EVENT_VALID_OFFSET	7
#define EVENT_VALID_MASK	(0x1 << EVENT_VALID_OFFSET)
#define EVENT_ID_OFFSET		2
#define EVENT_ID_MASK		(0xf << EVENT_ID_OFFSET)
#define EVENT_IN_RANGE		(0x1 << 1)
#define EVENT_DOWN_UP		(0X1 << 0)

#define MAX_I2C_DATA_LEN	10

#define EGALAX_MAX_X	32760
#define EGALAX_MAX_Y	32760
#define EGALAX_MAX_Z	2048
#define EGALAX_MAX_TRIES 100

struct finger_info {
	s16	x;
	s16	y;
	s16	z;
};

struct egalax_ts {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
#ifdef CONFIG_EARLYSUSPEND
	struct early_suspend		es_handler;
#endif
	u32				finger_mask;
	int				touch_no_wake;
	struct finger_info		fingers[MAX_SUPPORT_POINTS];
};

static void report_input_data(struct egalax_ts *data)
{
	int i;
	int num_fingers_down;

	num_fingers_down = 0;
	for (i = 0; i < MAX_SUPPORT_POINTS; i++) {
		if (data->fingers[i].z == -1)
			continue;

		input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					data->fingers[i].x);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					data->fingers[i].y);
		input_report_abs(data->input_dev, ABS_MT_PRESSURE,
					data->fingers[i].z);
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, i);
		input_mt_sync(data->input_dev);
		num_fingers_down++;
	}
	data->finger_mask = 0;

	if (num_fingers_down == 0)
		input_mt_sync(data->input_dev);
	input_sync(data->input_dev);
}

static irqreturn_t egalax_ts_interrupt(int irq, void *dev_id)
{
	struct egalax_ts *data = dev_id;
	struct i2c_client *client = data->client;
	u8 buf[MAX_I2C_DATA_LEN];
	int id, ret, x, y, z;
	int tries = 0;
	bool down, valid;
	u8 state;

	memset(buf, 0, MAX_I2C_DATA_LEN);
	do {
		ret = i2c_master_recv(client, buf, MAX_I2C_DATA_LEN);
	} while (ret == -EAGAIN && tries++ < EGALAX_MAX_TRIES);

	if (ret < 0)
		return IRQ_HANDLED;

	if (buf[0] != REPORT_MODE_MTTOUCH) {
		/* ignore mouse events and vendor events */
		return IRQ_HANDLED;
	}

	state = buf[1];
	x = (buf[3] << 8) | buf[2];
	y = (buf[5] << 8) | buf[4];
	z = (buf[7] << 8) | buf[6];

	valid = state & EVENT_VALID_MASK;
	id = (state & EVENT_ID_MASK) >> EVENT_ID_OFFSET;
	down = state & EVENT_DOWN_UP;

	if (!valid || id >= MAX_SUPPORT_POINTS) {
		dev_dbg(&client->dev, "point invalid\n");
		return IRQ_HANDLED;
	}

	if (data->finger_mask & (1U << id))
		report_input_data(data);

	if (!down) {
		data->fingers[id].z = -1;
		data->finger_mask |= 1U << id;
	} else {
		data->fingers[id].x = x;
		data->fingers[id].y = y;
		data->fingers[id].z = z;
		data->finger_mask |= 1U << id;
	}

	dev_dbg(&client->dev, "%s id:%d x:%d y:%d z:%d\n",
		(down ? "down" : "up"), id, x, y, z);

	if (data->finger_mask)
		report_input_data(data);

	return IRQ_HANDLED;
}

/* wake up controller by an falling edge of interrupt gpio.  */
static int egalax_wake_up_device(struct i2c_client *client)
{
	int gpio = irq_to_gpio(client->irq);
	u8 buf[MAX_I2C_DATA_LEN];
	int ret, tries = 0;

	ret = gpio_request(gpio, "egalax_irq");
	if (ret < 0) {
		dev_err(&client->dev, "request gpio failed,"
			"cannot wake up controller:%d\n", ret);
		return ret;
	}
	/* wake up controller via an falling edge on IRQ gpio. */
	gpio_direction_output(gpio, 1);
	gpio_set_value(gpio, 0);
	gpio_set_value(gpio, 1);
	/* controller should be waken up, return irq.  */
	gpio_direction_input(gpio);
	gpio_free(gpio);

	/* If the touch controller has some data pending, read it */
	/* or the INT line will remian low */
	while ((gpio_get_value(gpio) == 0) && (tries++ < EGALAX_MAX_TRIES))
		i2c_master_recv(client, buf, MAX_I2C_DATA_LEN);

	return 0;
}

static int egalax_firmware_version(struct i2c_client *client)
{
	static const u8 cmd[MAX_I2C_DATA_LEN] = { 0x03, 0x03, 0xa, 0x01, 0x41 };
	int ret;
	ret = i2c_master_send(client, cmd, MAX_I2C_DATA_LEN);
	if (ret < 0)
		return ret;
	return 0;
}

static void egalax_early_suspend(struct early_suspend *h);
static void egalax_later_resume(struct early_suspend *h);

static int __devinit egalax_ts_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct egalax_ts *data;
	struct input_dev *input_dev;
	int ret;

	data = kzalloc(sizeof(struct egalax_ts), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_free_data;
	}

	data->client = client;
	data->input_dev = input_dev;
	/* controller may be in sleep, wake it up. */
	if (egalax_wake_up_device(client) != 0) {
		dev_info(&client->dev, "Failed to wake up, disable suspend,"
			 " otherwise it can not wake up\n");
		data->touch_no_wake = true;
	}
	msleep(10);
	/* the controller needs some time to wakeup, otherwise the
	 * following firmware version read will be failed.
	 */
	ret = egalax_firmware_version(client);
	if (ret < 0) {
		dev_err(&client->dev,
			"egalax_ts: failed to read firmware version\n");
		ret = -EIO;
		goto err_free_dev;
	}

	input_dev->name = "eGalax Touch Screen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, EGALAX_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, EGALAX_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_PRESSURE, 0, EGALAX_MAX_Z, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,
					MAX_SUPPORT_POINTS-1, 0, 0);

	input_set_drvdata(input_dev, data);

	ret = request_threaded_irq(client->irq, NULL, egalax_ts_interrupt,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"egalax_ts", data);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_dev;
	}

	ret = input_register_device(data->input_dev);
	if (ret < 0)
		goto err_free_irq;
	i2c_set_clientdata(client, data);

#ifdef CONFIG_EARLYSUSPEND
	/* Not register earlysuspend if not way to wake device. */
	if (data->touch_no_wake == false) {
		/* register this client's earlysuspend */
		data->es_handler.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
		data->es_handler.suspend = egalax_early_suspend;
		data->es_handler.resume = egalax_later_resume;
		data->es_handler.data = (void *)client;
		register_early_suspend(&data->es_handler);
	}
#endif

	return 0;

err_free_irq:
	free_irq(client->irq, data);
err_free_dev:
	input_free_device(input_dev);
err_free_data:
	kfree(data);

	return ret;
}

static __devexit int egalax_ts_remove(struct i2c_client *client)
{
	struct egalax_ts *data = i2c_get_clientdata(client);
#ifdef CONFIG_EARLYSUSPEND
	unregister_early_suspend(&data->es_handler);
#endif
	free_irq(client->irq, data);
	input_free_device(data->input_dev);
	input_unregister_device(data->input_dev);
	kfree(data);

	return 0;
}

static const struct i2c_device_id egalax_ts_id[] = {
	{"egalax_ts", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, egalax_ts_id);

#ifdef CONFIG_PM_SLEEP
static int egalax_ts_suspend(struct device *dev)
{
	int ret;
	u8 suspend_cmd[MAX_I2C_DATA_LEN] = {0x3, 0x6, 0xa, 0x3, 0x36,
					    0x3f, 0x2, 0, 0, 0};
	struct i2c_client *client = to_i2c_client(dev);
	struct egalax_ts *data = i2c_get_clientdata(client);

	/* If can not wake up, not suspend. */
	if (data->touch_no_wake) {
		dev_info(&client->dev, "not suspend because unable to wake up device\n");
		return 0;
	}
	ret = i2c_master_send(client, suspend_cmd, MAX_I2C_DATA_LEN);
	return ret > 0 ? 0 : ret;
}

static int egalax_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct egalax_ts *data = i2c_get_clientdata(client);

	/* If not wake up, don't needs resume. */
	if (data->touch_no_wake)
		return 0;
	return egalax_wake_up_device(client);
}
#endif

static SIMPLE_DEV_PM_OPS(egalax_ts_pm_ops, egalax_ts_suspend, egalax_ts_resume);
static struct i2c_driver egalax_ts_driver = {
	.driver = {
		.name = "egalax_ts",
#ifndef CONFIG_HAS_EARLYSUSPEND
		.pm	= &egalax_ts_pm_ops,
#endif
	},
	.id_table	= egalax_ts_id,
	.probe		= egalax_ts_probe,
	.remove		= __devexit_p(egalax_ts_remove),
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void egalax_early_suspend(struct early_suspend *h)
{
	u8 suspend_cmd[MAX_I2C_DATA_LEN] = {0x3, 0x6, 0xa, 0x3, 0x36,
					    0x3f, 0x2, 0, 0, 0};

	if (h->data == NULL)
		return;
	i2c_master_send((struct i2c_client *)h->data,
				suspend_cmd, MAX_I2C_DATA_LEN);
}

static void egalax_later_resume(struct early_suspend *h)
{
	if (h->data)
		egalax_wake_up_device((struct i2c_client *)h->data);
}
#else
static void egalax_early_suspend(struct early_suspend *h)
{
}
static void egalax_later_resume(struct early_suspend *h)
{
}
#endif

static int __init egalax_ts_init(void)
{
	return i2c_add_driver(&egalax_ts_driver);
}

static void __exit egalax_ts_exit(void)
{
	i2c_del_driver(&egalax_ts_driver);
}

module_init(egalax_ts_init);
module_exit(egalax_ts_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Touchscreen driver for EETI eGalax touch controller");
MODULE_LICENSE("GPL");
