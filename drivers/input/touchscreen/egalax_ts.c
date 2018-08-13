/*
 * Driver for EETI eGalax Multiple Touch Controller
 *
 * Copyright (C) 2011-2015 Freescale Semiconductor, Inc.
 *
 * based on max11801_ts.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* EETI eGalax serial touch screen controller is a I2C based multiple
 * touch screen controller, it supports 5 point multiple touch. */

/* TODO:
  - auto idle mode support
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
#include <linux/of_gpio.h>
/*
 * Mouse Mode: some panel may configure the controller to mouse mode,
 * which can only report one point at a given time.
 * This driver will ignore events in this mode.
 */
#define REPORT_MODE_MOUSE		0x1
/*
 * Vendor Mode: this mode is used to transfer some vendor specific
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
	u32				finger_mask;
	int				touch_no_wake;
	struct finger_info		fingers[MAX_SUPPORT_POINTS];
};

static void report_input_data(struct egalax_ts *ts)
{
	int i;
	int num_fingers_down;

	num_fingers_down = 0;
	for (i = 0; i < MAX_SUPPORT_POINTS; i++) {
		if (ts->fingers[i].z == -1)
			continue;

		input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					ts->fingers[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					ts->fingers[i].y);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					ts->fingers[i].z);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
		input_mt_sync(ts->input_dev);
		num_fingers_down++;
	}
	ts->finger_mask = 0;

	if (num_fingers_down == 0)
		input_mt_sync(ts->input_dev);
	input_sync(ts->input_dev);
}
static irqreturn_t egalax_ts_interrupt(int irq, void *dev_id)
{
	struct egalax_ts *ts = dev_id;
	struct i2c_client *client = ts->client;
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
		/* invalid point */

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

	if (ts->finger_mask & (1U << id))
		report_input_data(ts);
	if (!down) {
		ts->fingers[id].z = -1;
		ts->finger_mask |= 1U << id;

	} else {
		ts->fingers[id].x = x;
		ts->fingers[id].y = y;
		ts->fingers[id].z = z;
		ts->finger_mask |= 1U << id;
	}

	/* report all pointers */
	dev_dbg(&client->dev, "%s id:%d x:%d y:%d z:%d\n",
		(down ? "down" : "up"), id, x, y, z);

	if (ts->finger_mask)
		report_input_data(ts);

	return IRQ_HANDLED;
}

static int egalax_irq_request(struct egalax_ts *ts)
{
	int ret;
	struct i2c_client *client = ts->client;

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					  egalax_ts_interrupt,
					  IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					  "egalax_ts", ts);
	if (ret < 0)
		dev_err(&client->dev, "Failed to register interrupt\n");

	return ret;
}

static void egalax_free_irq(struct egalax_ts *ts)
{
	devm_free_irq(&ts->client->dev, ts->client->irq, ts);
}

/* wake up controller by an falling edge of interrupt gpio.  */
static int egalax_wake_up_device(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	int gpio;
	u8 buf[MAX_I2C_DATA_LEN];
	int ret, tries = 0;

	if (!np)
		return -ENODEV;

	gpio = of_get_named_gpio(np, "wakeup-gpios", 0);
	if (!gpio_is_valid(gpio))
		return -ENODEV;

	ret = gpio_request(gpio, "egalax_irq");
	if (ret < 0) {
		dev_err(&client->dev,
			"request gpio failed, cannot wake up controller: %d\n",
			ret);
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


static int egalax_ts_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct egalax_ts *ts;
	struct input_dev *input_dev;
	int ret;
	int error;

	ts = kzalloc(sizeof(struct egalax_ts), GFP_KERNEL);
	if (!ts) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_ts;
	}

	ts->client = client;
	ts->input_dev = input_dev;

	/* HannStar (HSD100PXN1 Rev: 1-A00C11 F/W:0634) LVDS touch
	 * screen needs to trigger I2C event to device FW at booting
	 * first, and then the FW can switch to I2C interface.
	 * Otherwise, the FW can’t  work with I2C interface. So here
	 * just use the exist function egalax_firmware_version() to
	 * send a I2C command to the device, make sure the device FW
	 * switch to I2C interface.
	 */
	error = egalax_firmware_version(client);
	if (error) {
		dev_err(&client->dev, "Failed to switch to I2C interface\n");
		return error;
	}

	/* controller may be in sleep, wake it up. */
	error = egalax_wake_up_device(client);
	if (error) {
		dev_err(&client->dev, "Failed to wake up, disable suspend,"
				"otherwise it can not wake up\n");
		ts->touch_no_wake = true;
	}
	msleep(10);

	ret = egalax_firmware_version(client);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read firmware version\n");
		error = -EIO;
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
			     MAX_SUPPORT_POINTS - 1, 0, 0);

	error = egalax_irq_request(ts);
	if (error < 0) {
		goto err_free_dev;
	}

	error = input_register_device(ts->input_dev);
	if (error)
		goto err_free_irq;

	return 0;

err_free_irq:
	free_irq(client->irq, ts);
err_free_dev:
	input_free_device(input_dev);
err_free_ts:
	kfree(ts);

	return error;
}

static int egalax_ts_remove(struct i2c_client *client)
{
	struct egalax_ts *ts = i2c_get_clientdata(client);
	free_irq(client->irq, ts);
	input_free_device(ts->input_dev);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	i2c_set_clientdata(client, ts);

	return 0;
}

static const struct i2c_device_id egalax_ts_id[] = {
	{ "egalax_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, egalax_ts_id);

static int __maybe_unused egalax_ts_suspend(struct device *dev)
{
	static const u8 suspend_cmd[MAX_I2C_DATA_LEN] = {
		0x3, 0x6, 0xa, 0x3, 0x36, 0x3f, 0x2, 0, 0, 0
	};
	struct i2c_client *client = to_i2c_client(dev);
	struct egalax_ts *ts = i2c_get_clientdata(client);
	int ret;

	egalax_free_irq(ts);

	/* If can not wake up, not suspend. */
	if (ts->touch_no_wake) {
		dev_info(&client->dev,
				"not suspend because unable to wake up device\n");
		return 0;
	}
	ret = i2c_master_send(client, suspend_cmd, MAX_I2C_DATA_LEN);
	return ret > 0 ? 0 : ret;
}

static int __maybe_unused egalax_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct egalax_ts *ts = i2c_get_clientdata(client);
	int ret;
	/* If not wake up, don't needs resume. */
	if (ts->touch_no_wake)
		return 0;

	ret = egalax_wake_up_device(client);
	if (!ret)
		ret = egalax_irq_request(ts);

	return ret;
}

static SIMPLE_DEV_PM_OPS(egalax_ts_pm_ops, egalax_ts_suspend, egalax_ts_resume);

static const struct of_device_id egalax_ts_dt_ids[] = {
	{ .compatible = "eeti,egalax_ts" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, egalax_ts_dt_ids);

static struct i2c_driver egalax_ts_driver = {
	.driver = {
		.name	= "egalax_ts",
		.of_match_table	= of_match_ptr(egalax_ts_dt_ids),
	},
	.id_table	= egalax_ts_id,
	.probe		= egalax_ts_probe,
	.remove		= egalax_ts_remove,
};

module_i2c_driver(egalax_ts_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Touchscreen driver for EETI eGalax touch controller");
MODULE_LICENSE("GPL");
