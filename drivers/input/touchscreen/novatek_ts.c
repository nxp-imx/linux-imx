/*
 * Driver for Novatek NT11003 Multiple Touch Controller
 *
 * Copyright (C) 2012 Novatek Ltd.
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/i2c/novatek_ts.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define NOVATEK_I2C_NAME	"novatek-ts"
#define MAX_SUPPORT_POINTS	5
#define FINGER_EVENT_LEN	6

#define NOVATEK_MAX_X		1280
#define NOVATEK_MAX_Y		800
#define NT_QUIRK_ID		((0xFF >> 3) - 1)

struct tp_event {
	u16 x;
	u16 y;
	s16 id;
	u16 pressure;
	u8 status;
};

struct novatek_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	u8 fingers[MAX_SUPPORT_POINTS];
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
};

static struct i2c_client *this_client;

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend novatek_power;
static void novatek_suspend_early(struct early_suspend *h);
static void novatek_resume_early(struct early_suspend *h);
#endif

#define FINGER_STATUS_MASK 0x3;
enum {
	FINGER_DOWN = 1,
	FINGER_MOVE,
	FINGER_UP,
};

static int novatek_init_panel(struct novatek_ts_data *ts)
{
	ts->abs_x_max = NOVATEK_MAX_X;
	ts->abs_y_max = NOVATEK_MAX_Y;
	ts->max_touch_num = MAX_SUPPORT_POINTS;

	return 0;
}

static void parser_finger_events(u8 *buf, struct tp_event *event)
{
	event->id = (buf[0] >> 3) - 1;
	event->status = buf[0] & FINGER_STATUS_MASK;
	event->x = (buf[1] << 4) | ((buf[3] & 0xf0) >> 4);
	event->y = (buf[2] << 4) | (buf[3] & 0xf);
	event->pressure = buf[5];
}

static int novatek_ts_chipid(struct i2c_client *client)
{
	struct novatek_ts_data *ts = i2c_get_clientdata(client);
	u8 id_data[] = { 0xff, 0xf0, 0x00 };
	int ret;

	ret = i2c_master_send(ts->client, id_data, ARRAY_SIZE(id_data));

	if (ret < 0)
		return -ret;

	return i2c_smbus_read_byte_data(ts->client, 0);
}

static irqreturn_t novatek_ts_threaded_irq_handler(int irq, void *dev_id)
{
	struct novatek_ts_data *ts = dev_id;
	struct i2c_client *client = ts->client;
	struct input_dev *input_dev = ts->input_dev;
	u8 buffer[MAX_SUPPORT_POINTS * FINGER_EVENT_LEN];
	struct tp_event event;
	bool down;
	int ret;
	int i;

	memset(buffer, 0, ARRAY_SIZE(buffer));
	ret = i2c_smbus_read_i2c_block_data(client, 0,
					ARRAY_SIZE(buffer), buffer);

	dev_vdbg(&client->dev, "------------------------\n");
	for (i = 0; i < ARRAY_SIZE(buffer); i++)
		dev_vdbg(&client->dev, "reg:%d val:0x%X\n", i, buffer[i]);

	for (i = 0; i < MAX_SUPPORT_POINTS; i++) {
		memset(&event, 0, sizeof(event));
		parser_finger_events(&buffer[i * FINGER_EVENT_LEN], &event);

		/* workaround FW sometime report touch up is not
		 * corrent, but report all 0xFF package, so it lost
		 * track of ID, so workaround by add a touch ID by the
		 * parser id. */
		if (ts->fingers[i] == FINGER_MOVE && event.status == FINGER_UP
		    && event.id == NT_QUIRK_ID)
			event.id = i;

		if (event.status == 0)
			continue;

		/* ignore the event already up. */
		if (event.status == FINGER_UP && ts->fingers[i] == FINGER_UP)
			continue;

		input_mt_slot(input_dev, event.id);

		down = (event.status == FINGER_UP) ? false : true;

		dev_dbg(&client->dev,
			"id: %d status:%d x:%d  y:%d pressure:%d down:%d\n",
			event.id, event.status, event.x, event.y,
			event.pressure, down);

		ts->fingers[i] = event.status;

		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, down);

		if (down) {
			input_report_abs(input_dev, ABS_MT_POSITION_X, event.x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, event.y);
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					event.pressure);
		}

		input_mt_report_pointer_emulation(input_dev, true);
		input_sync(input_dev);
	}

	return IRQ_HANDLED;
}

static int novatek_gpio_reset_chip(struct i2c_client *client, int gpio)
{
	int ret;
	ret = gpio_request(gpio, "novatek reset");
	if (ret) {
		dev_err(&client->dev, "failed to request reset gpio.\n");
		return ret;
	}

	gpio_direction_output(gpio, 1);
	udelay(1);
	gpio_set_value(gpio, 0);
	msleep(25);
	gpio_set_value(gpio, 1);

	gpio_free(gpio);

	/* This chip needs time after reset pin. Otherwise, the i2c
	 * command will failed.*/
	msleep(25);

	return 0;
}

static int novatek_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	int chipid = 0;
	int reset_gpio;
	struct novatek_ts_data *ts;
	struct novatek_platform_data *pdata;

	pdata = client->dev.platform_data;
	reset_gpio = pdata->reset_gpio;

	if (reset_gpio > 0)
		novatek_gpio_reset_chip(client, reset_gpio);
	else
		dev_warn(&client->dev,
			"no reset gpio given, can not reset chip\n");


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ret = novatek_init_panel(ts);
	ts->client = this_client = client;
	i2c_set_clientdata(client, ts);

	chipid = novatek_ts_chipid(ts->client);
	if (chipid < 0) {
		dev_err(&client->dev, "read chip id failed: %d\n", chipid);
		goto err_init_panel_fail;
	} else
		dev_info(&client->dev, "success read chip id:%x\n", chipid);

	ret = request_threaded_irq(client->irq, NULL,
				novatek_ts_threaded_irq_handler,
				IRQF_TRIGGER_FALLING, "novatek_ts", ts);
	if (ret != 0) {
		dev_err(&client->dev, "request irq failed.\n");
		goto err_irq_request_failed;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(BTN_TOUCH, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev,
			     ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev,
			     ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_mt_init_slots(ts->input_dev, MAX_SUPPORT_POINTS);

	input_set_drvdata(ts->input_dev, ts);

	ts->input_dev->name = "Novatek NT11003 Touch Screen";
	ts->input_dev->id.bustype = BUS_I2C;

	ret = input_register_device(ts->input_dev);
	if (ret != 0) {
		dev_err(&client->dev,
			"Probe: unable to register %s input device\n",
			ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	i2c_set_clientdata(client, ts);

#ifdef CONFIG_HAS_EARLYSUSPEND
	novatek_power.suspend = novatek_suspend_early;
	novatek_power.resume = novatek_resume_early;
	novatek_power.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	novatek_power.data  = &client->dev;
	register_early_suspend(&novatek_power);
#endif
	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	free_irq(client->irq, ts);
err_irq_request_failed:
err_init_panel_fail:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int novatek_ts_remove(struct i2c_client *client)
{
	struct novatek_ts_data *ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&novatek_power);
#endif
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	free_irq(client->irq, ts);
	kfree(ts);

	return 0;
}

#ifdef CONFIG_PM

static int novatek_suspend_resume_cmd(struct i2c_client *client, bool suspend)
{
	int ret;
	uint8_t serial_mode_data[] = { 0xFF, 0x8F, 0xFF };
	uint8_t resume_data[] = { 0x00, 0x00 };
	uint8_t suspend_data[] = { 0x00, 0xAE };

	ret = i2c_master_send(client, serial_mode_data,
				ARRAY_SIZE(serial_mode_data));
	if (ret < 0) {
		dev_err(&client->dev, "i2c master send failed:%d\n", ret);
		goto err;
	}

	if (suspend)
		ret = i2c_master_send(client, suspend_data,
				      ARRAY_SIZE(suspend_data));
	else
		ret = i2c_master_send(client, resume_data,
				      ARRAY_SIZE(resume_data));

	if (ret < 0) {
		dev_err(&client->dev, "i2c master send failed:%d\n", ret);
		goto err;
	}

	return 0;

err:
	return ret;
}

int novatek_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	return novatek_suspend_resume_cmd(client, true);
}

int novatek_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	return novatek_suspend_resume_cmd(client, false);
}

#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void novatek_suspend_early(struct early_suspend *h)
{
	struct device *dev = h->data;
	novatek_ts_suspend(dev);
}

static void novatek_resume_early(struct early_suspend *h)
{
	struct device *dev = h->data;
	novatek_ts_resume(dev);
}
#endif

static const struct i2c_device_id novatek_ts_id[] = {
	{NOVATEK_I2C_NAME, 0},
	{}
};

static SIMPLE_DEV_PM_OPS(novatek_ts_pm_ops, novatek_ts_suspend, \
			 novatek_ts_resume);
static struct i2c_driver novatek_ts_driver = {
	.driver = {
		.name = NOVATEK_I2C_NAME,
		.owner = THIS_MODULE,
#if (defined CONFIG_PM) && !(defined CONFIG_HAS_EARLYSUSPEND)
		.pm = &novatek_ts_pm_ops,
#endif
	},
	.probe = novatek_ts_probe,
	.remove = novatek_ts_remove,
	.id_table = novatek_ts_id,

};

static int __devinit novatek_ts_init(void)
{
	return i2c_add_driver(&novatek_ts_driver);
}

static void __exit novatek_ts_exit(void)
{
	i2c_del_driver(&novatek_ts_driver);
}

module_init(novatek_ts_init);
module_exit(novatek_ts_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Touchscreen driver for Novatek NT11003 touch controller");
MODULE_LICENSE("GPL");
