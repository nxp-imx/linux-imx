/* drivers/input/touchscreen/ft5x06_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include "ft5x46_ts.h"

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define FTS_CTL_IIC
//#define FTS_GESTRUE
#define FT5X0X_ENABLE_IRQ

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

#ifdef FTS_GESTRUE
#include "ft_gesture_lib.h"
short pointnum = 0;
int gestrue_id = 0;
#endif

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
							   0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};

struct ft5x0x_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	unsigned int x_max;
	unsigned int y_max;
	struct gpio_desc *reset_gpio;
	struct workqueue_struct *workqueue;
	struct work_struct work;
};

#define ANDROID_INPUT_PROTOCOL_B

/*
*ft5x0x_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

/*write data by i2c*/
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

#ifdef FTS_GESTRUE
static int ft5x0x_read_Touchdata(struct ft5x0x_ts_data *data)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 2] = { 0 };
	int ret = -1;
	int i = 0;
	buf[0] = 0xd3;

	ret = ft5x0x_i2c_Read(data->client, buf, 1, buf, 8);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}

	if (0x24 == buf[0]) {
		gestrue_id = 0x24;
		dev_warn(&data->client->dev, "The Gesture ID is %x %x \n",
			 gestrue_id, pointnum);
		return -1;
	}

	pointnum = (short)(buf[1]) & 0xff;

	buf[0] = 0xd3;

	ret = ft5x0x_i2c_Read(data->client, buf, 1, buf, (pointnum * 4 + 8));
	/* Read two times */
	//ret = ft5x0x_i2c_Read(data->client, buf, 1, buf, (8));
	//ret = ft5x0x_i2c_Read(data->client, buf, 0, (buf+8), (8));

	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}

	gestrue_id = fetch_object_sample(buf, pointnum);

	dev_warn(&data->client->dev, "The Gesture ID is %x %x\n", gestrue_id,
		 pointnum);

	for (i = 0; i < pointnum; i++) {
		coordinate_x[i] = (((s16) buf[0 + (4 * i)]) & 0x0F) <<
		    8 | (((s16) buf[1 + (4 * i)]) & 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
		    8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}

	return -1;
}
#else
/*Read touch point information when the interrupt  is asserted.*/
static int ft5x0x_read_Touchdata(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = ft5x0x_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
		    (((s16) buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i]) & 0x0F)
		    << 8 | (((s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i]) &
			    0xFF);
		event->au16_y[i] =
		    (((s16) buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i]) & 0x0F)
		    << 8 | (((s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i]) &
			    0xFF);
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
#if 0
		dev_info(&data->client->dev, "id=%d event=%d x=%d y=%d\n",
			 event->au8_finger_id[i], event->au8_touch_event[i],
			 event->au16_x[i], event->au16_y[i]);
#endif
	}

	event->pressure = FT_PRESS;

	return 0;
}
#endif

/*
*report the point information
*/
static void ft5x0x_report_value(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;

	/*protocol B */
	for (i = 0; i < event->touch_point; i++) {
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);

		if (event->au8_touch_event[i] == 0
		    || event->au8_touch_event[i] == 2) {
			input_event(data->input_dev, EV_ABS, ABS_X,
				    event->au16_x[i]);
			input_event(data->input_dev, EV_ABS, ABS_Y,
				    event->au16_y[i]);
			input_event(data->input_dev, EV_KEY, BTN_TOUCH, 1);
			input_report_abs(data->input_dev, ABS_PRESSURE, 1);
			input_sync(data->input_dev);
		} else {
			uppoint++;
			input_mt_report_slot_state(data->input_dev,
						   MT_TOOL_FINGER, false);
		}
	}

	if (event->touch_point == uppoint) {
		input_report_key(data->input_dev, BTN_TOUCH, 0);
	} else
		input_report_key(data->input_dev, BTN_TOUCH,
				 event->touch_point > 0);
	input_sync(data->input_dev);

}

static void ft5x0x_ts_work(struct work_struct *work)
{
	struct ft5x0x_ts_data *ft5x0x_ts =
	    container_of(work, struct ft5x0x_ts_data, work);
	int ret;

	disable_irq_nosync(ft5x0x_ts->client->irq);

	ret = ft5x0x_read_Touchdata(ft5x0x_ts);
	if (ret == 0)
		ft5x0x_report_value(ft5x0x_ts);

	enable_irq(ft5x0x_ts->client->irq);
}

/*The ft5x0x device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
	queue_work(ft5x0x_ts->workqueue, &ft5x0x_ts->work);
	return IRQ_HANDLED;
}

void ft5x0x_reset_tp(struct i2c_client *client, int HighOrLow)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	if (ft5x0x_ts->reset_gpio) {
		dev_info(&client->dev, "set tp reset pin to %d\n", HighOrLow);
		gpiod_set_value(ft5x0x_ts->reset_gpio, HighOrLow);
	} else
		dev_warn(&client->dev, "reset pin is not set\n");
}

static void fts_un_init_gpio_hw(struct ft5x0x_ts_data *ft5x0x_ts)
{
	if (ft5x0x_ts->reset_gpio) {
		devm_gpiod_put(&ft5x0x_ts->client->dev, ft5x0x_ts->reset_gpio);
		ft5x0x_ts->reset_gpio = NULL;
	}
}

static int ft5x0x_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	/*struct ft5x0x_platform_data *pdata =
	   (struct ft5x0x_platform_data *)client->dev.platform_data; */
	struct device_node *node = client->dev.of_node;
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;

#ifdef FTS_GESTRUE
	u8 auc_i2c_write_buf[2] = { 0 };
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(struct ft5x0x_ts_data), GFP_KERNEL);

	if (!ft5x0x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	ft5x0x_ts->client = client;
	i2c_set_clientdata(client, ft5x0x_ts);

	if (of_property_read_u32(node, "x-max", &ft5x0x_ts->x_max)) {
		dev_err(&client->dev, "x-resolution is not set\n");
		ft5x0x_ts->x_max = 800;
	}
	if (of_property_read_u32(node, "y-max", &ft5x0x_ts->y_max)) {
		dev_err(&client->dev, "y-resolution is not set\n");
		ft5x0x_ts->y_max = 600;
	}
	--ft5x0x_ts->x_max;
	--ft5x0x_ts->y_max;

	dev_info(&client->dev, "resolution: %dx%d\n", ft5x0x_ts->x_max,
		 ft5x0x_ts->y_max);

	ft5x0x_ts->reset_gpio = devm_gpiod_get(&client->dev,
					       "reset", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(ft5x0x_ts->reset_gpio)) {
		err =
		    ft5x0x_ts->
		    reset_gpio ? PTR_ERR(ft5x0x_ts->reset_gpio) : -EINVAL;
		ft5x0x_ts->reset_gpio = NULL;
		dev_warn(&client->dev, "Operating withoug nReset pin (%d)\n",
			 err);
	}

	if (client->irq) {
		err =
		    devm_request_irq(&client->dev, client->irq,
				     ft5x0x_ts_interrupt,
				     IRQF_TRIGGER_FALLING,
				     client->dev.driver->name, ft5x0x_ts);

		if (err < 0) {
			dev_err(&client->dev, "request irq failed\n");
			goto exit_irq_request_failed;
		}
		disable_irq(client->irq);
	} else {
		dev_err(&client->dev, "mode without irq is not supported\n");
		goto exit_irq_request_failed;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);

	//input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS);
	input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, ft5x0x_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, ft5x0x_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_X, 0, ft5x0x_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ft5x0x_ts->y_max, 0, 0);

	input_dev->name = FT5X0X_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ft5x0x_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	/*make sure CTP already finish startup process */
	msleep(150);

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev,
			"%s:[FTS] create fts control iic driver failed\n",
			__func__);
#endif

	/*get some register information */
	uc_reg_addr = FT5x0x_REG_FW_VER;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] Firmware version = 0x%x\n", uc_reg_value);

	uc_reg_addr = FT5x0x_REG_POINT_RATE;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] report rate is %dHz.\n", uc_reg_value * 10);

	uc_reg_addr = FT5X0X_REG_THGROUP;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] touch threshold is %d.\n", uc_reg_value * 4);

#ifdef FTS_GESTRUE
	init_para(720, 1280, 100, 0, 0);

	auc_i2c_write_buf[0] = 0xd0;
	auc_i2c_write_buf[1] = 0x01;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 2);	//let fw open gestrue function

	auc_i2c_write_buf[0] = 0xd1;
	auc_i2c_write_buf[1] = 0xff;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 2);
	/*
	   auc_i2c_write_buf[0] = 0xd0;
	   auc_i2c_write_buf[1] = 0x00;
	   ft5x0x_i2c_Write(client, auc_i2c_write_buf, 2);      //let fw close gestrue function
	 */
#endif

	ft5x0x_ts->workqueue = create_singlethread_workqueue("ft5x0x_ts");

	if (ft5x0x_ts->workqueue == NULL) {
		dev_err(&client->dev, "failed to create workqueue\n");
		err = -ENOMEM;
		goto exit_input_register_device_failed;
	}
	INIT_WORK(&ft5x0x_ts->work, ft5x0x_ts_work);

	enable_irq(client->irq);
	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
	devm_free_irq(&client->dev, client->irq, ft5x0x_ts);

	fts_un_init_gpio_hw(ft5x0x_ts);

exit_irq_request_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

#ifdef CONFIG_PM_SLEEP
static int ft5x0x_ts_suspend(struct device *dev)
{
	struct ft5x0x_ts_data *ts = dev_get_drvdata(dev);

	dev_dbg(&ts->client->dev, "[FTS]ft5x0x suspend\n");
	disable_irq(ts->client->irq);
	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int ft5x0x_ts_resume(struct device *dev)
{
	struct ft5x0x_ts_data *ts = dev_get_drvdata(dev);

	dev_dbg(&ts->client->dev, "[FTS]ft5x0x resume.\n");
	pinctrl_pm_select_default_state(dev);
	ft5x0x_reset_tp(ts->client, 0);
	msleep(20);
	ft5x0x_reset_tp(ts->client, 1);
	enable_irq(ts->client->irq);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops ft5x0x_ts_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ft5x0x_ts_suspend, ft5x0x_ts_resume)
};

static int ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	ft5x0x_ts = i2c_get_clientdata(client);
	input_unregister_device(ft5x0x_ts->input_dev);

	destroy_workqueue(ft5x0x_ts->workqueue);

#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
#endif

	devm_free_irq(&client->dev, client->irq, ft5x0x_ts);

	fts_un_init_gpio_hw(ft5x0x_ts);

	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);

	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{FT5X0X_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static const struct of_device_id ft5x0x_ts_of_match[] = {
	{.compatible = "focaltech,ft5x2"},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ft5x0x_ts_of_match);

static struct i2c_driver ft5x0x_ts_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = FT5X0X_NAME,
		   .pm = &ft5x0x_ts_ops,
		   .of_match_table = ft5x0x_ts_of_match,
		   },
	.id_table = ft5x0x_ts_id,
	.probe = ft5x0x_ts_probe,
	.remove = ft5x0x_ts_remove,
};

module_i2c_driver(ft5x0x_ts_driver);

MODULE_AUTHOR("<luowj>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
