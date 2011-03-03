/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>

struct point_state {
	int x1;
	int y1;
	int x2;
	int y2;
	int state;
};

struct p1003_priv {
	struct i2c_client *client;
	struct input_dev *input;
	char phys[32];
	unsigned int irq;
	struct work_struct work;
	struct point_state old_state;
	struct workqueue_struct *workqueue;
};

#define POINTER_DATA_LEN		9
#define PANEL_INFO_LEN			6
#define READ_TOUCH_INFO_REPORT		0x10
#define READ_PANEL_INFO			0x20
#define MASS_PRODUCTION_CALIBRATION	0xcc

static int calibration_env(struct i2c_client *client);

static ssize_t p1003_cal_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Do touch panel hardware"
		       "calibration by write \"calibration\"\n");
}

static ssize_t p1003_cal_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct p1003_priv *priv = dev_get_drvdata(dev);
	struct i2c_client *client = priv->client;
	const char *str = "calibration";

	if (strncmp(buf, str, strlen(str)) == 0) {
		calibration_env(client);
		return count;
	} else
		return -EINVAL;
}

static DEVICE_ATTR(calibrate, 0664, p1003_cal_show, p1003_cal_store);

static struct attribute *p1003_attributs[] = {
	&dev_attr_calibrate.attr,
	NULL
};

static const struct attribute_group p1003_attr_group = {
	.attrs = p1003_attributs,
};

/* Mass Production Calibration function.
 *
 * The touch panel is calibrated with the system environmnet
 *
 * Notes:
 * - Only suggested to be used for mass production purpose.
 * - When use this command, it very important to avoid any touch
     object surrounding the whole system
 * - need some time to execute, it takes about 5 seconds to be
     finished.
 */
static int calibration_env(struct i2c_client *client)
{
	int ret;
	dev_info(&client->dev, "calibration started\n");
	ret = i2c_smbus_write_byte_data(client, 0xCC, 0);
	if (ret < 0)
		return ret;
	mdelay(5000);
	dev_info(&client->dev, "calibration finished\n");
	return 0;
}

static int p1003_i2c_read_packet(struct i2c_client *client, char *value)
{
	return i2c_smbus_read_i2c_block_data(client, READ_TOUCH_INFO_REPORT,
					     POINTER_DATA_LEN, (u8 *) value);
}

static void p1003_work(struct work_struct *work)
{
	struct p1003_priv *p1003 = container_of(work, struct p1003_priv, work);
	struct i2c_client *client = p1003->client;
	struct p1003_ts_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input = p1003->input;
	struct point_state *old_state = &p1003->old_state;
	char data[POINTER_DATA_LEN];
	int x1, x2, y1, y2;

	/* the sample can only be read when intr pin low */
	while (!pdata->hw_status()) {
		if (p1003_i2c_read_packet(client, data) < 0) {
			dev_err(&client->dev, "read i2c packet failed\n");
			continue;
		}

		if (data[0] == 0x01) {
			/* single point */
			x1 = (data[2] << 8) | data[1];
			y1 = (data[4] << 8) | data[3];

			/* If it's same point, free some CPU */
			if (old_state->state == data[0] &&
			    old_state->x1 == x1 && old_state->y1 == y1) {
				msleep(1);
				continue;
			}
			input_event(input, EV_ABS, ABS_MT_POSITION_X, x1);
			input_event(input, EV_ABS, ABS_MT_POSITION_Y, y1);
			input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
			input_mt_sync(input);
			input_event(input, EV_ABS, ABS_X, x1);
			input_event(input, EV_ABS, ABS_Y, y1);
			input_event(input, EV_KEY, BTN_TOUCH, 1);
			input_sync(input);
			old_state->x1 = x1;
			old_state->y1 = y1;
			old_state->state = 0x01;
		} else if (data[0] == 0x03 || data[0] == 0x02) {
			/* two point */
			x1 = (data[2] << 8) | data[1];
			y1 = (data[4] << 8) | data[3];
			x2 = (data[6] << 8) | data[5];
			y2 = (data[8] << 8) | data[7];

			/* If they are same points, free some CPU */
			if (old_state->state == data[0] &&
			    old_state->x1 == x1 && old_state->y1 == y1 &&
			    old_state->x2 == x2 && old_state->y2 == y2) {
				msleep(1);
				continue;
			}
			input_event(input, EV_ABS, ABS_MT_POSITION_X, x1);
			input_event(input, EV_ABS, ABS_MT_POSITION_Y, y1);
			input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
			input_mt_sync(input);
			input_event(input, EV_ABS, ABS_MT_POSITION_X, x2);
			input_event(input, EV_ABS, ABS_MT_POSITION_Y, y2);
			input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
			input_mt_sync(input);
			input_sync(input);
			old_state->state = data[0];
			old_state->x1 = x1;
			old_state->y1 = y1;
			old_state->x2 = x2;
			old_state->y2 = y2;
		}
	};

	/* the irq is high now, means figure is leave the panel, send
	 * release to user space. */
	input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
	input_mt_sync(input);
	input_event(input, EV_KEY, BTN_TOUCH, 0);
	input_sync(input);
	old_state->state = data[0];
}

static irqreturn_t p1003_irq(int irq, void *handle)
{
	struct p1003_priv *p1003 = handle;
	queue_work(p1003->workqueue, &p1003->work);
	return IRQ_HANDLED;
}

static int read_max_range(struct i2c_client *client, int *xmax, int *ymax)
{
	char buf[PANEL_INFO_LEN];
	int x, y, ret;

	ret = i2c_smbus_read_i2c_block_data(client, READ_PANEL_INFO,
					    PANEL_INFO_LEN, buf);
	if (ret < 0)
		return ret;

	x = buf[0];
	x |= buf[1] << 8;

	y = buf[2];
	y |= buf[3] << 8;

	*xmax = x;
	*ymax = y;
	dev_info(&client->dev, "max range: x:%d y:%d \n", x, y);
	return 0;
}

static int __devinit p1003_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret, xmax, ymax;
	struct p1003_priv *p1003;
	struct input_dev *input_dev;
	struct p1003_ts_platform_data *pdata = client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
		dev_err(&client->dev, "I2C don't support enough function");
		return -EIO;
	}

	if (!pdata || !pdata->hw_status) {
		dev_err(&client->dev, "No hw status function!\n");
		return -EIO;
	}

	p1003 = kzalloc(sizeof(struct p1003_priv), GFP_KERNEL);
	if (!p1003)
		return -ENOMEM;

	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		goto err_free_mem;
	}

	p1003->client = client;
	p1003->irq = client->irq;
	p1003->input = input_dev;
	p1003->workqueue = create_singlethread_workqueue("p1003");
	INIT_WORK(&p1003->work, p1003_work);

	if (p1003->workqueue == NULL) {
		dev_err(&client->dev, "couldn't create workqueue\n");
		ret = -ENOMEM;
		goto err_free_dev;
	}

	snprintf(p1003->phys, sizeof(p1003->phys),
		 "%s/input0", dev_name(&client->dev));

	if (read_max_range(client, &xmax, &ymax) < 0) {
		dev_err(&client->dev, "couldn't read panel infomation.\n");
		ret = -EIO;
		goto err_free_wq;
	}

	input_dev->name = "HannStar P1003 Touchscreen";
	input_dev->phys = p1003->phys;
	input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, xmax, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ymax, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);

	ret = input_register_device(input_dev);
	if (ret)
		goto err_free_wq;

	i2c_set_clientdata(client, p1003);

	ret = sysfs_create_group(&client->dev.kobj, &p1003_attr_group);
	if (ret)
		goto err_free_wq;

	/* set irq type to edge falling */
	set_irq_type(p1003->irq, IRQF_TRIGGER_FALLING);
	ret = request_irq(p1003->irq, p1003_irq, 0,
			  client->dev.driver->name, p1003);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register irq %d!\n",
			p1003->irq);
		goto err_unreg_dev;
	}

	if (!pdata->hw_status())
		queue_work(p1003->workqueue, &p1003->work);

	return 0;
err_unreg_dev:
	input_unregister_device(input_dev);
err_free_wq:
	destroy_workqueue(p1003->workqueue);
err_free_dev:
	input_free_device(input_dev);
err_free_mem:
	kfree(p1003);
	return ret;
}

static int __devexit p1003_remove(struct i2c_client *client)
{
	struct p1003_priv *p1003 = i2c_get_clientdata(client);
	free_irq(p1003->irq, p1003);
	cancel_work_sync(&p1003->work);
	destroy_workqueue(p1003->workqueue);
	input_unregister_device(p1003->input);
	input_free_device(p1003->input);
	kfree(p1003);

	return 0;
}

static const struct i2c_device_id p1003_idtable[] = {
	{"p1003_fwv33", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, p1003_idtable);

static struct i2c_driver p1003_driver = {

	.driver = {
		   .owner = THIS_MODULE,
		   .name = "p1003_fwv33",
		   },
	.id_table = p1003_idtable,
	.probe = p1003_probe,
	.remove = __devexit_p(p1003_remove),
};

static int __init p1003_init(void)
{
	return i2c_add_driver(&p1003_driver);
}

static void __exit p1003_exit(void)
{
	i2c_del_driver(&p1003_driver);
}

module_init(p1003_init);
module_exit(p1003_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("HannStar P1003 multitouch driver");
MODULE_LICENSE("GPL");
