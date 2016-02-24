/*
 *
 * Copyright (C) 2011-2016 Freescale Semiconductor, Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/input-polldev.h>
#include <linux/hwmon.h>
#include <linux/input.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#define ABS_STATUS ABS_WHEEL

#define MAG3110_DRV_NAME       "mag3110"
#define MAG3110_ID              0xC4
#define MAG3110_XYZ_DATA_LEN    6
#define MAG3110_STATUS_ZYXDR    0x08

#define MAG3110_AC_MASK         (0x01)
#define MAG3110_AC_OFFSET       0
#define MAG3110_DR_MODE_MASK    (0x7 << 5)
#define MAG3110_DR_MODE_OFFSET  5
#define MAG3110_IRQ_USED        1

#define POLL_INTERVAL_MAX       500
#define POLL_INTERVAL_MIN       1
#define POLL_INTERVAL           100
/* if sensor is standby ,set POLL_STOP_TIME to slow down the poll */
#define POLL_STOP_TIME          200

#define INT_TIMEOUT   1000
/* register enum for mag3110 registers */
enum {
	MAG3110_DR_STATUS = 0x00,
	MAG3110_OUT_X_MSB,
	MAG3110_OUT_X_LSB,
	MAG3110_OUT_Y_MSB,
	MAG3110_OUT_Y_LSB,
	MAG3110_OUT_Z_MSB,
	MAG3110_OUT_Z_LSB,
	MAG3110_WHO_AM_I,

	MAG3110_OFF_X_MSB,
	MAG3110_OFF_X_LSB,
	MAG3110_OFF_Y_MSB,
	MAG3110_OFF_Y_LSB,
	MAG3110_OFF_Z_MSB,
	MAG3110_OFF_Z_LSB,

	MAG3110_DIE_TEMP,

	MAG3110_CTRL_REG1 = 0x10,
	MAG3110_CTRL_REG2,
};
enum {
	MAG_STANDBY,
	MAG_ACTIVED
};
struct mag3110_data_axis {
	short x;
	short y;
	short z;
};

struct mag3110_data {
	struct i2c_client *client;
	struct input_polled_dev *poll_dev;
    /* mag3110 calibrated data report input dev */
	struct input_dev *cal_input;
	struct mutex data_lock;
	wait_queue_head_t waitq;
	bool data_ready;
	u8 ctl_reg1;
	int active;
	int position;
};
static short mag3110_position_setting[8][3][3] = {
	{ { 0,	-1, 0  }, { 1,	0,  0  }, { 0, 0, -1 } },
	{ { -1, 0,  0  }, { 0,	-1, 0  }, { 0, 0, -1 } },
	{ { 0,	1,  0  }, { -1, 0,  0  }, { 0, 0, -1 } },
	{ { 1,	0,  0  }, { 0,	1,  0  }, { 0, 0, -1 } },

	{ { 0,	-1, 0  }, { -1, 0,  0  }, { 0, 0, 1  } },
	{ { -1, 0,  0  }, { 0,	1,  0  }, { 0, 0, 1  } },
	{ { 0,	1,  0  }, { 1,	0,  0  }, { 0, 0, 1  } },
	{ { 1,	0,  0  }, { 0,	-1, 0  }, { 0, 0, 1  } },

};

/*!
 * This function do one mag3110 register read.
 */
static int mag3110_data_convert(int position,
	   struct mag3110_data_axis *axis_data)
{
	short rawdata[3], data[3];
	int i, j;

	if (position < 0 || position > 7)
		position = 0;
	rawdata[0] = axis_data->x;
	rawdata[1] = axis_data->y;
	rawdata[2] = axis_data->z;
	for (i = 0; i < 3; i++) {
		data[i] = 0;
		for (j = 0; j < 3; j++)
			data[i] += rawdata[j] * mag3110_position_setting[position][i][j];
	}
	axis_data->x = data[0];
	axis_data->y = data[1];
	axis_data->z = data[2];
	return 0;
}
static int mag3110_read_reg(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

/*!
 * This function do one mag3110 register write.
 */
static int mag3110_write_reg(struct i2c_client *client, u8 reg, char value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);
	if (ret < 0)
		dev_err(&client->dev, "i2c write failed\n");
	return ret;
}

/*!
 * This function do multiple mag3110 registers read.
 */
static int mag3110_read_block_data(struct i2c_client *client, u8 reg,
				   int count, u8 *addr)
{
	if (i2c_smbus_read_i2c_block_data
		    (client, reg, count, addr) < count) {
		dev_err(&client->dev, "i2c block read failed\n");
		return -1;
	}

	return count;
}

/*
 * Initialization function
 */
static int mag3110_init_client(struct i2c_client *client)
{
	int val, ret;

	/* enable automatic resets */
	val = 0x80;
	ret = mag3110_write_reg(client, MAG3110_CTRL_REG2, val);

	/* set default data rate to 10HZ */
	val = mag3110_read_reg(client, MAG3110_CTRL_REG1);
	val |= (0x0 << MAG3110_DR_MODE_OFFSET);
	ret = mag3110_write_reg(client, MAG3110_CTRL_REG1, val);

	return ret;
}

/***************************************************************
*
* read sensor data from mag3110
*
***************************************************************/
static int mag3110_read_data(struct i2c_client *client,
		struct mag3110_data_axis *pdata)
{
	struct mag3110_data *mag = i2c_get_clientdata(client);
	u8 tmp_data[MAG3110_XYZ_DATA_LEN];

	if (!mag || mag->active == MAG_STANDBY)
		return -EINVAL;
#if MAG3110_IRQ_USED
	if (!wait_event_interruptible_timeout
		    (mag->waitq, mag->data_ready != 0,
		    msecs_to_jiffies(INT_TIMEOUT))) {
		dev_dbg(&mag->client->dev, "interrupt not received\n");
		return -ETIME;
	}
#endif
	mag->data_ready = 0;
	if (mag3110_read_block_data(client,
				    MAG3110_OUT_X_MSB,
					MAG3110_XYZ_DATA_LEN, tmp_data) < 0)
		return -1;
	pdata->x = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];
	pdata->y = ((tmp_data[2] << 8) & 0xff00) | tmp_data[3];
	pdata->z = ((tmp_data[4] << 8) & 0xff00) | tmp_data[5];

	return 0;
}

static void mag3110_report_data(struct mag3110_data *mag)
{

	struct input_polled_dev *poll_dev = mag->poll_dev;
	struct input_dev *idev = poll_dev->input;
	struct mag3110_data_axis data;

	mutex_lock(&mag->data_lock);
	if (mag3110_read_data(mag->client, &data) != 0) {
		poll_dev->poll_interval = POLL_STOP_TIME;
		goto out;
	} else if (poll_dev->poll_interval == POLL_STOP_TIME)
		poll_dev->poll_interval = POLL_INTERVAL;
	mag3110_data_convert(mag->position, &data);
	idev = mag->poll_dev->input;
	input_report_abs(idev, ABS_X, data.x);
	input_report_abs(idev, ABS_Y, data.y);
	input_report_abs(idev, ABS_Z, data.z);
	input_sync(idev);
out:
	mutex_unlock(&mag->data_lock);
}

static void mag3110_dev_poll(struct input_polled_dev *dev)
{
	struct mag3110_data * mag = (struct mag3110_data *)dev->private;

	mag3110_report_data(mag);
}

#if MAG3110_IRQ_USED
static irqreturn_t mag3110_irq_handler(int irq, void *dev)
{
	struct mag3110_data *mag = (struct mag3110_data *)dev;

	mag->data_ready = 1;
	wake_up_interruptible(&mag->waitq);

	return IRQ_HANDLED;
}
#endif
static int mag3110_reigister_caldata_input(struct mag3110_data *mag)
{
	struct input_dev *idev;
	struct i2c_client *client = mag->client;
	int ret;

	idev = input_allocate_device();
	if (!idev) {
		dev_err(&client->dev, "alloc calibrated data device error\n");
		return -EINVAL;
	}
	idev->name = "eCompass";
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);
	input_set_abs_params(idev, ABS_X, -15000, 15000, 0, 0);
	input_set_abs_params(idev, ABS_Y, -15000, 15000, 0, 0);
	input_set_abs_params(idev, ABS_Z, -15000, 15000, 0, 0);
	input_set_abs_params(idev, ABS_RX,         0, 36000, 0, 0);
	input_set_abs_params(idev, ABS_RY, -18000, 18000, 0, 0);
	input_set_abs_params(idev, ABS_RZ, -9000,  9000, 0, 0);
	input_set_abs_params(idev, ABS_STATUS, 0,     3, 0, 0);
	ret = input_register_device(idev);
	if (ret) {
		dev_err(&client->dev, "register poll device failed!\n");
		return -EINVAL;
	}
	mag->cal_input = idev;
	return 0;
}
static int mag3110_unreigister_caldata_input(struct mag3110_data *mag)
{
	struct input_dev *idev = mag->cal_input;

	if (idev) {
		input_unregister_device(idev);
		input_free_device(idev);
	}
	mag->cal_input = NULL;
	return 0;
}
static ssize_t mag3110_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct mag3110_data *mag = (struct mag3110_data *)(poll_dev->private);
	struct i2c_client *client = mag->client;
	int val;

	mutex_lock(&mag->data_lock);
	val = mag3110_read_reg(client, MAG3110_CTRL_REG1);
	val &= MAG3110_AC_MASK;
	mutex_unlock(&mag->data_lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t mag3110_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct mag3110_data *mag = (struct mag3110_data *)(poll_dev->private);
	struct i2c_client *client = mag->client;
	int reg, ret, enable;
	u8 tmp_data[MAG3110_XYZ_DATA_LEN];

	enable = simple_strtoul(buf, NULL, 10);
	mutex_lock(&mag->data_lock);
	client = mag->client;
	reg = mag3110_read_reg(client, MAG3110_CTRL_REG1);
	if (enable &&  mag->active == MAG_STANDBY) {
		reg |= MAG3110_AC_MASK;
		ret = mag3110_write_reg(client, MAG3110_CTRL_REG1, reg);
		if (!ret)
			mag->active = MAG_ACTIVED;
		printk(KERN_INFO"mag3110 set active\n");
	} else if (!enable && mag->active == MAG_ACTIVED) {
		reg &= ~MAG3110_AC_MASK;
		ret = mag3110_write_reg(client, MAG3110_CTRL_REG1, reg);
		if (!ret)
			mag->active = MAG_STANDBY;
		printk(KERN_INFO"mag3110 set inactive\n");

	}

	if (mag->active == MAG_ACTIVED) {
		msleep(100);
		/* Read out MSB data to clear interrupt flag automatically */
		mag3110_read_block_data(client, MAG3110_OUT_X_MSB,
					MAG3110_XYZ_DATA_LEN, tmp_data);
	}
	mutex_unlock(&mag->data_lock);
	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   mag3110_enable_show, mag3110_enable_store);

static ssize_t mag3110_dr_mode_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct mag3110_data *mag = (struct mag3110_data *)(poll_dev->private);
	struct i2c_client *client = mag->client;
	int val;

	client = mag->client;
	val = (mag3110_read_reg(client, MAG3110_CTRL_REG1)
	       & MAG3110_DR_MODE_MASK) >> MAG3110_DR_MODE_OFFSET;

	return sprintf(buf, "%d\n", val);
}

static ssize_t mag3110_dr_mode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct mag3110_data *mag = (struct mag3110_data *)(poll_dev->private);
	struct i2c_client *client = mag->client;
	int reg, ret;
	unsigned long val;

	/* This must be done when mag3110 is disabled */
	if ((kstrtoul(buf, 10, &val) < 0) || (val > 7))
		return -EINVAL;

	client = mag->client;
	reg = mag3110_read_reg(client, MAG3110_CTRL_REG1) &
	      ~MAG3110_DR_MODE_MASK;
	reg |= (val << MAG3110_DR_MODE_OFFSET);
	/* MAG3110_CTRL_REG1 bit 5-7: data rate mode */
	ret = mag3110_write_reg(client, MAG3110_CTRL_REG1, reg);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(dr_mode, S_IWUSR | S_IRUGO,
		   mag3110_dr_mode_show, mag3110_dr_mode_store);

static ssize_t mag3110_position_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct mag3110_data *mag = (struct mag3110_data *)(poll_dev->private);
	int val;

	mutex_lock(&mag->data_lock);
	val = mag->position;
	mutex_unlock(&mag->data_lock);
	return sprintf(buf, "%d\n", val);
}
static ssize_t mag3110_position_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct input_polled_dev *poll_dev = dev_get_drvdata(dev);
	struct mag3110_data *mag = (struct mag3110_data *)(poll_dev->private);
	int position;

	position = simple_strtoul(buf, NULL, 10);
	mutex_lock(&mag->data_lock);
	mag->position = position;
	mutex_unlock(&mag->data_lock);
	return count;
}
static DEVICE_ATTR(position, S_IWUSR | S_IRUGO,
		   mag3110_position_show, mag3110_position_store);

static struct attribute *mag3110_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_dr_mode.attr,
	&dev_attr_position.attr,
	NULL
};

static const struct attribute_group mag3110_attr_group = {
	.attrs	= mag3110_attributes,
};

static int mag3110_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter;
	struct input_dev *idev;
	struct mag3110_data *mag;
	struct regulator *vdd, *vdd_io;
	u32 irq_flag;
	struct irq_data *irq_data;
	struct device_node *of_node = client->dev.of_node;
	bool shared_irq;
	int ret = 0;
	u32 pos;
	vdd = devm_regulator_get(&client->dev, "vdd");
	if (!IS_ERR(vdd)) {
		ret = regulator_enable(vdd);
		if (ret) {
			dev_err(&client->dev, "vdd set voltage error\n");
			return ret;
		}
	}

	vdd_io = devm_regulator_get(&client->dev, "vddio");
	if (!IS_ERR(vdd_io)) {
		ret = regulator_enable(vdd_io);
		if (ret) {
			dev_err(&client->dev, "vddio set voltage error\n");
			return ret;
		}
	}
	adapter = to_i2c_adapter(client->dev.parent);
	if (!i2c_check_functionality(adapter,
				     I2C_FUNC_SMBUS_BYTE |
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	dev_info(&client->dev, "check mag3110 chip ID\n");
	ret = mag3110_read_reg(client, MAG3110_WHO_AM_I);

	if (MAG3110_ID != ret) {
		dev_err(&client->dev,
			"read chip ID 0x%x is not equal to 0x%x!\n", ret,
			MAG3110_ID);
		return -EINVAL;
	}
	mag = kzalloc(sizeof(struct mag3110_data), GFP_KERNEL);
	if (!mag)
		return -ENOMEM;
	mag->client = client;
	mag->active = MAG_STANDBY;
	ret = of_property_read_u32(of_node, "position", &pos);
	if (ret)
		pos = 2;
	mag->position = (int)pos;
	mutex_init(&mag->data_lock);
	i2c_set_clientdata(client, mag);
	/* Init queue */
	init_waitqueue_head(&mag->waitq);

	/*input poll device register */
	mag->poll_dev = input_allocate_polled_device();
	if (!mag->poll_dev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		ret = -ENOMEM;
		goto error_alloc_poll_dev;
	}
	mag->poll_dev->poll = mag3110_dev_poll;
	mag->poll_dev->poll_interval = POLL_STOP_TIME;
	mag->poll_dev->poll_interval_max = POLL_INTERVAL_MAX;
	mag->poll_dev->poll_interval_min = POLL_INTERVAL_MIN;
	mag->poll_dev->private = mag;
	idev = mag->poll_dev->input;
	idev->name = "FreescaleMagnetometer";
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);
	input_set_abs_params(idev, ABS_X, -0x7fff, 0x7fff, 0, 0);
	input_set_abs_params(idev, ABS_Y, -0x7fff, 0x7fff, 0, 0);
	input_set_abs_params(idev, ABS_Z, -0x7fff, 0x7fff, 0, 0);
	ret = input_register_polled_device(mag->poll_dev);
	if (ret) {
		dev_err(&client->dev, "register poll device failed!\n");
		goto error_reg_poll_dev;
	}
	ret = mag3110_reigister_caldata_input(mag);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed to register calibrated input device!\n");
		goto erorr_reg_caldev;
	}
	/*create device group in sysfs as user interface */
	ret = sysfs_create_group(&idev->dev.kobj, &mag3110_attr_group);
	if (ret) {
		dev_err(&client->dev, "create sysfs device file failed!\n");
		ret = -EINVAL;
		goto error_reg_sysfs;
	}

	/* set irq type to edge rising */
#if MAG3110_IRQ_USED
	shared_irq = of_property_read_bool(of_node, "shared-interrupt");
	if (client->irq) {
		irq_data = irq_get_irq_data(client->irq);
		irq_flag = irqd_get_trigger_type(irq_data);
		irq_flag |= IRQF_ONESHOT;
		if (shared_irq)
				irq_flag |= IRQF_SHARED;
		ret = request_irq(client->irq, mag3110_irq_handler,
			irq_flag, client->dev.driver->name, mag);
		if (ret < 0) {
			dev_err(&client->dev, "failed to register irq %d!\n",
					client->irq);
			goto error_reg_irq;
	}
	}
#endif
	/* Initialize mag3110 chip */

	mag3110_init_client(client);
	dev_info(&client->dev, "mag3110 is probed\n");
	return 0;
#if MAG3110_IRQ_USED
error_reg_irq:
	sysfs_remove_group(&client->dev.kobj, &mag3110_attr_group);
#endif
error_reg_sysfs:
	mag3110_unreigister_caldata_input(mag);
erorr_reg_caldev:
	input_unregister_polled_device(mag->poll_dev);
error_reg_poll_dev:
	input_free_polled_device(mag->poll_dev);
error_alloc_poll_dev:
	kfree(mag);
	return ret;
}

static int mag3110_remove(struct i2c_client *client)
{
	struct mag3110_data *mag = i2c_get_clientdata(client);
	int ret;

	mag->ctl_reg1 = mag3110_read_reg(client, MAG3110_CTRL_REG1);
	ret = mag3110_write_reg(client, MAG3110_CTRL_REG1,
				mag->ctl_reg1 & ~MAG3110_AC_MASK);
	free_irq(client->irq, mag);
	input_unregister_polled_device(mag->poll_dev);
	input_free_polled_device(mag->poll_dev);
	sysfs_remove_group(&client->dev.kobj, &mag3110_attr_group);
	mag3110_unreigister_caldata_input(mag);
	kfree(mag);
	mag = NULL;

	return ret;
}

#ifdef CONFIG_PM
static int mag3110_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mag3110_data *mag = i2c_get_clientdata(client);
	int ret = 0;

	if (mag->active == MAG_ACTIVED) {
		mag->ctl_reg1 = mag3110_read_reg(client, MAG3110_CTRL_REG1);
		ret = mag3110_write_reg(client, MAG3110_CTRL_REG1,
					mag->ctl_reg1 & ~MAG3110_AC_MASK);
	}
	return ret;
}

static int mag3110_resume(struct device *dev)
{
	int ret = 0;
	u8 tmp_data[MAG3110_XYZ_DATA_LEN];
	struct i2c_client *client = to_i2c_client(dev);
	struct mag3110_data *mag = i2c_get_clientdata(client);

	if (mag->active == MAG_ACTIVED) {
		ret = mag3110_write_reg(client, MAG3110_CTRL_REG1,
					mag->ctl_reg1);

		if (mag->ctl_reg1 & MAG3110_AC_MASK) {
			/* Read out MSB data to clear */
			/* interrupt flag automatically */
			mag3110_read_block_data(client, MAG3110_OUT_X_MSB,
						MAG3110_XYZ_DATA_LEN, tmp_data);
		}
	}
	return ret;
}

#else
#define mag3110_suspend        NULL
#define mag3110_resume         NULL
#endif                          /* CONFIG_PM */

static const struct i2c_device_id mag3110_id[] = {
	{ MAG3110_DRV_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, mag3110_id);

static SIMPLE_DEV_PM_OPS(mag3110_pm_ops, mag3110_suspend, mag3110_resume);
static struct i2c_driver mag3110_driver = {
	.driver		= {
		.name	= MAG3110_DRV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &mag3110_pm_ops,
	},
	.probe			= mag3110_probe,
	.remove			= mag3110_remove,
	.id_table		= mag3110_id,
};


module_i2c_driver(mag3110_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale mag3110 3-axis magnetometer driver");
MODULE_LICENSE("GPL");
