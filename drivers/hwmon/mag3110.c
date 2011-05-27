/*
 *
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
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

#define MAG3110_DRV_NAME       "mag3110"
#define MAG3110_ID		0xC4
#define MAG3110_XYZ_DATA_LEN	6

#define MAG3110_AC_MASK         (0x01)
#define MAG3110_AC_OFFSET       0
#define MAG3110_DR_MODE_MASK    (0x7 << 5)
#define MAG3110_DR_MODE_OFFSET  5

#define POLL_INTERVAL_MAX	500
#define POLL_INTERVAL		100
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

struct mag3110_data {
	struct i2c_client *client;
	struct input_polled_dev *poll_dev;
	struct device *hwmon_dev;
	wait_queue_head_t waitq;
	bool data_ready;
	u8 ctl_reg1;
};

static struct mag3110_data *mag3110_pdata;

/*!
 * This function do one mag3110 register read.
 */
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
	val |= (0x3 << MAG3110_DR_MODE_OFFSET);
	ret = mag3110_write_reg(client, MAG3110_CTRL_REG1, val);

	return ret;
}

/***************************************************************
*
* read sensor data from mag3110
*
***************************************************************/
static int mag3110_read_data(short *x, short *y, short *z)
{
	struct mag3110_data *data;
	u8 tmp_data[MAG3110_XYZ_DATA_LEN];

	if (!mag3110_pdata)
		return -EINVAL;

	data = mag3110_pdata;
	if (!wait_event_interruptible_timeout
	    (data->waitq, data->data_ready != 0,
	     msecs_to_jiffies(INT_TIMEOUT))) {
		dev_dbg(&data->client->dev, "interrupt not received\n");
		return -ETIME;
	}

	/* Clear data_ready flag after data is read out */
	data->data_ready = 0;

	if (mag3110_read_block_data(data->client,
			MAG3110_OUT_X_MSB, MAG3110_XYZ_DATA_LEN, tmp_data) < 0)
		return -1;

	*x = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];
	*y = ((tmp_data[2] << 8) & 0xff00) | tmp_data[3];
	*z = ((tmp_data[4] << 8) & 0xff00) | tmp_data[5];

	return 0;
}

static void report_abs(void)
{
	struct input_dev *idev;
	short x, y, z;

	if (mag3110_read_data(&x, &y, &z) != 0)
		return;

	idev = mag3110_pdata->poll_dev->input;
	input_report_abs(idev, ABS_X, x);
	input_report_abs(idev, ABS_Y, y);
	input_report_abs(idev, ABS_Z, z);
	input_sync(idev);
}

static void mag3110_dev_poll(struct input_polled_dev *dev)
{
	report_abs();
}

static irqreturn_t mag3110_irq_handler(int irq, void *dev_id)
{
	mag3110_pdata->data_ready = 1;
	wake_up_interruptible(&mag3110_pdata->waitq);

	return IRQ_HANDLED;
}

static ssize_t mag3110_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val;

	val = mag3110_read_reg(client, MAG3110_CTRL_REG1) & MAG3110_AC_MASK;

	return sprintf(buf, "%d\n", val);
}

static ssize_t mag3110_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client;
	int reg, ret, enable;
	unsigned long val;
	u8 tmp_data[MAG3110_XYZ_DATA_LEN];

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	enable = (val == 1) ? 1 : 0;

	client = to_i2c_client(dev);
	reg = mag3110_read_reg(client, MAG3110_CTRL_REG1);
	if (enable)
		reg |= MAG3110_AC_MASK;
	else
		reg &= ~MAG3110_AC_MASK;

	/* MAG3110_CTRL_REG1 bit 0, 0: STANDBY mode; 1: ACTIVE mode */
	ret = mag3110_write_reg(client, MAG3110_CTRL_REG1, reg);
	if (ret < 0)
		return ret;

	if (enable) {
		msleep(100);
		/* Read out MSB data to clear interrupt flag automatically */
		 mag3110_read_block_data(client, MAG3110_OUT_X_MSB,
					 MAG3110_XYZ_DATA_LEN, tmp_data);
	}

	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   mag3110_enable_show, mag3110_enable_store);

static ssize_t mag3110_dr_mode_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val;

	val = (mag3110_read_reg(client, MAG3110_CTRL_REG1)
		    & MAG3110_DR_MODE_MASK) >> MAG3110_DR_MODE_OFFSET;

	return sprintf(buf, "%d\n", val);
}

static ssize_t mag3110_dr_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client;
	int reg, ret;
	unsigned long val;

	/* This must be done when mag3110 is disabled */
	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 7))
		return -EINVAL;

	client = to_i2c_client(dev);

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

static struct attribute *mag3110_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_dr_mode.attr,
	NULL
};

static const struct attribute_group mag3110_attr_group = {
	.attrs = mag3110_attributes,
};

static int __devinit mag3110_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter;
	struct input_dev *idev;
	struct mag3110_data *data;
	int ret = 0;

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

	data = kzalloc(sizeof(struct mag3110_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->client = client;
	i2c_set_clientdata(client, data);

	/*create device group in sysfs as user interface */
	ret = sysfs_create_group(&client->dev.kobj, &mag3110_attr_group);
	if (ret) {
		dev_err(&client->dev, "create device file failed!\n");
		ret = -EINVAL;
		goto error_kfree;
	}

	/* Init queue */
	init_waitqueue_head(&data->waitq);

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		dev_err(&client->dev, "hwmon register failed!\n");
		ret = PTR_ERR(data->hwmon_dev);
		goto error_rm_dev_sysfs;
	}

	/*input poll device register */
	data->poll_dev = input_allocate_polled_device();
	if (!data->poll_dev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		ret = -ENOMEM;
		goto error_rm_hwmon_dev;
	}
	data->poll_dev->poll = mag3110_dev_poll;
	data->poll_dev->poll_interval = POLL_INTERVAL;
	data->poll_dev->poll_interval_max = POLL_INTERVAL_MAX;
	idev = data->poll_dev->input;
	idev->name = MAG3110_DRV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(idev, ABS_X, -15000, 15000, 0, 0);
	input_set_abs_params(idev, ABS_Y, -15000, 15000, 0, 0);
	input_set_abs_params(idev, ABS_Z, -15000, 15000, 0, 0);
	ret = input_register_polled_device(data->poll_dev);
	if (ret) {
		dev_err(&client->dev, "register poll device failed!\n");
		goto error_free_poll_dev;
	}

	/* set irq type to edge rising */
	ret = request_irq(client->irq, mag3110_irq_handler,
			  IRQF_TRIGGER_RISING, client->dev.driver->name, idev);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register irq %d!\n",
			client->irq);
		goto error_rm_poll_dev;
	}

	/* Initialize mag3110 chip */
	mag3110_init_client(client);
	mag3110_pdata = data;
	dev_info(&client->dev, "mag3110 is probed\n");
	return 0;

error_rm_poll_dev:
	input_unregister_polled_device(data->poll_dev);
error_free_poll_dev:
	input_free_polled_device(data->poll_dev);
error_rm_hwmon_dev:
	hwmon_device_unregister(data->hwmon_dev);
error_rm_dev_sysfs:
	sysfs_remove_group(&client->dev.kobj, &mag3110_attr_group);
error_kfree:
	kfree(data);
	mag3110_pdata = NULL;

	return ret;
}

static int __devexit mag3110_remove(struct i2c_client *client)
{
	struct mag3110_data *data;
	int ret;

	data = i2c_get_clientdata(client);

	data->ctl_reg1 = mag3110_read_reg(client, MAG3110_CTRL_REG1);
	ret = mag3110_write_reg(client, MAG3110_CTRL_REG1,
				    data->ctl_reg1 & ~MAG3110_AC_MASK);

	free_irq(client->irq, data);
	input_unregister_polled_device(data->poll_dev);
	input_free_polled_device(data->poll_dev);
	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &mag3110_attr_group);
	kfree(data);
	mag3110_pdata = NULL;

	return ret;
}

#ifdef CONFIG_PM
static int mag3110_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct mag3110_data *data = i2c_get_clientdata(client);

	data->ctl_reg1 = mag3110_read_reg(client, MAG3110_CTRL_REG1);
	ret = mag3110_write_reg(client, MAG3110_CTRL_REG1,
				   data->ctl_reg1 & ~MAG3110_AC_MASK);
	return ret;
}

static int mag3110_resume(struct i2c_client *client)
{
	int ret;
	u8 tmp_data[MAG3110_XYZ_DATA_LEN];
	struct mag3110_data *data = i2c_get_clientdata(client);

	ret = mag3110_write_reg(client, MAG3110_CTRL_REG1,
				   data->ctl_reg1);

	if (data->ctl_reg1 & MAG3110_AC_MASK) {
		/* Read out MSB data to clear interrupt flag automatically */
		mag3110_read_block_data(client, MAG3110_OUT_X_MSB,
					MAG3110_XYZ_DATA_LEN, tmp_data);
	}

	return ret;
}

#else
#define mag3110_suspend        NULL
#define mag3110_resume         NULL
#endif				/* CONFIG_PM */

static const struct i2c_device_id mag3110_id[] = {
	{MAG3110_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mag3110_id);
static struct i2c_driver mag3110_driver = {
	.driver = {.name = MAG3110_DRV_NAME,
		   .owner = THIS_MODULE,},
	.suspend = mag3110_suspend,
	.resume = mag3110_resume,
	.probe = mag3110_probe,
	.remove = __devexit_p(mag3110_remove),
	.id_table = mag3110_id,
};

static int __init mag3110_init(void)
{
	return i2c_add_driver(&mag3110_driver);
}

static void __exit mag3110_exit(void)
{
	i2c_del_driver(&mag3110_driver);
}

module_init(mag3110_init);
module_exit(mag3110_exit);
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale mag3110 3-axis magnetometer driver");
MODULE_LICENSE("GPL");
