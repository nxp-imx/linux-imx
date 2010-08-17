/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file drivers/hwmon/isl29003.c
 *
 * @brief ISL29003 light sensor Driver
 *
 * @ingroup
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>

enum isl29003_width {
	ISL29003_WIDTH_16 = 0,
	ISL29003_WIDTH_12,
	ISL29003_WIDTH_8,
	ISL29003_WIDTH_4,
};

enum isl29003_gain {
	ISL29003_GAIN_1000 = 0,
	ISL29003_GAIN_4000,
	ISL29003_GAIN_16000,
	ISL29003_GAIN_64000,
};

enum isl29003_mode {
	ISL29003_MODE_DIODE1 = 0,
	ISL29003_MODE_DIODE2,
	ISL29003_MODE_DIODE1_2,
};

struct isl29003_param {
	enum isl29003_width width;
	enum isl29003_gain gain;
	enum isl29003_mode mode;
};

/* bit definition for ISL29003_CMD reg */
#define ENABLE 7
#define ADCPD 6
#define TIMEING_MODE 5
#define MODE 2
#define WIDTH 0

/* bit definition for ISL29003_CTRL reg */
#define INT_FLAG 5
#define GAIN 2
#define INT_PERSIST 0

enum isl29003_reg {
	ISL29003_CMD = 0,
	ISL29003_CTRL,
	ISL29003_THRS_HI,
	ISL29003_THRS_LO,
	ISL29003_LSB_S,
	ISL29003_MSB_S,
	ISL29003_LSB_T,
	ISL29003_MSB_T,
	ISL29003_SYNC_IIC = 0x80,
	ISL29003_CLAR_INT = 0x40
};

/* default configure for ISL29003 */
#define ISL29003_WIDTH_DEFAULT ISL29003_WIDTH_16
#define ISL29003_GAIN_DEFAULT ISL29003_GAIN_16000
#define ISL29003_MODE_DEFAULT ISL29003_MODE_DIODE1

/* range table for different GAIN settings */
int range[4] = { 973, 3892, 15568, 62272 };

/* width table for different WIDTH settings */
int width[4] = { 16, 1, 256, 16 };

struct isl29003_data {
	struct i2c_client *client;
	struct device *hwmon_dev;
	struct regulator *vdd_reg;
	struct isl29003_param param;
	int lux_coeff;
	unsigned char enable;
};

static struct i2c_client *isl29003_client;

/*!
 * This function do the isl29003 register read.
 */
int isl29003_read(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

/*!
 * This function do the isl29003 register write.
 */
int isl29003_write(struct i2c_client *client, u8 reg, char value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

/*!
 * This function do the isl29003 config and enable.
 */
static int isl29003_on(void)
{
	unsigned char cmd;
	int err = 0;
	struct mxc_lightsensor_platform_data *ls_data;
	struct isl29003_data *data = i2c_get_clientdata(isl29003_client);

	if (data->enable)
		goto exit;

	ls_data = (struct mxc_lightsensor_platform_data *)
	    (isl29003_client->dev).platform_data;

	/* coeff=range*100k/rext/2^n */
	data->lux_coeff = range[data->param.gain] * 100 /
	    ls_data->rext / width[data->param.width];

	if (data->vdd_reg)
		regulator_enable(data->vdd_reg);
	msleep(100);

	cmd = data->param.gain << GAIN;
	if (isl29003_write(isl29003_client, ISL29003_CTRL, cmd)) {
		err = -ENODEV;
		goto exit;
	}

	cmd = (data->param.width << WIDTH) | (data->param.mode << MODE) |
	    (1 << ENABLE);
	if (isl29003_write(isl29003_client, ISL29003_CMD, cmd)) {
		err = -ENODEV;
		goto exit;
	}

	data->enable = 1;

	pr_info("isl29003 on\n");
	return 0;
exit:
	return err;
}

/*!
 * This function shut down the isl29003.
 */
static int isl29003_off(void)
{
	struct isl29003_data *data = i2c_get_clientdata(isl29003_client);
	int cmd;

	if (!data->enable)
		return 0;

	cmd = isl29003_read(isl29003_client, ISL29003_CMD);
	if (cmd < 0)
		return -ENODEV;

	cmd = ((cmd | (1 << ADCPD)) & (~(1 << ENABLE)));
	if (isl29003_write(isl29003_client, ISL29003_CMD, (char)cmd))
		return -ENODEV;

	if (data->vdd_reg)
		regulator_disable(data->vdd_reg);

	data->enable = 0;

	pr_info("isl29003 off\n");
	return 0;
}

/*!
 * This function read the isl29003 lux registers and convert them to the lux
 * value.
 *
 * @output buffer	this param holds the lux value, when =-1, read fail
 *
 * @return 0
 */
static int isl29003_read_lux(void)
{
	int d;
	int lux;
	struct isl29003_data *data = i2c_get_clientdata(isl29003_client);

	d = isl29003_read(isl29003_client, ISL29003_MSB_S);
	if (d < 0)
		goto err;

	lux = d;
	d = isl29003_read(isl29003_client, ISL29003_LSB_S);
	if (d < 0)
		goto err;

	lux = (lux << 8) + d;

	if (data->param.width < ISL29003_WIDTH_8)
		lux = (data->lux_coeff * lux) >> 12;
	else
		lux = data->lux_coeff * lux;

	return lux;
err:
	return -1;
}

static ssize_t ls_enable(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	char *endp;
	int enable = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if (enable == 1) {
		if (isl29003_on())
			pr_info("device open fail\n");
	}
	if (enable == 0) {
		if (isl29003_off())
			pr_info("device powerdown fail\n");
	}

	return count;
}

static SENSOR_DEVICE_ATTR(enable, S_IWUGO, NULL, ls_enable, 0);

static ssize_t show_lux(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", isl29003_read_lux());
}

static SENSOR_DEVICE_ATTR(lux, S_IRUGO, show_lux, NULL, 0);

static int isl29003_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *did)
{
	int err = 0;
	struct isl29003_data *data;
	struct regulator *vdd_reg;
	struct mxc_lightsensor_platform_data *ls_data;

	ls_data = (struct mxc_lightsensor_platform_data *)
	    (client->dev).platform_data;

	if (ls_data && ls_data->vdd_reg)
		vdd_reg = regulator_get(&client->dev, ls_data->vdd_reg);
	else
		vdd_reg = NULL;

	/* check the existence of the device */
	if (vdd_reg)
		regulator_enable(vdd_reg);
	msleep(100);

	if (isl29003_write(client, ISL29003_CMD, 0))
		err = -ENODEV;

	if (!err)
		if (isl29003_read(client, ISL29003_CMD))
			err = -ENODEV;

	if (vdd_reg)
		regulator_disable(vdd_reg);
	if (err < 0)
		goto exit1;

	isl29003_client = client;
	data = kzalloc(sizeof(struct isl29003_data), GFP_KERNEL);
	if (data == NULL) {
		err = -ENOMEM;
		goto exit1;
	}

	i2c_set_clientdata(client, data);
	data->client = client;

	data->param.width = ISL29003_WIDTH_DEFAULT;
	data->param.gain = ISL29003_GAIN_DEFAULT;
	data->param.mode = ISL29003_MODE_DEFAULT;

	data->enable = 0;

	err = device_create_file(&client->dev,
				 &sensor_dev_attr_enable.dev_attr);
	if (err)
		goto exit2;

	err = device_create_file(&client->dev, &sensor_dev_attr_lux.dev_attr);
	if (err)
		goto exit_remove1;

	/* Register sysfs hooks */
	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove2;
	}

	data->vdd_reg = vdd_reg;
	return 0;

exit_remove2:
	device_remove_file(&client->dev, &sensor_dev_attr_lux.dev_attr);
exit_remove1:
	device_remove_file(&client->dev, &sensor_dev_attr_enable.dev_attr);
exit2:
	kfree(data);
exit1:
	if (vdd_reg) {
		regulator_put(vdd_reg);
		vdd_reg = NULL;
	}
	isl29003_client = NULL;
	return err;
}

static int isl29003_i2c_remove(struct i2c_client *client)
{
	struct isl29003_data *data = i2c_get_clientdata(client);

	if (data->vdd_reg) {
		regulator_put(data->vdd_reg);
		data->vdd_reg = NULL;
	}
	hwmon_device_unregister(data->hwmon_dev);
	device_remove_file(&client->dev, &sensor_dev_attr_enable.dev_attr);
	device_remove_file(&client->dev, &sensor_dev_attr_lux.dev_attr);
	kfree(data);
	return 0;
}

static int isl29003_suspend(struct i2c_client *client, pm_message_t message)
{
	int cmd;

	struct isl29003_data *data = i2c_get_clientdata(client);

	if (!data->enable)
		goto exit;

	cmd = isl29003_read(client, ISL29003_CMD);
	if (cmd < 0)
		goto err;

	cmd = (cmd | (1 << ADCPD));
	if (isl29003_write(client, ISL29003_CMD, (char)cmd))
		goto err;
exit:
	return 0;
err:
	return -ENODEV;
}

static int isl29003_resume(struct i2c_client *client)
{
	int cmd;

	struct isl29003_data *data = i2c_get_clientdata(client);

	if (!data->enable)
		goto exit;

	cmd = isl29003_read(client, ISL29003_CMD);
	if (cmd < 0)
		goto err;

	cmd = (cmd & (~(1 << ADCPD)));
	if (isl29003_write(client, ISL29003_CMD, (char)cmd))
		goto err;
exit:
	return 0;
err:
	return -ENODEV;
}

static const struct i2c_device_id isl29003_id[] = {
	{"isl29003", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, isl29003_id);

static struct i2c_driver isl29003_driver = {
	.driver = {
		   .name = "isl29003",
		   },
	.probe = isl29003_i2c_probe,
	.remove = isl29003_i2c_remove,
	.suspend = isl29003_suspend,
	.resume = isl29003_resume,
	.id_table = isl29003_id,
};

static int __init isl29003_init(void)
{
	return i2c_add_driver(&isl29003_driver);;
}

static void __exit isl29003_cleanup(void)
{
	i2c_del_driver(&isl29003_driver);
}

module_init(isl29003_init);
module_exit(isl29003_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ISL29003 light sensor driver");
MODULE_LICENSE("GPL");
