/*
 *  mma8450.c - Linux kernel modules for 3-Axis Orientation/Motion
 *  Detection Sensor
 *
 *  Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>

/*
 * Defines
 */
#define assert(expr)\
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}

#define MMA8450_DRV_NAME	"mma8450"
#define MMA8450_I2C_ADDR	0x1C
#define MMA8450_ID		0xC6
#define MMA8450_STATUS 0x00

#define MODE_CHANGE_DELAY_MS 100
#define POLL_INTERVAL_MAX	500
#define POLL_INTERVAL		100
#define INPUT_FUZZ	32
#define INPUT_FLAT	32

/* register enum for mma8450 registers */
enum {
	MMA8450_STATUS1 = 0x00,
	MMA8450_OUT_X8,
	MMA8450_OUT_Y8,
	MMA8450_OUT_Z8,

	MMA8450_STATUS2,
	MMA8450_OUT_X_LSB,
	MMA8450_OUT_X_MSB,
	MMA8450_OUT_Y_LSB,
	MMA8450_OUT_Y_MSB,
	MMA8450_OUT_Z_LSB,
	MMA8450_OUT_Z_MSB,

	MMA8450_STATUS3,
	MMA8450_OUT_X_DELTA,
	MMA8450_OUT_Y_DELTA,
	MMA8450_OUT_Z_DELTA,

	MMA8450_WHO_AM_I,

	MMA8450_F_STATUS,
	MMA8450_F_8DATA,
	MMA8450_F_12DATA,
	MMA8450_F_SETUP,

	MMA8450_SYSMOD,
	MMA8450_INT_SOURCE,
	MMA8450_XYZ_DATA_CFG,
	MMA8450_HP_FILTER_CUTOFF,

	MMA8450_PL_STATUS,
	MMA8450_PL_PRE_STATUS,
	MMA8450_PL_CFG,
	MMA8450_PL_COUNT,
	MMA8450_PL_BF_ZCOMP,
	MMA8450_PL_P_L_THS_REG1,
	MMA8450_PL_P_L_THS_REG2,
	MMA8450_PL_P_L_THS_REG3,
	MMA8450_PL_L_P_THS_REG1,
	MMA8450_PL_L_P_THS_REG2,
	MMA8450_PL_L_P_THS_REG3,

	MMA8450_FF_MT_CFG_1,
	MMA8450_FF_MT_SRC_1,
	MMA8450_FF_MT_THS_1,
	MMA8450_FF_MT_COUNT_1,
	MMA8450_FF_MT_CFG_2,
	MMA8450_FF_MT_SRC_2,
	MMA8450_FF_MT_THS_2,
	MMA8450_FF_MT_COUNT_2,

	MMA8450_TRANSIENT_CFG,
	MMA8450_TRANSIENT_SRC,
	MMA8450_TRANSIENT_THS,
	MMA8450_TRANSIENT_COUNT,

	MMA8450_PULSE_CFG,
	MMA8450_PULSE_SRC,
	MMA8450_PULSE_THSX,
	MMA8450_PULSE_THSY,
	MMA8450_PULSE_THSZ,
	MMA8450_PULSE_TMLT,
	MMA8450_PULSE_LTCY,
	MMA8450_PULSE_WIND,

	MMA8450_ASLP_COUNT,
	MMA8450_CTRL_REG1,
	MMA8450_CTRL_REG2,
	MMA8450_CTRL_REG3,
	MMA8450_CTRL_REG4,
	MMA8450_CTRL_REG5,

	MMA8450_OFF_X,
	MMA8450_OFF_Y,
	MMA8450_OFF_Z,

	MMA8450_REG_END,
};

enum {
	MODE_STANDBY,
	MODE_2G,
	MODE_4G,
	MODE_8G,
};

/* mma8450 status */
struct mma8450_status {
	u8 mode;
	u8 ctl_reg2;
	u8 ctl_reg1;
};

static struct mma8450_status mma_status = {
	.mode = 0,
	.ctl_reg2 = 0,
	.ctl_reg1 = 0
};

static struct device *hwmon_dev;
static struct i2c_client *mma8450_i2c_client;
static struct input_polled_dev *mma8450_idev;

/*
 * Initialization function
 */
static int mma8450_init_client(struct i2c_client *client)
{
	int result;

	mma_status.mode = MODE_2G;

	result = i2c_smbus_write_byte_data(client, MMA8450_XYZ_DATA_CFG, 0x07);
	assert(result == 0);

	result =
	    i2c_smbus_write_byte_data(client, MMA8450_CTRL_REG1,
				      mma_status.mode);
	assert(result == 0);

	mdelay(MODE_CHANGE_DELAY_MS);

	return result;
}

/*
 * read sensor data from mma8450
 */
static int mma8450_read_data(short *x, short *y, short *z)
{
	u8 tmp_data[7];

	if (i2c_smbus_read_i2c_block_data
	    (mma8450_i2c_client, MMA8450_OUT_X_LSB, 7, tmp_data) < 7) {
		dev_err(&mma8450_i2c_client->dev, "i2c block read failed\n");
		return -3;
	}

	*x = ((tmp_data[1] << 8) & 0xff00) | ((tmp_data[0] << 4) & 0x00f0);
	*y = ((tmp_data[3] << 8) & 0xff00) | ((tmp_data[2] << 4) & 0x00f0);
	*z = ((tmp_data[5] << 8) & 0xff00) | ((tmp_data[4] << 4) & 0x00f0);

	*x = (short)(*x) >> 4;
	*y = (short)(*y) >> 4;
	*z = (short)(*z) >> 4;

	if (mma_status.mode == MODE_4G) {
		(*x) = (*x) << 1;
		(*y) = (*y) << 1;
		(*z) = (*z) << 1;
	} else if (mma_status.mode == MODE_8G) {
		(*x) = (*x) << 2;
		(*y) = (*y) << 2;
		(*z) = (*z) << 2;
	}

	return 0;
}

static void report_abs(void)
{
	short x, y, z;
	int result;

	do {
		result =
		    i2c_smbus_read_byte_data(mma8450_i2c_client,
					     MMA8450_STATUS3);
	} while (!(result & 0x08));	/* wait for new data */

	if (mma8450_read_data(&x, &y, &z) != 0)
		return;

	input_report_abs(mma8450_idev->input, ABS_X, x);
	input_report_abs(mma8450_idev->input, ABS_Y, y);
	input_report_abs(mma8450_idev->input, ABS_Z, z);
	input_sync(mma8450_idev->input);
}

static void mma8450_dev_poll(struct input_polled_dev *dev)
{
	report_abs();
}

/*
 * I2C init/probing/exit functions
 */
static int __devinit mma8450_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result;
	struct i2c_adapter *adapter;
	struct input_dev *idev;

	mma8450_i2c_client = client;
	adapter = to_i2c_adapter(client->dev.parent);

	result = i2c_check_functionality(adapter,
					 I2C_FUNC_SMBUS_BYTE |
					 I2C_FUNC_SMBUS_BYTE_DATA);
	assert(result);

	printk(KERN_INFO "check mma8450 chip ID\n");
	result = i2c_smbus_read_byte_data(client, MMA8450_WHO_AM_I);

	if (MMA8450_ID != (result)) {
		dev_err(&client->dev,
			"read chip ID 0x%x is not equal to 0x%x!\n", result,
			MMA8450_ID);
		printk(KERN_INFO "read chip ID failed\n");
		result = -EINVAL;
		goto err_detach_client;
	}

	/* Initialize the MMA8450 chip */
	result = mma8450_init_client(client);
	assert(result == 0);

	hwmon_dev = hwmon_device_register(&client->dev);
	assert(!(IS_ERR(hwmon_dev)));

	dev_info(&client->dev, "build time %s %s\n", __DATE__, __TIME__);

	/*input poll device register */
	mma8450_idev = input_allocate_polled_device();
	if (!mma8450_idev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		result = -ENOMEM;
		return result;
	}
	mma8450_idev->poll = mma8450_dev_poll;
	mma8450_idev->poll_interval = POLL_INTERVAL;
	mma8450_idev->poll_interval_max = POLL_INTERVAL_MAX;
	idev = mma8450_idev->input;
	idev->name = MMA8450_DRV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(idev, ABS_X, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Y, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Z, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);
	result = input_register_polled_device(mma8450_idev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		return result;
	}

	return result;

err_detach_client:
	return result;
}

static int __devexit mma8450_remove(struct i2c_client *client)
{
	int result;
	mma_status.ctl_reg1 =
	    i2c_smbus_read_byte_data(client, MMA8450_CTRL_REG1);
	result =
	    i2c_smbus_write_byte_data(client, MMA8450_CTRL_REG1,
				      mma_status.ctl_reg1 & 0xFC);
	assert(result == 0);

	hwmon_device_unregister(hwmon_dev);

	return result;
}

static int mma8450_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int result;
	mma_status.ctl_reg1 =
	    i2c_smbus_read_byte_data(client, MMA8450_CTRL_REG1);
	result =
	    i2c_smbus_write_byte_data(client, MMA8450_CTRL_REG1,
				      mma_status.ctl_reg1 & 0xFC);
	assert(result == 0);
	return result;
}

static int mma8450_resume(struct i2c_client *client)
{
	int result;
	result =
	    i2c_smbus_write_byte_data(client, MMA8450_CTRL_REG1,
				      mma_status.mode);
	assert(result == 0);
	return result;
}

static const struct i2c_device_id mma8450_id[] = {
	{MMA8450_DRV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mma8450_id);

static struct i2c_driver mma8450_driver = {
	.driver = {
		   .name = MMA8450_DRV_NAME,
		   .owner = THIS_MODULE,
		   },
	.suspend = mma8450_suspend,
	.resume = mma8450_resume,
	.probe = mma8450_probe,
	.remove = __devexit_p(mma8450_remove),
	.id_table = mma8450_id,
};

static int __init mma8450_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&mma8450_driver);
	if (res < 0) {
		printk(KERN_INFO "add mma8450 i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "add mma8450 i2c driver\n");

	return res;
}

static void __exit mma8450_exit(void)
{
	printk(KERN_INFO "remove mma8450 i2c driver.\n");
	i2c_del_driver(&mma8450_driver);
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MMA8450 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");

module_init(mma8450_init);
module_exit(mma8450_exit);
