/*
 *  mma8x5x.c - Linux kernel modules for 3-Axis Orientation/Motion
 *  Detection Sensor MMA8451/MMA8452/MMA8453/MMA8652/MMA8653
 *
 *  Copyright (C) 2012-2014 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/poll.h>
#include <linux/hwmon.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>

#define MMA8X5X_I2C_ADDR        0x1D
#define MMA8451_ID                      0x1A
#define MMA8452_ID                      0x2A
#define MMA8453_ID                      0x3A
#define MMA8652_ID                      0x4A
#define MMA8653_ID                      0x5A


#define POLL_INTERVAL_MIN       1
#define POLL_INTERVAL_MAX       500
#define POLL_INTERVAL           100 /* msecs */

/* if sensor is standby ,set POLL_STOP_TIME to slow down the poll */
#define POLL_STOP_TIME          200
#define INPUT_FUZZ                      32
#define INPUT_FLAT                      32
#define MODE_CHANGE_DELAY_MS    100

#define MMA8X5X_STATUS_ZYXDR    0x08
#define MMA8X5X_BUF_SIZE    6

#define MMA8X5X_FIFO_SIZE		32
/* register enum for mma8x5x registers */
enum {
	MMA8X5X_STATUS = 0x00,
	MMA8X5X_OUT_X_MSB,
	MMA8X5X_OUT_X_LSB,
	MMA8X5X_OUT_Y_MSB,
	MMA8X5X_OUT_Y_LSB,
	MMA8X5X_OUT_Z_MSB,
	MMA8X5X_OUT_Z_LSB,

	MMA8X5X_F_SETUP = 0x09,
	MMA8X5X_TRIG_CFG,
	MMA8X5X_SYSMOD,
	MMA8X5X_INT_SOURCE,
	MMA8X5X_WHO_AM_I,
	MMA8X5X_XYZ_DATA_CFG,
	MMA8X5X_HP_FILTER_CUTOFF,

	MMA8X5X_PL_STATUS,
	MMA8X5X_PL_CFG,
	MMA8X5X_PL_COUNT,
	MMA8X5X_PL_BF_ZCOMP,
	MMA8X5X_P_L_THS_REG,

	MMA8X5X_FF_MT_CFG,
	MMA8X5X_FF_MT_SRC,
	MMA8X5X_FF_MT_THS,
	MMA8X5X_FF_MT_COUNT,

	MMA8X5X_TRANSIENT_CFG = 0x1D,
	MMA8X5X_TRANSIENT_SRC,
	MMA8X5X_TRANSIENT_THS,
	MMA8X5X_TRANSIENT_COUNT,

	MMA8X5X_PULSE_CFG,
	MMA8X5X_PULSE_SRC,
	MMA8X5X_PULSE_THSX,
	MMA8X5X_PULSE_THSY,
	MMA8X5X_PULSE_THSZ,
	MMA8X5X_PULSE_TMLT,
	MMA8X5X_PULSE_LTCY,
	MMA8X5X_PULSE_WIND,

	MMA8X5X_ASLP_COUNT,
	MMA8X5X_CTRL_REG1,
	MMA8X5X_CTRL_REG2,
	MMA8X5X_CTRL_REG3,
	MMA8X5X_CTRL_REG4,
	MMA8X5X_CTRL_REG5,

	MMA8X5X_OFF_X,
	MMA8X5X_OFF_Y,
	MMA8X5X_OFF_Z,

	MMA8X5X_REG_END,
};

/* The sensitivity is represented in counts/g. In 2g mode the
   sensitivity is 1024 counts/g. In 4g mode the sensitivity is 512
   counts/g and in 8g mode the sensitivity is 256 counts/g.
 */
enum {
	MODE_2G = 0,
	MODE_4G,
	MODE_8G,
};

enum {
	MMA_STANDBY = 0,
	MMA_ACTIVED,
};
#pragma pack(1)
struct mma8x5x_data_axis {
	short x;
	short y;
	short z;
};
struct mma8x5x_fifo{
	int count;
	s64 period;
	s64 timestamp;
	struct mma8x5x_data_axis fifo_data[MMA8X5X_FIFO_SIZE];
};
#pragma pack()

struct mma8x5x_data {
	struct i2c_client *client;
	struct input_dev *idev;
	struct delayed_work work;
	struct mutex data_lock;
	struct mma8x5x_fifo fifo;
	wait_queue_head_t fifo_wq;
	atomic_t fifo_ready;
	int active;
	int delay;
	int position;
	u8  chip_id;
	int mode;
	int awaken;
	s64 period_rel;
	int fifo_wakeup;
	int fifo_timeout;
	u32 int_pin;
};

static struct mma8x5x_data *p_mma8x5x_data;

/* Addresses scanned */
static const unsigned short normal_i2c[] = { 0x1c, 0x1d, I2C_CLIENT_END };

static int mma8x5x_chip_id[] = {
	MMA8451_ID,
	MMA8452_ID,
	MMA8453_ID,
	MMA8652_ID,
	MMA8653_ID,
};
static char *mma8x5x_names[] = {
	"mma8451",
	"mma8452",
	"mma8453",
	"mma8652",
	"mma8653",
};
static int mma8x5x_position_setting[8][3][3] = {
	{ { 0,	-1, 0  }, { 1,	0,  0	   }, { 0, 0, 1	     } },
	{ { -1, 0,  0  }, { 0,	-1, 0	   }, { 0, 0, 1	     } },
	{ { 0,	1,  0  }, { -1, 0,  0	   }, { 0, 0, 1	     } },
	{ { 1,	0,  0  }, { 0,	1,  0	   }, { 0, 0, 1	     } },

	{ { 0,	-1, 0  }, { -1, 0,  0	   }, { 0, 0, -1     } },
	{ { -1, 0,  0  }, { 0,	1,  0	   }, { 0, 0, -1     } },
	{ { 0,	1,  0  }, { 1,	0,  0	   }, { 0, 0, -1     } },
	{ { 1,	0,  0  }, { 0,	-1, 0	   }, { 0, 0, -1     } },
};
static int mma8x5x_data_convert(struct mma8x5x_data *pdata,
		struct mma8x5x_data_axis *axis_data)
{
	short rawdata[3], data[3];
	int i, j;
	int position = pdata->position;

	if (position < 0 || position > 7)
		position = 0;
	rawdata[0] = axis_data->x;
	rawdata[1] = axis_data->y;
	rawdata[2] = axis_data->z;
	for (i = 0; i < 3; i++) {
		data[i] = 0;
		for (j = 0; j < 3; j++)
			data[i] += rawdata[j] *
					mma8x5x_position_setting[position][i][j];
	}
	axis_data->x = data[0];
	axis_data->y = data[1];
	axis_data->z = data[2];
	return 0;
}
static int mma8x5x_check_id(int id)
{
	int i = 0;

	for (i = 0; i < sizeof(mma8x5x_chip_id) /
			sizeof(mma8x5x_chip_id[0]); i++)
		if (id == mma8x5x_chip_id[i])
			return 1;
	return 0;
}
static char *mma8x5x_id2name(u8 id)
{
	return mma8x5x_names[(id >> 4) - 1];
}

static int mma8x5x_i2c_read_fifo(struct i2c_client *client,
					u8 reg, char *buf, int len)
{
	char send_buf[] = {reg};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = send_buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	if (i2c_transfer(client->adapter, msgs, 2) < 0) {
		printk(KERN_ERR "mma8x5x: transfer error\n");
		return -EIO;
	} else
		return len;
}

/*period is ms, return the real period per event*/
static s64 mma8x5x_odr_set(struct i2c_client *client, int period)
{
	u8 odr;
	u8 val;
	s64 period_rel;
	val = i2c_smbus_read_byte_data(client, MMA8X5X_CTRL_REG1);
	/*Standby*/
	i2c_smbus_write_byte_data(client, MMA8X5X_CTRL_REG1, val & (~0x01));
	val &= ~(0x07 << 3);
	if (period >= 640) {
		/*1.56HZ*/
		odr = 0x7;
		period_rel = 640 * NSEC_PER_MSEC;
	} else if (period >= 160) {
		/*6.25HZ*/
		odr = 0x06;
		period_rel = 160 * NSEC_PER_MSEC;
	} else if (period >= 80) {
		/*12.5HZ*/
		odr = 0x05;
		period_rel = 80 * NSEC_PER_MSEC;
	} else if (period >= 20) {
		/*50HZ*/
		odr = 0x04;
		period_rel = 20 * NSEC_PER_MSEC;
	} else if (period >= 10) {
		/*100HZ*/
		odr = 0x03;
		period_rel = 10 * NSEC_PER_MSEC;
	} else if (period >= 5) {
		/*200HZ*/
		odr = 0x02;
		period_rel = 5 * NSEC_PER_MSEC;
	} else if ((period * 2) >= 5) {
		/*400HZ*/
		odr = 0x01;
		period_rel = 2500 * NSEC_PER_USEC;
	} else {
		/*800HZ*/
		odr = 0x00;
		period_rel = 1250 * NSEC_PER_USEC;
	}
	val |= (odr << 3);
	/*Standby*/
	i2c_smbus_write_byte_data(client, MMA8X5X_CTRL_REG1, val);
	return period_rel;
}
static int mma8x5x_device_init(struct i2c_client *client)
{
	int result;
	struct mma8x5x_data *pdata = i2c_get_clientdata(client);

	result = i2c_smbus_write_byte_data(client, MMA8X5X_CTRL_REG1, 0);
	if (result < 0)
		goto out;

	result = i2c_smbus_write_byte_data(client, MMA8X5X_XYZ_DATA_CFG,
					   pdata->mode);
	if (result < 0)
		goto out;
	pdata->active = MMA_STANDBY;
	msleep(MODE_CHANGE_DELAY_MS);
	return 0;
out:
	dev_err(&client->dev, "error when init mma8x5x:(%d)", result);
	return result;
}
static int mma8x5x_device_stop(struct i2c_client *client)
{
	u8 val;

	val = i2c_smbus_read_byte_data(client, MMA8X5X_CTRL_REG1);
	i2c_smbus_write_byte_data(client, MMA8X5X_CTRL_REG1, val & 0xfe);
	return 0;
}
static int mma8x5x_read_data(struct i2c_client *client,
		struct mma8x5x_data_axis *data)
{
	u8 tmp_data[MMA8X5X_BUF_SIZE];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client,
					    MMA8X5X_OUT_X_MSB,
						MMA8X5X_BUF_SIZE, tmp_data);
	if (ret < MMA8X5X_BUF_SIZE) {
		dev_err(&client->dev, "i2c block read failed\n");
		return -EIO;
	}
	data->x = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];
	data->y = ((tmp_data[2] << 8) & 0xff00) | tmp_data[3];
	data->z = ((tmp_data[4] << 8) & 0xff00) | tmp_data[5];
	return 0;
}

static int mma8x5x_fifo_interrupt(struct i2c_client *client, int enable)
{
	u8 val, sys_mode;
	sys_mode =  i2c_smbus_read_byte_data(client, MMA8X5X_CTRL_REG1);
	/*standby*/
	i2c_smbus_write_byte_data(client, MMA8X5X_CTRL_REG1,
					(sys_mode & (~0x01)));
	val = i2c_smbus_read_byte_data(client, MMA8X5X_CTRL_REG4);
	val &= ~(0x01 << 6);
	if (enable)
		val |= (0x01 << 6);
	i2c_smbus_write_byte_data(client, MMA8X5X_CTRL_REG4, val);
	i2c_smbus_write_byte_data(client, MMA8X5X_CTRL_REG1, sys_mode);
	return 0;
}

static int mma8x5x_fifo_setting(struct mma8x5x_data *pdata,
				int time_out, int is_overwrite)
{
	u8 val, sys_mode, pin_cfg;
	struct i2c_client *client = pdata->client;
	sys_mode =  i2c_smbus_read_byte_data(client, MMA8X5X_CTRL_REG1);
	/*standby*/
	i2c_smbus_write_byte_data(client,
				MMA8X5X_CTRL_REG1,
				(sys_mode & (~0x01)));
	pin_cfg = i2c_smbus_read_byte_data(client, MMA8X5X_CTRL_REG5);
	val = i2c_smbus_read_byte_data(client, MMA8X5X_F_SETUP);
	val &= ~(0x03 << 6);
	if (time_out > 0) {
		if (is_overwrite)
			val |= (0x01 << 6);
		else
			val |= (0x02 << 6);
	}
	i2c_smbus_write_byte_data(client, MMA8X5X_F_SETUP, val);
	/*route to pin 1*/
	if (pdata->int_pin == 1)
		i2c_smbus_write_byte_data(client,
					MMA8X5X_CTRL_REG5,
					pin_cfg | (0x01 << 6));
	/*route to pin 1*/
	else
		i2c_smbus_write_byte_data(client,
					MMA8X5X_CTRL_REG5,
					pin_cfg & ~(0x01 << 6));

	i2c_smbus_write_byte_data(client, MMA8X5X_CTRL_REG1, sys_mode);

	if (time_out > 0) {
		/*fifo len is 32*/
		 pdata->period_rel = mma8x5x_odr_set(client, time_out/32);
	}
	return 0;
}
static int mma8x5x_read_fifo_data(struct mma8x5x_data *pdata)
{
	int count, cnt;
	u8 buf[256], val;
	int i, index;
	struct i2c_client *client = pdata->client;
	struct mma8x5x_fifo *pfifo = &pdata->fifo;
	struct timespec ts;
	val = i2c_smbus_read_byte_data(client, MMA8X5X_STATUS);
	/*FIFO overflow*/
	if (val & (0x01 << 7)) {
		cnt = (val & 0x3f);
		count = mma8x5x_i2c_read_fifo(client, MMA8X5X_OUT_X_MSB,
						buf, MMA8X5X_BUF_SIZE * cnt);
		if (count > 0) {
			ktime_get_ts(&ts);
			for (i = 0; i < count/MMA8X5X_BUF_SIZE ; i++) {
				index = MMA8X5X_BUF_SIZE * i;
				pfifo->fifo_data[i].x =
					((buf[index] << 8) & 0xff00) | buf[index + 1];
				pfifo->fifo_data[i].y =
					((buf[index + 2] << 8) & 0xff00) | buf[index + 3];
				pfifo->fifo_data[i].z =
					((buf[index + 4] << 8) & 0xff00) | buf[index + 5];
				mma8x5x_data_convert(pdata,
							&pfifo->fifo_data[i]);
			}
			pfifo->period = pdata->period_rel;
			pfifo->count = count / MMA8X5X_BUF_SIZE;
			pfifo->timestamp = ((s64)ts.tv_sec) * NSEC_PER_SEC
						+ ts.tv_nsec;
			return 0;
		}
	}
	return -1;
}

static void mma8x5x_report_data(struct mma8x5x_data *pdata)
{
	struct mma8x5x_data_axis data;
	int ret;
	ret = mma8x5x_read_data(pdata->client, &data);
	if (!ret) {
		mma8x5x_data_convert(pdata, &data);
		input_report_abs(pdata->idev, ABS_X, data.x);
		input_report_abs(pdata->idev, ABS_Y, data.y);
		input_report_abs(pdata->idev, ABS_Z, data.z);
		input_sync(pdata->idev);
	}
}
static void mma8x5x_work(struct mma8x5x_data *pdata)
{
	int delay;
	if (pdata->active == MMA_ACTIVED) {
		delay = msecs_to_jiffies(pdata->delay);
		if (delay >= HZ)
			delay = round_jiffies_relative(delay);
		schedule_delayed_work(&pdata->work, delay);
	}
}
static void mma8x5x_dev_poll(struct work_struct *work)
{
	struct mma8x5x_data *pdata = container_of(work,
						struct mma8x5x_data,
						work.work);
	mma8x5x_report_data(pdata);
	mma8x5x_work(pdata);
}
static irqreturn_t mma8x5x_irq_handler(int irq, void *dev)
{
	int ret;
	u8 int_src;
	struct mma8x5x_data *pdata = (struct mma8x5x_data *)dev;
	int_src = i2c_smbus_read_byte_data(pdata->client, MMA8X5X_INT_SOURCE);
	if (int_src & (0x01 << 6)) {
		ret = mma8x5x_read_fifo_data(pdata);
		if (!ret) {
			atomic_set(&pdata->fifo_ready, 1);
			wake_up(&pdata->fifo_wq);
		}
		/*is just awken from suspend*/
		if (pdata->awaken) {
			/*10s timeout*/
			mma8x5x_fifo_setting(pdata, pdata->fifo_timeout, 0);
			mma8x5x_fifo_interrupt(pdata->client, 1);
			pdata->awaken = 0;
		}
	}
	return IRQ_HANDLED;
}

static ssize_t mma8x5x_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mma8x5x_data *pdata = dev_get_drvdata(dev);
	struct i2c_client *client = pdata->client;
	u8 val;
	int enable;

	mutex_lock(&pdata->data_lock);
	val = i2c_smbus_read_byte_data(client, MMA8X5X_CTRL_REG1);
	if ((val & 0x01) && pdata->active == MMA_ACTIVED)
		enable = 1;
	else
		enable = 0;
	mutex_unlock(&pdata->data_lock);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t mma8x5x_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct mma8x5x_data *pdata = dev_get_drvdata(dev);
	struct i2c_client *client = pdata->client;
	int ret;
	unsigned long enable;
	u8 val = 0;

	ret = strict_strtol(buf, 10, &enable);
	if (ret) {
		dev_err(dev, "string to long error\n");
		return ret;
	}
	mutex_lock(&pdata->data_lock);
	enable = (enable > 0) ? 1 : 0;
	if (enable && pdata->active == MMA_STANDBY) {
		val = i2c_smbus_read_byte_data(client, MMA8X5X_CTRL_REG1);
		ret = i2c_smbus_write_byte_data(client,
						MMA8X5X_CTRL_REG1,
						val | 0x01);
		if (!ret) {
			pdata->active = MMA_ACTIVED;
			/*continuous mode*/
			if (pdata->fifo_timeout <= 0)
				mma8x5x_work(pdata);
			else {
				/*fifo mode*/
				mma8x5x_fifo_setting(pdata,
							pdata->fifo_timeout, 0);
				mma8x5x_fifo_interrupt(client, 1);
			}
			printk(KERN_INFO"mma enable setting active\n");
		}
	} else if (enable == 0 && pdata->active == MMA_ACTIVED) {
		val = i2c_smbus_read_byte_data(client, MMA8X5X_CTRL_REG1);
		ret = i2c_smbus_write_byte_data(client, MMA8X5X_CTRL_REG1,
							val & 0xFE);
		if (!ret) {
			pdata->active = MMA_STANDBY;
			if (pdata->fifo_timeout <= 0)
				/*continuous mode*/
				cancel_delayed_work_sync(&pdata->work);
			else {
				/*fifo mode*/
				mma8x5x_fifo_setting(pdata, 0, 0);
				mma8x5x_fifo_interrupt(client, 0);
			}
			printk(KERN_INFO"mma enable setting inactive\n");
		}
	}
	mutex_unlock(&pdata->data_lock);
	return count;
}

static ssize_t mma8x5x_delay_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mma8x5x_data *pdata = dev_get_drvdata(dev);
	int delay;
	mutex_lock(&pdata->data_lock);
	delay  = pdata->delay;
	mutex_unlock(&pdata->data_lock);
	return sprintf(buf, "%d\n", delay);
}

static ssize_t mma8x5x_delay_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct mma8x5x_data *pdata = dev_get_drvdata(dev);
	struct i2c_client *client = pdata->client;
	int ret;
	long delay;
	ret = strict_strtol(buf, 10, &delay);
	if (ret) {
		dev_err(dev, "string to long error\n");
		return ret;
	}

	mutex_lock(&pdata->data_lock);
	cancel_delayed_work_sync(&pdata->work);
	pdata->delay = (int)delay;
	if (pdata->active == MMA_ACTIVED && pdata->fifo_timeout <= 0) {
		mma8x5x_odr_set(client, (int)delay);
		mma8x5x_work(pdata);
	}
	mutex_unlock(&pdata->data_lock);
	return count;
}

static ssize_t mma8x5x_fifo_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct mma8x5x_data *pdata = dev_get_drvdata(dev);
	mutex_lock(&pdata->data_lock);
	count = sprintf(&buf[count], "period poll  :%d  ms\n", pdata->delay);
	count += sprintf(&buf[count], "period fifo  :%lld  ns\n",
				pdata->period_rel);
	count += sprintf(&buf[count], "timeout :%d ms\n", pdata->fifo_timeout);
	/*is the interrupt enable*/
	count += sprintf(&buf[count], "interrupt wake up: %s\n",
				(pdata->fifo_wakeup ? "yes" : "no"));
	mutex_unlock(&pdata->data_lock);
	return count;
}

static ssize_t mma8x5x_fifo_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct mma8x5x_data *pdata = dev_get_drvdata(dev);
	struct i2c_client *client = pdata->client;
	int period, timeout, wakeup;
	sscanf(buf, "%d,%d,%d", &period, &timeout, &wakeup);
	printk(KERN_INFO"period %d ,timeout is %d, wake up is :%d\n",
			period, timeout, wakeup);
	if (timeout > 0) {
		mutex_lock(&pdata->data_lock);
		cancel_delayed_work_sync(&pdata->work);
		pdata->delay = period;
		mutex_unlock(&pdata->data_lock);
		/*no overwirte fifo*/
		mma8x5x_fifo_setting(pdata, timeout, 0);
		mma8x5x_fifo_interrupt(client, 1);
		pdata->fifo_timeout = timeout;
		pdata->fifo_wakeup = wakeup;
	} else {
		/*no overwirte fifo*/
		mma8x5x_fifo_setting(pdata, timeout, 0);
		mma8x5x_fifo_interrupt(client, 0);
		pdata->fifo_timeout = timeout;
		pdata->fifo_wakeup = wakeup;
		mutex_lock(&pdata->data_lock);
		pdata->delay = period;
		if (pdata->active == MMA_ACTIVED)
			mma8x5x_work(pdata);
		mutex_unlock(&pdata->data_lock);
	}
	return count;
}

static ssize_t mma8x5x_position_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct mma8x5x_data *pdata  = dev_get_drvdata(dev);
	int position = 0;
	mutex_lock(&pdata->data_lock);
	position = pdata->position;
	mutex_unlock(&pdata->data_lock);
	return sprintf(buf, "%d\n", position);
}

static ssize_t mma8x5x_position_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct mma8x5x_data *pdata  = dev_get_drvdata(dev);
	int ret;
	long position;
	ret = strict_strtol(buf, 10, &position);
	if (ret) {
		dev_err(dev, "string to long error\n");
		return ret;
	}
	mutex_lock(&pdata->data_lock);
	pdata->position = (int)position;
	mutex_unlock(&pdata->data_lock);
	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   mma8x5x_enable_show, mma8x5x_enable_store);
static DEVICE_ATTR(poll_delay, S_IWUSR | S_IRUGO,
		   mma8x5x_delay_show, mma8x5x_delay_store);

static DEVICE_ATTR(fifo, S_IWUSR | S_IRUGO,
		   mma8x5x_fifo_show, mma8x5x_fifo_store);
static DEVICE_ATTR(position, S_IWUSR | S_IRUGO,
		   mma8x5x_position_show, mma8x5x_position_store);

static struct attribute *mma8x5x_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_fifo.attr,
	&dev_attr_position.attr,
	NULL
};

static const struct attribute_group mma8x5x_attr_group = {
	.attrs	= mma8x5x_attributes,
};
static int mma8x5x_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int chip_id;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -ENODEV;
	chip_id = i2c_smbus_read_byte_data(client, MMA8X5X_WHO_AM_I);
	if (!mma8x5x_check_id(chip_id))
		return -ENODEV;
	printk(KERN_INFO"check %s i2c address 0x%x\n",
			mma8x5x_id2name(chip_id), client->addr);
	strlcpy(info->type, "mma8x5x", I2C_NAME_SIZE);
	return 0;
}
static int mma8x5x_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err)
		return err;
	file->private_data = p_mma8x5x_data;
	return 0;
}
static ssize_t mma8x5x_read(struct file *file,
				char __user *buf,
				size_t size, loff_t *ppos)
{
	struct mma8x5x_data *pdata = file->private_data;
	int ret = 0;
	if (!(file->f_flags & O_NONBLOCK)) {
		ret = wait_event_interruptible(pdata->fifo_wq,
			(atomic_read(&pdata->fifo_ready) != 0));
		if (ret)
			return ret;
	}
	if (!atomic_read(&pdata->fifo_ready))
		return -ENODEV;
	if (size < sizeof(struct mma8x5x_fifo)) {
		printk(KERN_ERR"the buffer leght less than need\n");
		return -ENOMEM;
	}
	if (!copy_to_user(buf, &pdata->fifo, sizeof(struct mma8x5x_fifo))) {
		atomic_set(&pdata->fifo_ready, 0);
		return size;
	}
	return -ENOMEM ;
}
static unsigned int mma8x5x_poll(struct file *file,
					struct poll_table_struct *wait)
{
	struct mma8x5x_data *pdata = file->private_data;
	poll_wait(file, &pdata->fifo_wq, wait);
	if (atomic_read(&pdata->fifo_ready))
		return POLLIN | POLLRDNORM;
	return 0;
}

static const struct file_operations mma8x5x_fops = {
	.owner = THIS_MODULE,
	.open = mma8x5x_open,
	.read = mma8x5x_read,
	.poll = mma8x5x_poll,
};

static struct miscdevice mma8x5x_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mma8x5x",
	.fops = &mma8x5x_fops,
};

static int mma8x5x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result, chip_id;
	struct input_dev *idev;
	struct mma8x5x_data *pdata;
	struct i2c_adapter *adapter;
	struct device_node *of_node = client->dev.of_node;
	u32 pos = 0;
	struct regulator *vdd, *vdd_io;
	u32 irq_flag;
	struct irq_data *irq_data;

	vdd = devm_regulator_get(&client->dev, "vdd");
	if (!IS_ERR(vdd)) {
		result = regulator_enable(vdd);
		if (result) {
			dev_err(&client->dev, "vdd set voltage error\n");
			return result;
		}
	}

	vdd_io = devm_regulator_get(&client->dev, "vddio");
	if (!IS_ERR(vdd_io)) {
		result = regulator_enable(vdd_io);
		if (result) {
			dev_err(&client->dev, "vddio set voltage error\n");
			return result;
		}
	}
	adapter = to_i2c_adapter(client->dev.parent);
	result = i2c_check_functionality(adapter,
					 I2C_FUNC_SMBUS_BYTE |
					 I2C_FUNC_SMBUS_BYTE_DATA);
	if (!result)
		goto err_out;

	chip_id = i2c_smbus_read_byte_data(client, MMA8X5X_WHO_AM_I);

	if (!mma8x5x_check_id(chip_id)) {
		dev_err(&client->dev,
			"read chip ID 0x%x is not equal to 0x%x,0x%x,0x%x,0x%x,0x%x!\n",
			chip_id, MMA8451_ID, MMA8452_ID,
			MMA8453_ID, MMA8652_ID, MMA8653_ID);
		result = -EINVAL;
		goto err_out;
	}
	pdata = kzalloc(sizeof(struct mma8x5x_data), GFP_KERNEL);
	if (!pdata) {
		result = -ENOMEM;
		dev_err(&client->dev, "alloc data memory error!\n");
		goto err_out;
	}

	/* Initialize the MMA8X5X chip */
	memset(pdata, 0, sizeof(struct mma8x5x_data));
	pdata->client = client;
	pdata->chip_id = chip_id;
	pdata->mode = MODE_2G;
	pdata->fifo_wakeup = 0;
	pdata->fifo_timeout = 0;

	result = of_property_read_u32(of_node, "position", &pos);
	if (result)
		pos = 1;
	pdata->position = (int)pos;
	p_mma8x5x_data = pdata;
	mutex_init(&pdata->data_lock);
	i2c_set_clientdata(client, pdata);

	mma8x5x_device_init(client);

	idev = input_allocate_device();
	if (!idev) {
		result = -ENOMEM;
		dev_err(&client->dev, "alloc input device failed!\n");
		goto err_alloc_input_device;
	}
	idev->name = "FreescaleAccelerometer";
	idev->uniq = mma8x5x_id2name(pdata->chip_id);
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);
	input_set_abs_params(idev, ABS_X, -0x7fff, 0x7fff, 0, 0);
	input_set_abs_params(idev, ABS_Y, -0x7fff, 0x7fff, 0, 0);
	input_set_abs_params(idev, ABS_Z, -0x7fff, 0x7fff, 0, 0);
	dev_set_drvdata(&idev->dev, pdata);
	pdata->idev = idev;
	result = input_register_device(pdata->idev);
	if (result) {
		dev_err(&client->dev, "register input device failed!\n");
		goto err_register_input_device;
	}
	pdata->delay = POLL_INTERVAL;
	INIT_DELAYED_WORK(&pdata->work, mma8x5x_dev_poll);
	result = sysfs_create_group(&idev->dev.kobj, &mma8x5x_attr_group);
	if (result) {
		dev_err(&client->dev, "create device file failed!\n");
		result = -EINVAL;
		goto err_create_sysfs;
	}
	init_waitqueue_head(&pdata->fifo_wq);

	if (client->irq) {
		irq_data = irq_get_irq_data(client->irq);
		irq_flag = irqd_get_trigger_type(irq_data);
		irq_flag |= IRQF_ONESHOT;
		result = request_threaded_irq(client->irq, NULL,
						mma8x5x_irq_handler,
						irq_flag,
						client->dev.driver->name,
						pdata);
		if (result < 0) {
			dev_err(&client->dev,
					"failed to register MMA8x5x irq %d!\n",
				client->irq);
			goto err_register_irq;
		} else {
			result = misc_register(&mma8x5x_dev);
			if (result) {
				dev_err(&client->dev,
						"register fifo device error\n");
				goto err_reigster_dev;
			}
		}

		result = of_property_read_u32(of_node,
						"interrupt-route",
						&pdata->int_pin);
		if (result) {
			result = -EINVAL;
			dev_err(&client->dev,
				"Can't find interrupt-pin value\n");
			goto err_reigster_dev;

		}
		if (pdata->int_pin == 0 || pdata->int_pin > 2) {
			result = -EINVAL;
			dev_err(&client->dev,
				"The interrupt-pin value is invalid\n");
			goto err_reigster_dev;
		}
	}
	printk(KERN_INFO"mma8x5x device driver probe successfully\n");
	return 0;
err_reigster_dev:
	free_irq(client->irq, pdata);
err_register_irq:
	sysfs_remove_group(&idev->dev.kobj, &mma8x5x_attr_group);
err_create_sysfs:
	input_unregister_device(pdata->idev);
err_register_input_device:
	input_free_device(idev);
err_alloc_input_device:
	kfree(pdata);
err_out:
	return result;
}

static int mma8x5x_remove(struct i2c_client *client)
{
	struct mma8x5x_data *pdata = i2c_get_clientdata(client);
	struct input_dev *idev = pdata->idev;
	mma8x5x_device_stop(client);
	if (pdata) {
		sysfs_remove_group(&idev->dev.kobj, &mma8x5x_attr_group);
		input_unregister_device(pdata->idev);
		input_free_device(pdata->idev);
		kfree(pdata);
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mma8x5x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mma8x5x_data *pdata = i2c_get_clientdata(client);
	if (pdata->fifo_timeout <= 0) {
		if (pdata->active == MMA_ACTIVED)
				mma8x5x_device_stop(client);
	} else {
		if (pdata->active == MMA_ACTIVED) {
			if (pdata->fifo_wakeup) {
				/*10s timeout , overwrite*/
				mma8x5x_fifo_setting(pdata, 10000, 0);
				mma8x5x_fifo_interrupt(client, 1);
			} else {
				mma8x5x_fifo_interrupt(client, 0);
				/*10s timeout , overwrite*/
				mma8x5x_fifo_setting(pdata, 10000, 1);
			}
		}
	}
	return 0;
}

static int mma8x5x_resume(struct device *dev)
{
	int val = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct mma8x5x_data *pdata = i2c_get_clientdata(client);
	if (pdata->fifo_timeout <= 0) {
		if (pdata->active == MMA_ACTIVED) {
			val = i2c_smbus_read_byte_data(client,
				MMA8X5X_CTRL_REG1);
			i2c_smbus_write_byte_data(client,
				MMA8X5X_CTRL_REG1, val | 0x01);
		}
	} else {
		if (pdata->active == MMA_ACTIVED) {
			mma8x5x_fifo_interrupt(client, 1);
			/*Awake from suspend*/
			pdata->awaken = 1;
		}
	}
	return 0;

}
#endif

static const struct i2c_device_id mma8x5x_id[] = {
	{"mma8451", 0},
	{"mma8452", 0},
	{"mma8453", 0},
	{"mma8652", 0},
	{"mma8653", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mma8x5x_id);

static SIMPLE_DEV_PM_OPS(mma8x5x_pm_ops, mma8x5x_suspend, mma8x5x_resume);
static struct i2c_driver mma8x5x_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver		= {
		.name	= "mma8x5x",
		.owner	= THIS_MODULE,
		.pm	= &mma8x5x_pm_ops,
	},
	.probe		= mma8x5x_probe,
	.remove		= mma8x5x_remove,
	.id_table	= mma8x5x_id,
	.detect		= mma8x5x_detect,
	.address_list	= normal_i2c,
};


module_i2c_driver(mma8x5x_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MMA8X5X 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");

