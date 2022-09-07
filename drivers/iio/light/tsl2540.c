// SPDX-License-Identifier: GPL-2.0
/*
 * Device driver for AMS Light-to-Digital Sensor TSL2540
 *
 * Copyright 2022 NXP
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

/* TSL2540 Device/Reverdion ID */
#define TSL2540_REV_ID			0x01
#define TSL2540_DEVICE_ID		0xE4

/*Time from power-on to ready to receive I²C commands*/
#define TSL2540_BOOT_MAX_SLEEP_TIME	1600
#define TSL2540_BOOT_MIN_SLEEP_TIME	1500

/* Device Registers and Masks */
#define TSL2540_ENABLE			0x80
#define TSL2540_ALS_TIME		0X81
#define TSL2540_WAIT_TIME		0x03
#define TSL2540_ALS_LOWTHRESHLO		0X84
#define TSL2540_ALS_LOWTHRESHHI		0X85
#define TSL2540_ALS_HIGHTHRESHLO	0X86
#define TSL2540_ALS_HIGHTHRESHHI	0X77
#define TSL2540_PERSISTENCE		0x8C
#define TSL2540_CONFIG0			0x8D
#define TSL2540_CONFIG1			0x90
#define TSL2540_REVID			0x91
#define TSL2540_DEVICEID		0x92
#define TSL2540_STATUS			0x93
#define TSL2540_VIS_CHANLO		0x94
#define TSL2540_VIS_CHANHI		0x95
#define TSL2540_IR_CHANLO		0x96
#define TSL2540_IR_CHANHI		0x97
#define TSL2540_REVID2			0x9E
#define TSL2540_CONFIG2			0x9F
#define TSL2540_CONFIG3			0xAB
#define TSL2540_AZ_CONFIG		0xD6
#define TSL2540_INTENAB			0xDD

/* tsl2540 enable reg masks */
#define TSL2540_ENABLE_WEN		0x08
#define TSL2540_ENABLE_AEN		0x02
#define TSL2540_ENABLE_PON		0x01

/* tsl2540 persistence reg masks */
#define TSL2540_APERS_MASKS		0x0F

/* tsl2540 CFG0 reg masks */
#define TSL2540_CFG0_WLONG		0x04  //If WLONG is enabled then Wait time = (wtime +1) x 2.8ms x 12

/* tsl2540 CFG1 reg masks */
#define TSL2540_CFG1_AGAIN_MASKS	0x03 //gain of the ALS sensor, 1x,4x,16x,64x

/* tsl2540 rev id reg masks */
#define TSL2540_REV_ID_MASKS		0x07
/* tsl2540 device id reg masks */
#define TSL2540_DEVICE_ID_MASKS		0xFC

/* tsl2540 status reg masks */
#define TSL2540_STA_ASAT		0x80 // Saturation flag
#define TSL2540_STA_AINT                0x10 // ALS Interrupt flag
#define TSL2540_STA_CINT                0x08 // Calibration Interrupt flag

/* tsl2540 revid2 reg masks */
#define TSL2540_REVID2_MASKS		0x01

/* tsl2540 CFG2 reg masks */
#define TSL2540_CFG2_AGAINMAX		0x10 // overall ALS gain factor, 1 means 2x of all
#define TSL2540_CFG2_AGAINL		0x04 // overall ALS gain factor, 0 means 1/2 of all

/* tsl2540 CFG2 reg masks */
#define TSL2540_CFG3_INT_READ_CLEAR	0x80 // Interrupt-Clear-by-Read bit
#define TSL2540_CFG3_SAI		0x10 // Sleep After Interrupt bit

/* tsl2540 CFG2 reg masks */
#define TSL2540_AZ_CONFIG_MASKS		0x7F // Run autozero automatically every n ALS cycle

/* tsl2540 INTENAB reg masks */
#define TSL2540_INTENAB_ASIEN		0x80 // ALS Saturation Interrupt Enable
#define TSL2540_INTENAB_AIEN		0x10 // ALS Interrupt Enable

struct tsl2540_settings {
	int int_time; // integration time
	int als_gain;
	int als_gain_low;
	int wait_time;
	int als_persistence;
	int als_interrupt_en;
	int als_thresh_low;
	int als_thresh_high;
};

struct tsl2540_chip {
	struct mutex als_mutex;
	struct i2c_client *client;
	struct tsl2540_settings settings;
	const struct iio_info *info;
	s64 event_timestamp;
};

static const struct tsl2540_settings tsl2540_default_settings = {
	.int_time = 180000, /* 180000us = 64*2.8ms to make ALS full scale */
	.als_gain = 67,  /* set 64x to register will get 67x according to ALS Operating Characteristics */
	.als_gain_low = 0, /* fractional part of als_gain */
	.wait_time = 2810, /* wait time between ALS cycles, 0 means 2.81ms */
	.als_persistence = 10, /* 10 consecutive ALS values out of range */
	.als_interrupt_en = TSL2540_INTENAB_AIEN,
	.als_thresh_low = 200,
	.als_thresh_high = 20000,
};

static const int tsl2540_int_time_avail[] = {
	0, 2810, 0, 2810, 0, 721000
};

static int tsl2540_hw_gain_avail[] = {
	0, 500000, 1, 0, 4, 0, 16, 0, 67, 0, 140, 0
};

static int tsl2540_read_reg(struct tsl2540_chip *chip, u8 addr)
{
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client, addr);
	if (ret < 0)
		dev_err(&chip->client->dev,
			"%s: failed to read register 0x%x: %d\n", __func__,
			addr, ret);

	return ret;
}

static int tsl2540_write_reg(struct tsl2540_chip *chip, u8 addr, u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(chip->client, addr, data);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to write to register 0x%x with 0x%x: %d\n",
			__func__, addr, data, ret);
	}

	return ret;
}

static int tsl2540_chip_init(struct iio_dev *indio_dev)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	int again, againmax, againl = 1;
	int cfg1, cfg2, pers;

	tsl2540_write_reg(chip, TSL2540_ALS_TIME, chip->settings.int_time / tsl2540_int_time_avail[3] - 1);
	tsl2540_write_reg(chip, TSL2540_WAIT_TIME, chip->settings.wait_time / tsl2540_int_time_avail[3] - 1);

	tsl2540_write_reg(chip, TSL2540_ALS_LOWTHRESHLO, chip->settings.als_thresh_low & 0xFF);
	tsl2540_write_reg(chip, TSL2540_ALS_LOWTHRESHHI, (chip->settings.als_thresh_low >> 8) & 0xFF);
	tsl2540_write_reg(chip, TSL2540_ALS_HIGHTHRESHLO, chip->settings.als_thresh_high & 0xFF);
	tsl2540_write_reg(chip, TSL2540_ALS_HIGHTHRESHHI, (chip->settings.als_thresh_high >> 8) & 0xFF);

	if (chip->settings.als_persistence > 3)
		pers = (chip->settings.als_persistence / 5) + 3;
	else
		pers = chip->settings.als_persistence;
	tsl2540_write_reg(chip, TSL2540_PERSISTENCE, pers);

	switch (chip->settings.als_gain) {
	case 0:
		if (chip->settings.als_gain_low == 500000) { // gain=0.5
			again = 0;
			againmax = 0;
			againl = 0;
		} else {
			return -EINVAL;
		}
		break;
	case 1:
		again = 0;
		againmax = 0;
		break;
	case 4:
		again = 1;
		againmax = 0;
		break;
	case 16:
		again = 2;
		againmax = 0;
		break;
	case 67:
		again = 3;
		againmax = 0;
		break;
	case 140:
		again = 3;
		againmax = 1;
		break;
	default:
		return -EINVAL;
	}
	cfg1 = (again & TSL2540_CFG1_AGAIN_MASKS);
	cfg2 = ((againl << 2) | (againmax << 4))
		& (TSL2540_CFG2_AGAINMAX | TSL2540_CFG2_AGAINL);

	tsl2540_write_reg(chip, TSL2540_CONFIG1, cfg1);
	tsl2540_write_reg(chip, TSL2540_CONFIG2, cfg2);

	/* Enable interrupt clear by read, and Sleep after interrupt by default*/
	tsl2540_write_reg(chip, TSL2540_CONFIG3, TSL2540_CFG3_INT_READ_CLEAR | TSL2540_CFG3_SAI);
	tsl2540_write_reg(chip, TSL2540_INTENAB, chip->settings.als_interrupt_en); //config interrupt register

	tsl2540_write_reg(chip, TSL2540_ENABLE, TSL2540_ENABLE_AEN | TSL2540_ENABLE_PON);

	return 0;
}

static int tsl2540_chip_on(struct iio_dev *indio_dev)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);

	/* turn device on */
	return tsl2540_write_reg(chip, TSL2540_ENABLE, TSL2540_ENABLE_AEN | TSL2540_ENABLE_PON);
}

static int tsl2540_chip_off(struct iio_dev *indio_dev)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);

	/* turn device off */
	return tsl2540_write_reg(chip, TSL2540_ENABLE, 0x00);
}

static void tsl2540_chip_off_action(void *data)
{
	struct iio_dev *indio_dev = data;

	tsl2540_chip_off(indio_dev);
}

static int tsl2540_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->type == IIO_LIGHT) {
			*length = ARRAY_SIZE(tsl2540_hw_gain_avail);
			*vals = tsl2540_hw_gain_avail;
		} else {
			return -EINVAL;
		}
		*type = IIO_VAL_INT_PLUS_MICRO;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_INT_TIME:
		*length = ARRAY_SIZE(tsl2540_int_time_avail);
		*vals = tsl2540_int_time_avail;
		*type = IIO_VAL_INT_PLUS_MICRO;
		return IIO_AVAIL_RANGE;
	}

	return -EINVAL;
}

static int tsl2540_read_interrupt_config(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan,
					 enum iio_event_type type,
					 enum iio_event_direction dir)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);

	if (chan->type == IIO_LIGHT)
		return (chip->settings.als_interrupt_en & TSL2540_INTENAB_AIEN) ? true : false;
	else
		return -EINVAL;
}

static int tsl2540_write_interrupt_config(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir,
					  int val)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	int bits, mask, ret;

	mask = chip->settings.als_interrupt_en;
	if (chan->type == IIO_LIGHT) {
		if (val)
			bits = mask & TSL2540_INTENAB_AIEN;
		else
			bits = mask & ~TSL2540_INTENAB_AIEN;
		ret = tsl2540_write_reg(chip, TSL2540_INTENAB, bits);
		if (ret >= 0)
			chip->settings.als_interrupt_en = bits;
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static int tsl2540_write_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int val, int val2)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	int ret = 0, persistence;

	switch (info) {
	case IIO_EV_INFO_VALUE:
		if (chan->type == IIO_LIGHT) {
			switch (dir) {
			case IIO_EV_DIR_RISING:
				tsl2540_write_reg(chip, TSL2540_ALS_HIGHTHRESHLO, (val & 0xFF));
				ret = tsl2540_write_reg(chip, TSL2540_ALS_HIGHTHRESHHI, ((val >> 8) & 0xFF));
				if (ret >= 0)
					chip->settings.als_thresh_high = val;
				break;
			case IIO_EV_DIR_FALLING:
				tsl2540_write_reg(chip, TSL2540_ALS_LOWTHRESHLO, (val & 0xFF));
				ret = tsl2540_write_reg(chip, TSL2540_ALS_LOWTHRESHHI, ((val >> 8) & 0xFF));
				if (ret >= 0)
					chip->settings.als_thresh_low = val;
				break;
			default:
				break;
			}
		} else {
			return -EINVAL;
		}
		break;
	case IIO_EV_INFO_PERIOD:
		if (chan->type == IIO_LIGHT) {
			persistence = ((val * 1000000) + val2) / chip->settings.int_time;
			/* ALS filter values are 1, 2, 3, 5, 10, 15, ..., 60 */
			if (persistence > 3)
				persistence = (persistence / 5) + 3;

			ret = tsl2540_write_reg(chip, TSL2540_PERSISTENCE, persistence);
			if (ret >= 0)
				chip->settings.als_persistence = persistence;
		} else {
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	return ret;
}

static int tsl2540_read_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int *val, int *val2)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	int filter_delay, persistence;
	int time;

	switch (info) {
	case IIO_EV_INFO_VALUE:
		if (chan->type == IIO_LIGHT) {
			switch (dir) {
			case IIO_EV_DIR_RISING:
				*val = chip->settings.als_thresh_high;
				return IIO_VAL_INT;
			case IIO_EV_DIR_FALLING:
				*val = chip->settings.als_thresh_low;
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
		} else {
			return -EINVAL;
		}
		break;
	case IIO_EV_INFO_PERIOD:
		if (chan->type == IIO_LIGHT) {
			time = chip->settings.int_time;
			persistence = chip->settings.als_persistence;

			/* ALS filter values are 1, 2, 3, 5, 10, 15, ..., 60 */
			if (persistence > 3)
				persistence = (persistence - 3) * 5;

			filter_delay = persistence * time;
			*val = filter_delay / 1000000;
			*val2 = filter_delay % 1000000;
			return IIO_VAL_INT_PLUS_MICRO;
		} else {
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int tsl2540_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	int data0, data1;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_LIGHT:
			data0 = tsl2540_read_reg(chip, TSL2540_VIS_CHANLO);
			data1 = tsl2540_read_reg(chip, TSL2540_VIS_CHANHI);
			if ((data0 < 0) || (data1 < 0))
				return -EIO;

			*val = data0 | (data1 << 8);
			return IIO_VAL_INT;
		case IIO_INTENSITY:
			data0 = tsl2540_read_reg(chip, TSL2540_IR_CHANLO);
			data1 = tsl2540_read_reg(chip, TSL2540_IR_CHANHI);
			if ((data0 < 0) || (data1 < 0))
				return -EIO;

			*val = data0 | (data1 << 8);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*val = chip->settings.als_gain;
		*val2 = chip->settings.als_gain_low;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_INT_TIME:
		*val = 0;
		*val2 = chip->settings.int_time;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int tsl2540_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	int again, againmax, againl = 1;
	int cfg1, cfg2, int_val, ret;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		switch (val) {
		case 0:
			if (val2 == 500000) { // gain=0.5
				again = 0;
				againmax = 0;
				againl = 0;
			} else {
				return -EINVAL;
			}
			break;
		case 1:
			again = 0;
			againmax = 0;
			break;
		case 4:
			again = 1;
			againmax = 0;
			break;
		case 16:
			again = 2;
			againmax = 0;
			break;
		case 67:
			again = 3;
			againmax = 0;
			break;
		case 140:
			again = 3;
			againmax = 1;
			break;
		default:
			return -EINVAL;
		}
		cfg1 = (again & TSL2540_CFG1_AGAIN_MASKS);
		cfg2 = ((againl << 2) | (againmax << 4))
			& (TSL2540_CFG2_AGAINMAX | TSL2540_CFG2_AGAINL);

		ret = tsl2540_write_reg(chip, TSL2540_CONFIG1, cfg1);
		if (ret < 0)
			return ret;
		ret = tsl2540_write_reg(chip, TSL2540_CONFIG2, cfg2);
		if (ret < 0)
			return ret;

		chip->settings.als_gain = val;
		chip->settings.als_gain_low = val2;
		break;
	case IIO_CHAN_INFO_INT_TIME:
		if (val != 0 || val2 < tsl2540_int_time_avail[1] ||
		    val2 > tsl2540_int_time_avail[5])
			return -EINVAL;

		int_val = (val2 / tsl2540_int_time_avail[3]) - 1;
		ret = tsl2540_write_reg(chip, TSL2540_ALS_TIME, int_val);
		if (ret < 0)
			return ret;

		chip->settings.int_time = (int_val + 1) * tsl2540_int_time_avail[3];
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static irqreturn_t tsl2540_event_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct tsl2540_chip *chip = iio_priv(indio_dev);
	s64 timestamp = iio_get_time_ns(indio_dev);
	int ret;

	ret = tsl2540_read_reg(chip, TSL2540_STATUS);
	if (ret < 0)
		return IRQ_HANDLED;

	if (ret & TSL2540_STA_AINT) {
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_LIGHT,
						    0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
			       timestamp);
	}
	/* The status bits will cleared automatically when the INT_READ_CLEAR bit set*/

	return IRQ_HANDLED;
}

static const struct iio_info tsl2540_device_info = {
	.read_raw = &tsl2540_read_raw,
	.read_avail = &tsl2540_read_avail,
	.write_raw = &tsl2540_write_raw,
	.read_event_value = &tsl2540_read_event_value,
	.write_event_value = &tsl2540_write_event_value,
	.read_event_config = &tsl2540_read_interrupt_config,
	.write_event_config = &tsl2540_write_interrupt_config,
};

static const struct iio_event_spec tsl2540_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_PERIOD) |
			BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec tsl2540_channels[] = {
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}, {
		.type = IIO_LIGHT,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_INT_TIME) |
			BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.info_mask_separate_available =
			BIT(IIO_CHAN_INFO_INT_TIME) |
			BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.event_spec = tsl2540_events,
		.num_event_specs = ARRAY_SIZE(tsl2540_events),
	}
};

static int tsl2540_probe(struct i2c_client *clientp,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct tsl2540_chip *chip;
	int ret;

	indio_dev = devm_iio_device_alloc(&clientp->dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	chip = iio_priv(indio_dev);
	chip->client = clientp;
	i2c_set_clientdata(clientp, indio_dev);

	usleep_range(TSL2540_BOOT_MIN_SLEEP_TIME, TSL2540_BOOT_MAX_SLEEP_TIME);

	ret = i2c_smbus_read_byte_data(chip->client, TSL2540_DEVICEID);
	if (ret < 0)
		return ret;

	if ((ret & TSL2540_DEVICE_ID_MASKS) != TSL2540_DEVICE_ID) {
		dev_info(&chip->client->dev,
			 "%s: i2c device found does not match expected id\n",
				__func__);
		return -EINVAL;
	}

	mutex_init(&chip->als_mutex);

	indio_dev->info = &tsl2540_device_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = chip->client->name;
	indio_dev->num_channels = ARRAY_SIZE(tsl2540_channels);
	indio_dev->channels = tsl2540_channels;
	ret = devm_request_threaded_irq(&clientp->dev, clientp->irq,
					NULL,
					&tsl2540_event_handler,
					IRQF_TRIGGER_FALLING |
					IRQF_ONESHOT,
					"TSL2540_event",
					indio_dev);
	if (ret) {
		dev_err(&clientp->dev,
			"%s: irq request failed\n", __func__);
		return ret;
	}

	memcpy(&chip->settings, &tsl2540_default_settings,
		sizeof(tsl2540_default_settings));
	ret = tsl2540_chip_init(indio_dev);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&clientp->dev,
					tsl2540_chip_off_action,
					indio_dev);
	if (ret < 0)
		return ret;

	return devm_iio_device_register(&clientp->dev, indio_dev);
}

static int tsl2540_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	return tsl2540_chip_off(indio_dev);
}

static int tsl2540_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	return tsl2540_chip_on(indio_dev);
}

static const struct i2c_device_id tsl2540_idtable[] = {
	{ "tsl2540", },
	{}
};

MODULE_DEVICE_TABLE(i2c, tsl2540_idtable);

static const struct of_device_id tsl2540_of_match[] = {
	{ .compatible = "ams,tsl2540" },
	{}
};
MODULE_DEVICE_TABLE(of, tsl2540_of_match);

static const struct dev_pm_ops tsl2540_pm_ops = {
	.suspend = tsl2540_suspend,
	.resume  = tsl2540_resume,
};

static struct i2c_driver tsl2540_driver = {
	.driver = {
		.name = "tsl2540",
		.of_match_table = tsl2540_of_match,
		.pm = &tsl2540_pm_ops,
	},
	.id_table = tsl2540_idtable,
	.probe = tsl2540_probe,
};

module_i2c_driver(tsl2540_driver);

MODULE_AUTHOR("Zhang Bo <bo.zhang@nxp.com>");
MODULE_DESCRIPTION("AMS tsl2540 ambient light sensor driver");
MODULE_LICENSE("GPL");
