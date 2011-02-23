 /*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file ltc3589-regulator.c
 * @brief This is the main file for the Linear PMIC regulator driver. i2c
 * should be providing the interface between the PMIC and the MCU.
 *
 * @ingroup PMIC_CORE
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/ltc3589.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mfd/ltc3589/core.h>
#include <mach/hardware.h>
#include <mach/common.h>

/* Register definitions */
#define	LTC3589_REG_IRSTAT		0x02
#define	LTC3589_REG_SCR1		0x07
#define	LTC3589_REG_OVEN		0x10
#define	LTC3589_REG_SCR2		0x12
#define	LTC3589_REG_PGSTAT		0x13
#define	LTC3589_REG_VCCR		0x20
#define	LTC3589_REG_CLIRQ		0x21
#define	LTC3589_REG_B1DTV1		0x23
#define	LTC3589_REG_B1DTV2		0x24
#define	LTC3589_REG_VRRCR		0x25
#define	LTC3589_REG_B2DTV1		0x26
#define	LTC3589_REG_B2DTV2		0x27
#define	LTC3589_REG_B3DTV1		0x29
#define	LTC3589_REG_B3DTV2		0x2A
#define	LTC3589_REG_L2DTV1		0x32
#define	LTC3589_REG_L2DTV2		0x33

/* SCR1 bitfields */
#define	LTC3589_SCR1_	BIT(7)
#define	LTC3589_PGOODZ_LOWBATTZ	BIT(6)
#define	LTC3589_PGOODZ_VDCDC1		BIT(5)
#define	LTC3589_PGOODZ_VDCDC2		BIT(4)
#define	LTC3589_PGOODZ_VDCDC3		BIT(3)
#define	LTC3589_PGOODZ_LDO2		BIT(2)
#define	LTC3589_PGOODZ_LDO1		BIT(1)

/* MASK bitfields */
#define	LTC3589_MASK_PWRFAILZ		BIT(7)
#define	LTC3589_MASK_LOWBATTZ		BIT(6)
#define	LTC3589_MASK_VDCDC1		BIT(5)
#define	LTC3589_MASK_VDCDC2		BIT(4)
#define	LTC3589_MASK_VDCDC3		BIT(3)
#define	LTC3589_MASK_LDO2		BIT(2)
#define	LTC3589_MASK_LDO1		BIT(1)

/* REG_CTRL bitfields */
#define LTC3589_REG_CTRL_VDCDC1_EN	BIT(5)
#define LTC3589_REG_CTRL_VDCDC2_EN	BIT(4)
#define LTC3589_REG_CTRL_VDCDC3_EN	BIT(3)
#define LTC3589_REG_CTRL_LDO2_EN	BIT(2)
#define LTC3589_REG_CTRL_LDO1_EN	BIT(1)

/* LDO_CTRL bitfields */
#define LTC3589_LDO_CTRL_LDOx_SHIFT(ldo_id)	((ldo_id)*4)
#define LTC3589_LDO_CTRL_LDOx_MASK(ldo_id)	(0xF0 >> ((ldo_id)*4))

/* Number of step-down converters available */
#define LTC3589_NUM_DCDC		4
/* Number of LDO voltage regulators  available */
#define LTC3589_NUM_LDO		4
/* Number of total regulators available */
#define LTC3589_NUM_REGULATOR	(LTC3589_NUM_DCDC + LTC3589_NUM_LDO)

/* DCDCs */
#define LTC3589_DCDC_1			0
#define LTC3589_DCDC_2			1
#define LTC3589_DCDC_3			2
#define LTC3589_DCDC_4			3
/* LDOs */
#define LTC3589_LDO_1			4
#define LTC3589_LDO_2			5
#define LTC3589_LDO_3			6
#define LTC3589_LDO_4			7
/* Resistorss */
#define LTC3589_LDO2_R1			180
#define LTC3589_LDO2_R2_TO1		232
#define LTC3589_LDO2_R2_TO2		191
#define LTC3589_SW1_R1			100
#define LTC3589_SW1_R2			180
#define LTC3589_SW2_R1			180
#define LTC3589_SW2_R2_TO1		232
#define LTC3589_SW2_R2_TO2		191
#define LTC3589_SW3_R1			270
#define LTC3589_SW3_R2			100

#define LTC3589_MAX_REG_ID		LTC3589_LDO_4

/* Supported voltage values for regulators */
static const u16 LDO4_VSEL_table[] = {
	2800, 2500, 1800, 3300,
};

struct i2c_client *ltc3589_client;

/* Regulator specific details */
struct ltc_info {
	const char *name;
	unsigned min_uV;
	unsigned max_uV;
	bool fixed;
	u8 table_len;
	const u16 *table;
};

/* PMIC details */
struct ltc_pmic {
	struct regulator_desc desc[LTC3589_NUM_REGULATOR];
	struct i2c_client *client;
	struct regulator_dev *rdev[LTC3589_NUM_REGULATOR];
	const struct ltc_info *info[LTC3589_NUM_REGULATOR];
	struct mutex io_lock;
};

static int ltc3589_ldo2_r2;
static int ltc3589_sw2_r2;

/*
 * LTC3589 Device IO
 */
static DEFINE_MUTEX(io_mutex);

static int ltc_3589_set_bits(struct ltc3589 *ltc, u8 reg, u8 mask)
{
	int err, data;

	mutex_lock(&io_mutex);

	data = ltc->read_dev(ltc, reg);
	if (data < 0) {
		dev_err(&ltc->i2c_client->dev,
			"Read from reg 0x%x failed\n", reg);
		err = data;
		goto out;
	}

	data |= mask;
	err = ltc->write_dev(ltc, reg, data);
	if (err)
		dev_err(&ltc->i2c_client->dev,
			"Write for reg 0x%x failed\n", reg);

out:
	mutex_unlock(&io_mutex);
	return err;
}

static int ltc_3589_clear_bits(struct ltc3589 *ltc, u8 reg, u8 mask)
{
	int err, data;

	mutex_lock(&io_mutex);

	data = ltc->read_dev(ltc, reg);
	if (data < 0) {
		dev_err(&ltc->i2c_client->dev,
			"Read from reg 0x%x failed\n", reg);
		err = data;
		goto out;
	}

	data &= ~mask;

	err = ltc->write_dev(ltc, reg, data);
	if (err)
		dev_err(&ltc->i2c_client->dev,
			"Write for reg 0x%x failed\n", reg);

out:
	mutex_unlock(&io_mutex);
	return err;

}

static int ltc_3589_reg_read(struct ltc3589 *ltc, u8 reg)
{
	int data;

	mutex_lock(&io_mutex);

	data = ltc->read_dev(ltc, reg);
	if (data < 0)
		dev_err(&ltc->i2c_client->dev,
			"Read from reg 0x%x failed\n", reg);

	mutex_unlock(&io_mutex);
	return data;
}

static int ltc_3589_reg_write(struct ltc3589 *ltc, u8 reg, u8 val)
{
	int err;

	mutex_lock(&io_mutex);

	err = ltc->write_dev(ltc, reg, val);
	if (err < 0)
		dev_err(&ltc->i2c_client->dev,
			"Write for reg 0x%x failed\n", reg);

	mutex_unlock(&io_mutex);
	return err;
}

int ltc3589_set_slew_rate(struct regulator_dev *dev, int rate)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int data, dcdc = rdev_get_id(dev);

	if (dcdc == LTC3589_DCDC_4 || dcdc == LTC3589_LDO_1 ||
		dcdc == LTC3589_LDO_3 || dcdc == LTC3589_LDO_4)
		return 0;

	if (rate > 0x3)
		return -EINVAL;

	data = ltc_3589_reg_read(ltc, LTC3589_REG_VRRCR);
	if (data < 0)
		return data;

	if (dcdc == LTC3589_LDO_2)
		dcdc = 3;

	data &= ~(3 << (2 * dcdc));
	data |= (3 << (2 * dcdc));

	ltc_3589_reg_write(ltc, LTC3589_REG_VRRCR, data);

	return 0;
}

static int ltc3589_dcdc_is_enabled(struct regulator_dev *dev)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int data, dcdc = rdev_get_id(dev);
	u8 shift;

	if (dcdc < LTC3589_DCDC_1 || dcdc > LTC3589_DCDC_4)
		return -EINVAL;

	shift = dcdc;
	data = ltc_3589_reg_read(ltc, LTC3589_REG_OVEN);

	if (data < 0)
		return data;
	else
		return (data & 1<<shift) ? 1 : 0;
}

static int ltc3589_ldo_is_enabled(struct regulator_dev *dev)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int data, ldo = rdev_get_id(dev);
	u8 shift;

	if (ldo < LTC3589_LDO_1 || ldo > LTC3589_LDO_4)
		return -EINVAL;

	if (ldo == LTC3589_LDO_1)
		return 1;

	shift = ldo - 1;
	data = ltc_3589_reg_read(ltc, LTC3589_REG_OVEN);

	if (data < 0)
		return data;
	else
		return (data & 1<<shift) ? 1 : 0;
}

static int ltc3589_dcdc_enable(struct regulator_dev *dev)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	u8 shift;

	if (dcdc < LTC3589_DCDC_1 || dcdc > LTC3589_DCDC_4)
		return -EINVAL;

	shift = dcdc;
	return ltc_3589_set_bits(ltc, LTC3589_REG_OVEN, 1 << shift);
}

static int ltc3589_dcdc_disable(struct regulator_dev *dev)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	u8 shift;

	if (dcdc < LTC3589_DCDC_1 || dcdc > LTC3589_DCDC_4)
		return -EINVAL;

	shift = dcdc;
	return ltc_3589_clear_bits(ltc, LTC3589_REG_OVEN, 1 << shift);
}

static int ltc3589_ldo_enable(struct regulator_dev *dev)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int ldo = rdev_get_id(dev);
	u8 shift;

	if (ldo < LTC3589_LDO_1 || ldo > LTC3589_LDO_4)
		return -EINVAL;

	if (ldo == LTC3589_LDO_1) {
		printk(KERN_ERR "LDO1 is always enabled\n");
		return 0;
	}

	shift = (ldo - 1);
	return ltc_3589_set_bits(ltc, LTC3589_REG_OVEN, 1 << shift);
}

static int ltc3589_ldo_disable(struct regulator_dev *dev)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int ldo = rdev_get_id(dev);
	u8 shift;

	if (ldo < LTC3589_LDO_1 || ldo > LTC3589_LDO_4)
		return -EINVAL;

	if (ldo == LTC3589_LDO_1) {
		printk(KERN_ERR "LDO1 can not be disabled\n");
		return 0;
	}

	shift = (ldo - 1);
	return ltc_3589_clear_bits(ltc, LTC3589_REG_OVEN, 1 << shift);
}

static int ltc3589_dcdc_set_mode(struct regulator_dev *dev,
				       unsigned int mode)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	u16 val;
	int data;

	if (dcdc < LTC3589_DCDC_1 || dcdc > LTC3589_DCDC_4)
		return -EINVAL;

	val = 1 << (dcdc - LTC3589_DCDC_1);
	data = ltc_3589_reg_read(ltc, LTC3589_REG_SCR1);
	if (data < 0)
		return data;

	data &= ~(3 << (2 * dcdc));

	switch (mode) {
	case REGULATOR_MODE_FAST:
		/* burst mode */
		data |= (1 << (2 * dcdc));
		break;
	case REGULATOR_MODE_NORMAL:
		/* active / pulse skipping */
		data |= (0 << (2 * dcdc));
		break;
	case REGULATOR_MODE_IDLE:
	case REGULATOR_MODE_STANDBY:
		/* force continuous mode */
		data |= (2 << (2 * dcdc));
	}

	return ltc_3589_reg_write(ltc, LTC3589_REG_SCR1, data);;
}

static unsigned int ltc3589_dcdc_get_mode(struct regulator_dev *dev)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	int data;

	data = ltc_3589_reg_read(ltc, LTC3589_REG_VCCR);
	if (data < 0)
		return data;

	if (dcdc == LTC3589_DCDC_4) {
		data >>= (2 * dcdc);
		data &= 0x01;

		if (data == 1)
			return REGULATOR_MODE_FAST;
		else
			return REGULATOR_MODE_NORMAL;
	} else {
		data >>= (2 * dcdc);
		data &= 0x03;

		switch (data) {
		case 0:
			return REGULATOR_MODE_NORMAL;
		case 1:
			return REGULATOR_MODE_FAST;
		case 2:
			return REGULATOR_MODE_IDLE;
		}
	}

	return 0;
}

static int ltc3589_dcdc_get_voltage(struct regulator_dev *dev)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int volt_reg, data, dcdc = rdev_get_id(dev);
	int shift, mask, r1, r2;
	int uV;

	r1 = 0;
	r2 = 1;

	if (dcdc < LTC3589_DCDC_1 || dcdc > LTC3589_DCDC_3)
		return -EINVAL;

	data = ltc_3589_reg_read(ltc, LTC3589_REG_VCCR);
	if (data < 0)
		return data;

	switch (dcdc) {
	case LTC3589_DCDC_1:
		r1 = LTC3589_SW1_R1;
		r2 = LTC3589_SW1_R2;
		shift = LTC3589_B1DTV2_REG1_REF_INPUT_V2_SHIFT;
		mask = LTC3589_B1DTV2_REG1_REF_INPUT_V2_MASK;
		if (data & LTC3589_VCCR_REG1_REF_SELECT)
			volt_reg = LTC3589_REG_B1DTV2;
		else
			volt_reg = LTC3589_REG_B1DTV1;
		break;
	case LTC3589_DCDC_2:
		r1 = LTC3589_SW2_R1;
		r2 = ltc3589_sw2_r2;
		shift = LTC3589_B2DTV2_REG2_REF_INPUT_V2_SHIFT;
		mask = LTC3589_B2DTV2_REG2_REF_INPUT_V2_MASK;
		if (data & LTC3589_VCCR_REG2_REF_SELECT)
			volt_reg = LTC3589_REG_B2DTV2;
		else
			volt_reg = LTC3589_REG_B2DTV1;
		break;
	case LTC3589_DCDC_3:
		r1 = LTC3589_SW3_R1;
		r2 = LTC3589_SW3_R2;
		shift = LTC3589_B3DTV2_REG3_REF_INPUT_V2_SHIFT;
		mask = LTC3589_B3DTV2_REG3_REF_INPUT_V2_MASK;
		if (data & LTC3589_VCCR_REG3_REF_SELECT)
			volt_reg = LTC3589_REG_B3DTV2;
		else
			volt_reg = LTC3589_REG_B3DTV1;
		break;
	default:
		return -EINVAL;
	}

	data = ltc_3589_reg_read(ltc, volt_reg);
	if (data < 0)
		return data;

	data >>= shift;
	data &= mask;
	uV = (((r1 + r2) * (362500 + data * 12500)) / r2);
	return uV;
}

static int ltc3589_dcdc_set_voltage(struct regulator_dev *dev,
				int min_uV, int max_uV)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	int volt_reg1;
	int volt_reg2;
	int shift, mask;
	int r1, r2;
	int dac_vol, data, slew_val;

	r1 = 0;
	r2 = 1;
	if (dcdc < LTC3589_DCDC_1 || dcdc > LTC3589_DCDC_3)
		return -EINVAL;

	switch (dcdc) {
	case LTC3589_DCDC_1:
		r1 = LTC3589_SW1_R1;
		r2 = LTC3589_SW1_R2;
		volt_reg1 = 0;
		volt_reg2 = LTC3589_REG_B1DTV1;
		slew_val = 1;
		shift = LTC3589_B1DTV2_REG1_REF_INPUT_V2_SHIFT;
		mask = LTC3589_B1DTV2_REG1_REF_INPUT_V2_MASK;
		break;
	case LTC3589_DCDC_2:
		r1 = LTC3589_SW2_R1;
		r2 = ltc3589_sw2_r2;
		volt_reg1 = 0;
		volt_reg2 = LTC3589_REG_B2DTV1;
		slew_val = 1;
		shift = LTC3589_B2DTV2_REG2_REF_INPUT_V2_SHIFT;
		mask = LTC3589_B2DTV2_REG2_REF_INPUT_V2_MASK;
		break;
	case LTC3589_DCDC_3:
		r1 = LTC3589_SW3_R1;
		r2 = LTC3589_SW3_R2;
		volt_reg1 = 0;
		volt_reg2 = LTC3589_REG_B3DTV1;
		slew_val = 1;
		shift = LTC3589_B3DTV2_REG3_REF_INPUT_V2_SHIFT;
		mask = LTC3589_B3DTV2_REG3_REF_INPUT_V2_MASK;
		break;
	default:
		return -EINVAL;
	}

	dac_vol = ((min_uV * r2 - 362500 * (r1 + r2)) / (12500 * (r1 + r2)));
	if ((dac_vol > 0x1F) || (dac_vol < 0))
		return -EINVAL;

	/* Set voltage */
	if (volt_reg1 != 0) {
		data = ltc_3589_reg_read(ltc, volt_reg1);
		if (data < 0)
			return data;

		data >>= shift;
		data &= ~mask;
		data |= dac_vol;
		ltc_3589_reg_write(ltc, volt_reg1, data);
	}

	if (volt_reg2 != 0) {
		data = ltc_3589_reg_read(ltc, volt_reg2);
		if (data < 0)
			return data;

		data >>= shift;
		data &= ~mask;
		data |= dac_vol;
		ltc_3589_reg_write(ltc, volt_reg2, data);
	}

	/* Set slew config */
	data = ltc_3589_reg_read(ltc, LTC3589_REG_VCCR);
	if (data < 0)
		return data;

	data &= ~(3 << (2 * dcdc));
	data |= (slew_val << (2 * dcdc));

	ltc_3589_reg_write(ltc, LTC3589_REG_VCCR, data);

	data = ltc_3589_reg_read(ltc, LTC3589_REG_SCR2);
	if (data < 0)
		return data;

	data &= ~(1 << dcdc);
	data |= (1 << dcdc);

	ltc_3589_reg_write(ltc, LTC3589_REG_SCR2, data);

	data = ltc_3589_reg_read(ltc, LTC3589_REG_OVEN);
	if (data < 0)
		return data;

	data &= ~(1 << dcdc);
	data |= (1 << dcdc);

	ltc_3589_reg_write(ltc, LTC3589_REG_OVEN, data);

	mdelay(10);

	return 0;
}

static int ltc3589_set_suspend_voltage(struct regulator_dev *dev,
						   int uV)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);
	int volt_reg1;
	int shift, mask;
	int r1, r2;
	int dac_vol, data;

	r1 = 0;
	r2 = 1;
	if (dcdc < LTC3589_DCDC_1 || dcdc > LTC3589_DCDC_3)
		return -EINVAL;

	switch (dcdc) {
	case LTC3589_DCDC_1:
		r1 = LTC3589_SW1_R1;
		r2 = LTC3589_SW1_R2;
		volt_reg1 = LTC3589_REG_B1DTV2;
		shift = LTC3589_B1DTV2_REG1_REF_INPUT_V2_SHIFT;
		mask = LTC3589_B1DTV2_REG1_REF_INPUT_V2_MASK;
		break;
	case LTC3589_DCDC_2:
		r1 = LTC3589_SW2_R1;
		r2 = ltc3589_sw2_r2;
		volt_reg1 = LTC3589_REG_B2DTV2;
		shift = LTC3589_B2DTV2_REG2_REF_INPUT_V2_SHIFT;
		mask = LTC3589_B2DTV2_REG2_REF_INPUT_V2_MASK;
		break;
	default:
		return 0;
	}

	dac_vol = ((uV * r2 - 362500 * (r1 + r2)) / (12500 * (r1 + r2)));
	if ((dac_vol > 0x1F) || (dac_vol < 0))
		return -EINVAL;

	data = ltc_3589_reg_read(ltc, volt_reg1);
	if (data < 0)
		return data;

	data >>= shift;
	data &= ~mask;
	data |= dac_vol;
	ltc_3589_reg_write(ltc, volt_reg1, data);
	return 0;
}

static int ltc3589_set_suspend_enable(struct regulator_dev *rdev)
{
	return 0;
}

static int ltc3589_set_suspend_disable(struct regulator_dev *rdev)
{
	return 0;
}

static int ltc3589_set_suspend_mode(struct regulator_dev *rdev,
	unsigned int mode)
{
	return 0;
}

static int ltc3589_ldo_get_voltage(struct regulator_dev *dev)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int data, ldo = rdev_get_id(dev);
	int uV;

	if (ldo < LTC3589_LDO_1 || ldo > LTC3589_LDO_4)
		return -EINVAL;

	if (ldo == LTC3589_LDO_1 || ldo == LTC3589_LDO_3)
		return -EINVAL;

	if (ldo == LTC3589_LDO_4) {
		data = ltc_3589_reg_read(ltc, LTC3589_REG_L2DTV2);
		if (data < 0)
			return data;

		data >>= LTC3589_L2DTV2_LDO4_OUTPUT_VOLTAGE_SHIFT;
		data &= LTC3589_L2DTV2_LDO4_OUTPUT_VOLTAGE_MASK;
		switch (data) {
		case 0:
			return 1800000;
		case 1:
			return 2500000;
		case 2:
			return 3000000;
		case 3:
			return 3300000;
		default:
			return -EINVAL;
		}
	} else {
		data = ltc_3589_reg_read(ltc, LTC3589_REG_VCCR);
		if (data < 0)
			return data;

		if (data & LTC3589_VCCR_LDO2_REF_SELECT)
			data = ltc_3589_reg_read(ltc, LTC3589_REG_L2DTV2);
		else
			data = ltc_3589_reg_read(ltc, LTC3589_REG_L2DTV1);

		if (data < 0)
			return data;

		data >>= LTC3589_L2DTV2_LDO2_REF_INPUT_V2_SHIFT;
		data &= LTC3589_L2DTV2_LDO2_REF_INPUT_V2_MASK;
		uV = (((LTC3589_LDO2_R1 + ltc3589_ldo2_r2)
			* (362500 + data * 12500)) / ltc3589_ldo2_r2);
		return uV;
	}

	return -EINVAL;
}

static int ltc3589_ldo_set_voltage(struct regulator_dev *dev,
				int min_uV, int max_uV)
{
	struct ltc3589 *ltc = rdev_get_drvdata(dev);
	int data, vsel, ldo = rdev_get_id(dev);

	if (ldo < LTC3589_LDO_1 || ldo > LTC3589_LDO_4)
		return -EINVAL;

	if (ldo == LTC3589_LDO_1 || ldo == LTC3589_LDO_3)
		return -EINVAL;

	if (ldo == LTC3589_LDO_4) {
		for (vsel = 0; vsel < 4; vsel++) {
			int mV = LDO4_VSEL_table[vsel];
			int uV = mV * 1000;

			/* Break at the first in-range value */
			if (min_uV <= uV && uV <= max_uV)
				break;
		}

		if (vsel == 4)
			return -EINVAL;

		data = ltc_3589_reg_read(ltc, LTC3589_REG_L2DTV2);
		if (data < 0)
			return data;

		data &= ~(0x3 << LTC3589_L2DTV2_LDO4_OUTPUT_VOLTAGE_SHIFT);
		data |= (vsel << LTC3589_L2DTV2_LDO4_OUTPUT_VOLTAGE_SHIFT);
		data &= ~(1 << LTC3589_L2DTV2_LDO4_CONTROL_MODE_SHIFT);
		data |= (1 << LTC3589_L2DTV2_LDO4_CONTROL_MODE_SHIFT);
		ltc_3589_reg_write(ltc, LTC3589_REG_L2DTV2, data);


		data = ltc_3589_reg_read(ltc, LTC3589_REG_SCR2);
		if (data < 0)
			return data;

		data &= ~(1 << LTC3589_SCR2_LDO4_STARTUP_SHIFT);
		data |= (1 << LTC3589_SCR2_LDO4_STARTUP_SHIFT);

		ltc_3589_reg_write(ltc, LTC3589_REG_SCR2, data);

		data = ltc_3589_reg_read(ltc, LTC3589_REG_OVEN);
		if (data < 0)
			return data;

		data &= ~(LTC3589_OVEN_EN_LDO4);
		data |= LTC3589_OVEN_EN_LDO4;

		ltc_3589_reg_write(ltc, LTC3589_REG_OVEN, data);

		mdelay(10);
	} else {
		int dac_vol;

		dac_vol = ((min_uV * ltc3589_ldo2_r2 -
			    362500 * (LTC3589_LDO2_R1 + ltc3589_ldo2_r2))
			    / (12500 * (LTC3589_LDO2_R1 + ltc3589_ldo2_r2)));
		if ((dac_vol > 0x1F) || (dac_vol < 0))
			return -EINVAL;

		data = ltc_3589_reg_read(ltc, LTC3589_REG_L2DTV1);
		if (data < 0)
			return data;

		data &= ~LTC3589_L2DTV2_LDO2_REF_INPUT_V2_MASK;
		data |= dac_vol;
		ltc_3589_reg_write(ltc, LTC3589_REG_L2DTV1, data);

		/* Set slew config */
		data = ltc_3589_reg_read(ltc, LTC3589_REG_VCCR);
		if (data < 0)
			return data;

		data &= ~(LTC3589_VCCR_LDO2_SLEW |
			  LTC3589_VCCR_LDO2_REF_SELECT);
		data |= LTC3589_VCCR_LDO2_SLEW;

		ltc_3589_reg_write(ltc, LTC3589_REG_VCCR, data);

		data = ltc_3589_reg_read(ltc, LTC3589_REG_SCR2);
		if (data < 0)
			return data;

		data &= ~(1 << LTC3589_SCR2_LDO2_STARTUP_SHIFT);
		data |= (1 << LTC3589_SCR2_LDO2_STARTUP_SHIFT);

		ltc_3589_reg_write(ltc, LTC3589_REG_SCR2, data);

		data = ltc_3589_reg_read(ltc, LTC3589_REG_OVEN);
		if (data < 0)
			return data;

		data &= ~(LTC3589_OVEN_EN_LDO2);
		data |= (LTC3589_OVEN_EN_LDO2);

		ltc_3589_reg_write(ltc, LTC3589_REG_OVEN, data);

		mdelay(10);
	}
	return 0;
}

static int ltc3589_ldo4_list_voltage(struct regulator_dev *dev,
					unsigned selector)
{
	int ldo = rdev_get_id(dev);

	if (ldo != LTC3589_LDO_4)
		return -EINVAL;

	if (selector >= 4)
		return -EINVAL;
	else
		return LDO4_VSEL_table[selector] * 1000;
}

/* Operations permitted on SWx */
static struct regulator_ops ltc3589_sw_ops = {
	.is_enabled = ltc3589_dcdc_is_enabled,
	.enable = ltc3589_dcdc_enable,
	.disable = ltc3589_dcdc_disable,
	.get_mode = ltc3589_dcdc_get_mode,
	.set_mode = ltc3589_dcdc_set_mode,
	.get_voltage = ltc3589_dcdc_get_voltage,
	.set_voltage = ltc3589_dcdc_set_voltage,
	.set_suspend_voltage = ltc3589_set_suspend_voltage,
	.set_suspend_enable = ltc3589_set_suspend_enable,
	.set_suspend_disable = ltc3589_set_suspend_disable,
	.set_suspend_mode = ltc3589_set_suspend_mode,
};

/* Operations permitted on SW4 Buck-Boost */
static struct regulator_ops ltc3589_sw4_ops = {
	.is_enabled = ltc3589_dcdc_is_enabled,
	.enable = ltc3589_dcdc_enable,
	.disable = ltc3589_dcdc_disable,
	.get_mode = ltc3589_dcdc_get_mode,
	.set_mode = ltc3589_dcdc_set_mode,
};

/* Operations permitted on LDO1_STBY, LDO3 */
static struct regulator_ops ltc3589_ldo13_ops = {
	.is_enabled = ltc3589_ldo_is_enabled,
	.enable = ltc3589_ldo_enable,
	.disable = ltc3589_ldo_disable,
};

/* Operations permitted on LDO2 */
static struct regulator_ops ltc3589_ldo2_ops = {
	.is_enabled = ltc3589_ldo_is_enabled,
	.enable = ltc3589_ldo_enable,
	.disable = ltc3589_ldo_disable,
	.get_voltage = ltc3589_ldo_get_voltage,
	.set_voltage = ltc3589_ldo_set_voltage,
};

/* Operations permitted on LDO4 */
static struct regulator_ops ltc3589_ldo4_ops = {
	.is_enabled = ltc3589_ldo_is_enabled,
	.enable = ltc3589_ldo_enable,
	.disable = ltc3589_ldo_disable,
	.get_voltage = ltc3589_ldo_get_voltage,
	.set_voltage = ltc3589_ldo_set_voltage,
	.list_voltage = ltc3589_ldo4_list_voltage,
};

static struct regulator_desc ltc3589_reg[] = {
	{
		.name = "SW1",
		.id = LTC3589_SW1,
		.ops = &ltc3589_sw_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 0x1F + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "SW2",
		.id = LTC3589_SW2,
		.ops = &ltc3589_sw_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 0x1F + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "SW3",
		.id = LTC3589_SW3,
		.ops = &ltc3589_sw_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 0x1F + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "SW4",
		.id = LTC3589_SW4,
		.ops = &ltc3589_sw4_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO1_STBY",
		.id = LTC3589_LDO1,
		.ops = &ltc3589_ldo13_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO2",
		.id = LTC3589_LDO2,
		.ops = &ltc3589_ldo2_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 0x1F + 1,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO3",
		.id = LTC3589_LDO3,
		.ops = &ltc3589_ldo13_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO4",
		.id = LTC3589_LDO4,
		.ops = &ltc3589_ldo4_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = ARRAY_SIZE(LDO4_VSEL_table),
		.owner = THIS_MODULE,
	},
};

static int ltc3589_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;

	if (pdev->id < LTC3589_DCDC_1 || pdev->id > LTC3589_LDO4)
		return -ENODEV;

	if (mx53_revision() >= IMX_CHIP_REVISION_2_0) {
		ltc3589_ldo2_r2 = LTC3589_LDO2_R2_TO2;
		ltc3589_sw2_r2 = LTC3589_SW2_R2_TO2;
	} else {
		ltc3589_ldo2_r2 = LTC3589_LDO2_R2_TO1;
		ltc3589_sw2_r2 = LTC3589_SW2_R2_TO1;
	}

	/* register regulator */
	rdev = regulator_register(&ltc3589_reg[pdev->id], &pdev->dev,
				  pdev->dev.platform_data,
				  dev_get_drvdata(&pdev->dev));
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev, "failed to register %s\n",
			ltc3589_reg[pdev->id].name);
			return PTR_ERR(rdev);
		}

	ltc3589_set_slew_rate(rdev, LTC3589_VRRCR_SLEW_RATE_7P00MV_US);

	return 0;
}

static int ltc3589_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;
}

int ltc3589_register_regulator(struct ltc3589 *ltc3589, int reg,
			      struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;

	if (ltc3589->pmic.pdev[reg])
		return -EBUSY;
	if (reg < LTC3589_DCDC_1 || reg > LTC3589_LDO_4)
		return -ENODEV;
	pdev = platform_device_alloc("ltc3589-regulator", reg);
	if (!pdev)
		return -ENOMEM;
	ltc3589->pmic.pdev[reg] = pdev;

	initdata->driver_data = ltc3589;

	pdev->dev.platform_data = initdata;
	pdev->dev.parent = ltc3589->dev;
	platform_set_drvdata(pdev, ltc3589);

	ret = platform_device_add(pdev);

	if (ret != 0) {
		dev_err(ltc3589->dev, "Failed to register regulator %d: %d\n",
			reg, ret);
		platform_device_del(pdev);
		ltc3589->pmic.pdev[reg] = NULL;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(ltc3589_register_regulator);

static struct platform_driver ltc3589_regulator_driver = {
	.driver = {
		   .name = "ltc3589-regulator",
	},
	.probe = ltc3589_regulator_probe,
	.remove = ltc3589_regulator_remove,
};

static int __init ltc3589_regulator_init(void)
{
	return platform_driver_register(&ltc3589_regulator_driver);

}
subsys_initcall_sync(ltc3589_regulator_init);

static void __exit ltc3589_regulator_exit(void)
{
	platform_driver_unregister(&ltc3589_regulator_driver);
}
module_exit(ltc3589_regulator_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("LTC3589 Regulator driver");
MODULE_LICENSE("GPL");
