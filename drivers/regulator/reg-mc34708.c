/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/mc34708/core.h>
#include <linux/platform_device.h>
#include <linux/pmic_status.h>
#include <linux/mfd/mc34708/mc34708.h>
#include <linux/pmic_external.h>

/*
 * Convenience conversion.
 * Here atm, maybe there is somewhere better for this.
 */
#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

/*!
 * @enum regulator_voltage_vpll
 * @brief PMIC regulator VPLL output voltage.
 */
enum {
	VPLL_1_2V = 0,
	VPLL_1_25V,
	VPLL_1_5V,
	VPLL_1_8V,
};

/*!
 * @enum regulator_voltage_vusb2
 * @brief PMIC regulator VUSB2 output voltage.
 */
enum {
	VUSB2_2_5V = 0,
	VUSB2_2_6V,
	VUSB2_2_75V,
	VUSB2_3V,
};

/*!
 * @enum regulator_voltage_vdac
 * @brief PMIC regulator VDAC output voltage.
 */
enum {
	VDAC_2_5V = 0,
	VDAC_2_6V,
	VDAC_2_7V,
	VDAC_2_775V,
} regulator_voltage_vdac;

/*!
 * @enum regulator_voltage_vgen1
 * @brief PMIC regulator VGEN1 output voltage.
 */
enum {
	VGEN1_1_2V = 0,
	VGEN1_1_25V,
	VGEN1_1_3V,
	VGEN1_1_35V,
	VGEN1_1_4V,
	VGEN1_1_45V,
	VGEN1_1_5V,
	VGEN1_1_55V,
};

/*!
 * @enum regulator_voltage_vgen2
 * @brief mc34708 PMIC regulator VGEN2 output voltage.
 */
enum {
	VGEN2_2_5V = 0,
	VGEN2_2_7V,
	VGEN2_2_8V,
	VGEN2_2_9V,
	VGEN2_3V,
	VGEN2_3_1V,
	VGEN2_3_15V,
	VGEN2_3_3V,
};

/*!
     * The \b TPmicDVSTransitionSpeed enum defines the rate with which the
     * voltage transition occurs.
     */
enum {
	ESysDependent,
	E25mVEach4us,
	E25mVEach8us,
	E25mvEach16us
} DVS_transition_speed;

/*
 * Reg Regulator Mode 0
 */

#define VGEN1_EN_LSH	0
#define VGEN1_EN_WID	1
#define VGEN1_EN_ENABLE	1
#define VGEN1_EN_DISABLE	0
#define VDAC_EN_LSH	4
#define VDAC_EN_WID	1
#define VDAC_EN_ENABLE	1
#define VDAC_EN_DISABLE	0
#define VREFDDR_EN_LSH	10
#define VREFDDR_EN_WID	1
#define VREFDDR_EN_ENABLE	1
#define VREFDDR_EN_DISABLE	0
#define VGEN2_EN_LSH	12
#define VGEN2_EN_WID	1
#define VGEN2_EN_ENABLE	1
#define VGEN2_EN_DISABLE	0
#define VPLL_EN_LSH	15
#define VPLL_EN_WID	1
#define VPLL_EN_ENABLE	1
#define VPLL_EN_DISABLE	0
#define VUSB2_EN_LSH	18
#define VUSB2_EN_WID	1
#define VUSB2_EN_ENABLE	1
#define VUSB2_EN_DISABLE	0
#define VUSB_EN_LSH	3
#define VUSB_EN_WID	1
#define VUSB_EN_ENABLE	1
#define VUSB_EN_DISABLE	0

#define VUSBSEL_LSH	2
#define VUSBSEL_WID	1
#define VUSBSEL_ENABLE	1
#define VUSBSEL_DISABLE	0

/*
 * Reg Regulator Setting 0
 */
#define VGEN1_LSH		0
#define VGEN1_WID		3
#define VDAC_LSH		4
#define VDAC_WID		2
#define VGEN2_LSH		6
#define VGEN2_WID		3
#define VPLL_LSH		9
#define VPLL_WID		2
#define VUSB2_LSH		11
#define VUSB2_WID		2

/*
 * Reg Switcher 1 A/B
 */
#define SW1A_LSH		0
#define SW1A_WID		6
#define SW1A_STDBY_LSH	6
#define SW1A_STDBY_WID	6

#define SW1B_LSH		12
#define SW1B_WID		6
#define SW1B_STDBY_LSH	18
#define SW1B_STDBY_WID	6

/*
 * Reg Switcher 2&3
 */
#define SW2_LSH		0
#define SW2_WID		6
#define SW2_STDBY_LSH	6
#define SW2_STDBY_WID	6
#define SW3_LSH		12
#define SW3_WID		5
#define SW3_STDBY_LSH	18
#define SW3_STDBY_WID	5

/*
 * Reg Switcher 4 A/B
 */
#define SW4A_LSH		0
#define SW4A_WID		5
#define SW4A_STDBY_LSH	5
#define SW4A_STDBY_WID	5
#define SW4AHI_LSH		10
#define SW4AHI_WID		2

#define SW4B_LSH		12
#define SW4B_WID		5
#define SW4B_STDBY_LSH	17
#define SW4B_STDBY_WID	5
#define SW4BHI_LSH		22
#define SW4BHI_WID		2

/*
 * Reg Switcher 5
 */
#define SW5_LSH		0
#define SW5_WID		5
#define SW5_STDBY_LSH	10
#define SW5_STDBY_WID	5

#define SWXHI_LSH	23
#define SWXHI_WID	1
#define SWXHI_ON	1
#define SWXHI_OFF	0

/*
 * Switcher mode configuration
 */
#define SW_MODE_SYNC_RECT_EN	0
#define SW_MODE_PULSE_NO_SKIP_EN	1
#define SW_MODE_PULSE_SKIP_EN	2
#define SW_MODE_LOW_POWER_EN	3

#define dvs_speed E25mvEach16us

enum {
	SW1_MIN_UV = 650000,
	SW1_MAX_UV = 1437500,
	SW1_STEP_UV = 12500,
};

enum {
	SW2_MIN_UV = 650000,
	SW2_MAX_UV = 1437500,
	SW2_STEP_UV = 12500,
};

enum {
	SW3_MIN_MV = 650,
	SW3_MAX_MV = 1425,
	SW3_STEP_MV = 25,
};

enum {
	SW4_MIN_MV = 1200,
	SW4_MAX_MV = 1975,
	SW4_STEP_MV = 25,
	SW4_HI_2500_MV = 2500,
	SW4_HI_3150_MV = 3150,
	SW4_HI_3300_MV = 3300,
};

enum {
	SW5_MIN_MV = 1200,
	SW5_MAX_MV = 1975,
	SW5_STEP_MV = 25,
};

enum {
	VGEN1_MIN_MV = 1200,
	VGEN1_MAX_MV = 1550,
	VGEN1_STEP_MV = 50,
};

static unsigned int mv_to_bit_value(int mv, int min_mv, int max_mv, int step_mv)
{
	return ((unsigned int)(((mv < min_mv) ?
				min_mv : (mv > max_mv ? max_mv : mv)
				- min_mv) / step_mv));
}

static int bit_value_to_mv(unsigned int val, int min_mv, int step_mv)
{
	return ((unsigned int)val) * step_mv + min_mv;
}

static unsigned int uv_to_bit_value(int uv, int min_uv, int max_uv, int step_uv)
{
	return (unsigned int)(((uv < min_uv) ?
			       min_uv : (uv > max_uv ? max_uv : uv)
			       - min_uv) / step_uv);
}

static int bit_value_to_uv(unsigned int val, int min_uv, int step_uv)
{
	return ((unsigned int)val) * step_uv + min_uv;
}

static int mc34708_get_voltage_value(int sw, int mV)
{
	int voltage;

	switch (sw) {
	case MC34708_SW1A:
	case MC34708_SW1B:
	case MC34708_SW2:
		if (mV < 650)
			mV = 650;
		if (mV > 1437)
			mV = 1437;
		voltage = (mV - 650) * 10 / 125;
		break;
	case MC34708_SW3:
		if (mV < 650)
			mV = 650;
		if (mV > 1425)
			mV = 1425;
		voltage = (mV - 650) / 25;
		break;

	default:
		return -EINVAL;
	}

	return voltage;
}

static int mc34708_vpll_set_voltage(struct regulator_dev *reg,
				    int minuV, int uV)
{
	unsigned int register_val = 0, register_mask = 0, register1 = 0;
	int voltage, mV = uV / 1000;

	if (mV < 1250)
		voltage = VPLL_1_2V;
	else if (mV < 1500)
		voltage = VPLL_1_25V;
	else if (mV < 1800)
		voltage = VPLL_1_5V;
	else
		voltage = VPLL_1_8V;

	register_val = BITFVAL(VPLL, voltage);
	register_mask = BITFMASK(VPLL);
	register1 = MC34708_REG_REGULATOR_SETTING0;

	return pmic_write_reg(register1, register_val, register_mask);
}

static int mc34708_vpll_get_voltage(struct regulator_dev *reg)
{
	unsigned int register_val = 0;
	int voltage = 0, mV = 0;

	CHECK_ERROR(pmic_read_reg(MC34708_REG_REGULATOR_SETTING0,
				  &register_val, PMIC_ALL_BITS));
	voltage = BITFEXT(register_val, VPLL);

	switch (voltage) {
	case VPLL_1_2V:
		mV = 1200;
		break;
	case VPLL_1_25V:
		mV = 1250;
		break;
	case VPLL_1_5V:
		mV = 1500;
		break;
	case VPLL_1_8V:
		mV = 1800;
		break;
	default:
		return -EINVAL;
	}

	return mV * 1000;
}

static int mc34708_regulator_is_enabled(struct regulator_dev *rdev)
{
	unsigned int register1;
	unsigned int register_mask;
	int id = rdev_get_id(rdev);
	unsigned int register_val = 0;

	switch (id) {
	case MC34708_SWBST:
		register_mask = BITFMASK(VUSBSEL);
		register1 = MC34708_REG_REGULATOR_MODE0;
		break;
	case MC34708_VUSB:
		register_mask = BITFMASK(VUSB_EN);
		register1 = MC34708_REG_REGULATOR_MODE0;
		break;
	default:
		return 1;
	}
	CHECK_ERROR(pmic_read_reg(register1, &register_val, register_mask));
	return (register_val != 0);
}

static int mc34708_vusb2_set_voltage(struct regulator_dev *reg,
				     int minuV, int uV)
{
	unsigned int register_val = 0, register_mask = 0;
	unsigned int register1;
	int voltage, mV = uV / 1000;

	if (mV < 2600)
		voltage = VUSB2_2_5V;
	else if (mV < 2750)
		voltage = VUSB2_2_6V;
	else if (mV < 3000)
		voltage = VUSB2_2_75V;
	else
		voltage = VUSB2_3V;

	register_val = BITFVAL(VUSB2, voltage);
	register_mask = BITFMASK(VUSB2);
	register1 = MC34708_REG_REGULATOR_SETTING0;

	return pmic_write_reg(register1, register_val, register_mask);
}

static int mc34708_vusb2_get_voltage(struct regulator_dev *reg)
{
	unsigned int register_val = 0;
	int voltage = 0, mV = 0;

	CHECK_ERROR(pmic_read_reg(MC34708_REG_REGULATOR_SETTING0,
				  &register_val, PMIC_ALL_BITS));
	voltage = BITFEXT(register_val, VUSB2);

	switch (voltage) {
	case VUSB2_2_5V:
		mV = 2500;
		break;
	case VUSB2_2_6V:
		mV = 2600;
		break;
	case VUSB2_2_75V:
		mV = 2750;
		break;
	case VUSB2_3V:
		mV = 3000;
		break;
	default:
		return -EINVAL;
	}

	return mV * 1000;
}

static int mc34708_vgen1_set_voltage(struct regulator_dev *reg,
				     int minuV, int uV)
{
	unsigned int register_val = 0, register_mask = 0;
	unsigned int register1;
	int voltage, mV = uV / 1000;

	voltage = mv_to_bit_value(mV, VGEN1_MIN_MV,
				  VGEN1_MAX_MV, VGEN1_STEP_MV);
	register_val = BITFVAL(VGEN1, voltage);
	register_mask = BITFMASK(VGEN1);
	register1 = MC34708_REG_REGULATOR_SETTING0;

	return pmic_write_reg(register1, register_val, register_mask);
}

static int mc34708_vgen1_get_voltage(struct regulator_dev *reg)
{
	unsigned int register_val = 0;
	int voltage = 0, mV = 0;

	CHECK_ERROR(pmic_read_reg(MC34708_REG_REGULATOR_SETTING0,
				  &register_val, PMIC_ALL_BITS));
	voltage = BITFEXT(register_val, VGEN1);
	mV = bit_value_to_mv(voltage, VGEN1_MIN_MV, VGEN1_STEP_MV);

	return mV * 1000;
}

static int mc34708_ldo_enable(struct regulator_dev *reg)
{
	unsigned int register_val = 0, register_mask = 0;
	unsigned int register1;
	int id = rdev_get_id(reg);

	switch (id) {
	case MC34708_VREFDDR:
		register_val = BITFVAL(VREFDDR_EN, VREFDDR_EN_ENABLE);
		register_mask = BITFMASK(VREFDDR_EN);
		break;
	case MC34708_VUSB:
		register_val = BITFVAL(VUSB_EN, VUSB_EN_ENABLE);
		register_mask = BITFMASK(VUSB_EN);
		break;
	case MC34708_VUSB2:
		register_val = BITFVAL(VUSB2_EN, VUSB2_EN_ENABLE);
		register_mask = BITFMASK(VUSB2_EN);
		break;
	case MC34708_VDAC:
		register_val = BITFVAL(VDAC_EN, VDAC_EN_ENABLE);
		register_mask = BITFMASK(VDAC_EN);
		break;
	case MC34708_VGEN1:
		register_val = BITFVAL(VGEN1_EN, VGEN1_EN_ENABLE);
		register_mask = BITFMASK(VGEN1_EN);
		break;
	case MC34708_VGEN2:
		register_val = BITFVAL(VGEN2_EN, VGEN2_EN_ENABLE);
		register_mask = BITFMASK(VGEN2_EN);
		break;
	default:
		return -EINVAL;
	}
	register1 = MC34708_REG_REGULATOR_MODE0;

	return pmic_write_reg(register1, register_val, register_mask);
}

static int mc34708_ldo_disable(struct regulator_dev *reg)
{
	unsigned int register_val = 0, register_mask = 0;
	unsigned int register1;
	int id = rdev_get_id(reg);
	switch (id) {
	case MC34708_SWBST:
		register_val = BITFVAL(VUSBSEL, VUSBSEL_DISABLE);
		register_mask = BITFMASK(VUSBSEL);
		break;
	case MC34708_VREFDDR:
		register_val = BITFVAL(VREFDDR_EN, VREFDDR_EN_DISABLE);
		register_mask = BITFMASK(VREFDDR_EN);
		break;
	case MC34708_VUSB:
		register_val = BITFVAL(VUSB_EN, VUSB_EN_DISABLE);
		register_mask = BITFMASK(VUSB_EN);
		break;
	case MC34708_VUSB2:
		register_val = BITFVAL(VUSB2_EN, VUSB2_EN_DISABLE);
		register_mask = BITFMASK(VUSB2_EN);
		break;
	case MC34708_VDAC:
		register_val = BITFVAL(VDAC_EN, VDAC_EN_DISABLE);
		register_mask = BITFMASK(VDAC_EN);
		break;
	case MC34708_VGEN1:
		register_val = BITFVAL(VGEN1_EN, VGEN1_EN_DISABLE);
		register_mask = BITFMASK(VGEN1_EN);
		break;
	case MC34708_VGEN2:
		register_val = BITFVAL(VGEN2_EN, VGEN2_EN_DISABLE);
		register_mask = BITFMASK(VGEN2_EN);
		break;
	default:
		return -EINVAL;
	}
	register1 = MC34708_REG_REGULATOR_MODE0;
	return pmic_write_reg(register1, register_val, register_mask);
}

static int mc34708_vdac_set_voltage(struct regulator_dev *reg,
				    int minuV, int uV)
{
	unsigned int register_val = 0, register_mask = 0;
	unsigned int register1;
	int voltage, mV = uV / 1000;

	if (mV < 2600)
		voltage = VDAC_2_5V;
	else if (mV < 2700)
		voltage = VDAC_2_6V;
	else if (mV < 2775)
		voltage = VDAC_2_7V;
	else
		voltage = VDAC_2_775V;

	register_val = BITFVAL(VDAC, voltage);
	register_mask = BITFMASK(VDAC);
	register1 = MC34708_REG_REGULATOR_SETTING0;

	return pmic_write_reg(register1, register_val, register_mask);
}

static int mc34708_vdac_get_voltage(struct regulator_dev *reg)
{
	unsigned int register_val = 0;
	int voltage = 0, mV = 0;

	CHECK_ERROR(pmic_read_reg(MC34708_REG_REGULATOR_SETTING0,
				  &register_val, PMIC_ALL_BITS));
	voltage = BITFEXT(register_val, VDAC);

	switch (voltage) {
	case VDAC_2_5V:
		mV = 2500;
		break;
	case VDAC_2_6V:
		mV = 2600;
		break;
	case VDAC_2_7V:
		mV = 2700;
		break;
	case VDAC_2_775V:
		mV = 2775;
		break;
	default:
		return -EINVAL;
	}

	return mV * 1000;
}

static int mc34708_vgen2_set_voltage(struct regulator_dev *reg,
				     int minuV, int uV)
{
	unsigned int register_val = 0, register_mask = 0;
	unsigned int register1;
	int voltage, mV = uV / 1000;

	if (mV < 2700)
		voltage = VGEN2_2_5V;
	else if (mV < 2800)
		voltage = VGEN2_2_7V;
	else if (mV < 2900)
		voltage = VGEN2_2_8V;
	else if (mV < 3000)
		voltage = VGEN2_2_9V;
	else if (mV < 3100)
		voltage = VGEN2_3V;
	else if (mV < 3150)
		voltage = VGEN2_3_1V;
	else if (mV < 3300)
		voltage = VGEN2_3_15V;
	else
		voltage = VGEN2_3_3V;

	register1 = MC34708_REG_REGULATOR_SETTING0;

	return pmic_write_reg(register1, register_val, register_mask);
}

static int mc34708_vgen2_get_voltage(struct regulator_dev *reg)
{
	unsigned int register_val = 0;
	int voltage = 0, mV = 0;

	CHECK_ERROR(pmic_read_reg(MC34708_REG_REGULATOR_SETTING0,
				  &register_val, PMIC_ALL_BITS));

	voltage = BITFEXT(register_val, VGEN2);

	switch (voltage) {
	case VGEN2_2_5V:
		mV = 2500;
		break;
	case VGEN2_2_7V:
		mV = 2700;
		break;
	case VGEN2_2_8V:
		mV = 2800;
		break;
	case VGEN2_2_9V:
		mV = 2900;
		break;
	case VGEN2_3V:
		mV = 3000;
		break;
	case VGEN2_3_1V:
		mV = 3100;
		break;
	case VGEN2_3_15V:
		mV = 3150;
		break;
	case VGEN2_3_3V:
		mV = 3300;
		break;
	default:
		return -EINVAL;
	}

	return mV * 1000;
}

static int mc34708_sw_set_normal_voltage(struct regulator_dev *reg, int minuV,
					 int uV)
{
	unsigned int register_val = 0, register_mask = 0, register1 = 0;
	int voltage, mV = uV / 1000;
	int id = rdev_get_id(reg);

	switch (id) {
	case MC34708_SW1A:
		voltage =
			uv_to_bit_value(mV * 1000, SW1_MIN_UV, SW1_MAX_UV,
				    SW1_STEP_UV);
		register_val = BITFVAL(SW1A, voltage);
		register_mask = BITFMASK(SW1A);
		register1 = MC34708_REG_SW1AB;
		break;
	case MC34708_SW1B:
		voltage =
			uv_to_bit_value(mV * 1000, SW1_MIN_UV, SW1_MAX_UV,
				    SW1_STEP_UV);
		register_val = BITFVAL(SW1B, voltage);
		register_mask = BITFMASK(SW1B);
		register1 = MC34708_REG_SW1AB;
		break;
	case MC34708_SW2:
		voltage =
			uv_to_bit_value(mV * 1000, SW2_MIN_UV, SW2_MAX_UV,
					SW2_STEP_UV);
		register_val = BITFVAL(SW2, voltage);
		register_mask = BITFMASK(SW2);
		register1 = MC34708_REG_SW2_3;
		break;
	case MC34708_SW3:
		voltage =
			mv_to_bit_value(mV, SW3_MIN_MV, SW3_MAX_MV,
					SW3_STEP_MV);
		register_val = BITFVAL(SW3, voltage);
		register_mask = BITFMASK(SW3);
		register1 = MC34708_REG_SW2_3;
		break;
	case MC34708_SW4A:
		if (mV < SW4_HI_2500_MV) {
			voltage =
			    mv_to_bit_value(mV, SW4_MIN_MV, SW4_MAX_MV,
					    SW4_STEP_MV);
			register_val =
			    BITFVAL(SW4A, voltage) | BITFVAL(SW4AHI, 0);
			register_mask = BITFMASK(SW4A) | BITFMASK(SW4AHI);
			register1 = MC34708_REG_SW4AB;
		} else {
			if (mV < SW4_HI_3150_MV)
				register_val = BITFVAL(SW4AHI, 1);
			else if (mV < SW4_HI_3300_MV)
				register_val = BITFVAL(SW4AHI, 2);
			else
				register_val = BITFVAL(SW4AHI, 3);
			register_mask = BITFMASK(SW4AHI);
			register1 = MC34708_REG_SW4AB;
		}
		break;
	case MC34708_SW4B:
		if (mV < SW4_HI_2500_MV) {
			voltage =
			    mv_to_bit_value(mV, SW4_MIN_MV, SW4_MAX_MV,
					    SW4_STEP_MV);
			register_val =
			    BITFVAL(SW4B, voltage) | BITFVAL(SW4BHI, 0);
			register_mask = BITFMASK(SW4B) | BITFMASK(SW4BHI);
			register1 = MC34708_REG_SW4AB;
		} else {
			if (mV < SW4_HI_3150_MV)
				register_val = BITFVAL(SW4BHI, 1);
			else if (mV < SW4_HI_3300_MV)
				register_val = BITFVAL(SW4BHI, 2);
			else
				register_val = BITFVAL(SW4BHI, 3);
			register_mask = BITFMASK(SW4BHI);
			register1 = MC34708_REG_SW4AB;
		}
		break;
	case MC34708_SW5:
		voltage =
		    mv_to_bit_value(mV, SW5_MIN_MV, SW5_MAX_MV, SW5_STEP_MV);
		register_val = BITFVAL(SW5, voltage);
		register_mask = BITFMASK(SW5);
		register1 = MC34708_REG_SW5;
		break;
	default:
		break;
	}

	return pmic_write_reg(register1, register_val, register_mask);
}

static int mc34708_sw_get_normal_voltage(struct regulator_dev *reg)
{
	unsigned int register_val = 0;
	int voltage = 0, mV = 0;
	int id = rdev_get_id(reg);

	switch (id) {
	case MC34708_SW1A:
		CHECK_ERROR(pmic_read_reg(MC34708_REG_SW1AB,
					  &register_val, PMIC_ALL_BITS));
		voltage = BITFEXT(register_val, SW1A);
		mV = bit_value_to_uv(voltage, SW1_MIN_UV, SW1_STEP_UV) / 1000;
		break;
	case MC34708_SW1B:
		CHECK_ERROR(pmic_read_reg(MC34708_REG_SW1AB,
					  &register_val, PMIC_ALL_BITS));
		voltage = BITFEXT(register_val, SW1B);
		mV = bit_value_to_uv(voltage, SW1_MIN_UV, SW1_STEP_UV) / 1000;
		break;
	case MC34708_SW2:
		CHECK_ERROR(pmic_read_reg(MC34708_REG_SW2_3,
					  &register_val, PMIC_ALL_BITS));
		voltage = BITFEXT(register_val, SW2);
		mV = bit_value_to_uv(voltage, SW2_MIN_UV, SW2_STEP_UV) / 1000;
		break;
	case MC34708_SW3:
		CHECK_ERROR(pmic_read_reg(MC34708_REG_SW2_3,
					  &register_val, PMIC_ALL_BITS));
		voltage = BITFEXT(register_val, SW3);
		mV = bit_value_to_mv(voltage, SW3_MIN_MV, SW3_STEP_MV);
		break;
	case MC34708_SW4A:
		CHECK_ERROR(pmic_read_reg(MC34708_REG_SW4AB,
					  &register_val, PMIC_ALL_BITS));
		voltage = BITFEXT(register_val, SW4AHI);
		if (voltage == 0) {
			voltage = BITFEXT(register_val, SW4A);
			mV = bit_value_to_mv(voltage, SW4_MIN_MV, SW4_STEP_MV);
		} else if (voltage == 1)
			mV = SW4_HI_2500_MV;
		else if (voltage == 2)
			mV = SW4_HI_3150_MV;
		else
			mV = SW4_HI_3300_MV;
		break;
	case MC34708_SW4B:
		CHECK_ERROR(pmic_read_reg(MC34708_REG_SW4AB,
					  &register_val, PMIC_ALL_BITS));
		voltage = BITFEXT(register_val, SW4BHI);
		if (voltage == 0) {
			voltage = BITFEXT(register_val, SW4B);
			mV = bit_value_to_mv(voltage, SW4_MIN_MV, SW4_STEP_MV);
		} else if (voltage == 1)
			mV = SW4_HI_2500_MV;
		else if (voltage == 2)
			mV = SW4_HI_3150_MV;
		else
			mV = SW4_HI_3300_MV;
		break;
	case MC34708_SW5:
		CHECK_ERROR(pmic_read_reg(MC34708_REG_SW5,
					  &register_val, PMIC_ALL_BITS));
		voltage = BITFEXT(register_val, SW5);
		mV = bit_value_to_mv(voltage, SW5_MIN_MV, SW5_STEP_MV);
		break;
	default:
		break;
	}
	if (mV == 0)
		return -EINVAL;
	else
		return mV * 1000;
}

static int mc34708_sw_normal_enable(struct regulator_dev *reg)
{
	return 0;
}

static int mc34708_sw_normal_disable(struct regulator_dev *reg)
{
	return 0;
}

static int mc34708_sw_stby_enable(struct regulator_dev *reg)
{
	return 0;
}

static int mc34708_sw_stby_disable(struct regulator_dev *reg)
{
	return 0;
}

static int mc34708_sw_set_stby_voltage(struct regulator_dev *reg, int uV)
{
	unsigned int register_val = 0, register_mask = 0, register_valtest = 0;
	unsigned int register1 = 0;

	int voltage, mV = uV / 1000;
	int sw = rdev_get_id(reg);

	voltage = mc34708_get_voltage_value(sw, mV);

	switch (sw) {
	case MC34708_SW1A:
		register1 = REG_MC34708_SW_1_A_B;
		register_val = BITFVAL(SW1A_STDBY, voltage);
		register_mask = BITFMASK(SW1A_STDBY);
		break;
	case MC34708_SW1B:
		register1 = REG_MC34708_SW_1_A_B;
		register_val = BITFVAL(SW1B_STDBY, voltage);
		register_mask = BITFMASK(SW1B_STDBY);
		break;
	case MC34708_SW2:
		register1 = REG_MC34708_SW_2_3;
		register_val = BITFVAL(SW2_STDBY, voltage);
		register_mask = BITFMASK(SW2_STDBY);
		break;
	case MC34708_SW3:
		register1 = REG_MC34708_SW_2_3;
		register_val = BITFVAL(SW3_STDBY, voltage);
		register_mask = BITFMASK(SW3_STDBY);
		break;
	case MC34708_SW4A:
		register1 = REG_MC34708_SW_4_A_B;
		register_val = BITFVAL(SW4A_STDBY, voltage);
		register_mask = BITFMASK(SW4A_STDBY);
		break;
	case MC34708_SW4B:
		register1 = REG_MC34708_SW_4_A_B;
		register_val = BITFVAL(SW4B_STDBY, voltage);
		register_mask = BITFMASK(SW4B_STDBY);
		break;
	case MC34708_SW5:
		register1 = REG_MC34708_SW_5;
		register_val = BITFVAL(SW5_STDBY, voltage);
		register_mask = BITFMASK(SW5_STDBY);
		break;
	default:
		return -EINVAL;
	}

	return pmic_write_reg(register1, register_val, register_mask);
}

static int mc34708_sw_set_normal_mode(struct regulator_dev *reg,
				      unsigned int mode)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int register1 = 0;
	unsigned int l_mode;
	int sw = rdev_get_id(reg);

	switch (mode) {
	case REGULATOR_MODE_FAST:
		/* SYNC RECT mode */
		l_mode = SW_MODE_SYNC_RECT_EN;
		break;
	case REGULATOR_MODE_NORMAL:
		/* PULSE SKIP mode */
		l_mode = SW_MODE_PULSE_SKIP_EN;
		break;
	case REGULATOR_MODE_IDLE:
		/* LOW POWER mode */
		l_mode = SW_MODE_LOW_POWER_EN;
		break;
	case REGULATOR_MODE_STANDBY:
		/* NO PULSE SKIP mode */
		l_mode = SW_MODE_PULSE_NO_SKIP_EN;
		break;
	default:
		return -EINVAL;
	}
	return pmic_write_reg(register1, reg_val, reg_mask);
}

static unsigned int mc34708_sw_get_normal_mode(struct regulator_dev *reg)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int register1 = 0;
	unsigned int l_mode = 0;
	int sw = rdev_get_id(reg);
	int ret = 0;

	ret = pmic_read_reg(register1, &reg_val, reg_mask);
	return ret;

}

static int mc34708_sw_set_stby_mode(struct regulator_dev *reg,
				    unsigned int mode)
{
	unsigned int reg_val = 0, reg_mask = 0;
	unsigned int register1 = 0;
	unsigned int l_mode;
	int sw = rdev_get_id(reg);

	return pmic_write_reg(register1, reg_val, reg_mask);
}

static int mc34708_swbst_enable(struct regulator_dev *reg)
{
	unsigned int register_val = 0, register_mask = 0;
	unsigned int register1;
	int id = rdev_get_id(reg);

	switch (id) {
	case MC34708_SWBST:
		register_val = 0xC;	/* SWBSTMODE=0x3 */
		register_mask = 0xC;
		pmic_write_reg(MC34708_REG_SWBST_CTL, register_val,
			       register_mask);
		register_val = BITFVAL(VUSBSEL, VUSBSEL_ENABLE);
		register_mask = BITFMASK(VUSBSEL);
		break;
	default:
		return -EINVAL;
	}
	register1 = MC34708_REG_REGULATOR_MODE0;

	return pmic_write_reg(register1, register_val, register_mask);
}

static int mc34708_swbst_disable(struct regulator_dev *reg)
{
	unsigned int register_val = 0, register_mask = 0;
	unsigned int register1;
	int id = rdev_get_id(reg);

	switch (id) {
	case MC34708_SWBST:
		register_val = BITFVAL(VUSBSEL, VUSBSEL_DISABLE);
		register_mask = BITFMASK(VUSBSEL);
		break;
	default:
		return -EINVAL;
	}
	register1 = MC34708_REG_REGULATOR_MODE0;
	return pmic_write_reg(register1, register_val, register_mask);
}

static struct regulator_ops mc34708_vrefddr_ops = {
	.enable = mc34708_ldo_enable,
	.disable = mc34708_ldo_disable,
};

static struct regulator_ops mc34708_vpll_ops = {
	.set_voltage = mc34708_vpll_set_voltage,
	.get_voltage = mc34708_vpll_get_voltage,
};

static struct regulator_ops mc34708_swbst_ops = {
	.enable = mc34708_swbst_enable,
	.disable = mc34708_swbst_disable,
	.is_enabled = mc34708_regulator_is_enabled,
};

static struct regulator_ops mc34708_vusb_ops = {
	.enable = mc34708_ldo_enable,
	.disable = mc34708_ldo_disable,
	.is_enabled = mc34708_regulator_is_enabled,
};

static struct regulator_ops mc34708_vusb2_ops = {
	.set_voltage = mc34708_vusb2_set_voltage,
	.get_voltage = mc34708_vusb2_get_voltage,
	.enable = mc34708_ldo_enable,
	.disable = mc34708_ldo_disable,
};

static struct regulator_ops mc34708_vgen1_ops = {
	.set_voltage = mc34708_vgen1_set_voltage,
	.get_voltage = mc34708_vgen1_get_voltage,
	.enable = mc34708_ldo_enable,
	.disable = mc34708_ldo_disable,
};

static struct regulator_ops mc34708_vgen2_ops = {
	.set_voltage = mc34708_vgen2_set_voltage,
	.get_voltage = mc34708_vgen2_get_voltage,
	.enable = mc34708_ldo_enable,
	.disable = mc34708_ldo_disable,
};

static struct regulator_ops mc34708_vdac_ops = {
	.set_voltage = mc34708_vdac_set_voltage,
	.get_voltage = mc34708_vdac_get_voltage,
	.enable = mc34708_ldo_enable,
	.disable = mc34708_ldo_disable,
};

static struct regulator_ops mc34708_sw_ops = {
	.set_voltage = mc34708_sw_set_normal_voltage,
	.get_voltage = mc34708_sw_get_normal_voltage,
	.get_mode = mc34708_sw_get_normal_mode,
	.set_mode = mc34708_sw_set_normal_mode,
	.set_suspend_voltage = mc34708_sw_set_stby_voltage,
	.set_suspend_enable = mc34708_sw_stby_enable,
	.set_suspend_disable = mc34708_sw_stby_disable,
	.set_suspend_mode = mc34708_sw_set_stby_mode,
};

static struct regulator_desc reg_mc34708[] = {
	{
	 .name = "SW1A",
	 .id = MC34708_SW1A,
	 .ops = &mc34708_sw_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "SW1B",
	 .id = MC34708_SW1B,
	 .ops = &mc34708_sw_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "SW2",
	 .id = MC34708_SW2,
	 .ops = &mc34708_sw_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "SW3",
	 .id = MC34708_SW3,
	 .ops = &mc34708_sw_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "SW4A",
	 .id = MC34708_SW4A,
	 .ops = &mc34708_sw_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "SW4B",
	 .id = MC34708_SW4B,
	 .ops = &mc34708_sw_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "SW5",
	 .id = MC34708_SW5,
	 .ops = &mc34708_sw_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "SWBST",
	 .id = MC34708_SWBST,
	 .ops = &mc34708_swbst_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "VPLL",
	 .id = MC34708_VPLL,
	 .ops = &mc34708_vpll_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "VREFDDR",
	 .id = MC34708_VREFDDR,
	 .ops = &mc34708_vrefddr_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "VUSB",
	 .id = MC34708_VUSB,
	 .ops = &mc34708_vusb_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "VUSB2",
	 .id = MC34708_VUSB2,
	 .ops = &mc34708_vusb2_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "VDAC",
	 .id = MC34708_VDAC,
	 .ops = &mc34708_vdac_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "VGEN1",
	 .id = MC34708_VGEN1,
	 .ops = &mc34708_vgen1_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "VGEN2",
	 .id = MC34708_VGEN2,
	 .ops = &mc34708_vgen2_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
};

/*
 * Init and Exit
 */

static int reg_mc34708_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	/* register regulator */
	rdev = regulator_register(&reg_mc34708[pdev->id], &pdev->dev,
				  pdev->dev.platform_data,
				  dev_get_drvdata(&pdev->dev));
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			reg_mc34708[pdev->id].name);
		return PTR_ERR(rdev);
	}
	platform_set_drvdata(pdev, rdev);

	return 0;
}

static int mc34708_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;
}

int mc34708_register_regulator(struct mc34708 *mc34708, int reg,
			       struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;
	if (mc34708->pmic.pdev[reg])
		return -EBUSY;

	pdev = platform_device_alloc("mc34708-regulatr", reg);
	if (!pdev)
		return -ENOMEM;

	mc34708->pmic.pdev[reg] = pdev;

	initdata->driver_data = mc34708;

	pdev->dev.platform_data = initdata;
	pdev->dev.parent = mc34708->dev;
	ret = platform_device_add(pdev);

	if (ret != 0) {
		dev_err(mc34708->dev, "Failed to register regulator %d: %d\n",
			reg, ret);
		platform_device_del(pdev);
		mc34708->pmic.pdev[reg] = NULL;
	}

	return ret;
}

EXPORT_SYMBOL_GPL(mc34708_register_regulator);

static struct platform_driver mc34708_regulator_driver = {
	.probe = reg_mc34708_probe,
	.remove = mc34708_regulator_remove,
	.driver = {
		   .name = "mc34708-regulatr",
		   /* o left out due to string length */
		   },
};

static int __init mc34708_regulator_subsys_init(void)
{
	return platform_driver_register(&mc34708_regulator_driver);
}

subsys_initcall(mc34708_regulator_subsys_init);

static void __exit mc34708_regulator_exit(void)
{
	platform_driver_unregister(&mc34708_regulator_driver);
}

module_exit(mc34708_regulator_exit);

/* Module information */
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("mc34708 Regulator driver");
MODULE_LICENSE("GPL");
