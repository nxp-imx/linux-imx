/*
 * Copyright 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mc9s08dz60/mcu_pmic_gpio.c
 * @brief This is the main file of mc9s08dz60 Power Control driver.
 *
 * @ingroup PMIC_POWER
 */

/*
 * Includes
 */
#include <linux/platform_device.h>
#include <linux/mfd/mc9s08dz60/pmic.h>
#include <linux/pmic_status.h>
#include <linux/ioctl.h>

#define SET_BIT_IN_BYTE(byte, pos) (byte |= (0x01 << pos))
#define CLEAR_BIT_IN_BYTE(byte, pos) (byte &= ~(0x01 << pos))

int pmic_gpio_set_bit_val(int reg, unsigned int bit,
				  unsigned int val)
{
	int reg_name;
	u8 reg_mask = 0;

	if (bit > 7)
		return -1;

	switch (reg) {
	case MCU_GPIO_REG_RESET_1:
		reg_name = REG_MCU_RESET_1;
		break;
	case MCU_GPIO_REG_RESET_2:
		reg_name = REG_MCU_RESET_2;
		break;
	case MCU_GPIO_REG_POWER_CONTROL:
		reg_name = REG_MCU_POWER_CTL;
		break;
	case MCU_GPIO_REG_GPIO_CONTROL_1:
		reg_name = REG_MCU_GPIO_1;
		break;
	case MCU_GPIO_REG_GPIO_CONTROL_2:
		reg_name = REG_MCU_GPIO_2;
		break;
	default:
		return -1;
	}

	SET_BIT_IN_BYTE(reg_mask, bit);
	if (0 == val)
		CHECK_ERROR(mcu_pmic_write_reg(reg_name, 0, reg_mask));
	else
		CHECK_ERROR(mcu_pmic_write_reg(reg_name, reg_mask, reg_mask));

	return 0;
}
EXPORT_SYMBOL(pmic_gpio_set_bit_val);

int pmic_gpio_get_bit_val(int reg, unsigned int bit,
				  unsigned int *val)
{
	int reg_name;
	unsigned int reg_read_val;
	u8 reg_mask = 0;

	if (bit > 7)
		return -1;

	switch (reg) {
	case MCU_GPIO_REG_RESET_1:
		reg_name = REG_MCU_RESET_1;
		break;
	case MCU_GPIO_REG_RESET_2:
		reg_name = REG_MCU_RESET_2;
		break;
	case MCU_GPIO_REG_POWER_CONTROL:
		reg_name = REG_MCU_POWER_CTL;
		break;
	case MCU_GPIO_REG_GPIO_CONTROL_1:
		reg_name = REG_MCU_GPIO_1;
		break;
	case MCU_GPIO_REG_GPIO_CONTROL_2:
		reg_name = REG_MCU_GPIO_2;
		break;
	default:
		return -1;
	}

	SET_BIT_IN_BYTE(reg_mask, bit);
	CHECK_ERROR(mcu_pmic_read_reg(reg_name, &reg_read_val, reg_mask));
	if (0 == reg_read_val)
		*val = 0;
	else
		*val = 1;

	return 0;
}
EXPORT_SYMBOL(pmic_gpio_get_bit_val);

int pmic_gpio_get_designation_bit_val(unsigned int bit,
					unsigned int *val)
{
	unsigned int reg_read_val;
	u8 reg_mask = 0;

	if (bit > 7)
		return -1;

	SET_BIT_IN_BYTE(reg_mask, bit);
	CHECK_ERROR(
		mcu_pmic_read_reg(REG_MCU_DES_FLAG, &reg_read_val, reg_mask));
	if (0 == reg_read_val)
		*val = 0;
	else
		*val = 1;

	return 0;
}
EXPORT_SYMBOL(pmic_gpio_get_designation_bit_val);
