/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


#ifndef __LINUX_MCU_PMIC_H_
#define __LINUX_MCU_PMIC_H_

enum {

	/*reg names for mcu */
	REG_MCU_VERSION = 0,
	REG_MCU_SECS,
	REG_MCU_MINS,
	REG_MCU_HRS,
	REG_MCU_DAY,
	REG_MCU_DATE,
	REG_MCU_MONTH,
	REG_MCU_YEAR,
	REG_MCU_ALARM_SECS,
	REG_MCU_ALARM_MINS,
	REG_MCU_ALARM_HRS,
	REG_MCU_TS_CONTROL,
	REG_MCU_X_LOW,
	REG_MCU_Y_LOW,
	REG_MCU_XY_HIGH,
	REG_MCU_X_LEFT_LOW,
	REG_MCU_X_LEFT_HIGH,
	REG_MCU_X_RIGHT,
	REG_MCU_Y_TOP_LOW,
	REG_MCU_Y_TOP_HIGH,
	REG_MCU_Y_BOTTOM,
	REG_MCU_RESET_1,
	REG_MCU_RESET_2,
	REG_MCU_POWER_CTL,
	REG_MCU_DELAY_CONFIG,
	REG_MCU_GPIO_1,
	REG_MCU_GPIO_2,
	REG_MCU_KPD_1,
	REG_MCU_KPD_2,
	REG_MCU_KPD_CONTROL,
	REG_MCU_INT_ENABLE_1,
	REG_MCU_INT_ENABLE_2,
	REG_MCU_INT_FLAG_1,
	REG_MCU_INT_FLAG_2,
	REG_MCU_DES_FLAG,
};

enum {

	MCU_GPIO_REG_RESET_1,
	MCU_GPIO_REG_RESET_2,
	MCU_GPIO_REG_POWER_CONTROL,
	MCU_GPIO_REG_GPIO_CONTROL_1,
	MCU_GPIO_REG_GPIO_CONTROL_2,
};

int pmic_gpio_set_bit_val(int reg, unsigned int bit,
				  unsigned int val);

int pmic_gpio_get_bit_val(int reg, unsigned int bit,
				  unsigned int *val);

int pmic_gpio_get_designation_bit_val(unsigned int bit,
					      unsigned int *val);


int mcu_pmic_read_reg(int reg, unsigned int *reg_value,
			  unsigned int reg_mask);

int mcu_pmic_write_reg(int reg, unsigned int reg_value,
			   unsigned int reg_mask);
#endif
