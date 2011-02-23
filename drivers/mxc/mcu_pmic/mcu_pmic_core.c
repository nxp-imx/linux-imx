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
 * @file mc9s08dz60/mcu_pmic_core.c
 * @brief This is the main file of mc9s08dz60 Power Control driver.
 *
 * @ingroup PMIC_POWER
 */

/*
 * Includes
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/mc9s08dz60/pmic.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>

#include "mcu_pmic_core.h"
#include "mc9s08dz60.h"
#include "max8660.h"

/* bitfield macros for mcu pmic*/
#define SET_BIT_IN_BYTE(byte, pos) (byte |= (0x01 << pos))
#define CLEAR_BIT_IN_BYTE(byte, pos) (byte &= ~(0x01 << pos))


/* map reg names (enum pmic_reg in pmic_external.h) to real addr*/
const static u8 mcu_pmic_reg_addr_table[] = {
	MCU_VERSION,
	MCU_SECS,
	MCU_MINS,
	MCU_HRS,
	MCU_DAY,
	MCU_DATE,
	MCU_MONTH,
	MCU_YEAR,
	MCU_ALARM_SECS,
	MCU_ALARM_MINS,
	MCU_ALARM_HRS,
	MCU_TS_CONTROL,
	MCU_X_LOW,
	MCU_Y_LOW,
	MCU_XY_HIGH,
	MCU_X_LEFT_LOW,
	MCU_X_LEFT_HIGH,
	MCU_X_RIGHT,
	MCU_Y_TOP_LOW,
	MCU_Y_TOP_HIGH,
	MCU_Y_BOTTOM,
	MCU_RESET_1,
	MCU_RESET_2,
	MCU_POWER_CTL,
	MCU_DELAY_CONFIG,
	MCU_GPIO_1,
	MCU_GPIO_2,
	MCU_KPD_1,
	MCU_KPD_2,
	MCU_KPD_CONTROL,
	MCU_INT_ENABLE_1,
	MCU_INT_ENABLE_2,
	MCU_INT_FLAG_1,
	MCU_INT_FLAG_2,
	MCU_DES_FLAG,
	MAX8660_OUTPUT_ENABLE_1,
	MAX8660_OUTPUT_ENABLE_2,
	MAX8660_VOLT_CHANGE_CONTROL,
	MAX8660_V3_TARGET_VOLT_1,
	MAX8660_V3_TARGET_VOLT_2,
	MAX8660_V4_TARGET_VOLT_1,
	MAX8660_V4_TARGET_VOLT_2,
	MAX8660_V5_TARGET_VOLT_1,
	MAX8660_V5_TARGET_VOLT_2,
	MAX8660_V6V7_TARGET_VOLT,
	MAX8660_FORCE_PWM
};

static int mcu_pmic_read(int reg_num, unsigned int *reg_val)
{
	int ret;
	u8 value = 0;
	/* mcu ops */
	if (reg_num >= REG_MCU_VERSION && reg_num <= REG_MCU_DES_FLAG)
		ret = mc9s08dz60_read_reg(mcu_pmic_reg_addr_table[reg_num],
					&value);
	else if (reg_num >= REG_MAX8660_OUTPUT_ENABLE_1
		   && reg_num <= REG_MAX8660_FORCE_PWM)
		ret = max8660_get_buffered_reg_val(reg_num, &value);
	else
		return -1;

	if (ret < 0)
		return -1;
	*reg_val = value;

	return 0;
}

static int mcu_pmic_write(int reg_num, const unsigned int reg_val)
{
	int ret;
	u8 value = reg_val;
	/* mcu ops */
	if (reg_num >= REG_MCU_VERSION && reg_num <= REG_MCU_DES_FLAG) {

		ret =
		    mc9s08dz60_write_reg(
			mcu_pmic_reg_addr_table[reg_num], value);
		if (ret < 0)
			return -1;
	} else if (reg_num >= REG_MAX8660_OUTPUT_ENABLE_1
		   && reg_num <= REG_MAX8660_FORCE_PWM) {
		ret =
		    max8660_write_reg(mcu_pmic_reg_addr_table[reg_num], value);

		if (ret < 0)
			return -1;

		ret = max8660_save_buffered_reg_val(reg_num, value);
	} else
		return -1;

	return 0;
}

int mcu_pmic_read_reg(int reg, unsigned int *reg_value,
			  unsigned int reg_mask)
{
	int ret = 0;
	unsigned int temp = 0;

	ret = mcu_pmic_read(reg, &temp);
	if (ret != 0)
		return -1;
	*reg_value = (temp & reg_mask);

	pr_debug("Read REG[ %d ] = 0x%x\n", reg, *reg_value);

	return ret;
}


int mcu_pmic_write_reg(int reg, unsigned int reg_value,
			   unsigned int reg_mask)
{
	int ret = 0;
	unsigned int temp = 0;

	ret = mcu_pmic_read(reg, &temp);
	if (ret != 0)
		return -1;
	temp = (temp & (~reg_mask)) | reg_value;

	ret = mcu_pmic_write(reg, temp);
	if (ret != 0)
		return -1;

	pr_debug("Write REG[ %d ] = 0x%x\n", reg, reg_value);

	return ret;
}

/*!
 * make max8660 - mc9s08dz60 enter low-power mode
 */
static void pmic_power_off(void)
{
	mcu_pmic_write_reg(REG_MCU_POWER_CTL, 0x10, 0x10);
}

static int __init mcu_pmic_init(void)
{
	int err;

	/* init chips */
	err = max8660_init();
	if (err)
		goto fail1;

	err = mc9s08dz60_init();
	if (err)
		goto fail1;

	if (is_max8660_present()) {
		pr_info("max8660 is present \n");
		pm_power_off = pmic_power_off;
	} else
		pr_debug("max8660 is not present\n");
	pr_info("mcu_pmic_init completed!\n");
	return 0;

fail1:
	pr_err("mcu_pmic_init failed!\n");
	return err;
}

static void __exit mcu_pmic_exit(void)
{
	reg_max8660_remove();
	mc9s08dz60_exit();
	max8660_exit();
}

subsys_initcall_sync(mcu_pmic_init);
module_exit(mcu_pmic_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("mcu pmic driver");
MODULE_LICENSE("GPL");
