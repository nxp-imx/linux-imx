/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file pmic_external.c
 * @brief This file contains all external functions of PMIC drivers.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/init.h>
#include <linux/errno.h>

#include <linux/pmic_external.h>
#include <linux/pmic_status.h>

/*
 * External Functions
 */
extern int pmic_read(int reg_num, unsigned int *reg_val);
extern int pmic_write(int reg_num, const unsigned int reg_val);

/*!
 * This function is called by PMIC clients to read a register on PMIC.
 *
 * @param        reg        number of register
 * @param        reg_value  return value of register
 * @param        reg_mask   Bitmap mask indicating which bits to modify
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_read_reg(int reg, unsigned int *reg_value,
			  unsigned int reg_mask)
{
	int ret = 0;
	unsigned int temp = 0;

	ret = pmic_read(reg, &temp);
	if (ret != PMIC_SUCCESS) {
		return PMIC_ERROR;
	}
	*reg_value = (temp & reg_mask);

	pr_debug("Read REG[ %d ] = 0x%x\n", reg, *reg_value);

	return ret;
}

/*!
 * This function is called by PMIC clients to write a register on PMIC.
 *
 * @param        reg        number of register
 * @param        reg_value  New value of register
 * @param        reg_mask   Bitmap mask indicating which bits to modify
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_write_reg(int reg, unsigned int reg_value,
			   unsigned int reg_mask)
{
	int ret = 0;
	unsigned int temp = 0;

	ret = pmic_read(reg, &temp);
	if (ret != PMIC_SUCCESS) {
		return PMIC_ERROR;
	}
	temp = (temp & (~reg_mask)) | reg_value;
#ifdef CONFIG_MXC_PMIC_MC13783
	if (reg == REG_POWER_MISCELLANEOUS)
		temp &= 0xFFFE7FFF;
#endif
	ret = pmic_write(reg, temp);
	if (ret != PMIC_SUCCESS) {
		return PMIC_ERROR;
	}

	pr_debug("Write REG[ %d ] = 0x%x\n", reg, reg_value);

	return ret;
}

EXPORT_SYMBOL(pmic_read_reg);
EXPORT_SYMBOL(pmic_write_reg);
