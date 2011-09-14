/*
 * Copyright 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mcu_pmic_core.h
 * @brief Driver for max8660
 *
 * @ingroup pmic
 */
#ifndef _MCU_PMIC_CORE_H_
#define _MCU_PMIC_CORE_H_

#include <linux/mfd/mc9s08dz60/pmic.h>

#define MAX8660_REG_START (REG_MCU_DES_FLAG + 1)
enum {

	/* reg names for max8660 */
	REG_MAX8660_OUTPUT_ENABLE_1 = MAX8660_REG_START,
	REG_MAX8660_OUTPUT_ENABLE_2,
	REG_MAX8660_VOLT_CHANGE_CONTROL_1,
	REG_MAX8660_V3_TARGET_VOLT_1,
	REG_MAX8660_V3_TARGET_VOLT_2,
	REG_MAX8660_V4_TARGET_VOLT_1,
	REG_MAX8660_V4_TARGET_VOLT_2,
	REG_MAX8660_V5_TARGET_VOLT_1,
	REG_MAX8660_V5_TARGET_VOLT_2,
	REG_MAX8660_V6V7_TARGET_VOLT,
	REG_MAX8660_FORCE_PWM
};


#endif				/* _MCU_PMIC_CORE_H_ */
