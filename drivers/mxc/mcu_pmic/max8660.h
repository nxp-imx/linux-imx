/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file max8660.h
 * @brief Driver for max8660
 *
 * @ingroup pmic
 */
#ifndef _MAX8660_H_
#define _MAX8660_H_

#ifdef __KERNEL__

#define MAX8660_OUTPUT_ENABLE_1	0x10
#define MAX8660_OUTPUT_ENABLE_2	0x12
#define MAX8660_VOLT_CHANGE_CONTROL		0x20
#define MAX8660_V3_TARGET_VOLT_1	0x23
#define MAX8660_V3_TARGET_VOLT_2	0x24
#define MAX8660_V4_TARGET_VOLT_1	0x29
#define MAX8660_V4_TARGET_VOLT_2	0x2A
#define MAX8660_V5_TARGET_VOLT_1	0x32
#define MAX8660_V5_TARGET_VOLT_2	0x33
#define MAX8660_V6V7_TARGET_VOLT	0x39
#define MAX8660_FORCE_PWM			0x80

int is_max8660_present(void);
int max8660_write_reg(u8 reg, u8 value);
int max8660_save_buffered_reg_val(int reg_name, u8 value);
int max8660_get_buffered_reg_val(int reg_name, u8 *value);
int max8660_init(void);
void max8660_exit(void);

extern int reg_max8660_probe(void);
extern int reg_max8660_remove(void);

#endif				/* __KERNEL__ */

#endif				/* _MAX8660_H_ */
