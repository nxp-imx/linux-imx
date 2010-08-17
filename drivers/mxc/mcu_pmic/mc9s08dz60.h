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
 * @file mc9s08dz60.h
 * @brief Driver for mc9s08dz60
 *
 * @ingroup pmic
 */
#ifndef _MC9SDZ60_H_
#define _MC9SDZ60_H_

#define MCU_VERSION	0x00
/*#define Reserved	0x01*/
#define MCU_SECS		0x02
#define MCU_MINS		0x03
#define MCU_HRS			0x04
#define MCU_DAY			0x05
#define MCU_DATE		0x06
#define MCU_MONTH		0x07
#define MCU_YEAR		0x08

#define MCU_ALARM_SECS	0x09
#define MCU_ALARM_MINS	0x0A
#define MCU_ALARM_HRS	0x0B
/* #define Reserved	0x0C*/
/* #define Reserved	0x0D*/
#define MCU_TS_CONTROL	0x0E
#define MCU_X_LOW	0x0F
#define MCU_Y_LOW	0x10
#define MCU_XY_HIGH	0x11
#define MCU_X_LEFT_LOW	0x12
#define MCU_X_LEFT_HIGH	0x13
#define MCU_X_RIGHT	0x14
#define MCU_Y_TOP_LOW	0x15
#define MCU_Y_TOP_HIGH	0x16
#define MCU_Y_BOTTOM	0x17
/* #define Reserved	0x18*/
/* #define Reserved	0x19*/
#define MCU_RESET_1	0x1A
#define MCU_RESET_2	0x1B
#define MCU_POWER_CTL	0x1C
#define MCU_DELAY_CONFIG	0x1D
/* #define Reserved	0x1E */
/* #define Reserved	0x1F */
#define MCU_GPIO_1	0x20
#define MCU_GPIO_2	0x21
#define MCU_KPD_1	0x22
#define MCU_KPD_2	0x23
#define MCU_KPD_CONTROL	0x24
#define MCU_INT_ENABLE_1	0x25
#define MCU_INT_ENABLE_2	0x26
#define MCU_INT_FLAG_1	0x27
#define MCU_INT_FLAG_2	0x28
#define MCU_DES_FLAG		0x29
int mc9s08dz60_read_reg(u8 reg, u8 *value);
int mc9s08dz60_write_reg(u8 reg, u8 value);
int mc9s08dz60_init(void);
void mc9s08dz60_exit(void);

extern int reg_mc9s08dz60_probe(void);
extern int reg_mc9s08dz60_remove(void);

#endif	/* _MC9SDZ60_H_ */
