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
 * @file mc13783/pmic_battery_defs.h
 * @brief This is the internal header for PMIC(mc13783) Battery driver.
 *
 * @ingroup PMIC_BATTERY
 */

#ifndef __PMIC_BATTERY_DEFS_H__
#define __PMIC_BATTERY_DEFS_H__

#define         PMIC_BATTERY_STRING    "pmic_battery"

/* REG_CHARGE */
#define MC13783_BATT_DAC_V_DAC_LSH		0
#define MC13783_BATT_DAC_V_DAC_WID		3
#define MC13783_BATT_DAC_DAC_LSH			3
#define MC13783_BATT_DAC_DAC_WID			4
#define	MC13783_BATT_DAC_TRCKLE_LSH		7
#define	MC13783_BATT_DAC_TRCKLE_WID		3
#define MC13783_BATT_DAC_FETOVRD_EN_LSH		10
#define MC13783_BATT_DAC_FETOVRD_EN_WID		1
#define MC13783_BATT_DAC_FETCTRL_EN_LSH		11
#define MC13783_BATT_DAC_FETCTRL_EN_WID		1
#define MC13783_BATT_DAC_REVERSE_SUPPLY_LSH	13
#define MC13783_BATT_DAC_REVERSE_SUPPLY_WID	1
#define MC13783_BATT_DAC_OVCTRL_LSH		15
#define MC13783_BATT_DAC_OVCTRL_WID		2
#define MC13783_BATT_DAC_UNREGULATED_LSH		17
#define MC13783_BATT_DAC_UNREGULATED_WID		1
#define MC13783_BATT_DAC_LED_EN_LSH		18
#define MC13783_BATT_DAC_LED_EN_WID		1
#define MC13783_BATT_DAC_5K_LSH			19
#define MC13783_BATT_DAC_5K_WID			1

#define         BITS_OUT_VOLTAGE        0
#define         LONG_OUT_VOLTAGE        3
#define         BITS_CURRENT_MAIN       3
#define         LONG_CURRENT_MAIN       4
#define         BITS_CURRENT_TRICKLE    7
#define         LONG_CURRENT_TRICKLE    3
#define         BIT_FETOVRD             10
#define         BIT_FETCTRL             11
#define         BIT_RVRSMODE            13
#define         BITS_OVERVOLTAGE        15
#define         LONG_OVERVOLTAGE        2
#define         BIT_UNREGULATED         17
#define         BIT_CHRG_LED            18
#define         BIT_CHRGRAWPDEN         19

/* REG_POWXER_CONTROL_0 */
#define MC13783_BATT_DAC_V_COIN_LSH		20
#define MC13783_BATT_DAC_V_COIN_WID		3
#define MC13783_BATT_DAC_COIN_CH_EN_LSH		23
#define MC13783_BATT_DAC_COIN_CH_EN_WID		1
#define MC13783_BATT_DAC_COIN_CH_EN_ENABLED	1
#define MC13783_BATT_DAC_COIN_CH_EN_DISABLED	0
#define MC13783_BATT_DAC_EOL_CMP_EN_LSH		18
#define MC13783_BATT_DAC_EOL_CMP_EN_WID		1
#define MC13783_BATT_DAC_EOL_CMP_EN_ENABLE	1
#define MC13783_BATT_DAC_EOL_CMP_EN_DISABLE	0
#define MC13783_BATT_DAC_EOL_SEL_LSH		16
#define MC13783_BATT_DAC_EOL_SEL_WID		2

#define         DEF_VALUE               0

#define         BAT_THRESHOLD_MAX       3

#endif				/*  __PMIC_BATTERY_DEFS_H__ */
