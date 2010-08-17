/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mc13783/pmic_light_defs.h
 * @brief This is the internal header PMIC(mc13783) Light and Backlight driver.
 *
 * @ingroup PMIC_LIGHT
 */

#ifndef __MC13783_LIGHT_DEFS_H__
#define __MC13783_LIGHT_DEFS_H__

#define LREG_0   REG_LED_CONTROL_0
#define LREG_1   REG_LED_CONTROL_1
#define LREG_2   REG_LED_CONTROL_2
#define LREG_3   REG_LED_CONTROL_3
#define LREG_4   REG_LED_CONTROL_4
#define LREG_5   REG_LED_CONTROL_5

/* REG_LED_CONTROL_0 */

#define         BIT_LEDEN_LSH           0
#define         BIT_LEDEN_WID           1
#define         MASK_TRIODE_MAIN_BL     0x080
#define         INDEX_AUXILIARY         1
#define         INDEX_KEYPAD            2
#define         BITS_FUN_LIGHT_LSH      17
#define         BITS_FUN_LIGHT_WID      4
#define         MASK_FUN_LIGHT          0x1E0000
#define         MASK_BK1_FL             0x200000
#define         ENABLE_BK1_FL           0x200000
#define         MASK_BK2_FL             0x400000
#define         ENABLE_BK2_FL           0x400000
#define         MASK_BK3_FL             0x800000
#define         ENABLE_BK3_FL           0x800000
#define 	BIT_UP_MAIN_BL_LSH	1
#define 	BIT_UP_MAIN_BL_WID	1
#define 	BIT_UP_AUX_BL_LSH	2
#define 	BIT_UP_AUX_BL_WID	1
#define 	BIT_UP_KEY_BL_LSH	3
#define 	BIT_UP_KEY_BL_WID	1
#define 	BIT_DOWN_MAIN_BL_LSH	4
#define 	BIT_DOWN_MAIN_BL_WID	1
#define 	BIT_DOWN_AUX_BL_LSH	5
#define 	BIT_DOWN_AUX_BL_WID	1
#define 	BIT_DOWN_KEY_BL_LSH	6
#define 	BIT_DOWN_KEY_BL_WID	1
#define 	BIT_TRIODE_MAIN_BL_LSH	7
#define 	BIT_TRIODE_MAIN_BL_WID	1
#define 	BIT_TRIODE_AUX_BL_LSH	8
#define 	BIT_TRIODE_AUX_BL_WID	1
#define 	BIT_TRIODE_KEY_BL_LSH	9
#define 	BIT_TRIODE_KEY_BL_WID	1

#define         BIT_BOOSTEN_LSH         10
#define         BIT_BOOSTEN_WID         1
#define         BITS_BOOST_LSH          11
#define         BITS_BOOST_WID          5
#define         BITS_BOOST_ABMS_LSH     11
#define         BITS_BOOST_ABMS_WID     3
#define         BITS_BOOST_ABR_LSH      14
#define         BITS_BOOST_ABR_WID      2

#define         MAX_BOOST_ABMS          7
#define         MAX_BOOST_ABR           3

/* REG_LED_CONTROL_1 */

#define         BIT_SLEWLIMTC_LSH       23
#define         BIT_SLEWLIMTC_WID       1
#define         BIT_TC1HALF_LSH         18
#define         BIT_TC1HALF_WID         1
#define         LEDR1RAMPUP             0x000001
#define         LEDR2RAMPUP             0x000040
#define         LEDR3RAMPUP             0x001000
#define         LEDR1RAMPDOWN           0x000008
#define         LEDR2RAMPDOWN           0x000200
#define         LEDR3RAMPDOWN           0x008000

/* REG_LED_CONTROL_2 */

#define         BIT_SLEWLIMBL_LSH       23
#define         BIT_SLEWLIMBL_WID       1
#define         BIT_DUTY_CYCLE          9
#define         MASK_DUTY_CYCLE         0x001E00
#define         INDEX_AUX               4
#define         INDEX_KYD               8
#define         BIT_CL_MAIN_LSH		0
#define         BIT_CL_MAIN_WID		3
#define         BIT_CL_AUX_LSH		3
#define         BIT_CL_AUX_WID		3
#define         BIT_CL_KEY_LSH		6
#define         BIT_CL_KEY_WID		3

/* REG_LED_CONTROL_3 4 5 */
#define         BITS_CL_RED_LSH         0
#define         BITS_CL_RED_WID         2
#define         BITS_CL_GREEN_LSH       2
#define         BITS_CL_GREEN_WID       2
#define         BITS_CL_BLUE_LSH        4
#define         BITS_CL_BLUE_WID        2
#define         BITS_DC_RED_LSH         6
#define         BITS_DC_RED_WID         5
#define         BITS_DC_GREEN_LSH       11
#define         BITS_DC_GREEN_WID       5
#define         BITS_DC_BLUE_LSH        16
#define         BITS_DC_BLUE_WID        5
#define         BIT_PERIOD_LSH          21
#define         BIT_PERIOD_WID          2

#define         DUTY_CYCLE_MAX          31

/* Fun light pattern */
#define		BLENDED_RAMPS_SLOW      	0
#define		BLENDED_RAMPS_FAST      	1
#define		SAW_RAMPS_SLOW  		2
#define		SAW_RAMPS_FAST	     		3
#define		BLENDED_INVERSE_RAMPS_SLOW	4
#define		BLENDED_INVERSE_RAMPS_FAST	5
#define		CHASING_LIGHTS_RGB_SLOW		6
#define		CHASING_LIGHTS_RGB_FAST		7
#define		CHASING_LIGHTS_BGR_SLOW		8
#define		CHASING_LIGHTS_BGR_FAST		9
#define		FUN_LIGHTS_OFF			15

/*!
 * This function initialize Light registers of mc13783 with 0.
 *
 * @return       This function returns 0 if successful.
 */
int pmic_light_init_reg(void);

#endif				/*  __MC13783_LIGHT_DEFS_H__ */
