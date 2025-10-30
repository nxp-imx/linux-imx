// SPDX-License-Identifier: GPL-2.0-only
/*
 * Register configurations for all sensor supported modes
 * Copyright 2024-2025, NXP
 * Copyright 2024, Omnivision
 * Copyright 2024, Verisilicon
 *
 */

#include <media/v4l2-cci.h>
#include "ox05b1s.h"

#define OS08A20_REG_PLL_CTRL_05		CCI_REG8(0x0305)
#define OS08A20_REG_PLL_CTRL_25		CCI_REG8(0x0325)
#define OS08A20_REG_MIPI_BIT_10_12	CCI_REG8(0x031e)
/* Analog Control Registers 0x3600-0x3637 */
#define OS08A20_REG_ANA_CTRL		CCI_REG8(0x3600)
#define OS08A20_REG_CORE_0		CCI_REG8(0x3660)
/* Sensor Timing Control Registers 0x3700-0x37ff */
#define OS08A20_REG_SENSOR_TIMING_CTRL	CCI_REG8(0x3700)
#define OS08A20_REG_L2S_MODE		CCI_REG8(0x3797)
#define OS08A20_REG_X_ODD_INC		CCI_REG8(0x3814)
#define OS08A20_REG_Y_ODD_INC		CCI_REG8(0x3816)
#define OS08A20_REG_FORMAT1		CCI_REG8(0x3820)
#define OS08A20_REG_FORMAT2		CCI_REG8(0x3821)
#define OS08A20_REG_BLC_CTRL08		CCI_REG8(0x4008)
#define OS08A20_REG_BLC_CTRL09		CCI_REG8(0x4009)
#define OS08A20_REG_PCLK_PERIOD		CCI_REG8(0x4837)
#define OS08A20_REG_ISP_CTRL_1		CCI_REG8(0x5001)
#define OS08A20_REG_ISP_CTRL_5		CCI_REG8(0x5005)

/* Common register configuration for Omnivision OS08A20 raw camera */
static const struct cci_reg_sequence os08a20_init_setting_common[] = {
	{OS08A20_REG_ANA_CTRL + 0x05, 0x50},
	{OS08A20_REG_ANA_CTRL + 0x10, 0x39},
	{OS08A20_REG_SENSOR_TIMING_CTRL + 0x5e, 0x0b},
	{OS08A20_REG_L2S_MODE, 0x04},
	{OS08A20_REG_BLC_CTRL08, 0x02},
	{OS08A20_REG_BLC_CTRL09, 0x0d},
	{OS08A20_REG_ISP_CTRL_1, 0x42},
	{OS08A20_REG_ISP_CTRL_5, 0x00},
};

/* Common register configuration for Omnivision OS08A20 10 bit */
static const struct cci_reg_sequence os08a20_init_setting_10bit[] = {
	{OS08A20_REG_MIPI_BIT_10_12, 0x09},
	{OS08A20_REG_ANA_CTRL + 0x09, 0xb5},
	{OS08A20_REG_CORE_0, 0x43},
	{OS08A20_REG_SENSOR_TIMING_CTRL + 0x06, 0x35},
	{CCI_REG24(0x3709), 0x490098},
};

/* Common register configuration for Omnivision OS08A20 12 bit */
static const struct cci_reg_sequence os08a20_init_setting_12bit[] = {
	{OS08A20_REG_MIPI_BIT_10_12, 0x0a},
	{OS08A20_REG_ANA_CTRL + 0x09, 0xdb},
	{OS08A20_REG_CORE_0, 0xd3},
	{OS08A20_REG_SENSOR_TIMING_CTRL + 0x06, 0x6a},
	{CCI_REG24(0x3709), 0x480130},
};

/* Mode specific register configurations for Omnivision OS08A20 raw camera */

/* OS08A20 3840 x 2160 @30fps BGGR10 */
static const struct cci_reg_sequence os08a20_init_setting_4k_10b[] = {
	{OS08A20_REG_PLL_CTRL_05, 0x3c},
	{OS08A20_REG_FORMAT2, 0x04}, /* mirror */
	{OS08A20_REG_PCLK_PERIOD, 0x10},
};

/* OS08A20 3840 x 2160 @30fps BGGR12 */
static const struct cci_reg_sequence os08a20_init_setting_4k_12b[] = {
	{OS08A20_REG_PLL_CTRL_05, 0x3c},
	{OS08A20_REG_PLL_CTRL_25, 0x47},
	{OS08A20_REG_FORMAT2, 0x04}, /* mirror */
	{OS08A20_REG_PCLK_PERIOD, 0x10},
};

/* OS08A20 1920 x 1080 @60fps BGGR10 */
static const struct cci_reg_sequence os08a20_init_setting_1080p_10b[] = {
	{OS08A20_REG_PLL_CTRL_05, 0x2d},
	{OS08A20_REG_PLL_CTRL_25, 0x45},
	{OS08A20_REG_X_ODD_INC, 0x03},
	{OS08A20_REG_Y_ODD_INC, 0x03},
	{OS08A20_REG_FORMAT1, 0x01}, /* vertical bining */
	{OS08A20_REG_FORMAT2, 0x05}, /* mirror, horizontal bining */
	{OS08A20_REG_BLC_CTRL09, 0x05},
	{OS08A20_REG_PCLK_PERIOD, 0x16},
};

const struct ox05b1s_reglist os08a20_reglist_4k_10b[] = {
	{
		.regs = os08a20_init_setting_common,
		.count = ARRAY_SIZE(os08a20_init_setting_common),
	},
	{
		.regs = os08a20_init_setting_10bit,
		.count = ARRAY_SIZE(os08a20_init_setting_10bit),
	},
	{
		.regs = os08a20_init_setting_4k_10b,
		.count = ARRAY_SIZE(os08a20_init_setting_4k_10b),
	},
	{
		/* sentinel */
	}
};

const struct ox05b1s_reglist os08a20_reglist_4k_12b[] = {
	{
		.regs = os08a20_init_setting_common,
		.count = ARRAY_SIZE(os08a20_init_setting_common),
	},
	{
		.regs = os08a20_init_setting_12bit,
		.count = ARRAY_SIZE(os08a20_init_setting_12bit),
	},
	{
		.regs = os08a20_init_setting_4k_12b,
		.count = ARRAY_SIZE(os08a20_init_setting_4k_12b),
	},
	{
		/* sentinel */
	}
};

const struct ox05b1s_reglist os08a20_reglist_1080p_10b[] = {
	{
		.regs = os08a20_init_setting_common,
		.count = ARRAY_SIZE(os08a20_init_setting_common),
	},
	{
		.regs = os08a20_init_setting_10bit,
		.count = ARRAY_SIZE(os08a20_init_setting_10bit),
	},
	{
		.regs = os08a20_init_setting_1080p_10b,
		.count = ARRAY_SIZE(os08a20_init_setting_1080p_10b),
	},
	{
		/* sentinel */
	}
};

#define OX05B1S_REG_PLL1_CTRL_REG07	CCI_REG8(0x0307)
#define OX05B1S_REG_PLL3_CTRL_REG4A	CCI_REG8(0x034a)
#define OX05B1S_REG_PLL_MONITOR_REG0B	CCI_REG8(0x040b)
#define OX05B1S_REG_PLL_MONITOR_REG0C	CCI_REG8(0x040c)
#define OX05B1S_REG_SC_CMMN_REG09	CCI_REG8(0x3009)
#define OX05B1S_REG_GROUP_HLD_REG19	CCI_REG8(0x3219)
#define OX05B1S_REG_ANA_REG		CCI_REG8(0x3600)
#define OX05B1S_REG_SENSOR_CTRL02	CCI_REG8(0x3702)
#define OX05B1S_REG_TIMING_CTRL		CCI_REG8(0x3800)
#define OX05B1S_REG_MIPI_CORE_REG02	CCI_REG8(0x4802)
#define OX05B1S_REG_MIPI_CORE_REG1B	CCI_REG8(0x481b)
#define OX05B1S_REG_PCLK_PERIOD		CCI_REG8(0x4837)

/* OX05B1S 2592 x 1944 @30fps GRBG10 */
static const struct cci_reg_sequence ovx5b_init_setting_2592x1944[] = {
	{CCI_REG8(0x0107), 0x01}, /* Reserved */
	{OX05B1S_REG_PLL1_CTRL_REG07, 0x02},
	{OX05B1S_REG_PLL3_CTRL_REG4A, 0x05},
	{OX05B1S_REG_PLL_MONITOR_REG0B, 0x5c},
	{OX05B1S_REG_PLL_MONITOR_REG0C, 0xcd},
	{OX05B1S_REG_SC_CMMN_REG09, 0x2e},
	{OX05B1S_REG_GROUP_HLD_REG19, 0x08},
	{OX05B1S_REG_ANA_REG + 0x84, 0x6d},
	{OX05B1S_REG_ANA_REG + 0x85, 0x6d},
	{OX05B1S_REG_ANA_REG + 0x86, 0x6d},
	{OX05B1S_REG_ANA_REG + 0x87, 0x6d},
	{OX05B1S_REG_ANA_REG + 0x8c, 0x07},
	{OX05B1S_REG_ANA_REG + 0x8d, 0x07},
	{OX05B1S_REG_ANA_REG + 0x8e, 0x07},
	{OX05B1S_REG_ANA_REG + 0x8f, 0x00},
	{OX05B1S_REG_ANA_REG + 0x90, 0x04},
	{OX05B1S_REG_ANA_REG + 0x91, 0x04},
	{OX05B1S_REG_ANA_REG + 0x92, 0x04},
	{OX05B1S_REG_ANA_REG + 0x93, 0x04},
	{OX05B1S_REG_ANA_REG + 0x98, 0x00},
	{OX05B1S_REG_ANA_REG + 0xa0, 0x05},
	{OX05B1S_REG_ANA_REG + 0xa2, 0x16},
	{OX05B1S_REG_ANA_REG + 0xa3, 0x03},
	{OX05B1S_REG_ANA_REG + 0xa4, 0x07},
	{OX05B1S_REG_ANA_REG + 0xa5, 0x24},
	{OX05B1S_REG_ANA_REG + 0xe3, 0x09},
	{OX05B1S_REG_SENSOR_CTRL02, 0x0a},
	{OX05B1S_REG_TIMING_CTRL + 0x21, 0x04}, /* mirror */
	{OX05B1S_REG_TIMING_CTRL + 0x22, 0x10},
	{OX05B1S_REG_TIMING_CTRL + 0x2b, 0x03},
	{OX05B1S_REG_TIMING_CTRL + 0x66, 0x10},
	{OX05B1S_REG_TIMING_CTRL + 0x6c, 0x46},
	{OX05B1S_REG_TIMING_CTRL + 0x6d, 0x08},
	{OX05B1S_REG_TIMING_CTRL + 0x6e, 0x7b},
	{OX05B1S_REG_MIPI_CORE_REG02, 0x00},
	{OX05B1S_REG_MIPI_CORE_REG1B, 0x3c},
	{OX05B1S_REG_PCLK_PERIOD, 0x19},
};

const struct ox05b1s_reglist ox05b1s_reglist_2592x1944[] = {
	{
		.regs = ovx5b_init_setting_2592x1944,
		.count = ARRAY_SIZE(ovx5b_init_setting_2592x1944),
	},
	{
		/* sentinel */
	}
};
