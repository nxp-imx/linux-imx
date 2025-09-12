// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 NXP
 */

#include <linux/pcs/pcs-xpcs.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include "pcs-xpcs.h"

#define XPCS_DEV		0x0
#define XPCS_PHY_DEV		0x10
#define XPCS_POLLING_DELAY_US	2
#define XPCS_POLLING_TIMEOUT_US	30000

#define XPCS_PHY_GLOBAL		0x0
#define XPCS_PHY_MPLLA		0x1
#define XPCS_PHY_MPLLB		0x2
#define XPCS_PHY_LANE		0x3
#define XPCS_PHY_MAC_ADAPTER	0x1f

#define XPCS_PHY_REG(x) (((x) & 0x1fffe) >> 1)

/* MAC ADAPTER */
#define MAC_ADAPTER_LOCK_PHY	0x200
#define MAC_ADAPTER_LOCK_MPLLA	0x204
#define MAC_ADAPTER_LOCK_MPLLB	0x208
#define MAC_ADAPTER_LOCK_ROM	0x20c
#define MAC_ADAPTER_LOCK_RAM	0x210
#define MAC_ADAPTER_LOCK_EVENT	0x214

#define MAC_ADAPTER_LOCK_LOCK_OWNER_MASK	GENMASK(3, 0)
#define MAC_ADAPTER_LOCK_LOCK_OWNER_SHIFT	0
#define MAC_ADAPTER_LOCK_LOCK			BIT(7)
#define MAC_ADAPTER_LOCK_LOCK_WHOAMI_MASK	GENMASK(15, 12)
#define MAC_ADAPTER_LOCK_LOCK_WHOAMI_SHIFT	12

/* PMA */
#define PMA_RX_LSTS					0x10040
#define PMA_RX_LSTS_RX_VALID_0			BIT(12)
#define PMA_MP_12G_16G_25G_TX_GENCTRL0			0x10060
#define PMA_TX_GENCTRL0_TX_RST_0		BIT(8)
#define PMA_TX_GENCTRL0_TX_DT_EN_0		BIT(12)
#define PMA_MP_12G_16G_25G_TX_GENCTRL1			0x10062
#define PMA_TX_GENCTRL1_VBOOST_EN_0		BIT(4)
#define PMA_TX_GENCTRL1_VBOOST_LVL_MASK		GENMASK(10, 8)
#define PMA_TX_GENCTRL1_VBOOST_LVL(x)		(((x) << 8) & GENMASK(10, 8))
#define PMA_TX_GENCTRL1_TX_CLK_RDY_0		BIT(12)
#define PMA_MP_12G_16G_TX_GENCTRL2			0x10064
#define PMA_TX_GENCTRL2_TX_REQ_0		BIT(0)
#define PMA_TX_GENCTRL2_TX0_WIDTH_MASK		GENMASK(9, 8)
#define PMA_TX_GENCTRL2_TX0_WIDTH(x)		(((x) << 8) & GENMASK(9, 8))
#define PMA_MP_12G_16G_25G_TX_BOOST_CTRL		0x10066
#define PMA_TX_BOOST_CTRL_TX0_IBOOST_MASK	GENMASK(3, 0)
#define PMA_TX_BOOST_CTRL_TX0_IBOOST(x)		((x) & GENMASK(3, 0))
#define PMA_MP_12G_16G_25G_TX_RATE_CTRL			0x10068
#define PMA_TX_RATE_CTRL_TX0_RATE_MASK		GENMASK(2, 0)
#define PMA_TX_RATE_CTRL_TX0_RATE(x)		((x) & GENMASK(2, 0))
#define PMA_MP_12G_16G_25G_TX_POWER_STATE_CTRL		0x1006A
#define PMA_POWER_STATE_CTRL_TX0_PSTATE_MASK	GENMASK(1, 0)
#define PMA_POWER_STATE_CTRL_TX0_PSTATE(x)	((x) & GENMASK(1, 0))
#define PMA_POWER_STATE_CTRL_TX_DISABLE_0	BIT(8)
#define PMA_MP_12G_16G_25G_TX_EQ_CTRL0			0x1006C
#define PMA_TX_EQ_CTRL0_TX_EQ_PRE_MASK		GENMASK(5, 0)
#define PMA_TX_EQ_CTRL0_TX_EQ_PRE(x)		((x) & GENMASK(5, 0))
#define PMA_TX_EQ_CTRL0_TX_EQ_MAIN_MASK		GENMASK(13, 8)
#define PMA_TX_EQ_CTRL0_TX_EQ_MAIN(x)		(((x) << 8) & GENMASK(13, 8))
#define PMA_MP_12G_16G_25G_TX_EQ_CTRL1			0x1006E
#define PMA_TX_EQ_CTRL1_TX_EQ_POST_MASK		GENMASK(5, 0)
#define PMA_TX_EQ_CTRL1_TX_EQ_POST(x)		((x) & GENMASK(5, 0))
#define PMA_MP_16G_25G_TX_MISC_CTRL0			0x1007C
#define PMA_TX_MISC_CTRL0_TX0_MISC_MASK		GENMASK(7, 0)
#define PMA_TX_MISC_CTRL0_TX0_MISC(x)		((x) & GENMASK(7, 0))
#define PMA_MP_12G_16G_25G_TX_STS			0x10080
#define PMA_TX_STS_TX_ACK_0			BIT(0)
#define PMA_MP_12G_16G_25G_RX_GENCTRL0			0x100A0
#define PMA_RX_GENCTRL0_RX_DT_EN_0		BIT(8)
#define PMA_MP_12G_16G_25G_RX_GENCTRL1			0x100A2
#define PMA_RX_GENCTRL1_RX_RST_0		BIT(4)
#define PMA_RX_GENCTRL1_RX_TERM_ACDC_0		BIT(8)
#define PMA_RX_GENCTRL1_RX_DIV16P5_CLK_EN_0	BIT(12)
#define PMA_MP_12G_16G_RX_GENCTRL2			0x100A4
#define PMA_RX_GENCTRL2_RX_REQ_0		BIT(0)
#define PMA_RX_GENCTRL2_RX0_WIDTH_MASK		GENMASK(9, 8)
#define PMA_RX_GENCTRL2_RX0_WIDTH(x)		(((x) << 8) & GENMASK(9, 8))
#define PMA_MP_12G_16G_RX_GENCTRL3			0x100A6
#define PMA_RX_GENCTRL3_LOS_TRSHLD_0_MASK	GENMASK(2, 0)
#define PMA_RX_GENCTRL3_LOS_TRSHLD_0(x)		((x) & GENMASK(2, 0))
#define PMA_RX_GENCTRL3_LOS_LFPS_EN_0		BIT(12)
#define PMA_MP_12G_16G_25G_RX_RATE_CTRL			0x100A8
#define PMA_RX_RATE_CTRL_RX0_RATE_MASK		GENMASK(1, 0)
#define PMA_RX_RATE_CTRL_RX0_RATE(x)		((x) & GENMASK(1, 0))
#define PMA_MP_12G_16G_25G_RX_POWER_STATE_CTRL		0x100AA
#define PMA_RX_POWER_STATE_CTRL_RX0_PSTATE_MASK	GENMASK(1, 0)
#define PMA_RX_POWER_STATE_CTRL_RX0_PSTATE(x)	((x) & GENMASK(1, 0))
#define PMA_RX_POWER_STATE_CTRL_RX_DISABLE_0	BIT(8)
#define PMA_MP_12G_16G_25G_RX_CDR_CTRL			0x100AC
#define PMA_RX_CDR_CTRL_CDR_SSC_EN_0		BIT(4)
#define PMA_MP_12G_16G_25G_RX_ATTN_CTRL			0x100AE
#define PMA_RX_ATTN_CTRL_RX0_EQ_ATT_LVL_MASK	GENMASK(2, 0)
#define PMA_RX_ATTN_CTRL_RX0_EQ_ATT_LVL(x)	((x) & GENMASK(2, 0))
#define PMA_MP_16G_25G_RX_EQ_CTRL0			0x100B0
#define PMA_RX_EQ_CTRL0_CTLE_BOOST_0_MASK	GENMASK(4, 0)
#define PMA_RX_EQ_CTRL0_CTLE_BOOST_0(x)		((x) & GENMASK(4, 0))
#define PMA_RX_EQ_CTRL0_CTLE_POLE_0_MASK	GENMASK(6, 5)
#define PMA_RX_EQ_CTRL0_CTLE_POLE_0(x)		(((x) << 5) & GENMASK(6, 5))
#define PMA_RX_EQ_CTRL0_VGA2_GAIN_0_MASK	GENMASK(10, 8)
#define PMA_RX_EQ_CTRL0_VGA2_GAIN_0(x)		(((x) << 8) & GENMASK(10, 8))
#define PMA_RX_EQ_CTRL0_VGA1_GAIN_0_MASK	GENMASK(14, 12)
#define PMA_RX_EQ_CTRL0_VGA1_GAIN_0(x)		(((x) << 12) & GENMASK(14, 12))
#define PMA_MP_12G_16G_25G_RX_EQ_CTRL4			0x100B8
#define PMA_RX_EQ_CTRL4_CONT_ADAPT_0		BIT(0)
#define PMA_RX_EQ_CTRL4_RX_AD_REQ		BIT(12)
#define PMA_MP_16G_25G_RX_EQ_CTRL5			0x100BA
#define PMA_RX_EQ_CTRL5_RX_ADPT_SEL_0		BIT(0)
#define PMA_RX_EQ_CTRL5_RX0_ADPT_MODE_MASK	GENMASK(5, 4)
#define PMA_RX_EQ_CTRL5_RX0_ADPT_MODE(x)	(((x) << 4) & GENMASK(5, 4))
#define PMA_MP_12G_16G_25G_DFE_TAP_CTRL0		0x100BC
#define PMA_DFE_TAP_CTRL0_DFE_TAP1_0_MASK	GENMASK(7, 0)
#define PMA_DFE_TAP_CTRL0_DFE_TAP1_0(x)		((x) & GENMASK(7, 0))
#define PMA_MP_12G_16G_25G_RX_STS			0x100C0
#define PMA_RX_STS_RX_ACK_0				BIT(0)
#define PMA_MP_16G_RX_CDR_CTRL1				0x100C8
#define PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0	BIT(0)
#define PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0	BIT(4)
#define PMA_RX_CDR_CTRL1_VCO_FRQBAND_0_MASK	GENMASK(9, 8)
#define PMA_RX_CDR_CTRL1_VCO_FRQBAND_0(x)	(((x) << 8) & GENMASK(9, 8))
#define PMA_MP_16G_25G_RX_PPM_CTRL0			0x100CA
#define PMA_RX_PPM_CTRL0_RX0_CDR_PPM_MAX_MASK	GENMASK(4, 0)
#define PMA_RX_PPM_CTRL0_RX0_CDR_PPM_MAX(x)	((x) & GENMASK(4, 0))
#define PMA_MP_16G_25G_RX_GENCTRL4			0x100D0
#define PMA_RX_GENCTRL4_RX_DFE_BYP_0		BIT(8)
#define PMA_MP_16G_25G_RX_MISC_CTRL0			0x100D2
#define PMA_RX_MISC_CTRL0_RX0_MISC_MASK		GENMASK(7, 0)
#define PMA_RX_MISC_CTRL0_RX0_MISC(x)		((x) & GENMASK(7, 0))
#define PMA_MP_16G_25G_RX_IQ_CTRL0			0x100D6
#define PMA_RX_IQ_CTRL0_RX0_MARGIN_IQ_MASK	GENMASK(6, 0)
#define PMA_RX_IQ_CTRL0_RX0_MARGIN_IQ(x)	((x) & GENMASK(6, 0))
#define PMA_RX_IQ_CTRL0_RX0_DELTA_IQ_MASK	GENMASK(11, 8)
#define PMA_RX_IQ_CTRL0_RX0_DELTA_IQ(x)		(((x) << 8) & GENMASK(11, 8))
#define PMA_MP_12G_16G_25G_MPLL_CMN_CTRL		0x100E0
#define PMA_MPLL_CMN_CTRL_MPLL_EN_0		BIT(0)
#define PMA_MPLL_CMN_CTRL_MPLLB_SEL_0		BIT(4)
#define PMA_MP_12G_16G_MPLLA_CTRL0			0x100E2
#define PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER_MASK	GENMASK(7, 0)
#define PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER(x)	((x) & GENMASK(7, 0))
#define PMA_MPLLA_CTRL0_MPLLA_CAL_DISABLE	BIT(15)
#define PMA_MP_16G_MPLLA_CTRL1				0x100E4
#define PMA_MPLLA_CTRL1_MPLLA_SSC_EN		BIT(0)
#define PMA_MPLLA_CTRL1_MPLLA_SSC_CLK_SEL	BIT(4)
#define PMA_MPLLA_CTRL1_MPLLA_FRACN_CTRL_MASK	GENMASK(15, 5)
#define PMA_MPLLA_CTRL1_MPLLA_FRACN_CTRL(x)	(((x) << 5) & GENMASK(15, 5))
#define PMA_MP_12G_16G_MPLLA_CTRL2			0x100E6
#define PMA_MPLLA_CTRL2_MPLLA_DIV_MULT_MASK	GENMASK(6, 0)
#define PMA_MPLLA_CTRL2_MPLLA_DIV_MULT(x)	((x) & GENMASK(6, 0))
#define PMA_MPLLA_CTRL2_MPLLA_DIV_CLK_EN	BIT(7)
#define PMA_MPLLA_CTRL2_MPLLA_DIV8_CLK_EN	BIT(8)
#define PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN	BIT(9)
#define PMA_MPLLA_CTRL2_MPLLA_DIV16P5_CLK_EN	BIT(10)
#define PMA_MPLLA_CTRL2_MPLLA_TX_CLK_DIV_MASK	GENMASK(12, 11)
#define PMA_MPLLA_CTRL2_MPLLA_TX_CLK_DIV(x)	(((x) << 11) & GENMASK(12, 11))
#define PMA_MP_12G_16G_MPLLB_CTRL0			0x100E8
#define PMA_MPLLB_CTRL0_MPLLB_MULTIPLIER_MASK	GENMASK(7, 0)
#define PMA_MPLLB_CTRL0_MPLLB_MULTIPLIER(x)	((x) & GENMASK(7, 0))
#define PMA_MPLLB_CTRL0_MPLLB_CAL_DISABLE	BIT(15)
#define PMA_MP_12G_16G_MPLLB_CTRL1			0x100EA
#define PMA_MPLLB_CTRL1_MPLLB_SSC_EN		BIT(0)
#define PMA_MPLLB_CTRL1_MPLLB_SSC_CLK_SEL	BIT(4)
#define PMA_MPLLB_CTRL1_MPLLB_FRACN_CTRL_MASK	GENMASK(15, 5)
#define PMA_MPLLB_CTRL1_MPLLB_FRACN_CTRL(x)	(((x) << 5) & GENMASK(15, 5))
#define PMA_MP_12G_16G_MPLLB_CTRL2			0x100EC
#define PMA_MPLLB_CTRL2_MPLLB_DIV_MULT_MASK	GENMASK(6, 0)
#define PMA_MPLLB_CTRL2_MPLLB_DIV_MULT(x)	((x) & GENMASK(6, 0))
#define PMA_MPLLB_CTRL2_MPLLB_DIV_CLK_EN	BIT(7)
#define PMA_MPLLB_CTRL2_MPLLB_DIV8_CLK_EN	BIT(8)
#define PMA_MPLLB_CTRL2_MPLLB_DIV10_CLK_EN	BIT(9)
#define PMA_MPLLB_CTRL2_MPLLB_TX_CLK_DIV_MASK	GENMASK(12, 11)
#define PMA_MPLLB_CTRL2_MPLLB_TX_CLK_DIV(x)	(((x) << 11) & GENMASK(12, 11))
#define PMA_MP_16G_MPLLA_CTRL3				0x100EE
#define PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH_MASK	GENMASK(15, 0)
#define PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH(x)	((x) & GENMASK(15, 0))
#define PMA_MP_16G_MPLLB_CTRL3				0x100F0
#define PMA_MPLL_CTRL3_MPLLB_BANDWIDTH_MASK	GENMASK(15, 0)
#define PMA_MPLL_CTRL3_MPLLB_BANDWIDTH(x)	((x) & GENMASK(15, 0))
#define PMA_MP_16G_MPLLA_CTRL4				0x100F2
#define PMA_MPLLA_CTRL4_MPLLA_SSC_FRQ_CNT_INT_MASK GENMASK(11, 0)
#define PMA_MPLLA_CTRL4_MPLLA_SSC_FRQ_CNT_INT(x) ((x) & GENMASK(11, 0))
#define PMA_MP_16G_MPLLA_CTRL5				0x100F4
#define PMA_MPLLA_CTRL5_MPLLA_SSC_FRQ_CNT_PK_MASK GENMASK(7, 0)
#define PMA_MPLLA_CTRL5_MPLLA_SSC_FRQ_CNT_PK(x)	((x) & GENMASK(7, 0))
#define PMA_MPLLA_CTRL5_MPLLA_SSC_SPD_EN	BIT(8)
#define PMA_MP_16G_MPLLB_CTRL4				0x100F6
#define PMA_MPLLB_CTRL4_MPLLB_SSC_FRQ_CNT_INT_MASK GENMASK(11, 0)
#define PMA_MPLLB_CTRL4_MPLLB_SSC_FRQ_CNT_INT(x) ((x) & GENMASK(11, 0))
#define PMA_MP_16G_MPLLB_CTRL5				0x100F8
#define PMA_MPLLB_CTRL5_MPLLB_SSC_FRQ_CNT_PK_MASK GENMASK(7, 0)
#define PMA_MPLLB_CTRL5_MPLLB_SSC_FRQ_CNT_PK(x)	((x) & GENMASK(7, 0))
#define PMA_MPLLB_CTRL5_MPLLB_SSC_SPD_EN	BIT(8)
#define PMA_MP_12G_16G_25G_MISC_CTRL0			0x10120
#define PMA_MISC_CTRL0_RX_VREF_CTRL_MASK	GENMASK(12, 8)
#define PMA_MISC_CTRL0_RX_VREF_CTRL(x)		(((x) << 8) & GENMASK(12, 8))
#define PMA_MP_12G_16G_25G_REF_CLK_CTRL			0x10122
#define PMA_REF_CLK_CTRL_REF_CLK_DIV2		BIT(2)
#define PMA_REF_CLK_CTRL_REF_RANGE_MASK		GENMASK(5, 3)
#define PMA_REF_CLK_CTRL_REF_RANGE(x)		(((x) << 3) & GENMASK(5, 3))
#define PMA_REF_CLK_CTRL_REF_MPLLA_DIV2		BIT(6)
#define PMA_REF_CLK_CTRL_REF_MPLLB_DIV2		BIT(7)
#define PMA_MP_12G_16G_25G_VCO_CAL_LD0			0x10124
#define PMA_VCO_CAL_LD0_VCO_LD_VAL_0_MASK	GENMASK(12, 0)
#define PMA_VCO_CAL_LD0_VCO_LD_VAL_0(x)		((x) & GENMASK(12, 0))
#define PMA_MP_16G_25G_VCO_CAL_REF0			0x1012C
#define PMA_VCO_CAL_REF0_VCO_REF_LD_0_MASK	GENMASK(6, 0)
#define PMA_VCO_CAL_REF0_VCO_REF_LD_0(x)	((x) & GENMASK(6, 0))
#define PMA_MP_12G_16G_25G_MISC_STS			0x10130
#define PMA_MISC_STS_RX_ADPT_ACK		BIT(12)
#define PMA_MP_12G_16G_25G_SRAM				0x10136
#define PMA_SRAM_INIT_DN			BIT(0)
#define PMA_SRAM_EXT_LD_DN			BIT(1)
#define PMA_MP_16G_25G_MISC_CTRL2			0x10138
#define PMA_MISC_CTRL2_SUP_MISC_MASK		GENMASK(7, 0)
#define PMA_MISC_CTRL2_SUP_MISC(x)		((x) & GENMASK(7, 0))

/* PCS */
#define PCS_CTRL1				0x0
#define PCS_CTRL1_RESET			BIT(15)
#define PCS_CTRL2				0xE
#define PCS_CTRL2_PCS_TYPE_SEL_MASK	GENMASK(3, 0)
#define PCS_CTRL2_PCS_TYPE_SEL(x)	((x) & GENMASK(3, 0))
#define PCS_DIG_CTRL1				0x10000
#define PCS_DIG_CTRL1_EN_2_5G_MODE	BIT(2)
#define PCS_DIG_CTRL1_USXG_EN		BIT(9)
#define PCS_DIG_CTRL1_USRA_RST		BIT(10)
#define PCS_DIG_CTRL1_VR_RST		BIT(15)
#define PCS_XAUI_CTRL				0x10008
#define PCS_XAUI_CTRL_XAUI_MODE		BIT(0)
#define PCS_DEBUG_CTRL				0x1000A
#define PCS_DEBUG_CTRL_SUPRESS_LOS_DET	BIT(4)
#define PCS_DEBUG_CTRL_RX_DT_EN_CTL	BIT(6)
#define PCS_DEBUG_CTRL_TX_PMBL_CTL	BIT(8)
#define PCS_KR_CTRL1				0x1000E
#define PCS_KR_CTRL1_USXG_MODE_MASK	GENMASK(12, 10)
#define PCS_KR_CTRL1_USXG_MODE(x)	(((x) << 10) & GENMASK(12, 10))

/* VS MII MMD */
#define MII_CTRL					0x0
#define MII_CTRL_SS5				BIT(5)
#define MII_CTRL_SS6				BIT(6)
#define MII_CTRL_AN_ENABLE			BIT(12)
#define MII_CTRL_SS13				BIT(13)
#define MII_CTRL_RST				BIT(15)
#define MII_STS						0x2
#define MII_STS_LINK_STS			BIT(2)
#define MII_DIG_CTRL1					0x10000
#define MII_DIG_CTRL1_EN_2_5G_MODE		BIT(2)
#define MII_DIG_CTRL1_CL37_TMR_OVR_RIDE		BIT(3)
#define MII_DIG_CTRL1_VR_RST			BIT(15)
#define MII_AN_CTRL					0x10002
#define MII_AN_CTRL_MII_AN_INTR_EN		BIT(0)
#define MII_AN_CTRL_TX_CONFIG			BIT(3)
#define MII_AN_INTR_STS					0x10004
#define MII_AN_INTR_STS_CL37_ANCMPLT_INTR	BIT(0)
#define MII_LINK_TIMER_CTRL				0x10014
#define MII_LINK_TIMER_CTRL_CL37_LINK_TIME_MASK		GENMASK(15, 0)
#define MII_LINK_TIMER_CTRL_CL37_LINK_TIME(x)		((x) & GENMASK(15, 0))

/* E16 MEM MAP */
#define IDCODE_LO						0x0
#define IDCODE_HI						0x4
#define GLOBAL_CTRL_EX_0					0x114
#define GLOBAL_CTRL_EX_0_PHY_SRAM_BYPASS		BIT(0)
#define GLOBAL_CTRL_EX_0_XPCS0_SEL			BIT(4)
#define GLOBAL_CTRL_EX_0_XPCS1_SEL			BIT(5)
#define GLOBAL_CTRL_EX_0_MPLLA_SEL			BIT(6)
#define GLOBAL_CTRL_EX_0_MPLLB_SEL			BIT(7)
#define GLOBAL_CTRL_EX_0_PHY_SUP_MISC_MASK		GENMASK(15, 8)
#define GLOBAL_CTRL_EX_0_PHY_SUP_MISC(x)		(((x) << 8) & GENMASK(15, 8))
#define GLOBAL_CTRL_EX_4					0x124
#define GLOBAL_CTRL_EX_4_PHY_PCS_PWR_STABLE		BIT(8)
#define GLOBAL_CTRL_EX_4_PHY_PMA_PWR_STABLE		BIT(14)
#define MPLLA_CTRL_EX_0						0xac
#define MPLLA_CTRL_EX_0_MPLLA_CFG_DRIVER_MASK		GENMASK(11, 8)
#define MPLLA_CTRL_EX_0_MPLLA_CFG_DRIVER_SHIFT		(8)
#define MPLLA_CTRL_EX_0_MPLLA_CFG_DRIVER		(((x) << 8) & GENMASK(11, 8))
#define MPLLB_CTRL_EX_0						0xac
#define MPLLB_CTRL_EX_0_MPLLB_CFG_DRIVER_MASK		GENMASK(11, 8)
#define MPLLB_CTRL_EX_0_MPLLB_CFG_DRIVER_SHIFT		(8)
#define MPLLB_CTRL_EX_0_MPLLB_CFG_DRIVER		(((x) << 8) & GENMASK(11, 8))
#define L0_RX_VCO_OVRD_OUT_0					0x20c
#define L0_RX_VCO_OVRD_OUT_0_RX_ANA_CDR_FREQ_TUNE_MASK	GENMASK(12, 3)
#define L0_RX_VCO_OVRD_OUT_0_RX_ANA_CDR_FREQ_TUNE(x)	(((x) << 3) & GENMASK(12, 3))
#define L0_RX_VCO_OVRD_OUT_0_RX_CDR_FREQ_TUNE_OVRD_EN	BIT(15)
#define L0_RX_VCO_OVRD_OUT_2					0x214
#define L0_RX_VCO_OVRD_OUT_2_RX_ANA_CDR_FREQ_TUNE_CLK	BIT(0)

#define XPCS_G_ALL_BITS					0xFFFF

static int xpcs_phy_read(struct dw_xpcs *xpcs, u8 devad, u32 reg)
{
	return mdiodev_c45_read(xpcs->phydev, devad, reg);
}

static int xpcs_phy_write(struct dw_xpcs *xpcs, u8 devad, u32 reg, u16 val)
{
	return mdiodev_c45_write(xpcs->phydev, devad, reg, val);
}

static int xpcs_phy_modify(struct dw_xpcs *xpcs, u8 dev, u8 devad, u32 reg, u16 mask, u16 set)
{
	int ret;

	switch (dev) {
	case XPCS_DEV:
		ret = mdiodev_c45_modify(xpcs->mdiodev, devad, XPCS_PHY_REG(reg), mask, set);
		break;
	case XPCS_PHY_DEV:
		ret = mdiodev_c45_modify(xpcs->phydev, devad, XPCS_PHY_REG(reg), mask, set);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int xpcs_phy_polling_timeout(struct dw_xpcs *xpcs, u8 dev, u8 devad,
				    u32 reg, u16 mask, u8 wait_for)
{
	u16 val;
	int ret;

	switch (dev) {
	case XPCS_DEV:
		ret = read_poll_timeout(xpcs_read, val,
					((val & mask) == (wait_for ? mask : 0)),
					XPCS_POLLING_DELAY_US, XPCS_POLLING_TIMEOUT_US,
					false, xpcs, devad, XPCS_PHY_REG(reg));
		break;
	case XPCS_PHY_DEV:
		ret = read_poll_timeout(xpcs_phy_read, val,
					((val & mask) == (wait_for ? mask : 0)),
					XPCS_POLLING_DELAY_US, XPCS_POLLING_TIMEOUT_US,
					false, xpcs, devad, XPCS_PHY_REG(reg));
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

void xpcs_phy_reset(struct dw_xpcs *xpcs)
{
	xpcs_write(xpcs, MDIO_MMD_PCS, XPCS_PHY_REG(PCS_CTRL1), PCS_CTRL1_RESET);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_GENCTRL0,
			PMA_TX_GENCTRL0_TX_RST_0, PMA_TX_GENCTRL0_TX_RST_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_RST_0, PMA_RX_GENCTRL1_RX_RST_0);
}

static void mx95_xpcs_phy_reg_lock(struct dw_xpcs *xpcs)
{
	int ret;

	if (xpcs_phy_read(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_PHY)) &
	    MAC_ADAPTER_LOCK_LOCK)
		return;

	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_PHY),
		       MAC_ADAPTER_LOCK_LOCK);
	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_MPLLA),
		       MAC_ADAPTER_LOCK_LOCK);
	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_MPLLB),
		       MAC_ADAPTER_LOCK_LOCK);
	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_ROM),
		       MAC_ADAPTER_LOCK_LOCK);
	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_RAM),
		       MAC_ADAPTER_LOCK_LOCK);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_SRAM,
				       PMA_SRAM_INIT_DN, 1);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	/* Work around */
	xpcs_phy_write(xpcs, XPCS_PHY_GLOBAL, XPCS_PHY_REG(GLOBAL_CTRL_EX_0),
		       GLOBAL_CTRL_EX_0_PHY_SRAM_BYPASS);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_CTRL1, PCS_CTRL1_RESET, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	mdelay(1);

timeout:
	return;
}

static int mx94_xpcs_phy_reg_lock(struct dw_xpcs *xpcs)
{
	u8 whoami, owner;
	u16 val;
	int ret;

	val = xpcs_phy_read(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_PHY));
	whoami = (val & MAC_ADAPTER_LOCK_LOCK_WHOAMI_MASK) >> MAC_ADAPTER_LOCK_LOCK_WHOAMI_SHIFT;
	owner = (val & MAC_ADAPTER_LOCK_LOCK_OWNER_MASK) >> MAC_ADAPTER_LOCK_LOCK_OWNER_SHIFT;
	if (whoami != owner) {
		ret = xpcs_phy_polling_timeout(xpcs, XPCS_PHY_DEV, XPCS_PHY_MAC_ADAPTER,
					       MAC_ADAPTER_LOCK_PHY, MAC_ADAPTER_LOCK_LOCK, 0);
		if (ret) {
			dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
			goto timeout;
		}
	} else {
		return 0;
	}

	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_PHY),
		       MAC_ADAPTER_LOCK_LOCK);
	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_MPLLA),
		       MAC_ADAPTER_LOCK_LOCK);
	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_MPLLB),
		       MAC_ADAPTER_LOCK_LOCK);
	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_ROM),
		       MAC_ADAPTER_LOCK_LOCK);

	return 0;

timeout:

	return -ETIMEDOUT;
}

static int xpcs_phy_reg_lock(struct dw_xpcs *xpcs)
{
	int ret = 0;

	switch (xpcs->info.pma) {
	case NXP_MX95_XPCS_ID:
		mx95_xpcs_phy_reg_lock(xpcs);
		break;
	case NXP_MX94_XPCS_ID:
		ret = mx94_xpcs_phy_reg_lock(xpcs);
		break;
	default:
		dev_err(&xpcs->phydev->dev, "Unknown PMA ID: %d\n", xpcs->info.pma);
		ret = -ENODEV;
	}

	return ret;
}

static int mx94_xpcs_phy_reg_unlock(struct dw_xpcs *xpcs)
{
	u8 whoami, owner;
	u16 val;

	val = xpcs_phy_read(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_PHY));
	whoami = (val & MAC_ADAPTER_LOCK_LOCK_WHOAMI_MASK) >> MAC_ADAPTER_LOCK_LOCK_WHOAMI_SHIFT;
	owner = (val & MAC_ADAPTER_LOCK_LOCK_OWNER_MASK) >> MAC_ADAPTER_LOCK_LOCK_OWNER_SHIFT;
	if (whoami != owner) {
		dev_err(&xpcs->phydev->dev, "PHY is locked by: %d, cannot unlock!\n", owner);
		return -EBUSY;
	}

	mdelay(10);

	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_PHY), 0);
	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_MPLLA), 0);
	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_MPLLB), 0);
	xpcs_phy_write(xpcs, XPCS_PHY_MAC_ADAPTER, XPCS_PHY_REG(MAC_ADAPTER_LOCK_ROM), 0);

	return 0;
}

static int xpcs_phy_reg_unlock(struct dw_xpcs *xpcs)
{
	int ret = 0;

	switch (xpcs->info.pma) {
	case NXP_MX95_XPCS_ID:
		break;
	case NXP_MX94_XPCS_ID:
		ret = mx94_xpcs_phy_reg_unlock(xpcs);
		break;
	default:
		dev_err(&xpcs->phydev->dev, "Unknown PMA ID: %d\n",
			xpcs->info.pma);
		ret = -ENODEV;
	}

	return ret;
}

static int imx94_xpcs_phy_port_init(struct dw_xpcs *xpcs, bool is_2p5g)
{
	u16 val;
	int ret;

	if (is_2p5g) {
		val = xpcs_phy_read(xpcs, XPCS_PHY_MPLLA, XPCS_PHY_REG(MPLLA_CTRL_EX_0));
		val &= ~MPLLA_CTRL_EX_0_MPLLA_CFG_DRIVER_MASK;
		val |= xpcs->portid << MPLLA_CTRL_EX_0_MPLLA_CFG_DRIVER_SHIFT;
		xpcs_phy_write(xpcs, XPCS_PHY_MPLLA, XPCS_PHY_REG(MPLLA_CTRL_EX_0), val);
	} else {
		val = xpcs_phy_read(xpcs, XPCS_PHY_MPLLB, XPCS_PHY_REG(MPLLB_CTRL_EX_0));
		val &= ~MPLLB_CTRL_EX_0_MPLLB_CFG_DRIVER_MASK;
		val |= xpcs->portid << MPLLB_CTRL_EX_0_MPLLB_CFG_DRIVER_SHIFT;
		xpcs_phy_write(xpcs, XPCS_PHY_MPLLB, XPCS_PHY_REG(MPLLB_CTRL_EX_0), val);
	}

	xpcs_phy_modify(xpcs, XPCS_PHY_DEV, XPCS_PHY_GLOBAL, GLOBAL_CTRL_EX_4,
			GLOBAL_CTRL_EX_4_PHY_PCS_PWR_STABLE, GLOBAL_CTRL_EX_4_PHY_PCS_PWR_STABLE);
	xpcs_phy_modify(xpcs, XPCS_PHY_DEV, XPCS_PHY_GLOBAL, GLOBAL_CTRL_EX_4,
			GLOBAL_CTRL_EX_4_PHY_PMA_PWR_STABLE, GLOBAL_CTRL_EX_4_PHY_PMA_PWR_STABLE);

	val = xpcs_phy_read(xpcs, XPCS_PHY_GLOBAL, XPCS_PHY_REG(GLOBAL_CTRL_EX_0));
	if (xpcs->portid == 0)
		val = is_2p5g ? (val & ~GLOBAL_CTRL_EX_0_MPLLB_SEL) :
				(val | GLOBAL_CTRL_EX_0_MPLLB_SEL);
	else
		val = is_2p5g ? (val & ~GLOBAL_CTRL_EX_0_MPLLA_SEL) :
				(val | GLOBAL_CTRL_EX_0_MPLLA_SEL);
	xpcs_phy_write(xpcs, XPCS_PHY_GLOBAL, XPCS_PHY_REG(GLOBAL_CTRL_EX_0), val);

	xpcs_phy_modify(xpcs, XPCS_PHY_DEV, XPCS_PHY_GLOBAL, GLOBAL_CTRL_EX_0,
			xpcs->portid ? GLOBAL_CTRL_EX_0_XPCS1_SEL : GLOBAL_CTRL_EX_0_XPCS0_SEL,
			xpcs->portid ? GLOBAL_CTRL_EX_0_XPCS1_SEL : GLOBAL_CTRL_EX_0_XPCS0_SEL);

	mdelay(1);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_SRAM,
				       PMA_SRAM_INIT_DN, 1);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	mdelay(1);

	xpcs_phy_modify(xpcs, XPCS_PHY_DEV, XPCS_PHY_GLOBAL, GLOBAL_CTRL_EX_0,
			GLOBAL_CTRL_EX_0_PHY_SRAM_BYPASS, GLOBAL_CTRL_EX_0_PHY_SRAM_BYPASS);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PCS_CTRL1, PCS_CTRL1_RESET,
				       0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

		return 0;

timeout:

	return -ETIMEDOUT;
}

static int xpcs_phy_common_init_seq_1(struct dw_xpcs *xpcs, bool has_pcs_pma, bool an)
{
	int ret;
	u8 devad = has_pcs_pma ? MDIO_MMD_PMAPMD : MDIO_MMD_VEND2;

	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_CTRL, MII_CTRL_AN_ENABLE,
			an ? MII_CTRL_AN_ENABLE : 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_GENCTRL0,
			PMA_TX_GENCTRL0_TX_RST_0, PMA_TX_GENCTRL0_TX_RST_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_RST_0, PMA_RX_GENCTRL1_RX_RST_0);

	mdelay(1);

	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_GENCTRL0,
			PMA_TX_GENCTRL0_TX_RST_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_RST_0, 0);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_STS,
				       PMA_TX_STS_TX_ACK_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_POWER_STATE_CTRL,
			PMA_POWER_STATE_CTRL_TX0_PSTATE_MASK, PMA_POWER_STATE_CTRL_TX0_PSTATE(3));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_MPLL_CMN_CTRL,
			PMA_MPLL_CMN_CTRL_MPLL_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_GENCTRL0,
			PMA_TX_GENCTRL0_TX_DT_EN_0, 0);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_STS,
				       PMA_RX_STS_RX_ACK_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_GENCTRL0,
			PMA_RX_GENCTRL0_RX_DT_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_POWER_STATE_CTRL,
			PMA_RX_POWER_STATE_CTRL_RX0_PSTATE_MASK,
			PMA_RX_POWER_STATE_CTRL_RX0_PSTATE(1));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_POWER_STATE_CTRL,
			PMA_RX_POWER_STATE_CTRL_RX0_PSTATE_MASK,
			PMA_RX_POWER_STATE_CTRL_RX0_PSTATE(3));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_TX_GENCTRL2,
			PMA_TX_GENCTRL2_TX_REQ_0, PMA_TX_GENCTRL2_TX_REQ_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX_REQ_0, PMA_RX_GENCTRL2_RX_REQ_0);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_TX_GENCTRL2,
				       PMA_TX_GENCTRL2_TX_REQ_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL2,
				       PMA_RX_GENCTRL2_RX_REQ_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	mdelay(1);

	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_TX_GENCTRL2, PMA_TX_GENCTRL2_TX_REQ_0,
			0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL2, PMA_RX_GENCTRL2_RX_REQ_0,
			0);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_STS,
				       PMA_TX_STS_TX_ACK_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_STS,
				       PMA_RX_STS_RX_ACK_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	return 0;

timeout:

	return -ETIMEDOUT;
}

static int xpcs_phy_mplla_configuration_xaui_kx4(struct dw_xpcs *xpcs, bool has_pcs_pma)
{
	u8 devad = has_pcs_pma ? MDIO_MMD_PMAPMD : MDIO_MMD_VEND2;
	int ret;

	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_CLK_DIV2, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_RANGE_MASK, PMA_REF_CLK_CTRL_REF_RANGE(0x6));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_MPLLA_DIV2, PMA_REF_CLK_CTRL_REF_MPLLA_DIV2);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV_MULT_MASK, PMA_MPLLA_CTRL2_MPLLA_DIV_MULT(0xA));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV_CLK_EN, PMA_MPLLA_CTRL2_MPLLA_DIV_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV8_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN, PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV16P5_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_TX_CLK_DIV_MASK, PMA_MPLLA_CTRL2_MPLLA_TX_CLK_DIV(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_MPLLA_CTRL1, XPCS_G_ALL_BITS, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_MPLLA_CTRL5, XPCS_G_ALL_BITS, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_MPLLA_CTRL4, XPCS_G_ALL_BITS, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_MPLLA_CTRL5, XPCS_G_ALL_BITS, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_MPLLA_CTRL1, XPCS_G_ALL_BITS, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_MPLLA_CTRL0,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER_MASK,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER(0x28));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_LVL_MASK, PMA_TX_GENCTRL1_VBOOST_LVL(0x5));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_MPLLA_CTRL3,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH_MASK,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH(0xA017));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_MISC_CTRL0,
			PMA_MISC_CTRL0_RX_VREF_CTRL_MASK, PMA_MISC_CTRL0_RX_VREF_CTRL(0x11));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_MISC_CTRL2,
			PMA_MISC_CTRL2_SUP_MISC_MASK, PMA_MISC_CTRL2_SUP_MISC(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_VCO_CAL_REF0,
			PMA_VCO_CAL_REF0_VCO_REF_LD_0_MASK, PMA_VCO_CAL_REF0_VCO_REF_LD_0(0x22));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_VCO_CAL_LD0,
			PMA_VCO_CAL_LD0_VCO_LD_VAL_0_MASK, PMA_VCO_CAL_LD0_VCO_LD_VAL_0(0x550));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_PPM_CTRL0,
			PMA_RX_PPM_CTRL0_RX0_CDR_PPM_MAX_MASK,
			PMA_RX_PPM_CTRL0_RX0_CDR_PPM_MAX(0x12));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_TX_MISC_CTRL0,
			PMA_TX_MISC_CTRL0_TX0_MISC_MASK, PMA_TX_MISC_CTRL0_TX0_MISC(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_RATE_CTRL,
			PMA_TX_RATE_CTRL_TX0_RATE_MASK, PMA_TX_RATE_CTRL_TX0_RATE(0x2));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_MPLL_CMN_CTRL,
			PMA_MPLL_CMN_CTRL_MPLLB_SEL_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_TX_GENCTRL2,
			PMA_TX_GENCTRL2_TX0_WIDTH_MASK, PMA_TX_GENCTRL2_TX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_EN_0, PMA_TX_GENCTRL1_VBOOST_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_BOOST_CTRL,
			PMA_TX_BOOST_CTRL_TX0_IBOOST_MASK, PMA_TX_BOOST_CTRL_TX0_IBOOST(0xF));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_PRE_MASK, PMA_TX_EQ_CTRL0_TX_EQ_PRE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_EQ_CTRL1,
			PMA_TX_EQ_CTRL1_TX_EQ_POST_MASK, PMA_TX_EQ_CTRL1_TX_EQ_POST(0x20));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_MAIN_MASK, PMA_TX_EQ_CTRL0_TX_EQ_MAIN(0x20));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_RATE_CTRL,
			PMA_RX_RATE_CTRL_RX0_RATE_MASK, PMA_RX_RATE_CTRL_RX0_RATE(0x2));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_BOOST_0_MASK, PMA_RX_EQ_CTRL0_CTLE_BOOST_0(0x10));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_POLE_0_MASK, PMA_RX_EQ_CTRL0_CTLE_POLE_0(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL3,
			PMA_RX_GENCTRL3_LOS_TRSHLD_0_MASK, PMA_RX_GENCTRL3_LOS_TRSHLD_0(0x2));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0, PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_MISC_CTRL0,
			PMA_RX_MISC_CTRL0_RX0_MISC_MASK, PMA_RX_MISC_CTRL0_RX0_MISC(0x17));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX0_WIDTH_MASK, PMA_RX_GENCTRL2_RX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_DIV16P5_CLK_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_CDR_CTRL,
			PMA_RX_CDR_CTRL_CDR_SSC_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL3,
			PMA_RX_GENCTRL3_LOS_LFPS_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_GENCTRL4,
			PMA_RX_GENCTRL4_RX_DFE_BYP_0, PMA_RX_GENCTRL4_RX_DFE_BYP_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_ATTN_CTRL,
			PMA_RX_ATTN_CTRL_RX0_EQ_ATT_LVL_MASK, PMA_RX_ATTN_CTRL_RX0_EQ_ATT_LVL(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_VGA1_GAIN_0_MASK, PMA_RX_EQ_CTRL0_VGA1_GAIN_0(0x4));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_VGA2_GAIN_0_MASK, PMA_RX_EQ_CTRL0_VGA2_GAIN_0(0x4));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_DFE_TAP_CTRL0,
			PMA_DFE_TAP_CTRL0_DFE_TAP1_0_MASK, PMA_DFE_TAP_CTRL0_DFE_TAP1_0(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_FRQBAND_0_MASK, PMA_RX_CDR_CTRL1_VCO_FRQBAND_0(0x2));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_TERM_ACDC_0, PMA_RX_GENCTRL1_RX_TERM_ACDC_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_IQ_CTRL0,
			PMA_RX_IQ_CTRL0_RX0_DELTA_IQ_MASK, PMA_RX_IQ_CTRL0_RX0_DELTA_IQ(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX_ADPT_SEL_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX0_ADPT_MODE_MASK, PMA_RX_EQ_CTRL5_RX0_ADPT_MODE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, has_pcs_pma ? MDIO_MMD_PCS : MDIO_MMD_VEND2, PCS_CTRL2,
			PCS_CTRL2_PCS_TYPE_SEL_MASK, PCS_CTRL2_PCS_TYPE_SEL(0x1));
	if (has_pcs_pma)
		xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_XAUI_CTRL,
				PCS_XAUI_CTRL_XAUI_MODE, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, has_pcs_pma ? MDIO_MMD_PCS : MDIO_MMD_VEND2, MII_DIG_CTRL1,
			MII_DIG_CTRL1_EN_2_5G_MODE, MII_DIG_CTRL1_EN_2_5G_MODE);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_CTRL, MII_CTRL_SS13, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_CTRL, MII_CTRL_SS6, MII_CTRL_SS6);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_MPLLA_CTRL0,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER_MASK,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER(0x28));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_MPLLA_CTRL3,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH_MASK,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH(0xA017));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_VCO_CAL_LD0,
			PMA_VCO_CAL_LD0_VCO_LD_VAL_0_MASK, PMA_VCO_CAL_LD0_VCO_LD_VAL_0(0x550));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_VCO_CAL_REF0,
			PMA_VCO_CAL_REF0_VCO_REF_LD_0_MASK, PMA_VCO_CAL_REF0_VCO_REF_LD_0(0x22));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_EQ_CTRL4,
			PMA_RX_EQ_CTRL4_CONT_ADAPT_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_RATE_CTRL,
			PMA_TX_RATE_CTRL_TX0_RATE_MASK, PMA_TX_RATE_CTRL_TX0_RATE(0x2));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_RATE_CTRL,
			PMA_RX_RATE_CTRL_RX0_RATE_MASK, PMA_RX_RATE_CTRL_RX0_RATE(0x2));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_TX_GENCTRL2,
			PMA_TX_GENCTRL2_TX0_WIDTH_MASK, PMA_TX_GENCTRL2_TX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX0_WIDTH_MASK, PMA_RX_GENCTRL2_RX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV8_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN, PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV16P5_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_EN_0, PMA_TX_GENCTRL1_VBOOST_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_BOOST_0_MASK, PMA_RX_EQ_CTRL0_CTLE_BOOST_0(0x10));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0, PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_MISC_CTRL0,
			PMA_RX_MISC_CTRL0_RX0_MISC_MASK, PMA_RX_MISC_CTRL0_RX0_MISC(0x17));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_GENCTRL4,
			PMA_RX_GENCTRL4_RX_DFE_BYP_0, PMA_RX_GENCTRL4_RX_DFE_BYP_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_FRQBAND_0_MASK, PMA_RX_CDR_CTRL1_VCO_FRQBAND_0(0x2));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_IQ_CTRL0,
			PMA_RX_IQ_CTRL0_RX0_DELTA_IQ_MASK, PMA_RX_IQ_CTRL0_RX0_DELTA_IQ(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX0_ADPT_MODE_MASK, PMA_RX_EQ_CTRL5_RX0_ADPT_MODE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, has_pcs_pma ? MDIO_MMD_PCS : MDIO_MMD_VEND2, MII_DIG_CTRL1,
			MII_DIG_CTRL1_VR_RST, MII_DIG_CTRL1_VR_RST);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_SRAM,
				       PMA_SRAM_INIT_DN, 1);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	mdelay(1);

	xpcs_phy_modify(xpcs, XPCS_PHY_DEV, XPCS_PHY_GLOBAL, GLOBAL_CTRL_EX_0,
			GLOBAL_CTRL_EX_0_PHY_SRAM_BYPASS, GLOBAL_CTRL_EX_0_PHY_SRAM_BYPASS);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, has_pcs_pma ? MDIO_MMD_PCS : MDIO_MMD_VEND2,
				       MII_DIG_CTRL1, MII_DIG_CTRL1_VR_RST, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_TX_CLK_RDY_0, PMA_TX_GENCTRL1_TX_CLK_RDY_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_PRE_MASK, PMA_TX_EQ_CTRL0_TX_EQ_PRE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_MAIN_MASK, PMA_TX_EQ_CTRL0_TX_EQ_MAIN(0x20));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_EQ_CTRL1,
			PMA_TX_EQ_CTRL1_TX_EQ_POST_MASK, PMA_TX_EQ_CTRL1_TX_EQ_POST(0x20));
	xpcs_phy_modify(xpcs, XPCS_DEV, has_pcs_pma ? MDIO_MMD_PCS : MDIO_MMD_VEND2, PCS_DEBUG_CTRL,
			PCS_DEBUG_CTRL_SUPRESS_LOS_DET, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, has_pcs_pma ? MDIO_MMD_PCS : MDIO_MMD_VEND2, PCS_DEBUG_CTRL,
			PCS_DEBUG_CTRL_RX_DT_EN_CTL, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_LINK_TIMER_CTRL,
			MII_LINK_TIMER_CTRL_CL37_LINK_TIME_MASK,
			MII_LINK_TIMER_CTRL_CL37_LINK_TIME(0x07A1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_DIG_CTRL1,
			MII_DIG_CTRL1_CL37_TMR_OVR_RIDE, MII_DIG_CTRL1_CL37_TMR_OVR_RIDE);

	return 0;

timeout:

	return -ETIMEDOUT;
}

static int imx94_xpcs_phy_mpllb_configuration_sgmii(struct dw_xpcs *xpcs)
{
	int ret;

	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_CLK_DIV2, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_RANGE_MASK, PMA_REF_CLK_CTRL_REF_RANGE(6));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_MPLLA_DIV2, PMA_REF_CLK_CTRL_REF_MPLLA_DIV2);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_MPLLB_DIV2, PMA_REF_CLK_CTRL_REF_MPLLB_DIV2);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_MPLLB_CTRL0,
			PMA_MPLLB_CTRL0_MPLLB_CAL_DISABLE, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_MPLLB_CTRL2,
			PMA_MPLLB_CTRL2_MPLLB_DIV_MULT_MASK, PMA_MPLLB_CTRL2_MPLLB_DIV_MULT(0x1e));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_MPLLB_CTRL2,
			PMA_MPLLB_CTRL2_MPLLB_DIV_CLK_EN, PMA_MPLLB_CTRL2_MPLLB_DIV_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_MPLLB_CTRL2,
			PMA_MPLLB_CTRL2_MPLLB_DIV8_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_MPLLB_CTRL2,
			PMA_MPLLB_CTRL2_MPLLB_DIV10_CLK_EN, PMA_MPLLB_CTRL2_MPLLB_DIV10_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_MPLLB_CTRL2,
			PMA_MPLLB_CTRL2_MPLLB_TX_CLK_DIV_MASK, PMA_MPLLB_CTRL2_MPLLB_TX_CLK_DIV(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_MPLLB_CTRL1, XPCS_G_ALL_BITS,
			0x0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_MPLLB_CTRL5, XPCS_G_ALL_BITS,
			0x0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_MPLLB_CTRL4, XPCS_G_ALL_BITS,
			0x0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_MPLLB_CTRL0,
			PMA_MPLLB_CTRL0_MPLLB_MULTIPLIER_MASK,
			PMA_MPLLB_CTRL0_MPLLB_MULTIPLIER(0x30));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_LVL_MASK, PMA_TX_GENCTRL1_VBOOST_LVL(0x5));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_MPLLB_CTRL3,
			PMA_MPLL_CTRL3_MPLLB_BANDWIDTH_MASK,
			PMA_MPLL_CTRL3_MPLLB_BANDWIDTH(0xA017));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_MISC_CTRL0,
			PMA_MISC_CTRL0_RX_VREF_CTRL_MASK, PMA_MISC_CTRL0_RX_VREF_CTRL(0x11));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_MISC_CTRL2,
			PMA_MISC_CTRL2_SUP_MISC_MASK, PMA_MISC_CTRL2_SUP_MISC(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_VCO_CAL_REF0,
			PMA_VCO_CAL_REF0_VCO_REF_LD_0_MASK, PMA_VCO_CAL_REF0_VCO_REF_LD_0(0x2a));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_VCO_CAL_LD0,
			PMA_VCO_CAL_LD0_VCO_LD_VAL_0_MASK, PMA_VCO_CAL_LD0_VCO_LD_VAL_0(0x540));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_PPM_CTRL0,
			PMA_RX_PPM_CTRL0_RX0_CDR_PPM_MAX_MASK,
			PMA_RX_PPM_CTRL0_RX0_CDR_PPM_MAX(0x12));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_TX_MISC_CTRL0,
			PMA_TX_MISC_CTRL0_TX0_MISC_MASK, PMA_TX_MISC_CTRL0_TX0_MISC(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_RATE_CTRL,
			PMA_TX_RATE_CTRL_TX0_RATE_MASK, PMA_TX_RATE_CTRL_TX0_RATE(0x6));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_MPLL_CMN_CTRL,
			PMA_MPLL_CMN_CTRL_MPLLB_SEL_0, PMA_MPLL_CMN_CTRL_MPLLB_SEL_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_TX_GENCTRL2,
			PMA_TX_GENCTRL2_TX0_WIDTH_MASK, PMA_TX_GENCTRL2_TX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_EN_0, PMA_TX_GENCTRL1_VBOOST_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_BOOST_CTRL,
			PMA_TX_BOOST_CTRL_TX0_IBOOST_MASK, PMA_TX_BOOST_CTRL_TX0_IBOOST(0xF));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_PRE_MASK, PMA_TX_EQ_CTRL0_TX_EQ_PRE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_EQ_CTRL1,
			PMA_TX_EQ_CTRL1_TX_EQ_POST_MASK, PMA_TX_EQ_CTRL1_TX_EQ_POST(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_MAIN_MASK, PMA_TX_EQ_CTRL0_TX_EQ_MAIN(0x28));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_RX_RATE_CTRL,
			PMA_RX_RATE_CTRL_RX0_RATE_MASK, PMA_RX_RATE_CTRL_RX0_RATE(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_BOOST_0_MASK, PMA_RX_EQ_CTRL0_CTLE_BOOST_0(0x12));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_POLE_0_MASK, PMA_RX_EQ_CTRL0_CTLE_POLE_0(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_RX_GENCTRL3,
			PMA_RX_GENCTRL3_LOS_TRSHLD_0_MASK, PMA_RX_GENCTRL3_LOS_TRSHLD_0(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0, PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0, PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_MISC_CTRL0,
			PMA_RX_MISC_CTRL0_RX0_MISC_MASK, PMA_RX_MISC_CTRL0_RX0_MISC(0x16));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX0_WIDTH_MASK, PMA_RX_GENCTRL2_RX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_DIV16P5_CLK_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_RX_CDR_CTRL,
			PMA_RX_CDR_CTRL_CDR_SSC_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_RX_GENCTRL3,
			PMA_RX_GENCTRL3_LOS_LFPS_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_GENCTRL4,
			PMA_RX_GENCTRL4_RX_DFE_BYP_0, PMA_RX_GENCTRL4_RX_DFE_BYP_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_RX_ATTN_CTRL,
			PMA_RX_ATTN_CTRL_RX0_EQ_ATT_LVL_MASK, PMA_RX_ATTN_CTRL_RX0_EQ_ATT_LVL(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_VGA1_GAIN_0_MASK, PMA_RX_EQ_CTRL0_VGA1_GAIN_0(0x4));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_VGA2_GAIN_0_MASK, PMA_RX_EQ_CTRL0_VGA2_GAIN_0(0x4));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_DFE_TAP_CTRL0,
			PMA_DFE_TAP_CTRL0_DFE_TAP1_0_MASK, PMA_DFE_TAP_CTRL0_DFE_TAP1_0(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_FRQBAND_0_MASK, PMA_RX_CDR_CTRL1_VCO_FRQBAND_0(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_TERM_ACDC_0, PMA_RX_GENCTRL1_RX_TERM_ACDC_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_IQ_CTRL0,
			PMA_RX_IQ_CTRL0_RX0_DELTA_IQ_MASK, PMA_RX_IQ_CTRL0_RX0_DELTA_IQ(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX_ADPT_SEL_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX0_ADPT_MODE_MASK, PMA_RX_EQ_CTRL5_RX0_ADPT_MODE(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PCS_CTRL2, PCS_CTRL2_PCS_TYPE_SEL_MASK,
			PCS_CTRL2_PCS_TYPE_SEL(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_DIG_CTRL1, MII_DIG_CTRL1_EN_2_5G_MODE,
			0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_CTRL, MII_CTRL_SS13, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_CTRL, MII_CTRL_SS6, MII_CTRL_SS6);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_MPLLB_CTRL0,
			PMA_MPLLB_CTRL0_MPLLB_MULTIPLIER_MASK,
			PMA_MPLLB_CTRL0_MPLLB_MULTIPLIER(0x30));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_MPLLB_CTRL3,
			PMA_MPLL_CTRL3_MPLLB_BANDWIDTH_MASK,
			PMA_MPLL_CTRL3_MPLLB_BANDWIDTH(0xA017));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_VCO_CAL_LD0,
			PMA_VCO_CAL_LD0_VCO_LD_VAL_0_MASK, PMA_VCO_CAL_LD0_VCO_LD_VAL_0(0x540));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_VCO_CAL_REF0,
			PMA_VCO_CAL_REF0_VCO_REF_LD_0_MASK, PMA_VCO_CAL_REF0_VCO_REF_LD_0(0x2a));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_RX_EQ_CTRL4,
			PMA_RX_EQ_CTRL4_CONT_ADAPT_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_RATE_CTRL,
			PMA_TX_RATE_CTRL_TX0_RATE_MASK, PMA_TX_RATE_CTRL_TX0_RATE(0x6));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_RX_RATE_CTRL,
			PMA_RX_RATE_CTRL_RX0_RATE_MASK, PMA_RX_RATE_CTRL_RX0_RATE(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_TX_GENCTRL2,
			PMA_TX_GENCTRL2_TX0_WIDTH_MASK, PMA_TX_GENCTRL2_TX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX0_WIDTH_MASK, PMA_RX_GENCTRL2_RX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_MPLLB_CTRL2,
			PMA_MPLLB_CTRL2_MPLLB_DIV8_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_MPLLB_CTRL2,
			PMA_MPLLB_CTRL2_MPLLB_DIV10_CLK_EN, PMA_MPLLB_CTRL2_MPLLB_DIV10_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_BOOST_0_MASK, PMA_RX_EQ_CTRL0_CTLE_BOOST_0(0x6));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0, PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0, PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_MISC_CTRL0,
			PMA_RX_MISC_CTRL0_RX0_MISC_MASK, PMA_RX_MISC_CTRL0_RX0_MISC(0x16));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_GENCTRL4,
			PMA_RX_GENCTRL4_RX_DFE_BYP_0, PMA_RX_GENCTRL4_RX_DFE_BYP_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_FRQBAND_0_MASK, PMA_RX_CDR_CTRL1_VCO_FRQBAND_0(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_IQ_CTRL0,
			PMA_RX_IQ_CTRL0_RX0_DELTA_IQ_MASK, PMA_RX_IQ_CTRL0_RX0_DELTA_IQ(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX0_ADPT_MODE_MASK, PMA_RX_EQ_CTRL5_RX0_ADPT_MODE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_TX_CLK_RDY_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_DIG_CTRL1, MII_DIG_CTRL1_VR_RST,
			MII_DIG_CTRL1_VR_RST);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_SRAM,
				       PMA_SRAM_INIT_DN, 1);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	mdelay(1);

	xpcs_phy_modify(xpcs, XPCS_PHY_DEV, XPCS_PHY_GLOBAL, GLOBAL_CTRL_EX_0,
			GLOBAL_CTRL_EX_0_PHY_SRAM_BYPASS, GLOBAL_CTRL_EX_0_PHY_SRAM_BYPASS);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_DIG_CTRL1,
				       MII_DIG_CTRL1_VR_RST, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_TX_CLK_RDY_0, PMA_TX_GENCTRL1_TX_CLK_RDY_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_PRE_MASK, PMA_TX_EQ_CTRL0_TX_EQ_PRE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_MAIN_MASK, PMA_TX_EQ_CTRL0_TX_EQ_MAIN(0x14));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PMA_MP_12G_16G_25G_TX_EQ_CTRL1,
			PMA_TX_EQ_CTRL1_TX_EQ_POST_MASK, PMA_TX_EQ_CTRL1_TX_EQ_POST(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PCS_DEBUG_CTRL,
			PCS_DEBUG_CTRL_SUPRESS_LOS_DET, 0x0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, PCS_DEBUG_CTRL, PCS_DEBUG_CTRL_RX_DT_EN_CTL,
			0x0);

	return 0;

timeout:

	return -ETIMEDOUT;
}

static int xpcs_phy_common_init_seq_2(struct dw_xpcs *xpcs, bool has_pcs_pma)
{
	u8 devad = has_pcs_pma ? MDIO_MMD_PMAPMD : MDIO_MMD_VEND2;
	int ret;

	xpcs_phy_modify(xpcs, XPCS_DEV, has_pcs_pma ? MDIO_MMD_PCS : MDIO_MMD_VEND2, PCS_DEBUG_CTRL,
			PCS_DEBUG_CTRL_TX_PMBL_CTL, PCS_DEBUG_CTRL_TX_PMBL_CTL);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_POWER_STATE_CTRL,
			PMA_POWER_STATE_CTRL_TX0_PSTATE_MASK, PMA_POWER_STATE_CTRL_TX0_PSTATE(0x2));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_MPLL_CMN_CTRL,
			PMA_MPLL_CMN_CTRL_MPLL_EN_0, PMA_MPLL_CMN_CTRL_MPLL_EN_0);

	mdelay(1);

	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_TX_GENCTRL2,
			PMA_TX_GENCTRL2_TX_REQ_0, PMA_TX_GENCTRL2_TX_REQ_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX_REQ_0, PMA_RX_GENCTRL2_RX_REQ_0);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL2,
				       PMA_RX_GENCTRL2_RX_REQ_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_TX_GENCTRL2,
				       PMA_TX_GENCTRL2_TX_REQ_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	mdelay(1);

	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_TX_GENCTRL2, PMA_TX_GENCTRL2_TX_REQ_0,
			0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL2, PMA_RX_GENCTRL2_RX_REQ_0,
			0);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_STS,
				       PMA_TX_STS_TX_ACK_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_STS,
				       PMA_RX_STS_RX_ACK_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_POWER_STATE_CTRL,
			PMA_POWER_STATE_CTRL_TX0_PSTATE_MASK, PMA_POWER_STATE_CTRL_TX0_PSTATE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_GENCTRL0,
			PMA_TX_GENCTRL0_TX_RST_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_POWER_STATE_CTRL,
			PMA_POWER_STATE_CTRL_TX_DISABLE_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_MPLL_CMN_CTRL,
			PMA_MPLL_CMN_CTRL_MPLL_EN_0, PMA_MPLL_CMN_CTRL_MPLL_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_RST_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_POWER_STATE_CTRL,
			PMA_RX_POWER_STATE_CTRL_RX_DISABLE_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_POWER_STATE_CTRL,
			PMA_RX_POWER_STATE_CTRL_RX0_PSTATE_MASK,
			PMA_RX_POWER_STATE_CTRL_RX0_PSTATE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_GENCTRL0,
			PMA_TX_GENCTRL0_TX_DT_EN_0, PMA_TX_GENCTRL0_TX_DT_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_GENCTRL0,
			PMA_RX_GENCTRL0_RX_DT_EN_0, PMA_RX_GENCTRL0_RX_DT_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_TX_GENCTRL2,
			PMA_TX_GENCTRL2_TX_REQ_0, PMA_TX_GENCTRL2_TX_REQ_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX_REQ_0, PMA_RX_GENCTRL2_RX_REQ_0);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL2,
				       PMA_RX_GENCTRL2_RX_REQ_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_TX_GENCTRL2,
				       PMA_TX_GENCTRL2_TX_REQ_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	mdelay(1);

	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_TX_GENCTRL2, PMA_TX_GENCTRL2_TX_REQ_0,
			0);
	xpcs_phy_modify(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_RX_GENCTRL2, PMA_RX_GENCTRL2_RX_REQ_0,
			0);

	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_TX_STS,
				       PMA_TX_STS_TX_ACK_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, devad, PMA_MP_12G_16G_25G_RX_STS,
				       PMA_RX_STS_RX_ACK_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	return 0;

timeout:

	return -ETIMEDOUT;
}

static int imx94_xpcs_phy_sgmii_config(struct dw_xpcs *xpcs, bool is_2p5g)
{
	int ret;

	ret = xpcs_phy_reg_lock(xpcs);
	if (ret)
		return ret;

	ret = imx94_xpcs_phy_port_init(xpcs, is_2p5g);
	if (ret)
		return ret;
	ret = xpcs_phy_common_init_seq_1(xpcs, false, is_2p5g ? false : true);
	if (ret)
		return ret;

	if (is_2p5g)
		ret = xpcs_phy_mplla_configuration_xaui_kx4(xpcs, false);
	else
		ret = imx94_xpcs_phy_mpllb_configuration_sgmii(xpcs);
	if (ret)
		return ret;

	ret = xpcs_phy_common_init_seq_2(xpcs, false);
	if (ret)
		return ret;

	ret = xpcs_phy_reg_unlock(xpcs);
	if (ret)
		return ret;

	return 0;
}

int imx94_xpcs_phy_sgmii_2p5g_config(struct dw_xpcs *xpcs)
{
	return imx94_xpcs_phy_sgmii_config(xpcs, true);
}

int imx94_xpcs_phy_sgmii_1g_config(struct dw_xpcs *xpcs)
{
	return imx94_xpcs_phy_sgmii_config(xpcs, false);
}

static int imx95_xpcs_phy_mplla_configuration_sgmii(struct dw_xpcs *xpcs)
{
	int ret;

	/* 2 Config MPLL for SGMII */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_RANGE_MASK, PMA_REF_CLK_CTRL_REF_RANGE(0x6));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_CLK_DIV2, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_MPLLA_DIV2, PMA_REF_CLK_CTRL_REF_MPLLA_DIV2);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV8_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN, PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV16P5_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_TX_CLK_DIV_MASK, PMA_MPLLA_CTRL2_MPLLA_TX_CLK_DIV(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV_CLK_EN, PMA_MPLLA_CTRL2_MPLLA_DIV_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV_MULT_MASK, PMA_MPLLA_CTRL2_MPLLA_DIV_MULT(0x14));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL1,
			PMA_MPLLA_CTRL1_MPLLA_SSC_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL1,
			PMA_MPLLA_CTRL1_MPLLA_SSC_CLK_SEL, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL5,
			PMA_MPLLA_CTRL5_MPLLA_SSC_FRQ_CNT_PK_MASK,
			PMA_MPLLA_CTRL5_MPLLA_SSC_FRQ_CNT_PK(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL4,
			PMA_MPLLA_CTRL4_MPLLA_SSC_FRQ_CNT_INT_MASK,
			PMA_MPLLA_CTRL4_MPLLA_SSC_FRQ_CNT_INT(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL5,
			PMA_MPLLA_CTRL5_MPLLA_SSC_SPD_EN, 0);

	/* TODO: check if is needed */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL1,
			PMA_MPLLA_CTRL1_MPLLA_FRACN_CTRL_MASK, PMA_MPLLA_CTRL1_MPLLA_FRACN_CTRL(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL0,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER_MASK,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER(0x20));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_LVL_MASK, PMA_TX_GENCTRL1_VBOOST_LVL(0x5));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL3,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH_MASK,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH(0xA035));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_MISC_CTRL0,
			PMA_MISC_CTRL0_RX_VREF_CTRL_MASK, PMA_MISC_CTRL0_RX_VREF_CTRL(0x11));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_MISC_CTRL2,
			PMA_MISC_CTRL2_SUP_MISC_MASK, PMA_MISC_CTRL2_SUP_MISC(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_VCO_CAL_REF0,
			PMA_VCO_CAL_REF0_VCO_REF_LD_0_MASK, PMA_VCO_CAL_REF0_VCO_REF_LD_0(0x2a));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_VCO_CAL_LD0,
			PMA_VCO_CAL_LD0_VCO_LD_VAL_0_MASK, PMA_VCO_CAL_LD0_VCO_LD_VAL_0(0x540));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_PPM_CTRL0,
			PMA_RX_PPM_CTRL0_RX0_CDR_PPM_MAX_MASK,
			PMA_RX_PPM_CTRL0_RX0_CDR_PPM_MAX(0x12));

	/* 3 Configure LANE0 for 1G SGMII */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_TX_MISC_CTRL0,
			PMA_TX_MISC_CTRL0_TX0_MISC_MASK, PMA_TX_MISC_CTRL0_TX0_MISC(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_RATE_CTRL,
			PMA_TX_RATE_CTRL_TX0_RATE_MASK, PMA_TX_RATE_CTRL_TX0_RATE(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_MPLL_CMN_CTRL,
			PMA_MPLL_CMN_CTRL_MPLLB_SEL_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_TX_GENCTRL2,
			PMA_TX_GENCTRL2_TX0_WIDTH_MASK, PMA_TX_GENCTRL2_TX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_EN_0, PMA_TX_GENCTRL1_VBOOST_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_BOOST_CTRL,
			PMA_TX_BOOST_CTRL_TX0_IBOOST_MASK, PMA_TX_BOOST_CTRL_TX0_IBOOST(0xf));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_PRE_MASK, PMA_TX_EQ_CTRL0_TX_EQ_PRE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_EQ_CTRL1,
			PMA_TX_EQ_CTRL1_TX_EQ_POST_MASK, PMA_TX_EQ_CTRL1_TX_EQ_POST(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_MAIN_MASK, PMA_TX_EQ_CTRL0_TX_EQ_MAIN(0x28));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_RATE_CTRL,
			PMA_RX_RATE_CTRL_RX0_RATE_MASK, PMA_RX_RATE_CTRL_RX0_RATE(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_POLE_0_MASK, PMA_RX_EQ_CTRL0_CTLE_POLE_0(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_BOOST_0_MASK, PMA_RX_EQ_CTRL0_CTLE_BOOST_0(0x12));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_RX_GENCTRL3,
			PMA_RX_GENCTRL3_LOS_TRSHLD_0_MASK, PMA_RX_GENCTRL3_LOS_TRSHLD_0(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0, PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0, PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_MISC_CTRL0,
			PMA_RX_MISC_CTRL0_RX0_MISC_MASK, PMA_RX_MISC_CTRL0_RX0_MISC(0x16));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX0_WIDTH_MASK, PMA_RX_GENCTRL2_RX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_DIV16P5_CLK_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_CDR_CTRL,
			PMA_RX_CDR_CTRL_CDR_SSC_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_RX_GENCTRL3,
			PMA_RX_GENCTRL3_LOS_LFPS_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_GENCTRL4,
			PMA_RX_GENCTRL4_RX_DFE_BYP_0, PMA_RX_GENCTRL4_RX_DFE_BYP_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_ATTN_CTRL,
			PMA_RX_ATTN_CTRL_RX0_EQ_ATT_LVL_MASK, PMA_RX_ATTN_CTRL_RX0_EQ_ATT_LVL(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_VGA1_GAIN_0_MASK, PMA_RX_EQ_CTRL0_VGA1_GAIN_0(0x4));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_VGA2_GAIN_0_MASK, PMA_RX_EQ_CTRL0_VGA2_GAIN_0(0x4));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_DFE_TAP_CTRL0,
			PMA_DFE_TAP_CTRL0_DFE_TAP1_0_MASK, PMA_DFE_TAP_CTRL0_DFE_TAP1_0(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_FRQBAND_0_MASK, PMA_RX_CDR_CTRL1_VCO_FRQBAND_0(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_TERM_ACDC_0, PMA_RX_GENCTRL1_RX_TERM_ACDC_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_IQ_CTRL0,
			PMA_RX_IQ_CTRL0_RX0_DELTA_IQ_MASK, PMA_RX_IQ_CTRL0_RX0_DELTA_IQ(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX_ADPT_SEL_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX0_ADPT_MODE_MASK, PMA_RX_EQ_CTRL5_RX0_ADPT_MODE(0x3));

	/* 4 Configure XPCS for 1G SGMII */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_CTRL2, PCS_CTRL2_PCS_TYPE_SEL_MASK,
			PCS_CTRL2_PCS_TYPE_SEL(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_CTRL, MII_CTRL_SS13, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_CTRL, MII_CTRL_SS6, MII_CTRL_SS6);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL0,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER_MASK,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER(0x20));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL3,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH_MASK,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH(0xA035));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_VCO_CAL_LD0,
			PMA_VCO_CAL_LD0_VCO_LD_VAL_0_MASK, PMA_VCO_CAL_LD0_VCO_LD_VAL_0(0x540));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_VCO_CAL_REF0,
			PMA_VCO_CAL_REF0_VCO_REF_LD_0_MASK, PMA_VCO_CAL_REF0_VCO_REF_LD_0(0x2a));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_EQ_CTRL4,
			PMA_RX_EQ_CTRL4_CONT_ADAPT_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_RATE_CTRL,
			PMA_TX_RATE_CTRL_TX0_RATE_MASK, PMA_TX_RATE_CTRL_TX0_RATE(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_RATE_CTRL,
			PMA_RX_RATE_CTRL_RX0_RATE_MASK, PMA_RX_RATE_CTRL_RX0_RATE(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_TX_GENCTRL2,
			PMA_TX_GENCTRL2_TX0_WIDTH_MASK, PMA_TX_GENCTRL2_TX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX0_WIDTH_MASK, PMA_RX_GENCTRL2_RX0_WIDTH(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV16P5_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN, PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV8_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_EN_0, PMA_TX_GENCTRL1_VBOOST_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_BOOST_0_MASK, PMA_RX_EQ_CTRL0_CTLE_BOOST_0(0x6));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0, PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0, PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_MISC_CTRL0,
			PMA_RX_MISC_CTRL0_RX0_MISC_MASK, PMA_RX_MISC_CTRL0_RX0_MISC(0x6));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_GENCTRL4,
			PMA_RX_GENCTRL4_RX_DFE_BYP_0, PMA_RX_GENCTRL4_RX_DFE_BYP_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_FRQBAND_0_MASK, PMA_RX_CDR_CTRL1_VCO_FRQBAND_0(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_IQ_CTRL0,
			PMA_RX_IQ_CTRL0_RX0_DELTA_IQ_MASK, PMA_RX_IQ_CTRL0_RX0_DELTA_IQ(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX0_ADPT_MODE_MASK, PMA_RX_EQ_CTRL5_RX0_ADPT_MODE(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_TX_CLK_RDY_0, 0);

	/* 4.1 Assert soft reset */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_DIG_CTRL1, PCS_DIG_CTRL1_VR_RST,
			PCS_DIG_CTRL1_VR_RST);

	/* 4.2 Poll for SRAM initialization done */
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_SRAM,
				       PMA_SRAM_INIT_DN, 1);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	/* 4.3 Assert SRAM external loading done */
	xpcs_phy_write(xpcs, XPCS_PHY_GLOBAL, XPCS_PHY_REG(GLOBAL_CTRL_EX_0),
		       GLOBAL_CTRL_EX_0_PHY_SRAM_BYPASS);

	/* 4.4 Poll for vendor-specific soft reset */
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_DIG_CTRL1,
				       PCS_DIG_CTRL1_VR_RST, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	/* 4.5 Assert TX0 clock is active and stable */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_TX_CLK_RDY_0, PMA_TX_GENCTRL1_TX_CLK_RDY_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_PRE_MASK, PMA_TX_EQ_CTRL0_TX_EQ_PRE(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_MAIN_MASK, PMA_TX_EQ_CTRL0_TX_EQ_MAIN(0x28));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_EQ_CTRL1,
			PMA_TX_EQ_CTRL1_TX_EQ_POST_MASK, PMA_TX_EQ_CTRL1_TX_EQ_POST(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_DEBUG_CTRL,
			PCS_DEBUG_CTRL_SUPRESS_LOS_DET, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_DEBUG_CTRL, PCS_DEBUG_CTRL_RX_DT_EN_CTL,
			0);

	return 0;

timeout:

	return -ETIMEDOUT;
}

static int imx95_xpcs_phy_xfi_10g_config(struct dw_xpcs *xpcs)
{
	int ret;

	/* 2 Config MPLL for 10G XGMII */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_RANGE_MASK, PMA_REF_CLK_CTRL_REF_RANGE(6));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_CLK_DIV2, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_REF_CLK_CTRL,
			PMA_REF_CLK_CTRL_REF_MPLLA_DIV2, PMA_REF_CLK_CTRL_REF_MPLLA_DIV2);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV8_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN, PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV16P5_CLK_EN, PMA_MPLLA_CTRL2_MPLLA_DIV16P5_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_TX_CLK_DIV_MASK, PMA_MPLLA_CTRL2_MPLLA_TX_CLK_DIV(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV_CLK_EN, PMA_MPLLA_CTRL2_MPLLA_DIV_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV_MULT_MASK, PMA_MPLLA_CTRL2_MPLLA_DIV_MULT(5));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL1,
			PMA_MPLLA_CTRL1_MPLLA_SSC_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL1,
			PMA_MPLLA_CTRL1_MPLLA_SSC_CLK_SEL, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL5,
			PMA_MPLLA_CTRL5_MPLLA_SSC_FRQ_CNT_PK_MASK,
			PMA_MPLLA_CTRL5_MPLLA_SSC_FRQ_CNT_PK(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL4,
			PMA_MPLLA_CTRL4_MPLLA_SSC_FRQ_CNT_INT_MASK,
			PMA_MPLLA_CTRL4_MPLLA_SSC_FRQ_CNT_INT(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL5,
			PMA_MPLLA_CTRL5_MPLLA_SSC_SPD_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL1,
			PMA_MPLLA_CTRL1_MPLLA_FRACN_CTRL_MASK, PMA_MPLLA_CTRL1_MPLLA_FRACN_CTRL(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL0,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER_MASK,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER(33));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_LVL_MASK, PMA_TX_GENCTRL1_VBOOST_LVL(5));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL3,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH_MASK,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH(0xA016));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_MISC_CTRL0,
			PMA_MISC_CTRL0_RX_VREF_CTRL_MASK, PMA_MISC_CTRL0_RX_VREF_CTRL(0x11));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_MISC_CTRL2,
			PMA_MISC_CTRL2_SUP_MISC_MASK, PMA_MISC_CTRL2_SUP_MISC(1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_VCO_CAL_REF0,
			PMA_VCO_CAL_REF0_VCO_REF_LD_0_MASK, PMA_VCO_CAL_REF0_VCO_REF_LD_0(0x29));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_VCO_CAL_LD0,
			PMA_VCO_CAL_LD0_VCO_LD_VAL_0_MASK, PMA_VCO_CAL_LD0_VCO_LD_VAL_0(0x549));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_PPM_CTRL0,
			PMA_RX_PPM_CTRL0_RX0_CDR_PPM_MAX_MASK,
			PMA_RX_PPM_CTRL0_RX0_CDR_PPM_MAX(0x12));

	/* 3 Configure LANE0 for 10G XGMII */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_TX_MISC_CTRL0,
			PMA_TX_MISC_CTRL0_TX0_MISC_MASK, PMA_TX_MISC_CTRL0_TX0_MISC(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_RATE_CTRL,
			PMA_TX_RATE_CTRL_TX0_RATE_MASK, PMA_TX_RATE_CTRL_TX0_RATE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_MPLL_CMN_CTRL,
			PMA_MPLL_CMN_CTRL_MPLLB_SEL_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_TX_GENCTRL2,
			PMA_TX_GENCTRL2_TX0_WIDTH_MASK, PMA_TX_GENCTRL2_TX0_WIDTH(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_EN_0, PMA_TX_GENCTRL1_VBOOST_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_BOOST_CTRL,
			PMA_TX_BOOST_CTRL_TX0_IBOOST_MASK, PMA_TX_BOOST_CTRL_TX0_IBOOST(0xf));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_PRE_MASK, PMA_TX_EQ_CTRL0_TX_EQ_PRE(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_EQ_CTRL1,
			PMA_TX_EQ_CTRL1_TX_EQ_POST_MASK, PMA_TX_EQ_CTRL1_TX_EQ_POST(0x20));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_EQ_CTRL0,
			PMA_TX_EQ_CTRL0_TX_EQ_MAIN_MASK, PMA_TX_EQ_CTRL0_TX_EQ_MAIN(0x20));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_RATE_CTRL,
			PMA_RX_RATE_CTRL_RX0_RATE_MASK, PMA_RX_RATE_CTRL_RX0_RATE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_POLE_0_MASK, PMA_RX_EQ_CTRL0_CTLE_POLE_0(0x2));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_BOOST_0_MASK, PMA_RX_EQ_CTRL0_CTLE_BOOST_0(0x10));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_RX_GENCTRL3,
			PMA_RX_GENCTRL3_LOS_TRSHLD_0_MASK, PMA_RX_GENCTRL3_LOS_TRSHLD_0(0x7));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0, PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0, PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_MISC_CTRL0,
			PMA_RX_MISC_CTRL0_RX0_MISC_MASK, PMA_RX_MISC_CTRL0_RX0_MISC(0x12));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX0_WIDTH_MASK, PMA_RX_GENCTRL2_RX0_WIDTH(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_DIV16P5_CLK_EN_0, PMA_RX_GENCTRL1_RX_DIV16P5_CLK_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_CDR_CTRL,
			PMA_RX_CDR_CTRL_CDR_SSC_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_RX_GENCTRL3,
			PMA_RX_GENCTRL3_LOS_LFPS_EN_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_GENCTRL4,
			PMA_RX_GENCTRL4_RX_DFE_BYP_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_ATTN_CTRL,
			PMA_RX_ATTN_CTRL_RX0_EQ_ATT_LVL_MASK, PMA_RX_ATTN_CTRL_RX0_EQ_ATT_LVL(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_VGA1_GAIN_0_MASK, PMA_RX_EQ_CTRL0_VGA1_GAIN_0(0x5));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_VGA2_GAIN_0_MASK, PMA_RX_EQ_CTRL0_VGA2_GAIN_0(0x5));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_DFE_TAP_CTRL0,
			PMA_DFE_TAP_CTRL0_DFE_TAP1_0_MASK, PMA_DFE_TAP_CTRL0_DFE_TAP1_0(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_FRQBAND_0_MASK, PMA_RX_CDR_CTRL1_VCO_FRQBAND_0(0x1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_TERM_ACDC_0, PMA_RX_GENCTRL1_RX_TERM_ACDC_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_IQ_CTRL0,
			PMA_RX_IQ_CTRL0_RX0_DELTA_IQ_MASK, PMA_RX_IQ_CTRL0_RX0_DELTA_IQ(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX_ADPT_SEL_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX0_ADPT_MODE_MASK, PMA_RX_EQ_CTRL5_RX0_ADPT_MODE(0x3));

	/* 4 Configure XPCS for 10G XGMII */
	xpcs_write(xpcs, MDIO_MMD_PCS, XPCS_PHY_REG(PCS_CTRL2), 0x0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_DIG_CTRL1, PCS_DIG_CTRL1_USXG_EN,
			PCS_DIG_CTRL1_USXG_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_KR_CTRL1, PCS_KR_CTRL1_USXG_MODE_MASK,
			PCS_KR_CTRL1_USXG_MODE(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL0,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER_MASK,
			PMA_MPLLA_CTRL0_MPLLA_MULTIPLIER(33));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_MPLLA_CTRL3,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH_MASK,
			PMA_MPLLA_CTRL3_MPLLA_BANDWIDTH(0xA016));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_VCO_CAL_LD0,
			PMA_VCO_CAL_LD0_VCO_LD_VAL_0_MASK, PMA_VCO_CAL_LD0_VCO_LD_VAL_0(0x549));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_VCO_CAL_REF0,
			PMA_VCO_CAL_REF0_VCO_REF_LD_0_MASK, PMA_VCO_CAL_REF0_VCO_REF_LD_0(0x29));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_EQ_CTRL4,
			PMA_RX_EQ_CTRL4_CONT_ADAPT_0, PMA_RX_EQ_CTRL4_CONT_ADAPT_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_RATE_CTRL,
			PMA_TX_RATE_CTRL_TX0_RATE_MASK, PMA_TX_RATE_CTRL_TX0_RATE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_RATE_CTRL,
			PMA_RX_RATE_CTRL_RX0_RATE_MASK, PMA_RX_RATE_CTRL_RX0_RATE(0x0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_TX_GENCTRL2,
			PMA_TX_GENCTRL2_TX0_WIDTH_MASK, PMA_TX_GENCTRL2_TX0_WIDTH(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX0_WIDTH_MASK, PMA_RX_GENCTRL2_RX0_WIDTH(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV16P5_CLK_EN, PMA_MPLLA_CTRL2_MPLLA_DIV16P5_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN, PMA_MPLLA_CTRL2_MPLLA_DIV10_CLK_EN);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_MPLLA_CTRL2,
			PMA_MPLLA_CTRL2_MPLLA_DIV8_CLK_EN, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_VBOOST_EN_0, PMA_TX_GENCTRL1_VBOOST_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL0,
			PMA_RX_EQ_CTRL0_CTLE_BOOST_0_MASK, PMA_RX_EQ_CTRL0_CTLE_BOOST_0(0x10));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0, PMA_RX_CDR_CTRL1_VCO_STEP_CTRL_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0, PMA_RX_CDR_CTRL1_VCO_TEMP_COMP_EN_0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_MISC_CTRL0,
			PMA_RX_MISC_CTRL0_RX0_MISC_MASK, PMA_RX_MISC_CTRL0_RX0_MISC(0x12));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_GENCTRL4,
			PMA_RX_GENCTRL4_RX_DFE_BYP_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_RX_CDR_CTRL1,
			PMA_RX_CDR_CTRL1_VCO_FRQBAND_0_MASK, PMA_RX_CDR_CTRL1_VCO_FRQBAND_0(1));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_IQ_CTRL0,
			PMA_RX_IQ_CTRL0_RX0_DELTA_IQ_MASK, PMA_RX_IQ_CTRL0_RX0_DELTA_IQ(0));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_16G_25G_RX_EQ_CTRL5,
			PMA_RX_EQ_CTRL5_RX0_ADPT_MODE_MASK, PMA_RX_EQ_CTRL5_RX0_ADPT_MODE(0x3));
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_TX_CLK_RDY_0, 0);

	/* 5 Assert soft reset */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_DIG_CTRL1, PCS_DIG_CTRL1_VR_RST,
			PCS_DIG_CTRL1_VR_RST);

	/* 6 Poll for SRAM initialization done */
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_SRAM,
				       PMA_SRAM_INIT_DN, 1);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	/* 7 Assert SRAM external loading done */
	/* Workaround */
	xpcs_phy_write(xpcs, XPCS_PHY_GLOBAL, XPCS_PHY_REG(GLOBAL_CTRL_EX_0),
		       GLOBAL_CTRL_EX_0_PHY_SRAM_BYPASS);

	/* 8 Poll for vendor-specific soft reset */
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_DIG_CTRL1,
				       PCS_DIG_CTRL1_VR_RST, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	/* 9 Turn receive to P0 state */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_GENCTRL1,
			PMA_RX_GENCTRL1_RX_RST_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_POWER_STATE_CTRL,
			PMA_RX_POWER_STATE_CTRL_RX_DISABLE_0, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_POWER_STATE_CTRL,
			PMA_RX_POWER_STATE_CTRL_RX0_PSTATE_MASK,
			PMA_RX_POWER_STATE_CTRL_RX0_PSTATE(0));

	/* 10 Enable receiver data output from PHY */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_GENCTRL0,
			PMA_RX_GENCTRL0_RX_DT_EN_0, PMA_RX_GENCTRL0_RX_DT_EN_0);

	/* 11 Assert request of receive */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_RX_GENCTRL2,
			PMA_RX_GENCTRL2_RX_REQ_0, PMA_RX_GENCTRL2_RX_REQ_0);

	/* 11.1 Poll for acknowledge */
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_RX_GENCTRL2,
				       PMA_RX_GENCTRL2_RX_REQ_0, 0);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	/* 12 Assert TX0 clock is active and stable */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_TX_GENCTRL1,
			PMA_TX_GENCTRL1_TX_CLK_RDY_0, PMA_TX_GENCTRL1_TX_CLK_RDY_0);

	/* 13.1 Configure XPCS to consider Loss-of-Signal indicated by the
	 * PHY while evaluating the receive link status */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_DEBUG_CTRL,
			PCS_DEBUG_CTRL_SUPRESS_LOS_DET, PCS_DEBUG_CTRL_SUPRESS_LOS_DET);
	/* 13.2 Configure XPCS to deassert "receiver data enable" on
	 * detecting of Loss-of-Signal */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PCS, PCS_DEBUG_CTRL, PCS_DEBUG_CTRL_RX_DT_EN_CTL,
			PCS_DEBUG_CTRL_RX_DT_EN_CTL);

	/* 14 Poll for DPLL lock status for Lane 0 */
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_RX_LSTS,
				       PMA_RX_LSTS_RX_VALID_0, 1);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	/* 15 Assert request of receive adaptation */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_EQ_CTRL4,
			PMA_RX_EQ_CTRL4_RX_AD_REQ, PMA_RX_EQ_CTRL4_RX_AD_REQ);

	/* 16 Poll for acknowledge */
	ret = xpcs_phy_polling_timeout(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_MISC_STS,
				       PMA_MISC_STS_RX_ADPT_ACK, 1);
	if (ret) {
		dev_err(&xpcs->phydev->dev, "Polling timeout, line: %d\n", __LINE__);
		goto timeout;
	}

	/* 17 Deassert request of receive adaptation */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_PMAPMD, PMA_MP_12G_16G_25G_RX_EQ_CTRL4,
			PMA_RX_EQ_CTRL4_RX_AD_REQ, 0);

	/* 18 Set the value of Config_Reg to 0 for Clause 37 autonegotiation. */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_AN_CTRL, MII_AN_CTRL_TX_CONFIG, 0);

	/* 19 Select XGMII speed */
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_CTRL, MII_CTRL_SS5, 0);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_CTRL, MII_CTRL_SS6, MII_CTRL_SS6);
	xpcs_phy_modify(xpcs, XPCS_DEV, MDIO_MMD_VEND2, MII_CTRL, MII_CTRL_SS13, MII_CTRL_SS13);

	return 0;

timeout:
	return -ETIMEDOUT;
}

static int imx95_xpcs_phy_sgmii_config(struct dw_xpcs *xpcs, bool is_2p5g)
{
	int ret;

	ret = xpcs_phy_reg_lock(xpcs);
	if (ret)
		return ret;

	ret = xpcs_phy_common_init_seq_1(xpcs, true, is_2p5g ? false : true);
	if (ret)
		return ret;

	if (is_2p5g)
		ret = xpcs_phy_mplla_configuration_xaui_kx4(xpcs, true);
	else
		ret = imx95_xpcs_phy_mplla_configuration_sgmii(xpcs);
	if (ret)
		return ret;

	ret = xpcs_phy_common_init_seq_2(xpcs, true);
	if (ret)
		return ret;

	ret = xpcs_phy_reg_unlock(xpcs);
	if (ret)
		return ret;

	return 0;
}

int imx95_xpcs_phy_sgmii_2p5g_config(struct dw_xpcs *xpcs)
{
	return imx95_xpcs_phy_sgmii_config(xpcs, true);
}

int imx95_xpcs_phy_sgmii_1g_config(struct dw_xpcs *xpcs)
{
	return imx95_xpcs_phy_sgmii_config(xpcs, false);
}

int imx95_xpcs_phy_xfi_config(struct dw_xpcs *xpcs)
{
	int ret;

	ret = xpcs_phy_reg_lock(xpcs);
	if (ret)
		return ret;

	ret = xpcs_phy_common_init_seq_1(xpcs, true, false);
	if (ret)
		return ret;

	ret = imx95_xpcs_phy_xfi_10g_config(xpcs);
	if (ret)
		return ret;

	ret = xpcs_phy_common_init_seq_2(xpcs, true);
	if (ret)
		return ret;

	ret = xpcs_phy_reg_unlock(xpcs);
	if (ret)
		return ret;

	return 0;
}

u32 xpcs_phy_get_id(struct dw_xpcs *xpcs)
{
	int ret;
	u32 id;

	/* First, search C73 PCS using PCS MMD */
	ret = xpcs_phy_read(xpcs, XPCS_PHY_GLOBAL, XPCS_PHY_REG(IDCODE_HI));
	if (ret < 0)
		return 0xffffffff;

	id = ret << 16;

	ret = xpcs_phy_read(xpcs, XPCS_PHY_GLOBAL, XPCS_PHY_REG(IDCODE_LO));
	if (ret < 0)
		return 0xffffffff;

	/* If Device IDs are not all zeros or all ones,
	 * we found C73 AN-type device
	 */
	if ((id | ret) && (id | ret) != 0xffffffff)
		return id | ret;

	return 0xffffffff;
}

int xpcs_phy_check_id(u32 id)
{
	return id == NXP_MX94_XPCS_ID || id == NXP_MX95_XPCS_ID;
}
