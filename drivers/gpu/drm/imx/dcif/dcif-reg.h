/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright 2025 NXP
 */
#ifndef __DCIF_REG_H__
#define __DCIF_REG_H__

#include <linux/bits.h>

/* Version ID Register */
#define DCIF_VER				0x0

/* Parameter Registers */
#define DCIF_PAR_0				0x4
#define DCIF_PAR_1				0x8

/* Display Control and Parameter Registers */
#define DCIF_DISP_CTRL				0x10
#define DCIF_DISP_PAR				0x14
#define DCIF_DISP_SIZE				0x18

/* Display Status Registers */
#define DCIF_DISP_SR0				0x1C
#define DCIF_DISP_SR1				0x20

/* Interrupt Enable and Status Registers, n=0-2*/
#define DCIF_IE0(n)				(0x24 + (n) * 0x10000)
#define DCIF_IS0(n)				(0x28 + (n) * 0x10000)
#define DCIF_IE1(n)				(0x2C + (n) * 0x10000)
#define DCIF_IS1(n)				(0x30 + (n) * 0x10000)

/* DPI Control and Sync Parameter Registers */
#define DCIF_DPI_CTRL				0x40
#define DCIF_DPI_HSYN_PAR			0x44
#define DCIF_DPI_VSYN_PAR			0x48
#define DCIF_DPI_VSYN_HSYN_WIDTH		0x4C

/* Control Descriptor Registers, n=0-1*/
#define DCIF_CTRLDESC0(n)			(0x10000 + (n) * 0x10000)
#define DCIF_CTRLDESC1(n)			(0x10004 + (n) * 0x10000)
#define DCIF_CTRLDESC2(n)			(0x10008 + (n) * 0x10000)
#define DCIF_CTRLDESC3(n)			(0x1000C + (n) * 0x10000)
#define DCIF_CTRLDESC4(n)			(0x10010 + (n) * 0x10000)
#define DCIF_CTRLDESC5(n)			(0x10014 + (n) * 0x10000)
#define DCIF_CTRLDESC6(n)			(0x10018 + (n) * 0x10000)

/* CLUT control Register */
#define DCIF_CLUT_CTRL				0x1003C

/* FIFO Panic Threshold Register, n=0-1 */
#define DCIF_PANIC_THRES(n)			(0x10040 + (n) * 0x10000)

/* Layer Status Register 0, n=0-1 */
#define DCIF_LAYER_SR0(n)			(0x10044 + (n) * 0x10000)

/* Color Space Conversion Control and Coefficient Registers for Layer 0 */
#define DCIF_CSC_CTRL_L0			0x10050
#define DCIF_CSC_COEF0_L0			0x10054
#define DCIF_CSC_COEF1_L0			0x10058
#define DCIF_CSC_COEF2_L0			0x1005C
#define DCIF_CSC_COEF3_L0			0x10060
#define DCIF_CSC_COEF4_L0			0x10064
#define DCIF_CSC_COEF5_L0			0x10068

/* CRC Control, Threshold, and Histogram Coefficient Registers */
#define DCIF_CRC_CTRL				0x20100
#define DCIF_CRC_THRES				0x20104
#define DCIF_CRC_HIST_COEF			0x20108
#define DCIF_CRC_ERR_CNT			0x2010C
#define DCIF_CRC_SR				0x20110
#define DCIF_CRC_HIST_CNT_B(n)			(0x20114 + (n) * 4)

/* CRC Region Position, Size, Value, and Expected Value Registers, n=0-3 */
#define DCIF_CRC_POS_R(n)			(0x20214 + (n) * 0x10)
#define DCIF_CRC_SIZE_R(n)			(0x20218 + (n) * 0x10)
#define DCIF_CRC_VAL_R(n)			(0x2021C + (n) * 0x10)
#define DCIF_CRC_EXP_VAL_R(n)			(0x20220 + (n) * 0x10)

#define REG_PUT(x, h, l)			(((x) << (l)) & GENMASK(h, l))
#define REG_GET(x, h, l)			(((x) & GENMASK(h, l)) >> (l))

/* VER - Version ID Register */
#define DCIF_VER_GET_FEATURE(x)			REG_GET((x), 15, 0)
#define DCIF_VER_GET_MINOR(x)			REG_GET((x), 23, 16)
#define DCIF_VER_GET_MAJOR(x)			REG_GET((x), 31, 24)

/* PAR_0 - Parameter Register 0 */
#define DCIF_PAR_0_LAYER_NUM(x)			REG_PUT((x), 3, 0)
#define DCIF_PAR_0_DOMAIN_NUM(x)		REG_PUT((x), 5, 4)
#define DCIF_PAR_0_AXI_DATA_WIDTH(x)		REG_PUT((x), 7, 6)
#define DCIF_PAR_0_CLUT_RAM_NUM(x)		REG_PUT((x), 11, 8)
#define DCIF_PAR_0_CSC_NUM(x)			REG_PUT((x), 13, 12)
#define DCIF_PAR_0_CRC_REGION_NUM(x)		REG_PUT((x), 18, 16)
#define DCIF_PAR_0_BACKUP(x)			REG_PUT((x), 31, 28)

/* PAR_1 - Parameter Register 1 */
#define DCIF_PAR_1_LAYER0_FIFO_SIZE(x)		REG_PUT((x), 3, 0)
#define DCIF_PAR_1_LAYER1_FIFO_SIZE(x)		REG_PUT((x), 7, 4)

/* DISP_CTRL - Display Control Register */
#define DCIF_DISP_CTRL_DISP_ON			BIT(0)
#define DCIF_DISP_CTRL_AXI_RD_HOLD		BIT(30)
#define DCIF_DISP_CTRL_SW_RST			BIT(31)

/* DISP_PAR - Display Parameter Register */
#define DCIF_DISP_PAR_BGND_B(x)			REG_PUT((x), 7, 0)
#define DCIF_DISP_PAR_BGND_G(x)			REG_PUT((x), 15, 8)
#define DCIF_DISP_PAR_BGND_R(x)			REG_PUT((x), 23, 16)

/* DISP_SIZE - Display Size Register */
#define DCIF_DISP_SIZE_DISP_WIDTH(x)		REG_PUT((x), 11, 0)
#define DCIF_DISP_SIZE_DISP_HEIGHT(x)		REG_PUT((x), 27, 16)

/* DISP_SR0 - Display Status Register 0 */
#define DCIF_DISP_SR0_AXI_RD_PEND(x)		REG_PUT((x), 4, 0)
#define DCIF_DISP_SR0_DPI_BUSY(x)		REG_PUT((x), 14, 14)
#define DCIF_DISP_SR0_AXI_RD_BUSY(x)		REG_PUT((x), 15, 15)
#define DCIF_DISP_SR0_TXFIFO_CNT(x)		REG_PUT((x), 23, 16)

/* DISP_SR1 - Display Status Register 1 */
#define DCIF_DISP_SR1_H_CNT(x)			REG_PUT((x), 11, 0)
#define DCIF_DISP_SR1_V_CNT(x)			REG_PUT((x), 27, 16)

/* INT0 - Interrupt Enable/Status  Register 0 for Domain 0/1/2 */
#define DCIF_INT0_VSYNC				BIT(0)
#define DCIF_INT0_UNDERRUN			BIT(1)
#define DCIF_INT0_VS_BLANK			BIT(2)
#define DCIF_INT0_HIST_DONE			BIT(5)
#define DCIF_INT0_CRC_ERR			BIT(6)
#define DCIF_INT0_CRC_ERR_SAT			BIT(7)

/* INT1 - Interrupt Enable/Status Register 1 for Domain 0/1/2 */
#define DCIF_INT1_FIFO_PANIC0			BIT(0)
#define DCIF_INT1_FIFO_PANIC1			BIT(1)
#define DCIF_INT1_DMA_ERR0			BIT(8)
#define DCIF_INT1_DMA_ERR1			BIT(9)
#define DCIF_INT1_DMA_DONE0			BIT(16)
#define DCIF_INT1_DMA_DONE1			BIT(17)
#define DCIF_INT1_FIFO_EMPTY0			BIT(24)
#define DCIF_INT1_FIFO_EMPTY1			BIT(25)

/* DPI_CTRL - DPI Control Register */
#define DCIF_DPI_CTRL_HSYNC_POL_LOW		BIT(0)
#define DCIF_DPI_CTRL_VSYNC_POL_LOW		BIT(1)
#define DCIF_DPI_CTRL_DE_POL_LOW		BIT(2)
#define DCIF_DPI_CTRL_PCLK_EDGE_FALLING		BIT(3)
#define DCIF_DPI_CTRL_POL_MASK			GENMASK(3, 0)

#define DCIF_DPI_CTRL_DATA_INV(x)		REG_PUT((x), 4, 4)
#define DCIF_DPI_CTRL_DEF_BGND_EN(x)		REG_PUT((x), 5, 5)
#define DCIF_DPI_CTRL_FETCH_OPT(x)		REG_PUT((x), 9, 8)
#define DCIF_DPI_CTRL_DISP_MODE(x)		REG_PUT((x), 13, 12)
#define DCIF_DPI_CTRL_DATA_PATTERN_MASK		GENMASK(18, 16)
#define DCIF_DPI_CTRL_DATA_PATTERN(x)		REG_PUT((x), 18, 16)
#define PATTERN_RGB888				0
#define PATTERN_RBG888				1
#define PATTERN_GBR888				2
#define PATTERN_GRB888				3
#define PATTERN_BRG888				4
#define PATTERN_BGR888				5
#define PATTERN_RGB555				6
#define PATTERN_RGB565				7

/* DPI_HSYN_PAR - DPI Horizontal Sync Parameter Register */
#define DCIF_DPI_HSYN_PAR_FP_H(x)		REG_PUT((x), 11, 0)
#define DCIF_DPI_HSYN_PAR_BP_H(x)		REG_PUT((x), 27, 16)

/* DPI_VSYN_PAR - DPI Vertical Sync Parameter Register */
#define DCIF_DPI_VSYN_PAR_FP_V(x)		REG_PUT((x), 11, 0)
#define DCIF_DPI_VSYN_PAR_BP_V(x)		REG_PUT((x), 27, 16)

/* DPI_VSYN_HSYN_WIDTH - DPI Vertical and Horizontal Pulse Width Parameter Register */
#define DCIF_DPI_VSYN_HSYN_WIDTH_PW_H(x)	REG_PUT((x), 11, 0)
#define DCIF_DPI_VSYN_HSYN_WIDTH_PW_V(x)	REG_PUT((x), 27, 16)

/* CTRLDESC0 - Control Descriptor Register 0 */
#define DCIF_CTRLDESC0_AB_MODE(x)		REG_PUT((x), 1, 0)
#define ALPHA_EMBEDDED				0
#define ALPHA_GLOBAL				1
#define DCIF_CTRLDESC0_YUV_FORMAT_MASK		GENMASK(15, 14)
#define DCIF_CTRLDESC0_YUV_FORMAT(x)		REG_PUT((x), 15, 14)
#define CTRLDESCL0_YUV_FORMAT_Y2VY1U		0x0
#define CTRLDESCL0_YUV_FORMAT_Y2UY1V		0x1
#define CTRLDESCL0_YUV_FORMAT_VY2UY1		0x2
#define CTRLDESCL0_YUV_FORMAT_UY2VY1		0x3
#define DCIF_CTRLDESC0_GLOBAL_ALPHA(x)		REG_PUT((x), 23, 16)
#define DCIF_CTRLDESC0_FORMAT_MASK		GENMASK(27, 24)
#define DCIF_CTRLDESC0_FORMAT(x)		REG_PUT((x), 27, 24)
#define CTRLDESCL0_FORMAT_RGB565		0x4
#define CTRLDESCL0_FORMAT_ARGB1555		0x5
#define CTRLDESCL0_FORMAT_ARGB4444		0x6
#define CTRLDESCL0_FORMAT_YCBCR422		0x7
#define CTRLDESCL0_FORMAT_RGB888		0x8
#define CTRLDESCL0_FORMAT_ARGB8888		0x9
#define CTRLDESCL0_FORMAT_ABGR8888		0xa
#define DCIF_CTRLDESC0_SHADOW_LOAD_EN		BIT(30)
#define DCIF_CTRLDESC0_EN			BIT(31)

/* CTRLDESC1 - Control Descriptor Register 1 */
#define DCIF_CTRLDESC1_POSX(x)			REG_PUT((x), 11, 0)
#define DCIF_CTRLDESC1_POSY(x)			REG_PUT((x), 27, 16)

/* CTRLDESC2 - Control Descriptor Register */
#define DCIF_CTRLDESC2_WIDTH(x)			REG_PUT((x), 11, 0)
#define DCIF_CTRLDESC2_HEIGHT(x)		REG_PUT((x), 27, 16)

/* CTRLDESC3 - Control Descriptor Register 3 */
#define DCIF_CTRLDESC3_PITCH(x)			REG_PUT((x), 15, 0)
#define DCIF_CTRLDESC3_T_SIZE(x)		REG_PUT((x), 17, 16)
#define DCIF_CTRLDESC3_P_SIZE(x)		REG_PUT((x), 22, 20)

/* CTRLDESC4 - Control Descriptor Register 4 */
#define DCIF_CTRLDESC4_ADDR(x)REG_PUT((x), 31, 0)

/* CTRLDESC6 - Control Descriptor Register 6  */
#define DCIF_CTRLDESC6_BCLR_B(x)		REG_PUT((x), 7, 0)
#define DCIF_CTRLDESC6_BCLR_G(x)		REG_PUT((x), 15, 8)
#define DCIF_CTRLDESC6_BCLR_R(x)		REG_PUT((x), 23, 16)
#define DCIF_CTRLDESC6_BCLR_A(x)		REG_PUT((x), 31, 24)

/* CLUT_CTRL - CLUT control Register */
#define DCIF_CLUT_CTRL_CLUT0_SEL(x)		REG_PUT((x), 0, 0)
#define DCIF_CLUT_CTRL_CLUT1_SEL(x)		REG_PUT((x), 3, 3)
#define DCIF_CLUT_CTRL_CLUT_MUX(x)		REG_PUT((x), 29, 28)
#define DCIF_CLUT_CTRL_CLUT_SHADOW_LOAD_EN(x)	REG_PUT((x), 31, 31)

/* PANIC_THRES_L0 - FIFO Panic Threshold Register For Layer 0 */
#define DCIF_PANIC_THRES_LOW_MASK		GENMASK(11, 0)
#define DCIF_PANIC_THRES_LOW(x)			REG_PUT((x), 11, 00)
#define DCIF_PANIC_THRES_HIGH_MASK		GENMASK(27, 16)
#define DCIF_PANIC_THRES_HIGH(x)		REG_PUT((x), 27, 16)
#define DCIF_PANIC_THRES_REQ_EN			BIT(31)
#define PANIC0_THRES_MAX			511

/* LAYER_SR0_L0 - Layer Status Register 0 for Layer 0 */
#define DCIF_LAYER_SR0_L0_FIFO_CNT_MASK		GENMASK(9, 0)
#define DCIF_LAYER_SR0_L0_FIFO_CNT(x)		REG_PUT((x), 9, 0)

/* CSC_CTRL_L0 - Color Space Conversion Control Register For Layer 0 */
#define DCIF_CSC_CTRL_L0_CSC_EN			BIT(0)
#define DCIF_CSC_CTRL_L0_CSC_MODE_YCBCR2RGB	BIT(1)

/* CSC_COEF0_L0 - Color Space Conversion Coefficient Register 0 For Layer 0 */
#define DCIF_CSC_COEF0_L0_A1(x)			REG_PUT((x), 10, 0)
#define DCIF_CSC_COEF0_L0_A2(x)			REG_PUT((x), 26, 16)

/* CSC_COEF1_L0 - Color Space Conversion Coefficient Register 1 For Layer 0 */
#define DCIF_CSC_COEF1_L0_A3(x)			REG_PUT((x), 10, 0)
#define DCIF_CSC_COEF1_L0_B1(x)			REG_PUT((x), 26, 16)

/* CSC_COEF2_L0 - Color Space Conversion Coefficient Register 2 For Layer 0 */
#define DCIF_CSC_COEF2_L0_B2(x)			REG_PUT((x), 10, 0)
#define DCIF_CSC_COEF2_L0_B3(x)			REG_PUT((x), 26, 16)

/* CSC_COEF3_L0 - Color Space Conversion Coefficient Register 3 For Layer 0 */
#define DCIF_CSC_COEF3_L0_C1(x)			REG_PUT((x), 10, 0)
#define DCIF_CSC_COEF3_L0_C2(x)			REG_PUT((x), 26, 16)

/* CSC_COEF4_L0 - Color Space Conversion Coefficient Register 4 For Layer 0 */
#define DCIF_CSC_COEF4_L0_C3(x)			REG_PUT((x), 10, 0)
#define DCIF_CSC_COEF4_L0_D1(x)			REG_PUT((x), 8, 0)

/* CSC_COEF5_L0 - Color Space Conversion Coefficient Register 5 For Layer 0 */
#define DCIF_CSC_COEF5_L0_D2(x)			REG_PUT((x), 8, 0)
#define DCIF_CSC_COEF5_L0_D3(x)			REG_PUT((x), 26, 16)

/* CTRLDESC0_L1 - Control Descriptor Register 0 for Layer 1 */
#define DCIF_CTRLDESC0_L1_AB_MODE_MASK		GENMASK(1, 0)
#define DCIF_CTRLDESC0_L1_AB_MODE(x)		REG_PUT((x), 1, 0)

/* CRC_CTRL - CRC Control Register */
#define DCIF_CRC_CTRL_CRC_EN(x)			(1 << (x))
#define DCIF_CRC_CTRL_HIST_REGION_SEL(x)	REG_PUT((x), 17, 16)
#define DCIF_CRC_CTRL_HIST_MODE			BIT(21)
#define DCIF_CRC_CTRL_HIST_TRIG			BIT(22)
#define DCIF_CRC_CTRL_HIST_EN			BIT(23)
#define DCIF_CRC_CTRL_CRC_MODE			BIT(28)
#define DCIF_CRC_CTRL_CRC_TRIG			BIT(29)
#define DCIF_CRC_CTRL_CRC_ERR_CNT_RST		BIT(30)
#define DCIF_CRC_CTRL_CRC_SHADOW_LOAD_EN	BIT(31)

/* CRC_THRES - CRC Threshold Register */
#define DCIF_CRC_THRES_CRC_THRESHOLD_MASK	GENMASK(31, 0)
#define DCIF_CRC_THRES_CRC_THRESHOLD(x)		REG_PUT((x), 31, 0)

/* CRC_HIST_COEF - CRC Region Histogram Coefficient Register */
#define DCIF_CRC_HIST_COEF_HIST_WB_MASK		GENMASK(7, 0)
#define DCIF_CRC_HIST_COEF_HIST_WB(x)		REG_PUT((x), 7, 0)
#define DCIF_CRC_HIST_COEF_HIST_WG_MASK		GENMASK(15, 8)
#define DCIF_CRC_HIST_COEF_HIST_WG(x)		REG_PUT((x), 15, 8)
#define DCIF_CRC_HIST_COEF_HIST_WR_MASK		GENMASK(23, 16)
#define DCIF_CRC_HIST_COEF_HIST_WR(x)		REG_PUT((x), 23, 16)

/* CRC_ERR_CNT - CRC Error Counter Register */
#define DCIF_CRC_ERR_CNT_CRC_ERR_CNT_MASK	GENMASK(31, 0)
#define DCIF_CRC_ERR_CNT_CRC_ERR_CNT(x)		REG_PUT((x), 31, 0)

/* CRC_SR - CRC Status Register */
#define DCIF_CRC_SR_HIST_CNT_SAT_MASK		BIT(13)
#define DCIF_CRC_SR_HIST_CNT_SAT(x)		REG_PUT((x), 13, 13)
#define DCIF_CRC_SR_HIST_SAT_MASK		BIT(14)
#define DCIF_CRC_SR_HIST_SAT(x)			REG_PUT((x), 14, 14)
#define DCIF_CRC_SR_HIST_BUSY_MASK		BIT(15)
#define DCIF_CRC_SR_HIST_BUSY(x)		REG_PUT((x), 15, 15)
#define DCIF_CRC_SR_CRC_STATUS_MASK		BIT(31)
#define DCIF_CRC_SR_CRC_STATUS(x)		REG_PUT((x), 31, 31)

/* CRC Region Histogram Counter Register For Bin n */
#define DCIF_B_BIN_CNT_MASK			GENMASK(20, 0)
#define DCIF_B_BIN_CNT(x)			REG_PUT((x), 20, 0)

/* CRC_POS - CRC Position Register */
#define DCIF_CRC_POS_CRC_HOR_POS(x)		REG_PUT((x), 11, 0)
#define DCIF_CRC_POS_CRC_VER_POS(x)		REG_PUT((x), 27, 16)

/* CRC_SIZE - CRC Size Register */
#define DCIF_CRC_SIZE_CRC_HOR_SIZE(x)		REG_PUT((x), 11, 0)
#define DCIF_CRC_SIZE_CRC_VER_SIZE(x)		REG_PUT((x), 27, 16)

/* CRC_VAL - CRC Value Register */
#define DCIF_CRC_VAL_CRC_VAL_MASK		GENMASK(31, 0)
#define DCIF_CRC_VAL_CRC_VAL(x)			REG_PUT((x), 31, 0)

/* CRC_EXP_VAL - CRC Expected Value Register */
#define DCIF_CRC_EXP_VAL_CRC_EXP_VAL_MASK	GENMASK(31, 0)
#define DCIF_CRC_EXP_VAL_CRC_EXP_VAL(x)		REG_PUT((x), 31, 0)

#endif /* __DCIF_REG_H__ */
