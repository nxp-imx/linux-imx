/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Coda series multi-standard codec IP - coda register definitions
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#ifndef __CODA_REGDEFINE_H__
#define __CODA_REGDEFINE_H__

enum coda_vpu_command {
	ENC_SEQ_INIT = 1,
	ENC_SEQ_END = 2,
	PIC_RUN = 3,
	SET_FRAME_BUF = 4,
	ENCODE_HEADER = 5,
	ENC_PARA_SET = 6,
	ENC_PARAM_CHANGE = 9,
	VPU_SLEEP = 10,
	VPU_WAKE = 11,
	ENC_ROI_INIT = 12,
	FIRMWARE_GET = 15
};

enum coda_interrupt_bit {
	INT_BIT_INIT = 0,
	INT_BIT_SEQ_INIT = 1,
	INT_BIT_SEQ_END = 2,
	INT_BIT_PIC_RUN = 3,
	INT_BIT_FRAMEBUF_SET = 4,
	INT_BIT_ENC_HEADER = 5,
	INT_BIT_USERDATA = 9,
	INT_BIT_BIT_BUF_EMPTY = 14,
	INT_BIT_BIT_BUF_FULL = 15
};

enum coda_param_change_enable {
	PARAM_CHANGE_ENABLE_BITRATE = 2
};

/************************************************************/
/* REGISTER BASE                                            */
/************************************************************/
#define BIT_BASE        0x0000
#define MBC_BASE        0x0400
#define ME_BASE         0x0600
#define DBK_BASE        0x0800
#define MC_BASE         0x0C00
#define GDMA_BASE       0x1000

/************************************************************/
/* HARDWARE REGISTER                                        */
/************************************************************/
#define VPU_PRODUCT_NAME        0x1040
#define VPU_PRODUCT_CODE        0x1044

// SW reset command
#define VPU_SW_RESET_BPU_CORE   0x008
#define VPU_SW_RESET_BPU_BUS    0x010
#define VPU_SW_RESET_VCE_CORE   0x020
#define VPU_SW_RESET_VCE_BUS    0x040
#define VPU_SW_RESET_GDI_CORE   0x080
#define VPU_SW_RESET_GDI_BUS    0x100

#define BIT_CODE_RUN            (BIT_BASE + 0x000)
#define BIT_CODE_DOWN           (BIT_BASE + 0x004)
#define BIT_INT_REQ             (BIT_BASE + 0x008)
#define BIT_INT_CLEAR           (BIT_BASE + 0x00C)
#define BIT_INT_STS             (BIT_BASE + 0x010)
#define BIT_CODE_RESET          (BIT_BASE + 0x014)
#define BIT_CUR_PC              (BIT_BASE + 0x018)
#define BIT_SW_RESET            (BIT_BASE + 0x024)
#define BIT_SW_RESET_STATUS     (BIT_BASE + 0x034)
#define MBC_SET_SUBBLK_EN       (MBC_BASE + 0x0A0)

/************************************************************/
/* GLOBAL REGISTER                                          */
/************************************************************/
#define BIT_CODE_BUF_ADDR               (BIT_BASE + 0x100)
#define BIT_WORK_BUF_ADDR               (BIT_BASE + 0x104)
#define BIT_PARA_BUF_ADDR               (BIT_BASE + 0x108)
#define BIT_BIT_STREAM_CTRL             (BIT_BASE + 0x10C)
#define BIT_FRAME_MEM_CTRL              (BIT_BASE + 0x110)
#define BIT_BIT_STREAM_PARAM            (BIT_BASE + 0x114)
#define BIT_TEMP_BUF_ADDR               (BIT_BASE + 0x118)
#define BIT_RD_PTR                      (BIT_BASE + 0x120)
#define BIT_WR_PTR                      (BIT_BASE + 0x124)
#define BIT_ROLLBACK_STATUS             (BIT_BASE + 0x128)
#define BIT_AXI_SRAM_USE                (BIT_BASE + 0x140)
#define BIT_BYTE_POS_FRAME_START        (BIT_BASE + 0x144)
#define BIT_BYTE_POS_FRAME_END          (BIT_BASE + 0x148)
#define BIT_FRAME_CYCLE                 (BIT_BASE + 0x14C)
#define BIT_FRM_DIS_FLG                 (BIT_BASE + 0x150)
#define BIT_BUSY_FLAG                   (BIT_BASE + 0x160)
#define BIT_RUN_COMMAND                 (BIT_BASE + 0x164)
#define BIT_RUN_INDEX                   (BIT_BASE + 0x168)
#define BIT_RUN_COD_STD                 (BIT_BASE + 0x16C)
#define BIT_INT_ENABLE                  (BIT_BASE + 0x170)
#define BIT_INT_REASON                  (BIT_BASE + 0x174)
#define BIT_RUN_AUX_STD                 (BIT_BASE + 0x178)

/************************************************************/
/* GDMA MODULE REGISTER                                     */
/************************************************************/
#define GDI_PINFO_REQ                   (GDMA_BASE + 0x060)
#define GDI_PINFO_ACK                   (GDMA_BASE + 0x064)
#define GDI_PINFO_ADDR                  (GDMA_BASE + 0x068)
#define GDI_PINFO_DATA                  (GDMA_BASE + 0x06C)
#define GDI_BWB_STATUS                  (GDMA_BASE + 0x07C)

// Write protect
#define GDI_WPROT_ERR_CLR               (GDMA_BASE + 0x0A0)
#define GDI_WPROT_ERR_RSN               (GDMA_BASE + 0x0A4)
#define GDI_WPROT_ERR_ADR               (GDMA_BASE + 0x0A8)
#define GDI_WPROT_RGN_EN                (GDMA_BASE + 0x0AC)
#define GDI_WPROT_RGN0_STA              (GDMA_BASE + 0x0B0)
#define GDI_WPROT_RGN0_END              (GDMA_BASE + 0x0B4)
#define GDI_WPROT_RGN1_STA              (GDMA_BASE + 0x0B8)
#define GDI_WPROT_RGN1_END              (GDMA_BASE + 0x0BC)
#define GDI_WPROT_RGN2_STA              (GDMA_BASE + 0x0C0)
#define GDI_WPROT_RGN2_END              (GDMA_BASE + 0x0C4)
#define GDI_WPROT_RGN3_STA              (GDMA_BASE + 0x0C8)
#define GDI_WPROT_RGN3_END              (GDMA_BASE + 0x0CC)
#define GDI_WPROT_RGN4_STA              (GDMA_BASE + 0x0D0)
#define GDI_WPROT_RGN4_END              (GDMA_BASE + 0x0D4)
#define GDI_WPROT_RGN5_STA              (GDMA_BASE + 0x0D8)
#define GDI_WPROT_RGN5_END              (GDMA_BASE + 0x0DC)
#define GDI_WPROT_REGIONS               6

#define GDI_BUS_CTRL                    (GDMA_BASE + 0x0F0)
#define GDI_BUS_STATUS                  (GDMA_BASE + 0x0F4)

// GDI 2.0
#define GDI_INFO_CONTROL                (GDMA_BASE + 0x400)
#define GDI_INFO_PIC_SIZE               (GDMA_BASE + 0x404)
#define GDI_INFO_BASE_Y_TOP             (GDMA_BASE + 0x408)
#define GDI_INFO_BASE_CB_TOP            (GDMA_BASE + 0x40C)
#define GDI_INFO_BASE_CR_TOP            (GDMA_BASE + 0x410)
#define GDI_INFO_BASE_Y_BOT             (GDMA_BASE + 0x414)
#define GDI_INFO_BASE_CB_BOT            (GDMA_BASE + 0x418)
#define GDI_INFO_BASE_CR_BOT            (GDMA_BASE + 0x41C)
#define GDI_XY2AXI_LUM_BIT00            (GDMA_BASE + 0x800)
#define GDI_XY2AXI_LUM_BIT1F            (GDMA_BASE + 0x87C)
#define GDI_XY2AXI_CHR_BIT00            (GDMA_BASE + 0x880)
#define GDI_XY2AXI_CHR_BIT1F            (GDMA_BASE + 0x8FC)
#define GDI_XY2AXI_CONFIG               (GDMA_BASE + 0x900)

/************************************************************/
/* COMMON - FIRMWARE_GET                                    */
/************************************************************/
#define RET_FW_VER_NUM                  (BIT_BASE + 0x1C0)
#define RET_FW_CODE_REV                 (BIT_BASE + 0x1C4)

/************************************************************/
/* ENCODER - SET_FRAME_BUF                                  */
/************************************************************/
#define CMD_SET_FRAME_BUF_NUM           (BIT_BASE + 0x180)
#define CMD_SET_FRAME_BUF_STRIDE        (BIT_BASE + 0x184)
#define CMD_SET_FRAME_AXI_BIT_ADDR      (BIT_BASE + 0x190)
#define CMD_SET_FRAME_AXI_IPACDC_ADDR   (BIT_BASE + 0x194)
#define CMD_SET_FRAME_AXI_DBKY_ADDR     (BIT_BASE + 0x198)
#define CMD_SET_FRAME_AXI_DBKC_ADDR     (BIT_BASE + 0x19C)
#define CMD_SET_FRAME_CACHE_SIZE        (BIT_BASE + 0x1A8)
#define CMD_SET_FRAME_CACHE_CONFIG      (BIT_BASE + 0x1AC)
#define CMD_SET_FRAME_DP_BUF_BASE       (BIT_BASE + 0x1B0)
#define CMD_SET_FRAME_DP_BUF_SIZE       (BIT_BASE + 0x1B4)

#define RET_SET_FRAME_SUCCESS           (BIT_BASE + 0x1C0)

/************************************************************/
/* ENCODER - SEQ_INIT                                       */
/************************************************************/
#define CMD_ENC_SEQ_BB_START            (BIT_BASE + 0x180)
#define CMD_ENC_SEQ_BB_SIZE             (BIT_BASE + 0x184)
#define CMD_ENC_SEQ_OPTION              (BIT_BASE + 0x188)
#define CMD_ENC_SEQ_COD_STD             (BIT_BASE + 0x18C)
#define CMD_ENC_SEQ_SRC_SIZE            (BIT_BASE + 0x190)
#define CMD_ENC_SEQ_SRC_F_RATE          (BIT_BASE + 0x194)
#define CMD_ENC_SEQ_MP4_PARA            (BIT_BASE + 0x198)
#define CMD_ENC_SEQ_263_PARA            (BIT_BASE + 0x19C)
#define CMD_ENC_SEQ_264_PARA            (BIT_BASE + 0x1A0)
#define CMD_ENC_SEQ_SLICE_MODE          (BIT_BASE + 0x1A4)
#define CMD_ENC_SEQ_GOP_NUM             (BIT_BASE + 0x1A8)
#define CMD_ENC_SEQ_RC_PARA             (BIT_BASE + 0x1AC)
#define CMD_ENC_SEQ_INTRA_REFRESH       (BIT_BASE + 0x1B4)
#define CMD_ENC_SEQ_INTRA_QP            (BIT_BASE + 0x1C4)
#define CMD_ENC_SEQ_RC_GAMMA            (BIT_BASE + 0x1CC)
#define CMD_ENC_SEQ_RC_INTERVAL_MODE    (BIT_BASE + 0x1D0)
#define CMD_ENC_SEQ_INTRA_WEIGHT        (BIT_BASE + 0x1D4)
#define CMD_ENC_SEQ_ME_OPTION           (BIT_BASE + 0x1D8)
#define CMD_ENC_SEQ_RC_PARA2            (BIT_BASE + 0x1DC)
#define CMD_ENC_SEQ_QP_RANGE_SET        (BIT_BASE + 0x1E0)
#define CMD_ENC_SEQ_RC_MAX_INTRA_SIZE   (BIT_BASE + 0x1F0)

#define RET_ENC_SEQ_SUCCESS             (BIT_BASE + 0x1C0)

/************************************************************/
/* ENCODER - ENCODE_HEADER                                  */
/************************************************************/
#define CMD_ENC_HEADER_CODE             (BIT_BASE + 0x180)
#define CMD_ENC_HEADER_BB_START         (BIT_BASE + 0x184)
#define CMD_ENC_HEADER_BB_SIZE          (BIT_BASE + 0x188)
#define CMD_ENC_HEADER_FRAME_CROP_H     (BIT_BASE + 0x18C)
#define CMD_ENC_HEADER_FRAME_CROP_V     (BIT_BASE + 0x190)
#define CMD_ENC_HEADER_CABAC_MODE       (BIT_BASE + 0x194)
#define CMD_ENC_HEADER_CABAC_INIT_IDC   (BIT_BASE + 0x198)
#define CMD_ENC_HEADER_TRANSFORM_8X8    (BIT_BASE + 0x19C)
#define CMD_ENC_HEADER_CHROMA_FORMAT    (BIT_BASE + 0x1A0)
#define CMD_ENC_HEADER_FIELD_FLAG       (BIT_BASE + 0x1A4)
#define CMD_ENC_HEADER_PROFILE          (BIT_BASE + 0x1A8)
#define CMD_ENC_HEADER_VUI_INFO         (BIT_BASE + 0x1C4)
#define CMD_ENC_HEADER_VUI_EXTENDED_SAR (BIT_BASE + 0x1C8)
#define CMD_ENC_HEADER_VUI_COLOR_INFO   (BIT_BASE + 0x1CC)

#define RET_ENC_HEADER_SUCCESS          (BIT_BASE + 0x1C0)

/************************************************************/
/* ENCODER - PIC_RUN                                        */
/************************************************************/
#define BIT_ME_LINEBUFFER_MODE          (ME_BASE + 0x004)

#define CMD_ENC_PIC_SRC_INDEX           (BIT_BASE + 0x180)
#define CMD_ENC_PIC_SRC_STRIDE          (BIT_BASE + 0x184)
#define CMD_ENC_PIC_SRC_ADDR_Y          (BIT_BASE + 0x1A8)
#define CMD_ENC_PIC_SRC_ADDR_CB         (BIT_BASE + 0x1AC)
#define CMD_ENC_PIC_SRC_ADDR_CR         (BIT_BASE + 0x1B0)
#define CMD_ENC_PIC_SRC_BOTTOM_Y        (BIT_BASE + 0x1E8)
#define CMD_ENC_PIC_SRC_BOTTOM_CB       (BIT_BASE + 0x1EC)
#define CMD_ENC_PIC_SRC_BOTTOM_CR       (BIT_BASE + 0x1F0)
#define CMD_ENC_PIC_QS                  (BIT_BASE + 0x18C)
#define CMD_ENC_PIC_ROT_MODE            (BIT_BASE + 0x190)
#define CMD_ENC_PIC_OPTION              (BIT_BASE + 0x194)
#define CMD_ENC_PIC_BB_START            (BIT_BASE + 0x198)
#define CMD_ENC_PIC_BB_SIZE             (BIT_BASE + 0x19C)
#define CMD_ENC_PIC_PARA_BASE_ADDR      (BIT_BASE + 0x1A0)
#define CMD_ENC_PIC_SUB_FRAME_SYNC      (BIT_BASE + 0x1A4)
#ifdef SUPPORT_ENC_ELAPSED_TIME
#define CMD_ENC_PIC_ELAPSED_TIME        (BIT_BASE + 0x1C4)
#endif

#define RET_ENC_PIC_FRAME_NUM           (BIT_BASE + 0x1C0)
#define RET_ENC_PIC_TYPE                (BIT_BASE + 0x1C4)
#define RET_ENC_PIC_FRAME_IDX           (BIT_BASE + 0x1C8)
#define RET_ENC_PIC_SLICE_NUM           (BIT_BASE + 0x1CC)
#define RET_ENC_PIC_FLAG                (BIT_BASE + 0x1D0)
#define RET_ENC_PIC_SUCCESS             (BIT_BASE + 0x1D8)
#define RET_ENC_PIC_AVG_QP              (BIT_BASE + 0x1DC)

/************************************************************/
/* ENCODER - PARAM_CHANGE                                   */
/************************************************************/
#define CMD_ENC_PARAM_CHANGE_ENABLE     (BIT_BASE + 0x180)
#define CMD_ENC_PARAM_CHANGE_BITRATE    (BIT_BASE + 0x18C)

#define RET_ENC_PARAM_CHANGE_SUCCESS    (BIT_BASE + 0x1C0)

#endif
