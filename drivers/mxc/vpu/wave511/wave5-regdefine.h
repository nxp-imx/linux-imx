/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave5 series multi-standard codec IP - wave5 register definitions
 *
 * Copyright (C) 2021-2023 CHIPS&MEDIA INC
 */

#ifndef __WAVE5_REGISTER_DEFINE_H__
#define __WAVE5_REGISTER_DEFINE_H__

enum W5_VPU_COMMAND {
	W5_INIT_VPU		= 0x0001,
	W5_WAKEUP_VPU		= 0x0002,
	W5_SLEEP_VPU		= 0x0004,
	W5_CREATE_INSTANCE	= 0x0008,       /* queuing command */
	W5_FLUSH_INSTANCE	= 0x0010,
	W5_DESTROY_INSTANCE	= 0x0020,       /* queuing command */
	W5_INIT_SEQ		= 0x0040,       /* queuing command */
	W5_SET_FB		= 0x0080,
	W5_DEC_PIC		= 0x0100,       /* queuing command */
	W5_INIT_WORK_BUF        = 0x1000,
	W5_QUERY		= 0x4000,
	W5_UPDATE_BS		= 0x8000,
	W5_MAX_VPU_COMD		= 0x10000,
};

enum query_opt {
	GET_VPU_INFO		= 0,
	SET_WRITE_PROT		= 1,
	GET_RESULT		= 2,
	UPDATE_DISP_FLAG	= 3,
	GET_BW_REPORT		= 4,
	GET_BS_RD_PTR		= 5,		/* for decoder */
	SET_BS_RD_PTR		= 8,		/* for decoder */
	GET_DEBUG_INFO		= 0x61,
};

#define W5_REG_BASE                     0x00000000
#define W5_CMD_REG_BASE                 0x00000100
#define W5_CMD_REG_END                  0x00000200

/*
 * COMMON
 *
 * ----
 *
 * Power on configuration
 * PO_DEBUG_MODE    [0]     1 - power on with debug mode
 * USE_PO_CONF      [3]     1 - use power-on-configuration
 */
#define W5_VPU_PO_CONF                 (W5_REG_BASE + 0x0000)
#define W5_VCPU_CUR_PC                 (W5_REG_BASE + 0x0004)
#define W5_VCPU_CUR_LR                 (W5_REG_BASE + 0x0008)
#define W5_VPU_PDBG_STEP_MASK_V        (W5_REG_BASE + 0x000C)
#define W5_VPU_PDBG_CTRL               (W5_REG_BASE + 0x0010) /* v_cpu debugger ctrl register */
#define W5_VPU_PDBG_IDX_REG            (W5_REG_BASE + 0x0014) /* v_cpu debugger index register */
#define W5_VPU_PDBG_WDATA_REG          (W5_REG_BASE + 0x0018) /* v_cpu debugger write data reg */
#define W5_VPU_PDBG_RDATA_REG          (W5_REG_BASE + 0x001C) /* v_cpu debugger read data reg */

#define W5_VPU_FIO_CTRL_ADDR           (W5_REG_BASE + 0x0020)
#define W5_VPU_FIO_DATA                (W5_REG_BASE + 0x0024)
#define W5_VPU_VINT_REASON_USR         (W5_REG_BASE + 0x0030)
#define W5_VPU_VINT_REASON_CLR         (W5_REG_BASE + 0x0034)
#define W5_VPU_HOST_INT_REQ            (W5_REG_BASE + 0x0038)
#define W5_VPU_VINT_CLEAR              (W5_REG_BASE + 0x003C)
#define W5_VPU_HINT_CLEAR              (W5_REG_BASE + 0x0040)
#define W5_VPU_VPU_INT_STS             (W5_REG_BASE + 0x0044)
#define W5_VPU_VINT_ENABLE             (W5_REG_BASE + 0x0048)
#define W5_VPU_VINT_REASON             (W5_REG_BASE + 0x004C)
#define W5_VPU_RESET_REQ_GB            (W5_REG_BASE + 0x0050)
#define W5_RST_BLOCK_CCLK(_core)       BIT((_core))
#define W5_RST_BLOCK_CCLK_ALL          (0xff)
#define W5_RST_BLOCK_BCLK(_core)       (0x100 << (_core))
#define W5_RST_BLOCK_BCLK_ALL          (0xff00)
#define W5_RST_BLOCK_ACLK(_core)       (0x10000 << (_core))
#define W5_RST_BLOCK_ACLK_ALL          (0xff0000)
#define W5_RST_BLOCK_VCPU_ALL          (0x3f000000)
#define W5_RST_BLOCK_ALL               (0x3fffffff)
#define W5_VPU_RESET_STATUS            (W5_REG_BASE + 0x0054)

#define W5_VCPU_RESTART                (W5_REG_BASE + 0x0058)
#define W5_VPU_CLK_MASK                (W5_REG_BASE + 0x005C)

/* REMAP_CTRL
 * PAGE SIZE:   [8:0]   0x001 - 4K
 *                      0x002 - 8K
 *                      0x004 - 16K
 *                      ...
 *                      0x100 - 1M
 * REGION ATTR1 [10]    0     - normal
 *                      1     - make bus error for the region
 * REGION ATTR2 [11]    0     - normal
 *                      1     - bypass region
 * REMAP INDEX  [15:12]       - 0 ~ 3
 * ENDIAN       [19:16]       - NOTE: Currently not supported in this driver
 * AXI-ID       [23:20]       - upper AXI-ID
 * BUS_ERROR    [29]    0     - bypass
 *                      1     - make BUS_ERROR for unmapped region
 * BYPASS_ALL   [30]    1     - bypass all
 * ENABLE       [31]    1     - update control register[30:16]
 */
#define W5_VPU_REMAP_CTRL_GB                    (W5_REG_BASE + 0x0060)
#define W5_VPU_REMAP_VADDR_GB                   (W5_REG_BASE + 0x0064)
#define W5_VPU_REMAP_PADDR_GB                   (W5_REG_BASE + 0x0068)
#define W5_VPU_REMAP_CORE_START_GB              (W5_REG_BASE + 0x006C)
#define W5_VPU_BUSY_STATUS                      (W5_REG_BASE + 0x00BC)
#define W5_VPU_HALT_STATUS                      (W5_REG_BASE + 0x0074)
#define W5_VPU_VCPU_STATUS                      (W5_REG_BASE + 0x0078)
#define W5_VPU_RET_PRODUCT_VERSION              (W5_REG_BASE + 0x0094)
/*
 * assign vpu_config0          = {conf_map_converter_reg,      // [31]
 * conf_map_converter_sig,         // [30]
 * 8'd0,                        // [29:22]
 * conf_std_switch_en,          // [21]
 * conf_bg_detect,              // [20]
 * conf_3dnr_en,                // [19]
 * conf_one_axi_en,             // [18]
 * conf_sec_axi_en,             // [17]
 * conf_bus_info,               // [16]
 * conf_afbc_en,                // [15]
 * conf_afbc_version_id,        // [14:12]
 * conf_fbc_en,                 // [11]
 * conf_fbc_version_id,         // [10:08]
 * conf_scaler_en,              // [07]
 * conf_scaler_version_id,      // [06:04]
 * conf_bwb_en,                 // [03]
 * 3'd0};                       // [02:00]
 */
#define W5_VPU_RET_VPU_CONFIG0                  (W5_REG_BASE + 0x0098)
/*
 * assign vpu_config1          = {4'd0,                        // [31:28]
 * conf_perf_timer_en,          // [27]
 * conf_multi_core_en,          // [26]
 * conf_gcu_en,                 // [25]
 * conf_cu_report,              // [24]
 * 4'd0,                        // [23:20]
 * conf_vcore_id_3,             // [19]
 * conf_vcore_id_2,             // [18]
 * conf_vcore_id_1,             // [17]
 * conf_vcore_id_0,             // [16]
 * conf_bwb_opt,                // [15]
 * 7'd0,                        // [14:08]
 * conf_cod_std_en_reserved_7,  // [7]
 * conf_cod_std_en_reserved_6,  // [6]
 * conf_cod_std_en_reserved_5,  // [5]
 * conf_cod_std_en_reserved_4,  // [4]
 * conf_cod_std_en_reserved_3,  // [3]
 * conf_cod_std_en_reserved_2,  // [2]
 * conf_cod_std_en_vp9,         // [1]
 * conf_cod_std_en_hevc};       // [0]
 * }
 */
#define W5_VPU_RET_VPU_CONFIG1                  (W5_REG_BASE + 0x009C)

#define W5_VPU_DBG_REG0				(W5_REG_BASE + 0x00f0)
#define W5_VPU_DBG_REG1				(W5_REG_BASE + 0x00f4)
#define W5_CMD_SET_CTRL_WORK_BUF_ADDR		(W5_REG_BASE + 0x00f8)
#define W5_CMD_SET_CTRL_WORK_BUF_SIZE		(W5_REG_BASE + 0x00fc)

/************************************************************************/
/* DECODER COMMON                                                       */
/************************************************************************/
#define W5_COMMAND                              (W5_REG_BASE + 0x0100)
#define W5_COMMAND_GB                           (W5_REG_BASE + 0x0104)
#define W5_COMMAND_OPTION                       (W5_REG_BASE + 0x0104)
#define W5_QUERY_OPTION                         (W5_REG_BASE + 0x0104)
#define W5_RET_SUCCESS                          (W5_REG_BASE + 0x0108)
#define W5_RET_FAIL_REASON                      (W5_REG_BASE + 0x010C)
#define W5_RET_QUEUE_FAIL_REASON                (W5_REG_BASE + 0x0110)
#define W5_CMD_INSTANCE_INFO                    (W5_REG_BASE + 0x0110)

#define W5_RET_QUEUE_STATUS                     (W5_REG_BASE + 0x01E0)
#define W5_RET_BS_EMPTY_INST                    (W5_REG_BASE + 0x01E4)
#define W5_RET_QUEUE_CMD_DONE_INST              (W5_REG_BASE + 0x01E8)
#define W5_RET_STAGE0_INSTANCE_INFO             (W5_REG_BASE + 0x01EC)
#define W5_RET_STAGE1_INSTANCE_INFO             (W5_REG_BASE + 0x01F0)
#define W5_RET_STAGE2_INSTANCE_INFO             (W5_REG_BASE + 0x01F4)

#define W5_RET_SEQ_DONE_INSTANCE_INFO           (W5_REG_BASE + 0x01FC)

#define W5_BS_OPTION                            (W5_REG_BASE + 0x0120)

/* return info when QUERY (GET_RESULT) for en/decoder */
#define W5_RET_VLC_BUF_SIZE                     (W5_REG_BASE + 0x01B0)
/* return info when QUERY (GET_RESULT) for en/decoder */
#define W5_RET_PARAM_BUF_SIZE                   (W5_REG_BASE + 0x01B4)

/* set when SET_FB for en/decoder */
#define W5_CMD_SET_FB_ADDR_TASK_BUF             (W5_REG_BASE + 0x01D4)
#define W5_CMD_SET_FB_TASK_BUF_SIZE             (W5_REG_BASE + 0x01D8)
/************************************************************************/
/* INIT_VPU - COMMON                                                    */
/************************************************************************/
/* note: W5_CMD_INIT_ADDR_CODE_BASE should be aligned to 4KB */
#define W5_CMD_INIT_ADDR_CODE_BASE		(W5_REG_BASE + 0x0110)
#define W5_CMD_INIT_CODE_SIZE			(W5_REG_BASE + 0x0114)
#define W5_CMD_INIT_PARAM			(W5_REG_BASE + 0x0118)
#define W5_CMD_INIT_SEC_AXI_ADDR		(W5_REG_BASE + 0x0124)
#define W5_CMD_INIT_SEC_AXI_SIZE		(W5_REG_BASE + 0x0128)
#define W5_CMD_INIT_HW_OPTION                   (W5_REG_BASE + 0x012C)
#define W5_CMD_INIT_NUM_TASK_BUF		(W5_REG_BASE + 0x0134)
#define W5_CMD_INIT_ADDR_TASK_BUF0		(W5_REG_BASE + 0x0138)
#define W5_CMD_INIT_TASK_BUF_SIZE		(W5_REG_BASE + 0x0178)
#define W5_SEC_AXI_PARAM                        (W5_REG_BASE + 0x0180)

/************************************************************************/
/* CREATE_INSTANCE - COMMON                                             */
/************************************************************************/
#define W5_CMD_DEC_BS_START_ADDR                (W5_REG_BASE + 0x011C)
#define W5_CMD_DEC_BS_SIZE                      (W5_REG_BASE + 0x0120)
#define W5_CMD_BS_PARAM                         (W5_REG_BASE + 0x0124)
#define W5_CMD_ADDR_TEMP_BASE                   (W5_REG_BASE + 0x012C)
#define W5_CMD_TEMP_SIZE                        (W5_REG_BASE + 0x0130)
#define W5_CMD_EXT_ADDR                         (W5_REG_BASE + 0x0138)
#define W5_CMD_NUM_CQ_DEPTH_M1                  (W5_REG_BASE + 0x013C)
#define W5_CMD_ERR_CONCEAL                      (W5_REG_BASE + 0x0140)
#define W5_CMD_USER_DEFINED_ID                  (W5_REG_BASE + 0x0198)
#define W5_RET_INSTANCE_ID                      (W5_REG_BASE + 0x0144)
#define W5_RET_SEC_AXI_ADDR                     (W5_REG_BASE + 0x0148)
#define W5_RET_SEC_AXI_SIZE                     (W5_REG_BASE + 0x014C)

/************************************************************************/
/* DECODER - INIT_SEQ                                                   */
/************************************************************************/
#define W5_BS_RD_PTR                            (W5_REG_BASE + 0x0118)
#define W5_BS_WR_PTR                            (W5_REG_BASE + 0x011C)
/************************************************************************/
/* SET_FRAME_BUF                                                        */
/************************************************************************/
/* SET_FB_OPTION 0x00       REGISTER FRAMEBUFFERS
 * 0x01       UPDATE FRAMEBUFFER, just one framebuffer(linear, fbc and mvcol)
 */
#define W5_SFB_OPTION                           (W5_REG_BASE + 0x0104)
#define W5_COMMON_PIC_INFO                      (W5_REG_BASE + 0x0118)
#define W5_PIC_SIZE                             (W5_REG_BASE + 0x011C)
#define W5_SET_FB_NUM                           (W5_REG_BASE + 0x0120)
#define W5_EXTRA_PIC_INFO                       (W5_REG_BASE + 0x0124)

#define W5_ADDR_LUMA_BASE0                      (W5_REG_BASE + 0x0134)
#define W5_ADDR_CB_BASE0                        (W5_REG_BASE + 0x0138)
#define W5_ADDR_CR_BASE0                        (W5_REG_BASE + 0x013C)
/* compression offset table for luma */
#define W5_ADDR_FBC_Y_OFFSET0                   (W5_REG_BASE + 0x013C)
/* compression offset table for chroma */
#define W5_ADDR_FBC_C_OFFSET0                   (W5_REG_BASE + 0x0140)
#define W5_ADDR_LUMA_BASE1                      (W5_REG_BASE + 0x0144)
#define W5_ADDR_CB_ADDR1                        (W5_REG_BASE + 0x0148)
#define W5_ADDR_CR_ADDR1                        (W5_REG_BASE + 0x014C)
/* compression offset table for luma */
#define W5_ADDR_FBC_Y_OFFSET1                   (W5_REG_BASE + 0x014C)
/* compression offset table for chroma */
#define W5_ADDR_FBC_C_OFFSET1                   (W5_REG_BASE + 0x0150)
#define W5_ADDR_LUMA_BASE2                      (W5_REG_BASE + 0x0154)
#define W5_ADDR_CB_ADDR2                        (W5_REG_BASE + 0x0158)
#define W5_ADDR_CR_ADDR2                        (W5_REG_BASE + 0x015C)
/* compression offset table for luma */
#define W5_ADDR_FBC_Y_OFFSET2                   (W5_REG_BASE + 0x015C)
/* compression offset table for chroma */
#define W5_ADDR_FBC_C_OFFSET2                   (W5_REG_BASE + 0x0160)
#define W5_ADDR_LUMA_BASE3                      (W5_REG_BASE + 0x0164)
#define W5_ADDR_CB_ADDR3                        (W5_REG_BASE + 0x0168)
#define W5_ADDR_CR_ADDR3                        (W5_REG_BASE + 0x016C)
/* compression offset table for luma */
#define W5_ADDR_FBC_Y_OFFSET3                   (W5_REG_BASE + 0x016C)
/* compression offset table for chroma */
#define W5_ADDR_FBC_C_OFFSET3                   (W5_REG_BASE + 0x0170)
#define W5_ADDR_LUMA_BASE4                      (W5_REG_BASE + 0x0174)
#define W5_ADDR_CB_ADDR4                        (W5_REG_BASE + 0x0178)
#define W5_ADDR_CR_ADDR4                        (W5_REG_BASE + 0x017C)
/* compression offset table for luma */
#define W5_ADDR_FBC_Y_OFFSET4                   (W5_REG_BASE + 0x017C)
/* compression offset table for chroma */
#define W5_ADDR_FBC_C_OFFSET4                   (W5_REG_BASE + 0x0180)
#define W5_ADDR_LUMA_BASE5                      (W5_REG_BASE + 0x0184)
#define W5_ADDR_CB_ADDR5                        (W5_REG_BASE + 0x0188)
#define W5_ADDR_CR_ADDR5                        (W5_REG_BASE + 0x018C)
/* compression offset table for luma */
#define W5_ADDR_FBC_Y_OFFSET5                   (W5_REG_BASE + 0x018C)
/* compression offset table for chroma */
#define W5_ADDR_FBC_C_OFFSET5                   (W5_REG_BASE + 0x0190)
#define W5_ADDR_LUMA_BASE6                      (W5_REG_BASE + 0x0194)
#define W5_ADDR_CB_ADDR6                        (W5_REG_BASE + 0x0198)
#define W5_ADDR_CR_ADDR6                        (W5_REG_BASE + 0x019C)
/* compression offset table for luma */
#define W5_ADDR_FBC_Y_OFFSET6                   (W5_REG_BASE + 0x019C)
/* compression offset table for chroma */
#define W5_ADDR_FBC_C_OFFSET6                   (W5_REG_BASE + 0x01A0)
#define W5_ADDR_LUMA_BASE7                      (W5_REG_BASE + 0x01A4)
#define W5_ADDR_CB_ADDR7                        (W5_REG_BASE + 0x01A8)
#define W5_ADDR_CR_ADDR7                        (W5_REG_BASE + 0x01AC)
/* compression offset table for luma */
#define W5_ADDR_FBC_Y_OFFSET7                   (W5_REG_BASE + 0x01AC)
/* compression offset table for chroma */
#define W5_ADDR_FBC_C_OFFSET7                   (W5_REG_BASE + 0x01B0)
#define W5_ADDR_MV_COL0                         (W5_REG_BASE + 0x01B4)
#define W5_ADDR_MV_COL1                         (W5_REG_BASE + 0x01B8)
#define W5_ADDR_MV_COL2                         (W5_REG_BASE + 0x01BC)
#define W5_ADDR_MV_COL3                         (W5_REG_BASE + 0x01C0)
#define W5_ADDR_MV_COL4                         (W5_REG_BASE + 0x01C4)
#define W5_ADDR_MV_COL5                         (W5_REG_BASE + 0x01C8)
#define W5_ADDR_MV_COL6                         (W5_REG_BASE + 0x01CC)
#define W5_ADDR_MV_COL7                         (W5_REG_BASE + 0x01D0)

/* UPDATE_FB */
/* CMD_SET_FB_STRIDE [15:0]     - FBC framebuffer stride
 * [31:15]    - linear framebuffer stride
 */
#define W5_CMD_SET_FB_STRIDE                    (W5_REG_BASE + 0x0118)
#define W5_CMD_SET_FB_INDEX                     (W5_REG_BASE + 0x0120)
#define W5_ADDR_LUMA_BASE                       (W5_REG_BASE + 0x0134)
#define W5_ADDR_CB_BASE                         (W5_REG_BASE + 0x0138)
#define W5_ADDR_CR_BASE                         (W5_REG_BASE + 0x013C)
#define W5_ADDR_MV_COL                          (W5_REG_BASE + 0x0140)
#define W5_ADDR_FBC_Y_BASE                      (W5_REG_BASE + 0x0144)
#define W5_ADDR_FBC_C_BASE                      (W5_REG_BASE + 0x0148)
#define W5_ADDR_FBC_Y_OFFSET                    (W5_REG_BASE + 0x014C)
#define W5_ADDR_FBC_C_OFFSET                    (W5_REG_BASE + 0x0150)

/************************************************************************/
/* DECODER - DEC_PIC                                                    */
/************************************************************************/
#define W5_CMD_DEC_VCORE_INFO                   (W5_REG_BASE + 0x0194)
/* sequence change enable mask register
 * CMD_SEQ_CHANGE_ENABLE_FLAG [5]   profile_idc
 *                            [16]  pic_width/height_in_luma_sample
 *                            [19]  sps_max_dec_pic_buffering, max_num_reorder, max_latency_increase
 */
#define W5_CMD_SEQ_CHANGE_ENABLE_FLAG           (W5_REG_BASE + 0x0128)
#define W5_CMD_DEC_USER_MASK                    (W5_REG_BASE + 0x012C)
#define W5_CMD_DEC_TEMPORAL_ID_PLUS1            (W5_REG_BASE + 0x0130)
#define W5_CMD_DEC_FORCE_FB_LATENCY_PLUS1       (W5_REG_BASE + 0x0134)
#define W5_USE_SEC_AXI                          (W5_REG_BASE + 0x0150)

/************************************************************************/
/* DECODER - QUERY : GET_VPU_INFO                                       */
/************************************************************************/
#define W5_RET_FW_VERSION                       (W5_REG_BASE + 0x0118)
#define W5_RET_PRODUCT_NAME                     (W5_REG_BASE + 0x011C)
#define W5_RET_PRODUCT_VERSION                  (W5_REG_BASE + 0x0120)
#define W5_RET_STD_DEF0                         (W5_REG_BASE + 0x0124)
#define W5_RET_STD_DEF1                         (W5_REG_BASE + 0x0128)
#define W5_RET_CONF_FEATURE                     (W5_REG_BASE + 0x012C)
#define W5_RET_CONF_DATE                        (W5_REG_BASE + 0x0130)
#define W5_RET_CONF_REVISION                    (W5_REG_BASE + 0x0134)
#define W5_RET_CONF_TYPE                        (W5_REG_BASE + 0x0138)
#define W5_RET_PRODUCT_ID                       (W5_REG_BASE + 0x013C)
#define W5_RET_CUSTOMER_ID                      (W5_REG_BASE + 0x0140)

/************************************************************************/
/* DECODER - QUERY : GET_RESULT                                         */
/************************************************************************/
#define W5_CMD_DEC_ADDR_REPORT_BASE         (W5_REG_BASE + 0x0114)
#define W5_CMD_DEC_REPORT_SIZE              (W5_REG_BASE + 0x0118)
#define W5_CMD_DEC_REPORT_PARAM             (W5_REG_BASE + 0x011C)

#define W5_RET_DEC_BS_RD_PTR                (W5_REG_BASE + 0x011C)
#define W5_RET_DEC_SEQ_PARAM                (W5_REG_BASE + 0x0120)
#define W5_RET_DEC_COLOR_SAMPLE_INFO        (W5_REG_BASE + 0x0124)
#define W5_RET_DEC_ASPECT_RATIO             (W5_REG_BASE + 0x0128)
#define W5_RET_DEC_BIT_RATE                 (W5_REG_BASE + 0x012C)
#define W5_RET_DEC_FRAME_RATE_NR            (W5_REG_BASE + 0x0130)
#define W5_RET_DEC_FRAME_RATE_DR            (W5_REG_BASE + 0x0134)
#define W5_RET_DEC_NUM_REQUIRED_FB          (W5_REG_BASE + 0x0138)
#define W5_RET_DEC_NUM_REORDER_DELAY        (W5_REG_BASE + 0x013C)
#define W5_RET_DEC_SUB_LAYER_INFO           (W5_REG_BASE + 0x0140)
#define W5_RET_DEC_NOTIFICATION             (W5_REG_BASE + 0x0144)
/*
 * USER_DATA_FLAGS for HEVC/H264 only.
 * Bits:
 * [1] - User data buffer full boolean
 * [2] - VUI parameter flag
 * [4] - Pic_timing SEI flag
 * [5] - 1st user_data_registed_itu_t_t35 prefix SEI flag
 * [6] - user_data_unregistered prefix SEI flag
 * [7] - 1st user_data_registed_itu_t_t35 suffix SEI flag
 * [8] - user_data_unregistered suffix SEI flag
 * [10]- mastering_display_color_volume prefix SEI flag
 * [11]- chroma_resampling_display_color_volume prefix SEI flag
 * [12]- knee_function_info SEI flag
 * [13]- tone_mapping_info prefix SEI flag
 * [14]- film_grain_characteristics_info prefix SEI flag
 * [15]- content_light_level_info prefix SEI flag
 * [16]- color_remapping_info prefix SEI flag
 * [28]- 2nd user_data_registed_itu_t_t35 prefix SEI flag
 * [29]- 3rd user_data_registed_itu_t_t35 prefix SEI flag
 * [30]- 2nd user_data_registed_itu_t_t35 suffix SEI flag
 * [31]- 3rd user_data_registed_itu_t_t35 suffix SEI flag
 */
#define W5_RET_DEC_USERDATA_IDC             (W5_REG_BASE + 0x0148)
#define W5_RET_DEC_PIC_SIZE                 (W5_REG_BASE + 0x014C)
#define W5_RET_DEC_CROP_TOP_BOTTOM          (W5_REG_BASE + 0x0150)
#define W5_RET_DEC_CROP_LEFT_RIGHT          (W5_REG_BASE + 0x0154)
/*
 * #define W5_RET_DEC_AU_START_POS             (W5_REG_BASE + 0x0158)
 * => Access unit (AU) Bitstream start position
 * #define W5_RET_DEC_AU_END_POS               (W5_REG_BASE + 0x015C)
 * => Access unit (AU) Bitstream end position
 */

/*
 * Decoded picture type:
 * reg_val & 0x7			=> picture type
 * (reg_val >> 4) & 0x3f		=> VCL NAL unit type
 * (reg_val >> 31) & 0x1		=> output_flag
 * 16 << ((reg_val >> 10) & 0x3)	=> ctu_size
 */
#define W5_RET_DEC_PIC_TYPE                 (W5_REG_BASE + 0x0160)
#define W5_RET_DEC_PIC_POC                  (W5_REG_BASE + 0x0164)
/*
 * #define W5_RET_DEC_RECOVERY_POINT           (W5_REG_BASE + 0x0168)
 * => HEVC recovery point
 * reg_val & 0xff => number of signed recovery picture order counts
 * (reg_val >> 16) & 0x1 => exact match flag
 * (reg_val >> 17) & 0x1 => broken link flag
 * (reg_val >> 18) & 0x1 => exist flag
 */
#define W5_RET_DEC_DEBUG_INDEX              (W5_REG_BASE + 0x016C)
#define W5_RET_DEC_DECODED_INDEX            (W5_REG_BASE + 0x0170)
#define W5_RET_DEC_DISPLAY_INDEX            (W5_REG_BASE + 0x0174)
/*
 * #define W5_RET_DEC_REALLOC_INDEX            (W5_REG_BASE + 0x0178)
 * => display picture index in decoded picture buffer
 * reg_val & 0xf => display picture index for FBC buffer (by reordering)
 */
#define W5_RET_DEC_DISP_IDC                 (W5_REG_BASE + 0x017C)
/*
 * #define W5_RET_DEC_ERR_CTB_NUM              (W5_REG_BASE + 0x0180)
 * => Number of error CTUs
 * reg_val >> 16	=> erroneous CTUs in bitstream
 * reg_val & 0xffff	=> total CTUs in bitstream
 *
 * #define W5_RET_DEC_PIC_PARAM                (W5_REG_BASE + 0x01A0)
 * => Bitstream sequence/picture parameter information (AV1 only)
 * reg_val & 0x1 => intrabc tool enable
 * (reg_val >> 1) & 0x1 => screen content tools enable
 */
#define W5_RET_DEC_VUI_INFO                 (W5_REG_BASE + 0x01A4)
#define W5_RET_DEC_HOST_CMD_TICK            (W5_REG_BASE + 0x01B8)
#define W5_RET_DEC_SEEK_START_TICK          (W5_REG_BASE + 0x01BC)
#define W5_RET_DEC_SEEK_END_TICK            (W5_REG_BASE + 0x01C0)
//=> Start and end ticks for seeking slices of the picture
#define W5_RET_DEC_PARSING_START_TICK       (W5_REG_BASE + 0x01C4)
#define W5_RET_DEC_PARSING_END_TICK         (W5_REG_BASE + 0x01C8)
//=> Start and end ticks for parsing slices of the picture
#define W5_RET_DEC_DECODING_START_TICK      (W5_REG_BASE + 0x01CC)
//=> Start tick for decoding slices of the picture
#define W5_RET_DEC_DECODING_END_TICK        (W5_REG_BASE + 0x01D0)
#define W5_RET_DEC_WARN_INFO                (W5_REG_BASE + 0x01D4)
#define W5_RET_DEC_ERR_INFO                 (W5_REG_BASE + 0x01D8)
#define W5_RET_DEC_DECODING_SUCCESS         (W5_REG_BASE + 0x01DC)

/************************************************************************/
/* DECODER - FLUSH_INSTANCE                                             */
/************************************************************************/
#define W5_CMD_FLUSH_INST_OPT               (W5_REG_BASE + 0x104)

/************************************************************************/
/* DECODER - QUERY : UPDATE_DISP_FLAG                                   */
/************************************************************************/
#define W5_CMD_DEC_SET_DISP_IDC             (W5_REG_BASE + 0x0118)
#define W5_CMD_DEC_CLR_DISP_IDC             (W5_REG_BASE + 0x011C)

/************************************************************************/
/* DECODER - QUERY : SET_BS_RD_PTR                                      */
/************************************************************************/
#define W5_RET_QUERY_DEC_SET_BS_RD_PTR      (W5_REG_BASE + 0x011C)

/************************************************************************/
/* DECODER - QUERY : GET_BS_RD_PTR                                      */
/************************************************************************/
#define W5_RET_QUERY_DEC_BS_RD_PTR          (W5_REG_BASE + 0x011C)

/************************************************************************/
/* QUERY : GET_DEBUG_INFO                                               */
/************************************************************************/
#define W5_RET_QUERY_DEBUG_PRI_REASON       (W5_REG_BASE + 0x114)

/************************************************************************/
/* GDI register for debugging                                           */
/************************************************************************/
#define W5_GDI_BASE                         0x8800
#define W5_GDI_BUS_CTRL                     (W5_GDI_BASE + 0x0F0)
#define W5_GDI_BUS_STATUS                   (W5_GDI_BASE + 0x0F4)

#define W5_BACKBONE_BASE_VCPU               0xFE00
#define W5_BACKBONE_BUS_CTRL_VCPU           (W5_BACKBONE_BASE_VCPU + 0x010)
#define W5_BACKBONE_BUS_STATUS_VCPU         (W5_BACKBONE_BASE_VCPU + 0x014)
#define W5_BACKBONE_PROG_AXI_ID             (W5_BACKBONE_BASE_VCPU + 0x00C)

#define W5_BACKBONE_PROC_EXT_ADDR           (W5_BACKBONE_BASE_VCPU + 0x0C0)
#define W5_BACKBONE_AXI_PARAM               (W5_BACKBONE_BASE_VCPU + 0x0E0)

#define W5_BACKBONE_BASE_VCORE0             0x8E00
#define W5_BACKBONE_BUS_CTRL_VCORE0         (W5_BACKBONE_BASE_VCORE0 + 0x010)
#define W5_BACKBONE_BUS_STATUS_VCORE0       (W5_BACKBONE_BASE_VCORE0 + 0x014)

#define W5_BACKBONE_BASE_VCORE1             0x9E00  /* for dual-core product */
#define W5_BACKBONE_BUS_CTRL_VCORE1         (W5_BACKBONE_BASE_VCORE1 + 0x010)
#define W5_BACKBONE_BUS_STATUS_VCORE1       (W5_BACKBONE_BASE_VCORE1 + 0x014)

#define W5_COMBINED_BACKBONE_BASE           0xFE00
#define W5_COMBINED_BACKBONE_BUS_CTRL       (W5_COMBINED_BACKBONE_BASE + 0x010)
#define W5_COMBINED_BACKBONE_BUS_STATUS     (W5_COMBINED_BACKBONE_BASE + 0x014)

#endif /* __WAVE5_REGISTER_DEFINE_H__ */
