/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Advanced Micro Devices nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

#ifndef __GSL__CONFIG_H
#define __GSL__CONFIG_H

/* ------------------------
 * Yamato ringbuffer config
 * ------------------------ */
static const unsigned int gsl_cfg_rb_sizelog2quadwords = GSL_RB_SIZE_32K;
static const unsigned int gsl_cfg_rb_blksizequadwords  = GSL_RB_SIZE_16;

/* ------------------------
 * Yamato MH arbiter config
 * ------------------------ */
static const mh_arbiter_config_t  gsl_cfg_yamato_mharb = {
    0x10,   /* same_page_limit */
    0,      /* same_page_granularity  */
    1,      /* l1_arb_enable */
    1,      /* l1_arb_hold_enable */
    0,      /* l2_arb_control */
    1,      /* page_size */
    1,      /* tc_reorder_enable */
    1,      /* tc_arb_hold_enable */
    1,      /* in_flight_limit_enable */
    0x8,    /* in_flight_limit */
    1,      /* cp_clnt_enable */
    1,      /* vgt_clnt_enable */
    1,      /* tc_clnt_enable */
    1,      /* rb_clnt_enable */
    1,      /* pa_clnt_enable */
};

/* ---------------------
 * G12 MH arbiter config
 * --------------------- */
static const REG_MH_ARBITER_CONFIG gsl_cfg_g12_mharb = {
    0x10,   /* SAME_PAGE_LIMIT */
    0,      /* SAME_PAGE_GRANULARITY */
    1,      /* L1_ARB_ENABLE */
    1,      /* L1_ARB_HOLD_ENABLE */
    0,      /* L2_ARB_CONTROL */
    1,      /* PAGE_SIZE */
    1,      /* TC_REORDER_ENABLE */
    1,      /* TC_ARB_HOLD_ENABLE */
    0,      /* IN_FLIGHT_LIMIT_ENABLE */
    0x8,    /* IN_FLIGHT_LIMIT */
    1,      /* CP_CLNT_ENABLE */
    1,      /* VGT_CLNT_ENABLE */
    1,      /* TC_CLNT_ENABLE */
    1,      /* RB_CLNT_ENABLE */
    1,      /* PA_CLNT_ENABLE */
};

/* -----------------------------
 * interrupt block register data
 * ----------------------------- */
static const gsl_intrblock_reg_t gsl_cfg_intrblock_reg[GSL_INTR_BLOCK_COUNT] = {
    {   /* Yamato MH */
	GSL_INTR_BLOCK_YDX_MH,
	GSL_INTR_YDX_MH_AXI_READ_ERROR,
	GSL_INTR_YDX_MH_MMU_PAGE_FAULT,
	mmMH_INTERRUPT_STATUS,
	mmMH_INTERRUPT_CLEAR,
	mmMH_INTERRUPT_MASK
    },
    {   /* Yamato CP */
	GSL_INTR_BLOCK_YDX_CP,
	GSL_INTR_YDX_CP_SW_INT,
	GSL_INTR_YDX_CP_RING_BUFFER,
	mmCP_INT_STATUS,
	mmCP_INT_ACK,
	mmCP_INT_CNTL
    },
    {   /* Yamato RBBM */
	GSL_INTR_BLOCK_YDX_RBBM,
	GSL_INTR_YDX_RBBM_READ_ERROR,
	GSL_INTR_YDX_RBBM_GUI_IDLE,
	mmRBBM_INT_STATUS,
	mmRBBM_INT_ACK,
	mmRBBM_INT_CNTL
    },
    {   /* Yamato SQ */
	GSL_INTR_BLOCK_YDX_SQ,
	GSL_INTR_YDX_SQ_PS_WATCHDOG,
	GSL_INTR_YDX_SQ_VS_WATCHDOG,
	mmSQ_INT_STATUS,
	mmSQ_INT_ACK,
	mmSQ_INT_CNTL
    },
    {   /* G12 */
	GSL_INTR_BLOCK_G12,
	GSL_INTR_G12_MH,
#ifndef _Z180
	GSL_INTR_G12_FBC,
#else
	GSL_INTR_G12_FIFO,
#endif /* _Z180 */
	(ADDR_VGC_IRQSTATUS >> 2),
	(ADDR_VGC_IRQSTATUS >> 2),
	(ADDR_VGC_IRQENABLE >> 2)
    },
    {   /* G12 MH */
	GSL_INTR_BLOCK_G12_MH,
	GSL_INTR_G12_MH_AXI_READ_ERROR,
	GSL_INTR_G12_MH_MMU_PAGE_FAULT,
	ADDR_MH_INTERRUPT_STATUS,       /* G12 MH offsets are considered to be dword based, therefore no down shift */
	ADDR_MH_INTERRUPT_CLEAR,
	ADDR_MH_INTERRUPT_MASK
    },
};

/* -----------------------
 * interrupt mask bit data
 * ----------------------- */
static const int gsl_cfg_intr_mask[GSL_INTR_COUNT] = {
    MH_INTERRUPT_MASK__AXI_READ_ERROR,
    MH_INTERRUPT_MASK__AXI_WRITE_ERROR,
    MH_INTERRUPT_MASK__MMU_PAGE_FAULT,

    CP_INT_CNTL__SW_INT_MASK,
    CP_INT_CNTL__T0_PACKET_IN_IB_MASK,
    CP_INT_CNTL__OPCODE_ERROR_MASK,
    CP_INT_CNTL__PROTECTED_MODE_ERROR_MASK,
    CP_INT_CNTL__RESERVED_BIT_ERROR_MASK,
    CP_INT_CNTL__IB_ERROR_MASK,
    CP_INT_CNTL__IB2_INT_MASK,
    CP_INT_CNTL__IB1_INT_MASK,
    CP_INT_CNTL__RB_INT_MASK,

    RBBM_INT_CNTL__RDERR_INT_MASK,
    RBBM_INT_CNTL__DISPLAY_UPDATE_INT_MASK,
    RBBM_INT_CNTL__GUI_IDLE_INT_MASK,

    SQ_INT_CNTL__PS_WATCHDOG_MASK,
    SQ_INT_CNTL__VS_WATCHDOG_MASK,

    (1 << VGC_IRQENABLE_MH_FSHIFT),
    (1 << VGC_IRQENABLE_G2D_FSHIFT),
    (1 << VGC_IRQENABLE_FIFO_FSHIFT),
#ifndef _Z180
    (1 << VGC_IRQENABLE_FBC_FSHIFT),
#endif
    (1 << MH_INTERRUPT_MASK_AXI_READ_ERROR_FSHIFT),
    (1 << MH_INTERRUPT_MASK_AXI_WRITE_ERROR_FSHIFT),
    (1 << MH_INTERRUPT_MASK_MMU_PAGE_FAULT_FSHIFT),
};

/* -----------------
 * mmu register data
 * ----------------- */
static const gsl_mmu_reg_t gsl_cfg_mmu_reg[GSL_DEVICE_MAX] = {
    {   /* Yamato */
	mmMH_MMU_CONFIG,
	mmMH_MMU_MPU_BASE,
	mmMH_MMU_MPU_END,
	mmMH_MMU_VA_RANGE,
	mmMH_MMU_PT_BASE,
	mmMH_MMU_PAGE_FAULT,
	mmMH_MMU_TRAN_ERROR,
	mmMH_MMU_INVALIDATE,
    },
    {   /* G12 - MH offsets are considered to be dword based, therefore no down shift */
	ADDR_MH_MMU_CONFIG,
	ADDR_MH_MMU_MPU_BASE,
	ADDR_MH_MMU_MPU_END,
	ADDR_MH_MMU_VA_RANGE,
	ADDR_MH_MMU_PT_BASE,
	ADDR_MH_MMU_PAGE_FAULT,
	ADDR_MH_MMU_TRAN_ERROR,
	ADDR_MH_MMU_INVALIDATE,
    },
};

/* -----------------
 * mh interrupt data
 * ----------------- */
static const gsl_mh_intr_t gsl_cfg_mh_intr[GSL_DEVICE_MAX] =
{
    {   /* Yamato */
	GSL_INTR_YDX_MH_AXI_READ_ERROR,
	GSL_INTR_YDX_MH_AXI_WRITE_ERROR,
	GSL_INTR_YDX_MH_MMU_PAGE_FAULT,
    },
    {   /* G12 */
	GSL_INTR_G12_MH_AXI_READ_ERROR,
	GSL_INTR_G12_MH_AXI_WRITE_ERROR,
	GSL_INTR_G12_MH_MMU_PAGE_FAULT,
    }
};

#endif /* __GSL__CONFIG_H */
