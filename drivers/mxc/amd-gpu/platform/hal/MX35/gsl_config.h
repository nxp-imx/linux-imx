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

#ifndef __GSL__CONFIG_H
#define __GSL__CONFIG_H


// ---------------------
// G12 MH arbiter config
// ---------------------
static const REG_MH_ARBITER_CONFIG gsl_cfg_g12_mharb = 
{
    0x10,   // SAME_PAGE_LIMIT
    0,      // SAME_PAGE_GRANULARITY 
    1,      // L1_ARB_ENABLE
    1,      // L1_ARB_HOLD_ENABLE
    0,      // L2_ARB_CONTROL
    1,      // PAGE_SIZE
    1,      // TC_REORDER_ENABLE
    1,      // TC_ARB_HOLD_ENABLE
    0,      // IN_FLIGHT_LIMIT_ENABLE
    0x8,    // IN_FLIGHT_LIMIT
    1,      // CP_CLNT_ENABLE
    1,      // VGT_CLNT_ENABLE
    1,      // TC_CLNT_ENABLE
    1,      // RB_CLNT_ENABLE
    1,      // PA_CLNT_ENABLE
};

// -----------------------------
// interrupt block register data
// -----------------------------
static const gsl_intrblock_reg_t gsl_cfg_intrblock_reg[GSL_INTR_BLOCK_COUNT] =
{
    {   // Yamato MH
        0,
        0,
        0,
        0,
        0,
        0
    },
    {   // Yamato CP
        0,
        0,
        0,
        0,
        0,
        0
    },
    {   // Yamato RBBM
        0,
        0,
        0,
        0,
        0,
        0
    },
    {   // Yamato SQ
        0,
        0,
        0,
        0,
        0,
        0
    },
    {   // G12
        GSL_INTR_BLOCK_G12,
        GSL_INTR_G12_MH,
#ifndef _Z180
        GSL_INTR_G12_FBC,
#else
        GSL_INTR_G12_FIFO,
#endif //_Z180
        (ADDR_VGC_IRQSTATUS >> 2),
        (ADDR_VGC_IRQSTATUS >> 2),
        (ADDR_VGC_IRQENABLE >> 2)
    },
    {   // G12 MH
        GSL_INTR_BLOCK_G12_MH,
        GSL_INTR_G12_MH_AXI_READ_ERROR,
        GSL_INTR_G12_MH_MMU_PAGE_FAULT,
        ADDR_MH_INTERRUPT_STATUS,       // G12 MH offsets are considered to be dword based, therefore no down shift
        ADDR_MH_INTERRUPT_CLEAR,
        ADDR_MH_INTERRUPT_MASK
    },
};

// -----------------------
// interrupt mask bit data
// -----------------------
static const int gsl_cfg_intr_mask[GSL_INTR_COUNT] =
{
    0,
    0,
    0,

    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,

    0,
    0,
    0,

    0,
    0,

    (1 << VGC_IRQENABLE_MH_FSHIFT),
    (1 << VGC_IRQENABLE_G2D_FSHIFT),
    (1 << VGC_IRQENABLE_FIFO_FSHIFT),
#ifndef _Z180
    (1 << VGC_IRQENABLE_FBC_FSHIFT),
#endif
    0,
    0,
    0,
};

// -----------------
// mmu register data
// -----------------
static const gsl_mmu_reg_t gsl_cfg_mmu_reg[GSL_DEVICE_MAX] =
{
    {   // Yamato
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    },
    {   // G12 - MH offsets are considered to be dword based, therefore no down shift
        ADDR_MH_MMU_CONFIG,
        ADDR_MH_MMU_MPU_BASE,
        ADDR_MH_MMU_MPU_END,
        ADDR_MH_MMU_VA_RANGE,
        ADDR_MH_MMU_PT_BASE,
        ADDR_MH_MMU_PAGE_FAULT,
        ADDR_MH_MMU_TRAN_ERROR,
        ADDR_MH_MMU_INVALIDATE,
    }
};

// -----------------
// mh interrupt data
// -----------------
static const gsl_mh_intr_t gsl_cfg_mh_intr[GSL_DEVICE_MAX] =
{
    {   // Yamato
        0,
        0,
        0,
    },
    {   // G12
        GSL_INTR_G12_MH_AXI_READ_ERROR,
        GSL_INTR_G12_MH_AXI_WRITE_ERROR,
        GSL_INTR_G12_MH_MMU_PAGE_FAULT,
    }
};

#endif // __GSL__CONFIG_H
