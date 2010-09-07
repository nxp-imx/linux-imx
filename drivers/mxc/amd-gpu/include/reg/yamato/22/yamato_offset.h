/* Copyright (c) 2002,2007-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _yamato_OFFSET_HEADER
#define _yamato_OFFSET_HEADER


// Registers from PA block

#define mmPA_CL_VPORT_XSCALE           0x210F
#define mmPA_CL_VPORT_XOFFSET          0x2110
#define mmPA_CL_VPORT_YSCALE           0x2111
#define mmPA_CL_VPORT_YOFFSET          0x2112
#define mmPA_CL_VPORT_ZSCALE           0x2113
#define mmPA_CL_VPORT_ZOFFSET          0x2114
#define mmPA_CL_VTE_CNTL               0x2206
#define mmPA_CL_CLIP_CNTL              0x2204
#define mmPA_CL_GB_VERT_CLIP_ADJ       0x2303
#define mmPA_CL_GB_VERT_DISC_ADJ       0x2304
#define mmPA_CL_GB_HORZ_CLIP_ADJ       0x2305
#define mmPA_CL_GB_HORZ_DISC_ADJ       0x2306
#define mmPA_CL_ENHANCE                0x0C85
#define mmPA_SC_ENHANCE                0x0CA5
#define mmPA_SU_VTX_CNTL               0x2302
#define mmPA_SU_POINT_SIZE             0x2280
#define mmPA_SU_POINT_MINMAX           0x2281
#define mmPA_SU_LINE_CNTL              0x2282
#define mmPA_SU_FACE_DATA              0x0C86
#define mmPA_SU_SC_MODE_CNTL           0x2205
#define mmPA_SU_POLY_OFFSET_FRONT_SCALE 0x2380
#define mmPA_SU_POLY_OFFSET_FRONT_OFFSET 0x2381
#define mmPA_SU_POLY_OFFSET_BACK_SCALE 0x2382
#define mmPA_SU_POLY_OFFSET_BACK_OFFSET 0x2383
#define mmPA_SU_PERFCOUNTER0_SELECT    0x0C88
#define mmPA_SU_PERFCOUNTER1_SELECT    0x0C89
#define mmPA_SU_PERFCOUNTER2_SELECT    0x0C8A
#define mmPA_SU_PERFCOUNTER3_SELECT    0x0C8B
#define mmPA_SU_PERFCOUNTER0_LOW       0x0C8C
#define mmPA_SU_PERFCOUNTER0_HI        0x0C8D
#define mmPA_SU_PERFCOUNTER1_LOW       0x0C8E
#define mmPA_SU_PERFCOUNTER1_HI        0x0C8F
#define mmPA_SU_PERFCOUNTER2_LOW       0x0C90
#define mmPA_SU_PERFCOUNTER2_HI        0x0C91
#define mmPA_SU_PERFCOUNTER3_LOW       0x0C92
#define mmPA_SU_PERFCOUNTER3_HI        0x0C93
#define mmPA_SC_WINDOW_OFFSET          0x2080
#define mmPA_SC_AA_CONFIG              0x2301
#define mmPA_SC_AA_MASK                0x2312
#define mmPA_SC_LINE_STIPPLE           0x2283
#define mmPA_SC_LINE_CNTL              0x2300
#define mmPA_SC_WINDOW_SCISSOR_TL      0x2081
#define mmPA_SC_WINDOW_SCISSOR_BR      0x2082
#define mmPA_SC_SCREEN_SCISSOR_TL      0x200E
#define mmPA_SC_SCREEN_SCISSOR_BR      0x200F
#define mmPA_SC_VIZ_QUERY              0x2293
#define mmPA_SC_VIZ_QUERY_STATUS       0x0C44
#define mmPA_SC_LINE_STIPPLE_STATE     0x0C40
#define mmPA_SC_PERFCOUNTER0_SELECT    0x0C98
#define mmPA_SC_PERFCOUNTER0_LOW       0x0C99
#define mmPA_SC_PERFCOUNTER0_HI        0x0C9A
#define mmPA_CL_CNTL_STATUS            0x0C84
#define mmPA_SU_CNTL_STATUS            0x0C94
#define mmPA_SC_CNTL_STATUS            0x0CA4
#define mmPA_SU_DEBUG_CNTL             0x0C80
#define mmPA_SU_DEBUG_DATA             0x0C81
#define mmPA_SC_DEBUG_CNTL             0x0C82
#define mmPA_SC_DEBUG_DATA             0x0C83


// Registers from VGT block

#define mmGFX_COPY_STATE               0x21F4
#define mmVGT_DRAW_INITIATOR           0x21FC
#define mmVGT_EVENT_INITIATOR          0x21F9
#define mmVGT_DMA_BASE                 0x21FA
#define mmVGT_DMA_SIZE                 0x21FB
#define mmVGT_BIN_BASE                 0x21FE
#define mmVGT_BIN_SIZE                 0x21FF
#define mmVGT_CURRENT_BIN_ID_MIN       0x2207
#define mmVGT_CURRENT_BIN_ID_MAX       0x2203
#define mmVGT_IMMED_DATA               0x21FD
#define mmVGT_MAX_VTX_INDX             0x2100
#define mmVGT_MIN_VTX_INDX             0x2101
#define mmVGT_INDX_OFFSET              0x2102
#define mmVGT_VERTEX_REUSE_BLOCK_CNTL  0x2316
#define mmVGT_OUT_DEALLOC_CNTL         0x2317
#define mmVGT_MULTI_PRIM_IB_RESET_INDX 0x2103
#define mmVGT_ENHANCE                  0x2294
#define mmVGT_VTX_VECT_EJECT_REG       0x0C2C
#define mmVGT_LAST_COPY_STATE          0x0C30
#define mmVGT_DEBUG_CNTL               0x0C38
#define mmVGT_DEBUG_DATA               0x0C39
#define mmVGT_CNTL_STATUS              0x0C3C
#define mmVGT_CRC_SQ_DATA              0x0C3A
#define mmVGT_CRC_SQ_CTRL              0x0C3B
#define mmVGT_PERFCOUNTER0_SELECT      0x0C48
#define mmVGT_PERFCOUNTER1_SELECT      0x0C49
#define mmVGT_PERFCOUNTER2_SELECT      0x0C4A
#define mmVGT_PERFCOUNTER3_SELECT      0x0C4B
#define mmVGT_PERFCOUNTER0_LOW         0x0C4C
#define mmVGT_PERFCOUNTER1_LOW         0x0C4E
#define mmVGT_PERFCOUNTER2_LOW         0x0C50
#define mmVGT_PERFCOUNTER3_LOW         0x0C52
#define mmVGT_PERFCOUNTER0_HI          0x0C4D
#define mmVGT_PERFCOUNTER1_HI          0x0C4F
#define mmVGT_PERFCOUNTER2_HI          0x0C51
#define mmVGT_PERFCOUNTER3_HI          0x0C53


// Registers from TP block

#define mmTC_CNTL_STATUS               0x0E00
#define mmTCR_CHICKEN                  0x0E02
#define mmTCF_CHICKEN                  0x0E03
#define mmTCM_CHICKEN                  0x0E04
#define mmTCR_PERFCOUNTER0_SELECT      0x0E05
#define mmTCR_PERFCOUNTER1_SELECT      0x0E08
#define mmTCR_PERFCOUNTER0_HI          0x0E06
#define mmTCR_PERFCOUNTER1_HI          0x0E09
#define mmTCR_PERFCOUNTER0_LOW         0x0E07
#define mmTCR_PERFCOUNTER1_LOW         0x0E0A
#define mmTP_TC_CLKGATE_CNTL           0x0E17
#define mmTPC_CNTL_STATUS              0x0E18
#define mmTPC_DEBUG0                   0x0E19
#define mmTPC_DEBUG1                   0x0E1A
#define mmTPC_CHICKEN                  0x0E1B
#define mmTP0_CNTL_STATUS              0x0E1C
#define mmTP0_DEBUG                    0x0E1D
#define mmTP0_CHICKEN                  0x0E1E
#define mmTP0_PERFCOUNTER0_SELECT      0x0E1F
#define mmTP0_PERFCOUNTER0_HI          0x0E20
#define mmTP0_PERFCOUNTER0_LOW         0x0E21
#define mmTP0_PERFCOUNTER1_SELECT      0x0E22
#define mmTP0_PERFCOUNTER1_HI          0x0E23
#define mmTP0_PERFCOUNTER1_LOW         0x0E24
#define mmTCM_PERFCOUNTER0_SELECT      0x0E54
#define mmTCM_PERFCOUNTER1_SELECT      0x0E57
#define mmTCM_PERFCOUNTER0_HI          0x0E55
#define mmTCM_PERFCOUNTER1_HI          0x0E58
#define mmTCM_PERFCOUNTER0_LOW         0x0E56
#define mmTCM_PERFCOUNTER1_LOW         0x0E59
#define mmTCF_PERFCOUNTER0_SELECT      0x0E5A
#define mmTCF_PERFCOUNTER1_SELECT      0x0E5D
#define mmTCF_PERFCOUNTER2_SELECT      0x0E60
#define mmTCF_PERFCOUNTER3_SELECT      0x0E63
#define mmTCF_PERFCOUNTER4_SELECT      0x0E66
#define mmTCF_PERFCOUNTER5_SELECT      0x0E69
#define mmTCF_PERFCOUNTER6_SELECT      0x0E6C
#define mmTCF_PERFCOUNTER7_SELECT      0x0E6F
#define mmTCF_PERFCOUNTER8_SELECT      0x0E72
#define mmTCF_PERFCOUNTER9_SELECT      0x0E75
#define mmTCF_PERFCOUNTER10_SELECT     0x0E78
#define mmTCF_PERFCOUNTER11_SELECT     0x0E7B
#define mmTCF_PERFCOUNTER0_HI          0x0E5B
#define mmTCF_PERFCOUNTER1_HI          0x0E5E
#define mmTCF_PERFCOUNTER2_HI          0x0E61
#define mmTCF_PERFCOUNTER3_HI          0x0E64
#define mmTCF_PERFCOUNTER4_HI          0x0E67
#define mmTCF_PERFCOUNTER5_HI          0x0E6A
#define mmTCF_PERFCOUNTER6_HI          0x0E6D
#define mmTCF_PERFCOUNTER7_HI          0x0E70
#define mmTCF_PERFCOUNTER8_HI          0x0E73
#define mmTCF_PERFCOUNTER9_HI          0x0E76
#define mmTCF_PERFCOUNTER10_HI         0x0E79
#define mmTCF_PERFCOUNTER11_HI         0x0E7C
#define mmTCF_PERFCOUNTER0_LOW         0x0E5C
#define mmTCF_PERFCOUNTER1_LOW         0x0E5F
#define mmTCF_PERFCOUNTER2_LOW         0x0E62
#define mmTCF_PERFCOUNTER3_LOW         0x0E65
#define mmTCF_PERFCOUNTER4_LOW         0x0E68
#define mmTCF_PERFCOUNTER5_LOW         0x0E6B
#define mmTCF_PERFCOUNTER6_LOW         0x0E6E
#define mmTCF_PERFCOUNTER7_LOW         0x0E71
#define mmTCF_PERFCOUNTER8_LOW         0x0E74
#define mmTCF_PERFCOUNTER9_LOW         0x0E77
#define mmTCF_PERFCOUNTER10_LOW        0x0E7A
#define mmTCF_PERFCOUNTER11_LOW        0x0E7D
#define mmTCF_DEBUG                    0x0EC0
#define mmTCA_FIFO_DEBUG               0x0EC1
#define mmTCA_PROBE_DEBUG              0x0EC2
#define mmTCA_TPC_DEBUG                0x0EC3
#define mmTCB_CORE_DEBUG               0x0EC4
#define mmTCB_TAG0_DEBUG               0x0EC5
#define mmTCB_TAG1_DEBUG               0x0EC6
#define mmTCB_TAG2_DEBUG               0x0EC7
#define mmTCB_TAG3_DEBUG               0x0EC8
#define mmTCB_FETCH_GEN_SECTOR_WALKER0_DEBUG 0x0EC9
#define mmTCB_FETCH_GEN_WALKER_DEBUG   0x0ECB
#define mmTCB_FETCH_GEN_PIPE0_DEBUG    0x0ECC
#define mmTCD_INPUT0_DEBUG             0x0ED0
#define mmTCD_DEGAMMA_DEBUG            0x0ED4
#define mmTCD_DXTMUX_SCTARB_DEBUG      0x0ED5
#define mmTCD_DXTC_ARB_DEBUG           0x0ED6
#define mmTCD_STALLS_DEBUG             0x0ED7
#define mmTCO_STALLS_DEBUG             0x0EE0
#define mmTCO_QUAD0_DEBUG0             0x0EE1
#define mmTCO_QUAD0_DEBUG1             0x0EE2


// Registers from TC block



// Registers from SQ block

#define mmSQ_GPR_MANAGEMENT            0x0D00
#define mmSQ_FLOW_CONTROL              0x0D01
#define mmSQ_INST_STORE_MANAGMENT      0x0D02
#define mmSQ_RESOURCE_MANAGMENT        0x0D03
#define mmSQ_EO_RT                     0x0D04
#define mmSQ_DEBUG_MISC                0x0D05
#define mmSQ_ACTIVITY_METER_CNTL       0x0D06
#define mmSQ_ACTIVITY_METER_STATUS     0x0D07
#define mmSQ_INPUT_ARB_PRIORITY        0x0D08
#define mmSQ_THREAD_ARB_PRIORITY       0x0D09
#define mmSQ_VS_WATCHDOG_TIMER         0x0D0A
#define mmSQ_PS_WATCHDOG_TIMER         0x0D0B
#define mmSQ_INT_CNTL                  0x0D34
#define mmSQ_INT_STATUS                0x0D35
#define mmSQ_INT_ACK                   0x0D36
#define mmSQ_DEBUG_INPUT_FSM           0x0DAE
#define mmSQ_DEBUG_CONST_MGR_FSM       0x0DAF
#define mmSQ_DEBUG_TP_FSM              0x0DB0
#define mmSQ_DEBUG_FSM_ALU_0           0x0DB1
#define mmSQ_DEBUG_FSM_ALU_1           0x0DB2
#define mmSQ_DEBUG_EXP_ALLOC           0x0DB3
#define mmSQ_DEBUG_PTR_BUFF            0x0DB4
#define mmSQ_DEBUG_GPR_VTX             0x0DB5
#define mmSQ_DEBUG_GPR_PIX             0x0DB6
#define mmSQ_DEBUG_TB_STATUS_SEL       0x0DB7
#define mmSQ_DEBUG_VTX_TB_0            0x0DB8
#define mmSQ_DEBUG_VTX_TB_1            0x0DB9
#define mmSQ_DEBUG_VTX_TB_STATUS_REG   0x0DBA
#define mmSQ_DEBUG_VTX_TB_STATE_MEM    0x0DBB
#define mmSQ_DEBUG_PIX_TB_0            0x0DBC
#define mmSQ_DEBUG_PIX_TB_STATUS_REG_0 0x0DBD
#define mmSQ_DEBUG_PIX_TB_STATUS_REG_1 0x0DBE
#define mmSQ_DEBUG_PIX_TB_STATUS_REG_2 0x0DBF
#define mmSQ_DEBUG_PIX_TB_STATUS_REG_3 0x0DC0
#define mmSQ_DEBUG_PIX_TB_STATE_MEM    0x0DC1
#define mmSQ_PERFCOUNTER0_SELECT       0x0DC8
#define mmSQ_PERFCOUNTER1_SELECT       0x0DC9
#define mmSQ_PERFCOUNTER2_SELECT       0x0DCA
#define mmSQ_PERFCOUNTER3_SELECT       0x0DCB
#define mmSQ_PERFCOUNTER0_LOW          0x0DCC
#define mmSQ_PERFCOUNTER0_HI           0x0DCD
#define mmSQ_PERFCOUNTER1_LOW          0x0DCE
#define mmSQ_PERFCOUNTER1_HI           0x0DCF
#define mmSQ_PERFCOUNTER2_LOW          0x0DD0
#define mmSQ_PERFCOUNTER2_HI           0x0DD1
#define mmSQ_PERFCOUNTER3_LOW          0x0DD2
#define mmSQ_PERFCOUNTER3_HI           0x0DD3
#define mmSX_PERFCOUNTER0_SELECT       0x0DD4
#define mmSX_PERFCOUNTER0_LOW          0x0DD8
#define mmSX_PERFCOUNTER0_HI           0x0DD9
#define mmSQ_INSTRUCTION_ALU_0         0x5000
#define mmSQ_INSTRUCTION_ALU_1         0x5001
#define mmSQ_INSTRUCTION_ALU_2         0x5002
#define mmSQ_INSTRUCTION_CF_EXEC_0     0x5080
#define mmSQ_INSTRUCTION_CF_EXEC_1     0x5081
#define mmSQ_INSTRUCTION_CF_EXEC_2     0x5082
#define mmSQ_INSTRUCTION_CF_LOOP_0     0x5083
#define mmSQ_INSTRUCTION_CF_LOOP_1     0x5084
#define mmSQ_INSTRUCTION_CF_LOOP_2     0x5085
#define mmSQ_INSTRUCTION_CF_JMP_CALL_0 0x5086
#define mmSQ_INSTRUCTION_CF_JMP_CALL_1 0x5087
#define mmSQ_INSTRUCTION_CF_JMP_CALL_2 0x5088
#define mmSQ_INSTRUCTION_CF_ALLOC_0    0x5089
#define mmSQ_INSTRUCTION_CF_ALLOC_1    0x508A
#define mmSQ_INSTRUCTION_CF_ALLOC_2    0x508B
#define mmSQ_INSTRUCTION_TFETCH_0      0x5043
#define mmSQ_INSTRUCTION_TFETCH_1      0x5044
#define mmSQ_INSTRUCTION_TFETCH_2      0x5045
#define mmSQ_INSTRUCTION_VFETCH_0      0x5040
#define mmSQ_INSTRUCTION_VFETCH_1      0x5041
#define mmSQ_INSTRUCTION_VFETCH_2      0x5042
#define mmSQ_CONSTANT_0                0x4000
#define mmSQ_CONSTANT_1                0x4001
#define mmSQ_CONSTANT_2                0x4002
#define mmSQ_CONSTANT_3                0x4003
#define mmSQ_FETCH_0                   0x4800
#define mmSQ_FETCH_1                   0x4801
#define mmSQ_FETCH_2                   0x4802
#define mmSQ_FETCH_3                   0x4803
#define mmSQ_FETCH_4                   0x4804
#define mmSQ_FETCH_5                   0x4805
#define mmSQ_CONSTANT_VFETCH_0         0x4806
#define mmSQ_CONSTANT_VFETCH_1         0x4808
#define mmSQ_CONSTANT_T2               0x480C
#define mmSQ_CONSTANT_T3               0x4812
#define mmSQ_CF_BOOLEANS               0x4900
#define mmSQ_CF_LOOP                   0x4908
#define mmSQ_CONSTANT_RT_0             0x4940
#define mmSQ_CONSTANT_RT_1             0x4941
#define mmSQ_CONSTANT_RT_2             0x4942
#define mmSQ_CONSTANT_RT_3             0x4943
#define mmSQ_FETCH_RT_0                0x4D40
#define mmSQ_FETCH_RT_1                0x4D41
#define mmSQ_FETCH_RT_2                0x4D42
#define mmSQ_FETCH_RT_3                0x4D43
#define mmSQ_FETCH_RT_4                0x4D44
#define mmSQ_FETCH_RT_5                0x4D45
#define mmSQ_CF_RT_BOOLEANS            0x4E00
#define mmSQ_CF_RT_LOOP                0x4E14
#define mmSQ_VS_PROGRAM                0x21F7
#define mmSQ_PS_PROGRAM                0x21F6
#define mmSQ_CF_PROGRAM_SIZE           0x2315
#define mmSQ_INTERPOLATOR_CNTL         0x2182
#define mmSQ_PROGRAM_CNTL              0x2180
#define mmSQ_WRAPPING_0                0x2183
#define mmSQ_WRAPPING_1                0x2184
#define mmSQ_VS_CONST                  0x2307
#define mmSQ_PS_CONST                  0x2308
#define mmSQ_CONTEXT_MISC              0x2181
#define mmSQ_CF_RD_BASE                0x21F5
#define mmSQ_DEBUG_MISC_0              0x2309
#define mmSQ_DEBUG_MISC_1              0x230A


// Registers from SX block



// Registers from MH block

#define mmMH_ARBITER_CONFIG            0x0A40
#define mmMH_CLNT_AXI_ID_REUSE         0x0A41
#define mmMH_INTERRUPT_MASK            0x0A42
#define mmMH_INTERRUPT_STATUS          0x0A43
#define mmMH_INTERRUPT_CLEAR           0x0A44
#define mmMH_AXI_ERROR                 0x0A45
#define mmMH_PERFCOUNTER0_SELECT       0x0A46
#define mmMH_PERFCOUNTER1_SELECT       0x0A4A
#define mmMH_PERFCOUNTER0_CONFIG       0x0A47
#define mmMH_PERFCOUNTER1_CONFIG       0x0A4B
#define mmMH_PERFCOUNTER0_LOW          0x0A48
#define mmMH_PERFCOUNTER1_LOW          0x0A4C
#define mmMH_PERFCOUNTER0_HI           0x0A49
#define mmMH_PERFCOUNTER1_HI           0x0A4D
#define mmMH_DEBUG_CTRL                0x0A4E
#define mmMH_DEBUG_DATA                0x0A4F
#define mmMH_AXI_HALT_CONTROL          0x0A50
#define mmMH_MMU_CONFIG                0x0040
#define mmMH_MMU_VA_RANGE              0x0041
#define mmMH_MMU_PT_BASE               0x0042
#define mmMH_MMU_PAGE_FAULT            0x0043
#define mmMH_MMU_TRAN_ERROR            0x0044
#define mmMH_MMU_INVALIDATE            0x0045
#define mmMH_MMU_MPU_BASE              0x0046
#define mmMH_MMU_MPU_END               0x0047


// Registers from RBBM block

#define mmWAIT_UNTIL                   0x05C8
#define mmRBBM_ISYNC_CNTL              0x05C9
#define mmRBBM_STATUS                  0x05D0
#define mmRBBM_DSPLY                   0x0391
#define mmRBBM_RENDER_LATEST           0x0392
#define mmRBBM_RTL_RELEASE             0x0000
#define mmRBBM_PATCH_RELEASE           0x0001
#define mmRBBM_AUXILIARY_CONFIG        0x0002
#define mmRBBM_PERIPHID0               0x03F8
#define mmRBBM_PERIPHID1               0x03F9
#define mmRBBM_PERIPHID2               0x03FA
#define mmRBBM_PERIPHID3               0x03FB
#define mmRBBM_CNTL                    0x003B
#define mmRBBM_SKEW_CNTL               0x003D
#define mmRBBM_SOFT_RESET              0x003C
#define mmRBBM_PM_OVERRIDE1            0x039C
#define mmRBBM_PM_OVERRIDE2            0x039D
#define mmGC_SYS_IDLE                  0x039E
#define mmNQWAIT_UNTIL                 0x0394
#define mmRBBM_DEBUG_OUT               0x03A0
#define mmRBBM_DEBUG_CNTL              0x03A1
#define mmRBBM_DEBUG                   0x039B
#define mmRBBM_READ_ERROR              0x03B3
#define mmRBBM_WAIT_IDLE_CLOCKS        0x03B2
#define mmRBBM_INT_CNTL                0x03B4
#define mmRBBM_INT_STATUS              0x03B5
#define mmRBBM_INT_ACK                 0x03B6
#define mmMASTER_INT_SIGNAL            0x03B7
#define mmRBBM_PERFCOUNTER1_SELECT     0x0395
#define mmRBBM_PERFCOUNTER1_LO         0x0397
#define mmRBBM_PERFCOUNTER1_HI         0x0398


// Registers from CP block

#define mmCP_RB_BASE                   0x01C0
#define mmCP_RB_CNTL                   0x01C1
#define mmCP_RB_RPTR_ADDR              0x01C3
#define mmCP_RB_RPTR                   0x01C4
#define mmCP_RB_RPTR_WR                0x01C7
#define mmCP_RB_WPTR                   0x01C5
#define mmCP_RB_WPTR_DELAY             0x01C6
#define mmCP_RB_WPTR_BASE              0x01C8
#define mmCP_IB1_BASE                  0x0458
#define mmCP_IB1_BUFSZ                 0x0459
#define mmCP_IB2_BASE                  0x045A
#define mmCP_IB2_BUFSZ                 0x045B
#define mmCP_ST_BASE                   0x044D
#define mmCP_ST_BUFSZ                  0x044E
#define mmCP_QUEUE_THRESHOLDS          0x01D5
#define mmCP_MEQ_THRESHOLDS            0x01D6
#define mmCP_CSQ_AVAIL                 0x01D7
#define mmCP_STQ_AVAIL                 0x01D8
#define mmCP_MEQ_AVAIL                 0x01D9
#define mmCP_CSQ_RB_STAT               0x01FD
#define mmCP_CSQ_IB1_STAT              0x01FE
#define mmCP_CSQ_IB2_STAT              0x01FF
#define mmCP_NON_PREFETCH_CNTRS        0x0440
#define mmCP_STQ_ST_STAT               0x0443
#define mmCP_MEQ_STAT                  0x044F
#define mmCP_MIU_TAG_STAT              0x0452
#define mmCP_CMD_INDEX                 0x01DA
#define mmCP_CMD_DATA                  0x01DB
#define mmCP_ME_CNTL                   0x01F6
#define mmCP_ME_STATUS                 0x01F7
#define mmCP_ME_RAM_WADDR              0x01F8
#define mmCP_ME_RAM_RADDR              0x01F9
#define mmCP_ME_RAM_DATA               0x01FA
#define mmCP_ME_RDADDR                 0x01EA
#define mmCP_DEBUG                     0x01FC
#define mmSCRATCH_REG0                 0x0578
#define mmGUI_SCRATCH_REG0             0x0578
#define mmSCRATCH_REG1                 0x0579
#define mmGUI_SCRATCH_REG1             0x0579
#define mmSCRATCH_REG2                 0x057A
#define mmGUI_SCRATCH_REG2             0x057A
#define mmSCRATCH_REG3                 0x057B
#define mmGUI_SCRATCH_REG3             0x057B
#define mmSCRATCH_REG4                 0x057C
#define mmGUI_SCRATCH_REG4             0x057C
#define mmSCRATCH_REG5                 0x057D
#define mmGUI_SCRATCH_REG5             0x057D
#define mmSCRATCH_REG6                 0x057E
#define mmGUI_SCRATCH_REG6             0x057E
#define mmSCRATCH_REG7                 0x057F
#define mmGUI_SCRATCH_REG7             0x057F
#define mmSCRATCH_UMSK                 0x01DC
#define mmSCRATCH_ADDR                 0x01DD
#define mmCP_ME_VS_EVENT_SRC           0x0600
#define mmCP_ME_VS_EVENT_ADDR          0x0601
#define mmCP_ME_VS_EVENT_DATA          0x0602
#define mmCP_ME_VS_EVENT_ADDR_SWM      0x0603
#define mmCP_ME_VS_EVENT_DATA_SWM      0x0604
#define mmCP_ME_PS_EVENT_SRC           0x0605
#define mmCP_ME_PS_EVENT_ADDR          0x0606
#define mmCP_ME_PS_EVENT_DATA          0x0607
#define mmCP_ME_PS_EVENT_ADDR_SWM      0x0608
#define mmCP_ME_PS_EVENT_DATA_SWM      0x0609
#define mmCP_ME_CF_EVENT_SRC           0x060A
#define mmCP_ME_CF_EVENT_ADDR          0x060B
#define mmCP_ME_CF_EVENT_DATA          0x060C
#define mmCP_ME_NRT_ADDR               0x060D
#define mmCP_ME_NRT_DATA               0x060E
#define mmCP_ME_VS_FETCH_DONE_SRC      0x0612
#define mmCP_ME_VS_FETCH_DONE_ADDR     0x0613
#define mmCP_ME_VS_FETCH_DONE_DATA     0x0614
#define mmCP_INT_CNTL                  0x01F2
#define mmCP_INT_STATUS                0x01F3
#define mmCP_INT_ACK                   0x01F4
#define mmCP_PFP_UCODE_ADDR            0x00C0
#define mmCP_PFP_UCODE_DATA            0x00C1
#define mmCP_PERFMON_CNTL              0x0444
#define mmCP_PERFCOUNTER_SELECT        0x0445
#define mmCP_PERFCOUNTER_LO            0x0446
#define mmCP_PERFCOUNTER_HI            0x0447
#define mmCP_BIN_MASK_LO               0x0454
#define mmCP_BIN_MASK_HI               0x0455
#define mmCP_BIN_SELECT_LO             0x0456
#define mmCP_BIN_SELECT_HI             0x0457
#define mmCP_NV_FLAGS_0                0x01EE
#define mmCP_NV_FLAGS_1                0x01EF
#define mmCP_NV_FLAGS_2                0x01F0
#define mmCP_NV_FLAGS_3                0x01F1
#define mmCP_STATE_DEBUG_INDEX         0x01EC
#define mmCP_STATE_DEBUG_DATA          0x01ED
#define mmCP_PROG_COUNTER              0x044B
#define mmCP_STAT                      0x047F
#define mmBIOS_0_SCRATCH               0x0004
#define mmBIOS_1_SCRATCH               0x0005
#define mmBIOS_2_SCRATCH               0x0006
#define mmBIOS_3_SCRATCH               0x0007
#define mmBIOS_4_SCRATCH               0x0008
#define mmBIOS_5_SCRATCH               0x0009
#define mmBIOS_6_SCRATCH               0x000A
#define mmBIOS_7_SCRATCH               0x000B
#define mmBIOS_8_SCRATCH               0x0580
#define mmBIOS_9_SCRATCH               0x0581
#define mmBIOS_10_SCRATCH              0x0582
#define mmBIOS_11_SCRATCH              0x0583
#define mmBIOS_12_SCRATCH              0x0584
#define mmBIOS_13_SCRATCH              0x0585
#define mmBIOS_14_SCRATCH              0x0586
#define mmBIOS_15_SCRATCH              0x0587
#define mmCOHER_SIZE_PM4               0x0A29
#define mmCOHER_BASE_PM4               0x0A2A
#define mmCOHER_STATUS_PM4             0x0A2B
#define mmCOHER_SIZE_HOST              0x0A2F
#define mmCOHER_BASE_HOST              0x0A30
#define mmCOHER_STATUS_HOST            0x0A31
#define mmCOHER_DEST_BASE_0            0x2006
#define mmCOHER_DEST_BASE_1            0x2007
#define mmCOHER_DEST_BASE_2            0x2008
#define mmCOHER_DEST_BASE_3            0x2009
#define mmCOHER_DEST_BASE_4            0x200A
#define mmCOHER_DEST_BASE_5            0x200B
#define mmCOHER_DEST_BASE_6            0x200C
#define mmCOHER_DEST_BASE_7            0x200D


// Registers from SC block



// Registers from BC block

#define mmRB_SURFACE_INFO              0x2000
#define mmRB_COLOR_INFO                0x2001
#define mmRB_DEPTH_INFO                0x2002
#define mmRB_STENCILREFMASK            0x210D
#define mmRB_ALPHA_REF                 0x210E
#define mmRB_COLOR_MASK                0x2104
#define mmRB_BLEND_RED                 0x2105
#define mmRB_BLEND_GREEN               0x2106
#define mmRB_BLEND_BLUE                0x2107
#define mmRB_BLEND_ALPHA               0x2108
#define mmRB_FOG_COLOR                 0x2109
#define mmRB_STENCILREFMASK_BF         0x210C
#define mmRB_DEPTHCONTROL              0x2200
#define mmRB_BLENDCONTROL              0x2201
#define mmRB_COLORCONTROL              0x2202
#define mmRB_MODECONTROL               0x2208
#define mmRB_COLOR_DEST_MASK           0x2326
#define mmRB_COPY_CONTROL              0x2318
#define mmRB_COPY_DEST_BASE            0x2319
#define mmRB_COPY_DEST_PITCH           0x231A
#define mmRB_COPY_DEST_INFO            0x231B
#define mmRB_COPY_DEST_PIXEL_OFFSET    0x231C
#define mmRB_DEPTH_CLEAR               0x231D
#define mmRB_SAMPLE_COUNT_CTL          0x2324
#define mmRB_SAMPLE_COUNT_ADDR         0x2325
#define mmRB_BC_CONTROL                0x0F01
#define mmRB_EDRAM_INFO                0x0F02
#define mmRB_CRC_RD_PORT               0x0F0C
#define mmRB_CRC_CONTROL               0x0F0D
#define mmRB_CRC_MASK                  0x0F0E
#define mmRB_PERFCOUNTER0_SELECT       0x0F04
#define mmRB_PERFCOUNTER0_LOW          0x0F08
#define mmRB_PERFCOUNTER0_HI           0x0F09
#define mmRB_TOTAL_SAMPLES             0x0F0F
#define mmRB_ZPASS_SAMPLES             0x0F10
#define mmRB_ZFAIL_SAMPLES             0x0F11
#define mmRB_SFAIL_SAMPLES             0x0F12
#define mmRB_DEBUG_0                   0x0F26
#define mmRB_DEBUG_1                   0x0F27
#define mmRB_DEBUG_2                   0x0F28
#define mmRB_DEBUG_3                   0x0F29
#define mmRB_DEBUG_4                   0x0F2A
#define mmRB_FLAG_CONTROL              0x0F2B
#define mmRB_BC_SPARES                 0x0F2C
#define mmBC_DUMMY_CRAYRB_ENUMS        0x0F15
#define mmBC_DUMMY_CRAYRB_MOREENUMS    0x0F16
#endif
