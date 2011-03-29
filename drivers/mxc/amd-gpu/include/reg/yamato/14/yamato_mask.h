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

#if !defined (_yamato_MASK_HEADER)
#define _yamato_MASK_HEADER

// PA_CL_VPORT_XSCALE
#define PA_CL_VPORT_XSCALE__VPORT_XSCALE_MASK              0xffffffffL

// PA_CL_VPORT_XOFFSET
#define PA_CL_VPORT_XOFFSET__VPORT_XOFFSET_MASK            0xffffffffL

// PA_CL_VPORT_YSCALE
#define PA_CL_VPORT_YSCALE__VPORT_YSCALE_MASK              0xffffffffL

// PA_CL_VPORT_YOFFSET
#define PA_CL_VPORT_YOFFSET__VPORT_YOFFSET_MASK            0xffffffffL

// PA_CL_VPORT_ZSCALE
#define PA_CL_VPORT_ZSCALE__VPORT_ZSCALE_MASK              0xffffffffL

// PA_CL_VPORT_ZOFFSET
#define PA_CL_VPORT_ZOFFSET__VPORT_ZOFFSET_MASK            0xffffffffL

// PA_CL_VTE_CNTL
#define PA_CL_VTE_CNTL__VPORT_X_SCALE_ENA_MASK             0x00000001L
#define PA_CL_VTE_CNTL__VPORT_X_SCALE_ENA                  0x00000001L
#define PA_CL_VTE_CNTL__VPORT_X_OFFSET_ENA_MASK            0x00000002L
#define PA_CL_VTE_CNTL__VPORT_X_OFFSET_ENA                 0x00000002L
#define PA_CL_VTE_CNTL__VPORT_Y_SCALE_ENA_MASK             0x00000004L
#define PA_CL_VTE_CNTL__VPORT_Y_SCALE_ENA                  0x00000004L
#define PA_CL_VTE_CNTL__VPORT_Y_OFFSET_ENA_MASK            0x00000008L
#define PA_CL_VTE_CNTL__VPORT_Y_OFFSET_ENA                 0x00000008L
#define PA_CL_VTE_CNTL__VPORT_Z_SCALE_ENA_MASK             0x00000010L
#define PA_CL_VTE_CNTL__VPORT_Z_SCALE_ENA                  0x00000010L
#define PA_CL_VTE_CNTL__VPORT_Z_OFFSET_ENA_MASK            0x00000020L
#define PA_CL_VTE_CNTL__VPORT_Z_OFFSET_ENA                 0x00000020L
#define PA_CL_VTE_CNTL__VTX_XY_FMT_MASK                    0x00000100L
#define PA_CL_VTE_CNTL__VTX_XY_FMT                         0x00000100L
#define PA_CL_VTE_CNTL__VTX_Z_FMT_MASK                     0x00000200L
#define PA_CL_VTE_CNTL__VTX_Z_FMT                          0x00000200L
#define PA_CL_VTE_CNTL__VTX_W0_FMT_MASK                    0x00000400L
#define PA_CL_VTE_CNTL__VTX_W0_FMT                         0x00000400L
#define PA_CL_VTE_CNTL__PERFCOUNTER_REF_MASK               0x00000800L
#define PA_CL_VTE_CNTL__PERFCOUNTER_REF                    0x00000800L

// PA_CL_CLIP_CNTL
#define PA_CL_CLIP_CNTL__CLIP_DISABLE_MASK                 0x00010000L
#define PA_CL_CLIP_CNTL__CLIP_DISABLE                      0x00010000L
#define PA_CL_CLIP_CNTL__BOUNDARY_EDGE_FLAG_ENA_MASK       0x00040000L
#define PA_CL_CLIP_CNTL__BOUNDARY_EDGE_FLAG_ENA            0x00040000L
#define PA_CL_CLIP_CNTL__DX_CLIP_SPACE_DEF_MASK            0x00080000L
#define PA_CL_CLIP_CNTL__DX_CLIP_SPACE_DEF                 0x00080000L
#define PA_CL_CLIP_CNTL__DIS_CLIP_ERR_DETECT_MASK          0x00100000L
#define PA_CL_CLIP_CNTL__DIS_CLIP_ERR_DETECT               0x00100000L
#define PA_CL_CLIP_CNTL__VTX_KILL_OR_MASK                  0x00200000L
#define PA_CL_CLIP_CNTL__VTX_KILL_OR                       0x00200000L
#define PA_CL_CLIP_CNTL__XY_NAN_RETAIN_MASK                0x00400000L
#define PA_CL_CLIP_CNTL__XY_NAN_RETAIN                     0x00400000L
#define PA_CL_CLIP_CNTL__Z_NAN_RETAIN_MASK                 0x00800000L
#define PA_CL_CLIP_CNTL__Z_NAN_RETAIN                      0x00800000L
#define PA_CL_CLIP_CNTL__W_NAN_RETAIN_MASK                 0x01000000L
#define PA_CL_CLIP_CNTL__W_NAN_RETAIN                      0x01000000L

// PA_CL_GB_VERT_CLIP_ADJ
#define PA_CL_GB_VERT_CLIP_ADJ__DATA_REGISTER_MASK         0xffffffffL

// PA_CL_GB_VERT_DISC_ADJ
#define PA_CL_GB_VERT_DISC_ADJ__DATA_REGISTER_MASK         0xffffffffL

// PA_CL_GB_HORZ_CLIP_ADJ
#define PA_CL_GB_HORZ_CLIP_ADJ__DATA_REGISTER_MASK         0xffffffffL

// PA_CL_GB_HORZ_DISC_ADJ
#define PA_CL_GB_HORZ_DISC_ADJ__DATA_REGISTER_MASK         0xffffffffL

// PA_CL_ENHANCE
#define PA_CL_ENHANCE__CLIP_VTX_REORDER_ENA_MASK           0x00000001L
#define PA_CL_ENHANCE__CLIP_VTX_REORDER_ENA                0x00000001L
#define PA_CL_ENHANCE__ECO_SPARE3_MASK                     0x10000000L
#define PA_CL_ENHANCE__ECO_SPARE3                          0x10000000L
#define PA_CL_ENHANCE__ECO_SPARE2_MASK                     0x20000000L
#define PA_CL_ENHANCE__ECO_SPARE2                          0x20000000L
#define PA_CL_ENHANCE__ECO_SPARE1_MASK                     0x40000000L
#define PA_CL_ENHANCE__ECO_SPARE1                          0x40000000L
#define PA_CL_ENHANCE__ECO_SPARE0_MASK                     0x80000000L
#define PA_CL_ENHANCE__ECO_SPARE0                          0x80000000L

// PA_SC_ENHANCE
#define PA_SC_ENHANCE__ECO_SPARE3_MASK                     0x10000000L
#define PA_SC_ENHANCE__ECO_SPARE3                          0x10000000L
#define PA_SC_ENHANCE__ECO_SPARE2_MASK                     0x20000000L
#define PA_SC_ENHANCE__ECO_SPARE2                          0x20000000L
#define PA_SC_ENHANCE__ECO_SPARE1_MASK                     0x40000000L
#define PA_SC_ENHANCE__ECO_SPARE1                          0x40000000L
#define PA_SC_ENHANCE__ECO_SPARE0_MASK                     0x80000000L
#define PA_SC_ENHANCE__ECO_SPARE0                          0x80000000L

// PA_SU_VTX_CNTL
#define PA_SU_VTX_CNTL__PIX_CENTER_MASK                    0x00000001L
#define PA_SU_VTX_CNTL__PIX_CENTER                         0x00000001L
#define PA_SU_VTX_CNTL__ROUND_MODE_MASK                    0x00000006L
#define PA_SU_VTX_CNTL__QUANT_MODE_MASK                    0x00000038L

// PA_SU_POINT_SIZE
#define PA_SU_POINT_SIZE__HEIGHT_MASK                      0x0000ffffL
#define PA_SU_POINT_SIZE__WIDTH_MASK                       0xffff0000L

// PA_SU_POINT_MINMAX
#define PA_SU_POINT_MINMAX__MIN_SIZE_MASK                  0x0000ffffL
#define PA_SU_POINT_MINMAX__MAX_SIZE_MASK                  0xffff0000L

// PA_SU_LINE_CNTL
#define PA_SU_LINE_CNTL__WIDTH_MASK                        0x0000ffffL

// PA_SU_SC_MODE_CNTL
#define PA_SU_SC_MODE_CNTL__CULL_FRONT_MASK                0x00000001L
#define PA_SU_SC_MODE_CNTL__CULL_FRONT                     0x00000001L
#define PA_SU_SC_MODE_CNTL__CULL_BACK_MASK                 0x00000002L
#define PA_SU_SC_MODE_CNTL__CULL_BACK                      0x00000002L
#define PA_SU_SC_MODE_CNTL__FACE_MASK                      0x00000004L
#define PA_SU_SC_MODE_CNTL__FACE                           0x00000004L
#define PA_SU_SC_MODE_CNTL__POLY_MODE_MASK                 0x00000018L
#define PA_SU_SC_MODE_CNTL__POLYMODE_FRONT_PTYPE_MASK      0x000000e0L
#define PA_SU_SC_MODE_CNTL__POLYMODE_BACK_PTYPE_MASK       0x00000700L
#define PA_SU_SC_MODE_CNTL__POLY_OFFSET_FRONT_ENABLE_MASK  0x00000800L
#define PA_SU_SC_MODE_CNTL__POLY_OFFSET_FRONT_ENABLE       0x00000800L
#define PA_SU_SC_MODE_CNTL__POLY_OFFSET_BACK_ENABLE_MASK   0x00001000L
#define PA_SU_SC_MODE_CNTL__POLY_OFFSET_BACK_ENABLE        0x00001000L
#define PA_SU_SC_MODE_CNTL__POLY_OFFSET_PARA_ENABLE_MASK   0x00002000L
#define PA_SU_SC_MODE_CNTL__POLY_OFFSET_PARA_ENABLE        0x00002000L
#define PA_SU_SC_MODE_CNTL__MSAA_ENABLE_MASK               0x00008000L
#define PA_SU_SC_MODE_CNTL__MSAA_ENABLE                    0x00008000L
#define PA_SU_SC_MODE_CNTL__VTX_WINDOW_OFFSET_ENABLE_MASK  0x00010000L
#define PA_SU_SC_MODE_CNTL__VTX_WINDOW_OFFSET_ENABLE       0x00010000L
#define PA_SU_SC_MODE_CNTL__LINE_STIPPLE_ENABLE_MASK       0x00040000L
#define PA_SU_SC_MODE_CNTL__LINE_STIPPLE_ENABLE            0x00040000L
#define PA_SU_SC_MODE_CNTL__PROVOKING_VTX_LAST_MASK        0x00080000L
#define PA_SU_SC_MODE_CNTL__PROVOKING_VTX_LAST             0x00080000L
#define PA_SU_SC_MODE_CNTL__PERSP_CORR_DIS_MASK            0x00100000L
#define PA_SU_SC_MODE_CNTL__PERSP_CORR_DIS                 0x00100000L
#define PA_SU_SC_MODE_CNTL__MULTI_PRIM_IB_ENA_MASK         0x00200000L
#define PA_SU_SC_MODE_CNTL__MULTI_PRIM_IB_ENA              0x00200000L
#define PA_SU_SC_MODE_CNTL__QUAD_ORDER_ENABLE_MASK         0x00800000L
#define PA_SU_SC_MODE_CNTL__QUAD_ORDER_ENABLE              0x00800000L
#define PA_SU_SC_MODE_CNTL__WAIT_RB_IDLE_ALL_TRI_MASK      0x02000000L
#define PA_SU_SC_MODE_CNTL__WAIT_RB_IDLE_ALL_TRI           0x02000000L
#define PA_SU_SC_MODE_CNTL__WAIT_RB_IDLE_FIRST_TRI_NEW_STATE_MASK 0x04000000L
#define PA_SU_SC_MODE_CNTL__WAIT_RB_IDLE_FIRST_TRI_NEW_STATE 0x04000000L

// PA_SU_POLY_OFFSET_FRONT_SCALE
#define PA_SU_POLY_OFFSET_FRONT_SCALE__SCALE_MASK          0xffffffffL

// PA_SU_POLY_OFFSET_FRONT_OFFSET
#define PA_SU_POLY_OFFSET_FRONT_OFFSET__OFFSET_MASK        0xffffffffL

// PA_SU_POLY_OFFSET_BACK_SCALE
#define PA_SU_POLY_OFFSET_BACK_SCALE__SCALE_MASK           0xffffffffL

// PA_SU_POLY_OFFSET_BACK_OFFSET
#define PA_SU_POLY_OFFSET_BACK_OFFSET__OFFSET_MASK         0xffffffffL

// PA_SU_PERFCOUNTER0_SELECT
#define PA_SU_PERFCOUNTER0_SELECT__PERF_SEL_MASK           0x000000ffL

// PA_SU_PERFCOUNTER1_SELECT
#define PA_SU_PERFCOUNTER1_SELECT__PERF_SEL_MASK           0x000000ffL

// PA_SU_PERFCOUNTER2_SELECT
#define PA_SU_PERFCOUNTER2_SELECT__PERF_SEL_MASK           0x000000ffL

// PA_SU_PERFCOUNTER3_SELECT
#define PA_SU_PERFCOUNTER3_SELECT__PERF_SEL_MASK           0x000000ffL

// PA_SU_PERFCOUNTER0_LOW
#define PA_SU_PERFCOUNTER0_LOW__PERF_COUNT_MASK            0xffffffffL

// PA_SU_PERFCOUNTER0_HI
#define PA_SU_PERFCOUNTER0_HI__PERF_COUNT_MASK             0x0000ffffL

// PA_SU_PERFCOUNTER1_LOW
#define PA_SU_PERFCOUNTER1_LOW__PERF_COUNT_MASK            0xffffffffL

// PA_SU_PERFCOUNTER1_HI
#define PA_SU_PERFCOUNTER1_HI__PERF_COUNT_MASK             0x0000ffffL

// PA_SU_PERFCOUNTER2_LOW
#define PA_SU_PERFCOUNTER2_LOW__PERF_COUNT_MASK            0xffffffffL

// PA_SU_PERFCOUNTER2_HI
#define PA_SU_PERFCOUNTER2_HI__PERF_COUNT_MASK             0x0000ffffL

// PA_SU_PERFCOUNTER3_LOW
#define PA_SU_PERFCOUNTER3_LOW__PERF_COUNT_MASK            0xffffffffL

// PA_SU_PERFCOUNTER3_HI
#define PA_SU_PERFCOUNTER3_HI__PERF_COUNT_MASK             0x0000ffffL

// PA_SC_WINDOW_OFFSET
#define PA_SC_WINDOW_OFFSET__WINDOW_X_OFFSET_MASK          0x00007fffL
#define PA_SC_WINDOW_OFFSET__WINDOW_Y_OFFSET_MASK          0x7fff0000L

// PA_SC_AA_CONFIG
#define PA_SC_AA_CONFIG__MSAA_NUM_SAMPLES_MASK             0x00000007L
#define PA_SC_AA_CONFIG__MAX_SAMPLE_DIST_MASK              0x0001e000L

// PA_SC_AA_MASK
#define PA_SC_AA_MASK__AA_MASK_MASK                        0x0000ffffL

// PA_SC_LINE_STIPPLE
#define PA_SC_LINE_STIPPLE__LINE_PATTERN_MASK              0x0000ffffL
#define PA_SC_LINE_STIPPLE__REPEAT_COUNT_MASK              0x00ff0000L
#define PA_SC_LINE_STIPPLE__PATTERN_BIT_ORDER_MASK         0x10000000L
#define PA_SC_LINE_STIPPLE__PATTERN_BIT_ORDER              0x10000000L
#define PA_SC_LINE_STIPPLE__AUTO_RESET_CNTL_MASK           0x60000000L

// PA_SC_LINE_CNTL
#define PA_SC_LINE_CNTL__BRES_CNTL_MASK                    0x000000ffL
#define PA_SC_LINE_CNTL__USE_BRES_CNTL_MASK                0x00000100L
#define PA_SC_LINE_CNTL__USE_BRES_CNTL                     0x00000100L
#define PA_SC_LINE_CNTL__EXPAND_LINE_WIDTH_MASK            0x00000200L
#define PA_SC_LINE_CNTL__EXPAND_LINE_WIDTH                 0x00000200L
#define PA_SC_LINE_CNTL__LAST_PIXEL_MASK                   0x00000400L
#define PA_SC_LINE_CNTL__LAST_PIXEL                        0x00000400L

// PA_SC_WINDOW_SCISSOR_TL
#define PA_SC_WINDOW_SCISSOR_TL__TL_X_MASK                 0x00003fffL
#define PA_SC_WINDOW_SCISSOR_TL__TL_Y_MASK                 0x3fff0000L
#define PA_SC_WINDOW_SCISSOR_TL__WINDOW_OFFSET_DISABLE_MASK 0x80000000L
#define PA_SC_WINDOW_SCISSOR_TL__WINDOW_OFFSET_DISABLE     0x80000000L

// PA_SC_WINDOW_SCISSOR_BR
#define PA_SC_WINDOW_SCISSOR_BR__BR_X_MASK                 0x00003fffL
#define PA_SC_WINDOW_SCISSOR_BR__BR_Y_MASK                 0x3fff0000L

// PA_SC_SCREEN_SCISSOR_TL
#define PA_SC_SCREEN_SCISSOR_TL__TL_X_MASK                 0x00007fffL
#define PA_SC_SCREEN_SCISSOR_TL__TL_Y_MASK                 0x7fff0000L

// PA_SC_SCREEN_SCISSOR_BR
#define PA_SC_SCREEN_SCISSOR_BR__BR_X_MASK                 0x00007fffL
#define PA_SC_SCREEN_SCISSOR_BR__BR_Y_MASK                 0x7fff0000L

// PA_SC_VIZ_QUERY
#define PA_SC_VIZ_QUERY__VIZ_QUERY_ENA_MASK                0x00000001L
#define PA_SC_VIZ_QUERY__VIZ_QUERY_ENA                     0x00000001L
#define PA_SC_VIZ_QUERY__VIZ_QUERY_ID_MASK                 0x0000003eL
#define PA_SC_VIZ_QUERY__KILL_PIX_POST_EARLY_Z_MASK        0x00000080L
#define PA_SC_VIZ_QUERY__KILL_PIX_POST_EARLY_Z             0x00000080L

// PA_SC_VIZ_QUERY_STATUS
#define PA_SC_VIZ_QUERY_STATUS__STATUS_BITS_MASK           0xffffffffL

// PA_SC_LINE_STIPPLE_STATE
#define PA_SC_LINE_STIPPLE_STATE__CURRENT_PTR_MASK         0x0000000fL
#define PA_SC_LINE_STIPPLE_STATE__CURRENT_COUNT_MASK       0x0000ff00L

// PA_SC_PERFCOUNTER0_SELECT
#define PA_SC_PERFCOUNTER0_SELECT__PERF_SEL_MASK           0x000000ffL

// PA_SC_PERFCOUNTER0_LOW
#define PA_SC_PERFCOUNTER0_LOW__PERF_COUNT_MASK            0xffffffffL

// PA_SC_PERFCOUNTER0_HI
#define PA_SC_PERFCOUNTER0_HI__PERF_COUNT_MASK             0x0000ffffL

// PA_CL_CNTL_STATUS
#define PA_CL_CNTL_STATUS__CL_BUSY_MASK                    0x80000000L
#define PA_CL_CNTL_STATUS__CL_BUSY                         0x80000000L

// PA_SU_CNTL_STATUS
#define PA_SU_CNTL_STATUS__SU_BUSY_MASK                    0x80000000L
#define PA_SU_CNTL_STATUS__SU_BUSY                         0x80000000L

// PA_SC_CNTL_STATUS
#define PA_SC_CNTL_STATUS__SC_BUSY_MASK                    0x80000000L
#define PA_SC_CNTL_STATUS__SC_BUSY                         0x80000000L

// PA_SU_DEBUG_CNTL
#define PA_SU_DEBUG_CNTL__SU_DEBUG_INDX_MASK               0x0000001fL

// PA_SU_DEBUG_DATA
#define PA_SU_DEBUG_DATA__DATA_MASK                        0xffffffffL

// CLIPPER_DEBUG_REG00
#define CLIPPER_DEBUG_REG00__clip_ga_bc_fifo_write_MASK    0x00000001L
#define CLIPPER_DEBUG_REG00__clip_ga_bc_fifo_write         0x00000001L
#define CLIPPER_DEBUG_REG00__clip_ga_bc_fifo_full_MASK     0x00000002L
#define CLIPPER_DEBUG_REG00__clip_ga_bc_fifo_full          0x00000002L
#define CLIPPER_DEBUG_REG00__clip_to_ga_fifo_write_MASK    0x00000004L
#define CLIPPER_DEBUG_REG00__clip_to_ga_fifo_write         0x00000004L
#define CLIPPER_DEBUG_REG00__clip_to_ga_fifo_full_MASK     0x00000008L
#define CLIPPER_DEBUG_REG00__clip_to_ga_fifo_full          0x00000008L
#define CLIPPER_DEBUG_REG00__primic_to_clprim_fifo_empty_MASK 0x00000010L
#define CLIPPER_DEBUG_REG00__primic_to_clprim_fifo_empty   0x00000010L
#define CLIPPER_DEBUG_REG00__primic_to_clprim_fifo_full_MASK 0x00000020L
#define CLIPPER_DEBUG_REG00__primic_to_clprim_fifo_full    0x00000020L
#define CLIPPER_DEBUG_REG00__clip_to_outsm_fifo_empty_MASK 0x00000040L
#define CLIPPER_DEBUG_REG00__clip_to_outsm_fifo_empty      0x00000040L
#define CLIPPER_DEBUG_REG00__clip_to_outsm_fifo_full_MASK  0x00000080L
#define CLIPPER_DEBUG_REG00__clip_to_outsm_fifo_full       0x00000080L
#define CLIPPER_DEBUG_REG00__vgt_to_clipp_fifo_empty_MASK  0x00000100L
#define CLIPPER_DEBUG_REG00__vgt_to_clipp_fifo_empty       0x00000100L
#define CLIPPER_DEBUG_REG00__vgt_to_clipp_fifo_full_MASK   0x00000200L
#define CLIPPER_DEBUG_REG00__vgt_to_clipp_fifo_full        0x00000200L
#define CLIPPER_DEBUG_REG00__vgt_to_clips_fifo_empty_MASK  0x00000400L
#define CLIPPER_DEBUG_REG00__vgt_to_clips_fifo_empty       0x00000400L
#define CLIPPER_DEBUG_REG00__vgt_to_clips_fifo_full_MASK   0x00000800L
#define CLIPPER_DEBUG_REG00__vgt_to_clips_fifo_full        0x00000800L
#define CLIPPER_DEBUG_REG00__clipcode_fifo_fifo_empty_MASK 0x00001000L
#define CLIPPER_DEBUG_REG00__clipcode_fifo_fifo_empty      0x00001000L
#define CLIPPER_DEBUG_REG00__clipcode_fifo_full_MASK       0x00002000L
#define CLIPPER_DEBUG_REG00__clipcode_fifo_full            0x00002000L
#define CLIPPER_DEBUG_REG00__vte_out_clip_fifo_fifo_empty_MASK 0x00004000L
#define CLIPPER_DEBUG_REG00__vte_out_clip_fifo_fifo_empty  0x00004000L
#define CLIPPER_DEBUG_REG00__vte_out_clip_fifo_fifo_full_MASK 0x00008000L
#define CLIPPER_DEBUG_REG00__vte_out_clip_fifo_fifo_full   0x00008000L
#define CLIPPER_DEBUG_REG00__vte_out_orig_fifo_fifo_empty_MASK 0x00010000L
#define CLIPPER_DEBUG_REG00__vte_out_orig_fifo_fifo_empty  0x00010000L
#define CLIPPER_DEBUG_REG00__vte_out_orig_fifo_fifo_full_MASK 0x00020000L
#define CLIPPER_DEBUG_REG00__vte_out_orig_fifo_fifo_full   0x00020000L
#define CLIPPER_DEBUG_REG00__ccgen_to_clipcc_fifo_empty_MASK 0x00040000L
#define CLIPPER_DEBUG_REG00__ccgen_to_clipcc_fifo_empty    0x00040000L
#define CLIPPER_DEBUG_REG00__ccgen_to_clipcc_fifo_full_MASK 0x00080000L
#define CLIPPER_DEBUG_REG00__ccgen_to_clipcc_fifo_full     0x00080000L
#define CLIPPER_DEBUG_REG00__ALWAYS_ZERO_MASK              0xfff00000L

// CLIPPER_DEBUG_REG01
#define CLIPPER_DEBUG_REG01__clip_to_outsm_end_of_packet_MASK 0x00000001L
#define CLIPPER_DEBUG_REG01__clip_to_outsm_end_of_packet   0x00000001L
#define CLIPPER_DEBUG_REG01__clip_to_outsm_first_prim_of_slot_MASK 0x00000002L
#define CLIPPER_DEBUG_REG01__clip_to_outsm_first_prim_of_slot 0x00000002L
#define CLIPPER_DEBUG_REG01__clip_to_outsm_deallocate_slot_MASK 0x0000001cL
#define CLIPPER_DEBUG_REG01__clip_to_outsm_clipped_prim_MASK 0x00000020L
#define CLIPPER_DEBUG_REG01__clip_to_outsm_clipped_prim    0x00000020L
#define CLIPPER_DEBUG_REG01__clip_to_outsm_null_primitive_MASK 0x00000040L
#define CLIPPER_DEBUG_REG01__clip_to_outsm_null_primitive  0x00000040L
#define CLIPPER_DEBUG_REG01__clip_to_outsm_vertex_store_indx_2_MASK 0x00000780L
#define CLIPPER_DEBUG_REG01__clip_to_outsm_vertex_store_indx_1_MASK 0x00007800L
#define CLIPPER_DEBUG_REG01__clip_to_outsm_vertex_store_indx_0_MASK 0x00078000L
#define CLIPPER_DEBUG_REG01__clip_vert_vte_valid_MASK      0x00380000L
#define CLIPPER_DEBUG_REG01__vte_out_clip_rd_vertex_store_indx_MASK 0x00c00000L
#define CLIPPER_DEBUG_REG01__ALWAYS_ZERO_MASK              0xff000000L

// CLIPPER_DEBUG_REG02
#define CLIPPER_DEBUG_REG02__ALWAYS_ZERO1_MASK             0x001fffffL
#define CLIPPER_DEBUG_REG02__clipsm0_clip_to_clipga_clip_to_outsm_cnt_MASK 0x00e00000L
#define CLIPPER_DEBUG_REG02__ALWAYS_ZERO0_MASK             0x7f000000L
#define CLIPPER_DEBUG_REG02__clipsm0_clprim_to_clip_prim_valid_MASK 0x80000000L
#define CLIPPER_DEBUG_REG02__clipsm0_clprim_to_clip_prim_valid 0x80000000L

// CLIPPER_DEBUG_REG03
#define CLIPPER_DEBUG_REG03__ALWAYS_ZERO3_MASK             0x00000007L
#define CLIPPER_DEBUG_REG03__clipsm0_clprim_to_clip_clip_primitive_MASK 0x00000008L
#define CLIPPER_DEBUG_REG03__clipsm0_clprim_to_clip_clip_primitive 0x00000008L
#define CLIPPER_DEBUG_REG03__ALWAYS_ZERO2_MASK             0x00000070L
#define CLIPPER_DEBUG_REG03__clipsm0_clprim_to_clip_null_primitive_MASK 0x00000080L
#define CLIPPER_DEBUG_REG03__clipsm0_clprim_to_clip_null_primitive 0x00000080L
#define CLIPPER_DEBUG_REG03__ALWAYS_ZERO1_MASK             0x000fff00L
#define CLIPPER_DEBUG_REG03__clipsm0_clprim_to_clip_clip_code_or_MASK 0x03f00000L
#define CLIPPER_DEBUG_REG03__ALWAYS_ZERO0_MASK             0xfc000000L

// CLIPPER_DEBUG_REG04
#define CLIPPER_DEBUG_REG04__ALWAYS_ZERO2_MASK             0x00000007L
#define CLIPPER_DEBUG_REG04__clipsm0_clprim_to_clip_first_prim_of_slot_MASK 0x00000008L
#define CLIPPER_DEBUG_REG04__clipsm0_clprim_to_clip_first_prim_of_slot 0x00000008L
#define CLIPPER_DEBUG_REG04__ALWAYS_ZERO1_MASK             0x00000070L
#define CLIPPER_DEBUG_REG04__clipsm0_clprim_to_clip_event_MASK 0x00000080L
#define CLIPPER_DEBUG_REG04__clipsm0_clprim_to_clip_event  0x00000080L
#define CLIPPER_DEBUG_REG04__ALWAYS_ZERO0_MASK             0xffffff00L

// CLIPPER_DEBUG_REG05
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_state_var_indx_MASK 0x00000001L
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_state_var_indx 0x00000001L
#define CLIPPER_DEBUG_REG05__ALWAYS_ZERO3_MASK             0x00000006L
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_deallocate_slot_MASK 0x00000038L
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_event_id_MASK 0x00000fc0L
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_vertex_store_indx_2_MASK 0x0000f000L
#define CLIPPER_DEBUG_REG05__ALWAYS_ZERO2_MASK             0x00030000L
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_vertex_store_indx_1_MASK 0x003c0000L
#define CLIPPER_DEBUG_REG05__ALWAYS_ZERO1_MASK             0x00c00000L
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_vertex_store_indx_0_MASK 0x0f000000L
#define CLIPPER_DEBUG_REG05__ALWAYS_ZERO0_MASK             0xf0000000L

// CLIPPER_DEBUG_REG09
#define CLIPPER_DEBUG_REG09__clprim_in_back_event_MASK     0x00000001L
#define CLIPPER_DEBUG_REG09__clprim_in_back_event          0x00000001L
#define CLIPPER_DEBUG_REG09__outputclprimtoclip_null_primitive_MASK 0x00000002L
#define CLIPPER_DEBUG_REG09__outputclprimtoclip_null_primitive 0x00000002L
#define CLIPPER_DEBUG_REG09__clprim_in_back_vertex_store_indx_2_MASK 0x0000003cL
#define CLIPPER_DEBUG_REG09__ALWAYS_ZERO2_MASK             0x000000c0L
#define CLIPPER_DEBUG_REG09__clprim_in_back_vertex_store_indx_1_MASK 0x00000f00L
#define CLIPPER_DEBUG_REG09__ALWAYS_ZERO1_MASK             0x00003000L
#define CLIPPER_DEBUG_REG09__clprim_in_back_vertex_store_indx_0_MASK 0x0003c000L
#define CLIPPER_DEBUG_REG09__ALWAYS_ZERO0_MASK             0x000c0000L
#define CLIPPER_DEBUG_REG09__prim_back_valid_MASK          0x00100000L
#define CLIPPER_DEBUG_REG09__prim_back_valid               0x00100000L
#define CLIPPER_DEBUG_REG09__clip_priority_seq_indx_out_cnt_MASK 0x01e00000L
#define CLIPPER_DEBUG_REG09__outsm_clr_rd_orig_vertices_MASK 0x06000000L
#define CLIPPER_DEBUG_REG09__outsm_clr_rd_clipsm_wait_MASK 0x08000000L
#define CLIPPER_DEBUG_REG09__outsm_clr_rd_clipsm_wait      0x08000000L
#define CLIPPER_DEBUG_REG09__outsm_clr_fifo_empty_MASK     0x10000000L
#define CLIPPER_DEBUG_REG09__outsm_clr_fifo_empty          0x10000000L
#define CLIPPER_DEBUG_REG09__outsm_clr_fifo_full_MASK      0x20000000L
#define CLIPPER_DEBUG_REG09__outsm_clr_fifo_full           0x20000000L
#define CLIPPER_DEBUG_REG09__clip_priority_seq_indx_load_MASK 0xc0000000L

// CLIPPER_DEBUG_REG10
#define CLIPPER_DEBUG_REG10__primic_to_clprim_fifo_vertex_store_indx_2_MASK 0x0000000fL
#define CLIPPER_DEBUG_REG10__ALWAYS_ZERO3_MASK             0x00000030L
#define CLIPPER_DEBUG_REG10__primic_to_clprim_fifo_vertex_store_indx_1_MASK 0x000003c0L
#define CLIPPER_DEBUG_REG10__ALWAYS_ZERO2_MASK             0x00000c00L
#define CLIPPER_DEBUG_REG10__primic_to_clprim_fifo_vertex_store_indx_0_MASK 0x0000f000L
#define CLIPPER_DEBUG_REG10__ALWAYS_ZERO1_MASK             0x00030000L
#define CLIPPER_DEBUG_REG10__clprim_in_back_state_var_indx_MASK 0x00040000L
#define CLIPPER_DEBUG_REG10__clprim_in_back_state_var_indx 0x00040000L
#define CLIPPER_DEBUG_REG10__ALWAYS_ZERO0_MASK             0x00180000L
#define CLIPPER_DEBUG_REG10__clprim_in_back_end_of_packet_MASK 0x00200000L
#define CLIPPER_DEBUG_REG10__clprim_in_back_end_of_packet  0x00200000L
#define CLIPPER_DEBUG_REG10__clprim_in_back_first_prim_of_slot_MASK 0x00400000L
#define CLIPPER_DEBUG_REG10__clprim_in_back_first_prim_of_slot 0x00400000L
#define CLIPPER_DEBUG_REG10__clprim_in_back_deallocate_slot_MASK 0x03800000L
#define CLIPPER_DEBUG_REG10__clprim_in_back_event_id_MASK  0xfc000000L

// CLIPPER_DEBUG_REG11
#define CLIPPER_DEBUG_REG11__vertval_bits_vertex_vertex_store_msb_MASK 0x0000000fL
#define CLIPPER_DEBUG_REG11__ALWAYS_ZERO_MASK              0xfffffff0L

// CLIPPER_DEBUG_REG12
#define CLIPPER_DEBUG_REG12__clip_priority_available_vte_out_clip_MASK 0x00000003L
#define CLIPPER_DEBUG_REG12__ALWAYS_ZERO2_MASK             0x0000001cL
#define CLIPPER_DEBUG_REG12__clip_vertex_fifo_empty_MASK   0x00000020L
#define CLIPPER_DEBUG_REG12__clip_vertex_fifo_empty        0x00000020L
#define CLIPPER_DEBUG_REG12__clip_priority_available_clip_verts_MASK 0x000007c0L
#define CLIPPER_DEBUG_REG12__ALWAYS_ZERO1_MASK             0x00007800L
#define CLIPPER_DEBUG_REG12__vertval_bits_vertex_cc_next_valid_MASK 0x00078000L
#define CLIPPER_DEBUG_REG12__clipcc_vertex_store_indx_MASK 0x00180000L
#define CLIPPER_DEBUG_REG12__primic_to_clprim_valid_MASK   0x00200000L
#define CLIPPER_DEBUG_REG12__primic_to_clprim_valid        0x00200000L
#define CLIPPER_DEBUG_REG12__ALWAYS_ZERO0_MASK             0xffc00000L

// CLIPPER_DEBUG_REG13
#define CLIPPER_DEBUG_REG13__sm0_clip_vert_cnt_MASK        0x0000000fL
#define CLIPPER_DEBUG_REG13__sm0_prim_end_state_MASK       0x000007f0L
#define CLIPPER_DEBUG_REG13__ALWAYS_ZERO1_MASK             0x00003800L
#define CLIPPER_DEBUG_REG13__sm0_vertex_clip_cnt_MASK      0x0003c000L
#define CLIPPER_DEBUG_REG13__sm0_inv_to_clip_data_valid_1_MASK 0x00040000L
#define CLIPPER_DEBUG_REG13__sm0_inv_to_clip_data_valid_1  0x00040000L
#define CLIPPER_DEBUG_REG13__sm0_inv_to_clip_data_valid_0_MASK 0x00080000L
#define CLIPPER_DEBUG_REG13__sm0_inv_to_clip_data_valid_0  0x00080000L
#define CLIPPER_DEBUG_REG13__sm0_current_state_MASK        0x07f00000L
#define CLIPPER_DEBUG_REG13__ALWAYS_ZERO0_MASK             0xf8000000L

// SXIFCCG_DEBUG_REG0
#define SXIFCCG_DEBUG_REG0__nan_kill_flag_MASK             0x0000000fL
#define SXIFCCG_DEBUG_REG0__position_address_MASK          0x00000070L
#define SXIFCCG_DEBUG_REG0__ALWAYS_ZERO2_MASK              0x00000380L
#define SXIFCCG_DEBUG_REG0__point_address_MASK             0x00001c00L
#define SXIFCCG_DEBUG_REG0__ALWAYS_ZERO1_MASK              0x0000e000L
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_state_var_indx_MASK 0x00010000L
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_state_var_indx   0x00010000L
#define SXIFCCG_DEBUG_REG0__ALWAYS_ZERO0_MASK              0x00060000L
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_req_mask_MASK    0x00780000L
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_pci_MASK         0x3f800000L
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_aux_inc_MASK     0x40000000L
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_aux_inc          0x40000000L
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_aux_sel_MASK     0x80000000L
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_aux_sel          0x80000000L

// SXIFCCG_DEBUG_REG1
#define SXIFCCG_DEBUG_REG1__ALWAYS_ZERO3_MASK              0x00000003L
#define SXIFCCG_DEBUG_REG1__sx_to_pa_empty_MASK            0x0000000cL
#define SXIFCCG_DEBUG_REG1__available_positions_MASK       0x00000070L
#define SXIFCCG_DEBUG_REG1__ALWAYS_ZERO2_MASK              0x00000780L
#define SXIFCCG_DEBUG_REG1__sx_pending_advance_MASK        0x00000800L
#define SXIFCCG_DEBUG_REG1__sx_pending_advance             0x00000800L
#define SXIFCCG_DEBUG_REG1__sx_receive_indx_MASK           0x00007000L
#define SXIFCCG_DEBUG_REG1__statevar_bits_sxpa_aux_vector_MASK 0x00008000L
#define SXIFCCG_DEBUG_REG1__statevar_bits_sxpa_aux_vector  0x00008000L
#define SXIFCCG_DEBUG_REG1__ALWAYS_ZERO1_MASK              0x000f0000L
#define SXIFCCG_DEBUG_REG1__aux_sel_MASK                   0x00100000L
#define SXIFCCG_DEBUG_REG1__aux_sel                        0x00100000L
#define SXIFCCG_DEBUG_REG1__ALWAYS_ZERO0_MASK              0x00600000L
#define SXIFCCG_DEBUG_REG1__pasx_req_cnt_MASK              0x01800000L
#define SXIFCCG_DEBUG_REG1__param_cache_base_MASK          0xfe000000L

// SXIFCCG_DEBUG_REG2
#define SXIFCCG_DEBUG_REG2__sx_sent_MASK                   0x00000001L
#define SXIFCCG_DEBUG_REG2__sx_sent                        0x00000001L
#define SXIFCCG_DEBUG_REG2__ALWAYS_ZERO3_MASK              0x00000002L
#define SXIFCCG_DEBUG_REG2__ALWAYS_ZERO3                   0x00000002L
#define SXIFCCG_DEBUG_REG2__sx_aux_MASK                    0x00000004L
#define SXIFCCG_DEBUG_REG2__sx_aux                         0x00000004L
#define SXIFCCG_DEBUG_REG2__sx_request_indx_MASK           0x000001f8L
#define SXIFCCG_DEBUG_REG2__req_active_verts_MASK          0x0000fe00L
#define SXIFCCG_DEBUG_REG2__ALWAYS_ZERO2_MASK              0x00010000L
#define SXIFCCG_DEBUG_REG2__ALWAYS_ZERO2                   0x00010000L
#define SXIFCCG_DEBUG_REG2__vgt_to_ccgen_state_var_indx_MASK 0x00020000L
#define SXIFCCG_DEBUG_REG2__vgt_to_ccgen_state_var_indx    0x00020000L
#define SXIFCCG_DEBUG_REG2__ALWAYS_ZERO1_MASK              0x000c0000L
#define SXIFCCG_DEBUG_REG2__vgt_to_ccgen_active_verts_MASK 0x00300000L
#define SXIFCCG_DEBUG_REG2__ALWAYS_ZERO0_MASK              0x03c00000L
#define SXIFCCG_DEBUG_REG2__req_active_verts_loaded_MASK   0x04000000L
#define SXIFCCG_DEBUG_REG2__req_active_verts_loaded        0x04000000L
#define SXIFCCG_DEBUG_REG2__sx_pending_fifo_empty_MASK     0x08000000L
#define SXIFCCG_DEBUG_REG2__sx_pending_fifo_empty          0x08000000L
#define SXIFCCG_DEBUG_REG2__sx_pending_fifo_full_MASK      0x10000000L
#define SXIFCCG_DEBUG_REG2__sx_pending_fifo_full           0x10000000L
#define SXIFCCG_DEBUG_REG2__sx_pending_fifo_contents_MASK  0xe0000000L

// SXIFCCG_DEBUG_REG3
#define SXIFCCG_DEBUG_REG3__vertex_fifo_entriesavailable_MASK 0x0000000fL
#define SXIFCCG_DEBUG_REG3__ALWAYS_ZERO3_MASK              0x00000010L
#define SXIFCCG_DEBUG_REG3__ALWAYS_ZERO3                   0x00000010L
#define SXIFCCG_DEBUG_REG3__available_positions_MASK       0x000000e0L
#define SXIFCCG_DEBUG_REG3__ALWAYS_ZERO2_MASK              0x00000f00L
#define SXIFCCG_DEBUG_REG3__current_state_MASK             0x00003000L
#define SXIFCCG_DEBUG_REG3__vertex_fifo_empty_MASK         0x00004000L
#define SXIFCCG_DEBUG_REG3__vertex_fifo_empty              0x00004000L
#define SXIFCCG_DEBUG_REG3__vertex_fifo_full_MASK          0x00008000L
#define SXIFCCG_DEBUG_REG3__vertex_fifo_full               0x00008000L
#define SXIFCCG_DEBUG_REG3__ALWAYS_ZERO1_MASK              0x00030000L
#define SXIFCCG_DEBUG_REG3__sx0_receive_fifo_empty_MASK    0x00040000L
#define SXIFCCG_DEBUG_REG3__sx0_receive_fifo_empty         0x00040000L
#define SXIFCCG_DEBUG_REG3__sx0_receive_fifo_full_MASK     0x00080000L
#define SXIFCCG_DEBUG_REG3__sx0_receive_fifo_full          0x00080000L
#define SXIFCCG_DEBUG_REG3__vgt_to_ccgen_fifo_empty_MASK   0x00100000L
#define SXIFCCG_DEBUG_REG3__vgt_to_ccgen_fifo_empty        0x00100000L
#define SXIFCCG_DEBUG_REG3__vgt_to_ccgen_fifo_full_MASK    0x00200000L
#define SXIFCCG_DEBUG_REG3__vgt_to_ccgen_fifo_full         0x00200000L
#define SXIFCCG_DEBUG_REG3__ALWAYS_ZERO0_MASK              0xffc00000L

// SETUP_DEBUG_REG0
#define SETUP_DEBUG_REG0__su_cntl_state_MASK               0x0000001fL
#define SETUP_DEBUG_REG0__pmode_state_MASK                 0x000007e0L
#define SETUP_DEBUG_REG0__ge_stallb_MASK                   0x00000800L
#define SETUP_DEBUG_REG0__ge_stallb                        0x00000800L
#define SETUP_DEBUG_REG0__geom_enable_MASK                 0x00001000L
#define SETUP_DEBUG_REG0__geom_enable                      0x00001000L
#define SETUP_DEBUG_REG0__su_clip_baryc_rtr_MASK           0x00002000L
#define SETUP_DEBUG_REG0__su_clip_baryc_rtr                0x00002000L
#define SETUP_DEBUG_REG0__su_clip_rtr_MASK                 0x00004000L
#define SETUP_DEBUG_REG0__su_clip_rtr                      0x00004000L
#define SETUP_DEBUG_REG0__pfifo_busy_MASK                  0x00008000L
#define SETUP_DEBUG_REG0__pfifo_busy                       0x00008000L
#define SETUP_DEBUG_REG0__su_cntl_busy_MASK                0x00010000L
#define SETUP_DEBUG_REG0__su_cntl_busy                     0x00010000L
#define SETUP_DEBUG_REG0__geom_busy_MASK                   0x00020000L
#define SETUP_DEBUG_REG0__geom_busy                        0x00020000L

// SETUP_DEBUG_REG1
#define SETUP_DEBUG_REG1__y_sort0_gated_17_4_MASK          0x00003fffL
#define SETUP_DEBUG_REG1__x_sort0_gated_17_4_MASK          0x0fffc000L

// SETUP_DEBUG_REG2
#define SETUP_DEBUG_REG2__y_sort1_gated_17_4_MASK          0x00003fffL
#define SETUP_DEBUG_REG2__x_sort1_gated_17_4_MASK          0x0fffc000L

// SETUP_DEBUG_REG3
#define SETUP_DEBUG_REG3__y_sort2_gated_17_4_MASK          0x00003fffL
#define SETUP_DEBUG_REG3__x_sort2_gated_17_4_MASK          0x0fffc000L

// SETUP_DEBUG_REG4
#define SETUP_DEBUG_REG4__attr_indx_sort0_gated_MASK       0x000007ffL
#define SETUP_DEBUG_REG4__null_prim_gated_MASK             0x00000800L
#define SETUP_DEBUG_REG4__null_prim_gated                  0x00000800L
#define SETUP_DEBUG_REG4__backfacing_gated_MASK            0x00001000L
#define SETUP_DEBUG_REG4__backfacing_gated                 0x00001000L
#define SETUP_DEBUG_REG4__st_indx_gated_MASK               0x0000e000L
#define SETUP_DEBUG_REG4__clipped_gated_MASK               0x00010000L
#define SETUP_DEBUG_REG4__clipped_gated                    0x00010000L
#define SETUP_DEBUG_REG4__dealloc_slot_gated_MASK          0x000e0000L
#define SETUP_DEBUG_REG4__xmajor_gated_MASK                0x00100000L
#define SETUP_DEBUG_REG4__xmajor_gated                     0x00100000L
#define SETUP_DEBUG_REG4__diamond_rule_gated_MASK          0x00600000L
#define SETUP_DEBUG_REG4__type_gated_MASK                  0x03800000L
#define SETUP_DEBUG_REG4__fpov_gated_MASK                  0x04000000L
#define SETUP_DEBUG_REG4__fpov_gated                       0x04000000L
#define SETUP_DEBUG_REG4__pmode_prim_gated_MASK            0x08000000L
#define SETUP_DEBUG_REG4__pmode_prim_gated                 0x08000000L
#define SETUP_DEBUG_REG4__event_gated_MASK                 0x10000000L
#define SETUP_DEBUG_REG4__event_gated                      0x10000000L
#define SETUP_DEBUG_REG4__eop_gated_MASK                   0x20000000L
#define SETUP_DEBUG_REG4__eop_gated                        0x20000000L

// SETUP_DEBUG_REG5
#define SETUP_DEBUG_REG5__attr_indx_sort2_gated_MASK       0x000007ffL
#define SETUP_DEBUG_REG5__attr_indx_sort1_gated_MASK       0x003ff800L
#define SETUP_DEBUG_REG5__provoking_vtx_gated_MASK         0x00c00000L
#define SETUP_DEBUG_REG5__event_id_gated_MASK              0x1f000000L

// PA_SC_DEBUG_CNTL
#define PA_SC_DEBUG_CNTL__SC_DEBUG_INDX_MASK               0x0000001fL

// PA_SC_DEBUG_DATA
#define PA_SC_DEBUG_DATA__DATA_MASK                        0xffffffffL

// SC_DEBUG_0
#define SC_DEBUG_0__pa_freeze_b1_MASK                      0x00000001L
#define SC_DEBUG_0__pa_freeze_b1                           0x00000001L
#define SC_DEBUG_0__pa_sc_valid_MASK                       0x00000002L
#define SC_DEBUG_0__pa_sc_valid                            0x00000002L
#define SC_DEBUG_0__pa_sc_phase_MASK                       0x0000001cL
#define SC_DEBUG_0__cntx_cnt_MASK                          0x00000fe0L
#define SC_DEBUG_0__decr_cntx_cnt_MASK                     0x00001000L
#define SC_DEBUG_0__decr_cntx_cnt                          0x00001000L
#define SC_DEBUG_0__incr_cntx_cnt_MASK                     0x00002000L
#define SC_DEBUG_0__incr_cntx_cnt                          0x00002000L
#define SC_DEBUG_0__trigger_MASK                           0x80000000L
#define SC_DEBUG_0__trigger                                0x80000000L

// SC_DEBUG_1
#define SC_DEBUG_1__em_state_MASK                          0x00000007L
#define SC_DEBUG_1__em1_data_ready_MASK                    0x00000008L
#define SC_DEBUG_1__em1_data_ready                         0x00000008L
#define SC_DEBUG_1__em2_data_ready_MASK                    0x00000010L
#define SC_DEBUG_1__em2_data_ready                         0x00000010L
#define SC_DEBUG_1__move_em1_to_em2_MASK                   0x00000020L
#define SC_DEBUG_1__move_em1_to_em2                        0x00000020L
#define SC_DEBUG_1__ef_data_ready_MASK                     0x00000040L
#define SC_DEBUG_1__ef_data_ready                          0x00000040L
#define SC_DEBUG_1__ef_state_MASK                          0x00000180L
#define SC_DEBUG_1__pipe_valid_MASK                        0x00000200L
#define SC_DEBUG_1__pipe_valid                             0x00000200L
#define SC_DEBUG_1__trigger_MASK                           0x80000000L
#define SC_DEBUG_1__trigger                                0x80000000L

// SC_DEBUG_2
#define SC_DEBUG_2__rc_rtr_dly_MASK                        0x00000001L
#define SC_DEBUG_2__rc_rtr_dly                             0x00000001L
#define SC_DEBUG_2__qmask_ff_alm_full_d1_MASK              0x00000002L
#define SC_DEBUG_2__qmask_ff_alm_full_d1                   0x00000002L
#define SC_DEBUG_2__pipe_freeze_b_MASK                     0x00000008L
#define SC_DEBUG_2__pipe_freeze_b                          0x00000008L
#define SC_DEBUG_2__prim_rts_MASK                          0x00000010L
#define SC_DEBUG_2__prim_rts                               0x00000010L
#define SC_DEBUG_2__next_prim_rts_dly_MASK                 0x00000020L
#define SC_DEBUG_2__next_prim_rts_dly                      0x00000020L
#define SC_DEBUG_2__next_prim_rtr_dly_MASK                 0x00000040L
#define SC_DEBUG_2__next_prim_rtr_dly                      0x00000040L
#define SC_DEBUG_2__pre_stage1_rts_d1_MASK                 0x00000080L
#define SC_DEBUG_2__pre_stage1_rts_d1                      0x00000080L
#define SC_DEBUG_2__stage0_rts_MASK                        0x00000100L
#define SC_DEBUG_2__stage0_rts                             0x00000100L
#define SC_DEBUG_2__phase_rts_dly_MASK                     0x00000200L
#define SC_DEBUG_2__phase_rts_dly                          0x00000200L
#define SC_DEBUG_2__end_of_prim_s1_dly_MASK                0x00008000L
#define SC_DEBUG_2__end_of_prim_s1_dly                     0x00008000L
#define SC_DEBUG_2__pass_empty_prim_s1_MASK                0x00010000L
#define SC_DEBUG_2__pass_empty_prim_s1                     0x00010000L
#define SC_DEBUG_2__event_id_s1_MASK                       0x003e0000L
#define SC_DEBUG_2__event_s1_MASK                          0x00400000L
#define SC_DEBUG_2__event_s1                               0x00400000L
#define SC_DEBUG_2__trigger_MASK                           0x80000000L
#define SC_DEBUG_2__trigger                                0x80000000L

// SC_DEBUG_3
#define SC_DEBUG_3__x_curr_s1_MASK                         0x000007ffL
#define SC_DEBUG_3__y_curr_s1_MASK                         0x003ff800L
#define SC_DEBUG_3__trigger_MASK                           0x80000000L
#define SC_DEBUG_3__trigger                                0x80000000L

// SC_DEBUG_4
#define SC_DEBUG_4__y_end_s1_MASK                          0x00003fffL
#define SC_DEBUG_4__y_start_s1_MASK                        0x0fffc000L
#define SC_DEBUG_4__y_dir_s1_MASK                          0x10000000L
#define SC_DEBUG_4__y_dir_s1                               0x10000000L
#define SC_DEBUG_4__trigger_MASK                           0x80000000L
#define SC_DEBUG_4__trigger                                0x80000000L

// SC_DEBUG_5
#define SC_DEBUG_5__x_end_s1_MASK                          0x00003fffL
#define SC_DEBUG_5__x_start_s1_MASK                        0x0fffc000L
#define SC_DEBUG_5__x_dir_s1_MASK                          0x10000000L
#define SC_DEBUG_5__x_dir_s1                               0x10000000L
#define SC_DEBUG_5__trigger_MASK                           0x80000000L
#define SC_DEBUG_5__trigger                                0x80000000L

// SC_DEBUG_6
#define SC_DEBUG_6__z_ff_empty_MASK                        0x00000001L
#define SC_DEBUG_6__z_ff_empty                             0x00000001L
#define SC_DEBUG_6__qmcntl_ff_empty_MASK                   0x00000002L
#define SC_DEBUG_6__qmcntl_ff_empty                        0x00000002L
#define SC_DEBUG_6__xy_ff_empty_MASK                       0x00000004L
#define SC_DEBUG_6__xy_ff_empty                            0x00000004L
#define SC_DEBUG_6__event_flag_MASK                        0x00000008L
#define SC_DEBUG_6__event_flag                             0x00000008L
#define SC_DEBUG_6__z_mask_needed_MASK                     0x00000010L
#define SC_DEBUG_6__z_mask_needed                          0x00000010L
#define SC_DEBUG_6__state_MASK                             0x000000e0L
#define SC_DEBUG_6__state_delayed_MASK                     0x00000700L
#define SC_DEBUG_6__data_valid_MASK                        0x00000800L
#define SC_DEBUG_6__data_valid                             0x00000800L
#define SC_DEBUG_6__data_valid_d_MASK                      0x00001000L
#define SC_DEBUG_6__data_valid_d                           0x00001000L
#define SC_DEBUG_6__tilex_delayed_MASK                     0x003fe000L
#define SC_DEBUG_6__tiley_delayed_MASK                     0x7fc00000L
#define SC_DEBUG_6__trigger_MASK                           0x80000000L
#define SC_DEBUG_6__trigger                                0x80000000L

// SC_DEBUG_7
#define SC_DEBUG_7__event_flag_MASK                        0x00000001L
#define SC_DEBUG_7__event_flag                             0x00000001L
#define SC_DEBUG_7__deallocate_MASK                        0x0000000eL
#define SC_DEBUG_7__fpos_MASK                              0x00000010L
#define SC_DEBUG_7__fpos                                   0x00000010L
#define SC_DEBUG_7__sr_prim_we_MASK                        0x00000020L
#define SC_DEBUG_7__sr_prim_we                             0x00000020L
#define SC_DEBUG_7__last_tile_MASK                         0x00000040L
#define SC_DEBUG_7__last_tile                              0x00000040L
#define SC_DEBUG_7__tile_ff_we_MASK                        0x00000080L
#define SC_DEBUG_7__tile_ff_we                             0x00000080L
#define SC_DEBUG_7__qs_data_valid_MASK                     0x00000100L
#define SC_DEBUG_7__qs_data_valid                          0x00000100L
#define SC_DEBUG_7__qs_q0_y_MASK                           0x00000600L
#define SC_DEBUG_7__qs_q0_x_MASK                           0x00001800L
#define SC_DEBUG_7__qs_q0_valid_MASK                       0x00002000L
#define SC_DEBUG_7__qs_q0_valid                            0x00002000L
#define SC_DEBUG_7__prim_ff_we_MASK                        0x00004000L
#define SC_DEBUG_7__prim_ff_we                             0x00004000L
#define SC_DEBUG_7__tile_ff_re_MASK                        0x00008000L
#define SC_DEBUG_7__tile_ff_re                             0x00008000L
#define SC_DEBUG_7__fw_prim_data_valid_MASK                0x00010000L
#define SC_DEBUG_7__fw_prim_data_valid                     0x00010000L
#define SC_DEBUG_7__last_quad_of_tile_MASK                 0x00020000L
#define SC_DEBUG_7__last_quad_of_tile                      0x00020000L
#define SC_DEBUG_7__first_quad_of_tile_MASK                0x00040000L
#define SC_DEBUG_7__first_quad_of_tile                     0x00040000L
#define SC_DEBUG_7__first_quad_of_prim_MASK                0x00080000L
#define SC_DEBUG_7__first_quad_of_prim                     0x00080000L
#define SC_DEBUG_7__new_prim_MASK                          0x00100000L
#define SC_DEBUG_7__new_prim                               0x00100000L
#define SC_DEBUG_7__load_new_tile_data_MASK                0x00200000L
#define SC_DEBUG_7__load_new_tile_data                     0x00200000L
#define SC_DEBUG_7__state_MASK                             0x00c00000L
#define SC_DEBUG_7__fifos_ready_MASK                       0x01000000L
#define SC_DEBUG_7__fifos_ready                            0x01000000L
#define SC_DEBUG_7__trigger_MASK                           0x80000000L
#define SC_DEBUG_7__trigger                                0x80000000L

// SC_DEBUG_8
#define SC_DEBUG_8__sample_last_MASK                       0x00000001L
#define SC_DEBUG_8__sample_last                            0x00000001L
#define SC_DEBUG_8__sample_mask_MASK                       0x0000001eL
#define SC_DEBUG_8__sample_y_MASK                          0x00000060L
#define SC_DEBUG_8__sample_x_MASK                          0x00000180L
#define SC_DEBUG_8__sample_send_MASK                       0x00000200L
#define SC_DEBUG_8__sample_send                            0x00000200L
#define SC_DEBUG_8__next_cycle_MASK                        0x00000c00L
#define SC_DEBUG_8__ez_sample_ff_full_MASK                 0x00001000L
#define SC_DEBUG_8__ez_sample_ff_full                      0x00001000L
#define SC_DEBUG_8__rb_sc_samp_rtr_MASK                    0x00002000L
#define SC_DEBUG_8__rb_sc_samp_rtr                         0x00002000L
#define SC_DEBUG_8__num_samples_MASK                       0x0000c000L
#define SC_DEBUG_8__last_quad_of_tile_MASK                 0x00010000L
#define SC_DEBUG_8__last_quad_of_tile                      0x00010000L
#define SC_DEBUG_8__last_quad_of_prim_MASK                 0x00020000L
#define SC_DEBUG_8__last_quad_of_prim                      0x00020000L
#define SC_DEBUG_8__first_quad_of_prim_MASK                0x00040000L
#define SC_DEBUG_8__first_quad_of_prim                     0x00040000L
#define SC_DEBUG_8__sample_we_MASK                         0x00080000L
#define SC_DEBUG_8__sample_we                              0x00080000L
#define SC_DEBUG_8__fpos_MASK                              0x00100000L
#define SC_DEBUG_8__fpos                                   0x00100000L
#define SC_DEBUG_8__event_id_MASK                          0x03e00000L
#define SC_DEBUG_8__event_flag_MASK                        0x04000000L
#define SC_DEBUG_8__event_flag                             0x04000000L
#define SC_DEBUG_8__fw_prim_data_valid_MASK                0x08000000L
#define SC_DEBUG_8__fw_prim_data_valid                     0x08000000L
#define SC_DEBUG_8__trigger_MASK                           0x80000000L
#define SC_DEBUG_8__trigger                                0x80000000L

// SC_DEBUG_9
#define SC_DEBUG_9__rb_sc_send_MASK                        0x00000001L
#define SC_DEBUG_9__rb_sc_send                             0x00000001L
#define SC_DEBUG_9__rb_sc_ez_mask_MASK                     0x0000001eL
#define SC_DEBUG_9__fifo_data_ready_MASK                   0x00000020L
#define SC_DEBUG_9__fifo_data_ready                        0x00000020L
#define SC_DEBUG_9__early_z_enable_MASK                    0x00000040L
#define SC_DEBUG_9__early_z_enable                         0x00000040L
#define SC_DEBUG_9__mask_state_MASK                        0x00000180L
#define SC_DEBUG_9__next_ez_mask_MASK                      0x01fffe00L
#define SC_DEBUG_9__mask_ready_MASK                        0x02000000L
#define SC_DEBUG_9__mask_ready                             0x02000000L
#define SC_DEBUG_9__drop_sample_MASK                       0x04000000L
#define SC_DEBUG_9__drop_sample                            0x04000000L
#define SC_DEBUG_9__fetch_new_sample_data_MASK             0x08000000L
#define SC_DEBUG_9__fetch_new_sample_data                  0x08000000L
#define SC_DEBUG_9__fetch_new_ez_sample_mask_MASK          0x10000000L
#define SC_DEBUG_9__fetch_new_ez_sample_mask               0x10000000L
#define SC_DEBUG_9__pkr_fetch_new_sample_data_MASK         0x20000000L
#define SC_DEBUG_9__pkr_fetch_new_sample_data              0x20000000L
#define SC_DEBUG_9__pkr_fetch_new_prim_data_MASK           0x40000000L
#define SC_DEBUG_9__pkr_fetch_new_prim_data                0x40000000L
#define SC_DEBUG_9__trigger_MASK                           0x80000000L
#define SC_DEBUG_9__trigger                                0x80000000L

// SC_DEBUG_10
#define SC_DEBUG_10__combined_sample_mask_MASK             0x0000ffffL
#define SC_DEBUG_10__trigger_MASK                          0x80000000L
#define SC_DEBUG_10__trigger                               0x80000000L

// SC_DEBUG_11
#define SC_DEBUG_11__ez_sample_data_ready_MASK             0x00000001L
#define SC_DEBUG_11__ez_sample_data_ready                  0x00000001L
#define SC_DEBUG_11__pkr_fetch_new_sample_data_MASK        0x00000002L
#define SC_DEBUG_11__pkr_fetch_new_sample_data             0x00000002L
#define SC_DEBUG_11__ez_prim_data_ready_MASK               0x00000004L
#define SC_DEBUG_11__ez_prim_data_ready                    0x00000004L
#define SC_DEBUG_11__pkr_fetch_new_prim_data_MASK          0x00000008L
#define SC_DEBUG_11__pkr_fetch_new_prim_data               0x00000008L
#define SC_DEBUG_11__iterator_input_fz_MASK                0x00000010L
#define SC_DEBUG_11__iterator_input_fz                     0x00000010L
#define SC_DEBUG_11__packer_send_quads_MASK                0x00000020L
#define SC_DEBUG_11__packer_send_quads                     0x00000020L
#define SC_DEBUG_11__packer_send_cmd_MASK                  0x00000040L
#define SC_DEBUG_11__packer_send_cmd                       0x00000040L
#define SC_DEBUG_11__packer_send_event_MASK                0x00000080L
#define SC_DEBUG_11__packer_send_event                     0x00000080L
#define SC_DEBUG_11__next_state_MASK                       0x00000700L
#define SC_DEBUG_11__state_MASK                            0x00003800L
#define SC_DEBUG_11__stall_MASK                            0x00004000L
#define SC_DEBUG_11__stall                                 0x00004000L
#define SC_DEBUG_11__trigger_MASK                          0x80000000L
#define SC_DEBUG_11__trigger                               0x80000000L

// SC_DEBUG_12
#define SC_DEBUG_12__SQ_iterator_free_buff_MASK            0x00000001L
#define SC_DEBUG_12__SQ_iterator_free_buff                 0x00000001L
#define SC_DEBUG_12__event_id_MASK                         0x0000003eL
#define SC_DEBUG_12__event_flag_MASK                       0x00000040L
#define SC_DEBUG_12__event_flag                            0x00000040L
#define SC_DEBUG_12__itercmdfifo_busy_nc_dly_MASK          0x00000080L
#define SC_DEBUG_12__itercmdfifo_busy_nc_dly               0x00000080L
#define SC_DEBUG_12__itercmdfifo_full_MASK                 0x00000100L
#define SC_DEBUG_12__itercmdfifo_full                      0x00000100L
#define SC_DEBUG_12__itercmdfifo_empty_MASK                0x00000200L
#define SC_DEBUG_12__itercmdfifo_empty                     0x00000200L
#define SC_DEBUG_12__iter_ds_one_clk_command_MASK          0x00000400L
#define SC_DEBUG_12__iter_ds_one_clk_command               0x00000400L
#define SC_DEBUG_12__iter_ds_end_of_prim0_MASK             0x00000800L
#define SC_DEBUG_12__iter_ds_end_of_prim0                  0x00000800L
#define SC_DEBUG_12__iter_ds_end_of_vector_MASK            0x00001000L
#define SC_DEBUG_12__iter_ds_end_of_vector                 0x00001000L
#define SC_DEBUG_12__iter_qdhit0_MASK                      0x00002000L
#define SC_DEBUG_12__iter_qdhit0                           0x00002000L
#define SC_DEBUG_12__bc_use_centers_reg_MASK               0x00004000L
#define SC_DEBUG_12__bc_use_centers_reg                    0x00004000L
#define SC_DEBUG_12__bc_output_xy_reg_MASK                 0x00008000L
#define SC_DEBUG_12__bc_output_xy_reg                      0x00008000L
#define SC_DEBUG_12__iter_phase_out_MASK                   0x00030000L
#define SC_DEBUG_12__iter_phase_reg_MASK                   0x000c0000L
#define SC_DEBUG_12__iterator_SP_valid_MASK                0x00100000L
#define SC_DEBUG_12__iterator_SP_valid                     0x00100000L
#define SC_DEBUG_12__eopv_reg_MASK                         0x00200000L
#define SC_DEBUG_12__eopv_reg                              0x00200000L
#define SC_DEBUG_12__one_clk_cmd_reg_MASK                  0x00400000L
#define SC_DEBUG_12__one_clk_cmd_reg                       0x00400000L
#define SC_DEBUG_12__iter_dx_end_of_prim_MASK              0x00800000L
#define SC_DEBUG_12__iter_dx_end_of_prim                   0x00800000L
#define SC_DEBUG_12__trigger_MASK                          0x80000000L
#define SC_DEBUG_12__trigger                               0x80000000L

// GFX_COPY_STATE
#define GFX_COPY_STATE__SRC_STATE_ID_MASK                  0x00000001L
#define GFX_COPY_STATE__SRC_STATE_ID                       0x00000001L

// VGT_DRAW_INITIATOR
#define VGT_DRAW_INITIATOR__PRIM_TYPE_MASK                 0x0000003fL
#define VGT_DRAW_INITIATOR__SOURCE_SELECT_MASK             0x000000c0L
#define VGT_DRAW_INITIATOR__INDEX_SIZE_MASK                0x00000800L
#define VGT_DRAW_INITIATOR__INDEX_SIZE                     0x00000800L
#define VGT_DRAW_INITIATOR__NOT_EOP_MASK                   0x00001000L
#define VGT_DRAW_INITIATOR__NOT_EOP                        0x00001000L
#define VGT_DRAW_INITIATOR__SMALL_INDEX_MASK               0x00002000L
#define VGT_DRAW_INITIATOR__SMALL_INDEX                    0x00002000L
#define VGT_DRAW_INITIATOR__PRE_FETCH_CULL_ENABLE_MASK     0x00004000L
#define VGT_DRAW_INITIATOR__PRE_FETCH_CULL_ENABLE          0x00004000L
#define VGT_DRAW_INITIATOR__GRP_CULL_ENABLE_MASK           0x00008000L
#define VGT_DRAW_INITIATOR__GRP_CULL_ENABLE                0x00008000L
#define VGT_DRAW_INITIATOR__NUM_INDICES_MASK               0xffff0000L

// VGT_EVENT_INITIATOR
#define VGT_EVENT_INITIATOR__EVENT_TYPE_MASK               0x0000003fL

// VGT_DMA_BASE
#define VGT_DMA_BASE__BASE_ADDR_MASK                       0xffffffffL

// VGT_DMA_SIZE
#define VGT_DMA_SIZE__NUM_WORDS_MASK                       0x00ffffffL
#define VGT_DMA_SIZE__SWAP_MODE_MASK                       0xc0000000L

// VGT_BIN_BASE
#define VGT_BIN_BASE__BIN_BASE_ADDR_MASK                   0xffffffffL

// VGT_BIN_SIZE
#define VGT_BIN_SIZE__NUM_WORDS_MASK                       0x00ffffffL

// VGT_CURRENT_BIN_ID_MIN
#define VGT_CURRENT_BIN_ID_MIN__COLUMN_MASK                0x00000007L
#define VGT_CURRENT_BIN_ID_MIN__ROW_MASK                   0x00000038L
#define VGT_CURRENT_BIN_ID_MIN__GUARD_BAND_MASK            0x000001c0L

// VGT_CURRENT_BIN_ID_MAX
#define VGT_CURRENT_BIN_ID_MAX__COLUMN_MASK                0x00000007L
#define VGT_CURRENT_BIN_ID_MAX__ROW_MASK                   0x00000038L
#define VGT_CURRENT_BIN_ID_MAX__GUARD_BAND_MASK            0x000001c0L

// VGT_IMMED_DATA
#define VGT_IMMED_DATA__DATA_MASK                          0xffffffffL

// VGT_MAX_VTX_INDX
#define VGT_MAX_VTX_INDX__MAX_INDX_MASK                    0x00ffffffL

// VGT_MIN_VTX_INDX
#define VGT_MIN_VTX_INDX__MIN_INDX_MASK                    0x00ffffffL

// VGT_INDX_OFFSET
#define VGT_INDX_OFFSET__INDX_OFFSET_MASK                  0x00ffffffL

// VGT_VERTEX_REUSE_BLOCK_CNTL
#define VGT_VERTEX_REUSE_BLOCK_CNTL__VTX_REUSE_DEPTH_MASK  0x00000007L

// VGT_OUT_DEALLOC_CNTL
#define VGT_OUT_DEALLOC_CNTL__DEALLOC_DIST_MASK            0x00000003L

// VGT_MULTI_PRIM_IB_RESET_INDX
#define VGT_MULTI_PRIM_IB_RESET_INDX__RESET_INDX_MASK      0x00ffffffL

// VGT_ENHANCE
#define VGT_ENHANCE__MISC_MASK                             0x0000ffffL

// VGT_VTX_VECT_EJECT_REG
#define VGT_VTX_VECT_EJECT_REG__PRIM_COUNT_MASK            0x0000001fL

// VGT_LAST_COPY_STATE
#define VGT_LAST_COPY_STATE__SRC_STATE_ID_MASK             0x00000001L
#define VGT_LAST_COPY_STATE__SRC_STATE_ID                  0x00000001L
#define VGT_LAST_COPY_STATE__DST_STATE_ID_MASK             0x00010000L
#define VGT_LAST_COPY_STATE__DST_STATE_ID                  0x00010000L

// VGT_DEBUG_CNTL
#define VGT_DEBUG_CNTL__VGT_DEBUG_INDX_MASK                0x0000001fL

// VGT_DEBUG_DATA
#define VGT_DEBUG_DATA__DATA_MASK                          0xffffffffL

// VGT_CNTL_STATUS
#define VGT_CNTL_STATUS__VGT_BUSY_MASK                     0x00000001L
#define VGT_CNTL_STATUS__VGT_BUSY                          0x00000001L
#define VGT_CNTL_STATUS__VGT_DMA_BUSY_MASK                 0x00000002L
#define VGT_CNTL_STATUS__VGT_DMA_BUSY                      0x00000002L
#define VGT_CNTL_STATUS__VGT_DMA_REQ_BUSY_MASK             0x00000004L
#define VGT_CNTL_STATUS__VGT_DMA_REQ_BUSY                  0x00000004L
#define VGT_CNTL_STATUS__VGT_GRP_BUSY_MASK                 0x00000008L
#define VGT_CNTL_STATUS__VGT_GRP_BUSY                      0x00000008L
#define VGT_CNTL_STATUS__VGT_VR_BUSY_MASK                  0x00000010L
#define VGT_CNTL_STATUS__VGT_VR_BUSY                       0x00000010L
#define VGT_CNTL_STATUS__VGT_BIN_BUSY_MASK                 0x00000020L
#define VGT_CNTL_STATUS__VGT_BIN_BUSY                      0x00000020L
#define VGT_CNTL_STATUS__VGT_PT_BUSY_MASK                  0x00000040L
#define VGT_CNTL_STATUS__VGT_PT_BUSY                       0x00000040L
#define VGT_CNTL_STATUS__VGT_OUT_BUSY_MASK                 0x00000080L
#define VGT_CNTL_STATUS__VGT_OUT_BUSY                      0x00000080L
#define VGT_CNTL_STATUS__VGT_OUT_INDX_BUSY_MASK            0x00000100L
#define VGT_CNTL_STATUS__VGT_OUT_INDX_BUSY                 0x00000100L

// VGT_DEBUG_REG0
#define VGT_DEBUG_REG0__te_grp_busy_MASK                   0x00000001L
#define VGT_DEBUG_REG0__te_grp_busy                        0x00000001L
#define VGT_DEBUG_REG0__pt_grp_busy_MASK                   0x00000002L
#define VGT_DEBUG_REG0__pt_grp_busy                        0x00000002L
#define VGT_DEBUG_REG0__vr_grp_busy_MASK                   0x00000004L
#define VGT_DEBUG_REG0__vr_grp_busy                        0x00000004L
#define VGT_DEBUG_REG0__dma_request_busy_MASK              0x00000008L
#define VGT_DEBUG_REG0__dma_request_busy                   0x00000008L
#define VGT_DEBUG_REG0__out_busy_MASK                      0x00000010L
#define VGT_DEBUG_REG0__out_busy                           0x00000010L
#define VGT_DEBUG_REG0__grp_backend_busy_MASK              0x00000020L
#define VGT_DEBUG_REG0__grp_backend_busy                   0x00000020L
#define VGT_DEBUG_REG0__grp_busy_MASK                      0x00000040L
#define VGT_DEBUG_REG0__grp_busy                           0x00000040L
#define VGT_DEBUG_REG0__dma_busy_MASK                      0x00000080L
#define VGT_DEBUG_REG0__dma_busy                           0x00000080L
#define VGT_DEBUG_REG0__rbiu_dma_request_busy_MASK         0x00000100L
#define VGT_DEBUG_REG0__rbiu_dma_request_busy              0x00000100L
#define VGT_DEBUG_REG0__rbiu_busy_MASK                     0x00000200L
#define VGT_DEBUG_REG0__rbiu_busy                          0x00000200L
#define VGT_DEBUG_REG0__vgt_no_dma_busy_extended_MASK      0x00000400L
#define VGT_DEBUG_REG0__vgt_no_dma_busy_extended           0x00000400L
#define VGT_DEBUG_REG0__vgt_no_dma_busy_MASK               0x00000800L
#define VGT_DEBUG_REG0__vgt_no_dma_busy                    0x00000800L
#define VGT_DEBUG_REG0__vgt_busy_extended_MASK             0x00001000L
#define VGT_DEBUG_REG0__vgt_busy_extended                  0x00001000L
#define VGT_DEBUG_REG0__vgt_busy_MASK                      0x00002000L
#define VGT_DEBUG_REG0__vgt_busy                           0x00002000L
#define VGT_DEBUG_REG0__rbbm_skid_fifo_busy_out_MASK       0x00004000L
#define VGT_DEBUG_REG0__rbbm_skid_fifo_busy_out            0x00004000L
#define VGT_DEBUG_REG0__VGT_RBBM_no_dma_busy_MASK          0x00008000L
#define VGT_DEBUG_REG0__VGT_RBBM_no_dma_busy               0x00008000L
#define VGT_DEBUG_REG0__VGT_RBBM_busy_MASK                 0x00010000L
#define VGT_DEBUG_REG0__VGT_RBBM_busy                      0x00010000L

// VGT_DEBUG_REG1
#define VGT_DEBUG_REG1__out_te_data_read_MASK              0x00000001L
#define VGT_DEBUG_REG1__out_te_data_read                   0x00000001L
#define VGT_DEBUG_REG1__te_out_data_valid_MASK             0x00000002L
#define VGT_DEBUG_REG1__te_out_data_valid                  0x00000002L
#define VGT_DEBUG_REG1__out_pt_prim_read_MASK              0x00000004L
#define VGT_DEBUG_REG1__out_pt_prim_read                   0x00000004L
#define VGT_DEBUG_REG1__pt_out_prim_valid_MASK             0x00000008L
#define VGT_DEBUG_REG1__pt_out_prim_valid                  0x00000008L
#define VGT_DEBUG_REG1__out_pt_data_read_MASK              0x00000010L
#define VGT_DEBUG_REG1__out_pt_data_read                   0x00000010L
#define VGT_DEBUG_REG1__pt_out_indx_valid_MASK             0x00000020L
#define VGT_DEBUG_REG1__pt_out_indx_valid                  0x00000020L
#define VGT_DEBUG_REG1__out_vr_prim_read_MASK              0x00000040L
#define VGT_DEBUG_REG1__out_vr_prim_read                   0x00000040L
#define VGT_DEBUG_REG1__vr_out_prim_valid_MASK             0x00000080L
#define VGT_DEBUG_REG1__vr_out_prim_valid                  0x00000080L
#define VGT_DEBUG_REG1__out_vr_indx_read_MASK              0x00000100L
#define VGT_DEBUG_REG1__out_vr_indx_read                   0x00000100L
#define VGT_DEBUG_REG1__vr_out_indx_valid_MASK             0x00000200L
#define VGT_DEBUG_REG1__vr_out_indx_valid                  0x00000200L
#define VGT_DEBUG_REG1__te_grp_read_MASK                   0x00000400L
#define VGT_DEBUG_REG1__te_grp_read                        0x00000400L
#define VGT_DEBUG_REG1__grp_te_valid_MASK                  0x00000800L
#define VGT_DEBUG_REG1__grp_te_valid                       0x00000800L
#define VGT_DEBUG_REG1__pt_grp_read_MASK                   0x00001000L
#define VGT_DEBUG_REG1__pt_grp_read                        0x00001000L
#define VGT_DEBUG_REG1__grp_pt_valid_MASK                  0x00002000L
#define VGT_DEBUG_REG1__grp_pt_valid                       0x00002000L
#define VGT_DEBUG_REG1__vr_grp_read_MASK                   0x00004000L
#define VGT_DEBUG_REG1__vr_grp_read                        0x00004000L
#define VGT_DEBUG_REG1__grp_vr_valid_MASK                  0x00008000L
#define VGT_DEBUG_REG1__grp_vr_valid                       0x00008000L
#define VGT_DEBUG_REG1__grp_dma_read_MASK                  0x00010000L
#define VGT_DEBUG_REG1__grp_dma_read                       0x00010000L
#define VGT_DEBUG_REG1__dma_grp_valid_MASK                 0x00020000L
#define VGT_DEBUG_REG1__dma_grp_valid                      0x00020000L
#define VGT_DEBUG_REG1__grp_rbiu_di_read_MASK              0x00040000L
#define VGT_DEBUG_REG1__grp_rbiu_di_read                   0x00040000L
#define VGT_DEBUG_REG1__rbiu_grp_di_valid_MASK             0x00080000L
#define VGT_DEBUG_REG1__rbiu_grp_di_valid                  0x00080000L
#define VGT_DEBUG_REG1__MH_VGT_rtr_MASK                    0x00100000L
#define VGT_DEBUG_REG1__MH_VGT_rtr                         0x00100000L
#define VGT_DEBUG_REG1__VGT_MH_send_MASK                   0x00200000L
#define VGT_DEBUG_REG1__VGT_MH_send                        0x00200000L
#define VGT_DEBUG_REG1__PA_VGT_clip_s_rtr_MASK             0x00400000L
#define VGT_DEBUG_REG1__PA_VGT_clip_s_rtr                  0x00400000L
#define VGT_DEBUG_REG1__VGT_PA_clip_s_send_MASK            0x00800000L
#define VGT_DEBUG_REG1__VGT_PA_clip_s_send                 0x00800000L
#define VGT_DEBUG_REG1__PA_VGT_clip_p_rtr_MASK             0x01000000L
#define VGT_DEBUG_REG1__PA_VGT_clip_p_rtr                  0x01000000L
#define VGT_DEBUG_REG1__VGT_PA_clip_p_send_MASK            0x02000000L
#define VGT_DEBUG_REG1__VGT_PA_clip_p_send                 0x02000000L
#define VGT_DEBUG_REG1__PA_VGT_clip_v_rtr_MASK             0x04000000L
#define VGT_DEBUG_REG1__PA_VGT_clip_v_rtr                  0x04000000L
#define VGT_DEBUG_REG1__VGT_PA_clip_v_send_MASK            0x08000000L
#define VGT_DEBUG_REG1__VGT_PA_clip_v_send                 0x08000000L
#define VGT_DEBUG_REG1__SQ_VGT_rtr_MASK                    0x10000000L
#define VGT_DEBUG_REG1__SQ_VGT_rtr                         0x10000000L
#define VGT_DEBUG_REG1__VGT_SQ_send_MASK                   0x20000000L
#define VGT_DEBUG_REG1__VGT_SQ_send                        0x20000000L
#define VGT_DEBUG_REG1__mh_vgt_tag_7_q_MASK                0x40000000L
#define VGT_DEBUG_REG1__mh_vgt_tag_7_q                     0x40000000L

// VGT_DEBUG_REG3
#define VGT_DEBUG_REG3__vgt_clk_en_MASK                    0x00000001L
#define VGT_DEBUG_REG3__vgt_clk_en                         0x00000001L
#define VGT_DEBUG_REG3__reg_fifos_clk_en_MASK              0x00000002L
#define VGT_DEBUG_REG3__reg_fifos_clk_en                   0x00000002L

// VGT_DEBUG_REG6
#define VGT_DEBUG_REG6__shifter_byte_count_q_MASK          0x0000001fL
#define VGT_DEBUG_REG6__right_word_indx_q_MASK             0x000003e0L
#define VGT_DEBUG_REG6__input_data_valid_MASK              0x00000400L
#define VGT_DEBUG_REG6__input_data_valid                   0x00000400L
#define VGT_DEBUG_REG6__input_data_xfer_MASK               0x00000800L
#define VGT_DEBUG_REG6__input_data_xfer                    0x00000800L
#define VGT_DEBUG_REG6__next_shift_is_vect_1_q_MASK        0x00001000L
#define VGT_DEBUG_REG6__next_shift_is_vect_1_q             0x00001000L
#define VGT_DEBUG_REG6__next_shift_is_vect_1_d_MASK        0x00002000L
#define VGT_DEBUG_REG6__next_shift_is_vect_1_d             0x00002000L
#define VGT_DEBUG_REG6__next_shift_is_vect_1_pre_d_MASK    0x00004000L
#define VGT_DEBUG_REG6__next_shift_is_vect_1_pre_d         0x00004000L
#define VGT_DEBUG_REG6__space_avail_from_shift_MASK        0x00008000L
#define VGT_DEBUG_REG6__space_avail_from_shift             0x00008000L
#define VGT_DEBUG_REG6__shifter_first_load_MASK            0x00010000L
#define VGT_DEBUG_REG6__shifter_first_load                 0x00010000L
#define VGT_DEBUG_REG6__di_state_sel_q_MASK                0x00020000L
#define VGT_DEBUG_REG6__di_state_sel_q                     0x00020000L
#define VGT_DEBUG_REG6__shifter_waiting_for_first_load_q_MASK 0x00040000L
#define VGT_DEBUG_REG6__shifter_waiting_for_first_load_q   0x00040000L
#define VGT_DEBUG_REG6__di_first_group_flag_q_MASK         0x00080000L
#define VGT_DEBUG_REG6__di_first_group_flag_q              0x00080000L
#define VGT_DEBUG_REG6__di_event_flag_q_MASK               0x00100000L
#define VGT_DEBUG_REG6__di_event_flag_q                    0x00100000L
#define VGT_DEBUG_REG6__read_draw_initiator_MASK           0x00200000L
#define VGT_DEBUG_REG6__read_draw_initiator                0x00200000L
#define VGT_DEBUG_REG6__loading_di_requires_shifter_MASK   0x00400000L
#define VGT_DEBUG_REG6__loading_di_requires_shifter        0x00400000L
#define VGT_DEBUG_REG6__last_shift_of_packet_MASK          0x00800000L
#define VGT_DEBUG_REG6__last_shift_of_packet               0x00800000L
#define VGT_DEBUG_REG6__last_decr_of_packet_MASK           0x01000000L
#define VGT_DEBUG_REG6__last_decr_of_packet                0x01000000L
#define VGT_DEBUG_REG6__extract_vector_MASK                0x02000000L
#define VGT_DEBUG_REG6__extract_vector                     0x02000000L
#define VGT_DEBUG_REG6__shift_vect_rtr_MASK                0x04000000L
#define VGT_DEBUG_REG6__shift_vect_rtr                     0x04000000L
#define VGT_DEBUG_REG6__destination_rtr_MASK               0x08000000L
#define VGT_DEBUG_REG6__destination_rtr                    0x08000000L
#define VGT_DEBUG_REG6__grp_trigger_MASK                   0x10000000L
#define VGT_DEBUG_REG6__grp_trigger                        0x10000000L

// VGT_DEBUG_REG7
#define VGT_DEBUG_REG7__di_index_counter_q_MASK            0x0000ffffL
#define VGT_DEBUG_REG7__shift_amount_no_extract_MASK       0x000f0000L
#define VGT_DEBUG_REG7__shift_amount_extract_MASK          0x00f00000L
#define VGT_DEBUG_REG7__di_prim_type_q_MASK                0x3f000000L
#define VGT_DEBUG_REG7__current_source_sel_MASK            0xc0000000L

// VGT_DEBUG_REG8
#define VGT_DEBUG_REG8__current_source_sel_MASK            0x00000003L
#define VGT_DEBUG_REG8__left_word_indx_q_MASK              0x0000007cL
#define VGT_DEBUG_REG8__input_data_cnt_MASK                0x00000f80L
#define VGT_DEBUG_REG8__input_data_lsw_MASK                0x0001f000L
#define VGT_DEBUG_REG8__input_data_msw_MASK                0x003e0000L
#define VGT_DEBUG_REG8__next_small_stride_shift_limit_q_MASK 0x07c00000L
#define VGT_DEBUG_REG8__current_small_stride_shift_limit_q_MASK 0xf8000000L

// VGT_DEBUG_REG9
#define VGT_DEBUG_REG9__next_stride_q_MASK                 0x0000001fL
#define VGT_DEBUG_REG9__next_stride_d_MASK                 0x000003e0L
#define VGT_DEBUG_REG9__current_shift_q_MASK               0x00007c00L
#define VGT_DEBUG_REG9__current_shift_d_MASK               0x000f8000L
#define VGT_DEBUG_REG9__current_stride_q_MASK              0x01f00000L
#define VGT_DEBUG_REG9__current_stride_d_MASK              0x3e000000L
#define VGT_DEBUG_REG9__grp_trigger_MASK                   0x40000000L
#define VGT_DEBUG_REG9__grp_trigger                        0x40000000L

// VGT_DEBUG_REG10
#define VGT_DEBUG_REG10__temp_derived_di_prim_type_t0_MASK 0x00000001L
#define VGT_DEBUG_REG10__temp_derived_di_prim_type_t0      0x00000001L
#define VGT_DEBUG_REG10__temp_derived_di_small_index_t0_MASK 0x00000002L
#define VGT_DEBUG_REG10__temp_derived_di_small_index_t0    0x00000002L
#define VGT_DEBUG_REG10__temp_derived_di_cull_enable_t0_MASK 0x00000004L
#define VGT_DEBUG_REG10__temp_derived_di_cull_enable_t0    0x00000004L
#define VGT_DEBUG_REG10__temp_derived_di_pre_fetch_cull_enable_t0_MASK 0x00000008L
#define VGT_DEBUG_REG10__temp_derived_di_pre_fetch_cull_enable_t0 0x00000008L
#define VGT_DEBUG_REG10__di_state_sel_q_MASK               0x00000010L
#define VGT_DEBUG_REG10__di_state_sel_q                    0x00000010L
#define VGT_DEBUG_REG10__last_decr_of_packet_MASK          0x00000020L
#define VGT_DEBUG_REG10__last_decr_of_packet               0x00000020L
#define VGT_DEBUG_REG10__bin_valid_MASK                    0x00000040L
#define VGT_DEBUG_REG10__bin_valid                         0x00000040L
#define VGT_DEBUG_REG10__read_block_MASK                   0x00000080L
#define VGT_DEBUG_REG10__read_block                        0x00000080L
#define VGT_DEBUG_REG10__grp_bgrp_last_bit_read_MASK       0x00000100L
#define VGT_DEBUG_REG10__grp_bgrp_last_bit_read            0x00000100L
#define VGT_DEBUG_REG10__last_bit_enable_q_MASK            0x00000200L
#define VGT_DEBUG_REG10__last_bit_enable_q                 0x00000200L
#define VGT_DEBUG_REG10__last_bit_end_di_q_MASK            0x00000400L
#define VGT_DEBUG_REG10__last_bit_end_di_q                 0x00000400L
#define VGT_DEBUG_REG10__selected_data_MASK                0x0007f800L
#define VGT_DEBUG_REG10__mask_input_data_MASK              0x07f80000L
#define VGT_DEBUG_REG10__gap_q_MASK                        0x08000000L
#define VGT_DEBUG_REG10__gap_q                             0x08000000L
#define VGT_DEBUG_REG10__temp_mini_reset_z_MASK            0x10000000L
#define VGT_DEBUG_REG10__temp_mini_reset_z                 0x10000000L
#define VGT_DEBUG_REG10__temp_mini_reset_y_MASK            0x20000000L
#define VGT_DEBUG_REG10__temp_mini_reset_y                 0x20000000L
#define VGT_DEBUG_REG10__temp_mini_reset_x_MASK            0x40000000L
#define VGT_DEBUG_REG10__temp_mini_reset_x                 0x40000000L
#define VGT_DEBUG_REG10__grp_trigger_MASK                  0x80000000L
#define VGT_DEBUG_REG10__grp_trigger                       0x80000000L

// VGT_DEBUG_REG12
#define VGT_DEBUG_REG12__shifter_byte_count_q_MASK         0x0000001fL
#define VGT_DEBUG_REG12__right_word_indx_q_MASK            0x000003e0L
#define VGT_DEBUG_REG12__input_data_valid_MASK             0x00000400L
#define VGT_DEBUG_REG12__input_data_valid                  0x00000400L
#define VGT_DEBUG_REG12__input_data_xfer_MASK              0x00000800L
#define VGT_DEBUG_REG12__input_data_xfer                   0x00000800L
#define VGT_DEBUG_REG12__next_shift_is_vect_1_q_MASK       0x00001000L
#define VGT_DEBUG_REG12__next_shift_is_vect_1_q            0x00001000L
#define VGT_DEBUG_REG12__next_shift_is_vect_1_d_MASK       0x00002000L
#define VGT_DEBUG_REG12__next_shift_is_vect_1_d            0x00002000L
#define VGT_DEBUG_REG12__next_shift_is_vect_1_pre_d_MASK   0x00004000L
#define VGT_DEBUG_REG12__next_shift_is_vect_1_pre_d        0x00004000L
#define VGT_DEBUG_REG12__space_avail_from_shift_MASK       0x00008000L
#define VGT_DEBUG_REG12__space_avail_from_shift            0x00008000L
#define VGT_DEBUG_REG12__shifter_first_load_MASK           0x00010000L
#define VGT_DEBUG_REG12__shifter_first_load                0x00010000L
#define VGT_DEBUG_REG12__di_state_sel_q_MASK               0x00020000L
#define VGT_DEBUG_REG12__di_state_sel_q                    0x00020000L
#define VGT_DEBUG_REG12__shifter_waiting_for_first_load_q_MASK 0x00040000L
#define VGT_DEBUG_REG12__shifter_waiting_for_first_load_q  0x00040000L
#define VGT_DEBUG_REG12__di_first_group_flag_q_MASK        0x00080000L
#define VGT_DEBUG_REG12__di_first_group_flag_q             0x00080000L
#define VGT_DEBUG_REG12__di_event_flag_q_MASK              0x00100000L
#define VGT_DEBUG_REG12__di_event_flag_q                   0x00100000L
#define VGT_DEBUG_REG12__read_draw_initiator_MASK          0x00200000L
#define VGT_DEBUG_REG12__read_draw_initiator               0x00200000L
#define VGT_DEBUG_REG12__loading_di_requires_shifter_MASK  0x00400000L
#define VGT_DEBUG_REG12__loading_di_requires_shifter       0x00400000L
#define VGT_DEBUG_REG12__last_shift_of_packet_MASK         0x00800000L
#define VGT_DEBUG_REG12__last_shift_of_packet              0x00800000L
#define VGT_DEBUG_REG12__last_decr_of_packet_MASK          0x01000000L
#define VGT_DEBUG_REG12__last_decr_of_packet               0x01000000L
#define VGT_DEBUG_REG12__extract_vector_MASK               0x02000000L
#define VGT_DEBUG_REG12__extract_vector                    0x02000000L
#define VGT_DEBUG_REG12__shift_vect_rtr_MASK               0x04000000L
#define VGT_DEBUG_REG12__shift_vect_rtr                    0x04000000L
#define VGT_DEBUG_REG12__destination_rtr_MASK              0x08000000L
#define VGT_DEBUG_REG12__destination_rtr                   0x08000000L
#define VGT_DEBUG_REG12__bgrp_trigger_MASK                 0x10000000L
#define VGT_DEBUG_REG12__bgrp_trigger                      0x10000000L

// VGT_DEBUG_REG13
#define VGT_DEBUG_REG13__di_index_counter_q_MASK           0x0000ffffL
#define VGT_DEBUG_REG13__shift_amount_no_extract_MASK      0x000f0000L
#define VGT_DEBUG_REG13__shift_amount_extract_MASK         0x00f00000L
#define VGT_DEBUG_REG13__di_prim_type_q_MASK               0x3f000000L
#define VGT_DEBUG_REG13__current_source_sel_MASK           0xc0000000L

// VGT_DEBUG_REG14
#define VGT_DEBUG_REG14__current_source_sel_MASK           0x00000003L
#define VGT_DEBUG_REG14__left_word_indx_q_MASK             0x0000007cL
#define VGT_DEBUG_REG14__input_data_cnt_MASK               0x00000f80L
#define VGT_DEBUG_REG14__input_data_lsw_MASK               0x0001f000L
#define VGT_DEBUG_REG14__input_data_msw_MASK               0x003e0000L
#define VGT_DEBUG_REG14__next_small_stride_shift_limit_q_MASK 0x07c00000L
#define VGT_DEBUG_REG14__current_small_stride_shift_limit_q_MASK 0xf8000000L

// VGT_DEBUG_REG15
#define VGT_DEBUG_REG15__next_stride_q_MASK                0x0000001fL
#define VGT_DEBUG_REG15__next_stride_d_MASK                0x000003e0L
#define VGT_DEBUG_REG15__current_shift_q_MASK              0x00007c00L
#define VGT_DEBUG_REG15__current_shift_d_MASK              0x000f8000L
#define VGT_DEBUG_REG15__current_stride_q_MASK             0x01f00000L
#define VGT_DEBUG_REG15__current_stride_d_MASK             0x3e000000L
#define VGT_DEBUG_REG15__bgrp_trigger_MASK                 0x40000000L
#define VGT_DEBUG_REG15__bgrp_trigger                      0x40000000L

// VGT_DEBUG_REG16
#define VGT_DEBUG_REG16__bgrp_cull_fetch_fifo_full_MASK    0x00000001L
#define VGT_DEBUG_REG16__bgrp_cull_fetch_fifo_full         0x00000001L
#define VGT_DEBUG_REG16__bgrp_cull_fetch_fifo_empty_MASK   0x00000002L
#define VGT_DEBUG_REG16__bgrp_cull_fetch_fifo_empty        0x00000002L
#define VGT_DEBUG_REG16__dma_bgrp_cull_fetch_read_MASK     0x00000004L
#define VGT_DEBUG_REG16__dma_bgrp_cull_fetch_read          0x00000004L
#define VGT_DEBUG_REG16__bgrp_cull_fetch_fifo_we_MASK      0x00000008L
#define VGT_DEBUG_REG16__bgrp_cull_fetch_fifo_we           0x00000008L
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_full_MASK     0x00000010L
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_full          0x00000010L
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_empty_MASK    0x00000020L
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_empty         0x00000020L
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_re_q_MASK     0x00000040L
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_re_q          0x00000040L
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_we_MASK       0x00000080L
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_we            0x00000080L
#define VGT_DEBUG_REG16__bgrp_dma_mask_kill_MASK           0x00000100L
#define VGT_DEBUG_REG16__bgrp_dma_mask_kill                0x00000100L
#define VGT_DEBUG_REG16__bgrp_grp_bin_valid_MASK           0x00000200L
#define VGT_DEBUG_REG16__bgrp_grp_bin_valid                0x00000200L
#define VGT_DEBUG_REG16__rst_last_bit_MASK                 0x00000400L
#define VGT_DEBUG_REG16__rst_last_bit                      0x00000400L
#define VGT_DEBUG_REG16__current_state_q_MASK              0x00000800L
#define VGT_DEBUG_REG16__current_state_q                   0x00000800L
#define VGT_DEBUG_REG16__old_state_q_MASK                  0x00001000L
#define VGT_DEBUG_REG16__old_state_q                       0x00001000L
#define VGT_DEBUG_REG16__old_state_en_MASK                 0x00002000L
#define VGT_DEBUG_REG16__old_state_en                      0x00002000L
#define VGT_DEBUG_REG16__prev_last_bit_q_MASK              0x00004000L
#define VGT_DEBUG_REG16__prev_last_bit_q                   0x00004000L
#define VGT_DEBUG_REG16__dbl_last_bit_q_MASK               0x00008000L
#define VGT_DEBUG_REG16__dbl_last_bit_q                    0x00008000L
#define VGT_DEBUG_REG16__last_bit_block_q_MASK             0x00010000L
#define VGT_DEBUG_REG16__last_bit_block_q                  0x00010000L
#define VGT_DEBUG_REG16__ast_bit_block2_q_MASK             0x00020000L
#define VGT_DEBUG_REG16__ast_bit_block2_q                  0x00020000L
#define VGT_DEBUG_REG16__load_empty_reg_MASK               0x00040000L
#define VGT_DEBUG_REG16__load_empty_reg                    0x00040000L
#define VGT_DEBUG_REG16__bgrp_grp_byte_mask_rdata_MASK     0x07f80000L
#define VGT_DEBUG_REG16__dma_bgrp_dma_data_fifo_rptr_MASK  0x18000000L
#define VGT_DEBUG_REG16__top_di_pre_fetch_cull_enable_MASK 0x20000000L
#define VGT_DEBUG_REG16__top_di_pre_fetch_cull_enable      0x20000000L
#define VGT_DEBUG_REG16__top_di_grp_cull_enable_q_MASK     0x40000000L
#define VGT_DEBUG_REG16__top_di_grp_cull_enable_q          0x40000000L
#define VGT_DEBUG_REG16__bgrp_trigger_MASK                 0x80000000L
#define VGT_DEBUG_REG16__bgrp_trigger                      0x80000000L

// VGT_DEBUG_REG17
#define VGT_DEBUG_REG17__save_read_q_MASK                  0x00000001L
#define VGT_DEBUG_REG17__save_read_q                       0x00000001L
#define VGT_DEBUG_REG17__extend_read_q_MASK                0x00000002L
#define VGT_DEBUG_REG17__extend_read_q                     0x00000002L
#define VGT_DEBUG_REG17__grp_indx_size_MASK                0x0000000cL
#define VGT_DEBUG_REG17__cull_prim_true_MASK               0x00000010L
#define VGT_DEBUG_REG17__cull_prim_true                    0x00000010L
#define VGT_DEBUG_REG17__reset_bit2_q_MASK                 0x00000020L
#define VGT_DEBUG_REG17__reset_bit2_q                      0x00000020L
#define VGT_DEBUG_REG17__reset_bit1_q_MASK                 0x00000040L
#define VGT_DEBUG_REG17__reset_bit1_q                      0x00000040L
#define VGT_DEBUG_REG17__first_reg_first_q_MASK            0x00000080L
#define VGT_DEBUG_REG17__first_reg_first_q                 0x00000080L
#define VGT_DEBUG_REG17__check_second_reg_MASK             0x00000100L
#define VGT_DEBUG_REG17__check_second_reg                  0x00000100L
#define VGT_DEBUG_REG17__check_first_reg_MASK              0x00000200L
#define VGT_DEBUG_REG17__check_first_reg                   0x00000200L
#define VGT_DEBUG_REG17__bgrp_cull_fetch_fifo_wdata_MASK   0x00000400L
#define VGT_DEBUG_REG17__bgrp_cull_fetch_fifo_wdata        0x00000400L
#define VGT_DEBUG_REG17__save_cull_fetch_data2_q_MASK      0x00000800L
#define VGT_DEBUG_REG17__save_cull_fetch_data2_q           0x00000800L
#define VGT_DEBUG_REG17__save_cull_fetch_data1_q_MASK      0x00001000L
#define VGT_DEBUG_REG17__save_cull_fetch_data1_q           0x00001000L
#define VGT_DEBUG_REG17__save_byte_mask_data2_q_MASK       0x00002000L
#define VGT_DEBUG_REG17__save_byte_mask_data2_q            0x00002000L
#define VGT_DEBUG_REG17__save_byte_mask_data1_q_MASK       0x00004000L
#define VGT_DEBUG_REG17__save_byte_mask_data1_q            0x00004000L
#define VGT_DEBUG_REG17__to_second_reg_q_MASK              0x00008000L
#define VGT_DEBUG_REG17__to_second_reg_q                   0x00008000L
#define VGT_DEBUG_REG17__roll_over_msk_q_MASK              0x00010000L
#define VGT_DEBUG_REG17__roll_over_msk_q                   0x00010000L
#define VGT_DEBUG_REG17__max_msk_ptr_q_MASK                0x00fe0000L
#define VGT_DEBUG_REG17__min_msk_ptr_q_MASK                0x7f000000L
#define VGT_DEBUG_REG17__bgrp_trigger_MASK                 0x80000000L
#define VGT_DEBUG_REG17__bgrp_trigger                      0x80000000L

// VGT_DEBUG_REG18
#define VGT_DEBUG_REG18__dma_data_fifo_mem_raddr_MASK      0x0000003fL
#define VGT_DEBUG_REG18__dma_data_fifo_mem_waddr_MASK      0x00000fc0L
#define VGT_DEBUG_REG18__dma_bgrp_byte_mask_fifo_re_MASK   0x00001000L
#define VGT_DEBUG_REG18__dma_bgrp_byte_mask_fifo_re        0x00001000L
#define VGT_DEBUG_REG18__dma_bgrp_dma_data_fifo_rptr_MASK  0x00006000L
#define VGT_DEBUG_REG18__dma_mem_full_MASK                 0x00008000L
#define VGT_DEBUG_REG18__dma_mem_full                      0x00008000L
#define VGT_DEBUG_REG18__dma_ram_re_MASK                   0x00010000L
#define VGT_DEBUG_REG18__dma_ram_re                        0x00010000L
#define VGT_DEBUG_REG18__dma_ram_we_MASK                   0x00020000L
#define VGT_DEBUG_REG18__dma_ram_we                        0x00020000L
#define VGT_DEBUG_REG18__dma_mem_empty_MASK                0x00040000L
#define VGT_DEBUG_REG18__dma_mem_empty                     0x00040000L
#define VGT_DEBUG_REG18__dma_data_fifo_mem_re_MASK         0x00080000L
#define VGT_DEBUG_REG18__dma_data_fifo_mem_re              0x00080000L
#define VGT_DEBUG_REG18__dma_data_fifo_mem_we_MASK         0x00100000L
#define VGT_DEBUG_REG18__dma_data_fifo_mem_we              0x00100000L
#define VGT_DEBUG_REG18__bin_mem_full_MASK                 0x00200000L
#define VGT_DEBUG_REG18__bin_mem_full                      0x00200000L
#define VGT_DEBUG_REG18__bin_ram_we_MASK                   0x00400000L
#define VGT_DEBUG_REG18__bin_ram_we                        0x00400000L
#define VGT_DEBUG_REG18__bin_ram_re_MASK                   0x00800000L
#define VGT_DEBUG_REG18__bin_ram_re                        0x00800000L
#define VGT_DEBUG_REG18__bin_mem_empty_MASK                0x01000000L
#define VGT_DEBUG_REG18__bin_mem_empty                     0x01000000L
#define VGT_DEBUG_REG18__start_bin_req_MASK                0x02000000L
#define VGT_DEBUG_REG18__start_bin_req                     0x02000000L
#define VGT_DEBUG_REG18__fetch_cull_not_used_MASK          0x04000000L
#define VGT_DEBUG_REG18__fetch_cull_not_used               0x04000000L
#define VGT_DEBUG_REG18__dma_req_xfer_MASK                 0x08000000L
#define VGT_DEBUG_REG18__dma_req_xfer                      0x08000000L
#define VGT_DEBUG_REG18__have_valid_bin_req_MASK           0x10000000L
#define VGT_DEBUG_REG18__have_valid_bin_req                0x10000000L
#define VGT_DEBUG_REG18__have_valid_dma_req_MASK           0x20000000L
#define VGT_DEBUG_REG18__have_valid_dma_req                0x20000000L
#define VGT_DEBUG_REG18__bgrp_dma_di_grp_cull_enable_MASK  0x40000000L
#define VGT_DEBUG_REG18__bgrp_dma_di_grp_cull_enable       0x40000000L
#define VGT_DEBUG_REG18__bgrp_dma_di_pre_fetch_cull_enable_MASK 0x80000000L
#define VGT_DEBUG_REG18__bgrp_dma_di_pre_fetch_cull_enable 0x80000000L

// VGT_DEBUG_REG20
#define VGT_DEBUG_REG20__prim_side_indx_valid_MASK         0x00000001L
#define VGT_DEBUG_REG20__prim_side_indx_valid              0x00000001L
#define VGT_DEBUG_REG20__indx_side_fifo_empty_MASK         0x00000002L
#define VGT_DEBUG_REG20__indx_side_fifo_empty              0x00000002L
#define VGT_DEBUG_REG20__indx_side_fifo_re_MASK            0x00000004L
#define VGT_DEBUG_REG20__indx_side_fifo_re                 0x00000004L
#define VGT_DEBUG_REG20__indx_side_fifo_we_MASK            0x00000008L
#define VGT_DEBUG_REG20__indx_side_fifo_we                 0x00000008L
#define VGT_DEBUG_REG20__indx_side_fifo_full_MASK          0x00000010L
#define VGT_DEBUG_REG20__indx_side_fifo_full               0x00000010L
#define VGT_DEBUG_REG20__prim_buffer_empty_MASK            0x00000020L
#define VGT_DEBUG_REG20__prim_buffer_empty                 0x00000020L
#define VGT_DEBUG_REG20__prim_buffer_re_MASK               0x00000040L
#define VGT_DEBUG_REG20__prim_buffer_re                    0x00000040L
#define VGT_DEBUG_REG20__prim_buffer_we_MASK               0x00000080L
#define VGT_DEBUG_REG20__prim_buffer_we                    0x00000080L
#define VGT_DEBUG_REG20__prim_buffer_full_MASK             0x00000100L
#define VGT_DEBUG_REG20__prim_buffer_full                  0x00000100L
#define VGT_DEBUG_REG20__indx_buffer_empty_MASK            0x00000200L
#define VGT_DEBUG_REG20__indx_buffer_empty                 0x00000200L
#define VGT_DEBUG_REG20__indx_buffer_re_MASK               0x00000400L
#define VGT_DEBUG_REG20__indx_buffer_re                    0x00000400L
#define VGT_DEBUG_REG20__indx_buffer_we_MASK               0x00000800L
#define VGT_DEBUG_REG20__indx_buffer_we                    0x00000800L
#define VGT_DEBUG_REG20__indx_buffer_full_MASK             0x00001000L
#define VGT_DEBUG_REG20__indx_buffer_full                  0x00001000L
#define VGT_DEBUG_REG20__hold_prim_MASK                    0x00002000L
#define VGT_DEBUG_REG20__hold_prim                         0x00002000L
#define VGT_DEBUG_REG20__sent_cnt_MASK                     0x0003c000L
#define VGT_DEBUG_REG20__start_of_vtx_vector_MASK          0x00040000L
#define VGT_DEBUG_REG20__start_of_vtx_vector               0x00040000L
#define VGT_DEBUG_REG20__clip_s_pre_hold_prim_MASK         0x00080000L
#define VGT_DEBUG_REG20__clip_s_pre_hold_prim              0x00080000L
#define VGT_DEBUG_REG20__clip_p_pre_hold_prim_MASK         0x00100000L
#define VGT_DEBUG_REG20__clip_p_pre_hold_prim              0x00100000L
#define VGT_DEBUG_REG20__buffered_prim_type_event_MASK     0x03e00000L
#define VGT_DEBUG_REG20__out_trigger_MASK                  0x04000000L
#define VGT_DEBUG_REG20__out_trigger                       0x04000000L

// VGT_DEBUG_REG21
#define VGT_DEBUG_REG21__null_terminate_vtx_vector_MASK    0x00000001L
#define VGT_DEBUG_REG21__null_terminate_vtx_vector         0x00000001L
#define VGT_DEBUG_REG21__prim_end_of_vtx_vect_flags_MASK   0x0000000eL
#define VGT_DEBUG_REG21__alloc_counter_q_MASK              0x00000070L
#define VGT_DEBUG_REG21__curr_slot_in_vtx_vect_q_MASK      0x00000380L
#define VGT_DEBUG_REG21__int_vtx_counter_q_MASK            0x00003c00L
#define VGT_DEBUG_REG21__curr_dealloc_distance_q_MASK      0x0003c000L
#define VGT_DEBUG_REG21__new_packet_q_MASK                 0x00040000L
#define VGT_DEBUG_REG21__new_packet_q                      0x00040000L
#define VGT_DEBUG_REG21__new_allocate_q_MASK               0x00080000L
#define VGT_DEBUG_REG21__new_allocate_q                    0x00080000L
#define VGT_DEBUG_REG21__num_new_unique_rel_indx_MASK      0x00300000L
#define VGT_DEBUG_REG21__inserted_null_prim_q_MASK         0x00400000L
#define VGT_DEBUG_REG21__inserted_null_prim_q              0x00400000L
#define VGT_DEBUG_REG21__insert_null_prim_MASK             0x00800000L
#define VGT_DEBUG_REG21__insert_null_prim                  0x00800000L
#define VGT_DEBUG_REG21__buffered_prim_eop_mux_MASK        0x01000000L
#define VGT_DEBUG_REG21__buffered_prim_eop_mux             0x01000000L
#define VGT_DEBUG_REG21__prim_buffer_empty_mux_MASK        0x02000000L
#define VGT_DEBUG_REG21__prim_buffer_empty_mux             0x02000000L
#define VGT_DEBUG_REG21__buffered_thread_size_MASK         0x04000000L
#define VGT_DEBUG_REG21__buffered_thread_size              0x04000000L
#define VGT_DEBUG_REG21__out_trigger_MASK                  0x80000000L
#define VGT_DEBUG_REG21__out_trigger                       0x80000000L

// VGT_CRC_SQ_DATA
#define VGT_CRC_SQ_DATA__CRC_MASK                          0xffffffffL

// VGT_CRC_SQ_CTRL
#define VGT_CRC_SQ_CTRL__CRC_MASK                          0xffffffffL

// VGT_PERFCOUNTER0_SELECT
#define VGT_PERFCOUNTER0_SELECT__PERF_SEL_MASK             0x000000ffL

// VGT_PERFCOUNTER1_SELECT
#define VGT_PERFCOUNTER1_SELECT__PERF_SEL_MASK             0x000000ffL

// VGT_PERFCOUNTER2_SELECT
#define VGT_PERFCOUNTER2_SELECT__PERF_SEL_MASK             0x000000ffL

// VGT_PERFCOUNTER3_SELECT
#define VGT_PERFCOUNTER3_SELECT__PERF_SEL_MASK             0x000000ffL

// VGT_PERFCOUNTER0_LOW
#define VGT_PERFCOUNTER0_LOW__PERF_COUNT_MASK              0xffffffffL

// VGT_PERFCOUNTER1_LOW
#define VGT_PERFCOUNTER1_LOW__PERF_COUNT_MASK              0xffffffffL

// VGT_PERFCOUNTER2_LOW
#define VGT_PERFCOUNTER2_LOW__PERF_COUNT_MASK              0xffffffffL

// VGT_PERFCOUNTER3_LOW
#define VGT_PERFCOUNTER3_LOW__PERF_COUNT_MASK              0xffffffffL

// VGT_PERFCOUNTER0_HI
#define VGT_PERFCOUNTER0_HI__PERF_COUNT_MASK               0x0000ffffL

// VGT_PERFCOUNTER1_HI
#define VGT_PERFCOUNTER1_HI__PERF_COUNT_MASK               0x0000ffffL

// VGT_PERFCOUNTER2_HI
#define VGT_PERFCOUNTER2_HI__PERF_COUNT_MASK               0x0000ffffL

// VGT_PERFCOUNTER3_HI
#define VGT_PERFCOUNTER3_HI__PERF_COUNT_MASK               0x0000ffffL

// TC_CNTL_STATUS
#define TC_CNTL_STATUS__L2_INVALIDATE_MASK                 0x00000001L
#define TC_CNTL_STATUS__L2_INVALIDATE                      0x00000001L
#define TC_CNTL_STATUS__TC_L2_HIT_MISS_MASK                0x000c0000L
#define TC_CNTL_STATUS__TC_BUSY_MASK                       0x80000000L
#define TC_CNTL_STATUS__TC_BUSY                            0x80000000L

// TCR_CHICKEN
#define TCR_CHICKEN__SPARE_MASK                            0xffffffffL

// TCF_CHICKEN
#define TCF_CHICKEN__SPARE_MASK                            0xffffffffL

// TCM_CHICKEN
#define TCM_CHICKEN__TCO_READ_LATENCY_FIFO_PROG_DEPTH_MASK 0x000000ffL
#define TCM_CHICKEN__ETC_COLOR_ENDIAN_MASK                 0x00000100L
#define TCM_CHICKEN__ETC_COLOR_ENDIAN                      0x00000100L
#define TCM_CHICKEN__SPARE_MASK                            0xfffffe00L

// TCR_PERFCOUNTER0_SELECT
#define TCR_PERFCOUNTER0_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCR_PERFCOUNTER1_SELECT
#define TCR_PERFCOUNTER1_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCR_PERFCOUNTER0_HI
#define TCR_PERFCOUNTER0_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCR_PERFCOUNTER1_HI
#define TCR_PERFCOUNTER1_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCR_PERFCOUNTER0_LOW
#define TCR_PERFCOUNTER0_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCR_PERFCOUNTER1_LOW
#define TCR_PERFCOUNTER1_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TP_TC_CLKGATE_CNTL
#define TP_TC_CLKGATE_CNTL__TP_BUSY_EXTEND_MASK            0x00000007L
#define TP_TC_CLKGATE_CNTL__TC_BUSY_EXTEND_MASK            0x00000038L

// TPC_CNTL_STATUS
#define TPC_CNTL_STATUS__TPC_INPUT_BUSY_MASK               0x00000001L
#define TPC_CNTL_STATUS__TPC_INPUT_BUSY                    0x00000001L
#define TPC_CNTL_STATUS__TPC_TC_FIFO_BUSY_MASK             0x00000002L
#define TPC_CNTL_STATUS__TPC_TC_FIFO_BUSY                  0x00000002L
#define TPC_CNTL_STATUS__TPC_STATE_FIFO_BUSY_MASK          0x00000004L
#define TPC_CNTL_STATUS__TPC_STATE_FIFO_BUSY               0x00000004L
#define TPC_CNTL_STATUS__TPC_FETCH_FIFO_BUSY_MASK          0x00000008L
#define TPC_CNTL_STATUS__TPC_FETCH_FIFO_BUSY               0x00000008L
#define TPC_CNTL_STATUS__TPC_WALKER_PIPE_BUSY_MASK         0x00000010L
#define TPC_CNTL_STATUS__TPC_WALKER_PIPE_BUSY              0x00000010L
#define TPC_CNTL_STATUS__TPC_WALK_FIFO_BUSY_MASK           0x00000020L
#define TPC_CNTL_STATUS__TPC_WALK_FIFO_BUSY                0x00000020L
#define TPC_CNTL_STATUS__TPC_WALKER_BUSY_MASK              0x00000040L
#define TPC_CNTL_STATUS__TPC_WALKER_BUSY                   0x00000040L
#define TPC_CNTL_STATUS__TPC_ALIGNER_PIPE_BUSY_MASK        0x00000100L
#define TPC_CNTL_STATUS__TPC_ALIGNER_PIPE_BUSY             0x00000100L
#define TPC_CNTL_STATUS__TPC_ALIGN_FIFO_BUSY_MASK          0x00000200L
#define TPC_CNTL_STATUS__TPC_ALIGN_FIFO_BUSY               0x00000200L
#define TPC_CNTL_STATUS__TPC_ALIGNER_BUSY_MASK             0x00000400L
#define TPC_CNTL_STATUS__TPC_ALIGNER_BUSY                  0x00000400L
#define TPC_CNTL_STATUS__TPC_RR_FIFO_BUSY_MASK             0x00001000L
#define TPC_CNTL_STATUS__TPC_RR_FIFO_BUSY                  0x00001000L
#define TPC_CNTL_STATUS__TPC_BLEND_PIPE_BUSY_MASK          0x00002000L
#define TPC_CNTL_STATUS__TPC_BLEND_PIPE_BUSY               0x00002000L
#define TPC_CNTL_STATUS__TPC_OUT_FIFO_BUSY_MASK            0x00004000L
#define TPC_CNTL_STATUS__TPC_OUT_FIFO_BUSY                 0x00004000L
#define TPC_CNTL_STATUS__TPC_BLEND_BUSY_MASK               0x00008000L
#define TPC_CNTL_STATUS__TPC_BLEND_BUSY                    0x00008000L
#define TPC_CNTL_STATUS__TF_TW_RTS_MASK                    0x00010000L
#define TPC_CNTL_STATUS__TF_TW_RTS                         0x00010000L
#define TPC_CNTL_STATUS__TF_TW_STATE_RTS_MASK              0x00020000L
#define TPC_CNTL_STATUS__TF_TW_STATE_RTS                   0x00020000L
#define TPC_CNTL_STATUS__TF_TW_RTR_MASK                    0x00080000L
#define TPC_CNTL_STATUS__TF_TW_RTR                         0x00080000L
#define TPC_CNTL_STATUS__TW_TA_RTS_MASK                    0x00100000L
#define TPC_CNTL_STATUS__TW_TA_RTS                         0x00100000L
#define TPC_CNTL_STATUS__TW_TA_TT_RTS_MASK                 0x00200000L
#define TPC_CNTL_STATUS__TW_TA_TT_RTS                      0x00200000L
#define TPC_CNTL_STATUS__TW_TA_LAST_RTS_MASK               0x00400000L
#define TPC_CNTL_STATUS__TW_TA_LAST_RTS                    0x00400000L
#define TPC_CNTL_STATUS__TW_TA_RTR_MASK                    0x00800000L
#define TPC_CNTL_STATUS__TW_TA_RTR                         0x00800000L
#define TPC_CNTL_STATUS__TA_TB_RTS_MASK                    0x01000000L
#define TPC_CNTL_STATUS__TA_TB_RTS                         0x01000000L
#define TPC_CNTL_STATUS__TA_TB_TT_RTS_MASK                 0x02000000L
#define TPC_CNTL_STATUS__TA_TB_TT_RTS                      0x02000000L
#define TPC_CNTL_STATUS__TA_TB_RTR_MASK                    0x08000000L
#define TPC_CNTL_STATUS__TA_TB_RTR                         0x08000000L
#define TPC_CNTL_STATUS__TA_TF_RTS_MASK                    0x10000000L
#define TPC_CNTL_STATUS__TA_TF_RTS                         0x10000000L
#define TPC_CNTL_STATUS__TA_TF_TC_FIFO_REN_MASK            0x20000000L
#define TPC_CNTL_STATUS__TA_TF_TC_FIFO_REN                 0x20000000L
#define TPC_CNTL_STATUS__TP_SQ_DEC_MASK                    0x40000000L
#define TPC_CNTL_STATUS__TP_SQ_DEC                         0x40000000L
#define TPC_CNTL_STATUS__TPC_BUSY_MASK                     0x80000000L
#define TPC_CNTL_STATUS__TPC_BUSY                          0x80000000L

// TPC_DEBUG0
#define TPC_DEBUG0__LOD_CNTL_MASK                          0x00000003L
#define TPC_DEBUG0__IC_CTR_MASK                            0x0000000cL
#define TPC_DEBUG0__WALKER_CNTL_MASK                       0x000000f0L
#define TPC_DEBUG0__ALIGNER_CNTL_MASK                      0x00000700L
#define TPC_DEBUG0__PREV_TC_STATE_VALID_MASK               0x00001000L
#define TPC_DEBUG0__PREV_TC_STATE_VALID                    0x00001000L
#define TPC_DEBUG0__WALKER_STATE_MASK                      0x03ff0000L
#define TPC_DEBUG0__ALIGNER_STATE_MASK                     0x0c000000L
#define TPC_DEBUG0__REG_CLK_EN_MASK                        0x20000000L
#define TPC_DEBUG0__REG_CLK_EN                             0x20000000L
#define TPC_DEBUG0__TPC_CLK_EN_MASK                        0x40000000L
#define TPC_DEBUG0__TPC_CLK_EN                             0x40000000L
#define TPC_DEBUG0__SQ_TP_WAKEUP_MASK                      0x80000000L
#define TPC_DEBUG0__SQ_TP_WAKEUP                           0x80000000L

// TPC_DEBUG1
#define TPC_DEBUG1__UNUSED_MASK                            0x00000001L
#define TPC_DEBUG1__UNUSED                                 0x00000001L

// TPC_CHICKEN
#define TPC_CHICKEN__BLEND_PRECISION_MASK                  0x00000001L
#define TPC_CHICKEN__BLEND_PRECISION                       0x00000001L
#define TPC_CHICKEN__SPARE_MASK                            0xfffffffeL

// TP0_CNTL_STATUS
#define TP0_CNTL_STATUS__TP_INPUT_BUSY_MASK                0x00000001L
#define TP0_CNTL_STATUS__TP_INPUT_BUSY                     0x00000001L
#define TP0_CNTL_STATUS__TP_LOD_BUSY_MASK                  0x00000002L
#define TP0_CNTL_STATUS__TP_LOD_BUSY                       0x00000002L
#define TP0_CNTL_STATUS__TP_LOD_FIFO_BUSY_MASK             0x00000004L
#define TP0_CNTL_STATUS__TP_LOD_FIFO_BUSY                  0x00000004L
#define TP0_CNTL_STATUS__TP_ADDR_BUSY_MASK                 0x00000008L
#define TP0_CNTL_STATUS__TP_ADDR_BUSY                      0x00000008L
#define TP0_CNTL_STATUS__TP_ALIGN_FIFO_BUSY_MASK           0x00000010L
#define TP0_CNTL_STATUS__TP_ALIGN_FIFO_BUSY                0x00000010L
#define TP0_CNTL_STATUS__TP_ALIGNER_BUSY_MASK              0x00000020L
#define TP0_CNTL_STATUS__TP_ALIGNER_BUSY                   0x00000020L
#define TP0_CNTL_STATUS__TP_TC_FIFO_BUSY_MASK              0x00000040L
#define TP0_CNTL_STATUS__TP_TC_FIFO_BUSY                   0x00000040L
#define TP0_CNTL_STATUS__TP_RR_FIFO_BUSY_MASK              0x00000080L
#define TP0_CNTL_STATUS__TP_RR_FIFO_BUSY                   0x00000080L
#define TP0_CNTL_STATUS__TP_FETCH_BUSY_MASK                0x00000100L
#define TP0_CNTL_STATUS__TP_FETCH_BUSY                     0x00000100L
#define TP0_CNTL_STATUS__TP_CH_BLEND_BUSY_MASK             0x00000200L
#define TP0_CNTL_STATUS__TP_CH_BLEND_BUSY                  0x00000200L
#define TP0_CNTL_STATUS__TP_TT_BUSY_MASK                   0x00000400L
#define TP0_CNTL_STATUS__TP_TT_BUSY                        0x00000400L
#define TP0_CNTL_STATUS__TP_HICOLOR_BUSY_MASK              0x00000800L
#define TP0_CNTL_STATUS__TP_HICOLOR_BUSY                   0x00000800L
#define TP0_CNTL_STATUS__TP_BLEND_BUSY_MASK                0x00001000L
#define TP0_CNTL_STATUS__TP_BLEND_BUSY                     0x00001000L
#define TP0_CNTL_STATUS__TP_OUT_FIFO_BUSY_MASK             0x00002000L
#define TP0_CNTL_STATUS__TP_OUT_FIFO_BUSY                  0x00002000L
#define TP0_CNTL_STATUS__TP_OUTPUT_BUSY_MASK               0x00004000L
#define TP0_CNTL_STATUS__TP_OUTPUT_BUSY                    0x00004000L
#define TP0_CNTL_STATUS__IN_LC_RTS_MASK                    0x00010000L
#define TP0_CNTL_STATUS__IN_LC_RTS                         0x00010000L
#define TP0_CNTL_STATUS__LC_LA_RTS_MASK                    0x00020000L
#define TP0_CNTL_STATUS__LC_LA_RTS                         0x00020000L
#define TP0_CNTL_STATUS__LA_FL_RTS_MASK                    0x00040000L
#define TP0_CNTL_STATUS__LA_FL_RTS                         0x00040000L
#define TP0_CNTL_STATUS__FL_TA_RTS_MASK                    0x00080000L
#define TP0_CNTL_STATUS__FL_TA_RTS                         0x00080000L
#define TP0_CNTL_STATUS__TA_FA_RTS_MASK                    0x00100000L
#define TP0_CNTL_STATUS__TA_FA_RTS                         0x00100000L
#define TP0_CNTL_STATUS__TA_FA_TT_RTS_MASK                 0x00200000L
#define TP0_CNTL_STATUS__TA_FA_TT_RTS                      0x00200000L
#define TP0_CNTL_STATUS__FA_AL_RTS_MASK                    0x00400000L
#define TP0_CNTL_STATUS__FA_AL_RTS                         0x00400000L
#define TP0_CNTL_STATUS__FA_AL_TT_RTS_MASK                 0x00800000L
#define TP0_CNTL_STATUS__FA_AL_TT_RTS                      0x00800000L
#define TP0_CNTL_STATUS__AL_TF_RTS_MASK                    0x01000000L
#define TP0_CNTL_STATUS__AL_TF_RTS                         0x01000000L
#define TP0_CNTL_STATUS__AL_TF_TT_RTS_MASK                 0x02000000L
#define TP0_CNTL_STATUS__AL_TF_TT_RTS                      0x02000000L
#define TP0_CNTL_STATUS__TF_TB_RTS_MASK                    0x04000000L
#define TP0_CNTL_STATUS__TF_TB_RTS                         0x04000000L
#define TP0_CNTL_STATUS__TF_TB_TT_RTS_MASK                 0x08000000L
#define TP0_CNTL_STATUS__TF_TB_TT_RTS                      0x08000000L
#define TP0_CNTL_STATUS__TB_TT_RTS_MASK                    0x10000000L
#define TP0_CNTL_STATUS__TB_TT_RTS                         0x10000000L
#define TP0_CNTL_STATUS__TB_TT_TT_RESET_MASK               0x20000000L
#define TP0_CNTL_STATUS__TB_TT_TT_RESET                    0x20000000L
#define TP0_CNTL_STATUS__TB_TO_RTS_MASK                    0x40000000L
#define TP0_CNTL_STATUS__TB_TO_RTS                         0x40000000L
#define TP0_CNTL_STATUS__TP_BUSY_MASK                      0x80000000L
#define TP0_CNTL_STATUS__TP_BUSY                           0x80000000L

// TP0_DEBUG
#define TP0_DEBUG__Q_LOD_CNTL_MASK                         0x00000003L
#define TP0_DEBUG__Q_SQ_TP_WAKEUP_MASK                     0x00000008L
#define TP0_DEBUG__Q_SQ_TP_WAKEUP                          0x00000008L
#define TP0_DEBUG__FL_TA_ADDRESSER_CNTL_MASK               0x001ffff0L
#define TP0_DEBUG__REG_CLK_EN_MASK                         0x00200000L
#define TP0_DEBUG__REG_CLK_EN                              0x00200000L
#define TP0_DEBUG__PERF_CLK_EN_MASK                        0x00400000L
#define TP0_DEBUG__PERF_CLK_EN                             0x00400000L
#define TP0_DEBUG__TP_CLK_EN_MASK                          0x00800000L
#define TP0_DEBUG__TP_CLK_EN                               0x00800000L
#define TP0_DEBUG__Q_WALKER_CNTL_MASK                      0x0f000000L
#define TP0_DEBUG__Q_ALIGNER_CNTL_MASK                     0x70000000L

// TP0_CHICKEN
#define TP0_CHICKEN__TT_MODE_MASK                          0x00000001L
#define TP0_CHICKEN__TT_MODE                               0x00000001L
#define TP0_CHICKEN__VFETCH_ADDRESS_MODE_MASK              0x00000002L
#define TP0_CHICKEN__VFETCH_ADDRESS_MODE                   0x00000002L
#define TP0_CHICKEN__SPARE_MASK                            0xfffffffcL

// TP0_PERFCOUNTER0_SELECT
#define TP0_PERFCOUNTER0_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TP0_PERFCOUNTER0_HI
#define TP0_PERFCOUNTER0_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TP0_PERFCOUNTER0_LOW
#define TP0_PERFCOUNTER0_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TP0_PERFCOUNTER1_SELECT
#define TP0_PERFCOUNTER1_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TP0_PERFCOUNTER1_HI
#define TP0_PERFCOUNTER1_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TP0_PERFCOUNTER1_LOW
#define TP0_PERFCOUNTER1_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCM_PERFCOUNTER0_SELECT
#define TCM_PERFCOUNTER0_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCM_PERFCOUNTER1_SELECT
#define TCM_PERFCOUNTER1_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCM_PERFCOUNTER0_HI
#define TCM_PERFCOUNTER0_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCM_PERFCOUNTER1_HI
#define TCM_PERFCOUNTER1_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCM_PERFCOUNTER0_LOW
#define TCM_PERFCOUNTER0_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCM_PERFCOUNTER1_LOW
#define TCM_PERFCOUNTER1_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCF_PERFCOUNTER0_SELECT
#define TCF_PERFCOUNTER0_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCF_PERFCOUNTER1_SELECT
#define TCF_PERFCOUNTER1_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCF_PERFCOUNTER2_SELECT
#define TCF_PERFCOUNTER2_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCF_PERFCOUNTER3_SELECT
#define TCF_PERFCOUNTER3_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCF_PERFCOUNTER4_SELECT
#define TCF_PERFCOUNTER4_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCF_PERFCOUNTER5_SELECT
#define TCF_PERFCOUNTER5_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCF_PERFCOUNTER6_SELECT
#define TCF_PERFCOUNTER6_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCF_PERFCOUNTER7_SELECT
#define TCF_PERFCOUNTER7_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCF_PERFCOUNTER8_SELECT
#define TCF_PERFCOUNTER8_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCF_PERFCOUNTER9_SELECT
#define TCF_PERFCOUNTER9_SELECT__PERFCOUNTER_SELECT_MASK   0x000000ffL

// TCF_PERFCOUNTER10_SELECT
#define TCF_PERFCOUNTER10_SELECT__PERFCOUNTER_SELECT_MASK  0x000000ffL

// TCF_PERFCOUNTER11_SELECT
#define TCF_PERFCOUNTER11_SELECT__PERFCOUNTER_SELECT_MASK  0x000000ffL

// TCF_PERFCOUNTER0_HI
#define TCF_PERFCOUNTER0_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCF_PERFCOUNTER1_HI
#define TCF_PERFCOUNTER1_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCF_PERFCOUNTER2_HI
#define TCF_PERFCOUNTER2_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCF_PERFCOUNTER3_HI
#define TCF_PERFCOUNTER3_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCF_PERFCOUNTER4_HI
#define TCF_PERFCOUNTER4_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCF_PERFCOUNTER5_HI
#define TCF_PERFCOUNTER5_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCF_PERFCOUNTER6_HI
#define TCF_PERFCOUNTER6_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCF_PERFCOUNTER7_HI
#define TCF_PERFCOUNTER7_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCF_PERFCOUNTER8_HI
#define TCF_PERFCOUNTER8_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCF_PERFCOUNTER9_HI
#define TCF_PERFCOUNTER9_HI__PERFCOUNTER_HI_MASK           0x0000ffffL

// TCF_PERFCOUNTER10_HI
#define TCF_PERFCOUNTER10_HI__PERFCOUNTER_HI_MASK          0x0000ffffL

// TCF_PERFCOUNTER11_HI
#define TCF_PERFCOUNTER11_HI__PERFCOUNTER_HI_MASK          0x0000ffffL

// TCF_PERFCOUNTER0_LOW
#define TCF_PERFCOUNTER0_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCF_PERFCOUNTER1_LOW
#define TCF_PERFCOUNTER1_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCF_PERFCOUNTER2_LOW
#define TCF_PERFCOUNTER2_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCF_PERFCOUNTER3_LOW
#define TCF_PERFCOUNTER3_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCF_PERFCOUNTER4_LOW
#define TCF_PERFCOUNTER4_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCF_PERFCOUNTER5_LOW
#define TCF_PERFCOUNTER5_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCF_PERFCOUNTER6_LOW
#define TCF_PERFCOUNTER6_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCF_PERFCOUNTER7_LOW
#define TCF_PERFCOUNTER7_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCF_PERFCOUNTER8_LOW
#define TCF_PERFCOUNTER8_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCF_PERFCOUNTER9_LOW
#define TCF_PERFCOUNTER9_LOW__PERFCOUNTER_LOW_MASK         0xffffffffL

// TCF_PERFCOUNTER10_LOW
#define TCF_PERFCOUNTER10_LOW__PERFCOUNTER_LOW_MASK        0xffffffffL

// TCF_PERFCOUNTER11_LOW
#define TCF_PERFCOUNTER11_LOW__PERFCOUNTER_LOW_MASK        0xffffffffL

// TCF_DEBUG
#define TCF_DEBUG__not_MH_TC_rtr_MASK                      0x00000040L
#define TCF_DEBUG__not_MH_TC_rtr                           0x00000040L
#define TCF_DEBUG__TC_MH_send_MASK                         0x00000080L
#define TCF_DEBUG__TC_MH_send                              0x00000080L
#define TCF_DEBUG__not_FG0_rtr_MASK                        0x00000100L
#define TCF_DEBUG__not_FG0_rtr                             0x00000100L
#define TCF_DEBUG__not_TCB_TCO_rtr_MASK                    0x00001000L
#define TCF_DEBUG__not_TCB_TCO_rtr                         0x00001000L
#define TCF_DEBUG__TCB_ff_stall_MASK                       0x00002000L
#define TCF_DEBUG__TCB_ff_stall                            0x00002000L
#define TCF_DEBUG__TCB_miss_stall_MASK                     0x00004000L
#define TCF_DEBUG__TCB_miss_stall                          0x00004000L
#define TCF_DEBUG__TCA_TCB_stall_MASK                      0x00008000L
#define TCF_DEBUG__TCA_TCB_stall                           0x00008000L
#define TCF_DEBUG__PF0_stall_MASK                          0x00010000L
#define TCF_DEBUG__PF0_stall                               0x00010000L
#define TCF_DEBUG__TP0_full_MASK                           0x00100000L
#define TCF_DEBUG__TP0_full                                0x00100000L
#define TCF_DEBUG__TPC_full_MASK                           0x01000000L
#define TCF_DEBUG__TPC_full                                0x01000000L
#define TCF_DEBUG__not_TPC_rtr_MASK                        0x02000000L
#define TCF_DEBUG__not_TPC_rtr                             0x02000000L
#define TCF_DEBUG__tca_state_rts_MASK                      0x04000000L
#define TCF_DEBUG__tca_state_rts                           0x04000000L
#define TCF_DEBUG__tca_rts_MASK                            0x08000000L
#define TCF_DEBUG__tca_rts                                 0x08000000L

// TCA_FIFO_DEBUG
#define TCA_FIFO_DEBUG__tp0_full_MASK                      0x00000001L
#define TCA_FIFO_DEBUG__tp0_full                           0x00000001L
#define TCA_FIFO_DEBUG__tpc_full_MASK                      0x00000010L
#define TCA_FIFO_DEBUG__tpc_full                           0x00000010L
#define TCA_FIFO_DEBUG__load_tpc_fifo_MASK                 0x00000020L
#define TCA_FIFO_DEBUG__load_tpc_fifo                      0x00000020L
#define TCA_FIFO_DEBUG__load_tp_fifos_MASK                 0x00000040L
#define TCA_FIFO_DEBUG__load_tp_fifos                      0x00000040L
#define TCA_FIFO_DEBUG__FW_full_MASK                       0x00000080L
#define TCA_FIFO_DEBUG__FW_full                            0x00000080L
#define TCA_FIFO_DEBUG__not_FW_rtr0_MASK                   0x00000100L
#define TCA_FIFO_DEBUG__not_FW_rtr0                        0x00000100L
#define TCA_FIFO_DEBUG__FW_rts0_MASK                       0x00001000L
#define TCA_FIFO_DEBUG__FW_rts0                            0x00001000L
#define TCA_FIFO_DEBUG__not_FW_tpc_rtr_MASK                0x00010000L
#define TCA_FIFO_DEBUG__not_FW_tpc_rtr                     0x00010000L
#define TCA_FIFO_DEBUG__FW_tpc_rts_MASK                    0x00020000L
#define TCA_FIFO_DEBUG__FW_tpc_rts                         0x00020000L

// TCA_PROBE_DEBUG
#define TCA_PROBE_DEBUG__ProbeFilter_stall_MASK            0x00000001L
#define TCA_PROBE_DEBUG__ProbeFilter_stall                 0x00000001L

// TCA_TPC_DEBUG
#define TCA_TPC_DEBUG__captue_state_rts_MASK               0x00001000L
#define TCA_TPC_DEBUG__captue_state_rts                    0x00001000L
#define TCA_TPC_DEBUG__capture_tca_rts_MASK                0x00002000L
#define TCA_TPC_DEBUG__capture_tca_rts                     0x00002000L

// TCB_CORE_DEBUG
#define TCB_CORE_DEBUG__access512_MASK                     0x00000001L
#define TCB_CORE_DEBUG__access512                          0x00000001L
#define TCB_CORE_DEBUG__tiled_MASK                         0x00000002L
#define TCB_CORE_DEBUG__tiled                              0x00000002L
#define TCB_CORE_DEBUG__opcode_MASK                        0x00000070L
#define TCB_CORE_DEBUG__format_MASK                        0x00003f00L
#define TCB_CORE_DEBUG__sector_format_MASK                 0x001f0000L
#define TCB_CORE_DEBUG__sector_format512_MASK              0x07000000L

// TCB_TAG0_DEBUG
#define TCB_TAG0_DEBUG__mem_read_cycle_MASK                0x000003ffL
#define TCB_TAG0_DEBUG__tag_access_cycle_MASK              0x001ff000L
#define TCB_TAG0_DEBUG__miss_stall_MASK                    0x00800000L
#define TCB_TAG0_DEBUG__miss_stall                         0x00800000L
#define TCB_TAG0_DEBUG__num_feee_lines_MASK                0x1f000000L
#define TCB_TAG0_DEBUG__max_misses_MASK                    0xe0000000L

// TCB_TAG1_DEBUG
#define TCB_TAG1_DEBUG__mem_read_cycle_MASK                0x000003ffL
#define TCB_TAG1_DEBUG__tag_access_cycle_MASK              0x001ff000L
#define TCB_TAG1_DEBUG__miss_stall_MASK                    0x00800000L
#define TCB_TAG1_DEBUG__miss_stall                         0x00800000L
#define TCB_TAG1_DEBUG__num_feee_lines_MASK                0x1f000000L
#define TCB_TAG1_DEBUG__max_misses_MASK                    0xe0000000L

// TCB_TAG2_DEBUG
#define TCB_TAG2_DEBUG__mem_read_cycle_MASK                0x000003ffL
#define TCB_TAG2_DEBUG__tag_access_cycle_MASK              0x001ff000L
#define TCB_TAG2_DEBUG__miss_stall_MASK                    0x00800000L
#define TCB_TAG2_DEBUG__miss_stall                         0x00800000L
#define TCB_TAG2_DEBUG__num_feee_lines_MASK                0x1f000000L
#define TCB_TAG2_DEBUG__max_misses_MASK                    0xe0000000L

// TCB_TAG3_DEBUG
#define TCB_TAG3_DEBUG__mem_read_cycle_MASK                0x000003ffL
#define TCB_TAG3_DEBUG__tag_access_cycle_MASK              0x001ff000L
#define TCB_TAG3_DEBUG__miss_stall_MASK                    0x00800000L
#define TCB_TAG3_DEBUG__miss_stall                         0x00800000L
#define TCB_TAG3_DEBUG__num_feee_lines_MASK                0x1f000000L
#define TCB_TAG3_DEBUG__max_misses_MASK                    0xe0000000L

// TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__left_done_MASK 0x00000001L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__left_done      0x00000001L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__fg0_sends_left_MASK 0x00000004L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__fg0_sends_left 0x00000004L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__one_sector_to_go_left_q_MASK 0x00000010L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__one_sector_to_go_left_q 0x00000010L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__no_sectors_to_go_MASK 0x00000020L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__no_sectors_to_go 0x00000020L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__update_left_MASK 0x00000040L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__update_left    0x00000040L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__sector_mask_left_count_q_MASK 0x00000f80L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__sector_mask_left_q_MASK 0x0ffff000L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__valid_left_q_MASK 0x10000000L
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__valid_left_q   0x10000000L

// TCB_FETCH_GEN_WALKER_DEBUG
#define TCB_FETCH_GEN_WALKER_DEBUG__quad_sel_left_MASK     0x00000030L
#define TCB_FETCH_GEN_WALKER_DEBUG__set_sel_left_MASK      0x000000c0L
#define TCB_FETCH_GEN_WALKER_DEBUG__right_eq_left_MASK     0x00000800L
#define TCB_FETCH_GEN_WALKER_DEBUG__right_eq_left          0x00000800L
#define TCB_FETCH_GEN_WALKER_DEBUG__ff_fg_type512_MASK     0x00007000L
#define TCB_FETCH_GEN_WALKER_DEBUG__busy_MASK              0x00008000L
#define TCB_FETCH_GEN_WALKER_DEBUG__busy                   0x00008000L
#define TCB_FETCH_GEN_WALKER_DEBUG__setquads_to_send_MASK  0x000f0000L

// TCB_FETCH_GEN_PIPE0_DEBUG
#define TCB_FETCH_GEN_PIPE0_DEBUG__tc0_arb_rts_MASK        0x00000001L
#define TCB_FETCH_GEN_PIPE0_DEBUG__tc0_arb_rts             0x00000001L
#define TCB_FETCH_GEN_PIPE0_DEBUG__ga_out_rts_MASK         0x00000004L
#define TCB_FETCH_GEN_PIPE0_DEBUG__ga_out_rts              0x00000004L
#define TCB_FETCH_GEN_PIPE0_DEBUG__tc_arb_format_MASK      0x0000fff0L
#define TCB_FETCH_GEN_PIPE0_DEBUG__tc_arb_fmsopcode_MASK   0x001f0000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__tc_arb_request_type_MASK 0x00600000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__busy_MASK               0x00800000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__busy                    0x00800000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__fgo_busy_MASK           0x01000000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__fgo_busy                0x01000000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__ga_busy_MASK            0x02000000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__ga_busy                 0x02000000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__mc_sel_q_MASK           0x0c000000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__valid_q_MASK            0x10000000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__valid_q                 0x10000000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__arb_RTR_MASK            0x40000000L
#define TCB_FETCH_GEN_PIPE0_DEBUG__arb_RTR                 0x40000000L

// TCD_INPUT0_DEBUG
#define TCD_INPUT0_DEBUG__empty_MASK                       0x00010000L
#define TCD_INPUT0_DEBUG__empty                            0x00010000L
#define TCD_INPUT0_DEBUG__full_MASK                        0x00020000L
#define TCD_INPUT0_DEBUG__full                             0x00020000L
#define TCD_INPUT0_DEBUG__valid_q1_MASK                    0x00100000L
#define TCD_INPUT0_DEBUG__valid_q1                         0x00100000L
#define TCD_INPUT0_DEBUG__cnt_q1_MASK                      0x00600000L
#define TCD_INPUT0_DEBUG__last_send_q1_MASK                0x00800000L
#define TCD_INPUT0_DEBUG__last_send_q1                     0x00800000L
#define TCD_INPUT0_DEBUG__ip_send_MASK                     0x01000000L
#define TCD_INPUT0_DEBUG__ip_send                          0x01000000L
#define TCD_INPUT0_DEBUG__ipbuf_dxt_send_MASK              0x02000000L
#define TCD_INPUT0_DEBUG__ipbuf_dxt_send                   0x02000000L
#define TCD_INPUT0_DEBUG__ipbuf_busy_MASK                  0x04000000L
#define TCD_INPUT0_DEBUG__ipbuf_busy                       0x04000000L

// TCD_DEGAMMA_DEBUG
#define TCD_DEGAMMA_DEBUG__dgmm_ftfconv_dgmmen_MASK        0x00000003L
#define TCD_DEGAMMA_DEBUG__dgmm_ctrl_dgmm8_MASK            0x00000004L
#define TCD_DEGAMMA_DEBUG__dgmm_ctrl_dgmm8                 0x00000004L
#define TCD_DEGAMMA_DEBUG__dgmm_ctrl_last_send_MASK        0x00000008L
#define TCD_DEGAMMA_DEBUG__dgmm_ctrl_last_send             0x00000008L
#define TCD_DEGAMMA_DEBUG__dgmm_ctrl_send_MASK             0x00000010L
#define TCD_DEGAMMA_DEBUG__dgmm_ctrl_send                  0x00000010L
#define TCD_DEGAMMA_DEBUG__dgmm_stall_MASK                 0x00000020L
#define TCD_DEGAMMA_DEBUG__dgmm_stall                      0x00000020L
#define TCD_DEGAMMA_DEBUG__dgmm_pstate_MASK                0x00000040L
#define TCD_DEGAMMA_DEBUG__dgmm_pstate                     0x00000040L

// TCD_DXTMUX_SCTARB_DEBUG
#define TCD_DXTMUX_SCTARB_DEBUG__pstate_MASK               0x00000200L
#define TCD_DXTMUX_SCTARB_DEBUG__pstate                    0x00000200L
#define TCD_DXTMUX_SCTARB_DEBUG__sctrmx_rtr_MASK           0x00000400L
#define TCD_DXTMUX_SCTARB_DEBUG__sctrmx_rtr                0x00000400L
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_rtr_MASK             0x00000800L
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_rtr                  0x00000800L
#define TCD_DXTMUX_SCTARB_DEBUG__sctrarb_multcyl_send_MASK 0x00008000L
#define TCD_DXTMUX_SCTARB_DEBUG__sctrarb_multcyl_send      0x00008000L
#define TCD_DXTMUX_SCTARB_DEBUG__sctrmx0_sctrarb_rts_MASK  0x00010000L
#define TCD_DXTMUX_SCTARB_DEBUG__sctrmx0_sctrarb_rts       0x00010000L
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_sctrarb_send_MASK    0x00100000L
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_sctrarb_send         0x00100000L
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_dgmmpd_last_send_MASK 0x08000000L
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_dgmmpd_last_send     0x08000000L
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_dgmmpd_send_MASK     0x10000000L
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_dgmmpd_send          0x10000000L
#define TCD_DXTMUX_SCTARB_DEBUG__dcmp_mux_send_MASK        0x20000000L
#define TCD_DXTMUX_SCTARB_DEBUG__dcmp_mux_send             0x20000000L

// TCD_DXTC_ARB_DEBUG
#define TCD_DXTC_ARB_DEBUG__n0_stall_MASK                  0x00000010L
#define TCD_DXTC_ARB_DEBUG__n0_stall                       0x00000010L
#define TCD_DXTC_ARB_DEBUG__pstate_MASK                    0x00000020L
#define TCD_DXTC_ARB_DEBUG__pstate                         0x00000020L
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_last_send_MASK      0x00000040L
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_last_send           0x00000040L
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_cnt_MASK            0x00000180L
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_sector_MASK         0x00000e00L
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_cacheline_MASK      0x0003f000L
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_format_MASK         0x3ffc0000L
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_send_MASK           0x40000000L
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_send                0x40000000L
#define TCD_DXTC_ARB_DEBUG__n0_dxt2_4_types_MASK           0x80000000L
#define TCD_DXTC_ARB_DEBUG__n0_dxt2_4_types                0x80000000L

// TCD_STALLS_DEBUG
#define TCD_STALLS_DEBUG__not_multcyl_sctrarb_rtr_MASK     0x00000400L
#define TCD_STALLS_DEBUG__not_multcyl_sctrarb_rtr          0x00000400L
#define TCD_STALLS_DEBUG__not_sctrmx0_sctrarb_rtr_MASK     0x00000800L
#define TCD_STALLS_DEBUG__not_sctrmx0_sctrarb_rtr          0x00000800L
#define TCD_STALLS_DEBUG__not_dcmp0_arb_rtr_MASK           0x00020000L
#define TCD_STALLS_DEBUG__not_dcmp0_arb_rtr                0x00020000L
#define TCD_STALLS_DEBUG__not_dgmmpd_dxtc_rtr_MASK         0x00040000L
#define TCD_STALLS_DEBUG__not_dgmmpd_dxtc_rtr              0x00040000L
#define TCD_STALLS_DEBUG__not_mux_dcmp_rtr_MASK            0x00080000L
#define TCD_STALLS_DEBUG__not_mux_dcmp_rtr                 0x00080000L
#define TCD_STALLS_DEBUG__not_incoming_rtr_MASK            0x80000000L
#define TCD_STALLS_DEBUG__not_incoming_rtr                 0x80000000L

// TCO_STALLS_DEBUG
#define TCO_STALLS_DEBUG__quad0_sg_crd_RTR_MASK            0x00000020L
#define TCO_STALLS_DEBUG__quad0_sg_crd_RTR                 0x00000020L
#define TCO_STALLS_DEBUG__quad0_rl_sg_RTR_MASK             0x00000040L
#define TCO_STALLS_DEBUG__quad0_rl_sg_RTR                  0x00000040L
#define TCO_STALLS_DEBUG__quad0_TCO_TCB_rtr_d_MASK         0x00000080L
#define TCO_STALLS_DEBUG__quad0_TCO_TCB_rtr_d              0x00000080L

// TCO_QUAD0_DEBUG0
#define TCO_QUAD0_DEBUG0__rl_sg_sector_format_MASK         0x000000ffL
#define TCO_QUAD0_DEBUG0__rl_sg_end_of_sample_MASK         0x00000100L
#define TCO_QUAD0_DEBUG0__rl_sg_end_of_sample              0x00000100L
#define TCO_QUAD0_DEBUG0__rl_sg_rtr_MASK                   0x00000200L
#define TCO_QUAD0_DEBUG0__rl_sg_rtr                        0x00000200L
#define TCO_QUAD0_DEBUG0__rl_sg_rts_MASK                   0x00000400L
#define TCO_QUAD0_DEBUG0__rl_sg_rts                        0x00000400L
#define TCO_QUAD0_DEBUG0__sg_crd_end_of_sample_MASK        0x00000800L
#define TCO_QUAD0_DEBUG0__sg_crd_end_of_sample             0x00000800L
#define TCO_QUAD0_DEBUG0__sg_crd_rtr_MASK                  0x00001000L
#define TCO_QUAD0_DEBUG0__sg_crd_rtr                       0x00001000L
#define TCO_QUAD0_DEBUG0__sg_crd_rts_MASK                  0x00002000L
#define TCO_QUAD0_DEBUG0__sg_crd_rts                       0x00002000L
#define TCO_QUAD0_DEBUG0__stageN1_valid_q_MASK             0x00010000L
#define TCO_QUAD0_DEBUG0__stageN1_valid_q                  0x00010000L
#define TCO_QUAD0_DEBUG0__read_cache_q_MASK                0x01000000L
#define TCO_QUAD0_DEBUG0__read_cache_q                     0x01000000L
#define TCO_QUAD0_DEBUG0__cache_read_RTR_MASK              0x02000000L
#define TCO_QUAD0_DEBUG0__cache_read_RTR                   0x02000000L
#define TCO_QUAD0_DEBUG0__all_sectors_written_set3_MASK    0x04000000L
#define TCO_QUAD0_DEBUG0__all_sectors_written_set3         0x04000000L
#define TCO_QUAD0_DEBUG0__all_sectors_written_set2_MASK    0x08000000L
#define TCO_QUAD0_DEBUG0__all_sectors_written_set2         0x08000000L
#define TCO_QUAD0_DEBUG0__all_sectors_written_set1_MASK    0x10000000L
#define TCO_QUAD0_DEBUG0__all_sectors_written_set1         0x10000000L
#define TCO_QUAD0_DEBUG0__all_sectors_written_set0_MASK    0x20000000L
#define TCO_QUAD0_DEBUG0__all_sectors_written_set0         0x20000000L
#define TCO_QUAD0_DEBUG0__busy_MASK                        0x40000000L
#define TCO_QUAD0_DEBUG0__busy                             0x40000000L

// TCO_QUAD0_DEBUG1
#define TCO_QUAD0_DEBUG1__fifo_busy_MASK                   0x00000001L
#define TCO_QUAD0_DEBUG1__fifo_busy                        0x00000001L
#define TCO_QUAD0_DEBUG1__empty_MASK                       0x00000002L
#define TCO_QUAD0_DEBUG1__empty                            0x00000002L
#define TCO_QUAD0_DEBUG1__full_MASK                        0x00000004L
#define TCO_QUAD0_DEBUG1__full                             0x00000004L
#define TCO_QUAD0_DEBUG1__write_enable_MASK                0x00000008L
#define TCO_QUAD0_DEBUG1__write_enable                     0x00000008L
#define TCO_QUAD0_DEBUG1__fifo_write_ptr_MASK              0x000007f0L
#define TCO_QUAD0_DEBUG1__fifo_read_ptr_MASK               0x0003f800L
#define TCO_QUAD0_DEBUG1__cache_read_busy_MASK             0x00100000L
#define TCO_QUAD0_DEBUG1__cache_read_busy                  0x00100000L
#define TCO_QUAD0_DEBUG1__latency_fifo_busy_MASK           0x00200000L
#define TCO_QUAD0_DEBUG1__latency_fifo_busy                0x00200000L
#define TCO_QUAD0_DEBUG1__input_quad_busy_MASK             0x00400000L
#define TCO_QUAD0_DEBUG1__input_quad_busy                  0x00400000L
#define TCO_QUAD0_DEBUG1__tco_quad_pipe_busy_MASK          0x00800000L
#define TCO_QUAD0_DEBUG1__tco_quad_pipe_busy               0x00800000L
#define TCO_QUAD0_DEBUG1__TCB_TCO_rtr_d_MASK               0x01000000L
#define TCO_QUAD0_DEBUG1__TCB_TCO_rtr_d                    0x01000000L
#define TCO_QUAD0_DEBUG1__TCB_TCO_xfc_q_MASK               0x02000000L
#define TCO_QUAD0_DEBUG1__TCB_TCO_xfc_q                    0x02000000L
#define TCO_QUAD0_DEBUG1__rl_sg_rtr_MASK                   0x04000000L
#define TCO_QUAD0_DEBUG1__rl_sg_rtr                        0x04000000L
#define TCO_QUAD0_DEBUG1__rl_sg_rts_MASK                   0x08000000L
#define TCO_QUAD0_DEBUG1__rl_sg_rts                        0x08000000L
#define TCO_QUAD0_DEBUG1__sg_crd_rtr_MASK                  0x10000000L
#define TCO_QUAD0_DEBUG1__sg_crd_rtr                       0x10000000L
#define TCO_QUAD0_DEBUG1__sg_crd_rts_MASK                  0x20000000L
#define TCO_QUAD0_DEBUG1__sg_crd_rts                       0x20000000L
#define TCO_QUAD0_DEBUG1__TCO_TCB_read_xfc_MASK            0x40000000L
#define TCO_QUAD0_DEBUG1__TCO_TCB_read_xfc                 0x40000000L

// SQ_GPR_MANAGEMENT
#define SQ_GPR_MANAGEMENT__REG_DYNAMIC_MASK                0x00000001L
#define SQ_GPR_MANAGEMENT__REG_DYNAMIC                     0x00000001L
#define SQ_GPR_MANAGEMENT__REG_SIZE_PIX_MASK               0x000007f0L
#define SQ_GPR_MANAGEMENT__REG_SIZE_VTX_MASK               0x0007f000L

// SQ_FLOW_CONTROL
#define SQ_FLOW_CONTROL__INPUT_ARBITRATION_POLICY_MASK     0x00000003L
#define SQ_FLOW_CONTROL__ONE_THREAD_MASK                   0x00000010L
#define SQ_FLOW_CONTROL__ONE_THREAD                        0x00000010L
#define SQ_FLOW_CONTROL__ONE_ALU_MASK                      0x00000100L
#define SQ_FLOW_CONTROL__ONE_ALU                           0x00000100L
#define SQ_FLOW_CONTROL__CF_WR_BASE_MASK                   0x0000f000L
#define SQ_FLOW_CONTROL__NO_PV_PS_MASK                     0x00010000L
#define SQ_FLOW_CONTROL__NO_PV_PS                          0x00010000L
#define SQ_FLOW_CONTROL__NO_LOOP_EXIT_MASK                 0x00020000L
#define SQ_FLOW_CONTROL__NO_LOOP_EXIT                      0x00020000L
#define SQ_FLOW_CONTROL__NO_CEXEC_OPTIMIZE_MASK            0x00040000L
#define SQ_FLOW_CONTROL__NO_CEXEC_OPTIMIZE                 0x00040000L
#define SQ_FLOW_CONTROL__TEXTURE_ARBITRATION_POLICY_MASK   0x00180000L
#define SQ_FLOW_CONTROL__VC_ARBITRATION_POLICY_MASK        0x00200000L
#define SQ_FLOW_CONTROL__VC_ARBITRATION_POLICY             0x00200000L
#define SQ_FLOW_CONTROL__ALU_ARBITRATION_POLICY_MASK       0x00400000L
#define SQ_FLOW_CONTROL__ALU_ARBITRATION_POLICY            0x00400000L
#define SQ_FLOW_CONTROL__NO_ARB_EJECT_MASK                 0x00800000L
#define SQ_FLOW_CONTROL__NO_ARB_EJECT                      0x00800000L
#define SQ_FLOW_CONTROL__NO_CFS_EJECT_MASK                 0x01000000L
#define SQ_FLOW_CONTROL__NO_CFS_EJECT                      0x01000000L
#define SQ_FLOW_CONTROL__POS_EXP_PRIORITY_MASK             0x02000000L
#define SQ_FLOW_CONTROL__POS_EXP_PRIORITY                  0x02000000L
#define SQ_FLOW_CONTROL__NO_EARLY_THREAD_TERMINATION_MASK  0x04000000L
#define SQ_FLOW_CONTROL__NO_EARLY_THREAD_TERMINATION       0x04000000L
#define SQ_FLOW_CONTROL__PS_PREFETCH_COLOR_ALLOC_MASK      0x08000000L
#define SQ_FLOW_CONTROL__PS_PREFETCH_COLOR_ALLOC           0x08000000L

// SQ_INST_STORE_MANAGMENT
#define SQ_INST_STORE_MANAGMENT__INST_BASE_PIX_MASK        0x00000fffL
#define SQ_INST_STORE_MANAGMENT__INST_BASE_VTX_MASK        0x0fff0000L

// SQ_RESOURCE_MANAGMENT
#define SQ_RESOURCE_MANAGMENT__VTX_THREAD_BUF_ENTRIES_MASK 0x000000ffL
#define SQ_RESOURCE_MANAGMENT__PIX_THREAD_BUF_ENTRIES_MASK 0x0000ff00L
#define SQ_RESOURCE_MANAGMENT__EXPORT_BUF_ENTRIES_MASK     0x01ff0000L

// SQ_EO_RT
#define SQ_EO_RT__EO_CONSTANTS_RT_MASK                     0x000000ffL
#define SQ_EO_RT__EO_TSTATE_RT_MASK                        0x00ff0000L

// SQ_DEBUG_MISC
#define SQ_DEBUG_MISC__DB_ALUCST_SIZE_MASK                 0x000007ffL
#define SQ_DEBUG_MISC__DB_TSTATE_SIZE_MASK                 0x000ff000L
#define SQ_DEBUG_MISC__DB_READ_CTX_MASK                    0x00100000L
#define SQ_DEBUG_MISC__DB_READ_CTX                         0x00100000L
#define SQ_DEBUG_MISC__RESERVED_MASK                       0x00600000L
#define SQ_DEBUG_MISC__DB_READ_MEMORY_MASK                 0x01800000L
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_0_MASK                0x02000000L
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_0                     0x02000000L
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_1_MASK                0x04000000L
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_1                     0x04000000L
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_2_MASK                0x08000000L
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_2                     0x08000000L
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_3_MASK                0x10000000L
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_3                     0x10000000L

// SQ_ACTIVITY_METER_CNTL
#define SQ_ACTIVITY_METER_CNTL__TIMEBASE_MASK              0x000000ffL
#define SQ_ACTIVITY_METER_CNTL__THRESHOLD_LOW_MASK         0x0000ff00L
#define SQ_ACTIVITY_METER_CNTL__THRESHOLD_HIGH_MASK        0x00ff0000L
#define SQ_ACTIVITY_METER_CNTL__SPARE_MASK                 0xff000000L

// SQ_ACTIVITY_METER_STATUS
#define SQ_ACTIVITY_METER_STATUS__PERCENT_BUSY_MASK        0x000000ffL

// SQ_INPUT_ARB_PRIORITY
#define SQ_INPUT_ARB_PRIORITY__PC_AVAIL_WEIGHT_MASK        0x00000007L
#define SQ_INPUT_ARB_PRIORITY__PC_AVAIL_SIGN_MASK          0x00000008L
#define SQ_INPUT_ARB_PRIORITY__PC_AVAIL_SIGN               0x00000008L
#define SQ_INPUT_ARB_PRIORITY__SX_AVAIL_WEIGHT_MASK        0x00000070L
#define SQ_INPUT_ARB_PRIORITY__SX_AVAIL_SIGN_MASK          0x00000080L
#define SQ_INPUT_ARB_PRIORITY__SX_AVAIL_SIGN               0x00000080L
#define SQ_INPUT_ARB_PRIORITY__THRESHOLD_MASK              0x0003ff00L

// SQ_THREAD_ARB_PRIORITY
#define SQ_THREAD_ARB_PRIORITY__PC_AVAIL_WEIGHT_MASK       0x00000007L
#define SQ_THREAD_ARB_PRIORITY__PC_AVAIL_SIGN_MASK         0x00000008L
#define SQ_THREAD_ARB_PRIORITY__PC_AVAIL_SIGN              0x00000008L
#define SQ_THREAD_ARB_PRIORITY__SX_AVAIL_WEIGHT_MASK       0x00000070L
#define SQ_THREAD_ARB_PRIORITY__SX_AVAIL_SIGN_MASK         0x00000080L
#define SQ_THREAD_ARB_PRIORITY__SX_AVAIL_SIGN              0x00000080L
#define SQ_THREAD_ARB_PRIORITY__THRESHOLD_MASK             0x0003ff00L
#define SQ_THREAD_ARB_PRIORITY__RESERVED_MASK              0x000c0000L
#define SQ_THREAD_ARB_PRIORITY__VS_PRIORITIZE_SERIAL_MASK  0x00100000L
#define SQ_THREAD_ARB_PRIORITY__VS_PRIORITIZE_SERIAL       0x00100000L
#define SQ_THREAD_ARB_PRIORITY__PS_PRIORITIZE_SERIAL_MASK  0x00200000L
#define SQ_THREAD_ARB_PRIORITY__PS_PRIORITIZE_SERIAL       0x00200000L
#define SQ_THREAD_ARB_PRIORITY__USE_SERIAL_COUNT_THRESHOLD_MASK 0x00400000L
#define SQ_THREAD_ARB_PRIORITY__USE_SERIAL_COUNT_THRESHOLD 0x00400000L

// SQ_DEBUG_INPUT_FSM
#define SQ_DEBUG_INPUT_FSM__VC_VSR_LD_MASK                 0x00000007L
#define SQ_DEBUG_INPUT_FSM__RESERVED_MASK                  0x00000008L
#define SQ_DEBUG_INPUT_FSM__RESERVED                       0x00000008L
#define SQ_DEBUG_INPUT_FSM__VC_GPR_LD_MASK                 0x000000f0L
#define SQ_DEBUG_INPUT_FSM__PC_PISM_MASK                   0x00000700L
#define SQ_DEBUG_INPUT_FSM__RESERVED1_MASK                 0x00000800L
#define SQ_DEBUG_INPUT_FSM__RESERVED1                      0x00000800L
#define SQ_DEBUG_INPUT_FSM__PC_AS_MASK                     0x00007000L
#define SQ_DEBUG_INPUT_FSM__PC_INTERP_CNT_MASK             0x000f8000L
#define SQ_DEBUG_INPUT_FSM__PC_GPR_SIZE_MASK               0x0ff00000L

// SQ_DEBUG_CONST_MGR_FSM
#define SQ_DEBUG_CONST_MGR_FSM__TEX_CONST_EVENT_STATE_MASK 0x0000001fL
#define SQ_DEBUG_CONST_MGR_FSM__RESERVED1_MASK             0x000000e0L
#define SQ_DEBUG_CONST_MGR_FSM__ALU_CONST_EVENT_STATE_MASK 0x00001f00L
#define SQ_DEBUG_CONST_MGR_FSM__RESERVED2_MASK             0x0000e000L
#define SQ_DEBUG_CONST_MGR_FSM__ALU_CONST_CNTX_VALID_MASK  0x00030000L
#define SQ_DEBUG_CONST_MGR_FSM__TEX_CONST_CNTX_VALID_MASK  0x000c0000L
#define SQ_DEBUG_CONST_MGR_FSM__CNTX0_VTX_EVENT_DONE_MASK  0x00100000L
#define SQ_DEBUG_CONST_MGR_FSM__CNTX0_VTX_EVENT_DONE       0x00100000L
#define SQ_DEBUG_CONST_MGR_FSM__CNTX0_PIX_EVENT_DONE_MASK  0x00200000L
#define SQ_DEBUG_CONST_MGR_FSM__CNTX0_PIX_EVENT_DONE       0x00200000L
#define SQ_DEBUG_CONST_MGR_FSM__CNTX1_VTX_EVENT_DONE_MASK  0x00400000L
#define SQ_DEBUG_CONST_MGR_FSM__CNTX1_VTX_EVENT_DONE       0x00400000L
#define SQ_DEBUG_CONST_MGR_FSM__CNTX1_PIX_EVENT_DONE_MASK  0x00800000L
#define SQ_DEBUG_CONST_MGR_FSM__CNTX1_PIX_EVENT_DONE       0x00800000L

// SQ_DEBUG_TP_FSM
#define SQ_DEBUG_TP_FSM__EX_TP_MASK                        0x00000007L
#define SQ_DEBUG_TP_FSM__RESERVED0_MASK                    0x00000008L
#define SQ_DEBUG_TP_FSM__RESERVED0                         0x00000008L
#define SQ_DEBUG_TP_FSM__CF_TP_MASK                        0x000000f0L
#define SQ_DEBUG_TP_FSM__IF_TP_MASK                        0x00000700L
#define SQ_DEBUG_TP_FSM__RESERVED1_MASK                    0x00000800L
#define SQ_DEBUG_TP_FSM__RESERVED1                         0x00000800L
#define SQ_DEBUG_TP_FSM__TIS_TP_MASK                       0x00003000L
#define SQ_DEBUG_TP_FSM__RESERVED2_MASK                    0x0000c000L
#define SQ_DEBUG_TP_FSM__GS_TP_MASK                        0x00030000L
#define SQ_DEBUG_TP_FSM__RESERVED3_MASK                    0x000c0000L
#define SQ_DEBUG_TP_FSM__FCR_TP_MASK                       0x00300000L
#define SQ_DEBUG_TP_FSM__RESERVED4_MASK                    0x00c00000L
#define SQ_DEBUG_TP_FSM__FCS_TP_MASK                       0x03000000L
#define SQ_DEBUG_TP_FSM__RESERVED5_MASK                    0x0c000000L
#define SQ_DEBUG_TP_FSM__ARB_TR_TP_MASK                    0x70000000L

// SQ_DEBUG_FSM_ALU_0
#define SQ_DEBUG_FSM_ALU_0__EX_ALU_0_MASK                  0x00000007L
#define SQ_DEBUG_FSM_ALU_0__RESERVED0_MASK                 0x00000008L
#define SQ_DEBUG_FSM_ALU_0__RESERVED0                      0x00000008L
#define SQ_DEBUG_FSM_ALU_0__CF_ALU_0_MASK                  0x000000f0L
#define SQ_DEBUG_FSM_ALU_0__IF_ALU_0_MASK                  0x00000700L
#define SQ_DEBUG_FSM_ALU_0__RESERVED1_MASK                 0x00000800L
#define SQ_DEBUG_FSM_ALU_0__RESERVED1                      0x00000800L
#define SQ_DEBUG_FSM_ALU_0__DU1_ALU_0_MASK                 0x00007000L
#define SQ_DEBUG_FSM_ALU_0__RESERVED2_MASK                 0x00008000L
#define SQ_DEBUG_FSM_ALU_0__RESERVED2                      0x00008000L
#define SQ_DEBUG_FSM_ALU_0__DU0_ALU_0_MASK                 0x00070000L
#define SQ_DEBUG_FSM_ALU_0__RESERVED3_MASK                 0x00080000L
#define SQ_DEBUG_FSM_ALU_0__RESERVED3                      0x00080000L
#define SQ_DEBUG_FSM_ALU_0__AIS_ALU_0_MASK                 0x00700000L
#define SQ_DEBUG_FSM_ALU_0__RESERVED4_MASK                 0x00800000L
#define SQ_DEBUG_FSM_ALU_0__RESERVED4                      0x00800000L
#define SQ_DEBUG_FSM_ALU_0__ACS_ALU_0_MASK                 0x07000000L
#define SQ_DEBUG_FSM_ALU_0__RESERVED5_MASK                 0x08000000L
#define SQ_DEBUG_FSM_ALU_0__RESERVED5                      0x08000000L
#define SQ_DEBUG_FSM_ALU_0__ARB_TR_ALU_MASK                0x70000000L

// SQ_DEBUG_FSM_ALU_1
#define SQ_DEBUG_FSM_ALU_1__EX_ALU_0_MASK                  0x00000007L
#define SQ_DEBUG_FSM_ALU_1__RESERVED0_MASK                 0x00000008L
#define SQ_DEBUG_FSM_ALU_1__RESERVED0                      0x00000008L
#define SQ_DEBUG_FSM_ALU_1__CF_ALU_0_MASK                  0x000000f0L
#define SQ_DEBUG_FSM_ALU_1__IF_ALU_0_MASK                  0x00000700L
#define SQ_DEBUG_FSM_ALU_1__RESERVED1_MASK                 0x00000800L
#define SQ_DEBUG_FSM_ALU_1__RESERVED1                      0x00000800L
#define SQ_DEBUG_FSM_ALU_1__DU1_ALU_0_MASK                 0x00007000L
#define SQ_DEBUG_FSM_ALU_1__RESERVED2_MASK                 0x00008000L
#define SQ_DEBUG_FSM_ALU_1__RESERVED2                      0x00008000L
#define SQ_DEBUG_FSM_ALU_1__DU0_ALU_0_MASK                 0x00070000L
#define SQ_DEBUG_FSM_ALU_1__RESERVED3_MASK                 0x00080000L
#define SQ_DEBUG_FSM_ALU_1__RESERVED3                      0x00080000L
#define SQ_DEBUG_FSM_ALU_1__AIS_ALU_0_MASK                 0x00700000L
#define SQ_DEBUG_FSM_ALU_1__RESERVED4_MASK                 0x00800000L
#define SQ_DEBUG_FSM_ALU_1__RESERVED4                      0x00800000L
#define SQ_DEBUG_FSM_ALU_1__ACS_ALU_0_MASK                 0x07000000L
#define SQ_DEBUG_FSM_ALU_1__RESERVED5_MASK                 0x08000000L
#define SQ_DEBUG_FSM_ALU_1__RESERVED5                      0x08000000L
#define SQ_DEBUG_FSM_ALU_1__ARB_TR_ALU_MASK                0x70000000L

// SQ_DEBUG_EXP_ALLOC
#define SQ_DEBUG_EXP_ALLOC__POS_BUF_AVAIL_MASK             0x0000000fL
#define SQ_DEBUG_EXP_ALLOC__COLOR_BUF_AVAIL_MASK           0x00000ff0L
#define SQ_DEBUG_EXP_ALLOC__EA_BUF_AVAIL_MASK              0x00007000L
#define SQ_DEBUG_EXP_ALLOC__RESERVED_MASK                  0x00008000L
#define SQ_DEBUG_EXP_ALLOC__RESERVED                       0x00008000L
#define SQ_DEBUG_EXP_ALLOC__ALLOC_TBL_BUF_AVAIL_MASK       0x003f0000L

// SQ_DEBUG_PTR_BUFF
#define SQ_DEBUG_PTR_BUFF__END_OF_BUFFER_MASK              0x00000001L
#define SQ_DEBUG_PTR_BUFF__END_OF_BUFFER                   0x00000001L
#define SQ_DEBUG_PTR_BUFF__DEALLOC_CNT_MASK                0x0000001eL
#define SQ_DEBUG_PTR_BUFF__QUAL_NEW_VECTOR_MASK            0x00000020L
#define SQ_DEBUG_PTR_BUFF__QUAL_NEW_VECTOR                 0x00000020L
#define SQ_DEBUG_PTR_BUFF__EVENT_CONTEXT_ID_MASK           0x000001c0L
#define SQ_DEBUG_PTR_BUFF__SC_EVENT_ID_MASK                0x00003e00L
#define SQ_DEBUG_PTR_BUFF__QUAL_EVENT_MASK                 0x00004000L
#define SQ_DEBUG_PTR_BUFF__QUAL_EVENT                      0x00004000L
#define SQ_DEBUG_PTR_BUFF__PRIM_TYPE_POLYGON_MASK          0x00008000L
#define SQ_DEBUG_PTR_BUFF__PRIM_TYPE_POLYGON               0x00008000L
#define SQ_DEBUG_PTR_BUFF__EF_EMPTY_MASK                   0x00010000L
#define SQ_DEBUG_PTR_BUFF__EF_EMPTY                        0x00010000L
#define SQ_DEBUG_PTR_BUFF__VTX_SYNC_CNT_MASK               0x0ffe0000L

// SQ_DEBUG_GPR_VTX
#define SQ_DEBUG_GPR_VTX__VTX_TAIL_PTR_MASK                0x0000007fL
#define SQ_DEBUG_GPR_VTX__RESERVED_MASK                    0x00000080L
#define SQ_DEBUG_GPR_VTX__RESERVED                         0x00000080L
#define SQ_DEBUG_GPR_VTX__VTX_HEAD_PTR_MASK                0x00007f00L
#define SQ_DEBUG_GPR_VTX__RESERVED1_MASK                   0x00008000L
#define SQ_DEBUG_GPR_VTX__RESERVED1                        0x00008000L
#define SQ_DEBUG_GPR_VTX__VTX_MAX_MASK                     0x007f0000L
#define SQ_DEBUG_GPR_VTX__RESERVED2_MASK                   0x00800000L
#define SQ_DEBUG_GPR_VTX__RESERVED2                        0x00800000L
#define SQ_DEBUG_GPR_VTX__VTX_FREE_MASK                    0x7f000000L

// SQ_DEBUG_GPR_PIX
#define SQ_DEBUG_GPR_PIX__PIX_TAIL_PTR_MASK                0x0000007fL
#define SQ_DEBUG_GPR_PIX__RESERVED_MASK                    0x00000080L
#define SQ_DEBUG_GPR_PIX__RESERVED                         0x00000080L
#define SQ_DEBUG_GPR_PIX__PIX_HEAD_PTR_MASK                0x00007f00L
#define SQ_DEBUG_GPR_PIX__RESERVED1_MASK                   0x00008000L
#define SQ_DEBUG_GPR_PIX__RESERVED1                        0x00008000L
#define SQ_DEBUG_GPR_PIX__PIX_MAX_MASK                     0x007f0000L
#define SQ_DEBUG_GPR_PIX__RESERVED2_MASK                   0x00800000L
#define SQ_DEBUG_GPR_PIX__RESERVED2                        0x00800000L
#define SQ_DEBUG_GPR_PIX__PIX_FREE_MASK                    0x7f000000L

// SQ_DEBUG_TB_STATUS_SEL
#define SQ_DEBUG_TB_STATUS_SEL__VTX_TB_STATUS_REG_SEL_MASK 0x0000000fL
#define SQ_DEBUG_TB_STATUS_SEL__VTX_TB_STATE_MEM_DW_SEL_MASK 0x00000070L
#define SQ_DEBUG_TB_STATUS_SEL__VTX_TB_STATE_MEM_RD_ADDR_MASK 0x00000780L
#define SQ_DEBUG_TB_STATUS_SEL__VTX_TB_STATE_MEM_RD_EN_MASK 0x00000800L
#define SQ_DEBUG_TB_STATUS_SEL__VTX_TB_STATE_MEM_RD_EN     0x00000800L
#define SQ_DEBUG_TB_STATUS_SEL__PIX_TB_STATE_MEM_RD_EN_MASK 0x00001000L
#define SQ_DEBUG_TB_STATUS_SEL__PIX_TB_STATE_MEM_RD_EN     0x00001000L
#define SQ_DEBUG_TB_STATUS_SEL__DEBUG_BUS_TRIGGER_SEL_MASK 0x0000c000L
#define SQ_DEBUG_TB_STATUS_SEL__PIX_TB_STATUS_REG_SEL_MASK 0x000f0000L
#define SQ_DEBUG_TB_STATUS_SEL__PIX_TB_STATE_MEM_DW_SEL_MASK 0x00700000L
#define SQ_DEBUG_TB_STATUS_SEL__PIX_TB_STATE_MEM_RD_ADDR_MASK 0x1f800000L
#define SQ_DEBUG_TB_STATUS_SEL__VC_THREAD_BUF_DLY_MASK     0x60000000L
#define SQ_DEBUG_TB_STATUS_SEL__DISABLE_STRICT_CTX_SYNC_MASK 0x80000000L
#define SQ_DEBUG_TB_STATUS_SEL__DISABLE_STRICT_CTX_SYNC    0x80000000L

// SQ_DEBUG_VTX_TB_0
#define SQ_DEBUG_VTX_TB_0__VTX_HEAD_PTR_Q_MASK             0x0000000fL
#define SQ_DEBUG_VTX_TB_0__TAIL_PTR_Q_MASK                 0x000000f0L
#define SQ_DEBUG_VTX_TB_0__FULL_CNT_Q_MASK                 0x00000f00L
#define SQ_DEBUG_VTX_TB_0__NXT_POS_ALLOC_CNT_MASK          0x0000f000L
#define SQ_DEBUG_VTX_TB_0__NXT_PC_ALLOC_CNT_MASK           0x000f0000L
#define SQ_DEBUG_VTX_TB_0__SX_EVENT_FULL_MASK              0x00100000L
#define SQ_DEBUG_VTX_TB_0__SX_EVENT_FULL                   0x00100000L
#define SQ_DEBUG_VTX_TB_0__BUSY_Q_MASK                     0x00200000L
#define SQ_DEBUG_VTX_TB_0__BUSY_Q                          0x00200000L

// SQ_DEBUG_VTX_TB_1
#define SQ_DEBUG_VTX_TB_1__VS_DONE_PTR_MASK                0x0000ffffL

// SQ_DEBUG_VTX_TB_STATUS_REG
#define SQ_DEBUG_VTX_TB_STATUS_REG__VS_STATUS_REG_MASK     0xffffffffL

// SQ_DEBUG_VTX_TB_STATE_MEM
#define SQ_DEBUG_VTX_TB_STATE_MEM__VS_STATE_MEM_MASK       0xffffffffL

// SQ_DEBUG_PIX_TB_0
#define SQ_DEBUG_PIX_TB_0__PIX_HEAD_PTR_MASK               0x0000003fL
#define SQ_DEBUG_PIX_TB_0__TAIL_PTR_MASK                   0x00000fc0L
#define SQ_DEBUG_PIX_TB_0__FULL_CNT_MASK                   0x0007f000L
#define SQ_DEBUG_PIX_TB_0__NXT_PIX_ALLOC_CNT_MASK          0x01f80000L
#define SQ_DEBUG_PIX_TB_0__NXT_PIX_EXP_CNT_MASK            0x7e000000L
#define SQ_DEBUG_PIX_TB_0__BUSY_MASK                       0x80000000L
#define SQ_DEBUG_PIX_TB_0__BUSY                            0x80000000L

// SQ_DEBUG_PIX_TB_STATUS_REG_0
#define SQ_DEBUG_PIX_TB_STATUS_REG_0__PIX_TB_STATUS_REG_0_MASK 0xffffffffL

// SQ_DEBUG_PIX_TB_STATUS_REG_1
#define SQ_DEBUG_PIX_TB_STATUS_REG_1__PIX_TB_STATUS_REG_1_MASK 0xffffffffL

// SQ_DEBUG_PIX_TB_STATUS_REG_2
#define SQ_DEBUG_PIX_TB_STATUS_REG_2__PIX_TB_STATUS_REG_2_MASK 0xffffffffL

// SQ_DEBUG_PIX_TB_STATUS_REG_3
#define SQ_DEBUG_PIX_TB_STATUS_REG_3__PIX_TB_STATUS_REG_3_MASK 0xffffffffL

// SQ_DEBUG_PIX_TB_STATE_MEM
#define SQ_DEBUG_PIX_TB_STATE_MEM__PIX_TB_STATE_MEM_MASK   0xffffffffL

// SQ_PERFCOUNTER0_SELECT
#define SQ_PERFCOUNTER0_SELECT__PERF_SEL_MASK              0x000000ffL

// SQ_PERFCOUNTER1_SELECT
#define SQ_PERFCOUNTER1_SELECT__PERF_SEL_MASK              0x000000ffL

// SQ_PERFCOUNTER2_SELECT
#define SQ_PERFCOUNTER2_SELECT__PERF_SEL_MASK              0x000000ffL

// SQ_PERFCOUNTER3_SELECT
#define SQ_PERFCOUNTER3_SELECT__PERF_SEL_MASK              0x000000ffL

// SQ_PERFCOUNTER0_LOW
#define SQ_PERFCOUNTER0_LOW__PERF_COUNT_MASK               0xffffffffL

// SQ_PERFCOUNTER0_HI
#define SQ_PERFCOUNTER0_HI__PERF_COUNT_MASK                0x0000ffffL

// SQ_PERFCOUNTER1_LOW
#define SQ_PERFCOUNTER1_LOW__PERF_COUNT_MASK               0xffffffffL

// SQ_PERFCOUNTER1_HI
#define SQ_PERFCOUNTER1_HI__PERF_COUNT_MASK                0x0000ffffL

// SQ_PERFCOUNTER2_LOW
#define SQ_PERFCOUNTER2_LOW__PERF_COUNT_MASK               0xffffffffL

// SQ_PERFCOUNTER2_HI
#define SQ_PERFCOUNTER2_HI__PERF_COUNT_MASK                0x0000ffffL

// SQ_PERFCOUNTER3_LOW
#define SQ_PERFCOUNTER3_LOW__PERF_COUNT_MASK               0xffffffffL

// SQ_PERFCOUNTER3_HI
#define SQ_PERFCOUNTER3_HI__PERF_COUNT_MASK                0x0000ffffL

// SX_PERFCOUNTER0_SELECT
#define SX_PERFCOUNTER0_SELECT__PERF_SEL_MASK              0x000000ffL

// SX_PERFCOUNTER0_LOW
#define SX_PERFCOUNTER0_LOW__PERF_COUNT_MASK               0xffffffffL

// SX_PERFCOUNTER0_HI
#define SX_PERFCOUNTER0_HI__PERF_COUNT_MASK                0x0000ffffL

// SQ_INSTRUCTION_ALU_0
#define SQ_INSTRUCTION_ALU_0__VECTOR_RESULT_MASK           0x0000003fL
#define SQ_INSTRUCTION_ALU_0__CST_0_ABS_MOD_MASK           0x00000040L
#define SQ_INSTRUCTION_ALU_0__CST_0_ABS_MOD                0x00000040L
#define SQ_INSTRUCTION_ALU_0__LOW_PRECISION_16B_FP_MASK    0x00000080L
#define SQ_INSTRUCTION_ALU_0__LOW_PRECISION_16B_FP         0x00000080L
#define SQ_INSTRUCTION_ALU_0__SCALAR_RESULT_MASK           0x00003f00L
#define SQ_INSTRUCTION_ALU_0__SST_0_ABS_MOD_MASK           0x00004000L
#define SQ_INSTRUCTION_ALU_0__SST_0_ABS_MOD                0x00004000L
#define SQ_INSTRUCTION_ALU_0__EXPORT_DATA_MASK             0x00008000L
#define SQ_INSTRUCTION_ALU_0__EXPORT_DATA                  0x00008000L
#define SQ_INSTRUCTION_ALU_0__VECTOR_WRT_MSK_MASK          0x000f0000L
#define SQ_INSTRUCTION_ALU_0__SCALAR_WRT_MSK_MASK          0x00f00000L
#define SQ_INSTRUCTION_ALU_0__VECTOR_CLAMP_MASK            0x01000000L
#define SQ_INSTRUCTION_ALU_0__VECTOR_CLAMP                 0x01000000L
#define SQ_INSTRUCTION_ALU_0__SCALAR_CLAMP_MASK            0x02000000L
#define SQ_INSTRUCTION_ALU_0__SCALAR_CLAMP                 0x02000000L
#define SQ_INSTRUCTION_ALU_0__SCALAR_OPCODE_MASK           0xfc000000L

// SQ_INSTRUCTION_ALU_1
#define SQ_INSTRUCTION_ALU_1__SRC_C_SWIZZLE_R_MASK         0x00000003L
#define SQ_INSTRUCTION_ALU_1__SRC_C_SWIZZLE_G_MASK         0x0000000cL
#define SQ_INSTRUCTION_ALU_1__SRC_C_SWIZZLE_B_MASK         0x00000030L
#define SQ_INSTRUCTION_ALU_1__SRC_C_SWIZZLE_A_MASK         0x000000c0L
#define SQ_INSTRUCTION_ALU_1__SRC_B_SWIZZLE_R_MASK         0x00000300L
#define SQ_INSTRUCTION_ALU_1__SRC_B_SWIZZLE_G_MASK         0x00000c00L
#define SQ_INSTRUCTION_ALU_1__SRC_B_SWIZZLE_B_MASK         0x00003000L
#define SQ_INSTRUCTION_ALU_1__SRC_B_SWIZZLE_A_MASK         0x0000c000L
#define SQ_INSTRUCTION_ALU_1__SRC_A_SWIZZLE_R_MASK         0x00030000L
#define SQ_INSTRUCTION_ALU_1__SRC_A_SWIZZLE_G_MASK         0x000c0000L
#define SQ_INSTRUCTION_ALU_1__SRC_A_SWIZZLE_B_MASK         0x00300000L
#define SQ_INSTRUCTION_ALU_1__SRC_A_SWIZZLE_A_MASK         0x00c00000L
#define SQ_INSTRUCTION_ALU_1__SRC_C_ARG_MOD_MASK           0x01000000L
#define SQ_INSTRUCTION_ALU_1__SRC_C_ARG_MOD                0x01000000L
#define SQ_INSTRUCTION_ALU_1__SRC_B_ARG_MOD_MASK           0x02000000L
#define SQ_INSTRUCTION_ALU_1__SRC_B_ARG_MOD                0x02000000L
#define SQ_INSTRUCTION_ALU_1__SRC_A_ARG_MOD_MASK           0x04000000L
#define SQ_INSTRUCTION_ALU_1__SRC_A_ARG_MOD                0x04000000L
#define SQ_INSTRUCTION_ALU_1__PRED_SELECT_MASK             0x18000000L
#define SQ_INSTRUCTION_ALU_1__RELATIVE_ADDR_MASK           0x20000000L
#define SQ_INSTRUCTION_ALU_1__RELATIVE_ADDR                0x20000000L
#define SQ_INSTRUCTION_ALU_1__CONST_1_REL_ABS_MASK         0x40000000L
#define SQ_INSTRUCTION_ALU_1__CONST_1_REL_ABS              0x40000000L
#define SQ_INSTRUCTION_ALU_1__CONST_0_REL_ABS_MASK         0x80000000L
#define SQ_INSTRUCTION_ALU_1__CONST_0_REL_ABS              0x80000000L

// SQ_INSTRUCTION_ALU_2
#define SQ_INSTRUCTION_ALU_2__SRC_C_REG_PTR_MASK           0x0000003fL
#define SQ_INSTRUCTION_ALU_2__REG_SELECT_C_MASK            0x00000040L
#define SQ_INSTRUCTION_ALU_2__REG_SELECT_C                 0x00000040L
#define SQ_INSTRUCTION_ALU_2__REG_ABS_MOD_C_MASK           0x00000080L
#define SQ_INSTRUCTION_ALU_2__REG_ABS_MOD_C                0x00000080L
#define SQ_INSTRUCTION_ALU_2__SRC_B_REG_PTR_MASK           0x00003f00L
#define SQ_INSTRUCTION_ALU_2__REG_SELECT_B_MASK            0x00004000L
#define SQ_INSTRUCTION_ALU_2__REG_SELECT_B                 0x00004000L
#define SQ_INSTRUCTION_ALU_2__REG_ABS_MOD_B_MASK           0x00008000L
#define SQ_INSTRUCTION_ALU_2__REG_ABS_MOD_B                0x00008000L
#define SQ_INSTRUCTION_ALU_2__SRC_A_REG_PTR_MASK           0x003f0000L
#define SQ_INSTRUCTION_ALU_2__REG_SELECT_A_MASK            0x00400000L
#define SQ_INSTRUCTION_ALU_2__REG_SELECT_A                 0x00400000L
#define SQ_INSTRUCTION_ALU_2__REG_ABS_MOD_A_MASK           0x00800000L
#define SQ_INSTRUCTION_ALU_2__REG_ABS_MOD_A                0x00800000L
#define SQ_INSTRUCTION_ALU_2__VECTOR_OPCODE_MASK           0x1f000000L
#define SQ_INSTRUCTION_ALU_2__SRC_C_SEL_MASK               0x20000000L
#define SQ_INSTRUCTION_ALU_2__SRC_C_SEL                    0x20000000L
#define SQ_INSTRUCTION_ALU_2__SRC_B_SEL_MASK               0x40000000L
#define SQ_INSTRUCTION_ALU_2__SRC_B_SEL                    0x40000000L
#define SQ_INSTRUCTION_ALU_2__SRC_A_SEL_MASK               0x80000000L
#define SQ_INSTRUCTION_ALU_2__SRC_A_SEL                    0x80000000L

// SQ_INSTRUCTION_CF_EXEC_0
#define SQ_INSTRUCTION_CF_EXEC_0__ADDRESS_MASK             0x000001ffL
#define SQ_INSTRUCTION_CF_EXEC_0__RESERVED_MASK            0x00000e00L
#define SQ_INSTRUCTION_CF_EXEC_0__COUNT_MASK               0x00007000L
#define SQ_INSTRUCTION_CF_EXEC_0__YIELD_MASK               0x00008000L
#define SQ_INSTRUCTION_CF_EXEC_0__YIELD                    0x00008000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_0_MASK         0x00010000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_0              0x00010000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_0_MASK       0x00020000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_0            0x00020000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_1_MASK         0x00040000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_1              0x00040000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_1_MASK       0x00080000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_1            0x00080000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_2_MASK         0x00100000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_2              0x00100000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_2_MASK       0x00200000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_2            0x00200000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_3_MASK         0x00400000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_3              0x00400000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_3_MASK       0x00800000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_3            0x00800000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_4_MASK         0x01000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_4              0x01000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_4_MASK       0x02000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_4            0x02000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_5_MASK         0x04000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_5              0x04000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_5_MASK       0x08000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_5            0x08000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_0_MASK           0x10000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_0                0x10000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_1_MASK           0x20000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_1                0x20000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_2_MASK           0x40000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_2                0x40000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_3_MASK           0x80000000L
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_3                0x80000000L

// SQ_INSTRUCTION_CF_EXEC_1
#define SQ_INSTRUCTION_CF_EXEC_1__INST_VC_4_MASK           0x00000001L
#define SQ_INSTRUCTION_CF_EXEC_1__INST_VC_4                0x00000001L
#define SQ_INSTRUCTION_CF_EXEC_1__INST_VC_5_MASK           0x00000002L
#define SQ_INSTRUCTION_CF_EXEC_1__INST_VC_5                0x00000002L
#define SQ_INSTRUCTION_CF_EXEC_1__BOOL_ADDR_MASK           0x000003fcL
#define SQ_INSTRUCTION_CF_EXEC_1__CONDITION_MASK           0x00000400L
#define SQ_INSTRUCTION_CF_EXEC_1__CONDITION                0x00000400L
#define SQ_INSTRUCTION_CF_EXEC_1__ADDRESS_MODE_MASK        0x00000800L
#define SQ_INSTRUCTION_CF_EXEC_1__ADDRESS_MODE             0x00000800L
#define SQ_INSTRUCTION_CF_EXEC_1__OPCODE_MASK              0x0000f000L
#define SQ_INSTRUCTION_CF_EXEC_1__ADDRESS_MASK             0x01ff0000L
#define SQ_INSTRUCTION_CF_EXEC_1__RESERVED_MASK            0x0e000000L
#define SQ_INSTRUCTION_CF_EXEC_1__COUNT_MASK               0x70000000L
#define SQ_INSTRUCTION_CF_EXEC_1__YIELD_MASK               0x80000000L
#define SQ_INSTRUCTION_CF_EXEC_1__YIELD                    0x80000000L

// SQ_INSTRUCTION_CF_EXEC_2
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_0_MASK         0x00000001L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_0              0x00000001L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_0_MASK       0x00000002L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_0            0x00000002L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_1_MASK         0x00000004L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_1              0x00000004L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_1_MASK       0x00000008L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_1            0x00000008L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_2_MASK         0x00000010L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_2              0x00000010L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_2_MASK       0x00000020L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_2            0x00000020L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_3_MASK         0x00000040L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_3              0x00000040L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_3_MASK       0x00000080L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_3            0x00000080L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_4_MASK         0x00000100L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_4              0x00000100L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_4_MASK       0x00000200L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_4            0x00000200L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_5_MASK         0x00000400L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_5              0x00000400L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_5_MASK       0x00000800L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_5            0x00000800L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_0_MASK           0x00001000L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_0                0x00001000L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_1_MASK           0x00002000L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_1                0x00002000L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_2_MASK           0x00004000L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_2                0x00004000L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_3_MASK           0x00008000L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_3                0x00008000L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_4_MASK           0x00010000L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_4                0x00010000L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_5_MASK           0x00020000L
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_5                0x00020000L
#define SQ_INSTRUCTION_CF_EXEC_2__BOOL_ADDR_MASK           0x03fc0000L
#define SQ_INSTRUCTION_CF_EXEC_2__CONDITION_MASK           0x04000000L
#define SQ_INSTRUCTION_CF_EXEC_2__CONDITION                0x04000000L
#define SQ_INSTRUCTION_CF_EXEC_2__ADDRESS_MODE_MASK        0x08000000L
#define SQ_INSTRUCTION_CF_EXEC_2__ADDRESS_MODE             0x08000000L
#define SQ_INSTRUCTION_CF_EXEC_2__OPCODE_MASK              0xf0000000L

// SQ_INSTRUCTION_CF_LOOP_0
#define SQ_INSTRUCTION_CF_LOOP_0__ADDRESS_MASK             0x000003ffL
#define SQ_INSTRUCTION_CF_LOOP_0__RESERVED_0_MASK          0x0000fc00L
#define SQ_INSTRUCTION_CF_LOOP_0__LOOP_ID_MASK             0x001f0000L
#define SQ_INSTRUCTION_CF_LOOP_0__RESERVED_1_MASK          0xffe00000L

// SQ_INSTRUCTION_CF_LOOP_1
#define SQ_INSTRUCTION_CF_LOOP_1__RESERVED_0_MASK          0x000007ffL
#define SQ_INSTRUCTION_CF_LOOP_1__ADDRESS_MODE_MASK        0x00000800L
#define SQ_INSTRUCTION_CF_LOOP_1__ADDRESS_MODE             0x00000800L
#define SQ_INSTRUCTION_CF_LOOP_1__OPCODE_MASK              0x0000f000L
#define SQ_INSTRUCTION_CF_LOOP_1__ADDRESS_MASK             0x03ff0000L
#define SQ_INSTRUCTION_CF_LOOP_1__RESERVED_1_MASK          0xfc000000L

// SQ_INSTRUCTION_CF_LOOP_2
#define SQ_INSTRUCTION_CF_LOOP_2__LOOP_ID_MASK             0x0000001fL
#define SQ_INSTRUCTION_CF_LOOP_2__RESERVED_MASK            0x07ffffe0L
#define SQ_INSTRUCTION_CF_LOOP_2__ADDRESS_MODE_MASK        0x08000000L
#define SQ_INSTRUCTION_CF_LOOP_2__ADDRESS_MODE             0x08000000L
#define SQ_INSTRUCTION_CF_LOOP_2__OPCODE_MASK              0xf0000000L

// SQ_INSTRUCTION_CF_JMP_CALL_0
#define SQ_INSTRUCTION_CF_JMP_CALL_0__ADDRESS_MASK         0x000003ffL
#define SQ_INSTRUCTION_CF_JMP_CALL_0__RESERVED_0_MASK      0x00001c00L
#define SQ_INSTRUCTION_CF_JMP_CALL_0__FORCE_CALL_MASK      0x00002000L
#define SQ_INSTRUCTION_CF_JMP_CALL_0__FORCE_CALL           0x00002000L
#define SQ_INSTRUCTION_CF_JMP_CALL_0__PREDICATED_JMP_MASK  0x00004000L
#define SQ_INSTRUCTION_CF_JMP_CALL_0__PREDICATED_JMP       0x00004000L
#define SQ_INSTRUCTION_CF_JMP_CALL_0__RESERVED_1_MASK      0xffff8000L

// SQ_INSTRUCTION_CF_JMP_CALL_1
#define SQ_INSTRUCTION_CF_JMP_CALL_1__RESERVED_0_MASK      0x00000001L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__RESERVED_0           0x00000001L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__DIRECTION_MASK       0x00000002L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__DIRECTION            0x00000002L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__BOOL_ADDR_MASK       0x000003fcL
#define SQ_INSTRUCTION_CF_JMP_CALL_1__CONDITION_MASK       0x00000400L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__CONDITION            0x00000400L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__ADDRESS_MODE_MASK    0x00000800L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__ADDRESS_MODE         0x00000800L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__OPCODE_MASK          0x0000f000L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__ADDRESS_MASK         0x03ff0000L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__RESERVED_1_MASK      0x1c000000L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__FORCE_CALL_MASK      0x20000000L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__FORCE_CALL           0x20000000L
#define SQ_INSTRUCTION_CF_JMP_CALL_1__RESERVED_2_MASK      0xc0000000L

// SQ_INSTRUCTION_CF_JMP_CALL_2
#define SQ_INSTRUCTION_CF_JMP_CALL_2__RESERVED_MASK        0x0001ffffL
#define SQ_INSTRUCTION_CF_JMP_CALL_2__DIRECTION_MASK       0x00020000L
#define SQ_INSTRUCTION_CF_JMP_CALL_2__DIRECTION            0x00020000L
#define SQ_INSTRUCTION_CF_JMP_CALL_2__BOOL_ADDR_MASK       0x03fc0000L
#define SQ_INSTRUCTION_CF_JMP_CALL_2__CONDITION_MASK       0x04000000L
#define SQ_INSTRUCTION_CF_JMP_CALL_2__CONDITION            0x04000000L
#define SQ_INSTRUCTION_CF_JMP_CALL_2__ADDRESS_MODE_MASK    0x08000000L
#define SQ_INSTRUCTION_CF_JMP_CALL_2__ADDRESS_MODE         0x08000000L
#define SQ_INSTRUCTION_CF_JMP_CALL_2__OPCODE_MASK          0xf0000000L

// SQ_INSTRUCTION_CF_ALLOC_0
#define SQ_INSTRUCTION_CF_ALLOC_0__SIZE_MASK               0x0000000fL
#define SQ_INSTRUCTION_CF_ALLOC_0__RESERVED_MASK           0xfffffff0L

// SQ_INSTRUCTION_CF_ALLOC_1
#define SQ_INSTRUCTION_CF_ALLOC_1__RESERVED_0_MASK         0x000000ffL
#define SQ_INSTRUCTION_CF_ALLOC_1__NO_SERIAL_MASK          0x00000100L
#define SQ_INSTRUCTION_CF_ALLOC_1__NO_SERIAL               0x00000100L
#define SQ_INSTRUCTION_CF_ALLOC_1__BUFFER_SELECT_MASK      0x00000600L
#define SQ_INSTRUCTION_CF_ALLOC_1__ALLOC_MODE_MASK         0x00000800L
#define SQ_INSTRUCTION_CF_ALLOC_1__ALLOC_MODE              0x00000800L
#define SQ_INSTRUCTION_CF_ALLOC_1__OPCODE_MASK             0x0000f000L
#define SQ_INSTRUCTION_CF_ALLOC_1__SIZE_MASK               0x000f0000L
#define SQ_INSTRUCTION_CF_ALLOC_1__RESERVED_1_MASK         0xfff00000L

// SQ_INSTRUCTION_CF_ALLOC_2
#define SQ_INSTRUCTION_CF_ALLOC_2__RESERVED_MASK           0x00ffffffL
#define SQ_INSTRUCTION_CF_ALLOC_2__NO_SERIAL_MASK          0x01000000L
#define SQ_INSTRUCTION_CF_ALLOC_2__NO_SERIAL               0x01000000L
#define SQ_INSTRUCTION_CF_ALLOC_2__BUFFER_SELECT_MASK      0x06000000L
#define SQ_INSTRUCTION_CF_ALLOC_2__ALLOC_MODE_MASK         0x08000000L
#define SQ_INSTRUCTION_CF_ALLOC_2__ALLOC_MODE              0x08000000L
#define SQ_INSTRUCTION_CF_ALLOC_2__OPCODE_MASK             0xf0000000L

// SQ_INSTRUCTION_TFETCH_0
#define SQ_INSTRUCTION_TFETCH_0__OPCODE_MASK               0x0000001fL
#define SQ_INSTRUCTION_TFETCH_0__SRC_GPR_MASK              0x000007e0L
#define SQ_INSTRUCTION_TFETCH_0__SRC_GPR_AM_MASK           0x00000800L
#define SQ_INSTRUCTION_TFETCH_0__SRC_GPR_AM                0x00000800L
#define SQ_INSTRUCTION_TFETCH_0__DST_GPR_MASK              0x0003f000L
#define SQ_INSTRUCTION_TFETCH_0__DST_GPR_AM_MASK           0x00040000L
#define SQ_INSTRUCTION_TFETCH_0__DST_GPR_AM                0x00040000L
#define SQ_INSTRUCTION_TFETCH_0__FETCH_VALID_ONLY_MASK     0x00080000L
#define SQ_INSTRUCTION_TFETCH_0__FETCH_VALID_ONLY          0x00080000L
#define SQ_INSTRUCTION_TFETCH_0__CONST_INDEX_MASK          0x01f00000L
#define SQ_INSTRUCTION_TFETCH_0__TX_COORD_DENORM_MASK      0x02000000L
#define SQ_INSTRUCTION_TFETCH_0__TX_COORD_DENORM           0x02000000L
#define SQ_INSTRUCTION_TFETCH_0__SRC_SEL_X_MASK            0x0c000000L
#define SQ_INSTRUCTION_TFETCH_0__SRC_SEL_Y_MASK            0x30000000L
#define SQ_INSTRUCTION_TFETCH_0__SRC_SEL_Z_MASK            0xc0000000L

// SQ_INSTRUCTION_TFETCH_1
#define SQ_INSTRUCTION_TFETCH_1__DST_SEL_X_MASK            0x00000007L
#define SQ_INSTRUCTION_TFETCH_1__DST_SEL_Y_MASK            0x00000038L
#define SQ_INSTRUCTION_TFETCH_1__DST_SEL_Z_MASK            0x000001c0L
#define SQ_INSTRUCTION_TFETCH_1__DST_SEL_W_MASK            0x00000e00L
#define SQ_INSTRUCTION_TFETCH_1__MAG_FILTER_MASK           0x00003000L
#define SQ_INSTRUCTION_TFETCH_1__MIN_FILTER_MASK           0x0000c000L
#define SQ_INSTRUCTION_TFETCH_1__MIP_FILTER_MASK           0x00030000L
#define SQ_INSTRUCTION_TFETCH_1__ANISO_FILTER_MASK         0x001c0000L
#define SQ_INSTRUCTION_TFETCH_1__ARBITRARY_FILTER_MASK     0x00e00000L
#define SQ_INSTRUCTION_TFETCH_1__VOL_MAG_FILTER_MASK       0x03000000L
#define SQ_INSTRUCTION_TFETCH_1__VOL_MIN_FILTER_MASK       0x0c000000L
#define SQ_INSTRUCTION_TFETCH_1__USE_COMP_LOD_MASK         0x10000000L
#define SQ_INSTRUCTION_TFETCH_1__USE_COMP_LOD              0x10000000L
#define SQ_INSTRUCTION_TFETCH_1__USE_REG_LOD_MASK          0x60000000L
#define SQ_INSTRUCTION_TFETCH_1__PRED_SELECT_MASK          0x80000000L
#define SQ_INSTRUCTION_TFETCH_1__PRED_SELECT               0x80000000L

// SQ_INSTRUCTION_TFETCH_2
#define SQ_INSTRUCTION_TFETCH_2__USE_REG_GRADIENTS_MASK    0x00000001L
#define SQ_INSTRUCTION_TFETCH_2__USE_REG_GRADIENTS         0x00000001L
#define SQ_INSTRUCTION_TFETCH_2__SAMPLE_LOCATION_MASK      0x00000002L
#define SQ_INSTRUCTION_TFETCH_2__SAMPLE_LOCATION           0x00000002L
#define SQ_INSTRUCTION_TFETCH_2__LOD_BIAS_MASK             0x000001fcL
#define SQ_INSTRUCTION_TFETCH_2__UNUSED_MASK               0x0000fe00L
#define SQ_INSTRUCTION_TFETCH_2__OFFSET_X_MASK             0x001f0000L
#define SQ_INSTRUCTION_TFETCH_2__OFFSET_Y_MASK             0x03e00000L
#define SQ_INSTRUCTION_TFETCH_2__OFFSET_Z_MASK             0x7c000000L
#define SQ_INSTRUCTION_TFETCH_2__PRED_CONDITION_MASK       0x80000000L
#define SQ_INSTRUCTION_TFETCH_2__PRED_CONDITION            0x80000000L

// SQ_INSTRUCTION_VFETCH_0
#define SQ_INSTRUCTION_VFETCH_0__OPCODE_MASK               0x0000001fL
#define SQ_INSTRUCTION_VFETCH_0__SRC_GPR_MASK              0x000007e0L
#define SQ_INSTRUCTION_VFETCH_0__SRC_GPR_AM_MASK           0x00000800L
#define SQ_INSTRUCTION_VFETCH_0__SRC_GPR_AM                0x00000800L
#define SQ_INSTRUCTION_VFETCH_0__DST_GPR_MASK              0x0003f000L
#define SQ_INSTRUCTION_VFETCH_0__DST_GPR_AM_MASK           0x00040000L
#define SQ_INSTRUCTION_VFETCH_0__DST_GPR_AM                0x00040000L
#define SQ_INSTRUCTION_VFETCH_0__MUST_BE_ONE_MASK          0x00080000L
#define SQ_INSTRUCTION_VFETCH_0__MUST_BE_ONE               0x00080000L
#define SQ_INSTRUCTION_VFETCH_0__CONST_INDEX_MASK          0x01f00000L
#define SQ_INSTRUCTION_VFETCH_0__CONST_INDEX_SEL_MASK      0x06000000L
#define SQ_INSTRUCTION_VFETCH_0__SRC_SEL_MASK              0xc0000000L

// SQ_INSTRUCTION_VFETCH_1
#define SQ_INSTRUCTION_VFETCH_1__DST_SEL_X_MASK            0x00000007L
#define SQ_INSTRUCTION_VFETCH_1__DST_SEL_Y_MASK            0x00000038L
#define SQ_INSTRUCTION_VFETCH_1__DST_SEL_Z_MASK            0x000001c0L
#define SQ_INSTRUCTION_VFETCH_1__DST_SEL_W_MASK            0x00000e00L
#define SQ_INSTRUCTION_VFETCH_1__FORMAT_COMP_ALL_MASK      0x00001000L
#define SQ_INSTRUCTION_VFETCH_1__FORMAT_COMP_ALL           0x00001000L
#define SQ_INSTRUCTION_VFETCH_1__NUM_FORMAT_ALL_MASK       0x00002000L
#define SQ_INSTRUCTION_VFETCH_1__NUM_FORMAT_ALL            0x00002000L
#define SQ_INSTRUCTION_VFETCH_1__SIGNED_RF_MODE_ALL_MASK   0x00004000L
#define SQ_INSTRUCTION_VFETCH_1__SIGNED_RF_MODE_ALL        0x00004000L
#define SQ_INSTRUCTION_VFETCH_1__DATA_FORMAT_MASK          0x003f0000L
#define SQ_INSTRUCTION_VFETCH_1__EXP_ADJUST_ALL_MASK       0x3f800000L
#define SQ_INSTRUCTION_VFETCH_1__PRED_SELECT_MASK          0x80000000L
#define SQ_INSTRUCTION_VFETCH_1__PRED_SELECT               0x80000000L

// SQ_INSTRUCTION_VFETCH_2
#define SQ_INSTRUCTION_VFETCH_2__STRIDE_MASK               0x000000ffL
#define SQ_INSTRUCTION_VFETCH_2__OFFSET_MASK               0x00ff0000L
#define SQ_INSTRUCTION_VFETCH_2__PRED_CONDITION_MASK       0x80000000L
#define SQ_INSTRUCTION_VFETCH_2__PRED_CONDITION            0x80000000L

// SQ_CONSTANT_0
#define SQ_CONSTANT_0__RED_MASK                            0xffffffffL

// SQ_CONSTANT_1
#define SQ_CONSTANT_1__GREEN_MASK                          0xffffffffL

// SQ_CONSTANT_2
#define SQ_CONSTANT_2__BLUE_MASK                           0xffffffffL

// SQ_CONSTANT_3
#define SQ_CONSTANT_3__ALPHA_MASK                          0xffffffffL

// SQ_FETCH_0
#define SQ_FETCH_0__VALUE_MASK                             0xffffffffL

// SQ_FETCH_1
#define SQ_FETCH_1__VALUE_MASK                             0xffffffffL

// SQ_FETCH_2
#define SQ_FETCH_2__VALUE_MASK                             0xffffffffL

// SQ_FETCH_3
#define SQ_FETCH_3__VALUE_MASK                             0xffffffffL

// SQ_FETCH_4
#define SQ_FETCH_4__VALUE_MASK                             0xffffffffL

// SQ_FETCH_5
#define SQ_FETCH_5__VALUE_MASK                             0xffffffffL

// SQ_CONSTANT_VFETCH_0
#define SQ_CONSTANT_VFETCH_0__TYPE_MASK                    0x00000001L
#define SQ_CONSTANT_VFETCH_0__TYPE                         0x00000001L
#define SQ_CONSTANT_VFETCH_0__STATE_MASK                   0x00000002L
#define SQ_CONSTANT_VFETCH_0__STATE                        0x00000002L
#define SQ_CONSTANT_VFETCH_0__BASE_ADDRESS_MASK            0xfffffffcL

// SQ_CONSTANT_VFETCH_1
#define SQ_CONSTANT_VFETCH_1__ENDIAN_SWAP_MASK             0x00000003L
#define SQ_CONSTANT_VFETCH_1__LIMIT_ADDRESS_MASK           0xfffffffcL

// SQ_CONSTANT_T2
#define SQ_CONSTANT_T2__VALUE_MASK                         0xffffffffL

// SQ_CONSTANT_T3
#define SQ_CONSTANT_T3__VALUE_MASK                         0xffffffffL

// SQ_CF_BOOLEANS
#define SQ_CF_BOOLEANS__CF_BOOLEANS_0_MASK                 0x000000ffL
#define SQ_CF_BOOLEANS__CF_BOOLEANS_1_MASK                 0x0000ff00L
#define SQ_CF_BOOLEANS__CF_BOOLEANS_2_MASK                 0x00ff0000L
#define SQ_CF_BOOLEANS__CF_BOOLEANS_3_MASK                 0xff000000L

// SQ_CF_LOOP
#define SQ_CF_LOOP__CF_LOOP_COUNT_MASK                     0x000000ffL
#define SQ_CF_LOOP__CF_LOOP_START_MASK                     0x0000ff00L
#define SQ_CF_LOOP__CF_LOOP_STEP_MASK                      0x00ff0000L

// SQ_CONSTANT_RT_0
#define SQ_CONSTANT_RT_0__RED_MASK                         0xffffffffL

// SQ_CONSTANT_RT_1
#define SQ_CONSTANT_RT_1__GREEN_MASK                       0xffffffffL

// SQ_CONSTANT_RT_2
#define SQ_CONSTANT_RT_2__BLUE_MASK                        0xffffffffL

// SQ_CONSTANT_RT_3
#define SQ_CONSTANT_RT_3__ALPHA_MASK                       0xffffffffL

// SQ_FETCH_RT_0
#define SQ_FETCH_RT_0__VALUE_MASK                          0xffffffffL

// SQ_FETCH_RT_1
#define SQ_FETCH_RT_1__VALUE_MASK                          0xffffffffL

// SQ_FETCH_RT_2
#define SQ_FETCH_RT_2__VALUE_MASK                          0xffffffffL

// SQ_FETCH_RT_3
#define SQ_FETCH_RT_3__VALUE_MASK                          0xffffffffL

// SQ_FETCH_RT_4
#define SQ_FETCH_RT_4__VALUE_MASK                          0xffffffffL

// SQ_FETCH_RT_5
#define SQ_FETCH_RT_5__VALUE_MASK                          0xffffffffL

// SQ_CF_RT_BOOLEANS
#define SQ_CF_RT_BOOLEANS__CF_BOOLEANS_0_MASK              0x000000ffL
#define SQ_CF_RT_BOOLEANS__CF_BOOLEANS_1_MASK              0x0000ff00L
#define SQ_CF_RT_BOOLEANS__CF_BOOLEANS_2_MASK              0x00ff0000L
#define SQ_CF_RT_BOOLEANS__CF_BOOLEANS_3_MASK              0xff000000L

// SQ_CF_RT_LOOP
#define SQ_CF_RT_LOOP__CF_LOOP_COUNT_MASK                  0x000000ffL
#define SQ_CF_RT_LOOP__CF_LOOP_START_MASK                  0x0000ff00L
#define SQ_CF_RT_LOOP__CF_LOOP_STEP_MASK                   0x00ff0000L

// SQ_VS_PROGRAM
#define SQ_VS_PROGRAM__BASE_MASK                           0x00000fffL
#define SQ_VS_PROGRAM__SIZE_MASK                           0x00fff000L

// SQ_PS_PROGRAM
#define SQ_PS_PROGRAM__BASE_MASK                           0x00000fffL
#define SQ_PS_PROGRAM__SIZE_MASK                           0x00fff000L

// SQ_CF_PROGRAM_SIZE
#define SQ_CF_PROGRAM_SIZE__VS_CF_SIZE_MASK                0x000007ffL
#define SQ_CF_PROGRAM_SIZE__PS_CF_SIZE_MASK                0x007ff000L

// SQ_INTERPOLATOR_CNTL
#define SQ_INTERPOLATOR_CNTL__PARAM_SHADE_MASK             0x0000ffffL
#define SQ_INTERPOLATOR_CNTL__SAMPLING_PATTERN_MASK        0xffff0000L

// SQ_PROGRAM_CNTL
#define SQ_PROGRAM_CNTL__VS_NUM_REG_MASK                   0x0000003fL
#define SQ_PROGRAM_CNTL__PS_NUM_REG_MASK                   0x00003f00L
#define SQ_PROGRAM_CNTL__VS_RESOURCE_MASK                  0x00010000L
#define SQ_PROGRAM_CNTL__VS_RESOURCE                       0x00010000L
#define SQ_PROGRAM_CNTL__PS_RESOURCE_MASK                  0x00020000L
#define SQ_PROGRAM_CNTL__PS_RESOURCE                       0x00020000L
#define SQ_PROGRAM_CNTL__PARAM_GEN_MASK                    0x00040000L
#define SQ_PROGRAM_CNTL__PARAM_GEN                         0x00040000L
#define SQ_PROGRAM_CNTL__GEN_INDEX_PIX_MASK                0x00080000L
#define SQ_PROGRAM_CNTL__GEN_INDEX_PIX                     0x00080000L
#define SQ_PROGRAM_CNTL__VS_EXPORT_COUNT_MASK              0x00f00000L
#define SQ_PROGRAM_CNTL__VS_EXPORT_MODE_MASK               0x07000000L
#define SQ_PROGRAM_CNTL__PS_EXPORT_MODE_MASK               0x78000000L
#define SQ_PROGRAM_CNTL__GEN_INDEX_VTX_MASK                0x80000000L
#define SQ_PROGRAM_CNTL__GEN_INDEX_VTX                     0x80000000L

// SQ_WRAPPING_0
#define SQ_WRAPPING_0__PARAM_WRAP_0_MASK                   0x0000000fL
#define SQ_WRAPPING_0__PARAM_WRAP_1_MASK                   0x000000f0L
#define SQ_WRAPPING_0__PARAM_WRAP_2_MASK                   0x00000f00L
#define SQ_WRAPPING_0__PARAM_WRAP_3_MASK                   0x0000f000L
#define SQ_WRAPPING_0__PARAM_WRAP_4_MASK                   0x000f0000L
#define SQ_WRAPPING_0__PARAM_WRAP_5_MASK                   0x00f00000L
#define SQ_WRAPPING_0__PARAM_WRAP_6_MASK                   0x0f000000L
#define SQ_WRAPPING_0__PARAM_WRAP_7_MASK                   0xf0000000L

// SQ_WRAPPING_1
#define SQ_WRAPPING_1__PARAM_WRAP_8_MASK                   0x0000000fL
#define SQ_WRAPPING_1__PARAM_WRAP_9_MASK                   0x000000f0L
#define SQ_WRAPPING_1__PARAM_WRAP_10_MASK                  0x00000f00L
#define SQ_WRAPPING_1__PARAM_WRAP_11_MASK                  0x0000f000L
#define SQ_WRAPPING_1__PARAM_WRAP_12_MASK                  0x000f0000L
#define SQ_WRAPPING_1__PARAM_WRAP_13_MASK                  0x00f00000L
#define SQ_WRAPPING_1__PARAM_WRAP_14_MASK                  0x0f000000L
#define SQ_WRAPPING_1__PARAM_WRAP_15_MASK                  0xf0000000L

// SQ_VS_CONST
#define SQ_VS_CONST__BASE_MASK                             0x000001ffL
#define SQ_VS_CONST__SIZE_MASK                             0x001ff000L

// SQ_PS_CONST
#define SQ_PS_CONST__BASE_MASK                             0x000001ffL
#define SQ_PS_CONST__SIZE_MASK                             0x001ff000L

// SQ_CONTEXT_MISC
#define SQ_CONTEXT_MISC__INST_PRED_OPTIMIZE_MASK           0x00000001L
#define SQ_CONTEXT_MISC__INST_PRED_OPTIMIZE                0x00000001L
#define SQ_CONTEXT_MISC__SC_OUTPUT_SCREEN_XY_MASK          0x00000002L
#define SQ_CONTEXT_MISC__SC_OUTPUT_SCREEN_XY               0x00000002L
#define SQ_CONTEXT_MISC__SC_SAMPLE_CNTL_MASK               0x0000000cL
#define SQ_CONTEXT_MISC__PARAM_GEN_POS_MASK                0x0000ff00L
#define SQ_CONTEXT_MISC__PERFCOUNTER_REF_MASK              0x00010000L
#define SQ_CONTEXT_MISC__PERFCOUNTER_REF                   0x00010000L
#define SQ_CONTEXT_MISC__YEILD_OPTIMIZE_MASK               0x00020000L
#define SQ_CONTEXT_MISC__YEILD_OPTIMIZE                    0x00020000L
#define SQ_CONTEXT_MISC__TX_CACHE_SEL_MASK                 0x00040000L
#define SQ_CONTEXT_MISC__TX_CACHE_SEL                      0x00040000L

// SQ_CF_RD_BASE
#define SQ_CF_RD_BASE__RD_BASE_MASK                        0x00000007L

// SQ_DEBUG_MISC_0
#define SQ_DEBUG_MISC_0__DB_PROB_ON_MASK                   0x00000001L
#define SQ_DEBUG_MISC_0__DB_PROB_ON                        0x00000001L
#define SQ_DEBUG_MISC_0__DB_PROB_BREAK_MASK                0x00000010L
#define SQ_DEBUG_MISC_0__DB_PROB_BREAK                     0x00000010L
#define SQ_DEBUG_MISC_0__DB_PROB_ADDR_MASK                 0x0007ff00L
#define SQ_DEBUG_MISC_0__DB_PROB_COUNT_MASK                0xff000000L

// SQ_DEBUG_MISC_1
#define SQ_DEBUG_MISC_1__DB_ON_PIX_MASK                    0x00000001L
#define SQ_DEBUG_MISC_1__DB_ON_PIX                         0x00000001L
#define SQ_DEBUG_MISC_1__DB_ON_VTX_MASK                    0x00000002L
#define SQ_DEBUG_MISC_1__DB_ON_VTX                         0x00000002L
#define SQ_DEBUG_MISC_1__DB_INST_COUNT_MASK                0x0000ff00L
#define SQ_DEBUG_MISC_1__DB_BREAK_ADDR_MASK                0x07ff0000L

// MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG__SAME_PAGE_LIMIT_MASK            0x0000003fL
#define MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY_MASK      0x00000040L
#define MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY           0x00000040L
#define MH_ARBITER_CONFIG__L1_ARB_ENABLE_MASK              0x00000080L
#define MH_ARBITER_CONFIG__L1_ARB_ENABLE                   0x00000080L
#define MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE_MASK         0x00000100L
#define MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE              0x00000100L
#define MH_ARBITER_CONFIG__L2_ARB_CONTROL_MASK             0x00000200L
#define MH_ARBITER_CONFIG__L2_ARB_CONTROL                  0x00000200L
#define MH_ARBITER_CONFIG__PAGE_SIZE_MASK                  0x00001c00L
#define MH_ARBITER_CONFIG__TC_REORDER_ENABLE_MASK          0x00002000L
#define MH_ARBITER_CONFIG__TC_REORDER_ENABLE               0x00002000L
#define MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE_MASK         0x00004000L
#define MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE              0x00004000L
#define MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE_MASK     0x00008000L
#define MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE          0x00008000L
#define MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_MASK            0x003f0000L
#define MH_ARBITER_CONFIG__CP_CLNT_ENABLE_MASK             0x00400000L
#define MH_ARBITER_CONFIG__CP_CLNT_ENABLE                  0x00400000L
#define MH_ARBITER_CONFIG__VGT_CLNT_ENABLE_MASK            0x00800000L
#define MH_ARBITER_CONFIG__VGT_CLNT_ENABLE                 0x00800000L
#define MH_ARBITER_CONFIG__TC_CLNT_ENABLE_MASK             0x01000000L
#define MH_ARBITER_CONFIG__TC_CLNT_ENABLE                  0x01000000L
#define MH_ARBITER_CONFIG__RB_CLNT_ENABLE_MASK             0x02000000L
#define MH_ARBITER_CONFIG__RB_CLNT_ENABLE                  0x02000000L

// MH_CLNT_AXI_ID_REUSE
#define MH_CLNT_AXI_ID_REUSE__CPw_ID_MASK                  0x00000007L
#define MH_CLNT_AXI_ID_REUSE__RESERVED1_MASK               0x00000008L
#define MH_CLNT_AXI_ID_REUSE__RESERVED1                    0x00000008L
#define MH_CLNT_AXI_ID_REUSE__RBw_ID_MASK                  0x00000070L
#define MH_CLNT_AXI_ID_REUSE__RESERVED2_MASK               0x00000080L
#define MH_CLNT_AXI_ID_REUSE__RESERVED2                    0x00000080L
#define MH_CLNT_AXI_ID_REUSE__MMUr_ID_MASK                 0x00000700L

// MH_INTERRUPT_MASK
#define MH_INTERRUPT_MASK__AXI_READ_ERROR_MASK             0x00000001L
#define MH_INTERRUPT_MASK__AXI_READ_ERROR                  0x00000001L
#define MH_INTERRUPT_MASK__AXI_WRITE_ERROR_MASK            0x00000002L
#define MH_INTERRUPT_MASK__AXI_WRITE_ERROR                 0x00000002L
#define MH_INTERRUPT_MASK__MMU_PAGE_FAULT_MASK             0x00000004L
#define MH_INTERRUPT_MASK__MMU_PAGE_FAULT                  0x00000004L

// MH_INTERRUPT_STATUS
#define MH_INTERRUPT_STATUS__AXI_READ_ERROR_MASK           0x00000001L
#define MH_INTERRUPT_STATUS__AXI_READ_ERROR                0x00000001L
#define MH_INTERRUPT_STATUS__AXI_WRITE_ERROR_MASK          0x00000002L
#define MH_INTERRUPT_STATUS__AXI_WRITE_ERROR               0x00000002L
#define MH_INTERRUPT_STATUS__MMU_PAGE_FAULT_MASK           0x00000004L
#define MH_INTERRUPT_STATUS__MMU_PAGE_FAULT                0x00000004L

// MH_INTERRUPT_CLEAR
#define MH_INTERRUPT_CLEAR__AXI_READ_ERROR_MASK            0x00000001L
#define MH_INTERRUPT_CLEAR__AXI_READ_ERROR                 0x00000001L
#define MH_INTERRUPT_CLEAR__AXI_WRITE_ERROR_MASK           0x00000002L
#define MH_INTERRUPT_CLEAR__AXI_WRITE_ERROR                0x00000002L
#define MH_INTERRUPT_CLEAR__MMU_PAGE_FAULT_MASK            0x00000004L
#define MH_INTERRUPT_CLEAR__MMU_PAGE_FAULT                 0x00000004L

// MH_AXI_ERROR
#define MH_AXI_ERROR__AXI_READ_ID_MASK                     0x00000007L
#define MH_AXI_ERROR__AXI_READ_ERROR_MASK                  0x00000008L
#define MH_AXI_ERROR__AXI_READ_ERROR                       0x00000008L
#define MH_AXI_ERROR__AXI_WRITE_ID_MASK                    0x00000070L
#define MH_AXI_ERROR__AXI_WRITE_ERROR_MASK                 0x00000080L
#define MH_AXI_ERROR__AXI_WRITE_ERROR                      0x00000080L

// MH_PERFCOUNTER0_SELECT
#define MH_PERFCOUNTER0_SELECT__PERF_SEL_MASK              0x000000ffL

// MH_PERFCOUNTER1_SELECT
#define MH_PERFCOUNTER1_SELECT__PERF_SEL_MASK              0x000000ffL

// MH_PERFCOUNTER0_CONFIG
#define MH_PERFCOUNTER0_CONFIG__N_VALUE_MASK               0x000000ffL

// MH_PERFCOUNTER1_CONFIG
#define MH_PERFCOUNTER1_CONFIG__N_VALUE_MASK               0x000000ffL

// MH_PERFCOUNTER0_LOW
#define MH_PERFCOUNTER0_LOW__PERF_COUNTER_LOW_MASK         0xffffffffL

// MH_PERFCOUNTER1_LOW
#define MH_PERFCOUNTER1_LOW__PERF_COUNTER_LOW_MASK         0xffffffffL

// MH_PERFCOUNTER0_HI
#define MH_PERFCOUNTER0_HI__PERF_COUNTER_HI_MASK           0x0000ffffL

// MH_PERFCOUNTER1_HI
#define MH_PERFCOUNTER1_HI__PERF_COUNTER_HI_MASK           0x0000ffffL

// MH_DEBUG_CTRL
#define MH_DEBUG_CTRL__INDEX_MASK                          0x0000003fL

// MH_DEBUG_DATA
#define MH_DEBUG_DATA__DATA_MASK                           0xffffffffL

// MH_DEBUG_REG00
#define MH_DEBUG_REG00__MH_BUSY_MASK                       0x00000001L
#define MH_DEBUG_REG00__MH_BUSY                            0x00000001L
#define MH_DEBUG_REG00__TRANS_OUTSTANDING_MASK             0x00000002L
#define MH_DEBUG_REG00__TRANS_OUTSTANDING                  0x00000002L
#define MH_DEBUG_REG00__CP_REQUEST_MASK                    0x00000004L
#define MH_DEBUG_REG00__CP_REQUEST                         0x00000004L
#define MH_DEBUG_REG00__VGT_REQUEST_MASK                   0x00000008L
#define MH_DEBUG_REG00__VGT_REQUEST                        0x00000008L
#define MH_DEBUG_REG00__TC_REQUEST_MASK                    0x00000010L
#define MH_DEBUG_REG00__TC_REQUEST                         0x00000010L
#define MH_DEBUG_REG00__TC_CAM_EMPTY_MASK                  0x00000020L
#define MH_DEBUG_REG00__TC_CAM_EMPTY                       0x00000020L
#define MH_DEBUG_REG00__TC_CAM_FULL_MASK                   0x00000040L
#define MH_DEBUG_REG00__TC_CAM_FULL                        0x00000040L
#define MH_DEBUG_REG00__TCD_EMPTY_MASK                     0x00000080L
#define MH_DEBUG_REG00__TCD_EMPTY                          0x00000080L
#define MH_DEBUG_REG00__TCD_FULL_MASK                      0x00000100L
#define MH_DEBUG_REG00__TCD_FULL                           0x00000100L
#define MH_DEBUG_REG00__RB_REQUEST_MASK                    0x00000200L
#define MH_DEBUG_REG00__RB_REQUEST                         0x00000200L
#define MH_DEBUG_REG00__MH_CLK_EN_STATE_MASK               0x00000400L
#define MH_DEBUG_REG00__MH_CLK_EN_STATE                    0x00000400L
#define MH_DEBUG_REG00__ARQ_EMPTY_MASK                     0x00000800L
#define MH_DEBUG_REG00__ARQ_EMPTY                          0x00000800L
#define MH_DEBUG_REG00__ARQ_FULL_MASK                      0x00001000L
#define MH_DEBUG_REG00__ARQ_FULL                           0x00001000L
#define MH_DEBUG_REG00__WDB_EMPTY_MASK                     0x00002000L
#define MH_DEBUG_REG00__WDB_EMPTY                          0x00002000L
#define MH_DEBUG_REG00__WDB_FULL_MASK                      0x00004000L
#define MH_DEBUG_REG00__WDB_FULL                           0x00004000L
#define MH_DEBUG_REG00__AXI_AVALID_MASK                    0x00008000L
#define MH_DEBUG_REG00__AXI_AVALID                         0x00008000L
#define MH_DEBUG_REG00__AXI_AREADY_MASK                    0x00010000L
#define MH_DEBUG_REG00__AXI_AREADY                         0x00010000L
#define MH_DEBUG_REG00__AXI_ARVALID_MASK                   0x00020000L
#define MH_DEBUG_REG00__AXI_ARVALID                        0x00020000L
#define MH_DEBUG_REG00__AXI_ARREADY_MASK                   0x00040000L
#define MH_DEBUG_REG00__AXI_ARREADY                        0x00040000L
#define MH_DEBUG_REG00__AXI_WVALID_MASK                    0x00080000L
#define MH_DEBUG_REG00__AXI_WVALID                         0x00080000L
#define MH_DEBUG_REG00__AXI_WREADY_MASK                    0x00100000L
#define MH_DEBUG_REG00__AXI_WREADY                         0x00100000L
#define MH_DEBUG_REG00__AXI_RVALID_MASK                    0x00200000L
#define MH_DEBUG_REG00__AXI_RVALID                         0x00200000L
#define MH_DEBUG_REG00__AXI_RREADY_MASK                    0x00400000L
#define MH_DEBUG_REG00__AXI_RREADY                         0x00400000L
#define MH_DEBUG_REG00__AXI_BVALID_MASK                    0x00800000L
#define MH_DEBUG_REG00__AXI_BVALID                         0x00800000L
#define MH_DEBUG_REG00__AXI_BREADY_MASK                    0x01000000L
#define MH_DEBUG_REG00__AXI_BREADY                         0x01000000L
#define MH_DEBUG_REG00__AXI_HALT_REQ_MASK                  0x02000000L
#define MH_DEBUG_REG00__AXI_HALT_REQ                       0x02000000L
#define MH_DEBUG_REG00__AXI_HALT_ACK_MASK                  0x04000000L
#define MH_DEBUG_REG00__AXI_HALT_ACK                       0x04000000L

// MH_DEBUG_REG01
#define MH_DEBUG_REG01__CP_SEND_q_MASK                     0x00000001L
#define MH_DEBUG_REG01__CP_SEND_q                          0x00000001L
#define MH_DEBUG_REG01__CP_RTR_q_MASK                      0x00000002L
#define MH_DEBUG_REG01__CP_RTR_q                           0x00000002L
#define MH_DEBUG_REG01__CP_WRITE_q_MASK                    0x00000004L
#define MH_DEBUG_REG01__CP_WRITE_q                         0x00000004L
#define MH_DEBUG_REG01__CP_TAG_q_MASK                      0x00000038L
#define MH_DEBUG_REG01__CP_BE_q_MASK                       0x00003fc0L
#define MH_DEBUG_REG01__VGT_SEND_q_MASK                    0x00004000L
#define MH_DEBUG_REG01__VGT_SEND_q                         0x00004000L
#define MH_DEBUG_REG01__VGT_RTR_q_MASK                     0x00008000L
#define MH_DEBUG_REG01__VGT_RTR_q                          0x00008000L
#define MH_DEBUG_REG01__VGT_TAG_q_MASK                     0x00010000L
#define MH_DEBUG_REG01__VGT_TAG_q                          0x00010000L
#define MH_DEBUG_REG01__TC_SEND_q_MASK                     0x00020000L
#define MH_DEBUG_REG01__TC_SEND_q                          0x00020000L
#define MH_DEBUG_REG01__TC_RTR_q_MASK                      0x00040000L
#define MH_DEBUG_REG01__TC_RTR_q                           0x00040000L
#define MH_DEBUG_REG01__TC_ROQ_SEND_q_MASK                 0x00080000L
#define MH_DEBUG_REG01__TC_ROQ_SEND_q                      0x00080000L
#define MH_DEBUG_REG01__TC_ROQ_RTR_q_MASK                  0x00100000L
#define MH_DEBUG_REG01__TC_ROQ_RTR_q                       0x00100000L
#define MH_DEBUG_REG01__TC_MH_written_MASK                 0x00200000L
#define MH_DEBUG_REG01__TC_MH_written                      0x00200000L
#define MH_DEBUG_REG01__RB_SEND_q_MASK                     0x00400000L
#define MH_DEBUG_REG01__RB_SEND_q                          0x00400000L
#define MH_DEBUG_REG01__RB_RTR_q_MASK                      0x00800000L
#define MH_DEBUG_REG01__RB_RTR_q                           0x00800000L
#define MH_DEBUG_REG01__RB_BE_q_MASK                       0xff000000L

// MH_DEBUG_REG02
#define MH_DEBUG_REG02__MH_CP_grb_send_MASK                0x00000001L
#define MH_DEBUG_REG02__MH_CP_grb_send                     0x00000001L
#define MH_DEBUG_REG02__MH_VGT_grb_send_MASK               0x00000002L
#define MH_DEBUG_REG02__MH_VGT_grb_send                    0x00000002L
#define MH_DEBUG_REG02__MH_TC_mcsend_MASK                  0x00000004L
#define MH_DEBUG_REG02__MH_TC_mcsend                       0x00000004L
#define MH_DEBUG_REG02__MH_CLNT_rlast_MASK                 0x00000008L
#define MH_DEBUG_REG02__MH_CLNT_rlast                      0x00000008L
#define MH_DEBUG_REG02__MH_CLNT_tag_MASK                   0x00000070L
#define MH_DEBUG_REG02__RDC_RID_MASK                       0x00000380L
#define MH_DEBUG_REG02__RDC_RRESP_MASK                     0x00000c00L
#define MH_DEBUG_REG02__MH_CP_writeclean_MASK              0x00001000L
#define MH_DEBUG_REG02__MH_CP_writeclean                   0x00001000L
#define MH_DEBUG_REG02__MH_RB_writeclean_MASK              0x00002000L
#define MH_DEBUG_REG02__MH_RB_writeclean                   0x00002000L
#define MH_DEBUG_REG02__BRC_BID_MASK                       0x0001c000L
#define MH_DEBUG_REG02__BRC_BRESP_MASK                     0x00060000L

// MH_DEBUG_REG03
#define MH_DEBUG_REG03__MH_CLNT_data_31_0_MASK             0xffffffffL

// MH_DEBUG_REG04
#define MH_DEBUG_REG04__MH_CLNT_data_63_32_MASK            0xffffffffL

// MH_DEBUG_REG05
#define MH_DEBUG_REG05__CP_MH_send_MASK                    0x00000001L
#define MH_DEBUG_REG05__CP_MH_send                         0x00000001L
#define MH_DEBUG_REG05__CP_MH_write_MASK                   0x00000002L
#define MH_DEBUG_REG05__CP_MH_write                        0x00000002L
#define MH_DEBUG_REG05__CP_MH_tag_MASK                     0x0000001cL
#define MH_DEBUG_REG05__CP_MH_ad_31_5_MASK                 0xffffffe0L

// MH_DEBUG_REG06
#define MH_DEBUG_REG06__CP_MH_data_31_0_MASK               0xffffffffL

// MH_DEBUG_REG07
#define MH_DEBUG_REG07__CP_MH_data_63_32_MASK              0xffffffffL

// MH_DEBUG_REG08
#define MH_DEBUG_REG08__ALWAYS_ZERO_MASK                   0x00000007L
#define MH_DEBUG_REG08__VGT_MH_send_MASK                   0x00000008L
#define MH_DEBUG_REG08__VGT_MH_send                        0x00000008L
#define MH_DEBUG_REG08__VGT_MH_tagbe_MASK                  0x00000010L
#define MH_DEBUG_REG08__VGT_MH_tagbe                       0x00000010L
#define MH_DEBUG_REG08__VGT_MH_ad_31_5_MASK                0xffffffe0L

// MH_DEBUG_REG09
#define MH_DEBUG_REG09__ALWAYS_ZERO_MASK                   0x00000003L
#define MH_DEBUG_REG09__TC_MH_send_MASK                    0x00000004L
#define MH_DEBUG_REG09__TC_MH_send                         0x00000004L
#define MH_DEBUG_REG09__TC_MH_mask_MASK                    0x00000018L
#define MH_DEBUG_REG09__TC_MH_addr_31_5_MASK               0xffffffe0L

// MH_DEBUG_REG10
#define MH_DEBUG_REG10__TC_MH_info_MASK                    0x01ffffffL
#define MH_DEBUG_REG10__TC_MH_send_MASK                    0x02000000L
#define MH_DEBUG_REG10__TC_MH_send                         0x02000000L

// MH_DEBUG_REG11
#define MH_DEBUG_REG11__MH_TC_mcinfo_MASK                  0x01ffffffL
#define MH_DEBUG_REG11__MH_TC_mcinfo_send_MASK             0x02000000L
#define MH_DEBUG_REG11__MH_TC_mcinfo_send                  0x02000000L
#define MH_DEBUG_REG11__TC_MH_written_MASK                 0x04000000L
#define MH_DEBUG_REG11__TC_MH_written                      0x04000000L

// MH_DEBUG_REG12
#define MH_DEBUG_REG12__ALWAYS_ZERO_MASK                   0x00000003L
#define MH_DEBUG_REG12__TC_ROQ_SEND_MASK                   0x00000004L
#define MH_DEBUG_REG12__TC_ROQ_SEND                        0x00000004L
#define MH_DEBUG_REG12__TC_ROQ_MASK_MASK                   0x00000018L
#define MH_DEBUG_REG12__TC_ROQ_ADDR_31_5_MASK              0xffffffe0L

// MH_DEBUG_REG13
#define MH_DEBUG_REG13__TC_ROQ_INFO_MASK                   0x01ffffffL
#define MH_DEBUG_REG13__TC_ROQ_SEND_MASK                   0x02000000L
#define MH_DEBUG_REG13__TC_ROQ_SEND                        0x02000000L

// MH_DEBUG_REG14
#define MH_DEBUG_REG14__ALWAYS_ZERO_MASK                   0x0000000fL
#define MH_DEBUG_REG14__RB_MH_send_MASK                    0x00000010L
#define MH_DEBUG_REG14__RB_MH_send                         0x00000010L
#define MH_DEBUG_REG14__RB_MH_addr_31_5_MASK               0xffffffe0L

// MH_DEBUG_REG15
#define MH_DEBUG_REG15__RB_MH_data_31_0_MASK               0xffffffffL

// MH_DEBUG_REG16
#define MH_DEBUG_REG16__RB_MH_data_63_32_MASK              0xffffffffL

// MH_DEBUG_REG17
#define MH_DEBUG_REG17__AVALID_q_MASK                      0x00000001L
#define MH_DEBUG_REG17__AVALID_q                           0x00000001L
#define MH_DEBUG_REG17__AREADY_q_MASK                      0x00000002L
#define MH_DEBUG_REG17__AREADY_q                           0x00000002L
#define MH_DEBUG_REG17__AID_q_MASK                         0x0000001cL
#define MH_DEBUG_REG17__ALEN_q_2_0_MASK                    0x000000e0L
#define MH_DEBUG_REG17__ARVALID_q_MASK                     0x00000100L
#define MH_DEBUG_REG17__ARVALID_q                          0x00000100L
#define MH_DEBUG_REG17__ARREADY_q_MASK                     0x00000200L
#define MH_DEBUG_REG17__ARREADY_q                          0x00000200L
#define MH_DEBUG_REG17__ARID_q_MASK                        0x00001c00L
#define MH_DEBUG_REG17__ARLEN_q_1_0_MASK                   0x00006000L
#define MH_DEBUG_REG17__RVALID_q_MASK                      0x00008000L
#define MH_DEBUG_REG17__RVALID_q                           0x00008000L
#define MH_DEBUG_REG17__RREADY_q_MASK                      0x00010000L
#define MH_DEBUG_REG17__RREADY_q                           0x00010000L
#define MH_DEBUG_REG17__RLAST_q_MASK                       0x00020000L
#define MH_DEBUG_REG17__RLAST_q                            0x00020000L
#define MH_DEBUG_REG17__RID_q_MASK                         0x001c0000L
#define MH_DEBUG_REG17__WVALID_q_MASK                      0x00200000L
#define MH_DEBUG_REG17__WVALID_q                           0x00200000L
#define MH_DEBUG_REG17__WREADY_q_MASK                      0x00400000L
#define MH_DEBUG_REG17__WREADY_q                           0x00400000L
#define MH_DEBUG_REG17__WLAST_q_MASK                       0x00800000L
#define MH_DEBUG_REG17__WLAST_q                            0x00800000L
#define MH_DEBUG_REG17__WID_q_MASK                         0x07000000L
#define MH_DEBUG_REG17__BVALID_q_MASK                      0x08000000L
#define MH_DEBUG_REG17__BVALID_q                           0x08000000L
#define MH_DEBUG_REG17__BREADY_q_MASK                      0x10000000L
#define MH_DEBUG_REG17__BREADY_q                           0x10000000L
#define MH_DEBUG_REG17__BID_q_MASK                         0xe0000000L

// MH_DEBUG_REG18
#define MH_DEBUG_REG18__AVALID_q_MASK                      0x00000001L
#define MH_DEBUG_REG18__AVALID_q                           0x00000001L
#define MH_DEBUG_REG18__AREADY_q_MASK                      0x00000002L
#define MH_DEBUG_REG18__AREADY_q                           0x00000002L
#define MH_DEBUG_REG18__AID_q_MASK                         0x0000001cL
#define MH_DEBUG_REG18__ALEN_q_1_0_MASK                    0x00000060L
#define MH_DEBUG_REG18__ARVALID_q_MASK                     0x00000080L
#define MH_DEBUG_REG18__ARVALID_q                          0x00000080L
#define MH_DEBUG_REG18__ARREADY_q_MASK                     0x00000100L
#define MH_DEBUG_REG18__ARREADY_q                          0x00000100L
#define MH_DEBUG_REG18__ARID_q_MASK                        0x00000e00L
#define MH_DEBUG_REG18__ARLEN_q_1_1_MASK                   0x00001000L
#define MH_DEBUG_REG18__ARLEN_q_1_1                        0x00001000L
#define MH_DEBUG_REG18__WVALID_q_MASK                      0x00002000L
#define MH_DEBUG_REG18__WVALID_q                           0x00002000L
#define MH_DEBUG_REG18__WREADY_q_MASK                      0x00004000L
#define MH_DEBUG_REG18__WREADY_q                           0x00004000L
#define MH_DEBUG_REG18__WLAST_q_MASK                       0x00008000L
#define MH_DEBUG_REG18__WLAST_q                            0x00008000L
#define MH_DEBUG_REG18__WID_q_MASK                         0x00070000L
#define MH_DEBUG_REG18__WSTRB_q_MASK                       0x07f80000L
#define MH_DEBUG_REG18__BVALID_q_MASK                      0x08000000L
#define MH_DEBUG_REG18__BVALID_q                           0x08000000L
#define MH_DEBUG_REG18__BREADY_q_MASK                      0x10000000L
#define MH_DEBUG_REG18__BREADY_q                           0x10000000L
#define MH_DEBUG_REG18__BID_q_MASK                         0xe0000000L

// MH_DEBUG_REG19
#define MH_DEBUG_REG19__ARC_CTRL_RE_q_MASK                 0x00000001L
#define MH_DEBUG_REG19__ARC_CTRL_RE_q                      0x00000001L
#define MH_DEBUG_REG19__CTRL_ARC_ID_MASK                   0x0000000eL
#define MH_DEBUG_REG19__CTRL_ARC_PAD_MASK                  0xfffffff0L

// MH_DEBUG_REG20
#define MH_DEBUG_REG20__ALWAYS_ZERO_MASK                   0x00000003L
#define MH_DEBUG_REG20__REG_A_MASK                         0x0000fffcL
#define MH_DEBUG_REG20__REG_RE_MASK                        0x00010000L
#define MH_DEBUG_REG20__REG_RE                             0x00010000L
#define MH_DEBUG_REG20__REG_WE_MASK                        0x00020000L
#define MH_DEBUG_REG20__REG_WE                             0x00020000L
#define MH_DEBUG_REG20__BLOCK_RS_MASK                      0x00040000L
#define MH_DEBUG_REG20__BLOCK_RS                           0x00040000L

// MH_DEBUG_REG21
#define MH_DEBUG_REG21__REG_WD_MASK                        0xffffffffL

// MH_DEBUG_REG22
#define MH_DEBUG_REG22__CIB_MH_axi_halt_req_MASK           0x00000001L
#define MH_DEBUG_REG22__CIB_MH_axi_halt_req                0x00000001L
#define MH_DEBUG_REG22__MH_CIB_axi_halt_ack_MASK           0x00000002L
#define MH_DEBUG_REG22__MH_CIB_axi_halt_ack                0x00000002L
#define MH_DEBUG_REG22__MH_RBBM_busy_MASK                  0x00000004L
#define MH_DEBUG_REG22__MH_RBBM_busy                       0x00000004L
#define MH_DEBUG_REG22__MH_CIB_mh_clk_en_int_MASK          0x00000008L
#define MH_DEBUG_REG22__MH_CIB_mh_clk_en_int               0x00000008L
#define MH_DEBUG_REG22__MH_CIB_mmu_clk_en_int_MASK         0x00000010L
#define MH_DEBUG_REG22__MH_CIB_mmu_clk_en_int              0x00000010L
#define MH_DEBUG_REG22__MH_CIB_tcroq_clk_en_int_MASK       0x00000020L
#define MH_DEBUG_REG22__MH_CIB_tcroq_clk_en_int            0x00000020L
#define MH_DEBUG_REG22__GAT_CLK_ENA_MASK                   0x00000040L
#define MH_DEBUG_REG22__GAT_CLK_ENA                        0x00000040L
#define MH_DEBUG_REG22__AXI_RDY_ENA_MASK                   0x00000080L
#define MH_DEBUG_REG22__AXI_RDY_ENA                        0x00000080L
#define MH_DEBUG_REG22__RBBM_MH_clk_en_override_MASK       0x00000100L
#define MH_DEBUG_REG22__RBBM_MH_clk_en_override            0x00000100L
#define MH_DEBUG_REG22__CNT_q_MASK                         0x00007e00L
#define MH_DEBUG_REG22__TCD_EMPTY_q_MASK                   0x00008000L
#define MH_DEBUG_REG22__TCD_EMPTY_q                        0x00008000L
#define MH_DEBUG_REG22__TC_ROQ_EMPTY_MASK                  0x00010000L
#define MH_DEBUG_REG22__TC_ROQ_EMPTY                       0x00010000L
#define MH_DEBUG_REG22__MH_BUSY_d_MASK                     0x00020000L
#define MH_DEBUG_REG22__MH_BUSY_d                          0x00020000L
#define MH_DEBUG_REG22__ANY_CLNT_BUSY_MASK                 0x00040000L
#define MH_DEBUG_REG22__ANY_CLNT_BUSY                      0x00040000L
#define MH_DEBUG_REG22__MH_MMU_INVALIDATE_INVALIDATE_ALL_MASK 0x00080000L
#define MH_DEBUG_REG22__MH_MMU_INVALIDATE_INVALIDATE_ALL   0x00080000L
#define MH_DEBUG_REG22__CP_SEND_q_MASK                     0x00100000L
#define MH_DEBUG_REG22__CP_SEND_q                          0x00100000L
#define MH_DEBUG_REG22__CP_RTR_q_MASK                      0x00200000L
#define MH_DEBUG_REG22__CP_RTR_q                           0x00200000L
#define MH_DEBUG_REG22__VGT_SEND_q_MASK                    0x00400000L
#define MH_DEBUG_REG22__VGT_SEND_q                         0x00400000L
#define MH_DEBUG_REG22__VGT_RTR_q_MASK                     0x00800000L
#define MH_DEBUG_REG22__VGT_RTR_q                          0x00800000L
#define MH_DEBUG_REG22__TC_ROQ_SEND_q_MASK                 0x01000000L
#define MH_DEBUG_REG22__TC_ROQ_SEND_q                      0x01000000L
#define MH_DEBUG_REG22__TC_ROQ_RTR_q_MASK                  0x02000000L
#define MH_DEBUG_REG22__TC_ROQ_RTR_q                       0x02000000L
#define MH_DEBUG_REG22__RB_SEND_q_MASK                     0x04000000L
#define MH_DEBUG_REG22__RB_SEND_q                          0x04000000L
#define MH_DEBUG_REG22__RB_RTR_q_MASK                      0x08000000L
#define MH_DEBUG_REG22__RB_RTR_q                           0x08000000L
#define MH_DEBUG_REG22__RDC_VALID_MASK                     0x10000000L
#define MH_DEBUG_REG22__RDC_VALID                          0x10000000L
#define MH_DEBUG_REG22__RDC_RLAST_MASK                     0x20000000L
#define MH_DEBUG_REG22__RDC_RLAST                          0x20000000L
#define MH_DEBUG_REG22__TLBMISS_VALID_MASK                 0x40000000L
#define MH_DEBUG_REG22__TLBMISS_VALID                      0x40000000L
#define MH_DEBUG_REG22__BRC_VALID_MASK                     0x80000000L
#define MH_DEBUG_REG22__BRC_VALID                          0x80000000L

// MH_DEBUG_REG23
#define MH_DEBUG_REG23__EFF2_FP_WINNER_MASK                0x00000007L
#define MH_DEBUG_REG23__EFF2_LRU_WINNER_out_MASK           0x00000038L
#define MH_DEBUG_REG23__EFF1_WINNER_MASK                   0x000001c0L
#define MH_DEBUG_REG23__ARB_WINNER_MASK                    0x00000e00L
#define MH_DEBUG_REG23__ARB_WINNER_q_MASK                  0x00007000L
#define MH_DEBUG_REG23__EFF1_WIN_MASK                      0x00008000L
#define MH_DEBUG_REG23__EFF1_WIN                           0x00008000L
#define MH_DEBUG_REG23__KILL_EFF1_MASK                     0x00010000L
#define MH_DEBUG_REG23__KILL_EFF1                          0x00010000L
#define MH_DEBUG_REG23__ARB_HOLD_MASK                      0x00020000L
#define MH_DEBUG_REG23__ARB_HOLD                           0x00020000L
#define MH_DEBUG_REG23__ARB_RTR_q_MASK                     0x00040000L
#define MH_DEBUG_REG23__ARB_RTR_q                          0x00040000L
#define MH_DEBUG_REG23__CP_SEND_QUAL_MASK                  0x00080000L
#define MH_DEBUG_REG23__CP_SEND_QUAL                       0x00080000L
#define MH_DEBUG_REG23__VGT_SEND_QUAL_MASK                 0x00100000L
#define MH_DEBUG_REG23__VGT_SEND_QUAL                      0x00100000L
#define MH_DEBUG_REG23__TC_SEND_QUAL_MASK                  0x00200000L
#define MH_DEBUG_REG23__TC_SEND_QUAL                       0x00200000L
#define MH_DEBUG_REG23__TC_SEND_EFF1_QUAL_MASK             0x00400000L
#define MH_DEBUG_REG23__TC_SEND_EFF1_QUAL                  0x00400000L
#define MH_DEBUG_REG23__RB_SEND_QUAL_MASK                  0x00800000L
#define MH_DEBUG_REG23__RB_SEND_QUAL                       0x00800000L
#define MH_DEBUG_REG23__ARB_QUAL_MASK                      0x01000000L
#define MH_DEBUG_REG23__ARB_QUAL                           0x01000000L
#define MH_DEBUG_REG23__CP_EFF1_REQ_MASK                   0x02000000L
#define MH_DEBUG_REG23__CP_EFF1_REQ                        0x02000000L
#define MH_DEBUG_REG23__VGT_EFF1_REQ_MASK                  0x04000000L
#define MH_DEBUG_REG23__VGT_EFF1_REQ                       0x04000000L
#define MH_DEBUG_REG23__TC_EFF1_REQ_MASK                   0x08000000L
#define MH_DEBUG_REG23__TC_EFF1_REQ                        0x08000000L
#define MH_DEBUG_REG23__RB_EFF1_REQ_MASK                   0x10000000L
#define MH_DEBUG_REG23__RB_EFF1_REQ                        0x10000000L
#define MH_DEBUG_REG23__ANY_SAME_ROW_BANK_MASK             0x20000000L
#define MH_DEBUG_REG23__ANY_SAME_ROW_BANK                  0x20000000L
#define MH_DEBUG_REG23__TCD_NEARFULL_q_MASK                0x40000000L
#define MH_DEBUG_REG23__TCD_NEARFULL_q                     0x40000000L
#define MH_DEBUG_REG23__TCHOLD_IP_q_MASK                   0x80000000L
#define MH_DEBUG_REG23__TCHOLD_IP_q                        0x80000000L

// MH_DEBUG_REG24
#define MH_DEBUG_REG24__EFF1_WINNER_MASK                   0x00000007L
#define MH_DEBUG_REG24__ARB_WINNER_MASK                    0x00000038L
#define MH_DEBUG_REG24__CP_SEND_QUAL_MASK                  0x00000040L
#define MH_DEBUG_REG24__CP_SEND_QUAL                       0x00000040L
#define MH_DEBUG_REG24__VGT_SEND_QUAL_MASK                 0x00000080L
#define MH_DEBUG_REG24__VGT_SEND_QUAL                      0x00000080L
#define MH_DEBUG_REG24__TC_SEND_QUAL_MASK                  0x00000100L
#define MH_DEBUG_REG24__TC_SEND_QUAL                       0x00000100L
#define MH_DEBUG_REG24__TC_SEND_EFF1_QUAL_MASK             0x00000200L
#define MH_DEBUG_REG24__TC_SEND_EFF1_QUAL                  0x00000200L
#define MH_DEBUG_REG24__RB_SEND_QUAL_MASK                  0x00000400L
#define MH_DEBUG_REG24__RB_SEND_QUAL                       0x00000400L
#define MH_DEBUG_REG24__ARB_QUAL_MASK                      0x00000800L
#define MH_DEBUG_REG24__ARB_QUAL                           0x00000800L
#define MH_DEBUG_REG24__CP_EFF1_REQ_MASK                   0x00001000L
#define MH_DEBUG_REG24__CP_EFF1_REQ                        0x00001000L
#define MH_DEBUG_REG24__VGT_EFF1_REQ_MASK                  0x00002000L
#define MH_DEBUG_REG24__VGT_EFF1_REQ                       0x00002000L
#define MH_DEBUG_REG24__TC_EFF1_REQ_MASK                   0x00004000L
#define MH_DEBUG_REG24__TC_EFF1_REQ                        0x00004000L
#define MH_DEBUG_REG24__RB_EFF1_REQ_MASK                   0x00008000L
#define MH_DEBUG_REG24__RB_EFF1_REQ                        0x00008000L
#define MH_DEBUG_REG24__EFF1_WIN_MASK                      0x00010000L
#define MH_DEBUG_REG24__EFF1_WIN                           0x00010000L
#define MH_DEBUG_REG24__KILL_EFF1_MASK                     0x00020000L
#define MH_DEBUG_REG24__KILL_EFF1                          0x00020000L
#define MH_DEBUG_REG24__TCD_NEARFULL_q_MASK                0x00040000L
#define MH_DEBUG_REG24__TCD_NEARFULL_q                     0x00040000L
#define MH_DEBUG_REG24__TC_ARB_HOLD_MASK                   0x00080000L
#define MH_DEBUG_REG24__TC_ARB_HOLD                        0x00080000L
#define MH_DEBUG_REG24__ARB_HOLD_MASK                      0x00100000L
#define MH_DEBUG_REG24__ARB_HOLD                           0x00100000L
#define MH_DEBUG_REG24__ARB_RTR_q_MASK                     0x00200000L
#define MH_DEBUG_REG24__ARB_RTR_q                          0x00200000L
#define MH_DEBUG_REG24__SAME_PAGE_LIMIT_COUNT_q_MASK       0xffc00000L

// MH_DEBUG_REG25
#define MH_DEBUG_REG25__EFF2_LRU_WINNER_out_MASK           0x00000007L
#define MH_DEBUG_REG25__ARB_WINNER_MASK                    0x00000038L
#define MH_DEBUG_REG25__LEAST_RECENT_INDEX_d_MASK          0x000001c0L
#define MH_DEBUG_REG25__LEAST_RECENT_d_MASK                0x00000e00L
#define MH_DEBUG_REG25__UPDATE_RECENT_STACK_d_MASK         0x00001000L
#define MH_DEBUG_REG25__UPDATE_RECENT_STACK_d              0x00001000L
#define MH_DEBUG_REG25__ARB_HOLD_MASK                      0x00002000L
#define MH_DEBUG_REG25__ARB_HOLD                           0x00002000L
#define MH_DEBUG_REG25__ARB_RTR_q_MASK                     0x00004000L
#define MH_DEBUG_REG25__ARB_RTR_q                          0x00004000L
#define MH_DEBUG_REG25__EFF1_WIN_MASK                      0x00008000L
#define MH_DEBUG_REG25__EFF1_WIN                           0x00008000L
#define MH_DEBUG_REG25__CLNT_REQ_MASK                      0x000f0000L
#define MH_DEBUG_REG25__RECENT_d_0_MASK                    0x00700000L
#define MH_DEBUG_REG25__RECENT_d_1_MASK                    0x03800000L
#define MH_DEBUG_REG25__RECENT_d_2_MASK                    0x1c000000L
#define MH_DEBUG_REG25__RECENT_d_3_MASK                    0xe0000000L

// MH_DEBUG_REG26
#define MH_DEBUG_REG26__TC_ARB_HOLD_MASK                   0x00000001L
#define MH_DEBUG_REG26__TC_ARB_HOLD                        0x00000001L
#define MH_DEBUG_REG26__TC_NOROQ_SAME_ROW_BANK_MASK        0x00000002L
#define MH_DEBUG_REG26__TC_NOROQ_SAME_ROW_BANK             0x00000002L
#define MH_DEBUG_REG26__TC_ROQ_SAME_ROW_BANK_MASK          0x00000004L
#define MH_DEBUG_REG26__TC_ROQ_SAME_ROW_BANK               0x00000004L
#define MH_DEBUG_REG26__TCD_NEARFULL_q_MASK                0x00000008L
#define MH_DEBUG_REG26__TCD_NEARFULL_q                     0x00000008L
#define MH_DEBUG_REG26__TCHOLD_IP_q_MASK                   0x00000010L
#define MH_DEBUG_REG26__TCHOLD_IP_q                        0x00000010L
#define MH_DEBUG_REG26__TCHOLD_CNT_q_MASK                  0x000000e0L
#define MH_DEBUG_REG26__MH_ARBITER_CONFIG_TC_REORDER_ENABLE_MASK 0x00000100L
#define MH_DEBUG_REG26__MH_ARBITER_CONFIG_TC_REORDER_ENABLE 0x00000100L
#define MH_DEBUG_REG26__TC_ROQ_RTR_DBG_q_MASK              0x00000200L
#define MH_DEBUG_REG26__TC_ROQ_RTR_DBG_q                   0x00000200L
#define MH_DEBUG_REG26__TC_ROQ_SEND_q_MASK                 0x00000400L
#define MH_DEBUG_REG26__TC_ROQ_SEND_q                      0x00000400L
#define MH_DEBUG_REG26__TC_MH_written_MASK                 0x00000800L
#define MH_DEBUG_REG26__TC_MH_written                      0x00000800L
#define MH_DEBUG_REG26__TCD_FULLNESS_CNT_q_MASK            0x0007f000L
#define MH_DEBUG_REG26__WBURST_ACTIVE_MASK                 0x00080000L
#define MH_DEBUG_REG26__WBURST_ACTIVE                      0x00080000L
#define MH_DEBUG_REG26__WLAST_q_MASK                       0x00100000L
#define MH_DEBUG_REG26__WLAST_q                            0x00100000L
#define MH_DEBUG_REG26__WBURST_IP_q_MASK                   0x00200000L
#define MH_DEBUG_REG26__WBURST_IP_q                        0x00200000L
#define MH_DEBUG_REG26__WBURST_CNT_q_MASK                  0x01c00000L
#define MH_DEBUG_REG26__CP_SEND_QUAL_MASK                  0x02000000L
#define MH_DEBUG_REG26__CP_SEND_QUAL                       0x02000000L
#define MH_DEBUG_REG26__CP_MH_write_MASK                   0x04000000L
#define MH_DEBUG_REG26__CP_MH_write                        0x04000000L
#define MH_DEBUG_REG26__RB_SEND_QUAL_MASK                  0x08000000L
#define MH_DEBUG_REG26__RB_SEND_QUAL                       0x08000000L
#define MH_DEBUG_REG26__ARB_WINNER_MASK                    0x70000000L

// MH_DEBUG_REG27
#define MH_DEBUG_REG27__RF_ARBITER_CONFIG_q_MASK           0x03ffffffL
#define MH_DEBUG_REG27__MH_CLNT_AXI_ID_REUSE_MMUr_ID_MASK  0x1c000000L

// MH_DEBUG_REG28
#define MH_DEBUG_REG28__SAME_ROW_BANK_q_MASK               0x000000ffL
#define MH_DEBUG_REG28__ROQ_MARK_q_MASK                    0x0000ff00L
#define MH_DEBUG_REG28__ROQ_VALID_q_MASK                   0x00ff0000L
#define MH_DEBUG_REG28__TC_MH_send_MASK                    0x01000000L
#define MH_DEBUG_REG28__TC_MH_send                         0x01000000L
#define MH_DEBUG_REG28__TC_ROQ_RTR_q_MASK                  0x02000000L
#define MH_DEBUG_REG28__TC_ROQ_RTR_q                       0x02000000L
#define MH_DEBUG_REG28__KILL_EFF1_MASK                     0x04000000L
#define MH_DEBUG_REG28__KILL_EFF1                          0x04000000L
#define MH_DEBUG_REG28__TC_ROQ_SAME_ROW_BANK_SEL_MASK      0x08000000L
#define MH_DEBUG_REG28__TC_ROQ_SAME_ROW_BANK_SEL           0x08000000L
#define MH_DEBUG_REG28__ANY_SAME_ROW_BANK_MASK             0x10000000L
#define MH_DEBUG_REG28__ANY_SAME_ROW_BANK                  0x10000000L
#define MH_DEBUG_REG28__TC_EFF1_QUAL_MASK                  0x20000000L
#define MH_DEBUG_REG28__TC_EFF1_QUAL                       0x20000000L
#define MH_DEBUG_REG28__TC_ROQ_EMPTY_MASK                  0x40000000L
#define MH_DEBUG_REG28__TC_ROQ_EMPTY                       0x40000000L
#define MH_DEBUG_REG28__TC_ROQ_FULL_MASK                   0x80000000L
#define MH_DEBUG_REG28__TC_ROQ_FULL                        0x80000000L

// MH_DEBUG_REG29
#define MH_DEBUG_REG29__SAME_ROW_BANK_q_MASK               0x000000ffL
#define MH_DEBUG_REG29__ROQ_MARK_d_MASK                    0x0000ff00L
#define MH_DEBUG_REG29__ROQ_VALID_d_MASK                   0x00ff0000L
#define MH_DEBUG_REG29__TC_MH_send_MASK                    0x01000000L
#define MH_DEBUG_REG29__TC_MH_send                         0x01000000L
#define MH_DEBUG_REG29__TC_ROQ_RTR_q_MASK                  0x02000000L
#define MH_DEBUG_REG29__TC_ROQ_RTR_q                       0x02000000L
#define MH_DEBUG_REG29__KILL_EFF1_MASK                     0x04000000L
#define MH_DEBUG_REG29__KILL_EFF1                          0x04000000L
#define MH_DEBUG_REG29__TC_ROQ_SAME_ROW_BANK_SEL_MASK      0x08000000L
#define MH_DEBUG_REG29__TC_ROQ_SAME_ROW_BANK_SEL           0x08000000L
#define MH_DEBUG_REG29__ANY_SAME_ROW_BANK_MASK             0x10000000L
#define MH_DEBUG_REG29__ANY_SAME_ROW_BANK                  0x10000000L
#define MH_DEBUG_REG29__TC_EFF1_QUAL_MASK                  0x20000000L
#define MH_DEBUG_REG29__TC_EFF1_QUAL                       0x20000000L
#define MH_DEBUG_REG29__TC_ROQ_EMPTY_MASK                  0x40000000L
#define MH_DEBUG_REG29__TC_ROQ_EMPTY                       0x40000000L
#define MH_DEBUG_REG29__TC_ROQ_FULL_MASK                   0x80000000L
#define MH_DEBUG_REG29__TC_ROQ_FULL                        0x80000000L

// MH_DEBUG_REG30
#define MH_DEBUG_REG30__SAME_ROW_BANK_WIN_MASK             0x000000ffL
#define MH_DEBUG_REG30__SAME_ROW_BANK_REQ_MASK             0x0000ff00L
#define MH_DEBUG_REG30__NON_SAME_ROW_BANK_WIN_MASK         0x00ff0000L
#define MH_DEBUG_REG30__NON_SAME_ROW_BANK_REQ_MASK         0xff000000L

// MH_DEBUG_REG31
#define MH_DEBUG_REG31__TC_MH_send_MASK                    0x00000001L
#define MH_DEBUG_REG31__TC_MH_send                         0x00000001L
#define MH_DEBUG_REG31__TC_ROQ_RTR_q_MASK                  0x00000002L
#define MH_DEBUG_REG31__TC_ROQ_RTR_q                       0x00000002L
#define MH_DEBUG_REG31__ROQ_MARK_q_0_MASK                  0x00000004L
#define MH_DEBUG_REG31__ROQ_MARK_q_0                       0x00000004L
#define MH_DEBUG_REG31__ROQ_VALID_q_0_MASK                 0x00000008L
#define MH_DEBUG_REG31__ROQ_VALID_q_0                      0x00000008L
#define MH_DEBUG_REG31__SAME_ROW_BANK_q_0_MASK             0x00000010L
#define MH_DEBUG_REG31__SAME_ROW_BANK_q_0                  0x00000010L
#define MH_DEBUG_REG31__ROQ_ADDR_0_MASK                    0xffffffe0L

// MH_DEBUG_REG32
#define MH_DEBUG_REG32__TC_MH_send_MASK                    0x00000001L
#define MH_DEBUG_REG32__TC_MH_send                         0x00000001L
#define MH_DEBUG_REG32__TC_ROQ_RTR_q_MASK                  0x00000002L
#define MH_DEBUG_REG32__TC_ROQ_RTR_q                       0x00000002L
#define MH_DEBUG_REG32__ROQ_MARK_q_1_MASK                  0x00000004L
#define MH_DEBUG_REG32__ROQ_MARK_q_1                       0x00000004L
#define MH_DEBUG_REG32__ROQ_VALID_q_1_MASK                 0x00000008L
#define MH_DEBUG_REG32__ROQ_VALID_q_1                      0x00000008L
#define MH_DEBUG_REG32__SAME_ROW_BANK_q_1_MASK             0x00000010L
#define MH_DEBUG_REG32__SAME_ROW_BANK_q_1                  0x00000010L
#define MH_DEBUG_REG32__ROQ_ADDR_1_MASK                    0xffffffe0L

// MH_DEBUG_REG33
#define MH_DEBUG_REG33__TC_MH_send_MASK                    0x00000001L
#define MH_DEBUG_REG33__TC_MH_send                         0x00000001L
#define MH_DEBUG_REG33__TC_ROQ_RTR_q_MASK                  0x00000002L
#define MH_DEBUG_REG33__TC_ROQ_RTR_q                       0x00000002L
#define MH_DEBUG_REG33__ROQ_MARK_q_2_MASK                  0x00000004L
#define MH_DEBUG_REG33__ROQ_MARK_q_2                       0x00000004L
#define MH_DEBUG_REG33__ROQ_VALID_q_2_MASK                 0x00000008L
#define MH_DEBUG_REG33__ROQ_VALID_q_2                      0x00000008L
#define MH_DEBUG_REG33__SAME_ROW_BANK_q_2_MASK             0x00000010L
#define MH_DEBUG_REG33__SAME_ROW_BANK_q_2                  0x00000010L
#define MH_DEBUG_REG33__ROQ_ADDR_2_MASK                    0xffffffe0L

// MH_DEBUG_REG34
#define MH_DEBUG_REG34__TC_MH_send_MASK                    0x00000001L
#define MH_DEBUG_REG34__TC_MH_send                         0x00000001L
#define MH_DEBUG_REG34__TC_ROQ_RTR_q_MASK                  0x00000002L
#define MH_DEBUG_REG34__TC_ROQ_RTR_q                       0x00000002L
#define MH_DEBUG_REG34__ROQ_MARK_q_3_MASK                  0x00000004L
#define MH_DEBUG_REG34__ROQ_MARK_q_3                       0x00000004L
#define MH_DEBUG_REG34__ROQ_VALID_q_3_MASK                 0x00000008L
#define MH_DEBUG_REG34__ROQ_VALID_q_3                      0x00000008L
#define MH_DEBUG_REG34__SAME_ROW_BANK_q_3_MASK             0x00000010L
#define MH_DEBUG_REG34__SAME_ROW_BANK_q_3                  0x00000010L
#define MH_DEBUG_REG34__ROQ_ADDR_3_MASK                    0xffffffe0L

// MH_DEBUG_REG35
#define MH_DEBUG_REG35__TC_MH_send_MASK                    0x00000001L
#define MH_DEBUG_REG35__TC_MH_send                         0x00000001L
#define MH_DEBUG_REG35__TC_ROQ_RTR_q_MASK                  0x00000002L
#define MH_DEBUG_REG35__TC_ROQ_RTR_q                       0x00000002L
#define MH_DEBUG_REG35__ROQ_MARK_q_4_MASK                  0x00000004L
#define MH_DEBUG_REG35__ROQ_MARK_q_4                       0x00000004L
#define MH_DEBUG_REG35__ROQ_VALID_q_4_MASK                 0x00000008L
#define MH_DEBUG_REG35__ROQ_VALID_q_4                      0x00000008L
#define MH_DEBUG_REG35__SAME_ROW_BANK_q_4_MASK             0x00000010L
#define MH_DEBUG_REG35__SAME_ROW_BANK_q_4                  0x00000010L
#define MH_DEBUG_REG35__ROQ_ADDR_4_MASK                    0xffffffe0L

// MH_DEBUG_REG36
#define MH_DEBUG_REG36__TC_MH_send_MASK                    0x00000001L
#define MH_DEBUG_REG36__TC_MH_send                         0x00000001L
#define MH_DEBUG_REG36__TC_ROQ_RTR_q_MASK                  0x00000002L
#define MH_DEBUG_REG36__TC_ROQ_RTR_q                       0x00000002L
#define MH_DEBUG_REG36__ROQ_MARK_q_5_MASK                  0x00000004L
#define MH_DEBUG_REG36__ROQ_MARK_q_5                       0x00000004L
#define MH_DEBUG_REG36__ROQ_VALID_q_5_MASK                 0x00000008L
#define MH_DEBUG_REG36__ROQ_VALID_q_5                      0x00000008L
#define MH_DEBUG_REG36__SAME_ROW_BANK_q_5_MASK             0x00000010L
#define MH_DEBUG_REG36__SAME_ROW_BANK_q_5                  0x00000010L
#define MH_DEBUG_REG36__ROQ_ADDR_5_MASK                    0xffffffe0L

// MH_DEBUG_REG37
#define MH_DEBUG_REG37__TC_MH_send_MASK                    0x00000001L
#define MH_DEBUG_REG37__TC_MH_send                         0x00000001L
#define MH_DEBUG_REG37__TC_ROQ_RTR_q_MASK                  0x00000002L
#define MH_DEBUG_REG37__TC_ROQ_RTR_q                       0x00000002L
#define MH_DEBUG_REG37__ROQ_MARK_q_6_MASK                  0x00000004L
#define MH_DEBUG_REG37__ROQ_MARK_q_6                       0x00000004L
#define MH_DEBUG_REG37__ROQ_VALID_q_6_MASK                 0x00000008L
#define MH_DEBUG_REG37__ROQ_VALID_q_6                      0x00000008L
#define MH_DEBUG_REG37__SAME_ROW_BANK_q_6_MASK             0x00000010L
#define MH_DEBUG_REG37__SAME_ROW_BANK_q_6                  0x00000010L
#define MH_DEBUG_REG37__ROQ_ADDR_6_MASK                    0xffffffe0L

// MH_DEBUG_REG38
#define MH_DEBUG_REG38__TC_MH_send_MASK                    0x00000001L
#define MH_DEBUG_REG38__TC_MH_send                         0x00000001L
#define MH_DEBUG_REG38__TC_ROQ_RTR_q_MASK                  0x00000002L
#define MH_DEBUG_REG38__TC_ROQ_RTR_q                       0x00000002L
#define MH_DEBUG_REG38__ROQ_MARK_q_7_MASK                  0x00000004L
#define MH_DEBUG_REG38__ROQ_MARK_q_7                       0x00000004L
#define MH_DEBUG_REG38__ROQ_VALID_q_7_MASK                 0x00000008L
#define MH_DEBUG_REG38__ROQ_VALID_q_7                      0x00000008L
#define MH_DEBUG_REG38__SAME_ROW_BANK_q_7_MASK             0x00000010L
#define MH_DEBUG_REG38__SAME_ROW_BANK_q_7                  0x00000010L
#define MH_DEBUG_REG38__ROQ_ADDR_7_MASK                    0xffffffe0L

// MH_DEBUG_REG39
#define MH_DEBUG_REG39__ARB_WE_MASK                        0x00000001L
#define MH_DEBUG_REG39__ARB_WE                             0x00000001L
#define MH_DEBUG_REG39__MMU_RTR_MASK                       0x00000002L
#define MH_DEBUG_REG39__MMU_RTR                            0x00000002L
#define MH_DEBUG_REG39__ARB_ID_q_MASK                      0x0000001cL
#define MH_DEBUG_REG39__ARB_WRITE_q_MASK                   0x00000020L
#define MH_DEBUG_REG39__ARB_WRITE_q                        0x00000020L
#define MH_DEBUG_REG39__ARB_BLEN_q_MASK                    0x00000040L
#define MH_DEBUG_REG39__ARB_BLEN_q                         0x00000040L
#define MH_DEBUG_REG39__ARQ_CTRL_EMPTY_MASK                0x00000080L
#define MH_DEBUG_REG39__ARQ_CTRL_EMPTY                     0x00000080L
#define MH_DEBUG_REG39__ARQ_FIFO_CNT_q_MASK                0x00000700L
#define MH_DEBUG_REG39__MMU_WE_MASK                        0x00000800L
#define MH_DEBUG_REG39__MMU_WE                             0x00000800L
#define MH_DEBUG_REG39__ARQ_RTR_MASK                       0x00001000L
#define MH_DEBUG_REG39__ARQ_RTR                            0x00001000L
#define MH_DEBUG_REG39__MMU_ID_MASK                        0x0000e000L
#define MH_DEBUG_REG39__MMU_WRITE_MASK                     0x00010000L
#define MH_DEBUG_REG39__MMU_WRITE                          0x00010000L
#define MH_DEBUG_REG39__MMU_BLEN_MASK                      0x00020000L
#define MH_DEBUG_REG39__MMU_BLEN                           0x00020000L

// MH_DEBUG_REG40
#define MH_DEBUG_REG40__ARB_WE_MASK                        0x00000001L
#define MH_DEBUG_REG40__ARB_WE                             0x00000001L
#define MH_DEBUG_REG40__ARB_ID_q_MASK                      0x0000000eL
#define MH_DEBUG_REG40__ARB_VAD_q_MASK                     0xfffffff0L

// MH_DEBUG_REG41
#define MH_DEBUG_REG41__MMU_WE_MASK                        0x00000001L
#define MH_DEBUG_REG41__MMU_WE                             0x00000001L
#define MH_DEBUG_REG41__MMU_ID_MASK                        0x0000000eL
#define MH_DEBUG_REG41__MMU_PAD_MASK                       0xfffffff0L

// MH_DEBUG_REG42
#define MH_DEBUG_REG42__WDB_WE_MASK                        0x00000001L
#define MH_DEBUG_REG42__WDB_WE                             0x00000001L
#define MH_DEBUG_REG42__WDB_RTR_SKID_MASK                  0x00000002L
#define MH_DEBUG_REG42__WDB_RTR_SKID                       0x00000002L
#define MH_DEBUG_REG42__ARB_WSTRB_q_MASK                   0x000003fcL
#define MH_DEBUG_REG42__ARB_WLAST_MASK                     0x00000400L
#define MH_DEBUG_REG42__ARB_WLAST                          0x00000400L
#define MH_DEBUG_REG42__WDB_CTRL_EMPTY_MASK                0x00000800L
#define MH_DEBUG_REG42__WDB_CTRL_EMPTY                     0x00000800L
#define MH_DEBUG_REG42__WDB_FIFO_CNT_q_MASK                0x0001f000L
#define MH_DEBUG_REG42__WDC_WDB_RE_q_MASK                  0x00020000L
#define MH_DEBUG_REG42__WDC_WDB_RE_q                       0x00020000L
#define MH_DEBUG_REG42__WDB_WDC_WID_MASK                   0x001c0000L
#define MH_DEBUG_REG42__WDB_WDC_WLAST_MASK                 0x00200000L
#define MH_DEBUG_REG42__WDB_WDC_WLAST                      0x00200000L
#define MH_DEBUG_REG42__WDB_WDC_WSTRB_MASK                 0x3fc00000L

// MH_DEBUG_REG43
#define MH_DEBUG_REG43__ARB_WDATA_q_31_0_MASK              0xffffffffL

// MH_DEBUG_REG44
#define MH_DEBUG_REG44__ARB_WDATA_q_63_32_MASK             0xffffffffL

// MH_DEBUG_REG45
#define MH_DEBUG_REG45__WDB_WDC_WDATA_31_0_MASK            0xffffffffL

// MH_DEBUG_REG46
#define MH_DEBUG_REG46__WDB_WDC_WDATA_63_32_MASK           0xffffffffL

// MH_DEBUG_REG47
#define MH_DEBUG_REG47__CTRL_ARC_EMPTY_MASK                0x00000001L
#define MH_DEBUG_REG47__CTRL_ARC_EMPTY                     0x00000001L
#define MH_DEBUG_REG47__CTRL_RARC_EMPTY_MASK               0x00000002L
#define MH_DEBUG_REG47__CTRL_RARC_EMPTY                    0x00000002L
#define MH_DEBUG_REG47__ARQ_CTRL_EMPTY_MASK                0x00000004L
#define MH_DEBUG_REG47__ARQ_CTRL_EMPTY                     0x00000004L
#define MH_DEBUG_REG47__ARQ_CTRL_WRITE_MASK                0x00000008L
#define MH_DEBUG_REG47__ARQ_CTRL_WRITE                     0x00000008L
#define MH_DEBUG_REG47__TLBMISS_CTRL_RTS_MASK              0x00000010L
#define MH_DEBUG_REG47__TLBMISS_CTRL_RTS                   0x00000010L
#define MH_DEBUG_REG47__CTRL_TLBMISS_RE_q_MASK             0x00000020L
#define MH_DEBUG_REG47__CTRL_TLBMISS_RE_q                  0x00000020L
#define MH_DEBUG_REG47__INFLT_LIMIT_q_MASK                 0x00000040L
#define MH_DEBUG_REG47__INFLT_LIMIT_q                      0x00000040L
#define MH_DEBUG_REG47__INFLT_LIMIT_CNT_q_MASK             0x00001f80L
#define MH_DEBUG_REG47__ARC_CTRL_RE_q_MASK                 0x00002000L
#define MH_DEBUG_REG47__ARC_CTRL_RE_q                      0x00002000L
#define MH_DEBUG_REG47__RARC_CTRL_RE_q_MASK                0x00004000L
#define MH_DEBUG_REG47__RARC_CTRL_RE_q                     0x00004000L
#define MH_DEBUG_REG47__RVALID_q_MASK                      0x00008000L
#define MH_DEBUG_REG47__RVALID_q                           0x00008000L
#define MH_DEBUG_REG47__RREADY_q_MASK                      0x00010000L
#define MH_DEBUG_REG47__RREADY_q                           0x00010000L
#define MH_DEBUG_REG47__RLAST_q_MASK                       0x00020000L
#define MH_DEBUG_REG47__RLAST_q                            0x00020000L
#define MH_DEBUG_REG47__BVALID_q_MASK                      0x00040000L
#define MH_DEBUG_REG47__BVALID_q                           0x00040000L
#define MH_DEBUG_REG47__BREADY_q_MASK                      0x00080000L
#define MH_DEBUG_REG47__BREADY_q                           0x00080000L

// MH_DEBUG_REG48
#define MH_DEBUG_REG48__MH_CP_grb_send_MASK                0x00000001L
#define MH_DEBUG_REG48__MH_CP_grb_send                     0x00000001L
#define MH_DEBUG_REG48__MH_VGT_grb_send_MASK               0x00000002L
#define MH_DEBUG_REG48__MH_VGT_grb_send                    0x00000002L
#define MH_DEBUG_REG48__MH_TC_mcsend_MASK                  0x00000004L
#define MH_DEBUG_REG48__MH_TC_mcsend                       0x00000004L
#define MH_DEBUG_REG48__MH_TLBMISS_SEND_MASK               0x00000008L
#define MH_DEBUG_REG48__MH_TLBMISS_SEND                    0x00000008L
#define MH_DEBUG_REG48__TLBMISS_VALID_MASK                 0x00000010L
#define MH_DEBUG_REG48__TLBMISS_VALID                      0x00000010L
#define MH_DEBUG_REG48__RDC_VALID_MASK                     0x00000020L
#define MH_DEBUG_REG48__RDC_VALID                          0x00000020L
#define MH_DEBUG_REG48__RDC_RID_MASK                       0x000001c0L
#define MH_DEBUG_REG48__RDC_RLAST_MASK                     0x00000200L
#define MH_DEBUG_REG48__RDC_RLAST                          0x00000200L
#define MH_DEBUG_REG48__RDC_RRESP_MASK                     0x00000c00L
#define MH_DEBUG_REG48__TLBMISS_CTRL_RTS_MASK              0x00001000L
#define MH_DEBUG_REG48__TLBMISS_CTRL_RTS                   0x00001000L
#define MH_DEBUG_REG48__CTRL_TLBMISS_RE_q_MASK             0x00002000L
#define MH_DEBUG_REG48__CTRL_TLBMISS_RE_q                  0x00002000L
#define MH_DEBUG_REG48__MMU_ID_REQUEST_q_MASK              0x00004000L
#define MH_DEBUG_REG48__MMU_ID_REQUEST_q                   0x00004000L
#define MH_DEBUG_REG48__OUTSTANDING_MMUID_CNT_q_MASK       0x001f8000L
#define MH_DEBUG_REG48__MMU_ID_RESPONSE_MASK               0x00200000L
#define MH_DEBUG_REG48__MMU_ID_RESPONSE                    0x00200000L
#define MH_DEBUG_REG48__TLBMISS_RETURN_CNT_q_MASK          0x0fc00000L
#define MH_DEBUG_REG48__CNT_HOLD_q1_MASK                   0x10000000L
#define MH_DEBUG_REG48__CNT_HOLD_q1                        0x10000000L
#define MH_DEBUG_REG48__MH_CLNT_AXI_ID_REUSE_MMUr_ID_MASK  0xe0000000L

// MH_DEBUG_REG49
#define MH_DEBUG_REG49__RF_MMU_PAGE_FAULT_MASK             0xffffffffL

// MH_DEBUG_REG50
#define MH_DEBUG_REG50__RF_MMU_CONFIG_q_MASK               0x00ffffffL
#define MH_DEBUG_REG50__ARB_ID_q_MASK                      0x07000000L
#define MH_DEBUG_REG50__ARB_WRITE_q_MASK                   0x08000000L
#define MH_DEBUG_REG50__ARB_WRITE_q                        0x08000000L
#define MH_DEBUG_REG50__client_behavior_q_MASK             0x30000000L
#define MH_DEBUG_REG50__ARB_WE_MASK                        0x40000000L
#define MH_DEBUG_REG50__ARB_WE                             0x40000000L
#define MH_DEBUG_REG50__MMU_RTR_MASK                       0x80000000L
#define MH_DEBUG_REG50__MMU_RTR                            0x80000000L

// MH_DEBUG_REG51
#define MH_DEBUG_REG51__stage1_valid_MASK                  0x00000001L
#define MH_DEBUG_REG51__stage1_valid                       0x00000001L
#define MH_DEBUG_REG51__IGNORE_TAG_MISS_q_MASK             0x00000002L
#define MH_DEBUG_REG51__IGNORE_TAG_MISS_q                  0x00000002L
#define MH_DEBUG_REG51__pa_in_mpu_range_MASK               0x00000004L
#define MH_DEBUG_REG51__pa_in_mpu_range                    0x00000004L
#define MH_DEBUG_REG51__tag_match_q_MASK                   0x00000008L
#define MH_DEBUG_REG51__tag_match_q                        0x00000008L
#define MH_DEBUG_REG51__tag_miss_q_MASK                    0x00000010L
#define MH_DEBUG_REG51__tag_miss_q                         0x00000010L
#define MH_DEBUG_REG51__va_in_range_q_MASK                 0x00000020L
#define MH_DEBUG_REG51__va_in_range_q                      0x00000020L
#define MH_DEBUG_REG51__MMU_MISS_MASK                      0x00000040L
#define MH_DEBUG_REG51__MMU_MISS                           0x00000040L
#define MH_DEBUG_REG51__MMU_READ_MISS_MASK                 0x00000080L
#define MH_DEBUG_REG51__MMU_READ_MISS                      0x00000080L
#define MH_DEBUG_REG51__MMU_WRITE_MISS_MASK                0x00000100L
#define MH_DEBUG_REG51__MMU_WRITE_MISS                     0x00000100L
#define MH_DEBUG_REG51__MMU_HIT_MASK                       0x00000200L
#define MH_DEBUG_REG51__MMU_HIT                            0x00000200L
#define MH_DEBUG_REG51__MMU_READ_HIT_MASK                  0x00000400L
#define MH_DEBUG_REG51__MMU_READ_HIT                       0x00000400L
#define MH_DEBUG_REG51__MMU_WRITE_HIT_MASK                 0x00000800L
#define MH_DEBUG_REG51__MMU_WRITE_HIT                      0x00000800L
#define MH_DEBUG_REG51__MMU_SPLIT_MODE_TC_MISS_MASK        0x00001000L
#define MH_DEBUG_REG51__MMU_SPLIT_MODE_TC_MISS             0x00001000L
#define MH_DEBUG_REG51__MMU_SPLIT_MODE_TC_HIT_MASK         0x00002000L
#define MH_DEBUG_REG51__MMU_SPLIT_MODE_TC_HIT              0x00002000L
#define MH_DEBUG_REG51__MMU_SPLIT_MODE_nonTC_MISS_MASK     0x00004000L
#define MH_DEBUG_REG51__MMU_SPLIT_MODE_nonTC_MISS          0x00004000L
#define MH_DEBUG_REG51__MMU_SPLIT_MODE_nonTC_HIT_MASK      0x00008000L
#define MH_DEBUG_REG51__MMU_SPLIT_MODE_nonTC_HIT           0x00008000L
#define MH_DEBUG_REG51__REQ_VA_OFFSET_q_MASK               0xffff0000L

// MH_DEBUG_REG52
#define MH_DEBUG_REG52__ARQ_RTR_MASK                       0x00000001L
#define MH_DEBUG_REG52__ARQ_RTR                            0x00000001L
#define MH_DEBUG_REG52__MMU_WE_MASK                        0x00000002L
#define MH_DEBUG_REG52__MMU_WE                             0x00000002L
#define MH_DEBUG_REG52__CTRL_TLBMISS_RE_q_MASK             0x00000004L
#define MH_DEBUG_REG52__CTRL_TLBMISS_RE_q                  0x00000004L
#define MH_DEBUG_REG52__TLBMISS_CTRL_RTS_MASK              0x00000008L
#define MH_DEBUG_REG52__TLBMISS_CTRL_RTS                   0x00000008L
#define MH_DEBUG_REG52__MH_TLBMISS_SEND_MASK               0x00000010L
#define MH_DEBUG_REG52__MH_TLBMISS_SEND                    0x00000010L
#define MH_DEBUG_REG52__MMU_STALL_AWAITING_TLB_MISS_FETCH_MASK 0x00000020L
#define MH_DEBUG_REG52__MMU_STALL_AWAITING_TLB_MISS_FETCH  0x00000020L
#define MH_DEBUG_REG52__pa_in_mpu_range_MASK               0x00000040L
#define MH_DEBUG_REG52__pa_in_mpu_range                    0x00000040L
#define MH_DEBUG_REG52__stage1_valid_MASK                  0x00000080L
#define MH_DEBUG_REG52__stage1_valid                       0x00000080L
#define MH_DEBUG_REG52__stage2_valid_MASK                  0x00000100L
#define MH_DEBUG_REG52__stage2_valid                       0x00000100L
#define MH_DEBUG_REG52__client_behavior_q_MASK             0x00000600L
#define MH_DEBUG_REG52__IGNORE_TAG_MISS_q_MASK             0x00000800L
#define MH_DEBUG_REG52__IGNORE_TAG_MISS_q                  0x00000800L
#define MH_DEBUG_REG52__tag_match_q_MASK                   0x00001000L
#define MH_DEBUG_REG52__tag_match_q                        0x00001000L
#define MH_DEBUG_REG52__tag_miss_q_MASK                    0x00002000L
#define MH_DEBUG_REG52__tag_miss_q                         0x00002000L
#define MH_DEBUG_REG52__va_in_range_q_MASK                 0x00004000L
#define MH_DEBUG_REG52__va_in_range_q                      0x00004000L
#define MH_DEBUG_REG52__PTE_FETCH_COMPLETE_q_MASK          0x00008000L
#define MH_DEBUG_REG52__PTE_FETCH_COMPLETE_q               0x00008000L
#define MH_DEBUG_REG52__TAG_valid_q_MASK                   0xffff0000L

// MH_DEBUG_REG53
#define MH_DEBUG_REG53__TAG0_VA_MASK                       0x00001fffL
#define MH_DEBUG_REG53__TAG_valid_q_0_MASK                 0x00002000L
#define MH_DEBUG_REG53__TAG_valid_q_0                      0x00002000L
#define MH_DEBUG_REG53__ALWAYS_ZERO_MASK                   0x0000c000L
#define MH_DEBUG_REG53__TAG1_VA_MASK                       0x1fff0000L
#define MH_DEBUG_REG53__TAG_valid_q_1_MASK                 0x20000000L
#define MH_DEBUG_REG53__TAG_valid_q_1                      0x20000000L

// MH_DEBUG_REG54
#define MH_DEBUG_REG54__TAG2_VA_MASK                       0x00001fffL
#define MH_DEBUG_REG54__TAG_valid_q_2_MASK                 0x00002000L
#define MH_DEBUG_REG54__TAG_valid_q_2                      0x00002000L
#define MH_DEBUG_REG54__ALWAYS_ZERO_MASK                   0x0000c000L
#define MH_DEBUG_REG54__TAG3_VA_MASK                       0x1fff0000L
#define MH_DEBUG_REG54__TAG_valid_q_3_MASK                 0x20000000L
#define MH_DEBUG_REG54__TAG_valid_q_3                      0x20000000L

// MH_DEBUG_REG55
#define MH_DEBUG_REG55__TAG4_VA_MASK                       0x00001fffL
#define MH_DEBUG_REG55__TAG_valid_q_4_MASK                 0x00002000L
#define MH_DEBUG_REG55__TAG_valid_q_4                      0x00002000L
#define MH_DEBUG_REG55__ALWAYS_ZERO_MASK                   0x0000c000L
#define MH_DEBUG_REG55__TAG5_VA_MASK                       0x1fff0000L
#define MH_DEBUG_REG55__TAG_valid_q_5_MASK                 0x20000000L
#define MH_DEBUG_REG55__TAG_valid_q_5                      0x20000000L

// MH_DEBUG_REG56
#define MH_DEBUG_REG56__TAG6_VA_MASK                       0x00001fffL
#define MH_DEBUG_REG56__TAG_valid_q_6_MASK                 0x00002000L
#define MH_DEBUG_REG56__TAG_valid_q_6                      0x00002000L
#define MH_DEBUG_REG56__ALWAYS_ZERO_MASK                   0x0000c000L
#define MH_DEBUG_REG56__TAG7_VA_MASK                       0x1fff0000L
#define MH_DEBUG_REG56__TAG_valid_q_7_MASK                 0x20000000L
#define MH_DEBUG_REG56__TAG_valid_q_7                      0x20000000L

// MH_DEBUG_REG57
#define MH_DEBUG_REG57__TAG8_VA_MASK                       0x00001fffL
#define MH_DEBUG_REG57__TAG_valid_q_8_MASK                 0x00002000L
#define MH_DEBUG_REG57__TAG_valid_q_8                      0x00002000L
#define MH_DEBUG_REG57__ALWAYS_ZERO_MASK                   0x0000c000L
#define MH_DEBUG_REG57__TAG9_VA_MASK                       0x1fff0000L
#define MH_DEBUG_REG57__TAG_valid_q_9_MASK                 0x20000000L
#define MH_DEBUG_REG57__TAG_valid_q_9                      0x20000000L

// MH_DEBUG_REG58
#define MH_DEBUG_REG58__TAG10_VA_MASK                      0x00001fffL
#define MH_DEBUG_REG58__TAG_valid_q_10_MASK                0x00002000L
#define MH_DEBUG_REG58__TAG_valid_q_10                     0x00002000L
#define MH_DEBUG_REG58__ALWAYS_ZERO_MASK                   0x0000c000L
#define MH_DEBUG_REG58__TAG11_VA_MASK                      0x1fff0000L
#define MH_DEBUG_REG58__TAG_valid_q_11_MASK                0x20000000L
#define MH_DEBUG_REG58__TAG_valid_q_11                     0x20000000L

// MH_DEBUG_REG59
#define MH_DEBUG_REG59__TAG12_VA_MASK                      0x00001fffL
#define MH_DEBUG_REG59__TAG_valid_q_12_MASK                0x00002000L
#define MH_DEBUG_REG59__TAG_valid_q_12                     0x00002000L
#define MH_DEBUG_REG59__ALWAYS_ZERO_MASK                   0x0000c000L
#define MH_DEBUG_REG59__TAG13_VA_MASK                      0x1fff0000L
#define MH_DEBUG_REG59__TAG_valid_q_13_MASK                0x20000000L
#define MH_DEBUG_REG59__TAG_valid_q_13                     0x20000000L

// MH_DEBUG_REG60
#define MH_DEBUG_REG60__TAG14_VA_MASK                      0x00001fffL
#define MH_DEBUG_REG60__TAG_valid_q_14_MASK                0x00002000L
#define MH_DEBUG_REG60__TAG_valid_q_14                     0x00002000L
#define MH_DEBUG_REG60__ALWAYS_ZERO_MASK                   0x0000c000L
#define MH_DEBUG_REG60__TAG15_VA_MASK                      0x1fff0000L
#define MH_DEBUG_REG60__TAG_valid_q_15_MASK                0x20000000L
#define MH_DEBUG_REG60__TAG_valid_q_15                     0x20000000L

// MH_DEBUG_REG61
#define MH_DEBUG_REG61__MH_DBG_DEFAULT_MASK                0xffffffffL

// MH_DEBUG_REG62
#define MH_DEBUG_REG62__MH_DBG_DEFAULT_MASK                0xffffffffL

// MH_DEBUG_REG63
#define MH_DEBUG_REG63__MH_DBG_DEFAULT_MASK                0xffffffffL

// MH_MMU_CONFIG
#define MH_MMU_CONFIG__MMU_ENABLE_MASK                     0x00000001L
#define MH_MMU_CONFIG__MMU_ENABLE                          0x00000001L
#define MH_MMU_CONFIG__SPLIT_MODE_ENABLE_MASK              0x00000002L
#define MH_MMU_CONFIG__SPLIT_MODE_ENABLE                   0x00000002L
#define MH_MMU_CONFIG__RESERVED1_MASK                      0x0000000cL
#define MH_MMU_CONFIG__RB_W_CLNT_BEHAVIOR_MASK             0x00000030L
#define MH_MMU_CONFIG__CP_W_CLNT_BEHAVIOR_MASK             0x000000c0L
#define MH_MMU_CONFIG__CP_R0_CLNT_BEHAVIOR_MASK            0x00000300L
#define MH_MMU_CONFIG__CP_R1_CLNT_BEHAVIOR_MASK            0x00000c00L
#define MH_MMU_CONFIG__CP_R2_CLNT_BEHAVIOR_MASK            0x00003000L
#define MH_MMU_CONFIG__CP_R3_CLNT_BEHAVIOR_MASK            0x0000c000L
#define MH_MMU_CONFIG__CP_R4_CLNT_BEHAVIOR_MASK            0x00030000L
#define MH_MMU_CONFIG__VGT_R0_CLNT_BEHAVIOR_MASK           0x000c0000L
#define MH_MMU_CONFIG__VGT_R1_CLNT_BEHAVIOR_MASK           0x00300000L
#define MH_MMU_CONFIG__TC_R_CLNT_BEHAVIOR_MASK             0x00c00000L

// MH_MMU_VA_RANGE
#define MH_MMU_VA_RANGE__NUM_64KB_REGIONS_MASK             0x00000fffL
#define MH_MMU_VA_RANGE__VA_BASE_MASK                      0xfffff000L

// MH_MMU_PT_BASE
#define MH_MMU_PT_BASE__PT_BASE_MASK                       0xfffff000L

// MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT__PAGE_FAULT_MASK                 0x00000001L
#define MH_MMU_PAGE_FAULT__PAGE_FAULT                      0x00000001L
#define MH_MMU_PAGE_FAULT__OP_TYPE_MASK                    0x00000002L
#define MH_MMU_PAGE_FAULT__OP_TYPE                         0x00000002L
#define MH_MMU_PAGE_FAULT__CLNT_BEHAVIOR_MASK              0x0000000cL
#define MH_MMU_PAGE_FAULT__AXI_ID_MASK                     0x00000070L
#define MH_MMU_PAGE_FAULT__RESERVED1_MASK                  0x00000080L
#define MH_MMU_PAGE_FAULT__RESERVED1                       0x00000080L
#define MH_MMU_PAGE_FAULT__MPU_ADDRESS_OUT_OF_RANGE_MASK   0x00000100L
#define MH_MMU_PAGE_FAULT__MPU_ADDRESS_OUT_OF_RANGE        0x00000100L
#define MH_MMU_PAGE_FAULT__ADDRESS_OUT_OF_RANGE_MASK       0x00000200L
#define MH_MMU_PAGE_FAULT__ADDRESS_OUT_OF_RANGE            0x00000200L
#define MH_MMU_PAGE_FAULT__READ_PROTECTION_ERROR_MASK      0x00000400L
#define MH_MMU_PAGE_FAULT__READ_PROTECTION_ERROR           0x00000400L
#define MH_MMU_PAGE_FAULT__WRITE_PROTECTION_ERROR_MASK     0x00000800L
#define MH_MMU_PAGE_FAULT__WRITE_PROTECTION_ERROR          0x00000800L
#define MH_MMU_PAGE_FAULT__REQ_VA_MASK                     0xfffff000L

// MH_MMU_TRAN_ERROR
#define MH_MMU_TRAN_ERROR__TRAN_ERROR_MASK                 0xffffffe0L

// MH_MMU_INVALIDATE
#define MH_MMU_INVALIDATE__INVALIDATE_ALL_MASK             0x00000001L
#define MH_MMU_INVALIDATE__INVALIDATE_ALL                  0x00000001L
#define MH_MMU_INVALIDATE__INVALIDATE_TC_MASK              0x00000002L
#define MH_MMU_INVALIDATE__INVALIDATE_TC                   0x00000002L

// MH_MMU_MPU_BASE
#define MH_MMU_MPU_BASE__MPU_BASE_MASK                     0xfffff000L

// MH_MMU_MPU_END
#define MH_MMU_MPU_END__MPU_END_MASK                       0xfffff000L

// WAIT_UNTIL
#define WAIT_UNTIL__WAIT_RE_VSYNC_MASK                     0x00000002L
#define WAIT_UNTIL__WAIT_RE_VSYNC                          0x00000002L
#define WAIT_UNTIL__WAIT_FE_VSYNC_MASK                     0x00000004L
#define WAIT_UNTIL__WAIT_FE_VSYNC                          0x00000004L
#define WAIT_UNTIL__WAIT_VSYNC_MASK                        0x00000008L
#define WAIT_UNTIL__WAIT_VSYNC                             0x00000008L
#define WAIT_UNTIL__WAIT_DSPLY_ID0_MASK                    0x00000010L
#define WAIT_UNTIL__WAIT_DSPLY_ID0                         0x00000010L
#define WAIT_UNTIL__WAIT_DSPLY_ID1_MASK                    0x00000020L
#define WAIT_UNTIL__WAIT_DSPLY_ID1                         0x00000020L
#define WAIT_UNTIL__WAIT_DSPLY_ID2_MASK                    0x00000040L
#define WAIT_UNTIL__WAIT_DSPLY_ID2                         0x00000040L
#define WAIT_UNTIL__WAIT_CMDFIFO_MASK                      0x00000400L
#define WAIT_UNTIL__WAIT_CMDFIFO                           0x00000400L
#define WAIT_UNTIL__WAIT_2D_IDLE_MASK                      0x00004000L
#define WAIT_UNTIL__WAIT_2D_IDLE                           0x00004000L
#define WAIT_UNTIL__WAIT_3D_IDLE_MASK                      0x00008000L
#define WAIT_UNTIL__WAIT_3D_IDLE                           0x00008000L
#define WAIT_UNTIL__WAIT_2D_IDLECLEAN_MASK                 0x00010000L
#define WAIT_UNTIL__WAIT_2D_IDLECLEAN                      0x00010000L
#define WAIT_UNTIL__WAIT_3D_IDLECLEAN_MASK                 0x00020000L
#define WAIT_UNTIL__WAIT_3D_IDLECLEAN                      0x00020000L
#define WAIT_UNTIL__CMDFIFO_ENTRIES_MASK                   0x00f00000L

// RBBM_ISYNC_CNTL
#define RBBM_ISYNC_CNTL__ISYNC_WAIT_IDLEGUI_MASK           0x00000010L
#define RBBM_ISYNC_CNTL__ISYNC_WAIT_IDLEGUI                0x00000010L
#define RBBM_ISYNC_CNTL__ISYNC_CPSCRATCH_IDLEGUI_MASK      0x00000020L
#define RBBM_ISYNC_CNTL__ISYNC_CPSCRATCH_IDLEGUI           0x00000020L

// RBBM_STATUS
#define RBBM_STATUS__CMDFIFO_AVAIL_MASK                    0x0000001fL
#define RBBM_STATUS__TC_BUSY_MASK                          0x00000020L
#define RBBM_STATUS__TC_BUSY                               0x00000020L
#define RBBM_STATUS__HIRQ_PENDING_MASK                     0x00000100L
#define RBBM_STATUS__HIRQ_PENDING                          0x00000100L
#define RBBM_STATUS__CPRQ_PENDING_MASK                     0x00000200L
#define RBBM_STATUS__CPRQ_PENDING                          0x00000200L
#define RBBM_STATUS__CFRQ_PENDING_MASK                     0x00000400L
#define RBBM_STATUS__CFRQ_PENDING                          0x00000400L
#define RBBM_STATUS__PFRQ_PENDING_MASK                     0x00000800L
#define RBBM_STATUS__PFRQ_PENDING                          0x00000800L
#define RBBM_STATUS__VGT_BUSY_NO_DMA_MASK                  0x00001000L
#define RBBM_STATUS__VGT_BUSY_NO_DMA                       0x00001000L
#define RBBM_STATUS__RBBM_WU_BUSY_MASK                     0x00004000L
#define RBBM_STATUS__RBBM_WU_BUSY                          0x00004000L
#define RBBM_STATUS__CP_NRT_BUSY_MASK                      0x00010000L
#define RBBM_STATUS__CP_NRT_BUSY                           0x00010000L
#define RBBM_STATUS__MH_BUSY_MASK                          0x00040000L
#define RBBM_STATUS__MH_BUSY                               0x00040000L
#define RBBM_STATUS__MH_COHERENCY_BUSY_MASK                0x00080000L
#define RBBM_STATUS__MH_COHERENCY_BUSY                     0x00080000L
#define RBBM_STATUS__SX_BUSY_MASK                          0x00200000L
#define RBBM_STATUS__SX_BUSY                               0x00200000L
#define RBBM_STATUS__TPC_BUSY_MASK                         0x00400000L
#define RBBM_STATUS__TPC_BUSY                              0x00400000L
#define RBBM_STATUS__SC_CNTX_BUSY_MASK                     0x01000000L
#define RBBM_STATUS__SC_CNTX_BUSY                          0x01000000L
#define RBBM_STATUS__PA_BUSY_MASK                          0x02000000L
#define RBBM_STATUS__PA_BUSY                               0x02000000L
#define RBBM_STATUS__VGT_BUSY_MASK                         0x04000000L
#define RBBM_STATUS__VGT_BUSY                              0x04000000L
#define RBBM_STATUS__SQ_CNTX17_BUSY_MASK                   0x08000000L
#define RBBM_STATUS__SQ_CNTX17_BUSY                        0x08000000L
#define RBBM_STATUS__SQ_CNTX0_BUSY_MASK                    0x10000000L
#define RBBM_STATUS__SQ_CNTX0_BUSY                         0x10000000L
#define RBBM_STATUS__RB_CNTX_BUSY_MASK                     0x40000000L
#define RBBM_STATUS__RB_CNTX_BUSY                          0x40000000L
#define RBBM_STATUS__GUI_ACTIVE_MASK                       0x80000000L
#define RBBM_STATUS__GUI_ACTIVE                            0x80000000L

// RBBM_DSPLY
#define RBBM_DSPLY__DISPLAY_ID0_ACTIVE_MASK                0x00000001L
#define RBBM_DSPLY__DISPLAY_ID0_ACTIVE                     0x00000001L
#define RBBM_DSPLY__DISPLAY_ID1_ACTIVE_MASK                0x00000002L
#define RBBM_DSPLY__DISPLAY_ID1_ACTIVE                     0x00000002L
#define RBBM_DSPLY__DISPLAY_ID2_ACTIVE_MASK                0x00000004L
#define RBBM_DSPLY__DISPLAY_ID2_ACTIVE                     0x00000004L
#define RBBM_DSPLY__VSYNC_ACTIVE_MASK                      0x00000008L
#define RBBM_DSPLY__VSYNC_ACTIVE                           0x00000008L
#define RBBM_DSPLY__USE_DISPLAY_ID0_MASK                   0x00000010L
#define RBBM_DSPLY__USE_DISPLAY_ID0                        0x00000010L
#define RBBM_DSPLY__USE_DISPLAY_ID1_MASK                   0x00000020L
#define RBBM_DSPLY__USE_DISPLAY_ID1                        0x00000020L
#define RBBM_DSPLY__USE_DISPLAY_ID2_MASK                   0x00000040L
#define RBBM_DSPLY__USE_DISPLAY_ID2                        0x00000040L
#define RBBM_DSPLY__SW_CNTL_MASK                           0x00000080L
#define RBBM_DSPLY__SW_CNTL                                0x00000080L
#define RBBM_DSPLY__NUM_BUFS_MASK                          0x00000300L

// RBBM_RENDER_LATEST
#define RBBM_RENDER_LATEST__BUFFER_ID_MASK                 0x00000003L

// RBBM_RTL_RELEASE
#define RBBM_RTL_RELEASE__CHANGELIST_MASK                  0xffffffffL

// RBBM_PATCH_RELEASE
#define RBBM_PATCH_RELEASE__PATCH_REVISION_MASK            0x0000ffffL
#define RBBM_PATCH_RELEASE__PATCH_SELECTION_MASK           0x00ff0000L
#define RBBM_PATCH_RELEASE__CUSTOMER_ID_MASK               0xff000000L

// RBBM_AUXILIARY_CONFIG
#define RBBM_AUXILIARY_CONFIG__RESERVED_MASK               0xffffffffL

// RBBM_PERIPHID0
#define RBBM_PERIPHID0__PARTNUMBER0_MASK                   0x000000ffL

// RBBM_PERIPHID1
#define RBBM_PERIPHID1__PARTNUMBER1_MASK                   0x0000000fL
#define RBBM_PERIPHID1__DESIGNER0_MASK                     0x000000f0L

// RBBM_PERIPHID2
#define RBBM_PERIPHID2__DESIGNER1_MASK                     0x0000000fL
#define RBBM_PERIPHID2__REVISION_MASK                      0x000000f0L

// RBBM_PERIPHID3
#define RBBM_PERIPHID3__RBBM_HOST_INTERFACE_MASK           0x00000003L
#define RBBM_PERIPHID3__GARB_SLAVE_INTERFACE_MASK          0x0000000cL
#define RBBM_PERIPHID3__MH_INTERFACE_MASK                  0x00000030L
#define RBBM_PERIPHID3__CONTINUATION_MASK                  0x00000080L
#define RBBM_PERIPHID3__CONTINUATION                       0x00000080L

// RBBM_CNTL
#define RBBM_CNTL__READ_TIMEOUT_MASK                       0x000000ffL
#define RBBM_CNTL__REGCLK_DEASSERT_TIME_MASK               0x0001ff00L

// RBBM_SKEW_CNTL
#define RBBM_SKEW_CNTL__SKEW_TOP_THRESHOLD_MASK            0x0000001fL
#define RBBM_SKEW_CNTL__SKEW_COUNT_MASK                    0x000003e0L

// RBBM_SOFT_RESET
#define RBBM_SOFT_RESET__SOFT_RESET_CP_MASK                0x00000001L
#define RBBM_SOFT_RESET__SOFT_RESET_CP                     0x00000001L
#define RBBM_SOFT_RESET__SOFT_RESET_PA_MASK                0x00000004L
#define RBBM_SOFT_RESET__SOFT_RESET_PA                     0x00000004L
#define RBBM_SOFT_RESET__SOFT_RESET_MH_MASK                0x00000008L
#define RBBM_SOFT_RESET__SOFT_RESET_MH                     0x00000008L
#define RBBM_SOFT_RESET__SOFT_RESET_BC_MASK                0x00000010L
#define RBBM_SOFT_RESET__SOFT_RESET_BC                     0x00000010L
#define RBBM_SOFT_RESET__SOFT_RESET_SQ_MASK                0x00000020L
#define RBBM_SOFT_RESET__SOFT_RESET_SQ                     0x00000020L
#define RBBM_SOFT_RESET__SOFT_RESET_SX_MASK                0x00000040L
#define RBBM_SOFT_RESET__SOFT_RESET_SX                     0x00000040L
#define RBBM_SOFT_RESET__SOFT_RESET_CIB_MASK               0x00001000L
#define RBBM_SOFT_RESET__SOFT_RESET_CIB                    0x00001000L
#define RBBM_SOFT_RESET__SOFT_RESET_SC_MASK                0x00008000L
#define RBBM_SOFT_RESET__SOFT_RESET_SC                     0x00008000L
#define RBBM_SOFT_RESET__SOFT_RESET_VGT_MASK               0x00010000L
#define RBBM_SOFT_RESET__SOFT_RESET_VGT                    0x00010000L

// RBBM_PM_OVERRIDE1
#define RBBM_PM_OVERRIDE1__RBBM_AHBCLK_PM_OVERRIDE_MASK    0x00000001L
#define RBBM_PM_OVERRIDE1__RBBM_AHBCLK_PM_OVERRIDE         0x00000001L
#define RBBM_PM_OVERRIDE1__SC_REG_SCLK_PM_OVERRIDE_MASK    0x00000002L
#define RBBM_PM_OVERRIDE1__SC_REG_SCLK_PM_OVERRIDE         0x00000002L
#define RBBM_PM_OVERRIDE1__SC_SCLK_PM_OVERRIDE_MASK        0x00000004L
#define RBBM_PM_OVERRIDE1__SC_SCLK_PM_OVERRIDE             0x00000004L
#define RBBM_PM_OVERRIDE1__SP_TOP_SCLK_PM_OVERRIDE_MASK    0x00000008L
#define RBBM_PM_OVERRIDE1__SP_TOP_SCLK_PM_OVERRIDE         0x00000008L
#define RBBM_PM_OVERRIDE1__SP_V0_SCLK_PM_OVERRIDE_MASK     0x00000010L
#define RBBM_PM_OVERRIDE1__SP_V0_SCLK_PM_OVERRIDE          0x00000010L
#define RBBM_PM_OVERRIDE1__SQ_REG_SCLK_PM_OVERRIDE_MASK    0x00000020L
#define RBBM_PM_OVERRIDE1__SQ_REG_SCLK_PM_OVERRIDE         0x00000020L
#define RBBM_PM_OVERRIDE1__SQ_REG_FIFOS_SCLK_PM_OVERRIDE_MASK 0x00000040L
#define RBBM_PM_OVERRIDE1__SQ_REG_FIFOS_SCLK_PM_OVERRIDE   0x00000040L
#define RBBM_PM_OVERRIDE1__SQ_CONST_MEM_SCLK_PM_OVERRIDE_MASK 0x00000080L
#define RBBM_PM_OVERRIDE1__SQ_CONST_MEM_SCLK_PM_OVERRIDE   0x00000080L
#define RBBM_PM_OVERRIDE1__SQ_SQ_SCLK_PM_OVERRIDE_MASK     0x00000100L
#define RBBM_PM_OVERRIDE1__SQ_SQ_SCLK_PM_OVERRIDE          0x00000100L
#define RBBM_PM_OVERRIDE1__SX_SCLK_PM_OVERRIDE_MASK        0x00000200L
#define RBBM_PM_OVERRIDE1__SX_SCLK_PM_OVERRIDE             0x00000200L
#define RBBM_PM_OVERRIDE1__SX_REG_SCLK_PM_OVERRIDE_MASK    0x00000400L
#define RBBM_PM_OVERRIDE1__SX_REG_SCLK_PM_OVERRIDE         0x00000400L
#define RBBM_PM_OVERRIDE1__TCM_TCO_SCLK_PM_OVERRIDE_MASK   0x00000800L
#define RBBM_PM_OVERRIDE1__TCM_TCO_SCLK_PM_OVERRIDE        0x00000800L
#define RBBM_PM_OVERRIDE1__TCM_TCM_SCLK_PM_OVERRIDE_MASK   0x00001000L
#define RBBM_PM_OVERRIDE1__TCM_TCM_SCLK_PM_OVERRIDE        0x00001000L
#define RBBM_PM_OVERRIDE1__TCM_TCD_SCLK_PM_OVERRIDE_MASK   0x00002000L
#define RBBM_PM_OVERRIDE1__TCM_TCD_SCLK_PM_OVERRIDE        0x00002000L
#define RBBM_PM_OVERRIDE1__TCM_REG_SCLK_PM_OVERRIDE_MASK   0x00004000L
#define RBBM_PM_OVERRIDE1__TCM_REG_SCLK_PM_OVERRIDE        0x00004000L
#define RBBM_PM_OVERRIDE1__TPC_TPC_SCLK_PM_OVERRIDE_MASK   0x00008000L
#define RBBM_PM_OVERRIDE1__TPC_TPC_SCLK_PM_OVERRIDE        0x00008000L
#define RBBM_PM_OVERRIDE1__TPC_REG_SCLK_PM_OVERRIDE_MASK   0x00010000L
#define RBBM_PM_OVERRIDE1__TPC_REG_SCLK_PM_OVERRIDE        0x00010000L
#define RBBM_PM_OVERRIDE1__TCF_TCA_SCLK_PM_OVERRIDE_MASK   0x00020000L
#define RBBM_PM_OVERRIDE1__TCF_TCA_SCLK_PM_OVERRIDE        0x00020000L
#define RBBM_PM_OVERRIDE1__TCF_TCB_SCLK_PM_OVERRIDE_MASK   0x00040000L
#define RBBM_PM_OVERRIDE1__TCF_TCB_SCLK_PM_OVERRIDE        0x00040000L
#define RBBM_PM_OVERRIDE1__TCF_TCB_READ_SCLK_PM_OVERRIDE_MASK 0x00080000L
#define RBBM_PM_OVERRIDE1__TCF_TCB_READ_SCLK_PM_OVERRIDE   0x00080000L
#define RBBM_PM_OVERRIDE1__TP_TP_SCLK_PM_OVERRIDE_MASK     0x00100000L
#define RBBM_PM_OVERRIDE1__TP_TP_SCLK_PM_OVERRIDE          0x00100000L
#define RBBM_PM_OVERRIDE1__TP_REG_SCLK_PM_OVERRIDE_MASK    0x00200000L
#define RBBM_PM_OVERRIDE1__TP_REG_SCLK_PM_OVERRIDE         0x00200000L
#define RBBM_PM_OVERRIDE1__CP_G_SCLK_PM_OVERRIDE_MASK      0x00400000L
#define RBBM_PM_OVERRIDE1__CP_G_SCLK_PM_OVERRIDE           0x00400000L
#define RBBM_PM_OVERRIDE1__CP_REG_SCLK_PM_OVERRIDE_MASK    0x00800000L
#define RBBM_PM_OVERRIDE1__CP_REG_SCLK_PM_OVERRIDE         0x00800000L
#define RBBM_PM_OVERRIDE1__CP_G_REG_SCLK_PM_OVERRIDE_MASK  0x01000000L
#define RBBM_PM_OVERRIDE1__CP_G_REG_SCLK_PM_OVERRIDE       0x01000000L
#define RBBM_PM_OVERRIDE1__SPI_SCLK_PM_OVERRIDE_MASK       0x02000000L
#define RBBM_PM_OVERRIDE1__SPI_SCLK_PM_OVERRIDE            0x02000000L
#define RBBM_PM_OVERRIDE1__RB_REG_SCLK_PM_OVERRIDE_MASK    0x04000000L
#define RBBM_PM_OVERRIDE1__RB_REG_SCLK_PM_OVERRIDE         0x04000000L
#define RBBM_PM_OVERRIDE1__RB_SCLK_PM_OVERRIDE_MASK        0x08000000L
#define RBBM_PM_OVERRIDE1__RB_SCLK_PM_OVERRIDE             0x08000000L
#define RBBM_PM_OVERRIDE1__MH_MH_SCLK_PM_OVERRIDE_MASK     0x10000000L
#define RBBM_PM_OVERRIDE1__MH_MH_SCLK_PM_OVERRIDE          0x10000000L
#define RBBM_PM_OVERRIDE1__MH_REG_SCLK_PM_OVERRIDE_MASK    0x20000000L
#define RBBM_PM_OVERRIDE1__MH_REG_SCLK_PM_OVERRIDE         0x20000000L
#define RBBM_PM_OVERRIDE1__MH_MMU_SCLK_PM_OVERRIDE_MASK    0x40000000L
#define RBBM_PM_OVERRIDE1__MH_MMU_SCLK_PM_OVERRIDE         0x40000000L
#define RBBM_PM_OVERRIDE1__MH_TCROQ_SCLK_PM_OVERRIDE_MASK  0x80000000L
#define RBBM_PM_OVERRIDE1__MH_TCROQ_SCLK_PM_OVERRIDE       0x80000000L

// RBBM_PM_OVERRIDE2
#define RBBM_PM_OVERRIDE2__PA_REG_SCLK_PM_OVERRIDE_MASK    0x00000001L
#define RBBM_PM_OVERRIDE2__PA_REG_SCLK_PM_OVERRIDE         0x00000001L
#define RBBM_PM_OVERRIDE2__PA_PA_SCLK_PM_OVERRIDE_MASK     0x00000002L
#define RBBM_PM_OVERRIDE2__PA_PA_SCLK_PM_OVERRIDE          0x00000002L
#define RBBM_PM_OVERRIDE2__PA_AG_SCLK_PM_OVERRIDE_MASK     0x00000004L
#define RBBM_PM_OVERRIDE2__PA_AG_SCLK_PM_OVERRIDE          0x00000004L
#define RBBM_PM_OVERRIDE2__VGT_REG_SCLK_PM_OVERRIDE_MASK   0x00000008L
#define RBBM_PM_OVERRIDE2__VGT_REG_SCLK_PM_OVERRIDE        0x00000008L
#define RBBM_PM_OVERRIDE2__VGT_FIFOS_SCLK_PM_OVERRIDE_MASK 0x00000010L
#define RBBM_PM_OVERRIDE2__VGT_FIFOS_SCLK_PM_OVERRIDE      0x00000010L
#define RBBM_PM_OVERRIDE2__VGT_VGT_SCLK_PM_OVERRIDE_MASK   0x00000020L
#define RBBM_PM_OVERRIDE2__VGT_VGT_SCLK_PM_OVERRIDE        0x00000020L
#define RBBM_PM_OVERRIDE2__DEBUG_PERF_SCLK_PM_OVERRIDE_MASK 0x00000040L
#define RBBM_PM_OVERRIDE2__DEBUG_PERF_SCLK_PM_OVERRIDE     0x00000040L
#define RBBM_PM_OVERRIDE2__PERM_SCLK_PM_OVERRIDE_MASK      0x00000080L
#define RBBM_PM_OVERRIDE2__PERM_SCLK_PM_OVERRIDE           0x00000080L
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM0_PM_OVERRIDE_MASK    0x00000100L
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM0_PM_OVERRIDE         0x00000100L
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM1_PM_OVERRIDE_MASK    0x00000200L
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM1_PM_OVERRIDE         0x00000200L
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM2_PM_OVERRIDE_MASK    0x00000400L
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM2_PM_OVERRIDE         0x00000400L
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM3_PM_OVERRIDE_MASK    0x00000800L
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM3_PM_OVERRIDE         0x00000800L

// GC_SYS_IDLE
#define GC_SYS_IDLE__GC_SYS_IDLE_DELAY_MASK                0x0000ffffL
#define GC_SYS_IDLE__GC_SYS_IDLE_OVERRIDE_MASK             0x80000000L
#define GC_SYS_IDLE__GC_SYS_IDLE_OVERRIDE                  0x80000000L

// NQWAIT_UNTIL
#define NQWAIT_UNTIL__WAIT_GUI_IDLE_MASK                   0x00000001L
#define NQWAIT_UNTIL__WAIT_GUI_IDLE                        0x00000001L

// RBBM_DEBUG
#define RBBM_DEBUG__IGNORE_RTR_MASK                        0x00000002L
#define RBBM_DEBUG__IGNORE_RTR                             0x00000002L
#define RBBM_DEBUG__IGNORE_CP_SCHED_WU_MASK                0x00000004L
#define RBBM_DEBUG__IGNORE_CP_SCHED_WU                     0x00000004L
#define RBBM_DEBUG__IGNORE_CP_SCHED_ISYNC_MASK             0x00000008L
#define RBBM_DEBUG__IGNORE_CP_SCHED_ISYNC                  0x00000008L
#define RBBM_DEBUG__IGNORE_CP_SCHED_NQ_HI_MASK             0x00000010L
#define RBBM_DEBUG__IGNORE_CP_SCHED_NQ_HI                  0x00000010L
#define RBBM_DEBUG__HYSTERESIS_NRT_GUI_ACTIVE_MASK         0x00000f00L
#define RBBM_DEBUG__IGNORE_RTR_FOR_HI_MASK                 0x00010000L
#define RBBM_DEBUG__IGNORE_RTR_FOR_HI                      0x00010000L
#define RBBM_DEBUG__IGNORE_CP_RBBM_NRTRTR_FOR_HI_MASK      0x00020000L
#define RBBM_DEBUG__IGNORE_CP_RBBM_NRTRTR_FOR_HI           0x00020000L
#define RBBM_DEBUG__IGNORE_VGT_RBBM_NRTRTR_FOR_HI_MASK     0x00040000L
#define RBBM_DEBUG__IGNORE_VGT_RBBM_NRTRTR_FOR_HI          0x00040000L
#define RBBM_DEBUG__IGNORE_SQ_RBBM_NRTRTR_FOR_HI_MASK      0x00080000L
#define RBBM_DEBUG__IGNORE_SQ_RBBM_NRTRTR_FOR_HI           0x00080000L
#define RBBM_DEBUG__CP_RBBM_NRTRTR_MASK                    0x00100000L
#define RBBM_DEBUG__CP_RBBM_NRTRTR                         0x00100000L
#define RBBM_DEBUG__VGT_RBBM_NRTRTR_MASK                   0x00200000L
#define RBBM_DEBUG__VGT_RBBM_NRTRTR                        0x00200000L
#define RBBM_DEBUG__SQ_RBBM_NRTRTR_MASK                    0x00400000L
#define RBBM_DEBUG__SQ_RBBM_NRTRTR                         0x00400000L
#define RBBM_DEBUG__CLIENTS_FOR_NRT_RTR_FOR_HI_MASK        0x00800000L
#define RBBM_DEBUG__CLIENTS_FOR_NRT_RTR_FOR_HI             0x00800000L
#define RBBM_DEBUG__CLIENTS_FOR_NRT_RTR_MASK               0x01000000L
#define RBBM_DEBUG__CLIENTS_FOR_NRT_RTR                    0x01000000L
#define RBBM_DEBUG__IGNORE_SX_RBBM_BUSY_MASK               0x80000000L
#define RBBM_DEBUG__IGNORE_SX_RBBM_BUSY                    0x80000000L

// RBBM_READ_ERROR
#define RBBM_READ_ERROR__READ_ADDRESS_MASK                 0x0001fffcL
#define RBBM_READ_ERROR__READ_REQUESTER_MASK               0x40000000L
#define RBBM_READ_ERROR__READ_REQUESTER                    0x40000000L
#define RBBM_READ_ERROR__READ_ERROR_MASK                   0x80000000L
#define RBBM_READ_ERROR__READ_ERROR                        0x80000000L

// RBBM_WAIT_IDLE_CLOCKS
#define RBBM_WAIT_IDLE_CLOCKS__WAIT_IDLE_CLOCKS_NRT_MASK   0x000000ffL

// RBBM_INT_CNTL
#define RBBM_INT_CNTL__RDERR_INT_MASK_MASK                 0x00000001L
#define RBBM_INT_CNTL__RDERR_INT_MASK                      0x00000001L
#define RBBM_INT_CNTL__DISPLAY_UPDATE_INT_MASK_MASK        0x00000002L
#define RBBM_INT_CNTL__DISPLAY_UPDATE_INT_MASK             0x00000002L
#define RBBM_INT_CNTL__GUI_IDLE_INT_MASK_MASK              0x00080000L
#define RBBM_INT_CNTL__GUI_IDLE_INT_MASK                   0x00080000L

// RBBM_INT_STATUS
#define RBBM_INT_STATUS__RDERR_INT_STAT_MASK               0x00000001L
#define RBBM_INT_STATUS__RDERR_INT_STAT                    0x00000001L
#define RBBM_INT_STATUS__DISPLAY_UPDATE_INT_STAT_MASK      0x00000002L
#define RBBM_INT_STATUS__DISPLAY_UPDATE_INT_STAT           0x00000002L
#define RBBM_INT_STATUS__GUI_IDLE_INT_STAT_MASK            0x00080000L
#define RBBM_INT_STATUS__GUI_IDLE_INT_STAT                 0x00080000L

// RBBM_INT_ACK
#define RBBM_INT_ACK__RDERR_INT_ACK_MASK                   0x00000001L
#define RBBM_INT_ACK__RDERR_INT_ACK                        0x00000001L
#define RBBM_INT_ACK__DISPLAY_UPDATE_INT_ACK_MASK          0x00000002L
#define RBBM_INT_ACK__DISPLAY_UPDATE_INT_ACK               0x00000002L
#define RBBM_INT_ACK__GUI_IDLE_INT_ACK_MASK                0x00080000L
#define RBBM_INT_ACK__GUI_IDLE_INT_ACK                     0x00080000L

// MASTER_INT_SIGNAL
#define MASTER_INT_SIGNAL__MH_INT_STAT_MASK                0x00000020L
#define MASTER_INT_SIGNAL__MH_INT_STAT                     0x00000020L
#define MASTER_INT_SIGNAL__CP_INT_STAT_MASK                0x40000000L
#define MASTER_INT_SIGNAL__CP_INT_STAT                     0x40000000L
#define MASTER_INT_SIGNAL__RBBM_INT_STAT_MASK              0x80000000L
#define MASTER_INT_SIGNAL__RBBM_INT_STAT                   0x80000000L

// RBBM_PERFCOUNTER1_SELECT
#define RBBM_PERFCOUNTER1_SELECT__PERF_COUNT1_SEL_MASK     0x0000003fL

// RBBM_PERFCOUNTER1_LO
#define RBBM_PERFCOUNTER1_LO__PERF_COUNT1_LO_MASK          0xffffffffL

// RBBM_PERFCOUNTER1_HI
#define RBBM_PERFCOUNTER1_HI__PERF_COUNT1_HI_MASK          0x0000ffffL

// CP_RB_BASE
#define CP_RB_BASE__RB_BASE_MASK                           0xffffffe0L

// CP_RB_CNTL
#define CP_RB_CNTL__RB_BUFSZ_MASK                          0x0000003fL
#define CP_RB_CNTL__RB_BLKSZ_MASK                          0x00003f00L
#define CP_RB_CNTL__BUF_SWAP_MASK                          0x00030000L
#define CP_RB_CNTL__RB_POLL_EN_MASK                        0x00100000L
#define CP_RB_CNTL__RB_POLL_EN                             0x00100000L
#define CP_RB_CNTL__RB_NO_UPDATE_MASK                      0x08000000L
#define CP_RB_CNTL__RB_NO_UPDATE                           0x08000000L
#define CP_RB_CNTL__RB_RPTR_WR_ENA_MASK                    0x80000000L
#define CP_RB_CNTL__RB_RPTR_WR_ENA                         0x80000000L

// CP_RB_RPTR_ADDR
#define CP_RB_RPTR_ADDR__RB_RPTR_SWAP_MASK                 0x00000003L
#define CP_RB_RPTR_ADDR__RB_RPTR_ADDR_MASK                 0xfffffffcL

// CP_RB_RPTR
#define CP_RB_RPTR__RB_RPTR_MASK                           0x000fffffL

// CP_RB_RPTR_WR
#define CP_RB_RPTR_WR__RB_RPTR_WR_MASK                     0x000fffffL

// CP_RB_WPTR
#define CP_RB_WPTR__RB_WPTR_MASK                           0x000fffffL

// CP_RB_WPTR_DELAY
#define CP_RB_WPTR_DELAY__PRE_WRITE_TIMER_MASK             0x0fffffffL
#define CP_RB_WPTR_DELAY__PRE_WRITE_LIMIT_MASK             0xf0000000L

// CP_RB_WPTR_BASE
#define CP_RB_WPTR_BASE__RB_WPTR_SWAP_MASK                 0x00000003L
#define CP_RB_WPTR_BASE__RB_WPTR_BASE_MASK                 0xfffffffcL

// CP_IB1_BASE
#define CP_IB1_BASE__IB1_BASE_MASK                         0xfffffffcL

// CP_IB1_BUFSZ
#define CP_IB1_BUFSZ__IB1_BUFSZ_MASK                       0x000fffffL

// CP_IB2_BASE
#define CP_IB2_BASE__IB2_BASE_MASK                         0xfffffffcL

// CP_IB2_BUFSZ
#define CP_IB2_BUFSZ__IB2_BUFSZ_MASK                       0x000fffffL

// CP_ST_BASE
#define CP_ST_BASE__ST_BASE_MASK                           0xfffffffcL

// CP_ST_BUFSZ
#define CP_ST_BUFSZ__ST_BUFSZ_MASK                         0x000fffffL

// CP_QUEUE_THRESHOLDS
#define CP_QUEUE_THRESHOLDS__CSQ_IB1_START_MASK            0x0000000fL
#define CP_QUEUE_THRESHOLDS__CSQ_IB2_START_MASK            0x00000f00L
#define CP_QUEUE_THRESHOLDS__CSQ_ST_START_MASK             0x000f0000L

// CP_MEQ_THRESHOLDS
#define CP_MEQ_THRESHOLDS__MEQ_END_MASK                    0x001f0000L
#define CP_MEQ_THRESHOLDS__ROQ_END_MASK                    0x1f000000L

// CP_CSQ_AVAIL
#define CP_CSQ_AVAIL__CSQ_CNT_RING_MASK                    0x0000007fL
#define CP_CSQ_AVAIL__CSQ_CNT_IB1_MASK                     0x00007f00L
#define CP_CSQ_AVAIL__CSQ_CNT_IB2_MASK                     0x007f0000L

// CP_STQ_AVAIL
#define CP_STQ_AVAIL__STQ_CNT_ST_MASK                      0x0000007fL

// CP_MEQ_AVAIL
#define CP_MEQ_AVAIL__MEQ_CNT_MASK                         0x0000001fL

// CP_CSQ_RB_STAT
#define CP_CSQ_RB_STAT__CSQ_RPTR_PRIMARY_MASK              0x0000007fL
#define CP_CSQ_RB_STAT__CSQ_WPTR_PRIMARY_MASK              0x007f0000L

// CP_CSQ_IB1_STAT
#define CP_CSQ_IB1_STAT__CSQ_RPTR_INDIRECT1_MASK           0x0000007fL
#define CP_CSQ_IB1_STAT__CSQ_WPTR_INDIRECT1_MASK           0x007f0000L

// CP_CSQ_IB2_STAT
#define CP_CSQ_IB2_STAT__CSQ_RPTR_INDIRECT2_MASK           0x0000007fL
#define CP_CSQ_IB2_STAT__CSQ_WPTR_INDIRECT2_MASK           0x007f0000L

// CP_NON_PREFETCH_CNTRS
#define CP_NON_PREFETCH_CNTRS__IB1_COUNTER_MASK            0x00000007L
#define CP_NON_PREFETCH_CNTRS__IB2_COUNTER_MASK            0x00000700L

// CP_STQ_ST_STAT
#define CP_STQ_ST_STAT__STQ_RPTR_ST_MASK                   0x0000007fL
#define CP_STQ_ST_STAT__STQ_WPTR_ST_MASK                   0x007f0000L

// CP_MEQ_STAT
#define CP_MEQ_STAT__MEQ_RPTR_MASK                         0x000003ffL
#define CP_MEQ_STAT__MEQ_WPTR_MASK                         0x03ff0000L

// CP_MIU_TAG_STAT
#define CP_MIU_TAG_STAT__TAG_0_STAT_MASK                   0x00000001L
#define CP_MIU_TAG_STAT__TAG_0_STAT                        0x00000001L
#define CP_MIU_TAG_STAT__TAG_1_STAT_MASK                   0x00000002L
#define CP_MIU_TAG_STAT__TAG_1_STAT                        0x00000002L
#define CP_MIU_TAG_STAT__TAG_2_STAT_MASK                   0x00000004L
#define CP_MIU_TAG_STAT__TAG_2_STAT                        0x00000004L
#define CP_MIU_TAG_STAT__TAG_3_STAT_MASK                   0x00000008L
#define CP_MIU_TAG_STAT__TAG_3_STAT                        0x00000008L
#define CP_MIU_TAG_STAT__TAG_4_STAT_MASK                   0x00000010L
#define CP_MIU_TAG_STAT__TAG_4_STAT                        0x00000010L
#define CP_MIU_TAG_STAT__TAG_5_STAT_MASK                   0x00000020L
#define CP_MIU_TAG_STAT__TAG_5_STAT                        0x00000020L
#define CP_MIU_TAG_STAT__TAG_6_STAT_MASK                   0x00000040L
#define CP_MIU_TAG_STAT__TAG_6_STAT                        0x00000040L
#define CP_MIU_TAG_STAT__TAG_7_STAT_MASK                   0x00000080L
#define CP_MIU_TAG_STAT__TAG_7_STAT                        0x00000080L
#define CP_MIU_TAG_STAT__TAG_8_STAT_MASK                   0x00000100L
#define CP_MIU_TAG_STAT__TAG_8_STAT                        0x00000100L
#define CP_MIU_TAG_STAT__TAG_9_STAT_MASK                   0x00000200L
#define CP_MIU_TAG_STAT__TAG_9_STAT                        0x00000200L
#define CP_MIU_TAG_STAT__TAG_10_STAT_MASK                  0x00000400L
#define CP_MIU_TAG_STAT__TAG_10_STAT                       0x00000400L
#define CP_MIU_TAG_STAT__TAG_11_STAT_MASK                  0x00000800L
#define CP_MIU_TAG_STAT__TAG_11_STAT                       0x00000800L
#define CP_MIU_TAG_STAT__TAG_12_STAT_MASK                  0x00001000L
#define CP_MIU_TAG_STAT__TAG_12_STAT                       0x00001000L
#define CP_MIU_TAG_STAT__TAG_13_STAT_MASK                  0x00002000L
#define CP_MIU_TAG_STAT__TAG_13_STAT                       0x00002000L
#define CP_MIU_TAG_STAT__TAG_14_STAT_MASK                  0x00004000L
#define CP_MIU_TAG_STAT__TAG_14_STAT                       0x00004000L
#define CP_MIU_TAG_STAT__TAG_15_STAT_MASK                  0x00008000L
#define CP_MIU_TAG_STAT__TAG_15_STAT                       0x00008000L
#define CP_MIU_TAG_STAT__TAG_16_STAT_MASK                  0x00010000L
#define CP_MIU_TAG_STAT__TAG_16_STAT                       0x00010000L
#define CP_MIU_TAG_STAT__TAG_17_STAT_MASK                  0x00020000L
#define CP_MIU_TAG_STAT__TAG_17_STAT                       0x00020000L
#define CP_MIU_TAG_STAT__INVALID_RETURN_TAG_MASK           0x80000000L
#define CP_MIU_TAG_STAT__INVALID_RETURN_TAG                0x80000000L

// CP_CMD_INDEX
#define CP_CMD_INDEX__CMD_INDEX_MASK                       0x0000007fL
#define CP_CMD_INDEX__CMD_QUEUE_SEL_MASK                   0x00030000L

// CP_CMD_DATA
#define CP_CMD_DATA__CMD_DATA_MASK                         0xffffffffL

// CP_ME_CNTL
#define CP_ME_CNTL__ME_STATMUX_MASK                        0x0000ffffL
#define CP_ME_CNTL__VTX_DEALLOC_FIFO_EMPTY_MASK            0x02000000L
#define CP_ME_CNTL__VTX_DEALLOC_FIFO_EMPTY                 0x02000000L
#define CP_ME_CNTL__PIX_DEALLOC_FIFO_EMPTY_MASK            0x04000000L
#define CP_ME_CNTL__PIX_DEALLOC_FIFO_EMPTY                 0x04000000L
#define CP_ME_CNTL__ME_HALT_MASK                           0x10000000L
#define CP_ME_CNTL__ME_HALT                                0x10000000L
#define CP_ME_CNTL__ME_BUSY_MASK                           0x20000000L
#define CP_ME_CNTL__ME_BUSY                                0x20000000L
#define CP_ME_CNTL__PROG_CNT_SIZE_MASK                     0x80000000L
#define CP_ME_CNTL__PROG_CNT_SIZE                          0x80000000L

// CP_ME_STATUS
#define CP_ME_STATUS__ME_DEBUG_DATA_MASK                   0xffffffffL

// CP_ME_RAM_WADDR
#define CP_ME_RAM_WADDR__ME_RAM_WADDR_MASK                 0x000003ffL

// CP_ME_RAM_RADDR
#define CP_ME_RAM_RADDR__ME_RAM_RADDR_MASK                 0x000003ffL

// CP_ME_RAM_DATA
#define CP_ME_RAM_DATA__ME_RAM_DATA_MASK                   0xffffffffL

// CP_ME_RDADDR
#define CP_ME_RDADDR__ME_RDADDR_MASK                       0xffffffffL

// CP_DEBUG
#define CP_DEBUG__CP_DEBUG_UNUSED_22_to_0_MASK             0x007fffffL
#define CP_DEBUG__PREDICATE_DISABLE_MASK                   0x00800000L
#define CP_DEBUG__PREDICATE_DISABLE                        0x00800000L
#define CP_DEBUG__PROG_END_PTR_ENABLE_MASK                 0x01000000L
#define CP_DEBUG__PROG_END_PTR_ENABLE                      0x01000000L
#define CP_DEBUG__MIU_128BIT_WRITE_ENABLE_MASK             0x02000000L
#define CP_DEBUG__MIU_128BIT_WRITE_ENABLE                  0x02000000L
#define CP_DEBUG__PREFETCH_PASS_NOPS_MASK                  0x04000000L
#define CP_DEBUG__PREFETCH_PASS_NOPS                       0x04000000L
#define CP_DEBUG__DYNAMIC_CLK_DISABLE_MASK                 0x08000000L
#define CP_DEBUG__DYNAMIC_CLK_DISABLE                      0x08000000L
#define CP_DEBUG__PREFETCH_MATCH_DISABLE_MASK              0x10000000L
#define CP_DEBUG__PREFETCH_MATCH_DISABLE                   0x10000000L
#define CP_DEBUG__SIMPLE_ME_FLOW_CONTROL_MASK              0x40000000L
#define CP_DEBUG__SIMPLE_ME_FLOW_CONTROL                   0x40000000L
#define CP_DEBUG__MIU_WRITE_PACK_DISABLE_MASK              0x80000000L
#define CP_DEBUG__MIU_WRITE_PACK_DISABLE                   0x80000000L

// SCRATCH_REG0
#define SCRATCH_REG0__SCRATCH_REG0_MASK                    0xffffffffL
#define GUI_SCRATCH_REG0__SCRATCH_REG0_MASK                0xffffffffL

// SCRATCH_REG1
#define SCRATCH_REG1__SCRATCH_REG1_MASK                    0xffffffffL
#define GUI_SCRATCH_REG1__SCRATCH_REG1_MASK                0xffffffffL

// SCRATCH_REG2
#define SCRATCH_REG2__SCRATCH_REG2_MASK                    0xffffffffL
#define GUI_SCRATCH_REG2__SCRATCH_REG2_MASK                0xffffffffL

// SCRATCH_REG3
#define SCRATCH_REG3__SCRATCH_REG3_MASK                    0xffffffffL
#define GUI_SCRATCH_REG3__SCRATCH_REG3_MASK                0xffffffffL

// SCRATCH_REG4
#define SCRATCH_REG4__SCRATCH_REG4_MASK                    0xffffffffL
#define GUI_SCRATCH_REG4__SCRATCH_REG4_MASK                0xffffffffL

// SCRATCH_REG5
#define SCRATCH_REG5__SCRATCH_REG5_MASK                    0xffffffffL
#define GUI_SCRATCH_REG5__SCRATCH_REG5_MASK                0xffffffffL

// SCRATCH_REG6
#define SCRATCH_REG6__SCRATCH_REG6_MASK                    0xffffffffL
#define GUI_SCRATCH_REG6__SCRATCH_REG6_MASK                0xffffffffL

// SCRATCH_REG7
#define SCRATCH_REG7__SCRATCH_REG7_MASK                    0xffffffffL
#define GUI_SCRATCH_REG7__SCRATCH_REG7_MASK                0xffffffffL

// SCRATCH_UMSK
#define SCRATCH_UMSK__SCRATCH_UMSK_MASK                    0x000000ffL
#define SCRATCH_UMSK__SCRATCH_SWAP_MASK                    0x00030000L

// SCRATCH_ADDR
#define SCRATCH_ADDR__SCRATCH_ADDR_MASK                    0xffffffe0L

// CP_ME_VS_EVENT_SRC
#define CP_ME_VS_EVENT_SRC__VS_DONE_SWM_MASK               0x00000001L
#define CP_ME_VS_EVENT_SRC__VS_DONE_SWM                    0x00000001L
#define CP_ME_VS_EVENT_SRC__VS_DONE_CNTR_MASK              0x00000002L
#define CP_ME_VS_EVENT_SRC__VS_DONE_CNTR                   0x00000002L

// CP_ME_VS_EVENT_ADDR
#define CP_ME_VS_EVENT_ADDR__VS_DONE_SWAP_MASK             0x00000003L
#define CP_ME_VS_EVENT_ADDR__VS_DONE_ADDR_MASK             0xfffffffcL

// CP_ME_VS_EVENT_DATA
#define CP_ME_VS_EVENT_DATA__VS_DONE_DATA_MASK             0xffffffffL

// CP_ME_VS_EVENT_ADDR_SWM
#define CP_ME_VS_EVENT_ADDR_SWM__VS_DONE_SWAP_SWM_MASK     0x00000003L
#define CP_ME_VS_EVENT_ADDR_SWM__VS_DONE_ADDR_SWM_MASK     0xfffffffcL

// CP_ME_VS_EVENT_DATA_SWM
#define CP_ME_VS_EVENT_DATA_SWM__VS_DONE_DATA_SWM_MASK     0xffffffffL

// CP_ME_PS_EVENT_SRC
#define CP_ME_PS_EVENT_SRC__PS_DONE_SWM_MASK               0x00000001L
#define CP_ME_PS_EVENT_SRC__PS_DONE_SWM                    0x00000001L
#define CP_ME_PS_EVENT_SRC__PS_DONE_CNTR_MASK              0x00000002L
#define CP_ME_PS_EVENT_SRC__PS_DONE_CNTR                   0x00000002L

// CP_ME_PS_EVENT_ADDR
#define CP_ME_PS_EVENT_ADDR__PS_DONE_SWAP_MASK             0x00000003L
#define CP_ME_PS_EVENT_ADDR__PS_DONE_ADDR_MASK             0xfffffffcL

// CP_ME_PS_EVENT_DATA
#define CP_ME_PS_EVENT_DATA__PS_DONE_DATA_MASK             0xffffffffL

// CP_ME_PS_EVENT_ADDR_SWM
#define CP_ME_PS_EVENT_ADDR_SWM__PS_DONE_SWAP_SWM_MASK     0x00000003L
#define CP_ME_PS_EVENT_ADDR_SWM__PS_DONE_ADDR_SWM_MASK     0xfffffffcL

// CP_ME_PS_EVENT_DATA_SWM
#define CP_ME_PS_EVENT_DATA_SWM__PS_DONE_DATA_SWM_MASK     0xffffffffL

// CP_ME_CF_EVENT_SRC
#define CP_ME_CF_EVENT_SRC__CF_DONE_SRC_MASK               0x00000001L
#define CP_ME_CF_EVENT_SRC__CF_DONE_SRC                    0x00000001L

// CP_ME_CF_EVENT_ADDR
#define CP_ME_CF_EVENT_ADDR__CF_DONE_SWAP_MASK             0x00000003L
#define CP_ME_CF_EVENT_ADDR__CF_DONE_ADDR_MASK             0xfffffffcL

// CP_ME_CF_EVENT_DATA
#define CP_ME_CF_EVENT_DATA__CF_DONE_DATA_MASK             0xffffffffL

// CP_ME_NRT_ADDR
#define CP_ME_NRT_ADDR__NRT_WRITE_SWAP_MASK                0x00000003L
#define CP_ME_NRT_ADDR__NRT_WRITE_ADDR_MASK                0xfffffffcL

// CP_ME_NRT_DATA
#define CP_ME_NRT_DATA__NRT_WRITE_DATA_MASK                0xffffffffL

// CP_ME_VS_FETCH_DONE_SRC
#define CP_ME_VS_FETCH_DONE_SRC__VS_FETCH_DONE_CNTR_MASK   0x00000001L
#define CP_ME_VS_FETCH_DONE_SRC__VS_FETCH_DONE_CNTR        0x00000001L

// CP_ME_VS_FETCH_DONE_ADDR
#define CP_ME_VS_FETCH_DONE_ADDR__VS_FETCH_DONE_SWAP_MASK  0x00000003L
#define CP_ME_VS_FETCH_DONE_ADDR__VS_FETCH_DONE_ADDR_MASK  0xfffffffcL

// CP_ME_VS_FETCH_DONE_DATA
#define CP_ME_VS_FETCH_DONE_DATA__VS_FETCH_DONE_DATA_MASK  0xffffffffL

// CP_INT_CNTL
#define CP_INT_CNTL__SW_INT_MASK_MASK                      0x00080000L
#define CP_INT_CNTL__SW_INT_MASK                           0x00080000L
#define CP_INT_CNTL__T0_PACKET_IN_IB_MASK_MASK             0x00800000L
#define CP_INT_CNTL__T0_PACKET_IN_IB_MASK                  0x00800000L
#define CP_INT_CNTL__OPCODE_ERROR_MASK_MASK                0x01000000L
#define CP_INT_CNTL__OPCODE_ERROR_MASK                     0x01000000L
#define CP_INT_CNTL__PROTECTED_MODE_ERROR_MASK_MASK        0x02000000L
#define CP_INT_CNTL__PROTECTED_MODE_ERROR_MASK             0x02000000L
#define CP_INT_CNTL__RESERVED_BIT_ERROR_MASK_MASK          0x04000000L
#define CP_INT_CNTL__RESERVED_BIT_ERROR_MASK               0x04000000L
#define CP_INT_CNTL__IB_ERROR_MASK_MASK                    0x08000000L
#define CP_INT_CNTL__IB_ERROR_MASK                         0x08000000L
#define CP_INT_CNTL__IB2_INT_MASK_MASK                     0x20000000L
#define CP_INT_CNTL__IB2_INT_MASK                          0x20000000L
#define CP_INT_CNTL__IB1_INT_MASK_MASK                     0x40000000L
#define CP_INT_CNTL__IB1_INT_MASK                          0x40000000L
#define CP_INT_CNTL__RB_INT_MASK_MASK                      0x80000000L
#define CP_INT_CNTL__RB_INT_MASK                           0x80000000L

// CP_INT_STATUS
#define CP_INT_STATUS__SW_INT_STAT_MASK                    0x00080000L
#define CP_INT_STATUS__SW_INT_STAT                         0x00080000L
#define CP_INT_STATUS__T0_PACKET_IN_IB_STAT_MASK           0x00800000L
#define CP_INT_STATUS__T0_PACKET_IN_IB_STAT                0x00800000L
#define CP_INT_STATUS__OPCODE_ERROR_STAT_MASK              0x01000000L
#define CP_INT_STATUS__OPCODE_ERROR_STAT                   0x01000000L
#define CP_INT_STATUS__PROTECTED_MODE_ERROR_STAT_MASK      0x02000000L
#define CP_INT_STATUS__PROTECTED_MODE_ERROR_STAT           0x02000000L
#define CP_INT_STATUS__RESERVED_BIT_ERROR_STAT_MASK        0x04000000L
#define CP_INT_STATUS__RESERVED_BIT_ERROR_STAT             0x04000000L
#define CP_INT_STATUS__IB_ERROR_STAT_MASK                  0x08000000L
#define CP_INT_STATUS__IB_ERROR_STAT                       0x08000000L
#define CP_INT_STATUS__IB2_INT_STAT_MASK                   0x20000000L
#define CP_INT_STATUS__IB2_INT_STAT                        0x20000000L
#define CP_INT_STATUS__IB1_INT_STAT_MASK                   0x40000000L
#define CP_INT_STATUS__IB1_INT_STAT                        0x40000000L
#define CP_INT_STATUS__RB_INT_STAT_MASK                    0x80000000L
#define CP_INT_STATUS__RB_INT_STAT                         0x80000000L

// CP_INT_ACK
#define CP_INT_ACK__SW_INT_ACK_MASK                        0x00080000L
#define CP_INT_ACK__SW_INT_ACK                             0x00080000L
#define CP_INT_ACK__T0_PACKET_IN_IB_ACK_MASK               0x00800000L
#define CP_INT_ACK__T0_PACKET_IN_IB_ACK                    0x00800000L
#define CP_INT_ACK__OPCODE_ERROR_ACK_MASK                  0x01000000L
#define CP_INT_ACK__OPCODE_ERROR_ACK                       0x01000000L
#define CP_INT_ACK__PROTECTED_MODE_ERROR_ACK_MASK          0x02000000L
#define CP_INT_ACK__PROTECTED_MODE_ERROR_ACK               0x02000000L
#define CP_INT_ACK__RESERVED_BIT_ERROR_ACK_MASK            0x04000000L
#define CP_INT_ACK__RESERVED_BIT_ERROR_ACK                 0x04000000L
#define CP_INT_ACK__IB_ERROR_ACK_MASK                      0x08000000L
#define CP_INT_ACK__IB_ERROR_ACK                           0x08000000L
#define CP_INT_ACK__IB2_INT_ACK_MASK                       0x20000000L
#define CP_INT_ACK__IB2_INT_ACK                            0x20000000L
#define CP_INT_ACK__IB1_INT_ACK_MASK                       0x40000000L
#define CP_INT_ACK__IB1_INT_ACK                            0x40000000L
#define CP_INT_ACK__RB_INT_ACK_MASK                        0x80000000L
#define CP_INT_ACK__RB_INT_ACK                             0x80000000L

// CP_PFP_UCODE_ADDR
#define CP_PFP_UCODE_ADDR__UCODE_ADDR_MASK                 0x000001ffL

// CP_PFP_UCODE_DATA
#define CP_PFP_UCODE_DATA__UCODE_DATA_MASK                 0x00ffffffL

// CP_PERFMON_CNTL
#define CP_PERFMON_CNTL__PERFMON_STATE_MASK                0x0000000fL
#define CP_PERFMON_CNTL__PERFMON_ENABLE_MODE_MASK          0x00000300L

// CP_PERFCOUNTER_SELECT
#define CP_PERFCOUNTER_SELECT__PERFCOUNT_SEL_MASK          0x0000003fL

// CP_PERFCOUNTER_LO
#define CP_PERFCOUNTER_LO__PERFCOUNT_LO_MASK               0xffffffffL

// CP_PERFCOUNTER_HI
#define CP_PERFCOUNTER_HI__PERFCOUNT_HI_MASK               0x0000ffffL

// CP_BIN_MASK_LO
#define CP_BIN_MASK_LO__BIN_MASK_LO_MASK                   0xffffffffL

// CP_BIN_MASK_HI
#define CP_BIN_MASK_HI__BIN_MASK_HI_MASK                   0xffffffffL

// CP_BIN_SELECT_LO
#define CP_BIN_SELECT_LO__BIN_SELECT_LO_MASK               0xffffffffL

// CP_BIN_SELECT_HI
#define CP_BIN_SELECT_HI__BIN_SELECT_HI_MASK               0xffffffffL

// CP_NV_FLAGS_0
#define CP_NV_FLAGS_0__DISCARD_0_MASK                      0x00000001L
#define CP_NV_FLAGS_0__DISCARD_0                           0x00000001L
#define CP_NV_FLAGS_0__END_RCVD_0_MASK                     0x00000002L
#define CP_NV_FLAGS_0__END_RCVD_0                          0x00000002L
#define CP_NV_FLAGS_0__DISCARD_1_MASK                      0x00000004L
#define CP_NV_FLAGS_0__DISCARD_1                           0x00000004L
#define CP_NV_FLAGS_0__END_RCVD_1_MASK                     0x00000008L
#define CP_NV_FLAGS_0__END_RCVD_1                          0x00000008L
#define CP_NV_FLAGS_0__DISCARD_2_MASK                      0x00000010L
#define CP_NV_FLAGS_0__DISCARD_2                           0x00000010L
#define CP_NV_FLAGS_0__END_RCVD_2_MASK                     0x00000020L
#define CP_NV_FLAGS_0__END_RCVD_2                          0x00000020L
#define CP_NV_FLAGS_0__DISCARD_3_MASK                      0x00000040L
#define CP_NV_FLAGS_0__DISCARD_3                           0x00000040L
#define CP_NV_FLAGS_0__END_RCVD_3_MASK                     0x00000080L
#define CP_NV_FLAGS_0__END_RCVD_3                          0x00000080L
#define CP_NV_FLAGS_0__DISCARD_4_MASK                      0x00000100L
#define CP_NV_FLAGS_0__DISCARD_4                           0x00000100L
#define CP_NV_FLAGS_0__END_RCVD_4_MASK                     0x00000200L
#define CP_NV_FLAGS_0__END_RCVD_4                          0x00000200L
#define CP_NV_FLAGS_0__DISCARD_5_MASK                      0x00000400L
#define CP_NV_FLAGS_0__DISCARD_5                           0x00000400L
#define CP_NV_FLAGS_0__END_RCVD_5_MASK                     0x00000800L
#define CP_NV_FLAGS_0__END_RCVD_5                          0x00000800L
#define CP_NV_FLAGS_0__DISCARD_6_MASK                      0x00001000L
#define CP_NV_FLAGS_0__DISCARD_6                           0x00001000L
#define CP_NV_FLAGS_0__END_RCVD_6_MASK                     0x00002000L
#define CP_NV_FLAGS_0__END_RCVD_6                          0x00002000L
#define CP_NV_FLAGS_0__DISCARD_7_MASK                      0x00004000L
#define CP_NV_FLAGS_0__DISCARD_7                           0x00004000L
#define CP_NV_FLAGS_0__END_RCVD_7_MASK                     0x00008000L
#define CP_NV_FLAGS_0__END_RCVD_7                          0x00008000L
#define CP_NV_FLAGS_0__DISCARD_8_MASK                      0x00010000L
#define CP_NV_FLAGS_0__DISCARD_8                           0x00010000L
#define CP_NV_FLAGS_0__END_RCVD_8_MASK                     0x00020000L
#define CP_NV_FLAGS_0__END_RCVD_8                          0x00020000L
#define CP_NV_FLAGS_0__DISCARD_9_MASK                      0x00040000L
#define CP_NV_FLAGS_0__DISCARD_9                           0x00040000L
#define CP_NV_FLAGS_0__END_RCVD_9_MASK                     0x00080000L
#define CP_NV_FLAGS_0__END_RCVD_9                          0x00080000L
#define CP_NV_FLAGS_0__DISCARD_10_MASK                     0x00100000L
#define CP_NV_FLAGS_0__DISCARD_10                          0x00100000L
#define CP_NV_FLAGS_0__END_RCVD_10_MASK                    0x00200000L
#define CP_NV_FLAGS_0__END_RCVD_10                         0x00200000L
#define CP_NV_FLAGS_0__DISCARD_11_MASK                     0x00400000L
#define CP_NV_FLAGS_0__DISCARD_11                          0x00400000L
#define CP_NV_FLAGS_0__END_RCVD_11_MASK                    0x00800000L
#define CP_NV_FLAGS_0__END_RCVD_11                         0x00800000L
#define CP_NV_FLAGS_0__DISCARD_12_MASK                     0x01000000L
#define CP_NV_FLAGS_0__DISCARD_12                          0x01000000L
#define CP_NV_FLAGS_0__END_RCVD_12_MASK                    0x02000000L
#define CP_NV_FLAGS_0__END_RCVD_12                         0x02000000L
#define CP_NV_FLAGS_0__DISCARD_13_MASK                     0x04000000L
#define CP_NV_FLAGS_0__DISCARD_13                          0x04000000L
#define CP_NV_FLAGS_0__END_RCVD_13_MASK                    0x08000000L
#define CP_NV_FLAGS_0__END_RCVD_13                         0x08000000L
#define CP_NV_FLAGS_0__DISCARD_14_MASK                     0x10000000L
#define CP_NV_FLAGS_0__DISCARD_14                          0x10000000L
#define CP_NV_FLAGS_0__END_RCVD_14_MASK                    0x20000000L
#define CP_NV_FLAGS_0__END_RCVD_14                         0x20000000L
#define CP_NV_FLAGS_0__DISCARD_15_MASK                     0x40000000L
#define CP_NV_FLAGS_0__DISCARD_15                          0x40000000L
#define CP_NV_FLAGS_0__END_RCVD_15_MASK                    0x80000000L
#define CP_NV_FLAGS_0__END_RCVD_15                         0x80000000L

// CP_NV_FLAGS_1
#define CP_NV_FLAGS_1__DISCARD_16_MASK                     0x00000001L
#define CP_NV_FLAGS_1__DISCARD_16                          0x00000001L
#define CP_NV_FLAGS_1__END_RCVD_16_MASK                    0x00000002L
#define CP_NV_FLAGS_1__END_RCVD_16                         0x00000002L
#define CP_NV_FLAGS_1__DISCARD_17_MASK                     0x00000004L
#define CP_NV_FLAGS_1__DISCARD_17                          0x00000004L
#define CP_NV_FLAGS_1__END_RCVD_17_MASK                    0x00000008L
#define CP_NV_FLAGS_1__END_RCVD_17                         0x00000008L
#define CP_NV_FLAGS_1__DISCARD_18_MASK                     0x00000010L
#define CP_NV_FLAGS_1__DISCARD_18                          0x00000010L
#define CP_NV_FLAGS_1__END_RCVD_18_MASK                    0x00000020L
#define CP_NV_FLAGS_1__END_RCVD_18                         0x00000020L
#define CP_NV_FLAGS_1__DISCARD_19_MASK                     0x00000040L
#define CP_NV_FLAGS_1__DISCARD_19                          0x00000040L
#define CP_NV_FLAGS_1__END_RCVD_19_MASK                    0x00000080L
#define CP_NV_FLAGS_1__END_RCVD_19                         0x00000080L
#define CP_NV_FLAGS_1__DISCARD_20_MASK                     0x00000100L
#define CP_NV_FLAGS_1__DISCARD_20                          0x00000100L
#define CP_NV_FLAGS_1__END_RCVD_20_MASK                    0x00000200L
#define CP_NV_FLAGS_1__END_RCVD_20                         0x00000200L
#define CP_NV_FLAGS_1__DISCARD_21_MASK                     0x00000400L
#define CP_NV_FLAGS_1__DISCARD_21                          0x00000400L
#define CP_NV_FLAGS_1__END_RCVD_21_MASK                    0x00000800L
#define CP_NV_FLAGS_1__END_RCVD_21                         0x00000800L
#define CP_NV_FLAGS_1__DISCARD_22_MASK                     0x00001000L
#define CP_NV_FLAGS_1__DISCARD_22                          0x00001000L
#define CP_NV_FLAGS_1__END_RCVD_22_MASK                    0x00002000L
#define CP_NV_FLAGS_1__END_RCVD_22                         0x00002000L
#define CP_NV_FLAGS_1__DISCARD_23_MASK                     0x00004000L
#define CP_NV_FLAGS_1__DISCARD_23                          0x00004000L
#define CP_NV_FLAGS_1__END_RCVD_23_MASK                    0x00008000L
#define CP_NV_FLAGS_1__END_RCVD_23                         0x00008000L
#define CP_NV_FLAGS_1__DISCARD_24_MASK                     0x00010000L
#define CP_NV_FLAGS_1__DISCARD_24                          0x00010000L
#define CP_NV_FLAGS_1__END_RCVD_24_MASK                    0x00020000L
#define CP_NV_FLAGS_1__END_RCVD_24                         0x00020000L
#define CP_NV_FLAGS_1__DISCARD_25_MASK                     0x00040000L
#define CP_NV_FLAGS_1__DISCARD_25                          0x00040000L
#define CP_NV_FLAGS_1__END_RCVD_25_MASK                    0x00080000L
#define CP_NV_FLAGS_1__END_RCVD_25                         0x00080000L
#define CP_NV_FLAGS_1__DISCARD_26_MASK                     0x00100000L
#define CP_NV_FLAGS_1__DISCARD_26                          0x00100000L
#define CP_NV_FLAGS_1__END_RCVD_26_MASK                    0x00200000L
#define CP_NV_FLAGS_1__END_RCVD_26                         0x00200000L
#define CP_NV_FLAGS_1__DISCARD_27_MASK                     0x00400000L
#define CP_NV_FLAGS_1__DISCARD_27                          0x00400000L
#define CP_NV_FLAGS_1__END_RCVD_27_MASK                    0x00800000L
#define CP_NV_FLAGS_1__END_RCVD_27                         0x00800000L
#define CP_NV_FLAGS_1__DISCARD_28_MASK                     0x01000000L
#define CP_NV_FLAGS_1__DISCARD_28                          0x01000000L
#define CP_NV_FLAGS_1__END_RCVD_28_MASK                    0x02000000L
#define CP_NV_FLAGS_1__END_RCVD_28                         0x02000000L
#define CP_NV_FLAGS_1__DISCARD_29_MASK                     0x04000000L
#define CP_NV_FLAGS_1__DISCARD_29                          0x04000000L
#define CP_NV_FLAGS_1__END_RCVD_29_MASK                    0x08000000L
#define CP_NV_FLAGS_1__END_RCVD_29                         0x08000000L
#define CP_NV_FLAGS_1__DISCARD_30_MASK                     0x10000000L
#define CP_NV_FLAGS_1__DISCARD_30                          0x10000000L
#define CP_NV_FLAGS_1__END_RCVD_30_MASK                    0x20000000L
#define CP_NV_FLAGS_1__END_RCVD_30                         0x20000000L
#define CP_NV_FLAGS_1__DISCARD_31_MASK                     0x40000000L
#define CP_NV_FLAGS_1__DISCARD_31                          0x40000000L
#define CP_NV_FLAGS_1__END_RCVD_31_MASK                    0x80000000L
#define CP_NV_FLAGS_1__END_RCVD_31                         0x80000000L

// CP_NV_FLAGS_2
#define CP_NV_FLAGS_2__DISCARD_32_MASK                     0x00000001L
#define CP_NV_FLAGS_2__DISCARD_32                          0x00000001L
#define CP_NV_FLAGS_2__END_RCVD_32_MASK                    0x00000002L
#define CP_NV_FLAGS_2__END_RCVD_32                         0x00000002L
#define CP_NV_FLAGS_2__DISCARD_33_MASK                     0x00000004L
#define CP_NV_FLAGS_2__DISCARD_33                          0x00000004L
#define CP_NV_FLAGS_2__END_RCVD_33_MASK                    0x00000008L
#define CP_NV_FLAGS_2__END_RCVD_33                         0x00000008L
#define CP_NV_FLAGS_2__DISCARD_34_MASK                     0x00000010L
#define CP_NV_FLAGS_2__DISCARD_34                          0x00000010L
#define CP_NV_FLAGS_2__END_RCVD_34_MASK                    0x00000020L
#define CP_NV_FLAGS_2__END_RCVD_34                         0x00000020L
#define CP_NV_FLAGS_2__DISCARD_35_MASK                     0x00000040L
#define CP_NV_FLAGS_2__DISCARD_35                          0x00000040L
#define CP_NV_FLAGS_2__END_RCVD_35_MASK                    0x00000080L
#define CP_NV_FLAGS_2__END_RCVD_35                         0x00000080L
#define CP_NV_FLAGS_2__DISCARD_36_MASK                     0x00000100L
#define CP_NV_FLAGS_2__DISCARD_36                          0x00000100L
#define CP_NV_FLAGS_2__END_RCVD_36_MASK                    0x00000200L
#define CP_NV_FLAGS_2__END_RCVD_36                         0x00000200L
#define CP_NV_FLAGS_2__DISCARD_37_MASK                     0x00000400L
#define CP_NV_FLAGS_2__DISCARD_37                          0x00000400L
#define CP_NV_FLAGS_2__END_RCVD_37_MASK                    0x00000800L
#define CP_NV_FLAGS_2__END_RCVD_37                         0x00000800L
#define CP_NV_FLAGS_2__DISCARD_38_MASK                     0x00001000L
#define CP_NV_FLAGS_2__DISCARD_38                          0x00001000L
#define CP_NV_FLAGS_2__END_RCVD_38_MASK                    0x00002000L
#define CP_NV_FLAGS_2__END_RCVD_38                         0x00002000L
#define CP_NV_FLAGS_2__DISCARD_39_MASK                     0x00004000L
#define CP_NV_FLAGS_2__DISCARD_39                          0x00004000L
#define CP_NV_FLAGS_2__END_RCVD_39_MASK                    0x00008000L
#define CP_NV_FLAGS_2__END_RCVD_39                         0x00008000L
#define CP_NV_FLAGS_2__DISCARD_40_MASK                     0x00010000L
#define CP_NV_FLAGS_2__DISCARD_40                          0x00010000L
#define CP_NV_FLAGS_2__END_RCVD_40_MASK                    0x00020000L
#define CP_NV_FLAGS_2__END_RCVD_40                         0x00020000L
#define CP_NV_FLAGS_2__DISCARD_41_MASK                     0x00040000L
#define CP_NV_FLAGS_2__DISCARD_41                          0x00040000L
#define CP_NV_FLAGS_2__END_RCVD_41_MASK                    0x00080000L
#define CP_NV_FLAGS_2__END_RCVD_41                         0x00080000L
#define CP_NV_FLAGS_2__DISCARD_42_MASK                     0x00100000L
#define CP_NV_FLAGS_2__DISCARD_42                          0x00100000L
#define CP_NV_FLAGS_2__END_RCVD_42_MASK                    0x00200000L
#define CP_NV_FLAGS_2__END_RCVD_42                         0x00200000L
#define CP_NV_FLAGS_2__DISCARD_43_MASK                     0x00400000L
#define CP_NV_FLAGS_2__DISCARD_43                          0x00400000L
#define CP_NV_FLAGS_2__END_RCVD_43_MASK                    0x00800000L
#define CP_NV_FLAGS_2__END_RCVD_43                         0x00800000L
#define CP_NV_FLAGS_2__DISCARD_44_MASK                     0x01000000L
#define CP_NV_FLAGS_2__DISCARD_44                          0x01000000L
#define CP_NV_FLAGS_2__END_RCVD_44_MASK                    0x02000000L
#define CP_NV_FLAGS_2__END_RCVD_44                         0x02000000L
#define CP_NV_FLAGS_2__DISCARD_45_MASK                     0x04000000L
#define CP_NV_FLAGS_2__DISCARD_45                          0x04000000L
#define CP_NV_FLAGS_2__END_RCVD_45_MASK                    0x08000000L
#define CP_NV_FLAGS_2__END_RCVD_45                         0x08000000L
#define CP_NV_FLAGS_2__DISCARD_46_MASK                     0x10000000L
#define CP_NV_FLAGS_2__DISCARD_46                          0x10000000L
#define CP_NV_FLAGS_2__END_RCVD_46_MASK                    0x20000000L
#define CP_NV_FLAGS_2__END_RCVD_46                         0x20000000L
#define CP_NV_FLAGS_2__DISCARD_47_MASK                     0x40000000L
#define CP_NV_FLAGS_2__DISCARD_47                          0x40000000L
#define CP_NV_FLAGS_2__END_RCVD_47_MASK                    0x80000000L
#define CP_NV_FLAGS_2__END_RCVD_47                         0x80000000L

// CP_NV_FLAGS_3
#define CP_NV_FLAGS_3__DISCARD_48_MASK                     0x00000001L
#define CP_NV_FLAGS_3__DISCARD_48                          0x00000001L
#define CP_NV_FLAGS_3__END_RCVD_48_MASK                    0x00000002L
#define CP_NV_FLAGS_3__END_RCVD_48                         0x00000002L
#define CP_NV_FLAGS_3__DISCARD_49_MASK                     0x00000004L
#define CP_NV_FLAGS_3__DISCARD_49                          0x00000004L
#define CP_NV_FLAGS_3__END_RCVD_49_MASK                    0x00000008L
#define CP_NV_FLAGS_3__END_RCVD_49                         0x00000008L
#define CP_NV_FLAGS_3__DISCARD_50_MASK                     0x00000010L
#define CP_NV_FLAGS_3__DISCARD_50                          0x00000010L
#define CP_NV_FLAGS_3__END_RCVD_50_MASK                    0x00000020L
#define CP_NV_FLAGS_3__END_RCVD_50                         0x00000020L
#define CP_NV_FLAGS_3__DISCARD_51_MASK                     0x00000040L
#define CP_NV_FLAGS_3__DISCARD_51                          0x00000040L
#define CP_NV_FLAGS_3__END_RCVD_51_MASK                    0x00000080L
#define CP_NV_FLAGS_3__END_RCVD_51                         0x00000080L
#define CP_NV_FLAGS_3__DISCARD_52_MASK                     0x00000100L
#define CP_NV_FLAGS_3__DISCARD_52                          0x00000100L
#define CP_NV_FLAGS_3__END_RCVD_52_MASK                    0x00000200L
#define CP_NV_FLAGS_3__END_RCVD_52                         0x00000200L
#define CP_NV_FLAGS_3__DISCARD_53_MASK                     0x00000400L
#define CP_NV_FLAGS_3__DISCARD_53                          0x00000400L
#define CP_NV_FLAGS_3__END_RCVD_53_MASK                    0x00000800L
#define CP_NV_FLAGS_3__END_RCVD_53                         0x00000800L
#define CP_NV_FLAGS_3__DISCARD_54_MASK                     0x00001000L
#define CP_NV_FLAGS_3__DISCARD_54                          0x00001000L
#define CP_NV_FLAGS_3__END_RCVD_54_MASK                    0x00002000L
#define CP_NV_FLAGS_3__END_RCVD_54                         0x00002000L
#define CP_NV_FLAGS_3__DISCARD_55_MASK                     0x00004000L
#define CP_NV_FLAGS_3__DISCARD_55                          0x00004000L
#define CP_NV_FLAGS_3__END_RCVD_55_MASK                    0x00008000L
#define CP_NV_FLAGS_3__END_RCVD_55                         0x00008000L
#define CP_NV_FLAGS_3__DISCARD_56_MASK                     0x00010000L
#define CP_NV_FLAGS_3__DISCARD_56                          0x00010000L
#define CP_NV_FLAGS_3__END_RCVD_56_MASK                    0x00020000L
#define CP_NV_FLAGS_3__END_RCVD_56                         0x00020000L
#define CP_NV_FLAGS_3__DISCARD_57_MASK                     0x00040000L
#define CP_NV_FLAGS_3__DISCARD_57                          0x00040000L
#define CP_NV_FLAGS_3__END_RCVD_57_MASK                    0x00080000L
#define CP_NV_FLAGS_3__END_RCVD_57                         0x00080000L
#define CP_NV_FLAGS_3__DISCARD_58_MASK                     0x00100000L
#define CP_NV_FLAGS_3__DISCARD_58                          0x00100000L
#define CP_NV_FLAGS_3__END_RCVD_58_MASK                    0x00200000L
#define CP_NV_FLAGS_3__END_RCVD_58                         0x00200000L
#define CP_NV_FLAGS_3__DISCARD_59_MASK                     0x00400000L
#define CP_NV_FLAGS_3__DISCARD_59                          0x00400000L
#define CP_NV_FLAGS_3__END_RCVD_59_MASK                    0x00800000L
#define CP_NV_FLAGS_3__END_RCVD_59                         0x00800000L
#define CP_NV_FLAGS_3__DISCARD_60_MASK                     0x01000000L
#define CP_NV_FLAGS_3__DISCARD_60                          0x01000000L
#define CP_NV_FLAGS_3__END_RCVD_60_MASK                    0x02000000L
#define CP_NV_FLAGS_3__END_RCVD_60                         0x02000000L
#define CP_NV_FLAGS_3__DISCARD_61_MASK                     0x04000000L
#define CP_NV_FLAGS_3__DISCARD_61                          0x04000000L
#define CP_NV_FLAGS_3__END_RCVD_61_MASK                    0x08000000L
#define CP_NV_FLAGS_3__END_RCVD_61                         0x08000000L
#define CP_NV_FLAGS_3__DISCARD_62_MASK                     0x10000000L
#define CP_NV_FLAGS_3__DISCARD_62                          0x10000000L
#define CP_NV_FLAGS_3__END_RCVD_62_MASK                    0x20000000L
#define CP_NV_FLAGS_3__END_RCVD_62                         0x20000000L
#define CP_NV_FLAGS_3__DISCARD_63_MASK                     0x40000000L
#define CP_NV_FLAGS_3__DISCARD_63                          0x40000000L
#define CP_NV_FLAGS_3__END_RCVD_63_MASK                    0x80000000L
#define CP_NV_FLAGS_3__END_RCVD_63                         0x80000000L

// CP_STATE_DEBUG_INDEX
#define CP_STATE_DEBUG_INDEX__STATE_DEBUG_INDEX_MASK       0x0000001fL

// CP_STATE_DEBUG_DATA
#define CP_STATE_DEBUG_DATA__STATE_DEBUG_DATA_MASK         0xffffffffL

// CP_PROG_COUNTER
#define CP_PROG_COUNTER__COUNTER_MASK                      0xffffffffL

// CP_STAT
#define CP_STAT__MIU_WR_BUSY_MASK                          0x00000001L
#define CP_STAT__MIU_WR_BUSY                               0x00000001L
#define CP_STAT__MIU_RD_REQ_BUSY_MASK                      0x00000002L
#define CP_STAT__MIU_RD_REQ_BUSY                           0x00000002L
#define CP_STAT__MIU_RD_RETURN_BUSY_MASK                   0x00000004L
#define CP_STAT__MIU_RD_RETURN_BUSY                        0x00000004L
#define CP_STAT__RBIU_BUSY_MASK                            0x00000008L
#define CP_STAT__RBIU_BUSY                                 0x00000008L
#define CP_STAT__RCIU_BUSY_MASK                            0x00000010L
#define CP_STAT__RCIU_BUSY                                 0x00000010L
#define CP_STAT__CSF_RING_BUSY_MASK                        0x00000020L
#define CP_STAT__CSF_RING_BUSY                             0x00000020L
#define CP_STAT__CSF_INDIRECTS_BUSY_MASK                   0x00000040L
#define CP_STAT__CSF_INDIRECTS_BUSY                        0x00000040L
#define CP_STAT__CSF_INDIRECT2_BUSY_MASK                   0x00000080L
#define CP_STAT__CSF_INDIRECT2_BUSY                        0x00000080L
#define CP_STAT__CSF_ST_BUSY_MASK                          0x00000200L
#define CP_STAT__CSF_ST_BUSY                               0x00000200L
#define CP_STAT__CSF_BUSY_MASK                             0x00000400L
#define CP_STAT__CSF_BUSY                                  0x00000400L
#define CP_STAT__RING_QUEUE_BUSY_MASK                      0x00000800L
#define CP_STAT__RING_QUEUE_BUSY                           0x00000800L
#define CP_STAT__INDIRECTS_QUEUE_BUSY_MASK                 0x00001000L
#define CP_STAT__INDIRECTS_QUEUE_BUSY                      0x00001000L
#define CP_STAT__INDIRECT2_QUEUE_BUSY_MASK                 0x00002000L
#define CP_STAT__INDIRECT2_QUEUE_BUSY                      0x00002000L
#define CP_STAT__ST_QUEUE_BUSY_MASK                        0x00010000L
#define CP_STAT__ST_QUEUE_BUSY                             0x00010000L
#define CP_STAT__PFP_BUSY_MASK                             0x00020000L
#define CP_STAT__PFP_BUSY                                  0x00020000L
#define CP_STAT__MEQ_RING_BUSY_MASK                        0x00040000L
#define CP_STAT__MEQ_RING_BUSY                             0x00040000L
#define CP_STAT__MEQ_INDIRECTS_BUSY_MASK                   0x00080000L
#define CP_STAT__MEQ_INDIRECTS_BUSY                        0x00080000L
#define CP_STAT__MEQ_INDIRECT2_BUSY_MASK                   0x00100000L
#define CP_STAT__MEQ_INDIRECT2_BUSY                        0x00100000L
#define CP_STAT__MIU_WC_STALL_MASK                         0x00200000L
#define CP_STAT__MIU_WC_STALL                              0x00200000L
#define CP_STAT__CP_NRT_BUSY_MASK                          0x00400000L
#define CP_STAT__CP_NRT_BUSY                               0x00400000L
#define CP_STAT___3D_BUSY_MASK                             0x00800000L
#define CP_STAT___3D_BUSY                                  0x00800000L
#define CP_STAT__ME_BUSY_MASK                              0x04000000L
#define CP_STAT__ME_BUSY                                   0x04000000L
#define CP_STAT__ME_WC_BUSY_MASK                           0x20000000L
#define CP_STAT__ME_WC_BUSY                                0x20000000L
#define CP_STAT__MIU_WC_TRACK_FIFO_EMPTY_MASK              0x40000000L
#define CP_STAT__MIU_WC_TRACK_FIFO_EMPTY                   0x40000000L
#define CP_STAT__CP_BUSY_MASK                              0x80000000L
#define CP_STAT__CP_BUSY                                   0x80000000L

// BIOS_0_SCRATCH
#define BIOS_0_SCRATCH__BIOS_SCRATCH_MASK                  0xffffffffL

// BIOS_1_SCRATCH
#define BIOS_1_SCRATCH__BIOS_SCRATCH_MASK                  0xffffffffL

// BIOS_2_SCRATCH
#define BIOS_2_SCRATCH__BIOS_SCRATCH_MASK                  0xffffffffL

// BIOS_3_SCRATCH
#define BIOS_3_SCRATCH__BIOS_SCRATCH_MASK                  0xffffffffL

// BIOS_4_SCRATCH
#define BIOS_4_SCRATCH__BIOS_SCRATCH_MASK                  0xffffffffL

// BIOS_5_SCRATCH
#define BIOS_5_SCRATCH__BIOS_SCRATCH_MASK                  0xffffffffL

// BIOS_6_SCRATCH
#define BIOS_6_SCRATCH__BIOS_SCRATCH_MASK                  0xffffffffL

// BIOS_7_SCRATCH
#define BIOS_7_SCRATCH__BIOS_SCRATCH_MASK                  0xffffffffL

// BIOS_8_SCRATCH
#define BIOS_8_SCRATCH__BIOS_SCRATCH_MASK                  0xffffffffL

// BIOS_9_SCRATCH
#define BIOS_9_SCRATCH__BIOS_SCRATCH_MASK                  0xffffffffL

// BIOS_10_SCRATCH
#define BIOS_10_SCRATCH__BIOS_SCRATCH_MASK                 0xffffffffL

// BIOS_11_SCRATCH
#define BIOS_11_SCRATCH__BIOS_SCRATCH_MASK                 0xffffffffL

// BIOS_12_SCRATCH
#define BIOS_12_SCRATCH__BIOS_SCRATCH_MASK                 0xffffffffL

// BIOS_13_SCRATCH
#define BIOS_13_SCRATCH__BIOS_SCRATCH_MASK                 0xffffffffL

// BIOS_14_SCRATCH
#define BIOS_14_SCRATCH__BIOS_SCRATCH_MASK                 0xffffffffL

// BIOS_15_SCRATCH
#define BIOS_15_SCRATCH__BIOS_SCRATCH_MASK                 0xffffffffL

// COHER_SIZE_PM4
#define COHER_SIZE_PM4__SIZE_MASK                          0xffffffffL

// COHER_BASE_PM4
#define COHER_BASE_PM4__BASE_MASK                          0xffffffffL

// COHER_STATUS_PM4
#define COHER_STATUS_PM4__MATCHING_CONTEXTS_MASK           0x000000ffL
#define COHER_STATUS_PM4__RB_COPY_DEST_BASE_ENA_MASK       0x00000100L
#define COHER_STATUS_PM4__RB_COPY_DEST_BASE_ENA            0x00000100L
#define COHER_STATUS_PM4__DEST_BASE_0_ENA_MASK             0x00000200L
#define COHER_STATUS_PM4__DEST_BASE_0_ENA                  0x00000200L
#define COHER_STATUS_PM4__DEST_BASE_1_ENA_MASK             0x00000400L
#define COHER_STATUS_PM4__DEST_BASE_1_ENA                  0x00000400L
#define COHER_STATUS_PM4__DEST_BASE_2_ENA_MASK             0x00000800L
#define COHER_STATUS_PM4__DEST_BASE_2_ENA                  0x00000800L
#define COHER_STATUS_PM4__DEST_BASE_3_ENA_MASK             0x00001000L
#define COHER_STATUS_PM4__DEST_BASE_3_ENA                  0x00001000L
#define COHER_STATUS_PM4__DEST_BASE_4_ENA_MASK             0x00002000L
#define COHER_STATUS_PM4__DEST_BASE_4_ENA                  0x00002000L
#define COHER_STATUS_PM4__DEST_BASE_5_ENA_MASK             0x00004000L
#define COHER_STATUS_PM4__DEST_BASE_5_ENA                  0x00004000L
#define COHER_STATUS_PM4__DEST_BASE_6_ENA_MASK             0x00008000L
#define COHER_STATUS_PM4__DEST_BASE_6_ENA                  0x00008000L
#define COHER_STATUS_PM4__DEST_BASE_7_ENA_MASK             0x00010000L
#define COHER_STATUS_PM4__DEST_BASE_7_ENA                  0x00010000L
#define COHER_STATUS_PM4__TC_ACTION_ENA_MASK               0x02000000L
#define COHER_STATUS_PM4__TC_ACTION_ENA                    0x02000000L
#define COHER_STATUS_PM4__STATUS_MASK                      0x80000000L
#define COHER_STATUS_PM4__STATUS                           0x80000000L

// COHER_SIZE_HOST
#define COHER_SIZE_HOST__SIZE_MASK                         0xffffffffL

// COHER_BASE_HOST
#define COHER_BASE_HOST__BASE_MASK                         0xffffffffL

// COHER_STATUS_HOST
#define COHER_STATUS_HOST__MATCHING_CONTEXTS_MASK          0x000000ffL
#define COHER_STATUS_HOST__RB_COPY_DEST_BASE_ENA_MASK      0x00000100L
#define COHER_STATUS_HOST__RB_COPY_DEST_BASE_ENA           0x00000100L
#define COHER_STATUS_HOST__DEST_BASE_0_ENA_MASK            0x00000200L
#define COHER_STATUS_HOST__DEST_BASE_0_ENA                 0x00000200L
#define COHER_STATUS_HOST__DEST_BASE_1_ENA_MASK            0x00000400L
#define COHER_STATUS_HOST__DEST_BASE_1_ENA                 0x00000400L
#define COHER_STATUS_HOST__DEST_BASE_2_ENA_MASK            0x00000800L
#define COHER_STATUS_HOST__DEST_BASE_2_ENA                 0x00000800L
#define COHER_STATUS_HOST__DEST_BASE_3_ENA_MASK            0x00001000L
#define COHER_STATUS_HOST__DEST_BASE_3_ENA                 0x00001000L
#define COHER_STATUS_HOST__DEST_BASE_4_ENA_MASK            0x00002000L
#define COHER_STATUS_HOST__DEST_BASE_4_ENA                 0x00002000L
#define COHER_STATUS_HOST__DEST_BASE_5_ENA_MASK            0x00004000L
#define COHER_STATUS_HOST__DEST_BASE_5_ENA                 0x00004000L
#define COHER_STATUS_HOST__DEST_BASE_6_ENA_MASK            0x00008000L
#define COHER_STATUS_HOST__DEST_BASE_6_ENA                 0x00008000L
#define COHER_STATUS_HOST__DEST_BASE_7_ENA_MASK            0x00010000L
#define COHER_STATUS_HOST__DEST_BASE_7_ENA                 0x00010000L
#define COHER_STATUS_HOST__TC_ACTION_ENA_MASK              0x02000000L
#define COHER_STATUS_HOST__TC_ACTION_ENA                   0x02000000L
#define COHER_STATUS_HOST__STATUS_MASK                     0x80000000L
#define COHER_STATUS_HOST__STATUS                          0x80000000L

// COHER_DEST_BASE_0
#define COHER_DEST_BASE_0__DEST_BASE_0_MASK                0xfffff000L

// COHER_DEST_BASE_1
#define COHER_DEST_BASE_1__DEST_BASE_1_MASK                0xfffff000L

// COHER_DEST_BASE_2
#define COHER_DEST_BASE_2__DEST_BASE_2_MASK                0xfffff000L

// COHER_DEST_BASE_3
#define COHER_DEST_BASE_3__DEST_BASE_3_MASK                0xfffff000L

// COHER_DEST_BASE_4
#define COHER_DEST_BASE_4__DEST_BASE_4_MASK                0xfffff000L

// COHER_DEST_BASE_5
#define COHER_DEST_BASE_5__DEST_BASE_5_MASK                0xfffff000L

// COHER_DEST_BASE_6
#define COHER_DEST_BASE_6__DEST_BASE_6_MASK                0xfffff000L

// COHER_DEST_BASE_7
#define COHER_DEST_BASE_7__DEST_BASE_7_MASK                0xfffff000L

// RB_SURFACE_INFO
#define RB_SURFACE_INFO__SURFACE_PITCH_MASK                0x00003fffL
#define RB_SURFACE_INFO__MSAA_SAMPLES_MASK                 0x0000c000L

// RB_COLOR_INFO
#define RB_COLOR_INFO__COLOR_FORMAT_MASK                   0x0000000fL
#define RB_COLOR_INFO__COLOR_ROUND_MODE_MASK               0x00000030L
#define RB_COLOR_INFO__COLOR_LINEAR_MASK                   0x00000040L
#define RB_COLOR_INFO__COLOR_LINEAR                        0x00000040L
#define RB_COLOR_INFO__COLOR_ENDIAN_MASK                   0x00000180L
#define RB_COLOR_INFO__COLOR_SWAP_MASK                     0x00000600L
#define RB_COLOR_INFO__COLOR_BASE_MASK                     0xfffff000L

// RB_DEPTH_INFO
#define RB_DEPTH_INFO__DEPTH_FORMAT_MASK                   0x00000001L
#define RB_DEPTH_INFO__DEPTH_FORMAT                        0x00000001L
#define RB_DEPTH_INFO__DEPTH_BASE_MASK                     0xfffff000L

// RB_STENCILREFMASK
#define RB_STENCILREFMASK__STENCILREF_MASK                 0x000000ffL
#define RB_STENCILREFMASK__STENCILMASK_MASK                0x0000ff00L
#define RB_STENCILREFMASK__STENCILWRITEMASK_MASK           0x00ff0000L

// RB_ALPHA_REF
#define RB_ALPHA_REF__ALPHA_REF_MASK                       0xffffffffL

// RB_COLOR_MASK
#define RB_COLOR_MASK__WRITE_RED_MASK                      0x00000001L
#define RB_COLOR_MASK__WRITE_RED                           0x00000001L
#define RB_COLOR_MASK__WRITE_GREEN_MASK                    0x00000002L
#define RB_COLOR_MASK__WRITE_GREEN                         0x00000002L
#define RB_COLOR_MASK__WRITE_BLUE_MASK                     0x00000004L
#define RB_COLOR_MASK__WRITE_BLUE                          0x00000004L
#define RB_COLOR_MASK__WRITE_ALPHA_MASK                    0x00000008L
#define RB_COLOR_MASK__WRITE_ALPHA                         0x00000008L

// RB_BLEND_RED
#define RB_BLEND_RED__BLEND_RED_MASK                       0x000000ffL

// RB_BLEND_GREEN
#define RB_BLEND_GREEN__BLEND_GREEN_MASK                   0x000000ffL

// RB_BLEND_BLUE
#define RB_BLEND_BLUE__BLEND_BLUE_MASK                     0x000000ffL

// RB_BLEND_ALPHA
#define RB_BLEND_ALPHA__BLEND_ALPHA_MASK                   0x000000ffL

// RB_FOG_COLOR
#define RB_FOG_COLOR__FOG_RED_MASK                         0x000000ffL
#define RB_FOG_COLOR__FOG_GREEN_MASK                       0x0000ff00L
#define RB_FOG_COLOR__FOG_BLUE_MASK                        0x00ff0000L

// RB_STENCILREFMASK_BF
#define RB_STENCILREFMASK_BF__STENCILREF_BF_MASK           0x000000ffL
#define RB_STENCILREFMASK_BF__STENCILMASK_BF_MASK          0x0000ff00L
#define RB_STENCILREFMASK_BF__STENCILWRITEMASK_BF_MASK     0x00ff0000L

// RB_DEPTHCONTROL
#define RB_DEPTHCONTROL__STENCIL_ENABLE_MASK               0x00000001L
#define RB_DEPTHCONTROL__STENCIL_ENABLE                    0x00000001L
#define RB_DEPTHCONTROL__Z_ENABLE_MASK                     0x00000002L
#define RB_DEPTHCONTROL__Z_ENABLE                          0x00000002L
#define RB_DEPTHCONTROL__Z_WRITE_ENABLE_MASK               0x00000004L
#define RB_DEPTHCONTROL__Z_WRITE_ENABLE                    0x00000004L
#define RB_DEPTHCONTROL__EARLY_Z_ENABLE_MASK               0x00000008L
#define RB_DEPTHCONTROL__EARLY_Z_ENABLE                    0x00000008L
#define RB_DEPTHCONTROL__ZFUNC_MASK                        0x00000070L
#define RB_DEPTHCONTROL__BACKFACE_ENABLE_MASK              0x00000080L
#define RB_DEPTHCONTROL__BACKFACE_ENABLE                   0x00000080L
#define RB_DEPTHCONTROL__STENCILFUNC_MASK                  0x00000700L
#define RB_DEPTHCONTROL__STENCILFAIL_MASK                  0x00003800L
#define RB_DEPTHCONTROL__STENCILZPASS_MASK                 0x0001c000L
#define RB_DEPTHCONTROL__STENCILZFAIL_MASK                 0x000e0000L
#define RB_DEPTHCONTROL__STENCILFUNC_BF_MASK               0x00700000L
#define RB_DEPTHCONTROL__STENCILFAIL_BF_MASK               0x03800000L
#define RB_DEPTHCONTROL__STENCILZPASS_BF_MASK              0x1c000000L
#define RB_DEPTHCONTROL__STENCILZFAIL_BF_MASK              0xe0000000L

// RB_BLENDCONTROL
#define RB_BLENDCONTROL__COLOR_SRCBLEND_MASK               0x0000001fL
#define RB_BLENDCONTROL__COLOR_COMB_FCN_MASK               0x000000e0L
#define RB_BLENDCONTROL__COLOR_DESTBLEND_MASK              0x00001f00L
#define RB_BLENDCONTROL__ALPHA_SRCBLEND_MASK               0x001f0000L
#define RB_BLENDCONTROL__ALPHA_COMB_FCN_MASK               0x00e00000L
#define RB_BLENDCONTROL__ALPHA_DESTBLEND_MASK              0x1f000000L
#define RB_BLENDCONTROL__BLEND_FORCE_ENABLE_MASK           0x20000000L
#define RB_BLENDCONTROL__BLEND_FORCE_ENABLE                0x20000000L
#define RB_BLENDCONTROL__BLEND_FORCE_MASK                  0x40000000L
#define RB_BLENDCONTROL__BLEND_FORCE                       0x40000000L

// RB_COLORCONTROL
#define RB_COLORCONTROL__ALPHA_FUNC_MASK                   0x00000007L
#define RB_COLORCONTROL__ALPHA_TEST_ENABLE_MASK            0x00000008L
#define RB_COLORCONTROL__ALPHA_TEST_ENABLE                 0x00000008L
#define RB_COLORCONTROL__ALPHA_TO_MASK_ENABLE_MASK         0x00000010L
#define RB_COLORCONTROL__ALPHA_TO_MASK_ENABLE              0x00000010L
#define RB_COLORCONTROL__BLEND_DISABLE_MASK                0x00000020L
#define RB_COLORCONTROL__BLEND_DISABLE                     0x00000020L
#define RB_COLORCONTROL__FOG_ENABLE_MASK                   0x00000040L
#define RB_COLORCONTROL__FOG_ENABLE                        0x00000040L
#define RB_COLORCONTROL__VS_EXPORTS_FOG_MASK               0x00000080L
#define RB_COLORCONTROL__VS_EXPORTS_FOG                    0x00000080L
#define RB_COLORCONTROL__ROP_CODE_MASK                     0x00000f00L
#define RB_COLORCONTROL__DITHER_MODE_MASK                  0x00003000L
#define RB_COLORCONTROL__DITHER_TYPE_MASK                  0x0000c000L
#define RB_COLORCONTROL__PIXEL_FOG_MASK                    0x00010000L
#define RB_COLORCONTROL__PIXEL_FOG                         0x00010000L
#define RB_COLORCONTROL__ALPHA_TO_MASK_OFFSET0_MASK        0x03000000L
#define RB_COLORCONTROL__ALPHA_TO_MASK_OFFSET1_MASK        0x0c000000L
#define RB_COLORCONTROL__ALPHA_TO_MASK_OFFSET2_MASK        0x30000000L
#define RB_COLORCONTROL__ALPHA_TO_MASK_OFFSET3_MASK        0xc0000000L

// RB_MODECONTROL
#define RB_MODECONTROL__EDRAM_MODE_MASK                    0x00000007L

// RB_COLOR_DEST_MASK
#define RB_COLOR_DEST_MASK__COLOR_DEST_MASK_MASK           0xffffffffL

// RB_COPY_CONTROL
#define RB_COPY_CONTROL__COPY_SAMPLE_SELECT_MASK           0x00000007L
#define RB_COPY_CONTROL__DEPTH_CLEAR_ENABLE_MASK           0x00000008L
#define RB_COPY_CONTROL__DEPTH_CLEAR_ENABLE                0x00000008L
#define RB_COPY_CONTROL__CLEAR_MASK_MASK                   0x000000f0L

// RB_COPY_DEST_BASE
#define RB_COPY_DEST_BASE__COPY_DEST_BASE_MASK             0xfffff000L

// RB_COPY_DEST_PITCH
#define RB_COPY_DEST_PITCH__COPY_DEST_PITCH_MASK           0x000001ffL

// RB_COPY_DEST_INFO
#define RB_COPY_DEST_INFO__COPY_DEST_ENDIAN_MASK           0x00000007L
#define RB_COPY_DEST_INFO__COPY_DEST_LINEAR_MASK           0x00000008L
#define RB_COPY_DEST_INFO__COPY_DEST_LINEAR                0x00000008L
#define RB_COPY_DEST_INFO__COPY_DEST_FORMAT_MASK           0x000000f0L
#define RB_COPY_DEST_INFO__COPY_DEST_SWAP_MASK             0x00000300L
#define RB_COPY_DEST_INFO__COPY_DEST_DITHER_MODE_MASK      0x00000c00L
#define RB_COPY_DEST_INFO__COPY_DEST_DITHER_TYPE_MASK      0x00003000L
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_RED_MASK        0x00004000L
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_RED             0x00004000L
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_GREEN_MASK      0x00008000L
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_GREEN           0x00008000L
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_BLUE_MASK       0x00010000L
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_BLUE            0x00010000L
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_ALPHA_MASK      0x00020000L
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_ALPHA           0x00020000L

// RB_COPY_DEST_PIXEL_OFFSET
#define RB_COPY_DEST_PIXEL_OFFSET__OFFSET_X_MASK           0x00001fffL
#define RB_COPY_DEST_PIXEL_OFFSET__OFFSET_Y_MASK           0x03ffe000L

// RB_DEPTH_CLEAR
#define RB_DEPTH_CLEAR__DEPTH_CLEAR_MASK                   0xffffffffL

// RB_SAMPLE_COUNT_CTL
#define RB_SAMPLE_COUNT_CTL__RESET_SAMPLE_COUNT_MASK       0x00000001L
#define RB_SAMPLE_COUNT_CTL__RESET_SAMPLE_COUNT            0x00000001L
#define RB_SAMPLE_COUNT_CTL__COPY_SAMPLE_COUNT_MASK        0x00000002L
#define RB_SAMPLE_COUNT_CTL__COPY_SAMPLE_COUNT             0x00000002L

// RB_SAMPLE_COUNT_ADDR
#define RB_SAMPLE_COUNT_ADDR__SAMPLE_COUNT_ADDR_MASK       0xffffffffL

// RB_BC_CONTROL
#define RB_BC_CONTROL__ACCUM_LINEAR_MODE_ENABLE_MASK       0x00000001L
#define RB_BC_CONTROL__ACCUM_LINEAR_MODE_ENABLE            0x00000001L
#define RB_BC_CONTROL__ACCUM_TIMEOUT_SELECT_MASK           0x00000006L
#define RB_BC_CONTROL__DISABLE_EDRAM_CAM_MASK              0x00000008L
#define RB_BC_CONTROL__DISABLE_EDRAM_CAM                   0x00000008L
#define RB_BC_CONTROL__DISABLE_EZ_FAST_CONTEXT_SWITCH_MASK 0x00000010L
#define RB_BC_CONTROL__DISABLE_EZ_FAST_CONTEXT_SWITCH      0x00000010L
#define RB_BC_CONTROL__DISABLE_EZ_NULL_ZCMD_DROP_MASK      0x00000020L
#define RB_BC_CONTROL__DISABLE_EZ_NULL_ZCMD_DROP           0x00000020L
#define RB_BC_CONTROL__DISABLE_LZ_NULL_ZCMD_DROP_MASK      0x00000040L
#define RB_BC_CONTROL__DISABLE_LZ_NULL_ZCMD_DROP           0x00000040L
#define RB_BC_CONTROL__ENABLE_AZ_THROTTLE_MASK             0x00000080L
#define RB_BC_CONTROL__ENABLE_AZ_THROTTLE                  0x00000080L
#define RB_BC_CONTROL__AZ_THROTTLE_COUNT_MASK              0x00001f00L
#define RB_BC_CONTROL__ENABLE_CRC_UPDATE_MASK              0x00004000L
#define RB_BC_CONTROL__ENABLE_CRC_UPDATE                   0x00004000L
#define RB_BC_CONTROL__CRC_MODE_MASK                       0x00008000L
#define RB_BC_CONTROL__CRC_MODE                            0x00008000L
#define RB_BC_CONTROL__DISABLE_SAMPLE_COUNTERS_MASK        0x00010000L
#define RB_BC_CONTROL__DISABLE_SAMPLE_COUNTERS             0x00010000L
#define RB_BC_CONTROL__DISABLE_ACCUM_MASK                  0x00020000L
#define RB_BC_CONTROL__DISABLE_ACCUM                       0x00020000L
#define RB_BC_CONTROL__ACCUM_ALLOC_MASK_MASK               0x003c0000L
#define RB_BC_CONTROL__LINEAR_PERFORMANCE_ENABLE_MASK      0x00400000L
#define RB_BC_CONTROL__LINEAR_PERFORMANCE_ENABLE           0x00400000L
#define RB_BC_CONTROL__ACCUM_DATA_FIFO_LIMIT_MASK          0x07800000L
#define RB_BC_CONTROL__MEM_EXPORT_TIMEOUT_SELECT_MASK      0x18000000L
#define RB_BC_CONTROL__MEM_EXPORT_LINEAR_MODE_ENABLE_MASK  0x20000000L
#define RB_BC_CONTROL__MEM_EXPORT_LINEAR_MODE_ENABLE       0x20000000L
#define RB_BC_CONTROL__RESERVED9_MASK                      0x40000000L
#define RB_BC_CONTROL__RESERVED9                           0x40000000L
#define RB_BC_CONTROL__RESERVED10_MASK                     0x80000000L
#define RB_BC_CONTROL__RESERVED10                          0x80000000L

// RB_EDRAM_INFO
#define RB_EDRAM_INFO__EDRAM_SIZE_MASK                     0x0000000fL
#define RB_EDRAM_INFO__EDRAM_MAPPING_MODE_MASK             0x00000030L
#define RB_EDRAM_INFO__EDRAM_RANGE_MASK                    0xffffc000L

// RB_CRC_RD_PORT
#define RB_CRC_RD_PORT__CRC_DATA_MASK                      0xffffffffL

// RB_CRC_CONTROL
#define RB_CRC_CONTROL__CRC_RD_ADVANCE_MASK                0x00000001L
#define RB_CRC_CONTROL__CRC_RD_ADVANCE                     0x00000001L

// RB_CRC_MASK
#define RB_CRC_MASK__CRC_MASK_MASK                         0xffffffffL

// RB_PERFCOUNTER0_SELECT
#define RB_PERFCOUNTER0_SELECT__PERF_SEL_MASK              0x000000ffL

// RB_PERFCOUNTER0_LOW
#define RB_PERFCOUNTER0_LOW__PERF_COUNT_MASK               0xffffffffL

// RB_PERFCOUNTER0_HI
#define RB_PERFCOUNTER0_HI__PERF_COUNT_MASK                0x0000ffffL

// RB_TOTAL_SAMPLES
#define RB_TOTAL_SAMPLES__TOTAL_SAMPLES_MASK               0xffffffffL

// RB_ZPASS_SAMPLES
#define RB_ZPASS_SAMPLES__ZPASS_SAMPLES_MASK               0xffffffffL

// RB_ZFAIL_SAMPLES
#define RB_ZFAIL_SAMPLES__ZFAIL_SAMPLES_MASK               0xffffffffL

// RB_SFAIL_SAMPLES
#define RB_SFAIL_SAMPLES__SFAIL_SAMPLES_MASK               0xffffffffL

// RB_DEBUG_0
#define RB_DEBUG_0__RDREQ_CTL_Z1_PRE_FULL_MASK             0x00000001L
#define RB_DEBUG_0__RDREQ_CTL_Z1_PRE_FULL                  0x00000001L
#define RB_DEBUG_0__RDREQ_CTL_Z0_PRE_FULL_MASK             0x00000002L
#define RB_DEBUG_0__RDREQ_CTL_Z0_PRE_FULL                  0x00000002L
#define RB_DEBUG_0__RDREQ_CTL_C1_PRE_FULL_MASK             0x00000004L
#define RB_DEBUG_0__RDREQ_CTL_C1_PRE_FULL                  0x00000004L
#define RB_DEBUG_0__RDREQ_CTL_C0_PRE_FULL_MASK             0x00000008L
#define RB_DEBUG_0__RDREQ_CTL_C0_PRE_FULL                  0x00000008L
#define RB_DEBUG_0__RDREQ_E1_ORDERING_FULL_MASK            0x00000010L
#define RB_DEBUG_0__RDREQ_E1_ORDERING_FULL                 0x00000010L
#define RB_DEBUG_0__RDREQ_E0_ORDERING_FULL_MASK            0x00000020L
#define RB_DEBUG_0__RDREQ_E0_ORDERING_FULL                 0x00000020L
#define RB_DEBUG_0__RDREQ_Z1_FULL_MASK                     0x00000040L
#define RB_DEBUG_0__RDREQ_Z1_FULL                          0x00000040L
#define RB_DEBUG_0__RDREQ_Z0_FULL_MASK                     0x00000080L
#define RB_DEBUG_0__RDREQ_Z0_FULL                          0x00000080L
#define RB_DEBUG_0__RDREQ_C1_FULL_MASK                     0x00000100L
#define RB_DEBUG_0__RDREQ_C1_FULL                          0x00000100L
#define RB_DEBUG_0__RDREQ_C0_FULL_MASK                     0x00000200L
#define RB_DEBUG_0__RDREQ_C0_FULL                          0x00000200L
#define RB_DEBUG_0__WRREQ_E1_MACRO_HI_FULL_MASK            0x00000400L
#define RB_DEBUG_0__WRREQ_E1_MACRO_HI_FULL                 0x00000400L
#define RB_DEBUG_0__WRREQ_E1_MACRO_LO_FULL_MASK            0x00000800L
#define RB_DEBUG_0__WRREQ_E1_MACRO_LO_FULL                 0x00000800L
#define RB_DEBUG_0__WRREQ_E0_MACRO_HI_FULL_MASK            0x00001000L
#define RB_DEBUG_0__WRREQ_E0_MACRO_HI_FULL                 0x00001000L
#define RB_DEBUG_0__WRREQ_E0_MACRO_LO_FULL_MASK            0x00002000L
#define RB_DEBUG_0__WRREQ_E0_MACRO_LO_FULL                 0x00002000L
#define RB_DEBUG_0__WRREQ_C_WE_HI_FULL_MASK                0x00004000L
#define RB_DEBUG_0__WRREQ_C_WE_HI_FULL                     0x00004000L
#define RB_DEBUG_0__WRREQ_C_WE_LO_FULL_MASK                0x00008000L
#define RB_DEBUG_0__WRREQ_C_WE_LO_FULL                     0x00008000L
#define RB_DEBUG_0__WRREQ_Z1_FULL_MASK                     0x00010000L
#define RB_DEBUG_0__WRREQ_Z1_FULL                          0x00010000L
#define RB_DEBUG_0__WRREQ_Z0_FULL_MASK                     0x00020000L
#define RB_DEBUG_0__WRREQ_Z0_FULL                          0x00020000L
#define RB_DEBUG_0__WRREQ_C1_FULL_MASK                     0x00040000L
#define RB_DEBUG_0__WRREQ_C1_FULL                          0x00040000L
#define RB_DEBUG_0__WRREQ_C0_FULL_MASK                     0x00080000L
#define RB_DEBUG_0__WRREQ_C0_FULL                          0x00080000L
#define RB_DEBUG_0__CMDFIFO_Z1_HOLD_FULL_MASK              0x00100000L
#define RB_DEBUG_0__CMDFIFO_Z1_HOLD_FULL                   0x00100000L
#define RB_DEBUG_0__CMDFIFO_Z0_HOLD_FULL_MASK              0x00200000L
#define RB_DEBUG_0__CMDFIFO_Z0_HOLD_FULL                   0x00200000L
#define RB_DEBUG_0__CMDFIFO_C1_HOLD_FULL_MASK              0x00400000L
#define RB_DEBUG_0__CMDFIFO_C1_HOLD_FULL                   0x00400000L
#define RB_DEBUG_0__CMDFIFO_C0_HOLD_FULL_MASK              0x00800000L
#define RB_DEBUG_0__CMDFIFO_C0_HOLD_FULL                   0x00800000L
#define RB_DEBUG_0__CMDFIFO_Z_ORDERING_FULL_MASK           0x01000000L
#define RB_DEBUG_0__CMDFIFO_Z_ORDERING_FULL                0x01000000L
#define RB_DEBUG_0__CMDFIFO_C_ORDERING_FULL_MASK           0x02000000L
#define RB_DEBUG_0__CMDFIFO_C_ORDERING_FULL                0x02000000L
#define RB_DEBUG_0__C_SX_LAT_FULL_MASK                     0x04000000L
#define RB_DEBUG_0__C_SX_LAT_FULL                          0x04000000L
#define RB_DEBUG_0__C_SX_CMD_FULL_MASK                     0x08000000L
#define RB_DEBUG_0__C_SX_CMD_FULL                          0x08000000L
#define RB_DEBUG_0__C_EZ_TILE_FULL_MASK                    0x10000000L
#define RB_DEBUG_0__C_EZ_TILE_FULL                         0x10000000L
#define RB_DEBUG_0__C_REQ_FULL_MASK                        0x20000000L
#define RB_DEBUG_0__C_REQ_FULL                             0x20000000L
#define RB_DEBUG_0__C_MASK_FULL_MASK                       0x40000000L
#define RB_DEBUG_0__C_MASK_FULL                            0x40000000L
#define RB_DEBUG_0__EZ_INFSAMP_FULL_MASK                   0x80000000L
#define RB_DEBUG_0__EZ_INFSAMP_FULL                        0x80000000L

// RB_DEBUG_1
#define RB_DEBUG_1__RDREQ_Z1_CMD_EMPTY_MASK                0x00000001L
#define RB_DEBUG_1__RDREQ_Z1_CMD_EMPTY                     0x00000001L
#define RB_DEBUG_1__RDREQ_Z0_CMD_EMPTY_MASK                0x00000002L
#define RB_DEBUG_1__RDREQ_Z0_CMD_EMPTY                     0x00000002L
#define RB_DEBUG_1__RDREQ_C1_CMD_EMPTY_MASK                0x00000004L
#define RB_DEBUG_1__RDREQ_C1_CMD_EMPTY                     0x00000004L
#define RB_DEBUG_1__RDREQ_C0_CMD_EMPTY_MASK                0x00000008L
#define RB_DEBUG_1__RDREQ_C0_CMD_EMPTY                     0x00000008L
#define RB_DEBUG_1__RDREQ_E1_ORDERING_EMPTY_MASK           0x00000010L
#define RB_DEBUG_1__RDREQ_E1_ORDERING_EMPTY                0x00000010L
#define RB_DEBUG_1__RDREQ_E0_ORDERING_EMPTY_MASK           0x00000020L
#define RB_DEBUG_1__RDREQ_E0_ORDERING_EMPTY                0x00000020L
#define RB_DEBUG_1__RDREQ_Z1_EMPTY_MASK                    0x00000040L
#define RB_DEBUG_1__RDREQ_Z1_EMPTY                         0x00000040L
#define RB_DEBUG_1__RDREQ_Z0_EMPTY_MASK                    0x00000080L
#define RB_DEBUG_1__RDREQ_Z0_EMPTY                         0x00000080L
#define RB_DEBUG_1__RDREQ_C1_EMPTY_MASK                    0x00000100L
#define RB_DEBUG_1__RDREQ_C1_EMPTY                         0x00000100L
#define RB_DEBUG_1__RDREQ_C0_EMPTY_MASK                    0x00000200L
#define RB_DEBUG_1__RDREQ_C0_EMPTY                         0x00000200L
#define RB_DEBUG_1__WRREQ_E1_MACRO_HI_EMPTY_MASK           0x00000400L
#define RB_DEBUG_1__WRREQ_E1_MACRO_HI_EMPTY                0x00000400L
#define RB_DEBUG_1__WRREQ_E1_MACRO_LO_EMPTY_MASK           0x00000800L
#define RB_DEBUG_1__WRREQ_E1_MACRO_LO_EMPTY                0x00000800L
#define RB_DEBUG_1__WRREQ_E0_MACRO_HI_EMPTY_MASK           0x00001000L
#define RB_DEBUG_1__WRREQ_E0_MACRO_HI_EMPTY                0x00001000L
#define RB_DEBUG_1__WRREQ_E0_MACRO_LO_EMPTY_MASK           0x00002000L
#define RB_DEBUG_1__WRREQ_E0_MACRO_LO_EMPTY                0x00002000L
#define RB_DEBUG_1__WRREQ_C_WE_HI_EMPTY_MASK               0x00004000L
#define RB_DEBUG_1__WRREQ_C_WE_HI_EMPTY                    0x00004000L
#define RB_DEBUG_1__WRREQ_C_WE_LO_EMPTY_MASK               0x00008000L
#define RB_DEBUG_1__WRREQ_C_WE_LO_EMPTY                    0x00008000L
#define RB_DEBUG_1__WRREQ_Z1_EMPTY_MASK                    0x00010000L
#define RB_DEBUG_1__WRREQ_Z1_EMPTY                         0x00010000L
#define RB_DEBUG_1__WRREQ_Z0_EMPTY_MASK                    0x00020000L
#define RB_DEBUG_1__WRREQ_Z0_EMPTY                         0x00020000L
#define RB_DEBUG_1__WRREQ_C1_PRE_EMPTY_MASK                0x00040000L
#define RB_DEBUG_1__WRREQ_C1_PRE_EMPTY                     0x00040000L
#define RB_DEBUG_1__WRREQ_C0_PRE_EMPTY_MASK                0x00080000L
#define RB_DEBUG_1__WRREQ_C0_PRE_EMPTY                     0x00080000L
#define RB_DEBUG_1__CMDFIFO_Z1_HOLD_EMPTY_MASK             0x00100000L
#define RB_DEBUG_1__CMDFIFO_Z1_HOLD_EMPTY                  0x00100000L
#define RB_DEBUG_1__CMDFIFO_Z0_HOLD_EMPTY_MASK             0x00200000L
#define RB_DEBUG_1__CMDFIFO_Z0_HOLD_EMPTY                  0x00200000L
#define RB_DEBUG_1__CMDFIFO_C1_HOLD_EMPTY_MASK             0x00400000L
#define RB_DEBUG_1__CMDFIFO_C1_HOLD_EMPTY                  0x00400000L
#define RB_DEBUG_1__CMDFIFO_C0_HOLD_EMPTY_MASK             0x00800000L
#define RB_DEBUG_1__CMDFIFO_C0_HOLD_EMPTY                  0x00800000L
#define RB_DEBUG_1__CMDFIFO_Z_ORDERING_EMPTY_MASK          0x01000000L
#define RB_DEBUG_1__CMDFIFO_Z_ORDERING_EMPTY               0x01000000L
#define RB_DEBUG_1__CMDFIFO_C_ORDERING_EMPTY_MASK          0x02000000L
#define RB_DEBUG_1__CMDFIFO_C_ORDERING_EMPTY               0x02000000L
#define RB_DEBUG_1__C_SX_LAT_EMPTY_MASK                    0x04000000L
#define RB_DEBUG_1__C_SX_LAT_EMPTY                         0x04000000L
#define RB_DEBUG_1__C_SX_CMD_EMPTY_MASK                    0x08000000L
#define RB_DEBUG_1__C_SX_CMD_EMPTY                         0x08000000L
#define RB_DEBUG_1__C_EZ_TILE_EMPTY_MASK                   0x10000000L
#define RB_DEBUG_1__C_EZ_TILE_EMPTY                        0x10000000L
#define RB_DEBUG_1__C_REQ_EMPTY_MASK                       0x20000000L
#define RB_DEBUG_1__C_REQ_EMPTY                            0x20000000L
#define RB_DEBUG_1__C_MASK_EMPTY_MASK                      0x40000000L
#define RB_DEBUG_1__C_MASK_EMPTY                           0x40000000L
#define RB_DEBUG_1__EZ_INFSAMP_EMPTY_MASK                  0x80000000L
#define RB_DEBUG_1__EZ_INFSAMP_EMPTY                       0x80000000L

// RB_DEBUG_2
#define RB_DEBUG_2__TILE_FIFO_COUNT_MASK                   0x0000000fL
#define RB_DEBUG_2__SX_LAT_FIFO_COUNT_MASK                 0x000007f0L
#define RB_DEBUG_2__MEM_EXPORT_FLAG_MASK                   0x00000800L
#define RB_DEBUG_2__MEM_EXPORT_FLAG                        0x00000800L
#define RB_DEBUG_2__SYSMEM_BLEND_FLAG_MASK                 0x00001000L
#define RB_DEBUG_2__SYSMEM_BLEND_FLAG                      0x00001000L
#define RB_DEBUG_2__CURRENT_TILE_EVENT_MASK                0x00002000L
#define RB_DEBUG_2__CURRENT_TILE_EVENT                     0x00002000L
#define RB_DEBUG_2__EZ_INFTILE_FULL_MASK                   0x00004000L
#define RB_DEBUG_2__EZ_INFTILE_FULL                        0x00004000L
#define RB_DEBUG_2__EZ_MASK_LOWER_FULL_MASK                0x00008000L
#define RB_DEBUG_2__EZ_MASK_LOWER_FULL                     0x00008000L
#define RB_DEBUG_2__EZ_MASK_UPPER_FULL_MASK                0x00010000L
#define RB_DEBUG_2__EZ_MASK_UPPER_FULL                     0x00010000L
#define RB_DEBUG_2__Z0_MASK_FULL_MASK                      0x00020000L
#define RB_DEBUG_2__Z0_MASK_FULL                           0x00020000L
#define RB_DEBUG_2__Z1_MASK_FULL_MASK                      0x00040000L
#define RB_DEBUG_2__Z1_MASK_FULL                           0x00040000L
#define RB_DEBUG_2__Z0_REQ_FULL_MASK                       0x00080000L
#define RB_DEBUG_2__Z0_REQ_FULL                            0x00080000L
#define RB_DEBUG_2__Z1_REQ_FULL_MASK                       0x00100000L
#define RB_DEBUG_2__Z1_REQ_FULL                            0x00100000L
#define RB_DEBUG_2__Z_SAMP_FULL_MASK                       0x00200000L
#define RB_DEBUG_2__Z_SAMP_FULL                            0x00200000L
#define RB_DEBUG_2__Z_TILE_FULL_MASK                       0x00400000L
#define RB_DEBUG_2__Z_TILE_FULL                            0x00400000L
#define RB_DEBUG_2__EZ_INFTILE_EMPTY_MASK                  0x00800000L
#define RB_DEBUG_2__EZ_INFTILE_EMPTY                       0x00800000L
#define RB_DEBUG_2__EZ_MASK_LOWER_EMPTY_MASK               0x01000000L
#define RB_DEBUG_2__EZ_MASK_LOWER_EMPTY                    0x01000000L
#define RB_DEBUG_2__EZ_MASK_UPPER_EMPTY_MASK               0x02000000L
#define RB_DEBUG_2__EZ_MASK_UPPER_EMPTY                    0x02000000L
#define RB_DEBUG_2__Z0_MASK_EMPTY_MASK                     0x04000000L
#define RB_DEBUG_2__Z0_MASK_EMPTY                          0x04000000L
#define RB_DEBUG_2__Z1_MASK_EMPTY_MASK                     0x08000000L
#define RB_DEBUG_2__Z1_MASK_EMPTY                          0x08000000L
#define RB_DEBUG_2__Z0_REQ_EMPTY_MASK                      0x10000000L
#define RB_DEBUG_2__Z0_REQ_EMPTY                           0x10000000L
#define RB_DEBUG_2__Z1_REQ_EMPTY_MASK                      0x20000000L
#define RB_DEBUG_2__Z1_REQ_EMPTY                           0x20000000L
#define RB_DEBUG_2__Z_SAMP_EMPTY_MASK                      0x40000000L
#define RB_DEBUG_2__Z_SAMP_EMPTY                           0x40000000L
#define RB_DEBUG_2__Z_TILE_EMPTY_MASK                      0x80000000L
#define RB_DEBUG_2__Z_TILE_EMPTY                           0x80000000L

// RB_DEBUG_3
#define RB_DEBUG_3__ACCUM_VALID_MASK                       0x0000000fL
#define RB_DEBUG_3__ACCUM_FLUSHING_MASK                    0x000000f0L
#define RB_DEBUG_3__ACCUM_WRITE_CLEAN_COUNT_MASK           0x00003f00L
#define RB_DEBUG_3__ACCUM_INPUT_REG_VALID_MASK             0x00004000L
#define RB_DEBUG_3__ACCUM_INPUT_REG_VALID                  0x00004000L
#define RB_DEBUG_3__ACCUM_DATA_FIFO_CNT_MASK               0x00078000L
#define RB_DEBUG_3__SHD_FULL_MASK                          0x00080000L
#define RB_DEBUG_3__SHD_FULL                               0x00080000L
#define RB_DEBUG_3__SHD_EMPTY_MASK                         0x00100000L
#define RB_DEBUG_3__SHD_EMPTY                              0x00100000L
#define RB_DEBUG_3__EZ_RETURN_LOWER_EMPTY_MASK             0x00200000L
#define RB_DEBUG_3__EZ_RETURN_LOWER_EMPTY                  0x00200000L
#define RB_DEBUG_3__EZ_RETURN_UPPER_EMPTY_MASK             0x00400000L
#define RB_DEBUG_3__EZ_RETURN_UPPER_EMPTY                  0x00400000L
#define RB_DEBUG_3__EZ_RETURN_LOWER_FULL_MASK              0x00800000L
#define RB_DEBUG_3__EZ_RETURN_LOWER_FULL                   0x00800000L
#define RB_DEBUG_3__EZ_RETURN_UPPER_FULL_MASK              0x01000000L
#define RB_DEBUG_3__EZ_RETURN_UPPER_FULL                   0x01000000L
#define RB_DEBUG_3__ZEXP_LOWER_EMPTY_MASK                  0x02000000L
#define RB_DEBUG_3__ZEXP_LOWER_EMPTY                       0x02000000L
#define RB_DEBUG_3__ZEXP_UPPER_EMPTY_MASK                  0x04000000L
#define RB_DEBUG_3__ZEXP_UPPER_EMPTY                       0x04000000L
#define RB_DEBUG_3__ZEXP_LOWER_FULL_MASK                   0x08000000L
#define RB_DEBUG_3__ZEXP_LOWER_FULL                        0x08000000L
#define RB_DEBUG_3__ZEXP_UPPER_FULL_MASK                   0x10000000L
#define RB_DEBUG_3__ZEXP_UPPER_FULL                        0x10000000L

// RB_DEBUG_4
#define RB_DEBUG_4__GMEM_RD_ACCESS_FLAG_MASK               0x00000001L
#define RB_DEBUG_4__GMEM_RD_ACCESS_FLAG                    0x00000001L
#define RB_DEBUG_4__GMEM_WR_ACCESS_FLAG_MASK               0x00000002L
#define RB_DEBUG_4__GMEM_WR_ACCESS_FLAG                    0x00000002L
#define RB_DEBUG_4__SYSMEM_RD_ACCESS_FLAG_MASK             0x00000004L
#define RB_DEBUG_4__SYSMEM_RD_ACCESS_FLAG                  0x00000004L
#define RB_DEBUG_4__SYSMEM_WR_ACCESS_FLAG_MASK             0x00000008L
#define RB_DEBUG_4__SYSMEM_WR_ACCESS_FLAG                  0x00000008L
#define RB_DEBUG_4__ACCUM_DATA_FIFO_EMPTY_MASK             0x00000010L
#define RB_DEBUG_4__ACCUM_DATA_FIFO_EMPTY                  0x00000010L
#define RB_DEBUG_4__ACCUM_ORDER_FIFO_EMPTY_MASK            0x00000020L
#define RB_DEBUG_4__ACCUM_ORDER_FIFO_EMPTY                 0x00000020L
#define RB_DEBUG_4__ACCUM_DATA_FIFO_FULL_MASK              0x00000040L
#define RB_DEBUG_4__ACCUM_DATA_FIFO_FULL                   0x00000040L
#define RB_DEBUG_4__ACCUM_ORDER_FIFO_FULL_MASK             0x00000080L
#define RB_DEBUG_4__ACCUM_ORDER_FIFO_FULL                  0x00000080L
#define RB_DEBUG_4__SYSMEM_WRITE_COUNT_OVERFLOW_MASK       0x00000100L
#define RB_DEBUG_4__SYSMEM_WRITE_COUNT_OVERFLOW            0x00000100L
#define RB_DEBUG_4__CONTEXT_COUNT_DEBUG_MASK               0x00001e00L

// RB_FLAG_CONTROL
#define RB_FLAG_CONTROL__DEBUG_FLAG_CLEAR_MASK             0x00000001L
#define RB_FLAG_CONTROL__DEBUG_FLAG_CLEAR                  0x00000001L

// BC_DUMMY_CRAYRB_ENUMS
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_DEPTH_FORMAT_MASK 0x0000003fL
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_SWAP_MASK 0x00000040L
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_SWAP   0x00000040L
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_DEPTH_ARRAY_MASK 0x00000180L
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_ARRAY_MASK     0x00000600L
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_COLOR_FORMAT_MASK 0x0001f800L
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_NUMBER_MASK 0x000e0000L
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_FORMAT_MASK 0x03f00000L
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_TILING_MASK 0x04000000L
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_TILING 0x04000000L
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_ARRAY_MASK 0x18000000L
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_RB_COPY_DEST_INFO_NUMBER_MASK 0xe0000000L

// BC_DUMMY_CRAYRB_MOREENUMS
#define BC_DUMMY_CRAYRB_MOREENUMS__DUMMY_CRAYRB_COLORARRAYX_MASK 0x00000003L

#endif
