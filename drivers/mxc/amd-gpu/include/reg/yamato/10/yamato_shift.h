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

#if !defined (_yamato_SHIFT_HEADER)
#define _yamato_SHIFT_HEADER

// PA_CL_VPORT_XSCALE
#define PA_CL_VPORT_XSCALE__VPORT_XSCALE__SHIFT            0x00000000

// PA_CL_VPORT_XOFFSET
#define PA_CL_VPORT_XOFFSET__VPORT_XOFFSET__SHIFT          0x00000000

// PA_CL_VPORT_YSCALE
#define PA_CL_VPORT_YSCALE__VPORT_YSCALE__SHIFT            0x00000000

// PA_CL_VPORT_YOFFSET
#define PA_CL_VPORT_YOFFSET__VPORT_YOFFSET__SHIFT          0x00000000

// PA_CL_VPORT_ZSCALE
#define PA_CL_VPORT_ZSCALE__VPORT_ZSCALE__SHIFT            0x00000000

// PA_CL_VPORT_ZOFFSET
#define PA_CL_VPORT_ZOFFSET__VPORT_ZOFFSET__SHIFT          0x00000000

// PA_CL_VTE_CNTL
#define PA_CL_VTE_CNTL__VPORT_X_SCALE_ENA__SHIFT           0x00000000
#define PA_CL_VTE_CNTL__VPORT_X_OFFSET_ENA__SHIFT          0x00000001
#define PA_CL_VTE_CNTL__VPORT_Y_SCALE_ENA__SHIFT           0x00000002
#define PA_CL_VTE_CNTL__VPORT_Y_OFFSET_ENA__SHIFT          0x00000003
#define PA_CL_VTE_CNTL__VPORT_Z_SCALE_ENA__SHIFT           0x00000004
#define PA_CL_VTE_CNTL__VPORT_Z_OFFSET_ENA__SHIFT          0x00000005
#define PA_CL_VTE_CNTL__VTX_XY_FMT__SHIFT                  0x00000008
#define PA_CL_VTE_CNTL__VTX_Z_FMT__SHIFT                   0x00000009
#define PA_CL_VTE_CNTL__VTX_W0_FMT__SHIFT                  0x0000000a
#define PA_CL_VTE_CNTL__PERFCOUNTER_REF__SHIFT             0x0000000b

// PA_CL_CLIP_CNTL
#define PA_CL_CLIP_CNTL__CLIP_DISABLE__SHIFT               0x00000010
#define PA_CL_CLIP_CNTL__BOUNDARY_EDGE_FLAG_ENA__SHIFT     0x00000012
#define PA_CL_CLIP_CNTL__DX_CLIP_SPACE_DEF__SHIFT          0x00000013
#define PA_CL_CLIP_CNTL__DIS_CLIP_ERR_DETECT__SHIFT        0x00000014
#define PA_CL_CLIP_CNTL__VTX_KILL_OR__SHIFT                0x00000015
#define PA_CL_CLIP_CNTL__XY_NAN_RETAIN__SHIFT              0x00000016
#define PA_CL_CLIP_CNTL__Z_NAN_RETAIN__SHIFT               0x00000017
#define PA_CL_CLIP_CNTL__W_NAN_RETAIN__SHIFT               0x00000018

// PA_CL_GB_VERT_CLIP_ADJ
#define PA_CL_GB_VERT_CLIP_ADJ__DATA_REGISTER__SHIFT       0x00000000

// PA_CL_GB_VERT_DISC_ADJ
#define PA_CL_GB_VERT_DISC_ADJ__DATA_REGISTER__SHIFT       0x00000000

// PA_CL_GB_HORZ_CLIP_ADJ
#define PA_CL_GB_HORZ_CLIP_ADJ__DATA_REGISTER__SHIFT       0x00000000

// PA_CL_GB_HORZ_DISC_ADJ
#define PA_CL_GB_HORZ_DISC_ADJ__DATA_REGISTER__SHIFT       0x00000000

// PA_CL_ENHANCE
#define PA_CL_ENHANCE__CLIP_VTX_REORDER_ENA__SHIFT         0x00000000
#define PA_CL_ENHANCE__ECO_SPARE3__SHIFT                   0x0000001c
#define PA_CL_ENHANCE__ECO_SPARE2__SHIFT                   0x0000001d
#define PA_CL_ENHANCE__ECO_SPARE1__SHIFT                   0x0000001e
#define PA_CL_ENHANCE__ECO_SPARE0__SHIFT                   0x0000001f

// PA_SC_ENHANCE
#define PA_SC_ENHANCE__ECO_SPARE3__SHIFT                   0x0000001c
#define PA_SC_ENHANCE__ECO_SPARE2__SHIFT                   0x0000001d
#define PA_SC_ENHANCE__ECO_SPARE1__SHIFT                   0x0000001e
#define PA_SC_ENHANCE__ECO_SPARE0__SHIFT                   0x0000001f

// PA_SU_VTX_CNTL
#define PA_SU_VTX_CNTL__PIX_CENTER__SHIFT                  0x00000000
#define PA_SU_VTX_CNTL__ROUND_MODE__SHIFT                  0x00000001
#define PA_SU_VTX_CNTL__QUANT_MODE__SHIFT                  0x00000003

// PA_SU_POINT_SIZE
#define PA_SU_POINT_SIZE__HEIGHT__SHIFT                    0x00000000
#define PA_SU_POINT_SIZE__WIDTH__SHIFT                     0x00000010

// PA_SU_POINT_MINMAX
#define PA_SU_POINT_MINMAX__MIN_SIZE__SHIFT                0x00000000
#define PA_SU_POINT_MINMAX__MAX_SIZE__SHIFT                0x00000010

// PA_SU_LINE_CNTL
#define PA_SU_LINE_CNTL__WIDTH__SHIFT                      0x00000000

// PA_SU_FACE_DATA
#define PA_SU_FACE_DATA__BASE_ADDR__SHIFT                  0x00000005

// PA_SU_SC_MODE_CNTL
#define PA_SU_SC_MODE_CNTL__CULL_FRONT__SHIFT              0x00000000
#define PA_SU_SC_MODE_CNTL__CULL_BACK__SHIFT               0x00000001
#define PA_SU_SC_MODE_CNTL__FACE__SHIFT                    0x00000002
#define PA_SU_SC_MODE_CNTL__POLY_MODE__SHIFT               0x00000003
#define PA_SU_SC_MODE_CNTL__POLYMODE_FRONT_PTYPE__SHIFT    0x00000005
#define PA_SU_SC_MODE_CNTL__POLYMODE_BACK_PTYPE__SHIFT     0x00000008
#define PA_SU_SC_MODE_CNTL__POLY_OFFSET_FRONT_ENABLE__SHIFT 0x0000000b
#define PA_SU_SC_MODE_CNTL__POLY_OFFSET_BACK_ENABLE__SHIFT 0x0000000c
#define PA_SU_SC_MODE_CNTL__POLY_OFFSET_PARA_ENABLE__SHIFT 0x0000000d
#define PA_SU_SC_MODE_CNTL__MSAA_ENABLE__SHIFT             0x0000000f
#define PA_SU_SC_MODE_CNTL__VTX_WINDOW_OFFSET_ENABLE__SHIFT 0x00000010
#define PA_SU_SC_MODE_CNTL__LINE_STIPPLE_ENABLE__SHIFT     0x00000012
#define PA_SU_SC_MODE_CNTL__PROVOKING_VTX_LAST__SHIFT      0x00000013
#define PA_SU_SC_MODE_CNTL__PERSP_CORR_DIS__SHIFT          0x00000014
#define PA_SU_SC_MODE_CNTL__MULTI_PRIM_IB_ENA__SHIFT       0x00000015
#define PA_SU_SC_MODE_CNTL__QUAD_ORDER_ENABLE__SHIFT       0x00000017
#define PA_SU_SC_MODE_CNTL__WAIT_RB_IDLE_ALL_TRI__SHIFT    0x00000019
#define PA_SU_SC_MODE_CNTL__WAIT_RB_IDLE_FIRST_TRI_NEW_STATE__SHIFT 0x0000001a
#define PA_SU_SC_MODE_CNTL__ZERO_AREA_FACENESS__SHIFT      0x0000001d
#define PA_SU_SC_MODE_CNTL__FACE_KILL_ENABLE__SHIFT        0x0000001e
#define PA_SU_SC_MODE_CNTL__FACE_WRITE_ENABLE__SHIFT       0x0000001f

// PA_SU_POLY_OFFSET_FRONT_SCALE
#define PA_SU_POLY_OFFSET_FRONT_SCALE__SCALE__SHIFT        0x00000000

// PA_SU_POLY_OFFSET_FRONT_OFFSET
#define PA_SU_POLY_OFFSET_FRONT_OFFSET__OFFSET__SHIFT      0x00000000

// PA_SU_POLY_OFFSET_BACK_SCALE
#define PA_SU_POLY_OFFSET_BACK_SCALE__SCALE__SHIFT         0x00000000

// PA_SU_POLY_OFFSET_BACK_OFFSET
#define PA_SU_POLY_OFFSET_BACK_OFFSET__OFFSET__SHIFT       0x00000000

// PA_SU_PERFCOUNTER0_SELECT
#define PA_SU_PERFCOUNTER0_SELECT__PERF_SEL__SHIFT         0x00000000

// PA_SU_PERFCOUNTER1_SELECT
#define PA_SU_PERFCOUNTER1_SELECT__PERF_SEL__SHIFT         0x00000000

// PA_SU_PERFCOUNTER2_SELECT
#define PA_SU_PERFCOUNTER2_SELECT__PERF_SEL__SHIFT         0x00000000

// PA_SU_PERFCOUNTER3_SELECT
#define PA_SU_PERFCOUNTER3_SELECT__PERF_SEL__SHIFT         0x00000000

// PA_SU_PERFCOUNTER0_LOW
#define PA_SU_PERFCOUNTER0_LOW__PERF_COUNT__SHIFT          0x00000000

// PA_SU_PERFCOUNTER0_HI
#define PA_SU_PERFCOUNTER0_HI__PERF_COUNT__SHIFT           0x00000000

// PA_SU_PERFCOUNTER1_LOW
#define PA_SU_PERFCOUNTER1_LOW__PERF_COUNT__SHIFT          0x00000000

// PA_SU_PERFCOUNTER1_HI
#define PA_SU_PERFCOUNTER1_HI__PERF_COUNT__SHIFT           0x00000000

// PA_SU_PERFCOUNTER2_LOW
#define PA_SU_PERFCOUNTER2_LOW__PERF_COUNT__SHIFT          0x00000000

// PA_SU_PERFCOUNTER2_HI
#define PA_SU_PERFCOUNTER2_HI__PERF_COUNT__SHIFT           0x00000000

// PA_SU_PERFCOUNTER3_LOW
#define PA_SU_PERFCOUNTER3_LOW__PERF_COUNT__SHIFT          0x00000000

// PA_SU_PERFCOUNTER3_HI
#define PA_SU_PERFCOUNTER3_HI__PERF_COUNT__SHIFT           0x00000000

// PA_SC_WINDOW_OFFSET
#define PA_SC_WINDOW_OFFSET__WINDOW_X_OFFSET__SHIFT        0x00000000
#define PA_SC_WINDOW_OFFSET__WINDOW_Y_OFFSET__SHIFT        0x00000010

// PA_SC_AA_CONFIG
#define PA_SC_AA_CONFIG__MSAA_NUM_SAMPLES__SHIFT           0x00000000
#define PA_SC_AA_CONFIG__MAX_SAMPLE_DIST__SHIFT            0x0000000d

// PA_SC_AA_MASK
#define PA_SC_AA_MASK__AA_MASK__SHIFT                      0x00000000

// PA_SC_LINE_STIPPLE
#define PA_SC_LINE_STIPPLE__LINE_PATTERN__SHIFT            0x00000000
#define PA_SC_LINE_STIPPLE__REPEAT_COUNT__SHIFT            0x00000010
#define PA_SC_LINE_STIPPLE__PATTERN_BIT_ORDER__SHIFT       0x0000001c
#define PA_SC_LINE_STIPPLE__AUTO_RESET_CNTL__SHIFT         0x0000001d

// PA_SC_LINE_CNTL
#define PA_SC_LINE_CNTL__BRES_CNTL__SHIFT                  0x00000000
#define PA_SC_LINE_CNTL__USE_BRES_CNTL__SHIFT              0x00000008
#define PA_SC_LINE_CNTL__EXPAND_LINE_WIDTH__SHIFT          0x00000009
#define PA_SC_LINE_CNTL__LAST_PIXEL__SHIFT                 0x0000000a

// PA_SC_WINDOW_SCISSOR_TL
#define PA_SC_WINDOW_SCISSOR_TL__TL_X__SHIFT               0x00000000
#define PA_SC_WINDOW_SCISSOR_TL__TL_Y__SHIFT               0x00000010
#define PA_SC_WINDOW_SCISSOR_TL__WINDOW_OFFSET_DISABLE__SHIFT 0x0000001f

// PA_SC_WINDOW_SCISSOR_BR
#define PA_SC_WINDOW_SCISSOR_BR__BR_X__SHIFT               0x00000000
#define PA_SC_WINDOW_SCISSOR_BR__BR_Y__SHIFT               0x00000010

// PA_SC_SCREEN_SCISSOR_TL
#define PA_SC_SCREEN_SCISSOR_TL__TL_X__SHIFT               0x00000000
#define PA_SC_SCREEN_SCISSOR_TL__TL_Y__SHIFT               0x00000010

// PA_SC_SCREEN_SCISSOR_BR
#define PA_SC_SCREEN_SCISSOR_BR__BR_X__SHIFT               0x00000000
#define PA_SC_SCREEN_SCISSOR_BR__BR_Y__SHIFT               0x00000010

// PA_SC_VIZ_QUERY
#define PA_SC_VIZ_QUERY__VIZ_QUERY_ENA__SHIFT              0x00000000
#define PA_SC_VIZ_QUERY__VIZ_QUERY_ID__SHIFT               0x00000001
#define PA_SC_VIZ_QUERY__KILL_PIX_POST_EARLY_Z__SHIFT      0x00000007

// PA_SC_VIZ_QUERY_STATUS
#define PA_SC_VIZ_QUERY_STATUS__STATUS_BITS__SHIFT         0x00000000

// PA_SC_LINE_STIPPLE_STATE
#define PA_SC_LINE_STIPPLE_STATE__CURRENT_PTR__SHIFT       0x00000000
#define PA_SC_LINE_STIPPLE_STATE__CURRENT_COUNT__SHIFT     0x00000008

// PA_SC_PERFCOUNTER0_SELECT
#define PA_SC_PERFCOUNTER0_SELECT__PERF_SEL__SHIFT         0x00000000

// PA_SC_PERFCOUNTER0_LOW
#define PA_SC_PERFCOUNTER0_LOW__PERF_COUNT__SHIFT          0x00000000

// PA_SC_PERFCOUNTER0_HI
#define PA_SC_PERFCOUNTER0_HI__PERF_COUNT__SHIFT           0x00000000

// PA_CL_CNTL_STATUS
#define PA_CL_CNTL_STATUS__CL_BUSY__SHIFT                  0x0000001f

// PA_SU_CNTL_STATUS
#define PA_SU_CNTL_STATUS__SU_BUSY__SHIFT                  0x0000001f

// PA_SC_CNTL_STATUS
#define PA_SC_CNTL_STATUS__SC_BUSY__SHIFT                  0x0000001f

// PA_SU_DEBUG_CNTL
#define PA_SU_DEBUG_CNTL__SU_DEBUG_INDX__SHIFT             0x00000000

// PA_SU_DEBUG_DATA
#define PA_SU_DEBUG_DATA__DATA__SHIFT                      0x00000000

// CLIPPER_DEBUG_REG00
#define CLIPPER_DEBUG_REG00__clip_ga_bc_fifo_write__SHIFT  0x00000000
#define CLIPPER_DEBUG_REG00__clip_ga_bc_fifo_full__SHIFT   0x00000001
#define CLIPPER_DEBUG_REG00__clip_to_ga_fifo_write__SHIFT  0x00000002
#define CLIPPER_DEBUG_REG00__clip_to_ga_fifo_full__SHIFT   0x00000003
#define CLIPPER_DEBUG_REG00__primic_to_clprim_fifo_empty__SHIFT 0x00000004
#define CLIPPER_DEBUG_REG00__primic_to_clprim_fifo_full__SHIFT 0x00000005
#define CLIPPER_DEBUG_REG00__clip_to_outsm_fifo_empty__SHIFT 0x00000006
#define CLIPPER_DEBUG_REG00__clip_to_outsm_fifo_full__SHIFT 0x00000007
#define CLIPPER_DEBUG_REG00__vgt_to_clipp_fifo_empty__SHIFT 0x00000008
#define CLIPPER_DEBUG_REG00__vgt_to_clipp_fifo_full__SHIFT 0x00000009
#define CLIPPER_DEBUG_REG00__vgt_to_clips_fifo_empty__SHIFT 0x0000000a
#define CLIPPER_DEBUG_REG00__vgt_to_clips_fifo_full__SHIFT 0x0000000b
#define CLIPPER_DEBUG_REG00__clipcode_fifo_fifo_empty__SHIFT 0x0000000c
#define CLIPPER_DEBUG_REG00__clipcode_fifo_full__SHIFT     0x0000000d
#define CLIPPER_DEBUG_REG00__vte_out_clip_fifo_fifo_empty__SHIFT 0x0000000e
#define CLIPPER_DEBUG_REG00__vte_out_clip_fifo_fifo_full__SHIFT 0x0000000f
#define CLIPPER_DEBUG_REG00__vte_out_orig_fifo_fifo_empty__SHIFT 0x00000010
#define CLIPPER_DEBUG_REG00__vte_out_orig_fifo_fifo_full__SHIFT 0x00000011
#define CLIPPER_DEBUG_REG00__ccgen_to_clipcc_fifo_empty__SHIFT 0x00000012
#define CLIPPER_DEBUG_REG00__ccgen_to_clipcc_fifo_full__SHIFT 0x00000013
#define CLIPPER_DEBUG_REG00__ALWAYS_ZERO__SHIFT            0x00000014

// CLIPPER_DEBUG_REG01
#define CLIPPER_DEBUG_REG01__clip_to_outsm_end_of_packet__SHIFT 0x00000000
#define CLIPPER_DEBUG_REG01__clip_to_outsm_first_prim_of_slot__SHIFT 0x00000001
#define CLIPPER_DEBUG_REG01__clip_to_outsm_deallocate_slot__SHIFT 0x00000002
#define CLIPPER_DEBUG_REG01__clip_to_outsm_clipped_prim__SHIFT 0x00000005
#define CLIPPER_DEBUG_REG01__clip_to_outsm_null_primitive__SHIFT 0x00000006
#define CLIPPER_DEBUG_REG01__clip_to_outsm_vertex_store_indx_2__SHIFT 0x00000007
#define CLIPPER_DEBUG_REG01__clip_to_outsm_vertex_store_indx_1__SHIFT 0x0000000b
#define CLIPPER_DEBUG_REG01__clip_to_outsm_vertex_store_indx_0__SHIFT 0x0000000f
#define CLIPPER_DEBUG_REG01__clip_vert_vte_valid__SHIFT    0x00000013
#define CLIPPER_DEBUG_REG01__vte_out_clip_rd_vertex_store_indx__SHIFT 0x00000016
#define CLIPPER_DEBUG_REG01__ALWAYS_ZERO__SHIFT            0x00000018

// CLIPPER_DEBUG_REG02
#define CLIPPER_DEBUG_REG02__ALWAYS_ZERO1__SHIFT           0x00000000
#define CLIPPER_DEBUG_REG02__clipsm0_clip_to_clipga_clip_to_outsm_cnt__SHIFT 0x00000015
#define CLIPPER_DEBUG_REG02__ALWAYS_ZERO0__SHIFT           0x00000018
#define CLIPPER_DEBUG_REG02__clipsm0_clprim_to_clip_prim_valid__SHIFT 0x0000001f

// CLIPPER_DEBUG_REG03
#define CLIPPER_DEBUG_REG03__ALWAYS_ZERO3__SHIFT           0x00000000
#define CLIPPER_DEBUG_REG03__clipsm0_clprim_to_clip_clip_primitive__SHIFT 0x00000003
#define CLIPPER_DEBUG_REG03__ALWAYS_ZERO2__SHIFT           0x00000004
#define CLIPPER_DEBUG_REG03__clipsm0_clprim_to_clip_null_primitive__SHIFT 0x00000007
#define CLIPPER_DEBUG_REG03__ALWAYS_ZERO1__SHIFT           0x00000008
#define CLIPPER_DEBUG_REG03__clipsm0_clprim_to_clip_clip_code_or__SHIFT 0x00000014
#define CLIPPER_DEBUG_REG03__ALWAYS_ZERO0__SHIFT           0x0000001a

// CLIPPER_DEBUG_REG04
#define CLIPPER_DEBUG_REG04__ALWAYS_ZERO2__SHIFT           0x00000000
#define CLIPPER_DEBUG_REG04__clipsm0_clprim_to_clip_first_prim_of_slot__SHIFT 0x00000003
#define CLIPPER_DEBUG_REG04__ALWAYS_ZERO1__SHIFT           0x00000004
#define CLIPPER_DEBUG_REG04__clipsm0_clprim_to_clip_event__SHIFT 0x00000007
#define CLIPPER_DEBUG_REG04__ALWAYS_ZERO0__SHIFT           0x00000008

// CLIPPER_DEBUG_REG05
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_state_var_indx__SHIFT 0x00000000
#define CLIPPER_DEBUG_REG05__ALWAYS_ZERO3__SHIFT           0x00000001
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_deallocate_slot__SHIFT 0x00000003
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_event_id__SHIFT 0x00000006
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_vertex_store_indx_2__SHIFT 0x0000000c
#define CLIPPER_DEBUG_REG05__ALWAYS_ZERO2__SHIFT           0x00000010
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_vertex_store_indx_1__SHIFT 0x00000012
#define CLIPPER_DEBUG_REG05__ALWAYS_ZERO1__SHIFT           0x00000016
#define CLIPPER_DEBUG_REG05__clipsm0_clprim_to_clip_vertex_store_indx_0__SHIFT 0x00000018
#define CLIPPER_DEBUG_REG05__ALWAYS_ZERO0__SHIFT           0x0000001c

// CLIPPER_DEBUG_REG09
#define CLIPPER_DEBUG_REG09__clprim_in_back_event__SHIFT   0x00000000
#define CLIPPER_DEBUG_REG09__outputclprimtoclip_null_primitive__SHIFT 0x00000001
#define CLIPPER_DEBUG_REG09__clprim_in_back_vertex_store_indx_2__SHIFT 0x00000002
#define CLIPPER_DEBUG_REG09__ALWAYS_ZERO2__SHIFT           0x00000006
#define CLIPPER_DEBUG_REG09__clprim_in_back_vertex_store_indx_1__SHIFT 0x00000008
#define CLIPPER_DEBUG_REG09__ALWAYS_ZERO1__SHIFT           0x0000000c
#define CLIPPER_DEBUG_REG09__clprim_in_back_vertex_store_indx_0__SHIFT 0x0000000e
#define CLIPPER_DEBUG_REG09__ALWAYS_ZERO0__SHIFT           0x00000012
#define CLIPPER_DEBUG_REG09__prim_back_valid__SHIFT        0x00000014
#define CLIPPER_DEBUG_REG09__clip_priority_seq_indx_out_cnt__SHIFT 0x00000015
#define CLIPPER_DEBUG_REG09__outsm_clr_rd_orig_vertices__SHIFT 0x00000019
#define CLIPPER_DEBUG_REG09__outsm_clr_rd_clipsm_wait__SHIFT 0x0000001b
#define CLIPPER_DEBUG_REG09__outsm_clr_fifo_empty__SHIFT   0x0000001c
#define CLIPPER_DEBUG_REG09__outsm_clr_fifo_full__SHIFT    0x0000001d
#define CLIPPER_DEBUG_REG09__clip_priority_seq_indx_load__SHIFT 0x0000001e

// CLIPPER_DEBUG_REG10
#define CLIPPER_DEBUG_REG10__primic_to_clprim_fifo_vertex_store_indx_2__SHIFT 0x00000000
#define CLIPPER_DEBUG_REG10__ALWAYS_ZERO3__SHIFT           0x00000004
#define CLIPPER_DEBUG_REG10__primic_to_clprim_fifo_vertex_store_indx_1__SHIFT 0x00000006
#define CLIPPER_DEBUG_REG10__ALWAYS_ZERO2__SHIFT           0x0000000a
#define CLIPPER_DEBUG_REG10__primic_to_clprim_fifo_vertex_store_indx_0__SHIFT 0x0000000c
#define CLIPPER_DEBUG_REG10__ALWAYS_ZERO1__SHIFT           0x00000010
#define CLIPPER_DEBUG_REG10__clprim_in_back_state_var_indx__SHIFT 0x00000012
#define CLIPPER_DEBUG_REG10__ALWAYS_ZERO0__SHIFT           0x00000013
#define CLIPPER_DEBUG_REG10__clprim_in_back_end_of_packet__SHIFT 0x00000015
#define CLIPPER_DEBUG_REG10__clprim_in_back_first_prim_of_slot__SHIFT 0x00000016
#define CLIPPER_DEBUG_REG10__clprim_in_back_deallocate_slot__SHIFT 0x00000017
#define CLIPPER_DEBUG_REG10__clprim_in_back_event_id__SHIFT 0x0000001a

// CLIPPER_DEBUG_REG11
#define CLIPPER_DEBUG_REG11__vertval_bits_vertex_vertex_store_msb__SHIFT 0x00000000
#define CLIPPER_DEBUG_REG11__ALWAYS_ZERO__SHIFT            0x00000004

// CLIPPER_DEBUG_REG12
#define CLIPPER_DEBUG_REG12__clip_priority_available_vte_out_clip__SHIFT 0x00000000
#define CLIPPER_DEBUG_REG12__ALWAYS_ZERO2__SHIFT           0x00000002
#define CLIPPER_DEBUG_REG12__clip_vertex_fifo_empty__SHIFT 0x00000005
#define CLIPPER_DEBUG_REG12__clip_priority_available_clip_verts__SHIFT 0x00000006
#define CLIPPER_DEBUG_REG12__ALWAYS_ZERO1__SHIFT           0x0000000b
#define CLIPPER_DEBUG_REG12__vertval_bits_vertex_cc_next_valid__SHIFT 0x0000000f
#define CLIPPER_DEBUG_REG12__clipcc_vertex_store_indx__SHIFT 0x00000013
#define CLIPPER_DEBUG_REG12__primic_to_clprim_valid__SHIFT 0x00000015
#define CLIPPER_DEBUG_REG12__ALWAYS_ZERO0__SHIFT           0x00000016

// CLIPPER_DEBUG_REG13
#define CLIPPER_DEBUG_REG13__sm0_clip_vert_cnt__SHIFT      0x00000000
#define CLIPPER_DEBUG_REG13__sm0_prim_end_state__SHIFT     0x00000004
#define CLIPPER_DEBUG_REG13__ALWAYS_ZERO1__SHIFT           0x0000000b
#define CLIPPER_DEBUG_REG13__sm0_vertex_clip_cnt__SHIFT    0x0000000e
#define CLIPPER_DEBUG_REG13__sm0_inv_to_clip_data_valid_1__SHIFT 0x00000012
#define CLIPPER_DEBUG_REG13__sm0_inv_to_clip_data_valid_0__SHIFT 0x00000013
#define CLIPPER_DEBUG_REG13__sm0_current_state__SHIFT      0x00000014
#define CLIPPER_DEBUG_REG13__ALWAYS_ZERO0__SHIFT           0x0000001b

// SXIFCCG_DEBUG_REG0
#define SXIFCCG_DEBUG_REG0__nan_kill_flag__SHIFT           0x00000000
#define SXIFCCG_DEBUG_REG0__position_address__SHIFT        0x00000004
#define SXIFCCG_DEBUG_REG0__ALWAYS_ZERO2__SHIFT            0x00000007
#define SXIFCCG_DEBUG_REG0__point_address__SHIFT           0x0000000a
#define SXIFCCG_DEBUG_REG0__ALWAYS_ZERO1__SHIFT            0x0000000d
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_state_var_indx__SHIFT 0x00000010
#define SXIFCCG_DEBUG_REG0__ALWAYS_ZERO0__SHIFT            0x00000011
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_req_mask__SHIFT  0x00000013
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_pci__SHIFT       0x00000017
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_aux_inc__SHIFT   0x0000001e
#define SXIFCCG_DEBUG_REG0__sx_pending_rd_aux_sel__SHIFT   0x0000001f

// SXIFCCG_DEBUG_REG1
#define SXIFCCG_DEBUG_REG1__ALWAYS_ZERO3__SHIFT            0x00000000
#define SXIFCCG_DEBUG_REG1__sx_to_pa_empty__SHIFT          0x00000002
#define SXIFCCG_DEBUG_REG1__available_positions__SHIFT     0x00000004
#define SXIFCCG_DEBUG_REG1__ALWAYS_ZERO2__SHIFT            0x00000007
#define SXIFCCG_DEBUG_REG1__sx_pending_advance__SHIFT      0x0000000b
#define SXIFCCG_DEBUG_REG1__sx_receive_indx__SHIFT         0x0000000c
#define SXIFCCG_DEBUG_REG1__statevar_bits_sxpa_aux_vector__SHIFT 0x0000000f
#define SXIFCCG_DEBUG_REG1__ALWAYS_ZERO1__SHIFT            0x00000010
#define SXIFCCG_DEBUG_REG1__aux_sel__SHIFT                 0x00000014
#define SXIFCCG_DEBUG_REG1__ALWAYS_ZERO0__SHIFT            0x00000015
#define SXIFCCG_DEBUG_REG1__pasx_req_cnt__SHIFT            0x00000017
#define SXIFCCG_DEBUG_REG1__param_cache_base__SHIFT        0x00000019

// SXIFCCG_DEBUG_REG2
#define SXIFCCG_DEBUG_REG2__sx_sent__SHIFT                 0x00000000
#define SXIFCCG_DEBUG_REG2__ALWAYS_ZERO3__SHIFT            0x00000001
#define SXIFCCG_DEBUG_REG2__sx_aux__SHIFT                  0x00000002
#define SXIFCCG_DEBUG_REG2__sx_request_indx__SHIFT         0x00000003
#define SXIFCCG_DEBUG_REG2__req_active_verts__SHIFT        0x00000009
#define SXIFCCG_DEBUG_REG2__ALWAYS_ZERO2__SHIFT            0x00000010
#define SXIFCCG_DEBUG_REG2__vgt_to_ccgen_state_var_indx__SHIFT 0x00000011
#define SXIFCCG_DEBUG_REG2__ALWAYS_ZERO1__SHIFT            0x00000012
#define SXIFCCG_DEBUG_REG2__vgt_to_ccgen_active_verts__SHIFT 0x00000014
#define SXIFCCG_DEBUG_REG2__ALWAYS_ZERO0__SHIFT            0x00000016
#define SXIFCCG_DEBUG_REG2__req_active_verts_loaded__SHIFT 0x0000001a
#define SXIFCCG_DEBUG_REG2__sx_pending_fifo_empty__SHIFT   0x0000001b
#define SXIFCCG_DEBUG_REG2__sx_pending_fifo_full__SHIFT    0x0000001c
#define SXIFCCG_DEBUG_REG2__sx_pending_fifo_contents__SHIFT 0x0000001d

// SXIFCCG_DEBUG_REG3
#define SXIFCCG_DEBUG_REG3__vertex_fifo_entriesavailable__SHIFT 0x00000000
#define SXIFCCG_DEBUG_REG3__ALWAYS_ZERO3__SHIFT            0x00000004
#define SXIFCCG_DEBUG_REG3__available_positions__SHIFT     0x00000005
#define SXIFCCG_DEBUG_REG3__ALWAYS_ZERO2__SHIFT            0x00000008
#define SXIFCCG_DEBUG_REG3__current_state__SHIFT           0x0000000c
#define SXIFCCG_DEBUG_REG3__vertex_fifo_empty__SHIFT       0x0000000e
#define SXIFCCG_DEBUG_REG3__vertex_fifo_full__SHIFT        0x0000000f
#define SXIFCCG_DEBUG_REG3__ALWAYS_ZERO1__SHIFT            0x00000010
#define SXIFCCG_DEBUG_REG3__sx0_receive_fifo_empty__SHIFT  0x00000012
#define SXIFCCG_DEBUG_REG3__sx0_receive_fifo_full__SHIFT   0x00000013
#define SXIFCCG_DEBUG_REG3__vgt_to_ccgen_fifo_empty__SHIFT 0x00000014
#define SXIFCCG_DEBUG_REG3__vgt_to_ccgen_fifo_full__SHIFT  0x00000015
#define SXIFCCG_DEBUG_REG3__ALWAYS_ZERO0__SHIFT            0x00000016

// SETUP_DEBUG_REG0
#define SETUP_DEBUG_REG0__su_cntl_state__SHIFT             0x00000000
#define SETUP_DEBUG_REG0__pmode_state__SHIFT               0x00000005
#define SETUP_DEBUG_REG0__ge_stallb__SHIFT                 0x0000000b
#define SETUP_DEBUG_REG0__geom_enable__SHIFT               0x0000000c
#define SETUP_DEBUG_REG0__su_clip_baryc_rtr__SHIFT         0x0000000d
#define SETUP_DEBUG_REG0__su_clip_rtr__SHIFT               0x0000000e
#define SETUP_DEBUG_REG0__pfifo_busy__SHIFT                0x0000000f
#define SETUP_DEBUG_REG0__su_cntl_busy__SHIFT              0x00000010
#define SETUP_DEBUG_REG0__geom_busy__SHIFT                 0x00000011

// SETUP_DEBUG_REG1
#define SETUP_DEBUG_REG1__y_sort0_gated_17_4__SHIFT        0x00000000
#define SETUP_DEBUG_REG1__x_sort0_gated_17_4__SHIFT        0x0000000e

// SETUP_DEBUG_REG2
#define SETUP_DEBUG_REG2__y_sort1_gated_17_4__SHIFT        0x00000000
#define SETUP_DEBUG_REG2__x_sort1_gated_17_4__SHIFT        0x0000000e

// SETUP_DEBUG_REG3
#define SETUP_DEBUG_REG3__y_sort2_gated_17_4__SHIFT        0x00000000
#define SETUP_DEBUG_REG3__x_sort2_gated_17_4__SHIFT        0x0000000e

// SETUP_DEBUG_REG4
#define SETUP_DEBUG_REG4__attr_indx_sort0_gated__SHIFT     0x00000000
#define SETUP_DEBUG_REG4__null_prim_gated__SHIFT           0x0000000b
#define SETUP_DEBUG_REG4__backfacing_gated__SHIFT          0x0000000c
#define SETUP_DEBUG_REG4__st_indx_gated__SHIFT             0x0000000d
#define SETUP_DEBUG_REG4__clipped_gated__SHIFT             0x00000010
#define SETUP_DEBUG_REG4__dealloc_slot_gated__SHIFT        0x00000011
#define SETUP_DEBUG_REG4__xmajor_gated__SHIFT              0x00000014
#define SETUP_DEBUG_REG4__diamond_rule_gated__SHIFT        0x00000015
#define SETUP_DEBUG_REG4__type_gated__SHIFT                0x00000017
#define SETUP_DEBUG_REG4__fpov_gated__SHIFT                0x0000001a
#define SETUP_DEBUG_REG4__pmode_prim_gated__SHIFT          0x0000001b
#define SETUP_DEBUG_REG4__event_gated__SHIFT               0x0000001c
#define SETUP_DEBUG_REG4__eop_gated__SHIFT                 0x0000001d

// SETUP_DEBUG_REG5
#define SETUP_DEBUG_REG5__attr_indx_sort2_gated__SHIFT     0x00000000
#define SETUP_DEBUG_REG5__attr_indx_sort1_gated__SHIFT     0x0000000b
#define SETUP_DEBUG_REG5__provoking_vtx_gated__SHIFT       0x00000016
#define SETUP_DEBUG_REG5__event_id_gated__SHIFT            0x00000018

// PA_SC_DEBUG_CNTL
#define PA_SC_DEBUG_CNTL__SC_DEBUG_INDX__SHIFT             0x00000000

// PA_SC_DEBUG_DATA
#define PA_SC_DEBUG_DATA__DATA__SHIFT                      0x00000000

// SC_DEBUG_0
#define SC_DEBUG_0__pa_freeze_b1__SHIFT                    0x00000000
#define SC_DEBUG_0__pa_sc_valid__SHIFT                     0x00000001
#define SC_DEBUG_0__pa_sc_phase__SHIFT                     0x00000002
#define SC_DEBUG_0__cntx_cnt__SHIFT                        0x00000005
#define SC_DEBUG_0__decr_cntx_cnt__SHIFT                   0x0000000c
#define SC_DEBUG_0__incr_cntx_cnt__SHIFT                   0x0000000d
#define SC_DEBUG_0__trigger__SHIFT                         0x0000001f

// SC_DEBUG_1
#define SC_DEBUG_1__em_state__SHIFT                        0x00000000
#define SC_DEBUG_1__em1_data_ready__SHIFT                  0x00000003
#define SC_DEBUG_1__em2_data_ready__SHIFT                  0x00000004
#define SC_DEBUG_1__move_em1_to_em2__SHIFT                 0x00000005
#define SC_DEBUG_1__ef_data_ready__SHIFT                   0x00000006
#define SC_DEBUG_1__ef_state__SHIFT                        0x00000007
#define SC_DEBUG_1__pipe_valid__SHIFT                      0x00000009
#define SC_DEBUG_1__trigger__SHIFT                         0x0000001f

// SC_DEBUG_2
#define SC_DEBUG_2__rc_rtr_dly__SHIFT                      0x00000000
#define SC_DEBUG_2__qmask_ff_alm_full_d1__SHIFT            0x00000001
#define SC_DEBUG_2__pipe_freeze_b__SHIFT                   0x00000003
#define SC_DEBUG_2__prim_rts__SHIFT                        0x00000004
#define SC_DEBUG_2__next_prim_rts_dly__SHIFT               0x00000005
#define SC_DEBUG_2__next_prim_rtr_dly__SHIFT               0x00000006
#define SC_DEBUG_2__pre_stage1_rts_d1__SHIFT               0x00000007
#define SC_DEBUG_2__stage0_rts__SHIFT                      0x00000008
#define SC_DEBUG_2__phase_rts_dly__SHIFT                   0x00000009
#define SC_DEBUG_2__end_of_prim_s1_dly__SHIFT              0x0000000f
#define SC_DEBUG_2__pass_empty_prim_s1__SHIFT              0x00000010
#define SC_DEBUG_2__event_id_s1__SHIFT                     0x00000011
#define SC_DEBUG_2__event_s1__SHIFT                        0x00000016
#define SC_DEBUG_2__trigger__SHIFT                         0x0000001f

// SC_DEBUG_3
#define SC_DEBUG_3__x_curr_s1__SHIFT                       0x00000000
#define SC_DEBUG_3__y_curr_s1__SHIFT                       0x0000000b
#define SC_DEBUG_3__trigger__SHIFT                         0x0000001f

// SC_DEBUG_4
#define SC_DEBUG_4__y_end_s1__SHIFT                        0x00000000
#define SC_DEBUG_4__y_start_s1__SHIFT                      0x0000000e
#define SC_DEBUG_4__y_dir_s1__SHIFT                        0x0000001c
#define SC_DEBUG_4__trigger__SHIFT                         0x0000001f

// SC_DEBUG_5
#define SC_DEBUG_5__x_end_s1__SHIFT                        0x00000000
#define SC_DEBUG_5__x_start_s1__SHIFT                      0x0000000e
#define SC_DEBUG_5__x_dir_s1__SHIFT                        0x0000001c
#define SC_DEBUG_5__trigger__SHIFT                         0x0000001f

// SC_DEBUG_6
#define SC_DEBUG_6__z_ff_empty__SHIFT                      0x00000000
#define SC_DEBUG_6__qmcntl_ff_empty__SHIFT                 0x00000001
#define SC_DEBUG_6__xy_ff_empty__SHIFT                     0x00000002
#define SC_DEBUG_6__event_flag__SHIFT                      0x00000003
#define SC_DEBUG_6__z_mask_needed__SHIFT                   0x00000004
#define SC_DEBUG_6__state__SHIFT                           0x00000005
#define SC_DEBUG_6__state_delayed__SHIFT                   0x00000008
#define SC_DEBUG_6__data_valid__SHIFT                      0x0000000b
#define SC_DEBUG_6__data_valid_d__SHIFT                    0x0000000c
#define SC_DEBUG_6__tilex_delayed__SHIFT                   0x0000000d
#define SC_DEBUG_6__tiley_delayed__SHIFT                   0x00000016
#define SC_DEBUG_6__trigger__SHIFT                         0x0000001f

// SC_DEBUG_7
#define SC_DEBUG_7__event_flag__SHIFT                      0x00000000
#define SC_DEBUG_7__deallocate__SHIFT                      0x00000001
#define SC_DEBUG_7__fposition__SHIFT                       0x00000004
#define SC_DEBUG_7__sr_prim_we__SHIFT                      0x00000005
#define SC_DEBUG_7__last_tile__SHIFT                       0x00000006
#define SC_DEBUG_7__tile_ff_we__SHIFT                      0x00000007
#define SC_DEBUG_7__qs_data_valid__SHIFT                   0x00000008
#define SC_DEBUG_7__qs_q0_y__SHIFT                         0x00000009
#define SC_DEBUG_7__qs_q0_x__SHIFT                         0x0000000b
#define SC_DEBUG_7__qs_q0_valid__SHIFT                     0x0000000d
#define SC_DEBUG_7__prim_ff_we__SHIFT                      0x0000000e
#define SC_DEBUG_7__tile_ff_re__SHIFT                      0x0000000f
#define SC_DEBUG_7__fw_prim_data_valid__SHIFT              0x00000010
#define SC_DEBUG_7__last_quad_of_tile__SHIFT               0x00000011
#define SC_DEBUG_7__first_quad_of_tile__SHIFT              0x00000012
#define SC_DEBUG_7__first_quad_of_prim__SHIFT              0x00000013
#define SC_DEBUG_7__new_prim__SHIFT                        0x00000014
#define SC_DEBUG_7__load_new_tile_data__SHIFT              0x00000015
#define SC_DEBUG_7__state__SHIFT                           0x00000016
#define SC_DEBUG_7__fifos_ready__SHIFT                     0x00000018
#define SC_DEBUG_7__trigger__SHIFT                         0x0000001f

// SC_DEBUG_8
#define SC_DEBUG_8__sample_last__SHIFT                     0x00000000
#define SC_DEBUG_8__sample_mask__SHIFT                     0x00000001
#define SC_DEBUG_8__sample_y__SHIFT                        0x00000005
#define SC_DEBUG_8__sample_x__SHIFT                        0x00000007
#define SC_DEBUG_8__sample_send__SHIFT                     0x00000009
#define SC_DEBUG_8__next_cycle__SHIFT                      0x0000000a
#define SC_DEBUG_8__ez_sample_ff_full__SHIFT               0x0000000c
#define SC_DEBUG_8__rb_sc_samp_rtr__SHIFT                  0x0000000d
#define SC_DEBUG_8__num_samples__SHIFT                     0x0000000e
#define SC_DEBUG_8__last_quad_of_tile__SHIFT               0x00000010
#define SC_DEBUG_8__last_quad_of_prim__SHIFT               0x00000011
#define SC_DEBUG_8__first_quad_of_prim__SHIFT              0x00000012
#define SC_DEBUG_8__sample_we__SHIFT                       0x00000013
#define SC_DEBUG_8__fposition__SHIFT                       0x00000014
#define SC_DEBUG_8__event_id__SHIFT                        0x00000015
#define SC_DEBUG_8__event_flag__SHIFT                      0x0000001a
#define SC_DEBUG_8__fw_prim_data_valid__SHIFT              0x0000001b
#define SC_DEBUG_8__trigger__SHIFT                         0x0000001f

// SC_DEBUG_9
#define SC_DEBUG_9__rb_sc_send__SHIFT                      0x00000000
#define SC_DEBUG_9__rb_sc_ez_mask__SHIFT                   0x00000001
#define SC_DEBUG_9__fifo_data_ready__SHIFT                 0x00000005
#define SC_DEBUG_9__early_z_enable__SHIFT                  0x00000006
#define SC_DEBUG_9__mask_state__SHIFT                      0x00000007
#define SC_DEBUG_9__next_ez_mask__SHIFT                    0x00000009
#define SC_DEBUG_9__mask_ready__SHIFT                      0x00000019
#define SC_DEBUG_9__drop_sample__SHIFT                     0x0000001a
#define SC_DEBUG_9__fetch_new_sample_data__SHIFT           0x0000001b
#define SC_DEBUG_9__fetch_new_ez_sample_mask__SHIFT        0x0000001c
#define SC_DEBUG_9__pkr_fetch_new_sample_data__SHIFT       0x0000001d
#define SC_DEBUG_9__pkr_fetch_new_prim_data__SHIFT         0x0000001e
#define SC_DEBUG_9__trigger__SHIFT                         0x0000001f

// SC_DEBUG_10
#define SC_DEBUG_10__combined_sample_mask__SHIFT           0x00000000
#define SC_DEBUG_10__trigger__SHIFT                        0x0000001f

// SC_DEBUG_11
#define SC_DEBUG_11__ez_sample_data_ready__SHIFT           0x00000000
#define SC_DEBUG_11__pkr_fetch_new_sample_data__SHIFT      0x00000001
#define SC_DEBUG_11__ez_prim_data_ready__SHIFT             0x00000002
#define SC_DEBUG_11__pkr_fetch_new_prim_data__SHIFT        0x00000003
#define SC_DEBUG_11__iterator_input_fz__SHIFT              0x00000004
#define SC_DEBUG_11__packer_send_quads__SHIFT              0x00000005
#define SC_DEBUG_11__packer_send_cmd__SHIFT                0x00000006
#define SC_DEBUG_11__packer_send_event__SHIFT              0x00000007
#define SC_DEBUG_11__next_state__SHIFT                     0x00000008
#define SC_DEBUG_11__state__SHIFT                          0x0000000b
#define SC_DEBUG_11__stall__SHIFT                          0x0000000e
#define SC_DEBUG_11__trigger__SHIFT                        0x0000001f

// SC_DEBUG_12
#define SC_DEBUG_12__SQ_iterator_free_buff__SHIFT          0x00000000
#define SC_DEBUG_12__event_id__SHIFT                       0x00000001
#define SC_DEBUG_12__event_flag__SHIFT                     0x00000006
#define SC_DEBUG_12__itercmdfifo_busy_nc_dly__SHIFT        0x00000007
#define SC_DEBUG_12__itercmdfifo_full__SHIFT               0x00000008
#define SC_DEBUG_12__itercmdfifo_empty__SHIFT              0x00000009
#define SC_DEBUG_12__iter_ds_one_clk_command__SHIFT        0x0000000a
#define SC_DEBUG_12__iter_ds_end_of_prim0__SHIFT           0x0000000b
#define SC_DEBUG_12__iter_ds_end_of_vector__SHIFT          0x0000000c
#define SC_DEBUG_12__iter_qdhit0__SHIFT                    0x0000000d
#define SC_DEBUG_12__bc_use_centers_reg__SHIFT             0x0000000e
#define SC_DEBUG_12__bc_output_xy_reg__SHIFT               0x0000000f
#define SC_DEBUG_12__iter_phase_out__SHIFT                 0x00000010
#define SC_DEBUG_12__iter_phase_reg__SHIFT                 0x00000012
#define SC_DEBUG_12__iterator_SP_valid__SHIFT              0x00000014
#define SC_DEBUG_12__eopv_reg__SHIFT                       0x00000015
#define SC_DEBUG_12__one_clk_cmd_reg__SHIFT                0x00000016
#define SC_DEBUG_12__iter_dx_end_of_prim__SHIFT            0x00000017
#define SC_DEBUG_12__trigger__SHIFT                        0x0000001f

// GFX_COPY_STATE
#define GFX_COPY_STATE__SRC_STATE_ID__SHIFT                0x00000000

// VGT_DRAW_INITIATOR
#define VGT_DRAW_INITIATOR__PRIM_TYPE__SHIFT               0x00000000
#define VGT_DRAW_INITIATOR__SOURCE_SELECT__SHIFT           0x00000006
#define VGT_DRAW_INITIATOR__FACENESS_CULL_SELECT__SHIFT    0x00000008
#define VGT_DRAW_INITIATOR__INDEX_SIZE__SHIFT              0x0000000b
#define VGT_DRAW_INITIATOR__NOT_EOP__SHIFT                 0x0000000c
#define VGT_DRAW_INITIATOR__SMALL_INDEX__SHIFT             0x0000000d
#define VGT_DRAW_INITIATOR__PRE_FETCH_CULL_ENABLE__SHIFT   0x0000000e
#define VGT_DRAW_INITIATOR__GRP_CULL_ENABLE__SHIFT         0x0000000f
#define VGT_DRAW_INITIATOR__NUM_INDICES__SHIFT             0x00000010

// VGT_EVENT_INITIATOR
#define VGT_EVENT_INITIATOR__EVENT_TYPE__SHIFT             0x00000000

// VGT_DMA_BASE
#define VGT_DMA_BASE__BASE_ADDR__SHIFT                     0x00000000

// VGT_DMA_SIZE
#define VGT_DMA_SIZE__NUM_WORDS__SHIFT                     0x00000000
#define VGT_DMA_SIZE__SWAP_MODE__SHIFT                     0x0000001e

// VGT_BIN_BASE
#define VGT_BIN_BASE__BIN_BASE_ADDR__SHIFT                 0x00000000

// VGT_BIN_SIZE
#define VGT_BIN_SIZE__NUM_WORDS__SHIFT                     0x00000000
#define VGT_BIN_SIZE__FACENESS_FETCH__SHIFT                0x0000001e
#define VGT_BIN_SIZE__FACENESS_RESET__SHIFT                0x0000001f

// VGT_CURRENT_BIN_ID_MIN
#define VGT_CURRENT_BIN_ID_MIN__COLUMN__SHIFT              0x00000000
#define VGT_CURRENT_BIN_ID_MIN__ROW__SHIFT                 0x00000003
#define VGT_CURRENT_BIN_ID_MIN__GUARD_BAND__SHIFT          0x00000006

// VGT_CURRENT_BIN_ID_MAX
#define VGT_CURRENT_BIN_ID_MAX__COLUMN__SHIFT              0x00000000
#define VGT_CURRENT_BIN_ID_MAX__ROW__SHIFT                 0x00000003
#define VGT_CURRENT_BIN_ID_MAX__GUARD_BAND__SHIFT          0x00000006

// VGT_IMMED_DATA
#define VGT_IMMED_DATA__DATA__SHIFT                        0x00000000

// VGT_MAX_VTX_INDX
#define VGT_MAX_VTX_INDX__MAX_INDX__SHIFT                  0x00000000

// VGT_MIN_VTX_INDX
#define VGT_MIN_VTX_INDX__MIN_INDX__SHIFT                  0x00000000

// VGT_INDX_OFFSET
#define VGT_INDX_OFFSET__INDX_OFFSET__SHIFT                0x00000000

// VGT_VERTEX_REUSE_BLOCK_CNTL
#define VGT_VERTEX_REUSE_BLOCK_CNTL__VTX_REUSE_DEPTH__SHIFT 0x00000000

// VGT_OUT_DEALLOC_CNTL
#define VGT_OUT_DEALLOC_CNTL__DEALLOC_DIST__SHIFT          0x00000000

// VGT_MULTI_PRIM_IB_RESET_INDX
#define VGT_MULTI_PRIM_IB_RESET_INDX__RESET_INDX__SHIFT    0x00000000

// VGT_ENHANCE
#define VGT_ENHANCE__MISC__SHIFT                           0x00000000

// VGT_VTX_VECT_EJECT_REG
#define VGT_VTX_VECT_EJECT_REG__PRIM_COUNT__SHIFT          0x00000000

// VGT_LAST_COPY_STATE
#define VGT_LAST_COPY_STATE__SRC_STATE_ID__SHIFT           0x00000000
#define VGT_LAST_COPY_STATE__DST_STATE_ID__SHIFT           0x00000010

// VGT_DEBUG_CNTL
#define VGT_DEBUG_CNTL__VGT_DEBUG_INDX__SHIFT              0x00000000

// VGT_DEBUG_DATA
#define VGT_DEBUG_DATA__DATA__SHIFT                        0x00000000

// VGT_CNTL_STATUS
#define VGT_CNTL_STATUS__VGT_BUSY__SHIFT                   0x00000000
#define VGT_CNTL_STATUS__VGT_DMA_BUSY__SHIFT               0x00000001
#define VGT_CNTL_STATUS__VGT_DMA_REQ_BUSY__SHIFT           0x00000002
#define VGT_CNTL_STATUS__VGT_GRP_BUSY__SHIFT               0x00000003
#define VGT_CNTL_STATUS__VGT_VR_BUSY__SHIFT                0x00000004
#define VGT_CNTL_STATUS__VGT_BIN_BUSY__SHIFT               0x00000005
#define VGT_CNTL_STATUS__VGT_PT_BUSY__SHIFT                0x00000006
#define VGT_CNTL_STATUS__VGT_OUT_BUSY__SHIFT               0x00000007
#define VGT_CNTL_STATUS__VGT_OUT_INDX_BUSY__SHIFT          0x00000008

// VGT_DEBUG_REG0
#define VGT_DEBUG_REG0__te_grp_busy__SHIFT                 0x00000000
#define VGT_DEBUG_REG0__pt_grp_busy__SHIFT                 0x00000001
#define VGT_DEBUG_REG0__vr_grp_busy__SHIFT                 0x00000002
#define VGT_DEBUG_REG0__dma_request_busy__SHIFT            0x00000003
#define VGT_DEBUG_REG0__out_busy__SHIFT                    0x00000004
#define VGT_DEBUG_REG0__grp_backend_busy__SHIFT            0x00000005
#define VGT_DEBUG_REG0__grp_busy__SHIFT                    0x00000006
#define VGT_DEBUG_REG0__dma_busy__SHIFT                    0x00000007
#define VGT_DEBUG_REG0__rbiu_dma_request_busy__SHIFT       0x00000008
#define VGT_DEBUG_REG0__rbiu_busy__SHIFT                   0x00000009
#define VGT_DEBUG_REG0__vgt_no_dma_busy_extended__SHIFT    0x0000000a
#define VGT_DEBUG_REG0__vgt_no_dma_busy__SHIFT             0x0000000b
#define VGT_DEBUG_REG0__vgt_busy_extended__SHIFT           0x0000000c
#define VGT_DEBUG_REG0__vgt_busy__SHIFT                    0x0000000d
#define VGT_DEBUG_REG0__rbbm_skid_fifo_busy_out__SHIFT     0x0000000e
#define VGT_DEBUG_REG0__VGT_RBBM_no_dma_busy__SHIFT        0x0000000f
#define VGT_DEBUG_REG0__VGT_RBBM_busy__SHIFT               0x00000010

// VGT_DEBUG_REG1
#define VGT_DEBUG_REG1__out_te_data_read__SHIFT            0x00000000
#define VGT_DEBUG_REG1__te_out_data_valid__SHIFT           0x00000001
#define VGT_DEBUG_REG1__out_pt_prim_read__SHIFT            0x00000002
#define VGT_DEBUG_REG1__pt_out_prim_valid__SHIFT           0x00000003
#define VGT_DEBUG_REG1__out_pt_data_read__SHIFT            0x00000004
#define VGT_DEBUG_REG1__pt_out_indx_valid__SHIFT           0x00000005
#define VGT_DEBUG_REG1__out_vr_prim_read__SHIFT            0x00000006
#define VGT_DEBUG_REG1__vr_out_prim_valid__SHIFT           0x00000007
#define VGT_DEBUG_REG1__out_vr_indx_read__SHIFT            0x00000008
#define VGT_DEBUG_REG1__vr_out_indx_valid__SHIFT           0x00000009
#define VGT_DEBUG_REG1__te_grp_read__SHIFT                 0x0000000a
#define VGT_DEBUG_REG1__grp_te_valid__SHIFT                0x0000000b
#define VGT_DEBUG_REG1__pt_grp_read__SHIFT                 0x0000000c
#define VGT_DEBUG_REG1__grp_pt_valid__SHIFT                0x0000000d
#define VGT_DEBUG_REG1__vr_grp_read__SHIFT                 0x0000000e
#define VGT_DEBUG_REG1__grp_vr_valid__SHIFT                0x0000000f
#define VGT_DEBUG_REG1__grp_dma_read__SHIFT                0x00000010
#define VGT_DEBUG_REG1__dma_grp_valid__SHIFT               0x00000011
#define VGT_DEBUG_REG1__grp_rbiu_di_read__SHIFT            0x00000012
#define VGT_DEBUG_REG1__rbiu_grp_di_valid__SHIFT           0x00000013
#define VGT_DEBUG_REG1__MH_VGT_rtr__SHIFT                  0x00000014
#define VGT_DEBUG_REG1__VGT_MH_send__SHIFT                 0x00000015
#define VGT_DEBUG_REG1__PA_VGT_clip_s_rtr__SHIFT           0x00000016
#define VGT_DEBUG_REG1__VGT_PA_clip_s_send__SHIFT          0x00000017
#define VGT_DEBUG_REG1__PA_VGT_clip_p_rtr__SHIFT           0x00000018
#define VGT_DEBUG_REG1__VGT_PA_clip_p_send__SHIFT          0x00000019
#define VGT_DEBUG_REG1__PA_VGT_clip_v_rtr__SHIFT           0x0000001a
#define VGT_DEBUG_REG1__VGT_PA_clip_v_send__SHIFT          0x0000001b
#define VGT_DEBUG_REG1__SQ_VGT_rtr__SHIFT                  0x0000001c
#define VGT_DEBUG_REG1__VGT_SQ_send__SHIFT                 0x0000001d
#define VGT_DEBUG_REG1__mh_vgt_tag_7_q__SHIFT              0x0000001e

// VGT_DEBUG_REG3
#define VGT_DEBUG_REG3__vgt_clk_en__SHIFT                  0x00000000
#define VGT_DEBUG_REG3__reg_fifos_clk_en__SHIFT            0x00000001

// VGT_DEBUG_REG6
#define VGT_DEBUG_REG6__shifter_byte_count_q__SHIFT        0x00000000
#define VGT_DEBUG_REG6__right_word_indx_q__SHIFT           0x00000005
#define VGT_DEBUG_REG6__input_data_valid__SHIFT            0x0000000a
#define VGT_DEBUG_REG6__input_data_xfer__SHIFT             0x0000000b
#define VGT_DEBUG_REG6__next_shift_is_vect_1_q__SHIFT      0x0000000c
#define VGT_DEBUG_REG6__next_shift_is_vect_1_d__SHIFT      0x0000000d
#define VGT_DEBUG_REG6__next_shift_is_vect_1_pre_d__SHIFT  0x0000000e
#define VGT_DEBUG_REG6__space_avail_from_shift__SHIFT      0x0000000f
#define VGT_DEBUG_REG6__shifter_first_load__SHIFT          0x00000010
#define VGT_DEBUG_REG6__di_state_sel_q__SHIFT              0x00000011
#define VGT_DEBUG_REG6__shifter_waiting_for_first_load_q__SHIFT 0x00000012
#define VGT_DEBUG_REG6__di_first_group_flag_q__SHIFT       0x00000013
#define VGT_DEBUG_REG6__di_event_flag_q__SHIFT             0x00000014
#define VGT_DEBUG_REG6__read_draw_initiator__SHIFT         0x00000015
#define VGT_DEBUG_REG6__loading_di_requires_shifter__SHIFT 0x00000016
#define VGT_DEBUG_REG6__last_shift_of_packet__SHIFT        0x00000017
#define VGT_DEBUG_REG6__last_decr_of_packet__SHIFT         0x00000018
#define VGT_DEBUG_REG6__extract_vector__SHIFT              0x00000019
#define VGT_DEBUG_REG6__shift_vect_rtr__SHIFT              0x0000001a
#define VGT_DEBUG_REG6__destination_rtr__SHIFT             0x0000001b
#define VGT_DEBUG_REG6__grp_trigger__SHIFT                 0x0000001c

// VGT_DEBUG_REG7
#define VGT_DEBUG_REG7__di_index_counter_q__SHIFT          0x00000000
#define VGT_DEBUG_REG7__shift_amount_no_extract__SHIFT     0x00000010
#define VGT_DEBUG_REG7__shift_amount_extract__SHIFT        0x00000014
#define VGT_DEBUG_REG7__di_prim_type_q__SHIFT              0x00000018
#define VGT_DEBUG_REG7__current_source_sel__SHIFT          0x0000001e

// VGT_DEBUG_REG8
#define VGT_DEBUG_REG8__current_source_sel__SHIFT          0x00000000
#define VGT_DEBUG_REG8__left_word_indx_q__SHIFT            0x00000002
#define VGT_DEBUG_REG8__input_data_cnt__SHIFT              0x00000007
#define VGT_DEBUG_REG8__input_data_lsw__SHIFT              0x0000000c
#define VGT_DEBUG_REG8__input_data_msw__SHIFT              0x00000011
#define VGT_DEBUG_REG8__next_small_stride_shift_limit_q__SHIFT 0x00000016
#define VGT_DEBUG_REG8__current_small_stride_shift_limit_q__SHIFT 0x0000001b

// VGT_DEBUG_REG9
#define VGT_DEBUG_REG9__next_stride_q__SHIFT               0x00000000
#define VGT_DEBUG_REG9__next_stride_d__SHIFT               0x00000005
#define VGT_DEBUG_REG9__current_shift_q__SHIFT             0x0000000a
#define VGT_DEBUG_REG9__current_shift_d__SHIFT             0x0000000f
#define VGT_DEBUG_REG9__current_stride_q__SHIFT            0x00000014
#define VGT_DEBUG_REG9__current_stride_d__SHIFT            0x00000019
#define VGT_DEBUG_REG9__grp_trigger__SHIFT                 0x0000001e

// VGT_DEBUG_REG10
#define VGT_DEBUG_REG10__temp_derived_di_prim_type_t0__SHIFT 0x00000000
#define VGT_DEBUG_REG10__temp_derived_di_small_index_t0__SHIFT 0x00000001
#define VGT_DEBUG_REG10__temp_derived_di_cull_enable_t0__SHIFT 0x00000002
#define VGT_DEBUG_REG10__temp_derived_di_pre_fetch_cull_enable_t0__SHIFT 0x00000003
#define VGT_DEBUG_REG10__di_state_sel_q__SHIFT             0x00000004
#define VGT_DEBUG_REG10__last_decr_of_packet__SHIFT        0x00000005
#define VGT_DEBUG_REG10__bin_valid__SHIFT                  0x00000006
#define VGT_DEBUG_REG10__read_block__SHIFT                 0x00000007
#define VGT_DEBUG_REG10__grp_bgrp_last_bit_read__SHIFT     0x00000008
#define VGT_DEBUG_REG10__last_bit_enable_q__SHIFT          0x00000009
#define VGT_DEBUG_REG10__last_bit_end_di_q__SHIFT          0x0000000a
#define VGT_DEBUG_REG10__selected_data__SHIFT              0x0000000b
#define VGT_DEBUG_REG10__mask_input_data__SHIFT            0x00000013
#define VGT_DEBUG_REG10__gap_q__SHIFT                      0x0000001b
#define VGT_DEBUG_REG10__temp_mini_reset_z__SHIFT          0x0000001c
#define VGT_DEBUG_REG10__temp_mini_reset_y__SHIFT          0x0000001d
#define VGT_DEBUG_REG10__temp_mini_reset_x__SHIFT          0x0000001e
#define VGT_DEBUG_REG10__grp_trigger__SHIFT                0x0000001f

// VGT_DEBUG_REG12
#define VGT_DEBUG_REG12__shifter_byte_count_q__SHIFT       0x00000000
#define VGT_DEBUG_REG12__right_word_indx_q__SHIFT          0x00000005
#define VGT_DEBUG_REG12__input_data_valid__SHIFT           0x0000000a
#define VGT_DEBUG_REG12__input_data_xfer__SHIFT            0x0000000b
#define VGT_DEBUG_REG12__next_shift_is_vect_1_q__SHIFT     0x0000000c
#define VGT_DEBUG_REG12__next_shift_is_vect_1_d__SHIFT     0x0000000d
#define VGT_DEBUG_REG12__next_shift_is_vect_1_pre_d__SHIFT 0x0000000e
#define VGT_DEBUG_REG12__space_avail_from_shift__SHIFT     0x0000000f
#define VGT_DEBUG_REG12__shifter_first_load__SHIFT         0x00000010
#define VGT_DEBUG_REG12__di_state_sel_q__SHIFT             0x00000011
#define VGT_DEBUG_REG12__shifter_waiting_for_first_load_q__SHIFT 0x00000012
#define VGT_DEBUG_REG12__di_first_group_flag_q__SHIFT      0x00000013
#define VGT_DEBUG_REG12__di_event_flag_q__SHIFT            0x00000014
#define VGT_DEBUG_REG12__read_draw_initiator__SHIFT        0x00000015
#define VGT_DEBUG_REG12__loading_di_requires_shifter__SHIFT 0x00000016
#define VGT_DEBUG_REG12__last_shift_of_packet__SHIFT       0x00000017
#define VGT_DEBUG_REG12__last_decr_of_packet__SHIFT        0x00000018
#define VGT_DEBUG_REG12__extract_vector__SHIFT             0x00000019
#define VGT_DEBUG_REG12__shift_vect_rtr__SHIFT             0x0000001a
#define VGT_DEBUG_REG12__destination_rtr__SHIFT            0x0000001b
#define VGT_DEBUG_REG12__bgrp_trigger__SHIFT               0x0000001c

// VGT_DEBUG_REG13
#define VGT_DEBUG_REG13__di_index_counter_q__SHIFT         0x00000000
#define VGT_DEBUG_REG13__shift_amount_no_extract__SHIFT    0x00000010
#define VGT_DEBUG_REG13__shift_amount_extract__SHIFT       0x00000014
#define VGT_DEBUG_REG13__di_prim_type_q__SHIFT             0x00000018
#define VGT_DEBUG_REG13__current_source_sel__SHIFT         0x0000001e

// VGT_DEBUG_REG14
#define VGT_DEBUG_REG14__current_source_sel__SHIFT         0x00000000
#define VGT_DEBUG_REG14__left_word_indx_q__SHIFT           0x00000002
#define VGT_DEBUG_REG14__input_data_cnt__SHIFT             0x00000007
#define VGT_DEBUG_REG14__input_data_lsw__SHIFT             0x0000000c
#define VGT_DEBUG_REG14__input_data_msw__SHIFT             0x00000011
#define VGT_DEBUG_REG14__next_small_stride_shift_limit_q__SHIFT 0x00000016
#define VGT_DEBUG_REG14__current_small_stride_shift_limit_q__SHIFT 0x0000001b

// VGT_DEBUG_REG15
#define VGT_DEBUG_REG15__next_stride_q__SHIFT              0x00000000
#define VGT_DEBUG_REG15__next_stride_d__SHIFT              0x00000005
#define VGT_DEBUG_REG15__current_shift_q__SHIFT            0x0000000a
#define VGT_DEBUG_REG15__current_shift_d__SHIFT            0x0000000f
#define VGT_DEBUG_REG15__current_stride_q__SHIFT           0x00000014
#define VGT_DEBUG_REG15__current_stride_d__SHIFT           0x00000019
#define VGT_DEBUG_REG15__bgrp_trigger__SHIFT               0x0000001e

// VGT_DEBUG_REG16
#define VGT_DEBUG_REG16__bgrp_cull_fetch_fifo_full__SHIFT  0x00000000
#define VGT_DEBUG_REG16__bgrp_cull_fetch_fifo_empty__SHIFT 0x00000001
#define VGT_DEBUG_REG16__dma_bgrp_cull_fetch_read__SHIFT   0x00000002
#define VGT_DEBUG_REG16__bgrp_cull_fetch_fifo_we__SHIFT    0x00000003
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_full__SHIFT   0x00000004
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_empty__SHIFT  0x00000005
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_re_q__SHIFT   0x00000006
#define VGT_DEBUG_REG16__bgrp_byte_mask_fifo_we__SHIFT     0x00000007
#define VGT_DEBUG_REG16__bgrp_dma_mask_kill__SHIFT         0x00000008
#define VGT_DEBUG_REG16__bgrp_grp_bin_valid__SHIFT         0x00000009
#define VGT_DEBUG_REG16__rst_last_bit__SHIFT               0x0000000a
#define VGT_DEBUG_REG16__current_state_q__SHIFT            0x0000000b
#define VGT_DEBUG_REG16__old_state_q__SHIFT                0x0000000c
#define VGT_DEBUG_REG16__old_state_en__SHIFT               0x0000000d
#define VGT_DEBUG_REG16__prev_last_bit_q__SHIFT            0x0000000e
#define VGT_DEBUG_REG16__dbl_last_bit_q__SHIFT             0x0000000f
#define VGT_DEBUG_REG16__last_bit_block_q__SHIFT           0x00000010
#define VGT_DEBUG_REG16__ast_bit_block2_q__SHIFT           0x00000011
#define VGT_DEBUG_REG16__load_empty_reg__SHIFT             0x00000012
#define VGT_DEBUG_REG16__bgrp_grp_byte_mask_rdata__SHIFT   0x00000013
#define VGT_DEBUG_REG16__dma_bgrp_dma_data_fifo_rptr__SHIFT 0x0000001b
#define VGT_DEBUG_REG16__top_di_pre_fetch_cull_enable__SHIFT 0x0000001d
#define VGT_DEBUG_REG16__top_di_grp_cull_enable_q__SHIFT   0x0000001e
#define VGT_DEBUG_REG16__bgrp_trigger__SHIFT               0x0000001f

// VGT_DEBUG_REG17
#define VGT_DEBUG_REG17__save_read_q__SHIFT                0x00000000
#define VGT_DEBUG_REG17__extend_read_q__SHIFT              0x00000001
#define VGT_DEBUG_REG17__grp_indx_size__SHIFT              0x00000002
#define VGT_DEBUG_REG17__cull_prim_true__SHIFT             0x00000004
#define VGT_DEBUG_REG17__reset_bit2_q__SHIFT               0x00000005
#define VGT_DEBUG_REG17__reset_bit1_q__SHIFT               0x00000006
#define VGT_DEBUG_REG17__first_reg_first_q__SHIFT          0x00000007
#define VGT_DEBUG_REG17__check_second_reg__SHIFT           0x00000008
#define VGT_DEBUG_REG17__check_first_reg__SHIFT            0x00000009
#define VGT_DEBUG_REG17__bgrp_cull_fetch_fifo_wdata__SHIFT 0x0000000a
#define VGT_DEBUG_REG17__save_cull_fetch_data2_q__SHIFT    0x0000000b
#define VGT_DEBUG_REG17__save_cull_fetch_data1_q__SHIFT    0x0000000c
#define VGT_DEBUG_REG17__save_byte_mask_data2_q__SHIFT     0x0000000d
#define VGT_DEBUG_REG17__save_byte_mask_data1_q__SHIFT     0x0000000e
#define VGT_DEBUG_REG17__to_second_reg_q__SHIFT            0x0000000f
#define VGT_DEBUG_REG17__roll_over_msk_q__SHIFT            0x00000010
#define VGT_DEBUG_REG17__max_msk_ptr_q__SHIFT              0x00000011
#define VGT_DEBUG_REG17__min_msk_ptr_q__SHIFT              0x00000018
#define VGT_DEBUG_REG17__bgrp_trigger__SHIFT               0x0000001f

// VGT_DEBUG_REG18
#define VGT_DEBUG_REG18__dma_data_fifo_mem_raddr__SHIFT    0x00000000
#define VGT_DEBUG_REG18__dma_data_fifo_mem_waddr__SHIFT    0x00000006
#define VGT_DEBUG_REG18__dma_bgrp_byte_mask_fifo_re__SHIFT 0x0000000c
#define VGT_DEBUG_REG18__dma_bgrp_dma_data_fifo_rptr__SHIFT 0x0000000d
#define VGT_DEBUG_REG18__dma_mem_full__SHIFT               0x0000000f
#define VGT_DEBUG_REG18__dma_ram_re__SHIFT                 0x00000010
#define VGT_DEBUG_REG18__dma_ram_we__SHIFT                 0x00000011
#define VGT_DEBUG_REG18__dma_mem_empty__SHIFT              0x00000012
#define VGT_DEBUG_REG18__dma_data_fifo_mem_re__SHIFT       0x00000013
#define VGT_DEBUG_REG18__dma_data_fifo_mem_we__SHIFT       0x00000014
#define VGT_DEBUG_REG18__bin_mem_full__SHIFT               0x00000015
#define VGT_DEBUG_REG18__bin_ram_we__SHIFT                 0x00000016
#define VGT_DEBUG_REG18__bin_ram_re__SHIFT                 0x00000017
#define VGT_DEBUG_REG18__bin_mem_empty__SHIFT              0x00000018
#define VGT_DEBUG_REG18__start_bin_req__SHIFT              0x00000019
#define VGT_DEBUG_REG18__fetch_cull_not_used__SHIFT        0x0000001a
#define VGT_DEBUG_REG18__dma_req_xfer__SHIFT               0x0000001b
#define VGT_DEBUG_REG18__have_valid_bin_req__SHIFT         0x0000001c
#define VGT_DEBUG_REG18__have_valid_dma_req__SHIFT         0x0000001d
#define VGT_DEBUG_REG18__bgrp_dma_di_grp_cull_enable__SHIFT 0x0000001e
#define VGT_DEBUG_REG18__bgrp_dma_di_pre_fetch_cull_enable__SHIFT 0x0000001f

// VGT_DEBUG_REG20
#define VGT_DEBUG_REG20__prim_side_indx_valid__SHIFT       0x00000000
#define VGT_DEBUG_REG20__indx_side_fifo_empty__SHIFT       0x00000001
#define VGT_DEBUG_REG20__indx_side_fifo_re__SHIFT          0x00000002
#define VGT_DEBUG_REG20__indx_side_fifo_we__SHIFT          0x00000003
#define VGT_DEBUG_REG20__indx_side_fifo_full__SHIFT        0x00000004
#define VGT_DEBUG_REG20__prim_buffer_empty__SHIFT          0x00000005
#define VGT_DEBUG_REG20__prim_buffer_re__SHIFT             0x00000006
#define VGT_DEBUG_REG20__prim_buffer_we__SHIFT             0x00000007
#define VGT_DEBUG_REG20__prim_buffer_full__SHIFT           0x00000008
#define VGT_DEBUG_REG20__indx_buffer_empty__SHIFT          0x00000009
#define VGT_DEBUG_REG20__indx_buffer_re__SHIFT             0x0000000a
#define VGT_DEBUG_REG20__indx_buffer_we__SHIFT             0x0000000b
#define VGT_DEBUG_REG20__indx_buffer_full__SHIFT           0x0000000c
#define VGT_DEBUG_REG20__hold_prim__SHIFT                  0x0000000d
#define VGT_DEBUG_REG20__sent_cnt__SHIFT                   0x0000000e
#define VGT_DEBUG_REG20__start_of_vtx_vector__SHIFT        0x00000012
#define VGT_DEBUG_REG20__clip_s_pre_hold_prim__SHIFT       0x00000013
#define VGT_DEBUG_REG20__clip_p_pre_hold_prim__SHIFT       0x00000014
#define VGT_DEBUG_REG20__buffered_prim_type_event__SHIFT   0x00000015
#define VGT_DEBUG_REG20__out_trigger__SHIFT                0x0000001a

// VGT_DEBUG_REG21
#define VGT_DEBUG_REG21__null_terminate_vtx_vector__SHIFT  0x00000000
#define VGT_DEBUG_REG21__prim_end_of_vtx_vect_flags__SHIFT 0x00000001
#define VGT_DEBUG_REG21__alloc_counter_q__SHIFT            0x00000004
#define VGT_DEBUG_REG21__curr_slot_in_vtx_vect_q__SHIFT    0x00000007
#define VGT_DEBUG_REG21__int_vtx_counter_q__SHIFT          0x0000000a
#define VGT_DEBUG_REG21__curr_dealloc_distance_q__SHIFT    0x0000000e
#define VGT_DEBUG_REG21__new_packet_q__SHIFT               0x00000012
#define VGT_DEBUG_REG21__new_allocate_q__SHIFT             0x00000013
#define VGT_DEBUG_REG21__num_new_unique_rel_indx__SHIFT    0x00000014
#define VGT_DEBUG_REG21__inserted_null_prim_q__SHIFT       0x00000016
#define VGT_DEBUG_REG21__insert_null_prim__SHIFT           0x00000017
#define VGT_DEBUG_REG21__buffered_prim_eop_mux__SHIFT      0x00000018
#define VGT_DEBUG_REG21__prim_buffer_empty_mux__SHIFT      0x00000019
#define VGT_DEBUG_REG21__buffered_thread_size__SHIFT       0x0000001a
#define VGT_DEBUG_REG21__out_trigger__SHIFT                0x0000001f

// VGT_CRC_SQ_DATA
#define VGT_CRC_SQ_DATA__CRC__SHIFT                        0x00000000

// VGT_CRC_SQ_CTRL
#define VGT_CRC_SQ_CTRL__CRC__SHIFT                        0x00000000

// VGT_PERFCOUNTER0_SELECT
#define VGT_PERFCOUNTER0_SELECT__PERF_SEL__SHIFT           0x00000000

// VGT_PERFCOUNTER1_SELECT
#define VGT_PERFCOUNTER1_SELECT__PERF_SEL__SHIFT           0x00000000

// VGT_PERFCOUNTER2_SELECT
#define VGT_PERFCOUNTER2_SELECT__PERF_SEL__SHIFT           0x00000000

// VGT_PERFCOUNTER3_SELECT
#define VGT_PERFCOUNTER3_SELECT__PERF_SEL__SHIFT           0x00000000

// VGT_PERFCOUNTER0_LOW
#define VGT_PERFCOUNTER0_LOW__PERF_COUNT__SHIFT            0x00000000

// VGT_PERFCOUNTER1_LOW
#define VGT_PERFCOUNTER1_LOW__PERF_COUNT__SHIFT            0x00000000

// VGT_PERFCOUNTER2_LOW
#define VGT_PERFCOUNTER2_LOW__PERF_COUNT__SHIFT            0x00000000

// VGT_PERFCOUNTER3_LOW
#define VGT_PERFCOUNTER3_LOW__PERF_COUNT__SHIFT            0x00000000

// VGT_PERFCOUNTER0_HI
#define VGT_PERFCOUNTER0_HI__PERF_COUNT__SHIFT             0x00000000

// VGT_PERFCOUNTER1_HI
#define VGT_PERFCOUNTER1_HI__PERF_COUNT__SHIFT             0x00000000

// VGT_PERFCOUNTER2_HI
#define VGT_PERFCOUNTER2_HI__PERF_COUNT__SHIFT             0x00000000

// VGT_PERFCOUNTER3_HI
#define VGT_PERFCOUNTER3_HI__PERF_COUNT__SHIFT             0x00000000

// TC_CNTL_STATUS
#define TC_CNTL_STATUS__L2_INVALIDATE__SHIFT               0x00000000
#define TC_CNTL_STATUS__TC_L2_HIT_MISS__SHIFT              0x00000012
#define TC_CNTL_STATUS__TC_BUSY__SHIFT                     0x0000001f

// TCR_CHICKEN
#define TCR_CHICKEN__SPARE__SHIFT                          0x00000000

// TCF_CHICKEN
#define TCF_CHICKEN__SPARE__SHIFT                          0x00000000

// TCM_CHICKEN
#define TCM_CHICKEN__TCO_READ_LATENCY_FIFO_PROG_DEPTH__SHIFT 0x00000000
#define TCM_CHICKEN__ETC_COLOR_ENDIAN__SHIFT               0x00000008
#define TCM_CHICKEN__SPARE__SHIFT                          0x00000009

// TCR_PERFCOUNTER0_SELECT
#define TCR_PERFCOUNTER0_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCR_PERFCOUNTER1_SELECT
#define TCR_PERFCOUNTER1_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCR_PERFCOUNTER0_HI
#define TCR_PERFCOUNTER0_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCR_PERFCOUNTER1_HI
#define TCR_PERFCOUNTER1_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCR_PERFCOUNTER0_LOW
#define TCR_PERFCOUNTER0_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCR_PERFCOUNTER1_LOW
#define TCR_PERFCOUNTER1_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TP_TC_CLKGATE_CNTL
#define TP_TC_CLKGATE_CNTL__TP_BUSY_EXTEND__SHIFT          0x00000000
#define TP_TC_CLKGATE_CNTL__TC_BUSY_EXTEND__SHIFT          0x00000003

// TPC_CNTL_STATUS
#define TPC_CNTL_STATUS__TPC_INPUT_BUSY__SHIFT             0x00000000
#define TPC_CNTL_STATUS__TPC_TC_FIFO_BUSY__SHIFT           0x00000001
#define TPC_CNTL_STATUS__TPC_STATE_FIFO_BUSY__SHIFT        0x00000002
#define TPC_CNTL_STATUS__TPC_FETCH_FIFO_BUSY__SHIFT        0x00000003
#define TPC_CNTL_STATUS__TPC_WALKER_PIPE_BUSY__SHIFT       0x00000004
#define TPC_CNTL_STATUS__TPC_WALK_FIFO_BUSY__SHIFT         0x00000005
#define TPC_CNTL_STATUS__TPC_WALKER_BUSY__SHIFT            0x00000006
#define TPC_CNTL_STATUS__TPC_ALIGNER_PIPE_BUSY__SHIFT      0x00000008
#define TPC_CNTL_STATUS__TPC_ALIGN_FIFO_BUSY__SHIFT        0x00000009
#define TPC_CNTL_STATUS__TPC_ALIGNER_BUSY__SHIFT           0x0000000a
#define TPC_CNTL_STATUS__TPC_RR_FIFO_BUSY__SHIFT           0x0000000c
#define TPC_CNTL_STATUS__TPC_BLEND_PIPE_BUSY__SHIFT        0x0000000d
#define TPC_CNTL_STATUS__TPC_OUT_FIFO_BUSY__SHIFT          0x0000000e
#define TPC_CNTL_STATUS__TPC_BLEND_BUSY__SHIFT             0x0000000f
#define TPC_CNTL_STATUS__TF_TW_RTS__SHIFT                  0x00000010
#define TPC_CNTL_STATUS__TF_TW_STATE_RTS__SHIFT            0x00000011
#define TPC_CNTL_STATUS__TF_TW_RTR__SHIFT                  0x00000013
#define TPC_CNTL_STATUS__TW_TA_RTS__SHIFT                  0x00000014
#define TPC_CNTL_STATUS__TW_TA_TT_RTS__SHIFT               0x00000015
#define TPC_CNTL_STATUS__TW_TA_LAST_RTS__SHIFT             0x00000016
#define TPC_CNTL_STATUS__TW_TA_RTR__SHIFT                  0x00000017
#define TPC_CNTL_STATUS__TA_TB_RTS__SHIFT                  0x00000018
#define TPC_CNTL_STATUS__TA_TB_TT_RTS__SHIFT               0x00000019
#define TPC_CNTL_STATUS__TA_TB_RTR__SHIFT                  0x0000001b
#define TPC_CNTL_STATUS__TA_TF_RTS__SHIFT                  0x0000001c
#define TPC_CNTL_STATUS__TA_TF_TC_FIFO_REN__SHIFT          0x0000001d
#define TPC_CNTL_STATUS__TP_SQ_DEC__SHIFT                  0x0000001e
#define TPC_CNTL_STATUS__TPC_BUSY__SHIFT                   0x0000001f

// TPC_DEBUG0
#define TPC_DEBUG0__LOD_CNTL__SHIFT                        0x00000000
#define TPC_DEBUG0__IC_CTR__SHIFT                          0x00000002
#define TPC_DEBUG0__WALKER_CNTL__SHIFT                     0x00000004
#define TPC_DEBUG0__ALIGNER_CNTL__SHIFT                    0x00000008
#define TPC_DEBUG0__PREV_TC_STATE_VALID__SHIFT             0x0000000c
#define TPC_DEBUG0__WALKER_STATE__SHIFT                    0x00000010
#define TPC_DEBUG0__ALIGNER_STATE__SHIFT                   0x0000001a
#define TPC_DEBUG0__REG_CLK_EN__SHIFT                      0x0000001d
#define TPC_DEBUG0__TPC_CLK_EN__SHIFT                      0x0000001e
#define TPC_DEBUG0__SQ_TP_WAKEUP__SHIFT                    0x0000001f

// TPC_DEBUG1
#define TPC_DEBUG1__UNUSED__SHIFT                          0x00000000

// TPC_CHICKEN
#define TPC_CHICKEN__BLEND_PRECISION__SHIFT                0x00000000
#define TPC_CHICKEN__SPARE__SHIFT                          0x00000001

// TP0_CNTL_STATUS
#define TP0_CNTL_STATUS__TP_INPUT_BUSY__SHIFT              0x00000000
#define TP0_CNTL_STATUS__TP_LOD_BUSY__SHIFT                0x00000001
#define TP0_CNTL_STATUS__TP_LOD_FIFO_BUSY__SHIFT           0x00000002
#define TP0_CNTL_STATUS__TP_ADDR_BUSY__SHIFT               0x00000003
#define TP0_CNTL_STATUS__TP_ALIGN_FIFO_BUSY__SHIFT         0x00000004
#define TP0_CNTL_STATUS__TP_ALIGNER_BUSY__SHIFT            0x00000005
#define TP0_CNTL_STATUS__TP_TC_FIFO_BUSY__SHIFT            0x00000006
#define TP0_CNTL_STATUS__TP_RR_FIFO_BUSY__SHIFT            0x00000007
#define TP0_CNTL_STATUS__TP_FETCH_BUSY__SHIFT              0x00000008
#define TP0_CNTL_STATUS__TP_CH_BLEND_BUSY__SHIFT           0x00000009
#define TP0_CNTL_STATUS__TP_TT_BUSY__SHIFT                 0x0000000a
#define TP0_CNTL_STATUS__TP_HICOLOR_BUSY__SHIFT            0x0000000b
#define TP0_CNTL_STATUS__TP_BLEND_BUSY__SHIFT              0x0000000c
#define TP0_CNTL_STATUS__TP_OUT_FIFO_BUSY__SHIFT           0x0000000d
#define TP0_CNTL_STATUS__TP_OUTPUT_BUSY__SHIFT             0x0000000e
#define TP0_CNTL_STATUS__IN_LC_RTS__SHIFT                  0x00000010
#define TP0_CNTL_STATUS__LC_LA_RTS__SHIFT                  0x00000011
#define TP0_CNTL_STATUS__LA_FL_RTS__SHIFT                  0x00000012
#define TP0_CNTL_STATUS__FL_TA_RTS__SHIFT                  0x00000013
#define TP0_CNTL_STATUS__TA_FA_RTS__SHIFT                  0x00000014
#define TP0_CNTL_STATUS__TA_FA_TT_RTS__SHIFT               0x00000015
#define TP0_CNTL_STATUS__FA_AL_RTS__SHIFT                  0x00000016
#define TP0_CNTL_STATUS__FA_AL_TT_RTS__SHIFT               0x00000017
#define TP0_CNTL_STATUS__AL_TF_RTS__SHIFT                  0x00000018
#define TP0_CNTL_STATUS__AL_TF_TT_RTS__SHIFT               0x00000019
#define TP0_CNTL_STATUS__TF_TB_RTS__SHIFT                  0x0000001a
#define TP0_CNTL_STATUS__TF_TB_TT_RTS__SHIFT               0x0000001b
#define TP0_CNTL_STATUS__TB_TT_RTS__SHIFT                  0x0000001c
#define TP0_CNTL_STATUS__TB_TT_TT_RESET__SHIFT             0x0000001d
#define TP0_CNTL_STATUS__TB_TO_RTS__SHIFT                  0x0000001e
#define TP0_CNTL_STATUS__TP_BUSY__SHIFT                    0x0000001f

// TP0_DEBUG
#define TP0_DEBUG__Q_LOD_CNTL__SHIFT                       0x00000000
#define TP0_DEBUG__Q_SQ_TP_WAKEUP__SHIFT                   0x00000003
#define TP0_DEBUG__FL_TA_ADDRESSER_CNTL__SHIFT             0x00000004
#define TP0_DEBUG__REG_CLK_EN__SHIFT                       0x00000015
#define TP0_DEBUG__PERF_CLK_EN__SHIFT                      0x00000016
#define TP0_DEBUG__TP_CLK_EN__SHIFT                        0x00000017
#define TP0_DEBUG__Q_WALKER_CNTL__SHIFT                    0x00000018
#define TP0_DEBUG__Q_ALIGNER_CNTL__SHIFT                   0x0000001c

// TP0_CHICKEN
#define TP0_CHICKEN__TT_MODE__SHIFT                        0x00000000
#define TP0_CHICKEN__VFETCH_ADDRESS_MODE__SHIFT            0x00000001
#define TP0_CHICKEN__SPARE__SHIFT                          0x00000002

// TP0_PERFCOUNTER0_SELECT
#define TP0_PERFCOUNTER0_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TP0_PERFCOUNTER0_HI
#define TP0_PERFCOUNTER0_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TP0_PERFCOUNTER0_LOW
#define TP0_PERFCOUNTER0_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TP0_PERFCOUNTER1_SELECT
#define TP0_PERFCOUNTER1_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TP0_PERFCOUNTER1_HI
#define TP0_PERFCOUNTER1_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TP0_PERFCOUNTER1_LOW
#define TP0_PERFCOUNTER1_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCM_PERFCOUNTER0_SELECT
#define TCM_PERFCOUNTER0_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCM_PERFCOUNTER1_SELECT
#define TCM_PERFCOUNTER1_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCM_PERFCOUNTER0_HI
#define TCM_PERFCOUNTER0_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCM_PERFCOUNTER1_HI
#define TCM_PERFCOUNTER1_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCM_PERFCOUNTER0_LOW
#define TCM_PERFCOUNTER0_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCM_PERFCOUNTER1_LOW
#define TCM_PERFCOUNTER1_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCF_PERFCOUNTER0_SELECT
#define TCF_PERFCOUNTER0_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER1_SELECT
#define TCF_PERFCOUNTER1_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER2_SELECT
#define TCF_PERFCOUNTER2_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER3_SELECT
#define TCF_PERFCOUNTER3_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER4_SELECT
#define TCF_PERFCOUNTER4_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER5_SELECT
#define TCF_PERFCOUNTER5_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER6_SELECT
#define TCF_PERFCOUNTER6_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER7_SELECT
#define TCF_PERFCOUNTER7_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER8_SELECT
#define TCF_PERFCOUNTER8_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER9_SELECT
#define TCF_PERFCOUNTER9_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER10_SELECT
#define TCF_PERFCOUNTER10_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER11_SELECT
#define TCF_PERFCOUNTER11_SELECT__PERFCOUNTER_SELECT__SHIFT 0x00000000

// TCF_PERFCOUNTER0_HI
#define TCF_PERFCOUNTER0_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCF_PERFCOUNTER1_HI
#define TCF_PERFCOUNTER1_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCF_PERFCOUNTER2_HI
#define TCF_PERFCOUNTER2_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCF_PERFCOUNTER3_HI
#define TCF_PERFCOUNTER3_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCF_PERFCOUNTER4_HI
#define TCF_PERFCOUNTER4_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCF_PERFCOUNTER5_HI
#define TCF_PERFCOUNTER5_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCF_PERFCOUNTER6_HI
#define TCF_PERFCOUNTER6_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCF_PERFCOUNTER7_HI
#define TCF_PERFCOUNTER7_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCF_PERFCOUNTER8_HI
#define TCF_PERFCOUNTER8_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCF_PERFCOUNTER9_HI
#define TCF_PERFCOUNTER9_HI__PERFCOUNTER_HI__SHIFT         0x00000000

// TCF_PERFCOUNTER10_HI
#define TCF_PERFCOUNTER10_HI__PERFCOUNTER_HI__SHIFT        0x00000000

// TCF_PERFCOUNTER11_HI
#define TCF_PERFCOUNTER11_HI__PERFCOUNTER_HI__SHIFT        0x00000000

// TCF_PERFCOUNTER0_LOW
#define TCF_PERFCOUNTER0_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCF_PERFCOUNTER1_LOW
#define TCF_PERFCOUNTER1_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCF_PERFCOUNTER2_LOW
#define TCF_PERFCOUNTER2_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCF_PERFCOUNTER3_LOW
#define TCF_PERFCOUNTER3_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCF_PERFCOUNTER4_LOW
#define TCF_PERFCOUNTER4_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCF_PERFCOUNTER5_LOW
#define TCF_PERFCOUNTER5_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCF_PERFCOUNTER6_LOW
#define TCF_PERFCOUNTER6_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCF_PERFCOUNTER7_LOW
#define TCF_PERFCOUNTER7_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCF_PERFCOUNTER8_LOW
#define TCF_PERFCOUNTER8_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCF_PERFCOUNTER9_LOW
#define TCF_PERFCOUNTER9_LOW__PERFCOUNTER_LOW__SHIFT       0x00000000

// TCF_PERFCOUNTER10_LOW
#define TCF_PERFCOUNTER10_LOW__PERFCOUNTER_LOW__SHIFT      0x00000000

// TCF_PERFCOUNTER11_LOW
#define TCF_PERFCOUNTER11_LOW__PERFCOUNTER_LOW__SHIFT      0x00000000

// TCF_DEBUG
#define TCF_DEBUG__not_MH_TC_rtr__SHIFT                    0x00000006
#define TCF_DEBUG__TC_MH_send__SHIFT                       0x00000007
#define TCF_DEBUG__not_FG0_rtr__SHIFT                      0x00000008
#define TCF_DEBUG__not_TCB_TCO_rtr__SHIFT                  0x0000000c
#define TCF_DEBUG__TCB_ff_stall__SHIFT                     0x0000000d
#define TCF_DEBUG__TCB_miss_stall__SHIFT                   0x0000000e
#define TCF_DEBUG__TCA_TCB_stall__SHIFT                    0x0000000f
#define TCF_DEBUG__PF0_stall__SHIFT                        0x00000010
#define TCF_DEBUG__TP0_full__SHIFT                         0x00000014
#define TCF_DEBUG__TPC_full__SHIFT                         0x00000018
#define TCF_DEBUG__not_TPC_rtr__SHIFT                      0x00000019
#define TCF_DEBUG__tca_state_rts__SHIFT                    0x0000001a
#define TCF_DEBUG__tca_rts__SHIFT                          0x0000001b

// TCA_FIFO_DEBUG
#define TCA_FIFO_DEBUG__tp0_full__SHIFT                    0x00000000
#define TCA_FIFO_DEBUG__tpc_full__SHIFT                    0x00000004
#define TCA_FIFO_DEBUG__load_tpc_fifo__SHIFT               0x00000005
#define TCA_FIFO_DEBUG__load_tp_fifos__SHIFT               0x00000006
#define TCA_FIFO_DEBUG__FW_full__SHIFT                     0x00000007
#define TCA_FIFO_DEBUG__not_FW_rtr0__SHIFT                 0x00000008
#define TCA_FIFO_DEBUG__FW_rts0__SHIFT                     0x0000000c
#define TCA_FIFO_DEBUG__not_FW_tpc_rtr__SHIFT              0x00000010
#define TCA_FIFO_DEBUG__FW_tpc_rts__SHIFT                  0x00000011

// TCA_PROBE_DEBUG
#define TCA_PROBE_DEBUG__ProbeFilter_stall__SHIFT          0x00000000

// TCA_TPC_DEBUG
#define TCA_TPC_DEBUG__captue_state_rts__SHIFT             0x0000000c
#define TCA_TPC_DEBUG__capture_tca_rts__SHIFT              0x0000000d

// TCB_CORE_DEBUG
#define TCB_CORE_DEBUG__access512__SHIFT                   0x00000000
#define TCB_CORE_DEBUG__tiled__SHIFT                       0x00000001
#define TCB_CORE_DEBUG__opcode__SHIFT                      0x00000004
#define TCB_CORE_DEBUG__format__SHIFT                      0x00000008
#define TCB_CORE_DEBUG__sector_format__SHIFT               0x00000010
#define TCB_CORE_DEBUG__sector_format512__SHIFT            0x00000018

// TCB_TAG0_DEBUG
#define TCB_TAG0_DEBUG__mem_read_cycle__SHIFT              0x00000000
#define TCB_TAG0_DEBUG__tag_access_cycle__SHIFT            0x0000000c
#define TCB_TAG0_DEBUG__miss_stall__SHIFT                  0x00000017
#define TCB_TAG0_DEBUG__num_feee_lines__SHIFT              0x00000018
#define TCB_TAG0_DEBUG__max_misses__SHIFT                  0x0000001d

// TCB_TAG1_DEBUG
#define TCB_TAG1_DEBUG__mem_read_cycle__SHIFT              0x00000000
#define TCB_TAG1_DEBUG__tag_access_cycle__SHIFT            0x0000000c
#define TCB_TAG1_DEBUG__miss_stall__SHIFT                  0x00000017
#define TCB_TAG1_DEBUG__num_feee_lines__SHIFT              0x00000018
#define TCB_TAG1_DEBUG__max_misses__SHIFT                  0x0000001d

// TCB_TAG2_DEBUG
#define TCB_TAG2_DEBUG__mem_read_cycle__SHIFT              0x00000000
#define TCB_TAG2_DEBUG__tag_access_cycle__SHIFT            0x0000000c
#define TCB_TAG2_DEBUG__miss_stall__SHIFT                  0x00000017
#define TCB_TAG2_DEBUG__num_feee_lines__SHIFT              0x00000018
#define TCB_TAG2_DEBUG__max_misses__SHIFT                  0x0000001d

// TCB_TAG3_DEBUG
#define TCB_TAG3_DEBUG__mem_read_cycle__SHIFT              0x00000000
#define TCB_TAG3_DEBUG__tag_access_cycle__SHIFT            0x0000000c
#define TCB_TAG3_DEBUG__miss_stall__SHIFT                  0x00000017
#define TCB_TAG3_DEBUG__num_feee_lines__SHIFT              0x00000018
#define TCB_TAG3_DEBUG__max_misses__SHIFT                  0x0000001d

// TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__left_done__SHIFT 0x00000000
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__fg0_sends_left__SHIFT 0x00000002
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__one_sector_to_go_left_q__SHIFT 0x00000004
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__no_sectors_to_go__SHIFT 0x00000005
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__update_left__SHIFT 0x00000006
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__sector_mask_left_count_q__SHIFT 0x00000007
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__sector_mask_left_q__SHIFT 0x0000000c
#define TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG__valid_left_q__SHIFT 0x0000001c

// TCB_FETCH_GEN_WALKER_DEBUG
#define TCB_FETCH_GEN_WALKER_DEBUG__quad_sel_left__SHIFT   0x00000004
#define TCB_FETCH_GEN_WALKER_DEBUG__set_sel_left__SHIFT    0x00000006
#define TCB_FETCH_GEN_WALKER_DEBUG__right_eq_left__SHIFT   0x0000000b
#define TCB_FETCH_GEN_WALKER_DEBUG__ff_fg_type512__SHIFT   0x0000000c
#define TCB_FETCH_GEN_WALKER_DEBUG__busy__SHIFT            0x0000000f
#define TCB_FETCH_GEN_WALKER_DEBUG__setquads_to_send__SHIFT 0x00000010

// TCB_FETCH_GEN_PIPE0_DEBUG
#define TCB_FETCH_GEN_PIPE0_DEBUG__tc0_arb_rts__SHIFT      0x00000000
#define TCB_FETCH_GEN_PIPE0_DEBUG__ga_out_rts__SHIFT       0x00000002
#define TCB_FETCH_GEN_PIPE0_DEBUG__tc_arb_format__SHIFT    0x00000004
#define TCB_FETCH_GEN_PIPE0_DEBUG__tc_arb_fmsopcode__SHIFT 0x00000010
#define TCB_FETCH_GEN_PIPE0_DEBUG__tc_arb_request_type__SHIFT 0x00000015
#define TCB_FETCH_GEN_PIPE0_DEBUG__busy__SHIFT             0x00000017
#define TCB_FETCH_GEN_PIPE0_DEBUG__fgo_busy__SHIFT         0x00000018
#define TCB_FETCH_GEN_PIPE0_DEBUG__ga_busy__SHIFT          0x00000019
#define TCB_FETCH_GEN_PIPE0_DEBUG__mc_sel_q__SHIFT         0x0000001a
#define TCB_FETCH_GEN_PIPE0_DEBUG__valid_q__SHIFT          0x0000001c
#define TCB_FETCH_GEN_PIPE0_DEBUG__arb_RTR__SHIFT          0x0000001e

// TCD_INPUT0_DEBUG
#define TCD_INPUT0_DEBUG__empty__SHIFT                     0x00000010
#define TCD_INPUT0_DEBUG__full__SHIFT                      0x00000011
#define TCD_INPUT0_DEBUG__valid_q1__SHIFT                  0x00000014
#define TCD_INPUT0_DEBUG__cnt_q1__SHIFT                    0x00000015
#define TCD_INPUT0_DEBUG__last_send_q1__SHIFT              0x00000017
#define TCD_INPUT0_DEBUG__ip_send__SHIFT                   0x00000018
#define TCD_INPUT0_DEBUG__ipbuf_dxt_send__SHIFT            0x00000019
#define TCD_INPUT0_DEBUG__ipbuf_busy__SHIFT                0x0000001a

// TCD_DEGAMMA_DEBUG
#define TCD_DEGAMMA_DEBUG__dgmm_ftfconv_dgmmen__SHIFT      0x00000000
#define TCD_DEGAMMA_DEBUG__dgmm_ctrl_dgmm8__SHIFT          0x00000002
#define TCD_DEGAMMA_DEBUG__dgmm_ctrl_last_send__SHIFT      0x00000003
#define TCD_DEGAMMA_DEBUG__dgmm_ctrl_send__SHIFT           0x00000004
#define TCD_DEGAMMA_DEBUG__dgmm_stall__SHIFT               0x00000005
#define TCD_DEGAMMA_DEBUG__dgmm_pstate__SHIFT              0x00000006

// TCD_DXTMUX_SCTARB_DEBUG
#define TCD_DXTMUX_SCTARB_DEBUG__pstate__SHIFT             0x00000009
#define TCD_DXTMUX_SCTARB_DEBUG__sctrmx_rtr__SHIFT         0x0000000a
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_rtr__SHIFT           0x0000000b
#define TCD_DXTMUX_SCTARB_DEBUG__sctrarb_multcyl_send__SHIFT 0x0000000f
#define TCD_DXTMUX_SCTARB_DEBUG__sctrmx0_sctrarb_rts__SHIFT 0x00000010
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_sctrarb_send__SHIFT  0x00000014
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_dgmmpd_last_send__SHIFT 0x0000001b
#define TCD_DXTMUX_SCTARB_DEBUG__dxtc_dgmmpd_send__SHIFT   0x0000001c
#define TCD_DXTMUX_SCTARB_DEBUG__dcmp_mux_send__SHIFT      0x0000001d

// TCD_DXTC_ARB_DEBUG
#define TCD_DXTC_ARB_DEBUG__n0_stall__SHIFT                0x00000004
#define TCD_DXTC_ARB_DEBUG__pstate__SHIFT                  0x00000005
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_last_send__SHIFT    0x00000006
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_cnt__SHIFT          0x00000007
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_sector__SHIFT       0x00000009
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_cacheline__SHIFT    0x0000000c
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_format__SHIFT       0x00000012
#define TCD_DXTC_ARB_DEBUG__arb_dcmp01_send__SHIFT         0x0000001e
#define TCD_DXTC_ARB_DEBUG__n0_dxt2_4_types__SHIFT         0x0000001f

// TCD_STALLS_DEBUG
#define TCD_STALLS_DEBUG__not_multcyl_sctrarb_rtr__SHIFT   0x0000000a
#define TCD_STALLS_DEBUG__not_sctrmx0_sctrarb_rtr__SHIFT   0x0000000b
#define TCD_STALLS_DEBUG__not_dcmp0_arb_rtr__SHIFT         0x00000011
#define TCD_STALLS_DEBUG__not_dgmmpd_dxtc_rtr__SHIFT       0x00000012
#define TCD_STALLS_DEBUG__not_mux_dcmp_rtr__SHIFT          0x00000013
#define TCD_STALLS_DEBUG__not_incoming_rtr__SHIFT          0x0000001f

// TCO_STALLS_DEBUG
#define TCO_STALLS_DEBUG__quad0_sg_crd_RTR__SHIFT          0x00000005
#define TCO_STALLS_DEBUG__quad0_rl_sg_RTR__SHIFT           0x00000006
#define TCO_STALLS_DEBUG__quad0_TCO_TCB_rtr_d__SHIFT       0x00000007

// TCO_QUAD0_DEBUG0
#define TCO_QUAD0_DEBUG0__rl_sg_sector_format__SHIFT       0x00000000
#define TCO_QUAD0_DEBUG0__rl_sg_end_of_sample__SHIFT       0x00000008
#define TCO_QUAD0_DEBUG0__rl_sg_rtr__SHIFT                 0x00000009
#define TCO_QUAD0_DEBUG0__rl_sg_rts__SHIFT                 0x0000000a
#define TCO_QUAD0_DEBUG0__sg_crd_end_of_sample__SHIFT      0x0000000b
#define TCO_QUAD0_DEBUG0__sg_crd_rtr__SHIFT                0x0000000c
#define TCO_QUAD0_DEBUG0__sg_crd_rts__SHIFT                0x0000000d
#define TCO_QUAD0_DEBUG0__stageN1_valid_q__SHIFT           0x00000010
#define TCO_QUAD0_DEBUG0__read_cache_q__SHIFT              0x00000018
#define TCO_QUAD0_DEBUG0__cache_read_RTR__SHIFT            0x00000019
#define TCO_QUAD0_DEBUG0__all_sectors_written_set3__SHIFT  0x0000001a
#define TCO_QUAD0_DEBUG0__all_sectors_written_set2__SHIFT  0x0000001b
#define TCO_QUAD0_DEBUG0__all_sectors_written_set1__SHIFT  0x0000001c
#define TCO_QUAD0_DEBUG0__all_sectors_written_set0__SHIFT  0x0000001d
#define TCO_QUAD0_DEBUG0__busy__SHIFT                      0x0000001e

// TCO_QUAD0_DEBUG1
#define TCO_QUAD0_DEBUG1__fifo_busy__SHIFT                 0x00000000
#define TCO_QUAD0_DEBUG1__empty__SHIFT                     0x00000001
#define TCO_QUAD0_DEBUG1__full__SHIFT                      0x00000002
#define TCO_QUAD0_DEBUG1__write_enable__SHIFT              0x00000003
#define TCO_QUAD0_DEBUG1__fifo_write_ptr__SHIFT            0x00000004
#define TCO_QUAD0_DEBUG1__fifo_read_ptr__SHIFT             0x0000000b
#define TCO_QUAD0_DEBUG1__cache_read_busy__SHIFT           0x00000014
#define TCO_QUAD0_DEBUG1__latency_fifo_busy__SHIFT         0x00000015
#define TCO_QUAD0_DEBUG1__input_quad_busy__SHIFT           0x00000016
#define TCO_QUAD0_DEBUG1__tco_quad_pipe_busy__SHIFT        0x00000017
#define TCO_QUAD0_DEBUG1__TCB_TCO_rtr_d__SHIFT             0x00000018
#define TCO_QUAD0_DEBUG1__TCB_TCO_xfc_q__SHIFT             0x00000019
#define TCO_QUAD0_DEBUG1__rl_sg_rtr__SHIFT                 0x0000001a
#define TCO_QUAD0_DEBUG1__rl_sg_rts__SHIFT                 0x0000001b
#define TCO_QUAD0_DEBUG1__sg_crd_rtr__SHIFT                0x0000001c
#define TCO_QUAD0_DEBUG1__sg_crd_rts__SHIFT                0x0000001d
#define TCO_QUAD0_DEBUG1__TCO_TCB_read_xfc__SHIFT          0x0000001e

// SQ_GPR_MANAGEMENT
#define SQ_GPR_MANAGEMENT__REG_DYNAMIC__SHIFT              0x00000000
#define SQ_GPR_MANAGEMENT__REG_SIZE_PIX__SHIFT             0x00000004
#define SQ_GPR_MANAGEMENT__REG_SIZE_VTX__SHIFT             0x0000000c

// SQ_FLOW_CONTROL
#define SQ_FLOW_CONTROL__INPUT_ARBITRATION_POLICY__SHIFT   0x00000000
#define SQ_FLOW_CONTROL__ONE_THREAD__SHIFT                 0x00000004
#define SQ_FLOW_CONTROL__ONE_ALU__SHIFT                    0x00000008
#define SQ_FLOW_CONTROL__CF_WR_BASE__SHIFT                 0x0000000c
#define SQ_FLOW_CONTROL__NO_PV_PS__SHIFT                   0x00000010
#define SQ_FLOW_CONTROL__NO_LOOP_EXIT__SHIFT               0x00000011
#define SQ_FLOW_CONTROL__NO_CEXEC_OPTIMIZE__SHIFT          0x00000012
#define SQ_FLOW_CONTROL__TEXTURE_ARBITRATION_POLICY__SHIFT 0x00000013
#define SQ_FLOW_CONTROL__VC_ARBITRATION_POLICY__SHIFT      0x00000015
#define SQ_FLOW_CONTROL__ALU_ARBITRATION_POLICY__SHIFT     0x00000016
#define SQ_FLOW_CONTROL__NO_ARB_EJECT__SHIFT               0x00000017
#define SQ_FLOW_CONTROL__NO_CFS_EJECT__SHIFT               0x00000018
#define SQ_FLOW_CONTROL__POS_EXP_PRIORITY__SHIFT           0x00000019
#define SQ_FLOW_CONTROL__NO_EARLY_THREAD_TERMINATION__SHIFT 0x0000001a
#define SQ_FLOW_CONTROL__PS_PREFETCH_COLOR_ALLOC__SHIFT    0x0000001b

// SQ_INST_STORE_MANAGMENT
#define SQ_INST_STORE_MANAGMENT__INST_BASE_PIX__SHIFT      0x00000000
#define SQ_INST_STORE_MANAGMENT__INST_BASE_VTX__SHIFT      0x00000010

// SQ_RESOURCE_MANAGMENT
#define SQ_RESOURCE_MANAGMENT__VTX_THREAD_BUF_ENTRIES__SHIFT 0x00000000
#define SQ_RESOURCE_MANAGMENT__PIX_THREAD_BUF_ENTRIES__SHIFT 0x00000008
#define SQ_RESOURCE_MANAGMENT__EXPORT_BUF_ENTRIES__SHIFT   0x00000010

// SQ_EO_RT
#define SQ_EO_RT__EO_CONSTANTS_RT__SHIFT                   0x00000000
#define SQ_EO_RT__EO_TSTATE_RT__SHIFT                      0x00000010

// SQ_DEBUG_MISC
#define SQ_DEBUG_MISC__DB_ALUCST_SIZE__SHIFT               0x00000000
#define SQ_DEBUG_MISC__DB_TSTATE_SIZE__SHIFT               0x0000000c
#define SQ_DEBUG_MISC__DB_READ_CTX__SHIFT                  0x00000014
#define SQ_DEBUG_MISC__RESERVED__SHIFT                     0x00000015
#define SQ_DEBUG_MISC__DB_READ_MEMORY__SHIFT               0x00000017
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_0__SHIFT              0x00000019
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_1__SHIFT              0x0000001a
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_2__SHIFT              0x0000001b
#define SQ_DEBUG_MISC__DB_WEN_MEMORY_3__SHIFT              0x0000001c

// SQ_ACTIVITY_METER_CNTL
#define SQ_ACTIVITY_METER_CNTL__TIMEBASE__SHIFT            0x00000000
#define SQ_ACTIVITY_METER_CNTL__THRESHOLD_LOW__SHIFT       0x00000008
#define SQ_ACTIVITY_METER_CNTL__THRESHOLD_HIGH__SHIFT      0x00000010
#define SQ_ACTIVITY_METER_CNTL__SPARE__SHIFT               0x00000018

// SQ_ACTIVITY_METER_STATUS
#define SQ_ACTIVITY_METER_STATUS__PERCENT_BUSY__SHIFT      0x00000000

// SQ_INPUT_ARB_PRIORITY
#define SQ_INPUT_ARB_PRIORITY__PC_AVAIL_WEIGHT__SHIFT      0x00000000
#define SQ_INPUT_ARB_PRIORITY__PC_AVAIL_SIGN__SHIFT        0x00000003
#define SQ_INPUT_ARB_PRIORITY__SX_AVAIL_WEIGHT__SHIFT      0x00000004
#define SQ_INPUT_ARB_PRIORITY__SX_AVAIL_SIGN__SHIFT        0x00000007
#define SQ_INPUT_ARB_PRIORITY__THRESHOLD__SHIFT            0x00000008

// SQ_THREAD_ARB_PRIORITY
#define SQ_THREAD_ARB_PRIORITY__PC_AVAIL_WEIGHT__SHIFT     0x00000000
#define SQ_THREAD_ARB_PRIORITY__PC_AVAIL_SIGN__SHIFT       0x00000003
#define SQ_THREAD_ARB_PRIORITY__SX_AVAIL_WEIGHT__SHIFT     0x00000004
#define SQ_THREAD_ARB_PRIORITY__SX_AVAIL_SIGN__SHIFT       0x00000007
#define SQ_THREAD_ARB_PRIORITY__THRESHOLD__SHIFT           0x00000008
#define SQ_THREAD_ARB_PRIORITY__RESERVED__SHIFT            0x00000012
#define SQ_THREAD_ARB_PRIORITY__VS_PRIORITIZE_SERIAL__SHIFT 0x00000014
#define SQ_THREAD_ARB_PRIORITY__PS_PRIORITIZE_SERIAL__SHIFT 0x00000015
#define SQ_THREAD_ARB_PRIORITY__USE_SERIAL_COUNT_THRESHOLD__SHIFT 0x00000016

// SQ_VS_WATCHDOG_TIMER
#define SQ_VS_WATCHDOG_TIMER__ENABLE__SHIFT                0x00000000
#define SQ_VS_WATCHDOG_TIMER__TIMEOUT_COUNT__SHIFT         0x00000001

// SQ_PS_WATCHDOG_TIMER
#define SQ_PS_WATCHDOG_TIMER__ENABLE__SHIFT                0x00000000
#define SQ_PS_WATCHDOG_TIMER__TIMEOUT_COUNT__SHIFT         0x00000001

// SQ_INT_CNTL
#define SQ_INT_CNTL__PS_WATCHDOG_MASK__SHIFT               0x00000000
#define SQ_INT_CNTL__VS_WATCHDOG_MASK__SHIFT               0x00000001

// SQ_INT_STATUS
#define SQ_INT_STATUS__PS_WATCHDOG_TIMEOUT__SHIFT          0x00000000
#define SQ_INT_STATUS__VS_WATCHDOG_TIMEOUT__SHIFT          0x00000001

// SQ_INT_ACK
#define SQ_INT_ACK__PS_WATCHDOG_ACK__SHIFT                 0x00000000
#define SQ_INT_ACK__VS_WATCHDOG_ACK__SHIFT                 0x00000001

// SQ_DEBUG_INPUT_FSM
#define SQ_DEBUG_INPUT_FSM__VC_VSR_LD__SHIFT               0x00000000
#define SQ_DEBUG_INPUT_FSM__RESERVED__SHIFT                0x00000003
#define SQ_DEBUG_INPUT_FSM__VC_GPR_LD__SHIFT               0x00000004
#define SQ_DEBUG_INPUT_FSM__PC_PISM__SHIFT                 0x00000008
#define SQ_DEBUG_INPUT_FSM__RESERVED1__SHIFT               0x0000000b
#define SQ_DEBUG_INPUT_FSM__PC_AS__SHIFT                   0x0000000c
#define SQ_DEBUG_INPUT_FSM__PC_INTERP_CNT__SHIFT           0x0000000f
#define SQ_DEBUG_INPUT_FSM__PC_GPR_SIZE__SHIFT             0x00000014

// SQ_DEBUG_CONST_MGR_FSM
#define SQ_DEBUG_CONST_MGR_FSM__TEX_CONST_EVENT_STATE__SHIFT 0x00000000
#define SQ_DEBUG_CONST_MGR_FSM__RESERVED1__SHIFT           0x00000005
#define SQ_DEBUG_CONST_MGR_FSM__ALU_CONST_EVENT_STATE__SHIFT 0x00000008
#define SQ_DEBUG_CONST_MGR_FSM__RESERVED2__SHIFT           0x0000000d
#define SQ_DEBUG_CONST_MGR_FSM__ALU_CONST_CNTX_VALID__SHIFT 0x00000010
#define SQ_DEBUG_CONST_MGR_FSM__TEX_CONST_CNTX_VALID__SHIFT 0x00000012
#define SQ_DEBUG_CONST_MGR_FSM__CNTX0_VTX_EVENT_DONE__SHIFT 0x00000014
#define SQ_DEBUG_CONST_MGR_FSM__CNTX0_PIX_EVENT_DONE__SHIFT 0x00000015
#define SQ_DEBUG_CONST_MGR_FSM__CNTX1_VTX_EVENT_DONE__SHIFT 0x00000016
#define SQ_DEBUG_CONST_MGR_FSM__CNTX1_PIX_EVENT_DONE__SHIFT 0x00000017

// SQ_DEBUG_TP_FSM
#define SQ_DEBUG_TP_FSM__EX_TP__SHIFT                      0x00000000
#define SQ_DEBUG_TP_FSM__RESERVED0__SHIFT                  0x00000003
#define SQ_DEBUG_TP_FSM__CF_TP__SHIFT                      0x00000004
#define SQ_DEBUG_TP_FSM__IF_TP__SHIFT                      0x00000008
#define SQ_DEBUG_TP_FSM__RESERVED1__SHIFT                  0x0000000b
#define SQ_DEBUG_TP_FSM__TIS_TP__SHIFT                     0x0000000c
#define SQ_DEBUG_TP_FSM__RESERVED2__SHIFT                  0x0000000e
#define SQ_DEBUG_TP_FSM__GS_TP__SHIFT                      0x00000010
#define SQ_DEBUG_TP_FSM__RESERVED3__SHIFT                  0x00000012
#define SQ_DEBUG_TP_FSM__FCR_TP__SHIFT                     0x00000014
#define SQ_DEBUG_TP_FSM__RESERVED4__SHIFT                  0x00000016
#define SQ_DEBUG_TP_FSM__FCS_TP__SHIFT                     0x00000018
#define SQ_DEBUG_TP_FSM__RESERVED5__SHIFT                  0x0000001a
#define SQ_DEBUG_TP_FSM__ARB_TR_TP__SHIFT                  0x0000001c

// SQ_DEBUG_FSM_ALU_0
#define SQ_DEBUG_FSM_ALU_0__EX_ALU_0__SHIFT                0x00000000
#define SQ_DEBUG_FSM_ALU_0__RESERVED0__SHIFT               0x00000003
#define SQ_DEBUG_FSM_ALU_0__CF_ALU_0__SHIFT                0x00000004
#define SQ_DEBUG_FSM_ALU_0__IF_ALU_0__SHIFT                0x00000008
#define SQ_DEBUG_FSM_ALU_0__RESERVED1__SHIFT               0x0000000b
#define SQ_DEBUG_FSM_ALU_0__DU1_ALU_0__SHIFT               0x0000000c
#define SQ_DEBUG_FSM_ALU_0__RESERVED2__SHIFT               0x0000000f
#define SQ_DEBUG_FSM_ALU_0__DU0_ALU_0__SHIFT               0x00000010
#define SQ_DEBUG_FSM_ALU_0__RESERVED3__SHIFT               0x00000013
#define SQ_DEBUG_FSM_ALU_0__AIS_ALU_0__SHIFT               0x00000014
#define SQ_DEBUG_FSM_ALU_0__RESERVED4__SHIFT               0x00000017
#define SQ_DEBUG_FSM_ALU_0__ACS_ALU_0__SHIFT               0x00000018
#define SQ_DEBUG_FSM_ALU_0__RESERVED5__SHIFT               0x0000001b
#define SQ_DEBUG_FSM_ALU_0__ARB_TR_ALU__SHIFT              0x0000001c

// SQ_DEBUG_FSM_ALU_1
#define SQ_DEBUG_FSM_ALU_1__EX_ALU_0__SHIFT                0x00000000
#define SQ_DEBUG_FSM_ALU_1__RESERVED0__SHIFT               0x00000003
#define SQ_DEBUG_FSM_ALU_1__CF_ALU_0__SHIFT                0x00000004
#define SQ_DEBUG_FSM_ALU_1__IF_ALU_0__SHIFT                0x00000008
#define SQ_DEBUG_FSM_ALU_1__RESERVED1__SHIFT               0x0000000b
#define SQ_DEBUG_FSM_ALU_1__DU1_ALU_0__SHIFT               0x0000000c
#define SQ_DEBUG_FSM_ALU_1__RESERVED2__SHIFT               0x0000000f
#define SQ_DEBUG_FSM_ALU_1__DU0_ALU_0__SHIFT               0x00000010
#define SQ_DEBUG_FSM_ALU_1__RESERVED3__SHIFT               0x00000013
#define SQ_DEBUG_FSM_ALU_1__AIS_ALU_0__SHIFT               0x00000014
#define SQ_DEBUG_FSM_ALU_1__RESERVED4__SHIFT               0x00000017
#define SQ_DEBUG_FSM_ALU_1__ACS_ALU_0__SHIFT               0x00000018
#define SQ_DEBUG_FSM_ALU_1__RESERVED5__SHIFT               0x0000001b
#define SQ_DEBUG_FSM_ALU_1__ARB_TR_ALU__SHIFT              0x0000001c

// SQ_DEBUG_EXP_ALLOC
#define SQ_DEBUG_EXP_ALLOC__POS_BUF_AVAIL__SHIFT           0x00000000
#define SQ_DEBUG_EXP_ALLOC__COLOR_BUF_AVAIL__SHIFT         0x00000004
#define SQ_DEBUG_EXP_ALLOC__EA_BUF_AVAIL__SHIFT            0x0000000c
#define SQ_DEBUG_EXP_ALLOC__RESERVED__SHIFT                0x0000000f
#define SQ_DEBUG_EXP_ALLOC__ALLOC_TBL_BUF_AVAIL__SHIFT     0x00000010

// SQ_DEBUG_PTR_BUFF
#define SQ_DEBUG_PTR_BUFF__END_OF_BUFFER__SHIFT            0x00000000
#define SQ_DEBUG_PTR_BUFF__DEALLOC_CNT__SHIFT              0x00000001
#define SQ_DEBUG_PTR_BUFF__QUAL_NEW_VECTOR__SHIFT          0x00000005
#define SQ_DEBUG_PTR_BUFF__EVENT_CONTEXT_ID__SHIFT         0x00000006
#define SQ_DEBUG_PTR_BUFF__SC_EVENT_ID__SHIFT              0x00000009
#define SQ_DEBUG_PTR_BUFF__QUAL_EVENT__SHIFT               0x0000000e
#define SQ_DEBUG_PTR_BUFF__PRIM_TYPE_POLYGON__SHIFT        0x0000000f
#define SQ_DEBUG_PTR_BUFF__EF_EMPTY__SHIFT                 0x00000010
#define SQ_DEBUG_PTR_BUFF__VTX_SYNC_CNT__SHIFT             0x00000011

// SQ_DEBUG_GPR_VTX
#define SQ_DEBUG_GPR_VTX__VTX_TAIL_PTR__SHIFT              0x00000000
#define SQ_DEBUG_GPR_VTX__RESERVED__SHIFT                  0x00000007
#define SQ_DEBUG_GPR_VTX__VTX_HEAD_PTR__SHIFT              0x00000008
#define SQ_DEBUG_GPR_VTX__RESERVED1__SHIFT                 0x0000000f
#define SQ_DEBUG_GPR_VTX__VTX_MAX__SHIFT                   0x00000010
#define SQ_DEBUG_GPR_VTX__RESERVED2__SHIFT                 0x00000017
#define SQ_DEBUG_GPR_VTX__VTX_FREE__SHIFT                  0x00000018

// SQ_DEBUG_GPR_PIX
#define SQ_DEBUG_GPR_PIX__PIX_TAIL_PTR__SHIFT              0x00000000
#define SQ_DEBUG_GPR_PIX__RESERVED__SHIFT                  0x00000007
#define SQ_DEBUG_GPR_PIX__PIX_HEAD_PTR__SHIFT              0x00000008
#define SQ_DEBUG_GPR_PIX__RESERVED1__SHIFT                 0x0000000f
#define SQ_DEBUG_GPR_PIX__PIX_MAX__SHIFT                   0x00000010
#define SQ_DEBUG_GPR_PIX__RESERVED2__SHIFT                 0x00000017
#define SQ_DEBUG_GPR_PIX__PIX_FREE__SHIFT                  0x00000018

// SQ_DEBUG_TB_STATUS_SEL
#define SQ_DEBUG_TB_STATUS_SEL__VTX_TB_STATUS_REG_SEL__SHIFT 0x00000000
#define SQ_DEBUG_TB_STATUS_SEL__VTX_TB_STATE_MEM_DW_SEL__SHIFT 0x00000004
#define SQ_DEBUG_TB_STATUS_SEL__VTX_TB_STATE_MEM_RD_ADDR__SHIFT 0x00000007
#define SQ_DEBUG_TB_STATUS_SEL__VTX_TB_STATE_MEM_RD_EN__SHIFT 0x0000000b
#define SQ_DEBUG_TB_STATUS_SEL__PIX_TB_STATE_MEM_RD_EN__SHIFT 0x0000000c
#define SQ_DEBUG_TB_STATUS_SEL__DEBUG_BUS_TRIGGER_SEL__SHIFT 0x0000000e
#define SQ_DEBUG_TB_STATUS_SEL__PIX_TB_STATUS_REG_SEL__SHIFT 0x00000010
#define SQ_DEBUG_TB_STATUS_SEL__PIX_TB_STATE_MEM_DW_SEL__SHIFT 0x00000014
#define SQ_DEBUG_TB_STATUS_SEL__PIX_TB_STATE_MEM_RD_ADDR__SHIFT 0x00000017
#define SQ_DEBUG_TB_STATUS_SEL__VC_THREAD_BUF_DLY__SHIFT   0x0000001d
#define SQ_DEBUG_TB_STATUS_SEL__DISABLE_STRICT_CTX_SYNC__SHIFT 0x0000001f

// SQ_DEBUG_VTX_TB_0
#define SQ_DEBUG_VTX_TB_0__VTX_HEAD_PTR_Q__SHIFT           0x00000000
#define SQ_DEBUG_VTX_TB_0__TAIL_PTR_Q__SHIFT               0x00000004
#define SQ_DEBUG_VTX_TB_0__FULL_CNT_Q__SHIFT               0x00000008
#define SQ_DEBUG_VTX_TB_0__NXT_POS_ALLOC_CNT__SHIFT        0x0000000c
#define SQ_DEBUG_VTX_TB_0__NXT_PC_ALLOC_CNT__SHIFT         0x00000010
#define SQ_DEBUG_VTX_TB_0__SX_EVENT_FULL__SHIFT            0x00000014
#define SQ_DEBUG_VTX_TB_0__BUSY_Q__SHIFT                   0x00000015

// SQ_DEBUG_VTX_TB_1
#define SQ_DEBUG_VTX_TB_1__VS_DONE_PTR__SHIFT              0x00000000

// SQ_DEBUG_VTX_TB_STATUS_REG
#define SQ_DEBUG_VTX_TB_STATUS_REG__VS_STATUS_REG__SHIFT   0x00000000

// SQ_DEBUG_VTX_TB_STATE_MEM
#define SQ_DEBUG_VTX_TB_STATE_MEM__VS_STATE_MEM__SHIFT     0x00000000

// SQ_DEBUG_PIX_TB_0
#define SQ_DEBUG_PIX_TB_0__PIX_HEAD_PTR__SHIFT             0x00000000
#define SQ_DEBUG_PIX_TB_0__TAIL_PTR__SHIFT                 0x00000006
#define SQ_DEBUG_PIX_TB_0__FULL_CNT__SHIFT                 0x0000000c
#define SQ_DEBUG_PIX_TB_0__NXT_PIX_ALLOC_CNT__SHIFT        0x00000013
#define SQ_DEBUG_PIX_TB_0__NXT_PIX_EXP_CNT__SHIFT          0x00000019
#define SQ_DEBUG_PIX_TB_0__BUSY__SHIFT                     0x0000001f

// SQ_DEBUG_PIX_TB_STATUS_REG_0
#define SQ_DEBUG_PIX_TB_STATUS_REG_0__PIX_TB_STATUS_REG_0__SHIFT 0x00000000

// SQ_DEBUG_PIX_TB_STATUS_REG_1
#define SQ_DEBUG_PIX_TB_STATUS_REG_1__PIX_TB_STATUS_REG_1__SHIFT 0x00000000

// SQ_DEBUG_PIX_TB_STATUS_REG_2
#define SQ_DEBUG_PIX_TB_STATUS_REG_2__PIX_TB_STATUS_REG_2__SHIFT 0x00000000

// SQ_DEBUG_PIX_TB_STATUS_REG_3
#define SQ_DEBUG_PIX_TB_STATUS_REG_3__PIX_TB_STATUS_REG_3__SHIFT 0x00000000

// SQ_DEBUG_PIX_TB_STATE_MEM
#define SQ_DEBUG_PIX_TB_STATE_MEM__PIX_TB_STATE_MEM__SHIFT 0x00000000

// SQ_PERFCOUNTER0_SELECT
#define SQ_PERFCOUNTER0_SELECT__PERF_SEL__SHIFT            0x00000000

// SQ_PERFCOUNTER1_SELECT
#define SQ_PERFCOUNTER1_SELECT__PERF_SEL__SHIFT            0x00000000

// SQ_PERFCOUNTER2_SELECT
#define SQ_PERFCOUNTER2_SELECT__PERF_SEL__SHIFT            0x00000000

// SQ_PERFCOUNTER3_SELECT
#define SQ_PERFCOUNTER3_SELECT__PERF_SEL__SHIFT            0x00000000

// SQ_PERFCOUNTER0_LOW
#define SQ_PERFCOUNTER0_LOW__PERF_COUNT__SHIFT             0x00000000

// SQ_PERFCOUNTER0_HI
#define SQ_PERFCOUNTER0_HI__PERF_COUNT__SHIFT              0x00000000

// SQ_PERFCOUNTER1_LOW
#define SQ_PERFCOUNTER1_LOW__PERF_COUNT__SHIFT             0x00000000

// SQ_PERFCOUNTER1_HI
#define SQ_PERFCOUNTER1_HI__PERF_COUNT__SHIFT              0x00000000

// SQ_PERFCOUNTER2_LOW
#define SQ_PERFCOUNTER2_LOW__PERF_COUNT__SHIFT             0x00000000

// SQ_PERFCOUNTER2_HI
#define SQ_PERFCOUNTER2_HI__PERF_COUNT__SHIFT              0x00000000

// SQ_PERFCOUNTER3_LOW
#define SQ_PERFCOUNTER3_LOW__PERF_COUNT__SHIFT             0x00000000

// SQ_PERFCOUNTER3_HI
#define SQ_PERFCOUNTER3_HI__PERF_COUNT__SHIFT              0x00000000

// SX_PERFCOUNTER0_SELECT
#define SX_PERFCOUNTER0_SELECT__PERF_SEL__SHIFT            0x00000000

// SX_PERFCOUNTER0_LOW
#define SX_PERFCOUNTER0_LOW__PERF_COUNT__SHIFT             0x00000000

// SX_PERFCOUNTER0_HI
#define SX_PERFCOUNTER0_HI__PERF_COUNT__SHIFT              0x00000000

// SQ_INSTRUCTION_ALU_0
#define SQ_INSTRUCTION_ALU_0__VECTOR_RESULT__SHIFT         0x00000000
#define SQ_INSTRUCTION_ALU_0__VECTOR_DST_REL__SHIFT        0x00000006
#define SQ_INSTRUCTION_ALU_0__LOW_PRECISION_16B_FP__SHIFT  0x00000007
#define SQ_INSTRUCTION_ALU_0__SCALAR_RESULT__SHIFT         0x00000008
#define SQ_INSTRUCTION_ALU_0__SCALAR_DST_REL__SHIFT        0x0000000e
#define SQ_INSTRUCTION_ALU_0__EXPORT_DATA__SHIFT           0x0000000f
#define SQ_INSTRUCTION_ALU_0__VECTOR_WRT_MSK__SHIFT        0x00000010
#define SQ_INSTRUCTION_ALU_0__SCALAR_WRT_MSK__SHIFT        0x00000014
#define SQ_INSTRUCTION_ALU_0__VECTOR_CLAMP__SHIFT          0x00000018
#define SQ_INSTRUCTION_ALU_0__SCALAR_CLAMP__SHIFT          0x00000019
#define SQ_INSTRUCTION_ALU_0__SCALAR_OPCODE__SHIFT         0x0000001a

// SQ_INSTRUCTION_ALU_1
#define SQ_INSTRUCTION_ALU_1__SRC_C_SWIZZLE_R__SHIFT       0x00000000
#define SQ_INSTRUCTION_ALU_1__SRC_C_SWIZZLE_G__SHIFT       0x00000002
#define SQ_INSTRUCTION_ALU_1__SRC_C_SWIZZLE_B__SHIFT       0x00000004
#define SQ_INSTRUCTION_ALU_1__SRC_C_SWIZZLE_A__SHIFT       0x00000006
#define SQ_INSTRUCTION_ALU_1__SRC_B_SWIZZLE_R__SHIFT       0x00000008
#define SQ_INSTRUCTION_ALU_1__SRC_B_SWIZZLE_G__SHIFT       0x0000000a
#define SQ_INSTRUCTION_ALU_1__SRC_B_SWIZZLE_B__SHIFT       0x0000000c
#define SQ_INSTRUCTION_ALU_1__SRC_B_SWIZZLE_A__SHIFT       0x0000000e
#define SQ_INSTRUCTION_ALU_1__SRC_A_SWIZZLE_R__SHIFT       0x00000010
#define SQ_INSTRUCTION_ALU_1__SRC_A_SWIZZLE_G__SHIFT       0x00000012
#define SQ_INSTRUCTION_ALU_1__SRC_A_SWIZZLE_B__SHIFT       0x00000014
#define SQ_INSTRUCTION_ALU_1__SRC_A_SWIZZLE_A__SHIFT       0x00000016
#define SQ_INSTRUCTION_ALU_1__SRC_C_ARG_MOD__SHIFT         0x00000018
#define SQ_INSTRUCTION_ALU_1__SRC_B_ARG_MOD__SHIFT         0x00000019
#define SQ_INSTRUCTION_ALU_1__SRC_A_ARG_MOD__SHIFT         0x0000001a
#define SQ_INSTRUCTION_ALU_1__PRED_SELECT__SHIFT           0x0000001b
#define SQ_INSTRUCTION_ALU_1__RELATIVE_ADDR__SHIFT         0x0000001d
#define SQ_INSTRUCTION_ALU_1__CONST_1_REL_ABS__SHIFT       0x0000001e
#define SQ_INSTRUCTION_ALU_1__CONST_0_REL_ABS__SHIFT       0x0000001f

// SQ_INSTRUCTION_ALU_2
#define SQ_INSTRUCTION_ALU_2__SRC_C_REG_PTR__SHIFT         0x00000000
#define SQ_INSTRUCTION_ALU_2__REG_SELECT_C__SHIFT          0x00000006
#define SQ_INSTRUCTION_ALU_2__REG_ABS_MOD_C__SHIFT         0x00000007
#define SQ_INSTRUCTION_ALU_2__SRC_B_REG_PTR__SHIFT         0x00000008
#define SQ_INSTRUCTION_ALU_2__REG_SELECT_B__SHIFT          0x0000000e
#define SQ_INSTRUCTION_ALU_2__REG_ABS_MOD_B__SHIFT         0x0000000f
#define SQ_INSTRUCTION_ALU_2__SRC_A_REG_PTR__SHIFT         0x00000010
#define SQ_INSTRUCTION_ALU_2__REG_SELECT_A__SHIFT          0x00000016
#define SQ_INSTRUCTION_ALU_2__REG_ABS_MOD_A__SHIFT         0x00000017
#define SQ_INSTRUCTION_ALU_2__VECTOR_OPCODE__SHIFT         0x00000018
#define SQ_INSTRUCTION_ALU_2__SRC_C_SEL__SHIFT             0x0000001d
#define SQ_INSTRUCTION_ALU_2__SRC_B_SEL__SHIFT             0x0000001e
#define SQ_INSTRUCTION_ALU_2__SRC_A_SEL__SHIFT             0x0000001f

// SQ_INSTRUCTION_CF_EXEC_0
#define SQ_INSTRUCTION_CF_EXEC_0__ADDRESS__SHIFT           0x00000000
#define SQ_INSTRUCTION_CF_EXEC_0__RESERVED__SHIFT          0x00000009
#define SQ_INSTRUCTION_CF_EXEC_0__COUNT__SHIFT             0x0000000c
#define SQ_INSTRUCTION_CF_EXEC_0__YIELD__SHIFT             0x0000000f
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_0__SHIFT       0x00000010
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_0__SHIFT     0x00000011
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_1__SHIFT       0x00000012
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_1__SHIFT     0x00000013
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_2__SHIFT       0x00000014
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_2__SHIFT     0x00000015
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_3__SHIFT       0x00000016
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_3__SHIFT     0x00000017
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_4__SHIFT       0x00000018
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_4__SHIFT     0x00000019
#define SQ_INSTRUCTION_CF_EXEC_0__INST_TYPE_5__SHIFT       0x0000001a
#define SQ_INSTRUCTION_CF_EXEC_0__INST_SERIAL_5__SHIFT     0x0000001b
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_0__SHIFT         0x0000001c
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_1__SHIFT         0x0000001d
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_2__SHIFT         0x0000001e
#define SQ_INSTRUCTION_CF_EXEC_0__INST_VC_3__SHIFT         0x0000001f

// SQ_INSTRUCTION_CF_EXEC_1
#define SQ_INSTRUCTION_CF_EXEC_1__INST_VC_4__SHIFT         0x00000000
#define SQ_INSTRUCTION_CF_EXEC_1__INST_VC_5__SHIFT         0x00000001
#define SQ_INSTRUCTION_CF_EXEC_1__BOOL_ADDR__SHIFT         0x00000002
#define SQ_INSTRUCTION_CF_EXEC_1__CONDITION__SHIFT         0x0000000a
#define SQ_INSTRUCTION_CF_EXEC_1__ADDRESS_MODE__SHIFT      0x0000000b
#define SQ_INSTRUCTION_CF_EXEC_1__OPCODE__SHIFT            0x0000000c
#define SQ_INSTRUCTION_CF_EXEC_1__ADDRESS__SHIFT           0x00000010
#define SQ_INSTRUCTION_CF_EXEC_1__RESERVED__SHIFT          0x00000019
#define SQ_INSTRUCTION_CF_EXEC_1__COUNT__SHIFT             0x0000001c
#define SQ_INSTRUCTION_CF_EXEC_1__YIELD__SHIFT             0x0000001f

// SQ_INSTRUCTION_CF_EXEC_2
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_0__SHIFT       0x00000000
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_0__SHIFT     0x00000001
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_1__SHIFT       0x00000002
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_1__SHIFT     0x00000003
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_2__SHIFT       0x00000004
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_2__SHIFT     0x00000005
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_3__SHIFT       0x00000006
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_3__SHIFT     0x00000007
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_4__SHIFT       0x00000008
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_4__SHIFT     0x00000009
#define SQ_INSTRUCTION_CF_EXEC_2__INST_TYPE_5__SHIFT       0x0000000a
#define SQ_INSTRUCTION_CF_EXEC_2__INST_SERIAL_5__SHIFT     0x0000000b
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_0__SHIFT         0x0000000c
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_1__SHIFT         0x0000000d
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_2__SHIFT         0x0000000e
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_3__SHIFT         0x0000000f
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_4__SHIFT         0x00000010
#define SQ_INSTRUCTION_CF_EXEC_2__INST_VC_5__SHIFT         0x00000011
#define SQ_INSTRUCTION_CF_EXEC_2__BOOL_ADDR__SHIFT         0x00000012
#define SQ_INSTRUCTION_CF_EXEC_2__CONDITION__SHIFT         0x0000001a
#define SQ_INSTRUCTION_CF_EXEC_2__ADDRESS_MODE__SHIFT      0x0000001b
#define SQ_INSTRUCTION_CF_EXEC_2__OPCODE__SHIFT            0x0000001c

// SQ_INSTRUCTION_CF_LOOP_0
#define SQ_INSTRUCTION_CF_LOOP_0__ADDRESS__SHIFT           0x00000000
#define SQ_INSTRUCTION_CF_LOOP_0__RESERVED_0__SHIFT        0x0000000a
#define SQ_INSTRUCTION_CF_LOOP_0__LOOP_ID__SHIFT           0x00000010
#define SQ_INSTRUCTION_CF_LOOP_0__RESERVED_1__SHIFT        0x00000015

// SQ_INSTRUCTION_CF_LOOP_1
#define SQ_INSTRUCTION_CF_LOOP_1__RESERVED_0__SHIFT        0x00000000
#define SQ_INSTRUCTION_CF_LOOP_1__ADDRESS_MODE__SHIFT      0x0000000b
#define SQ_INSTRUCTION_CF_LOOP_1__OPCODE__SHIFT            0x0000000c
#define SQ_INSTRUCTION_CF_LOOP_1__ADDRESS__SHIFT           0x00000010
#define SQ_INSTRUCTION_CF_LOOP_1__RESERVED_1__SHIFT        0x0000001a

// SQ_INSTRUCTION_CF_LOOP_2
#define SQ_INSTRUCTION_CF_LOOP_2__LOOP_ID__SHIFT           0x00000000
#define SQ_INSTRUCTION_CF_LOOP_2__RESERVED__SHIFT          0x00000005
#define SQ_INSTRUCTION_CF_LOOP_2__ADDRESS_MODE__SHIFT      0x0000001b
#define SQ_INSTRUCTION_CF_LOOP_2__OPCODE__SHIFT            0x0000001c

// SQ_INSTRUCTION_CF_JMP_CALL_0
#define SQ_INSTRUCTION_CF_JMP_CALL_0__ADDRESS__SHIFT       0x00000000
#define SQ_INSTRUCTION_CF_JMP_CALL_0__RESERVED_0__SHIFT    0x0000000a
#define SQ_INSTRUCTION_CF_JMP_CALL_0__FORCE_CALL__SHIFT    0x0000000d
#define SQ_INSTRUCTION_CF_JMP_CALL_0__PREDICATED_JMP__SHIFT 0x0000000e
#define SQ_INSTRUCTION_CF_JMP_CALL_0__RESERVED_1__SHIFT    0x0000000f

// SQ_INSTRUCTION_CF_JMP_CALL_1
#define SQ_INSTRUCTION_CF_JMP_CALL_1__RESERVED_0__SHIFT    0x00000000
#define SQ_INSTRUCTION_CF_JMP_CALL_1__DIRECTION__SHIFT     0x00000001
#define SQ_INSTRUCTION_CF_JMP_CALL_1__BOOL_ADDR__SHIFT     0x00000002
#define SQ_INSTRUCTION_CF_JMP_CALL_1__CONDITION__SHIFT     0x0000000a
#define SQ_INSTRUCTION_CF_JMP_CALL_1__ADDRESS_MODE__SHIFT  0x0000000b
#define SQ_INSTRUCTION_CF_JMP_CALL_1__OPCODE__SHIFT        0x0000000c
#define SQ_INSTRUCTION_CF_JMP_CALL_1__ADDRESS__SHIFT       0x00000010
#define SQ_INSTRUCTION_CF_JMP_CALL_1__RESERVED_1__SHIFT    0x0000001a
#define SQ_INSTRUCTION_CF_JMP_CALL_1__FORCE_CALL__SHIFT    0x0000001d
#define SQ_INSTRUCTION_CF_JMP_CALL_1__RESERVED_2__SHIFT    0x0000001e

// SQ_INSTRUCTION_CF_JMP_CALL_2
#define SQ_INSTRUCTION_CF_JMP_CALL_2__RESERVED__SHIFT      0x00000000
#define SQ_INSTRUCTION_CF_JMP_CALL_2__DIRECTION__SHIFT     0x00000011
#define SQ_INSTRUCTION_CF_JMP_CALL_2__BOOL_ADDR__SHIFT     0x00000012
#define SQ_INSTRUCTION_CF_JMP_CALL_2__CONDITION__SHIFT     0x0000001a
#define SQ_INSTRUCTION_CF_JMP_CALL_2__ADDRESS_MODE__SHIFT  0x0000001b
#define SQ_INSTRUCTION_CF_JMP_CALL_2__OPCODE__SHIFT        0x0000001c

// SQ_INSTRUCTION_CF_ALLOC_0
#define SQ_INSTRUCTION_CF_ALLOC_0__SIZE__SHIFT             0x00000000
#define SQ_INSTRUCTION_CF_ALLOC_0__RESERVED__SHIFT         0x00000004

// SQ_INSTRUCTION_CF_ALLOC_1
#define SQ_INSTRUCTION_CF_ALLOC_1__RESERVED_0__SHIFT       0x00000000
#define SQ_INSTRUCTION_CF_ALLOC_1__NO_SERIAL__SHIFT        0x00000008
#define SQ_INSTRUCTION_CF_ALLOC_1__BUFFER_SELECT__SHIFT    0x00000009
#define SQ_INSTRUCTION_CF_ALLOC_1__ALLOC_MODE__SHIFT       0x0000000b
#define SQ_INSTRUCTION_CF_ALLOC_1__OPCODE__SHIFT           0x0000000c
#define SQ_INSTRUCTION_CF_ALLOC_1__SIZE__SHIFT             0x00000010
#define SQ_INSTRUCTION_CF_ALLOC_1__RESERVED_1__SHIFT       0x00000014

// SQ_INSTRUCTION_CF_ALLOC_2
#define SQ_INSTRUCTION_CF_ALLOC_2__RESERVED__SHIFT         0x00000000
#define SQ_INSTRUCTION_CF_ALLOC_2__NO_SERIAL__SHIFT        0x00000018
#define SQ_INSTRUCTION_CF_ALLOC_2__BUFFER_SELECT__SHIFT    0x00000019
#define SQ_INSTRUCTION_CF_ALLOC_2__ALLOC_MODE__SHIFT       0x0000001b
#define SQ_INSTRUCTION_CF_ALLOC_2__OPCODE__SHIFT           0x0000001c

// SQ_INSTRUCTION_TFETCH_0
#define SQ_INSTRUCTION_TFETCH_0__OPCODE__SHIFT             0x00000000
#define SQ_INSTRUCTION_TFETCH_0__SRC_GPR__SHIFT            0x00000005
#define SQ_INSTRUCTION_TFETCH_0__SRC_GPR_AM__SHIFT         0x0000000b
#define SQ_INSTRUCTION_TFETCH_0__DST_GPR__SHIFT            0x0000000c
#define SQ_INSTRUCTION_TFETCH_0__DST_GPR_AM__SHIFT         0x00000012
#define SQ_INSTRUCTION_TFETCH_0__FETCH_VALID_ONLY__SHIFT   0x00000013
#define SQ_INSTRUCTION_TFETCH_0__CONST_INDEX__SHIFT        0x00000014
#define SQ_INSTRUCTION_TFETCH_0__TX_COORD_DENORM__SHIFT    0x00000019
#define SQ_INSTRUCTION_TFETCH_0__SRC_SEL_X__SHIFT          0x0000001a
#define SQ_INSTRUCTION_TFETCH_0__SRC_SEL_Y__SHIFT          0x0000001c
#define SQ_INSTRUCTION_TFETCH_0__SRC_SEL_Z__SHIFT          0x0000001e

// SQ_INSTRUCTION_TFETCH_1
#define SQ_INSTRUCTION_TFETCH_1__DST_SEL_X__SHIFT          0x00000000
#define SQ_INSTRUCTION_TFETCH_1__DST_SEL_Y__SHIFT          0x00000003
#define SQ_INSTRUCTION_TFETCH_1__DST_SEL_Z__SHIFT          0x00000006
#define SQ_INSTRUCTION_TFETCH_1__DST_SEL_W__SHIFT          0x00000009
#define SQ_INSTRUCTION_TFETCH_1__MAG_FILTER__SHIFT         0x0000000c
#define SQ_INSTRUCTION_TFETCH_1__MIN_FILTER__SHIFT         0x0000000e
#define SQ_INSTRUCTION_TFETCH_1__MIP_FILTER__SHIFT         0x00000010
#define SQ_INSTRUCTION_TFETCH_1__ANISO_FILTER__SHIFT       0x00000012
#define SQ_INSTRUCTION_TFETCH_1__ARBITRARY_FILTER__SHIFT   0x00000015
#define SQ_INSTRUCTION_TFETCH_1__VOL_MAG_FILTER__SHIFT     0x00000018
#define SQ_INSTRUCTION_TFETCH_1__VOL_MIN_FILTER__SHIFT     0x0000001a
#define SQ_INSTRUCTION_TFETCH_1__USE_COMP_LOD__SHIFT       0x0000001c
#define SQ_INSTRUCTION_TFETCH_1__USE_REG_LOD__SHIFT        0x0000001d
#define SQ_INSTRUCTION_TFETCH_1__PRED_SELECT__SHIFT        0x0000001f

// SQ_INSTRUCTION_TFETCH_2
#define SQ_INSTRUCTION_TFETCH_2__USE_REG_GRADIENTS__SHIFT  0x00000000
#define SQ_INSTRUCTION_TFETCH_2__SAMPLE_LOCATION__SHIFT    0x00000001
#define SQ_INSTRUCTION_TFETCH_2__LOD_BIAS__SHIFT           0x00000002
#define SQ_INSTRUCTION_TFETCH_2__UNUSED__SHIFT             0x00000009
#define SQ_INSTRUCTION_TFETCH_2__OFFSET_X__SHIFT           0x00000010
#define SQ_INSTRUCTION_TFETCH_2__OFFSET_Y__SHIFT           0x00000015
#define SQ_INSTRUCTION_TFETCH_2__OFFSET_Z__SHIFT           0x0000001a
#define SQ_INSTRUCTION_TFETCH_2__PRED_CONDITION__SHIFT     0x0000001f

// SQ_INSTRUCTION_VFETCH_0
#define SQ_INSTRUCTION_VFETCH_0__OPCODE__SHIFT             0x00000000
#define SQ_INSTRUCTION_VFETCH_0__SRC_GPR__SHIFT            0x00000005
#define SQ_INSTRUCTION_VFETCH_0__SRC_GPR_AM__SHIFT         0x0000000b
#define SQ_INSTRUCTION_VFETCH_0__DST_GPR__SHIFT            0x0000000c
#define SQ_INSTRUCTION_VFETCH_0__DST_GPR_AM__SHIFT         0x00000012
#define SQ_INSTRUCTION_VFETCH_0__MUST_BE_ONE__SHIFT        0x00000013
#define SQ_INSTRUCTION_VFETCH_0__CONST_INDEX__SHIFT        0x00000014
#define SQ_INSTRUCTION_VFETCH_0__CONST_INDEX_SEL__SHIFT    0x00000019
#define SQ_INSTRUCTION_VFETCH_0__SRC_SEL__SHIFT            0x0000001e

// SQ_INSTRUCTION_VFETCH_1
#define SQ_INSTRUCTION_VFETCH_1__DST_SEL_X__SHIFT          0x00000000
#define SQ_INSTRUCTION_VFETCH_1__DST_SEL_Y__SHIFT          0x00000003
#define SQ_INSTRUCTION_VFETCH_1__DST_SEL_Z__SHIFT          0x00000006
#define SQ_INSTRUCTION_VFETCH_1__DST_SEL_W__SHIFT          0x00000009
#define SQ_INSTRUCTION_VFETCH_1__FORMAT_COMP_ALL__SHIFT    0x0000000c
#define SQ_INSTRUCTION_VFETCH_1__NUM_FORMAT_ALL__SHIFT     0x0000000d
#define SQ_INSTRUCTION_VFETCH_1__SIGNED_RF_MODE_ALL__SHIFT 0x0000000e
#define SQ_INSTRUCTION_VFETCH_1__DATA_FORMAT__SHIFT        0x00000010
#define SQ_INSTRUCTION_VFETCH_1__EXP_ADJUST_ALL__SHIFT     0x00000017
#define SQ_INSTRUCTION_VFETCH_1__PRED_SELECT__SHIFT        0x0000001f

// SQ_INSTRUCTION_VFETCH_2
#define SQ_INSTRUCTION_VFETCH_2__STRIDE__SHIFT             0x00000000
#define SQ_INSTRUCTION_VFETCH_2__OFFSET__SHIFT             0x00000010
#define SQ_INSTRUCTION_VFETCH_2__PRED_CONDITION__SHIFT     0x0000001f

// SQ_CONSTANT_0
#define SQ_CONSTANT_0__RED__SHIFT                          0x00000000

// SQ_CONSTANT_1
#define SQ_CONSTANT_1__GREEN__SHIFT                        0x00000000

// SQ_CONSTANT_2
#define SQ_CONSTANT_2__BLUE__SHIFT                         0x00000000

// SQ_CONSTANT_3
#define SQ_CONSTANT_3__ALPHA__SHIFT                        0x00000000

// SQ_FETCH_0
#define SQ_FETCH_0__VALUE__SHIFT                           0x00000000

// SQ_FETCH_1
#define SQ_FETCH_1__VALUE__SHIFT                           0x00000000

// SQ_FETCH_2
#define SQ_FETCH_2__VALUE__SHIFT                           0x00000000

// SQ_FETCH_3
#define SQ_FETCH_3__VALUE__SHIFT                           0x00000000

// SQ_FETCH_4
#define SQ_FETCH_4__VALUE__SHIFT                           0x00000000

// SQ_FETCH_5
#define SQ_FETCH_5__VALUE__SHIFT                           0x00000000

// SQ_CONSTANT_VFETCH_0
#define SQ_CONSTANT_VFETCH_0__TYPE__SHIFT                  0x00000000
#define SQ_CONSTANT_VFETCH_0__STATE__SHIFT                 0x00000001
#define SQ_CONSTANT_VFETCH_0__BASE_ADDRESS__SHIFT          0x00000002

// SQ_CONSTANT_VFETCH_1
#define SQ_CONSTANT_VFETCH_1__ENDIAN_SWAP__SHIFT           0x00000000
#define SQ_CONSTANT_VFETCH_1__LIMIT_ADDRESS__SHIFT         0x00000002

// SQ_CONSTANT_T2
#define SQ_CONSTANT_T2__VALUE__SHIFT                       0x00000000

// SQ_CONSTANT_T3
#define SQ_CONSTANT_T3__VALUE__SHIFT                       0x00000000

// SQ_CF_BOOLEANS
#define SQ_CF_BOOLEANS__CF_BOOLEANS_0__SHIFT               0x00000000
#define SQ_CF_BOOLEANS__CF_BOOLEANS_1__SHIFT               0x00000008
#define SQ_CF_BOOLEANS__CF_BOOLEANS_2__SHIFT               0x00000010
#define SQ_CF_BOOLEANS__CF_BOOLEANS_3__SHIFT               0x00000018

// SQ_CF_LOOP
#define SQ_CF_LOOP__CF_LOOP_COUNT__SHIFT                   0x00000000
#define SQ_CF_LOOP__CF_LOOP_START__SHIFT                   0x00000008
#define SQ_CF_LOOP__CF_LOOP_STEP__SHIFT                    0x00000010

// SQ_CONSTANT_RT_0
#define SQ_CONSTANT_RT_0__RED__SHIFT                       0x00000000

// SQ_CONSTANT_RT_1
#define SQ_CONSTANT_RT_1__GREEN__SHIFT                     0x00000000

// SQ_CONSTANT_RT_2
#define SQ_CONSTANT_RT_2__BLUE__SHIFT                      0x00000000

// SQ_CONSTANT_RT_3
#define SQ_CONSTANT_RT_3__ALPHA__SHIFT                     0x00000000

// SQ_FETCH_RT_0
#define SQ_FETCH_RT_0__VALUE__SHIFT                        0x00000000

// SQ_FETCH_RT_1
#define SQ_FETCH_RT_1__VALUE__SHIFT                        0x00000000

// SQ_FETCH_RT_2
#define SQ_FETCH_RT_2__VALUE__SHIFT                        0x00000000

// SQ_FETCH_RT_3
#define SQ_FETCH_RT_3__VALUE__SHIFT                        0x00000000

// SQ_FETCH_RT_4
#define SQ_FETCH_RT_4__VALUE__SHIFT                        0x00000000

// SQ_FETCH_RT_5
#define SQ_FETCH_RT_5__VALUE__SHIFT                        0x00000000

// SQ_CF_RT_BOOLEANS
#define SQ_CF_RT_BOOLEANS__CF_BOOLEANS_0__SHIFT            0x00000000
#define SQ_CF_RT_BOOLEANS__CF_BOOLEANS_1__SHIFT            0x00000008
#define SQ_CF_RT_BOOLEANS__CF_BOOLEANS_2__SHIFT            0x00000010
#define SQ_CF_RT_BOOLEANS__CF_BOOLEANS_3__SHIFT            0x00000018

// SQ_CF_RT_LOOP
#define SQ_CF_RT_LOOP__CF_LOOP_COUNT__SHIFT                0x00000000
#define SQ_CF_RT_LOOP__CF_LOOP_START__SHIFT                0x00000008
#define SQ_CF_RT_LOOP__CF_LOOP_STEP__SHIFT                 0x00000010

// SQ_VS_PROGRAM
#define SQ_VS_PROGRAM__BASE__SHIFT                         0x00000000
#define SQ_VS_PROGRAM__SIZE__SHIFT                         0x0000000c

// SQ_PS_PROGRAM
#define SQ_PS_PROGRAM__BASE__SHIFT                         0x00000000
#define SQ_PS_PROGRAM__SIZE__SHIFT                         0x0000000c

// SQ_CF_PROGRAM_SIZE
#define SQ_CF_PROGRAM_SIZE__VS_CF_SIZE__SHIFT              0x00000000
#define SQ_CF_PROGRAM_SIZE__PS_CF_SIZE__SHIFT              0x0000000c

// SQ_INTERPOLATOR_CNTL
#define SQ_INTERPOLATOR_CNTL__PARAM_SHADE__SHIFT           0x00000000
#define SQ_INTERPOLATOR_CNTL__SAMPLING_PATTERN__SHIFT      0x00000010

// SQ_PROGRAM_CNTL
#define SQ_PROGRAM_CNTL__VS_NUM_REG__SHIFT                 0x00000000
#define SQ_PROGRAM_CNTL__PS_NUM_REG__SHIFT                 0x00000008
#define SQ_PROGRAM_CNTL__VS_RESOURCE__SHIFT                0x00000010
#define SQ_PROGRAM_CNTL__PS_RESOURCE__SHIFT                0x00000011
#define SQ_PROGRAM_CNTL__PARAM_GEN__SHIFT                  0x00000012
#define SQ_PROGRAM_CNTL__GEN_INDEX_PIX__SHIFT              0x00000013
#define SQ_PROGRAM_CNTL__VS_EXPORT_COUNT__SHIFT            0x00000014
#define SQ_PROGRAM_CNTL__VS_EXPORT_MODE__SHIFT             0x00000018
#define SQ_PROGRAM_CNTL__PS_EXPORT_MODE__SHIFT             0x0000001b
#define SQ_PROGRAM_CNTL__GEN_INDEX_VTX__SHIFT              0x0000001f

// SQ_WRAPPING_0
#define SQ_WRAPPING_0__PARAM_WRAP_0__SHIFT                 0x00000000
#define SQ_WRAPPING_0__PARAM_WRAP_1__SHIFT                 0x00000004
#define SQ_WRAPPING_0__PARAM_WRAP_2__SHIFT                 0x00000008
#define SQ_WRAPPING_0__PARAM_WRAP_3__SHIFT                 0x0000000c
#define SQ_WRAPPING_0__PARAM_WRAP_4__SHIFT                 0x00000010
#define SQ_WRAPPING_0__PARAM_WRAP_5__SHIFT                 0x00000014
#define SQ_WRAPPING_0__PARAM_WRAP_6__SHIFT                 0x00000018
#define SQ_WRAPPING_0__PARAM_WRAP_7__SHIFT                 0x0000001c

// SQ_WRAPPING_1
#define SQ_WRAPPING_1__PARAM_WRAP_8__SHIFT                 0x00000000
#define SQ_WRAPPING_1__PARAM_WRAP_9__SHIFT                 0x00000004
#define SQ_WRAPPING_1__PARAM_WRAP_10__SHIFT                0x00000008
#define SQ_WRAPPING_1__PARAM_WRAP_11__SHIFT                0x0000000c
#define SQ_WRAPPING_1__PARAM_WRAP_12__SHIFT                0x00000010
#define SQ_WRAPPING_1__PARAM_WRAP_13__SHIFT                0x00000014
#define SQ_WRAPPING_1__PARAM_WRAP_14__SHIFT                0x00000018
#define SQ_WRAPPING_1__PARAM_WRAP_15__SHIFT                0x0000001c

// SQ_VS_CONST
#define SQ_VS_CONST__BASE__SHIFT                           0x00000000
#define SQ_VS_CONST__SIZE__SHIFT                           0x0000000c

// SQ_PS_CONST
#define SQ_PS_CONST__BASE__SHIFT                           0x00000000
#define SQ_PS_CONST__SIZE__SHIFT                           0x0000000c

// SQ_CONTEXT_MISC
#define SQ_CONTEXT_MISC__INST_PRED_OPTIMIZE__SHIFT         0x00000000
#define SQ_CONTEXT_MISC__SC_OUTPUT_SCREEN_XY__SHIFT        0x00000001
#define SQ_CONTEXT_MISC__SC_SAMPLE_CNTL__SHIFT             0x00000002
#define SQ_CONTEXT_MISC__PARAM_GEN_POS__SHIFT              0x00000008
#define SQ_CONTEXT_MISC__PERFCOUNTER_REF__SHIFT            0x00000010
#define SQ_CONTEXT_MISC__YEILD_OPTIMIZE__SHIFT             0x00000011
#define SQ_CONTEXT_MISC__TX_CACHE_SEL__SHIFT               0x00000012

// SQ_CF_RD_BASE
#define SQ_CF_RD_BASE__RD_BASE__SHIFT                      0x00000000

// SQ_DEBUG_MISC_0
#define SQ_DEBUG_MISC_0__DB_PROB_ON__SHIFT                 0x00000000
#define SQ_DEBUG_MISC_0__DB_PROB_BREAK__SHIFT              0x00000004
#define SQ_DEBUG_MISC_0__DB_PROB_ADDR__SHIFT               0x00000008
#define SQ_DEBUG_MISC_0__DB_PROB_COUNT__SHIFT              0x00000018

// SQ_DEBUG_MISC_1
#define SQ_DEBUG_MISC_1__DB_ON_PIX__SHIFT                  0x00000000
#define SQ_DEBUG_MISC_1__DB_ON_VTX__SHIFT                  0x00000001
#define SQ_DEBUG_MISC_1__DB_INST_COUNT__SHIFT              0x00000008
#define SQ_DEBUG_MISC_1__DB_BREAK_ADDR__SHIFT              0x00000010

// MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG__SAME_PAGE_LIMIT__SHIFT          0x00000000
#define MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY__SHIFT    0x00000006
#define MH_ARBITER_CONFIG__L1_ARB_ENABLE__SHIFT            0x00000007
#define MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE__SHIFT       0x00000008
#define MH_ARBITER_CONFIG__L2_ARB_CONTROL__SHIFT           0x00000009
#define MH_ARBITER_CONFIG__PAGE_SIZE__SHIFT                0x0000000a
#define MH_ARBITER_CONFIG__TC_REORDER_ENABLE__SHIFT        0x0000000d
#define MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE__SHIFT       0x0000000e
#define MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT   0x0000000f
#define MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT          0x00000010
#define MH_ARBITER_CONFIG__CP_CLNT_ENABLE__SHIFT           0x00000016
#define MH_ARBITER_CONFIG__VGT_CLNT_ENABLE__SHIFT          0x00000017
#define MH_ARBITER_CONFIG__TC_CLNT_ENABLE__SHIFT           0x00000018
#define MH_ARBITER_CONFIG__RB_CLNT_ENABLE__SHIFT           0x00000019
#define MH_ARBITER_CONFIG__PA_CLNT_ENABLE__SHIFT           0x0000001a

// MH_CLNT_AXI_ID_REUSE
#define MH_CLNT_AXI_ID_REUSE__CPw_ID__SHIFT                0x00000000
#define MH_CLNT_AXI_ID_REUSE__RESERVED1__SHIFT             0x00000003
#define MH_CLNT_AXI_ID_REUSE__RBw_ID__SHIFT                0x00000004
#define MH_CLNT_AXI_ID_REUSE__RESERVED2__SHIFT             0x00000007
#define MH_CLNT_AXI_ID_REUSE__MMUr_ID__SHIFT               0x00000008
#define MH_CLNT_AXI_ID_REUSE__RESERVED3__SHIFT             0x0000000b
#define MH_CLNT_AXI_ID_REUSE__PAw_ID__SHIFT                0x0000000c

// MH_INTERRUPT_MASK
#define MH_INTERRUPT_MASK__AXI_READ_ERROR__SHIFT           0x00000000
#define MH_INTERRUPT_MASK__AXI_WRITE_ERROR__SHIFT          0x00000001
#define MH_INTERRUPT_MASK__MMU_PAGE_FAULT__SHIFT           0x00000002

// MH_INTERRUPT_STATUS
#define MH_INTERRUPT_STATUS__AXI_READ_ERROR__SHIFT         0x00000000
#define MH_INTERRUPT_STATUS__AXI_WRITE_ERROR__SHIFT        0x00000001
#define MH_INTERRUPT_STATUS__MMU_PAGE_FAULT__SHIFT         0x00000002

// MH_INTERRUPT_CLEAR
#define MH_INTERRUPT_CLEAR__AXI_READ_ERROR__SHIFT          0x00000000
#define MH_INTERRUPT_CLEAR__AXI_WRITE_ERROR__SHIFT         0x00000001
#define MH_INTERRUPT_CLEAR__MMU_PAGE_FAULT__SHIFT          0x00000002

// MH_AXI_ERROR
#define MH_AXI_ERROR__AXI_READ_ID__SHIFT                   0x00000000
#define MH_AXI_ERROR__AXI_READ_ERROR__SHIFT                0x00000003
#define MH_AXI_ERROR__AXI_WRITE_ID__SHIFT                  0x00000004
#define MH_AXI_ERROR__AXI_WRITE_ERROR__SHIFT               0x00000007

// MH_PERFCOUNTER0_SELECT
#define MH_PERFCOUNTER0_SELECT__PERF_SEL__SHIFT            0x00000000

// MH_PERFCOUNTER1_SELECT
#define MH_PERFCOUNTER1_SELECT__PERF_SEL__SHIFT            0x00000000

// MH_PERFCOUNTER0_CONFIG
#define MH_PERFCOUNTER0_CONFIG__N_VALUE__SHIFT             0x00000000

// MH_PERFCOUNTER1_CONFIG
#define MH_PERFCOUNTER1_CONFIG__N_VALUE__SHIFT             0x00000000

// MH_PERFCOUNTER0_LOW
#define MH_PERFCOUNTER0_LOW__PERF_COUNTER_LOW__SHIFT       0x00000000

// MH_PERFCOUNTER1_LOW
#define MH_PERFCOUNTER1_LOW__PERF_COUNTER_LOW__SHIFT       0x00000000

// MH_PERFCOUNTER0_HI
#define MH_PERFCOUNTER0_HI__PERF_COUNTER_HI__SHIFT         0x00000000

// MH_PERFCOUNTER1_HI
#define MH_PERFCOUNTER1_HI__PERF_COUNTER_HI__SHIFT         0x00000000

// MH_DEBUG_CTRL
#define MH_DEBUG_CTRL__INDEX__SHIFT                        0x00000000

// MH_DEBUG_DATA
#define MH_DEBUG_DATA__DATA__SHIFT                         0x00000000

// MH_AXI_HALT_CONTROL
#define MH_AXI_HALT_CONTROL__AXI_HALT__SHIFT               0x00000000

// MH_DEBUG_REG00
#define MH_DEBUG_REG00__MH_BUSY__SHIFT                     0x00000000
#define MH_DEBUG_REG00__TRANS_OUTSTANDING__SHIFT           0x00000001
#define MH_DEBUG_REG00__CP_REQUEST__SHIFT                  0x00000002
#define MH_DEBUG_REG00__VGT_REQUEST__SHIFT                 0x00000003
#define MH_DEBUG_REG00__TC_REQUEST__SHIFT                  0x00000004
#define MH_DEBUG_REG00__TC_CAM_EMPTY__SHIFT                0x00000005
#define MH_DEBUG_REG00__TC_CAM_FULL__SHIFT                 0x00000006
#define MH_DEBUG_REG00__TCD_EMPTY__SHIFT                   0x00000007
#define MH_DEBUG_REG00__TCD_FULL__SHIFT                    0x00000008
#define MH_DEBUG_REG00__RB_REQUEST__SHIFT                  0x00000009
#define MH_DEBUG_REG00__PA_REQUEST__SHIFT                  0x0000000a
#define MH_DEBUG_REG00__MH_CLK_EN_STATE__SHIFT             0x0000000b
#define MH_DEBUG_REG00__ARQ_EMPTY__SHIFT                   0x0000000c
#define MH_DEBUG_REG00__ARQ_FULL__SHIFT                    0x0000000d
#define MH_DEBUG_REG00__WDB_EMPTY__SHIFT                   0x0000000e
#define MH_DEBUG_REG00__WDB_FULL__SHIFT                    0x0000000f
#define MH_DEBUG_REG00__AXI_AVALID__SHIFT                  0x00000010
#define MH_DEBUG_REG00__AXI_AREADY__SHIFT                  0x00000011
#define MH_DEBUG_REG00__AXI_ARVALID__SHIFT                 0x00000012
#define MH_DEBUG_REG00__AXI_ARREADY__SHIFT                 0x00000013
#define MH_DEBUG_REG00__AXI_WVALID__SHIFT                  0x00000014
#define MH_DEBUG_REG00__AXI_WREADY__SHIFT                  0x00000015
#define MH_DEBUG_REG00__AXI_RVALID__SHIFT                  0x00000016
#define MH_DEBUG_REG00__AXI_RREADY__SHIFT                  0x00000017
#define MH_DEBUG_REG00__AXI_BVALID__SHIFT                  0x00000018
#define MH_DEBUG_REG00__AXI_BREADY__SHIFT                  0x00000019
#define MH_DEBUG_REG00__AXI_HALT_REQ__SHIFT                0x0000001a
#define MH_DEBUG_REG00__AXI_HALT_ACK__SHIFT                0x0000001b
#define MH_DEBUG_REG00__AXI_RDY_ENA__SHIFT                 0x0000001c

// MH_DEBUG_REG01
#define MH_DEBUG_REG01__CP_SEND_q__SHIFT                   0x00000000
#define MH_DEBUG_REG01__CP_RTR_q__SHIFT                    0x00000001
#define MH_DEBUG_REG01__CP_WRITE_q__SHIFT                  0x00000002
#define MH_DEBUG_REG01__CP_TAG_q__SHIFT                    0x00000003
#define MH_DEBUG_REG01__CP_BLEN_q__SHIFT                   0x00000006
#define MH_DEBUG_REG01__VGT_SEND_q__SHIFT                  0x00000007
#define MH_DEBUG_REG01__VGT_RTR_q__SHIFT                   0x00000008
#define MH_DEBUG_REG01__VGT_TAG_q__SHIFT                   0x00000009
#define MH_DEBUG_REG01__TC_SEND_q__SHIFT                   0x0000000a
#define MH_DEBUG_REG01__TC_RTR_q__SHIFT                    0x0000000b
#define MH_DEBUG_REG01__TC_BLEN_q__SHIFT                   0x0000000c
#define MH_DEBUG_REG01__TC_ROQ_SEND_q__SHIFT               0x0000000d
#define MH_DEBUG_REG01__TC_ROQ_RTR_q__SHIFT                0x0000000e
#define MH_DEBUG_REG01__TC_MH_written__SHIFT               0x0000000f
#define MH_DEBUG_REG01__RB_SEND_q__SHIFT                   0x00000010
#define MH_DEBUG_REG01__RB_RTR_q__SHIFT                    0x00000011
#define MH_DEBUG_REG01__PA_SEND_q__SHIFT                   0x00000012
#define MH_DEBUG_REG01__PA_RTR_q__SHIFT                    0x00000013

// MH_DEBUG_REG02
#define MH_DEBUG_REG02__MH_CP_grb_send__SHIFT              0x00000000
#define MH_DEBUG_REG02__MH_VGT_grb_send__SHIFT             0x00000001
#define MH_DEBUG_REG02__MH_TC_mcsend__SHIFT                0x00000002
#define MH_DEBUG_REG02__MH_CLNT_rlast__SHIFT               0x00000003
#define MH_DEBUG_REG02__MH_CLNT_tag__SHIFT                 0x00000004
#define MH_DEBUG_REG02__RDC_RID__SHIFT                     0x00000007
#define MH_DEBUG_REG02__RDC_RRESP__SHIFT                   0x0000000a
#define MH_DEBUG_REG02__MH_CP_writeclean__SHIFT            0x0000000c
#define MH_DEBUG_REG02__MH_RB_writeclean__SHIFT            0x0000000d
#define MH_DEBUG_REG02__MH_PA_writeclean__SHIFT            0x0000000e
#define MH_DEBUG_REG02__BRC_BID__SHIFT                     0x0000000f
#define MH_DEBUG_REG02__BRC_BRESP__SHIFT                   0x00000012

// MH_DEBUG_REG03
#define MH_DEBUG_REG03__MH_CLNT_data_31_0__SHIFT           0x00000000

// MH_DEBUG_REG04
#define MH_DEBUG_REG04__MH_CLNT_data_63_32__SHIFT          0x00000000

// MH_DEBUG_REG05
#define MH_DEBUG_REG05__CP_MH_send__SHIFT                  0x00000000
#define MH_DEBUG_REG05__CP_MH_write__SHIFT                 0x00000001
#define MH_DEBUG_REG05__CP_MH_tag__SHIFT                   0x00000002
#define MH_DEBUG_REG05__CP_MH_ad_31_5__SHIFT               0x00000005

// MH_DEBUG_REG06
#define MH_DEBUG_REG06__CP_MH_data_31_0__SHIFT             0x00000000

// MH_DEBUG_REG07
#define MH_DEBUG_REG07__CP_MH_data_63_32__SHIFT            0x00000000

// MH_DEBUG_REG08
#define MH_DEBUG_REG08__CP_MH_be__SHIFT                    0x00000000
#define MH_DEBUG_REG08__RB_MH_be__SHIFT                    0x00000008
#define MH_DEBUG_REG08__PA_MH_be__SHIFT                    0x00000010

// MH_DEBUG_REG09
#define MH_DEBUG_REG09__ALWAYS_ZERO__SHIFT                 0x00000000
#define MH_DEBUG_REG09__VGT_MH_send__SHIFT                 0x00000003
#define MH_DEBUG_REG09__VGT_MH_tagbe__SHIFT                0x00000004
#define MH_DEBUG_REG09__VGT_MH_ad_31_5__SHIFT              0x00000005

// MH_DEBUG_REG10
#define MH_DEBUG_REG10__ALWAYS_ZERO__SHIFT                 0x00000000
#define MH_DEBUG_REG10__TC_MH_send__SHIFT                  0x00000002
#define MH_DEBUG_REG10__TC_MH_mask__SHIFT                  0x00000003
#define MH_DEBUG_REG10__TC_MH_addr_31_5__SHIFT             0x00000005

// MH_DEBUG_REG11
#define MH_DEBUG_REG11__TC_MH_info__SHIFT                  0x00000000
#define MH_DEBUG_REG11__TC_MH_send__SHIFT                  0x00000019

// MH_DEBUG_REG12
#define MH_DEBUG_REG12__MH_TC_mcinfo__SHIFT                0x00000000
#define MH_DEBUG_REG12__MH_TC_mcinfo_send__SHIFT           0x00000019
#define MH_DEBUG_REG12__TC_MH_written__SHIFT               0x0000001a

// MH_DEBUG_REG13
#define MH_DEBUG_REG13__ALWAYS_ZERO__SHIFT                 0x00000000
#define MH_DEBUG_REG13__TC_ROQ_SEND__SHIFT                 0x00000002
#define MH_DEBUG_REG13__TC_ROQ_MASK__SHIFT                 0x00000003
#define MH_DEBUG_REG13__TC_ROQ_ADDR_31_5__SHIFT            0x00000005

// MH_DEBUG_REG14
#define MH_DEBUG_REG14__TC_ROQ_INFO__SHIFT                 0x00000000
#define MH_DEBUG_REG14__TC_ROQ_SEND__SHIFT                 0x00000019

// MH_DEBUG_REG15
#define MH_DEBUG_REG15__ALWAYS_ZERO__SHIFT                 0x00000000
#define MH_DEBUG_REG15__RB_MH_send__SHIFT                  0x00000004
#define MH_DEBUG_REG15__RB_MH_addr_31_5__SHIFT             0x00000005

// MH_DEBUG_REG16
#define MH_DEBUG_REG16__RB_MH_data_31_0__SHIFT             0x00000000

// MH_DEBUG_REG17
#define MH_DEBUG_REG17__RB_MH_data_63_32__SHIFT            0x00000000

// MH_DEBUG_REG18
#define MH_DEBUG_REG18__ALWAYS_ZERO__SHIFT                 0x00000000
#define MH_DEBUG_REG18__PA_MH_send__SHIFT                  0x00000004
#define MH_DEBUG_REG18__PA_MH_addr_31_5__SHIFT             0x00000005

// MH_DEBUG_REG19
#define MH_DEBUG_REG19__PA_MH_data_31_0__SHIFT             0x00000000

// MH_DEBUG_REG20
#define MH_DEBUG_REG20__PA_MH_data_63_32__SHIFT            0x00000000

// MH_DEBUG_REG21
#define MH_DEBUG_REG21__AVALID_q__SHIFT                    0x00000000
#define MH_DEBUG_REG21__AREADY_q__SHIFT                    0x00000001
#define MH_DEBUG_REG21__AID_q__SHIFT                       0x00000002
#define MH_DEBUG_REG21__ALEN_q_2_0__SHIFT                  0x00000005
#define MH_DEBUG_REG21__ARVALID_q__SHIFT                   0x00000008
#define MH_DEBUG_REG21__ARREADY_q__SHIFT                   0x00000009
#define MH_DEBUG_REG21__ARID_q__SHIFT                      0x0000000a
#define MH_DEBUG_REG21__ARLEN_q_1_0__SHIFT                 0x0000000d
#define MH_DEBUG_REG21__RVALID_q__SHIFT                    0x0000000f
#define MH_DEBUG_REG21__RREADY_q__SHIFT                    0x00000010
#define MH_DEBUG_REG21__RLAST_q__SHIFT                     0x00000011
#define MH_DEBUG_REG21__RID_q__SHIFT                       0x00000012
#define MH_DEBUG_REG21__WVALID_q__SHIFT                    0x00000015
#define MH_DEBUG_REG21__WREADY_q__SHIFT                    0x00000016
#define MH_DEBUG_REG21__WLAST_q__SHIFT                     0x00000017
#define MH_DEBUG_REG21__WID_q__SHIFT                       0x00000018
#define MH_DEBUG_REG21__BVALID_q__SHIFT                    0x0000001b
#define MH_DEBUG_REG21__BREADY_q__SHIFT                    0x0000001c
#define MH_DEBUG_REG21__BID_q__SHIFT                       0x0000001d

// MH_DEBUG_REG22
#define MH_DEBUG_REG22__AVALID_q__SHIFT                    0x00000000
#define MH_DEBUG_REG22__AREADY_q__SHIFT                    0x00000001
#define MH_DEBUG_REG22__AID_q__SHIFT                       0x00000002
#define MH_DEBUG_REG22__ALEN_q_1_0__SHIFT                  0x00000005
#define MH_DEBUG_REG22__ARVALID_q__SHIFT                   0x00000007
#define MH_DEBUG_REG22__ARREADY_q__SHIFT                   0x00000008
#define MH_DEBUG_REG22__ARID_q__SHIFT                      0x00000009
#define MH_DEBUG_REG22__ARLEN_q_1_1__SHIFT                 0x0000000c
#define MH_DEBUG_REG22__WVALID_q__SHIFT                    0x0000000d
#define MH_DEBUG_REG22__WREADY_q__SHIFT                    0x0000000e
#define MH_DEBUG_REG22__WLAST_q__SHIFT                     0x0000000f
#define MH_DEBUG_REG22__WID_q__SHIFT                       0x00000010
#define MH_DEBUG_REG22__WSTRB_q__SHIFT                     0x00000013
#define MH_DEBUG_REG22__BVALID_q__SHIFT                    0x0000001b
#define MH_DEBUG_REG22__BREADY_q__SHIFT                    0x0000001c
#define MH_DEBUG_REG22__BID_q__SHIFT                       0x0000001d

// MH_DEBUG_REG23
#define MH_DEBUG_REG23__ARC_CTRL_RE_q__SHIFT               0x00000000
#define MH_DEBUG_REG23__CTRL_ARC_ID__SHIFT                 0x00000001
#define MH_DEBUG_REG23__CTRL_ARC_PAD__SHIFT                0x00000004

// MH_DEBUG_REG24
#define MH_DEBUG_REG24__ALWAYS_ZERO__SHIFT                 0x00000000
#define MH_DEBUG_REG24__REG_A__SHIFT                       0x00000002
#define MH_DEBUG_REG24__REG_RE__SHIFT                      0x00000010
#define MH_DEBUG_REG24__REG_WE__SHIFT                      0x00000011
#define MH_DEBUG_REG24__BLOCK_RS__SHIFT                    0x00000012

// MH_DEBUG_REG25
#define MH_DEBUG_REG25__REG_WD__SHIFT                      0x00000000

// MH_DEBUG_REG26
#define MH_DEBUG_REG26__MH_RBBM_busy__SHIFT                0x00000000
#define MH_DEBUG_REG26__MH_CIB_mh_clk_en_int__SHIFT        0x00000001
#define MH_DEBUG_REG26__MH_CIB_mmu_clk_en_int__SHIFT       0x00000002
#define MH_DEBUG_REG26__MH_CIB_tcroq_clk_en_int__SHIFT     0x00000003
#define MH_DEBUG_REG26__GAT_CLK_ENA__SHIFT                 0x00000004
#define MH_DEBUG_REG26__RBBM_MH_clk_en_override__SHIFT     0x00000005
#define MH_DEBUG_REG26__CNT_q__SHIFT                       0x00000006
#define MH_DEBUG_REG26__TCD_EMPTY_q__SHIFT                 0x0000000c
#define MH_DEBUG_REG26__TC_ROQ_EMPTY__SHIFT                0x0000000d
#define MH_DEBUG_REG26__MH_BUSY_d__SHIFT                   0x0000000e
#define MH_DEBUG_REG26__ANY_CLNT_BUSY__SHIFT               0x0000000f
#define MH_DEBUG_REG26__MH_MMU_INVALIDATE_INVALIDATE_ALL__SHIFT 0x00000010
#define MH_DEBUG_REG26__MH_MMU_INVALIDATE_INVALIDATE_TC__SHIFT 0x00000011
#define MH_DEBUG_REG26__CP_SEND_q__SHIFT                   0x00000012
#define MH_DEBUG_REG26__CP_RTR_q__SHIFT                    0x00000013
#define MH_DEBUG_REG26__VGT_SEND_q__SHIFT                  0x00000014
#define MH_DEBUG_REG26__VGT_RTR_q__SHIFT                   0x00000015
#define MH_DEBUG_REG26__TC_ROQ_SEND_q__SHIFT               0x00000016
#define MH_DEBUG_REG26__TC_ROQ_RTR_DBG_q__SHIFT            0x00000017
#define MH_DEBUG_REG26__RB_SEND_q__SHIFT                   0x00000018
#define MH_DEBUG_REG26__RB_RTR_q__SHIFT                    0x00000019
#define MH_DEBUG_REG26__PA_SEND_q__SHIFT                   0x0000001a
#define MH_DEBUG_REG26__PA_RTR_q__SHIFT                    0x0000001b
#define MH_DEBUG_REG26__RDC_VALID__SHIFT                   0x0000001c
#define MH_DEBUG_REG26__RDC_RLAST__SHIFT                   0x0000001d
#define MH_DEBUG_REG26__TLBMISS_VALID__SHIFT               0x0000001e
#define MH_DEBUG_REG26__BRC_VALID__SHIFT                   0x0000001f

// MH_DEBUG_REG27
#define MH_DEBUG_REG27__EFF2_FP_WINNER__SHIFT              0x00000000
#define MH_DEBUG_REG27__EFF2_LRU_WINNER_out__SHIFT         0x00000003
#define MH_DEBUG_REG27__EFF1_WINNER__SHIFT                 0x00000006
#define MH_DEBUG_REG27__ARB_WINNER__SHIFT                  0x00000009
#define MH_DEBUG_REG27__ARB_WINNER_q__SHIFT                0x0000000c
#define MH_DEBUG_REG27__EFF1_WIN__SHIFT                    0x0000000f
#define MH_DEBUG_REG27__KILL_EFF1__SHIFT                   0x00000010
#define MH_DEBUG_REG27__ARB_HOLD__SHIFT                    0x00000011
#define MH_DEBUG_REG27__ARB_RTR_q__SHIFT                   0x00000012
#define MH_DEBUG_REG27__CP_SEND_QUAL__SHIFT                0x00000013
#define MH_DEBUG_REG27__VGT_SEND_QUAL__SHIFT               0x00000014
#define MH_DEBUG_REG27__TC_SEND_QUAL__SHIFT                0x00000015
#define MH_DEBUG_REG27__TC_SEND_EFF1_QUAL__SHIFT           0x00000016
#define MH_DEBUG_REG27__RB_SEND_QUAL__SHIFT                0x00000017
#define MH_DEBUG_REG27__PA_SEND_QUAL__SHIFT                0x00000018
#define MH_DEBUG_REG27__ARB_QUAL__SHIFT                    0x00000019
#define MH_DEBUG_REG27__CP_EFF1_REQ__SHIFT                 0x0000001a
#define MH_DEBUG_REG27__VGT_EFF1_REQ__SHIFT                0x0000001b
#define MH_DEBUG_REG27__TC_EFF1_REQ__SHIFT                 0x0000001c
#define MH_DEBUG_REG27__RB_EFF1_REQ__SHIFT                 0x0000001d
#define MH_DEBUG_REG27__TCD_NEARFULL_q__SHIFT              0x0000001e
#define MH_DEBUG_REG27__TCHOLD_IP_q__SHIFT                 0x0000001f

// MH_DEBUG_REG28
#define MH_DEBUG_REG28__EFF1_WINNER__SHIFT                 0x00000000
#define MH_DEBUG_REG28__ARB_WINNER__SHIFT                  0x00000003
#define MH_DEBUG_REG28__CP_SEND_QUAL__SHIFT                0x00000006
#define MH_DEBUG_REG28__VGT_SEND_QUAL__SHIFT               0x00000007
#define MH_DEBUG_REG28__TC_SEND_QUAL__SHIFT                0x00000008
#define MH_DEBUG_REG28__TC_SEND_EFF1_QUAL__SHIFT           0x00000009
#define MH_DEBUG_REG28__RB_SEND_QUAL__SHIFT                0x0000000a
#define MH_DEBUG_REG28__ARB_QUAL__SHIFT                    0x0000000b
#define MH_DEBUG_REG28__CP_EFF1_REQ__SHIFT                 0x0000000c
#define MH_DEBUG_REG28__VGT_EFF1_REQ__SHIFT                0x0000000d
#define MH_DEBUG_REG28__TC_EFF1_REQ__SHIFT                 0x0000000e
#define MH_DEBUG_REG28__RB_EFF1_REQ__SHIFT                 0x0000000f
#define MH_DEBUG_REG28__EFF1_WIN__SHIFT                    0x00000010
#define MH_DEBUG_REG28__KILL_EFF1__SHIFT                   0x00000011
#define MH_DEBUG_REG28__TCD_NEARFULL_q__SHIFT              0x00000012
#define MH_DEBUG_REG28__TC_ARB_HOLD__SHIFT                 0x00000013
#define MH_DEBUG_REG28__ARB_HOLD__SHIFT                    0x00000014
#define MH_DEBUG_REG28__ARB_RTR_q__SHIFT                   0x00000015
#define MH_DEBUG_REG28__SAME_PAGE_LIMIT_COUNT_q__SHIFT     0x00000016

// MH_DEBUG_REG29
#define MH_DEBUG_REG29__EFF2_LRU_WINNER_out__SHIFT         0x00000000
#define MH_DEBUG_REG29__LEAST_RECENT_INDEX_d__SHIFT        0x00000003
#define MH_DEBUG_REG29__LEAST_RECENT_d__SHIFT              0x00000006
#define MH_DEBUG_REG29__UPDATE_RECENT_STACK_d__SHIFT       0x00000009
#define MH_DEBUG_REG29__ARB_HOLD__SHIFT                    0x0000000a
#define MH_DEBUG_REG29__ARB_RTR_q__SHIFT                   0x0000000b
#define MH_DEBUG_REG29__CLNT_REQ__SHIFT                    0x0000000c
#define MH_DEBUG_REG29__RECENT_d_0__SHIFT                  0x00000011
#define MH_DEBUG_REG29__RECENT_d_1__SHIFT                  0x00000014
#define MH_DEBUG_REG29__RECENT_d_2__SHIFT                  0x00000017
#define MH_DEBUG_REG29__RECENT_d_3__SHIFT                  0x0000001a
#define MH_DEBUG_REG29__RECENT_d_4__SHIFT                  0x0000001d

// MH_DEBUG_REG30
#define MH_DEBUG_REG30__TC_ARB_HOLD__SHIFT                 0x00000000
#define MH_DEBUG_REG30__TC_NOROQ_SAME_ROW_BANK__SHIFT      0x00000001
#define MH_DEBUG_REG30__TC_ROQ_SAME_ROW_BANK__SHIFT        0x00000002
#define MH_DEBUG_REG30__TCD_NEARFULL_q__SHIFT              0x00000003
#define MH_DEBUG_REG30__TCHOLD_IP_q__SHIFT                 0x00000004
#define MH_DEBUG_REG30__TCHOLD_CNT_q__SHIFT                0x00000005
#define MH_DEBUG_REG30__MH_ARBITER_CONFIG_TC_REORDER_ENABLE__SHIFT 0x00000008
#define MH_DEBUG_REG30__TC_ROQ_RTR_DBG_q__SHIFT            0x00000009
#define MH_DEBUG_REG30__TC_ROQ_SEND_q__SHIFT               0x0000000a
#define MH_DEBUG_REG30__TC_MH_written__SHIFT               0x0000000b
#define MH_DEBUG_REG30__TCD_FULLNESS_CNT_q__SHIFT          0x0000000c
#define MH_DEBUG_REG30__WBURST_ACTIVE__SHIFT               0x00000013
#define MH_DEBUG_REG30__WLAST_q__SHIFT                     0x00000014
#define MH_DEBUG_REG30__WBURST_IP_q__SHIFT                 0x00000015
#define MH_DEBUG_REG30__WBURST_CNT_q__SHIFT                0x00000016
#define MH_DEBUG_REG30__CP_SEND_QUAL__SHIFT                0x00000019
#define MH_DEBUG_REG30__CP_MH_write__SHIFT                 0x0000001a
#define MH_DEBUG_REG30__RB_SEND_QUAL__SHIFT                0x0000001b
#define MH_DEBUG_REG30__PA_SEND_QUAL__SHIFT                0x0000001c
#define MH_DEBUG_REG30__ARB_WINNER__SHIFT                  0x0000001d

// MH_DEBUG_REG31
#define MH_DEBUG_REG31__RF_ARBITER_CONFIG_q__SHIFT         0x00000000
#define MH_DEBUG_REG31__MH_CLNT_AXI_ID_REUSE_MMUr_ID__SHIFT 0x0000001a

// MH_DEBUG_REG32
#define MH_DEBUG_REG32__SAME_ROW_BANK_q__SHIFT             0x00000000
#define MH_DEBUG_REG32__ROQ_MARK_q__SHIFT                  0x00000008
#define MH_DEBUG_REG32__ROQ_VALID_q__SHIFT                 0x00000010
#define MH_DEBUG_REG32__TC_MH_send__SHIFT                  0x00000018
#define MH_DEBUG_REG32__TC_ROQ_RTR_q__SHIFT                0x00000019
#define MH_DEBUG_REG32__KILL_EFF1__SHIFT                   0x0000001a
#define MH_DEBUG_REG32__TC_ROQ_SAME_ROW_BANK_SEL__SHIFT    0x0000001b
#define MH_DEBUG_REG32__ANY_SAME_ROW_BANK__SHIFT           0x0000001c
#define MH_DEBUG_REG32__TC_EFF1_QUAL__SHIFT                0x0000001d
#define MH_DEBUG_REG32__TC_ROQ_EMPTY__SHIFT                0x0000001e
#define MH_DEBUG_REG32__TC_ROQ_FULL__SHIFT                 0x0000001f

// MH_DEBUG_REG33
#define MH_DEBUG_REG33__SAME_ROW_BANK_q__SHIFT             0x00000000
#define MH_DEBUG_REG33__ROQ_MARK_d__SHIFT                  0x00000008
#define MH_DEBUG_REG33__ROQ_VALID_d__SHIFT                 0x00000010
#define MH_DEBUG_REG33__TC_MH_send__SHIFT                  0x00000018
#define MH_DEBUG_REG33__TC_ROQ_RTR_q__SHIFT                0x00000019
#define MH_DEBUG_REG33__KILL_EFF1__SHIFT                   0x0000001a
#define MH_DEBUG_REG33__TC_ROQ_SAME_ROW_BANK_SEL__SHIFT    0x0000001b
#define MH_DEBUG_REG33__ANY_SAME_ROW_BANK__SHIFT           0x0000001c
#define MH_DEBUG_REG33__TC_EFF1_QUAL__SHIFT                0x0000001d
#define MH_DEBUG_REG33__TC_ROQ_EMPTY__SHIFT                0x0000001e
#define MH_DEBUG_REG33__TC_ROQ_FULL__SHIFT                 0x0000001f

// MH_DEBUG_REG34
#define MH_DEBUG_REG34__SAME_ROW_BANK_WIN__SHIFT           0x00000000
#define MH_DEBUG_REG34__SAME_ROW_BANK_REQ__SHIFT           0x00000008
#define MH_DEBUG_REG34__NON_SAME_ROW_BANK_WIN__SHIFT       0x00000010
#define MH_DEBUG_REG34__NON_SAME_ROW_BANK_REQ__SHIFT       0x00000018

// MH_DEBUG_REG35
#define MH_DEBUG_REG35__TC_MH_send__SHIFT                  0x00000000
#define MH_DEBUG_REG35__TC_ROQ_RTR_q__SHIFT                0x00000001
#define MH_DEBUG_REG35__ROQ_MARK_q_0__SHIFT                0x00000002
#define MH_DEBUG_REG35__ROQ_VALID_q_0__SHIFT               0x00000003
#define MH_DEBUG_REG35__SAME_ROW_BANK_q_0__SHIFT           0x00000004
#define MH_DEBUG_REG35__ROQ_ADDR_0__SHIFT                  0x00000005

// MH_DEBUG_REG36
#define MH_DEBUG_REG36__TC_MH_send__SHIFT                  0x00000000
#define MH_DEBUG_REG36__TC_ROQ_RTR_q__SHIFT                0x00000001
#define MH_DEBUG_REG36__ROQ_MARK_q_1__SHIFT                0x00000002
#define MH_DEBUG_REG36__ROQ_VALID_q_1__SHIFT               0x00000003
#define MH_DEBUG_REG36__SAME_ROW_BANK_q_1__SHIFT           0x00000004
#define MH_DEBUG_REG36__ROQ_ADDR_1__SHIFT                  0x00000005

// MH_DEBUG_REG37
#define MH_DEBUG_REG37__TC_MH_send__SHIFT                  0x00000000
#define MH_DEBUG_REG37__TC_ROQ_RTR_q__SHIFT                0x00000001
#define MH_DEBUG_REG37__ROQ_MARK_q_2__SHIFT                0x00000002
#define MH_DEBUG_REG37__ROQ_VALID_q_2__SHIFT               0x00000003
#define MH_DEBUG_REG37__SAME_ROW_BANK_q_2__SHIFT           0x00000004
#define MH_DEBUG_REG37__ROQ_ADDR_2__SHIFT                  0x00000005

// MH_DEBUG_REG38
#define MH_DEBUG_REG38__TC_MH_send__SHIFT                  0x00000000
#define MH_DEBUG_REG38__TC_ROQ_RTR_q__SHIFT                0x00000001
#define MH_DEBUG_REG38__ROQ_MARK_q_3__SHIFT                0x00000002
#define MH_DEBUG_REG38__ROQ_VALID_q_3__SHIFT               0x00000003
#define MH_DEBUG_REG38__SAME_ROW_BANK_q_3__SHIFT           0x00000004
#define MH_DEBUG_REG38__ROQ_ADDR_3__SHIFT                  0x00000005

// MH_DEBUG_REG39
#define MH_DEBUG_REG39__TC_MH_send__SHIFT                  0x00000000
#define MH_DEBUG_REG39__TC_ROQ_RTR_q__SHIFT                0x00000001
#define MH_DEBUG_REG39__ROQ_MARK_q_4__SHIFT                0x00000002
#define MH_DEBUG_REG39__ROQ_VALID_q_4__SHIFT               0x00000003
#define MH_DEBUG_REG39__SAME_ROW_BANK_q_4__SHIFT           0x00000004
#define MH_DEBUG_REG39__ROQ_ADDR_4__SHIFT                  0x00000005

// MH_DEBUG_REG40
#define MH_DEBUG_REG40__TC_MH_send__SHIFT                  0x00000000
#define MH_DEBUG_REG40__TC_ROQ_RTR_q__SHIFT                0x00000001
#define MH_DEBUG_REG40__ROQ_MARK_q_5__SHIFT                0x00000002
#define MH_DEBUG_REG40__ROQ_VALID_q_5__SHIFT               0x00000003
#define MH_DEBUG_REG40__SAME_ROW_BANK_q_5__SHIFT           0x00000004
#define MH_DEBUG_REG40__ROQ_ADDR_5__SHIFT                  0x00000005

// MH_DEBUG_REG41
#define MH_DEBUG_REG41__TC_MH_send__SHIFT                  0x00000000
#define MH_DEBUG_REG41__TC_ROQ_RTR_q__SHIFT                0x00000001
#define MH_DEBUG_REG41__ROQ_MARK_q_6__SHIFT                0x00000002
#define MH_DEBUG_REG41__ROQ_VALID_q_6__SHIFT               0x00000003
#define MH_DEBUG_REG41__SAME_ROW_BANK_q_6__SHIFT           0x00000004
#define MH_DEBUG_REG41__ROQ_ADDR_6__SHIFT                  0x00000005

// MH_DEBUG_REG42
#define MH_DEBUG_REG42__TC_MH_send__SHIFT                  0x00000000
#define MH_DEBUG_REG42__TC_ROQ_RTR_q__SHIFT                0x00000001
#define MH_DEBUG_REG42__ROQ_MARK_q_7__SHIFT                0x00000002
#define MH_DEBUG_REG42__ROQ_VALID_q_7__SHIFT               0x00000003
#define MH_DEBUG_REG42__SAME_ROW_BANK_q_7__SHIFT           0x00000004
#define MH_DEBUG_REG42__ROQ_ADDR_7__SHIFT                  0x00000005

// MH_DEBUG_REG43
#define MH_DEBUG_REG43__ARB_REG_WE_q__SHIFT                0x00000000
#define MH_DEBUG_REG43__ARB_WE__SHIFT                      0x00000001
#define MH_DEBUG_REG43__ARB_REG_VALID_q__SHIFT             0x00000002
#define MH_DEBUG_REG43__ARB_RTR_q__SHIFT                   0x00000003
#define MH_DEBUG_REG43__ARB_REG_RTR__SHIFT                 0x00000004
#define MH_DEBUG_REG43__WDAT_BURST_RTR__SHIFT              0x00000005
#define MH_DEBUG_REG43__MMU_RTR__SHIFT                     0x00000006
#define MH_DEBUG_REG43__ARB_ID_q__SHIFT                    0x00000007
#define MH_DEBUG_REG43__ARB_WRITE_q__SHIFT                 0x0000000a
#define MH_DEBUG_REG43__ARB_BLEN_q__SHIFT                  0x0000000b
#define MH_DEBUG_REG43__ARQ_CTRL_EMPTY__SHIFT              0x0000000c
#define MH_DEBUG_REG43__ARQ_FIFO_CNT_q__SHIFT              0x0000000d
#define MH_DEBUG_REG43__MMU_WE__SHIFT                      0x00000010
#define MH_DEBUG_REG43__ARQ_RTR__SHIFT                     0x00000011
#define MH_DEBUG_REG43__MMU_ID__SHIFT                      0x00000012
#define MH_DEBUG_REG43__MMU_WRITE__SHIFT                   0x00000015
#define MH_DEBUG_REG43__MMU_BLEN__SHIFT                    0x00000016
#define MH_DEBUG_REG43__WBURST_IP_q__SHIFT                 0x00000017
#define MH_DEBUG_REG43__WDAT_REG_WE_q__SHIFT               0x00000018
#define MH_DEBUG_REG43__WDB_WE__SHIFT                      0x00000019
#define MH_DEBUG_REG43__WDB_RTR_SKID_4__SHIFT              0x0000001a
#define MH_DEBUG_REG43__WDB_RTR_SKID_3__SHIFT              0x0000001b

// MH_DEBUG_REG44
#define MH_DEBUG_REG44__ARB_WE__SHIFT                      0x00000000
#define MH_DEBUG_REG44__ARB_ID_q__SHIFT                    0x00000001
#define MH_DEBUG_REG44__ARB_VAD_q__SHIFT                   0x00000004

// MH_DEBUG_REG45
#define MH_DEBUG_REG45__MMU_WE__SHIFT                      0x00000000
#define MH_DEBUG_REG45__MMU_ID__SHIFT                      0x00000001
#define MH_DEBUG_REG45__MMU_PAD__SHIFT                     0x00000004

// MH_DEBUG_REG46
#define MH_DEBUG_REG46__WDAT_REG_WE_q__SHIFT               0x00000000
#define MH_DEBUG_REG46__WDB_WE__SHIFT                      0x00000001
#define MH_DEBUG_REG46__WDAT_REG_VALID_q__SHIFT            0x00000002
#define MH_DEBUG_REG46__WDB_RTR_SKID_4__SHIFT              0x00000003
#define MH_DEBUG_REG46__ARB_WSTRB_q__SHIFT                 0x00000004
#define MH_DEBUG_REG46__ARB_WLAST__SHIFT                   0x0000000c
#define MH_DEBUG_REG46__WDB_CTRL_EMPTY__SHIFT              0x0000000d
#define MH_DEBUG_REG46__WDB_FIFO_CNT_q__SHIFT              0x0000000e
#define MH_DEBUG_REG46__WDC_WDB_RE_q__SHIFT                0x00000013
#define MH_DEBUG_REG46__WDB_WDC_WID__SHIFT                 0x00000014
#define MH_DEBUG_REG46__WDB_WDC_WLAST__SHIFT               0x00000017
#define MH_DEBUG_REG46__WDB_WDC_WSTRB__SHIFT               0x00000018

// MH_DEBUG_REG47
#define MH_DEBUG_REG47__WDB_WDC_WDATA_31_0__SHIFT          0x00000000

// MH_DEBUG_REG48
#define MH_DEBUG_REG48__WDB_WDC_WDATA_63_32__SHIFT         0x00000000

// MH_DEBUG_REG49
#define MH_DEBUG_REG49__CTRL_ARC_EMPTY__SHIFT              0x00000000
#define MH_DEBUG_REG49__CTRL_RARC_EMPTY__SHIFT             0x00000001
#define MH_DEBUG_REG49__ARQ_CTRL_EMPTY__SHIFT              0x00000002
#define MH_DEBUG_REG49__ARQ_CTRL_WRITE__SHIFT              0x00000003
#define MH_DEBUG_REG49__TLBMISS_CTRL_RTS__SHIFT            0x00000004
#define MH_DEBUG_REG49__CTRL_TLBMISS_RE_q__SHIFT           0x00000005
#define MH_DEBUG_REG49__INFLT_LIMIT_q__SHIFT               0x00000006
#define MH_DEBUG_REG49__INFLT_LIMIT_CNT_q__SHIFT           0x00000007
#define MH_DEBUG_REG49__ARC_CTRL_RE_q__SHIFT               0x0000000d
#define MH_DEBUG_REG49__RARC_CTRL_RE_q__SHIFT              0x0000000e
#define MH_DEBUG_REG49__RVALID_q__SHIFT                    0x0000000f
#define MH_DEBUG_REG49__RREADY_q__SHIFT                    0x00000010
#define MH_DEBUG_REG49__RLAST_q__SHIFT                     0x00000011
#define MH_DEBUG_REG49__BVALID_q__SHIFT                    0x00000012
#define MH_DEBUG_REG49__BREADY_q__SHIFT                    0x00000013

// MH_DEBUG_REG50
#define MH_DEBUG_REG50__MH_CP_grb_send__SHIFT              0x00000000
#define MH_DEBUG_REG50__MH_VGT_grb_send__SHIFT             0x00000001
#define MH_DEBUG_REG50__MH_TC_mcsend__SHIFT                0x00000002
#define MH_DEBUG_REG50__MH_TLBMISS_SEND__SHIFT             0x00000003
#define MH_DEBUG_REG50__TLBMISS_VALID__SHIFT               0x00000004
#define MH_DEBUG_REG50__RDC_VALID__SHIFT                   0x00000005
#define MH_DEBUG_REG50__RDC_RID__SHIFT                     0x00000006
#define MH_DEBUG_REG50__RDC_RLAST__SHIFT                   0x00000009
#define MH_DEBUG_REG50__RDC_RRESP__SHIFT                   0x0000000a
#define MH_DEBUG_REG50__TLBMISS_CTRL_RTS__SHIFT            0x0000000c
#define MH_DEBUG_REG50__CTRL_TLBMISS_RE_q__SHIFT           0x0000000d
#define MH_DEBUG_REG50__MMU_ID_REQUEST_q__SHIFT            0x0000000e
#define MH_DEBUG_REG50__OUTSTANDING_MMUID_CNT_q__SHIFT     0x0000000f
#define MH_DEBUG_REG50__MMU_ID_RESPONSE__SHIFT             0x00000015
#define MH_DEBUG_REG50__TLBMISS_RETURN_CNT_q__SHIFT        0x00000016
#define MH_DEBUG_REG50__CNT_HOLD_q1__SHIFT                 0x0000001c
#define MH_DEBUG_REG50__MH_CLNT_AXI_ID_REUSE_MMUr_ID__SHIFT 0x0000001d

// MH_DEBUG_REG51
#define MH_DEBUG_REG51__RF_MMU_PAGE_FAULT__SHIFT           0x00000000

// MH_DEBUG_REG52
#define MH_DEBUG_REG52__RF_MMU_CONFIG_q_1_to_0__SHIFT      0x00000000
#define MH_DEBUG_REG52__ARB_WE__SHIFT                      0x00000002
#define MH_DEBUG_REG52__MMU_RTR__SHIFT                     0x00000003
#define MH_DEBUG_REG52__RF_MMU_CONFIG_q_25_to_4__SHIFT     0x00000004
#define MH_DEBUG_REG52__ARB_ID_q__SHIFT                    0x0000001a
#define MH_DEBUG_REG52__ARB_WRITE_q__SHIFT                 0x0000001d
#define MH_DEBUG_REG52__client_behavior_q__SHIFT           0x0000001e

// MH_DEBUG_REG53
#define MH_DEBUG_REG53__stage1_valid__SHIFT                0x00000000
#define MH_DEBUG_REG53__IGNORE_TAG_MISS_q__SHIFT           0x00000001
#define MH_DEBUG_REG53__pa_in_mpu_range__SHIFT             0x00000002
#define MH_DEBUG_REG53__tag_match_q__SHIFT                 0x00000003
#define MH_DEBUG_REG53__tag_miss_q__SHIFT                  0x00000004
#define MH_DEBUG_REG53__va_in_range_q__SHIFT               0x00000005
#define MH_DEBUG_REG53__MMU_MISS__SHIFT                    0x00000006
#define MH_DEBUG_REG53__MMU_READ_MISS__SHIFT               0x00000007
#define MH_DEBUG_REG53__MMU_WRITE_MISS__SHIFT              0x00000008
#define MH_DEBUG_REG53__MMU_HIT__SHIFT                     0x00000009
#define MH_DEBUG_REG53__MMU_READ_HIT__SHIFT                0x0000000a
#define MH_DEBUG_REG53__MMU_WRITE_HIT__SHIFT               0x0000000b
#define MH_DEBUG_REG53__MMU_SPLIT_MODE_TC_MISS__SHIFT      0x0000000c
#define MH_DEBUG_REG53__MMU_SPLIT_MODE_TC_HIT__SHIFT       0x0000000d
#define MH_DEBUG_REG53__MMU_SPLIT_MODE_nonTC_MISS__SHIFT   0x0000000e
#define MH_DEBUG_REG53__MMU_SPLIT_MODE_nonTC_HIT__SHIFT    0x0000000f
#define MH_DEBUG_REG53__REQ_VA_OFFSET_q__SHIFT             0x00000010

// MH_DEBUG_REG54
#define MH_DEBUG_REG54__ARQ_RTR__SHIFT                     0x00000000
#define MH_DEBUG_REG54__MMU_WE__SHIFT                      0x00000001
#define MH_DEBUG_REG54__CTRL_TLBMISS_RE_q__SHIFT           0x00000002
#define MH_DEBUG_REG54__TLBMISS_CTRL_RTS__SHIFT            0x00000003
#define MH_DEBUG_REG54__MH_TLBMISS_SEND__SHIFT             0x00000004
#define MH_DEBUG_REG54__MMU_STALL_AWAITING_TLB_MISS_FETCH__SHIFT 0x00000005
#define MH_DEBUG_REG54__pa_in_mpu_range__SHIFT             0x00000006
#define MH_DEBUG_REG54__stage1_valid__SHIFT                0x00000007
#define MH_DEBUG_REG54__stage2_valid__SHIFT                0x00000008
#define MH_DEBUG_REG54__client_behavior_q__SHIFT           0x00000009
#define MH_DEBUG_REG54__IGNORE_TAG_MISS_q__SHIFT           0x0000000b
#define MH_DEBUG_REG54__tag_match_q__SHIFT                 0x0000000c
#define MH_DEBUG_REG54__tag_miss_q__SHIFT                  0x0000000d
#define MH_DEBUG_REG54__va_in_range_q__SHIFT               0x0000000e
#define MH_DEBUG_REG54__PTE_FETCH_COMPLETE_q__SHIFT        0x0000000f
#define MH_DEBUG_REG54__TAG_valid_q__SHIFT                 0x00000010

// MH_DEBUG_REG55
#define MH_DEBUG_REG55__TAG0_VA__SHIFT                     0x00000000
#define MH_DEBUG_REG55__TAG_valid_q_0__SHIFT               0x0000000d
#define MH_DEBUG_REG55__ALWAYS_ZERO__SHIFT                 0x0000000e
#define MH_DEBUG_REG55__TAG1_VA__SHIFT                     0x00000010
#define MH_DEBUG_REG55__TAG_valid_q_1__SHIFT               0x0000001d

// MH_DEBUG_REG56
#define MH_DEBUG_REG56__TAG2_VA__SHIFT                     0x00000000
#define MH_DEBUG_REG56__TAG_valid_q_2__SHIFT               0x0000000d
#define MH_DEBUG_REG56__ALWAYS_ZERO__SHIFT                 0x0000000e
#define MH_DEBUG_REG56__TAG3_VA__SHIFT                     0x00000010
#define MH_DEBUG_REG56__TAG_valid_q_3__SHIFT               0x0000001d

// MH_DEBUG_REG57
#define MH_DEBUG_REG57__TAG4_VA__SHIFT                     0x00000000
#define MH_DEBUG_REG57__TAG_valid_q_4__SHIFT               0x0000000d
#define MH_DEBUG_REG57__ALWAYS_ZERO__SHIFT                 0x0000000e
#define MH_DEBUG_REG57__TAG5_VA__SHIFT                     0x00000010
#define MH_DEBUG_REG57__TAG_valid_q_5__SHIFT               0x0000001d

// MH_DEBUG_REG58
#define MH_DEBUG_REG58__TAG6_VA__SHIFT                     0x00000000
#define MH_DEBUG_REG58__TAG_valid_q_6__SHIFT               0x0000000d
#define MH_DEBUG_REG58__ALWAYS_ZERO__SHIFT                 0x0000000e
#define MH_DEBUG_REG58__TAG7_VA__SHIFT                     0x00000010
#define MH_DEBUG_REG58__TAG_valid_q_7__SHIFT               0x0000001d

// MH_DEBUG_REG59
#define MH_DEBUG_REG59__TAG8_VA__SHIFT                     0x00000000
#define MH_DEBUG_REG59__TAG_valid_q_8__SHIFT               0x0000000d
#define MH_DEBUG_REG59__ALWAYS_ZERO__SHIFT                 0x0000000e
#define MH_DEBUG_REG59__TAG9_VA__SHIFT                     0x00000010
#define MH_DEBUG_REG59__TAG_valid_q_9__SHIFT               0x0000001d

// MH_DEBUG_REG60
#define MH_DEBUG_REG60__TAG10_VA__SHIFT                    0x00000000
#define MH_DEBUG_REG60__TAG_valid_q_10__SHIFT              0x0000000d
#define MH_DEBUG_REG60__ALWAYS_ZERO__SHIFT                 0x0000000e
#define MH_DEBUG_REG60__TAG11_VA__SHIFT                    0x00000010
#define MH_DEBUG_REG60__TAG_valid_q_11__SHIFT              0x0000001d

// MH_DEBUG_REG61
#define MH_DEBUG_REG61__TAG12_VA__SHIFT                    0x00000000
#define MH_DEBUG_REG61__TAG_valid_q_12__SHIFT              0x0000000d
#define MH_DEBUG_REG61__ALWAYS_ZERO__SHIFT                 0x0000000e
#define MH_DEBUG_REG61__TAG13_VA__SHIFT                    0x00000010
#define MH_DEBUG_REG61__TAG_valid_q_13__SHIFT              0x0000001d

// MH_DEBUG_REG62
#define MH_DEBUG_REG62__TAG14_VA__SHIFT                    0x00000000
#define MH_DEBUG_REG62__TAG_valid_q_14__SHIFT              0x0000000d
#define MH_DEBUG_REG62__ALWAYS_ZERO__SHIFT                 0x0000000e
#define MH_DEBUG_REG62__TAG15_VA__SHIFT                    0x00000010
#define MH_DEBUG_REG62__TAG_valid_q_15__SHIFT              0x0000001d

// MH_DEBUG_REG63
#define MH_DEBUG_REG63__MH_DBG_DEFAULT__SHIFT              0x00000000

// MH_MMU_CONFIG
#define MH_MMU_CONFIG__MMU_ENABLE__SHIFT                   0x00000000
#define MH_MMU_CONFIG__SPLIT_MODE_ENABLE__SHIFT            0x00000001
#define MH_MMU_CONFIG__RESERVED1__SHIFT                    0x00000002
#define MH_MMU_CONFIG__RB_W_CLNT_BEHAVIOR__SHIFT           0x00000004
#define MH_MMU_CONFIG__CP_W_CLNT_BEHAVIOR__SHIFT           0x00000006
#define MH_MMU_CONFIG__CP_R0_CLNT_BEHAVIOR__SHIFT          0x00000008
#define MH_MMU_CONFIG__CP_R1_CLNT_BEHAVIOR__SHIFT          0x0000000a
#define MH_MMU_CONFIG__CP_R2_CLNT_BEHAVIOR__SHIFT          0x0000000c
#define MH_MMU_CONFIG__CP_R3_CLNT_BEHAVIOR__SHIFT          0x0000000e
#define MH_MMU_CONFIG__CP_R4_CLNT_BEHAVIOR__SHIFT          0x00000010
#define MH_MMU_CONFIG__VGT_R0_CLNT_BEHAVIOR__SHIFT         0x00000012
#define MH_MMU_CONFIG__VGT_R1_CLNT_BEHAVIOR__SHIFT         0x00000014
#define MH_MMU_CONFIG__TC_R_CLNT_BEHAVIOR__SHIFT           0x00000016
#define MH_MMU_CONFIG__PA_W_CLNT_BEHAVIOR__SHIFT           0x00000018

// MH_MMU_VA_RANGE
#define MH_MMU_VA_RANGE__NUM_64KB_REGIONS__SHIFT           0x00000000
#define MH_MMU_VA_RANGE__VA_BASE__SHIFT                    0x0000000c

// MH_MMU_PT_BASE
#define MH_MMU_PT_BASE__PT_BASE__SHIFT                     0x0000000c

// MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT__PAGE_FAULT__SHIFT               0x00000000
#define MH_MMU_PAGE_FAULT__OP_TYPE__SHIFT                  0x00000001
#define MH_MMU_PAGE_FAULT__CLNT_BEHAVIOR__SHIFT            0x00000002
#define MH_MMU_PAGE_FAULT__AXI_ID__SHIFT                   0x00000004
#define MH_MMU_PAGE_FAULT__RESERVED1__SHIFT                0x00000007
#define MH_MMU_PAGE_FAULT__MPU_ADDRESS_OUT_OF_RANGE__SHIFT 0x00000008
#define MH_MMU_PAGE_FAULT__ADDRESS_OUT_OF_RANGE__SHIFT     0x00000009
#define MH_MMU_PAGE_FAULT__READ_PROTECTION_ERROR__SHIFT    0x0000000a
#define MH_MMU_PAGE_FAULT__WRITE_PROTECTION_ERROR__SHIFT   0x0000000b
#define MH_MMU_PAGE_FAULT__REQ_VA__SHIFT                   0x0000000c

// MH_MMU_TRAN_ERROR
#define MH_MMU_TRAN_ERROR__TRAN_ERROR__SHIFT               0x00000005

// MH_MMU_INVALIDATE
#define MH_MMU_INVALIDATE__INVALIDATE_ALL__SHIFT           0x00000000
#define MH_MMU_INVALIDATE__INVALIDATE_TC__SHIFT            0x00000001

// MH_MMU_MPU_BASE
#define MH_MMU_MPU_BASE__MPU_BASE__SHIFT                   0x0000000c

// MH_MMU_MPU_END
#define MH_MMU_MPU_END__MPU_END__SHIFT                     0x0000000c

// WAIT_UNTIL
#define WAIT_UNTIL__WAIT_RE_VSYNC__SHIFT                   0x00000001
#define WAIT_UNTIL__WAIT_FE_VSYNC__SHIFT                   0x00000002
#define WAIT_UNTIL__WAIT_VSYNC__SHIFT                      0x00000003
#define WAIT_UNTIL__WAIT_DSPLY_ID0__SHIFT                  0x00000004
#define WAIT_UNTIL__WAIT_DSPLY_ID1__SHIFT                  0x00000005
#define WAIT_UNTIL__WAIT_DSPLY_ID2__SHIFT                  0x00000006
#define WAIT_UNTIL__WAIT_CMDFIFO__SHIFT                    0x0000000a
#define WAIT_UNTIL__WAIT_2D_IDLE__SHIFT                    0x0000000e
#define WAIT_UNTIL__WAIT_3D_IDLE__SHIFT                    0x0000000f
#define WAIT_UNTIL__WAIT_2D_IDLECLEAN__SHIFT               0x00000010
#define WAIT_UNTIL__WAIT_3D_IDLECLEAN__SHIFT               0x00000011
#define WAIT_UNTIL__CMDFIFO_ENTRIES__SHIFT                 0x00000014

// RBBM_ISYNC_CNTL
#define RBBM_ISYNC_CNTL__ISYNC_WAIT_IDLEGUI__SHIFT         0x00000004
#define RBBM_ISYNC_CNTL__ISYNC_CPSCRATCH_IDLEGUI__SHIFT    0x00000005

// RBBM_STATUS
#define RBBM_STATUS__CMDFIFO_AVAIL__SHIFT                  0x00000000
#define RBBM_STATUS__TC_BUSY__SHIFT                        0x00000005
#define RBBM_STATUS__HIRQ_PENDING__SHIFT                   0x00000008
#define RBBM_STATUS__CPRQ_PENDING__SHIFT                   0x00000009
#define RBBM_STATUS__CFRQ_PENDING__SHIFT                   0x0000000a
#define RBBM_STATUS__PFRQ_PENDING__SHIFT                   0x0000000b
#define RBBM_STATUS__VGT_BUSY_NO_DMA__SHIFT                0x0000000c
#define RBBM_STATUS__RBBM_WU_BUSY__SHIFT                   0x0000000e
#define RBBM_STATUS__CP_NRT_BUSY__SHIFT                    0x00000010
#define RBBM_STATUS__MH_BUSY__SHIFT                        0x00000012
#define RBBM_STATUS__MH_COHERENCY_BUSY__SHIFT              0x00000013
#define RBBM_STATUS__SX_BUSY__SHIFT                        0x00000015
#define RBBM_STATUS__TPC_BUSY__SHIFT                       0x00000016
#define RBBM_STATUS__SC_CNTX_BUSY__SHIFT                   0x00000018
#define RBBM_STATUS__PA_BUSY__SHIFT                        0x00000019
#define RBBM_STATUS__VGT_BUSY__SHIFT                       0x0000001a
#define RBBM_STATUS__SQ_CNTX17_BUSY__SHIFT                 0x0000001b
#define RBBM_STATUS__SQ_CNTX0_BUSY__SHIFT                  0x0000001c
#define RBBM_STATUS__RB_CNTX_BUSY__SHIFT                   0x0000001e
#define RBBM_STATUS__GUI_ACTIVE__SHIFT                     0x0000001f

// RBBM_DSPLY
#define RBBM_DSPLY__SEL_DMI_ACTIVE_BUFID0__SHIFT           0x00000000
#define RBBM_DSPLY__SEL_DMI_ACTIVE_BUFID1__SHIFT           0x00000001
#define RBBM_DSPLY__SEL_DMI_ACTIVE_BUFID2__SHIFT           0x00000002
#define RBBM_DSPLY__SEL_DMI_VSYNC_VALID__SHIFT             0x00000003
#define RBBM_DSPLY__DMI_CH1_USE_BUFID0__SHIFT              0x00000004
#define RBBM_DSPLY__DMI_CH1_USE_BUFID1__SHIFT              0x00000005
#define RBBM_DSPLY__DMI_CH1_USE_BUFID2__SHIFT              0x00000006
#define RBBM_DSPLY__DMI_CH1_SW_CNTL__SHIFT                 0x00000007
#define RBBM_DSPLY__DMI_CH1_NUM_BUFS__SHIFT                0x00000008
#define RBBM_DSPLY__DMI_CH2_USE_BUFID0__SHIFT              0x0000000a
#define RBBM_DSPLY__DMI_CH2_USE_BUFID1__SHIFT              0x0000000b
#define RBBM_DSPLY__DMI_CH2_USE_BUFID2__SHIFT              0x0000000c
#define RBBM_DSPLY__DMI_CH2_SW_CNTL__SHIFT                 0x0000000d
#define RBBM_DSPLY__DMI_CH2_NUM_BUFS__SHIFT                0x0000000e
#define RBBM_DSPLY__DMI_CHANNEL_SELECT__SHIFT              0x00000010
#define RBBM_DSPLY__DMI_CH3_USE_BUFID0__SHIFT              0x00000014
#define RBBM_DSPLY__DMI_CH3_USE_BUFID1__SHIFT              0x00000015
#define RBBM_DSPLY__DMI_CH3_USE_BUFID2__SHIFT              0x00000016
#define RBBM_DSPLY__DMI_CH3_SW_CNTL__SHIFT                 0x00000017
#define RBBM_DSPLY__DMI_CH3_NUM_BUFS__SHIFT                0x00000018
#define RBBM_DSPLY__DMI_CH4_USE_BUFID0__SHIFT              0x0000001a
#define RBBM_DSPLY__DMI_CH4_USE_BUFID1__SHIFT              0x0000001b
#define RBBM_DSPLY__DMI_CH4_USE_BUFID2__SHIFT              0x0000001c
#define RBBM_DSPLY__DMI_CH4_SW_CNTL__SHIFT                 0x0000001d
#define RBBM_DSPLY__DMI_CH4_NUM_BUFS__SHIFT                0x0000001e

// RBBM_RENDER_LATEST
#define RBBM_RENDER_LATEST__DMI_CH1_BUFFER_ID__SHIFT       0x00000000
#define RBBM_RENDER_LATEST__DMI_CH2_BUFFER_ID__SHIFT       0x00000008
#define RBBM_RENDER_LATEST__DMI_CH3_BUFFER_ID__SHIFT       0x00000010
#define RBBM_RENDER_LATEST__DMI_CH4_BUFFER_ID__SHIFT       0x00000018

// RBBM_RTL_RELEASE
#define RBBM_RTL_RELEASE__CHANGELIST__SHIFT                0x00000000

// RBBM_PATCH_RELEASE
#define RBBM_PATCH_RELEASE__PATCH_REVISION__SHIFT          0x00000000
#define RBBM_PATCH_RELEASE__PATCH_SELECTION__SHIFT         0x00000010
#define RBBM_PATCH_RELEASE__CUSTOMER_ID__SHIFT             0x00000018

// RBBM_AUXILIARY_CONFIG
#define RBBM_AUXILIARY_CONFIG__RESERVED__SHIFT             0x00000000

// RBBM_PERIPHID0
#define RBBM_PERIPHID0__PARTNUMBER0__SHIFT                 0x00000000

// RBBM_PERIPHID1
#define RBBM_PERIPHID1__PARTNUMBER1__SHIFT                 0x00000000
#define RBBM_PERIPHID1__DESIGNER0__SHIFT                   0x00000004

// RBBM_PERIPHID2
#define RBBM_PERIPHID2__DESIGNER1__SHIFT                   0x00000000
#define RBBM_PERIPHID2__REVISION__SHIFT                    0x00000004

// RBBM_PERIPHID3
#define RBBM_PERIPHID3__RBBM_HOST_INTERFACE__SHIFT         0x00000000
#define RBBM_PERIPHID3__GARB_SLAVE_INTERFACE__SHIFT        0x00000002
#define RBBM_PERIPHID3__MH_INTERFACE__SHIFT                0x00000004
#define RBBM_PERIPHID3__CONTINUATION__SHIFT                0x00000007

// RBBM_CNTL
#define RBBM_CNTL__READ_TIMEOUT__SHIFT                     0x00000000
#define RBBM_CNTL__REGCLK_DEASSERT_TIME__SHIFT             0x00000008

// RBBM_SKEW_CNTL
#define RBBM_SKEW_CNTL__SKEW_TOP_THRESHOLD__SHIFT          0x00000000
#define RBBM_SKEW_CNTL__SKEW_COUNT__SHIFT                  0x00000005

// RBBM_SOFT_RESET
#define RBBM_SOFT_RESET__SOFT_RESET_CP__SHIFT              0x00000000
#define RBBM_SOFT_RESET__SOFT_RESET_PA__SHIFT              0x00000002
#define RBBM_SOFT_RESET__SOFT_RESET_MH__SHIFT              0x00000003
#define RBBM_SOFT_RESET__SOFT_RESET_BC__SHIFT              0x00000004
#define RBBM_SOFT_RESET__SOFT_RESET_SQ__SHIFT              0x00000005
#define RBBM_SOFT_RESET__SOFT_RESET_SX__SHIFT              0x00000006
#define RBBM_SOFT_RESET__SOFT_RESET_CIB__SHIFT             0x0000000c
#define RBBM_SOFT_RESET__SOFT_RESET_SC__SHIFT              0x0000000f
#define RBBM_SOFT_RESET__SOFT_RESET_VGT__SHIFT             0x00000010

// RBBM_PM_OVERRIDE1
#define RBBM_PM_OVERRIDE1__RBBM_AHBCLK_PM_OVERRIDE__SHIFT  0x00000000
#define RBBM_PM_OVERRIDE1__SC_REG_SCLK_PM_OVERRIDE__SHIFT  0x00000001
#define RBBM_PM_OVERRIDE1__SC_SCLK_PM_OVERRIDE__SHIFT      0x00000002
#define RBBM_PM_OVERRIDE1__SP_TOP_SCLK_PM_OVERRIDE__SHIFT  0x00000003
#define RBBM_PM_OVERRIDE1__SP_V0_SCLK_PM_OVERRIDE__SHIFT   0x00000004
#define RBBM_PM_OVERRIDE1__SQ_REG_SCLK_PM_OVERRIDE__SHIFT  0x00000005
#define RBBM_PM_OVERRIDE1__SQ_REG_FIFOS_SCLK_PM_OVERRIDE__SHIFT 0x00000006
#define RBBM_PM_OVERRIDE1__SQ_CONST_MEM_SCLK_PM_OVERRIDE__SHIFT 0x00000007
#define RBBM_PM_OVERRIDE1__SQ_SQ_SCLK_PM_OVERRIDE__SHIFT   0x00000008
#define RBBM_PM_OVERRIDE1__SX_SCLK_PM_OVERRIDE__SHIFT      0x00000009
#define RBBM_PM_OVERRIDE1__SX_REG_SCLK_PM_OVERRIDE__SHIFT  0x0000000a
#define RBBM_PM_OVERRIDE1__TCM_TCO_SCLK_PM_OVERRIDE__SHIFT 0x0000000b
#define RBBM_PM_OVERRIDE1__TCM_TCM_SCLK_PM_OVERRIDE__SHIFT 0x0000000c
#define RBBM_PM_OVERRIDE1__TCM_TCD_SCLK_PM_OVERRIDE__SHIFT 0x0000000d
#define RBBM_PM_OVERRIDE1__TCM_REG_SCLK_PM_OVERRIDE__SHIFT 0x0000000e
#define RBBM_PM_OVERRIDE1__TPC_TPC_SCLK_PM_OVERRIDE__SHIFT 0x0000000f
#define RBBM_PM_OVERRIDE1__TPC_REG_SCLK_PM_OVERRIDE__SHIFT 0x00000010
#define RBBM_PM_OVERRIDE1__TCF_TCA_SCLK_PM_OVERRIDE__SHIFT 0x00000011
#define RBBM_PM_OVERRIDE1__TCF_TCB_SCLK_PM_OVERRIDE__SHIFT 0x00000012
#define RBBM_PM_OVERRIDE1__TCF_TCB_READ_SCLK_PM_OVERRIDE__SHIFT 0x00000013
#define RBBM_PM_OVERRIDE1__TP_TP_SCLK_PM_OVERRIDE__SHIFT   0x00000014
#define RBBM_PM_OVERRIDE1__TP_REG_SCLK_PM_OVERRIDE__SHIFT  0x00000015
#define RBBM_PM_OVERRIDE1__CP_G_SCLK_PM_OVERRIDE__SHIFT    0x00000016
#define RBBM_PM_OVERRIDE1__CP_REG_SCLK_PM_OVERRIDE__SHIFT  0x00000017
#define RBBM_PM_OVERRIDE1__CP_G_REG_SCLK_PM_OVERRIDE__SHIFT 0x00000018
#define RBBM_PM_OVERRIDE1__SPI_SCLK_PM_OVERRIDE__SHIFT     0x00000019
#define RBBM_PM_OVERRIDE1__RB_REG_SCLK_PM_OVERRIDE__SHIFT  0x0000001a
#define RBBM_PM_OVERRIDE1__RB_SCLK_PM_OVERRIDE__SHIFT      0x0000001b
#define RBBM_PM_OVERRIDE1__MH_MH_SCLK_PM_OVERRIDE__SHIFT   0x0000001c
#define RBBM_PM_OVERRIDE1__MH_REG_SCLK_PM_OVERRIDE__SHIFT  0x0000001d
#define RBBM_PM_OVERRIDE1__MH_MMU_SCLK_PM_OVERRIDE__SHIFT  0x0000001e
#define RBBM_PM_OVERRIDE1__MH_TCROQ_SCLK_PM_OVERRIDE__SHIFT 0x0000001f

// RBBM_PM_OVERRIDE2
#define RBBM_PM_OVERRIDE2__PA_REG_SCLK_PM_OVERRIDE__SHIFT  0x00000000
#define RBBM_PM_OVERRIDE2__PA_PA_SCLK_PM_OVERRIDE__SHIFT   0x00000001
#define RBBM_PM_OVERRIDE2__PA_AG_SCLK_PM_OVERRIDE__SHIFT   0x00000002
#define RBBM_PM_OVERRIDE2__VGT_REG_SCLK_PM_OVERRIDE__SHIFT 0x00000003
#define RBBM_PM_OVERRIDE2__VGT_FIFOS_SCLK_PM_OVERRIDE__SHIFT 0x00000004
#define RBBM_PM_OVERRIDE2__VGT_VGT_SCLK_PM_OVERRIDE__SHIFT 0x00000005
#define RBBM_PM_OVERRIDE2__DEBUG_PERF_SCLK_PM_OVERRIDE__SHIFT 0x00000006
#define RBBM_PM_OVERRIDE2__PERM_SCLK_PM_OVERRIDE__SHIFT    0x00000007
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM0_PM_OVERRIDE__SHIFT  0x00000008
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM1_PM_OVERRIDE__SHIFT  0x00000009
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM2_PM_OVERRIDE__SHIFT  0x0000000a
#define RBBM_PM_OVERRIDE2__GC_GA_GMEM3_PM_OVERRIDE__SHIFT  0x0000000b

// GC_SYS_IDLE
#define GC_SYS_IDLE__GC_SYS_IDLE_DELAY__SHIFT              0x00000000
#define GC_SYS_IDLE__GC_SYS_WAIT_DMI_MASK__SHIFT           0x00000010
#define GC_SYS_IDLE__GC_SYS_URGENT_RAMP__SHIFT             0x00000018
#define GC_SYS_IDLE__GC_SYS_WAIT_DMI__SHIFT                0x00000019
#define GC_SYS_IDLE__GC_SYS_URGENT_RAMP_OVERRIDE__SHIFT    0x0000001d
#define GC_SYS_IDLE__GC_SYS_WAIT_DMI_OVERRIDE__SHIFT       0x0000001e
#define GC_SYS_IDLE__GC_SYS_IDLE_OVERRIDE__SHIFT           0x0000001f

// NQWAIT_UNTIL
#define NQWAIT_UNTIL__WAIT_GUI_IDLE__SHIFT                 0x00000000

// RBBM_DEBUG_OUT
#define RBBM_DEBUG_OUT__DEBUG_BUS_OUT__SHIFT               0x00000000

// RBBM_DEBUG_CNTL
#define RBBM_DEBUG_CNTL__SUB_BLOCK_ADDR__SHIFT             0x00000000
#define RBBM_DEBUG_CNTL__SUB_BLOCK_SEL__SHIFT              0x00000008
#define RBBM_DEBUG_CNTL__SW_ENABLE__SHIFT                  0x0000000c
#define RBBM_DEBUG_CNTL__GPIO_SUB_BLOCK_ADDR__SHIFT        0x00000010
#define RBBM_DEBUG_CNTL__GPIO_SUB_BLOCK_SEL__SHIFT         0x00000018
#define RBBM_DEBUG_CNTL__GPIO_BYTE_LANE_ENB__SHIFT         0x0000001c

// RBBM_DEBUG
#define RBBM_DEBUG__IGNORE_RTR__SHIFT                      0x00000001
#define RBBM_DEBUG__IGNORE_CP_SCHED_WU__SHIFT              0x00000002
#define RBBM_DEBUG__IGNORE_CP_SCHED_ISYNC__SHIFT           0x00000003
#define RBBM_DEBUG__IGNORE_CP_SCHED_NQ_HI__SHIFT           0x00000004
#define RBBM_DEBUG__HYSTERESIS_NRT_GUI_ACTIVE__SHIFT       0x00000008
#define RBBM_DEBUG__IGNORE_RTR_FOR_HI__SHIFT               0x00000010
#define RBBM_DEBUG__IGNORE_CP_RBBM_NRTRTR_FOR_HI__SHIFT    0x00000011
#define RBBM_DEBUG__IGNORE_VGT_RBBM_NRTRTR_FOR_HI__SHIFT   0x00000012
#define RBBM_DEBUG__IGNORE_SQ_RBBM_NRTRTR_FOR_HI__SHIFT    0x00000013
#define RBBM_DEBUG__CP_RBBM_NRTRTR__SHIFT                  0x00000014
#define RBBM_DEBUG__VGT_RBBM_NRTRTR__SHIFT                 0x00000015
#define RBBM_DEBUG__SQ_RBBM_NRTRTR__SHIFT                  0x00000016
#define RBBM_DEBUG__CLIENTS_FOR_NRT_RTR_FOR_HI__SHIFT      0x00000017
#define RBBM_DEBUG__CLIENTS_FOR_NRT_RTR__SHIFT             0x00000018
#define RBBM_DEBUG__IGNORE_SX_RBBM_BUSY__SHIFT             0x0000001f

// RBBM_READ_ERROR
#define RBBM_READ_ERROR__READ_ADDRESS__SHIFT               0x00000002
#define RBBM_READ_ERROR__READ_REQUESTER__SHIFT             0x0000001e
#define RBBM_READ_ERROR__READ_ERROR__SHIFT                 0x0000001f

// RBBM_WAIT_IDLE_CLOCKS
#define RBBM_WAIT_IDLE_CLOCKS__WAIT_IDLE_CLOCKS_NRT__SHIFT 0x00000000

// RBBM_INT_CNTL
#define RBBM_INT_CNTL__RDERR_INT_MASK__SHIFT               0x00000000
#define RBBM_INT_CNTL__DISPLAY_UPDATE_INT_MASK__SHIFT      0x00000001
#define RBBM_INT_CNTL__GUI_IDLE_INT_MASK__SHIFT            0x00000013

// RBBM_INT_STATUS
#define RBBM_INT_STATUS__RDERR_INT_STAT__SHIFT             0x00000000
#define RBBM_INT_STATUS__DISPLAY_UPDATE_INT_STAT__SHIFT    0x00000001
#define RBBM_INT_STATUS__GUI_IDLE_INT_STAT__SHIFT          0x00000013

// RBBM_INT_ACK
#define RBBM_INT_ACK__RDERR_INT_ACK__SHIFT                 0x00000000
#define RBBM_INT_ACK__DISPLAY_UPDATE_INT_ACK__SHIFT        0x00000001
#define RBBM_INT_ACK__GUI_IDLE_INT_ACK__SHIFT              0x00000013

// MASTER_INT_SIGNAL
#define MASTER_INT_SIGNAL__MH_INT_STAT__SHIFT              0x00000005
#define MASTER_INT_SIGNAL__SQ_INT_STAT__SHIFT              0x0000001a
#define MASTER_INT_SIGNAL__CP_INT_STAT__SHIFT              0x0000001e
#define MASTER_INT_SIGNAL__RBBM_INT_STAT__SHIFT            0x0000001f

// RBBM_PERFCOUNTER1_SELECT
#define RBBM_PERFCOUNTER1_SELECT__PERF_COUNT1_SEL__SHIFT   0x00000000

// RBBM_PERFCOUNTER1_LO
#define RBBM_PERFCOUNTER1_LO__PERF_COUNT1_LO__SHIFT        0x00000000

// RBBM_PERFCOUNTER1_HI
#define RBBM_PERFCOUNTER1_HI__PERF_COUNT1_HI__SHIFT        0x00000000

// CP_RB_BASE
#define CP_RB_BASE__RB_BASE__SHIFT                         0x00000005

// CP_RB_CNTL
#define CP_RB_CNTL__RB_BUFSZ__SHIFT                        0x00000000
#define CP_RB_CNTL__RB_BLKSZ__SHIFT                        0x00000008
#define CP_RB_CNTL__BUF_SWAP__SHIFT                        0x00000010
#define CP_RB_CNTL__RB_POLL_EN__SHIFT                      0x00000014
#define CP_RB_CNTL__RB_NO_UPDATE__SHIFT                    0x0000001b
#define CP_RB_CNTL__RB_RPTR_WR_ENA__SHIFT                  0x0000001f

// CP_RB_RPTR_ADDR
#define CP_RB_RPTR_ADDR__RB_RPTR_SWAP__SHIFT               0x00000000
#define CP_RB_RPTR_ADDR__RB_RPTR_ADDR__SHIFT               0x00000002

// CP_RB_RPTR
#define CP_RB_RPTR__RB_RPTR__SHIFT                         0x00000000

// CP_RB_RPTR_WR
#define CP_RB_RPTR_WR__RB_RPTR_WR__SHIFT                   0x00000000

// CP_RB_WPTR
#define CP_RB_WPTR__RB_WPTR__SHIFT                         0x00000000

// CP_RB_WPTR_DELAY
#define CP_RB_WPTR_DELAY__PRE_WRITE_TIMER__SHIFT           0x00000000
#define CP_RB_WPTR_DELAY__PRE_WRITE_LIMIT__SHIFT           0x0000001c

// CP_RB_WPTR_BASE
#define CP_RB_WPTR_BASE__RB_WPTR_SWAP__SHIFT               0x00000000
#define CP_RB_WPTR_BASE__RB_WPTR_BASE__SHIFT               0x00000002

// CP_IB1_BASE
#define CP_IB1_BASE__IB1_BASE__SHIFT                       0x00000002

// CP_IB1_BUFSZ
#define CP_IB1_BUFSZ__IB1_BUFSZ__SHIFT                     0x00000000

// CP_IB2_BASE
#define CP_IB2_BASE__IB2_BASE__SHIFT                       0x00000002

// CP_IB2_BUFSZ
#define CP_IB2_BUFSZ__IB2_BUFSZ__SHIFT                     0x00000000

// CP_ST_BASE
#define CP_ST_BASE__ST_BASE__SHIFT                         0x00000002

// CP_ST_BUFSZ
#define CP_ST_BUFSZ__ST_BUFSZ__SHIFT                       0x00000000

// CP_QUEUE_THRESHOLDS
#define CP_QUEUE_THRESHOLDS__CSQ_IB1_START__SHIFT          0x00000000
#define CP_QUEUE_THRESHOLDS__CSQ_IB2_START__SHIFT          0x00000008
#define CP_QUEUE_THRESHOLDS__CSQ_ST_START__SHIFT           0x00000010

// CP_MEQ_THRESHOLDS
#define CP_MEQ_THRESHOLDS__MEQ_END__SHIFT                  0x00000010
#define CP_MEQ_THRESHOLDS__ROQ_END__SHIFT                  0x00000018

// CP_CSQ_AVAIL
#define CP_CSQ_AVAIL__CSQ_CNT_RING__SHIFT                  0x00000000
#define CP_CSQ_AVAIL__CSQ_CNT_IB1__SHIFT                   0x00000008
#define CP_CSQ_AVAIL__CSQ_CNT_IB2__SHIFT                   0x00000010

// CP_STQ_AVAIL
#define CP_STQ_AVAIL__STQ_CNT_ST__SHIFT                    0x00000000

// CP_MEQ_AVAIL
#define CP_MEQ_AVAIL__MEQ_CNT__SHIFT                       0x00000000

// CP_CSQ_RB_STAT
#define CP_CSQ_RB_STAT__CSQ_RPTR_PRIMARY__SHIFT            0x00000000
#define CP_CSQ_RB_STAT__CSQ_WPTR_PRIMARY__SHIFT            0x00000010

// CP_CSQ_IB1_STAT
#define CP_CSQ_IB1_STAT__CSQ_RPTR_INDIRECT1__SHIFT         0x00000000
#define CP_CSQ_IB1_STAT__CSQ_WPTR_INDIRECT1__SHIFT         0x00000010

// CP_CSQ_IB2_STAT
#define CP_CSQ_IB2_STAT__CSQ_RPTR_INDIRECT2__SHIFT         0x00000000
#define CP_CSQ_IB2_STAT__CSQ_WPTR_INDIRECT2__SHIFT         0x00000010

// CP_NON_PREFETCH_CNTRS
#define CP_NON_PREFETCH_CNTRS__IB1_COUNTER__SHIFT          0x00000000
#define CP_NON_PREFETCH_CNTRS__IB2_COUNTER__SHIFT          0x00000008

// CP_STQ_ST_STAT
#define CP_STQ_ST_STAT__STQ_RPTR_ST__SHIFT                 0x00000000
#define CP_STQ_ST_STAT__STQ_WPTR_ST__SHIFT                 0x00000010

// CP_MEQ_STAT
#define CP_MEQ_STAT__MEQ_RPTR__SHIFT                       0x00000000
#define CP_MEQ_STAT__MEQ_WPTR__SHIFT                       0x00000010

// CP_MIU_TAG_STAT
#define CP_MIU_TAG_STAT__TAG_0_STAT__SHIFT                 0x00000000
#define CP_MIU_TAG_STAT__TAG_1_STAT__SHIFT                 0x00000001
#define CP_MIU_TAG_STAT__TAG_2_STAT__SHIFT                 0x00000002
#define CP_MIU_TAG_STAT__TAG_3_STAT__SHIFT                 0x00000003
#define CP_MIU_TAG_STAT__TAG_4_STAT__SHIFT                 0x00000004
#define CP_MIU_TAG_STAT__TAG_5_STAT__SHIFT                 0x00000005
#define CP_MIU_TAG_STAT__TAG_6_STAT__SHIFT                 0x00000006
#define CP_MIU_TAG_STAT__TAG_7_STAT__SHIFT                 0x00000007
#define CP_MIU_TAG_STAT__TAG_8_STAT__SHIFT                 0x00000008
#define CP_MIU_TAG_STAT__TAG_9_STAT__SHIFT                 0x00000009
#define CP_MIU_TAG_STAT__TAG_10_STAT__SHIFT                0x0000000a
#define CP_MIU_TAG_STAT__TAG_11_STAT__SHIFT                0x0000000b
#define CP_MIU_TAG_STAT__TAG_12_STAT__SHIFT                0x0000000c
#define CP_MIU_TAG_STAT__TAG_13_STAT__SHIFT                0x0000000d
#define CP_MIU_TAG_STAT__TAG_14_STAT__SHIFT                0x0000000e
#define CP_MIU_TAG_STAT__TAG_15_STAT__SHIFT                0x0000000f
#define CP_MIU_TAG_STAT__TAG_16_STAT__SHIFT                0x00000010
#define CP_MIU_TAG_STAT__TAG_17_STAT__SHIFT                0x00000011
#define CP_MIU_TAG_STAT__INVALID_RETURN_TAG__SHIFT         0x0000001f

// CP_CMD_INDEX
#define CP_CMD_INDEX__CMD_INDEX__SHIFT                     0x00000000
#define CP_CMD_INDEX__CMD_QUEUE_SEL__SHIFT                 0x00000010

// CP_CMD_DATA
#define CP_CMD_DATA__CMD_DATA__SHIFT                       0x00000000

// CP_ME_CNTL
#define CP_ME_CNTL__ME_STATMUX__SHIFT                      0x00000000
#define CP_ME_CNTL__VTX_DEALLOC_FIFO_EMPTY__SHIFT          0x00000019
#define CP_ME_CNTL__PIX_DEALLOC_FIFO_EMPTY__SHIFT          0x0000001a
#define CP_ME_CNTL__ME_HALT__SHIFT                         0x0000001c
#define CP_ME_CNTL__ME_BUSY__SHIFT                         0x0000001d
#define CP_ME_CNTL__PROG_CNT_SIZE__SHIFT                   0x0000001f

// CP_ME_STATUS
#define CP_ME_STATUS__ME_DEBUG_DATA__SHIFT                 0x00000000

// CP_ME_RAM_WADDR
#define CP_ME_RAM_WADDR__ME_RAM_WADDR__SHIFT               0x00000000

// CP_ME_RAM_RADDR
#define CP_ME_RAM_RADDR__ME_RAM_RADDR__SHIFT               0x00000000

// CP_ME_RAM_DATA
#define CP_ME_RAM_DATA__ME_RAM_DATA__SHIFT                 0x00000000

// CP_ME_RDADDR
#define CP_ME_RDADDR__ME_RDADDR__SHIFT                     0x00000000

// CP_DEBUG
#define CP_DEBUG__CP_DEBUG_UNUSED_22_to_0__SHIFT           0x00000000
#define CP_DEBUG__PREDICATE_DISABLE__SHIFT                 0x00000017
#define CP_DEBUG__PROG_END_PTR_ENABLE__SHIFT               0x00000018
#define CP_DEBUG__MIU_128BIT_WRITE_ENABLE__SHIFT           0x00000019
#define CP_DEBUG__PREFETCH_PASS_NOPS__SHIFT                0x0000001a
#define CP_DEBUG__DYNAMIC_CLK_DISABLE__SHIFT               0x0000001b
#define CP_DEBUG__PREFETCH_MATCH_DISABLE__SHIFT            0x0000001c
#define CP_DEBUG__SIMPLE_ME_FLOW_CONTROL__SHIFT            0x0000001e
#define CP_DEBUG__MIU_WRITE_PACK_DISABLE__SHIFT            0x0000001f

// SCRATCH_REG0
#define SCRATCH_REG0__SCRATCH_REG0__SHIFT                  0x00000000
#define GUI_SCRATCH_REG0__SCRATCH_REG0__SHIFT              0x00000000

// SCRATCH_REG1
#define SCRATCH_REG1__SCRATCH_REG1__SHIFT                  0x00000000
#define GUI_SCRATCH_REG1__SCRATCH_REG1__SHIFT              0x00000000

// SCRATCH_REG2
#define SCRATCH_REG2__SCRATCH_REG2__SHIFT                  0x00000000
#define GUI_SCRATCH_REG2__SCRATCH_REG2__SHIFT              0x00000000

// SCRATCH_REG3
#define SCRATCH_REG3__SCRATCH_REG3__SHIFT                  0x00000000
#define GUI_SCRATCH_REG3__SCRATCH_REG3__SHIFT              0x00000000

// SCRATCH_REG4
#define SCRATCH_REG4__SCRATCH_REG4__SHIFT                  0x00000000
#define GUI_SCRATCH_REG4__SCRATCH_REG4__SHIFT              0x00000000

// SCRATCH_REG5
#define SCRATCH_REG5__SCRATCH_REG5__SHIFT                  0x00000000
#define GUI_SCRATCH_REG5__SCRATCH_REG5__SHIFT              0x00000000

// SCRATCH_REG6
#define SCRATCH_REG6__SCRATCH_REG6__SHIFT                  0x00000000
#define GUI_SCRATCH_REG6__SCRATCH_REG6__SHIFT              0x00000000

// SCRATCH_REG7
#define SCRATCH_REG7__SCRATCH_REG7__SHIFT                  0x00000000
#define GUI_SCRATCH_REG7__SCRATCH_REG7__SHIFT              0x00000000

// SCRATCH_UMSK
#define SCRATCH_UMSK__SCRATCH_UMSK__SHIFT                  0x00000000
#define SCRATCH_UMSK__SCRATCH_SWAP__SHIFT                  0x00000010

// SCRATCH_ADDR
#define SCRATCH_ADDR__SCRATCH_ADDR__SHIFT                  0x00000005

// CP_ME_VS_EVENT_SRC
#define CP_ME_VS_EVENT_SRC__VS_DONE_SWM__SHIFT             0x00000000
#define CP_ME_VS_EVENT_SRC__VS_DONE_CNTR__SHIFT            0x00000001

// CP_ME_VS_EVENT_ADDR
#define CP_ME_VS_EVENT_ADDR__VS_DONE_SWAP__SHIFT           0x00000000
#define CP_ME_VS_EVENT_ADDR__VS_DONE_ADDR__SHIFT           0x00000002

// CP_ME_VS_EVENT_DATA
#define CP_ME_VS_EVENT_DATA__VS_DONE_DATA__SHIFT           0x00000000

// CP_ME_VS_EVENT_ADDR_SWM
#define CP_ME_VS_EVENT_ADDR_SWM__VS_DONE_SWAP_SWM__SHIFT   0x00000000
#define CP_ME_VS_EVENT_ADDR_SWM__VS_DONE_ADDR_SWM__SHIFT   0x00000002

// CP_ME_VS_EVENT_DATA_SWM
#define CP_ME_VS_EVENT_DATA_SWM__VS_DONE_DATA_SWM__SHIFT   0x00000000

// CP_ME_PS_EVENT_SRC
#define CP_ME_PS_EVENT_SRC__PS_DONE_SWM__SHIFT             0x00000000
#define CP_ME_PS_EVENT_SRC__PS_DONE_CNTR__SHIFT            0x00000001

// CP_ME_PS_EVENT_ADDR
#define CP_ME_PS_EVENT_ADDR__PS_DONE_SWAP__SHIFT           0x00000000
#define CP_ME_PS_EVENT_ADDR__PS_DONE_ADDR__SHIFT           0x00000002

// CP_ME_PS_EVENT_DATA
#define CP_ME_PS_EVENT_DATA__PS_DONE_DATA__SHIFT           0x00000000

// CP_ME_PS_EVENT_ADDR_SWM
#define CP_ME_PS_EVENT_ADDR_SWM__PS_DONE_SWAP_SWM__SHIFT   0x00000000
#define CP_ME_PS_EVENT_ADDR_SWM__PS_DONE_ADDR_SWM__SHIFT   0x00000002

// CP_ME_PS_EVENT_DATA_SWM
#define CP_ME_PS_EVENT_DATA_SWM__PS_DONE_DATA_SWM__SHIFT   0x00000000

// CP_ME_CF_EVENT_SRC
#define CP_ME_CF_EVENT_SRC__CF_DONE_SRC__SHIFT             0x00000000

// CP_ME_CF_EVENT_ADDR
#define CP_ME_CF_EVENT_ADDR__CF_DONE_SWAP__SHIFT           0x00000000
#define CP_ME_CF_EVENT_ADDR__CF_DONE_ADDR__SHIFT           0x00000002

// CP_ME_CF_EVENT_DATA
#define CP_ME_CF_EVENT_DATA__CF_DONE_DATA__SHIFT           0x00000000

// CP_ME_NRT_ADDR
#define CP_ME_NRT_ADDR__NRT_WRITE_SWAP__SHIFT              0x00000000
#define CP_ME_NRT_ADDR__NRT_WRITE_ADDR__SHIFT              0x00000002

// CP_ME_NRT_DATA
#define CP_ME_NRT_DATA__NRT_WRITE_DATA__SHIFT              0x00000000

// CP_ME_VS_FETCH_DONE_SRC
#define CP_ME_VS_FETCH_DONE_SRC__VS_FETCH_DONE_CNTR__SHIFT 0x00000000

// CP_ME_VS_FETCH_DONE_ADDR
#define CP_ME_VS_FETCH_DONE_ADDR__VS_FETCH_DONE_SWAP__SHIFT 0x00000000
#define CP_ME_VS_FETCH_DONE_ADDR__VS_FETCH_DONE_ADDR__SHIFT 0x00000002

// CP_ME_VS_FETCH_DONE_DATA
#define CP_ME_VS_FETCH_DONE_DATA__VS_FETCH_DONE_DATA__SHIFT 0x00000000

// CP_INT_CNTL
#define CP_INT_CNTL__SW_INT_MASK__SHIFT                    0x00000013
#define CP_INT_CNTL__T0_PACKET_IN_IB_MASK__SHIFT           0x00000017
#define CP_INT_CNTL__OPCODE_ERROR_MASK__SHIFT              0x00000018
#define CP_INT_CNTL__PROTECTED_MODE_ERROR_MASK__SHIFT      0x00000019
#define CP_INT_CNTL__RESERVED_BIT_ERROR_MASK__SHIFT        0x0000001a
#define CP_INT_CNTL__IB_ERROR_MASK__SHIFT                  0x0000001b
#define CP_INT_CNTL__IB2_INT_MASK__SHIFT                   0x0000001d
#define CP_INT_CNTL__IB1_INT_MASK__SHIFT                   0x0000001e
#define CP_INT_CNTL__RB_INT_MASK__SHIFT                    0x0000001f

// CP_INT_STATUS
#define CP_INT_STATUS__SW_INT_STAT__SHIFT                  0x00000013
#define CP_INT_STATUS__T0_PACKET_IN_IB_STAT__SHIFT         0x00000017
#define CP_INT_STATUS__OPCODE_ERROR_STAT__SHIFT            0x00000018
#define CP_INT_STATUS__PROTECTED_MODE_ERROR_STAT__SHIFT    0x00000019
#define CP_INT_STATUS__RESERVED_BIT_ERROR_STAT__SHIFT      0x0000001a
#define CP_INT_STATUS__IB_ERROR_STAT__SHIFT                0x0000001b
#define CP_INT_STATUS__IB2_INT_STAT__SHIFT                 0x0000001d
#define CP_INT_STATUS__IB1_INT_STAT__SHIFT                 0x0000001e
#define CP_INT_STATUS__RB_INT_STAT__SHIFT                  0x0000001f

// CP_INT_ACK
#define CP_INT_ACK__SW_INT_ACK__SHIFT                      0x00000013
#define CP_INT_ACK__T0_PACKET_IN_IB_ACK__SHIFT             0x00000017
#define CP_INT_ACK__OPCODE_ERROR_ACK__SHIFT                0x00000018
#define CP_INT_ACK__PROTECTED_MODE_ERROR_ACK__SHIFT        0x00000019
#define CP_INT_ACK__RESERVED_BIT_ERROR_ACK__SHIFT          0x0000001a
#define CP_INT_ACK__IB_ERROR_ACK__SHIFT                    0x0000001b
#define CP_INT_ACK__IB2_INT_ACK__SHIFT                     0x0000001d
#define CP_INT_ACK__IB1_INT_ACK__SHIFT                     0x0000001e
#define CP_INT_ACK__RB_INT_ACK__SHIFT                      0x0000001f

// CP_PFP_UCODE_ADDR
#define CP_PFP_UCODE_ADDR__UCODE_ADDR__SHIFT               0x00000000

// CP_PFP_UCODE_DATA
#define CP_PFP_UCODE_DATA__UCODE_DATA__SHIFT               0x00000000

// CP_PERFMON_CNTL
#define CP_PERFMON_CNTL__PERFMON_STATE__SHIFT              0x00000000
#define CP_PERFMON_CNTL__PERFMON_ENABLE_MODE__SHIFT        0x00000008

// CP_PERFCOUNTER_SELECT
#define CP_PERFCOUNTER_SELECT__PERFCOUNT_SEL__SHIFT        0x00000000

// CP_PERFCOUNTER_LO
#define CP_PERFCOUNTER_LO__PERFCOUNT_LO__SHIFT             0x00000000

// CP_PERFCOUNTER_HI
#define CP_PERFCOUNTER_HI__PERFCOUNT_HI__SHIFT             0x00000000

// CP_BIN_MASK_LO
#define CP_BIN_MASK_LO__BIN_MASK_LO__SHIFT                 0x00000000

// CP_BIN_MASK_HI
#define CP_BIN_MASK_HI__BIN_MASK_HI__SHIFT                 0x00000000

// CP_BIN_SELECT_LO
#define CP_BIN_SELECT_LO__BIN_SELECT_LO__SHIFT             0x00000000

// CP_BIN_SELECT_HI
#define CP_BIN_SELECT_HI__BIN_SELECT_HI__SHIFT             0x00000000

// CP_NV_FLAGS_0
#define CP_NV_FLAGS_0__DISCARD_0__SHIFT                    0x00000000
#define CP_NV_FLAGS_0__END_RCVD_0__SHIFT                   0x00000001
#define CP_NV_FLAGS_0__DISCARD_1__SHIFT                    0x00000002
#define CP_NV_FLAGS_0__END_RCVD_1__SHIFT                   0x00000003
#define CP_NV_FLAGS_0__DISCARD_2__SHIFT                    0x00000004
#define CP_NV_FLAGS_0__END_RCVD_2__SHIFT                   0x00000005
#define CP_NV_FLAGS_0__DISCARD_3__SHIFT                    0x00000006
#define CP_NV_FLAGS_0__END_RCVD_3__SHIFT                   0x00000007
#define CP_NV_FLAGS_0__DISCARD_4__SHIFT                    0x00000008
#define CP_NV_FLAGS_0__END_RCVD_4__SHIFT                   0x00000009
#define CP_NV_FLAGS_0__DISCARD_5__SHIFT                    0x0000000a
#define CP_NV_FLAGS_0__END_RCVD_5__SHIFT                   0x0000000b
#define CP_NV_FLAGS_0__DISCARD_6__SHIFT                    0x0000000c
#define CP_NV_FLAGS_0__END_RCVD_6__SHIFT                   0x0000000d
#define CP_NV_FLAGS_0__DISCARD_7__SHIFT                    0x0000000e
#define CP_NV_FLAGS_0__END_RCVD_7__SHIFT                   0x0000000f
#define CP_NV_FLAGS_0__DISCARD_8__SHIFT                    0x00000010
#define CP_NV_FLAGS_0__END_RCVD_8__SHIFT                   0x00000011
#define CP_NV_FLAGS_0__DISCARD_9__SHIFT                    0x00000012
#define CP_NV_FLAGS_0__END_RCVD_9__SHIFT                   0x00000013
#define CP_NV_FLAGS_0__DISCARD_10__SHIFT                   0x00000014
#define CP_NV_FLAGS_0__END_RCVD_10__SHIFT                  0x00000015
#define CP_NV_FLAGS_0__DISCARD_11__SHIFT                   0x00000016
#define CP_NV_FLAGS_0__END_RCVD_11__SHIFT                  0x00000017
#define CP_NV_FLAGS_0__DISCARD_12__SHIFT                   0x00000018
#define CP_NV_FLAGS_0__END_RCVD_12__SHIFT                  0x00000019
#define CP_NV_FLAGS_0__DISCARD_13__SHIFT                   0x0000001a
#define CP_NV_FLAGS_0__END_RCVD_13__SHIFT                  0x0000001b
#define CP_NV_FLAGS_0__DISCARD_14__SHIFT                   0x0000001c
#define CP_NV_FLAGS_0__END_RCVD_14__SHIFT                  0x0000001d
#define CP_NV_FLAGS_0__DISCARD_15__SHIFT                   0x0000001e
#define CP_NV_FLAGS_0__END_RCVD_15__SHIFT                  0x0000001f

// CP_NV_FLAGS_1
#define CP_NV_FLAGS_1__DISCARD_16__SHIFT                   0x00000000
#define CP_NV_FLAGS_1__END_RCVD_16__SHIFT                  0x00000001
#define CP_NV_FLAGS_1__DISCARD_17__SHIFT                   0x00000002
#define CP_NV_FLAGS_1__END_RCVD_17__SHIFT                  0x00000003
#define CP_NV_FLAGS_1__DISCARD_18__SHIFT                   0x00000004
#define CP_NV_FLAGS_1__END_RCVD_18__SHIFT                  0x00000005
#define CP_NV_FLAGS_1__DISCARD_19__SHIFT                   0x00000006
#define CP_NV_FLAGS_1__END_RCVD_19__SHIFT                  0x00000007
#define CP_NV_FLAGS_1__DISCARD_20__SHIFT                   0x00000008
#define CP_NV_FLAGS_1__END_RCVD_20__SHIFT                  0x00000009
#define CP_NV_FLAGS_1__DISCARD_21__SHIFT                   0x0000000a
#define CP_NV_FLAGS_1__END_RCVD_21__SHIFT                  0x0000000b
#define CP_NV_FLAGS_1__DISCARD_22__SHIFT                   0x0000000c
#define CP_NV_FLAGS_1__END_RCVD_22__SHIFT                  0x0000000d
#define CP_NV_FLAGS_1__DISCARD_23__SHIFT                   0x0000000e
#define CP_NV_FLAGS_1__END_RCVD_23__SHIFT                  0x0000000f
#define CP_NV_FLAGS_1__DISCARD_24__SHIFT                   0x00000010
#define CP_NV_FLAGS_1__END_RCVD_24__SHIFT                  0x00000011
#define CP_NV_FLAGS_1__DISCARD_25__SHIFT                   0x00000012
#define CP_NV_FLAGS_1__END_RCVD_25__SHIFT                  0x00000013
#define CP_NV_FLAGS_1__DISCARD_26__SHIFT                   0x00000014
#define CP_NV_FLAGS_1__END_RCVD_26__SHIFT                  0x00000015
#define CP_NV_FLAGS_1__DISCARD_27__SHIFT                   0x00000016
#define CP_NV_FLAGS_1__END_RCVD_27__SHIFT                  0x00000017
#define CP_NV_FLAGS_1__DISCARD_28__SHIFT                   0x00000018
#define CP_NV_FLAGS_1__END_RCVD_28__SHIFT                  0x00000019
#define CP_NV_FLAGS_1__DISCARD_29__SHIFT                   0x0000001a
#define CP_NV_FLAGS_1__END_RCVD_29__SHIFT                  0x0000001b
#define CP_NV_FLAGS_1__DISCARD_30__SHIFT                   0x0000001c
#define CP_NV_FLAGS_1__END_RCVD_30__SHIFT                  0x0000001d
#define CP_NV_FLAGS_1__DISCARD_31__SHIFT                   0x0000001e
#define CP_NV_FLAGS_1__END_RCVD_31__SHIFT                  0x0000001f

// CP_NV_FLAGS_2
#define CP_NV_FLAGS_2__DISCARD_32__SHIFT                   0x00000000
#define CP_NV_FLAGS_2__END_RCVD_32__SHIFT                  0x00000001
#define CP_NV_FLAGS_2__DISCARD_33__SHIFT                   0x00000002
#define CP_NV_FLAGS_2__END_RCVD_33__SHIFT                  0x00000003
#define CP_NV_FLAGS_2__DISCARD_34__SHIFT                   0x00000004
#define CP_NV_FLAGS_2__END_RCVD_34__SHIFT                  0x00000005
#define CP_NV_FLAGS_2__DISCARD_35__SHIFT                   0x00000006
#define CP_NV_FLAGS_2__END_RCVD_35__SHIFT                  0x00000007
#define CP_NV_FLAGS_2__DISCARD_36__SHIFT                   0x00000008
#define CP_NV_FLAGS_2__END_RCVD_36__SHIFT                  0x00000009
#define CP_NV_FLAGS_2__DISCARD_37__SHIFT                   0x0000000a
#define CP_NV_FLAGS_2__END_RCVD_37__SHIFT                  0x0000000b
#define CP_NV_FLAGS_2__DISCARD_38__SHIFT                   0x0000000c
#define CP_NV_FLAGS_2__END_RCVD_38__SHIFT                  0x0000000d
#define CP_NV_FLAGS_2__DISCARD_39__SHIFT                   0x0000000e
#define CP_NV_FLAGS_2__END_RCVD_39__SHIFT                  0x0000000f
#define CP_NV_FLAGS_2__DISCARD_40__SHIFT                   0x00000010
#define CP_NV_FLAGS_2__END_RCVD_40__SHIFT                  0x00000011
#define CP_NV_FLAGS_2__DISCARD_41__SHIFT                   0x00000012
#define CP_NV_FLAGS_2__END_RCVD_41__SHIFT                  0x00000013
#define CP_NV_FLAGS_2__DISCARD_42__SHIFT                   0x00000014
#define CP_NV_FLAGS_2__END_RCVD_42__SHIFT                  0x00000015
#define CP_NV_FLAGS_2__DISCARD_43__SHIFT                   0x00000016
#define CP_NV_FLAGS_2__END_RCVD_43__SHIFT                  0x00000017
#define CP_NV_FLAGS_2__DISCARD_44__SHIFT                   0x00000018
#define CP_NV_FLAGS_2__END_RCVD_44__SHIFT                  0x00000019
#define CP_NV_FLAGS_2__DISCARD_45__SHIFT                   0x0000001a
#define CP_NV_FLAGS_2__END_RCVD_45__SHIFT                  0x0000001b
#define CP_NV_FLAGS_2__DISCARD_46__SHIFT                   0x0000001c
#define CP_NV_FLAGS_2__END_RCVD_46__SHIFT                  0x0000001d
#define CP_NV_FLAGS_2__DISCARD_47__SHIFT                   0x0000001e
#define CP_NV_FLAGS_2__END_RCVD_47__SHIFT                  0x0000001f

// CP_NV_FLAGS_3
#define CP_NV_FLAGS_3__DISCARD_48__SHIFT                   0x00000000
#define CP_NV_FLAGS_3__END_RCVD_48__SHIFT                  0x00000001
#define CP_NV_FLAGS_3__DISCARD_49__SHIFT                   0x00000002
#define CP_NV_FLAGS_3__END_RCVD_49__SHIFT                  0x00000003
#define CP_NV_FLAGS_3__DISCARD_50__SHIFT                   0x00000004
#define CP_NV_FLAGS_3__END_RCVD_50__SHIFT                  0x00000005
#define CP_NV_FLAGS_3__DISCARD_51__SHIFT                   0x00000006
#define CP_NV_FLAGS_3__END_RCVD_51__SHIFT                  0x00000007
#define CP_NV_FLAGS_3__DISCARD_52__SHIFT                   0x00000008
#define CP_NV_FLAGS_3__END_RCVD_52__SHIFT                  0x00000009
#define CP_NV_FLAGS_3__DISCARD_53__SHIFT                   0x0000000a
#define CP_NV_FLAGS_3__END_RCVD_53__SHIFT                  0x0000000b
#define CP_NV_FLAGS_3__DISCARD_54__SHIFT                   0x0000000c
#define CP_NV_FLAGS_3__END_RCVD_54__SHIFT                  0x0000000d
#define CP_NV_FLAGS_3__DISCARD_55__SHIFT                   0x0000000e
#define CP_NV_FLAGS_3__END_RCVD_55__SHIFT                  0x0000000f
#define CP_NV_FLAGS_3__DISCARD_56__SHIFT                   0x00000010
#define CP_NV_FLAGS_3__END_RCVD_56__SHIFT                  0x00000011
#define CP_NV_FLAGS_3__DISCARD_57__SHIFT                   0x00000012
#define CP_NV_FLAGS_3__END_RCVD_57__SHIFT                  0x00000013
#define CP_NV_FLAGS_3__DISCARD_58__SHIFT                   0x00000014
#define CP_NV_FLAGS_3__END_RCVD_58__SHIFT                  0x00000015
#define CP_NV_FLAGS_3__DISCARD_59__SHIFT                   0x00000016
#define CP_NV_FLAGS_3__END_RCVD_59__SHIFT                  0x00000017
#define CP_NV_FLAGS_3__DISCARD_60__SHIFT                   0x00000018
#define CP_NV_FLAGS_3__END_RCVD_60__SHIFT                  0x00000019
#define CP_NV_FLAGS_3__DISCARD_61__SHIFT                   0x0000001a
#define CP_NV_FLAGS_3__END_RCVD_61__SHIFT                  0x0000001b
#define CP_NV_FLAGS_3__DISCARD_62__SHIFT                   0x0000001c
#define CP_NV_FLAGS_3__END_RCVD_62__SHIFT                  0x0000001d
#define CP_NV_FLAGS_3__DISCARD_63__SHIFT                   0x0000001e
#define CP_NV_FLAGS_3__END_RCVD_63__SHIFT                  0x0000001f

// CP_STATE_DEBUG_INDEX
#define CP_STATE_DEBUG_INDEX__STATE_DEBUG_INDEX__SHIFT     0x00000000

// CP_STATE_DEBUG_DATA
#define CP_STATE_DEBUG_DATA__STATE_DEBUG_DATA__SHIFT       0x00000000

// CP_PROG_COUNTER
#define CP_PROG_COUNTER__COUNTER__SHIFT                    0x00000000

// CP_STAT
#define CP_STAT__MIU_WR_BUSY__SHIFT                        0x00000000
#define CP_STAT__MIU_RD_REQ_BUSY__SHIFT                    0x00000001
#define CP_STAT__MIU_RD_RETURN_BUSY__SHIFT                 0x00000002
#define CP_STAT__RBIU_BUSY__SHIFT                          0x00000003
#define CP_STAT__RCIU_BUSY__SHIFT                          0x00000004
#define CP_STAT__CSF_RING_BUSY__SHIFT                      0x00000005
#define CP_STAT__CSF_INDIRECTS_BUSY__SHIFT                 0x00000006
#define CP_STAT__CSF_INDIRECT2_BUSY__SHIFT                 0x00000007
#define CP_STAT__CSF_ST_BUSY__SHIFT                        0x00000009
#define CP_STAT__CSF_BUSY__SHIFT                           0x0000000a
#define CP_STAT__RING_QUEUE_BUSY__SHIFT                    0x0000000b
#define CP_STAT__INDIRECTS_QUEUE_BUSY__SHIFT               0x0000000c
#define CP_STAT__INDIRECT2_QUEUE_BUSY__SHIFT               0x0000000d
#define CP_STAT__ST_QUEUE_BUSY__SHIFT                      0x00000010
#define CP_STAT__PFP_BUSY__SHIFT                           0x00000011
#define CP_STAT__MEQ_RING_BUSY__SHIFT                      0x00000012
#define CP_STAT__MEQ_INDIRECTS_BUSY__SHIFT                 0x00000013
#define CP_STAT__MEQ_INDIRECT2_BUSY__SHIFT                 0x00000014
#define CP_STAT__MIU_WC_STALL__SHIFT                       0x00000015
#define CP_STAT__CP_NRT_BUSY__SHIFT                        0x00000016
#define CP_STAT___3D_BUSY__SHIFT                           0x00000017
#define CP_STAT__ME_BUSY__SHIFT                            0x0000001a
#define CP_STAT__ME_WC_BUSY__SHIFT                         0x0000001d
#define CP_STAT__MIU_WC_TRACK_FIFO_EMPTY__SHIFT            0x0000001e
#define CP_STAT__CP_BUSY__SHIFT                            0x0000001f

// BIOS_0_SCRATCH
#define BIOS_0_SCRATCH__BIOS_SCRATCH__SHIFT                0x00000000

// BIOS_1_SCRATCH
#define BIOS_1_SCRATCH__BIOS_SCRATCH__SHIFT                0x00000000

// BIOS_2_SCRATCH
#define BIOS_2_SCRATCH__BIOS_SCRATCH__SHIFT                0x00000000

// BIOS_3_SCRATCH
#define BIOS_3_SCRATCH__BIOS_SCRATCH__SHIFT                0x00000000

// BIOS_4_SCRATCH
#define BIOS_4_SCRATCH__BIOS_SCRATCH__SHIFT                0x00000000

// BIOS_5_SCRATCH
#define BIOS_5_SCRATCH__BIOS_SCRATCH__SHIFT                0x00000000

// BIOS_6_SCRATCH
#define BIOS_6_SCRATCH__BIOS_SCRATCH__SHIFT                0x00000000

// BIOS_7_SCRATCH
#define BIOS_7_SCRATCH__BIOS_SCRATCH__SHIFT                0x00000000

// BIOS_8_SCRATCH
#define BIOS_8_SCRATCH__BIOS_SCRATCH__SHIFT                0x00000000

// BIOS_9_SCRATCH
#define BIOS_9_SCRATCH__BIOS_SCRATCH__SHIFT                0x00000000

// BIOS_10_SCRATCH
#define BIOS_10_SCRATCH__BIOS_SCRATCH__SHIFT               0x00000000

// BIOS_11_SCRATCH
#define BIOS_11_SCRATCH__BIOS_SCRATCH__SHIFT               0x00000000

// BIOS_12_SCRATCH
#define BIOS_12_SCRATCH__BIOS_SCRATCH__SHIFT               0x00000000

// BIOS_13_SCRATCH
#define BIOS_13_SCRATCH__BIOS_SCRATCH__SHIFT               0x00000000

// BIOS_14_SCRATCH
#define BIOS_14_SCRATCH__BIOS_SCRATCH__SHIFT               0x00000000

// BIOS_15_SCRATCH
#define BIOS_15_SCRATCH__BIOS_SCRATCH__SHIFT               0x00000000

// COHER_SIZE_PM4
#define COHER_SIZE_PM4__SIZE__SHIFT                        0x00000000

// COHER_BASE_PM4
#define COHER_BASE_PM4__BASE__SHIFT                        0x00000000

// COHER_STATUS_PM4
#define COHER_STATUS_PM4__MATCHING_CONTEXTS__SHIFT         0x00000000
#define COHER_STATUS_PM4__RB_COPY_DEST_BASE_ENA__SHIFT     0x00000008
#define COHER_STATUS_PM4__DEST_BASE_0_ENA__SHIFT           0x00000009
#define COHER_STATUS_PM4__DEST_BASE_1_ENA__SHIFT           0x0000000a
#define COHER_STATUS_PM4__DEST_BASE_2_ENA__SHIFT           0x0000000b
#define COHER_STATUS_PM4__DEST_BASE_3_ENA__SHIFT           0x0000000c
#define COHER_STATUS_PM4__DEST_BASE_4_ENA__SHIFT           0x0000000d
#define COHER_STATUS_PM4__DEST_BASE_5_ENA__SHIFT           0x0000000e
#define COHER_STATUS_PM4__DEST_BASE_6_ENA__SHIFT           0x0000000f
#define COHER_STATUS_PM4__DEST_BASE_7_ENA__SHIFT           0x00000010
#define COHER_STATUS_PM4__RB_COLOR_INFO_ENA__SHIFT         0x00000011
#define COHER_STATUS_PM4__TC_ACTION_ENA__SHIFT             0x00000019
#define COHER_STATUS_PM4__STATUS__SHIFT                    0x0000001f

// COHER_SIZE_HOST
#define COHER_SIZE_HOST__SIZE__SHIFT                       0x00000000

// COHER_BASE_HOST
#define COHER_BASE_HOST__BASE__SHIFT                       0x00000000

// COHER_STATUS_HOST
#define COHER_STATUS_HOST__MATCHING_CONTEXTS__SHIFT        0x00000000
#define COHER_STATUS_HOST__RB_COPY_DEST_BASE_ENA__SHIFT    0x00000008
#define COHER_STATUS_HOST__DEST_BASE_0_ENA__SHIFT          0x00000009
#define COHER_STATUS_HOST__DEST_BASE_1_ENA__SHIFT          0x0000000a
#define COHER_STATUS_HOST__DEST_BASE_2_ENA__SHIFT          0x0000000b
#define COHER_STATUS_HOST__DEST_BASE_3_ENA__SHIFT          0x0000000c
#define COHER_STATUS_HOST__DEST_BASE_4_ENA__SHIFT          0x0000000d
#define COHER_STATUS_HOST__DEST_BASE_5_ENA__SHIFT          0x0000000e
#define COHER_STATUS_HOST__DEST_BASE_6_ENA__SHIFT          0x0000000f
#define COHER_STATUS_HOST__DEST_BASE_7_ENA__SHIFT          0x00000010
#define COHER_STATUS_HOST__RB_COLOR_INFO_ENA__SHIFT        0x00000011
#define COHER_STATUS_HOST__TC_ACTION_ENA__SHIFT            0x00000019
#define COHER_STATUS_HOST__STATUS__SHIFT                   0x0000001f

// COHER_DEST_BASE_0
#define COHER_DEST_BASE_0__DEST_BASE_0__SHIFT              0x0000000c

// COHER_DEST_BASE_1
#define COHER_DEST_BASE_1__DEST_BASE_1__SHIFT              0x0000000c

// COHER_DEST_BASE_2
#define COHER_DEST_BASE_2__DEST_BASE_2__SHIFT              0x0000000c

// COHER_DEST_BASE_3
#define COHER_DEST_BASE_3__DEST_BASE_3__SHIFT              0x0000000c

// COHER_DEST_BASE_4
#define COHER_DEST_BASE_4__DEST_BASE_4__SHIFT              0x0000000c

// COHER_DEST_BASE_5
#define COHER_DEST_BASE_5__DEST_BASE_5__SHIFT              0x0000000c

// COHER_DEST_BASE_6
#define COHER_DEST_BASE_6__DEST_BASE_6__SHIFT              0x0000000c

// COHER_DEST_BASE_7
#define COHER_DEST_BASE_7__DEST_BASE_7__SHIFT              0x0000000c

// RB_SURFACE_INFO
#define RB_SURFACE_INFO__SURFACE_PITCH__SHIFT              0x00000000
#define RB_SURFACE_INFO__MSAA_SAMPLES__SHIFT               0x0000000e

// RB_COLOR_INFO
#define RB_COLOR_INFO__COLOR_FORMAT__SHIFT                 0x00000000
#define RB_COLOR_INFO__COLOR_ROUND_MODE__SHIFT             0x00000004
#define RB_COLOR_INFO__COLOR_LINEAR__SHIFT                 0x00000006
#define RB_COLOR_INFO__COLOR_ENDIAN__SHIFT                 0x00000007
#define RB_COLOR_INFO__COLOR_SWAP__SHIFT                   0x00000009
#define RB_COLOR_INFO__COLOR_BASE__SHIFT                   0x0000000c

// RB_DEPTH_INFO
#define RB_DEPTH_INFO__DEPTH_FORMAT__SHIFT                 0x00000000
#define RB_DEPTH_INFO__DEPTH_BASE__SHIFT                   0x0000000c

// RB_STENCILREFMASK
#define RB_STENCILREFMASK__STENCILREF__SHIFT               0x00000000
#define RB_STENCILREFMASK__STENCILMASK__SHIFT              0x00000008
#define RB_STENCILREFMASK__STENCILWRITEMASK__SHIFT         0x00000010
#define RB_STENCILREFMASK__RESERVED0__SHIFT                0x00000018
#define RB_STENCILREFMASK__RESERVED1__SHIFT                0x00000019

// RB_ALPHA_REF
#define RB_ALPHA_REF__ALPHA_REF__SHIFT                     0x00000000

// RB_COLOR_MASK
#define RB_COLOR_MASK__WRITE_RED__SHIFT                    0x00000000
#define RB_COLOR_MASK__WRITE_GREEN__SHIFT                  0x00000001
#define RB_COLOR_MASK__WRITE_BLUE__SHIFT                   0x00000002
#define RB_COLOR_MASK__WRITE_ALPHA__SHIFT                  0x00000003
#define RB_COLOR_MASK__RESERVED2__SHIFT                    0x00000004
#define RB_COLOR_MASK__RESERVED3__SHIFT                    0x00000005

// RB_BLEND_RED
#define RB_BLEND_RED__BLEND_RED__SHIFT                     0x00000000

// RB_BLEND_GREEN
#define RB_BLEND_GREEN__BLEND_GREEN__SHIFT                 0x00000000

// RB_BLEND_BLUE
#define RB_BLEND_BLUE__BLEND_BLUE__SHIFT                   0x00000000

// RB_BLEND_ALPHA
#define RB_BLEND_ALPHA__BLEND_ALPHA__SHIFT                 0x00000000

// RB_FOG_COLOR
#define RB_FOG_COLOR__FOG_RED__SHIFT                       0x00000000
#define RB_FOG_COLOR__FOG_GREEN__SHIFT                     0x00000008
#define RB_FOG_COLOR__FOG_BLUE__SHIFT                      0x00000010

// RB_STENCILREFMASK_BF
#define RB_STENCILREFMASK_BF__STENCILREF_BF__SHIFT         0x00000000
#define RB_STENCILREFMASK_BF__STENCILMASK_BF__SHIFT        0x00000008
#define RB_STENCILREFMASK_BF__STENCILWRITEMASK_BF__SHIFT   0x00000010
#define RB_STENCILREFMASK_BF__RESERVED4__SHIFT             0x00000018
#define RB_STENCILREFMASK_BF__RESERVED5__SHIFT             0x00000019

// RB_DEPTHCONTROL
#define RB_DEPTHCONTROL__STENCIL_ENABLE__SHIFT             0x00000000
#define RB_DEPTHCONTROL__Z_ENABLE__SHIFT                   0x00000001
#define RB_DEPTHCONTROL__Z_WRITE_ENABLE__SHIFT             0x00000002
#define RB_DEPTHCONTROL__EARLY_Z_ENABLE__SHIFT             0x00000003
#define RB_DEPTHCONTROL__ZFUNC__SHIFT                      0x00000004
#define RB_DEPTHCONTROL__BACKFACE_ENABLE__SHIFT            0x00000007
#define RB_DEPTHCONTROL__STENCILFUNC__SHIFT                0x00000008
#define RB_DEPTHCONTROL__STENCILFAIL__SHIFT                0x0000000b
#define RB_DEPTHCONTROL__STENCILZPASS__SHIFT               0x0000000e
#define RB_DEPTHCONTROL__STENCILZFAIL__SHIFT               0x00000011
#define RB_DEPTHCONTROL__STENCILFUNC_BF__SHIFT             0x00000014
#define RB_DEPTHCONTROL__STENCILFAIL_BF__SHIFT             0x00000017
#define RB_DEPTHCONTROL__STENCILZPASS_BF__SHIFT            0x0000001a
#define RB_DEPTHCONTROL__STENCILZFAIL_BF__SHIFT            0x0000001d

// RB_BLENDCONTROL
#define RB_BLENDCONTROL__COLOR_SRCBLEND__SHIFT             0x00000000
#define RB_BLENDCONTROL__COLOR_COMB_FCN__SHIFT             0x00000005
#define RB_BLENDCONTROL__COLOR_DESTBLEND__SHIFT            0x00000008
#define RB_BLENDCONTROL__ALPHA_SRCBLEND__SHIFT             0x00000010
#define RB_BLENDCONTROL__ALPHA_COMB_FCN__SHIFT             0x00000015
#define RB_BLENDCONTROL__ALPHA_DESTBLEND__SHIFT            0x00000018
#define RB_BLENDCONTROL__BLEND_FORCE_ENABLE__SHIFT         0x0000001d
#define RB_BLENDCONTROL__BLEND_FORCE__SHIFT                0x0000001e

// RB_COLORCONTROL
#define RB_COLORCONTROL__ALPHA_FUNC__SHIFT                 0x00000000
#define RB_COLORCONTROL__ALPHA_TEST_ENABLE__SHIFT          0x00000003
#define RB_COLORCONTROL__ALPHA_TO_MASK_ENABLE__SHIFT       0x00000004
#define RB_COLORCONTROL__BLEND_DISABLE__SHIFT              0x00000005
#define RB_COLORCONTROL__FOG_ENABLE__SHIFT                 0x00000006
#define RB_COLORCONTROL__VS_EXPORTS_FOG__SHIFT             0x00000007
#define RB_COLORCONTROL__ROP_CODE__SHIFT                   0x00000008
#define RB_COLORCONTROL__DITHER_MODE__SHIFT                0x0000000c
#define RB_COLORCONTROL__DITHER_TYPE__SHIFT                0x0000000e
#define RB_COLORCONTROL__PIXEL_FOG__SHIFT                  0x00000010
#define RB_COLORCONTROL__ALPHA_TO_MASK_OFFSET0__SHIFT      0x00000018
#define RB_COLORCONTROL__ALPHA_TO_MASK_OFFSET1__SHIFT      0x0000001a
#define RB_COLORCONTROL__ALPHA_TO_MASK_OFFSET2__SHIFT      0x0000001c
#define RB_COLORCONTROL__ALPHA_TO_MASK_OFFSET3__SHIFT      0x0000001e

// RB_MODECONTROL
#define RB_MODECONTROL__EDRAM_MODE__SHIFT                  0x00000000

// RB_COLOR_DEST_MASK
#define RB_COLOR_DEST_MASK__COLOR_DEST_MASK__SHIFT         0x00000000

// RB_COPY_CONTROL
#define RB_COPY_CONTROL__COPY_SAMPLE_SELECT__SHIFT         0x00000000
#define RB_COPY_CONTROL__DEPTH_CLEAR_ENABLE__SHIFT         0x00000003
#define RB_COPY_CONTROL__CLEAR_MASK__SHIFT                 0x00000004

// RB_COPY_DEST_BASE
#define RB_COPY_DEST_BASE__COPY_DEST_BASE__SHIFT           0x0000000c

// RB_COPY_DEST_PITCH
#define RB_COPY_DEST_PITCH__COPY_DEST_PITCH__SHIFT         0x00000000

// RB_COPY_DEST_INFO
#define RB_COPY_DEST_INFO__COPY_DEST_ENDIAN__SHIFT         0x00000000
#define RB_COPY_DEST_INFO__COPY_DEST_LINEAR__SHIFT         0x00000003
#define RB_COPY_DEST_INFO__COPY_DEST_FORMAT__SHIFT         0x00000004
#define RB_COPY_DEST_INFO__COPY_DEST_SWAP__SHIFT           0x00000008
#define RB_COPY_DEST_INFO__COPY_DEST_DITHER_MODE__SHIFT    0x0000000a
#define RB_COPY_DEST_INFO__COPY_DEST_DITHER_TYPE__SHIFT    0x0000000c
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_RED__SHIFT      0x0000000e
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_GREEN__SHIFT    0x0000000f
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_BLUE__SHIFT     0x00000010
#define RB_COPY_DEST_INFO__COPY_MASK_WRITE_ALPHA__SHIFT    0x00000011

// RB_COPY_DEST_PIXEL_OFFSET
#define RB_COPY_DEST_PIXEL_OFFSET__OFFSET_X__SHIFT         0x00000000
#define RB_COPY_DEST_PIXEL_OFFSET__OFFSET_Y__SHIFT         0x0000000d

// RB_DEPTH_CLEAR
#define RB_DEPTH_CLEAR__DEPTH_CLEAR__SHIFT                 0x00000000

// RB_SAMPLE_COUNT_CTL
#define RB_SAMPLE_COUNT_CTL__RESET_SAMPLE_COUNT__SHIFT     0x00000000
#define RB_SAMPLE_COUNT_CTL__COPY_SAMPLE_COUNT__SHIFT      0x00000001

// RB_SAMPLE_COUNT_ADDR
#define RB_SAMPLE_COUNT_ADDR__SAMPLE_COUNT_ADDR__SHIFT     0x00000000

// RB_BC_CONTROL
#define RB_BC_CONTROL__ACCUM_LINEAR_MODE_ENABLE__SHIFT     0x00000000
#define RB_BC_CONTROL__ACCUM_TIMEOUT_SELECT__SHIFT         0x00000001
#define RB_BC_CONTROL__DISABLE_EDRAM_CAM__SHIFT            0x00000003
#define RB_BC_CONTROL__DISABLE_EZ_FAST_CONTEXT_SWITCH__SHIFT 0x00000004
#define RB_BC_CONTROL__DISABLE_EZ_NULL_ZCMD_DROP__SHIFT    0x00000005
#define RB_BC_CONTROL__DISABLE_LZ_NULL_ZCMD_DROP__SHIFT    0x00000006
#define RB_BC_CONTROL__ENABLE_AZ_THROTTLE__SHIFT           0x00000007
#define RB_BC_CONTROL__AZ_THROTTLE_COUNT__SHIFT            0x00000008
#define RB_BC_CONTROL__ENABLE_CRC_UPDATE__SHIFT            0x0000000e
#define RB_BC_CONTROL__CRC_MODE__SHIFT                     0x0000000f
#define RB_BC_CONTROL__DISABLE_SAMPLE_COUNTERS__SHIFT      0x00000010
#define RB_BC_CONTROL__DISABLE_ACCUM__SHIFT                0x00000011
#define RB_BC_CONTROL__ACCUM_ALLOC_MASK__SHIFT             0x00000012
#define RB_BC_CONTROL__LINEAR_PERFORMANCE_ENABLE__SHIFT    0x00000016
#define RB_BC_CONTROL__ACCUM_DATA_FIFO_LIMIT__SHIFT        0x00000017
#define RB_BC_CONTROL__MEM_EXPORT_TIMEOUT_SELECT__SHIFT    0x0000001b
#define RB_BC_CONTROL__MEM_EXPORT_LINEAR_MODE_ENABLE__SHIFT 0x0000001d
#define RB_BC_CONTROL__CRC_SYSTEM__SHIFT                   0x0000001e
#define RB_BC_CONTROL__RESERVED6__SHIFT                    0x0000001f

// RB_EDRAM_INFO
#define RB_EDRAM_INFO__EDRAM_SIZE__SHIFT                   0x00000000
#define RB_EDRAM_INFO__EDRAM_MAPPING_MODE__SHIFT           0x00000004
#define RB_EDRAM_INFO__EDRAM_RANGE__SHIFT                  0x0000000e

// RB_CRC_RD_PORT
#define RB_CRC_RD_PORT__CRC_DATA__SHIFT                    0x00000000

// RB_CRC_CONTROL
#define RB_CRC_CONTROL__CRC_RD_ADVANCE__SHIFT              0x00000000

// RB_CRC_MASK
#define RB_CRC_MASK__CRC_MASK__SHIFT                       0x00000000

// RB_PERFCOUNTER0_SELECT
#define RB_PERFCOUNTER0_SELECT__PERF_SEL__SHIFT            0x00000000

// RB_PERFCOUNTER0_LOW
#define RB_PERFCOUNTER0_LOW__PERF_COUNT__SHIFT             0x00000000

// RB_PERFCOUNTER0_HI
#define RB_PERFCOUNTER0_HI__PERF_COUNT__SHIFT              0x00000000

// RB_TOTAL_SAMPLES
#define RB_TOTAL_SAMPLES__TOTAL_SAMPLES__SHIFT             0x00000000

// RB_ZPASS_SAMPLES
#define RB_ZPASS_SAMPLES__ZPASS_SAMPLES__SHIFT             0x00000000

// RB_ZFAIL_SAMPLES
#define RB_ZFAIL_SAMPLES__ZFAIL_SAMPLES__SHIFT             0x00000000

// RB_SFAIL_SAMPLES
#define RB_SFAIL_SAMPLES__SFAIL_SAMPLES__SHIFT             0x00000000

// RB_DEBUG_0
#define RB_DEBUG_0__RDREQ_CTL_Z1_PRE_FULL__SHIFT           0x00000000
#define RB_DEBUG_0__RDREQ_CTL_Z0_PRE_FULL__SHIFT           0x00000001
#define RB_DEBUG_0__RDREQ_CTL_C1_PRE_FULL__SHIFT           0x00000002
#define RB_DEBUG_0__RDREQ_CTL_C0_PRE_FULL__SHIFT           0x00000003
#define RB_DEBUG_0__RDREQ_E1_ORDERING_FULL__SHIFT          0x00000004
#define RB_DEBUG_0__RDREQ_E0_ORDERING_FULL__SHIFT          0x00000005
#define RB_DEBUG_0__RDREQ_Z1_FULL__SHIFT                   0x00000006
#define RB_DEBUG_0__RDREQ_Z0_FULL__SHIFT                   0x00000007
#define RB_DEBUG_0__RDREQ_C1_FULL__SHIFT                   0x00000008
#define RB_DEBUG_0__RDREQ_C0_FULL__SHIFT                   0x00000009
#define RB_DEBUG_0__WRREQ_E1_MACRO_HI_FULL__SHIFT          0x0000000a
#define RB_DEBUG_0__WRREQ_E1_MACRO_LO_FULL__SHIFT          0x0000000b
#define RB_DEBUG_0__WRREQ_E0_MACRO_HI_FULL__SHIFT          0x0000000c
#define RB_DEBUG_0__WRREQ_E0_MACRO_LO_FULL__SHIFT          0x0000000d
#define RB_DEBUG_0__WRREQ_C_WE_HI_FULL__SHIFT              0x0000000e
#define RB_DEBUG_0__WRREQ_C_WE_LO_FULL__SHIFT              0x0000000f
#define RB_DEBUG_0__WRREQ_Z1_FULL__SHIFT                   0x00000010
#define RB_DEBUG_0__WRREQ_Z0_FULL__SHIFT                   0x00000011
#define RB_DEBUG_0__WRREQ_C1_FULL__SHIFT                   0x00000012
#define RB_DEBUG_0__WRREQ_C0_FULL__SHIFT                   0x00000013
#define RB_DEBUG_0__CMDFIFO_Z1_HOLD_FULL__SHIFT            0x00000014
#define RB_DEBUG_0__CMDFIFO_Z0_HOLD_FULL__SHIFT            0x00000015
#define RB_DEBUG_0__CMDFIFO_C1_HOLD_FULL__SHIFT            0x00000016
#define RB_DEBUG_0__CMDFIFO_C0_HOLD_FULL__SHIFT            0x00000017
#define RB_DEBUG_0__CMDFIFO_Z_ORDERING_FULL__SHIFT         0x00000018
#define RB_DEBUG_0__CMDFIFO_C_ORDERING_FULL__SHIFT         0x00000019
#define RB_DEBUG_0__C_SX_LAT_FULL__SHIFT                   0x0000001a
#define RB_DEBUG_0__C_SX_CMD_FULL__SHIFT                   0x0000001b
#define RB_DEBUG_0__C_EZ_TILE_FULL__SHIFT                  0x0000001c
#define RB_DEBUG_0__C_REQ_FULL__SHIFT                      0x0000001d
#define RB_DEBUG_0__C_MASK_FULL__SHIFT                     0x0000001e
#define RB_DEBUG_0__EZ_INFSAMP_FULL__SHIFT                 0x0000001f

// RB_DEBUG_1
#define RB_DEBUG_1__RDREQ_Z1_CMD_EMPTY__SHIFT              0x00000000
#define RB_DEBUG_1__RDREQ_Z0_CMD_EMPTY__SHIFT              0x00000001
#define RB_DEBUG_1__RDREQ_C1_CMD_EMPTY__SHIFT              0x00000002
#define RB_DEBUG_1__RDREQ_C0_CMD_EMPTY__SHIFT              0x00000003
#define RB_DEBUG_1__RDREQ_E1_ORDERING_EMPTY__SHIFT         0x00000004
#define RB_DEBUG_1__RDREQ_E0_ORDERING_EMPTY__SHIFT         0x00000005
#define RB_DEBUG_1__RDREQ_Z1_EMPTY__SHIFT                  0x00000006
#define RB_DEBUG_1__RDREQ_Z0_EMPTY__SHIFT                  0x00000007
#define RB_DEBUG_1__RDREQ_C1_EMPTY__SHIFT                  0x00000008
#define RB_DEBUG_1__RDREQ_C0_EMPTY__SHIFT                  0x00000009
#define RB_DEBUG_1__WRREQ_E1_MACRO_HI_EMPTY__SHIFT         0x0000000a
#define RB_DEBUG_1__WRREQ_E1_MACRO_LO_EMPTY__SHIFT         0x0000000b
#define RB_DEBUG_1__WRREQ_E0_MACRO_HI_EMPTY__SHIFT         0x0000000c
#define RB_DEBUG_1__WRREQ_E0_MACRO_LO_EMPTY__SHIFT         0x0000000d
#define RB_DEBUG_1__WRREQ_C_WE_HI_EMPTY__SHIFT             0x0000000e
#define RB_DEBUG_1__WRREQ_C_WE_LO_EMPTY__SHIFT             0x0000000f
#define RB_DEBUG_1__WRREQ_Z1_EMPTY__SHIFT                  0x00000010
#define RB_DEBUG_1__WRREQ_Z0_EMPTY__SHIFT                  0x00000011
#define RB_DEBUG_1__WRREQ_C1_PRE_EMPTY__SHIFT              0x00000012
#define RB_DEBUG_1__WRREQ_C0_PRE_EMPTY__SHIFT              0x00000013
#define RB_DEBUG_1__CMDFIFO_Z1_HOLD_EMPTY__SHIFT           0x00000014
#define RB_DEBUG_1__CMDFIFO_Z0_HOLD_EMPTY__SHIFT           0x00000015
#define RB_DEBUG_1__CMDFIFO_C1_HOLD_EMPTY__SHIFT           0x00000016
#define RB_DEBUG_1__CMDFIFO_C0_HOLD_EMPTY__SHIFT           0x00000017
#define RB_DEBUG_1__CMDFIFO_Z_ORDERING_EMPTY__SHIFT        0x00000018
#define RB_DEBUG_1__CMDFIFO_C_ORDERING_EMPTY__SHIFT        0x00000019
#define RB_DEBUG_1__C_SX_LAT_EMPTY__SHIFT                  0x0000001a
#define RB_DEBUG_1__C_SX_CMD_EMPTY__SHIFT                  0x0000001b
#define RB_DEBUG_1__C_EZ_TILE_EMPTY__SHIFT                 0x0000001c
#define RB_DEBUG_1__C_REQ_EMPTY__SHIFT                     0x0000001d
#define RB_DEBUG_1__C_MASK_EMPTY__SHIFT                    0x0000001e
#define RB_DEBUG_1__EZ_INFSAMP_EMPTY__SHIFT                0x0000001f

// RB_DEBUG_2
#define RB_DEBUG_2__TILE_FIFO_COUNT__SHIFT                 0x00000000
#define RB_DEBUG_2__SX_LAT_FIFO_COUNT__SHIFT               0x00000004
#define RB_DEBUG_2__MEM_EXPORT_FLAG__SHIFT                 0x0000000b
#define RB_DEBUG_2__SYSMEM_BLEND_FLAG__SHIFT               0x0000000c
#define RB_DEBUG_2__CURRENT_TILE_EVENT__SHIFT              0x0000000d
#define RB_DEBUG_2__EZ_INFTILE_FULL__SHIFT                 0x0000000e
#define RB_DEBUG_2__EZ_MASK_LOWER_FULL__SHIFT              0x0000000f
#define RB_DEBUG_2__EZ_MASK_UPPER_FULL__SHIFT              0x00000010
#define RB_DEBUG_2__Z0_MASK_FULL__SHIFT                    0x00000011
#define RB_DEBUG_2__Z1_MASK_FULL__SHIFT                    0x00000012
#define RB_DEBUG_2__Z0_REQ_FULL__SHIFT                     0x00000013
#define RB_DEBUG_2__Z1_REQ_FULL__SHIFT                     0x00000014
#define RB_DEBUG_2__Z_SAMP_FULL__SHIFT                     0x00000015
#define RB_DEBUG_2__Z_TILE_FULL__SHIFT                     0x00000016
#define RB_DEBUG_2__EZ_INFTILE_EMPTY__SHIFT                0x00000017
#define RB_DEBUG_2__EZ_MASK_LOWER_EMPTY__SHIFT             0x00000018
#define RB_DEBUG_2__EZ_MASK_UPPER_EMPTY__SHIFT             0x00000019
#define RB_DEBUG_2__Z0_MASK_EMPTY__SHIFT                   0x0000001a
#define RB_DEBUG_2__Z1_MASK_EMPTY__SHIFT                   0x0000001b
#define RB_DEBUG_2__Z0_REQ_EMPTY__SHIFT                    0x0000001c
#define RB_DEBUG_2__Z1_REQ_EMPTY__SHIFT                    0x0000001d
#define RB_DEBUG_2__Z_SAMP_EMPTY__SHIFT                    0x0000001e
#define RB_DEBUG_2__Z_TILE_EMPTY__SHIFT                    0x0000001f

// RB_DEBUG_3
#define RB_DEBUG_3__ACCUM_VALID__SHIFT                     0x00000000
#define RB_DEBUG_3__ACCUM_FLUSHING__SHIFT                  0x00000004
#define RB_DEBUG_3__ACCUM_WRITE_CLEAN_COUNT__SHIFT         0x00000008
#define RB_DEBUG_3__ACCUM_INPUT_REG_VALID__SHIFT           0x0000000e
#define RB_DEBUG_3__ACCUM_DATA_FIFO_CNT__SHIFT             0x0000000f
#define RB_DEBUG_3__SHD_FULL__SHIFT                        0x00000013
#define RB_DEBUG_3__SHD_EMPTY__SHIFT                       0x00000014
#define RB_DEBUG_3__EZ_RETURN_LOWER_EMPTY__SHIFT           0x00000015
#define RB_DEBUG_3__EZ_RETURN_UPPER_EMPTY__SHIFT           0x00000016
#define RB_DEBUG_3__EZ_RETURN_LOWER_FULL__SHIFT            0x00000017
#define RB_DEBUG_3__EZ_RETURN_UPPER_FULL__SHIFT            0x00000018
#define RB_DEBUG_3__ZEXP_LOWER_EMPTY__SHIFT                0x00000019
#define RB_DEBUG_3__ZEXP_UPPER_EMPTY__SHIFT                0x0000001a
#define RB_DEBUG_3__ZEXP_LOWER_FULL__SHIFT                 0x0000001b
#define RB_DEBUG_3__ZEXP_UPPER_FULL__SHIFT                 0x0000001c

// RB_DEBUG_4
#define RB_DEBUG_4__GMEM_RD_ACCESS_FLAG__SHIFT             0x00000000
#define RB_DEBUG_4__GMEM_WR_ACCESS_FLAG__SHIFT             0x00000001
#define RB_DEBUG_4__SYSMEM_RD_ACCESS_FLAG__SHIFT           0x00000002
#define RB_DEBUG_4__SYSMEM_WR_ACCESS_FLAG__SHIFT           0x00000003
#define RB_DEBUG_4__ACCUM_DATA_FIFO_EMPTY__SHIFT           0x00000004
#define RB_DEBUG_4__ACCUM_ORDER_FIFO_EMPTY__SHIFT          0x00000005
#define RB_DEBUG_4__ACCUM_DATA_FIFO_FULL__SHIFT            0x00000006
#define RB_DEBUG_4__ACCUM_ORDER_FIFO_FULL__SHIFT           0x00000007
#define RB_DEBUG_4__SYSMEM_WRITE_COUNT_OVERFLOW__SHIFT     0x00000008
#define RB_DEBUG_4__CONTEXT_COUNT_DEBUG__SHIFT             0x00000009

// RB_FLAG_CONTROL
#define RB_FLAG_CONTROL__DEBUG_FLAG_CLEAR__SHIFT           0x00000000

// RB_BC_SPARES
#define RB_BC_SPARES__RESERVED__SHIFT                      0x00000000

// BC_DUMMY_CRAYRB_ENUMS
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_DEPTH_FORMAT__SHIFT 0x00000000
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_SWAP__SHIFT 0x00000006
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_DEPTH_ARRAY__SHIFT 0x00000007
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_ARRAY__SHIFT   0x00000009
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_COLOR_FORMAT__SHIFT 0x0000000b
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_NUMBER__SHIFT 0x00000011
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_FORMAT__SHIFT 0x00000014
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_TILING__SHIFT 0x0000001a
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_CRAYRB_SURFACE_ARRAY__SHIFT 0x0000001b
#define BC_DUMMY_CRAYRB_ENUMS__DUMMY_RB_COPY_DEST_INFO_NUMBER__SHIFT 0x0000001d

// BC_DUMMY_CRAYRB_MOREENUMS
#define BC_DUMMY_CRAYRB_MOREENUMS__DUMMY_CRAYRB_COLORARRAYX__SHIFT 0x00000000

#endif
