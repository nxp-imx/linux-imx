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

#if !defined (_yamato_REG_HEADER)
#define _yamato_REG_HEADER

    union PA_CL_VPORT_XSCALE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    VPORT_XSCALE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    VPORT_XSCALE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_VPORT_XOFFSET {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   VPORT_XOFFSET : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   VPORT_XOFFSET : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_VPORT_YSCALE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    VPORT_YSCALE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    VPORT_YSCALE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_VPORT_YOFFSET {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   VPORT_YOFFSET : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   VPORT_YOFFSET : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_VPORT_ZSCALE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    VPORT_ZSCALE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    VPORT_ZSCALE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_VPORT_ZOFFSET {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   VPORT_ZOFFSET : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   VPORT_ZOFFSET : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_VTE_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int               VPORT_X_SCALE_ENA : 1;
        unsigned int              VPORT_X_OFFSET_ENA : 1;
        unsigned int               VPORT_Y_SCALE_ENA : 1;
        unsigned int              VPORT_Y_OFFSET_ENA : 1;
        unsigned int               VPORT_Z_SCALE_ENA : 1;
        unsigned int              VPORT_Z_OFFSET_ENA : 1;
        unsigned int                                 : 2;
        unsigned int                      VTX_XY_FMT : 1;
        unsigned int                       VTX_Z_FMT : 1;
        unsigned int                      VTX_W0_FMT : 1;
        unsigned int                 PERFCOUNTER_REF : 1;
        unsigned int                                 : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 20;
        unsigned int                 PERFCOUNTER_REF : 1;
        unsigned int                      VTX_W0_FMT : 1;
        unsigned int                       VTX_Z_FMT : 1;
        unsigned int                      VTX_XY_FMT : 1;
        unsigned int                                 : 2;
        unsigned int              VPORT_Z_OFFSET_ENA : 1;
        unsigned int               VPORT_Z_SCALE_ENA : 1;
        unsigned int              VPORT_Y_OFFSET_ENA : 1;
        unsigned int               VPORT_Y_SCALE_ENA : 1;
        unsigned int              VPORT_X_OFFSET_ENA : 1;
        unsigned int               VPORT_X_SCALE_ENA : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_CLIP_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 16;
        unsigned int                    CLIP_DISABLE : 1;
        unsigned int                                 : 1;
        unsigned int          BOUNDARY_EDGE_FLAG_ENA : 1;
        unsigned int               DX_CLIP_SPACE_DEF : 1;
        unsigned int             DIS_CLIP_ERR_DETECT : 1;
        unsigned int                     VTX_KILL_OR : 1;
        unsigned int                   XY_NAN_RETAIN : 1;
        unsigned int                    Z_NAN_RETAIN : 1;
        unsigned int                    W_NAN_RETAIN : 1;
        unsigned int                                 : 7;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 7;
        unsigned int                    W_NAN_RETAIN : 1;
        unsigned int                    Z_NAN_RETAIN : 1;
        unsigned int                   XY_NAN_RETAIN : 1;
        unsigned int                     VTX_KILL_OR : 1;
        unsigned int             DIS_CLIP_ERR_DETECT : 1;
        unsigned int               DX_CLIP_SPACE_DEF : 1;
        unsigned int          BOUNDARY_EDGE_FLAG_ENA : 1;
        unsigned int                                 : 1;
        unsigned int                    CLIP_DISABLE : 1;
        unsigned int                                 : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_GB_VERT_CLIP_ADJ {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   DATA_REGISTER : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   DATA_REGISTER : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_GB_VERT_DISC_ADJ {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   DATA_REGISTER : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   DATA_REGISTER : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_GB_HORZ_CLIP_ADJ {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   DATA_REGISTER : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   DATA_REGISTER : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_GB_HORZ_DISC_ADJ {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   DATA_REGISTER : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   DATA_REGISTER : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_ENHANCE {
    struct {
#if     defined(qLittleEndian)
        unsigned int            CLIP_VTX_REORDER_ENA : 1;
        unsigned int                                 : 27;
        unsigned int                      ECO_SPARE3 : 1;
        unsigned int                      ECO_SPARE2 : 1;
        unsigned int                      ECO_SPARE1 : 1;
        unsigned int                      ECO_SPARE0 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                      ECO_SPARE0 : 1;
        unsigned int                      ECO_SPARE1 : 1;
        unsigned int                      ECO_SPARE2 : 1;
        unsigned int                      ECO_SPARE3 : 1;
        unsigned int                                 : 27;
        unsigned int            CLIP_VTX_REORDER_ENA : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_ENHANCE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 28;
        unsigned int                      ECO_SPARE3 : 1;
        unsigned int                      ECO_SPARE2 : 1;
        unsigned int                      ECO_SPARE1 : 1;
        unsigned int                      ECO_SPARE0 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                      ECO_SPARE0 : 1;
        unsigned int                      ECO_SPARE1 : 1;
        unsigned int                      ECO_SPARE2 : 1;
        unsigned int                      ECO_SPARE3 : 1;
        unsigned int                                 : 28;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_VTX_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PIX_CENTER : 1;
        unsigned int                      ROUND_MODE : 2;
        unsigned int                      QUANT_MODE : 3;
        unsigned int                                 : 26;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 26;
        unsigned int                      QUANT_MODE : 3;
        unsigned int                      ROUND_MODE : 2;
        unsigned int                      PIX_CENTER : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_POINT_SIZE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          HEIGHT : 16;
        unsigned int                           WIDTH : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                           WIDTH : 16;
        unsigned int                          HEIGHT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_POINT_MINMAX {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        MIN_SIZE : 16;
        unsigned int                        MAX_SIZE : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                        MAX_SIZE : 16;
        unsigned int                        MIN_SIZE : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_LINE_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           WIDTH : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                           WIDTH : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_FACE_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 5;
        unsigned int                       BASE_ADDR : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                       BASE_ADDR : 27;
        unsigned int                                 : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_SC_MODE_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      CULL_FRONT : 1;
        unsigned int                       CULL_BACK : 1;
        unsigned int                            FACE : 1;
        unsigned int                       POLY_MODE : 2;
        unsigned int            POLYMODE_FRONT_PTYPE : 3;
        unsigned int             POLYMODE_BACK_PTYPE : 3;
        unsigned int        POLY_OFFSET_FRONT_ENABLE : 1;
        unsigned int         POLY_OFFSET_BACK_ENABLE : 1;
        unsigned int         POLY_OFFSET_PARA_ENABLE : 1;
        unsigned int                                 : 1;
        unsigned int                     MSAA_ENABLE : 1;
        unsigned int        VTX_WINDOW_OFFSET_ENABLE : 1;
        unsigned int                                 : 1;
        unsigned int             LINE_STIPPLE_ENABLE : 1;
        unsigned int              PROVOKING_VTX_LAST : 1;
        unsigned int                  PERSP_CORR_DIS : 1;
        unsigned int               MULTI_PRIM_IB_ENA : 1;
        unsigned int                                 : 1;
        unsigned int               QUAD_ORDER_ENABLE : 1;
        unsigned int                                 : 1;
        unsigned int            WAIT_RB_IDLE_ALL_TRI : 1;
        unsigned int WAIT_RB_IDLE_FIRST_TRI_NEW_STATE : 1;
        unsigned int                                 : 2;
        unsigned int              ZERO_AREA_FACENESS : 1;
        unsigned int                FACE_KILL_ENABLE : 1;
        unsigned int               FACE_WRITE_ENABLE : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int               FACE_WRITE_ENABLE : 1;
        unsigned int                FACE_KILL_ENABLE : 1;
        unsigned int              ZERO_AREA_FACENESS : 1;
        unsigned int                                 : 2;
        unsigned int WAIT_RB_IDLE_FIRST_TRI_NEW_STATE : 1;
        unsigned int            WAIT_RB_IDLE_ALL_TRI : 1;
        unsigned int                                 : 1;
        unsigned int               QUAD_ORDER_ENABLE : 1;
        unsigned int                                 : 1;
        unsigned int               MULTI_PRIM_IB_ENA : 1;
        unsigned int                  PERSP_CORR_DIS : 1;
        unsigned int              PROVOKING_VTX_LAST : 1;
        unsigned int             LINE_STIPPLE_ENABLE : 1;
        unsigned int                                 : 1;
        unsigned int        VTX_WINDOW_OFFSET_ENABLE : 1;
        unsigned int                     MSAA_ENABLE : 1;
        unsigned int                                 : 1;
        unsigned int         POLY_OFFSET_PARA_ENABLE : 1;
        unsigned int         POLY_OFFSET_BACK_ENABLE : 1;
        unsigned int        POLY_OFFSET_FRONT_ENABLE : 1;
        unsigned int             POLYMODE_BACK_PTYPE : 3;
        unsigned int            POLYMODE_FRONT_PTYPE : 3;
        unsigned int                       POLY_MODE : 2;
        unsigned int                            FACE : 1;
        unsigned int                       CULL_BACK : 1;
        unsigned int                      CULL_FRONT : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_POLY_OFFSET_FRONT_SCALE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           SCALE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           SCALE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_POLY_OFFSET_FRONT_OFFSET {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          OFFSET : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                          OFFSET : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_POLY_OFFSET_BACK_SCALE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           SCALE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           SCALE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_POLY_OFFSET_BACK_OFFSET {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          OFFSET : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                          OFFSET : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER0_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER1_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER2_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER3_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER0_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER0_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER1_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER1_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER2_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER2_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER3_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_PERFCOUNTER3_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_WINDOW_OFFSET {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 WINDOW_X_OFFSET : 15;
        unsigned int                                 : 1;
        unsigned int                 WINDOW_Y_OFFSET : 15;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                 WINDOW_Y_OFFSET : 15;
        unsigned int                                 : 1;
        unsigned int                 WINDOW_X_OFFSET : 15;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_AA_CONFIG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                MSAA_NUM_SAMPLES : 3;
        unsigned int                                 : 10;
        unsigned int                 MAX_SAMPLE_DIST : 4;
        unsigned int                                 : 15;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 15;
        unsigned int                 MAX_SAMPLE_DIST : 4;
        unsigned int                                 : 10;
        unsigned int                MSAA_NUM_SAMPLES : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_AA_MASK {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         AA_MASK : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                         AA_MASK : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_LINE_STIPPLE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    LINE_PATTERN : 16;
        unsigned int                    REPEAT_COUNT : 8;
        unsigned int                                 : 4;
        unsigned int               PATTERN_BIT_ORDER : 1;
        unsigned int                 AUTO_RESET_CNTL : 2;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                 AUTO_RESET_CNTL : 2;
        unsigned int               PATTERN_BIT_ORDER : 1;
        unsigned int                                 : 4;
        unsigned int                    REPEAT_COUNT : 8;
        unsigned int                    LINE_PATTERN : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_LINE_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       BRES_CNTL : 8;
        unsigned int                   USE_BRES_CNTL : 1;
        unsigned int               EXPAND_LINE_WIDTH : 1;
        unsigned int                      LAST_PIXEL : 1;
        unsigned int                                 : 21;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 21;
        unsigned int                      LAST_PIXEL : 1;
        unsigned int               EXPAND_LINE_WIDTH : 1;
        unsigned int                   USE_BRES_CNTL : 1;
        unsigned int                       BRES_CNTL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_WINDOW_SCISSOR_TL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            TL_X : 14;
        unsigned int                                 : 2;
        unsigned int                            TL_Y : 14;
        unsigned int                                 : 1;
        unsigned int           WINDOW_OFFSET_DISABLE : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int           WINDOW_OFFSET_DISABLE : 1;
        unsigned int                                 : 1;
        unsigned int                            TL_Y : 14;
        unsigned int                                 : 2;
        unsigned int                            TL_X : 14;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_WINDOW_SCISSOR_BR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            BR_X : 14;
        unsigned int                                 : 2;
        unsigned int                            BR_Y : 14;
        unsigned int                                 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 2;
        unsigned int                            BR_Y : 14;
        unsigned int                                 : 2;
        unsigned int                            BR_X : 14;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_SCREEN_SCISSOR_TL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            TL_X : 15;
        unsigned int                                 : 1;
        unsigned int                            TL_Y : 15;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                            TL_Y : 15;
        unsigned int                                 : 1;
        unsigned int                            TL_X : 15;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_SCREEN_SCISSOR_BR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            BR_X : 15;
        unsigned int                                 : 1;
        unsigned int                            BR_Y : 15;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                            BR_Y : 15;
        unsigned int                                 : 1;
        unsigned int                            BR_X : 15;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_VIZ_QUERY {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   VIZ_QUERY_ENA : 1;
        unsigned int                    VIZ_QUERY_ID : 5;
        unsigned int                                 : 1;
        unsigned int           KILL_PIX_POST_EARLY_Z : 1;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int           KILL_PIX_POST_EARLY_Z : 1;
        unsigned int                                 : 1;
        unsigned int                    VIZ_QUERY_ID : 5;
        unsigned int                   VIZ_QUERY_ENA : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_VIZ_QUERY_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     STATUS_BITS : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                     STATUS_BITS : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_LINE_STIPPLE_STATE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     CURRENT_PTR : 4;
        unsigned int                                 : 4;
        unsigned int                   CURRENT_COUNT : 8;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                   CURRENT_COUNT : 8;
        unsigned int                                 : 4;
        unsigned int                     CURRENT_PTR : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_PERFCOUNTER0_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_PERFCOUNTER0_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_PERFCOUNTER0_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_CL_CNTL_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 31;
        unsigned int                         CL_BUSY : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         CL_BUSY : 1;
        unsigned int                                 : 31;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_CNTL_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 31;
        unsigned int                         SU_BUSY : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         SU_BUSY : 1;
        unsigned int                                 : 31;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_CNTL_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 31;
        unsigned int                         SC_BUSY : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         SC_BUSY : 1;
        unsigned int                                 : 31;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_DEBUG_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   SU_DEBUG_INDX : 5;
        unsigned int                                 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 27;
        unsigned int                   SU_DEBUG_INDX : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SU_DEBUG_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                            DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CLIPPER_DEBUG_REG00 {
    struct {
#if     defined(qLittleEndian)
        unsigned int           clip_ga_bc_fifo_write : 1;
        unsigned int            clip_ga_bc_fifo_full : 1;
        unsigned int           clip_to_ga_fifo_write : 1;
        unsigned int            clip_to_ga_fifo_full : 1;
        unsigned int     primic_to_clprim_fifo_empty : 1;
        unsigned int      primic_to_clprim_fifo_full : 1;
        unsigned int        clip_to_outsm_fifo_empty : 1;
        unsigned int         clip_to_outsm_fifo_full : 1;
        unsigned int         vgt_to_clipp_fifo_empty : 1;
        unsigned int          vgt_to_clipp_fifo_full : 1;
        unsigned int         vgt_to_clips_fifo_empty : 1;
        unsigned int          vgt_to_clips_fifo_full : 1;
        unsigned int        clipcode_fifo_fifo_empty : 1;
        unsigned int              clipcode_fifo_full : 1;
        unsigned int    vte_out_clip_fifo_fifo_empty : 1;
        unsigned int     vte_out_clip_fifo_fifo_full : 1;
        unsigned int    vte_out_orig_fifo_fifo_empty : 1;
        unsigned int     vte_out_orig_fifo_fifo_full : 1;
        unsigned int      ccgen_to_clipcc_fifo_empty : 1;
        unsigned int       ccgen_to_clipcc_fifo_full : 1;
        unsigned int                     ALWAYS_ZERO : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                     ALWAYS_ZERO : 12;
        unsigned int       ccgen_to_clipcc_fifo_full : 1;
        unsigned int      ccgen_to_clipcc_fifo_empty : 1;
        unsigned int     vte_out_orig_fifo_fifo_full : 1;
        unsigned int    vte_out_orig_fifo_fifo_empty : 1;
        unsigned int     vte_out_clip_fifo_fifo_full : 1;
        unsigned int    vte_out_clip_fifo_fifo_empty : 1;
        unsigned int              clipcode_fifo_full : 1;
        unsigned int        clipcode_fifo_fifo_empty : 1;
        unsigned int          vgt_to_clips_fifo_full : 1;
        unsigned int         vgt_to_clips_fifo_empty : 1;
        unsigned int          vgt_to_clipp_fifo_full : 1;
        unsigned int         vgt_to_clipp_fifo_empty : 1;
        unsigned int         clip_to_outsm_fifo_full : 1;
        unsigned int        clip_to_outsm_fifo_empty : 1;
        unsigned int      primic_to_clprim_fifo_full : 1;
        unsigned int     primic_to_clprim_fifo_empty : 1;
        unsigned int            clip_to_ga_fifo_full : 1;
        unsigned int           clip_to_ga_fifo_write : 1;
        unsigned int            clip_ga_bc_fifo_full : 1;
        unsigned int           clip_ga_bc_fifo_write : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CLIPPER_DEBUG_REG01 {
    struct {
#if     defined(qLittleEndian)
        unsigned int     clip_to_outsm_end_of_packet : 1;
        unsigned int clip_to_outsm_first_prim_of_slot : 1;
        unsigned int   clip_to_outsm_deallocate_slot : 3;
        unsigned int      clip_to_outsm_clipped_prim : 1;
        unsigned int    clip_to_outsm_null_primitive : 1;
        unsigned int clip_to_outsm_vertex_store_indx_2 : 4;
        unsigned int clip_to_outsm_vertex_store_indx_1 : 4;
        unsigned int clip_to_outsm_vertex_store_indx_0 : 4;
        unsigned int             clip_vert_vte_valid : 3;
        unsigned int vte_out_clip_rd_vertex_store_indx : 2;
        unsigned int                     ALWAYS_ZERO : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                     ALWAYS_ZERO : 8;
        unsigned int vte_out_clip_rd_vertex_store_indx : 2;
        unsigned int             clip_vert_vte_valid : 3;
        unsigned int clip_to_outsm_vertex_store_indx_0 : 4;
        unsigned int clip_to_outsm_vertex_store_indx_1 : 4;
        unsigned int clip_to_outsm_vertex_store_indx_2 : 4;
        unsigned int    clip_to_outsm_null_primitive : 1;
        unsigned int      clip_to_outsm_clipped_prim : 1;
        unsigned int   clip_to_outsm_deallocate_slot : 3;
        unsigned int clip_to_outsm_first_prim_of_slot : 1;
        unsigned int     clip_to_outsm_end_of_packet : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CLIPPER_DEBUG_REG02 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    ALWAYS_ZERO1 : 21;
        unsigned int clipsm0_clip_to_clipga_clip_to_outsm_cnt : 3;
        unsigned int                    ALWAYS_ZERO0 : 7;
        unsigned int clipsm0_clprim_to_clip_prim_valid : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int clipsm0_clprim_to_clip_prim_valid : 1;
        unsigned int                    ALWAYS_ZERO0 : 7;
        unsigned int clipsm0_clip_to_clipga_clip_to_outsm_cnt : 3;
        unsigned int                    ALWAYS_ZERO1 : 21;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CLIPPER_DEBUG_REG03 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    ALWAYS_ZERO3 : 3;
        unsigned int clipsm0_clprim_to_clip_clip_primitive : 1;
        unsigned int                    ALWAYS_ZERO2 : 3;
        unsigned int clipsm0_clprim_to_clip_null_primitive : 1;
        unsigned int                    ALWAYS_ZERO1 : 12;
        unsigned int clipsm0_clprim_to_clip_clip_code_or : 6;
        unsigned int                    ALWAYS_ZERO0 : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int                    ALWAYS_ZERO0 : 6;
        unsigned int clipsm0_clprim_to_clip_clip_code_or : 6;
        unsigned int                    ALWAYS_ZERO1 : 12;
        unsigned int clipsm0_clprim_to_clip_null_primitive : 1;
        unsigned int                    ALWAYS_ZERO2 : 3;
        unsigned int clipsm0_clprim_to_clip_clip_primitive : 1;
        unsigned int                    ALWAYS_ZERO3 : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CLIPPER_DEBUG_REG04 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    ALWAYS_ZERO2 : 3;
        unsigned int clipsm0_clprim_to_clip_first_prim_of_slot : 1;
        unsigned int                    ALWAYS_ZERO1 : 3;
        unsigned int    clipsm0_clprim_to_clip_event : 1;
        unsigned int                    ALWAYS_ZERO0 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                    ALWAYS_ZERO0 : 24;
        unsigned int    clipsm0_clprim_to_clip_event : 1;
        unsigned int                    ALWAYS_ZERO1 : 3;
        unsigned int clipsm0_clprim_to_clip_first_prim_of_slot : 1;
        unsigned int                    ALWAYS_ZERO2 : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CLIPPER_DEBUG_REG05 {
    struct {
#if     defined(qLittleEndian)
        unsigned int clipsm0_clprim_to_clip_state_var_indx : 1;
        unsigned int                    ALWAYS_ZERO3 : 2;
        unsigned int clipsm0_clprim_to_clip_deallocate_slot : 3;
        unsigned int clipsm0_clprim_to_clip_event_id : 6;
        unsigned int clipsm0_clprim_to_clip_vertex_store_indx_2 : 4;
        unsigned int                    ALWAYS_ZERO2 : 2;
        unsigned int clipsm0_clprim_to_clip_vertex_store_indx_1 : 4;
        unsigned int                    ALWAYS_ZERO1 : 2;
        unsigned int clipsm0_clprim_to_clip_vertex_store_indx_0 : 4;
        unsigned int                    ALWAYS_ZERO0 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                    ALWAYS_ZERO0 : 4;
        unsigned int clipsm0_clprim_to_clip_vertex_store_indx_0 : 4;
        unsigned int                    ALWAYS_ZERO1 : 2;
        unsigned int clipsm0_clprim_to_clip_vertex_store_indx_1 : 4;
        unsigned int                    ALWAYS_ZERO2 : 2;
        unsigned int clipsm0_clprim_to_clip_vertex_store_indx_2 : 4;
        unsigned int clipsm0_clprim_to_clip_event_id : 6;
        unsigned int clipsm0_clprim_to_clip_deallocate_slot : 3;
        unsigned int                    ALWAYS_ZERO3 : 2;
        unsigned int clipsm0_clprim_to_clip_state_var_indx : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CLIPPER_DEBUG_REG09 {
    struct {
#if     defined(qLittleEndian)
        unsigned int            clprim_in_back_event : 1;
        unsigned int outputclprimtoclip_null_primitive : 1;
        unsigned int clprim_in_back_vertex_store_indx_2 : 4;
        unsigned int                    ALWAYS_ZERO2 : 2;
        unsigned int clprim_in_back_vertex_store_indx_1 : 4;
        unsigned int                    ALWAYS_ZERO1 : 2;
        unsigned int clprim_in_back_vertex_store_indx_0 : 4;
        unsigned int                    ALWAYS_ZERO0 : 2;
        unsigned int                 prim_back_valid : 1;
        unsigned int  clip_priority_seq_indx_out_cnt : 4;
        unsigned int      outsm_clr_rd_orig_vertices : 2;
        unsigned int        outsm_clr_rd_clipsm_wait : 1;
        unsigned int            outsm_clr_fifo_empty : 1;
        unsigned int             outsm_clr_fifo_full : 1;
        unsigned int     clip_priority_seq_indx_load : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int     clip_priority_seq_indx_load : 2;
        unsigned int             outsm_clr_fifo_full : 1;
        unsigned int            outsm_clr_fifo_empty : 1;
        unsigned int        outsm_clr_rd_clipsm_wait : 1;
        unsigned int      outsm_clr_rd_orig_vertices : 2;
        unsigned int  clip_priority_seq_indx_out_cnt : 4;
        unsigned int                 prim_back_valid : 1;
        unsigned int                    ALWAYS_ZERO0 : 2;
        unsigned int clprim_in_back_vertex_store_indx_0 : 4;
        unsigned int                    ALWAYS_ZERO1 : 2;
        unsigned int clprim_in_back_vertex_store_indx_1 : 4;
        unsigned int                    ALWAYS_ZERO2 : 2;
        unsigned int clprim_in_back_vertex_store_indx_2 : 4;
        unsigned int outputclprimtoclip_null_primitive : 1;
        unsigned int            clprim_in_back_event : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CLIPPER_DEBUG_REG10 {
    struct {
#if     defined(qLittleEndian)
        unsigned int primic_to_clprim_fifo_vertex_store_indx_2 : 4;
        unsigned int                    ALWAYS_ZERO3 : 2;
        unsigned int primic_to_clprim_fifo_vertex_store_indx_1 : 4;
        unsigned int                    ALWAYS_ZERO2 : 2;
        unsigned int primic_to_clprim_fifo_vertex_store_indx_0 : 4;
        unsigned int                    ALWAYS_ZERO1 : 2;
        unsigned int   clprim_in_back_state_var_indx : 1;
        unsigned int                    ALWAYS_ZERO0 : 2;
        unsigned int    clprim_in_back_end_of_packet : 1;
        unsigned int clprim_in_back_first_prim_of_slot : 1;
        unsigned int  clprim_in_back_deallocate_slot : 3;
        unsigned int         clprim_in_back_event_id : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int         clprim_in_back_event_id : 6;
        unsigned int  clprim_in_back_deallocate_slot : 3;
        unsigned int clprim_in_back_first_prim_of_slot : 1;
        unsigned int    clprim_in_back_end_of_packet : 1;
        unsigned int                    ALWAYS_ZERO0 : 2;
        unsigned int   clprim_in_back_state_var_indx : 1;
        unsigned int                    ALWAYS_ZERO1 : 2;
        unsigned int primic_to_clprim_fifo_vertex_store_indx_0 : 4;
        unsigned int                    ALWAYS_ZERO2 : 2;
        unsigned int primic_to_clprim_fifo_vertex_store_indx_1 : 4;
        unsigned int                    ALWAYS_ZERO3 : 2;
        unsigned int primic_to_clprim_fifo_vertex_store_indx_2 : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CLIPPER_DEBUG_REG11 {
    struct {
#if     defined(qLittleEndian)
        unsigned int vertval_bits_vertex_vertex_store_msb : 4;
        unsigned int                     ALWAYS_ZERO : 28;
#else       /* !defined(qLittleEndian) */
        unsigned int                     ALWAYS_ZERO : 28;
        unsigned int vertval_bits_vertex_vertex_store_msb : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CLIPPER_DEBUG_REG12 {
    struct {
#if     defined(qLittleEndian)
        unsigned int clip_priority_available_vte_out_clip : 2;
        unsigned int                    ALWAYS_ZERO2 : 3;
        unsigned int          clip_vertex_fifo_empty : 1;
        unsigned int clip_priority_available_clip_verts : 5;
        unsigned int                    ALWAYS_ZERO1 : 4;
        unsigned int vertval_bits_vertex_cc_next_valid : 4;
        unsigned int        clipcc_vertex_store_indx : 2;
        unsigned int          primic_to_clprim_valid : 1;
        unsigned int                    ALWAYS_ZERO0 : 10;
#else       /* !defined(qLittleEndian) */
        unsigned int                    ALWAYS_ZERO0 : 10;
        unsigned int          primic_to_clprim_valid : 1;
        unsigned int        clipcc_vertex_store_indx : 2;
        unsigned int vertval_bits_vertex_cc_next_valid : 4;
        unsigned int                    ALWAYS_ZERO1 : 4;
        unsigned int clip_priority_available_clip_verts : 5;
        unsigned int          clip_vertex_fifo_empty : 1;
        unsigned int                    ALWAYS_ZERO2 : 3;
        unsigned int clip_priority_available_vte_out_clip : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CLIPPER_DEBUG_REG13 {
    struct {
#if     defined(qLittleEndian)
        unsigned int               sm0_clip_vert_cnt : 4;
        unsigned int              sm0_prim_end_state : 7;
        unsigned int                    ALWAYS_ZERO1 : 3;
        unsigned int             sm0_vertex_clip_cnt : 4;
        unsigned int    sm0_inv_to_clip_data_valid_1 : 1;
        unsigned int    sm0_inv_to_clip_data_valid_0 : 1;
        unsigned int               sm0_current_state : 7;
        unsigned int                    ALWAYS_ZERO0 : 5;
#else       /* !defined(qLittleEndian) */
        unsigned int                    ALWAYS_ZERO0 : 5;
        unsigned int               sm0_current_state : 7;
        unsigned int    sm0_inv_to_clip_data_valid_0 : 1;
        unsigned int    sm0_inv_to_clip_data_valid_1 : 1;
        unsigned int             sm0_vertex_clip_cnt : 4;
        unsigned int                    ALWAYS_ZERO1 : 3;
        unsigned int              sm0_prim_end_state : 7;
        unsigned int               sm0_clip_vert_cnt : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SXIFCCG_DEBUG_REG0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   nan_kill_flag : 4;
        unsigned int                position_address : 3;
        unsigned int                    ALWAYS_ZERO2 : 3;
        unsigned int                   point_address : 3;
        unsigned int                    ALWAYS_ZERO1 : 3;
        unsigned int    sx_pending_rd_state_var_indx : 1;
        unsigned int                    ALWAYS_ZERO0 : 2;
        unsigned int          sx_pending_rd_req_mask : 4;
        unsigned int               sx_pending_rd_pci : 7;
        unsigned int           sx_pending_rd_aux_inc : 1;
        unsigned int           sx_pending_rd_aux_sel : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int           sx_pending_rd_aux_sel : 1;
        unsigned int           sx_pending_rd_aux_inc : 1;
        unsigned int               sx_pending_rd_pci : 7;
        unsigned int          sx_pending_rd_req_mask : 4;
        unsigned int                    ALWAYS_ZERO0 : 2;
        unsigned int    sx_pending_rd_state_var_indx : 1;
        unsigned int                    ALWAYS_ZERO1 : 3;
        unsigned int                   point_address : 3;
        unsigned int                    ALWAYS_ZERO2 : 3;
        unsigned int                position_address : 3;
        unsigned int                   nan_kill_flag : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SXIFCCG_DEBUG_REG1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    ALWAYS_ZERO3 : 2;
        unsigned int                  sx_to_pa_empty : 2;
        unsigned int             available_positions : 3;
        unsigned int                    ALWAYS_ZERO2 : 4;
        unsigned int              sx_pending_advance : 1;
        unsigned int                 sx_receive_indx : 3;
        unsigned int   statevar_bits_sxpa_aux_vector : 1;
        unsigned int                    ALWAYS_ZERO1 : 4;
        unsigned int                         aux_sel : 1;
        unsigned int                    ALWAYS_ZERO0 : 2;
        unsigned int                    pasx_req_cnt : 2;
        unsigned int                param_cache_base : 7;
#else       /* !defined(qLittleEndian) */
        unsigned int                param_cache_base : 7;
        unsigned int                    pasx_req_cnt : 2;
        unsigned int                    ALWAYS_ZERO0 : 2;
        unsigned int                         aux_sel : 1;
        unsigned int                    ALWAYS_ZERO1 : 4;
        unsigned int   statevar_bits_sxpa_aux_vector : 1;
        unsigned int                 sx_receive_indx : 3;
        unsigned int              sx_pending_advance : 1;
        unsigned int                    ALWAYS_ZERO2 : 4;
        unsigned int             available_positions : 3;
        unsigned int                  sx_to_pa_empty : 2;
        unsigned int                    ALWAYS_ZERO3 : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SXIFCCG_DEBUG_REG2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         sx_sent : 1;
        unsigned int                    ALWAYS_ZERO3 : 1;
        unsigned int                          sx_aux : 1;
        unsigned int                 sx_request_indx : 6;
        unsigned int                req_active_verts : 7;
        unsigned int                    ALWAYS_ZERO2 : 1;
        unsigned int     vgt_to_ccgen_state_var_indx : 1;
        unsigned int                    ALWAYS_ZERO1 : 2;
        unsigned int       vgt_to_ccgen_active_verts : 2;
        unsigned int                    ALWAYS_ZERO0 : 4;
        unsigned int         req_active_verts_loaded : 1;
        unsigned int           sx_pending_fifo_empty : 1;
        unsigned int            sx_pending_fifo_full : 1;
        unsigned int        sx_pending_fifo_contents : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int        sx_pending_fifo_contents : 3;
        unsigned int            sx_pending_fifo_full : 1;
        unsigned int           sx_pending_fifo_empty : 1;
        unsigned int         req_active_verts_loaded : 1;
        unsigned int                    ALWAYS_ZERO0 : 4;
        unsigned int       vgt_to_ccgen_active_verts : 2;
        unsigned int                    ALWAYS_ZERO1 : 2;
        unsigned int     vgt_to_ccgen_state_var_indx : 1;
        unsigned int                    ALWAYS_ZERO2 : 1;
        unsigned int                req_active_verts : 7;
        unsigned int                 sx_request_indx : 6;
        unsigned int                          sx_aux : 1;
        unsigned int                    ALWAYS_ZERO3 : 1;
        unsigned int                         sx_sent : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SXIFCCG_DEBUG_REG3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int    vertex_fifo_entriesavailable : 4;
        unsigned int                    ALWAYS_ZERO3 : 1;
        unsigned int             available_positions : 3;
        unsigned int                    ALWAYS_ZERO2 : 4;
        unsigned int                   current_state : 2;
        unsigned int               vertex_fifo_empty : 1;
        unsigned int                vertex_fifo_full : 1;
        unsigned int                    ALWAYS_ZERO1 : 2;
        unsigned int          sx0_receive_fifo_empty : 1;
        unsigned int           sx0_receive_fifo_full : 1;
        unsigned int         vgt_to_ccgen_fifo_empty : 1;
        unsigned int          vgt_to_ccgen_fifo_full : 1;
        unsigned int                    ALWAYS_ZERO0 : 10;
#else       /* !defined(qLittleEndian) */
        unsigned int                    ALWAYS_ZERO0 : 10;
        unsigned int          vgt_to_ccgen_fifo_full : 1;
        unsigned int         vgt_to_ccgen_fifo_empty : 1;
        unsigned int           sx0_receive_fifo_full : 1;
        unsigned int          sx0_receive_fifo_empty : 1;
        unsigned int                    ALWAYS_ZERO1 : 2;
        unsigned int                vertex_fifo_full : 1;
        unsigned int               vertex_fifo_empty : 1;
        unsigned int                   current_state : 2;
        unsigned int                    ALWAYS_ZERO2 : 4;
        unsigned int             available_positions : 3;
        unsigned int                    ALWAYS_ZERO3 : 1;
        unsigned int    vertex_fifo_entriesavailable : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SETUP_DEBUG_REG0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   su_cntl_state : 5;
        unsigned int                     pmode_state : 6;
        unsigned int                       ge_stallb : 1;
        unsigned int                     geom_enable : 1;
        unsigned int               su_clip_baryc_rtr : 1;
        unsigned int                     su_clip_rtr : 1;
        unsigned int                      pfifo_busy : 1;
        unsigned int                    su_cntl_busy : 1;
        unsigned int                       geom_busy : 1;
        unsigned int                                 : 14;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 14;
        unsigned int                       geom_busy : 1;
        unsigned int                    su_cntl_busy : 1;
        unsigned int                      pfifo_busy : 1;
        unsigned int                     su_clip_rtr : 1;
        unsigned int               su_clip_baryc_rtr : 1;
        unsigned int                     geom_enable : 1;
        unsigned int                       ge_stallb : 1;
        unsigned int                     pmode_state : 6;
        unsigned int                   su_cntl_state : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SETUP_DEBUG_REG1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int              y_sort0_gated_17_4 : 14;
        unsigned int              x_sort0_gated_17_4 : 14;
        unsigned int                                 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 4;
        unsigned int              x_sort0_gated_17_4 : 14;
        unsigned int              y_sort0_gated_17_4 : 14;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SETUP_DEBUG_REG2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int              y_sort1_gated_17_4 : 14;
        unsigned int              x_sort1_gated_17_4 : 14;
        unsigned int                                 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 4;
        unsigned int              x_sort1_gated_17_4 : 14;
        unsigned int              y_sort1_gated_17_4 : 14;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SETUP_DEBUG_REG3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int              y_sort2_gated_17_4 : 14;
        unsigned int              x_sort2_gated_17_4 : 14;
        unsigned int                                 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 4;
        unsigned int              x_sort2_gated_17_4 : 14;
        unsigned int              y_sort2_gated_17_4 : 14;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SETUP_DEBUG_REG4 {
    struct {
#if     defined(qLittleEndian)
        unsigned int           attr_indx_sort0_gated : 11;
        unsigned int                 null_prim_gated : 1;
        unsigned int                backfacing_gated : 1;
        unsigned int                   st_indx_gated : 3;
        unsigned int                   clipped_gated : 1;
        unsigned int              dealloc_slot_gated : 3;
        unsigned int                    xmajor_gated : 1;
        unsigned int              diamond_rule_gated : 2;
        unsigned int                      type_gated : 3;
        unsigned int                      fpov_gated : 1;
        unsigned int                pmode_prim_gated : 1;
        unsigned int                     event_gated : 1;
        unsigned int                       eop_gated : 1;
        unsigned int                                 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 2;
        unsigned int                       eop_gated : 1;
        unsigned int                     event_gated : 1;
        unsigned int                pmode_prim_gated : 1;
        unsigned int                      fpov_gated : 1;
        unsigned int                      type_gated : 3;
        unsigned int              diamond_rule_gated : 2;
        unsigned int                    xmajor_gated : 1;
        unsigned int              dealloc_slot_gated : 3;
        unsigned int                   clipped_gated : 1;
        unsigned int                   st_indx_gated : 3;
        unsigned int                backfacing_gated : 1;
        unsigned int                 null_prim_gated : 1;
        unsigned int           attr_indx_sort0_gated : 11;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SETUP_DEBUG_REG5 {
    struct {
#if     defined(qLittleEndian)
        unsigned int           attr_indx_sort2_gated : 11;
        unsigned int           attr_indx_sort1_gated : 11;
        unsigned int             provoking_vtx_gated : 2;
        unsigned int                  event_id_gated : 5;
        unsigned int                                 : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 3;
        unsigned int                  event_id_gated : 5;
        unsigned int             provoking_vtx_gated : 2;
        unsigned int           attr_indx_sort1_gated : 11;
        unsigned int           attr_indx_sort2_gated : 11;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_DEBUG_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   SC_DEBUG_INDX : 5;
        unsigned int                                 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 27;
        unsigned int                   SC_DEBUG_INDX : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union PA_SC_DEBUG_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                            DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    pa_freeze_b1 : 1;
        unsigned int                     pa_sc_valid : 1;
        unsigned int                     pa_sc_phase : 3;
        unsigned int                        cntx_cnt : 7;
        unsigned int                   decr_cntx_cnt : 1;
        unsigned int                   incr_cntx_cnt : 1;
        unsigned int                                 : 17;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                                 : 17;
        unsigned int                   incr_cntx_cnt : 1;
        unsigned int                   decr_cntx_cnt : 1;
        unsigned int                        cntx_cnt : 7;
        unsigned int                     pa_sc_phase : 3;
        unsigned int                     pa_sc_valid : 1;
        unsigned int                    pa_freeze_b1 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        em_state : 3;
        unsigned int                  em1_data_ready : 1;
        unsigned int                  em2_data_ready : 1;
        unsigned int                 move_em1_to_em2 : 1;
        unsigned int                   ef_data_ready : 1;
        unsigned int                        ef_state : 2;
        unsigned int                      pipe_valid : 1;
        unsigned int                                 : 21;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                                 : 21;
        unsigned int                      pipe_valid : 1;
        unsigned int                        ef_state : 2;
        unsigned int                   ef_data_ready : 1;
        unsigned int                 move_em1_to_em2 : 1;
        unsigned int                  em2_data_ready : 1;
        unsigned int                  em1_data_ready : 1;
        unsigned int                        em_state : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      rc_rtr_dly : 1;
        unsigned int            qmask_ff_alm_full_d1 : 1;
        unsigned int                                 : 1;
        unsigned int                   pipe_freeze_b : 1;
        unsigned int                        prim_rts : 1;
        unsigned int               next_prim_rts_dly : 1;
        unsigned int               next_prim_rtr_dly : 1;
        unsigned int               pre_stage1_rts_d1 : 1;
        unsigned int                      stage0_rts : 1;
        unsigned int                   phase_rts_dly : 1;
        unsigned int                                 : 5;
        unsigned int              end_of_prim_s1_dly : 1;
        unsigned int              pass_empty_prim_s1 : 1;
        unsigned int                     event_id_s1 : 5;
        unsigned int                        event_s1 : 1;
        unsigned int                                 : 8;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                                 : 8;
        unsigned int                        event_s1 : 1;
        unsigned int                     event_id_s1 : 5;
        unsigned int              pass_empty_prim_s1 : 1;
        unsigned int              end_of_prim_s1_dly : 1;
        unsigned int                                 : 5;
        unsigned int                   phase_rts_dly : 1;
        unsigned int                      stage0_rts : 1;
        unsigned int               pre_stage1_rts_d1 : 1;
        unsigned int               next_prim_rtr_dly : 1;
        unsigned int               next_prim_rts_dly : 1;
        unsigned int                        prim_rts : 1;
        unsigned int                   pipe_freeze_b : 1;
        unsigned int                                 : 1;
        unsigned int            qmask_ff_alm_full_d1 : 1;
        unsigned int                      rc_rtr_dly : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       x_curr_s1 : 11;
        unsigned int                       y_curr_s1 : 11;
        unsigned int                                 : 9;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                                 : 9;
        unsigned int                       y_curr_s1 : 11;
        unsigned int                       x_curr_s1 : 11;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_4 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        y_end_s1 : 14;
        unsigned int                      y_start_s1 : 14;
        unsigned int                        y_dir_s1 : 1;
        unsigned int                                 : 2;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                                 : 2;
        unsigned int                        y_dir_s1 : 1;
        unsigned int                      y_start_s1 : 14;
        unsigned int                        y_end_s1 : 14;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_5 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        x_end_s1 : 14;
        unsigned int                      x_start_s1 : 14;
        unsigned int                        x_dir_s1 : 1;
        unsigned int                                 : 2;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                                 : 2;
        unsigned int                        x_dir_s1 : 1;
        unsigned int                      x_start_s1 : 14;
        unsigned int                        x_end_s1 : 14;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_6 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      z_ff_empty : 1;
        unsigned int                 qmcntl_ff_empty : 1;
        unsigned int                     xy_ff_empty : 1;
        unsigned int                      event_flag : 1;
        unsigned int                   z_mask_needed : 1;
        unsigned int                           state : 3;
        unsigned int                   state_delayed : 3;
        unsigned int                      data_valid : 1;
        unsigned int                    data_valid_d : 1;
        unsigned int                   tilex_delayed : 9;
        unsigned int                   tiley_delayed : 9;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                   tiley_delayed : 9;
        unsigned int                   tilex_delayed : 9;
        unsigned int                    data_valid_d : 1;
        unsigned int                      data_valid : 1;
        unsigned int                   state_delayed : 3;
        unsigned int                           state : 3;
        unsigned int                   z_mask_needed : 1;
        unsigned int                      event_flag : 1;
        unsigned int                     xy_ff_empty : 1;
        unsigned int                 qmcntl_ff_empty : 1;
        unsigned int                      z_ff_empty : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_7 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      event_flag : 1;
        unsigned int                      deallocate : 3;
        unsigned int                       fposition : 1;
        unsigned int                      sr_prim_we : 1;
        unsigned int                       last_tile : 1;
        unsigned int                      tile_ff_we : 1;
        unsigned int                   qs_data_valid : 1;
        unsigned int                         qs_q0_y : 2;
        unsigned int                         qs_q0_x : 2;
        unsigned int                     qs_q0_valid : 1;
        unsigned int                      prim_ff_we : 1;
        unsigned int                      tile_ff_re : 1;
        unsigned int              fw_prim_data_valid : 1;
        unsigned int               last_quad_of_tile : 1;
        unsigned int              first_quad_of_tile : 1;
        unsigned int              first_quad_of_prim : 1;
        unsigned int                        new_prim : 1;
        unsigned int              load_new_tile_data : 1;
        unsigned int                           state : 2;
        unsigned int                     fifos_ready : 1;
        unsigned int                                 : 6;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                                 : 6;
        unsigned int                     fifos_ready : 1;
        unsigned int                           state : 2;
        unsigned int              load_new_tile_data : 1;
        unsigned int                        new_prim : 1;
        unsigned int              first_quad_of_prim : 1;
        unsigned int              first_quad_of_tile : 1;
        unsigned int               last_quad_of_tile : 1;
        unsigned int              fw_prim_data_valid : 1;
        unsigned int                      tile_ff_re : 1;
        unsigned int                      prim_ff_we : 1;
        unsigned int                     qs_q0_valid : 1;
        unsigned int                         qs_q0_x : 2;
        unsigned int                         qs_q0_y : 2;
        unsigned int                   qs_data_valid : 1;
        unsigned int                      tile_ff_we : 1;
        unsigned int                       last_tile : 1;
        unsigned int                      sr_prim_we : 1;
        unsigned int                       fposition : 1;
        unsigned int                      deallocate : 3;
        unsigned int                      event_flag : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_8 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     sample_last : 1;
        unsigned int                     sample_mask : 4;
        unsigned int                        sample_y : 2;
        unsigned int                        sample_x : 2;
        unsigned int                     sample_send : 1;
        unsigned int                      next_cycle : 2;
        unsigned int               ez_sample_ff_full : 1;
        unsigned int                  rb_sc_samp_rtr : 1;
        unsigned int                     num_samples : 2;
        unsigned int               last_quad_of_tile : 1;
        unsigned int               last_quad_of_prim : 1;
        unsigned int              first_quad_of_prim : 1;
        unsigned int                       sample_we : 1;
        unsigned int                       fposition : 1;
        unsigned int                        event_id : 5;
        unsigned int                      event_flag : 1;
        unsigned int              fw_prim_data_valid : 1;
        unsigned int                                 : 3;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                                 : 3;
        unsigned int              fw_prim_data_valid : 1;
        unsigned int                      event_flag : 1;
        unsigned int                        event_id : 5;
        unsigned int                       fposition : 1;
        unsigned int                       sample_we : 1;
        unsigned int              first_quad_of_prim : 1;
        unsigned int               last_quad_of_prim : 1;
        unsigned int               last_quad_of_tile : 1;
        unsigned int                     num_samples : 2;
        unsigned int                  rb_sc_samp_rtr : 1;
        unsigned int               ez_sample_ff_full : 1;
        unsigned int                      next_cycle : 2;
        unsigned int                     sample_send : 1;
        unsigned int                        sample_x : 2;
        unsigned int                        sample_y : 2;
        unsigned int                     sample_mask : 4;
        unsigned int                     sample_last : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_9 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      rb_sc_send : 1;
        unsigned int                   rb_sc_ez_mask : 4;
        unsigned int                 fifo_data_ready : 1;
        unsigned int                  early_z_enable : 1;
        unsigned int                      mask_state : 2;
        unsigned int                    next_ez_mask : 16;
        unsigned int                      mask_ready : 1;
        unsigned int                     drop_sample : 1;
        unsigned int           fetch_new_sample_data : 1;
        unsigned int        fetch_new_ez_sample_mask : 1;
        unsigned int       pkr_fetch_new_sample_data : 1;
        unsigned int         pkr_fetch_new_prim_data : 1;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int         pkr_fetch_new_prim_data : 1;
        unsigned int       pkr_fetch_new_sample_data : 1;
        unsigned int        fetch_new_ez_sample_mask : 1;
        unsigned int           fetch_new_sample_data : 1;
        unsigned int                     drop_sample : 1;
        unsigned int                      mask_ready : 1;
        unsigned int                    next_ez_mask : 16;
        unsigned int                      mask_state : 2;
        unsigned int                  early_z_enable : 1;
        unsigned int                 fifo_data_ready : 1;
        unsigned int                   rb_sc_ez_mask : 4;
        unsigned int                      rb_sc_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_10 {
    struct {
#if     defined(qLittleEndian)
        unsigned int            combined_sample_mask : 16;
        unsigned int                                 : 15;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                                 : 15;
        unsigned int            combined_sample_mask : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_11 {
    struct {
#if     defined(qLittleEndian)
        unsigned int            ez_sample_data_ready : 1;
        unsigned int       pkr_fetch_new_sample_data : 1;
        unsigned int              ez_prim_data_ready : 1;
        unsigned int         pkr_fetch_new_prim_data : 1;
        unsigned int               iterator_input_fz : 1;
        unsigned int               packer_send_quads : 1;
        unsigned int                 packer_send_cmd : 1;
        unsigned int               packer_send_event : 1;
        unsigned int                      next_state : 3;
        unsigned int                           state : 3;
        unsigned int                           stall : 1;
        unsigned int                                 : 16;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                                 : 16;
        unsigned int                           stall : 1;
        unsigned int                           state : 3;
        unsigned int                      next_state : 3;
        unsigned int               packer_send_event : 1;
        unsigned int                 packer_send_cmd : 1;
        unsigned int               packer_send_quads : 1;
        unsigned int               iterator_input_fz : 1;
        unsigned int         pkr_fetch_new_prim_data : 1;
        unsigned int              ez_prim_data_ready : 1;
        unsigned int       pkr_fetch_new_sample_data : 1;
        unsigned int            ez_sample_data_ready : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SC_DEBUG_12 {
    struct {
#if     defined(qLittleEndian)
        unsigned int           SQ_iterator_free_buff : 1;
        unsigned int                        event_id : 5;
        unsigned int                      event_flag : 1;
        unsigned int         itercmdfifo_busy_nc_dly : 1;
        unsigned int                itercmdfifo_full : 1;
        unsigned int               itercmdfifo_empty : 1;
        unsigned int         iter_ds_one_clk_command : 1;
        unsigned int            iter_ds_end_of_prim0 : 1;
        unsigned int           iter_ds_end_of_vector : 1;
        unsigned int                     iter_qdhit0 : 1;
        unsigned int              bc_use_centers_reg : 1;
        unsigned int                bc_output_xy_reg : 1;
        unsigned int                  iter_phase_out : 2;
        unsigned int                  iter_phase_reg : 2;
        unsigned int               iterator_SP_valid : 1;
        unsigned int                        eopv_reg : 1;
        unsigned int                 one_clk_cmd_reg : 1;
        unsigned int             iter_dx_end_of_prim : 1;
        unsigned int                                 : 7;
        unsigned int                         trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         trigger : 1;
        unsigned int                                 : 7;
        unsigned int             iter_dx_end_of_prim : 1;
        unsigned int                 one_clk_cmd_reg : 1;
        unsigned int                        eopv_reg : 1;
        unsigned int               iterator_SP_valid : 1;
        unsigned int                  iter_phase_reg : 2;
        unsigned int                  iter_phase_out : 2;
        unsigned int                bc_output_xy_reg : 1;
        unsigned int              bc_use_centers_reg : 1;
        unsigned int                     iter_qdhit0 : 1;
        unsigned int           iter_ds_end_of_vector : 1;
        unsigned int            iter_ds_end_of_prim0 : 1;
        unsigned int         iter_ds_one_clk_command : 1;
        unsigned int               itercmdfifo_empty : 1;
        unsigned int                itercmdfifo_full : 1;
        unsigned int         itercmdfifo_busy_nc_dly : 1;
        unsigned int                      event_flag : 1;
        unsigned int                        event_id : 5;
        unsigned int           SQ_iterator_free_buff : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union GFX_COPY_STATE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    SRC_STATE_ID : 1;
        unsigned int                                 : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 31;
        unsigned int                    SRC_STATE_ID : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DRAW_INITIATOR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       PRIM_TYPE : 6;
        unsigned int                   SOURCE_SELECT : 2;
        unsigned int            FACENESS_CULL_SELECT : 2;
        unsigned int                                 : 1;
        unsigned int                      INDEX_SIZE : 1;
        unsigned int                         NOT_EOP : 1;
        unsigned int                     SMALL_INDEX : 1;
        unsigned int           PRE_FETCH_CULL_ENABLE : 1;
        unsigned int                 GRP_CULL_ENABLE : 1;
        unsigned int                     NUM_INDICES : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                     NUM_INDICES : 16;
        unsigned int                 GRP_CULL_ENABLE : 1;
        unsigned int           PRE_FETCH_CULL_ENABLE : 1;
        unsigned int                     SMALL_INDEX : 1;
        unsigned int                         NOT_EOP : 1;
        unsigned int                      INDEX_SIZE : 1;
        unsigned int                                 : 1;
        unsigned int            FACENESS_CULL_SELECT : 2;
        unsigned int                   SOURCE_SELECT : 2;
        unsigned int                       PRIM_TYPE : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_EVENT_INITIATOR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      EVENT_TYPE : 6;
        unsigned int                                 : 26;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 26;
        unsigned int                      EVENT_TYPE : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DMA_BASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       BASE_ADDR : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                       BASE_ADDR : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DMA_SIZE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       NUM_WORDS : 24;
        unsigned int                                 : 6;
        unsigned int                       SWAP_MODE : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                       SWAP_MODE : 2;
        unsigned int                                 : 6;
        unsigned int                       NUM_WORDS : 24;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_BIN_BASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   BIN_BASE_ADDR : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   BIN_BASE_ADDR : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_BIN_SIZE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       NUM_WORDS : 24;
        unsigned int                                 : 6;
        unsigned int                  FACENESS_FETCH : 1;
        unsigned int                  FACENESS_RESET : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                  FACENESS_RESET : 1;
        unsigned int                  FACENESS_FETCH : 1;
        unsigned int                                 : 6;
        unsigned int                       NUM_WORDS : 24;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_CURRENT_BIN_ID_MIN {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          COLUMN : 3;
        unsigned int                             ROW : 3;
        unsigned int                      GUARD_BAND : 3;
        unsigned int                                 : 23;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 23;
        unsigned int                      GUARD_BAND : 3;
        unsigned int                             ROW : 3;
        unsigned int                          COLUMN : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_CURRENT_BIN_ID_MAX {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          COLUMN : 3;
        unsigned int                             ROW : 3;
        unsigned int                      GUARD_BAND : 3;
        unsigned int                                 : 23;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 23;
        unsigned int                      GUARD_BAND : 3;
        unsigned int                             ROW : 3;
        unsigned int                          COLUMN : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_IMMED_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                            DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_MAX_VTX_INDX {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        MAX_INDX : 24;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                        MAX_INDX : 24;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_MIN_VTX_INDX {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        MIN_INDX : 24;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                        MIN_INDX : 24;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_INDX_OFFSET {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     INDX_OFFSET : 24;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                     INDX_OFFSET : 24;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_VERTEX_REUSE_BLOCK_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 VTX_REUSE_DEPTH : 3;
        unsigned int                                 : 29;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 29;
        unsigned int                 VTX_REUSE_DEPTH : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_OUT_DEALLOC_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    DEALLOC_DIST : 2;
        unsigned int                                 : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 30;
        unsigned int                    DEALLOC_DIST : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_MULTI_PRIM_IB_RESET_INDX {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      RESET_INDX : 24;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                      RESET_INDX : 24;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_ENHANCE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            MISC : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                            MISC : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_VTX_VECT_EJECT_REG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PRIM_COUNT : 5;
        unsigned int                                 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 27;
        unsigned int                      PRIM_COUNT : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_LAST_COPY_STATE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    SRC_STATE_ID : 1;
        unsigned int                                 : 15;
        unsigned int                    DST_STATE_ID : 1;
        unsigned int                                 : 15;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 15;
        unsigned int                    DST_STATE_ID : 1;
        unsigned int                                 : 15;
        unsigned int                    SRC_STATE_ID : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  VGT_DEBUG_INDX : 5;
        unsigned int                                 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 27;
        unsigned int                  VGT_DEBUG_INDX : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                            DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_CNTL_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        VGT_BUSY : 1;
        unsigned int                    VGT_DMA_BUSY : 1;
        unsigned int                VGT_DMA_REQ_BUSY : 1;
        unsigned int                    VGT_GRP_BUSY : 1;
        unsigned int                     VGT_VR_BUSY : 1;
        unsigned int                    VGT_BIN_BUSY : 1;
        unsigned int                     VGT_PT_BUSY : 1;
        unsigned int                    VGT_OUT_BUSY : 1;
        unsigned int               VGT_OUT_INDX_BUSY : 1;
        unsigned int                                 : 23;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 23;
        unsigned int               VGT_OUT_INDX_BUSY : 1;
        unsigned int                    VGT_OUT_BUSY : 1;
        unsigned int                     VGT_PT_BUSY : 1;
        unsigned int                    VGT_BIN_BUSY : 1;
        unsigned int                     VGT_VR_BUSY : 1;
        unsigned int                    VGT_GRP_BUSY : 1;
        unsigned int                VGT_DMA_REQ_BUSY : 1;
        unsigned int                    VGT_DMA_BUSY : 1;
        unsigned int                        VGT_BUSY : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     te_grp_busy : 1;
        unsigned int                     pt_grp_busy : 1;
        unsigned int                     vr_grp_busy : 1;
        unsigned int                dma_request_busy : 1;
        unsigned int                        out_busy : 1;
        unsigned int                grp_backend_busy : 1;
        unsigned int                        grp_busy : 1;
        unsigned int                        dma_busy : 1;
        unsigned int           rbiu_dma_request_busy : 1;
        unsigned int                       rbiu_busy : 1;
        unsigned int        vgt_no_dma_busy_extended : 1;
        unsigned int                 vgt_no_dma_busy : 1;
        unsigned int               vgt_busy_extended : 1;
        unsigned int                        vgt_busy : 1;
        unsigned int         rbbm_skid_fifo_busy_out : 1;
        unsigned int            VGT_RBBM_no_dma_busy : 1;
        unsigned int                   VGT_RBBM_busy : 1;
        unsigned int                                 : 15;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 15;
        unsigned int                   VGT_RBBM_busy : 1;
        unsigned int            VGT_RBBM_no_dma_busy : 1;
        unsigned int         rbbm_skid_fifo_busy_out : 1;
        unsigned int                        vgt_busy : 1;
        unsigned int               vgt_busy_extended : 1;
        unsigned int                 vgt_no_dma_busy : 1;
        unsigned int        vgt_no_dma_busy_extended : 1;
        unsigned int                       rbiu_busy : 1;
        unsigned int           rbiu_dma_request_busy : 1;
        unsigned int                        dma_busy : 1;
        unsigned int                        grp_busy : 1;
        unsigned int                grp_backend_busy : 1;
        unsigned int                        out_busy : 1;
        unsigned int                dma_request_busy : 1;
        unsigned int                     vr_grp_busy : 1;
        unsigned int                     pt_grp_busy : 1;
        unsigned int                     te_grp_busy : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                out_te_data_read : 1;
        unsigned int               te_out_data_valid : 1;
        unsigned int                out_pt_prim_read : 1;
        unsigned int               pt_out_prim_valid : 1;
        unsigned int                out_pt_data_read : 1;
        unsigned int               pt_out_indx_valid : 1;
        unsigned int                out_vr_prim_read : 1;
        unsigned int               vr_out_prim_valid : 1;
        unsigned int                out_vr_indx_read : 1;
        unsigned int               vr_out_indx_valid : 1;
        unsigned int                     te_grp_read : 1;
        unsigned int                    grp_te_valid : 1;
        unsigned int                     pt_grp_read : 1;
        unsigned int                    grp_pt_valid : 1;
        unsigned int                     vr_grp_read : 1;
        unsigned int                    grp_vr_valid : 1;
        unsigned int                    grp_dma_read : 1;
        unsigned int                   dma_grp_valid : 1;
        unsigned int                grp_rbiu_di_read : 1;
        unsigned int               rbiu_grp_di_valid : 1;
        unsigned int                      MH_VGT_rtr : 1;
        unsigned int                     VGT_MH_send : 1;
        unsigned int               PA_VGT_clip_s_rtr : 1;
        unsigned int              VGT_PA_clip_s_send : 1;
        unsigned int               PA_VGT_clip_p_rtr : 1;
        unsigned int              VGT_PA_clip_p_send : 1;
        unsigned int               PA_VGT_clip_v_rtr : 1;
        unsigned int              VGT_PA_clip_v_send : 1;
        unsigned int                      SQ_VGT_rtr : 1;
        unsigned int                     VGT_SQ_send : 1;
        unsigned int                  mh_vgt_tag_7_q : 1;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                  mh_vgt_tag_7_q : 1;
        unsigned int                     VGT_SQ_send : 1;
        unsigned int                      SQ_VGT_rtr : 1;
        unsigned int              VGT_PA_clip_v_send : 1;
        unsigned int               PA_VGT_clip_v_rtr : 1;
        unsigned int              VGT_PA_clip_p_send : 1;
        unsigned int               PA_VGT_clip_p_rtr : 1;
        unsigned int              VGT_PA_clip_s_send : 1;
        unsigned int               PA_VGT_clip_s_rtr : 1;
        unsigned int                     VGT_MH_send : 1;
        unsigned int                      MH_VGT_rtr : 1;
        unsigned int               rbiu_grp_di_valid : 1;
        unsigned int                grp_rbiu_di_read : 1;
        unsigned int                   dma_grp_valid : 1;
        unsigned int                    grp_dma_read : 1;
        unsigned int                    grp_vr_valid : 1;
        unsigned int                     vr_grp_read : 1;
        unsigned int                    grp_pt_valid : 1;
        unsigned int                     pt_grp_read : 1;
        unsigned int                    grp_te_valid : 1;
        unsigned int                     te_grp_read : 1;
        unsigned int               vr_out_indx_valid : 1;
        unsigned int                out_vr_indx_read : 1;
        unsigned int               vr_out_prim_valid : 1;
        unsigned int                out_vr_prim_read : 1;
        unsigned int               pt_out_indx_valid : 1;
        unsigned int                out_pt_data_read : 1;
        unsigned int               pt_out_prim_valid : 1;
        unsigned int                out_pt_prim_read : 1;
        unsigned int               te_out_data_valid : 1;
        unsigned int                out_te_data_read : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      vgt_clk_en : 1;
        unsigned int                reg_fifos_clk_en : 1;
        unsigned int                                 : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 30;
        unsigned int                reg_fifos_clk_en : 1;
        unsigned int                      vgt_clk_en : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG6 {
    struct {
#if     defined(qLittleEndian)
        unsigned int            shifter_byte_count_q : 5;
        unsigned int               right_word_indx_q : 5;
        unsigned int                input_data_valid : 1;
        unsigned int                 input_data_xfer : 1;
        unsigned int          next_shift_is_vect_1_q : 1;
        unsigned int          next_shift_is_vect_1_d : 1;
        unsigned int      next_shift_is_vect_1_pre_d : 1;
        unsigned int          space_avail_from_shift : 1;
        unsigned int              shifter_first_load : 1;
        unsigned int                  di_state_sel_q : 1;
        unsigned int shifter_waiting_for_first_load_q : 1;
        unsigned int           di_first_group_flag_q : 1;
        unsigned int                 di_event_flag_q : 1;
        unsigned int             read_draw_initiator : 1;
        unsigned int     loading_di_requires_shifter : 1;
        unsigned int            last_shift_of_packet : 1;
        unsigned int             last_decr_of_packet : 1;
        unsigned int                  extract_vector : 1;
        unsigned int                  shift_vect_rtr : 1;
        unsigned int                 destination_rtr : 1;
        unsigned int                     grp_trigger : 1;
        unsigned int                                 : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 3;
        unsigned int                     grp_trigger : 1;
        unsigned int                 destination_rtr : 1;
        unsigned int                  shift_vect_rtr : 1;
        unsigned int                  extract_vector : 1;
        unsigned int             last_decr_of_packet : 1;
        unsigned int            last_shift_of_packet : 1;
        unsigned int     loading_di_requires_shifter : 1;
        unsigned int             read_draw_initiator : 1;
        unsigned int                 di_event_flag_q : 1;
        unsigned int           di_first_group_flag_q : 1;
        unsigned int shifter_waiting_for_first_load_q : 1;
        unsigned int                  di_state_sel_q : 1;
        unsigned int              shifter_first_load : 1;
        unsigned int          space_avail_from_shift : 1;
        unsigned int      next_shift_is_vect_1_pre_d : 1;
        unsigned int          next_shift_is_vect_1_d : 1;
        unsigned int          next_shift_is_vect_1_q : 1;
        unsigned int                 input_data_xfer : 1;
        unsigned int                input_data_valid : 1;
        unsigned int               right_word_indx_q : 5;
        unsigned int            shifter_byte_count_q : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG7 {
    struct {
#if     defined(qLittleEndian)
        unsigned int              di_index_counter_q : 16;
        unsigned int         shift_amount_no_extract : 4;
        unsigned int            shift_amount_extract : 4;
        unsigned int                  di_prim_type_q : 6;
        unsigned int              current_source_sel : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int              current_source_sel : 2;
        unsigned int                  di_prim_type_q : 6;
        unsigned int            shift_amount_extract : 4;
        unsigned int         shift_amount_no_extract : 4;
        unsigned int              di_index_counter_q : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG8 {
    struct {
#if     defined(qLittleEndian)
        unsigned int              current_source_sel : 2;
        unsigned int                left_word_indx_q : 5;
        unsigned int                  input_data_cnt : 5;
        unsigned int                  input_data_lsw : 5;
        unsigned int                  input_data_msw : 5;
        unsigned int next_small_stride_shift_limit_q : 5;
        unsigned int current_small_stride_shift_limit_q : 5;
#else       /* !defined(qLittleEndian) */
        unsigned int current_small_stride_shift_limit_q : 5;
        unsigned int next_small_stride_shift_limit_q : 5;
        unsigned int                  input_data_msw : 5;
        unsigned int                  input_data_lsw : 5;
        unsigned int                  input_data_cnt : 5;
        unsigned int                left_word_indx_q : 5;
        unsigned int              current_source_sel : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG9 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   next_stride_q : 5;
        unsigned int                   next_stride_d : 5;
        unsigned int                 current_shift_q : 5;
        unsigned int                 current_shift_d : 5;
        unsigned int                current_stride_q : 5;
        unsigned int                current_stride_d : 5;
        unsigned int                     grp_trigger : 1;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                     grp_trigger : 1;
        unsigned int                current_stride_d : 5;
        unsigned int                current_stride_q : 5;
        unsigned int                 current_shift_d : 5;
        unsigned int                 current_shift_q : 5;
        unsigned int                   next_stride_d : 5;
        unsigned int                   next_stride_q : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG10 {
    struct {
#if     defined(qLittleEndian)
        unsigned int    temp_derived_di_prim_type_t0 : 1;
        unsigned int  temp_derived_di_small_index_t0 : 1;
        unsigned int  temp_derived_di_cull_enable_t0 : 1;
        unsigned int temp_derived_di_pre_fetch_cull_enable_t0 : 1;
        unsigned int                  di_state_sel_q : 1;
        unsigned int             last_decr_of_packet : 1;
        unsigned int                       bin_valid : 1;
        unsigned int                      read_block : 1;
        unsigned int          grp_bgrp_last_bit_read : 1;
        unsigned int               last_bit_enable_q : 1;
        unsigned int               last_bit_end_di_q : 1;
        unsigned int                   selected_data : 8;
        unsigned int                 mask_input_data : 8;
        unsigned int                           gap_q : 1;
        unsigned int               temp_mini_reset_z : 1;
        unsigned int               temp_mini_reset_y : 1;
        unsigned int               temp_mini_reset_x : 1;
        unsigned int                     grp_trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     grp_trigger : 1;
        unsigned int               temp_mini_reset_x : 1;
        unsigned int               temp_mini_reset_y : 1;
        unsigned int               temp_mini_reset_z : 1;
        unsigned int                           gap_q : 1;
        unsigned int                 mask_input_data : 8;
        unsigned int                   selected_data : 8;
        unsigned int               last_bit_end_di_q : 1;
        unsigned int               last_bit_enable_q : 1;
        unsigned int          grp_bgrp_last_bit_read : 1;
        unsigned int                      read_block : 1;
        unsigned int                       bin_valid : 1;
        unsigned int             last_decr_of_packet : 1;
        unsigned int                  di_state_sel_q : 1;
        unsigned int temp_derived_di_pre_fetch_cull_enable_t0 : 1;
        unsigned int  temp_derived_di_cull_enable_t0 : 1;
        unsigned int  temp_derived_di_small_index_t0 : 1;
        unsigned int    temp_derived_di_prim_type_t0 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG12 {
    struct {
#if     defined(qLittleEndian)
        unsigned int            shifter_byte_count_q : 5;
        unsigned int               right_word_indx_q : 5;
        unsigned int                input_data_valid : 1;
        unsigned int                 input_data_xfer : 1;
        unsigned int          next_shift_is_vect_1_q : 1;
        unsigned int          next_shift_is_vect_1_d : 1;
        unsigned int      next_shift_is_vect_1_pre_d : 1;
        unsigned int          space_avail_from_shift : 1;
        unsigned int              shifter_first_load : 1;
        unsigned int                  di_state_sel_q : 1;
        unsigned int shifter_waiting_for_first_load_q : 1;
        unsigned int           di_first_group_flag_q : 1;
        unsigned int                 di_event_flag_q : 1;
        unsigned int             read_draw_initiator : 1;
        unsigned int     loading_di_requires_shifter : 1;
        unsigned int            last_shift_of_packet : 1;
        unsigned int             last_decr_of_packet : 1;
        unsigned int                  extract_vector : 1;
        unsigned int                  shift_vect_rtr : 1;
        unsigned int                 destination_rtr : 1;
        unsigned int                    bgrp_trigger : 1;
        unsigned int                                 : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 3;
        unsigned int                    bgrp_trigger : 1;
        unsigned int                 destination_rtr : 1;
        unsigned int                  shift_vect_rtr : 1;
        unsigned int                  extract_vector : 1;
        unsigned int             last_decr_of_packet : 1;
        unsigned int            last_shift_of_packet : 1;
        unsigned int     loading_di_requires_shifter : 1;
        unsigned int             read_draw_initiator : 1;
        unsigned int                 di_event_flag_q : 1;
        unsigned int           di_first_group_flag_q : 1;
        unsigned int shifter_waiting_for_first_load_q : 1;
        unsigned int                  di_state_sel_q : 1;
        unsigned int              shifter_first_load : 1;
        unsigned int          space_avail_from_shift : 1;
        unsigned int      next_shift_is_vect_1_pre_d : 1;
        unsigned int          next_shift_is_vect_1_d : 1;
        unsigned int          next_shift_is_vect_1_q : 1;
        unsigned int                 input_data_xfer : 1;
        unsigned int                input_data_valid : 1;
        unsigned int               right_word_indx_q : 5;
        unsigned int            shifter_byte_count_q : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG13 {
    struct {
#if     defined(qLittleEndian)
        unsigned int              di_index_counter_q : 16;
        unsigned int         shift_amount_no_extract : 4;
        unsigned int            shift_amount_extract : 4;
        unsigned int                  di_prim_type_q : 6;
        unsigned int              current_source_sel : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int              current_source_sel : 2;
        unsigned int                  di_prim_type_q : 6;
        unsigned int            shift_amount_extract : 4;
        unsigned int         shift_amount_no_extract : 4;
        unsigned int              di_index_counter_q : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG14 {
    struct {
#if     defined(qLittleEndian)
        unsigned int              current_source_sel : 2;
        unsigned int                left_word_indx_q : 5;
        unsigned int                  input_data_cnt : 5;
        unsigned int                  input_data_lsw : 5;
        unsigned int                  input_data_msw : 5;
        unsigned int next_small_stride_shift_limit_q : 5;
        unsigned int current_small_stride_shift_limit_q : 5;
#else       /* !defined(qLittleEndian) */
        unsigned int current_small_stride_shift_limit_q : 5;
        unsigned int next_small_stride_shift_limit_q : 5;
        unsigned int                  input_data_msw : 5;
        unsigned int                  input_data_lsw : 5;
        unsigned int                  input_data_cnt : 5;
        unsigned int                left_word_indx_q : 5;
        unsigned int              current_source_sel : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG15 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   next_stride_q : 5;
        unsigned int                   next_stride_d : 5;
        unsigned int                 current_shift_q : 5;
        unsigned int                 current_shift_d : 5;
        unsigned int                current_stride_q : 5;
        unsigned int                current_stride_d : 5;
        unsigned int                    bgrp_trigger : 1;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                    bgrp_trigger : 1;
        unsigned int                current_stride_d : 5;
        unsigned int                current_stride_q : 5;
        unsigned int                 current_shift_d : 5;
        unsigned int                 current_shift_q : 5;
        unsigned int                   next_stride_d : 5;
        unsigned int                   next_stride_q : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG16 {
    struct {
#if     defined(qLittleEndian)
        unsigned int       bgrp_cull_fetch_fifo_full : 1;
        unsigned int      bgrp_cull_fetch_fifo_empty : 1;
        unsigned int        dma_bgrp_cull_fetch_read : 1;
        unsigned int         bgrp_cull_fetch_fifo_we : 1;
        unsigned int        bgrp_byte_mask_fifo_full : 1;
        unsigned int       bgrp_byte_mask_fifo_empty : 1;
        unsigned int        bgrp_byte_mask_fifo_re_q : 1;
        unsigned int          bgrp_byte_mask_fifo_we : 1;
        unsigned int              bgrp_dma_mask_kill : 1;
        unsigned int              bgrp_grp_bin_valid : 1;
        unsigned int                    rst_last_bit : 1;
        unsigned int                 current_state_q : 1;
        unsigned int                     old_state_q : 1;
        unsigned int                    old_state_en : 1;
        unsigned int                 prev_last_bit_q : 1;
        unsigned int                  dbl_last_bit_q : 1;
        unsigned int                last_bit_block_q : 1;
        unsigned int                ast_bit_block2_q : 1;
        unsigned int                  load_empty_reg : 1;
        unsigned int        bgrp_grp_byte_mask_rdata : 8;
        unsigned int     dma_bgrp_dma_data_fifo_rptr : 2;
        unsigned int    top_di_pre_fetch_cull_enable : 1;
        unsigned int        top_di_grp_cull_enable_q : 1;
        unsigned int                    bgrp_trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                    bgrp_trigger : 1;
        unsigned int        top_di_grp_cull_enable_q : 1;
        unsigned int    top_di_pre_fetch_cull_enable : 1;
        unsigned int     dma_bgrp_dma_data_fifo_rptr : 2;
        unsigned int        bgrp_grp_byte_mask_rdata : 8;
        unsigned int                  load_empty_reg : 1;
        unsigned int                ast_bit_block2_q : 1;
        unsigned int                last_bit_block_q : 1;
        unsigned int                  dbl_last_bit_q : 1;
        unsigned int                 prev_last_bit_q : 1;
        unsigned int                    old_state_en : 1;
        unsigned int                     old_state_q : 1;
        unsigned int                 current_state_q : 1;
        unsigned int                    rst_last_bit : 1;
        unsigned int              bgrp_grp_bin_valid : 1;
        unsigned int              bgrp_dma_mask_kill : 1;
        unsigned int          bgrp_byte_mask_fifo_we : 1;
        unsigned int        bgrp_byte_mask_fifo_re_q : 1;
        unsigned int       bgrp_byte_mask_fifo_empty : 1;
        unsigned int        bgrp_byte_mask_fifo_full : 1;
        unsigned int         bgrp_cull_fetch_fifo_we : 1;
        unsigned int        dma_bgrp_cull_fetch_read : 1;
        unsigned int      bgrp_cull_fetch_fifo_empty : 1;
        unsigned int       bgrp_cull_fetch_fifo_full : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG17 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     save_read_q : 1;
        unsigned int                   extend_read_q : 1;
        unsigned int                   grp_indx_size : 2;
        unsigned int                  cull_prim_true : 1;
        unsigned int                    reset_bit2_q : 1;
        unsigned int                    reset_bit1_q : 1;
        unsigned int               first_reg_first_q : 1;
        unsigned int                check_second_reg : 1;
        unsigned int                 check_first_reg : 1;
        unsigned int      bgrp_cull_fetch_fifo_wdata : 1;
        unsigned int         save_cull_fetch_data2_q : 1;
        unsigned int         save_cull_fetch_data1_q : 1;
        unsigned int          save_byte_mask_data2_q : 1;
        unsigned int          save_byte_mask_data1_q : 1;
        unsigned int                 to_second_reg_q : 1;
        unsigned int                 roll_over_msk_q : 1;
        unsigned int                   max_msk_ptr_q : 7;
        unsigned int                   min_msk_ptr_q : 7;
        unsigned int                    bgrp_trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                    bgrp_trigger : 1;
        unsigned int                   min_msk_ptr_q : 7;
        unsigned int                   max_msk_ptr_q : 7;
        unsigned int                 roll_over_msk_q : 1;
        unsigned int                 to_second_reg_q : 1;
        unsigned int          save_byte_mask_data1_q : 1;
        unsigned int          save_byte_mask_data2_q : 1;
        unsigned int         save_cull_fetch_data1_q : 1;
        unsigned int         save_cull_fetch_data2_q : 1;
        unsigned int      bgrp_cull_fetch_fifo_wdata : 1;
        unsigned int                 check_first_reg : 1;
        unsigned int                check_second_reg : 1;
        unsigned int               first_reg_first_q : 1;
        unsigned int                    reset_bit1_q : 1;
        unsigned int                    reset_bit2_q : 1;
        unsigned int                  cull_prim_true : 1;
        unsigned int                   grp_indx_size : 2;
        unsigned int                   extend_read_q : 1;
        unsigned int                     save_read_q : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG18 {
    struct {
#if     defined(qLittleEndian)
        unsigned int         dma_data_fifo_mem_raddr : 6;
        unsigned int         dma_data_fifo_mem_waddr : 6;
        unsigned int      dma_bgrp_byte_mask_fifo_re : 1;
        unsigned int     dma_bgrp_dma_data_fifo_rptr : 2;
        unsigned int                    dma_mem_full : 1;
        unsigned int                      dma_ram_re : 1;
        unsigned int                      dma_ram_we : 1;
        unsigned int                   dma_mem_empty : 1;
        unsigned int            dma_data_fifo_mem_re : 1;
        unsigned int            dma_data_fifo_mem_we : 1;
        unsigned int                    bin_mem_full : 1;
        unsigned int                      bin_ram_we : 1;
        unsigned int                      bin_ram_re : 1;
        unsigned int                   bin_mem_empty : 1;
        unsigned int                   start_bin_req : 1;
        unsigned int             fetch_cull_not_used : 1;
        unsigned int                    dma_req_xfer : 1;
        unsigned int              have_valid_bin_req : 1;
        unsigned int              have_valid_dma_req : 1;
        unsigned int     bgrp_dma_di_grp_cull_enable : 1;
        unsigned int bgrp_dma_di_pre_fetch_cull_enable : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int bgrp_dma_di_pre_fetch_cull_enable : 1;
        unsigned int     bgrp_dma_di_grp_cull_enable : 1;
        unsigned int              have_valid_dma_req : 1;
        unsigned int              have_valid_bin_req : 1;
        unsigned int                    dma_req_xfer : 1;
        unsigned int             fetch_cull_not_used : 1;
        unsigned int                   start_bin_req : 1;
        unsigned int                   bin_mem_empty : 1;
        unsigned int                      bin_ram_re : 1;
        unsigned int                      bin_ram_we : 1;
        unsigned int                    bin_mem_full : 1;
        unsigned int            dma_data_fifo_mem_we : 1;
        unsigned int            dma_data_fifo_mem_re : 1;
        unsigned int                   dma_mem_empty : 1;
        unsigned int                      dma_ram_we : 1;
        unsigned int                      dma_ram_re : 1;
        unsigned int                    dma_mem_full : 1;
        unsigned int     dma_bgrp_dma_data_fifo_rptr : 2;
        unsigned int      dma_bgrp_byte_mask_fifo_re : 1;
        unsigned int         dma_data_fifo_mem_waddr : 6;
        unsigned int         dma_data_fifo_mem_raddr : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG20 {
    struct {
#if     defined(qLittleEndian)
        unsigned int            prim_side_indx_valid : 1;
        unsigned int            indx_side_fifo_empty : 1;
        unsigned int               indx_side_fifo_re : 1;
        unsigned int               indx_side_fifo_we : 1;
        unsigned int             indx_side_fifo_full : 1;
        unsigned int               prim_buffer_empty : 1;
        unsigned int                  prim_buffer_re : 1;
        unsigned int                  prim_buffer_we : 1;
        unsigned int                prim_buffer_full : 1;
        unsigned int               indx_buffer_empty : 1;
        unsigned int                  indx_buffer_re : 1;
        unsigned int                  indx_buffer_we : 1;
        unsigned int                indx_buffer_full : 1;
        unsigned int                       hold_prim : 1;
        unsigned int                        sent_cnt : 4;
        unsigned int             start_of_vtx_vector : 1;
        unsigned int            clip_s_pre_hold_prim : 1;
        unsigned int            clip_p_pre_hold_prim : 1;
        unsigned int        buffered_prim_type_event : 5;
        unsigned int                     out_trigger : 1;
        unsigned int                                 : 5;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 5;
        unsigned int                     out_trigger : 1;
        unsigned int        buffered_prim_type_event : 5;
        unsigned int            clip_p_pre_hold_prim : 1;
        unsigned int            clip_s_pre_hold_prim : 1;
        unsigned int             start_of_vtx_vector : 1;
        unsigned int                        sent_cnt : 4;
        unsigned int                       hold_prim : 1;
        unsigned int                indx_buffer_full : 1;
        unsigned int                  indx_buffer_we : 1;
        unsigned int                  indx_buffer_re : 1;
        unsigned int               indx_buffer_empty : 1;
        unsigned int                prim_buffer_full : 1;
        unsigned int                  prim_buffer_we : 1;
        unsigned int                  prim_buffer_re : 1;
        unsigned int               prim_buffer_empty : 1;
        unsigned int             indx_side_fifo_full : 1;
        unsigned int               indx_side_fifo_we : 1;
        unsigned int               indx_side_fifo_re : 1;
        unsigned int            indx_side_fifo_empty : 1;
        unsigned int            prim_side_indx_valid : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_DEBUG_REG21 {
    struct {
#if     defined(qLittleEndian)
        unsigned int       null_terminate_vtx_vector : 1;
        unsigned int      prim_end_of_vtx_vect_flags : 3;
        unsigned int                 alloc_counter_q : 3;
        unsigned int         curr_slot_in_vtx_vect_q : 3;
        unsigned int               int_vtx_counter_q : 4;
        unsigned int         curr_dealloc_distance_q : 4;
        unsigned int                    new_packet_q : 1;
        unsigned int                  new_allocate_q : 1;
        unsigned int         num_new_unique_rel_indx : 2;
        unsigned int            inserted_null_prim_q : 1;
        unsigned int                insert_null_prim : 1;
        unsigned int           buffered_prim_eop_mux : 1;
        unsigned int           prim_buffer_empty_mux : 1;
        unsigned int            buffered_thread_size : 1;
        unsigned int                                 : 4;
        unsigned int                     out_trigger : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     out_trigger : 1;
        unsigned int                                 : 4;
        unsigned int            buffered_thread_size : 1;
        unsigned int           prim_buffer_empty_mux : 1;
        unsigned int           buffered_prim_eop_mux : 1;
        unsigned int                insert_null_prim : 1;
        unsigned int            inserted_null_prim_q : 1;
        unsigned int         num_new_unique_rel_indx : 2;
        unsigned int                  new_allocate_q : 1;
        unsigned int                    new_packet_q : 1;
        unsigned int         curr_dealloc_distance_q : 4;
        unsigned int               int_vtx_counter_q : 4;
        unsigned int         curr_slot_in_vtx_vect_q : 3;
        unsigned int                 alloc_counter_q : 3;
        unsigned int      prim_end_of_vtx_vect_flags : 3;
        unsigned int       null_terminate_vtx_vector : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_CRC_SQ_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                             CRC : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                             CRC : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_CRC_SQ_CTRL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                             CRC : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                             CRC : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER0_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER1_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER2_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER3_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER0_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER1_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER2_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER3_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER0_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER1_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER2_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union VGT_PERFCOUNTER3_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TC_CNTL_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   L2_INVALIDATE : 1;
        unsigned int                                 : 17;
        unsigned int                  TC_L2_HIT_MISS : 2;
        unsigned int                                 : 11;
        unsigned int                         TC_BUSY : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         TC_BUSY : 1;
        unsigned int                                 : 11;
        unsigned int                  TC_L2_HIT_MISS : 2;
        unsigned int                                 : 17;
        unsigned int                   L2_INVALIDATE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCR_CHICKEN {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           SPARE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           SPARE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_CHICKEN {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           SPARE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           SPARE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCM_CHICKEN {
    struct {
#if     defined(qLittleEndian)
        unsigned int TCO_READ_LATENCY_FIFO_PROG_DEPTH : 8;
        unsigned int                ETC_COLOR_ENDIAN : 1;
        unsigned int                           SPARE : 23;
#else       /* !defined(qLittleEndian) */
        unsigned int                           SPARE : 23;
        unsigned int                ETC_COLOR_ENDIAN : 1;
        unsigned int TCO_READ_LATENCY_FIFO_PROG_DEPTH : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCR_PERFCOUNTER0_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCR_PERFCOUNTER1_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCR_PERFCOUNTER0_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCR_PERFCOUNTER1_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCR_PERFCOUNTER0_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCR_PERFCOUNTER1_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TP_TC_CLKGATE_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  TP_BUSY_EXTEND : 3;
        unsigned int                  TC_BUSY_EXTEND : 3;
        unsigned int                                 : 26;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 26;
        unsigned int                  TC_BUSY_EXTEND : 3;
        unsigned int                  TP_BUSY_EXTEND : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TPC_CNTL_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  TPC_INPUT_BUSY : 1;
        unsigned int                TPC_TC_FIFO_BUSY : 1;
        unsigned int             TPC_STATE_FIFO_BUSY : 1;
        unsigned int             TPC_FETCH_FIFO_BUSY : 1;
        unsigned int            TPC_WALKER_PIPE_BUSY : 1;
        unsigned int              TPC_WALK_FIFO_BUSY : 1;
        unsigned int                 TPC_WALKER_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int           TPC_ALIGNER_PIPE_BUSY : 1;
        unsigned int             TPC_ALIGN_FIFO_BUSY : 1;
        unsigned int                TPC_ALIGNER_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                TPC_RR_FIFO_BUSY : 1;
        unsigned int             TPC_BLEND_PIPE_BUSY : 1;
        unsigned int               TPC_OUT_FIFO_BUSY : 1;
        unsigned int                  TPC_BLEND_BUSY : 1;
        unsigned int                       TF_TW_RTS : 1;
        unsigned int                 TF_TW_STATE_RTS : 1;
        unsigned int                                 : 1;
        unsigned int                       TF_TW_RTR : 1;
        unsigned int                       TW_TA_RTS : 1;
        unsigned int                    TW_TA_TT_RTS : 1;
        unsigned int                  TW_TA_LAST_RTS : 1;
        unsigned int                       TW_TA_RTR : 1;
        unsigned int                       TA_TB_RTS : 1;
        unsigned int                    TA_TB_TT_RTS : 1;
        unsigned int                                 : 1;
        unsigned int                       TA_TB_RTR : 1;
        unsigned int                       TA_TF_RTS : 1;
        unsigned int               TA_TF_TC_FIFO_REN : 1;
        unsigned int                       TP_SQ_DEC : 1;
        unsigned int                        TPC_BUSY : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                        TPC_BUSY : 1;
        unsigned int                       TP_SQ_DEC : 1;
        unsigned int               TA_TF_TC_FIFO_REN : 1;
        unsigned int                       TA_TF_RTS : 1;
        unsigned int                       TA_TB_RTR : 1;
        unsigned int                                 : 1;
        unsigned int                    TA_TB_TT_RTS : 1;
        unsigned int                       TA_TB_RTS : 1;
        unsigned int                       TW_TA_RTR : 1;
        unsigned int                  TW_TA_LAST_RTS : 1;
        unsigned int                    TW_TA_TT_RTS : 1;
        unsigned int                       TW_TA_RTS : 1;
        unsigned int                       TF_TW_RTR : 1;
        unsigned int                                 : 1;
        unsigned int                 TF_TW_STATE_RTS : 1;
        unsigned int                       TF_TW_RTS : 1;
        unsigned int                  TPC_BLEND_BUSY : 1;
        unsigned int               TPC_OUT_FIFO_BUSY : 1;
        unsigned int             TPC_BLEND_PIPE_BUSY : 1;
        unsigned int                TPC_RR_FIFO_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                TPC_ALIGNER_BUSY : 1;
        unsigned int             TPC_ALIGN_FIFO_BUSY : 1;
        unsigned int           TPC_ALIGNER_PIPE_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                 TPC_WALKER_BUSY : 1;
        unsigned int              TPC_WALK_FIFO_BUSY : 1;
        unsigned int            TPC_WALKER_PIPE_BUSY : 1;
        unsigned int             TPC_FETCH_FIFO_BUSY : 1;
        unsigned int             TPC_STATE_FIFO_BUSY : 1;
        unsigned int                TPC_TC_FIFO_BUSY : 1;
        unsigned int                  TPC_INPUT_BUSY : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TPC_DEBUG0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        LOD_CNTL : 2;
        unsigned int                          IC_CTR : 2;
        unsigned int                     WALKER_CNTL : 4;
        unsigned int                    ALIGNER_CNTL : 3;
        unsigned int                                 : 1;
        unsigned int             PREV_TC_STATE_VALID : 1;
        unsigned int                                 : 3;
        unsigned int                    WALKER_STATE : 10;
        unsigned int                   ALIGNER_STATE : 2;
        unsigned int                                 : 1;
        unsigned int                      REG_CLK_EN : 1;
        unsigned int                      TPC_CLK_EN : 1;
        unsigned int                    SQ_TP_WAKEUP : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                    SQ_TP_WAKEUP : 1;
        unsigned int                      TPC_CLK_EN : 1;
        unsigned int                      REG_CLK_EN : 1;
        unsigned int                                 : 1;
        unsigned int                   ALIGNER_STATE : 2;
        unsigned int                    WALKER_STATE : 10;
        unsigned int                                 : 3;
        unsigned int             PREV_TC_STATE_VALID : 1;
        unsigned int                                 : 1;
        unsigned int                    ALIGNER_CNTL : 3;
        unsigned int                     WALKER_CNTL : 4;
        unsigned int                          IC_CTR : 2;
        unsigned int                        LOD_CNTL : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TPC_DEBUG1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          UNUSED : 1;
        unsigned int                                 : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 31;
        unsigned int                          UNUSED : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TPC_CHICKEN {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 BLEND_PRECISION : 1;
        unsigned int                           SPARE : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                           SPARE : 31;
        unsigned int                 BLEND_PRECISION : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TP0_CNTL_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   TP_INPUT_BUSY : 1;
        unsigned int                     TP_LOD_BUSY : 1;
        unsigned int                TP_LOD_FIFO_BUSY : 1;
        unsigned int                    TP_ADDR_BUSY : 1;
        unsigned int              TP_ALIGN_FIFO_BUSY : 1;
        unsigned int                 TP_ALIGNER_BUSY : 1;
        unsigned int                 TP_TC_FIFO_BUSY : 1;
        unsigned int                 TP_RR_FIFO_BUSY : 1;
        unsigned int                   TP_FETCH_BUSY : 1;
        unsigned int                TP_CH_BLEND_BUSY : 1;
        unsigned int                      TP_TT_BUSY : 1;
        unsigned int                 TP_HICOLOR_BUSY : 1;
        unsigned int                   TP_BLEND_BUSY : 1;
        unsigned int                TP_OUT_FIFO_BUSY : 1;
        unsigned int                  TP_OUTPUT_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                       IN_LC_RTS : 1;
        unsigned int                       LC_LA_RTS : 1;
        unsigned int                       LA_FL_RTS : 1;
        unsigned int                       FL_TA_RTS : 1;
        unsigned int                       TA_FA_RTS : 1;
        unsigned int                    TA_FA_TT_RTS : 1;
        unsigned int                       FA_AL_RTS : 1;
        unsigned int                    FA_AL_TT_RTS : 1;
        unsigned int                       AL_TF_RTS : 1;
        unsigned int                    AL_TF_TT_RTS : 1;
        unsigned int                       TF_TB_RTS : 1;
        unsigned int                    TF_TB_TT_RTS : 1;
        unsigned int                       TB_TT_RTS : 1;
        unsigned int                  TB_TT_TT_RESET : 1;
        unsigned int                       TB_TO_RTS : 1;
        unsigned int                         TP_BUSY : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         TP_BUSY : 1;
        unsigned int                       TB_TO_RTS : 1;
        unsigned int                  TB_TT_TT_RESET : 1;
        unsigned int                       TB_TT_RTS : 1;
        unsigned int                    TF_TB_TT_RTS : 1;
        unsigned int                       TF_TB_RTS : 1;
        unsigned int                    AL_TF_TT_RTS : 1;
        unsigned int                       AL_TF_RTS : 1;
        unsigned int                    FA_AL_TT_RTS : 1;
        unsigned int                       FA_AL_RTS : 1;
        unsigned int                    TA_FA_TT_RTS : 1;
        unsigned int                       TA_FA_RTS : 1;
        unsigned int                       FL_TA_RTS : 1;
        unsigned int                       LA_FL_RTS : 1;
        unsigned int                       LC_LA_RTS : 1;
        unsigned int                       IN_LC_RTS : 1;
        unsigned int                                 : 1;
        unsigned int                  TP_OUTPUT_BUSY : 1;
        unsigned int                TP_OUT_FIFO_BUSY : 1;
        unsigned int                   TP_BLEND_BUSY : 1;
        unsigned int                 TP_HICOLOR_BUSY : 1;
        unsigned int                      TP_TT_BUSY : 1;
        unsigned int                TP_CH_BLEND_BUSY : 1;
        unsigned int                   TP_FETCH_BUSY : 1;
        unsigned int                 TP_RR_FIFO_BUSY : 1;
        unsigned int                 TP_TC_FIFO_BUSY : 1;
        unsigned int                 TP_ALIGNER_BUSY : 1;
        unsigned int              TP_ALIGN_FIFO_BUSY : 1;
        unsigned int                    TP_ADDR_BUSY : 1;
        unsigned int                TP_LOD_FIFO_BUSY : 1;
        unsigned int                     TP_LOD_BUSY : 1;
        unsigned int                   TP_INPUT_BUSY : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TP0_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      Q_LOD_CNTL : 2;
        unsigned int                                 : 1;
        unsigned int                  Q_SQ_TP_WAKEUP : 1;
        unsigned int            FL_TA_ADDRESSER_CNTL : 17;
        unsigned int                      REG_CLK_EN : 1;
        unsigned int                     PERF_CLK_EN : 1;
        unsigned int                       TP_CLK_EN : 1;
        unsigned int                   Q_WALKER_CNTL : 4;
        unsigned int                  Q_ALIGNER_CNTL : 3;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                  Q_ALIGNER_CNTL : 3;
        unsigned int                   Q_WALKER_CNTL : 4;
        unsigned int                       TP_CLK_EN : 1;
        unsigned int                     PERF_CLK_EN : 1;
        unsigned int                      REG_CLK_EN : 1;
        unsigned int            FL_TA_ADDRESSER_CNTL : 17;
        unsigned int                  Q_SQ_TP_WAKEUP : 1;
        unsigned int                                 : 1;
        unsigned int                      Q_LOD_CNTL : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TP0_CHICKEN {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         TT_MODE : 1;
        unsigned int             VFETCH_ADDRESS_MODE : 1;
        unsigned int                           SPARE : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                           SPARE : 30;
        unsigned int             VFETCH_ADDRESS_MODE : 1;
        unsigned int                         TT_MODE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TP0_PERFCOUNTER0_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TP0_PERFCOUNTER0_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TP0_PERFCOUNTER0_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TP0_PERFCOUNTER1_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TP0_PERFCOUNTER1_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TP0_PERFCOUNTER1_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCM_PERFCOUNTER0_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCM_PERFCOUNTER1_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCM_PERFCOUNTER0_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCM_PERFCOUNTER1_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCM_PERFCOUNTER0_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCM_PERFCOUNTER1_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER0_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER1_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER2_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER3_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER4_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER5_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER6_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER7_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER8_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER9_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER10_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER11_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              PERFCOUNTER_SELECT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int              PERFCOUNTER_SELECT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER0_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER1_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER2_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER3_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER4_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER5_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER6_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER7_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER8_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER9_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER10_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER11_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERFCOUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERFCOUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER0_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER1_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER2_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER3_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER4_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER5_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER6_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER7_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER8_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER9_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER10_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_PERFCOUNTER11_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERFCOUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PERFCOUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCF_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 6;
        unsigned int                   not_MH_TC_rtr : 1;
        unsigned int                      TC_MH_send : 1;
        unsigned int                     not_FG0_rtr : 1;
        unsigned int                                 : 3;
        unsigned int                 not_TCB_TCO_rtr : 1;
        unsigned int                    TCB_ff_stall : 1;
        unsigned int                  TCB_miss_stall : 1;
        unsigned int                   TCA_TCB_stall : 1;
        unsigned int                       PF0_stall : 1;
        unsigned int                                 : 3;
        unsigned int                        TP0_full : 1;
        unsigned int                                 : 3;
        unsigned int                        TPC_full : 1;
        unsigned int                     not_TPC_rtr : 1;
        unsigned int                   tca_state_rts : 1;
        unsigned int                         tca_rts : 1;
        unsigned int                                 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 4;
        unsigned int                         tca_rts : 1;
        unsigned int                   tca_state_rts : 1;
        unsigned int                     not_TPC_rtr : 1;
        unsigned int                        TPC_full : 1;
        unsigned int                                 : 3;
        unsigned int                        TP0_full : 1;
        unsigned int                                 : 3;
        unsigned int                       PF0_stall : 1;
        unsigned int                   TCA_TCB_stall : 1;
        unsigned int                  TCB_miss_stall : 1;
        unsigned int                    TCB_ff_stall : 1;
        unsigned int                 not_TCB_TCO_rtr : 1;
        unsigned int                                 : 3;
        unsigned int                     not_FG0_rtr : 1;
        unsigned int                      TC_MH_send : 1;
        unsigned int                   not_MH_TC_rtr : 1;
        unsigned int                                 : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCA_FIFO_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        tp0_full : 1;
        unsigned int                                 : 3;
        unsigned int                        tpc_full : 1;
        unsigned int                   load_tpc_fifo : 1;
        unsigned int                   load_tp_fifos : 1;
        unsigned int                         FW_full : 1;
        unsigned int                     not_FW_rtr0 : 1;
        unsigned int                                 : 3;
        unsigned int                         FW_rts0 : 1;
        unsigned int                                 : 3;
        unsigned int                  not_FW_tpc_rtr : 1;
        unsigned int                      FW_tpc_rts : 1;
        unsigned int                                 : 14;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 14;
        unsigned int                      FW_tpc_rts : 1;
        unsigned int                  not_FW_tpc_rtr : 1;
        unsigned int                                 : 3;
        unsigned int                         FW_rts0 : 1;
        unsigned int                                 : 3;
        unsigned int                     not_FW_rtr0 : 1;
        unsigned int                         FW_full : 1;
        unsigned int                   load_tp_fifos : 1;
        unsigned int                   load_tpc_fifo : 1;
        unsigned int                        tpc_full : 1;
        unsigned int                                 : 3;
        unsigned int                        tp0_full : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCA_PROBE_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int               ProbeFilter_stall : 1;
        unsigned int                                 : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 31;
        unsigned int               ProbeFilter_stall : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCA_TPC_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                captue_state_rts : 1;
        unsigned int                 capture_tca_rts : 1;
        unsigned int                                 : 18;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 18;
        unsigned int                 capture_tca_rts : 1;
        unsigned int                captue_state_rts : 1;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCB_CORE_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       access512 : 1;
        unsigned int                           tiled : 1;
        unsigned int                                 : 2;
        unsigned int                          opcode : 3;
        unsigned int                                 : 1;
        unsigned int                          format : 6;
        unsigned int                                 : 2;
        unsigned int                   sector_format : 5;
        unsigned int                                 : 3;
        unsigned int                sector_format512 : 3;
        unsigned int                                 : 5;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 5;
        unsigned int                sector_format512 : 3;
        unsigned int                                 : 3;
        unsigned int                   sector_format : 5;
        unsigned int                                 : 2;
        unsigned int                          format : 6;
        unsigned int                                 : 1;
        unsigned int                          opcode : 3;
        unsigned int                                 : 2;
        unsigned int                           tiled : 1;
        unsigned int                       access512 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCB_TAG0_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  mem_read_cycle : 10;
        unsigned int                                 : 2;
        unsigned int                tag_access_cycle : 9;
        unsigned int                                 : 2;
        unsigned int                      miss_stall : 1;
        unsigned int                  num_feee_lines : 5;
        unsigned int                      max_misses : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                      max_misses : 3;
        unsigned int                  num_feee_lines : 5;
        unsigned int                      miss_stall : 1;
        unsigned int                                 : 2;
        unsigned int                tag_access_cycle : 9;
        unsigned int                                 : 2;
        unsigned int                  mem_read_cycle : 10;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCB_TAG1_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  mem_read_cycle : 10;
        unsigned int                                 : 2;
        unsigned int                tag_access_cycle : 9;
        unsigned int                                 : 2;
        unsigned int                      miss_stall : 1;
        unsigned int                  num_feee_lines : 5;
        unsigned int                      max_misses : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                      max_misses : 3;
        unsigned int                  num_feee_lines : 5;
        unsigned int                      miss_stall : 1;
        unsigned int                                 : 2;
        unsigned int                tag_access_cycle : 9;
        unsigned int                                 : 2;
        unsigned int                  mem_read_cycle : 10;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCB_TAG2_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  mem_read_cycle : 10;
        unsigned int                                 : 2;
        unsigned int                tag_access_cycle : 9;
        unsigned int                                 : 2;
        unsigned int                      miss_stall : 1;
        unsigned int                  num_feee_lines : 5;
        unsigned int                      max_misses : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                      max_misses : 3;
        unsigned int                  num_feee_lines : 5;
        unsigned int                      miss_stall : 1;
        unsigned int                                 : 2;
        unsigned int                tag_access_cycle : 9;
        unsigned int                                 : 2;
        unsigned int                  mem_read_cycle : 10;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCB_TAG3_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  mem_read_cycle : 10;
        unsigned int                                 : 2;
        unsigned int                tag_access_cycle : 9;
        unsigned int                                 : 2;
        unsigned int                      miss_stall : 1;
        unsigned int                  num_feee_lines : 5;
        unsigned int                      max_misses : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                      max_misses : 3;
        unsigned int                  num_feee_lines : 5;
        unsigned int                      miss_stall : 1;
        unsigned int                                 : 2;
        unsigned int                tag_access_cycle : 9;
        unsigned int                                 : 2;
        unsigned int                  mem_read_cycle : 10;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCB_FETCH_GEN_SECTOR_WALKER0_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       left_done : 1;
        unsigned int                                 : 1;
        unsigned int                  fg0_sends_left : 1;
        unsigned int                                 : 1;
        unsigned int         one_sector_to_go_left_q : 1;
        unsigned int                no_sectors_to_go : 1;
        unsigned int                     update_left : 1;
        unsigned int        sector_mask_left_count_q : 5;
        unsigned int              sector_mask_left_q : 16;
        unsigned int                    valid_left_q : 1;
        unsigned int                                 : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 3;
        unsigned int                    valid_left_q : 1;
        unsigned int              sector_mask_left_q : 16;
        unsigned int        sector_mask_left_count_q : 5;
        unsigned int                     update_left : 1;
        unsigned int                no_sectors_to_go : 1;
        unsigned int         one_sector_to_go_left_q : 1;
        unsigned int                                 : 1;
        unsigned int                  fg0_sends_left : 1;
        unsigned int                                 : 1;
        unsigned int                       left_done : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCB_FETCH_GEN_WALKER_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 4;
        unsigned int                   quad_sel_left : 2;
        unsigned int                    set_sel_left : 2;
        unsigned int                                 : 3;
        unsigned int                   right_eq_left : 1;
        unsigned int                   ff_fg_type512 : 3;
        unsigned int                            busy : 1;
        unsigned int                setquads_to_send : 4;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                setquads_to_send : 4;
        unsigned int                            busy : 1;
        unsigned int                   ff_fg_type512 : 3;
        unsigned int                   right_eq_left : 1;
        unsigned int                                 : 3;
        unsigned int                    set_sel_left : 2;
        unsigned int                   quad_sel_left : 2;
        unsigned int                                 : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCB_FETCH_GEN_PIPE0_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     tc0_arb_rts : 1;
        unsigned int                                 : 1;
        unsigned int                      ga_out_rts : 1;
        unsigned int                                 : 1;
        unsigned int                   tc_arb_format : 12;
        unsigned int                tc_arb_fmsopcode : 5;
        unsigned int             tc_arb_request_type : 2;
        unsigned int                            busy : 1;
        unsigned int                        fgo_busy : 1;
        unsigned int                         ga_busy : 1;
        unsigned int                        mc_sel_q : 2;
        unsigned int                         valid_q : 1;
        unsigned int                                 : 1;
        unsigned int                         arb_RTR : 1;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                         arb_RTR : 1;
        unsigned int                                 : 1;
        unsigned int                         valid_q : 1;
        unsigned int                        mc_sel_q : 2;
        unsigned int                         ga_busy : 1;
        unsigned int                        fgo_busy : 1;
        unsigned int                            busy : 1;
        unsigned int             tc_arb_request_type : 2;
        unsigned int                tc_arb_fmsopcode : 5;
        unsigned int                   tc_arb_format : 12;
        unsigned int                                 : 1;
        unsigned int                      ga_out_rts : 1;
        unsigned int                                 : 1;
        unsigned int                     tc0_arb_rts : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCD_INPUT0_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 16;
        unsigned int                           empty : 1;
        unsigned int                            full : 1;
        unsigned int                                 : 2;
        unsigned int                        valid_q1 : 1;
        unsigned int                          cnt_q1 : 2;
        unsigned int                    last_send_q1 : 1;
        unsigned int                         ip_send : 1;
        unsigned int                  ipbuf_dxt_send : 1;
        unsigned int                      ipbuf_busy : 1;
        unsigned int                                 : 5;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 5;
        unsigned int                      ipbuf_busy : 1;
        unsigned int                  ipbuf_dxt_send : 1;
        unsigned int                         ip_send : 1;
        unsigned int                    last_send_q1 : 1;
        unsigned int                          cnt_q1 : 2;
        unsigned int                        valid_q1 : 1;
        unsigned int                                 : 2;
        unsigned int                            full : 1;
        unsigned int                           empty : 1;
        unsigned int                                 : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCD_DEGAMMA_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int             dgmm_ftfconv_dgmmen : 2;
        unsigned int                 dgmm_ctrl_dgmm8 : 1;
        unsigned int             dgmm_ctrl_last_send : 1;
        unsigned int                  dgmm_ctrl_send : 1;
        unsigned int                      dgmm_stall : 1;
        unsigned int                     dgmm_pstate : 1;
        unsigned int                                 : 25;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 25;
        unsigned int                     dgmm_pstate : 1;
        unsigned int                      dgmm_stall : 1;
        unsigned int                  dgmm_ctrl_send : 1;
        unsigned int             dgmm_ctrl_last_send : 1;
        unsigned int                 dgmm_ctrl_dgmm8 : 1;
        unsigned int             dgmm_ftfconv_dgmmen : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCD_DXTMUX_SCTARB_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 9;
        unsigned int                          pstate : 1;
        unsigned int                      sctrmx_rtr : 1;
        unsigned int                        dxtc_rtr : 1;
        unsigned int                                 : 3;
        unsigned int            sctrarb_multcyl_send : 1;
        unsigned int             sctrmx0_sctrarb_rts : 1;
        unsigned int                                 : 3;
        unsigned int               dxtc_sctrarb_send : 1;
        unsigned int                                 : 6;
        unsigned int           dxtc_dgmmpd_last_send : 1;
        unsigned int                dxtc_dgmmpd_send : 1;
        unsigned int                   dcmp_mux_send : 1;
        unsigned int                                 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 2;
        unsigned int                   dcmp_mux_send : 1;
        unsigned int                dxtc_dgmmpd_send : 1;
        unsigned int           dxtc_dgmmpd_last_send : 1;
        unsigned int                                 : 6;
        unsigned int               dxtc_sctrarb_send : 1;
        unsigned int                                 : 3;
        unsigned int             sctrmx0_sctrarb_rts : 1;
        unsigned int            sctrarb_multcyl_send : 1;
        unsigned int                                 : 3;
        unsigned int                        dxtc_rtr : 1;
        unsigned int                      sctrmx_rtr : 1;
        unsigned int                          pstate : 1;
        unsigned int                                 : 9;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCD_DXTC_ARB_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 4;
        unsigned int                        n0_stall : 1;
        unsigned int                          pstate : 1;
        unsigned int            arb_dcmp01_last_send : 1;
        unsigned int                  arb_dcmp01_cnt : 2;
        unsigned int               arb_dcmp01_sector : 3;
        unsigned int            arb_dcmp01_cacheline : 6;
        unsigned int               arb_dcmp01_format : 12;
        unsigned int                 arb_dcmp01_send : 1;
        unsigned int                 n0_dxt2_4_types : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                 n0_dxt2_4_types : 1;
        unsigned int                 arb_dcmp01_send : 1;
        unsigned int               arb_dcmp01_format : 12;
        unsigned int            arb_dcmp01_cacheline : 6;
        unsigned int               arb_dcmp01_sector : 3;
        unsigned int                  arb_dcmp01_cnt : 2;
        unsigned int            arb_dcmp01_last_send : 1;
        unsigned int                          pstate : 1;
        unsigned int                        n0_stall : 1;
        unsigned int                                 : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCD_STALLS_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 10;
        unsigned int         not_multcyl_sctrarb_rtr : 1;
        unsigned int         not_sctrmx0_sctrarb_rtr : 1;
        unsigned int                                 : 5;
        unsigned int               not_dcmp0_arb_rtr : 1;
        unsigned int             not_dgmmpd_dxtc_rtr : 1;
        unsigned int                not_mux_dcmp_rtr : 1;
        unsigned int                                 : 11;
        unsigned int                not_incoming_rtr : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                not_incoming_rtr : 1;
        unsigned int                                 : 11;
        unsigned int                not_mux_dcmp_rtr : 1;
        unsigned int             not_dgmmpd_dxtc_rtr : 1;
        unsigned int               not_dcmp0_arb_rtr : 1;
        unsigned int                                 : 5;
        unsigned int         not_sctrmx0_sctrarb_rtr : 1;
        unsigned int         not_multcyl_sctrarb_rtr : 1;
        unsigned int                                 : 10;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCO_STALLS_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 5;
        unsigned int                quad0_sg_crd_RTR : 1;
        unsigned int                 quad0_rl_sg_RTR : 1;
        unsigned int             quad0_TCO_TCB_rtr_d : 1;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int             quad0_TCO_TCB_rtr_d : 1;
        unsigned int                 quad0_rl_sg_RTR : 1;
        unsigned int                quad0_sg_crd_RTR : 1;
        unsigned int                                 : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCO_QUAD0_DEBUG0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int             rl_sg_sector_format : 8;
        unsigned int             rl_sg_end_of_sample : 1;
        unsigned int                       rl_sg_rtr : 1;
        unsigned int                       rl_sg_rts : 1;
        unsigned int            sg_crd_end_of_sample : 1;
        unsigned int                      sg_crd_rtr : 1;
        unsigned int                      sg_crd_rts : 1;
        unsigned int                                 : 2;
        unsigned int                 stageN1_valid_q : 1;
        unsigned int                                 : 7;
        unsigned int                    read_cache_q : 1;
        unsigned int                  cache_read_RTR : 1;
        unsigned int        all_sectors_written_set3 : 1;
        unsigned int        all_sectors_written_set2 : 1;
        unsigned int        all_sectors_written_set1 : 1;
        unsigned int        all_sectors_written_set0 : 1;
        unsigned int                            busy : 1;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                            busy : 1;
        unsigned int        all_sectors_written_set0 : 1;
        unsigned int        all_sectors_written_set1 : 1;
        unsigned int        all_sectors_written_set2 : 1;
        unsigned int        all_sectors_written_set3 : 1;
        unsigned int                  cache_read_RTR : 1;
        unsigned int                    read_cache_q : 1;
        unsigned int                                 : 7;
        unsigned int                 stageN1_valid_q : 1;
        unsigned int                                 : 2;
        unsigned int                      sg_crd_rts : 1;
        unsigned int                      sg_crd_rtr : 1;
        unsigned int            sg_crd_end_of_sample : 1;
        unsigned int                       rl_sg_rts : 1;
        unsigned int                       rl_sg_rtr : 1;
        unsigned int             rl_sg_end_of_sample : 1;
        unsigned int             rl_sg_sector_format : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union TCO_QUAD0_DEBUG1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       fifo_busy : 1;
        unsigned int                           empty : 1;
        unsigned int                            full : 1;
        unsigned int                    write_enable : 1;
        unsigned int                  fifo_write_ptr : 7;
        unsigned int                   fifo_read_ptr : 7;
        unsigned int                                 : 2;
        unsigned int                 cache_read_busy : 1;
        unsigned int               latency_fifo_busy : 1;
        unsigned int                 input_quad_busy : 1;
        unsigned int              tco_quad_pipe_busy : 1;
        unsigned int                   TCB_TCO_rtr_d : 1;
        unsigned int                   TCB_TCO_xfc_q : 1;
        unsigned int                       rl_sg_rtr : 1;
        unsigned int                       rl_sg_rts : 1;
        unsigned int                      sg_crd_rtr : 1;
        unsigned int                      sg_crd_rts : 1;
        unsigned int                TCO_TCB_read_xfc : 1;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                TCO_TCB_read_xfc : 1;
        unsigned int                      sg_crd_rts : 1;
        unsigned int                      sg_crd_rtr : 1;
        unsigned int                       rl_sg_rts : 1;
        unsigned int                       rl_sg_rtr : 1;
        unsigned int                   TCB_TCO_xfc_q : 1;
        unsigned int                   TCB_TCO_rtr_d : 1;
        unsigned int              tco_quad_pipe_busy : 1;
        unsigned int                 input_quad_busy : 1;
        unsigned int               latency_fifo_busy : 1;
        unsigned int                 cache_read_busy : 1;
        unsigned int                                 : 2;
        unsigned int                   fifo_read_ptr : 7;
        unsigned int                  fifo_write_ptr : 7;
        unsigned int                    write_enable : 1;
        unsigned int                            full : 1;
        unsigned int                           empty : 1;
        unsigned int                       fifo_busy : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_GPR_MANAGEMENT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     REG_DYNAMIC : 1;
        unsigned int                                 : 3;
        unsigned int                    REG_SIZE_PIX : 7;
        unsigned int                                 : 1;
        unsigned int                    REG_SIZE_VTX : 7;
        unsigned int                                 : 13;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 13;
        unsigned int                    REG_SIZE_VTX : 7;
        unsigned int                                 : 1;
        unsigned int                    REG_SIZE_PIX : 7;
        unsigned int                                 : 3;
        unsigned int                     REG_DYNAMIC : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FLOW_CONTROL {
    struct {
#if     defined(qLittleEndian)
        unsigned int        INPUT_ARBITRATION_POLICY : 2;
        unsigned int                                 : 2;
        unsigned int                      ONE_THREAD : 1;
        unsigned int                                 : 3;
        unsigned int                         ONE_ALU : 1;
        unsigned int                                 : 3;
        unsigned int                      CF_WR_BASE : 4;
        unsigned int                        NO_PV_PS : 1;
        unsigned int                    NO_LOOP_EXIT : 1;
        unsigned int               NO_CEXEC_OPTIMIZE : 1;
        unsigned int      TEXTURE_ARBITRATION_POLICY : 2;
        unsigned int           VC_ARBITRATION_POLICY : 1;
        unsigned int          ALU_ARBITRATION_POLICY : 1;
        unsigned int                    NO_ARB_EJECT : 1;
        unsigned int                    NO_CFS_EJECT : 1;
        unsigned int                POS_EXP_PRIORITY : 1;
        unsigned int     NO_EARLY_THREAD_TERMINATION : 1;
        unsigned int         PS_PREFETCH_COLOR_ALLOC : 1;
        unsigned int                                 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 4;
        unsigned int         PS_PREFETCH_COLOR_ALLOC : 1;
        unsigned int     NO_EARLY_THREAD_TERMINATION : 1;
        unsigned int                POS_EXP_PRIORITY : 1;
        unsigned int                    NO_CFS_EJECT : 1;
        unsigned int                    NO_ARB_EJECT : 1;
        unsigned int          ALU_ARBITRATION_POLICY : 1;
        unsigned int           VC_ARBITRATION_POLICY : 1;
        unsigned int      TEXTURE_ARBITRATION_POLICY : 2;
        unsigned int               NO_CEXEC_OPTIMIZE : 1;
        unsigned int                    NO_LOOP_EXIT : 1;
        unsigned int                        NO_PV_PS : 1;
        unsigned int                      CF_WR_BASE : 4;
        unsigned int                                 : 3;
        unsigned int                         ONE_ALU : 1;
        unsigned int                                 : 3;
        unsigned int                      ONE_THREAD : 1;
        unsigned int                                 : 2;
        unsigned int        INPUT_ARBITRATION_POLICY : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INST_STORE_MANAGMENT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   INST_BASE_PIX : 12;
        unsigned int                                 : 4;
        unsigned int                   INST_BASE_VTX : 12;
        unsigned int                                 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 4;
        unsigned int                   INST_BASE_VTX : 12;
        unsigned int                                 : 4;
        unsigned int                   INST_BASE_PIX : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_RESOURCE_MANAGMENT {
    struct {
#if     defined(qLittleEndian)
        unsigned int          VTX_THREAD_BUF_ENTRIES : 8;
        unsigned int          PIX_THREAD_BUF_ENTRIES : 8;
        unsigned int              EXPORT_BUF_ENTRIES : 9;
        unsigned int                                 : 7;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 7;
        unsigned int              EXPORT_BUF_ENTRIES : 9;
        unsigned int          PIX_THREAD_BUF_ENTRIES : 8;
        unsigned int          VTX_THREAD_BUF_ENTRIES : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_EO_RT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 EO_CONSTANTS_RT : 8;
        unsigned int                                 : 8;
        unsigned int                    EO_TSTATE_RT : 8;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                    EO_TSTATE_RT : 8;
        unsigned int                                 : 8;
        unsigned int                 EO_CONSTANTS_RT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_MISC {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  DB_ALUCST_SIZE : 11;
        unsigned int                                 : 1;
        unsigned int                  DB_TSTATE_SIZE : 8;
        unsigned int                     DB_READ_CTX : 1;
        unsigned int                        RESERVED : 2;
        unsigned int                  DB_READ_MEMORY : 2;
        unsigned int                 DB_WEN_MEMORY_0 : 1;
        unsigned int                 DB_WEN_MEMORY_1 : 1;
        unsigned int                 DB_WEN_MEMORY_2 : 1;
        unsigned int                 DB_WEN_MEMORY_3 : 1;
        unsigned int                                 : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 3;
        unsigned int                 DB_WEN_MEMORY_3 : 1;
        unsigned int                 DB_WEN_MEMORY_2 : 1;
        unsigned int                 DB_WEN_MEMORY_1 : 1;
        unsigned int                 DB_WEN_MEMORY_0 : 1;
        unsigned int                  DB_READ_MEMORY : 2;
        unsigned int                        RESERVED : 2;
        unsigned int                     DB_READ_CTX : 1;
        unsigned int                  DB_TSTATE_SIZE : 8;
        unsigned int                                 : 1;
        unsigned int                  DB_ALUCST_SIZE : 11;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_ACTIVITY_METER_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        TIMEBASE : 8;
        unsigned int                   THRESHOLD_LOW : 8;
        unsigned int                  THRESHOLD_HIGH : 8;
        unsigned int                           SPARE : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                           SPARE : 8;
        unsigned int                  THRESHOLD_HIGH : 8;
        unsigned int                   THRESHOLD_LOW : 8;
        unsigned int                        TIMEBASE : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_ACTIVITY_METER_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    PERCENT_BUSY : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                    PERCENT_BUSY : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INPUT_ARB_PRIORITY {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PC_AVAIL_WEIGHT : 3;
        unsigned int                   PC_AVAIL_SIGN : 1;
        unsigned int                 SX_AVAIL_WEIGHT : 3;
        unsigned int                   SX_AVAIL_SIGN : 1;
        unsigned int                       THRESHOLD : 10;
        unsigned int                                 : 14;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 14;
        unsigned int                       THRESHOLD : 10;
        unsigned int                   SX_AVAIL_SIGN : 1;
        unsigned int                 SX_AVAIL_WEIGHT : 3;
        unsigned int                   PC_AVAIL_SIGN : 1;
        unsigned int                 PC_AVAIL_WEIGHT : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_THREAD_ARB_PRIORITY {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PC_AVAIL_WEIGHT : 3;
        unsigned int                   PC_AVAIL_SIGN : 1;
        unsigned int                 SX_AVAIL_WEIGHT : 3;
        unsigned int                   SX_AVAIL_SIGN : 1;
        unsigned int                       THRESHOLD : 10;
        unsigned int                        RESERVED : 2;
        unsigned int            VS_PRIORITIZE_SERIAL : 1;
        unsigned int            PS_PRIORITIZE_SERIAL : 1;
        unsigned int      USE_SERIAL_COUNT_THRESHOLD : 1;
        unsigned int                                 : 9;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 9;
        unsigned int      USE_SERIAL_COUNT_THRESHOLD : 1;
        unsigned int            PS_PRIORITIZE_SERIAL : 1;
        unsigned int            VS_PRIORITIZE_SERIAL : 1;
        unsigned int                        RESERVED : 2;
        unsigned int                       THRESHOLD : 10;
        unsigned int                   SX_AVAIL_SIGN : 1;
        unsigned int                 SX_AVAIL_WEIGHT : 3;
        unsigned int                   PC_AVAIL_SIGN : 1;
        unsigned int                 PC_AVAIL_WEIGHT : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_VS_WATCHDOG_TIMER {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          ENABLE : 1;
        unsigned int                   TIMEOUT_COUNT : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                   TIMEOUT_COUNT : 31;
        unsigned int                          ENABLE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PS_WATCHDOG_TIMER {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          ENABLE : 1;
        unsigned int                   TIMEOUT_COUNT : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                   TIMEOUT_COUNT : 31;
        unsigned int                          ENABLE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INT_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                PS_WATCHDOG_MASK : 1;
        unsigned int                VS_WATCHDOG_MASK : 1;
        unsigned int                                 : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 30;
        unsigned int                VS_WATCHDOG_MASK : 1;
        unsigned int                PS_WATCHDOG_MASK : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INT_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int             PS_WATCHDOG_TIMEOUT : 1;
        unsigned int             VS_WATCHDOG_TIMEOUT : 1;
        unsigned int                                 : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 30;
        unsigned int             VS_WATCHDOG_TIMEOUT : 1;
        unsigned int             PS_WATCHDOG_TIMEOUT : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INT_ACK {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PS_WATCHDOG_ACK : 1;
        unsigned int                 VS_WATCHDOG_ACK : 1;
        unsigned int                                 : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 30;
        unsigned int                 VS_WATCHDOG_ACK : 1;
        unsigned int                 PS_WATCHDOG_ACK : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_INPUT_FSM {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       VC_VSR_LD : 3;
        unsigned int                        RESERVED : 1;
        unsigned int                       VC_GPR_LD : 4;
        unsigned int                         PC_PISM : 3;
        unsigned int                       RESERVED1 : 1;
        unsigned int                           PC_AS : 3;
        unsigned int                   PC_INTERP_CNT : 5;
        unsigned int                     PC_GPR_SIZE : 8;
        unsigned int                                 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 4;
        unsigned int                     PC_GPR_SIZE : 8;
        unsigned int                   PC_INTERP_CNT : 5;
        unsigned int                           PC_AS : 3;
        unsigned int                       RESERVED1 : 1;
        unsigned int                         PC_PISM : 3;
        unsigned int                       VC_GPR_LD : 4;
        unsigned int                        RESERVED : 1;
        unsigned int                       VC_VSR_LD : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_CONST_MGR_FSM {
    struct {
#if     defined(qLittleEndian)
        unsigned int           TEX_CONST_EVENT_STATE : 5;
        unsigned int                       RESERVED1 : 3;
        unsigned int           ALU_CONST_EVENT_STATE : 5;
        unsigned int                       RESERVED2 : 3;
        unsigned int            ALU_CONST_CNTX_VALID : 2;
        unsigned int            TEX_CONST_CNTX_VALID : 2;
        unsigned int            CNTX0_VTX_EVENT_DONE : 1;
        unsigned int            CNTX0_PIX_EVENT_DONE : 1;
        unsigned int            CNTX1_VTX_EVENT_DONE : 1;
        unsigned int            CNTX1_PIX_EVENT_DONE : 1;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int            CNTX1_PIX_EVENT_DONE : 1;
        unsigned int            CNTX1_VTX_EVENT_DONE : 1;
        unsigned int            CNTX0_PIX_EVENT_DONE : 1;
        unsigned int            CNTX0_VTX_EVENT_DONE : 1;
        unsigned int            TEX_CONST_CNTX_VALID : 2;
        unsigned int            ALU_CONST_CNTX_VALID : 2;
        unsigned int                       RESERVED2 : 3;
        unsigned int           ALU_CONST_EVENT_STATE : 5;
        unsigned int                       RESERVED1 : 3;
        unsigned int           TEX_CONST_EVENT_STATE : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_TP_FSM {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           EX_TP : 3;
        unsigned int                       RESERVED0 : 1;
        unsigned int                           CF_TP : 4;
        unsigned int                           IF_TP : 3;
        unsigned int                       RESERVED1 : 1;
        unsigned int                          TIS_TP : 2;
        unsigned int                       RESERVED2 : 2;
        unsigned int                           GS_TP : 2;
        unsigned int                       RESERVED3 : 2;
        unsigned int                          FCR_TP : 2;
        unsigned int                       RESERVED4 : 2;
        unsigned int                          FCS_TP : 2;
        unsigned int                       RESERVED5 : 2;
        unsigned int                       ARB_TR_TP : 3;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                       ARB_TR_TP : 3;
        unsigned int                       RESERVED5 : 2;
        unsigned int                          FCS_TP : 2;
        unsigned int                       RESERVED4 : 2;
        unsigned int                          FCR_TP : 2;
        unsigned int                       RESERVED3 : 2;
        unsigned int                           GS_TP : 2;
        unsigned int                       RESERVED2 : 2;
        unsigned int                          TIS_TP : 2;
        unsigned int                       RESERVED1 : 1;
        unsigned int                           IF_TP : 3;
        unsigned int                           CF_TP : 4;
        unsigned int                       RESERVED0 : 1;
        unsigned int                           EX_TP : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_FSM_ALU_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        EX_ALU_0 : 3;
        unsigned int                       RESERVED0 : 1;
        unsigned int                        CF_ALU_0 : 4;
        unsigned int                        IF_ALU_0 : 3;
        unsigned int                       RESERVED1 : 1;
        unsigned int                       DU1_ALU_0 : 3;
        unsigned int                       RESERVED2 : 1;
        unsigned int                       DU0_ALU_0 : 3;
        unsigned int                       RESERVED3 : 1;
        unsigned int                       AIS_ALU_0 : 3;
        unsigned int                       RESERVED4 : 1;
        unsigned int                       ACS_ALU_0 : 3;
        unsigned int                       RESERVED5 : 1;
        unsigned int                      ARB_TR_ALU : 3;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                      ARB_TR_ALU : 3;
        unsigned int                       RESERVED5 : 1;
        unsigned int                       ACS_ALU_0 : 3;
        unsigned int                       RESERVED4 : 1;
        unsigned int                       AIS_ALU_0 : 3;
        unsigned int                       RESERVED3 : 1;
        unsigned int                       DU0_ALU_0 : 3;
        unsigned int                       RESERVED2 : 1;
        unsigned int                       DU1_ALU_0 : 3;
        unsigned int                       RESERVED1 : 1;
        unsigned int                        IF_ALU_0 : 3;
        unsigned int                        CF_ALU_0 : 4;
        unsigned int                       RESERVED0 : 1;
        unsigned int                        EX_ALU_0 : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_FSM_ALU_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        EX_ALU_0 : 3;
        unsigned int                       RESERVED0 : 1;
        unsigned int                        CF_ALU_0 : 4;
        unsigned int                        IF_ALU_0 : 3;
        unsigned int                       RESERVED1 : 1;
        unsigned int                       DU1_ALU_0 : 3;
        unsigned int                       RESERVED2 : 1;
        unsigned int                       DU0_ALU_0 : 3;
        unsigned int                       RESERVED3 : 1;
        unsigned int                       AIS_ALU_0 : 3;
        unsigned int                       RESERVED4 : 1;
        unsigned int                       ACS_ALU_0 : 3;
        unsigned int                       RESERVED5 : 1;
        unsigned int                      ARB_TR_ALU : 3;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                      ARB_TR_ALU : 3;
        unsigned int                       RESERVED5 : 1;
        unsigned int                       ACS_ALU_0 : 3;
        unsigned int                       RESERVED4 : 1;
        unsigned int                       AIS_ALU_0 : 3;
        unsigned int                       RESERVED3 : 1;
        unsigned int                       DU0_ALU_0 : 3;
        unsigned int                       RESERVED2 : 1;
        unsigned int                       DU1_ALU_0 : 3;
        unsigned int                       RESERVED1 : 1;
        unsigned int                        IF_ALU_0 : 3;
        unsigned int                        CF_ALU_0 : 4;
        unsigned int                       RESERVED0 : 1;
        unsigned int                        EX_ALU_0 : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_EXP_ALLOC {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   POS_BUF_AVAIL : 4;
        unsigned int                 COLOR_BUF_AVAIL : 8;
        unsigned int                    EA_BUF_AVAIL : 3;
        unsigned int                        RESERVED : 1;
        unsigned int             ALLOC_TBL_BUF_AVAIL : 6;
        unsigned int                                 : 10;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 10;
        unsigned int             ALLOC_TBL_BUF_AVAIL : 6;
        unsigned int                        RESERVED : 1;
        unsigned int                    EA_BUF_AVAIL : 3;
        unsigned int                 COLOR_BUF_AVAIL : 8;
        unsigned int                   POS_BUF_AVAIL : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_PTR_BUFF {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   END_OF_BUFFER : 1;
        unsigned int                     DEALLOC_CNT : 4;
        unsigned int                 QUAL_NEW_VECTOR : 1;
        unsigned int                EVENT_CONTEXT_ID : 3;
        unsigned int                     SC_EVENT_ID : 5;
        unsigned int                      QUAL_EVENT : 1;
        unsigned int               PRIM_TYPE_POLYGON : 1;
        unsigned int                        EF_EMPTY : 1;
        unsigned int                    VTX_SYNC_CNT : 11;
        unsigned int                                 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 4;
        unsigned int                    VTX_SYNC_CNT : 11;
        unsigned int                        EF_EMPTY : 1;
        unsigned int               PRIM_TYPE_POLYGON : 1;
        unsigned int                      QUAL_EVENT : 1;
        unsigned int                     SC_EVENT_ID : 5;
        unsigned int                EVENT_CONTEXT_ID : 3;
        unsigned int                 QUAL_NEW_VECTOR : 1;
        unsigned int                     DEALLOC_CNT : 4;
        unsigned int                   END_OF_BUFFER : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_GPR_VTX {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    VTX_TAIL_PTR : 7;
        unsigned int                        RESERVED : 1;
        unsigned int                    VTX_HEAD_PTR : 7;
        unsigned int                       RESERVED1 : 1;
        unsigned int                         VTX_MAX : 7;
        unsigned int                       RESERVED2 : 1;
        unsigned int                        VTX_FREE : 7;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                        VTX_FREE : 7;
        unsigned int                       RESERVED2 : 1;
        unsigned int                         VTX_MAX : 7;
        unsigned int                       RESERVED1 : 1;
        unsigned int                    VTX_HEAD_PTR : 7;
        unsigned int                        RESERVED : 1;
        unsigned int                    VTX_TAIL_PTR : 7;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_GPR_PIX {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    PIX_TAIL_PTR : 7;
        unsigned int                        RESERVED : 1;
        unsigned int                    PIX_HEAD_PTR : 7;
        unsigned int                       RESERVED1 : 1;
        unsigned int                         PIX_MAX : 7;
        unsigned int                       RESERVED2 : 1;
        unsigned int                        PIX_FREE : 7;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                        PIX_FREE : 7;
        unsigned int                       RESERVED2 : 1;
        unsigned int                         PIX_MAX : 7;
        unsigned int                       RESERVED1 : 1;
        unsigned int                    PIX_HEAD_PTR : 7;
        unsigned int                        RESERVED : 1;
        unsigned int                    PIX_TAIL_PTR : 7;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_TB_STATUS_SEL {
    struct {
#if     defined(qLittleEndian)
        unsigned int           VTX_TB_STATUS_REG_SEL : 4;
        unsigned int         VTX_TB_STATE_MEM_DW_SEL : 3;
        unsigned int        VTX_TB_STATE_MEM_RD_ADDR : 4;
        unsigned int          VTX_TB_STATE_MEM_RD_EN : 1;
        unsigned int          PIX_TB_STATE_MEM_RD_EN : 1;
        unsigned int                                 : 1;
        unsigned int           DEBUG_BUS_TRIGGER_SEL : 2;
        unsigned int           PIX_TB_STATUS_REG_SEL : 4;
        unsigned int         PIX_TB_STATE_MEM_DW_SEL : 3;
        unsigned int        PIX_TB_STATE_MEM_RD_ADDR : 6;
        unsigned int               VC_THREAD_BUF_DLY : 2;
        unsigned int         DISABLE_STRICT_CTX_SYNC : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int         DISABLE_STRICT_CTX_SYNC : 1;
        unsigned int               VC_THREAD_BUF_DLY : 2;
        unsigned int        PIX_TB_STATE_MEM_RD_ADDR : 6;
        unsigned int         PIX_TB_STATE_MEM_DW_SEL : 3;
        unsigned int           PIX_TB_STATUS_REG_SEL : 4;
        unsigned int           DEBUG_BUS_TRIGGER_SEL : 2;
        unsigned int                                 : 1;
        unsigned int          PIX_TB_STATE_MEM_RD_EN : 1;
        unsigned int          VTX_TB_STATE_MEM_RD_EN : 1;
        unsigned int        VTX_TB_STATE_MEM_RD_ADDR : 4;
        unsigned int         VTX_TB_STATE_MEM_DW_SEL : 3;
        unsigned int           VTX_TB_STATUS_REG_SEL : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_VTX_TB_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  VTX_HEAD_PTR_Q : 4;
        unsigned int                      TAIL_PTR_Q : 4;
        unsigned int                      FULL_CNT_Q : 4;
        unsigned int               NXT_POS_ALLOC_CNT : 4;
        unsigned int                NXT_PC_ALLOC_CNT : 4;
        unsigned int                   SX_EVENT_FULL : 1;
        unsigned int                          BUSY_Q : 1;
        unsigned int                                 : 10;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 10;
        unsigned int                          BUSY_Q : 1;
        unsigned int                   SX_EVENT_FULL : 1;
        unsigned int                NXT_PC_ALLOC_CNT : 4;
        unsigned int               NXT_POS_ALLOC_CNT : 4;
        unsigned int                      FULL_CNT_Q : 4;
        unsigned int                      TAIL_PTR_Q : 4;
        unsigned int                  VTX_HEAD_PTR_Q : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_VTX_TB_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     VS_DONE_PTR : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                     VS_DONE_PTR : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_VTX_TB_STATUS_REG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   VS_STATUS_REG : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   VS_STATUS_REG : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_VTX_TB_STATE_MEM {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    VS_STATE_MEM : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    VS_STATE_MEM : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_PIX_TB_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    PIX_HEAD_PTR : 6;
        unsigned int                        TAIL_PTR : 6;
        unsigned int                        FULL_CNT : 7;
        unsigned int               NXT_PIX_ALLOC_CNT : 6;
        unsigned int                 NXT_PIX_EXP_CNT : 6;
        unsigned int                            BUSY : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                            BUSY : 1;
        unsigned int                 NXT_PIX_EXP_CNT : 6;
        unsigned int               NXT_PIX_ALLOC_CNT : 6;
        unsigned int                        FULL_CNT : 7;
        unsigned int                        TAIL_PTR : 6;
        unsigned int                    PIX_HEAD_PTR : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_PIX_TB_STATUS_REG_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int             PIX_TB_STATUS_REG_0 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int             PIX_TB_STATUS_REG_0 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_PIX_TB_STATUS_REG_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int             PIX_TB_STATUS_REG_1 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int             PIX_TB_STATUS_REG_1 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_PIX_TB_STATUS_REG_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int             PIX_TB_STATUS_REG_2 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int             PIX_TB_STATUS_REG_2 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_PIX_TB_STATUS_REG_3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int             PIX_TB_STATUS_REG_3 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int             PIX_TB_STATUS_REG_3 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_PIX_TB_STATE_MEM {
    struct {
#if     defined(qLittleEndian)
        unsigned int                PIX_TB_STATE_MEM : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                PIX_TB_STATE_MEM : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER0_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER1_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER2_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER3_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER0_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER0_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER1_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER1_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER2_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER2_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER3_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PERFCOUNTER3_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SX_PERFCOUNTER0_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SX_PERFCOUNTER0_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SX_PERFCOUNTER0_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_ALU_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   VECTOR_RESULT : 6;
        unsigned int                  VECTOR_DST_REL : 1;
        unsigned int            LOW_PRECISION_16B_FP : 1;
        unsigned int                   SCALAR_RESULT : 6;
        unsigned int                  SCALAR_DST_REL : 1;
        unsigned int                     EXPORT_DATA : 1;
        unsigned int                  VECTOR_WRT_MSK : 4;
        unsigned int                  SCALAR_WRT_MSK : 4;
        unsigned int                    VECTOR_CLAMP : 1;
        unsigned int                    SCALAR_CLAMP : 1;
        unsigned int                   SCALAR_OPCODE : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int                   SCALAR_OPCODE : 6;
        unsigned int                    SCALAR_CLAMP : 1;
        unsigned int                    VECTOR_CLAMP : 1;
        unsigned int                  SCALAR_WRT_MSK : 4;
        unsigned int                  VECTOR_WRT_MSK : 4;
        unsigned int                     EXPORT_DATA : 1;
        unsigned int                  SCALAR_DST_REL : 1;
        unsigned int                   SCALAR_RESULT : 6;
        unsigned int            LOW_PRECISION_16B_FP : 1;
        unsigned int                  VECTOR_DST_REL : 1;
        unsigned int                   VECTOR_RESULT : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_ALU_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 SRC_C_SWIZZLE_R : 2;
        unsigned int                 SRC_C_SWIZZLE_G : 2;
        unsigned int                 SRC_C_SWIZZLE_B : 2;
        unsigned int                 SRC_C_SWIZZLE_A : 2;
        unsigned int                 SRC_B_SWIZZLE_R : 2;
        unsigned int                 SRC_B_SWIZZLE_G : 2;
        unsigned int                 SRC_B_SWIZZLE_B : 2;
        unsigned int                 SRC_B_SWIZZLE_A : 2;
        unsigned int                 SRC_A_SWIZZLE_R : 2;
        unsigned int                 SRC_A_SWIZZLE_G : 2;
        unsigned int                 SRC_A_SWIZZLE_B : 2;
        unsigned int                 SRC_A_SWIZZLE_A : 2;
        unsigned int                   SRC_C_ARG_MOD : 1;
        unsigned int                   SRC_B_ARG_MOD : 1;
        unsigned int                   SRC_A_ARG_MOD : 1;
        unsigned int                     PRED_SELECT : 2;
        unsigned int                   RELATIVE_ADDR : 1;
        unsigned int                 CONST_1_REL_ABS : 1;
        unsigned int                 CONST_0_REL_ABS : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                 CONST_0_REL_ABS : 1;
        unsigned int                 CONST_1_REL_ABS : 1;
        unsigned int                   RELATIVE_ADDR : 1;
        unsigned int                     PRED_SELECT : 2;
        unsigned int                   SRC_A_ARG_MOD : 1;
        unsigned int                   SRC_B_ARG_MOD : 1;
        unsigned int                   SRC_C_ARG_MOD : 1;
        unsigned int                 SRC_A_SWIZZLE_A : 2;
        unsigned int                 SRC_A_SWIZZLE_B : 2;
        unsigned int                 SRC_A_SWIZZLE_G : 2;
        unsigned int                 SRC_A_SWIZZLE_R : 2;
        unsigned int                 SRC_B_SWIZZLE_A : 2;
        unsigned int                 SRC_B_SWIZZLE_B : 2;
        unsigned int                 SRC_B_SWIZZLE_G : 2;
        unsigned int                 SRC_B_SWIZZLE_R : 2;
        unsigned int                 SRC_C_SWIZZLE_A : 2;
        unsigned int                 SRC_C_SWIZZLE_B : 2;
        unsigned int                 SRC_C_SWIZZLE_G : 2;
        unsigned int                 SRC_C_SWIZZLE_R : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_ALU_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   SRC_C_REG_PTR : 6;
        unsigned int                    REG_SELECT_C : 1;
        unsigned int                   REG_ABS_MOD_C : 1;
        unsigned int                   SRC_B_REG_PTR : 6;
        unsigned int                    REG_SELECT_B : 1;
        unsigned int                   REG_ABS_MOD_B : 1;
        unsigned int                   SRC_A_REG_PTR : 6;
        unsigned int                    REG_SELECT_A : 1;
        unsigned int                   REG_ABS_MOD_A : 1;
        unsigned int                   VECTOR_OPCODE : 5;
        unsigned int                       SRC_C_SEL : 1;
        unsigned int                       SRC_B_SEL : 1;
        unsigned int                       SRC_A_SEL : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                       SRC_A_SEL : 1;
        unsigned int                       SRC_B_SEL : 1;
        unsigned int                       SRC_C_SEL : 1;
        unsigned int                   VECTOR_OPCODE : 5;
        unsigned int                   REG_ABS_MOD_A : 1;
        unsigned int                    REG_SELECT_A : 1;
        unsigned int                   SRC_A_REG_PTR : 6;
        unsigned int                   REG_ABS_MOD_B : 1;
        unsigned int                    REG_SELECT_B : 1;
        unsigned int                   SRC_B_REG_PTR : 6;
        unsigned int                   REG_ABS_MOD_C : 1;
        unsigned int                    REG_SELECT_C : 1;
        unsigned int                   SRC_C_REG_PTR : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_EXEC_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         ADDRESS : 9;
        unsigned int                        RESERVED : 3;
        unsigned int                           COUNT : 3;
        unsigned int                           YIELD : 1;
        unsigned int                     INST_TYPE_0 : 1;
        unsigned int                   INST_SERIAL_0 : 1;
        unsigned int                     INST_TYPE_1 : 1;
        unsigned int                   INST_SERIAL_1 : 1;
        unsigned int                     INST_TYPE_2 : 1;
        unsigned int                   INST_SERIAL_2 : 1;
        unsigned int                     INST_TYPE_3 : 1;
        unsigned int                   INST_SERIAL_3 : 1;
        unsigned int                     INST_TYPE_4 : 1;
        unsigned int                   INST_SERIAL_4 : 1;
        unsigned int                     INST_TYPE_5 : 1;
        unsigned int                   INST_SERIAL_5 : 1;
        unsigned int                       INST_VC_0 : 1;
        unsigned int                       INST_VC_1 : 1;
        unsigned int                       INST_VC_2 : 1;
        unsigned int                       INST_VC_3 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                       INST_VC_3 : 1;
        unsigned int                       INST_VC_2 : 1;
        unsigned int                       INST_VC_1 : 1;
        unsigned int                       INST_VC_0 : 1;
        unsigned int                   INST_SERIAL_5 : 1;
        unsigned int                     INST_TYPE_5 : 1;
        unsigned int                   INST_SERIAL_4 : 1;
        unsigned int                     INST_TYPE_4 : 1;
        unsigned int                   INST_SERIAL_3 : 1;
        unsigned int                     INST_TYPE_3 : 1;
        unsigned int                   INST_SERIAL_2 : 1;
        unsigned int                     INST_TYPE_2 : 1;
        unsigned int                   INST_SERIAL_1 : 1;
        unsigned int                     INST_TYPE_1 : 1;
        unsigned int                   INST_SERIAL_0 : 1;
        unsigned int                     INST_TYPE_0 : 1;
        unsigned int                           YIELD : 1;
        unsigned int                           COUNT : 3;
        unsigned int                        RESERVED : 3;
        unsigned int                         ADDRESS : 9;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_EXEC_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       INST_VC_4 : 1;
        unsigned int                       INST_VC_5 : 1;
        unsigned int                       BOOL_ADDR : 8;
        unsigned int                       CONDITION : 1;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                          OPCODE : 4;
        unsigned int                         ADDRESS : 9;
        unsigned int                        RESERVED : 3;
        unsigned int                           COUNT : 3;
        unsigned int                           YIELD : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                           YIELD : 1;
        unsigned int                           COUNT : 3;
        unsigned int                        RESERVED : 3;
        unsigned int                         ADDRESS : 9;
        unsigned int                          OPCODE : 4;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                       CONDITION : 1;
        unsigned int                       BOOL_ADDR : 8;
        unsigned int                       INST_VC_5 : 1;
        unsigned int                       INST_VC_4 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_EXEC_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     INST_TYPE_0 : 1;
        unsigned int                   INST_SERIAL_0 : 1;
        unsigned int                     INST_TYPE_1 : 1;
        unsigned int                   INST_SERIAL_1 : 1;
        unsigned int                     INST_TYPE_2 : 1;
        unsigned int                   INST_SERIAL_2 : 1;
        unsigned int                     INST_TYPE_3 : 1;
        unsigned int                   INST_SERIAL_3 : 1;
        unsigned int                     INST_TYPE_4 : 1;
        unsigned int                   INST_SERIAL_4 : 1;
        unsigned int                     INST_TYPE_5 : 1;
        unsigned int                   INST_SERIAL_5 : 1;
        unsigned int                       INST_VC_0 : 1;
        unsigned int                       INST_VC_1 : 1;
        unsigned int                       INST_VC_2 : 1;
        unsigned int                       INST_VC_3 : 1;
        unsigned int                       INST_VC_4 : 1;
        unsigned int                       INST_VC_5 : 1;
        unsigned int                       BOOL_ADDR : 8;
        unsigned int                       CONDITION : 1;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                          OPCODE : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                          OPCODE : 4;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                       CONDITION : 1;
        unsigned int                       BOOL_ADDR : 8;
        unsigned int                       INST_VC_5 : 1;
        unsigned int                       INST_VC_4 : 1;
        unsigned int                       INST_VC_3 : 1;
        unsigned int                       INST_VC_2 : 1;
        unsigned int                       INST_VC_1 : 1;
        unsigned int                       INST_VC_0 : 1;
        unsigned int                   INST_SERIAL_5 : 1;
        unsigned int                     INST_TYPE_5 : 1;
        unsigned int                   INST_SERIAL_4 : 1;
        unsigned int                     INST_TYPE_4 : 1;
        unsigned int                   INST_SERIAL_3 : 1;
        unsigned int                     INST_TYPE_3 : 1;
        unsigned int                   INST_SERIAL_2 : 1;
        unsigned int                     INST_TYPE_2 : 1;
        unsigned int                   INST_SERIAL_1 : 1;
        unsigned int                     INST_TYPE_1 : 1;
        unsigned int                   INST_SERIAL_0 : 1;
        unsigned int                     INST_TYPE_0 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_LOOP_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         ADDRESS : 10;
        unsigned int                      RESERVED_0 : 6;
        unsigned int                         LOOP_ID : 5;
        unsigned int                      RESERVED_1 : 11;
#else       /* !defined(qLittleEndian) */
        unsigned int                      RESERVED_1 : 11;
        unsigned int                         LOOP_ID : 5;
        unsigned int                      RESERVED_0 : 6;
        unsigned int                         ADDRESS : 10;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_LOOP_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      RESERVED_0 : 11;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                          OPCODE : 4;
        unsigned int                         ADDRESS : 10;
        unsigned int                      RESERVED_1 : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int                      RESERVED_1 : 6;
        unsigned int                         ADDRESS : 10;
        unsigned int                          OPCODE : 4;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                      RESERVED_0 : 11;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_LOOP_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         LOOP_ID : 5;
        unsigned int                        RESERVED : 22;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                          OPCODE : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                          OPCODE : 4;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                        RESERVED : 22;
        unsigned int                         LOOP_ID : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_JMP_CALL_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         ADDRESS : 10;
        unsigned int                      RESERVED_0 : 3;
        unsigned int                      FORCE_CALL : 1;
        unsigned int                  PREDICATED_JMP : 1;
        unsigned int                      RESERVED_1 : 17;
#else       /* !defined(qLittleEndian) */
        unsigned int                      RESERVED_1 : 17;
        unsigned int                  PREDICATED_JMP : 1;
        unsigned int                      FORCE_CALL : 1;
        unsigned int                      RESERVED_0 : 3;
        unsigned int                         ADDRESS : 10;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_JMP_CALL_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      RESERVED_0 : 1;
        unsigned int                       DIRECTION : 1;
        unsigned int                       BOOL_ADDR : 8;
        unsigned int                       CONDITION : 1;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                          OPCODE : 4;
        unsigned int                         ADDRESS : 10;
        unsigned int                      RESERVED_1 : 3;
        unsigned int                      FORCE_CALL : 1;
        unsigned int                      RESERVED_2 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                      RESERVED_2 : 2;
        unsigned int                      FORCE_CALL : 1;
        unsigned int                      RESERVED_1 : 3;
        unsigned int                         ADDRESS : 10;
        unsigned int                          OPCODE : 4;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                       CONDITION : 1;
        unsigned int                       BOOL_ADDR : 8;
        unsigned int                       DIRECTION : 1;
        unsigned int                      RESERVED_0 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_JMP_CALL_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        RESERVED : 17;
        unsigned int                       DIRECTION : 1;
        unsigned int                       BOOL_ADDR : 8;
        unsigned int                       CONDITION : 1;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                          OPCODE : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                          OPCODE : 4;
        unsigned int                    ADDRESS_MODE : 1;
        unsigned int                       CONDITION : 1;
        unsigned int                       BOOL_ADDR : 8;
        unsigned int                       DIRECTION : 1;
        unsigned int                        RESERVED : 17;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_ALLOC_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            SIZE : 4;
        unsigned int                        RESERVED : 28;
#else       /* !defined(qLittleEndian) */
        unsigned int                        RESERVED : 28;
        unsigned int                            SIZE : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_ALLOC_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      RESERVED_0 : 8;
        unsigned int                       NO_SERIAL : 1;
        unsigned int                   BUFFER_SELECT : 2;
        unsigned int                      ALLOC_MODE : 1;
        unsigned int                          OPCODE : 4;
        unsigned int                            SIZE : 4;
        unsigned int                      RESERVED_1 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                      RESERVED_1 : 12;
        unsigned int                            SIZE : 4;
        unsigned int                          OPCODE : 4;
        unsigned int                      ALLOC_MODE : 1;
        unsigned int                   BUFFER_SELECT : 2;
        unsigned int                       NO_SERIAL : 1;
        unsigned int                      RESERVED_0 : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_CF_ALLOC_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        RESERVED : 24;
        unsigned int                       NO_SERIAL : 1;
        unsigned int                   BUFFER_SELECT : 2;
        unsigned int                      ALLOC_MODE : 1;
        unsigned int                          OPCODE : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                          OPCODE : 4;
        unsigned int                      ALLOC_MODE : 1;
        unsigned int                   BUFFER_SELECT : 2;
        unsigned int                       NO_SERIAL : 1;
        unsigned int                        RESERVED : 24;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_TFETCH_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          OPCODE : 5;
        unsigned int                         SRC_GPR : 6;
        unsigned int                      SRC_GPR_AM : 1;
        unsigned int                         DST_GPR : 6;
        unsigned int                      DST_GPR_AM : 1;
        unsigned int                FETCH_VALID_ONLY : 1;
        unsigned int                     CONST_INDEX : 5;
        unsigned int                 TX_COORD_DENORM : 1;
        unsigned int                       SRC_SEL_X : 2;
        unsigned int                       SRC_SEL_Y : 2;
        unsigned int                       SRC_SEL_Z : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                       SRC_SEL_Z : 2;
        unsigned int                       SRC_SEL_Y : 2;
        unsigned int                       SRC_SEL_X : 2;
        unsigned int                 TX_COORD_DENORM : 1;
        unsigned int                     CONST_INDEX : 5;
        unsigned int                FETCH_VALID_ONLY : 1;
        unsigned int                      DST_GPR_AM : 1;
        unsigned int                         DST_GPR : 6;
        unsigned int                      SRC_GPR_AM : 1;
        unsigned int                         SRC_GPR : 6;
        unsigned int                          OPCODE : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_TFETCH_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       DST_SEL_X : 3;
        unsigned int                       DST_SEL_Y : 3;
        unsigned int                       DST_SEL_Z : 3;
        unsigned int                       DST_SEL_W : 3;
        unsigned int                      MAG_FILTER : 2;
        unsigned int                      MIN_FILTER : 2;
        unsigned int                      MIP_FILTER : 2;
        unsigned int                    ANISO_FILTER : 3;
        unsigned int                ARBITRARY_FILTER : 3;
        unsigned int                  VOL_MAG_FILTER : 2;
        unsigned int                  VOL_MIN_FILTER : 2;
        unsigned int                    USE_COMP_LOD : 1;
        unsigned int                     USE_REG_LOD : 2;
        unsigned int                     PRED_SELECT : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     PRED_SELECT : 1;
        unsigned int                     USE_REG_LOD : 2;
        unsigned int                    USE_COMP_LOD : 1;
        unsigned int                  VOL_MIN_FILTER : 2;
        unsigned int                  VOL_MAG_FILTER : 2;
        unsigned int                ARBITRARY_FILTER : 3;
        unsigned int                    ANISO_FILTER : 3;
        unsigned int                      MIP_FILTER : 2;
        unsigned int                      MIN_FILTER : 2;
        unsigned int                      MAG_FILTER : 2;
        unsigned int                       DST_SEL_W : 3;
        unsigned int                       DST_SEL_Z : 3;
        unsigned int                       DST_SEL_Y : 3;
        unsigned int                       DST_SEL_X : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_TFETCH_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int               USE_REG_GRADIENTS : 1;
        unsigned int                 SAMPLE_LOCATION : 1;
        unsigned int                        LOD_BIAS : 7;
        unsigned int                          UNUSED : 7;
        unsigned int                        OFFSET_X : 5;
        unsigned int                        OFFSET_Y : 5;
        unsigned int                        OFFSET_Z : 5;
        unsigned int                  PRED_CONDITION : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                  PRED_CONDITION : 1;
        unsigned int                        OFFSET_Z : 5;
        unsigned int                        OFFSET_Y : 5;
        unsigned int                        OFFSET_X : 5;
        unsigned int                          UNUSED : 7;
        unsigned int                        LOD_BIAS : 7;
        unsigned int                 SAMPLE_LOCATION : 1;
        unsigned int               USE_REG_GRADIENTS : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_VFETCH_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          OPCODE : 5;
        unsigned int                         SRC_GPR : 6;
        unsigned int                      SRC_GPR_AM : 1;
        unsigned int                         DST_GPR : 6;
        unsigned int                      DST_GPR_AM : 1;
        unsigned int                     MUST_BE_ONE : 1;
        unsigned int                     CONST_INDEX : 5;
        unsigned int                 CONST_INDEX_SEL : 2;
        unsigned int                                 : 3;
        unsigned int                         SRC_SEL : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                         SRC_SEL : 2;
        unsigned int                                 : 3;
        unsigned int                 CONST_INDEX_SEL : 2;
        unsigned int                     CONST_INDEX : 5;
        unsigned int                     MUST_BE_ONE : 1;
        unsigned int                      DST_GPR_AM : 1;
        unsigned int                         DST_GPR : 6;
        unsigned int                      SRC_GPR_AM : 1;
        unsigned int                         SRC_GPR : 6;
        unsigned int                          OPCODE : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_VFETCH_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       DST_SEL_X : 3;
        unsigned int                       DST_SEL_Y : 3;
        unsigned int                       DST_SEL_Z : 3;
        unsigned int                       DST_SEL_W : 3;
        unsigned int                 FORMAT_COMP_ALL : 1;
        unsigned int                  NUM_FORMAT_ALL : 1;
        unsigned int              SIGNED_RF_MODE_ALL : 1;
        unsigned int                                 : 1;
        unsigned int                     DATA_FORMAT : 6;
        unsigned int                                 : 1;
        unsigned int                  EXP_ADJUST_ALL : 7;
        unsigned int                                 : 1;
        unsigned int                     PRED_SELECT : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     PRED_SELECT : 1;
        unsigned int                                 : 1;
        unsigned int                  EXP_ADJUST_ALL : 7;
        unsigned int                                 : 1;
        unsigned int                     DATA_FORMAT : 6;
        unsigned int                                 : 1;
        unsigned int              SIGNED_RF_MODE_ALL : 1;
        unsigned int                  NUM_FORMAT_ALL : 1;
        unsigned int                 FORMAT_COMP_ALL : 1;
        unsigned int                       DST_SEL_W : 3;
        unsigned int                       DST_SEL_Z : 3;
        unsigned int                       DST_SEL_Y : 3;
        unsigned int                       DST_SEL_X : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INSTRUCTION_VFETCH_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          STRIDE : 8;
        unsigned int                                 : 8;
        unsigned int                          OFFSET : 8;
        unsigned int                                 : 7;
        unsigned int                  PRED_CONDITION : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                  PRED_CONDITION : 1;
        unsigned int                                 : 7;
        unsigned int                          OFFSET : 8;
        unsigned int                                 : 8;
        unsigned int                          STRIDE : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                             RED : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                             RED : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           GREEN : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           GREEN : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            BLUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                            BLUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           ALPHA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           ALPHA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_4 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_5 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_VFETCH_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            TYPE : 1;
        unsigned int                           STATE : 1;
        unsigned int                    BASE_ADDRESS : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BASE_ADDRESS : 30;
        unsigned int                           STATE : 1;
        unsigned int                            TYPE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_VFETCH_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     ENDIAN_SWAP : 2;
        unsigned int                   LIMIT_ADDRESS : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                   LIMIT_ADDRESS : 30;
        unsigned int                     ENDIAN_SWAP : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_T2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_T3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CF_BOOLEANS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   CF_BOOLEANS_0 : 8;
        unsigned int                   CF_BOOLEANS_1 : 8;
        unsigned int                   CF_BOOLEANS_2 : 8;
        unsigned int                   CF_BOOLEANS_3 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                   CF_BOOLEANS_3 : 8;
        unsigned int                   CF_BOOLEANS_2 : 8;
        unsigned int                   CF_BOOLEANS_1 : 8;
        unsigned int                   CF_BOOLEANS_0 : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CF_LOOP {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   CF_LOOP_COUNT : 8;
        unsigned int                   CF_LOOP_START : 8;
        unsigned int                    CF_LOOP_STEP : 8;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                    CF_LOOP_STEP : 8;
        unsigned int                   CF_LOOP_START : 8;
        unsigned int                   CF_LOOP_COUNT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_RT_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                             RED : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                             RED : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_RT_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           GREEN : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           GREEN : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_RT_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            BLUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                            BLUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONSTANT_RT_3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           ALPHA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           ALPHA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_RT_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_RT_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_RT_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_RT_3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_RT_4 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_FETCH_RT_5 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           VALUE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                           VALUE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CF_RT_BOOLEANS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   CF_BOOLEANS_0 : 8;
        unsigned int                   CF_BOOLEANS_1 : 8;
        unsigned int                   CF_BOOLEANS_2 : 8;
        unsigned int                   CF_BOOLEANS_3 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                   CF_BOOLEANS_3 : 8;
        unsigned int                   CF_BOOLEANS_2 : 8;
        unsigned int                   CF_BOOLEANS_1 : 8;
        unsigned int                   CF_BOOLEANS_0 : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CF_RT_LOOP {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   CF_LOOP_COUNT : 8;
        unsigned int                   CF_LOOP_START : 8;
        unsigned int                    CF_LOOP_STEP : 8;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                    CF_LOOP_STEP : 8;
        unsigned int                   CF_LOOP_START : 8;
        unsigned int                   CF_LOOP_COUNT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_VS_PROGRAM {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            BASE : 12;
        unsigned int                            SIZE : 12;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                            SIZE : 12;
        unsigned int                            BASE : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PS_PROGRAM {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            BASE : 12;
        unsigned int                            SIZE : 12;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                            SIZE : 12;
        unsigned int                            BASE : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CF_PROGRAM_SIZE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      VS_CF_SIZE : 11;
        unsigned int                                 : 1;
        unsigned int                      PS_CF_SIZE : 11;
        unsigned int                                 : 9;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 9;
        unsigned int                      PS_CF_SIZE : 11;
        unsigned int                                 : 1;
        unsigned int                      VS_CF_SIZE : 11;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_INTERPOLATOR_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     PARAM_SHADE : 16;
        unsigned int                SAMPLING_PATTERN : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                SAMPLING_PATTERN : 16;
        unsigned int                     PARAM_SHADE : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PROGRAM_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      VS_NUM_REG : 6;
        unsigned int                                 : 2;
        unsigned int                      PS_NUM_REG : 6;
        unsigned int                                 : 2;
        unsigned int                     VS_RESOURCE : 1;
        unsigned int                     PS_RESOURCE : 1;
        unsigned int                       PARAM_GEN : 1;
        unsigned int                   GEN_INDEX_PIX : 1;
        unsigned int                 VS_EXPORT_COUNT : 4;
        unsigned int                  VS_EXPORT_MODE : 3;
        unsigned int                  PS_EXPORT_MODE : 4;
        unsigned int                   GEN_INDEX_VTX : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                   GEN_INDEX_VTX : 1;
        unsigned int                  PS_EXPORT_MODE : 4;
        unsigned int                  VS_EXPORT_MODE : 3;
        unsigned int                 VS_EXPORT_COUNT : 4;
        unsigned int                   GEN_INDEX_PIX : 1;
        unsigned int                       PARAM_GEN : 1;
        unsigned int                     PS_RESOURCE : 1;
        unsigned int                     VS_RESOURCE : 1;
        unsigned int                                 : 2;
        unsigned int                      PS_NUM_REG : 6;
        unsigned int                                 : 2;
        unsigned int                      VS_NUM_REG : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_WRAPPING_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    PARAM_WRAP_0 : 4;
        unsigned int                    PARAM_WRAP_1 : 4;
        unsigned int                    PARAM_WRAP_2 : 4;
        unsigned int                    PARAM_WRAP_3 : 4;
        unsigned int                    PARAM_WRAP_4 : 4;
        unsigned int                    PARAM_WRAP_5 : 4;
        unsigned int                    PARAM_WRAP_6 : 4;
        unsigned int                    PARAM_WRAP_7 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                    PARAM_WRAP_7 : 4;
        unsigned int                    PARAM_WRAP_6 : 4;
        unsigned int                    PARAM_WRAP_5 : 4;
        unsigned int                    PARAM_WRAP_4 : 4;
        unsigned int                    PARAM_WRAP_3 : 4;
        unsigned int                    PARAM_WRAP_2 : 4;
        unsigned int                    PARAM_WRAP_1 : 4;
        unsigned int                    PARAM_WRAP_0 : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_WRAPPING_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    PARAM_WRAP_8 : 4;
        unsigned int                    PARAM_WRAP_9 : 4;
        unsigned int                   PARAM_WRAP_10 : 4;
        unsigned int                   PARAM_WRAP_11 : 4;
        unsigned int                   PARAM_WRAP_12 : 4;
        unsigned int                   PARAM_WRAP_13 : 4;
        unsigned int                   PARAM_WRAP_14 : 4;
        unsigned int                   PARAM_WRAP_15 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                   PARAM_WRAP_15 : 4;
        unsigned int                   PARAM_WRAP_14 : 4;
        unsigned int                   PARAM_WRAP_13 : 4;
        unsigned int                   PARAM_WRAP_12 : 4;
        unsigned int                   PARAM_WRAP_11 : 4;
        unsigned int                   PARAM_WRAP_10 : 4;
        unsigned int                    PARAM_WRAP_9 : 4;
        unsigned int                    PARAM_WRAP_8 : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_VS_CONST {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            BASE : 9;
        unsigned int                                 : 3;
        unsigned int                            SIZE : 9;
        unsigned int                                 : 11;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 11;
        unsigned int                            SIZE : 9;
        unsigned int                                 : 3;
        unsigned int                            BASE : 9;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_PS_CONST {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            BASE : 9;
        unsigned int                                 : 3;
        unsigned int                            SIZE : 9;
        unsigned int                                 : 11;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 11;
        unsigned int                            SIZE : 9;
        unsigned int                                 : 3;
        unsigned int                            BASE : 9;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CONTEXT_MISC {
    struct {
#if     defined(qLittleEndian)
        unsigned int              INST_PRED_OPTIMIZE : 1;
        unsigned int             SC_OUTPUT_SCREEN_XY : 1;
        unsigned int                  SC_SAMPLE_CNTL : 2;
        unsigned int                                 : 4;
        unsigned int                   PARAM_GEN_POS : 8;
        unsigned int                 PERFCOUNTER_REF : 1;
        unsigned int                  YEILD_OPTIMIZE : 1;
        unsigned int                    TX_CACHE_SEL : 1;
        unsigned int                                 : 13;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 13;
        unsigned int                    TX_CACHE_SEL : 1;
        unsigned int                  YEILD_OPTIMIZE : 1;
        unsigned int                 PERFCOUNTER_REF : 1;
        unsigned int                   PARAM_GEN_POS : 8;
        unsigned int                                 : 4;
        unsigned int                  SC_SAMPLE_CNTL : 2;
        unsigned int             SC_OUTPUT_SCREEN_XY : 1;
        unsigned int              INST_PRED_OPTIMIZE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_CF_RD_BASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         RD_BASE : 3;
        unsigned int                                 : 29;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 29;
        unsigned int                         RD_BASE : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_MISC_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      DB_PROB_ON : 1;
        unsigned int                                 : 3;
        unsigned int                   DB_PROB_BREAK : 1;
        unsigned int                                 : 3;
        unsigned int                    DB_PROB_ADDR : 11;
        unsigned int                                 : 5;
        unsigned int                   DB_PROB_COUNT : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                   DB_PROB_COUNT : 8;
        unsigned int                                 : 5;
        unsigned int                    DB_PROB_ADDR : 11;
        unsigned int                                 : 3;
        unsigned int                   DB_PROB_BREAK : 1;
        unsigned int                                 : 3;
        unsigned int                      DB_PROB_ON : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SQ_DEBUG_MISC_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       DB_ON_PIX : 1;
        unsigned int                       DB_ON_VTX : 1;
        unsigned int                                 : 6;
        unsigned int                   DB_INST_COUNT : 8;
        unsigned int                   DB_BREAK_ADDR : 11;
        unsigned int                                 : 5;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 5;
        unsigned int                   DB_BREAK_ADDR : 11;
        unsigned int                   DB_INST_COUNT : 8;
        unsigned int                                 : 6;
        unsigned int                       DB_ON_VTX : 1;
        unsigned int                       DB_ON_PIX : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_ARBITER_CONFIG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 SAME_PAGE_LIMIT : 6;
        unsigned int           SAME_PAGE_GRANULARITY : 1;
        unsigned int                   L1_ARB_ENABLE : 1;
        unsigned int              L1_ARB_HOLD_ENABLE : 1;
        unsigned int                  L2_ARB_CONTROL : 1;
        unsigned int                       PAGE_SIZE : 3;
        unsigned int               TC_REORDER_ENABLE : 1;
        unsigned int              TC_ARB_HOLD_ENABLE : 1;
        unsigned int          IN_FLIGHT_LIMIT_ENABLE : 1;
        unsigned int                 IN_FLIGHT_LIMIT : 6;
        unsigned int                  CP_CLNT_ENABLE : 1;
        unsigned int                 VGT_CLNT_ENABLE : 1;
        unsigned int                  TC_CLNT_ENABLE : 1;
        unsigned int                  RB_CLNT_ENABLE : 1;
        unsigned int                  PA_CLNT_ENABLE : 1;
        unsigned int                                 : 5;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 5;
        unsigned int                  PA_CLNT_ENABLE : 1;
        unsigned int                  RB_CLNT_ENABLE : 1;
        unsigned int                  TC_CLNT_ENABLE : 1;
        unsigned int                 VGT_CLNT_ENABLE : 1;
        unsigned int                  CP_CLNT_ENABLE : 1;
        unsigned int                 IN_FLIGHT_LIMIT : 6;
        unsigned int          IN_FLIGHT_LIMIT_ENABLE : 1;
        unsigned int              TC_ARB_HOLD_ENABLE : 1;
        unsigned int               TC_REORDER_ENABLE : 1;
        unsigned int                       PAGE_SIZE : 3;
        unsigned int                  L2_ARB_CONTROL : 1;
        unsigned int              L1_ARB_HOLD_ENABLE : 1;
        unsigned int                   L1_ARB_ENABLE : 1;
        unsigned int           SAME_PAGE_GRANULARITY : 1;
        unsigned int                 SAME_PAGE_LIMIT : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_CLNT_AXI_ID_REUSE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          CPw_ID : 3;
        unsigned int                       RESERVED1 : 1;
        unsigned int                          RBw_ID : 3;
        unsigned int                       RESERVED2 : 1;
        unsigned int                         MMUr_ID : 3;
        unsigned int                       RESERVED3 : 1;
        unsigned int                          PAw_ID : 3;
        unsigned int                                 : 17;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 17;
        unsigned int                          PAw_ID : 3;
        unsigned int                       RESERVED3 : 1;
        unsigned int                         MMUr_ID : 3;
        unsigned int                       RESERVED2 : 1;
        unsigned int                          RBw_ID : 3;
        unsigned int                       RESERVED1 : 1;
        unsigned int                          CPw_ID : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_INTERRUPT_MASK {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  AXI_READ_ERROR : 1;
        unsigned int                 AXI_WRITE_ERROR : 1;
        unsigned int                  MMU_PAGE_FAULT : 1;
        unsigned int                                 : 29;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 29;
        unsigned int                  MMU_PAGE_FAULT : 1;
        unsigned int                 AXI_WRITE_ERROR : 1;
        unsigned int                  AXI_READ_ERROR : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_INTERRUPT_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  AXI_READ_ERROR : 1;
        unsigned int                 AXI_WRITE_ERROR : 1;
        unsigned int                  MMU_PAGE_FAULT : 1;
        unsigned int                                 : 29;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 29;
        unsigned int                  MMU_PAGE_FAULT : 1;
        unsigned int                 AXI_WRITE_ERROR : 1;
        unsigned int                  AXI_READ_ERROR : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_INTERRUPT_CLEAR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  AXI_READ_ERROR : 1;
        unsigned int                 AXI_WRITE_ERROR : 1;
        unsigned int                  MMU_PAGE_FAULT : 1;
        unsigned int                                 : 29;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 29;
        unsigned int                  MMU_PAGE_FAULT : 1;
        unsigned int                 AXI_WRITE_ERROR : 1;
        unsigned int                  AXI_READ_ERROR : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_AXI_ERROR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     AXI_READ_ID : 3;
        unsigned int                  AXI_READ_ERROR : 1;
        unsigned int                    AXI_WRITE_ID : 3;
        unsigned int                 AXI_WRITE_ERROR : 1;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                 AXI_WRITE_ERROR : 1;
        unsigned int                    AXI_WRITE_ID : 3;
        unsigned int                  AXI_READ_ERROR : 1;
        unsigned int                     AXI_READ_ID : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_PERFCOUNTER0_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_PERFCOUNTER1_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_PERFCOUNTER0_CONFIG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         N_VALUE : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                         N_VALUE : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_PERFCOUNTER1_CONFIG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         N_VALUE : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                         N_VALUE : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_PERFCOUNTER0_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                PERF_COUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                PERF_COUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_PERFCOUNTER1_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                PERF_COUNTER_LOW : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                PERF_COUNTER_LOW : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_PERFCOUNTER0_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERF_COUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                 PERF_COUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_PERFCOUNTER1_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERF_COUNTER_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                 PERF_COUNTER_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_CTRL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                           INDEX : 6;
        unsigned int                                 : 26;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 26;
        unsigned int                           INDEX : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                            DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_AXI_HALT_CONTROL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        AXI_HALT : 1;
        unsigned int                                 : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 31;
        unsigned int                        AXI_HALT : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG00 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         MH_BUSY : 1;
        unsigned int               TRANS_OUTSTANDING : 1;
        unsigned int                      CP_REQUEST : 1;
        unsigned int                     VGT_REQUEST : 1;
        unsigned int                      TC_REQUEST : 1;
        unsigned int                    TC_CAM_EMPTY : 1;
        unsigned int                     TC_CAM_FULL : 1;
        unsigned int                       TCD_EMPTY : 1;
        unsigned int                        TCD_FULL : 1;
        unsigned int                      RB_REQUEST : 1;
        unsigned int                      PA_REQUEST : 1;
        unsigned int                 MH_CLK_EN_STATE : 1;
        unsigned int                       ARQ_EMPTY : 1;
        unsigned int                        ARQ_FULL : 1;
        unsigned int                       WDB_EMPTY : 1;
        unsigned int                        WDB_FULL : 1;
        unsigned int                      AXI_AVALID : 1;
        unsigned int                      AXI_AREADY : 1;
        unsigned int                     AXI_ARVALID : 1;
        unsigned int                     AXI_ARREADY : 1;
        unsigned int                      AXI_WVALID : 1;
        unsigned int                      AXI_WREADY : 1;
        unsigned int                      AXI_RVALID : 1;
        unsigned int                      AXI_RREADY : 1;
        unsigned int                      AXI_BVALID : 1;
        unsigned int                      AXI_BREADY : 1;
        unsigned int                    AXI_HALT_REQ : 1;
        unsigned int                    AXI_HALT_ACK : 1;
        unsigned int                     AXI_RDY_ENA : 1;
        unsigned int                                 : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 3;
        unsigned int                     AXI_RDY_ENA : 1;
        unsigned int                    AXI_HALT_ACK : 1;
        unsigned int                    AXI_HALT_REQ : 1;
        unsigned int                      AXI_BREADY : 1;
        unsigned int                      AXI_BVALID : 1;
        unsigned int                      AXI_RREADY : 1;
        unsigned int                      AXI_RVALID : 1;
        unsigned int                      AXI_WREADY : 1;
        unsigned int                      AXI_WVALID : 1;
        unsigned int                     AXI_ARREADY : 1;
        unsigned int                     AXI_ARVALID : 1;
        unsigned int                      AXI_AREADY : 1;
        unsigned int                      AXI_AVALID : 1;
        unsigned int                        WDB_FULL : 1;
        unsigned int                       WDB_EMPTY : 1;
        unsigned int                        ARQ_FULL : 1;
        unsigned int                       ARQ_EMPTY : 1;
        unsigned int                 MH_CLK_EN_STATE : 1;
        unsigned int                      PA_REQUEST : 1;
        unsigned int                      RB_REQUEST : 1;
        unsigned int                        TCD_FULL : 1;
        unsigned int                       TCD_EMPTY : 1;
        unsigned int                     TC_CAM_FULL : 1;
        unsigned int                    TC_CAM_EMPTY : 1;
        unsigned int                      TC_REQUEST : 1;
        unsigned int                     VGT_REQUEST : 1;
        unsigned int                      CP_REQUEST : 1;
        unsigned int               TRANS_OUTSTANDING : 1;
        unsigned int                         MH_BUSY : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG01 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       CP_SEND_q : 1;
        unsigned int                        CP_RTR_q : 1;
        unsigned int                      CP_WRITE_q : 1;
        unsigned int                        CP_TAG_q : 3;
        unsigned int                       CP_BLEN_q : 1;
        unsigned int                      VGT_SEND_q : 1;
        unsigned int                       VGT_RTR_q : 1;
        unsigned int                       VGT_TAG_q : 1;
        unsigned int                       TC_SEND_q : 1;
        unsigned int                        TC_RTR_q : 1;
        unsigned int                       TC_BLEN_q : 1;
        unsigned int                   TC_ROQ_SEND_q : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                   TC_MH_written : 1;
        unsigned int                       RB_SEND_q : 1;
        unsigned int                        RB_RTR_q : 1;
        unsigned int                       PA_SEND_q : 1;
        unsigned int                        PA_RTR_q : 1;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                        PA_RTR_q : 1;
        unsigned int                       PA_SEND_q : 1;
        unsigned int                        RB_RTR_q : 1;
        unsigned int                       RB_SEND_q : 1;
        unsigned int                   TC_MH_written : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                   TC_ROQ_SEND_q : 1;
        unsigned int                       TC_BLEN_q : 1;
        unsigned int                        TC_RTR_q : 1;
        unsigned int                       TC_SEND_q : 1;
        unsigned int                       VGT_TAG_q : 1;
        unsigned int                       VGT_RTR_q : 1;
        unsigned int                      VGT_SEND_q : 1;
        unsigned int                       CP_BLEN_q : 1;
        unsigned int                        CP_TAG_q : 3;
        unsigned int                      CP_WRITE_q : 1;
        unsigned int                        CP_RTR_q : 1;
        unsigned int                       CP_SEND_q : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG02 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  MH_CP_grb_send : 1;
        unsigned int                 MH_VGT_grb_send : 1;
        unsigned int                    MH_TC_mcsend : 1;
        unsigned int                   MH_CLNT_rlast : 1;
        unsigned int                     MH_CLNT_tag : 3;
        unsigned int                         RDC_RID : 3;
        unsigned int                       RDC_RRESP : 2;
        unsigned int                MH_CP_writeclean : 1;
        unsigned int                MH_RB_writeclean : 1;
        unsigned int                MH_PA_writeclean : 1;
        unsigned int                         BRC_BID : 3;
        unsigned int                       BRC_BRESP : 2;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                       BRC_BRESP : 2;
        unsigned int                         BRC_BID : 3;
        unsigned int                MH_PA_writeclean : 1;
        unsigned int                MH_RB_writeclean : 1;
        unsigned int                MH_CP_writeclean : 1;
        unsigned int                       RDC_RRESP : 2;
        unsigned int                         RDC_RID : 3;
        unsigned int                     MH_CLNT_tag : 3;
        unsigned int                   MH_CLNT_rlast : 1;
        unsigned int                    MH_TC_mcsend : 1;
        unsigned int                 MH_VGT_grb_send : 1;
        unsigned int                  MH_CP_grb_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG03 {
    struct {
#if     defined(qLittleEndian)
        unsigned int               MH_CLNT_data_31_0 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int               MH_CLNT_data_31_0 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG04 {
    struct {
#if     defined(qLittleEndian)
        unsigned int              MH_CLNT_data_63_32 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int              MH_CLNT_data_63_32 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG05 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      CP_MH_send : 1;
        unsigned int                     CP_MH_write : 1;
        unsigned int                       CP_MH_tag : 3;
        unsigned int                   CP_MH_ad_31_5 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                   CP_MH_ad_31_5 : 27;
        unsigned int                       CP_MH_tag : 3;
        unsigned int                     CP_MH_write : 1;
        unsigned int                      CP_MH_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG06 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 CP_MH_data_31_0 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 CP_MH_data_31_0 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG07 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                CP_MH_data_63_32 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                CP_MH_data_63_32 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG08 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        CP_MH_be : 8;
        unsigned int                        RB_MH_be : 8;
        unsigned int                        PA_MH_be : 8;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                        PA_MH_be : 8;
        unsigned int                        RB_MH_be : 8;
        unsigned int                        CP_MH_be : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG09 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     ALWAYS_ZERO : 3;
        unsigned int                     VGT_MH_send : 1;
        unsigned int                    VGT_MH_tagbe : 1;
        unsigned int                  VGT_MH_ad_31_5 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                  VGT_MH_ad_31_5 : 27;
        unsigned int                    VGT_MH_tagbe : 1;
        unsigned int                     VGT_MH_send : 1;
        unsigned int                     ALWAYS_ZERO : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG10 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                      TC_MH_send : 1;
        unsigned int                      TC_MH_mask : 2;
        unsigned int                 TC_MH_addr_31_5 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                 TC_MH_addr_31_5 : 27;
        unsigned int                      TC_MH_mask : 2;
        unsigned int                      TC_MH_send : 1;
        unsigned int                     ALWAYS_ZERO : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG11 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      TC_MH_info : 25;
        unsigned int                      TC_MH_send : 1;
        unsigned int                                 : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 6;
        unsigned int                      TC_MH_send : 1;
        unsigned int                      TC_MH_info : 25;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG12 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    MH_TC_mcinfo : 25;
        unsigned int               MH_TC_mcinfo_send : 1;
        unsigned int                   TC_MH_written : 1;
        unsigned int                                 : 5;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 5;
        unsigned int                   TC_MH_written : 1;
        unsigned int               MH_TC_mcinfo_send : 1;
        unsigned int                    MH_TC_mcinfo : 25;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG13 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                     TC_ROQ_SEND : 1;
        unsigned int                     TC_ROQ_MASK : 2;
        unsigned int                TC_ROQ_ADDR_31_5 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                TC_ROQ_ADDR_31_5 : 27;
        unsigned int                     TC_ROQ_MASK : 2;
        unsigned int                     TC_ROQ_SEND : 1;
        unsigned int                     ALWAYS_ZERO : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG14 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     TC_ROQ_INFO : 25;
        unsigned int                     TC_ROQ_SEND : 1;
        unsigned int                                 : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 6;
        unsigned int                     TC_ROQ_SEND : 1;
        unsigned int                     TC_ROQ_INFO : 25;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG15 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     ALWAYS_ZERO : 4;
        unsigned int                      RB_MH_send : 1;
        unsigned int                 RB_MH_addr_31_5 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                 RB_MH_addr_31_5 : 27;
        unsigned int                      RB_MH_send : 1;
        unsigned int                     ALWAYS_ZERO : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG16 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 RB_MH_data_31_0 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 RB_MH_data_31_0 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG17 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                RB_MH_data_63_32 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                RB_MH_data_63_32 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG18 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     ALWAYS_ZERO : 4;
        unsigned int                      PA_MH_send : 1;
        unsigned int                 PA_MH_addr_31_5 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PA_MH_addr_31_5 : 27;
        unsigned int                      PA_MH_send : 1;
        unsigned int                     ALWAYS_ZERO : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG19 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PA_MH_data_31_0 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PA_MH_data_31_0 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG20 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                PA_MH_data_63_32 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                PA_MH_data_63_32 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG21 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        AVALID_q : 1;
        unsigned int                        AREADY_q : 1;
        unsigned int                           AID_q : 3;
        unsigned int                      ALEN_q_2_0 : 3;
        unsigned int                       ARVALID_q : 1;
        unsigned int                       ARREADY_q : 1;
        unsigned int                          ARID_q : 3;
        unsigned int                     ARLEN_q_1_0 : 2;
        unsigned int                        RVALID_q : 1;
        unsigned int                        RREADY_q : 1;
        unsigned int                         RLAST_q : 1;
        unsigned int                           RID_q : 3;
        unsigned int                        WVALID_q : 1;
        unsigned int                        WREADY_q : 1;
        unsigned int                         WLAST_q : 1;
        unsigned int                           WID_q : 3;
        unsigned int                        BVALID_q : 1;
        unsigned int                        BREADY_q : 1;
        unsigned int                           BID_q : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                           BID_q : 3;
        unsigned int                        BREADY_q : 1;
        unsigned int                        BVALID_q : 1;
        unsigned int                           WID_q : 3;
        unsigned int                         WLAST_q : 1;
        unsigned int                        WREADY_q : 1;
        unsigned int                        WVALID_q : 1;
        unsigned int                           RID_q : 3;
        unsigned int                         RLAST_q : 1;
        unsigned int                        RREADY_q : 1;
        unsigned int                        RVALID_q : 1;
        unsigned int                     ARLEN_q_1_0 : 2;
        unsigned int                          ARID_q : 3;
        unsigned int                       ARREADY_q : 1;
        unsigned int                       ARVALID_q : 1;
        unsigned int                      ALEN_q_2_0 : 3;
        unsigned int                           AID_q : 3;
        unsigned int                        AREADY_q : 1;
        unsigned int                        AVALID_q : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG22 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        AVALID_q : 1;
        unsigned int                        AREADY_q : 1;
        unsigned int                           AID_q : 3;
        unsigned int                      ALEN_q_1_0 : 2;
        unsigned int                       ARVALID_q : 1;
        unsigned int                       ARREADY_q : 1;
        unsigned int                          ARID_q : 3;
        unsigned int                     ARLEN_q_1_1 : 1;
        unsigned int                        WVALID_q : 1;
        unsigned int                        WREADY_q : 1;
        unsigned int                         WLAST_q : 1;
        unsigned int                           WID_q : 3;
        unsigned int                         WSTRB_q : 8;
        unsigned int                        BVALID_q : 1;
        unsigned int                        BREADY_q : 1;
        unsigned int                           BID_q : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                           BID_q : 3;
        unsigned int                        BREADY_q : 1;
        unsigned int                        BVALID_q : 1;
        unsigned int                         WSTRB_q : 8;
        unsigned int                           WID_q : 3;
        unsigned int                         WLAST_q : 1;
        unsigned int                        WREADY_q : 1;
        unsigned int                        WVALID_q : 1;
        unsigned int                     ARLEN_q_1_1 : 1;
        unsigned int                          ARID_q : 3;
        unsigned int                       ARREADY_q : 1;
        unsigned int                       ARVALID_q : 1;
        unsigned int                      ALEN_q_1_0 : 2;
        unsigned int                           AID_q : 3;
        unsigned int                        AREADY_q : 1;
        unsigned int                        AVALID_q : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG23 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   ARC_CTRL_RE_q : 1;
        unsigned int                     CTRL_ARC_ID : 3;
        unsigned int                    CTRL_ARC_PAD : 28;
#else       /* !defined(qLittleEndian) */
        unsigned int                    CTRL_ARC_PAD : 28;
        unsigned int                     CTRL_ARC_ID : 3;
        unsigned int                   ARC_CTRL_RE_q : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG24 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                           REG_A : 14;
        unsigned int                          REG_RE : 1;
        unsigned int                          REG_WE : 1;
        unsigned int                        BLOCK_RS : 1;
        unsigned int                                 : 13;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 13;
        unsigned int                        BLOCK_RS : 1;
        unsigned int                          REG_WE : 1;
        unsigned int                          REG_RE : 1;
        unsigned int                           REG_A : 14;
        unsigned int                     ALWAYS_ZERO : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG25 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          REG_WD : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                          REG_WD : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG26 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    MH_RBBM_busy : 1;
        unsigned int            MH_CIB_mh_clk_en_int : 1;
        unsigned int           MH_CIB_mmu_clk_en_int : 1;
        unsigned int         MH_CIB_tcroq_clk_en_int : 1;
        unsigned int                     GAT_CLK_ENA : 1;
        unsigned int         RBBM_MH_clk_en_override : 1;
        unsigned int                           CNT_q : 6;
        unsigned int                     TCD_EMPTY_q : 1;
        unsigned int                    TC_ROQ_EMPTY : 1;
        unsigned int                       MH_BUSY_d : 1;
        unsigned int                   ANY_CLNT_BUSY : 1;
        unsigned int MH_MMU_INVALIDATE_INVALIDATE_ALL : 1;
        unsigned int MH_MMU_INVALIDATE_INVALIDATE_TC : 1;
        unsigned int                       CP_SEND_q : 1;
        unsigned int                        CP_RTR_q : 1;
        unsigned int                      VGT_SEND_q : 1;
        unsigned int                       VGT_RTR_q : 1;
        unsigned int                   TC_ROQ_SEND_q : 1;
        unsigned int                TC_ROQ_RTR_DBG_q : 1;
        unsigned int                       RB_SEND_q : 1;
        unsigned int                        RB_RTR_q : 1;
        unsigned int                       PA_SEND_q : 1;
        unsigned int                        PA_RTR_q : 1;
        unsigned int                       RDC_VALID : 1;
        unsigned int                       RDC_RLAST : 1;
        unsigned int                   TLBMISS_VALID : 1;
        unsigned int                       BRC_VALID : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                       BRC_VALID : 1;
        unsigned int                   TLBMISS_VALID : 1;
        unsigned int                       RDC_RLAST : 1;
        unsigned int                       RDC_VALID : 1;
        unsigned int                        PA_RTR_q : 1;
        unsigned int                       PA_SEND_q : 1;
        unsigned int                        RB_RTR_q : 1;
        unsigned int                       RB_SEND_q : 1;
        unsigned int                TC_ROQ_RTR_DBG_q : 1;
        unsigned int                   TC_ROQ_SEND_q : 1;
        unsigned int                       VGT_RTR_q : 1;
        unsigned int                      VGT_SEND_q : 1;
        unsigned int                        CP_RTR_q : 1;
        unsigned int                       CP_SEND_q : 1;
        unsigned int MH_MMU_INVALIDATE_INVALIDATE_TC : 1;
        unsigned int MH_MMU_INVALIDATE_INVALIDATE_ALL : 1;
        unsigned int                   ANY_CLNT_BUSY : 1;
        unsigned int                       MH_BUSY_d : 1;
        unsigned int                    TC_ROQ_EMPTY : 1;
        unsigned int                     TCD_EMPTY_q : 1;
        unsigned int                           CNT_q : 6;
        unsigned int         RBBM_MH_clk_en_override : 1;
        unsigned int                     GAT_CLK_ENA : 1;
        unsigned int         MH_CIB_tcroq_clk_en_int : 1;
        unsigned int           MH_CIB_mmu_clk_en_int : 1;
        unsigned int            MH_CIB_mh_clk_en_int : 1;
        unsigned int                    MH_RBBM_busy : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG27 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  EFF2_FP_WINNER : 3;
        unsigned int             EFF2_LRU_WINNER_out : 3;
        unsigned int                     EFF1_WINNER : 3;
        unsigned int                      ARB_WINNER : 3;
        unsigned int                    ARB_WINNER_q : 3;
        unsigned int                        EFF1_WIN : 1;
        unsigned int                       KILL_EFF1 : 1;
        unsigned int                        ARB_HOLD : 1;
        unsigned int                       ARB_RTR_q : 1;
        unsigned int                    CP_SEND_QUAL : 1;
        unsigned int                   VGT_SEND_QUAL : 1;
        unsigned int                    TC_SEND_QUAL : 1;
        unsigned int               TC_SEND_EFF1_QUAL : 1;
        unsigned int                    RB_SEND_QUAL : 1;
        unsigned int                    PA_SEND_QUAL : 1;
        unsigned int                        ARB_QUAL : 1;
        unsigned int                     CP_EFF1_REQ : 1;
        unsigned int                    VGT_EFF1_REQ : 1;
        unsigned int                     TC_EFF1_REQ : 1;
        unsigned int                     RB_EFF1_REQ : 1;
        unsigned int                  TCD_NEARFULL_q : 1;
        unsigned int                     TCHOLD_IP_q : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     TCHOLD_IP_q : 1;
        unsigned int                  TCD_NEARFULL_q : 1;
        unsigned int                     RB_EFF1_REQ : 1;
        unsigned int                     TC_EFF1_REQ : 1;
        unsigned int                    VGT_EFF1_REQ : 1;
        unsigned int                     CP_EFF1_REQ : 1;
        unsigned int                        ARB_QUAL : 1;
        unsigned int                    PA_SEND_QUAL : 1;
        unsigned int                    RB_SEND_QUAL : 1;
        unsigned int               TC_SEND_EFF1_QUAL : 1;
        unsigned int                    TC_SEND_QUAL : 1;
        unsigned int                   VGT_SEND_QUAL : 1;
        unsigned int                    CP_SEND_QUAL : 1;
        unsigned int                       ARB_RTR_q : 1;
        unsigned int                        ARB_HOLD : 1;
        unsigned int                       KILL_EFF1 : 1;
        unsigned int                        EFF1_WIN : 1;
        unsigned int                    ARB_WINNER_q : 3;
        unsigned int                      ARB_WINNER : 3;
        unsigned int                     EFF1_WINNER : 3;
        unsigned int             EFF2_LRU_WINNER_out : 3;
        unsigned int                  EFF2_FP_WINNER : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG28 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     EFF1_WINNER : 3;
        unsigned int                      ARB_WINNER : 3;
        unsigned int                    CP_SEND_QUAL : 1;
        unsigned int                   VGT_SEND_QUAL : 1;
        unsigned int                    TC_SEND_QUAL : 1;
        unsigned int               TC_SEND_EFF1_QUAL : 1;
        unsigned int                    RB_SEND_QUAL : 1;
        unsigned int                        ARB_QUAL : 1;
        unsigned int                     CP_EFF1_REQ : 1;
        unsigned int                    VGT_EFF1_REQ : 1;
        unsigned int                     TC_EFF1_REQ : 1;
        unsigned int                     RB_EFF1_REQ : 1;
        unsigned int                        EFF1_WIN : 1;
        unsigned int                       KILL_EFF1 : 1;
        unsigned int                  TCD_NEARFULL_q : 1;
        unsigned int                     TC_ARB_HOLD : 1;
        unsigned int                        ARB_HOLD : 1;
        unsigned int                       ARB_RTR_q : 1;
        unsigned int         SAME_PAGE_LIMIT_COUNT_q : 10;
#else       /* !defined(qLittleEndian) */
        unsigned int         SAME_PAGE_LIMIT_COUNT_q : 10;
        unsigned int                       ARB_RTR_q : 1;
        unsigned int                        ARB_HOLD : 1;
        unsigned int                     TC_ARB_HOLD : 1;
        unsigned int                  TCD_NEARFULL_q : 1;
        unsigned int                       KILL_EFF1 : 1;
        unsigned int                        EFF1_WIN : 1;
        unsigned int                     RB_EFF1_REQ : 1;
        unsigned int                     TC_EFF1_REQ : 1;
        unsigned int                    VGT_EFF1_REQ : 1;
        unsigned int                     CP_EFF1_REQ : 1;
        unsigned int                        ARB_QUAL : 1;
        unsigned int                    RB_SEND_QUAL : 1;
        unsigned int               TC_SEND_EFF1_QUAL : 1;
        unsigned int                    TC_SEND_QUAL : 1;
        unsigned int                   VGT_SEND_QUAL : 1;
        unsigned int                    CP_SEND_QUAL : 1;
        unsigned int                      ARB_WINNER : 3;
        unsigned int                     EFF1_WINNER : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG29 {
    struct {
#if     defined(qLittleEndian)
        unsigned int             EFF2_LRU_WINNER_out : 3;
        unsigned int            LEAST_RECENT_INDEX_d : 3;
        unsigned int                  LEAST_RECENT_d : 3;
        unsigned int           UPDATE_RECENT_STACK_d : 1;
        unsigned int                        ARB_HOLD : 1;
        unsigned int                       ARB_RTR_q : 1;
        unsigned int                        CLNT_REQ : 5;
        unsigned int                      RECENT_d_0 : 3;
        unsigned int                      RECENT_d_1 : 3;
        unsigned int                      RECENT_d_2 : 3;
        unsigned int                      RECENT_d_3 : 3;
        unsigned int                      RECENT_d_4 : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                      RECENT_d_4 : 3;
        unsigned int                      RECENT_d_3 : 3;
        unsigned int                      RECENT_d_2 : 3;
        unsigned int                      RECENT_d_1 : 3;
        unsigned int                      RECENT_d_0 : 3;
        unsigned int                        CLNT_REQ : 5;
        unsigned int                       ARB_RTR_q : 1;
        unsigned int                        ARB_HOLD : 1;
        unsigned int           UPDATE_RECENT_STACK_d : 1;
        unsigned int                  LEAST_RECENT_d : 3;
        unsigned int            LEAST_RECENT_INDEX_d : 3;
        unsigned int             EFF2_LRU_WINNER_out : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG30 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     TC_ARB_HOLD : 1;
        unsigned int          TC_NOROQ_SAME_ROW_BANK : 1;
        unsigned int            TC_ROQ_SAME_ROW_BANK : 1;
        unsigned int                  TCD_NEARFULL_q : 1;
        unsigned int                     TCHOLD_IP_q : 1;
        unsigned int                    TCHOLD_CNT_q : 3;
        unsigned int MH_ARBITER_CONFIG_TC_REORDER_ENABLE : 1;
        unsigned int                TC_ROQ_RTR_DBG_q : 1;
        unsigned int                   TC_ROQ_SEND_q : 1;
        unsigned int                   TC_MH_written : 1;
        unsigned int              TCD_FULLNESS_CNT_q : 7;
        unsigned int                   WBURST_ACTIVE : 1;
        unsigned int                         WLAST_q : 1;
        unsigned int                     WBURST_IP_q : 1;
        unsigned int                    WBURST_CNT_q : 3;
        unsigned int                    CP_SEND_QUAL : 1;
        unsigned int                     CP_MH_write : 1;
        unsigned int                    RB_SEND_QUAL : 1;
        unsigned int                    PA_SEND_QUAL : 1;
        unsigned int                      ARB_WINNER : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                      ARB_WINNER : 3;
        unsigned int                    PA_SEND_QUAL : 1;
        unsigned int                    RB_SEND_QUAL : 1;
        unsigned int                     CP_MH_write : 1;
        unsigned int                    CP_SEND_QUAL : 1;
        unsigned int                    WBURST_CNT_q : 3;
        unsigned int                     WBURST_IP_q : 1;
        unsigned int                         WLAST_q : 1;
        unsigned int                   WBURST_ACTIVE : 1;
        unsigned int              TCD_FULLNESS_CNT_q : 7;
        unsigned int                   TC_MH_written : 1;
        unsigned int                   TC_ROQ_SEND_q : 1;
        unsigned int                TC_ROQ_RTR_DBG_q : 1;
        unsigned int MH_ARBITER_CONFIG_TC_REORDER_ENABLE : 1;
        unsigned int                    TCHOLD_CNT_q : 3;
        unsigned int                     TCHOLD_IP_q : 1;
        unsigned int                  TCD_NEARFULL_q : 1;
        unsigned int            TC_ROQ_SAME_ROW_BANK : 1;
        unsigned int          TC_NOROQ_SAME_ROW_BANK : 1;
        unsigned int                     TC_ARB_HOLD : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG31 {
    struct {
#if     defined(qLittleEndian)
        unsigned int             RF_ARBITER_CONFIG_q : 26;
        unsigned int    MH_CLNT_AXI_ID_REUSE_MMUr_ID : 3;
        unsigned int                                 : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 3;
        unsigned int    MH_CLNT_AXI_ID_REUSE_MMUr_ID : 3;
        unsigned int             RF_ARBITER_CONFIG_q : 26;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG32 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 SAME_ROW_BANK_q : 8;
        unsigned int                      ROQ_MARK_q : 8;
        unsigned int                     ROQ_VALID_q : 8;
        unsigned int                      TC_MH_send : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                       KILL_EFF1 : 1;
        unsigned int        TC_ROQ_SAME_ROW_BANK_SEL : 1;
        unsigned int               ANY_SAME_ROW_BANK : 1;
        unsigned int                    TC_EFF1_QUAL : 1;
        unsigned int                    TC_ROQ_EMPTY : 1;
        unsigned int                     TC_ROQ_FULL : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     TC_ROQ_FULL : 1;
        unsigned int                    TC_ROQ_EMPTY : 1;
        unsigned int                    TC_EFF1_QUAL : 1;
        unsigned int               ANY_SAME_ROW_BANK : 1;
        unsigned int        TC_ROQ_SAME_ROW_BANK_SEL : 1;
        unsigned int                       KILL_EFF1 : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                      TC_MH_send : 1;
        unsigned int                     ROQ_VALID_q : 8;
        unsigned int                      ROQ_MARK_q : 8;
        unsigned int                 SAME_ROW_BANK_q : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG33 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 SAME_ROW_BANK_q : 8;
        unsigned int                      ROQ_MARK_d : 8;
        unsigned int                     ROQ_VALID_d : 8;
        unsigned int                      TC_MH_send : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                       KILL_EFF1 : 1;
        unsigned int        TC_ROQ_SAME_ROW_BANK_SEL : 1;
        unsigned int               ANY_SAME_ROW_BANK : 1;
        unsigned int                    TC_EFF1_QUAL : 1;
        unsigned int                    TC_ROQ_EMPTY : 1;
        unsigned int                     TC_ROQ_FULL : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     TC_ROQ_FULL : 1;
        unsigned int                    TC_ROQ_EMPTY : 1;
        unsigned int                    TC_EFF1_QUAL : 1;
        unsigned int               ANY_SAME_ROW_BANK : 1;
        unsigned int        TC_ROQ_SAME_ROW_BANK_SEL : 1;
        unsigned int                       KILL_EFF1 : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                      TC_MH_send : 1;
        unsigned int                     ROQ_VALID_d : 8;
        unsigned int                      ROQ_MARK_d : 8;
        unsigned int                 SAME_ROW_BANK_q : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG34 {
    struct {
#if     defined(qLittleEndian)
        unsigned int               SAME_ROW_BANK_WIN : 8;
        unsigned int               SAME_ROW_BANK_REQ : 8;
        unsigned int           NON_SAME_ROW_BANK_WIN : 8;
        unsigned int           NON_SAME_ROW_BANK_REQ : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int           NON_SAME_ROW_BANK_REQ : 8;
        unsigned int           NON_SAME_ROW_BANK_WIN : 8;
        unsigned int               SAME_ROW_BANK_REQ : 8;
        unsigned int               SAME_ROW_BANK_WIN : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG35 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      TC_MH_send : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                    ROQ_MARK_q_0 : 1;
        unsigned int                   ROQ_VALID_q_0 : 1;
        unsigned int               SAME_ROW_BANK_q_0 : 1;
        unsigned int                      ROQ_ADDR_0 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                      ROQ_ADDR_0 : 27;
        unsigned int               SAME_ROW_BANK_q_0 : 1;
        unsigned int                   ROQ_VALID_q_0 : 1;
        unsigned int                    ROQ_MARK_q_0 : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                      TC_MH_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG36 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      TC_MH_send : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                    ROQ_MARK_q_1 : 1;
        unsigned int                   ROQ_VALID_q_1 : 1;
        unsigned int               SAME_ROW_BANK_q_1 : 1;
        unsigned int                      ROQ_ADDR_1 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                      ROQ_ADDR_1 : 27;
        unsigned int               SAME_ROW_BANK_q_1 : 1;
        unsigned int                   ROQ_VALID_q_1 : 1;
        unsigned int                    ROQ_MARK_q_1 : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                      TC_MH_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG37 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      TC_MH_send : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                    ROQ_MARK_q_2 : 1;
        unsigned int                   ROQ_VALID_q_2 : 1;
        unsigned int               SAME_ROW_BANK_q_2 : 1;
        unsigned int                      ROQ_ADDR_2 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                      ROQ_ADDR_2 : 27;
        unsigned int               SAME_ROW_BANK_q_2 : 1;
        unsigned int                   ROQ_VALID_q_2 : 1;
        unsigned int                    ROQ_MARK_q_2 : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                      TC_MH_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG38 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      TC_MH_send : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                    ROQ_MARK_q_3 : 1;
        unsigned int                   ROQ_VALID_q_3 : 1;
        unsigned int               SAME_ROW_BANK_q_3 : 1;
        unsigned int                      ROQ_ADDR_3 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                      ROQ_ADDR_3 : 27;
        unsigned int               SAME_ROW_BANK_q_3 : 1;
        unsigned int                   ROQ_VALID_q_3 : 1;
        unsigned int                    ROQ_MARK_q_3 : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                      TC_MH_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG39 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      TC_MH_send : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                    ROQ_MARK_q_4 : 1;
        unsigned int                   ROQ_VALID_q_4 : 1;
        unsigned int               SAME_ROW_BANK_q_4 : 1;
        unsigned int                      ROQ_ADDR_4 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                      ROQ_ADDR_4 : 27;
        unsigned int               SAME_ROW_BANK_q_4 : 1;
        unsigned int                   ROQ_VALID_q_4 : 1;
        unsigned int                    ROQ_MARK_q_4 : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                      TC_MH_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG40 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      TC_MH_send : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                    ROQ_MARK_q_5 : 1;
        unsigned int                   ROQ_VALID_q_5 : 1;
        unsigned int               SAME_ROW_BANK_q_5 : 1;
        unsigned int                      ROQ_ADDR_5 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                      ROQ_ADDR_5 : 27;
        unsigned int               SAME_ROW_BANK_q_5 : 1;
        unsigned int                   ROQ_VALID_q_5 : 1;
        unsigned int                    ROQ_MARK_q_5 : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                      TC_MH_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG41 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      TC_MH_send : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                    ROQ_MARK_q_6 : 1;
        unsigned int                   ROQ_VALID_q_6 : 1;
        unsigned int               SAME_ROW_BANK_q_6 : 1;
        unsigned int                      ROQ_ADDR_6 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                      ROQ_ADDR_6 : 27;
        unsigned int               SAME_ROW_BANK_q_6 : 1;
        unsigned int                   ROQ_VALID_q_6 : 1;
        unsigned int                    ROQ_MARK_q_6 : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                      TC_MH_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG42 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      TC_MH_send : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                    ROQ_MARK_q_7 : 1;
        unsigned int                   ROQ_VALID_q_7 : 1;
        unsigned int               SAME_ROW_BANK_q_7 : 1;
        unsigned int                      ROQ_ADDR_7 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                      ROQ_ADDR_7 : 27;
        unsigned int               SAME_ROW_BANK_q_7 : 1;
        unsigned int                   ROQ_VALID_q_7 : 1;
        unsigned int                    ROQ_MARK_q_7 : 1;
        unsigned int                    TC_ROQ_RTR_q : 1;
        unsigned int                      TC_MH_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG43 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    ARB_REG_WE_q : 1;
        unsigned int                          ARB_WE : 1;
        unsigned int                 ARB_REG_VALID_q : 1;
        unsigned int                       ARB_RTR_q : 1;
        unsigned int                     ARB_REG_RTR : 1;
        unsigned int                  WDAT_BURST_RTR : 1;
        unsigned int                         MMU_RTR : 1;
        unsigned int                        ARB_ID_q : 3;
        unsigned int                     ARB_WRITE_q : 1;
        unsigned int                      ARB_BLEN_q : 1;
        unsigned int                  ARQ_CTRL_EMPTY : 1;
        unsigned int                  ARQ_FIFO_CNT_q : 3;
        unsigned int                          MMU_WE : 1;
        unsigned int                         ARQ_RTR : 1;
        unsigned int                          MMU_ID : 3;
        unsigned int                       MMU_WRITE : 1;
        unsigned int                        MMU_BLEN : 1;
        unsigned int                     WBURST_IP_q : 1;
        unsigned int                   WDAT_REG_WE_q : 1;
        unsigned int                          WDB_WE : 1;
        unsigned int                  WDB_RTR_SKID_4 : 1;
        unsigned int                  WDB_RTR_SKID_3 : 1;
        unsigned int                                 : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 4;
        unsigned int                  WDB_RTR_SKID_3 : 1;
        unsigned int                  WDB_RTR_SKID_4 : 1;
        unsigned int                          WDB_WE : 1;
        unsigned int                   WDAT_REG_WE_q : 1;
        unsigned int                     WBURST_IP_q : 1;
        unsigned int                        MMU_BLEN : 1;
        unsigned int                       MMU_WRITE : 1;
        unsigned int                          MMU_ID : 3;
        unsigned int                         ARQ_RTR : 1;
        unsigned int                          MMU_WE : 1;
        unsigned int                  ARQ_FIFO_CNT_q : 3;
        unsigned int                  ARQ_CTRL_EMPTY : 1;
        unsigned int                      ARB_BLEN_q : 1;
        unsigned int                     ARB_WRITE_q : 1;
        unsigned int                        ARB_ID_q : 3;
        unsigned int                         MMU_RTR : 1;
        unsigned int                  WDAT_BURST_RTR : 1;
        unsigned int                     ARB_REG_RTR : 1;
        unsigned int                       ARB_RTR_q : 1;
        unsigned int                 ARB_REG_VALID_q : 1;
        unsigned int                          ARB_WE : 1;
        unsigned int                    ARB_REG_WE_q : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG44 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          ARB_WE : 1;
        unsigned int                        ARB_ID_q : 3;
        unsigned int                       ARB_VAD_q : 28;
#else       /* !defined(qLittleEndian) */
        unsigned int                       ARB_VAD_q : 28;
        unsigned int                        ARB_ID_q : 3;
        unsigned int                          ARB_WE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG45 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                          MMU_WE : 1;
        unsigned int                          MMU_ID : 3;
        unsigned int                         MMU_PAD : 28;
#else       /* !defined(qLittleEndian) */
        unsigned int                         MMU_PAD : 28;
        unsigned int                          MMU_ID : 3;
        unsigned int                          MMU_WE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG46 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   WDAT_REG_WE_q : 1;
        unsigned int                          WDB_WE : 1;
        unsigned int                WDAT_REG_VALID_q : 1;
        unsigned int                  WDB_RTR_SKID_4 : 1;
        unsigned int                     ARB_WSTRB_q : 8;
        unsigned int                       ARB_WLAST : 1;
        unsigned int                  WDB_CTRL_EMPTY : 1;
        unsigned int                  WDB_FIFO_CNT_q : 5;
        unsigned int                    WDC_WDB_RE_q : 1;
        unsigned int                     WDB_WDC_WID : 3;
        unsigned int                   WDB_WDC_WLAST : 1;
        unsigned int                   WDB_WDC_WSTRB : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                   WDB_WDC_WSTRB : 8;
        unsigned int                   WDB_WDC_WLAST : 1;
        unsigned int                     WDB_WDC_WID : 3;
        unsigned int                    WDC_WDB_RE_q : 1;
        unsigned int                  WDB_FIFO_CNT_q : 5;
        unsigned int                  WDB_CTRL_EMPTY : 1;
        unsigned int                       ARB_WLAST : 1;
        unsigned int                     ARB_WSTRB_q : 8;
        unsigned int                  WDB_RTR_SKID_4 : 1;
        unsigned int                WDAT_REG_VALID_q : 1;
        unsigned int                          WDB_WE : 1;
        unsigned int                   WDAT_REG_WE_q : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG47 {
    struct {
#if     defined(qLittleEndian)
        unsigned int              WDB_WDC_WDATA_31_0 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int              WDB_WDC_WDATA_31_0 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG48 {
    struct {
#if     defined(qLittleEndian)
        unsigned int             WDB_WDC_WDATA_63_32 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int             WDB_WDC_WDATA_63_32 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG49 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  CTRL_ARC_EMPTY : 1;
        unsigned int                 CTRL_RARC_EMPTY : 1;
        unsigned int                  ARQ_CTRL_EMPTY : 1;
        unsigned int                  ARQ_CTRL_WRITE : 1;
        unsigned int                TLBMISS_CTRL_RTS : 1;
        unsigned int               CTRL_TLBMISS_RE_q : 1;
        unsigned int                   INFLT_LIMIT_q : 1;
        unsigned int               INFLT_LIMIT_CNT_q : 6;
        unsigned int                   ARC_CTRL_RE_q : 1;
        unsigned int                  RARC_CTRL_RE_q : 1;
        unsigned int                        RVALID_q : 1;
        unsigned int                        RREADY_q : 1;
        unsigned int                         RLAST_q : 1;
        unsigned int                        BVALID_q : 1;
        unsigned int                        BREADY_q : 1;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                        BREADY_q : 1;
        unsigned int                        BVALID_q : 1;
        unsigned int                         RLAST_q : 1;
        unsigned int                        RREADY_q : 1;
        unsigned int                        RVALID_q : 1;
        unsigned int                  RARC_CTRL_RE_q : 1;
        unsigned int                   ARC_CTRL_RE_q : 1;
        unsigned int               INFLT_LIMIT_CNT_q : 6;
        unsigned int                   INFLT_LIMIT_q : 1;
        unsigned int               CTRL_TLBMISS_RE_q : 1;
        unsigned int                TLBMISS_CTRL_RTS : 1;
        unsigned int                  ARQ_CTRL_WRITE : 1;
        unsigned int                  ARQ_CTRL_EMPTY : 1;
        unsigned int                 CTRL_RARC_EMPTY : 1;
        unsigned int                  CTRL_ARC_EMPTY : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG50 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  MH_CP_grb_send : 1;
        unsigned int                 MH_VGT_grb_send : 1;
        unsigned int                    MH_TC_mcsend : 1;
        unsigned int                 MH_TLBMISS_SEND : 1;
        unsigned int                   TLBMISS_VALID : 1;
        unsigned int                       RDC_VALID : 1;
        unsigned int                         RDC_RID : 3;
        unsigned int                       RDC_RLAST : 1;
        unsigned int                       RDC_RRESP : 2;
        unsigned int                TLBMISS_CTRL_RTS : 1;
        unsigned int               CTRL_TLBMISS_RE_q : 1;
        unsigned int                MMU_ID_REQUEST_q : 1;
        unsigned int         OUTSTANDING_MMUID_CNT_q : 6;
        unsigned int                 MMU_ID_RESPONSE : 1;
        unsigned int            TLBMISS_RETURN_CNT_q : 6;
        unsigned int                     CNT_HOLD_q1 : 1;
        unsigned int    MH_CLNT_AXI_ID_REUSE_MMUr_ID : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int    MH_CLNT_AXI_ID_REUSE_MMUr_ID : 3;
        unsigned int                     CNT_HOLD_q1 : 1;
        unsigned int            TLBMISS_RETURN_CNT_q : 6;
        unsigned int                 MMU_ID_RESPONSE : 1;
        unsigned int         OUTSTANDING_MMUID_CNT_q : 6;
        unsigned int                MMU_ID_REQUEST_q : 1;
        unsigned int               CTRL_TLBMISS_RE_q : 1;
        unsigned int                TLBMISS_CTRL_RTS : 1;
        unsigned int                       RDC_RRESP : 2;
        unsigned int                       RDC_RLAST : 1;
        unsigned int                         RDC_RID : 3;
        unsigned int                       RDC_VALID : 1;
        unsigned int                   TLBMISS_VALID : 1;
        unsigned int                 MH_TLBMISS_SEND : 1;
        unsigned int                    MH_TC_mcsend : 1;
        unsigned int                 MH_VGT_grb_send : 1;
        unsigned int                  MH_CP_grb_send : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG51 {
    struct {
#if     defined(qLittleEndian)
        unsigned int               RF_MMU_PAGE_FAULT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int               RF_MMU_PAGE_FAULT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG52 {
    struct {
#if     defined(qLittleEndian)
        unsigned int          RF_MMU_CONFIG_q_1_to_0 : 2;
        unsigned int                          ARB_WE : 1;
        unsigned int                         MMU_RTR : 1;
        unsigned int         RF_MMU_CONFIG_q_25_to_4 : 22;
        unsigned int                        ARB_ID_q : 3;
        unsigned int                     ARB_WRITE_q : 1;
        unsigned int               client_behavior_q : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int               client_behavior_q : 2;
        unsigned int                     ARB_WRITE_q : 1;
        unsigned int                        ARB_ID_q : 3;
        unsigned int         RF_MMU_CONFIG_q_25_to_4 : 22;
        unsigned int                         MMU_RTR : 1;
        unsigned int                          ARB_WE : 1;
        unsigned int          RF_MMU_CONFIG_q_1_to_0 : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG53 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    stage1_valid : 1;
        unsigned int               IGNORE_TAG_MISS_q : 1;
        unsigned int                 pa_in_mpu_range : 1;
        unsigned int                     tag_match_q : 1;
        unsigned int                      tag_miss_q : 1;
        unsigned int                   va_in_range_q : 1;
        unsigned int                        MMU_MISS : 1;
        unsigned int                   MMU_READ_MISS : 1;
        unsigned int                  MMU_WRITE_MISS : 1;
        unsigned int                         MMU_HIT : 1;
        unsigned int                    MMU_READ_HIT : 1;
        unsigned int                   MMU_WRITE_HIT : 1;
        unsigned int          MMU_SPLIT_MODE_TC_MISS : 1;
        unsigned int           MMU_SPLIT_MODE_TC_HIT : 1;
        unsigned int       MMU_SPLIT_MODE_nonTC_MISS : 1;
        unsigned int        MMU_SPLIT_MODE_nonTC_HIT : 1;
        unsigned int                 REQ_VA_OFFSET_q : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                 REQ_VA_OFFSET_q : 16;
        unsigned int        MMU_SPLIT_MODE_nonTC_HIT : 1;
        unsigned int       MMU_SPLIT_MODE_nonTC_MISS : 1;
        unsigned int           MMU_SPLIT_MODE_TC_HIT : 1;
        unsigned int          MMU_SPLIT_MODE_TC_MISS : 1;
        unsigned int                   MMU_WRITE_HIT : 1;
        unsigned int                    MMU_READ_HIT : 1;
        unsigned int                         MMU_HIT : 1;
        unsigned int                  MMU_WRITE_MISS : 1;
        unsigned int                   MMU_READ_MISS : 1;
        unsigned int                        MMU_MISS : 1;
        unsigned int                   va_in_range_q : 1;
        unsigned int                      tag_miss_q : 1;
        unsigned int                     tag_match_q : 1;
        unsigned int                 pa_in_mpu_range : 1;
        unsigned int               IGNORE_TAG_MISS_q : 1;
        unsigned int                    stage1_valid : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG54 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         ARQ_RTR : 1;
        unsigned int                          MMU_WE : 1;
        unsigned int               CTRL_TLBMISS_RE_q : 1;
        unsigned int                TLBMISS_CTRL_RTS : 1;
        unsigned int                 MH_TLBMISS_SEND : 1;
        unsigned int MMU_STALL_AWAITING_TLB_MISS_FETCH : 1;
        unsigned int                 pa_in_mpu_range : 1;
        unsigned int                    stage1_valid : 1;
        unsigned int                    stage2_valid : 1;
        unsigned int               client_behavior_q : 2;
        unsigned int               IGNORE_TAG_MISS_q : 1;
        unsigned int                     tag_match_q : 1;
        unsigned int                      tag_miss_q : 1;
        unsigned int                   va_in_range_q : 1;
        unsigned int            PTE_FETCH_COMPLETE_q : 1;
        unsigned int                     TAG_valid_q : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                     TAG_valid_q : 16;
        unsigned int            PTE_FETCH_COMPLETE_q : 1;
        unsigned int                   va_in_range_q : 1;
        unsigned int                      tag_miss_q : 1;
        unsigned int                     tag_match_q : 1;
        unsigned int               IGNORE_TAG_MISS_q : 1;
        unsigned int               client_behavior_q : 2;
        unsigned int                    stage2_valid : 1;
        unsigned int                    stage1_valid : 1;
        unsigned int                 pa_in_mpu_range : 1;
        unsigned int MMU_STALL_AWAITING_TLB_MISS_FETCH : 1;
        unsigned int                 MH_TLBMISS_SEND : 1;
        unsigned int                TLBMISS_CTRL_RTS : 1;
        unsigned int               CTRL_TLBMISS_RE_q : 1;
        unsigned int                          MMU_WE : 1;
        unsigned int                         ARQ_RTR : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG55 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         TAG0_VA : 13;
        unsigned int                   TAG_valid_q_0 : 1;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                         TAG1_VA : 13;
        unsigned int                   TAG_valid_q_1 : 1;
        unsigned int                                 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 2;
        unsigned int                   TAG_valid_q_1 : 1;
        unsigned int                         TAG1_VA : 13;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                   TAG_valid_q_0 : 1;
        unsigned int                         TAG0_VA : 13;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG56 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         TAG2_VA : 13;
        unsigned int                   TAG_valid_q_2 : 1;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                         TAG3_VA : 13;
        unsigned int                   TAG_valid_q_3 : 1;
        unsigned int                                 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 2;
        unsigned int                   TAG_valid_q_3 : 1;
        unsigned int                         TAG3_VA : 13;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                   TAG_valid_q_2 : 1;
        unsigned int                         TAG2_VA : 13;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG57 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         TAG4_VA : 13;
        unsigned int                   TAG_valid_q_4 : 1;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                         TAG5_VA : 13;
        unsigned int                   TAG_valid_q_5 : 1;
        unsigned int                                 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 2;
        unsigned int                   TAG_valid_q_5 : 1;
        unsigned int                         TAG5_VA : 13;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                   TAG_valid_q_4 : 1;
        unsigned int                         TAG4_VA : 13;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG58 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         TAG6_VA : 13;
        unsigned int                   TAG_valid_q_6 : 1;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                         TAG7_VA : 13;
        unsigned int                   TAG_valid_q_7 : 1;
        unsigned int                                 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 2;
        unsigned int                   TAG_valid_q_7 : 1;
        unsigned int                         TAG7_VA : 13;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                   TAG_valid_q_6 : 1;
        unsigned int                         TAG6_VA : 13;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG59 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         TAG8_VA : 13;
        unsigned int                   TAG_valid_q_8 : 1;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                         TAG9_VA : 13;
        unsigned int                   TAG_valid_q_9 : 1;
        unsigned int                                 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 2;
        unsigned int                   TAG_valid_q_9 : 1;
        unsigned int                         TAG9_VA : 13;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                   TAG_valid_q_8 : 1;
        unsigned int                         TAG8_VA : 13;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG60 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        TAG10_VA : 13;
        unsigned int                  TAG_valid_q_10 : 1;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                        TAG11_VA : 13;
        unsigned int                  TAG_valid_q_11 : 1;
        unsigned int                                 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 2;
        unsigned int                  TAG_valid_q_11 : 1;
        unsigned int                        TAG11_VA : 13;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                  TAG_valid_q_10 : 1;
        unsigned int                        TAG10_VA : 13;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG61 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        TAG12_VA : 13;
        unsigned int                  TAG_valid_q_12 : 1;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                        TAG13_VA : 13;
        unsigned int                  TAG_valid_q_13 : 1;
        unsigned int                                 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 2;
        unsigned int                  TAG_valid_q_13 : 1;
        unsigned int                        TAG13_VA : 13;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                  TAG_valid_q_12 : 1;
        unsigned int                        TAG12_VA : 13;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG62 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        TAG14_VA : 13;
        unsigned int                  TAG_valid_q_14 : 1;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                        TAG15_VA : 13;
        unsigned int                  TAG_valid_q_15 : 1;
        unsigned int                                 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 2;
        unsigned int                  TAG_valid_q_15 : 1;
        unsigned int                        TAG15_VA : 13;
        unsigned int                     ALWAYS_ZERO : 2;
        unsigned int                  TAG_valid_q_14 : 1;
        unsigned int                        TAG14_VA : 13;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_DEBUG_REG63 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  MH_DBG_DEFAULT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                  MH_DBG_DEFAULT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_MMU_CONFIG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      MMU_ENABLE : 1;
        unsigned int               SPLIT_MODE_ENABLE : 1;
        unsigned int                       RESERVED1 : 2;
        unsigned int              RB_W_CLNT_BEHAVIOR : 2;
        unsigned int              CP_W_CLNT_BEHAVIOR : 2;
        unsigned int             CP_R0_CLNT_BEHAVIOR : 2;
        unsigned int             CP_R1_CLNT_BEHAVIOR : 2;
        unsigned int             CP_R2_CLNT_BEHAVIOR : 2;
        unsigned int             CP_R3_CLNT_BEHAVIOR : 2;
        unsigned int             CP_R4_CLNT_BEHAVIOR : 2;
        unsigned int            VGT_R0_CLNT_BEHAVIOR : 2;
        unsigned int            VGT_R1_CLNT_BEHAVIOR : 2;
        unsigned int              TC_R_CLNT_BEHAVIOR : 2;
        unsigned int              PA_W_CLNT_BEHAVIOR : 2;
        unsigned int                                 : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 6;
        unsigned int              PA_W_CLNT_BEHAVIOR : 2;
        unsigned int              TC_R_CLNT_BEHAVIOR : 2;
        unsigned int            VGT_R1_CLNT_BEHAVIOR : 2;
        unsigned int            VGT_R0_CLNT_BEHAVIOR : 2;
        unsigned int             CP_R4_CLNT_BEHAVIOR : 2;
        unsigned int             CP_R3_CLNT_BEHAVIOR : 2;
        unsigned int             CP_R2_CLNT_BEHAVIOR : 2;
        unsigned int             CP_R1_CLNT_BEHAVIOR : 2;
        unsigned int             CP_R0_CLNT_BEHAVIOR : 2;
        unsigned int              CP_W_CLNT_BEHAVIOR : 2;
        unsigned int              RB_W_CLNT_BEHAVIOR : 2;
        unsigned int                       RESERVED1 : 2;
        unsigned int               SPLIT_MODE_ENABLE : 1;
        unsigned int                      MMU_ENABLE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_MMU_VA_RANGE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                NUM_64KB_REGIONS : 12;
        unsigned int                         VA_BASE : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                         VA_BASE : 20;
        unsigned int                NUM_64KB_REGIONS : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_MMU_PT_BASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                         PT_BASE : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                         PT_BASE : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_MMU_PAGE_FAULT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PAGE_FAULT : 1;
        unsigned int                         OP_TYPE : 1;
        unsigned int                   CLNT_BEHAVIOR : 2;
        unsigned int                          AXI_ID : 3;
        unsigned int                       RESERVED1 : 1;
        unsigned int        MPU_ADDRESS_OUT_OF_RANGE : 1;
        unsigned int            ADDRESS_OUT_OF_RANGE : 1;
        unsigned int           READ_PROTECTION_ERROR : 1;
        unsigned int          WRITE_PROTECTION_ERROR : 1;
        unsigned int                          REQ_VA : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                          REQ_VA : 20;
        unsigned int          WRITE_PROTECTION_ERROR : 1;
        unsigned int           READ_PROTECTION_ERROR : 1;
        unsigned int            ADDRESS_OUT_OF_RANGE : 1;
        unsigned int        MPU_ADDRESS_OUT_OF_RANGE : 1;
        unsigned int                       RESERVED1 : 1;
        unsigned int                          AXI_ID : 3;
        unsigned int                   CLNT_BEHAVIOR : 2;
        unsigned int                         OP_TYPE : 1;
        unsigned int                      PAGE_FAULT : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_MMU_TRAN_ERROR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 5;
        unsigned int                      TRAN_ERROR : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                      TRAN_ERROR : 27;
        unsigned int                                 : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_MMU_INVALIDATE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  INVALIDATE_ALL : 1;
        unsigned int                   INVALIDATE_TC : 1;
        unsigned int                                 : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 30;
        unsigned int                   INVALIDATE_TC : 1;
        unsigned int                  INVALIDATE_ALL : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_MMU_MPU_BASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                        MPU_BASE : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                        MPU_BASE : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MH_MMU_MPU_END {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                         MPU_END : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                         MPU_END : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union WAIT_UNTIL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 1;
        unsigned int                   WAIT_RE_VSYNC : 1;
        unsigned int                   WAIT_FE_VSYNC : 1;
        unsigned int                      WAIT_VSYNC : 1;
        unsigned int                  WAIT_DSPLY_ID0 : 1;
        unsigned int                  WAIT_DSPLY_ID1 : 1;
        unsigned int                  WAIT_DSPLY_ID2 : 1;
        unsigned int                                 : 3;
        unsigned int                    WAIT_CMDFIFO : 1;
        unsigned int                                 : 3;
        unsigned int                    WAIT_2D_IDLE : 1;
        unsigned int                    WAIT_3D_IDLE : 1;
        unsigned int               WAIT_2D_IDLECLEAN : 1;
        unsigned int               WAIT_3D_IDLECLEAN : 1;
        unsigned int                                 : 2;
        unsigned int                 CMDFIFO_ENTRIES : 4;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                 CMDFIFO_ENTRIES : 4;
        unsigned int                                 : 2;
        unsigned int               WAIT_3D_IDLECLEAN : 1;
        unsigned int               WAIT_2D_IDLECLEAN : 1;
        unsigned int                    WAIT_3D_IDLE : 1;
        unsigned int                    WAIT_2D_IDLE : 1;
        unsigned int                                 : 3;
        unsigned int                    WAIT_CMDFIFO : 1;
        unsigned int                                 : 3;
        unsigned int                  WAIT_DSPLY_ID2 : 1;
        unsigned int                  WAIT_DSPLY_ID1 : 1;
        unsigned int                  WAIT_DSPLY_ID0 : 1;
        unsigned int                      WAIT_VSYNC : 1;
        unsigned int                   WAIT_FE_VSYNC : 1;
        unsigned int                   WAIT_RE_VSYNC : 1;
        unsigned int                                 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_ISYNC_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 4;
        unsigned int              ISYNC_WAIT_IDLEGUI : 1;
        unsigned int         ISYNC_CPSCRATCH_IDLEGUI : 1;
        unsigned int                                 : 26;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 26;
        unsigned int         ISYNC_CPSCRATCH_IDLEGUI : 1;
        unsigned int              ISYNC_WAIT_IDLEGUI : 1;
        unsigned int                                 : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   CMDFIFO_AVAIL : 5;
        unsigned int                         TC_BUSY : 1;
        unsigned int                                 : 2;
        unsigned int                    HIRQ_PENDING : 1;
        unsigned int                    CPRQ_PENDING : 1;
        unsigned int                    CFRQ_PENDING : 1;
        unsigned int                    PFRQ_PENDING : 1;
        unsigned int                 VGT_BUSY_NO_DMA : 1;
        unsigned int                                 : 1;
        unsigned int                    RBBM_WU_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                     CP_NRT_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                         MH_BUSY : 1;
        unsigned int               MH_COHERENCY_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                         SX_BUSY : 1;
        unsigned int                        TPC_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                    SC_CNTX_BUSY : 1;
        unsigned int                         PA_BUSY : 1;
        unsigned int                        VGT_BUSY : 1;
        unsigned int                  SQ_CNTX17_BUSY : 1;
        unsigned int                   SQ_CNTX0_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                    RB_CNTX_BUSY : 1;
        unsigned int                      GUI_ACTIVE : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                      GUI_ACTIVE : 1;
        unsigned int                    RB_CNTX_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                   SQ_CNTX0_BUSY : 1;
        unsigned int                  SQ_CNTX17_BUSY : 1;
        unsigned int                        VGT_BUSY : 1;
        unsigned int                         PA_BUSY : 1;
        unsigned int                    SC_CNTX_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                        TPC_BUSY : 1;
        unsigned int                         SX_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int               MH_COHERENCY_BUSY : 1;
        unsigned int                         MH_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                     CP_NRT_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                    RBBM_WU_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                 VGT_BUSY_NO_DMA : 1;
        unsigned int                    PFRQ_PENDING : 1;
        unsigned int                    CFRQ_PENDING : 1;
        unsigned int                    CPRQ_PENDING : 1;
        unsigned int                    HIRQ_PENDING : 1;
        unsigned int                                 : 2;
        unsigned int                         TC_BUSY : 1;
        unsigned int                   CMDFIFO_AVAIL : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_DSPLY {
    struct {
#if     defined(qLittleEndian)
        unsigned int           SEL_DMI_ACTIVE_BUFID0 : 1;
        unsigned int           SEL_DMI_ACTIVE_BUFID1 : 1;
        unsigned int           SEL_DMI_ACTIVE_BUFID2 : 1;
        unsigned int             SEL_DMI_VSYNC_VALID : 1;
        unsigned int              DMI_CH1_USE_BUFID0 : 1;
        unsigned int              DMI_CH1_USE_BUFID1 : 1;
        unsigned int              DMI_CH1_USE_BUFID2 : 1;
        unsigned int                 DMI_CH1_SW_CNTL : 1;
        unsigned int                DMI_CH1_NUM_BUFS : 2;
        unsigned int              DMI_CH2_USE_BUFID0 : 1;
        unsigned int              DMI_CH2_USE_BUFID1 : 1;
        unsigned int              DMI_CH2_USE_BUFID2 : 1;
        unsigned int                 DMI_CH2_SW_CNTL : 1;
        unsigned int                DMI_CH2_NUM_BUFS : 2;
        unsigned int              DMI_CHANNEL_SELECT : 2;
        unsigned int                                 : 2;
        unsigned int              DMI_CH3_USE_BUFID0 : 1;
        unsigned int              DMI_CH3_USE_BUFID1 : 1;
        unsigned int              DMI_CH3_USE_BUFID2 : 1;
        unsigned int                 DMI_CH3_SW_CNTL : 1;
        unsigned int                DMI_CH3_NUM_BUFS : 2;
        unsigned int              DMI_CH4_USE_BUFID0 : 1;
        unsigned int              DMI_CH4_USE_BUFID1 : 1;
        unsigned int              DMI_CH4_USE_BUFID2 : 1;
        unsigned int                 DMI_CH4_SW_CNTL : 1;
        unsigned int                DMI_CH4_NUM_BUFS : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int                DMI_CH4_NUM_BUFS : 2;
        unsigned int                 DMI_CH4_SW_CNTL : 1;
        unsigned int              DMI_CH4_USE_BUFID2 : 1;
        unsigned int              DMI_CH4_USE_BUFID1 : 1;
        unsigned int              DMI_CH4_USE_BUFID0 : 1;
        unsigned int                DMI_CH3_NUM_BUFS : 2;
        unsigned int                 DMI_CH3_SW_CNTL : 1;
        unsigned int              DMI_CH3_USE_BUFID2 : 1;
        unsigned int              DMI_CH3_USE_BUFID1 : 1;
        unsigned int              DMI_CH3_USE_BUFID0 : 1;
        unsigned int                                 : 2;
        unsigned int              DMI_CHANNEL_SELECT : 2;
        unsigned int                DMI_CH2_NUM_BUFS : 2;
        unsigned int                 DMI_CH2_SW_CNTL : 1;
        unsigned int              DMI_CH2_USE_BUFID2 : 1;
        unsigned int              DMI_CH2_USE_BUFID1 : 1;
        unsigned int              DMI_CH2_USE_BUFID0 : 1;
        unsigned int                DMI_CH1_NUM_BUFS : 2;
        unsigned int                 DMI_CH1_SW_CNTL : 1;
        unsigned int              DMI_CH1_USE_BUFID2 : 1;
        unsigned int              DMI_CH1_USE_BUFID1 : 1;
        unsigned int              DMI_CH1_USE_BUFID0 : 1;
        unsigned int             SEL_DMI_VSYNC_VALID : 1;
        unsigned int           SEL_DMI_ACTIVE_BUFID2 : 1;
        unsigned int           SEL_DMI_ACTIVE_BUFID1 : 1;
        unsigned int           SEL_DMI_ACTIVE_BUFID0 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_RENDER_LATEST {
    struct {
#if     defined(qLittleEndian)
        unsigned int               DMI_CH1_BUFFER_ID : 2;
        unsigned int                                 : 6;
        unsigned int               DMI_CH2_BUFFER_ID : 2;
        unsigned int                                 : 6;
        unsigned int               DMI_CH3_BUFFER_ID : 2;
        unsigned int                                 : 6;
        unsigned int               DMI_CH4_BUFFER_ID : 2;
        unsigned int                                 : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 6;
        unsigned int               DMI_CH4_BUFFER_ID : 2;
        unsigned int                                 : 6;
        unsigned int               DMI_CH3_BUFFER_ID : 2;
        unsigned int                                 : 6;
        unsigned int               DMI_CH2_BUFFER_ID : 2;
        unsigned int                                 : 6;
        unsigned int               DMI_CH1_BUFFER_ID : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_RTL_RELEASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      CHANGELIST : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      CHANGELIST : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_PATCH_RELEASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PATCH_REVISION : 16;
        unsigned int                 PATCH_SELECTION : 8;
        unsigned int                     CUSTOMER_ID : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                     CUSTOMER_ID : 8;
        unsigned int                 PATCH_SELECTION : 8;
        unsigned int                  PATCH_REVISION : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_AUXILIARY_CONFIG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        RESERVED : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                        RESERVED : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_PERIPHID0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     PARTNUMBER0 : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                     PARTNUMBER0 : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_PERIPHID1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     PARTNUMBER1 : 4;
        unsigned int                       DESIGNER0 : 4;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                       DESIGNER0 : 4;
        unsigned int                     PARTNUMBER1 : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_PERIPHID2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       DESIGNER1 : 4;
        unsigned int                        REVISION : 4;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        REVISION : 4;
        unsigned int                       DESIGNER1 : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_PERIPHID3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int             RBBM_HOST_INTERFACE : 2;
        unsigned int            GARB_SLAVE_INTERFACE : 2;
        unsigned int                    MH_INTERFACE : 2;
        unsigned int                                 : 1;
        unsigned int                    CONTINUATION : 1;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                    CONTINUATION : 1;
        unsigned int                                 : 1;
        unsigned int                    MH_INTERFACE : 2;
        unsigned int            GARB_SLAVE_INTERFACE : 2;
        unsigned int             RBBM_HOST_INTERFACE : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    READ_TIMEOUT : 8;
        unsigned int            REGCLK_DEASSERT_TIME : 9;
        unsigned int                                 : 15;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 15;
        unsigned int            REGCLK_DEASSERT_TIME : 9;
        unsigned int                    READ_TIMEOUT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_SKEW_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int              SKEW_TOP_THRESHOLD : 5;
        unsigned int                      SKEW_COUNT : 5;
        unsigned int                                 : 22;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 22;
        unsigned int                      SKEW_COUNT : 5;
        unsigned int              SKEW_TOP_THRESHOLD : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_SOFT_RESET {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   SOFT_RESET_CP : 1;
        unsigned int                                 : 1;
        unsigned int                   SOFT_RESET_PA : 1;
        unsigned int                   SOFT_RESET_MH : 1;
        unsigned int                   SOFT_RESET_BC : 1;
        unsigned int                   SOFT_RESET_SQ : 1;
        unsigned int                   SOFT_RESET_SX : 1;
        unsigned int                                 : 5;
        unsigned int                  SOFT_RESET_CIB : 1;
        unsigned int                                 : 2;
        unsigned int                   SOFT_RESET_SC : 1;
        unsigned int                  SOFT_RESET_VGT : 1;
        unsigned int                                 : 15;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 15;
        unsigned int                  SOFT_RESET_VGT : 1;
        unsigned int                   SOFT_RESET_SC : 1;
        unsigned int                                 : 2;
        unsigned int                  SOFT_RESET_CIB : 1;
        unsigned int                                 : 5;
        unsigned int                   SOFT_RESET_SX : 1;
        unsigned int                   SOFT_RESET_SQ : 1;
        unsigned int                   SOFT_RESET_BC : 1;
        unsigned int                   SOFT_RESET_MH : 1;
        unsigned int                   SOFT_RESET_PA : 1;
        unsigned int                                 : 1;
        unsigned int                   SOFT_RESET_CP : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_PM_OVERRIDE1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int         RBBM_AHBCLK_PM_OVERRIDE : 1;
        unsigned int         SC_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int             SC_SCLK_PM_OVERRIDE : 1;
        unsigned int         SP_TOP_SCLK_PM_OVERRIDE : 1;
        unsigned int          SP_V0_SCLK_PM_OVERRIDE : 1;
        unsigned int         SQ_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int   SQ_REG_FIFOS_SCLK_PM_OVERRIDE : 1;
        unsigned int   SQ_CONST_MEM_SCLK_PM_OVERRIDE : 1;
        unsigned int          SQ_SQ_SCLK_PM_OVERRIDE : 1;
        unsigned int             SX_SCLK_PM_OVERRIDE : 1;
        unsigned int         SX_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCM_TCO_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCM_TCM_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCM_TCD_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCM_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int        TPC_TPC_SCLK_PM_OVERRIDE : 1;
        unsigned int        TPC_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCF_TCA_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCF_TCB_SCLK_PM_OVERRIDE : 1;
        unsigned int   TCF_TCB_READ_SCLK_PM_OVERRIDE : 1;
        unsigned int          TP_TP_SCLK_PM_OVERRIDE : 1;
        unsigned int         TP_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int           CP_G_SCLK_PM_OVERRIDE : 1;
        unsigned int         CP_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int       CP_G_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int            SPI_SCLK_PM_OVERRIDE : 1;
        unsigned int         RB_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int             RB_SCLK_PM_OVERRIDE : 1;
        unsigned int          MH_MH_SCLK_PM_OVERRIDE : 1;
        unsigned int         MH_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int         MH_MMU_SCLK_PM_OVERRIDE : 1;
        unsigned int       MH_TCROQ_SCLK_PM_OVERRIDE : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int       MH_TCROQ_SCLK_PM_OVERRIDE : 1;
        unsigned int         MH_MMU_SCLK_PM_OVERRIDE : 1;
        unsigned int         MH_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int          MH_MH_SCLK_PM_OVERRIDE : 1;
        unsigned int             RB_SCLK_PM_OVERRIDE : 1;
        unsigned int         RB_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int            SPI_SCLK_PM_OVERRIDE : 1;
        unsigned int       CP_G_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int         CP_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int           CP_G_SCLK_PM_OVERRIDE : 1;
        unsigned int         TP_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int          TP_TP_SCLK_PM_OVERRIDE : 1;
        unsigned int   TCF_TCB_READ_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCF_TCB_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCF_TCA_SCLK_PM_OVERRIDE : 1;
        unsigned int        TPC_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int        TPC_TPC_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCM_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCM_TCD_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCM_TCM_SCLK_PM_OVERRIDE : 1;
        unsigned int        TCM_TCO_SCLK_PM_OVERRIDE : 1;
        unsigned int         SX_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int             SX_SCLK_PM_OVERRIDE : 1;
        unsigned int          SQ_SQ_SCLK_PM_OVERRIDE : 1;
        unsigned int   SQ_CONST_MEM_SCLK_PM_OVERRIDE : 1;
        unsigned int   SQ_REG_FIFOS_SCLK_PM_OVERRIDE : 1;
        unsigned int         SQ_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int          SP_V0_SCLK_PM_OVERRIDE : 1;
        unsigned int         SP_TOP_SCLK_PM_OVERRIDE : 1;
        unsigned int             SC_SCLK_PM_OVERRIDE : 1;
        unsigned int         SC_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int         RBBM_AHBCLK_PM_OVERRIDE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_PM_OVERRIDE2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int         PA_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int          PA_PA_SCLK_PM_OVERRIDE : 1;
        unsigned int          PA_AG_SCLK_PM_OVERRIDE : 1;
        unsigned int        VGT_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int      VGT_FIFOS_SCLK_PM_OVERRIDE : 1;
        unsigned int        VGT_VGT_SCLK_PM_OVERRIDE : 1;
        unsigned int     DEBUG_PERF_SCLK_PM_OVERRIDE : 1;
        unsigned int           PERM_SCLK_PM_OVERRIDE : 1;
        unsigned int         GC_GA_GMEM0_PM_OVERRIDE : 1;
        unsigned int         GC_GA_GMEM1_PM_OVERRIDE : 1;
        unsigned int         GC_GA_GMEM2_PM_OVERRIDE : 1;
        unsigned int         GC_GA_GMEM3_PM_OVERRIDE : 1;
        unsigned int                                 : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 20;
        unsigned int         GC_GA_GMEM3_PM_OVERRIDE : 1;
        unsigned int         GC_GA_GMEM2_PM_OVERRIDE : 1;
        unsigned int         GC_GA_GMEM1_PM_OVERRIDE : 1;
        unsigned int         GC_GA_GMEM0_PM_OVERRIDE : 1;
        unsigned int           PERM_SCLK_PM_OVERRIDE : 1;
        unsigned int     DEBUG_PERF_SCLK_PM_OVERRIDE : 1;
        unsigned int        VGT_VGT_SCLK_PM_OVERRIDE : 1;
        unsigned int      VGT_FIFOS_SCLK_PM_OVERRIDE : 1;
        unsigned int        VGT_REG_SCLK_PM_OVERRIDE : 1;
        unsigned int          PA_AG_SCLK_PM_OVERRIDE : 1;
        unsigned int          PA_PA_SCLK_PM_OVERRIDE : 1;
        unsigned int         PA_REG_SCLK_PM_OVERRIDE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union GC_SYS_IDLE {
    struct {
#if     defined(qLittleEndian)
        unsigned int               GC_SYS_IDLE_DELAY : 16;
        unsigned int            GC_SYS_WAIT_DMI_MASK : 6;
        unsigned int                                 : 2;
        unsigned int              GC_SYS_URGENT_RAMP : 1;
        unsigned int                 GC_SYS_WAIT_DMI : 1;
        unsigned int                                 : 3;
        unsigned int     GC_SYS_URGENT_RAMP_OVERRIDE : 1;
        unsigned int        GC_SYS_WAIT_DMI_OVERRIDE : 1;
        unsigned int            GC_SYS_IDLE_OVERRIDE : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int            GC_SYS_IDLE_OVERRIDE : 1;
        unsigned int        GC_SYS_WAIT_DMI_OVERRIDE : 1;
        unsigned int     GC_SYS_URGENT_RAMP_OVERRIDE : 1;
        unsigned int                                 : 3;
        unsigned int                 GC_SYS_WAIT_DMI : 1;
        unsigned int              GC_SYS_URGENT_RAMP : 1;
        unsigned int                                 : 2;
        unsigned int            GC_SYS_WAIT_DMI_MASK : 6;
        unsigned int               GC_SYS_IDLE_DELAY : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union NQWAIT_UNTIL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   WAIT_GUI_IDLE : 1;
        unsigned int                                 : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 31;
        unsigned int                   WAIT_GUI_IDLE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_DEBUG_OUT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   DEBUG_BUS_OUT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   DEBUG_BUS_OUT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_DEBUG_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  SUB_BLOCK_ADDR : 6;
        unsigned int                                 : 2;
        unsigned int                   SUB_BLOCK_SEL : 4;
        unsigned int                       SW_ENABLE : 1;
        unsigned int                                 : 3;
        unsigned int             GPIO_SUB_BLOCK_ADDR : 6;
        unsigned int                                 : 2;
        unsigned int              GPIO_SUB_BLOCK_SEL : 4;
        unsigned int              GPIO_BYTE_LANE_ENB : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int              GPIO_BYTE_LANE_ENB : 4;
        unsigned int              GPIO_SUB_BLOCK_SEL : 4;
        unsigned int                                 : 2;
        unsigned int             GPIO_SUB_BLOCK_ADDR : 6;
        unsigned int                                 : 3;
        unsigned int                       SW_ENABLE : 1;
        unsigned int                   SUB_BLOCK_SEL : 4;
        unsigned int                                 : 2;
        unsigned int                  SUB_BLOCK_ADDR : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 1;
        unsigned int                      IGNORE_RTR : 1;
        unsigned int              IGNORE_CP_SCHED_WU : 1;
        unsigned int           IGNORE_CP_SCHED_ISYNC : 1;
        unsigned int           IGNORE_CP_SCHED_NQ_HI : 1;
        unsigned int                                 : 3;
        unsigned int       HYSTERESIS_NRT_GUI_ACTIVE : 4;
        unsigned int                                 : 4;
        unsigned int               IGNORE_RTR_FOR_HI : 1;
        unsigned int    IGNORE_CP_RBBM_NRTRTR_FOR_HI : 1;
        unsigned int   IGNORE_VGT_RBBM_NRTRTR_FOR_HI : 1;
        unsigned int    IGNORE_SQ_RBBM_NRTRTR_FOR_HI : 1;
        unsigned int                  CP_RBBM_NRTRTR : 1;
        unsigned int                 VGT_RBBM_NRTRTR : 1;
        unsigned int                  SQ_RBBM_NRTRTR : 1;
        unsigned int      CLIENTS_FOR_NRT_RTR_FOR_HI : 1;
        unsigned int             CLIENTS_FOR_NRT_RTR : 1;
        unsigned int                                 : 6;
        unsigned int             IGNORE_SX_RBBM_BUSY : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int             IGNORE_SX_RBBM_BUSY : 1;
        unsigned int                                 : 6;
        unsigned int             CLIENTS_FOR_NRT_RTR : 1;
        unsigned int      CLIENTS_FOR_NRT_RTR_FOR_HI : 1;
        unsigned int                  SQ_RBBM_NRTRTR : 1;
        unsigned int                 VGT_RBBM_NRTRTR : 1;
        unsigned int                  CP_RBBM_NRTRTR : 1;
        unsigned int    IGNORE_SQ_RBBM_NRTRTR_FOR_HI : 1;
        unsigned int   IGNORE_VGT_RBBM_NRTRTR_FOR_HI : 1;
        unsigned int    IGNORE_CP_RBBM_NRTRTR_FOR_HI : 1;
        unsigned int               IGNORE_RTR_FOR_HI : 1;
        unsigned int                                 : 4;
        unsigned int       HYSTERESIS_NRT_GUI_ACTIVE : 4;
        unsigned int                                 : 3;
        unsigned int           IGNORE_CP_SCHED_NQ_HI : 1;
        unsigned int           IGNORE_CP_SCHED_ISYNC : 1;
        unsigned int              IGNORE_CP_SCHED_WU : 1;
        unsigned int                      IGNORE_RTR : 1;
        unsigned int                                 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_READ_ERROR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 2;
        unsigned int                    READ_ADDRESS : 15;
        unsigned int                                 : 13;
        unsigned int                  READ_REQUESTER : 1;
        unsigned int                      READ_ERROR : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                      READ_ERROR : 1;
        unsigned int                  READ_REQUESTER : 1;
        unsigned int                                 : 13;
        unsigned int                    READ_ADDRESS : 15;
        unsigned int                                 : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_WAIT_IDLE_CLOCKS {
    struct {
#if     defined(qLittleEndian)
        unsigned int            WAIT_IDLE_CLOCKS_NRT : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int            WAIT_IDLE_CLOCKS_NRT : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_INT_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  RDERR_INT_MASK : 1;
        unsigned int         DISPLAY_UPDATE_INT_MASK : 1;
        unsigned int                                 : 17;
        unsigned int               GUI_IDLE_INT_MASK : 1;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int               GUI_IDLE_INT_MASK : 1;
        unsigned int                                 : 17;
        unsigned int         DISPLAY_UPDATE_INT_MASK : 1;
        unsigned int                  RDERR_INT_MASK : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_INT_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  RDERR_INT_STAT : 1;
        unsigned int         DISPLAY_UPDATE_INT_STAT : 1;
        unsigned int                                 : 17;
        unsigned int               GUI_IDLE_INT_STAT : 1;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int               GUI_IDLE_INT_STAT : 1;
        unsigned int                                 : 17;
        unsigned int         DISPLAY_UPDATE_INT_STAT : 1;
        unsigned int                  RDERR_INT_STAT : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_INT_ACK {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   RDERR_INT_ACK : 1;
        unsigned int          DISPLAY_UPDATE_INT_ACK : 1;
        unsigned int                                 : 17;
        unsigned int                GUI_IDLE_INT_ACK : 1;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                GUI_IDLE_INT_ACK : 1;
        unsigned int                                 : 17;
        unsigned int          DISPLAY_UPDATE_INT_ACK : 1;
        unsigned int                   RDERR_INT_ACK : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union MASTER_INT_SIGNAL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 5;
        unsigned int                     MH_INT_STAT : 1;
        unsigned int                                 : 20;
        unsigned int                     SQ_INT_STAT : 1;
        unsigned int                                 : 3;
        unsigned int                     CP_INT_STAT : 1;
        unsigned int                   RBBM_INT_STAT : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                   RBBM_INT_STAT : 1;
        unsigned int                     CP_INT_STAT : 1;
        unsigned int                                 : 3;
        unsigned int                     SQ_INT_STAT : 1;
        unsigned int                                 : 20;
        unsigned int                     MH_INT_STAT : 1;
        unsigned int                                 : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_PERFCOUNTER1_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PERF_COUNT1_SEL : 6;
        unsigned int                                 : 26;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 26;
        unsigned int                 PERF_COUNT1_SEL : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_PERFCOUNTER1_LO {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERF_COUNT1_LO : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                  PERF_COUNT1_LO : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RBBM_PERFCOUNTER1_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  PERF_COUNT1_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                  PERF_COUNT1_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_RB_BASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 5;
        unsigned int                         RB_BASE : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                         RB_BASE : 27;
        unsigned int                                 : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_RB_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        RB_BUFSZ : 6;
        unsigned int                                 : 2;
        unsigned int                        RB_BLKSZ : 6;
        unsigned int                                 : 2;
        unsigned int                        BUF_SWAP : 2;
        unsigned int                                 : 2;
        unsigned int                      RB_POLL_EN : 1;
        unsigned int                                 : 6;
        unsigned int                    RB_NO_UPDATE : 1;
        unsigned int                                 : 3;
        unsigned int                  RB_RPTR_WR_ENA : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                  RB_RPTR_WR_ENA : 1;
        unsigned int                                 : 3;
        unsigned int                    RB_NO_UPDATE : 1;
        unsigned int                                 : 6;
        unsigned int                      RB_POLL_EN : 1;
        unsigned int                                 : 2;
        unsigned int                        BUF_SWAP : 2;
        unsigned int                                 : 2;
        unsigned int                        RB_BLKSZ : 6;
        unsigned int                                 : 2;
        unsigned int                        RB_BUFSZ : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_RB_RPTR_ADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    RB_RPTR_SWAP : 2;
        unsigned int                    RB_RPTR_ADDR : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                    RB_RPTR_ADDR : 30;
        unsigned int                    RB_RPTR_SWAP : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_RB_RPTR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         RB_RPTR : 20;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                         RB_RPTR : 20;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_RB_RPTR_WR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      RB_RPTR_WR : 20;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                      RB_RPTR_WR : 20;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_RB_WPTR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         RB_WPTR : 20;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                         RB_WPTR : 20;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_RB_WPTR_DELAY {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 PRE_WRITE_TIMER : 28;
        unsigned int                 PRE_WRITE_LIMIT : 4;
#else       /* !defined(qLittleEndian) */
        unsigned int                 PRE_WRITE_LIMIT : 4;
        unsigned int                 PRE_WRITE_TIMER : 28;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_RB_WPTR_BASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    RB_WPTR_SWAP : 2;
        unsigned int                    RB_WPTR_BASE : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                    RB_WPTR_BASE : 30;
        unsigned int                    RB_WPTR_SWAP : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_IB1_BASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 2;
        unsigned int                        IB1_BASE : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                        IB1_BASE : 30;
        unsigned int                                 : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_IB1_BUFSZ {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       IB1_BUFSZ : 20;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                       IB1_BUFSZ : 20;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_IB2_BASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 2;
        unsigned int                        IB2_BASE : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                        IB2_BASE : 30;
        unsigned int                                 : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_IB2_BUFSZ {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       IB2_BUFSZ : 20;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                       IB2_BUFSZ : 20;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ST_BASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 2;
        unsigned int                         ST_BASE : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                         ST_BASE : 30;
        unsigned int                                 : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ST_BUFSZ {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        ST_BUFSZ : 20;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                        ST_BUFSZ : 20;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_QUEUE_THRESHOLDS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   CSQ_IB1_START : 4;
        unsigned int                                 : 4;
        unsigned int                   CSQ_IB2_START : 4;
        unsigned int                                 : 4;
        unsigned int                    CSQ_ST_START : 4;
        unsigned int                                 : 12;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 12;
        unsigned int                    CSQ_ST_START : 4;
        unsigned int                                 : 4;
        unsigned int                   CSQ_IB2_START : 4;
        unsigned int                                 : 4;
        unsigned int                   CSQ_IB1_START : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_MEQ_THRESHOLDS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 16;
        unsigned int                         MEQ_END : 5;
        unsigned int                                 : 3;
        unsigned int                         ROQ_END : 5;
        unsigned int                                 : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 3;
        unsigned int                         ROQ_END : 5;
        unsigned int                                 : 3;
        unsigned int                         MEQ_END : 5;
        unsigned int                                 : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_CSQ_AVAIL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    CSQ_CNT_RING : 7;
        unsigned int                                 : 1;
        unsigned int                     CSQ_CNT_IB1 : 7;
        unsigned int                                 : 1;
        unsigned int                     CSQ_CNT_IB2 : 7;
        unsigned int                                 : 9;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 9;
        unsigned int                     CSQ_CNT_IB2 : 7;
        unsigned int                                 : 1;
        unsigned int                     CSQ_CNT_IB1 : 7;
        unsigned int                                 : 1;
        unsigned int                    CSQ_CNT_RING : 7;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_STQ_AVAIL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      STQ_CNT_ST : 7;
        unsigned int                                 : 25;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 25;
        unsigned int                      STQ_CNT_ST : 7;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_MEQ_AVAIL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         MEQ_CNT : 5;
        unsigned int                                 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 27;
        unsigned int                         MEQ_CNT : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_CSQ_RB_STAT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                CSQ_RPTR_PRIMARY : 7;
        unsigned int                                 : 9;
        unsigned int                CSQ_WPTR_PRIMARY : 7;
        unsigned int                                 : 9;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 9;
        unsigned int                CSQ_WPTR_PRIMARY : 7;
        unsigned int                                 : 9;
        unsigned int                CSQ_RPTR_PRIMARY : 7;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_CSQ_IB1_STAT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              CSQ_RPTR_INDIRECT1 : 7;
        unsigned int                                 : 9;
        unsigned int              CSQ_WPTR_INDIRECT1 : 7;
        unsigned int                                 : 9;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 9;
        unsigned int              CSQ_WPTR_INDIRECT1 : 7;
        unsigned int                                 : 9;
        unsigned int              CSQ_RPTR_INDIRECT1 : 7;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_CSQ_IB2_STAT {
    struct {
#if     defined(qLittleEndian)
        unsigned int              CSQ_RPTR_INDIRECT2 : 7;
        unsigned int                                 : 9;
        unsigned int              CSQ_WPTR_INDIRECT2 : 7;
        unsigned int                                 : 9;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 9;
        unsigned int              CSQ_WPTR_INDIRECT2 : 7;
        unsigned int                                 : 9;
        unsigned int              CSQ_RPTR_INDIRECT2 : 7;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_NON_PREFETCH_CNTRS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     IB1_COUNTER : 3;
        unsigned int                                 : 5;
        unsigned int                     IB2_COUNTER : 3;
        unsigned int                                 : 21;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 21;
        unsigned int                     IB2_COUNTER : 3;
        unsigned int                                 : 5;
        unsigned int                     IB1_COUNTER : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_STQ_ST_STAT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     STQ_RPTR_ST : 7;
        unsigned int                                 : 9;
        unsigned int                     STQ_WPTR_ST : 7;
        unsigned int                                 : 9;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 9;
        unsigned int                     STQ_WPTR_ST : 7;
        unsigned int                                 : 9;
        unsigned int                     STQ_RPTR_ST : 7;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_MEQ_STAT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        MEQ_RPTR : 10;
        unsigned int                                 : 6;
        unsigned int                        MEQ_WPTR : 10;
        unsigned int                                 : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 6;
        unsigned int                        MEQ_WPTR : 10;
        unsigned int                                 : 6;
        unsigned int                        MEQ_RPTR : 10;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_MIU_TAG_STAT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      TAG_0_STAT : 1;
        unsigned int                      TAG_1_STAT : 1;
        unsigned int                      TAG_2_STAT : 1;
        unsigned int                      TAG_3_STAT : 1;
        unsigned int                      TAG_4_STAT : 1;
        unsigned int                      TAG_5_STAT : 1;
        unsigned int                      TAG_6_STAT : 1;
        unsigned int                      TAG_7_STAT : 1;
        unsigned int                      TAG_8_STAT : 1;
        unsigned int                      TAG_9_STAT : 1;
        unsigned int                     TAG_10_STAT : 1;
        unsigned int                     TAG_11_STAT : 1;
        unsigned int                     TAG_12_STAT : 1;
        unsigned int                     TAG_13_STAT : 1;
        unsigned int                     TAG_14_STAT : 1;
        unsigned int                     TAG_15_STAT : 1;
        unsigned int                     TAG_16_STAT : 1;
        unsigned int                     TAG_17_STAT : 1;
        unsigned int                                 : 13;
        unsigned int              INVALID_RETURN_TAG : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int              INVALID_RETURN_TAG : 1;
        unsigned int                                 : 13;
        unsigned int                     TAG_17_STAT : 1;
        unsigned int                     TAG_16_STAT : 1;
        unsigned int                     TAG_15_STAT : 1;
        unsigned int                     TAG_14_STAT : 1;
        unsigned int                     TAG_13_STAT : 1;
        unsigned int                     TAG_12_STAT : 1;
        unsigned int                     TAG_11_STAT : 1;
        unsigned int                     TAG_10_STAT : 1;
        unsigned int                      TAG_9_STAT : 1;
        unsigned int                      TAG_8_STAT : 1;
        unsigned int                      TAG_7_STAT : 1;
        unsigned int                      TAG_6_STAT : 1;
        unsigned int                      TAG_5_STAT : 1;
        unsigned int                      TAG_4_STAT : 1;
        unsigned int                      TAG_3_STAT : 1;
        unsigned int                      TAG_2_STAT : 1;
        unsigned int                      TAG_1_STAT : 1;
        unsigned int                      TAG_0_STAT : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_CMD_INDEX {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       CMD_INDEX : 7;
        unsigned int                                 : 9;
        unsigned int                   CMD_QUEUE_SEL : 2;
        unsigned int                                 : 14;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 14;
        unsigned int                   CMD_QUEUE_SEL : 2;
        unsigned int                                 : 9;
        unsigned int                       CMD_INDEX : 7;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_CMD_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        CMD_DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                        CMD_DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      ME_STATMUX : 16;
        unsigned int                                 : 9;
        unsigned int          VTX_DEALLOC_FIFO_EMPTY : 1;
        unsigned int          PIX_DEALLOC_FIFO_EMPTY : 1;
        unsigned int                                 : 1;
        unsigned int                         ME_HALT : 1;
        unsigned int                         ME_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                   PROG_CNT_SIZE : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                   PROG_CNT_SIZE : 1;
        unsigned int                                 : 1;
        unsigned int                         ME_BUSY : 1;
        unsigned int                         ME_HALT : 1;
        unsigned int                                 : 1;
        unsigned int          PIX_DEALLOC_FIFO_EMPTY : 1;
        unsigned int          VTX_DEALLOC_FIFO_EMPTY : 1;
        unsigned int                                 : 9;
        unsigned int                      ME_STATMUX : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   ME_DEBUG_DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   ME_DEBUG_DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_RAM_WADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    ME_RAM_WADDR : 10;
        unsigned int                                 : 22;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 22;
        unsigned int                    ME_RAM_WADDR : 10;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_RAM_RADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    ME_RAM_RADDR : 10;
        unsigned int                                 : 22;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 22;
        unsigned int                    ME_RAM_RADDR : 10;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_RAM_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     ME_RAM_DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                     ME_RAM_DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_RDADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       ME_RDADDR : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                       ME_RDADDR : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_DEBUG {
    struct {
#if     defined(qLittleEndian)
        unsigned int         CP_DEBUG_UNUSED_22_to_0 : 23;
        unsigned int               PREDICATE_DISABLE : 1;
        unsigned int             PROG_END_PTR_ENABLE : 1;
        unsigned int         MIU_128BIT_WRITE_ENABLE : 1;
        unsigned int              PREFETCH_PASS_NOPS : 1;
        unsigned int             DYNAMIC_CLK_DISABLE : 1;
        unsigned int          PREFETCH_MATCH_DISABLE : 1;
        unsigned int                                 : 1;
        unsigned int          SIMPLE_ME_FLOW_CONTROL : 1;
        unsigned int          MIU_WRITE_PACK_DISABLE : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int          MIU_WRITE_PACK_DISABLE : 1;
        unsigned int          SIMPLE_ME_FLOW_CONTROL : 1;
        unsigned int                                 : 1;
        unsigned int          PREFETCH_MATCH_DISABLE : 1;
        unsigned int             DYNAMIC_CLK_DISABLE : 1;
        unsigned int              PREFETCH_PASS_NOPS : 1;
        unsigned int         MIU_128BIT_WRITE_ENABLE : 1;
        unsigned int             PROG_END_PTR_ENABLE : 1;
        unsigned int               PREDICATE_DISABLE : 1;
        unsigned int         CP_DEBUG_UNUSED_22_to_0 : 23;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SCRATCH_REG0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    SCRATCH_REG0 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    SCRATCH_REG0 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SCRATCH_REG1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    SCRATCH_REG1 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    SCRATCH_REG1 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SCRATCH_REG2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    SCRATCH_REG2 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    SCRATCH_REG2 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SCRATCH_REG3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    SCRATCH_REG3 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    SCRATCH_REG3 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SCRATCH_REG4 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    SCRATCH_REG4 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    SCRATCH_REG4 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SCRATCH_REG5 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    SCRATCH_REG5 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    SCRATCH_REG5 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SCRATCH_REG6 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    SCRATCH_REG6 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    SCRATCH_REG6 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SCRATCH_REG7 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    SCRATCH_REG7 : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    SCRATCH_REG7 : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SCRATCH_UMSK {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    SCRATCH_UMSK : 8;
        unsigned int                                 : 8;
        unsigned int                    SCRATCH_SWAP : 2;
        unsigned int                                 : 14;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 14;
        unsigned int                    SCRATCH_SWAP : 2;
        unsigned int                                 : 8;
        unsigned int                    SCRATCH_UMSK : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union SCRATCH_ADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 5;
        unsigned int                    SCRATCH_ADDR : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                    SCRATCH_ADDR : 27;
        unsigned int                                 : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_VS_EVENT_SRC {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     VS_DONE_SWM : 1;
        unsigned int                    VS_DONE_CNTR : 1;
        unsigned int                                 : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 30;
        unsigned int                    VS_DONE_CNTR : 1;
        unsigned int                     VS_DONE_SWM : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_VS_EVENT_ADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    VS_DONE_SWAP : 2;
        unsigned int                    VS_DONE_ADDR : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                    VS_DONE_ADDR : 30;
        unsigned int                    VS_DONE_SWAP : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_VS_EVENT_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    VS_DONE_DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    VS_DONE_DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_VS_EVENT_ADDR_SWM {
    struct {
#if     defined(qLittleEndian)
        unsigned int                VS_DONE_SWAP_SWM : 2;
        unsigned int                VS_DONE_ADDR_SWM : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                VS_DONE_ADDR_SWM : 30;
        unsigned int                VS_DONE_SWAP_SWM : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_VS_EVENT_DATA_SWM {
    struct {
#if     defined(qLittleEndian)
        unsigned int                VS_DONE_DATA_SWM : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                VS_DONE_DATA_SWM : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_PS_EVENT_SRC {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     PS_DONE_SWM : 1;
        unsigned int                    PS_DONE_CNTR : 1;
        unsigned int                                 : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 30;
        unsigned int                    PS_DONE_CNTR : 1;
        unsigned int                     PS_DONE_SWM : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_PS_EVENT_ADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    PS_DONE_SWAP : 2;
        unsigned int                    PS_DONE_ADDR : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                    PS_DONE_ADDR : 30;
        unsigned int                    PS_DONE_SWAP : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_PS_EVENT_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    PS_DONE_DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    PS_DONE_DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_PS_EVENT_ADDR_SWM {
    struct {
#if     defined(qLittleEndian)
        unsigned int                PS_DONE_SWAP_SWM : 2;
        unsigned int                PS_DONE_ADDR_SWM : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                PS_DONE_ADDR_SWM : 30;
        unsigned int                PS_DONE_SWAP_SWM : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_PS_EVENT_DATA_SWM {
    struct {
#if     defined(qLittleEndian)
        unsigned int                PS_DONE_DATA_SWM : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                PS_DONE_DATA_SWM : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_CF_EVENT_SRC {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     CF_DONE_SRC : 1;
        unsigned int                                 : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 31;
        unsigned int                     CF_DONE_SRC : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_CF_EVENT_ADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    CF_DONE_SWAP : 2;
        unsigned int                    CF_DONE_ADDR : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                    CF_DONE_ADDR : 30;
        unsigned int                    CF_DONE_SWAP : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_CF_EVENT_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    CF_DONE_DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    CF_DONE_DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_NRT_ADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  NRT_WRITE_SWAP : 2;
        unsigned int                  NRT_WRITE_ADDR : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                  NRT_WRITE_ADDR : 30;
        unsigned int                  NRT_WRITE_SWAP : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_NRT_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  NRT_WRITE_DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                  NRT_WRITE_DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_VS_FETCH_DONE_SRC {
    struct {
#if     defined(qLittleEndian)
        unsigned int              VS_FETCH_DONE_CNTR : 1;
        unsigned int                                 : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 31;
        unsigned int              VS_FETCH_DONE_CNTR : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_VS_FETCH_DONE_ADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int              VS_FETCH_DONE_SWAP : 2;
        unsigned int              VS_FETCH_DONE_ADDR : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int              VS_FETCH_DONE_ADDR : 30;
        unsigned int              VS_FETCH_DONE_SWAP : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_ME_VS_FETCH_DONE_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int              VS_FETCH_DONE_DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int              VS_FETCH_DONE_DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_INT_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 19;
        unsigned int                     SW_INT_MASK : 1;
        unsigned int                                 : 3;
        unsigned int            T0_PACKET_IN_IB_MASK : 1;
        unsigned int               OPCODE_ERROR_MASK : 1;
        unsigned int       PROTECTED_MODE_ERROR_MASK : 1;
        unsigned int         RESERVED_BIT_ERROR_MASK : 1;
        unsigned int                   IB_ERROR_MASK : 1;
        unsigned int                                 : 1;
        unsigned int                    IB2_INT_MASK : 1;
        unsigned int                    IB1_INT_MASK : 1;
        unsigned int                     RB_INT_MASK : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     RB_INT_MASK : 1;
        unsigned int                    IB1_INT_MASK : 1;
        unsigned int                    IB2_INT_MASK : 1;
        unsigned int                                 : 1;
        unsigned int                   IB_ERROR_MASK : 1;
        unsigned int         RESERVED_BIT_ERROR_MASK : 1;
        unsigned int       PROTECTED_MODE_ERROR_MASK : 1;
        unsigned int               OPCODE_ERROR_MASK : 1;
        unsigned int            T0_PACKET_IN_IB_MASK : 1;
        unsigned int                                 : 3;
        unsigned int                     SW_INT_MASK : 1;
        unsigned int                                 : 19;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_INT_STATUS {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 19;
        unsigned int                     SW_INT_STAT : 1;
        unsigned int                                 : 3;
        unsigned int            T0_PACKET_IN_IB_STAT : 1;
        unsigned int               OPCODE_ERROR_STAT : 1;
        unsigned int       PROTECTED_MODE_ERROR_STAT : 1;
        unsigned int         RESERVED_BIT_ERROR_STAT : 1;
        unsigned int                   IB_ERROR_STAT : 1;
        unsigned int                                 : 1;
        unsigned int                    IB2_INT_STAT : 1;
        unsigned int                    IB1_INT_STAT : 1;
        unsigned int                     RB_INT_STAT : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     RB_INT_STAT : 1;
        unsigned int                    IB1_INT_STAT : 1;
        unsigned int                    IB2_INT_STAT : 1;
        unsigned int                                 : 1;
        unsigned int                   IB_ERROR_STAT : 1;
        unsigned int         RESERVED_BIT_ERROR_STAT : 1;
        unsigned int       PROTECTED_MODE_ERROR_STAT : 1;
        unsigned int               OPCODE_ERROR_STAT : 1;
        unsigned int            T0_PACKET_IN_IB_STAT : 1;
        unsigned int                                 : 3;
        unsigned int                     SW_INT_STAT : 1;
        unsigned int                                 : 19;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_INT_ACK {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 19;
        unsigned int                      SW_INT_ACK : 1;
        unsigned int                                 : 3;
        unsigned int             T0_PACKET_IN_IB_ACK : 1;
        unsigned int                OPCODE_ERROR_ACK : 1;
        unsigned int        PROTECTED_MODE_ERROR_ACK : 1;
        unsigned int          RESERVED_BIT_ERROR_ACK : 1;
        unsigned int                    IB_ERROR_ACK : 1;
        unsigned int                                 : 1;
        unsigned int                     IB2_INT_ACK : 1;
        unsigned int                     IB1_INT_ACK : 1;
        unsigned int                      RB_INT_ACK : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                      RB_INT_ACK : 1;
        unsigned int                     IB1_INT_ACK : 1;
        unsigned int                     IB2_INT_ACK : 1;
        unsigned int                                 : 1;
        unsigned int                    IB_ERROR_ACK : 1;
        unsigned int          RESERVED_BIT_ERROR_ACK : 1;
        unsigned int        PROTECTED_MODE_ERROR_ACK : 1;
        unsigned int                OPCODE_ERROR_ACK : 1;
        unsigned int             T0_PACKET_IN_IB_ACK : 1;
        unsigned int                                 : 3;
        unsigned int                      SW_INT_ACK : 1;
        unsigned int                                 : 19;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_PFP_UCODE_ADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      UCODE_ADDR : 9;
        unsigned int                                 : 23;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 23;
        unsigned int                      UCODE_ADDR : 9;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_PFP_UCODE_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      UCODE_DATA : 24;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                      UCODE_DATA : 24;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_PERFMON_CNTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   PERFMON_STATE : 4;
        unsigned int                                 : 4;
        unsigned int             PERFMON_ENABLE_MODE : 2;
        unsigned int                                 : 22;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 22;
        unsigned int             PERFMON_ENABLE_MODE : 2;
        unsigned int                                 : 4;
        unsigned int                   PERFMON_STATE : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_PERFCOUNTER_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   PERFCOUNT_SEL : 6;
        unsigned int                                 : 26;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 26;
        unsigned int                   PERFCOUNT_SEL : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_PERFCOUNTER_LO {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    PERFCOUNT_LO : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    PERFCOUNT_LO : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_PERFCOUNTER_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    PERFCOUNT_HI : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                    PERFCOUNT_HI : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_BIN_MASK_LO {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     BIN_MASK_LO : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                     BIN_MASK_LO : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_BIN_MASK_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     BIN_MASK_HI : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                     BIN_MASK_HI : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_BIN_SELECT_LO {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   BIN_SELECT_LO : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   BIN_SELECT_LO : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_BIN_SELECT_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   BIN_SELECT_HI : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   BIN_SELECT_HI : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_NV_FLAGS_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       DISCARD_0 : 1;
        unsigned int                      END_RCVD_0 : 1;
        unsigned int                       DISCARD_1 : 1;
        unsigned int                      END_RCVD_1 : 1;
        unsigned int                       DISCARD_2 : 1;
        unsigned int                      END_RCVD_2 : 1;
        unsigned int                       DISCARD_3 : 1;
        unsigned int                      END_RCVD_3 : 1;
        unsigned int                       DISCARD_4 : 1;
        unsigned int                      END_RCVD_4 : 1;
        unsigned int                       DISCARD_5 : 1;
        unsigned int                      END_RCVD_5 : 1;
        unsigned int                       DISCARD_6 : 1;
        unsigned int                      END_RCVD_6 : 1;
        unsigned int                       DISCARD_7 : 1;
        unsigned int                      END_RCVD_7 : 1;
        unsigned int                       DISCARD_8 : 1;
        unsigned int                      END_RCVD_8 : 1;
        unsigned int                       DISCARD_9 : 1;
        unsigned int                      END_RCVD_9 : 1;
        unsigned int                      DISCARD_10 : 1;
        unsigned int                     END_RCVD_10 : 1;
        unsigned int                      DISCARD_11 : 1;
        unsigned int                     END_RCVD_11 : 1;
        unsigned int                      DISCARD_12 : 1;
        unsigned int                     END_RCVD_12 : 1;
        unsigned int                      DISCARD_13 : 1;
        unsigned int                     END_RCVD_13 : 1;
        unsigned int                      DISCARD_14 : 1;
        unsigned int                     END_RCVD_14 : 1;
        unsigned int                      DISCARD_15 : 1;
        unsigned int                     END_RCVD_15 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     END_RCVD_15 : 1;
        unsigned int                      DISCARD_15 : 1;
        unsigned int                     END_RCVD_14 : 1;
        unsigned int                      DISCARD_14 : 1;
        unsigned int                     END_RCVD_13 : 1;
        unsigned int                      DISCARD_13 : 1;
        unsigned int                     END_RCVD_12 : 1;
        unsigned int                      DISCARD_12 : 1;
        unsigned int                     END_RCVD_11 : 1;
        unsigned int                      DISCARD_11 : 1;
        unsigned int                     END_RCVD_10 : 1;
        unsigned int                      DISCARD_10 : 1;
        unsigned int                      END_RCVD_9 : 1;
        unsigned int                       DISCARD_9 : 1;
        unsigned int                      END_RCVD_8 : 1;
        unsigned int                       DISCARD_8 : 1;
        unsigned int                      END_RCVD_7 : 1;
        unsigned int                       DISCARD_7 : 1;
        unsigned int                      END_RCVD_6 : 1;
        unsigned int                       DISCARD_6 : 1;
        unsigned int                      END_RCVD_5 : 1;
        unsigned int                       DISCARD_5 : 1;
        unsigned int                      END_RCVD_4 : 1;
        unsigned int                       DISCARD_4 : 1;
        unsigned int                      END_RCVD_3 : 1;
        unsigned int                       DISCARD_3 : 1;
        unsigned int                      END_RCVD_2 : 1;
        unsigned int                       DISCARD_2 : 1;
        unsigned int                      END_RCVD_1 : 1;
        unsigned int                       DISCARD_1 : 1;
        unsigned int                      END_RCVD_0 : 1;
        unsigned int                       DISCARD_0 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_NV_FLAGS_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      DISCARD_16 : 1;
        unsigned int                     END_RCVD_16 : 1;
        unsigned int                      DISCARD_17 : 1;
        unsigned int                     END_RCVD_17 : 1;
        unsigned int                      DISCARD_18 : 1;
        unsigned int                     END_RCVD_18 : 1;
        unsigned int                      DISCARD_19 : 1;
        unsigned int                     END_RCVD_19 : 1;
        unsigned int                      DISCARD_20 : 1;
        unsigned int                     END_RCVD_20 : 1;
        unsigned int                      DISCARD_21 : 1;
        unsigned int                     END_RCVD_21 : 1;
        unsigned int                      DISCARD_22 : 1;
        unsigned int                     END_RCVD_22 : 1;
        unsigned int                      DISCARD_23 : 1;
        unsigned int                     END_RCVD_23 : 1;
        unsigned int                      DISCARD_24 : 1;
        unsigned int                     END_RCVD_24 : 1;
        unsigned int                      DISCARD_25 : 1;
        unsigned int                     END_RCVD_25 : 1;
        unsigned int                      DISCARD_26 : 1;
        unsigned int                     END_RCVD_26 : 1;
        unsigned int                      DISCARD_27 : 1;
        unsigned int                     END_RCVD_27 : 1;
        unsigned int                      DISCARD_28 : 1;
        unsigned int                     END_RCVD_28 : 1;
        unsigned int                      DISCARD_29 : 1;
        unsigned int                     END_RCVD_29 : 1;
        unsigned int                      DISCARD_30 : 1;
        unsigned int                     END_RCVD_30 : 1;
        unsigned int                      DISCARD_31 : 1;
        unsigned int                     END_RCVD_31 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     END_RCVD_31 : 1;
        unsigned int                      DISCARD_31 : 1;
        unsigned int                     END_RCVD_30 : 1;
        unsigned int                      DISCARD_30 : 1;
        unsigned int                     END_RCVD_29 : 1;
        unsigned int                      DISCARD_29 : 1;
        unsigned int                     END_RCVD_28 : 1;
        unsigned int                      DISCARD_28 : 1;
        unsigned int                     END_RCVD_27 : 1;
        unsigned int                      DISCARD_27 : 1;
        unsigned int                     END_RCVD_26 : 1;
        unsigned int                      DISCARD_26 : 1;
        unsigned int                     END_RCVD_25 : 1;
        unsigned int                      DISCARD_25 : 1;
        unsigned int                     END_RCVD_24 : 1;
        unsigned int                      DISCARD_24 : 1;
        unsigned int                     END_RCVD_23 : 1;
        unsigned int                      DISCARD_23 : 1;
        unsigned int                     END_RCVD_22 : 1;
        unsigned int                      DISCARD_22 : 1;
        unsigned int                     END_RCVD_21 : 1;
        unsigned int                      DISCARD_21 : 1;
        unsigned int                     END_RCVD_20 : 1;
        unsigned int                      DISCARD_20 : 1;
        unsigned int                     END_RCVD_19 : 1;
        unsigned int                      DISCARD_19 : 1;
        unsigned int                     END_RCVD_18 : 1;
        unsigned int                      DISCARD_18 : 1;
        unsigned int                     END_RCVD_17 : 1;
        unsigned int                      DISCARD_17 : 1;
        unsigned int                     END_RCVD_16 : 1;
        unsigned int                      DISCARD_16 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_NV_FLAGS_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      DISCARD_32 : 1;
        unsigned int                     END_RCVD_32 : 1;
        unsigned int                      DISCARD_33 : 1;
        unsigned int                     END_RCVD_33 : 1;
        unsigned int                      DISCARD_34 : 1;
        unsigned int                     END_RCVD_34 : 1;
        unsigned int                      DISCARD_35 : 1;
        unsigned int                     END_RCVD_35 : 1;
        unsigned int                      DISCARD_36 : 1;
        unsigned int                     END_RCVD_36 : 1;
        unsigned int                      DISCARD_37 : 1;
        unsigned int                     END_RCVD_37 : 1;
        unsigned int                      DISCARD_38 : 1;
        unsigned int                     END_RCVD_38 : 1;
        unsigned int                      DISCARD_39 : 1;
        unsigned int                     END_RCVD_39 : 1;
        unsigned int                      DISCARD_40 : 1;
        unsigned int                     END_RCVD_40 : 1;
        unsigned int                      DISCARD_41 : 1;
        unsigned int                     END_RCVD_41 : 1;
        unsigned int                      DISCARD_42 : 1;
        unsigned int                     END_RCVD_42 : 1;
        unsigned int                      DISCARD_43 : 1;
        unsigned int                     END_RCVD_43 : 1;
        unsigned int                      DISCARD_44 : 1;
        unsigned int                     END_RCVD_44 : 1;
        unsigned int                      DISCARD_45 : 1;
        unsigned int                     END_RCVD_45 : 1;
        unsigned int                      DISCARD_46 : 1;
        unsigned int                     END_RCVD_46 : 1;
        unsigned int                      DISCARD_47 : 1;
        unsigned int                     END_RCVD_47 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     END_RCVD_47 : 1;
        unsigned int                      DISCARD_47 : 1;
        unsigned int                     END_RCVD_46 : 1;
        unsigned int                      DISCARD_46 : 1;
        unsigned int                     END_RCVD_45 : 1;
        unsigned int                      DISCARD_45 : 1;
        unsigned int                     END_RCVD_44 : 1;
        unsigned int                      DISCARD_44 : 1;
        unsigned int                     END_RCVD_43 : 1;
        unsigned int                      DISCARD_43 : 1;
        unsigned int                     END_RCVD_42 : 1;
        unsigned int                      DISCARD_42 : 1;
        unsigned int                     END_RCVD_41 : 1;
        unsigned int                      DISCARD_41 : 1;
        unsigned int                     END_RCVD_40 : 1;
        unsigned int                      DISCARD_40 : 1;
        unsigned int                     END_RCVD_39 : 1;
        unsigned int                      DISCARD_39 : 1;
        unsigned int                     END_RCVD_38 : 1;
        unsigned int                      DISCARD_38 : 1;
        unsigned int                     END_RCVD_37 : 1;
        unsigned int                      DISCARD_37 : 1;
        unsigned int                     END_RCVD_36 : 1;
        unsigned int                      DISCARD_36 : 1;
        unsigned int                     END_RCVD_35 : 1;
        unsigned int                      DISCARD_35 : 1;
        unsigned int                     END_RCVD_34 : 1;
        unsigned int                      DISCARD_34 : 1;
        unsigned int                     END_RCVD_33 : 1;
        unsigned int                      DISCARD_33 : 1;
        unsigned int                     END_RCVD_32 : 1;
        unsigned int                      DISCARD_32 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_NV_FLAGS_3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      DISCARD_48 : 1;
        unsigned int                     END_RCVD_48 : 1;
        unsigned int                      DISCARD_49 : 1;
        unsigned int                     END_RCVD_49 : 1;
        unsigned int                      DISCARD_50 : 1;
        unsigned int                     END_RCVD_50 : 1;
        unsigned int                      DISCARD_51 : 1;
        unsigned int                     END_RCVD_51 : 1;
        unsigned int                      DISCARD_52 : 1;
        unsigned int                     END_RCVD_52 : 1;
        unsigned int                      DISCARD_53 : 1;
        unsigned int                     END_RCVD_53 : 1;
        unsigned int                      DISCARD_54 : 1;
        unsigned int                     END_RCVD_54 : 1;
        unsigned int                      DISCARD_55 : 1;
        unsigned int                     END_RCVD_55 : 1;
        unsigned int                      DISCARD_56 : 1;
        unsigned int                     END_RCVD_56 : 1;
        unsigned int                      DISCARD_57 : 1;
        unsigned int                     END_RCVD_57 : 1;
        unsigned int                      DISCARD_58 : 1;
        unsigned int                     END_RCVD_58 : 1;
        unsigned int                      DISCARD_59 : 1;
        unsigned int                     END_RCVD_59 : 1;
        unsigned int                      DISCARD_60 : 1;
        unsigned int                     END_RCVD_60 : 1;
        unsigned int                      DISCARD_61 : 1;
        unsigned int                     END_RCVD_61 : 1;
        unsigned int                      DISCARD_62 : 1;
        unsigned int                     END_RCVD_62 : 1;
        unsigned int                      DISCARD_63 : 1;
        unsigned int                     END_RCVD_63 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                     END_RCVD_63 : 1;
        unsigned int                      DISCARD_63 : 1;
        unsigned int                     END_RCVD_62 : 1;
        unsigned int                      DISCARD_62 : 1;
        unsigned int                     END_RCVD_61 : 1;
        unsigned int                      DISCARD_61 : 1;
        unsigned int                     END_RCVD_60 : 1;
        unsigned int                      DISCARD_60 : 1;
        unsigned int                     END_RCVD_59 : 1;
        unsigned int                      DISCARD_59 : 1;
        unsigned int                     END_RCVD_58 : 1;
        unsigned int                      DISCARD_58 : 1;
        unsigned int                     END_RCVD_57 : 1;
        unsigned int                      DISCARD_57 : 1;
        unsigned int                     END_RCVD_56 : 1;
        unsigned int                      DISCARD_56 : 1;
        unsigned int                     END_RCVD_55 : 1;
        unsigned int                      DISCARD_55 : 1;
        unsigned int                     END_RCVD_54 : 1;
        unsigned int                      DISCARD_54 : 1;
        unsigned int                     END_RCVD_53 : 1;
        unsigned int                      DISCARD_53 : 1;
        unsigned int                     END_RCVD_52 : 1;
        unsigned int                      DISCARD_52 : 1;
        unsigned int                     END_RCVD_51 : 1;
        unsigned int                      DISCARD_51 : 1;
        unsigned int                     END_RCVD_50 : 1;
        unsigned int                      DISCARD_50 : 1;
        unsigned int                     END_RCVD_49 : 1;
        unsigned int                      DISCARD_49 : 1;
        unsigned int                     END_RCVD_48 : 1;
        unsigned int                      DISCARD_48 : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_STATE_DEBUG_INDEX {
    struct {
#if     defined(qLittleEndian)
        unsigned int               STATE_DEBUG_INDEX : 5;
        unsigned int                                 : 27;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 27;
        unsigned int               STATE_DEBUG_INDEX : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_STATE_DEBUG_DATA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                STATE_DEBUG_DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                STATE_DEBUG_DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_PROG_COUNTER {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         COUNTER : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                         COUNTER : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union CP_STAT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     MIU_WR_BUSY : 1;
        unsigned int                 MIU_RD_REQ_BUSY : 1;
        unsigned int              MIU_RD_RETURN_BUSY : 1;
        unsigned int                       RBIU_BUSY : 1;
        unsigned int                       RCIU_BUSY : 1;
        unsigned int                   CSF_RING_BUSY : 1;
        unsigned int              CSF_INDIRECTS_BUSY : 1;
        unsigned int              CSF_INDIRECT2_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int                     CSF_ST_BUSY : 1;
        unsigned int                        CSF_BUSY : 1;
        unsigned int                 RING_QUEUE_BUSY : 1;
        unsigned int            INDIRECTS_QUEUE_BUSY : 1;
        unsigned int            INDIRECT2_QUEUE_BUSY : 1;
        unsigned int                                 : 2;
        unsigned int                   ST_QUEUE_BUSY : 1;
        unsigned int                        PFP_BUSY : 1;
        unsigned int                   MEQ_RING_BUSY : 1;
        unsigned int              MEQ_INDIRECTS_BUSY : 1;
        unsigned int              MEQ_INDIRECT2_BUSY : 1;
        unsigned int                    MIU_WC_STALL : 1;
        unsigned int                     CP_NRT_BUSY : 1;
        unsigned int                        _3D_BUSY : 1;
        unsigned int                                 : 2;
        unsigned int                         ME_BUSY : 1;
        unsigned int                                 : 2;
        unsigned int                      ME_WC_BUSY : 1;
        unsigned int         MIU_WC_TRACK_FIFO_EMPTY : 1;
        unsigned int                         CP_BUSY : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                         CP_BUSY : 1;
        unsigned int         MIU_WC_TRACK_FIFO_EMPTY : 1;
        unsigned int                      ME_WC_BUSY : 1;
        unsigned int                                 : 2;
        unsigned int                         ME_BUSY : 1;
        unsigned int                                 : 2;
        unsigned int                        _3D_BUSY : 1;
        unsigned int                     CP_NRT_BUSY : 1;
        unsigned int                    MIU_WC_STALL : 1;
        unsigned int              MEQ_INDIRECT2_BUSY : 1;
        unsigned int              MEQ_INDIRECTS_BUSY : 1;
        unsigned int                   MEQ_RING_BUSY : 1;
        unsigned int                        PFP_BUSY : 1;
        unsigned int                   ST_QUEUE_BUSY : 1;
        unsigned int                                 : 2;
        unsigned int            INDIRECT2_QUEUE_BUSY : 1;
        unsigned int            INDIRECTS_QUEUE_BUSY : 1;
        unsigned int                 RING_QUEUE_BUSY : 1;
        unsigned int                        CSF_BUSY : 1;
        unsigned int                     CSF_ST_BUSY : 1;
        unsigned int                                 : 1;
        unsigned int              CSF_INDIRECT2_BUSY : 1;
        unsigned int              CSF_INDIRECTS_BUSY : 1;
        unsigned int                   CSF_RING_BUSY : 1;
        unsigned int                       RCIU_BUSY : 1;
        unsigned int                       RBIU_BUSY : 1;
        unsigned int              MIU_RD_RETURN_BUSY : 1;
        unsigned int                 MIU_RD_REQ_BUSY : 1;
        unsigned int                     MIU_WR_BUSY : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_0_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_1_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_2_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_3_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_4_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_5_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_6_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_7_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_8_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_9_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_10_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_11_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_12_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_13_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_14_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BIOS_15_SCRATCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    BIOS_SCRATCH : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                    BIOS_SCRATCH : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_SIZE_PM4 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            SIZE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                            SIZE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_BASE_PM4 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            BASE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                            BASE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_STATUS_PM4 {
    struct {
#if     defined(qLittleEndian)
        unsigned int               MATCHING_CONTEXTS : 8;
        unsigned int           RB_COPY_DEST_BASE_ENA : 1;
        unsigned int                 DEST_BASE_0_ENA : 1;
        unsigned int                 DEST_BASE_1_ENA : 1;
        unsigned int                 DEST_BASE_2_ENA : 1;
        unsigned int                 DEST_BASE_3_ENA : 1;
        unsigned int                 DEST_BASE_4_ENA : 1;
        unsigned int                 DEST_BASE_5_ENA : 1;
        unsigned int                 DEST_BASE_6_ENA : 1;
        unsigned int                 DEST_BASE_7_ENA : 1;
        unsigned int               RB_COLOR_INFO_ENA : 1;
        unsigned int                                 : 7;
        unsigned int                   TC_ACTION_ENA : 1;
        unsigned int                                 : 5;
        unsigned int                          STATUS : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                          STATUS : 1;
        unsigned int                                 : 5;
        unsigned int                   TC_ACTION_ENA : 1;
        unsigned int                                 : 7;
        unsigned int               RB_COLOR_INFO_ENA : 1;
        unsigned int                 DEST_BASE_7_ENA : 1;
        unsigned int                 DEST_BASE_6_ENA : 1;
        unsigned int                 DEST_BASE_5_ENA : 1;
        unsigned int                 DEST_BASE_4_ENA : 1;
        unsigned int                 DEST_BASE_3_ENA : 1;
        unsigned int                 DEST_BASE_2_ENA : 1;
        unsigned int                 DEST_BASE_1_ENA : 1;
        unsigned int                 DEST_BASE_0_ENA : 1;
        unsigned int           RB_COPY_DEST_BASE_ENA : 1;
        unsigned int               MATCHING_CONTEXTS : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_SIZE_HOST {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            SIZE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                            SIZE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_BASE_HOST {
    struct {
#if     defined(qLittleEndian)
        unsigned int                            BASE : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                            BASE : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_STATUS_HOST {
    struct {
#if     defined(qLittleEndian)
        unsigned int               MATCHING_CONTEXTS : 8;
        unsigned int           RB_COPY_DEST_BASE_ENA : 1;
        unsigned int                 DEST_BASE_0_ENA : 1;
        unsigned int                 DEST_BASE_1_ENA : 1;
        unsigned int                 DEST_BASE_2_ENA : 1;
        unsigned int                 DEST_BASE_3_ENA : 1;
        unsigned int                 DEST_BASE_4_ENA : 1;
        unsigned int                 DEST_BASE_5_ENA : 1;
        unsigned int                 DEST_BASE_6_ENA : 1;
        unsigned int                 DEST_BASE_7_ENA : 1;
        unsigned int               RB_COLOR_INFO_ENA : 1;
        unsigned int                                 : 7;
        unsigned int                   TC_ACTION_ENA : 1;
        unsigned int                                 : 5;
        unsigned int                          STATUS : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                          STATUS : 1;
        unsigned int                                 : 5;
        unsigned int                   TC_ACTION_ENA : 1;
        unsigned int                                 : 7;
        unsigned int               RB_COLOR_INFO_ENA : 1;
        unsigned int                 DEST_BASE_7_ENA : 1;
        unsigned int                 DEST_BASE_6_ENA : 1;
        unsigned int                 DEST_BASE_5_ENA : 1;
        unsigned int                 DEST_BASE_4_ENA : 1;
        unsigned int                 DEST_BASE_3_ENA : 1;
        unsigned int                 DEST_BASE_2_ENA : 1;
        unsigned int                 DEST_BASE_1_ENA : 1;
        unsigned int                 DEST_BASE_0_ENA : 1;
        unsigned int           RB_COPY_DEST_BASE_ENA : 1;
        unsigned int               MATCHING_CONTEXTS : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_DEST_BASE_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                     DEST_BASE_0 : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                     DEST_BASE_0 : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_DEST_BASE_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                     DEST_BASE_1 : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                     DEST_BASE_1 : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_DEST_BASE_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                     DEST_BASE_2 : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                     DEST_BASE_2 : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_DEST_BASE_3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                     DEST_BASE_3 : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                     DEST_BASE_3 : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_DEST_BASE_4 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                     DEST_BASE_4 : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                     DEST_BASE_4 : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_DEST_BASE_5 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                     DEST_BASE_5 : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                     DEST_BASE_5 : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_DEST_BASE_6 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                     DEST_BASE_6 : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                     DEST_BASE_6 : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union COHER_DEST_BASE_7 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                     DEST_BASE_7 : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                     DEST_BASE_7 : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_SURFACE_INFO {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   SURFACE_PITCH : 14;
        unsigned int                    MSAA_SAMPLES : 2;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                    MSAA_SAMPLES : 2;
        unsigned int                   SURFACE_PITCH : 14;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_COLOR_INFO {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    COLOR_FORMAT : 4;
        unsigned int                COLOR_ROUND_MODE : 2;
        unsigned int                    COLOR_LINEAR : 1;
        unsigned int                    COLOR_ENDIAN : 2;
        unsigned int                      COLOR_SWAP : 2;
        unsigned int                                 : 1;
        unsigned int                      COLOR_BASE : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                      COLOR_BASE : 20;
        unsigned int                                 : 1;
        unsigned int                      COLOR_SWAP : 2;
        unsigned int                    COLOR_ENDIAN : 2;
        unsigned int                    COLOR_LINEAR : 1;
        unsigned int                COLOR_ROUND_MODE : 2;
        unsigned int                    COLOR_FORMAT : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_DEPTH_INFO {
    struct {
#if     defined(qLittleEndian)
        unsigned int                    DEPTH_FORMAT : 1;
        unsigned int                                 : 11;
        unsigned int                      DEPTH_BASE : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                      DEPTH_BASE : 20;
        unsigned int                                 : 11;
        unsigned int                    DEPTH_FORMAT : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_STENCILREFMASK {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      STENCILREF : 8;
        unsigned int                     STENCILMASK : 8;
        unsigned int                STENCILWRITEMASK : 8;
        unsigned int                       RESERVED0 : 1;
        unsigned int                       RESERVED1 : 1;
        unsigned int                                 : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 6;
        unsigned int                       RESERVED1 : 1;
        unsigned int                       RESERVED0 : 1;
        unsigned int                STENCILWRITEMASK : 8;
        unsigned int                     STENCILMASK : 8;
        unsigned int                      STENCILREF : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_ALPHA_REF {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       ALPHA_REF : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                       ALPHA_REF : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_COLOR_MASK {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       WRITE_RED : 1;
        unsigned int                     WRITE_GREEN : 1;
        unsigned int                      WRITE_BLUE : 1;
        unsigned int                     WRITE_ALPHA : 1;
        unsigned int                       RESERVED2 : 1;
        unsigned int                       RESERVED3 : 1;
        unsigned int                                 : 26;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 26;
        unsigned int                       RESERVED3 : 1;
        unsigned int                       RESERVED2 : 1;
        unsigned int                     WRITE_ALPHA : 1;
        unsigned int                      WRITE_BLUE : 1;
        unsigned int                     WRITE_GREEN : 1;
        unsigned int                       WRITE_RED : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_BLEND_RED {
    struct {
#if     defined(qLittleEndian)
        unsigned int                       BLEND_RED : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                       BLEND_RED : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_BLEND_GREEN {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     BLEND_GREEN : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                     BLEND_GREEN : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_BLEND_BLUE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      BLEND_BLUE : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                      BLEND_BLUE : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_BLEND_ALPHA {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     BLEND_ALPHA : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                     BLEND_ALPHA : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_FOG_COLOR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                         FOG_RED : 8;
        unsigned int                       FOG_GREEN : 8;
        unsigned int                        FOG_BLUE : 8;
        unsigned int                                 : 8;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 8;
        unsigned int                        FOG_BLUE : 8;
        unsigned int                       FOG_GREEN : 8;
        unsigned int                         FOG_RED : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_STENCILREFMASK_BF {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   STENCILREF_BF : 8;
        unsigned int                  STENCILMASK_BF : 8;
        unsigned int             STENCILWRITEMASK_BF : 8;
        unsigned int                       RESERVED4 : 1;
        unsigned int                       RESERVED5 : 1;
        unsigned int                                 : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 6;
        unsigned int                       RESERVED5 : 1;
        unsigned int                       RESERVED4 : 1;
        unsigned int             STENCILWRITEMASK_BF : 8;
        unsigned int                  STENCILMASK_BF : 8;
        unsigned int                   STENCILREF_BF : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_DEPTHCONTROL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  STENCIL_ENABLE : 1;
        unsigned int                        Z_ENABLE : 1;
        unsigned int                  Z_WRITE_ENABLE : 1;
        unsigned int                  EARLY_Z_ENABLE : 1;
        unsigned int                           ZFUNC : 3;
        unsigned int                 BACKFACE_ENABLE : 1;
        unsigned int                     STENCILFUNC : 3;
        unsigned int                     STENCILFAIL : 3;
        unsigned int                    STENCILZPASS : 3;
        unsigned int                    STENCILZFAIL : 3;
        unsigned int                  STENCILFUNC_BF : 3;
        unsigned int                  STENCILFAIL_BF : 3;
        unsigned int                 STENCILZPASS_BF : 3;
        unsigned int                 STENCILZFAIL_BF : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                 STENCILZFAIL_BF : 3;
        unsigned int                 STENCILZPASS_BF : 3;
        unsigned int                  STENCILFAIL_BF : 3;
        unsigned int                  STENCILFUNC_BF : 3;
        unsigned int                    STENCILZFAIL : 3;
        unsigned int                    STENCILZPASS : 3;
        unsigned int                     STENCILFAIL : 3;
        unsigned int                     STENCILFUNC : 3;
        unsigned int                 BACKFACE_ENABLE : 1;
        unsigned int                           ZFUNC : 3;
        unsigned int                  EARLY_Z_ENABLE : 1;
        unsigned int                  Z_WRITE_ENABLE : 1;
        unsigned int                        Z_ENABLE : 1;
        unsigned int                  STENCIL_ENABLE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_BLENDCONTROL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  COLOR_SRCBLEND : 5;
        unsigned int                  COLOR_COMB_FCN : 3;
        unsigned int                 COLOR_DESTBLEND : 5;
        unsigned int                                 : 3;
        unsigned int                  ALPHA_SRCBLEND : 5;
        unsigned int                  ALPHA_COMB_FCN : 3;
        unsigned int                 ALPHA_DESTBLEND : 5;
        unsigned int              BLEND_FORCE_ENABLE : 1;
        unsigned int                     BLEND_FORCE : 1;
        unsigned int                                 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 1;
        unsigned int                     BLEND_FORCE : 1;
        unsigned int              BLEND_FORCE_ENABLE : 1;
        unsigned int                 ALPHA_DESTBLEND : 5;
        unsigned int                  ALPHA_COMB_FCN : 3;
        unsigned int                  ALPHA_SRCBLEND : 5;
        unsigned int                                 : 3;
        unsigned int                 COLOR_DESTBLEND : 5;
        unsigned int                  COLOR_COMB_FCN : 3;
        unsigned int                  COLOR_SRCBLEND : 5;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_COLORCONTROL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      ALPHA_FUNC : 3;
        unsigned int               ALPHA_TEST_ENABLE : 1;
        unsigned int            ALPHA_TO_MASK_ENABLE : 1;
        unsigned int                   BLEND_DISABLE : 1;
        unsigned int                      FOG_ENABLE : 1;
        unsigned int                  VS_EXPORTS_FOG : 1;
        unsigned int                        ROP_CODE : 4;
        unsigned int                     DITHER_MODE : 2;
        unsigned int                     DITHER_TYPE : 2;
        unsigned int                       PIXEL_FOG : 1;
        unsigned int                                 : 7;
        unsigned int           ALPHA_TO_MASK_OFFSET0 : 2;
        unsigned int           ALPHA_TO_MASK_OFFSET1 : 2;
        unsigned int           ALPHA_TO_MASK_OFFSET2 : 2;
        unsigned int           ALPHA_TO_MASK_OFFSET3 : 2;
#else       /* !defined(qLittleEndian) */
        unsigned int           ALPHA_TO_MASK_OFFSET3 : 2;
        unsigned int           ALPHA_TO_MASK_OFFSET2 : 2;
        unsigned int           ALPHA_TO_MASK_OFFSET1 : 2;
        unsigned int           ALPHA_TO_MASK_OFFSET0 : 2;
        unsigned int                                 : 7;
        unsigned int                       PIXEL_FOG : 1;
        unsigned int                     DITHER_TYPE : 2;
        unsigned int                     DITHER_MODE : 2;
        unsigned int                        ROP_CODE : 4;
        unsigned int                  VS_EXPORTS_FOG : 1;
        unsigned int                      FOG_ENABLE : 1;
        unsigned int                   BLEND_DISABLE : 1;
        unsigned int            ALPHA_TO_MASK_ENABLE : 1;
        unsigned int               ALPHA_TEST_ENABLE : 1;
        unsigned int                      ALPHA_FUNC : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_MODECONTROL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      EDRAM_MODE : 3;
        unsigned int                                 : 29;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 29;
        unsigned int                      EDRAM_MODE : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_COLOR_DEST_MASK {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 COLOR_DEST_MASK : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                 COLOR_DEST_MASK : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_COPY_CONTROL {
    struct {
#if     defined(qLittleEndian)
        unsigned int              COPY_SAMPLE_SELECT : 3;
        unsigned int              DEPTH_CLEAR_ENABLE : 1;
        unsigned int                      CLEAR_MASK : 4;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                      CLEAR_MASK : 4;
        unsigned int              DEPTH_CLEAR_ENABLE : 1;
        unsigned int              COPY_SAMPLE_SELECT : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_COPY_DEST_BASE {
    struct {
#if     defined(qLittleEndian)
        unsigned int                                 : 12;
        unsigned int                  COPY_DEST_BASE : 20;
#else       /* !defined(qLittleEndian) */
        unsigned int                  COPY_DEST_BASE : 20;
        unsigned int                                 : 12;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_COPY_DEST_PITCH {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 COPY_DEST_PITCH : 9;
        unsigned int                                 : 23;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 23;
        unsigned int                 COPY_DEST_PITCH : 9;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_COPY_DEST_INFO {
    struct {
#if     defined(qLittleEndian)
        unsigned int                COPY_DEST_ENDIAN : 3;
        unsigned int                COPY_DEST_LINEAR : 1;
        unsigned int                COPY_DEST_FORMAT : 4;
        unsigned int                  COPY_DEST_SWAP : 2;
        unsigned int           COPY_DEST_DITHER_MODE : 2;
        unsigned int           COPY_DEST_DITHER_TYPE : 2;
        unsigned int             COPY_MASK_WRITE_RED : 1;
        unsigned int           COPY_MASK_WRITE_GREEN : 1;
        unsigned int            COPY_MASK_WRITE_BLUE : 1;
        unsigned int           COPY_MASK_WRITE_ALPHA : 1;
        unsigned int                                 : 14;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 14;
        unsigned int           COPY_MASK_WRITE_ALPHA : 1;
        unsigned int            COPY_MASK_WRITE_BLUE : 1;
        unsigned int           COPY_MASK_WRITE_GREEN : 1;
        unsigned int             COPY_MASK_WRITE_RED : 1;
        unsigned int           COPY_DEST_DITHER_TYPE : 2;
        unsigned int           COPY_DEST_DITHER_MODE : 2;
        unsigned int                  COPY_DEST_SWAP : 2;
        unsigned int                COPY_DEST_FORMAT : 4;
        unsigned int                COPY_DEST_LINEAR : 1;
        unsigned int                COPY_DEST_ENDIAN : 3;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_COPY_DEST_PIXEL_OFFSET {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        OFFSET_X : 13;
        unsigned int                        OFFSET_Y : 13;
        unsigned int                                 : 6;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 6;
        unsigned int                        OFFSET_Y : 13;
        unsigned int                        OFFSET_X : 13;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_DEPTH_CLEAR {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     DEPTH_CLEAR : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                     DEPTH_CLEAR : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_SAMPLE_COUNT_CTL {
    struct {
#if     defined(qLittleEndian)
        unsigned int              RESET_SAMPLE_COUNT : 1;
        unsigned int               COPY_SAMPLE_COUNT : 1;
        unsigned int                                 : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 30;
        unsigned int               COPY_SAMPLE_COUNT : 1;
        unsigned int              RESET_SAMPLE_COUNT : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_SAMPLE_COUNT_ADDR {
    struct {
#if     defined(qLittleEndian)
        unsigned int               SAMPLE_COUNT_ADDR : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int               SAMPLE_COUNT_ADDR : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_BC_CONTROL {
    struct {
#if     defined(qLittleEndian)
        unsigned int        ACCUM_LINEAR_MODE_ENABLE : 1;
        unsigned int            ACCUM_TIMEOUT_SELECT : 2;
        unsigned int               DISABLE_EDRAM_CAM : 1;
        unsigned int  DISABLE_EZ_FAST_CONTEXT_SWITCH : 1;
        unsigned int       DISABLE_EZ_NULL_ZCMD_DROP : 1;
        unsigned int       DISABLE_LZ_NULL_ZCMD_DROP : 1;
        unsigned int              ENABLE_AZ_THROTTLE : 1;
        unsigned int               AZ_THROTTLE_COUNT : 5;
        unsigned int                                 : 1;
        unsigned int               ENABLE_CRC_UPDATE : 1;
        unsigned int                        CRC_MODE : 1;
        unsigned int         DISABLE_SAMPLE_COUNTERS : 1;
        unsigned int                   DISABLE_ACCUM : 1;
        unsigned int                ACCUM_ALLOC_MASK : 4;
        unsigned int       LINEAR_PERFORMANCE_ENABLE : 1;
        unsigned int           ACCUM_DATA_FIFO_LIMIT : 4;
        unsigned int       MEM_EXPORT_TIMEOUT_SELECT : 2;
        unsigned int   MEM_EXPORT_LINEAR_MODE_ENABLE : 1;
        unsigned int                      CRC_SYSTEM : 1;
        unsigned int                       RESERVED6 : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                       RESERVED6 : 1;
        unsigned int                      CRC_SYSTEM : 1;
        unsigned int   MEM_EXPORT_LINEAR_MODE_ENABLE : 1;
        unsigned int       MEM_EXPORT_TIMEOUT_SELECT : 2;
        unsigned int           ACCUM_DATA_FIFO_LIMIT : 4;
        unsigned int       LINEAR_PERFORMANCE_ENABLE : 1;
        unsigned int                ACCUM_ALLOC_MASK : 4;
        unsigned int                   DISABLE_ACCUM : 1;
        unsigned int         DISABLE_SAMPLE_COUNTERS : 1;
        unsigned int                        CRC_MODE : 1;
        unsigned int               ENABLE_CRC_UPDATE : 1;
        unsigned int                                 : 1;
        unsigned int               AZ_THROTTLE_COUNT : 5;
        unsigned int              ENABLE_AZ_THROTTLE : 1;
        unsigned int       DISABLE_LZ_NULL_ZCMD_DROP : 1;
        unsigned int       DISABLE_EZ_NULL_ZCMD_DROP : 1;
        unsigned int  DISABLE_EZ_FAST_CONTEXT_SWITCH : 1;
        unsigned int               DISABLE_EDRAM_CAM : 1;
        unsigned int            ACCUM_TIMEOUT_SELECT : 2;
        unsigned int        ACCUM_LINEAR_MODE_ENABLE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_EDRAM_INFO {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      EDRAM_SIZE : 4;
        unsigned int              EDRAM_MAPPING_MODE : 2;
        unsigned int                                 : 8;
        unsigned int                     EDRAM_RANGE : 18;
#else       /* !defined(qLittleEndian) */
        unsigned int                     EDRAM_RANGE : 18;
        unsigned int                                 : 8;
        unsigned int              EDRAM_MAPPING_MODE : 2;
        unsigned int                      EDRAM_SIZE : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_CRC_RD_PORT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        CRC_DATA : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                        CRC_DATA : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_CRC_CONTROL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                  CRC_RD_ADVANCE : 1;
        unsigned int                                 : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 31;
        unsigned int                  CRC_RD_ADVANCE : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_CRC_MASK {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        CRC_MASK : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                        CRC_MASK : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_PERFCOUNTER0_SELECT {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        PERF_SEL : 8;
        unsigned int                                 : 24;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 24;
        unsigned int                        PERF_SEL : 8;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_PERFCOUNTER0_LOW {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                      PERF_COUNT : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_PERFCOUNTER0_HI {
    struct {
#if     defined(qLittleEndian)
        unsigned int                      PERF_COUNT : 16;
        unsigned int                                 : 16;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 16;
        unsigned int                      PERF_COUNT : 16;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_TOTAL_SAMPLES {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   TOTAL_SAMPLES : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   TOTAL_SAMPLES : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_ZPASS_SAMPLES {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   ZPASS_SAMPLES : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   ZPASS_SAMPLES : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_ZFAIL_SAMPLES {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   ZFAIL_SAMPLES : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   ZFAIL_SAMPLES : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_SFAIL_SAMPLES {
    struct {
#if     defined(qLittleEndian)
        unsigned int                   SFAIL_SAMPLES : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                   SFAIL_SAMPLES : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_DEBUG_0 {
    struct {
#if     defined(qLittleEndian)
        unsigned int           RDREQ_CTL_Z1_PRE_FULL : 1;
        unsigned int           RDREQ_CTL_Z0_PRE_FULL : 1;
        unsigned int           RDREQ_CTL_C1_PRE_FULL : 1;
        unsigned int           RDREQ_CTL_C0_PRE_FULL : 1;
        unsigned int          RDREQ_E1_ORDERING_FULL : 1;
        unsigned int          RDREQ_E0_ORDERING_FULL : 1;
        unsigned int                   RDREQ_Z1_FULL : 1;
        unsigned int                   RDREQ_Z0_FULL : 1;
        unsigned int                   RDREQ_C1_FULL : 1;
        unsigned int                   RDREQ_C0_FULL : 1;
        unsigned int          WRREQ_E1_MACRO_HI_FULL : 1;
        unsigned int          WRREQ_E1_MACRO_LO_FULL : 1;
        unsigned int          WRREQ_E0_MACRO_HI_FULL : 1;
        unsigned int          WRREQ_E0_MACRO_LO_FULL : 1;
        unsigned int              WRREQ_C_WE_HI_FULL : 1;
        unsigned int              WRREQ_C_WE_LO_FULL : 1;
        unsigned int                   WRREQ_Z1_FULL : 1;
        unsigned int                   WRREQ_Z0_FULL : 1;
        unsigned int                   WRREQ_C1_FULL : 1;
        unsigned int                   WRREQ_C0_FULL : 1;
        unsigned int            CMDFIFO_Z1_HOLD_FULL : 1;
        unsigned int            CMDFIFO_Z0_HOLD_FULL : 1;
        unsigned int            CMDFIFO_C1_HOLD_FULL : 1;
        unsigned int            CMDFIFO_C0_HOLD_FULL : 1;
        unsigned int         CMDFIFO_Z_ORDERING_FULL : 1;
        unsigned int         CMDFIFO_C_ORDERING_FULL : 1;
        unsigned int                   C_SX_LAT_FULL : 1;
        unsigned int                   C_SX_CMD_FULL : 1;
        unsigned int                  C_EZ_TILE_FULL : 1;
        unsigned int                      C_REQ_FULL : 1;
        unsigned int                     C_MASK_FULL : 1;
        unsigned int                 EZ_INFSAMP_FULL : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                 EZ_INFSAMP_FULL : 1;
        unsigned int                     C_MASK_FULL : 1;
        unsigned int                      C_REQ_FULL : 1;
        unsigned int                  C_EZ_TILE_FULL : 1;
        unsigned int                   C_SX_CMD_FULL : 1;
        unsigned int                   C_SX_LAT_FULL : 1;
        unsigned int         CMDFIFO_C_ORDERING_FULL : 1;
        unsigned int         CMDFIFO_Z_ORDERING_FULL : 1;
        unsigned int            CMDFIFO_C0_HOLD_FULL : 1;
        unsigned int            CMDFIFO_C1_HOLD_FULL : 1;
        unsigned int            CMDFIFO_Z0_HOLD_FULL : 1;
        unsigned int            CMDFIFO_Z1_HOLD_FULL : 1;
        unsigned int                   WRREQ_C0_FULL : 1;
        unsigned int                   WRREQ_C1_FULL : 1;
        unsigned int                   WRREQ_Z0_FULL : 1;
        unsigned int                   WRREQ_Z1_FULL : 1;
        unsigned int              WRREQ_C_WE_LO_FULL : 1;
        unsigned int              WRREQ_C_WE_HI_FULL : 1;
        unsigned int          WRREQ_E0_MACRO_LO_FULL : 1;
        unsigned int          WRREQ_E0_MACRO_HI_FULL : 1;
        unsigned int          WRREQ_E1_MACRO_LO_FULL : 1;
        unsigned int          WRREQ_E1_MACRO_HI_FULL : 1;
        unsigned int                   RDREQ_C0_FULL : 1;
        unsigned int                   RDREQ_C1_FULL : 1;
        unsigned int                   RDREQ_Z0_FULL : 1;
        unsigned int                   RDREQ_Z1_FULL : 1;
        unsigned int          RDREQ_E0_ORDERING_FULL : 1;
        unsigned int          RDREQ_E1_ORDERING_FULL : 1;
        unsigned int           RDREQ_CTL_C0_PRE_FULL : 1;
        unsigned int           RDREQ_CTL_C1_PRE_FULL : 1;
        unsigned int           RDREQ_CTL_Z0_PRE_FULL : 1;
        unsigned int           RDREQ_CTL_Z1_PRE_FULL : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_DEBUG_1 {
    struct {
#if     defined(qLittleEndian)
        unsigned int              RDREQ_Z1_CMD_EMPTY : 1;
        unsigned int              RDREQ_Z0_CMD_EMPTY : 1;
        unsigned int              RDREQ_C1_CMD_EMPTY : 1;
        unsigned int              RDREQ_C0_CMD_EMPTY : 1;
        unsigned int         RDREQ_E1_ORDERING_EMPTY : 1;
        unsigned int         RDREQ_E0_ORDERING_EMPTY : 1;
        unsigned int                  RDREQ_Z1_EMPTY : 1;
        unsigned int                  RDREQ_Z0_EMPTY : 1;
        unsigned int                  RDREQ_C1_EMPTY : 1;
        unsigned int                  RDREQ_C0_EMPTY : 1;
        unsigned int         WRREQ_E1_MACRO_HI_EMPTY : 1;
        unsigned int         WRREQ_E1_MACRO_LO_EMPTY : 1;
        unsigned int         WRREQ_E0_MACRO_HI_EMPTY : 1;
        unsigned int         WRREQ_E0_MACRO_LO_EMPTY : 1;
        unsigned int             WRREQ_C_WE_HI_EMPTY : 1;
        unsigned int             WRREQ_C_WE_LO_EMPTY : 1;
        unsigned int                  WRREQ_Z1_EMPTY : 1;
        unsigned int                  WRREQ_Z0_EMPTY : 1;
        unsigned int              WRREQ_C1_PRE_EMPTY : 1;
        unsigned int              WRREQ_C0_PRE_EMPTY : 1;
        unsigned int           CMDFIFO_Z1_HOLD_EMPTY : 1;
        unsigned int           CMDFIFO_Z0_HOLD_EMPTY : 1;
        unsigned int           CMDFIFO_C1_HOLD_EMPTY : 1;
        unsigned int           CMDFIFO_C0_HOLD_EMPTY : 1;
        unsigned int        CMDFIFO_Z_ORDERING_EMPTY : 1;
        unsigned int        CMDFIFO_C_ORDERING_EMPTY : 1;
        unsigned int                  C_SX_LAT_EMPTY : 1;
        unsigned int                  C_SX_CMD_EMPTY : 1;
        unsigned int                 C_EZ_TILE_EMPTY : 1;
        unsigned int                     C_REQ_EMPTY : 1;
        unsigned int                    C_MASK_EMPTY : 1;
        unsigned int                EZ_INFSAMP_EMPTY : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                EZ_INFSAMP_EMPTY : 1;
        unsigned int                    C_MASK_EMPTY : 1;
        unsigned int                     C_REQ_EMPTY : 1;
        unsigned int                 C_EZ_TILE_EMPTY : 1;
        unsigned int                  C_SX_CMD_EMPTY : 1;
        unsigned int                  C_SX_LAT_EMPTY : 1;
        unsigned int        CMDFIFO_C_ORDERING_EMPTY : 1;
        unsigned int        CMDFIFO_Z_ORDERING_EMPTY : 1;
        unsigned int           CMDFIFO_C0_HOLD_EMPTY : 1;
        unsigned int           CMDFIFO_C1_HOLD_EMPTY : 1;
        unsigned int           CMDFIFO_Z0_HOLD_EMPTY : 1;
        unsigned int           CMDFIFO_Z1_HOLD_EMPTY : 1;
        unsigned int              WRREQ_C0_PRE_EMPTY : 1;
        unsigned int              WRREQ_C1_PRE_EMPTY : 1;
        unsigned int                  WRREQ_Z0_EMPTY : 1;
        unsigned int                  WRREQ_Z1_EMPTY : 1;
        unsigned int             WRREQ_C_WE_LO_EMPTY : 1;
        unsigned int             WRREQ_C_WE_HI_EMPTY : 1;
        unsigned int         WRREQ_E0_MACRO_LO_EMPTY : 1;
        unsigned int         WRREQ_E0_MACRO_HI_EMPTY : 1;
        unsigned int         WRREQ_E1_MACRO_LO_EMPTY : 1;
        unsigned int         WRREQ_E1_MACRO_HI_EMPTY : 1;
        unsigned int                  RDREQ_C0_EMPTY : 1;
        unsigned int                  RDREQ_C1_EMPTY : 1;
        unsigned int                  RDREQ_Z0_EMPTY : 1;
        unsigned int                  RDREQ_Z1_EMPTY : 1;
        unsigned int         RDREQ_E0_ORDERING_EMPTY : 1;
        unsigned int         RDREQ_E1_ORDERING_EMPTY : 1;
        unsigned int              RDREQ_C0_CMD_EMPTY : 1;
        unsigned int              RDREQ_C1_CMD_EMPTY : 1;
        unsigned int              RDREQ_Z0_CMD_EMPTY : 1;
        unsigned int              RDREQ_Z1_CMD_EMPTY : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_DEBUG_2 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                 TILE_FIFO_COUNT : 4;
        unsigned int               SX_LAT_FIFO_COUNT : 7;
        unsigned int                 MEM_EXPORT_FLAG : 1;
        unsigned int               SYSMEM_BLEND_FLAG : 1;
        unsigned int              CURRENT_TILE_EVENT : 1;
        unsigned int                 EZ_INFTILE_FULL : 1;
        unsigned int              EZ_MASK_LOWER_FULL : 1;
        unsigned int              EZ_MASK_UPPER_FULL : 1;
        unsigned int                    Z0_MASK_FULL : 1;
        unsigned int                    Z1_MASK_FULL : 1;
        unsigned int                     Z0_REQ_FULL : 1;
        unsigned int                     Z1_REQ_FULL : 1;
        unsigned int                     Z_SAMP_FULL : 1;
        unsigned int                     Z_TILE_FULL : 1;
        unsigned int                EZ_INFTILE_EMPTY : 1;
        unsigned int             EZ_MASK_LOWER_EMPTY : 1;
        unsigned int             EZ_MASK_UPPER_EMPTY : 1;
        unsigned int                   Z0_MASK_EMPTY : 1;
        unsigned int                   Z1_MASK_EMPTY : 1;
        unsigned int                    Z0_REQ_EMPTY : 1;
        unsigned int                    Z1_REQ_EMPTY : 1;
        unsigned int                    Z_SAMP_EMPTY : 1;
        unsigned int                    Z_TILE_EMPTY : 1;
#else       /* !defined(qLittleEndian) */
        unsigned int                    Z_TILE_EMPTY : 1;
        unsigned int                    Z_SAMP_EMPTY : 1;
        unsigned int                    Z1_REQ_EMPTY : 1;
        unsigned int                    Z0_REQ_EMPTY : 1;
        unsigned int                   Z1_MASK_EMPTY : 1;
        unsigned int                   Z0_MASK_EMPTY : 1;
        unsigned int             EZ_MASK_UPPER_EMPTY : 1;
        unsigned int             EZ_MASK_LOWER_EMPTY : 1;
        unsigned int                EZ_INFTILE_EMPTY : 1;
        unsigned int                     Z_TILE_FULL : 1;
        unsigned int                     Z_SAMP_FULL : 1;
        unsigned int                     Z1_REQ_FULL : 1;
        unsigned int                     Z0_REQ_FULL : 1;
        unsigned int                    Z1_MASK_FULL : 1;
        unsigned int                    Z0_MASK_FULL : 1;
        unsigned int              EZ_MASK_UPPER_FULL : 1;
        unsigned int              EZ_MASK_LOWER_FULL : 1;
        unsigned int                 EZ_INFTILE_FULL : 1;
        unsigned int              CURRENT_TILE_EVENT : 1;
        unsigned int               SYSMEM_BLEND_FLAG : 1;
        unsigned int                 MEM_EXPORT_FLAG : 1;
        unsigned int               SX_LAT_FIFO_COUNT : 7;
        unsigned int                 TILE_FIFO_COUNT : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_DEBUG_3 {
    struct {
#if     defined(qLittleEndian)
        unsigned int                     ACCUM_VALID : 4;
        unsigned int                  ACCUM_FLUSHING : 4;
        unsigned int         ACCUM_WRITE_CLEAN_COUNT : 6;
        unsigned int           ACCUM_INPUT_REG_VALID : 1;
        unsigned int             ACCUM_DATA_FIFO_CNT : 4;
        unsigned int                        SHD_FULL : 1;
        unsigned int                       SHD_EMPTY : 1;
        unsigned int           EZ_RETURN_LOWER_EMPTY : 1;
        unsigned int           EZ_RETURN_UPPER_EMPTY : 1;
        unsigned int            EZ_RETURN_LOWER_FULL : 1;
        unsigned int            EZ_RETURN_UPPER_FULL : 1;
        unsigned int                ZEXP_LOWER_EMPTY : 1;
        unsigned int                ZEXP_UPPER_EMPTY : 1;
        unsigned int                 ZEXP_LOWER_FULL : 1;
        unsigned int                 ZEXP_UPPER_FULL : 1;
        unsigned int                                 : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 3;
        unsigned int                 ZEXP_UPPER_FULL : 1;
        unsigned int                 ZEXP_LOWER_FULL : 1;
        unsigned int                ZEXP_UPPER_EMPTY : 1;
        unsigned int                ZEXP_LOWER_EMPTY : 1;
        unsigned int            EZ_RETURN_UPPER_FULL : 1;
        unsigned int            EZ_RETURN_LOWER_FULL : 1;
        unsigned int           EZ_RETURN_UPPER_EMPTY : 1;
        unsigned int           EZ_RETURN_LOWER_EMPTY : 1;
        unsigned int                       SHD_EMPTY : 1;
        unsigned int                        SHD_FULL : 1;
        unsigned int             ACCUM_DATA_FIFO_CNT : 4;
        unsigned int           ACCUM_INPUT_REG_VALID : 1;
        unsigned int         ACCUM_WRITE_CLEAN_COUNT : 6;
        unsigned int                  ACCUM_FLUSHING : 4;
        unsigned int                     ACCUM_VALID : 4;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_DEBUG_4 {
    struct {
#if     defined(qLittleEndian)
        unsigned int             GMEM_RD_ACCESS_FLAG : 1;
        unsigned int             GMEM_WR_ACCESS_FLAG : 1;
        unsigned int           SYSMEM_RD_ACCESS_FLAG : 1;
        unsigned int           SYSMEM_WR_ACCESS_FLAG : 1;
        unsigned int           ACCUM_DATA_FIFO_EMPTY : 1;
        unsigned int          ACCUM_ORDER_FIFO_EMPTY : 1;
        unsigned int            ACCUM_DATA_FIFO_FULL : 1;
        unsigned int           ACCUM_ORDER_FIFO_FULL : 1;
        unsigned int     SYSMEM_WRITE_COUNT_OVERFLOW : 1;
        unsigned int             CONTEXT_COUNT_DEBUG : 4;
        unsigned int                                 : 19;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 19;
        unsigned int             CONTEXT_COUNT_DEBUG : 4;
        unsigned int     SYSMEM_WRITE_COUNT_OVERFLOW : 1;
        unsigned int           ACCUM_ORDER_FIFO_FULL : 1;
        unsigned int            ACCUM_DATA_FIFO_FULL : 1;
        unsigned int          ACCUM_ORDER_FIFO_EMPTY : 1;
        unsigned int           ACCUM_DATA_FIFO_EMPTY : 1;
        unsigned int           SYSMEM_WR_ACCESS_FLAG : 1;
        unsigned int           SYSMEM_RD_ACCESS_FLAG : 1;
        unsigned int             GMEM_WR_ACCESS_FLAG : 1;
        unsigned int             GMEM_RD_ACCESS_FLAG : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_FLAG_CONTROL {
    struct {
#if     defined(qLittleEndian)
        unsigned int                DEBUG_FLAG_CLEAR : 1;
        unsigned int                                 : 31;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 31;
        unsigned int                DEBUG_FLAG_CLEAR : 1;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union RB_BC_SPARES {
    struct {
#if     defined(qLittleEndian)
        unsigned int                        RESERVED : 32;
#else       /* !defined(qLittleEndian) */
        unsigned int                        RESERVED : 32;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BC_DUMMY_CRAYRB_ENUMS {
    struct {
#if     defined(qLittleEndian)
        unsigned int       DUMMY_CRAYRB_DEPTH_FORMAT : 6;
        unsigned int       DUMMY_CRAYRB_SURFACE_SWAP : 1;
        unsigned int        DUMMY_CRAYRB_DEPTH_ARRAY : 2;
        unsigned int              DUMMY_CRAYRB_ARRAY : 2;
        unsigned int       DUMMY_CRAYRB_COLOR_FORMAT : 6;
        unsigned int     DUMMY_CRAYRB_SURFACE_NUMBER : 3;
        unsigned int     DUMMY_CRAYRB_SURFACE_FORMAT : 6;
        unsigned int     DUMMY_CRAYRB_SURFACE_TILING : 1;
        unsigned int      DUMMY_CRAYRB_SURFACE_ARRAY : 2;
        unsigned int  DUMMY_RB_COPY_DEST_INFO_NUMBER : 3;
#else       /* !defined(qLittleEndian) */
        unsigned int  DUMMY_RB_COPY_DEST_INFO_NUMBER : 3;
        unsigned int      DUMMY_CRAYRB_SURFACE_ARRAY : 2;
        unsigned int     DUMMY_CRAYRB_SURFACE_TILING : 1;
        unsigned int     DUMMY_CRAYRB_SURFACE_FORMAT : 6;
        unsigned int     DUMMY_CRAYRB_SURFACE_NUMBER : 3;
        unsigned int       DUMMY_CRAYRB_COLOR_FORMAT : 6;
        unsigned int              DUMMY_CRAYRB_ARRAY : 2;
        unsigned int        DUMMY_CRAYRB_DEPTH_ARRAY : 2;
        unsigned int       DUMMY_CRAYRB_SURFACE_SWAP : 1;
        unsigned int       DUMMY_CRAYRB_DEPTH_FORMAT : 6;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


    union BC_DUMMY_CRAYRB_MOREENUMS {
    struct {
#if     defined(qLittleEndian)
        unsigned int        DUMMY_CRAYRB_COLORARRAYX : 2;
        unsigned int                                 : 30;
#else       /* !defined(qLittleEndian) */
        unsigned int                                 : 30;
        unsigned int        DUMMY_CRAYRB_COLORARRAYX : 2;
#endif      /* defined(qLittleEndian) */
    } bitfields, bits;
    unsigned int    u32All;
    signed int  i32All;
    float   f32All;
    };


#endif
