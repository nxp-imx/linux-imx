/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __REGS_G4X_DRIVER_H
#define __REGS_G4X_DRIVER_H

#ifndef _LINUX
#include <assert.h>
#else
#ifndef assert
#define assert(expr)
#endif
#endif

//-----------------------------------------------------
// REGISTER ADDRESSES
//-----------------------------------------------------

#define ADDR_FBC_BASE                    0x84
#define ADDR_FBC_DATA                    0x86
#define ADDR_FBC_HEIGHT                  0x8a
#define ADDR_FBC_START                   0x8e
#define ADDR_FBC_STRIDE                  0x8c
#define ADDR_FBC_WIDTH                   0x88
#define ADDR_VGC_CLOCKEN                 0x508
#define ADDR_VGC_COMMANDSTREAM           0x0
#define ADDR_VGC_FIFOFREE                0x7c0
#define ADDR_VGC_IRQENABLE               0x438
#define ADDR_VGC_IRQSTATUS               0x418
#define ADDR_VGC_IRQ_ACTIVE_CNT          0x4e0
#define ADDR_VGC_MMUCOMMANDSTREAM        0x3fc
#define ADDR_VGC_REVISION                0x400
#define ADDR_VGC_SYSSTATUS               0x410
#define ADDR_G2D_ALPHABLEND              0xc
#define ADDR_G2D_BACKGROUND              0xb
#define ADDR_G2D_BASE0                   0x0
#define ADDR_G2D_BASE1                   0x2
#define ADDR_G2D_BASE2                   0x4
#define ADDR_G2D_BASE3                   0x6
#define ADDR_G2D_BLENDERCFG              0x11
#define ADDR_G2D_BLEND_A0                0x14
#define ADDR_G2D_BLEND_A1                0x15
#define ADDR_G2D_BLEND_A2                0x16
#define ADDR_G2D_BLEND_A3                0x17
#define ADDR_G2D_BLEND_C0                0x18
#define ADDR_G2D_BLEND_C1                0x19
#define ADDR_G2D_BLEND_C2                0x1a
#define ADDR_G2D_BLEND_C3                0x1b
#define ADDR_G2D_BLEND_C4                0x1c
#define ADDR_G2D_BLEND_C5                0x1d
#define ADDR_G2D_BLEND_C6                0x1e
#define ADDR_G2D_BLEND_C7                0x1f
#define ADDR_G2D_CFG0                    0x1
#define ADDR_G2D_CFG1                    0x3
#define ADDR_G2D_CFG2                    0x5
#define ADDR_G2D_CFG3                    0x7
#define ADDR_G2D_COLOR                   0xff
#define ADDR_G2D_CONFIG                  0xe
#define ADDR_G2D_CONST0                  0xb0
#define ADDR_G2D_CONST1                  0xb1
#define ADDR_G2D_CONST2                  0xb2
#define ADDR_G2D_CONST3                  0xb3
#define ADDR_G2D_CONST4                  0xb4
#define ADDR_G2D_CONST5                  0xb5
#define ADDR_G2D_CONST6                  0xb6
#define ADDR_G2D_CONST7                  0xb7
#define ADDR_G2D_FOREGROUND              0xa
#define ADDR_G2D_GRADIENT                0xd0
#define ADDR_G2D_IDLE                    0xfe
#define ADDR_G2D_INPUT                   0xf
#define ADDR_G2D_MASK                    0x10
#define ADDR_G2D_ROP                     0xd
#define ADDR_G2D_SCISSORX                0x8
#define ADDR_G2D_SCISSORY                0x9
#define ADDR_G2D_SXY                     0xf2
#define ADDR_G2D_SXY2                    0xf3
#define ADDR_G2D_VGSPAN                  0xf4
#define ADDR_G2D_WIDTHHEIGHT             0xf1
#define ADDR_G2D_XY                      0xf0
#define ADDR_GRADW_BORDERCOLOR           0xd4
#define ADDR_GRADW_CONST0                0xc0
#define ADDR_GRADW_CONST1                0xc1
#define ADDR_GRADW_CONST2                0xc2
#define ADDR_GRADW_CONST3                0xc3
#define ADDR_GRADW_CONST4                0xc4
#define ADDR_GRADW_CONST5                0xc5
#define ADDR_GRADW_CONST6                0xc6
#define ADDR_GRADW_CONST7                0xc7
#define ADDR_GRADW_CONST8                0xc8
#define ADDR_GRADW_CONST9                0xc9
#define ADDR_GRADW_CONSTA                0xca
#define ADDR_GRADW_CONSTB                0xcb
#define ADDR_GRADW_INST0                 0xe0
#define ADDR_GRADW_INST1                 0xe1
#define ADDR_GRADW_INST2                 0xe2
#define ADDR_GRADW_INST3                 0xe3
#define ADDR_GRADW_INST4                 0xe4
#define ADDR_GRADW_INST5                 0xe5
#define ADDR_GRADW_INST6                 0xe6
#define ADDR_GRADW_INST7                 0xe7
#define ADDR_GRADW_TEXBASE               0xd3
#define ADDR_GRADW_TEXCFG                0xd1
#define ADDR_GRADW_TEXSIZE               0xd2
#define ADDR_MH_ARBITER_CONFIG           0xa40
#define ADDR_MH_AXI_ERROR                0xa45
#define ADDR_MH_AXI_HALT_CONTROL         0xa50
#define ADDR_MH_CLNT_AXI_ID_REUSE        0xa41
#define ADDR_MH_DEBUG_CTRL               0xa4e
#define ADDR_MH_DEBUG_DATA               0xa4f
#define ADDR_MH_INTERRUPT_CLEAR          0xa44
#define ADDR_MH_INTERRUPT_MASK           0xa42
#define ADDR_MH_INTERRUPT_STATUS         0xa43
#define ADDR_MH_MMU_CONFIG               0x40
#define ADDR_MH_MMU_INVALIDATE           0x45
#define ADDR_MH_MMU_MPU_BASE             0x46
#define ADDR_MH_MMU_MPU_END              0x47
#define ADDR_MH_MMU_PAGE_FAULT           0x43
#define ADDR_MH_MMU_PT_BASE              0x42
#define ADDR_MH_MMU_TRAN_ERROR           0x44
#define ADDR_MH_MMU_VA_RANGE             0x41
#define ADDR_MH_PERFCOUNTER0_CONFIG      0xa47
#define ADDR_MH_PERFCOUNTER0_HI          0xa49
#define ADDR_MH_PERFCOUNTER0_LOW         0xa48
#define ADDR_MH_PERFCOUNTER0_SELECT      0xa46
#define ADDR_MH_PERFCOUNTER1_CONFIG      0xa4b
#define ADDR_MH_PERFCOUNTER1_HI          0xa4d
#define ADDR_MH_PERFCOUNTER1_LOW         0xa4c
#define ADDR_MH_PERFCOUNTER1_SELECT      0xa4a
#define ADDR_MMU_READ_ADDR               0x510
#define ADDR_MMU_READ_DATA               0x518
#define ADDR_VGV1_CBASE1                 0x2a
#define ADDR_VGV1_CFG1                   0x27
#define ADDR_VGV1_CFG2                   0x28
#define ADDR_VGV1_DIRTYBASE              0x29
#define ADDR_VGV1_FILL                   0x23
#define ADDR_VGV1_SCISSORX               0x24
#define ADDR_VGV1_SCISSORY               0x25
#define ADDR_VGV1_TILEOFS                0x22
#define ADDR_VGV1_UBASE2                 0x2b
#define ADDR_VGV1_VTX0                   0x20
#define ADDR_VGV1_VTX1                   0x21
#define ADDR_VGV2_ACCURACY               0x60
#define ADDR_VGV2_ACTION                 0x6f
#define ADDR_VGV2_ARCCOS                 0x62
#define ADDR_VGV2_ARCSIN                 0x63
#define ADDR_VGV2_ARCTAN                 0x64
#define ADDR_VGV2_BBOXMAXX               0x5c
#define ADDR_VGV2_BBOXMAXY               0x5d
#define ADDR_VGV2_BBOXMINX               0x5a
#define ADDR_VGV2_BBOXMINY               0x5b
#define ADDR_VGV2_BIAS                   0x5f
#define ADDR_VGV2_C1X                    0x40
#define ADDR_VGV2_C1XREL                 0x48
#define ADDR_VGV2_C1Y                    0x41
#define ADDR_VGV2_C1YREL                 0x49
#define ADDR_VGV2_C2X                    0x42
#define ADDR_VGV2_C2XREL                 0x4a
#define ADDR_VGV2_C2Y                    0x43
#define ADDR_VGV2_C2YREL                 0x4b
#define ADDR_VGV2_C3X                    0x44
#define ADDR_VGV2_C3XREL                 0x4c
#define ADDR_VGV2_C3Y                    0x45
#define ADDR_VGV2_C3YREL                 0x4d
#define ADDR_VGV2_C4X                    0x46
#define ADDR_VGV2_C4XREL                 0x4e
#define ADDR_VGV2_C4Y                    0x47
#define ADDR_VGV2_C4YREL                 0x4f
#define ADDR_VGV2_CLIP                   0x68
#define ADDR_VGV2_FIRST                  0x40
#define ADDR_VGV2_LAST                   0x6f
#define ADDR_VGV2_MITER                  0x66
#define ADDR_VGV2_MODE                   0x6e
#define ADDR_VGV2_RADIUS                 0x65
#define ADDR_VGV2_SCALE                  0x5e
#define ADDR_VGV2_THINRADIUS             0x61
#define ADDR_VGV2_XFSTXX                 0x56
#define ADDR_VGV2_XFSTXY                 0x58
#define ADDR_VGV2_XFSTYX                 0x57
#define ADDR_VGV2_XFSTYY                 0x59
#define ADDR_VGV2_XFXA                   0x54
#define ADDR_VGV2_XFXX                   0x50
#define ADDR_VGV2_XFXY                   0x52
#define ADDR_VGV2_XFYA                   0x55
#define ADDR_VGV2_XFYX                   0x51
#define ADDR_VGV2_XFYY                   0x53
#define ADDR_VGV3_CONTROL                0x70
#define ADDR_VGV3_FIRST                  0x70
#define ADDR_VGV3_LAST                   0x7f
#define ADDR_VGV3_MODE                   0x71
#define ADDR_VGV3_NEXTADDR               0x75
#define ADDR_VGV3_NEXTCMD                0x76
#define ADDR_VGV3_VGBYPASS               0x77
#define ADDR_VGV3_WRITE                  0x73
#define ADDR_VGV3_WRITEADDR              0x72
#define ADDR_VGV3_WRITEDMI               0x7d
#define ADDR_VGV3_WRITEF32               0x7b
#define ADDR_VGV3_WRITEIFPAUSED          0x74
#define ADDR_VGV3_WRITERAW               0x7c
#define ADDR_VGV3_WRITES16               0x79
#define ADDR_VGV3_WRITES32               0x7a
#define ADDR_VGV3_WRITES8                0x78

// FBC_BASE
typedef struct _REG_FBC_BASE {
    unsigned BASE                : 32;
} REG_FBC_BASE;

// FBC_DATA
typedef struct _REG_FBC_DATA {
    unsigned DATA                : 32;
} REG_FBC_DATA;

// FBC_HEIGHT
typedef struct _REG_FBC_HEIGHT {
    unsigned HEIGHT              : 11;
} REG_FBC_HEIGHT;

// FBC_START
typedef struct _REG_FBC_START {
    unsigned DUMMY               : 1;
} REG_FBC_START;

// FBC_STRIDE
typedef struct _REG_FBC_STRIDE {
    unsigned STRIDE              : 11;
} REG_FBC_STRIDE;

// FBC_WIDTH
typedef struct _REG_FBC_WIDTH {
    unsigned WIDTH               : 11;
} REG_FBC_WIDTH;

// VGC_CLOCKEN
typedef struct _REG_VGC_CLOCKEN {
    unsigned BCACHE              : 1;
    unsigned G2D_VGL3            : 1;
    unsigned VG_L1L2             : 1;
    unsigned RESERVED            : 3;
} REG_VGC_CLOCKEN;

// VGC_COMMANDSTREAM
typedef struct _REG_VGC_COMMANDSTREAM {
    unsigned DATA                : 32;
} REG_VGC_COMMANDSTREAM;

// VGC_FIFOFREE
typedef struct _REG_VGC_FIFOFREE {
    unsigned FREE                : 1;
} REG_VGC_FIFOFREE;

// VGC_IRQENABLE
typedef struct _REG_VGC_IRQENABLE {
    unsigned MH                  : 1;
    unsigned G2D                 : 1;
    unsigned FIFO                : 1;
    unsigned FBC                 : 1;
} REG_VGC_IRQENABLE;

// VGC_IRQSTATUS
typedef struct _REG_VGC_IRQSTATUS {
    unsigned MH                  : 1;
    unsigned G2D                 : 1;
    unsigned FIFO                : 1;
    unsigned FBC                 : 1;
} REG_VGC_IRQSTATUS;

// VGC_IRQ_ACTIVE_CNT
typedef struct _REG_VGC_IRQ_ACTIVE_CNT {
    unsigned MH                  : 8;
    unsigned G2D                 : 8;
    unsigned ERRORS              : 8;
    unsigned FBC                 : 8;
} REG_VGC_IRQ_ACTIVE_CNT;

// VGC_MMUCOMMANDSTREAM
typedef struct _REG_VGC_MMUCOMMANDSTREAM {
    unsigned DATA                : 32;
} REG_VGC_MMUCOMMANDSTREAM;

// VGC_REVISION
typedef struct _REG_VGC_REVISION {
    unsigned MINOR_REVISION      : 4;
    unsigned MAJOR_REVISION      : 4;
} REG_VGC_REVISION;

// VGC_SYSSTATUS
typedef struct _REG_VGC_SYSSTATUS {
    unsigned RESET               : 1;
} REG_VGC_SYSSTATUS;

// G2D_ALPHABLEND
typedef struct _REG_G2D_ALPHABLEND {
    unsigned ALPHA               : 8;
    unsigned OBS_ENABLE          : 1;
    unsigned CONSTANT            : 1;
    unsigned INVERT              : 1;
    unsigned OPTIMIZE            : 1;
    unsigned MODULATE            : 1;
    unsigned INVERTMASK          : 1;
    unsigned PREMULTIPLYDST      : 1;
    unsigned MASKTOALPHA         : 1;
} REG_G2D_ALPHABLEND;

// G2D_BACKGROUND
typedef struct _REG_G2D_BACKGROUND {
    unsigned COLOR               : 32;
} REG_G2D_BACKGROUND;

// G2D_BASE0
typedef struct _REG_G2D_BASE0 {
    unsigned ADDR                : 32;
} REG_G2D_BASE0;

// G2D_BASE1
typedef struct _REG_G2D_BASE1 {
    unsigned ADDR                : 32;
} REG_G2D_BASE1;

// G2D_BASE2
typedef struct _REG_G2D_BASE2 {
    unsigned ADDR                : 32;
} REG_G2D_BASE2;

// G2D_BASE3
typedef struct _REG_G2D_BASE3 {
    unsigned ADDR                : 32;
} REG_G2D_BASE3;

// G2D_BLENDERCFG
typedef struct _REG_G2D_BLENDERCFG {
    unsigned PASSES              : 3;
    unsigned ALPHAPASSES         : 2;
    unsigned ENABLE              : 1;
    unsigned OOALPHA             : 1;
    unsigned OBS_DIVALPHA        : 1;
    unsigned NOMASK              : 1;
} REG_G2D_BLENDERCFG;

// G2D_BLEND_A0
typedef struct _REG_G2D_BLEND_A0 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_A0;

// G2D_BLEND_A1
typedef struct _REG_G2D_BLEND_A1 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_A1;

// G2D_BLEND_A2
typedef struct _REG_G2D_BLEND_A2 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_A2;

// G2D_BLEND_A3
typedef struct _REG_G2D_BLEND_A3 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_A3;

// G2D_BLEND_C0
typedef struct _REG_G2D_BLEND_C0 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_C0;

// G2D_BLEND_C1
typedef struct _REG_G2D_BLEND_C1 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_C1;

// G2D_BLEND_C2
typedef struct _REG_G2D_BLEND_C2 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_C2;

// G2D_BLEND_C3
typedef struct _REG_G2D_BLEND_C3 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_C3;

// G2D_BLEND_C4
typedef struct _REG_G2D_BLEND_C4 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_C4;

// G2D_BLEND_C5
typedef struct _REG_G2D_BLEND_C5 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_C5;

// G2D_BLEND_C6
typedef struct _REG_G2D_BLEND_C6 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_C6;

// G2D_BLEND_C7
typedef struct _REG_G2D_BLEND_C7 {
    unsigned OPERATION           : 2;
    unsigned DST_A               : 2;
    unsigned DST_B               : 2;
    unsigned DST_C               : 2;
    unsigned AR_A                : 1;
    unsigned AR_B                : 1;
    unsigned AR_C                : 1;
    unsigned AR_D                : 1;
    unsigned INV_A               : 1;
    unsigned INV_B               : 1;
    unsigned INV_C               : 1;
    unsigned INV_D               : 1;
    unsigned SRC_A               : 3;
    unsigned SRC_B               : 3;
    unsigned SRC_C               : 3;
    unsigned SRC_D               : 3;
    unsigned CONST_A             : 1;
    unsigned CONST_B             : 1;
    unsigned CONST_C             : 1;
    unsigned CONST_D             : 1;
} REG_G2D_BLEND_C7;

// G2D_CFG0
typedef struct _REG_G2D_CFG0 {
    unsigned STRIDE              : 12;
    unsigned FORMAT              : 4;
    unsigned TILED               : 1;
    unsigned SRGB                : 1;
    unsigned SWAPWORDS           : 1;
    unsigned SWAPBYTES           : 1;
    unsigned SWAPALL             : 1;
    unsigned SWAPRB              : 1;
    unsigned SWAPBITS            : 1;
    unsigned STRIDESIGN          : 1;
} REG_G2D_CFG0;

// G2D_CFG1
typedef struct _REG_G2D_CFG1 {
    unsigned STRIDE              : 12;
    unsigned FORMAT              : 4;
    unsigned TILED               : 1;
    unsigned SRGB                : 1;
    unsigned SWAPWORDS           : 1;
    unsigned SWAPBYTES           : 1;
    unsigned SWAPALL             : 1;
    unsigned SWAPRB              : 1;
    unsigned SWAPBITS            : 1;
    unsigned STRIDESIGN          : 1;
} REG_G2D_CFG1;

// G2D_CFG2
typedef struct _REG_G2D_CFG2 {
    unsigned STRIDE              : 12;
    unsigned FORMAT              : 4;
    unsigned TILED               : 1;
    unsigned SRGB                : 1;
    unsigned SWAPWORDS           : 1;
    unsigned SWAPBYTES           : 1;
    unsigned SWAPALL             : 1;
    unsigned SWAPRB              : 1;
    unsigned SWAPBITS            : 1;
    unsigned STRIDESIGN          : 1;
} REG_G2D_CFG2;

// G2D_CFG3
typedef struct _REG_G2D_CFG3 {
    unsigned STRIDE              : 12;
    unsigned FORMAT              : 4;
    unsigned TILED               : 1;
    unsigned SRGB                : 1;
    unsigned SWAPWORDS           : 1;
    unsigned SWAPBYTES           : 1;
    unsigned SWAPALL             : 1;
    unsigned SWAPRB              : 1;
    unsigned SWAPBITS            : 1;
    unsigned STRIDESIGN          : 1;
} REG_G2D_CFG3;

// G2D_COLOR
typedef struct _REG_G2D_COLOR {
    unsigned ARGB                : 32;
} REG_G2D_COLOR;

// G2D_CONFIG
typedef struct _REG_G2D_CONFIG {
    unsigned DST                 : 1;
    unsigned SRC1                : 1;
    unsigned SRC2                : 1;
    unsigned SRC3                : 1;
    unsigned SRCCK               : 1;
    unsigned DSTCK               : 1;
    unsigned ROTATE              : 2;
    unsigned OBS_GAMMA           : 1;
    unsigned IGNORECKALPHA       : 1;
    unsigned DITHER              : 1;
    unsigned WRITESRGB           : 1;
    unsigned ARGBMASK            : 4;
    unsigned ALPHATEX            : 1;
    unsigned PALMLINES           : 1;
    unsigned NOLASTPIXEL         : 1;
    unsigned NOPROTECT           : 1;
} REG_G2D_CONFIG;

// G2D_CONST0
typedef struct _REG_G2D_CONST0 {
    unsigned ARGB                : 32;
} REG_G2D_CONST0;

// G2D_CONST1
typedef struct _REG_G2D_CONST1 {
    unsigned ARGB                : 32;
} REG_G2D_CONST1;

// G2D_CONST2
typedef struct _REG_G2D_CONST2 {
    unsigned ARGB                : 32;
} REG_G2D_CONST2;

// G2D_CONST3
typedef struct _REG_G2D_CONST3 {
    unsigned ARGB                : 32;
} REG_G2D_CONST3;

// G2D_CONST4
typedef struct _REG_G2D_CONST4 {
    unsigned ARGB                : 32;
} REG_G2D_CONST4;

// G2D_CONST5
typedef struct _REG_G2D_CONST5 {
    unsigned ARGB                : 32;
} REG_G2D_CONST5;

// G2D_CONST6
typedef struct _REG_G2D_CONST6 {
    unsigned ARGB                : 32;
} REG_G2D_CONST6;

// G2D_CONST7
typedef struct _REG_G2D_CONST7 {
    unsigned ARGB                : 32;
} REG_G2D_CONST7;

// G2D_FOREGROUND
typedef struct _REG_G2D_FOREGROUND {
    unsigned COLOR               : 32;
} REG_G2D_FOREGROUND;

// G2D_GRADIENT
typedef struct _REG_G2D_GRADIENT {
    unsigned INSTRUCTIONS        : 3;
    unsigned INSTRUCTIONS2       : 3;
    unsigned ENABLE              : 1;
    unsigned ENABLE2             : 1;
    unsigned SEL                 : 1;
} REG_G2D_GRADIENT;

// G2D_IDLE
typedef struct _REG_G2D_IDLE {
    unsigned IRQ                 : 1;
    unsigned BCFLUSH             : 1;
    unsigned V3                  : 1;
} REG_G2D_IDLE;

// G2D_INPUT
typedef struct _REG_G2D_INPUT {
    unsigned COLOR               : 1;
    unsigned SCOORD1             : 1;
    unsigned SCOORD2             : 1;
    unsigned COPYCOORD           : 1;
    unsigned VGMODE              : 1;
    unsigned LINEMODE            : 1;
} REG_G2D_INPUT;

// G2D_MASK
typedef struct _REG_G2D_MASK {
    unsigned YMASK               : 12;
    unsigned XMASK               : 12;
} REG_G2D_MASK;

// G2D_ROP
typedef struct _REG_G2D_ROP {
    unsigned ROP                 : 16;
} REG_G2D_ROP;

// G2D_SCISSORX
typedef struct _REG_G2D_SCISSORX {
    unsigned LEFT                : 11;
    unsigned RIGHT               : 11;
} REG_G2D_SCISSORX;

// G2D_SCISSORY
typedef struct _REG_G2D_SCISSORY {
    unsigned TOP                 : 11;
    unsigned BOTTOM              : 11;
} REG_G2D_SCISSORY;

// G2D_SXY
typedef struct _REG_G2D_SXY {
    unsigned Y                   : 11;
    unsigned PAD                 : 5;
    unsigned X                   : 11;
} REG_G2D_SXY;

// G2D_SXY2
typedef struct _REG_G2D_SXY2 {
    unsigned Y                   : 11;
    unsigned PAD                 : 5;
    unsigned X                   : 11;
} REG_G2D_SXY2;

// G2D_VGSPAN
typedef struct _REG_G2D_VGSPAN {
    int WIDTH               : 12;
    unsigned PAD                 : 4;
    unsigned COVERAGE            : 4;
} REG_G2D_VGSPAN;

// G2D_WIDTHHEIGHT
typedef struct _REG_G2D_WIDTHHEIGHT {
    int HEIGHT              : 12;
    unsigned PAD                 : 4;
    int WIDTH               : 12;
} REG_G2D_WIDTHHEIGHT;

// G2D_XY
typedef struct _REG_G2D_XY {
    int Y                   : 12;
    unsigned PAD                 : 4;
    int X                   : 12;
} REG_G2D_XY;

// GRADW_BORDERCOLOR
typedef struct _REG_GRADW_BORDERCOLOR {
    unsigned COLOR               : 32;
} REG_GRADW_BORDERCOLOR;

// GRADW_CONST0
typedef struct _REG_GRADW_CONST0 {
    unsigned VALUE               : 16;
} REG_GRADW_CONST0;

// GRADW_CONST1
typedef struct _REG_GRADW_CONST1 {
    unsigned VALUE               : 16;
} REG_GRADW_CONST1;

// GRADW_CONST2
typedef struct _REG_GRADW_CONST2 {
    unsigned VALUE               : 16;
} REG_GRADW_CONST2;

// GRADW_CONST3
typedef struct _REG_GRADW_CONST3 {
    unsigned VALUE               : 16;
} REG_GRADW_CONST3;

// GRADW_CONST4
typedef struct _REG_GRADW_CONST4 {
    unsigned VALUE               : 16;
} REG_GRADW_CONST4;

// GRADW_CONST5
typedef struct _REG_GRADW_CONST5 {
    unsigned VALUE               : 16;
} REG_GRADW_CONST5;

// GRADW_CONST6
typedef struct _REG_GRADW_CONST6 {
    unsigned VALUE               : 16;
} REG_GRADW_CONST6;

// GRADW_CONST7
typedef struct _REG_GRADW_CONST7 {
    unsigned VALUE               : 16;
} REG_GRADW_CONST7;

// GRADW_CONST8
typedef struct _REG_GRADW_CONST8 {
    unsigned VALUE               : 16;
} REG_GRADW_CONST8;

// GRADW_CONST9
typedef struct _REG_GRADW_CONST9 {
    unsigned VALUE               : 16;
} REG_GRADW_CONST9;

// GRADW_CONSTA
typedef struct _REG_GRADW_CONSTA {
    unsigned VALUE               : 16;
} REG_GRADW_CONSTA;

// GRADW_CONSTB
typedef struct _REG_GRADW_CONSTB {
    unsigned VALUE               : 16;
} REG_GRADW_CONSTB;

// GRADW_INST0
typedef struct _REG_GRADW_INST0 {
    unsigned SRC_E               : 5;
    unsigned SRC_D               : 5;
    unsigned SRC_C               : 5;
    unsigned SRC_B               : 5;
    unsigned SRC_A               : 5;
    unsigned DST                 : 4;
    unsigned OPCODE              : 2;
} REG_GRADW_INST0;

// GRADW_INST1
typedef struct _REG_GRADW_INST1 {
    unsigned SRC_E               : 5;
    unsigned SRC_D               : 5;
    unsigned SRC_C               : 5;
    unsigned SRC_B               : 5;
    unsigned SRC_A               : 5;
    unsigned DST                 : 4;
    unsigned OPCODE              : 2;
} REG_GRADW_INST1;

// GRADW_INST2
typedef struct _REG_GRADW_INST2 {
    unsigned SRC_E               : 5;
    unsigned SRC_D               : 5;
    unsigned SRC_C               : 5;
    unsigned SRC_B               : 5;
    unsigned SRC_A               : 5;
    unsigned DST                 : 4;
    unsigned OPCODE              : 2;
} REG_GRADW_INST2;

// GRADW_INST3
typedef struct _REG_GRADW_INST3 {
    unsigned SRC_E               : 5;
    unsigned SRC_D               : 5;
    unsigned SRC_C               : 5;
    unsigned SRC_B               : 5;
    unsigned SRC_A               : 5;
    unsigned DST                 : 4;
    unsigned OPCODE              : 2;
} REG_GRADW_INST3;

// GRADW_INST4
typedef struct _REG_GRADW_INST4 {
    unsigned SRC_E               : 5;
    unsigned SRC_D               : 5;
    unsigned SRC_C               : 5;
    unsigned SRC_B               : 5;
    unsigned SRC_A               : 5;
    unsigned DST                 : 4;
    unsigned OPCODE              : 2;
} REG_GRADW_INST4;

// GRADW_INST5
typedef struct _REG_GRADW_INST5 {
    unsigned SRC_E               : 5;
    unsigned SRC_D               : 5;
    unsigned SRC_C               : 5;
    unsigned SRC_B               : 5;
    unsigned SRC_A               : 5;
    unsigned DST                 : 4;
    unsigned OPCODE              : 2;
} REG_GRADW_INST5;

// GRADW_INST6
typedef struct _REG_GRADW_INST6 {
    unsigned SRC_E               : 5;
    unsigned SRC_D               : 5;
    unsigned SRC_C               : 5;
    unsigned SRC_B               : 5;
    unsigned SRC_A               : 5;
    unsigned DST                 : 4;
    unsigned OPCODE              : 2;
} REG_GRADW_INST6;

// GRADW_INST7
typedef struct _REG_GRADW_INST7 {
    unsigned SRC_E               : 5;
    unsigned SRC_D               : 5;
    unsigned SRC_C               : 5;
    unsigned SRC_B               : 5;
    unsigned SRC_A               : 5;
    unsigned DST                 : 4;
    unsigned OPCODE              : 2;
} REG_GRADW_INST7;

// GRADW_TEXBASE
typedef struct _REG_GRADW_TEXBASE {
    unsigned ADDR                : 32;
} REG_GRADW_TEXBASE;

// GRADW_TEXCFG
typedef struct _REG_GRADW_TEXCFG {
    unsigned STRIDE              : 12;
    unsigned FORMAT              : 4;
    unsigned TILED               : 1;
    unsigned WRAPU               : 2;
    unsigned WRAPV               : 2;
    unsigned BILIN               : 1;
    unsigned SRGB                : 1;
    unsigned PREMULTIPLY         : 1;
    unsigned SWAPWORDS           : 1;
    unsigned SWAPBYTES           : 1;
    unsigned SWAPALL             : 1;
    unsigned SWAPRB              : 1;
    unsigned TEX2D               : 1;
    unsigned SWAPBITS            : 1;
} REG_GRADW_TEXCFG;

// GRADW_TEXSIZE
typedef struct _REG_GRADW_TEXSIZE {
    unsigned WIDTH               : 11;
    unsigned HEIGHT              : 11;
} REG_GRADW_TEXSIZE;

// MH_ARBITER_CONFIG
typedef struct _REG_MH_ARBITER_CONFIG {
    unsigned SAME_PAGE_LIMIT     : 6;
    unsigned SAME_PAGE_GRANULARITY : 1;
    unsigned L1_ARB_ENABLE       : 1;
    unsigned L1_ARB_HOLD_ENABLE  : 1;
    unsigned L2_ARB_CONTROL      : 1;
    unsigned PAGE_SIZE           : 3;
    unsigned TC_REORDER_ENABLE   : 1;
    unsigned TC_ARB_HOLD_ENABLE  : 1;
    unsigned IN_FLIGHT_LIMIT_ENABLE : 1;
    unsigned IN_FLIGHT_LIMIT     : 6;
    unsigned CP_CLNT_ENABLE      : 1;
    unsigned VGT_CLNT_ENABLE     : 1;
    unsigned TC_CLNT_ENABLE      : 1;
    unsigned RB_CLNT_ENABLE      : 1;
    unsigned PA_CLNT_ENABLE      : 1;
} REG_MH_ARBITER_CONFIG;

// MH_AXI_ERROR
typedef struct _REG_MH_AXI_ERROR {
    unsigned AXI_READ_ID         : 3;
    unsigned AXI_READ_ERROR      : 1;
    unsigned AXI_WRITE_ID        : 3;
    unsigned AXI_WRITE_ERROR     : 1;
} REG_MH_AXI_ERROR;

// MH_AXI_HALT_CONTROL
typedef struct _REG_MH_AXI_HALT_CONTROL {
    unsigned AXI_HALT            : 1;
} REG_MH_AXI_HALT_CONTROL;

// MH_CLNT_AXI_ID_REUSE
typedef struct _REG_MH_CLNT_AXI_ID_REUSE {
    unsigned CPW_ID              : 3;
    unsigned PAD                 : 1;
    unsigned RBW_ID              : 3;
    unsigned PAD2                : 1;
    unsigned MMUR_ID             : 3;
    unsigned PAD3                : 1;
    unsigned PAW_ID              : 3;
} REG_MH_CLNT_AXI_ID_REUSE;

// MH_DEBUG_CTRL
typedef struct _REG_MH_DEBUG_CTRL {
    unsigned INDEX               : 6;
} REG_MH_DEBUG_CTRL;

// MH_DEBUG_DATA
typedef struct _REG_MH_DEBUG_DATA {
    unsigned DATA                : 32;
} REG_MH_DEBUG_DATA;

// MH_INTERRUPT_CLEAR
typedef struct _REG_MH_INTERRUPT_CLEAR {
    unsigned AXI_READ_ERROR      : 1;
    unsigned AXI_WRITE_ERROR     : 1;
    unsigned MMU_PAGE_FAULT      : 1;
} REG_MH_INTERRUPT_CLEAR;

// MH_INTERRUPT_MASK
typedef struct _REG_MH_INTERRUPT_MASK {
    unsigned AXI_READ_ERROR      : 1;
    unsigned AXI_WRITE_ERROR     : 1;
    unsigned MMU_PAGE_FAULT      : 1;
} REG_MH_INTERRUPT_MASK;

// MH_INTERRUPT_STATUS
typedef struct _REG_MH_INTERRUPT_STATUS {
    unsigned AXI_READ_ERROR      : 1;
    unsigned AXI_WRITE_ERROR     : 1;
    unsigned MMU_PAGE_FAULT      : 1;
} REG_MH_INTERRUPT_STATUS;

// MH_MMU_CONFIG
typedef struct _REG_MH_MMU_CONFIG {
    unsigned MMU_ENABLE          : 1;
    unsigned SPLIT_MODE_ENABLE   : 1;
    unsigned PAD                 : 2;
    unsigned RB_W_CLNT_BEHAVIOR  : 2;
    unsigned CP_W_CLNT_BEHAVIOR  : 2;
    unsigned CP_R0_CLNT_BEHAVIOR : 2;
    unsigned CP_R1_CLNT_BEHAVIOR : 2;
    unsigned CP_R2_CLNT_BEHAVIOR : 2;
    unsigned CP_R3_CLNT_BEHAVIOR : 2;
    unsigned CP_R4_CLNT_BEHAVIOR : 2;
    unsigned VGT_R0_CLNT_BEHAVIOR : 2;
    unsigned VGT_R1_CLNT_BEHAVIOR : 2;
    unsigned TC_R_CLNT_BEHAVIOR  : 2;
    unsigned PA_W_CLNT_BEHAVIOR  : 2;
} REG_MH_MMU_CONFIG;

// MH_MMU_INVALIDATE
typedef struct _REG_MH_MMU_INVALIDATE {
    unsigned INVALIDATE_ALL      : 1;
    unsigned INVALIDATE_TC       : 1;
} REG_MH_MMU_INVALIDATE;

// MH_MMU_MPU_BASE
typedef struct _REG_MH_MMU_MPU_BASE {
    unsigned ZERO                : 12;
    unsigned MPU_BASE            : 20;
} REG_MH_MMU_MPU_BASE;

// MH_MMU_MPU_END
typedef struct _REG_MH_MMU_MPU_END {
    unsigned ZERO                : 12;
    unsigned MPU_END             : 20;
} REG_MH_MMU_MPU_END;

// MH_MMU_PAGE_FAULT
typedef struct _REG_MH_MMU_PAGE_FAULT {
    unsigned PAGE_FAULT          : 1;
    unsigned OP_TYPE             : 1;
    unsigned CLNT_BEHAVIOR       : 2;
    unsigned AXI_ID              : 3;
    unsigned PAD                 : 1;
    unsigned MPU_ADDRESS_OUT_OF_RANGE : 1;
    unsigned ADDRESS_OUT_OF_RANGE : 1;
    unsigned READ_PROTECTION_ERROR : 1;
    unsigned WRITE_PROTECTION_ERROR : 1;
    unsigned REQ_VA              : 20;
} REG_MH_MMU_PAGE_FAULT;

// MH_MMU_PT_BASE
typedef struct _REG_MH_MMU_PT_BASE {
    unsigned ZERO                : 12;
    unsigned PT_BASE             : 20;
} REG_MH_MMU_PT_BASE;

// MH_MMU_TRAN_ERROR
typedef struct _REG_MH_MMU_TRAN_ERROR {
    unsigned ZERO                : 5;
    unsigned TRAN_ERROR          : 27;
} REG_MH_MMU_TRAN_ERROR;

// MH_MMU_VA_RANGE
typedef struct _REG_MH_MMU_VA_RANGE {
    unsigned NUM_64KB_REGIONS    : 12;
    unsigned VA_BASE             : 20;
} REG_MH_MMU_VA_RANGE;

// MH_PERFCOUNTER0_CONFIG
typedef struct _REG_MH_PERFCOUNTER0_CONFIG {
    unsigned N_VALUE             : 8;
} REG_MH_PERFCOUNTER0_CONFIG;

// MH_PERFCOUNTER0_HI
typedef struct _REG_MH_PERFCOUNTER0_HI {
    unsigned PERF_COUNTER_HI     : 16;
} REG_MH_PERFCOUNTER0_HI;

// MH_PERFCOUNTER0_LOW
typedef struct _REG_MH_PERFCOUNTER0_LOW {
    unsigned PERF_COUNTER_LOW    : 32;
} REG_MH_PERFCOUNTER0_LOW;

// MH_PERFCOUNTER0_SELECT
typedef struct _REG_MH_PERFCOUNTER0_SELECT {
    unsigned PERF_SEL            : 8;
} REG_MH_PERFCOUNTER0_SELECT;

// MH_PERFCOUNTER1_CONFIG
typedef struct _REG_MH_PERFCOUNTER1_CONFIG {
    unsigned N_VALUE             : 8;
} REG_MH_PERFCOUNTER1_CONFIG;

// MH_PERFCOUNTER1_HI
typedef struct _REG_MH_PERFCOUNTER1_HI {
    unsigned PERF_COUNTER_HI     : 16;
} REG_MH_PERFCOUNTER1_HI;

// MH_PERFCOUNTER1_LOW
typedef struct _REG_MH_PERFCOUNTER1_LOW {
    unsigned PERF_COUNTER_LOW    : 32;
} REG_MH_PERFCOUNTER1_LOW;

// MH_PERFCOUNTER1_SELECT
typedef struct _REG_MH_PERFCOUNTER1_SELECT {
    unsigned PERF_SEL            : 8;
} REG_MH_PERFCOUNTER1_SELECT;

// MMU_READ_ADDR
typedef struct _REG_MMU_READ_ADDR {
    unsigned ADDR                : 15;
} REG_MMU_READ_ADDR;

// MMU_READ_DATA
typedef struct _REG_MMU_READ_DATA {
    unsigned DATA                : 32;
} REG_MMU_READ_DATA;

// VGV1_CBASE1
typedef struct _REG_VGV1_CBASE1 {
    unsigned ADDR                : 32;
} REG_VGV1_CBASE1;

// VGV1_CFG1
typedef struct _REG_VGV1_CFG1 {
    unsigned WINDRULE            : 1;
} REG_VGV1_CFG1;

// VGV1_CFG2
typedef struct _REG_VGV1_CFG2 {
    unsigned AAMODE              : 2;
} REG_VGV1_CFG2;

// VGV1_DIRTYBASE
typedef struct _REG_VGV1_DIRTYBASE {
    unsigned ADDR                : 32;
} REG_VGV1_DIRTYBASE;

// VGV1_FILL
typedef struct _REG_VGV1_FILL {
    unsigned INHERIT             : 1;
} REG_VGV1_FILL;

// VGV1_SCISSORX
typedef struct _REG_VGV1_SCISSORX {
    unsigned LEFT                : 11;
    unsigned PAD                 : 5;
    unsigned RIGHT               : 11;
} REG_VGV1_SCISSORX;

// VGV1_SCISSORY
typedef struct _REG_VGV1_SCISSORY {
    unsigned TOP                 : 11;
    unsigned PAD                 : 5;
    unsigned BOTTOM              : 11;
} REG_VGV1_SCISSORY;

// VGV1_TILEOFS
typedef struct _REG_VGV1_TILEOFS {
    unsigned X                   : 12;
    unsigned Y                   : 12;
    unsigned LEFTMOST            : 1;
} REG_VGV1_TILEOFS;

// VGV1_UBASE2
typedef struct _REG_VGV1_UBASE2 {
    unsigned ADDR                : 32;
} REG_VGV1_UBASE2;

// VGV1_VTX0
typedef struct _REG_VGV1_VTX0 {
    int X                   : 16;
    int Y                   : 16;
} REG_VGV1_VTX0;

// VGV1_VTX1
typedef struct _REG_VGV1_VTX1 {
    int X                   : 16;
    int Y                   : 16;
} REG_VGV1_VTX1;

// VGV2_ACCURACY
typedef struct _REG_VGV2_ACCURACY {
    unsigned F                   : 24;
} REG_VGV2_ACCURACY;

// VGV2_ACTION
typedef struct _REG_VGV2_ACTION {
    unsigned ACTION              : 4;
} REG_VGV2_ACTION;

// VGV2_ARCCOS
typedef struct _REG_VGV2_ARCCOS {
    unsigned F                   : 24;
} REG_VGV2_ARCCOS;

// VGV2_ARCSIN
typedef struct _REG_VGV2_ARCSIN {
    unsigned F                   : 24;
} REG_VGV2_ARCSIN;

// VGV2_ARCTAN
typedef struct _REG_VGV2_ARCTAN {
    unsigned F                   : 24;
} REG_VGV2_ARCTAN;

// VGV2_BBOXMAXX
typedef struct _REG_VGV2_BBOXMAXX {
    unsigned F                   : 24;
} REG_VGV2_BBOXMAXX;

// VGV2_BBOXMAXY
typedef struct _REG_VGV2_BBOXMAXY {
    unsigned F                   : 24;
} REG_VGV2_BBOXMAXY;

// VGV2_BBOXMINX
typedef struct _REG_VGV2_BBOXMINX {
    unsigned F                   : 24;
} REG_VGV2_BBOXMINX;

// VGV2_BBOXMINY
typedef struct _REG_VGV2_BBOXMINY {
    unsigned F                   : 24;
} REG_VGV2_BBOXMINY;

// VGV2_BIAS
typedef struct _REG_VGV2_BIAS {
    unsigned F                   : 24;
} REG_VGV2_BIAS;

// VGV2_C1X
typedef struct _REG_VGV2_C1X {
    unsigned F                   : 24;
} REG_VGV2_C1X;

// VGV2_C1XREL
typedef struct _REG_VGV2_C1XREL {
    unsigned F                   : 24;
} REG_VGV2_C1XREL;

// VGV2_C1Y
typedef struct _REG_VGV2_C1Y {
    unsigned F                   : 24;
} REG_VGV2_C1Y;

// VGV2_C1YREL
typedef struct _REG_VGV2_C1YREL {
    unsigned F                   : 24;
} REG_VGV2_C1YREL;

// VGV2_C2X
typedef struct _REG_VGV2_C2X {
    unsigned F                   : 24;
} REG_VGV2_C2X;

// VGV2_C2XREL
typedef struct _REG_VGV2_C2XREL {
    unsigned F                   : 24;
} REG_VGV2_C2XREL;

// VGV2_C2Y
typedef struct _REG_VGV2_C2Y {
    unsigned F                   : 24;
} REG_VGV2_C2Y;

// VGV2_C2YREL
typedef struct _REG_VGV2_C2YREL {
    unsigned F                   : 24;
} REG_VGV2_C2YREL;

// VGV2_C3X
typedef struct _REG_VGV2_C3X {
    unsigned F                   : 24;
} REG_VGV2_C3X;

// VGV2_C3XREL
typedef struct _REG_VGV2_C3XREL {
    unsigned F                   : 24;
} REG_VGV2_C3XREL;

// VGV2_C3Y
typedef struct _REG_VGV2_C3Y {
    unsigned F                   : 24;
} REG_VGV2_C3Y;

// VGV2_C3YREL
typedef struct _REG_VGV2_C3YREL {
    unsigned F                   : 24;
} REG_VGV2_C3YREL;

// VGV2_C4X
typedef struct _REG_VGV2_C4X {
    unsigned F                   : 24;
} REG_VGV2_C4X;

// VGV2_C4XREL
typedef struct _REG_VGV2_C4XREL {
    unsigned F                   : 24;
} REG_VGV2_C4XREL;

// VGV2_C4Y
typedef struct _REG_VGV2_C4Y {
    unsigned F                   : 24;
} REG_VGV2_C4Y;

// VGV2_C4YREL
typedef struct _REG_VGV2_C4YREL {
    unsigned F                   : 24;
} REG_VGV2_C4YREL;

// VGV2_CLIP
typedef struct _REG_VGV2_CLIP {
    unsigned F                   : 24;
} REG_VGV2_CLIP;

// VGV2_FIRST
typedef struct _REG_VGV2_FIRST {
    unsigned DUMMY               : 1;
} REG_VGV2_FIRST;

// VGV2_LAST
typedef struct _REG_VGV2_LAST {
    unsigned DUMMY               : 1;
} REG_VGV2_LAST;

// VGV2_MITER
typedef struct _REG_VGV2_MITER {
    unsigned F                   : 24;
} REG_VGV2_MITER;

// VGV2_MODE
typedef struct _REG_VGV2_MODE {
    unsigned MAXSPLIT            : 4;
    unsigned CAP                 : 2;
    unsigned JOIN                : 2;
    unsigned STROKE              : 1;
    unsigned STROKESPLIT         : 1;
    unsigned FULLSPLIT           : 1;
    unsigned NODOTS              : 1;
    unsigned OPENFILL            : 1;
    unsigned DROPLEFT            : 1;
    unsigned DROPOTHER           : 1;
    unsigned SYMMETRICJOINS      : 1;
    unsigned SIMPLESTROKE        : 1;
    unsigned SIMPLECLIP          : 1;
    int EXPONENTADD         : 6;
} REG_VGV2_MODE;

// VGV2_RADIUS
typedef struct _REG_VGV2_RADIUS {
    unsigned F                   : 24;
} REG_VGV2_RADIUS;

// VGV2_SCALE
typedef struct _REG_VGV2_SCALE {
    unsigned F                   : 24;
} REG_VGV2_SCALE;

// VGV2_THINRADIUS
typedef struct _REG_VGV2_THINRADIUS {
    unsigned F                   : 24;
} REG_VGV2_THINRADIUS;

// VGV2_XFSTXX
typedef struct _REG_VGV2_XFSTXX {
    unsigned F                   : 24;
} REG_VGV2_XFSTXX;

// VGV2_XFSTXY
typedef struct _REG_VGV2_XFSTXY {
    unsigned F                   : 24;
} REG_VGV2_XFSTXY;

// VGV2_XFSTYX
typedef struct _REG_VGV2_XFSTYX {
    unsigned F                   : 24;
} REG_VGV2_XFSTYX;

// VGV2_XFSTYY
typedef struct _REG_VGV2_XFSTYY {
    unsigned F                   : 24;
} REG_VGV2_XFSTYY;

// VGV2_XFXA
typedef struct _REG_VGV2_XFXA {
    unsigned F                   : 24;
} REG_VGV2_XFXA;

// VGV2_XFXX
typedef struct _REG_VGV2_XFXX {
    unsigned F                   : 24;
} REG_VGV2_XFXX;

// VGV2_XFXY
typedef struct _REG_VGV2_XFXY {
    unsigned F                   : 24;
} REG_VGV2_XFXY;

// VGV2_XFYA
typedef struct _REG_VGV2_XFYA {
    unsigned F                   : 24;
} REG_VGV2_XFYA;

// VGV2_XFYX
typedef struct _REG_VGV2_XFYX {
    unsigned F                   : 24;
} REG_VGV2_XFYX;

// VGV2_XFYY
typedef struct _REG_VGV2_XFYY {
    unsigned F                   : 24;
} REG_VGV2_XFYY;

// VGV3_CONTROL
typedef struct _REG_VGV3_CONTROL {
    unsigned MARKADD             : 12;
    unsigned DMIWAITCHMASK       : 4;
    unsigned PAUSE               : 1;
    unsigned ABORT               : 1;
    unsigned WRITE               : 1;
    unsigned BCFLUSH             : 1;
    unsigned V0SYNC              : 1;
    unsigned DMIWAITBUF          : 3;
} REG_VGV3_CONTROL;

// VGV3_FIRST
typedef struct _REG_VGV3_FIRST {
    unsigned DUMMY               : 1;
} REG_VGV3_FIRST;

// VGV3_LAST
typedef struct _REG_VGV3_LAST {
    unsigned DUMMY               : 1;
} REG_VGV3_LAST;

// VGV3_MODE
typedef struct _REG_VGV3_MODE {
    unsigned FLIPENDIAN          : 1;
    unsigned UNUSED              : 1;
    unsigned WRITEFLUSH          : 1;
    unsigned DMIPAUSETYPE        : 1;
    unsigned DMIRESET            : 1;
} REG_VGV3_MODE;

// VGV3_NEXTADDR
typedef struct _REG_VGV3_NEXTADDR {
    unsigned CALLADDR            : 32;
} REG_VGV3_NEXTADDR;

// VGV3_NEXTCMD
typedef struct _REG_VGV3_NEXTCMD {
    unsigned COUNT               : 12;
    unsigned NEXTCMD             : 3;
    unsigned MARK                : 1;
    unsigned CALLCOUNT           : 12;
} REG_VGV3_NEXTCMD;

// VGV3_VGBYPASS
typedef struct _REG_VGV3_VGBYPASS {
    unsigned BYPASS              : 1;
} REG_VGV3_VGBYPASS;

// VGV3_WRITE
typedef struct _REG_VGV3_WRITE {
    unsigned VALUE               : 32;
} REG_VGV3_WRITE;

// VGV3_WRITEADDR
typedef struct _REG_VGV3_WRITEADDR {
    unsigned ADDR                : 32;
} REG_VGV3_WRITEADDR;

// VGV3_WRITEDMI
typedef struct _REG_VGV3_WRITEDMI {
    unsigned CHANMASK            : 4;
    unsigned BUFFER              : 3;
} REG_VGV3_WRITEDMI;

// VGV3_WRITEF32
typedef struct _REG_VGV3_WRITEF32 {
    unsigned ADDR                : 8;
    unsigned COUNT               : 8;
    unsigned LOOP                : 4;
    unsigned ACTION              : 4;
    unsigned FORMAT              : 3;
} REG_VGV3_WRITEF32;

// VGV3_WRITEIFPAUSED
typedef struct _REG_VGV3_WRITEIFPAUSED {
    unsigned VALUE               : 32;
} REG_VGV3_WRITEIFPAUSED;

// VGV3_WRITERAW
typedef struct _REG_VGV3_WRITERAW {
    unsigned ADDR                : 8;
    unsigned COUNT               : 8;
    unsigned LOOP                : 4;
    unsigned ACTION              : 4;
    unsigned FORMAT              : 3;
} REG_VGV3_WRITERAW;

// VGV3_WRITES16
typedef struct _REG_VGV3_WRITES16 {
    unsigned ADDR                : 8;
    unsigned COUNT               : 8;
    unsigned LOOP                : 4;
    unsigned ACTION              : 4;
    unsigned FORMAT              : 3;
} REG_VGV3_WRITES16;

// VGV3_WRITES32
typedef struct _REG_VGV3_WRITES32 {
    unsigned ADDR                : 8;
    unsigned COUNT               : 8;
    unsigned LOOP                : 4;
    unsigned ACTION              : 4;
    unsigned FORMAT              : 3;
} REG_VGV3_WRITES32;

// VGV3_WRITES8
typedef struct _REG_VGV3_WRITES8 {
    unsigned ADDR                : 8;
    unsigned COUNT               : 8;
    unsigned LOOP                : 4;
    unsigned ACTION              : 4;
    unsigned FORMAT              : 3;
} REG_VGV3_WRITES8;

// Register address, down shift, AND mask
#define FBC_BASE_BASE_FADDR ADDR_FBC_BASE
#define FBC_BASE_BASE_FSHIFT 0
#define FBC_BASE_BASE_FMASK 0xffffffff
#define FBC_DATA_DATA_FADDR ADDR_FBC_DATA
#define FBC_DATA_DATA_FSHIFT 0
#define FBC_DATA_DATA_FMASK 0xffffffff
#define FBC_HEIGHT_HEIGHT_FADDR ADDR_FBC_HEIGHT
#define FBC_HEIGHT_HEIGHT_FSHIFT 0
#define FBC_HEIGHT_HEIGHT_FMASK 0x7ff
#define FBC_START_DUMMY_FADDR ADDR_FBC_START
#define FBC_START_DUMMY_FSHIFT 0
#define FBC_START_DUMMY_FMASK 0x1
#define FBC_STRIDE_STRIDE_FADDR ADDR_FBC_STRIDE
#define FBC_STRIDE_STRIDE_FSHIFT 0
#define FBC_STRIDE_STRIDE_FMASK 0x7ff
#define FBC_WIDTH_WIDTH_FADDR ADDR_FBC_WIDTH
#define FBC_WIDTH_WIDTH_FSHIFT 0
#define FBC_WIDTH_WIDTH_FMASK 0x7ff
#define VGC_CLOCKEN_BCACHE_FADDR ADDR_VGC_CLOCKEN
#define VGC_CLOCKEN_BCACHE_FSHIFT 0
#define VGC_CLOCKEN_BCACHE_FMASK 0x1
#define VGC_CLOCKEN_G2D_VGL3_FADDR ADDR_VGC_CLOCKEN
#define VGC_CLOCKEN_G2D_VGL3_FSHIFT 1
#define VGC_CLOCKEN_G2D_VGL3_FMASK 0x1
#define VGC_CLOCKEN_VG_L1L2_FADDR ADDR_VGC_CLOCKEN
#define VGC_CLOCKEN_VG_L1L2_FSHIFT 2
#define VGC_CLOCKEN_VG_L1L2_FMASK 0x1
#define VGC_CLOCKEN_RESERVED_FADDR ADDR_VGC_CLOCKEN
#define VGC_CLOCKEN_RESERVED_FSHIFT 3
#define VGC_CLOCKEN_RESERVED_FMASK 0x7
#define VGC_COMMANDSTREAM_DATA_FADDR ADDR_VGC_COMMANDSTREAM
#define VGC_COMMANDSTREAM_DATA_FSHIFT 0
#define VGC_COMMANDSTREAM_DATA_FMASK 0xffffffff
#define VGC_FIFOFREE_FREE_FADDR ADDR_VGC_FIFOFREE
#define VGC_FIFOFREE_FREE_FSHIFT 0
#define VGC_FIFOFREE_FREE_FMASK 0x1
#define VGC_IRQENABLE_MH_FADDR ADDR_VGC_IRQENABLE
#define VGC_IRQENABLE_MH_FSHIFT 0
#define VGC_IRQENABLE_MH_FMASK 0x1
#define VGC_IRQENABLE_G2D_FADDR ADDR_VGC_IRQENABLE
#define VGC_IRQENABLE_G2D_FSHIFT 1
#define VGC_IRQENABLE_G2D_FMASK 0x1
#define VGC_IRQENABLE_FIFO_FADDR ADDR_VGC_IRQENABLE
#define VGC_IRQENABLE_FIFO_FSHIFT 2
#define VGC_IRQENABLE_FIFO_FMASK 0x1
#define VGC_IRQENABLE_FBC_FADDR ADDR_VGC_IRQENABLE
#define VGC_IRQENABLE_FBC_FSHIFT 3
#define VGC_IRQENABLE_FBC_FMASK 0x1
#define VGC_IRQSTATUS_MH_FADDR ADDR_VGC_IRQSTATUS
#define VGC_IRQSTATUS_MH_FSHIFT 0
#define VGC_IRQSTATUS_MH_FMASK 0x1
#define VGC_IRQSTATUS_G2D_FADDR ADDR_VGC_IRQSTATUS
#define VGC_IRQSTATUS_G2D_FSHIFT 1
#define VGC_IRQSTATUS_G2D_FMASK 0x1
#define VGC_IRQSTATUS_FIFO_FADDR ADDR_VGC_IRQSTATUS
#define VGC_IRQSTATUS_FIFO_FSHIFT 2
#define VGC_IRQSTATUS_FIFO_FMASK 0x1
#define VGC_IRQSTATUS_FBC_FADDR ADDR_VGC_IRQSTATUS
#define VGC_IRQSTATUS_FBC_FSHIFT 3
#define VGC_IRQSTATUS_FBC_FMASK 0x1
#define VGC_IRQ_ACTIVE_CNT_MH_FADDR ADDR_VGC_IRQ_ACTIVE_CNT
#define VGC_IRQ_ACTIVE_CNT_MH_FSHIFT 0
#define VGC_IRQ_ACTIVE_CNT_MH_FMASK 0xff
#define VGC_IRQ_ACTIVE_CNT_G2D_FADDR ADDR_VGC_IRQ_ACTIVE_CNT
#define VGC_IRQ_ACTIVE_CNT_G2D_FSHIFT 8
#define VGC_IRQ_ACTIVE_CNT_G2D_FMASK 0xff
#define VGC_IRQ_ACTIVE_CNT_ERRORS_FADDR ADDR_VGC_IRQ_ACTIVE_CNT
#define VGC_IRQ_ACTIVE_CNT_ERRORS_FSHIFT 16
#define VGC_IRQ_ACTIVE_CNT_ERRORS_FMASK 0xff
#define VGC_IRQ_ACTIVE_CNT_FBC_FADDR ADDR_VGC_IRQ_ACTIVE_CNT
#define VGC_IRQ_ACTIVE_CNT_FBC_FSHIFT 24
#define VGC_IRQ_ACTIVE_CNT_FBC_FMASK 0xff
#define VGC_MMUCOMMANDSTREAM_DATA_FADDR ADDR_VGC_MMUCOMMANDSTREAM
#define VGC_MMUCOMMANDSTREAM_DATA_FSHIFT 0
#define VGC_MMUCOMMANDSTREAM_DATA_FMASK 0xffffffff
#define VGC_REVISION_MINOR_REVISION_FADDR ADDR_VGC_REVISION
#define VGC_REVISION_MINOR_REVISION_FSHIFT 0
#define VGC_REVISION_MINOR_REVISION_FMASK 0xf
#define VGC_REVISION_MAJOR_REVISION_FADDR ADDR_VGC_REVISION
#define VGC_REVISION_MAJOR_REVISION_FSHIFT 4
#define VGC_REVISION_MAJOR_REVISION_FMASK 0xf
#define VGC_SYSSTATUS_RESET_FADDR ADDR_VGC_SYSSTATUS
#define VGC_SYSSTATUS_RESET_FSHIFT 0
#define VGC_SYSSTATUS_RESET_FMASK 0x1
#define G2D_ALPHABLEND_ALPHA_FADDR ADDR_G2D_ALPHABLEND
#define G2D_ALPHABLEND_ALPHA_FSHIFT 0
#define G2D_ALPHABLEND_ALPHA_FMASK 0xff
#define G2D_ALPHABLEND_OBS_ENABLE_FADDR ADDR_G2D_ALPHABLEND
#define G2D_ALPHABLEND_OBS_ENABLE_FSHIFT 8
#define G2D_ALPHABLEND_OBS_ENABLE_FMASK 0x1
#define G2D_ALPHABLEND_CONSTANT_FADDR ADDR_G2D_ALPHABLEND
#define G2D_ALPHABLEND_CONSTANT_FSHIFT 9
#define G2D_ALPHABLEND_CONSTANT_FMASK 0x1
#define G2D_ALPHABLEND_INVERT_FADDR ADDR_G2D_ALPHABLEND
#define G2D_ALPHABLEND_INVERT_FSHIFT 10
#define G2D_ALPHABLEND_INVERT_FMASK 0x1
#define G2D_ALPHABLEND_OPTIMIZE_FADDR ADDR_G2D_ALPHABLEND
#define G2D_ALPHABLEND_OPTIMIZE_FSHIFT 11
#define G2D_ALPHABLEND_OPTIMIZE_FMASK 0x1
#define G2D_ALPHABLEND_MODULATE_FADDR ADDR_G2D_ALPHABLEND
#define G2D_ALPHABLEND_MODULATE_FSHIFT 12
#define G2D_ALPHABLEND_MODULATE_FMASK 0x1
#define G2D_ALPHABLEND_INVERTMASK_FADDR ADDR_G2D_ALPHABLEND
#define G2D_ALPHABLEND_INVERTMASK_FSHIFT 13
#define G2D_ALPHABLEND_INVERTMASK_FMASK 0x1
#define G2D_ALPHABLEND_PREMULTIPLYDST_FADDR ADDR_G2D_ALPHABLEND
#define G2D_ALPHABLEND_PREMULTIPLYDST_FSHIFT 14
#define G2D_ALPHABLEND_PREMULTIPLYDST_FMASK 0x1
#define G2D_ALPHABLEND_MASKTOALPHA_FADDR ADDR_G2D_ALPHABLEND
#define G2D_ALPHABLEND_MASKTOALPHA_FSHIFT 15
#define G2D_ALPHABLEND_MASKTOALPHA_FMASK 0x1
#define G2D_BACKGROUND_COLOR_FADDR ADDR_G2D_BACKGROUND
#define G2D_BACKGROUND_COLOR_FSHIFT 0
#define G2D_BACKGROUND_COLOR_FMASK 0xffffffff
#define G2D_BASE0_ADDR_FADDR ADDR_G2D_BASE0
#define G2D_BASE0_ADDR_FSHIFT 0
#define G2D_BASE0_ADDR_FMASK 0xffffffff
#define G2D_BASE1_ADDR_FADDR ADDR_G2D_BASE1
#define G2D_BASE1_ADDR_FSHIFT 0
#define G2D_BASE1_ADDR_FMASK 0xffffffff
#define G2D_BASE2_ADDR_FADDR ADDR_G2D_BASE2
#define G2D_BASE2_ADDR_FSHIFT 0
#define G2D_BASE2_ADDR_FMASK 0xffffffff
#define G2D_BASE3_ADDR_FADDR ADDR_G2D_BASE3
#define G2D_BASE3_ADDR_FSHIFT 0
#define G2D_BASE3_ADDR_FMASK 0xffffffff
#define G2D_BLENDERCFG_PASSES_FADDR ADDR_G2D_BLENDERCFG
#define G2D_BLENDERCFG_PASSES_FSHIFT 0
#define G2D_BLENDERCFG_PASSES_FMASK 0x7
#define G2D_BLENDERCFG_ALPHAPASSES_FADDR ADDR_G2D_BLENDERCFG
#define G2D_BLENDERCFG_ALPHAPASSES_FSHIFT 3
#define G2D_BLENDERCFG_ALPHAPASSES_FMASK 0x3
#define G2D_BLENDERCFG_ENABLE_FADDR ADDR_G2D_BLENDERCFG
#define G2D_BLENDERCFG_ENABLE_FSHIFT 5
#define G2D_BLENDERCFG_ENABLE_FMASK 0x1
#define G2D_BLENDERCFG_OOALPHA_FADDR ADDR_G2D_BLENDERCFG
#define G2D_BLENDERCFG_OOALPHA_FSHIFT 6
#define G2D_BLENDERCFG_OOALPHA_FMASK 0x1
#define G2D_BLENDERCFG_OBS_DIVALPHA_FADDR ADDR_G2D_BLENDERCFG
#define G2D_BLENDERCFG_OBS_DIVALPHA_FSHIFT 7
#define G2D_BLENDERCFG_OBS_DIVALPHA_FMASK 0x1
#define G2D_BLENDERCFG_NOMASK_FADDR ADDR_G2D_BLENDERCFG
#define G2D_BLENDERCFG_NOMASK_FSHIFT 8
#define G2D_BLENDERCFG_NOMASK_FMASK 0x1
#define G2D_BLEND_A0_OPERATION_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_OPERATION_FSHIFT 0
#define G2D_BLEND_A0_OPERATION_FMASK 0x3
#define G2D_BLEND_A0_DST_A_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_DST_A_FSHIFT 2
#define G2D_BLEND_A0_DST_A_FMASK 0x3
#define G2D_BLEND_A0_DST_B_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_DST_B_FSHIFT 4
#define G2D_BLEND_A0_DST_B_FMASK 0x3
#define G2D_BLEND_A0_DST_C_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_DST_C_FSHIFT 6
#define G2D_BLEND_A0_DST_C_FMASK 0x3
#define G2D_BLEND_A0_AR_A_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_AR_A_FSHIFT 8
#define G2D_BLEND_A0_AR_A_FMASK 0x1
#define G2D_BLEND_A0_AR_B_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_AR_B_FSHIFT 9
#define G2D_BLEND_A0_AR_B_FMASK 0x1
#define G2D_BLEND_A0_AR_C_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_AR_C_FSHIFT 10
#define G2D_BLEND_A0_AR_C_FMASK 0x1
#define G2D_BLEND_A0_AR_D_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_AR_D_FSHIFT 11
#define G2D_BLEND_A0_AR_D_FMASK 0x1
#define G2D_BLEND_A0_INV_A_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_INV_A_FSHIFT 12
#define G2D_BLEND_A0_INV_A_FMASK 0x1
#define G2D_BLEND_A0_INV_B_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_INV_B_FSHIFT 13
#define G2D_BLEND_A0_INV_B_FMASK 0x1
#define G2D_BLEND_A0_INV_C_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_INV_C_FSHIFT 14
#define G2D_BLEND_A0_INV_C_FMASK 0x1
#define G2D_BLEND_A0_INV_D_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_INV_D_FSHIFT 15
#define G2D_BLEND_A0_INV_D_FMASK 0x1
#define G2D_BLEND_A0_SRC_A_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_SRC_A_FSHIFT 16
#define G2D_BLEND_A0_SRC_A_FMASK 0x7
#define G2D_BLEND_A0_SRC_B_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_SRC_B_FSHIFT 19
#define G2D_BLEND_A0_SRC_B_FMASK 0x7
#define G2D_BLEND_A0_SRC_C_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_SRC_C_FSHIFT 22
#define G2D_BLEND_A0_SRC_C_FMASK 0x7
#define G2D_BLEND_A0_SRC_D_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_SRC_D_FSHIFT 25
#define G2D_BLEND_A0_SRC_D_FMASK 0x7
#define G2D_BLEND_A0_CONST_A_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_CONST_A_FSHIFT 28
#define G2D_BLEND_A0_CONST_A_FMASK 0x1
#define G2D_BLEND_A0_CONST_B_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_CONST_B_FSHIFT 29
#define G2D_BLEND_A0_CONST_B_FMASK 0x1
#define G2D_BLEND_A0_CONST_C_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_CONST_C_FSHIFT 30
#define G2D_BLEND_A0_CONST_C_FMASK 0x1
#define G2D_BLEND_A0_CONST_D_FADDR ADDR_G2D_BLEND_A0
#define G2D_BLEND_A0_CONST_D_FSHIFT 31
#define G2D_BLEND_A0_CONST_D_FMASK 0x1
#define G2D_BLEND_A1_OPERATION_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_OPERATION_FSHIFT 0
#define G2D_BLEND_A1_OPERATION_FMASK 0x3
#define G2D_BLEND_A1_DST_A_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_DST_A_FSHIFT 2
#define G2D_BLEND_A1_DST_A_FMASK 0x3
#define G2D_BLEND_A1_DST_B_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_DST_B_FSHIFT 4
#define G2D_BLEND_A1_DST_B_FMASK 0x3
#define G2D_BLEND_A1_DST_C_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_DST_C_FSHIFT 6
#define G2D_BLEND_A1_DST_C_FMASK 0x3
#define G2D_BLEND_A1_AR_A_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_AR_A_FSHIFT 8
#define G2D_BLEND_A1_AR_A_FMASK 0x1
#define G2D_BLEND_A1_AR_B_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_AR_B_FSHIFT 9
#define G2D_BLEND_A1_AR_B_FMASK 0x1
#define G2D_BLEND_A1_AR_C_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_AR_C_FSHIFT 10
#define G2D_BLEND_A1_AR_C_FMASK 0x1
#define G2D_BLEND_A1_AR_D_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_AR_D_FSHIFT 11
#define G2D_BLEND_A1_AR_D_FMASK 0x1
#define G2D_BLEND_A1_INV_A_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_INV_A_FSHIFT 12
#define G2D_BLEND_A1_INV_A_FMASK 0x1
#define G2D_BLEND_A1_INV_B_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_INV_B_FSHIFT 13
#define G2D_BLEND_A1_INV_B_FMASK 0x1
#define G2D_BLEND_A1_INV_C_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_INV_C_FSHIFT 14
#define G2D_BLEND_A1_INV_C_FMASK 0x1
#define G2D_BLEND_A1_INV_D_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_INV_D_FSHIFT 15
#define G2D_BLEND_A1_INV_D_FMASK 0x1
#define G2D_BLEND_A1_SRC_A_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_SRC_A_FSHIFT 16
#define G2D_BLEND_A1_SRC_A_FMASK 0x7
#define G2D_BLEND_A1_SRC_B_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_SRC_B_FSHIFT 19
#define G2D_BLEND_A1_SRC_B_FMASK 0x7
#define G2D_BLEND_A1_SRC_C_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_SRC_C_FSHIFT 22
#define G2D_BLEND_A1_SRC_C_FMASK 0x7
#define G2D_BLEND_A1_SRC_D_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_SRC_D_FSHIFT 25
#define G2D_BLEND_A1_SRC_D_FMASK 0x7
#define G2D_BLEND_A1_CONST_A_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_CONST_A_FSHIFT 28
#define G2D_BLEND_A1_CONST_A_FMASK 0x1
#define G2D_BLEND_A1_CONST_B_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_CONST_B_FSHIFT 29
#define G2D_BLEND_A1_CONST_B_FMASK 0x1
#define G2D_BLEND_A1_CONST_C_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_CONST_C_FSHIFT 30
#define G2D_BLEND_A1_CONST_C_FMASK 0x1
#define G2D_BLEND_A1_CONST_D_FADDR ADDR_G2D_BLEND_A1
#define G2D_BLEND_A1_CONST_D_FSHIFT 31
#define G2D_BLEND_A1_CONST_D_FMASK 0x1
#define G2D_BLEND_A2_OPERATION_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_OPERATION_FSHIFT 0
#define G2D_BLEND_A2_OPERATION_FMASK 0x3
#define G2D_BLEND_A2_DST_A_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_DST_A_FSHIFT 2
#define G2D_BLEND_A2_DST_A_FMASK 0x3
#define G2D_BLEND_A2_DST_B_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_DST_B_FSHIFT 4
#define G2D_BLEND_A2_DST_B_FMASK 0x3
#define G2D_BLEND_A2_DST_C_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_DST_C_FSHIFT 6
#define G2D_BLEND_A2_DST_C_FMASK 0x3
#define G2D_BLEND_A2_AR_A_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_AR_A_FSHIFT 8
#define G2D_BLEND_A2_AR_A_FMASK 0x1
#define G2D_BLEND_A2_AR_B_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_AR_B_FSHIFT 9
#define G2D_BLEND_A2_AR_B_FMASK 0x1
#define G2D_BLEND_A2_AR_C_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_AR_C_FSHIFT 10
#define G2D_BLEND_A2_AR_C_FMASK 0x1
#define G2D_BLEND_A2_AR_D_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_AR_D_FSHIFT 11
#define G2D_BLEND_A2_AR_D_FMASK 0x1
#define G2D_BLEND_A2_INV_A_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_INV_A_FSHIFT 12
#define G2D_BLEND_A2_INV_A_FMASK 0x1
#define G2D_BLEND_A2_INV_B_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_INV_B_FSHIFT 13
#define G2D_BLEND_A2_INV_B_FMASK 0x1
#define G2D_BLEND_A2_INV_C_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_INV_C_FSHIFT 14
#define G2D_BLEND_A2_INV_C_FMASK 0x1
#define G2D_BLEND_A2_INV_D_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_INV_D_FSHIFT 15
#define G2D_BLEND_A2_INV_D_FMASK 0x1
#define G2D_BLEND_A2_SRC_A_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_SRC_A_FSHIFT 16
#define G2D_BLEND_A2_SRC_A_FMASK 0x7
#define G2D_BLEND_A2_SRC_B_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_SRC_B_FSHIFT 19
#define G2D_BLEND_A2_SRC_B_FMASK 0x7
#define G2D_BLEND_A2_SRC_C_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_SRC_C_FSHIFT 22
#define G2D_BLEND_A2_SRC_C_FMASK 0x7
#define G2D_BLEND_A2_SRC_D_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_SRC_D_FSHIFT 25
#define G2D_BLEND_A2_SRC_D_FMASK 0x7
#define G2D_BLEND_A2_CONST_A_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_CONST_A_FSHIFT 28
#define G2D_BLEND_A2_CONST_A_FMASK 0x1
#define G2D_BLEND_A2_CONST_B_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_CONST_B_FSHIFT 29
#define G2D_BLEND_A2_CONST_B_FMASK 0x1
#define G2D_BLEND_A2_CONST_C_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_CONST_C_FSHIFT 30
#define G2D_BLEND_A2_CONST_C_FMASK 0x1
#define G2D_BLEND_A2_CONST_D_FADDR ADDR_G2D_BLEND_A2
#define G2D_BLEND_A2_CONST_D_FSHIFT 31
#define G2D_BLEND_A2_CONST_D_FMASK 0x1
#define G2D_BLEND_A3_OPERATION_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_OPERATION_FSHIFT 0
#define G2D_BLEND_A3_OPERATION_FMASK 0x3
#define G2D_BLEND_A3_DST_A_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_DST_A_FSHIFT 2
#define G2D_BLEND_A3_DST_A_FMASK 0x3
#define G2D_BLEND_A3_DST_B_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_DST_B_FSHIFT 4
#define G2D_BLEND_A3_DST_B_FMASK 0x3
#define G2D_BLEND_A3_DST_C_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_DST_C_FSHIFT 6
#define G2D_BLEND_A3_DST_C_FMASK 0x3
#define G2D_BLEND_A3_AR_A_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_AR_A_FSHIFT 8
#define G2D_BLEND_A3_AR_A_FMASK 0x1
#define G2D_BLEND_A3_AR_B_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_AR_B_FSHIFT 9
#define G2D_BLEND_A3_AR_B_FMASK 0x1
#define G2D_BLEND_A3_AR_C_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_AR_C_FSHIFT 10
#define G2D_BLEND_A3_AR_C_FMASK 0x1
#define G2D_BLEND_A3_AR_D_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_AR_D_FSHIFT 11
#define G2D_BLEND_A3_AR_D_FMASK 0x1
#define G2D_BLEND_A3_INV_A_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_INV_A_FSHIFT 12
#define G2D_BLEND_A3_INV_A_FMASK 0x1
#define G2D_BLEND_A3_INV_B_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_INV_B_FSHIFT 13
#define G2D_BLEND_A3_INV_B_FMASK 0x1
#define G2D_BLEND_A3_INV_C_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_INV_C_FSHIFT 14
#define G2D_BLEND_A3_INV_C_FMASK 0x1
#define G2D_BLEND_A3_INV_D_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_INV_D_FSHIFT 15
#define G2D_BLEND_A3_INV_D_FMASK 0x1
#define G2D_BLEND_A3_SRC_A_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_SRC_A_FSHIFT 16
#define G2D_BLEND_A3_SRC_A_FMASK 0x7
#define G2D_BLEND_A3_SRC_B_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_SRC_B_FSHIFT 19
#define G2D_BLEND_A3_SRC_B_FMASK 0x7
#define G2D_BLEND_A3_SRC_C_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_SRC_C_FSHIFT 22
#define G2D_BLEND_A3_SRC_C_FMASK 0x7
#define G2D_BLEND_A3_SRC_D_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_SRC_D_FSHIFT 25
#define G2D_BLEND_A3_SRC_D_FMASK 0x7
#define G2D_BLEND_A3_CONST_A_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_CONST_A_FSHIFT 28
#define G2D_BLEND_A3_CONST_A_FMASK 0x1
#define G2D_BLEND_A3_CONST_B_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_CONST_B_FSHIFT 29
#define G2D_BLEND_A3_CONST_B_FMASK 0x1
#define G2D_BLEND_A3_CONST_C_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_CONST_C_FSHIFT 30
#define G2D_BLEND_A3_CONST_C_FMASK 0x1
#define G2D_BLEND_A3_CONST_D_FADDR ADDR_G2D_BLEND_A3
#define G2D_BLEND_A3_CONST_D_FSHIFT 31
#define G2D_BLEND_A3_CONST_D_FMASK 0x1
#define G2D_BLEND_C0_OPERATION_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_OPERATION_FSHIFT 0
#define G2D_BLEND_C0_OPERATION_FMASK 0x3
#define G2D_BLEND_C0_DST_A_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_DST_A_FSHIFT 2
#define G2D_BLEND_C0_DST_A_FMASK 0x3
#define G2D_BLEND_C0_DST_B_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_DST_B_FSHIFT 4
#define G2D_BLEND_C0_DST_B_FMASK 0x3
#define G2D_BLEND_C0_DST_C_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_DST_C_FSHIFT 6
#define G2D_BLEND_C0_DST_C_FMASK 0x3
#define G2D_BLEND_C0_AR_A_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_AR_A_FSHIFT 8
#define G2D_BLEND_C0_AR_A_FMASK 0x1
#define G2D_BLEND_C0_AR_B_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_AR_B_FSHIFT 9
#define G2D_BLEND_C0_AR_B_FMASK 0x1
#define G2D_BLEND_C0_AR_C_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_AR_C_FSHIFT 10
#define G2D_BLEND_C0_AR_C_FMASK 0x1
#define G2D_BLEND_C0_AR_D_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_AR_D_FSHIFT 11
#define G2D_BLEND_C0_AR_D_FMASK 0x1
#define G2D_BLEND_C0_INV_A_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_INV_A_FSHIFT 12
#define G2D_BLEND_C0_INV_A_FMASK 0x1
#define G2D_BLEND_C0_INV_B_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_INV_B_FSHIFT 13
#define G2D_BLEND_C0_INV_B_FMASK 0x1
#define G2D_BLEND_C0_INV_C_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_INV_C_FSHIFT 14
#define G2D_BLEND_C0_INV_C_FMASK 0x1
#define G2D_BLEND_C0_INV_D_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_INV_D_FSHIFT 15
#define G2D_BLEND_C0_INV_D_FMASK 0x1
#define G2D_BLEND_C0_SRC_A_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_SRC_A_FSHIFT 16
#define G2D_BLEND_C0_SRC_A_FMASK 0x7
#define G2D_BLEND_C0_SRC_B_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_SRC_B_FSHIFT 19
#define G2D_BLEND_C0_SRC_B_FMASK 0x7
#define G2D_BLEND_C0_SRC_C_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_SRC_C_FSHIFT 22
#define G2D_BLEND_C0_SRC_C_FMASK 0x7
#define G2D_BLEND_C0_SRC_D_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_SRC_D_FSHIFT 25
#define G2D_BLEND_C0_SRC_D_FMASK 0x7
#define G2D_BLEND_C0_CONST_A_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_CONST_A_FSHIFT 28
#define G2D_BLEND_C0_CONST_A_FMASK 0x1
#define G2D_BLEND_C0_CONST_B_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_CONST_B_FSHIFT 29
#define G2D_BLEND_C0_CONST_B_FMASK 0x1
#define G2D_BLEND_C0_CONST_C_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_CONST_C_FSHIFT 30
#define G2D_BLEND_C0_CONST_C_FMASK 0x1
#define G2D_BLEND_C0_CONST_D_FADDR ADDR_G2D_BLEND_C0
#define G2D_BLEND_C0_CONST_D_FSHIFT 31
#define G2D_BLEND_C0_CONST_D_FMASK 0x1
#define G2D_BLEND_C1_OPERATION_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_OPERATION_FSHIFT 0
#define G2D_BLEND_C1_OPERATION_FMASK 0x3
#define G2D_BLEND_C1_DST_A_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_DST_A_FSHIFT 2
#define G2D_BLEND_C1_DST_A_FMASK 0x3
#define G2D_BLEND_C1_DST_B_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_DST_B_FSHIFT 4
#define G2D_BLEND_C1_DST_B_FMASK 0x3
#define G2D_BLEND_C1_DST_C_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_DST_C_FSHIFT 6
#define G2D_BLEND_C1_DST_C_FMASK 0x3
#define G2D_BLEND_C1_AR_A_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_AR_A_FSHIFT 8
#define G2D_BLEND_C1_AR_A_FMASK 0x1
#define G2D_BLEND_C1_AR_B_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_AR_B_FSHIFT 9
#define G2D_BLEND_C1_AR_B_FMASK 0x1
#define G2D_BLEND_C1_AR_C_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_AR_C_FSHIFT 10
#define G2D_BLEND_C1_AR_C_FMASK 0x1
#define G2D_BLEND_C1_AR_D_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_AR_D_FSHIFT 11
#define G2D_BLEND_C1_AR_D_FMASK 0x1
#define G2D_BLEND_C1_INV_A_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_INV_A_FSHIFT 12
#define G2D_BLEND_C1_INV_A_FMASK 0x1
#define G2D_BLEND_C1_INV_B_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_INV_B_FSHIFT 13
#define G2D_BLEND_C1_INV_B_FMASK 0x1
#define G2D_BLEND_C1_INV_C_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_INV_C_FSHIFT 14
#define G2D_BLEND_C1_INV_C_FMASK 0x1
#define G2D_BLEND_C1_INV_D_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_INV_D_FSHIFT 15
#define G2D_BLEND_C1_INV_D_FMASK 0x1
#define G2D_BLEND_C1_SRC_A_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_SRC_A_FSHIFT 16
#define G2D_BLEND_C1_SRC_A_FMASK 0x7
#define G2D_BLEND_C1_SRC_B_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_SRC_B_FSHIFT 19
#define G2D_BLEND_C1_SRC_B_FMASK 0x7
#define G2D_BLEND_C1_SRC_C_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_SRC_C_FSHIFT 22
#define G2D_BLEND_C1_SRC_C_FMASK 0x7
#define G2D_BLEND_C1_SRC_D_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_SRC_D_FSHIFT 25
#define G2D_BLEND_C1_SRC_D_FMASK 0x7
#define G2D_BLEND_C1_CONST_A_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_CONST_A_FSHIFT 28
#define G2D_BLEND_C1_CONST_A_FMASK 0x1
#define G2D_BLEND_C1_CONST_B_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_CONST_B_FSHIFT 29
#define G2D_BLEND_C1_CONST_B_FMASK 0x1
#define G2D_BLEND_C1_CONST_C_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_CONST_C_FSHIFT 30
#define G2D_BLEND_C1_CONST_C_FMASK 0x1
#define G2D_BLEND_C1_CONST_D_FADDR ADDR_G2D_BLEND_C1
#define G2D_BLEND_C1_CONST_D_FSHIFT 31
#define G2D_BLEND_C1_CONST_D_FMASK 0x1
#define G2D_BLEND_C2_OPERATION_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_OPERATION_FSHIFT 0
#define G2D_BLEND_C2_OPERATION_FMASK 0x3
#define G2D_BLEND_C2_DST_A_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_DST_A_FSHIFT 2
#define G2D_BLEND_C2_DST_A_FMASK 0x3
#define G2D_BLEND_C2_DST_B_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_DST_B_FSHIFT 4
#define G2D_BLEND_C2_DST_B_FMASK 0x3
#define G2D_BLEND_C2_DST_C_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_DST_C_FSHIFT 6
#define G2D_BLEND_C2_DST_C_FMASK 0x3
#define G2D_BLEND_C2_AR_A_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_AR_A_FSHIFT 8
#define G2D_BLEND_C2_AR_A_FMASK 0x1
#define G2D_BLEND_C2_AR_B_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_AR_B_FSHIFT 9
#define G2D_BLEND_C2_AR_B_FMASK 0x1
#define G2D_BLEND_C2_AR_C_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_AR_C_FSHIFT 10
#define G2D_BLEND_C2_AR_C_FMASK 0x1
#define G2D_BLEND_C2_AR_D_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_AR_D_FSHIFT 11
#define G2D_BLEND_C2_AR_D_FMASK 0x1
#define G2D_BLEND_C2_INV_A_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_INV_A_FSHIFT 12
#define G2D_BLEND_C2_INV_A_FMASK 0x1
#define G2D_BLEND_C2_INV_B_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_INV_B_FSHIFT 13
#define G2D_BLEND_C2_INV_B_FMASK 0x1
#define G2D_BLEND_C2_INV_C_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_INV_C_FSHIFT 14
#define G2D_BLEND_C2_INV_C_FMASK 0x1
#define G2D_BLEND_C2_INV_D_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_INV_D_FSHIFT 15
#define G2D_BLEND_C2_INV_D_FMASK 0x1
#define G2D_BLEND_C2_SRC_A_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_SRC_A_FSHIFT 16
#define G2D_BLEND_C2_SRC_A_FMASK 0x7
#define G2D_BLEND_C2_SRC_B_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_SRC_B_FSHIFT 19
#define G2D_BLEND_C2_SRC_B_FMASK 0x7
#define G2D_BLEND_C2_SRC_C_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_SRC_C_FSHIFT 22
#define G2D_BLEND_C2_SRC_C_FMASK 0x7
#define G2D_BLEND_C2_SRC_D_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_SRC_D_FSHIFT 25
#define G2D_BLEND_C2_SRC_D_FMASK 0x7
#define G2D_BLEND_C2_CONST_A_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_CONST_A_FSHIFT 28
#define G2D_BLEND_C2_CONST_A_FMASK 0x1
#define G2D_BLEND_C2_CONST_B_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_CONST_B_FSHIFT 29
#define G2D_BLEND_C2_CONST_B_FMASK 0x1
#define G2D_BLEND_C2_CONST_C_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_CONST_C_FSHIFT 30
#define G2D_BLEND_C2_CONST_C_FMASK 0x1
#define G2D_BLEND_C2_CONST_D_FADDR ADDR_G2D_BLEND_C2
#define G2D_BLEND_C2_CONST_D_FSHIFT 31
#define G2D_BLEND_C2_CONST_D_FMASK 0x1
#define G2D_BLEND_C3_OPERATION_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_OPERATION_FSHIFT 0
#define G2D_BLEND_C3_OPERATION_FMASK 0x3
#define G2D_BLEND_C3_DST_A_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_DST_A_FSHIFT 2
#define G2D_BLEND_C3_DST_A_FMASK 0x3
#define G2D_BLEND_C3_DST_B_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_DST_B_FSHIFT 4
#define G2D_BLEND_C3_DST_B_FMASK 0x3
#define G2D_BLEND_C3_DST_C_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_DST_C_FSHIFT 6
#define G2D_BLEND_C3_DST_C_FMASK 0x3
#define G2D_BLEND_C3_AR_A_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_AR_A_FSHIFT 8
#define G2D_BLEND_C3_AR_A_FMASK 0x1
#define G2D_BLEND_C3_AR_B_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_AR_B_FSHIFT 9
#define G2D_BLEND_C3_AR_B_FMASK 0x1
#define G2D_BLEND_C3_AR_C_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_AR_C_FSHIFT 10
#define G2D_BLEND_C3_AR_C_FMASK 0x1
#define G2D_BLEND_C3_AR_D_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_AR_D_FSHIFT 11
#define G2D_BLEND_C3_AR_D_FMASK 0x1
#define G2D_BLEND_C3_INV_A_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_INV_A_FSHIFT 12
#define G2D_BLEND_C3_INV_A_FMASK 0x1
#define G2D_BLEND_C3_INV_B_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_INV_B_FSHIFT 13
#define G2D_BLEND_C3_INV_B_FMASK 0x1
#define G2D_BLEND_C3_INV_C_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_INV_C_FSHIFT 14
#define G2D_BLEND_C3_INV_C_FMASK 0x1
#define G2D_BLEND_C3_INV_D_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_INV_D_FSHIFT 15
#define G2D_BLEND_C3_INV_D_FMASK 0x1
#define G2D_BLEND_C3_SRC_A_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_SRC_A_FSHIFT 16
#define G2D_BLEND_C3_SRC_A_FMASK 0x7
#define G2D_BLEND_C3_SRC_B_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_SRC_B_FSHIFT 19
#define G2D_BLEND_C3_SRC_B_FMASK 0x7
#define G2D_BLEND_C3_SRC_C_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_SRC_C_FSHIFT 22
#define G2D_BLEND_C3_SRC_C_FMASK 0x7
#define G2D_BLEND_C3_SRC_D_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_SRC_D_FSHIFT 25
#define G2D_BLEND_C3_SRC_D_FMASK 0x7
#define G2D_BLEND_C3_CONST_A_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_CONST_A_FSHIFT 28
#define G2D_BLEND_C3_CONST_A_FMASK 0x1
#define G2D_BLEND_C3_CONST_B_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_CONST_B_FSHIFT 29
#define G2D_BLEND_C3_CONST_B_FMASK 0x1
#define G2D_BLEND_C3_CONST_C_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_CONST_C_FSHIFT 30
#define G2D_BLEND_C3_CONST_C_FMASK 0x1
#define G2D_BLEND_C3_CONST_D_FADDR ADDR_G2D_BLEND_C3
#define G2D_BLEND_C3_CONST_D_FSHIFT 31
#define G2D_BLEND_C3_CONST_D_FMASK 0x1
#define G2D_BLEND_C4_OPERATION_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_OPERATION_FSHIFT 0
#define G2D_BLEND_C4_OPERATION_FMASK 0x3
#define G2D_BLEND_C4_DST_A_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_DST_A_FSHIFT 2
#define G2D_BLEND_C4_DST_A_FMASK 0x3
#define G2D_BLEND_C4_DST_B_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_DST_B_FSHIFT 4
#define G2D_BLEND_C4_DST_B_FMASK 0x3
#define G2D_BLEND_C4_DST_C_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_DST_C_FSHIFT 6
#define G2D_BLEND_C4_DST_C_FMASK 0x3
#define G2D_BLEND_C4_AR_A_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_AR_A_FSHIFT 8
#define G2D_BLEND_C4_AR_A_FMASK 0x1
#define G2D_BLEND_C4_AR_B_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_AR_B_FSHIFT 9
#define G2D_BLEND_C4_AR_B_FMASK 0x1
#define G2D_BLEND_C4_AR_C_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_AR_C_FSHIFT 10
#define G2D_BLEND_C4_AR_C_FMASK 0x1
#define G2D_BLEND_C4_AR_D_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_AR_D_FSHIFT 11
#define G2D_BLEND_C4_AR_D_FMASK 0x1
#define G2D_BLEND_C4_INV_A_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_INV_A_FSHIFT 12
#define G2D_BLEND_C4_INV_A_FMASK 0x1
#define G2D_BLEND_C4_INV_B_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_INV_B_FSHIFT 13
#define G2D_BLEND_C4_INV_B_FMASK 0x1
#define G2D_BLEND_C4_INV_C_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_INV_C_FSHIFT 14
#define G2D_BLEND_C4_INV_C_FMASK 0x1
#define G2D_BLEND_C4_INV_D_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_INV_D_FSHIFT 15
#define G2D_BLEND_C4_INV_D_FMASK 0x1
#define G2D_BLEND_C4_SRC_A_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_SRC_A_FSHIFT 16
#define G2D_BLEND_C4_SRC_A_FMASK 0x7
#define G2D_BLEND_C4_SRC_B_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_SRC_B_FSHIFT 19
#define G2D_BLEND_C4_SRC_B_FMASK 0x7
#define G2D_BLEND_C4_SRC_C_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_SRC_C_FSHIFT 22
#define G2D_BLEND_C4_SRC_C_FMASK 0x7
#define G2D_BLEND_C4_SRC_D_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_SRC_D_FSHIFT 25
#define G2D_BLEND_C4_SRC_D_FMASK 0x7
#define G2D_BLEND_C4_CONST_A_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_CONST_A_FSHIFT 28
#define G2D_BLEND_C4_CONST_A_FMASK 0x1
#define G2D_BLEND_C4_CONST_B_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_CONST_B_FSHIFT 29
#define G2D_BLEND_C4_CONST_B_FMASK 0x1
#define G2D_BLEND_C4_CONST_C_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_CONST_C_FSHIFT 30
#define G2D_BLEND_C4_CONST_C_FMASK 0x1
#define G2D_BLEND_C4_CONST_D_FADDR ADDR_G2D_BLEND_C4
#define G2D_BLEND_C4_CONST_D_FSHIFT 31
#define G2D_BLEND_C4_CONST_D_FMASK 0x1
#define G2D_BLEND_C5_OPERATION_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_OPERATION_FSHIFT 0
#define G2D_BLEND_C5_OPERATION_FMASK 0x3
#define G2D_BLEND_C5_DST_A_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_DST_A_FSHIFT 2
#define G2D_BLEND_C5_DST_A_FMASK 0x3
#define G2D_BLEND_C5_DST_B_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_DST_B_FSHIFT 4
#define G2D_BLEND_C5_DST_B_FMASK 0x3
#define G2D_BLEND_C5_DST_C_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_DST_C_FSHIFT 6
#define G2D_BLEND_C5_DST_C_FMASK 0x3
#define G2D_BLEND_C5_AR_A_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_AR_A_FSHIFT 8
#define G2D_BLEND_C5_AR_A_FMASK 0x1
#define G2D_BLEND_C5_AR_B_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_AR_B_FSHIFT 9
#define G2D_BLEND_C5_AR_B_FMASK 0x1
#define G2D_BLEND_C5_AR_C_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_AR_C_FSHIFT 10
#define G2D_BLEND_C5_AR_C_FMASK 0x1
#define G2D_BLEND_C5_AR_D_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_AR_D_FSHIFT 11
#define G2D_BLEND_C5_AR_D_FMASK 0x1
#define G2D_BLEND_C5_INV_A_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_INV_A_FSHIFT 12
#define G2D_BLEND_C5_INV_A_FMASK 0x1
#define G2D_BLEND_C5_INV_B_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_INV_B_FSHIFT 13
#define G2D_BLEND_C5_INV_B_FMASK 0x1
#define G2D_BLEND_C5_INV_C_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_INV_C_FSHIFT 14
#define G2D_BLEND_C5_INV_C_FMASK 0x1
#define G2D_BLEND_C5_INV_D_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_INV_D_FSHIFT 15
#define G2D_BLEND_C5_INV_D_FMASK 0x1
#define G2D_BLEND_C5_SRC_A_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_SRC_A_FSHIFT 16
#define G2D_BLEND_C5_SRC_A_FMASK 0x7
#define G2D_BLEND_C5_SRC_B_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_SRC_B_FSHIFT 19
#define G2D_BLEND_C5_SRC_B_FMASK 0x7
#define G2D_BLEND_C5_SRC_C_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_SRC_C_FSHIFT 22
#define G2D_BLEND_C5_SRC_C_FMASK 0x7
#define G2D_BLEND_C5_SRC_D_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_SRC_D_FSHIFT 25
#define G2D_BLEND_C5_SRC_D_FMASK 0x7
#define G2D_BLEND_C5_CONST_A_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_CONST_A_FSHIFT 28
#define G2D_BLEND_C5_CONST_A_FMASK 0x1
#define G2D_BLEND_C5_CONST_B_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_CONST_B_FSHIFT 29
#define G2D_BLEND_C5_CONST_B_FMASK 0x1
#define G2D_BLEND_C5_CONST_C_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_CONST_C_FSHIFT 30
#define G2D_BLEND_C5_CONST_C_FMASK 0x1
#define G2D_BLEND_C5_CONST_D_FADDR ADDR_G2D_BLEND_C5
#define G2D_BLEND_C5_CONST_D_FSHIFT 31
#define G2D_BLEND_C5_CONST_D_FMASK 0x1
#define G2D_BLEND_C6_OPERATION_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_OPERATION_FSHIFT 0
#define G2D_BLEND_C6_OPERATION_FMASK 0x3
#define G2D_BLEND_C6_DST_A_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_DST_A_FSHIFT 2
#define G2D_BLEND_C6_DST_A_FMASK 0x3
#define G2D_BLEND_C6_DST_B_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_DST_B_FSHIFT 4
#define G2D_BLEND_C6_DST_B_FMASK 0x3
#define G2D_BLEND_C6_DST_C_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_DST_C_FSHIFT 6
#define G2D_BLEND_C6_DST_C_FMASK 0x3
#define G2D_BLEND_C6_AR_A_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_AR_A_FSHIFT 8
#define G2D_BLEND_C6_AR_A_FMASK 0x1
#define G2D_BLEND_C6_AR_B_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_AR_B_FSHIFT 9
#define G2D_BLEND_C6_AR_B_FMASK 0x1
#define G2D_BLEND_C6_AR_C_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_AR_C_FSHIFT 10
#define G2D_BLEND_C6_AR_C_FMASK 0x1
#define G2D_BLEND_C6_AR_D_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_AR_D_FSHIFT 11
#define G2D_BLEND_C6_AR_D_FMASK 0x1
#define G2D_BLEND_C6_INV_A_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_INV_A_FSHIFT 12
#define G2D_BLEND_C6_INV_A_FMASK 0x1
#define G2D_BLEND_C6_INV_B_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_INV_B_FSHIFT 13
#define G2D_BLEND_C6_INV_B_FMASK 0x1
#define G2D_BLEND_C6_INV_C_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_INV_C_FSHIFT 14
#define G2D_BLEND_C6_INV_C_FMASK 0x1
#define G2D_BLEND_C6_INV_D_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_INV_D_FSHIFT 15
#define G2D_BLEND_C6_INV_D_FMASK 0x1
#define G2D_BLEND_C6_SRC_A_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_SRC_A_FSHIFT 16
#define G2D_BLEND_C6_SRC_A_FMASK 0x7
#define G2D_BLEND_C6_SRC_B_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_SRC_B_FSHIFT 19
#define G2D_BLEND_C6_SRC_B_FMASK 0x7
#define G2D_BLEND_C6_SRC_C_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_SRC_C_FSHIFT 22
#define G2D_BLEND_C6_SRC_C_FMASK 0x7
#define G2D_BLEND_C6_SRC_D_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_SRC_D_FSHIFT 25
#define G2D_BLEND_C6_SRC_D_FMASK 0x7
#define G2D_BLEND_C6_CONST_A_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_CONST_A_FSHIFT 28
#define G2D_BLEND_C6_CONST_A_FMASK 0x1
#define G2D_BLEND_C6_CONST_B_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_CONST_B_FSHIFT 29
#define G2D_BLEND_C6_CONST_B_FMASK 0x1
#define G2D_BLEND_C6_CONST_C_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_CONST_C_FSHIFT 30
#define G2D_BLEND_C6_CONST_C_FMASK 0x1
#define G2D_BLEND_C6_CONST_D_FADDR ADDR_G2D_BLEND_C6
#define G2D_BLEND_C6_CONST_D_FSHIFT 31
#define G2D_BLEND_C6_CONST_D_FMASK 0x1
#define G2D_BLEND_C7_OPERATION_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_OPERATION_FSHIFT 0
#define G2D_BLEND_C7_OPERATION_FMASK 0x3
#define G2D_BLEND_C7_DST_A_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_DST_A_FSHIFT 2
#define G2D_BLEND_C7_DST_A_FMASK 0x3
#define G2D_BLEND_C7_DST_B_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_DST_B_FSHIFT 4
#define G2D_BLEND_C7_DST_B_FMASK 0x3
#define G2D_BLEND_C7_DST_C_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_DST_C_FSHIFT 6
#define G2D_BLEND_C7_DST_C_FMASK 0x3
#define G2D_BLEND_C7_AR_A_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_AR_A_FSHIFT 8
#define G2D_BLEND_C7_AR_A_FMASK 0x1
#define G2D_BLEND_C7_AR_B_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_AR_B_FSHIFT 9
#define G2D_BLEND_C7_AR_B_FMASK 0x1
#define G2D_BLEND_C7_AR_C_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_AR_C_FSHIFT 10
#define G2D_BLEND_C7_AR_C_FMASK 0x1
#define G2D_BLEND_C7_AR_D_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_AR_D_FSHIFT 11
#define G2D_BLEND_C7_AR_D_FMASK 0x1
#define G2D_BLEND_C7_INV_A_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_INV_A_FSHIFT 12
#define G2D_BLEND_C7_INV_A_FMASK 0x1
#define G2D_BLEND_C7_INV_B_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_INV_B_FSHIFT 13
#define G2D_BLEND_C7_INV_B_FMASK 0x1
#define G2D_BLEND_C7_INV_C_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_INV_C_FSHIFT 14
#define G2D_BLEND_C7_INV_C_FMASK 0x1
#define G2D_BLEND_C7_INV_D_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_INV_D_FSHIFT 15
#define G2D_BLEND_C7_INV_D_FMASK 0x1
#define G2D_BLEND_C7_SRC_A_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_SRC_A_FSHIFT 16
#define G2D_BLEND_C7_SRC_A_FMASK 0x7
#define G2D_BLEND_C7_SRC_B_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_SRC_B_FSHIFT 19
#define G2D_BLEND_C7_SRC_B_FMASK 0x7
#define G2D_BLEND_C7_SRC_C_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_SRC_C_FSHIFT 22
#define G2D_BLEND_C7_SRC_C_FMASK 0x7
#define G2D_BLEND_C7_SRC_D_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_SRC_D_FSHIFT 25
#define G2D_BLEND_C7_SRC_D_FMASK 0x7
#define G2D_BLEND_C7_CONST_A_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_CONST_A_FSHIFT 28
#define G2D_BLEND_C7_CONST_A_FMASK 0x1
#define G2D_BLEND_C7_CONST_B_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_CONST_B_FSHIFT 29
#define G2D_BLEND_C7_CONST_B_FMASK 0x1
#define G2D_BLEND_C7_CONST_C_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_CONST_C_FSHIFT 30
#define G2D_BLEND_C7_CONST_C_FMASK 0x1
#define G2D_BLEND_C7_CONST_D_FADDR ADDR_G2D_BLEND_C7
#define G2D_BLEND_C7_CONST_D_FSHIFT 31
#define G2D_BLEND_C7_CONST_D_FMASK 0x1
#define G2D_CFG0_STRIDE_FADDR ADDR_G2D_CFG0
#define G2D_CFG0_STRIDE_FSHIFT 0
#define G2D_CFG0_STRIDE_FMASK 0xfff
#define G2D_CFG0_FORMAT_FADDR ADDR_G2D_CFG0
#define G2D_CFG0_FORMAT_FSHIFT 12
#define G2D_CFG0_FORMAT_FMASK 0xf
#define G2D_CFG0_TILED_FADDR ADDR_G2D_CFG0
#define G2D_CFG0_TILED_FSHIFT 16
#define G2D_CFG0_TILED_FMASK 0x1
#define G2D_CFG0_SRGB_FADDR ADDR_G2D_CFG0
#define G2D_CFG0_SRGB_FSHIFT 17
#define G2D_CFG0_SRGB_FMASK 0x1
#define G2D_CFG0_SWAPWORDS_FADDR ADDR_G2D_CFG0
#define G2D_CFG0_SWAPWORDS_FSHIFT 18
#define G2D_CFG0_SWAPWORDS_FMASK 0x1
#define G2D_CFG0_SWAPBYTES_FADDR ADDR_G2D_CFG0
#define G2D_CFG0_SWAPBYTES_FSHIFT 19
#define G2D_CFG0_SWAPBYTES_FMASK 0x1
#define G2D_CFG0_SWAPALL_FADDR ADDR_G2D_CFG0
#define G2D_CFG0_SWAPALL_FSHIFT 20
#define G2D_CFG0_SWAPALL_FMASK 0x1
#define G2D_CFG0_SWAPRB_FADDR ADDR_G2D_CFG0
#define G2D_CFG0_SWAPRB_FSHIFT 21
#define G2D_CFG0_SWAPRB_FMASK 0x1
#define G2D_CFG0_SWAPBITS_FADDR ADDR_G2D_CFG0
#define G2D_CFG0_SWAPBITS_FSHIFT 22
#define G2D_CFG0_SWAPBITS_FMASK 0x1
#define G2D_CFG0_STRIDESIGN_FADDR ADDR_G2D_CFG0
#define G2D_CFG0_STRIDESIGN_FSHIFT 23
#define G2D_CFG0_STRIDESIGN_FMASK 0x1
#define G2D_CFG1_STRIDE_FADDR ADDR_G2D_CFG1
#define G2D_CFG1_STRIDE_FSHIFT 0
#define G2D_CFG1_STRIDE_FMASK 0xfff
#define G2D_CFG1_FORMAT_FADDR ADDR_G2D_CFG1
#define G2D_CFG1_FORMAT_FSHIFT 12
#define G2D_CFG1_FORMAT_FMASK 0xf
#define G2D_CFG1_TILED_FADDR ADDR_G2D_CFG1
#define G2D_CFG1_TILED_FSHIFT 16
#define G2D_CFG1_TILED_FMASK 0x1
#define G2D_CFG1_SRGB_FADDR ADDR_G2D_CFG1
#define G2D_CFG1_SRGB_FSHIFT 17
#define G2D_CFG1_SRGB_FMASK 0x1
#define G2D_CFG1_SWAPWORDS_FADDR ADDR_G2D_CFG1
#define G2D_CFG1_SWAPWORDS_FSHIFT 18
#define G2D_CFG1_SWAPWORDS_FMASK 0x1
#define G2D_CFG1_SWAPBYTES_FADDR ADDR_G2D_CFG1
#define G2D_CFG1_SWAPBYTES_FSHIFT 19
#define G2D_CFG1_SWAPBYTES_FMASK 0x1
#define G2D_CFG1_SWAPALL_FADDR ADDR_G2D_CFG1
#define G2D_CFG1_SWAPALL_FSHIFT 20
#define G2D_CFG1_SWAPALL_FMASK 0x1
#define G2D_CFG1_SWAPRB_FADDR ADDR_G2D_CFG1
#define G2D_CFG1_SWAPRB_FSHIFT 21
#define G2D_CFG1_SWAPRB_FMASK 0x1
#define G2D_CFG1_SWAPBITS_FADDR ADDR_G2D_CFG1
#define G2D_CFG1_SWAPBITS_FSHIFT 22
#define G2D_CFG1_SWAPBITS_FMASK 0x1
#define G2D_CFG1_STRIDESIGN_FADDR ADDR_G2D_CFG1
#define G2D_CFG1_STRIDESIGN_FSHIFT 23
#define G2D_CFG1_STRIDESIGN_FMASK 0x1
#define G2D_CFG2_STRIDE_FADDR ADDR_G2D_CFG2
#define G2D_CFG2_STRIDE_FSHIFT 0
#define G2D_CFG2_STRIDE_FMASK 0xfff
#define G2D_CFG2_FORMAT_FADDR ADDR_G2D_CFG2
#define G2D_CFG2_FORMAT_FSHIFT 12
#define G2D_CFG2_FORMAT_FMASK 0xf
#define G2D_CFG2_TILED_FADDR ADDR_G2D_CFG2
#define G2D_CFG2_TILED_FSHIFT 16
#define G2D_CFG2_TILED_FMASK 0x1
#define G2D_CFG2_SRGB_FADDR ADDR_G2D_CFG2
#define G2D_CFG2_SRGB_FSHIFT 17
#define G2D_CFG2_SRGB_FMASK 0x1
#define G2D_CFG2_SWAPWORDS_FADDR ADDR_G2D_CFG2
#define G2D_CFG2_SWAPWORDS_FSHIFT 18
#define G2D_CFG2_SWAPWORDS_FMASK 0x1
#define G2D_CFG2_SWAPBYTES_FADDR ADDR_G2D_CFG2
#define G2D_CFG2_SWAPBYTES_FSHIFT 19
#define G2D_CFG2_SWAPBYTES_FMASK 0x1
#define G2D_CFG2_SWAPALL_FADDR ADDR_G2D_CFG2
#define G2D_CFG2_SWAPALL_FSHIFT 20
#define G2D_CFG2_SWAPALL_FMASK 0x1
#define G2D_CFG2_SWAPRB_FADDR ADDR_G2D_CFG2
#define G2D_CFG2_SWAPRB_FSHIFT 21
#define G2D_CFG2_SWAPRB_FMASK 0x1
#define G2D_CFG2_SWAPBITS_FADDR ADDR_G2D_CFG2
#define G2D_CFG2_SWAPBITS_FSHIFT 22
#define G2D_CFG2_SWAPBITS_FMASK 0x1
#define G2D_CFG2_STRIDESIGN_FADDR ADDR_G2D_CFG2
#define G2D_CFG2_STRIDESIGN_FSHIFT 23
#define G2D_CFG2_STRIDESIGN_FMASK 0x1
#define G2D_CFG3_STRIDE_FADDR ADDR_G2D_CFG3
#define G2D_CFG3_STRIDE_FSHIFT 0
#define G2D_CFG3_STRIDE_FMASK 0xfff
#define G2D_CFG3_FORMAT_FADDR ADDR_G2D_CFG3
#define G2D_CFG3_FORMAT_FSHIFT 12
#define G2D_CFG3_FORMAT_FMASK 0xf
#define G2D_CFG3_TILED_FADDR ADDR_G2D_CFG3
#define G2D_CFG3_TILED_FSHIFT 16
#define G2D_CFG3_TILED_FMASK 0x1
#define G2D_CFG3_SRGB_FADDR ADDR_G2D_CFG3
#define G2D_CFG3_SRGB_FSHIFT 17
#define G2D_CFG3_SRGB_FMASK 0x1
#define G2D_CFG3_SWAPWORDS_FADDR ADDR_G2D_CFG3
#define G2D_CFG3_SWAPWORDS_FSHIFT 18
#define G2D_CFG3_SWAPWORDS_FMASK 0x1
#define G2D_CFG3_SWAPBYTES_FADDR ADDR_G2D_CFG3
#define G2D_CFG3_SWAPBYTES_FSHIFT 19
#define G2D_CFG3_SWAPBYTES_FMASK 0x1
#define G2D_CFG3_SWAPALL_FADDR ADDR_G2D_CFG3
#define G2D_CFG3_SWAPALL_FSHIFT 20
#define G2D_CFG3_SWAPALL_FMASK 0x1
#define G2D_CFG3_SWAPRB_FADDR ADDR_G2D_CFG3
#define G2D_CFG3_SWAPRB_FSHIFT 21
#define G2D_CFG3_SWAPRB_FMASK 0x1
#define G2D_CFG3_SWAPBITS_FADDR ADDR_G2D_CFG3
#define G2D_CFG3_SWAPBITS_FSHIFT 22
#define G2D_CFG3_SWAPBITS_FMASK 0x1
#define G2D_CFG3_STRIDESIGN_FADDR ADDR_G2D_CFG3
#define G2D_CFG3_STRIDESIGN_FSHIFT 23
#define G2D_CFG3_STRIDESIGN_FMASK 0x1
#define G2D_COLOR_ARGB_FADDR ADDR_G2D_COLOR
#define G2D_COLOR_ARGB_FSHIFT 0
#define G2D_COLOR_ARGB_FMASK 0xffffffff
#define G2D_CONFIG_DST_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_DST_FSHIFT 0
#define G2D_CONFIG_DST_FMASK 0x1
#define G2D_CONFIG_SRC1_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_SRC1_FSHIFT 1
#define G2D_CONFIG_SRC1_FMASK 0x1
#define G2D_CONFIG_SRC2_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_SRC2_FSHIFT 2
#define G2D_CONFIG_SRC2_FMASK 0x1
#define G2D_CONFIG_SRC3_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_SRC3_FSHIFT 3
#define G2D_CONFIG_SRC3_FMASK 0x1
#define G2D_CONFIG_SRCCK_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_SRCCK_FSHIFT 4
#define G2D_CONFIG_SRCCK_FMASK 0x1
#define G2D_CONFIG_DSTCK_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_DSTCK_FSHIFT 5
#define G2D_CONFIG_DSTCK_FMASK 0x1
#define G2D_CONFIG_ROTATE_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_ROTATE_FSHIFT 6
#define G2D_CONFIG_ROTATE_FMASK 0x3
#define G2D_CONFIG_OBS_GAMMA_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_OBS_GAMMA_FSHIFT 8
#define G2D_CONFIG_OBS_GAMMA_FMASK 0x1
#define G2D_CONFIG_IGNORECKALPHA_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_IGNORECKALPHA_FSHIFT 9
#define G2D_CONFIG_IGNORECKALPHA_FMASK 0x1
#define G2D_CONFIG_DITHER_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_DITHER_FSHIFT 10
#define G2D_CONFIG_DITHER_FMASK 0x1
#define G2D_CONFIG_WRITESRGB_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_WRITESRGB_FSHIFT 11
#define G2D_CONFIG_WRITESRGB_FMASK 0x1
#define G2D_CONFIG_ARGBMASK_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_ARGBMASK_FSHIFT 12
#define G2D_CONFIG_ARGBMASK_FMASK 0xf
#define G2D_CONFIG_ALPHATEX_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_ALPHATEX_FSHIFT 16
#define G2D_CONFIG_ALPHATEX_FMASK 0x1
#define G2D_CONFIG_PALMLINES_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_PALMLINES_FSHIFT 17
#define G2D_CONFIG_PALMLINES_FMASK 0x1
#define G2D_CONFIG_NOLASTPIXEL_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_NOLASTPIXEL_FSHIFT 18
#define G2D_CONFIG_NOLASTPIXEL_FMASK 0x1
#define G2D_CONFIG_NOPROTECT_FADDR ADDR_G2D_CONFIG
#define G2D_CONFIG_NOPROTECT_FSHIFT 19
#define G2D_CONFIG_NOPROTECT_FMASK 0x1
#define G2D_CONST0_ARGB_FADDR ADDR_G2D_CONST0
#define G2D_CONST0_ARGB_FSHIFT 0
#define G2D_CONST0_ARGB_FMASK 0xffffffff
#define G2D_CONST1_ARGB_FADDR ADDR_G2D_CONST1
#define G2D_CONST1_ARGB_FSHIFT 0
#define G2D_CONST1_ARGB_FMASK 0xffffffff
#define G2D_CONST2_ARGB_FADDR ADDR_G2D_CONST2
#define G2D_CONST2_ARGB_FSHIFT 0
#define G2D_CONST2_ARGB_FMASK 0xffffffff
#define G2D_CONST3_ARGB_FADDR ADDR_G2D_CONST3
#define G2D_CONST3_ARGB_FSHIFT 0
#define G2D_CONST3_ARGB_FMASK 0xffffffff
#define G2D_CONST4_ARGB_FADDR ADDR_G2D_CONST4
#define G2D_CONST4_ARGB_FSHIFT 0
#define G2D_CONST4_ARGB_FMASK 0xffffffff
#define G2D_CONST5_ARGB_FADDR ADDR_G2D_CONST5
#define G2D_CONST5_ARGB_FSHIFT 0
#define G2D_CONST5_ARGB_FMASK 0xffffffff
#define G2D_CONST6_ARGB_FADDR ADDR_G2D_CONST6
#define G2D_CONST6_ARGB_FSHIFT 0
#define G2D_CONST6_ARGB_FMASK 0xffffffff
#define G2D_CONST7_ARGB_FADDR ADDR_G2D_CONST7
#define G2D_CONST7_ARGB_FSHIFT 0
#define G2D_CONST7_ARGB_FMASK 0xffffffff
#define G2D_FOREGROUND_COLOR_FADDR ADDR_G2D_FOREGROUND
#define G2D_FOREGROUND_COLOR_FSHIFT 0
#define G2D_FOREGROUND_COLOR_FMASK 0xffffffff
#define G2D_GRADIENT_INSTRUCTIONS_FADDR ADDR_G2D_GRADIENT
#define G2D_GRADIENT_INSTRUCTIONS_FSHIFT 0
#define G2D_GRADIENT_INSTRUCTIONS_FMASK 0x7
#define G2D_GRADIENT_INSTRUCTIONS2_FADDR ADDR_G2D_GRADIENT
#define G2D_GRADIENT_INSTRUCTIONS2_FSHIFT 3
#define G2D_GRADIENT_INSTRUCTIONS2_FMASK 0x7
#define G2D_GRADIENT_ENABLE_FADDR ADDR_G2D_GRADIENT
#define G2D_GRADIENT_ENABLE_FSHIFT 6
#define G2D_GRADIENT_ENABLE_FMASK 0x1
#define G2D_GRADIENT_ENABLE2_FADDR ADDR_G2D_GRADIENT
#define G2D_GRADIENT_ENABLE2_FSHIFT 7
#define G2D_GRADIENT_ENABLE2_FMASK 0x1
#define G2D_GRADIENT_SEL_FADDR ADDR_G2D_GRADIENT
#define G2D_GRADIENT_SEL_FSHIFT 8
#define G2D_GRADIENT_SEL_FMASK 0x1
#define G2D_IDLE_IRQ_FADDR ADDR_G2D_IDLE
#define G2D_IDLE_IRQ_FSHIFT 0
#define G2D_IDLE_IRQ_FMASK 0x1
#define G2D_IDLE_BCFLUSH_FADDR ADDR_G2D_IDLE
#define G2D_IDLE_BCFLUSH_FSHIFT 1
#define G2D_IDLE_BCFLUSH_FMASK 0x1
#define G2D_IDLE_V3_FADDR ADDR_G2D_IDLE
#define G2D_IDLE_V3_FSHIFT 2
#define G2D_IDLE_V3_FMASK 0x1
#define G2D_INPUT_COLOR_FADDR ADDR_G2D_INPUT
#define G2D_INPUT_COLOR_FSHIFT 0
#define G2D_INPUT_COLOR_FMASK 0x1
#define G2D_INPUT_SCOORD1_FADDR ADDR_G2D_INPUT
#define G2D_INPUT_SCOORD1_FSHIFT 1
#define G2D_INPUT_SCOORD1_FMASK 0x1
#define G2D_INPUT_SCOORD2_FADDR ADDR_G2D_INPUT
#define G2D_INPUT_SCOORD2_FSHIFT 2
#define G2D_INPUT_SCOORD2_FMASK 0x1
#define G2D_INPUT_COPYCOORD_FADDR ADDR_G2D_INPUT
#define G2D_INPUT_COPYCOORD_FSHIFT 3
#define G2D_INPUT_COPYCOORD_FMASK 0x1
#define G2D_INPUT_VGMODE_FADDR ADDR_G2D_INPUT
#define G2D_INPUT_VGMODE_FSHIFT 4
#define G2D_INPUT_VGMODE_FMASK 0x1
#define G2D_INPUT_LINEMODE_FADDR ADDR_G2D_INPUT
#define G2D_INPUT_LINEMODE_FSHIFT 5
#define G2D_INPUT_LINEMODE_FMASK 0x1
#define G2D_MASK_YMASK_FADDR ADDR_G2D_MASK
#define G2D_MASK_YMASK_FSHIFT 0
#define G2D_MASK_YMASK_FMASK 0xfff
#define G2D_MASK_XMASK_FADDR ADDR_G2D_MASK
#define G2D_MASK_XMASK_FSHIFT 12
#define G2D_MASK_XMASK_FMASK 0xfff
#define G2D_ROP_ROP_FADDR ADDR_G2D_ROP
#define G2D_ROP_ROP_FSHIFT 0
#define G2D_ROP_ROP_FMASK 0xffff
#define G2D_SCISSORX_LEFT_FADDR ADDR_G2D_SCISSORX
#define G2D_SCISSORX_LEFT_FSHIFT 0
#define G2D_SCISSORX_LEFT_FMASK 0x7ff
#define G2D_SCISSORX_RIGHT_FADDR ADDR_G2D_SCISSORX
#define G2D_SCISSORX_RIGHT_FSHIFT 11
#define G2D_SCISSORX_RIGHT_FMASK 0x7ff
#define G2D_SCISSORY_TOP_FADDR ADDR_G2D_SCISSORY
#define G2D_SCISSORY_TOP_FSHIFT 0
#define G2D_SCISSORY_TOP_FMASK 0x7ff
#define G2D_SCISSORY_BOTTOM_FADDR ADDR_G2D_SCISSORY
#define G2D_SCISSORY_BOTTOM_FSHIFT 11
#define G2D_SCISSORY_BOTTOM_FMASK 0x7ff
#define G2D_SXY_Y_FADDR ADDR_G2D_SXY
#define G2D_SXY_Y_FSHIFT 0
#define G2D_SXY_Y_FMASK 0x7ff
#define G2D_SXY_PAD_FADDR ADDR_G2D_SXY
#define G2D_SXY_PAD_FSHIFT 11
#define G2D_SXY_PAD_FMASK 0x1f
#define G2D_SXY_X_FADDR ADDR_G2D_SXY
#define G2D_SXY_X_FSHIFT 16
#define G2D_SXY_X_FMASK 0x7ff
#define G2D_SXY2_Y_FADDR ADDR_G2D_SXY2
#define G2D_SXY2_Y_FSHIFT 0
#define G2D_SXY2_Y_FMASK 0x7ff
#define G2D_SXY2_PAD_FADDR ADDR_G2D_SXY2
#define G2D_SXY2_PAD_FSHIFT 11
#define G2D_SXY2_PAD_FMASK 0x1f
#define G2D_SXY2_X_FADDR ADDR_G2D_SXY2
#define G2D_SXY2_X_FSHIFT 16
#define G2D_SXY2_X_FMASK 0x7ff
#define G2D_VGSPAN_WIDTH_FADDR ADDR_G2D_VGSPAN
#define G2D_VGSPAN_WIDTH_FSHIFT 0
#define G2D_VGSPAN_WIDTH_FMASK 0xfff
#define G2D_VGSPAN_PAD_FADDR ADDR_G2D_VGSPAN
#define G2D_VGSPAN_PAD_FSHIFT 12
#define G2D_VGSPAN_PAD_FMASK 0xf
#define G2D_VGSPAN_COVERAGE_FADDR ADDR_G2D_VGSPAN
#define G2D_VGSPAN_COVERAGE_FSHIFT 16
#define G2D_VGSPAN_COVERAGE_FMASK 0xf
#define G2D_WIDTHHEIGHT_HEIGHT_FADDR ADDR_G2D_WIDTHHEIGHT
#define G2D_WIDTHHEIGHT_HEIGHT_FSHIFT 0
#define G2D_WIDTHHEIGHT_HEIGHT_FMASK 0xfff
#define G2D_WIDTHHEIGHT_PAD_FADDR ADDR_G2D_WIDTHHEIGHT
#define G2D_WIDTHHEIGHT_PAD_FSHIFT 12
#define G2D_WIDTHHEIGHT_PAD_FMASK 0xf
#define G2D_WIDTHHEIGHT_WIDTH_FADDR ADDR_G2D_WIDTHHEIGHT
#define G2D_WIDTHHEIGHT_WIDTH_FSHIFT 16
#define G2D_WIDTHHEIGHT_WIDTH_FMASK 0xfff
#define G2D_XY_Y_FADDR ADDR_G2D_XY
#define G2D_XY_Y_FSHIFT 0
#define G2D_XY_Y_FMASK 0xfff
#define G2D_XY_PAD_FADDR ADDR_G2D_XY
#define G2D_XY_PAD_FSHIFT 12
#define G2D_XY_PAD_FMASK 0xf
#define G2D_XY_X_FADDR ADDR_G2D_XY
#define G2D_XY_X_FSHIFT 16
#define G2D_XY_X_FMASK 0xfff
#define GRADW_BORDERCOLOR_COLOR_FADDR ADDR_GRADW_BORDERCOLOR
#define GRADW_BORDERCOLOR_COLOR_FSHIFT 0
#define GRADW_BORDERCOLOR_COLOR_FMASK 0xffffffff
#define GRADW_CONST0_VALUE_FADDR ADDR_GRADW_CONST0
#define GRADW_CONST0_VALUE_FSHIFT 0
#define GRADW_CONST0_VALUE_FMASK 0xffff
#define GRADW_CONST1_VALUE_FADDR ADDR_GRADW_CONST1
#define GRADW_CONST1_VALUE_FSHIFT 0
#define GRADW_CONST1_VALUE_FMASK 0xffff
#define GRADW_CONST2_VALUE_FADDR ADDR_GRADW_CONST2
#define GRADW_CONST2_VALUE_FSHIFT 0
#define GRADW_CONST2_VALUE_FMASK 0xffff
#define GRADW_CONST3_VALUE_FADDR ADDR_GRADW_CONST3
#define GRADW_CONST3_VALUE_FSHIFT 0
#define GRADW_CONST3_VALUE_FMASK 0xffff
#define GRADW_CONST4_VALUE_FADDR ADDR_GRADW_CONST4
#define GRADW_CONST4_VALUE_FSHIFT 0
#define GRADW_CONST4_VALUE_FMASK 0xffff
#define GRADW_CONST5_VALUE_FADDR ADDR_GRADW_CONST5
#define GRADW_CONST5_VALUE_FSHIFT 0
#define GRADW_CONST5_VALUE_FMASK 0xffff
#define GRADW_CONST6_VALUE_FADDR ADDR_GRADW_CONST6
#define GRADW_CONST6_VALUE_FSHIFT 0
#define GRADW_CONST6_VALUE_FMASK 0xffff
#define GRADW_CONST7_VALUE_FADDR ADDR_GRADW_CONST7
#define GRADW_CONST7_VALUE_FSHIFT 0
#define GRADW_CONST7_VALUE_FMASK 0xffff
#define GRADW_CONST8_VALUE_FADDR ADDR_GRADW_CONST8
#define GRADW_CONST8_VALUE_FSHIFT 0
#define GRADW_CONST8_VALUE_FMASK 0xffff
#define GRADW_CONST9_VALUE_FADDR ADDR_GRADW_CONST9
#define GRADW_CONST9_VALUE_FSHIFT 0
#define GRADW_CONST9_VALUE_FMASK 0xffff
#define GRADW_CONSTA_VALUE_FADDR ADDR_GRADW_CONSTA
#define GRADW_CONSTA_VALUE_FSHIFT 0
#define GRADW_CONSTA_VALUE_FMASK 0xffff
#define GRADW_CONSTB_VALUE_FADDR ADDR_GRADW_CONSTB
#define GRADW_CONSTB_VALUE_FSHIFT 0
#define GRADW_CONSTB_VALUE_FMASK 0xffff
#define GRADW_INST0_SRC_E_FADDR ADDR_GRADW_INST0
#define GRADW_INST0_SRC_E_FSHIFT 0
#define GRADW_INST0_SRC_E_FMASK 0x1f
#define GRADW_INST0_SRC_D_FADDR ADDR_GRADW_INST0
#define GRADW_INST0_SRC_D_FSHIFT 5
#define GRADW_INST0_SRC_D_FMASK 0x1f
#define GRADW_INST0_SRC_C_FADDR ADDR_GRADW_INST0
#define GRADW_INST0_SRC_C_FSHIFT 10
#define GRADW_INST0_SRC_C_FMASK 0x1f
#define GRADW_INST0_SRC_B_FADDR ADDR_GRADW_INST0
#define GRADW_INST0_SRC_B_FSHIFT 15
#define GRADW_INST0_SRC_B_FMASK 0x1f
#define GRADW_INST0_SRC_A_FADDR ADDR_GRADW_INST0
#define GRADW_INST0_SRC_A_FSHIFT 20
#define GRADW_INST0_SRC_A_FMASK 0x1f
#define GRADW_INST0_DST_FADDR ADDR_GRADW_INST0
#define GRADW_INST0_DST_FSHIFT 25
#define GRADW_INST0_DST_FMASK 0xf
#define GRADW_INST0_OPCODE_FADDR ADDR_GRADW_INST0
#define GRADW_INST0_OPCODE_FSHIFT 29
#define GRADW_INST0_OPCODE_FMASK 0x3
#define GRADW_INST1_SRC_E_FADDR ADDR_GRADW_INST1
#define GRADW_INST1_SRC_E_FSHIFT 0
#define GRADW_INST1_SRC_E_FMASK 0x1f
#define GRADW_INST1_SRC_D_FADDR ADDR_GRADW_INST1
#define GRADW_INST1_SRC_D_FSHIFT 5
#define GRADW_INST1_SRC_D_FMASK 0x1f
#define GRADW_INST1_SRC_C_FADDR ADDR_GRADW_INST1
#define GRADW_INST1_SRC_C_FSHIFT 10
#define GRADW_INST1_SRC_C_FMASK 0x1f
#define GRADW_INST1_SRC_B_FADDR ADDR_GRADW_INST1
#define GRADW_INST1_SRC_B_FSHIFT 15
#define GRADW_INST1_SRC_B_FMASK 0x1f
#define GRADW_INST1_SRC_A_FADDR ADDR_GRADW_INST1
#define GRADW_INST1_SRC_A_FSHIFT 20
#define GRADW_INST1_SRC_A_FMASK 0x1f
#define GRADW_INST1_DST_FADDR ADDR_GRADW_INST1
#define GRADW_INST1_DST_FSHIFT 25
#define GRADW_INST1_DST_FMASK 0xf
#define GRADW_INST1_OPCODE_FADDR ADDR_GRADW_INST1
#define GRADW_INST1_OPCODE_FSHIFT 29
#define GRADW_INST1_OPCODE_FMASK 0x3
#define GRADW_INST2_SRC_E_FADDR ADDR_GRADW_INST2
#define GRADW_INST2_SRC_E_FSHIFT 0
#define GRADW_INST2_SRC_E_FMASK 0x1f
#define GRADW_INST2_SRC_D_FADDR ADDR_GRADW_INST2
#define GRADW_INST2_SRC_D_FSHIFT 5
#define GRADW_INST2_SRC_D_FMASK 0x1f
#define GRADW_INST2_SRC_C_FADDR ADDR_GRADW_INST2
#define GRADW_INST2_SRC_C_FSHIFT 10
#define GRADW_INST2_SRC_C_FMASK 0x1f
#define GRADW_INST2_SRC_B_FADDR ADDR_GRADW_INST2
#define GRADW_INST2_SRC_B_FSHIFT 15
#define GRADW_INST2_SRC_B_FMASK 0x1f
#define GRADW_INST2_SRC_A_FADDR ADDR_GRADW_INST2
#define GRADW_INST2_SRC_A_FSHIFT 20
#define GRADW_INST2_SRC_A_FMASK 0x1f
#define GRADW_INST2_DST_FADDR ADDR_GRADW_INST2
#define GRADW_INST2_DST_FSHIFT 25
#define GRADW_INST2_DST_FMASK 0xf
#define GRADW_INST2_OPCODE_FADDR ADDR_GRADW_INST2
#define GRADW_INST2_OPCODE_FSHIFT 29
#define GRADW_INST2_OPCODE_FMASK 0x3
#define GRADW_INST3_SRC_E_FADDR ADDR_GRADW_INST3
#define GRADW_INST3_SRC_E_FSHIFT 0
#define GRADW_INST3_SRC_E_FMASK 0x1f
#define GRADW_INST3_SRC_D_FADDR ADDR_GRADW_INST3
#define GRADW_INST3_SRC_D_FSHIFT 5
#define GRADW_INST3_SRC_D_FMASK 0x1f
#define GRADW_INST3_SRC_C_FADDR ADDR_GRADW_INST3
#define GRADW_INST3_SRC_C_FSHIFT 10
#define GRADW_INST3_SRC_C_FMASK 0x1f
#define GRADW_INST3_SRC_B_FADDR ADDR_GRADW_INST3
#define GRADW_INST3_SRC_B_FSHIFT 15
#define GRADW_INST3_SRC_B_FMASK 0x1f
#define GRADW_INST3_SRC_A_FADDR ADDR_GRADW_INST3
#define GRADW_INST3_SRC_A_FSHIFT 20
#define GRADW_INST3_SRC_A_FMASK 0x1f
#define GRADW_INST3_DST_FADDR ADDR_GRADW_INST3
#define GRADW_INST3_DST_FSHIFT 25
#define GRADW_INST3_DST_FMASK 0xf
#define GRADW_INST3_OPCODE_FADDR ADDR_GRADW_INST3
#define GRADW_INST3_OPCODE_FSHIFT 29
#define GRADW_INST3_OPCODE_FMASK 0x3
#define GRADW_INST4_SRC_E_FADDR ADDR_GRADW_INST4
#define GRADW_INST4_SRC_E_FSHIFT 0
#define GRADW_INST4_SRC_E_FMASK 0x1f
#define GRADW_INST4_SRC_D_FADDR ADDR_GRADW_INST4
#define GRADW_INST4_SRC_D_FSHIFT 5
#define GRADW_INST4_SRC_D_FMASK 0x1f
#define GRADW_INST4_SRC_C_FADDR ADDR_GRADW_INST4
#define GRADW_INST4_SRC_C_FSHIFT 10
#define GRADW_INST4_SRC_C_FMASK 0x1f
#define GRADW_INST4_SRC_B_FADDR ADDR_GRADW_INST4
#define GRADW_INST4_SRC_B_FSHIFT 15
#define GRADW_INST4_SRC_B_FMASK 0x1f
#define GRADW_INST4_SRC_A_FADDR ADDR_GRADW_INST4
#define GRADW_INST4_SRC_A_FSHIFT 20
#define GRADW_INST4_SRC_A_FMASK 0x1f
#define GRADW_INST4_DST_FADDR ADDR_GRADW_INST4
#define GRADW_INST4_DST_FSHIFT 25
#define GRADW_INST4_DST_FMASK 0xf
#define GRADW_INST4_OPCODE_FADDR ADDR_GRADW_INST4
#define GRADW_INST4_OPCODE_FSHIFT 29
#define GRADW_INST4_OPCODE_FMASK 0x3
#define GRADW_INST5_SRC_E_FADDR ADDR_GRADW_INST5
#define GRADW_INST5_SRC_E_FSHIFT 0
#define GRADW_INST5_SRC_E_FMASK 0x1f
#define GRADW_INST5_SRC_D_FADDR ADDR_GRADW_INST5
#define GRADW_INST5_SRC_D_FSHIFT 5
#define GRADW_INST5_SRC_D_FMASK 0x1f
#define GRADW_INST5_SRC_C_FADDR ADDR_GRADW_INST5
#define GRADW_INST5_SRC_C_FSHIFT 10
#define GRADW_INST5_SRC_C_FMASK 0x1f
#define GRADW_INST5_SRC_B_FADDR ADDR_GRADW_INST5
#define GRADW_INST5_SRC_B_FSHIFT 15
#define GRADW_INST5_SRC_B_FMASK 0x1f
#define GRADW_INST5_SRC_A_FADDR ADDR_GRADW_INST5
#define GRADW_INST5_SRC_A_FSHIFT 20
#define GRADW_INST5_SRC_A_FMASK 0x1f
#define GRADW_INST5_DST_FADDR ADDR_GRADW_INST5
#define GRADW_INST5_DST_FSHIFT 25
#define GRADW_INST5_DST_FMASK 0xf
#define GRADW_INST5_OPCODE_FADDR ADDR_GRADW_INST5
#define GRADW_INST5_OPCODE_FSHIFT 29
#define GRADW_INST5_OPCODE_FMASK 0x3
#define GRADW_INST6_SRC_E_FADDR ADDR_GRADW_INST6
#define GRADW_INST6_SRC_E_FSHIFT 0
#define GRADW_INST6_SRC_E_FMASK 0x1f
#define GRADW_INST6_SRC_D_FADDR ADDR_GRADW_INST6
#define GRADW_INST6_SRC_D_FSHIFT 5
#define GRADW_INST6_SRC_D_FMASK 0x1f
#define GRADW_INST6_SRC_C_FADDR ADDR_GRADW_INST6
#define GRADW_INST6_SRC_C_FSHIFT 10
#define GRADW_INST6_SRC_C_FMASK 0x1f
#define GRADW_INST6_SRC_B_FADDR ADDR_GRADW_INST6
#define GRADW_INST6_SRC_B_FSHIFT 15
#define GRADW_INST6_SRC_B_FMASK 0x1f
#define GRADW_INST6_SRC_A_FADDR ADDR_GRADW_INST6
#define GRADW_INST6_SRC_A_FSHIFT 20
#define GRADW_INST6_SRC_A_FMASK 0x1f
#define GRADW_INST6_DST_FADDR ADDR_GRADW_INST6
#define GRADW_INST6_DST_FSHIFT 25
#define GRADW_INST6_DST_FMASK 0xf
#define GRADW_INST6_OPCODE_FADDR ADDR_GRADW_INST6
#define GRADW_INST6_OPCODE_FSHIFT 29
#define GRADW_INST6_OPCODE_FMASK 0x3
#define GRADW_INST7_SRC_E_FADDR ADDR_GRADW_INST7
#define GRADW_INST7_SRC_E_FSHIFT 0
#define GRADW_INST7_SRC_E_FMASK 0x1f
#define GRADW_INST7_SRC_D_FADDR ADDR_GRADW_INST7
#define GRADW_INST7_SRC_D_FSHIFT 5
#define GRADW_INST7_SRC_D_FMASK 0x1f
#define GRADW_INST7_SRC_C_FADDR ADDR_GRADW_INST7
#define GRADW_INST7_SRC_C_FSHIFT 10
#define GRADW_INST7_SRC_C_FMASK 0x1f
#define GRADW_INST7_SRC_B_FADDR ADDR_GRADW_INST7
#define GRADW_INST7_SRC_B_FSHIFT 15
#define GRADW_INST7_SRC_B_FMASK 0x1f
#define GRADW_INST7_SRC_A_FADDR ADDR_GRADW_INST7
#define GRADW_INST7_SRC_A_FSHIFT 20
#define GRADW_INST7_SRC_A_FMASK 0x1f
#define GRADW_INST7_DST_FADDR ADDR_GRADW_INST7
#define GRADW_INST7_DST_FSHIFT 25
#define GRADW_INST7_DST_FMASK 0xf
#define GRADW_INST7_OPCODE_FADDR ADDR_GRADW_INST7
#define GRADW_INST7_OPCODE_FSHIFT 29
#define GRADW_INST7_OPCODE_FMASK 0x3
#define GRADW_TEXBASE_ADDR_FADDR ADDR_GRADW_TEXBASE
#define GRADW_TEXBASE_ADDR_FSHIFT 0
#define GRADW_TEXBASE_ADDR_FMASK 0xffffffff
#define GRADW_TEXCFG_STRIDE_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_STRIDE_FSHIFT 0
#define GRADW_TEXCFG_STRIDE_FMASK 0xfff
#define GRADW_TEXCFG_FORMAT_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_FORMAT_FSHIFT 12
#define GRADW_TEXCFG_FORMAT_FMASK 0xf
#define GRADW_TEXCFG_TILED_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_TILED_FSHIFT 16
#define GRADW_TEXCFG_TILED_FMASK 0x1
#define GRADW_TEXCFG_WRAPU_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_WRAPU_FSHIFT 17
#define GRADW_TEXCFG_WRAPU_FMASK 0x3
#define GRADW_TEXCFG_WRAPV_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_WRAPV_FSHIFT 19
#define GRADW_TEXCFG_WRAPV_FMASK 0x3
#define GRADW_TEXCFG_BILIN_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_BILIN_FSHIFT 21
#define GRADW_TEXCFG_BILIN_FMASK 0x1
#define GRADW_TEXCFG_SRGB_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_SRGB_FSHIFT 22
#define GRADW_TEXCFG_SRGB_FMASK 0x1
#define GRADW_TEXCFG_PREMULTIPLY_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_PREMULTIPLY_FSHIFT 23
#define GRADW_TEXCFG_PREMULTIPLY_FMASK 0x1
#define GRADW_TEXCFG_SWAPWORDS_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_SWAPWORDS_FSHIFT 24
#define GRADW_TEXCFG_SWAPWORDS_FMASK 0x1
#define GRADW_TEXCFG_SWAPBYTES_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_SWAPBYTES_FSHIFT 25
#define GRADW_TEXCFG_SWAPBYTES_FMASK 0x1
#define GRADW_TEXCFG_SWAPALL_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_SWAPALL_FSHIFT 26
#define GRADW_TEXCFG_SWAPALL_FMASK 0x1
#define GRADW_TEXCFG_SWAPRB_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_SWAPRB_FSHIFT 27
#define GRADW_TEXCFG_SWAPRB_FMASK 0x1
#define GRADW_TEXCFG_TEX2D_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_TEX2D_FSHIFT 28
#define GRADW_TEXCFG_TEX2D_FMASK 0x1
#define GRADW_TEXCFG_SWAPBITS_FADDR ADDR_GRADW_TEXCFG
#define GRADW_TEXCFG_SWAPBITS_FSHIFT 29
#define GRADW_TEXCFG_SWAPBITS_FMASK 0x1
#define GRADW_TEXSIZE_WIDTH_FADDR ADDR_GRADW_TEXSIZE
#define GRADW_TEXSIZE_WIDTH_FSHIFT 0
#define GRADW_TEXSIZE_WIDTH_FMASK 0x7ff
#define GRADW_TEXSIZE_HEIGHT_FADDR ADDR_GRADW_TEXSIZE
#define GRADW_TEXSIZE_HEIGHT_FSHIFT 11
#define GRADW_TEXSIZE_HEIGHT_FMASK 0x7ff
#define MH_ARBITER_CONFIG_SAME_PAGE_LIMIT_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_SAME_PAGE_LIMIT_FSHIFT 0
#define MH_ARBITER_CONFIG_SAME_PAGE_LIMIT_FMASK 0x3f
#define MH_ARBITER_CONFIG_SAME_PAGE_GRANULARITY_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_SAME_PAGE_GRANULARITY_FSHIFT 6
#define MH_ARBITER_CONFIG_SAME_PAGE_GRANULARITY_FMASK 0x1
#define MH_ARBITER_CONFIG_L1_ARB_ENABLE_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_L1_ARB_ENABLE_FSHIFT 7
#define MH_ARBITER_CONFIG_L1_ARB_ENABLE_FMASK 0x1
#define MH_ARBITER_CONFIG_L1_ARB_HOLD_ENABLE_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_L1_ARB_HOLD_ENABLE_FSHIFT 8
#define MH_ARBITER_CONFIG_L1_ARB_HOLD_ENABLE_FMASK 0x1
#define MH_ARBITER_CONFIG_L2_ARB_CONTROL_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_L2_ARB_CONTROL_FSHIFT 9
#define MH_ARBITER_CONFIG_L2_ARB_CONTROL_FMASK 0x1
#define MH_ARBITER_CONFIG_PAGE_SIZE_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_PAGE_SIZE_FSHIFT 10
#define MH_ARBITER_CONFIG_PAGE_SIZE_FMASK 0x7
#define MH_ARBITER_CONFIG_TC_REORDER_ENABLE_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_TC_REORDER_ENABLE_FSHIFT 13
#define MH_ARBITER_CONFIG_TC_REORDER_ENABLE_FMASK 0x1
#define MH_ARBITER_CONFIG_TC_ARB_HOLD_ENABLE_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_TC_ARB_HOLD_ENABLE_FSHIFT 14
#define MH_ARBITER_CONFIG_TC_ARB_HOLD_ENABLE_FMASK 0x1
#define MH_ARBITER_CONFIG_IN_FLIGHT_LIMIT_ENABLE_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_IN_FLIGHT_LIMIT_ENABLE_FSHIFT 15
#define MH_ARBITER_CONFIG_IN_FLIGHT_LIMIT_ENABLE_FMASK 0x1
#define MH_ARBITER_CONFIG_IN_FLIGHT_LIMIT_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_IN_FLIGHT_LIMIT_FSHIFT 16
#define MH_ARBITER_CONFIG_IN_FLIGHT_LIMIT_FMASK 0x3f
#define MH_ARBITER_CONFIG_CP_CLNT_ENABLE_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_CP_CLNT_ENABLE_FSHIFT 22
#define MH_ARBITER_CONFIG_CP_CLNT_ENABLE_FMASK 0x1
#define MH_ARBITER_CONFIG_VGT_CLNT_ENABLE_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_VGT_CLNT_ENABLE_FSHIFT 23
#define MH_ARBITER_CONFIG_VGT_CLNT_ENABLE_FMASK 0x1
#define MH_ARBITER_CONFIG_TC_CLNT_ENABLE_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_TC_CLNT_ENABLE_FSHIFT 24
#define MH_ARBITER_CONFIG_TC_CLNT_ENABLE_FMASK 0x1
#define MH_ARBITER_CONFIG_RB_CLNT_ENABLE_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_RB_CLNT_ENABLE_FSHIFT 25
#define MH_ARBITER_CONFIG_RB_CLNT_ENABLE_FMASK 0x1
#define MH_ARBITER_CONFIG_PA_CLNT_ENABLE_FADDR ADDR_MH_ARBITER_CONFIG
#define MH_ARBITER_CONFIG_PA_CLNT_ENABLE_FSHIFT 26
#define MH_ARBITER_CONFIG_PA_CLNT_ENABLE_FMASK 0x1
#define MH_AXI_ERROR_AXI_READ_ID_FADDR ADDR_MH_AXI_ERROR
#define MH_AXI_ERROR_AXI_READ_ID_FSHIFT 0
#define MH_AXI_ERROR_AXI_READ_ID_FMASK 0x7
#define MH_AXI_ERROR_AXI_READ_ERROR_FADDR ADDR_MH_AXI_ERROR
#define MH_AXI_ERROR_AXI_READ_ERROR_FSHIFT 3
#define MH_AXI_ERROR_AXI_READ_ERROR_FMASK 0x1
#define MH_AXI_ERROR_AXI_WRITE_ID_FADDR ADDR_MH_AXI_ERROR
#define MH_AXI_ERROR_AXI_WRITE_ID_FSHIFT 4
#define MH_AXI_ERROR_AXI_WRITE_ID_FMASK 0x7
#define MH_AXI_ERROR_AXI_WRITE_ERROR_FADDR ADDR_MH_AXI_ERROR
#define MH_AXI_ERROR_AXI_WRITE_ERROR_FSHIFT 7
#define MH_AXI_ERROR_AXI_WRITE_ERROR_FMASK 0x1
#define MH_AXI_HALT_CONTROL_AXI_HALT_FADDR ADDR_MH_AXI_HALT_CONTROL
#define MH_AXI_HALT_CONTROL_AXI_HALT_FSHIFT 0
#define MH_AXI_HALT_CONTROL_AXI_HALT_FMASK 0x1
#define MH_CLNT_AXI_ID_REUSE_CPW_ID_FADDR ADDR_MH_CLNT_AXI_ID_REUSE
#define MH_CLNT_AXI_ID_REUSE_CPW_ID_FSHIFT 0
#define MH_CLNT_AXI_ID_REUSE_CPW_ID_FMASK 0x7
#define MH_CLNT_AXI_ID_REUSE_PAD_FADDR ADDR_MH_CLNT_AXI_ID_REUSE
#define MH_CLNT_AXI_ID_REUSE_PAD_FSHIFT 3
#define MH_CLNT_AXI_ID_REUSE_PAD_FMASK 0x1
#define MH_CLNT_AXI_ID_REUSE_RBW_ID_FADDR ADDR_MH_CLNT_AXI_ID_REUSE
#define MH_CLNT_AXI_ID_REUSE_RBW_ID_FSHIFT 4
#define MH_CLNT_AXI_ID_REUSE_RBW_ID_FMASK 0x7
#define MH_CLNT_AXI_ID_REUSE_PAD2_FADDR ADDR_MH_CLNT_AXI_ID_REUSE
#define MH_CLNT_AXI_ID_REUSE_PAD2_FSHIFT 7
#define MH_CLNT_AXI_ID_REUSE_PAD2_FMASK 0x1
#define MH_CLNT_AXI_ID_REUSE_MMUR_ID_FADDR ADDR_MH_CLNT_AXI_ID_REUSE
#define MH_CLNT_AXI_ID_REUSE_MMUR_ID_FSHIFT 8
#define MH_CLNT_AXI_ID_REUSE_MMUR_ID_FMASK 0x7
#define MH_CLNT_AXI_ID_REUSE_PAD3_FADDR ADDR_MH_CLNT_AXI_ID_REUSE
#define MH_CLNT_AXI_ID_REUSE_PAD3_FSHIFT 11
#define MH_CLNT_AXI_ID_REUSE_PAD3_FMASK 0x1
#define MH_CLNT_AXI_ID_REUSE_PAW_ID_FADDR ADDR_MH_CLNT_AXI_ID_REUSE
#define MH_CLNT_AXI_ID_REUSE_PAW_ID_FSHIFT 12
#define MH_CLNT_AXI_ID_REUSE_PAW_ID_FMASK 0x7
#define MH_DEBUG_CTRL_INDEX_FADDR ADDR_MH_DEBUG_CTRL
#define MH_DEBUG_CTRL_INDEX_FSHIFT 0
#define MH_DEBUG_CTRL_INDEX_FMASK 0x3f
#define MH_DEBUG_DATA_DATA_FADDR ADDR_MH_DEBUG_DATA
#define MH_DEBUG_DATA_DATA_FSHIFT 0
#define MH_DEBUG_DATA_DATA_FMASK 0xffffffff
#define MH_INTERRUPT_CLEAR_AXI_READ_ERROR_FADDR ADDR_MH_INTERRUPT_CLEAR
#define MH_INTERRUPT_CLEAR_AXI_READ_ERROR_FSHIFT 0
#define MH_INTERRUPT_CLEAR_AXI_READ_ERROR_FMASK 0x1
#define MH_INTERRUPT_CLEAR_AXI_WRITE_ERROR_FADDR ADDR_MH_INTERRUPT_CLEAR
#define MH_INTERRUPT_CLEAR_AXI_WRITE_ERROR_FSHIFT 1
#define MH_INTERRUPT_CLEAR_AXI_WRITE_ERROR_FMASK 0x1
#define MH_INTERRUPT_CLEAR_MMU_PAGE_FAULT_FADDR ADDR_MH_INTERRUPT_CLEAR
#define MH_INTERRUPT_CLEAR_MMU_PAGE_FAULT_FSHIFT 2
#define MH_INTERRUPT_CLEAR_MMU_PAGE_FAULT_FMASK 0x1
#define MH_INTERRUPT_MASK_AXI_READ_ERROR_FADDR ADDR_MH_INTERRUPT_MASK
#define MH_INTERRUPT_MASK_AXI_READ_ERROR_FSHIFT 0
#define MH_INTERRUPT_MASK_AXI_READ_ERROR_FMASK 0x1
#define MH_INTERRUPT_MASK_AXI_WRITE_ERROR_FADDR ADDR_MH_INTERRUPT_MASK
#define MH_INTERRUPT_MASK_AXI_WRITE_ERROR_FSHIFT 1
#define MH_INTERRUPT_MASK_AXI_WRITE_ERROR_FMASK 0x1
#define MH_INTERRUPT_MASK_MMU_PAGE_FAULT_FADDR ADDR_MH_INTERRUPT_MASK
#define MH_INTERRUPT_MASK_MMU_PAGE_FAULT_FSHIFT 2
#define MH_INTERRUPT_MASK_MMU_PAGE_FAULT_FMASK 0x1
#define MH_INTERRUPT_STATUS_AXI_READ_ERROR_FADDR ADDR_MH_INTERRUPT_STATUS
#define MH_INTERRUPT_STATUS_AXI_READ_ERROR_FSHIFT 0
#define MH_INTERRUPT_STATUS_AXI_READ_ERROR_FMASK 0x1
#define MH_INTERRUPT_STATUS_AXI_WRITE_ERROR_FADDR ADDR_MH_INTERRUPT_STATUS
#define MH_INTERRUPT_STATUS_AXI_WRITE_ERROR_FSHIFT 1
#define MH_INTERRUPT_STATUS_AXI_WRITE_ERROR_FMASK 0x1
#define MH_INTERRUPT_STATUS_MMU_PAGE_FAULT_FADDR ADDR_MH_INTERRUPT_STATUS
#define MH_INTERRUPT_STATUS_MMU_PAGE_FAULT_FSHIFT 2
#define MH_INTERRUPT_STATUS_MMU_PAGE_FAULT_FMASK 0x1
#define MH_MMU_CONFIG_MMU_ENABLE_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_MMU_ENABLE_FSHIFT 0
#define MH_MMU_CONFIG_MMU_ENABLE_FMASK 0x1
#define MH_MMU_CONFIG_SPLIT_MODE_ENABLE_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_SPLIT_MODE_ENABLE_FSHIFT 1
#define MH_MMU_CONFIG_SPLIT_MODE_ENABLE_FMASK 0x1
#define MH_MMU_CONFIG_PAD_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_PAD_FSHIFT 2
#define MH_MMU_CONFIG_PAD_FMASK 0x3
#define MH_MMU_CONFIG_RB_W_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_RB_W_CLNT_BEHAVIOR_FSHIFT 4
#define MH_MMU_CONFIG_RB_W_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_CONFIG_CP_W_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_CP_W_CLNT_BEHAVIOR_FSHIFT 6
#define MH_MMU_CONFIG_CP_W_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_CONFIG_CP_R0_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_CP_R0_CLNT_BEHAVIOR_FSHIFT 8
#define MH_MMU_CONFIG_CP_R0_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_CONFIG_CP_R1_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_CP_R1_CLNT_BEHAVIOR_FSHIFT 10
#define MH_MMU_CONFIG_CP_R1_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_CONFIG_CP_R2_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_CP_R2_CLNT_BEHAVIOR_FSHIFT 12
#define MH_MMU_CONFIG_CP_R2_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_CONFIG_CP_R3_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_CP_R3_CLNT_BEHAVIOR_FSHIFT 14
#define MH_MMU_CONFIG_CP_R3_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_CONFIG_CP_R4_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_CP_R4_CLNT_BEHAVIOR_FSHIFT 16
#define MH_MMU_CONFIG_CP_R4_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_CONFIG_VGT_R0_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_VGT_R0_CLNT_BEHAVIOR_FSHIFT 18
#define MH_MMU_CONFIG_VGT_R0_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_CONFIG_VGT_R1_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_VGT_R1_CLNT_BEHAVIOR_FSHIFT 20
#define MH_MMU_CONFIG_VGT_R1_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_CONFIG_TC_R_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_TC_R_CLNT_BEHAVIOR_FSHIFT 22
#define MH_MMU_CONFIG_TC_R_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_CONFIG_PA_W_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_CONFIG
#define MH_MMU_CONFIG_PA_W_CLNT_BEHAVIOR_FSHIFT 24
#define MH_MMU_CONFIG_PA_W_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_INVALIDATE_INVALIDATE_ALL_FADDR ADDR_MH_MMU_INVALIDATE
#define MH_MMU_INVALIDATE_INVALIDATE_ALL_FSHIFT 0
#define MH_MMU_INVALIDATE_INVALIDATE_ALL_FMASK 0x1
#define MH_MMU_INVALIDATE_INVALIDATE_TC_FADDR ADDR_MH_MMU_INVALIDATE
#define MH_MMU_INVALIDATE_INVALIDATE_TC_FSHIFT 1
#define MH_MMU_INVALIDATE_INVALIDATE_TC_FMASK 0x1
#define MH_MMU_MPU_BASE_ZERO_FADDR ADDR_MH_MMU_MPU_BASE
#define MH_MMU_MPU_BASE_ZERO_FSHIFT 0
#define MH_MMU_MPU_BASE_ZERO_FMASK 0xfff
#define MH_MMU_MPU_BASE_MPU_BASE_FADDR ADDR_MH_MMU_MPU_BASE
#define MH_MMU_MPU_BASE_MPU_BASE_FSHIFT 12
#define MH_MMU_MPU_BASE_MPU_BASE_FMASK 0xfffff
#define MH_MMU_MPU_END_ZERO_FADDR ADDR_MH_MMU_MPU_END
#define MH_MMU_MPU_END_ZERO_FSHIFT 0
#define MH_MMU_MPU_END_ZERO_FMASK 0xfff
#define MH_MMU_MPU_END_MPU_END_FADDR ADDR_MH_MMU_MPU_END
#define MH_MMU_MPU_END_MPU_END_FSHIFT 12
#define MH_MMU_MPU_END_MPU_END_FMASK 0xfffff
#define MH_MMU_PAGE_FAULT_PAGE_FAULT_FADDR ADDR_MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT_PAGE_FAULT_FSHIFT 0
#define MH_MMU_PAGE_FAULT_PAGE_FAULT_FMASK 0x1
#define MH_MMU_PAGE_FAULT_OP_TYPE_FADDR ADDR_MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT_OP_TYPE_FSHIFT 1
#define MH_MMU_PAGE_FAULT_OP_TYPE_FMASK 0x1
#define MH_MMU_PAGE_FAULT_CLNT_BEHAVIOR_FADDR ADDR_MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT_CLNT_BEHAVIOR_FSHIFT 2
#define MH_MMU_PAGE_FAULT_CLNT_BEHAVIOR_FMASK 0x3
#define MH_MMU_PAGE_FAULT_AXI_ID_FADDR ADDR_MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT_AXI_ID_FSHIFT 4
#define MH_MMU_PAGE_FAULT_AXI_ID_FMASK 0x7
#define MH_MMU_PAGE_FAULT_PAD_FADDR ADDR_MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT_PAD_FSHIFT 7
#define MH_MMU_PAGE_FAULT_PAD_FMASK 0x1
#define MH_MMU_PAGE_FAULT_MPU_ADDRESS_OUT_OF_RANGE_FADDR ADDR_MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT_MPU_ADDRESS_OUT_OF_RANGE_FSHIFT 8
#define MH_MMU_PAGE_FAULT_MPU_ADDRESS_OUT_OF_RANGE_FMASK 0x1
#define MH_MMU_PAGE_FAULT_ADDRESS_OUT_OF_RANGE_FADDR ADDR_MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT_ADDRESS_OUT_OF_RANGE_FSHIFT 9
#define MH_MMU_PAGE_FAULT_ADDRESS_OUT_OF_RANGE_FMASK 0x1
#define MH_MMU_PAGE_FAULT_READ_PROTECTION_ERROR_FADDR ADDR_MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT_READ_PROTECTION_ERROR_FSHIFT 10
#define MH_MMU_PAGE_FAULT_READ_PROTECTION_ERROR_FMASK 0x1
#define MH_MMU_PAGE_FAULT_WRITE_PROTECTION_ERROR_FADDR ADDR_MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT_WRITE_PROTECTION_ERROR_FSHIFT 11
#define MH_MMU_PAGE_FAULT_WRITE_PROTECTION_ERROR_FMASK 0x1
#define MH_MMU_PAGE_FAULT_REQ_VA_FADDR ADDR_MH_MMU_PAGE_FAULT
#define MH_MMU_PAGE_FAULT_REQ_VA_FSHIFT 12
#define MH_MMU_PAGE_FAULT_REQ_VA_FMASK 0xfffff
#define MH_MMU_PT_BASE_ZERO_FADDR ADDR_MH_MMU_PT_BASE
#define MH_MMU_PT_BASE_ZERO_FSHIFT 0
#define MH_MMU_PT_BASE_ZERO_FMASK 0xfff
#define MH_MMU_PT_BASE_PT_BASE_FADDR ADDR_MH_MMU_PT_BASE
#define MH_MMU_PT_BASE_PT_BASE_FSHIFT 12
#define MH_MMU_PT_BASE_PT_BASE_FMASK 0xfffff
#define MH_MMU_TRAN_ERROR_ZERO_FADDR ADDR_MH_MMU_TRAN_ERROR
#define MH_MMU_TRAN_ERROR_ZERO_FSHIFT 0
#define MH_MMU_TRAN_ERROR_ZERO_FMASK 0x1f
#define MH_MMU_TRAN_ERROR_TRAN_ERROR_FADDR ADDR_MH_MMU_TRAN_ERROR
#define MH_MMU_TRAN_ERROR_TRAN_ERROR_FSHIFT 5
#define MH_MMU_TRAN_ERROR_TRAN_ERROR_FMASK 0x7ffffff
#define MH_MMU_VA_RANGE_NUM_64KB_REGIONS_FADDR ADDR_MH_MMU_VA_RANGE
#define MH_MMU_VA_RANGE_NUM_64KB_REGIONS_FSHIFT 0
#define MH_MMU_VA_RANGE_NUM_64KB_REGIONS_FMASK 0xfff
#define MH_MMU_VA_RANGE_VA_BASE_FADDR ADDR_MH_MMU_VA_RANGE
#define MH_MMU_VA_RANGE_VA_BASE_FSHIFT 12
#define MH_MMU_VA_RANGE_VA_BASE_FMASK 0xfffff
#define MH_PERFCOUNTER0_CONFIG_N_VALUE_FADDR ADDR_MH_PERFCOUNTER0_CONFIG
#define MH_PERFCOUNTER0_CONFIG_N_VALUE_FSHIFT 0
#define MH_PERFCOUNTER0_CONFIG_N_VALUE_FMASK 0xff
#define MH_PERFCOUNTER0_HI_PERF_COUNTER_HI_FADDR ADDR_MH_PERFCOUNTER0_HI
#define MH_PERFCOUNTER0_HI_PERF_COUNTER_HI_FSHIFT 0
#define MH_PERFCOUNTER0_HI_PERF_COUNTER_HI_FMASK 0xffff
#define MH_PERFCOUNTER0_LOW_PERF_COUNTER_LOW_FADDR ADDR_MH_PERFCOUNTER0_LOW
#define MH_PERFCOUNTER0_LOW_PERF_COUNTER_LOW_FSHIFT 0
#define MH_PERFCOUNTER0_LOW_PERF_COUNTER_LOW_FMASK 0xffffffff
#define MH_PERFCOUNTER0_SELECT_PERF_SEL_FADDR ADDR_MH_PERFCOUNTER0_SELECT
#define MH_PERFCOUNTER0_SELECT_PERF_SEL_FSHIFT 0
#define MH_PERFCOUNTER0_SELECT_PERF_SEL_FMASK 0xff
#define MH_PERFCOUNTER1_CONFIG_N_VALUE_FADDR ADDR_MH_PERFCOUNTER1_CONFIG
#define MH_PERFCOUNTER1_CONFIG_N_VALUE_FSHIFT 0
#define MH_PERFCOUNTER1_CONFIG_N_VALUE_FMASK 0xff
#define MH_PERFCOUNTER1_HI_PERF_COUNTER_HI_FADDR ADDR_MH_PERFCOUNTER1_HI
#define MH_PERFCOUNTER1_HI_PERF_COUNTER_HI_FSHIFT 0
#define MH_PERFCOUNTER1_HI_PERF_COUNTER_HI_FMASK 0xffff
#define MH_PERFCOUNTER1_LOW_PERF_COUNTER_LOW_FADDR ADDR_MH_PERFCOUNTER1_LOW
#define MH_PERFCOUNTER1_LOW_PERF_COUNTER_LOW_FSHIFT 0
#define MH_PERFCOUNTER1_LOW_PERF_COUNTER_LOW_FMASK 0xffffffff
#define MH_PERFCOUNTER1_SELECT_PERF_SEL_FADDR ADDR_MH_PERFCOUNTER1_SELECT
#define MH_PERFCOUNTER1_SELECT_PERF_SEL_FSHIFT 0
#define MH_PERFCOUNTER1_SELECT_PERF_SEL_FMASK 0xff
#define MMU_READ_ADDR_ADDR_FADDR ADDR_MMU_READ_ADDR
#define MMU_READ_ADDR_ADDR_FSHIFT 0
#define MMU_READ_ADDR_ADDR_FMASK 0x7fff
#define MMU_READ_DATA_DATA_FADDR ADDR_MMU_READ_DATA
#define MMU_READ_DATA_DATA_FSHIFT 0
#define MMU_READ_DATA_DATA_FMASK 0xffffffff
#define VGV1_CBASE1_ADDR_FADDR ADDR_VGV1_CBASE1
#define VGV1_CBASE1_ADDR_FSHIFT 0
#define VGV1_CBASE1_ADDR_FMASK 0xffffffff
#define VGV1_CFG1_WINDRULE_FADDR ADDR_VGV1_CFG1
#define VGV1_CFG1_WINDRULE_FSHIFT 0
#define VGV1_CFG1_WINDRULE_FMASK 0x1
#define VGV1_CFG2_AAMODE_FADDR ADDR_VGV1_CFG2
#define VGV1_CFG2_AAMODE_FSHIFT 0
#define VGV1_CFG2_AAMODE_FMASK 0x3
#define VGV1_DIRTYBASE_ADDR_FADDR ADDR_VGV1_DIRTYBASE
#define VGV1_DIRTYBASE_ADDR_FSHIFT 0
#define VGV1_DIRTYBASE_ADDR_FMASK 0xffffffff
#define VGV1_FILL_INHERIT_FADDR ADDR_VGV1_FILL
#define VGV1_FILL_INHERIT_FSHIFT 0
#define VGV1_FILL_INHERIT_FMASK 0x1
#define VGV1_SCISSORX_LEFT_FADDR ADDR_VGV1_SCISSORX
#define VGV1_SCISSORX_LEFT_FSHIFT 0
#define VGV1_SCISSORX_LEFT_FMASK 0x7ff
#define VGV1_SCISSORX_PAD_FADDR ADDR_VGV1_SCISSORX
#define VGV1_SCISSORX_PAD_FSHIFT 11
#define VGV1_SCISSORX_PAD_FMASK 0x1f
#define VGV1_SCISSORX_RIGHT_FADDR ADDR_VGV1_SCISSORX
#define VGV1_SCISSORX_RIGHT_FSHIFT 16
#define VGV1_SCISSORX_RIGHT_FMASK 0x7ff
#define VGV1_SCISSORY_TOP_FADDR ADDR_VGV1_SCISSORY
#define VGV1_SCISSORY_TOP_FSHIFT 0
#define VGV1_SCISSORY_TOP_FMASK 0x7ff
#define VGV1_SCISSORY_PAD_FADDR ADDR_VGV1_SCISSORY
#define VGV1_SCISSORY_PAD_FSHIFT 11
#define VGV1_SCISSORY_PAD_FMASK 0x1f
#define VGV1_SCISSORY_BOTTOM_FADDR ADDR_VGV1_SCISSORY
#define VGV1_SCISSORY_BOTTOM_FSHIFT 16
#define VGV1_SCISSORY_BOTTOM_FMASK 0x7ff
#define VGV1_TILEOFS_X_FADDR ADDR_VGV1_TILEOFS
#define VGV1_TILEOFS_X_FSHIFT 0
#define VGV1_TILEOFS_X_FMASK 0xfff
#define VGV1_TILEOFS_Y_FADDR ADDR_VGV1_TILEOFS
#define VGV1_TILEOFS_Y_FSHIFT 12
#define VGV1_TILEOFS_Y_FMASK 0xfff
#define VGV1_TILEOFS_LEFTMOST_FADDR ADDR_VGV1_TILEOFS
#define VGV1_TILEOFS_LEFTMOST_FSHIFT 24
#define VGV1_TILEOFS_LEFTMOST_FMASK 0x1
#define VGV1_UBASE2_ADDR_FADDR ADDR_VGV1_UBASE2
#define VGV1_UBASE2_ADDR_FSHIFT 0
#define VGV1_UBASE2_ADDR_FMASK 0xffffffff
#define VGV1_VTX0_X_FADDR ADDR_VGV1_VTX0
#define VGV1_VTX0_X_FSHIFT 0
#define VGV1_VTX0_X_FMASK 0xffff
#define VGV1_VTX0_Y_FADDR ADDR_VGV1_VTX0
#define VGV1_VTX0_Y_FSHIFT 16
#define VGV1_VTX0_Y_FMASK 0xffff
#define VGV1_VTX1_X_FADDR ADDR_VGV1_VTX1
#define VGV1_VTX1_X_FSHIFT 0
#define VGV1_VTX1_X_FMASK 0xffff
#define VGV1_VTX1_Y_FADDR ADDR_VGV1_VTX1
#define VGV1_VTX1_Y_FSHIFT 16
#define VGV1_VTX1_Y_FMASK 0xffff
#define VGV2_ACCURACY_F_FADDR ADDR_VGV2_ACCURACY
#define VGV2_ACCURACY_F_FSHIFT 0
#define VGV2_ACCURACY_F_FMASK 0xffffff
#define VGV2_ACTION_ACTION_FADDR ADDR_VGV2_ACTION
#define VGV2_ACTION_ACTION_FSHIFT 0
#define VGV2_ACTION_ACTION_FMASK 0xf
#define VGV2_ARCCOS_F_FADDR ADDR_VGV2_ARCCOS
#define VGV2_ARCCOS_F_FSHIFT 0
#define VGV2_ARCCOS_F_FMASK 0xffffff
#define VGV2_ARCSIN_F_FADDR ADDR_VGV2_ARCSIN
#define VGV2_ARCSIN_F_FSHIFT 0
#define VGV2_ARCSIN_F_FMASK 0xffffff
#define VGV2_ARCTAN_F_FADDR ADDR_VGV2_ARCTAN
#define VGV2_ARCTAN_F_FSHIFT 0
#define VGV2_ARCTAN_F_FMASK 0xffffff
#define VGV2_BBOXMAXX_F_FADDR ADDR_VGV2_BBOXMAXX
#define VGV2_BBOXMAXX_F_FSHIFT 0
#define VGV2_BBOXMAXX_F_FMASK 0xffffff
#define VGV2_BBOXMAXY_F_FADDR ADDR_VGV2_BBOXMAXY
#define VGV2_BBOXMAXY_F_FSHIFT 0
#define VGV2_BBOXMAXY_F_FMASK 0xffffff
#define VGV2_BBOXMINX_F_FADDR ADDR_VGV2_BBOXMINX
#define VGV2_BBOXMINX_F_FSHIFT 0
#define VGV2_BBOXMINX_F_FMASK 0xffffff
#define VGV2_BBOXMINY_F_FADDR ADDR_VGV2_BBOXMINY
#define VGV2_BBOXMINY_F_FSHIFT 0
#define VGV2_BBOXMINY_F_FMASK 0xffffff
#define VGV2_BIAS_F_FADDR ADDR_VGV2_BIAS
#define VGV2_BIAS_F_FSHIFT 0
#define VGV2_BIAS_F_FMASK 0xffffff
#define VGV2_C1X_F_FADDR ADDR_VGV2_C1X
#define VGV2_C1X_F_FSHIFT 0
#define VGV2_C1X_F_FMASK 0xffffff
#define VGV2_C1XREL_F_FADDR ADDR_VGV2_C1XREL
#define VGV2_C1XREL_F_FSHIFT 0
#define VGV2_C1XREL_F_FMASK 0xffffff
#define VGV2_C1Y_F_FADDR ADDR_VGV2_C1Y
#define VGV2_C1Y_F_FSHIFT 0
#define VGV2_C1Y_F_FMASK 0xffffff
#define VGV2_C1YREL_F_FADDR ADDR_VGV2_C1YREL
#define VGV2_C1YREL_F_FSHIFT 0
#define VGV2_C1YREL_F_FMASK 0xffffff
#define VGV2_C2X_F_FADDR ADDR_VGV2_C2X
#define VGV2_C2X_F_FSHIFT 0
#define VGV2_C2X_F_FMASK 0xffffff
#define VGV2_C2XREL_F_FADDR ADDR_VGV2_C2XREL
#define VGV2_C2XREL_F_FSHIFT 0
#define VGV2_C2XREL_F_FMASK 0xffffff
#define VGV2_C2Y_F_FADDR ADDR_VGV2_C2Y
#define VGV2_C2Y_F_FSHIFT 0
#define VGV2_C2Y_F_FMASK 0xffffff
#define VGV2_C2YREL_F_FADDR ADDR_VGV2_C2YREL
#define VGV2_C2YREL_F_FSHIFT 0
#define VGV2_C2YREL_F_FMASK 0xffffff
#define VGV2_C3X_F_FADDR ADDR_VGV2_C3X
#define VGV2_C3X_F_FSHIFT 0
#define VGV2_C3X_F_FMASK 0xffffff
#define VGV2_C3XREL_F_FADDR ADDR_VGV2_C3XREL
#define VGV2_C3XREL_F_FSHIFT 0
#define VGV2_C3XREL_F_FMASK 0xffffff
#define VGV2_C3Y_F_FADDR ADDR_VGV2_C3Y
#define VGV2_C3Y_F_FSHIFT 0
#define VGV2_C3Y_F_FMASK 0xffffff
#define VGV2_C3YREL_F_FADDR ADDR_VGV2_C3YREL
#define VGV2_C3YREL_F_FSHIFT 0
#define VGV2_C3YREL_F_FMASK 0xffffff
#define VGV2_C4X_F_FADDR ADDR_VGV2_C4X
#define VGV2_C4X_F_FSHIFT 0
#define VGV2_C4X_F_FMASK 0xffffff
#define VGV2_C4XREL_F_FADDR ADDR_VGV2_C4XREL
#define VGV2_C4XREL_F_FSHIFT 0
#define VGV2_C4XREL_F_FMASK 0xffffff
#define VGV2_C4Y_F_FADDR ADDR_VGV2_C4Y
#define VGV2_C4Y_F_FSHIFT 0
#define VGV2_C4Y_F_FMASK 0xffffff
#define VGV2_C4YREL_F_FADDR ADDR_VGV2_C4YREL
#define VGV2_C4YREL_F_FSHIFT 0
#define VGV2_C4YREL_F_FMASK 0xffffff
#define VGV2_CLIP_F_FADDR ADDR_VGV2_CLIP
#define VGV2_CLIP_F_FSHIFT 0
#define VGV2_CLIP_F_FMASK 0xffffff
#define VGV2_FIRST_DUMMY_FADDR ADDR_VGV2_FIRST
#define VGV2_FIRST_DUMMY_FSHIFT 0
#define VGV2_FIRST_DUMMY_FMASK 0x1
#define VGV2_LAST_DUMMY_FADDR ADDR_VGV2_LAST
#define VGV2_LAST_DUMMY_FSHIFT 0
#define VGV2_LAST_DUMMY_FMASK 0x1
#define VGV2_MITER_F_FADDR ADDR_VGV2_MITER
#define VGV2_MITER_F_FSHIFT 0
#define VGV2_MITER_F_FMASK 0xffffff
#define VGV2_MODE_MAXSPLIT_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_MAXSPLIT_FSHIFT 0
#define VGV2_MODE_MAXSPLIT_FMASK 0xf
#define VGV2_MODE_CAP_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_CAP_FSHIFT 4
#define VGV2_MODE_CAP_FMASK 0x3
#define VGV2_MODE_JOIN_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_JOIN_FSHIFT 6
#define VGV2_MODE_JOIN_FMASK 0x3
#define VGV2_MODE_STROKE_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_STROKE_FSHIFT 8
#define VGV2_MODE_STROKE_FMASK 0x1
#define VGV2_MODE_STROKESPLIT_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_STROKESPLIT_FSHIFT 9
#define VGV2_MODE_STROKESPLIT_FMASK 0x1
#define VGV2_MODE_FULLSPLIT_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_FULLSPLIT_FSHIFT 10
#define VGV2_MODE_FULLSPLIT_FMASK 0x1
#define VGV2_MODE_NODOTS_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_NODOTS_FSHIFT 11
#define VGV2_MODE_NODOTS_FMASK 0x1
#define VGV2_MODE_OPENFILL_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_OPENFILL_FSHIFT 12
#define VGV2_MODE_OPENFILL_FMASK 0x1
#define VGV2_MODE_DROPLEFT_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_DROPLEFT_FSHIFT 13
#define VGV2_MODE_DROPLEFT_FMASK 0x1
#define VGV2_MODE_DROPOTHER_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_DROPOTHER_FSHIFT 14
#define VGV2_MODE_DROPOTHER_FMASK 0x1
#define VGV2_MODE_SYMMETRICJOINS_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_SYMMETRICJOINS_FSHIFT 15
#define VGV2_MODE_SYMMETRICJOINS_FMASK 0x1
#define VGV2_MODE_SIMPLESTROKE_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_SIMPLESTROKE_FSHIFT 16
#define VGV2_MODE_SIMPLESTROKE_FMASK 0x1
#define VGV2_MODE_SIMPLECLIP_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_SIMPLECLIP_FSHIFT 17
#define VGV2_MODE_SIMPLECLIP_FMASK 0x1
#define VGV2_MODE_EXPONENTADD_FADDR ADDR_VGV2_MODE
#define VGV2_MODE_EXPONENTADD_FSHIFT 18
#define VGV2_MODE_EXPONENTADD_FMASK 0x3f
#define VGV2_RADIUS_F_FADDR ADDR_VGV2_RADIUS
#define VGV2_RADIUS_F_FSHIFT 0
#define VGV2_RADIUS_F_FMASK 0xffffff
#define VGV2_SCALE_F_FADDR ADDR_VGV2_SCALE
#define VGV2_SCALE_F_FSHIFT 0
#define VGV2_SCALE_F_FMASK 0xffffff
#define VGV2_THINRADIUS_F_FADDR ADDR_VGV2_THINRADIUS
#define VGV2_THINRADIUS_F_FSHIFT 0
#define VGV2_THINRADIUS_F_FMASK 0xffffff
#define VGV2_XFSTXX_F_FADDR ADDR_VGV2_XFSTXX
#define VGV2_XFSTXX_F_FSHIFT 0
#define VGV2_XFSTXX_F_FMASK 0xffffff
#define VGV2_XFSTXY_F_FADDR ADDR_VGV2_XFSTXY
#define VGV2_XFSTXY_F_FSHIFT 0
#define VGV2_XFSTXY_F_FMASK 0xffffff
#define VGV2_XFSTYX_F_FADDR ADDR_VGV2_XFSTYX
#define VGV2_XFSTYX_F_FSHIFT 0
#define VGV2_XFSTYX_F_FMASK 0xffffff
#define VGV2_XFSTYY_F_FADDR ADDR_VGV2_XFSTYY
#define VGV2_XFSTYY_F_FSHIFT 0
#define VGV2_XFSTYY_F_FMASK 0xffffff
#define VGV2_XFXA_F_FADDR ADDR_VGV2_XFXA
#define VGV2_XFXA_F_FSHIFT 0
#define VGV2_XFXA_F_FMASK 0xffffff
#define VGV2_XFXX_F_FADDR ADDR_VGV2_XFXX
#define VGV2_XFXX_F_FSHIFT 0
#define VGV2_XFXX_F_FMASK 0xffffff
#define VGV2_XFXY_F_FADDR ADDR_VGV2_XFXY
#define VGV2_XFXY_F_FSHIFT 0
#define VGV2_XFXY_F_FMASK 0xffffff
#define VGV2_XFYA_F_FADDR ADDR_VGV2_XFYA
#define VGV2_XFYA_F_FSHIFT 0
#define VGV2_XFYA_F_FMASK 0xffffff
#define VGV2_XFYX_F_FADDR ADDR_VGV2_XFYX
#define VGV2_XFYX_F_FSHIFT 0
#define VGV2_XFYX_F_FMASK 0xffffff
#define VGV2_XFYY_F_FADDR ADDR_VGV2_XFYY
#define VGV2_XFYY_F_FSHIFT 0
#define VGV2_XFYY_F_FMASK 0xffffff
#define VGV3_CONTROL_MARKADD_FADDR ADDR_VGV3_CONTROL
#define VGV3_CONTROL_MARKADD_FSHIFT 0
#define VGV3_CONTROL_MARKADD_FMASK 0xfff
#define VGV3_CONTROL_DMIWAITCHMASK_FADDR ADDR_VGV3_CONTROL
#define VGV3_CONTROL_DMIWAITCHMASK_FSHIFT 12
#define VGV3_CONTROL_DMIWAITCHMASK_FMASK 0xf
#define VGV3_CONTROL_PAUSE_FADDR ADDR_VGV3_CONTROL
#define VGV3_CONTROL_PAUSE_FSHIFT 16
#define VGV3_CONTROL_PAUSE_FMASK 0x1
#define VGV3_CONTROL_ABORT_FADDR ADDR_VGV3_CONTROL
#define VGV3_CONTROL_ABORT_FSHIFT 17
#define VGV3_CONTROL_ABORT_FMASK 0x1
#define VGV3_CONTROL_WRITE_FADDR ADDR_VGV3_CONTROL
#define VGV3_CONTROL_WRITE_FSHIFT 18
#define VGV3_CONTROL_WRITE_FMASK 0x1
#define VGV3_CONTROL_BCFLUSH_FADDR ADDR_VGV3_CONTROL
#define VGV3_CONTROL_BCFLUSH_FSHIFT 19
#define VGV3_CONTROL_BCFLUSH_FMASK 0x1
#define VGV3_CONTROL_V0SYNC_FADDR ADDR_VGV3_CONTROL
#define VGV3_CONTROL_V0SYNC_FSHIFT 20
#define VGV3_CONTROL_V0SYNC_FMASK 0x1
#define VGV3_CONTROL_DMIWAITBUF_FADDR ADDR_VGV3_CONTROL
#define VGV3_CONTROL_DMIWAITBUF_FSHIFT 21
#define VGV3_CONTROL_DMIWAITBUF_FMASK 0x7
#define VGV3_FIRST_DUMMY_FADDR ADDR_VGV3_FIRST
#define VGV3_FIRST_DUMMY_FSHIFT 0
#define VGV3_FIRST_DUMMY_FMASK 0x1
#define VGV3_LAST_DUMMY_FADDR ADDR_VGV3_LAST
#define VGV3_LAST_DUMMY_FSHIFT 0
#define VGV3_LAST_DUMMY_FMASK 0x1
#define VGV3_MODE_FLIPENDIAN_FADDR ADDR_VGV3_MODE
#define VGV3_MODE_FLIPENDIAN_FSHIFT 0
#define VGV3_MODE_FLIPENDIAN_FMASK 0x1
#define VGV3_MODE_UNUSED_FADDR ADDR_VGV3_MODE
#define VGV3_MODE_UNUSED_FSHIFT 1
#define VGV3_MODE_UNUSED_FMASK 0x1
#define VGV3_MODE_WRITEFLUSH_FADDR ADDR_VGV3_MODE
#define VGV3_MODE_WRITEFLUSH_FSHIFT 2
#define VGV3_MODE_WRITEFLUSH_FMASK 0x1
#define VGV3_MODE_DMIPAUSETYPE_FADDR ADDR_VGV3_MODE
#define VGV3_MODE_DMIPAUSETYPE_FSHIFT 3
#define VGV3_MODE_DMIPAUSETYPE_FMASK 0x1
#define VGV3_MODE_DMIRESET_FADDR ADDR_VGV3_MODE
#define VGV3_MODE_DMIRESET_FSHIFT 4
#define VGV3_MODE_DMIRESET_FMASK 0x1
#define VGV3_NEXTADDR_CALLADDR_FADDR ADDR_VGV3_NEXTADDR
#define VGV3_NEXTADDR_CALLADDR_FSHIFT 0
#define VGV3_NEXTADDR_CALLADDR_FMASK 0xffffffff
#define VGV3_NEXTCMD_COUNT_FADDR ADDR_VGV3_NEXTCMD
#define VGV3_NEXTCMD_COUNT_FSHIFT 0
#define VGV3_NEXTCMD_COUNT_FMASK 0xfff
#define VGV3_NEXTCMD_NEXTCMD_FADDR ADDR_VGV3_NEXTCMD
#define VGV3_NEXTCMD_NEXTCMD_FSHIFT 12
#define VGV3_NEXTCMD_NEXTCMD_FMASK 0x7
#define VGV3_NEXTCMD_MARK_FADDR ADDR_VGV3_NEXTCMD
#define VGV3_NEXTCMD_MARK_FSHIFT 15
#define VGV3_NEXTCMD_MARK_FMASK 0x1
#define VGV3_NEXTCMD_CALLCOUNT_FADDR ADDR_VGV3_NEXTCMD
#define VGV3_NEXTCMD_CALLCOUNT_FSHIFT 16
#define VGV3_NEXTCMD_CALLCOUNT_FMASK 0xfff
#define VGV3_VGBYPASS_BYPASS_FADDR ADDR_VGV3_VGBYPASS
#define VGV3_VGBYPASS_BYPASS_FSHIFT 0
#define VGV3_VGBYPASS_BYPASS_FMASK 0x1
#define VGV3_WRITE_VALUE_FADDR ADDR_VGV3_WRITE
#define VGV3_WRITE_VALUE_FSHIFT 0
#define VGV3_WRITE_VALUE_FMASK 0xffffffff
#define VGV3_WRITEADDR_ADDR_FADDR ADDR_VGV3_WRITEADDR
#define VGV3_WRITEADDR_ADDR_FSHIFT 0
#define VGV3_WRITEADDR_ADDR_FMASK 0xffffffff
#define VGV3_WRITEDMI_CHANMASK_FADDR ADDR_VGV3_WRITEDMI
#define VGV3_WRITEDMI_CHANMASK_FSHIFT 0
#define VGV3_WRITEDMI_CHANMASK_FMASK 0xf
#define VGV3_WRITEDMI_BUFFER_FADDR ADDR_VGV3_WRITEDMI
#define VGV3_WRITEDMI_BUFFER_FSHIFT 4
#define VGV3_WRITEDMI_BUFFER_FMASK 0x7
#define VGV3_WRITEF32_ADDR_FADDR ADDR_VGV3_WRITEF32
#define VGV3_WRITEF32_ADDR_FSHIFT 0
#define VGV3_WRITEF32_ADDR_FMASK 0xff
#define VGV3_WRITEF32_COUNT_FADDR ADDR_VGV3_WRITEF32
#define VGV3_WRITEF32_COUNT_FSHIFT 8
#define VGV3_WRITEF32_COUNT_FMASK 0xff
#define VGV3_WRITEF32_LOOP_FADDR ADDR_VGV3_WRITEF32
#define VGV3_WRITEF32_LOOP_FSHIFT 16
#define VGV3_WRITEF32_LOOP_FMASK 0xf
#define VGV3_WRITEF32_ACTION_FADDR ADDR_VGV3_WRITEF32
#define VGV3_WRITEF32_ACTION_FSHIFT 20
#define VGV3_WRITEF32_ACTION_FMASK 0xf
#define VGV3_WRITEF32_FORMAT_FADDR ADDR_VGV3_WRITEF32
#define VGV3_WRITEF32_FORMAT_FSHIFT 24
#define VGV3_WRITEF32_FORMAT_FMASK 0x7
#define VGV3_WRITEIFPAUSED_VALUE_FADDR ADDR_VGV3_WRITEIFPAUSED
#define VGV3_WRITEIFPAUSED_VALUE_FSHIFT 0
#define VGV3_WRITEIFPAUSED_VALUE_FMASK 0xffffffff
#define VGV3_WRITERAW_ADDR_FADDR ADDR_VGV3_WRITERAW
#define VGV3_WRITERAW_ADDR_FSHIFT 0
#define VGV3_WRITERAW_ADDR_FMASK 0xff
#define VGV3_WRITERAW_COUNT_FADDR ADDR_VGV3_WRITERAW
#define VGV3_WRITERAW_COUNT_FSHIFT 8
#define VGV3_WRITERAW_COUNT_FMASK 0xff
#define VGV3_WRITERAW_LOOP_FADDR ADDR_VGV3_WRITERAW
#define VGV3_WRITERAW_LOOP_FSHIFT 16
#define VGV3_WRITERAW_LOOP_FMASK 0xf
#define VGV3_WRITERAW_ACTION_FADDR ADDR_VGV3_WRITERAW
#define VGV3_WRITERAW_ACTION_FSHIFT 20
#define VGV3_WRITERAW_ACTION_FMASK 0xf
#define VGV3_WRITERAW_FORMAT_FADDR ADDR_VGV3_WRITERAW
#define VGV3_WRITERAW_FORMAT_FSHIFT 24
#define VGV3_WRITERAW_FORMAT_FMASK 0x7
#define VGV3_WRITES16_ADDR_FADDR ADDR_VGV3_WRITES16
#define VGV3_WRITES16_ADDR_FSHIFT 0
#define VGV3_WRITES16_ADDR_FMASK 0xff
#define VGV3_WRITES16_COUNT_FADDR ADDR_VGV3_WRITES16
#define VGV3_WRITES16_COUNT_FSHIFT 8
#define VGV3_WRITES16_COUNT_FMASK 0xff
#define VGV3_WRITES16_LOOP_FADDR ADDR_VGV3_WRITES16
#define VGV3_WRITES16_LOOP_FSHIFT 16
#define VGV3_WRITES16_LOOP_FMASK 0xf
#define VGV3_WRITES16_ACTION_FADDR ADDR_VGV3_WRITES16
#define VGV3_WRITES16_ACTION_FSHIFT 20
#define VGV3_WRITES16_ACTION_FMASK 0xf
#define VGV3_WRITES16_FORMAT_FADDR ADDR_VGV3_WRITES16
#define VGV3_WRITES16_FORMAT_FSHIFT 24
#define VGV3_WRITES16_FORMAT_FMASK 0x7
#define VGV3_WRITES32_ADDR_FADDR ADDR_VGV3_WRITES32
#define VGV3_WRITES32_ADDR_FSHIFT 0
#define VGV3_WRITES32_ADDR_FMASK 0xff
#define VGV3_WRITES32_COUNT_FADDR ADDR_VGV3_WRITES32
#define VGV3_WRITES32_COUNT_FSHIFT 8
#define VGV3_WRITES32_COUNT_FMASK 0xff
#define VGV3_WRITES32_LOOP_FADDR ADDR_VGV3_WRITES32
#define VGV3_WRITES32_LOOP_FSHIFT 16
#define VGV3_WRITES32_LOOP_FMASK 0xf
#define VGV3_WRITES32_ACTION_FADDR ADDR_VGV3_WRITES32
#define VGV3_WRITES32_ACTION_FSHIFT 20
#define VGV3_WRITES32_ACTION_FMASK 0xf
#define VGV3_WRITES32_FORMAT_FADDR ADDR_VGV3_WRITES32
#define VGV3_WRITES32_FORMAT_FSHIFT 24
#define VGV3_WRITES32_FORMAT_FMASK 0x7
#define VGV3_WRITES8_ADDR_FADDR ADDR_VGV3_WRITES8
#define VGV3_WRITES8_ADDR_FSHIFT 0
#define VGV3_WRITES8_ADDR_FMASK 0xff
#define VGV3_WRITES8_COUNT_FADDR ADDR_VGV3_WRITES8
#define VGV3_WRITES8_COUNT_FSHIFT 8
#define VGV3_WRITES8_COUNT_FMASK 0xff
#define VGV3_WRITES8_LOOP_FADDR ADDR_VGV3_WRITES8
#define VGV3_WRITES8_LOOP_FSHIFT 16
#define VGV3_WRITES8_LOOP_FMASK 0xf
#define VGV3_WRITES8_ACTION_FADDR ADDR_VGV3_WRITES8
#define VGV3_WRITES8_ACTION_FSHIFT 20
#define VGV3_WRITES8_ACTION_FMASK 0xf
#define VGV3_WRITES8_FORMAT_FADDR ADDR_VGV3_WRITES8
#define VGV3_WRITES8_FORMAT_FSHIFT 24
#define VGV3_WRITES8_FORMAT_FMASK 0x7
typedef struct {
    unsigned RS[256];
    unsigned GRADW[2][40];
} regstate_t;

#define GRADW_WINDOW_START 0xc0
#define GRADW_WINDOW_LEN 0x28
#define GRADW_WINDOW_NUM 0x2

static unsigned __inline __getwrs__(regstate_t* RS, unsigned win, unsigned addr, unsigned shift, unsigned mask) {
    if ( addr >= 0xc0 && addr < 0xe8 ) { 
        assert( win < 2 );
        return (RS->GRADW[win][addr-0xc0] >>
                shift) & mask;
    }
    return ((RS->RS[addr] >> shift) & mask);
}

static void __inline __setwrs__(regstate_t* RS, unsigned win, unsigned addr, unsigned shift, unsigned mask, unsigned data) {
    if ( addr >= 0xc0 && addr < 0xe8 ) { 
        assert( win < 2 );
        RS->GRADW[win][addr-0xc0] = (RS->GRADW[win][addr-0xc0] & 
                   ~(mask << shift)) | 
                   ((mask & data) << shift);
    }
    RS->RS[addr] = (RS->RS[addr] & ~(mask << shift)) | ((mask & data) << shift);
}

static void __inline __setwreg__(regstate_t* RS, unsigned win, unsigned addr, unsigned data) {
    if ( addr >= 0xc0 && addr < 0xe8 ) { 
        assert( win < 2 );
        RS->GRADW[win][addr-0xc0] = data;
    }
    RS->RS[addr] = data;
}

static unsigned __inline __getrs__(regstate_t* RS, unsigned addr, unsigned shift, unsigned mask) {
    return ((RS->RS[addr] >> shift) & mask);
}

static void __inline __setrs__(regstate_t* RS, unsigned addr, unsigned shift, unsigned mask, unsigned data) {
    if ( addr >= 0xc0 && addr < 0xe8 ) { 
        unsigned win = __getrs__(RS, G2D_GRADIENT_SEL_FADDR, G2D_GRADIENT_SEL_FSHIFT, G2D_GRADIENT_SEL_FMASK);
        assert( win < 2 );
        RS->GRADW[win][addr-0xc0] = (RS->GRADW[win][addr-0xc0] & 
                   ~(mask << shift)) | ((mask & data) << shift);
    }
    RS->RS[addr] = (RS->RS[addr] & ~(mask << shift)) | ((mask & data) << shift);
}

static void __inline __setreg__(regstate_t* RS, unsigned addr, unsigned data) {
    if ( addr >= 0xc0 && addr < 0xe8 ) { 
        unsigned win = __getrs__(RS, G2D_GRADIENT_SEL_FADDR, G2D_GRADIENT_SEL_FSHIFT, G2D_GRADIENT_SEL_FMASK);
        assert( win < 2 );
        RS->GRADW[win][addr-0xc0] = data;
    }
    RS->RS[addr] = data;
}

#define SETWRS(win, id, value) __setwrs__(&RS, win, id##_FADDR, id##_FSHIFT, id##_FMASK, value)
#define GETWRS(win, id) __getwrs__(&RS, win, id##_FADDR, id##_FSHIFT, id##_FMASK)
#define SETWREG(win, reg, data) __setwreg__(&RS, win, reg, data)
#define SETRS(id, value) __setrs__(&RS, id##_FADDR, id##_FSHIFT, id##_FMASK, value)
#define GETRS(id) __getrs__(&RS, id##_FADDR, id##_FSHIFT, id##_FMASK)
#define SETREG(reg, data) __setreg__(&RS, reg, data)
#endif
