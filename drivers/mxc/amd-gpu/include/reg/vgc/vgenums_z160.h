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

#ifndef __REGS_ENUMS_H
#define __REGS_ENUMS_H

typedef enum _BB_CULL {
    BB_CULL_NONE                     = 0,
    BB_CULL_CW                       = 1,
    BB_CULL_CCW                      = 2,
} BB_CULL;

typedef enum _BB_TEXTUREADDRESS {
    BB_TADDRESS_WRAP                 = 0,
    BB_TADDRESS_CLAMP                = 1,
    BB_TADDRESS_BORDER               = 2,
    BB_TADDRESS_MIRROR               = 4,
    BB_TADDRESS_MIRRORCLAMP          = 5,   // Not supported on G3x cores
    BB_TADDRESS_MIRRORBORDER         = 6,   // Not supported on G3x cores
} BB_TEXTUREADDRESS;

typedef enum _BB_TEXTYPE {
    BB_TEXTYPE_4444                  = 0,
    BB_TEXTYPE_1555                  = 1,
    BB_TEXTYPE_5551                  = 2,
    BB_TEXTYPE_565                   = 3,
    BB_TEXTYPE_8888                  = 4,
    BB_TEXTYPE_8                     = 5,
    BB_TEXTYPE_88                    = 6,
    BB_TEXTYPE_4                     = 7,
    BB_TEXTYPE_44                    = 8,
    BB_TEXTYPE_UYVY                  = 9,
    BB_TEXTYPE_YUY2                  = 10,
    BB_TEXTYPE_YVYU                  = 11,
    BB_TEXTYPE_DXT1                  = 12,
    BB_TEXTYPE_PACKMAN               = 13,
    BB_TEXTYPE_PACKMAN_ALPHA4        = 14,
    BB_TEXTYPE_1F16                  = 15,
    BB_TEXTYPE_2F16                  = 16,
    BB_TEXTYPE_4F16                  = 17,
    BB_TEXTYPE_IPACKMAN_RGB          = 18,
    BB_TEXTYPE_IPACKMAN_RGBA         = 19,
} BB_TEXTYPE;

typedef enum _BB_CMPFUNC {
    BB_CMP_NEVER                     = 0,
    BB_CMP_LESS                      = 1,
    BB_CMP_EQUAL                     = 2,
    BB_CMP_LESSEQUAL                 = 3,
    BB_CMP_GREATER                   = 4,
    BB_CMP_NOTEQUAL                  = 5,
    BB_CMP_GREATEREQUAL              = 6,
    BB_CMP_ALWAYS                    = 7,
} BB_CMPFUNC;

typedef enum _BB_STENCILOP {
    BB_STENCILOP_KEEP                = 0,
    BB_STENCILOP_ZERO                = 1,
    BB_STENCILOP_REPLACE             = 2,
    BB_STENCILOP_INCRSAT             = 3,
    BB_STENCILOP_DECRSAT             = 4,
    BB_STENCILOP_INVERT              = 5,
    BB_STENCILOP_INCR                = 6,
    BB_STENCILOP_DECR                = 7,
} BB_STENCILOP;

typedef enum _BB_PRIMITIVETYPE {
    BB_PT_POINTLIST                  = 0,
    BB_PT_LINELIST                   = 1,
    BB_PT_LINESTRIP                  = 2,
    BB_PT_TRIANGLELIST               = 3,
    BB_PT_TRIANGLESTRIP              = 4,
    BB_PT_TRIANGLEFAN                = 5,
} BB_PRIMITIVETYPE;

typedef enum _BB_TEXTUREFILTERTYPE {
    BB_TEXF_NONE                     = 0,   // filtering disabled (valid for mip filter only)
    BB_TEXF_POINT                    = 1,   // nearest
    BB_TEXF_LINEAR                   = 2,   // linear interpolation
} BB_TEXTUREFILTERTYPE;

typedef enum _BB_BUFFER {
    BB_BUFFER_PPCODE                 = 0,   // Pixel processor code
    BB_BUFFER_UNUSED                 = 1,   // Unused
    BB_BUFFER_CBUF                   = 2,   // Color buffer
    BB_BUFFER_ZBUF                   = 3,   // Z buffer
    BB_BUFFER_AUXBUF0                = 4,   // AUX0 buffer
    BB_BUFFER_AUXBUF1                = 5,   // AUX1 buffer
    BB_BUFFER_AUXBUF2                = 6,   // AUX2 buffer
    BB_BUFFER_AUXBUF3                = 7,   // AUX3 buffer
} BB_BUFFER;

typedef enum _BB_COLORFORMAT {
    BB_COLOR_ARGB4444                = 0,
    BB_COLOR_ARGB0565                = 1,
    BB_COLOR_ARGB1555                = 2,
    BB_COLOR_RGBA5551                = 3,
    BB_COLOR_ARGB8888                = 4,
    BB_COLOR_R16                     = 5,
    BB_COLOR_RG1616                  = 6,
    BB_COLOR_ARGB16161616            = 7,
    BB_COLOR_D16                     = 8,
    BB_COLOR_S4D12                   = 9,
    BB_COLOR_S1D15                   = 10,
    BB_COLOR_X8D24                   = 11,
    BB_COLOR_S8D24                   = 12,
    BB_COLOR_X2D30                   = 13,
} BB_COLORFORMAT;

typedef enum _BB_PP_REGCONFIG {
    BB_PP_REGCONFIG_1                = 0,
    BB_PP_REGCONFIG_2                = 1,
    BB_PP_REGCONFIG_3                = 8,
    BB_PP_REGCONFIG_4                = 2,
    BB_PP_REGCONFIG_6                = 9,
    BB_PP_REGCONFIG_8                = 3,
    BB_PP_REGCONFIG_12               = 10,
    BB_PP_REGCONFIG_16               = 4,
    BB_PP_REGCONFIG_24               = 11,
    BB_PP_REGCONFIG_32               = 5,
} BB_PP_REGCONFIG;

typedef enum _G2D_read_t {
    G2D_READ_DST                     = 0,
    G2D_READ_SRC1                    = 1,
    G2D_READ_SRC2                    = 2,
    G2D_READ_SRC3                    = 3,
} G2D_read_t;

typedef enum _G2D_format_t {
    G2D_1                            = 0,   // foreground & background
    G2D_1BW                          = 1,   // black & white
    G2D_4                            = 2,
    G2D_8                            = 3,   // alpha
    G2D_4444                         = 4,
    G2D_1555                         = 5,
    G2D_0565                         = 6,
    G2D_8888                         = 7,
    G2D_YUY2                         = 8,
    G2D_UYVY                         = 9,
    G2D_YVYU                         = 10,
    G2D_4444_RGBA                    = 11,
    G2D_5551_RGBA                    = 12,
    G2D_8888_RGBA                    = 13,
    G2D_A8                           = 14,  // for alpha texture only
} G2D_format_t;

typedef enum _G2D_wrap_t {
    G2D_WRAP_CLAMP                   = 0,
    G2D_WRAP_REPEAT                  = 1,
    G2D_WRAP_MIRROR                  = 2,
    G2D_WRAP_BORDER                  = 3,
} G2D_wrap_t;

typedef enum _G2D_BLEND_OP {
    G2D_BLENDOP_ADD                  = 0,
    G2D_BLENDOP_SUB                  = 1,
    G2D_BLENDOP_MIN                  = 2,
    G2D_BLENDOP_MAX                  = 3,
} G2D_BLEND_OP;

typedef enum _G2D_GRAD_OP {
    G2D_GRADOP_DOT                   = 0,
    G2D_GRADOP_RCP                   = 1,
    G2D_GRADOP_SQRTMUL               = 2,
    G2D_GRADOP_SQRTADD               = 3,
} G2D_GRAD_OP;

typedef enum _G2D_BLEND_SRC {
    G2D_BLENDSRC_ZERO                = 0,   // One with invert
    G2D_BLENDSRC_SOURCE              = 1,   // Paint with coverage alpha applied
    G2D_BLENDSRC_DESTINATION         = 2,
    G2D_BLENDSRC_IMAGE               = 3,   // Second texture
    G2D_BLENDSRC_TEMP0               = 4,
    G2D_BLENDSRC_TEMP1               = 5,
    G2D_BLENDSRC_TEMP2               = 6,
} G2D_BLEND_SRC;

typedef enum _G2D_BLEND_DST {
    G2D_BLENDDST_IGNORE              = 0,   // Ignore destination
    G2D_BLENDDST_TEMP0               = 1,
    G2D_BLENDDST_TEMP1               = 2,
    G2D_BLENDDST_TEMP2               = 3,
} G2D_BLEND_DST;

typedef enum _G2D_BLEND_CONST {
    G2D_BLENDSRC_CONST0              = 0,
    G2D_BLENDSRC_CONST1              = 1,
    G2D_BLENDSRC_CONST2              = 2,
    G2D_BLENDSRC_CONST3              = 3,
    G2D_BLENDSRC_CONST4              = 4,
    G2D_BLENDSRC_CONST5              = 5,
    G2D_BLENDSRC_CONST6              = 6,
    G2D_BLENDSRC_CONST7              = 7,
} G2D_BLEND_CONST;

typedef enum _V3_NEXTCMD {
    VGV3_NEXTCMD_CONTINUE            = 0,   // Continue reading at current address, COUNT gives size of next packet.
    VGV3_NEXTCMD_JUMP                = 1,   // Jump to CALLADDR, COUNT gives size of next packet.
    VGV3_NEXTCMD_CALL                = 2,   // First call a sub-stream at CALLADDR for CALLCOUNT dwords. Then perform a continue.
    VGV3_NEXTCMD_CALLV2TRUE          = 3,   // Not supported.
    VGV3_NEXTCMD_CALLV2FALSE         = 4,   // Not supported.
    VGV3_NEXTCMD_ABORT               = 5,   // Abort reading. This ends the stream. Normally stream can just be paused (or automatically pauses at the end) which avoids any data being lost.
} V3_NEXTCMD;

typedef enum _V3_FORMAT {
    VGV3_FORMAT_S8                   = 0,   // Signed 8 bit data (4 writes per data dword) => VGV2-float
    VGV3_FORMAT_S16                  = 1,   // Signed 16 bit data (2 writes per data dword) => VGV2-float
    VGV3_FORMAT_S32                  = 2,   // Signed 32 bit data => VGV2-float
    VGV3_FORMAT_F32                  = 3,   // IEEE 32-bit floating point => VGV2-float
    VGV3_FORMAT_RAW                  = 4,   // No conversion
} V3_FORMAT;

typedef enum _V2_ACTION {
    VGV2_ACTION_END                  = 0,   // end previous path
    VGV2_ACTION_MOVETOOPEN           = 1,   // end previous path, C1=C4, start new open subpath
    VGV2_ACTION_MOVETOCLOSED         = 2,   // end previous path, C1=C4, start new closed subpath
    VGV2_ACTION_LINETO               = 3,   // line C1,C4
    VGV2_ACTION_CUBICTO              = 4,   // cubic C1,C2,C3,C4.
    VGV2_ACTION_QUADTO               = 5,   // quadratic C1,C3,C4.
    VGV2_ACTION_SCUBICTO             = 6,   // smooth cubic C1,C4.
    VGV2_ACTION_SQUADTO              = 7,   // smooth quadratic C1,C3,C4.
    VGV2_ACTION_VERTEXTO             = 8,   // half lineto C4=pos, C3=normal.
    VGV2_ACTION_VERTEXTOOPEN         = 9,   // moveto open + half lineto C4=pos, C3=normal.
    VGV2_ACTION_VERTEXTOCLOSED       = 10,  // moveto closed + half lineto C4=pos, C3=normal.
    VGV2_ACTION_MOVETOMOVE           = 11,  // end previous path, C1=C4, move but do not start a subpath
    VGV2_ACTION_FLUSH                = 15,  // end previous path and block following regwrites until all lines sent
} V2_ACTION;

typedef enum _V2_CAP {
    VGV2_CAP_BUTT                    = 0,   // butt caps (straight line overlappin starting point
    VGV2_CAP_ROUND                   = 1,   // round caps (smoothness depends on ARCSIN/ARCCOS registers)
    VGV2_CAP_SQUARE                  = 2,   // square caps (square centered on starting point)
} V2_CAP;

typedef enum _V2_JOIN {
    VGV2_JOIN_MITER                  = 0,   // miter joins (both sides extended towards intersection. If angle is too small (compared to STMITER register) the miter is converted into a BEVEL.
    VGV2_JOIN_ROUND                  = 1,   // round joins (smoothness depends on ARCSIN/ARCCOS registers)
    VGV2_JOIN_BEVEL                  = 2,   // bevel joins (ends of both sides are connected with a single line)
} V2_JOIN;

enum
{
    G2D_GRADREG_X        = 0,  // also usable as temp
    G2D_GRADREG_Y        = 1,  // also usable as temp
    G2D_GRADREG_OUTX     = 8,
    G2D_GRADREG_OUTY     = 9,
    G2D_GRADREG_C0       = 16,
    G2D_GRADREG_C1       = 17,
    G2D_GRADREG_C2       = 18,
    G2D_GRADREG_C3       = 19,
    G2D_GRADREG_C4       = 20,
    G2D_GRADREG_C5       = 21,
    G2D_GRADREG_C6       = 22,
    G2D_GRADREG_C7       = 23,
    G2D_GRADREG_C8       = 24,
    G2D_GRADREG_C9       = 25,
    G2D_GRADREG_C10      = 26,
    G2D_GRADREG_C11      = 27,
    G2D_GRADREG_ZERO     = 28,
    G2D_GRADREG_ONE      = 29,
    G2D_GRADREG_MINUSONE = 30,
};

#endif
