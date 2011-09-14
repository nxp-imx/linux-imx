//------------------------------------------------------------------------------
// <copyright file="AR6002_regdump.h" company="Atheros">
//    Copyright (c) 2006 Atheros Corporation.  All rights reserved.
// 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// Software distributed under the License is distributed on an "AS
// IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
// implied. See the License for the specific language governing
// rights and limitations under the License.
//
//
//------------------------------------------------------------------------------
//==============================================================================
// Author(s): ="Atheros"
//==============================================================================

#ifndef __AR6002_REGDUMP_H__
#define __AR6002_REGDUMP_H__

#if !defined(__ASSEMBLER__)
/*
 * XTensa CPU state
 * This must match the state saved by the target exception handler.
 */
struct XTensa_exception_frame_s {
    A_UINT32 xt_pc;
    A_UINT32 xt_ps;
    A_UINT32 xt_sar;
    A_UINT32 xt_vpri;
    A_UINT32 xt_a2;
    A_UINT32 xt_a3;
    A_UINT32 xt_a4;
    A_UINT32 xt_a5;
    A_UINT32 xt_exccause;
    A_UINT32 xt_lcount;
    A_UINT32 xt_lbeg;
    A_UINT32 xt_lend;

    A_UINT32 epc1, epc2, epc3, epc4;

    /* Extra info to simplify post-mortem stack walkback */
#define AR6002_REGDUMP_FRAMES 10
    struct {
        A_UINT32 a0;  /* pc */
        A_UINT32 a1;  /* sp */
        A_UINT32 a2;
        A_UINT32 a3;
    } wb[AR6002_REGDUMP_FRAMES];
};
typedef struct XTensa_exception_frame_s CPU_exception_frame_t; 
#define RD_SIZE sizeof(CPU_exception_frame_t)

#endif
#endif /* __AR6002_REGDUMP_H__ */
