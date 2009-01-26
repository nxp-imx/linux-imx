/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/linkage.h>

	.text
	.align	2
ENTRY(opl_rotate90_u16_qcif)
        STMFD    sp!,{r4-r10}
        MOV      r12,#0x160
        MOV      r10,#0x90
        MOV      r3,r10,LSR #4
.L1.216:
        RSB      r2,r3,r10,LSR #4
        MOV      r4,#0x20
        SMULBB   r5,r4,r2
        MOV      r4,#0x1600
        SMULBB   r2,r4,r2
        ADD      r4,r0,#0xc000
        ADD      r4,r4,#0x4a0
        SUB      r4,r4,r2
        MOV      r2,r12,LSR #1
        ADD      r5,r1,r5
.L1.256:
        LDRH     r6,[r4],-r12
        LDRH     r7,[r4],-r12
        LDRH     r8,[r4],-r12
        LDRH     r9,[r4],-r12
        ORR      r6,r6,r7,LSL #16
        ORR      r7,r8,r9,LSL #16
        STMIA    r5!,{r6,r7}
        SUBS     r2,r2,#1
        LDRH     r6,[r4],-r12
        LDRH     r7,[r4],-r12
        LDRH     r8,[r4],-r12
        LDRH     r9,[r4],-r12
        ORR      r6,r6,r7,LSL #16
        ORR      r7,r8,r9,LSL #16
        STMIA    r5!,{r6,r7}
        LDRH     r6,[r4],-r12
        LDRH     r7,[r4],-r12
        LDRH     r8,[r4],-r12
        LDRH     r9,[r4],-r12
        ORR      r6,r6,r7,LSL #16
        ORR      r7,r8,r9,LSL #16
        STMIA    r5!,{r6,r7}
        LDRH     r6,[r4],-r12
        LDRH     r7,[r4],-r12
        LDRH     r8,[r4],-r12
        LDRH     r9,[r4],-r12
        ORR      r6,r6,r7,LSL #16
        ORR      r7,r8,r9,LSL #16
        ADD      r4,r4,#0x1600
        STMIA    r5!,{r6,r7}
        ADD      r5,r5,#0x100
        ADD      r4,r4,#2
        BGT      .L1.256
        SUBS     r3,r3,#1
        BGT      .L1.216
        LDMFD    sp!,{r4-r10}
        BX       lr
	.size	opl_rotate90_u16_qcif, . - opl_rotate90_u16_qcif
