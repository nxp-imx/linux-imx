/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
ENTRY(opl_rotate270_u16_qcif)
	STMFD    sp!,{r4-r10}
        MOV      r12,#0x160
        MOV      r10,#0x90
        MOV      r3,r10,LSR #4
.L1.16:
        RSB      r2,r3,r10,LSR #4
        MOV      r5,r2,LSL #5
        MOV      r4,r12,LSR #1
        SMULBB   r4,r5,r4
        ADD      r2,r1,r2,LSL #5
        ADD      r5,r2,#0xc000
        ADD      r5,r5,#0x4e0
        MOV      r2,r12,LSR #1
        ADD      r4,r0,r4
.L1.52:
        LDRH     r6,[r4],r12
        LDRH     r7,[r4],r12
        LDRH     r8,[r4],r12
        LDRH     r9,[r4],r12
        ORR      r6,r6,r7,LSL #16
        ORR      r7,r8,r9,LSL #16
        STMIA    r5!,{r6,r7}
        SUBS     r2,r2,#1
        LDRH     r6,[r4],r12
        LDRH     r7,[r4],r12
        LDRH     r8,[r4],r12
        LDRH     r9,[r4],r12
        ORR      r6,r6,r7,LSL #16
        ORR      r7,r8,r9,LSL #16
        STMIA    r5!,{r6,r7}
        LDRH     r6,[r4],r12
        LDRH     r7,[r4],r12
        LDRH     r8,[r4],r12
        LDRH     r9,[r4],r12
        ORR      r6,r6,r7,LSL #16
        ORR      r7,r8,r9,LSL #16
        STMIA    r5!,{r6,r7}
        LDRH     r6,[r4],r12
        LDRH     r7,[r4],r12
        LDRH     r8,[r4],r12
        LDRH     r9,[r4],r12
        ORR      r6,r6,r7,LSL #16
        ORR      r7,r8,r9,LSL #16
        SUB      r4,r4,#0x1500
        STMIA    r5,{r6,r7}
        SUB      r5,r5,#0x138
        SUB      r4,r4,#0xfe
        BGT      .L1.52
        SUBS     r3,r3,#1
        BGT      .L1.16
        LDMFD    sp!,{r4-r10}
        BX       lr
	.size	opl_rotate270_u16_qcif, . - opl_rotate270_u16_qcif
