/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/linkage.h>
#include <linux/init.h>

ENTRY(v7_invalidate_l1)
        mov     r0, #0
        mcr     p15, 2, r0, c0, c0, 0
        mrc     p15, 1, r0, c0, c0, 0

        ldr     r1, =0x7fff
        and     r2, r1, r0, lsr #13

        ldr     r1, =0x3ff

        and     r3, r1, r0, lsr #3  @ NumWays - 1
        add     r2, r2, #1          @ NumSets

        and     r0, r0, #0x7
        add     r0, r0, #4          @ SetShift

        clz     r1, r3              @ WayShift
        add     r4, r3, #1          @ NumWays
1:      sub     r2, r2, #1          @ NumSets--
        mov     r3, r4              @ Temp = NumWays
2:      subs    r3, r3, #1          @ Temp--
        mov     r5, r3, lsl r1
        mov     r6, r2, lsl r0
        orr     r5, r5, r6          @ Reg = (Temp<<WayShift)|(NumSets<<SetShift)
        mcr     p15, 0, r5, c7, c6, 2
        bgt     2b
        cmp     r2, #0
        bgt     1b
        dsb
        isb
        mov     pc, lr
ENDPROC(v7_invalidate_l1)

	__CPUINIT
ENTRY(mx6_secondary_startup)

	/* Invalidate L1 I-cache first */
	mov	r1,	#0x0
	mcr p15, 0, r1, c7, c5, 0 @ Invalidate I-Cache
	/* Invalidate L1 D-cache */
	bl      v7_invalidate_l1
	/* Set ARM working mode */
	msr     cpsr_fsxc, #0xd3

	mrc p15, 0, r0, c0, c0, 5
	and r0, r0, #15
	ldr r1, = 0x020d8020
	add r1, r0, LSL#3

	/*Clear SRC_GPR register */
	mov r0, #0
	str r0, [r1]
	str r0, [r1, #0x4]

	/* Jump to secondary_startup */
	b secondary_startup

ENDPROC(mx6_secondary_startup)
