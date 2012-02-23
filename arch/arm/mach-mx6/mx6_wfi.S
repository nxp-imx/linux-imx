/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <mach/hardware.h>

/*
 *  mx6_wait
 *
 *  Idle the processor (eg, wait for interrupt).
 *  Make sure DDR is in self-refresh.
 *  IRQs are already disabled.
 */
ENTRY(mx6_wait)

    push {r4, r5, r6}

      ldr     r2, =ANATOP_BASE_ADDR
      add   r2, r2, #PERIPBASE_VIRT

    /* get the flags variables into the cache */
	ldr     r3, [r0]

    /* get CPU ID */
    mrc     p15,0,r5,c0,c0,5
    and     r5, r5, #0x3

    mov r4,#0xff
    strb r4,[r0,r5]

    dsb

    mvn r4, #0x0
    ldr r3, [r0]
    cmp r3, r4
    bne DO_WFI

    mov r4, #0x1
    ldrex r3, [r1]
    cmp r3, #0x0
    strexeq r3, r4, [r1]
    cmpeq r3, #0x0
    bne DO_WFI

    mov r3, #0xff

    ldr     r6, =(1 << 16)
    str     r6, [r2, #0x04]

    /* dmb */

    str r3, [r1]

    dsb

    mvn r4, #0x0
    ldr r3, [r0]
    cmp r3, r4
    movne r3, #0x0
    strne r6, [r2, #0x08]
    strne r3, [r1]

DO_WFI:
	dsb

	wfi

    mov r4, #0x0
    strb r4, [r0, r5]

    dsb

    ldr r3, [r1]
    cmp r3, #0xff
    bne DONE

    mov r4, #0x0
    ldr     r6, =(1 << 16)
    str r6, [r2, #0x08]

    mov r3, #0x0
    str r3, [r1]

DONE:

    pop {r4,r5, r6}

    /* Restore registers */
    mov     pc, lr

    .type   mx6_do_wait, #object
ENTRY(mx6_do_wait)
    .word   mx6_wait
    .size    mx6_wait, . - mx6_wait
