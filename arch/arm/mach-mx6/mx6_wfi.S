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

    push {r4, r5, r6, r7, r8, r9}

    mov    r7, r2       /* Store the arm_podf to be used. */
    mov   r6, r3

    ldr     r2, =ANATOP_BASE_ADDR
    add   r2, r2, #PERIPBASE_VIRT

    ldr     r8, =CCM_BASE_ADDR
    add   r8, r8, #PERIPBASE_VIRT


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

    /* Check to see if we need to switch to 24MHz */
    cmp    r7, #0
    bne    use_podf
    /* Switch ARM to PLL1 output. */
    /* PLL1 should already be in bypass state. */
    ldr    r6, [r8, #0x0C]
    bic    r6, r6, #0x04
    str    r6, [r8, #0x0C]
    b      cont

use_podf:
    /* Change ARM_PODF to the max possible podf
     * so that ARM_CLK to IPG_CLK is in 12:5 ratio.
     */
    str    r7, [r8, #0x10]
    /* Loop till podf is accepted. */
podf_loop:
     ldr     r4, [r8, #0x48]
     cmp   r4, #0x0
     bne   podf_loop

    /* dmb */

cont:
    str r3, [r1]

    dsb

    mvn r4, #0x0
    ldr r3, [r0]
    cmp r3, r4
    beq   DO_WFI

    mov   r3, #0x0
    /* Switch to 24MHz or use ARM_PODF. */
    cmp   r7, #0x0
    bne    use_podf1
    /* Set pll1_sw_clk to run from STEP_CLK. */
    orr     r6, r6, #0x04
    str     r6, [r8, #0x0C]
    b       DO_WFI
use_podf1:
    str     r6, [r8, #0x10]

    str r3, [r1]

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
    cmp   r7, #0x0
    bne    use_podf2
    /* Set pll1_sw_clk to run from STEP_CLK. */
    ldr     r6, [r8, #0x0C]
    orr    r6, r6, #0x04
    str    r6, [r8, #0x0C]

    b       cont1

use_podf2:
    str     r6, [r8, #0x10]

cont1:
    mov r3, #0x0
    str r3, [r1]

DONE:

    pop {r4,r5, r6, r7, r8, r9}

    /* Restore registers */
    mov     pc, lr

    .type   mx6_do_wait, #object
ENTRY(mx6_do_wait)
    .word   mx6_wait
    .size    mx6_wait, . - mx6_wait
