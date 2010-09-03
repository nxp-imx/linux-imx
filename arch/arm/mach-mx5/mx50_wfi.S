/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

/*
 *  mx50_wait
 *
 *  Idle the processor (eg, wait for interrupt).
 *  Make sure DDR is in self-refresh.
 *  IRQs are already disabled.
 */
ENTRY(mx50_wait)
    stmfd   sp!, {r3,r4,r5,r6,r7,r8,r9,r10,r11}     @ Save registers

    mov    r6, r0                       @save CCM address
    mov    r5, r1                       @save DataBahn address

    /*
     * Make sure the DDR is self-refresh, before setting the clock bits.
     */

     /* Step 2: Poll the CKE_STATUS bit. */
LoopCKE0:
    /* Wait for CKE = 0 */
    ldr     r0,[r5, #0xfc]
    and     r0, r0, #0x10000
    ldr     r2, =0x10000
    cmp     r0, r2
    beq     LoopCKE0

    /* Set the DDR_CLKGATE to 0x1. */
    ldr      r0, [r6, #0x98]
    bic     r0, r0, #0x80000000
    str     r0, [r6, #0x98]

    .long     0xe320f003              @ Opcode for WFI

    /* Set the DDR_CLKGATE to 0x3. */
    ldr      r0, [r6, #0x98]
    orr     r0, r0, #0xC0000000
    str     r0, [r6, #0x98]

    /* Restore registers */
    ldmfd sp!, {r3,r4,r5,r6,r7,r8,r9,r10,r11}
    mov     pc, lr

    .type   mx50_do_wait, #object
ENTRY(mx50_do_wait)
    .word   mx50_wait
    .size    mx50_wait, . - mx50_wait
