/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
    mov    r7, r2                       @save sys_clk usecount

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

    /* Check if Databahn is in SYNC or ASYNC mode. */
    ldr      r4, [r5, #0xdc]
    and     r4, r4, #0x30000
    cmp    r4, #0x30000
    beq     Sync_mode

    /* Set the DDR_CLKGATE to 0x1. */
    ldr      r0, [r6, #0x98]
    bic     r0, r0, #0x80000000
    str     r0, [r6, #0x98]

    .long     0xe320f003              @ Opcode for WFI

    /* Set the DDR_CLKGATE to 0x3. */
    ldr      r0, [r6, #0x98]
    orr     r0, r0, #0xC0000000
    str     r0, [r6, #0x98]
    b       Wfi_Done

Sync_mode:
    /* If usecount of sys_clk is greater than 0, donot gate it. */
    cmp    r7, #0
    bgt    do_wfi

    /* Check if PLL1 is sourcing SYS_CLK. */
    ldr     r5, [r6, #0x90]
    and    r5, r0, #0x1
    cmp   r5, #0x1
    beq    pll1_source

    /* Set the SYS_XTAL_CLKGATE to 0x1. */
    ldr      r0, [r6, #0x94]
    bic     r0, r0, #0x80000000
    str     r0, [r6, #0x94]

    /* Set the SYS_XTAL_DIV to 0xF (1.6MHz) to reduce power.
     * since this clock is not gated when ARM is in WFI.
     */

    ldr      r0, [r6, #0x94]
    orr      r0, r0, #0x3c0
    str      r0, [r6, #0x94]

    b       do_wfi
pll1_source:
    /* Set the SYS_PLL_CLKGATE to 0x1. */
    ldr      r0, [r6, #0x94]
    bic     r0, r0, #0x40000000
    str     r0, [r6, #0x94]

do_wfi:
    .long     0xe320f003              @ Opcode for WFI

    cmp    r7, #0
    bgt     Wfi_Done

     cmp    r5, #1
     beq    pll1_source1
    /* Set the SYS_XTAL_DIV to 24MHz.*/
    ldr      r0, [r6, #0x94]
    bic      r0, r0, #0x3c0
    orr      r0, r0, #0x40
    str      r0, [r6, #0x94]

    /* Set the SYS_XTAL_CLKGATE to 0x3. */
    ldr      r0, [r6, #0x94]
    orr     r0, r0, #0xC0000000
    str     r0, [r6, #0x94]
    b       Wfi_Done

pll1_source1:
    /* Set the SYS_PLL_CLKGATE to 0x3. */
    ldr      r0, [r6, #0x94]
    orr     r0, r0, #0x30000000
    str     r0, [r6, #0x94]

Wfi_Done:
    /* Restore registers */
    ldmfd sp!, {r3,r4,r5,r6,r7,r8,r9,r10,r11}
    mov     pc, lr

    .type   mx50_do_wait, #object
ENTRY(mx50_do_wait)
    .word   mx50_wait
    .size    mx50_wait, . - mx50_wait
