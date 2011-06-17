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
#include "crm_regs.h"

#define ARM_CTRL_DCACHE     1 << 2
#define ARM_CTRL_ICACHE     1 << 12
#define ARM_AUXCR_L2EN      1 << 1

/*
 *  mx50_suspend
 *
 *  Suspend the processor (eg, wait for interrupt).
 *  Set the DDR into Self Refresh
 *  IRQs are already disabled.
 */
ENTRY(mx50_suspend)
    stmfd   sp!, {r4,r5,r6,r7,r8, r9,r10,r11, r12}     @ Save registers

    mov    r6, r0                       @save databahn address
    mov    r8, r1                       @save ccm base address
    mov    r12, r2                     @save pll1 base address

    ldr      r0, [r6]
    ldr      r0, [r8]
    ldr      r0, [r12]

/* Before putting DDR into self-refresh, make sure
    any LPM mode that the DDR might be in is exited.
*/
    /* If Databahn is in LPM4, exit that mode first. */
    ldr     r0,[r6, #0x50]             @Store LPM mode in r8
    bic     r0, r0, #0x1F
    str     r0,[r6, #0x50]


    /* Disable L1 caches */
    mrc     p15, 0, r0, c1, c0, 0    @ R0 = system control reg
    bic     r0, r0, #ARM_CTRL_ICACHE @ Disable ICache
    bic     r0, r0, #ARM_CTRL_DCACHE @ Disable DCache
    mcr     p15, 0, r0, c1, c0, 0    @ Update system control reg

    mrc     p15, 1, r0, c0, c0, 1    @ Read CLIDR
    ands    r3, r0, #0x7000000    @ Isolate level of coherency
    mov     r3, r3, lsr #23           @ Cache level value (naturally aligned)
    beq     FinishedClean
    mov     r10, #0
Loop1Clean:
    add     r2, r10, r10, lsr #1    @ Work out cache level
    mov     r1, r0, lsr r2             @ R0 bottom 3 bits = Cache Type
                                            @ for this level
    and     r1, r1, #7                 @ Get those 3 bits alone
    cmp     r1, #2
    blt     SkipClean                  @ No cache or only instruction cache
                                            @ at this level
    mcr     p15, 2, r10, c0, c0, 0  @ Write the Cache Size selection register
    mov     r1, #0
    .long   0xF57FF06F              @ ISB
    mrc     p15, 1, r1, c0, c0, 0   @ Reads current Cache Size ID register
    and     r2, r1, #7                 @ Extract the line length field
    add     r2, r2, #4                 @ Add 4 for the line length offset
                                            @ (log2 16 bytes)
    ldr     r4, =0x3FF
    ands    r4, r4, r1, lsr #3      @ R4 is the max number on the
                                           @ way size (right aligned)
    clz     r5, r4                       @ R5 is the bit position of the way
                                           @ size increment
    ldr     r7, =0x00007FFF
    ands    r7, r7, r1, lsr #13     @ R7 is the max number of the index
                                           @ size (right aligned)
Loop2Clean:
    mov     r9, r4                     @ R9 working copy of the max way size
                                           @ (right aligned)
Loop3Clean:
    orr     r11, r10, r9, lsl r5      @ Factor in the way number and cache
                                           @ number into R11
    orr     r11, r11, r7, lsl r2      @ Factor in the index number
    mcr     p15, 0, r11, c7, c14, 2 @ Clean and invalidate by set/way
    subs    r9, r9, #1               @ Decrement the way number
    bge     Loop3Clean
    subs    r7, r7, #1               @ Decrement the index
    bge     Loop2Clean
SkipClean:
    add     r10, r10, #2            @ Increment the cache number
    cmp     r3, r10
    bgt     Loop1Clean

FinishedClean:

    /* Disable L2 cache */
    mrc     p15, 0, r0, c1, c0, 1   @ R0 = auxiliary control reg
    bic     r0, r0, #ARM_AUXCR_L2EN @ Disable L2 cache
    mcr     p15, 0, r0, c1, c0, 1   @ Update aux control reg

/* Wait for the databahn to idle
     Meaning, no access to the databahn is
     being made.
*/
EnterWFI:
    ldr     r0,[r6, #0x13c]
    and    r0, r0, #0x100
    ldr     r2, =0x100
    cmp     r0, r2
    beq     EnterWFI

    /* Enter self-refresh mode */
    ldr     r0,[r6, #0x4c]
    orr     r0,r0,#0x1
    str     r0,[r6, #0x4c]

LoopCKE0:
    /* Wait for CKE = 0 */
    ldr     r0,[r6, #0xfc]
    and    r0, r0, #0x10000
    ldr     r2, =0x10000
    cmp     r0, r2
    beq     LoopCKE0

    /* Stop controller */
    ldr     r0,[r6]
    bic     r0, r0, #0x1
    str     r0,[r6]

    /* Apply the PLL issue workaround.
       DDR is in self-refresh. So no need to move DDR clock.
       1. Change ARM clock to be sourced from PLL2.
       2. Update MFN to transition to PLL1 to 864 MHz by applying the
          following factors:
          MFI = 8, MFN = 179, MFD = 179. PLL1 Freq = ~ 864MHz
       3. Request PLL to load new MFN using DP_CONFIG (LDREQ bit).
       4. No need to wait for new PLL rate.  PLL will be disabled
           during suspend.
     */

   /* Make sure Step-clk is sourced from 24MHz. */
    ldr      r1, [r8, #0x0c]
    bic      r1, r1, #(0x3 << 7)
    str      r1, [r8, #0x0c]

    /* Set ARM to be sourced from Step-clk. */
    ldr     r1, [r8, #0x0c]
    orr     r1, r1, #0x4
    str     r1, [r8, #0x0c]

    /* Now do the MFN changes to relock PLL1 at 864MHz. */
    ldr     r1, [r12, #MXC_PLL_DP_CONFIG]
    bic     r1, r1, #0x2
    str     r1, [r12, #MXC_PLL_DP_CONFIG] /* disable auto-restart AREN bit */

   /* MFI = 8, MFN = 180, MFD = 179. PLL1 Freq = ~ 864MHz. */
    ldr     r1, =0x80
    str     r1, [r12, #MXC_PLL_DP_OP]
    str     r1, [r12, #MXC_PLL_DP_HFS_OP]

    ldr  	r1, =180
    str     r1, [r12, #MXC_PLL_DP_MFN]
    str     r1, [r12, #MXC_PLL_DP_HFS_MFN]

    ldr  	r1, =179
    str     r1, [r12, #MXC_PLL_DP_MFD]
    str     r1, [r12, #MXC_PLL_DP_HFS_MFD]

    /* Manually restart PLL1 */
    ldr     r1, =0x00001236 /* Set PLM =1, manual restart and enable PLL*/
    str     r1, [r12, #MXC_PLL_DP_CTL]

    .long     0xe320f003              @ Opcode for WFI

    /* Continue DPLL issue workaround.
      5. System will resume with PLL1 locked at 864 MHz.
          ARM will be sourced from 24MHz OSC.
      6. Update MFN to transition to 800 MHz by applying
          the following factor:
          MFN = 60
      7. Request PLL to load new MFN using DP_CONFIG (LDREQ bit).
      8. Wait for acknowledge of new MFN factor from
          PLL by polling DP_CONFIG (LDREQ bit).
      9. PLL1 will now be locked at 800 MHz.
      10. Delay 4 usec to avoid PLL instability window
      11. Move ARM clock to be sourced from PLL1.
    */

    /* Make sure the PLL is locked. */
 1:    ldr    r1, [r12, #MXC_PLL_DP_CTL]
	ands  r1, r1, #0x1
	beq   1b

    /* Set PLL1 to 800MHz, MFN = 60. */
    ldr    r1, =60
    str   r1, [r12, #MXC_PLL_DP_MFN]
    str   r1, [r12, #MXC_PLL_DP_HFS_MFN]

    /* Set up the LDREQ */
    ldr    r1, [r12, #MXC_PLL_DP_CONFIG]
    orr    r1, r1, #1
    str    r1, [r12, #MXC_PLL_DP_CONFIG]

   /* Wait for LDREQ bit to clear. */
2: ldr    r1, [r12, #MXC_PLL_DP_CONFIG]
    tst  r1, #1
    bne   2b

    /* Wait for ~4 us. */
    mov   r1, #100
3: subs   r1, r1, #1
    bge    3b

    /* Move ARM back to PLL1 */
    ldr     r1, [r8, #0x0c]
    bic     r1, r1, #0x4
    str     r1, [r8, #0x0c]

    /* Start DDR controller */
    ldr     r0,[r6]
    orr     r0,r0,#0x1
    str     r0,[r6]

LoopPHY:
    /* Wait for PHY ready */
    ldr     r0,[r6, #0x264]
    and    r0, r0, #0xfffffffe
    ldr      r2, =0x0
    cmp    r0, r2
    beq     LoopPHY

    /*Leave self-refresh mode */
    ldr     r0,[r6, #0x4c]
    and    r0,r0,#0xfffffffe
    str     r0,[r6, #0x4c]

LoopCKE1:
    /*Wait for CKE = 1 */
    ldr     r0,[r6, #0xfc]
    and    r0, r0, #0x10000
    ldr     r2, =0x10000
    cmp     r0, r2
    bne     LoopCKE1

    mov     r0, #0
    mcr     p15, 0, r0, c7, c5, 0      @ Invalidate inst cache

/* Invalidate data caches */
    mrc     p15, 1, r0, c0, c0, 1      @ Read CLIDR
    ands    r3, r0, #0x7000000      @ Isolate level of coherency
    mov     r3, r3, lsr #23             @ Cache level value (naturally aligned)
    beq     FinishedInvalidate
    mov     r10, #0
Loop1Invalidate:
    add     r2, r10, r10, lsr #1       @ Work out cache level
    mov     r1, r0, lsr r2               @ R0 bottom 3 bits = Cache
                                              @ Type for this level
    and     r1, r1, #7                   @ Get those 3 bits alone
    cmp     r1, #2
    blt     SkipInvalidate              @ No cache or only instruction cache
                                              @at this level
    mcr     p15, 2, r10, c0, c0, 0   @ Write the Cache Size selection register
    mov     r1, #0
    .long   0xF57FF06F                @ ISB
    mrc     p15, 1, r1, c0, c0, 0     @ Reads current Cache Size ID register
    and     r2, r1, #7                  @ Extract the line length field
    add     r2, r2, #4                  @ Add 4 for the line length offset
                                             @(log2 16 bytes)
    ldr     r4, =0x3FF
    ands    r4, r4, r1, lsr #3        @ R4 is the max number on the way
                                             @size (right aligned)
    clz     r5, r4                         @ R5 is the bit position of the way
                                             @ size increment
    ldr     r7, =0x00007FFF
    ands    r7, r7, r1, lsr #13     @ R7 is the max number of the
                                           @ index size (right aligned)
Loop2Invalidate:
    mov     r9, r4                    @ R9 working copy of the max way
                                          @ size (right aligned)
Loop3Invalidate:
    orr     r11, r10, r9, lsl r5     @ Factor in the way number and cache
                                          @ number into R11
    orr     r11, r11, r7, lsl r2     @ Factor in the index number
    mcr     p15, 0, r11, c7, c6, 2  @ Invalidate by set/way
    subs    r9, r9, #1              @ Decrement the way number
    bge     Loop3Invalidate
    subs    r7, r7, #1              @ Decrement the index
    bge     Loop2Invalidate
SkipInvalidate:
    add     r10, r10, #2           @ Increment the cache number
    cmp     r3, r10
    bgt     Loop1Invalidate

FinishedInvalidate:

    /* Enable L2 cache */
    mrc     p15, 0, r0, c1, c0, 1   @ R0 = auxiliary control reg
    orr     r0, r0, #ARM_AUXCR_L2EN @ Enable L2 cache
    mcr     p15, 0, r0, c1, c0, 1   @ Update aux control reg

    /* Enable L1 caches */
    mrc     p15, 0, r0, c1, c0, 0    @ R0 = system control reg
    orr     r0, r0, #ARM_CTRL_ICACHE @ Enable ICache
    orr     r0, r0, #ARM_CTRL_DCACHE @ Enable DCache
    mcr     p15, 0, r0, c1, c0, 0    @ Update system control reg

    /* Restore registers */
    ldmfd sp!, {r4,r5,r6,r7,r8,r9,r10,r11, r12}
    mov     pc, lr

    .type   mx50_do_suspend, #object
ENTRY(mx50_do_suspend)
    .word   mx50_suspend
    .size   mx50_suspend, . - mx50_suspend
