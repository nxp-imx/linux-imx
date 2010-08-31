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
 *  mx50_ddr_freq_change
 *
 *  Idle the processor (eg, wait for interrupt).
 *  Make sure DDR is in self-refresh.
 *  IRQs are already disabled.
 */
ENTRY(mx50_ddr_freq_change)
    stmfd   sp!, {r3,r4,r5,r6}      @ Save registers

    mov    r6, r0                       @save CCM address
    mov    r5, r1                       @save DataBahn address
    mov    r4, r2                       @save new freq requested

    ldr     r0, [r6, #0x90]

    /* If Databahn is in LPM4, exit that mode first. */
    ldr     r1,[r5, #0x50]             @Store LPM mode in r1.
    mov   r0, r1
    bic     r0, #0x1F
    str     r0,[r5, #0x50]

    /*
     * Make sure the DDR is self-refresh, before switching its frequency
     * and clock source
     */

    /* Step 1: Enter self-refresh mode */
    ldr     r0,[r5, #0x4c]
    orr     r0,r0,#0x1
    str     r0,[r5, #0x4c]

     /* Step 2: Poll the CKE_STATUS bit. */
LoopCKE0:
    /* Wait for CKE = 0 */
    ldr     r0,[r5, #0xfc]
    and     r0, r0, #0x10000
    ldr     r2, =0x10000
    cmp     r0, r2
    beq     LoopCKE0

    /* Step 3: Mask the DLL lock state change, set bit 8 in int_mask. */
    ldr    r0, [r5, #0xac]
    orr    r0, r0, #0x100
    str    r0, [r5, #0xac]

    /* Step 4: Stop the Controller. */
    ldr     r0,[r5]
    bic     r0, r0, #0x1
    str     r0,[r5]

    /* Step 5: Clear the DLL lock state change bit 8 in int_ack */
    ldr     r0, [r5, #0xa8]
    orr     r0, r0, #0x1000000
    str     r0, [r5, #0xa8]

    /* Step 6: Clear the interrupt mask for DLL lock state.
      * Bit 8 in int_mask */
    ldr     r0, [r5, #0xac]
    bic     r0, r0, #0x100
    str     r0, [r5, #0xac]

     /* Change the freq now */
     /* If the freq req is below 200MHz, set DDR to synchronous mode.
      * else set to async mode. */
    ldr     r0, =200000000
    cmp   r4, r0
    bgt    Async_Mode

     /* Set the DDR to be Synchronous
     mode. */
    /* Set the Databahn to sync mode. */
    ldr     r0, [r5, #0xdc]
    orr     r0, r0, #0x30000
    str    r0, [r5, #0xdc]

    /* Turn OFF the DDR_CKLGATE_MASK in MXC_CCM_DDR */
    ldr     r0, [r6, #0x98]
    bic     r0, r0, #0xC0000000
    str     r0, [r6, #0x98]

    /* Check if XTAL can source the DDR. */
    ldr     r0, =24000000
    cmp   r4, r0
    ble    databahn_ddr_24

    /*Source DDR from PLL1.  Setup the dividers accordingly. */
    ldr    r0, =800000000
    ldr    r3, =1
Loop1:
    sub    r0, r0, r4
    cmp   r0, r4
    blt     Div_Found
    add    r3, r3, #1
    bgt     Loop1

Div_Found:
    ldr     r0, [r6, #0x94]
    bic     r0, r0, #0x3f
    orr     r0, r0, r3
    str     r0, [r6, #0x94]
    /* Set the DDR to sourced from PLL1 in sync path */
    ldr     r0, [r6, #0x90]
    orr     r0, r0, #0x3
    str     r0, [r6, #0x90]

    /* Turn OFF the DDR_CKLGATE_MASK in MXC_CCM_DDR */
    ldr     r0, [r6, #0x98]
    bic     r0, r0, #0xC0000000
    str     r0, [r6, #0x98]

    ldr     r0, =24000000
    cmp   r4, r0
    beq   databahn_ddr_24

    b       Ddr_not_24

databahn_ddr_24:
    ldr    r0, =0x00000003
    str    r0, [r5, #0x08]
    ldr    r0, =0x000012c0
    str    r0, [r5, #0x0c]

    ldr    r0, =0x00000018
    str    r0, [r5, #0x10]
    ldr    r0, =0x000000f0
    str    r0, [r5, #0x14]
    ldr    r0, =0x02010b0c
    str    r0, [r5, #0x18]
    ldr    r0, =0x02020102
    str    r0, [r5, #0x1c]

    ldr    r0, =0x05010102
    str    r0, [r5, #0x20]
    ldr    r0, =0x01000103
    str    r0, [r5, #0x28]
    ldr    r0, =0x04030101
    str    r0, [r5, #0x2c]

    ldr    r0, =0x00000202
    str    r0, [r5, #0x34]
    ldr    r0, =0x00000001
    str    r0, [r5, #0x38]
    ldr    r0, =0x00000401
    str    r0, [r5, #0x3c]

    ldr    r0, =0x00050056
    str    r0, [r5, #0x40]
    ldr    r0, =0x00040004
    str    r0, [r5, #0x48]

    ldr    r0, =0x00040022
    str    r0, [r5, #0x6c]

    ldr    r0, =0x00040022
    str    r0, [r5, #0x78]

    ldr    r0, =0x00180000
    str    r0, [r5, #0x80]
    ldr    r0, =0x00000009
    str    r0, [r5, #0x84]
    ldr    r0, =0x02400003
    str    r0, [r5, #0x88]
    ldr    r0, =0x01000200
    str    r0, [r5, #0x8c]

    ldr    r0, =0x00000000
    str    r0, [r5, #0xcc]

    ldr    r0, =0x01000201
    str    r0, [r5, #0xd0]
    ldr    r0, =0x01010301
    str    r0, [r5, #0xd4]
    ldr    r0, =0x00000101
    str    r0, [r5, #0xd8]

    ldr    r0, =0x02000602
    str    r0, [r5, #0x104]
    ldr    r0, =0x00560000
    str    r0, [r5, #0x108]
    ldr    r0, =0x00560056
    str    r0, [r5, #0x10c]

    ldr    r0, =0x00560056
    str    r0, [r5, #0x110]
    ldr    r0, =0x03060056
    str    r0, [r5, #0x114]

    /* Set the Databahn DLL in bypass mode */
    /* PHY Register settings. */
    ldr    r0, =0x00000100
    str    r0, [r5, #0x200]
    ldr    r0, =0x000f1100
    str    r0, [r5, #0x204]
    ldr    r0, =0xf3003a27
    str    r0, [r5, #0x208]
    ldr    r0, =0x074002c1
    str    r0, [r5, #0x20c]

    ldr    r0, =0xf3003a27
    str    r0, [r5, #0x210]
    ldr    r0, =0x074002c1
    str    r0, [r5, #0x214]
    ldr    r0, =0xf3003a27
    str    r0, [r5, #0x218]
    ldr    r0, =0x074002c1
    str    r0, [r5, #0x21c]

    ldr    r0, =0xf3003a27
    str    r0, [r5, #0x220]
    ldr    r0, =0x074002c1
    str    r0, [r5, #0x224]
    ldr    r0, =0xf3003a27
    str    r0, [r5, #0x228]
    ldr    r0, =0x074002c1
    str    r0, [r5, #0x22c]

    ldr    r0, =0x00810004
    str    r0, [r5, #0x234]
    ldr     r0, =0x30219f14
    str    r0, [r5, #0x238]
    ldr    r0, =0x00219f01
    str    r0, [r5, #0x23c]

    ldr     r0, =0x30219f14
    str    r0, [r5, #0x240]
    ldr    r0, =0x00219f01
    str    r0, [r5, #0x244]
    ldr     r0, =0x30219f14
    str    r0, [r5, #0x248]
    ldr    r0, =0x00219f01
    str    r0, [r5, #0x24c]

    ldr     r0, =0x30219f14
    str    r0, [r5, #0x250]
    ldr    r0, =0x00219f01
    str    r0, [r5, #0x254]
    ldr     r0, =0x30219f14
    str    r0, [r5, #0x258]
    ldr    r0, =0x00219f01
    str    r0, [r5, #0x25c]

    b       Setup_Done

Async_Mode:
    /* Set the Databahn to async mode. */
    ldr     r0, [r5, #0xdc]
    and    r0, r0, #0xfffcffff
    str    r0, [r5, #0xdc]

    /*Source DDR from PLL1.  Setup the dividers accordingly. */
    ldr    r0, =800000000
    ldr    r3, =1
Loop2:
    sub    r0, r0, r4
    cmp   r0, r4
    blt     Div_Found1
    add    r3, r3, #1
    bgt     Loop2

Div_Found1:
    ldr     r0, [r6, #0x98]
    bic    r0, r0, #0x3f
    orr     r0, r0, r3
    str     r0, [r6, #0x98]

    /* Set the DDR to sourced from PLL1 in async path */
    ldr     r0, [r6, #0x98]
    bic     r0, r0, #0x40
    str     r0, [r6, #0x98]

    /* Turn ON the DDR_CKLGATE_MASK in MXC_CCM_DDR */
    ldr     r0, [r6, #0x98]
    orr     r0, r0, #0x40000000
    str     r0, [r6, #0x98]

    ldr     r0, =24000000
    cmp   r4, r0
    beq    databahn_ddr_24

Ddr_not_24:
    ldr     r0, =0x0000001b
    str     r0, [r5, #0x8]
    ldr     r0, =0x0000d056
    str     r0, [r5, #0xc]

    ldr     r0, =0x0000010b
    str     r0, [r5, #0x10]
    ldr     r0, =0x00000a6b
    str     r0, [r5, #0x14]
    ldr     r0, =0x02020d0c
    str     r0, [r5, #0x18]
    ldr     r0, =0x0c110302
    str     r0, [r5, #0x1c]

    ldr     r0, =0x05020503
    str     r0, [r5, #0x20]
    ldr     r0, =0x01000403
    str     r0, [r5, #0x28]
    ldr     r0, =0x09040501
    str     r0, [r5, #0x2c]

    ldr     r0, =0x00000e02
    str     r0, [r5, #0x34]
    ldr     r0, =0x00000006
    str     r0, [r5, #0x38]
    ldr     r0, =0x00002301
    str     r0, [r5, #0x3c]

    ldr     r0, =0x00050408
    str     r0, [r5, #0x40]
    ldr     r0, =0x00260026
    str     r0, [r5, #0x48]

    ldr     r0, =0x00040042
    str     r0, [r5, #0x6c]

    ldr     r0, =0x00040042
    str     r0, [r5, #0x78]

    ldr     r0, =0x010b0000
    str     r0, [r5, #0x80]
    ldr     r0, =0x00000060
    str     r0, [r5, #0x84]
    ldr     r0, =0x02400018
    str     r0, [r5, #0x88]
    ldr     r0, =0x01000e00
    str     r0, [r5, #0x8c]

    ldr     r0, =0x01000000
    str     r0, [r5, #0xcc]

    ldr     r0, =0x01000201
    str     r0, [r5, #0xd0]
    ldr     r0, =0x00000200
    str     r0, [r5, #0xd4]
    ldr     r0, =0x00000102
    str     r0, [r5, #0xd8]

    ldr     r0, =0x02000802
    str     r0, [r5, #0x104]
    ldr     r0, =0x04080000
    str     r0, [r5, #0x108]
    ldr     r0, =0x04080408
    str     r0, [r5, #0x10c]

    ldr     r0, =0x04080408
    str     r0, [r5, #0x110]
    ldr     r0, =0x03060408
    str     r0, [r5, #0x114]

    /* PHY setting for 266MHz */
    ldr     r0, =0x00000000
    str     r0, [r5, #0x200]
    ldr     r0, =0x00000000
    str     r0, [r5, #0x204]
    ldr     r0, =0xf5003a27
    str     r0, [r5, #0x208]

    ldr     r0, =0xf5003a27
    str     r0, [r5, #0x210]
    ldr     r0, =0xf5003a27
    str     r0, [r5, #0x218]

    ldr     r0, =0xf5003a27
    str     r0, [r5, #0x220]
    ldr     r0, =0xf5003a27
    str     r0, [r5, #0x228]

    ldr     r0, =0x074002e1
    str     r0, [r5, #0x20c]
    ldr     r0, =0x074002e1
    str     r0, [r5, #0x214]
    ldr     r0, =0x074002e1
    str     r0, [r5, #0x21c]
    ldr     r0, =0x074002e1
    str     r0, [r5, #0x224]
    ldr     r0, =0x074002e1
    str     r0, [r5, #0x22c]

    ldr     r0, =0x00810006
    str     r0, [r5, #0x234]
    ldr     r0, =0x20099414
    str     r0, [r5, #0x238]
    ldr     r0, =0x000a1401
    str     r0, [r5, #0x23c]

    ldr     r0, =0x20099414
    str     r0, [r5, #0x240]
    ldr     r0, =0x000a1401
    str     r0, [r5, #0x244]
    ldr     r0, =0x20099414
    str     r0, [r5, #0x248]
    ldr     r0, =0x000a1401
    str     r0, [r5, #0x24c]

    ldr     r0, =0x20099414
    str     r0, [r5, #0x250]
    ldr     r0, =0x000a1401
    str     r0, [r5, #0x254]
    ldr     r0, =0x20099414
    str     r0, [r5, #0x258]
    ldr     r0, =0x000a1401
    str     r0, [r5, #0x25c]

    b       Setup_Done


Setup_Done:
    /* Start controller */
    ldr     r0,[r5]
    orr     r0, r0,#0x1
    str     r0,[r5]

    /* Poll the DLL lock state change in int_status reg*/
    /* DLL is bypassed in the 24MHz mode, so no waiting for DLL to lock. */
    ldr     r0, =24000000
    cmp   r4, r0
    beq    Exit_Self_Refresh
DllLock:
    ldr     r0, [r5, #0xa8]
    and    r0, r0, #0x100
    ldr     r2, =0x100
    cmp   r0, r2
    bne    DllLock

    /*Leave self-refresh mode */
Exit_Self_Refresh:
    ldr     r0,[r5, #0x4c]
    and    r0,r0,#0xfffffffe
    str     r0,[r5, #0x4c]

LoopCKE1:
    /*Wait for CKE = 1 */
    ldr     r0,[r5, #0xfc]
    and    r0, r0, #0x10000
    ldr     r2, =0x10000
    cmp     r0, r2
    bne     LoopCKE1

    /* Put the databahn back to into the LPM mode. */
    str     r1,[r5, #0x50]

    /* Restore registers */
    ldmfd sp!, {r3,r4,r5,r6}
    mov     pc, lr

    .type   mx50_do_ddr_freq_change, #object
ENTRY(mx50_do_ddr_freq_change)
    .word   mx50_ddr_freq_change
    .size    mx50_ddr_freq_change, . - mx50_ddr_freq_change
