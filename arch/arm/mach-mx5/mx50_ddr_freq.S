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
    stmfd   sp!, {r3,r4,r5,r6, r7}      @ Save registers

    mov    r6, r0                       @save CCM address
    mov    r5, r1                       @save DataBahn address
    mov    r4, r2                       @save new freq requested

    /* Make sure no TLB miss will occur when the DDR is in self refresh. */
    /* Invalidate TLB single entry to ensure that the address is not
      * already in the TLB.
      */
    adr   r3, LoopCKE2               @Address in this function.
    mcr   p15, 0, r3, c8, c7, 1      @ Make sure freq code address
                                             @ is not already in TLB.
    mcr   p15, 0, r6, c8, c7, 1      @ Make sure CCM address
                                             @ is not already in TLB.
    mcr   p15, 0, r5, c8, c7, 1      @ make sure Databahn address
                                             @ is not already in TLB.
    mrc   p15, 0, r0, c10, c0, 0    @ Read the TLB lockdown register
    orr    r0, r0, #1                    @ Set the Preserve bit.
    mcr   p15, 0, r0, c10, c0, 0    @ Write to the lockdown register
    ldr    r2, [r6]                        @ TLB will miss,
                                             @CCM address will be loaded
    ldr    r2, [r5]                        @ TLB will miss,
                                             @Databahn address will be loaded
    ldr    r2, [r3]                        @ TLB will miss
    mrc  p15, 0, r0, c10, c0, 0      @ Read the lockdown register
                                             @ (victim will be incremented)
    bic   r0, r0, #1                     @ Clear the preserve bit
    mcr   p15, 0, r0, c10, c0, 0     @ Write to the lockdown register.

    /* If Databahn is in LPM4, exit that mode first. */
    ldr     r1,[r5, #0x50]             @Store LPM mode in r1.
    mov   r0, r1
    bic     r0, #0x1F
    str     r0,[r5, #0x50]

LoopCKE2:
    /*Wait for CKE = 1 */
    ldr     r0,[r5, #0xfc]
    and    r0, r0, #0x10000
    ldr     r2, =0x10000
    cmp     r0, r2
    bne     LoopCKE2

/* Wait for the databahn to idle
     Meaning, no access to the databahn is
     being made.
*/
NotIdle:
    ldr     r0,[r5, #0x13c]
    and    r0, r0, #0x100
    ldr     r2, =0x100
    cmp     r0, r2
    beq     NotIdle

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
     /* If the freq req is below 24MHz, set DDR to synchronous mode.
      * else set to async mode. */
    ldr     r0, =24000000
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

    /* Check for mDDR v LPDDR2 memory type */
    ldr    r0, [r5]
    ldr    r2, =0x100
    and   r0, r0, #0xF00
    cmp   r0, r2
    beq    mddr_24

    /* LPDDR2 settings */
    ldr    r0, =0x00000003
    str    r0, [r5, #0x08]
    ldr    r0, =0x000012c0
    str    r0, [r5, #0x0c]
    ldr    r0, =0x00000018

    str    r0, [r5, #0x10]
    ldr    r0, =0x000000f0
    str    r0, [r5, #0x14]
    ldr    r0, =0x02030b0c
    str    r0, [r5, #0x18]
    ldr    r0, =0x02020104
    str    r0, [r5, #0x1c]

    ldr    r0, =0x05010102
    str    r0, [r5, #0x20]
    ldr    r0, =0x00068005
    str    r0, [r5, #0x24]
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

    /* Set TREF. */
    ldr    r0, =0x00030050
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
    ldr    r0, =0x0
    str    r0, [r5, #0x200]
    ldr    r0, =0x0
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
    ldr     r0, =0x30219fd3
    str    r0, [r5, #0x238]
    ldr    r0, =0x00219fc1
    str    r0, [r5, #0x23c]

    ldr     r0, =0x30219fd3
    str    r0, [r5, #0x240]
    ldr    r0, =0x00219fc1
    str    r0, [r5, #0x244]
    ldr     r0, =0x30219fd3
    str    r0, [r5, #0x248]
    ldr    r0, =0x00219fc1
    str    r0, [r5, #0x24c]

    ldr     r0, =0x30219fd3
    str    r0, [r5, #0x250]
    ldr    r0, =0x00219fc1
    str    r0, [r5, #0x254]
    ldr     r0, =0x30219fd3
    str    r0, [r5, #0x258]
    ldr    r0, =0x00219fc1
    str    r0, [r5, #0x25c]

    b clocking

/* mDDR settings */
mddr_24:
    ldr    r0, =0x000012c0
    str    r0, [r5, #0x08]
    ldr    r0, =0x02000000
    str    r0, [r5, #0x14]
    ldr    r0, =0x01010506
    str    r0, [r5, #0x18]
    ldr    r0, =0x01020101
    str    r0, [r5, #0x1c]

    ldr    r0, =0x02000103
    str    r0, [r5, #0x20]
    ldr    r0, =0x01069002
    str    r0, [r5, #0x24]
    ldr    r0, =0x01000101
    str    r0, [r5, #0x28]
    ldr    r0, =0x02010101
    str    r0, [r5, #0x2c]

    ldr    r0, =0x00000602
    str    r0, [r5, #0x34]
    ldr    r0, =0x00000001
    str    r0, [r5, #0x38]
    ldr    r0, =0x00000301
    str    r0, [r5, #0x3c]

    /* Set TREF. */
    ldr    r0, =0x000500b0
    str    r0, [r5, #0x40]
    ldr    r0, =0x00030003
    str    r0, [r5, #0x48]

    ldr    r0, =0x00000000
    str    r0, [r5, #0x6c]

    ldr    r0, =0x00000200
    str    r0, [r5, #0xd4]

    ldr    r0, =0x00b30000
    str    r0, [r5, #0x108]
    ldr    r0, =0x00b300b3
    str    r0, [r5, #0x10c]

    ldr    r0, =0x00b300b3
    str    r0, [r5, #0x110]
    ldr    r0, =0x010300b3
    str    r0, [r5, #0x114]

    /* Set the Databahn DLL in bypass mode */
    /* PHY Register settings. */
    ldr    r0, =0x00000100
    str    r0, [r5, #0x200]
    ldr    r0, =0x0
    str    r0, [r5, #0x204]
    ldr    r0, =0xf4003a27
    str    r0, [r5, #0x208]
    ldr    r0, =0x074002c0
    str    r0, [r5, #0x20c]

    ldr    r0, =0xf4003a27
    str    r0, [r5, #0x210]
    ldr    r0, =0x074002c0
    str    r0, [r5, #0x214]
    ldr    r0, =0xf4003a27
    str    r0, [r5, #0x218]
    ldr    r0, =0x074002c0
    str    r0, [r5, #0x21c]

    ldr    r0, =0xf4003a27
    str    r0, [r5, #0x220]
    ldr    r0, =0x074002c0
    str    r0, [r5, #0x224]
    ldr    r0, =0xf4003a27
    str    r0, [r5, #0x228]
    ldr    r0, =0x074002c0
    str    r0, [r5, #0x22c]

    ldr    r0, =0x00800005
    str    r0, [r5, #0x234]
    ldr     r0, =0x30319f14
    str    r0, [r5, #0x238]
    ldr    r0, =0x00319f01
    str    r0, [r5, #0x23c]

    ldr     r0, =0x30319f14
    str    r0, [r5, #0x240]
    ldr    r0, =0x00319f01
    str    r0, [r5, #0x244]
    ldr     r0, =0x30319f14
    str    r0, [r5, #0x248]
    ldr    r0, =0x00319f01
    str    r0, [r5, #0x24c]

    ldr     r0, =0x30319f14
    str    r0, [r5, #0x250]
    ldr    r0, =0x00319f01
    str    r0, [r5, #0x254]
    ldr     r0, =0x30319f14
    str    r0, [r5, #0x258]
    ldr    r0, =0x00319f01
    str    r0, [r5, #0x25c]

clocking:
    /* Set SYS_CLK to be sourced from 24MHz. */
    /* Set the SYS_XTAL_DIV */
    ldr    r0, [r6, #0x94]
    bic    r0, r0, #0x3c0
    orr    r0, r0, #0x40
    str    r0, [r6, #0x94]

    /* Enable SYS_XTAL_CLKGATE. */
    ldr    r0, [r6, #0x94]
    orr    r0, r0, #0xC0000000
    str    r0, [r6, #0x94]

    /* set SYS_CLK to be sourced from XTAL. */
    ldr     r0, [r6, #0x90]
    bic     r0, r0, #0x1
    str     r0, [r6, #0x90]

    /* Disable SYS_PLL_CLKGATE.*/
    ldr    r0, [r6, #0x94]
    bic    r0, r0, #0x30000000
    str    r0, [r6, #0x94]
    b       Setup_Done

Async_Mode:
    /* If SYS_CLK is running at 24MHz, increase
      * it to 200MHz.
     */
    /* r7 indicates that we are moving from 133Mhz<-> 266MHz */
    ldr     r7, =1
    ldr     r0, [r6, #0x90]
    and    r0, r0, #0x1
    cmp   r0, #0
    bne    Sys_Clk_Not_24
    ldr     r7, =0

   /* Disable SYS_PLL_CLKGATE. */
    ldr     r0, [r6, #0x94]
    bic     r0, r0, #0x30000000
    str     r0, [r6, #0x94]

    /* Set the new divider. */
    ldr     r0, [r6, #0x94]
    bic    r0, r0, #0x3f
    orr     r0, r0, #4
    str     r0, [r6, #0x94]

   /* Enable SYS_PLL_CLKGATE. */
    ldr     r0, [r6, #0x94]
    orr     r0, r0, #0x30000000
    str     r0, [r6, #0x94]

    /* SYS_CLK to be sourced from PLL1. */
    ldr     r0, [r6, #0x90]
    orr     r0, r0, #0x3
    str     r0, [r6, #0x90]

   /* Disable SYS_XTAL_CLKGATE. */
    ldr     r0, [r6, #0x94]
    bic     r0, r0, #0xC0000000
    str     r0, [r6, #0x94]

Sys_Clk_Not_24:
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
    /* Turn OFF the DDR_CKLGATE_MASK in MXC_CCM_DDR */
    ldr     r0, [r6, #0x98]
    bic     r0, r0, #0xC0000000
    str     r0, [r6, #0x98]

    ldr     r0, [r6, #0x98]
    bic     r0, r0, #0x3f
    orr     r0, r0, r3
    str     r0, [r6, #0x98]

    /* Set the DDR to sourced from PLL1 in async path */
    ldr     r0, [r6, #0x98]
    bic     r0, r0, #0x40
    str     r0, [r6, #0x98]

    /* Turn ON the DDR_CKLGATE_MASK in MXC_CCM_DDR */
    ldr     r0, [r6, #0x98]
    orr     r0, r0, #0xC0000000
    str     r0, [r6, #0x98]

    ldr     r0, =24000000
    cmp   r4, r0
    beq    databahn_ddr_24

Ddr_not_24:
    /* Check for mDDR v LPDDR2 memory type */
    ldr    r0, [r5]
    ldr    r2, =0x100
    and    r0, r0, #0xF00
    cmp   r0, r2
    beq    mddr_not_24

    cmp    r7, #1
    beq    just_set_tref

    ldr     r0, =0x0000001b
    str     r0, [r5, #0x8]
    ldr     r0, =0x0000d056
    str     r0, [r5, #0xc]

    ldr     r0, =0x0000010b
    str     r0, [r5, #0x10]
    ldr     r0, =0x00000a6b
    str     r0, [r5, #0x14]
    ldr     r0, =0x02030d0c
    str     r0, [r5, #0x18]
    ldr     r0, =0x0c110304
    str     r0, [r5, #0x1c]

    ldr     r0, =0x05020503
    str     r0, [r5, #0x20]
    ldr     r0, =0x0048D005
    str     r0, [r5, #0x24]
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

just_set_tref:
    ldr     r0, =133333333
    cmp   r4, r0
    bgt    ddr_266
    ldr     r0, =0x00050180
    b       tref_done
ddr_266:
    ldr     r0, =0x00050300
tref_done:
    str     r0, [r5, #0x40]

    cmp    r7, #1
    beq    Setup_Done

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
    ldr     r0, =0x60099414
    str     r0, [r5, #0x238]
    ldr     r0, =0x000a0b01
    str     r0, [r5, #0x23c]

    ldr     r0, =0x60099414
    str     r0, [r5, #0x240]
    ldr     r0, =0x000a0b01
    str     r0, [r5, #0x244]
    ldr     r0, =0x60099414
    str     r0, [r5, #0x248]
    ldr     r0, =0x000a0b01
    str     r0, [r5, #0x24c]

    ldr     r0, =0x60099414
    str     r0, [r5, #0x250]
    ldr     r0, =0x000a0b01
    str     r0, [r5, #0x254]
    ldr     r0, =0x60099414
    str     r0, [r5, #0x258]
    ldr     r0, =0x000a0b01
    str     r0, [r5, #0x25c]

    b       Setup_Done

mddr_not_24:
    /* mDDR settings */
    cmp    r7, #1
    beq    just_set_tref_mddr

    ldr     r0, =0x00009c40
    str     r0, [r5, #0x8]

    ldr     r0, =0x02000000
    str     r0, [r5, #0x14]
    ldr     r0, =0x01010706
    str     r0, [r5, #0x18]
    ldr     r0, =0x080b0201
    str     r0, [r5, #0x1c]

    ldr     r0, =0x02000303
    str     r0, [r5, #0x20]
    ldr     r0, =0x0136b002
    str     r0, [r5, #0x24]
    ldr     r0, =0x01000101
    str     r0, [r5, #0x28]
    ldr     r0, =0x06030301
    str     r0, [r5, #0x2c]

    ldr     r0, =0x00000a02
    str     r0, [r5, #0x34]
    ldr     r0, =0x00000003
    str     r0, [r5, #0x38]
    ldr     r0, =0x00001401
    str     r0, [r5, #0x3c]

just_set_tref_mddr:
    ldr     r0, =133333333
    cmp   r4, r0
    bgt    mddr_200
    ldr     r0, =0x00050208
    b       tref_done1
mddr_200:
    ldr     r0, =0x0005030f
tref_done1:
    str     r0, [r5, #0x40]

    cmp    r7, #1
    beq    Setup_Done

    ldr     r0, =0x00180018
    str     r0, [r5, #0x48]

    ldr     r0, =0x00800000
    str     r0, [r5, #0x6c]

    ldr     r0, =0x02030302
    str     r0, [r5, #0xd4]

    ldr     r0, =0x06120000
    str     r0, [r5, #0x108]
    ldr     r0, =0x06120612
    str     r0, [r5, #0x10c]

    ldr     r0, =0x06120612
    str     r0, [r5, #0x110]
    ldr     r0, =0x01030612
    str     r0, [r5, #0x114]

    /* PHY setting for 200 MHz */
    ldr     r0, =0x00000000
    str     r0, [r5, #0x200]
    ldr     r0, =0x00000000
    str     r0, [r5, #0x204]
    ldr     r0, =0xf5002725
    str     r0, [r5, #0x208]

    ldr     r0, =0xf5002725
    str     r0, [r5, #0x210]
    ldr     r0, =0xf5002725
    str     r0, [r5, #0x218]

    ldr     r0, =0xf5002725
    str     r0, [r5, #0x220]
    ldr     r0, =0xf5002725
    str     r0, [r5, #0x228]

    ldr     r0, =0x070002d0
    str     r0, [r5, #0x20c]
    ldr     r0, =0x074002d0
    str     r0, [r5, #0x214]
    ldr     r0, =0x074002d0
    str     r0, [r5, #0x21c]
    ldr     r0, =0x074002d0
    str     r0, [r5, #0x224]
    ldr     r0, =0x074002d0
    str     r0, [r5, #0x22c]

    ldr     r0, =0x00800006
    str     r0, [r5, #0x234]
    ldr     r0, =0x200e1014
    str     r0, [r5, #0x238]
    ldr     r0, =0x000d9f01
    str     r0, [r5, #0x23c]

    ldr     r0, =0x200e1014
    str     r0, [r5, #0x240]
    ldr     r0, =0x000d9f01
    str     r0, [r5, #0x244]
    ldr     r0, =0x200e1014
    str     r0, [r5, #0x248]
    ldr     r0, =0x000d9f01
    str     r0, [r5, #0x24c]

    ldr     r0, =0x200e1014
    str     r0, [r5, #0x250]
    ldr     r0, =0x000d9f01
    str     r0, [r5, #0x254]
    ldr     r0, =0x200e1014
    str     r0, [r5, #0x258]
    ldr     r0, =0x000d9f01
    str     r0, [r5, #0x25c]

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
    ldmfd sp!, {r3,r4,r5,r6, r7}
    mov     pc, lr

    .type   mx50_do_ddr_freq_change, #object
ENTRY(mx50_do_ddr_freq_change)
    .word   mx50_ddr_freq_change
    .size    mx50_ddr_freq_change, . - mx50_ddr_freq_change
