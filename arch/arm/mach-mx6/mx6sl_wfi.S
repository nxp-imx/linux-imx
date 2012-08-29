/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#define IRAM_WAIT_SIZE 	(1 << 11)


	.macro	sl_ddr_io_save

	ldr	r4, [r1, #0x30c] /* DRAM_DQM0 */
	ldr	r5, [r1, #0x310] /* DRAM_DQM1 */
	ldr	r6, [r1, #0x314] /* DRAM_DQM2 */
	ldr	r7, [r1, #0x318] /* DRAM_DQM3 */
	stmfd	r9!, {r4-r7}

	ldr	r4, [r1, #0x344] /* DRAM_SDQS0 */
	ldr	r5, [r1, #0x348] /* DRAM_SDQS1 */
	ldr	r6, [r1, #0x34c] /* DRAM_SDQS2 */
	ldr	r7, [r1, #0x350] /* DRAM_SDQS3 */
	stmfd	r9!, {r4-r7}

	ldr	r4, [r1, #0x5c4] /* GPR_B0DS */
	ldr	r5, [r1, #0x5cc] /* GPR_B1DS */
	ldr	r6, [r1, #0x5d4] /* GPR_B2DS */
	ldr	r7, [r1, #0x5d8] /* GPR_B3DS */
	stmfd	r9!, {r4-r7}

	ldr	r4, [r1, #0x300] /* DRAM_CAS */
	ldr	r5, [r1, #0x31c] /* DRAM_RAS */
	ldr	r6, [r1, #0x338] /* DRAM_SDCLK_0 */
	ldr r7, [r1, #0x5ac] /* GPR_ADDS*/
	stmfd	r9!, {r4-r7}

	ldr	r4, [r1, #0x5b0] /* DDRMODE_CTL */
	ldr	r5, [r1, #0x5c0] /* DDRMODE */
	ldr	r6, [r1, #0x33c] /* DRAM_SODT0*/
	ldr	r7, [r1, #0x340] /* DRAM_SODT1*/
	stmfd	r9!, {r4-r7}

	ldr	r4, [r1, #0x330] /* DRAM_SDCKE0 */
	ldr	r5, [r1, #0x334] /* DRAM_SDCKE1 */
	ldr	r6, [r1, #0x320] /* DRAM_RESET */
	ldr	r7, [r1, #0x5c8] /* GPR_CTLDS */
	stmfd	r9!, {r4-r7}

	.endm

	.macro	sl_ddr_io_restore

	ldmea	r9!, {r4-r7}
	str	r4, [r1, #0x30c] /* DRAM_DQM0 */
	str	r5, [r1, #0x310] /* DRAM_DQM1 */
	str	r6, [r1, #0x314] /* DRAM_DQM2 */
	str	r7, [r1, #0x318] /* DRAM_DQM3 */

	ldmea	r9!, {r4-r7}
	str	r4, [r1, #0x344] /* DRAM_SDQS0 */
	str	r5, [r1, #0x348] /* DRAM_SDQS1 */
	str	r6, [r1, #0x34c] /* DRAM_SDQS2 */
	str	r7, [r1, #0x350] /* DRAM_SDQS3 */

	ldmea	r9!, {r4-r7}
	str	r4, [r1, #0x5c4] /* GPR_B0DS */
	str	r5, [r1, #0x5cc] /* GPR_B1DS */
	str r6, [r1, #0x5d4] /* GPR_B2DS */
	str r7, [r1, #0x5d8] /* GPR_B3DS */

	ldmea	r9!, {r4-r7}
	str	r4, [r1, #0x300] /* DRAM_CAS */
	str	r5, [r1, #0x31c] /* DRAM_RAS */
	str	r6, [r1, #0x338] /* DRAM_SDCLK_0 */
	str r7, [r1, #0x5ac] /* GPR_ADDS*/

	ldmea	r9!, {r4-r7}
	str	r4, [r1, #0x5b0] /* DDRMODE_CTL */
	str	r5, [r1, #0x5c0] /* DDRMODE */
	str	r6, [r1, #0x33c] /* DRAM_SODT0*/
	str	r7, [r1, #0x340] /* DRAM_SODT1*/

	ldmea	r9!, {r4-r7}
	str	r4, [r1, #0x330] /* DRAM_SDCKE0 */
	str	r5, [r1, #0x334] /* DRAM_SDCKE1 */
	str	r6, [r1, #0x320] /* DRAM_RESET */
	str	r7, [r1, #0x5c8] /* GPR_CTLDS */

	.endm

	.macro	sl_ddr_io_set_lpm

	mov	r4, #0
	str	r4, [r1, #0x30c] /* DRAM_DQM0 */
	str	r4, [r1, #0x310] /* DRAM_DQM1 */
	str	r4, [r1, #0x314] /* DRAM_DQM2 */
	str	r4, [r1, #0x318] /* DRAM_DQM3 */

	/* Make sure the Pull Ups are enabled.
	 * So only reduce the drive stength, but
	 * leave the pull-ups in the original state.
	 * This is required for LPDDR2.
	 */
	ldr	r4, [r1, #0x344]
	orr	r4, r4, #0x3000
	str	r4, [r1, #0x344] /* DRAM_SDQS0 */
	str	r4, [r1, #0x348] /* DRAM_SDQS1 */
	str	r4, [r1, #0x34c] /* DRAM_SDQS2 */
	str	r4, [r1, #0x350] /* DRAM_SDQS3 */

	str	r4, [r1, #0x5c4] /* GPR_B0DS */
	str	r4, [r1, #0x5cc] /* GPR_B1DS */
	str r4, [r1, #0x5d4] /* GPR_B2DS */
	str r4, [r1, #0x5d8] /* GPR_B3DS */

	str	r4, [r1, #0x300] /* DRAM_CAS */
	str	r4, [r1, #0x31c] /* DRAM_RAS */
	str	r4, [r1, #0x338] /* DRAM_SDCLK_0 */
	str r4, [r1, #0x5ac] /* GPR_ADDS*/

	str	r4, [r1, #0x5b0] /* DDRMODE_CTL */
	str	r4, [r1, #0x5c0] /* DDRMODE */
	str	r4, [r1, #0x33c] /* DRAM_SODT0*/
	str	r4, [r1, #0x340] /* DRAM_SODT1*/

	mov	r4, #0x80000
	str	r4, [r1, #0x320] /* DRAM_RESET */
	mov	r4, #0x1000
	str	r4, [r1, #0x330] /* DRAM_SDCKE0 */
	str	r4, [r1, #0x334] /* DRAM_SDCKE1 */

	.endm

/*
 *  mx6sl_wait
 *
 *  Idle the processor (eg, wait for interrupt).
 *  Make sure DDR is in self-refresh.
 *  IRQs are already disabled.
 * r0 : arm_podf before WFI is entered
 * r1: WFI IRAMcode base address.
 */
ENTRY(mx6sl_wait)

    push {r4, r5, r6, r7, r8, r9, r10}

mx6sl_lpm_wfi:
    /* Get the IRAM data storage address. */
    mov    r10, r1
    mov	r9, r1 		/* get suspend_iram_base */
     add	r9, r9, #IRAM_WAIT_SIZE	/* 4K */

    ldr     r1, =MX6Q_IOMUXC_BASE_ADDR
    add   r1, r1, #PERIPBASE_VIRT

      /* Save the DDR IO state. */
     sl_ddr_io_save

    ldr     r3, =ANATOP_BASE_ADDR
    add   r3, r3, #PERIPBASE_VIRT

    ldr     r2, =CCM_BASE_ADDR
    add   r2, r2, #PERIPBASE_VIRT

    ldr     r8, =MMDC_P0_BASE_ADDR
    add   r8, r8, #PERIPBASE_VIRT


    /* Prime all TLB entries. */
    adr   r7, mx6sl_lpm_wfi               @Address in this function.

    ldr     r6, [r7]

    ldr     r6, [r8]
    ldr     r6, [r3]
    ldr     r6, [r2]
    ldr     r6, [r1]

     dsb

     /* Disable Automatic power savings. */
     ldr    r6, [r8, #0x404]
     orr    r6, r6, #0x01
     str    r6, [r8, #0x404]

     /* Make the DDR explicitly enter self-refresh. */
     ldr     r6, [r8, #0x404]
     orr    r6, r6, #0x200000
     str     r6, [r8, #0x404]

poll_dvfs_set_1:
     ldr     r6, [r8, #0x404]
     and     r6, r6, #0x2000000
     cmp    r6, #0x2000000
     bne     poll_dvfs_set_1

     /* set SBS step-by-step mode */
     ldr     r6, [r8, #0x410]
     orr     r6, r6, #0x100
     str     r6, [r8, #0x410]

     /* Now set DDR rate to 1MHz. */
     /* DDR is from bypassed PLL2 on periph2_clk2 path.
       * Set the periph2_clk2_podf to divide by 8.
       */
     ldr    r6, [r2, #0x14]
     orr   r6, r6, #0x07
     str    r6, [r2, #0x14]

     /* Now set MMDC PODF to divide by 3. */
     ldr    r6, [r2, #0x14]
     bic    r6, r6, #0x38
     orr    r6, r6, #0x10
     str     r6, [r2, #0x14]

    /* Loop till podf is accepted. */
mmdc_podf:
     ldr     r6, [r2, #0x48]
     cmp   r6, #0x0
     bne   mmdc_podf

      /* Set the DDR IO in LPM state. */
     sl_ddr_io_set_lpm

     /* Set AHB to 8MHz., AXI to 3MHz */
     /* We want to ensure IPG_PERCLK to AHB
      * clk ratio is 1:2.5
      */
     /* Store the AXI/AHB podfs. */
     ldr    r9, [r2, #0x14]
     mov   r6, r9
     bic      r6, r6, #0x1c00
     orr    r6, r6, #0x800
     orr    r6, r6, #0x70000
     str     r6, [r2, #0x14]

    /* Loop till podf is accepted. */
ahb_podf:
     ldr     r6, [r2, #0x48]
     cmp   r6, #0x0
     bne   podf_loop

    /* Now set ARM to 24MHz. */
    /* Move ARM to be sourced from STEP_CLK
     * after setting STEP_CLK to 24MHz.
     */
     ldr    r6, [r2, #0xc]
     bic   r6, r6, #0x100
     str    r6, [r2, #0x0c]
     /* Now PLL1_SW_CLK to step_clk. */
     ldr    r6, [r2, #0x0c]
     orr    r6, r6, #0x4
     str    r6, [r2, #0x0c]

    /* Bypass PLL1 and power it down. */
    ldr     r6, =(1 << 16)
    orr    r6, r6, #0x1000
    str     r6, [r3, #0x04]

    /* Set the ARM PODF to divide by 8. */
    /* IPG is at 4MHz here, we need ARM to
     * run at the 12:5 ratio (WAIT mode issue).
     */
    ldr    r6, =0x7
    str    r6, [r2, #0x10]

    /* Loop till podf is accepted. */
podf_loop:
     ldr     r6, [r2, #0x48]
     cmp   r6, #0x0
     bne   podf_loop

     /* Now do WFI. */
     dsb

     wfi

     /* Set original ARM PODF back. */
    str    r0, [r2, #0x10]

    /* Loop till podf is accepted. */
podf_loop1:
     ldr     r6, [r2, #0x48]
     cmp   r6, #0x0
     bne   podf_loop1

     /* Power up PLL1 and un-bypass it. */
    ldr     r6, =(1 << 12)
    str     r6, [r3, #0x08]

    /* Wait for PLL1 to relock. */
wait_for_pll_lock:
    ldr    r6, [r3, #0x0]
    and   r6, r6, #0x80000000
    cmp  r6, #0x80000000
    bne   wait_for_pll_lock

    ldr     r6, =(1 << 16)
    str     r6, [r3, #0x08]

    /* Set PLL1_sw_clk back to PLL1. */
    ldr    r6, [r2, #0x0c]
    bic    r6, r6, #0x4
    str    r6, [r2, #0xc]

    /* Restore AHB/AXI back. */
    str      r9, [r2, #0x14]

    /* Loop till podf is accepted. */
ahb_podf1:
     ldr     r6, [r2, #0x48]
     cmp   r6, #0x0
     bne   podf_loop1

    mov	r9, r10 		/* get suspend_iram_base */
     add	r9, r9, #IRAM_WAIT_SIZE	/* 4K */

    /* Restore the DDR IO before exiting self-refresh. */
     sl_ddr_io_restore

    /* Set MMDC back to 24MHz. */
    /* Set periph2_clk2_podf to divide by 1. */
     /* Now set MMDC PODF to divide by 1. */
     ldr    r6, [r2, #0x14]
     bic   r6, r6, #0x3f
     str    r6, [r2, #0x14]

mmdc_podf1:
     ldr     r6, [r2, #0x48]
     cmp   r6, #0x0
     bne   mmdc_podf1

     /* clear DVFS - exit from self refresh mode */
     ldr     r6, [r8, #0x404]
     bic     r6, r6, #0x200000
     str     r6, [r8, #0x404]

poll_dvfs_clear_1:
     ldr     r6, [r8, #0x404]
     and     r6, r6, #0x2000000
     cmp     r6, #0x2000000
     beq     poll_dvfs_clear_1

     /* Add these nops so that the
      * prefetcher will not try to get
      * any instructions from DDR.
      * The prefetch depth is about 23
      * on A9, so adding 25 nops.
      */
     nop
     nop
     nop
     nop
     nop

     nop
     nop
     nop
     nop
     nop

     nop
     nop
     nop
     nop
     nop

     nop
     nop
     nop
     nop
     nop

     nop
     nop
     nop
     nop
     nop

     /* Enable Automatic power savings. */
     ldr    r6, [r8, #0x404]
     bic    r6, r6, #0x01
     str    r6, [r8, #0x404]

     /* clear SBS - unblock DDR accesses */
     ldr     r6, [r8, #0x410]
     bic     r6, r6, #0x100
     str     r6, [r8, #0x410]


    pop {r4,r5, r6, r7, r8, r9, r10}

    /* Restore registers */
    mov     pc, lr

    .type   mx6sl_do_wait, #object
ENTRY(mx6sl_do_wait)
    .word   mx6sl_wait
    .size    mx6sl_wait, . - mx6sl_wait
