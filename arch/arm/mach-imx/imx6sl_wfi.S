/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#define IRAM_WAIT_SIZE 	(1 << 11)

	.macro	sl_ddr_io_save

	ldr	r4, [r1, #0x30c] /* DRAM_DQM0 */
	ldr	r5, [r1, #0x310] /* DRAM_DQM1 */
	ldr	r6, [r1, #0x314] /* DRAM_DQM2 */
	ldr	r7, [r1, #0x318] /* DRAM_DQM3 */
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
	stmfd	r9!, {r4-r6}

	.endm

	.macro	sl_ddr_io_restore

	/*
	 * r9 points to IRAM stack.
	 * r1 points to IOMUX base address.
	 * r8 points to MMDC  base address.
	 */
	ldmea	r9!, {r4-r7}
	str	r4, [r1, #0x30c] /* DRAM_DQM0 */
	str	r5, [r1, #0x310] /* DRAM_DQM1 */
	str	r6, [r1, #0x314] /* DRAM_DQM2 */
	str	r7, [r1, #0x318] /* DRAM_DQM3 */

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

	ldmea	r9!, {r4-r6}
	str	r4, [r1, #0x330] /* DRAM_SDCKE0 */
	str	r5, [r1, #0x334] /* DRAM_SDCKE1 */
	str	r6, [r1, #0x320] /* DRAM_RESET */

	/*
	 * Need to reset the FIFO to avoid MMDC lockup
	 * caused because of floating/changing the
	 * configuration of many DDR IO pads.
	 */
	ldr	r7, =0x83c
	ldr	r6, [r8, r7]
	orr	r6, r6, #0x80000000
	str	r6, [r8, r7]
fifo_reset1_wait:
	ldr	r6, [r8, r7]
	and	r6, r6, #0x80000000
	cmp	r6, #0
	bne	fifo_reset1_wait

	/* reset FIFO a second time */
	ldr	r6, [r8, r7]
	orr	r6, r6, #0x80000000
	str	r6, [r8, r7]
fifo_reset2_wait:
	ldr	r6, [r8, r7]
	and	r6, r6, #0x80000000
	cmp	r6, #0
	bne	fifo_reset2_wait

	.endm

	.macro	sl_ddr_io_set_lpm

	mov	r4, #0
	str	r4, [r1, #0x30c] /* DRAM_DQM0 */
	str	r4, [r1, #0x310] /* DRAM_DQM1 */
	str	r4, [r1, #0x314] /* DRAM_DQM2 */
	str	r4, [r1, #0x318] /* DRAM_DQM3 */

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
 *  imx6sl_low_power_wfi
 *
 *  Idle the processor (eg, wait for interrupt).
 *  Make sure DDR is in self-refresh.
 *  IRQs are already disabled.
 * r0: WFI IRAMcode base address.
 * r1: IOMUX base address
 * r2: Base address of CCM, ANATOP and MMDC
 * r3: 1 if in audio_bus_freq_mode
 */
	.align 3
ENTRY(imx6sl_low_power_wfi)

	push {r4-r11}

mx6sl_lpm_wfi:
	/* Store audio_bus_freq_mode */
	mov	r11, r3

	mov	r4,r2
	/* Get the IRAM data storage address. */
	mov	r10, r0
	mov	r9, r0		/* get suspend_iram_base */
	add	r9, r9, #IRAM_WAIT_SIZE

	/* Anatop Base address in r3. */
	ldr	r3, [r4]
	/* CCM Base Address in r2 */
	ldr	r2, [r4, #0x4]
	/* MMDC Base Address in r8 */
	ldr	r8, [r4, #0x8]
	/* L2 Base Address in r7 */
	ldr	r7, [r4, #0xC]

	ldr	r6, [r8]
	ldr	r6, [r3]
	ldr	r6, [r2]
	ldr	r6, [r1]

	/* Store the original ARM PODF. */
	ldr	r0, [r2, #0x10]

	/* Drain all the L1 buffers. */
	dsb

#ifdef CONFIG_CACHE_L2X0
	/*
	 * Need to make sure the buffers in L2 are drained.
	 * Performing a sync operation does this.
	 */
	mov	r6, #0x0
	str	r6, [r7, #0x730]
#endif

	/*
	 * The second dsb might be needed to keep cache sync (device write)
	 * ordering with the memory accesses before it.
	 */
	dsb
	isb

	/* Save the DDR IO state. */
	sl_ddr_io_save

	/* Disable Automatic power savings. */
	ldr	r6, [r8, #0x404]
	orr	r6, r6, #0x01
	str	r6, [r8, #0x404]

	/* Make the DDR explicitly enter self-refresh. */
	ldr	r6, [r8, #0x404]
	orr	r6, r6, #0x200000
	str	r6, [r8, #0x404]

poll_dvfs_set_1:
	ldr	r6, [r8, #0x404]
	and	r6, r6, #0x2000000
	cmp	r6, #0x2000000
	bne	poll_dvfs_set_1

	/* set SBS step-by-step mode */
	ldr	r6, [r8, #0x410]
	orr	r6, r6, #0x100
	str	r6, [r8, #0x410]

	cmp	r11, #1
	beq	audio_mode
	/*
	 * Now set DDR rate to 1MHz.
	 * DDR is from bypassed PLL2 on periph2_clk2 path.
	 * Set the periph2_clk2_podf to divide by 8.
	 */
	ldr	r6, [r2, #0x14]
	orr	r6, r6, #0x07
	str	r6, [r2, #0x14]

	/* Now set MMDC PODF to divide by 3. */
	ldr	r6, [r2, #0x14]
	bic	r6, r6, #0x38
	orr	r6, r6, #0x10
	str	r6, [r2, #0x14]
	b	mmdc_podf

audio_mode:
	/* MMDC is from PLL2_200M.
	 * Set the mmdc_podf to div by 8.
	 */
	ldr	r6, [r2, #0x14]
	orr	r6, r6, #0x38
	str	r6, [r2, #0x14]

	/* Loop till podf is accepted. */
mmdc_podf:
	ldr	r6, [r2, #0x48]
	cmp	r6, #0x0
	bne	mmdc_podf

	/* Set the DDR IO in LPM state. */
	sl_ddr_io_set_lpm

	cmp	r11, #1
	beq	do_audio_arm_clk

	/*
	 * Check if none of the PLLs are
	 * locked, except PLL1 which will get
	 * bypassed below.
	 * We should not be here if PLL2 is not
	 * bypassed.
	 */
	ldr	r7, =1
	/* USB1 PLL3 */
	ldr	r6, [r3, #0x10]
	and	r6, r6, #0x80000000
	cmp	r6, #0x80000000
	beq	no_analog_saving

	/* USB2 PLL7 */
	ldr	r6, [r3, #0x20]
	and	r6, r6, #0x80000000
	cmp	r6, #0x80000000
	beq	no_analog_saving

	/* Audio PLL4 */
	ldr	r6, [r3, #0x70]
	and	r6, r6, #0x80000000
	cmp	r6, #0x80000000
	beq	no_analog_saving

	/* Video PLL5 */
	ldr	r6, [r3, #0xA0]
	and	r6, r6, #0x80000000
	cmp	r6, #0x80000000
	beq	no_analog_saving

	/* ENET PLL8 */
	ldr	r6, [r3, #0xE0]
	and	r6, r6, #0x80000000
	cmp	r6, #0x80000000
	beq	no_analog_saving

	b	cont

no_analog_saving:
	ldr	r7, =0

cont:
	/* Set the AHB to 3MHz. AXI to 3MHz. */
	ldr	r9, [r2, #0x14]
	mov	r6, r9
	orr	r6, r6, #0x1c00
	orr	r6, r6, #0x70000
	str	r6, [r2, #0x14]

	/* Loop till podf is accepted. */
ahb_podf:
	ldr	r6, [r2, #0x48]
	cmp	r6, #0x0
	bne	podf_loop

	/*
	 * Now set ARM to 24MHz.
	 * Move ARM to be sourced from STEP_CLK
	 * after setting STEP_CLK to 24MHz.
	 */
	ldr	r6, [r2, #0xc]
	bic	r6, r6, #0x100
	str	r6, [r2, #0x0c]
	/* Now PLL1_SW_CLK to step_clk. */
	ldr	r6, [r2, #0x0c]
	orr	r6, r6, #0x4
	str	r6, [r2, #0x0c]

	/* Bypass PLL1 and power it down. */
	ldr	r6, =(1 << 16)
	orr	r6, r6, #0x1000
	str	r6, [r3, #0x04]

	/*
	 * Set the ARM PODF to divide by 8.
	 * IPG is at 1.5MHz here, we need ARM to
	 * run at the 12:5 ratio (WAIT mode issue).
	 */
	ldr	r6, =0x7
	str	r6, [r2, #0x10]

	/* Loop till podf is accepted. */
podf_loop:
	ldr	r6, [r2, #0x48]
	cmp	r6, #0x0
	bne   podf_loop

	/*
	 * Check if we can save some
	 * power in the Analog section.
	 */
	cmp	r7, #0x1
	bne	do_wfi

	/* Disable 1p1 brown out. */
	ldr	r6, [r3, #0x110]
	bic	r6, r6, #0x2
	str	r6, [r3, #0x110]

	/* Enable the weak 2P5 */
	ldr	r6, [r3, #0x130]
	orr	r6, r6, #0x40000
	str	r6, [r3, #0x130]

	/* Disable main 2p5. */
	ldr	r6, [r3, #0x130]
	bic	r6, r6, #0x1
	str	r6, [r3, #0x130]

	/*
	 * Set the OSC bias current to -37.5%
	 * to drop the power on VDDHIGH.
	 */
	ldr	r6, [r3, #0x150]
	orr	r6, r6, #0xC000
	str	r6, [r3, #0x150]

	/* Enable low power bandgap */
	ldr	r6, [r3, #0x260]
	orr	r6, r6, #0x20
	str	r6, [r3, #0x260]

	/*
	 * Turn off the bias current
	 * from the regular bandgap.
	 */
	ldr	r6, [r3, #0x260]
	orr	r6, r6, #0x80
	str	r6, [r3, #0x260]

	/*
	 * Clear the REFTOP_SELFBIASOFF,
	 * self-bias circuit of the band gap.
	 * Per RM, should be cleared when
	 * band gap is powered down.
	 */
	ldr	r6, [r3, #0x150]
	bic	r6, r6, #0x8
	str	r6, [r3, #0x150]

	/* Power down the regular bandgap. */
	ldr	r6, [r3, #0x150]
	orr	r6, r6, #0x1
	str	r6, [r3, #0x150]

	b	do_wfi

do_audio_arm_clk:
	/* ARM is from PLL2_PFD2_400M here.
	 * Switch ARM to bypassed PLL1.
	 */
	ldr	r6, [r2, #0xC]
	bic	r6, r6, #0x4
	str	r6, [r2, #0xC]

	/* Set the ARM_PODF to divide by 2
	* as IPG is at 4MHz, we cannot run
	* ARM_CLK above 9.6MHz when
	* system enters WAIT mode.
	*/
	ldr	r6, =0x2
	str	r6, [r2, #0x10]

	/* Loop till podf is accepted. */
podf_loop_audio:
	ldr	r6, [r2, #0x48]
	cmp	r6, #0x0
	bne	podf_loop_audio

do_wfi:
	/* Now do WFI. */
	wfi

	/* Set original ARM PODF back. */
	str	r0, [r2, #0x10]

	/* Loop till podf is accepted. */
podf_loop1:
	ldr	r6, [r2, #0x48]
	cmp	r6, #0x0
	bne	podf_loop1

	cmp	r11, #1
	beq	audio_arm_clk_restore

	/*
	 * Check if powered down
	 * analog components.
	 */
	cmp	r7, #0x1
	bne	skip_analog_restore

	/* Power up the regular bandgap. */
	ldr	r6, [r3, #0x150]
	bic	r6, r6, #0x1
	str	r6, [r3, #0x150]

	/*
	 * Turn on the bias current
	 * from the regular bandgap.
	 */
	ldr	r6, [r3, #0x260]
	bic	r6, r6, #0x80
	str	r6, [r3, #0x260]

	/* Disable the low power bandgap */
	ldr	r6, [r3, #0x260]
	bic	r6, r6, #0x20
	str	r6, [r3, #0x260]

	/*
	 * Set the OSC bias current to max
	 * value for normal operation.
	 */
	ldr	r6, [r3, #0x150]
	bic	r6, r6, #0xC000
	str	r6, [r3, #0x150]

	/* Enable main 2p5. */
	ldr	r6, [r3, #0x130]
	orr	r6, r6, #0x1
	str	r6, [r3, #0x130]

	/* Ensure the 2P5 is up. */
loop_2p5:
	ldr	r6, [r3, #0x130]
	and	r6, r6, #0x20000
	cmp	r6, #0x20000
	bne	loop_2p5

	/* Disable the  weak 2P5 */
	ldr	r6, [r3, #0x130]
	bic	r6, r6, #0x40000
	str	r6, [r3, #0x130]

	/* Enable 1p1 brown out. */
	ldr	r6, [r3, #0x110]
	orr	r6, r6, #0x2
	str	r6, [r3, #0x110]

skip_analog_restore:

	/* Power up PLL1 and un-bypass it. */
	ldr	r6, =(1 << 12)
	str	r6, [r3, #0x08]

	/* Wait for PLL1 to relock. */
wait_for_pll_lock:
	ldr	r6, [r3, #0x0]
	and	r6, r6, #0x80000000
	cmp	r6, #0x80000000
	bne	wait_for_pll_lock

	ldr	r6, =(1 << 16)
	str	r6, [r3, #0x08]

	/* Set PLL1_sw_clk back to PLL1. */
	ldr	r6, [r2, #0x0c]
	bic	r6, r6, #0x4
	str	r6, [r2, #0xc]

	/* Restore AHB/AXI back. */
	str	r9, [r2, #0x14]

	/* Loop till podf is accepted. */
ahb_podf1:
	ldr	r6, [r2, #0x48]
	cmp	r6, #0x0
	bne	podf_loop1

	b	wfi_restore

	audio_arm_clk_restore:
	/* Move ARM back to PLL2_PFD2_400M */
	ldr	r6, [r2, #0xC]
	orr	r6, r6, #0x4
	str	r6, [r2, #0xC]

wfi_restore:
	/* get suspend_iram_base */
	mov	r9, r10
	add	r9, r9, #IRAM_WAIT_SIZE

	/* Restore the DDR IO before exiting self-refresh. */
	sl_ddr_io_restore

	/*
	 * Set MMDC back to 24MHz.
	 * Set periph2_clk2_podf to divide by 1
	 * Now set MMDC PODF to divide by 1.
	 */
	ldr	r6, [r2, #0x14]
	bic	r6, r6, #0x3f
	str	r6, [r2, #0x14]

mmdc_podf1:
	ldr	r6, [r2, #0x48]
	cmp	r6, #0x0
	bne	mmdc_podf1

	/* clear DVFS - exit from self refresh mode */
	ldr	r6, [r8, #0x404]
	bic	r6, r6, #0x200000
	str	r6, [r8, #0x404]

poll_dvfs_clear_1:
	ldr	r6, [r8, #0x404]
	and	r6, r6, #0x2000000
	cmp	r6, #0x2000000
	beq	poll_dvfs_clear_1

	/*
	 * Add these nops so that the
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
	ldr	r6, [r8, #0x404]
	bic	r6, r6, #0x01
	str	r6, [r8, #0x404]

	/* clear SBS - unblock DDR accesses */
	ldr	r6, [r8, #0x410]
	bic	r6, r6, #0x100
	str	r6, [r8, #0x410]


	pop {r4-r11}

	/* Restore registers */
	mov	pc, lr
