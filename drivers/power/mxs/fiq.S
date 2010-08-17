/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>
#include <asm/pgtable-hwdef.h>
#include <mach/regs-power.h>
#include <mach/../../regs-clkctrl.h>
#include <mach/regs-timrot.h>

	.align 5
	.globl power_fiq_start
	.globl power_fiq_end
	.globl power_fiq_count
	.globl lock_vector_tlb

power_fiq_start:
	ldr 	r8,power_reg
	ldr	r9,[r8,#HW_POWER_CTRL ]
	ldr	r10,power_off

	@ when VDDIO_BO_IRQ,
	@ disabled, handled in IRQ for now
	@tst 	r9, #BM_POWER_CTRL_VDDIO_BO_IRQ


	@ when BATT_BO_IRQ,  VDDD_BO_IRQ, VDDA_BO_IRQ, power off chip
	ldr     r11,power_bo
	tst	r9, r11
	strne 	r10,[r8,#HW_POWER_RESET]

	@VDD5V_DROOP_IRQ
	tst     r9, #BM_POWER_CTRL_VDD5V_DROOP_IRQ
	beq	    check_dcdc4p2

	@ handle errata
	ldr	r10, [r8, #HW_POWER_DCDC4P2]
	orr	r10,r10,#(BM_POWER_DCDC4P2_TRG)
	orr 	r10,r10,#(BF_POWER_DCDC4P2_CMPTRIP(31))
	str	r10,[r8, #(HW_POWER_DCDC4P2)]

	@ if battery is below brownout level, shutdown asap
	ldr	r10,  [r8, #HW_POWER_STS]
	tst	r10, #BM_POWER_STS_BATT_BO
	ldr	r10, power_off
	strne	r10, [r8, #HW_POWER_RESET]

	@ disable viddio irq
	mov	r11, #BM_POWER_CTRL_ENIRQ_VDDIO_BO
	str 	r11, [r8, #HW_POWER_CTRL_CLR]

	@ enable battery BO irq
	mov	r11, #BM_POWER_CTRL_BATT_BO_IRQ
	str	r11, [r8, #HW_POWER_CTRL_CLR]
	mov	r11, #BM_POWER_CTRL_ENIRQBATT_BO
	str	r11, [r8, #HW_POWER_CTRL_SET]

	@ disable dcdc4p2 interrupt
	mov	r11, #BM_POWER_CTRL_ENIRQ_DCDC4P2_BO
	str	r11, [r8, #HW_POWER_CTRL_CLR]

	@ disable vdd5v_droop interrupt
	mov	r11, #BM_POWER_CTRL_ENIRQ_VDD5V_DROOP
	str	r11, [r8, #HW_POWER_CTRL_CLR]

check_dcdc4p2:
		@ when DCDC4P2_BO_IRQ,
	tst 	r9, #BM_POWER_CTRL_DCDC4P2_BO_IRQ

	mov	r11, #BM_POWER_CTRL_BATT_BO_IRQ
	strne	r11, [r8, #HW_POWER_CTRL_CLR]

	mov	r11, #BM_POWER_CTRL_ENIRQBATT_BO
	strne   r11, [r8, #HW_POWER_CTRL_SET]

	mov	r11, #BM_POWER_CTRL_ENIRQ_DCDC4P2_BO
	strne	r11, [r8, #HW_POWER_CTRL_CLR]



	@return from fiq
	subs    pc,lr, #4

power_reg:
	.long IO_ADDRESS(POWER_PHYS_ADDR)
power_off:
	.long 0x3e770001
power_bo:
	.long BM_POWER_CTRL_BATT_BO_IRQ | \
	      BM_POWER_CTRL_VDDA_BO_IRQ | BM_POWER_CTRL_VDDD_BO_IRQ
power_fiq_count:
	.long 0
power_fiq_end:

lock_vector_tlb:

	mov r1, r0 @ set r1 to the value of the address to be locked down
	mcr p15,0,r1,c8,c7,1    @ invalidate TLB single entry to ensure that
				@ LockAddr is not already in the TLB
	mrc p15,0,r0,c10,c0,0   @ read the lockdown register
	orr r0,r0,#1            @ set the preserve bit
	mcr p15,0,r0,c10,c0,0   @ write to the lockdown register
	ldr r1,[r1]             @ TLB will miss, and entry will be loaded
	mrc p15,0,r0,c10,c0,0   @ read the lockdown register (victim will have
				@ incremented)
	bic r0,r0,#1            @ clear preserve bit
	mcr p15,0,r0,c10,c0,0   @ write to the lockdown registerADR r1,LockAddr
	mov pc,lr		@
