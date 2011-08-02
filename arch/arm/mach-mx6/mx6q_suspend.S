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
#include <mach/hardware.h>
#include <asm/memory.h>
#include "src-reg.h"

#define ARM_CTRL_DCACHE     1 << 2
#define ARM_CTRL_ICACHE     1 << 12
#define ARM_AUXCR_L2EN      1 << 1
#define TTRBIT_MASK 		0xffffc000
#define TABLE_INDEX_MASK 	0xfff00000
#define TABLE_ENTRY 		0x00000c02
#define CACHE_DISABLE_MASK 	0xffffe7fb

/*************************************************************
mx6q_suspend:

Suspend the processor (eg, wait for interrupt).
Set the DDR into Self Refresh
IRQs are already disabled.

The following code contain both standby and
dormant mode for MX6, decided by the parameter
passed in r0:
see define in include/linux/suspend.h
1 -> cpu enter stop mode;
3 -> cpu enter dormant mode.
*************************************************************/

ENTRY(mx6q_suspend)
	stmfd   sp!, {r0-r12}     @ Save registers
/*************************************************************
suspend mode entry
*************************************************************/

	cmp r0, #0x1
	bne dormant		/* dormant mode */

	dsb
	wfi

	nop
	nop
	nop
	nop

/***********************************************************
never run to here
************************************************************/
	b out	/* exit standby */

/************************************************************
dormant entry, data save in stack, save sp in the src_gpr2
************************************************************/
dormant:
	ldr r1, =SRC_BASE_ADDR
	add r1, r1, #PERIPBASE_VIRT
	str sp, [r1, #SRC_GPR2_OFFSET]	/* save sp in src_gpr2 */
/**********************************************************
saved register and context as below:
	sp
	spsr
	lr
	CPACR
	TTBR0
	TTBR1
	TTBCR
	DACR
	PRRR
	NMRR
	ACTLR
	Context ID
	User r/w thread ID
	Secure or NS VBAR
	CPSR
	SCTLR
************************************************************/
	/*
	 * Save only needed CPU CP15 registers. VFP, breakpoint,
	 * performance monitor registers are not saved. Generic
	 * code suppose to take care of those.
	 */
	mrs	r5, spsr			@ Store spsr
	mov	r6, lr				@ Store lr
	stmfd	sp!, {r5-r6}

	/* c1 and c2 registers */
	mrc	p15, 0, r4, c1, c0, 2		@ CPACR
	mrc	p15, 0, r5, c2, c0, 0		@ TTBR0
	mrc	p15, 0, r6, c2, c0, 1		@ TTBR1
	mrc	p15, 0, r7, c2, c0, 2		@ TTBCR
	stmfd	sp!, {r4-r7}

	/* c3 and c10 registers */
	mrc	p15, 0, r4, c3, c0, 0		@ DACR
	mrc	p15, 0, r5, c10, c2, 0		@ PRRR
	mrc	p15, 0, r6, c10, c2, 1		@ NMRR
	mrc	p15, 0, r7, c1, c0, 1		@ ACTLR
	stmfd	sp!,{r4-r7}

	/* c12, c13 and CPSR registers */
	mrc	p15, 0, r4, c13, c0, 1		@ Context ID
	mrc	p15, 0, r5, c13, c0, 2		@ User r/w thread ID
	mrc	p15, 0, r6, c12, c0, 0		@ Secure or NS VBAR
	mrs	r7, cpsr					@ Store CPSR
	stmfd	sp!, {r4-r7}

	/* c1 control register */
	mrc	p15, 0, r4, c1, c0, 0		@ SCTLR
	stmfd	sp!, {r4}

	/*
	 * Flush all data from the L1 data cache before disabling
	 * SCTLR.C bit.
	 */
	push {r0-r12}
	bl v7_flush_dcache_all
	pop {r0-r12}

	/*
	 * Clear the SCTLR.C bit to prevent further data cache
	 * allocation. Clearing SCTLR.C would make all the data accesses
	 * strongly ordered and would not hit the cache.
	 */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #(1 << 2)		@ Disable the C bit
	mcr	p15, 0, r0, c1, c0, 0
	isb

	/*
	 * Invalidate L1 data cache. Even though only invalidate is
	 * necessary exported flush API is used here. Doing clean
	 * on already clean cache would be almost NOP.
	 */
	push {r0-r12}
	bl v7_flush_dcache_all
	pop {r0-r12}

	/*
	 * Execute an ISB instruction to ensure that all of the
	 * CP15 register changes have been committed.
	 */
	isb

	/*
	 * Execute a barrier instruction to ensure that all cache,
	 * TLB and branch predictor maintenance operations issued
	 * by any CPU in the cluster have completed.
	 */
	dsb
	dmb


/****************************************************************
save resume pointer into SRC_GPR1
****************************************************************/
	ldr r0, =resume
	ldr r1, =va2pa_offset
	sub r0, r0, r1
	ldr r1, =SRC_BASE_ADDR
	add r1, r1, #PERIPBASE_VIRT
	str r0, [r1, #SRC_GPR1_OFFSET]
/****************************************************************
execute a wfi instruction to let SOC go into stop mode.
****************************************************************/
	wfi

	nop
	nop
	nop
	nop

/****************************************************************
never go here.
****************************************************************/







/****************************************************************
when SOC exit stop mode, arm core restart from here, currently
are running with MMU off.
****************************************************************/
resume:
	mov r1, #0
	ldr r0, =SRC_BASE_ADDR
	str r1, [r0, #SRC_GPR1_OFFSET] /* clear SRC_GPR1 */
	ldr sp, [r0, #SRC_GPR2_OFFSET]
	ldr r1, =va2pa_offset
	sub sp, sp, r1
	/* Restore cp15 registers */
	ldmea	sp!, {r5-r6}
	msr	spsr_cxsf, r5		@ Restore spsr
	mov	lr, r6				@ Restore lr

	/* c1 and c2 registers */
	ldmea	sp!, {r4-r7}
	mcr	p15, 0, r4, c1, c0, 2		@ CPACR
	mcr	p15, 0, r5, c2, c0, 0		@ TTBR0
	mcr	p15, 0, r6, c2, c0, 1		@ TTBR1
	mcr	p15, 0, r7, c2, c0, 2		@ TTBCR

	/* c3 and c10 registers */
	ldmea	sp!,{r4-r7}
	mcr	p15, 0, r4, c3, c0, 0		@ DACR
	mcr	p15, 0, r5, c10, c2, 0		@ PRRR
	mcr	p15, 0, r6, c10, c2, 1		@ NMRR
	mcr	p15, 0, r7, c1, c0, 1		@ ACTLR

	/* c12, c13 and CPSR registers */
	ldmea	sp!,{r4-r7}
	mcr	p15, 0, r4, c13, c0, 1		@ Context ID
	mcr	p15, 0, r5, c13, c0, 2		@ User r/w thread ID
	mrc	p15, 0, r6, c12, c0, 0		@ Secure or NS VBAR
	msr	cpsr, r7			@ store cpsr

	/*
	 * Enabling MMU here. Page entry needs to be altered
	 * to create temporary 1:1 map and then resore the entry
	 * ones MMU is enabled
	 */
	mrc	p15, 0, r7, c2, c0, 2		@ Read TTBRControl
	and	r7, #0x7			@ Extract N (0:2) to decide
	cmp	r7, #0x0			@ TTBR0/TTBR1
	beq	use_ttbr0
ttbr_error:
	b	ttbr_error			@ Only N = 0 supported
use_ttbr0:
	mrc	p15, 0, r2, c2, c0, 0		@ Read TTBR0
	ldr	r5, =TTRBIT_MASK
	and	r2, r5
	mov	r4, pc
	ldr	r5, =TABLE_INDEX_MASK
	and	r4, r5				@ r4 = 31 to 20 bits of pc
	ldr	r1, =TABLE_ENTRY
	add	r1, r1, r4			@ r1 has value of table entry
	lsr	r4, #18				@ Address of table entry
	add	r2, r4				@ r2 - location to be modified

	/* Storing previous entry of location being modified */
	ldr	r4, [r2]
	mov r9, r4
	str	r1, [r2]

	/*
	 * Storing address of entry being modified
	 * It will be restored after enabling MMU
	 */
	mov r10, r2

	mov	r1, #0
	mcr	p15, 0, r1, c7, c5, 4		@ Flush prefetch buffer
	mcr	p15, 0, r1, c7, c5, 6		@ Invalidate BTB
	mcr	p15, 0, r1, c8, c5, 0		@ Invalidate ITLB
	mcr	p15, 0, r1, c8, c6, 0		@ Invalidate DTLB

	/*
	 * Restore control register  but don't enable Data caches here.
	 * Caches will be enabled after restoring MMU table entry.
	 */
	ldmea	sp!, {r4}
	mov r11, r4
	ldr	r2, =CACHE_DISABLE_MASK
	and	r4, r4, r2
	mcr	p15, 0, r4, c1, c0, 0
	isb
	dsb
	ldr	r1, =mmu_on_label
	bx	r1
mmu_on_label:
	/* Set up the per-CPU stacks */
	ldr r1, =va2pa_offset
	add sp, sp, r1
	mov r5, lr
	bl	cpu_init

	/*
	 * Restore the MMU table entry that was modified for
	 * enabling MMU.
	 */
	mov r0, r9
	mov r10, r0

	mov	r0, #0
	mcr	p15, 0, r0, c7, c1, 6		@ flush TLB and issue barriers
	mcr	p15, 0, r0, c7, c5, 4		@ Flush prefetch buffer
	mcr	p15, 0, r0, c7, c5, 6		@ Invalidate BTB
	mcr	p15, 0, r0, c8, c5, 0		@ Invalidate ITLB
	mcr	p15, 0, r0, c8, c6, 0		@ Invalidate DTLB
	dsb
	isb

/******************************************************************
invalidate l1 dcache, r0-r4, r6, r7 used
******************************************************************/
	mov     r0, #0
	mcr     p15, 2, r0, c0, c0, 0
	mrc     p15, 1, r0, c0, c0, 0

	ldr     r1, =0x7fff
	and     r2, r1, r0, lsr #13

	ldr     r1, =0x3ff

	and     r3, r1, r0, lsr #3  @ NumWays - 1
	add     r2, r2, #1          @ NumSets

	and     r0, r0, #0x7
	add     r0, r0, #4          @ SetShift

	clz     r1, r3              @ WayShift
	add     r4, r3, #1          @ NumWays
1:
	sub     r2, r2, #1          @ NumSets--
	mov     r3, r4              @ Temp = NumWays
2:
	subs    r3, r3, #1          @ Temp--
	mov     r7, r3, lsl r1
	mov     r6, r2, lsl r0
	orr     r7, r7, r6          @ Reg = (Temp<<WayShift)|(NumSets<<SetShift)
	mcr     p15, 0, r7, c7, c6, 2
	bgt     2b
	cmp     r2, #0
	bgt     1b
	dsb
	isb

/************************************************************
restore control register to enable cache
************************************************************/
	mov r0, r11
	mcr     p15, 0, r0, c1, c0, 0	@ with caches enabled.
	isb

/************************************************************
clear src register we used
************************************************************/
	ldr r8, =SRC_BASE_ADDR
	add r8, r8, #PERIPBASE_VIRT
	ldr sp, [r8, #SRC_GPR2_OFFSET]

/***********************************************************
return back to mx6_suspend_enter for dormant
***********************************************************/
	mov lr, r5
	ldmfd   sp!, {r0-r12}
	mov pc, lr


/************************************************
return back to mx6_suspend_enter for suspend
*************************************************/
out:
	ldmfd   sp!, {r0-r12}
	mov pc, lr

	.equ	va2pa_offset, (PAGE_OFFSET - MX6_PHYS_OFFSET)
	.type   mx6q_do_suspend, #object
ENTRY(mx6q_do_suspend)
	.word   mx6q_suspend
	.size   mx6q_suspend, . - mx6q_suspend
