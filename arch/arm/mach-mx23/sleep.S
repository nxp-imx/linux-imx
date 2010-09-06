/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
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
#include <asm/system.h>
#include <asm/pgtable-hwdef.h>
#include <mach/hardware.h>
#include <mach/regs-power.h>
#include "regs-clkctrl.h"
#include "sleep.h"

#define HW_CLKCTRL_CPU_ADDR \
	(MX23_SOC_IO_ADDRESS(CLKCTRL_PHYS_ADDR) + HW_CLKCTRL_CPU)
#define HW_POWER_MINPWR_ADDR \
	(MX23_SOC_IO_ADDRESS(POWER_PHYS_ADDR) + HW_POWER_MINPWR)
#define HW_POWER_RESET_ADDR \
	(MX23_SOC_IO_ADDRESS(POWER_PHYS_ADDR) + HW_POWER_RESET)

#define HW_DRAM_CTL06	MX23_SOC_IO_ADDRESS(0x800E0018)
#define HW_DRAM_CTL08	MX23_SOC_IO_ADDRESS(0x800E0020)
#define HW_EMI_STAT	MX23_SOC_IO_ADDRESS(0x80020010)
#define HW_RTC_PERSISTENT0 \
			MX23_SOC_IO_ADDRESS(0x8005C060)

#define PHYS_RAM_START		0x40000000

.global cpu_arm926_switch_mm

		.text

.align 8
ENTRY(mx23_cpu_standby)
	@ save registers on stack
	stmfd	sp!, {r0 - r9, lr}

	adr	r9, __mx23_temp_stack

	@ clean cache
	ldr	r1, __mx23_flush_cache_addr
	mov	lr, pc
	mov	pc, r1

	@ put DRAM into self refresh
	mov	r0, #(HW_DRAM_CTL08 & 0x000000FF)
	orr	r0, r0, #(HW_DRAM_CTL08 & 0x0000FF00)
	orr	r0, r0, #(HW_DRAM_CTL08 & 0x00FF0000)
	orr	r0, r0, #(HW_DRAM_CTL08 & 0xFF000000)
	ldr	r1, [r0]
	orr	r1, r1, #(1 << 8)
	str	r1, [r0]
	@ wait for it to actually happen
	mov	r0, #(HW_EMI_STAT & 0x000000FF)
	orr	r0, r0, #(HW_EMI_STAT & 0x0000FF00)
	orr	r0, r0, #(HW_EMI_STAT & 0x00FF0000)
	orr	r0, r0, #(HW_EMI_STAT & 0xFF000000)
1:	ldr	r1, [r0]
	teq	r1, #(1 << 1)
	beq	1b
	nop
	nop
	nop
	nop
	nop

#ifdef CONFIG_STMP378X_RAM_FREQ_SCALING
	@ RAM to clk from xtal
	mov	lr, pc
	b	mx23_ram_save_timings
	mov	lr, pc
	b	mx23_ram_24M_set_timings

	mov	r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0xFF000000)
	ldr	r4, [r0]
	mov	r1, #(1<<6)
	str	r1, [r0, #4]
1:	ldr	r1, [r0]
	tst	r1, #BM_CLKCTRL_EMI_BUSY_REF_XTAL
	bne	1b

	@ save RAM divisors
	mov	r0, #(HW_CLKCTRL_FRAC_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_ADDR & 0xFF000000)
	ldr	r8, [r0]
	and	r8, r8, #(0x3F << 8)
	lsr	r8, r8, #8
	mov	r0, #(HW_CLKCTRL_EMI_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0xFF000000)
	ldr	r7, [r0]
	and	r7, r7, #0x3F

	@ shut the PLL down
	mov	r0, #(HW_CLKCTRL_PLLCTRL0_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_PLLCTRL0_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_PLLCTRL0_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_PLLCTRL0_ADDR & 0xFF000000)
	mov	r1, #(1<<16)
	str	r1, [r0, #0x08]	@ clear

	@ set vddd to minimum
	mov	r0, #(HW_POWER_VDDDCTRL_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_POWER_VDDDCTRL_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_POWER_VDDDCTRL_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_POWER_VDDDCTRL_ADDR & 0xFF000000)
	ldr	r6, [r0]
	bic	r1, r6, #0xFF
	bic	r1, r1, #0x30
	orr	r1, r1, #0xa
	str	r1, [r0]
	/* now wait 1000 us = 24000 cycles */
	mov	r0, #24 << 10
3:	sub	r0, r0, #1
	cmp	r0, #0
	bne	3b
	nop
#endif

	@ do enter standby
	mov	r0, #(HW_CLKCTRL_CPU_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_CPU_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_CPU_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_CPU_ADDR & 0xFF000000)
	mov	r1, #(1<<12)
	str	r1, [r0, #4]
	mov	r2, #0
	mcr	p15, 0, r2, c7, c0, 4
	nop

	@ sleeping now...

	@ remove INTERRUPT_WAIT bit
	str	r1, [r0, #8]
	nop
	nop
	nop

#ifdef CONFIG_STMP378X_RAM_FREQ_SCALING
	@ restore vddd
	mov	r0, #(HW_POWER_VDDDCTRL_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_POWER_VDDDCTRL_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_POWER_VDDDCTRL_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_POWER_VDDDCTRL_ADDR & 0xFF000000)
	ldr	r1, [r0]
	str	r6, [r0]
	/* now wait 1000 us = 24000 cycles */
	mov	r0, #24 << 10
12:	sub	r0, r0, #1
	cmp	r0, #0
	bne	12b
	nop

	@ put the PLL back up
	mov	r0, #(HW_CLKCTRL_PLLCTRL0_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_PLLCTRL0_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_PLLCTRL0_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_PLLCTRL0_ADDR & 0xFF000000)
	mov	r1, #(1<<16)
	str	r1, [r0, #0x04]	@ set
	/* now wait 10 us = 240 cycles */
	mov	r0, #240
11:	sub	r0, r0, #1
	cmp	r0, #0
	bne	11b
	nop

	@ set divisors and switch EMI back to PLL
	mov	lr, pc
	b	mx23_ram_restore_timings
	mov	lr, pc
	b	__mx23_emi_set_values

	mov	r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0xFF000000)
	mov	r1, #(1<<6)
	str	r1, [r0, #8]

	mov	r0, #(HW_CLKCTRL_EMI_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0xFF000000)
	ldr	r1, [r0]
	bic	r1, #BM_CLKCTRL_EMI_DCC_RESYNC_ENABLE
	str	r1, [r0]
#endif

	@ restore normal DRAM mode
	mov	r0, #(HW_DRAM_CTL08 & 0x000000FF)
	orr	r0, r0, #(HW_DRAM_CTL08 & 0x0000FF00)
	orr	r0, r0, #(HW_DRAM_CTL08 & 0x00FF0000)
	orr	r0, r0, #(HW_DRAM_CTL08 & 0xFF000000)
	ldr	r1, [r0]
	bic	r1, r1, #(1 << 8)
	str	r1, [r0]
	@ wait for it to actually happen
	mov	r0, #(HW_EMI_STAT & 0x000000FF)
	orr	r0, r0, #(HW_EMI_STAT & 0x0000FF00)
	orr	r0, r0, #(HW_EMI_STAT & 0x00FF0000)
	orr	r0, r0, #(HW_EMI_STAT & 0xFF000000)
102:	ldr	r1, [r0]
	tst	r1, #(1 << 1)
	bne	102b

	nop
	nop
	nop

	@ restore regs and return
	ldmfd   sp!, {r0 - r9, pc}

	.space	0x100
__mx23_temp_stack:
	.word	0

#ifdef CONFIG_STMP378X_RAM_FREQ_SCALING
#include "emi.inc"
#endif

__mx23_flush_cache_addr:
	.word	arm926_flush_kern_cache_all

ENTRY(mx23_standby_alloc_sz)
	.word	. - mx23_cpu_standby

ENTRY(mx23_cpu_suspend)
	@ save registers on stack
	stmfd	sp!, {r1 - r12, lr}

	@ save context
	mov	r0, #0xd3	@ SVC, Interrupts disabled
	msr	cpsr, r0
	mov	r1, #0xC0000000
	ldr	r1, [r1]
	mrc	p15, 0, r0, c1, c0, 0
	str	r0, [r1, #MMUCTL_OFFS]
	mrc	p15, 0, r0, c15, c1, 0
	str	r0, [r1, #MMUCPACCESS_OFS]
	mrc	p15, 0, r0, c2, c0, 0
	str	r0, [r1, #MMUTTB_OFFS]
	mrc	p15, 0, r0, c3, c0, 0
	str	r0, [r1, #MMUDOMAIN_OFFS]
	mrc	p15, 0, r0, c13, c0, 0
	str	r0, [r1, #MMUPID_OFFS]

	str	sp, [r1, #SVC_SP_OFFS]
	mrs	r0, spsr
	str	r0, [r1, #SVC_SPSR_OFFS]

	add	r2, r1, #FIQ_SPSR_OFFS
	mov	r0, #0xd1	@ FIQ, Interrupts disabled
	msr	cpsr, r0
	mrs	r3, spsr
	stmia	r2!, {r3, r8-r12, sp, lr}

	add	r2, r1, #ABT_SPSR_OFFS
	mov	r0, #0xd7	@ ABT, Interrupts disabled
	msr	cpsr, r0
	mrs	r3, spsr
	stmia	r2!, {r3, sp, lr}

	add	r2, r1, #IRQ_SPSR_OFFS
	mov	r0, #0xd2	@ IRQ, Interrupts disabled
	msr	cpsr, r0
	mrs	r3, spsr
	stmia	r2!, {r3, sp, lr}

	add	r2, r1, #UND_SPSR_OFFS
	mov	r0, #0xdb	@ UND, Interrupts disabled
	msr	cpsr, r0
	mrs	r3, spsr
	stmia	r2!, {r3, sp, lr}

	add	r2, r1, #SYS_SP_OFFS
	mov	r0, #0xdf	@ SYS, Interrupts disabled
	msr	cpsr, r0
	stmia	r2!, {sp, lr}

	add	r2, r1, #SVC_R8_OFFS
	mov	r0, #0xd3	@ Back to SVC, Interrupts disabled
	msr	cpsr, r0

	@ save entry point
	sub	r1, r1, #(0xC0000000 - PHYS_RAM_START)
	mov	r0, #0xC0000000
	str	r1, [r0]
	ldr	r1, __mx23_resume_point
	sub	r1, r1, #(0xC0000000 - PHYS_RAM_START)
	str	r1, [r0, #4]
	mov	r0, #0

	@ clean cache
	ldr	r1, __mx23_flush_cache_addr2
	mov	lr, pc
	mov	pc, r1

	@ enable internal xtal
	mov	r2, #(HW_POWER_MINPWR_ADDR & 0x000000FF)
	orr	r2, r2, #(HW_POWER_MINPWR_ADDR & 0x0000FF00)
	orr	r2, r2, #(HW_POWER_MINPWR_ADDR & 0x00FF0000)
	orr	r2, r2, #(HW_POWER_MINPWR_ADDR & 0xFF000000)
	ldr	r1, [r2]
	orr	r1, r1, #(1<<9)
	str	r1, [r2]
	orr	r1, r1, #(1<<8)
	str	r1, [r2]

	@ enable RTC/RAM clocks
	mov	r0, #(HW_RTC_PERSISTENT0 & 0x000000FF)
	orr	r0, r0, #(HW_RTC_PERSISTENT0 & 0x0000FF00)
	orr	r0, r0, #(HW_RTC_PERSISTENT0 & 0x00FF0000)
	orr	r0, r0, #(HW_RTC_PERSISTENT0 & 0xFF000000)
	mov	r1, #((1<<4)|(1<<5)|1)
	str	r1, [r0, #4]

	@ put DRAM into self refresh
	mov	r0, #(HW_DRAM_CTL08 & 0x000000FF)
	orr	r0, r0, #(HW_DRAM_CTL08 & 0x0000FF00)
	orr	r0, r0, #(HW_DRAM_CTL08 & 0x00FF0000)
	orr	r0, r0, #(HW_DRAM_CTL08 & 0xFF000000)
	ldr	r1, [r0]
	orr	r1, r1, #(1 << 8)
	str	r1, [r0]
	@ wait for it to actually happen
	mov	r0, #(HW_EMI_STAT & 0x000000FF)
	orr	r0, r0, #(HW_EMI_STAT & 0x0000FF00)
	orr	r0, r0, #(HW_EMI_STAT & 0x00FF0000)
	orr	r0, r0, #(HW_EMI_STAT & 0xFF000000)
1:	ldr	r1, [r0]
	teq	r1, #(1 << 1)
	beq	1b
	nop
	nop
	nop
	nop
	nop
	nop

	@ power off RAM
	mov	r0, #(HW_DRAM_CTL06 & 0x000000FF)
	orr	r0, r0, #(HW_DRAM_CTL06 & 0x0000FF00)
	orr	r0, r0, #(HW_DRAM_CTL06 & 0x00FF0000)
	orr	r0, r0, #(HW_DRAM_CTL06 & 0xFF000000)
	ldr	r1, [r0]
	orr	r1, r1, #(1<<24)
	str	r1, [r0]
	nop
	nop
	nop
	nop

	@ do enter sleep
	mov	r0, #(HW_POWER_RESET_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_POWER_RESET_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_POWER_RESET_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_POWER_RESET_ADDR & 0xFF000000)
	mov	r1, #0xFF000000
	orr	r1, r1, #0x00FF0000
	str	r1, [r0, #8]
	mov	r1, #0x3E000000
	orr	r1, r1, #0x00770000
	str	r1, [r0, #4]
	mov	r1, #2
	str	r1, [r0, #8]
	mov	r1, #1
	str	r1, [r0, #4]
	nop
	nop
	nop
	nop
	nop
	nop

	@ sleeping now...

__restore_context:
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4	@ Drain write buffer
	mcr	p15, 0, r0, c8, c7, 0	@ Invalidate TLBs
	mcr	p15, 0, r0, c7, c7, 0	@ Invalidate I & D cache
	nop
	nop

	mov	r0, #0xd3
	msr	cpsr, r0

	bl	__create_temp_page_tables
	mov	r3, r4

	mov	r1, #PHYS_RAM_START
	ldr	r1, [r1]
	ldr	r2, [r1, #MMUDOMAIN_OFFS]
	ldr	r4, [r1, #MMUCPACCESS_OFS]
	ldr	r5, [r1, #MMUPID_OFFS]
	ldr	r6, =__resume_after_mmu
	ldr	r7, [r1, #MMUCTL_OFFS]
	ldr	r8, [r1, #MMUTTB_OFFS]
	add	r1, r1, #(0xC0000000 - PHYS_RAM_START)
	mov	r0, #0
@	mcr	p15, 0, r4, c15, c1, 0	@ cpaccess
	mcr	p15, 0, r5, c13, c0, 0	@ pid
	mcr	p15, 0, r2, c3, c0, 0	@ domain
	mcr	p15, 0, r3, c2, c0, 0	@ ttb
	b	1f
	.align 5
1:	mov	r0, r0
	mcr     p15, 0, r7, c1, c0, 0	@ mmuctl
	nop
	mrc	p15, 0, r0, c3, c0, 0	@ read id
	mov	r0, r0
	mov	r0, r0
	sub	pc, r6, r5, lsr #32
	nop
	nop
	nop
__resume_after_mmu:
	mov	r0, #0
	mcr	p15, 0, r0, c8, c7, 0	@ Invalidate TLBs
	mcr	p15, 0, r0, c7, c7, 0	@ Invalidate I & D cache

	mov	r0, r8
	bl	cpu_arm926_switch_mm

	mov	r0, #0xd1 @FIQ, Interrupts disabled
	ldr	r2, [r1, #FIQ_SPSR_OFFS]
	add	r3, r1, #FIQ_R8_OFFS
	msr	cpsr, r0
	msr	spsr, r2
	ldmia	r3!, {r8-r12, sp, lr}

	mov	r0, #0xd7 @ABT, Interrupts disabled
	ldr	r2, [r1, #ABT_SPSR_OFFS]
	add	r3, r1, #ABT_SP_OFFS
	msr	cpsr, r0
	msr	spsr, r2
	ldmia	r3!, {sp, lr}

	mov	r0, #0xd2 @IRQ, Interrupts disabled
	ldr	r2, [r1, #IRQ_SPSR_OFFS]
	add	r3, r1, #IRQ_SP_OFFS
	msr	cpsr, r0
	msr	spsr, r2
	ldmia	r3!, {sp, lr}

	mov	r0, #0xdb @UND, Interrupts disabled
	ldr	r2, [r1, #UND_SPSR_OFFS]
	add	r3, r1, #UND_SP_OFFS
	msr	cpsr, r0
	msr	spsr, r2
	ldmia	r3!, {sp, lr}

	mov	r0, #0xdf @SYS, Interrupts disabled
	add	r3, r1, #SYS_SP_OFFS
	msr	cpsr, r0
	ldmia	r3!, {sp, lr}

	mov	r0, #0xd3 @SVC, interrupts disabled
	ldr	r2, [r1, #SVC_SPSR_OFFS]
	ldr	r3, [r1, #SVC_SP_OFFS]
	msr	cpsr, r0
	msr	spsr, r2
	mov	sp, r3

#if 0
	@ select CPU bypass, will be cleared afterwards
	ldr	r0, =HW_CLKCTRL_CLKSEQ_ADDR
	ldr	r2, =HW_CLKCTRL_HBUS_ADDR
	ldr	r4, =HW_CLKCTRL_CPU_ADDR
	mov	r1, #(1<<7)
	ldr	r3, [r2]
	bic	r3, r3, #BM_CLKCTRL_HBUS_DIV
	orr	r3, r3, #1
	ldr	r5, [r4]
	bic	r5, r5, #BM_CLKCTRL_CPU_DIV_CPU
	orr	r5, r5, #1
	str	r1, [r0, #4]
	str	r3, [r2]
	str	r5, [r4]
#endif
	@ restore regs and return
	ldmfd   sp!, {r1 - r12, lr}
	mov	pc, lr

__mx23_flush_cache_addr2:
	.word	arm926_flush_kern_cache_all
__mx23_resume_point:
	.word	__restore_context
ENTRY(mx23_s2ram_alloc_sz)
	.word	. - mx23_cpu_suspend

__create_temp_page_tables:
	ldr	r4, =(__temp_ttb - 0xC0000000 + PHYS_RAM_START)

	/*
	 * Clear the 16K level 1 swapper page table
	 */
	mov	r0, r4
	mov	r3, #0
	add	r6, r0, #0x4000
1:	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
	teq	r0, r6
	bne	1b

	/*
	 * Create identity mapping for the area close to where we are to
	 * cater for the MMU enable.
	 */
	mov	r6, pc, lsr #20			@ kind of where we are
	ldr	r7, =\
	(PMD_TYPE_SECT | PMD_SECT_BUFFERABLE | PMD_SECT_CACHEABLE\
	| PMD_BIT4 | PMD_SECT_AP_WRITE | PMD_SECT_AP_READ)

	orr	r3, r7, r6, lsl #20		@ flags + kernel base
	str	r3, [r4, r6, lsl #2]		@ identity mapping

	mov	r6, r6, lsl #20
	add	r6, r6, #(0xC0000000-PHYS_RAM_START)
	str	r3, [r4, r6, lsr #18]

	mov	pc, lr
	.ltorg

	.section ".sdata", "a"
	.align 14
__temp_ttb:
	.space 0x8000
