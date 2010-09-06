/*
 * Freescale MX23 low level RAM frequency manipulation
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
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
#include <asm/system.h>
#include <asm/pgtable-hwdef.h>
#include <mach/hardware.h>
#include <mach/regs-power.h>
#include <mach/regs-emi.h>
#include "regs-clkctrl.h"
#include "regs-dram.h"
#include "regs-digctl.h"

/* TODO should be move to clock.h */
#define SCALING_DATA_EMI_DIV_OFFSET     0
#define SCALING_DATA_FRAC_DIV_OFFSET    4
#define SCALING_DATA_CUR_FREQ_OFFSET    8
#define SCALING_DATA_NEW_FREQ_OFFSET    12
#define REGS_CLKCTRL_BASE MX23_SOC_IO_ADDRESS(CLKCTRL_PHYS_ADDR)
#define HW_CLKCTRL_EMI_ADDR (REGS_CLKCTRL_BASE + HW_CLKCTRL_EMI)
#define HW_CLKCTRL_FRAC_SET_ADDR (REGS_CLKCTRL_BASE + HW_CLKCTRL_FRAC_SET)
#define HW_CLKCTRL_FRAC_CLR_ADDR (REGS_CLKCTRL_BASE + HW_CLKCTRL_FRAC_CLR)
#define HW_CLKCTRL_FRAC_ADDR (REGS_CLKCTRL_BASE + HW_CLKCTRL_FRAC)
#define HW_EMI_CTRL_ADDR MX23_SOC_IO_ADDRESS(REGS_EMI_PHYS + HW_EMI_CTRL)
#define HW_DRAM_CTL04_ADDR MX23_SOC_IO_ADDRESS(REGS_DRAM_PHYS + HW_DRAM_CTL04)
#define HW_CLKCTRL_CLKSEQ_ADDR (REGS_CLKCTRL_BASE + HW_CLKCTRL_CLKSEQ)
.global cpu_arm926_switch_mm

.align 8
ENTRY(mxs_ram_freq_scale)
	stmfd	sp!, {r1 - r9, lr}

	ldr	r5, [r0, #SCALING_DATA_NEW_FREQ_OFFSET]
	ldr	r6, [r0, #SCALING_DATA_CUR_FREQ_OFFSET]
	ldr	r7, [r0, #SCALING_DATA_EMI_DIV_OFFSET]
	ldr	r8, [r0, #SCALING_DATA_FRAC_DIV_OFFSET]

	adr	r9, __mxs_temp_stack

	@ clean cache
	ldr	r1, __mxs_flush_cache_addr
	mov	lr, pc
	mov	pc, r1

	@ put DRAM into self refresh
	ldr	r0, __mx23_dram_ctl00
	ldr	r1, [r0, #0x20]
	orr	r1, r1, #(1 << 8)
	str	r1, [r0, #0x20]
	@ wait for it to actually happen
	ldr	r0, __mx23_dram_emi00
1:	ldr	r1, [r0, #0x10]
	tst	r1, #(1 << 1)
	beq	1b
	nop


	@ RAM to clk from xtal
	mov	r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0xFF000000)
	mov	r1, #(1<<6)
	str	r1, [r0, #4]
	mov	r0, #(HW_CLKCTRL_EMI_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0xFF000000)
101:	ldr     r1, [r0]
	tst     r1, #BM_CLKCTRL_EMI_BUSY_REF_XTAL
	bne     101b

	@ Gate ref_emi
	mov	r0, #(HW_CLKCTRL_FRAC_SET_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_SET_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_SET_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_SET_ADDR & 0xFF000000)

	mov r1, #(BM_CLKCTRL_FRAC_CLKGATEEMI)
	str r1, [r0]


	@ prepare for change
	cmp	r5, #24
	bgt	2f
	bl	mx23_ram_24M_set_timings
	b	44f
2:	cmp	r5, #48
	bgt	3f
	bl	mx23_ram_48M_set_timings
	b	55f
3:	cmp	r5, #60
	bgt	4f
	bl	mx23_ram_60M_set_timings
	b	55f
4:	cmp	r5, #80
	bgt	5f
	bl	mx23_ram_80M_set_timings
	b	55f
5:	cmp	r5, #96
	bgt	6f
	bl	mx23_ram_96M_set_timings
	b	55f
6:	cmp	r5, #120
	bgt	7f
	bl	mx23_ram_120M_set_timings
	b	55f
7:	cmp     r5, #133
	bgt	8f
	bl	mx23_ram_133M_set_timings
	b	55f
8:	bl	mx23_ram_150M_set_timings


44:

	bl	__mx23_emi_set_values_xtal

	@ resttore normal DRAM mode
        ldr     r0, __mx23_dram_ctl00
        ldr     r1, [r0, #0x20]
        bic     r1, r1, #(1 << 8)
        str     r1, [r0, #0x20]

        @ wait for it to actually happen
        ldr     r0, __mx23_dram_emi00
99:    	ldr     r1, [r0, #0x10]
        tst     r1, #(1 << 1)
        bne     99b
        b		110f

55:
	@When are using the DLL, reset the DRAM controller and DLL
	@start point logic (via DLL_SHIFT_RESET and DLL_RESET).
	@After changing clock dividers and loading
	@the new HW_DRAM_CTL* parameters, we will wait for a new DLL_LOCK

	@todo - for DRAM's that will use DLL bypass (non DDR1)
	@	we should not use DLL_RESET and DLL_SHIFT_RESET.

	mov     r0,     #(HW_EMI_CTRL_ADDR & 0x000000FF)
	orr     r0, r0, #(HW_EMI_CTRL_ADDR & 0x0000FF00)
	orr     r0, r0, #(HW_EMI_CTRL_ADDR & 0x00FF0000)
	orr     r0, r0, #(HW_EMI_CTRL_ADDR & 0xFF000000)
	ldr     r1, [r0]            @read values of HW_EMI_CTRL into R1
	orr r1, r1, #BM_EMI_CTRL_DLL_SHIFT_RESET    @Set these 2 fields.
	orr r1, r1, #BM_EMI_CTRL_DLL_RESET
	str     r1, [r0]            @write back values to HW_EMI_CTRL register.

	bl __mx23_emi_set_values2

	@ EMI back to PLL
	mov	r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_CLKSEQ_ADDR & 0xFF000000)
	mov	r1, #(BM_CLKCTRL_CLKSEQ_BYPASS_EMI)
	@clear bypass bit
	str	r1, [r0, #8]

	@ Wait for BUSY_REF_EMI, to assure new clock dividers
	@ are done transferring
	mov	r0, #(HW_CLKCTRL_EMI_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_EMI_ADDR & 0xFF000000)
1:	ldr	r1, [r0]
	tst	r1, #BM_CLKCTRL_EMI_BUSY_REF_EMI
	bne	1b
	str	r1, [r0]

@todo - for DRAM's that will use DLL bypass (non DDR1)
@we should not use DLL_RESET and DLL_SHIFT_RESET.
@	if(HW_DRAM_CTL04.B.DLL_BYPASS_MODE==0)
@	{
@
@	    Clear the DLL_RESET and DLL_SHIFT_RESET bitfields
@	    (\todo - is that necessary?
@	     they were already set previously to reset
@            the controller/DLL start point,
@	     so clearing should have no effect..)
@
@	     BF_CS2(EMI_CTRL, DLL_RESET, 0, DLL_SHIFT_RESET, 0);

	mov     r0,     #(HW_EMI_CTRL_ADDR & 0x000000FF)
	orr     r0, r0, #(HW_EMI_CTRL_ADDR & 0x0000FF00)
	orr     r0, r0, #(HW_EMI_CTRL_ADDR & 0x00FF0000)
	orr     r0, r0, #(HW_EMI_CTRL_ADDR & 0xFF000000)
	ldr     r1, [r0]
	bic r1, #BM_EMI_CTRL_DLL_SHIFT_RESET
	bic r1, #BM_EMI_CTRL_DLL_RESET
	str     r1, [r0]

@ Wait for DLL locking.
@        while(HW_DRAM_CTL04.B.DLLLOCKREG==0);

	mov     r0,     #(HW_DRAM_CTL04_ADDR & 0x000000FF)
	orr     r0, r0, #(HW_DRAM_CTL04_ADDR & 0x0000FF00)
	orr     r0, r0, #(HW_DRAM_CTL04_ADDR & 0x00FF0000)
	orr     r0, r0, #(HW_DRAM_CTL04_ADDR & 0xFF000000)
77:	ldr     r1, [r0]
	tst     r1, #BM_DRAM_CTL04_DLLLOCKREG
	beq     77b

88:
	@ resttore normal DRAM mode
        ldr     r0, __mx23_dram_ctl00
        ldr     r1, [r0, #0x20]
        bic     r1, r1, #(1 << 8)
        str     r1, [r0, #0x20]

        @ wait for it to actually happen
        ldr     r0, __mx23_dram_emi00
102:    ldr     r1, [r0, #0x10]
        tst     r1, #(1 << 1)
        bne     102b

110:
@ restore regs and return
	ldmfd   sp!, {r1 - r9, lr}
	mov	pc, lr

	.space	0x100
__mxs_temp_stack:
	.word	0

#include "emi.inc"

__mxs_flush_cache_addr:
	.word	arm926_flush_kern_cache_all

ENTRY(mxs_ram_funcs_sz)
	.word	. - mxs_ram_freq_scale

