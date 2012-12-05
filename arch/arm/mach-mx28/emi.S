/*
 * Freescale MX28 low level RAM frequency manipulation
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008-2012 Freescale Semiconductor, Inc.
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
#include "regs-clkctrl.h"
#include "regs-dram.h"
#include "regs-digctl.h"

#include "emi_settings.h"

.global cpu_arm926_switch_mm

.align 8
ENTRY(mxs_ram_freq_scale)
	stmfd	sp!, {r1 - r12, lr}
	ldr	r5, [r0, #SCALING_DATA_NEW_FREQ_OFFSET]
	ldr	r6, [r0, #SCALING_DATA_CUR_FREQ_OFFSET]
	ldr	r7, [r0, #SCALING_DATA_EMI_DIV_OFFSET]
	mov	r7, r7,	LSL #BP_CLKCTRL_EMI_DIV_EMI
	ldr	r8, [r0, #SCALING_DATA_FRAC_DIV_OFFSET]
	mov	r8, r8,	LSL #BP_CLKCTRL_FRAC0_EMIFRAC
	add	r2, r2, #0xC00
	mov	r12, r2

	@copy memory setting to iram
	mov	r2, #MX28_DRAMCTRLREGNUM
	mov	r0, r12
1:	ldr	r3, [r1]
	str 	r3, [r0]
	add 	r0, r0, #4
	add 	r1, r1, #4
	subs 	r2, r2, #1
	bne 1b

	ldr r1, =arm926_flush_kern_cache_all
	stmfd sp!, {r1-r12,lr}
	mov	lr, pc
	mov	pc, r1
	ldmfd sp!, {r1-r12,lr}

	mov r2, #MX28_SOC_IO_ADDRESS(CLKCTRL_PHYS_ADDR)&0xFF
	orr r2, r2,  #MX28_SOC_IO_ADDRESS(CLKCTRL_PHYS_ADDR)&0xFF00
	orr r2, r2,  #MX28_SOC_IO_ADDRESS(CLKCTRL_PHYS_ADDR)&0xFF0000
	orr r2, r2,  #MX28_SOC_IO_ADDRESS(CLKCTRL_PHYS_ADDR)&0xFF000000

	mov r0, #MX28_SOC_IO_ADDRESS(DRAM_PHYS_ADDR)&0xFF
	orr r0, r0, #MX28_SOC_IO_ADDRESS(DRAM_PHYS_ADDR)&0xFF00
	orr r0, r0, #MX28_SOC_IO_ADDRESS(DRAM_PHYS_ADDR)&0xFF0000
	orr r0, r0, #MX28_SOC_IO_ADDRESS(DRAM_PHYS_ADDR)&0xFF000000

	mov r3, r12

	mov r1, #0x0
        mcr p15,0,r1,c7,c10,4    @ invalidate TLB single entry to ensure that

        mcr p15,0,r0,c8,c7,1    @ invalidate TLB single entry to ensure that
                                @ LockAddr is not already in the TLB
        mcr p15,0,r2,c8,c7,1    @ invalidate TLB single entry to ensure that
                                @ LockAddr is not already in the TLB
        mcr p15,0,r3,c8,c7,1    @ invalidate TLB single entry to ensure that
                                @ LockAddr is not already in the TLB
        mrc p15,0,r1,c10,c0,0   @ read the lockdown register
        orr r1,r1,#1            @ set the preserve bit
        mcr p15,0,r1,c10,c0,0   @ write to the lockdown register
        ldr r1,[r0]             @ TLB will miss, and entry will be loaded
        ldr r1,[r2]             @ TLB will miss, and entry will be loaded
        ldr r1,[r3]             @ TLB will miss, and entry will be loaded
        mrc p15,0,r1,c10,c0,0   @ read the lockdown register (victim will have
                                @ incremented)
        bic r1,r1,#1            @ clear preserve bit
        mcr p15,0,r1,c10,c0,0   @ write to the lockdown registerADR r1,LockAddr
	@bl lock_vector_tlb
	@ Make sure emi not busy
2:
	ldr r1, [r0, #HW_DRAM_CTL08]
	and r1, #BM_DRAM_CTL08_CONTROLLER_BUSY
	cmp r1, #0x0
	bne 2b

	@ put DRAM into self refresh
	ldr r1, [r0, #HW_DRAM_CTL17]
	orr r1, r1, #BM_DRAM_CTL17_SREFRESH
	str	r1,  [r0, #HW_DRAM_CTL17]
3:
	ldr r1, [r0, #HW_DRAM_CTL172]
	tst r1, #BM_DRAM_CTL172_CKE_STATUS
	beq 3b

	ldr r1, [r0, #HW_DRAM_CTL58]
	orr r1, #BF_DRAM_CTL58_INT_MASK(0x100)
	str r1, [r0, #HW_DRAM_CTL58]

	@stop emi controller
	ldr r1, [r0, #HW_DRAM_CTL16]
	bic r1, r1, #BM_DRAM_CTL16_START
	str r1, [r0, #HW_DRAM_CTL16]

	@clear lock status HW_DRAM_CTL164_CLR(BF_DRAM_CTL164_INT_ACK(0x3ff));
	ldr r1, [r0, #HW_DRAM_CTL164]
	bic r1, r1, #BF_DRAM_CTL164_INT_ACK(0xff)
	bic r1, r1, #BF_DRAM_CTL164_INT_ACK(0x300)
	str r1, [r0, #HW_DRAM_CTL164]

	ldr r1, [r2, #HW_CLKCTRL_FRAC0]
	and r1, #BM_CLKCTRL_FRAC0_EMIFRAC
	ldr r3, [r2, #HW_CLKCTRL_EMI]
	and r3, #BM_CLKCTRL_EMI_DIV_EMI

/*
 *	The fractional divider and integer divider must be written in such
 *	an order to guarantee that when going from a lower frequency to a
 *	higher frequency that any intermediate frequencies do not exceed
 *	the final frequency. For this reason, we must make sure to check
 *	the current divider values with the new divider values and write
 *	them in the correct order.
 */

	ldr r9, [r2, #HW_CLKCTRL_FRAC0]
	bic r9, #BM_CLKCTRL_FRAC0_EMIFRAC
	orr r9, r8

	ldr r10, [r2, #HW_CLKCTRL_EMI]
	bic r10, #BM_CLKCTRL_EMI_DIV_EMI
	orr r10, r7

	cmp r8, r1
	strgt r9,   [r2, #HW_CLKCTRL_FRAC0]
	cmp r7, r3
	strgt r10,  [r2, #HW_CLKCTRL_EMI]

	cmp r8, r1
	strlt r9,   [r2, #HW_CLKCTRL_FRAC0]
	cmp r7, r3
	strlt r10,  [r2, #HW_CLKCTRL_EMI]

	@copy memory setting to iram
	mov     r3, r0
	mov     r4, r12
	mov 	r6, #MX28_DRAMCTRLREGNUM
8:	ldr     r5, [r4]
	str     r5, [r3]
	add     r3, r3, #4
	add     r4, r4, #4
	subs    r6, r6, #1
	bne 8b

7:      ldr     r1, [r2, #HW_CLKCTRL_EMI]
        tst     r1, #BM_CLKCTRL_EMI_BUSY_REF_EMI
	bne     7b

	@Restart memory controller
	ldr r1, [r0, #HW_DRAM_CTL16]
	orr r1, #BM_DRAM_CTL16_START
	str	r1, [r0, #HW_DRAM_CTL16]

	/*Wait DLL is locked*/
9:
	ldr r1, [r0, #HW_DRAM_CTL21]
	tst r1, #BM_DRAM_CTL21_DLLLOCKREG
	beq 9b

    	@11. Exit Memory self-refresh
	ldr r1, [r0, #HW_DRAM_CTL17]
	bic r1, r1, #BM_DRAM_CTL17_SREFRESH
	str r1, [r0, #HW_DRAM_CTL17]

	@Wait Memory device exit into self-refresh
10:
	ldr r1, [r0, #HW_DRAM_CTL172]
	tst r1, #BM_DRAM_CTL172_CKE_STATUS
	bne 10b

	mov	r2, #MX28_SOC_IO_ADDRESS(DIGCTL_PHYS_ADDR)&0xFF
	orr	r2, r2, #MX28_SOC_IO_ADDRESS(DIGCTL_PHYS_ADDR)&0xFF00
	orr	r2, r2, #MX28_SOC_IO_ADDRESS(DIGCTL_PHYS_ADDR)&0xFF0000
	orr	r2, r2, #MX28_SOC_IO_ADDRESS(DIGCTL_PHYS_ADDR)&0xFF000000

	ldr 	r0, [r2, #HW_DIGCTL_MICROSECONDS];
	add	r0, #1000
11:	ldr	r1, [r2, #HW_DIGCTL_MICROSECONDS];
	cmp	r1, r0
	blt	11b

@ restore regs and return
	ldmfd   sp!, {r1 - r12, lr}
	mov	pc, lr

	.space	0x100

ENTRY(mxs_ram_funcs_sz)
	.word	. - mxs_ram_freq_scale
ENTRY(mxs_ram_freq_scale_end)
