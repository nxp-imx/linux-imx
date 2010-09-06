/*
 * Freescale MX23 low level RAM timings tables for Micron mDDR
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

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


__mx23_emi_set_values_xtal:
	stmfd	r9!, {r0 - r4, lr}

	mov	r1, #(HW_CLKCTRL_EMI_ADDR & 0x000000FF)
	orr	r1, r1, #(HW_CLKCTRL_EMI_ADDR & 0x0000FF00)
	orr	r1, r1, #(HW_CLKCTRL_EMI_ADDR & 0x00FF0000)
	orr	r1, r1, #(HW_CLKCTRL_EMI_ADDR & 0xFF000000)

32:	ldr	r4, [r1]
	tst	r4, #BM_CLKCTRL_EMI_BUSY_REF_XTAL
	bne	32b
	b	4f

__mx23_emi_set_values2:

	stmfd	r9!, {r0 - r4, lr}

	mov	r1, #(HW_CLKCTRL_EMI_ADDR & 0x000000FF)
	orr	r1, r1, #(HW_CLKCTRL_EMI_ADDR & 0x0000FF00)
	orr	r1, r1, #(HW_CLKCTRL_EMI_ADDR & 0x00FF0000)
	orr	r1, r1, #(HW_CLKCTRL_EMI_ADDR & 0xFF000000)

	mov	r0, #(HW_CLKCTRL_FRAC_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_ADDR & 0xFF000000)
	ldr	r2, [r0]

	@clear EMIFRAC bits and store result in r4
	bic	r4, r2, #BM_CLKCTRL_FRAC_EMIFRAC

	orr 	r4, r4, r8, lsl #BP_CLKCTRL_FRAC_EMIFRAC
	str 	r4, [r0]

	@ ungate ref_emi
	mov	r0, #(HW_CLKCTRL_FRAC_CLR_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_CLR_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_CLR_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_CLR_ADDR & 0xFF000000)

	mov r2, #(BM_CLKCTRL_FRAC_CLKGATEEMI)
	str r2, [r0]


	@ set the integer divider
	ldr r2, [r1]
	bic r2, r2, #BM_CLKCTRL_EMI_DIV_EMI
	orr r2, r2, r7, lsl #BP_CLKCTRL_EMI_DIV_EMI

	str r2, [r1]

	@ wait for clock to stabilize
50:	ldr	r2, [r1]
	tst	r2, #BM_CLKCTRL_EMI_BUSY_REF_EMI
	bne	50b
	b 4f

@ Change integer/fractional dividers.

@ The fractional divider and integer divider must be written in such
@  an order to guarantee that when going from a lower frequency to a
@  higher frequency that any intermediate frequencies do not exceed
@  the final frequency. For this reason, we must make sure to check
@  the current divider values with the new divider values and write
@  them in the correct order.

1:	ldr	r4, [r1]
	and	r4, r4, #BM_CLKCTRL_EMI_DIV_EMI
	/* new emi div > cur emi div? */
	cmp	r4, r7
	bgt	2f
	mov	r4, r7
	orr	r4, r4, #0x100

@ This was for DCC_RESYNC_ENABLE, which is deprecated in mx23
@	orr	r4, r4, r3

	str	r4, [r1]
11:	ldr	r4, [r1]
	tst	r4, #BM_CLKCTRL_EMI_BUSY_REF_EMI
	bne	11b
	tst	r4, #BM_CLKCTRL_EMI_BUSY_REF_XTAL
	bne	11b

2:	ldr	r2, [r0]

	and	r4, r2, #BM_CLKCTRL_FRAC_EMIFRAC
	lsr	r4, r4, #8
	/* new pll div != cur pll div? */
	cmp	r4, r8
	beq	3f
	bic	r4, r2, #BM_CLKCTRL_FRAC_EMIFRAC
	orr	r4, r4, r8, lsl #8
	str	r4, [r0]
	nop
	nop
	nop

3:	ldr	r4, [r1]
	and	r4, r4, #BM_CLKCTRL_EMI_DIV_EMI
	/* new emi div != cur emi div? */
	cmp	r4, r7
	beq	4f
	mov	r4, r7
	orr	r4, r4, #0x100

@ This was for DCC_RESYNC_ENABLE, which is deprecated in mx23
@	orr	r4, r4, r3
	str	r4, [r1]
31:	ldr	r4, [r1]
	tst	r4, #BM_CLKCTRL_EMI_BUSY_REF_EMI
	bne	31b

4:	ldmfd	r9!, {r0 - r4, lr}
	mov	pc, lr

mx23_ram_24M_set_timings:
	ldr	r0, __mx23_dram_ctl00
#ifdef CONFIG_MXS_RAM_MDDR
	adr	r1, __mx23_dram_24M_values
#else
//  96MHz is the lowest frequency supported for DDR1.
	adr	r1, __mx23_dram_96M_values
#endif
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

mx23_ram_48M_set_timings:
	ldr	r0, __mx23_dram_ctl00
#ifdef CONFIG_MXS_RAM_MDDR
	adr	r1, __mx23_dram_48M_values
#else
//  96MHz is the lowest frequency supported for DDR1.
	adr	r1, __mx23_dram_96M_values
#endif
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

mx23_ram_60M_set_timings:
	ldr	r0, __mx23_dram_ctl00
#ifdef CONFIG_MXS_RAM_MDDR
	adr	r1, __mx23_dram_60M_values
#else
//  96MHz is the lowest frequency supported for DDR1.
	adr	r1, __mx23_dram_96M_values
#endif
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

mx23_ram_80M_set_timings:
	ldr	r0, __mx23_dram_ctl00
#ifdef CONFIG_MXS_RAM_MDDR
	adr	r1, __mx23_dram_80M_values
#else
//  96MHz is the lowest frequency supported for DDR1.
	adr	r1, __mx23_dram_96M_values
#endif
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

mx23_ram_96M_set_timings:
	ldr	r0, __mx23_dram_ctl00
	adr	r1, __mx23_dram_96M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

mx23_ram_120M_set_timings:
	ldr	r0, __mx23_dram_ctl00
	adr	r1, __mx23_dram_120M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

mx23_ram_133M_set_timings:
	ldr	r0, __mx23_dram_ctl00
	adr	r1, __mx23_dram_133M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	str	r3, [r0, r2, lsl #2]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

mx23_ram_150M_set_timings:
	ldr	r0, __mx23_dram_ctl00
	adr	r1, __mx23_dram_150M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	str	r3, [r0, r2, lsl #2]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

__mx23_dram_ctl00:
	.word 	MX23_SOC_IO_ADDRESS(REGS_DRAM_PHYS)
__mx23_dram_emi00:
	.word	MX23_SOC_IO_ADDRESS(REGS_EMI_PHYS)
__mx23_power_vdddctrl:
	.word	MX23_SOC_IO_ADDRESS(REGS_POWER_PHYS + HW_POWER_VDDDCTRL)

mx23_ram_save_timings:
	ldr	r0, __mx23_dram_ctl00
	adr	r1, __mx23_dram_saved_values
1:	ldr	r2, [r1]
	mov	r4, r2, lsl #2
	ldr	r3, [r0, r4]
	str	r3, [r1, #4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

#ifdef CONFIG_MXS_RAM_MDDR
__mx23_dram_24M_values:
	.word    4
	.word    0x01000101
	.word    7
	.word    0x01000101
	.word    12
	.word    0x02010002
	.word    13
	.word    0x06060a02
	.word    15
	.word    0x01030000
	.word    17
	.word    0x2d000102
	.word    18
	.word    0x20200000
	.word    19
	.word    0x027f1414
	.word    20
	.word    0x01021608
	.word    21
	.word    0x00000002
	.word    26
	.word    0x000000b3
	.word    32
	.word    0x00030687
	.word    33
	.word    0x00000003
	.word    34
	.word    0x000012c1
	.word    40
	.word    0x00010000

__mx23_dram_48M_values:
	.word    4
	.word    0x01000101
	.word    7
	.word    0x01000101
	.word    13
	.word    0x06060a02
	.word    12
	.word    0x02010002
	.word    15
	.word    0x02040000
	.word    17
	.word    0x2d000104
	.word    18
	.word    0x1f1f0000
	.word    19
	.word    0x027f0a0a
	.word    20
	.word    0x02030a10
	.word    21
	.word    0x00000004
	.word    26
	.word    0x0000016f
	.word    32
	.word    0x00060d17
	.word    33
	.word    0x00000006
	.word    34
	.word    0x00002582
	.word    40
	.word    0x00020000

__mx23_dram_60M_values:
__mx23_dram_80M_values:
	.word    4
	.word    0x01000101
	.word    7
	.word    0x01000101
	.word    12
	.word    0x02020002
	.word    13
	.word    0x06060a02
	.word    15
	.word    0x02040000
	.word    17
	.word    0x2d000005
	.word    18
	.word    0x1f1f0000
	.word    19
	.word    0x027f0a0a
	.word    20
	.word    0x02040a10
	.word    21
	.word    0x00000006
	.word    26
	.word    0x000001cc
	.word    32
	.word    0x00081060
	.word    33
	.word    0x00000008
	.word    34
	.word    0x00002ee5
	.word    40
	.word    0x00020000

__mx23_dram_96M_values:
	.word    4
	.word    0x00000101
	.word    7
	.word    0x01000001
	.word    12
	.word    0x02020002
	.word    13
	.word    0x06070a02
	.word    15
	.word    0x03050000
	.word    17
	.word    0x2d000808
	.word    18
	.word    0x1f1f0000
	.word    19
	.word    0x020c1010
	.word    20
	.word    0x0305101c
	.word    21
	.word    0x00000007
	.word    26
	.word    0x000002e6
	.word    32
	.word    0x000c1a3b
	.word    33
	.word    0x0000000c
	.word    34
	.word    0x00004b0d
	.word    40
	.word    0x00030000

__mx23_dram_120M_values:
	.word    4
	.word    0x00000101
	.word    7
	.word    0x01000001
	.word    12
	.word    0x02020002
	.word    13
	.word    0x06070a02
	.word    15
	.word    0x03050000
	.word    17
	.word    0x2300080a
	.word    18
	.word    0x1f1f0000
	.word    19
	.word    0x020c1010
	.word    20
	.word    0x0306101c
	.word    21
	.word    0x00000009
	.word    26
	.word    0x000003a1
	.word    32
	.word    0x000f20ca
	.word    33
	.word    0x0000000f
	.word    34
	.word    0x00005dca
	.word    40
	.word    0x00040000

__mx23_dram_133M_values:
__mx23_dram_150M_values:
	.word    4
	.word    0x00000101
	.word    7
	.word    0x01000001
	.word    12
	.word    0x02020002
	.word    13
	.word    0x06070a02
	.word    15
	.word    0x03050000
	.word    17
	.word    0x2000080a
	.word    18
	.word    0x1f1f0000
	.word    19
	.word    0x020c1010
	.word    20
	.word    0x0306101c
	.word    21
	.word    0x0000000a
	.word    26
	.word    0x00000408
	.word    32
	.word    0x0010245f
	.word    33
	.word    0x00000010
	.word    34
	.word    0x00006808
	.word    40
	.word    0x00040000

#elif CONFIG_MXS_RAM_DDR

__mx23_dram_96M_values:
	.word	4
	.word	0x00000101
	.word	7
	.word	0x01000001
	.word	11
	.word	0x00070202
	.word	12
	.word	0x02020000 @ t_wr 2, t_rrd 2, t_cke 0
	.word	13
	.word	0x04040a01 @ t_wtr 1
	.word	15
	.word	0x02040000 @ t_rp 2, t_dal 4
	.word	17
	.word	0x2f001706 @ dll_start_point 0x2f, dll_increment 0x17, t_rc 6
	.word	19
	.word	0x027f1a1a
	.word	20
	.word	0x02051c21 @ t_rcd 2, t_rasmin 5, wr_dqs_shift 0x22
	.word	21
	.word	0x00000007 @ t_rfc 7
	.word	26
	.word	0x000002e6 /* 0x347b */ @ t_ref
	.word	32
	.word	0x00081a3e @ t_xsnr 8, t_rasmax 0x1a3e
	.word	33
	.word	0x000000c8 @ t_xsr 0xc8
	.word	34
	.word	0x00004b0d @ t_init
	.word	40
	.word	0x00010000

__mx23_dram_120M_values:
	.word	4
	.word	0x00000101
	.word	7
	.word	0x01000001
	.word	11
	.word	0x00070202
	.word	12
	.word	0x02020000 @ t_wr 2, t_rrd 2, t_cke 0
	.word	13
	.word	0x04040a01 @ t_wtr 1
	.word	15
	.word	0x02040000 @ t_rp 2, t_dal 4
	.word	17
	.word	0x26001308 @ dll_start_point 0x26, dll_increment 0x13, t_rc 8
	.word	19
	.word	0x027f1a1a
	.word	20
	.word	0x02061c23 @ t_rcd 2, t_rasmin 6
	.word	21
	.word	0x00000009 @ t_rfc 9
	.word	26
	.word	0x000003a1 /* 0x41a6 */ @ t_ref
	.word	32
	.word	0x000a20ca @ t_xsnr 9, t_rasmax 0x20ca
	.word	33
	.word	0x000000c8 @ t_xsr 0xc8
	.word	34
	.word	0x00005dca @ t_init
	.word	40
	.word	0x00010000

__mx23_dram_133M_values:
	.word	4
	.word	0x00000101
	.word	7
	.word	0x01000001
	.word	11
	.word	0x00070202
	.word	12
	.word	0x02020000
	.word	13
	.word	0x04040a01
	.word	15
	.word	0x02040000
	.word	17
	.word	0x19000f08 @ t_rc 0xa
	.word	19
	.word	0x02021313
	.word	20
	.word	0x02061521
	.word	21
	.word	0x0000000a
	.word	26
	.word	0x000003f7 /* 0x48b9 */
	.word	32
	.word	0x000a23cd
	.word	33
	.word	0x000000c8 @ t_xsr 0xc8
	.word	34
	.word	0x00006665
	.word	40
	.word	0x00010000

__mx23_dram_150M_values:
	.word	4
	.word	0x00000101
	.word	7
	.word	0x01000001
/*
Note that CASLAT of 0x06 means 2.5 cycles.  This is needed to operate at this
frequency. HOWEVER, we would need to implement the setting of WRITEMODEREG
after setting CAS latency to assure that the new CAS latency is actually
being used in the EMI controller.  Otherwise, the controller will still be
using whatever was set the first time the EMI controller was initialized.
Also, a CASLAT of 2.5 needs caslat_lin and caslat_lin_gate of 0x05
(also 2.5 cycles).
*/
	.word	11
	.word	0x00070206
	.word	12
	.word	0x02020000 @ t_wr 2, t_rrd 2, t_cke 0
/*
	caslat_lin and caslat_lin_gate of 0x05 since CASLAT is 0x06 (2.5 cycles).  See above note.
*/
	.word	13
	.word	0x05050a02 @ t_wtr 2
	.word	15
	.word	0x03060000 @ t_rp 3, t_dal 6
	.word	17
	.word	0x18000d0c @ dll_start_point 0x18, dll_increment 0xd, t_rc 0xc
	.word	19
	.word	0x027f0f0f
	.word	20
	.word	0x03071121 @ t_rcd 3, t_rasmin 7
	.word	21
	.word	0x0000000c @ t_rfc 0xc
	.word	26
	.word	0x000001cc /* 0x20cd */ @ t_ref
	.word	32
	.word	0x000c2860 @ t_xsnr 0xc, t_rasmax 0x2860
	.word	33
	.word	0x000000c8 @ t_xsr 0xc8
	.word	34
	.word	0x00007554 @ t_init
	.word	40
	.word	0x00010000

#else
#error RAM chip not defined
#endif

mx23_ram_restore_timings:
	ldr	r0, __mx23_dram_ctl00
	adr	r1, __mx23_dram_saved_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

__mx23_dram_saved_values:
	.word	4
	.word	0
	.word	7
	.word	0
	.word	12
	.word	0
	.word	13
	.word	0
	.word	15
	.word	0
	.word	17
	.word	0
	.word	18
	.word	0
	.word	19
	.word	0
	.word	20
	.word	0
	.word	21
	.word	0
	.word	26
	.word	0
	.word	32
	.word	0
	.word	33
	.word	0
	.word	34
	.word	0
	.word	40
	.word	0

