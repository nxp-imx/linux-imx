/*
 * Freescale STMP378X low level RAM timings tables for Micron mDDR
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
__stmp_emi_set_values:
	stmfd	r9!, {r0 - r4, lr}
	mov	r1, #(HW_CLKCTRL_EMI_ADDR & 0x000000FF)
	orr	r1, r1, #(HW_CLKCTRL_EMI_ADDR & 0x0000FF00)
	orr	r1, r1, #(HW_CLKCTRL_EMI_ADDR & 0x00FF0000)
	orr	r1, r1, #(HW_CLKCTRL_EMI_ADDR & 0xFF000000)

	mov	r3, #BM_CLKCTRL_EMI_DCC_RESYNC_ENABLE

	mov	r0, #(HW_CLKCTRL_FRAC_ADDR & 0x000000FF)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_ADDR & 0x0000FF00)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_ADDR & 0x00FF0000)
	orr	r0, r0, #(HW_CLKCTRL_FRAC_ADDR & 0xFF000000)
	ldr	r2, [r0]

	and	r4, r2, #BM_CLKCTRL_FRAC_EMIFRAC
	lsr	r4, r4, #8
	/* new pll div > cur pll div? */
	cmp	r4, r8
	bgt	1f
	bic	r4, r2, #BM_CLKCTRL_FRAC_EMIFRAC
	orr	r4, r4, r8, lsl #8
	str	r4, [r0]
	nop
	nop
	nop

1:	ldr	r4, [r1]
	and	r4, r4, #BM_CLKCTRL_EMI_DIV_EMI
	/* new emi div > cur emi div? */
	cmp	r4, r7
	bgt	2f
	mov	r4, r7
	orr	r4, r4, #0x100
	orr	r4, r4, r3
	str	r4, [r1]
11:	ldr	r4, [r1]
	tst	r4, #BM_CLKCTRL_EMI_BUSY_REF_EMI
	bne	11b
	tst	r4, #BM_CLKCTRL_EMI_BUSY_REF_XTAL
	bne	11b
	tst	r4, #BM_CLKCTRL_EMI_BUSY_DCC_RESYNC
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
	orr	r4, r4, r3
	str	r4, [r1]
31:	ldr	r4, [r1]
	tst	r4, #BM_CLKCTRL_EMI_BUSY_REF_EMI
	bne	31b
	tst	r4, #BM_CLKCTRL_EMI_BUSY_REF_XTAL
	bne	31b
	tst	r4, #BM_CLKCTRL_EMI_BUSY_DCC_RESYNC
	bne	31b

4:	ldmfd	r9!, {r0 - r4, lr}
	mov	pc, lr

stmp3xxx_ram_24M_set_timings:
	ldr	r0, __stmp_dram_ctl00
	adr	r1, __stmp_dram_24M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

stmp3xxx_ram_48M_set_timings:
	ldr	r0, __stmp_dram_ctl00
	adr	r1, __stmp_dram_48M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

stmp3xxx_ram_60M_set_timings:
	ldr	r0, __stmp_dram_ctl00
	adr	r1, __stmp_dram_60M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

stmp3xxx_ram_80M_set_timings:
	ldr	r0, __stmp_dram_ctl00
	adr	r1, __stmp_dram_80M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

stmp3xxx_ram_96M_set_timings:
	ldr	r0, __stmp_dram_ctl00
	adr	r1, __stmp_dram_96M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

stmp3xxx_ram_120M_set_timings:
	ldr	r0, __stmp_dram_ctl00
	adr	r1, __stmp_dram_120M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

stmp3xxx_ram_133M_set_timings:
	ldr	r0, __stmp_dram_ctl00
	adr	r1, __stmp_dram_133M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	str	r3, [r0, r2, lsl #2]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

stmp3xxx_ram_150M_set_timings:
	ldr	r0, __stmp_dram_ctl00
	adr	r1, __stmp_dram_150M_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	str	r3, [r0, r2, lsl #2]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

__stmp_dram_ctl00:
	.word	IO_ADDRESS(0x800E0000)
__stmp_dram_emi00:
	.word	IO_ADDRESS(0x80020000)
__stmp_power_vdddctrl:
	.word	IO_ADDRESS(0x80044040)

stmp3xxx_ram_save_timings:
	ldr	r0, __stmp_dram_ctl00
	adr	r1, __stmp_dram_saved_values
1:	ldr	r2, [r1]
	mov	r4, r2, lsl #2
	ldr	r3, [r0, r4]
	str	r3, [r1, #4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

#ifdef CONFIG_STMP378X_RAM_MDDR
__stmp_dram_24M_values:
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

__stmp_dram_48M_values:
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

__stmp_dram_60M_values:
__stmp_dram_80M_values:
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

__stmp_dram_96M_values:
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

__stmp_dram_120M_values:
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

__stmp_dram_133M_values:
__stmp_dram_150M_values:
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

#elif CONFIG_STMP378X_RAM_DDR
/* XXX: not quite ready yet */
__stmp_dram_24M_values:
	.word	4
	.word	0x01000101
	.word	7
	.word	0x01000101
	.word	11
	.word	0x00070206
	.word	12
	.word	0x01010000 @ t_wr 1, t_rrd 1, t_cke 0
	.word	13
	.word	0x04040a01 @ t_wtr 1
	.word	15
	.word	0x01020000 @ t_rp 1, t_dal 2
	.word	17
	.word	0x3d000302 @ t_rc 2
	.word	20
	.word	0x01020508
	.word	21
	.word	0x00000002 @ t_rfc 2
	.word	26
	.word	0x000000b3 /* 0xd20 */ @ t_ref
	.word	32
	.word	0x00020690 @ t_xsnr 2, t_rasmax 0x690
	.word	33
	.word	0x000000c8 @ t_xsr 0xc8
	.word	34
	.word	0x000012c1 @ t_init
	.word	40
	.word	0x00010000

@ not yet
__stmp_dram_48M_values:
	.word	4
	.word	0x01000101
	.word	7
	.word	0x01000101
	.word	11
	.word	0x00070206
	.word	12
	.word	0x01010000 @ t_wr 1, t_rrd 1, t_cke 0
	.word	13
	.word	0x04040a01 @ t_wtr 1
	.word	15
	.word	0x01020000 @ t_rp 1, t_dal 2
	.word	17
	.word	0x39000104 @ t_rc 4
	.word	19
	.word	0x027f1010
	.word	20
	.word	0x02030a10
	.word	21
	.word	0x00000004 @ t_rfc
	.word	26
	.word	0x00000173 /* 0x1a42 */ @ t_ref
	.word	32
	.word	0x00040d21 @ t_xsnr 4, t_rasmax 0xd21
	.word	33
	.word	0x000000c8 @ t_xsr 0xc8
	.word	34
	.word	0x00002586 @ t_init
	.word	40
	.word	0x00010000

__stmp_dram_60M_values:
	.word	4
	.word	0x01000101
	.word	7
	.word	0x01000101
	.word	11
	.word	0x00070206
	.word	12
	.word	0x01010000 @ t_wr 1, t_rrd 1, t_cke 0
	.word	13
	.word	0x04040a01 @ t_wtr 1
	.word	15
	.word	0x01020000 @ t_rp 1, t_dal 2
	.word	17
	.word	0x3d000105 @ t_rc 5
	.word	19
	.word	0x027f1313
	.word	20
	.word	0x01031523 @ t_rcd 1, t_rasmin 3
	.word	21
	.word	0x00000005 @ t_rfc 5
	.word	26
	.word	0x000001cc /* 0x20cd */ @ t_ref
	.word	32
	.word	0x00051068 @ t_xsnr 5, t_rasmax 0x1068
	.word	33
	.word	0x000000c8 @ t_xsr 0xc8
	.word	34
	.word	0x00002ee5 @ t_init
	.word	40
	.word	0x00010000

__stmp_dram_80M_values:
	.word	4
	.word	0x00000101
	.word	7
	.word	0x01000001
	.word	11
	.word	0x00070206
	.word	12
	.word	0x02010000 @ t_wr 2, t_rrd 1, t_cke 0
	.word	13
	.word	0x04040a01 @ t_wtr 1
	.word	15
	.word	0x02040000 @ t_rp 2, t_dal 4
	.word	17
	.word	0x20001c05 @ dll_start_point 0x20, dll_increment 0x1c, t_rc 5
	.word	19
	.word	0x027f1313
	.word	20
	.word	0x02041522 @ t_rcd 2, t_rasmin 4, wr_dqs_shift 0x22
	.word	21
	.word	0x00000006 @ t_rfc 6
	.word	26
	.word	0x00000269 @ t_ref
	.word	32
	.word	0x000615d6 @ t_xsnr 6, t_rasmax 0x15d6
	.word	33
	.word	0x000000c8 @ t_xsr 0xc8
	.word	34
	.word	0x00003e80 @ t_init
	.word	40
	.word	0x00010000

__stmp_dram_96M_values:
	.word	4
	.word	0x00000101
	.word	7
	.word	0x01000001
	.word	11
	.word	0x00070206
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

__stmp_dram_120M_values:
	.word	4
	.word	0x00000101
	.word	7
	.word	0x01000001
	.word	11
	.word	0x00070206
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

__stmp_dram_133M_values:
	.word	4
	.word	0x00000101
	.word	7
	.word	0x01000001
	.word	11
	.word	0x00070204
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

__stmp_dram_150M_values:
	.word	4
	.word	0x00000101
	.word	7
	.word	0x01000001
	.word	11
	.word	0x00070206
	.word	12
	.word	0x02020000 @ t_wr 2, t_rrd 2, t_cke 0
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

stmp3xxx_ram_restore_timings:
	ldr	r0, __stmp_dram_ctl00
	adr	r1, __stmp_dram_saved_values
1:	ldr	r2, [r1]
	ldr	r3, [r1, #4]
	mov	r4, r2, lsl #2
	str	r3, [r0, r4]
	add	r1, r1, #8
	cmp	r2, #40
	bne	1b
	mov	pc, lr

__stmp_dram_saved_values:
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

