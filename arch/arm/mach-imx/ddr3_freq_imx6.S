/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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

#define MMDC0_MDPDC		0x4
#define MMDC0_MDCF0			0x0c
#define MMDC0_MDCF1			0x10
#define MMDC0_MDMISC		0x18
#define MMDC0_MDSCR		0x1c
#define MMDC0_MAPSR			0x404
#define MMDC0_MADPCR0		0x410
#define MMDC0_MPZQHWCTRL	0x800
#define MMDC1_MPZQHWCTRL	0x4800
#define MMDC0_MPODTCTRL	0x818
#define MMDC1_MPODTCTRL	0x4818
#define MMDC0_MPDGCTRL0	0x83c
#define MMDC1_MPDGCTRL0	0x483c
#define MMDC0_MPMUR0		0x8b8
#define MMDC1_MPMUR0		0x48b8

#define CCM_CBCDR			0x14
#define CCM_CBCMR			0x18
#define CCM_CSCMR1			0x1c
#define CCM_CDHIPR			0x48

#define L2_CACHE_SYNC		0x730

	.align 3

	.macro	switch_to_528MHz

	/* check if periph_clk_sel is already set */
	ldr	r0, [r6, #CCM_CBCDR]
	and	r0, r0, #(1 << 25)
	cmp	r0, #(1 << 25)
	beq	set_ahb_podf_before_switch

	/* change periph_clk to be sourced from pll3_clk. */
	ldr	r0, [r6, #CCM_CBCMR]
	bic	r0, r0, #(3 << 12)
	str	r0, [r6, #CCM_CBCMR]

	ldr	r0, [r6, #CCM_CBCDR]
	bic	r0, r0, #(0x38 << 20)
	str	r0, [r6, #CCM_CBCDR]

	/*
	 * set the AHB dividers before the switch,
	 * don't change AXI clock divider,
	 * set the MMDC_DIV=1, AXI_DIV = 2, AHB_DIV=4,
	 */
	ldr	r0, [r6, #CCM_CBCDR]
	ldr	r2, =0x3f1f00
	bic	r0, r0, r2
	orr	r0, r0, #0xd00
	orr	r0, r0, #(1 << 16)
	str	r0, [r6, #CCM_CBCDR]

wait_div_update528:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	wait_div_update528

	/* now switch periph_clk to pll3_main_clk. */
	ldr	r0, [r6, #CCM_CBCDR]
	orr	r0, r0, #(1 << 25)
	str	r0, [r6, #CCM_CBCDR]

periph_clk_switch3:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	periph_clk_switch3

	b	switch_pre_periph_clk_528

set_ahb_podf_before_switch:
	/*
	 * set the MMDC_DIV=1, AXI_DIV = 2, AHB_DIV=4,
	 */
	ldr	r0, [r6, #CCM_CBCDR]
	ldr	r2, =0x3f1f00
	bic	r0, r0, r2
	orr	r0, r0, #0xd00
	orr	r0, r0, #(1 << 16)
	str	r0, [r6, #CCM_CBCDR]

wait_div_update528_1:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	wait_div_update528_1

switch_pre_periph_clk_528:

	/* now switch pre_periph_clk to PLL2_528MHz. */
	ldr	r0, [r6, #CCM_CBCMR]
	bic	r0, r0, #(0xc << 16)
	str	r0, [r6, #CCM_CBCMR]

	/* now switch periph_clk back. */
	ldr	r0, [r6, #CCM_CBCDR]
	bic	r0, r0, #(1 << 25)
	str	r0, [r6, #CCM_CBCDR]

periph_clk_switch4:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	periph_clk_switch4

	.endm

	.macro	switch_to_400MHz

	/* check if periph_clk_sel is already set. */
	ldr	r0, [r6, #CCM_CBCDR]
	and	r0, r0, #(1 << 25)
	cmp	r0, #(1 << 25)
	beq	set_ahb_podf_before_switch1

	/* change periph_clk to be sourced from pll3_clk. */
	ldr	r0, [r6, #CCM_CBCMR]
	bic	r0, r0, #(3 << 12)
	str	r0, [r6, #CCM_CBCMR]

	ldr	r0, [r6, #CCM_CBCDR]
	bic	r0, r0, #(0x38 << 24)
	str	r0, [r6, #CCM_CBCDR]

	/* now switch periph_clk to pll3_main_clk. */
	ldr	r0, [r6, #CCM_CBCDR]
	orr	r0, r0, #(1 << 25)
	str	r0, [r6, #CCM_CBCDR]

periph_clk_switch5:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	periph_clk_switch5

	b	switch_pre_periph_clk_400

set_ahb_podf_before_switch1:
	/*
	 * set the MMDC_DIV=1, AXI_DIV = 2, AHB_DIV=4,
	 */
	ldr	r0, [r6, #CCM_CBCDR]
	ldr	r2, =0x3f1f00
	bic	r0, r0, r2
	orr	r0, r0, #(0x9 << 8)
	orr	r0, r0, #(1 << 16)
	str	r0, [r6, #CCM_CBCDR]

wait_div_update400_1:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	wait_div_update400_1

switch_pre_periph_clk_400:

	/* now switch pre_periph_clk to PFD_400MHz. */
	ldr	r0, [r6, #CCM_CBCMR]
	bic	r0, r0, #(0xc << 16)
	orr	r0, r0, #(0x4 << 16)
	str	r0, [r6, #CCM_CBCMR]

	/* now switch periph_clk back. */
	ldr	r0, [r6, #CCM_CBCDR]
	bic	r0, r0, #(1 << 25)
	str	r0, [r6, #CCM_CBCDR]

periph_clk_switch6:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	periph_clk_switch6

	/*
	 * change AHB divider so that we are at 400/3=133MHz.
	 * don't change AXI clock divider.
	 * set the MMDC_DIV=1, AXI_DIV=2, AHB_DIV=3,
	 */
	ldr	r0, [r6, #CCM_CBCDR]
	ldr	r2, =0x3f1f00
	bic	r0, r0, r2
	orr	r0, r0, #(0x9 << 8)
	orr	r0, r0, #(1 << 16)
	str	r0, [r6, #CCM_CBCDR]

wait_div_update400_2:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	wait_div_update400_2

	.endm

	.macro	switch_to_50MHz

	/* check if periph_clk_sel is already set. */
	ldr	r0, [r6, #CCM_CBCDR]
	and	r0, r0, #(1 << 25)
	cmp	r0, #(1 << 25)
	beq	switch_pre_periph_clk_50

	/*
	 * set the periph_clk to be sourced from PLL2_PFD_200M
	 * change periph_clk to be sourced from pll3_clk.
	 * ensure PLL3 is the source and set the divider to 1.
	 */
	ldr	r0, [r6, #CCM_CBCMR]
	bic	r0, r0, #(0x3 << 12)
	str	r0, [r6, #CCM_CBCMR]

	ldr	r0, [r6, #CCM_CBCDR]
	bic	r0, r0, #(0x38 << 24)
	str	r0, [r6, #CCM_CBCDR]

	/* now switch periph_clk to pll3_main_clk. */
	ldr	r0, [r6, #CCM_CBCDR]
	orr	r0, r0, #(1 << 25)
	str	r0, [r6, #CCM_CBCDR]

periph_clk_switch_50:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	periph_clk_switch_50

switch_pre_periph_clk_50:

	/* now switch pre_periph_clk to PFD_200MHz. */
	ldr	r0, [r6, #CCM_CBCMR]
	orr	r0, r0, #(0xc << 16)
	str	r0, [r6, #CCM_CBCMR]

	/*
	 * set the MMDC_DIV=4, AXI_DIV = 4, AHB_DIV=8,
	 */
	ldr	r0, [r6, #CCM_CBCDR]
	ldr	r2, =0x3f1f00
	bic	r0, r0, r2
	orr	r0, r0, #(0x18 << 16)
	orr	r0, r0, #(0x3 << 16)

	/*
	 * if changing AHB divider remember to change
	 * the IPGPER divider too below.
	 */
	orr	r0, r0, #0x1d00
	str	r0, [r6, #CCM_CBCDR]

wait_div_update_50:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	wait_div_update_50

	/* now switch periph_clk back. */
	ldr	r0, [r6, #CCM_CBCDR]
	bic	r0, r0, #(1 << 25)
	str	r0, [r6, #CCM_CBCDR]

periph_clk_switch2:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	periph_clk_switch2

	.endm

	.macro	switch_to_24MHz
	/*
	 * change the freq now try setting DDR to 24MHz.
	 * source it from the periph_clk2 ensure the
	 * periph_clk2 is sourced from 24MHz and the
	 * divider is 1.
	 */

	ldr	r0, [r6, #CCM_CBCMR]
	bic	r0, r0, #(0x3 << 12)
	orr	r0, r0, #(1 << 12)
	str	r0, [r6, #CCM_CBCMR]

	ldr	r0, [r6, #CCM_CBCDR]
	bic	r0, r0, #(0x38 << 24)
	str	r0, [r6, #CCM_CBCDR]

	/* now switch periph_clk to 24MHz. */
	ldr	r0, [r6, #CCM_CBCDR]
	orr	r0, r0, #(1 << 25)
	str	r0, [r6, #CCM_CBCDR]

periph_clk_switch1:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	periph_clk_switch1

	/* change all the dividers to 1. */
	ldr	r0, [r6, #CCM_CBCDR]
	ldr	r2, =0x3f1f00
	bic	r0, r0, r2
	orr	r0, r0, #(1 << 8)
	str	r0, [r6, #CCM_CBCDR]

	/* Wait for the divider to change. */
wait_div_update:
	ldr	r0, [r6, #CCM_CDHIPR]
	cmp	r0, #0
	bne	wait_div_update

	.endm

/*
 *  mx6_ddr3_freq_change
 *
 *  idle the processor (eg, wait for interrupt).
 *  make sure DDR is in self-refresh.
 *  IRQs are already disabled.
 */
ENTRY(mx6_ddr3_freq_change)

	stmfd	sp!, {r4-r12}

	/*
	  * r5 -> mmdc_base
	  * r6 -> ccm_base
	  * r7 -> iomux_base
	  * r12 -> l2_base
	  */
	mov	r4, r0
	mov	r8, r1
	mov	r9, r2
	mov	r11, r3

	/*
	  * Get the addresses of the registers.
	  * They are last few entries in the
	  * ddr_settings parameter.
	  * The first entry contains the count,
	  * and each entry is 2 words.
	  */
	ldr	r0, [r1]
	add	r0, r0, #1
	lsl	r0, r0, #3
	add	r1, r0, r1
	/* mmdc_base. */
	ldr	r5, [r1]
	add	r1, #8
	/* ccm_base */
	ldr	r6, [r1]
	add	r1, #8
	/*iomux_base */
	ldr	r7, [r1]
	add	r1, #8
	/*l2_base */
	ldr	r12, [r1]

ddr_freq_change:
	/*
	 * make sure no TLB miss will occur when
	 * the DDR is in self refresh. invalidate
	 * TLB single entry to ensure that the
	 * address is not already in the TLB.
	 */

	adr	r10, ddr_freq_change

	ldr	r2, [r6]
	ldr	r2, [r5]
	ldr	r2, [r7]
	ldr	r2, [r8]
	ldr	r2, [r10]
	ldr	r2, [r11]
	ldr	r2, [r12]

#ifdef CONFIG_CACHE_L2X0
	/*
	  * Make sure the L2 buffers are drained.
	  * Sync operation on L2 drains the buffers.
	  */
	mov    r1, #0x0
	str      r1, [r12, #L2_CACHE_SYNC]
#endif

	/* disable automatic power saving. */
	ldr	r0, [r5, #MMDC0_MAPSR]
	orr	r0, r0, #0x01
	str	r0, [r5, #MMDC0_MAPSR]

	/* disable MMDC power down timer. */
	ldr	r0, [r5, #MMDC0_MDPDC]
	bic	r0, r0, #(0xff << 8)
	str	r0, [r5, #MMDC0_MDPDC]

	/* delay for a while */
	ldr	r1, =4
delay1:
	ldr	r2, =0
cont1:
	ldr	r0, [r5, r2]
	add	r2, r2, #4
	cmp	r2, #16
	bne	cont1
	sub	r1, r1, #1
	cmp	r1, #0
	bgt	delay1

	/* set CON_REG */
	ldr	r0, =0x8000
	str	r0, [r5, #MMDC0_MDSCR]
poll_conreq_set_1:
	ldr	r0, [r5, #MMDC0_MDSCR]
	and	r0, r0, #(0x4 << 12)
	cmp	r0, #(0x4 << 12)
	bne	poll_conreq_set_1

	ldr	r0, =0x00008010
	str	r0, [r5, #MMDC0_MDSCR]
	ldr	r0, =0x00008018
	str	r0, [r5, #MMDC0_MDSCR]

	/*
	 * if requested frequency is greater than
	 * 300MHz go to DLL on mode.
	 */
	ldr	r1, =300000000
	cmp	r4, r1
	bge	dll_on_mode

dll_off_mode:

	/* if DLL is currently on, turn it off. */
	cmp	r9, #1
	beq	continue_dll_off_1

	ldr	r0, =0x00018031
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r0, =0x00018039
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r1, =10
delay1a:
	ldr	r2, =0
cont1a:
	ldr	r0, [r5, r2]
	add	r2, r2, #4
	cmp	r2, #16
	bne	cont1a
	sub	r1, r1, #1
	cmp	r1, #0
	bgt	delay1a

continue_dll_off_1:
	/* set DVFS - enter self refresh mode */
	ldr	r0, [r5, #MMDC0_MAPSR]
	orr	r0, r0, #(1 << 21)
	str	r0, [r5, #MMDC0_MAPSR]

	/* de-assert con_req */
	mov	r0, #0x0
	str	r0, [r5, #MMDC0_MDSCR]

poll_dvfs_set_1:
	ldr	r0, [r5, #MMDC0_MAPSR]
	and	r0, r0, #(1 << 25)
	cmp	r0, #(1 << 25)
	bne	poll_dvfs_set_1

	ldr	r1, =24000000
	cmp	r4, r1
	beq	switch_freq_24

	switch_to_50MHz
	b	continue_dll_off_2

switch_freq_24:
	switch_to_24MHz

continue_dll_off_2:

	/* set SBS - block ddr accesses */
	ldr	r0, [r5, #MMDC0_MADPCR0]
	orr	r0, r0, #(1 << 8)
	str	r0, [r5, #MMDC0_MADPCR0]

	/* clear DVFS - exit from self refresh mode */
	ldr	r0, [r5, #MMDC0_MAPSR]
	bic	r0, r0, #(1 << 21)
	str	r0, [r5, #MMDC0_MAPSR]

poll_dvfs_clear_1:
	ldr	r0, [r5, #MMDC0_MAPSR]
	and	r0, r0, #(1 << 25)
	cmp	r0, #(1 << 25)
	beq	poll_dvfs_clear_1

	/* if DLL was previously on, continue DLL off routine. */
	cmp    	r9, #1
	beq 	continue_dll_off_3

	ldr	r0, =0x00018031
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r0, =0x00018039
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r0, =0x08208030
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r0, =0x08208038
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r0, =0x00088032
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r0, =0x0008803A
	str	r0, [r5, #MMDC0_MDSCR]

	/* delay for a while. */
	ldr	r1, =4
delay_1:
	ldr	r2, =0
cont_1:
	ldr	r0, [r5, r2]
	add	r2, r2, #4
	cmp	r2, #16
	bne	cont_1
	sub	r1, r1, #1
	cmp	r1, #0
	bgt	delay_1

	ldr	r0, [r5, #MMDC0_MDCF0]
	bic	r0, r0, #0xf
	orr	r0, r0, #0x3
	str	r0, [r5, #MMDC0_MDCF0]

	ldr	r0, [r5, #MMDC0_MDCF1]
	bic	r0, r0, #0x7
	orr	r0, r0, #0x4
	str	r0, [r5, #MMDC0_MDCF1]

	ldr	r0, =0x00091680
	str	r0, [r5, #MMDC0_MDMISC]

	/* enable dqs pull down in the IOMUX. */
	ldr	r1, [r11]
	add	r11, r11, #8
	ldr	r2, =0x3028
update_iomux:
	ldr	r0, [r11, #0x0]
	ldr	r3, [r7, r0]
	bic	r3, r3, r2
	orr	r3, r3, #(0x3 << 12)
	orr	r3, r3, #0x28
	str	r3, [r7, r0]
	add	r11, r11, #8
	sub	r1, r1, #1
	cmp	r1, #0
	bgt	update_iomux

	/*  ODT disabled. */
	ldr	r0, =0x0
	ldr	r2, =MMDC0_MPODTCTRL
	str	r0, [r5, r2]
	ldr	r2, =MMDC1_MPODTCTRL
	str	r0, [r5, r2]

	/* DQS gating disabled. */
	ldr	r2, =MMDC0_MPDGCTRL0
	ldr	r0, [r5, r2]
	orr	r0, r0, #(1 << 29)
	str	r0, [r5, r2]

	ldr	r2, =MMDC1_MPDGCTRL0
	ldr	r0, [r5, r2]
	orr	r0, r0, #(0x1 << 29)
	str	r0, [r5, r2]

	/* MMDC0_MAPSR adopt power down enable. */
	ldr	r0, [r5, #MMDC0_MAPSR]
	bic	r0, r0, #0x01
	str	r0, [r5, #MMDC0_MAPSR]

	/* frc_msr + mu bypass */
	ldr	r0, =0x00000060
	str	r0, [r5, #MMDC0_MPMUR0]
	ldr	r2, =MMDC1_MPMUR0
	str	r0, [r5, r2]
	ldr	r0, =0x00000460
	str	r0, [r5, #MMDC0_MPMUR0]
	ldr	r2, =MMDC1_MPMUR0
	str	r0, [r5, r2]
	ldr	r0, =0x00000c60
	str	r0, [r5, #MMDC0_MPMUR0]
	ldr	r2, =MMDC1_MPMUR0
	str	r0, [r5, r2]

continue_dll_off_3:
	/* clear SBS - unblock accesses to DDR. */
	ldr	r0, [r5, #MMDC0_MADPCR0]
	bic	r0, r0, #(0x1 << 8)
	str	r0, [r5, #MMDC0_MADPCR0]

	mov	r0, #0x0
	str	r0, [r5, #MMDC0_MDSCR]
poll_conreq_clear_1:
	ldr	r0, [r5, #MMDC0_MDSCR]
	and	r0, r0, #(0x4 << 12)
	cmp	r0, #(0x4 << 12)
	beq	poll_conreq_clear_1

	b	done

dll_on_mode:
	/* assert DVFS - enter self refresh mode. */
	ldr	r0, [r5, #MMDC0_MAPSR]
	orr	r0, r0, #(1 << 21)
	str	r0, [r5, #MMDC0_MAPSR]

	/* de-assert CON_REQ. */
	mov	r0, #0x0
	str	r0, [r5, #MMDC0_MDSCR]

	/* poll DVFS ack. */
poll_dvfs_set_2:
	ldr	r0, [r5, #MMDC0_MAPSR]
	and	r0, r0, #(1 << 25)
	cmp	r0, #(1 << 25)
	bne	poll_dvfs_set_2

	ldr	r1, =528000000
	cmp	r4, r1
	beq	switch_freq_528

	switch_to_400MHz

	b	continue_dll_on

switch_freq_528:
	switch_to_528MHz

continue_dll_on:

	/* set SBS step-by-step mode. */
	ldr	r0, [r5, #MMDC0_MADPCR0]
	orr	r0, r0, #( 1 << 8)
	str	r0, [r5, #MMDC0_MADPCR0]

	/* clear DVFS - exit self refresh mode. */
	ldr	r0, [r5, #MMDC0_MAPSR]
	bic	r0, r0, #(1 << 21)
	str	r0, [r5, #MMDC0_MAPSR]

poll_dvfs_clear_2:
	ldr	r0, [r5, #MMDC0_MAPSR]
	and	r0, r0, #(1 << 25)
	cmp	r0, #(1 << 25)
	beq	poll_dvfs_clear_2

	/* if DLL is currently off, turn it back on. */
	cmp	r9, #0
	beq	update_calibration_only

	ldr	r0, =0xa5390003
	str	r0, [r5, #MMDC0_MPZQHWCTRL]
	ldr	r2, =MMDC1_MPZQHWCTRL
	str	r0, [r5, r2]

	/* enable DQS gating. */
	ldr	r2, =MMDC0_MPDGCTRL0
	ldr	r0, [r5, r2]
	bic	r0, r0, #(1 << 29)
	str	r0, [r5, r2]

	ldr	r2, =MMDC1_MPDGCTRL0
	ldr	r0, [r5, r2]
	bic	r0, r0, #(1 << 29)
	str	r0, [r5, r2]

	/* force measure. */
	ldr	r0, =0x00000800
	str	r0, [r5, #MMDC0_MPMUR0]
	ldr	r2, =MMDC1_MPMUR0
	str	r0, [r5, r2]

	/* delay for while. */
	ldr	r1, =4
delay5:
	ldr	r2, =0
cont5:
	ldr	r0, [r5, r2]
	add	r2, r2, #4
	cmp	r2, #16
	bne	cont5
	sub	r1, r1, #1
	cmp	r1, #0
	bgt	delay5

	/* disable dqs pull down in the IOMUX. */
	ldr	r1, [r11]
	add	r11, r11, #8
update_iomux1:
	ldr	r0, [r11, #0x0]
	ldr	r3, [r11, #0x4]
	str	r3, [r7, r0]
	add	r11, r11, #8
	sub	r1, r1, #1
	cmp	r1, #0
	bgt	update_iomux1

	/* config MMDC timings to 528MHz. */
	ldr	r9, [r8]
	add	r8, r8, #8
	ldr	r0, [r8, #0x0]
	ldr	r3, [r8, #0x4]
	str	r3, [r5, r0]
	add	r8, r8, #8

	ldr	r0, [r8, #0x0]
	ldr	r3, [r8, #0x4]
	str	r3, [r5, r0]
	add	r8, r8, #8

	/* update MISC register: WALAT, RALAT */
	ldr	r0, =0x00081740
	str	r0, [r5, #MMDC0_MDMISC]

	/* configure ddr devices to dll on, odt. */
	ldr	r0, =0x00028031
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r0, =0x00028039
	str	r0, [r5, #MMDC0_MDSCR]

	/* delay for while. */
	ldr	r1, =4
delay7:
	ldr	r2, =0
cont7:
	ldr	r0, [r5, r2]
	add	r2, r2, #4
	cmp	r2, #16
	bne	cont7
	sub	r1, r1, #1
	cmp	r1, #0
	bgt	delay7

	/* reset dll. */
	ldr	r0, =0x09208030
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r0, =0x09208038
	str	r0, [r5, #MMDC0_MDSCR]

	/* delay for while. */
	ldr	r1, =100
delay8:
	ldr	r2, =0
cont8:
	ldr	r0, [r5, r2]
	add	r2, r2, #4
	cmp	r2, #16
	bne	cont8
	sub	r1, r1, #1
	cmp	r1, #0
	bgt	delay8

	ldr	r0, [r8, #0x0]
	ldr	r3, [r8, #0x4]
	str	r3, [r5, r0]
	add	r8, r8, #8

	ldr	r0, [r8, #0x0]
	ldr	r3, [r8, #0x4]
	str	r3, [r5, r0]
	add	r8, r8, #8

	ldr	r0, =0x00428031
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r0, =0x00428039
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r0, [r8, #0x0]
	ldr	r3, [r8, #0x4]
	str	r3, [r5, r0]
	add	r8, r8, #8

	ldr	r0, [r8, #0x0]
	ldr	r3, [r8, #0x4]
	str	r3, [r5, r0]
	add	r8, r8, #8

	/* issue a zq command. */
	ldr	r0, =0x04008040
	str	r0, [r5, #MMDC0_MDSCR]

	ldr	r0, =0x04008048
	str	r0, [r5, #MMDC0_MDSCR]

	/* MMDC ODT enable. */
	ldr	r0, [r8, #0x0]
	ldr	r3, [r8, #0x4]
	str	r3, [r5, r0]
	add	r8, r8, #8

	ldr	r2, =0x4818
	str	r0, [r5, r2]

	/* delay for while. */
	ldr	r1, =40
delay15:
	ldr	r2, =0
cont15:
	ldr	r0, [r5, r2]
	add	r2, r2, #4
	cmp	r2, #16
	bne	cont15
	sub	r1, r1, #1
	cmp	r1, #0
	bgt	delay15

	/* MMDC0_MAPSR adopt power down enable. */
	ldr	r0, [r5, #MMDC0_MAPSR]
	bic	r0, r0, #0x01
	str	r0, [r5, #MMDC0_MAPSR]

	/* enable MMDC power down timer. */
	ldr	r0, [r5, #MMDC0_MDPDC]
	orr	r0, r0, #(0x55 << 8)
	str	r0, [r5, #MMDC0_MDPDC]

	b	update_calibration

update_calibration_only:
	ldr	r1, [r8]
	sub	r1, r1, #7
	add	r8, r8, #64
	b	update_calib

update_calibration:
	/* write the new calibration values. */
	mov	r1, r9
	sub	r1, r1, #7

update_calib:
	ldr	r0, [r8, #0x0]
	ldr	r3, [r8, #0x4]
	str	r3, [r5, r0]
	add	r8, r8, #8
	sub	r1, r1, #1
	cmp	r1, #0
	bgt	update_calib

	/* perform a force measurement. */
	ldr	r0, =0x800
	str	r0, [r5, #MMDC0_MPMUR0]
	ldr	r2, =MMDC1_MPMUR0
	str	r0, [r5, r2]

	/* clear SBS - unblock DDR accesses. */
	ldr	r0, [r5, #MMDC0_MADPCR0]
	bic	r0, r0, #(1 << 8)
	str	r0, [r5, #MMDC0_MADPCR0]

	mov	r0, #0x0
	str	r0, [r5, #MMDC0_MDSCR]
poll_conreq_clear_2:
	ldr	r0, [r5, #MMDC0_MDSCR]
	and	r0, r0, #(0x4 << 12)
	cmp	r0, #(0x4 << 12)
	beq	poll_conreq_clear_2

done:
	/* restore registers */

	ldmfd	sp!, {r4-r12}
	mov	pc, lr

	.type	mx6_do_ddr3_freq_change, #object
ENTRY(mx6_do_ddr_freq_change)
	.word	mx6_ddr3_freq_change
	.size	mx6_ddr3_freq_change, . - mx6_ddr3_freq_change
