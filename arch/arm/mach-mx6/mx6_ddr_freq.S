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
#include <mach/hardware.h>

	.macro   switch_to_528MHz

	/* DDR freq change to 528MHz */

	/* check if periph_clk_sel is already set */
	ldr     r0, [r6, #0x14]
	and     r0, r0, #0x2000000
	cmp     r0, #0x2000000
	beq	    set_ahb_podf_before_switch

	/* Step 1: Change periph_clk to be sourced from pll3_clk. */
	/* Ensure PLL3 is the source and set the divider to 1. */
	ldr     r0, [r6, #0x18]
	bic     r0, r0, #0x3000
	str     r0, [r6, #0x18]

	ldr     r0, [r6, #0x14]
	bic     r0, r0, #0x38000000
	str     r0, [r6, #0x14]

	/* Set the AHB dividers before the switch. */
	/* Don't change AXI clock divider. */
	/* Set the MMDC_DIV=1, AXI_DIV = 2, AHB_DIV=4 (need to maintain GPT divider). */
	ldr     r0, [r6, #0x14]
	ldr     r2, =0x3f1f00
	bic     r0, r0, r2
	orr     r0, r0, #0xd00
	orr     r0, r0, #0x10000
	str     r0, [r6, #0x14]

wait_div_update528:
	ldr     r0, [r6, #0x48]
	cmp     r0, #0
	bne     wait_div_update528

	/* Now switch periph_clk to pll3_main_clk. */
	ldr     r0, [r6, #0x14]
	orr     r0, r0, #0x2000000
	str     r0, [r6, #0x14]

periph_clk_switch3:
	ldr      r0, [r6, #0x48]
	cmp    r0, #0
	bne     periph_clk_switch3

	b       switch_pre_periph_clk_528

set_ahb_podf_before_switch:
	/* Set the AHB dividers before the switch. */
	/* Especially if the AHB is at 24MHz, divider
	  * would be at divide by 1 and clock
	  * would be too fast when switching to PLL3.
	  */
	/* Don't change AXI clock divider. */
	/* Set the MMDC_DIV=1, AXI_DIV = 2, AHB_DIV=4 (need to maintain GPT divider). */
	ldr     r0, [r6, #0x14]
	ldr     r2, =0x3f1f00
	bic     r0, r0, r2
	orr     r0, r0, #0xd00
	orr     r0, r0, #0x10000
	str     r0, [r6, #0x14]

wait_div_update528_1:
	ldr     r0, [r6, #0x48]
	cmp     r0, #0
	bne     wait_div_update528_1

switch_pre_periph_clk_528:

	/* Now switch pre_periph_clk to PLL2_528MHz. */
	ldr     r0, [r6, #0x18]
	bic     r0, r0, #0xC0000
	str     r0, [r6, #0x18]

	/* Now switch periph_clk back. */
	ldr     r0, [r6, #0x14]
	bic     r0, r0, #0x2000000
	str     r0, [r6, #0x14]

periph_clk_switch4:
	ldr     r0, [r6, #0x48]
	cmp     r0, #0
	bne     periph_clk_switch4

	ldr     r0, =ANATOP_BASE_ADDR
	add     r0, r0, #PERIPBASE_VIRT
	ldr	r1, [r0, #0x260]
	mov	r2, r1
	/*Is mx6q?*/
	and	r1, r1, #0xff0000
	cmp	r1, #0x630000
	bne	skip_gpt_workaround1
	/*Is mx6q TO1.0?*/
	and	r2, r2, #0xff
	cmp	r2, #0x0
	bne	skip_gpt_workaround1
	/* Change the GPT divider so that its at 6MHz. */
	ldr     r0, [r6, #0x1C]
	bic     r0, r0, #0x3F
	orr     r0, r0, #0xA
	str     r0, [r6, #0x1C]
skip_gpt_workaround1:
	.endm

	.macro   switch_to_400MHz

	/* check if periph_clk_sel is already set */
	ldr         r0, [r6, #0x14]
	and       r0, r0, #0x2000000
	cmp      r0, #0x2000000
	beq	     set_ahb_podf_before_switch1

	/* Step 1: Change periph_clk to be sourced from pll3_clk. */
	/* Ensure PLL3 is the source and set the divider to 1. */
	ldr     r0, [r6, #0x18]
	bic     r0, r0, #0x3000
	str     r0, [r6, #0x18]

	ldr     r0, [r6, #0x14]
	bic     r0, r0, #0x38000000
	str     r0, [r6, #0x14]

	/* Now switch periph_clk to pll3_main_clk. */
	ldr     r0, [r6, #0x14]
	orr     r0, r0, #0x2000000
	str     r0, [r6, #0x14]

periph_clk_switch5:
	ldr      r0, [r6, #0x48]
	cmp    r0, #0
	bne     periph_clk_switch5

	b       switch_pre_periph_clk_400

set_ahb_podf_before_switch1:
	/* Set the AHB dividers before the switch. */
	/* Especially if the AHB is at 24MHz, divider
	  * would be at divide by 1 and clock
	  * would be too fast when switching to PLL3.
	  */
	/* Don't change AXI clock divider. */
	/* Set the MMDC_DIV=1, AXI_DIV = 2, AHB_DIV=4 (need to maintain GPT divider). */
	ldr     r0, [r6, #0x14]
	ldr     r2, =0x3f1f00
	bic     r0, r0, r2
	orr     r0, r0, #0x900
	orr     r0, r0, #0x10000
	str     r0, [r6, #0x14]

wait_div_update400_1:
	ldr     r0, [r6, #0x48]
	cmp     r0, #0
	bne     wait_div_update400_1

switch_pre_periph_clk_400:

	/* Now switch pre_periph_clk to PFD_400MHz. */
	ldr     r0, [r6, #0x18]
	bic     r0, r0, #0xC0000
	orr     r0, r0, #0x40000
	str     r0, [r6, #0x18]

	/* Now switch periph_clk back. */
	ldr     r0, [r6, #0x14]
	bic     r0, r0, #0x2000000
	str     r0, [r6, #0x14]

periph_clk_switch6:
	ldr      r0, [r6, #0x48]
	cmp    r0, #0
	bne     periph_clk_switch6

	/* Change AHB divider so that we are at 400/3=133MHz. */
	/* Don't change AXI clock divider. */
	/* Set the MMDC_DIV=1, AXI_DIV=2, AHB_DIV=3 (need to maintain GPT divider). */
	ldr     r0, [r6, #0x14]
	ldr     r2, =0x3f1f00
	bic     r0, r0, r2
	orr     r0, r0, #0x900
	orr     r0, r0, #0x10000
	str     r0, [r6, #0x14]

wait_div_update400_2:
	ldr     r0, [r6, #0x48]
	cmp     r0, #0
	bne     wait_div_update400_2

	ldr     r0, =ANATOP_BASE_ADDR
	add     r0, r0, #PERIPBASE_VIRT
	ldr	r1, [r0, #0x260]
	mov	r2, r1
	/*Is mx6q?*/
	and	r1, r1, #0xff0000
	cmp	r1, #0x630000
	bne	skip_gpt_workaround2
	/*Is mx6q TO1.0?*/
	and	r2, r2, #0xff
	cmp	r2, #0x0
	bne	skip_gpt_workaround2
	/* Change the GPT divider so that its at 6MHz. */
	ldr     r0, [r6, #0x1C]
	bic     r0, r0, #0x3F
	orr     r0, r0, #0xA
	str     r0, [r6, #0x1C]
skip_gpt_workaround2:
	.endm

	.macro   switch_to_50MHz

	/* Set DDR to 50MHz. */
	/* check if periph_clk_sel is already set */
	ldr     r0, [r6, #0x14]
	and     r0, r0, #0x2000000
	cmp 	 r0, #0x2000000
	beq	 switch_pre_periph_clk_50

	/* Set the periph_clk to be sourced from PLL2_PFD_200M */
	/* Step 1: Change periph_clk to be sourced from pll3_clk. */
	/* Ensure PLL3 is the source and set the divider to 1. */
	ldr     r0, [r6, #0x18]
	bic     r0, r0, #0x3000
	str     r0, [r6, #0x18]

	ldr     r0, [r6, #0x14]
	bic     r0, r0, #0x38000000
	str     r0, [r6, #0x14]

	/* Now switch periph_clk to pll3_main_clk. */
	ldr     r0, [r6, #0x14]
	orr     r0, r0, #0x2000000
	str     r0, [r6, #0x14]

periph_clk_switch_50:
	ldr      r0, [r6, #0x48]
	cmp    r0, #0
	bne     periph_clk_switch_50

switch_pre_periph_clk_50:

	/* Now switch pre_periph_clk to PFD_200MHz. */
	ldr     r0, [r6, #0x18]
	orr     r0, r0, #0xC0000
	str     r0, [r6, #0x18]

	/* Set the MMDC_DIV=4, AXI_DIV = 4, AHB_DIV=8 (need to maintain GPT divider). */
	ldr     r0, [r6, #0x14]
	ldr     r2, =0x3f1f00
	bic     r0, r0, r2

	orr     r0, r0, #0x180000
	orr     r0, r0, #0x30000

	/* If changing AHB divider remember to change the IPGPER divider too below. */
	orr     r0, r0, #0x1d00
	str     r0, [r6, #0x14]

wait_div_update_50:
	ldr      r0, [r6, #0x48]
	cmp    r0, #0
	bne     wait_div_update_50

	/* Now switch periph_clk back. */
	ldr     r0, [r6, #0x14]
	bic     r0, r0, #0x2000000
	str     r0, [r6, #0x14]

periph_clk_switch2:
	ldr      r0, [r6, #0x48]
	cmp    r0, #0
	bne     periph_clk_switch2

	ldr     r0, =ANATOP_BASE_ADDR
	add     r0, r0, #PERIPBASE_VIRT
	ldr	r1, [r0, #0x260]
	mov	r2, r1
	/*Is mx6q?*/
	and	r1, r1, #0xff0000
	cmp	r1, #0x630000
	bne	skip_gpt_workaround3
	/*Is mx6q TO1.0?*/
	and	r2, r2, #0xff
	cmp	r2, #0x0
	bne	skip_gpt_workaround3
	/* Change the GPT divider so that its at 6MHz. */
	ldr     r0, [r6, #0x1C]
	bic     r0, r0, #0x3F
	orr     r0, r0, #0x1
	str     r0, [r6, #0x1C]
skip_gpt_workaround3:

	.endm

	.macro   switch_to_24MHz
	/* Change the freq now */
	/* Try setting DDR to 24MHz. */
	/* Source it from the periph_clk2 */
	/* Ensure the periph_clk2 is sourced from 24MHz
	and the divider is 1. */
	ldr     r0, [r6, #0x18]
	bic     r0, r0, #0x3000
	orr     r0, r0, #0x1000
	str     r0, [r6, #0x18]

	ldr     r0, [r6, #0x14]
	bic     r0, r0, #0x38000000
	str     r0, [r6, #0x14]

	/* Now switch periph_clk to 24MHz. */
	ldr     r0, [r6, #0x14]
	orr     r0, r0, #0x2000000
	str     r0, [r6, #0x14]

periph_clk_switch1:
	ldr      r0, [r6, #0x48]
	cmp    r0, #0
	bne     periph_clk_switch1

	/* Change all the dividers to 1. */
	ldr     r0, [r6, #0x14]
	ldr     r2, =0x3f1f00
	bic     r0, r0, r2
	orr	r0, r0, #0x100
	str     r0, [r6, #0x14]

	/* Wait for the divider to change. */
wait_div_update:
	ldr      r0, [r6, #0x48]
	cmp    r0, #0
	bne     wait_div_update

	ldr     r0, =ANATOP_BASE_ADDR
	add     r0, r0, #PERIPBASE_VIRT
	ldr	r1, [r0, #0x260]
	mov	r2, r1
	/*Is mx6q?*/
	and	r1, r1, #0xff0000
	cmp	r1, #0x630000
	bne	skip_gpt_workaround4
	/*Is mx6q TO1.0?*/
	and	r2, r2, #0xff
	cmp	r2, #0x0
	bne	skip_gpt_workaround4
	/* Change the GPT divider so that its at 6MHz. */
	ldr     r0, [r6, #0x1C]
	bic     r0, r0, #0x3F
	orr     r0, r0, #0x1
	str     r0, [r6, #0x1C]
skip_gpt_workaround4:

	.endm

/*
 *  mx6_ddr_freq_change
 *
 *  Idle the processor (eg, wait for interrupt).
 *  Make sure DDR is in self-refresh.
 *  IRQs are already disabled.
 */
ENTRY(mx6_ddr_freq_change)

	stmfd   sp!, {r4,r5,r6, r7, r8, r9, r10, r11}      @ Save registers

	ldr     r6, =CCM_BASE_ADDR
	add     r6, r6, #PERIPBASE_VIRT
	ldr     r5, =MMDC_P0_BASE_ADDR
	add     r5, r5, #PERIPBASE_VIRT
	ldr     r7, =MX6Q_IOMUXC_BASE_ADDR
	add     r7, r7, #PERIPBASE_VIRT

	mov     r4, r0				@save new freq requested
	mov     r8, r1				@save the ddr settings for the new rate
	mov     r9, r2				@save the mode DDR is currently in (DLL ON/OFF)
	mov     r11, r3			@save iomux offsets

ddr_freq_change:
	/* Make sure no TLB miss will occur when the DDR is in self refresh. */
	/* Invalidate TLB single entry to ensure that the address is not
	  * already in the TLB.
	  */

	adr   r10, ddr_freq_change               @Address in this function.

	ldr    r2, [r6]                        @ TLB will miss,
	                                         @CCM address will be loaded
	ldr    r2, [r5]                        @ TLB will miss,
	                                         @MMDC address will be loaded
	ldr    r2, [r7]                        @ TLB will miss,
	                                         @IOMUX will be loaded

	ldr    r2, [r8]                        @ Get the DDR settings.
	ldr    r2, [r10]                        @ TLB will miss
	ldr    r2, [r11]				@Get the IOMUX settings

	/* Make sure all the L1 & L2 buffers are drained, as
	  * we don't want any writes to the DDR when it is
	  * in self-refresh.
	  */
	/* Make sure the L1 buffers are drained. */
	dsb

#ifdef CONFIG_CACHE_L2X0
	/* Make sure the L2 buffers are drained.
	  * Sync operation on L2 drains the buffers.
	  */
	ldr      r0, =L2_BASE_ADDR
	add     r0, r0, #PERIPBASE_VIRT
	mov    r1, #0x0
	str      r1, [r0, #0x730]
#endif

     /* The second dsb might be needed to keep cache sync (device write)
       * ordering with the memory accesses before it.
       */
	dsb
	isb

	/* Disable automatic power saving. */
	ldr    r0, [r5, #0x404]
	orr    r0, r0, #0x01
	str    r0, [r5, #0x404]

	/* Disable MMDC power down timer. */
	/*MMDC0_MDPDC disable power down timer */
	ldr    r0, [r5, #0x4]
	bic    r0, r0, #0xff00
	str    r0, [r5, #0x4]

/* Delay for a while */
	ldr      r1, =4
delay1:
	ldr      r2, =0
cont1:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont1
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay1

	/* set CON_REG */
	ldr     r0, =0x8000
	str     r0, [r5, #0x1C]
poll_conreq_set_1:
	ldr     r0, [r5, #0x1C]
	and     r0, r0, #0x4000
	cmp	    r0, #0x4000
	bne     poll_conreq_set_1

	/*setmem /32 0x021b001c = 0x00008010 //Precharge all on cs0  */
	/*setmem /32 0x021b001c = 0x00008018 //Precharge all on cs1 */
	ldr     r0, =0x00008010
	str     r0, [r5, #0x1C]
	ldr     r0, =0x00008018
	str     r0, [r5, #0x1C]

	/* if requested frequency is greater than 300MHz go to DLL on mode */
	ldr     r1, =300000000
	cmp     r4, r1
	bge     dll_on_mode

dll_off_mode:

	/* if DLL is currently on, turn it off
	cmp    	r9, #1
	beq 	continue_dll_off_1

	/* setmem /32 0x021b001c = 0x00018031 //Rtt_NOM off + set dll off, cs 0 */
	/* setmem /32 0x021b001c = 0x00018039 //Rtt_NOM off + set dll off, cs 1 */
	ldr     r0, =0x00018031
	str     r0, [r5, #0x1C]

	ldr     r0, =0x00018039
	str     r0, [r5, #0x1C]

	ldr      r1, =10
delay1a:
	ldr      r2, =0
cont1a:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont1a
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay1a

continue_dll_off_1:

	/* set DVFS - enter self refresh mode */
	ldr     r0, [r5, #0x404]
	orr    r0, r0, #0x200000
	str     r0, [r5, #0x404]

	/* de-assert con_req */
	mov    r0, #0x0
	str    r0, [r5, #0x1C]

poll_dvfs_set_1:
	ldr     r0, [r5, #0x404]
	and     r0, r0, #0x2000000
	cmp	    r0, #0x2000000
	bne   	poll_dvfs_set_1

	ldr     r1, =24000000
	cmp     r4, r1
	beq     switch_freq_24

	switch_to_50MHz
	b continue_dll_off_2

switch_freq_24:
	switch_to_24MHz

continue_dll_off_2:

	/* set SBS - block ddr accesses */
	ldr     r0, [r5, #0x410]
	orr    	r0, r0, #0x100
	str     r0, [r5, #0x410]

	/* clear DVFS - exit from self refresh mode */
	ldr     r0, [r5, #0x404]
	bic     r0, r0, #0x200000
	str     r0, [r5, #0x404]

poll_dvfs_clear_1:
	ldr     r0, [r5, #0x404]
	and     r0, r0, #0x2000000
	cmp     r0, #0x2000000
	beq     poll_dvfs_clear_1

	/* if DLL was previously on, continue DLL off routine
	cmp    	r9, #1
	beq 	continue_dll_off_3

	/* setmem /32 0x021b001c = 0x00018031 //Rtt_NOM off + set dll off, cs 0 */
	/* setmem /32 0x021b001c = 0x00018039 //Rtt_NOM off + set dll off, cs 1 */
	ldr     r0, =0x00018031
	str     r0, [r5, #0x1C]

	ldr     r0, =0x00018039
	str     r0, [r5, #0x1C]

	/* setmem /32 0x021b001c = 0x04208030 //write mode reg MR0: CL=6, wr=6 ,cs 0  */
	/* setmem /32 0x021b001c = 0x04208038 //write mode reg MR0: CL=6, wr=6 ,cs 1  */
	ldr     r0, =0x08208030
	str     r0, [r5, #0x1C]

	ldr     r0, =0x08208038
	str     r0, [r5, #0x1C]

	/* setmem /32 0x021b001c = 0x02088032 //write mode reg MR2 , CWL=6 ,cs0 */
	/* setmem /32 0x021b001c = 0x0208803A //write mode reg MR2 , CWL=6 ,cs1 */
	ldr     r0, =0x00088032
	str     r0, [r5, #0x1C]

	ldr     r0, =0x0008803A
	str     r0, [r5, #0x1C]

	/* double refresh ????
	 ldr     r0, =0x00001800
	 str     r0, [r5, #0x20]*/

	/* delay for a while. */
	ldr      r1, =4
delay_1:
	ldr      r2, =0
cont_1:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont_1
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay_1

	/* MMDC0_MDCFG0 see spread sheet for timings, CAS=6 */
	ldr     r0, [r5, #0x0C]
	bic     r0, r0, #0xf
	orr     r0, r0, #0x3
	str     r0, [r5, #0x0C]

	/* MMDC0_MDCFG1 see spread sheet for timings, tCWL=6 */
	ldr     r0, [r5, #0x10]
	bic     r0, r0, #0x7
	orr     r0, r0, #0x4
	str     r0, [r5, #0x10]

	/* Enable bank interleaving, Address mirror on, WALAT = 0x1, RALAT = 0x2, DDR2_EN = 0 */
	/*setmem /32 0x021b0018 = 0x00091680 */
	ldr     r0, =0x00091680
	str     r0, [r5, #0x18]

	/* enable dqs pull down in the IOMUX. */
	/*
	setmem /32 0x020e05a8 = 0x00003030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0 - DSE=110
	setmem /32 0x020e05b0 = 0x00003030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1 - DSE=110
	setmem /32 0x020e0524 = 0x00003030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2 - DSE=110
	setmem /32 0x020e051c = 0x00003030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3 - DSE=110
	setmem /32 0x020e0518 = 0x00003030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4 - DSE=110
	setmem /32 0x020e050c = 0x00003030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5 - DSE=110
	setmem /32 0x020e05b8 = 0x00003030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6 - DSE=110
	setmem /32 0x020e05c0 = 0x00003030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7 - DSE=110
	*/
	ldr    r1, [r11]                   @size of array
	add   r11, r11, #8            @skip first eight bytes in array
	ldr     r2, =0x3028
update_iomux:
	ldr    r0, [r11, #0x0]           @ offset
	ldr    r3, [r7, r0]
	bic    r3, r3, r2		      @ Clear the DSE, PUE and PKE bits
	orr	 r3, r3, #0x3000          @ Enable the Pull downs and lower the drive strength.
	orr    r3, r3, #0x28
	str    r3, [r7, r0]
	add   r11, r11, #8
	sub   r1, r1, #1
	cmp  r1, #0
	bgt   update_iomux

	/*  ODT disabled */
	/* setmem /32 0x021b0818 = 0x0      // DDR_PHY_P0_MPODTCTRL */
	/* setmem /32 0x021b4818 = 0x0      // DDR_PHY_P1_MPODTCTRL */
	ldr    r0, =0x0
	ldr    r2, =0x818
	str    r0, [r5, r2]
	ldr    r2, =0x4818
	str    r0, [r5, r2]

	/* DQS gating disabled */
	/* setmem /32 0x021b083c = 0x233f033f */
	ldr    r2, =0x83c
	ldr    r0, [r5, r2]
	orr     r0, r0, #0x20000000
	str    r0, [r5, r2]

	ldr    r2, =0x483c
	ldr    r0, [r5, r2]
	orr     r0, r0, #0x20000000
	str    r0, [r5, r2]

	/* MMDC0_MAPSR adopt power down enable */
	/* setmem /32 0x021b0404 = 0x00011006 */
	ldr    r0, [r5, #0x404]
	bic    r0, r0, #0x01
	str    r0, [r5, #0x404]

	/* frc_msr + mu bypass*/
	ldr       r0, =0x00000060
	str       r0, [r5, #0x8b8]
	ldr       r2, =0x48b8
	str       r0, [r5, r2]
	ldr       r0, =0x00000460
	str       r0, [r5, #0x8b8]
	ldr       r2, =0x48b8
	str       r0, [r5, r2]
	ldr       r0, =0x00000c60
	str       r0, [r5, #0x8b8]
	ldr       r2, =0x48b8
	str       r0, [r5, r2]

continue_dll_off_3:

	/* clear SBS - unblock accesses to DDR */
	ldr     r0, [r5, #0x410]
	bic     r0, r0, #0x100
	str     r0, [r5, #0x410]

	mov		r0, #0x0
	str		r0, [r5, #0x1C]
poll_conreq_clear_1:
	ldr		r0, [r5, #0x1C]
	and	    r0, r0, #0x4000
	cmp	    r0, #0x4000
	beq     poll_conreq_clear_1

	b        done

dll_on_mode:
	/* assert DVFS - enter self refresh mode */
	ldr     r0, [r5, #0x404]
	orr    r0, r0, #0x200000
	str     r0, [r5, #0x404]

	/* de-assert CON_REQ */
	mov    r0, #0x0
	str    r0, [r5, #0x1C]

	/* poll DVFS ack */
poll_dvfs_set_2:
	ldr     r0, [r5, #0x404]
	and     r0, r0, #0x2000000
	cmp     r0, #0x2000000
	bne     poll_dvfs_set_2

	ldr    r1, =528000000
	cmp  r4, r1
	beq   switch_freq_528

	switch_to_400MHz

	b continue_dll_on

switch_freq_528:
	switch_to_528MHz

continue_dll_on:

	/* set SBS step-by-step mode */
	ldr     r0, [r5, #0x410]
	orr     r0, r0, #0x100
	str     r0, [r5, #0x410]

	/* clear DVFS - exit self refresh mode */
	ldr     r0, [r5, #0x404]
	bic     r0, r0, #0x200000
	str     r0, [r5, #0x404]

poll_dvfs_clear_2:
	ldr     r0, [r5, #0x404]
	and     r0, r0, #0x2000000
	cmp     r0, #0x2000000
	beq     poll_dvfs_clear_2

	/* if DLL is currently off, turn it back on */
	cmp    	r9, #0
	beq 	       update_calibration_only

	ldr     r0, =0xa5390003
	str     r0, [r5, #0x800]
	ldr     r2, =0x4800
	str     r0, [r5, r2]

	/* enable DQS gating */
	ldr    r2, =0x83c
	ldr    r0, [r5, r2]
	bic    r0, r0, #0x20000000
	str    r0, [r5, r2]

	ldr    r2, =0x483c
	ldr    r0, [r5, r2]
	bic    r0, r0, #0x20000000
	str    r0, [r5, r2]

	/* force measure */
	ldr       r0, =0x00000800
	str       r0, [r5, #0x8b8]
	ldr       r2, =0x48b8
	str       r0, [r5, r2]

	/* delay for while */
	ldr      r1, =4
delay5:
	ldr      r2, =0
cont5:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont5
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay5
	/* Disable dqs pull down in the IOMUX. */
	/*
	setmem /32 0x020e05a8 = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0 - DSE=110
	setmem /32 0x020e05b0 = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1 - DSE=110
	setmem /32 0x020e0524 = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2 - DSE=110
	setmem /32 0x020e051c = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3 - DSE=110
	setmem /32 0x020e0518 = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4 - DSE=110
	setmem /32 0x020e050c = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5 - DSE=110
	setmem /32 0x020e05b8 = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6 - DSE=110
	setmem /32 0x020e05c0 = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7 - DSE=110
	*/
	ldr    r1, [r11]                   @size of array
	add  r11, r11, #8             @skip first eight bytes in array
update_iomux1:
	ldr    r0, [r11, #0x0]       @ offset
	ldr    r3, [r11, #0x4]
	str    r3, [r7, r0]		  @Store the original IOMUX value read during boot
	add   r11, r11, #8
	sub   r1, r1, #1
	cmp  r1, #0
	bgt   update_iomux1

	/* config ESDCTL timings to 528MHz:
	@// setmem /32 0x021b000c = 0x555A7975		@// MMDC0_MDCFG0 see spread sheet for timings
	@//setmem /32 0x021b0010 = 0xFF538E64		      @// MMDC0_MDCFG1 see spread sheet for timings
	@//setmem /32 0x021b0014 = 0x01ff00db               @// MMDC0_MDCFG2 - tRRD - 4ck; tWTR - 4ck; tRTP - 4ck; tDLLK - 512ck
	@//setmem /32 0x021b0018 = 0x00081740             @// MMDC0_MDMISC, RALAT=0x5 (original value)
	*/
	ldr     r9, [r8]             @size of array
	add   r8, r8, #8             @skip first eight bytes in array
	ldr    r0, [r8, #0x0]       @ offset
	ldr    r3, [r8, #0x4]       @ value
	str    r3, [r5, r0]
	add   r8, r8, #8

	ldr    r0, [r8, #0x0]       @ offset
	ldr    r3, [r8, #0x4]       @ value
	str    r3, [r5, r0]
	add   r8, r8, #8

	/* update MISC register: WALAT, RALAT */
	ldr       r0, =0x00081740
	str       r0, [r5, #0x18]

	/*configure ddr devices to dll on, odt
	@//setmem /32 0x021b001c = 0x00428031
	@//setmem /32 0x021b001c = 0x00428039
	*/
	ldr      r0, =0x00028031
	str      r0, [r5, #0x1C]

	ldr      r0, =0x00028039
	str      r0, [r5, #0x1C]

	/* delay for while  */
	ldr      r1, =4
delay7:
	ldr      r2, =0
cont7:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont7
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay7

	/* reset dll
	@// setmem /32 0x021b001c = 0x09208030
	@// setmem /32 0x021b001c = 0x09208038
	*/
	ldr       r0, =0x09208030
	str       r0, [r5, #0x1C]

	ldr       r0, =0x09208038
	str       r0, [r5, #0x1C]

	/* delay for while  */
	ldr      r1, =100
delay8:
	ldr      r2, =0
cont8:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont8
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay8

	/* tcwl=6:
	@//setmem /32 0x021b001c = 0x04088032
	@//setmem /32 0x021b001c = 0x0408803a
	*/
	/* MR2 -  CS0 */
	ldr    r0, [r8, #0x0]       @ offset
	ldr    r3, [r8, #0x4]       @ value
	str    r3, [r5, r0]
	add   r8, r8, #8

	/*MR2 - CS1 */
	ldr    r0, [r8, #0x0]       @ offset
	ldr    r3, [r8, #0x4]       @ value
	str    r3, [r5, r0]
	add   r8, r8, #8

	ldr      r0, =0x00428031
	str      r0, [r5, #0x1C]

	ldr      r0, =0x00428039
	str      r0, [r5, #0x1C]

	/* tcl=8
	@// setmem /32 0x021b001c = 0x08408030
	@// setmem /32 0x021b001c = 0x08408038
	*/
	/* MR1 -  CS0 */
	ldr    r0, [r8, #0x0]       @ offset
	ldr    r3, [r8, #0x4]       @ value
	str    r3, [r5, r0]
	add   r8, r8, #8

	/*MR1 - CS1 */
	ldr    r0, [r8, #0x0]       @ offset
	ldr    r3, [r8, #0x4]       @ value
	str    r3, [r5, r0]
	add   r8, r8, #8

	/* issue a zq command
	@// setmem /32 0x021b001c = 0x04000040
	@// setmem /32 0x021b001c = 0x04000048
	 */
	ldr       r0, =0x04008040
	str       r0, [r5, #0x1C]

	ldr       r0, =0x04008048
	str       r0, [r5, #0x1C]

	/* ESDCTL ODT enable
	@//setmem /32 0x021b0818 = 0x00022225      @// DDR_PHY_P0_MPODTCTRL
	@//setmem /32 0x021b4818 = 0x00022225      @// DDR_PHY_P1_MPODTCTRL
	*/
	ldr    r0, [r8, #0x0]       @ offset
	ldr    r3, [r8, #0x4]       @ value
	str    r3, [r5, r0]
	add   r8, r8, #8

	ldr       r2, =0x4818
	str       r0, [r5, r2]

	/* delay for while */
	ldr      r1, =40
delay15:
	ldr      r2, =0
cont15:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont15
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay15

	/* MMDC0_MAPSR adopt power down enable */
	/* setmem /32 0x021b0404 = 0x00011006 */
	ldr    r0, [r5, #0x404]
	bic    r0, r0, #0x01
	str    r0, [r5, #0x404]

	/* Enable MMDC power down timer. */
	ldr    r0, [r5, #0x4]
	orr    r0, r0, #0x5500
	str    r0, [r5, #0x4]

	b 	update_calibration

update_calibration_only:
	ldr     r1, [r8]                  @ size of array
	sub    r1, r1, #7           @ first 7 entries are not related to calibration
	add   r8, r8, #64	      @ Skip the first 7 entries that are needed only when DLL was OFF + count entry.
	b      update_calib

update_calibration:
	/* Write the new calibration values. */
	mov   r1, r9                 @ size of array
	sub    r1, r1, #7           @ first 7 entries are not related to calibration

update_calib:
	ldr    r0, [r8, #0x0]       @ offset
	ldr    r3, [r8, #0x4]       @ value
	str    r3, [r5, r0]
	add   r8, r8, #8
	sub   r1, r1, #1
	cmp  r1, #0
	bgt   update_calib

	/* Perform a force measurement. */
	ldr     r0, =0x800
	str     r0, [r5, #0x8B8]
	ldr      r2, =0x48B8
	str     r0, [r5, r2]

	/* clear SBS - unblock DDR accesses */
	ldr     r0, [r5, #0x410]
	bic     r0, r0, #0x100
	str     r0, [r5, #0x410]

	mov		r0, #0x0
	str		r0, [r5, #0x1C]
poll_conreq_clear_2:
	ldr		r0, [r5, #0x1C]
	and	    r0, r0, #0x4000
	cmp	    r0, #0x4000
	beq     poll_conreq_clear_2

done:

	/* Restore registers */

	ldmfd sp!, {r4,r5,r6, r7, r8, r9, r10, r11}

	mov     pc, lr

	.type   mx6_do_ddr_freq_change, #object
ENTRY(mx6_do_ddr_freq_change)
	.word   mx6_ddr_freq_change
	.size    mx6_ddr_freq_change, . - mx6_ddr_freq_change
