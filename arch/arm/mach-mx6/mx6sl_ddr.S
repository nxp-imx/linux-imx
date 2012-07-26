/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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


	 .macro    mx6sl_switch_to_24MHz

      /* Set MMDC clock to be sourced from PLL3. */
      /* Ensure first periph2_clk2 is sourced from PLL3. */
      /* Set the PERIPH2_CLK2_PODF to divide by 2. */
     ldr    r6, [r2, #0x14]
     bic    r6, r6, #0x7
     orr    r6, r6, #0x1
     str     r6, [r2, #0x14]

     /* Select PLL3 to source MMDC. */
     ldr    r6, [r2, #0x18]
     bic    r6, r6, #0x100000
     str    r6, [r2, #0x18]

     /* Swtich periph2_clk_sel to run from PLL3. */
      ldr     r6, [r2, #0x14]
      orr     r6, r6, #0x4000000
      str     r6, [r2, #0x14]

periph2_clk_switch1:
     ldr     r6, [r2, #0x48]
     cmp     r6, #0
     bne     periph2_clk_switch1

    /* Need to clock gate the 528 PFDs before
      * powering down PLL2.
      * Only the PLL2_PFD2_400M should be ON
      * as it feeds the MMDC
      */
     ldr     r6, [r3, #0x100]
     orr    r6, r6, #0x800000
     str     r6, [r3, #0x100]

    /* Set PLL2 to bypass state. We should be here
      *only if MMDC is not sourced from PLL2.*/
     ldr    r6, [r3, #0x30]
     orr    r6, r6, #0x10000
     str     r6, [r3, #0x30]

     ldr    r6, [r3, #0x30]
     orr    r6, r6, #0x1000
     str     r6, [r3, #0x30]

    /* Ensure pre_periph2_clk_mux is set to pll2 */
     ldr     r6, [r2, #0x18]
     bic     r6, r6, #0x600000
     str     r6, [r2, #0x18]

     /* Set MMDC clock to be sourced from the bypassed PLL2. */
     ldr    r6, [r2, #0x14]
     bic    r6, r6, #0x4000000
     str    r6, [r2, #0x14]

periph2_clk_switch2:
     ldr     r6, [r2, #0x48]
     cmp     r6, #0
     bne     periph2_clk_switch2

    /* Now move MMDC back to periph2_clk2 source.
      * after selecting PLL2 as the option.
      */
     /* Select PLL2 as the source. */
     ldr    r6, [r2, #0x18]
     orr    r6, r6, #0x100000
     str    r6, [r2, #0x18]

    /* set periph2_clk2_podf to divide by 1. */
     ldr    r6, [r2, #0x14]
     bic    r6, r6, #0x7
     str     r6, [r2, #0x14]

     /* Now move periph2_clk to periph2_clk2 source */
     ldr    r6, [r2, #0x14]
     orr    r6, r6, #0x4000000
     str    r6, [r2, #0x14]

periph2_clk_switch3:
     ldr     r6, [r2, #0x48]
     cmp     r6, #0
     bne     periph2_clk_switch3

	.endm

	  .macro	ddr_switch_400MHz

    /* Set MMDC divider first, in case PLL3 is at 480MHz. */
      ldr     r6, [r3, #0x10]
      and    r6, r6, #0x10000
      cmp   r6, #0x10000
      beq    pll3_in_bypass
      /* Set MMDC divder to divide by 2. */
      ldr    r6, [r2, #0x14]
      bic    r6, r6, #0x38
      orr    r6, r6, #0x8
      str    r6, [r2, #0x14]

mmdc_podf:
     ldr     r6, [r2, #0x48]
     cmp     r6, #0
     bne     mmdc_podf

pll3_in_bypass:

     /* Ensure that MMDC is sourced from PLL2 mux first. */
     ldr     r6, [r2, #0x14]
     bic     r6, r6, #0x4000000
     str     r6, [r2, #0x14]

periph2_clk_switch4:
     ldr     r6, [r2, #0x48]
     cmp     r6, #0
     bne     periph2_clk_switch4

     /* Now ensure periph2_clk2_sel mux is set to PLL3 */
     ldr     r6, [r2, #0x18]
     bic     r6, r6, #0x100000
     str     r6, [r2, #0x18]

      /* Now switch MMDC to PLL3. */
     ldr     r6, [r2, #0x14]
     orr     r6, r6, #0x4000000
     str     r6, [r2, #0x14]

periph2_clk_switch5:
     ldr     r6, [r2, #0x48]
     cmp     r6, #0
     bne     periph2_clk_switch5

     /* Now power up PLL2 and unbypass it. */
     ldr    r6, [r3, #0x30]
     bic    r6, r6, #0x1000
     str     r6, [r3, #0x30]

     /* Make sure PLL2 has locked.*/
wait_for_pll_lock:
     ldr    r6, [r3, #0x30]
     and   r6, r6, #0x80000000
     cmp  r6, #0x80000000
     bne   wait_for_pll_lock

      ldr    r6, [r3, #0x30]
     bic    r6, r6, #0x10000
     str     r6, [r3, #0x30]

    /* Need to enable the 528 PFDs after
      * powering up PLL2.
      * Only the PLL2_PFD2_400M should be ON
      * as it feeds the MMDC. Rest should have
      * been managed by clock code.
      */
     ldr     r6, [r3, #0x100]
     bic    r6, r6, #0x800000
     str     r6, [r3, #0x100]

    /* Now switch MMDC clk back to pll2_mux option. */
    /* Ensure pre_periph2_clk2 is set to pll2_pfd_400M */
     ldr     r6, [r2, #0x18]
     bic     r6, r6, #0x600000
     orr     r6, r6, #0x200000
     str     r6, [r2, #0x18]

     ldr     r6, [r2, #0x14]
     bic     r6, r6, #0x4000000
     str     r6, [r2, #0x14]

periph2_clk_switch6:
     ldr     r6, [r2, #0x48]
     cmp     r6, #0
     bne     periph2_clk_switch6

     /* Now set the MMDC PODF back to 1.*/

      ldr    r6, [r2, #0x14]
      bic    r6, r6, #0x38
      str    r6, [r2, #0x14]

mmdc_podf1:
     ldr     r6, [r2, #0x48]
     cmp     r6, #0
     bne     mmdc_podf1

    .endm

/*
 *  mx6sl_ddr_iram
 *
 *  Idle the processor (eg, wait for interrupt).
 *  Make sure DDR is in self-refresh.
 *  IRQs are already disabled.
 * r0 : DDR freq.
 */
ENTRY(mx6sl_ddr_iram)

    push {r4, r5, r6, r7, r8, r9, r10 }

mx6sl_ddr_freq_change:
    ldr     r3, =ANATOP_BASE_ADDR
    add   r3, r3, #PERIPBASE_VIRT

    ldr     r2, =CCM_BASE_ADDR
    add   r2, r2, #PERIPBASE_VIRT

    ldr     r8, =MMDC_P0_BASE_ADDR
    add   r8, r8, #PERIPBASE_VIRT

    /* Prime all TLB entries. */
    adr   r7, mx6sl_ddr_freq_change               @Address in this function.

    ldr     r6, [r7]
    ldr     r6, [r8]
    ldr     r6, [r3]
    ldr     r6, [r2]

     dsb
     isb

     /* Disable Automatic power savings. */
     ldr    r6, [r8, #0x404]
     orr    r6, r6, #0x01
     str    r6, [r8, #0x404]

     /* Disable MMDC power down timer. */
     /*MMDC0_MDPDC disable power down timer */
     ldr    r6, [r8, #0x4]
     bic    r6, r6, #0xff00
     str    r6, [r8, #0x4]

     /* Delay for a while */
     ldr      r1, =10
delay1:
     ldr      r7, =0
cont1:
     ldr      r6, [r8, r7]
     add     r7, r7, #4
     cmp    r7, #16
     bne     cont1
     sub     r1, r1, #1
     cmp    r1, #0
     bgt     delay1

     /* Make the DDR explicitly enter self-refresh. */
     ldr     r6, [r8, #0x404]
     orr    r6, r6, #0x200000
     str     r6, [r8, #0x404]

poll_dvfs_set_1:
     ldr     r6, [r8, #0x404]
     and     r6, r6, #0x2000000
     cmp    r6, #0x2000000
     bne     poll_dvfs_set_1

     /* set SBS step-by-step mode */
     ldr     r6, [r8, #0x410]
     orr     r6, r6, #0x100
     str     r6, [r8, #0x410]

     ldr     r1, =24000000
     cmp    r0, r1
     beq     set_to_24MHz

     ddr_switch_400MHz
     b    done

set_to_24MHz:
     mx6sl_switch_to_24MHz

done:
     /* clear DVFS - exit from self refresh mode */
     ldr     r6, [r8, #0x404]
     bic     r6, r6, #0x200000
     str     r6, [r8, #0x404]

poll_dvfs_clear_1:
     ldr     r6, [r8, #0x404]
     and     r6, r6, #0x2000000
     cmp     r6, #0x2000000
     beq     poll_dvfs_clear_1

     /* Enable Automatic power savings. */
     ldr    r6, [r8, #0x404]
     bic    r6, r6, #0x01
     str    r6, [r8, #0x404]

     ldr     r1, =24000000
     cmp    r0, r1
     beq     skip_power_down

     /* Enable MMDC power down timer. */
     ldr    r6, [r8, #0x4]
     orr    r6, r6, #0x5500
     str    r6, [r8, #0x4]

skip_power_down:
     /* clear SBS - unblock DDR accesses */
     ldr     r6, [r8, #0x410]
     bic     r6, r6, #0x100
     str     r6, [r8, #0x410]

    pop {r4,r5, r6, r7, r8, r9, r10}

    /* Restore registers */
    mov     pc, lr

    .type   mx6sl_ddr_do_iram, #object
ENTRY(mx6sl_ddr_do_iram)
    .word   mx6sl_ddr_iram
    .size    mx6sl_ddr_iram, . - mx6sl_ddr_iram
