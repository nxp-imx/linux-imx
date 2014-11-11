/*
 * Copyright 2011-2014 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/asm-offsets.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/smp_scu.h>

#define SCU_CPU_STATUS		0x08

	.section ".text.head", "ax"
.extern imx_scu_base

#ifdef CONFIG_SMP
diag_reg_offset:
	.word	g_diag_reg - .

	.macro	set_diag_reg
	adr	r0, diag_reg_offset
	ldr	r1, [r0]
	add	r1, r1, r0		@ r1 = physical &g_diag_reg
	ldr	r0, [r1]
	mcr	p15, 0, r0, c15, c0, 1	@ write diagnostic register
	.endm

ENTRY(v7_secondary_startup)
	bl	v7_invalidate_l1
	set_diag_reg

	/* Set the CPU status in SCU CPU status register. */
	mrc	p15, 0, r0, c0, c0, 5
	and	r0, r0, #3
	mrc	p15, 4, r1, c15, c0, 0
	ldr	r2, =SCU_CPU_STATUS
	orr	r2, r2, r0
	ldr	r0, =SCU_PM_NORMAL
	strb	r0, [r1, r2]

	b	secondary_startup
ENDPROC(v7_secondary_startup)
#endif

#ifdef CONFIG_ARM_CPU_SUSPEND
/*
 * The following code must assume it is running from physical address
 * where absolute virtual addresses to the data section have to be
 * turned into relative ones.
 */

#ifdef CONFIG_CACHE_L2X0
	.macro	pl310_resume
	adr	r0, l2x0_saved_regs_offset
	ldr	r2, [r0]
	add	r2, r2, r0
	ldr	r0, [r2, #L2X0_R_PHY_BASE]	@ get physical base of l2x0
	ldr	r1, [r2, #L2X0_R_AUX_CTRL]	@ get aux_ctrl value
	str	r1, [r0, #L2X0_AUX_CTRL]	@ restore aux_ctrl
	ldr	r1, [r2, #L2X0_R_TAG_LATENCY]		@ get tag latency value
	str	r1, [r0, #L2X0_TAG_LATENCY_CTRL]	@ restore tag latency
	ldr	r1, [r2, #L2X0_R_DATA_LATENCY]		@ get data latency value
	str	r1, [r0, #L2X0_DATA_LATENCY_CTRL]	@ restore data latency
	mov	r1, #0x1
	str	r1, [r0, #L2X0_CTRL]		@ re-enable L2
	.endm

l2x0_saved_regs_offset:
	.word	l2x0_saved_regs - .

#else
	.macro	pl310_resume
	.endm
#endif
#endif
