/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file plat-mxc/entry-pm.S
 *
 * @brief This file contains common pm entry .
 *
 * @ingroup MXC_PM
 */

#include <asm/assembler.h>
#include <asm/ptrace.h>
#include <asm/memory.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/asm-offsets.h>
#include <asm/thread_info.h>
#include <asm/proc-fns.h>
#include <asm/vfpmacros.h>

#define WAIT_MODE               111
#define DOZE_MODE               112
#define STOP_MODE               113
#define DSM_MODE                114

#define PM_XLOAD_SIZE		0x04
#define PM_XLOAD_ENTRY		0x08
#define PM_XLOAD_SUSPEND_MODE	0x0C
#define PM_XLOAD_CORE_SP	0x10

#define PROCINFO_PROC_FNS	36
#define PROC_FIN_FN		12
#define PROC_IDLE_FN		20

#ifdef CONFIG_FIQ
#define ARM_CONTEXT_SIZE 12
#else
#define ARM_CONTEXT_SIZE 8
#endif

#ifdef CONFIG_PM_VERBOSE
resume_str:
	.string "Resume from DSM..."
	.size resume_str, . - resume_str

.macro show_resume_str
	ldr r0, =resume_str
	bl printk
.endm

#else
.macro show_resume_str
.endm
#endif

	.data
	.align 3
arm_core_context:
	.rept ARM_CONTEXT_SIZE
	.long 0
	.endr

#ifdef CONFIG_VFP
	.text
	.align 5
arm_vfp_save:
	mov ip, sp
	stmdb sp!, {r0-r8, fp, ip, lr, pc}
	sub fp, ip, #4
	mov r1, #THREAD_SIZE
	sub r1, r1, #1
	bic r0, sp, r1
	ldr r8, [r0, #TI_CPU]
	add r4, r0, #TI_VFPSTATE

	ldr r3, =last_VFP_context
	VFPFMRX	r2, FPEXC
	tst r2, #FPEXC_EN
	bne 1f

	ldr r4, [r3, r8, lsl #2]
	cmp r4, #0
	beq dead_vfp
1:
	bic r1, r2, #FPEXC_EN
	VFPFMXR FPEXC, r1
	/*TODO: SMP */
	VFPFSTMIA r4, r1
	VFPFMRX	r5, FPSCR
	tst r2, #FPEXC_EX
	VFPFMRX r6, FPINST, NE
	tstne r2, #FPEXC_FP2V
	VFPFMRX r7, FPINST2, NE
	stmia r4, {r2, r5, r6, r7}

	mov r1, #0
	str r1, [r3, r8, lsl #2]
dead_vfp:
	ldmia sp, {r0-r8, fp, sp, pc}
#endif
/*
 * The function just be called in this file
 * Current r0 ~r4 are not saved.
 * Otherwise, the working registers should be saved
 */
	.text
	.align 5
arm_core_save:
	mov ip, sp
	stmdb sp!, {r8, r9, sl, fp, ip, lr, pc}
	sub fp, ip, #4
	ldr r0, =arm_core_context
	mov r3, r0
	/* SVC mode */
	mrs r1, spsr	@Save spsr
	mrs r2, cpsr	@Save cpsr
	stmia r0!, {r1, r2}
	/* Abort mode */
	msr cpsr_c, #PSR_F_BIT | PSR_I_BIT | ABT_MODE
	stmia r0!, {sp}		@Save stack pointer for abort mode
	msr cpsr_c, #PSR_F_BIT | PSR_I_BIT | UND_MODE
	stmia r0!, {sp}		@Save stack pointer for undefine mode
	msr cpsr_c, #PSR_F_BIT | PSR_I_BIT | IRQ_MODE
	stmia r0!, {sp}		@Save stack pointer for irq mode
#ifdef CONFIG_FIQ
	msr cpsr_c, #PSR_F_BIT | PSR_I_BIT | FIQ_MODE
	/*Save general register and sp for fiq mode*/
	stmia r0!, {r8-r9, sl, fp, ip, sp}
#endif
	ldr r0, [r3, #4]
	msr cpsr_c, r0
	ldmia sp, {r8-r9, sl, fp, sp, pc}

/*
 * The function just be called in this file
 * Current r0 ~r4 are not saved.
 * Otherwise, the working registers should be saved
 */
arm_core_restore:
	mov ip, sp
	stmdb sp!, {fp, ip, lr, pc}
	sub fp, ip, #4
	ldr r0, =arm_core_context
	mov r3, r0
	/* SVC mode */
	add r0, r0, #8		@skip svc mode
	/* Abort mode */
	msr cpsr_c, #PSR_F_BIT | PSR_I_BIT | ABT_MODE
	ldmia r0!, {sp}		@restore stack pointer for abort mode
	msr cpsr_c, #PSR_F_BIT | PSR_I_BIT | UND_MODE
	ldmia r0!, {sp}		@restore stack pointer for undefine mode
	msr cpsr_c, #PSR_F_BIT | PSR_I_BIT | IRQ_MODE
	ldmia r0!, {sp}		@restore stack pointer for irq mode
#ifdef CONFIG_FIQ
	msr cpsr_c, #PSR_F_BIT | PSR_I_BIT | FIQ_MODE
	/*Save general register and sp for fiq mode*/
	ldmia r0!, {r8-r9, sl, fp, ip, sp}
#endif
	ldmia r3!, {r1, r2}
	msr cpsr, r2		@restore cpsr
	msr spsr, r1		@restore spsr
	ldmia sp, {fp, sp, pc}

mxc_cp15_context:
	.rept 16
	.long 0
	.endr

	.align 5
mxc_cp15_restore:
	/* Physical address */
	adr r0, mxc_cp15_context
	ldmia r0, {r1-r9}
#ifndef CONFIG_PM_DEBUG
	@Add dynamic check to skip this block when debug
	sub lr, lr, #PHYS_OFFSET
	add lr, lr, #PAGE_OFFSET
#endif
	mcr p15, 0, r3, c1, c0, 2	@CP Access Register
	mcr p15, 0, r2, c1, c0, 1	@Aux Control register

#ifndef CONFIG_PM_DEBUG
	mcr p15, 0, r0, c7, c5, 6	@flush BTAC/BTB
	mcr p15, 0, r0, c7, c7, 0	@invalidate both caches
	mcr p15, 0, r0, c8, c7, 0	@Inval TLBs
#endif

	mcr p15, 0, r4, c13, c0, 0	@PID
	mcr p15, 0, r5, c13, c0, 1	@Context ID

	mcr p15, 0, r6, c3, c0, 0	@Domain Access Register
	mcr p15, 0, r7, c2, c0, 0	@TTB0
	mcr p15, 0, r8, c2, c0, 1	@TTB1
	mcr p15, 0, r9, c2, c0, 2	@TTBC

	mcr p15, 0, r1, c1, c0, 0	@Control Register
	/* mcu enabled */
	mrc p15, 0, r0, c2, c0, 0

	mov pc, lr
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

mxc_cp15_save:
	mov ip, sp
	stmdb sp!, {r8-r9, fp, ip, lr, pc}
	sub fp, ip, #4
	ldr r0, =mxc_cp15_context
/* System Control Registers */
	mrc p15, 0, r1, c1, c0, 0	@Control Register
	mrc p15, 0, r2, c1, c0, 1	@Aux Control Register
	mrc p15, 0, r3, c1, c0, 2	@CP access Register

/* Memory management Registers */
	mrc p15, 0, r4, c13, c0, 0	@PID
	mrc p15, 0, r5, c13, c0, 1	@Context ID

	mrc p15, 0, r6, c3, c0, 0	@Domain Access Register

	mrc p15, 0, r7, c2, c0, 0	@TTB0
	mrc p15, 0, r8, c2, c0, 1	@TTB1
	mrc p15, 0, r9, c2, c0, 2	@TTBC
	stmia r0, {r1-r9}
	ldmia sp, {r8, r9, fp, sp, pc}

/*
 * int __mxc_pm_arch_entry(u32 entry, u32 size)
 */
	.align 5
	.globl mxc_pm_arch_entry
mxc_pm_arch_entry:
	mov ip, sp
	stmdb sp!, {r4-r9, sl, fp, ip, lr, pc}
	sub fp, ip, #4
	sub sp, sp, #4
	mov r8, r0	@save entry
	mov r9, r1	@save entry size
#ifdef CONFIG_VFP
	bl arm_vfp_save
#endif
	/* r0 ~r3, ip is dirty*/
	bl arm_core_save	@save arm context
	bl mxc_cp15_save
	mov r0, sp
	mov r1, r8	@restore entry
	mov r2, r9	@restore entry size
	bl __mxc_pm_xload_setup
1:	bl cpu_v6_proc_fin
	bl cpu_v6_do_idle
	nop
	nop
	nop
	nop
__mxc_pm_arch_leave:
	adr r0, __mxc_pm_xload_info
	ldr sp, [r0, #PM_XLOAD_CORE_SP]

#ifndef CONFIG_PM_DEBUG
	sub sp, sp, #PAGE_OFFSET
	add sp, sp, #PHYS_OFFSET
#endif
	bl mxc_cp15_restore
#ifndef CONFIG_PM_DEBUG
	sub sp, sp, #PHYS_OFFSET
	add sp, sp, #PAGE_OFFSET
#endif
	show_resume_str
	bl arm_core_restore
	ldmib sp, {r4-r9, sl, fp, sp, pc}

__mxc_pm_xload_info:
	adr pc, __mxc_pm_xload_entry		@Jump instruction
	.long	__mxc_pm_xload_end - __mxc_pm_xload_info	@loader size
	.long 	(__mxc_pm_arch_leave - PAGE_OFFSET + PHYS_OFFSET) @resume entry
	.long   0 		@suspend state
	.long   0		@Core Stack pointer
__mxc_pm_xload_entry:
	adr r0, __mxc_pm_xload_info
	ldr pc, [r0, #PM_XLOAD_ENTRY]
__mxc_pm_xload_end:

/*
 * __mxc_pm_xload_setup(u32 sp, u32 entry, u32 size)
 * r0~r6 is dirty
 */
__mxc_pm_xload_setup:
	ldr r3, =__mxc_pm_xload_info
	str r0, [r3, #PM_XLOAD_CORE_SP]
	ldr r4, [r3, #PM_XLOAD_SIZE]
	cmp r2, r4
	blo 2f
1:	ldr r5, [r3], #4
	str r5, [r1], #4
	subs r4, r4, #4
	bhi 1b
	b 3f
2:	str r3, [r1]
3:	mov pc, lr
