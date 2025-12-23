/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025 Google LLC
 * Author: Fuad Tabba <tabba@google.com>
 * Author: Will Deacon <will@kernel.org>
 */

#include "pkvm.h"

	.text
	.balign 16
	.globl	pvm_entry_start
pvm_entry_start:
	mov	x30, x0

	ldr	x0, [x30, #PVM_BOOT_ARGS_OFF_X0]
	ldr	x1, [x30, #PVM_BOOT_ARGS_OFF_X1]
	ldr	x2, [x30, #PVM_BOOT_ARGS_OFF_X2]
	ldr	x3, [x30, #PVM_BOOT_ARGS_OFF_X3]
	ldr	x4, [x30, #PVM_BOOT_ARGS_OFF_X4]
	ldr	x5, [x30, #PVM_BOOT_ARGS_OFF_X5]
	ldr	x6, [x30, #PVM_BOOT_ARGS_OFF_X6]
	ldr	x7, [x30, #PVM_BOOT_ARGS_OFF_X7]

	ldr	x8, [x30, #PVM_BOOT_ARGS_OFF_JUMP_TGT]

	ldr	x9, [x30, #PVM_BOOT_ARGS_OFF_CPACR_EL1]
	msr	cpacr_el1, x9

	ldr	x9, [x30, #PVM_BOOT_ARGS_OFF_TCR_EL1]
	msr	tcr_el1, x9

	ldr	x9, [x30, #PVM_BOOT_ARGS_OFF_TTBR0_EL1]
	msr	ttbr0_el1, x9

	ldr	x9, [x30, #PVM_BOOT_ARGS_OFF_VBAR_EL1]
	msr	vbar_el1, x9

	ldr	x9, [x30, #PVM_BOOT_ARGS_OFF_TPIDR_EL1]
	msr	tpidr_el1, x9

	ldr	x9, [x30, #PVM_BOOT_ARGS_OFF_SP_EL1]
	mov	sp, x9

	isb
	ldr	x9, [x30, #PVM_BOOT_ARGS_OFF_SCTLR_EL1]
	msr	sctlr_el1, x9
	isb

	br	x8
	.globl	pvm_entry_end
pvm_entry_end:
