/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025 Google LLC
 * Author: Fuad Tabba <tabba@google.com>
 * Author: Will Deacon <will@kernel.org>
 */

#ifndef SELFTEST_KVM_ARM64_PKVM_H
#define SELFTEST_KVM_ARM64_PKVM_H

#ifndef __ASSEMBLY__
struct pvm_boot_args {
	u64	jump_tgt;

	u64	regs[8];
	u64	cpacr_el1;
	u64	sctlr_el1;
	u64	tcr_el1;
	u64	ttbr0_el1;
	u64	vbar_el1;

	u64	tpidr_el1;
	u64	sp_el1;
};
#endif

#define PVM_BOOT_ARGS_OFF_JUMP_TGT	0
#define PVM_BOOT_ARGS_OFF_X0		(PVM_BOOT_ARGS_OFF_JUMP_TGT + 8)
#define PVM_BOOT_ARGS_OFF_X1		(PVM_BOOT_ARGS_OFF_X0 + 8)
#define PVM_BOOT_ARGS_OFF_X2		(PVM_BOOT_ARGS_OFF_X1 + 8)
#define PVM_BOOT_ARGS_OFF_X3		(PVM_BOOT_ARGS_OFF_X2 + 8)
#define PVM_BOOT_ARGS_OFF_X4		(PVM_BOOT_ARGS_OFF_X3 + 8)
#define PVM_BOOT_ARGS_OFF_X5		(PVM_BOOT_ARGS_OFF_X4 + 8)
#define PVM_BOOT_ARGS_OFF_X6		(PVM_BOOT_ARGS_OFF_X5 + 8)
#define PVM_BOOT_ARGS_OFF_X7		(PVM_BOOT_ARGS_OFF_X6 + 8)
#define PVM_BOOT_ARGS_OFF_CPACR_EL1	(PVM_BOOT_ARGS_OFF_X7 + 8)
#define PVM_BOOT_ARGS_OFF_SCTLR_EL1	(PVM_BOOT_ARGS_OFF_CPACR_EL1 + 8)
#define PVM_BOOT_ARGS_OFF_TCR_EL1	(PVM_BOOT_ARGS_OFF_SCTLR_EL1 + 8)
#define PVM_BOOT_ARGS_OFF_TTBR0_EL1	(PVM_BOOT_ARGS_OFF_TCR_EL1 + 8)
#define PVM_BOOT_ARGS_OFF_VBAR_EL1	(PVM_BOOT_ARGS_OFF_TTBR0_EL1 + 8)
#define PVM_BOOT_ARGS_OFF_TPIDR_EL1	(PVM_BOOT_ARGS_OFF_VBAR_EL1 + 8)
#define PVM_BOOT_ARGS_OFF_SP_EL1	(PVM_BOOT_ARGS_OFF_TPIDR_EL1 + 8)

#endif /* SELFTEST_KVM_ARM64_PKVM_H */
