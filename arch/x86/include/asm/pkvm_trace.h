/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_PKVM_TRACE_H
#define _ASM_X86_PKVM_TRACE_H

#include <asm/kvm_pkvm.h>
#include <asm/pkvm_spinlock.h>
#include <asm/vmx.h>

/*
 * TODO: If there is new vmx vmexit reason in the future which is larger than
 * the EXIT_REASON_MSR_WRITE_IMM, the MAX_VMX_EXIT_REASONS should also be
 * enlarged.
 *
 * TODO: The implementation of vmexit tracing is VMX-specific. Make it generic.
 */
#define MAX_VMX_EXIT_REASONS	(EXIT_REASON_MSR_WRITE_IMM + 1)
#define MAX_EXIT_REASONS	MAX_VMX_EXIT_REASONS

struct vmexit_stats {
	u64 count;
	u64 cycles;
};

struct perf_data {
	struct vmexit_stats vmexit_reasons[MAX_EXIT_REASONS];
	struct vmexit_stats hypercalls[MAX_PKVM_HYPERCALLS];
	int vcpu_id;
	int vm_handle;
};

struct vmexit_perf {
	pkvm_spinlock_t lock;
	struct perf_data data;
	unsigned long rax;
	unsigned long long tsc;
	unsigned int age;
};

#endif /* _ASM_X86_PKVM_TRACE_H */
