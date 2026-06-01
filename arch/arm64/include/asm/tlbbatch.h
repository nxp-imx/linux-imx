/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ARCH_ARM64_TLBBATCH_H
#define _ARCH_ARM64_TLBBATCH_H

#include <linux/android_kabi.h>
#include <linux/cpumask.h>

struct arch_tlbflush_unmap_batch {
#ifdef CONFIG_ARM64_ERRATUM_4193714
	/*
	 * Track CPUs that need SME DVMSync on completion of this batch.
	 * Otherwise, the arm64 HW can do tlb shootdown, so we don't need to
	 * record cpumask for sending IPI
	 */
	cpumask_var_t cpumask;
#endif
	ANDROID_KABI_RESERVE(1);
};

#endif /* _ARCH_ARM64_TLBBATCH_H */
