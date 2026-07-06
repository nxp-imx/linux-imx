/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ARCH_ARM64_TLBBATCH_H
#define _ARCH_ARM64_TLBBATCH_H

#include <linux/android_kabi.h>
#include <linux/cpumask.h>

struct arch_tlbflush_unmap_batch {
	/*
	 * For arm64, HW can do TLB shootdown, so we don't need to record a
	 * cpumask for sending IPIs.
	 */
#ifdef CONFIG_ARM64_ERRATUM_4193714
	ANDROID_KABI_REPLACE(cpumask_var_t, cpumask, u64 __donotuse_b448672457);
#endif
	ANDROID_KABI_RESERVE(1);
};

#endif /* _ARCH_ARM64_TLBBATCH_H */
