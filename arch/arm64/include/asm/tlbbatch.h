/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ARCH_ARM64_TLBBATCH_H
#define _ARCH_ARM64_TLBBATCH_H

#include <linux/android_kabi.h>

struct arch_tlbflush_unmap_batch {
	/*
	 * For arm64, HW can do tlb shootdown, so we don't
	 * need to record cpumask for sending IPI
	 */
	ANDROID_KABI_RESERVE(1);
};

#endif /* _ARCH_ARM64_TLBBATCH_H */
