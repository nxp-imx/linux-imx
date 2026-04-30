// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/percpu-defs.h>
#include "kern_stub.h"

void warn_thunk_thunk(void) {}

#ifndef CONFIG_PREEMPTION
int __cond_resched(void) { return 0; }
#endif

#ifdef CONFIG_DEBUG_ATOMIC_SLEEP
void __might_resched(const char *file, int line, unsigned int offsets) {}
#endif

#ifdef CONFIG_DEBUG_PREEMPT
void __this_cpu_preempt_check(const char *op) {}

unsigned int debug_smp_processor_id(void)
{
	return raw_smp_processor_id();
}
#endif
