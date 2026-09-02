/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
#ifndef _ASM_X86_PKVM_SPINLOCK_H
#define _ASM_X86_PKVM_SPINLOCK_H

#include <linux/types.h>

typedef union pkvm_spinlock {
	u64	__val;
	struct {
		u32 head;
		u32 tail;
	};
} pkvm_spinlock_t;

#define __PKVM_SPINLOCK_UNLOCKED	((pkvm_spinlock_t){ .__val = 0 })

#define DEFINE_PKVM_SPINLOCK(x)		pkvm_spinlock_t x = __PKVM_SPINLOCK_UNLOCKED

#define pkvm_spin_lock_init(l)		(*(l) = __PKVM_SPINLOCK_UNLOCKED)

static inline void pkvm_spin_lock(pkvm_spinlock_t *lock)
{
	/* The lock function atomically increments and exchanges the head
	 * counter of the queue. If the old head of the queue is equal to the
	 * tail, we have locked the spinlock. Otherwise we have to wait.
	 */
	asm volatile ("   movl $0x1,%%eax\n"
		      "   lock xaddl %%eax,%[head]\n"
		      "   cmpl %%eax,%[tail]\n"
		      "   jz 1f\n"
		      "2: pause\n"
		      "   cmpl %%eax,%[tail]\n"
		      "   jnz 2b\n"
		      "1:\n"
		      : [head] "+m"(lock->head)
		      : [tail] "m"(lock->tail)
		      : "cc", "memory", "eax");
}

static inline void pkvm_spin_unlock(pkvm_spinlock_t *lock)
{
	/* Increment tail of queue */
	asm volatile ("   lock incl %[tail]\n"
		      : [tail] "+m" (lock->tail)
		      :
		      : "cc", "memory");
}

#endif
