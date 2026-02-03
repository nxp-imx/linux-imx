/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_X86_LAPIC_H
#define __PKVM_X86_LAPIC_H

#include <linux/types.h>

int pkvm_lapic_init(void);
void pkvm_lapic_send_init(int cpu);
int pkvm_lapic_msr_write(u32 msr, u64 val);

#endif /* __PKVM_X86_LAPIC_H */
