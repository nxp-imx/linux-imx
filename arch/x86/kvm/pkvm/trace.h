/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _HYP_TRACE_H_
#define _HYP_TRACE_H_

#include <linux/kvm_host.h>

void pkvm_trace_vmexit_start(struct kvm_vcpu *vcpu);
void pkvm_trace_vmexit_end(struct kvm_vcpu *vcpu, u32 index);
void pkvm_vcpu_perf_init(struct kvm_vcpu *vcpu);
void pkvm_enable_vmexit_trace(bool en);
int pkvm_dump_vmexit_trace(phys_addr_t phys, unsigned long size, int vm_handle);

#endif
