/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_VMX_HOST_VMX_H
#define __PKVM_VMX_HOST_VMX_H

#include <vmx/vmx.h>

static inline void request_host_immediate_exit(struct vcpu_vmx *vmx)
{
	pin_controls_setbit(vmx, PIN_BASED_VMX_PREEMPTION_TIMER);
	vmcs_write32(VMX_PREEMPTION_TIMER_VALUE, 0);
}

void pkvm_host_vmexit_main(struct vcpu_vmx *vmx);

void pkvm_vmx_reprivilege_cpu(unsigned long *vcpu_regs);

void pkvm_host_vmx_fixup(struct vcpu_vmx *vmx);

#endif /* __PKVM_VMX_HOST_VMX_H */
