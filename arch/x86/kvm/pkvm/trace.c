// SPDX-License-Identifier: GPL-2.0
#include <asm/pkvm_trace.h>
#include <asm/tsc.h>
#include "memory.h"
#include "pkvm.h"
#include "trace.h"

struct perf_ctrl {
	unsigned int age;
	bool on;
};
static DEFINE_PER_CPU(struct vmexit_perf, hvcpu_perf);
static DEFINE_PER_CPU(struct perf_ctrl, perf_ctrl);

static inline bool is_host_vcpu(struct kvm_vcpu *vcpu)
{
	return this_cpu_read(host_vcpu) == vcpu;
}

static inline struct vmexit_perf *vcpu_to_perf(struct kvm_vcpu *vcpu)
{
	/*
	 * TODO: Currently there is only host vCPU but not guest vCPU.
	 * Enable the trace support for guest once the pKVM hypervisor
	 * support running a guest vCPU.
	 */
	BUG_ON(!is_host_vcpu(vcpu));

	return this_cpu_ptr(&hvcpu_perf);
}

static void refresh_vmexit_perf(struct perf_ctrl *pctrl, struct vmexit_perf *perf)
{
	memset(perf->data.vmexit_reasons, 0, sizeof(perf->data.vmexit_reasons));
	memset(perf->data.hypercalls, 0, sizeof(perf->data.hypercalls));

	perf->age = pctrl->age;
}

static void copy_vmexit_perf_data(struct perf_data *dst, struct vmexit_perf *perf)
{
	pkvm_spin_lock(&perf->lock);
	memcpy(dst, &perf->data, sizeof(struct perf_data));
	pkvm_spin_unlock(&perf->lock);
}

static void copy_host_vm_trace(void *dst, unsigned long size)
{
	struct vmexit_perf *perf;
	int cpu;

	for_each_possible_cpu(cpu) {
		perf = per_cpu_ptr(&hvcpu_perf, cpu);
		if (size >= sizeof(struct perf_data)) {
			copy_vmexit_perf_data(dst, perf);
			dst += sizeof(struct perf_data);
			size -= sizeof(struct perf_data);
		}
	}
}

void pkvm_trace_vmexit_start(struct kvm_vcpu *vcpu)
{
	struct perf_ctrl *pctrl = this_cpu_ptr(&perf_ctrl);
	struct vmexit_perf *perf;

	if (!pctrl->on)
		return;

	perf = vcpu_to_perf(vcpu);
	if (pctrl->age != perf->age)
		refresh_vmexit_perf(pctrl, perf);

	perf->rax = vcpu->arch.regs[VCPU_REGS_RAX];
	perf->tsc = rdtsc_ordered();
}

void pkvm_trace_vmexit_end(struct kvm_vcpu *vcpu, u32 reason)
{
	struct perf_ctrl *pctrl = this_cpu_ptr(&perf_ctrl);
	struct vmexit_perf *perf;
	unsigned long long cycles;

	if (!pctrl->on)
		return;

	if (reason >= MAX_EXIT_REASONS)
		return;

	perf = vcpu_to_perf(vcpu);
	if (pctrl->age != perf->age) {
		refresh_vmexit_perf(pctrl, perf);
		return;
	}

	cycles = rdtsc_ordered() - perf->tsc;

	pkvm_spin_lock(&perf->lock);

	perf->data.vmexit_reasons[reason].count++;
	perf->data.vmexit_reasons[reason].cycles += cycles;

	if (is_host_vcpu(vcpu) && reason == EXIT_REASON_VMCALL &&
	    perf->rax < MAX_PKVM_HYPERCALLS) {
		perf->data.hypercalls[perf->rax].count++;
		perf->data.hypercalls[perf->rax].cycles += cycles;
	}

	pkvm_spin_unlock(&perf->lock);
}

void pkvm_vcpu_perf_init(struct kvm_vcpu *vcpu)
{
	struct vmexit_perf *perf = vcpu_to_perf(vcpu);

	perf->data.vcpu_id = vcpu->vcpu_id;

	pkvm_spin_lock_init(&perf->lock);
}

void pkvm_enable_vmexit_trace(bool en)
{
	struct perf_ctrl *pctrl = this_cpu_ptr(&perf_ctrl);

	if (en && !pctrl->on) {
		pctrl->age++;
		pctrl->on = true;
	} else if (!en && pctrl->on) {
		pctrl->on = false;
	}
}

int pkvm_dump_vmexit_trace(phys_addr_t phys, unsigned long size)
{
	int ret = pkvm_host_share_hyp(phys, size);

	if (ret)
		return ret;

	copy_host_vm_trace(__pkvm_va(phys), size);

	pkvm_host_unshare_hyp(phys, size);

	return 0;
}
