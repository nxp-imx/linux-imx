// SPDX-License-Identifier: GPL-2.0
#include <asm/pkvm_trace.h>
#include <asm/tsc.h>
#include "memory.h"
#include "pkvm.h"
#include "trace.h"

static bool trace_on;
static atomic_t trace_age;

static DEFINE_PER_CPU(struct vmexit_perf, hvcpu_perf);

static inline bool is_host_vcpu(struct kvm_vcpu *vcpu)
{
	return this_cpu_read(host_vcpu) == vcpu;
}

static inline struct vmexit_perf *vcpu_to_perf(struct kvm_vcpu *vcpu)
{
	return is_host_vcpu(vcpu) ? this_cpu_ptr(&hvcpu_perf) :
				    &to_pkvm_vcpu(vcpu)->perf;
}

static void refresh_vmexit_perf(struct vmexit_perf *perf)
{
	pkvm_spin_lock(&perf->lock);
	memset(perf->data.vmexit_reasons, 0, sizeof(perf->data.vmexit_reasons));
	memset(perf->data.hypercalls, 0, sizeof(perf->data.hypercalls));
	pkvm_spin_unlock(&perf->lock);
}

static void refresh_host_vm_trace(void)
{
	int cpu;

	for_each_possible_cpu(cpu)
		refresh_vmexit_perf(per_cpu_ptr(&hvcpu_perf, cpu));
}

static int __refresh_guest_vm_trace(struct pkvm_vm *vm, void *param)
{
	struct kvm_vcpu *vcpu;
	int i;

	pkvm_spin_lock(&vm->lock);
	for_each_pkvm_guest_vcpu(i, vcpu, vm)
		refresh_vmexit_perf(&to_pkvm_vcpu(vcpu)->perf);
	pkvm_spin_unlock(&vm->lock);

	return 0;
}

static void refresh_guest_vm_trace(void)
{
	pkvm_walk_each_vm(__refresh_guest_vm_trace, NULL);
}

static void copy_vmexit_perf_data(struct perf_data *dst, struct vmexit_perf *perf)
{
	pkvm_spin_lock(&perf->lock);
	memcpy(dst, &perf->data, sizeof(struct perf_data));
	pkvm_spin_unlock(&perf->lock);
}

static void copy_host_vm_trace(void **dst, unsigned long *size)
{
	struct vmexit_perf *perf;
	int cpu;

	for_each_possible_cpu(cpu) {
		perf = per_cpu_ptr(&hvcpu_perf, cpu);
		if (*size >= sizeof(struct perf_data)) {
			copy_vmexit_perf_data(*dst, perf);
			*dst += sizeof(struct perf_data);
			*size -= sizeof(struct perf_data);
		}
	}
}

struct copy_arg {
	int vm_handle;
	void *dst;
	unsigned long size;
};

static int __copy_guest_vm_trace(struct pkvm_vm *vm, void *param)
{
	struct copy_arg *arg = param;
	struct kvm_vcpu *vcpu;
	int i;

	if (arg->vm_handle != PKVM_HOST_VM_HANDLE &&
	    arg->vm_handle != vm->kvm.arch.pkvm.handle)
		return 0;

	pkvm_spin_lock(&vm->lock);
	for_each_pkvm_guest_vcpu(i, vcpu, vm) {
		if (arg->size < sizeof(struct perf_data)) {
			pkvm_spin_unlock(&vm->lock);
			return -ENOSPC;
		}

		copy_vmexit_perf_data(arg->dst, &to_pkvm_vcpu(vcpu)->perf);
		arg->dst += sizeof(struct perf_data);
		arg->size -= sizeof(struct perf_data);
	}
	pkvm_spin_unlock(&vm->lock);

	/*
	 * If vm_handle != PKVM_HOST_VM_HANDLE, it means the host wants to get
	 * the trace for a specific guest VM, and the pKVM can stop dumping the
	 * other guest VM's trace. Otherwise, the pKVM will continue.
	 */
	return arg->vm_handle != PKVM_HOST_VM_HANDLE;
}

static void copy_guest_vm_trace(int vm_handle, void *dst, unsigned long size)
{
	struct copy_arg arg = {
		.vm_handle = vm_handle,
		.dst = dst,
		.size = size,
	};

	pkvm_walk_each_vm(__copy_guest_vm_trace, &arg);
}

void pkvm_trace_vmexit_start(struct kvm_vcpu *vcpu)
{
	struct vmexit_perf *perf;

	/*
	 * Read trace_on before trace_age. Pairs with smp_store_release() in
	 * pkvm_enable_vmexit_trace().
	 */
	if (likely(!smp_load_acquire(&trace_on)))
		return;

	perf = vcpu_to_perf(vcpu);

	perf->rax = vcpu->arch.regs[VCPU_REGS_RAX];
	perf->tsc = rdtsc_ordered();
	perf->age = atomic_read(&trace_age);
}

void pkvm_trace_vmexit_end(struct kvm_vcpu *vcpu, u32 reason)
{
	struct vmexit_perf *perf;
	unsigned long long cycles;

	/*
	 * Read trace_on before trace_age. Pairs with smp_store_release() in
	 * pkvm_enable_vmexit_trace().
	 */
	if (likely(!smp_load_acquire(&trace_on)))
		return;

	if (reason >= MAX_EXIT_REASONS)
		return;

	perf = vcpu_to_perf(vcpu);
	if (perf->age != atomic_read(&trace_age))
		return;

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

	perf->data.vm_handle = vcpu->kvm->arch.pkvm.handle;
	perf->data.vcpu_id = vcpu->vcpu_id;

	pkvm_spin_lock_init(&perf->lock);
}

void pkvm_enable_vmexit_trace(bool en)
{
	if (en) {
		refresh_host_vm_trace();
		refresh_guest_vm_trace();

		atomic_inc(&trace_age);
	}

	/*
	 * Update trace_on after updating trace_age. Pairs with smp_load_acquire()
	 * in pkvm_trace_vmexit_start()/pkvm_trace_vmexit_end().
	 */
	smp_store_release(&trace_on, en);
}

int pkvm_dump_vmexit_trace(phys_addr_t phys, unsigned long size, int vm_handle)
{
	int ret = pkvm_host_share_hyp(phys, size);
	unsigned long dst_size = size;
	void *dst = __pkvm_va(phys);

	if (ret)
		return ret;

	if (vm_handle == PKVM_HOST_VM_HANDLE)
		copy_host_vm_trace(&dst, &dst_size);

	copy_guest_vm_trace(vm_handle, dst, dst_size);

	pkvm_host_unshare_hyp(phys, size);

	return 0;
}
