/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_X86_PKVM_H
#define __PKVM_X86_PKVM_H

#include <linux/kvm_host.h>
#include <asm/kvm_pkvm.h>
#include <asm/pkvm_spinlock.h>
#include <asm/pkvm_trace.h>
#include "gfp.h"
#include "pgtable.h"

DECLARE_PER_CPU(struct pkvm_pcpu *, phys_cpu);
DECLARE_PER_CPU(struct kvm_vcpu *, host_vcpu);
extern size_t kvm_vcpu_sz;

/* Represents a guest vCPU. */
struct pkvm_vcpu {
	/* Point to the kvm_vcpu structure owned by the host */
	struct kvm_vcpu *shared_vcpu;
	/* Point to the pkvm_vm this pkvm_vcpu belongs to */
	struct pkvm_vm *pkvm_vm;
	/* Bitmap of requests for the host to handle */
	unsigned long reqs_to_host;
	/* The host emulated MSR error */
	int host_emulated_msr_err;
	/*
	 * The donated structure size, possibly including a vendor specific
	 * structure wrapping the kvm_vcpu structure (see below).
	 */
	size_t size;
	/* Maximum IRR value recorded for posted interrupts. */
	int max_irr;
	/* Vmexit perf data on this vcpu */
	struct vmexit_perf perf;
	/*
	 * The struct kvm_vcpu should be the last element. In cases where struct
	 * kvm_vcpu is wrapped by a vendor specific structure, putting it as the
	 * last element can safely extend the size of pkvm_vcpu w/o affecting
	 * layout compatibility.
	 *
	 * If struct kvm_vcpu is wrapped by a vendor specific structure, it must
	 * reside at offset 0 of that structure. This ensures &pkvm_vcpu->vcpu
	 * correctly resolves to the underlying struct kvm_vcpu instance. It is
	 * the responsibility of vendor code to guarantee this layout.
	 */
	struct kvm_vcpu vcpu;
};

/* The pkvm_vcpu structure size w/o struct kvm_vcpu */
#define PKVM_VCPU_BASE_SIZE		offsetof(struct pkvm_vcpu, vcpu)

/* Represents a guest VM. */
struct pkvm_vm {
	/* Point to the kvm structure owned by the host */
	struct kvm *shared_kvm;
	/*
	 * The donated structure size, possibly including a vendor specific
	 * structure wrapping the kvm structure (see below).
	 */
	size_t size;
	pkvm_spinlock_t lock;
	struct pkvm_vcpu *vcpus[KVM_MAX_VCPUS];
	atomic_t vcpu_refs[KVM_MAX_VCPUS];
	/* Guest MMU (stage-2) page table managed by the hypervisor */
	struct pkvm_pgtable mmu;
	struct pkvm_pool mmu_pool;
	pkvm_spinlock_t mmu_lock;
	/*
	 * The struct kvm should be the last element. In cases where struct kvm
	 * is wrapped by a vendor specific structure, putting it as the last
	 * element can safely extend the size of pkvm_vm w/o affecting layout
	 * compatibility.
	 *
	 * If struct kvm is wrapped by a vendor specific structure, it must reside
	 * at offset 0 of that structure. This ensures that &pkvm_vm->kvm correctly
	 * resolves to the underlying struct kvm instance. It is the responsibility
	 * of vendor code to guarantee this layout.
	 */
	struct kvm kvm;
};

/* The pkvm_vm structure size w/o struct kvm */
#define PKVM_VM_BASE_SIZE		offsetof(struct pkvm_vm, kvm)

static inline struct pkvm_vm *to_pkvm(struct kvm *kvm)
{
	/*
	 * Make sure the kvm structure is the last element of pkvm_vm. See
	 * comments of struct pkvm_vm.
	 */
	BUILD_BUG_ON(sizeof(struct pkvm_vm) !=
		     PKVM_VM_BASE_SIZE + sizeof(struct kvm));

	return container_of(kvm, struct pkvm_vm, kvm);
}

static inline struct pkvm_vcpu *to_pkvm_vcpu(struct kvm_vcpu *vcpu)
{
	/*
	 * Compiling check to guarantee the kvm_vcpu structure is the last
	 * element of pkvm_vcpu. See comments of struct pkvm_vcpu.
	 */
	BUILD_BUG_ON(sizeof(struct pkvm_vcpu) !=
		     PKVM_VCPU_BASE_SIZE + sizeof(struct kvm_vcpu));

	return container_of(vcpu, struct pkvm_vcpu, vcpu);
}

static inline struct pkvm_vm *pgt_to_pkvm(struct pkvm_pgtable *pgt)
{
	return container_of(pgt, struct pkvm_vm, mmu);
}

static inline struct kvm *pgt_to_kvm(struct pkvm_pgtable *pgt)
{
	return &pgt_to_pkvm(pgt)->kvm;
}

static inline void pkvm_make_req_to_host(int req, struct kvm_vcpu *vcpu)
{
	BUILD_BUG_ON(req >= sizeof(to_pkvm_vcpu(vcpu)->reqs_to_host) * 8);

	set_bit(req, &to_pkvm_vcpu(vcpu)->reqs_to_host);
}

static inline bool pkvm_has_req_to_host(int req, struct kvm_vcpu *vcpu)
{
	BUILD_BUG_ON(req >= sizeof(to_pkvm_vcpu(vcpu)->reqs_to_host) * 8);

	return test_bit(req, &to_pkvm_vcpu(vcpu)->reqs_to_host);
}

static inline bool pkvm_vm_has_pvmfw(struct kvm *kvm)
{
	return kvm->arch.pkvm.pvmfw_load_addr != INVALID_GPA;
}

static inline bool pkvm_vcpu_is_pvmfw_bsp(struct kvm_vcpu *vcpu)
{
	return kvm_vcpu_is_reset_bsp(vcpu) && pkvm_vm_has_pvmfw(vcpu->kvm);
}

struct pkvm_x86_ops {
	void (*update_vcpu_state_from_host)(struct kvm_vcpu *vcpu);
	void (*share_vcpu_state_with_host)(struct kvm_vcpu *vcpu);
};

void pkvm_handle_host_hypercall(struct kvm_vcpu *vcpu);
void pkvm_kick_vcpu(struct kvm_vcpu *vcpu);
int pkvm_x86_vendor_init(struct kvm_x86_init_ops *ops);
struct pkvm_vm *pkvm_get_vm(int vm_handle);
void pkvm_put_vm(struct pkvm_vm *pkvm_vm);
struct pkvm_vcpu *pkvm_get_vcpu(int vm_handle, int vcpu_handle);
void pkvm_put_vcpu(struct pkvm_vcpu *pkvm_vcpu);
unsigned long pkvm_pcpu_tss(int cpu);
int pkvm_start_secondary_vcpu(struct kvm *kvm, u32 apic_id, unsigned long start_ip);
int pkvm_vcpu_enter_guest(struct kvm_vcpu *vcpu, bool force_immediate_exit,
			  unsigned long *reqs_to_host);
void pkvm_x86_ops_init(struct pkvm_x86_ops *ops);
int pkvm_emulate_hypercall(struct kvm_vcpu *vcpu);
typedef int (*pkvm_vm_func_t)(struct pkvm_vm *vm, void *arg);
int pkvm_walk_each_vm(pkvm_vm_func_t func, void *arg);

#endif /* __PKVM_X86_PKVM_H */
