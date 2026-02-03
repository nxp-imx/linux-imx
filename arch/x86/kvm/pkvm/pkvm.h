/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_X86_PKVM_H
#define __PKVM_X86_PKVM_H

#include <linux/kvm_host.h>
#include <asm/kvm_pkvm.h>
#include <asm/pkvm_spinlock.h>

DECLARE_PER_CPU(struct pkvm_pcpu *, phys_cpu);
DECLARE_PER_CPU(struct kvm_vcpu *, host_vcpu);
extern size_t kvm_vcpu_sz;

/* Represents a guest vCPU. */
struct pkvm_vcpu {
	/* Point to the kvm_vcpu structure owned by the host */
	struct kvm_vcpu *shared_vcpu;
	/* Point to the pkvm_vm this pkvm_vcpu belongs to */
	struct pkvm_vm *pkvm_vm;
	/*
	 * The donated structure size, possibly including a vendor specific
	 * structure wrapping the kvm_vcpu structure (see below).
	 */
	size_t size;
	/* Maximum IRR value recorded for posted interrupts. */
	int max_irr;
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

void pkvm_handle_host_hypercall(struct kvm_vcpu *vcpu);
void pkvm_kick_vcpu(struct kvm_vcpu *vcpu);
int pkvm_x86_vendor_init(struct kvm_x86_init_ops *ops);
struct pkvm_vm *pkvm_get_vm(int vm_handle);
void pkvm_put_vm(struct pkvm_vm *pkvm_vm);
struct pkvm_vcpu *pkvm_get_vcpu(int vm_handle, int vcpu_handle);
void pkvm_put_vcpu(struct pkvm_vcpu *pkvm_vcpu);
unsigned long pkvm_pcpu_tss(int cpu);

#endif /* __PKVM_X86_PKVM_H */
