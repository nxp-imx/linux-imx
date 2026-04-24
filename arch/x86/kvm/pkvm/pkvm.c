// SPDX-License-Identifier: GPL-2.0
#include <linux/types.h>
#include <asm/fpu/xcr.h>
#include <asm/pkvm_spinlock.h>
#include "debug.h"
#include "fpu.h"
#include "init.h"
#include "lapic.h"
#include "mem_protect.h"
#include "memory.h"
#include "mmu.h"
#include "pkvm.h"
#include "trace.h"
#include "../x86.h"
#include "../lapic.h"
#include "pkvm_iommu.h"

/*
 * Needed by kvm_spurious_fault() which is a generic fault function for the
 * vendor operations, e.g., vmx ops or svm ops. The pKVM hypervisor doesn't
 * have the knowledge about the platform reboot or shutdown, so kvm_rebooting
 * is always false in the pKVM hypervisor.
 */
__visible bool kvm_rebooting;

/*
 * Needed by code sharing with the KVM. As the pKVM hypervisor requires to have
 * a second level page table to translate GPA to HPA, set tdp_enabled as true.
 */
bool tdp_enabled = true;

struct pkvm_hyp *pkvm_hyp;
DEFINE_PER_CPU(struct pkvm_pcpu *, phys_cpu);
DEFINE_PER_CPU(struct kvm_vcpu *, host_vcpu);
DEFINE_PER_CPU(bool, host_vcpu_fixup);
/*
 * similarly pmu.c is not compiled. define kvm_mmu_cap here for the use
 * in cpuid.c
 */
struct x86_pmu_capability __read_mostly kvm_pmu_cap = {0};

/* The maximum number of VMs under pkvm. */
#define MAX_PKVM_VMS		64

static DECLARE_BITMAP(pkvm_vms_bitmap, MAX_PKVM_VMS);
static DEFINE_PKVM_SPINLOCK(pkvm_vms_lock);
static struct pkvm_vm_ref {
	/* Reference counter to indicate if pkvm_vm is in use */
	atomic_t refcount;
	/* Point to pkvm_vm in pkvm */
	struct pkvm_vm *pkvm_vm;
} pkvm_vms_ref[MAX_PKVM_VMS];

/*
 * Represents the actual, extended kvm_vcpu structure size. It is initialized as
 * the size of struct kvm_vcpu. And if the vendor code extends kvm_vcpu instance
 * via embedding struct kvm_vcpu to its specific structure, this size should also
 * be extended by the vendor code.
 */
size_t kvm_vcpu_sz = sizeof(struct kvm_vcpu);

/* The current loaded guest vCPU. */
static DEFINE_PER_CPU(struct kvm_vcpu*, cur_guest_vcpu);

static struct pkvm_x86_ops pkvm_x86_ops __read_mostly;

/* TODO: If can be optimized with the static call mechanism. */
#define pkvm_x86_call(func)		(pkvm_x86_ops.func)

static int __pkvm_vcpu_free(struct pkvm_vm *pkvm_vm, int vcpu_handle,
			    struct pkvm_memcache *mc);

static int pkvm_enable_virtualization_cpu(void)
{
	kvm_user_return_msr_cpu_online();

	return kvm_x86_call(enable_virtualization_cpu)();
}

static int allocate_pkvm_vm_handle(struct pkvm_vm *pkvm_vm)
{
	struct pkvm_vm_ref *pkvm_vm_ref;
	int idx;

	pkvm_spin_lock(&pkvm_vms_lock);

	idx = find_first_zero_bit(pkvm_vms_bitmap, MAX_PKVM_VMS);
	if (idx == MAX_PKVM_VMS) {
		pkvm_spin_unlock(&pkvm_vms_lock);
		return -ENOMEM;
	}
	__set_bit(idx, pkvm_vms_bitmap);

	pkvm_vm_ref = &pkvm_vms_ref[idx];
	pkvm_vm_ref->pkvm_vm = pkvm_vm;
	atomic_set(&pkvm_vm_ref->refcount, 1);

	pkvm_spin_unlock(&pkvm_vms_lock);

	return idx;
}

static struct pkvm_vm *free_pkvm_vm_handle(int handle)
{
	struct pkvm_vm_ref *pkvm_vm_ref;
	struct pkvm_vm *pkvm_vm = NULL;
	int idx = handle;

	if (idx < 0 || idx >= MAX_PKVM_VMS)
		return NULL;

	pkvm_spin_lock(&pkvm_vms_lock);

	idx = array_index_nospec(idx, MAX_PKVM_VMS);
	pkvm_vm_ref = &pkvm_vms_ref[idx];
	if (atomic_cmpxchg(&pkvm_vm_ref->refcount, 1, 0) != 1) {
		pkvm_err("VM%d is busy, refcount %d\n", handle,
			 atomic_read(&pkvm_vm_ref->refcount));
		goto out;
	}

	pkvm_vm = pkvm_vm_ref->pkvm_vm;
	pkvm_vm_ref->pkvm_vm = NULL;

	__clear_bit(idx, pkvm_vms_bitmap);
out:
	pkvm_spin_unlock(&pkvm_vms_lock);
	return pkvm_vm;
}

static int pkvm_vm_init(phys_addr_t host_kvm_pa, phys_addr_t pkvm_vm_pa,
			phys_addr_t pgd_pa)
{
	struct pkvm_vm *pkvm_vm;
	struct kvm *kvm;
	size_t size;
	u8 vm_type;
	int ret;

	ret = pkvm_host_share_hyp(host_kvm_pa, kvm_x86_ops.vm_size);
	if (ret)
		return ret;

	size = PAGE_ALIGN(PKVM_VM_BASE_SIZE + kvm_x86_ops.vm_size);
	ret = pkvm_host_donate_hyp(pkvm_vm_pa, size, true);
	if (ret)
		goto unshare;

	pkvm_vm = __pkvm_va(pkvm_vm_pa);
	pkvm_vm->size = size;
	pkvm_vm->shared_kvm = __pkvm_va(host_kvm_pa);
	kvm = &pkvm_vm->kvm;

	vm_type = pkvm_vm->shared_kvm->arch.vm_type;
	if (!kvm_is_vm_type_supported(vm_type)) {
		ret = -EOPNOTSUPP;
		goto undonate;
	}

	kvm->arch.vm_type = vm_type;
	if (pkvm_is_protected_vm(kvm))
		kvm->arch.disabled_quirks = kvm_caps.inapplicable_quirks &
					    kvm_caps.supported_quirks;
	else
		kvm->arch.disabled_quirks = (kvm_caps.inapplicable_quirks |
					     pkvm_vm->shared_kvm->arch.disabled_quirks) &
					    kvm_caps.supported_quirks;
	kvm->arch.apic_bus_cycle_ns = APIC_BUS_CYCLE_NS_DEFAULT;
	kvm->arch.pkvm.pvmfw_load_addr = INVALID_GPA;

	pkvm_spin_lock_init(&pkvm_vm->lock);

	ret = pkvm_guest_mmu_init(pkvm_vm, pgd_pa);
	if (ret)
		goto undonate;

	ret = allocate_pkvm_vm_handle(pkvm_vm);
	if (ret < 0)
		goto mmu_destroy;

	kvm->arch.pkvm.handle = ret;

	ret = kvm_x86_call(vm_init)(kvm);
	if (ret)
		goto free_handle;

	return kvm->arch.pkvm.handle;

free_handle:
	free_pkvm_vm_handle(kvm->arch.pkvm.handle);
mmu_destroy:
	pkvm_guest_mmu_destroy(pkvm_vm);
undonate:
	pkvm_hyp_donate_host(__pkvm_pa(pkvm_vm), size, false);
unshare:
	pkvm_host_unshare_hyp(host_kvm_pa, kvm_x86_ops.vm_size);
	return ret;
}

static void teardown_donated_memory(struct pkvm_memcache *mc, void *addr, size_t size)
{
	BUG_ON(!PAGE_ALIGNED(addr) || !PAGE_ALIGNED(size));

	pkvm_clear_memory(addr, size);

	push_pkvm_memcache(mc, addr, size, pkvm_virt_to_host_gpa);

	/*
	 * Sensitive data in this memory range has been already cleared
	 * by pkvm_clear_memory(). Now this memory is used to store the
	 * information about the memory pages for the host to free by
	 * push_pkvm_memcache(), so undonate without clearing.
	 */
	pkvm_hyp_donate_host(__pkvm_pa(addr), size, false);
}

static void pkvm_vm_destroy(int vm_handle, struct pkvm_memcache *mc)
{
	struct pkvm_vm *pkvm_vm = free_pkvm_vm_handle(vm_handle);
	unsigned long shared_kvm_pa;
	int i;

	if (!pkvm_vm)
		return;

	memset(mc, 0, sizeof(*mc));

	/*
	 * Normally all the created pkvm_vcpus should have been freed already
	 * by the vcpu_free PV interface. In case any pkvm_vcpu is still not
	 * freed, try to free it here.
	 */
	for (i = 0; i < pkvm_vm->kvm.created_vcpus; i++)
		__pkvm_vcpu_free(pkvm_vm, i, mc);

	shared_kvm_pa = __pkvm_pa(pkvm_vm->shared_kvm);

	kvm_x86_call(vm_destroy)(&pkvm_vm->kvm);

	pkvm_guest_mmu_destroy(pkvm_vm);

	teardown_donated_memory(mc, (void *)pkvm_vm, pkvm_vm->size);

	pkvm_host_unshare_hyp(shared_kvm_pa, kvm_x86_ops.vm_size);
}

static int attach_pkvm_vcpu_to_vm(struct pkvm_vm *pkvm_vm, struct pkvm_vcpu *pkvm_vcpu)
{
	struct kvm *kvm = &pkvm_vm->kvm;
	int vcpu_handle;

	pkvm_spin_lock(&pkvm_vm->lock);

	if (kvm->created_vcpus == KVM_MAX_VCPUS) {
		pkvm_spin_unlock(&pkvm_vm->lock);
		return -EINVAL;
	}
	vcpu_handle = kvm->created_vcpus++;
	pkvm_vcpu->vcpu.arch.pkvm.handle = vcpu_handle;
	pkvm_vcpu->pkvm_vm = pkvm_vm;
	pkvm_vm->vcpus[vcpu_handle] = pkvm_vcpu;

	pkvm_spin_unlock(&pkvm_vm->lock);

	atomic_set(&pkvm_vm->vcpu_refs[vcpu_handle], 1);

	return vcpu_handle;
}

static struct pkvm_vcpu *detach_pkvm_vcpu_from_vm(struct pkvm_vm *pkvm_vm, int vcpu_handle)
{
	int refcount = atomic_cmpxchg(&pkvm_vm->vcpu_refs[vcpu_handle], 1, 0);
	struct pkvm_vcpu *pkvm_vcpu;

	if (refcount > 1) {
		/* The pkvm_vcpu is in use and cannot be detached. */
		pkvm_err("VM%d vcpu%d is busy, refcount %d\n",
			 pkvm_vm->kvm.arch.pkvm.handle,
			 vcpu_handle, refcount);
		return NULL;
	} else if (refcount == 0) {
		/* No pkvm_vcpu is attached. */
		return NULL;
	}

	BUG_ON(refcount != 1);

	pkvm_spin_lock(&pkvm_vm->lock);

	pkvm_vcpu = pkvm_vm->vcpus[vcpu_handle];
	pkvm_vm->vcpus[vcpu_handle] = NULL;

	pkvm_spin_unlock(&pkvm_vm->lock);

	return pkvm_vcpu;
}

static int setup_vcpu_lapic(struct kvm_vcpu *vcpu)
{
	struct kvm_lapic *apic = vcpu->arch.apic, *shared_apic;
	struct pkvm_vcpu *pkvm_vcpu = to_pkvm_vcpu(vcpu);
	size_t apic_size = sizeof(struct kvm_lapic);
	void *shared_lapic_regs = NULL;
	int ret;

	if (!apic)
		return 0;

	shared_apic = kern_pkvm_va(pkvm_vcpu->shared_vcpu->arch.apic);
	/*
	 * Temporary sharing host's apic structure to access its elements for
	 * setting up pKVM's apic structure. It will be unshared after that.
	 */
	ret = pkvm_host_share_hyp(__pkvm_pa(shared_apic), apic_size);
	if (ret)
		return ret;

	shared_lapic_regs = kern_pkvm_va(shared_apic->regs);
	if (!shared_lapic_regs) {
		ret = -EINVAL;
		goto unshare_apic;
	}

	ret = pkvm_host_share_hyp(__pkvm_pa(shared_lapic_regs), PAGE_SIZE);
	if (ret)
		goto unshare_apic;

	pkvm_vcpu->shared_lapic_regs = shared_lapic_regs;
	/*
	 * For a protected vCPU with APICv enabled, the pKVM hypervisor will
	 * donate the separate page for the vCPU's LAPIC registers to enforce
	 * the protection. For other cases, the pKVM hypervisor can directly
	 * use the host's apic page as the host will use PV interfaces which are
	 * enforced the protection to inject interrupts.
	 */
	if (!pkvm_is_protected_vcpu(vcpu) || !enable_apicv) {
		apic->regs = shared_lapic_regs;
	} else {
		apic->regs = PTR_ALIGN((void *)apic + apic_size, PAGE_SIZE);
		apic->guest_apic_protected = true;

		/*
		 * The separate page for the vCPU's LAPIC registers should be
		 * within the donated memory range of pkvm_vcpu, which is
		 * guaranteed by the pkvm_vcpu_create. Otherwise it is a code
		 * bug.
		 */
		BUG_ON((apic->regs + PAGE_SIZE) > ((void *)pkvm_vcpu + pkvm_vcpu->size));
	}

	if (enable_apicv) {
		apic->apicv_active = true;
		kvm_make_request(KVM_REQ_APICV_UPDATE, vcpu);
	}
	apic->nr_lvt_entries = kvm_apic_calc_nr_lvt_entries(vcpu);
	apic->vcpu = vcpu;

unshare_apic:
	pkvm_host_unshare_hyp(__pkvm_pa(shared_apic), apic_size);
	return ret;
}

static int pkvm_vm_finalize(int vm_handle)
{
	struct kvm *kvm, *shared_kvm;
	struct pkvm_vm *pkvm_vm;
	struct kvm_vcpu *vcpu;
	u64 pvmfw_load_addr;
	int ret = 0, i;

	pkvm_vm = pkvm_get_vm(vm_handle);
	if (!pkvm_vm)
		return -EINVAL;

	kvm = &pkvm_vm->kvm;
	shared_kvm = pkvm_vm->shared_kvm;

	if (!pkvm_is_protected_vm(kvm)) {
		ret = -EINVAL;
		goto put_pkvm_vm;
	}

	pkvm_spin_lock(&pkvm_vm->lock);

	if (kvm->arch.pkvm.finalized) {
		ret = -EBUSY;
		goto unlock;
	}

	pvmfw_load_addr = READ_ONCE(shared_kvm->arch.pkvm.pvmfw_load_addr);
	if (pvmfw_load_addr != INVALID_GPA) {
		if (!pvmfw_present || U64_MAX - pvmfw_load_addr < pvmfw_size) {
			ret = -EINVAL;
			goto unlock;
		}
		kvm->arch.pkvm.pvmfw_load_addr = pvmfw_load_addr;
	}

	for_each_pkvm_guest_vcpu(i, vcpu, pkvm_vm) {
		if (vcpu->vcpu_id == kvm->arch.bsp_vcpu_id) {
			/*
			 * Make sure pvmfw_load_addr and bsp_vcpu_id are updated before
			 * updating mp_state, i.e. before allowing the primary vCPU
			 * to run. Pairs with smp_load_acquire() in pkvm_vcpu_run().
			 */
			smp_store_release(&vcpu->arch.mp_state, KVM_MP_STATE_RUNNABLE);
			break;
		}
	}

	kvm->arch.pkvm.finalized = true;
	shared_kvm->arch.pkvm.finalized = true;
unlock:
	pkvm_spin_unlock(&pkvm_vm->lock);
put_pkvm_vm:
	pkvm_put_vm(pkvm_vm);
	return ret;
}

static void unsetup_vcpu_lapic(struct kvm_vcpu *vcpu)
{
	void *regs = to_pkvm_vcpu(vcpu)->shared_lapic_regs;

	if (!regs)
		return;

	pkvm_host_unshare_hyp(__pkvm_pa(regs), PAGE_SIZE);
}

static int share_vcpu_mce_banks(struct kvm_vcpu *vcpu)
{
	int ret;

	if (pkvm_is_protected_vcpu(vcpu))
		return -EINVAL;

	ret = pkvm_host_share_hyp(__pkvm_pa(vcpu->arch.mce_banks), KVM_MCE_SIZE);
	if (ret)
		return ret;

	ret = pkvm_host_share_hyp(__pkvm_pa(vcpu->arch.mci_ctl2_banks), KVM_MCI_CTL2_SIZE);
	if (ret)
		pkvm_host_unshare_hyp(__pkvm_pa(vcpu->arch.mce_banks), KVM_MCE_SIZE);

	return ret;
}

static void unshare_vcpu_mce_banks(struct kvm_vcpu *vcpu)
{
	if (pkvm_is_protected_vcpu(vcpu))
		return;

	pkvm_host_unshare_hyp(__pkvm_pa(vcpu->arch.mce_banks), KVM_MCE_SIZE);
	pkvm_host_unshare_hyp(__pkvm_pa(vcpu->arch.mci_ctl2_banks), KVM_MCI_CTL2_SIZE);
}

static void pkvm_vcpu_reset(struct kvm_vcpu *vcpu, bool init_event)
{
	kvm_vcpu_reset(vcpu, init_event);

	if (lapic_in_kernel(vcpu) && vcpu->arch.apic->guest_apic_protected) {
		/*
		 * TPR is the key register for the protected APIC, as it will be
		 * used by the APICv to evaluate pending interrupts. To prevent
		 * the host from injecting exception vectors (0 - 31) via the
		 * posted interrupt mechanism into the pVM, the TPR should be
		 * set as 0x10 to prevent both class 1 and class 0 interrupts.
		 * As the pVM OS may not set the TPR during early boot, or even
		 * doesn't set the TPR at all if it doesn't use APIC, these will
		 * make the TPR as 0x0 which allows the host to inject exception
		 * vectors (16 - 31) into the pVM and may cause security issues.
		 * So enforce the TPR as 0x10 after reset vcpu in the pKVM to
		 * prevent the host from injecting exception vectors from the
		 * beginning of the pVM boot.
		 */
		kvm_set_cr8(vcpu, 1);
	}
}

static int __vcpu_create(struct kvm *kvm, struct kvm_vcpu *vcpu, struct fpstate *fps)
{
	struct pkvm_vcpu *pkvm_vcpu = to_pkvm_vcpu(vcpu);
	int ret = kvm_x86_call(vcpu_precreate)(kvm);
	struct pkvm_vm *pkvm_vm = to_pkvm(kvm);
	void *unused = (void *)pkvm_vcpu +
		       PKVM_VCPU_BASE_SIZE +
		       kvm_vcpu_sz;
	int cpu = raw_smp_processor_id();

	if (ret)
		return ret;

	pkvm_spin_lock(&pkvm_vm->lock);

	/*
	 * The following setup is per VM, not per vCPU, however it cannot be
	 * done during VM creation, since these values are set by the host VMM
	 * via an ioctl after a VM is already created. At the same time, the
	 * host KVM relies on these values being already set when setting up a
	 * vCPU, thus implicitly assuming that the VMM should set them before
	 * creating vCPUs. So it is ok to assume these host's values here are
	 * up-to-date.
	 */
	if (!kvm->arch.bus_lock_detection_enabled &&
	    pkvm_vm->shared_kvm->arch.bus_lock_detection_enabled &&
	    kvm_caps.has_bus_lock_exit)
		kvm->arch.bus_lock_detection_enabled = true;
	if (!kvm->arch.notify_vmexit_flags &&
	    pkvm_vm->shared_kvm->arch.notify_vmexit_flags &&
	    kvm_caps.has_notify_vmexit) {
		kvm->arch.notify_window = pkvm_vm->shared_kvm->arch.notify_window;
		kvm->arch.notify_vmexit_flags = pkvm_vm->shared_kvm->arch.notify_vmexit_flags;
	}
	if (pkvm_vm->shared_kvm->arch.apic_bus_cycle_ns)
		kvm->arch.apic_bus_cycle_ns = pkvm_vm->shared_kvm->arch.apic_bus_cycle_ns;
	if (!pkvm_is_protected_vm(kvm))
		kvm->arch.disabled_exits = pkvm_vm->shared_kvm->arch.disabled_exits;

	if (!kvm->created_vcpus)
		kvm->arch.bsp_vcpu_id = pkvm_vm->shared_kvm->arch.bsp_vcpu_id;

	pkvm_spin_unlock(&pkvm_vm->lock);

	vcpu->kvm = kvm;
	/* Set cpu to -1 to indicate it is not loaded on any CPU */
	vcpu->cpu = -1;

	vcpu->vcpu_id = pkvm_vcpu->shared_vcpu->vcpu_id;
	vcpu->arch.last_vmentry_cpu = -1;
	vcpu->arch.regs_avail = ~0;
	vcpu->arch.regs_dirty = ~0;
	vcpu->arch.mp_state = KVM_MP_STATE_UNINITIALIZED;
	vcpu->arch.pat = MSR_IA32_CR_PAT_DEFAULT;
	if (!pkvm_is_protected_vcpu(vcpu)) {
		vcpu->arch.mce_banks = kern_pkvm_va(pkvm_vcpu->shared_vcpu->arch.mce_banks);
		vcpu->arch.mci_ctl2_banks =
			kern_pkvm_va(pkvm_vcpu->shared_vcpu->arch.mci_ctl2_banks);
		ret = share_vcpu_mce_banks(vcpu);
		if (ret)
			return ret;
	} else {
		vcpu->arch.mce_banks = unused;
		unused += KVM_MCE_SIZE;
		vcpu->arch.mci_ctl2_banks = unused;
		unused += KVM_MCI_CTL2_SIZE;
	}
	vcpu->arch.mcg_cap = KVM_MAX_MCE_BANKS;

	if (lapic_in_kernel(pkvm_vcpu->shared_vcpu))
		vcpu->arch.apic = unused;

	ret = setup_vcpu_lapic(vcpu);
	if (ret)
		goto unshare_mce;

	vcpu->arch.guest_fpu.fpstate = fps;
	pkvm_init_guest_fpu(&vcpu->arch.guest_fpu);
	if (pkvm_is_protected_vcpu(vcpu))
		fpstate_set_confidential(&vcpu->arch.guest_fpu);

	if (kvm_check_has_quirk(vcpu->kvm, KVM_X86_QUIRK_STUFF_FEATURE_MSRS)) {
		vcpu->arch.arch_capabilities = kvm_get_arch_capabilities();
		vcpu->arch.msr_platform_info = MSR_PLATFORM_INFO_CPUID_FAULT;
		vcpu->arch.perf_capabilities = kvm_caps.supported_perf_cap;
	}

	vcpu->arch.root_mmu.root.hpa = pkvm_vm->mmu.root_pa;
	vcpu->arch.root_mmu.root_role.level = pkvm_vm->mmu.cap.level;
	vcpu->arch.mmu = &vcpu->arch.root_mmu;
	vcpu->arch.walk_mmu = &vcpu->arch.root_mmu;

	ret = kvm_x86_call(vcpu_create)(vcpu);
	if (ret)
		goto unsetup_lapic;

	pkvm_vcpu_perf_init(vcpu);

	/* Load guest vCPU to post-set after it is created. */
	kvm_x86_call(vcpu_load)(vcpu, cpu);

	kvm_vcpu_after_set_cpuid(vcpu);

	pkvm_vcpu_reset(vcpu, false);

	if (pkvm_is_protected_vcpu(vcpu)) {
		u64 apic_base = APIC_DEFAULT_PHYS_BASE | LAPIC_MODE_X2APIC |
				(kvm_vcpu_is_reset_bsp(vcpu) ? MSR_IA32_APICBASE_BSP : 0);

		/*
		 * Force set the X86_FEATURE_X2APIC to enable x2apic mode by
		 * default for pVMs to let the pVM use MSR instructions to
		 * access lapic, as emulating xapic mode will require the host
		 * to decode MMIO instruction which is not supported if the
		 * guest is a pVM as the pVM's CPU and memory state will be
		 * isolated. Doing this when creating vCPU to guarantee the
		 * x2apic mode will be enabled.
		 *
		 * Setting X86_FEATURE_X2APIC without checking the pVM's CPUID
		 * is fine as the pVM's CPUID will be enforced by the pKVM to
		 * have this feature.
		 */
		guest_cpu_cap_set(vcpu, X86_FEATURE_X2APIC);
		/*
		 * The kvm_apic_set_base should not be failed as the apic_base
		 * is a valid value, and the pKVM hypervisor has already set up
		 * the reserved bits for checking this apic_base. It should be a
		 * code bug if it is failed.
		 */
		BUG_ON(kvm_apic_set_base(vcpu, apic_base, true));
	}

	/*
	 * The guest vCPU should be put before switching back to the host vCPU
	 * to make sure the vcpu state is not cached on this CPU as this guest
	 * vCPU may be loaded on another CPU later by the host via the PV
	 * interface.
	 */
	kvm_x86_call(vcpu_put)(vcpu);

	kvm_x86_call(vcpu_load)(this_cpu_read(host_vcpu), cpu);

	return 0;

unsetup_lapic:
	unsetup_vcpu_lapic(vcpu);
unshare_mce:
	unshare_vcpu_mce_banks(vcpu);
	return ret;
}

static void __vcpu_free(struct kvm_vcpu *vcpu)
{
	kvm_x86_call(vcpu_free)(vcpu);

	unsetup_vcpu_lapic(vcpu);
	unshare_vcpu_mce_banks(vcpu);
}

static int pkvm_vcpu_create(int vm_handle, phys_addr_t host_vcpu_pa,
			    phys_addr_t pkvm_vcpu_pa, phys_addr_t fpu_pa)
{
	struct kvm_vcpu *shared_vcpu;
	struct pkvm_vcpu *pkvm_vcpu;
	size_t vcpu_size, fps_size;
	struct pkvm_vm *pkvm_vm;
	struct fpstate *fps;
	int ret;

	pkvm_vm = pkvm_get_vm(vm_handle);
	if (!pkvm_vm)
		return -EINVAL;

	ret = pkvm_host_share_hyp(host_vcpu_pa, kvm_vcpu_sz);
	if (ret)
		goto put_vm;

	shared_vcpu = __pkvm_va(host_vcpu_pa);
	vcpu_size = PKVM_VCPU_BASE_SIZE + kvm_vcpu_sz;
	if (pkvm_is_protected_vm(&pkvm_vm->kvm))
		vcpu_size += KVM_MCE_SIZE + KVM_MCI_CTL2_SIZE;
	if (lapic_in_kernel(shared_vcpu)) {
		vcpu_size += sizeof(struct kvm_lapic);
		if (pkvm_is_protected_vm(&pkvm_vm->kvm) && enable_apicv)
			vcpu_size += PAGE_SIZE;
	}
	vcpu_size = PAGE_ALIGN(vcpu_size);

	ret = pkvm_host_donate_hyp(pkvm_vcpu_pa, vcpu_size, true);
	if (ret)
		goto unshare_vcpu;

	pkvm_vcpu = __pkvm_va(pkvm_vcpu_pa);
	pkvm_vcpu->shared_vcpu = shared_vcpu;
	pkvm_vcpu->size = vcpu_size;

	fps_size = pkvm_guest_initial_fpstate_size(&pkvm_vm->kvm);
	ret = pkvm_host_donate_hyp(fpu_pa, fps_size, true);
	if (ret)
		goto undonate_vcpu;

	fps = __pkvm_va(fpu_pa);
	fps->size = fps_size;

	ret = __vcpu_create(&pkvm_vm->kvm, &pkvm_vcpu->vcpu, fps);
	if (ret)
		goto undonate_fps;

	ret = attach_pkvm_vcpu_to_vm(pkvm_vm, pkvm_vcpu);
	if (ret < 0)
		goto destroy_vcpu;

	pkvm_put_vm(pkvm_vm);

	return pkvm_vcpu->vcpu.arch.pkvm.handle;

destroy_vcpu:
	__vcpu_free(&pkvm_vcpu->vcpu);
undonate_fps:
	pkvm_hyp_donate_host(__pkvm_pa(fps), fps_size, false);
undonate_vcpu:
	pkvm_hyp_donate_host(__pkvm_pa(pkvm_vcpu), vcpu_size, false);
unshare_vcpu:
	pkvm_host_unshare_hyp(host_vcpu_pa, kvm_vcpu_sz);
put_vm:
	pkvm_put_vm(pkvm_vm);
	return ret;
}

static int __pkvm_vcpu_free(struct pkvm_vm *pkvm_vm, int vcpu_handle,
			    struct pkvm_memcache *mc)
{
	struct pkvm_vcpu *pkvm_vcpu = detach_pkvm_vcpu_from_vm(pkvm_vm, vcpu_handle);
	unsigned long shared_vcpu_pa;
	struct fpstate *fps;

	if (!pkvm_vcpu)
		return -EINVAL;

	shared_vcpu_pa = __pkvm_pa(pkvm_vcpu->shared_vcpu);

	__vcpu_free(&pkvm_vcpu->vcpu);

	pkvm_guest_mmu_free_memcache(pkvm_vcpu);

	fps = pkvm_vcpu->vcpu.arch.guest_fpu.fpstate;
	teardown_donated_memory(mc, fps, fps->size);
	if (pkvm_vcpu->vcpu.arch.cpuid_entries)
		teardown_donated_memory(mc, pkvm_vcpu->vcpu.arch.cpuid_entries,
					PAGE_ALIGN(sizeof(struct kvm_cpuid_entry2) *
					pkvm_vcpu->vcpu.arch.cpuid_nent));
	teardown_donated_memory(mc, pkvm_vcpu, pkvm_vcpu->size);

	pkvm_host_unshare_hyp(shared_vcpu_pa, kvm_vcpu_sz);

	return 0;
}

static int pkvm_vcpu_free(int vm_handle, int vcpu_handle, struct pkvm_memcache *mc)
{
	struct pkvm_vm *pkvm_vm;
	int ret;

	if (vcpu_handle < 0 || vcpu_handle >= KVM_MAX_VCPUS)
		return -EINVAL;

	pkvm_vm = pkvm_get_vm(vm_handle);
	if (!pkvm_vm)
		return -EINVAL;

	memset(mc, 0, sizeof(*mc));

	ret = __pkvm_vcpu_free(pkvm_vm, array_index_nospec(vcpu_handle, KVM_MAX_VCPUS), mc);

	pkvm_put_vm(pkvm_vm);
	return ret;
}

static int pkvm_vcpu_load(int vm_handle, int vcpu_handle)
{
	struct pkvm_vcpu *pkvm_vcpu = pkvm_get_vcpu(vm_handle, vcpu_handle);
	int cpu = raw_smp_processor_id();
	struct kvm_vcpu *vcpu;
	int loaded_cpu;
	int ret = 0;

	if (!pkvm_vcpu)
		return -EINVAL;

	vcpu = &pkvm_vcpu->vcpu;
	loaded_cpu = cmpxchg(&vcpu->cpu, -1, cpu);
	if (loaded_cpu == -1) {
		/*
		 * Get the pkvm_vcpu to prevent it from being freed via the
		 * vcpu_free PV interface while it is still loaded. If the
		 * obtained pkvm_vcpu is not the same as the original one, it
		 * must be a pkvm bug.
		 */
		BUG_ON(pkvm_vcpu != pkvm_get_vcpu(vm_handle, vcpu_handle));

		/*
		 * Save the PKRU used by the host for the pKVM hypervisor to
		 * switch with the guest. The XCR0 and XSS are already saved in
		 * the kvm_host structure which are not changed at the running
		 * time.
		 */
		vcpu->arch.host_pkru = read_pkru();

		this_cpu_write(cur_guest_vcpu, vcpu);
	} else if (loaded_cpu == cpu) {
		/* The guest vCPU is already loaded on this CPU. */
		this_cpu_write(cur_guest_vcpu, vcpu);
	} else {
		/* The guest vCPU is already loaded on another CPU. */
		ret = -EBUSY;
	}

	pkvm_put_vcpu(pkvm_vcpu);

	return ret;
}

static int pkvm_vcpu_put(int vm_handle, int vcpu_handle)
{
	struct pkvm_vcpu *pkvm_vcpu = pkvm_get_vcpu(vm_handle, vcpu_handle);
	int cpu = raw_smp_processor_id(), loaded_cpu, ret = 0;
	struct kvm_vcpu *vcpu;

	if (!pkvm_vcpu)
		return -EINVAL;

	vcpu = &pkvm_vcpu->vcpu;
	loaded_cpu = vcpu->cpu;
	if (loaded_cpu == cpu) {
		/*
		 * The current active vCPU is the host vCPU. Switch to the guest
		 * vCPU in case vcpu_put operation requires.
		 */
		kvm_x86_call(vcpu_load)(vcpu, cpu);

		/*
		 * Another guest vCPU may have already been loaded on this CPU
		 * thus the cur_guest_vcpu may be overridden. So only set the
		 * cur_guest_vcpu as NULL if it points to the guest vCPU being
		 * put.
		 */
		if (vcpu == this_cpu_read(cur_guest_vcpu))
			this_cpu_write(cur_guest_vcpu, NULL);

		kvm_x86_call(vcpu_put)(vcpu);

		/*
		 * Put this pkvm_vcpu to allow it to be freed via the vcpu_free PV
		 * interface.
		 */
		pkvm_put_vcpu(pkvm_vcpu);

		/* Switch to the host vCPU as a guest vCPU was just loaded. */
		kvm_x86_call(vcpu_load)(this_cpu_read(host_vcpu), cpu);

		/*
		 * Paired with cmpxchg in pkvm_vcpu_load() to make sure the
		 * vcpu->cpu is set only after the put is completed.
		 */
		smp_store_release(&vcpu->cpu, -1);
	} else {
		/*
		 * The guest vCPU is not loaded on any CPU or is loaded on a
		 * different CPU.
		 */
		ret = -EINVAL;
	}

	pkvm_put_vcpu(pkvm_vcpu);

	return ret;
}

static bool is_guest_vcpu_accessible(struct kvm_vcpu *vcpu, enum pkvm_hc hc)
{
	/*
	 * There is no isolation between non-protected VMs and the host, thus
	 * all the PV interfaces are allowed for an npVM.
	 */
	if (!pkvm_is_protected_vcpu(vcpu))
		return true;

	switch (hc) {
	case __pkvm__enable_nmi_window:
	case __pkvm__enable_irq_window:
	case __pkvm__interrupt_allowed:
	case __pkvm__nmi_allowed:
	case __pkvm__get_nmi_mask:
	case __pkvm__inject_irq:
	case __pkvm__inject_nmi:
	case __pkvm__cancel_injection:
	case __pkvm__update_cr8_intercept:
	case __pkvm__refresh_apicv_exec_ctrl:
	case __pkvm__load_eoi_exitmap:
	case __pkvm__hwapic_isr_update:
	case __pkvm__write_tsc_offset:
	case __pkvm__write_tsc_multiplier:
	case __pkvm__setup_mce:
	case __pkvm__vcpu_run:
	case __pkvm__complete_emulated_msr:
	case __pkvm__protected_apic_has_interrupt:
		/*
		 * The host is responsible for running vCPU, injecting
		 * interrupts, emulating lapic etc. Always allow the related PV
		 * interfaces.
		 *
		 * TODO: As the pVM can use another secure time source, the
		 * guest TSC is allowed for the host to emulate and access. To
		 * support the pVM with secure TSC, add protection for TSC
		 * related PV interfaces.
		 *	__pkvm__write_tsc_offset
		 *	__pkvm__write_tsc_multiplier
		 */
		return true;
	case __pkvm__set_efer:
	case __pkvm__set_msr:
	case __pkvm__get_msr:
	case __pkvm__set_cr4:
	case __pkvm__set_cr0:
	case __pkvm__set_rflags:
	case __pkvm__get_rflags:
	case __pkvm__set_segment:
	case __pkvm__get_segment:
	case __pkvm__get_segment_base:
	case __pkvm__set_idt:
	case __pkvm__get_idt:
	case __pkvm__set_gdt:
	case __pkvm__get_gdt:
	case __pkvm__flush_tlb_all:
	case __pkvm__flush_tlb_current:
	case __pkvm__flush_tlb_gva:
	case __pkvm__flush_tlb_guest:
	case __pkvm__vcpu_after_set_cpuid:
	case __pkvm__vcpu_add_fpstate:
	case __pkvm__load_mmu_pgd:
		/*
		 * As the host needs to pre-configure the pVM's vCPU state for
		 * booting, the protection for pVM is only enforced by the pKVM
		 * hypervisor once the vCPU has started running.
		 *
		 * If the pVM runs with pvmfw, the pKVM hypervisor itself will
		 * enforce most of the vcpu's initial state before the first vcpu
		 * starts running, in pkvm_vcpu_pvmfw_entry_init(). However,
		 * before that we don't know yet if the pVM will run with pvmfw
		 * or not, since the host VMM may issue the ioctl enabling pvmfw
		 * either before or after using any of the above PV interfaces.
		 *
		 * For secondary vCPUs, also allow the host to pre-configure the
		 * initial state of the vcpu, even though the hypervisor itself
		 * will enforce the initial state before the secondary vcpu starts
		 * running, in pkvm_vcpu_ap_entry_init(), discarding whatever
		 * the host has pre-configured. This is just for simplicity, to
		 * let the host KVM code work as usual.
		 */
		return !kvm_vcpu_has_run(vcpu);
	default:
		/*
		 * The other PV interfaces are not necessary for the host to
		 * access the pVM's vCPU state. Deny these PV interfaces by
		 * default.
		 */
		return false;
	}
}

static void pkvm_update_exception_bitmap(struct kvm_vcpu *vcpu)
{
	/*
	 * The guest_debug will impact what exceptions should be intercepted
	 * for the debugging purpose. Debugging npVMs from the host side is
	 * allowed thus updating its guest_debug flags accordingly, but
	 * debugging pVMs from the host side is not allowed.
	 *
	 * As the __pkvm__update_exception_bitmap is always denied for the pVM,
	 * it must be a code bug if the vcpu is protected.
	 */
	BUG_ON(pkvm_is_protected_vcpu(vcpu));
	vcpu->guest_debug = to_pkvm_vcpu(vcpu)->shared_vcpu->guest_debug;

	kvm_x86_call(update_exception_bitmap)(vcpu);
}

static int pkvm_set_msr(struct kvm_vcpu *vcpu, u32 index, u64 data)
{
	if (pkvm_is_protected_vcpu(vcpu)) {
		if (WARN_ON(kvm_vcpu_has_run(vcpu)))
			return -EPERM;

		/*
		 * For simplicity and security, allow the host to change
		 * initial values of those MSRs (or individual bits in MSRs)
		 * that are currently tweaked by crosvm, and only those.
		 * The allowed set can be extended as needed.
		 */
		switch (index) {
		case MTRRphysBase_MSR(0) ... MSR_MTRRfix4K_F8000:
		case MSR_MTRRdefType:
			break;
		case MSR_IA32_MISC_ENABLE:
			if (data & ~(MSR_IA32_MISC_ENABLE_FAST_STRING |
				     MSR_IA32_MISC_ENABLE_PEBS_UNAVAIL |
				     MSR_IA32_MISC_ENABLE_BTS_UNAVAIL))
				return -EPERM;

			/*
			 * vPMU is not supported by pKVM yet. Don't trick the pVM
			 * that it is.
			 */
			data |= MSR_IA32_MISC_ENABLE_PEBS_UNAVAIL |
				MSR_IA32_MISC_ENABLE_BTS_UNAVAIL;
			break;
		case MSR_STAR:
		case MSR_LSTAR:
		case MSR_CSTAR:
		case MSR_SYSCALL_MASK:
		case MSR_KERNEL_GS_BASE:
		case MSR_IA32_SYSENTER_CS:
		case MSR_IA32_SYSENTER_ESP:
		case MSR_IA32_SYSENTER_EIP:
			/*
			 * TODO: The user space VMM from the host side (e.g.,
			 * crosvm) may still try to set these MSRs which are
			 * protected by the pKVM hypervisor for a pVM. Ignore
			 * writings to these MSRs and return 0 to make such
			 * user space VMM happy, meanwhile doesn't really modify
			 * these MSRs. This eventually will be fixed in the user
			 * space VMM to avoid doing so for a pVM. Once this is
			 * implemented, these can be removed.
			 */
			return 0;
		default:
			return -EPERM;
		}
	}

	return kvm_msr_write(vcpu, index, data);
}

static int pkvm_cache_reg(struct kvm_vcpu *vcpu, enum kvm_reg reg,
			  union pkvm_hc_data *out)
{
	kvm_x86_call(cache_reg)(vcpu, reg);

	switch (reg) {
	case VCPU_REGS_RSP:
		out->cache_reg.rsp = vcpu->arch.regs[VCPU_REGS_RSP];
		break;
	case VCPU_REGS_RIP:
		out->cache_reg.rip = vcpu->arch.regs[VCPU_REGS_RIP];
		break;
	case VCPU_EXREG_PDPTR: {
		struct kvm_mmu *mmu = vcpu->arch.walk_mmu;

		out->cache_reg.pdptrs[0] = mmu->pdptrs[0];
		out->cache_reg.pdptrs[1] = mmu->pdptrs[1];
		out->cache_reg.pdptrs[2] = mmu->pdptrs[2];
		out->cache_reg.pdptrs[3] = mmu->pdptrs[3];
		break;
	}
	case VCPU_EXREG_CR0:
		out->cache_reg.cr0 = vcpu->arch.cr0;
		break;
	case VCPU_EXREG_CR3:
		out->cache_reg.cr3 = vcpu->arch.cr3;
		break;
	case VCPU_EXREG_CR4:
		out->cache_reg.cr4 = vcpu->arch.cr4;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static void pkvm_set_dr7(struct kvm_vcpu *vcpu, unsigned long val)
{
	unsigned long dr7 = val;

	kvm_x86_call(set_dr7)(vcpu, dr7);
	vcpu->arch.switch_db_regs &= ~KVM_DEBUGREG_BP_ENABLED;
	if (dr7 & DR7_BP_EN_MASK)
		vcpu->arch.switch_db_regs |= KVM_DEBUGREG_BP_ENABLED;
}

static inline bool pkvm_event_injection_allowed(struct kvm_vcpu *vcpu)
{
	return !kvm_event_needs_reinjection(vcpu) &&
	       !vcpu->arch.exception.pending &&
	       !to_pkvm_vcpu(vcpu)->host_emulated_msr_err;
}

static int pkvm_interrupt_allowed(struct kvm_vcpu *vcpu, bool for_injection)
{
	if (for_injection && !pkvm_event_injection_allowed(vcpu))
		return -EBUSY;

	return kvm_x86_call(interrupt_allowed)(vcpu, for_injection);
}

static int pkvm_nmi_allowed(struct kvm_vcpu *vcpu, bool for_injection)
{
	if (for_injection && !pkvm_event_injection_allowed(vcpu))
		return -EBUSY;

	return kvm_x86_call(nmi_allowed)(vcpu, for_injection);
}

static int pkvm_inject_irq(struct kvm_vcpu *vcpu)
{
	struct kvm_vcpu *shared_vcpu = to_pkvm_vcpu(vcpu)->shared_vcpu;

	if (WARN_ON_ONCE(pkvm_interrupt_allowed(vcpu, true) <= 0))
		return -EBUSY;

	/*
	 * Injecting software interrupts will change the guest's RIP. As there
	 * is no usage to require the host to do so for a pVM, disallow the host
	 * to inject software interrupts to a pVM for security reason.
	 *
	 * As the pVM's exceptions are emulated and injected by the pKVM itself,
	 * the host is not allowed to inject exceptions to the pVM. So validate
	 * the interrupt vector number to make sure it won't be a reserved
	 * vector number by the Intel 64 and IA-32 architectures for
	 * architecture-defined exceptions.
	 */
	if (pkvm_is_protected_vcpu(vcpu) && (shared_vcpu->arch.interrupt.soft ||
					     shared_vcpu->arch.interrupt.nr < 32))
		return -EPERM;

	vcpu->arch.interrupt.soft = shared_vcpu->arch.interrupt.soft;
	vcpu->arch.interrupt.nr = shared_vcpu->arch.interrupt.nr;
	kvm_x86_call(inject_irq)(vcpu, false);

	return 0;
}

static int pkvm_inject_nmi(struct kvm_vcpu *vcpu)
{
	if (WARN_ON_ONCE(pkvm_nmi_allowed(vcpu, true) <= 0))
		return -EBUSY;

	kvm_x86_call(inject_nmi)(vcpu);

	return 0;
}

static void pkvm_inject_exception(struct kvm_vcpu *vcpu)
{
	/*
	 * As the __pkvm__inject_exception is always denied for the pVM,
	 * it must be a code bug if the vcpu is protected.
	 */
	BUG_ON(pkvm_is_protected_vcpu(vcpu));
	vcpu->arch.exception = to_pkvm_vcpu(vcpu)->shared_vcpu->arch.exception;

	kvm_x86_call(inject_exception)(vcpu);
}

static void pkvm_cancel_injection(struct kvm_vcpu *vcpu)
{
	struct pkvm_vcpu *pkvm_vcpu = to_pkvm_vcpu(vcpu);
	struct kvm_vcpu *shared_vcpu;

	kvm_x86_call(cancel_injection)(vcpu);

	shared_vcpu = pkvm_vcpu->shared_vcpu;
	if (vcpu->arch.nmi_injected) {
		shared_vcpu->arch.nmi_injected = true;
		vcpu->arch.nmi_injected = false;
	} else if (vcpu->arch.interrupt.injected) {
		/*
		 * The npVM's injected software and external interrupts can be
		 * canceled as the host is allowed to inject both. But the host
		 * is not allowed to inject the pVM's software interrupt, and
		 * the pending pVM's software interrupt (exits during delivering
		 * a software interrupt) should be injected by the pKVM, thus
		 * the host cannot cancel such injection.
		 */
		if (!pkvm_is_protected_vcpu(vcpu) || !vcpu->arch.interrupt.soft) {
			kvm_queue_interrupt(shared_vcpu, vcpu->arch.interrupt.nr,
					    vcpu->arch.interrupt.soft);
			kvm_clear_interrupt_queue(vcpu);
		}
	} else if (!pkvm_is_protected_vcpu(vcpu) && vcpu->arch.exception.injected) {
		/*
		 * For the pVM, the exception can only be injected and canceled
		 * by the pkvm hypervisor.
		 * For the npVM, the exception can be injected and canceled by
		 * both sides.
		 */
		shared_vcpu->arch.exception = vcpu->arch.exception;
		kvm_clear_exception_queue(vcpu);
	}
}

static int pkvm_refresh_apicv_exec_ctrl(struct kvm_vcpu *vcpu, bool apicv_active)
{
	if (!lapic_in_kernel(vcpu) || (!enable_apicv && apicv_active))
		return -EINVAL;

	vcpu->arch.apic->apicv_active = apicv_active;
	kvm_make_request(KVM_REQ_APICV_UPDATE, vcpu);

	return 0;
}

static void pkvm_load_eoi_exitmap(struct kvm_vcpu *vcpu, u64 eoi_exit_bitmap0,
				  u64 eoi_exit_bitmap1, u64 eoi_exit_bitmap2,
				  u64 eoi_exit_bitmap3)
{
	u64 eoi_exit_bitmap[] = {
		eoi_exit_bitmap0,
		eoi_exit_bitmap1,
		eoi_exit_bitmap2,
		eoi_exit_bitmap3,
	};

	kvm_x86_call(load_eoi_exitmap)(vcpu, eoi_exit_bitmap);
}

static int pkvm_hwapic_isr_update(struct kvm_vcpu *vcpu, int max_isr)
{
	if (!lapic_in_kernel(vcpu) || !vcpu->arch.apic->apicv_active)
		return -EOPNOTSUPP;

	/*
	 * The value -1 represents no interrupt, thus always allow the host to
	 * update the ISR in this case. For the other values, should do proper
	 * checks.
	 *
	 * For a protected APIC, the ISR state is protected so don't allow the
	 * host to update it. But the host will recognize the protected apic
	 * after the vCPU starts running. Before that the host may still use
	 * this PV interface with value -1 for the protected apic to indicate no
	 * interrupts when reset the vCPU. So don't allow for any other max_isr
	 * values for the protected apic.
	 *
	 * For pVMs which don't have protected apic, also needs to validate the
	 * max_isr value to make sure the host cannot inject an exception vector
	 * for the same security reason with the PV interface __pkvm__inject_irq.
	 * See comments in the function pkvm_inject_irq.
	 */
	if ((max_isr != -1) && (vcpu->arch.apic->guest_apic_protected ||
				(pkvm_is_protected_vcpu(vcpu) && max_isr < 32)))
		return -EPERM;

	kvm_x86_call(hwapic_isr_update)(vcpu, max_isr);

	return 0;
}

static int pkvm_vcpu_after_set_cpuid(struct kvm_vcpu *vcpu,
				     phys_addr_t cpuid_pa,
				     struct pkvm_memcache *mc)
{
	struct kvm_cpuid_entry2 *new, *old;
	int new_nent, old_nent, ret;
	u64 size, aligned_size;

	new_nent = to_pkvm_vcpu(vcpu)->shared_vcpu->arch.cpuid_nent;
	size = sizeof(struct kvm_cpuid_entry2) * new_nent;
	aligned_size = PAGE_ALIGN(size);
	ret = pkvm_host_donate_hyp(cpuid_pa, aligned_size, false);
	if (ret)
		return ret;

	new = __pkvm_va(cpuid_pa);
	if (pkvm_is_protected_vcpu(vcpu)) {
		/*
		 * Donation is page-granule, so the host must ensure that
		 * the cpuid buffer size is page aligned though the actual
		 * nent only records valid entries.
		 *
		 * Clear the trailing space after nent so it can be used
		 * to hold missing cpuid entries enforced by pkvm.
		 */
		memset((void *)new + size, 0, aligned_size - size);

		ret = pkvm_enforce_cpuid(new, &new_nent,
					 aligned_size / sizeof(struct kvm_cpuid_entry2));
		if (ret)
			goto undonate;
	}

	old = vcpu->arch.cpuid_entries;
	old_nent = vcpu->arch.cpuid_nent;

	ret = kvm_set_cpuid(vcpu, new, new_nent);
	if (ret)
		goto undonate;

	/*
	 * The pVM will directly boot with lapic in x2apic mode, which requires
	 * the X86_FEATURE_X2APIC to be set in the vCPUID. The vCPUID entries
	 * are enforced by pkvm_enforce_cpuid() via overriding the vCPUID leaf
	 * 0x1 ECX X2APIC feature bit with the value from the native CPUID. As
	 * the pKVM initialization requires the native lapic in X2APIC mode, it
	 * means that the native CPUID will always have the X2APIC feature bit
	 * set, thus the enforced vCPUID will also always have the X2APIC set
	 * for a pVM. So it must be a pKVM code bug if the pVM doesn't have
	 * X2APIC feature after enforcing.
	 */
	if (pkvm_is_protected_vcpu(vcpu))
		BUG_ON(!guest_cpu_cap_has(vcpu, X86_FEATURE_X2APIC));

	memset(mc, 0, sizeof(*mc));
	/*
	 * New cpuid entries memory is consumed. Tear down the old cpuid
	 * entries memory if there is.
	 */
	if (old)
		teardown_donated_memory(mc, (void *)old,
					PAGE_ALIGN(sizeof(struct kvm_cpuid_entry2) *
						   old_nent));

	return 0;

undonate:
	pkvm_hyp_donate_host(__pkvm_pa(new), aligned_size, false);
	return ret;
}

static int pkvm_vcpu_add_fpstate(struct kvm_vcpu *vcpu,
				 phys_addr_t fpstate_pa, size_t size,
				 struct pkvm_memcache *mc)
{
	struct fpstate *new, *old;
	int ret;

	/* Expect the host to use this PV interface for pVM only. */
	if (!pkvm_is_protected_vcpu(vcpu))
		return -EINVAL;

	ret = pkvm_host_donate_hyp(fpstate_pa, size, true);
	if (ret)
		return ret;

	memset(mc, 0, sizeof(*mc));

	old = vcpu->arch.guest_fpu.fpstate;
	new = __pkvm_va(fpstate_pa);
	/*
	 * Reuse the existing fpstate memory if it's sufficiently large. At this
	 * stage, we can't determine whether the new fpstate size matches the
	 * vCPUID or not, because that check only occurs when the host calls
	 * __pkvm__vcpu_after_set_cpuid to update the vCPUID. If the new fpstate
	 * size is smaller than what the new vCPUID requires, the vCPUID won't
	 * be updated. Therefore, ensuring the new fpstate size is at least as
	 * large as the previous one allows continued support for this scenario.
	 */
	if (old && old->size >= size) {
		teardown_donated_memory(mc, new, size);
		return 0;
	}

	new->size = size;
	vcpu->arch.guest_fpu.fpstate = new;

	pkvm_init_guest_fpu(&vcpu->arch.guest_fpu);
	fpstate_set_confidential(&vcpu->arch.guest_fpu);

	/*
	 * New physical fpstate memory is consumed. Tear down the old fpstate
	 * memory if there is.
	 */
	if (old)
		teardown_donated_memory(mc, old, old->size);

	return 0;
}

static void pkvm_write_tsc_offset(struct kvm_vcpu *vcpu)
{
	u64 tsc_offset = to_pkvm_vcpu(vcpu)->shared_vcpu->arch.tsc_offset;

	vcpu->arch.l1_tsc_offset = tsc_offset;
	vcpu->arch.tsc_offset = tsc_offset;
	kvm_x86_call(write_tsc_offset)(vcpu);
}

static int pkvm_write_tsc_multiplier(struct kvm_vcpu *vcpu)
{
	u64 ratio = to_pkvm_vcpu(vcpu)->shared_vcpu->arch.tsc_scaling_ratio;

	if (!kvm_caps.has_tsc_control)
		return -EOPNOTSUPP;

	vcpu->arch.l1_tsc_scaling_ratio = ratio;
	vcpu->arch.tsc_scaling_ratio = ratio;
	kvm_x86_call(write_tsc_multiplier)(vcpu);

	return 0;
}

static int pkvm_load_mmu_pgd(struct kvm_vcpu *vcpu)
{
	struct kvm_vcpu *shared_vcpu = to_pkvm_vcpu(vcpu)->shared_vcpu;

	/*
	 * The guest CR3/PDPTR may be updated by the load_mmu_pgd. Sync the
	 * guest CR3/PDPTR from the host.
	 */
	if (kvm_register_is_dirty(shared_vcpu, VCPU_EXREG_CR3)) {
		vcpu->arch.cr3 = shared_vcpu->arch.cr3;
		kvm_register_mark_dirty(vcpu, VCPU_EXREG_CR3);
	}

	if (kvm_register_is_dirty(shared_vcpu, VCPU_EXREG_PDPTR)) {
		struct kvm_mmu *shared_walk_mmu = kern_pkvm_va(shared_vcpu->arch.walk_mmu);
		struct kvm_mmu *walk_mmu = vcpu->arch.walk_mmu;
		int ret;

		ret = pkvm_host_share_hyp(__pkvm_pa(shared_walk_mmu),
					  sizeof(struct kvm_mmu));
		if (ret)
			return ret;

		walk_mmu->pdptrs[0] = shared_walk_mmu->pdptrs[0];
		walk_mmu->pdptrs[1] = shared_walk_mmu->pdptrs[1];
		walk_mmu->pdptrs[2] = shared_walk_mmu->pdptrs[2];
		walk_mmu->pdptrs[3] = shared_walk_mmu->pdptrs[3];
		kvm_register_mark_dirty(vcpu, VCPU_EXREG_PDPTR);

		pkvm_host_unshare_hyp(__pkvm_pa(shared_walk_mmu),
				      sizeof(struct kvm_mmu));
	}

	kvm_x86_call(load_mmu_pgd)(vcpu, vcpu->arch.mmu->root.hpa,
				   vcpu->arch.mmu->root_role.level);

	return 0;
}

static void pkvm_vcpu_pvmfw_entry_init(struct kvm_vcpu *vcpu)
{
	struct kvm_segment seg;
	struct desc_ptr dt;

	/* pvmfw entry point is at the beginning of the pvmfw image. */
	kvm_rip_write(vcpu, vcpu->kvm->arch.pkvm.pvmfw_load_addr);

	/* Force RFLAGS and CR4 to their reset values. */
	kvm_x86_call(set_rflags)(vcpu, X86_EFLAGS_FIXED);
	kvm_x86_call(set_cr4)(vcpu, 0);

	/* pvmfw starts in 32-bit protected mode with paging disabled. */
	kvm_x86_call(set_cr0)(vcpu, X86_CR0_PE | X86_CR0_ET);
	kvm_x86_call(set_efer)(vcpu, 0);

	/* Set up flat 4GB segments. */
	memset(&seg, 0, sizeof(seg));
	seg.limit = 0xffffffff;
	seg.type = 0xb;
	seg.present = 1;
	seg.db = 1;	/* 32-bit segment */
	seg.s = 1;
	seg.g = 1;
	kvm_x86_call(set_segment)(vcpu, &seg, VCPU_SREG_CS);
	seg.type = 0x3;
	kvm_x86_call(set_segment)(vcpu, &seg, VCPU_SREG_DS);
	kvm_x86_call(set_segment)(vcpu, &seg, VCPU_SREG_ES);
	kvm_x86_call(set_segment)(vcpu, &seg, VCPU_SREG_FS);
	kvm_x86_call(set_segment)(vcpu, &seg, VCPU_SREG_GS);
	kvm_x86_call(set_segment)(vcpu, &seg, VCPU_SREG_SS);

	memset(&dt, 0, sizeof(dt));

	/*
	 * Initially hardware will use the cached segment descriptors we've set up
	 * above, so GDT in memory does not matter, until the guest reloads
	 * a segment register. Set the initial GDTR to an invalid GDT, so that
	 * if pvmfw accidentally reloads a segment register before it has set up
	 * its own GDT, it generates a #GP.
	 */
	kvm_x86_call(set_gdt)(vcpu, &dt);

	/* Similarly for TSS */
	memset(&seg, 0, sizeof(seg));
	seg.type = 0xb;
	seg.present = 1;
	kvm_x86_call(set_segment)(vcpu, &seg, VCPU_SREG_TR);

	/* ...and LDT */
	memset(&seg, 0, sizeof(seg));
	seg.unusable = 1;
	kvm_x86_call(set_segment)(vcpu, &seg, VCPU_SREG_LDTR);

	/*
	 * Set the initial IDTR to an invalid IDT, so that any early exception
	 * (before pvmfw sets up its own IDT) results in a triple fault.
	 */
	kvm_x86_call(set_idt)(vcpu, &dt);
}

static void pkvm_vcpu_ap_entry_init(struct kvm_vcpu *vcpu)
{
	pkvm_vcpu_reset(vcpu, true);
	kvm_vcpu_deliver_sipi_vector(vcpu, vcpu->arch.apic->sipi_vector);
}

static void update_vcpu_state_from_host(struct kvm_vcpu *vcpu)
{
	struct kvm_vcpu *shared_vcpu = to_pkvm_vcpu(vcpu)->shared_vcpu;

	if (!pkvm_is_protected_vcpu(vcpu)) {
		/*
		 * Make sure the RSP/RIP in shared_vcpu are aligned with the
		 * private vcpu if they are not dirty.
		 */
		if (kvm_register_is_dirty(shared_vcpu, VCPU_REGS_RSP))
			kvm_register_mark_dirty(vcpu, VCPU_REGS_RSP);
		else
			shared_vcpu->arch.regs[VCPU_REGS_RSP] = kvm_rsp_read(vcpu);
		if (kvm_register_is_dirty(shared_vcpu, VCPU_REGS_RIP))
			kvm_register_mark_dirty(vcpu, VCPU_REGS_RIP);
		else
			shared_vcpu->arch.regs[VCPU_REGS_RIP] = kvm_rip_read(vcpu);
		/* Update the npVM's GPRs from the host */
		memcpy(vcpu->arch.regs, shared_vcpu->arch.regs,
		       NR_VCPU_REGS * sizeof(*vcpu->arch.regs));

		/* Update the debug registers from the host */
		memcpy(vcpu->arch.db, shared_vcpu->arch.db,
		       ARRAY_SIZE(vcpu->arch.db) * sizeof(*vcpu->arch.db));
		memcpy(vcpu->arch.eff_db, shared_vcpu->arch.eff_db,
		       ARRAY_SIZE(vcpu->arch.eff_db) * sizeof(*vcpu->arch.eff_db));
		vcpu->arch.dr6 = shared_vcpu->arch.dr6;
		vcpu->arch.dr7 = shared_vcpu->arch.dr7;
		vcpu->arch.xcr0 = shared_vcpu->arch.xcr0;
		if (vcpu->guest_debug & KVM_GUESTDBG_USE_HW_BP)
			vcpu->arch.guest_debug_dr7 = shared_vcpu->arch.guest_debug_dr7;
		if (vcpu->guest_debug & KVM_GUESTDBG_SINGLESTEP)
			vcpu->arch.singlestep_rip = shared_vcpu->arch.singlestep_rip;
	} else if (unlikely(!kvm_vcpu_has_run(vcpu) && kvm_vcpu_is_reset_bsp(vcpu))) {
		/*
		 * Allow the host VMM to set the initial values of most GPRs
		 * to let it pass boot information to the pVM payload and/or
		 * to pvmfw using various boot protocols, e.g. in RSI with the
		 * Linux/x86 boot protocol or in RAX/RBX with Multiboot.
		 */
		kvm_rax_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_RAX]);
		kvm_rbx_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_RBX]);
		kvm_rcx_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_RCX]);
		kvm_rdx_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_RDX]);
		kvm_rsi_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_RSI]);
		kvm_rdi_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_RDI]);
		if (kvm_register_is_dirty(shared_vcpu, VCPU_REGS_RSP))
			kvm_rsp_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_RSP]);
		kvm_rbp_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_RBP]);
		kvm_r8_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_R8]);
		kvm_r9_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_R9]);
		kvm_r10_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_R10]);
		kvm_r11_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_R11]);
		kvm_r12_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_R12]);
		kvm_r13_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_R13]);
		kvm_r14_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_R14]);

		/* Reserve R15 for pKVM for future extensions. */
		kvm_r15_write(vcpu, 0);

		/*
		 * If the host VMM boots the pVM directly, without pvmfw,
		 * let it set the boot entry address.
		 */
		if (!pkvm_vcpu_is_pvmfw_bsp(vcpu) &&
		    kvm_register_is_dirty(shared_vcpu, VCPU_REGS_RIP))
			kvm_rip_write(vcpu, shared_vcpu->arch.regs[VCPU_REGS_RIP]);
	}

	pkvm_x86_call(update_vcpu_state_from_host)(vcpu);
}

static void share_vcpu_state_with_host(struct kvm_vcpu *vcpu)
{
	struct kvm_vcpu *shared_vcpu = to_pkvm_vcpu(vcpu)->shared_vcpu;

	if (!pkvm_is_protected_vcpu(vcpu)) {
		/* Make sure the RSP/RIP in private vcpu are up-to-date */
		if (!kvm_register_is_available(vcpu, VCPU_REGS_RSP))
			kvm_rsp_read(vcpu);
		if (!kvm_register_is_available(vcpu, VCPU_REGS_RIP))
			kvm_rip_read(vcpu);

		/*
		 * Share the npVM's GPRs/EFER/CR0/CR4 to the host which may be
		 * used by the host to handle vmexit.
		 *
		 * In particular, EFER/CR0/CR4 need to be shared when paging
		 * role bits are changed, to let the host update the guest
		 * stage-1 MMU info which is needed for instruction emulation
		 * for npVM.
		 */
		memcpy(shared_vcpu->arch.regs, vcpu->arch.regs,
		       NR_VCPU_REGS * sizeof(*vcpu->arch.regs));
		kvm_register_mark_available(shared_vcpu, VCPU_REGS_RSP);
		kvm_register_mark_available(shared_vcpu, VCPU_REGS_RIP);
		shared_vcpu->arch.cr0 = kvm_read_cr0(vcpu);
		kvm_register_mark_available(shared_vcpu, VCPU_EXREG_CR0);
		shared_vcpu->arch.cr4 = kvm_read_cr4(vcpu);
		kvm_register_mark_available(shared_vcpu, VCPU_EXREG_CR4);
		shared_vcpu->arch.efer = vcpu->arch.efer;

		/* Share the exception information to the host if there is any */
		if (vcpu->arch.exception.pending || vcpu->arch.exception.injected) {
			shared_vcpu->arch.exception = vcpu->arch.exception;
			kvm_clear_exception_queue(vcpu);
		}

		/* Share the debug registers to the host */
		memcpy(shared_vcpu->arch.db, vcpu->arch.db,
		       ARRAY_SIZE(vcpu->arch.db) * sizeof(*vcpu->arch.db));
		memcpy(shared_vcpu->arch.eff_db, vcpu->arch.eff_db,
		       ARRAY_SIZE(vcpu->arch.eff_db) * sizeof(*vcpu->arch.eff_db));
		shared_vcpu->arch.dr6 = vcpu->arch.dr6;
		shared_vcpu->arch.dr7 = vcpu->arch.dr7;
		shared_vcpu->arch.xcr0 = vcpu->arch.xcr0;
	}

	pkvm_x86_call(share_vcpu_state_with_host)(vcpu);
}

static int pkvm_vcpu_run(struct kvm_vcpu *vcpu, bool force_immediate_exit,
			 unsigned long *reqs_to_host)
{
	int ret;

	if (pkvm_is_protected_vcpu(vcpu)) {
		/*
		 * Pairs with smp_store_release() in pkvm_vm_finalize() and in
		 * pkvm_start_secondary_vcpu(), to make sure that pvmfw_load_addr,
		 * bsp_vcpu_id and sipi_vector are read after reading mp_state,
		 * so they are read with up-to-date values.
		 */
		if (smp_load_acquire(&vcpu->arch.mp_state) != KVM_MP_STATE_RUNNABLE)
			return -EPERM;

		if (unlikely(!kvm_vcpu_has_run(vcpu))) {
			if (pkvm_vcpu_is_pvmfw_bsp(vcpu))
				pkvm_vcpu_pvmfw_entry_init(vcpu);
			else if (!kvm_vcpu_is_reset_bsp(vcpu))
				pkvm_vcpu_ap_entry_init(vcpu);
		}
	}

	if (unlikely(!kvm_vcpu_has_run(vcpu)))
		pkvm_load_mmu_pgd(vcpu);

	/*
	 * Flush predictor when switching from host VM to pVM to prevent host VM
	 * from attacking pVM. This is not needed if switch from host VM to npVM
	 * as host VM is in npVM's trust boundary.
	 */
	if (static_branch_likely(&switch_vcpu_ibpb) && pkvm_is_protected_vcpu(vcpu))
		indirect_branch_prediction_barrier();

	update_vcpu_state_from_host(vcpu);

	ret = pkvm_vcpu_enter_guest(vcpu, force_immediate_exit, reqs_to_host);

	share_vcpu_state_with_host(vcpu);

	/*
	 * Flush predictor when switching from any guest VM (either npVM or pVM)
	 * to host VM, to prevent guest VM from attacking host VM.
	 */
	if (static_branch_likely(&switch_vcpu_ibpb))
		indirect_branch_prediction_barrier();

	return ret;
}

static int pkvm_complete_emulated_msr(struct kvm_vcpu *vcpu, int err)
{
	/*
	 * For the npVM, the host itself can complete the emulated MSR by either
	 * injecting the exception or skipping the instruction, according to the
	 * emulation result.
	 */
	if (!pkvm_is_protected_vcpu(vcpu))
		return -EOPNOTSUPP;

	/*
	 * For the pVM, just save the error code rather than completing the MSR
	 * emulation via kvm_x86_call(complete_emulated_msr), to prevent the
	 * host from injecting exception or skipping instructions as the host
	 * can use this PV interface at any scenario, e.g, not for MSR emulation
	 * at all. The pKVM hypervisor will decide how to complete the MSR
	 * emulation according to the last exit reason and this saved error code
	 * before entering the guest again.
	 */
	to_pkvm_vcpu(vcpu)->host_emulated_msr_err = err;
	return 1;
}

static int pkvm_protected_apic_has_interrupt(struct kvm_vcpu *vcpu, bool *has_intr)
{
	if (!lapic_in_kernel(vcpu) || !vcpu->arch.apic->guest_apic_protected)
		return -EOPNOTSUPP;

	*has_intr = (kvm_apic_has_interrupt(vcpu) != -1);

	return 0;
}

static int pkvm_vm_mmu_map(unsigned long gpa, unsigned long hpa,
			   unsigned long size, bool writable)
{
	struct kvm_vcpu *vcpu = this_cpu_read(cur_guest_vcpu);
	int ret;

	if (!vcpu)
		return -EINVAL;

	ret = pkvm_guest_mmu_refill_memcache(to_pkvm_vcpu(vcpu));
	if (ret)
		return ret;

	if (pkvm_is_protected_vcpu(vcpu)) {
		if (!writable)
			return -EPERM;

		ret = pkvm_host_donate_guest(vcpu, gpa, hpa, size);
	} else {
		ret = pkvm_host_share_guest(vcpu, gpa, hpa, size, writable);
	}

	return ret;
}

static int pkvm_vm_mmu_unmap(int vm_handle, unsigned long gpa,
			     unsigned long size)
{
	struct pkvm_vm *pkvm_vm;
	int ret;

	pkvm_vm = pkvm_get_vm(vm_handle);
	if (!pkvm_vm)
		return -EINVAL;

	if (pkvm_is_protected_vm(&pkvm_vm->kvm)) {
		ret = -EPERM;
		goto put_vm;
	}

	ret = pkvm_host_unshare_guest(&pkvm_vm->kvm, gpa, size);
put_vm:
	pkvm_put_vm(pkvm_vm);
	return ret;
}

static int pkvm_vm_mmu_age(int vm_handle, unsigned long gpa,
			   unsigned long size, bool mkold)
{
	struct pkvm_vm *pkvm_vm;
	int ret;

	pkvm_vm = pkvm_get_vm(vm_handle);
	if (!pkvm_vm)
		return -EINVAL;

	if (pkvm_is_protected_vm(&pkvm_vm->kvm)) {
		ret = -EPERM;
		goto put_vm;
	}

	ret = pkvm_host_test_clear_young_guest(&pkvm_vm->kvm, gpa, size, mkold);

	/*
	 * Do not flush TLB. It will be flushed by the MMU notifier in KVM
	 * if needed.
	 */
put_vm:
	pkvm_put_vm(pkvm_vm);
	return ret;
}

static int pkvm_vcpu_handle_host_hypercall(struct kvm_vcpu *hvcpu, enum pkvm_hc hc,
					   union pkvm_hc_data *in, union pkvm_hc_data *out)
{
	struct kvm_vcpu *vcpu = this_cpu_read(cur_guest_vcpu);
	int cpu = raw_smp_processor_id(), ret = 0;

	BUG_ON(hvcpu != this_cpu_read(host_vcpu));

	if (!vcpu)
		return -EINVAL;

	if (!is_guest_vcpu_accessible(vcpu, hc))
		return -EPERM;

	kvm_x86_call(vcpu_load)(vcpu, cpu);

	switch (hc) {
	case __pkvm__update_exception_bitmap:
		pkvm_update_exception_bitmap(vcpu);
		break;
	case __pkvm__set_efer:
		ret = kvm_x86_call(set_efer)(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__set_msr:
		ret = pkvm_set_msr(vcpu, pkvm_hc_input1(hvcpu),
				   pkvm_hc_input2(hvcpu));
		break;
	case __pkvm__get_msr:
		ret = kvm_msr_read(vcpu, pkvm_hc_input1(hvcpu), &out->get_msr.data);
		break;
	case __pkvm__cache_reg:
		ret = pkvm_cache_reg(vcpu, pkvm_hc_input1(hvcpu), out);
		break;
	case __pkvm__set_cr4:
		kvm_x86_call(set_cr4)(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__set_cr0:
		kvm_x86_call(set_cr0)(vcpu, pkvm_hc_input1(hvcpu));
		/*
		 * EFER will be updated if the vCPU enters to or exits from the
		 * long mode. Update the EFER for the host unconditionally. As
		 * the updating is just one line code which is simpler and has
		 * smaller overhead comparing with the case of doing the check
		 * first.
		 */
		to_pkvm_vcpu(vcpu)->shared_vcpu->arch.efer = vcpu->arch.efer;
		break;
	case __pkvm__set_rflags:
		kvm_x86_call(set_rflags)(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__get_rflags:
		out->get_rflags.data = kvm_x86_call(get_rflags)(vcpu);
		break;
	case __pkvm__set_dr7:
		pkvm_set_dr7(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__vcpu_reset:
		/*
		 * Only needs to support reset vCPU for INIT as the non-INIT reset
		 * is done by the pKVM hypervisor when creating this vCPU.
		 */
		kvm_vcpu_reset(vcpu, true);
		break;
	case __pkvm__set_segment:
		kvm_x86_call(set_segment)(vcpu, &in->set_segment.seg_val,
					  in->set_segment.seg);
		break;
	case __pkvm__get_segment:
		kvm_x86_call(get_segment)(vcpu, &out->get_segment.seg_val,
					  pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__get_segment_base:
		out->get_segment_base.data =
			kvm_x86_call(get_segment_base)(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__set_idt:
		kvm_x86_call(set_idt)(vcpu, &in->set_idt.desc);
		break;
	case __pkvm__get_idt:
		kvm_x86_call(get_idt)(vcpu, &out->get_idt.desc);
		break;
	case __pkvm__set_gdt:
		kvm_x86_call(set_gdt)(vcpu, &in->set_gdt.desc);
		break;
	case __pkvm__get_gdt:
		kvm_x86_call(get_gdt)(vcpu, &out->get_gdt.desc);
		break;
	case __pkvm__flush_tlb_all:
		kvm_x86_call(flush_tlb_all)(vcpu);
		break;
	case __pkvm__flush_tlb_current:
		kvm_x86_call(flush_tlb_current)(vcpu);
		break;
	case __pkvm__flush_tlb_gva:
		kvm_x86_call(flush_tlb_gva)(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__flush_tlb_guest:
		kvm_x86_call(flush_tlb_guest)(vcpu);
		break;
	case __pkvm__set_interrupt_shadow:
		kvm_x86_call(set_interrupt_shadow)(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__get_interrupt_shadow:
		out->get_interrupt_shadow.data = kvm_x86_call(get_interrupt_shadow)(vcpu);
		break;
	case __pkvm__enable_nmi_window:
		kvm_x86_call(enable_nmi_window)(vcpu);
		break;
	case __pkvm__enable_irq_window:
		kvm_x86_call(enable_irq_window)(vcpu);
		break;
	case __pkvm__interrupt_allowed:
		ret = pkvm_interrupt_allowed(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__nmi_allowed:
		ret = pkvm_nmi_allowed(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__get_nmi_mask:
		out->get_nmi_mask.data = kvm_x86_call(get_nmi_mask)(vcpu);
		break;
	case __pkvm__set_nmi_mask:
		kvm_x86_call(set_nmi_mask)(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__inject_irq:
		ret = pkvm_inject_irq(vcpu);
		break;
	case __pkvm__inject_nmi:
		ret = pkvm_inject_nmi(vcpu);
		break;
	case __pkvm__inject_exception:
		pkvm_inject_exception(vcpu);
		break;
	case __pkvm__cancel_injection:
		pkvm_cancel_injection(vcpu);
		break;
	case __pkvm__update_cr8_intercept:
		kvm_x86_call(update_cr8_intercept)(vcpu, pkvm_hc_input1(hvcpu),
						   pkvm_hc_input2(hvcpu));
		break;
	case __pkvm__set_virtual_apic_mode:
		ret = kvm_apic_set_base(vcpu, to_pkvm_vcpu(vcpu)->shared_vcpu->arch.apic_base,
					true);
		break;
	case __pkvm__refresh_apicv_exec_ctrl:
		ret = pkvm_refresh_apicv_exec_ctrl(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__load_eoi_exitmap:
		pkvm_load_eoi_exitmap(vcpu, pkvm_hc_input1(hvcpu), pkvm_hc_input2(hvcpu),
				      pkvm_hc_input3(hvcpu), pkvm_hc_input4(hvcpu));
		break;
	case __pkvm__hwapic_isr_update:
		ret = pkvm_hwapic_isr_update(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__vcpu_after_set_cpuid:
		ret = pkvm_vcpu_after_set_cpuid(vcpu, pkvm_host_gpa_to_phys(pkvm_hc_input1(hvcpu)),
						&out->vcpu_after_set_cpuid.memcache);
		break;
	case __pkvm__vcpu_add_fpstate:
		ret = pkvm_vcpu_add_fpstate(vcpu, pkvm_host_gpa_to_phys(pkvm_hc_input1(hvcpu)),
					    pkvm_hc_input2(hvcpu), &out->vcpu_add_fpstate.memcache);
		break;
	case __pkvm__write_tsc_offset:
		pkvm_write_tsc_offset(vcpu);
		break;
	case __pkvm__write_tsc_multiplier:
		ret = pkvm_write_tsc_multiplier(vcpu);
		break;
	case __pkvm__load_mmu_pgd:
		ret = pkvm_load_mmu_pgd(vcpu);
		break;
	case __pkvm__setup_mce:
		ret = kvm_vcpu_x86_setup_mce(vcpu, to_pkvm_vcpu(vcpu)->shared_vcpu->arch.mcg_cap);
		break;
	case __pkvm__vcpu_run:
		ret = pkvm_vcpu_run(vcpu, pkvm_hc_input1(hvcpu),
				    &out->vcpu_run.reqs_to_host);
		break;
	case __pkvm__complete_emulated_msr:
		ret = pkvm_complete_emulated_msr(vcpu, pkvm_hc_input1(hvcpu));
		break;
	case __pkvm__protected_apic_has_interrupt:
		ret = pkvm_protected_apic_has_interrupt(vcpu,
				&out->protected_apic_has_interrupt.has_intr);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	kvm_x86_call(vcpu_load)(hvcpu, cpu);
	return ret;
}

void pkvm_handle_host_hypercall(struct kvm_vcpu *vcpu)
{
	enum pkvm_hc hc = pkvm_hc(vcpu);
	/* Zero 'out' to prevent leaking stack data on error */
	union pkvm_hc_data in, out = {0};
	int ret = 0;

	pkvm_hc_get_input(vcpu, hc, &in);

	switch (hc) {
	case __pkvm__init:
		ret = pkvm_init((struct pkvm_mem_info *)pkvm_hc_input1(vcpu),
				pkvm_hc_input2(vcpu));
		break;
	case __pkvm__init_finalize:
		ret = pkvm_init_finalize();
		break;
	case __pkvm__reprivilege_cpu:
		ret = pkvm_reprivilege_vcpu(vcpu);
		break;
	case __pkvm__enable_vmexit_trace:
		pkvm_enable_vmexit_trace(pkvm_hc_input1(vcpu));
		break;
	case __pkvm__dump_vmexit_trace:
		ret = pkvm_dump_vmexit_trace(pkvm_host_gpa_to_phys(pkvm_hc_input1(vcpu)),
					     pkvm_hc_input2(vcpu), pkvm_hc_input3(vcpu));
		break;
	case __pkvm__check_processor_compatibility:
		ret = kvm_x86_call(check_processor_compatibility)();
		break;
	case __pkvm__enable_virtualization_cpu:
		ret = pkvm_enable_virtualization_cpu();
		break;
	case __pkvm__vm_init:
		ret = pkvm_vm_init(pkvm_host_gpa_to_phys(pkvm_hc_input1(vcpu)),
				   pkvm_host_gpa_to_phys(pkvm_hc_input2(vcpu)),
				   pkvm_host_gpa_to_phys(pkvm_hc_input3(vcpu)));
		break;
	case __pkvm__vm_finalize:
		ret = pkvm_vm_finalize(pkvm_hc_input1(vcpu));
		break;
	case __pkvm__vm_destroy:
		pkvm_vm_destroy(pkvm_hc_input1(vcpu), &out.vm_destroy.memcache);
		break;
	case __pkvm__vcpu_create:
		ret = pkvm_vcpu_create(pkvm_hc_input1(vcpu),
				       pkvm_host_gpa_to_phys(pkvm_hc_input2(vcpu)),
				       pkvm_host_gpa_to_phys(pkvm_hc_input3(vcpu)),
				       pkvm_host_gpa_to_phys(pkvm_hc_input4(vcpu)));
		break;
	case __pkvm__vcpu_free:
		ret = pkvm_vcpu_free(pkvm_hc_input1(vcpu), pkvm_hc_input2(vcpu),
				     &out.vcpu_free.memcache);
		break;
	case __pkvm__vcpu_load:
		ret = pkvm_vcpu_load(pkvm_hc_input1(vcpu),
				     pkvm_hc_input2(vcpu));
		break;
	case __pkvm__vcpu_put:
		ret = pkvm_vcpu_put(pkvm_hc_input1(vcpu),
				    pkvm_hc_input2(vcpu));
		break;
	case __pkvm__has_wbinvd_exit:
		ret = kvm_x86_call(has_wbinvd_exit)();
		break;
	case __pkvm__vm_mmu_map:
		ret = pkvm_vm_mmu_map(pkvm_hc_input1(vcpu),
				      pkvm_host_gpa_to_phys(pkvm_hc_input2(vcpu)),
				      pkvm_hc_input3(vcpu), pkvm_hc_input4(vcpu));
		break;
	case __pkvm__vm_mmu_unmap:
		ret = pkvm_vm_mmu_unmap(pkvm_hc_input1(vcpu),
					pkvm_hc_input2(vcpu),
					pkvm_hc_input3(vcpu));
		break;
	case __pkvm__vm_mmu_age:
		ret = pkvm_vm_mmu_age(pkvm_hc_input1(vcpu), pkvm_hc_input2(vcpu),
				      pkvm_hc_input3(vcpu), pkvm_hc_input4(vcpu));
		break;
#ifdef CONFIG_PKVM_INTEL
	case __pkvm__iommu_mmio_read:
		ret = pkvm_iommu_mmio_read(pkvm_hc_input1(vcpu),
					   pkvm_hc_input2(vcpu),
					   &out.iommu_mmio_read.val);
		break;
	case __pkvm__iommu_mmio_write:
		ret = pkvm_iommu_mmio_write(pkvm_hc_input1(vcpu),
					    pkvm_hc_input2(vcpu),
					    pkvm_hc_input3(vcpu));
		break;
	case __pkvm__iommu_iec_flush:
		ret = pkvm_iommu_iec_flush(pkvm_hc_input1(vcpu),
					   pkvm_hc_input2(vcpu),
					   pkvm_hc_input3(vcpu),
					   pkvm_hc_input4(vcpu));
		break;
	case __pkvm__iommu_clear_ce:
		ret = pkvm_iommu_clear_ce(&in.iommu_clear_ce.data);
		break;
	case __pkvm__iommu_set_lm_ce:
		ret = pkvm_iommu_set_lm_ce(&in.iommu_set_lm_ce.in,
					   &out.iommu_set_lm_ce.out);
		break;
	case __pkvm__iommu_set_sm_ce:
		ret = pkvm_iommu_set_sm_ce(&in.iommu_set_sm_ce.in,
					   &out.iommu_set_sm_ce.out);
		break;
	case __pkvm__iommu_pasid_setup_fl:
		ret = pkvm_iommu_pasid_setup_fl(&in.iommu_pasid_setup_fl.in,
						&out.iommu_pasid_setup_fl.out);
		break;
	case __pkvm__iommu_pasid_setup_sl:
		ret = pkvm_iommu_pasid_setup_sl(&in.iommu_pasid_setup_sl.in,
						&out.iommu_pasid_setup_sl.out);
		break;
	case __pkvm__iommu_pasid_teardown:
		ret = pkvm_iommu_pasid_teardown(&in.iommu_pasid_teardown.data);
		break;
	case __pkvm__iommu_alloc_domain:
		ret = pkvm_iommu_alloc_domain(&in.iommu_alloc_domain.data);
		break;
	case __pkvm__iommu_free_domain:
		ret = pkvm_iommu_free_domain(pkvm_hc_input1(vcpu),
					     &out.iommu_free_domain.memcache);
		break;
	case __pkvm__iommu_domain_map:
		ret = pkvm_iommu_domain_map(&in.iommu_domain_map.in,
					    &out.iommu_domain_map.out);
		break;
	case __pkvm__iommu_domain_unmap:
		ret = pkvm_iommu_domain_unmap(pkvm_hc_input1(vcpu),
					      pkvm_hc_input2(vcpu),
					      pkvm_hc_input3(vcpu));
		break;
#endif
	default:
		ret = pkvm_vcpu_handle_host_hypercall(vcpu, hc, &in, &out);
		break;
	}

	pkvm_hc_set_output(vcpu, hc, &out);

	pkvm_hc_set_ret(vcpu, ret);
}

void pkvm_kick_vcpu(struct kvm_vcpu *vcpu)
{
	/* No need to kick if a vcpu is already out of guest mode */
	if (kvm_vcpu_exiting_guest_mode(vcpu) != IN_GUEST_MODE)
		return;

	pkvm_lapic_send_init(READ_ONCE(vcpu->cpu));
}

void pkvm_wait_vcpu_kicked_out(struct kvm_vcpu *vcpu)
{
	int relax_iters = 0;
	u64 start;

	if (READ_ONCE(vcpu->mode) != EXITING_GUEST_MODE)
		return;

	start = rdtsc();
	do {
		cpu_relax();
		if (++relax_iters == 1000) {
			/*
			 * Bug the system if waiting for the remote CPU to ack
			 * the kick is taking longer than 1s. It may take
			 * microseconds, sometimes up to milliseconds (if the
			 * CPU needs to wake from a deeper low-power state) but
			 * should not take as long as a second.
			 */
			BUG_ON(tsc_khz && (((rdtsc() - start) / tsc_khz) > 1000));
			relax_iters = 0;
		}
	} while (READ_ONCE(vcpu->mode) == EXITING_GUEST_MODE);
}

int pkvm_x86_vendor_init(struct kvm_x86_init_ops *ops)
{
	int r;

	memset(&kvm_caps, 0, sizeof(kvm_caps));

	kvm_caps.supported_vm_types = BIT(KVM_X86_DEFAULT_VM) |
				      BIT(KVM_X86_PKVM_PROTECTED_VM);
	if (IS_ENABLED(CONFIG_KVM_SW_PROTECTED_VM))
		kvm_caps.supported_vm_types |= BIT(KVM_X86_SW_PROTECTED_VM);
	kvm_caps.supported_mce_cap = MCG_CTL_P | MCG_SER_P;

	if (boot_cpu_has(X86_FEATURE_XSAVE)) {
		kvm_host.xcr0 = xgetbv(XCR_XFEATURE_ENABLED_MASK);
		kvm_caps.supported_xcr0 = kvm_host.xcr0 & KVM_SUPPORTED_XCR0;
	}

	if (boot_cpu_has(X86_FEATURE_XSAVES)) {
		rdmsrq(MSR_IA32_XSS, kvm_host.xss);
		kvm_caps.supported_xss = kvm_host.xss & KVM_SUPPORTED_XSS;
	}

	kvm_caps.supported_quirks = KVM_X86_VALID_QUIRKS;
	kvm_caps.inapplicable_quirks = KVM_X86_CONDITIONAL_QUIRKS;

	rdmsrq_safe(MSR_EFER, &kvm_host.efer);

	if (boot_cpu_has(X86_FEATURE_ARCH_CAPABILITIES))
		rdmsrq(MSR_IA32_ARCH_CAPABILITIES, kvm_host.arch_capabilities);

	r = ops->hardware_setup();
	if (r)
		return r;

	memcpy(&kvm_x86_ops, ops->runtime_ops, sizeof(kvm_x86_ops));

	return 0;
}

struct pkvm_vm *pkvm_get_vm(int vm_handle)
{
	struct pkvm_vm_ref *pkvm_vm_ref;
	int idx = vm_handle;

	if (idx < 0 || idx >= MAX_PKVM_VMS)
		return NULL;

	idx = array_index_nospec(idx, MAX_PKVM_VMS);
	pkvm_vm_ref = &pkvm_vms_ref[idx];

	return atomic_inc_not_zero(&pkvm_vm_ref->refcount) ? pkvm_vm_ref->pkvm_vm : NULL;
}

void pkvm_put_vm(struct pkvm_vm *pkvm_vm)
{
	int idx = pkvm_vm->kvm.arch.pkvm.handle;
	struct pkvm_vm_ref *pkvm_vm_ref;

	if (idx < 0 || idx >= MAX_PKVM_VMS)
		return;

	pkvm_vm_ref = &pkvm_vms_ref[idx];

	WARN_ON(atomic_dec_if_positive(&pkvm_vm_ref->refcount) <= 0);
}

struct pkvm_vcpu *pkvm_get_vcpu(int vm_handle, int vcpu_handle)
{
	struct pkvm_vm *pkvm_vm;

	if (vcpu_handle < 0 || vcpu_handle >= KVM_MAX_VCPUS)
		return NULL;

	pkvm_vm = pkvm_get_vm(vm_handle);
	if (!pkvm_vm)
		return NULL;

	vcpu_handle = array_index_nospec(vcpu_handle, KVM_MAX_VCPUS);
	if (atomic_inc_not_zero(&pkvm_vm->vcpu_refs[vcpu_handle]))
		return pkvm_vm->vcpus[vcpu_handle];

	pkvm_put_vm(pkvm_vm);
	return NULL;
}

void pkvm_put_vcpu(struct pkvm_vcpu *pkvm_vcpu)
{
	int vcpu_handle = pkvm_vcpu->vcpu.arch.pkvm.handle;

	WARN_ON(atomic_dec_if_positive(&pkvm_vcpu->pkvm_vm->vcpu_refs[vcpu_handle]) <= 0);

	pkvm_put_vm(pkvm_vcpu->pkvm_vm);
}

unsigned long pkvm_pcpu_tss(int cpu)
{
#ifdef CONFIG_PKVM_X86_DEBUG
	return (unsigned long)&get_cpu_entry_area(cpu)->tss.x86_tss;
#else
	struct pkvm_pcpu *pcpu = per_cpu(phys_cpu, cpu);

	return (unsigned long)&pcpu->tss;
#endif
}

int pkvm_start_secondary_vcpu(struct kvm *kvm, u32 apic_id, unsigned long start_ip)
{
	struct pkvm_vm *pkvm_vm = to_pkvm(kvm);
	struct kvm_vcpu *vcpu;
	int ret = -EINVAL;
	int i;

	if (!pkvm_is_protected_vm(kvm))
		return -EINVAL;

	if (start_ip & ~0xff000)
		return -EFAULT;

	pkvm_spin_lock(&pkvm_vm->lock);

	for_each_pkvm_guest_vcpu(i, vcpu, pkvm_vm) {
		if (vcpu->vcpu_id != apic_id)
			continue;

		if (kvm_vcpu_is_reset_bsp(vcpu)) {
			ret = -EINVAL;
			break;
		}

		if (!lapic_in_kernel(vcpu)) {
			ret = -EOPNOTSUPP;
			break;
		}

		if (vcpu->arch.mp_state != KVM_MP_STATE_UNINITIALIZED) {
			ret = -EBUSY;
			break;
		}

		vcpu->arch.apic->sipi_vector = start_ip >> 12;
		/*
		 * Make sure to update sipi_vector before updating mp_state, i.e.
		 * before allowing the vCPU to run. Pairs with smp_load_acquire()
		 * in pkvm_vcpu_run().
		 */
		smp_store_release(&vcpu->arch.mp_state, KVM_MP_STATE_RUNNABLE);

		ret = 0;
		break;
	}

	pkvm_spin_unlock(&pkvm_vm->lock);

	return ret;
}

void pkvm_x86_ops_init(struct pkvm_x86_ops *ops)
{
	memcpy(&pkvm_x86_ops, ops, sizeof(struct pkvm_x86_ops));
}

int pkvm_walk_each_vm(pkvm_vm_func_t func, void *arg)
{
	struct pkvm_vm *vm;
	int i, ret = 0;

	pkvm_spin_lock(&pkvm_vms_lock);

	for (i = 0; i < MAX_PKVM_VMS; i++) {
		vm = pkvm_vms_ref[i].pkvm_vm;
		if (!vm)
			continue;
		ret = func(vm, arg);
		if (ret)
			break;
	}

	pkvm_spin_unlock(&pkvm_vms_lock);

	return ret;
}
