// SPDX-License-Identifier: GPL-2.0
#define pr_fmt(fmt) "pkvm_host: " fmt

#include <linux/kvm_host.h>
#include <asm/kvm_pkvm.h>
#include "pkvm_constants.h"
#include "posted_intr.h"
#include "trace.h"
#include "x86_ops.h"
#include "vmx.h"

static void pkvm_free_loaded_vmcs(struct loaded_vmcs *loaded_vmcs)
{
	if (!loaded_vmcs->vmcs)
		return;
	free_vmcs(loaded_vmcs->vmcs);
	loaded_vmcs->vmcs = NULL;
	if (loaded_vmcs->msr_bitmap)
		free_page((unsigned long)loaded_vmcs->msr_bitmap);
	WARN_ON(loaded_vmcs->shadow_vmcs != NULL);
}

static int pkvm_alloc_loaded_vmcs(struct loaded_vmcs *loaded_vmcs)
{
	loaded_vmcs->vmcs = alloc_vmcs(false);
	if (!loaded_vmcs->vmcs)
		return -ENOMEM;

	loaded_vmcs->shadow_vmcs = NULL;
	loaded_vmcs->cpu = -1;

	if (cpu_has_vmx_msr_bitmap()) {
		loaded_vmcs->msr_bitmap = (unsigned long *)
				__get_free_page(GFP_KERNEL_ACCOUNT);
		if (!loaded_vmcs->msr_bitmap)
			goto out_vmcs;
	}

	return 0;

out_vmcs:
	pkvm_free_loaded_vmcs(loaded_vmcs);
	return -ENOMEM;
}

static void __pkvm_vcpu_unload(void *arg)
{
	struct kvm_vcpu *vcpu = arg;
	struct vcpu_vmx *vmx;

	if (pkvm_hypercall(vcpu_put, vcpu->kvm->arch.pkvm.handle,
			   vcpu->arch.pkvm.handle))
		return;

	vmx = to_vmx(vcpu);
	vmx->loaded_vmcs->cpu = -1;
}

static void pkvm_vcpu_unload(struct kvm_vcpu *vcpu)
{
	int cpu = to_vmx(vcpu)->loaded_vmcs->cpu;

	if (cpu != -1)
		smp_call_function_single(cpu, __pkvm_vcpu_unload, vcpu, 1);
}

static bool pkvm_segment_cache_test(struct vcpu_vmx *vmx, int seg, int field)
{
	u32 mask = 1 << (seg * SEG_FIELD_NR + field);

	if (!kvm_register_is_available(&vmx->vcpu, VCPU_EXREG_SEGMENTS)) {
		kvm_register_mark_available(&vmx->vcpu, VCPU_EXREG_SEGMENTS);
		vmx->segment_cache.bitmask = 0;
	}

	return vmx->segment_cache.bitmask & mask;
}

static void pkvm_segment_cache_set(struct vcpu_vmx *vmx, int seg, int field)
{
	u32 mask = 1 << (seg * SEG_FIELD_NR + field);

	if (!kvm_register_is_available(&vmx->vcpu, VCPU_EXREG_SEGMENTS)) {
		kvm_register_mark_available(&vmx->vcpu, VCPU_EXREG_SEGMENTS);
		vmx->segment_cache.bitmask = 0;
	}

	/*
	 * Make sure the cached segment field value is updated before setting
	 * the bitmask. This code may get preempted by pkvm_get_cpl_no_cache()
	 * (on the same CPU), and we don't want pkvm_get_cpl_no_cache() to see
	 * the field marked in the bitmask as available while its cached value
	 * is still out of date.
	 */
	barrier();

	vmx->segment_cache.bitmask |= mask;
}

static void pkvm_cache_segment(struct vcpu_vmx *vmx, struct kvm_segment *var, int seg)
{
	struct kvm_save_segment *save = &vmx->segment_cache.seg[seg];

	save->selector = var->selector;
	pkvm_segment_cache_set(vmx, seg, SEG_FIELD_SEL);

	save->base = var->base;
	pkvm_segment_cache_set(vmx, seg, SEG_FIELD_BASE);

	save->limit = var->limit;
	pkvm_segment_cache_set(vmx, seg, SEG_FIELD_LIMIT);

	save->ar = (var->unusable << 16) |
		   (var->g << 15)	 |
		   (var->db << 14)	 |
		   (var->l << 13)	 |
		   (var->avl << 12)	 |
		   (var->present << 7)	 |
		   (var->dpl << 5)	 |
		   (var->s << 4)	 |
		   var->type;
	pkvm_segment_cache_set(vmx, seg, SEG_FIELD_AR);
}

static int pkvm_check_processor_compat(void)
{
	return pkvm_hypercall(check_processor_compatibility);
}

static int pkvm_enable_virtualization_cpu(void)
{
	return pkvm_hypercall(enable_virtualization_cpu);
}

static void pkvm_disable_virtualization_cpu(void)
{
	/*
	 * The pKVM hypervisor doesn't support disabling VMX for security
	 * reasons. This means that the CPU will remain in VMX non-root mode
	 * during rebooting if there was no hardware level reset. But pKVM
	 * does not support such warm reboots anyway.
	 */
}

/*
 * The kvm parameter can be NULL (module initialization, or invocation before
 * VM creation). Be sure to check the kvm parameter before using it.
 */
static bool pkvm_has_emulated_msr(struct kvm *kvm, u32 index)
{
	/* SMM mode is not supported by the pKVM hypervisor. */
	if (index == MSR_IA32_SMBASE)
		return false;

	if (!kvm)
		return vmx_has_emulated_msr(NULL, index);

	return pkvm_host_has_emulated_msr(kvm, index);
}

static int pkvm_vm_init(struct kvm *kvm)
{
	void *pkvm_vm;
	int ret;

	/*
	 * Some of struct kvm elements are initialized by the vmx_vm_init()
	 * which can be leveraged by the pKVM host as this initialization is
	 * simple and no VMX hardware involved.
	 */
	ret = vmx_vm_init(kvm);
	if (ret)
		return ret;

	pkvm_vm = alloc_pages_exact(PKVM_VMX_VM_SIZE, GFP_KERNEL_ACCOUNT);
	if (!pkvm_vm)
		return -ENOMEM;

	ret = pkvm_hypercall(vm_init, __pa(kvm), __pa(pkvm_vm));
	if (ret < 0)
		goto free_page;

	kvm->arch.pkvm.handle = ret;

	if (pkvm_is_protected_vm(kvm))
		kvm->arch.has_protected_state = true;

	return 0;

free_page:
	free_pages_exact(pkvm_vm, PKVM_VMX_VM_SIZE);
	return ret;
}

static void pkvm_vm_destroy(struct kvm *kvm)
{
	int vm_handle = kvm->arch.pkvm.handle;
	union pkvm_hc_data out;
	int ret;

	ret = pkvm_hypercall_out(vm_destroy, &out, vm_handle);
	if (ret) {
		pr_err("failed to destroy VM%d: %d\n", vm_handle, ret);
		return;
	}

	kvm_free_pkvm_memcache(&out.vm_destroy.memcache);

	vmx_vm_destroy(kvm);
}

static int pkvm_vcpu_create(struct kvm_vcpu *vcpu)
{
	size_t vcpu_size, fps_size;
	void *pkvm_vcpu, *fps;
	struct vcpu_vmx *vmx;
	int ret;

	vmx = to_vmx(vcpu);

	INIT_LIST_HEAD(&vmx->vt.pi_wakeup_list);

	ret = pkvm_alloc_loaded_vmcs(&vmx->vmcs01);
	if (ret < 0)
		return ret;

	vmx->loaded_vmcs = &vmx->vmcs01;
	vmx->loaded_vmcs->cpu = -1;

	vcpu_size = PKVM_VMX_VCPU_SIZE;
	if (pkvm_is_protected_vcpu(vcpu))
		vcpu_size += KVM_MCE_SIZE + KVM_MCI_CTL2_SIZE;
	if (lapic_in_kernel(vcpu))
		vcpu_size += sizeof(struct kvm_lapic);

	ret = -ENOMEM;
	pkvm_vcpu = alloc_pages_exact(vcpu_size, GFP_KERNEL_ACCOUNT);
	if (!pkvm_vcpu)
		goto free_vmcs;

	fps_size = pkvm_guest_initial_fpstate_size(vcpu->kvm);
	fps = alloc_pages_exact(fps_size, GFP_KERNEL_ACCOUNT);
	if (!fps)
		goto free_vcpu;

	ret = pkvm_hypercall(vcpu_create, vcpu->kvm->arch.pkvm.handle,
			     __pa(vcpu), __pa(pkvm_vcpu), __pa(fps));
	if (ret < 0)
		goto free_fpu;

	vcpu->arch.pkvm.handle = ret;

	return 0;

free_fpu:
	free_pages_exact(fps, fps_size);
free_vcpu:
	free_pages_exact(pkvm_vcpu, vcpu_size);
free_vmcs:
	pkvm_free_loaded_vmcs(vmx->loaded_vmcs);
	return ret;
}

static void pkvm_vcpu_free(struct kvm_vcpu *vcpu)
{
	int vm_handle = vcpu->kvm->arch.pkvm.handle;
	int vcpu_handle = vcpu->arch.pkvm.handle;
	struct vcpu_vmx *vmx = to_vmx(vcpu);
	union pkvm_hc_data out;
	int ret;

	pkvm_vcpu_unload(vcpu);

	ret = pkvm_hypercall_out(vcpu_free, &out, vm_handle, vcpu_handle);
	if (ret) {
		pr_err("failed to free VM%d vcpu%d: %d\n", vm_handle, vcpu_handle, ret);
		return;
	}

	kvm_free_pkvm_memcache(&out.vcpu_free.memcache);

	pkvm_free_loaded_vmcs(vmx->loaded_vmcs);
}

static void pkvm_vcpu_reset(struct kvm_vcpu *vcpu, bool init_event)
{
	struct vcpu_vmx *vmx = to_vmx(vcpu);

	/*
	 * TODO: The vcpu_reset PV interface will be disallowed for the pVM
	 * once its INIT event is handled inside the pKVM hypervisor. So should
	 * check `pkvm_is_protected_vcpu(vcpu)` rather than
	 * `vcpu->arch.guest_state_protected` once it is ready. See comments for
	 * `__pkvm__vcpu_reset` in pkvm_vcpu_handle_host_hypercall.
	 */
	if (!vcpu->arch.guest_state_protected && init_event)
		KVM_BUG_ON(pkvm_hypercall(vcpu_reset), vcpu->kvm);

	/*
	 * The host is responsible for injecting interrupts to the guest. The
	 * pi_desc is the key structure for the host to inject interrupts via
	 * the posted interrupt mechanism. Its physical address is used for the
	 * POSTED_INTR_DESC_ADDR in the VMCS by the pKVM hypervisor. Initialize
	 * the pi_desc when reset vcpu.
	 */
	vmx->vt.pi_desc.nv = POSTED_INTR_VECTOR;
	__pi_set_sn(&vmx->vt.pi_desc);

	/*
	 * The guest CR0/CR4 are managed by the pKVM hypervisor. When the host
	 * reads the guest CR0/CR4, it should get the up-to-date value from the
	 * pKVM. So make all bits in the CR0/CR4 as owned by the guest to
	 * indicate no bit is owned by the host.
	 */
	vcpu->arch.cr0_guest_owned_bits = ~0;
	vcpu->arch.cr4_guest_owned_bits = ~0;

	kvm_set_cr8(vcpu, 0);

	if (pkvm_is_protected_vcpu(vcpu)) {
		/*
		 * Emulating xapic mode will require the host to decode MMIO
		 * instruction which is not supported if the guest is a pVM as
		 * the pVM's CPU and memory state will be isolated. To avoid
		 * using xapic mode for a pVM, enable x2apic mode by default so
		 * that pVM will use MSR instructions to access lapic, which
		 * doesn't require decoding.
		 */
		u64 data = APIC_DEFAULT_PHYS_BASE | LAPIC_MODE_X2APIC |
			   (kvm_vcpu_is_reset_bsp(vcpu) ? MSR_IA32_APICBASE_BSP : 0);

		guest_cpu_cap_set(vcpu, X86_FEATURE_X2APIC);
		kvm_apic_set_base(vcpu, data, true);
	}
}

static void pkvm_vcpu_load(struct kvm_vcpu *vcpu, int cpu)
{
	struct vcpu_vmx *vmx = to_vmx(vcpu);
	bool already_loaded;

	already_loaded = vmx->loaded_vmcs->cpu == cpu;
	if (!already_loaded)
		pkvm_vcpu_unload(vcpu);

	if (KVM_BUG_ON(pkvm_hypercall(vcpu_load, vcpu->kvm->arch.pkvm.handle,
				      vcpu->arch.pkvm.handle), vcpu->kvm))
		return;

	if (!already_loaded)
		vmx->loaded_vmcs->cpu = cpu;

	vmx_vcpu_pi_load(vcpu, cpu);
}

static void pkvm_vcpu_put(struct kvm_vcpu *vcpu)
{
	vmx_vcpu_pi_put(vcpu);
}

static void pkvm_update_exception_bitmap(struct kvm_vcpu *vcpu)
{
	if (!pkvm_is_protected_vcpu(vcpu))
		KVM_BUG_ON(pkvm_hypercall(update_exception_bitmap), vcpu->kvm);
}

static int pkvm_get_feature_msr(u32 msr, u64 *data)
{
	switch (msr) {
	case KVM_FIRST_EMULATED_VMX_MSR ... KVM_LAST_EMULATED_VMX_MSR:
		return 1;
	default:
		return KVM_MSR_RET_UNSUPPORTED;
	}
}

static int pkvm_get_msr(struct kvm_vcpu *vcpu, struct msr_data *msr_info)
{
	if (pkvm_host_has_emulated_msr(vcpu->kvm, msr_info->index))
		return kvm_get_msr_common(vcpu, msr_info);

	if (!vcpu->arch.guest_state_protected) {
		union pkvm_hc_data out;
		int ret;

		ret = pkvm_hypercall_out(get_msr, &out, msr_info->index);
		if (!ret)
			msr_info->data = out.get_msr.data;

		return ret;
	}

	return -EPERM;
}

static int pkvm_set_msr(struct kvm_vcpu *vcpu, struct msr_data *msr_info)
{
	if (pkvm_host_has_emulated_msr(vcpu->kvm, msr_info->index))
		return kvm_set_msr_common(vcpu, msr_info);

	if (!vcpu->arch.guest_state_protected)
		return pkvm_hypercall(set_msr, msr_info->index, msr_info->data);

	return -EPERM;
}

static u64 pkvm_get_segment_base(struct kvm_vcpu *vcpu, int seg)
{
	struct vcpu_vmx *vmx = to_vmx(vcpu);
	union pkvm_hc_data out;
	ulong *p;

	if (vcpu->arch.guest_state_protected)
		return 0;

	p = &vmx->segment_cache.seg[seg].base;

	if (!pkvm_segment_cache_test(vmx, seg, SEG_FIELD_BASE)) {
		if (KVM_BUG_ON(pkvm_hypercall_out(get_segment_base, &out, seg), vcpu->kvm))
			return 0;

		*p = out.get_segment_base.data;
		pkvm_segment_cache_set(vmx, seg, SEG_FIELD_BASE);
	}

	return *p;
}

static void pkvm_get_segment(struct kvm_vcpu *vcpu, struct kvm_segment *var, int seg)
{
	struct vcpu_vmx *vmx = to_vmx(vcpu);
	struct kvm_save_segment *segment;
	u32 ar;

	if (vcpu->arch.guest_state_protected) {
		if (var)
			memset(var, 0, sizeof(*var));
		return;
	}

	if (!pkvm_segment_cache_test(vmx, seg, SEG_FIELD_SEL) ||
	    !pkvm_segment_cache_test(vmx, seg, SEG_FIELD_BASE) ||
	    !pkvm_segment_cache_test(vmx, seg, SEG_FIELD_LIMIT) ||
	    !pkvm_segment_cache_test(vmx, seg, SEG_FIELD_AR)) {
		union pkvm_hc_data out;

		if (KVM_BUG_ON(pkvm_hypercall_out(get_segment, &out, seg), vcpu->kvm))
			return;

		pkvm_cache_segment(vmx, &out.get_segment.seg_val, seg);
	}

	if (!var)
		return;

	segment = &vmx->segment_cache.seg[seg];
	var->selector = segment->selector;
	var->base = segment->base;
	var->limit = segment->limit;
	ar = segment->ar;
	var->unusable = (ar >> 16) & 1;
	var->type = ar & 15;
	var->s = (ar >> 4) & 1;
	var->dpl = (ar >> 5) & 3;
	/*
	 * Some userspaces do not preserve unusable property. Since usable
	 * segment has to be present according to VMX spec we can use present
	 * property to amend userspace bug by making unusable segment always
	 * nonpresent. vmx_segment_access_rights() already marks nonpresent
	 * segment as unusable.
	 */
	var->present = !var->unusable;
	var->avl = (ar >> 12) & 1;
	var->l = (ar >> 13) & 1;
	var->db = (ar >> 14) & 1;
	var->g = (ar >> 15) & 1;
}

static void pkvm_set_segment(struct kvm_vcpu *vcpu, struct kvm_segment *var, int seg)
{
	union pkvm_hc_data in = {
		.set_segment = {
			.seg_val = *var,
			.seg = seg,
		},
	};

	if (vcpu->arch.guest_state_protected)
		return;

	vmx_segment_cache_clear(to_vmx(vcpu));

	KVM_BUG_ON(pkvm_hypercall_in(set_segment, &in), vcpu->kvm);
}

static int pkvm_get_cpl(struct kvm_vcpu *vcpu)
{
	struct vcpu_vmx *vmx = to_vmx(vcpu);
	int seg = VCPU_SREG_SS;
	u32 ar;

	if (vcpu->arch.guest_state_protected)
		return 0;

	if (!pkvm_segment_cache_test(vmx, seg, SEG_FIELD_AR))
		pkvm_get_segment(vcpu, NULL, seg);

	ar = vmx->segment_cache.seg[seg].ar;
	return VMX_AR_DPL(ar);
}

static int pkvm_get_cpl_no_cache(struct kvm_vcpu *vcpu)
{
	struct vcpu_vmx *vmx = to_vmx(vcpu);
	int seg = VCPU_SREG_SS;
	union pkvm_hc_data out;

	if (vcpu->arch.guest_state_protected)
		return 0;

	/*
	 * Even though this is a no_cache version of get_cpl, still use the
	 * cached value if it is available, to avoid unnecessary calls to pKVM.
	 * It may be cached either by the pKVM hypervisor itself (when
	 * returning to the host after vcpu_run) or by the host after another
	 * get_segment call to pKVM (in such case, the barrier() in
	 * pkvm_segment_cache_set() makes sure that we are seeing the up-to-date
	 * value).
	 */
	if (likely(pkvm_segment_cache_test(vmx, seg, SEG_FIELD_AR)))
		return VMX_AR_DPL(vmx->segment_cache.seg[seg].ar);

	if (KVM_BUG_ON(pkvm_hypercall_out(get_segment, &out, seg), vcpu->kvm))
		return 0;

	return out.get_segment.seg_val.dpl;
}

static void pkvm_get_cs_db_l_bits(struct kvm_vcpu *vcpu, int *db, int *l)
{
	struct vcpu_vmx *vmx = to_vmx(vcpu);
	int seg = VCPU_SREG_CS;
	u32 ar;

	if (vcpu->arch.guest_state_protected) {
		*db = *l = 0;
		return;
	}

	if (!pkvm_segment_cache_test(vmx, seg, SEG_FIELD_AR))
		pkvm_get_segment(vcpu, NULL, seg);

	ar = vmx->segment_cache.seg[seg].ar;
	*db = (ar >> 14) & 1;
	*l = (ar >> 13) & 1;
}

static bool pkvm_is_valid_cr0(struct kvm_vcpu *vcpu, unsigned long cr0)
{
	return true;
}

static void pkvm_set_cr0(struct kvm_vcpu *vcpu, unsigned long cr0)
{
	/*
	 * Segment will updated by the pKVM hypervisor if the vCPU enters the
	 * long mode. Clears the segment cache unconditionally for below
	 * reasons:
	 * 1) the clearing is just one line of code which is simpler comparing
	 * with checking if the vCPU enters the long mode or not.
	 * 2) the overall overhead is smaller than checking if the vCPU enters
	 * the long mode or not. By clearing the segment cache unconditionally,
	 * the host will need to use the get_segment PV interface if the host
	 * wants to read the segment register after setting the CR0. So the
	 * additional overhead is by sending one more PV interface. But if check
	 * whether a vCPU will enter the long mode or not before clearing the
	 * segment cache, there is also one more PV interface overhead which is
	 * to send cache_reg PV interface to read CR0 PG bit first if the CR0 is
	 * not up-to-date. As it is unlikely that the host wants to read the
	 * segment register after setting the CR0 but likely the CR0 is not
	 * up-to-date before setting the CR0, it seems the overall overhead of
	 * clearing the segment cache unconditionally is smaller.
	 */
	vmx_segment_cache_clear(to_vmx(vcpu));

	if (!vcpu->arch.guest_state_protected)
		KVM_BUG_ON(pkvm_hypercall(set_cr0, cr0), vcpu->kvm);

	vcpu->arch.cr0 = cr0;
	kvm_register_mark_available(vcpu, VCPU_EXREG_CR0);
}

static bool pkvm_is_valid_cr4(struct kvm_vcpu *vcpu, unsigned long cr4)
{
	/* The pKVM doesn't support VMX feature. */
	return !(cr4 & X86_CR4_VMXE);
}

static void pkvm_set_cr4(struct kvm_vcpu *vcpu, unsigned long cr4)
{
	unsigned long old_cr4 = kvm_read_cr4(vcpu);
	if (!vcpu->arch.guest_state_protected)
		KVM_BUG_ON(pkvm_hypercall(set_cr4, cr4), vcpu->kvm);

	vcpu->arch.cr4 = cr4;
	kvm_register_mark_available(vcpu, VCPU_EXREG_CR4);
	if ((cr4 ^ old_cr4) & (X86_CR4_OSXSAVE | X86_CR4_PKE))
		vcpu->arch.cpuid_dynamic_bits_dirty = true;
}

static int pkvm_set_efer(struct kvm_vcpu *vcpu, u64 efer)
{
	int ret = -EINVAL;

	if (!vcpu->arch.guest_state_protected)
		ret = pkvm_hypercall(set_efer, efer);

	vcpu->arch.efer = efer;
	return ret;
}

static void pkvm_get_idt(struct kvm_vcpu *vcpu, struct desc_ptr *dt)
{
	union pkvm_hc_data data;

	if (vcpu->arch.guest_state_protected ||
	    KVM_BUG_ON(pkvm_hypercall_out(get_idt, &data), vcpu->kvm)) {
		memset(dt, 0, sizeof(*dt));
		return;
	}

	*dt = data.get_idt.desc;
}

static void pkvm_set_idt(struct kvm_vcpu *vcpu, struct desc_ptr *dt)
{
	union pkvm_hc_data data = {
		.set_idt.desc = *dt,
	};

	if (vcpu->arch.guest_state_protected)
		return;

	KVM_BUG_ON(pkvm_hypercall_in(set_idt, &data), vcpu->kvm);
}

static void pkvm_get_gdt(struct kvm_vcpu *vcpu, struct desc_ptr *dt)
{
	union pkvm_hc_data data;

	if (vcpu->arch.guest_state_protected ||
	    KVM_BUG_ON(pkvm_hypercall_out(get_gdt, &data), vcpu->kvm)) {
		memset(dt, 0, sizeof(*dt));
		return;
	}

	*dt = data.get_gdt.desc;
}

static void pkvm_set_gdt(struct kvm_vcpu *vcpu, struct desc_ptr *dt)
{
	union pkvm_hc_data data = {
		.set_gdt.desc = *dt,
	};

	if (vcpu->arch.guest_state_protected)
		return;

	KVM_BUG_ON(pkvm_hypercall_in(set_gdt, &data), vcpu->kvm);
}

static void pkvm_set_dr7(struct kvm_vcpu *vcpu, unsigned long val)
{
	if (!pkvm_is_protected_vcpu(vcpu))
		KVM_BUG_ON(pkvm_hypercall(set_dr7, val), vcpu->kvm);
}

static void pkvm_cache_reg(struct kvm_vcpu *vcpu, enum kvm_reg reg)
{
	union pkvm_hc_data out;

	if (pkvm_is_protected_vcpu(vcpu))
		return;

	if (KVM_BUG_ON(pkvm_hypercall_out(cache_reg, &out, reg), vcpu->kvm))
		return;

	kvm_register_mark_available(vcpu, reg);

	switch (reg) {
	case VCPU_REGS_RSP:
		vcpu->arch.regs[VCPU_REGS_RSP] = out.cache_reg.rsp;
		break;
	case VCPU_REGS_RIP:
		vcpu->arch.regs[VCPU_REGS_RIP] = out.cache_reg.rip;
		break;
	case VCPU_EXREG_PDPTR: {
		struct kvm_mmu *mmu = vcpu->arch.walk_mmu;

		mmu->pdptrs[0] = out.cache_reg.pdptrs[0];
		mmu->pdptrs[1] = out.cache_reg.pdptrs[1];
		mmu->pdptrs[2] = out.cache_reg.pdptrs[2];
		mmu->pdptrs[3] = out.cache_reg.pdptrs[3];
		break;
	}
	case VCPU_EXREG_CR0:
		vcpu->arch.cr0 = out.cache_reg.cr0;
		break;
	case VCPU_EXREG_CR3:
		vcpu->arch.cr3 = out.cache_reg.cr3;
		break;
	case VCPU_EXREG_CR4:
		vcpu->arch.cr4 = out.cache_reg.cr4;
		break;
	default:
		KVM_BUG_ON(1, vcpu->kvm);
		break;
	}
}

static unsigned long pkvm_get_rflags(struct kvm_vcpu *vcpu)
{
	struct vcpu_vmx *vmx = to_vmx(vcpu);

	if (!kvm_register_is_available(vcpu, VCPU_EXREG_RFLAGS)) {
		if (vcpu->arch.guest_state_protected) {
			vmx->rflags = 0;
		} else {
			union pkvm_hc_data out;

			if (KVM_BUG_ON(pkvm_hypercall_out(get_rflags, &out), vcpu->kvm))
				return 0;

			vmx->rflags = out.get_rflags.data;
		}
		kvm_register_mark_available(vcpu, VCPU_EXREG_RFLAGS);
	}

	return vmx->rflags;
}

static void pkvm_set_rflags(struct kvm_vcpu *vcpu, unsigned long rflags)
{
	to_vmx(vcpu)->rflags = rflags;
	if (!vcpu->arch.guest_state_protected)
		KVM_BUG_ON(pkvm_hypercall(set_rflags, rflags), vcpu->kvm);
	kvm_register_mark_available(vcpu, VCPU_EXREG_RFLAGS);
}

static bool pkvm_get_if_flag(struct kvm_vcpu *vcpu)
{
	return pkvm_get_rflags(vcpu) & X86_EFLAGS_IF;
}

static void pkvm_flush_tlb_all(struct kvm_vcpu *vcpu)
{
	if (!vcpu->arch.guest_state_protected)
		KVM_BUG_ON(pkvm_hypercall(flush_tlb_all), vcpu->kvm);
}

static void pkvm_flush_tlb_current(struct kvm_vcpu *vcpu)
{
	if (!vcpu->arch.guest_state_protected)
		KVM_BUG_ON(pkvm_hypercall(flush_tlb_current), vcpu->kvm);
}

static void pkvm_flush_tlb_gva(struct kvm_vcpu *vcpu, gva_t addr)
{
	if (!vcpu->arch.guest_state_protected)
		KVM_BUG_ON(pkvm_hypercall(flush_tlb_gva, addr), vcpu->kvm);
}

static void pkvm_flush_tlb_guest(struct kvm_vcpu *vcpu)
{
	if (!vcpu->arch.guest_state_protected)
		KVM_BUG_ON(pkvm_hypercall(flush_tlb_guest), vcpu->kvm);
}

static void pkvm_set_interrupt_shadow(struct kvm_vcpu *vcpu, int mask)
{
	if (!pkvm_is_protected_vcpu(vcpu))
		KVM_BUG_ON(pkvm_hypercall(set_interrupt_shadow, mask), vcpu->kvm);
}

static u32 pkvm_get_interrupt_shadow(struct kvm_vcpu *vcpu)
{
	union pkvm_hc_data out;

	if (pkvm_is_protected_vcpu(vcpu))
		return 0;

	KVM_BUG_ON(pkvm_hypercall_out(get_interrupt_shadow, &out), vcpu->kvm);

	return out.get_interrupt_shadow.data;
}

static void pkvm_inject_irq(struct kvm_vcpu *vcpu, bool reinjected)
{
	trace_kvm_inj_virq(vcpu->arch.interrupt.nr,
			   vcpu->arch.interrupt.soft, reinjected);

	++vcpu->stat.irq_injections;

	KVM_BUG_ON(pkvm_hypercall(inject_irq), vcpu->kvm);
}

static void pkvm_inject_nmi(struct kvm_vcpu *vcpu)
{
	++vcpu->stat.nmi_injections;

	KVM_BUG_ON(pkvm_hypercall(inject_nmi), vcpu->kvm);
}

static void pkvm_inject_exception(struct kvm_vcpu *vcpu)
{
	if (pkvm_is_protected_vcpu(vcpu))
		return;

	KVM_BUG_ON(pkvm_hypercall(inject_exception), vcpu->kvm);
}

static void pkvm_cancel_injection(struct kvm_vcpu *vcpu)
{
	vcpu->arch.nmi_injected = false;
	kvm_clear_exception_queue(vcpu);
	kvm_clear_interrupt_queue(vcpu);

	if (KVM_BUG_ON(pkvm_hypercall(cancel_injection), vcpu->kvm))
		return;

	if (vcpu->arch.nmi_injected ||
	    vcpu->arch.interrupt.injected ||
	    vcpu->arch.exception.injected)
		kvm_make_request(KVM_REQ_EVENT, vcpu);
}

static int pkvm_interrupt_allowed(struct kvm_vcpu *vcpu, bool for_injection)
{
	return pkvm_hypercall(interrupt_allowed, for_injection);
}

static int pkvm_nmi_allowed(struct kvm_vcpu *vcpu, bool for_injection)
{
	return pkvm_hypercall(nmi_allowed, for_injection);
}

static bool pkvm_get_nmi_mask(struct kvm_vcpu *vcpu)
{
	union pkvm_hc_data out;

	if (KVM_BUG_ON(pkvm_hypercall_out(get_nmi_mask, &out), vcpu->kvm))
		return false;

	return out.get_nmi_mask.data;
}

static void pkvm_set_nmi_mask(struct kvm_vcpu *vcpu, bool masked)
{
	if (!pkvm_is_protected_vcpu(vcpu))
		KVM_BUG_ON(pkvm_hypercall(set_nmi_mask, masked), vcpu->kvm);
}

static void pkvm_enable_nmi_window(struct kvm_vcpu *vcpu)
{
	KVM_BUG_ON(pkvm_hypercall(enable_nmi_window), vcpu->kvm);
}

static void pkvm_enable_irq_window(struct kvm_vcpu *vcpu)
{
	KVM_BUG_ON(pkvm_hypercall(enable_irq_window), vcpu->kvm);
}

static void pkvm_update_cr8_intercept(struct kvm_vcpu *vcpu, int tpr, int irr)
{
	KVM_BUG_ON(pkvm_hypercall(update_cr8_intercept, tpr, irr), vcpu->kvm);
}

static void pkvm_set_virtual_apic_mode(struct kvm_vcpu *vcpu)
{
	if (lapic_in_kernel(vcpu))
		KVM_BUG_ON(pkvm_hypercall(set_virtual_apic_mode), vcpu->kvm);
}

static void pkvm_refresh_apicv_exec_ctrl(struct kvm_vcpu *vcpu)
{
	if (lapic_in_kernel(vcpu))
		KVM_BUG_ON(pkvm_hypercall(refresh_apicv_exec_ctrl, vcpu->arch.apic->apicv_active),
			   vcpu->kvm);
}

static void pkvm_load_eoi_exitmap(struct kvm_vcpu *vcpu, u64 *eoi_exit_bitmap)
{
	if (kvm_vcpu_apicv_active(vcpu))
		KVM_BUG_ON(pkvm_hypercall(load_eoi_exitmap, eoi_exit_bitmap[0],
					  eoi_exit_bitmap[1], eoi_exit_bitmap[2],
					  eoi_exit_bitmap[3]),
			   vcpu->kvm);
}

#define VMX_REQUIRED_APICV_INHIBITS				\
	(BIT(APICV_INHIBIT_REASON_DISABLED) |			\
	 BIT(APICV_INHIBIT_REASON_ABSENT) |			\
	 BIT(APICV_INHIBIT_REASON_BLOCKIRQ) |			\
	 BIT(APICV_INHIBIT_REASON_PHYSICAL_ID_ALIASED) |	\
	 BIT(APICV_INHIBIT_REASON_APIC_ID_MODIFIED) |		\
	 BIT(APICV_INHIBIT_REASON_APIC_BASE_MODIFIED))

static void pkvm_hwapic_isr_update(struct kvm_vcpu *vcpu, int max_isr)
{
	KVM_BUG_ON(pkvm_hypercall(hwapic_isr_update, max_isr), vcpu->kvm);
}

static int pkvm_vcpu_realloc_fpstate(struct kvm_vcpu *vcpu)
{
	union pkvm_hc_data out;
	size_t fpsize;
	void *fps;
	int ret;

	fpsize = PAGE_ALIGN(vcpu->arch.guest_fpu.fpstate->size +
			    ALIGN(offsetof(struct fpstate, regs), 64));
	fps = alloc_pages_exact(fpsize, GFP_KERNEL_ACCOUNT);
	if (!fps)
		return -ENOMEM;

	ret = pkvm_hypercall_out(vcpu_add_fpstate, &out, __pa(fps), fpsize);
	if (KVM_BUG_ON(ret, vcpu->kvm))
		free_pages_exact(fps, fpsize);
	else
		kvm_free_pkvm_memcache(&out.vcpu_add_fpstate.memcache);

	return ret;
}

static void pkvm_vcpu_after_set_cpuid(struct kvm_vcpu *vcpu)
{
	struct kvm_cpuid_entry2 *e2 = vcpu->arch.cpuid_entries;
	int nent = vcpu->arch.cpuid_nent;
	union pkvm_hc_data out;
	void *entries;
	size_t size;

	if (vcpu->arch.guest_state_protected || !e2 || !nent)
		return;

	/*
	 * With exposing the FPU dynamic feature via the cpuid, the fpstate
	 * allocated when creating the vcpu may not be sufficient for the
	 * guest. As the pVM's FPU state is managed by the pKVM hypervisor
	 * while the npVM's FPU state is managed by the host, re-allocating the
	 * fpstate is only necessary for the pVM, and should be done before
	 * adding the new cpuid entries to the pKVM hypervisor.
	 */
	if ((vcpu->arch.guest_fpu.xfeatures & XFEATURE_MASK_USER_DYNAMIC) &&
	    pkvm_is_protected_vcpu(vcpu) &&
	    pkvm_vcpu_realloc_fpstate(vcpu))
		return;

	size = sizeof(struct kvm_cpuid_entry2) * nent;
	entries = alloc_pages_exact(size, GFP_KERNEL_ACCOUNT);
	if (!entries) {
		kvm_err("Failed to allocate cpuid pages for pKVM vcpu\n");
		return;
	}

	memcpy(entries, (void *)e2, size);

	if (KVM_BUG_ON(pkvm_hypercall_out(vcpu_after_set_cpuid, &out, __pa(entries)), vcpu->kvm))
		free_pages_exact(entries, size);
	else
		kvm_free_pkvm_memcache(&out.vcpu_after_set_cpuid.memcache);
}

static u64 pkvm_get_l2_tsc_offset(struct kvm_vcpu *vcpu)
{
	return 0;
}

static u64 pkvm_get_l2_tsc_multiplier(struct kvm_vcpu *vcpu)
{
	return kvm_caps.default_tsc_scaling_ratio;
}

static void pkvm_write_tsc_offset(struct kvm_vcpu *vcpu)
{
	KVM_BUG_ON(pkvm_hypercall(write_tsc_offset), vcpu->kvm);
}

static void pkvm_write_tsc_multiplier(struct kvm_vcpu *vcpu)
{
	KVM_BUG_ON(pkvm_hypercall(write_tsc_multiplier), vcpu->kvm);
}

static void pkvm_load_mmu_pgd(struct kvm_vcpu *vcpu, hpa_t root_hpa, int root_level)
{
	KVM_BUG_ON(pkvm_hypercall(load_mmu_pgd, root_hpa, root_level), vcpu->kvm);
}

static void pkvm_leave_nested(struct kvm_vcpu *vcpu) {}
static bool pkvm_nested_is_exception_vmexit(struct kvm_vcpu *vcpu, u8 vector,
					    u32 error_code)
{
	return false;
}
static int pkvm_check_nested_events(struct kvm_vcpu *vcpu) { return 0; }
static void pkvm_nested_triple_fault(struct kvm_vcpu *vcpu) {}
static bool pkvm_get_nested_state_pages(struct kvm_vcpu *vcpu) { return true; }
static int pkvm_nested_write_pml_buffer(struct kvm_vcpu *vcpu, gpa_t gpa) { return 0; }

static struct kvm_x86_nested_ops pkvm_nested_ops = {
	.leave_nested = pkvm_leave_nested,
	.is_exception_vmexit = pkvm_nested_is_exception_vmexit,
	.check_events = pkvm_check_nested_events,
	.triple_fault = pkvm_nested_triple_fault,
	.get_nested_state_pages = pkvm_get_nested_state_pages,
	.write_log_dirty = pkvm_nested_write_pml_buffer,
};

static void pkvm_setup_mce(struct kvm_vcpu *vcpu)
{
	KVM_BUG_ON(pkvm_hypercall(setup_mce), vcpu->kvm);
}

#ifdef CONFIG_KVM_SMM
static int pkvm_smi_allowed(struct kvm_vcpu *vcpu, bool for_injection)
{
	return false;
}

static int pkvm_enter_smm(struct kvm_vcpu *vcpu, union kvm_smram *smram)
{
	return -EOPNOTSUPP;
}

static int pkvm_leave_smm(struct kvm_vcpu *vcpu, const union kvm_smram *smram)
{
	return -EOPNOTSUPP;
}

static void pkvm_enable_smi_window(struct kvm_vcpu *vcpu) {}
#endif

static bool pkvm_apic_init_signal_blocked(struct kvm_vcpu *vcpu)
{
	/*
	 * The init signal will be blocked if the guest VM is emulating nested
	 * and in virtual VMX root mode. But as this is not a supported case by
	 * the pKVM hypervisor, the init signal should never be blocked for the
	 * guest VM.
	 */
	return false;
}

struct kvm_x86_ops pkvm_host_vt_x86_ops __initdata = {
	.name = KBUILD_MODNAME,

	.check_processor_compatibility = pkvm_check_processor_compat,

	.enable_virtualization_cpu = pkvm_enable_virtualization_cpu,
	.disable_virtualization_cpu = pkvm_disable_virtualization_cpu,
	.emergency_disable_virtualization_cpu = pkvm_disable_virtualization_cpu,

	.has_emulated_msr = pkvm_has_emulated_msr,

	.vm_size = sizeof(struct kvm_vmx),
	.vm_init = pkvm_vm_init,
	.vm_destroy = pkvm_vm_destroy,

	.vcpu_precreate = vmx_vcpu_precreate,
	.vcpu_create = pkvm_vcpu_create,
	.vcpu_free = pkvm_vcpu_free,
	.vcpu_reset = pkvm_vcpu_reset,

	.vcpu_load = pkvm_vcpu_load,
	.vcpu_put = pkvm_vcpu_put,

	.update_exception_bitmap = pkvm_update_exception_bitmap,
	.get_feature_msr = pkvm_get_feature_msr,
	.get_msr = pkvm_get_msr,
	.set_msr = pkvm_set_msr,
	.get_segment_base = pkvm_get_segment_base,
	.get_segment = pkvm_get_segment,
	.set_segment = pkvm_set_segment,
	.get_cpl = pkvm_get_cpl,
	.get_cpl_no_cache = pkvm_get_cpl_no_cache,
	.get_cs_db_l_bits = pkvm_get_cs_db_l_bits,
	.is_valid_cr0 = pkvm_is_valid_cr0,
	.set_cr0 = pkvm_set_cr0,
	.is_valid_cr4 = pkvm_is_valid_cr4,
	.set_cr4 = pkvm_set_cr4,
	.set_efer = pkvm_set_efer,
	.get_idt = pkvm_get_idt,
	.set_idt = pkvm_set_idt,
	.get_gdt = pkvm_get_gdt,
	.set_gdt = pkvm_set_gdt,
	.set_dr7 = pkvm_set_dr7,
	.cache_reg = pkvm_cache_reg,
	.get_rflags = pkvm_get_rflags,
	.set_rflags = pkvm_set_rflags,
	.get_if_flag = pkvm_get_if_flag,

	.flush_tlb_all = pkvm_flush_tlb_all,
	.flush_tlb_current = pkvm_flush_tlb_current,
	.flush_tlb_gva = pkvm_flush_tlb_gva,
	.flush_tlb_guest = pkvm_flush_tlb_guest,

	.set_interrupt_shadow = pkvm_set_interrupt_shadow,
	.get_interrupt_shadow = pkvm_get_interrupt_shadow,
	.inject_irq = pkvm_inject_irq,
	.inject_nmi = pkvm_inject_nmi,
	.inject_exception = pkvm_inject_exception,
	.cancel_injection = pkvm_cancel_injection,
	.interrupt_allowed = pkvm_interrupt_allowed,
	.nmi_allowed = pkvm_nmi_allowed,
	.get_nmi_mask = pkvm_get_nmi_mask,
	.set_nmi_mask = pkvm_set_nmi_mask,
	.enable_nmi_window = pkvm_enable_nmi_window,
	.enable_irq_window = pkvm_enable_irq_window,
	.update_cr8_intercept = pkvm_update_cr8_intercept,

	.x2apic_icr_is_split = false,
	.set_virtual_apic_mode = pkvm_set_virtual_apic_mode,
	.refresh_apicv_exec_ctrl = pkvm_refresh_apicv_exec_ctrl,
	.load_eoi_exitmap = pkvm_load_eoi_exitmap,
	.apicv_pre_state_restore = pi_apicv_pre_state_restore,
	.required_apicv_inhibits = VMX_REQUIRED_APICV_INHIBITS,
	.hwapic_isr_update = pkvm_hwapic_isr_update,
	.sync_pir_to_irr = vmx_sync_pir_to_irr,
	.deliver_interrupt = vmx_deliver_interrupt,
	.dy_apicv_has_pending_interrupt = pi_has_pending_interrupt,

	.vcpu_after_set_cpuid = pkvm_vcpu_after_set_cpuid,

	.get_l2_tsc_offset = pkvm_get_l2_tsc_offset,
	.get_l2_tsc_multiplier = pkvm_get_l2_tsc_multiplier,
	.write_tsc_offset = pkvm_write_tsc_offset,
	.write_tsc_multiplier = pkvm_write_tsc_multiplier,

	.load_mmu_pgd = pkvm_load_mmu_pgd,

	.nested_ops = &pkvm_nested_ops,

	.pi_update_irte = vmx_pi_update_irte,
	.pi_start_bypass = vmx_pi_start_bypass,

	.setup_mce = pkvm_setup_mce,

#ifdef CONFIG_KVM_SMM
	.smi_allowed = pkvm_smi_allowed,
	.enter_smm = pkvm_enter_smm,
	.leave_smm = pkvm_leave_smm,
	.enable_smi_window = pkvm_enable_smi_window,
#endif

	.apic_init_signal_blocked = pkvm_apic_init_signal_blocked,

	.vcpu_deliver_sipi_vector = kvm_vcpu_deliver_sipi_vector,
};

bool pkvm_interrupt_blocked(struct kvm_vcpu *vcpu)
{
	return (pkvm_interrupt_allowed(vcpu, false) <= 0);
}
