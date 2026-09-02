// SPDX-License-Identifier: GPL-2.0
#define pr_fmt(fmt) "pkvm_host: " fmt

#include <linux/kvm_host.h>
#include <asm/kvm_pkvm.h>
#include "pkvm_constants.h"
#include "posted_intr.h"
#include "trace.h"
#include "x86_ops.h"
#include "vmx.h"

static int pkvm_complete_emulated_msr(struct kvm_vcpu *vcpu, int err);
static unsigned short has_wbinvd_exit = USHRT_MAX;

static bool pkvm_has_vmx_wbinvd_exit(void);
static int vmx_get_msr_imm_reg(struct kvm_vcpu *vcpu);

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

static fastpath_t pkvm_exit_handlers_fastpath(struct kvm_vcpu *vcpu)
{
	switch (vmx_get_exit_reason(vcpu).basic) {
	case EXIT_REASON_PREEMPTION_TIMER:
		kvm_lapic_expired_hv_timer(vcpu);
		return EXIT_FASTPATH_REENTER_GUEST;
	case EXIT_REASON_MSR_WRITE:
		return handle_fastpath_wrmsr(vcpu);
	case EXIT_REASON_MSR_WRITE_IMM:
		return handle_fastpath_wrmsr_imm(vcpu, vmx_get_exit_qual(vcpu),
						 vmx_get_msr_imm_reg(vcpu));
	default:
		return EXIT_FASTPATH_NONE;
	}
}

static int handle_exception_nmi(struct kvm_vcpu *vcpu)
{
	struct vcpu_vmx *vmx = to_vmx(vcpu);
	struct kvm_run *kvm_run = vcpu->run;
	u32 intr_info, ex_no, error_code;
	unsigned long dr6;
	u32 vect_info;

	vect_info = vmx->idt_vectoring_info;
	intr_info = vmx_get_intr_info(vcpu);

	/*
	 * Machine checks are handled by handle_exit_irqoff operation, or by
	 * pkvm_vcpu_run() if a #MC occurs on VM-Entry.  NMIs are handled by
	 * pkvm_vcpu_run().
	 */
	if (is_machine_check(intr_info) || is_nmi(intr_info))
		return 1;

	if (is_invalid_opcode(intr_info))
		return handle_ud(vcpu);

	error_code = 0;
	if (intr_info & INTR_INFO_DELIVER_CODE_MASK)
		error_code = vmx->error_code;

	/*
	 * The #PF with PFEC.RSVD = 1 indicates the guest is accessing
	 * MMIO, it is better to report an internal error.
	 * See the comments in __pkvm_handle_exit.
	 */
	if ((vect_info & VECTORING_INFO_VALID_MASK) &&
	    !(is_page_fault(intr_info) && !(error_code & PFERR_RSVD_MASK))) {
		vcpu->run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		vcpu->run->internal.suberror = KVM_INTERNAL_ERROR_SIMUL_EX;
		vcpu->run->internal.ndata = 4;
		vcpu->run->internal.data[0] = vect_info;
		vcpu->run->internal.data[1] = intr_info;
		vcpu->run->internal.data[2] = error_code;
		vcpu->run->internal.data[3] = vcpu->arch.last_vmentry_cpu;
		return 0;
	}

	ex_no = intr_info & INTR_INFO_VECTOR_MASK;
	switch (ex_no) {
	case DB_VECTOR:
		dr6 = vmx_get_exit_qual(vcpu);
		/*
		 * Only handle KVM_GUESTDBG_SINGLESTEP or KVM_GUESTDBG_USE_HW_BP
		 * as the opposite cases are already handled by the pKVM
		 * hypervisor.
		 */
		if (WARN_ON_ONCE(!(vcpu->guest_debug &
			      (KVM_GUESTDBG_SINGLESTEP | KVM_GUESTDBG_USE_HW_BP))))
			break;
		kvm_run->debug.arch.dr6 = dr6 | DR6_ACTIVE_LOW;
		kvm_run->debug.arch.dr7 = vcpu->arch.dr7;
		fallthrough;
	case BP_VECTOR:
		kvm_run->exit_reason = KVM_EXIT_DEBUG;
		kvm_run->debug.arch.pc = kvm_get_linear_rip(vcpu);
		kvm_run->debug.arch.exception = ex_no;
		break;
	case AC_VECTOR:
		/*
		 * The #AC exception is injected by the pKVM hypervisor if it is
		 * required. And if #AC is not injected, handle split lock in
		 * the host. Depending on detection mode this will either warn
		 * and disable split lock detection for this task or force
		 * SIGBUS on it.
		 */
		if (handle_guest_split_lock(kvm_rip_read(vcpu)))
			return 1;
		fallthrough;
	default:
		pr_warn("pkvm_host: Unsupported exception_nmi: intr_info 0x%x\n", intr_info);
		kvm_run->exit_reason = KVM_EXIT_EXCEPTION;
		kvm_run->ex.exception = ex_no;
		kvm_run->ex.error_code = error_code;
		break;
	}

	return 0;
}

static __always_inline int handle_external_interrupt(struct kvm_vcpu *vcpu)
{
	++vcpu->stat.irq_exits;
	return 1;
}

static int handle_triple_fault(struct kvm_vcpu *vcpu)
{
	vcpu->run->exit_reason = KVM_EXIT_SHUTDOWN;
	vcpu->mmio_needed = 0;
	return 0;
}

static int handle_nmi_window(struct kvm_vcpu *vcpu)
{
	++vcpu->stat.nmi_window_exits;
	kvm_make_request(KVM_REQ_EVENT, vcpu);

	return 1;
}

/*
 * This function is a 1:1 copy of handle_io in vmx.c.
 * TODO: Check whether this requires updates whenever handle_io in vmx.c is
 * updated in the future.
 */
static int handle_io(struct kvm_vcpu *vcpu)
{
	unsigned long exit_qualification;
	int size, in, string;
	unsigned int port;

	exit_qualification = vmx_get_exit_qual(vcpu);
	string = (exit_qualification & 16) != 0;

	++vcpu->stat.io_exits;

	if (string)
		return kvm_emulate_instruction(vcpu, 0);

	port = exit_qualification >> 16;
	size = (exit_qualification & 7) + 1;
	in = (exit_qualification & 8) != 0;

	return kvm_fast_pio(vcpu, size, port, in);
}

static int handle_dr(struct kvm_vcpu *vcpu)
{
	if ((vcpu->arch.guest_debug_dr7 & DR7_GD) &&
	    (vcpu->guest_debug & KVM_GUESTDBG_USE_HW_BP)) {
		vcpu->run->debug.arch.dr6 = DR6_BD | DR6_ACTIVE_LOW;
		vcpu->run->debug.arch.dr7 = vcpu->arch.guest_debug_dr7;
		vcpu->run->debug.arch.pc = kvm_get_linear_rip(vcpu);
		vcpu->run->debug.arch.exception = DB_VECTOR;
		vcpu->run->exit_reason = KVM_EXIT_DEBUG;

		return 0;
	}

	return 1;
}

static int handle_rdmsr(struct kvm_vcpu *vcpu)
{
	if (pkvm_host_has_emulated_msr(vcpu->kvm, kvm_rcx_read(vcpu)))
		return kvm_emulate_rdmsr(vcpu);

	return pkvm_complete_emulated_msr(vcpu, 1);
}

static int handle_wrmsr(struct kvm_vcpu *vcpu)
{
	if (pkvm_host_has_emulated_msr(vcpu->kvm, kvm_rcx_read(vcpu)))
		return kvm_emulate_wrmsr(vcpu);

	return pkvm_complete_emulated_msr(vcpu, 1);
}

static int handle_interrupt_window(struct kvm_vcpu *vcpu)
{
	kvm_make_request(KVM_REQ_EVENT, vcpu);

	++vcpu->stat.irq_window_exits;
	return 1;
}

static int handle_halt(struct kvm_vcpu *vcpu)
{
	return kvm_emulate_halt_noskip(vcpu);
}

/*
 * This function is a 1:1 copy of handle_tpr_below_threshold in vmx.c.
 * TODO: Check whether this requires updates whenever handle_tpr_below_threshold
 * in vmx.c is updated in the future.
 */
static int handle_tpr_below_threshold(struct kvm_vcpu *vcpu)
{
	kvm_apic_update_ppr(vcpu);
	return 1;
}

/*
 * This function is a 1:1 copy of handle_apic_write in vmx.c.
 * TODO: Check whether this requires updates whenever handle_apic_write in vmx.c
 * is updated in the future.
 */
static int handle_apic_write(struct kvm_vcpu *vcpu)
{
	unsigned long exit_qualification = vmx_get_exit_qual(vcpu);

	/*
	 * APIC-write VM-Exit is trap-like, KVM doesn't need to advance RIP and
	 * hardware has done any necessary aliasing, offset adjustments, etc...
	 * for the access.  I.e. the correct value has already been  written to
	 * the vAPIC page for the correct 16-byte chunk.  KVM needs only to
	 * retrieve the register value and emulate the access.
	 */
	u32 offset = exit_qualification & 0xff0;

	kvm_apic_write_nodecode(vcpu, offset);
	return 1;
}

/*
 * This function is a 1:1 copy of handle_apic_eoi_induced in vmx.c.
 * TODO: Check whether this requires updates whenever handle_apic_eoi_induced in
 * vmx.c is updated in the future.
 */
static int handle_apic_eoi_induced(struct kvm_vcpu *vcpu)
{
	unsigned long exit_qualification = vmx_get_exit_qual(vcpu);
	int vector = exit_qualification & 0xff;

	/* EOI-induced VM exit is trap-like and thus no need to adjust IP */
	kvm_apic_set_eoi_accelerated(vcpu, vector);
	return 1;
}

static int handle_task_switch(struct kvm_vcpu *vcpu)
{
	struct vcpu_vmx *vmx = to_vmx(vcpu);
	unsigned long exit_qualification;
	bool has_error_code = false;
	u32 error_code = 0;
	u16 tss_selector;
	int reason, type, idt_v, idt_index;

	idt_v = (vmx->idt_vectoring_info & VECTORING_INFO_VALID_MASK);
	idt_index = (vmx->idt_vectoring_info & VECTORING_INFO_VECTOR_MASK);
	type = (vmx->idt_vectoring_info & VECTORING_INFO_TYPE_MASK);

	exit_qualification = vmx_get_exit_qual(vcpu);

	reason = (u32)exit_qualification >> 30;
	if (reason == TASK_SWITCH_GATE && idt_v) {
		switch (type) {
		case INTR_TYPE_NMI_INTR:
			vcpu->arch.nmi_injected = false;
			break;
		case INTR_TYPE_EXT_INTR:
		case INTR_TYPE_SOFT_INTR:
			kvm_clear_interrupt_queue(vcpu);
			break;
		case INTR_TYPE_HARD_EXCEPTION:
			if (vmx->idt_vectoring_info &
			    VECTORING_INFO_DELIVER_CODE_MASK) {
				has_error_code = true;
				error_code = vmx->error_code;
			}
			fallthrough;
		case INTR_TYPE_SOFT_EXCEPTION:
			kvm_clear_exception_queue(vcpu);
			break;
		default:
			break;
		}
	}
	tss_selector = exit_qualification;

	/*
	 * TODO: What about debug traps on tss switch?
	 *       Are we supposed to inject them and update dr6?
	 */
	return kvm_task_switch(vcpu, tss_selector,
			       type == INTR_TYPE_SOFT_INTR ? idt_index : -1,
			       reason, has_error_code, error_code);
}

static int handle_machine_check(struct kvm_vcpu *vcpu)
{
	/* handled by pkvm_vcpu_run() */
	return 1;
}

static int handle_ept_violation(struct kvm_vcpu *vcpu)
{
	unsigned long exit_qualification = vmx_get_exit_qual(vcpu);
	gpa_t gpa;

	gpa = to_vmx(vcpu)->exit_gpa;

	trace_kvm_page_fault(vcpu, gpa, exit_qualification);

	return __vmx_handle_ept_violation(vcpu, gpa, exit_qualification);
}

/*
 * Indicate a busy-waiting vcpu in spinlock. We do not enable the PAUSE
 * exiting, so only get here on cpu with PAUSE-Loop-Exiting.
 */
static int handle_pause(struct kvm_vcpu *vcpu)
{
	/*
	 * Intel sdm vol3 ch-25.1.3 says: The "PAUSE-loop exiting"
	 * VM-execution control is ignored if CPL > 0. OTOH, KVM
	 * never set PAUSE_EXITING and just set PLE if supported,
	 * so the vcpu must be CPL=0 if it gets a PAUSE exit.
	 */
	kvm_vcpu_on_spin(vcpu, true);
	return kvm_skip_emulated_instruction(vcpu);
}

static int handle_bus_lock_vmexit(struct kvm_vcpu *vcpu)
{
	/*
	 * The bus_lock_detected flag is set when got the vmexit reason from the
	 * pKVM hypervisor. Nothing to do here.
	 */
	return 1;
}

static int handle_notify(struct kvm_vcpu *vcpu)
{
	unsigned long exit_qual = vmx_get_exit_qual(vcpu);
	bool context_invalid = exit_qual & NOTIFY_VM_CONTEXT_INVALID;

	++vcpu->stat.notify_window_exits;

	if (vcpu->kvm->arch.notify_vmexit_flags & KVM_X86_NOTIFY_VMEXIT_USER ||
	    context_invalid) {
		vcpu->run->exit_reason = KVM_EXIT_NOTIFY;
		vcpu->run->notify.flags = context_invalid ?
					  KVM_NOTIFY_CONTEXT_INVALID : 0;
		return 0;
	}

	return 1;
}

static int vmx_get_msr_imm_reg(struct kvm_vcpu *vcpu)
{
	return vmx_get_instr_info_reg(to_vmx(vcpu)->instr_info);
}

static int handle_rdmsr_imm(struct kvm_vcpu *vcpu)
{
	if (pkvm_host_has_emulated_msr(vcpu->kvm, vmx_get_exit_qual(vcpu)))
		return kvm_emulate_rdmsr_imm(vcpu, vmx_get_exit_qual(vcpu),
					     vmx_get_msr_imm_reg(vcpu));

	return pkvm_complete_emulated_msr(vcpu, 1);
}

static int handle_wrmsr_imm(struct kvm_vcpu *vcpu)
{
	if (pkvm_host_has_emulated_msr(vcpu->kvm, vmx_get_exit_qual(vcpu)))
		return kvm_emulate_wrmsr_imm(vcpu, vmx_get_exit_qual(vcpu),
					     vmx_get_msr_imm_reg(vcpu));

	return pkvm_complete_emulated_msr(vcpu, 1);
}

/*
 * The exit handlers return 1 if the exit was handled fully and guest execution
 * may resume.  Otherwise they set the kvm_run parameter to indicate what needs
 * to be done to userspace and return 0.
 */
static int (*pkvm_vmx_exit_handlers[])(struct kvm_vcpu *vcpu) = {
	[EXIT_REASON_EXCEPTION_NMI]           = handle_exception_nmi,
	[EXIT_REASON_EXTERNAL_INTERRUPT]      = handle_external_interrupt,
	[EXIT_REASON_TRIPLE_FAULT]            = handle_triple_fault,
	[EXIT_REASON_NMI_WINDOW]	      = handle_nmi_window,
	[EXIT_REASON_IO_INSTRUCTION]          = handle_io,
	[EXIT_REASON_DR_ACCESS]               = handle_dr,
	[EXIT_REASON_MSR_READ]                = handle_rdmsr,
	[EXIT_REASON_MSR_WRITE]               = handle_wrmsr,
	[EXIT_REASON_INTERRUPT_WINDOW]        = handle_interrupt_window,
	[EXIT_REASON_HLT]                     = handle_halt,
	[EXIT_REASON_VMCALL]                  = kvm_emulate_hypercall,
	[EXIT_REASON_TPR_BELOW_THRESHOLD]     = handle_tpr_below_threshold,
	[EXIT_REASON_APIC_WRITE]              = handle_apic_write,
	[EXIT_REASON_EOI_INDUCED]             = handle_apic_eoi_induced,
	[EXIT_REASON_WBINVD]                  = kvm_emulate_wbinvd,
	[EXIT_REASON_TASK_SWITCH]             = handle_task_switch,
	[EXIT_REASON_MCE_DURING_VMENTRY]      = handle_machine_check,
	[EXIT_REASON_EPT_VIOLATION]	      = handle_ept_violation,
	[EXIT_REASON_PAUSE_INSTRUCTION]       = handle_pause,
	[EXIT_REASON_BUS_LOCK]                = handle_bus_lock_vmexit,
	[EXIT_REASON_NOTIFY]		      = handle_notify,
	[EXIT_REASON_MSR_READ_IMM]            = handle_rdmsr_imm,
	[EXIT_REASON_MSR_WRITE_IMM]           = handle_wrmsr_imm,
};

static const int pkvm_vmx_max_exit_handlers = ARRAY_SIZE(pkvm_vmx_exit_handlers);

static int __pkvm_handle_exit(struct kvm_vcpu *vcpu, fastpath_t exit_fastpath)
{
	union vmx_exit_reason exit_reason = vmx_get_exit_reason(vcpu);
	struct vcpu_vmx *vmx = to_vmx(vcpu);
	u16 exit_handler_index;
	u32 vectoring_info;

	if (exit_reason.failed_vmentry) {
		vcpu->run->exit_reason = KVM_EXIT_FAIL_ENTRY;
		vcpu->run->fail_entry.hardware_entry_failure_reason
			= exit_reason.full;
		vcpu->run->fail_entry.cpu = vcpu->arch.last_vmentry_cpu;
		return 0;
	}

	if (unlikely(vmx->fail)) {
		vcpu->run->exit_reason = KVM_EXIT_FAIL_ENTRY;
		vcpu->run->fail_entry.hardware_entry_failure_reason
			= vmx->error_code;
		vcpu->run->fail_entry.cpu = vcpu->arch.last_vmentry_cpu;
		return 0;
	}

	vectoring_info = vmx->idt_vectoring_info;
	if ((vectoring_info & VECTORING_INFO_VALID_MASK) &&
	    (exit_reason.basic != EXIT_REASON_EXCEPTION_NMI &&
	     exit_reason.basic != EXIT_REASON_EPT_VIOLATION &&
	     exit_reason.basic != EXIT_REASON_PML_FULL &&
	     exit_reason.basic != EXIT_REASON_APIC_ACCESS &&
	     exit_reason.basic != EXIT_REASON_TASK_SWITCH &&
	     exit_reason.basic != EXIT_REASON_NOTIFY &&
	     exit_reason.basic != EXIT_REASON_EPT_MISCONFIG)) {
		kvm_prepare_event_vectoring_exit(vcpu, INVALID_GPA);
		return 0;
	}

	if (exit_fastpath != EXIT_FASTPATH_NONE)
		return 1;

	if (exit_reason.basic >= pkvm_vmx_max_exit_handlers)
		goto unexpected_vmexit;

	exit_handler_index = array_index_nospec((u16)exit_reason.basic,
						pkvm_vmx_max_exit_handlers);
	if (!pkvm_vmx_exit_handlers[exit_handler_index])
		goto unexpected_vmexit;

	return pkvm_vmx_exit_handlers[exit_handler_index](vcpu);

unexpected_vmexit:
	vcpu_unimpl(vcpu, "vmx: unexpected exit reason 0x%x\n",
		    exit_reason.full);
	vcpu->run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
	vcpu->run->internal.suberror =
			KVM_INTERNAL_ERROR_UNEXPECTED_EXIT_REASON;
	vcpu->run->internal.ndata = 2;
	vcpu->run->internal.data[0] = exit_reason.full;
	vcpu->run->internal.data[1] = vcpu->arch.last_vmentry_cpu;
	return 0;
}

static int pkvm_check_processor_compat(void)
{
	return pkvm_hypercall(check_processor_compatibility);
}

static int pkvm_enable_virtualization_cpu(void)
{
	/*
	 * There is nothing to do here as virtualization was already enabled
	 * during pKVM initialization and is never disabled or re-enabled later.
	 */

	return 0;
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
	void *pkvm_vm, *pgd;
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

	pgd = (void *)__get_free_page(GFP_KERNEL_ACCOUNT);
	if (!pgd) {
		ret = -ENOMEM;
		goto free_page;
	}

	kvm_account_pgtable_pages(pgd, 1);

	ret = pkvm_hypercall(vm_init, __pa(kvm), __pa(pkvm_vm), __pa(pgd));
	if (ret < 0)
		goto free_pgtable_page;

	kvm->arch.pkvm.handle = ret;

	if (pkvm_is_protected_vm(kvm))
		kvm->arch.has_protected_state = true;

	return 0;

free_pgtable_page:
	kvm_account_pgtable_pages(pgd, -1);
	free_page((unsigned long)pgd);
free_page:
	free_pages_exact(pkvm_vm, PKVM_VMX_VM_SIZE);
	return ret;
}

static void pkvm_vm_destroy(struct kvm *kvm)
{
	int vm_handle = kvm->arch.pkvm.handle;
	struct pkvm_mapping *mapping;
	union pkvm_hc_data out;
	int ret;

	ret = pkvm_hypercall_out(vm_destroy, &out, vm_handle);
	if (ret) {
		pr_err("failed to destroy VM%d: %d\n", vm_handle, ret);
		return;
	}

	kvm_free_pkvm_memcache(&out.vm_destroy.memcache);
	kvm_free_pkvm_memcache(&kvm->arch.pkvm.guest_mmu_teardown_mc);

	/*
	 * TODO: do this in the generic code in kvm_mmu_uninit_vm() instead.
	 * For now we cannot do that, since kvm_mmu_uninit_vm() is called too
	 * early, when pKVM has not released guest pages to the host yet
	 * (which is currently completely postponed until vm_destroy).
	 */
	for_each_pkvm_mapping(kvm, 0, U64_MAX, mapping) {
		WARN_ON_ONCE((mapping->pinned_page != NULL) ^ pkvm_is_protected_vm(kvm));
		if (mapping->pinned_page)
			put_page(mapping->pinned_page);

		pkvm_mapping_remove(mapping, &kvm->arch.pkvm.mappings);
		kfree(mapping);
	}

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
	if (lapic_in_kernel(vcpu)) {
		vcpu_size += sizeof(struct kvm_lapic);
		if (pkvm_is_protected_vcpu(vcpu) && enable_apicv)
			vcpu_size += PAGE_SIZE;
	}

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

	if (!pkvm_is_protected_vcpu(vcpu) && init_event)
		KVM_BUG_ON(pkvm_hypercall(vcpu_reset), vcpu->kvm);

	/*
	 * The host is responsible for emulating guest timer by using the
	 * preemption timer if this feature is supported. The hv_deadline_tsc
	 * is synced to the pKVM hypervosr to update the preemption timer
	 * accordingly. Initialize hv_deadline_tsc as -1 so that the pKVM
	 * hypervisor can disable the preemption timer if it is unused.
	 */
	vmx->hv_deadline_tsc = -1;

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
		 * The pVM's lapic is set up in x2apic mode by the pKVM
		 * hypervisor when creating a vCPU. Set the apic_base from the
		 * host side to x2apic mode to be consistent with the pKVM to
		 * make sure the host emulated lapic registers (e.g., APIC_ID)
		 * are initialized properly.
		 */
		u64 data = APIC_DEFAULT_PHYS_BASE | LAPIC_MODE_X2APIC |
			   (kvm_vcpu_is_reset_bsp(vcpu) ? MSR_IA32_APICBASE_BSP : 0);

		/*
		 * Force set the X86_FEATURE_X2APIC bit in the vcpu caps to make
		 * sure the x2apic mode apic base can be set successfully. Doing
		 * so without respecting the X2APIC feature bit in the vCPUID
		 * entries is fine as the pVM's vCPUID entries will be enforced
		 * by the pKVM hypervisor to make sure the X2APIC feature bit
		 * is always set.
		 */
		guest_cpu_cap_set(vcpu, X86_FEATURE_X2APIC);
		kvm_apic_set_base(vcpu, data, true);
	}
}

static void pkvm_prepare_switch_to_guest(struct kvm_vcpu *vcpu) {}

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

	if (!vcpu->arch.guest_state_protected) {
		int ret = pkvm_hypercall(set_msr, msr_info->index, msr_info->data);

		if (ret)
			return ret;

		switch (msr_info->index) {
		case MSR_IA32_XFD:
			fpu_update_guest_xfd(&vcpu->arch.guest_fpu, msr_info->data);
			break;
		default:
			break;
		}

		return 0;
	}

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
	if (!vcpu->arch.guest_state_protected) {
		int ret = pkvm_hypercall(set_efer, efer);

		if (ret)
			return ret;
	}

	vcpu->arch.efer = efer;
	return 0;
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

static void pkvm_sync_dirty_debug_regs(struct kvm_vcpu *vcpu) {}

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

static int pkvm_vcpu_pre_run(struct kvm_vcpu *vcpu)
{
	struct kvm_pkvm_vm *pkvm = &vcpu->kvm->arch.pkvm;
	int ret = 0;

	if (unlikely(pkvm_is_protected_vcpu(vcpu) && !kvm_vcpu_has_run(vcpu) &&
		     kvm_vcpu_is_reset_bsp(vcpu))) {
		mutex_lock(&pkvm->finalized_lock);
		if (!pkvm->finalized)
			ret = pkvm_hypercall(vm_finalize, pkvm->handle);
		mutex_unlock(&pkvm->finalized_lock);
		if (ret < 0)
			return ret;
	}

	return 1;
}

static noinstr void pkvm_vcpu_enter_exit(struct kvm_vcpu *vcpu, u64 run_flags,
					 union pkvm_hc_data *out)
{
	bool force_immediate_exit = run_flags & KVM_RUN_FORCE_IMMEDIATE_EXIT;
	struct vcpu_vmx *vmx = to_vmx(vcpu);

	guest_state_enter_irqoff();

	vmx->vt.exit_reason.full = 0xdead;
	vcpu->arch.regs_avail &= ~VMX_REGS_LAZY_LOAD_SET;

	vmx->fail = !!pkvm_hypercall_out(vcpu_run, out, force_immediate_exit);
	if (unlikely(vmx->fail))
		goto done;

	if (unlikely(vmx_get_exit_reason(vcpu).full == 0xdead)) {
		vmx->fail = 1;
		goto done;
	}

	vcpu->arch.regs_dirty = 0;

	vmx_handle_nmi(vcpu);
done:
	guest_state_exit_irqoff();
}

static fastpath_t pkvm_vcpu_run(struct kvm_vcpu *vcpu, u64 run_flags)
{
	bool force_immediate_exit = run_flags & KVM_RUN_FORCE_IMMEDIATE_EXIT;
	struct vcpu_vmx *vmx = to_vmx(vcpu);
	unsigned long reqs_to_host;
	fastpath_t exit_fastpath;
	union pkvm_hc_data out;

	trace_kvm_entry(vcpu, force_immediate_exit);

	kvm_wait_lapic_expire(vcpu);

	vcpu->arch.nmi_injected = false;
	kvm_clear_exception_queue(vcpu);
	kvm_clear_interrupt_queue(vcpu);

	pkvm_vcpu_enter_exit(vcpu, run_flags, &out);
	if (unlikely(vmx->fail))
		return EXIT_FASTPATH_NONE;

	/*
	 * The host still needs to pre-configure pVM's vCPU state for booting.
	 * Once the vcpu has started running, some PV interfaces will be
	 * inaccessible to the host. Setting the guest_state_protected flag for
	 * the host to avoid using such PV interface.
	 */
	if (unlikely(vcpu->kvm->arch.has_protected_state &&
		     !vcpu->arch.guest_state_protected)) {
		vcpu->arch.guest_state_protected = true;
		fpstate_set_confidential(&vcpu->arch.guest_fpu);
		if (lapic_in_kernel(vcpu)) {
			/*
			 * As the host VMM (e.g., crosvm) is still using the
			 * ioctl to access the pVM's apic state, i.e.,
			 * kvm_vcpu_ioctl_get/set_lapic, defer setting the
			 * host's guest_apic_protected flag after the vCPU
			 * starts running to avoid failures to the crosvm.
			 * Allowing this should be fine as the apic page is
			 * donated to the pKVM which is inaccessible to the
			 * host, and the host VMM can only use this ioctl to
			 * access the states emulated by the host itself.
			 */
			vcpu->arch.apic->guest_apic_protected = enable_apicv;
		}
	}

	if (unlikely((u16)vmx_get_exit_reason(vcpu).basic == EXIT_REASON_MCE_DURING_VMENTRY))
		kvm_machine_check();

	trace_kvm_exit(vcpu, KVM_ISA_VMX);

	if (unlikely(vmx_get_exit_reason(vcpu).failed_vmentry))
		return EXIT_FASTPATH_NONE;

	if (vcpu->arch.nmi_injected || vcpu->arch.interrupt.injected ||
	    vcpu->arch.exception.pending || vcpu->arch.exception.injected)
		kvm_make_request(KVM_REQ_EVENT, vcpu);

	exit_fastpath = EXIT_FASTPATH_EXIT_HANDLED;
	reqs_to_host = out.vcpu_run.reqs_to_host;
	if (reqs_to_host) {
		if (test_and_clear_bit(HOST_HANDLE_EXIT, &reqs_to_host))
			exit_fastpath = EXIT_FASTPATH_NONE;

		if (test_and_clear_bit(HOST_HANDLE_GUESTDBG_SINGLESTEP, &reqs_to_host)) {
			struct kvm_run *kvm_run = vcpu->run;

			kvm_run->debug.arch.dr6 = DR6_BS | DR6_ACTIVE_LOW;
			kvm_run->debug.arch.pc = kvm_get_linear_rip(vcpu);
			kvm_run->debug.arch.exception = DB_VECTOR;
			kvm_run->exit_reason = KVM_EXIT_DEBUG;

			exit_fastpath = EXIT_FASTPATH_EXIT_USERSPACE;
		}

		if (test_and_clear_bit(HOST_INIT_MMU, &reqs_to_host))
			kvm_init_mmu(vcpu);

		if (test_and_clear_bit(HOST_RESET_MMU, &reqs_to_host))
			kvm_mmu_reset_context(vcpu);

		if (test_and_clear_bit(HOST_APF_READY, &reqs_to_host))
			kvm_make_request(KVM_REQ_APF_READY, vcpu);
	}

	if (exit_fastpath == EXIT_FASTPATH_EXIT_HANDLED ||
	    exit_fastpath == EXIT_FASTPATH_EXIT_USERSPACE)
		return exit_fastpath;

	return pkvm_exit_handlers_fastpath(vcpu);
}

static int pkvm_handle_exit(struct kvm_vcpu *vcpu, fastpath_t exit_fastpath)
{
	int ret = __pkvm_handle_exit(vcpu, exit_fastpath);

	/*
	 * Exit to user space when bus lock detected to inform that there is
	 * a bus lock in guest.
	 */
	if (vmx_get_exit_reason(vcpu).bus_lock_detected) {
		if (ret > 0)
			vcpu->run->exit_reason = KVM_EXIT_X86_BUS_LOCK;

		vcpu->run->flags |= KVM_RUN_X86_BUS_LOCK;
		return 0;
	}

	return ret;
}

static int pkvm_skip_emulated_instruction(struct kvm_vcpu *vcpu)
{
	if (vcpu->arch.guest_state_protected)
		return 1;

	if (vcpu->arch.event_exit_inst_len) {
		unsigned long rip, orig_rip;

		orig_rip = kvm_rip_read(vcpu);
		rip = orig_rip + vcpu->arch.event_exit_inst_len;
#ifdef CONFIG_X86_64
		/*
		 * We need to mask out the high 32 bits of RIP if not in 64-bit
		 * mode, but just finding out that we are in 64-bit mode is
		 * quite expensive.  Only do it if there was a carry.
		 */
		if (unlikely(((rip ^ orig_rip) >> 31) == 3) && !is_64_bit_mode(vcpu))
			rip = (u32)rip;
#endif
		kvm_rip_write(vcpu, rip);
	} else if (!kvm_emulate_instruction(vcpu, EMULTYPE_SKIP)) {
		return 0;
	}

	/* skipping an emulated instruction also counts */
	pkvm_hypercall(set_interrupt_shadow, 0);

	return 1;
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
	if (pkvm_is_protected_vcpu(vcpu))
		return;

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
	if (lapic_in_kernel(vcpu) && vcpu->arch.apic->guest_apic_protected)
		return;

	KVM_BUG_ON(pkvm_hypercall(hwapic_isr_update, max_isr), vcpu->kvm);
}

static int pkvm_sync_pir_to_irr(struct kvm_vcpu *vcpu)
{
	if (!lapic_in_kernel(vcpu) || vcpu->arch.apic->guest_apic_protected)
		return -1;

	return vmx_sync_pir_to_irr(vcpu);
}

static void pkvm_get_exit_info(struct kvm_vcpu *vcpu, u32 *reason, u64 *info1,
			       u64 *info2, u32 *intr_info, u32 *error_code)
{
	struct vcpu_vmx *vmx = to_vmx(vcpu);

	*reason = vmx->vt.exit_reason.full;
	*info1 = vmx_get_exit_qual(vcpu);
	if (!(vmx->vt.exit_reason.failed_vmentry)) {
		*info2 = vmx->idt_vectoring_info;
		*intr_info = vmx_get_intr_info(vcpu);
		if (is_exception_with_error_code(*intr_info))
			*error_code = vmx->error_code;
		else
			*error_code = 0;
	} else {
		*info2 = 0;
		*intr_info = 0;
		*error_code = 0;
	}
}

static void pkvm_get_entry_info(struct kvm_vcpu *vcpu, u32 *intr_info, u32 *error_code)
{
	if (vcpu->arch.exception.injected)
		*intr_info = vcpu->arch.exception.vector;
	else if (vcpu->arch.nmi_injected)
		*intr_info = NMI_VECTOR;
	else if (vcpu->arch.interrupt.injected)
		*intr_info = vcpu->arch.interrupt.nr;
	else
		*intr_info = 0;

	if (vcpu->arch.exception.injected &&
	    vcpu->arch.exception.has_error_code)
		*error_code = vcpu->arch.exception.error_code;
	else
		*error_code = 0;
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

static bool pkvm_has_vmx_wbinvd_exit(void)
{
	if (unlikely(has_wbinvd_exit == USHRT_MAX))
		has_wbinvd_exit = !!pkvm_hypercall(has_wbinvd_exit);

	return has_wbinvd_exit;
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
	/*
	 * The host's root_hpa and root_level values are ignored, since
	 * the EPT is managed by the pKVM hypervisor independently of the
	 * host, for both pVMs and npVMs. The purpose of the load_mmu_pgd
	 * PV interface is not to load the guest's EPT (pKVM will load it
	 * anyway, without the host's help) but only to load the guest's
	 * stage-1 page table root, i.e. CR3 and/or PDPTRs.
	 */
	if (!vcpu->arch.guest_state_protected)
		KVM_BUG_ON(pkvm_hypercall(load_mmu_pgd), vcpu->kvm);
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

static int pkvm_check_intercept(struct kvm_vcpu *vcpu,
				struct x86_instruction_info *info,
				enum x86_intercept_stage stage,
				struct x86_exception *exception)
{
	return X86EMUL_UNHANDLEABLE;
}

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

static int pkvm_check_emulate_instruction(struct kvm_vcpu *vcpu, int emul_type,
					  void *insn, int insn_len)
{
	/*
	 * The host can emulate instruction for npVMs but cannot for pVMs. Thus
	 * pVM's kernel should be enlightened to avoid causing such vmexits.
	 * Return X86EMUL_UNHANDLEABLE in case it is not.
	 */
	if (pkvm_is_protected_vcpu(vcpu))
		return X86EMUL_UNHANDLEABLE;

	return vmx_check_emulate_instruction(vcpu, emul_type, insn, insn_len);
}

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

static void pkvm_recalc_intercepts(struct kvm_vcpu *vcpu) {}

static int pkvm_complete_emulated_msr(struct kvm_vcpu *vcpu, int err)
{
	if (pkvm_is_protected_vcpu(vcpu)) {
		/*
		 * To complete emulation with error code, complete_emulated_msr
		 * PV interface should be used to tell the pKVM hypervisor to
		 * inject #GP as the host is not allowed to inject any exception
		 * to a pVM. If the emulation is successful, then nothing needs
		 * to be completed from the host side as the pKVM hypervisor will
		 * skip the MSR instruction before entering the pVM again.
		 */
		return err ? pkvm_hypercall(complete_emulated_msr, err) : 1;
	}

	/*
	 * For npVM, the host is allowed to inject #GP or skip the MSR
	 * instruction. No need to bother the pKVM hypervisor.
	 */
	return kvm_complete_insn_gp(vcpu, err);
}

static bool pkvm_protected_apic_has_interrupt(struct kvm_vcpu *vcpu)
{
	union pkvm_hc_data out;

	if (!lapic_in_kernel(vcpu) || KVM_BUG_ON(!vcpu->arch.apic->guest_apic_protected,
						 vcpu->kvm))
		return false;

	if (pi_has_pending_interrupt(vcpu))
		return true;

	if (KVM_BUG_ON(pkvm_hypercall_out(protected_apic_has_interrupt, &out), vcpu->kvm))
		return false;

	return out.protected_apic_has_interrupt.has_intr;
}

struct kvm_x86_ops pkvm_host_vt_x86_ops __initdata = {
	.name = KBUILD_MODNAME,

	.check_processor_compatibility = pkvm_check_processor_compat,

	.hardware_unsetup = vmx_hardware_unsetup,

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

	.prepare_switch_to_guest = pkvm_prepare_switch_to_guest,
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
	.sync_dirty_debug_regs = pkvm_sync_dirty_debug_regs,
	.cache_reg = pkvm_cache_reg,
	.get_rflags = pkvm_get_rflags,
	.set_rflags = pkvm_set_rflags,
	.get_if_flag = pkvm_get_if_flag,

	.flush_tlb_all = pkvm_flush_tlb_all,
	.flush_tlb_current = pkvm_flush_tlb_current,
	.flush_tlb_gva = pkvm_flush_tlb_gva,
	.flush_tlb_guest = pkvm_flush_tlb_guest,

	.vcpu_pre_run = pkvm_vcpu_pre_run,
	.vcpu_run = pkvm_vcpu_run,
	.handle_exit = pkvm_handle_exit,
	.skip_emulated_instruction = pkvm_skip_emulated_instruction,
	.set_interrupt_shadow = pkvm_set_interrupt_shadow,
	.get_interrupt_shadow = pkvm_get_interrupt_shadow,
	.patch_hypercall = vmx_patch_hypercall,
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
	.sync_pir_to_irr = pkvm_sync_pir_to_irr,
	.deliver_interrupt = vmx_deliver_interrupt,
	.dy_apicv_has_pending_interrupt = pi_has_pending_interrupt,

	.get_exit_info = pkvm_get_exit_info,
	.get_entry_info = pkvm_get_entry_info,

	.vcpu_after_set_cpuid = pkvm_vcpu_after_set_cpuid,

	.has_wbinvd_exit = pkvm_has_vmx_wbinvd_exit,

	.get_l2_tsc_offset = pkvm_get_l2_tsc_offset,
	.get_l2_tsc_multiplier = pkvm_get_l2_tsc_multiplier,
	.write_tsc_offset = pkvm_write_tsc_offset,
	.write_tsc_multiplier = pkvm_write_tsc_multiplier,

	.load_mmu_pgd = pkvm_load_mmu_pgd,

	.check_intercept = pkvm_check_intercept,
	.handle_exit_irqoff = vmx_handle_exit_irqoff,

	.nested_ops = &pkvm_nested_ops,

	.pi_update_irte = vmx_pi_update_irte,
	.pi_start_bypass = vmx_pi_start_bypass,

#ifdef CONFIG_X86_64
	.set_hv_timer = vmx_set_hv_timer,
	.cancel_hv_timer = vmx_cancel_hv_timer,
#endif
	.setup_mce = pkvm_setup_mce,

#ifdef CONFIG_KVM_SMM
	.smi_allowed = pkvm_smi_allowed,
	.enter_smm = pkvm_enter_smm,
	.leave_smm = pkvm_leave_smm,
	.enable_smi_window = pkvm_enable_smi_window,
#endif

	.check_emulate_instruction = pkvm_check_emulate_instruction,
	.apic_init_signal_blocked = pkvm_apic_init_signal_blocked,

	.recalc_intercepts = pkvm_recalc_intercepts,
	.complete_emulated_msr = pkvm_complete_emulated_msr,

	.vcpu_deliver_sipi_vector = kvm_vcpu_deliver_sipi_vector,

	.get_untagged_addr = vmx_get_untagged_addr,

	.protected_apic_has_interrupt = pkvm_protected_apic_has_interrupt,
};

static struct kvm_pmc *pkvm_intel_rdpmc_ecx_to_pmc(struct kvm_vcpu *vcpu,
						   unsigned int idx, u64 *mask)
{
	return NULL;
}
static struct kvm_pmc *pkvm_intel_msr_idx_to_pmc(struct kvm_vcpu *vcpu, u32 msr) { return NULL; }
static bool pkvm_intel_is_valid_msr(struct kvm_vcpu *vcpu, u32 msr) { return false; }
static int pkvm_intel_pmu_get_msr(struct kvm_vcpu *vcpu, struct msr_data *msr_info) { return 1; }
static int pkvm_intel_pmu_set_msr(struct kvm_vcpu *vcpu, struct msr_data *msr_info) { return 1; }
static void pkvm_intel_pmu_refresh(struct kvm_vcpu *vcpu) {}
static void pkvm_intel_pmu_init(struct kvm_vcpu *vcpu) {}
static void pkvm_intel_pmu_reset(struct kvm_vcpu *vcpu) {}
static void pkvm_intel_pmu_deliver_pmi(struct kvm_vcpu *vcpu) {}
static void pkvm_intel_pmu_cleanup(struct kvm_vcpu *vcpu) {}

static struct kvm_pmu_ops pkvm_host_vt_pmu_ops __initdata = {
	.rdpmc_ecx_to_pmc = pkvm_intel_rdpmc_ecx_to_pmc,
	.msr_idx_to_pmc = pkvm_intel_msr_idx_to_pmc,
	.is_valid_msr = pkvm_intel_is_valid_msr,
	.get_msr = pkvm_intel_pmu_get_msr,
	.set_msr = pkvm_intel_pmu_set_msr,
	.refresh = pkvm_intel_pmu_refresh,
	.init = pkvm_intel_pmu_init,
	.reset = pkvm_intel_pmu_reset,
	.deliver_pmi = pkvm_intel_pmu_deliver_pmi,
	.cleanup = pkvm_intel_pmu_cleanup,
	.EVENTSEL_EVENT = ARCH_PERFMON_EVENTSEL_EVENT,
	.MAX_NR_GP_COUNTERS = 0,
	.MIN_NR_GP_COUNTERS = 0,
};

struct kvm_x86_init_ops pkvm_host_vt_init_ops __initdata = {
	.hardware_setup = vmx_hardware_setup,
	.handle_intel_pt_intr = NULL,

	.runtime_ops = &pkvm_host_vt_x86_ops,
	.pmu_ops = &pkvm_host_vt_pmu_ops,
};

bool pkvm_interrupt_blocked(struct kvm_vcpu *vcpu)
{
	return (pkvm_interrupt_allowed(vcpu, false) <= 0);
}
