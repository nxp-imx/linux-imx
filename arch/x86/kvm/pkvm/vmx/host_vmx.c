// SPDX-License-Identifier: GPL-2.0
#include <linux/kvm_types.h>
#include <linux/memblock.h>
#include <asm/fpu/xcr.h>
#include <kvm_emulate.h>
#include <vmx/x86_ops.h>
#include "debug.h"
#include "ept.h"
#include "host_vmx.h"
#include "pkvm/init.h"
#include "pkvm/lapic.h"
#include "pkvm/trace.h"
#include "pkvm.h"
#include "pkvm_iommu.h"

#define CR4			4
#define MOV_TO_CR		0

struct vmcs_config host_vmcs_config;

static int vmx_hyp_mmu_finalize(struct pkvm_pgtable *pgt)
{
	if (!pgt)
		return -EINVAL;

	vmcs_writel(HOST_CR3, pgt->root_pa);

	return 0;
}

static struct pkvm_init_ops vmx_init_ops = {
	.hyp_mmu_finalize = vmx_hyp_mmu_finalize,
	.host_mmu_init = pkvm_host_ept_init,
	.host_mmu_finalize = pkvm_host_ept_finalize,
	.hyp_global_init = pkvm_vmx_init,
	.reprivilege_cpu = pkvm_vmx_reprivilege_cpu,
	.hyp_iommu_init = pkvm_intel_iommu_init,
};

struct pkvm_init_ops *pkvm_vmx_init_ops = &vmx_init_ops;

static void skip_emulated_instruction(struct kvm_vcpu *vcpu)
{
	unsigned long rip;

	rip = vmcs_readl(GUEST_RIP);
	rip += vmcs_read32(VM_EXIT_INSTRUCTION_LEN);
	vmcs_writel(GUEST_RIP, rip);

	vmx_set_interrupt_shadow(vcpu, 0);
}

static void handle_irq_window(struct kvm_vcpu *vcpu)
{
	u32 cpu_based_exec_ctrl = exec_controls_get(to_vmx(vcpu));

	exec_controls_set(to_vmx(vcpu), cpu_based_exec_ctrl &
					~CPU_BASED_INTR_WINDOW_EXITING);

	kvm_make_request(KVM_REQ_EVENT, vcpu);
}

static void handle_cpuid(struct kvm_vcpu *vcpu)
{
	u32 eax, ebx, ecx, edx;

	eax = vcpu->arch.regs[VCPU_REGS_RAX];
	ecx = vcpu->arch.regs[VCPU_REGS_RCX];
	native_cpuid(&eax, &ebx, &ecx, &edx);
	vcpu->arch.regs[VCPU_REGS_RAX] = eax;
	vcpu->arch.regs[VCPU_REGS_RBX] = ebx;
	vcpu->arch.regs[VCPU_REGS_RCX] = ecx;
	vcpu->arch.regs[VCPU_REGS_RDX] = edx;
}

static void handle_vmcall(struct kvm_vcpu *vcpu)
{
	pkvm_handle_host_hypercall(vcpu);
}

static void handle_cr(struct kvm_vcpu *vcpu)
{
	struct vcpu_vt *vt = to_vt(vcpu);
	unsigned long exit_qual, val;
	int cr, type, reg;

	exit_qual = vt->exit_qualification;
	cr = exit_qual & 15;
	type = (exit_qual >> 4)	& 3;
	reg = (exit_qual >> 8) & 15;

	switch (type) {
	case MOV_TO_CR:
		switch (cr) {
		case CR4:
			/*
			 * VMXE bit is owned by pkvm, others are owned by host
			 * So only when guest is trying to modify VMXE bit it
			 * can cause vmexit and get here.
			 */
			val = vcpu->arch.regs[reg];
			vmcs_writel(CR4_READ_SHADOW, val);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static bool is_msr_in_bitmap_range(u32 msr)
{
	return msr <= 0x1FFF || (msr >= 0xC0000000 && msr <= 0xC0001FFF);
}

static int handle_read_msr(struct kvm_vcpu *vcpu)
{
	u32 msr = vcpu->arch.regs[VCPU_REGS_RCX];
	u32 low, high;

	/*
	 * The MSR reading bitmap doesn't intercept any MSR. If the vmexit is
	 * caused by such MSR in the range of the bitmap, it should be a code
	 * bug.
	 */
	BUG_ON(is_msr_in_bitmap_range(msr));

	if (rdmsr_safe(msr, &low, &high)) {
		kvm_inject_gp(vcpu, 0);
		return X86EMUL_UNHANDLEABLE;
	}

	vcpu->arch.regs[VCPU_REGS_RAX] = low;
	vcpu->arch.regs[VCPU_REGS_RDX] = high;

	return X86EMUL_CONTINUE;
}

static int handle_write_msr(struct kvm_vcpu *vcpu)
{
	u32 msr = vcpu->arch.regs[VCPU_REGS_RCX];
	int ret = X86EMUL_CONTINUE;
	u32 low, high;
	u64 val;

	low = vcpu->arch.regs[VCPU_REGS_RAX];
	high = vcpu->arch.regs[VCPU_REGS_RDX];
	val = low | ((u64)high << 32);

	switch (msr) {
	case MSR_CORE_PERF_GLOBAL_CTRL: {
		struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
		struct vcpu_vmx *vmx = to_vmx(vcpu);

		if (!kvm_pmu_has_perf_global_ctrl(pmu)) {
			ret = X86EMUL_UNHANDLEABLE;
			break;
		}

		if (pmu->global_ctrl == val)
			break;

		/*
		 * PMU is owned by the host. But the host must be prevented
		 * from profiling pKVM or pVM so the global ctrl MSR is kept
		 * as ZERO (disabled) outside of the host context.
		 *
		 * Capture the value written by host. If it's non-zero then
		 * update the VMCS guest field and rely on VM entry/exit
		 * control to switch the MSR value. The VMCS host field is
		 * fixed to ZERO.
		 */
		pmu->global_ctrl = val;
		if (val) {
			vmcs_write64(GUEST_IA32_PERF_GLOBAL_CTRL, val);
			vm_entry_controls_setbit(vmx,
					VM_ENTRY_LOAD_IA32_PERF_GLOBAL_CTRL);
			vm_exit_controls_setbit(vmx,
					VM_EXIT_LOAD_IA32_PERF_GLOBAL_CTRL);
		} else {
			vm_entry_controls_clearbit(vmx,
					VM_ENTRY_LOAD_IA32_PERF_GLOBAL_CTRL);
			vm_exit_controls_clearbit(vmx,
					VM_EXIT_LOAD_IA32_PERF_GLOBAL_CTRL);
		}
		break;
	}
	case MSR_IA32_XSS:
		BUG_ON(!pkvm_cpu_initialized(vcpu->cpu));

		if (val != kvm_host.xss) {
			pkvm_warn("Host attempt to modify MSR_IA32_XSS: 0x%llx (expected 0x%llx)\n",
				  val, kvm_host.xss);
			ret = X86EMUL_UNHANDLEABLE;
			break;
		}
		if (wrmsr_safe(msr, low, high)) {
			ret = X86EMUL_UNHANDLEABLE;
			break;
		}
		break;
	case MSR_SYSCALL_MASK:
	case MSR_LSTAR:
	case MSR_CSTAR:
	case MSR_TSC_AUX:
	case MSR_STAR:
	case MSR_IA32_TSX_CTRL: {
		int slot;
		u64 cur;

		BUG_ON(!pkvm_cpu_initialized(vcpu->cpu));

		slot = kvm_find_user_return_msr(msr);
		if (slot >= 0) {
			cur = kvm_get_user_return_msr(slot);
			if (val != cur) {
				pkvm_warn("Host attempt to modify user-return MSR 0x%x: 0x%llx (expected 0x%llx)\n",
					  msr, val, cur);
				ret = X86EMUL_UNHANDLEABLE;
				break;
			}
		}

		if (wrmsr_safe(msr, low, high)) {
			ret = X86EMUL_UNHANDLEABLE;
			break;
		}
		break;
	}
	case MSR_IA32_APICBASE:
	case APIC_BASE_MSR ... APIC_BASE_MSR + 0xff:
		if (pkvm_lapic_msr_write(msr, val))
			ret = X86EMUL_UNHANDLEABLE;
		break;
	default:
		/*
		 * The MSRs intercepted by the writing bitmap should be
		 * emulated by the switch cases. Otherwise it should be a code
		 * bug.
		 */
		BUG_ON(is_msr_in_bitmap_range(msr));

		if (wrmsr_safe(msr, low, high))
			ret = X86EMUL_UNHANDLEABLE;

		break;
	}

	if (ret == X86EMUL_UNHANDLEABLE)
		kvm_inject_gp(vcpu, 0);

	return ret;
}

static void handle_preemption_timer(struct kvm_vcpu *vcpu)
{
	pin_controls_clearbit(to_vmx(vcpu), PIN_BASED_VMX_PREEMPTION_TIMER);
}

static int handle_xsetbv(struct kvm_vcpu *vcpu)
{
	u32 eax = (u32)(vcpu->arch.regs[VCPU_REGS_RAX] & -1u);
	u32 edx = (u32)(vcpu->arch.regs[VCPU_REGS_RDX] & -1u);
	u32 ecx = (u32)(vcpu->arch.regs[VCPU_REGS_RCX] & -1u);

	BUG_ON(!pkvm_cpu_initialized(vcpu->cpu));

	if (ecx == XCR_XFEATURE_ENABLED_MASK) {
		u64 xcr0 = (u64)eax | ((u64)edx << 32);

		if (xcr0 != kvm_host.xcr0) {
			pkvm_warn("Host attempt to modify XCR0: 0x%llx (expected 0x%llx)\n",
				  xcr0, kvm_host.xcr0);
			goto fault;
		}
	}

	asm goto("1: xsetbv\n\t"
		 _ASM_EXTABLE(1b, %l[fault])
		 : : "a" (eax), "d" (edx), "c" (ecx) : : fault);

	return X86EMUL_CONTINUE;

fault:
	/*
	 * Although the SDM doesn't describe the priority of #UD and
	 * interception for xsetbv, the experiment shows that #UD due to
	 * CR4.OSXSAVE[bit 18] == 0 and the LOCK prefix has priority
	 * over the interception.
	 *
	 * So the pKVM hypervisor itself won't generate #UD when
	 * executes the xsetbv instruction, only #GP can be generated
	 * due to invalid configurations. Always inject #GP if xsetbv
	 * is failed.
	 *
	 * TODO: CPUID.01H:ECX.XSAVE[bit 26] == 0 will also result in
	 * #UD but all modern Intel CPUs have XSAVE. If the pKVM runs
	 * on such CPU without XSAVE, verify if this #UD also has
	 * priority over the interception.
	 */
	kvm_inject_gp(vcpu, 0);
	return X86EMUL_UNHANDLEABLE;
}

static void inject_pending_nmi(struct kvm_vcpu *vcpu)
{
	if (!vcpu->arch.nmi_pending)
		return;

	/*
	 * Check for the NMI blocking and inject the NMI only when it is not
	 * blocked.
	 * The vmx code vmx_nmi_blocked() and vmx_inject_nmi() are not used at
	 * here as their implementation is related with the global parameter
	 * enable_vnmi which can determine how the guest VMs handle the NMI. The
	 * host VM has physical NMI passthrough which is not exactly fitting to
	 * the usage of enable_vnmi.
	 */
	if (!(vmcs_read32(GUEST_INTERRUPTIBILITY_INFO) &
	      (GUEST_INTR_STATE_MOV_SS | GUEST_INTR_STATE_STI |
	       GUEST_INTR_STATE_NMI))) {
		--vcpu->arch.nmi_pending;
		vmcs_write32(VM_ENTRY_INTR_INFO_FIELD,
			     INTR_TYPE_NMI_INTR | INTR_INFO_VALID_MASK | NMI_VECTOR);
		vmx_clear_hlt(vcpu);
	}

	/*
	 * If there are more pending NMI, open the irq window to inject the
	 * pending ones when the NMI is unblocked. Using irq window rather than
	 * the NMI window since this is for the physical NMI, while NMI window
	 * is for virtual-NMI when virtual-NMI execution control is enabled,
	 * which is not used for the host VM.
	 */
	if (vcpu->arch.nmi_pending)
		vmx_enable_irq_window(vcpu);
}

static void handle_pending_events(struct kvm_vcpu *vcpu, bool *req_immediate_exit)
{
	if (kvm_check_request(KVM_REQ_NMI, vcpu)) {
		vcpu->arch.nmi_pending += atomic_xchg(&vcpu->arch.nmi_queued, 0);
		kvm_make_request(KVM_REQ_EVENT, vcpu);
	}

	if (kvm_check_request(KVM_REQ_EVENT, vcpu)) {
		if (vcpu->arch.exception.pending) {
			vmx_inject_exception(vcpu);
			vcpu->arch.exception.pending = false;
			vcpu->arch.exception.injected = true;

			if (vmcs_readl(GUEST_CR4) & X86_CR4_FRED) {
				/*
				 * KVM doesn't implement FRED virtualization yet, so
				 * cannot rely on vmx_inject_exception() to deliver
				 * the fault address on #PF injection if FRED is
				 * enabled in the host. Do that manually here instead.
				 */
				vmcs_write64(INJECTED_EVENT_DATA,
					     vcpu->arch.exception.vector == PF_VECTOR ?
					     vcpu->arch.cr2 : 0);
			}
		}

		if (vcpu->arch.nmi_pending) {
			/*
			 * Inject pending NMI if no exception is already injected.
			 * Otherwise request an immediate exit to inject NMI in the
			 * next vmexit.
			 */
			if (!vcpu->arch.exception.injected)
				inject_pending_nmi(vcpu);
			else
				*req_immediate_exit = true;
		}
	}

	if (kvm_check_request(KVM_REQ_TLB_FLUSH_CURRENT, vcpu))
		pkvm_flush_host_ept();
}

static void fixup_host_vmx(struct vcpu_vmx *vmx)
{
	if (boot_cpu_has(X86_FEATURE_INTEL_PT)) {
		/*
		 * The VM_ENTRY_LOAD_IA32_RTIT_CTL bit may be cleared due to the
		 * MSR_IA32_RTIT_CTL TRACEEN bit is set before deprivileging. See
		 * comments in init_vmentry_control in pkvm_init.c.
		 *
		 * Ensure this bit is set after the host exits to the root mode.
		 * This can be done safely as VM_EXIT_CLEAR_IA32_RTIT_CTL is
		 * guaranteed to be set which causes the MSR_IA32_RTIT_CTL is 0.
		 */
		if (!(vm_entry_controls_get(vmx) & VM_ENTRY_LOAD_IA32_RTIT_CTL))
			vm_entry_controls_setbit(vmx, VM_ENTRY_LOAD_IA32_RTIT_CTL);
	}

	this_cpu_write(host_vcpu_fixup, false);
}

void pkvm_host_vmexit_main(struct vcpu_vmx *vmx)
{
	struct kvm_vcpu *vcpu = &vmx->vcpu;
	bool req_immediate_exit = false;
	struct vcpu_vt *vt = &vmx->vt;
	bool skip_instruction = false;

	pkvm_trace_vmexit_start(vcpu);

	pkvm_set_vcpu_outside_guest(vcpu);

	vcpu->arch.cr2 = native_read_cr2();
	vcpu->arch.exception.injected = false;

	vt->exit_reason.full = vmcs_read32(VM_EXIT_REASON);
	vt->exit_qualification = vmcs_readl(EXIT_QUALIFICATION);

	switch (vt->exit_reason.full) {
	case EXIT_REASON_INIT_SIGNAL:
		pkvm_handle_init_signal();
		break;
	case EXIT_REASON_INTERRUPT_WINDOW:
		handle_irq_window(vcpu);
		break;
	case EXIT_REASON_CPUID:
		handle_cpuid(vcpu);
		skip_instruction = true;
		break;
	case EXIT_REASON_VMCALL:
		handle_vmcall(vcpu);
		skip_instruction = true;
		break;
	case EXIT_REASON_CR_ACCESS:
		handle_cr(vcpu);
		skip_instruction = true;
		break;
	case EXIT_REASON_MSR_READ:
		if (handle_read_msr(vcpu) == X86EMUL_CONTINUE)
			skip_instruction = true;
		break;
	case EXIT_REASON_MSR_WRITE:
		if (handle_write_msr(vcpu) == X86EMUL_CONTINUE)
			skip_instruction = true;
		break;
	case EXIT_REASON_EPT_VIOLATION:
		pkvm_handle_host_ept_violation(vcpu);
		break;
	case EXIT_REASON_PREEMPTION_TIMER:
		handle_preemption_timer(vcpu);
		break;
	case EXIT_REASON_XSETBV:
		if (handle_xsetbv(vcpu) == X86EMUL_CONTINUE)
			skip_instruction = true;
		break;
	default:
		pkvm_err_ratelimited("Unsupported vmexit reason 0x%x.\n",
				      vt->exit_reason.full);
		kvm_inject_gp(vcpu, 0);
		break;
	}

	if (skip_instruction)
		skip_emulated_instruction(vcpu);

handle_events:
	handle_pending_events(vcpu, &req_immediate_exit);

	pkvm_set_vcpu_in_guest(vcpu);

	if (req_immediate_exit) {
		kvm_make_request(KVM_REQ_EVENT, vcpu);
		request_host_immediate_exit(vmx);
	} else if (READ_ONCE(vcpu->mode) == EXITING_GUEST_MODE ||
		   kvm_request_pending(vcpu)) {
		pkvm_set_vcpu_outside_guest(vcpu);
		/*
		 * Some vcpu requests may be set after handle_pending_events()
		 * but before set vcpu mode to IN_GUEST_MODE. In this case the
		 * init signal will not be send to kick the vcpu. To guarantee
		 * such vcpu requests can be handled timely, try to handle
		 * pending event again.
		 */
		goto handle_events;
	}

	if (vcpu->arch.cr2 != native_read_cr2())
		native_write_cr2(vcpu->arch.cr2);

	if (unlikely(this_cpu_read(host_vcpu_fixup)))
		fixup_host_vmx(vmx);

	pkvm_trace_vmexit_end(vcpu, vt->exit_reason.basic);
}
