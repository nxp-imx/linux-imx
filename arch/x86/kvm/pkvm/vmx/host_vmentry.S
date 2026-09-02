/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/linkage.h>
#include <asm/kvm_vcpu_regs.h>
#include <asm/frame.h>
#include <asm/asm.h>
#include <asm/unwind_hints.h>
#include <asm/nospec-branch.h>
#include "kvm-asm-offsets.h"
#include "run_flags.h"
#include "vcpu_regs.h"

/*
 * pkvm_host_vmexit_entry - Handle a VM-Exit event from the deprivileged
 * host in pkvm.
 *
 * The pKVM stack starts from fixed address, which is pkvm_sp_top
 * configured in VMCS.HOST_RSP.
 *
 * pkvm_sp_top = pcpu->stack + PKVM_STACK_SIZE - PKVM_STACK_TOP_RESV
 * pkvm_sp_top[0] --> pointer to vcpu->arch.regs
 * pkvm_sp_top[1] --> struct vcpu_vmx *vmx
 *
 */
SYM_FUNC_START(pkvm_host_vmexit_entry)
	UNWIND_HINT_ENTRY
	ENDBR
	push %_ASM_BX
	mov WORD_SIZE*1(%_ASM_SP), %_ASM_BX

	/*
	 * Unlike in vmx/vmenter.S, no need for FILL_RETURN_BUFFER here.
	 *
	 * pKVM requires eIBRS which has its own protection against poisoned RSB
	 * so no need for RSB filling.
	 *
	 * If eIBRS is affected by PBRSB, still no need for RSB filling here
	 * since there is no unbalanced RET after host vmexit.
	 */

	mov %_ASM_AX, VCPU_RAX(%_ASM_BX)
	mov %_ASM_CX, VCPU_RCX(%_ASM_BX)
	mov %_ASM_DX, VCPU_RDX(%_ASM_BX)
	mov %_ASM_SI, VCPU_RSI(%_ASM_BX)
	mov %_ASM_DI, VCPU_RDI(%_ASM_BX)
	mov %_ASM_BP, VCPU_RBP(%_ASM_BX)
	mov %r8 , VCPU_R8(%_ASM_BX)
	mov %r9 , VCPU_R9(%_ASM_BX)
	mov %r10, VCPU_R10(%_ASM_BX)
	mov %r11, VCPU_R11(%_ASM_BX)
	mov %r12, VCPU_R12(%_ASM_BX)
	mov %r13, VCPU_R13(%_ASM_BX)
	mov %r14, VCPU_R14(%_ASM_BX)
	mov %r15, VCPU_R15(%_ASM_BX)
	pop VCPU_RBX(%_ASM_BX)

	/*
	 * Clear all general purpose registers except RSP and RBX to prevent
	 * speculative use of the guest's values, even those that are reloaded
	 * via the stack. See the analogous comment in vmx/vmenter.S.
	 */
	xor %eax, %eax
	xor %ecx, %ecx
	xor %edx, %edx
	xor %ebp, %ebp
	xor %esi, %esi
	xor %edi, %edi
	xor %r8d,  %r8d
	xor %r9d,  %r9d
	xor %r10d, %r10d
	xor %r11d, %r11d
	xor %r12d, %r12d
	xor %r13d, %r13d
	xor %r14d, %r14d
	xor %r15d, %r15d

	/*
	 * Untrain the return predictor before any CALL or RET in the
	 * hypervisor. UNTRAIN_RET has three parts:
	 *
	 * 1) VALIDATE_UNRET_END: an objtool annotation (no runtime effect)
	 *    that satisfies objtool's UNRET validation.  UNWIND_HINT_ENTRY
	 *    above emits VALIDATE_UNRET_BEGIN which starts the validation walk;
	 *    VALIDATE_UNRET_END here marks the point where the walk succeeds,
	 *    preventing "RET before UNTRAIN" errors for subsequent CALLs.
	 *
	 * 2) CALL_UNTRAIN_RET: flushes the AMD return predictor state to
	 *    prevent the deprivileged host from using poisoned return predictor
	 *    entries to hijack speculative execution (AMD UNRET/SRSO).  Active
	 *    only when X86_FEATURE_UNRET or X86_FEATURE_SRSO_ALIAS is set;
	 *    NOPs on Intel.
	 *
	 * 3) ALTERNATIVE_2: either write_ibpb (gated on X86_FEATURE_ENTRY_IBPB
	 *    for IBPB-based retbleed mitigation) or RESET_CALL_DEPTH (gated on
	 *    X86_FEATURE_CALL_DEPTH, resets the call depth counter to
	 *    RET_DEPTH_INIT for correct RSB stuffing).
	 *
	 * On Intel platforms with eIBRS (required by pKVM), all three runtime
	 * alternatives are NOPs — eIBRS hardware handles return predictor
	 * protection on vmexit, IBPB is not required, and call depth tracking
	 * is inactive. UNTRAIN_RET is still required here solely for the
	 * VALIDATE_UNRET_END objtool annotation, and must precede the first
	 * CALL (vmx_spec_ctrl_restore_host).
	 *
	 * But in case the host kernel boots with retbleed=stuff which sets
	 * X86_FEATURE_CALL_DEPTH, RESET_CALL_DEPTH will become active to reset
	 * the call depth counter, ensuring RSB stuffing fires only when call
	 * depth tracking indicates the RSB may be exhausted.
	 */
	UNTRAIN_RET

	/* RDI: point to vcpu_vmx */
	mov WORD_SIZE*1(%_ASM_SP), %_ASM_DI
	mov $VMX_RUN_SAVE_SPEC_CTRL, %_ASM_SI

	call vmx_spec_ctrl_restore_host

	mov WORD_SIZE*1(%_ASM_SP), %_ASM_DI

	/* pKVM never enables interrupts */
	call pkvm_host_vmexit_main

	ALTERNATIVE "jmp .Lspec_ctrl_done", "", X86_FEATURE_MSR_SPEC_CTRL

	/*
	 * SPEC_CTRL handling: if the host's SPEC_CTRL value differs from the
	 * pkvm's, write the MSR.
	 *
	 * IMPORTANT: To avoid RSB underflow attacks and any other nastiness,
	 * there must not be any returns or indirect branches between this code
	 * and vmentry.
	 */
	mov WORD_SIZE*1(%_ASM_SP), %_ASM_DI
	movl VMX_spec_ctrl(%_ASM_DI), %edi
	movl PER_CPU_VAR(x86_spec_ctrl_current), %esi
	cmp %edi, %esi
	je .Lspec_ctrl_done
	mov $MSR_IA32_SPEC_CTRL, %ecx
	xor %edx, %edx
	mov %edi, %eax
	wrmsr

.Lspec_ctrl_done:
	/*
	 * Restore registers to reflect the vm-exit handling/emulation result
	 * before vmresume.
	 */
	mov VCPU_RAX(%_ASM_BX), %_ASM_AX
	mov VCPU_RCX(%_ASM_BX), %_ASM_CX
	mov VCPU_RDX(%_ASM_BX), %_ASM_DX
	mov VCPU_RSI(%_ASM_BX), %_ASM_SI
	mov VCPU_RDI(%_ASM_BX), %_ASM_DI
	mov VCPU_RBP(%_ASM_BX), %_ASM_BP
	mov VCPU_R8(%_ASM_BX),  %r8
	mov VCPU_R9(%_ASM_BX),  %r9
	mov VCPU_R10(%_ASM_BX), %r10
	mov VCPU_R11(%_ASM_BX), %r11
	mov VCPU_R12(%_ASM_BX), %r12
	mov VCPU_R13(%_ASM_BX), %r13
	mov VCPU_R14(%_ASM_BX), %r14
	mov VCPU_R15(%_ASM_BX), %r15
	mov VCPU_RBX(%_ASM_BX), %_ASM_BX

	CLEAR_CPU_BUFFERS

	/*
	 * The first time of entering guest is in local_deprivilege_cpu() which
	 * requires vmlaunch. This code path is called after that, hence only
	 * vmresume required.
	 */
	vmresume

	/* should not come here */
	mov $0x4400, %_ASM_CX
	vmread %_ASM_CX, %_ASM_AX
failed:
	jmp failed			/* should never come here */
SYM_FUNC_END(pkvm_host_vmexit_entry)
STACK_FRAME_NON_STANDARD(pkvm_host_vmexit_entry);
