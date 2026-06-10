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
	ENDBR
	push %_ASM_BX
	mov WORD_SIZE*1(%_ASM_SP), %_ASM_BX

	/*
	 * No need for FILL_RETURN_BUFFER since there is no unbalanced RET
	 * after host vmexit.
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
