/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/linkage.h>
#include <asm/asm.h>
#include <asm/cache.h>
#include <asm/nospec-branch.h>
#include <asm/segment.h>
#include <asm/trapnr.h>
#include <asm/unwind_hints.h>

.macro save_frame
	push %_ASM_DI
	push %_ASM_SI
	push %_ASM_DX
	push %_ASM_CX
	push %_ASM_AX
	push %r8
	push %r9
	push %r10
	push %r11
	push %_ASM_BX
	push %_ASM_BP
	push %r12
	push %r13
	push %r14
	push %r15
.endm

.macro restore_frame
	pop %r15
	pop %r14
	pop %r13
	pop %r12
	pop %_ASM_BP
	pop %_ASM_BX
	pop %r11
	pop %r10
	pop %r9
	pop %r8
	pop %_ASM_AX
	pop %_ASM_CX
	pop %_ASM_DX
	pop %_ASM_SI
	pop %_ASM_DI
.endm

.macro exception_entry name vector has_error_code
SYM_CODE_START(handle_exception_\name)
	UNWIND_HINT_UNDEFINED
	ENDBR

	.if \has_error_code == 0
		pushq   $-1
	.endif

	save_frame

	movq %rsp, %rdi
	movq $\vector, %rsi
	movq $\has_error_code, %rdx

	call handle_exception

	restore_frame

	/* Pop error code unconditionally as iretq won't do that */
	addq    $8, %rsp

	iretq
SYM_CODE_END(handle_exception_\name)
.endm

#define GEN(v, e)	exception_entry #v vector=v has_error_code=e
#include <asm/GEN-for-each-exc.h>
#undef GEN

.pushsection .pkvm.text, "ax"

.align L1_CACHE_BYTES, 0xcc
SYM_CODE_START_NOALIGN(x86_verw_sel)
	UNWIND_HINT_UNDEFINED
	ANNOTATE_NOENDBR
	.word __KERNEL_DS
.align L1_CACHE_BYTES, 0xcc
SYM_CODE_END(x86_verw_sel);

.popsection

/* Clobbers AX, CX, DX */
SYM_FUNC_START(write_ibpb)
	ANNOTATE_NOENDBR
	movl	$MSR_IA32_PRED_CMD, %ecx
	movl	_ASM_RIP(x86_pred_cmd), %eax
	xorl	%edx, %edx
	wrmsr

	/* Make sure IBPB clears return stack preductions too. */
	FILL_RETURN_BUFFER %rax, RSB_CLEAR_LOOPS, X86_BUG_IBPB_NO_RET
	RET
SYM_FUNC_END(write_ibpb)

#ifdef CONFIG_X86_64
/*
 * This sequence executes branches in order to remove user branch information
 * from the branch history tracker in the Branch Predictor, therefore removing
 * user influence on subsequent BTB lookups.
 *
 * It should be used on parts prior to Alder Lake. Newer parts should use the
 * BHI_DIS_S hardware control instead. If a pre-Alder Lake part is being
 * virtualized on newer hardware the VMM should protect against BHI attacks by
 * setting BHI_DIS_S for the guests.
 *
 * CALLs/RETs are necessary to prevent Loop Stream Detector(LSD) from engaging
 * and not clearing the branch history. The call tree looks like:
 *
 * call 1
 *    call 2
 *      call 2
 *        call 2
 *          call 2
 * 	      call 2
 * 	      ret
 * 	    ret
 *        ret
 *      ret
 *    ret
 * ret
 *
 * This means that the stack is non-constant and ORC can't unwind it with %rsp
 * alone.  Therefore we unconditionally set up the frame pointer, which allows
 * ORC to unwind properly.
 *
 * The alignment is for performance and not for safety, and may be safely
 * refactored in the future if needed. The .skips are for safety, to ensure
 * that all RETs are in the second half of a cacheline to mitigate Indirect
 * Target Selection, rather than taking the slowpath via its_return_thunk.
 */
SYM_FUNC_START(clear_bhb_loop)
	ANNOTATE_NOENDBR
	push	%rbp
	mov	%rsp, %rbp
	movl	$5, %ecx
	ANNOTATE_INTRA_FUNCTION_CALL
	call	1f
	jmp	5f
	.align 64, 0xcc
	/*
	 * Shift instructions so that the RET is in the upper half of the
	 * cacheline and don't take the slowpath to its_return_thunk.
	 */
	.skip 32 - (.Lret1 - 1f), 0xcc
	ANNOTATE_INTRA_FUNCTION_CALL
1:	call	2f
.Lret1:	RET
	.align 64, 0xcc
	/*
	 * As above shift instructions for RET at .Lret2 as well.
	 *
	 * This should be ideally be: .skip 32 - (.Lret2 - 2f), 0xcc
	 * but some Clang versions (e.g. 18) don't like this.
	 */
	.skip 32 - 18, 0xcc
2:	movl	$5, %eax
3:	jmp	4f
	nop
4:	sub	$1, %eax
	jnz	3b
	sub	$1, %ecx
	jnz	1b
.Lret2:	RET
5:	lfence
	pop	%rbp
	RET
SYM_FUNC_END(clear_bhb_loop)
STACK_FRAME_NON_STANDARD(clear_bhb_loop)
#endif /* CONFIG_X86_64 */
