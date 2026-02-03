// SPDX-License-Identifier: GPL-2.0
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/extable.h>
#include <asm/processor.h>
#include <asm/extable.h>
#include <asm/pkvm_image.h>
#include <asm/trapnr.h>
#include "debug.h"
#include "idt.h"

static const int pt_regoff[] = {
	offsetof(struct pt_regs, ax),
	offsetof(struct pt_regs, cx),
	offsetof(struct pt_regs, dx),
	offsetof(struct pt_regs, bx),
	offsetof(struct pt_regs, sp),
	offsetof(struct pt_regs, bp),
	offsetof(struct pt_regs, si),
	offsetof(struct pt_regs, di),
#ifdef CONFIG_X86_64
	offsetof(struct pt_regs, r8),
	offsetof(struct pt_regs, r9),
	offsetof(struct pt_regs, r10),
	offsetof(struct pt_regs, r11),
	offsetof(struct pt_regs, r12),
	offsetof(struct pt_regs, r13),
	offsetof(struct pt_regs, r14),
	offsetof(struct pt_regs, r15),
#else
	offsetof(struct pt_regs, ds),
	offsetof(struct pt_regs, es),
	offsetof(struct pt_regs, fs),
	offsetof(struct pt_regs, gs),
#endif
};

static int pt_regs_offset(struct pt_regs *regs, int regno)
{
	if ((unsigned)regno < ARRAY_SIZE(pt_regoff))
		return pt_regoff[regno];
	return -EDOM;
}

static inline unsigned long
ex_fixup_addr(const struct exception_table_entry *x)
{
	return (unsigned long)&x->fixup + x->fixup;
}

static inline unsigned long *pt_regs_nr(struct pt_regs *regs, int nr)
{
	int reg_offset = pt_regs_offset(regs, nr);
	static unsigned long __dummy;

	if (WARN_ON_ONCE(reg_offset < 0))
		return &__dummy;

	return (unsigned long *)((unsigned long)regs + reg_offset);
}

static bool ex_handler_default(const struct exception_table_entry *e,
			       struct pt_regs *regs)
{
	if (e->data & EX_FLAG_CLEAR_AX)
		regs->ax = 0;
	if (e->data & EX_FLAG_CLEAR_DX)
		regs->dx = 0;

	regs->ip = ex_fixup_addr(e);
	return true;
}

static bool ex_handler_msr(const struct exception_table_entry *fixup,
			   struct pt_regs *regs, bool wrmsr, bool safe, int reg)
{
#ifdef CONFIG_PKVM_X86_DEBUG
	if (__ONCE_LITE_IF(!safe && wrmsr)) {
		pkvm_err("unchecked MSR access error: WRMSR to 0x%x (tried to write 0x%08x%08x) at rIP: 0x%lx (%pS)\n",
			(unsigned int)regs->cx, (unsigned int)regs->dx,
			(unsigned int)regs->ax,  regs->ip, (void *)regs->ip);
	}

	if (__ONCE_LITE_IF(!safe && !wrmsr)) {
		pkvm_err("unchecked MSR access error: RDMSR from 0x%x at rIP: 0x%lx (%pS)\n",
			(unsigned int)regs->cx, regs->ip, (void *)regs->ip);
	}
#endif

	if (!wrmsr) {
		/* Pretend that the read succeeded and returned 0. */
		regs->ax = 0;
		regs->dx = 0;
	}

	if (safe)
		*pt_regs_nr(regs, reg) = -EIO;

	return ex_handler_default(fixup, regs);
}

static bool pkvm_fixup_exception(struct pt_regs *regs)
{
	const struct exception_table_entry *e;
	int type, reg;

	e = search_extable(__start___ex_table,
			   __stop___ex_table - __start___ex_table,
			   regs->ip);
	if (!e)
		return false;

	type = FIELD_GET(EX_DATA_TYPE_MASK, e->data);
	reg  = FIELD_GET(EX_DATA_REG_MASK,  e->data);

	switch (type) {
	case EX_TYPE_DEFAULT:
		return ex_handler_default(e, regs);
	case EX_TYPE_WRMSR:
		return ex_handler_msr(e, regs, true, false, reg);
	case EX_TYPE_RDMSR:
		return ex_handler_msr(e, regs, false, false, reg);
	case EX_TYPE_WRMSR_SAFE:
		return ex_handler_msr(e, regs, true, true, reg);
	case EX_TYPE_RDMSR_SAFE:
		return ex_handler_msr(e, regs, false, true, reg);
	default:
		BUG();
	}
}

static void default_exception_handler(struct pt_regs *regs,
				      int vector, bool has_error_code)
{
	if (has_error_code)
		pkvm_err("Exception %d @ip %pS (0x%px), err code 0x%lx\n",
			 vector, (void *)regs->ip, (void *)regs->ip, regs->orig_ax);
	else
		pkvm_err("Exception %d @ip %pS (0x%px), no err code\n",
			 vector, (void *)regs->ip, (void *)regs->ip);

	asm volatile("hlt" : : : "memory");
}

static exception_handler_t exception_handlers[X86_TRAP_IRET] = {
#define GEN(x, ...)	\
		[x] = default_exception_handler,
#include <asm/GEN-for-each-exc.h>
#undef GEN
};

void handle_exception(struct pt_regs *regs, int vector, bool has_error_code)
{
	exception_handler_t handler;

	if (vector >= X86_TRAP_IRET)
		return;

	if (pkvm_fixup_exception(regs))
		return;

	handler = exception_handlers[vector];
	handler(regs, vector, has_error_code);
}

void pkvm_register_excp_handler(int vector, exception_handler_t handler)
{
	if (vector >= X86_TRAP_IRET)
		return;

	exception_handlers[vector] = handler;
}
