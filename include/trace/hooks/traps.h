/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM traps
#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_TRAPS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_TRAPS_H

#include <trace/hooks/vendor_hooks.h>

struct pt_regs;

DECLARE_RESTRICTED_HOOK(android_rvh_do_el1_undef,
	TP_PROTO(struct pt_regs *regs, unsigned long esr),
	TP_ARGS(regs, esr), 1);

DECLARE_RESTRICTED_HOOK(android_rvh_do_el1_bti,
	TP_PROTO(struct pt_regs *regs, unsigned long esr),
	TP_ARGS(regs, esr), 1);

DECLARE_RESTRICTED_HOOK(android_rvh_do_el1_fpac,
	TP_PROTO(struct pt_regs *regs, unsigned long esr),
	TP_ARGS(regs, esr), 1);

DECLARE_RESTRICTED_HOOK(android_rvh_panic_unhandled,
	TP_PROTO(struct pt_regs *regs, const char *vector, unsigned long esr),
	TP_ARGS(regs, vector, esr), 1);

DECLARE_RESTRICTED_HOOK(android_rvh_handle_bad_stack,
	TP_PROTO(struct pt_regs *regs, unsigned long esr, unsigned long far),
	TP_ARGS(regs, esr, far), 1);

DECLARE_RESTRICTED_HOOK(android_rvh_arm64_serror_panic,
	TP_PROTO(struct pt_regs *regs, unsigned long esr),
	TP_ARGS(regs, esr), 1);

/*
 * Vendor hook fired for all unrecognised EL0 sync exceptions before
 * bad_el0_sync() is called. The full ESR is passed so vendor drivers
 * can inspect EC, IL and the trapped opcode (ESR_ELx[63:32]).
 * Set *handled = true to suppress bad_el0_sync() / SIGILL delivery.
 * If no handler is registered or *handled remains false, SIGILL is
 * delivered as usual.
 */
DECLARE_RESTRICTED_HOOK(android_rvh_el0_impdef_exception,
	TP_PROTO(struct pt_regs *regs, unsigned long esr, bool *handled),
	TP_ARGS(regs, esr, handled), 1)

#endif /* _TRACE_HOOK_TRAPS_H */
/* This part must be outside protection */
#include <trace/define_trace.h>
