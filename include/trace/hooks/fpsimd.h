/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM fpsimd

#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_FPSIMD_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_FPSIMD_H

#include <trace/hooks/vendor_hooks.h>

struct task_struct;

DECLARE_HOOK(android_vh_is_fpsimd_save,
	TP_PROTO(struct task_struct *prev, struct task_struct *next),
	TP_ARGS(prev, next))

/*
 * Vendor hook fired after the first SME access trap (EC=0x1D) is
 * handled and TIF_SME is set. Fired exactly once per task when SME
 * state is fully initialised and the task is ready to use SME.
 */
DECLARE_RESTRICTED_HOOK(android_rvh_sme_smstart,
	TP_PROTO(struct task_struct *task),
	TP_ARGS(task), 1)

#endif /* _TRACE_HOOK_FPSIMD_H */
/* This part must be outside protection */
#include <trace/define_trace.h>
