/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM ttm_pool

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_TTM_POOL_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_TTM_POOL_H
#include <trace/hooks/vendor_hooks.h>

struct gfp_t;

DECLARE_HOOK(android_vh_ttm_pool_alloc_max_page_order,
	     TP_PROTO(unsigned int *max_page_order),
	     TP_ARGS(max_page_order));

DECLARE_HOOK(android_vh_ttm_pool_alloc_page_flags,
	     TP_PROTO(unsigned int order, gfp_t *gfp_flags),
	     TP_ARGS(order, gfp_flags));

#endif /* _TRACE_HOOK_TTM_POOL_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
