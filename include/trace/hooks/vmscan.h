/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM vmscan

#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_VMSCAN_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_VMSCAN_H

#include <trace/hooks/vendor_hooks.h>

DECLARE_RESTRICTED_HOOK(android_rvh_set_balance_anon_file_reclaim,
			TP_PROTO(bool *balance_anon_file_reclaim),
			TP_ARGS(balance_anon_file_reclaim), 1);
DECLARE_HOOK(android_vh_mglru_should_abort_scan,
	TP_PROTO(unsigned long nr_reclaimed, unsigned long nr_to_reclaim,
	unsigned int order, bool *bypass),
	TP_ARGS(nr_to_reclaim, nr_reclaimed, order, bypass));
DECLARE_HOOK(android_vh_tune_swappiness,
	TP_PROTO(int *swappiness),
	TP_ARGS(swappiness));
DECLARE_HOOK(android_vh_async_psi_bypass,
	TP_PROTO(bool *bypass),
	TP_ARGS(bypass));
DECLARE_HOOK(android_vh_handle_folio_writeback,
	TP_PROTO(struct folio *folio, bool *bypass),
	TP_ARGS(folio, bypass));
DECLARE_HOOK(android_vh_reclaim_before_kswapd,
	TP_PROTO(unsigned long *nr_reclaimed),
	TP_ARGS(nr_reclaimed));
DECLARE_HOOK(android_vh_page_referenced_check_bypass,
	TP_PROTO(struct folio *folio, unsigned long nr_to_scan, int lru, bool *bypass),
	TP_ARGS(folio, nr_to_scan, lru, bypass));
enum scan_balance;
DECLARE_HOOK(android_vh_tune_scan_type,
	TP_PROTO(enum scan_balance *scan_type),
	TP_ARGS(scan_type));
DECLARE_HOOK(android_vh_shrink_slab_bypass,
	TP_PROTO(gfp_t gfp_mask, int nid, struct mem_cgroup *memcg, int priority, bool *bypass),
	TP_ARGS(gfp_mask, nid, memcg, priority, bypass));
DECLARE_HOOK(android_vh_throttle_direct_reclaim_bypass,
	TP_PROTO(bool *bypass),
	TP_ARGS(bypass));
DECLARE_HOOK(android_vh_check_folio_look_around_ref,
	TP_PROTO(struct folio *folio, int *skip),
	TP_ARGS(folio, skip));
DECLARE_HOOK(android_vh_should_memcg_bypass,
	TP_PROTO(struct mem_cgroup *memcg, int priority, bool *bypass),
	TP_ARGS(memcg, priority, bypass));
DECLARE_HOOK(android_vh_vmscan_kswapd_done,
	TP_PROTO(int node_id, unsigned int highest_zoneidx, unsigned int alloc_order,
		unsigned int reclaim_order),
	TP_ARGS(node_id, highest_zoneidx, alloc_order, reclaim_order));
DECLARE_HOOK(android_vh_remove_mapping,
	TP_PROTO(struct address_space *mapping, struct folio *folio, bool reclaimed),
	TP_ARGS(mapping, folio, reclaimed));
DECLARE_HOOK(android_vh_remove_mapping_failed,
	TP_PROTO(struct address_space *mapping, struct folio *folio, bool reclaimed),
	TP_ARGS(mapping, folio, reclaimed));
DECLARE_HOOK(android_vh_rebalance_anon_lru_bypass,
	TP_PROTO(bool *bypass),
	TP_ARGS(bypass));
DECLARE_HOOK(android_vh_tune_scan_control,
	TP_PROTO(bool *skip_swap),
	TP_ARGS(skip_swap));
#endif /* _TRACE_HOOK_VMSCAN_H */
/* This part must be outside protection */
#include <trace/define_trace.h>
