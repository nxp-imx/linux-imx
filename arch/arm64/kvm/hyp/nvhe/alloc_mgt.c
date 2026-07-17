// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allocator abstraction for the hypervisor.
 * Copyright (C) 2023 Google LLC
 * Author: Mostafa Saleh <smostafa@google.com>
 */

#include <nvhe/alloc.h>
#include <nvhe/alloc_mgt.h>
#include <nvhe/iommu.h>
#include <nvhe/mem_protect.h>

static struct hyp_mgt_allocator_ops *registered_allocators[] = {
	[HYP_ALLOC_MGT_HEAP_ID] = &hyp_alloc_ops,
	[HYP_ALLOC_MGT_IOMMU_ID] = &kvm_iommu_allocator_ops,
	[HYP_ALLOC_MGT_HOSTS2_ID] = &host_s2_pool_ops,
};

int hyp_alloc_mgt_refill(enum hyp_alloc_mgt_id id, struct kvm_hyp_memcache *host_mc)
{
	struct hyp_mgt_allocator_ops *ops;

	if (id >= NR_ALLOC_MGT_IDS)
		return -EINVAL;

	id = array_index_nospec(id, NR_ALLOC_MGT_IDS);

	BUILD_BUG_ON(ARRAY_SIZE(registered_allocators) != NR_ALLOC_MGT_IDS);
	ops = registered_allocators[id];

	return ops->refill ? ops->refill(host_mc) : 0;
}

int hyp_alloc_mgt_reclaimable(enum hyp_alloc_mgt_id id)
{
	struct hyp_mgt_allocator_ops *ops;

	if (id >= NR_ALLOC_MGT_IDS)
		return 0;

	id = array_index_nospec(id, NR_ALLOC_MGT_IDS);
	ops = registered_allocators[id];

	return ops->reclaimable ? ops->reclaimable() : 0;
}

void hyp_alloc_mgt_reclaim(enum hyp_alloc_mgt_id id, struct kvm_hyp_memcache *host_mc, int target)
{
	struct hyp_mgt_allocator_ops *ops;

	if (id >= NR_ALLOC_MGT_IDS)
		return;

	id = array_index_nospec(id, NR_ALLOC_MGT_IDS);

	BUILD_BUG_ON(ARRAY_SIZE(registered_allocators) != NR_ALLOC_MGT_IDS);
	ops = registered_allocators[id];

	/* Not fair but OK for now. */
	if (ops->reclaim)
		ops->reclaim(host_mc, target);
}
