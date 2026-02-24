// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Arm Ltd.
 */
#include <nvhe/alloc.h>
#include <nvhe/iommu.h>

#include <linux/io-pgtable.h>
#include "../../../io-pgtable-arm.h"
#include "arm-smmu-v3-module.h"

struct io_pgtable_ops *kvm_alloc_io_pgtable_ops(enum io_pgtable_fmt fmt,
						struct io_pgtable_cfg *cfg,
						void *cookie)
{
	struct io_pgtable *iop;

	if (fmt == ARM_64_LPAE_S2)
		iop = arm_64_lpae_alloc_pgtable_s2(cfg, cookie);
	else if (fmt == ARM_64_LPAE_S1)
		iop = arm_64_lpae_alloc_pgtable_s1(cfg, cookie);
	else
		return NULL;

	if (!iop)
		return NULL;

	iop->fmt	= fmt;
	iop->cookie	= cookie;
	iop->cfg	= *cfg;

	return &iop->ops;
}

void kvm_arm_io_pgtable_free(struct io_pgtable *iopt)
{
	io_pgtable_tlb_flush_all(iopt);
	arm_lpae_free_pgtable(iopt);
}

void *__arm_lpae_alloc_pages(size_t size, gfp_t gfp,
			     struct io_pgtable_cfg *cfg, void *cookie)
{
	void *addr;

	if (cfg->quirks & IO_PGTABLE_QUIRK_IDMAP)
		addr = kvm_iommu_donate_pages_atomic(get_order(size));
	else
		addr = kvm_iommu_donate_pages(get_order(size), 0);

	if (addr && !cfg->coherent_walk)
		kvm_flush_dcache_to_poc(addr, size);

	return addr;
}

void __arm_lpae_free_pages(void *addr, size_t size, struct io_pgtable_cfg *cfg,
			   void *cookie)
{
	if (!cfg->coherent_walk)
		kvm_flush_dcache_to_poc(addr, size);

	if (cfg->quirks & IO_PGTABLE_QUIRK_IDMAP)
		kvm_iommu_reclaim_pages_atomic(addr);
	else
		kvm_iommu_reclaim_pages(addr, get_order(size));
}

void __arm_lpae_sync_pte(arm_lpae_iopte *ptep, int num_entries,
			 struct io_pgtable_cfg *cfg)
{
	if (!cfg->coherent_walk)
		kvm_flush_dcache_to_poc(ptep, sizeof(*ptep) * num_entries);
}

/* At the moment this is only used once, so rounding up to a page is not really a problem. */
void *__arm_lpae_alloc_data(struct io_pgtable_cfg *cfg, size_t size, gfp_t gfp)
{
	void *p;

	if (size > PAGE_SIZE)
		return NULL;

	if (cfg->quirks & IO_PGTABLE_QUIRK_IDMAP)
		return kvm_iommu_donate_pages_atomic(get_order(size));

	p = hyp_alloc(size);
	if (!p)
		kvm_iommu_request_hyp_alloc();

	return p;
}

void __arm_lpae_free_data(struct io_pgtable_cfg *cfg, void *p)
{
	if (cfg->quirks & IO_PGTABLE_QUIRK_IDMAP)
		kvm_iommu_reclaim_pages_atomic(p);
	else
		hyp_free(p);
}

#if IS_ENABLED(CONFIG_IOMMUFD_DRIVER)
void iova_bitmap_set(struct iova_bitmap *bitmap,
		     unsigned long iova, size_t length)
{
	/* Dirty bit not supported */
	BUG();
}
#endif
