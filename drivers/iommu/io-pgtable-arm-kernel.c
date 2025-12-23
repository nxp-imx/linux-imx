// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPU-agnostic ARM page table allocator.
 *
 * Copyright (C) 2014 ARM Limited
 *
 * Author: Will Deacon <will.deacon@arm.com>
 */
#include <linux/dma-mapping.h>

#include <linux/io-pgtable.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "io-pgtable-arm.h"
#include "iommu-pages.h"

static dma_addr_t __arm_lpae_dma_addr(void *pages)
{
	return (dma_addr_t)virt_to_phys(pages);
}

void *__arm_lpae_alloc_pages(size_t size, gfp_t gfp,
			     struct io_pgtable_cfg *cfg,
			     void *cookie)
{
	struct device *dev = cfg->iommu_dev;
	size_t alloc_size;
	dma_addr_t dma;
	void *pages;

	/*
	 * For very small starting-level translation tables the HW requires a
	 * minimum alignment of at least 64 to cover all cases.
	 */
	alloc_size = max(size, 64);
	if (cfg->alloc)
		pages = cfg->alloc(cookie, alloc_size, gfp);
	else
		pages = iommu_alloc_pages_node_sz(dev_to_node(dev), gfp,
						  alloc_size);

	if (!pages)
		return NULL;

	if (!cfg->coherent_walk) {
		dma = dma_map_single(dev, pages, size, DMA_TO_DEVICE);
		if (dma_mapping_error(dev, dma))
			goto out_free;
		/*
		 * We depend on the IOMMU being able to work with any physical
		 * address directly, so if the DMA layer suggests otherwise by
		 * translating or truncating them, that bodes very badly...
		 */
		if (dma != virt_to_phys(pages))
			goto out_unmap;
	}

	return pages;

out_unmap:
	dev_err(dev, "Cannot accommodate DMA translation for IOMMU page tables\n");
	dma_unmap_single(dev, dma, size, DMA_TO_DEVICE);

out_free:
	if (cfg->free)
		cfg->free(cookie, pages, size);
	else
		iommu_free_pages(pages);

	return NULL;
}

void __arm_lpae_free_pages(void *pages, size_t size,
			   struct io_pgtable_cfg *cfg,
			   void *cookie)
{
	if (!cfg->coherent_walk)
		dma_unmap_single(cfg->iommu_dev, __arm_lpae_dma_addr(pages),
				 size, DMA_TO_DEVICE);

	if (cfg->free)
		cfg->free(cookie, pages, size);
	else
		iommu_free_pages(pages);
}

void __arm_lpae_sync_pte(arm_lpae_iopte *ptep, int num_entries,
			 struct io_pgtable_cfg *cfg)
{
	dma_sync_single_for_device(cfg->iommu_dev, __arm_lpae_dma_addr(ptep),
				   sizeof(*ptep) * num_entries, DMA_TO_DEVICE);
}

void *__arm_lpae_alloc_data(struct io_pgtable_cfg *cfg, size_t size, gfp_t gfp)
{
	return kmalloc(size, gfp);
}

void __arm_lpae_free_data(struct io_pgtable_cfg *cfg, void *p)
{
	return kfree(p);
}
