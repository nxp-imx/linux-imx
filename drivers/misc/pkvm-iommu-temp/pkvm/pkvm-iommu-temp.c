// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 - Google Inc
 * Author: Mostafa Saleh <smostafa@google.com>
 * Template module for pKVM IOMMU drivers
 */

#include <asm/kvm_pkvm_module.h>

#include <nvhe/iommu.h>

#include "pkvm-iommu-temp.h"

const struct pkvm_module_ops *pkvm_ops;

static int iommu_temp_init(pkvm_handle_t drv_id)
{
	/* Any common driver init. */
	return 0;
}

static void iommu_temp_host_stage2_idmap(phys_addr_t start, phys_addr_t end, int prot)
{
	/* Update the IOMMU/MPU page table. */
}

static bool iommu_temp_dabt_handler(struct user_pt_regs *regs, u64 esr, u64 addr)
{
	/* Check if data abort belongs to a device managed by this driver */
	return false;
}

static int iommu_temp_set_identity(pkvm_handle_t iommu, pkvm_handle_t sid, bool on)
{
	/* Set a device in either identity or blocking state. */
	return -ENODEV;
}

static int iommu_temp_alloc_domain(pkvm_handle_t iommu,
			     struct kvm_hyp_iommu_domain *domain, int type)
{
	/* Allocate a pv page table. */
	return -ENODEV;
}

static void iommu_temp_free_domain(struct kvm_hyp_iommu_domain *domain)
{
	/* Free the pv page table. */
}

static phys_addr_t iommu_temp_iova_to_phys(struct kvm_hyp_iommu_domain *domain,
				     unsigned long iova)
{
	/* Convert IOVA to physical address. */
	return 0;
}

static size_t iommu_temp_unmap_pages(struct kvm_hyp_iommu_domain *domain, unsigned long iova,
			       size_t pgsize, size_t pgcount, struct iommu_iotlb_gather *gather)
{
	/*
	 * Unmap pages from a pv domain and return bytes unmapped.
	 * NOTE: This function must call __pkvm_host_unuse_dma() for all physical
	 * addresses AFTER they are unmapped and TLB was invalidated, check pkvm-iommu.rst
	 */
	return 0;
}

static int iommu_temp_map_pages(struct kvm_hyp_iommu_domain *domain, unsigned long iova,
			  phys_addr_t paddr, size_t pgsize,
			  size_t pgcount, int prot, size_t *total_mapped)
{
	/*
	 * Map pages in a pv domain.
	 * NOTE: This function must call __pkvm_host_use_dma() for all physical
	 * addresses BEFORE they are mapped, check pkvm-iommu.rst
	 */
	return -ENODEV;
}

static int iommu_temp_attach_dev(pkvm_handle_t iommu, struct kvm_hyp_iommu_domain *domain,
			   u32 sid, u32 pasid, u32 pasid_bits, unsigned long flags)
{
	/* Attach a device to a pv domain. */
	return -ENODEV;
}

static int iommu_temp_detach_dev(pkvm_handle_t iommu, struct kvm_hyp_iommu_domain *domain,
			   u32 sid, u32 pasid)
{
	/* Detach a device from a pv domain. */
	return -ENODEV;
}


static void iommu_temp_iotlb_sync(struct kvm_hyp_iommu_domain *domain,
			    struct iommu_iotlb_gather *gather)
{
	/* Sync a gather list after an unmap. */
}

struct kvm_iommu_ops iommu_temp_ops = {
	/* Common ops */
	.init                           = iommu_temp_init,
	.dabt_handler			= iommu_temp_dabt_handler,
	.host_stage2_idmap		= iommu_temp_host_stage2_idmap,
	.set_identity			= iommu_temp_set_identity,

	/* para-virtual ops, optional */
	.alloc_domain			= iommu_temp_alloc_domain,
	.free_domain			= iommu_temp_free_domain,
	.iotlb_sync			= iommu_temp_iotlb_sync,
	.attach_dev			= iommu_temp_attach_dev,
	.detach_dev			= iommu_temp_detach_dev,
	.map_pages			= iommu_temp_map_pages,
	.unmap_pages			= iommu_temp_unmap_pages,
	.iova_to_phys			= iommu_temp_iova_to_phys,
};

int pkvm_iommu_temp_hyp_init(const struct pkvm_module_ops *ops)
{
	pkvm_ops = ops;
	return 0;
}
