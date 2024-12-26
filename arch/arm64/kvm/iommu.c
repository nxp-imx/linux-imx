// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Google LLC
 * Author: Mostafa Saleh <smostafa@google.com>
 */

#include <asm/kvm_mmu.h>

#include <kvm/iommu.h>

#include <linux/kvm_host.h>

struct kvm_iommu_driver *iommu_driver;
extern struct kvm_iommu_ops *kvm_nvhe_sym(kvm_iommu_ops);

int kvm_iommu_register_driver(struct kvm_iommu_driver *kern_ops, struct kvm_iommu_ops *el2_ops)
{
	int ret;

	if (WARN_ON(!kern_ops || !el2_ops))
		return -EINVAL;

	/*
	 * Paired with smp_load_acquire(&iommu_driver)
	 * Ensure memory stores happening during a driver
	 * init are observed before executing kvm iommu callbacks.
	 */
	ret = cmpxchg_release(&iommu_driver, NULL, kern_ops) ? -EBUSY : 0;
	if (ret)
		return ret;

	kvm_nvhe_sym(kvm_iommu_ops) = el2_ops;
	return 0;
}

int kvm_iommu_init_driver(void)
{
	if (!smp_load_acquire(&iommu_driver)) {
		kvm_err("pKVM enabled without an IOMMU driver, do not run confidential workloads in virtual machines\n");
		return 0;
	}

	kvm_hyp_iommu_domains = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO,
				get_order(KVM_IOMMU_DOMAINS_ROOT_SIZE));
	if (!kvm_hyp_iommu_domains)
		return -ENOMEM;

	kvm_hyp_iommu_domains = kern_hyp_va(kvm_hyp_iommu_domains);

	return iommu_driver->init_driver();
}

void kvm_iommu_remove_driver(void)
{
	if (smp_load_acquire(&iommu_driver))
		iommu_driver->remove_driver();
}
