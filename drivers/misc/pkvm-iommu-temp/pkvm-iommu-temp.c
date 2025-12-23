// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 - Google Inc
 * Author: Mostafa Saleh <smostafa@google.com>
 * Template module for pKVM IOMMU drivers
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/kvm_pkvm_module.h>

#include "pkvm/pkvm-iommu-temp.h"

#define ksym_ref_addr_nvhe(x) \
	((typeof(kvm_nvhe_sym(x)) *)(pkvm_el2_mod_va(&kvm_nvhe_sym(x))))

static int kvm_iommu_temp_init_drv(void)
{
	int ret = 0;
	pkvm_handle_t hyp_drv_id;

	ret = pkvm_load_el2_module(kvm_nvhe_sym(pkvm_iommu_temp_hyp_init));
	if (ret)
		pr_err("Failed to register pKVM IOMMU template: %d\n", ret);

	return kvm_iommu_register_hyp_ops(ksym_ref_addr_nvhe(iommu_temp_ops), &hyp_drv_id);
}


static struct kvm_iommu_driver kvm_iommu_temp_ops = {
	.init_driver = kvm_iommu_temp_init_drv,
};

static int __init pkvm_iommu_temp_init(void)
{
	return kvm_iommu_register_driver(&kvm_iommu_temp_ops, 1);
}

module_init(pkvm_iommu_temp_init);

MODULE_AUTHOR("Mostafa Saleh <smostafa@google.com>");
MODULE_DESCRIPTION("pKVM IOMMU template");
MODULE_LICENSE("GPL v2");
