// SPDX-License-Identifier: GPL-2.0
/*
 * pKVM host driver for the Arm SMMUv3
 *
 * Copyright (C) 2022 Linaro Ltd.
 */
#include <asm/kvm_mmu.h>
#include <asm/kvm_pkvm.h>

#include <linux/auxiliary_bus.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "arm-smmu-v3.h"
#include "pkvm/arm_smmu_v3.h"

#define SMMU_KVM_CMDQ_ORDER				4
#define SMMU_KVM_STRTAB_ORDER				(get_order(STRTAB_MAX_L1_ENTRIES * \
							 sizeof(struct arm_smmu_strtab_l1)))

extern struct kvm_iommu_ops kvm_nvhe_sym(smmu_ops);

static size_t				kvm_arm_smmu_count;
static struct hyp_arm_smmu_v3_nested_device	*kvm_arm_smmu_array;
static size_t				kvm_arm_smmu_cur;

#ifdef MODULE
#define ksym_ref_addr_nvhe(x) \
	((typeof(kvm_nvhe_sym(x)) *)(pkvm_el2_mod_va(&kvm_nvhe_sym(x))))

int kvm_nvhe_sym(smmu_init_hyp_module)(const struct pkvm_module_ops *ops);
#else
#define ksym_ref_addr_nvhe(x) \
	((typeof(kvm_nvhe_sym(x)) *)(kern_hyp_va(lm_alias(&kvm_nvhe_sym(x)))))
#endif

static void kvm_arm_smmu_array_free(void)
{
	int order;

	order = get_order(kvm_arm_smmu_count * sizeof(*kvm_arm_smmu_array));
	free_pages((unsigned long)kvm_arm_smmu_array, order);
}

/*
 * The hypervisor have to know the basic information about the SMMUs
 * from the firmware.
 * This has to be done before the SMMUv3 probes and does anything meaningful
 * with the hardware, otherwise it becomes harder to reason about the SMMU
 * state and we'd require to hand-off the state to the hypervisor at certain point
 * while devices are live, which is complicated and dangerous.
 * Instead, the hypervisor is interested in a very small part of the probe path,
 * so just add a separate logic for it.
 */
static int kvm_arm_smmu_array_alloc(void)
{
	int smmu_order;
	struct device_node *np;

	for_each_compatible_node(np, NULL, "arm,smmu-v3")
		kvm_arm_smmu_count++;

	if (!kvm_arm_smmu_count)
		return -ENODEV;
	smmu_order = get_order(kvm_arm_smmu_count * sizeof(*kvm_arm_smmu_array));
	kvm_arm_smmu_array = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, smmu_order);
	if (!kvm_arm_smmu_array)
		return -ENOMEM;
	return 0;
}

static size_t smmu_hyp_pgt_pages(void)
{
	/*
	 * SMMUv3 uses the same format as stage-2 and hence have the same memory
	 * requirements, we add extra 500 pages for L2 ste.
	 * For modules, we can't use host_s2_pgtable_pages(), so we return 1 page,
	 * and rely on the vendor passing the right commandline arg.
	 */
	if (of_find_compatible_node(NULL, NULL, "arm,smmu-v3")) {
#ifdef MODULE
		return 1;
#else
		return host_s2_pgtable_pages() + 500;
#endif
	}
	return 0;
}

static struct platform_driver smmuv3_nesting_driver;
static int smmuv3_nesting_probe(struct platform_device *pdev)
{
	struct resource *res;
	void *cmdq_base, *strtab;
	struct hyp_arm_smmu_v3_nested_device *smmu = &kvm_arm_smmu_array[kvm_arm_smmu_cur];

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	smmu->common.mmio_addr = res->start;
	smmu->common.mmio_size = resource_size(res);
	if (smmu->common.mmio_size < SZ_128K) {
		dev_err(&pdev->dev, "MMIO region too small(%pr)\n", &res);
		return -EINVAL;
	}

	if (of_dma_is_coherent(pdev->dev.of_node))
		smmu->common.features |= ARM_SMMU_FEAT_COHERENCY;

	/*
	 * Allocate the shadow command queue, it doesn't have to be the same
	 * size as the host.
	 * Only populate base_dma and llq.max_n_shift, the hypervisor will init
	 * the rest.
	 */
	cmdq_base = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, SMMU_KVM_CMDQ_ORDER);
	if (!cmdq_base)
		return -ENOMEM;

	smmu->common.cmdq.base_dma = virt_to_phys(cmdq_base);
	smmu->common.cmdq.llq.max_n_shift = SMMU_KVM_CMDQ_ORDER + PAGE_SHIFT - CMDQ_ENT_SZ_SHIFT;

	strtab = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, SMMU_KVM_STRTAB_ORDER);
	if (!strtab)
		return -ENOMEM;

	smmu->strtab_dma = virt_to_phys(strtab);
	smmu->strtab_size = PAGE_SIZE << SMMU_KVM_STRTAB_ORDER;

	kvm_arm_smmu_cur++;
	return 0;
}

static int kvm_arm_smmu_v3_post_init(void);

static int kvm_arm_smmu_v3_init(void)
{
	int ret;
	pkvm_handle_t hyp_drv_id;

#ifdef MODULE
	ret = pkvm_load_el2_module(kvm_nvhe_sym(smmu_init_hyp_module));

	if (ret) {
		pr_err("Failed to load SMMUv3 IOMMU EL2 module: %d\n", ret);
		return ret;
	}
#endif

	ret = kvm_iommu_register_hyp_ops(ksym_ref_addr_nvhe(smmu_ops), &hyp_drv_id);
	if (ret)
		return ret;

#ifdef MODULE
	return kvm_arm_smmu_v3_post_init();
#endif
	return 0;
}

struct kvm_iommu_driver kvm_smmu_v3_ops = {
	.init_driver = kvm_arm_smmu_v3_init,
};

static int kvm_arm_smmu_v3_register(void)
{
	size_t nr_pages = smmu_hyp_pgt_pages();
	int ret;

	if (!is_protected_kvm_enabled() || !nr_pages)
		return 0;

	ret = kvm_arm_smmu_array_alloc();
	if (ret)
		return ret;

	ret = platform_driver_probe(&smmuv3_nesting_driver, smmuv3_nesting_probe);
	if (ret)
		goto out_err;

	ret = kvm_iommu_register_driver(&kvm_smmu_v3_ops, nr_pages);
	if (ret)
		goto out_err;

	/*
	 * These variables are stored in the nVHE image, and won't be accessible
	 * after KVM initialization. Ownership of kvm_arm_smmu_array will be
	 * transferred to the hypervisor as well.
	 */
	kvm_hyp_arm_smmu_v3_smmus = kvm_arm_smmu_array;
	kvm_hyp_arm_smmu_v3_count = kvm_arm_smmu_count;
	return ret;

out_err:
	kvm_arm_smmu_array_free();
	return ret;
};

static int smmu_create_aux_device(struct device *dev, void *data)
{
	static int dev_id;
	struct auxiliary_device *auxdev;

	auxdev = __devm_auxiliary_device_create(dev, "protected_kvm",
						"smmu_v3_emu", NULL, dev_id++);
	if (!auxdev)
		return -ENODEV;

	auxdev->dev.parent = dev;

	return 0;
}

static int kvm_arm_smmu_v3_post_init(void)
{
	if (!is_protected_kvm_enabled() || !kvm_arm_smmu_cur)
		return 0;

	WARN_ON(driver_for_each_device(&smmuv3_nesting_driver.driver, NULL,
				       NULL, smmu_create_aux_device));

	return 0;
}

static const struct of_device_id smmuv3_nested_of_match[] = {
	{ .compatible = "arm,smmu-v3", },
	{ },
};

static struct platform_driver smmuv3_nesting_driver = {
	.driver = {
		.name = "smmuv3-nesting",
		.of_match_table = smmuv3_nested_of_match,
	},
};
subsys_initcall(kvm_arm_smmu_v3_register);

#ifndef MODULE
late_initcall(kvm_arm_smmu_v3_post_init);
#endif

MODULE_DESCRIPTION("pKVM SMMUv3 nested trap and emulated IOMMU driver.");
MODULE_LICENSE("GPL v2");
