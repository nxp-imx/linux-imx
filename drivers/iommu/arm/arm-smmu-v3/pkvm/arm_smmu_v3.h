/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __KVM_ARM_SMMU_V3_H
#define __KVM_ARM_SMMU_V3_H

#include <asm/kvm_asm.h>

#ifdef __KVM_NVHE_HYPERVISOR__
#include <nvhe/spinlock.h>
#endif

#include <kvm/power_domain.h>

#include "../arm-smmu-v3.h"

/*
 * Parameters from the trusted host:
 * @mmio_addr		base address of the SMMU registers
 * @mmio_size		size of the registers resource
 * @features		Features of SMMUv3, subset of the main driver
 *
 * Other members are filled and used at runtime by the SMMU driver.
 * @base		Virtual address of SMMU registers
 * @ias			IPA size
 * @oas			PA size
 * @pgsize_bitmap	Supported page sizes
 * @lock		Lock to protect SMMU
 * @cmdq		CMDQ as observed by HW
 * @strtab_cfg		Stream table as seen by HW
 */
struct hyp_arm_smmu_v3_device {
	phys_addr_t		mmio_addr;
	size_t			mmio_size;
	void __iomem		*base;
	u32			features;
	unsigned long		ias;
	unsigned long		oas;
	unsigned long		pgsize_bitmap;
	u32			lock;
	struct arm_smmu_queue	cmdq;
	struct arm_smmu_strtab_cfg strtab_cfg;
};

#if IS_ENABLED(CONFIG_ARM_SMMU_V3_PKVM)
/*
 * Parameters from the trusted host:
 * @strtab_dma		Phys address of stream table
 * @strtab_size		Stream table size
 *
 * Other members are filled and used at runtime by the SMMU driver.
 * @sid_bits		Max number of SID bits supported
 * @cmdq_host		Host view of the command queue
 * @cr0			Last value of CR0
 * @host_ste_cfg	Host stream table config
 * @host_ste_base	Host stream table base
 * @gbpa		Last value of GBPA from the host
 * @evtq_base		Host evtq base reg
 * @priq_base		Host priq base reg
 */
struct hyp_arm_smmu_v3_nested_device {
	struct hyp_arm_smmu_v3_device common;
	unsigned int		sid_bits;
	struct arm_smmu_queue	cmdq_host;
	u32			cr0;
	dma_addr_t		strtab_dma;
	size_t			strtab_size;
	u64			host_ste_cfg;
	u64			host_ste_base;
	u32			gbpa;
	unsigned long		evtq_base;
	unsigned long		priq_base;
};

extern size_t kvm_nvhe_sym(kvm_hyp_arm_smmu_v3_count);
#define kvm_hyp_arm_smmu_v3_count kvm_nvhe_sym(kvm_hyp_arm_smmu_v3_count)

extern struct hyp_arm_smmu_v3_nested_device *kvm_nvhe_sym(kvm_hyp_arm_smmu_v3_smmus);
#define kvm_hyp_arm_smmu_v3_smmus kvm_nvhe_sym(kvm_hyp_arm_smmu_v3_smmus)
#endif

#if IS_ENABLED(CONFIG_ARM_SMMU_V3_PKVM_PV)
struct hyp_arm_smmu_v3_device_pv {
	struct hyp_arm_smmu_v3_device common;
	struct arm_smmu_queue  		evtq;
	u32                   		ssid_bits;
	bool                  		power_is_off;
	struct kvm_power_domain		power_domain;
	unsigned long			idmap_ref;
};

extern size_t kvm_nvhe_sym(kvm_hyp_arm_smmu_v3_pv_count);
#define kvm_hyp_arm_smmu_v3_pv_count kvm_nvhe_sym(kvm_hyp_arm_smmu_v3_pv_count)

extern struct hyp_arm_smmu_v3_device_pv *kvm_nvhe_sym(kvm_hyp_arm_smmu_v3_pv_smmus);
#define kvm_hyp_arm_smmu_v3_pv_smmus kvm_nvhe_sym(kvm_hyp_arm_smmu_v3_pv_smmus)
#endif

#ifdef MODULE
int smmu_init_hyp_module(const struct pkvm_module_ops *ops);
#endif

#ifdef __KVM_NVHE_HYPERVISOR__
static inline hyp_spinlock_t *kvm_smmu_get_lock(struct hyp_arm_smmu_v3_device *smmu)
{
	/* See struct kvm_hyp_iommu */
	BUILD_BUG_ON(sizeof(smmu->lock) != sizeof(hyp_spinlock_t));
	return (hyp_spinlock_t *)(&smmu->lock);
}

static inline void kvm_smmu_lock_init(struct hyp_arm_smmu_v3_device *smmu)
{
	hyp_spin_lock_init(kvm_smmu_get_lock(smmu));
}

static inline void kvm_smmu_lock(struct hyp_arm_smmu_v3_device *smmu)
{
	hyp_spin_lock(kvm_smmu_get_lock(smmu));
}

static inline void kvm_smmu_unlock(struct hyp_arm_smmu_v3_device *smmu)
{
	hyp_spin_unlock(kvm_smmu_get_lock(smmu));
}
#endif

enum kvm_arm_smmu_domain_type {
	KVM_ARM_SMMU_DOMAIN_S1 = KVM_IOMMU_DOMAIN_ANY_TYPE,
	KVM_ARM_SMMU_DOMAIN_S2,
	KVM_ARM_SMMU_DOMAIN_MAX,
};

#endif /* __KVM_ARM_SMMU_V3_H */
