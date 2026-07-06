// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 - Google LLC
 * Author: Quentin Perret <qperret@google.com>
 */
#ifndef __ARM64_KVM_PKVM_H__
#define __ARM64_KVM_PKVM_H__

#include <linux/arm_ffa.h>
#include <linux/memblock.h>
#include <linux/scatterlist.h>
#include <asm/kvm_host.h>
#include <asm/kvm_pgtable.h>

/* Maximum number of VMs that can co-exist under pKVM. */
#define KVM_MAX_PVMS 255

#define HYP_MEMBLOCK_REGIONS 128
#define PVMFW_INVALID_LOAD_ADDR	(-1)

int pkvm_vm_ioctl_enable_cap(struct kvm *kvm,struct kvm_enable_cap *cap);
int pkvm_init_host_vm(struct kvm *kvm, unsigned long type);
int pkvm_create_hyp_vm(struct kvm *kvm);
bool pkvm_hyp_vm_is_created(struct kvm *kvm);
void pkvm_finalize_destroy_hyp_vm(struct kvm *kvm);
int pkvm_create_hyp_vcpu(struct kvm_vcpu *vcpu);
void pkvm_host_reclaim_page(struct kvm *host_kvm, phys_addr_t ipa);
int pkvm_enable_smc_forwarding(struct file *kvm_file);

/*
 * Check whether the specific capability is allowed in pKVM.
 *
 * Certain features are allowed only for non-protected VMs in pKVM, which is why
 * this takes the VM (kvm) as a parameter.
 */
static inline bool kvm_pkvm_ext_allowed(struct kvm *kvm, long ext)
{
	switch (ext) {
	case KVM_CAP_CORE:
	case KVM_CAP_IRQCHIP:
	case KVM_CAP_ONE_REG:
	case KVM_CAP_ARM_PSCI:
	case KVM_CAP_ARM_PSCI_0_2:
	case KVM_CAP_NR_VCPUS:
	case KVM_CAP_MAX_VCPUS:
	case KVM_CAP_MAX_VCPU_ID:
	case KVM_CAP_MSI_DEVID:
	case KVM_CAP_ARM_VM_IPA_SIZE:
	case KVM_CAP_ARM_PMU_V3:
	case KVM_CAP_ARM_SVE:
	case KVM_CAP_ARM_PTRAUTH_ADDRESS:
	case KVM_CAP_ARM_PTRAUTH_GENERIC:
	case KVM_CAP_ARM_PROTECTED_VM:
		return true;
	case KVM_CAP_ARM_MTE:
		return false;
	default:
		return !kvm || !kvm_vm_is_protected(kvm);
	}
}

static inline bool __ioctl_is_smccc_filter(unsigned int ioctl, void __user *argp)
{
	struct kvm_device_attr attr;

	switch (ioctl) {
	case KVM_SET_DEVICE_ATTR:
	case KVM_GET_DEVICE_ATTR:
	case KVM_HAS_DEVICE_ATTR:
		break;
	default:
		return false;
	}

	if (copy_from_user(&attr, argp, sizeof(attr)))
		return false;

	return attr.group == KVM_ARM_VM_SMCCC_CTRL &&
	       attr.attr == KVM_ARM_VM_SMCCC_FILTER;
}

/*
 * Check whether the KVM VM IOCTL is allowed in pKVM.
 *
 * Certain features are allowed only for non-protected VMs in pKVM, which is why
 * this takes the VM (kvm) as a parameter.
 */
static inline bool kvm_pkvm_ioctl_allowed(struct kvm *kvm, unsigned int ioctl, void __user *argp)
{
	long ext;
	int r;

	r = kvm_get_cap_for_kvm_ioctl(ioctl, &ext);

	if (WARN_ON_ONCE(r < 0))
		return false;

	if (kvm_pkvm_ext_allowed(kvm, ext))
		return true;

	return __ioctl_is_smccc_filter(ioctl, argp);
}

static inline unsigned long pvm_supported_vcpu_features(struct kvm *kvm)
{
	unsigned long features = 0;

	set_bit(KVM_ARM_VCPU_POWER_OFF, &features);

	if (kvm_pkvm_ext_allowed(kvm, KVM_CAP_ARM_EL1_32BIT))
		set_bit(KVM_ARM_VCPU_EL1_32BIT, &features);

	if (kvm_pkvm_ext_allowed(kvm, KVM_CAP_ARM_PSCI_0_2))
		set_bit(KVM_ARM_VCPU_PSCI_0_2, &features);

	if (kvm_pkvm_ext_allowed(kvm, KVM_CAP_ARM_PMU_V3))
		set_bit(KVM_ARM_VCPU_PMU_V3, &features);

	if (kvm_pkvm_ext_allowed(kvm, KVM_CAP_ARM_SVE))
		set_bit(KVM_ARM_VCPU_SVE, &features);

	if (kvm_pkvm_ext_allowed(kvm, KVM_CAP_ARM_PTRAUTH_ADDRESS) &&
	    kvm_pkvm_ext_allowed(kvm, KVM_CAP_ARM_PTRAUTH_GENERIC)) {
		set_bit(KVM_ARM_VCPU_PTRAUTH_ADDRESS, &features);
		set_bit(KVM_ARM_VCPU_PTRAUTH_GENERIC, &features);
	}

	return features;
}

enum pkvm_moveable_reg_type {
	PKVM_MREG_MEMORY,
	PKVM_MREG_PROTECTED_RANGE,
	PKVM_MREG_ASSIGN_MMIO,
};

struct pkvm_moveable_reg {
	phys_addr_t start;
	u64 size;
	enum pkvm_moveable_reg_type type;
};

#define PKVM_NR_MOVEABLE_REGS 512
extern struct pkvm_moveable_reg kvm_nvhe_sym(pkvm_moveable_regs)[];
extern unsigned int kvm_nvhe_sym(pkvm_moveable_regs_nr);

extern phys_addr_t kvm_nvhe_sym(host_s2_cma_base);
extern phys_addr_t kvm_nvhe_sym(host_s2_cma_size);

extern struct memblock_region kvm_nvhe_sym(hyp_memory)[];
extern unsigned int kvm_nvhe_sym(hyp_memblock_nr);

extern phys_addr_t kvm_nvhe_sym(pvmfw_base);
extern phys_addr_t kvm_nvhe_sym(pvmfw_size);

static inline unsigned long
hyp_vmemmap_memblock_size(struct memblock_region *reg, size_t vmemmap_entry_size)
{
	unsigned long nr_pages = reg->size >> PAGE_SHIFT;
	unsigned long start, end;

	start = (reg->base >> PAGE_SHIFT) * vmemmap_entry_size;
	end = start + nr_pages * vmemmap_entry_size;
	start = ALIGN_DOWN(start, PAGE_SIZE);
	end = ALIGN(end, PAGE_SIZE);

	return end - start;
}

static inline unsigned long hyp_vmemmap_pages(size_t vmemmap_entry_size)
{
	unsigned long res = 0, i;

	for (i = 0; i < kvm_nvhe_sym(hyp_memblock_nr); i++) {
		res += hyp_vmemmap_memblock_size(&kvm_nvhe_sym(hyp_memory)[i],
						 vmemmap_entry_size);
	}

	return res >> PAGE_SHIFT;
}

static inline unsigned long hyp_vm_table_pages(void)
{
	return PAGE_ALIGN(KVM_MAX_PVMS * sizeof(void *)) >> PAGE_SHIFT;
}

static inline unsigned long __hyp_pgtable_max_pages(unsigned long nr_pages)
{
	unsigned long total = 0;
	int i;

	/* Provision the worst case scenario */
	for (i = KVM_PGTABLE_FIRST_LEVEL; i <= KVM_PGTABLE_LAST_LEVEL; i++) {
		nr_pages = DIV_ROUND_UP(nr_pages, PTRS_PER_PTE);
		total += nr_pages;
	}

	return total;
}

static inline unsigned long __hyp_pgtable_moveable_regs_pages(void)
{
	unsigned long res = 0, i;

	/* Cover all of moveable regions with page-granularity */
	for (i = 0; i < kvm_nvhe_sym(pkvm_moveable_regs_nr); i++) {
		struct pkvm_moveable_reg *reg = &kvm_nvhe_sym(pkvm_moveable_regs)[i];
		res += __hyp_pgtable_max_pages(reg->size >> PAGE_SHIFT);
	}

	return res;
}

extern u64 kvm_nvhe_sym(hyp_lm_size_mb);

static inline unsigned long hyp_s1_pgtable_pages(void)
{
	unsigned long res;

	if (!kvm_nvhe_sym(hyp_lm_size_mb))
		res = __hyp_pgtable_moveable_regs_pages();
	else
		res = __hyp_pgtable_max_pages(kvm_nvhe_sym(hyp_lm_size_mb) * SZ_1M / PAGE_SIZE);

	/* Allow 1 GiB for private mappings */
	res += __hyp_pgtable_max_pages(SZ_1G >> PAGE_SHIFT);

	return res;
}

static inline unsigned long host_s2_pgtable_pages(void)
{
	/*
	 * Include an extra 16 pages to safely upper-bound the worst case of
	 * concatenated pgds.
	 */
	return __hyp_pgtable_moveable_regs_pages() + 16;
}

static inline unsigned long host_s2_mmio_pgtable_pages(void)
{
	/* Allow 1 GiB for non-moveable regions */
	return __hyp_pgtable_max_pages(SZ_1G >> PAGE_SHIFT);
}

#ifdef CONFIG_NVHE_EL2_DEBUG
static inline unsigned long pkvm_selftest_pages(void) { return 32; }
#else
static inline unsigned long pkvm_selftest_pages(void) { return 0; }
#endif

#define KVM_FFA_MBOX_NR_PAGES	1

#define KVM_FFA_MBOX_NR_PAGES		1
#define KVM_FFA_SPM_HANDLE_NR_PAGES	2

/*
 * Maximum number of constituents allowed in a descriptor. This number is
 * arbitrary, and can be overridden via command line kvm-arm.ffa_max_nr_constituents.
 * See comment below on SG_MAX_SEGMENTS in hyp_ffa_proxy_pages().
 */
#define KVM_FFA_MAX_NR_CONSTITUENTS	4096
extern size_t kvm_nvhe_sym(ffa_max_nr_constituents);

DECLARE_STATIC_KEY_FALSE(kvm_ffa_unmap_on_lend);

static inline unsigned long hyp_ffa_proxy_pages(void)
{
	size_t desc_max;
	unsigned long num_pages;

	/*
	 * SG_MAX_SEGMENTS is supposed to bound the number of elements in an
	 * sglist, which should match the number of consituents in the
	 * corresponding FFA descriptor. As such, the EL2 buffer needs to be
	 * large enough to hold a descriptor with SG_MAX_SEGMENTS consituents
	 * at least. But the kernel's DMA code doesn't enforce the limit, and
	 * it is sometimes abused, so let's allow larger descriptors and hope
	 * for the best.
	 */
	WARN_ON(kvm_nvhe_sym(ffa_max_nr_constituents) < SG_MAX_SEGMENTS);

	/*
	 * The hypervisor FFA proxy needs enough memory to buffer a fragmented
	 * descriptor returned from EL3 in response to a RETRIEVE_REQ call.
	 */
	desc_max = sizeof(struct ffa_mem_region) +
		   sizeof(struct ffa_mem_region_attributes) +
		   sizeof(struct ffa_composite_mem_region) +
		   kvm_nvhe_sym(ffa_max_nr_constituents) * sizeof(struct ffa_mem_region_addr_range);

	/* Plus a page each for the hypervisor's RX and TX mailboxes. */
	num_pages = (2 * KVM_FFA_MBOX_NR_PAGES) + DIV_ROUND_UP(desc_max, PAGE_SIZE);

	if (static_branch_unlikely(&kvm_ffa_unmap_on_lend))
		num_pages += KVM_FFA_SPM_HANDLE_NR_PAGES;

	return num_pages;
}

static inline size_t pkvm_host_sve_state_size(void)
{
	if (!system_supports_sve())
		return 0;

	return size_add(sizeof(struct cpu_sve_state),
			SVE_SIG_REGS_SIZE(sve_vq_from_vl(kvm_host_sve_max_vl)));
}

struct pkvm_mapping {
	struct rb_node node;
	u64 gfn;
	u64 pfn;
	u64 nr_pages;
	u64 __subtree_last;	/* Internal member for interval tree */
};

int pkvm_pgtable_stage2_init(struct kvm_pgtable *pgt, struct kvm_s2_mmu *mmu,
			     struct kvm_pgtable_mm_ops *mm_ops,
			     struct kvm_pgtable_pte_ops *pte_ops);
void pkvm_pgtable_stage2_destroy(struct kvm_pgtable *pgt);
int pkvm_pgtable_stage2_map(struct kvm_pgtable *pgt, u64 addr, u64 size, u64 phys,
			    enum kvm_pgtable_prot prot, void *mc,
			    enum kvm_pgtable_walk_flags flags);
int pkvm_pgtable_stage2_unmap(struct kvm_pgtable *pgt, u64 addr, u64 size);
int pkvm_pgtable_stage2_wrprotect(struct kvm_pgtable *pgt, u64 addr, u64 size);
int pkvm_pgtable_stage2_flush(struct kvm_pgtable *pgt, u64 addr, u64 size);
bool pkvm_pgtable_stage2_test_clear_young(struct kvm_pgtable *pgt, u64 addr, u64 size, bool mkold);
int pkvm_pgtable_stage2_relax_perms(struct kvm_pgtable *pgt, u64 addr, enum kvm_pgtable_prot prot,
				    enum kvm_pgtable_walk_flags flags);
void pkvm_pgtable_stage2_mkyoung(struct kvm_pgtable *pgt, u64 addr,
				 enum kvm_pgtable_walk_flags flags);
int pkvm_pgtable_stage2_split(struct kvm_pgtable *pgt, u64 addr, u64 size, void *mc);
void pkvm_pgtable_stage2_free_unlinked(struct kvm_pgtable_mm_ops *mm_ops,
				       struct kvm_pgtable_pte_ops *pte_ops, void *pgtable,
				       s8 level);
kvm_pte_t *pkvm_pgtable_stage2_create_unlinked(struct kvm_pgtable *pgt, u64 phys, s8 level,
					       enum kvm_pgtable_prot prot, void *mc,
					       bool force_pte);

int __pkvm_topup_hyp_alloc_mgt_mc(enum hyp_alloc_mgt_id id, struct kvm_hyp_memcache *mc);
int __pkvm_topup_hyp_alloc(unsigned long nr_pages);

int __pkvm_handle_smccc_req(struct arm_smccc_res *res, void *arg);

#define kvm_call_refill_hyp_nvhe(f, ...)				\
({									\
	struct arm_smccc_res res;					\
	int __ret;							\
	do {								\
		__ret = -1;						\
		arm_smccc_1_1_hvc(KVM_HOST_SMCCC_FUNC(f),		\
				  ##__VA_ARGS__, &res);			\
		if (WARN_ON(res.a0 != SMCCC_RET_SUCCESS))		\
			break;						\
		__ret = res.a1;						\
		if (!__ret)						\
			break;						\
		__ret = __pkvm_handle_smccc_req(&res, NULL);		\
	} while (!__ret);						\
	__ret;								\
})

#ifdef CONFIG_CMA
int pkvm_host_stage2_topup(gfp_t gfp);
#else
static inline int pkvm_host_stage2_topup(gfp_t gfp) { return -EINVAL; }
#endif

enum pkvm_ptdump_ops {
	PKVM_PTDUMP_GET_LEVEL,
	PKVM_PTDUMP_GET_RANGE,
	PKVM_PTDUMP_WALK_RANGE,
};

struct pkvm_ptdump_log {
	/* VA_BIT - PAGE_SHIFT + 1 (INVALID_PTDUMP_PFN) */
	u64	pfn: 41;
	bool	valid: 1;
	bool	r: 1;
	bool	w: 1;
	char	xn: 2;
	bool	table: 1;
	u16	page_state: 2;
	u16	level: 8;
	u8	mmio_guard : 1;
} __packed;

#define INVALID_PTDUMP_PFN	(BIT(41) - 1)

struct pkvm_ptdump_log_hdr {
	/* The next page */
	u64	pfn_next: 48;
	/* The write index in the log page */
	u64	w_index: 16;
};

#endif	/* __ARM64_KVM_PKVM_H__ */
