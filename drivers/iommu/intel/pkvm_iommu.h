/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright © 2026 Google
 */

#ifndef _PKVM_INTEL_IOMMU_H_
#define _PKVM_INTEL_IOMMU_H_

/* expose pkvm_enabled() when !CONFIG_PKVM_X86 */
#include <asm/kvm_host.h>
#include <asm/kvm_pkvm.h>

#define PKVM_MAX_SATC_DEVS	16

/* Page table level represented by IOMMU cap SAGAW bits */
#define IOMMU_PGT_4LEVEL	BIT(2)
#define IOMMU_PGT_5LEVEL	BIT(3)

struct intel_iommu_info {
	u64 reg_phys;
	u64 reg_size;
	u64 cap;
	u64 ecap;
	int seq_id;
	int agaw;
	int msagaw;
};

struct qi_desc;
struct intel_iommu;
struct dmar_domain;
struct device_domain_info;

#ifdef CONFIG_PKVM_INTEL
extern u16 pkvm_sym(satc_devs)[];
extern int pkvm_sym(nr_satc_devs);

extern unsigned int pkvm_sym(iommu_pglvl_mask);
extern unsigned int pkvm_sym(iommu_pgsz_mask);

extern int pkvm_sym(intel_iommu_sm);
extern int pkvm_sym(intel_iommu_superpage);

PKVM_DECLARE(int, prepare_iommu, (struct intel_iommu_info *info));

#ifndef __PKVM_HYP__
static inline u64 pkvm_readq(void __iomem *reg, unsigned long reg_phys, unsigned long offset)
{
	union pkvm_hc_data d;
	int ret;

	if (!pkvm_enabled())
		return readq(reg + offset);

	ret = pkvm_hypercall_out(iommu_mmio_read, &d, reg_phys + offset, sizeof(u64));
	if (ret)
		pr_err("%s: iommu_mmio_read(reg_phys: %lx, off: %lx) failed(err=%d)\n",
		       __func__, reg_phys, offset, ret);

	return d.iommu_mmio_read.val;
}

static inline u32 pkvm_readl(void __iomem *reg, unsigned long reg_phys, unsigned long offset)
{
	union pkvm_hc_data d;
	int ret;

	if (!pkvm_enabled())
		return readl(reg + offset);

	ret = pkvm_hypercall_out(iommu_mmio_read, &d, reg_phys + offset, sizeof(u32));
	if (ret)
		pr_err("%s: iommu_mmio_read(reg_phys: %lx, off: %lx) failed(err=%d)\n",
		       __func__, reg_phys, offset, ret);

	return (u32)d.iommu_mmio_read.val;
}

static inline void pkvm_writeq(void __iomem *reg, unsigned long reg_phys,
			       unsigned long offset, u64 val)
{
	int ret;

	if (!pkvm_enabled()) {
		writeq(val, reg + offset);
		return;
	}

	ret = pkvm_hypercall(iommu_mmio_write, reg_phys + offset, sizeof(u64), val);
	if (ret)
		pr_err("%s: iommu_mmio_write(phys: %lx, off: %lx, val: %llx) failed(err=%d)\n",
		       __func__, reg_phys, offset, val, ret);
}

static inline void pkvm_writel(void __iomem *reg, unsigned long reg_phys,
			       unsigned long offset, u32 val)
{
	int ret;

	if (!pkvm_enabled()) {
		writel(val, reg + offset);
		return;
	}

	ret = pkvm_hypercall(iommu_mmio_write, reg_phys + offset, sizeof(u32), val);
	if (ret)
		pr_err("%s: iommu_mmio_write(phys: %lx, off: %lx, val: %x) failed(err=%d)\n",
		       __func__, reg_phys, offset, val, ret);
}

int __init pkvm_scan_satc_devs(u16 satc_devs[], int *nr_satc_devs, int max_satc_devs);

int __init pkvm_host_prepare_iommu(void);
int __init pkvm_host_init_iommu(void);

int pkvm_iec_flush(struct intel_iommu *iommu, bool global, int index, int mask);
int pkvm_context_clear(u64 phys, u8 bus, u8 devfn, struct device_domain_info *info);
int pkvm_context_mapping(struct intel_iommu *iommu, struct device_domain_info *info,
			 u8 bus, u8 devfn, u64 pgd_gpa, u16 did);
int pkvm_pasid_table_setup(struct intel_iommu *iommu, struct device_domain_info *info,
			   u8 bus, u8 devfn);
int pkvm_pasid_setup_fl(struct device_domain_info *info, phys_addr_t fsptptr,
			u32 pasid, u16 did, u16 old_did, int flags);
int pkvm_pasid_setup_sl(struct device_domain_info *info, phys_addr_t ssptptr,
			u32 pasid, u16 did, u16 old_did);
int pkvm_pasid_teardown(struct device_domain_info *info, u32 pasid);
int pkvm_alloc_domain(struct device_domain_info *info, struct dmar_domain *domain);
int pkvm_free_domain(struct dmar_domain *domain);
int pkvm_domain_map(struct dmar_domain *domain, unsigned long iov_pfn,
		    unsigned long phys_pfn, unsigned long nr_pages,
		    int prot, int gfp);
int pkvm_domain_unmap(struct dmar_domain *domain, unsigned long start_pfn,
		      unsigned long last_pfn);
#else /* __PKVM_HYP__ */
/*
 * dev_iommu_priv_get is called from quite a few places in code re-used by
 * hypervisor and is defined in include/linux/iommu.h. It takes struct device
 * as an argument. Having struct device as a stack variable to pass to
 * dev_iommu_priv_get causes the stack to grow beyond safe limit. So we define
 * a wrapper struct and a wrapper function to avoid ifdef-ing all locations
 * where dev_iommu_priv_get is called. The C file(pasid.c) will redefine
 * dev_iommu_priv_get to pkvm_dev_iommu_priv_get to enable this re-use.
 */
struct pkvm_device {
	struct device_domain_info *info;
};

static inline struct device_domain_info *pkvm_dev_iommu_priv_get(struct pkvm_device *dev)
{
	return dev->info;
}

static inline bool iommu_supports_2m_page(void)
{
	return iommu_pgsz_mask & (1 << PG_LEVEL_2M);
}

static inline bool iommu_supports_1g_page(void)
{
	return iommu_pgsz_mask & (1 << PG_LEVEL_1G);
}

static inline bool iommu_supports_5levels(void)
{
	return iommu_pglvl_mask & IOMMU_PGT_5LEVEL;
}

struct intel_iommu *iommu_from_phys(unsigned long phys);
static inline bool is_iommu_mmio(unsigned long phys)
{
	return !!iommu_from_phys(phys);
}

extern struct dmar_domain pt_domain;
void init_pt_domain(void);

bool overlaps_iommu_mmio(unsigned long phys, unsigned long size);
bool is_dev_in_satc(u16 bdf);
bool pkvm_iommu_paging_structure_coherency(void);

struct dmar_domain *pkvm_alloc_iommu_domain(struct alloc_domain_data *data,
					    bool need_iotlb_sync_map);
struct dmar_domain *pkvm_get_iommu_domain(void *pgd);
struct dmar_domain *pkvm_get_iommu_domain_noref(void *pgd);
void pkvm_put_iommu_domain(struct dmar_domain *domain);
int pkvm_free_iommu_domain(struct dmar_domain *domain, struct pkvm_memcache *teardown_mc);

struct cache_tag *pkvm_alloc_cache_tag(void);
void pkvm_free_cache_tag(struct cache_tag *cache_tag);
void pkvm_iommu_pt_flush(unsigned long paddr, unsigned long size);

int pkvm_get_domain_cache_tag_assign(void *pgd, int did, u32 pasid,
				     struct device_domain_info *info);
void pkvm_put_domain_cache_tag_unassign(void *pgd, int did, u32 pasid,
					struct device_domain_info *info);

int pkvm_intel_iommu_init(void);

int pkvm_iommu_mmio_read(u64 phys, int len, u64 *val);
int pkvm_iommu_mmio_write(u64 phys, int len, u64 val);
int pkvm_iommu_iec_flush(u64 phys, int index, int mask, bool global);
int pkvm_iommu_clear_ce(struct clear_ce_data *data);
int pkvm_iommu_set_lm_ce(struct set_lm_ce_data *in, struct set_lm_ce_data *out);
int pkvm_iommu_set_sm_ce(struct set_sm_ce_data *in, struct set_sm_ce_data *out);
int pkvm_iommu_pasid_setup_fl(struct pasid_setup_fl_data *in, struct pasid_setup_fl_data *out);
int pkvm_iommu_pasid_setup_sl(struct pasid_setup_sl_data *in, struct pasid_setup_sl_data *out);
int pkvm_iommu_pasid_teardown(struct pasid_teardown_data *data);
int pkvm_iommu_alloc_domain(struct alloc_domain_data *data);
int pkvm_iommu_free_domain(u64 pgd_gpa, struct pkvm_memcache *mc);
int pkvm_iommu_domain_map(struct domain_map_data *in, struct domain_map_data *out);
int pkvm_iommu_domain_unmap(u64 pgd_gpa, u64 start_pfn, u64 last_pfn);
#endif /* !__PKVM_HYP__ */
#else /* !CONFIG_PKVM_INTEL */
static inline int pkvm_iec_flush(struct intel_iommu *iommu, bool global,
				 int index, int mask)
{
	return -EOPNOTSUPP;
}
static inline int pkvm_context_clear(u64 phys, u8 bus, u8 devfn,
				     struct device_domain_info *info)
{
	return -EOPNOTSUPP;
}
static inline int pkvm_context_mapping(struct intel_iommu *iommu,
				       struct device_domain_info *info,
				       u8 bus, u8 devfn, u64 pgd_gpa, u16 did)
{
	return -EOPNOTSUPP;
}
static inline int pkvm_pasid_table_setup(struct intel_iommu *iommu,
					 struct device_domain_info *info,
					 u8 bus, u8 devfn)
{
	return -EOPNOTSUPP;
}
static inline int pkvm_pasid_setup_fl(struct device_domain_info *info,
				      phys_addr_t fsptptr, u32 pasid,
				      u16 did, u16 old_did, int flags)
{
	return -EOPNOTSUPP;
}
static inline int pkvm_pasid_setup_sl(struct device_domain_info *info,
				      phys_addr_t ssptptr, u32 pasid,
				      u16 did, u16 old_did)
{
	return -EOPNOTSUPP;
}
static inline int pkvm_pasid_teardown(struct device_domain_info *info, u32 pasid)
{
	return -EOPNOTSUPP;
}
static inline int pkvm_alloc_domain(struct device_domain_info *info,
				    struct dmar_domain *domain)
{
	return -EOPNOTSUPP;
}
static inline int pkvm_free_domain(struct dmar_domain *domain)
{
	return -EOPNOTSUPP;
}
static inline int pkvm_domain_map(struct dmar_domain *domain, unsigned long iov_pfn,
				  unsigned long phys_pfn, unsigned long nr_pages,
				  int prot, int gfp)
{
	return -EOPNOTSUPP;
}
static inline int pkvm_domain_unmap(struct dmar_domain *domain, unsigned long start_pfn,
				    unsigned long last_pfn)
{
	return -EOPNOTSUPP;
}
#endif /* CONFIG_PKVM_INTEL */
#endif /* _PKVM_INTEL_IOMMU_H_ */
