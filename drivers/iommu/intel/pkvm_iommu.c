// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright © 2026 Google.
 */

#define pr_fmt(fmt)     "DMAR: pkvm: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include "iommu.h"
#include "pasid.h"
#include "../iommu-pages.h"

int __init pkvm_host_prepare_iommu(void)
{
	struct dmar_drhd_unit *drhd;
	struct intel_iommu *iommu;
	int ret;

	down_write(&dmar_global_lock);
	ret = dmar_table_init();
	if (ret) {
		pr_err("Failed to initialize DMAR table!\n");
		goto out;
	}

	ret = -ENODEV;
	for_each_iommu(iommu, drhd) {
		unsigned int pgsz_mask = 1 << PG_LEVEL_4K;
		unsigned int pglvl_mask = 0;

		if (drhd->ignored) {
			pr_warn("iommu%d ignored, but pKVM needs iommu to be enabled!\n",
				iommu->seq_id);
			goto out;
		}

		if (readl(iommu->reg + DMAR_GSTS_REG) & DMA_GSTS_TES) {
			pr_warn("iommu%d: Translation enabled before initialization!\n",
					iommu->seq_id);
			goto out;
		}

		/*
		 * Since pKVM is not expected to be supported on ancient hardware which
		 * requires write buffer flushing, require cap_rwbf=0 for simplicity.
		 */
		if (cap_rwbf(iommu->cap)) {
			pr_warn("iommu%d: CAP.RWBF=1 is not supported!\n", iommu->seq_id);
			goto out;
		}

		/* pKVM expects Queued Invalidation support for simplicity and efficiency */
		if (!ecap_qis(iommu->ecap)) {
			pr_warn("iommu%d: queued Invalidation not supported!\n", iommu->seq_id);
			goto out;
		}

		if (cap_sagaw(iommu->cap) & IOMMU_PGT_4LEVEL)
			pglvl_mask |= IOMMU_PGT_4LEVEL;
		if (cap_sagaw(iommu->cap) & IOMMU_PGT_5LEVEL)
			pglvl_mask |= IOMMU_PGT_5LEVEL;

		if (cap_super_page_val(iommu->cap) & BIT(0))
			pgsz_mask |= 1 << PG_LEVEL_2M;
		if (cap_super_page_val(iommu->cap) & BIT(1))
			pgsz_mask |= 1 << PG_LEVEL_1G;

		if (!pglvl_mask) {
			pr_warn("iommu%d: No supported page levels\n", iommu->seq_id);
			goto out;
		}

		pkvm_sym(iommu_pglvl_mask) &= pglvl_mask;
		pkvm_sym(iommu_pgsz_mask) &= pgsz_mask;
	}

	ret = pkvm_scan_satc_devs(pkvm_sym(satc_devs), &pkvm_sym(nr_satc_devs),
				  PKVM_MAX_SATC_DEVS);
	if (ret)
		goto out;

	pkvm_sym(intel_iommu_sm) = intel_iommu_sm;
	pkvm_sym(intel_iommu_superpage) = intel_iommu_superpage;

	for_each_iommu(iommu, drhd) {
		struct intel_iommu_info info = {
			.reg_phys = iommu->reg_phys,
			.reg_size = iommu->reg_size,
			.cap = iommu->cap,
			.ecap = iommu->ecap,
			.seq_id = iommu->seq_id,
			.agaw = iommu->agaw,
			.msagaw = iommu->msagaw,
		};

		ret = pkvm_sym(prepare_iommu)(&info);
		if (ret)
			goto out;
	}
out:
	up_write(&dmar_global_lock);
	return ret;
}

int __init pkvm_host_init_iommu(void)
{
	int ret = intel_iommu_init();

	if (!ret)
		pr_info("IOMMU initialized!\n");
	else
		pr_err("IOMMU initialize failed(err=%d)\n", ret);

	return ret;
}

int pkvm_iec_flush(struct intel_iommu *iommu, bool global, int index, int mask)
{
	return pkvm_hypercall(iommu_iec_flush, iommu->reg_phys, index, mask, global);
}

int pkvm_context_clear(u64 phys, u8 bus, u8 devfn, struct device_domain_info *info)
{
	union pkvm_hc_data d = { 0 };
	struct clear_ce_data *data = &d.iommu_clear_ce.data;

	data->phys = phys;
	data->bus = bus;
	data->devfn = devfn;
	data->ats_qdep = info->ats_qdep;
	data->ats_supported = info->ats_supported;

	return pkvm_hypercall_in(iommu_clear_ce, &d);
}

int pkvm_context_mapping(struct intel_iommu *iommu, struct device_domain_info *info,
			 u8 bus, u8 devfn, u64 pgd_gpa, u16 did)
{
	union pkvm_hc_data d = { 0 };
	struct set_lm_ce_data *data = &d.iommu_set_lm_ce.in;
	int ret;

	data->phys = iommu->reg_phys;
	data->pgd_gpa = pgd_gpa;
	data->did = did;
	data->bus = bus;
	data->devfn = devfn;
	data->ats_qdep = info->ats_qdep;
	data->ats_supported = info->ats_supported;

	spin_lock(&iommu->lock);
	ret = pkvm_hypercall_inout(iommu_set_lm_ce, &d, &d);
	if (ret == -ENOMEM) {
		void *donation_page = iommu_alloc_pages_node_sz(iommu->node, GFP_ATOMIC, SZ_4K);

		if (!donation_page) {
			pr_err("iommu%d: failed to allocate context page\n", iommu->seq_id);
			spin_unlock(&iommu->lock);
			return -ENOMEM;
		}
		data->donation_page_gpa = virt_to_phys(donation_page);
		ret = pkvm_hypercall_inout(iommu_set_lm_ce, &d, &d);

		/*
		 * If the hypervisor used donation_gpa, it will be set to 0.
		 * Free the page if hypervisor didn't use the page.
		 */
		if (data->donation_page_gpa)
			iommu_free_pages(phys_to_virt(data->donation_page_gpa));
	}
	spin_unlock(&iommu->lock);

	return ret;
}

int pkvm_pasid_table_setup(struct intel_iommu *iommu, struct device_domain_info *info,
			   u8 bus, u8 devfn)
{
	union pkvm_hc_data d = { 0 };
	struct set_sm_ce_data *data = &d.iommu_set_sm_ce.in;
	int ret;

	data->phys = iommu->reg_phys;
	data->pasid_table_gpa = virt_to_phys(info->pasid_table->table);
	data->max_pasid = info->pasid_table->max_pasid;
	data->bus = bus;
	data->devfn = devfn;
	data->pasid_supported = info->pasid_supported;
	data->pasid_enabled = info->pasid_enabled;
	data->ats_supported = info->ats_supported;
	data->ats_qdep = info->ats_qdep;

	spin_lock(&iommu->lock);
	ret = pkvm_hypercall_inout(iommu_set_sm_ce, &d, &d);
	if (ret == -ENOMEM) {
		void *donation_page = iommu_alloc_pages_node_sz(iommu->node,
								GFP_ATOMIC, SZ_4K);

		if (!donation_page) {
			pr_err("iommu%d: failed to allocate context page\n", iommu->seq_id);
			spin_unlock(&iommu->lock);
			return -ENOMEM;
		}
		data->donation_page_gpa = virt_to_phys(donation_page);
		ret = pkvm_hypercall_inout(iommu_set_sm_ce, &d, &d);

		if (data->donation_page_gpa)
			iommu_free_pages(phys_to_virt(data->donation_page_gpa));
	}
	spin_unlock(&iommu->lock);

	return ret;
}

int pkvm_pasid_setup_fl(struct device_domain_info *info, phys_addr_t fsptptr,
		      u32 pasid, u16 did, u16 old_did, int flags)
{
	union pkvm_hc_data d = { 0 };
	struct pasid_setup_fl_data *data = &d.iommu_pasid_setup_fl.in;
	struct intel_iommu *iommu = info->iommu;
	int ret;

	data->phys = iommu->reg_phys;
	data->fsptptr_gpa = fsptptr;
	data->pasid = pasid;
	data->flags = flags;
	data->did = did;
	data->old_did = old_did;
	data->bus = info->bus;
	data->devfn = info->devfn;
	data->ats_qdep = info->ats_qdep;
	data->ats_supported = info->ats_supported;

	spin_lock(&iommu->lock);
	ret = pkvm_hypercall_inout(iommu_pasid_setup_fl, &d, &d);
	if (ret == -ENOMEM) {
		void *donation_page = iommu_alloc_pages_node_sz(iommu->node, GFP_ATOMIC, SZ_4K);

		if (!donation_page) {
			pr_err("iommu%d: failed to allocate pasid table page\n", iommu->seq_id);
			spin_unlock(&iommu->lock);
			return -ENOMEM;
		}
		data->donation_page_gpa = virt_to_phys(donation_page);
		ret = pkvm_hypercall_inout(iommu_pasid_setup_fl, &d, &d);

		if (data->donation_page_gpa)
			iommu_free_pages(phys_to_virt(data->donation_page_gpa));
	}
	spin_unlock(&iommu->lock);

	return ret;
}

int pkvm_pasid_setup_sl(struct device_domain_info *info, phys_addr_t ssptptr,
			u32 pasid, u16 did, u16 old_did)
{
	union pkvm_hc_data d = { 0 };
	struct pasid_setup_sl_data *data = &d.iommu_pasid_setup_sl.in;
	struct intel_iommu *iommu = info->iommu;
	int ret;

	data->phys = iommu->reg_phys;
	data->ssptptr_gpa = ssptptr;
	data->pasid = pasid;
	data->did = did;
	data->old_did = old_did;
	data->bus = info->bus;
	data->devfn = info->devfn;
	data->ats_qdep = info->ats_qdep;
	data->ats_supported = info->ats_supported;

	spin_lock(&iommu->lock);
	ret = pkvm_hypercall_inout(iommu_pasid_setup_sl, &d, &d);
	if (ret == -ENOMEM) {
		void *donation_page = iommu_alloc_pages_node_sz(iommu->node, GFP_ATOMIC, SZ_4K);

		if (!donation_page) {
			pr_err("iommu%d: failed to allocate pasid table page\n", iommu->seq_id);
			spin_unlock(&iommu->lock);
			return -ENOMEM;
		}
		data->donation_page_gpa = virt_to_phys(donation_page);
		ret = pkvm_hypercall_inout(iommu_pasid_setup_sl, &d, &d);

		if (data->donation_page_gpa)
			iommu_free_pages(phys_to_virt(data->donation_page_gpa));
	}
	spin_unlock(&iommu->lock);

	return ret;
}

int pkvm_pasid_teardown(struct device_domain_info *info, u32 pasid)
{
	union pkvm_hc_data d = { 0 };
	struct pasid_teardown_data *data = &d.iommu_pasid_teardown.data;
	struct intel_iommu *iommu = info->iommu;

	data->phys = iommu->reg_phys;
	data->pasid = pasid;
	data->bus = info->bus;
	data->devfn = info->devfn;
	data->ats_qdep = info->ats_qdep;
	data->ats_supported = info->ats_supported;

	return pkvm_hypercall_in(iommu_pasid_teardown, &d);
}

int pkvm_alloc_domain(struct device_domain_info *info, struct dmar_domain *domain)
{
	union pkvm_hc_data d = { 0 };
	struct alloc_domain_data *data = &d.iommu_alloc_domain.data;
	int ret;

	data->phys = info->iommu->reg_phys;
	data->bdf = PCI_DEVID(info->bus, info->devfn);
	data->use_first_level = domain->use_first_level;
	data->pgd_gpa = virt_to_phys(domain->pgd);
	data->gaw = domain->gaw;
	data->agaw = domain->agaw;
	data->max_addr = domain->max_addr;
	data->iommu_coherency = domain->iommu_coherency;
	data->iommu_superpage = domain->iommu_superpage;

	ret = pkvm_hypercall_in(iommu_alloc_domain, &d);
	if (ret)
		pr_err("%s: pkvm failed to alloc domain for device[%x:%x.%x] (err=%d)\n", __func__,
		       info->bus, PCI_SLOT(info->devfn), PCI_FUNC(info->devfn), ret);

	return ret;
}

static phys_addr_t host_pa(void *addr)
{
	return __pa(addr);
}

static void *host_va(phys_addr_t phys)
{
	return __va(phys);
}

struct mc_alloc_arg {
	int nid;
	gfp_t gfp;
};

static void *host_mc_alloc_page(void *alloc_arg)
{
	struct mc_alloc_arg *arg = (struct mc_alloc_arg *)alloc_arg;

	return iommu_alloc_pages_node_sz(arg->nid, arg->gfp, SZ_4K);
}

static void free_domain_memcache(struct pkvm_memcache *mc)
{
	while (mc->count)
		iommu_free_pages(pop_pkvm_memcache_page(mc, host_va));
}

int pkvm_free_domain(struct dmar_domain *domain)
{
	union pkvm_hc_data out;
	int ret;

	ret = pkvm_hypercall_out(iommu_free_domain, &out, virt_to_phys(domain->pgd));
	free_domain_memcache(&out.iommu_free_domain.memcache);
	return ret;
}

int pkvm_domain_map(struct dmar_domain *domain, unsigned long iov_pfn,
		    unsigned long phys_pfn, unsigned long nr_pages,
		    int prot, int gfp)
{
	union pkvm_hc_data d = { 0 };
	struct domain_map_data *data = &d.iommu_domain_map.in;
	int ret;

	data->pgd_gpa = virt_to_phys(domain->pgd),
	data->iov_pfn = iov_pfn,
	data->phys_pfn = phys_pfn,
	data->nr_pages = nr_pages,
	data->prot = prot,

	ret = pkvm_hypercall_inout(iommu_domain_map, &d, &d);
	if (ret == -ENOMEM) {
		struct mc_alloc_arg arg = {
			.nid = domain->nid,
			.gfp = gfp,
		};

		ret = topup_pkvm_memcache(&data->mc, __pkvm_pgtable_max_pages(nr_pages),
					  host_mc_alloc_page, host_pa, &arg);
		if (!ret)
			ret = pkvm_hypercall_inout(iommu_domain_map, &d, &d);
		else
			pr_err("%s: memcache topup failed(err=%d)\n", __func__, ret);
	}

	if (ret) {
		pr_err("%s: domain map[iov_pfn: %lx, pfn: %lx, nr_pages: %lu] failed (err=%d)\n",
		       __func__, iov_pfn, phys_pfn, nr_pages, ret);

		/*
		 * pKVM would not have drained the memcache on
		 * hypercall failure. Free it if not empty.
		 */
		free_domain_memcache(&data->mc);
	}

	domain->has_mappings = true;
	return ret;
}

int pkvm_domain_unmap(struct dmar_domain *domain, unsigned long start_pfn, unsigned long last_pfn)
{
	int ret = pkvm_hypercall(iommu_domain_unmap, virt_to_phys(domain->pgd),
				 start_pfn, last_pfn);

	if (ret)
		pr_err("%s: domain unmap[start_pfn: %lx, last_pfn: %lx] failed (err=%d)\n",
		       __func__, start_pfn, last_pfn, ret);
	return ret;
}
