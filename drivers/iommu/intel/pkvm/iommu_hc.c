// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright © 2026 Google.
 *
 */
#include <asm/kvm_pkvm.h>
#include <linux/pci.h>
#include "pkvm/mmu.h"
#include "pkvm/vmx/ept.h"
#include "pkvm/memory.h"
#include "pkvm/pkvm.h"
#include "pkvm/debug.h"
#include "../iommu.h"
#include "../pasid.h"

int pkvm_iommu_iec_flush(u64 phys, int index, int mask, bool global)
{
	struct intel_iommu *iommu = iommu_from_phys(phys);

	if (!iommu)
		return -EINVAL;

	BUG_ON(!iommu->qi);

	if (global) {
		qi_global_iec(iommu);
		return 0;
	}

	return qi_flush_iec(iommu, index, mask);
}

int pkvm_iommu_clear_ce(struct clear_ce_data *data)
{
	struct intel_iommu *iommu = iommu_from_phys(data->phys);
	u16 bdf = PCI_DEVID(data->bus, data->devfn);
	struct device_domain_info info = { 0 };

	if (!iommu)
		return -EINVAL;

	if (data->ats_qdep > PCI_ATS_MAX_QDEP)
		return -EINVAL;

	if (is_dev_in_satc(bdf)) {
		/*
		 * Device is in SATC and optimistically assuming that a well crafted SATC
		 * would contain only physical functions, its safe to set pfsid = bdf.
		 * TODO: We should probably be verifying SATC for existence of only
		 * physical functions during pkvm initialization.
		 */
		if (ecap_dit(iommu->ecap))
			info.pfsid = bdf;
	} else if (data->ats_enabled || data->ats_supported) {
		return -EPERM;
	}

	info.iommu = iommu;
	info.bus = data->bus;
	info.devfn = data->devfn;
	info.ats_qdep = data->ats_qdep;
	info.ats_supported = data->ats_supported;
	info.ats_enabled = data->ats_enabled;

	pkvm_dbg("%s: dev[%x:%x], ats_qdep: %d\n",
		 __func__, data->bus, data->devfn, data->ats_qdep);
	domain_context_clear_one(&info, data->bus, data->devfn);

	return 0;
}

static int accept_page_donation(struct intel_iommu *iommu, u64 *donation_page_gpa)
{
	pkvm_spin_lock(&iommu->lock);
	if (*donation_page_gpa && !iommu->donation_page) {
		u64 donation_page_pa = pkvm_host_gpa_to_phys(*donation_page_gpa);

		int ret = pkvm_host_donate_hyp_share_ro(donation_page_pa,
							VTD_PAGE_SIZE, true);

		if (ret) {
			pkvm_spin_unlock(&iommu->lock);
			pkvm_err("iommu%d: failed to write protect donated page(err=%d)!\n",
				 iommu->seq_id, ret);
			return ret;
		}
		iommu->donation_page = __pkvm_va(donation_page_pa);
		*donation_page_gpa = 0ULL;
	}
	pkvm_spin_unlock(&iommu->lock);

	return 0;
}

static int iommu_set_lm_ce(struct set_lm_ce_data *data)
{
	struct intel_iommu *iommu = iommu_from_phys(data->phys);
	u16 bdf = PCI_DEVID(data->bus, data->devfn);
	struct device_domain_info info = { 0 };
	struct dmar_domain domain = { 0 };
	int ret;

	if (!iommu)
		return -EINVAL;

	if (data->ats_qdep > PCI_ATS_MAX_QDEP)
		return -EINVAL;

	if (is_dev_in_satc(bdf)) {
		if (ecap_dit(iommu->ecap))
			info.pfsid = bdf;
	} else if (data->ats_enabled || data->ats_supported) {
		return -EPERM;
	}

	info.bus = data->bus;
	info.devfn = data->devfn;
	info.iommu = iommu;
	info.ats_qdep = data->ats_qdep;
	info.ats_supported = data->ats_supported;
	info.ats_enabled = data->ats_enabled;
	if (data->did == FLPT_DEFAULT_DID) {
		/*
		 * Passthrough will break pkvm security guarantees as
		 * device would be able to access the whole physical
		 * memory range. Use Second stage translation with host ept
		 * as second stage pagetable so as to limit device access
		 * to host memory.
		 */
		domain.pgd = __pkvm_va(pkvm_host_ept_root());
		domain.agaw = level_to_agaw(pkvm_host_ept_level());
	} else {
		domain.pgd = pkvm_host_gpa_to_virt(data->pgd_gpa);
		domain.agaw = iommu->agaw;
	}

	ret = accept_page_donation(iommu, &data->donation_page_gpa);
	if (ret)
		return ret;

	pkvm_dbg("%s: dev[%x:%x], did: %d, pgd: %p, agaw: %d\n", __func__,
		 data->bus, data->devfn, data->did, domain.pgd, domain.agaw);
	return domain_context_mapping_one(&domain, iommu, &info, data->did,
					  data->bus, data->devfn);
}

int pkvm_iommu_set_lm_ce(struct set_lm_ce_data *in, struct set_lm_ce_data *out)
{
	int ret = iommu_set_lm_ce(in);

	*out = *in;
	return ret;
}

/*
 * Size of pasid directory in bytes, given the max pasid number
 * A pasid directory entry can address 64 pasids and a pasid
 * directory page holds 512 entries, hence one pasid dir page can
 * address (64 * 512) entries.
 * So pasid_dir_size = (max_pasid / (64 * 512)) * PAGE_SIZE
 *                   => = (max_pasid >> 15) << PAGE_SHIFT
 */
#define pasid_dir_size(max_pasid) ((max_pasid) >> (15 - PAGE_SHIFT))

static int iommu_set_sm_ce(struct set_sm_ce_data *data)
{
	struct intel_iommu *iommu = iommu_from_phys(data->phys);
	u16 bdf = PCI_DEVID(data->bus, data->devfn);
	struct device_domain_info info = { 0 };
	struct pasid_table table = { 0 };
	struct pkvm_device dev = { .info = &info };
	int ret;

	if (!iommu)
		return -EINVAL;

	if (data->ats_qdep > PCI_ATS_MAX_QDEP)
		return -EINVAL;

	if ((data->ats_supported || data->ats_enabled) &&
	    !is_dev_in_satc(bdf))
		return -EPERM;

	info.bus = data->bus;
	info.devfn = data->devfn;
	info.ats_qdep = data->ats_qdep;
	info.ats_supported = data->ats_supported;
	info.ats_enabled = data->ats_enabled;
	info.pasid_supported = data->pasid_supported;
	info.pasid_enabled = data->pasid_enabled;
	table.table = pkvm_host_gpa_to_virt(data->pasid_table_gpa);
	table.max_pasid = data->max_pasid;
	info.pasid_table = &table;
	info.iommu = iommu;

	ret = accept_page_donation(iommu, &data->donation_page_gpa);
	if (ret)
		return ret;

	ret = pkvm_host_donate_hyp_share_ro(pkvm_host_gpa_to_phys(data->pasid_table_gpa),
					    pasid_dir_size(data->max_pasid), true);
	if (ret) {
		pkvm_err("failed to write protect pasid dir for dev[%x:%x](err=%d)\n",
			 data->bus, data->devfn, ret);
		return ret;
	}

	__iommu_flush_cache(iommu, table.table, pasid_dir_size(data->max_pasid));

	pkvm_dbg("%s: dev[%x:%x], ats_qdep: %d, pasid_table_gpa: %llx\n", __func__,
		 data->bus, data->devfn, info.ats_qdep, data->pasid_table_gpa);
	ret = device_pasid_table_setup(&dev, data->bus, data->devfn);

	if (ret)
		pkvm_hyp_donate_host(pkvm_host_gpa_to_phys(data->pasid_table_gpa),
				     pasid_dir_size(data->max_pasid), false);
	return ret;
}

int pkvm_iommu_set_sm_ce(struct set_sm_ce_data *in, struct set_sm_ce_data *out)
{
	int ret = iommu_set_sm_ce(in);

	*out = *in;
	return ret;
}

static int __get_pasid_table(struct intel_iommu *iommu, u8 bus, u8 devfn, struct pasid_table *table)
{
	struct context_entry *context = iommu_context_addr(iommu, bus, devfn, false);
	u32 pds;

	if (!context || !context_present(context)) {
		pkvm_err("%s: pasid directory table not found: device[%x:%x]\n",
			 __func__, bus, devfn);
		return -EINVAL;
	}

	pds = get_pasid_dir_size(context);
	table->table = __pkvm_va(context->lo & VTD_PAGE_MASK);
	table->max_pasid = pds << PASID_PDE_SHIFT;

	return 0;
}

static int iommu_pasid_setup_fl(struct pasid_setup_fl_data *data)
{
	struct intel_iommu *iommu = iommu_from_phys(data->phys);
	u16 bdf = PCI_DEVID(data->bus, data->devfn);
	struct device_domain_info info = { 0 };
	struct pkvm_device dev = { .info = &info };
	struct pasid_table table = { 0 };
	u64 fsptptr;
	int ret;

	if (!iommu)
		return -EINVAL;

	if (data->ats_qdep > PCI_ATS_MAX_QDEP)
		return -EINVAL;

	if (is_dev_in_satc(bdf)) {
		if (ecap_dit(iommu->ecap))
			info.pfsid = bdf;
	} else if (data->ats_supported || data->ats_enabled) {
		return -EPERM;
	}

	ret = __get_pasid_table(iommu, data->bus, data->devfn, &table);
	if (ret)
		return ret;

	fsptptr = pkvm_host_gpa_to_phys(data->fsptptr_gpa);
	info.bus = data->bus;
	info.devfn = data->devfn;
	info.ats_qdep = data->ats_qdep;
	info.ats_enabled = data->ats_enabled;
	info.ats_supported = data->ats_supported;
	info.pasid_table = &table;
	info.iommu = iommu;

	ret = accept_page_donation(iommu, &data->donation_page_gpa);
	if (ret)
		return ret;

	pkvm_dbg("%s: dev[%x:%x], pasid: %x, fsptptr_gpa: %llx, did: %d, old_did: %d\n", __func__,
		 data->bus, data->devfn, data->pasid, data->fsptptr_gpa, data->did, data->old_did);
	if (!data->old_did) {
		return intel_pasid_setup_first_level(iommu, &dev, fsptptr,
						     data->pasid, data->did,
						     data->flags);
	}
	return intel_pasid_replace_first_level(iommu, &dev, fsptptr,
					       data->pasid, data->did,
					       data->old_did, data->flags);
}

int pkvm_iommu_pasid_setup_fl(struct pasid_setup_fl_data *in, struct pasid_setup_fl_data *out)
{
	int ret = iommu_pasid_setup_fl(in);

	*out = *in;
	return ret;
}

static int iommu_pasid_setup_sl(struct pasid_setup_sl_data *data)
{
	struct intel_iommu *iommu = iommu_from_phys(data->phys);
	u16 bdf = PCI_DEVID(data->bus, data->devfn);
	struct device_domain_info info = { 0 };
	struct pkvm_device dev = { .info = &info };
	struct dmar_domain domain = { 0 };
	struct pasid_table table = { 0 };
	int ret;

	if (!iommu)
		return -EINVAL;

	if (data->ats_qdep > PCI_ATS_MAX_QDEP)
		return -EINVAL;

	if (is_dev_in_satc(bdf)) {
		if (ecap_dit(iommu->ecap))
			info.pfsid = bdf;
	} else if (data->ats_supported || data->ats_enabled) {
		return -EPERM;
	}

	ret = __get_pasid_table(iommu, data->bus, data->devfn, &table);
	if (ret)
		return ret;

	info.bus = data->bus;
	info.devfn = data->devfn;
	info.pasid_table = &table;
	info.iommu = iommu;
	info.ats_qdep = data->ats_qdep;
	info.ats_supported = data->ats_supported;
	info.ats_enabled = data->ats_enabled;

	if (data->did == FLPT_DEFAULT_DID) {
		/*
		 * Passthrough will break pkvm security guarantees as
		 * device would be able to access the whole physical
		 * memory range. Use Second stage translation with host ept
		 * as second stage pagetable so as to limit device access
		 * to host memory.
		 */
		domain.pgd = __pkvm_va(pkvm_host_ept_root());
		domain.agaw = level_to_agaw(pkvm_host_ept_level());
	} else {
		domain.pgd = pkvm_host_gpa_to_virt(data->ssptptr_gpa);
		domain.agaw = iommu->agaw;
	}

	ret = accept_page_donation(iommu, &data->donation_page_gpa);
	if (ret)
		return ret;

	pkvm_dbg("%s: dev[%x:%x], pasid: %x ssptptr_gpa: %llx, did: %d, old_did: %d\n", __func__,
		 data->bus, data->devfn, data->pasid, data->ssptptr_gpa, data->did, data->old_did);
	if (!data->old_did) {
		return intel_pasid_setup_second_level(iommu, &domain, &dev,
						      data->did, data->pasid);
	}
	return intel_pasid_replace_second_level(iommu, &domain, &dev,
						data->did, data->old_did,
						data->pasid);
}

int pkvm_iommu_pasid_setup_sl(struct pasid_setup_sl_data *in, struct pasid_setup_sl_data *out)
{
	int ret = iommu_pasid_setup_sl(in);

	*out = *in;
	return ret;
}

int pkvm_iommu_pasid_teardown(struct pasid_teardown_data *data)
{
	struct intel_iommu *iommu = iommu_from_phys(data->phys);
	u16 bdf = PCI_DEVID(data->bus, data->devfn);
	struct device_domain_info info = { 0 };
	struct pkvm_device dev = { .info = &info };
	struct pasid_table table = { 0 };
	int ret;

	if (!iommu)
		return -EINVAL;

	if (data->ats_qdep > PCI_ATS_MAX_QDEP)
		return -EINVAL;

	if (is_dev_in_satc(bdf)) {
		if (ecap_dit(iommu->ecap))
			info.pfsid = bdf;
	} else if (data->ats_supported || data->ats_enabled) {
		return -EPERM;
	}

	ret = __get_pasid_table(iommu, data->bus, data->devfn, &table);
	if (ret)
		return ret;

	info.bus = data->bus;
	info.devfn = data->devfn;
	info.ats_qdep = data->ats_qdep;
	info.ats_enabled = data->ats_enabled;
	info.ats_supported = data->ats_supported;
	info.pasid_table = &table;
	info.iommu = iommu;

	pkvm_dbg("%s: dev[%x:%x], pasid: %x, ats_qdep: %d\n", __func__,
		 data->bus, data->devfn, data->pasid, data->ats_qdep);
	intel_pasid_tear_down_entry(iommu, &dev, data->pasid, false);
	return 0;
}

static int __validate_domain_params(struct intel_iommu *iommu, struct alloc_domain_data *data)
{
	int iommu_superpage = iommu_superpage_capability(iommu, data->use_first_level);
	int gaw = agaw_to_width(iommu->agaw);
	int ret = -EINVAL;

	if (gaw > cap_mgaw(iommu->cap))
		gaw = cap_mgaw(iommu->cap);

	if (data->iommu_superpage != iommu_superpage) {
		pkvm_err("%s: invalid iommu_superpage(%u) from host!\n",
			 __func__, data->iommu_superpage);
	} else if (data->iommu_coherency != iommu_paging_structure_coherency(iommu)) {
		pkvm_err("%s: invalid iommu_coherency(%u) from host!\n",
			 __func__, data->iommu_coherency);
	} else if (data->agaw != iommu->agaw) {
		pkvm_err("%s: invalid agaw(%u) from host!\n", __func__, data->agaw);
	} else if (data->gaw != gaw) {
		pkvm_err("%s: invalid gaw(%u) from host!\n", __func__, data->gaw);
	} else if (data->max_addr != __DOMAIN_MAX_ADDR(data->gaw)) {
		pkvm_err("%s: invalid max_addr(%llx) from host!\n",
			 __func__, data->max_addr);
	} else {
		ret = 0;
	}

	return ret;
}

int pkvm_iommu_alloc_domain(struct alloc_domain_data *data)
{
	struct dmar_domain *domain;
	struct intel_iommu *iommu;
	void *pgd;
	int ret;
	bool need_iotlb_sync_map;

	iommu = iommu_from_phys(data->phys);
	if (!iommu)
		return -EINVAL;

	ret = __validate_domain_params(iommu, data);
	if (ret)
		return ret;

	pgd = pkvm_host_gpa_to_virt(data->pgd_gpa);
	pkvm_dbg("%s: write protecting pgd: %p\n", __func__, pgd);
	ret = pkvm_host_donate_hyp_share_ro(__pkvm_pa(pgd), VTD_PAGE_SIZE, true);
	if (ret) {
		pkvm_err("%s: failed to write protect pgd: %p (err=%d)\n",
			 __func__, pgd, ret);
		return ret;
	}

	need_iotlb_sync_map = cap_caching_mode(iommu->cap) && !data->use_first_level;
	domain = pkvm_alloc_iommu_domain(data, need_iotlb_sync_map);
	if (IS_ERR(domain)) {
		pkvm_err("%s: domain alloc failed for device[%x] (err=%ld)\n",
			 __func__, data->bdf, PTR_ERR(domain));
		return PTR_ERR(domain);
	}

	domain_flush_cache(domain, pgd, VTD_PAGE_SIZE);

	pkvm_dbg("%s: allocated domain(pgd=%p) for device[%x]\n", __func__,
		 pgd, data->bdf);
	return 0;
}

int pkvm_iommu_free_domain(u64 pgd_gpa, struct pkvm_memcache *mc)
{
	struct dmar_domain *domain;
	void *pgd = pkvm_host_gpa_to_virt(pgd_gpa);
	int ret;

	domain = pkvm_get_iommu_domain_noref(pgd);
	if (!domain) {
		pkvm_err("%s: no domain exist for pgd: %p\n", __func__, pgd);
		return -EINVAL;
	}

	memset(mc, 0, sizeof(*mc));
	ret = pkvm_free_iommu_domain(domain, mc);
	if (ret) {
		pkvm_err("%s: failed to free the domain[pgd:%p] (err=%d)\n",
			 __func__, pgd, ret);
		return ret;
	}

	return ret;
}
