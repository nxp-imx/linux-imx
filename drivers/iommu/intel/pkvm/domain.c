// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2026 Google. */


#include <linux/hashtable.h>
#include <asm/pkvm_spinlock.h>
#include "pkvm/debug.h"
#include "pkvm/memory.h"
#include "../iommu.h"

/*
 * TODO: Make this a dynamic value.
 */
#define MAX_IOMMU_DOMAIN_NUM	128
static DEFINE_HASHTABLE(iommu_domain_hasht, 8);
static DECLARE_BITMAP(iommu_domains_bitmap, MAX_IOMMU_DOMAIN_NUM);
static struct dmar_domain iommu_domains[MAX_IOMMU_DOMAIN_NUM];
static DEFINE_PKVM_SPINLOCK(iommu_domain_lock);
struct dmar_domain pt_domain;

void init_pt_domain(void)
{
	INIT_LIST_HEAD(&pt_domain.cache_tags);
	pkvm_spin_lock_init(&pt_domain.cache_lock);
	pt_domain.qi_batch = &pt_domain._qi_batch;
}

static struct dmar_domain *__pkvm_get_iommu_domain_locked(void *pgd, bool inc_ref)
{
	struct dmar_domain *domain;

	hash_for_each_possible(iommu_domain_hasht, domain, hnode, (u64)pgd) {
		if (domain->pgd != pgd)
			continue;

		if (inc_ref && WARN_ON_ONCE(!atomic_inc_not_zero(&domain->refcount)))
			return NULL;

		return domain;
	}

	return NULL;
}

struct dmar_domain *pkvm_get_iommu_domain(void *pgd)
{
	struct dmar_domain *domain;

	pkvm_spin_lock(&iommu_domain_lock);
	domain = __pkvm_get_iommu_domain_locked(pgd, true);
	pkvm_spin_unlock(&iommu_domain_lock);

	return domain;
}

/*
 * Retrieve the domain without incrementing refcount.
 * This api is useful when there is a refcount on the domain
 * and refcount is guaranteed to be not dropped.
 */
struct dmar_domain *pkvm_get_iommu_domain_noref(void *pgd)
{
	struct dmar_domain *domain;

	pkvm_spin_lock(&iommu_domain_lock);
	domain = __pkvm_get_iommu_domain_locked(pgd, false);
	pkvm_spin_unlock(&iommu_domain_lock);

	return domain;
}

void pkvm_put_iommu_domain(struct dmar_domain *domain)
{
	WARN_ON_ONCE(atomic_dec_if_positive(&domain->refcount) <= 0);
}

int pkvm_get_domain_cache_tag_assign(void *pgd, int did, u32 pasid,
				     struct device_domain_info *info)
{
	struct pkvm_device dev = { .info = info };
	struct dmar_domain *domain;
	int ret;

	if (did == FLPT_DEFAULT_DID)
		return cache_tag_assign_domain(&pt_domain, did, &dev, pasid);

	domain = pkvm_get_iommu_domain(pgd);
	if (!domain) {
		pkvm_err("%s: Failed to locate domain with pgd: %p\n",
			 __func__, pgd);
		return -EFAULT;
	}

	ret = cache_tag_assign_domain(domain, did, &dev, pasid);
	if (ret) {
		pkvm_put_iommu_domain(domain);
		return ret;
	}
	return 0;
}

void pkvm_put_domain_cache_tag_unassign(void *pgd, int did, u32 pasid,
					struct device_domain_info *info)
{
	struct pkvm_device dev = { .info = info };
	struct dmar_domain *domain;

	if (did == FLPT_DEFAULT_DID) {
		cache_tag_unassign_domain(&pt_domain, did, &dev, pasid);
		return;
	}

	domain = pkvm_get_iommu_domain_noref(pgd);
	BUG_ON(!domain);

	cache_tag_unassign_domain(domain, did, &dev, pasid);
	pkvm_put_iommu_domain(domain);
}

static void *admit_host_domain_page(void *arg)
{
	struct pkvm_memcache *host_mc = (struct pkvm_memcache *)arg;
	void *page;

	page = pop_pkvm_memcache_page(host_mc, pkvm_host_gpa_to_virt);
	if (!page)
		return NULL;

	if (WARN_ON(pkvm_host_donate_hyp_share_ro(__pkvm_pa(page), VTD_PAGE_SIZE, true))) {
		push_pkvm_memcache_page(host_mc, page, pkvm_virt_to_host_gpa);
		return NULL;
	}

	return page;
}

static int refill_domain_memcache(struct dmar_domain *domain,
				  struct pkvm_memcache *host_mc)
{
	struct pkvm_memcache *mc = &domain->mc;
	unsigned long min_pages;

	/*
	 * The host expects pKVM to drain the memcache fully and use it until
	 * the domain is freed, as the host itself doesn't store the memcache
	 * in any persistent data structure. This will not result in the
	 * hypervisor's memcache growing endlessly, since the host only
	 * provides a memcache if the hypervisor's memcache doesn't already
	 * have enough pages.
	 */
	min_pages = mc->count + host_mc->count;
	return topup_pkvm_memcache(mc, min_pages, admit_host_domain_page,
				   pkvm_virt_to_phys, host_mc);
}

static void free_domain_memcache(struct dmar_domain *domain,
				 struct pkvm_memcache *teardown_mc)
{
	struct pkvm_memcache *mc = &domain->mc;

	while (mc->count) {
		void *addr = pop_pkvm_memcache_page(mc, pkvm_phys_to_virt);

		push_pkvm_memcache_page(teardown_mc, addr, pkvm_virt_to_host_gpa);
		pkvm_hyp_donate_host(__pkvm_pa(addr), VTD_PAGE_SIZE, false);
	}
}

int pkvm_free_iommu_domain(struct dmar_domain *domain, struct pkvm_memcache *teardown_mc)
{
	if (atomic_cmpxchg(&domain->refcount, 1, 0) != 1) {
		pkvm_err("%s: domain[pgd:%p] has users, refcount %d\n",
			 __func__, domain->pgd, atomic_read(&domain->refcount));
		return -EBUSY;
	}

	/* Unmap any remaining mappings. */
	domain_unmap(domain, 0, DOMAIN_MAX_PFN(domain->gaw), NULL);
	free_domain_memcache(domain, teardown_mc);
	/*
	 * pgd was not allocated through memcache, but its safe to return to
	 * memcache as the teardown mc frees it the same way host driver frees
	 * the pages.
	 */
	push_pkvm_memcache_page(teardown_mc, domain->pgd, pkvm_virt_to_host_gpa);
	pkvm_hyp_donate_host(__pkvm_pa(domain->pgd), VTD_PAGE_SIZE, false);

	pkvm_dbg("%s: freeing domain[pgd: %p], freed pages: %lu\n",
		 __func__, domain->pgd, teardown_mc->count);

	pkvm_spin_lock(&iommu_domain_lock);
	hash_del(&domain->hnode);
	__clear_bit(domain->index, iommu_domains_bitmap);
	memset(domain, 0, sizeof(struct dmar_domain));
	pkvm_spin_unlock(&iommu_domain_lock);

	return 0;
}

struct dmar_domain *pkvm_alloc_iommu_domain(struct alloc_domain_data *data,
					    bool need_iotlb_sync_map)
{
	void *pgd = pkvm_host_gpa_to_virt(data->pgd_gpa);
	struct dmar_domain *domain;
	unsigned long index;

	pkvm_spin_lock(&iommu_domain_lock);
	domain = __pkvm_get_iommu_domain_locked(pgd, false);
	if (unlikely(domain)) {
		pkvm_spin_unlock(&iommu_domain_lock);
		return ERR_PTR(-EEXIST);
	}

	index = find_first_zero_bit(iommu_domains_bitmap, MAX_IOMMU_DOMAIN_NUM);
	if (index < MAX_IOMMU_DOMAIN_NUM) {
		__set_bit(index, iommu_domains_bitmap);
		domain = &iommu_domains[index];
		INIT_LIST_HEAD(&domain->cache_tags);
		domain->pgd = pgd;
		domain->use_first_level = data->use_first_level;
		domain->iommu_superpage = data->iommu_superpage;
		domain->iommu_coherency = data->iommu_coherency;
		domain->iotlb_sync_map = need_iotlb_sync_map;
		domain->agaw = data->agaw;
		domain->gaw = data->gaw;
		domain->max_addr = data->max_addr;
		domain->index = index;
		domain->qi_batch = &domain->_qi_batch;
		atomic_set(&domain->refcount, 1);
		pkvm_spin_lock_init(&domain->lock);
		pkvm_spin_lock_init(&domain->cache_lock);
		hash_add(iommu_domain_hasht, &domain->hnode, (u64)pgd);
		pkvm_dbg("%s: allocated domain pgd: %p\n", __func__, pgd);
	} else {
		domain = ERR_PTR(-ENOMEM);
	}
	pkvm_spin_unlock(&iommu_domain_lock);

	return domain;
}

static int iommu_domain_map(struct domain_map_data *data)
{
	struct dmar_domain *domain;
	u64 iova, phys, size, end;
	int ret;

	/* Check for possible overflows that may have security implications */
	if (check_shl_overflow(data->iov_pfn, VTD_PAGE_SHIFT, &iova))
		return -EINVAL;
	if (check_shl_overflow(data->phys_pfn, VTD_PAGE_SHIFT, &phys))
		return -EINVAL;
	if (check_mul_overflow(data->nr_pages, VTD_PAGE_SIZE, &size))
		return -EINVAL;
	if (check_add_overflow(iova, size, &end))
		return -EINVAL;
	if (check_add_overflow(phys, size, &end))
		return -EINVAL;

	domain = pkvm_get_iommu_domain(pkvm_host_gpa_to_virt(data->pgd_gpa));
	if (!domain) {
		pkvm_err("%s: failed to get the domain [pgd:%llx]\n",
			 __func__, data->pgd_gpa);
		return -EINVAL;
	}

	pkvm_spin_lock(&domain->lock);
	if (data->mc.count) {
		ret = refill_domain_memcache(domain, &data->mc);
		if (ret) {
			pkvm_err("%s: failed to refill memcache for domain[pgd: %p] (err=%d)\n",
				 __func__, domain->pgd, ret);
			goto out_unlock;
		}
	}
	if (domain->mc.count < __pkvm_pgtable_max_pages(data->nr_pages)) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	ret = domain_map(domain, data->iov_pfn, data->phys_pfn,
			 data->nr_pages, data->prot, 0);

out_unlock:
	pkvm_spin_unlock(&domain->lock);
	pkvm_put_iommu_domain(domain);

	return ret;
}

int pkvm_iommu_domain_map(struct domain_map_data *in, struct domain_map_data *out)
{
	int ret = iommu_domain_map(in);

	*out = *in;
	return ret;
}

int pkvm_iommu_domain_unmap(u64 pgd_gpa, u64 start_pfn, u64 last_pfn)
{
	struct dmar_domain *domain;

	domain = pkvm_get_iommu_domain(pkvm_host_gpa_to_virt(pgd_gpa));
	if (!domain) {
		pkvm_err("%s, failed to get the domain [pgd:%llx]\n",
			 __func__, pgd_gpa);
		return -EINVAL;
	}

	pkvm_spin_lock(&domain->lock);
	domain_unmap(domain, start_pfn, last_pfn, NULL);
	pkvm_spin_unlock(&domain->lock);

	pkvm_put_iommu_domain(domain);

	return 0;
}
