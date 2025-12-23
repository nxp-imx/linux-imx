// SPDX-License-Identifier: GPL-2.0
/*
 * IOMMU operations for pKVM
 *
 * Copyright (C) 2022 Linaro Ltd.
 */
#include <asm/kvm_hyp.h>
#include <asm/kvm_hypevents.h>

#include <hyp/adjust_pc.h>

#include <linux/iommu.h>
#include <kvm/device.h>

#include <nvhe/iommu.h>
#include <nvhe/mem_protect.h>
#include <nvhe/mm.h>
#include <nvhe/rwlock.h>
#include <nvhe/spinlock.h>

#define KVM_IOMMU_MAX_DRV	5
static DEFINE_HYP_RWLOCK(kvm_iommu_reg_lock);
/* Both protected by kvm_iommu_reg_lock */
static size_t nr_drivers;
static struct kvm_iommu_ops *kvm_iommu_drivers[KVM_IOMMU_MAX_DRV];

static struct hyp_pool iommu_pages_pool_atomic;

/* Hypervisor is non-preemptable, so cur_context can be per cpu. */
DEFINE_PER_CPU(struct pkvm_hyp_vcpu *, __cur_context);
#define cur_context (*this_cpu_ptr(&__cur_context))
static struct hyp_pool iommu_host_pool;
static bool iommu_pools_ready;

/*
 * We support multiple drivers for the host kernel, but only one for the guest,
 * this can be registered from the driver.
 */
pkvm_handle_t pviommu_drv_id = KVM_IOMMU_MAX_DRV;

DECLARE_PER_CPU(struct kvm_hyp_req, host_hyp_reqs);

static struct kvm_hyp_iommu_domain kvm_iommu_domains[KVM_IOMMU_MAX_DOMAINS];

/* Protects domains in kvm_iommu_domains */
static DEFINE_HYP_SPINLOCK(kvm_iommu_domain_lock);

#define for_each_drv(drv) \
	for (typeof(kvm_iommu_drivers[0]) *_pp = (kvm_iommu_drivers); \
	     _pp < &kvm_iommu_drivers[nr_drivers]; _pp++) \
		if (((drv) = *_pp))

static void kvm_iommu_drv_lock(void)
{
	hyp_read_lock(&kvm_iommu_reg_lock);
}

static void kvm_iommu_drv_unlock(void)
{
	hyp_read_unlock(&kvm_iommu_reg_lock);
}

static int kvm_iommu_refill(struct kvm_hyp_memcache *host_mc)
{
	if (!iommu_pools_ready)
		return -EINVAL;

	return refill_hyp_pool(&iommu_host_pool, host_mc);
}

static void kvm_iommu_reclaim(struct kvm_hyp_memcache *host_mc, int target)
{
	if (!iommu_pools_ready)
		return;

	reclaim_hyp_pool(&iommu_host_pool, host_mc, target);
}

static int kvm_iommu_reclaimable(void)
{
	if (!iommu_pools_ready)
		return 0;

	return hyp_pool_free_pages(&iommu_host_pool);
}

static struct kvm_iommu_ops *get_drv(pkvm_handle_t drv_id)
{
	struct kvm_iommu_ops *drv = NULL;

	kvm_iommu_drv_lock();
	if (drv_id < nr_drivers) {
		drv_id = array_index_nospec(drv_id, nr_drivers);
		drv = kvm_iommu_drivers[drv_id];
	}
	kvm_iommu_drv_unlock();
	return drv;
}

struct hyp_mgt_allocator_ops kvm_iommu_allocator_ops = {
	.refill = kvm_iommu_refill,
	.reclaim = kvm_iommu_reclaim,
	.reclaimable = kvm_iommu_reclaimable,
};

static inline int pkvm_to_iommu_prot(enum kvm_pgtable_prot prot)
{
	int iommu_prot = 0;

	if (prot & KVM_PGTABLE_PROT_R)
		iommu_prot |= IOMMU_READ;
	if (prot & KVM_PGTABLE_PROT_W)
		iommu_prot |= IOMMU_WRITE;
	if (prot == PKVM_HOST_MMIO_PROT)
		iommu_prot |= IOMMU_MMIO;

	/* We don't understand that, might be dangerous. */
	WARN_ON(prot & ~PKVM_HOST_MEM_PROT);
	return iommu_prot;
}

static int __snapshot_host_stage2(const struct kvm_pgtable_visit_ctx *ctx,
				  enum kvm_pgtable_walk_flags visit)
{
	u64 start = ctx->addr;
	kvm_pte_t pte = *ctx->ptep;
	u32 level = ctx->level;
	u64 end = start + kvm_granule_size(level);
	int prot = IOMMU_READ | IOMMU_WRITE;
	struct kvm_iommu_ops *ops = (struct kvm_iommu_ops *)ctx->arg;

	/* Keep unmapped. */
	if (pte && !kvm_pte_valid(pte))
		return 0;

	if (kvm_pte_valid(pte))
		prot = pkvm_to_iommu_prot(kvm_pgtable_stage2_pte_prot(pte));
	else if (!addr_is_memory(start))
		prot |= IOMMU_MMIO;

	ops->host_stage2_idmap(start, end, prot);
	return 0;
}

static int kvm_iommu_snapshot_host_stage2(struct kvm_iommu_ops *ops)
{
	int ret;
	struct kvm_pgtable_walker walker = {
		.cb	= __snapshot_host_stage2,
		.flags	= KVM_PGTABLE_WALK_LEAF,
		.arg = ops,
	};
	struct kvm_pgtable *pgt = &host_mmu.pgt;

	hyp_spin_lock(&host_mmu.lock);
	ret = kvm_pgtable_walk(pgt, 0, BIT(pgt->ia_bits), &walker);
	hyp_spin_unlock(&host_mmu.lock);

	return ret;
}

int kvm_iommu_init(void *pool_base, size_t nr_pages)
{
	int ret;

	if (nr_pages) {
		ret = hyp_pool_init(&iommu_pages_pool_atomic, hyp_virt_to_pfn(pool_base),
				    nr_pages, 0);
		if (ret)
			return ret;
	}

	ret = hyp_pool_init_empty(&iommu_host_pool, 64);
	if (ret)
		return ret;

	iommu_pools_ready = true;
	return ret;
}

int kvm_iommu_register_ops(struct kvm_iommu_ops *ops, pkvm_handle_t *drv_id)
{
	int ret;

	if (!ops || !ops->init ||
	    !ops->host_stage2_idmap)
		return -ENODEV;

	hyp_write_lock(&kvm_iommu_reg_lock);

	if (nr_drivers == KVM_IOMMU_MAX_DRV) {
		hyp_write_unlock(&kvm_iommu_reg_lock);
		return -EBUSY;
	}

	*drv_id = nr_drivers;
	/* Reserve a place for this driver but it is registered later */
	nr_drivers++;

	hyp_write_unlock(&kvm_iommu_reg_lock);

	/*
	 * Init may require donation which has to be done outside the reg lock.
	 * The driver spot is already reserved so no race can happen on it.
	 */
	ret = ops->init(*drv_id);
	if (ret)
		return ret;

	hyp_write_lock(&kvm_iommu_reg_lock);
	ret = kvm_iommu_snapshot_host_stage2(ops);
	if (!ret)
		kvm_iommu_drivers[*drv_id] = ops;

	hyp_write_unlock(&kvm_iommu_reg_lock);
	return ret;
}

void kvm_iommu_host_stage2_idmap(phys_addr_t start, phys_addr_t end,
				 enum kvm_pgtable_prot prot)
{
	struct kvm_iommu_ops *kvm_iommu_ops;

	hyp_assert_lock_held(&host_mmu.lock);

	trace_iommu_idmap(start, end, prot);
	kvm_iommu_drv_lock();
	for_each_drv(kvm_iommu_ops) {
		kvm_iommu_ops->host_stage2_idmap(start, end, pkvm_to_iommu_prot(prot));
	}
	kvm_iommu_drv_unlock();
}

/* Return current vcpu or NULL for host. */
static struct pkvm_hyp_vcpu *__get_vcpu(void)
{
	struct kvm_vcpu *vcpu = this_cpu_ptr(&kvm_host_data)->host_ctxt.__hyp_running_vcpu;

	if (vcpu)
		return container_of(vcpu, struct pkvm_hyp_vcpu, vcpu);
	/* Maybe guest is not loaded but we are in teardown context. */
	return cur_context;
}

static void *__kvm_iommu_donate_pages(struct hyp_pool *pool,
				      u8 order, int flags)
{
	void *p;
	struct kvm_hyp_req *req = this_cpu_ptr(&host_hyp_reqs);
	size_t size = (1 << order) * PAGE_SIZE;
	struct pkvm_hyp_vcpu *hyp_vcpu = __get_vcpu();

	p = hyp_alloc_pages(pool, order);
	if (p)
		return p;

	if (hyp_vcpu) {
		req = pkvm_hyp_req_reserve(hyp_vcpu, KVM_HYP_REQ_TYPE_MEM);
		if (WARN_ON(!req))
			return NULL;
	}

	req->type = KVM_HYP_REQ_TYPE_MEM;
	req->mem.dest = REQ_MEM_DEST_HYP_IOMMU;
	req->mem.sz_alloc = size;
	req->mem.nr_pages = 1;
	return NULL;
}

static void __kvm_iommu_reclaim_pages(struct hyp_pool *pool, void *p, u8 order)
{
	hyp_put_page(pool, p);
}

void *kvm_iommu_donate_pages(u8 order, int flags)
{
	struct pkvm_hyp_vcpu *hyp_vcpu = __get_vcpu();
	struct hyp_pool *pool;

	if (hyp_vcpu)
		pool = &pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu)->iommu_pool;
	else
		pool = &iommu_host_pool;

	return __kvm_iommu_donate_pages(pool, order, flags);
}

void kvm_iommu_reclaim_pages(void *p, u8 order)
{
	struct pkvm_hyp_vcpu *hyp_vcpu = __get_vcpu();
	struct hyp_pool *pool;

	if (hyp_vcpu)
		pool = &pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu)->iommu_pool;
	else
		pool = &iommu_host_pool;

	__kvm_iommu_reclaim_pages(pool, p, order);
}

void *kvm_iommu_donate_pages_atomic(u8 order)
{
	return hyp_alloc_pages(&iommu_pages_pool_atomic, order);
}

void kvm_iommu_reclaim_pages_atomic(void *ptr)
{
	hyp_put_page(&iommu_pages_pool_atomic, ptr);
}

bool kvm_iommu_host_dabt_handler(struct user_pt_regs *regs, u64 esr, u64 addr)
{
	struct kvm_iommu_ops *kvm_iommu_ops;

	kvm_iommu_drv_lock();

	for_each_drv(kvm_iommu_ops) {
		if (kvm_iommu_ops && kvm_iommu_ops->dabt_handler &&
		    kvm_iommu_ops->dabt_handler(regs, esr, addr)) {
			/* DABT handled by the driver, skip to next instruction. */
			kvm_skip_host_instr();
			kvm_iommu_drv_unlock();
			return true;
		}
	}
	kvm_iommu_drv_unlock();
	return false;
}

void kvm_iommu_host_stage2_idmap_complete(bool map)
{
	struct kvm_iommu_ops *kvm_iommu_ops;

	trace_iommu_idmap_complete(map);
	kvm_iommu_drv_lock();
	for_each_drv(kvm_iommu_ops) {
		if (kvm_iommu_ops->host_stage2_idmap_complete)
			kvm_iommu_ops->host_stage2_idmap_complete(map);
	}
	kvm_iommu_drv_unlock();
}

static struct kvm_hyp_iommu_domain *handle_to_domain(pkvm_handle_t domain_id)
{
	if (domain_id >= KVM_IOMMU_MAX_DOMAINS)
		return NULL;

	domain_id = array_index_nospec(domain_id, KVM_IOMMU_MAX_DOMAINS);

	return &kvm_iommu_domains[domain_id];
}

static int domain_get(struct kvm_hyp_iommu_domain *domain)
{
	int old = atomic_fetch_inc_acquire(&domain->refs);
	struct pkvm_hyp_vcpu *hyp_vcpu = __get_vcpu();
	int ret = 0;

	BUG_ON(!old || (old + 1 < 0));

	/* check done after refcount is elevated to avoid race with alloc_domain */
	if (!hyp_vcpu && domain->vm)
		ret = -EPERM;
	if (hyp_vcpu && (domain->vm != pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu)))
		ret = -EPERM;

	if (ret)
		atomic_dec_return_release(&domain->refs);
	return ret;
}

static void domain_put(struct kvm_hyp_iommu_domain *domain)
{
	struct pkvm_hyp_vcpu *hyp_vcpu = __get_vcpu();

	BUG_ON(!atomic_dec_return_release(&domain->refs));
	WARN_ON(!hyp_vcpu && domain->vm);
	WARN_ON(hyp_vcpu && (domain->vm != pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu)));
}

int kvm_iommu_alloc_domain(pkvm_handle_t drv_id, pkvm_handle_t iommu_id,
			   pkvm_handle_t domain_id, int type)
{
	int ret = -EINVAL;
	struct kvm_hyp_iommu_domain *domain;
	struct kvm_iommu_ops *kvm_iommu_ops;
	struct pkvm_hyp_vcpu *hyp_vcpu = __get_vcpu();
	struct pkvm_hyp_vm *vm;

	kvm_iommu_ops = get_drv(drv_id);
	if (!kvm_iommu_ops || !kvm_iommu_ops->alloc_domain)
		return -ENODEV;

	/*
	 * Host only has access to the lower half of the domain IDs.
	 * Guest ID space is managed by the hypervisor, so it is trusted.
	 */
	if (!hyp_vcpu && (domain_id >= (KVM_IOMMU_MAX_DOMAINS >> 1)))
		return -EINVAL;

	domain = handle_to_domain(domain_id);
	if (!domain)
		return -ENOMEM;

	hyp_spin_lock(&kvm_iommu_domain_lock);
	if (atomic_read(&domain->refs))
		goto out_unlock;

	domain->domain_id = domain_id;
	ret = kvm_iommu_ops->alloc_domain(iommu_id, domain, type);
	if (ret)
		goto out_unlock;
	domain->driver = kvm_iommu_ops;

	if (hyp_vcpu) {
		vm = pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu);
		domain->vm = vm;
	}
	atomic_set_release(&domain->refs, 1);
out_unlock:
	hyp_spin_unlock(&kvm_iommu_domain_lock);
	return ret;
}

int kvm_iommu_free_domain(pkvm_handle_t domain_id)
{
	int ret = 0;
	struct kvm_hyp_iommu_domain *domain;
	struct kvm_iommu_ops *kvm_iommu_ops;
	struct pkvm_hyp_vcpu *hyp_vcpu = __get_vcpu();

	domain = handle_to_domain(domain_id);
	if (!domain)
		return -EINVAL;

	hyp_spin_lock(&kvm_iommu_domain_lock);
	kvm_iommu_ops = domain->driver;
	if (!kvm_iommu_ops || !kvm_iommu_ops->free_domain) {
		ret = -ENODEV;
		goto out_unlock;
	}

	if (hyp_vcpu) {
		if (domain->vm != pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu)) {
			ret = -EPERM;
			goto out_unlock;
		}
	}

	if (WARN_ON(atomic_cmpxchg_acquire(&domain->refs, 1, 0) != 1)) {
		ret = -EINVAL;
		goto out_unlock;
	}

	kvm_iommu_ops->free_domain(domain);
	memset(domain, 0, sizeof(*domain));
out_unlock:
	hyp_spin_unlock(&kvm_iommu_domain_lock);

	return ret;
}

int kvm_iommu_force_free_domain(pkvm_handle_t domain_id, struct pkvm_hyp_vm *vm)
{
	struct kvm_hyp_iommu_domain *domain = handle_to_domain(domain_id);
	struct kvm_iommu_ops *kvm_iommu_ops;
	int ret = 0;

	BUG_ON(!domain);
	cur_context = vm->vcpus[0];

	hyp_spin_lock(&kvm_iommu_domain_lock);
	kvm_iommu_ops = domain->driver;
	if (!kvm_iommu_ops || !kvm_iommu_ops->free_domain) {
		ret = -ENODEV;
		goto out_unlock;
	}

	atomic_set(&domain->refs, 0);
	kvm_iommu_ops->free_domain(domain);
	memset(domain, 0, sizeof(*domain));

out_unlock:
	hyp_spin_unlock(&kvm_iommu_domain_lock);
	cur_context = NULL;

	return ret;
}

int kvm_iommu_attach_dev(pkvm_handle_t iommu_id, pkvm_handle_t domain_id,
			 u32 endpoint_id, u32 pasid, u32 pasid_bits, unsigned long flags)
{
	int ret;
	struct kvm_hyp_iommu_domain *domain;
	struct kvm_iommu_ops *kvm_iommu_ops;
	struct pkvm_hyp_vcpu *hyp_vcpu = __get_vcpu();
	struct pkvm_hyp_vm *vm = NULL;

	if (hyp_vcpu)
		vm = pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu);
	/*
	 * Make sure device can't transition to/from VMs while in the middle of attach.
	 */
	ret = pkvm_devices_get_context(iommu_id, endpoint_id, vm);
	if (ret)
		return ret;

	hyp_spin_lock(&kvm_iommu_domain_lock);
	domain = handle_to_domain(domain_id);
	if (!domain || domain_get(domain)) {
		ret = -EINVAL;
		goto out_unlock;
	}

	kvm_iommu_ops = domain->driver;
	if (!kvm_iommu_ops || !kvm_iommu_ops->attach_dev) {
		ret = -ENODEV;
		domain_put(domain);
		goto out_unlock;
	}

	ret = kvm_iommu_ops->attach_dev(iommu_id, domain,
				        endpoint_id, pasid, pasid_bits, flags);
	if (ret)
		domain_put(domain);
out_unlock:
	hyp_spin_unlock(&kvm_iommu_domain_lock);
	pkvm_devices_put_context(iommu_id, endpoint_id);

	return ret;
}

int kvm_iommu_detach_dev(pkvm_handle_t iommu_id, pkvm_handle_t domain_id,
			 u32 endpoint_id, u32 pasid)
{
	int ret;
	struct kvm_hyp_iommu_domain *domain;
	struct kvm_iommu_ops *kvm_iommu_ops;
	struct pkvm_hyp_vcpu *hyp_vcpu = __get_vcpu();
	struct pkvm_hyp_vm *vm = NULL;

	if (hyp_vcpu)
		vm = pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu);
	/* See kvm_iommu_attach_dev(). */
	ret = pkvm_devices_get_context(iommu_id, endpoint_id, vm);
	if (ret)
		return ret;

	hyp_spin_lock(&kvm_iommu_domain_lock);
	domain = handle_to_domain(domain_id);
	if (!domain || atomic_read(&domain->refs) <= 1) {
		ret = -EINVAL;
		goto out_unlock;
	}
	kvm_iommu_ops = domain->driver;
	if (!kvm_iommu_ops || !kvm_iommu_ops->detach_dev) {
		ret = -ENODEV;
		goto out_unlock;
	}

	ret = kvm_iommu_ops->detach_dev(iommu_id, domain, endpoint_id, pasid);
	if (ret)
		goto out_unlock;
	domain_put(domain);
out_unlock:
	hyp_spin_unlock(&kvm_iommu_domain_lock);
	pkvm_devices_put_context(iommu_id, endpoint_id);
	return ret;
}

#define IOMMU_PROT_MASK (IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE |\
			 IOMMU_NOEXEC | IOMMU_MMIO | IOMMU_PRIV)

int kvm_iommu_map_pages(pkvm_handle_t domain_id,
			unsigned long iova, phys_addr_t paddr, size_t pgsize,
			size_t pgcount, int prot, unsigned long *mapped)
{
	size_t size;
	int ret;
	struct kvm_hyp_iommu_domain *domain;
	struct kvm_iommu_ops *kvm_iommu_ops;

	*mapped = 0;

	if (prot & ~IOMMU_PROT_MASK)
		return -EOPNOTSUPP;

	if (__builtin_mul_overflow(pgsize, pgcount, &size) ||
	    iova + size < iova || paddr + size < paddr)
		return -E2BIG;

	domain = handle_to_domain(domain_id);
	if (!domain || domain_get(domain))
		return -ENOENT;

	kvm_iommu_ops = domain->driver;
	if (!kvm_iommu_ops || !kvm_iommu_ops->map_pages) {
		domain_put(domain);
		ret = -ENODEV;
		return ret;
	}
	ret = kvm_iommu_ops->map_pages(domain, iova, paddr, pgsize, pgcount,
				       prot, mapped);

	domain_put(domain);
	/* Mask -ENOMEM, as it's passed as a request. */
	return ret == -ENOMEM ? 0 : ret;
}

static inline void kvm_iommu_iotlb_sync(struct kvm_hyp_iommu_domain *domain,
					struct iommu_iotlb_gather *iotlb_gather)
{
	if (domain->driver->iotlb_sync)
		domain->driver->iotlb_sync(domain, iotlb_gather);

	iommu_iotlb_gather_init(iotlb_gather);
}

void kvm_iommu_iotlb_gather_add_page(struct kvm_hyp_iommu_domain *domain,
				     struct iommu_iotlb_gather *gather,
				     unsigned long iova,
				     size_t size)
{
	_iommu_iotlb_add_page(domain, gather, iova, size, kvm_iommu_iotlb_sync);
}

size_t kvm_iommu_unmap_pages(pkvm_handle_t domain_id, unsigned long iova,
			     size_t pgsize, size_t pgcount)
{
	size_t size;
	size_t unmapped;
	struct kvm_hyp_iommu_domain *domain;
	struct iommu_iotlb_gather iotlb_gather;
	struct kvm_iommu_ops *kvm_iommu_ops;

	if (!pgsize || !pgcount)
		return 0;

	if (__builtin_mul_overflow(pgsize, pgcount, &size) ||
	    iova + size < iova)
		return 0;

	domain = handle_to_domain(domain_id);
	if (!domain || domain_get(domain))
		return 0;

	kvm_iommu_ops = domain->driver;
	if (!kvm_iommu_ops || !kvm_iommu_ops->unmap_pages) {
		domain_put(domain);
		return 0;
	}

	iommu_iotlb_gather_init(&iotlb_gather);
	unmapped = kvm_iommu_ops->unmap_pages(domain, iova, pgsize,
					      pgcount, &iotlb_gather);
	kvm_iommu_iotlb_sync(domain, &iotlb_gather);
	domain_put(domain);
	return unmapped;
}

phys_addr_t kvm_iommu_iova_to_phys(pkvm_handle_t domain_id, unsigned long iova)
{
	phys_addr_t phys = 0;
	struct kvm_hyp_iommu_domain *domain;
	struct kvm_iommu_ops *kvm_iommu_ops;

	domain = handle_to_domain( domain_id);

	if (!domain || domain_get(domain))
		return 0;

	kvm_iommu_ops = domain->driver;
	if (!kvm_iommu_ops || !kvm_iommu_ops->iova_to_phys) {
		domain_put(domain);
		return 0;
	}

	phys = kvm_iommu_ops->iova_to_phys(domain, iova);
	domain_put(domain);
	return phys;
}

int kvm_iommu_set_identity(pkvm_handle_t drv_id, pkvm_handle_t iommu,
			   pkvm_handle_t dev, bool on)
{
	struct kvm_iommu_ops *kvm_iommu_ops = get_drv(drv_id);
	int ret;

	if (!kvm_iommu_ops || !kvm_iommu_ops->set_identity)
		return -ENODEV;

	/* set_identity not exposed to guests. */
	ret = pkvm_devices_get_context(iommu, dev, NULL);
	if (ret)
		return ret;
	ret = kvm_iommu_ops->set_identity(iommu, dev, on);
	pkvm_devices_put_context(iommu, dev);
	return ret;
}

size_t kvm_iommu_map_sg(pkvm_handle_t domain_id, unsigned long iova, struct kvm_iommu_sg *sg,
			unsigned int nent, unsigned int prot)
{
	int ret;
	size_t total_mapped = 0, mapped;
	struct kvm_hyp_iommu_domain *domain;
	phys_addr_t phys;
	size_t size, pgsize, pgcount;
	unsigned int orig_nent = nent;
	struct kvm_iommu_sg *orig_sg = sg;
	struct kvm_iommu_ops *kvm_iommu_ops;

	if (prot & ~IOMMU_PROT_MASK)
		return 0;

	domain = handle_to_domain(domain_id);
	if (!domain || domain_get(domain))
		return 0;

	kvm_iommu_ops = domain->driver;
	if (!kvm_iommu_ops || !kvm_iommu_ops->map_pages) {
		domain_put(domain);
		return 0;
	}

	ret = hyp_pin_shared_mem(sg, sg + nent);
	if (ret)
		goto out_put_domain;

	while (nent--) {
		phys = sg->phys;
		pgsize = sg->pgsize;
		pgcount = sg->pgcount;

		if (__builtin_mul_overflow(pgsize, pgcount, &size) ||
		    iova + size < iova)
			goto out_unpin_sg;

		mapped = 0;
		kvm_iommu_ops->map_pages(domain, iova, phys, pgsize, pgcount, prot, &mapped);
		total_mapped += mapped;
		phys += mapped;
		iova += mapped;
		/* Might need memory */
		if (mapped != size)
			break;
		sg++;
	}

out_unpin_sg:
	hyp_unpin_shared_mem(orig_sg, orig_sg + orig_nent);
out_put_domain:
	domain_put(domain);
	return total_mapped;
}

int kvm_iommu_iotlb_sync_map(pkvm_handle_t domain_id,
			     unsigned long iova, size_t size)
{
	struct kvm_hyp_iommu_domain *domain;
	struct kvm_iommu_ops *kvm_iommu_ops;
	int ret;

	if (!size || (iova + size < iova))
		return -EINVAL;

	domain = handle_to_domain(domain_id);
	if (!domain || domain_get(domain))
		return -EINVAL;

	kvm_iommu_ops = domain->driver;
	if (!kvm_iommu_ops || !kvm_iommu_ops->iotlb_sync_map)
		return -ENODEV;

	ret = kvm_iommu_ops->iotlb_sync_map(domain, iova, size);
	domain_put(domain);
	return ret;
}

int kvm_iommu_register_pviommu_drv(pkvm_handle_t drv_id)
{
	return cmpxchg_release(&pviommu_drv_id, KVM_IOMMU_MAX_DRV, drv_id) ==
		KVM_IOMMU_MAX_DRV ? 0 : -EBUSY;
}

int kvm_iommu_dev_block_dma(pkvm_handle_t iommu_id, u32 endpoint_id, bool host_to_guest)
{
	struct kvm_iommu_ops *kvm_iommu_ops = get_drv(pviommu_drv_id);

	if (!kvm_iommu_ops || !kvm_iommu_ops->dev_block_dma)
		return -ENODEV;

	return kvm_iommu_ops->dev_block_dma(iommu_id, endpoint_id, host_to_guest);
}

int iommu_pkvm_use_dma(u64 phys_addr, size_t size)
{
	return __pkvm_use_dma(phys_addr, size, __get_vcpu());
}

int iommu_pkvm_unuse_dma(u64 phys_addr, size_t size)
{
	return __pkvm_unuse_dma(phys_addr, size, __get_vcpu());
}

int kvm_iommu_id_to_token(pkvm_handle_t id, u64 *out_token)
{
	struct kvm_iommu_ops *kvm_iommu_ops = get_drv(pviommu_drv_id);

	if (!kvm_iommu_ops || !kvm_iommu_ops->get_iommu_token_by_id)
		return -ENODEV;
	return kvm_iommu_ops->get_iommu_token_by_id(id, out_token);
}
