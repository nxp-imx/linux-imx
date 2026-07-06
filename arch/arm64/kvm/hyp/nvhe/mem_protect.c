// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Google LLC
 * Author: Quentin Perret <qperret@google.com>
 */

#include <linux/kvm_host.h>

#include <asm/kvm_emulate.h>
#include <asm/kvm_hyp.h>
#include <asm/kvm_hypevents.h>
#include <asm/kvm_mmu.h>
#include <asm/kvm_pgtable.h>
#include <asm/kvm_pkvm.h>
#include <asm/stage2_pgtable.h>

#include <hyp/fault.h>

#include <nvhe/arm-smccc.h>
#include <nvhe/errno.h>
#include <nvhe/gfp.h>
#include <nvhe/iommu.h>
#include <nvhe/memory.h>
#include <nvhe/mem_protect.h>
#include <nvhe/mm.h>
#include <nvhe/modules.h>

#define KVM_HOST_S2_FLAGS (KVM_PGTABLE_S2_NOFWB | KVM_PGTABLE_S2_IDMAP)

struct host_mmu host_mmu;

struct pkvm_moveable_reg pkvm_moveable_regs[PKVM_NR_MOVEABLE_REGS];
unsigned int pkvm_moveable_regs_nr;

static struct hyp_pool host_s2_pool;
static struct hyp_pool host_s2_mmio_pool;

/* Returns TRUE if a VM-owned page has a reference held for Hyp or DMA use. */
static bool hyp_page_referenced(void *addr)
{
	struct hyp_page *p = hyp_virt_to_page(addr);

	return hyp_refcount_get(p->refcount);
}

static int host_s2_pool_refill(struct kvm_hyp_memcache *host_mc)
{
	if (!host_s2_cma_size)
		return -EINVAL;

	return refill_hyp_pool(&host_s2_pool, host_mc);
}

static void host_s2_pool_reclaim(struct kvm_hyp_memcache *host_mc, int target)
{
	if (!host_s2_cma_size)
		return;

	reclaim_hyp_pool(&host_s2_pool, host_mc, target, false);
}

static int host_s2_pool_reclaimable(void)
{
	if (!host_s2_cma_size)
		return 0;

	return hyp_pool_reclaimable(&host_s2_pool, 0);
}

struct hyp_mgt_allocator_ops host_s2_pool_ops = {
	.refill		= host_s2_pool_refill,
	.reclaim	= host_s2_pool_reclaim,
	.reclaimable	= host_s2_pool_reclaimable,
};

void make_host_stage2_reclaimable(void)
{
	if (!host_s2_cma_size)
		return;

	__hyp_pool_set_range_reclaimable(&host_s2_pool);
}

static DEFINE_PER_CPU(struct pkvm_hyp_vm *, __current_vm);
#define current_vm (*this_cpu_ptr(&__current_vm))

static void pkvm_sme_dvmsync_fw_call(void)
{
	if (alternative_has_cap_unlikely(ARM64_WORKAROUND_4193714)) {
		struct arm_smccc_res res;

		/*
		 * Ignore the return value. Probing for the workaround
		 * availability took place in init_hyp_mode().
		 */
		arm_smccc_1_1_smc(ARM_SMCCC_CPU_WORKAROUND_4193714, &res);
	}
}

static struct kvm_pgtable_pte_ops host_s2_pte_ops;
static bool host_stage2_force_pte(u64 addr, u64 end, enum kvm_pgtable_prot prot);
static bool host_stage2_pte_is_counted(kvm_pte_t pte, u32 level);
static bool guest_stage2_pte_is_counted(kvm_pte_t pte, u32 level);

static struct kvm_pgtable_pte_ops guest_s2_pte_ops = {
	.pte_is_counted_cb = guest_stage2_pte_is_counted
};

static void guest_lock_component(struct pkvm_hyp_vm *vm)
{
	hyp_spin_lock(&vm->pgtable_lock);
	current_vm = vm;
}

static void guest_unlock_component(struct pkvm_hyp_vm *vm)
{
	current_vm = NULL;
	hyp_spin_unlock(&vm->pgtable_lock);
}

static void host_lock_component(void)
{
	hyp_spin_lock(&host_mmu.lock);
}

static void host_unlock_component(void)
{
	hyp_spin_unlock(&host_mmu.lock);
}

static void hyp_lock_component(void)
{
	hyp_spin_lock(&pkvm_pgd_lock);
}

static void hyp_unlock_component(void)
{
	hyp_spin_unlock(&pkvm_pgd_lock);
}

#define for_each_hyp_page(__p, __st, __sz)				\
	for (struct hyp_page *__p = hyp_phys_to_page(__st),		\
			     *__e = __p + ((__sz) >> PAGE_SHIFT);	\
	     __p < __e; __p++)

static void *host_s2_zalloc_pages_exact(size_t size)
{
	void *addr = hyp_alloc_pages(&host_s2_pool, get_order(size));

	hyp_split_page(hyp_virt_to_page(addr));

	/*
	 * The size of concatenated PGDs is always a power of two of PAGE_SIZE,
	 * so there should be no need to free any of the tail pages to make the
	 * allocation exact.
	 */
	WARN_ON(size != (PAGE_SIZE << get_order(size)));

	return addr;
}

static void *host_s2_zalloc_page(void *pool)
{
	return hyp_alloc_pages(pool, 0);
}

static struct hyp_pool *page_to_pool(void *addr)
{
	return hyp_pool_owned(&host_s2_pool, addr) ? &host_s2_pool : &host_s2_mmio_pool;
}

static struct hyp_pool *region_to_pool(bool is_memory)
{
	return is_memory ? &host_s2_pool : &host_s2_mmio_pool;
}

static void host_s2_get_page(void *addr)
{
	hyp_get_page(page_to_pool(addr), addr);
}

static void host_s2_put_page(void *addr)
{
	hyp_put_page(page_to_pool(addr), addr);
}

static void host_s2_free_unlinked_table(void *addr, s8 level)
{
	kvm_pgtable_stage2_free_unlinked(&host_mmu.mm_ops, host_mmu.pgt.pte_ops,
					 addr, level);
}

static int prepare_s2_pool(void *pgt_pool_base, void *mmio_pool_base)
{
	unsigned long nr_pages, pfn;
	int ret;

	pfn = hyp_virt_to_pfn(pgt_pool_base);
	nr_pages = host_s2_cma_size ? host_s2_cma_size >> PAGE_SHIFT : host_s2_pgtable_pages();
	ret = hyp_pool_init(&host_s2_pool, pfn, nr_pages, 0);
	if (ret)
		return ret;

	pfn = hyp_virt_to_pfn(mmio_pool_base);
	nr_pages = host_s2_mmio_pgtable_pages();
	ret = hyp_pool_init(&host_s2_mmio_pool, pfn, nr_pages, 0);
	if (ret)
		return ret;

	host_mmu.mm_ops = (struct kvm_pgtable_mm_ops) {
		.zalloc_pages_exact = host_s2_zalloc_pages_exact,
		.zalloc_page = host_s2_zalloc_page,
		.free_unlinked_table = host_s2_free_unlinked_table,
		.phys_to_virt = hyp_phys_to_virt,
		.virt_to_phys = hyp_virt_to_phys,
		.page_count = hyp_page_count,
		.get_page = host_s2_get_page,
		.put_page = host_s2_put_page,
	};

	return 0;
}

static void prepare_host_vtcr(void)
{
	u32 parange, phys_shift;

	/* The host stage 2 is id-mapped, so use parange for T0SZ */
	parange = kvm_get_parange(id_aa64mmfr0_el1_sys_val);
	phys_shift = id_aa64mmfr0_parange_to_phys_shift(parange);

	host_mmu.arch.mmu.vtcr = kvm_get_vtcr(id_aa64mmfr0_el1_sys_val,
					      id_aa64mmfr1_el1_sys_val, phys_shift);
}

static bool range_has_reclaimable_host_s2(u64 addr, u64 end)
{
	u64 cma_end = host_s2_cma_base + host_s2_cma_size;

	if (!host_s2_cma_size)
		return false;

	return addr < cma_end && end > host_s2_cma_base;
}

static int prepopulate_host_stage2(void)
{
	struct memblock_region *reg;
	int i, ret = 0;

	for (i = 0; i < hyp_memblock_nr; i++) {
		reg = &hyp_memory[i];
		u64 base, size;

		base = reg->base;
		size = reg->size;

		if (range_has_reclaimable_host_s2(base, base + size)) {
			ret = host_stage2_idmap_locked(reg->base, host_s2_cma_base - base,
						       PKVM_HOST_MEM_PROT, true);
			if (ret)
				return ret;

			ret = host_stage2_idmap_locked(host_s2_cma_base, host_s2_cma_size,
						       PKVM_HOST_MEM_PROT, true);
			if (ret)
				return ret;

			base = host_s2_cma_base + host_s2_cma_size;
			size = reg->base + reg->size - base;
		}

		ret = host_stage2_idmap_locked(base, size, PKVM_HOST_MEM_PROT, true);
		if (ret)
			return ret;
	}

	return ret;
}

int kvm_host_prepare_stage2(void *pgt_pool_base, void *mmio_pool_base)
{
	struct kvm_s2_mmu *mmu = &host_mmu.arch.mmu;
	int ret;

	prepare_host_vtcr();
	hyp_spin_lock_init(&host_mmu.lock);
	mmu->arch = &host_mmu.arch;

	ret = prepare_s2_pool(pgt_pool_base, mmio_pool_base);
	if (ret)
		return ret;

	host_s2_pte_ops.force_pte_cb = host_stage2_force_pte;
	host_s2_pte_ops.pte_is_counted_cb = host_stage2_pte_is_counted;

	ret = __kvm_pgtable_stage2_init(&host_mmu.pgt, mmu,
					&host_mmu.mm_ops, KVM_HOST_S2_FLAGS,
					&host_s2_pte_ops);
	if (ret)
		return ret;

	mmu->pgd_phys = __hyp_pa(host_mmu.pgt.pgd);
	mmu->pgt = &host_mmu.pgt;
	atomic64_set(&mmu->vmid.id, 0);

	return prepopulate_host_stage2();
}

enum host_set_page_state_flags {
	HOST_SET_IS_MMIO                = BIT(0),
	HOST_SET_NO_IOMMU_UPDATE        = BIT(1),
	HOST_SET_NO_COMPLETE            = BIT(2), /* Skip __host_stage2_set_owner_complete() */
	HOST_SET_PSCI_MEM_PROTECT	= BIT(3),
};

static int __host_stage2_set_owner_locked(phys_addr_t addr, u64 size, u8 owner_id,
					  enum pkvm_page_state nopage_state,
					  enum host_set_page_state_flags flags);


static bool guest_stage2_pte_is_counted(kvm_pte_t pte, u32 level)
{
	/*
	 * The refcount tracks valid entries as well as invalid entries if they
	 * encode ownership of a page to another entity than the page-table
	 * owner, whose id is 0.
	 */
	return !!pte;
}

static void *guest_s2_zalloc_pages_exact(size_t size)
{
	void *addr = hyp_alloc_pages(&current_vm->pool, get_order(size));

	WARN_ON(!addr || size != (PAGE_SIZE << get_order(size)));
	hyp_split_page(hyp_virt_to_page(addr));

	return addr;
}

static void guest_s2_free_pages_exact(void *addr, unsigned long size)
{
	u8 order = get_order(size);
	unsigned int i;

	for (i = 0; i < (1 << order); i++)
		hyp_put_page(&current_vm->pool, addr + (i * PAGE_SIZE));
}

static void *guest_s2_zalloc_page(void *mc)
{
	struct hyp_page *p;
	void *addr;
	unsigned long order;

	addr = hyp_alloc_pages(&current_vm->pool, 0);
	if (addr)
		return addr;

	addr = pop_hyp_memcache(mc, hyp_phys_to_virt, &order);
	if (!addr)
		return addr;

	WARN_ON(order);
	memset(addr, 0, PAGE_SIZE);
	p = hyp_virt_to_page(addr);
	hyp_set_page_refcounted(p);
	p->order = 0;

	return addr;
}

static void guest_s2_get_page(void *addr)
{
	hyp_get_page(&current_vm->pool, addr);
}

static void guest_s2_put_page(void *addr)
{
	hyp_put_page(&current_vm->pool, addr);
}

static void guest_s2_free_unlinked_table(void *addr, s8 level)
{
	/* We are trying to collapse a table into a block mapping. This is forbidden. */
	WARN_ON(1);
}

static void __apply_guest_page(void *va, size_t size,
			       void (*func)(void *addr, size_t size))
{
	size += va - PTR_ALIGN_DOWN(va, PAGE_SIZE);
	va = PTR_ALIGN_DOWN(va, PAGE_SIZE);
	size = PAGE_ALIGN(size);

	while (size) {
		size_t map_size = PAGE_SIZE;
		void *map;

		if (IS_ALIGNED((unsigned long)va, PMD_SIZE) && size >= PMD_SIZE)
			map = hyp_fixblock_map(__hyp_pa(va), &map_size);
		else
			map = hyp_fixmap_map(__hyp_pa(va));

		func(map, map_size);

		if (map_size == PMD_SIZE)
			hyp_fixblock_unmap();
		else
			hyp_fixmap_unmap();

		size -= map_size;
		va += map_size;
	}
}

static void clean_dcache_guest_page(void *va, size_t size)
{
	__apply_guest_page(va, size, __clean_dcache_guest_page);
}

static void invalidate_icache_guest_page(void *va, size_t size)
{
	__apply_guest_page(va, size, __invalidate_icache_guest_page);
}

static void __hyp_flush_page(void *addr, size_t size)
{
	/*
	 * Prefer kvm_flush_dcache_to_poc() over __clean_dcache_guest_page()
	 * here as the latter may elide the CMO under the assumption that FWB
	 * will be enabled on CPUs that support it. This is incorrect for the
	 * host stage-2 and would otherwise lead to a malicious host potentially
	 * being able to read the contents of newly reclaimed guest pages.
	 */
	kvm_flush_dcache_to_poc(addr, size);
}

static void hyp_flush_page(phys_addr_t phys, size_t size)
{
	__apply_guest_page(__hyp_va(phys), size, __hyp_flush_page);
}

int kvm_guest_prepare_stage2(struct pkvm_hyp_vm *vm, void *pgd,
			     enum kvm_pgtable_stage2_flags flags)
{
	struct kvm_s2_mmu *mmu = &vm->kvm.arch.mmu;
	unsigned long nr_pages;
	int ret;

	nr_pages = kvm_pgtable_stage2_pgd_size(mmu->vtcr) >> PAGE_SHIFT;
	ret = hyp_pool_init(&vm->pool, hyp_virt_to_pfn(pgd), nr_pages, 0);
	if (ret)
		return ret;

	hyp_spin_lock_init(&vm->pgtable_lock);
	vm->mm_ops = (struct kvm_pgtable_mm_ops) {
		.zalloc_pages_exact	= guest_s2_zalloc_pages_exact,
		.free_pages_exact	= guest_s2_free_pages_exact,
		.zalloc_page		= guest_s2_zalloc_page,
		.free_unlinked_table	= guest_s2_free_unlinked_table,
		.phys_to_virt		= hyp_phys_to_virt,
		.virt_to_phys		= hyp_virt_to_phys,
		.page_count		= hyp_page_count,
		.get_page		= guest_s2_get_page,
		.put_page		= guest_s2_put_page,
		.dcache_clean_inval_poc	= clean_dcache_guest_page,
		.icache_inval_pou	= invalidate_icache_guest_page,
	};

	guest_lock_component(vm);
	ret = __kvm_pgtable_stage2_init(mmu->pgt, mmu, &vm->mm_ops, flags,
					&guest_s2_pte_ops);
	guest_unlock_component(vm);
	if (ret)
		return ret;

	vm->kvm.arch.mmu.pgd_phys = __hyp_pa(vm->pgt.pgd);

	return 0;
}

void destroy_hyp_vm_pgt(struct pkvm_hyp_vm *vm)
{
	guest_lock_component(vm);
	kvm_pgtable_stage2_destroy(&vm->pgt);
	vm->kvm.arch.mmu.pgd_phys = 0ULL;
	guest_unlock_component(vm);
}

void drain_hyp_pool(struct hyp_pool *pool, struct kvm_hyp_memcache *mc)
{
	WARN_ON(reclaim_hyp_pool(pool, mc, INT_MAX, true) != -ENOMEM);
}

static int ___pkvm_guest_relinquish_to_module(struct pkvm_hyp_vcpu *vcpu, u64 ipa, u64 phys,
					      kvm_pte_t pte)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	struct hyp_page *page;

	if (pkvm_getstate(kvm_pgtable_stage2_pte_prot(pte)) != PKVM_PAGE_SHARED_BORROWED)
		return -EPERM;

	page = hyp_phys_to_page(phys);
	if (get_host_state(page) != PKVM_MODULE_SHARED_OWNED_PAGE)
		return -EPERM;

	/*
	 * We're guaranteed by the caller to be operating on existing last-level entries, so no
	 * risk of getting -ENOMEM here.
	 */
	WARN_ON(kvm_pgtable_stage2_annotate(&vm->pgt, ipa, PAGE_SIZE, &vcpu->vcpu.arch.stage2_mc,
					    KVM_ACCEPT_MODULE_PROT_NOTE));
	set_host_state(page, PKVM_MODULE_OWNED_PAGE);

	return 0;
}

static enum pkvm_page_state guest_get_page_state(kvm_pte_t pte, u64 addr);

int __pkvm_guest_relinquish_to_host(struct pkvm_hyp_vcpu *vcpu,
				    u64 ipa, u64 flags, u64 *ppa)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	enum pkvm_page_state state;
	u64 phys = 0, addr;
	kvm_pte_t pte;
	s8 level;
	int ret;

	if (!pkvm_hyp_vcpu_is_protected(vcpu))
		return 0;

	if (ipa & ~PAGE_MASK)
		return -EINVAL;

	host_lock_component();
	guest_lock_component(vm);

	ret = kvm_pgtable_get_leaf(&vm->pgt, ipa, &pte, &level);
	if (ret || !kvm_pte_valid(pte))
		goto end;

	/* We don't support splitting non-leaf mappings */
	if (level != KVM_PGTABLE_LAST_LEVEL) {
		ret = -E2BIG;
		goto end;
	}

	addr = ALIGN_DOWN(ipa, kvm_granule_size(level));
	phys = kvm_pte_to_phys(pte);
	phys += ipa - addr;
	if (!addr_is_memory(phys)) {
		ret = -EPERM;
		goto end;
	}
	/* page might be used for DMA! */
	if (hyp_page_referenced(hyp_phys_to_virt(phys))) {
		ret = -EBUSY;
		goto end;
	}

	state = guest_get_page_state(pte, addr);
	if (state != PKVM_PAGE_OWNED) {
		ret = ___pkvm_guest_relinquish_to_module(vcpu, ipa, phys, pte);
		goto end;
	}

	/* Zap the guest stage2 pte and return ownership to the host */
	WARN_ON(kvm_pgtable_stage2_unmap(&vm->pgt, ipa, PAGE_SIZE));

	if (!(flags & KVM_FUNC_MEM_RELINQUISH_NO_POISON))
		hyp_poison_page(phys, PAGE_SIZE);
	else
		hyp_flush_page(phys, PAGE_SIZE);

	ret = __host_stage2_set_owner_locked(phys, PAGE_SIZE, PKVM_ID_HOST, 0,
					     HOST_SET_PSCI_MEM_PROTECT);
	if (ret) {
		WARN_ON(kvm_pgtable_stage2_map(&vm->pgt, ipa, PAGE_SIZE, phys,
					       pkvm_mkstate(KVM_PGTABLE_PROT_RWX, PKVM_PAGE_OWNED),
					       &vcpu->vcpu.arch.stage2_mc, 0));
		goto end;
	}

	if (pkvm_ipa_range_has_pvmfw(vm, ipa, ipa + PAGE_SIZE))
		vm->kvm.arch.pkvm.pvmfw_load_addr = PVMFW_INVALID_LOAD_ADDR;
end:
	guest_unlock_component(vm);
	host_unlock_component();

	*ppa = phys;

	return ret;
}

static void pkvm_init_params_finalize(struct kvm_nvhe_init_params *params)
{
	struct kvm_s2_mmu *mmu = &host_mmu.arch.mmu;

	params->vttbr = kvm_get_vttbr(mmu);
	params->vtcr = mmu->vtcr;
	params->hcr_el2 |= HCR_VM;

	/*
	 * The CMO below not only cleans the updated params to the
	 * PoC, but also provides the DSB that ensures ongoing
	 * page-table walks that have started before we trapped to EL2
	 * have completed.
	 */
	kvm_flush_dcache_to_poc(params, sizeof(*params));
}

void __pkvm_late_cpus_finalize(void)
{
	struct kvm_nvhe_init_params *params;
	unsigned long i;

	for (i = 0; i < hyp_nr_cpus; i++) {
		if (!test_bit(KVM_HOST_DATA_FLAG_PKVM_LATE_CPU,
			      &per_cpu_ptr(&kvm_host_data, i)->flags))
			continue;

		params = per_cpu_ptr(&kvm_init_params, i);
		pkvm_init_params_finalize(params);

		/* For CPUs we've not seen before, assume the worst */
		if (!*per_cpu_ptr(&kvm_hyp_vector, i))
			__pkvm_cpu_set_vector(HYP_VECTOR_INDIRECT, i);
	}

	__pkvm_close_module_registration();
}

int __pkvm_prot_finalize(void)
{
	struct kvm_nvhe_init_params *params = this_cpu_ptr(&kvm_init_params);

	if (read_sysreg(HCR_EL2) & HCR_VM)
		return -EINVAL;

	pkvm_init_params_finalize(params);

	write_sysreg_hcr(params->hcr_el2);
	__load_stage2(&host_mmu.arch.mmu, &host_mmu.arch);

	/*
	 * Make sure to have an ISB before the TLB maintenance below but only
	 * when __load_stage2() doesn't include one already.
	 */
	asm(ALTERNATIVE("isb", "nop", ARM64_WORKAROUND_SPECULATIVE_AT));

	/* Invalidate stale HCR bits that may be cached in TLBs */
	__tlbi(vmalls12e1);
	dsb(nsh);
	isb();

	return 0;
}

int host_stage2_unmap_reg_locked(phys_addr_t start, u64 size)
{
	hyp_assert_lock_held(&host_mmu.lock);

	return kvm_pgtable_stage2_reclaim_leaves(&host_mmu.pgt, start, size);
}

static int host_stage2_unmap_unmoveable_regs(void)
{
	struct kvm_pgtable *pgt = &host_mmu.pgt;
	struct pkvm_moveable_reg *reg;
	u64 addr = 0;
	int i, ret;

	/* Unmap all unmoveable regions to recycle the pages */
	for (i = 0; i < pkvm_moveable_regs_nr; i++) {
		reg = &pkvm_moveable_regs[i];
		if (reg->start > addr) {
			ret = host_stage2_unmap_reg_locked(addr, reg->start - addr);
			if (ret)
				return ret;
		}
		addr = max(addr, reg->start + reg->size);
	}
	return host_stage2_unmap_reg_locked(addr, BIT(pgt->ia_bits) - addr);
}

/*
 * Ensure the PFN range is contained within PA-range.
 *
 * This check is also robust to overflows and is therefore a requirement before
 * using a pfn/nr_pages pair from an untrusted source.
 */
static bool pfn_range_is_valid(u64 pfn, u64 nr_pages)
{
	u64 limit = BIT(kvm_phys_shift(&host_mmu.arch.mmu) - PAGE_SHIFT);

	return pfn < limit && ((limit - pfn) >= nr_pages);
}

struct kvm_mem_range {
	u64 start;
	u64 end;
};

static struct memblock_region *find_mem_range(phys_addr_t addr, struct kvm_mem_range *range)
{
	int cur, left = 0, right = hyp_memblock_nr;
	struct memblock_region *reg;
	phys_addr_t end;

	range->start = 0;
	range->end = ULONG_MAX;

	/* The list of memblock regions is sorted, binary search it */
	while (left < right) {
		cur = (left + right) >> 1;
		reg = &hyp_memory[cur];
		end = reg->base + reg->size;
		if (addr < reg->base) {
			right = cur;
			range->end = reg->base;
		} else if (addr >= end) {
			left = cur + 1;
			range->start = end;
		} else {
			range->start = reg->base;
			range->end = end;
			return reg;
		}
	}

	return NULL;
}

static enum kvm_pgtable_prot default_host_prot(bool is_memory)
{
	return is_memory ? PKVM_HOST_MEM_PROT : PKVM_HOST_MMIO_PROT;
}

static enum kvm_pgtable_prot default_hyp_prot(phys_addr_t phys)
{
	return addr_is_memory(phys) ? PAGE_HYP : PAGE_HYP_DEVICE;
}

bool addr_is_memory(phys_addr_t phys)
{
	struct kvm_mem_range range;

	return !!find_mem_range(phys, &range);
}

static bool is_in_mem_range(u64 addr, struct kvm_mem_range *range)
{
	return range->start <= addr && addr < range->end;
}

static int check_range_allowed_memory(u64 start, u64 end)
{
	struct memblock_region *reg;
	struct kvm_mem_range range;

	/*
	 * Callers can't check the state of a range that overlaps memory and
	 * MMIO regions, so ensure [start, end[ is in the same kvm_mem_range.
	 */
	reg = find_mem_range(start, &range);
	if (!is_in_mem_range(end - 1, &range))
		return -EINVAL;

	if (!reg || reg->flags & MEMBLOCK_NOMAP)
		return -EPERM;

	return 0;
}

static bool range_is_memory(u64 start, u64 end)
{
	struct kvm_mem_range r;

	if (!find_mem_range(start, &r))
		return false;

	return is_in_mem_range(end - 1, &r);
}

/*
 * The pool has been provided with enough pages to cover all of moveable regions
 * with page granularity, but it is difficult to know how much of the
 * non-moveable regions we will need to cover upfront, so we may need to
 * 'recycle' the pages if we run out.
 */
#define host_stage2_try(is_memory, fn, ...)				\
	({								\
		int __ret;						\
		hyp_assert_lock_held(&host_mmu.lock);			\
		__ret = fn(__VA_ARGS__);				\
		if (!(is_memory) && __ret == -ENOMEM) {			\
			__ret = host_stage2_unmap_unmoveable_regs();	\
			if (!__ret)					\
				__ret = fn(__VA_ARGS__);		\
		}							\
		__ret;							\
	 })

static inline bool range_included(struct kvm_mem_range *child,
				  struct kvm_mem_range *parent)
{
	return parent->start <= child->start && child->end <= parent->end;
}

static int host_stage2_adjust_range(u64 addr, struct kvm_mem_range *range)
{
	struct kvm_mem_range cur;
	kvm_pte_t pte;
	u64 granule;
	s8 level;
	int ret;

	hyp_assert_lock_held(&host_mmu.lock);
	ret = kvm_pgtable_get_leaf(&host_mmu.pgt, addr, &pte, &level);
	if (ret)
		return ret;

	if (kvm_pte_valid(pte))
		return -EAGAIN;

	if (pte)
		return -EPERM;

	for (; level <= KVM_PGTABLE_LAST_LEVEL; level++) {
		if (!kvm_level_supports_block_mapping(level))
			continue;
		granule = kvm_granule_size(level);
		cur.start = ALIGN_DOWN(addr, granule);
		cur.end = cur.start + granule;
		if (!range_included(&cur, range) && level < KVM_PGTABLE_LAST_LEVEL)
			continue;
		*range = cur;
		return 0;
	}

	WARN_ON(1);

	return -EINVAL;
}

static int __host_stage2_idmap(phys_addr_t addr, u64 size, enum kvm_pgtable_prot prot,
			       bool is_memory)
{
	struct kvm_pgtable *pgt = &host_mmu.pgt;
	void *mc = region_to_pool(is_memory);
	int ret;

	ret = host_stage2_try(is_memory, kvm_pgtable_stage2_map, pgt, addr,
			      size, addr, prot, mc, 0);
	if (is_memory && ret == -ENOMEM)
		ret = -ENOMEMHOSTS2;

	return ret;
}

static void __host_stage2_idmap_complete(enum kvm_pgtable_prot prot)
{
	if ((prot & KVM_PGTABLE_PROT_RW) != KVM_PGTABLE_PROT_RW)
		pkvm_sme_dvmsync_fw_call();
}

int host_stage2_idmap_locked(phys_addr_t addr, u64 size, enum kvm_pgtable_prot prot, bool is_memory)
{
	int ret = __host_stage2_idmap(addr, size, prot, is_memory);

	if (ret)
		return ret;

	__host_stage2_idmap_complete(prot);

	return 0;
}

static void __host_update_page_state(phys_addr_t addr, u64 size, enum pkvm_page_state state)
{
	for_each_hyp_page(page, addr, size)
		set_host_state(page, state);
}

#define KVM_MAX_OWNER_ID		PKVM_ID_MAX

static kvm_pte_t kvm_init_invalid_leaf_owner(u8 owner_id)
{
	return FIELD_PREP(KVM_INVALID_PTE_OWNER_MASK, owner_id);
}

static void __host_stage2_set_owner_complete(u8 owner_id, enum host_set_page_state_flags flags)
{
	bool is_memory = !(flags & HOST_SET_IS_MMIO);
	bool map = owner_id == PKVM_ID_HOST;

	hyp_assert_lock_held(&host_mmu.lock);

	__host_stage2_idmap_complete(map ? default_host_prot(is_memory) : 0);

	if (flags & HOST_SET_NO_IOMMU_UPDATE)
		return;

	kvm_iommu_host_stage2_idmap_complete(map);
}

static int __host_stage2_set_owner_locked(phys_addr_t addr, u64 size, u8 owner_id,
					  enum pkvm_page_state nopage_state,
					  enum host_set_page_state_flags flags)
{
	bool is_memory = !(flags & HOST_SET_IS_MMIO);
	kvm_pte_t annotation;
	enum kvm_pgtable_prot prot;
	int ret;

	if (owner_id > KVM_MAX_OWNER_ID)
		return -EINVAL;

	if (owner_id == PKVM_ID_HOST) {
		prot = default_host_prot(is_memory);
		ret = __host_stage2_idmap(addr, size, prot, is_memory);
	} else {
		annotation = kvm_init_invalid_leaf_owner(owner_id);
		ret = host_stage2_try(is_memory, kvm_pgtable_stage2_annotate,
				      &host_mmu.pgt, addr, size,
				      region_to_pool(is_memory), annotation);
		if (is_memory && ret == -ENOMEM)
			ret = -ENOMEMHOSTS2;
	}

	if (ret) {
		WARN_ON(ret != -ENOMEMHOSTS2);
		return ret;
	}

	if (flags & HOST_SET_NO_IOMMU_UPDATE)
		goto psci_mem_protect;

	prot = owner_id == PKVM_ID_HOST ? PKVM_HOST_MEM_PROT : 0;
	WARN_ON(kvm_iommu_host_stage2_idmap(addr, addr + size, prot));

psci_mem_protect:
	if (flags & HOST_SET_PSCI_MEM_PROTECT) {
		if (owner_id == PKVM_ID_HOST)
			psci_mem_protect_dec(size >> PAGE_SHIFT);
		else
			psci_mem_protect_inc(size >> PAGE_SHIFT);
	}

	if (flags & HOST_SET_NO_COMPLETE)
		goto update_vmemmap;

	__host_stage2_set_owner_complete(owner_id, flags);

update_vmemmap:
	if (!is_memory)
		return 0;

	/* Don't forget to update the vmemmap tracking for the host */
	if (owner_id == PKVM_ID_HOST)
		__host_update_page_state(addr, size, PKVM_PAGE_OWNED);
	else
		__host_update_page_state(addr, size, PKVM_NOPAGE | nopage_state);

	return 0;
}

int host_stage2_set_owner_locked(phys_addr_t addr, u64 size, u8 owner_id)
{
	return __host_stage2_set_owner_locked(addr, size, owner_id, 0,
					      addr_is_memory(addr) ? 0 : HOST_SET_IS_MMIO);
}

static bool host_stage2_force_pte(u64 addr, u64 end, enum kvm_pgtable_prot prot)
{
	if (range_has_reclaimable_host_s2(addr, end))
		return true;

	/*
	 * Block mappings must be used with care in the host stage-2 as a
	 * kvm_pgtable_stage2_map() operation targeting a page in the range of
	 * an existing block will delete the block under the assumption that
	 * mappings in the rest of the block range can always be rebuilt lazily.
	 * That assumption is correct for the host stage-2 with RWX mappings
	 * targeting memory or RW mappings targeting MMIO ranges (see
	 * host_stage2_idmap() below which implements some of the host memory
	 * abort logic). However, this is not safe for any other mappings where
	 * the host stage-2 page-table is in fact the only place where this
	 * state is stored. In all those cases, it is safer to use page-level
	 * mappings, hence avoiding to lose the state because of side-effects in
	 * kvm_pgtable_stage2_map().
	 */
	return prot != default_host_prot(range_is_memory(addr, end));
}

static bool host_stage2_pte_is_counted(kvm_pte_t pte, u32 level)
{
	u64 phys;

	if (!kvm_pte_valid(pte))
		return !!pte;

	if (kvm_pte_table(pte, level))
		return true;

	phys = kvm_pte_to_phys(pte);
	if (addr_is_memory(phys))
		return (pte & KVM_HOST_S2_DEFAULT_MASK) !=
			KVM_HOST_S2_DEFAULT_MEM_PTE;

	return (pte & KVM_HOST_S2_DEFAULT_MASK) != KVM_HOST_S2_DEFAULT_MMIO_PTE;
}

static int host_stage2_fault(u64 addr)
{
	struct kvm_mem_range range;
	bool is_memory = !!find_mem_range(addr, &range);
	enum kvm_pgtable_prot prot = default_host_prot(is_memory);
	int ret;

	host_lock_component();
	ret = host_stage2_adjust_range(addr, &range);
	if (ret)
		goto unlock;

	/* Should never happen: host stage-2 memory region is prefaulted */
	WARN_ON(is_memory);

	ret = host_stage2_idmap_locked(range.start, range.end - range.start, prot, false);
unlock:
	host_unlock_component();

	return ret;
}

static void (*illegal_abt_notifier)(struct user_pt_regs *regs);

int __pkvm_register_illegal_abt_notifier(void (*cb)(struct user_pt_regs *))
{
	return cmpxchg(&illegal_abt_notifier, NULL, cb) ? -EBUSY : 0;
}

static void host_inject_abort(struct kvm_cpu_context *host_ctxt)
{
	u64 spsr = read_sysreg_el2(SYS_SPSR);
	u64 esr = read_sysreg_el2(SYS_ESR);
	u64 ventry, ec;

	if (READ_ONCE(illegal_abt_notifier))
		illegal_abt_notifier(&host_ctxt->regs);

	/* Repaint the ESR to report a same-level fault if taken from EL1 */
	if ((spsr & PSR_MODE_MASK) != PSR_MODE_EL0t) {
		ec = ESR_ELx_EC(esr);
		if (ec == ESR_ELx_EC_DABT_LOW)
			ec = ESR_ELx_EC_DABT_CUR;
		else if (ec == ESR_ELx_EC_IABT_LOW)
			ec = ESR_ELx_EC_IABT_CUR;
		else
			WARN_ON(1);
		esr &= ~ESR_ELx_EC_MASK;
		esr |= ec << ESR_ELx_EC_SHIFT;
	}

	/*
	 * Since S1PTW should only ever be set for stage-2 faults, we're pretty
	 * much guaranteed that it won't be set in ESR_EL1 by the hardware. So,
	 * let's use that bit to allow the host abort handler to differentiate
	 * this abort from normal userspace faults.
	 *
	 * Note: although S1PTW is RES0 at EL1, it is guaranteed by the
	 * architecture to be backed by flops, so it should be safe to use.
	 */
	esr |= ESR_ELx_S1PTW;

	write_sysreg_el1(esr, SYS_ESR);
	write_sysreg_el1(spsr, SYS_SPSR);
	write_sysreg_el1(read_sysreg_el2(SYS_ELR), SYS_ELR);
	write_sysreg_el1(read_sysreg_el2(SYS_FAR), SYS_FAR);

	ventry = read_sysreg_el1(SYS_VBAR);
	ventry += get_except64_offset(spsr, PSR_MODE_EL1h, except_type_sync);
	write_sysreg_el2(ventry, SYS_ELR);

	spsr = get_except64_cpsr(spsr, system_supports_mte(),
				 read_sysreg_el1(SYS_SCTLR), PSR_MODE_EL1h);
	write_sysreg_el2(spsr, SYS_SPSR);
}


static bool is_dabt(u64 esr)
{
	return ESR_ELx_EC(esr) == ESR_ELx_EC_DABT_LOW;
}

void handle_host_mem_abort(struct kvm_cpu_context *host_ctxt)
{
	struct kvm_vcpu_fault_info fault;
	u64 esr, addr;
	int ret = 0;

	esr = read_sysreg_el2(SYS_ESR);
	if (!__get_fault_info(esr, &fault)) {
		/* Setting the address to an invalid value for use in tracing. */
		addr = (u64)-1;
		/*
		 * We've presumably raced with a page-table change which caused
		 * AT to fail, try again.
		 */
		return;
	}


	/*
	 * Yikes, we couldn't resolve the fault IPA. This should reinject an
	 * abort into the host when we figure out how to do that.
	 */
	BUG_ON(!(fault.hpfar_el2 & HPFAR_EL2_NS));
	addr = FIELD_GET(HPFAR_EL2_FIPA, fault.hpfar_el2) << 12;
	addr |= fault.far_el2 & FAR_MASK;

	if (is_dabt(esr) && !addr_is_memory(addr) &&
	    kvm_iommu_host_dabt_handler(&host_ctxt->regs, esr, addr))
		return;


	switch (esr & ESR_ELx_FSC_TYPE) {
	case ESR_ELx_FSC_FAULT:
		ret = host_stage2_fault(addr);
		break;
	case ESR_ELx_FSC_PERM:
		ret = module_handle_host_perm_fault(&host_ctxt->regs, esr, addr);
		ret = ret ? 0 /* handled */ : -EPERM;
		break;
	default:
		ret = -EPERM;
		break;
	}

	if (ret == -EPERM)
		host_inject_abort(host_ctxt);
	else
		BUG_ON(ret && ret != -EAGAIN);

	trace_host_mem_abort(esr, addr);
}

struct check_walk_data {
	enum pkvm_page_state	desired;
	enum pkvm_page_state	(*get_page_state)(kvm_pte_t pte, u64 addr);
};

static int __check_page_state_visitor(const struct kvm_pgtable_visit_ctx *ctx,
				      enum kvm_pgtable_walk_flags visit)
{
	struct check_walk_data *d = ctx->arg;

	return d->get_page_state(ctx->old, ctx->addr) == d->desired ? 0 : -EPERM;
}

static int check_page_state_range(struct kvm_pgtable *pgt, u64 addr, u64 size,
				  struct check_walk_data *data)
{
	struct kvm_pgtable_walker walker = {
		.cb	= __check_page_state_visitor,
		.arg	= data,
		.flags	= KVM_PGTABLE_WALK_LEAF,
	};

	return kvm_pgtable_walk(pgt, addr, size, &walker);
}

static enum pkvm_page_state host_get_mmio_page_state(kvm_pte_t pte, u64 addr)
{
	enum pkvm_page_state state = 0;
	enum kvm_pgtable_prot prot;

	WARN_ON(addr_is_memory(addr));

	if (!kvm_pte_valid(pte) && pte)
		return PKVM_NOPAGE;

	prot = kvm_pgtable_stage2_pte_prot(pte);
	if (kvm_pte_valid(pte)) {
		if ((prot & KVM_PGTABLE_PROT_RWX) != PKVM_HOST_MMIO_PROT)
			state = PKVM_PAGE_RESTRICTED_PROT;
	}

	return state | pkvm_getstate(prot);
}

enum host_check_page_state_flags {
	HOST_CHECK_NULL_REFCNT		= BIT(0),
	HOST_CHECK_IS_MEMORY		= BIT(1),
	HOST_CHECK_ALLOW_NO_MAP		= BIT(2),
};

static int ___host_check_page_state_range(u64 addr, u64 size,
					  enum pkvm_page_state state,
					  enum host_check_page_state_flags flags)
{
	struct check_walk_data d = {
		.desired	= state,
		.get_page_state	= host_get_mmio_page_state,
	};
	struct memblock_region *reg;
	struct kvm_mem_range range;
	u64 end;

	if (check_add_overflow(addr, size, &end))
		return -EINVAL;

	/* Can't check the state of both MMIO and memory regions at once */
	reg = find_mem_range(addr, &range);
	if (!reg && (flags & HOST_CHECK_IS_MEMORY))
		return -EINVAL;

	if (!is_in_mem_range(end - 1, &range))
		return -EINVAL;

	hyp_assert_lock_held(&host_mmu.lock);

	/* MMIO state is still in the page-table */
	if (!reg)
		return check_page_state_range(&host_mmu.pgt, addr, size, &d);

	if (reg->flags & MEMBLOCK_NOMAP && !(flags & HOST_CHECK_ALLOW_NO_MAP))
		return -EPERM;

	for_each_hyp_page(page, addr, size) {
		if (get_host_state(page) != state)
			return -EPERM;
		if ((flags & HOST_CHECK_NULL_REFCNT) && hyp_refcount_get(page->refcount))
			return -EINVAL;
	}

	/*
	 * All memory pages with restricted permissions will already be covered
	 * by other states (e.g. PKVM_MODULE_OWNED_PAGE), so no need to retrieve
	 * the PKVM_PAGE_RESTRICTED_PROT state from the PTE.
	 */
	return 0;
}

static int __host_check_page_state_range(u64 addr, u64 size,
					 enum pkvm_page_state state)
{
	enum host_check_page_state_flags flags = HOST_CHECK_IS_MEMORY;

	if (state == PKVM_PAGE_OWNED)
		flags |= HOST_CHECK_NULL_REFCNT;

	/* Check the refcount of PAGE_OWNED pages as those may be used for DMA. */
	return ___host_check_page_state_range(addr, size, state, flags);
}


static void __hyp_set_page_state_range(phys_addr_t phys, u64 size, enum pkvm_page_state state)
{
	for_each_hyp_page(page, phys, size)
		set_hyp_state(page, state);
}

static enum pkvm_page_state hyp_get_page_state_mmio(kvm_pte_t pte, u64 addr)
{
	enum pkvm_page_state state = 0;
	enum kvm_pgtable_prot prot;

	if (!kvm_pte_valid(pte))
		return PKVM_NOPAGE;
	prot = kvm_pgtable_hyp_pte_prot(pte);
	if (kvm_pte_valid(pte) && ((prot & KVM_PGTABLE_PROT_RWX) != PAGE_HYP)) {
		state = PKVM_PAGE_RESTRICTED_PROT;
	}
	return state | pkvm_getstate(prot);
}

static int __hyp_check_page_state_range(phys_addr_t phys, u64 size,
					enum pkvm_page_state state)
{
	if (!range_is_memory(phys, phys + size)) {
		struct check_walk_data d = {
			.desired	= state,
			.get_page_state	= hyp_get_page_state_mmio,
		};

		hyp_assert_lock_held(&pkvm_pgd_lock);
		return check_page_state_range(&pkvm_pgtable, (u64)hyp_phys_to_virt(phys), size, &d);
	}

	for_each_hyp_page(page, phys, size) {
		if (get_hyp_state(page) != state)
			return -EPERM;
	}

	return 0;
}

int hyp_check_range_owned(u64 phys_addr, u64 size)
{
	int ret;

	hyp_lock_component();
	ret = __hyp_check_page_state_range(phys_addr, size, PKVM_PAGE_OWNED);
	hyp_unlock_component();

	return ret;
}

static enum pkvm_page_state guest_get_page_state(kvm_pte_t pte, u64 addr)
{
	enum pkvm_page_state state = 0;
	enum kvm_pgtable_prot prot;

	if (!kvm_pte_valid(pte)) {
		state = PKVM_NOPAGE;

		if (pte == KVM_INVALID_PTE_MMIO_NOTE)
			state |= PKVM_MMIO;
		else if (pte == KVM_ACCEPT_MODULE_PROT_NOTE)
			state |= PKVM_ACCEPT_MODULE_OWNED;

		return state;
	}

	prot = kvm_pgtable_stage2_pte_prot(pte);
	if (kvm_pte_valid(pte) && ((prot & KVM_PGTABLE_PROT_RWX) != KVM_PGTABLE_PROT_RWX))
		state = PKVM_PAGE_RESTRICTED_PROT;

	return state | pkvm_getstate(prot);
}

static int __guest_check_page_state_range(struct pkvm_hyp_vm *vm, u64 addr,
					  u64 size, enum pkvm_page_state state)
{
	struct check_walk_data d = {
		.desired	= state,
		.get_page_state	= guest_get_page_state,
	};
	u64 end;

	if (check_add_overflow(addr, size, &end))
		return -EINVAL;

	hyp_assert_lock_held(&vm->pgtable_lock);
	return check_page_state_range(&vm->pgt, addr, size, &d);
}

struct guest_request_walker_data {
	union {
		kvm_pte_t		pte_start; /* guest_request_walker() */
		unsigned long		ipa_start; /* guest_request_ioguard_walker() */
	};
	u64			size;
	enum pkvm_page_state	desired_state;
	int			max_ptes;
};

#define GUEST_WALKER_DATA_INIT(__state)							\
{											\
	.size		= 0,								\
	.desired_state	= __state,							\
	/*										\
	 * In the very unlucky case where we have:					\
	 *   1. A block-aligned start address						\
	 *   2. An existing table							\
	 *   3. Contiguous phys for the entire table					\
	 *										\
	 * The guest stage-2 mapping of that range would try to collapse the existing	\
	 * table into a block mapping. We do not want this to happen: the		\
	 * stage-2 geometry must remain synchronized with the host's			\
	 * kvm_pinned_page tree at all time.						\
	 *										\
	 * As a mitigation, limit the number of processed PTEs to half the size		\
	 * of a table on a 4K page-size system.						\
	 */										\
	.max_ptes	= 256,								\
}

static int guest_request_walker(const struct kvm_pgtable_visit_ctx *ctx,
				enum kvm_pgtable_walk_flags visit)
{
	struct guest_request_walker_data *data = (struct guest_request_walker_data *)ctx->arg;
	enum pkvm_page_state state;
	kvm_pte_t pte = *ctx->ptep;
	phys_addr_t phys;
	u64 granule_size;

	state = guest_get_page_state(pte, 0);
	if (data->desired_state != state)
		return (state == PKVM_NOPAGE) ? -ENOENT : -EPERM;

	/* state != PKVM_NOPAGE but invalid PTE? */
	if (WARN_ON(!kvm_pte_valid(pte)))
		return -EINVAL;

	granule_size = kvm_granule_size(ctx->level);
	phys = kvm_pte_to_phys(pte);

	/* First PTE */
	if (!data->size) {
		/* Request starts in the middle of a huge-mapping */
		if (!IS_ALIGNED(ctx->start, granule_size))
			return -E2BIG;

		data->pte_start = pte;
		data->size = granule_size;

		goto end;
	}

	if (kvm_pgtable_stage2_pte_prot(pte) !=
	    kvm_pgtable_stage2_pte_prot(data->pte_start))
		return -EINVAL;

	/* Can only describe physically contiguous mappings */
	if ((phys != kvm_pte_to_phys(data->pte_start) + data->size))
		return -ERANGE;

	data->size += granule_size;

end:
	/* Request ends in the middle of a huge-mapping */
	if (ctx->start + data->size > ctx->end)
		return -E2BIG;

	return --data->max_ptes > 0 ? 0 : -ERANGE;
}

static int __guest_request_page_transition(u64 ipa, kvm_pte_t *__pte, u64 *__nr_pages,
					   struct pkvm_hyp_vm *vm,
					   enum pkvm_page_state desired)
{
	struct guest_request_walker_data data = GUEST_WALKER_DATA_INIT(desired);
	struct kvm_pgtable_walker walker = {
		.cb     = guest_request_walker,
		.flags  = KVM_PGTABLE_WALK_LEAF,
		.arg    = (void *)&data,
	};
	phys_addr_t phys;
	size_t size;
	int ret;

	if (check_mul_overflow(*__nr_pages, PAGE_SIZE, &size) ||
	    ipa >= ipa + size)
		return -EINVAL;

	ret = kvm_pgtable_walk(&vm->pgt, ipa, size, &walker);
	/*
	 * Walker reached data.max_ptes or a non-physically-contiguous mapping.
	 * Proceed with the current valid region. The guest will have to issue a new call for the
	 * leftover.
	 */
	if (ret == -ERANGE)
		ret = 0;
	else if (ret)
		return ret;

	if (WARN_ON(!kvm_pte_valid(data.pte_start)))
		return -EINVAL;

	phys = kvm_pte_to_phys(data.pte_start);
	ret = check_range_allowed_memory(phys, phys + data.size);
	if (ret)
		return ret;

	*__pte = data.pte_start;
	*__nr_pages = data.size >> PAGE_SHIFT;

	return 0;
}

int __pkvm_host_share_hyp(u64 pfn)
{
	u64 phys = hyp_pfn_to_phys(pfn);
	u64 size = PAGE_SIZE;
	int ret;

	host_lock_component();
	hyp_lock_component();

	ret = __host_check_page_state_range(phys, size, PKVM_PAGE_OWNED);
	if (ret)
		goto unlock;
	ret = __hyp_check_page_state_range(phys, size, PKVM_NOPAGE);
	if (ret)
		goto unlock;

	__hyp_set_page_state_range(phys, size, PKVM_PAGE_SHARED_BORROWED);
	__host_update_page_state(phys, size, PKVM_PAGE_SHARED_OWNED);

unlock:
	hyp_unlock_component();
	host_unlock_component();

	return ret;
}

int __pkvm_host_unshare_hyp(u64 pfn)
{
	u64 phys = hyp_pfn_to_phys(pfn);
	u64 virt = (u64)__hyp_va(phys);
	u64 size = PAGE_SIZE;
	int ret;

	host_lock_component();
	hyp_lock_component();

	ret = __host_check_page_state_range(phys, size, PKVM_PAGE_SHARED_OWNED);
	if (ret)
		goto unlock;
	ret = __hyp_check_page_state_range(phys, size, PKVM_PAGE_SHARED_BORROWED);
	if (ret)
		goto unlock;
	if (hyp_page_referenced((void *)virt)) {
		ret = -EBUSY;
		goto unlock;
	}

	__hyp_set_page_state_range(phys, size, PKVM_NOPAGE);
	__host_update_page_state(phys, size, PKVM_PAGE_OWNED);

unlock:
	hyp_unlock_component();
	host_unlock_component();

	return ret;
}

int __pkvm_guest_share_hyp_page(struct pkvm_hyp_vcpu *vcpu, u64 ipa, u64 *hyp_va)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	int ret;
	kvm_pte_t pte;
	u64 phys;
	enum kvm_pgtable_prot prot;
	void *virt;
	u64 nr_pages = 1;

	hyp_lock_component();
	guest_lock_component(vm);

	ret = __guest_request_page_transition(ipa, &pte, &nr_pages, vm, PKVM_PAGE_OWNED);
	if (ret)
		goto unlock;

	phys = kvm_pte_to_phys(pte);

	virt = __hyp_va(phys);
	if (IS_ENABLED(CONFIG_NVHE_EL2_DEBUG)) {
		ret = __hyp_check_page_state_range(phys, PAGE_SIZE, PKVM_NOPAGE);
		if (ret)
			goto unlock;
	}

	__hyp_set_page_state_range(phys, PAGE_SIZE, PKVM_PAGE_SHARED_BORROWED);
	prot = pkvm_mkstate(PAGE_HYP, PKVM_PAGE_SHARED_BORROWED);
	ret = pkvm_create_mappings_locked(virt, virt + PAGE_SIZE, prot);
	if (ret) {
		/*
		 * Repaint the return code as we need to distinguish between the
		 * no memory from the guest which is recoverable and no memory
		 * from the hypervisor.
		 */
		if (ret == -ENOMEM)
			ret = -EBUSY;
		goto unlock;
	}

	ret = kvm_pgtable_stage2_map(&vm->pgt, ipa, PAGE_SIZE, phys,
				     pkvm_mkstate(KVM_PGTABLE_PROT_RWX, PKVM_PAGE_SHARED_OWNED),
				     &vcpu->vcpu.arch.stage2_mc, 0);
	if (!ret)
		*hyp_va = (u64)virt;
unlock:
	guest_unlock_component(vm);
	hyp_unlock_component();

	return ret;
}

int __pkvm_guest_unshare_hyp_page(struct pkvm_hyp_vm *vm, u64 ipa)
{
	int ret;
	kvm_pte_t pte;
	u64 phys, virt, nr_pages = 1;

	hyp_lock_component();
	guest_lock_component(vm);

	ret = __guest_request_page_transition(ipa, &pte, &nr_pages, vm, PKVM_PAGE_SHARED_OWNED);
	if (ret)
		goto unlock;

	phys = kvm_pte_to_phys(pte);

	virt = (u64)__hyp_va(phys);
	ret = __hyp_check_page_state_range(phys, PAGE_SIZE, PKVM_PAGE_SHARED_BORROWED);
	if (ret)
		goto unlock;

	__hyp_set_page_state_range(phys, PAGE_SIZE, PKVM_NOPAGE);
	WARN_ON(kvm_pgtable_hyp_unmap(&pkvm_pgtable, virt, PAGE_SIZE) != PAGE_SIZE);
	/*
	 * NULL memcache: this is the reverse of a prior PAGE_SIZE share, so the
	 * walker rejected any coarser leaf with -E2BIG. The map below only
	 * flips software state bits via the try_leaf fast path and never
	 * allocates. The teardown path runs without a loaded vCPU, so there is
	 * no correct memcache to pass.
	 */
	ret = kvm_pgtable_stage2_map(&vm->pgt, ipa, PAGE_SIZE, phys,
				     pkvm_mkstate(KVM_PGTABLE_PROT_RWX, PKVM_PAGE_OWNED),
				     NULL, 0);
unlock:
	guest_unlock_component(vm);
	hyp_unlock_component();

	return ret;
}

int __pkvm_guest_share_ffa_page(struct pkvm_hyp_vcpu *vcpu, u64 ipa, phys_addr_t *phys)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	int ret;
	kvm_pte_t pte;
	u64 nr_pages = 1;
	phys_addr_t pa;

	guest_lock_component(vm);
	ret = __guest_request_page_transition(ipa, &pte, &nr_pages, vm, PKVM_PAGE_OWNED);
	if (ret)
		goto unlock;

	pa = kvm_pte_to_phys(pte);
	ret = kvm_pgtable_stage2_map(&vm->pgt, ipa, PAGE_SIZE, pa,
				     pkvm_mkstate(KVM_PGTABLE_PROT_RWX, PKVM_PAGE_SHARED_OWNED),
				     &vcpu->vcpu.arch.stage2_mc, 0);
	if (!ret)
		*phys = pa;
unlock:
	guest_unlock_component(vm);

	return ret;
}

/*
 * The caller is responsible for tracking the FFA state and this function
 * should only be called for IPAs that have previously been shared with FFA.
 */
int __pkvm_guest_unshare_ffa_page(struct pkvm_hyp_vm *vm, u64 ipa)
{
	int ret;
	kvm_pte_t pte;
	u64 nr_pages = 1;

	guest_lock_component(vm);
	ret = __guest_request_page_transition(ipa, &pte, &nr_pages, vm, PKVM_PAGE_SHARED_OWNED);
	if (ret)
		goto unlock;

	/* See __pkvm_guest_unshare_hyp_page() for why mc is NULL. */
	ret = kvm_pgtable_stage2_map(&vm->pgt, ipa, PAGE_SIZE, kvm_pte_to_phys(pte),
				     pkvm_mkstate(KVM_PGTABLE_PROT_RWX, PKVM_PAGE_OWNED),
				     NULL, 0);
unlock:
	guest_unlock_component(vm);

	return ret;
}

static int pkvm_host_donate_hyp(u64 pfn, u64 nr_pages, enum kvm_pgtable_prot prot,
				enum host_check_page_state_flags flags)
{
	u64 size, phys;
	void *virt;
	int ret;

	if (!pfn_range_is_valid(pfn, nr_pages))
		return -EINVAL;

	phys = hyp_pfn_to_phys(pfn);
	size = nr_pages * PAGE_SIZE;
	virt = __hyp_va(phys);

	host_lock_component();
	hyp_lock_component();

	ret = ___host_check_page_state_range(phys, size, PKVM_PAGE_OWNED, flags);
	if (ret)
		goto unlock;
	ret = __hyp_check_page_state_range(phys, size, PKVM_NOPAGE);
	if (ret)
		goto unlock;

	ret = host_stage2_set_owner_locked(phys, size, PKVM_ID_HYP);
	if (ret)
		goto unlock;

	ret = pkvm_create_mappings_locked(virt, virt + size, prot);
	if (ret) {
		WARN_ON(ret != -ENOMEM);
		/* We might have failed halfway through, so remove anything we've installed */
		pkvm_remove_mappings_locked(virt, virt + size);
		host_stage2_set_owner_locked(phys, size, PKVM_ID_HOST);
		goto unlock;
	}

	/*
	 * Only allow hyp MMIO transitions to/from the host
	 */
	if (range_is_memory(phys, phys + size))
		__hyp_set_page_state_range(phys, size, PKVM_PAGE_OWNED);

unlock:
	hyp_unlock_component();
	host_unlock_component();

	return ret;
}

/* The Swiss Army knife of memory donation */
int ___pkvm_host_donate_hyp_prot(u64 pfn, u64 nr_pages,
				 bool accept_mmio, enum kvm_pgtable_prot prot)
{
	enum host_check_page_state_flags flags = HOST_CHECK_NULL_REFCNT;

	if (!accept_mmio)
		flags |= HOST_CHECK_IS_MEMORY;

	return pkvm_host_donate_hyp(pfn, nr_pages, prot, flags);
}

int ___pkvm_host_donate_hyp(u64 pfn, u64 nr_pages, bool accept_mmio)
{
	return ___pkvm_host_donate_hyp_prot(pfn, nr_pages, accept_mmio,
					    default_hyp_prot(hyp_pfn_to_phys(pfn)));
}

int __pkvm_host_donate_hyp(u64 pfn, u64 nr_pages)
{
	return ___pkvm_host_donate_hyp(pfn, nr_pages, false);
}

int __pkvm_host_donate_sglist_hyp(struct pkvm_sglist_page *sglist, size_t nr_pages)
{
	enum kvm_pgtable_prot prot;
	u8 max_order;
	size_t size;
	int p, ret;
	u64 phys;

	host_lock_component();
	hyp_lock_component();

	/* Checking we are reading hyp private memory */
	WARN_ON(__hyp_check_page_state_range((u64)sglist, nr_pages * sizeof(*sglist),
					     PKVM_PAGE_OWNED));

	for (p = 0; p < nr_pages; p++) {
		max_order = get_order(PMD_SIZE);

		if (sglist[p].order > max_order) {
			ret = -EINVAL;
			goto err_page_state;
		}
		size = PAGE_SIZE << sglist[p].order;

		if (!pfn_range_is_valid(sglist[p].pfn, size >> PAGE_SHIFT)) {
			ret = -EINVAL;
			goto err_page_state;
		}
		phys = hyp_pfn_to_phys(sglist[p].pfn);

		ret = __host_check_page_state_range(phys, size, PKVM_PAGE_OWNED);
		if (ret)
			goto err_page_state;

		ret = __hyp_check_page_state_range((u64)__hyp_va(phys), size, PKVM_NOPAGE);
		if (ret)
			goto err_page_state;

		prot = pkvm_mkstate(PAGE_HYP, PKVM_PAGE_OWNED);
		ret = pkvm_create_mappings_locked(__hyp_va(phys), __hyp_va(phys) + size, prot);
		if (ret == -ENOMEM)
			goto err_page_state;

		WARN_ON(ret);

		ret = __host_stage2_set_owner_locked(phys, size, PKVM_ID_HYP, 0,
						     HOST_SET_NO_COMPLETE);
		if (ret) {
			pkvm_remove_mappings_locked(__hyp_va(phys), __hyp_va(phys) + size);
			goto err_page_state;
		}
	}

	__host_stage2_set_owner_complete(PKVM_ID_HYP, 0);

unlock:
	hyp_unlock_component();
	host_unlock_component();

	return ret;
err_page_state:
	/*
	 * Roll back if either hyp stage-1 -ENOMEM or host stage-2 -ENOMEMHOSTS2
	 * or if the page state doesn't match the expected state.
	 */
	if (p == 0)
		goto unlock;

	__host_stage2_set_owner_complete(PKVM_ID_HYP, 0);
	while (p-- > 0) {
		phys = hyp_pfn_to_phys(sglist[p].pfn);
		size = PAGE_SIZE << sglist[p].order;

		pkvm_remove_mappings_locked(__hyp_va(phys), __hyp_va(phys) + size);
		WARN_ON(host_stage2_set_owner_locked(phys, size, PKVM_ID_HOST));
	}
	goto unlock;
}

static int pkvm_hyp_donate_guest(struct pkvm_hyp_vcpu *vcpu, u64 pfn, u64 gfn)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	u64 phys = hyp_pfn_to_phys(pfn);
	u64 ipa = hyp_pfn_to_phys(gfn);
	u64 hyp_addr = (u64)__hyp_va(phys);
	size_t size = PAGE_SIZE;
	enum kvm_pgtable_prot prot;
	int ret;

	if (addr_is_memory(phys))
		return -EINVAL;

	hyp_assert_lock_held(&pkvm_pgd_lock);
	hyp_assert_lock_held(&vm->pgtable_lock);

	ret = __hyp_check_page_state_range(phys, size, PKVM_PAGE_OWNED);
	if (ret)
		return ret;
	ret = __guest_check_page_state_range(vm, ipa, size, PKVM_NOPAGE);
	if (ret)
		return ret;

	WARN_ON(kvm_pgtable_hyp_unmap(&pkvm_pgtable, hyp_addr, size) != size);
	prot = pkvm_mkstate(KVM_PGTABLE_PROT_RW | KVM_PGTABLE_PROT_NORMAL_NC,
			      PKVM_PAGE_OWNED);
	return WARN_ON(kvm_pgtable_stage2_map(&vm->pgt, ipa, size, phys, prot,
					      &vcpu->vcpu.arch.stage2_mc, 0));
}

int __pkvm_hyp_donate_host(u64 pfn, u64 nr_pages)
{
	u64 size, phys = hyp_pfn_to_phys(pfn);
	u64 virt = (u64)__hyp_va(phys);
	int ret;

	if (check_shl_overflow(nr_pages, PAGE_SHIFT, &size))
		return -EINVAL;

	if (!pfn_range_is_valid(pfn, nr_pages))
		return -EINVAL;

	host_lock_component();
	hyp_lock_component();

	ret = __hyp_check_page_state_range(phys, size, PKVM_PAGE_OWNED);
	if (ret)
		goto unlock;
	ret = ___host_check_page_state_range(phys, size, PKVM_NOPAGE, 0);
	if (ret)
		goto unlock;

	/* See __pkvm_host_donate_hyp_locked() */
	if (range_is_memory(phys, phys + size))
		__hyp_set_page_state_range(phys, size, PKVM_NOPAGE);
	WARN_ON(kvm_pgtable_hyp_unmap(&pkvm_pgtable, virt, size) != size);
	WARN_ON(host_stage2_set_owner_locked(phys, size, PKVM_ID_HOST));

unlock:
	hyp_unlock_component();
	host_unlock_component();

	return ret;
}

/*
 * Rejects MMIO regions and is unsafe. Use with care!
 */
int __pkvm_host_donate_ffa(u64 pfn, u64 nr_pages)
{
	u64 size, phys = hyp_pfn_to_phys(pfn), end;
	int ret;

	if (check_shl_overflow(nr_pages, PAGE_SHIFT, &size) ||
	    check_add_overflow(phys, size, &end))
		return -EINVAL;

	host_lock_component();

	ret = ___host_check_page_state_range(phys, size, PKVM_PAGE_OWNED,
					     HOST_CHECK_IS_MEMORY |
						     HOST_CHECK_NULL_REFCNT |
						     HOST_CHECK_ALLOW_NO_MAP);
	if (ret)
		goto unlock;

	/* HOST_SET_NO_COMPLETE to skip pkvm_sme_dvmsync_fw_call() */
	ret = __host_stage2_set_owner_locked(phys, size, PKVM_ID_FFA, 0,
					     HOST_SET_NO_IOMMU_UPDATE | HOST_SET_NO_COMPLETE);

unlock:
	host_unlock_component();
	return ret;
}

/*
 * Just like __pkvm_donate_ffa, rejects MMIO regions and does not update the IOMMU.
 */
int __pkvm_host_reclaim_ffa(u64 pfn, u64 nr_pages)
{
	u64 size, phys = hyp_pfn_to_phys(pfn), end;
	int ret;

	if (check_shl_overflow(nr_pages, PAGE_SHIFT, &size) ||
	    check_add_overflow(phys, size, &end))
		return -EINVAL;

	host_lock_component();

	ret = ___host_check_page_state_range(phys, size, PKVM_NOPAGE,
					     HOST_CHECK_IS_MEMORY |
						     HOST_CHECK_ALLOW_NO_MAP);
	if (ret)
		goto unlock;

	WARN_ON(__host_stage2_set_owner_locked(phys, size, PKVM_ID_HOST, 0,
					       HOST_SET_NO_IOMMU_UPDATE));
unlock:
	host_unlock_component();
	return ret;
}

#define MODULE_PROT_ALLOWLIST (KVM_PGTABLE_PROT_RWX |		\
			       KVM_PGTABLE_PROT_DEVICE |	\
			       KVM_PGTABLE_PROT_NORMAL_NC |	\
			       KVM_PGTABLE_PROT_PXN |		\
			       KVM_PGTABLE_PROT_UXN)

int module_change_host_page_prot(u64 pfn, enum kvm_pgtable_prot prot,
				 u64 nr_pages, bool update_iommu)
{
	u64 i, end, addr = hyp_pfn_to_phys(pfn);
	struct hyp_page *page = NULL;
	struct kvm_mem_range range;
	struct memblock_region *reg;
	int ret;

	if ((prot & MODULE_PROT_ALLOWLIST) != prot)
		return -EINVAL;

	if (check_shl_overflow(nr_pages, PAGE_SHIFT, &end) ||
			check_add_overflow(addr, end, &end))
		return -EINVAL;

	reg = find_mem_range(addr, &range);
	if (end > range.end) {
		/* Specified range not in a single mmio or memory block. */
		return -EPERM;
	}

	host_lock_component();
	/*
	 * There is no hyp_vmemmap covering MMIO regions, which makes tracking
	 * of module-owned MMIO regions hard, so we trust the modules not to
	 * mess things up.
	 */
	if (!reg)
		goto update;

	/* Range is memory: we can track module ownership. */
	page = hyp_phys_to_page(addr);

	/*
	 * Modules can only modify pages they already own, and pristine host
	 * pages. The entire range must be consistently one or the other.
	 */
	if (get_host_state(page) & PKVM_MODULE_OWNED_PAGE) {
		/* The entire range must be module-owned. */
		ret = -EPERM;
		for (i = 1; i < nr_pages; i++) {
			if (!(get_host_state(&page[i]) & PKVM_MODULE_OWNED_PAGE))
				goto unlock;
		}
	} else {
		/* The entire range must be pristine. */
		ret = ___host_check_page_state_range(addr, nr_pages << PAGE_SHIFT,
						     PKVM_PAGE_OWNED, HOST_CHECK_NULL_REFCNT);
		if (ret)
			goto unlock;
	}

update:
	if (!prot) {
		enum host_set_page_state_flags flags = 0;

		if (!reg)
			flags |= HOST_SET_IS_MMIO;
		if (!update_iommu)
			flags |= HOST_SET_NO_IOMMU_UPDATE;

		ret = __host_stage2_set_owner_locked(addr, nr_pages << PAGE_SHIFT,
						     PKVM_ID_PROTECTED,
						     PKVM_MODULE_OWNED_PAGE, flags);
	} else {
		/* !update_iommu is fast but not safe. For that reason it also skips dvmsync */
		if (!update_iommu) {
			ret = __host_stage2_idmap(addr, nr_pages << PAGE_SHIFT, prot, reg);
		} else {
			ret = host_stage2_idmap_locked(addr, nr_pages << PAGE_SHIFT, prot, reg);
			if (!ret) {
				WARN_ON(kvm_iommu_host_stage2_idmap(addr, end, prot));
				kvm_iommu_host_stage2_idmap_complete(!!prot);
			}
		}
	}

	if (ret || !page || !prot)
		goto unlock;

	for (i = 0; i < nr_pages; i++) {
		if (prot != KVM_PGTABLE_PROT_RWX)
			set_host_state(&page[i], PKVM_MODULE_OWNED_PAGE);
		else
			set_host_state(&page[i], PKVM_PAGE_OWNED);
	}

unlock:
	host_unlock_component();
	return ret;
}

int hyp_pin_shared_mem(void *from, void *to)
{
	u64 cur, start = ALIGN_DOWN((u64)from, PAGE_SIZE);
	u64 end = PAGE_ALIGN((u64)to);
	u64 phys = __hyp_pa(start);
	u64 size = end - start;
	struct hyp_page *p;
	int ret;

	host_lock_component();
	hyp_lock_component();

	ret = __host_check_page_state_range(phys, size, PKVM_PAGE_SHARED_OWNED);
	if (ret)
		goto unlock;

	ret = __hyp_check_page_state_range(phys, size, PKVM_PAGE_SHARED_BORROWED);
	if (ret)
		goto unlock;

	for (cur = start; cur < end; cur += PAGE_SIZE) {
		p = hyp_virt_to_page(cur);
		hyp_page_ref_inc(p);
		if (p->refcount == 1)
			ret = pkvm_create_mappings_locked((void *)cur,
							  (void *)cur + PAGE_SIZE,
							  PAGE_HYP);
	}

	if (ret) {
		WARN_ON(ret != -ENOMEM);
		/* We might have failed halfway through, so remove anything we've installed */
		end = cur;
		for (cur = start; cur < end; cur += PAGE_SIZE) {
			p = hyp_virt_to_page(cur);
			hyp_page_ref_dec(p);
			if (p->refcount == 0)
				pkvm_remove_mappings_locked((void *)cur, (void *)cur + PAGE_SIZE);
		}
	}

unlock:
	hyp_unlock_component();
	host_unlock_component();

	return ret;
}

void hyp_unpin_shared_mem(void *from, void *to)
{
	u64 cur, start = ALIGN_DOWN((u64)from, PAGE_SIZE);
	u64 end = PAGE_ALIGN((u64)to);
	struct hyp_page *p;

	host_lock_component();
	hyp_lock_component();

	for (cur = start; cur < end; cur += PAGE_SIZE) {
		p = hyp_virt_to_page(cur);
		if (p->refcount == 1)
			WARN_ON(kvm_pgtable_hyp_unmap(&pkvm_pgtable, cur, PAGE_SIZE) != PAGE_SIZE);
		hyp_page_ref_dec(p);
	}

	hyp_unlock_component();
	host_unlock_component();
}

int __pkvm_host_share_ffa(u64 pfn, u64 nr_pages)
{
	u64 size, phys = hyp_pfn_to_phys(pfn);
	int ret;

	if (check_shl_overflow(nr_pages, PAGE_SHIFT, &size))
		return -EINVAL;

	if (!pfn_range_is_valid(pfn, nr_pages))
		return -EINVAL;

	host_lock_component();
	ret = ___host_check_page_state_range(phys, size, PKVM_PAGE_OWNED,
					     HOST_CHECK_IS_MEMORY |
						     HOST_CHECK_NULL_REFCNT |
						     HOST_CHECK_ALLOW_NO_MAP);
	if (!ret)
		__host_update_page_state(phys, size, PKVM_PAGE_SHARED_OWNED);
	host_unlock_component();

	return ret;
}

int __pkvm_host_unshare_ffa(u64 pfn, u64 nr_pages)
{
	u64 size, phys = hyp_pfn_to_phys(pfn);
	int ret;

	if (check_shl_overflow(nr_pages, PAGE_SHIFT, &size))
		return -EINVAL;

	if (!pfn_range_is_valid(pfn, nr_pages))
		return -EINVAL;

	host_lock_component();
	ret = ___host_check_page_state_range(phys, size, PKVM_PAGE_SHARED_OWNED,
					     HOST_CHECK_IS_MEMORY |
						     HOST_CHECK_ALLOW_NO_MAP);
	if (!ret)
		__host_update_page_state(phys, size, PKVM_PAGE_OWNED);
	host_unlock_component();

	return ret;
}

static int __guest_check_transition_size(u64 phys, u64 ipa, u64 nr_pages, u64 *size)
{
	size_t block_size;

	if (nr_pages == 1) {
		*size = PAGE_SIZE;
		return 0;
	}

	/* We solely support second to last level huge mapping */
	block_size = kvm_granule_size(KVM_PGTABLE_LAST_LEVEL - 1);

	if (nr_pages != block_size >> PAGE_SHIFT)
		return -EINVAL;

	if (!IS_ALIGNED(phys | ipa, block_size))
		return -EINVAL;

	*size = block_size;
	return 0;
}

static void __hyp_poison_page(void *addr, size_t size)
{
	memset(addr, 0, size);
	__hyp_flush_page(addr, size);
}

void hyp_poison_page(phys_addr_t phys, size_t size)
{
	__apply_guest_page(__hyp_va(phys), size, __hyp_poison_page);
}

static int get_valid_guest_pte(struct pkvm_hyp_vm *vm, u64 ipa, kvm_pte_t *ptep, u64 *physp,
			       size_t size)
{
	kvm_pte_t pte;
	s8 level;
	int ret;

	if (size != PAGE_SIZE && size != PMD_SIZE)
		return -EINVAL;

	if (ipa != ALIGN_DOWN(ipa, size))
		return -EINVAL;

	ret = kvm_pgtable_get_leaf(&vm->pgt, ipa, &pte, &level);
	if (ret)
		return ret;
	if (!kvm_pte_valid(pte))
		return -ENOENT;
	if (kvm_granule_size(level) != size)
		return -E2BIG;

	*ptep = pte;
	*physp = kvm_pte_to_phys(pte);

	return 0;
}

/* Return PA for an owned guest IPA or request it, and repeat the guest HVC */
int pkvm_get_guest_pa_request(struct pkvm_hyp_vcpu *hyp_vcpu, u64 ipa,
			      size_t ipa_size_request, u64 *out_pa, s8 *out_level)
{
	struct kvm_hyp_req *req;
	kvm_pte_t pte;
	enum pkvm_page_state state;
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu);
	int ret;

	guest_lock_component(vm);
	ret = kvm_pgtable_get_leaf(&vm->pgt, ipa, &pte, out_level);
	guest_unlock_component(vm);

	if (ret)
		return ret;

	if (!kvm_pte_valid(pte)) {
		/* Page not mapped, create a request*/
		req = pkvm_hyp_req_reserve(hyp_vcpu, KVM_HYP_REQ_TYPE_MAP);
		if (!req)
			return -ENOMEM;

		req->map.guest_ipa = ipa;
		req->map.size = ipa_size_request;
		return -ENOENT;
	}

	state = pkvm_getstate(kvm_pgtable_stage2_pte_prot(pte));
	if (state != PKVM_PAGE_OWNED)
		return -EPERM;

	*out_pa = kvm_pte_to_phys(pte);
	*out_pa |= (ipa & (kvm_granule_size(*out_level) - 1)) & PAGE_MASK;
	return 0;
}

static int ___pkvm_module_unshare_guest(struct pkvm_hyp_vm *vm, u64 phys, u64 ipa, u64 size)
{

	if (___host_check_page_state_range(phys, size, PKVM_MODULE_SHARED_OWNED_PAGE, HOST_CHECK_IS_MEMORY))
		return -EPERM;

	if (__guest_check_page_state_range(vm, ipa, size, PKVM_PAGE_SHARED_BORROWED))
		return -EPERM;

	WARN_ON(kvm_pgtable_stage2_unmap(&vm->pgt, ipa, size));
	__host_update_page_state(phys, size, PKVM_MODULE_OWNED_PAGE);

	return 0;
}

int __pkvm_host_reclaim_page_guest(u64 gfn, u64 nr_pages, struct pkvm_hyp_vm *vm)
{
	u64 phys, size, ipa = hyp_pfn_to_phys(gfn);
	enum host_set_page_state_flags flags;
	kvm_pte_t pte;
	int ret;

	ret = __guest_check_transition_size(0, ipa, nr_pages, &size);
	if (ret)
		return ret;

	if (!pkvm_hyp_vm_is_protected(vm))
		return -EPERM;

	host_lock_component();
	guest_lock_component(vm);

	ret = get_valid_guest_pte(vm, ipa, &pte, &phys, size);
	if (ret)
		goto unlock;

	switch ((int)guest_get_page_state(pte, ipa)) {
	case PKVM_PAGE_OWNED:
		WARN_ON(__host_check_page_state_range(phys, size, PKVM_NOPAGE));
		hyp_poison_page(phys, size);
		flags = HOST_SET_PSCI_MEM_PROTECT;
		break;
	case PKVM_PAGE_SHARED_BORROWED:
	case PKVM_PAGE_SHARED_BORROWED | PKVM_PAGE_RESTRICTED_PROT:
		ret = ___pkvm_module_unshare_guest(vm, phys, ipa, size);
		goto unlock;
	case PKVM_PAGE_SHARED_OWNED:
		if (__host_check_page_state_range(phys, size, PKVM_PAGE_SHARED_BORROWED)) {
			/* Presumably a page shared via FF-A, will be handled separately */
			ret = -EBUSY;
			goto unlock;
		}

		/* We only need to clear PKVM_PAGE_SHARED_BORROWED from the SW-bits */
		flags = HOST_SET_NO_COMPLETE | HOST_SET_NO_IOMMU_UPDATE;
		break;
	default:
		ret = -EPERM;
		goto unlock;
	}

	/* We could avoid TLB inval, it is done per VMID on the finalize path */
	WARN_ON(kvm_pgtable_stage2_unmap(&vm->pgt, ipa, size));
	WARN_ON(__host_stage2_set_owner_locked(phys, size, PKVM_ID_HOST, 0, flags));

unlock:
	guest_unlock_component(vm);
	host_unlock_component();

	return ret;
}

int __pkvm_guest_share_host(u64 gfn, struct pkvm_hyp_vcpu *vcpu, u64 nr_pages, u64 *nr_shared)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	u64 ipa = hyp_pfn_to_phys(gfn);
	enum kvm_pgtable_prot prot;
	kvm_pte_t pte;
	size_t size;
	u64 phys;
	int ret;

	host_lock_component();
	guest_lock_component(vm);
	ret = __guest_request_page_transition(ipa, &pte, &nr_pages, vm, PKVM_PAGE_OWNED);
	if (ret)
		goto unlock;

	phys = kvm_pte_to_phys(pte);
	if (check_shl_overflow(nr_pages, PAGE_SHIFT, &size)) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = __host_check_page_state_range(phys, size, PKVM_NOPAGE);
	if (ret)
		goto unlock;

	/*
	 * Setting PKVM_PAGE_SHARED_BORROWED ensures this mapping is refcounted
	 * which prevents stage-2 coalescing.
	 */
	prot = pkvm_mkstate(PKVM_HOST_MEM_PROT, PKVM_PAGE_SHARED_BORROWED);
	ret = host_stage2_idmap_locked(phys, size, prot, true);
	if (ret)
		goto unlock;

	WARN_ON(kvm_iommu_host_stage2_idmap(phys, phys + size, PKVM_HOST_MEM_PROT));
	kvm_iommu_host_stage2_idmap_complete(true);

	__host_update_page_state(phys, size, PKVM_PAGE_SHARED_BORROWED);

	psci_mem_protect_dec(nr_pages);
	WARN_ON(kvm_pgtable_stage2_map(&vm->pgt, ipa, size, phys,
				       pkvm_mkstate(KVM_PGTABLE_PROT_RWX, PKVM_PAGE_SHARED_OWNED),
				       &vcpu->vcpu.arch.stage2_mc, 0));
	*nr_shared = nr_pages;
unlock:
	guest_unlock_component(vm);
	host_unlock_component();

	return ret;
}

int __pkvm_guest_unshare_host(u64 gfn, struct pkvm_hyp_vcpu *vcpu, u64 nr_pages, u64 *nr_unshared)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	u64 ipa = hyp_pfn_to_phys(gfn);
	kvm_pte_t pte;
	size_t size;
	u64 phys;
	int ret;

	host_lock_component();
	guest_lock_component(vm);
	ret = __guest_request_page_transition(ipa, &pte, &nr_pages, vm, PKVM_PAGE_SHARED_OWNED);
	if (ret)
		goto unlock;

	phys = kvm_pte_to_phys(pte);
	if (check_shl_overflow(nr_pages, PAGE_SHIFT, &size)) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = __host_check_page_state_range(phys, size, PKVM_PAGE_SHARED_BORROWED);
	if (ret)
		goto unlock;

	ret = __host_stage2_set_owner_locked(phys, size, PKVM_ID_GUEST, 0,
					     HOST_SET_PSCI_MEM_PROTECT);
	if (ret)
		goto unlock;

	WARN_ON(kvm_pgtable_stage2_map(&vm->pgt, ipa, size, phys,
				       pkvm_mkstate(KVM_PGTABLE_PROT_RWX, PKVM_PAGE_OWNED),
				       &vcpu->vcpu.arch.stage2_mc, 0));
	*nr_unshared = nr_pages;
unlock:
	guest_unlock_component(vm);
	host_unlock_component();

	return ret;
}

static int ___pkvm_check_module_share_guest(struct pkvm_hyp_vm *vm, u64 phys, u64 ipa, u64 size)
{
	int ret;

	ret = ___host_check_page_state_range(phys, size,
					     PKVM_NOPAGE | PKVM_MODULE_OWNED_PAGE,
					     HOST_CHECK_IS_MEMORY);
	if (ret)
		return ret;

	ret = __guest_check_page_state_range(vm, ipa, size,
					     PKVM_NOPAGE | PKVM_ACCEPT_MODULE_OWNED);
	if (ret)
		return ret;

	return module_guest_accept_module_owned_share(phys, ipa, size, vm);
}

static int ___pkvm_module_share_guest(u64 pfn, u64 gfn, u64 nr_pages, struct pkvm_hyp_vcpu *vcpu)
{
	enum kvm_pgtable_prot prot = pkvm_mkstate(KVM_PGTABLE_PROT_RWX, PKVM_PAGE_SHARED_BORROWED);
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	u64 size = nr_pages * PAGE_SIZE;
	u64 phys = hyp_pfn_to_phys(pfn);
	u64 ipa = hyp_pfn_to_phys(gfn);
	int ret;

	ret = ___pkvm_check_module_share_guest(vm, phys, ipa, size);
	if (!ret) {
		__host_update_page_state(phys, size, PKVM_MODULE_SHARED_OWNED_PAGE);
		WARN_ON(kvm_pgtable_stage2_map(&vm->pgt, ipa, size, phys, prot,
					       &vcpu->vcpu.arch.stage2_mc, 0));
	}

	return ret;
}

int __pkvm_host_donate_guest(u64 pfn, u64 gfn, u64 nr_pages, struct pkvm_hyp_vcpu *vcpu)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	u64 phys = hyp_pfn_to_phys(pfn);
	u64 ipa = hyp_pfn_to_phys(gfn);
	enum kvm_pgtable_prot prot;
	u64 size;
	int ret;

	ret = __guest_check_transition_size(phys, ipa, nr_pages, &size);
	if (ret)
		return ret;

	host_lock_component();
	guest_lock_component(vm);

	ret = ___host_check_page_state_range(phys, size, PKVM_PAGE_OWNED,
					     HOST_CHECK_NULL_REFCNT |
					     HOST_CHECK_IS_MEMORY);
	if (ret) {
		if (ret == -EPERM)
			ret = ___pkvm_module_share_guest(pfn, gfn, nr_pages, vcpu);
		goto unlock;
	}

	ret = __guest_check_page_state_range(vm, ipa, size, PKVM_NOPAGE);
	if (ret)
		goto unlock;

	ret = __host_stage2_set_owner_locked(phys, size, PKVM_ID_GUEST, 0,
					     HOST_SET_PSCI_MEM_PROTECT);
	if (ret)
		goto unlock;

	if (pkvm_ipa_range_has_pvmfw(vm, ipa, ipa + size))
		WARN_ON(pkvm_load_pvmfw_pages(vm, ipa, phys, size));

	prot = pkvm_mkstate(KVM_PGTABLE_PROT_RWX, PKVM_PAGE_OWNED);
	WARN_ON(kvm_pgtable_stage2_map(&vm->pgt, ipa, size, phys, prot,
		&vcpu->vcpu.arch.stage2_mc, 0));
unlock:
	guest_unlock_component(vm);
	host_unlock_component();

	return ret;
}

struct kvm_hyp_pinned_page *hyp_ppages;

static int __copy_hyp_ppages(struct pkvm_hyp_vcpu *vcpu)
{
	struct kvm_hyp_pinned_page *ppage, *hyp_ppage;

	WARN_ON(!hyp_ppages);

	ppage = next_kvm_hyp_pinned_page(vcpu->vcpu.arch.hyp_reqs, NULL, true);
	if (!ppage)
		return -EINVAL;

	hyp_ppage = hyp_ppages;

	do {
		memcpy(hyp_ppage, ppage, sizeof(*ppage));
		ppage = next_kvm_hyp_pinned_page(vcpu->vcpu.arch.hyp_reqs, ppage, true);
		hyp_ppage++; /* No risk to overflow hyp_ppages */
	} while (ppage);

	hyp_ppage->order = 0xFF;

	return 0;
}

#define for_each_hyp_ppage(hyp_ppage)						\
	for (hyp_ppage = hyp_ppages; (hyp_ppage)->order != 0xFF; (hyp_ppage)++)

static int ___pkvm_module_share_guest_sglist(struct pkvm_hyp_vcpu *vcpu)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	struct kvm_hyp_pinned_page *ppage = hyp_ppages;
	u64 phys, ipa, size;
	int ret;

	for_each_hyp_ppage(ppage) {
		phys = hyp_pfn_to_phys(ppage->pfn);
		ipa = hyp_pfn_to_phys(ppage->gfn);

		if (!pfn_range_is_valid(ppage->pfn, 1UL << ppage->order)) {
			ret = -EINVAL;
			goto fail;
		}

		ret = __guest_check_transition_size(phys, ipa, 1UL << ppage->order, &size);
		if (ret)
			goto fail;

		ret = ___pkvm_check_module_share_guest(vm, phys, ipa, size);
		if (ret)
			goto fail;

		__host_update_page_state(phys, size, PKVM_MODULE_SHARED_OWNED_PAGE);
	}

	for_each_hyp_ppage(ppage) {
		phys = hyp_pfn_to_phys(ppage->pfn);
		ipa = hyp_pfn_to_phys(ppage->gfn);
		size = PAGE_SIZE << ppage->order;
		enum kvm_pgtable_prot prot = pkvm_mkstate(
			KVM_PGTABLE_PROT_RWX, PKVM_PAGE_SHARED_BORROWED);

		WARN_ON(kvm_pgtable_stage2_map(&vm->pgt, ipa, size, phys, prot,
					       &vcpu->vcpu.arch.stage2_mc, 0));
	}

	return 0;

fail:
	while (ppage-- != hyp_ppages) {
		u64 phys = hyp_pfn_to_phys(ppage->pfn);
		u64 size = PAGE_SIZE << ppage->order;

		__host_update_page_state(phys, size, PKVM_MODULE_OWNED_PAGE);
	}
	return ret;
}

int __pkvm_host_donate_sglist_guest(struct pkvm_hyp_vcpu *vcpu)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	struct kvm_hyp_pinned_page *ppage = hyp_ppages;
	enum kvm_pgtable_prot prot;
	u64 phys, ipa, size;
	int ret;

	host_lock_component();
	guest_lock_component(vm);

	ret = __copy_hyp_ppages(vcpu);
	if (ret)
		goto unlock;

	for_each_hyp_ppage(ppage) {
		phys = hyp_pfn_to_phys(ppage->pfn);
		ipa = hyp_pfn_to_phys(ppage->gfn);

		if (!pfn_range_is_valid(ppage->pfn, 1UL << ppage->order)) {
			ret = -EINVAL;
			goto err_rollback;
		}

		ret = __guest_check_transition_size(phys, ipa, 1UL << ppage->order, &size);
		if (ret)
			goto err_rollback;

		ret = ___host_check_page_state_range(phys, size, PKVM_PAGE_OWNED,
						     HOST_CHECK_NULL_REFCNT |
						     HOST_CHECK_IS_MEMORY);
		if (ret)
			goto err_rollback;

		ret = __guest_check_page_state_range(vm, ipa, size, PKVM_NOPAGE);
		if (ret)
			goto err_rollback;

		ret = __host_stage2_set_owner_locked(phys, size, PKVM_ID_GUEST, 0,
						     HOST_SET_NO_COMPLETE |
						     HOST_SET_PSCI_MEM_PROTECT);
		if (ret)
			goto err_rollback;
	}
	__host_stage2_set_owner_complete(PKVM_ID_GUEST, 0);

	for_each_hyp_ppage(ppage) {
		size = PAGE_SIZE << ppage->order;
		phys = hyp_pfn_to_phys(ppage->pfn);
		ipa = hyp_pfn_to_phys(ppage->gfn);

		if (pkvm_ipa_range_has_pvmfw(vm, ipa, ipa + size))
			WARN_ON(pkvm_load_pvmfw_pages(vm, ipa, phys, size));

		prot = pkvm_mkstate(KVM_PGTABLE_PROT_RWX, PKVM_PAGE_OWNED);
		WARN_ON(kvm_pgtable_stage2_map(&vm->pgt, ipa, size, phys, prot,
					       &vcpu->vcpu.arch.stage2_mc, 0));
	}

unlock:
	guest_unlock_component(vm);
	host_unlock_component();

	return ret;

err_rollback:
	if (ppage == hyp_ppages) {
		if (ret == -EPERM)
			ret = ___pkvm_module_share_guest_sglist(vcpu);
		goto unlock;
	}

	__host_stage2_set_owner_complete(PKVM_ID_GUEST, 0);
	while (ppage-- != hyp_ppages) {
		size = PAGE_SIZE << ppage->order;
		phys = hyp_pfn_to_phys(ppage->pfn);
		ipa = hyp_pfn_to_phys(ppage->gfn);
		WARN_ON(__host_stage2_set_owner_locked(phys, size, PKVM_ID_HOST, 0,
						       HOST_SET_NO_COMPLETE |
						       HOST_SET_PSCI_MEM_PROTECT));
	}
	__host_stage2_set_owner_complete(PKVM_ID_HOST, 0);
	goto unlock;
}

int __pkvm_host_share_guest(u64 pfn, u64 gfn, u64 nr_pages, struct pkvm_hyp_vcpu *vcpu,
			    enum kvm_pgtable_prot prot)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	u64 phys = hyp_pfn_to_phys(pfn);
	u64 ipa = hyp_pfn_to_phys(gfn);
	u64 size;
	int ret;

	if (prot & ~KVM_PGTABLE_PROT_RWX)
		return -EINVAL;

	if (!pfn_range_is_valid(pfn, nr_pages))
		return -EINVAL;

	ret = __guest_check_transition_size(phys, ipa, nr_pages, &size);
	if (ret)
		return ret;

	if (phys >= phys + size || ipa >= ipa + size)
		return -EINVAL;

	ret = check_range_allowed_memory(phys, phys + size);
	if (ret)
		return ret;

	host_lock_component();
	guest_lock_component(vm);

	ret = __guest_check_page_state_range(vm, ipa, size, PKVM_NOPAGE);
	if (ret)
		goto unlock;

	for_each_hyp_page(page, phys, size) {
		switch (get_host_state(page)) {
		case PKVM_PAGE_OWNED:
			continue;
		case PKVM_PAGE_SHARED_OWNED:
			if (page->host_share_guest_count == U32_MAX) {
				ret = -EBUSY;
				goto unlock;
			}

			/* Only host to np-guest multi-sharing is tolerated */
			if (page->host_share_guest_count)
				continue;

			fallthrough;
		default:
			ret = -EPERM;
			goto unlock;
		}
	}

	for_each_hyp_page(page, phys, size) {
		set_host_state(page, PKVM_PAGE_SHARED_OWNED);
		page->host_share_guest_count++;
	}

	WARN_ON(kvm_pgtable_stage2_map(&vm->pgt, ipa, size, phys,
				       pkvm_mkstate(prot, PKVM_PAGE_SHARED_BORROWED),
				       &vcpu->vcpu.arch.stage2_mc, 0));

unlock:
	guest_unlock_component(vm);
	host_unlock_component();

	return ret;
}

static int __check_host_shared_guest(struct pkvm_hyp_vm *vm, u64 *__phys, u64 ipa, u64 size)
{
	enum pkvm_page_state state;
	kvm_pte_t pte;
	u64 phys;
	s8 level;
	int ret;

	ret = kvm_pgtable_get_leaf(&vm->pgt, ipa, &pte, &level);
	if (ret)
		return ret;
	if (!kvm_pte_valid(pte))
		return -ENOENT;
	if (size && kvm_granule_size(level) != size)
		return -E2BIG;

	if (!size)
		size = kvm_granule_size(level);

	state = guest_get_page_state(pte, ipa) & ~PKVM_PAGE_RESTRICTED_PROT;
	if (state != PKVM_PAGE_SHARED_BORROWED)
		return -EPERM;

	phys = kvm_pte_to_phys(pte);
	if (phys >= phys + size)
		return -EINVAL;

	ret = check_range_allowed_memory(phys, phys + size);
	if (WARN_ON(ret))
		return ret;

	for_each_hyp_page(page, phys, size) {
		if (get_host_state(page) != PKVM_PAGE_SHARED_OWNED)
			return -EPERM;
		if (WARN_ON(!page->host_share_guest_count))
			return -EINVAL;
	}

	*__phys = phys;

	return 0;
}

int __pkvm_host_unshare_guest(u64 gfn, u64 nr_pages, struct pkvm_hyp_vm *vm)
{
	u64 ipa = hyp_pfn_to_phys(gfn);
	u64 size, phys;
	int ret;

	ret = __guest_check_transition_size(0, ipa, nr_pages, &size);
	if (ret)
		return ret;

	host_lock_component();
	guest_lock_component(vm);

	ret = __check_host_shared_guest(vm, &phys, ipa, size);
	if (ret)
		goto unlock;

	ret = kvm_pgtable_stage2_unmap(&vm->pgt, ipa, size);
	if (ret)
		goto unlock;

	for_each_hyp_page(page, phys, size) {
		/* __check_host_shared_guest() protects against underflow */
		page->host_share_guest_count--;
		if (!page->host_share_guest_count)
			set_host_state(page, PKVM_PAGE_OWNED);
	}

unlock:
	guest_unlock_component(vm);
	host_unlock_component();

	return ret;
}

static void assert_host_shared_guest(struct pkvm_hyp_vm *vm, u64 ipa, u64 size)
{
	u64 phys;
	int ret;

	if (!IS_ENABLED(CONFIG_PKVM_STRICT_CHECKS))
		return;

	host_lock_component();
	guest_lock_component(vm);

	ret = __check_host_shared_guest(vm, &phys, ipa, size);

	guest_unlock_component(vm);
	host_unlock_component();

	WARN_ON(ret && ret != -ENOENT);
}

int __pkvm_host_relax_perms_guest(u64 gfn, struct pkvm_hyp_vcpu *vcpu, enum kvm_pgtable_prot prot)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	u64 ipa = hyp_pfn_to_phys(gfn);
	int ret;

	if (pkvm_hyp_vm_is_protected(vm))
		return -EPERM;

	if (prot & ~KVM_PGTABLE_PROT_RWX)
		return -EINVAL;

	assert_host_shared_guest(vm, ipa, 0);
	guest_lock_component(vm);
	ret = kvm_pgtable_stage2_relax_perms(&vm->pgt, ipa, prot, 0);
	guest_unlock_component(vm);

	return ret;
}

int __pkvm_host_wrprotect_guest(u64 gfn, u64 nr_pages, struct pkvm_hyp_vm *vm)
{
	u64 size, ipa = hyp_pfn_to_phys(gfn);
	int ret;

	if (pkvm_hyp_vm_is_protected(vm))
		return -EPERM;

	ret = __guest_check_transition_size(0, ipa, nr_pages, &size);
	if (ret)
		return ret;

	assert_host_shared_guest(vm, ipa, size);
	guest_lock_component(vm);
	ret = kvm_pgtable_stage2_wrprotect(&vm->pgt, ipa, size);
	guest_unlock_component(vm);

	return ret;
}

int __pkvm_host_test_clear_young_guest(u64 gfn, u64 nr_pages, bool mkold, struct pkvm_hyp_vm *vm)
{
	u64 size, ipa = hyp_pfn_to_phys(gfn);
	int ret;

	if (pkvm_hyp_vm_is_protected(vm))
		return -EPERM;

	ret = __guest_check_transition_size(0, ipa, nr_pages, &size);
	if (ret)
		return ret;

	assert_host_shared_guest(vm, ipa, size);
	guest_lock_component(vm);
	ret = kvm_pgtable_stage2_test_clear_young(&vm->pgt, ipa, size, mkold);
	guest_unlock_component(vm);

	return ret;
}

int __pkvm_host_mkyoung_guest(u64 gfn, struct pkvm_hyp_vcpu *vcpu)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	u64 ipa = hyp_pfn_to_phys(gfn);

	if (pkvm_hyp_vm_is_protected(vm))
		return -EPERM;

	assert_host_shared_guest(vm, ipa, 0);
	guest_lock_component(vm);
	kvm_pgtable_stage2_mkyoung(&vm->pgt, ipa, 0);
	guest_unlock_component(vm);

	return 0;
}

int __pkvm_host_split_guest(u64 gfn, u64 size, struct pkvm_hyp_vcpu *vcpu)
{
	struct kvm_hyp_memcache *mc = &vcpu->vcpu.arch.stage2_mc;
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	u64 ipa = hyp_pfn_to_phys(gfn);
	int ret;

	if (size != PMD_SIZE)
		return -EINVAL;

	guest_lock_component(vm);

	/*
	 * stage2_split() already checks the existing mapping is valid and PMD-level.
	 * No other check is necessary.
	 */

	ret = kvm_pgtable_stage2_split(&vm->pgt, ipa, size, mc);

	guest_unlock_component(vm);

	return ret;
}

static bool __check_ioguard_page(struct pkvm_hyp_vcpu *hyp_vcpu, u64 ipa)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu);
	kvm_pte_t pte;
	s8 level;
	int ret;

	ret = kvm_pgtable_get_leaf(&vm->pgt, ipa, &pte, &level);
	if (ret)
		return false;

	/* Must be a PAGE_SIZE mapping with our annotation */
	return (BIT(ARM64_HW_PGTABLE_LEVEL_SHIFT(level)) == PAGE_SIZE &&
		pte == KVM_INVALID_PTE_MMIO_NOTE);
}

static int guest_request_ioguard_walker(const struct kvm_pgtable_visit_ctx *ctx,
					enum kvm_pgtable_walk_flags visit)
{

	struct guest_request_walker_data *data = (struct guest_request_walker_data *)ctx->arg;
	enum pkvm_page_state state;
	kvm_pte_t pte = *ctx->ptep;
	u64 granule_size;

	state = guest_get_page_state(pte, 0) & ~PKVM_MMIO;
	if (state != PKVM_NOPAGE)
		return -EPERM;

	granule_size = kvm_granule_size(ctx->level);

	/* First PTE */
	if (!data->size)
		data->ipa_start = ctx->addr & ~(granule_size - 1);

	data->size += granule_size;

	return --data->max_ptes > 0 ? 0 : -ERANGE;
}

int __pkvm_install_ioguard_page(struct pkvm_hyp_vcpu *hyp_vcpu, u64 ipa,
				u64 nr_pages, u64 *nr_guarded)
{
	struct guest_request_walker_data data = GUEST_WALKER_DATA_INIT(PKVM_NOPAGE);
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu);
	struct kvm_pgtable_walker walker = {
		.cb     = guest_request_ioguard_walker,
		.flags  = KVM_PGTABLE_WALK_LEAF,
		.arg    = (void *)&data,
	};
	u64 end;
	int ret;

	if (!test_bit(KVM_ARCH_FLAG_MMIO_GUARD, &vm->kvm.arch.flags))
		return -EINVAL;

	if (!PAGE_ALIGNED(ipa))
		return -EINVAL;

	guest_lock_component(vm);

	ret = kvm_pgtable_walk(&vm->pgt, ipa, nr_pages << PAGE_SHIFT, &walker);
	/* Walker reached data.max_ptes */
	if (ret == -ERANGE)
		ret = 0;
	else if (ret)
		goto unlock;

	/* Intersection between the requested region and what has been verified */
	end = min(ipa + (nr_pages << PAGE_SHIFT), data.ipa_start + data.size);
	if (ipa >= end) {
		ret = -EINVAL;
		goto unlock;
	}

	*nr_guarded = (end - ipa) >> PAGE_SHIFT;
	ret = kvm_pgtable_stage2_annotate(&vm->pgt, ipa, end - ipa,
					  &hyp_vcpu->vcpu.arch.stage2_mc,
					  KVM_INVALID_PTE_MMIO_NOTE);

unlock:
	guest_unlock_component(vm);
	return ret;
}

bool __pkvm_check_ioguard_page(struct pkvm_hyp_vcpu *hyp_vcpu)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu);
	u64 ipa, end;
	bool ret;

	if (!kvm_vcpu_dabt_isvalid(&hyp_vcpu->vcpu))
		return false;

	if (!test_bit(KVM_ARCH_FLAG_MMIO_GUARD, &vm->kvm.arch.flags))
		return true;

	ipa  = kvm_vcpu_get_fault_ipa(&hyp_vcpu->vcpu);
	ipa |= FAR_TO_FIPA_OFFSET(kvm_vcpu_get_hfar(&hyp_vcpu->vcpu));
	end = ipa + kvm_vcpu_dabt_get_as(&hyp_vcpu->vcpu) - 1;

	guest_lock_component(vm);
	ret = __check_ioguard_page(hyp_vcpu, ipa);
	if ((end & PAGE_MASK) != (ipa & PAGE_MASK))
		ret &= __check_ioguard_page(hyp_vcpu, end);
	guest_unlock_component(vm);

	return ret;
}

static int __pkvm_remove_ioguard_page(struct pkvm_hyp_vm *vm, u64 ipa)
{
	int ret;
	kvm_pte_t pte;
	s8 level;

	hyp_assert_lock_held(&vm->pgtable_lock);

	if (!test_bit(KVM_ARCH_FLAG_MMIO_GUARD, &vm->kvm.arch.flags))
		return -EINVAL;

	if (!PAGE_ALIGNED(ipa))
		return -EINVAL;

	ret = kvm_pgtable_get_leaf(&vm->pgt, ipa, &pte, &level);
	if (ret)
		return ret;

	if (BIT(ARM64_HW_PGTABLE_LEVEL_SHIFT(level)) == PAGE_SIZE &&
	    pte == KVM_INVALID_PTE_MMIO_NOTE)
		return kvm_pgtable_stage2_unmap(&vm->pgt, ipa, PAGE_SIZE);

	return kvm_pte_valid(pte) ? -EEXIST : -EINVAL;
}

int __pkvm_install_guest_mmio(struct pkvm_hyp_vcpu *hyp_vcpu, u64 pfn, u64 gfn)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu);
	u64 ipa = gfn << PAGE_SHIFT;
	int ret;

	hyp_lock_component();
	guest_lock_component(vm);
	ret = __pkvm_remove_ioguard_page(vm, ipa);
	if (ret)
		goto out_unlock;
	ret = pkvm_hyp_donate_guest(hyp_vcpu, pfn, gfn);
out_unlock:
	guest_unlock_component(vm);
	hyp_unlock_component();
	return ret;
}

static int guest_annot_module_prot_walker(const struct kvm_pgtable_visit_ctx *ctx,
					  enum kvm_pgtable_walk_flags visit)
{
	kvm_pte_t pte = *ctx->ptep;

	if (!pte || pte == KVM_ACCEPT_MODULE_PROT_NOTE)
		return 0;

	return -EBUSY;
}

int __pkvm_accept_module_prot_page(u64 ipa, u64 nr_pages)
{
	struct pkvm_hyp_vcpu *vcpu;
	struct pkvm_hyp_vm *vm;
	struct kvm_pgtable_walker walker = {
		.cb     = guest_annot_module_prot_walker,
		.flags  = KVM_PGTABLE_WALK_LEAF,
	};
	int ret;

	if (!PAGE_ALIGNED(ipa))
		return -EINVAL;

	vcpu = pkvm_get_loaded_hyp_vcpu();
	if (!vcpu || !pkvm_hyp_vcpu_is_protected(vcpu))
		return -EINVAL;

	vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);
	guest_lock_component(vm);

	ret = kvm_pgtable_walk(&vm->pgt, ipa, nr_pages << PAGE_SHIFT, &walker);
	if (ret)
		goto unlock;

	ret = kvm_pgtable_stage2_annotate(&vm->pgt, ipa, nr_pages << PAGE_SHIFT,
					  &vcpu->vcpu.arch.stage2_mc,
					  KVM_ACCEPT_MODULE_PROT_NOTE);

unlock:
	guest_unlock_component(vm);
	return ret;
}

int host_stage2_get_leaf(phys_addr_t phys, kvm_pte_t *ptep, s8 *level)
{
	int ret;

	host_lock_component();
	ret = kvm_pgtable_get_leaf(&host_mmu.pgt, phys, ptep, level);
	host_unlock_component();

	return ret;
}

#ifdef CONFIG_NVHE_EL2_DEBUG
struct pkvm_expected_state {
	enum pkvm_page_state host;
	enum pkvm_page_state hyp;
	enum pkvm_page_state guest[2]; /* [ gfn, gfn + 1 ] */
};

static struct pkvm_expected_state selftest_state;
static struct hyp_page *selftest_page;

static struct pkvm_hyp_vm selftest_vm = {
	.kvm = {
		.arch = {
			.mmu = {
				.arch = &selftest_vm.kvm.arch,
				.pgt = &selftest_vm.pgt,
			},
		},
	},
};

static struct pkvm_hyp_vcpu selftest_vcpu = {
	.vcpu = {
		.arch = {
			.hw_mmu = &selftest_vm.kvm.arch.mmu,
		},
		.kvm = &selftest_vm.kvm,
	},
};

static void init_selftest_vm(void *virt)
{
	struct hyp_page *p = hyp_virt_to_page(virt);
	int i;

	selftest_vm.kvm.arch.mmu.vtcr = host_mmu.arch.mmu.vtcr;
	WARN_ON(kvm_guest_prepare_stage2(&selftest_vm, virt, 0));

	for (i = 0; i < pkvm_selftest_pages(); i++) {
		if (p[i].refcount)
			continue;
		p[i].refcount = 1;
		hyp_put_page(&selftest_vm.pool, hyp_page_to_virt(&p[i]));
	}
}

static u64 selftest_ipa(void)
{
	return BIT(selftest_vm.pgt.ia_bits - 1);
}

static void assert_page_state(void)
{
	void *virt = hyp_page_to_virt(selftest_page);
	u64 size = PAGE_SIZE << selftest_page->order;
	struct pkvm_hyp_vcpu *vcpu = &selftest_vcpu;
	u64 phys = hyp_virt_to_phys(virt);
	u64 ipa[2] = { selftest_ipa(), selftest_ipa() + PAGE_SIZE };
	struct pkvm_hyp_vm *vm;

	vm = pkvm_hyp_vcpu_to_hyp_vm(vcpu);

	host_lock_component();
	WARN_ON(__host_check_page_state_range(phys, size, selftest_state.host));
	host_unlock_component();

	hyp_lock_component();
	WARN_ON(__hyp_check_page_state_range(phys, size, selftest_state.hyp));
	hyp_unlock_component();

	guest_lock_component(&selftest_vm);
	WARN_ON(__guest_check_page_state_range(vm, ipa[0], size, selftest_state.guest[0]));
	WARN_ON(__guest_check_page_state_range(vm, ipa[1], size, selftest_state.guest[1]));
	guest_unlock_component(&selftest_vm);
}

#define assert_transition_res(res, fn, ...)		\
	do {						\
		WARN_ON(fn(__VA_ARGS__) != res);	\
		assert_page_state();			\
	} while (0)

void pkvm_ownership_selftest(void *base)
{
	enum kvm_pgtable_prot prot = KVM_PGTABLE_PROT_RWX;
	void *virt = hyp_alloc_pages(&host_s2_pool, 0);
	struct pkvm_hyp_vcpu *vcpu = &selftest_vcpu;
	struct pkvm_hyp_vm *vm = &selftest_vm;
	u64 phys, size, pfn, gfn;

	WARN_ON(!virt);
	selftest_page = hyp_virt_to_page(virt);
	selftest_page->refcount = 0;
	init_selftest_vm(base);

	size = PAGE_SIZE << selftest_page->order;
	phys = hyp_virt_to_phys(virt);
	pfn = hyp_phys_to_pfn(phys);
	gfn = hyp_phys_to_pfn(selftest_ipa());

	selftest_state.host = PKVM_NOPAGE;
	selftest_state.hyp = PKVM_PAGE_OWNED;
	selftest_state.guest[0] = selftest_state.guest[1] = PKVM_NOPAGE;
	assert_page_state();
	assert_transition_res(-EPERM,	__pkvm_host_donate_hyp, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_share_hyp, pfn);
	assert_transition_res(-EPERM,	__pkvm_host_unshare_hyp, pfn);
	assert_transition_res(-EPERM,	__pkvm_host_share_ffa, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_unshare_ffa, pfn, 1);
	assert_transition_res(-EPERM,	hyp_pin_shared_mem, virt, virt + size);
	assert_transition_res(-EPERM,	__pkvm_host_share_guest, pfn, gfn, 1, vcpu, prot);
	assert_transition_res(-ENOENT,	__pkvm_host_unshare_guest, gfn, 1, vm);

	selftest_state.host = PKVM_PAGE_OWNED;
	selftest_state.hyp = PKVM_NOPAGE;
	assert_transition_res(0,	__pkvm_hyp_donate_host, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_hyp_donate_host, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_unshare_hyp, pfn);
	assert_transition_res(-EPERM,	__pkvm_host_unshare_ffa, pfn, 1);
	assert_transition_res(-ENOENT,	__pkvm_host_unshare_guest, gfn, 1, vm);
	assert_transition_res(-EPERM,	hyp_pin_shared_mem, virt, virt + size);

	selftest_state.host = PKVM_PAGE_SHARED_OWNED;
	selftest_state.hyp = PKVM_PAGE_SHARED_BORROWED;
	assert_transition_res(0,	__pkvm_host_share_hyp, pfn);
	assert_transition_res(-EPERM,	__pkvm_host_share_hyp, pfn);
	assert_transition_res(-EPERM,	__pkvm_host_donate_hyp, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_share_ffa, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_hyp_donate_host, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_share_guest, pfn, gfn, 1, vcpu, prot);
	assert_transition_res(-ENOENT,	__pkvm_host_unshare_guest, gfn, 1, vm);

	assert_transition_res(0,	hyp_pin_shared_mem, virt, virt + size);
	assert_transition_res(0,	hyp_pin_shared_mem, virt, virt + size);
	hyp_unpin_shared_mem(virt, virt + size);
	WARN_ON(!hyp_page_referenced(virt));
	assert_transition_res(-EBUSY,	__pkvm_host_unshare_hyp, pfn);
	assert_transition_res(-EPERM,	__pkvm_host_share_hyp, pfn);
	assert_transition_res(-EPERM,	__pkvm_host_donate_hyp, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_share_ffa, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_hyp_donate_host, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_share_guest, pfn, gfn, 1, vcpu, prot);
	assert_transition_res(-ENOENT,	__pkvm_host_unshare_guest, gfn, 1, vm);

	hyp_unpin_shared_mem(virt, virt + size);
	assert_page_state();
	WARN_ON(hyp_page_referenced(virt));

	selftest_state.host = PKVM_PAGE_OWNED;
	selftest_state.hyp = PKVM_NOPAGE;
	assert_transition_res(0,	__pkvm_host_unshare_hyp, pfn);

	selftest_state.host = PKVM_PAGE_SHARED_OWNED;
	selftest_state.hyp = PKVM_NOPAGE;
	assert_transition_res(0,	__pkvm_host_share_ffa, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_share_ffa, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_donate_hyp, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_share_hyp, pfn);
	assert_transition_res(-EPERM,	__pkvm_host_unshare_hyp, pfn);
	assert_transition_res(-EPERM,	__pkvm_hyp_donate_host, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_share_guest, pfn, gfn, 1, vcpu, prot);
	assert_transition_res(-ENOENT,	__pkvm_host_unshare_guest, gfn, 1, vm);
	assert_transition_res(-EPERM,	hyp_pin_shared_mem, virt, virt + size);

	selftest_state.host = PKVM_PAGE_OWNED;
	selftest_state.hyp = PKVM_NOPAGE;
	assert_transition_res(0,	__pkvm_host_unshare_ffa, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_unshare_ffa, pfn, 1);

	selftest_state.host = PKVM_PAGE_SHARED_OWNED;
	selftest_state.guest[0] = PKVM_PAGE_SHARED_BORROWED;
	assert_transition_res(0,	__pkvm_host_share_guest, pfn, gfn, 1, vcpu, prot);
	assert_transition_res(-EPERM,	__pkvm_host_share_guest, pfn, gfn, 1, vcpu, prot);
	assert_transition_res(-EPERM,	__pkvm_host_share_ffa, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_donate_hyp, pfn, 1);
	assert_transition_res(-EPERM,	__pkvm_host_share_hyp, pfn);
	assert_transition_res(-EPERM,	__pkvm_host_unshare_hyp, pfn);
	assert_transition_res(-EPERM,	__pkvm_hyp_donate_host, pfn, 1);
	assert_transition_res(-EPERM,	hyp_pin_shared_mem, virt, virt + size);

	selftest_state.guest[1] = PKVM_PAGE_SHARED_BORROWED;
	assert_transition_res(0,	__pkvm_host_share_guest, pfn, gfn + 1, 1, vcpu, prot);
	WARN_ON(hyp_virt_to_page(virt)->host_share_guest_count != 2);

	selftest_state.guest[0] = PKVM_NOPAGE;
	assert_transition_res(0,	__pkvm_host_unshare_guest, gfn, 1, vm);

	selftest_state.guest[1] = PKVM_NOPAGE;
	selftest_state.host = PKVM_PAGE_OWNED;
	assert_transition_res(0,	__pkvm_host_unshare_guest, gfn + 1, 1, vm);

	selftest_state.host = PKVM_NOPAGE;
	selftest_state.hyp = PKVM_PAGE_OWNED;
	assert_transition_res(0,	__pkvm_host_donate_hyp, pfn, 1);

	selftest_page->refcount = 1;
	hyp_put_page(&host_s2_pool, virt);
}
#endif

static u64 __pkvm_ptdump_get_host_config(enum pkvm_ptdump_ops op)
{
	u64 ret = 0;

	host_lock_component();
	if (op == PKVM_PTDUMP_GET_LEVEL)
		ret = host_mmu.pgt.start_level;
	else
		ret = host_mmu.pgt.ia_bits;
	host_unlock_component();

	return ret;
}

static u64 __pkvm_ptdump_get_guest_config(pkvm_handle_t handle, enum pkvm_ptdump_ops op)
{
	struct pkvm_hyp_vm *vm;
	u64 ret = 0;

	vm = get_pkvm_hyp_vm(handle);
	if (!vm)
		return -EINVAL;

	if (op == PKVM_PTDUMP_GET_LEVEL)
		ret = vm->pgt.start_level;
	else
		ret = vm->pgt.ia_bits;

	put_pkvm_hyp_vm(vm);
	return ret;
}

u64 __pkvm_ptdump_get_config(pkvm_handle_t handle, enum pkvm_ptdump_ops op)
{
	if (!handle)
		return __pkvm_ptdump_get_host_config(op);

	return __pkvm_ptdump_get_guest_config(handle, op);
}

static int pkvm_ptdump_walker(const struct kvm_pgtable_visit_ctx *ctx,
			      enum kvm_pgtable_walk_flags visit)
{
	struct pkvm_ptdump_log_hdr **log_hdr = ctx->arg;
	ssize_t avail_space = PAGE_SIZE - (*log_hdr)->w_index - sizeof(struct pkvm_ptdump_log_hdr);
	struct pkvm_ptdump_log *log;

	if (avail_space < sizeof(struct pkvm_ptdump_log)) {
		if ((*log_hdr)->pfn_next == INVALID_PTDUMP_PFN)
			return -ENOMEM;

		*log_hdr = hyp_phys_to_virt(hyp_pfn_to_phys((*log_hdr)->pfn_next));
		WARN_ON((*log_hdr)->w_index);
	}

	log = (struct pkvm_ptdump_log *)((void *)*log_hdr + (*log_hdr)->w_index +
					 sizeof(struct pkvm_ptdump_log_hdr));
	log->pfn = ctx->addr >> PAGE_SHIFT;
	log->valid = ctx->old & PTE_VALID;
	log->r = FIELD_GET(KVM_PTE_LEAF_ATTR_LO_S2_S2AP_R, ctx->old);
	log->w = FIELD_GET(KVM_PTE_LEAF_ATTR_LO_S2_S2AP_W, ctx->old);
	log->xn = FIELD_GET(KVM_PTE_LEAF_ATTR_HI_S2_XN, ctx->old);
	log->table = FIELD_GET(KVM_PTE_TYPE, ctx->old);
	log->level = ctx->level;
	log->page_state = FIELD_GET(PKVM_PAGE_STATE_PROT_MASK, ctx->old);
	log->mmio_guard = FIELD_GET(KVM_INVALID_PTE_MMIO_NOTE, ctx->old);

	(*log_hdr)->w_index += sizeof(struct pkvm_ptdump_log);
	return 0;
}

static void pkvm_ptdump_teardown_log(struct pkvm_ptdump_log_hdr *log_hva,
				     struct pkvm_ptdump_log_hdr *cur)
{
	struct pkvm_ptdump_log_hdr *tmp, *log = (void *)kern_hyp_va(log_hva);
	bool next_log_invalid = false;

	while (log != cur && !next_log_invalid) {
		next_log_invalid = log->pfn_next == INVALID_PTDUMP_PFN;
		tmp = hyp_phys_to_virt(hyp_pfn_to_phys(log->pfn_next));
		WARN_ON(__pkvm_hyp_donate_host(hyp_virt_to_pfn(log), 1));
		log = tmp;
	}
}

static int pkvm_ptdump_setup_log(struct pkvm_ptdump_log_hdr *log_hva)
{
	int ret;
	struct pkvm_ptdump_log_hdr *log = (void *)kern_hyp_va(log_hva);

	if (!PAGE_ALIGNED(log))
		return -EINVAL;

	for (;;) {
		ret = __pkvm_host_donate_hyp(hyp_virt_to_pfn(log), 1);
		if (ret) {
			pkvm_ptdump_teardown_log(log_hva, log);
			return ret;
		}

		log->w_index = 0;
		if (log->pfn_next == INVALID_PTDUMP_PFN)
			break;

		log = hyp_phys_to_virt(hyp_pfn_to_phys(log->pfn_next));
	}

	return 0;
}

static int pkvm_ptdump_walk_host(struct kvm_pgtable_walker *walker)
{
	int ret;

	host_lock_component();
	ret = kvm_pgtable_walk(&host_mmu.pgt, 0, BIT(host_mmu.pgt.ia_bits), walker);
	host_unlock_component();

	return ret;
}

static int pkvm_ptdump_walk_guest(struct pkvm_hyp_vm *vm, struct kvm_pgtable_walker *walker)
{
	int ret;

	guest_lock_component(vm);

	ret = kvm_pgtable_walk(&vm->pgt, 0, BIT(vm->pgt.ia_bits), walker);

	guest_unlock_component(vm);

	return ret;
}

u64 __pkvm_ptdump_walk_range(pkvm_handle_t handle, struct pkvm_ptdump_log_hdr *log)
{
	struct pkvm_hyp_vm *vm;
	int ret;
	struct pkvm_ptdump_log_hdr *log_hyp = kern_hyp_va(log);
	struct kvm_pgtable_walker walker = {
		.cb     = pkvm_ptdump_walker,
		.flags  = KVM_PGTABLE_WALK_LEAF,
		.arg    = &log_hyp,
	};

	ret = pkvm_ptdump_setup_log(log);
	if (ret)
		return ret;

	if (!handle)
		ret = pkvm_ptdump_walk_host(&walker);
	else {
		vm = get_pkvm_hyp_vm(handle);
		if (!vm) {
			ret = -EINVAL;
			goto teardown;
		}

		ret = pkvm_ptdump_walk_guest(vm, &walker);
		put_pkvm_hyp_vm(vm);
	}
teardown:
	pkvm_ptdump_teardown_log(log, NULL);
	return ret;
}

static void __pkvm_use_dma_page(phys_addr_t phys)
{
	struct hyp_page *p = hyp_phys_to_page(phys);

	hyp_page_ref_inc(p);
}

static void __pkvm_unuse_dma_page(phys_addr_t phys)
{
	struct hyp_page *p = hyp_phys_to_page(phys);

	hyp_page_ref_dec(p);
}

static int __pkvm_use_dma_locked(phys_addr_t phys, size_t size, struct pkvm_hyp_vm *vm)
{
	int i;
	int ret = 0;
	struct kvm_mem_range r;
	size_t nr_pages = size >> PAGE_SHIFT;
	struct memblock_region *reg = find_mem_range(phys, &r);

	if (!pfn_range_is_valid(hyp_phys_to_pfn(phys), nr_pages) ||
	    !is_in_mem_range(phys + size - 1, &r))
		return -EINVAL;
	/*
	 * Some differences between handling of RAM and device memory:
	 * - The hyp vmemmap area for device memory is not backed by physical
	 *   pages in the hyp page tables.
	 * - However, in some cases modules can donate MMIO, as they can't be
	 *   refcounted, taint them by marking them as shared borrowed, and that
	 *   will prevent any future transition.
	 */
	if (!reg) {
		enum kvm_pgtable_prot prot;

		if (vm)
			return -EINVAL;
		for (i = 0; i < nr_pages; i++) {
			u64 addr = phys + i * PAGE_SIZE;

			ret = ___host_check_page_state_range(addr, PAGE_SIZE,
							     PKVM_PAGE_SHARED_BORROWED, 0);
			/* Page already borrowed */
			if (!ret)
				continue;
			ret = ___host_check_page_state_range(addr, PAGE_SIZE,
							     PKVM_PAGE_OWNED, 0);
			if (ret)
				return ret;
		}
		prot = pkvm_mkstate(PKVM_HOST_MMIO_PROT, PKVM_PAGE_SHARED_BORROWED);
		WARN_ON(host_stage2_idmap_locked(phys, size, prot, false));
	} else {

		/* For VMs, we know if we reach this point the VM has access to the page. */
		if (!vm) {
			for_each_hyp_page(page, phys, size) {
				if (get_host_state(page) != PKVM_PAGE_OWNED)
					return -EPERM;
			}
		}

		for (i = 0; i < nr_pages; i++)
			__pkvm_use_dma_page(phys + i * PAGE_SIZE);
	}

	return ret;
}

/*
 * __pkvm_use_dma - Mark memory as used for DMA
 * @phys:	physical address of the DMA region
 * @size:	size of the DMA region
 * When a page is mapped in an IOMMU page table for DMA, it must
 * not be donated to a guest or the hypervisor we ensure this with:
 * - Host can only map pages that are OWNED
 * - Any page that is mapped is refcounted
 * - Donation/Sharing is prevented if a page is refcounted.
 * - Any MMIO ever mapped in the IOMMU can't be donated/shared.
 * In case in the future shared pages are allowed to be mapped,
 * similar checks are needed in host_request_unshare() and
 * host_ack_unshare()
 */
int __pkvm_use_dma(phys_addr_t phys, size_t size, struct pkvm_hyp_vm *vm)
{
	int ret;

	host_lock_component();
	ret = __pkvm_use_dma_locked(phys, size, vm);
	host_unlock_component();
	return ret;
}

/*
 * Must be called after a __pkvm_host_use_dma() for the same
 * range, typically after a page was unmapped from an IOMMU.
 */
int __pkvm_unuse_dma(phys_addr_t phys, size_t size, struct pkvm_hyp_vm *vm)
{
	int i;
	size_t nr_pages = size >> PAGE_SHIFT;

	if (!pfn_range_is_valid(hyp_phys_to_pfn(phys), nr_pages))
		return -EINVAL;

	if (!range_is_memory(phys, phys + size)) {
		WARN_ON(vm);
		return 0;
	}
	host_lock_component();

	for (i = 0; i < nr_pages; i++)
		__pkvm_unuse_dma_page(phys + i * PAGE_SIZE);

	host_unlock_component();
	return 0;
}

/*
 * Get a PA of an IPA and pin the memory starting from this PA till
 *  the requested ipa_size or the end of the page boundary returned in level.
 */
int pkvm_get_guest_pa_request_use_dma(struct pkvm_hyp_vcpu *hyp_vcpu, u64 ipa,
                                     size_t ipa_size, u64 *out_pa, s8 *level)
{
	struct pkvm_hyp_vm *vm = pkvm_hyp_vcpu_to_hyp_vm(hyp_vcpu);
	size_t off;
	int ret;

	host_lock_component();
	ret = pkvm_get_guest_pa_request(hyp_vcpu, ipa, ipa_size,
					out_pa, level);
	if (ret)
		goto out_ret;
	off = *out_pa - ALIGN_DOWN(*out_pa, kvm_granule_size(*level));
	WARN_ON(__pkvm_use_dma_locked(*out_pa,
				      min(kvm_granule_size(*level) - off, ipa_size), vm));
out_ret:
	host_unlock_component();
	return ret;
}
