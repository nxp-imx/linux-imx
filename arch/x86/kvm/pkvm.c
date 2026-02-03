// SPDX-License-Identifier: GPL-2.0
#include <linux/kvm_host.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/sort.h>
#include <asm/kvm_pkvm.h>

bool __read_mostly enable_pkvm;

static struct memblock_region *pkvm_memory = pkvm_sym(pkvm_memory);
static unsigned int pkvm_memblock_nr;

phys_addr_t pkvm_mem_base;
phys_addr_t pkvm_mem_size;

static int cmp_pkvm_memblock(const void *p1, const void *p2)
{
	const struct memblock_region *r1 = p1;
	const struct memblock_region *r2 = p2;

	return r1->base < r2->base ? -1 : (r1->base > r2->base);
}

static void __init sort_memblock_regions(void)
{
	sort(pkvm_memory,
	     pkvm_memblock_nr,
	     sizeof(struct memblock_region),
	     cmp_pkvm_memblock,
	     NULL);
}

static int __init register_memblock_regions(void)
{
	struct memblock_region *reg;

	for_each_mem_region(reg) {
		if (pkvm_memblock_nr >= PKVM_MEMBLOCK_REGIONS)
			return -ENOMEM;

		pkvm_memory[pkvm_memblock_nr] = *reg;
		pkvm_memblock_nr++;
	}
	sort_memblock_regions();
	pkvm_sym(pkvm_memblock_nr) = pkvm_memblock_nr;

	return 0;
}

void __init pkvm_reserve(void)
{
	int ret;

	if (!enable_pkvm)
		return;

	ret = register_memblock_regions();
	if (ret) {
		pkvm_memblock_nr = 0;
		kvm_err("Failed to register pkvm memblocks: %d\n", ret);
		return;
	}

	/*
	 * Try to allocate a PMD-aligned region to reduce TLB pressure once
	 * this is unmapped from the host stage-2, and fallback to PAGE_SIZE.
	 */
	pkvm_mem_size = pkvm_total_reserve_pages() << PAGE_SHIFT;
	pkvm_mem_base = memblock_phys_alloc(ALIGN(pkvm_mem_size, PMD_SIZE),
					   PMD_SIZE);
	if (!pkvm_mem_base)
		pkvm_mem_base = memblock_phys_alloc(pkvm_mem_size, PAGE_SIZE);
	else
		pkvm_mem_size = ALIGN(pkvm_mem_size, PMD_SIZE);

	if (!pkvm_mem_base) {
		kvm_err("Failed to reserve pkvm memory\n");
		return;
	}

	kvm_info("Reserved %lld MiB at 0x%llx for pkvm\n", pkvm_mem_size >> 20,
		 pkvm_mem_base);
}

static void *kvm_host_va(phys_addr_t phys)
{
	return __va(phys);
}

static void kvm_free_pkvm_page_range(struct pkvm_page_range range)
{
	void *vaddr = __va(range.addr);
	u64 nr_pages = range.nr_pages;

	if (WARN_ON_ONCE(!nr_pages))
		return;

	if (nr_pages > 1)
		free_pages_exact(vaddr, nr_pages << PAGE_SHIFT);
	else
		free_page((unsigned long)vaddr);
}

void kvm_free_pkvm_memcache(struct pkvm_memcache *mc)
{
	free_pkvm_memcache(mc, kvm_free_pkvm_page_range, kvm_host_va);
}
