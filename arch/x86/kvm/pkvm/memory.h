/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __PKVM_X86_MEMORY_H
#define __PKVM_X86_MEMORY_H

#include <linux/types.h>
#include <linux/range.h>
#include <linux/kvm_types.h>
#include <linux/mm.h>
#include <vdso/limits.h>
#include <asm/page.h>
#include "mem_protect.h"

#define __pkvm_pa		__pa
#define __pkvm_va		__va

struct pkvm_page {
	unsigned short refcount;
	u8 order;

	/* Store host memory page state. */
	enum pkvm_page_state host_state: 8;

	/* Tracks how many times the page is shared with pKVM. */
	u16 host_share_hyp_count;
};

/*
 * Make sure pkvm_page->host_state is large enough to store enum
 * pkvm_page_state.
 */
static_assert(PKVM_PAGE_STATE_BITS <= 8);

extern u64 __pkvm_vmemmap;
#define pkvm_vmemmap ((struct pkvm_page *)__pkvm_vmemmap)

#define pkvm_phys_to_pfn(phys)	((phys) >> PAGE_SHIFT)
#define pkvm_pfn_to_phys(pfn)	((phys_addr_t)((pfn) << PAGE_SHIFT))
#define pkvm_phys_to_page(phys)	(&pkvm_vmemmap[pkvm_phys_to_pfn(phys)])
#define pkvm_virt_to_page(virt)	pkvm_phys_to_page(__pkvm_pa(virt))
#define pkvm_virt_to_pfn(virt)	pkvm_phys_to_pfn(__pkvm_pa(virt))

#define pkvm_page_to_pfn(page)	((struct pkvm_page *)(page) - pkvm_vmemmap)
#define pkvm_page_to_phys(page)  pkvm_pfn_to_phys((pkvm_page_to_pfn(page)))
#define pkvm_page_to_virt(page)	__pkvm_va(pkvm_page_to_phys(page))
#define pkvm_page_to_pool(page)	(((struct pkvm_page *)page)->pool)

/* Caution: __st is evaluated twice. */
#define for_each_pkvm_page(__p, __st, __sz)						\
	for (struct pkvm_page *__p = pkvm_phys_to_page(PAGE_ALIGN_DOWN(__st)),		\
			      *__e = pkvm_phys_to_page(PAGE_ALIGN((__st) + (__sz)));	\
	     __p < __e; __p++)

/*
 * Refcounting for 'struct pkvm_page'.
 * pkvm_pool::lock must be held if atomic access to the refcount is required.
 */
static inline int pkvm_page_count(void *addr)
{
	struct pkvm_page *p = pkvm_virt_to_page(addr);

	return p->refcount;
}

static inline void pkvm_page_ref_inc(struct pkvm_page *p)
{
	BUG_ON(p->refcount == USHRT_MAX);
	p->refcount++;
}

static inline void pkvm_page_ref_dec(struct pkvm_page *p)
{
	BUG_ON(!p->refcount);
	p->refcount--;
}

static inline int pkvm_page_ref_dec_and_test(struct pkvm_page *p)
{
	pkvm_page_ref_dec(p);
	return (p->refcount == 0);
}

static inline void pkvm_set_page_refcounted(struct pkvm_page *p)
{
	BUG_ON(p->refcount);
	p->refcount = 1;
}

bool pkvm_find_addr_range(unsigned long phys, struct range *range);

static inline bool is_memory_range(unsigned long phys, unsigned long size)
{
	struct range target = {
		.start = PAGE_ALIGN_DOWN(phys),
		.end = PAGE_ALIGN(phys + size) - 1,
	};
	struct range range;

	if (!pkvm_find_addr_range(phys, &range))
		return false;

	return range_contains(&range, &target);
}

/*
 * !is_memory_range doesn't mean the range is mmio as it is possible to overlap
 * with memory, e.g., a memory-mmio mixed range.
 */
static inline bool is_mmio_range(unsigned long phys, unsigned long size)
{
	struct range target = {
		.start = PAGE_ALIGN_DOWN(phys),
		.end = PAGE_ALIGN(phys + size) - 1,
	};
	struct range range;

	if (pkvm_find_addr_range(phys, &range))
		return false;

	return range_contains(&range, &target);
}

/*
 * Convert a linux host kernel direct mapping virtual address to a pKVM mapping
 * virtual address. Currently the two virtual addresses are the same.
 */
static inline void *kern_pkvm_va(void *va)
{
	return va;
}

void pkvm_clflush_cache_range(void *vaddr, unsigned int size);

static inline void pkvm_clear_memory(void *va, size_t size)
{
	memset(va, 0, size);
	/*
	 * Flush CPU cache to ensure clearing the memory range in RAM, so that
	 * the previous contents cannot be read via non-coherent DMA.
	 */
	pkvm_clflush_cache_range(va, size);
}

static inline phys_addr_t pkvm_host_gpa_to_phys(gpa_t gpa)
{
	/*
	 * Host VM's GPA is identify-mapped in the host mmu, thus GPA equals
	 * physical address.
	 */
	return gpa;
}

static inline gpa_t pkvm_phys_to_host_gpa(unsigned long phys)
{
	/* See comments in pkvm_host_gpa_to_phys */
	return phys;
}

static inline void *pkvm_host_gpa_to_virt(gpa_t gpa)
{
	return __pkvm_va(pkvm_host_gpa_to_phys(gpa));
}

static inline gpa_t pkvm_virt_to_host_gpa(void *addr)
{
	return pkvm_phys_to_host_gpa(__pkvm_pa(addr));
}

#endif /* __PKVM_X86_MEMORY_H */
