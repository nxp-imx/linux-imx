// SPDX-License-Identifier: GPL-2.0
#include <linux/errno.h>
#include <linux/kvm_host.h>
#include <vdso/page.h>
#include "memory.h"
#include "pgtable.h"

#define PGTABLE_WALK_DONE      1

static void *pgtable_alloc_page(const struct pkvm_pgtable_mm_ops *mm_ops,
				struct pkvm_memcache *mc)
{
	return mm_ops->zalloc_page(mc);
}

static bool leaf_in_addr_range(unsigned long leaf_size,
			       unsigned long addr,
			       unsigned long end)
{
	if (!IS_ALIGNED(addr, leaf_size))
		return false;

	if (leaf_size > (end - addr))
		return false;

	return true;
}

struct pgt_map_data {
	unsigned long phys;
	u64 prot;
	u64 annotation;
	struct pkvm_memcache *memcache;
};

static bool leaf_mapping_changed(struct pkvm_pgtable_visit_ctx *ctx,
				 struct pgt_map_data *data)
{
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	unsigned long leaf_size, mapped_phys, new_phys;

	if (VALID_PAGE(data->phys)) {
		/* Property bits are changed */
		if (data->prot != pgt_ops->pte_to_prot(ctx->ptep))
			return true;

		leaf_size = pgt_ops->level_to_size(ctx->level);
		mapped_phys = pgt_ops->pte_to_phys(ctx->ptep);
		new_phys = data->phys + (ctx->addr - ctx->start);
		/*
		 * The new physical address is not covered by the old mapped range
		 * [mapped_phys, mapped_phys + leaf_size).
		 */
		if (!((new_phys >= mapped_phys) && new_phys < (mapped_phys + leaf_size)))
			return true;
	} else if (data->annotation != ctx->pgt->pgt_ops->pte_get(ctx->ptep)) {
		return true;
	}

	return false;
}

static bool leaf_mapping_allowed(struct pkvm_pgtable_visit_ctx *ctx,
				 struct pgt_map_data *data)
{
	unsigned long leaf_size = ctx->pgt->pgt_ops->level_to_size(ctx->level);

	if (!leaf_in_addr_range(leaf_size, ctx->addr, ctx->end))
		return false;

	if (!((1 << ctx->level) & ctx->pgt->cap.allowed_pgsz))
		return false;

	if (!VALID_PAGE(data->phys))
		return true;

	return IS_ALIGNED(data->phys + (ctx->addr - ctx->start), leaf_size);
}

static void split_huge_pte(struct pkvm_pgtable_visit_ctx *ctx, void *child_ptep)
{
	const struct pkvm_pgtable_mm_ops *mm_ops = ctx->pgt->mm_ops;
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	u64 prot = pgt_ops->pte_to_prot(ctx->ptep);
	unsigned long phys, phys_end, step_size;
	int i = 0, entry_size, child_level;

	child_level = ctx->level - 1;
	if (child_level > PG_LEVEL_4K)
		pgt_ops->pte_mkhuge(&prot);

	phys = pgt_ops->pte_to_phys(ctx->ptep);
	phys_end = phys + pgt_ops->level_to_size(ctx->level);
	step_size = pgt_ops->level_to_size(child_level);
	entry_size = pgt_ops->pte_size(child_level);

	for (i = 0; phys < phys_end; phys += step_size, i++) {
		pgt_ops->pte_set(child_ptep + i * entry_size, phys | prot);
		mm_ops->get_page(child_ptep);
	}
}

static void split_annotated_pte(struct pkvm_pgtable_visit_ctx *ctx, void *child_ptep)
{
	const struct pkvm_pgtable_mm_ops *mm_ops = ctx->pgt->mm_ops;
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	int entry_size = pgt_ops->pte_size(ctx->level - 1);
	int entries = pgt_ops->pte_count(ctx->level);
	u64 annotation = pgt_ops->pte_get(ctx->ptep);
	int i;

	for (i = 0; i < entries; i++) {
		pgt_ops->pte_set((child_ptep + i * entry_size), annotation);
		mm_ops->get_page(child_ptep);
	}
}

static void pgtable_split(struct pkvm_pgtable_visit_ctx *ctx, void *child_ptep)
{
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;

	BUG_ON(ctx->level <= PG_LEVEL_4K);

	if (pgt_ops->pte_huge(ctx->ptep))
		split_huge_pte(ctx, child_ptep);
	else if (pgt_ops->pte_annotated(ctx->ptep))
		split_annotated_pte(ctx, child_ptep);
}

static int pgtable_try_map_leaf(struct pkvm_pgtable_visit_ctx *ctx,
				struct pgt_map_data *data)
{
	const struct pkvm_pgtable_mm_ops *mm_ops = ctx->pgt->mm_ops;
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	bool flush_tlb, was_counted, is_counted;
	void *ptep = ctx->ptep;
	u64 new, old;

	if (!leaf_mapping_changed(ctx, data))
		return 0;

	if (!leaf_mapping_allowed(ctx, data)) {
		/*
		 * 4K mapping should always be allowed, since the start and end
		 * addresses are aligned to PAGE_SIZE.
		 */
		BUG_ON(ctx->level == PG_LEVEL_4K);
		/*
		 * For the other levels, goes down to the next level to try
		 * if the mapping would be allowed by returning -E2BIG.
		 */
		return -E2BIG;
	}

	if (VALID_PAGE(data->phys)) {
		new = (data->phys + (ctx->addr - ctx->start)) | data->prot;
		old = pgt_ops->pte_to_phys(ptep) | pgt_ops->pte_to_prot(ptep);
		/*
		 * Need to flush TLB when the previous mapping was present and the new
		 * mapping changes either physical address or property bits. Otherwise
		 * no need to flush TLB.
		 *
		 * No need to consider for the page state as it sits on the pte ignored
		 * bits, which doesn't impact the TLB.
		 */
		if (pgt_ops->pte_present(ptep))
			flush_tlb = (new ^ old) & ~pkvm_pgt_pgstate_mask(ctx->pgt);
		else
			flush_tlb = false;

		if (ctx->level != PG_LEVEL_4K)
			pgt_ops->pte_mkhuge(&new);
	} else {
		new = data->annotation;
		/*
		 * Need to flush TLB if a present leaf is going to be removed
		 * by installing an annotation.
		 */
		flush_tlb = pgt_ops->pte_present(ptep);
	}

	was_counted = pgt_ops->pte_present(ptep) || pgt_ops->pte_annotated(ptep);
	pgt_ops->pte_set(ptep, new);
	is_counted = pgt_ops->pte_present(ptep) || pgt_ops->pte_annotated(ptep);

	/*
	 * Need to increment the page table page refcnt when installs a present
	 * leaf or an annotation if the refcnt was not already incremented by
	 * the previous one.
	 *
	 * Similarly need to decrement the page table page refcnt when removes a
	 * present leaf or annotation if the refcnt was incremented by the
	 * previous one.
	 */
	if (is_counted && !was_counted)
		mm_ops->get_page(ptep);
	else if (!is_counted && was_counted)
		mm_ops->put_page(ptep);

	if (flush_tlb) {
		if (ctx->pgt->cap.flush_tlb_lazy)
			ctx->flush_tlb |= true;
		else
			pgt_ops->flush_tlb(ctx->pgt, ctx->addr,
					   pgt_ops->level_to_size(ctx->level));
	}

	return 0;
}

static int __map_walker(struct pkvm_pgtable_visit_ctx *ctx,
			struct pgt_map_data *data)
{
	const struct pkvm_pgtable_mm_ops *mm_ops = ctx->pgt->mm_ops;
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	void *ptep = ctx->ptep;
	void *page;
	int ret;

	/* Try to create leaf page mapping on current level */
	ret = pgtable_try_map_leaf(ctx, data);
	if (ret != -E2BIG)
		return ret;

	/*
	 * The mapping needs be done at the next level or even smaller.
	 * Allocate a new page as the next level page table page.
	 */
	page = pgtable_alloc_page(mm_ops, data->memcache);
	if (!page)
		return -ENOMEM;

	/*
	 * If a huge mapping already exists at the current level, or the pte
	 * contains annotation, split it into smaller ones.
	 */
	if (pgt_ops->pte_huge(ptep) || pgt_ops->pte_annotated(ptep)) {
		/* Split doesn't change the translation so no need to flush tlb. */
		mm_ops->put_page(ptep);
		pgtable_split(ctx, page);
	}

	mm_ops->get_page(ptep);
	pgt_ops->pte_set(ptep, ctx->pgt->cap.table_prot | __pkvm_pa(page));

	return 0;
}

/* TODO: Support merging small entries into a huge entry. */
static int map_walker(struct pkvm_pgtable_visit_ctx *ctx, unsigned long walk_flags,
		      void *const arg)
{
	struct pgt_map_data *data = arg;

	switch (walk_flags) {
	case PKVM_PGTABLE_WALK_LEAF:
		return __map_walker(ctx, data);
	case PKVM_PGTABLE_WALK_TABLE_PRE:
	case PKVM_PGTABLE_WALK_TABLE_POST:
		break;
	}

	return -EINVAL;
}

struct pgt_unmap_data {
	unsigned long phys;
};

static int pgtable_try_unmap_leaf(struct pkvm_pgtable_visit_ctx *ctx,
				  struct pgt_unmap_data *data)
{
	const struct pkvm_pgtable_mm_ops *mm_ops = ctx->pgt->mm_ops;
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	void *ptep = ctx->ptep;
	unsigned long size;
	bool was_present;

	size = pgt_ops->level_to_size(ctx->level);
	/*
	 * Unmap the page if:
	 * - 4K PTE entry
	 * - a fully covered huge page
	 * Otherwise return -E2BIG to go to the next level.
	 */
	if (!(ctx->level == PG_LEVEL_4K ||
	      (pgt_ops->pte_huge(ptep) && leaf_in_addr_range(size, ctx->addr, ctx->end))))
		return -E2BIG;

	was_present = pgt_ops->pte_present(ptep);
	if (VALID_PAGE(data->phys) && was_present) {
		/*
		 * The user can request to check if the unmapped physical
		 * address is the desired memory page or not. Panic the
		 * hypervisor if unexpected result happens.
		 */
		BUG_ON(pgt_ops->pte_to_phys(ptep) != data->phys + (ctx->addr - ctx->start));
	}

	pgt_ops->pte_set(ptep, 0);
	mm_ops->put_page(ptep);

	/*
	 * Flush TLB if a present entry is changed to
	 * non-present.
	 */
	if (was_present) {
		if (ctx->pgt->cap.flush_tlb_lazy)
			ctx->flush_tlb |= true;
		else
			pgt_ops->flush_tlb(ctx->pgt, ctx->addr, size);
	}

	return 0;
}

static void pgtable_free_child(struct pkvm_pgtable_visit_ctx *ctx)
{
	const struct pkvm_pgtable_mm_ops *mm_ops = ctx->pgt->mm_ops;
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	void *child, *ptep = ctx->ptep;

	/*
	 * Check the child pte page refcount. Put the child pte page if
	 * it doesn't contain any other PTEs.
	 */
	child = __pkvm_va(pgt_ops->pte_to_phys(ptep));
	if (mm_ops->page_count(child) == 1) {
		pgt_ops->pte_set(ptep, 0);
		if (ctx->pgt->cap.flush_tlb_lazy) {
			/*
			 * As the flushing TLB is expensive, TLB will be flushed
			 * when the walking is completed. In this case, the page
			 * table page cannot be freed immediately at here as it
			 * can be allocated as a page table page again before
			 * flushing TLB. To prevent this, push the going-to-be
			 * freed page table page to a teardown list and free
			 * them after the TLB is flushed.
			 */
			list_add_tail((struct list_head *)child, &ctx->teardown_pages);
			/*
			 * The old value of the parent pte (pointed by the
			 * ctx->ptep) may still be in the paging structure cache
			 * and point to the page-table page represented by child.
			 * But now the first two PTEs in the child page are used
			 * as the list_head. To prevent translating a virtual
			 * address via these two PTEs, they should be non-present
			 * PTEs. BUG_ON if they are not.
			 *
			 * We are relying on a combination of two things here:
			 * 1. addresses in list_head->next and list_head->prev are
			 * naturally aligned to 8 bytes so their 3 lowest bits are
			 * zero.
			 * 2. a PTE in the given page table format is non-present
			 * if its 3 lowest bits are zero.
			 */
			BUG_ON(pgt_ops->pte_present(child) ||
			       pgt_ops->pte_present(child + pgt_ops->pte_size(ctx->level)));
		} else {
			/* Flush TLB before release the child table page */
			pgt_ops->flush_tlb(ctx->pgt, ctx->addr,
					   pgt_ops->level_to_size(ctx->level));
			mm_ops->put_page(child);
		}
		mm_ops->put_page(ptep);
	}
}

static int unmap_walker(struct pkvm_pgtable_visit_ctx *ctx, unsigned long walk_flags,
			void *const arg)
{
	const struct pkvm_pgtable_mm_ops *mm_ops = ctx->pgt->mm_ops;
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	void *ptep = ctx->ptep;
	int ret;

	if (!pgt_ops->pte_present(ptep) && !pgt_ops->pte_annotated(ptep))
		return 0;

	ret = pgtable_try_unmap_leaf(ctx, (struct pgt_unmap_data *)arg);
	if (ret != -E2BIG)
		return ret;

	/*
	 * If a huge mapping already exists at the current level, or the pte
	 * contains annotation, split it into smaller ones.
	 */
	if (pgt_ops->pte_huge(ptep) || pgt_ops->pte_annotated(ptep)) {
		void *page;

		page = pgtable_alloc_page(mm_ops, NULL);
		if (!page)
			return -ENOMEM;

		/* Split doesn't change the translation so no need to flush tlb. */
		pgtable_split(ctx, page);
		pgt_ops->pte_set(ptep, ctx->pgt->cap.table_prot | __pkvm_pa(page));
	} else {
		/* Not a huge entry means it is a table entry */
		pgtable_free_child(ctx);
	}

	return 0;
}

static int free_walker(struct pkvm_pgtable_visit_ctx *ctx, unsigned long walk_flags,
		       void *const arg)
{
	const struct pkvm_pgtable_mm_ops *mm_ops = ctx->pgt->mm_ops;
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	void *child, *ptep = ctx->ptep;

	if (!pgt_ops->pte_present(ptep) && !pgt_ops->pte_annotated(ptep))
		return 0;

	mm_ops->put_page(ptep);

	if (!pgt_ops->pte_is_leaf(ptep, ctx->level)) {
		BUG_ON(!pgt_ops->pte_present(ptep));

		child = __pkvm_va(pgt_ops->pte_to_phys(ptep));
		BUG_ON(mm_ops->page_count(child) != 1);

		mm_ops->put_page(child);
	}

	return 0;
}

struct pgt_lookup_data {
	unsigned long vaddr;
	unsigned long phys;
	u64 prot;
	int level;
};

static int lookup_walker(struct pkvm_pgtable_visit_ctx *ctx, unsigned long walk_flags,
			 void *const arg)
{
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	u64 pte = pgt_ops->pte_get(ctx->ptep);
	struct pgt_lookup_data *data = arg;
	int level = ctx->level;

	data->level = level;

	if (pgt_ops->pte_present(&pte)) {
		unsigned long offset = data->vaddr & ~pgt_ops->level_to_mask(level);

		data->phys = pgt_ops->pte_to_phys(&pte) + offset;
		data->prot = pgt_ops->pte_to_prot(&pte);
	}

	return PGTABLE_WALK_DONE;
}

struct pgt_lookup_range_data {
	unsigned long vaddr, vaddr_end;
	unsigned long start, end;
	unsigned long phys;
	u64 prot;
};

static int lookup_range_walker(struct pkvm_pgtable_visit_ctx *ctx,
			       unsigned long walk_flags,
			       void *const arg)
{
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	struct pgt_lookup_range_data *data = arg;
	u64 pte = pgt_ops->pte_get(ctx->ptep);

	if (pgt_ops->pte_present(&pte)) {
		unsigned long size = pgt_ops->level_to_size(ctx->level);
		unsigned long phys = pgt_ops->pte_to_phys(&pte);
		u64 prot = pgt_ops->pte_to_prot(&pte);

		if (data->start == INVALID_PAGE) {
			unsigned long offset;

			data->start = max(ctx->addr, data->vaddr);
			offset = data->start & ~pgt_ops->level_to_mask(ctx->level);

			data->end = min(data->start + (size - offset),
					data->vaddr_end);
			data->phys = phys + offset;
			data->prot = prot;
		} else if (phys == data->phys + (data->end - data->start) &&
			   prot == data->prot) {
			data->end = min(ctx->addr + size, data->vaddr_end);
		} else {
			return PGTABLE_WALK_DONE;
		}
	} else if (data->start != INVALID_PAGE) {
		return PGTABLE_WALK_DONE;
	}

	return 0;
}

struct pgt_age_data {
	bool mkold;
	bool young;
};

static int age_walker(struct pkvm_pgtable_visit_ctx *ctx, unsigned long walk_flags,
		      void *const arg)
{
	const struct pkvm_pgtable_ops *pgt_ops = ctx->pgt->pgt_ops;
	struct pgt_age_data *data = arg;
	void *ptep = ctx->ptep;

	if (!pgt_ops->pte_present(ptep) || !pgt_ops->pte_young(ptep))
		return 0;

	data->young = true;
	if (data->mkold)
		pgt_ops->pte_mkold(ptep);

	return 0;
}

struct pgt_walk_data {
	struct pkvm_pgtable *pgt;
	unsigned long addr;
	const unsigned long start;
	const unsigned long end;
	struct pkvm_pgtable_walker *walker;
	bool flush_tlb;
	struct list_head teardown_pages;
};

static int _pgtable_walk(struct pgt_walk_data *data, void *ptep, int level);
static int pgtable_visit(struct pgt_walk_data *data, void *ptep, int level)
{
	const struct pkvm_pgtable_ops *pgt_ops = data->pgt->pgt_ops;
	struct pkvm_pgtable_walker *walker = data->walker;
	bool leaf = pgt_ops->pte_is_leaf(ptep, level);
	unsigned long walk_flags = walker->walk_flags;
	struct pkvm_pgtable_visit_ctx ctx = {
		.pgt = data->pgt,
		.start = data->start,
		.end = data->end,
		.addr = data->addr,
		.level = level,
		.ptep = ptep,
		.flush_tlb = false,
		.teardown_pages = LIST_HEAD_INIT(ctx.teardown_pages),
	};
	void *child_ptep;
	int ret = 0;

	if (!leaf && (walk_flags & PKVM_PGTABLE_WALK_TABLE_PRE))
		ret = walker->cb(&ctx, PKVM_PGTABLE_WALK_TABLE_PRE, walker->arg);

	if (leaf && (walk_flags & PKVM_PGTABLE_WALK_LEAF)) {
		ret = walker->cb(&ctx, PKVM_PGTABLE_WALK_LEAF, walker->arg);
		/*
		 * The ptep may be updated by the walker->cb. Revisit the ptep
		 * to see if it is still leaf or not.
		 */
		leaf = pgt_ops->pte_is_leaf(ptep, level);
	}

	if (ret)
		goto out;

	if (leaf) {
		unsigned long size = pgt_ops->level_to_size(level);

		data->addr = ALIGN_DOWN(data->addr, size);
		data->addr += size;
		goto out;
	}

	child_ptep = __pkvm_va(pgt_ops->pte_to_phys(ptep));
	ret = _pgtable_walk(data, child_ptep, level - 1);
	if (ret)
		goto out;

	if (walk_flags & PKVM_PGTABLE_WALK_TABLE_POST)
		ret = walker->cb(&ctx, PKVM_PGTABLE_WALK_TABLE_POST, walker->arg);
out:
	data->flush_tlb |= ctx.flush_tlb;
	list_splice_tail(&ctx.teardown_pages, &data->teardown_pages);
	return ret;
}

static int _pgtable_walk(struct pgt_walk_data *data, void *ptep, int level)
{
	const struct pkvm_pgtable_ops *pgt_ops = data->pgt->pgt_ops;
	int idx = pgt_ops->vaddr_to_index(data->addr, level);
	int entry_size = pgt_ops->pte_size(level);
	int entries = pgt_ops->pte_count(level);
	int ret;

	for (; idx < entries; idx++) {
		if (data->addr >= data->end)
			break;

		ret = pgtable_visit(data, (ptep + idx * entry_size), level);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * pkvm_pgtable_init() - Initialize a pKVM page table.
 * @pgt:	The page table to be initialized.
 * @cap:	The capability to initialize this page table.
 * @mm_ops:	The memory management ops for this page table.
 * @pgt_ops:	The page table ops for this page table.
 *
 * Return: 0 on success, negative error code on failure.
 */
int pkvm_pgtable_init(struct pkvm_pgtable *pgt,
		      struct pkvm_pgtable_cap cap,
		      const struct pkvm_pgtable_mm_ops *mm_ops,
		      const struct pkvm_pgtable_ops *pgt_ops)
{
	void *root;

	if (!pgt || !mm_ops || !pgt_ops)
		return -EINVAL;

	root = pgtable_alloc_page(mm_ops, NULL);
	if (!root)
		return -ENOMEM;

	pgt->root_pa = __pkvm_pa(root);
	pgt->cap = cap;
	pgt->mm_ops = mm_ops;
	pgt->pgt_ops = pgt_ops;

	return 0;
}

/**
 * pkvm_pgtable_walk() - Walk a pKVM page table
 * @pgt:	The page table to walk.
 * @vaddr:	The virtual address for the start of the walk.
 * @size:	The walk range size.
 * @walker:	The callback descriptor which will be executed
 *		during the walk.
 *
 * The walked range is the minimum PAGE_SIZE-aligned range covering
 * [@vaddr, @vaddr + @size).
 *
 * Each page table entry within the memory range [@vaddr, @vaddr + @size) is
 * visited during the walk, including present leaves, non-present leaves, and
 * present non-leaves. The callback in @walker is executed on leaves based on
 * the walk_flags in @walker. If the callback returns zero, the walk will
 * continue to the next entry. If the callback returns a non-zero, the walk will
 * be terminated immediately and return that value.
 *
 * Return: 0 on success, negative error code on failure.
 */
int pkvm_pgtable_walk(struct pkvm_pgtable *pgt, unsigned long vaddr,
		      unsigned long size, struct pkvm_pgtable_walker *walker)
{
	struct pgt_walk_data data = {
		.pgt = pgt,
		.addr = ALIGN_DOWN(vaddr, PAGE_SIZE),
		.start = ALIGN_DOWN(vaddr, PAGE_SIZE),
		.end = ALIGN(vaddr + size, PAGE_SIZE),
		.walker = walker,
		.flush_tlb = false,
		.teardown_pages = LIST_HEAD_INIT(data.teardown_pages),
	};
	int ret;

	if (!pgt->root_pa || vaddr + size <= vaddr || data.end <= data.start ||
	    data.end - data.start > pkvm_pgtable_max_size(pgt))
		return -EINVAL;

	ret = _pgtable_walk(&data, __pkvm_va(pgt->root_pa), pgt->cap.level);

	if (data.flush_tlb || !list_empty(&data.teardown_pages))
		pgt->pgt_ops->flush_tlb(pgt, data.start, data.end - data.start);

	/*
	 * Free the unused page table pages in the teardown_pages list after
	 * flushing the TLB, to make sure there is no stale TLB when these
	 * pages are allocated as page table pages.
	 */
	while (!list_empty(&data.teardown_pages)) {
		struct list_head *page = data.teardown_pages.next;

		list_del(page);
		pgt->mm_ops->put_page((void *)page);
	}

	return ret;
}

/**
 * pkvm_pgtable_map() - Install virtual addr to physical addr mappings in a pkvm
 *			pgtable.
 * @pgt:	The page table to install mappings.
 * @vaddr:	The virtual address of the installed mapping.
 * @phys:	The physical address to map.
 * @size:	The memory size to map.
 * @prot:	The property bits to create the mapping.
 * @mc:		Optional memcache for allocating page table pages.
 *
 * The mapped range is the minimum PAGE_SIZE-aligned range covering
 * [@vaddr, @vaddr + @size), and @phys is aligned down to the PAGE_SIZE.
 *
 * The mappings for @vaddr to @phys in [@vaddr, @vaddr + @size) are installed
 * to the page table, with the same @prot. If the mapping already exists at a
 * higher level, the huge mapping will be split into smaller ones and a new
 * mapping will be installed at the proper level. If the mapping already exists
 * at the same level but with different physical address or property bits, it
 * will be replaced with the new one and TLB will be flushed.
 *
 * Return: 0 on success, negative error code on failure.
 */
int pkvm_pgtable_map(struct pkvm_pgtable *pgt, unsigned long vaddr,
		     unsigned long phys, unsigned long size, u64 prot,
		     struct pkvm_memcache *mc)
{
	struct pgt_map_data data = {
		.prot = prot,
		.annotation = 0,
		.memcache = mc,
	};
	struct pkvm_pgtable_walker walker = {
		.cb = map_walker,
		.arg = &data,
		.walk_flags = PKVM_PGTABLE_WALK_LEAF,
	};

	if (!VALID_PAGE(phys))
		return -EINVAL;

	data.phys = ALIGN_DOWN(phys, PAGE_SIZE);
	return pkvm_pgtable_walk(pgt, vaddr, size, &walker);
}

/**
 * pkvm_pgtable_unmap() - Remove virtual addr to physical addr mappings from a
 *			  pKVM pgtable.
 * @pgt:	The page table to remove mapping.
 * @vaddr:	The virtual address of the mapping to be removed.
 * @phys:	The desired physical address for the unmap walker to verify.
 * @size:	The memory size to unmap.
 *
 * The unmapped range is the minimum PAGE_SIZE-aligned range covering
 * [@vaddr, @vaddr + @size), and @phys is aligned down to the PAGE_SIZE if @phys
 * is not INVALID_PAGE.
 *
 * All the mappings for the memory pages in [@vaddr, @vaddr + @size) are removed
 * from the page table. Each leaf entry in the range is visited by the unmap
 * walker, to unmap the present ones, followed by the TLB flushing. If the range
 * represented by a present leaf overlaps with the unmapped range, the leaf will
 * be split into smaller ones for the unmap walker to walk down to the next
 * level to remove. If all the mappings in a page table page are removed then
 * this page table page will be freed after the TLB flushing.
 *
 * When @phys is not INVALID_PAGE, the unmap walker will verify if the physical
 * addresses in the removed mappings in range [@vaddr, @vaddr + @size) match
 * with [@phys, @phys + @size) or not. The pKVM hypervisor will panic if the
 * verification for any of the removed mapping is failed. If @phys is
 * INVALID_PAGE, then the verification will not be performed.
 *
 * Return: 0 on success, negative error code on failure.
 */
int pkvm_pgtable_unmap(struct pkvm_pgtable *pgt, unsigned long vaddr,
		       unsigned long phys, unsigned long size)
{
	struct pgt_unmap_data data = {
		.phys = VALID_PAGE(phys) ? ALIGN_DOWN(phys, PAGE_SIZE) :
					   INVALID_PAGE,
	};
	struct pkvm_pgtable_walker walker = {
		.cb = unmap_walker,
		.arg = &data,
		.walk_flags = PKVM_PGTABLE_WALK_LEAF | PKVM_PGTABLE_WALK_TABLE_POST,
	};

	return pkvm_pgtable_walk(pgt, vaddr, size, &walker);
}

/**
 * pkvm_pgtable_set_owner() - Set the owner ID annotation for a range of virtual
 *			      addresses in a page table.
 * @pgt:	Page table to set owner.
 * @vaddr:	Starting virtual address of the range to set owner.
 * @size:	Size of the range to set owner.
 * @owner:	Owner ID to set as annotation for the specified range.
 *
 * The address range is the minimum PAGE_SIZE-aligned range covering [@vaddr,
 * @vaddr + @size).
 *
 * Walks the page table entries covering the specified virtual address range
 * [@vaddr, @vaddr + @size) and sets the owner ID annotation for each entry. If
 * the entry was a present leaf, it will become a non-present leaf, i.e.
 * unmapped, as the pte will be set with the @owner bits only.
 *
 * Return: 0 on success, negative value on failure.
 */
int pkvm_pgtable_set_owner(struct pkvm_pgtable *pgt, unsigned long vaddr,
			   unsigned long size, enum pkvm_owner_id owner)
{
	struct pgt_map_data data = {
		.phys = INVALID_PAGE,
		.prot = 0,
		.annotation = pkvm_pte_mk_owner_id(owner),
	};
	struct pkvm_pgtable_walker walker = {
		.cb = map_walker,
		.arg = &data,
		.walk_flags = PKVM_PGTABLE_WALK_LEAF,
	};

	return pkvm_pgtable_walk(pgt, vaddr, size, &walker);
}

/*
 * pkvm_pgtable_lookup() - Lookup the mapping information of a virtual address
 *			   in a page table.
 * @pgt:	The page table to lookup.
 * @vaddr:	The virtual address of the lookup mapping.
 * @phys:	To return the physical address bits in the present leaf entry.
 * @prot:	To return the property bits in the present leaf entry.
 * @level:	To return page table level of the leaf entry.
 *
 * @vaddr is aligned down to the PAGE_SIZE, and the lookup size is PAGE_SIZE.
 *
 * The page table is walked based on @vaddr to look up for the leaf entry. If
 * the leaf entry is present, will return physical address + the offset within
 * a page via @phys, property bits via @prot and the present leaf entry level
 * via @level. If the leaf entry is non-present, will return INVALID_PAGE via
 * @phys, 0 via @prot and the non-present leaf entry level via @level. So @level
 * always contains a valid number, regardless the leaf entry is present or not.
 */
void pkvm_pgtable_lookup(struct pkvm_pgtable *pgt, unsigned long vaddr,
			 unsigned long *phys, u64 *prot, int *level)
{
	struct pgt_lookup_data data = {
		.vaddr = vaddr,
		.phys = INVALID_PAGE,
		.prot = 0,
		.level = pgt->cap.level,
	};
	struct pkvm_pgtable_walker walker = {
		.cb = lookup_walker,
		.arg = &data,
		.walk_flags = PKVM_PGTABLE_WALK_LEAF,
	};

	pkvm_pgtable_walk(pgt, vaddr, PAGE_SIZE, &walker);

	if (phys)
		*phys = data.phys;
	if (prot)
		*prot = data.prot;
	if (level)
		*level = data.level;
}

/*
 * pkvm_pgtable_lookup_range() - Lookup the first contiguously mapped sub-range
 *				 within the given virtual address range.
 * @pgt:		The page table to lookup.
 * @vaddr:		The start address of virtual address range.
 * @size:		The size of the virtual address range.
 * @range_vaddr:	To return the start virtual address of the found range.
 * @range_size:		To return the size of the found range.
 * @phys:		To return the start physical address of the found range.
 * @prot:		To return the property bits of the found range.
 *
 * The page table is walked to look up the first mapped sub-range within the VA
 * range [@vaddr, @vaddr + @size) such that all the pages within this sub-range
 * are mapped physically contiguously and with the same property bits. If such a
 * VA sub-range is found, its start virtual address is returned as @range_vaddr,
 * its size is returned as @range_size, its start physical address is returned
 * as @phys and its property bits are returned as @prot. If no such VA sub-range
 * found, @range_vaddr and @phys are set to INVALID_PAGE, and @range_size and
 * @prot are set to 0.
 */
void pkvm_pgtable_lookup_range(struct pkvm_pgtable *pgt,
			       unsigned long vaddr, unsigned long size,
			       unsigned long *range_vaddr,
			       unsigned long *range_size,
			       unsigned long *phys, u64 *prot)
{
	struct pgt_lookup_range_data data = {
		.vaddr = vaddr,
		.vaddr_end = vaddr + size,
		.start = INVALID_PAGE,
		.end = INVALID_PAGE,
		.phys = INVALID_PAGE,
		.prot = 0,
	};
	struct pkvm_pgtable_walker walker = {
		.cb = lookup_range_walker,
		.arg = &data,
		.walk_flags = PKVM_PGTABLE_WALK_LEAF,
	};

	BUG_ON(!VALID_PAGE(vaddr));
	pkvm_pgtable_walk(pgt, vaddr, size, &walker);

	if (range_vaddr)
		*range_vaddr = data.start;
	if (range_size)
		*range_size = data.end - data.start;
	if (phys)
		*phys = data.phys;
	if (prot)
		*prot = data.prot;
}

/**
 * pkvm_pgtable_destroy() - Destroy an unused page table.
 * @pgt:	The page table to be destroyed.
 *
 * Frees pages allocated for the given page table. Does not clear PTEs
 * and does not flush TLB, assuming that the page table is no longer used
 * by any hardware walkers.
 */
void pkvm_pgtable_destroy(struct pkvm_pgtable *pgt)
{
	struct pkvm_pgtable_walker walker = {
		.cb = free_walker,
		.arg = NULL,
		.walk_flags = PKVM_PGTABLE_WALK_LEAF | PKVM_PGTABLE_WALK_TABLE_POST,
	};

	WARN_ON(pkvm_pgtable_walk(pgt, 0, pkvm_pgtable_max_size(pgt), &walker));

	pgt->mm_ops->put_page(__pkvm_va(pgt->root_pa));
}

/**
 * pkvm_pgtable_test_clear_young() - Test and optionally clear the access flag
 *				     in the page table entries for the given
 *				     virtual address range.
 * @pgt:	The page table.
 * @vaddr:	The start address of virtual address range.
 * @size:	The size of the virtual address range.
 * @mkold:	If true, clear the access flag if it is set.
 *
 * Does not flush the TLB after clearing the access flag. It is the caller's
 * responsibility to flush the TLB when needed.
 *
 * Return: true if any of the visited PTEs had the access flag set.
 */
bool pkvm_pgtable_test_clear_young(struct pkvm_pgtable *pgt, unsigned long vaddr,
				   unsigned long size, bool mkold)
{
	struct pgt_age_data data = {
		.mkold = mkold,
		.young = false,
	};
	struct pkvm_pgtable_walker walker = {
		.cb = age_walker,
		.arg = &data,
		.walk_flags = PKVM_PGTABLE_WALK_LEAF,
	};

	WARN_ON_ONCE(pkvm_pgtable_walk(pgt, vaddr, size, &walker));
	return data.young;
}
