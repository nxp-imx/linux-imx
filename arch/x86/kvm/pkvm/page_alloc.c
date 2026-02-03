// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is copied from arch/arm64/kvm/hyp/nvhe/page_alloc.c with
 * renaming some code to keep consistent with the naming usage in the
 * pkvm.
 *
 * Copyright (C) 2020 Google LLC
 * Author: Quentin Perret <qperret@google.com>
 */

#include "gfp.h"

u64 __pkvm_vmemmap;

/*
 * Index the pkvm_vmemmap to find a potential buddy page, but make no assumption
 * about its current state.
 *
 * Example buddy-tree for a 4-pages physically contiguous pool:
 *
 *                 o : Page 3
 *                /
 *               o-o : Page 2
 *              /
 *             /   o : Page 1
 *            /   /
 *           o---o-o : Page 0
 *    Order  2   1 0
 *
 * Example of requests on this pool:
 *   __find_buddy_nocheck(pool, page 0, order 0) => page 1
 *   __find_buddy_nocheck(pool, page 0, order 1) => page 2
 *   __find_buddy_nocheck(pool, page 1, order 0) => page 0
 *   __find_buddy_nocheck(pool, page 2, order 0) => page 3
 */
static struct pkvm_page *__find_buddy_nocheck(struct pkvm_pool *pool,
					      struct pkvm_page *p,
					      u8 order)
{
	phys_addr_t addr = pkvm_page_to_phys(p);

	addr ^= (PAGE_SIZE << order);

	/*
	 * Don't return a page outside the pool range -- it belongs to
	 * something else and may not be mapped in pkvm_vmemmap.
	 */
	if (addr < pool->range_start || addr >= pool->range_end)
		return NULL;

	return pkvm_phys_to_page(addr);
}

/* Find a buddy page currently available for allocation */
static struct pkvm_page *__find_buddy_avail(struct pkvm_pool *pool,
					    struct pkvm_page *p,
					    u8 order)
{
	struct pkvm_page *buddy = __find_buddy_nocheck(pool, p, order);

	if (!buddy || buddy->order != order || buddy->refcount)
		return NULL;

	return buddy;

}

/*
 * Pages that are available for allocation are tracked in free-lists, so we use
 * the pages themselves to store the list nodes to avoid wasting space. As the
 * allocator always returns zeroed pages (which are zeroed on the pkvm_put_page()
 * path to optimize allocation speed), we also need to clean-up the list node in
 * each page when we take it out of the list.
 */
static inline void page_remove_from_list(struct pkvm_page *p)
{
	struct list_head *node = pkvm_page_to_virt(p);

	__list_del_entry(node);
	memset(node, 0, sizeof(*node));
}

static inline void page_add_to_list(struct pkvm_page *p, struct list_head *head)
{
	struct list_head *node = pkvm_page_to_virt(p);

	INIT_LIST_HEAD(node);
	list_add_tail(node, head);
}

static inline struct pkvm_page *node_to_page(struct list_head *node)
{
	return pkvm_virt_to_page(node);
}

static void __pkvm_attach_page(struct pkvm_pool *pool,
			       struct pkvm_page *p)
{
	phys_addr_t phys = pkvm_page_to_phys(p);
	u8 order = p->order;
	struct pkvm_page *buddy;

	memset(pkvm_page_to_virt(p), 0, PAGE_SIZE << p->order);

	/* Skip coalescing for 'external' pages being freed into the pool. */
	if (phys < pool->range_start || phys >= pool->range_end)
		goto insert;

	/*
	 * Only the first struct pkvm_page of a high-order page (otherwise known
	 * as the 'head') should have p->order set. The non-head pages should
	 * have p->order = PKVM_NO_ORDER. Here @p may no longer be the head
	 * after coalescing, so make sure to mark it PKVM_NO_ORDER proactively.
	 */
	p->order = PKVM_NO_ORDER;
	for (; (order + 1) <= pool->max_order; order++) {
		buddy = __find_buddy_avail(pool, p, order);
		if (!buddy)
			break;

		/* Take the buddy out of its list, and coalesce with @p */
		page_remove_from_list(buddy);
		buddy->order = PKVM_NO_ORDER;
		p = min(p, buddy);
	}

insert:
	/* Mark the new head, and insert it */
	p->order = order;
	page_add_to_list(p, &pool->free_area[order]);
}

static struct pkvm_page *__pkvm_extract_page(struct pkvm_pool *pool,
					     struct pkvm_page *p,
					     u8 order)
{
	struct pkvm_page *buddy;

	page_remove_from_list(p);
	while (p->order > order) {
		/*
		 * The buddy of order n - 1 currently has PKVM_NO_ORDER as it
		 * is covered by a higher-level page (whose head is @p). Use
		 * __find_buddy_nocheck() to find it and inject it in the
		 * free_list[n - 1], effectively splitting @p in half.
		 */
		p->order--;
		buddy = __find_buddy_nocheck(pool, p, p->order);
		buddy->order = p->order;
		page_add_to_list(buddy, &pool->free_area[buddy->order]);
	}

	return p;
}

static void __pkvm_put_page(struct pkvm_pool *pool, struct pkvm_page *p)
{
	if (pkvm_page_ref_dec_and_test(p))
		__pkvm_attach_page(pool, p);
}

/*
 * Changes to the buddy tree and page refcounts must be done with the pkvm_pool
 * lock held. If a refcount change requires an update to the buddy tree (e.g.
 * pkvm_put_page()), both operations must be done within the same critical
 * section to guarantee transient states (e.g. a page with null refcount but
 * not yet attached to a free list) can't be observed by well-behaved readers.
 */
void pkvm_put_page(struct pkvm_pool *pool, void *addr)
{
	struct pkvm_page *p = pkvm_virt_to_page(addr);

	pkvm_spin_lock(&pool->lock);
	__pkvm_put_page(pool, p);
	pkvm_spin_unlock(&pool->lock);
}

void pkvm_get_page(struct pkvm_pool *pool, void *addr)
{
	struct pkvm_page *p = pkvm_virt_to_page(addr);

	pkvm_spin_lock(&pool->lock);
	pkvm_page_ref_inc(p);
	pkvm_spin_unlock(&pool->lock);
}

void pkvm_split_page(struct pkvm_page *p)
{
	u8 order = p->order;
	unsigned int i;

	p->order = 0;
	for (i = 1; i < (1 << order); i++) {
		struct pkvm_page *tail = p + i;

		tail->order = 0;
		pkvm_set_page_refcounted(tail);
	}
}

void *pkvm_alloc_pages(struct pkvm_pool *pool, u8 order)
{
	struct pkvm_page *p;
	u8 i = order;

	pkvm_spin_lock(&pool->lock);

	/* Look for a high-enough-order page */
	while (i <= pool->max_order && list_empty(&pool->free_area[i]))
		i++;
	if (i > pool->max_order) {
		pkvm_spin_unlock(&pool->lock);
		return NULL;
	}

	/* Extract it from the tree at the right order */
	p = node_to_page(pool->free_area[i].next);
	p = __pkvm_extract_page(pool, p, order);

	pkvm_set_page_refcounted(p);
	pkvm_spin_unlock(&pool->lock);

	return pkvm_page_to_virt(p);
}

int pkvm_pool_init(struct pkvm_pool *pool, u64 pfn, unsigned int nr_pages,
		   unsigned int reserved_pages)
{
	phys_addr_t phys = pkvm_pfn_to_phys(pfn);
	struct pkvm_page *p;
	int i;

	pkvm_spin_lock_init(&pool->lock);
	pool->max_order = min(MAX_PAGE_ORDER,
			      get_order(nr_pages << PAGE_SHIFT));
	for (i = 0; i <= pool->max_order; i++)
		INIT_LIST_HEAD(&pool->free_area[i]);
	pool->range_start = phys;
	pool->range_end = phys + (nr_pages << PAGE_SHIFT);

	/* Init the vmemmap portion */
	p = pkvm_phys_to_page(phys);
	for (i = 0; i < nr_pages; i++)
		pkvm_set_page_refcounted(&p[i]);

	/* Attach the unused pages to the buddy tree */
	for (i = reserved_pages; i < nr_pages; i++)
		__pkvm_put_page(pool, &p[i]);

	return 0;
}
