/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_X86_PGTABLE_H
#define __PKVM_X86_PGTABLE_H

#include <linux/types.h>
#include <linux/bitfield.h>
#include "mem_protect.h"

struct pkvm_pgtable;

/**
 * struct pkvm_pgtable_mm_ops - Page table memory management callbacks.
 * @zalloc_page:	Allocate a page with clearing its contents.
 * @get_page:		Increment the reference count of a given page.
 * @put_page:		Decrement the reference count of a given page.
 * @page_count:		Get the reference count of a given page.
 */
struct pkvm_pgtable_mm_ops {
	void *(*zalloc_page)(void);
	void (*get_page)(void *vaddr);
	void (*put_page)(void *vaddr);
	int (*page_count)(void *vaddr);
};

/**
 * struct pkvm_pgtable_ops - Page table operation callbacks.
 * @pte_present:	Check if a pte is present.
 * @pte_annotated:	Check if a pte is annotated with page state/owner id.
 * @pte_huge:		Check if a pte is huge.
 * @pte_mkhuge:		Set huge for the given pte.
 * @pte_to_phys:	Decode the physical address from pte.
 * @pte_to_prot:	Decode the property bits from pte, including the page
 *                      state bits.
 * @calc_pte_perm:	Calculate the pte permission bits according to the
 *			read/write/exec permissions.
 * @calc_pte_memtype:	Calculate the pte memory type bits for either memory or
 *			MMIO.
 * @vaddr_to_index:	Calculate the entry index for a virtual address
 *			according to the given page table level.
 * @level_to_size:	Calculate the page size at a page table level.
 * @level_to_mask:	Calculate the page size mask at a page table level.
 * @pte_is_leaf:	Check if a pte is leaf.
 * @pte_size:		Calculate the pte size in bytes at a given page table
 *			level.
 * @pte_count:		Calculate the number of ptes at a given page table
 *			level.
 * @pte_set:		Set a pte to a given raw value.
 * @pte_get:		Get the raw value stored in the pte.
 * @flush_tlb:		Flush the TLB for a virtual address range.
 * @pte_mk_pgstate:	Make a pte value with the given the page state.
 * @pte_pgstate:	Get the page state from the pte.
 * @pgstate_mask:	Get the page state bits mask.
 */
struct pkvm_pgtable_ops {
	bool (*pte_present)(void *ptep);
	bool (*pte_annotated)(void *pte);
	bool (*pte_huge)(void *ptep);
	void (*pte_mkhuge)(void *ptep);
	unsigned long (*pte_to_phys)(void *ptep);
	u64 (*pte_to_prot)(void *ptep);
	u64 (*calc_pte_perm)(bool read, bool write, bool exec);
	u64 (*calc_pte_memtype)(bool mmio);
	int (*vaddr_to_index)(unsigned long vaddr, int level);
	unsigned long (*level_to_size)(int level);
	u64 (*level_to_mask)(int level);
	bool (*pte_is_leaf)(void *ptep, int level);
	int (*pte_size)(int level);
	int (*pte_count)(int level);
	void (*pte_set)(void *ptep, u64 val);
	u64 (*pte_get)(void *ptep);
	void (*flush_tlb)(struct pkvm_pgtable *pgt,
			  unsigned long vaddr, unsigned long size);
	u64 (*pte_mk_pgstate)(enum pkvm_page_state state);
	enum pkvm_page_state (*pte_pgstate)(void *ptep);
	u64 (*pgstate_mask)(void);
};

struct pkvm_pgtable_cap {
	int level;
	int allowed_pgsz;
	u64 table_prot;
	bool flush_tlb_lazy;
};

struct pkvm_pgtable {
	unsigned long root_pa;
	struct pkvm_pgtable_cap cap;
	const struct pkvm_pgtable_mm_ops *mm_ops;
	const struct pkvm_pgtable_ops *pgt_ops;
};

struct pkvm_pgtable_visit_ctx {
	struct pkvm_pgtable *pgt;
	const unsigned long start;
	const unsigned long end;
	unsigned long addr;
	int level;
	void *ptep;
	bool flush_tlb;
	struct list_head teardown_pages;
};

typedef int (*pgtable_visit_fn_t)(struct pkvm_pgtable_visit_ctx *ctx,
				  unsigned long walk_flags, void *const arg);

/**
 * enum pkvm_pgtable_walk_flags - Flags to control page table walk.
 * @PKVM_PGTABLE_WALK_TABLE_PRE:	Execute the callback on a non-leaf
 *					entry before visiting its children.
 * @PKVM_PGTABLE_WALK_LEAF:		Execute the callback on a leaf entry
 *					(either present or non-present).
 * @PKVM_PGTABLE_WALK_TABLE_POST:	Execute the callback on a non-leaf
 *					entry after visiting its children.
 */
enum pkvm_pgtable_walk_flags {
	PKVM_PGTABLE_WALK_TABLE_PRE	= BIT(0),
	PKVM_PGTABLE_WALK_LEAF		= BIT(1),
	PKVM_PGTABLE_WALK_TABLE_POST	= BIT(2),
};

/**
 * struct pkvm_pgtable_walker - The page table walk callback descriptor.
 * @cb:		The callback executed during the page table walking.
 * @arg:	The argument passed to the callback.
 * @walk_flags:	A bitwise-or of enum pkvm_pgtable_walk_flags to indicate the
 *		walking behaviors.
 */
struct pkvm_pgtable_walker {
	const pgtable_visit_fn_t cb;
	void *const arg;
	const unsigned long walk_flags;
};

int pkvm_pgtable_init(struct pkvm_pgtable *pgt,
		      struct pkvm_pgtable_cap cap,
		      const struct pkvm_pgtable_mm_ops *mm_ops,
		      const struct pkvm_pgtable_ops *pgt_ops);
int pkvm_pgtable_walk(struct pkvm_pgtable *pgt, unsigned long vaddr,
		      unsigned long size, struct pkvm_pgtable_walker *walker);
int pkvm_pgtable_map(struct pkvm_pgtable *pgt, unsigned long vaddr,
		     unsigned long phys, unsigned long size, u64 prot);
int pkvm_pgtable_unmap(struct pkvm_pgtable *pgt, unsigned long vaddr,
		       unsigned long phys, unsigned long size);
int pkvm_pgtable_set_owner(struct pkvm_pgtable *pgt, unsigned long vaddr,
			   unsigned long size, enum pkvm_owner_id owner);
void pkvm_pgtable_lookup(struct pkvm_pgtable *pgt, unsigned long vaddr,
			 unsigned long *phys, u64 *prot, int *level);

/*
 * Return the max size of the virtual address space that can be
 * mapped by the page table @pgt.
 */
static inline unsigned long pkvm_pgtable_max_size(struct pkvm_pgtable *pgt)
{
	return pgt->pgt_ops->level_to_size(pgt->cap.level + 1);
}

static inline u64 pkvm_pgt_pgstate_mask(struct pkvm_pgtable *pgt)
{
	if (!pgt->pgt_ops->pgstate_mask)
		return 0;

	return pgt->pgt_ops->pgstate_mask();
}

static inline u64 pkvm_pte_mk_pgstate(struct pkvm_pgtable *pgt,
				      enum pkvm_page_state state)
{
	if (!pgt->pgt_ops->pte_mk_pgstate)
		return 0;

	return pgt->pgt_ops->pte_mk_pgstate(state);
}

static inline enum pkvm_page_state pkvm_pte_pgstate(struct pkvm_pgtable *pgt, void *ptep)
{
	if (!pgt->pgt_ops->pte_pgstate)
		return PKVM_PAGE_NONE;

	return pgt->pgt_ops->pte_pgstate(ptep);
}

/*
 * Use bit31 ~ bi12 as owner ID bits to avoid conflicting w/ low 12 property
 * bits for all kinds of page table.
 */
#define PKVM_INVALID_PTE_OWNER_SHIFT		12
#define PKVM_INVALID_PTE_OWNER_BITS		20
#define PKVM_INVALID_PTE_OWNER_MASK		GENMASK(PKVM_INVALID_PTE_OWNER_BITS +		\
							PKVM_INVALID_PTE_OWNER_SHIFT - 1,	\
							PKVM_INVALID_PTE_OWNER_SHIFT)

static inline u64 pkvm_pte_mk_owner_id(enum pkvm_owner_id owner)
{
	/*
	 * As 20 bits in the PTE are used to store pKVM owner ID, the number of
	 * pKVM owner ID bits should not be larger than this.
	 */
	BUILD_BUG_ON_MSG(PKVM_OWNER_ID_BITS > PKVM_INVALID_PTE_OWNER_BITS,
			"PTE doesn't have enough bits to store pkvm owner ID");

	BUG_ON(owner >= PKVM_MAX_ID);

	return FIELD_PREP(PKVM_INVALID_PTE_OWNER_MASK, owner);
}

static inline enum pkvm_owner_id pkvm_pte_owner_id(struct pkvm_pgtable *pgt, void *ptep)
{
	return FIELD_GET(PKVM_INVALID_PTE_OWNER_MASK, pgt->pgt_ops->pte_get(ptep));
}

#endif /* __PKVM_X86_PGTABLE_H */
