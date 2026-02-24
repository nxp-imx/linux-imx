// SPDX-License-Identifier: GPL-2.0
#include <linux/mm.h>
#include <asm/kvm_pkvm.h>
#include <asm/pkvm_spinlock.h>
#include <asm/string.h>
#include "early_alloc.h"
#include "pgtable.h"

static unsigned long base;
static unsigned long end;
static unsigned long cur;

static DEFINE_PKVM_SPINLOCK(early_lock);

void *pkvm_early_alloc_contig(unsigned int nr_pages)
{
	unsigned long size = (nr_pages << PAGE_SHIFT);
	void *ret;

	if (!nr_pages)
		return NULL;

	pkvm_spin_lock(&early_lock);
	if (end - cur < size) {
		pkvm_spin_unlock(&early_lock);
		return NULL;
	}

	ret = (void *)cur;
	cur += size;
	pkvm_spin_unlock(&early_lock);

	memset(ret, 0, size);

	return ret;
}

void *pkvm_early_alloc_page(struct pkvm_memcache *mc)
{
	return pkvm_early_alloc_contig(1);
}

static void pkvm_early_alloc_get_page(void *addr) {}
static void pkvm_early_alloc_put_page(void *addr) {}
static int pkvm_early_page_count(void *vaddr)
{
	/*
	 * The early alloc managed pages cannot be freed thus it doesn't need
	 * to support get_page/put_page. The page count is also meaningless.
	 * So always return a non-zero value.
	 */
	return INT_MAX;
}

const struct pkvm_pgtable_mm_ops pkvm_early_alloc_mm_ops = {
	.zalloc_page = pkvm_early_alloc_page,
	.get_page = pkvm_early_alloc_get_page,
	.put_page = pkvm_early_alloc_put_page,
	.page_count = pkvm_early_page_count,
};

void pkvm_early_alloc_init(void *virt, unsigned long size)
{
	base = cur = (unsigned long)virt;
	end = base + size;
}

unsigned long pkvm_early_alloc_nr_used_pages(void)
{
	return (cur - base) >> PAGE_SHIFT;
}
