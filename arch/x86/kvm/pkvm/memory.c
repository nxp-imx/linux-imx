// SPDX-License-Identifier: GPL-2.0
#include <linux/cache.h>
#include <linux/types.h>
#include <asm/kvm_pkvm.h>
#include "memory.h"

unsigned long page_offset_base __ro_after_init;
unsigned long phys_base;
#ifdef CONFIG_DYNAMIC_PHYSICAL_MASK
phys_addr_t physical_mask;
#endif
pteval_t __default_kernel_pte_mask;
#ifdef CONFIG_AMD_MEM_ENCRYPT
u64 sme_me_mask;
#endif

struct memblock_region pkvm_memory[PKVM_MEMBLOCK_REGIONS];
unsigned int pkvm_memblock_nr;

/**
 * pkvm_find_addr_range - Find out the addr range for the given physical addr.
 * @phys:	The physical address to check.
 * @range:	The structure to return the address range which contains the
 *		@phys.
 *
 * A binary search is performed to find out the address range which contains
 * @phys and this address range is returned via @range. The returned address
 * range would be either a memory range if @phys is memory, or a hole (which is
 * between memory ranges and usually can be used as the MMIO) range if @phys is
 * not memory, which is indicated by the return value.
 *
 * Return true if @phys is memory. Otherwise, return false.
 */
bool pkvm_find_addr_range(unsigned long phys, struct range *range)
{
	int cur, left = 0, right = pkvm_memblock_nr;
	struct memblock_region *reg;
	unsigned long end;

	range->start = 0;
	range->end = ULONG_MAX;

	/* The list of memblock regions is sorted, binary search it */
	while (left < right) {
		cur = (left + right) >> 1;
		reg = &pkvm_memory[cur];
		end = reg->base + reg->size;
		if (phys < reg->base) {
			right = cur;
			range->end = reg->base;
		} else if (phys >= end) {
			left = cur + 1;
			range->start = end;
		} else {
			range->start = reg->base;
			range->end = end;
			return true;
		}
	}

	return false;
}

static void pkvm_clflush_cache_range_opt(void *vaddr, unsigned int size)
{
	const unsigned long clflush_size = boot_cpu_data.x86_clflush_size;
	void *p = (void *)((unsigned long)vaddr & ~(clflush_size - 1));
	void *vend = vaddr + size;

	if (p >= vend)
		return;

	for (; p < vend; p += clflush_size)
		clflushopt(p);
}

/**
 * pkvm_clflush_cache_range - flush a cache range with clflush
 * which is implemented refer to clflush_cache_range() in kernel.
 *
 * @vaddr:	virtual start address
 * @size:	number of bytes to flush
 *
 * CLFLUSHOPT is an unordered instruction which needs fencing with MFENCE or
 * SFENCE to avoid ordering issues.
 */
void pkvm_clflush_cache_range(void *vaddr, unsigned int size)
{
	mb();
	pkvm_clflush_cache_range_opt(vaddr, size);
	mb();
}

/*
 * Ensure that __kcfi_typeid_ symbols are emitted for functions that may
 * not be indirectly called with all configurations.
 */
__ADDRESSABLE(__memcpy)
