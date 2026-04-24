/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_PAGE_SIZE_COMPAT_H
#define __LINUX_PAGE_SIZE_COMPAT_H

/*
 * include/linux/page_size_compat.h
 *
 * Page Size Emulation
 *
 * Copyright (c) 2024, Google LLC.
 * Author: Kalesh Singh <kaleshsingh@goole.com>

 * Helper macros for page size emulation.
 *
 * The macros for use with the emulated page size are all
 * namespaced by the prefix '__'.
 *
 * The valid range of androidboot.page_shift is [13, 16].
 * In other words page sizes of 8KB, 16KB, 32KB and 64KB can
 * be emulated.
 */

#include <linux/page_size_compat_defs.h>

#ifndef __ASSEMBLY__

#include <linux/mman.h>
#include <linux/printk.h>

#define pgcompat_err(fmt, ...) \
	pr_err("pgcompat [%i (%s)]: " fmt, task_pid_nr(current), current->comm, ## __VA_ARGS__)

#define __offset_in_page_log(addr)							\
({											\
	if (static_branch_unlikely(&page_shift_compat_enabled) &&			\
			__offset_in_page(addr))						\
		pgcompat_err("%s: addr (0x%08lx) not page aligned", __func__, addr);	\
	(__offset_in_page(addr));							\
})

#define __PAGE_ALIGNED(addr)    (!__offset_in_page_log(addr))

/*
 * Increases @size by an adequate amount to allow __PAGE_SIZE alignment
 * by rounding up; given that @size is already a multiple of the
 * base page size (PAGE_SIZE).
 *
 * Example:
 *     If __PAGE_SHIFT == PAGE_SHIFT == 12
 *         @size is increased by 0
 *             ((1 << (0)) - 1) << PAGE_SHIFT
 *             (1        ) - 1) << PAGE_SHIFT
 *             (0             ) << PAGE_SHIFT
 *
 *     If __PAGE_SHIFT == 13 and PAGE_SHIFT == 12
 *         @size is increased by PAGE_SIZE (4KB):
 *             ((1 << (1)) - 1) << PAGE_SHIFT
 *             (2        ) - 1) << PAGE_SHIFT
 *             (1             ) << PAGE_SHIFT
 *     If __PAGE_SHIFT == 14 and PAGE_SHIFT == 12
 *         @size is increased by 3xPAGE_SIZE (12KB):
 *             ((1 << (2)) - 1) << PAGE_SHIFT
 *             (4        ) - 1) << PAGE_SHIFT
 *             (3             ) << PAGE_SHIFT
 *     ...
 */
#define __PAGE_SIZE_ROUND_UP_ADJ(size) \
	((size) + (((1 << (__PAGE_SHIFT - PAGE_SHIFT)) - 1) << PAGE_SHIFT))

extern int __fixup_swap_header(struct file *swap_file, struct address_space *mapping);

bool bpf_is_ringbuf_file(struct file *file);

static inline unsigned long __bpf_pgoff_fixup(struct file *file, unsigned long pgoff)
{
	if (file && bpf_is_ringbuf_file(file)) {
		unsigned int nr_subpages = __PAGE_SIZE / PAGE_SIZE;

		if (nr_subpages > 1 && pgoff > 0 && (pgoff & (nr_subpages - 1)) == 0)
			pgoff /= nr_subpages;
	}

	return pgoff;
}

#ifdef CONFIG_PROC_PAGE_MONITOR
extern bool __is_emulated_pagemap_file(struct file *file);
#else
static inline bool __is_emulated_pagemap_file(struct file *file)
{
	return false;
}
#endif

static __always_inline void __adjust_cachestat_counters(struct cachestat *cs)
{
	unsigned int nr_sub_pages = __PAGE_SIZE / PAGE_SIZE;

	if (nr_sub_pages <= 1)
		return;

	cs->nr_cache /= nr_sub_pages;
	cs->nr_dirty /= nr_sub_pages;
	cs->nr_writeback /= nr_sub_pages;
	cs->nr_evicted /= nr_sub_pages;
	cs->nr_recently_evicted /= nr_sub_pages;
}

#endif /* !__ASSEMBLY__ */

#endif /* __LINUX_PAGE_SIZE_COMPAT_H */
