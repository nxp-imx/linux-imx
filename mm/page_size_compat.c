// SPDX-License-Identifier: GPL-2.0
/*
 * Page Size Emulation
 *
 * Copyright (c) 2024, Google LLC.
 * Author: Kalesh Singh <kaleshsingh@goole.com>
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kstrtox.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/page_size_compat.h>
#include <linux/swap.h>
#include <linux/perf_event.h>

#define MIN_PAGE_SHIFT_COMPAT (PAGE_SHIFT + 1)
#define MAX_PAGE_SHIFT_COMPAT 16 /* Max of 64KB */
#define __MMAP_RND_BITS(x)      (x - (__PAGE_SHIFT - PAGE_SHIFT))

DEFINE_STATIC_KEY_FALSE(page_shift_compat_enabled);
EXPORT_SYMBOL_GPL(page_shift_compat_enabled);

int page_shift_compat __ro_after_init = MIN_PAGE_SHIFT_COMPAT;
EXPORT_SYMBOL_GPL(page_shift_compat);

static int __init early_page_shift_compat(char *buf)
{
	int ret;

	ret = kstrtoint(buf, 10, &page_shift_compat);
	if (ret)
		return ret;

	/* Only supported on 4KB kernel */
	if (PAGE_SHIFT != 12)
		return -ENOTSUPP;

	if (page_shift_compat < MIN_PAGE_SHIFT_COMPAT ||
		page_shift_compat > MAX_PAGE_SHIFT_COMPAT)
		return -EINVAL;

	static_branch_enable(&page_shift_compat_enabled);

	return 0;
}
early_param("page_shift", early_page_shift_compat);

static int __init init_mmap_rnd_bits(void)
{
	if (!static_branch_unlikely(&page_shift_compat_enabled))
		return 0;

#ifdef CONFIG_HAVE_ARCH_MMAP_RND_BITS
	mmap_rnd_bits_min = __MMAP_RND_BITS(CONFIG_ARCH_MMAP_RND_BITS_MIN);
	mmap_rnd_bits_max = __MMAP_RND_BITS(CONFIG_ARCH_MMAP_RND_BITS_MAX);
	mmap_rnd_bits = __MMAP_RND_BITS(CONFIG_ARCH_MMAP_RND_BITS);
#endif

	return 0;
}
core_initcall(init_mmap_rnd_bits);

#ifdef CONFIG_BPF_SYSCALL
bool bpf_is_ringbuf_file(struct file *file)
{
	if (file->f_op != &bpf_map_fops)
		return false;

	struct bpf_map *map = file->private_data;

	return map->map_type == BPF_MAP_TYPE_RINGBUF ||
		map->map_type == BPF_MAP_TYPE_USER_RINGBUF;
}
#else
bool bpf_is_ringbuf_file(struct file *file)
{
	return false;
}
#endif

/*
 * The swap header is usually in the first page, with the magic in the last 10 bytes.
 * of the page. In the emulated mode, mkswap tools might place the magic on the last
 * 10 bytes of __PAGE_SIZE-ed page. Check if this is the case and place the magic on
 * the first page and clear the magic from the original page in which it was found.
 *
 */
int __fixup_swap_header(struct file *swap_file, struct address_space *mapping)
{
	union swap_header *swap_header;
	struct page *header_page = NULL;
	struct page *magic_page = NULL;
	int index;
	int error = 0;
	const char* magic = "SWAPSPACE2";

	if (__PAGE_SHIFT == PAGE_SHIFT)
		return 0;

	index = (1 << (__PAGE_SHIFT  - PAGE_SHIFT)) - 1;
	magic_page = read_mapping_page(mapping, index, swap_file);
	if (IS_ERR(magic_page)) {
		pgcompat_err("Failed reading swap magic page");
		return PTR_ERR(magic_page);
	}
	swap_header = kmap(magic_page);

	/* Nothing to do; mkswap tool may have hardcoded a 4096 page size */
	if (memcmp(magic, swap_header->magic.magic, 10))
		goto free_magic;

	memset(swap_header->magic.magic, 0, 10);

	index = 0;
	header_page = read_mapping_page(mapping, index, swap_file);
	if (IS_ERR(header_page)) {
		pgcompat_err("Failed reading swap header page");
		error = PTR_ERR(header_page);
		goto free_magic;
	}
	swap_header = kmap(header_page);

	memcpy(swap_header->magic.magic, magic, 10);

	kunmap(header_page);
	put_page(header_page);

free_magic:
	kunmap(magic_page);
	put_page(magic_page);

	return error;
}

#if IS_ENABLED(CONFIG_PERF_EVENTS)
static int __init init_sysctl_perf_event_mlock(void)
{
	if (!static_branch_unlikely(&page_shift_compat_enabled))
		return 0;

	/* Minimum for 512 kiB + 1 user control page */
	sysctl_perf_event_mlock = 512 + (__PAGE_SIZE / 1024); /* 'free' kiB per user */

	return 0;
}
core_initcall(init_sysctl_perf_event_mlock);
#endif
