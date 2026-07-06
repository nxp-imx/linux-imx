/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Static key and sysfs file controlling VMA readahead boundary.
 *
 * Copyright (c) 2026, Google LLC.
 * Author: Frederick Mayle <fmayle@google.com>
 */
#ifndef _LINUX_VMA_READAHEAD_BOUNDARY_H
#define _LINUX_VMA_READAHEAD_BOUNDARY_H

#include <linux/jump_label.h>

DECLARE_STATIC_KEY_FALSE(vma_readahead_boundary_enabled);

static inline bool is_vma_readahead_boundary_enabled(void)
{
	return static_branch_unlikely(&vma_readahead_boundary_enabled);
}

#endif /* _LINUX_VMA_READAHEAD_BOUNDARY_H */
