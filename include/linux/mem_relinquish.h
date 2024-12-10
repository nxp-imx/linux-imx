/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2022 Google LLC
 * Author: Keir Fraser <keirf@google.com>
 */

#ifndef __MEM_RELINQUISH_H__
#define __MEM_RELINQUISH_H__

struct page;

#ifdef CONFIG_MEMORY_RELINQUISH

void page_relinquish(struct page *page);

#else	/* !CONFIG_MEMORY_RELINQUISH */

static inline void page_relinquish(struct page *page) { }

#endif	/* CONFIG_MEMORY_RELINQUISH */

#endif	/* __MEM_RELINQUISH_H__ */
