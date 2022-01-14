/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2022 ARM Ltd.
 * Copyright (C) 2024 Google, Inc.
 */

#ifndef __LINUX_TRUSTY_FFA_H
#define __LINUX_TRUSTY_FFA_H

#include <linux/kconfig.h>
#include <linux/types.h>
#include <linux/uuid.h>

/**
 * DOC: FF-A driver version requirements
 *
 * This Trusty driver is dependent on a separate 'FF-A' driver.
 * To ensure comaptibility, the version of the FF-A driver
 * must be the same major version and a lower minor version as that
 * supported by this Trusty driver (defined below). Newer
 * versions of the FF-A driver may require changes to this driver
 * before adjusting the defines below.
 */
#define TRUSTY_FFA_VERSION_MAJOR	(1U)
#define TRUSTY_FFA_VERSION_MINOR	(0U)
#define TRUSTY_FFA_VERSION_MAJOR_SHIFT	(16U)
#define TRUSTY_FFA_VERSION_MAJOR_MASK	(0x7fffU)
#define TRUSTY_FFA_VERSION_MINOR_SHIFT	(0U)
#define TRUSTY_FFA_VERSION_MINOR_MASK	(0xffffU)

#define TO_TRUSTY_FFA_MAJOR(v)					\
	  ((u16)(((v) >> TRUSTY_FFA_VERSION_MAJOR_SHIFT) &	\
		 TRUSTY_FFA_VERSION_MAJOR_MASK))

#define TO_TRUSTY_FFA_MINOR(v)					\
	  ((u16)(((v) >> TRUSTY_FFA_VERSION_MINOR_SHIFT) &	\
		 TRUSTY_FFA_VERSION_MINOR_MASK))

#if IS_ENABLED(CONFIG_TRUSTY_FFA_TRANSPORT)
struct ns_mem_page_info;

struct device *trusty_ffa_find_device(void);

int trusty_ffa_dev_share_or_lend_memory(struct device *dev, u64 *id,
					struct scatterlist *sglist,
					unsigned int nents, pgprot_t pgprot,
					u64 tag, bool lend, struct ns_mem_page_info *pg_inf);
int trusty_ffa_dev_reclaim_memory(struct device *dev, u64 id,
				  struct scatterlist *sglist,
				  unsigned int nents);
#endif

#endif /* __LINUX_TRUSTY_FFA_H */
