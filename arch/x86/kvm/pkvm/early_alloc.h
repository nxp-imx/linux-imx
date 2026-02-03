/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_EARLY_ALLOC_H
#define __PKVM_EARLY_ALLOC_H

#include "pgtable.h"

extern const struct pkvm_pgtable_mm_ops pkvm_early_alloc_mm_ops;

unsigned long pkvm_early_alloc_nr_used_pages(void);

#endif /* __PKVM_EARLY_ALLOC_H */
