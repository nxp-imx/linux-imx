/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_VMX_EPT_H
#define __PKVM_VMX_EPT_H

#include "pgtable.h"

int pkvm_host_ept_init(struct pkvm_pgtable *pgt, void *pool_base,
		       unsigned long pool_pages);
int pkvm_host_ept_finalize(struct pkvm_pgtable *pgt);
int pkvm_handle_host_ept_violation(void);
void pkvm_flush_host_ept(void);

u64 pkvm_host_ept_root(void);
int pkvm_host_ept_level(void);

void pkvm_guest_ept_setup(void);

#endif /* __PKVM_VMX_EPT_H */
