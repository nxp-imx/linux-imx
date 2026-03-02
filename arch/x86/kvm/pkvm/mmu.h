/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_X86_MMU_H
#define __PKVM_X86_MMU_H

#include <asm/pkvm_spinlock.h>
#include "init.h"
#include "pgtable.h"
#include "pkvm.h"

extern pkvm_spinlock_t host_mmu_lock;

int pkvm_hyp_mmu_init(void *pool_base, unsigned long pool_pages);
int pkvm_hyp_mmu_switch_to_buddy(void *pool_base, unsigned long pool_pages);
void pkvm_hyp_mmu_load(void);
int pkvm_hyp_mmu_finalize(hyp_mmu_finalize_fn_t fn);
int pkvm_hyp_mmu_map(unsigned long vaddr, unsigned long phys,
		     unsigned long size, u64 prot);
#ifdef CONFIG_PKVM_X86_DEBUG
void pkvm_hyp_mmu_clone_host(unsigned long start_vaddr);
#endif

int pkvm_host_mmu_init(void *pool_base, unsigned long pool_pages,
		       const struct pkvm_mem_info infos[], int nr_infos,
		       host_mmu_init_fn_t fn);
int pkvm_host_mmu_finalize(host_mmu_finalize_fn_t fn);

static inline void pkvm_host_mmu_lock(void)
{
	pkvm_spin_lock(&host_mmu_lock);
}

static inline void pkvm_host_mmu_unlock(void)
{
	pkvm_spin_unlock(&host_mmu_lock);
}

void pkvm_guest_mmu_setup(const struct pkvm_pgtable_ops *pgt_ops,
			  struct pkvm_pgtable_cap pgt_cap);
int pkvm_guest_mmu_max_level(void);
int pkvm_guest_mmu_init(struct pkvm_vm *pkvm_vm, phys_addr_t pgd_pa);
void pkvm_guest_mmu_destroy(struct pkvm_vm *pkvm_vm);
int pkvm_guest_mmu_refill_memcache(struct pkvm_vcpu *pkvm_vcpu);
void pkvm_guest_mmu_free_memcache(struct pkvm_vcpu *pkvm_vcpu);

#endif /* __PKVM_X86_MMU_H */
