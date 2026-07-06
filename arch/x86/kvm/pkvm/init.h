/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_X86_INIT_H
#define __PKVM_X86_INIT_H

#include <asm/kvm_pkvm.h>
#include "pgtable.h"

typedef int (*hyp_mmu_finalize_fn_t)(struct pkvm_pgtable *pgt);
typedef int (*host_mmu_init_fn_t)(struct pkvm_pgtable *pgt, void *pool_base,
				  unsigned long pool_pages);
typedef int (*host_mmu_finalize_fn_t)(struct pkvm_pgtable *pgt);
typedef int (*hyp_iommu_init_fn_t)(void);
typedef int (*hyp_global_init_fn_t)(void);
typedef void (*reprivilege_cpu_fn_t)(unsigned long *vcpu_regs);

/**
 * pkvm_init_ops - The platform vendor specific pKVM init operations used by the
 *		   pkvm_init. Some operation could be NULL if it is not
 *		   necessary.
 *
 * @hyp_mmu_finalize:	Finalize the hypervisor mmu.
 * @host_mmu_init:	Initialize the host mmu.
 * @host_mmu_finalize:	Finalize the host mmu.
 * @hyp_global_init:	Initialize the hypervisor globally.
 * @reprivilege_cpu:	Switch the cpu back to root mode. Called if deprivilege
 *			or pKVM initialization fails.
 */
struct pkvm_init_ops {
	hyp_mmu_finalize_fn_t		hyp_mmu_finalize;
	host_mmu_init_fn_t		host_mmu_init;
	host_mmu_finalize_fn_t		host_mmu_finalize;
	hyp_iommu_init_fn_t		hyp_iommu_init;
	hyp_global_init_fn_t		hyp_global_init;
	reprivilege_cpu_fn_t		reprivilege_cpu;
};

int pkvm_init(struct pkvm_mem_info infos[], int nr_info);
int pkvm_init_finalize(void);
int pkvm_reprivilege_vcpu(struct kvm_vcpu *vcpu);
bool pkvm_cpu_initialized(int cpu);

/**
 * for_each_pkvm_initialized_cpu - iterate over initialized pKVM host vCPUs
 * @i: iterator variable for vCPU index
 * @vcpu: struct kvm_vcpu pointer to host vCPU, assigned by macro.
 *
 * Iterates over all host vCPUs, skipping the ones whose corresponding CPU is
 * not initialized by pkvm_init() yet.
 */
#define for_each_pkvm_initialized_cpu(i, vcpu)					\
	for ((i) = 0; (i) < pkvm_hyp->num_cpus &&				\
		      ({ (vcpu) = pkvm_hyp->host_vcpus[(i)]; true; }); (i)++)	\
		if (!pkvm_cpu_initialized((vcpu)->cpu))				\
			continue;						\
		else

/**
 * for_each_pkvm_pcpu - iterate over pKVM physical CPUs
 * @i: iterator variable for physical CPU index
 * @pcpu: struct pkvm_pcpu pointer, assigned by macro.
 *
 * NOTE: Iterates over all CPUs, including those that
 * might not be initialized by pkvm_init() yet.
 */
#define for_each_pkvm_pcpu(i, pcpu)						\
	for ((i) = 0; (i) < pkvm_hyp->num_cpus &&				\
		      ({ (pcpu) = pkvm_hyp->pcpus[(i)]; true; }); (i)++)	\

#endif /* __PKVM_X86_INIT_H */
