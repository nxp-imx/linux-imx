// SPDX-License-Identifier: GPL-2.0
#include <linux/kvm_host.h>
#include <asm/fpu/xstate.h>
#include <asm/kvm_pkvm.h>
#include "../cpuid.h"
#include "../x86.h"
#include "early_alloc.h"
#include "fpu.h"
#include "init.h"
#include "lapic.h"
#include "memory.h"
#include "mmu.h"
#include "pkvm.h"
#include "trace.h"

/*
 * Set by the host before deprivilege and used through the initialization
 * process.
 */
struct pkvm_init_ops *init_ops;

bool pvmfw_present;
phys_addr_t pvmfw_base;
phys_addr_t pvmfw_size;

static void *hyp_pgt_base;
static void *host_pgt_base;
static void *pkvm_vmemmap_base;
static DEFINE_PER_CPU(bool, cpu_initialized);

static int divide_memory_pool(phys_addr_t phys, unsigned long size)
{
	unsigned long nr_pages;

	pkvm_early_alloc_init(__pkvm_va(phys), size);

	nr_pages = pkvm_hyp_pgtable_pages();
	hyp_pgt_base = pkvm_early_alloc_contig(nr_pages);
	if (!hyp_pgt_base)
		return -ENOMEM;

	nr_pages = pkvm_host_pgtable_pages();
	host_pgt_base = pkvm_early_alloc_contig(nr_pages);
	if (!host_pgt_base)
		return -ENOMEM;

	nr_pages = pkvm_vmemmap_pages(sizeof(struct pkvm_page));
	pkvm_vmemmap_base = pkvm_early_alloc_contig(nr_pages);
	if (!pkvm_vmemmap_base)
		return -ENOMEM;

	return 0;
}

static int back_vmemmap(phys_addr_t back_pa)
{
	unsigned long start_va, size, end_va = 0;
	struct memblock_region *reg;
	unsigned int i;
	int ret;

	/*
	 * Map the vmemmap region at virtual address 64KB.
	 * Keep the lower pages unmapped, to catch NULL dereference bugs in the
	 * hypervisor code.
	 *
	 * 64KB guard hole is chosen as it mimics kernel's mmap_min_addr (which
	 * is typically set to 32KB or 64KB) and should catch most of struct
	 * member offsets.
	 */
	__pkvm_vmemmap = SZ_64K;

	for (i = 0; i < pkvm_memblock_nr; i++) {
		reg = &pkvm_memory[i];
		/* Translate a range of memory to vmemmap range */
		start_va = ALIGN_DOWN((unsigned long)pkvm_phys_to_page(reg->base),
				      PAGE_SIZE);
		/*
		 * The beginning of the pkvm_vmemmap region for the current
		 * memblock may already be backed by the page backing the end
		 * of the previous region, so avoid mapping it twice.
		 */
		start_va = max(start_va, end_va);

		end_va = ALIGN((unsigned long)pkvm_phys_to_page(reg->base + reg->size),
				PAGE_SIZE);
		/*
		 * The vmemmap va shall below PAGE_OFFSET to avoid overlapping
		 * with the pKVM direct mapping
		 */
		if (end_va >= PAGE_OFFSET)
			return -ENOMEM;
		if (start_va >= end_va)
			continue;

		size = end_va - start_va;
		/*
		 * Create mapping for vmemmap virtual address
		 * [start_va, start_va+size) to physical address
		 * [back_pa, back_pa+size).
		 */
		ret = pkvm_hyp_mmu_map(start_va, back_pa, size,
				       (u64)pgprot_val(PAGE_KERNEL));
		if (ret)
			return ret;

		memset(__pkvm_va(back_pa), 0, size);
		back_pa += size;
	}

	return 0;
}

static int create_hyp_mmu(const struct pkvm_mem_info infos[], int nr_infos)
{
	unsigned long nr_pages = pkvm_hyp_pgtable_pages();
	struct memblock_region *reg;
	unsigned int i;
	int ret;

	ret = pkvm_hyp_mmu_init(hyp_pgt_base, nr_pages);
	if (ret)
		return ret;

	/*
	 * Create mapping for the memory in memblocks, which includes all
	 * the memory host kernel can see, as well as the reserved memory
	 * for the pKVM hypervisor.
	 *
	 * The virtual address is the same with the kernel direct mapping.
	 */
	for (i = 0; i < pkvm_memblock_nr; i++) {
		reg = &pkvm_memory[i];
		ret = pkvm_hyp_mmu_map((unsigned long)__pkvm_va(reg->base), reg->base,
				       reg->size, (u64)pgprot_val(PAGE_KERNEL));
		if (ret)
			return ret;
	}

#ifdef CONFIG_PKVM_X86_DEBUG
	/*
	 * Clone host CR3 page mapping starting from VMALLOC_START for the pKVM
	 * hypervisor to run the linux kernel's symbols in the root mode. To
	 * reduce the differences between w/ DEBUG and w/o DEBUG, keep using
	 * the direct mapping created by the pKVM hypervisor itself.
	 */
	pkvm_hyp_mmu_clone_host(VMALLOC_START);
#else
	/*
	 * Map pkvm's TEXT/DATA memory. The virtual address is the same
	 * with the kernel symbol mapping.
	 */
	for (i = 0; i < nr_infos; i++) {
		if (infos[i].type == PKVM_TEXT_DATA) {
			ret = pkvm_hyp_mmu_map(infos[i].va, infos[i].pa,
					       infos[i].size, infos[i].prot);
			if (ret)
				return ret;
		}
	}
#endif

	ret = back_vmemmap(__pkvm_pa(pkvm_vmemmap_base));
	if (ret)
		return ret;

	if (pvmfw_present) {
		ret = pkvm_hyp_mmu_map((unsigned long)__pkvm_va(pvmfw_base),
				       pvmfw_base, pvmfw_size,
				       (u64)pgprot_val(PAGE_KERNEL_RO));
		if (ret)
			return ret;
	}

	if (pkvm_ramoops_console_size) {
		ret = pkvm_hyp_mmu_map((unsigned long)__pkvm_va(pkvm_ramoops_console_pa),
				       pkvm_ramoops_console_pa, pkvm_ramoops_console_size,
				       (u64)pgprot_val(PAGE_KERNEL));
		if (ret)
			return ret;
	}

	/* Load pKVM hypervisor's MMU to use pKVM vmemmap */
	pkvm_hyp_mmu_load();

	/* Update the pKVM mmu to use pKVM buddy allocator */
	return pkvm_hyp_mmu_switch_to_buddy(hyp_pgt_base, nr_pages);
}

static int create_host_mmu(const struct pkvm_mem_info infos[], int nr_infos,
			   host_mmu_init_fn_t host_mmu_init_fn)
{
	return pkvm_host_mmu_init(host_pgt_base, pkvm_host_pgtable_pages(),
				  infos, nr_infos, host_mmu_init_fn);
}

#define TMP_NR_INFOS	16
static int initialize_global(struct pkvm_mem_info infos[], int nr_infos)
{
	host_mmu_init_fn_t host_mmu_init_fn = init_ops ? init_ops->host_mmu_init : NULL;
	hyp_iommu_init_fn_t hyp_iommu_init = init_ops ? init_ops->hyp_iommu_init : NULL;
	hyp_global_init_fn_t hyp_global_init = init_ops ? init_ops->hyp_global_init : NULL;
	struct pkvm_mem_info tmp_infos[TMP_NR_INFOS];
	phys_addr_t mem_base = INVALID_PAGE;
	unsigned long mem_size = 0;
	int i, ret;

	if (!infos || !nr_infos)
		return -EINVAL;

	if (nr_infos > TMP_NR_INFOS)
		return -ENOMEM;

	/*
	 * The passed in parameter infos[] is in linux kernel's stack which
	 * may use VMAP_STACK. It will not be accessible to the pKVM hypervisor
	 * after switching to use its own mmu. Copy the infos[] to tmp_infos[]
	 * which is in its own stack.
	 */
	for (i = 0; i < nr_infos; i++) {
		if (infos[i].type == PKVM_RESERVED_UNUSED_MEMORY) {
			mem_base = infos[i].pa;
			mem_size = infos[i].size;
		}
		tmp_infos[i] = infos[i];
	}

	if (!PAGE_ALIGNED(mem_base) || !mem_size)
		return -EINVAL;

	ret = divide_memory_pool(mem_base, mem_size);
	if (ret)
		return ret;

	ret = create_hyp_mmu(infos, nr_infos);
	if (ret)
		return ret;

	ret = create_host_mmu(tmp_infos, nr_infos, host_mmu_init_fn);
	if (ret)
		return ret;

	pkvm_setup_xstate_cache();

	/*
	 * Initialize KVM cpuid_xstate_sizes to support CPUID emulation for the
	 * guest VMs.
	 */
	kvm_init_xstate_sizes();

	if (hyp_iommu_init) {
		ret = hyp_iommu_init();
		if (ret)
			return ret;
	}

	return hyp_global_init ? hyp_global_init() : 0;
}

int pkvm_init(struct pkvm_mem_info infos[], int nr_infos)
{
	hyp_mmu_finalize_fn_t hyp_mmu_finalize_fn = init_ops ? init_ops->hyp_mmu_finalize :
							       NULL;
	host_mmu_finalize_fn_t host_mmu_finalize_fn = init_ops ? init_ops->host_mmu_finalize :
								 NULL;
	static bool global_initialized;
	int ret;

	if (this_cpu_read(cpu_initialized))
		return -EBUSY;

	if (!global_initialized) {
		ret = initialize_global(infos, nr_infos);
		if (ret)
			return ret;

		global_initialized = true;
	} else {
		/*
		 * The pKVM hypervisor's MMU was already loaded on the first
		 * initialized CPU during the global initialize. Need to load it
		 * on all the other CPUs as well.
		 */
		pkvm_hyp_mmu_load();
	}

	ret = pkvm_lapic_init();
	if (ret)
		return ret;

	ret = pkvm_hyp_mmu_finalize(hyp_mmu_finalize_fn);
	if (ret)
		return ret;

	ret = pkvm_host_mmu_finalize(host_mmu_finalize_fn);
	if (ret)
		return ret;

	ret = pkvm_init_percpu_fpu();
	if (ret)
		return ret;

	pkvm_vcpu_perf_init(this_cpu_read(host_vcpu));

	kvm_user_return_msr_cpu_online();

	this_cpu_write(cpu_initialized, true);
	return 0;
}

/*
 * Flag indicating if pkvm is initialized successfully.
 * Used to enforce internal hypercalls to be unavailable
 * for general use once pkvm is initialized.
 */
static bool pkvm_initialized __ro_after_init;

int pkvm_reprivilege_vcpu(struct kvm_vcpu *vcpu)
{
	if (READ_ONCE(pkvm_initialized))
		return -EPERM;

	if (!init_ops || !init_ops->reprivilege_cpu)
		return -EOPNOTSUPP;

	init_ops->reprivilege_cpu(vcpu->arch.regs);

	/* Reach here only if reprivilege operation fails. */
	return -EFAULT;
}

int pkvm_init_finalize(void)
{
	int cpu;

	if (READ_ONCE(pkvm_initialized))
		return -EPERM;

	for_each_possible_cpu(cpu) {
		if (!per_cpu(cpu_initialized, cpu))
			return -EAGAIN;
	}
	WRITE_ONCE(pkvm_initialized, true);
	if (init_ops)
		init_ops->reprivilege_cpu = NULL;
	/*
	 * TODO: Move reprivilege logic to a separate
	 * section and zero it out here.
	 */

	return 0;
}

bool pkvm_cpu_initialized(int cpu)
{
	return READ_ONCE(per_cpu(cpu_initialized, cpu));
}
