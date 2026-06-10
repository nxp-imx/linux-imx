// SPDX-License-Identifier: GPL-2.0
#include <linux/kvm_host.h>
#include <linux/bitfield.h>
#include <asm/vmx.h>
#include <vmx/capabilities.h>
#include <mmu/spte.h>
#include <vmx/vmx.h>
#include <vmx/vmx_ops.h>
#include "debug.h"
#include "ept.h"
#include "gfp.h"
#include "pkvm/mmu.h"
#include "pkvm.h"
#include "pkvm_iommu.h"

/* The ignored bit56 ~ bit52 in EPT present leaf are used to store page state */
#define EPT_PAGE_STATE_SHIFT			52
#define EPT_PAGE_STATE_MAX_BITS			5
#define EPT_PAGE_STATE_MASK								\
	GENMASK(EPT_PAGE_STATE_SHIFT + PKVM_PAGE_STATE_BITS - 1, EPT_PAGE_STATE_SHIFT)

static struct pkvm_pgtable *host_ept;
static struct pkvm_pool host_ept_pool;

u64 pkvm_host_ept_root(void)
{
	BUG_ON(!host_ept);
	return host_ept->root_pa;
}

int pkvm_host_ept_level(void)
{
	BUG_ON(!host_ept);
	return host_ept->cap.level;
}

static void *host_ept_zalloc_page(struct pkvm_memcache *mc)
{
	return pkvm_alloc_pages(&host_ept_pool, 0);
}

static void host_ept_get_page(void *vaddr)
{
	pkvm_get_page(&host_ept_pool, vaddr);
}

static void host_ept_put_page(void *vaddr)
{
	pkvm_put_page(&host_ept_pool, vaddr);
}

static int host_ept_page_count(void *vaddr)
{
	return pkvm_page_count(vaddr);
}

static const struct pkvm_pgtable_mm_ops host_ept_mm_ops = {
	.zalloc_page = host_ept_zalloc_page,
	.get_page = host_ept_get_page,
	.put_page = host_ept_put_page,
	.page_count = host_ept_page_count,
};

static bool ept_pte_present(void *ptep)
{
	u64 val = *(u64 *)ptep;

	return !!(val & VMX_EPT_RWX_MASK);
}

static bool ept_pte_annotated(void *ptep)
{
	return !ept_pte_present(ptep) && *(u64 *)ptep != 0;
}

static bool ept_pte_huge(void *ptep)
{
	return is_large_pte(*(u64 *)ptep);
}

static void ept_pte_mkhuge(void *ptep)
{
	*(u64 *)ptep |= PT_PAGE_SIZE_MASK;
}

static bool ept_pte_young(void *ptep)
{
	u64 val = READ_ONCE(*(u64 *)ptep);

	return !!(val & VMX_EPT_ACCESS_BIT);
}

static void ept_pte_mkold(void *ptep)
{
	atomic64_and(~VMX_EPT_ACCESS_BIT, (atomic64_t *)ptep);
}

static unsigned long ept_pte_to_phys(void *ptep)
{
	return *(u64 *)ptep & SPTE_BASE_ADDR_MASK;
}

static u64 ept_pte_to_prot(void *ptep)
{
	u64 prot = *(u64 *)ptep & ~(SPTE_BASE_ADDR_MASK);

	return prot & ~PT_PAGE_SIZE_MASK;
}

static u64 ept_calc_pte_perm(bool read, bool write, bool exec)
{
	u64 prot = 0;

	if (read)
		prot |= VMX_EPT_READABLE_MASK;
	if (write)
		prot |= VMX_EPT_WRITABLE_MASK;
	if (exec)
		prot |= VMX_EPT_EXECUTABLE_MASK;

	return prot;
}

static u64 ept_calc_pte_memtype(bool mmio)
{
	return mmio ? X86_MEMTYPE_UC << VMX_EPT_MT_EPTE_SHIFT :
		      X86_MEMTYPE_WB << VMX_EPT_MT_EPTE_SHIFT;
}

static int ept_pte_to_index(unsigned long vaddr, int level)
{
	return SPTE_INDEX(vaddr, level);
}

static unsigned long ept_level_to_size(int level)
{
	return KVM_HPAGE_SIZE(level);
}

static u64 ept_level_to_mask(int level)
{
	return (~((1UL << SPTE_LEVEL_SHIFT(level)) - 1));
}

static bool ept_pte_is_leaf(void *ptep, int level)
{
	return level == PG_LEVEL_4K || !ept_pte_present(ptep) || ept_pte_huge(ptep);
}

static int ept_pte_size(int level)
{
	return PAGE_SIZE / SPTE_ENT_PER_PAGE;
}

static int ept_pte_count(int level)
{
	return SPTE_ENT_PER_PAGE;
}

static void ept_pte_set(void *ptep, u64 spte)
{
	WRITE_ONCE(*(u64 *)ptep, spte);
}

static void host_ept_pte_set(void *ptep, u64 spte)
{
	ept_pte_set(ptep, spte);

	/*
	 * TODO: no need to flush cache if none of the
	 * devices behind non-coherent IOMMUs are configured
	 * for passthrough mode.
	 */
	if (!pkvm_iommu_paging_structure_coherency())
		clflush_cache_range(ptep, sizeof(u64));
}

static void guest_ept_pte_set(void *ptep, u64 spte)
{
	ept_pte_set(ptep, spte);
}

static u64 ept_pte_get(void *ptep)
{
	return READ_ONCE(*(u64 *)ptep);
}

static void host_ept_flush_tlb(struct pkvm_pgtable *pgt,
			       unsigned long vaddr, unsigned long size)
{
	struct kvm_vcpu *vcpu;
	int i;

	/*
	 * During pKVM initialization phase, the host EPT will unmap the
	 * pKVM's memory pages which can trigger TLB flushing on each
	 * CPU. During this phase, a CPU may not be initialized yet to
	 * respond to this request. In fact, it is also not necessary to
	 * trigger TLB flushing for that CPU as the EPT will be flushed
	 * eventually on that CPU when the CPU initialization is done.
	 */
	for_each_pkvm_initialized_cpu(i, vcpu) {
		kvm_make_request(KVM_REQ_TLB_FLUSH_CURRENT, vcpu);
		pkvm_kick_vcpu(vcpu);
	}

	pkvm_iommu_pt_flush(vaddr, size);

	for_each_pkvm_initialized_cpu(i, vcpu)
		pkvm_wait_vcpu_kicked_out(vcpu);
}

static void guest_ept_flush_tlb(struct pkvm_pgtable *pgt,
				unsigned long vaddr, unsigned long size)
{
	struct pkvm_vm *pkvm_vm = pgt_to_pkvm(pgt);
	struct kvm_vcpu *vcpu;
	int i;

	pkvm_spin_lock(&pkvm_vm->lock);

	for_each_pkvm_guest_vcpu(i, vcpu, pkvm_vm) {
		kvm_make_request(KVM_REQ_TLB_FLUSH_CURRENT, vcpu);
		pkvm_kick_vcpu(vcpu);
	}

	for_each_pkvm_guest_vcpu(i, vcpu, pkvm_vm)
		pkvm_wait_vcpu_kicked_out(vcpu);

	pkvm_spin_unlock(&pkvm_vm->lock);
}

static u64 ept_pgstate_mask(void)
{
	return EPT_PAGE_STATE_MASK;
}

static u64 ept_pte_mk_pgstate(enum pkvm_page_state state)
{
	/*
	 * As the ignored bit56 ~ bit52 in the EPT present leaf are used to
	 * store page state, the number of page state bits should not be larger
	 * than this.
	 */
	BUILD_BUG_ON_MSG(EPT_PAGE_STATE_MAX_BITS < PKVM_PAGE_STATE_BITS,
			"EPT doesn't have enough bits to store page state");

	return FIELD_PREP(EPT_PAGE_STATE_MASK, state);
}

static enum pkvm_page_state ept_pte_pgstate(void *ptep)
{
	return (ept_pte_get(ptep) & EPT_PAGE_STATE_MASK) >> EPT_PAGE_STATE_SHIFT;
}

static const struct pkvm_pgtable_ops host_ept_pgt_ops = {
	.pte_present = ept_pte_present,
	.pte_annotated = ept_pte_annotated,
	.pte_huge = ept_pte_huge,
	.pte_mkhuge = ept_pte_mkhuge,
	.pte_to_phys = ept_pte_to_phys,
	.pte_to_prot = ept_pte_to_prot,
	.calc_pte_perm = ept_calc_pte_perm,
	.calc_pte_memtype = ept_calc_pte_memtype,
	.vaddr_to_index = ept_pte_to_index,
	.level_to_size = ept_level_to_size,
	.level_to_mask = ept_level_to_mask,
	.pte_is_leaf = ept_pte_is_leaf,
	.pte_size = ept_pte_size,
	.pte_count = ept_pte_count,
	.pte_set = host_ept_pte_set,
	.pte_get = ept_pte_get,
	.flush_tlb = host_ept_flush_tlb,
	.pte_mk_pgstate = ept_pte_mk_pgstate,
	.pte_pgstate = ept_pte_pgstate,
	.pgstate_mask = ept_pgstate_mask,
};

static const struct pkvm_pgtable_ops guest_ept_pgt_ops = {
	.pte_present = ept_pte_present,
	.pte_annotated = ept_pte_annotated,
	.pte_huge = ept_pte_huge,
	.pte_mkhuge = ept_pte_mkhuge,
	.pte_young = ept_pte_young,
	.pte_mkold = ept_pte_mkold,
	.pte_to_phys = ept_pte_to_phys,
	.pte_to_prot = ept_pte_to_prot,
	.calc_pte_perm = ept_calc_pte_perm,
	.calc_pte_memtype = ept_calc_pte_memtype,
	.vaddr_to_index = ept_pte_to_index,
	.level_to_size = ept_level_to_size,
	.level_to_mask = ept_level_to_mask,
	.pte_is_leaf = ept_pte_is_leaf,
	.pte_size = ept_pte_size,
	.pte_count = ept_pte_count,
	.pte_set = guest_ept_pte_set,
	.pte_get = ept_pte_get,
	.flush_tlb = guest_ept_flush_tlb,
	.pte_mk_pgstate = ept_pte_mk_pgstate,
	.pte_pgstate = ept_pte_pgstate,
	.pgstate_mask = ept_pgstate_mask,
};

static inline u64 construct_host_eptp(struct pkvm_pgtable *pgt)
{
	u64 eptp = pgt->root_pa | VMX_EPTP_MT_WB;

	eptp |= (pgt->cap.level == 5) ? VMX_EPTP_PWL_5 : VMX_EPTP_PWL_4;

	return eptp;
}

int pkvm_host_ept_init(struct pkvm_pgtable *pgt, void *pool_base,
		       unsigned long pool_pages)
{
	unsigned long pfn = __pkvm_pa(pool_base) >> PAGE_SHIFT;
	struct pkvm_pgtable_cap cap = {
		.level = 4,
		.allowed_pgsz = 1 << PG_LEVEL_4K,
		.table_prot = VMX_EPT_RWX_MASK,
		.flush_tlb_lazy = true,
	};
	int ret;

	if (!cpu_has_vmx_ept_4levels() ||
	    !cpu_has_vmx_ept_mt_wb() ||
	    !cpu_has_vmx_invept_global())
		return -EOPNOTSUPP;

	ret = pkvm_pool_init(&host_ept_pool, pfn, pool_pages, 0);
	if (ret)
		return ret;

	if (cpu_has_vmx_ept_2m_page() && iommu_supports_2m_page())
		cap.allowed_pgsz |=  1 << PG_LEVEL_2M;
	if (cpu_has_vmx_ept_1g_page() && iommu_supports_1g_page())
		cap.allowed_pgsz |=  1 << PG_LEVEL_1G;

	if (cpu_has_vmx_ept_5levels() && iommu_supports_5levels())
		cap.level = 5;

	ret = pkvm_pgtable_init(pgt, cap, &host_ept_mm_ops, &host_ept_pgt_ops);
	if (ret)
		return ret;

	host_ept = pgt;
	return 0;
}

int pkvm_host_ept_finalize(struct pkvm_pgtable *pgt)
{
	struct kvm_vcpu *hvcpu = this_cpu_read(host_vcpu);

	/* The EPT finalizing should be done after EPT initialization */
	if (!host_ept || pgt != host_ept)
		return -EINVAL;

	secondary_exec_controls_setbit(to_vmx(hvcpu), SECONDARY_EXEC_ENABLE_EPT);
	vmcs_write64(EPT_POINTER, construct_host_eptp(host_ept));

	/* enable vpid */
	if ((host_vmcs_config.cpu_based_2nd_exec_ctrl & SECONDARY_EXEC_ENABLE_VPID) &&
	    (cpu_has_vmx_invvpid_single() || cpu_has_vmx_invvpid_global()) &&
	    cpu_has_vmx_invvpid()) {
		/* Derive vpid from vcpu_id by adding 1 to ensure vpid is non-zero */
		int vpid = hvcpu->vcpu_id + 1;

		if (vpid > 0 && vpid < VMX_NR_VPIDS) {
			/*
			 * Fixed VPIDs for the host vCPUs, which implies that it
			 * could conflict with VPIDs from guest VMs.
			 *
			 * It's safe because cached mappings used in non-root
			 * mode are associated with EPTRTA, and the host's EPT
			 * root never changes and is different from any guest's
			 * EPT root.
			 */
			vmcs_write16(VIRTUAL_PROCESSOR_ID, vpid);
			secondary_exec_controls_setbit(to_vmx(hvcpu),
						       SECONDARY_EXEC_ENABLE_VPID);
		}
	}

	ept_sync_global();
	/*
	 * Clear the pending TLB flush request left after updating host EPT
	 * mappings in initialize_global(), as EPT has just been flushed with
	 * global context anyway.
	 */
	kvm_clear_request(KVM_REQ_TLB_FLUSH_CURRENT, hvcpu);

	/*
	 * Re-enable the SECONDARY_EXEC_PT_USE_GPA bit if support Intel PT
	 * which was disabled together with the EPT before deprivileging.
	 */
	if (boot_cpu_has(X86_FEATURE_INTEL_PT))
		secondary_exec_controls_setbit(to_vmx(hvcpu), SECONDARY_EXEC_PT_USE_GPA);

	return 0;
}

static void handle_host_ept_violation_failure(struct kvm_vcpu *vcpu)
{
	unsigned long exit_qualification = vmx_get_exit_qual(vcpu);
	struct x86_exception fault = { 0 };
	u64 error_code = 0;

	/*
	 * The guest linear address(GLA) should be valid for injecting #PF.
	 * But the GLA may be invalid according to SDM vol. 3 Exit Qualification
	 * for EPT Violations:
	 * "The guest linear-address field is valid for all EPT violations
	 * except those resulting from an attempt to load the guest PDPTEs as
	 * part of the execution of the MOV CR instruction and those due to
	 * trace-address pre-translation (TAPT; Section 27.5.4)."
	 *
	 * For loading guest PDPTEs via MOV CR, SDM Page-Fault Exceptions:
	 * "Failures to load the PDPTE registers with PAE paging (see Section
	 * 5.4.1) cause general-protection exceptions (#GP(0)) and not
	 * page-fault exceptions.", #GP(0) can be injected.
	 *
	 * For TAPT, SDM IA32_RTIT_OUTPUT_BASE MSR:
	 * "however, if any of those bits are set outside SEAM, trace output may
	 * fail in a model-specific manner." Similar for pKVM, injecting #GP(0)
	 * can be pKVM specific manner.
	 */
	if (!(exit_qualification & EPT_VIOLATION_GVA_IS_VALID)) {
		kvm_inject_gp(vcpu, 0);
		return;
	}

	error_code |= (exit_qualification & EPT_VIOLATION_PROT_MASK)
		      ? PFERR_PRESENT_MASK : 0;
	error_code |= (exit_qualification & EPT_VIOLATION_ACC_WRITE)
		      ? PFERR_WRITE_MASK : 0;
	error_code |= VMX_AR_DPL(vmcs_read32(GUEST_SS_AR_BYTES)) == 3
		      ? PFERR_USER_MASK : 0;
	error_code |= (exit_qualification & EPT_VIOLATION_ACC_INSTR)
		      ? PFERR_FETCH_MASK : 0;

	fault.vector = PF_VECTOR;
	fault.error_code_valid = true;
	fault.error_code = error_code;
	fault.address = vmcs_readl(GUEST_LINEAR_ADDRESS);
	fault.exit_qualification = exit_qualification;

	kvm_inject_page_fault(vcpu, &fault);
}

void pkvm_handle_host_ept_violation(struct kvm_vcpu *vcpu)
{
	struct range range, cur;
	int level, ret = -EPERM;
	unsigned long hpa, gpa;

	BUG_ON(!host_ept);

	gpa = vmcs_read64(GUEST_PHYSICAL_ADDRESS);
	/*
	 * All the memory and MMIO holes have already been mapped in the host
	 * EPT when initialize, except for the MMIO in the high-end address.
	 * Handle the MMIO only.
	 */
	if (pkvm_find_addr_range(gpa, &range) || is_pvmfw(gpa) ||
	    is_iommu_mmio(gpa)) {
		pkvm_err_ratelimited("Host access to protected memory at 0x%lx\n", gpa);
		goto failed;
	}

	pkvm_host_mmu_lock();

	pkvm_pgtable_lookup(host_ept, gpa, &hpa, NULL, &level);
	if (VALID_PAGE(hpa)) {
		/*
		 * Some other CPU has just mapped in the host EPT. Vmenter
		 * the host VM and retry.
		 */
		pkvm_host_mmu_unlock();
		return;
	}

	BUG_ON(level > host_ept->cap.level);
	/*
	 * Based on the looked up level, find out the possible maximum page-size
	 * aligned address range which contains the GPA address for installing
	 * the MMIO mapping, to minimize the memory consumption.
	 */
	for (; level > PG_LEVEL_NONE; level--) {
		unsigned long size;

		if (!((1 << level) & host_ept->cap.allowed_pgsz))
			continue;

		size = ept_level_to_size(level);
		cur.start = ALIGN_DOWN(gpa, size);
		cur.end = cur.start + size - 1;

		if (range_contains(&range, &cur) && !overlaps_pvmfw(cur.start, size) &&
		    !overlaps_iommu_mmio(cur.start, size)) {
			/*
			 * TODO: In case the host mmu free pages are not
			 * enough, -ENOMEM will be returned. This could be
			 * handled by unmaping some other MMIO mapped for the
			 * host VM to reclaim some mmu pages and try again.
			 */
			ret = pkvm_hyp_donate_host_mmio_locked(cur.start, size);
			break;
		}
	}

	if (ret)
		pkvm_err("No valid range found to map host GPA 0x%lx, err %d\n", gpa, ret);

	pkvm_host_mmu_unlock();

	if (likely(!ret))
		return;
failed:
	handle_host_ept_violation_failure(vcpu);
}

void pkvm_flush_host_ept(void)
{
	if (WARN_ON(!host_ept))
		return;

	ept_sync_context(construct_host_eptp(host_ept));
}

void pkvm_guest_ept_setup(void)
{
	struct pkvm_pgtable_cap cap = {
		.level = cpu_has_vmx_ept_5levels() ? 5 : 4,
		.allowed_pgsz = 1 << PG_LEVEL_4K,
		.table_prot = VMX_EPT_RWX_MASK,
		.flush_tlb_lazy = true,
	};

	if (cpu_has_vmx_ept_2m_page())
		cap.allowed_pgsz |= 1 << PG_LEVEL_2M;
	if (cpu_has_vmx_ept_1g_page())
		cap.allowed_pgsz |= 1 << PG_LEVEL_1G;

	pkvm_guest_mmu_setup(&guest_ept_pgt_ops, cap);
}
