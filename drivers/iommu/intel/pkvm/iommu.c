// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright © 2026 Google.
 */
#include <asm/kvm_pkvm.h>
#include "pkvm/mmu.h"
#include "pkvm/memory.h"
#include "pkvm/pkvm.h"
#include "pkvm/debug.h"
#include "iommu.h"

/*
 * IOMMU supported page size and page levels for second stage page table.
 *
 * Here we set it to the maximum supported values and during IOMMU initialization,
 * we determine the least common values supported by all the IOMMUs in the system.
 */
unsigned int iommu_pgsz_mask = 1 << PG_LEVEL_4K | 1 << PG_LEVEL_2M | 1 << PG_LEVEL_1G;
unsigned int iommu_pglvl_mask = IOMMU_PGT_4LEVEL | IOMMU_PGT_5LEVEL;

/*
 * x86 MSI address base: all legitimate MSI interrupts target this range
 * (Local APIC delivery). FEADDR must have this prefix.
 */
#define X86_MSI_ADDR_BASE		0xfee00000U
/* Mask for 0xFEE and RsvdZ bits 11:4 and 1:0 */
#define X86_MSI_ADDR_MASK		0xfff00ff3U
#define X86_MSI_ADDR_DEST_ID_MASK	GENMASK_U32(19, 12)
#define X86_MSI_ADDR_DEST_ID_SHIFT	12
#define X86_MSI_ADDR_DEST_MODE_LOGICAL	BIT(2)
#define X86_MSI_UADDR_DEST_ID_MASK	GENMASK_U32(31, 8)

/* GCMD bits that handle enabling/disabling of IOMMU features */
#define DMAR_GSTS_EN_BITS	(DMA_GCMD_TE | DMA_GCMD_QIE | DMA_GCMD_IRE | DMA_GCMD_CFI)
/* GCMD oneshot bits where unsetting the bit doesn't have an effect */
#define DMAR_GCMD_ONESHOT	(DMA_GCMD_SRTP | DMA_GCMD_SIRTP)
/* Mask of bits the host is allowed to access directly (passed through to hardware) */
#define DMAR_GCMD_DIRECT	(DMA_GCMD_IRE | DMA_GCMD_CFI)
/* Mask of bits supported by pKVM */
#define DMAR_GCMD_SUPPORTED_BITS	(DMAR_GSTS_EN_BITS | DMA_GCMD_SRTP)

u16 satc_devs[PKVM_MAX_SATC_DEVS];
int nr_satc_devs;

#define PKVM_MAX_IOMMU_NUM	16
static struct intel_iommu iommus[PKVM_MAX_IOMMU_NUM];
static int nr_iommus;

/*
 * Flag denoting if all IOMMUs in the system have page walk coherency support.
 * This is needed to decide whether to flush cpu caches on host ept updates as
 * pKVM uses host ept as the second stage page table when host configures device
 * for passthrough mode. IOMMUs that doesn't have page walk coherency support
 * needs the pagetable updates to be reflected in memory and hence we need to
 * flush cpu caches on host ept update if one or more IOMMUs doesn't have page
 * walk coherency support.
 */
static bool iommu_paging_structure_coherent = true;

bool pkvm_iommu_paging_structure_coherency(void)
{
	return iommu_paging_structure_coherent;
}

bool is_dev_in_satc(u16 bdf)
{
	int i;

	for (i = 0; i < nr_satc_devs; i++) {
		if (bdf == satc_devs[i])
			return true;
	}
	return false;
}

struct intel_iommu *iommu_from_phys(unsigned long phys)
{
	int i;

	for (i = 0; i < nr_iommus; i++) {
		struct intel_iommu *iommu = &iommus[i];

		if (phys >= iommu->reg_phys && phys < (iommu->reg_phys + iommu->reg_size))
			return iommu;
	}

	return NULL;
}

bool overlaps_iommu_mmio(unsigned long phys, unsigned long size)
{
	int i;

	for (i = 0; i < nr_iommus; i++) {
		struct intel_iommu *iommu = &iommus[i];

		if (phys < (iommu->reg_phys + iommu->reg_size) &&
		    (phys + size) > iommu->reg_phys)
			return true;
	}
	return false;
}

static int iommu_direct_mmio_read(struct intel_iommu *iommu, u64 phys,
				  int len, int expected_len, u64 *val)
{
	unsigned long offset = phys - iommu->reg_phys;
	void *reg = iommu->reg + offset;

	if (len > expected_len)
		return -EINVAL;

	switch (len) {
	case 4:
		*val = readl(reg);
		break;
	case 8:
		*val = readq(reg);
		break;
	default:
		pkvm_err("%s: unsupported len %d\n", __func__, len);
		return -EINVAL;
	}
	return 0;
}

static int iommu_direct_mmio_write(struct intel_iommu *iommu, u64 phys,
				   int len, int expected_len, u64 val)
{
	unsigned long offset = phys - iommu->reg_phys;
	void *reg = iommu->reg + offset;

	if (len > expected_len)
		return -EINVAL;

	switch (len) {
	case 4:
		writel((u32)val, reg);
		break;
	case 8:
		writeq(val, reg);
		break;
	default:
		pkvm_err("%s: unsupported len %d\n", __func__, len);
		return -EINVAL;
	}
	return 0;
}

static int handle_gcmd_direct(struct intel_iommu *iommu, u32 gcmd_bit, bool set)
{
	u32 gcmd = readl(iommu->reg + DMAR_GSTS_REG) & DMAR_GSTS_EN_BITS;
	u32 sts;

	if ((gcmd_bit & DMAR_GCMD_ONESHOT) && !set)
		return -EINVAL;

	if (set) {
		if (gcmd & gcmd_bit) {
			iommu->vgsts |= gcmd_bit;
			return 0;
		}
		gcmd |= gcmd_bit;
	} else {
		if (!(gcmd & gcmd_bit)) {
			iommu->vgsts &= ~gcmd_bit;
			return 0;
		}
		gcmd &= ~gcmd_bit;
	}

	writel(gcmd, iommu->reg + DMAR_GCMD_REG);
	if (set) {
		IOMMU_WAIT_OP(iommu, DMAR_GSTS_REG, readl, (sts & gcmd_bit), sts);
		iommu->vgsts |= gcmd_bit;
	} else {
		IOMMU_WAIT_OP(iommu, DMAR_GSTS_REG, readl, !(sts & gcmd_bit), sts);
		iommu->vgsts &= ~gcmd_bit;
	}

	return 0;
}

static int initialize_qi(struct intel_iommu *iommu)
{
	void *desc = pkvm_host_gpa_to_virt(iommu->viqa & VTD_PAGE_MASK);
	u64 desc_sz = ecap_smts(iommu->ecap) ? SZ_8K : SZ_4K;
	struct q_inval *qi = &iommu->_qi;
	u64 val = __pkvm_pa(desc);
	int ret;

	ret = pkvm_host_donate_hyp_share_ro(val, desc_sz, true);
	if (ret) {
		pkvm_err("iommu%d: failed to write protect QI desc!\n", iommu->seq_id);
		return ret;
	}

	iommu->flush.flush_context = qi_flush_context;
	iommu->flush.flush_iotlb = qi_flush_iotlb;

	pkvm_spin_lock_init(&qi->q_lock);
	qi->free_head = qi->free_tail = 0;
	qi->free_cnt = QI_LENGTH;
	qi->desc = desc;

	/*
	 * Set DW=1 and QS=1 in IQA_REG when Scalable Mode capability
	 * is present.
	 */
	if (sm_supported(iommu))
		val |= BIT_ULL(11) | BIT_ULL(0);

	/* write zero to the tail reg */
	writel(0, iommu->reg + DMAR_IQT_REG);
	/* Set IQA */
	writeq(val, iommu->reg + DMAR_IQA_REG);

	/*
	 * QIE is not oneshot bit for enabling thus should not be failed unless
	 * there is a code bug.
	 */
	BUG_ON(handle_gcmd_direct(iommu, DMA_GCMD_QIE, true));

	/*
	 * Host IOMMU driver dynamically allocates iommu->qi, but pKVM has it
	 * embedded. For easy re-use of host code, the embedded field is named
	 * as iommu->_qi, and the pointer iommu->qi points to iommu->_qi.
	 * Also, it serves as a flag to denote whether qi is
	 * enabled(similar to how host driver does)
	 */
	iommu->qi = qi;

	return 0;
}

static int handle_gcmd_qie(struct intel_iommu *iommu, bool enable)
{
	int ret = 0;

	if (enable) {
		if (iommu->qi || iommu->vgsts & DMA_GSTS_QIES) {
			pkvm_err("iommu%d: QI already enabled\n", iommu->seq_id);
			return -EBUSY;
		} else if (!iommu->viqa) {
			pkvm_err("iommu%d: QIE before setting IQA\n", iommu->seq_id);
			return -EINVAL;
		}

		ret = initialize_qi(iommu);
	} else {
		if (!iommu->qi)
			ret = handle_gcmd_direct(iommu, DMA_GCMD_QIE, false);
		else
			iommu->vgsts &= ~DMA_GSTS_QIES;
	}

	pkvm_dbg("iommu%d: Quueued Invalidation %s!\n", iommu->seq_id,
		 enable ? "enabled" : "disabled");
	return ret;
}

static void set_root_table(struct intel_iommu *iommu)
{
	writeq(iommu->vrta, iommu->reg + DMAR_RTADDR_REG);
	handle_gcmd_direct(iommu, DMA_GCMD_SRTP, true);

	if (cap_esrtps(iommu->cap))
		return;

	iommu->flush.flush_context(iommu, 0, 0, 0, DMA_CCMD_GLOBAL_INVL);
	if (sm_supported(iommu))
		qi_flush_pasid_cache(iommu, 0, QI_PC_GLOBAL, 0);
	iommu->flush.flush_iotlb(iommu, 0, 0, 0, DMA_TLB_GLOBAL_FLUSH);
}

static int handle_gcmd_srtp(struct intel_iommu *iommu)
{
	u32 gsts = readl(iommu->reg + DMAR_GSTS_REG);
	u64 root_pa;
	int ret;

	if (WARN_ON(gsts != iommu->vgsts))
		iommu->vgsts = gsts;

	if (!iommu->vrta) {
		pkvm_warn("iommu%d: host RTADDR_REG not set", iommu->seq_id);
		return -EINVAL;
	} else if (iommu->vgsts & DMA_GSTS_TES) {
		pkvm_warn("iommu%d: SRTP not allowed after TE", iommu->seq_id);
		return -EBUSY;
	} else if (iommu->root_entry) {
		pkvm_warn("iommu%d: SRTP allowed only once", iommu->seq_id);
		return -EBUSY;
	} else if (!cap_esrtps(iommu->cap) && !iommu->qi) {
		pkvm_warn("iommu%d: QI is required but not initialized yet", iommu->seq_id);
		return -EINVAL;
	}

	root_pa = pkvm_host_gpa_to_phys(iommu->vrta & VTD_PAGE_MASK);
	ret = pkvm_host_donate_hyp_share_ro(root_pa, VTD_PAGE_SIZE, true);
	if (ret) {
		pkvm_err("iommu%d: failed to write protect root table page(err=%d)!\n",
			 iommu->seq_id, ret);
		return ret;
	}

	iommu->root_entry = __pkvm_va(root_pa);
	__iommu_flush_cache(iommu, iommu->root_entry, VTD_PAGE_SIZE);

	set_root_table(iommu);

	pkvm_dbg("iommu%d Set Root Table(%llx)!\n", iommu->seq_id, iommu->vrta);
	return 0;
}

/*
 * Donate the IR table to the hypervisor as read-only and record its virtual
 * address in iommu->ir_table.
 */
static int iommu_protect_ir_table(struct intel_iommu *iommu)
{
	u64 ir_table_pa;
	int ret;

	if (!iommu->virta) {
		pkvm_err("iommu%d: IR table protection requested before IRTA set\n",
			 iommu->seq_id);
		return -EINVAL;
	}

	ir_table_pa = pkvm_host_gpa_to_phys(iommu->virta & VTD_PAGE_MASK);
	/*
	 * We reach here during IOMMU initialization and IR table is already
	 * setup by host. So, do not clear the table(Host is considered trusted
	 * at this stage)
	 */
	ret = pkvm_host_donate_hyp_share_ro(ir_table_pa, SZ_1M, false);
	if (ret) {
		pkvm_err("iommu%d: failed to write protect IR table[%llx] (err=%d)\n",
			 iommu->seq_id, ir_table_pa, ret);
		return ret;
	}

	iommu->ir_table = __pkvm_va(ir_table_pa);

	return 0;
}

static int handle_gcmd_te(struct intel_iommu *iommu, bool enable)
{
	if (enable) {
		if (iommu->vgsts & DMA_GSTS_TES) {
			pkvm_err("iommu%d: TE allowed only once\n", iommu->seq_id);
			return -EBUSY;
		} else if (!(iommu->vgsts & DMA_GSTS_RTPS)) {
			pkvm_err("iommu%d: TE not allowed before SRTP\n", iommu->seq_id);
			return -EINVAL;
		}

		handle_gcmd_direct(iommu, DMA_GCMD_TE, true);
		pkvm_dbg("iommu%d: Translation enabled!\n", iommu->seq_id);
	} else {
		/*
		 * Translation is not really disabled as it would
		 * compromise pKVM security guarantees.
		 */
		iommu->vgsts &= ~DMA_GSTS_TES;
		pkvm_dbg("iommu%d: Translation marked as disabled!\n", iommu->seq_id);
	}

	return 0;
}

static int handle_global_cmd(struct intel_iommu *iommu, u32 val)
{
	u32 changed = (iommu->vgsts & DMAR_GSTS_EN_BITS) ^ val;

	if (!changed)
		return 0;

	if (hweight32(changed) > 1) {
		pkvm_warn("iommu%d: more than one changed bit in a gcmd write(%x)\n",
			  iommu->seq_id, val);
		return -EINVAL;
	}

	if (changed & ~DMAR_GCMD_SUPPORTED_BITS) {
		pkvm_warn("iommu%d: received GCMD for unsupported bit: %x\n",
			  iommu->seq_id, changed);
		return -EOPNOTSUPP;
	}

	pkvm_dbg("iommu%d: handle gcmd val 0x%x gsts 0x%x changed 0x%x\n",
		 iommu->seq_id, val, iommu->vgsts, changed);

	if (changed & DMA_GCMD_QIE)
		return handle_gcmd_qie(iommu, !!(val & changed));

	if (changed & DMA_GCMD_SRTP)
		return handle_gcmd_srtp(iommu);

	if (changed & DMA_GCMD_TE)
		return handle_gcmd_te(iommu, !!(val & changed));

	/*
	 * Check if the bits are allowed to be directly accessible by the host
	 * and passthrough if so.
	 */
	if (changed & ~DMAR_GCMD_DIRECT) {
		pkvm_warn("iommu%d: direct access of GCMD bit: %x(set=%d) not allowed\n",
			  iommu->seq_id, changed, !!(val & changed));
		return -EPERM;
	}

	return handle_gcmd_direct(iommu, changed, !!(val & changed));
}

int pkvm_iommu_mmio_read(u64 phys, int len, u64 *val)
{
	struct intel_iommu *iommu = iommu_from_phys(phys);
	unsigned long offset;
	int ret = 0;

	if (!iommu)
		return -EINVAL;

	pkvm_spin_lock(&iommu->lock);
	offset = phys - iommu->reg_phys;

	switch (offset) {
	case DMAR_CAP_REG:
		*val = iommu->cap;
		break;
	case DMAR_ECAP_REG:
		*val = iommu->ecap;
		break;
	case DMAR_IQA_REG:
		*val = iommu->viqa;
		break;
	case DMAR_RTADDR_REG:
		*val = iommu->vrta;
		break;
	case DMAR_IRTA_REG:
		*val = iommu->virta;
		break;
	case DMAR_GSTS_REG:
		*val = iommu->vgsts;
		break;
	/* 32-bit registers */
	case DMAR_VER_REG:
	case DMAR_PMEN_REG:
	case DMAR_PRS_REG:
	case DMAR_PERFCFGOFF_REG:
	case DMAR_PERFOVFOFF_REG:
	case DMAR_PERFCNTROFF_REG:
	case DMAR_PERFINTRSTS_REG:
	case DMAR_PERFINTRCTL_REG:
	case DMAR_FSTS_REG:
	case DMAR_FECTL_REG:
#ifdef CONFIG_INTEL_IOMMU_DEBUGFS
	/*
	 * 32-bit registers read only by the Intel IOMMU debugfs register dump
	 * (iommu_regs_32[] in debugfs.c); the core driver never reads them, so
	 * allow them only when debugfs is built (deny-by-default keeps least
	 * privilege in production).
	 */
	case DMAR_GCMD_REG:
	case DMAR_PLMBASE_REG:
	case DMAR_PLMLIMIT_REG:
	case DMAR_ICS_REG:
	case DMAR_PECTL_REG:
	case DMAR_PEDATA_REG:
	case DMAR_PEADDR_REG:
	case DMAR_PEUADDR_REG:
	case DMAR_PERFINTRDATA_REG:
	case DMAR_PERFINTRADDR_REG:
	case DMAR_PERFINTRUADDR_REG:
	case DMAR_FEDATA_REG:
	case DMAR_FEADDR_REG:
	case DMAR_FEUADDR_REG:
#endif
		ret = iommu_direct_mmio_read(iommu, phys, len, 4, val);
		break;
	/* 64-bit registers */
	case DMAR_IQH_REG:
	case DMAR_IQT_REG:
	case DMAR_ECRSP_REG:
	case DMAR_IQER_REG:
	case DMAR_PERFCAP_REG:
	case DMAR_ECCAP_REG:
	case DMAR_ECCAP_REG + DMA_ECMD_REG_STEP:
	case DMAR_ECCAP_REG + 2 * DMA_ECMD_REG_STEP:
	case DMAR_ECCAP_REG + 3 * DMA_ECMD_REG_STEP:
#ifdef CONFIG_INTEL_IOMMU_DEBUGFS
	/*
	 * 64-bit registers read only by the Intel IOMMU debugfs register dump
	 * (iommu_regs_64[] in debugfs.c); same rationale as the 32-bit block.
	 */
	case DMAR_PHMBASE_REG:
	case DMAR_PHMLIMIT_REG:
	case DMAR_PQH_REG:
	case DMAR_PQT_REG:
	case DMAR_PQA_REG:
	case DMAR_MTRRCAP_REG:
	case DMAR_MTRRDEF_REG:
	case DMAR_MTRR_FIX64K_00000_REG:
	case DMAR_MTRR_FIX16K_80000_REG:
	case DMAR_MTRR_FIX16K_A0000_REG:
	case DMAR_MTRR_FIX4K_C0000_REG:
	case DMAR_MTRR_FIX4K_C8000_REG:
	case DMAR_MTRR_FIX4K_D0000_REG:
	case DMAR_MTRR_FIX4K_D8000_REG:
	case DMAR_MTRR_FIX4K_E0000_REG:
	case DMAR_MTRR_FIX4K_E8000_REG:
	case DMAR_MTRR_FIX4K_F0000_REG:
	case DMAR_MTRR_FIX4K_F8000_REG:
	case DMAR_MTRR_PHYSBASE0_REG:
	case DMAR_MTRR_PHYSMASK0_REG:
	case DMAR_MTRR_PHYSBASE1_REG:
	case DMAR_MTRR_PHYSMASK1_REG:
	case DMAR_MTRR_PHYSBASE2_REG:
	case DMAR_MTRR_PHYSMASK2_REG:
	case DMAR_MTRR_PHYSBASE3_REG:
	case DMAR_MTRR_PHYSMASK3_REG:
	case DMAR_MTRR_PHYSBASE4_REG:
	case DMAR_MTRR_PHYSMASK4_REG:
	case DMAR_MTRR_PHYSBASE5_REG:
	case DMAR_MTRR_PHYSMASK5_REG:
	case DMAR_MTRR_PHYSBASE6_REG:
	case DMAR_MTRR_PHYSMASK6_REG:
	case DMAR_MTRR_PHYSBASE7_REG:
	case DMAR_MTRR_PHYSMASK7_REG:
	case DMAR_MTRR_PHYSBASE8_REG:
	case DMAR_MTRR_PHYSMASK8_REG:
	case DMAR_MTRR_PHYSBASE9_REG:
	case DMAR_MTRR_PHYSMASK9_REG:
#endif
		ret = iommu_direct_mmio_read(iommu, phys, len, 8, val);
		break;
	default:
		ret = pkvm_iommu_pmu_validate_read(iommu, offset, len);
		if (ret == IOMMU_REG_NOT_HANDLED)
			ret = pkvm_iommu_frcd_validate_read(iommu, offset, len);

		if (ret == IOMMU_REG_NOT_HANDLED) {
			/*
			 * Deny-by-default: block all registers not explicitly handled
			 * above. Any register the host driver legitimately needs must
			 * be added as an explicit case; unknown or unreviewed registers
			 * must not reach hardware.
			 */
			pkvm_err("iommu%d: unsupported register read blocked at offset 0x%lx\n",
				 iommu->seq_id, offset);
			ret = -EOPNOTSUPP;
		}
		if (!ret)
			ret = iommu_direct_mmio_read(iommu, phys, len, len, val);
	}

	pkvm_spin_unlock(&iommu->lock);
	return ret;
}

int pkvm_iommu_mmio_write(u64 phys, int len, u64 val)
{
	struct intel_iommu *iommu = iommu_from_phys(phys);
	unsigned long offset;
	int ret = 0;

	if (!iommu)
		return -EINVAL;

	pkvm_spin_lock(&iommu->lock);
	offset = phys - iommu->reg_phys;

	switch (offset) {
	case DMAR_CAP_REG:
		fallthrough;
	case DMAR_ECAP_REG:
		fallthrough;
	case DMAR_GSTS_REG:
		fallthrough;
	case DMAR_IQH_REG:
		ret = -EINVAL;
		break;
	case DMAR_GCMD_REG:
		ret = handle_global_cmd(iommu, val);
		break;
	case DMAR_IQA_REG:
		if (iommu->viqa) {
			pkvm_err("iommu%d: IQA set more than once!\n",
				 iommu->seq_id);
			ret = -EINVAL;
		} else {
			iommu->viqa = val;
		}
		break;
	case DMAR_IQT_REG:
		if (iommu->qi) {
			pkvm_err("iommu%d: write to IQT not allowed!\n",
				 iommu->seq_id);
			ret = -EPERM;
		}
		break;
	case DMAR_RTADDR_REG:
		if (sm_supported(iommu) && !(val & DMA_RTADDR_SMT)) {
			pkvm_err("iommu%d: SM enabled but not set in RTA!\n",
				 iommu->seq_id);
			ret = -EINVAL;
		} else if (iommu->vgsts & DMA_GSTS_TES) {
			pkvm_err("iommu%d: Setting RTA after Translation enabled!\n",
				 iommu->seq_id);
			ret = -EBUSY;
		} else {
			iommu->vrta = val;
		}
		break;
	case DMAR_PMEN_REG: {
		/* RsvdP bits: 30:1 */
		u32 rsvdp_mask = GENMASK_U32(30, 1);
		u32 rsvdp = readl(iommu->reg + DMAR_PMEN_REG) & rsvdp_mask;

		if ((val & rsvdp_mask) != rsvdp) {
			pkvm_err("iommu%d: PMEN reserved bits mismatch(0x%x != 0x%x)\n",
				 iommu->seq_id, rsvdp, (u32)(val & rsvdp_mask));
			ret = -EINVAL;
		} else if (val & DMA_PMEN_EPM) {
			/*
			 * pKVM disables PMRs during iommu init. Block any attempt to
			 * re-enable PMRs (EPM bit set) as that could disrupt guest DMA.
			 */
			pkvm_err("iommu%d: attempt to enable PMRs blocked\n",
				 iommu->seq_id);
			ret = -EPERM;
		} else {
			ret = iommu_direct_mmio_write(iommu, phys, len, 4, val);
		}
		break;
	}
	case DMAR_ECEO_REG:
		if (!cap_ecmds(iommu->cap) && val) {
			pkvm_err("iommu%d: non-zero val 0x%llx when ECMD not supported\n",
				 iommu->seq_id, val);
			ret = -EINVAL;
		} else {
			ret = iommu_direct_mmio_write(iommu, phys, len, 8, val);
		}
		break;
	case DMAR_ECMD_REG:
		if (!cap_ecmds(iommu->cap) && val) {
			pkvm_err("iommu%d: non-zero val 0x%llx when ECMD not supported\n",
				 iommu->seq_id, val);
			ret = -EINVAL;
		} else if (val & GENMASK_ULL(15, 8)) {
			pkvm_err("iommu%d: ECMD 0x%llx has reserved bits set\n",
				 iommu->seq_id, val);
			ret = -EINVAL;
		} else {
			/*
			 * Extended Command Interface. Only allow performance monitoring
			 * counter control commands (ENABLE/DISABLE/FREEZE/UNFREEZE).
			 * These toggle counter state only and have no memory or DMA
			 * remapping implications. Block all other command codes as they
			 * are reserved or unknown and cannot be reasoned about safely.
			 *
			 * For simplicity don't validate that the command is supported
			 * in ECCAP3, since per VT-d spec section 11.4.14.3, usage of an
			 * unsupported command shall only result in an error reported in
			 * ECRSP rather than in an undefined behavior.
			 */
			u8 ecmd = val & 0xff;

			if (ecmd != DMA_ECMD_ENABLE && ecmd != DMA_ECMD_DISABLE &&
			    ecmd != DMA_ECMD_FREEZE && ecmd != DMA_ECMD_UNFREEZE) {
				pkvm_err("iommu%d: unsupported ECMD 0x%x blocked\n",
					 iommu->seq_id, ecmd);
				ret = -EPERM;
			} else {
				ret = iommu_direct_mmio_write(iommu, phys, len, 8, val);
			}
		}
		break;
	case DMAR_PERFINTRCTL_REG:
	case DMAR_FECTL_REG: {
		/* RsvdP bits: 29:0 */
		u32 rsvdp_mask = (~0U) >> 2;
		u32 rsvdp;

		if (offset == DMAR_PERFINTRCTL_REG) {
			/* RsvdZ if PMU is not supported */
			if (!ecap_pms(iommu->ecap) && val) {
				ret = -EINVAL;
				break;
			}
		}

		rsvdp = readl(iommu->reg + offset) & rsvdp_mask;
		if ((val & rsvdp_mask) != rsvdp) {
			pkvm_err("iommu%d: %s reserved bits mismatch(0x%x != 0x%x)\n",
				 iommu->seq_id,
				 offset == DMAR_FECTL_REG ? "FECTL" : "PERFINTRCTL",
				 rsvdp, (u32)(val & rsvdp_mask));
			ret = -EINVAL;
		} else {
			ret = iommu_direct_mmio_write(iommu, phys, len, 4, val);
		}
		break;
	}
	case DMAR_PERFINTRSTS_REG:
		if (val & (GENMASK_ULL(31, 1) |
			   (ecap_pms(iommu->ecap) ? 0 : DMA_PERFINTRSTS_PIS))) {
			pkvm_err("iommu%d: PERFINTRSTS 0x%llx has reserved bits set\n",
				 iommu->seq_id, val);
			ret = -EINVAL;
		} else {
			/* RW1C for clearing fault status bits */
			ret = iommu_direct_mmio_write(iommu, phys, len, 4, val);
		}
		break;
	case DMAR_FSTS_REG:
		/*
		 * Bit 7(DMA_FSTS_PRO) is deprecated and RsvdZ (since VT-d spec
		 * Revision 3.1). But host driver still sets this bit. So we
		 * don't validate it here.
		 */
		if (val & (GENMASK_U32(31, 16) | GENMASK_U32(3, 2))) {
			pkvm_err("iommu%d: FSTS 0x%llx has reserved bits set\n",
				 iommu->seq_id, val);
			ret = -EINVAL;
		} else {
			/* RW1C for clearing fault status bits */
			ret = iommu_direct_mmio_write(iommu, phys, len, 4, val);
		}
		break;
	default:
		ret = pkvm_iommu_pmu_validate_write(iommu, offset, len, val);
		if (ret == IOMMU_REG_NOT_HANDLED)
			ret = pkvm_iommu_frcd_validate_write(iommu, offset, len, val);

		if (ret == IOMMU_REG_NOT_HANDLED) {
			/*
			 * Deny-by-default: block all registers not explicitly handled
			 * above. Any register the host driver legitimately needs must
			 * be added as an explicit case; unknown or unreviewed registers
			 * must not reach hardware.
			 */
			pkvm_err("iommu%d: unsupported register write blocked at offset 0x%lx val 0x%llx\n",
				 iommu->seq_id, offset, val);
			ret = -EOPNOTSUPP;
		}
		if (!ret)
			ret = iommu_direct_mmio_write(iommu, phys, len, len, val);
	}

	pkvm_spin_unlock(&iommu->lock);
	return ret;
}

static u32 iommu_msi_dest_id(u32 addr, u32 uaddr)
{
	return ((addr & X86_MSI_ADDR_DEST_ID_MASK) >> X86_MSI_ADDR_DEST_ID_SHIFT) |
	       (uaddr & X86_MSI_UADDR_DEST_ID_MASK);
}

static bool iommu_msi_dest_id_valid(u32 dest_id)
{
	struct pkvm_pcpu *cpu;
	int i;

	for_each_pkvm_pcpu(i, cpu) {
		u32 cpu_dest_id = msi_dest_mode_logical ?
				  cpu->msi_dest_id :
				  cpu->apic_id;

		if (dest_id == cpu_dest_id)
			return true;
	}

	return false;
}

static int iommu_validate_msi_dest_id(struct intel_iommu *iommu, u32 addr,
				      u32 uaddr, const char *name)
{
	u32 dest_id = iommu_msi_dest_id(addr, uaddr);

	if (!iommu_msi_dest_id_valid(dest_id)) {
		pkvm_err("iommu%d: %s MSI destination 0x%x is not a pKVM CPU\n",
			 iommu->seq_id, name, dest_id);
		return -EINVAL;
	}

	return 0;
}

static int iommu_validate_msi_msg(struct intel_iommu *iommu, u32 offset,
				  u32 data, u32 addr, u32 uaddr,
				  const char *name)
{
	if (offset == DMAR_PERFINTRCTL_REG && !ecap_pms(iommu->ecap)) {
		pkvm_err("iommu%d: perf MSI write when PMU is not supported\n",
			 iommu->seq_id);
		return -EINVAL;
	}

	if (data >> 9) {
		pkvm_err("iommu%d: %s DATA 0x%x has reserved bits set\n",
			 iommu->seq_id, name, data);
		return -EINVAL;
	}

	if ((addr & X86_MSI_ADDR_MASK) != X86_MSI_ADDR_BASE) {
		pkvm_err("iommu%d: %s ADDR 0x%x not in MSI address range\n",
			 iommu->seq_id, name, addr);
		return -EINVAL;
	}

	if (!!(addr & X86_MSI_ADDR_DEST_MODE_LOGICAL) != msi_dest_mode_logical) {
		pkvm_err("iommu%d: %s ADDR 0x%x has invalid destination mode\n",
			 iommu->seq_id, name, addr);
		return -EINVAL;
	}

	if (uaddr & 0xFF) {
		pkvm_err("iommu%d: %s UADDR 0x%x has reserved bits set\n",
			 iommu->seq_id, name, uaddr);
		return -EINVAL;
	}

	return iommu_validate_msi_dest_id(iommu, addr, uaddr, name);
}

int pkvm_iommu_msi_write(u64 phys, u32 offset, u32 data, u32 addr, u32 uaddr)
{
	struct intel_iommu *iommu = iommu_from_phys(phys);
	const char *name;
	int ret;

	if (!iommu)
		return -EINVAL;

	switch (offset) {
	case DMAR_FECTL_REG:
		name = "fault event";
		break;
	case DMAR_PERFINTRCTL_REG:
		name = "perf monitoring";
		break;
	default:
		pkvm_err("iommu%d: unsupported MSI register group 0x%x\n",
			 iommu->seq_id, offset);
		return -EOPNOTSUPP;
	}

	ret = iommu_validate_msi_msg(iommu, offset, data, addr, uaddr, name);
	if (ret)
		return ret;

	pkvm_spin_lock(&iommu->lock);
	writel(data, iommu->reg + offset + 4);
	writel(addr, iommu->reg + offset + 8);
	writel(uaddr, iommu->reg + offset + 12);
	pkvm_spin_unlock(&iommu->lock);

	return ret;
}

int __init prepare_iommu(struct intel_iommu_info *info)
{
	struct intel_iommu *iommu;

	if (nr_iommus >= PKVM_MAX_IOMMU_NUM)
		return -ENOMEM;

	iommu = &iommus[nr_iommus++];
	iommu->reg_phys = info->reg_phys;
	iommu->reg_size = info->reg_size;
	iommu->cap = info->cap;
	iommu->ecap = info->ecap;
	iommu->agaw = info->agaw;
	iommu->msagaw = info->msagaw;
	iommu->seq_id = info->seq_id;

	iommu_paging_structure_coherent &= iommu_paging_structure_coherency(iommu);

	return 0;
}

static int iommu_init(struct intel_iommu *iommu)
{
	int ret;

	if (!iommu->reg_phys)
		return -EFAULT;

	iommu->reg = __pkvm_va(iommu->reg_phys);
	ret = pkvm_hyp_mmu_map((unsigned long)iommu->reg, iommu->reg_phys,
			       iommu->reg_size, (u64)pgprot_val(PAGE_KERNEL_IO_NOCACHE));
	if (ret) {
		pkvm_err("iommu%d: failed to map MMIO space in hyp(err=%d)\n",
			 iommu->seq_id, ret);
		return ret;
	}

	ret = pkvm_host_donate_hyp_mmio(iommu->reg_phys, PAGE_ALIGN(iommu->reg_size));
	if (ret) {
		pkvm_err("iommu%d: failed to donate MMIO space to hyp(err=%d)\n",
			 iommu->seq_id, ret);
		return ret;
	}

	pkvm_spin_lock_init(&iommu->lock);

	pkvm_iommu_pmu_init(iommu);

	/*
	 * Take a snapshot of GSTS. GCMD updates will be handled by pKVM and
	 * hence this snapshot will be kept up-to-date by pKVM and used as
	 * virtual GSTS for the host.
	 */
	iommu->vgsts = readl(iommu->reg + DMAR_GSTS_REG);

	/*
	 * Protected Memory Regions (PMRs) are a legacy DMA protection mechanism
	 * used by firmware before DMA remapping is active. Host driver disables
	 * PMRs once DMA remapping is set up. Disable them explicitly to be on
	 * the safer side in case host driver doesn't get to do it.
	 */
	if (cap_plmr(iommu->cap) || cap_phmr(iommu->cap)) {
		u32 pmen = readl(iommu->reg + DMAR_PMEN_REG);

		if (pmen & DMA_PMEN_EPM) {
			pkvm_dbg("iommu%d: disabling PMRs\n", iommu->seq_id);
			pmen &= ~DMA_PMEN_EPM;
			writel(pmen, iommu->reg + DMAR_PMEN_REG);
		}
	}

	/*
	 * Interrupt remapping(IR) hardware initialization happens during x2apic
	 * enable and should happen before pKVM initialization.
	 * Although pKVM itself does not rely on the IR functionality, it relies
	 * on x2apic which requires IR, so IR is supposed to be set up.
	 */
	if (!(iommu->vgsts & DMA_GSTS_IRTPS) || !(iommu->vgsts & DMA_GSTS_IRES)) {
		pkvm_err("iommu%d: Interrupt remapping hardware not setup!\n",
			 iommu->seq_id);
		return -EINVAL;
	}

	iommu->virta = readq(iommu->reg + DMAR_IRTA_REG);
	ret = iommu_protect_ir_table(iommu);
	if (ret)
		return ret;

	return 0;
}

int pkvm_intel_iommu_init(void)
{
	int i;

	for (i = 0; i < nr_iommus; i++) {
		int ret = iommu_init(&iommus[i]);

		if (ret)
			return ret;
	}

	init_pt_domain();

	return 0;
}

void pkvm_iommu_pt_flush(unsigned long paddr, unsigned long size)
{
	if (pt_domain.qi_batch)
		cache_tag_flush_range(&pt_domain, paddr, paddr + size - 1, 0);
}
