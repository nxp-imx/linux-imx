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

/* GCMD bits that handle enabling/disabling of IOMMU features */
#define DMAR_GSTS_EN_BITS	(DMA_GCMD_TE | DMA_GCMD_QIE | DMA_GCMD_IRE | DMA_GCMD_CFI)
/* GCMD oneshot bits where unsetting the bit doesn't have an effect */
#define DMAR_GCMD_ONESHOT	(DMA_GCMD_SRTP | DMA_GCMD_SIRTP)
/* Mask of bits the host is allowed to access directly (passed through to hardware) */
#define DMAR_GCMD_DIRECT	(DMA_GCMD_IRE | DMA_GCMD_CFI | DMA_GCMD_SIRTP)
/* Mask of bits supported by pKVM */
#define DMAR_GCMD_SUPPORTED_BITS	(DMAR_GSTS_EN_BITS | DMA_GCMD_SRTP | DMA_GCMD_SIRTP)

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
				  int len, u64 *val)
{
	unsigned long offset = phys - iommu->reg_phys;
	void *reg = iommu->reg + offset;

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
				   int len, u64 val)
{
	unsigned long offset = phys - iommu->reg_phys;
	void *reg = iommu->reg + offset;

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
	case DMAR_GCMD_REG:
		ret = -EINVAL;
		break;
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
	case DMAR_GSTS_REG:
		*val = iommu->vgsts;
		break;
	default:
		/* Not emulated MMIO can directly go to hardware */
		ret = iommu_direct_mmio_read(iommu, phys, len, val);
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
	default:
		/* Not emulated MMIO can directly go to hardware */
		ret = iommu_direct_mmio_write(iommu, phys, len, val);
	}

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

	/*
	 * Take a snapshot of GSTS. GCMD updates will be handled by pKVM and
	 * hence this snapshot will be kept up-to-date by pKVM and used as
	 * virtual GSTS for the host.
	 */
	iommu->vgsts = readl(iommu->reg + DMAR_GSTS_REG);

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
