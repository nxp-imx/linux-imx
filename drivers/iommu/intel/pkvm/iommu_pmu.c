// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Google LLC
 */
#include "pkvm/debug.h"
#include "iommu.h"
#include "perfmon.h"

/*
 * Validation of Performance Monitoring (PMU) registers for pKVM IOMMU MMIO
 * hardening.
 *
 * VT-d PMU registers are split between a fixed register block and register
 * windows whose offsets are reported by hardware through PERFCFGOFF,
 * PERFOVFOFF and PERFCNTROFF (VT-d spec 11.4.13). The host driver does not
 * access the PERFFRZSTS window (it freezes counters via the Enhanced Command
 * Interface), so that window is intentionally not decoded here.
 *
 * The host Intel IOMMU PMU driver uses these windows to discover event
 * capabilities, program counter configuration/filter registers, clear overflow
 * status and read/write counter values.  pKVM allows only those register slots
 * and validates writes against the architectural reserved bits and
 * capability-dependent fields. Capability and status registers that are
 * read-only to software remain read-only here.
 */

/*
 * PMU register classes understood by the pKVM MMIO validator. Some classes
 * identify fixed registers, while others identify entries inside dynamic
 * windows reported by the PMU offset registers.
 */
enum iommu_pmu_reg {
	IOMMU_PMU_REG_NONE,
	IOMMU_PMU_REG_EVENT_CAP,       /* fixed PERFEVNTCAP array */
	IOMMU_PMU_REG_CFG,             /* 64-bit PERFCNTRCFG */
	IOMMU_PMU_REG_FILTER_RID,      /* 32-bit requester-id filter */
	IOMMU_PMU_REG_FILTER_DID,      /* 32-bit domain-id filter */
	IOMMU_PMU_REG_FILTER_PASID,    /* 32-bit PASID filter */
	IOMMU_PMU_REG_FILTER_AT,       /* 32-bit address-type filter */
	IOMMU_PMU_REG_FILTER_PTL,      /* 32-bit page-table-level filter */
	IOMMU_PMU_REG_CNTR_CAP,        /* 32-bit PERFCNTRCAP */
	IOMMU_PMU_REG_CNTR_EVENT_CAP,  /* 32-bit per-counter event cap */
	IOMMU_PMU_REG_OVERFLOW,        /* 64-bit PERFOVFSTS */
	IOMMU_PMU_REG_COUNTER,         /* 64-bit PERFCNTR */
};

struct iommu_pmu_reg_info {
	enum iommu_pmu_reg type;
	u32 counter;
	u32 cntr_width;
	bool cntr_ios;
	u64 writable_mask;
};

/*
 * Verify a PMU register window fits in the remapping hardware MMIO region.
 * The offsets come from hardware registers and must not be trusted blindly.
 */
static bool iommu_pmu_window_in_range(struct intel_iommu *iommu,
				      u32 base, u64 size)
{
	return base <= iommu->reg_size && size <= iommu->reg_size - base;
}

void pkvm_iommu_pmu_init(struct intel_iommu *iommu)
{
	u64 perfcap;
	u32 cntr;

	if (!ecap_pms(iommu->ecap))
		return;

	perfcap = readq(iommu->reg + DMAR_PERFCAP_REG);
	if (!perfcap)
		return;

	/* Clamp the hardware-reported counter count to the pmu_cntrcap[] bound. */
	iommu->pmu_num_cntr = min_t(u32, pcap_num_cntr(perfcap), IOMMU_PMU_IDX_MAX);
	iommu->pmu_num_eg = pcap_num_event_group(perfcap);
	if (!iommu->pmu_num_cntr || !iommu->pmu_num_eg)
		return;

	iommu->pmu_perfcap = perfcap;
	iommu->pmu_cfg = readl(iommu->reg + DMAR_PERFCFGOFF_REG);
	iommu->pmu_overflow = readl(iommu->reg + DMAR_PERFOVFOFF_REG);
	iommu->pmu_counter = readl(iommu->reg + DMAR_PERFCNTROFF_REG);
	iommu->pmu_cntr_width = pcap_cntr_width(perfcap);
	iommu->pmu_cntr_stride = pcap_cntr_stride(perfcap);
	iommu->pmu_filter = pcap_filters_mask(perfcap);

	/*
	 * As per VT-d spec, the window offsets satisfy the following alignment
	 * and range constraints. If they do not, leave pmu_supported clear so
	 * that every PMU register access (including the fixed PERFEVNTCAP block)
	 * is denied by default; such hardware cannot drive a usable PMU anyway.
	 * Warn here to provide context for the resulting denied host PMU accesses.
	 */
	if (!IS_ALIGNED(iommu->pmu_cfg, 256) ||
	    !IS_ALIGNED(iommu->pmu_overflow, 8) ||
	    !IS_ALIGNED(iommu->pmu_counter, iommu->pmu_cntr_stride)) {
		pkvm_warn("iommu%d: PMU window offsets misaligned; PMU disabled\n",
			  iommu->seq_id);
		return;
	}

	if (!iommu_pmu_window_in_range(iommu, iommu->pmu_cfg,
				       iommu->pmu_num_cntr * IOMMU_PMU_CFG_OFFSET) ||
	    !iommu_pmu_window_in_range(iommu, iommu->pmu_overflow, 8) ||
	    !iommu_pmu_window_in_range(iommu, iommu->pmu_counter,
				       (iommu->pmu_num_cntr - 1) *
				       iommu->pmu_cntr_stride + 8)) {
		pkvm_warn("iommu%d: PMU window offsets out of MMIO range; PMU disabled\n",
			  iommu->seq_id);
		return;
	}

	iommu->pmu_supported = true;

	if (perfcap & BIT_ULL(51)) {
		for (cntr = 0; cntr < iommu->pmu_num_cntr; cntr++)
			iommu->pmu_cntrcap[cntr] =
				readl(iommu->reg + iommu->pmu_cfg +
				      cntr * IOMMU_PMU_CFG_OFFSET +
				      IOMMU_PMU_CFG_CNTRCAP_OFFSET);
	}
}

static void iommu_pmu_counter_caps(struct intel_iommu *iommu, u32 counter,
				   struct iommu_pmu_reg_info *info)
{
	info->cntr_width = iommu->pmu_cntr_width;
	info->cntr_ios = pcap_interrupt(iommu->pmu_perfcap);

	/*
	 * When per-counter capabilities are present (PERFCAP.PCCS) and valid
	 * for this counter (PERFCNTRCAP.PCC), override the globals above.
	 */
	if (iommu->pmu_perfcap & BIT_ULL(51)) {
		u32 cap = iommu->pmu_cntrcap[counter];

		if (iommu_cntrcap_pcc(cap)) {
			info->cntr_width = iommu_cntrcap_cw(cap);
			info->cntr_ios = iommu_cntrcap_ios(cap);
		}
	}
}

/**
 * iommu_pmu_get_reg_info - classify an MMIO access and check it targets a PMU reg.
 *
 * Returns true and fills @info when @offset/@len targets a supported PMU
 * register in either the fixed PMU capability block or one of the dynamic PMU
 * windows. Returns false for unsupported access sizes, unaligned offsets, PMU
 * windows outside the MMIO region, and capability-dependent registers that are
 * not reported by hardware.
 */
static bool iommu_pmu_get_reg_info(struct intel_iommu *iommu, unsigned long offset,
				   int len, struct iommu_pmu_reg_info *info)
{
	u32 num_cntr, num_eg, stride;
	u32 cfg, overflow, counter;

	memset(info, 0, sizeof(*info));

	if (!iommu->pmu_supported)
		return false;

	/*
	 * Each PMU register is matched at the exact access width the host PMU
	 * driver uses (8 bytes for PERFCNTRCFG/PERFCNTR/PERFOVFSTS/PERFEVNTCAP,
	 * 4 bytes for the filter and capability registers). Accesses at any
	 * other width are intentionally not recognized here; under
	 * deny-by-default the MMIO handler then rejects them. This is stricter
	 * than the statically-allowed registers (which tolerate narrower reads)
	 * and assumes the host never issues sub-width PMU accesses.
	 */
	num_cntr = iommu->pmu_num_cntr;
	num_eg = iommu->pmu_num_eg;

	/*
	 * Global event capability registers are a fixed 64-bit array starting
	 * at PERFEVNTCAP_REG, one entry per event group.
	 *
	 * Unlike the dynamic windows below, this block is not range-checked
	 * against reg_size: it lives at a fixed architectural offset and num_eg
	 * comes from a 5-bit PERFCAP field, so the array ends at
	 * PERFEVNTCAP_REG + 31 * 8 = 0x478 at most, always within the first
	 * (always-mapped) MMIO page when PMU is supported.
	 */
	if (len == 8 &&
	    offset >= DMAR_PERFEVNTCAP_REG &&
	    offset < DMAR_PERFEVNTCAP_REG + num_eg * IOMMU_PMU_CAP_REGS_STEP &&
	    IS_ALIGNED(offset - DMAR_PERFEVNTCAP_REG, IOMMU_PMU_CAP_REGS_STEP)) {
		info->type = IOMMU_PMU_REG_EVENT_CAP;
		return true;
	}

	cfg = iommu->pmu_cfg;
	overflow = iommu->pmu_overflow;
	counter = iommu->pmu_counter;
	stride = iommu->pmu_cntr_stride;

	if (offset >= cfg && offset < cfg + num_cntr * IOMMU_PMU_CFG_OFFSET) {
		unsigned long cfg_off = offset - cfg;
		u32 reg_off = cfg_off % IOMMU_PMU_CFG_OFFSET;

		/*
		 * Each counter has a 256-byte configuration block. Only the
		 * host driver-used registers in that block are allowed below.
		 */
		info->counter = cfg_off / IOMMU_PMU_CFG_OFFSET;
		iommu_pmu_counter_caps(iommu, info->counter, info);

		if (reg_off == 0) {
			if (len != 8)
				return false;

			info->type = IOMMU_PMU_REG_CFG;
			return true;
		}

		if (len != 4)
			return false;

		switch (reg_off) {
		case IOMMU_PMU_CFG_SIZE:
			if (!(iommu->pmu_filter & IOMMU_PMU_FILTER_REQUESTER_ID))
				return false;
			info->type = IOMMU_PMU_REG_FILTER_RID;
			return true;
		case IOMMU_PMU_CFG_SIZE + IOMMU_PMU_CFG_FILTERS_OFFSET:
			if (!(iommu->pmu_filter & IOMMU_PMU_FILTER_DOMAIN))
				return false;
			info->type = IOMMU_PMU_REG_FILTER_DID;
			return true;
		case IOMMU_PMU_CFG_SIZE + 2 * IOMMU_PMU_CFG_FILTERS_OFFSET:
			if (!(iommu->pmu_filter & IOMMU_PMU_FILTER_PASID))
				return false;
			info->type = IOMMU_PMU_REG_FILTER_PASID;
			return true;
		case IOMMU_PMU_CFG_SIZE + 3 * IOMMU_PMU_CFG_FILTERS_OFFSET:
			if (!(iommu->pmu_filter & IOMMU_PMU_FILTER_ATS))
				return false;
			info->type = IOMMU_PMU_REG_FILTER_AT;
			return true;
		case IOMMU_PMU_CFG_SIZE + 4 * IOMMU_PMU_CFG_FILTERS_OFFSET:
			if (!(iommu->pmu_filter & IOMMU_PMU_FILTER_PAGE_TABLE))
				return false;
			info->type = IOMMU_PMU_REG_FILTER_PTL;
			return true;
		case IOMMU_PMU_CFG_CNTRCAP_OFFSET:
			if (!(iommu->pmu_perfcap & BIT_ULL(51)))
				return false;
			info->type = IOMMU_PMU_REG_CNTR_CAP;
			return true;
		default:
			break;
		}

		/*
		 * Per-counter event capability registers are a 32-bit array
		 * starting at CNTREVCAP_OFFSET. The array length can be
		 * reported by that counter's CNTRCAP.EGCNT field.
		 */
		if (iommu->pmu_perfcap & BIT_ULL(51)) {
			u32 cap = iommu->pmu_cntrcap[info->counter];
			u32 cntr_egcnt = iommu_cntrcap_egcnt(cap);

			if (!iommu_cntrcap_pcc(cap))
				return false;

			if (reg_off >= IOMMU_PMU_CFG_CNTREVCAP_OFFSET &&
			    reg_off < IOMMU_PMU_CFG_CNTREVCAP_OFFSET +
				      cntr_egcnt * IOMMU_PMU_OFF_REGS_STEP &&
			    IS_ALIGNED(reg_off - IOMMU_PMU_CFG_CNTREVCAP_OFFSET,
				       IOMMU_PMU_OFF_REGS_STEP)) {
				info->type = IOMMU_PMU_REG_CNTR_EVENT_CAP;
				return true;
			}
		}

		return false;
	}

	/* Rest of the valid registers are 8 bytes in size */
	if (len != 8)
		return false;

	/*
	 * PERFOVFSTS is a bit vector with one bit per counter and may span
	 * several 8-byte registers (OVF Offset + m * 8). We match only the
	 * first register (m == 0) because num_cntr is capped at
	 * IOMMU_PMU_IDX_MAX (64), so all counters fit in that single 64-bit word.
	 * If IOMMU_PMU_IDX_MAX is ever raised above 64, the additional registers
	 * must be matched here too, otherwise counters >= 64 fall to deny-by-default.
	 */
	if (offset == overflow) {
		info->type = IOMMU_PMU_REG_OVERFLOW;
		/*
		 * PERFOVFSTS has one RW1C bit per counter. Bits beyond the
		 * number of counters are reserved and must remain zero.
		 * num_cntr is capped at IOMMU_PMU_IDX_MAX (64), so a single
		 * 64-bit mask always covers every counter.
		 */
		info->writable_mask = GENMASK_ULL(num_cntr - 1, 0);
		return true;
	}

	/*
	 * Counter value registers: one 8-byte PERFCNTR per counter, spaced
	 * stride bytes apart. Match an exact counter start offset.
	 */
	if (offset >= counter && offset < counter + num_cntr * stride &&
	    (offset - counter) % stride == 0) {
		info->type = IOMMU_PMU_REG_COUNTER;
		info->counter = (offset - counter) / stride;
		iommu_pmu_counter_caps(iommu, info->counter, info);
		return true;
	}

	return false;
}

/*
 * Validate a host MMIO read of a PMU register at @offset/@len.
 *
 * Returns 0 if the access targets a PMU register and is permitted (the
 * caller may pass it to hardware), IOMMU_REG_NOT_HANDLED if @offset is
 * not a PMU register (the caller should try the next
 * handler / deny-by-default), or another negative errno if it is a PMU
 * register but the access is rejected.
 */
int pkvm_iommu_pmu_validate_read(struct intel_iommu *iommu,
				 unsigned long offset, int len)
{
	struct iommu_pmu_reg_info info;

	if (!iommu_pmu_get_reg_info(iommu, offset, len, &info))
		return IOMMU_REG_NOT_HANDLED;

	/* Any recognized PMU register is readable. */
	return 0;
}

/*
 * Validate a host MMIO write of @val to a PMU register at @offset/@len.
 *
 * Returns 0 if the access targets a PMU register and is permitted (the
 * caller may pass it to hardware), IOMMU_REG_NOT_HANDLED if @offset is
 * not a PMU register (the caller should try the next
 * handler / deny-by-default), or another negative errno if it is a PMU
 * register but the access is rejected.
 */
int pkvm_iommu_pmu_validate_write(struct intel_iommu *iommu,
				  unsigned long offset, int len, u64 val)
{
	struct iommu_pmu_reg_info info;
	u32 val32 = (u32)val;

	if (!iommu_pmu_get_reg_info(iommu, offset, len, &info))
		return IOMMU_REG_NOT_HANDLED;

	switch (info.type) {
	case IOMMU_PMU_REG_CFG:
		/*
		 * PERFCNTRCFG is writable, but most bits are reserved and the
		 * IO bit is valid only when reported for this counter.
		 */
		if (val & (GENMASK_ULL(63, 60) | GENMASK_ULL(31, 12) |
			   GENMASK_ULL(7, 3) | BIT_ULL(0))) {
			pkvm_err("iommu%d: PERFCNTRCFG 0x%llx has reserved bits set\n",
				 iommu->seq_id, val);
			return -EINVAL;
		}
		/*
		 * GFO (Global Freeze on Overflow) freezes all counters and drives
		 * the PERFFRZSTS registers, which pKVM does not expose.  The host
		 * never sets it (it freezes via the Enhanced Command Interface),
		 * so reject it rather than allow unused hardware behaviour.
		 */
		if (val & BIT_ULL(2)) {
			pkvm_err("iommu%d: PERFCNTRCFG.GFO not supported\n",
				 iommu->seq_id);
			return -EOPNOTSUPP;
		}
		if ((val & IOMMU_EVENT_CFG_INT) && !info.cntr_ios) {
			pkvm_err("iommu%d: PERFCNTRCFG.IO set without IOS support\n",
				 iommu->seq_id);
			return -EINVAL;
		}
		return 0;
	case IOMMU_PMU_REG_FILTER_RID:
	case IOMMU_PMU_REG_FILTER_DID:
		/* RID/DID filters use bits 15:0 and enable at bit 31. */
		if (val32 & GENMASK(30, 16)) {
			pkvm_err("iommu%d: PERFCNTR ID filter 0x%x has reserved bits set\n",
				 iommu->seq_id, val32);
			return -EINVAL;
		}
		return 0;
	case IOMMU_PMU_REG_FILTER_PASID:
		/* PASID filters use PASID[19:0], PFM[1:0], and enable at bit 31. */
		if (val32 & GENMASK(30, 22)) {
			pkvm_err("iommu%d: PERFCNTR PASID filter 0x%x has reserved bits set\n",
				 iommu->seq_id, val32);
			return -EINVAL;
		}
		if (FIELD_GET(GENMASK(21, 20), val32) == 3) {
			pkvm_err("iommu%d: PERFCNTR PASID filter has reserved PFM encoding\n",
				 iommu->seq_id);
			return -EINVAL;
		}
		return 0;
	case IOMMU_PMU_REG_FILTER_AT:
		/* AT filters use AT[4:0] except bit 3, plus enable at bit 31. */
		if (val32 & (GENMASK(30, 5) | BIT(3))) {
			pkvm_err("iommu%d: PERFCNTR AT filter 0x%x has reserved bits set\n",
				 iommu->seq_id, val32);
			return -EINVAL;
		}
		return 0;
	case IOMMU_PMU_REG_FILTER_PTL:
		/* PTL filters use PTL[4:0] and enable at bit 31. */
		if (val32 & GENMASK(30, 5)) {
			pkvm_err("iommu%d: PERFCNTR PTL filter 0x%x has reserved bits set\n",
				 iommu->seq_id, val32);
			return -EINVAL;
		}
		return 0;
	case IOMMU_PMU_REG_OVERFLOW:
		if (val & ~info.writable_mask) {
			pkvm_err("iommu%d: PERFOVFSTS 0x%llx has reserved bits set\n",
				 iommu->seq_id, val);
			return -EINVAL;
		}
		return 0;
	case IOMMU_PMU_REG_COUNTER:
		/*
		 * Counter registers are RW in hardware. The host PMU driver does
		 * not currently write them (it reads PERFCNTR and accumulates the
		 * delta in software, and resets counters via the Enhanced Command
		 * Interface), but a write is harmless: it only sets this counter's
		 * own value and accesses no memory, so we allow it. Bits above the
		 * counter width are reserved and must stay zero.
		 *
		 * A sane counter is 1..64 bits wide. cntr_width is decoded from
		 * pcap_cntr_width()/iommu_cntrcap_cw(), so a value of 0 or >64 can
		 * only come from a hardware or programming bug; flag it and reject.
		 */
		if (WARN_ON_ONCE(!info.cntr_width || info.cntr_width > 64))
			return -EINVAL;
		if (val & ~GENMASK_ULL(info.cntr_width - 1, 0)) {
			pkvm_err("iommu%d: PERFCNTR 0x%llx has reserved bits set\n",
				 iommu->seq_id, val);
			return -EINVAL;
		}
		return 0;
	case IOMMU_PMU_REG_EVENT_CAP:
	case IOMMU_PMU_REG_CNTR_CAP:
	case IOMMU_PMU_REG_CNTR_EVENT_CAP:
		pkvm_err("iommu%d: write to read-only PMU register blocked\n",
			 iommu->seq_id);
		return -EPERM;
	default:
		pkvm_err("iommu%d: invalid PMU register write\n", iommu->seq_id);
		return -EINVAL;
	}
}
