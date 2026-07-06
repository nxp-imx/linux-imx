// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Google LLC
 */
#include "pkvm/debug.h"
#include "iommu.h"

/*
 * Validation of Fault Recording Registers (FRCD) for pKVM IOMMU MMIO hardening.
 *
 * The VT-d hardware logs non-recoverable DMA faults in a bank of 128-bit Fault
 * Recording Registers whose base offset and count are reported through
 * CAP.FRO and CAP.NFR (see spec section 11.4.2).
 *
 * These registers are architecturally read-only (ROS) for software except for
 * the single Fault (F) bit at position 127, which is RW1CS: software writes 1
 * to acknowledge and clear a logged fault. All other bits are set by hardware
 * and must not be modified by software.
 *
 * The host IOMMU fault handler reads the fault address qword and the
 * source/reason dwords, and clears a logged fault with a 32-bit write of the F
 * bit to the +12 word of an entry; that is all it requires. For completeness
 * pKVM also permits a 64-bit write to the +8 word that sets only the F bit (at
 * bit 63 of the FRCD_HI view), since it is the same RW1CS operation in a wider
 * access. All other writes are denied.
 */

/*
 * Sub-register types within a 128-bit Fault Recording Register (FRCD).
 *
 * Per VT-d spec 11.4.7.6, each FRCD is 16 bytes (128 bits):
 *   +0  (64-bit, FRCD_LO):    FI[63:12] fault address (ROS), RsvdZ[11:0]
 *   +8  (32-bit, FRCD_HI_LO): PP[31], EXE[30], PRIV[29], T2[28],
 *                              SID[15:0] (ROS), RsvdZ[27:16]
 *   +12 (32-bit, FRCD_HI_HI): F[31](RW1CS), T1[30], AT[29:28], PV[27:8],
 *                              FR[7:0] -> F is RW1CS, rest ROS
 *   +8  (64-bit, FRCD_HI):    64-bit access covering FRCD_HI_LO + FRCD_HI_HI
 */
enum iommu_frcd_reg {
	IOMMU_FRCD_REG_NONE,
	IOMMU_FRCD_REG_LO,        /* +0: FI (fault address), all ROS */
	IOMMU_FRCD_REG_HI_LO,     /* +8 (32-bit): SID/PP/EXE/PRIV/T2, all ROS */
	IOMMU_FRCD_REG_HI_HI,     /* +12 (32-bit): F(RW1CS)/T1/AT/PV/FR */
	IOMMU_FRCD_REG_HI,        /* +8 (64-bit): full FRCD_HI word */
};

struct iommu_frcd_reg_info {
	enum iommu_frcd_reg type;
};

/* Each fault recording register is 128 bits (16 bytes) */
#define FRCD_REG_SIZE	16

/* Byte offsets within one 16-byte FRCD entry */
#define FRCD_LO_OFF	0	/* 64-bit: fault address (FI), all ROS */
#define FRCD_HI_OFF	8	/* 64-bit: full HI word */
#define FRCD_HI_LO_OFF	8	/* 32-bit: SID, PP, EXE, PRIV, T2 - all ROS */
#define FRCD_HI_HI_OFF	12	/* 32-bit: F(RW1CS), T1, AT, PV, FR */

/*
 * Verify the entire FRCD array fits within the MMIO window.  The array starts
 * at byte offset @base and holds @num 16-byte entries.
 */
static bool iommu_frcd_window_in_range(struct intel_iommu *iommu,
				       u32 base, u32 num)
{
	u32 size = num * FRCD_REG_SIZE;

	return base <= iommu->reg_size && size <= iommu->reg_size - base;
}

/**
 * iommu_frcd_get_reg_info - classify an MMIO access and check it targets an FRCD.
 *
 * Returns true and fills @info when @offset/@len targets a valid FRCD
 * sub-register; returns false for any access outside the FRCD array or with
 * an unsupported access size or alignment.
 */
static bool iommu_frcd_get_reg_info(struct intel_iommu *iommu, unsigned long offset,
				    int len, struct iommu_frcd_reg_info *info)
{
	u32 fro, num_frcd, sub_off;

	memset(info, 0, sizeof(*info));

	num_frcd = cap_num_fault_regs(iommu->cap);
	fro = cap_fault_reg_offset(iommu->cap);

	/*
	 * fro is the FRCD array's byte offset, reported by CAP and trusted at
	 * init.  Treat fro == 0 as "no FRCD window present": it is the natural
	 * unconfigured value and would otherwise alias the Version Register at
	 * offset 0. This is a cheap sentinel, not a full validity check -- it
	 * does not reject every bogus offset (e.g. 0x10/0x20 would alias
	 * CAP/GCMD). A complete check is unnecessary here: emulated and
	 * allow-listed fixed registers are matched by earlier switch cases
	 * before reaching this classifier, and unknown offsets are denied by
	 * default. (num_frcd is CAP.NFR + 1 and so always >= 1; the test is
	 * kept only for symmetry.)
	 */
	if (!fro || !num_frcd)
		return false;

	if (!iommu_frcd_window_in_range(iommu, fro, num_frcd))
		return false;

	/* Is this offset inside the FRCD array? */
	if (offset < fro || offset >= fro + num_frcd * FRCD_REG_SIZE)
		return false;

	sub_off = (offset - fro) % FRCD_REG_SIZE;

	switch (len) {
	case 4:
		switch (sub_off) {
		case FRCD_HI_LO_OFF:
			info->type = IOMMU_FRCD_REG_HI_LO;
			return true;
		case FRCD_HI_HI_OFF:
			info->type = IOMMU_FRCD_REG_HI_HI;
			return true;
		default:
			return false;
		}
	case 8:
		switch (sub_off) {
		case FRCD_LO_OFF:
			info->type = IOMMU_FRCD_REG_LO;
			return true;
		case FRCD_HI_OFF:
			info->type = IOMMU_FRCD_REG_HI;
			return true;
		default:
			return false;
		}
	default:
		return false;
	}
}

/*
 * Validate a host MMIO read of a fault recording register at @offset/@len.
 *
 * Returns 0 if the access targets an FRCD sub-register and is permitted (the
 * caller may pass it to hardware), IOMMU_REG_NOT_HANDLED if @offset is not an
 * FRCD register (the caller should try the next handler / deny-by-default), or
 * another negative errno if it is an FRCD register but the access is rejected.
 */
int pkvm_iommu_frcd_validate_read(struct intel_iommu *iommu,
				  unsigned long offset, int len)
{
	struct iommu_frcd_reg_info info;

	if (!iommu_frcd_get_reg_info(iommu, offset, len, &info))
		return IOMMU_REG_NOT_HANDLED;

	/* Any recognized FRCD sub-register is readable. */
	return 0;
}

/*
 * Validate a host MMIO write of @val to a fault recording register at @offset/@len.
 *
 * Returns 0 if the access targets an FRCD sub-register and is permitted (the
 * caller may pass it to hardware), IOMMU_REG_NOT_HANDLED if @offset is not an
 * FRCD register (the caller should try the next handler / deny-by-default), or
 * another negative errno if it is an FRCD register but the access is rejected.
 */
int pkvm_iommu_frcd_validate_write(struct intel_iommu *iommu,
				   unsigned long offset, int len, u64 val)
{
	struct iommu_frcd_reg_info info;

	if (!iommu_frcd_get_reg_info(iommu, offset, len, &info))
		return IOMMU_REG_NOT_HANDLED;

	switch (info.type) {
	case IOMMU_FRCD_REG_HI_HI:
		/*
		 * 32-bit write to FRCD_HI_HI (+12). The F bit (bit 31) is
		 * RW1CS: software writes 1 to clear a logged fault. Every
		 * other bit in this word is ROS and must not be touched.
		 */
		if ((u32)val & ~DMA_FRCD_F) {
			pkvm_err("iommu%d: FRCD write 0x%llx sets non-F bits\n",
				 iommu->seq_id, val);
			return -EINVAL;
		}
		return 0;
	case IOMMU_FRCD_REG_HI:
		/*
		 * 64-bit write to FRCD_HI (+8). This qword is the upper half of
		 * the 128-bit record, so the F bit (DMA_FRCD_F, bit 31 of the +12
		 * dword) lands at bit 63 here. The lower 32 bits cover the
		 * SID/PP/EXE/PRIV/T2 fields which are all ROS.
		 */
		if (val & ~((u64)DMA_FRCD_F << 32)) {
			pkvm_err("iommu%d: FRCD 64-bit write 0x%llx sets non-F bits\n",
				 iommu->seq_id, val);
			return -EINVAL;
		}
		return 0;
	case IOMMU_FRCD_REG_LO:
	case IOMMU_FRCD_REG_HI_LO:
		/*
		 * FRCD_LO (fault address) and FRCD_HI_LO (SID/PP/EXE/PRIV/T2)
		 * are entirely read-only (ROS).  Block all writes.
		 */
		pkvm_err("iommu%d: write to read-only FRCD register blocked\n",
			 iommu->seq_id);
		return -EPERM;
	default:
		pkvm_err("iommu%d: invalid fault recording register write\n",
			 iommu->seq_id);
		return -EINVAL;
	}
}
