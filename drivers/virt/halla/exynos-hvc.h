/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2025 Samsung Electronics.
 *
 * A header for Hypervisor Call(HVC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_HVC_H__
#define __EXYNOS_HVC_H__

#include <linux/arm-smccc.h>

/* HVC FID */
#define HVC_FID_BASE					(0xC6000000)

#define HVC_FID_HVM_CREATE_VM				(HVC_FID_BASE + 0x400)
#define HVC_FID_HVM_DESTROY_VM				(HVC_FID_BASE + 0x401)
#define HVC_FID_HVM_PRE_DESTROY_VM			(HVC_FID_BASE + 0x402)
#define HVC_FID_HVM_CREATE_VCPU				(HVC_FID_BASE + 0x410)
#define HVC_FID_HVM_DESTROY_VCPU			(HVC_FID_BASE + 0x411)
#define HVC_FID_HVM_SET_MEMREGION			(HVC_FID_BASE + 0x412)
#define HVC_FID_HVM_VCPU_RUN				(HVC_FID_BASE + 0x413)
#define HVC_FID_HVM_GET_ONE_REG				(HVC_FID_BASE + 0x414)
#define HVC_FID_HVM_SET_ONE_REG				(HVC_FID_BASE + 0x415)
#define HVC_FID_HVM_INIT_STAGE2_MMU			(HVC_FID_BASE + 0x416)
#define HVC_FID_HVM_REGISTER_VM_PGTABLE			(HVC_FID_BASE + 0x417)
#define HVC_FID_HVM_NEED_VM_PGTABLE			(HVC_FID_BASE + 0x418)
#define HVC_FID_HVM_RECLAIM_GUEST_MEMORY		(HVC_FID_BASE + 0x419)
#define HVC_FID_HVM_RECLAIM_GUEST_S2_PGTABLE		(HVC_FID_BASE + 0x41A)
#define HVC_FID_HVM_IRQ_LINE				(HVC_FID_BASE + 0x420)
#define HVC_FID_HVM_CREATE_DEVICE			(HVC_FID_BASE + 0x421)
#define HVC_FID_HVM_PROBE				(HVC_FID_BASE + 0x430)
#define HVC_FID_HVM_ENABLE_CAP				(HVC_FID_BASE + 0x431)
#define HVC_FID_HVM_NOTIFY_EXIT				(HVC_FID_BASE + 0x432)
#define HVC_FID_HVM_MEMREGION_PURPOSE			(HVC_FID_BASE + 0x433)
#define HVC_FID_HVM_SET_DTB_CONFIG			(HVC_FID_BASE + 0x434)
#define HVC_FID_HVM_MAP_GUEST				(HVC_FID_BASE + 0x440)
#define HVC_FID_HVM_MAP_GUEST_BLOCK			(HVC_FID_BASE + 0x441)
#define HVC_FID_HVM_DEBUG				(HVC_FID_BASE + 0x442)
#define HVC_FID_HYPERVISOR_GET_VM_IDLE_TIME		(HVC_FID_BASE + 0x443)

/* HVC ERROR */
#define HVC_OK						(0x0)
#define HVC_UNK						(-1)

#ifndef	__ASSEMBLY__
#include <linux/types.h>

/* HVC_FID_GET_HALLA_INFO */
enum halla_info_type {
	HALLA_INFO_TYPE_VERSION = 0,
	HALLA_INFO_TYPE_HALLA_BASE,
	HALLA_INFO_TYPE_HALLA_SIZE,
	HALLA_INFO_TYPE_RAM_LOG_BASE,
	HALLA_INFO_TYPE_RAM_LOG_SIZE,
	HALLA_INFO_TYPE_VM_S2_PTDUMP_BASE = 8,
	HALLA_INFO_TYPE_VM_S2_PTDUMP_SIZE
};

static inline unsigned long exynos_hvc(unsigned long hvc_fid,
					unsigned long arg1,
					unsigned long arg2,
					unsigned long arg3,
					unsigned long arg4)
{
	struct arm_smccc_res res;

	arm_smccc_hvc(hvc_fid, arg1, arg2, arg3, arg4, 0, 0, 0, &res);

	return res.a0;
}

#endif	/* __ASSEMBLY__ */

#endif	/* __EXYNOS_HVC_H__ */
