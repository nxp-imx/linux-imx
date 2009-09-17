/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup DVFS Dynamic Voltage and Frequency Scaling (DVFS) Driver
 */

/*!
 * @file arch-mxc/mxc_dvfs.h
 *
 * @brief This file contains the DVFS configuration structure definition.
 *
 *
 * @ingroup DVFS
 */

#ifndef __ASM_ARCH_MXC_DVFS_H__
#define __ASM_ARCH_MXC_DVFS_H__

#ifdef __KERNEL__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/device.h>

#define MXC_GPCCNTR_GPCIRQ2M		(1 << 25)
#define MXC_GPCCNTR_GPCIRQ2		(1 << 24)
#define MXC_GPCCNTR_GPCIRQM		(1 << 21)
#define MXC_GPCCNTR_GPCIRQ_ARM	(1 << 20)
#define MXC_GPCCNTR_GPCIRQ_SDMA	(0 << 20)
#define MXC_GPCCNTR_DVFS0CR		(1 << 16)
#define MXC_GPCCNTR_ADU_MASK		0x8000
#define MXC_GPCCNTR_ADU			(1 << 15)
#define MXC_GPCCNTR_STRT			(1 << 14)
#define MXC_GPCCNTR_FUPD_MASK	0x2000
#define MXC_GPCCNTR_FUPD			(1 << 13)
#define MXC_GPCCNTR_HTRI_MASK		0x0000000F
#define MXC_GPCCNTR_HTRI_OFFSET	0

#define MXC_GPCVCR_VINC_MASK		0x00020000
#define MXC_GPCVCR_VINC_OFFSET	17
#define MXC_GPCVCR_VCNTU_MASK	0x00010000
#define MXC_GPCVCR_VCNTU_OFFSET	16
#define MXC_GPCVCR_VCNT_MASK		0x00007FFF
#define MXC_GPCVCR_VCNT_OFFSET	0

/* DVFS-PER */
#define MXC_DVFSPER_PMCR0_UDCS			(1 << 27)
#define MXC_DVFSPER_PMCR0_UDCS_MASK		0x8000000
#define MXC_DVFSPER_PMCR0_ENABLE_MASK	0x10
#define MXC_DVFSPER_PMCR0_ENABLE			(1 << 4)

/*
 * DVFS structure
 */
struct dvfs_wp {
	int upthr;
	int downthr;
	int panicthr;
	int upcnt;
	int downcnt;
	int emac;
};

#endif				/* __KERNEL__ */

#endif				/* __ASM_ARCH_MXC_DVFS_H__ */
