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
