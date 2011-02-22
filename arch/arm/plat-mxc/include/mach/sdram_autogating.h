/*
 * Copyright 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file arch-mxc/sdram_autogating.h
 *
 * @brief This file contains the SDRAM autogating function prototypes
 *
 *
 * @ingroup PM
 */

#ifndef __ASM_ARCH_SDRAM_AUTOGATING_H__
#define __ASM_ARCH_SDRAM_AUTOGATING_H__

#ifdef __KERNEL__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/device.h>


#ifdef CONFIG_ARCH_MX5
extern void start_sdram_autogating(void);
extern void stop_sdram_autogating(void);
extern int sdram_autogating_active(void);
#else
static inline void start_sdram_autogating(void)
{}

static inline void stop_sdram_autogating(void)
{}

static inline int sdram_autogating_active(void)
{
	return 0;
}
#endif

#endif				/*__KERNEL__ */
#endif				/* __ASM_ARCH_MXC_DVFS_H__ */
