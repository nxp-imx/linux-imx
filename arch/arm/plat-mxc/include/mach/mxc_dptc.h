/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup DPTC Dynamic Process and Temperatur Compensation (DPTC) Driver
 */

/*!
 * @file arch-mxc/mxc_dptc.h
 *
 * @brief This file contains the DPTC configuration structure definition.
 *
 *
 * @ingroup DPTC
 */

#ifndef __ASM_ARCH_MXC_DPTC_H__
#define __ASM_ARCH_MXC_DPTC_H__

#ifdef __KERNEL__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/device.h>

#define DPTC_WP_SUPPORTED	17
#define DPTC_GP_WP_SUPPORTED	7
#define DPTC_LP_WP_SUPPORTED	9

struct dptc_wp {
	u32 dcvr0;
	u32 dcvr1;
	u32 dcvr2;
	u32 dcvr3;
	u32 voltage;
};

/*!
 * This structure is used to define the dptc controller's platform
 * data. It includes the regulator name string and DPTC clock name string.
 */
struct mxc_dptc_data {
	/** Regulator name string */
	char *reg_id;
	/* DPTC clock name string */
	char *clk_id;
	/* Control reg address */
	unsigned int dptccr_reg_addr;
	/* Comparator value reg 0 address */
	unsigned int dcvr0_reg_addr;
	/* GPC control reg address */
	unsigned int gpc_cntr_reg_addr;
	/* DPTC interrupt status bit */
	unsigned int dptccr;
	/* The number of DPTC working points */
	unsigned int dptc_wp_supported;
	/* Maximum value of DPTC clock rate */
	unsigned long clk_max_val;
	/* DPTC working points */
	struct dptc_wp *dptc_wp_allfreq;
	/* DPTC enable bit */
	u32 dptc_enable_bit;
	/* DPTC ADU bit */
	int gpc_adu;
	/* VAI mask */
	u32 vai_mask;
	/* VAI offset */
	int vai_offset;
	/* Mask DPTC interrupt */
	u32 irq_mask;
	/* DPTC no voltage change request bit */
	u32 dptc_nvcr_bit;
	/* ARM interrrupt bit */
	u32 gpc_irq_bit;
	/* dptc init config */
	u32 init_config;
	/* dptc enable config */
	u32 enable_config;
	/* dptc counting range mask */
	u32 dcr_mask;
};

/*!
 * This function is called to put the DPTC in a low power state.
 *
 * @param   id   The DPTC device id. DPTC_GP_ID is for DPTC GP;
 *               DPTC_LP_ID is for DPTC LP
 */
void dptc_suspend(int id);
/*!
 * This function is called to resume the DPTC from a low power state.
 *
 * @param   id   The DPTC device id. DPTC_GP_ID is for DPTC GP;
 *               DPTC_LP_ID is for DPTC LP
 */
void dptc_resume(int id);

#endif				/* __KERNEL__ */

#endif				/* __ASM_ARCH_MXC_DPTC_H__ */
