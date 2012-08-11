/*
 * Copyright (C) 2009-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

extern void __iomem *gpc_base;
extern void __iomem *ccm_base;

#define MXC_GPCCNTR_GPCIRQ2M		(1 << 25)
#define MXC_GPCCNTR_GPCIRQ2		(1 << 24)
#define MXC_GPCCNTR_GPCIRQM		(1 << 21)
#define MXC_GPCCNTR_GPCIRQ_ARM	(1 << 20)
#define MXC_GPCCNTR_GPCIRQ_SDMA	(0 << 20)
#define MXC_GPCCNTR_DVFS0CR		(1 << 16)
#define MXC_GPCCNTR_DVFS1CR		(1 << 17)
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

#define MXC_DVFSLTR0_UPTHR_MASK		0x0FC00000
#define MXC_DVFSLTR0_UPTHR_OFFSET	22
#define MXC_DVFSLTR0_DNTHR_MASK		0x003F0000
#define MXC_DVFSLTR0_DNTHR_OFFSET	16

#define MXC_DVFSLTR1_PNCTHR_MASK	0x0000003F
#define MXC_DVFSLTR1_PNCTHR_OFFSET	0
#define MXC_DVFSLTR1_DNCNT_MASK		0x003FC000
#define MXC_DVFSLTR1_DNCNT_OFFSET	14
#define MXC_DVFSLTR1_UPCNT_MASK		0x00003FC0
#define MXC_DVFSLTR1_UPCNT_OFFSET	6
#define MXC_DVFSLTR1_LTBRSR		0x800000
#define MXC_DVFSLTR1_LTBRSH		0x400000

#define MXC_DVFSLTR2_EMAC_MASK		0x000001FF
#define MXC_DVFSLTR2_EMAC_OFFSET	0

#define MXC_DVFSPMCR0_UDCS		0x8000000
#define MXC_DVFSPMCR0_DVFEV		0x800000
#define MXC_DVFSPMCR0_DVFIS		0x400000
#define MXC_DVFSPMCR0_LBMI		0x200000
#define MXC_DVFSPMCR0_LBFL		0x100000
#define MXC_DVFSPMCR0_LBFC_MASK		0xC0000
#define MXC_DVFSPMCR0_LBFC_OFFSET	18
#define MXC_DVFSPMCR0_FSVAIM		0x00008000
#define MXC_DVFSPMCR0_FSVAI_MASK	0x00006000
#define MXC_DVFSPMCR0_FSVAI_OFFSET	13
#define MXC_DVFSPMCR0_WFIM		0x00000400
#define MXC_DVFSPMCR0_WFIM_OFFSET	10
#define MXC_DVFSPMCR0_DVFEN		0x00000010

#define MXC_DVFSPMCR1_P1INM		0x00100000
#define MXC_DVFSPMCR1_P1ISM		0x00080000
#define MXC_DVFSPMCR1_P1IFM		0x00040000
#define MXC_DVFSPMCR1_P4PM		0x00020000
#define MXC_DVFSPMCR1_P2PM		0x00010000

/* DVFS CORE register offsets*/
#define MXC_DVFSCORE_THRS		0x00
#define MXC_DVFSCORE_COUN		0x04
#define MXC_DVFSCORE_SIG1		0x08
#define MXC_DVFSCORE_SIG0		0x0C
#define MXC_DVFSCORE_GPC0		0x10
#define MXC_DVFSCORE_GPC1		0x14
#define MXC_DVFSCORE_GPBT		0x18
#define MXC_DVFSCORE_EMAC		0x1C
#define MXC_DVFSCORE_CNTR		0x20
#define MXC_DVFSCORE_LTR0_0		0x24
#define MXC_DVFSCORE_LTR0_1		0x28
#define MXC_DVFSCORE_LTR1_0		0x2C
#define MXC_DVFSCORE_LTR1_1		0x30
#define MXC_DVFSCORE_PT0 		0x34
#define MXC_DVFSCORE_PT1 		0x38
#define MXC_DVFSCORE_PT2 		0x3C
#define MXC_DVFSCORE_PT3 		0x40

/*
 * DVFS structure
 */
struct dvfs_op {
	int upthr;
	int downthr;
	int panicthr;
	int upcnt;
	int downcnt;
	int emac;
};

struct mxc_dvfs_platform_data {
	/** Supply voltage regulator name string */
	char *reg_id;
	/*vdd_soc regulator name string*/
	char *soc_id;
	/*vdd_pu regulator name string*/
	char *pu_id;
	/* CPU clock name string */
	char *clk1_id;
	/* DVFS clock name string */
	char *clk2_id;
	/* The base address of the DVFS core */
	void __iomem *membase;
	/* The interrupt number used by the DVFS core */
	int irq;
	/* GPC control reg offset */
	int gpc_cntr_offset;
	/* GPC voltage counter reg offset */
	int gpc_vcr_offset;
	/* CCM DVFS control reg offset */
	int ccm_cdcr_offset;
	/* CCM ARM clock root reg offset */
	int ccm_cacrr_offset;
	/* CCM divider handshake in-progress reg offset */
	int ccm_cdhipr_offset;
	/* PREDIV mask */
	u32 prediv_mask;
	/* PREDIV offset */
	int prediv_offset;
	/* PREDIV value */
	int prediv_val;
	/* DIV3CK mask */
	u32 div3ck_mask;
	/* DIV3CK offset */
	int div3ck_offset;
	/* DIV3CK value */
	int div3ck_val;
	/* EMAC value */
	int emac_val;
	/* Frequency increase threshold. Increase frequency change request
	   will be sent if DVFS counter value will be more than this value */
	int upthr_val;
	/* Frequency decrease threshold. Decrease frequency change request
	   will be sent if DVFS counter value will be less than this value */
	int dnthr_val;
	/* Panic threshold. Panic frequency change request
	   will be sent if DVFS counter value will be more than this value */
	int pncthr_val;
	/* The amount of times the up threshold should be exceeded
	   before DVFS will trigger frequency increase request */
	int upcnt_val;
	/* The amount of times the down threshold should be exceeded
	   before DVFS will trigger frequency decrease request */
	int dncnt_val;
	/* Delay time in us */
	int delay_time;
};

/*!
 * This structure is used to define the dvfs controller's platform
 * data. It includes the regulator name string and DVFS clock name string.
 */
struct mxc_dvfsper_data {
	/** Regulator name string */
	char *reg_id;
	/* DVFS clock name string */
	char *clk_id;
	/* The base address of the DVFS per */
	void __iomem *membase;
	/* GPC control reg address */
	void __iomem *gpc_cntr_reg_addr;
	/* GPC VCR reg address */
	void __iomem *gpc_vcr_reg_addr;
	/* DVFS enable bit */
	u32 dvfs_enable_bit;
	/* DVFS ADU bit */
	int gpc_adu;
	/* VAI mask */
	u32 vai_mask;
	/* VAI offset */
	int vai_offset;
	/* Mask DVFS interrupt */
	u32 irq_mask;
	/* Div3 clock offset. */
	u32 div3_offset;
	/*div3 clock mask. */
	u32 div3_mask;
	/*div3 clock divider */
	u32 div3_div;
	/* LP voltage - high setpoint*/
	u32 lp_high;
	/* LP voltage - low setpoint*/
	u32 lp_low;
};

/*!
 * This structure is used to define the platform data of bus freq
 * driver. It includes the regulator name strings.
 */
struct mxc_regulator_platform_data {
	/* VDDGP regulator name */
	char *cpu_reg_id;
	/* VCC regulator name */
	char *vcc_reg_id;
};

#if defined(CONFIG_MXC_DVFS_PER)
extern int start_dvfs_per(void);
extern void stop_dvfs_per(void);
extern int dvfs_per_active(void);
extern int dvfs_per_divider_active(void);
extern int dvfs_per_pixel_clk_limit(void);
#else
static inline int start_dvfs_per(void)
{
	return 0;
}

static inline void stop_dvfs_per(void)
{
}

static inline int dvfs_per_active(void)
{
	return 0;
}

static inline int dvfs_per_divider_active(void)
{
	return 0;
}

static inline int dvfs_per_pixel_clk_limit(void)
{
	return 0;
}

#endif

#endif				/* __KERNEL__ */

#endif				/* __ASM_ARCH_MXC_DVFS_H__ */
