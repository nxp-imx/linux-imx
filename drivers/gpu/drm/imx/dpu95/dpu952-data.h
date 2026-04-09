/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 NXP
 */

#ifndef __DRM_DPU952_DATA_H__
#define __DRM_DPU952_DATA_H__

#include "dpu95.h"
#include <linux/regmap.h>

/* register QoS_Setting_Ext2 in blk-ctrl */
#define QOS_SETTING_EXT2		0x2c
#define EXT_QOS_BE_EN_MASK		0x40000000
#define EXT_QOS_BE_EN(n)		(((n) & 0x1) << 30)

/* register CMDSEQ QOS_SETTING in blk-ctrl */
#define CMDSEQ_PANIC_AWQOS_MASK		0x70000000
#define CMDSEQ_PANIC_AWQOS(n)		(((n) & 0x7) << 28)
#define CMDSEQ_AWQOS_MASK		0x7000000
#define CMDSEQ_AWQOS(n)		(((n) & 0x7) << 24)

/* Constant Frame */
static const unsigned int dpu952_cf_ids[] = {0, 1, 4, 5};
static const enum dpu95_unit_type dpu952_cf_types[] = {
DPU95_DISP, DPU95_DISP, DPU95_DISP, DPU95_DISP};
static const unsigned long dpu952_cf_ofss[] = {
0xf0000, 0x130000, 0x100000, 0x140000};
static const unsigned long dpu952_cf_aux_ofss[] = {
0xf1000, 0x131000, 0x101000, 0x141000};

static const struct dpu95_units dpu952_dpu_cfs = {
	.ids		= dpu952_cf_ids,
	.types		= dpu952_cf_types,
	.ofss		= dpu952_cf_ofss,
	.aux_ofss	= dpu952_cf_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu952_cf_ids),
	.name		= DPU95_CONSTFRAME,
	.init		= dpu95_cf_init,
	.hw_init	= dpu95_cf_hw_init,
};

/* Domain Blend */
static const unsigned int dpu952_db_ids[] = {0, 1};
static const enum dpu95_unit_type dpu952_db_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long dpu952_db_ofss[] = {0x2a0000, 0x330000};

static const struct dpu95_units dpu952_dpu_dbs = {
	.ids		= dpu952_db_ids,
	.types		= dpu952_db_types,
	.ofss		= dpu952_db_ofss,
	.cnt		= ARRAY_SIZE(dpu952_db_ids),
	.name		= DPU95_DOMAINBLEND,
	.init		= dpu95_db_init,
	.hw_init	= dpu95_db_hw_init,
};

/* Dither */
static const unsigned int dpu952_dt_ids[] = {0, 1};
static const enum dpu95_unit_type dpu952_dt_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long dpu952_dt_ofss[] = {0x2f0000, 0x380000};
static const unsigned long dpu952_dt_aux_ofss[] = {0x2f1000, 0x381000};

static const struct dpu95_units dpu952_dpu_dts = {
	.ids		= dpu952_dt_ids,
	.types		= dpu952_dt_types,
	.ofss		= dpu952_dt_ofss,
	.aux_ofss	= dpu952_dt_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu952_dt_ids),
	.name		= DPU95_DITHER,
	.init		= dpu95_dt_init,
	.hw_init	= dpu95_dt_hw_init,
};

/* External Destination */
static const unsigned int dpu952_ed_ids[] = {0, 1, 4, 5};
static const enum dpu95_unit_type dpu952_ed_types[] = {
DPU95_DISP, DPU95_DISP, DPU95_DISP, DPU95_DISP};
static const unsigned long dpu952_ed_ofss[] = {
0x110000, 0x150000, 0x120000, 0x160000};
static const unsigned long dpu952_ed_aux_ofss[] = {
0x111000, 0x151000, 0x121000, 0x161000};

static const struct dpu95_units dpu952_dpu_eds = {
	.ids		= dpu952_ed_ids,
	.types		= dpu952_ed_types,
	.ofss		= dpu952_ed_ofss,
	.aux_ofss	= dpu952_ed_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu952_ed_ids),
	.name		= DPU95_EXTDST,
	.init		= dpu95_ed_init,
	.hw_init	= dpu95_ed_hw_init,
};

/* Fetch ECO */
static const unsigned int dpu952_fe_ids[] = {0, 1, 9};
static const enum dpu95_unit_type dpu952_fe_types[] = {
DPU95_DISP, DPU95_DISP, DPU95_BLIT};
static const unsigned long dpu952_fe_ofss[] = {
0x200000, 0x220000, 0xa0000};
static const unsigned long dpu952_fe_aux_ofss[] = {
0x201000, 0x221000, 0xa1000};

static const struct dpu95_units dpu952_dpu_fes = {
	.ids		= dpu952_fe_ids,
	.types		= dpu952_fe_types,
	.ofss		= dpu952_fe_ofss,
	.aux_ofss	= dpu952_fe_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu952_fe_ids),
	.name		= DPU95_FETCHECO,
	.init		= dpu95_fe_init,
	.hw_init	= dpu95_fe_hw_init,
};

/* Frame Generator */
static const unsigned int dpu952_fg_ids[] = {0, 1};
static const enum dpu95_unit_type dpu952_fg_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long dpu952_fg_ofss[] = {0x2b0000, 0x340000};

static const struct dpu95_units dpu952_dpu_fgs = {
	.ids		= dpu952_fg_ids,
	.types		= dpu952_fg_types,
	.ofss		= dpu952_fg_ofss,
	.cnt		= ARRAY_SIZE(dpu952_fg_ids),
	.name		= DPU95_FRAMEGEN,
	.init		= dpu95_fg_init,
	.hw_init	= dpu95_fg_hw_init,
};

/* Fetch Layer */
static const unsigned int dpu952_fl_ids[] = {0, 1};
static const enum dpu95_unit_type dpu952_fl_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long dpu952_fl_ofss[] = {0x1c0000, 0x1d0000};
static const unsigned long dpu952_fl_aux_ofss[] = {0x1c1000, 0x1d1000};

static const struct dpu95_units dpu952_dpu_fls = {
	.ids		= dpu952_fl_ids,
	.types		= dpu952_fl_types,
	.ofss		= dpu952_fl_ofss,
	.aux_ofss	= dpu952_fl_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu952_fl_ids),
	.name		= DPU95_FETCHLAYER,
	.init		= dpu95_fl_init,
	.hw_init	= dpu95_fl_hw_init,
};

/* Fetch YUV */
static const unsigned int dpu952_fy_ids[] = {0, 1, 3};
static const enum dpu95_unit_type dpu952_fy_types[] = {
DPU95_DISP, DPU95_DISP, DPU95_DISP};
static const unsigned long dpu952_fy_ofss[] = {
0x1f0000, 0x210000, 0x1e0000};
static const unsigned long dpu952_fy_aux_ofss[] = {
0x1f1000, 0x211000, 0x1e1000};

static const struct dpu95_units dpu952_dpu_fys = {
	.ids		= dpu952_fy_ids,
	.types		= dpu952_fy_types,
	.ofss		= dpu952_fy_ofss,
	.aux_ofss	= dpu952_fy_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu952_fy_ids),
	.name		= DPU95_FETCHYUV,
	.init		= dpu95_fy_init,
	.hw_init	= dpu95_fy_hw_init,
};

/* Horizontal Scaler */
static const unsigned int dpu952_hs_ids[] = {4, 9};
static const enum dpu95_unit_type dpu952_hs_types[] = {DPU95_DISP, DPU95_BLIT};
static const unsigned long dpu952_hs_ofss[] = {0x240000, 0xb0000};
static const unsigned long dpu952_hs_aux_ofss[] = {0x241000, 0xb1000};

static const struct dpu95_units dpu952_dpu_hss = {
	.ids		= dpu952_hs_ids,
	.types		= dpu952_hs_types,
	.ofss		= dpu952_hs_ofss,
	.aux_ofss	= dpu952_hs_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu952_hs_ids),
	.name		= DPU95_HSCALER,
	.init		= dpu95_hs_init,
	.hw_init	= dpu95_hs_hw_init,
};

/* Layer Blend */
static const unsigned int dpu952_lb_ids[] = {1, 2, 3, 4, 5};
static const enum dpu95_unit_type dpu952_lb_types[] = {
DPU95_DISP, DPU95_DISP, DPU95_DISP, DPU95_DISP, DPU95_DISP};
static const unsigned long dpu952_lb_ofss[] = {
0x170000, 0x180000, 0x190000, 0x1a0000, 0x1b0000};
static const unsigned long dpu952_lb_aux_ofss[] = {
0x171000, 0x181000, 0x191000, 0x1a1000, 0x1b1000};

static const struct dpu95_units dpu952_dpu_lbs = {
	.ids		= dpu952_lb_ids,
	.types		= dpu952_lb_types,
	.ofss		= dpu952_lb_ofss,
	.aux_ofss	= dpu952_lb_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu952_lb_ids),
	.name		= DPU95_LAYERBLEND,
	.init		= dpu95_lb_init,
	.hw_init	= dpu95_lb_hw_init,
};

/* Local Dimming */
static const unsigned int dpu952_ld_id[] = {0};
static const enum dpu95_unit_type dpu952_ld_type[] = {DPU95_DISP};
static const unsigned long dpu952_ld_ofss[] = {0x320000};

static const struct dpu95_units dpu952_dpu_ld = {
	.ids		= dpu952_ld_id,
	.types		= dpu952_ld_type,
	.ofss		= dpu952_ld_ofss,
	.cnt		= ARRAY_SIZE(dpu952_ld_id),
	.name		= DPU95_LOCALDIMMING,
	.init		= dpu95_ld_init,
	.hw_init	= dpu95_ld_hw_init,
};

/* Vertical Scaler */
static const unsigned int dpu952_vs_ids[] = {4, 9};
static const enum dpu95_unit_type dpu952_vs_types[] = {DPU95_DISP, DPU95_BLIT};
static const unsigned long dpu952_vs_ofss[] = {0x250000, 0xc0000};
static const unsigned long dpu952_vs_aux_ofss[] = {0x251000, 0xc1000};

static const struct dpu95_units dpu952_dpu_vss = {
	.ids		= dpu952_vs_ids,
	.types		= dpu952_vs_types,
	.ofss		= dpu952_vs_ofss,
	.aux_ofss	= dpu952_vs_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu952_vs_ids),
	.name		= DPU95_VSCALER,
	.init		= dpu95_vs_init,
	.hw_init	= dpu95_vs_hw_init,
};

static const struct dpu95_units *dpu952_all_units[] = {
	&dpu952_dpu_cfs,
	&dpu952_dpu_dbs,
	&dpu952_dpu_dts,
	&dpu952_dpu_eds,
	&dpu952_dpu_fes,
	&dpu952_dpu_fgs,
	&dpu952_dpu_fls,
	&dpu952_dpu_fys,
	&dpu952_dpu_hss,
	&dpu952_dpu_lbs,
	&dpu952_dpu_ld,
	&dpu952_dpu_vss,
};

static const enum dpu95_link_id dpu952_link_id_map[] = {
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_STORE9,
	DPU95_LINK_ID_EXTDST0,
	DPU95_LINK_ID_EXTDST4,
	DPU95_LINK_ID_EXTDST1,
	DPU95_LINK_ID_EXTDST5,
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_FETCHECO9,
	DPU95_LINK_ID_HSCALER9,
	DPU95_LINK_ID_VSCALER9,
	DPU95_LINK_ID_FILTER9,
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_CONSTFRAME0,
	DPU95_LINK_ID_CONSTFRAME4,
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_CONSTFRAME1,
	DPU95_LINK_ID_CONSTFRAME5,
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_LAYERBLEND1,
	DPU95_LINK_ID_LAYERBLEND2,
	DPU95_LINK_ID_LAYERBLEND3,
	DPU95_LINK_ID_LAYERBLEND4,
	DPU95_LINK_ID_LAYERBLEND5,
	DPU95_LINK_ID_NONE,
	0x19,	/* FETCHLAYER0 */
	0x1a,	/* FETCHLAYER1 */
	0x1b,	/* FETCHYUV3 */
	0x1c,	/* FETCHYUV0 */
	0x1d,	/* FETCHECO0 */
	0x1e,	/* FETCHYUV1 */
	0x1f,	/* FETCHECO1 */
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_NONE,
	0x20,	/* MATRIX4 */
	0x21,	/* HSCALER4 */
	0x22,	/* VSCALER4 */
};

static const enum dpu95_link_id dpu952_link_id_fy[] = {
	DPU95_LINK_ID_FETCHYUV0, DPU95_LINK_ID_FETCHYUV1,
	DPU95_LINK_ID_FETCHYUV3,
};

static const enum dpu95_link_id dpu952_link_id_fe[] = {
	DPU95_LINK_ID_FETCHECO0, DPU95_LINK_ID_FETCHECO1,
	DPU95_LINK_ID_FETCHECO9,
};

enum dpu952_irq {
	DPU952_IRQ_STORE9_SHDLOAD		= 0,
	DPU952_IRQ_STORE9_FRAMECOMPLETE		= 1,
	DPU952_IRQ_STORE9_SEQCOMPLETE		= 2,
	DPU952_IRQ_EXTDST0_SHDLOAD		= 3,
	DPU952_IRQ_EXTDST0_FRAMECOMPLETE	= 4,
	DPU952_IRQ_EXTDST0_SEQCOMPLETE		= 5,
	DPU952_IRQ_EXTDST4_SHDLOAD		= 6,
	DPU952_IRQ_EXTDST4_FRAMECOMPLETE	= 7,
	DPU952_IRQ_EXTDST4_SEQCOMPLETE		= 8,
	DPU952_IRQ_EXTDST1_SHDLOAD		= 9,
	DPU952_IRQ_EXTDST1_FRAMECOMPLETE	= 10,
	DPU952_IRQ_EXTDST1_SEQCOMPLETE		= 11,
	DPU952_IRQ_EXTDST5_SHDLOAD		= 12,
	DPU952_IRQ_EXTDST5_FRAMECOMPLETE	= 13,
	DPU952_IRQ_EXTDST5_SEQCOMPLETE		= 14,
	DPU952_IRQ_DOMAINBLEND0_SHDLOAD		= 15,
	DPU952_IRQ_DOMAINBLEND0_FRAMECOMPLETE	= 16,
	DPU952_IRQ_DISENGCFG_SHDLOAD0		= 17,
	DPU952_IRQ_DISENGCFG_FRAMECOMPLETE0	= 18,
	DPU952_IRQ_DISENGCFG_SEQCOMPLETE0	= 19,
	DPU952_IRQ_FRAMEGEN0_INT0		= 20,
	DPU952_IRQ_FRAMEGEN0_INT1		= 21,
	DPU952_IRQ_FRAMEGEN0_INT2		= 22,
	DPU952_IRQ_FRAMEGEN0_INT3		= 23,
	DPU952_IRQ_SIG0_SHDLOAD			= 24,
	DPU952_IRQ_SIG0_VALID			= 25,
	DPU952_IRQ_SIG0_ERROR			= 26,
	DPU952_IRQ_SIG0_CLUSTER_ERROR		= 27,
	DPU952_IRQ_SIG0_CLUSTER_MATCH		= 28,
	DPU952_IRQ_IDHASH0_SHDLOAD		= 29,
	DPU952_IRQ_IDHASH0_VALID		= 30,
	DPU952_IRQ_IDHASH0_WINDOWN_ERROR	= 31,
	DPU952_IRQ_LOCALDIMMING0_IRQ0		= 32,
	DPU952_IRQ_LOCALDIMMING0_IRQ1		= 33,
	DPU952_IRQ_LOCALDIMMING0_IRQ2		= 34,
	DPU952_IRQ_DOMAINBLEND1_SHDLOAD		= 35,
	DPU952_IRQ_DOMAINBLEND1_FRAMECOMPLETE	= 36,
	DPU952_IRQ_DISENGCFG_SHDLOAD1		= 37,
	DPU952_IRQ_DISENGCFG_FRAMECOMPLETE1	= 38,
	DPU952_IRQ_DISENGCFG_SEQCOMPLETE1	= 39,
	DPU952_IRQ_FRAMEGEN1_INT0		= 40,
	DPU952_IRQ_FRAMEGEN1_INT1		= 41,
	DPU952_IRQ_FRAMEGEN1_INT2		= 42,
	DPU952_IRQ_FRAMEGEN1_INT3		= 43,
	DPU952_IRQ_SIG1_SHDLOAD			= 44,
	DPU952_IRQ_SIG1_VALID			= 45,
	DPU952_IRQ_SIG1_ERROR			= 46,
	DPU952_IRQ_SIG1_CLUSTER_ERROR		= 47,
	DPU952_IRQ_SIG1_CLUSTER_MATCH		= 48,
	DPU952_IRQ_IDHASH1_SHDLOAD		= 49,
	DPU952_IRQ_IDHASH1_VALID		= 50,
	DPU952_IRQ_IDHASH1_WINDOWN_ERROR	= 51,
	DPU952_IRQ_CMDSEQ_ERROR			= 52,
	DPU952_IRQ_COMCTRL_SW0			= 53,
	DPU952_IRQ_COMCTRL_SW1			= 54,
	DPU952_IRQ_COMCTRL_SW2			= 55,
	DPU952_IRQ_COMCTRL_SW3			= 56,
	DPU952_IRQ_FRAMEGEN0_PRIMSYNC_ON	= 57,
	DPU952_IRQ_FRAMEGEN0_PRIMSYNC_OFF	= 58,
	DPU952_IRQ_FRAMEGEN0_OVERFLOW0_ON	= 59,
	DPU952_IRQ_FRAMEGEN0_OVERFLOW0_OFF	= 60,
	DPU952_IRQ_FRAMEGEN0_UNDERRUN0_ON	= 61,
	DPU952_IRQ_FRAMEGEN0_UNDERRUN0_OFF	= 62,
	DPU952_IRQ_FRAMEGEN0_THRESHOLD0_RISE	= 63,
	DPU952_IRQ_FRAMEGEN0_THRESHOLD0_FAIL	= 64,
	DPU952_IRQ_FRAMEGEN0_OVERFLOW1_ON	= 65,
	DPU952_IRQ_FRAMEGEN0_OVERFLOW1_OFF	= 66,
	DPU952_IRQ_FRAMEGEN0_UNDERRUN1_ON	= 67,
	DPU952_IRQ_FRAMEGEN0_UNDERRUN1_OFF	= 68,
	DPU952_IRQ_FRAMEGEN0_THRESHOLD1_RISE	= 69,
	DPU952_IRQ_FRAMEGEN0_THRESHOLD1_FAIL	= 70,
	DPU952_IRQ_FRAMEGEN1_PRIMSYNC_ON	= 71,
	DPU952_IRQ_FRAMEGEN1_PRIMSYNC_OFF	= 72,
	DPU952_IRQ_FRAMEGEN1_OVERFLOW0_ON	= 73,
	DPU952_IRQ_FRAMEGEN1_OVERFLOW0_OFF	= 74,
	DPU952_IRQ_FRAMEGEN1_UNDERRUN0_ON	= 75,
	DPU952_IRQ_FRAMEGEN1_UNDERRUN0_OFF	= 76,
	DPU952_IRQ_FRAMEGEN1_THRESHOLD0_RISE	= 77,
	DPU952_IRQ_FRAMEGEN1_THRESHOLD0_FAIL	= 78,
	DPU952_IRQ_FRAMEGEN1_OVERFLOW1_ON	= 79,
	DPU952_IRQ_FRAMEGEN1_OVERFLOW1_OFF	= 80,
	DPU952_IRQ_FRAMEGEN1_UNDERRUN1_ON	= 81,
	DPU952_IRQ_FRAMEGEN1_UNDERRUN1_OFF	= 82,
	DPU952_IRQ_FRAMEGEN1_THRESHOLD1_RISE	= 83,
	DPU952_IRQ_FRAMEGEN1_THRESHOLD1_FAIL	= 84,
	DPU952_IRQ_CNT				= 85,
};

static const unsigned long dpu952_unused_irq[] = {
0x00000000, 0x00000000, 0xffe00000};

static enum dpu952_irq dpu952_comctrl_irq[] = {
	DPU952_IRQ_COMCTRL_SW0,
	DPU952_IRQ_COMCTRL_SW1,
	DPU952_IRQ_COMCTRL_SW2,
	DPU952_IRQ_COMCTRL_SW3,
};

static enum dpu952_irq dpu952_disp_irq0[] = {
	DPU952_IRQ_EXTDST0_SHDLOAD,
	DPU952_IRQ_EXTDST0_FRAMECOMPLETE,
	DPU952_IRQ_EXTDST0_SEQCOMPLETE,
	DPU952_IRQ_EXTDST4_SHDLOAD,
	DPU952_IRQ_EXTDST4_FRAMECOMPLETE,
	DPU952_IRQ_EXTDST4_SEQCOMPLETE,
	DPU952_IRQ_DOMAINBLEND0_SHDLOAD,
	DPU952_IRQ_DOMAINBLEND0_FRAMECOMPLETE,
	DPU952_IRQ_DISENGCFG_SHDLOAD0,
	DPU952_IRQ_DISENGCFG_FRAMECOMPLETE0,
	DPU952_IRQ_DISENGCFG_SEQCOMPLETE0,
	DPU952_IRQ_FRAMEGEN0_INT0,
	DPU952_IRQ_FRAMEGEN0_INT1,
	DPU952_IRQ_FRAMEGEN0_INT2,
	DPU952_IRQ_FRAMEGEN0_INT3,
	DPU952_IRQ_SIG0_SHDLOAD,
	DPU952_IRQ_SIG0_VALID,
	DPU952_IRQ_SIG0_ERROR,
	DPU952_IRQ_SIG0_CLUSTER_ERROR,
	DPU952_IRQ_SIG0_CLUSTER_MATCH,
	DPU952_IRQ_IDHASH0_SHDLOAD,
	DPU952_IRQ_IDHASH0_VALID,
	DPU952_IRQ_IDHASH0_WINDOWN_ERROR,
	DPU952_IRQ_LOCALDIMMING0_IRQ0,
	DPU952_IRQ_LOCALDIMMING0_IRQ1,
	DPU952_IRQ_LOCALDIMMING0_IRQ2,
	DPU952_IRQ_FRAMEGEN0_PRIMSYNC_ON,
	DPU952_IRQ_FRAMEGEN0_PRIMSYNC_OFF,
	DPU952_IRQ_FRAMEGEN0_OVERFLOW0_ON,
	DPU952_IRQ_FRAMEGEN0_OVERFLOW0_OFF,
	DPU952_IRQ_FRAMEGEN0_UNDERRUN0_ON,
	DPU952_IRQ_FRAMEGEN0_UNDERRUN0_OFF,
	DPU952_IRQ_FRAMEGEN0_THRESHOLD0_RISE,
	DPU952_IRQ_FRAMEGEN0_THRESHOLD0_FAIL,
	DPU952_IRQ_FRAMEGEN0_OVERFLOW1_ON,
	DPU952_IRQ_FRAMEGEN0_OVERFLOW1_OFF,
	DPU952_IRQ_FRAMEGEN0_UNDERRUN1_ON,
	DPU952_IRQ_FRAMEGEN0_UNDERRUN1_OFF,
	DPU952_IRQ_FRAMEGEN0_THRESHOLD1_RISE,
	DPU952_IRQ_FRAMEGEN0_THRESHOLD1_FAIL,
};

static enum dpu952_irq dpu952_disp_irq2[] = {
	DPU952_IRQ_EXTDST1_SHDLOAD,
	DPU952_IRQ_EXTDST1_FRAMECOMPLETE,
	DPU952_IRQ_EXTDST1_SEQCOMPLETE,
	DPU952_IRQ_EXTDST5_SHDLOAD,
	DPU952_IRQ_EXTDST5_FRAMECOMPLETE,
	DPU952_IRQ_EXTDST5_SEQCOMPLETE,
	DPU952_IRQ_DOMAINBLEND1_SHDLOAD,
	DPU952_IRQ_DOMAINBLEND1_FRAMECOMPLETE,
	DPU952_IRQ_DISENGCFG_SHDLOAD1,
	DPU952_IRQ_DISENGCFG_FRAMECOMPLETE1,
	DPU952_IRQ_DISENGCFG_SEQCOMPLETE1,
	DPU952_IRQ_FRAMEGEN1_INT0,
	DPU952_IRQ_FRAMEGEN1_INT1,
	DPU952_IRQ_FRAMEGEN1_INT2,
	DPU952_IRQ_FRAMEGEN1_INT3,
	DPU952_IRQ_SIG1_SHDLOAD,
	DPU952_IRQ_SIG1_VALID,
	DPU952_IRQ_SIG1_ERROR,
	DPU952_IRQ_SIG1_CLUSTER_ERROR,
	DPU952_IRQ_SIG1_CLUSTER_MATCH,
	DPU952_IRQ_IDHASH1_SHDLOAD,
	DPU952_IRQ_IDHASH1_VALID,
	DPU952_IRQ_IDHASH1_WINDOWN_ERROR,
	DPU952_IRQ_FRAMEGEN1_PRIMSYNC_ON,
	DPU952_IRQ_FRAMEGEN1_PRIMSYNC_OFF,
	DPU952_IRQ_FRAMEGEN1_OVERFLOW0_ON,
	DPU952_IRQ_FRAMEGEN1_OVERFLOW0_OFF,
	DPU952_IRQ_FRAMEGEN1_UNDERRUN0_ON,
	DPU952_IRQ_FRAMEGEN1_UNDERRUN0_OFF,
	DPU952_IRQ_FRAMEGEN1_THRESHOLD0_RISE,
	DPU952_IRQ_FRAMEGEN1_THRESHOLD0_FAIL,
	DPU952_IRQ_FRAMEGEN1_OVERFLOW1_ON,
	DPU952_IRQ_FRAMEGEN1_OVERFLOW1_OFF,
	DPU952_IRQ_FRAMEGEN1_UNDERRUN1_ON,
	DPU952_IRQ_FRAMEGEN1_UNDERRUN1_OFF,
	DPU952_IRQ_FRAMEGEN1_THRESHOLD1_RISE,
	DPU952_IRQ_FRAMEGEN1_THRESHOLD1_FAIL,
};

static void dpu952_comctrl_sw0_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU952_IRQ_COMCTRL_SW0);
}

static void dpu952_comctrl_sw1_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU952_IRQ_COMCTRL_SW1);
}

static void dpu952_comctrl_sw2_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU952_IRQ_COMCTRL_SW2);
}

static void dpu952_comctrl_sw3_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU952_IRQ_COMCTRL_SW3);
}

static void dpu952_dec_framecomplete0_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU952_IRQ_DISENGCFG_FRAMECOMPLETE0);
}

static void dpu952_dec_framecomplete1_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU952_IRQ_DISENGCFG_FRAMECOMPLETE1);
}

static void dpu952_dec_seqcomplete0_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU952_IRQ_DISENGCFG_SEQCOMPLETE0);
}

static void dpu952_dec_seqcomplete1_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU952_IRQ_DISENGCFG_SEQCOMPLETE1);
}

static void dpu952_dec_shdload0_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU952_IRQ_DISENGCFG_SHDLOAD0);
}

static void dpu952_dec_shdload1_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU952_IRQ_DISENGCFG_SHDLOAD1);
}

static void dpu952_ed0_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU952_IRQ_EXTDST0_SHDLOAD);
}

static void dpu952_ed1_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU952_IRQ_EXTDST1_SHDLOAD);
}

static void dpu952_db0_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU952_IRQ_DOMAINBLEND0_SHDLOAD);
}

static void dpu952_db1_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU952_IRQ_DOMAINBLEND1_SHDLOAD);
}

static int dpu952_set_qos(struct dpu95_soc *dpu)
{
	int ret;

	ret = regmap_update_bits(dpu->regmap, QOS_SETTING_EXT2,
					EXT_QOS_BE_EN_MASK,
					EXT_QOS_BE_EN(0x1));

	if (ret < 0) {
		dev_err(dpu->dev, "failed to set QoS: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(dpu->regmap, dpu->data->qos_setting,
				DISPLAY_PANIC_QOS_MASK | DISPLAY_ARQOS_MASK |
				CMDSEQ_PANIC_AWQOS_MASK | CMDSEQ_AWQOS_MASK,
				DISPLAY_PANIC_QOS(0x3) | DISPLAY_ARQOS(0x3) |
				CMDSEQ_PANIC_AWQOS(0x1) | CMDSEQ_AWQOS(0x1));
	if (ret < 0) {
		dev_err(dpu->dev, "failed to set QoS: %d\n", ret);
		return ret;
	}

	return 0;
}

static void (* const dpu952_comctrl_irq_handler[DPU952_IRQ_CNT])(struct irq_desc *desc) = {
	[DPU952_IRQ_COMCTRL_SW0]              = dpu952_comctrl_sw0_irq_handler,
	[DPU952_IRQ_COMCTRL_SW1]              = dpu952_comctrl_sw1_irq_handler,
	[DPU952_IRQ_COMCTRL_SW2]              = dpu952_comctrl_sw2_irq_handler,
	[DPU952_IRQ_COMCTRL_SW3]              = dpu952_comctrl_sw3_irq_handler,
};

static void (* const dpu952_disp_irq0_handler[DPU952_IRQ_CNT])(struct irq_desc *desc) = {
	[DPU952_IRQ_DOMAINBLEND0_SHDLOAD]     = dpu952_db0_shdload_irq_handler,
	[DPU952_IRQ_EXTDST0_SHDLOAD]          = dpu952_ed0_shdload_irq_handler,
	[DPU952_IRQ_DISENGCFG_SHDLOAD0]       = dpu952_dec_shdload0_irq_handler,
	[DPU952_IRQ_DISENGCFG_FRAMECOMPLETE0] = dpu952_dec_framecomplete0_irq_handler,
	[DPU952_IRQ_DISENGCFG_SEQCOMPLETE0]   = dpu952_dec_seqcomplete0_irq_handler,
};

static void (* const dpu952_disp_irq2_handler[DPU952_IRQ_CNT])(struct irq_desc *desc) = {
	[DPU952_IRQ_DOMAINBLEND1_SHDLOAD]     = dpu952_db1_shdload_irq_handler,
	[DPU952_IRQ_EXTDST1_SHDLOAD]          = dpu952_ed1_shdload_irq_handler,
	[DPU952_IRQ_DISENGCFG_SHDLOAD1]       = dpu952_dec_shdload1_irq_handler,
	[DPU952_IRQ_DISENGCFG_FRAMECOMPLETE1] = dpu952_dec_framecomplete1_irq_handler,
	[DPU952_IRQ_DISENGCFG_SEQCOMPLETE1]   = dpu952_dec_seqcomplete1_irq_handler,
};

static const struct dpu95_data dpu952_data = {
	.units = dpu952_all_units,
	.units_cnt = ARRAY_SIZE(dpu952_all_units),

	.link_id_map = dpu952_link_id_map,
	.link_id_fy = dpu952_link_id_fy,
	.link_id_fy_src = dpu952_link_id_fe,
	.link_id_fe = dpu952_link_id_fe,

	.irq_cnt = DPU952_IRQ_CNT,
	.unused_irq = dpu952_unused_irq,

	.comctrl_irq = (int *)dpu952_comctrl_irq,
	.comctrl_irq_cnt = ARRAY_SIZE(dpu952_comctrl_irq),
	.comctrl_irq_handler = dpu952_comctrl_irq_handler,

	.disp_irq0 = (int *)dpu952_disp_irq0,
	.disp_irq0_cnt = ARRAY_SIZE(dpu952_disp_irq0),
	.disp_irq0_handler = dpu952_disp_irq0_handler,

	.disp_irq2 = (int *)dpu952_disp_irq2,
	.disp_irq2_cnt = ARRAY_SIZE(dpu952_disp_irq2),
	.disp_irq2_handler = dpu952_disp_irq2_handler,

	.dec_frame_complete_irq = {
		DPU952_IRQ_DISENGCFG_FRAMECOMPLETE0,
		DPU952_IRQ_DISENGCFG_FRAMECOMPLETE1,
	},
	.dec_seq_complete_irq = {
		DPU952_IRQ_DISENGCFG_SEQCOMPLETE0,
		DPU952_IRQ_DISENGCFG_SEQCOMPLETE1,
	},
	.dec_shdld_irq = {
		DPU952_IRQ_DISENGCFG_SHDLOAD0,
		DPU952_IRQ_DISENGCFG_SHDLOAD1,
	},
	.db_shdld_irq = {
		DPU952_IRQ_DOMAINBLEND0_SHDLOAD,
		DPU952_IRQ_DOMAINBLEND1_SHDLOAD
	},
	.ed_cont_shdld_irq = {
		DPU952_IRQ_EXTDST0_SHDLOAD,
		DPU952_IRQ_EXTDST1_SHDLOAD,
	},

	.irq0_addr = 0x3c1000,
	.irq2_addr = 0x3e1000,
	.clock_ctrl = 0x04,
	.qos_setting = 0x14,
	.set_qos = dpu952_set_qos,
	.plane_association = 0x18,
	.reg_polarityctrl = 0x10,

	.cmdseq_interrupt_clear0 = 0x11014,
};

#endif /* __DRM_DPU952_DATA_H__ */
