/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 NXP
 */

#ifndef __DRM_DPU95_DATA_H__
#define __DRM_DPU95_DATA_H__

#include "dpu95.h"
#include <linux/regmap.h>

/* Constant Frame */
static const unsigned int dpu95_cf_ids[] = {0, 1, 4, 5};
static const enum dpu95_unit_type dpu95_cf_types[] = {
DPU95_DISP, DPU95_DISP, DPU95_DISP, DPU95_DISP};
static const unsigned long dpu95_cf_ofss[] = {
0xf0000, 0x130000, 0x100000, 0x140000};
static const unsigned long dpu95_cf_aux_ofss[] = {
0xf1000, 0x131000, 0x101000, 0x141000};

static const struct dpu95_units dpu95_dpu_cfs = {
	.ids		= dpu95_cf_ids,
	.types		= dpu95_cf_types,
	.ofss		= dpu95_cf_ofss,
	.aux_ofss	= dpu95_cf_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu95_cf_ids),
	.name		= DPU95_CONSTFRAME,
	.init		= dpu95_cf_init,
	.hw_init	= dpu95_cf_hw_init,
};

/* Domain Blend */
static const unsigned int dpu95_db_ids[] = {0, 1};
static const enum dpu95_unit_type dpu95_db_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long dpu95_db_ofss[] = {0x2a0000, 0x320000};

static const struct dpu95_units dpu95_dpu_dbs = {
	.ids		= dpu95_db_ids,
	.types		= dpu95_db_types,
	.ofss		= dpu95_db_ofss,
	.cnt		= ARRAY_SIZE(dpu95_db_ids),
	.name		= DPU95_DOMAINBLEND,
	.init		= dpu95_db_init,
	.hw_init	= dpu95_db_hw_init,
};

/* Dither */
static const unsigned int dpu95_dt_ids[] = {0, 1};
static const enum dpu95_unit_type dpu95_dt_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long dpu95_dt_ofss[] = {0x310000, 0x370000};
static const unsigned long dpu95_dt_aux_ofss[] = {0x311000, 0x371020};

static const struct dpu95_units dpu95_dpu_dts = {
	.ids		= dpu95_dt_ids,
	.types		= dpu95_dt_types,
	.ofss		= dpu95_dt_ofss,
	.aux_ofss	= dpu95_dt_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu95_dt_ids),
	.name		= DPU95_DITHER,
	.init		= dpu95_dt_init,
	.hw_init	= dpu95_dt_hw_init,
};

/* External Destination */
static const unsigned int dpu95_ed_ids[] = {0, 1, 4, 5};
static const enum dpu95_unit_type dpu95_ed_types[] = {
DPU95_DISP, DPU95_DISP, DPU95_DISP, DPU95_DISP};
static const unsigned long dpu95_ed_ofss[] = {
0x110000, 0x150000, 0x120000, 0x160000};
static const unsigned long dpu95_ed_aux_ofss[] = {
0x111000, 0x151000, 0x121000, 0x161000};

static const struct dpu95_units dpu95_dpu_eds = {
	.ids		= dpu95_ed_ids,
	.types		= dpu95_ed_types,
	.ofss		= dpu95_ed_ofss,
	.aux_ofss	= dpu95_ed_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu95_ed_ids),
	.name		= DPU95_EXTDST,
	.init		= dpu95_ed_init,
	.hw_init	= dpu95_ed_hw_init,
};

/* Fetch ECO */
static const unsigned int dpu95_fe_ids[] = {0, 1, 2, 9};
static const enum dpu95_unit_type dpu95_fe_types[] = {
DPU95_DISP, DPU95_DISP, DPU95_DISP, DPU95_BLIT};
static const unsigned long dpu95_fe_ofss[] = {
0x210000, 0x230000, 0x250000, 0xa0000};
static const unsigned long dpu95_fe_aux_ofss[] = {
0x211000, 0x231000, 0x251000, 0xa1000};

static const struct dpu95_units dpu95_dpu_fes = {
	.ids		= dpu95_fe_ids,
	.types		= dpu95_fe_types,
	.ofss		= dpu95_fe_ofss,
	.aux_ofss	= dpu95_fe_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu95_fe_ids),
	.name		= DPU95_FETCHECO,
	.init		= dpu95_fe_init,
	.hw_init	= dpu95_fe_hw_init,
};

/* Frame Generator */
static const unsigned int dpu95_fg_ids[] = {0, 1};
static const enum dpu95_unit_type dpu95_fg_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long dpu95_fg_ofss[] = {0x2b0000, 0x330000};

static const struct dpu95_units dpu95_dpu_fgs = {
	.ids		= dpu95_fg_ids,
	.types		= dpu95_fg_types,
	.ofss		= dpu95_fg_ofss,
	.cnt		= ARRAY_SIZE(dpu95_fg_ids),
	.name		= DPU95_FRAMEGEN,
	.init		= dpu95_fg_init,
	.hw_init	= dpu95_fg_hw_init,
};

/* Fetch Layer */
static const unsigned int dpu95_fl_ids[] = {0, 1};
static const enum dpu95_unit_type dpu95_fl_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long dpu95_fl_ofss[] = {0x1d0000, 0x1e0000};
static const unsigned long dpu95_fl_aux_ofss[] = {0x1d1000, 0x1e1000};

static const struct dpu95_units dpu95_dpu_fls = {
	.ids		= dpu95_fl_ids,
	.types		= dpu95_fl_types,
	.ofss		= dpu95_fl_ofss,
	.aux_ofss	= dpu95_fl_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu95_fl_ids),
	.name		= DPU95_FETCHLAYER,
	.init		= dpu95_fl_init,
	.hw_init	= dpu95_fl_hw_init,
};

/* Fetch YUV */
static const unsigned int dpu95_fy_ids[] = {0, 1, 2, 3};
static const enum dpu95_unit_type dpu95_fy_types[] = {
DPU95_DISP, DPU95_DISP, DPU95_DISP, DPU95_DISP};
static const unsigned long dpu95_fy_ofss[] = {
0x200000, 0x220000, 0x240000, 0x1f0000};
static const unsigned long dpu95_fy_aux_ofss[] = {
0x201000, 0x221000, 0x241000, 0x1f1000};

static const struct dpu95_units dpu95_dpu_fys = {
	.ids		= dpu95_fy_ids,
	.types		= dpu95_fy_types,
	.ofss		= dpu95_fy_ofss,
	.aux_ofss	= dpu95_fy_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu95_fy_ids),
	.name		= DPU95_FETCHYUV,
	.init		= dpu95_fy_init,
	.hw_init	= dpu95_fy_hw_init,
};

/* Horizontal Scaler */
static const unsigned int dpu95_hs_ids[] = {4, 9};
static const enum dpu95_unit_type dpu95_hs_types[] = {DPU95_DISP, DPU95_BLIT};
static const unsigned long dpu95_hs_ofss[] = {0x270000, 0xb0000};
static const unsigned long dpu95_hs_aux_ofss[] = {0x271000, 0xb1000};

static const struct dpu95_units dpu95_dpu_hss = {
	.ids		= dpu95_hs_ids,
	.types		= dpu95_hs_types,
	.ofss		= dpu95_hs_ofss,
	.aux_ofss	= dpu95_hs_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu95_hs_ids),
	.name		= DPU95_HSCALER,
	.init		= dpu95_hs_init,
	.hw_init	= dpu95_hs_hw_init,
};

/* Layer Blend */
static const unsigned int dpu95_lb_ids[] = {1, 2, 3, 4, 5, 6};
static const enum dpu95_unit_type dpu95_lb_types[] = {
DPU95_DISP, DPU95_DISP, DPU95_DISP, DPU95_DISP, DPU95_DISP, DPU95_DISP};
static const unsigned long dpu95_lb_ofss[] = {
0x170000, 0x180000, 0x190000, 0x1a0000, 0x1b0000, 0x1c0000};
static const unsigned long dpu95_lb_aux_ofss[] = {
0x171000, 0x181000, 0x191000, 0x1a1000, 0x1b1000, 0x1c1000};

static const struct dpu95_units dpu95_dpu_lbs = {
	.ids		= dpu95_lb_ids,
	.types		= dpu95_lb_types,
	.ofss		= dpu95_lb_ofss,
	.aux_ofss	= dpu95_lb_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu95_lb_ids),
	.name		= DPU95_LAYERBLEND,
	.init		= dpu95_lb_init,
	.hw_init	= dpu95_lb_hw_init,
};

/* Vertical Scaler */
static const unsigned int dpu95_vs_ids[] = {4, 9};
static const enum dpu95_unit_type dpu95_vs_types[] = {DPU95_DISP, DPU95_BLIT};
static const unsigned long dpu95_vs_ofss[] = {0x280000, 0xc0000};
static const unsigned long dpu95_vs_aux_ofss[] = {0x281000, 0xc1000};

static const struct dpu95_units dpu95_dpu_vss = {
	.ids		= dpu95_vs_ids,
	.types		= dpu95_vs_types,
	.ofss		= dpu95_vs_ofss,
	.aux_ofss	= dpu95_vs_aux_ofss,
	.cnt		= ARRAY_SIZE(dpu95_vs_ids),
	.name		= DPU95_VSCALER,
	.init		= dpu95_vs_init,
	.hw_init	= dpu95_vs_hw_init,
};

static const struct dpu95_units *dpu95_all_units[] = {
	&dpu95_dpu_cfs,
	&dpu95_dpu_dbs,
	&dpu95_dpu_dts,
	&dpu95_dpu_eds,
	&dpu95_dpu_fes,
	&dpu95_dpu_fgs,
	&dpu95_dpu_fls,
	&dpu95_dpu_fys,
	&dpu95_dpu_hss,
	&dpu95_dpu_lbs,
	&dpu95_dpu_vss,
};

static const enum dpu95_link_id dpu95_link_id_map[] = {
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_STORE9,
	DPU95_LINK_ID_EXTDST0,
	DPU95_LINK_ID_EXTDST4,
	DPU95_LINK_ID_EXTDST1,
	DPU95_LINK_ID_EXTDST5,
	DPU95_LINK_ID_NONE,
	DPU95_LINK_ID_FETCHECO9,
	DPU95_LINK_ID_HSCALER9,
	DPU95_LINK_ID_NONE,
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
	DPU95_LINK_ID_LAYERBLEND6,
	DPU95_LINK_ID_FETCHLAYER0,
	DPU95_LINK_ID_FETCHLAYER1,
	DPU95_LINK_ID_FETCHYUV3,
	DPU95_LINK_ID_FETCHYUV0,
	DPU95_LINK_ID_FETCHECO0,
	DPU95_LINK_ID_FETCHYUV1,
	DPU95_LINK_ID_FETCHECO1,
	DPU95_LINK_ID_FETCHYUV2,
	DPU95_LINK_ID_FETCHECO2,
	DPU95_LINK_ID_MATRIX4,
	DPU95_LINK_ID_HSCALER4,
	DPU95_LINK_ID_VSCALER4,
};

static const enum dpu95_link_id dpu95_link_id_fy[] = {
	DPU95_LINK_ID_FETCHYUV0, DPU95_LINK_ID_FETCHYUV1,
	DPU95_LINK_ID_FETCHYUV2, DPU95_LINK_ID_FETCHYUV3,
};

static const enum dpu95_link_id dpu95_link_id_fe[] = {
	DPU95_LINK_ID_FETCHECO0, DPU95_LINK_ID_FETCHECO1,
	DPU95_LINK_ID_FETCHECO2, DPU95_LINK_ID_FETCHECO9,
};

enum dpu95_irq {
	DPU95_IRQ_STORE9_SHDLOAD		= 0,
	DPU95_IRQ_STORE9_FRAMECOMPLETE		= 1,
	DPU95_IRQ_STORE9_SEQCOMPLETE		= 2,
	DPU95_IRQ_EXTDST0_SHDLOAD		= 3,
	DPU95_IRQ_EXTDST0_FRAMECOMPLETE		= 4,
	DPU95_IRQ_EXTDST0_SEQCOMPLETE		= 5,
	DPU95_IRQ_EXTDST4_SHDLOAD		= 6,
	DPU95_IRQ_EXTDST4_FRAMECOMPLETE		= 7,
	DPU95_IRQ_EXTDST4_SEQCOMPLETE		= 8,
	DPU95_IRQ_EXTDST1_SHDLOAD		= 9,
	DPU95_IRQ_EXTDST1_FRAMECOMPLETE		= 10,
	DPU95_IRQ_EXTDST1_SEQCOMPLETE		= 11,
	DPU95_IRQ_EXTDST5_SHDLOAD		= 12,
	DPU95_IRQ_EXTDST5_FRAMECOMPLETE		= 13,
	DPU95_IRQ_EXTDST5_SEQCOMPLETE		= 14,
	DPU95_IRQ_DOMAINBLEND0_SHDLOAD		= 15,
	DPU95_IRQ_DOMAINBLEND0_FRAMECOMPLETE	= 16,
	DPU95_IRQ_DOMAINBLEND0_SEQCOMPLETE	= 17,
	DPU95_IRQ_DISENGCFG_SHDLOAD0		= 18,
	DPU95_IRQ_DISENGCFG_FRAMECOMPLETE0	= 19,
	DPU95_IRQ_DISENGCFG_SEQCOMPLETE0	= 20,
	DPU95_IRQ_FRAMEGEN0_INT0		= 21,
	DPU95_IRQ_FRAMEGEN0_INT1		= 22,
	DPU95_IRQ_FRAMEGEN0_INT2		= 23,
	DPU95_IRQ_FRAMEGEN0_INT3		= 24,
	DPU95_IRQ_SIG0_SHDLOAD			= 25,
	DPU95_IRQ_SIG0_VALID			= 26,
	DPU95_IRQ_SIG0_ERROR			= 27,
	DPU95_IRQ_SIG0_CLUSTER_ERROR		= 28,
	DPU95_IRQ_SIG0_CLUSTER_MATCH		= 29,
	DPU95_IRQ_SIG2_SHDLOAD			= 30,
	DPU95_IRQ_SIG2_VALID			= 31,
	DPU95_IRQ_SIG2_ERROR			= 32,
	DPU95_IRQ_SIG2_CLUSTER_ERROR		= 33,
	DPU95_IRQ_SIG2_CLUSTER_MATCH		= 34,
	DPU95_IRQ_IDHASH0_SHDLOAD		= 35,
	DPU95_IRQ_IDHASH0_VALID			= 36,
	DPU95_IRQ_IDHASH0_WINDOWN_ERROR		= 37,
	DPU95_IRQ_DOMAINBLEND1_SHDLOAD		= 38,
	DPU95_IRQ_DOMAINBLEND1_FRAMECOMPLETE	= 39,
	DPU95_IRQ_DOMAINBLEND1_SEQCOMPLETE	= 40,
	DPU95_IRQ_DISENGCFG_SHDLOAD1		= 41,
	DPU95_IRQ_DISENGCFG_FRAMECOMPLETE1	= 42,
	DPU95_IRQ_DISENGCFG_SEQCOMPLETE1	= 43,
	DPU95_IRQ_FRAMEGEN1_INT0		= 44,
	DPU95_IRQ_FRAMEGEN1_INT1		= 45,
	DPU95_IRQ_FRAMEGEN1_INT2		= 46,
	DPU95_IRQ_FRAMEGEN1_INT3		= 47,
	DPU95_IRQ_SIG1_SHDLOAD			= 48,
	DPU95_IRQ_SIG1_VALID			= 49,
	DPU95_IRQ_SIG1_ERROR			= 50,
	DPU95_IRQ_SIG1_CLUSTER_ERROR		= 51,
	DPU95_IRQ_SIG1_CLUSTER_MATCH		= 52,
	DPU95_IRQ_CMDSEQ_ERROR			= 53,
	DPU95_IRQ_COMCTRL_SW0			= 54,
	DPU95_IRQ_COMCTRL_SW1			= 55,
	DPU95_IRQ_COMCTRL_SW2			= 56,
	DPU95_IRQ_COMCTRL_SW3			= 57,
	DPU95_IRQ_FRAMEGEN0_PRIMSYNC_ON		= 58,
	DPU95_IRQ_FRAMEGEN0_PRIMSYNC_OFF	= 59,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW0_ON	= 60,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW0_OFF	= 61,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN0_ON	= 62,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN0_OFF	= 63,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD0_RISE	= 64,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD0_FAIL	= 65,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW1_ON	= 66,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW1_OFF	= 67,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN1_ON	= 68,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN1_OFF	= 69,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD1_RISE	= 70,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD1_FAIL	= 71,
	DPU95_IRQ_FRAMEGEN1_PRIMSYNC_ON		= 72,
	DPU95_IRQ_FRAMEGEN1_PRIMSYNC_OFF	= 73,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW0_ON	= 74,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW0_OFF	= 75,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN0_ON	= 76,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN0_OFF	= 77,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD0_RISE	= 78,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD0_FAIL	= 79,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW1_ON	= 80,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW1_OFF	= 81,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN1_ON	= 82,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN1_OFF	= 83,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD1_RISE	= 84,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD1_FAIL	= 85,
	DPU95_IRQ_CNT				= 86,
};

static const unsigned long dpu95_unused_irq[] = {
0x00000000, 0x00000000, 0xffc00000};

static enum dpu95_irq dpu95_comctrl_irq[] = {
	DPU95_IRQ_COMCTRL_SW0,
	DPU95_IRQ_COMCTRL_SW1,
	DPU95_IRQ_COMCTRL_SW2,
	DPU95_IRQ_COMCTRL_SW3,
};

static enum dpu95_irq dpu95_disp_irq0[] = {
	DPU95_IRQ_EXTDST0_SHDLOAD,
	DPU95_IRQ_EXTDST0_FRAMECOMPLETE,
	DPU95_IRQ_EXTDST0_SEQCOMPLETE,
	DPU95_IRQ_EXTDST4_SHDLOAD,
	DPU95_IRQ_EXTDST4_FRAMECOMPLETE,
	DPU95_IRQ_EXTDST4_SEQCOMPLETE,
	DPU95_IRQ_DOMAINBLEND0_SHDLOAD,
	DPU95_IRQ_DOMAINBLEND0_FRAMECOMPLETE,
	DPU95_IRQ_DOMAINBLEND0_SEQCOMPLETE,
	DPU95_IRQ_DISENGCFG_SHDLOAD0,
	DPU95_IRQ_DISENGCFG_FRAMECOMPLETE0,
	DPU95_IRQ_DISENGCFG_SEQCOMPLETE0,
	DPU95_IRQ_FRAMEGEN0_INT0,
	DPU95_IRQ_FRAMEGEN0_INT1,
	DPU95_IRQ_FRAMEGEN0_INT2,
	DPU95_IRQ_FRAMEGEN0_INT3,
	DPU95_IRQ_SIG0_SHDLOAD,
	DPU95_IRQ_SIG0_VALID,
	DPU95_IRQ_SIG0_ERROR,
	DPU95_IRQ_SIG0_CLUSTER_ERROR,
	DPU95_IRQ_SIG0_CLUSTER_MATCH,
	DPU95_IRQ_SIG2_SHDLOAD,
	DPU95_IRQ_SIG2_VALID,
	DPU95_IRQ_SIG2_ERROR,
	DPU95_IRQ_SIG2_CLUSTER_ERROR,
	DPU95_IRQ_SIG2_CLUSTER_MATCH,
	DPU95_IRQ_IDHASH0_SHDLOAD,
	DPU95_IRQ_IDHASH0_VALID,
	DPU95_IRQ_IDHASH0_WINDOWN_ERROR,
	DPU95_IRQ_FRAMEGEN0_PRIMSYNC_ON,
	DPU95_IRQ_FRAMEGEN0_PRIMSYNC_OFF,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW0_ON,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW0_OFF,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN0_ON,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN0_OFF,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD0_RISE,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD0_FAIL,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW1_ON,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW1_OFF,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN1_ON,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN1_OFF,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD1_RISE,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD1_FAIL,
};

static enum dpu95_irq dpu95_disp_irq2[] = {
	DPU95_IRQ_EXTDST1_SHDLOAD,
	DPU95_IRQ_EXTDST1_FRAMECOMPLETE,
	DPU95_IRQ_EXTDST1_SEQCOMPLETE,
	DPU95_IRQ_EXTDST5_SHDLOAD,
	DPU95_IRQ_EXTDST5_FRAMECOMPLETE,
	DPU95_IRQ_EXTDST5_SEQCOMPLETE,
	DPU95_IRQ_DOMAINBLEND1_SHDLOAD,
	DPU95_IRQ_DOMAINBLEND1_FRAMECOMPLETE,
	DPU95_IRQ_DOMAINBLEND1_SEQCOMPLETE,
	DPU95_IRQ_DISENGCFG_SHDLOAD1,
	DPU95_IRQ_DISENGCFG_FRAMECOMPLETE1,
	DPU95_IRQ_DISENGCFG_SEQCOMPLETE1,
	DPU95_IRQ_FRAMEGEN1_INT0,
	DPU95_IRQ_FRAMEGEN1_INT1,
	DPU95_IRQ_FRAMEGEN1_INT2,
	DPU95_IRQ_FRAMEGEN1_INT3,
	DPU95_IRQ_SIG1_SHDLOAD,
	DPU95_IRQ_SIG1_VALID,
	DPU95_IRQ_SIG1_ERROR,
	DPU95_IRQ_SIG1_CLUSTER_ERROR,
	DPU95_IRQ_SIG1_CLUSTER_MATCH,
	DPU95_IRQ_FRAMEGEN1_PRIMSYNC_ON,
	DPU95_IRQ_FRAMEGEN1_PRIMSYNC_OFF,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW0_ON,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW0_OFF,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN0_ON,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN0_OFF,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD0_RISE,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD0_FAIL,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW1_ON,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW1_OFF,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN1_ON,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN1_OFF,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD1_RISE,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD1_FAIL,
};

static void dpu95_comctrl_sw0_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU95_IRQ_COMCTRL_SW0);
}

static void dpu95_comctrl_sw1_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU95_IRQ_COMCTRL_SW1);
}

static void dpu95_comctrl_sw2_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU95_IRQ_COMCTRL_SW2);
}

static void dpu95_comctrl_sw3_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU95_IRQ_COMCTRL_SW3);
}

static void dpu95_dec_framecomplete0_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU95_IRQ_DISENGCFG_FRAMECOMPLETE0);
}

static void dpu95_dec_framecomplete1_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU95_IRQ_DISENGCFG_FRAMECOMPLETE1);
}

static void dpu95_dec_seqcomplete0_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU95_IRQ_DISENGCFG_SEQCOMPLETE0);
}

static void dpu95_dec_seqcomplete1_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU95_IRQ_DISENGCFG_SEQCOMPLETE1);
}

static void dpu95_dec_shdload0_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU95_IRQ_DISENGCFG_SHDLOAD0);
}

static void dpu95_dec_shdload1_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU95_IRQ_DISENGCFG_SHDLOAD1);
}

static void dpu95_ed0_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU95_IRQ_EXTDST0_SHDLOAD);
}

static void dpu95_ed1_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU95_IRQ_EXTDST1_SHDLOAD);
}

static void dpu95_db0_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU95_IRQ_DOMAINBLEND0_SHDLOAD);
}

static void dpu95_db1_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU95_IRQ_DOMAINBLEND1_SHDLOAD);
}

static int dpu95_set_qos(struct dpu95_soc *dpu)
{
	int ret;

	ret = regmap_update_bits(dpu->regmap, dpu->data->qos_setting,
				DISPLAY_PANIC_QOS_MASK | DISPLAY_ARQOS_MASK,
				DISPLAY_PANIC_QOS(0x3) | DISPLAY_ARQOS(0x3));
	if (ret < 0) {
		dev_err(dpu->dev, "failed to set QoS: %d\n", ret);
		return ret;
	}

	return 0;
}

static void (* const dpu95_comctrl_irq_handler[DPU95_IRQ_CNT])(struct irq_desc *desc) = {
	[DPU95_IRQ_COMCTRL_SW0]              = dpu95_comctrl_sw0_irq_handler,
	[DPU95_IRQ_COMCTRL_SW1]              = dpu95_comctrl_sw1_irq_handler,
	[DPU95_IRQ_COMCTRL_SW2]              = dpu95_comctrl_sw2_irq_handler,
	[DPU95_IRQ_COMCTRL_SW3]              = dpu95_comctrl_sw3_irq_handler,
};

static void (* const dpu95_disp_irq0_handler[DPU95_IRQ_CNT])(struct irq_desc *desc) = {
	[DPU95_IRQ_DOMAINBLEND0_SHDLOAD]     = dpu95_db0_shdload_irq_handler,
	[DPU95_IRQ_EXTDST0_SHDLOAD]          = dpu95_ed0_shdload_irq_handler,
	[DPU95_IRQ_DISENGCFG_SHDLOAD0]       = dpu95_dec_shdload0_irq_handler,
	[DPU95_IRQ_DISENGCFG_FRAMECOMPLETE0] = dpu95_dec_framecomplete0_irq_handler,
	[DPU95_IRQ_DISENGCFG_SEQCOMPLETE0]   = dpu95_dec_seqcomplete0_irq_handler,
};

static void (* const dpu95_disp_irq2_handler[DPU95_IRQ_CNT])(struct irq_desc *desc) = {
	[DPU95_IRQ_DOMAINBLEND1_SHDLOAD]     = dpu95_db1_shdload_irq_handler,
	[DPU95_IRQ_EXTDST1_SHDLOAD]          = dpu95_ed1_shdload_irq_handler,
	[DPU95_IRQ_DISENGCFG_SHDLOAD1]       = dpu95_dec_shdload1_irq_handler,
	[DPU95_IRQ_DISENGCFG_FRAMECOMPLETE1] = dpu95_dec_framecomplete1_irq_handler,
	[DPU95_IRQ_DISENGCFG_SEQCOMPLETE1]   = dpu95_dec_seqcomplete1_irq_handler,
};

static const struct dpu95_data dpu95_data = {
	.units = dpu95_all_units,
	.units_cnt = ARRAY_SIZE(dpu95_all_units),

	.link_id_map = dpu95_link_id_map,
	.link_id_fy = dpu95_link_id_fy,
	.link_id_fy_src = dpu95_link_id_fe,
	.link_id_fe = dpu95_link_id_fe,

	.irq_cnt = DPU95_IRQ_CNT,
	.unused_irq = dpu95_unused_irq,

	.comctrl_irq = (int *)dpu95_comctrl_irq,
	.comctrl_irq_cnt = ARRAY_SIZE(dpu95_comctrl_irq),
	.comctrl_irq_handler = dpu95_comctrl_irq_handler,

	.disp_irq0 = (int *)dpu95_disp_irq0,
	.disp_irq0_cnt = ARRAY_SIZE(dpu95_disp_irq0),
	.disp_irq0_handler = dpu95_disp_irq0_handler,

	.disp_irq2 = (int *)dpu95_disp_irq2,
	.disp_irq2_cnt = ARRAY_SIZE(dpu95_disp_irq2),
	.disp_irq2_handler = dpu95_disp_irq2_handler,

	.dec_frame_complete_irq = {
		DPU95_IRQ_DISENGCFG_FRAMECOMPLETE0,
		DPU95_IRQ_DISENGCFG_FRAMECOMPLETE1,
	},
	.dec_seq_complete_irq = {
		DPU95_IRQ_DISENGCFG_SEQCOMPLETE0,
		DPU95_IRQ_DISENGCFG_SEQCOMPLETE1,
	},
	.dec_shdld_irq = {
		DPU95_IRQ_DISENGCFG_SHDLOAD0,
		DPU95_IRQ_DISENGCFG_SHDLOAD1,
	},
	.db_shdld_irq = {
		DPU95_IRQ_DOMAINBLEND0_SHDLOAD,
		DPU95_IRQ_DOMAINBLEND1_SHDLOAD
	},
	.ed_cont_shdld_irq = {
		DPU95_IRQ_EXTDST0_SHDLOAD,
		DPU95_IRQ_EXTDST1_SHDLOAD,
	},

	.irq0_addr = 0x381000,
	.irq2_addr = 0x3a1000,
	.clock_ctrl = 0x00,
	.qos_setting = 0x1c,
	.set_qos = dpu95_set_qos,
	.plane_association = 0x20,
	.reg_polarityctrl = 0x08,
	.vsbp_quirk = true,

	.cmdseq_interrupt_clear0 = 0x11020,
};

#endif /* __DRM_DPU95_DATA_H__ */
