/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 NXP
 */

#ifndef __DRM_DPU937_DATA_H__
#define __DRM_DPU937_DATA_H__

#include "dpu952-data.h"

static const struct dpu95_units *dpu937_all_units[] = {
	&dpu952_dpu_cfs,
	&dpu952_dpu_cms,
	&dpu952_dpu_dbs,
	&dpu952_dpu_dts,
	&dpu952_dpu_eds,
	&dpu952_dpu_fes,
	&dpu952_dpu_fgs,
	&dpu952_dpu_fls,
	&dpu952_dpu_fys,
	&dpu952_dpu_hss,
	&dpu952_dpu_lbs,
	&dpu952_dpu_vss,
};

#define DPU937_IRQ_GAP_START	DPU952_IRQ_LOCALDIMMING0_IRQ0
#define DPU937_IRQ_GAP_END	DPU952_IRQ_LOCALDIMMING0_IRQ2
#define DPU937_IRQ_GAP_SIZE	(DPU937_IRQ_GAP_END - DPU937_IRQ_GAP_START + 1)

static const unsigned long dpu937_unused_irq[] = {
0x00000000, 0x00000007, 0xffe00000};

static int dpu937_get_platform_irq_num(int irq)
{
	if (irq > DPU937_IRQ_GAP_START)
		return irq - DPU937_IRQ_GAP_SIZE;

	return irq;
}

static enum dpu952_irq dpu937_disp_irq0[] = {
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

static const struct dpu95_data dpu937_data = {
	.units = dpu937_all_units,
	.units_cnt = ARRAY_SIZE(dpu937_all_units),

	.link_id_map = dpu952_link_id_map,
	.link_id_fy = dpu952_link_id_fy,
	.link_id_fy_src = dpu952_link_id_fe,
	.link_id_fe = dpu952_link_id_fe,

	.irq_cnt = DPU952_IRQ_CNT,
	.unused_irq = dpu937_unused_irq,
	.get_platform_irq_num = dpu937_get_platform_irq_num,

	.comctrl_irq = (int *)dpu952_comctrl_irq,
	.comctrl_irq_cnt = ARRAY_SIZE(dpu952_comctrl_irq),
	.comctrl_irq_handler = dpu952_comctrl_irq_handler,

	.disp_irq0 = (int *)dpu937_disp_irq0,
	.disp_irq0_cnt = ARRAY_SIZE(dpu937_disp_irq0),
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

#endif /* __DRM_DPU937_DATA_H__ */
