/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Coda series multi-standard codec IP - v4l2 interface helper definitions
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#ifndef __CODA_HELPER_H__
#define __CODA_HELPER_H__

#include "coda-vpu.h"

#define VPU_FMT_TYPES	2
#define MAX_VPU_FMTS	6

int coda_vpu_init_m2m_dev(struct vpu_device *vpu);
void coda_vpu_release_m2m_dev(struct vpu_device *vpu);
int coda_vpu_wait_interrupt(struct vpu_instance *inst, unsigned int timeout);
void coda_vpu_update_pix_fmt(struct v4l2_pix_format_mplane *pix_mp,
			     int pix_fmt_type,
			     unsigned int width,
			     unsigned int height,
			     const struct v4l2_frmsize_stepwise *frmsize);
void coda_vpu_set_instance_state(struct vpu_instance *inst, u32 state);
void coda_vpu_return_buffers(struct vpu_instance *inst, unsigned int type,
			     enum vb2_buffer_state state);
struct vb2_v4l2_buffer *coda_vpu_get_valid_src_buf(struct vpu_instance *inst);
struct vb2_v4l2_buffer *coda_vpu_get_valid_dst_buf(struct vpu_instance *inst);
struct vb2_v4l2_buffer *coda_vpu_get_dst_buf_by_addr(struct vpu_instance *inst,
						     dma_addr_t addr);
dma_addr_t coda_vpu_get_dma_addr(struct vb2_v4l2_buffer *buf, unsigned int plane_no);

#endif
