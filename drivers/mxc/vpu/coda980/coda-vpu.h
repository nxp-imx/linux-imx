/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Coda series multi-standard codec IP - basic types
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#ifndef __CODA_VPU_H__
#define __CODA_VPU_H__

#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>
#include "coda-vpuconfig.h"
#include "coda-vpuapi.h"

struct vpu_buffer {
	struct v4l2_m2m_buffer v4l2_m2m_buf;
	bool consumed;
	u32 average_qp;
};

enum vpu_fmt_type {
	VPU_FMT_TYPE_CODEC = 0,
	VPU_FMT_TYPE_RAW   = 1
};

struct vpu_format {
	unsigned int v4l2_pix_fmt;
	const struct v4l2_frmsize_stepwise *v4l2_frmsize;
};

static inline struct vpu_instance *coda_to_vpu_inst(struct v4l2_fh *vfh)
{
	return container_of(vfh, struct vpu_instance, v4l2_fh);
}

static inline struct vpu_instance *coda_ctrl_to_vpu_inst(struct v4l2_ctrl *vctrl)
{
	return container_of(vctrl->handler, struct vpu_instance, v4l2_ctrl_hdl);
}

static inline struct vpu_buffer *coda_to_vpu_buf(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct vpu_buffer, v4l2_m2m_buf.vb);
}

static inline bool coda_vpu_both_queues_are_streaming(struct vpu_instance *inst)
{
	struct vb2_queue *vq_cap = v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx);
	struct vb2_queue *vq_out = v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx);

	return vb2_is_streaming(vq_cap) && vb2_is_streaming(vq_out);
}

int coda_vpu_enc_register_device(struct vpu_device *vpu);
void coda_vpu_enc_unregister_device(struct vpu_device *vpu);

#endif
