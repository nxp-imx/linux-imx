/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave5 series multi-standard codec IP - basic types
 *
 * Copyright (C) 2026 CHIPS&MEDIA INC
 */
#ifndef __VPU_DRV_H__
#define __VPU_DRV_H__

#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>
#include "wave5-vpuconfig.h"
#include "wave5-vpuapi.h"

#define VPU_BUF_SYNC_TO_DEVICE 0
#define VPU_BUF_SYNC_FROM_DEVICE 1

struct vpu_src_buffer {
	struct v4l2_m2m_buffer	v4l2_m2m_buf;
	bool			consumed;
	ktime_t			ts_input;
	ktime_t			ts_start;
};

struct vpu_dst_buffer {
	struct v4l2_m2m_buffer	v4l2_m2m_buf;
	bool			display;
	bool			registered;
	bool			error;
	ktime_t			ts_input;
	ktime_t			ts_start;
	ktime_t			ts_finish;
	ktime_t			ts_output;
	u64			hw_time;
};

enum vpu_fmt_type {
	VPU_FMT_TYPE_CODEC = 0,
	VPU_FMT_TYPE_RAW   = 1
};

struct vpu_format {
	unsigned int v4l2_pix_fmt;
	const struct v4l2_frmsize_stepwise *v4l2_frmsize;
};

static inline struct vpu_instance *wave5_to_vpu_inst(struct v4l2_fh *vfh)
{
	return container_of(vfh, struct vpu_instance, v4l2_fh);
}

static inline struct vpu_instance *wave5_ctrl_to_vpu_inst(struct v4l2_ctrl *vctrl)
{
	return container_of(vctrl->handler, struct vpu_instance, v4l2_ctrl_hdl);
}

static inline struct vpu_src_buffer *wave5_to_vpu_src_buf(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct vpu_src_buffer, v4l2_m2m_buf.vb);
}

static inline struct vpu_dst_buffer *wave5_to_vpu_dst_buf(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct vpu_dst_buffer, v4l2_m2m_buf.vb);
}

void wave5_vpu_activate(struct vpu_device *dev);
void wave5_vpu_wait_activated(struct vpu_device *dev);
int wave5_vpu_wait_interrupt(struct vpu_instance *inst, unsigned int timeout);
int  wave5_vpu_dec_register_device(struct vpu_device *dev);
void wave5_vpu_dec_unregister_device(struct vpu_device *dev);
void wave5_vpu_enable_instance(struct vpu_instance *inst);
void wave5_vpu_disable_instance(struct vpu_instance *inst);
u32 wave5_vpu_cq_depth(struct vpu_device *vpu_dev);

#endif
