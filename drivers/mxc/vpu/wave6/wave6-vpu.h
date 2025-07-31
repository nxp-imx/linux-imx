/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - wave6 codec driver
 *
 * Copyright (C) 2025 CHIPS&MEDIA INC
 */

#ifndef __WAVE6_VPU_H__
#define __WAVE6_VPU_H__

#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include "wave6-vpuconfig.h"
#include "wave6-vpuapi.h"

struct vpu_buffer {
	struct v4l2_m2m_buffer v4l2_m2m_buf;
	bool consumed;
	bool used;
	bool error;
	bool force_key_frame;
	bool force_frame_qp;
	u32 force_i_frame_qp;
	u32 force_p_frame_qp;
	u32 force_b_frame_qp;
	ktime_t ts_input;
	ktime_t ts_start;
	ktime_t ts_finish;
	ktime_t ts_output;
	u64 hw_time;
	u32 average_qp;

	struct vpu_buf custom_qp_map;
};

enum vpu_fmt_type {
	VPU_FMT_TYPE_CODEC	= 0,
	VPU_FMT_TYPE_RAW	= 1
};

struct vpu_format {
	unsigned int v4l2_pix_fmt;
	unsigned int max_width;
	unsigned int min_width;
	unsigned int max_height;
	unsigned int min_height;
	unsigned int num_planes;

	enum frame_buffer_format src_format;
	enum endian_mode source_endian;
	enum packed_format_num packed_format;
	unsigned int csc_order;

	u32 is_yuv : 1;
	u32 is_rgb : 1;
	u32 is_10bit : 1;
	u32 cbcr_interleave : 1;
	u32 nv21 : 1;
};

static inline struct vpu_instance *wave6_to_vpu_inst(struct v4l2_fh *vfh)
{
	return container_of(vfh, struct vpu_instance, v4l2_fh);
}

static inline struct vpu_instance *wave6_ctrl_to_vpu_inst(struct v4l2_ctrl *vctrl)
{
	return container_of(vctrl->handler, struct vpu_instance, v4l2_ctrl_hdl);
}

static inline struct vpu_buffer *wave6_to_vpu_buf(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct vpu_buffer, v4l2_m2m_buf.vb);
}

static inline bool wave6_vpu_both_queues_are_streaming(struct vpu_instance *inst)
{
	struct vb2_queue *vq_cap = v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx);
	struct vb2_queue *vq_out = v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx);

	return vb2_is_streaming(vq_cap) && vb2_is_streaming(vq_out);
}

u32 wave6_vpu_get_consumed_fb_num(struct vpu_instance *inst);
u32 wave6_vpu_get_used_fb_num(struct vpu_instance *inst);
void wave6_vpu_pause(struct device *dev, int resume);
void wave6_vpu_activate(struct vpu_device *dev);
void wave6_vpu_wait_activated(struct vpu_device *dev);
void wave6_vpu_force_dma_sync_single_for_device(struct vpu_device *dev,
						dma_addr_t addr,
						size_t size,
						enum dma_data_direction dir);
void wave6_vpu_force_dma_sync_single_for_cpu(struct vpu_device *dev,
					     dma_addr_t addr,
					     size_t size,
					     enum dma_data_direction dir);
void wave6_update_pix_fmt(struct v4l2_pix_format_mplane *pix_mp,
			  unsigned int width,
			  unsigned int height);
struct vb2_v4l2_buffer *wave6_get_dst_buf_by_addr(struct vpu_instance *inst,
						  dma_addr_t addr);
dma_addr_t wave6_get_dma_addr(struct vb2_v4l2_buffer *buf,
			      unsigned int plane_no);
enum codec_std wave6_to_codec_std(enum vpu_instance_type type, unsigned int v4l2_pix_fmt);
const char *wave6_vpu_instance_state_name(u32 state);
void wave6_vpu_set_instance_state(struct vpu_instance *inst, u32 state);
u64 wave6_vpu_cycle_to_ns(struct vpu_device *vpu_dev, u64 cycle);
int wave6_vpu_wait_interrupt(struct vpu_instance *inst, unsigned int timeout);
int  wave6_vpu_dec_register_device(struct vpu_device *dev);
void wave6_vpu_dec_unregister_device(struct vpu_device *dev);
int  wave6_vpu_enc_register_device(struct vpu_device *dev);
void wave6_vpu_enc_unregister_device(struct vpu_device *dev);
void wave6_vpu_finish_job(struct vpu_instance *inst);
void wave6_vpu_handle_performance(struct vpu_instance *inst, struct vpu_buffer *vpu_buf);
void wave6_vpu_reset_performance(struct vpu_instance *inst);
int wave6_vpu_init_m2m_dev(struct vpu_device *dev);
void wave6_vpu_release_m2m_dev(struct vpu_device *dev);
int wave6_vpu_subscribe_event(struct v4l2_fh *fh,
			      const struct v4l2_event_subscription *sub);
void wave6_vpu_return_buffers(struct vpu_instance *inst,
			      unsigned int type, enum vb2_buffer_state state);

#endif /* __WAVE6_VPU_H__ */
