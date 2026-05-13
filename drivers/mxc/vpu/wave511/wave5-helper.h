/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave5 series multi-standard codec IP - basic types
 *
 * Copyright (C) 2026 CHIPS&MEDIA INC
 */

#ifndef __WAVE_HELPER_H__
#define __WAVE_HELPER_H__

#include "wave5-vpu.h"

#define FMT_TYPES	2
#define MAX_FMTS	14

typedef bool (*wave5_compare_vb)(struct vb2_v4l2_buffer *vbuf, unsigned long target);
const char *state_to_str(enum vpu_instance_state state);
void wave5_cleanup_instance(struct vpu_instance *inst, struct file *filp);
int wave5_vpu_release_device(struct file *filp,
			     int (*close_func)(struct vpu_instance *inst, u32 *fail_res),
			     char *name);
int wave5_vpu_queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq,
			 const struct vb2_ops *ops);
int wave5_vpu_subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub);
int wave5_vpu_g_fmt_out(struct file *file, void *fh, struct v4l2_format *f);
const struct vpu_format *wave5_find_vpu_fmt(unsigned int v4l2_pix_fmt,
					    const struct vpu_format fmt_list[MAX_FMTS]);
const struct vpu_format *wave5_find_vpu_fmt_by_idx(unsigned int idx,
						   const struct vpu_format fmt_list[MAX_FMTS]);
enum wave_std wave5_to_vpu_std(unsigned int v4l2_pix_fmt, enum vpu_instance_type type);
void wave5_return_bufs(struct vb2_queue *q, u32 state);
void wave5_update_pix_fmt(struct v4l2_pix_format_mplane *pix_mp,
			  int pix_fmt_type,
			  unsigned int width,
			  unsigned int height,
			  const struct v4l2_frmsize_stepwise *frmsize,
			  bool new_resolution);
void wave5_update_output_format_info(struct vpu_instance *inst);
struct vb2_v4l2_buffer *wave5_vpu_get_next_src_buf(struct vpu_instance *inst,
						   wave5_compare_vb compare,
						   unsigned long target);
struct vb2_v4l2_buffer *wave5_vpu_get_next_dst_buf(struct vpu_instance *inst,
						   wave5_compare_vb compare,
						   unsigned long target);
struct vb2_v4l2_buffer *wave5_vpu_get_dst_buffer_by_idx(struct vpu_instance *inst, int index);
struct vb2_v4l2_buffer *wave5_vpu_get_reusable_buffer(struct vpu_instance *inst, int index);
struct vb2_v4l2_buffer *wave5_vpu_get_display_buffer(struct vpu_instance *inst, int index);
bool wave5_vpu_check_fb_available(struct vpu_instance *inst);
void wave5_vpu_handle_performance(struct vpu_instance *inst, struct vpu_dst_buffer *vpu_buf);
void wave5_vpu_reset_performace(struct vpu_instance *inst);
dma_addr_t wave5_get_plane_dma_addr(struct vb2_buffer *buf, unsigned int plane_no);
unsigned long wave5_get_plane_payload(struct vb2_buffer *buf, unsigned int plane_no);

enum {
	WAVE5_VPU_FLOW_NONE,
	WAVE5_VPU_FLOW_SET_STATE,
	WAVE5_VPU_FLOW_OUTPUT_ON,
	WAVE5_VPU_FLOW_OUTPUT_OFF,
	WAVE5_VPU_FLOW_CAPTURE_ON,
	WAVE5_VPU_FLOW_CAPTURE_OFF,
	WAVE5_VPU_FLOW_START,
	WAVE5_VPU_FLOW_STOP,
	WAVE5_VPU_FLOW_SOURCE_CHANGE,
	WAVE5_VPU_FLOW_EOS,
	WAVE5_VPU_FLOW_MAXIMUM,
};

void wave5_vpu_record_flow(struct vpu_instance *inst, u32 flow, u32 arg1, u32 arg2);
#endif
