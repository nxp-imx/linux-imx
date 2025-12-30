// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave5 series multi-standard codec IP - decoder interface
 *
 * Copyright (C) 2021-2026 CHIPS&MEDIA INC
 */

#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/cleanup.h>
#include "wave5-helper.h"

#define VPU_DEC_DEV_NAME "C&M Wave5 VPU decoder"
#define VPU_DEC_DRV_NAME "wave5-dec"

static const struct v4l2_frmsize_stepwise dec_hevc_frmsize = {
	.min_width = W5_MIN_DEC_PIC_8_WIDTH,
	.max_width = W5_MAX_DEC_PIC_WIDTH,
	.step_width = W5_DEC_CODEC_STEP_WIDTH,
	.min_height = W5_MIN_DEC_PIC_8_HEIGHT,
	.max_height = W5_MAX_DEC_PIC_HEIGHT,
	.step_height = W5_DEC_CODEC_STEP_HEIGHT,
};

static const struct v4l2_frmsize_stepwise dec_h264_frmsize = {
	.min_width = W5_MIN_DEC_PIC_32_WIDTH,
	.max_width = W5_MAX_DEC_PIC_WIDTH,
	.step_width = W5_DEC_CODEC_STEP_WIDTH,
	.min_height = W5_MIN_DEC_PIC_32_HEIGHT,
	.max_height = W5_MAX_DEC_PIC_HEIGHT,
	.step_height = W5_DEC_CODEC_STEP_HEIGHT,
};

static const struct v4l2_frmsize_stepwise dec_raw_frmsize = {
	.min_width = W5_MIN_DEC_PIC_8_WIDTH,
	.max_width = W5_MAX_DEC_PIC_WIDTH,
	.step_width = W5_DEC_RAW_STEP_WIDTH,
	.min_height = W5_MIN_DEC_PIC_8_HEIGHT,
	.max_height = W5_MAX_DEC_PIC_HEIGHT,
	.step_height = W5_DEC_RAW_STEP_HEIGHT,
};

static const struct vpu_format dec_fmt_list[FMT_TYPES][MAX_FMTS] = {
	[VPU_FMT_TYPE_CODEC] = {
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_HEVC,
			.v4l2_frmsize = &dec_hevc_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_H264,
			.v4l2_frmsize = &dec_h264_frmsize,
		},
	},
	[VPU_FMT_TYPE_RAW] = {
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_YUV420,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV12,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV21,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_YUV422P,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV16,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV61,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_YUV420M,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV12M,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV21M,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_YUV422M,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV16M,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV61M,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_P010,
			.v4l2_frmsize = &dec_raw_frmsize,
		},
	}
};

/*
 * Make sure that the state switch is allowed and add logging for debugging
 * purposes
 */
static int switch_state(struct vpu_instance *inst, enum vpu_instance_state state)
{
	lockdep_assert_held(&inst->state_spinlock);

	switch (state) {
	case VPU_INST_STATE_NONE:
		break;
	case VPU_INST_STATE_OPEN:
		if (inst->state != VPU_INST_STATE_NONE)
			goto invalid_state_switch;
		goto valid_state_switch;
	case VPU_INST_STATE_INIT_SEQ:
		if (inst->state != VPU_INST_STATE_OPEN && inst->state != VPU_INST_STATE_STOP)
			goto invalid_state_switch;
		inst->v4l2_fh.m2m_ctx->ignore_cap_streaming = false;
		v4l2_m2m_set_dst_buffered(inst->v4l2_fh.m2m_ctx, false);
		goto valid_state_switch;
	case VPU_INST_STATE_PIC_RUN:
		if (inst->state != VPU_INST_STATE_INIT_SEQ)
			goto invalid_state_switch;
		goto valid_state_switch;
	case VPU_INST_STATE_STOP:
	case VPU_INST_STATE_ERROR:
		goto valid_state_switch;
	}
invalid_state_switch:
	WARN(1, "Invalid state switch from %s to %s.\n",
	     state_to_str(inst->state), state_to_str(state));
	return -EINVAL;
valid_state_switch:
	dev_dbg(inst->dev->dev, "Switch state from %s to %s.\n",
		state_to_str(inst->state), state_to_str(state));
	inst->state = state;
	return 0;
}

static void wave5_handle_src_buffer(struct vpu_instance *inst, dma_addr_t rd_ptr, u32 skip)
{
	struct vb2_v4l2_buffer *src_buf;
	struct vpu_src_buffer *vpu_buf;
	dma_addr_t start_addr;
	dma_addr_t wr_ptr;

	lockdep_assert_held(&inst->state_spinlock);

	src_buf = v4l2_m2m_next_src_buf(inst->v4l2_fh.m2m_ctx);
	if (!src_buf)
		return;
	vpu_buf = wave5_to_vpu_src_buf(src_buf);
	if (!vpu_buf->consumed) {
		dev_err(inst->dev->dev, "source %d is not consumed\n", src_buf->vb2_buf.index);
		return;
	}

	if (!skip) {
		start_addr = wave5_get_plane_dma_addr(&src_buf->vb2_buf, 0);
		wr_ptr = start_addr + wave5_get_plane_payload(&src_buf->vb2_buf, 0);
		if (rd_ptr < start_addr || rd_ptr > wr_ptr) {
			dev_dbg(inst->dev->dev, "[%pad, %pad] is not consumed\n",
				&start_addr, &wr_ptr);
			return;
		}

		if (rd_ptr != wr_ptr)
			dev_dbg(inst->dev->dev,
				"There is still data left in the source (%ld)\n",
				(unsigned long)(wr_ptr - rd_ptr));

		inst->timestamp = src_buf->vb2_buf.timestamp;
		inst->ts_input = vpu_buf->ts_input;
		inst->ts_start = max(vpu_buf->ts_start, inst->ts_last_end);
	} else {
		inst->skiped_frame_num++;
		inst->sequence++;
	}

	inst->processed_buf_num++;
	v4l2_m2m_src_buf_remove_by_buf(inst->v4l2_fh.m2m_ctx, src_buf);
	if (skip)
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
	else
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
}

static struct vpu_dst_buffer *wave5_get_unregistered_dst_buf(struct vpu_instance *inst)
{
	struct v4l2_m2m_buffer *v4l2_m2m_buf;
	struct vpu_dst_buffer *vpu_buf;

	lockdep_assert_held(&inst->state_spinlock);

	v4l2_m2m_for_each_dst_buf(inst->v4l2_fh.m2m_ctx, v4l2_m2m_buf) {
		vpu_buf = wave5_to_vpu_dst_buf(&v4l2_m2m_buf->vb);
		if (!vpu_buf->registered) {
			vpu_buf->registered = true;
			return vpu_buf;
		}
	}

	return NULL;
}

static struct vpu_dst_buffer *wave5_get_displayed_dst_buf(struct vpu_instance *inst)
{
	struct v4l2_m2m_buffer *v4l2_m2m_buf;
	struct vpu_dst_buffer *vpu_buf;

	lockdep_assert_held(&inst->state_spinlock);

	v4l2_m2m_for_each_dst_buf(inst->v4l2_fh.m2m_ctx, v4l2_m2m_buf) {
		vpu_buf = wave5_to_vpu_dst_buf(&v4l2_m2m_buf->vb);
		if (vpu_buf->display) {
			vpu_buf->display = false;
			return vpu_buf;
		}
	}

	return NULL;
}

static void wave5_handle_dst_buffer(struct vpu_instance *inst)
{
	struct vpu_dst_buffer *vpu_buf;
	struct frame_buffer frame = {0};
	int ret;

	scoped_guard(spinlock_irqsave, &inst->state_spinlock)
		vpu_buf = wave5_get_unregistered_dst_buf(inst);

	while (vpu_buf) {
		wave5_vpu_dec_fill_linear_frame(inst, &frame, &vpu_buf->v4l2_m2m_buf.vb.vb2_buf);
		ret = wave5_vpu_dec_register_display_buffer_ex(inst, &frame);
		if (ret) {
			dev_err(inst->dev->dev, "Fail to register capture buffer %d\n",
				vpu_buf->v4l2_m2m_buf.vb.vb2_buf.index);
			return;
		}

		scoped_guard(spinlock_irqsave, &inst->state_spinlock)
			vpu_buf = wave5_get_unregistered_dst_buf(inst);
	}

	scoped_guard(spinlock_irqsave, &inst->state_spinlock)
		vpu_buf = wave5_get_displayed_dst_buf(inst);

	while (vpu_buf) {
		ret = wave5_vpu_dec_clr_disp_flag(inst,
						  vpu_buf->v4l2_m2m_buf.vb.vb2_buf.index);
		if (ret) {
			dev_err(inst->dev->dev,
				"%s: Clearing the display flag of buffer index: %u, fail: %d\n",
				__func__, vpu_buf->v4l2_m2m_buf.vb.vb2_buf.index, ret);
			return;
		}

		scoped_guard(spinlock_irqsave, &inst->state_spinlock)
			vpu_buf = wave5_get_displayed_dst_buf(inst);
	}
}

static enum v4l2_quantization to_v4l2_quantization(bool video_full_range_flag)
{
	if (video_full_range_flag)
		return V4L2_QUANTIZATION_FULL_RANGE;
	else
		return V4L2_QUANTIZATION_LIM_RANGE;
}

static enum v4l2_colorspace to_v4l2_colorspace(u8 colour_primaries)
{
	switch (colour_primaries) {
	case 1:
		return V4L2_COLORSPACE_REC709;
	case 4:
		return V4L2_COLORSPACE_470_SYSTEM_M;
	case 5:
		return V4L2_COLORSPACE_470_SYSTEM_BG;
	case 6:
		return V4L2_COLORSPACE_SMPTE170M;
	case 7:
		return V4L2_COLORSPACE_SMPTE240M;
	case 9:
		return V4L2_COLORSPACE_BT2020;
	case 11:
		return V4L2_COLORSPACE_DCI_P3;
	default:
		return V4L2_COLORSPACE_DEFAULT;
	}
}

static enum v4l2_xfer_func to_v4l2_xfer_func(u8 transfer_characteristics)
{
	switch (transfer_characteristics) {
	case 1:
		return V4L2_XFER_FUNC_709;
	case 6:
		return V4L2_XFER_FUNC_709;
	case 7:
		return V4L2_XFER_FUNC_SMPTE240M;
	case 8:
		return V4L2_XFER_FUNC_NONE;
	case 13:
		return V4L2_XFER_FUNC_SRGB;
	case 14:
		return V4L2_XFER_FUNC_709;
	case 16:
		return V4L2_XFER_FUNC_SMPTE2084;
	default:
		return V4L2_XFER_FUNC_DEFAULT;
	}
}

static enum v4l2_ycbcr_encoding to_v4l2_ycbcr_encoding(u8 matrix_coeffs)
{
	switch (matrix_coeffs) {
	case 1:
		return V4L2_YCBCR_ENC_709;
	case 5:
		return V4L2_YCBCR_ENC_601;
	case 6:
		return V4L2_YCBCR_ENC_601;
	case 7:
		return V4L2_YCBCR_ENC_SMPTE240M;
	case 9:
		return V4L2_YCBCR_ENC_BT2020;
	case 10:
		return V4L2_YCBCR_ENC_BT2020_CONST_LUM;
	default:
		return V4L2_YCBCR_ENC_DEFAULT;
	}
}

static void wave5_update_color_info(struct vpu_instance *inst,
				    struct dec_initial_info *initial_info)
{
	struct color_param *color = &initial_info->color;

	if (!color->video_signal_type_present)
		goto set_default_all;

	inst->quantization = to_v4l2_quantization(color->color_range);

	if (!color->color_description_present)
		goto set_default_color;

	inst->colorspace = to_v4l2_colorspace(color->color_primaries);
	inst->xfer_func = to_v4l2_xfer_func(color->transfer_characteristics);
	inst->ycbcr_enc = to_v4l2_ycbcr_encoding(color->matrix_coefficients);

	return;

set_default_all:
	inst->quantization = V4L2_QUANTIZATION_DEFAULT;
set_default_color:
	inst->colorspace = V4L2_COLORSPACE_DEFAULT;
	inst->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	inst->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int start_decode(struct vpu_instance *inst)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	u32 fail_res = 0;
	int ret = 0;

	if (!inst->ts_last_end)
		inst->ts_last_end = ktime_get_raw();
	if (!inst->performance.ts_first)
		inst->performance.ts_first = ktime_get_raw();

	scoped_guard(spinlock_irqsave, &inst->state_spinlock) {
		if (inst->next_frame)
			atomic_inc(&inst->feed_frame_cnt);
		atomic_inc(&inst->queued_dec_cmd);
	}
	ret = wave5_vpu_dec_start_one_frame(inst, &fail_res);
	if (ret) {
		scoped_guard(spinlock_irqsave, &inst->state_spinlock) {
			if (inst->next_frame)
				atomic_dec_if_positive(&inst->feed_frame_cnt);
			atomic_dec_if_positive(&inst->queued_dec_cmd);

			if (fail_res != WAVE5_SYSERR_QUEUEING_FAIL) {
				if (inst->next_frame) {
					v4l2_m2m_src_buf_remove_by_buf(m2m_ctx, inst->next_frame);
					v4l2_m2m_buf_done(inst->next_frame, VB2_BUF_STATE_ERROR);
					inst->next_frame = NULL;
				}
				switch_state(inst, VPU_INST_STATE_ERROR);
				vb2_queue_error(v4l2_m2m_get_src_vq(m2m_ctx));
				vb2_queue_error(v4l2_m2m_get_dst_vq(m2m_ctx));
			} else {
				/*
				 * If the reason for failure is that the queue is full,
				 * try again later
				 */
				inst->retry_flag = true;
			}
		}

		dev_dbg(inst->dev->dev,
			"Frame decoding on m2m context (%p), fail: %d (result: %d)\n",
			m2m_ctx, ret, fail_res);
	} else {
		scoped_guard(spinlock_irqsave, &inst->state_spinlock) {
			if (inst->next_frame)
				inst->total_dec_cnt++;
			else
				inst->drain_dec_cnt++;
			inst->next_frame = NULL;
		}
	}

	return ret;
}

static void flag_last_buffer_done(struct vpu_instance *inst)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	struct vb2_v4l2_buffer *vb;
	int i;

	lockdep_assert_held(&inst->state_spinlock);

	vb = v4l2_m2m_dst_buf_remove(m2m_ctx);
	if (!vb) {
		m2m_ctx->is_draining = true;
		m2m_ctx->next_buf_last = true;
		return;
	}

	for (i = 0; i < vb->vb2_buf.num_planes; i++)
		vb2_set_plane_payload(&vb->vb2_buf, i, 0);
	vb->field = V4L2_FIELD_NONE;

	v4l2_m2m_last_buffer_done(m2m_ctx, vb);
}

static void send_eos_event(struct vpu_instance *inst)
{
	static const struct v4l2_event vpu_event_eos = {
		.type = V4L2_EVENT_EOS
	};

	lockdep_assert_held(&inst->state_spinlock);

	v4l2_event_queue_fh(&inst->v4l2_fh, &vpu_event_eos);
	inst->eos = false;
}

static int handle_dynamic_resolution_change(struct vpu_instance *inst)
{
	struct v4l2_fh *fh = &inst->v4l2_fh;
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;

	static const struct v4l2_event vpu_event_src_ch = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	struct dec_initial_info *initial_info = &inst->codec_info->dec_info.initial_info;

	lockdep_assert_held(&inst->state_spinlock);

	dev_dbg(inst->dev->dev, "%s: rd_ptr %pad", __func__, &initial_info->rd_ptr);

	dev_dbg(inst->dev->dev, "%s: width: %u height: %u profile: %u | minbuffer: %u delay %u\n",
		__func__, initial_info->pic_width, initial_info->pic_height,
		initial_info->profile, initial_info->min_frame_buffer_count,
		initial_info->reorder_delay);

	inst->needs_reallocation = true;
	inst->fbc_buf_count = initial_info->min_frame_buffer_count + 1;
	inst->disp_buf_count = max(initial_info->reorder_delay + 1, wave5_vpu_cq_depth(inst->dev));
	if (inst->disp_buf_count != v4l2_m2m_num_dst_bufs_ready(m2m_ctx)) {
		struct v4l2_ctrl *ctrl;

		ctrl = v4l2_ctrl_find(&inst->v4l2_ctrl_hdl,
				      V4L2_CID_MIN_BUFFERS_FOR_CAPTURE);
		if (ctrl)
			v4l2_ctrl_s_ctrl(ctrl, inst->disp_buf_count);
	}

	if (p_dec_info->initial_info_obtained) {
		const struct vpu_format *vpu_fmt;

		inst->conf_win.left = initial_info->pic_crop_rect.left;
		inst->conf_win.top = initial_info->pic_crop_rect.top;
		inst->conf_win.width = initial_info->pic_width -
			initial_info->pic_crop_rect.left - initial_info->pic_crop_rect.right;
		inst->conf_win.height = initial_info->pic_height -
			initial_info->pic_crop_rect.top - initial_info->pic_crop_rect.bottom;

		wave5_update_color_info(inst, initial_info);
		vpu_fmt = wave5_find_vpu_fmt(inst->src_fmt.pixelformat,
					     dec_fmt_list[VPU_FMT_TYPE_CODEC]);
		if (!vpu_fmt)
			return -EINVAL;

		wave5_update_pix_fmt(&inst->src_fmt,
				     VPU_FMT_TYPE_CODEC,
				     initial_info->pic_width,
				     initial_info->pic_height,
				     vpu_fmt->v4l2_frmsize,
				     true);

		if (initial_info->luma_bitdepth == 10)
			inst->dst_fmt.pixelformat = V4L2_PIX_FMT_P010;

		vpu_fmt = wave5_find_vpu_fmt(inst->dst_fmt.pixelformat,
					     dec_fmt_list[VPU_FMT_TYPE_RAW]);
		if (!vpu_fmt)
			return -EINVAL;

		wave5_update_pix_fmt(&inst->dst_fmt,
				     VPU_FMT_TYPE_RAW,
				     initial_info->pic_width,
				     initial_info->pic_height,
				     vpu_fmt->v4l2_frmsize,
				     true);
		wave5_update_output_format_info(inst);
	}

	v4l2_event_queue_fh(fh, &vpu_event_src_ch);

	return 0;
}

static u64 wave5_vpu_calc_hw_time(struct vpu_instance *inst, struct dec_output_info *dec_info)
{
	struct clk *vpu_clk = NULL;
	unsigned long rate;
	int i;

	if (!inst->dev->clks || !inst->dev->num_clks)
		return 0;

	for (i = 0; i < inst->dev->num_clks; i++) {
		if (inst->dev->clks[i].id && !strcmp(inst->dev->clks[i].id, "vpu")) {
			vpu_clk = inst->dev->clks[i].clk;
			break;
		}
	}

	if (!vpu_clk)
		return 0;

	rate = clk_get_rate(vpu_clk);
	if (!rate)
		return 0;

	return (dec_info->frame_cycle * NSEC_PER_SEC) / rate;
}

static void wave5_vpu_dec_decoding_error(struct vpu_instance *inst, struct dec_output_info *info)
{
	if (info->err_reason)
		dev_dbg(inst->dev->dev, "[%d] decoding %d error 0x%x\n",
			inst->id, inst->processed_buf_num, info->err_reason);
	if (info->warn_info)
		dev_dbg(inst->dev->dev, "[%d] decoding %d warn 0x%x\n",
			inst->id, inst->processed_buf_num, info->warn_info);
}

static void wave5_vpu_dec_finish_decode(struct vpu_instance *inst)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	struct dec_output_info dec_info;
	int ret;
	struct vb2_v4l2_buffer *dec_buf = NULL;
	struct vb2_v4l2_buffer *reuse_buf = NULL;
	struct vb2_v4l2_buffer *disp_buf = NULL;
	struct vb2_queue *dst_vq = v4l2_m2m_get_dst_vq(m2m_ctx);
	struct queue_status_info q_status;

	dev_dbg(inst->dev->dev, "%s: Fetch output info from firmware.", __func__);

	ret = wave5_vpu_dec_get_output_info(inst, &dec_info);
	if (ret) {
		dev_warn(inst->dev->dev, "%s: could not get output info.", __func__);
		goto exit;
	}

	wave5_vpu_dec_decoding_error(inst, &dec_info);

	scoped_guard(spinlock_irqsave, &inst->state_spinlock)
		wave5_handle_src_buffer(inst, dec_info.rd_ptr,
					dec_info.index_frame_decoded == DECODED_IDX_FLAG_SKIP);

	if (!vb2_is_streaming(dst_vq)) {
		dev_dbg(inst->dev->dev, "%s: capture is not streaming..", __func__);
		goto exit;
	}

	/* Remove decoded buffer from the ready queue now that it has been
	 * decoded.
	 */
	if (dec_info.index_frame_decoded >= 0) {
		scoped_guard(spinlock_irqsave, &inst->state_spinlock)
			dec_buf = wave5_get_decoded_buffer(inst,
							   dec_info.index_frame_decoded);
		if (dec_buf) {
			struct vpu_dst_buffer *dec_vpu_buf = wave5_to_vpu_dst_buf(dec_buf);

			dec_vpu_buf->ts_input = inst->ts_input;
			dec_vpu_buf->ts_start = inst->ts_start;
			dec_vpu_buf->ts_finish = ktime_get_raw();
			dec_vpu_buf->hw_time = wave5_vpu_calc_hw_time(inst, &dec_info);
			if (!inst->performance.first_hw_time)
				inst->performance.first_hw_time = dec_vpu_buf->hw_time;
			dec_buf->vb2_buf.timestamp = inst->timestamp;
			inst->ts_last_end = dec_vpu_buf->ts_finish;
		} else {
			dev_warn(inst->dev->dev, "%s: invalid decoded frame index %i",
				 __func__, dec_info.index_frame_decoded);
		}
	}

	if (dec_info.frame_display_flag) {
		for (int i = 0; i < WAVE5_MAX_FBS; i++) {
			if (!(BIT(i) & dec_info.frame_display_flag)) {
				scoped_guard(spinlock_irqsave, &inst->state_spinlock)
					reuse_buf = wave5_get_reusable_buffer(inst, i);

				if (reuse_buf)
					dev_dbg(inst->dev->dev, "%s: reuse buffer %d",
						__func__, i);
			}
		}
	}

	if (dec_info.index_frame_display >= 0) {
		scoped_guard(spinlock_irqsave, &inst->state_spinlock)
			disp_buf = wave5_get_display_buffer(inst,
							    dec_info.index_frame_display);
		if (!disp_buf)
			dev_warn(inst->dev->dev, "%s: invalid display frame index %i",
				 __func__, dec_info.index_frame_display);
	}

	/* If there is anything to display, do that now */
	if (disp_buf) {
		struct vpu_dst_buffer *dst_vpu_buf = wave5_to_vpu_dst_buf(disp_buf);

		dst_vpu_buf->ts_output = ktime_get_raw();
		inst->displayed_buf_num++;
		wave5_vpu_handle_performance(inst, dst_vpu_buf);

		if (inst->dst_fmt.num_planes == 1) {
			vb2_set_plane_payload(&disp_buf->vb2_buf, 0,
					      inst->dst_fmt.plane_fmt[0].sizeimage);
		} else if (inst->dst_fmt.num_planes == 2) {
			vb2_set_plane_payload(&disp_buf->vb2_buf, 0,
					      inst->dst_fmt.plane_fmt[0].sizeimage);
			vb2_set_plane_payload(&disp_buf->vb2_buf, 1,
					      inst->dst_fmt.plane_fmt[1].sizeimage);
		} else if (inst->dst_fmt.num_planes == 3) {
			vb2_set_plane_payload(&disp_buf->vb2_buf, 0,
					      inst->dst_fmt.plane_fmt[0].sizeimage);
			vb2_set_plane_payload(&disp_buf->vb2_buf, 1,
					      inst->dst_fmt.plane_fmt[1].sizeimage);
			vb2_set_plane_payload(&disp_buf->vb2_buf, 2,
					      inst->dst_fmt.plane_fmt[2].sizeimage);
		}

		disp_buf->sequence = inst->sequence++;
		/* TODO implement interlace support */
		disp_buf->field = V4L2_FIELD_NONE;
		v4l2_m2m_buf_done(disp_buf, VB2_BUF_STATE_DONE);

		dev_dbg(inst->dev->dev, "%s: frame_cycle %8lu (payload %lu)\n",
			__func__, dec_info.frame_cycle,
			wave5_get_plane_payload(&disp_buf->vb2_buf, 0));
	}

	if (dec_info.index_frame_display == DISPLAY_IDX_FLAG_SEQ_END ||
	    dec_info.sequence_changed) {
		scoped_guard(spinlock_irqsave, &inst->state_spinlock) {
			if (!v4l2_m2m_has_stopped(m2m_ctx)) {
				if (dec_info.sequence_changed) {
					inst->dynamic_source_change = true;
					handle_dynamic_resolution_change(inst);
				} else {
					send_eos_event(inst);
				}
				flag_last_buffer_done(inst);
				switch_state(inst, VPU_INST_STATE_STOP);
			}
		}
	}

	/*
	 * During a resolution change and while draining, the firmware may flush
	 * the reorder queue regardless of having a matching decoding operation
	 * pending. Only terminate the job if there are no more IRQ coming.
	 */
	wave5_vpu_dec_give_command(inst, DEC_GET_QUEUE_STATUS, &q_status);
	dev_dbg(inst->dev->dev, "instance_queue_count = %d, report_queue_count = %d\n",
		q_status.instance_queue_count, q_status.report_queue_count);

exit:
	scoped_guard(spinlock_irqsave, &inst->state_spinlock) {
		if (dec_info.index_frame_decoded >= 0 ||
		    dec_info.index_frame_decoded == DECODED_IDX_FLAG_SKIP)
			atomic_dec_if_positive(&inst->feed_frame_cnt);
		if (atomic_read(&inst->queued_dec_cmd) > atomic_read(&inst->feed_frame_cnt)) {
			atomic_dec_if_positive(&inst->queued_dec_cmd);
			inst->retry_flag = false;
		}
	}

	dev_dbg(inst->dev->dev,
		"[%d] dec_info rd_ptr %pad dec_idx %2i disp_idx %2i, queue %d, feed %d\n",
		inst->id, &dec_info.rd_ptr,
		dec_info.index_frame_decoded, dec_info.index_frame_display,
		atomic_read(&inst->queued_dec_cmd), atomic_read(&inst->feed_frame_cnt));


	dev_dbg(inst->dev->dev, "%s: finishing job.\n", __func__);
	v4l2_m2m_try_schedule(inst->v4l2_fh.m2m_ctx);
}

static int wave5_vpu_dec_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strscpy(cap->driver, VPU_DEC_DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, VPU_DEC_DRV_NAME, sizeof(cap->card));

	return 0;
}

static int wave5_vpu_dec_enum_framesizes(struct file *f, void *fh, struct v4l2_frmsizeenum *fsize)
{
	const struct vpu_format *vpu_fmt;

	if (fsize->index)
		return -EINVAL;

	vpu_fmt = wave5_find_vpu_fmt(fsize->pixel_format, dec_fmt_list[VPU_FMT_TYPE_CODEC]);
	if (!vpu_fmt) {
		vpu_fmt = wave5_find_vpu_fmt(fsize->pixel_format, dec_fmt_list[VPU_FMT_TYPE_RAW]);
		if (!vpu_fmt)
			return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = vpu_fmt->v4l2_frmsize->min_width;
	fsize->stepwise.max_width = vpu_fmt->v4l2_frmsize->max_width;
	fsize->stepwise.step_width = W5_DEC_CODEC_STEP_WIDTH;
	fsize->stepwise.min_height = vpu_fmt->v4l2_frmsize->min_height;
	fsize->stepwise.max_height = vpu_fmt->v4l2_frmsize->max_height;
	fsize->stepwise.step_height = W5_DEC_CODEC_STEP_HEIGHT;

	return 0;
}

static int wave5_vpu_dec_enum_fmt_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	const struct vpu_format *vpu_fmt;

	vpu_fmt = wave5_find_vpu_fmt_by_idx(f->index, dec_fmt_list[VPU_FMT_TYPE_RAW]);
	if (!vpu_fmt)
		return -EINVAL;

	f->pixelformat = vpu_fmt->v4l2_pix_fmt;
	f->flags = 0;

	return 0;
}

static int wave5_vpu_dec_try_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(file_to_v4l2_fh(file));
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	const struct v4l2_frmsize_stepwise *frmsize;
	const struct vpu_format *vpu_fmt;
	int width, height;

	dev_dbg(inst->dev->dev,
		"%s: fourcc: %u width: %u height: %u nm planes: %u colorspace: %u field: %u\n",
		__func__, f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		f->fmt.pix_mp.num_planes, f->fmt.pix_mp.colorspace, f->fmt.pix_mp.field);

	vpu_fmt = wave5_find_vpu_fmt(f->fmt.pix_mp.pixelformat, dec_fmt_list[VPU_FMT_TYPE_RAW]);
	if (!vpu_fmt) {
		width = inst->dst_fmt.width;
		height = inst->dst_fmt.height;
		f->fmt.pix_mp.pixelformat = inst->dst_fmt.pixelformat;
		frmsize = &dec_raw_frmsize;
	} else {
		width = f->fmt.pix_mp.width;
		height = f->fmt.pix_mp.height;
		f->fmt.pix_mp.pixelformat = vpu_fmt->v4l2_pix_fmt;
		frmsize = vpu_fmt->v4l2_frmsize;
	}

	if (p_dec_info->initial_info_obtained) {
		width = inst->dst_fmt.width;
		height = inst->dst_fmt.height;
	}

	wave5_update_pix_fmt(&f->fmt.pix_mp, VPU_FMT_TYPE_RAW,
			     width, height, frmsize, false);
	f->fmt.pix_mp.colorspace = inst->colorspace;
	f->fmt.pix_mp.ycbcr_enc = inst->ycbcr_enc;
	f->fmt.pix_mp.quantization = inst->quantization;
	f->fmt.pix_mp.xfer_func = inst->xfer_func;

	return 0;
}

static int wave5_vpu_dec_s_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(file_to_v4l2_fh(file));
	int i, ret;

	dev_dbg(inst->dev->dev,
		"%s: fourcc: %u width: %u height: %u num_planes: %u colorspace: %u field: %u\n",
		__func__, f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		f->fmt.pix_mp.num_planes, f->fmt.pix_mp.colorspace, f->fmt.pix_mp.field);

	ret = wave5_vpu_dec_try_fmt_cap(file, fh, f);
	if (ret)
		return ret;

	inst->dst_fmt.width = f->fmt.pix_mp.width;
	inst->dst_fmt.height = f->fmt.pix_mp.height;
	inst->dst_fmt.pixelformat = f->fmt.pix_mp.pixelformat;
	inst->dst_fmt.field = f->fmt.pix_mp.field;
	inst->dst_fmt.flags = f->fmt.pix_mp.flags;
	inst->dst_fmt.num_planes = f->fmt.pix_mp.num_planes;
	for (i = 0; i < inst->dst_fmt.num_planes; i++) {
		inst->dst_fmt.plane_fmt[i].bytesperline = f->fmt.pix_mp.plane_fmt[i].bytesperline;
		inst->dst_fmt.plane_fmt[i].sizeimage = f->fmt.pix_mp.plane_fmt[i].sizeimage;
	}

	wave5_update_output_format_info(inst);

	return 0;
}

static int wave5_vpu_dec_g_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(file_to_v4l2_fh(file));
	int i;

	f->fmt.pix_mp.width = inst->dst_fmt.width;
	f->fmt.pix_mp.height = inst->dst_fmt.height;
	f->fmt.pix_mp.pixelformat = inst->dst_fmt.pixelformat;
	f->fmt.pix_mp.field = inst->dst_fmt.field;
	f->fmt.pix_mp.flags = inst->dst_fmt.flags;
	f->fmt.pix_mp.num_planes = inst->dst_fmt.num_planes;
	for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		f->fmt.pix_mp.plane_fmt[i].bytesperline = inst->dst_fmt.plane_fmt[i].bytesperline;
		f->fmt.pix_mp.plane_fmt[i].sizeimage = inst->dst_fmt.plane_fmt[i].sizeimage;
	}

	f->fmt.pix_mp.colorspace = inst->colorspace;
	f->fmt.pix_mp.ycbcr_enc = inst->ycbcr_enc;
	f->fmt.pix_mp.quantization = inst->quantization;
	f->fmt.pix_mp.xfer_func = inst->xfer_func;

	return 0;
}

static int wave5_vpu_dec_enum_fmt_out(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(file_to_v4l2_fh(file));
	const struct vpu_format *vpu_fmt;

	dev_dbg(inst->dev->dev, "%s: index: %u\n", __func__, f->index);

	vpu_fmt = wave5_find_vpu_fmt_by_idx(f->index, dec_fmt_list[VPU_FMT_TYPE_CODEC]);
	if (!vpu_fmt)
		return -EINVAL;

	f->pixelformat = vpu_fmt->v4l2_pix_fmt;
	f->flags = V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED;

	return 0;
}

static int wave5_vpu_dec_try_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(file_to_v4l2_fh(file));
	const struct v4l2_frmsize_stepwise *frmsize;
	const struct vpu_format *vpu_fmt;
	int width, height;

	dev_dbg(inst->dev->dev,
		"%s: fourcc: %u width: %u height: %u num_planes: %u colorspace: %u field: %u\n",
		__func__, f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		f->fmt.pix_mp.num_planes, f->fmt.pix_mp.colorspace, f->fmt.pix_mp.field);

	vpu_fmt = wave5_find_vpu_fmt(f->fmt.pix_mp.pixelformat, dec_fmt_list[VPU_FMT_TYPE_CODEC]);
	if (!vpu_fmt) {
		width = inst->src_fmt.width;
		height = inst->src_fmt.height;
		f->fmt.pix_mp.pixelformat = inst->src_fmt.pixelformat;
		frmsize = &dec_hevc_frmsize;
	} else {
		width = f->fmt.pix_mp.width;
		height = f->fmt.pix_mp.height;
		f->fmt.pix_mp.pixelformat = vpu_fmt->v4l2_pix_fmt;
		frmsize = vpu_fmt->v4l2_frmsize;
	}

	wave5_update_pix_fmt(&f->fmt.pix_mp, VPU_FMT_TYPE_CODEC,
			     width, height, frmsize, false);
	f->fmt.pix_mp.colorspace = inst->colorspace;
	f->fmt.pix_mp.ycbcr_enc = inst->ycbcr_enc;
	f->fmt.pix_mp.quantization = inst->quantization;
	f->fmt.pix_mp.xfer_func = inst->xfer_func;

	return 0;
}

static int wave5_vpu_dec_s_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(file_to_v4l2_fh(file));
	struct v4l2_pix_format_mplane in_pix_mp = f->fmt.pix_mp;
	const struct vpu_format *vpu_fmt;
	int i, ret;

	dev_dbg(inst->dev->dev,
		"%s: fourcc: %u width: %u height: %u num_planes: %u field: %u\n",
		__func__, f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		f->fmt.pix_mp.num_planes, f->fmt.pix_mp.field);

	ret = wave5_vpu_dec_try_fmt_out(file, fh, f);
	if (ret)
		return ret;

	f->fmt.pix_mp.colorspace = in_pix_mp.colorspace;
	f->fmt.pix_mp.ycbcr_enc = in_pix_mp.ycbcr_enc;
	f->fmt.pix_mp.quantization = in_pix_mp.quantization;
	f->fmt.pix_mp.xfer_func = in_pix_mp.xfer_func;

	inst->std = wave5_to_vpu_std(f->fmt.pix_mp.pixelformat, inst->type);
	if (inst->std == STD_UNKNOWN) {
		dev_warn(inst->dev->dev, "unsupported pixelformat: %.4s\n",
			 (char *)&f->fmt.pix_mp.pixelformat);
		return -EINVAL;
	}

	inst->src_fmt.width = f->fmt.pix_mp.width;
	inst->src_fmt.height = f->fmt.pix_mp.height;
	inst->src_fmt.pixelformat = f->fmt.pix_mp.pixelformat;
	inst->src_fmt.field = f->fmt.pix_mp.field;
	inst->src_fmt.flags = f->fmt.pix_mp.flags;
	inst->src_fmt.num_planes = f->fmt.pix_mp.num_planes;
	for (i = 0; i < inst->src_fmt.num_planes; i++) {
		inst->src_fmt.plane_fmt[i].bytesperline = f->fmt.pix_mp.plane_fmt[i].bytesperline;
		inst->src_fmt.plane_fmt[i].sizeimage = f->fmt.pix_mp.plane_fmt[i].sizeimage;
	}

	inst->colorspace = f->fmt.pix_mp.colorspace;
	inst->ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	inst->quantization = f->fmt.pix_mp.quantization;
	inst->xfer_func = f->fmt.pix_mp.xfer_func;

	vpu_fmt = wave5_find_vpu_fmt(inst->dst_fmt.pixelformat, dec_fmt_list[VPU_FMT_TYPE_RAW]);
	if (!vpu_fmt)
		return -EINVAL;

	wave5_update_pix_fmt(&inst->dst_fmt, VPU_FMT_TYPE_RAW,
			     f->fmt.pix_mp.width, f->fmt.pix_mp.height,
			     vpu_fmt->v4l2_frmsize, true);

	return 0;
}

static int wave5_vpu_dec_g_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(file_to_v4l2_fh(file));

	dev_dbg(inst->dev->dev, "%s: type: %u | target: %u\n", __func__, s->type, s->target);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = inst->dst_fmt.width;
		s->r.height = inst->dst_fmt.height;
		break;
	case V4L2_SEL_TGT_COMPOSE_PADDED:
	case V4L2_SEL_TGT_COMPOSE:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = inst->src_fmt.width;
		s->r.height = inst->src_fmt.height;
		if (inst->scaler_info.enable) {
			s->r.width = inst->scaler_info.width;
			s->r.height = inst->scaler_info.height;
		} else if (inst->conf_win.width && inst->conf_win.height) {
			s->r = inst->conf_win;
		}
		break;
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = inst->src_fmt.width;
		s->r.height = inst->src_fmt.height;
		if (inst->conf_win.width && inst->conf_win.height)
			s->r = inst->conf_win;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wave5_vpu_dec_s_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(file_to_v4l2_fh(file));
	const struct vpu_format *vpu_fmt;
	int step = 2;
	int scale_width = 0, scale_height = 0;
	int min_scale_width = 0, min_scale_height = 0;
	u32 min_dec_pic_width = 0, min_dec_pic_height = 0;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	if (s->target != V4L2_SEL_TGT_COMPOSE)
		return -EINVAL;

	dev_dbg(inst->dev->dev, "V4L2_SEL_TGT_COMPOSE w: %u h: %u\n",
		s->r.width, s->r.height);

	if (!inst->dev->attr.support_scaler) {
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = inst->dst_fmt.width;
		s->r.height = inst->dst_fmt.height;

		return 0;
	}

	if (!(s->flags & (V4L2_SEL_FLAG_GE | V4L2_SEL_FLAG_LE)))
		s->flags |= V4L2_SEL_FLAG_LE;

	vpu_fmt = wave5_find_vpu_fmt(inst->src_fmt.pixelformat,
				     dec_fmt_list[VPU_FMT_TYPE_CODEC]);
	if (!vpu_fmt)
		return -EINVAL;

	min_dec_pic_width = vpu_fmt->v4l2_frmsize->min_width;
	min_dec_pic_height = vpu_fmt->v4l2_frmsize->min_height;

	scale_width = clamp(s->r.width, min_dec_pic_width,
			    round_up(inst->src_fmt.width, 32));
	scale_height = clamp(s->r.height, min_dec_pic_height,
			     inst->src_fmt.height);
	if (s->flags & V4L2_SEL_FLAG_GE) {
		scale_width = round_up(scale_width, step);
		scale_height = round_up(scale_height, step);
	}
	if (s->flags & V4L2_SEL_FLAG_LE) {
		scale_width = round_down(scale_width, step);
		scale_height = round_down(scale_height, step);
	}

	if (scale_width < inst->src_fmt.width ||
	    scale_height < inst->src_fmt.height)
		inst->scaler_info.enable = true;

	if (inst->scaler_info.enable) {
		min_scale_width = ALIGN((inst->src_fmt.width / 8), step);
		min_scale_height = ALIGN((inst->src_fmt.height / 8), step);

		if (scale_width < min_dec_pic_width)
			scale_width = min_dec_pic_width;
		if (scale_width < min_scale_width)
			scale_width = min_scale_width;
		if (scale_height < min_dec_pic_height)
			scale_height = min_dec_pic_height;
		if (scale_height < min_scale_height)
			scale_height = min_scale_height;

		inst->scaler_info.width = scale_width;
		inst->scaler_info.height = scale_height;
	}

	s->r.left = 0;
	s->r.top = 0;
	s->r.width = scale_width;
	s->r.height = scale_height;

	return 0;
}

static int wave5_vpu_dec_stop(struct vpu_instance *inst)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;

	if (m2m_ctx->is_draining)
		return -EBUSY;

	/*
	 * Used to remember the EOS state after the streamoff/on transition on
	 * the capture queue.
	 */
	inst->eos = true;

	if (m2m_ctx->has_stopped)
		return 0;

	m2m_ctx->last_src_buf = v4l2_m2m_last_src_buf(m2m_ctx);
	m2m_ctx->is_draining = true;
	v4l2_m2m_set_src_buffered(m2m_ctx, true);

	/*
	 * Deferred to device run in case it wasn't in the ring buffer
	 * yet. In other case, we have to send the EOS signal to the
	 * firmware so that any pending PIC_RUN ends without new
	 * bitstream buffer.
	 */
	if (m2m_ctx->last_src_buf)
		return 0;

	if (inst->state == VPU_INST_STATE_NONE) {
		send_eos_event(inst);
		flag_last_buffer_done(inst);
	}

	return 0;
}

static int wave5_vpu_dec_start(struct vpu_instance *inst)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	struct vb2_queue *dst_vq = v4l2_m2m_get_dst_vq(m2m_ctx);

	if (m2m_ctx->is_draining)
		return -EBUSY;

	if (m2m_ctx->has_stopped)
		m2m_ctx->has_stopped = false;

	vb2_clear_last_buffer_dequeued(dst_vq);
	inst->eos = false;

	return 0;
}

static int wave5_vpu_dec_decoder_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *dc)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(file_to_v4l2_fh(file));
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	int ret;

	dev_dbg(inst->dev->dev, "decoder command: %u\n", dc->cmd);

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, fh, dc);
	if (ret)
		return ret;

	switch (dc->cmd) {
	case V4L2_DEC_CMD_STOP:
		scoped_guard(spinlock_irqsave, &inst->state_spinlock)
			ret = wave5_vpu_dec_stop(inst);
		/* Just in case we don't have anything to decode anymore */
		v4l2_m2m_try_schedule(m2m_ctx);
		break;
	case V4L2_DEC_CMD_START:
		scoped_guard(spinlock_irqsave, &inst->state_spinlock)
			ret = wave5_vpu_dec_start(inst);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct v4l2_ioctl_ops wave5_vpu_dec_ioctl_ops = {
	.vidioc_querycap = wave5_vpu_dec_querycap,
	.vidioc_enum_framesizes = wave5_vpu_dec_enum_framesizes,

	.vidioc_enum_fmt_vid_cap	= wave5_vpu_dec_enum_fmt_cap,
	.vidioc_s_fmt_vid_cap_mplane = wave5_vpu_dec_s_fmt_cap,
	.vidioc_g_fmt_vid_cap_mplane = wave5_vpu_dec_g_fmt_cap,
	.vidioc_try_fmt_vid_cap_mplane = wave5_vpu_dec_try_fmt_cap,

	.vidioc_enum_fmt_vid_out	= wave5_vpu_dec_enum_fmt_out,
	.vidioc_s_fmt_vid_out_mplane = wave5_vpu_dec_s_fmt_out,
	.vidioc_g_fmt_vid_out_mplane = wave5_vpu_g_fmt_out,
	.vidioc_try_fmt_vid_out_mplane = wave5_vpu_dec_try_fmt_out,

	.vidioc_g_selection = wave5_vpu_dec_g_selection,
	.vidioc_s_selection = wave5_vpu_dec_s_selection,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	/*
	 * Firmware does not support CREATE_BUFS for CAPTURE queue. Since
	 * there is no immediate use-case for supporting CREATE_BUFS on
	 * just the OUTPUT queue, disable CREATE_BUFS altogether.
	 */
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd = wave5_vpu_dec_decoder_cmd,

	.vidioc_subscribe_event = wave5_vpu_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int wave5_vpu_dec_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
				     unsigned int *num_planes, unsigned int sizes[],
				     struct device *alloc_devs[])
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	unsigned int i;

	dev_dbg(inst->dev->dev, "%s: num_buffers: %u | num_planes: %u | type: %u\n", __func__,
		*num_buffers, *num_planes, q->type);

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		if (*num_buffers < wave5_vpu_cq_depth(inst->dev))
			*num_buffers = wave5_vpu_cq_depth(inst->dev);
		*num_planes = inst->src_fmt.num_planes;
		sizes[0] = inst->src_fmt.plane_fmt[0].sizeimage;
		dev_dbg(inst->dev->dev, "%s: size[0]: %u\n", __func__, sizes[0]);
	} else {
		*num_buffers = clamp(*num_buffers, inst->disp_buf_count, WAVE5_MAX_FBS);
		*num_planes = inst->dst_fmt.num_planes;
		for (i = 0; i < inst->dst_fmt.num_planes; i++) {
			sizes[i] = inst->dst_fmt.plane_fmt[i].sizeimage;
			dev_dbg(inst->dev->dev, "%s: size[%u]: %u\n", __func__, i, sizes[i]);
		}
	}

	return 0;
}

static int wave5_prepare_fb(struct vpu_instance *inst)
{
	int non_linear_num;
	int fb_stride = 0, fb_height = 0;
	int luma_size, chroma_size;
	int ret, i;
	u32 bitdepth = inst->codec_info->dec_info.initial_info.luma_bitdepth;

	switch (bitdepth) {
	case 8:
		break;
	case 10:
		if (inst->std == W_AVC_DEC &&
		    inst->dev->attr.support_avc10bit_dec)
			break;

		if (inst->std == W_HEVC_DEC &&
		    inst->dev->attr.support_hevc10bit_dec)
			break;

		fallthrough;
	default:
		dev_err(inst->dev->dev, "no support for %d bit depth\n", bitdepth);

		return -EINVAL;
	}

	non_linear_num = inst->fbc_buf_count;

	for (i = 0; i < non_linear_num; i++) {
		struct frame_buffer *frame = &inst->frame_buf[i];
		struct vpu_buf *vframe = &inst->frame_vbuf[i];

		fb_stride = ALIGN(inst->dst_fmt.width * bitdepth / 8, 32);
		fb_height = ALIGN(inst->dst_fmt.height, 32);
		luma_size = fb_stride * fb_height;

		chroma_size = ALIGN(fb_stride / 2, 16) * fb_height;

		if (vframe->size == (luma_size + chroma_size))
			continue;

		if (vframe->size)
			wave5_vpu_dec_reset_framebuffer(inst, i);

		vframe->size = luma_size + chroma_size;
		ret = wave5_vdi_allocate_dma_memory(inst->dev->dev, vframe);
		if (ret) {
			dev_dbg(inst->dev->dev,
				"%s: Allocating FBC buf of size %zu, fail: %d\n",
				__func__, vframe->size, ret);
			return ret;
		}

		frame->buf_y = vframe->daddr;
		frame->buf_cb = vframe->daddr + luma_size;
		frame->buf_cr = (dma_addr_t)-1;
		frame->size = vframe->size;
		frame->width = inst->src_fmt.width;
		frame->stride = fb_stride;
		frame->map_type = COMPRESSED_FRAME_MAP;
		frame->update_fb_info = true;
	}
	/* In case the count has reduced, clean up leftover framebuffer memory */
	for (i = non_linear_num; i < WAVE5_MAX_FBS; i++) {
		ret = wave5_vpu_dec_reset_framebuffer(inst, i);
		if (ret)
			break;
	}

	ret = wave5_vpu_dec_register_frame_buffer_ex(inst, inst->fbc_buf_count,
						     inst->frame_buf[0].stride,
						     inst->dst_fmt.height);
	if (ret)
		dev_err(inst->dev->dev, "vpu_dec_register_frame_buffer_ex fail: %d", ret);

	return 0;
}

static int wave5_vpu_dec_fill_source(struct vpu_instance *inst)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	struct v4l2_m2m_buffer *buf, *n;

	lockdep_assert_held(&inst->state_spinlock);

	if (m2m_ctx->last_src_buf) {
		struct vpu_src_buffer *vpu_buf = wave5_to_vpu_src_buf(m2m_ctx->last_src_buf);

		if (vpu_buf->consumed) {
			dev_dbg(inst->dev->dev, "last src buffer already written\n");
			return 0;
		}
	}

	v4l2_m2m_for_each_src_buf_safe(m2m_ctx, buf, n) {
		struct vb2_v4l2_buffer *vbuf = &buf->vb;
		struct vpu_src_buffer *vpu_buf = wave5_to_vpu_src_buf(vbuf);

		/*only fill one frame into ring buffer*/
		/*TODO, if only sequence header is fileed, need to fill frame*/
		if (inst->next_frame)
			break;

		if (vpu_buf->consumed) {
			dev_dbg(inst->dev->dev, "already copied src buf (%u) to the ring buffer\n",
				vbuf->vb2_buf.index);
			continue;
		}

		vpu_buf->consumed = true;
		vpu_buf->ts_start = ktime_get_raw();
		inst->next_frame = vbuf;

		/* Don't write buffers passed the last one while draining. */
		if (v4l2_m2m_is_last_draining_src_buf(m2m_ctx, vbuf)) {
			dev_dbg(inst->dev->dev, "last src buffer written to the ring buffer\n");
			break;
		}
	}

	return 0;
}

static int wave5_vpu_dec_buf_prepare(struct vb2_buffer *vb)
{
	struct vpu_instance *inst = vb2_get_drv_priv(vb->vb2_queue);
	int i;

	for (i = 0; i < vb->num_planes; i++) {
		if (!IS_ALIGNED(wave5_get_plane_dma_addr(vb, i), W5_DEC_ADDR_ALIGNMENT)) {
			dev_err(inst->dev->dev,
				"plane[%d] address is not %d aligned\n", i, W5_DEC_ADDR_ALIGNMENT);
			return -EINVAL;
		}
	}

	return 0;
}

static void wave5_vpu_dec_buf_queue_src(struct vb2_buffer *vb)
{
	struct vpu_instance *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vpu_src_buffer *vpu_buf = wave5_to_vpu_src_buf(vbuf);

	vpu_buf->consumed = false;
	vbuf->sequence = inst->queued_src_buf_num++;
	vpu_buf->ts_input = ktime_get_raw();

	v4l2_m2m_buf_queue(m2m_ctx, vbuf);
}

static void wave5_vpu_dec_buf_queue_dst(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vpu_dst_buffer *vpu_buf = wave5_to_vpu_dst_buf(vbuf);
	struct vpu_instance *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;

	inst->queued_dst_buf_num++;
	vpu_buf->decoded = false;

	if (vb2_is_streaming(vb->vb2_queue) && v4l2_m2m_dst_buf_is_last(m2m_ctx)) {
		unsigned int i;

		for (i = 0; i < vb->num_planes; i++)
			vb2_set_plane_payload(vb, i, 0);

		vbuf->field = V4L2_FIELD_NONE;

		send_eos_event(inst);
		v4l2_m2m_last_buffer_done(m2m_ctx, vbuf);
	} else {
		v4l2_m2m_buf_queue(m2m_ctx, vbuf);
	}
}

static void wave5_vpu_dec_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vpu_instance *inst = vb2_get_drv_priv(vb->vb2_queue);

	dev_dbg(inst->dev->dev, "%s: type: %4u index: %4u size: ([0]=%4lu, [1]=%4lu, [2]=%4lu)\n",
		__func__, vb->type, vb->index, vb2_plane_size(&vbuf->vb2_buf, 0),
		vb2_plane_size(&vbuf->vb2_buf, 1), vb2_plane_size(&vbuf->vb2_buf, 2));

	if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		wave5_vpu_dec_buf_queue_src(vb);
	else if (vb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		wave5_vpu_dec_buf_queue_dst(vb);
}

static int wave5_vpu_dec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	int ret = 0;

	dev_dbg(inst->dev->dev, "streamon %s\n",
		V4L2_TYPE_IS_OUTPUT(q->type) ? "output" : "capture");

	v4l2_m2m_update_start_streaming_state(m2m_ctx, q);

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE && inst->state == VPU_INST_STATE_NONE) {
		struct dec_open_param open_param;
		struct v4l2_ctrl *ctrl;

		memset(&open_param, 0, sizeof(struct dec_open_param));

		wave5_vpu_activate(inst->dev);
		ret = pm_runtime_resume_and_get(inst->dev->dev);
		if (ret) {
			dev_err(inst->dev->dev, "runtime_resume failed %d\n", ret);
			goto return_buffers;
		}

		wave5_vpu_wait_activated(inst->dev);

		open_param.bitstream_buffer = 0;
		open_param.bitstream_buffer_size = 0;
		ctrl = v4l2_ctrl_find(&inst->v4l2_ctrl_hdl,
				      V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY_ENABLE);
		if (ctrl)
			open_param.reorder_enable = v4l2_ctrl_g_ctrl(ctrl) ? false : true;
		else
			open_param.reorder_enable = true;

		ret = wave5_vpu_dec_open(inst, &open_param);
		if (ret) {
			dev_dbg(inst->dev->dev, "%s: decoder opening, fail: %d\n",
				__func__, ret);
			goto return_buffers;
		}

		scoped_guard(spinlock_irqsave, &inst->state_spinlock)
			ret = switch_state(inst, VPU_INST_STATE_OPEN);
		if (ret)
			goto return_buffers;
	} else if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (inst->state == VPU_INST_STATE_STOP) {
			scoped_guard(spinlock_irqsave, &inst->state_spinlock)
				ret = switch_state(inst, VPU_INST_STATE_INIT_SEQ);
		}
		if (ret)
			goto return_buffers;
	}

	return ret;

return_buffers:
	scoped_guard(spinlock_irqsave, &inst->state_spinlock)
		wave5_return_bufs(q, VB2_BUF_STATE_QUEUED);
	pm_runtime_put_sync(inst->dev->dev);
	return ret;
}

static int streamoff_output(struct vb2_queue *q)
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	int ret;

	scoped_guard(spinlock_irqsave, &inst->state_spinlock)
		wave5_return_bufs(v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx), VB2_BUF_STATE_ERROR);

	ret = wave5_vpu_flush_instance(inst);
	if (ret)
		return ret;

	inst->next_frame = NULL;
	v4l2_m2m_set_src_buffered(m2m_ctx, false);
	atomic_set(&inst->feed_frame_cnt, 0);

	wave5_vpu_reset_performace(inst);
	inst->queued_src_buf_num = 0;
	inst->queued_dst_buf_num = 0;
	inst->processed_buf_num = 0;
	inst->displayed_buf_num = 0;
	inst->skiped_frame_num = 0;
	inst->sequence = 0;
	inst->total_dec_cnt = 0;
	inst->drain_dec_cnt = 0;

	if (v4l2_m2m_has_stopped(m2m_ctx))
		send_eos_event(inst);

	/* streamoff on output cancels any draining operation */
	inst->eos = false;

	return 0;
}

static int streamoff_capture(struct vb2_queue *q)
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	int ret = 0;

	scoped_guard(spinlock_irqsave, &inst->state_spinlock)
		wave5_return_bufs(v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx), VB2_BUF_STATE_ERROR);

	if (inst->needs_reallocation) {
		wave5_vpu_dec_give_command(inst, DEC_RESET_FRAMEBUF_INFO, NULL);
		inst->needs_reallocation = false;
	}

	if (v4l2_m2m_has_stopped(m2m_ctx)) {
		scoped_guard(spinlock_irqsave, &inst->state_spinlock)
			ret = switch_state(inst, VPU_INST_STATE_INIT_SEQ);
		if (ret)
			return ret;
	}

	inst->v4l2_fh.m2m_ctx->ignore_cap_streaming = false;
	v4l2_m2m_set_dst_buffered(inst->v4l2_fh.m2m_ctx, false);

	return 0;
}

static void wave5_vpu_dec_stop_streaming(struct vb2_queue *q)
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	bool check_cmd = inst->dynamic_source_change ? false : true;

	dev_dbg(inst->dev->dev, "streamoff %s\n",
		V4L2_TYPE_IS_OUTPUT(q->type) ? "output" : "capture");

	while (check_cmd) {
		struct queue_status_info q_status;
		struct dec_output_info dec_output_info;

		wave5_vpu_dec_give_command(inst, DEC_GET_QUEUE_STATUS, &q_status);

		if (q_status.report_queue_count == 0)
			break;

		if (wave5_vpu_wait_interrupt(inst, VPU_DEC_TIMEOUT_MS) < 0)
			break;

		if (wave5_vpu_dec_get_output_info(inst, &dec_output_info))
			dev_dbg(inst->dev->dev, "Getting decoding results from fw, fail\n");
	}

	v4l2_m2m_update_stop_streaming_state(m2m_ctx, q);

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		streamoff_output(q);
	else
		streamoff_capture(q);
}

static int wave5_vpu_dec_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (V4L2_TYPE_IS_CAPTURE(vb->type)) {
		struct vpu_dst_buffer *vpu_buf = wave5_to_vpu_dst_buf(vbuf);

		vpu_buf->registered = false;
		vpu_buf->decoded = false;
		vpu_buf->display = false;
	}

	return 0;
}

static const struct vb2_ops wave5_vpu_dec_vb2_ops = {
	.queue_setup = wave5_vpu_dec_queue_setup,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.buf_queue = wave5_vpu_dec_buf_queue,
	.buf_init = wave5_vpu_dec_buf_init,
	.buf_prepare = wave5_vpu_dec_buf_prepare,
	.start_streaming = wave5_vpu_dec_start_streaming,
	.stop_streaming = wave5_vpu_dec_stop_streaming,
};

static void wave5_set_default_format(struct v4l2_pix_format_mplane *src_fmt,
				     struct v4l2_pix_format_mplane *dst_fmt)
{
	src_fmt->pixelformat = dec_fmt_list[VPU_FMT_TYPE_CODEC][0].v4l2_pix_fmt;
	wave5_update_pix_fmt(src_fmt, VPU_FMT_TYPE_CODEC,
			     W5_DEF_DEC_PIC_WIDTH, W5_DEF_DEC_PIC_HEIGHT,
			     &dec_hevc_frmsize, true);

	dst_fmt->pixelformat = dec_fmt_list[VPU_FMT_TYPE_RAW][0].v4l2_pix_fmt;
	wave5_update_pix_fmt(dst_fmt, VPU_FMT_TYPE_RAW,
			     W5_DEF_DEC_PIC_WIDTH, W5_DEF_DEC_PIC_HEIGHT,
			     &dec_raw_frmsize, true);
}

static int wave5_vpu_dec_queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	return wave5_vpu_queue_init(priv, src_vq, dst_vq, &wave5_vpu_dec_vb2_ops);
}

static const struct vpu_instance_ops wave5_vpu_dec_inst_ops = {
	.finish_process = wave5_vpu_dec_finish_decode,
};

static int initialize_sequence(struct vpu_instance *inst)
{
	struct dec_initial_info initial_info;
	int ret = 0;

	memset(&initial_info, 0, sizeof(struct dec_initial_info));

	ret = wave5_vpu_dec_issue_seq_init(inst);
	if (ret) {
		dev_err(inst->dev->dev, "[%d] wave5_vpu_dec_issue_seq_init, fail: %d\n",
			inst->id, ret);
		return ret;
	}

	if (wave5_vpu_wait_interrupt(inst, VPU_DEC_TIMEOUT_MS) < 0)
		dev_err(inst->dev->dev, "[%d] failed to call vpu_wait_interrupt()\n", inst->id);

	ret = wave5_vpu_dec_complete_seq_init(inst, &initial_info);
	if (ret) {
		dev_err(inst->dev->dev, "[%d] vpu_dec_complete_seq_init, fail: %d, reason: 0x%x\n",
			inst->id, ret, initial_info.err_reason);
		scoped_guard(spinlock_irqsave, &inst->state_spinlock) {
			if (inst->next_frame) {
				v4l2_m2m_src_buf_remove_by_buf(inst->v4l2_fh.m2m_ctx,
							       inst->next_frame);
				v4l2_m2m_buf_done(inst->next_frame, VB2_BUF_STATE_ERROR);
				inst->next_frame = NULL;
			}
		}

		return ret;
	}

	handle_dynamic_resolution_change(inst);

	return 0;
}

static bool wave5_is_draining_or_eos(struct vpu_instance *inst)
{
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;

	lockdep_assert_held(&inst->state_spinlock);
	return m2m_ctx->is_draining || inst->eos;
}

static void wave5_vpu_dec_device_run(void *priv)
{
	struct vpu_instance *inst = priv;
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	struct queue_status_info q_status;
	int ret = 0;

	scoped_guard(spinlock_irqsave, &inst->state_spinlock)
		ret = wave5_vpu_dec_fill_source(inst);
	if (ret) {
		dev_warn(inst->dev->dev, "[%d] Filling ring buffer failed, state = %d\n",
			 inst->id, inst->state);
		goto finish_job_and_return;
	}

	switch (inst->state) {
	case VPU_INST_STATE_OPEN:
		if (!inst->next_frame) {
			dev_warn(inst->dev->dev, "[%d] there is no input frame\n", inst->id);
			break;
		}
		ret = initialize_sequence(inst);
		if (ret) {
			vb2_queue_error(v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx));
			vb2_queue_error(v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx));
			/*
			 * Don't switch state to STOP, as it may confuse
			 * the drain or dynamic source change case
			 */
			scoped_guard(spinlock_irqsave, &inst->state_spinlock)
				switch_state(inst, VPU_INST_STATE_ERROR);
		} else {
			scoped_guard(spinlock_irqsave, &inst->state_spinlock)
				switch_state(inst, VPU_INST_STATE_INIT_SEQ);
		}

		break;

	case VPU_INST_STATE_INIT_SEQ:
		/*
		 * Do this early, preparing the fb can trigger an IRQ before
		 * we had a chance to switch, which leads to an invalid state
		 * change.
		 */
		scoped_guard(spinlock_irqsave, &inst->state_spinlock)
			switch_state(inst, VPU_INST_STATE_PIC_RUN);

		/*
		 * During DRC, the picture decoding remains pending, so just leave the job
		 * active until this decode operation completes.
		 */
		wave5_vpu_dec_give_command(inst, DEC_GET_QUEUE_STATUS, &q_status);

		wave5_handle_dst_buffer(inst);
		/*
		 * The sequence must be analyzed first to calculate the proper
		 * size of the auxiliary buffers.
		 */
		ret = wave5_prepare_fb(inst);
		if (ret) {
			dev_warn(inst->dev->dev, "Framebuffer preparation, fail: %d\n", ret);
			scoped_guard(spinlock_irqsave, &inst->state_spinlock)
				switch_state(inst, VPU_INST_STATE_ERROR);
			vb2_queue_error(v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx));
			vb2_queue_error(v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx));
			break;
		}
		if (inst->dynamic_source_change) {
			inst->dynamic_source_change = false;
			break;
		}

		fallthrough;
	case VPU_INST_STATE_PIC_RUN:
		wave5_handle_dst_buffer(inst);
		if (inst->dynamic_source_change && atomic_read(&inst->queued_dec_cmd))
			break;

		ret = start_decode(inst);
		if (ret)
			break;

		/* Return so that we leave this job active */
		dev_dbg(inst->dev->dev, "%s: leave with active job", __func__);
		break;
	default:
		dev_warn(inst->dev->dev, "Execution of a job in state %s illegal.\n",
			 state_to_str(inst->state));
		break;
	}

finish_job_and_return:
	dev_dbg(inst->dev->dev, "%s: leave and finish job", __func__);
	v4l2_m2m_job_finish(inst->v4l2_m2m_dev, m2m_ctx);
}

static bool wave5_vpu_check_input(struct vpu_instance *inst)
{
	struct v4l2_m2m_buffer *v4l2_m2m_buf;

	lockdep_assert_held(&inst->state_spinlock);

	if (v4l2_m2m_has_stopped(inst->v4l2_fh.m2m_ctx))
		return false;

	if (inst->retry_flag)
		return false;

	if (wave5_is_draining_or_eos(inst) && !atomic_read(&inst->queued_dec_cmd))
		return true;

	if (inst->next_frame)
		return true;

	v4l2_m2m_for_each_src_buf(inst->v4l2_fh.m2m_ctx, v4l2_m2m_buf) {
		struct vb2_v4l2_buffer *vbuf = &v4l2_m2m_buf->vb;
		struct vpu_src_buffer *vpu_buf = wave5_to_vpu_src_buf(vbuf);

		if (!vpu_buf->consumed)
			return true;
	}

	return false;
}

static int wave5_vpu_dec_job_ready(void *priv)
{
	struct vpu_instance *inst = priv;
	int ret = 0;

	guard(spinlock_irqsave)(&inst->state_spinlock);

	switch (inst->state) {
	case VPU_INST_STATE_OPEN:
	case VPU_INST_STATE_INIT_SEQ:
		ret = 1;
		break;
	case VPU_INST_STATE_PIC_RUN:
		if (atomic_read(&inst->queued_dec_cmd) >= wave5_vpu_cq_depth(inst->dev))
			break;
		if (wave5_vpu_check_input(inst) &&
		    wave5_vpu_check_fb_available(inst))
			ret = 1;
		break;
	default:
		break;
	}

	return ret;
}

static const struct v4l2_m2m_ops wave5_vpu_dec_m2m_ops = {
	.device_run = wave5_vpu_dec_device_run,
	.job_ready = wave5_vpu_dec_job_ready,
};

static int wave5_vpu_open_dec(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_device *dev = video_drvdata(filp);
	struct vpu_instance *inst = NULL;
	struct v4l2_m2m_ctx *m2m_ctx;
	int ret = 0;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->dev = dev;
	inst->type = VPU_INST_TYPE_DEC;
	inst->ops = &wave5_vpu_dec_inst_ops;
	inst->id = -1;

	spin_lock_init(&inst->state_spinlock);

	inst->codec_info = kzalloc(sizeof(*inst->codec_info), GFP_KERNEL);
	if (!inst->codec_info)
		return -ENOMEM;

	v4l2_fh_init(&inst->v4l2_fh, vdev);
	v4l2_fh_add(&inst->v4l2_fh, filp);

	atomic_set(&inst->feed_frame_cnt, 0);
	atomic_set(&inst->queued_dec_cmd, 0);
	INIT_LIST_HEAD(&inst->list);

	inst->v4l2_m2m_dev = inst->dev->v4l2_m2m_dec_dev;
	inst->v4l2_fh.m2m_ctx =
		v4l2_m2m_ctx_init(inst->v4l2_m2m_dev, inst, wave5_vpu_dec_queue_init);
	if (IS_ERR(inst->v4l2_fh.m2m_ctx)) {
		ret = PTR_ERR(inst->v4l2_fh.m2m_ctx);
		goto cleanup_inst;
	}
	m2m_ctx = inst->v4l2_fh.m2m_ctx;

	v4l2_m2m_set_dst_buffered(m2m_ctx, true);
	/*
	 * We use the M2M job queue to ensure synchronization of steps where
	 * needed, as IOCTLs can occur at anytime and we need to run commands on
	 * the firmware in a specified order.
	 * In order to initialize the sequence on the firmware within an M2M
	 * job, the M2M framework needs to be able to queue jobs before
	 * the CAPTURE queue has been started, because we need the results of the
	 * initialization to properly prepare the CAPTURE queue with the correct
	 * amount of buffers.
	 * By setting ignore_cap_streaming to true the m2m framework will call
	 * job_ready as soon as the OUTPUT queue is streaming, instead of
	 * waiting until both the CAPTURE and OUTPUT queues are streaming.
	 */
	m2m_ctx->ignore_cap_streaming = true;

	v4l2_ctrl_handler_init(&inst->v4l2_ctrl_hdl, 10);
	v4l2_ctrl_new_std(&inst->v4l2_ctrl_hdl, NULL,
			  V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, 1, 32, 1, 1);
	v4l2_ctrl_new_std(&inst->v4l2_ctrl_hdl, NULL,
			  V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY, 0, 0, 1, 0);
	v4l2_ctrl_new_std(&inst->v4l2_ctrl_hdl, NULL,
			  V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY_ENABLE, 0, 1, 1, 0);

	if (inst->v4l2_ctrl_hdl.error) {
		ret = -ENODEV;
		goto cleanup_inst;
	}

	inst->v4l2_fh.ctrl_handler = &inst->v4l2_ctrl_hdl;
	v4l2_ctrl_handler_setup(&inst->v4l2_ctrl_hdl);

	wave5_set_default_format(&inst->src_fmt, &inst->dst_fmt);
	inst->colorspace = V4L2_COLORSPACE_DEFAULT;
	inst->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	inst->quantization = V4L2_QUANTIZATION_DEFAULT;
	inst->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	init_completion(&inst->irq_done);

	scoped_guard(mutex, &dev->dev_lock) {
		if (dev->irq < 0 && !hrtimer_active(&dev->hrtimer) && list_empty(&dev->instances))
			hrtimer_start(&dev->hrtimer,
				      ns_to_ktime(dev->vpu_poll_interval * NSEC_PER_MSEC),
				      HRTIMER_MODE_REL_PINNED);
	}

	return 0;

cleanup_inst:
	wave5_cleanup_instance(inst, filp);
	return ret;
}

static int wave5_vpu_dec_release(struct file *filp)
{
	return wave5_vpu_release_device(filp, wave5_vpu_dec_close, "decoder");
}

static const struct v4l2_file_operations wave5_vpu_dec_fops = {
	.owner = THIS_MODULE,
	.open = wave5_vpu_open_dec,
	.release = wave5_vpu_dec_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
};

int wave5_vpu_dec_register_device(struct vpu_device *dev)
{
	struct video_device *vdev_dec;
	int ret;

	vdev_dec = devm_kzalloc(dev->v4l2_dev.dev, sizeof(*vdev_dec), GFP_KERNEL);
	if (!vdev_dec)
		return -ENOMEM;

	dev->v4l2_m2m_dec_dev = v4l2_m2m_init(&wave5_vpu_dec_m2m_ops);
	if (IS_ERR(dev->v4l2_m2m_dec_dev)) {
		ret = PTR_ERR(dev->v4l2_m2m_dec_dev);
		dev_err(dev->dev, "v4l2_m2m_init, fail: %d\n", ret);
		return -EINVAL;
	}

	dev->video_dev_dec = vdev_dec;

	strscpy(vdev_dec->name, VPU_DEC_DEV_NAME, sizeof(vdev_dec->name));
	vdev_dec->fops = &wave5_vpu_dec_fops;
	vdev_dec->ioctl_ops = &wave5_vpu_dec_ioctl_ops;
	vdev_dec->release = video_device_release_empty;
	vdev_dec->v4l2_dev = &dev->v4l2_dev;
	vdev_dec->vfl_dir = VFL_DIR_M2M;
	vdev_dec->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	vdev_dec->lock = &dev->dev_lock;
	video_set_drvdata(vdev_dec, dev);

	ret = video_register_device(vdev_dec, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;

	return 0;
}

void wave5_vpu_dec_unregister_device(struct vpu_device *dev)
{
	video_unregister_device(dev->video_dev_dec);
	if (dev->v4l2_m2m_dec_dev)
		v4l2_m2m_release(dev->v4l2_m2m_dec_dev);
}
