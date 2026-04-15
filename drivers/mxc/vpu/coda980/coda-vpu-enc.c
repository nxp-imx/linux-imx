// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Coda series multi-standard codec IP - encoder interface
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#include "coda-helper.h"

#define VPU_ENC_DEV_NAME "C&M Coda VPU encoder"
#define VPU_ENC_DRV_NAME "coda-enc"

static const struct v4l2_frmsize_stepwise enc_frmsize[VPU_FMT_TYPES] = {
	[VPU_FMT_TYPE_CODEC] = {
		.min_width = CODA_ENC_MIN_PIC_WIDTH,
		.max_width = CODA_ENC_MAX_PIC_WIDTH,
		.step_width = CODA_ENC_CODEC_PIC_STEP,
		.min_height = CODA_ENC_MIN_PIC_HEIGHT,
		.max_height = CODA_ENC_MAX_PIC_HEIGHT,
		.step_height = CODA_ENC_CODEC_PIC_STEP,
	},
	[VPU_FMT_TYPE_RAW] = {
		.min_width = CODA_ENC_MIN_PIC_WIDTH,
		.max_width = CODA_ENC_MAX_PIC_WIDTH,
		.step_width = CODA_ENC_RAW_PIC_STEP,
		.min_height = CODA_ENC_MIN_PIC_HEIGHT,
		.max_height = CODA_ENC_MAX_PIC_HEIGHT,
		.step_height = CODA_ENC_RAW_PIC_STEP,
	},
};

static const struct vpu_format enc_fmt_list[VPU_FMT_TYPES][MAX_VPU_FMTS] = {
	[VPU_FMT_TYPE_CODEC] = {
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_H264,
			.v4l2_frmsize = &enc_frmsize[VPU_FMT_TYPE_CODEC],
		},
	},
	[VPU_FMT_TYPE_RAW] = {
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_YUV420,
			.v4l2_frmsize = &enc_frmsize[VPU_FMT_TYPE_RAW],
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV12,
			.v4l2_frmsize = &enc_frmsize[VPU_FMT_TYPE_RAW],
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV21,
			.v4l2_frmsize = &enc_frmsize[VPU_FMT_TYPE_RAW],
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_YUV420M,
			.v4l2_frmsize = &enc_frmsize[VPU_FMT_TYPE_RAW],
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV12M,
			.v4l2_frmsize = &enc_frmsize[VPU_FMT_TYPE_RAW],
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV21M,
			.v4l2_frmsize = &enc_frmsize[VPU_FMT_TYPE_RAW],
		},
	}
};

static enum coda_std coda_vpu_enc_to_coda_std(unsigned int v4l2_pix_fmt)
{
	switch (v4l2_pix_fmt) {
	case V4L2_PIX_FMT_H264:
		return AVC_ENC;
	default:
		return STD_UNKNOWN;
	}
}

static const struct vpu_format *coda_vpu_enc_find_vpu_fmt(unsigned int v4l2_pix_fmt,
							  enum vpu_fmt_type type)
{
	unsigned int index;

	for (index = 0; index < MAX_VPU_FMTS; index++) {
		if (enc_fmt_list[type][index].v4l2_pix_fmt == v4l2_pix_fmt)
			return &enc_fmt_list[type][index];
	}

	return NULL;
}

static const struct vpu_format *coda_vpu_find_vpu_fmt_by_idx(unsigned int idx,
							     enum vpu_fmt_type type)
{
	if (idx >= MAX_VPU_FMTS)
		return NULL;

	if (!enc_fmt_list[type][idx].v4l2_pix_fmt)
		return NULL;

	return &enc_fmt_list[type][idx];
}

static u32 coda_vpu_enc_bitrate_bps_to_kbps(u32 bitrate_bps)
{
	return (bitrate_bps / 1000);
}

static u32 coda_vpu_enc_cpb_size_kb_to_msec(u32 cpb_size_kb, u32 bitrate)
{
	u64 cpb_size_bit;
	u64 cpb_size_msec;

	cpb_size_bit = (u64)cpb_size_kb * 1000 * BITS_PER_BYTE;
	cpb_size_msec = (cpb_size_bit * 1000) / bitrate;

	if (cpb_size_msec < 200 || cpb_size_msec > 32767)
		cpb_size_msec = 2000;

	return cpb_size_msec;
}

static void coda_vpu_enc_release_fb(struct vpu_instance *inst)
{
	int i;

	for (i = 0; i < CODA_MAX_FBS; i++) {
		coda_vdi_free_dma_memory(&inst->frame_vbuf[i]);
		memset(&inst->frame_buf[i], 0, sizeof(struct coda_frame_buffer));
	}
}

static void coda_vpu_enc_destroy_instance(struct vpu_instance *inst)
{
	u32 fail_res;
	int ret;

	ret = coda_vpuapi_enc_close(inst, &fail_res);
	if (ret) {
		dev_err(inst->vpu_dev->dev, "failed destroy instance: %d (%d)\n",
			ret, fail_res);
	}

	coda_vpu_enc_release_fb(inst);
	coda_vdi_free_dma_memory(&inst->work_vbuf);
	coda_vdi_free_dma_memory(&inst->report_vbuf);

	coda_vpu_set_instance_state(inst, VPU_INST_STATE_NONE);

	pm_runtime_put_sync(inst->vpu_dev->dev);
}

static void coda_vpu_enc_handle_last_frame(struct vpu_instance *inst, dma_addr_t addr)
{
	struct vb2_v4l2_buffer *dst_buf;

	dst_buf = coda_vpu_get_dst_buf_by_addr(inst, addr);
	if (!dst_buf)
		return;

	vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
	dst_buf->field = V4L2_FIELD_NONE;
	dst_buf->flags |= V4L2_BUF_FLAG_LAST;
	v4l2_m2m_dst_buf_remove_by_buf(inst->v4l2_fh.m2m_ctx, dst_buf);
	v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);

	coda_vpu_set_instance_state(inst, VPU_INST_STATE_PIC_RUN);

	dev_dbg(inst->vpu_dev->dev, "[%d] eos\n", inst->id);
	inst->eos = true;

	v4l2_m2m_set_src_buffered(inst->v4l2_fh.m2m_ctx, false);
}

static void coda_update_crop_info(struct vpu_instance *inst,
				  u32 left, u32 top, u32 width, u32 height)
{
	u32 enc_pic_width, enc_pic_height;

	inst->crop.left = left;
	inst->crop.top = top;
	inst->crop.width = width;
	inst->crop.height = height;

	inst->codec_rect.left = round_down(left, CODA_ENC_CROP_X_POS_STEP);
	inst->codec_rect.top = round_down(top, CODA_ENC_CROP_Y_POS_STEP);

	enc_pic_width = width + left - inst->codec_rect.left;
	inst->codec_rect.width = round_up(enc_pic_width, CODA_ENC_CODEC_PIC_STEP);

	enc_pic_height = height + top - inst->codec_rect.top;
	inst->codec_rect.height = round_up(enc_pic_height, CODA_ENC_CODEC_PIC_STEP);
}

static void coda_update_frame_buf_addr(struct vpu_instance *inst,
				       struct coda_frame_buffer *frame_buf)
{
	const struct v4l2_format_info *fmt_info;
	u32 stride = inst->src_fmt.plane_fmt[0].bytesperline;
	u32 offset;

	fmt_info = v4l2_format_info(inst->src_fmt.pixelformat);
	if (!fmt_info)
		return;

	offset = inst->codec_rect.top * stride + inst->codec_rect.left * fmt_info->bpp[0];
	frame_buf->buf_y += offset;

	stride = DIV_ROUND_UP(stride, fmt_info->bpp[0]) * fmt_info->bpp[1];
	offset = inst->codec_rect.top * stride / fmt_info->vdiv / fmt_info->hdiv
			+ inst->codec_rect.left * fmt_info->bpp[1] / fmt_info->hdiv;
	frame_buf->buf_cb += offset;
	frame_buf->buf_cr += offset;
}

static int coda_vpu_enc_start_encode(struct vpu_instance *inst)
{
	int ret = -EINVAL;
	struct vb2_v4l2_buffer *src_buf = NULL;
	struct vb2_v4l2_buffer *dst_buf = NULL;
	struct vpu_buffer *src_vbuf = NULL;
	struct vpu_buffer *dst_vbuf = NULL;
	struct coda_frame_buffer frame_buf;
	struct coda_enc_param pic_param;
	u32 stride = inst->src_fmt.plane_fmt[0].bytesperline;
	u32 luma_size = (stride * inst->src_fmt.height);
	u32 chroma_size;
	u32 fail_res;

	memset(&pic_param, 0, sizeof(struct coda_enc_param));
	memset(&frame_buf, 0, sizeof(struct coda_frame_buffer));

	if (inst->src_fmt.pixelformat == V4L2_PIX_FMT_YUV420 ||
	    inst->src_fmt.pixelformat == V4L2_PIX_FMT_YUV420M)
		chroma_size = ((stride / 2) * (inst->src_fmt.height / 2));
	else
		chroma_size = 0;

	src_buf = coda_vpu_get_valid_src_buf(inst);
	dst_buf = coda_vpu_get_valid_dst_buf(inst);

	if (!dst_buf) {
		dev_dbg(inst->vpu_dev->dev, "no valid dst buf\n");
		goto exit;
	}

	dst_vbuf = coda_to_vpu_buf(dst_buf);
	pic_param.bitstream_buf = coda_vpu_get_dma_addr(dst_buf, 0);
	pic_param.bitstream_buf_size = vb2_plane_size(&dst_buf->vb2_buf, 0);
	if (!src_buf) {
		dev_dbg(inst->vpu_dev->dev, "no valid src buf\n");
		if (inst->state == VPU_INST_STATE_STOP)
			coda_vpu_enc_handle_last_frame(inst, pic_param.bitstream_buf);

		goto exit;
	} else {
		src_vbuf = coda_to_vpu_buf(src_buf);
		if (inst->src_fmt.num_planes == 1) {
			frame_buf.buf_y = coda_vpu_get_dma_addr(src_buf, 0);
			frame_buf.buf_cb = frame_buf.buf_y + luma_size;
			frame_buf.buf_cr = frame_buf.buf_cb + chroma_size;
		} else if (inst->src_fmt.num_planes == 2) {
			frame_buf.buf_y = coda_vpu_get_dma_addr(src_buf, 0);
			frame_buf.buf_cb = coda_vpu_get_dma_addr(src_buf, 1);
			frame_buf.buf_cr = frame_buf.buf_cb + chroma_size;
		} else if (inst->src_fmt.num_planes == 3) {
			frame_buf.buf_y = coda_vpu_get_dma_addr(src_buf, 0);
			frame_buf.buf_cb = coda_vpu_get_dma_addr(src_buf, 1);
			frame_buf.buf_cr = coda_vpu_get_dma_addr(src_buf, 2);
		}
		coda_update_frame_buf_addr(inst, &frame_buf);
		frame_buf.stride = stride;
		frame_buf.index = src_buf->vb2_buf.index;
		pic_param.src_idx = src_buf->vb2_buf.index;
		pic_param.bitrate = coda_vpu_enc_bitrate_bps_to_kbps(inst->enc_ctrls.bitrate);
		pic_param.quant_param = inst->enc_ctrls.h264_p_frame_qp;
		if (src_buf->sequence == 0) {
			pic_param.force_i_picture = true;
			pic_param.quant_param = inst->enc_ctrls.h264_i_frame_qp;
		} else if ((inst->enc_ctrls.gop_size != 0) &&
			   (src_buf->sequence % inst->enc_ctrls.gop_size) == 0) {
			pic_param.force_i_picture = true;
			pic_param.quant_param = inst->enc_ctrls.h264_i_frame_qp;
		} else if (inst->enc_ctrls.force_key_frame) {
			pic_param.force_i_picture = true;
			pic_param.quant_param = inst->enc_ctrls.h264_i_frame_qp;
			inst->enc_ctrls.force_key_frame = 0;
		}
	}

	pic_param.source_frame = &frame_buf;

	if (src_vbuf)
		src_vbuf->consumed = true;
	if (dst_vbuf)
		dst_vbuf->consumed = true;

	inst->ts_start = ktime_get_raw();
	ret = coda_vpuapi_enc_start_one_frame(inst, &pic_param, &fail_res);
	if (ret) {
		dev_err(inst->vpu_dev->dev, "[%d] %s: fail %d\n", inst->id, __func__, ret);
		coda_vpu_set_instance_state(inst, VPU_INST_STATE_STOP);

		dst_buf = v4l2_m2m_dst_buf_remove(inst->v4l2_fh.m2m_ctx);
		if (dst_buf)
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);

		src_buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx);
		if (src_buf)
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
	} else {
		dev_dbg(inst->vpu_dev->dev, "%s: success\n", __func__);
	}

exit:
	return ret;
}

static void coda_vpu_enc_handle_encoded_frame(struct vpu_instance *inst,
					      struct coda_enc_output_info *info)
{
	struct vb2_v4l2_buffer *src_buf;
	struct vb2_v4l2_buffer *dst_buf;
	struct vpu_buffer *src_vpu_buf;
	struct vpu_buffer *dst_vpu_buf;
	enum vb2_buffer_state state;

	state = info->encoding_success ? VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR;

	src_buf = v4l2_m2m_src_buf_remove_by_idx(inst->v4l2_fh.m2m_ctx,
						 info->enc_src_idx);
	if (!src_buf) {
		dev_err(inst->vpu_dev->dev, "[%d] encoder can't find src buffer\n", inst->id);
		return;
	}

	src_vpu_buf = coda_to_vpu_buf(src_buf);
	if (!src_vpu_buf || !src_vpu_buf->consumed) {
		dev_err(inst->vpu_dev->dev, "[%d] src buffer is not consumed\n", inst->id);
		return;
	}

	dst_buf = coda_vpu_get_dst_buf_by_addr(inst, info->bitstream_buf);
	if (!dst_buf) {
		dev_err(inst->vpu_dev->dev, "[%d] encoder can't find dst buffer\n", inst->id);
		return;
	}

	inst->ts_finish = ktime_get_raw();
	inst->total_sw_time += (inst->ts_finish - inst->ts_start);
	inst->processed_buf_num++;

	dst_vpu_buf = coda_to_vpu_buf(dst_buf);
	if (dst_vpu_buf)
		dst_vpu_buf->average_qp = info->avg_ctu_qp;

	v4l2_m2m_buf_copy_metadata(src_buf, dst_buf, true);
	v4l2_m2m_buf_done(src_buf, state);

	vb2_set_plane_payload(&dst_buf->vb2_buf, 0, info->bitstream_buf_size);
	dst_buf->field = V4L2_FIELD_NONE;
	if (info->pic_type == PIC_TYPE_I)
		dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;
	else if (info->pic_type == PIC_TYPE_P)
		dst_buf->flags |= V4L2_BUF_FLAG_PFRAME;
	else if (info->pic_type == PIC_TYPE_B)
		dst_buf->flags |= V4L2_BUF_FLAG_BFRAME;

	v4l2_m2m_dst_buf_remove_by_buf(inst->v4l2_fh.m2m_ctx, dst_buf);
	if (state == VB2_BUF_STATE_ERROR)
		dev_err(inst->vpu_dev->dev, "[%d] error frame\n", inst->id);

	v4l2_m2m_buf_done(dst_buf, state);
}

static void coda_vpu_enc_finish_encode(struct vpu_instance *inst)
{
	int ret;
	struct coda_enc_output_info info;

	ret = coda_vpuapi_enc_get_output_info(inst, &info);
	if (ret) {
		dev_err(inst->vpu_dev->dev, "vpu_enc_get_output_info fail %d\n", ret);
		goto finish_encode;
	}

	dev_dbg(inst->vpu_dev->dev, "[%d] src_idx %d recon_idx %d rd_ptr 0x%pad wr_ptr 0x%pad\n",
		inst->id, info.enc_src_idx, info.recon_frame_index,
		&info.rd_ptr, &info.wr_ptr);

	if (info.enc_src_idx >= 0 && info.recon_frame_index >= 0)
		coda_vpu_enc_handle_encoded_frame(inst, &info);

finish_encode:
	cancel_delayed_work(&inst->vpu_dev->task_timer);
	v4l2_m2m_job_finish(inst->vpu_dev->m2m_dev, inst->v4l2_fh.m2m_ctx);
}

static void coda_vpu_enc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vpu_instance *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vpu_buffer *vpu_buf = coda_to_vpu_buf(vbuf);

	dev_dbg(inst->vpu_dev->dev, "%s: type: %4u idx: %4u size: ([0]=%4lu, [1]=%4lu, [2]=%4lu)\n",
		__func__, vb->type, vb->index, vb2_plane_size(&vbuf->vb2_buf, 0),
		vb2_plane_size(&vbuf->vb2_buf, 1), vb2_plane_size(&vbuf->vb2_buf, 2));

	if (V4L2_TYPE_IS_OUTPUT(vb->type))
		vbuf->sequence = inst->queued_src_buf_num++;
	else
		vbuf->sequence = inst->queued_dst_buf_num++;

	vpu_buf->consumed = false;
	vpu_buf->average_qp = 0;
	v4l2_m2m_buf_queue(inst->v4l2_fh.m2m_ctx, vbuf);
}

static void coda_vpu_enc_buf_finish(struct vb2_buffer *vb)
{
	struct vpu_instance *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vpu_buffer *vpu_buf = coda_to_vpu_buf(vbuf);
	struct v4l2_ctrl *ctrl;

	if (V4L2_TYPE_IS_OUTPUT(vb->type))
		return;

	ctrl = v4l2_ctrl_find(inst->v4l2_fh.ctrl_handler, V4L2_CID_MPEG_VIDEO_AVERAGE_QP);
	if (ctrl)
		v4l2_ctrl_s_ctrl(ctrl, vpu_buf->average_qp);
}

static int coda_vpu_enc_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
				    unsigned int *num_planes, unsigned int sizes[],
				    struct device *alloc_devs[])
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_pix_format_mplane inst_format =
		(V4L2_TYPE_IS_OUTPUT(q->type)) ? inst->src_fmt : inst->dst_fmt;
	unsigned int i;

	dev_dbg(inst->vpu_dev->dev, "%s: num_buffers %d num_planes %d type %d\n",
		__func__, *num_buffers, *num_planes, q->type);

	if (*num_planes) {
		if (inst_format.num_planes != *num_planes)
			return -EINVAL;

		for (i = 0; i < *num_planes; i++) {
			if (sizes[i] < inst_format.plane_fmt[i].sizeimage)
				return -EINVAL;
		}
	} else {
		*num_planes = inst_format.num_planes;
		for (i = 0; i < *num_planes; i++) {
			sizes[i] = inst_format.plane_fmt[i].sizeimage;
			dev_dbg(inst->vpu_dev->dev, "size[%d] : %d\n", i, sizes[i]);
		}
	}

	return 0;
}

static int coda_vpu_enc_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strscpy(cap->driver, VPU_ENC_DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, VPU_ENC_DRV_NAME, sizeof(cap->card));
	strscpy(cap->bus_info, "platform:" VPU_ENC_DRV_NAME, sizeof(cap->bus_info));

	return 0;
}

static int coda_vpu_enc_enum_framesizes(struct file *f, void *fh, struct v4l2_frmsizeenum *fsize)
{
	const struct vpu_format *vpu_fmt;

	if (fsize->index)
		return -EINVAL;

	vpu_fmt = coda_vpu_enc_find_vpu_fmt(fsize->pixel_format, VPU_FMT_TYPE_CODEC);
	if (!vpu_fmt) {
		vpu_fmt = coda_vpu_enc_find_vpu_fmt(fsize->pixel_format, VPU_FMT_TYPE_RAW);
		if (!vpu_fmt)
			return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise = *vpu_fmt->v4l2_frmsize;

	return 0;
}

static int coda_vpu_enc_enum_fmt_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	const struct vpu_format *vpu_fmt;

	vpu_fmt = coda_vpu_find_vpu_fmt_by_idx(f->index, VPU_FMT_TYPE_CODEC);
	if (!vpu_fmt)
		return -EINVAL;

	f->pixelformat = vpu_fmt->v4l2_pix_fmt;
	f->flags = 0;

	return 0;
}

static int coda_vpu_enc_try_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	const struct v4l2_frmsize_stepwise *frmsize;
	const struct vpu_format *vpu_fmt;
	int width, height;

	dev_dbg(inst->vpu_dev->dev, "%s: 4cc %d w %d h %d plane %d colorspace %d\n",
		__func__, pix_mp->pixelformat, pix_mp->width, pix_mp->height,
		pix_mp->num_planes, pix_mp->colorspace);

	if (!V4L2_TYPE_IS_CAPTURE(f->type))
		return -EINVAL;

	vpu_fmt = coda_vpu_enc_find_vpu_fmt(pix_mp->pixelformat, VPU_FMT_TYPE_CODEC);
	if (!vpu_fmt) {
		width = inst->dst_fmt.width;
		height = inst->dst_fmt.height;
		pix_mp->pixelformat = inst->dst_fmt.pixelformat;
		frmsize = &enc_frmsize[VPU_FMT_TYPE_CODEC];
	} else {
		width = pix_mp->width;
		height = pix_mp->height;
		pix_mp->pixelformat = vpu_fmt->v4l2_pix_fmt;
		frmsize = vpu_fmt->v4l2_frmsize;
	}

	coda_vpu_update_pix_fmt(pix_mp, VPU_FMT_TYPE_CODEC, width, height, frmsize);
	pix_mp->colorspace = inst->colorspace;
	pix_mp->ycbcr_enc = inst->ycbcr_enc;
	pix_mp->quantization = inst->quantization;
	pix_mp->xfer_func = inst->xfer_func;

	return 0;
}

static int coda_vpu_enc_s_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int i, ret;

	dev_dbg(inst->vpu_dev->dev, "%s: 4cc %d w %d h %d plane %d colorspace %d\n",
		__func__, pix_mp->pixelformat, pix_mp->width, pix_mp->height,
		pix_mp->num_planes, pix_mp->colorspace);

	ret = coda_vpu_enc_try_fmt_cap(file, fh, f);
	if (ret)
		return ret;

	inst->std = coda_vpu_enc_to_coda_std(inst->dst_fmt.pixelformat);
	if (inst->std == STD_UNKNOWN) {
		dev_err(inst->vpu_dev->dev, "unsupported pixelformat: %.4s\n",
			(char *)&inst->dst_fmt.pixelformat);
		return -EINVAL;
	}

	inst->dst_fmt.width = pix_mp->width;
	inst->dst_fmt.height = pix_mp->height;
	inst->dst_fmt.pixelformat = pix_mp->pixelformat;
	inst->dst_fmt.field = pix_mp->field;
	inst->dst_fmt.flags = pix_mp->flags;
	inst->dst_fmt.num_planes = pix_mp->num_planes;
	for (i = 0; i < inst->dst_fmt.num_planes; i++) {
		inst->dst_fmt.plane_fmt[i].bytesperline = pix_mp->plane_fmt[i].bytesperline;
		inst->dst_fmt.plane_fmt[i].sizeimage = pix_mp->plane_fmt[i].sizeimage;
	}

	return 0;
}

static int coda_vpu_enc_g_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int i;

	pix_mp->width = inst->dst_fmt.width;
	pix_mp->height = inst->dst_fmt.height;
	pix_mp->pixelformat = inst->dst_fmt.pixelformat;
	pix_mp->field = inst->dst_fmt.field;
	pix_mp->flags = inst->dst_fmt.flags;
	pix_mp->num_planes = inst->dst_fmt.num_planes;
	for (i = 0; i < pix_mp->num_planes; i++) {
		pix_mp->plane_fmt[i].bytesperline = inst->dst_fmt.plane_fmt[i].bytesperline;
		pix_mp->plane_fmt[i].sizeimage = inst->dst_fmt.plane_fmt[i].sizeimage;
	}

	pix_mp->colorspace = inst->colorspace;
	pix_mp->ycbcr_enc = inst->ycbcr_enc;
	pix_mp->quantization = inst->quantization;
	pix_mp->xfer_func = inst->xfer_func;

	return 0;
}

static int coda_vpu_enc_enum_fmt_out(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));
	const struct vpu_format *vpu_fmt;

	dev_dbg(inst->vpu_dev->dev, "%s: index %d\n", __func__, f->index);

	vpu_fmt = coda_vpu_find_vpu_fmt_by_idx(f->index, VPU_FMT_TYPE_RAW);
	if (!vpu_fmt)
		return -EINVAL;

	f->pixelformat = vpu_fmt->v4l2_pix_fmt;
	f->flags = 0;

	return 0;
}

static int coda_vpu_enc_try_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	const struct v4l2_frmsize_stepwise *frmsize;
	const struct vpu_format *vpu_fmt;
	int width, height;

	dev_dbg(inst->vpu_dev->dev, "%s: 4cc %d w %d h %d plane %d colorspace %d\n",
		__func__, pix_mp->pixelformat, pix_mp->width, pix_mp->height,
		pix_mp->num_planes, pix_mp->colorspace);

	if (!V4L2_TYPE_IS_OUTPUT(f->type))
		return -EINVAL;

	vpu_fmt = coda_vpu_enc_find_vpu_fmt(pix_mp->pixelformat, VPU_FMT_TYPE_RAW);
	if (!vpu_fmt) {
		width = inst->src_fmt.width;
		height = inst->src_fmt.height;
		pix_mp->pixelformat = inst->src_fmt.pixelformat;
		frmsize = &enc_frmsize[VPU_FMT_TYPE_RAW];
	} else {
		width = pix_mp->width;
		height = pix_mp->height;
		pix_mp->pixelformat = vpu_fmt->v4l2_pix_fmt;
		frmsize = vpu_fmt->v4l2_frmsize;
	}

	coda_vpu_update_pix_fmt(pix_mp, VPU_FMT_TYPE_RAW, width, height, frmsize);

	return 0;
}

static int coda_vpu_enc_s_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	const struct vpu_format *vpu_fmt;
	const struct v4l2_format_info *info;
	int i, ret;

	dev_dbg(inst->vpu_dev->dev, "%s: 4cc %d w %d h %d plane %d colorspace %d\n",
		__func__, pix_mp->pixelformat, pix_mp->width, pix_mp->height,
		pix_mp->num_planes, pix_mp->colorspace);

	ret = coda_vpu_enc_try_fmt_out(file, fh, f);
	if (ret)
		return ret;

	inst->src_fmt.width = pix_mp->width;
	inst->src_fmt.height = pix_mp->height;
	inst->src_fmt.pixelformat = pix_mp->pixelformat;
	inst->src_fmt.field = pix_mp->field;
	inst->src_fmt.flags = pix_mp->flags;
	inst->src_fmt.num_planes = pix_mp->num_planes;
	for (i = 0; i < inst->src_fmt.num_planes; i++) {
		inst->src_fmt.plane_fmt[i].bytesperline = pix_mp->plane_fmt[i].bytesperline;
		inst->src_fmt.plane_fmt[i].sizeimage = pix_mp->plane_fmt[i].sizeimage;
	}

	info = v4l2_format_info(inst->src_fmt.pixelformat);
	if (!info)
		return -EINVAL;

	inst->cbcr_interleave = (info->comp_planes == 2) ? true : false;

	switch (inst->src_fmt.pixelformat) {
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
		inst->nv21 = true;
		break;
	default:
		inst->nv21 = false;
	}

	inst->colorspace = pix_mp->colorspace;
	inst->ycbcr_enc = pix_mp->ycbcr_enc;
	inst->quantization = pix_mp->quantization;
	inst->xfer_func = pix_mp->xfer_func;

	vpu_fmt = coda_vpu_enc_find_vpu_fmt(inst->dst_fmt.pixelformat, VPU_FMT_TYPE_CODEC);
	if (!vpu_fmt)
		return -EINVAL;

	coda_vpu_update_pix_fmt(&inst->dst_fmt, VPU_FMT_TYPE_CODEC,
				pix_mp->width, pix_mp->height,
				vpu_fmt->v4l2_frmsize);
	coda_update_crop_info(inst, 0, 0, pix_mp->width, pix_mp->height);

	return 0;
}

static int coda_vpu_enc_g_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int i;

	pix_mp->width = inst->src_fmt.width;
	pix_mp->height = inst->src_fmt.height;
	pix_mp->pixelformat = inst->src_fmt.pixelformat;
	pix_mp->field = inst->src_fmt.field;
	pix_mp->flags = inst->src_fmt.flags;
	pix_mp->num_planes = inst->src_fmt.num_planes;
	for (i = 0; i < pix_mp->num_planes; i++) {
		pix_mp->plane_fmt[i].bytesperline = inst->src_fmt.plane_fmt[i].bytesperline;
		pix_mp->plane_fmt[i].sizeimage = inst->src_fmt.plane_fmt[i].sizeimage;
	}

	pix_mp->colorspace = inst->colorspace;
	pix_mp->ycbcr_enc = inst->ycbcr_enc;
	pix_mp->quantization = inst->quantization;
	pix_mp->xfer_func = inst->xfer_func;

	return 0;
}

static int coda_vpu_enc_g_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));

	if (!V4L2_TYPE_IS_OUTPUT(s->type))
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = inst->src_fmt.width;
		s->r.height = inst->src_fmt.height;
		break;
	case V4L2_SEL_TGT_CROP:
		s->r = inst->crop;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(inst->vpu_dev->dev, "%s: type %d target %d %dx%dx%dx%d\n",
		__func__, s->type, s->target,
		s->r.left, s->r.top, s->r.width, s->r.height);

	return 0;
}

static int coda_vpu_enc_s_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));
	const struct vpu_format *vpu_fmt;
	u32 max_crop_w, max_crop_h;

	if (!V4L2_TYPE_IS_OUTPUT(s->type))
		return -EINVAL;

	if (s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if (!(s->flags & (V4L2_SEL_FLAG_GE | V4L2_SEL_FLAG_LE)))
		s->flags |= V4L2_SEL_FLAG_LE;

	if (s->flags & V4L2_SEL_FLAG_GE) {
		s->r.left = round_up(s->r.left, CODA_ENC_CROP_STEP);
		s->r.top = round_up(s->r.top, CODA_ENC_CROP_STEP);
		s->r.width = round_up(s->r.width, CODA_ENC_CROP_STEP);
		s->r.height = round_up(s->r.height, CODA_ENC_CROP_STEP);
	}
	if (s->flags & V4L2_SEL_FLAG_LE) {
		s->r.left = round_down(s->r.left, CODA_ENC_CROP_STEP);
		s->r.top = round_down(s->r.top, CODA_ENC_CROP_STEP);
		s->r.width = round_down(s->r.width, CODA_ENC_CROP_STEP);
		s->r.height = round_down(s->r.height, CODA_ENC_CROP_STEP);
	}

	max_crop_w = inst->src_fmt.width - s->r.left;
	max_crop_h = inst->src_fmt.height - s->r.top;

	if (!s->r.width || !s->r.height)
		return 0;
	if (max_crop_w < CODA_ENC_MIN_PIC_WIDTH)
		return 0;
	if (max_crop_h < CODA_ENC_MIN_PIC_HEIGHT)
		return 0;

	s->r.width = clamp(s->r.width, CODA_ENC_MIN_PIC_WIDTH, max_crop_w);
	s->r.height = clamp(s->r.height, CODA_ENC_MIN_PIC_HEIGHT, max_crop_h);

	vpu_fmt = coda_vpu_enc_find_vpu_fmt(inst->dst_fmt.pixelformat, VPU_FMT_TYPE_CODEC);
	if (!vpu_fmt)
		return -EINVAL;

	coda_vpu_update_pix_fmt(&inst->dst_fmt, VPU_FMT_TYPE_CODEC,
				s->r.width, s->r.height,
				vpu_fmt->v4l2_frmsize);
	coda_update_crop_info(inst, s->r.left, s->r.top, s->r.width, s->r.height);

	dev_dbg(inst->vpu_dev->dev, "V4L2_SEL_TGT_CROP %dx%dx%dx%d\n",
		s->r.left, s->r.top, s->r.width, s->r.height);

	return 0;
}

static int coda_vpu_enc_encoder_cmd(struct file *file, void *fh, struct v4l2_encoder_cmd *ec)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));
	int ret;

	dev_dbg(inst->vpu_dev->dev, "[%d] %s: cmd %d\n", inst->id, __func__, ec->cmd);

	ret = v4l2_m2m_ioctl_try_encoder_cmd(file, fh, ec);
	if (ret)
		return ret;

	if (!coda_vpu_both_queues_are_streaming(inst))
		return 0;

	switch (ec->cmd) {
	case V4L2_ENC_CMD_STOP:
		coda_vpu_set_instance_state(inst, VPU_INST_STATE_STOP);
		v4l2_m2m_set_src_buffered(inst->v4l2_fh.m2m_ctx, true);
		v4l2_m2m_try_schedule(inst->v4l2_fh.m2m_ctx);
		break;
	case V4L2_ENC_CMD_START:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int coda_vpu_enc_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));

	dev_dbg(inst->vpu_dev->dev, "%s: type %d\n", __func__, a->type);

	if (!V4L2_TYPE_IS_OUTPUT(a->type))
		return -EINVAL;

	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.output.timeperframe.numerator = 1;
	a->parm.output.timeperframe.denominator = inst->framerate;

	dev_dbg(inst->vpu_dev->dev, "%s: numerator : %d | denominator : %d\n",
		__func__,
		a->parm.output.timeperframe.numerator,
		a->parm.output.timeperframe.denominator);

	return 0;
}

static int coda_vpu_enc_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(file));

	dev_dbg(inst->vpu_dev->dev, "%s: type %d\n", __func__, a->type);

	if (!V4L2_TYPE_IS_OUTPUT(a->type))
		return -EINVAL;

	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	if (a->parm.output.timeperframe.denominator && a->parm.output.timeperframe.numerator) {
		inst->framerate = a->parm.output.timeperframe.denominator /
				  a->parm.output.timeperframe.numerator;
	} else {
		a->parm.output.timeperframe.numerator = 1;
		a->parm.output.timeperframe.denominator = inst->framerate;
	}

	dev_dbg(inst->vpu_dev->dev, "%s: numerator : %d | denominator : %d\n",
		__func__,
		a->parm.output.timeperframe.numerator,
		a->parm.output.timeperframe.denominator);

	return 0;
}

static bool to_video_full_range_flag(enum v4l2_quantization quantization)
{
	switch (quantization) {
	case V4L2_QUANTIZATION_FULL_RANGE:
		return true;
	case V4L2_QUANTIZATION_LIM_RANGE:
	default:
		return false;
	}
}

static u8 to_colour_primaries(enum v4l2_colorspace colorspace)
{
	switch (colorspace) {
	case V4L2_COLORSPACE_SMPTE170M:
		return 6;
	case V4L2_COLORSPACE_REC709:
	case V4L2_COLORSPACE_SRGB:
	case V4L2_COLORSPACE_JPEG:
		return 1;
	case V4L2_COLORSPACE_BT2020:
		return 9;
	case V4L2_COLORSPACE_DCI_P3:
		return 11;
	case V4L2_COLORSPACE_SMPTE240M:
		return 7;
	case V4L2_COLORSPACE_470_SYSTEM_M:
		return 4;
	case V4L2_COLORSPACE_470_SYSTEM_BG:
		return 5;
	case V4L2_COLORSPACE_RAW:
	default:
		return 2;
	}
}

static u8 to_transfer_characteristics(enum v4l2_colorspace colorspace,
				      enum v4l2_xfer_func xfer_func)
{
	if (xfer_func == V4L2_XFER_FUNC_DEFAULT)
		xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(colorspace);

	switch (xfer_func) {
	case V4L2_XFER_FUNC_709:
		if (colorspace == V4L2_COLORSPACE_SMPTE170M)
			return 6;
		else if (colorspace == V4L2_COLORSPACE_BT2020)
			return 14;
		else
			return 1;
	case V4L2_XFER_FUNC_SRGB:
		return 13;
	case V4L2_XFER_FUNC_SMPTE240M:
		return 7;
	case V4L2_XFER_FUNC_NONE:
		return 8;
	case V4L2_XFER_FUNC_SMPTE2084:
		return 16;
	case V4L2_XFER_FUNC_DCI_P3:
	default:
		return 2;
	}
}

static u8 to_matrix_coeffs(enum v4l2_colorspace colorspace,
			   enum v4l2_ycbcr_encoding ycbcr_enc)
{
	if (ycbcr_enc == V4L2_YCBCR_ENC_DEFAULT)
		ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(colorspace);

	switch (ycbcr_enc) {
	case V4L2_YCBCR_ENC_601:
	case V4L2_YCBCR_ENC_XV601:
		if (colorspace == V4L2_COLORSPACE_SMPTE170M)
			return 6;
		else
			return 5;
	case V4L2_YCBCR_ENC_709:
	case V4L2_YCBCR_ENC_XV709:
		return 1;
	case V4L2_YCBCR_ENC_BT2020:
		return 9;
	case V4L2_YCBCR_ENC_BT2020_CONST_LUM:
		return 10;
	case V4L2_YCBCR_ENC_SMPTE240M:
		return 7;
	default:
		return 2;
	}
}

static int coda_vpu_enc_set_open_param(struct coda_enc_open_param *open_param,
				       struct vpu_instance *inst)
{
	struct vpu_enc_controls *ctrls = &inst->enc_ctrls;
	u32 mb_width = DIV_ROUND_UP(inst->codec_rect.width, 16);
	u32 mb_height = DIV_ROUND_UP(inst->codec_rect.height, 16);

	open_param->map_type = LINEAR_FRAME_MAP;
	open_param->en_line_buf_int = DEFAULT_EN_LINE_BUFFER_INTERRUPT;
	open_param->en_field_encoding = DEFAULT_EN_FIELD_ENCODING;
	open_param->field_ref_mode = DEFAULT_FIELD_REFERENCE_MODE;
	open_param->intra_qp = DEFAULT_INTRA_QP;
	open_param->idr_interval = DEFAULT_IDR_INTERVAL;
	open_param->interval_mode = DEFAULT_INTERVAL_MODE;
	open_param->intra_cost_weight = DEFAULT_INTRA_COST_WEIGHT;
	open_param->en_consecutive_intra_refresh = DEFAULT_EN_CONSECUTIVE_INTRA_REFRESH;
	open_param->en_count_intra_mb = DEFAULT_EN_COUNT_INTRA_MB;
	open_param->sec_axi_base = inst->vpu_dev->sram_buf.daddr;
	open_param->sec_axi_size = inst->vpu_dev->sram_buf.size;
	open_param->pic_width = inst->codec_rect.width;
	open_param->pic_height = inst->codec_rect.height;
	open_param->conf_win.left = inst->crop.left - inst->codec_rect.left;
	open_param->conf_win.top = inst->crop.top - inst->codec_rect.top;
	open_param->conf_win.right = inst->codec_rect.width
					- inst->crop.width - open_param->conf_win.left;
	open_param->conf_win.bottom = inst->codec_rect.height
					- inst->crop.height - open_param->conf_win.top;
	open_param->framerate = inst->framerate;
	open_param->mir_dir = ctrls->flip;
	open_param->rot_ang = ctrls->rotate;
	open_param->profile = ctrls->h264_profile;
	open_param->level = ctrls->h264_level;
	open_param->gop_size = ctrls->gop_size;
	if (ctrls->frame_rc_enable) {
		if (ctrls->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CBR)
			open_param->rate_control_type = RATE_CONTROL_TYPE_CBR;
		else if (ctrls->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR)
			open_param->rate_control_type = RATE_CONTROL_TYPE_ABR;
	}
	open_param->mb_interval = (ctrls->mb_rc_enable) ?
		DIV_ROUND_UP(mb_width, 8) : mb_width * mb_height;
	open_param->initial_delay = coda_vpu_enc_cpb_size_kb_to_msec(ctrls->h264_cpb_size,
								     ctrls->bitrate);
	open_param->min_qp = ctrls->h264_min_qp;
	open_param->max_qp = ctrls->h264_max_qp;
	open_param->bitrate = coda_vpu_enc_bitrate_bps_to_kbps(ctrls->bitrate);
	if (ctrls->h264_profile >= H264_PROFILE_MP)
		open_param->en_cabac_mode = ctrls->h264_entropy_mode;
	if (ctrls->h264_profile >= H264_PROFILE_HP)
		open_param->en_transform_8x8 = ctrls->h264_8x8_transform;
	open_param->en_constrained_intra_pred = ctrls->h264_constrained_intra_prediction;
	open_param->chroma_qp_offset = ctrls->h264_chroma_qp_index_offset;
	open_param->deblk_filter_idc = ctrls->h264_loop_filter_mode;
	open_param->deblk_filter_offset_alpha = ctrls->h264_loop_filter_alpha;
	open_param->deblk_filter_offset_beta = ctrls->h264_loop_filter_beta;
	if (ctrls->multi_slice_mode == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB) {
		open_param->slice_size_mode = SLICE_SIZE_MODE_MB;
		open_param->slice_size = ctrls->multi_slice_max_mb;
	}
	if (ctrls->intra_refresh_period)
		open_param->intra_refresh_mb_num = DIV_ROUND_UP(mb_width * mb_height,
								ctrls->intra_refresh_period);
	else
		open_param->intra_refresh_mb_num = ctrls->cyclic_intra_refresh_mb;
	open_param->en_frame_skip = (ctrls->frame_skip_mode) ? 1 : 0;
	open_param->me_search_range_x = ME_SEARCH_RANGE_X_16X16;
	if (ctrls->mv_h_search_range == 32)
		open_param->me_search_range_x = ME_SEARCH_RANGE_X_32X32;
	else if (ctrls->mv_h_search_range == 48)
		open_param->me_search_range_x = ME_SEARCH_RANGE_X_48X48;
	else if (ctrls->mv_h_search_range == 64)
		open_param->me_search_range_x = ME_SEARCH_RANGE_X_64X64;
	open_param->me_search_range_y = ME_SEARCH_RANGE_Y_16X16;
	if (ctrls->mv_v_search_range == 32)
		open_param->me_search_range_y = ME_SEARCH_RANGE_Y_32X32;
	else if (ctrls->mv_v_search_range == 48)
		open_param->me_search_range_y = ME_SEARCH_RANGE_Y_48X48;
	open_param->sar.enable = ctrls->h264_vui_sar_enable;
	open_param->sar.idc = ctrls->h264_vui_sar_idc;
	if (open_param->sar.idc == V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED)
		open_param->sar.idc = H264_VUI_SAR_IDC_EXTENDED;
	open_param->sar.width = ctrls->h264_vui_ext_sar_width;
	open_param->sar.height = ctrls->h264_vui_ext_sar_height;
	open_param->color.color_range = to_video_full_range_flag(inst->quantization);
	open_param->color.color_primaries = to_colour_primaries(inst->colorspace);
	open_param->color.transfer_characteristics = to_transfer_characteristics(inst->colorspace,
										 inst->xfer_func);
	open_param->color.matrix_coefficients = to_matrix_coeffs(inst->colorspace, inst->ycbcr_enc);

	return 0;
}

static int coda_vpu_enc_create_instance(struct vpu_instance *inst)
{
	int ret;
	struct coda_enc_open_param open_param;

	memset(&open_param, 0, sizeof(struct coda_enc_open_param));

	ret = pm_runtime_resume_and_get(inst->vpu_dev->dev);
	if (ret) {
		dev_err(inst->vpu_dev->dev, "runtime_resume failed %d\n", ret);
		return ret;
	}

	inst->work_vbuf.size = CODA_WORK_BUF_SIZE;
	ret = coda_vdi_allocate_dma_memory(inst->vpu_dev->dev, &inst->work_vbuf);
	if (ret) {
		dev_dbg(inst->vpu_dev->dev, "%s: allocate work buffer of size %zu fail: %d\n",
			__func__, inst->work_vbuf.size, ret);
		goto error_pm;
	}

	inst->report_vbuf.size = CODA_REPORT_BUF_SIZE_ADDR_INFO + CODA_REPORT_BUF_SIZE_MB_INFO +
				 CODA_REPORT_BUF_SIZE_MV_INFO + CODA_REPORT_BUF_SIZE_SLICE_INFO +
				 CODA_REPORT_BUF_SIZE_COST_INFO;
	ret = coda_vdi_allocate_dma_memory(inst->vpu_dev->dev, &inst->report_vbuf);
	if (ret) {
		dev_dbg(inst->vpu_dev->dev, "%s: allocate report buffer of size %zu fail: %d\n",
			__func__, inst->report_vbuf.size, ret);
		goto error_work_vbuf_free;
	}

	coda_vpu_enc_set_open_param(&open_param, inst);

	ret = coda_vpuapi_enc_open(inst, &open_param);
	if (ret) {
		dev_err(inst->vpu_dev->dev, "failed create instance : %d\n", ret);
		goto error_open;
	}

	coda_vpu_set_instance_state(inst, VPU_INST_STATE_OPEN);

	return 0;

error_open:
	coda_vdi_free_dma_memory(&inst->report_vbuf);
error_work_vbuf_free:
	coda_vdi_free_dma_memory(&inst->work_vbuf);
error_pm:
	pm_runtime_put_sync(inst->vpu_dev->dev);

	return ret;
}

static int coda_vpu_enc_initialize_instance(struct vpu_instance *inst)
{
	int ret;

	ret = coda_vpuapi_enc_issue_seq_init(inst);
	if (ret) {
		dev_err(inst->vpu_dev->dev, "seq init fail %d\n", ret);
		return ret;
	}

	coda_vpu_set_instance_state(inst, VPU_INST_STATE_INIT_SEQ);

	return 0;
}

static int coda_vpu_enc_prepare_fb(struct vpu_instance *inst)
{
	u32 fb_stride;
	u32 fb_height;
	int i, ret = 0;

	if (inst->enc_ctrls.rotate == ROT_ANG_90 || inst->enc_ctrls.rotate == ROT_ANG_270) {
		fb_stride = ALIGN(inst->codec_rect.height, 32);
		fb_height = ALIGN(inst->codec_rect.width, 32);
	} else {
		fb_stride = ALIGN(inst->codec_rect.width, 32);
		fb_height = ALIGN(inst->codec_rect.height, 32);
	}

	for (i = 0; i < CODA_MIN_FRAME_BUFFER_NUM; i++) {
		struct coda_frame_buffer *frame = &inst->frame_buf[i];
		struct vpu_buf *vframe = &inst->frame_vbuf[i];
		u32 luma_size = fb_stride * fb_height;
		u32 chroma_size = ALIGN(fb_stride / 2, 16) * fb_height;

		vframe->size = luma_size + chroma_size;
		ret = coda_vdi_allocate_dma_memory(inst->vpu_dev->dev, vframe);
		if (ret < 0) {
			dev_err(inst->vpu_dev->dev, "%s: failed to allocate FBC buffer %zu\n",
				__func__, vframe->size);
			goto free_buffers;
		}

		frame->buf_y = vframe->daddr;
		frame->buf_cb = vframe->daddr + luma_size;
		frame->buf_cr = vframe->daddr + luma_size + chroma_size / 2;
		frame->buf_y_bot = 0;
		frame->buf_cb_bot = 0;
		frame->buf_cr_bot = 0;
		frame->stride = fb_stride;
		frame->height = fb_height;
		frame->map_type = LINEAR_FRAME_MAP;
		frame->index = i;
	}

	ret = coda_vpuapi_enc_register_frame_buffer(inst, CODA_MIN_FRAME_BUFFER_NUM, fb_stride,
						    fb_height, LINEAR_FRAME_MAP);
	if (ret) {
		dev_err(inst->vpu_dev->dev, "%s: register_frame_buffer, fail: %d\n", __func__, ret);
		goto free_buffers;
	}

	coda_vpu_set_instance_state(inst, VPU_INST_STATE_PIC_RUN);

	return 0;

free_buffers:
	coda_vpu_enc_release_fb(inst);
	return ret;
}

static int coda_vpu_enc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_pix_format_mplane *fmt;
	struct vb2_queue *vq_peer;
	int ret = 0;

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		fmt = &inst->src_fmt;
		vq_peer = v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx);
	} else {
		fmt = &inst->dst_fmt;
		vq_peer = v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx);
	}

	dev_dbg(inst->vpu_dev->dev, "[%d] %s %c%c%c%c %dx%d, %d buffers\n",
		inst->id, V4L2_TYPE_IS_OUTPUT(q->type) ? "output" : "capture",
		fmt->pixelformat,
		fmt->pixelformat >> 8,
		fmt->pixelformat >> 16,
		fmt->pixelformat >> 24,
		fmt->width, fmt->height, vb2_get_num_buffers(q));

	if (!vb2_is_streaming(vq_peer))
		return 0;

	v4l2_m2m_suspend(inst->vpu_dev->m2m_dev);

	if (inst->state == VPU_INST_STATE_NONE) {
		ret = coda_vpu_enc_create_instance(inst);
		if (ret)
			goto exit;
	}

	if (inst->state == VPU_INST_STATE_OPEN) {
		ret = coda_vpu_enc_initialize_instance(inst);
		if (ret) {
			coda_vpu_enc_destroy_instance(inst);
			goto exit;
		}
	}

	if (inst->state == VPU_INST_STATE_INIT_SEQ) {
		ret = coda_vpu_enc_prepare_fb(inst);
		if (ret) {
			coda_vpu_enc_destroy_instance(inst);
			goto exit;
		}
	}

exit:
	v4l2_m2m_resume(inst->vpu_dev->m2m_dev);
	if (ret)
		coda_vpu_return_buffers(inst, q->type, VB2_BUF_STATE_QUEUED);

	return ret;
}

static void coda_vpu_enc_stop_streaming(struct vb2_queue *q)
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct vb2_queue *vq_peer;

	dev_dbg(inst->vpu_dev->dev, "[%d] %s: type %d\n", inst->id, __func__, q->type);

	if (coda_vpu_both_queues_are_streaming(inst))
		coda_vpu_set_instance_state(inst, VPU_INST_STATE_STOP);

	v4l2_m2m_suspend(inst->vpu_dev->m2m_dev);

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		inst->queued_src_buf_num = 0;
		v4l2_m2m_set_src_buffered(inst->v4l2_fh.m2m_ctx, false);

		if (inst->processed_buf_num) {
			u64 temp = inst->processed_buf_num * NSEC_PER_SEC;
			u64 fps = DIV_ROUND_CLOSEST(temp, inst->total_sw_time);

			dev_info(inst->vpu_dev->dev, "[%d] fps sw: %lld\n", inst->id, fps);
		}
		inst->processed_buf_num = 0;
		inst->total_sw_time = 0;
	} else {
		inst->eos = false;
		inst->queued_dst_buf_num = 0;
	}

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		vq_peer = v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx);
	else
		vq_peer = v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx);

	if (!vb2_is_streaming(vq_peer) && inst->state != VPU_INST_STATE_NONE)
		coda_vpu_enc_destroy_instance(inst);

	v4l2_m2m_resume(inst->vpu_dev->m2m_dev);

	coda_vpu_return_buffers(inst, q->type, VB2_BUF_STATE_ERROR);
}

static const struct v4l2_ioctl_ops coda_vpu_enc_ioctl_ops = {
	.vidioc_querycap = coda_vpu_enc_querycap,
	.vidioc_enum_framesizes = coda_vpu_enc_enum_framesizes,

	.vidioc_enum_fmt_vid_cap = coda_vpu_enc_enum_fmt_cap,
	.vidioc_s_fmt_vid_cap_mplane = coda_vpu_enc_s_fmt_cap,
	.vidioc_g_fmt_vid_cap_mplane = coda_vpu_enc_g_fmt_cap,
	.vidioc_try_fmt_vid_cap_mplane = coda_vpu_enc_try_fmt_cap,

	.vidioc_enum_fmt_vid_out = coda_vpu_enc_enum_fmt_out,
	.vidioc_s_fmt_vid_out_mplane = coda_vpu_enc_s_fmt_out,
	.vidioc_g_fmt_vid_out_mplane = coda_vpu_enc_g_fmt_out,
	.vidioc_try_fmt_vid_out_mplane = coda_vpu_enc_try_fmt_out,

	.vidioc_g_selection = coda_vpu_enc_g_selection,
	.vidioc_s_selection = coda_vpu_enc_s_selection,

	.vidioc_g_parm = coda_vpu_enc_g_parm,
	.vidioc_s_parm = coda_vpu_enc_s_parm,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_try_encoder_cmd = v4l2_m2m_ioctl_try_encoder_cmd,
	.vidioc_encoder_cmd = coda_vpu_enc_encoder_cmd,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct vb2_ops coda_vpu_enc_vb2_ops = {
	.queue_setup = coda_vpu_enc_queue_setup,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.buf_queue = coda_vpu_enc_buf_queue,
	.buf_finish = coda_vpu_enc_buf_finish,
	.start_streaming = coda_vpu_enc_start_streaming,
	.stop_streaming = coda_vpu_enc_stop_streaming,
};

static void coda_vpu_enc_set_default_format(struct v4l2_pix_format_mplane *src_fmt,
					    struct v4l2_pix_format_mplane *dst_fmt)
{
	const struct vpu_format *vpu_fmt;

	vpu_fmt = coda_vpu_find_vpu_fmt_by_idx(0, VPU_FMT_TYPE_RAW);
	if (vpu_fmt) {
		src_fmt->pixelformat = vpu_fmt->v4l2_pix_fmt;
		coda_vpu_update_pix_fmt(src_fmt, VPU_FMT_TYPE_RAW,
					CODA_ENC_DEF_PIC_WIDTH, CODA_ENC_DEF_PIC_HEIGHT,
					vpu_fmt->v4l2_frmsize);
	}

	vpu_fmt = coda_vpu_find_vpu_fmt_by_idx(0, VPU_FMT_TYPE_CODEC);
	if (vpu_fmt) {
		dst_fmt->pixelformat = vpu_fmt->v4l2_pix_fmt;
		coda_vpu_update_pix_fmt(dst_fmt, VPU_FMT_TYPE_CODEC,
					CODA_ENC_DEF_PIC_WIDTH, CODA_ENC_DEF_PIC_HEIGHT,
					vpu_fmt->v4l2_frmsize);
	}
}

static int coda_vpu_enc_queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct vpu_instance *inst = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->ops = &coda_vpu_enc_vb2_ops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->buf_struct_size = sizeof(struct vpu_buffer);
	src_vq->drv_priv = inst;
	src_vq->lock = &inst->vpu_dev->dev_lock;
	src_vq->dev = inst->vpu_dev->v4l2_dev.dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->ops = &coda_vpu_enc_vb2_ops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->buf_struct_size = sizeof(struct vpu_buffer);
	dst_vq->drv_priv = inst;
	dst_vq->lock = &inst->vpu_dev->dev_lock;
	dst_vq->dev = inst->vpu_dev->v4l2_dev.dev;
	ret = vb2_queue_init(dst_vq);
	if (ret)
		return ret;

	return 0;
}

static void coda_vpu_enc_stop_encode(struct vpu_instance *inst)
{
	dev_dbg(inst->vpu_dev->dev, "%s: state %d\n", __func__, inst->state);

	coda_vpu_set_instance_state(inst, VPU_INST_STATE_STOP);
}

static const struct vpu_instance_ops coda_vpu_enc_inst_ops = {
	.start_process = coda_vpu_enc_start_encode,
	.stop_process = coda_vpu_enc_stop_encode,
	.finish_process = coda_vpu_enc_finish_encode,
};

static int coda_vpu_enc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vpu_instance *inst = coda_ctrl_to_vpu_inst(ctrl);
	struct vpu_enc_controls *ctrls = &inst->enc_ctrls;

	dev_dbg(inst->vpu_dev->dev, "%s: name %s value %d\n", __func__, ctrl->name, ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		ctrls->flip |= (ctrl->val << 1);
		break;
	case V4L2_CID_VFLIP:
		ctrls->flip |= ctrl->val;
		break;
	case V4L2_CID_ROTATE:
		ctrls->rotate = ctrl->val;
		break;
	case V4L2_CID_MIN_BUFFERS_FOR_OUTPUT:
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctrls->gop_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE:
		ctrls->multi_slice_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
		ctrls->multi_slice_max_mb = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB:
		ctrls->cyclic_intra_refresh_mb = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD:
		ctrls->intra_refresh_period = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MV_H_SEARCH_RANGE:
		ctrls->mv_h_search_range = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MV_V_SEARCH_RANGE:
		ctrls->mv_v_search_range = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		ctrls->bitrate_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ctrls->bitrate = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		ctrls->frame_rc_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE:
		ctrls->mb_rc_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_SKIP_MODE:
		ctrls->frame_skip_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		ctrls->force_key_frame = 1;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE:
		ctrls->h264_cpb_size = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		switch (ctrl->val) {
		case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
		case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
			ctrls->h264_profile = H264_PROFILE_BP;
			break;
		case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
			ctrls->h264_profile = H264_PROFILE_MP;
			break;
		case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
			ctrls->h264_profile = H264_PROFILE_HP;
			break;
		default:
			return -EINVAL;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		switch (ctrl->val) {
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_0:
		case V4L2_MPEG_VIDEO_H264_LEVEL_1B:
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_1:
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_2:
		case V4L2_MPEG_VIDEO_H264_LEVEL_1_3:
		case V4L2_MPEG_VIDEO_H264_LEVEL_2_0:
		case V4L2_MPEG_VIDEO_H264_LEVEL_2_1:
		case V4L2_MPEG_VIDEO_H264_LEVEL_2_2:
		case V4L2_MPEG_VIDEO_H264_LEVEL_3_0:
		case V4L2_MPEG_VIDEO_H264_LEVEL_3_1:
		case V4L2_MPEG_VIDEO_H264_LEVEL_3_2:
		case V4L2_MPEG_VIDEO_H264_LEVEL_4_0:
		case V4L2_MPEG_VIDEO_H264_LEVEL_4_1:
		case V4L2_MPEG_VIDEO_H264_LEVEL_4_2:
		case V4L2_MPEG_VIDEO_H264_LEVEL_5_0:
		case V4L2_MPEG_VIDEO_H264_LEVEL_5_1:
			ctrls->h264_level = ctrl->val;
			break;
		default:
			return -EINVAL;
		}
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		ctrls->h264_i_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		ctrls->h264_p_frame_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		ctrls->h264_min_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		ctrls->h264_max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE:
		ctrls->h264_loop_filter_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA:
		ctrls->h264_loop_filter_alpha = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA:
		ctrls->h264_loop_filter_beta = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM:
		ctrls->h264_8x8_transform = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_CONSTRAINED_INTRA_PREDICTION:
		ctrls->h264_constrained_intra_prediction = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_CHROMA_QP_INDEX_OFFSET:
		ctrls->h264_chroma_qp_index_offset = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE:
		ctrls->h264_entropy_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE:
		ctrls->h264_vui_sar_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC:
		ctrls->h264_vui_sar_idc = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH:
		ctrls->h264_vui_ext_sar_width = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT:
		ctrls->h264_vui_ext_sar_height = ctrl->val;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops coda_vpu_enc_ctrl_ops = {
	.s_ctrl = coda_vpu_enc_s_ctrl,
};

static int coda_vpu_enc_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_device *vpu = video_drvdata(filp);
	struct vpu_instance *inst = NULL;
	struct v4l2_ctrl_handler *v4l2_ctrl_hdl;
	int ret;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;
	v4l2_ctrl_hdl = &inst->v4l2_ctrl_hdl;

	inst->vpu_dev = vpu;
	inst->type = VPU_INST_TYPE_ENC;
	inst->ops = &coda_vpu_enc_inst_ops;

	v4l2_fh_init(&inst->v4l2_fh, vdev);
	v4l2_fh_add(&inst->v4l2_fh, filp);

	inst->v4l2_fh.m2m_ctx =
		v4l2_m2m_ctx_init(vpu->m2m_dev, inst, coda_vpu_enc_queue_init);
	if (IS_ERR(inst->v4l2_fh.m2m_ctx)) {
		ret = PTR_ERR(inst->v4l2_fh.m2m_ctx);
		goto free_inst;
	}

	v4l2_ctrl_handler_init(v4l2_ctrl_hdl, 50);
	v4l2_ctrl_new_std_menu(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			       V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
			       ~(BIT(V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE) |
				 BIT(V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE) |
				 BIT(V4L2_MPEG_VIDEO_H264_PROFILE_MAIN) |
				 BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH)),
			       V4L2_MPEG_VIDEO_H264_PROFILE_HIGH);
	v4l2_ctrl_new_std_menu(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_LEVEL,
			       V4L2_MPEG_VIDEO_H264_LEVEL_5_1, 0,
			       V4L2_MPEG_VIDEO_H264_LEVEL_5_0);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
			  0, 51, 1, 30);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
			  0, 51, 1, 30);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
			  0, 51, 1, 8);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
			  0, 51, 1, 51);
	v4l2_ctrl_new_std_menu(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE,
			       V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY, 0,
			       V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA,
			  -6, 6, 1, 0);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA,
			  -6, 6, 1, 0);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM,
			  0, 1, 1, 1);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_CONSTRAINED_INTRA_PREDICTION,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_CHROMA_QP_INDEX_OFFSET,
			  -12, 12, 1, 0);
	v4l2_ctrl_new_std_menu(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE,
			       V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC, 0,
			       V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_HFLIP,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_VFLIP,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_ROTATE,
			  0, 270, 90, 0);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE,
			  0, 1073704, 1, 0);
	v4l2_ctrl_new_std_menu(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR, 0,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_BITRATE,
			  1000, 262143000, 1000, 2048000);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE,
			  0, 1, 1, 1);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE,
			  0, 1, 1, 1);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_GOP_SIZE,
			  0, 1023, 1, 30);
	v4l2_ctrl_new_std_menu(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
			       V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB, 0,
			       V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB,
			  0, 0x3FFFFFFF, 1, 1);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB,
			  0, 36864, 1, 0);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_INTRA_REFRESH_PERIOD,
			  0, 144, 1, 0);
	v4l2_ctrl_new_std_menu(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_FRAME_SKIP_MODE,
			       V4L2_MPEG_VIDEO_FRAME_SKIP_MODE_BUF_LIMIT,
			       (1 << V4L2_MPEG_VIDEO_FRAME_SKIP_MODE_LEVEL_LIMIT),
			       V4L2_MPEG_VIDEO_FRAME_SKIP_MODE_DISABLED);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_MV_H_SEARCH_RANGE,
			  16, 64, 16, 16);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_MV_V_SEARCH_RANGE,
			  16, 48, 16, 16);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE, 0, 1, 1, 0);
	v4l2_ctrl_new_std_menu(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC,
			       V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED, 0,
			       V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH,
			  0, 0xFFFF, 1, 0);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT,
			  0, 0xFFFF, 1, 0);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, &coda_vpu_enc_ctrl_ops,
			  V4L2_CID_MIN_BUFFERS_FOR_OUTPUT, 1, 32, 1, 1);
	v4l2_ctrl_new_std(v4l2_ctrl_hdl, NULL,
			  V4L2_CID_MPEG_VIDEO_AVERAGE_QP, 0, 51, 1, 0);

	if (v4l2_ctrl_hdl->error) {
		ret = -ENODEV;
		goto err_m2m_release;
	}

	inst->v4l2_fh.ctrl_handler = v4l2_ctrl_hdl;
	v4l2_ctrl_handler_setup(v4l2_ctrl_hdl);

	coda_vpu_enc_set_default_format(&inst->src_fmt, &inst->dst_fmt);
	coda_update_crop_info(inst, 0, 0, inst->dst_fmt.width, inst->dst_fmt.height);
	inst->colorspace = V4L2_COLORSPACE_DEFAULT;
	inst->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	inst->quantization = V4L2_QUANTIZATION_DEFAULT;
	inst->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	inst->framerate = DEFAULT_FRAME_RATE;

	inst->id = ida_alloc(&inst->vpu_dev->inst_ida, GFP_KERNEL);
	if (inst->id < 0) {
		dev_warn(vpu->dev, "Allocating instance ID, fail: %d\n", inst->id);
		ret = inst->id;
		goto cleanup_inst;
	}

	return 0;

cleanup_inst:
	v4l2_ctrl_handler_free(&inst->v4l2_ctrl_hdl);
	v4l2_fh_del(&inst->v4l2_fh, filp);
	v4l2_fh_exit(&inst->v4l2_fh);
err_m2m_release:
	v4l2_m2m_ctx_release(inst->v4l2_fh.m2m_ctx);
free_inst:
	kfree(inst);

	return ret;
}

static int coda_vpu_enc_release(struct file *filp)
{
	struct vpu_instance *inst = coda_to_vpu_inst(file_to_v4l2_fh(filp));

	dev_dbg(inst->vpu_dev->dev, "[%d] release %d\n", inst->id, inst->state);
	v4l2_m2m_ctx_release(inst->v4l2_fh.m2m_ctx);

	mutex_lock(&inst->vpu_dev->dev_lock);
	if (inst->state != VPU_INST_STATE_NONE) {
		v4l2_m2m_suspend(inst->vpu_dev->m2m_dev);
		coda_vpu_enc_destroy_instance(inst);
		v4l2_m2m_resume(inst->vpu_dev->m2m_dev);
	}
	mutex_unlock(&inst->vpu_dev->dev_lock);

	ida_free(&inst->vpu_dev->inst_ida, inst->id);
	v4l2_ctrl_handler_free(&inst->v4l2_ctrl_hdl);
	v4l2_fh_del(&inst->v4l2_fh, filp);
	v4l2_fh_exit(&inst->v4l2_fh);
	kfree(inst);

	return 0;
}

static const struct v4l2_file_operations coda_vpu_enc_fops = {
	.owner = THIS_MODULE,
	.open = coda_vpu_enc_open,
	.release = coda_vpu_enc_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
};

int coda_vpu_enc_register_device(struct vpu_device *vpu)
{
	struct video_device *vdev_enc;
	int ret;

	vdev_enc = devm_kzalloc(vpu->v4l2_dev.dev, sizeof(*vdev_enc), GFP_KERNEL);
	if (!vdev_enc)
		return -ENOMEM;

	vpu->video_dev_enc = vdev_enc;

	strscpy(vdev_enc->name, VPU_ENC_DEV_NAME, sizeof(vdev_enc->name));
	vdev_enc->fops = &coda_vpu_enc_fops;
	vdev_enc->ioctl_ops = &coda_vpu_enc_ioctl_ops;
	vdev_enc->release = video_device_release_empty;
	vdev_enc->v4l2_dev = &vpu->v4l2_dev;
	vdev_enc->vfl_dir = VFL_DIR_M2M;
	vdev_enc->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	vdev_enc->lock = &vpu->dev_lock;
	video_set_drvdata(vdev_enc, vpu);

	ret = video_register_device(vdev_enc, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;

	return 0;
}

void coda_vpu_enc_unregister_device(struct vpu_device *vpu)
{
	video_unregister_device(vpu->video_dev_enc);
}
