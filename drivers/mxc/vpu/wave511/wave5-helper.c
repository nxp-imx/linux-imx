// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave5 series multi-standard codec IP - decoder interface
 *
 * Copyright (C) 2026 CHIPS&MEDIA INC
 */

#include <linux/pm_runtime.h>
#include "wave5-helper.h"
#include "wave5-vpu-dbg.h"

#define DEFAULT_BS_SIZE(width, height) ((width) * (height) / 8 * 3)

const char *state_to_str(enum vpu_instance_state state)
{
	switch (state) {
	case VPU_INST_STATE_NONE:
		return "NONE";
	case VPU_INST_STATE_OPEN:
		return "OPEN";
	case VPU_INST_STATE_INIT_SEQ:
		return "INIT_SEQ";
	case VPU_INST_STATE_PIC_RUN:
		return "PIC_RUN";
	case VPU_INST_STATE_STOP:
		return "STOP";
	case VPU_INST_STATE_ERROR:
		return "ERROR";
	default:
		return "UNKNOWN";
	}
}

static void wave5_fill_pixfmt_mp(struct v4l2_pix_format_mplane *pix_mp,
				 unsigned int width,
				 unsigned int height)
{
	const struct v4l2_format_info *fmt_info;
	unsigned int stride_y;
	int i;

	fmt_info = v4l2_format_info(pix_mp->pixelformat);
	if (!fmt_info)
		return;

	pix_mp->width = width;
	pix_mp->height = height;
	pix_mp->num_planes = fmt_info->mem_planes;

	stride_y = width * fmt_info->bpp[0];
	if (pix_mp->plane_fmt[0].bytesperline <= W5_MAX_PIC_STRIDE)
		stride_y = max(stride_y, pix_mp->plane_fmt[0].bytesperline);
	stride_y = round_up(stride_y, W5_PIC_STRIDE_ALIGNMENT);
	pix_mp->plane_fmt[0].bytesperline = stride_y;
	pix_mp->plane_fmt[0].sizeimage = stride_y * height;

	stride_y = DIV_ROUND_UP(stride_y, fmt_info->bpp[0]);

	for (i = 1; i < fmt_info->comp_planes; i++) {
		unsigned int stride_c, sizeimage_c;

		stride_c = DIV_ROUND_UP(stride_y, fmt_info->hdiv) *
			   fmt_info->bpp[i];
		sizeimage_c = stride_c * DIV_ROUND_UP(height, fmt_info->vdiv);

		if (fmt_info->mem_planes == 1) {
			pix_mp->plane_fmt[0].sizeimage += sizeimage_c;
		} else {
			pix_mp->plane_fmt[i].bytesperline = stride_c;
			pix_mp->plane_fmt[i].sizeimage = sizeimage_c;
		}
	}
}

void wave5_update_pix_fmt(struct v4l2_pix_format_mplane *pix_mp,
			  int pix_fmt_type,
			  unsigned int width,
			  unsigned int height,
			  const struct v4l2_frmsize_stepwise *frmsize,
			  bool new_resolution)
{
	v4l2_apply_frmsize_constraints(&width, &height, frmsize);

	if (pix_fmt_type == VPU_FMT_TYPE_CODEC) {
		pix_mp->width = width;
		pix_mp->height = height;
		pix_mp->num_planes = 1;
		pix_mp->plane_fmt[0].bytesperline = 0;
		pix_mp->plane_fmt[0].sizeimage = max(DEFAULT_BS_SIZE(width, height),
						     pix_mp->plane_fmt[0].sizeimage);
	} else {
		if (new_resolution)
			pix_mp->plane_fmt[0].bytesperline = 0;

		wave5_fill_pixfmt_mp(pix_mp, width, height);
	}
	pix_mp->flags = 0;
	pix_mp->field = V4L2_FIELD_NONE;
}

void wave5_update_output_format_info(struct vpu_instance *inst)
{
	if (inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV12 ||
	    inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV12M) {
		inst->cbcr_interleave = true;
		inst->nv21 = false;
		inst->output_format = FORMAT_420;
	} else if (inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV21 ||
		   inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV21M) {
		inst->cbcr_interleave = true;
		inst->nv21 = true;
		inst->output_format = FORMAT_420;
	} else if (inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV16 ||
		   inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV16M) {
		inst->cbcr_interleave = true;
		inst->nv21 = false;
		inst->output_format = FORMAT_422;
	} else if (inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV61 ||
		   inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV61M) {
		inst->cbcr_interleave = true;
		inst->nv21 = true;
		inst->output_format = FORMAT_422;
	} else if (inst->dst_fmt.pixelformat == V4L2_PIX_FMT_YUV422P ||
		   inst->dst_fmt.pixelformat == V4L2_PIX_FMT_YUV422M) {
		inst->cbcr_interleave = false;
		inst->nv21 = false;
		inst->output_format = FORMAT_422;
	} else if (inst->dst_fmt.pixelformat == V4L2_PIX_FMT_GREY) {
		inst->cbcr_interleave = false;
		inst->nv21 = false;
		inst->output_format = FORMAT_400;
	} else if (inst->dst_fmt.pixelformat == V4L2_PIX_FMT_P010) {
		inst->cbcr_interleave = true;
		inst->nv21 = false;
		inst->output_format = FORMAT_420_P10_16BIT_MSB;
	} else {
		inst->cbcr_interleave = false;
		inst->nv21 = false;
		inst->output_format = FORMAT_420;
	}
}

void wave5_cleanup_instance(struct vpu_instance *inst, struct file *filp)
{
	long usage;

	imx_mur_release_v4l2_ctrl(inst->recorder);
	v4l2_ctrl_handler_free(&inst->v4l2_ctrl_hdl);
	if (inst->v4l2_fh.vdev) {
		v4l2_fh_del(&inst->v4l2_fh, filp);
		v4l2_fh_exit(&inst->v4l2_fh);
	}
	usage = imx_mur_long_read(inst->recorder);
	if (usage)
		dev_err(inst->dev->dev, "[%d] leak memory, size is %ld\n", inst->id, usage);
	imx_mur_destroy_node(inst->recorder);
	kfree(inst->codec_info);
	kfree(inst);
}

int wave5_vpu_release_device(struct file *filp,
			     int (*close_func)(struct vpu_instance *inst, u32 *fail_res),
			     char *name)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(file_to_v4l2_fh(filp));
	struct vpu_device *dev = inst->dev;
	int ret = 0;

	wave5_vpu_remove_dbgfs_file(inst);
	v4l2_m2m_ctx_release(inst->v4l2_fh.m2m_ctx);
	if (inst->state != VPU_INST_STATE_NONE) {
		u32 fail_res;

		ret = close_func(inst, &fail_res);
		if (fail_res == WAVE5_SYSERR_VPU_STILL_RUNNING) {
			dev_err(inst->dev->dev, "%s close failed, device is still running\n",
				name);
			return -EBUSY;
		}
		if (ret && ret != -EIO) {
			dev_err(inst->dev->dev, "%s close, fail: %d\n", name, ret);
			return ret;
		}

		if (!pm_runtime_suspended(inst->dev->dev))
			pm_runtime_put_sync(inst->dev->dev);
	}

	wave5_cleanup_instance(inst, filp);
	if (dev->irq < 0) {
		scoped_guard(mutex, &dev->dev_lock) {
			if (list_empty(&dev->instances)) {
				dev_dbg(dev->dev, "Disabling the hrtimer\n");
				hrtimer_cancel(&dev->hrtimer);
			}
		}
	}

	return ret;
}

int wave5_vpu_queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq,
			 const struct vb2_ops *ops)
{
	struct vpu_instance *inst = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->ops = ops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->buf_struct_size = sizeof(struct vpu_src_buffer);
	src_vq->drv_priv = inst;
	src_vq->lock = &inst->dev->dev_lock;
	src_vq->dev = inst->dev->v4l2_dev.dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->ops = ops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->buf_struct_size = sizeof(struct vpu_dst_buffer);
	dst_vq->drv_priv = inst;
	dst_vq->lock = &inst->dev->dev_lock;
	dst_vq->dev = inst->dev->v4l2_dev.dev;
	ret = vb2_queue_init(dst_vq);
	if (ret)
		return ret;

	return 0;
}

int wave5_vpu_subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);
	bool is_decoder = inst->type == VPU_INST_TYPE_DEC;

	dev_dbg(inst->dev->dev, "%s: [%s] type: %u id: %u | flags: %u\n", __func__,
		is_decoder ? "decoder" : "", sub->type, sub->id, sub->flags);

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		if (is_decoder)
			return v4l2_src_change_event_subscribe(fh, sub);
		return -EINVAL;
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

int wave5_vpu_g_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(file_to_v4l2_fh(file));
	int i;

	f->fmt.pix_mp.width = inst->src_fmt.width;
	f->fmt.pix_mp.height = inst->src_fmt.height;
	f->fmt.pix_mp.pixelformat = inst->src_fmt.pixelformat;
	f->fmt.pix_mp.field = inst->src_fmt.field;
	f->fmt.pix_mp.flags = inst->src_fmt.flags;
	f->fmt.pix_mp.num_planes = inst->src_fmt.num_planes;
	for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		f->fmt.pix_mp.plane_fmt[i].bytesperline = inst->src_fmt.plane_fmt[i].bytesperline;
		f->fmt.pix_mp.plane_fmt[i].sizeimage = inst->src_fmt.plane_fmt[i].sizeimage;
	}

	f->fmt.pix_mp.colorspace = inst->colorspace;
	f->fmt.pix_mp.ycbcr_enc = inst->ycbcr_enc;
	f->fmt.pix_mp.quantization = inst->quantization;
	f->fmt.pix_mp.xfer_func = inst->xfer_func;

	return 0;
}

const struct vpu_format *wave5_find_vpu_fmt(unsigned int v4l2_pix_fmt,
					    const struct vpu_format fmt_list[MAX_FMTS])
{
	unsigned int index;

	for (index = 0; index < MAX_FMTS; index++) {
		if (fmt_list[index].v4l2_pix_fmt == v4l2_pix_fmt)
			return &fmt_list[index];
	}

	return NULL;
}

const struct vpu_format *wave5_find_vpu_fmt_by_idx(unsigned int idx,
						   const struct vpu_format fmt_list[MAX_FMTS])
{
	if (idx >= MAX_FMTS)
		return NULL;

	if (!fmt_list[idx].v4l2_pix_fmt)
		return NULL;

	return &fmt_list[idx];
}

enum wave_std wave5_to_vpu_std(unsigned int v4l2_pix_fmt, enum vpu_instance_type type)
{
	switch (v4l2_pix_fmt) {
	case V4L2_PIX_FMT_H264:
		return type == VPU_INST_TYPE_DEC ? W_AVC_DEC : STD_UNKNOWN;
	case V4L2_PIX_FMT_HEVC:
		return type == VPU_INST_TYPE_DEC ? W_HEVC_DEC : STD_UNKNOWN;
	default:
		return STD_UNKNOWN;
	}
}

void wave5_return_bufs(struct vb2_queue *q, u32 state)
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			vbuf = v4l2_m2m_src_buf_remove(m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(m2m_ctx);
		if (!vbuf)
			return;
		v4l2_ctrl_request_complete(vbuf->vb2_buf.req_obj.req, &inst->v4l2_ctrl_hdl);
		v4l2_m2m_buf_done(vbuf, state);
	}
}

static struct vb2_v4l2_buffer *wave5_vpu_get_next_buf(struct v4l2_m2m_queue_ctx *q_ctx,
						      wave5_compare_vb compare,
						      unsigned long target)
{
	struct v4l2_m2m_buffer *buf, *tmp;

	if (!q_ctx || !compare)
		return NULL;

	scoped_guard(spinlock_irqsave, &q_ctx->rdy_spinlock) {
		if (list_empty(&q_ctx->rdy_queue))
			return NULL;

		list_for_each_entry_safe(buf, tmp, &q_ctx->rdy_queue, list) {
			if (compare(&buf->vb, target))
				return &buf->vb;
		}
	}

	return NULL;
}

struct vb2_v4l2_buffer *wave5_vpu_get_next_src_buf(struct vpu_instance *inst,
						   wave5_compare_vb compare,
						   unsigned long target)
{
	return wave5_vpu_get_next_buf(&inst->v4l2_fh.m2m_ctx->out_q_ctx, compare, target);
}

struct vb2_v4l2_buffer *wave5_vpu_get_next_dst_buf(struct vpu_instance *inst,
						   wave5_compare_vb compare,
						   unsigned long target)
{
	return wave5_vpu_get_next_buf(&inst->v4l2_fh.m2m_ctx->cap_q_ctx, compare, target);
}

static bool wave5_vpu_check_dst_buf_by_index(struct vb2_v4l2_buffer *vbuf, unsigned long index)
{
	return vbuf->vb2_buf.index == index ? true : false;
}

struct vb2_v4l2_buffer *wave5_vpu_get_dst_buffer_by_idx(struct vpu_instance *inst, int index)
{
	return wave5_vpu_get_next_dst_buf(inst, wave5_vpu_check_dst_buf_by_index, index);
}

struct vb2_v4l2_buffer *wave5_vpu_get_reusable_buffer(struct vpu_instance *inst, int index)
{
	struct vb2_v4l2_buffer *vbuf;

	vbuf = wave5_vpu_get_dst_buffer_by_idx(inst, index);
	if (vbuf && !test_and_set_bit(index, &inst->avail_dst_bufs)) {
		dev_dbg(inst->dev->dev, "[%d] reuse buffer %d\n", inst->id, index);
		return vbuf;
	}

	return NULL;
}

struct vb2_v4l2_buffer *wave5_vpu_get_display_buffer(struct vpu_instance *inst, int index)
{
	struct vb2_v4l2_buffer *vbuf;
	struct vpu_dst_buffer *vpu_buf;

	vbuf = v4l2_m2m_dst_buf_remove_by_idx(inst->v4l2_fh.m2m_ctx, index);
	if (!vbuf)
		return NULL;

	vpu_buf = wave5_to_vpu_dst_buf(vbuf);

	return vbuf;
}

bool wave5_vpu_check_fb_available(struct vpu_instance *inst)
{
	if (hweight_long(inst->avail_dst_bufs) > atomic_read(&inst->queued_dec_cmd))
		return true;

	return false;
}

void wave5_vpu_handle_performance(struct vpu_instance *inst, struct vpu_dst_buffer *vpu_buf)
{
	s64 latency, time_spent;

	if (!inst || !vpu_buf)
		return;

	inst->performance.ts_last = vpu_buf->ts_output;

	latency = vpu_buf->ts_output - vpu_buf->ts_input;
	time_spent = vpu_buf->ts_finish - vpu_buf->ts_start;

	if (!inst->performance.latency_first)
		inst->performance.latency_first = latency;
	inst->performance.latency_max = max_t(s64, latency, inst->performance.latency_max);

	if (!inst->performance.min_process_time)
		inst->performance.min_process_time = time_spent;
	else if (inst->performance.min_process_time > time_spent)
		inst->performance.min_process_time = time_spent;

	if (inst->performance.max_process_time < time_spent)
		inst->performance.max_process_time = time_spent;

	inst->performance.total_sw_time += time_spent;
	inst->performance.total_hw_time += vpu_buf->hw_time;
}

void wave5_vpu_reset_performace(struct vpu_instance *inst)
{
	s64 tmp;
	s64 fps_act = 0, fps_sw = 0, fps_hw = 0, fps_hw_2 = 0;
	struct vpu_performance_info *perf;

	if (!inst)
		return;

	if (!inst->processed_buf_num)
		goto exit;

	perf = &inst->performance;
	tmp = MSEC_PER_SEC * inst->displayed_buf_num * 10;
	fps_act = DIV_ROUND_CLOSEST(tmp * NSEC_PER_MSEC, (perf->ts_last - perf->ts_first));
	tmp = MSEC_PER_SEC * inst->processed_buf_num * 10;
	fps_sw = DIV_ROUND_CLOSEST(tmp * NSEC_PER_MSEC, perf->total_sw_time);
	if (perf->total_hw_time) {
		fps_hw = DIV_ROUND_CLOSEST(tmp * NSEC_PER_MSEC, perf->total_hw_time);
		if (perf->total_hw_time > perf->first_hw_time && inst->processed_buf_num > 1) {
			tmp = MSEC_PER_SEC * (inst->processed_buf_num - 1) * 10;
			fps_hw_2 = DIV_ROUND_CLOSEST(tmp * NSEC_PER_MSEC,
						     (perf->total_hw_time - perf->first_hw_time));
		}
	}
	dev_dbg(inst->dev->dev,
		"[%d] fps actual: %lld.%lld, sw: %lld.%lld, hw: %lld.%lld (%lld.%lld)\n",
		inst->id,
		fps_act / 10, fps_act % 10,
		fps_sw / 10, fps_sw % 10,
		fps_hw / 10, fps_hw % 10,
		fps_hw_2 / 10, fps_hw_2 % 10);
	dev_dbg(inst->dev->dev,
		"[%d] latency(ms) %llu.%06llu, decode %d, display %d, total_sw_time %lld\n",
		inst->id,
		perf->latency_first / NSEC_PER_MSEC,
		perf->latency_first % NSEC_PER_MSEC,
		inst->processed_buf_num, inst->displayed_buf_num, perf->total_sw_time);

exit:
	memset(&inst->performance, 0, sizeof(inst->performance));
}

dma_addr_t wave5_get_plane_dma_addr(struct vb2_buffer *buf, unsigned int plane_no)
{
	if (plane_no >= buf->num_planes)
		return 0;
	return vb2_dma_contig_plane_dma_addr(buf, plane_no) + buf->planes[plane_no].data_offset;
}

unsigned long wave5_get_plane_payload(struct vb2_buffer *buf, unsigned int plane_no)
{
	if (plane_no >= buf->num_planes)
		return 0;
	return vb2_get_plane_payload(buf, plane_no) - buf->planes[plane_no].data_offset;
}

void wave5_vpu_record_flow(struct vpu_instance *inst, u32 flow, u32 arg1, u32 arg2)
{
	int index;

	scoped_guard(spinlock_irqsave, &inst->flow.lock)
		index = inst->flow.index++;

	index %= WAVE5_VPU_FLOW_DEPTH;
	inst->flow.flows[index].arg1 = arg1;
	inst->flow.flows[index].arg2 = arg2;
	inst->flow.flows[index].key = flow;
}
