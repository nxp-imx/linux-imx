// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Coda series multi-standard codec IP - v4l2 interface helper
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#include <linux/sizes.h>
#include "coda-helper.h"

#define DEFAULT_BS_SIZE(width, height) ((width) * (height) / 8 * 3)

void coda_vpu_update_pix_fmt(struct v4l2_pix_format_mplane *pix_mp,
			     int pix_fmt_type,
			     unsigned int width,
			     unsigned int height,
			     const struct v4l2_frmsize_stepwise *frmsize)
{
	v4l2_apply_frmsize_constraints(&width, &height, frmsize);

	if (pix_fmt_type == VPU_FMT_TYPE_CODEC) {
		pix_mp->width = width;
		pix_mp->height = height;
		pix_mp->num_planes = 1;
		pix_mp->plane_fmt[0].bytesperline = 0;
		pix_mp->plane_fmt[0].sizeimage = max(ALIGN(DEFAULT_BS_SIZE(width, height), 1024),
						     ALIGN(pix_mp->plane_fmt[0].sizeimage, 1024));
		pix_mp->plane_fmt[0].sizeimage = clamp_t(u32, pix_mp->plane_fmt[0].sizeimage,
							 SZ_2K, SZ_256M);
	} else {
		const struct v4l2_format_info *fmt_info;
		unsigned int stride_y;
		int i;

		fmt_info = v4l2_format_info(pix_mp->pixelformat);
		if (!fmt_info) {
			pix_mp->plane_fmt[0].bytesperline = 0;
			if (!pix_mp->plane_fmt[0].sizeimage)
				pix_mp->plane_fmt[0].sizeimage = width * height;
			return;
		}
		pix_mp->width = width;
		pix_mp->height = height;
		pix_mp->num_planes = fmt_info->mem_planes;

		stride_y = width * fmt_info->bpp[0];
		if (pix_mp->plane_fmt[0].bytesperline <= CODA_ENC_MAX_PIC_WIDTH)
			stride_y = max(stride_y, pix_mp->plane_fmt[0].bytesperline);
		else
			stride_y = round_up(stride_y, 32);

		pix_mp->plane_fmt[0].bytesperline = stride_y;
		pix_mp->plane_fmt[0].sizeimage = stride_y * height;

		stride_y = DIV_ROUND_UP(stride_y, fmt_info->bpp[0]);
		for (i = 1; i < fmt_info->comp_planes; i++) {
			unsigned int stride_c, sizeimage_c;

			stride_c = DIV_ROUND_UP(stride_y, fmt_info->hdiv) * fmt_info->bpp[i];
			sizeimage_c = stride_c * DIV_ROUND_UP(height, fmt_info->vdiv);

			if (fmt_info->mem_planes == 1) {
				pix_mp->plane_fmt[0].sizeimage += sizeimage_c;
			} else {
				pix_mp->plane_fmt[i].bytesperline = stride_c;
				pix_mp->plane_fmt[i].sizeimage = sizeimage_c;
			}
		}
	}
	pix_mp->flags = 0;
	pix_mp->field = V4L2_FIELD_NONE;
}

static void coda_vpu_device_run_timeout(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct vpu_device *vpu = container_of(dwork, struct vpu_device, task_timer);
	struct vpu_instance *inst = v4l2_m2m_get_curr_priv(vpu->m2m_dev);
	struct vb2_v4l2_buffer *src_buf = NULL;
	struct vb2_v4l2_buffer *dst_buf = NULL;

	if (!inst)
		return;

	dev_err(vpu->dev, "[%d] timeout\n", inst->id);

	src_buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx);
	if (src_buf)
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);

	dst_buf = v4l2_m2m_dst_buf_remove(inst->v4l2_fh.m2m_ctx);
	if (dst_buf)
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);

	vb2_queue_error(v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx));
	vb2_queue_error(v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx));

	v4l2_m2m_job_finish(vpu->m2m_dev, inst->v4l2_fh.m2m_ctx);
}

static void coda_vpu_device_run(void *priv)
{
	struct vpu_instance *inst = priv;
	struct vpu_device *vpu = inst->vpu_dev;
	int ret;

	dev_dbg(vpu->dev, "[%d]%s: state %d\n", inst->id, __func__, inst->state);

	ret = inst->ops->start_process(inst);
	if (!ret)
		schedule_delayed_work(&vpu->task_timer, msecs_to_jiffies(CODA_VPU_TIMEOUT));
	else
		v4l2_m2m_job_finish(vpu->m2m_dev, inst->v4l2_fh.m2m_ctx);
}

static int coda_vpu_job_ready(void *priv)
{
	struct vpu_instance *inst = priv;

	dev_dbg(inst->vpu_dev->dev, "[%d]%s: state %d\n", inst->id, __func__, inst->state);

	if (inst->state == VPU_INST_STATE_STOP && inst->eos)
		return 0;

	return 1;
}

static void coda_vpu_job_abort(void *priv)
{
	struct vpu_instance *inst = priv;

	dev_dbg(inst->vpu_dev->dev, "[%d]%s: state %d\n",
		inst->id, __func__, inst->state);

	inst->ops->stop_process(inst);
}

static const struct v4l2_m2m_ops coda_vpu_m2m_ops = {
	.device_run = coda_vpu_device_run,
	.job_ready = coda_vpu_job_ready,
	.job_abort = coda_vpu_job_abort,
};

int coda_vpu_init_m2m_dev(struct vpu_device *vpu)
{
	vpu->m2m_dev = v4l2_m2m_init(&coda_vpu_m2m_ops);
	if (IS_ERR(vpu->m2m_dev)) {
		dev_err(vpu->dev, "v4l2_m2m_init fail: %ld\n", PTR_ERR(vpu->m2m_dev));
		return PTR_ERR(vpu->m2m_dev);
	}

	INIT_DELAYED_WORK(&vpu->task_timer, coda_vpu_device_run_timeout);

	return 0;
}

void coda_vpu_release_m2m_dev(struct vpu_device *vpu)
{
	v4l2_m2m_release(vpu->m2m_dev);
}

int coda_vpu_wait_interrupt(struct vpu_instance *inst, unsigned int timeout)
{
	int ret;

	ret = wait_for_completion_timeout(&inst->vpu_dev->irq_done,
					  msecs_to_jiffies(timeout));
	if (!ret)
		return -ETIMEDOUT;

	reinit_completion(&inst->vpu_dev->irq_done);

	return 0;
}

void coda_vpu_set_instance_state(struct vpu_instance *inst, u32 state)
{
	inst->state = state;
}

void coda_vpu_return_buffers(struct vpu_instance *inst,
			     unsigned int type, enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *buf;
	int i;

	if (V4L2_TYPE_IS_OUTPUT(type)) {
		while ((buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, state);
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(inst->v4l2_fh.m2m_ctx))) {
			for (i = 0; i < inst->dst_fmt.num_planes; i++)
				vb2_set_plane_payload(&buf->vb2_buf, i, 0);
			v4l2_m2m_buf_done(buf, state);
		}
	}
}

struct vb2_v4l2_buffer *coda_vpu_get_valid_src_buf(struct vpu_instance *inst)
{
	struct vb2_v4l2_buffer *vb2_v4l2_buf;
	struct v4l2_m2m_buffer *v4l2_m2m_buf;
	struct vpu_buffer *vpu_buf = NULL;

	v4l2_m2m_for_each_src_buf(inst->v4l2_fh.m2m_ctx, v4l2_m2m_buf) {
		vb2_v4l2_buf = &v4l2_m2m_buf->vb;
		vpu_buf = coda_to_vpu_buf(vb2_v4l2_buf);

		if (!vpu_buf->consumed) {
			dev_dbg(inst->vpu_dev->dev, "no consumed src idx : %d\n",
				vb2_v4l2_buf->vb2_buf.index);
			return vb2_v4l2_buf;
		}
	}

	return NULL;
}

struct vb2_v4l2_buffer *coda_vpu_get_valid_dst_buf(struct vpu_instance *inst)
{
	struct vb2_v4l2_buffer *vb2_v4l2_buf;
	struct v4l2_m2m_buffer *v4l2_m2m_buf;
	struct vpu_buffer *vpu_buf;

	v4l2_m2m_for_each_dst_buf(inst->v4l2_fh.m2m_ctx, v4l2_m2m_buf) {
		vb2_v4l2_buf = &v4l2_m2m_buf->vb;
		vpu_buf = coda_to_vpu_buf(vb2_v4l2_buf);

		if (!vpu_buf->consumed) {
			dev_dbg(inst->vpu_dev->dev, "no consumed dst idx : %d\n",
				vb2_v4l2_buf->vb2_buf.index);
			return vb2_v4l2_buf;
		}
	}

	return NULL;
}

struct vb2_v4l2_buffer *coda_vpu_get_dst_buf_by_addr(struct vpu_instance *inst,
						     dma_addr_t addr)
{
	struct vb2_v4l2_buffer *vb2_v4l2_buf;
	struct v4l2_m2m_buffer *v4l2_m2m_buf;
	struct vb2_v4l2_buffer *dst_buf = NULL;

	v4l2_m2m_for_each_dst_buf(inst->v4l2_fh.m2m_ctx, v4l2_m2m_buf) {
		vb2_v4l2_buf = &v4l2_m2m_buf->vb;
		if (addr == vb2_dma_contig_plane_dma_addr(&vb2_v4l2_buf->vb2_buf, 0)) {
			dst_buf = vb2_v4l2_buf;
			break;
		}
	}

	return dst_buf;
}

dma_addr_t coda_vpu_get_dma_addr(struct vb2_v4l2_buffer *buf, unsigned int plane_no)
{
	return vb2_dma_contig_plane_dma_addr(&buf->vb2_buf, plane_no) +
			buf->planes[plane_no].data_offset;
}
