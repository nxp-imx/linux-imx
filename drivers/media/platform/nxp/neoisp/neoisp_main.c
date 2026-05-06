// SPDX-License-Identifier: GPL-2.0+
/*
 * NEOISP main driver source code
 *
 * This is a derived work from the PiSP Back End driver
 * Copyright (c) 2021-2024 Raspberry Pi Limited
 *
 * Copyright 2023-2026 NXP
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/lockdep.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-isp.h>
#include <media/videobuf2-dma-contig.h>

#include "neoisp.h"
#include "neoisp_core.h"
#include "neoisp_fmt.h"
#include "neoisp_nodes.h"
#include "neoisp_regs.h"
#include "neoisp_ctx.h"

/* \todo : Must be in videodev2.h uapi file */
/* Vendor specific - used for NXP NEOISP sub-system */
#define V4L2_META_FMT_NEO_ISP_PARAMS v4l2_fourcc('N', 'N', 'I', 'P') /* NXP NEOISP Parameters */
#define V4L2_META_FMT_NEO_ISP_EXT_PARAMS v4l2_fourcc('N', 'N', 'E', 'P') /* NXP NEOISP Ext Params */
#define V4L2_META_FMT_NEO_ISP_STATS v4l2_fourcc('N', 'N', 'I', 'S') /* NXP NEOISP Statistics */
#define V4L2_META_FMT_NEO_ISP_EXT_STATS v4l2_fourcc('N', 'N', 'E', 'S') /* NXP NEOISP Ext Stats */

#define NODE_NAME(node) \
	(node_desc[(node)->id].ent_name + sizeof(NEOISP_NAME))

static int standalone_mdev;
module_param_named(standalone_mdev, standalone_mdev, uint, 0644);
MODULE_PARM_DESC(standalone_mdev, " Create standalone neoisp media device, default is 0 (off)");

static int enable_debugfs;
module_param_named(enable_debugfs, enable_debugfs, uint, 0644);
MODULE_PARM_DESC(enable_debugfs, " Turn on/off debugfs, default is 0 (off)");

static inline bool node_desc_is_output(const struct neoisp_node_desc_s *desc)
{
	return desc->buf_type == V4L2_BUF_TYPE_META_OUTPUT ||
		desc->buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT ||
		desc->buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
}

static inline bool node_is_meta(struct neoisp_node_s *node)
{
	return node->buf_type == V4L2_BUF_TYPE_META_OUTPUT ||
		node->buf_type == V4L2_BUF_TYPE_META_CAPTURE;
}

static inline bool node_is_output(struct neoisp_node_s *node)
{
	return node->buf_type == V4L2_BUF_TYPE_META_OUTPUT ||
		node->buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT ||
		node->buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
}

static inline bool node_is_capture(struct neoisp_node_s *node)
{
	return node->buf_type == V4L2_BUF_TYPE_META_CAPTURE ||
		node->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE ||
		node->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
}

static inline bool node_is_mplane(struct neoisp_node_s *node)
{
	return node->buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ||
		node->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
}

static inline const struct neoisp_fmt_s *neoisp_find_pixel_format(u32 pixel_format,
								  const struct neoisp_fmt_s *fmt,
								  u32 size)
{
	u32 i;

	for (i = 0; i < size; i++)
		if (fmt[i].fourcc == pixel_format)
			return &fmt[i];
	return NULL;
}

/*
 * The gain adjustment should be done, as the 12-bit format is managed in a specific way.
 * LPALIGN0/1 bit field is used to select LSB or MSB alignment. However, LPALIGN0/1
 * is disabled for 12-bit operations and data is always aligned in the following manner:
 * d[15] -> d[4]
 *
 * In this sense, a gain is applied to the HDR Decompression block to align the data on d[19] for
 * input0 as other formats are defined. As the working BPP of input1 is 16-bit depth, the data is
 * already MSB-aligned and do not need an extra gain.
 */
static inline void neoisp_adjust_gain(struct neoisp_context_s *ctx, u32 ibpp)
{
	struct neoisp_hdr_decompress0_s *hdr0 = &ctx->hw.hdr_decompress0;

	if (ibpp != 12)
		return;

	hdr0->knee_ratio4 =
		NEO_HDR_DECOMPRESS0_KNEE_RATIO4_CAM0_RATIO4_SET(16 << NEOISP_HDR_SHIFT_RADIX);
}

static void neoisp_fill_mp(struct v4l2_format *f, const struct neoisp_fmt_s *fmt)
{
	u32 nplanes = f->fmt.pix_mp.num_planes;
	u32 i;

	for (i = 0; i < nplanes; i++) {
		struct v4l2_plane_pix_format *p = &f->fmt.pix_mp.plane_fmt[i];
		u32 bpl, plane_size;

		bpl = f->fmt.pix_mp.width * ((fmt->bit_depth + 7) >> 3);
		bpl = ALIGN(max(p->bytesperline, bpl), fmt->align);

		plane_size = bpl * f->fmt.pix_mp.height;
		if (nplanes > 1)
			plane_size /= fmt->pl_divisors[i];
		plane_size = max(p->sizeimage, plane_size);

		p->bytesperline = bpl;
		p->sizeimage = plane_size;
	}
}

static const struct neoisp_fmt_s *neoisp_find_pixel_format_by_node(u32 pixel_format,
								   struct neoisp_node_s *node)
{
	if (IS_ERR_OR_NULL(node))
		return NULL;

	switch (node->id) {
	case NEOISP_INPUT0_NODE:
	case NEOISP_INPUT1_NODE:
		return neoisp_find_pixel_format(pixel_format,
						formats_vout,
						ARRAY_SIZE(formats_vout));
	case NEOISP_FRAME_NODE:
		return neoisp_find_pixel_format(pixel_format,
						formats_vcap,
						ARRAY_SIZE(formats_vcap));
	case NEOISP_IR_NODE:
		return neoisp_find_pixel_format(pixel_format,
						formats_vcap_ir,
						ARRAY_SIZE(formats_vcap_ir));
	case NEOISP_PARAMS_NODE:
		return neoisp_find_pixel_format(pixel_format,
						formats_mout,
						ARRAY_SIZE(formats_mout));
	case NEOISP_STATS_NODE:
		return neoisp_find_pixel_format(pixel_format,
						formats_mcap,
						ARRAY_SIZE(formats_mcap));
	default:
		return NULL;
	}
}

const struct neoisp_fmt_s *neoisp_find_video_capture_format(u32 pixel_format)
{
	return neoisp_find_pixel_format(pixel_format,
					formats_vcap,
					ARRAY_SIZE(formats_vcap));
}

static int neoisp_node_queue_setup(struct vb2_queue *q, u32 *nbuffers,
				   u32 *nplanes, u32 sizes[],
				   struct device *alloc_devs[])
{
	struct neoisp_node_s *node = vb2_get_drv_priv(q);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;
	u32 i, num_planes;

	num_planes = node_is_mplane(node) ?
		     node->format.fmt.pix_mp.num_planes : 1;
	if (*nplanes) {
		if (*nplanes != num_planes)
			return -EINVAL;

		for (i = 0; i < *nplanes; i++) {
			u32 size = node_is_mplane(node) ?
				     node->format.fmt.pix_mp.plane_fmt[i].sizeimage :
				     node->format.fmt.meta.buffersize;

			if (sizes[i] < size)
				return -EINVAL;
		}

		return 0;
	}

	*nplanes = num_planes;
	for (i = 0; i < *nplanes; i++)
		sizes[i] = node_is_mplane(node) ?
			   node->format.fmt.pix_mp.plane_fmt[i].sizeimage :
			   node->format.fmt.meta.buffersize;

	dev_dbg(neoispd->dev,
		"Image (or metadata) size %u, nbuffers %u for node %s\n",
		sizes[0], *nbuffers, NODE_NAME(node));

	return 0;
}

static int neoisp_node_buf_prepare(struct vb2_buffer *vb)
{
	struct neoisp_node_s *node = vb2_get_drv_priv(vb->vb2_queue);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;
	unsigned long size = 0;
	u32 i, num_planes = node_is_mplane(node) ?
		node->format.fmt.pix_mp.num_planes : 1;

	for (i = 0; i < num_planes; i++) {
		size = node_is_mplane(node)
			? node->format.fmt.pix_mp.plane_fmt[i].sizeimage
			: node->format.fmt.meta.buffersize;

		if (vb2_plane_size(vb, i) < size) {
			dev_err(neoispd->dev,
				"data will not fit into plane %d (%lu < %lu)\n",
				i, vb2_plane_size(vb, i), size);
			return -EINVAL;
		}

		vb2_set_plane_payload(vb, i, size);
	}
	return 0;
}

#define NEOISP_PARAMS_BLOCK_INFO(block, type, ext) \
	[NEOISP_PARAM_BLK_## block] = { \
		.size = sizeof(struct neoisp_ ## type ## _ ## ext ## _es), \
	}

#define NEOISP_PARAMS_BLOCK_INFO_CFG(block, type) \
	NEOISP_PARAMS_BLOCK_INFO(block, type, cfg)

#define NEOISP_PARAMS_BLOCK_INFO_MEMS(block, type) \
	NEOISP_PARAMS_BLOCK_INFO(block, type, mem_params)

static const struct v4l2_isp_params_block_type_info neois_params_block_types_info[] = {
	NEOISP_PARAMS_BLOCK_INFO_CFG(PIPE_CONF, pipe_conf),
	NEOISP_PARAMS_BLOCK_INFO_CFG(HEAD_COLOR, head_color),
	NEOISP_PARAMS_BLOCK_INFO_CFG(HDR_DECOMPRESS0, hdr_decompress0),
	NEOISP_PARAMS_BLOCK_INFO_CFG(HDR_DECOMPRESS1, hdr_decompress1),
	NEOISP_PARAMS_BLOCK_INFO_CFG(OBWB0, obwb),
	NEOISP_PARAMS_BLOCK_INFO_CFG(OBWB1, obwb),
	NEOISP_PARAMS_BLOCK_INFO_CFG(OBWB2, obwb),
	NEOISP_PARAMS_BLOCK_INFO_CFG(HDR_MERGE, hdr_merge),
	NEOISP_PARAMS_BLOCK_INFO_CFG(RGBIR, rgbir),
	NEOISP_PARAMS_BLOCK_INFO_CFG(STAT, stat),
	NEOISP_PARAMS_BLOCK_INFO_CFG(CTEMP, ctemp),
	NEOISP_PARAMS_BLOCK_INFO_CFG(IR_COMPRESS, ir_compress),
	NEOISP_PARAMS_BLOCK_INFO_CFG(BNR, bnr),
	NEOISP_PARAMS_BLOCK_INFO_CFG(VIGNETTING_CTRL, vignetting_ctrl),
	NEOISP_PARAMS_BLOCK_INFO_CFG(DEMOSAIC, demosaic),
	NEOISP_PARAMS_BLOCK_INFO_CFG(RGB2YUV, rgb2yuv),
	NEOISP_PARAMS_BLOCK_INFO_CFG(DR_COMP, dr_comp),
	NEOISP_PARAMS_BLOCK_INFO_CFG(NR, nr),
	NEOISP_PARAMS_BLOCK_INFO_CFG(AF, af),
	NEOISP_PARAMS_BLOCK_INFO_CFG(EE, ee),
	NEOISP_PARAMS_BLOCK_INFO_CFG(DF, df),
	NEOISP_PARAMS_BLOCK_INFO_CFG(CONVMED, convmed),
	NEOISP_PARAMS_BLOCK_INFO_CFG(CAS, cas),
	NEOISP_PARAMS_BLOCK_INFO_CFG(GCM, gcm),
	NEOISP_PARAMS_BLOCK_INFO_MEMS(VIGNETTING_TABLE, vignetting_table),
	NEOISP_PARAMS_BLOCK_INFO_MEMS(DRC_GLOBAL_TONEMAP, drc_global_tonemap),
	NEOISP_PARAMS_BLOCK_INFO_MEMS(DRC_LOCAL_TONEMAP, drc_local_tonemap),
};

static int neoisp_params_node_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct neoisp_node_s *node = vb2_get_drv_priv(vb->vb2_queue);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;
	struct v4l2_isp_params_buffer *params = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(neoispd->dev, vb,
						   node->format.fmt.meta.buffersize);
	if (ret)
		return ret;

	ret = v4l2_isp_params_validate_buffer(neoispd->dev, vb,
					      params, neois_params_block_types_info,
					      ARRAY_SIZE(neois_params_block_types_info));
	if (ret)
		return ret;

	vb2_set_plane_payload(vb, 0, node->format.fmt.meta.buffersize);
	return 0;
}

static void send_frame_sync_event(struct neoisp_dev_s *neoispd)
{
	struct v4l2_subdev *sd = &neoispd->queued_job.node_group->sd;
	u32 sequence = neoispd->queued_job.node_group->frame_sequence;

	struct v4l2_event ev = {
		.type = V4L2_EVENT_FRAME_SYNC,
		.u.frame_sync.frame_sequence = sequence,
	};

	v4l2_event_queue(sd->devnode, &ev);
}

static void neoisp_reset_hw(struct neoisp_dev_s *neoispd, bool is_hw)
{
	u32 bit = NEO_PIPE_CONF_SOFT_RESET_SOFT_RESET;
	u32 val, count = 100;

	if (is_hw)
		bit = NEO_PIPE_CONF_SOFT_RESET_HARD_RESET;

	neoisp_wr(neoispd, NEO_PIPE_CONF_SOFT_RESET, bit);

	/* Wait for auto-clear */
	do {
		usleep_range(1, 2);
		val = neoisp_rd(neoispd, NEO_PIPE_CONF_SOFT_RESET);
		count--;
	} while ((val & bit) && count);

	if (val & bit)
		dev_warn(neoispd->dev, "%s reset incomplete\n",
			 is_hw ? "hw" : "sw");
}

static void neoisp_run_job(struct neoisp_dev_s *neoispd)
{
	/* Update queued job context buf addresses */
	neoisp_ctx_update_buf_addr(neoispd);

	/* Update queued job context with user space values */
	neoisp_ctx_update_w_user_params(neoispd);

	/* Upload context into HW registers and memories */
	neoisp_ctx_upload_context(neoispd);

	/* Kick off the hw */
	neoisp_wr(neoispd, NEO_PIPE_CONF_TRIG_CAM0, NEO_PIPE_CONF_TRIG_CAM0_TRIGGER);
	send_frame_sync_event(neoispd);
	dev_dbg(neoispd->dev, "isp starting ctx id: %d\n",
		neoispd->queued_job.node_group->id);
}

static int neoisp_prepare_job(struct neoisp_node_group_s *node_group)
{
	struct neoisp_job_desc_s __free(kfree) *job = NULL;
	struct neoisp_dev_s *neoispd = node_group->neoisp_dev;
	struct neoisp_buffer_s *buf[NEOISP_NODES_COUNT];
	struct neoisp_node_s *node;
	unsigned int streaming_map;
	int i;

	lockdep_assert_irqs_enabled();

	/*
	 * To schedule a job, we need to have 1 buffer for any enabled node, knowing that:
	 *  - Input0 is immutable, so it must have 1 buffer.
	 *  - Input1 is mutable, so it is ignored if not used.
	 *  - Params and Stats are also mutable, but enabled by default.
	 *  - Frame and IR are mutable; Only Frame is enabled by default. At least one
	 *    of these 2 should be enabled.
	 *
	 * If all the buffers required to form a job are available, append the job
	 * descriptor to the job queue to be later queued to the HW.
	 */
	scoped_guard(spinlock_irq, &neoispd->hw_lock) {
		if ((BIT(NEOISP_INPUT0_NODE) & node_group->streaming_map)
		    != BIT(NEOISP_INPUT0_NODE)) {
			dev_dbg(neoispd->dev, "Input0 node not ready, nothing to do\n");
			return -EAGAIN;
		}

		node = &node_group->node[NEOISP_INPUT1_NODE];
		if (neoisp_node_link_is_enabled(node)) {
			if ((BIT(NEOISP_INPUT1_NODE) & node_group->streaming_map)
			    != BIT(NEOISP_INPUT1_NODE)) {
				dev_dbg(neoispd->dev, "Input1 is not disabled and not ready\n");
				return -EAGAIN;
			}
		}
		node = &node_group->node[NEOISP_PARAMS_NODE];
		if (neoisp_node_link_is_enabled(node)) {
			if ((BIT(NEOISP_PARAMS_NODE) & node_group->streaming_map)
			    != BIT(NEOISP_PARAMS_NODE)) {
				dev_dbg(neoispd->dev, "Params is not disabled and not ready\n");
				return -EAGAIN;
			}
		}
		node = &node_group->node[NEOISP_FRAME_NODE];
		if (neoisp_node_link_is_enabled(node)) {
			if ((BIT(NEOISP_FRAME_NODE) & node_group->streaming_map)
			    != BIT(NEOISP_FRAME_NODE)) {
				dev_dbg(neoispd->dev, "Frame node not ready, nothing to do\n");
				return -EAGAIN;
			}
		}
		node = &node_group->node[NEOISP_IR_NODE];
		if (neoisp_node_link_is_enabled(node)) {
			if ((BIT(NEOISP_IR_NODE) & node_group->streaming_map)
			    != BIT(NEOISP_IR_NODE)) {
				dev_dbg(neoispd->dev, "IR node not ready, nothing to do\n");
				return -EAGAIN;
			}
		}
		node = &node_group->node[NEOISP_STATS_NODE];
		if (neoisp_node_link_is_enabled(node)) {
			if ((BIT(NEOISP_STATS_NODE) & node_group->streaming_map)
			    != BIT(NEOISP_STATS_NODE)) {
				dev_dbg(neoispd->dev, "Stats is not disabled and not ready\n");
				return -EAGAIN;
			}
		}

		/*
		 * Take a copy of streaming_map: nodes activated after this
		 * point are ignored when preparing this job
		 */
		streaming_map = node_group->streaming_map;
	}

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return -ENOMEM;

	for (i = 0; i < NEOISP_NODES_COUNT; i++) {
		buf[i] = NULL;
		if (!(streaming_map & BIT(i)))
			continue;

		node = &node_group->node[i];
		buf[i] = list_first_entry_or_null(&node->ready_queue,
						  struct neoisp_buffer_s,
						  ready_list);

		if (!buf[i] && neoisp_node_link_is_enabled(node)) {
			dev_dbg(neoispd->dev, "Nothing to do\n");
			return -EINVAL;
		}
	}

	/* Pull a buffer from each V4L2 queue to form the queued job */
	for (i = 0; i < NEOISP_NODES_COUNT; i++) {
		if (buf[i]) {
			list_del(&buf[i]->ready_list);
			job->buffers[i] = buf[i];
		}
	}

	job->node_group = node_group;

	scoped_guard(spinlock_irq, &neoispd->hw_lock) {
		list_add_tail(&job->queue, &neoispd->job_queue);
	}

	/* Set job to NULL to avoid automatic release due to __free(). */
	job = NULL;

	return 0;
}

/*
 * Try to schedule a job for a single node group. If neoisp hw is free, and
 * a job has been prepared for this group move it into the queued_job, and
 * launch it.
 */
static void neoisp_schedule(struct neoisp_dev_s *neoispd,
			    bool clear_hw_busy)
{
	struct neoisp_job_desc_s *job;
	int i;

	scoped_guard(spinlock_irqsave, &neoispd->hw_lock) {
		if (clear_hw_busy)
			neoispd->hw_busy = false;

		if (neoispd->hw_busy)
			return;

		job = list_first_entry_or_null(&neoispd->job_queue,
					       struct neoisp_job_desc_s,
					       queue);

		if (!job)
			return;

		list_del(&job->queue);

		for (i = 0; i < NEOISP_NODES_COUNT; i++)
			neoispd->queued_job.buf[i] = job->buffers[i];
		neoispd->queued_job.node_group = job->node_group;

		neoispd->hw_busy = true;
	}

	/*
	 * We can kick the job off without the hw_lock, as this can
	 * never run again until hw_busy is cleared, which will happen
	 * only when the following job has been queued and an interrupt
	 * is raised.
	 */
	neoisp_run_job(neoispd);
	kfree(job);
}

static void neoisp_node_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct neoisp_buffer_s *buffer = to_neoisp_buffer(vbuf);
	struct neoisp_node_s *node = vb2_get_drv_priv(vb->vb2_queue);
	struct neoisp_node_group_s *node_group = node->node_group;
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;

	dev_dbg(neoispd->dev, "%s: for node %s\n", __func__, NODE_NAME(node));
	list_add_tail(&buffer->ready_list, &node->ready_queue);

	/*
	 * Every time we add a buffer, check if there's now some work for the hw
	 * to do, but only for this client.
	 */
	if (!neoisp_prepare_job(node_group))
		neoisp_schedule(neoispd, false);
}

static void neoisp_set_default_context(struct neoisp_dev_s *neoispd, int ctx_id)
{
	/* Prepare node group's context with default one */
	neoisp_ctx_set_default_context(neoispd, neoispd->node_group[ctx_id].context);

	/*
	 * As i.MX95 only supports MSB-aligned data when reading buffers coming
	 * from ISI, reset the input data alignment to MSB for both input nodes
	 * (i.e. INALIGN0 and INALIGN1) to ensure proper data handling.
	 */
	if (neoispd->info->capabilities & NEO_CAP_ALIGNMENT_MSB) {
		struct neoisp_context_s *context = neoispd->node_group[ctx_id].context;
		struct neoisp_pipe_conf_s *pc = &context->hw.pipe_conf;
		u32 tmp;

		tmp = pc->img_conf &
			~(NEO_PIPE_CONF_IMG_CONF_CAM0_INALIGN0 |
			  NEO_PIPE_CONF_IMG_CONF_CAM0_INALIGN1);
		tmp |= NEO_PIPE_CONF_IMG_CONF_CAM0_INALIGN0_SET(1) |
			NEO_PIPE_CONF_IMG_CONF_CAM0_INALIGN1_SET(1);
		pc->img_conf = tmp;
	}
}

static int neoisp_prepare_node_streaming(struct neoisp_node_s *node)
{
	struct neoisp_node_group_s *node_group = node->node_group;
	struct neoisp_dev_s *neoispd = node_group->neoisp_dev;
	struct neoisp_context_s *ctx = node_group->context;
	struct neoisp_node_s *in0_node;
	u32 pixfmt = node->format.fmt.pix_mp.pixelformat;

	switch (node->id) {
	case NEOISP_INPUT0_NODE:
		/* Preload default parameters */
		neoisp_adjust_gain(ctx, node->neoisp_format->bit_depth);

		neoisp_ctx_update_head_color(neoispd, ctx, pixfmt);
		neoisp_ctx_update_monochrome_fmt(neoispd, ctx, pixfmt);
		break;

	case NEOISP_INPUT1_NODE:
		/* Prepare HDR mode */
		neoisp_ctx_update_hdr_mode(neoispd, ctx);
		break;

	case NEOISP_FRAME_NODE:
		in0_node = &node_group->node[NEOISP_INPUT0_NODE];

		if (node->format.fmt.pix_mp.width != in0_node->crop.width ||
		    node->format.fmt.pix_mp.height != in0_node->crop.height) {
			dev_err(neoispd->dev,
				"Crop & output sizes don't match - w/cw: %d/%d, h/ch : %d/%d\n",
				node->format.fmt.pix_mp.width, in0_node->crop.width,
				node->format.fmt.pix_mp.height, in0_node->crop.height);
			return -EPIPE;
		}

		neoisp_ctx_update_gcm(neoispd, ctx, &node->format.fmt.pix_mp,
				      node->neoisp_format->is_rgb ?
				      V4L2_YCBCR_ENC_DEFAULT : node->format.fmt.pix_mp.ycbcr_enc);
		break;
	}

	/*
	 * Check output modes (frame, ir, dummy or combination)
	 */
	if (!neoisp_node_link_is_enabled(&node_group->node[NEOISP_FRAME_NODE]) ||
	    !neoisp_node_link_is_enabled(&node_group->node[NEOISP_IR_NODE]) ||
	    format_is_monochrome(pixfmt)) {
		if (!node_group->dummy_buf) {
			struct neoisp_node_s *in0_node = &node_group->node[NEOISP_INPUT0_NODE];

			/* Allocate a single line dummy buffer as line stride is set to 0 */
			node_group->dummy_size = in0_node->crop.width * NEOISP_MAX_BPP;
			node_group->dummy_buf =
				dma_alloc_coherent(neoispd->dev,
						   node_group->dummy_size,
						   &node_group->dummy_dma, GFP_KERNEL);
		}
	}

	return 0;
}

static int neoisp_node_start_streaming(struct vb2_queue *q, u32 count)
{
	struct neoisp_node_s *node = vb2_get_drv_priv(q);
	struct neoisp_node_group_s *node_group = node->node_group;
	struct neoisp_dev_s *neoispd = node_group->neoisp_dev;
	struct neoisp_buffer_s *buf, *tmp;
	int ret;

	ret = pm_runtime_resume_and_get(neoispd->dev);
	if (ret < 0)
		goto error;

	ret = neoisp_prepare_node_streaming(node);
	if (ret < 0)
		goto error;

	scoped_guard(spinlock_irq, &neoispd->hw_lock) {
		node_group->streaming_map |= BIT(node->id);
		node_group->frame_sequence = 0;
	}

	dev_dbg(neoispd->dev, "%s: for node %s (count %u)\n",
		__func__, NODE_NAME(node), count);
	dev_dbg(neoispd->dev, "Nodes streaming for this group now 0x%x\n",
		node->node_group->streaming_map);

	/* Update queued job context with current driver configuration */
	neoisp_ctx_update_packetizer(node_group);
	neoisp_ctx_update_pipe_conf(node_group);

	/* Maybe we're ready to run. */
	if (!neoisp_prepare_job(node_group))
		neoisp_schedule(neoispd, false);

	return 0;

error:
	list_for_each_entry_safe(buf, tmp, &node->ready_queue, ready_list) {
		list_del(&buf->ready_list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	}
	return ret;
}

static void neoisp_node_stop_streaming(struct vb2_queue *q)
{
	struct neoisp_node_s *node = vb2_get_drv_priv(q);
	struct neoisp_node_group_s *node_group = node->node_group;
	struct neoisp_dev_s *neoispd = node_group->neoisp_dev;
	struct neoisp_job_desc_s *job, *temp;
	struct neoisp_buffer_s *buf;

	/*
	 * Now this is a bit awkward. In a simple M2M device we could just wait
	 * for all queued jobs to complete, but here there's a risk that a
	 * partial set of buffers was queued and cannot be run. For now, just
	 * cancel all buffers stuck in the "ready queue", then wait for any
	 * running job.
	 *
	 * This may return buffers out of order.
	 */
	dev_dbg(neoispd->dev, "%s: for node %s\n", __func__, NODE_NAME(node));
	do {
		buf = list_first_entry_or_null(&node->ready_queue,
					       struct neoisp_buffer_s,
					       ready_list);
		if (buf) {
			list_del(&buf->ready_list);
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		}
	} while (buf);

	vb2_wait_for_all_buffers(&node->queue);

	if (node->id == NEOISP_INPUT0_NODE)
		neoisp_set_default_context(neoispd, node_group->id);

	if (node_group->dummy_buf) {
		dma_free_coherent(neoispd->dev,
				  node_group->dummy_size,
				  node_group->dummy_buf,
				  node_group->dummy_dma);
		node_group->dummy_buf = NULL;
	}

	spin_lock_irq(&neoispd->hw_lock);
	node_group->streaming_map &= ~BIT(node->id);

	/*
	 * If all nodes have stopped streaming release all jobs
	 * belonging to the node group immediately.
	 */
	list_for_each_entry_safe(job, temp, &neoispd->job_queue, queue) {
		if (job->node_group == node_group) {
			list_del(&job->queue);
			kfree(job);
		}
	}
	spin_unlock_irq(&neoispd->hw_lock);

	pm_runtime_mark_last_busy(neoispd->dev);
	pm_runtime_put_autosuspend(neoispd->dev);

	dev_dbg(neoispd->dev, "Nodes streaming for this group now 0x%x\n",
		node_group->streaming_map);
}

static const struct vb2_ops neoisp_params_node_queue_ops = {
	.queue_setup = neoisp_node_queue_setup,
	.buf_prepare = neoisp_params_node_buf_prepare,
	.buf_queue = neoisp_node_buf_queue,
	.start_streaming = neoisp_node_start_streaming,
	.stop_streaming = neoisp_node_stop_streaming,
};

static const struct vb2_ops neoisp_node_queue_ops = {
	.queue_setup = neoisp_node_queue_setup,
	.buf_prepare = neoisp_node_buf_prepare,
	.buf_queue = neoisp_node_buf_queue,
	.start_streaming = neoisp_node_start_streaming,
	.stop_streaming = neoisp_node_stop_streaming,
};

static const struct v4l2_file_operations neoisp_fops = {
	.owner          = THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = vb2_fop_release,
	.poll           = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = vb2_fop_mmap
};

static int neoisp_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct neoisp_node_s *node = video_drvdata(file);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;

	strscpy(cap->driver, NEOISP_NAME, sizeof(cap->driver));
	strscpy(cap->card, NEOISP_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(neoispd->dev));

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
		V4L2_CAP_VIDEO_OUTPUT_MPLANE |
		V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS |
		V4L2_CAP_META_OUTPUT | V4L2_CAP_META_CAPTURE;
	cap->device_caps = node->vfd.device_caps;

	dev_dbg(neoispd->dev, "Caps for node %s: %x and %x (dev %x)\n",
		NODE_NAME(node), cap->capabilities, cap->device_caps,
		node->vfd.device_caps);

	return 0;
}

static int neoisp_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	struct neoisp_node_s *node = video_drvdata(file);

	if (f->type != node->queue.type)
		return -EINVAL;

	f->flags = 0;
	if (node_is_meta(node)) {
		if (node_is_output(node)) {
			if (f->index >= ARRAY_SIZE(formats_mout))
				return -EINVAL;

			f->pixelformat = formats_mout[f->index].fourcc;
		} else {
			if (f->index >= ARRAY_SIZE(formats_mcap))
				return -EINVAL;

			f->pixelformat = formats_mcap[f->index].fourcc;
		}
		return 0;
	}
	if (node_is_output(node)) {
		if (f->index >= ARRAY_SIZE(formats_vout))
			return -EINVAL;

		f->pixelformat = formats_vout[f->index].fourcc;
	} else {
		if (node->id == NEOISP_IR_NODE) {
			if (f->index >= ARRAY_SIZE(formats_vcap_ir))
				return -EINVAL;

			f->pixelformat = formats_vcap_ir[f->index].fourcc;
		} else {
			if (f->index >= ARRAY_SIZE(formats_vcap))
				return -EINVAL;

			f->pixelformat = formats_vcap[f->index].fourcc;
		}
	}

	return 0;
}

static int neoisp_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct neoisp_node_s *node = video_drvdata(file);
	const struct neoisp_fmt_s *fmt;

	if (fsize->index)
		return -EINVAL;

	fmt = neoisp_find_pixel_format_by_node(fsize->pixel_format, node);
	if (!fmt)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise = neoisp_frmsize_stepwise;

	return 0;
}

static int neoisp_g_fmt_meta(struct file *file, void *priv, struct v4l2_format *f)
{
	struct neoisp_node_s *node = video_drvdata(file);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;

	if (!node_is_meta(node)) {
		dev_err(neoispd->dev,
			"Cannot get meta fmt for video node %s\n", NODE_NAME(node));
		return -EINVAL;
	}
	*f = node->format;
	dev_dbg(neoispd->dev, "Get meta format for node %s\n", NODE_NAME(node));
	return 0;
}

static int neoisp_try_fmt(struct v4l2_format *f, struct neoisp_node_s *node)
{
	const struct neoisp_fmt_s *fmt;
	u32 pixfmt = f->fmt.pix_mp.pixelformat;
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;

	if (node_is_meta(node)) {
		pixfmt = f->fmt.meta.dataformat;

		if (node_is_output(node) &&
		    pixfmt != V4L2_META_FMT_NEO_ISP_EXT_PARAMS)
			f->fmt.meta.dataformat = V4L2_META_FMT_NEO_ISP_EXT_PARAMS;
		else if (!node_is_output(node) &&
			 pixfmt != V4L2_META_FMT_NEO_ISP_EXT_STATS)
			f->fmt.meta.dataformat = V4L2_META_FMT_NEO_ISP_EXT_STATS;

		return 0;
	}

	fmt = neoisp_find_pixel_format_by_node(pixfmt, node);
	if (!fmt) {
		if (node_is_output(node))
			fmt = &formats_vout[0];
		else
			if (node->id == NEOISP_IR_NODE)
				fmt = &formats_vcap_ir[0];
			else
				fmt = &formats_vcap[0];
	}

	f->fmt.pix_mp.pixelformat = fmt->fourcc;
	f->fmt.pix_mp.num_planes = fmt->num_planes;
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;

	if (f->fmt.pix_mp.width % 16 != 0 || f->fmt.pix_mp.height % 2 != 0) {
		dev_warn(neoispd->dev,
			 "Width and height must be a multiple of 16 and 2 respectively\n");
		/* Round width and height to their respective nearest multiple */
		f->fmt.pix_mp.width = (f->fmt.pix_mp.width + 8) / 16 * 16;
		f->fmt.pix_mp.height = (f->fmt.pix_mp.height + 1) / 2 * 2;
	}
	f->fmt.pix_mp.width = clamp(f->fmt.pix_mp.width, NEOISP_MIN_W, NEOISP_MAX_W);
	f->fmt.pix_mp.height = clamp(f->fmt.pix_mp.height, NEOISP_MIN_H, NEOISP_MAX_H);

	/*
	 * Fill in the actual color space when the requested one was
	 * not supported. This also catches the case when the "default"
	 * color space was requested (as that's never in the mask).
	 */
	if (!(NEOISP_COLORSPACE_MASK(f->fmt.pix_mp.colorspace) &
	    fmt->colorspace_mask))
		f->fmt.pix_mp.colorspace = fmt->colorspace_default;

	/* In all cases, we only support the defaults for these: */
	f->fmt.pix_mp.ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(f->fmt.pix_mp.colorspace);
	f->fmt.pix_mp.xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(f->fmt.pix_mp.colorspace);

	f->fmt.pix_mp.quantization =
		V4L2_MAP_QUANTIZATION_DEFAULT(fmt->is_rgb, f->fmt.pix_mp.colorspace,
					      f->fmt.pix_mp.ycbcr_enc);

	/* Set plane size and bytes/line for each plane. */
	neoisp_fill_mp(f, fmt);

	return 0;
}

static int neoisp_try_fmt_meta_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct neoisp_node_s *node = video_drvdata(file);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;

	if (!node_is_meta(node) || node_is_capture(node)) {
		dev_err(neoispd->dev,
			"Cannot set capture fmt for meta output node %s\n",
			NODE_NAME(node));
		return -EINVAL;
	}

	if (!f->fmt.meta.buffersize)
		f->fmt.meta.buffersize = v4l2_isp_buffer_size(NEOISP_EXT_PARAMS_MAX_SIZE);

	return neoisp_try_fmt(f, node);
}

static int neoisp_try_fmt_meta_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct neoisp_node_s *node = video_drvdata(file);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;

	if (!node_is_meta(node) || node_is_output(node)) {
		dev_err(neoispd->dev,
			"Cannot set capture fmt for meta output node %s\n",
			NODE_NAME(node));
		return -EINVAL;
	}

	if (!f->fmt.meta.buffersize)
		f->fmt.meta.buffersize = v4l2_isp_buffer_size(NEOISP_EXT_STATS_MAX_SIZE);

	return neoisp_try_fmt(f, node);
}

static int neoisp_s_fmt_meta_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct neoisp_node_s *node = video_drvdata(file);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;
	int ret;

	ret = neoisp_try_fmt_meta_out(file, priv, f);
	if (ret < 0)
		return ret;

	if (vb2_is_busy(&node->queue))
		return -EBUSY;

	node->format = *f;
	node->neoisp_format =
		neoisp_find_pixel_format_by_node(f->fmt.meta.dataformat, node);

	dev_dbg(neoispd->dev,
		"Set output format for meta node %s to %x\n",
		NODE_NAME(node),
		f->fmt.meta.dataformat);

	return 0;
}

static int neoisp_s_fmt_meta_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct neoisp_node_s *node = video_drvdata(file);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;
	int ret;

	ret = neoisp_try_fmt_meta_cap(file, priv, f);
	if (ret < 0)
		return ret;

	if (vb2_is_busy(&node->queue))
		return -EBUSY;

	node->format = *f;
	node->neoisp_format =
		neoisp_find_pixel_format_by_node(f->fmt.meta.dataformat, node);

	dev_dbg(neoispd->dev,
		"Set capture format for meta node %s to %x\n",
		NODE_NAME(node),
		f->fmt.meta.dataformat);

	return 0;
}

static int neoisp_g_fmt_vid(struct file *file, void *priv, struct v4l2_format *f)
{
	struct neoisp_node_s *node = video_drvdata(file);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;

	if (node_is_meta(node)) {
		dev_err(neoispd->dev,
			"Cannot get video fmt for meta node %s\n", NODE_NAME(node));
		return -EINVAL;
	}

	*f = node->format;

	dev_dbg(neoispd->dev, "Get video format for node %s\n",
		NODE_NAME(node));

	return 0;
}

static int neoisp_try_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct neoisp_node_s *node = video_drvdata(file);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;

	if (!node_is_capture(node) || node_is_meta(node)) {
		dev_err(neoispd->dev,
			"Cannot set capture fmt for output node %s\n", NODE_NAME(node));
		return -EINVAL;
	}

	return neoisp_try_fmt(f, node);
}

static int neoisp_s_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct neoisp_node_s *node = video_drvdata(file);
	int ret;

	ret = neoisp_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	if (vb2_is_busy(&node->queue))
		return -EBUSY;

	node->format = *f;
	node->neoisp_format =
		neoisp_find_pixel_format_by_node(f->fmt.pix_mp.pixelformat, node);

	return 0;
}

static int neoisp_try_fmt_vid_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct neoisp_node_s *node = video_drvdata(file);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;

	if (!node_is_output(node) || node_is_meta(node)) {
		dev_err(neoispd->dev,
			"Cannot set capture fmt for output node %s\n",
			NODE_NAME(node));
		return -EINVAL;
	}

	return neoisp_try_fmt(f, node);
}

static int neoisp_s_fmt_vid_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct neoisp_node_s *node = video_drvdata(file);
	struct neoisp_dev_s *neoispd = node->node_group->neoisp_dev;
	int ret = neoisp_try_fmt_vid_out(file, priv, f);

	if (ret < 0)
		return ret;

	if (vb2_is_busy(&node->queue))
		return -EBUSY;

	node->format = *f;
	node->neoisp_format =
		neoisp_find_pixel_format_by_node(f->fmt.pix_mp.pixelformat, node);

	node->crop.top = 0;
	node->crop.left = 0;
	node->crop.width = f->fmt.pix_mp.width;
	node->crop.height = f->fmt.pix_mp.height;
	dev_dbg(neoispd->dev,
		"Set output format for node %s to %x\n",
		NODE_NAME(node),
		f->fmt.pix_mp.pixelformat);

	return 0;
}

static int neoisp_g_selection(struct file *file, void *fh, struct v4l2_selection *sel)
{
	struct neoisp_node_s *node = video_drvdata(file);

	if (sel->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = node->format.fmt.pix_mp.width;
		sel->r.height = node->format.fmt.pix_mp.height;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r.top = node->crop.top;
		sel->r.left = node->crop.left;
		sel->r.width = node->crop.width;
		sel->r.height = node->crop.height;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int neoisp_s_selection(struct file *file, void *fh, struct v4l2_selection *sel)
{
	struct neoisp_node_s *node = video_drvdata(file);
	u32 winput, hinput;

	if (sel->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	dev_dbg(node->node_group->neoisp_dev->dev,
		">>> Buffer Type: %u Target: %u Rect: %ux%u@%d.%d\n",
		sel->type, sel->target,
		sel->r.width, sel->r.height, sel->r.left, sel->r.top);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		winput = node->format.fmt.pix_mp.width;
		hinput = node->format.fmt.pix_mp.height;

		/* Left and width should be multiple of 16 */
		sel->r.left = (sel->r.left / 16) * 16;
		sel->r.width = (sel->r.width / 16) * 16;
		/* Top and height should be even */
		sel->r.top = (sel->r.top / 2) * 2;
		sel->r.height = (sel->r.height / 2) * 2;

		sel->r.top = clamp_t(int, sel->r.top, 0, hinput - NEOISP_MIN_H);
		sel->r.left = clamp_t(int, sel->r.left, 0, winput - NEOISP_MIN_W);
		sel->r.width = clamp(sel->r.width, NEOISP_MIN_W, winput - sel->r.left);
		sel->r.height = clamp(sel->r.height, NEOISP_MIN_H, hinput - sel->r.top);

		node->crop.top = sel->r.top;
		node->crop.left = sel->r.left;
		node->crop.width = sel->r.width;
		node->crop.height = sel->r.height;
		break;

	default:
		return -EINVAL;
	}

	dev_dbg(node->node_group->neoisp_dev->dev,
		"<<< Buffer Type: %u Target: %u Rect: %ux%u@%d.%d\n",
		sel->type, sel->target,
		sel->r.width, sel->r.height, sel->r.left, sel->r.top);

	return 0;
}

static const struct v4l2_ioctl_ops neoisp_ioctl_ops = {
	.vidioc_querycap		= neoisp_querycap,

	.vidioc_enum_fmt_vid_cap	= neoisp_enum_fmt,
	.vidioc_enum_fmt_meta_cap	= neoisp_enum_fmt,
	.vidioc_enum_framesizes		= neoisp_enum_framesizes,
	.vidioc_g_fmt_vid_cap_mplane	= neoisp_g_fmt_vid,
	.vidioc_s_fmt_vid_cap_mplane	= neoisp_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap_mplane	= neoisp_try_fmt_vid_cap,
	.vidioc_g_fmt_meta_cap		= neoisp_g_fmt_meta,
	.vidioc_s_fmt_meta_cap		= neoisp_s_fmt_meta_cap,
	.vidioc_try_fmt_meta_cap	= neoisp_try_fmt_meta_cap,

	.vidioc_enum_fmt_vid_out	= neoisp_enum_fmt,
	.vidioc_enum_fmt_meta_out	= neoisp_enum_fmt,
	.vidioc_g_fmt_vid_out_mplane	= neoisp_g_fmt_vid,
	.vidioc_s_fmt_vid_out_mplane	= neoisp_s_fmt_vid_out,
	.vidioc_try_fmt_vid_out_mplane	= neoisp_try_fmt_vid_out,
	.vidioc_g_fmt_meta_out		= neoisp_g_fmt_meta,
	.vidioc_s_fmt_meta_out		= neoisp_s_fmt_meta_out,
	.vidioc_try_fmt_meta_out	= neoisp_try_fmt_meta_out,

	.vidioc_g_selection		= neoisp_g_selection,
	.vidioc_s_selection		= neoisp_s_selection,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_expbuf			= vb2_ioctl_expbuf,

	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

static const struct video_device neoisp_videodev = {
	.name = NEOISP_NAME,
	.vfl_dir = VFL_DIR_M2M,
	.fops = &neoisp_fops,
	.ioctl_ops = &neoisp_ioctl_ops,
	.minor = -1,
	.release = video_device_release_empty,
};

static struct v4l2_ctrl_config controls[] = {
	[NEOISP_CTRLS_QUERYCAP] = {
		.id = V4L2_CID_NEOISP_QUERYCAP,
		.name = "Neoisp custom capabilities",
		.type = V4L2_CTRL_TYPE_BITMASK,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
	},
	[NEOISP_CTRLS_SUPPORTED_PARAMS_BLOCKS] = {
		.id = V4L2_CID_NEOISP_SUPPORTED_PARAMS_BLOCKS,
		.name = "Neoisp supported params blocks",
		.type = V4L2_CTRL_TYPE_BITMASK,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
	},
};

static irqreturn_t neoisp_irq_handler(int irq, void *dev_id)
{
	struct neoisp_dev_s *neoispd = (struct neoisp_dev_s *)dev_id;
	struct neoisp_buffer_s **buf = neoispd->queued_job.buf;
	struct neoisp_node_group_s *node_group = neoispd->queued_job.node_group;
	u64 ts = ktime_get_ns();
	u32 irq_status = 0;
	u32 irq_clear = 0;
	bool done = false;
	int i;

	irq_status = neoisp_rd(neoispd, NEO_PIPE_CONF_INT_STAT0);

	if (irq_status & NEO_PIPE_CONF_INT_STAT0_S_FS1) {
		dev_dbg(neoispd->dev, "Neo IRQ FS1 !\n");
		irq_clear |= NEO_PIPE_CONF_INT_STAT0_S_FS1;
		done = false;
	}

	if (irq_status & NEO_PIPE_CONF_INT_STAT0_S_FS2) {
		dev_dbg(neoispd->dev, "Neo IRQ FS2 !\n");
		irq_clear |= NEO_PIPE_CONF_INT_STAT0_S_FS2;
		done = false;
	}

	if (irq_status & NEO_PIPE_CONF_INT_STAT0_S_FD1) {
		dev_dbg(neoispd->dev, "Neo IRQ FD1 !\n");
		irq_clear |= NEO_PIPE_CONF_INT_STAT0_S_FD1;
		done = false;
	}

	if (irq_status & NEO_PIPE_CONF_INT_STAT0_S_STATD) {
		dev_dbg(neoispd->dev, "Neo IRQ STATD !\n");
		irq_clear |= NEO_PIPE_CONF_INT_STAT0_S_STATD;
		done = false;
	}

	if (irq_status & NEO_PIPE_CONF_INT_STAT0_S_DRCD) {
		dev_dbg(neoispd->dev, "Neo IRQ DRCD !\n");
		neoisp_ctx_get_stats(neoispd, buf[NEOISP_STATS_NODE]);
		irq_clear |= NEO_PIPE_CONF_INT_STAT0_S_DRCD;
		done = false;
	}

	if (irq_status & NEO_PIPE_CONF_INT_STAT0_S_BUS_ERR_MASK) {
		irq_clear |= NEO_PIPE_CONF_INT_STAT0_S_BUS_ERR_MASK;
		dev_err(neoispd->dev, "Neo IRQ BUS ERR!\n");
		done = true;
	}

	if (irq_status & NEO_PIPE_CONF_INT_STAT0_S_TRIG_ERR) {
		dev_err(neoispd->dev, "Neo IRQ TRIG ERR !\n");
		irq_clear |= NEO_PIPE_CONF_INT_STAT0_S_TRIG_ERR;
		done = true;
	}

	if (irq_status & NEO_PIPE_CONF_INT_STAT0_S_CSI_TERR) {
		dev_err(neoispd->dev, "Neo IRQ TRIG CSI Trigger ERR !\n");
		irq_clear |= NEO_PIPE_CONF_INT_STAT0_S_CSI_TERR;
		done = true;
	}

	if (irq_status & NEO_PIPE_CONF_INT_STAT0_S_FD2) {
		dev_dbg(neoispd->dev, "Neo IRQ FD2 !\n");
		irq_clear |= NEO_PIPE_CONF_INT_STAT0_S_FD2;
		done = true;
	}

	if (irq_status & NEO_PIPE_CONF_INT_STAT0_BUSY)
		dev_err(neoispd->dev, "Neo is busy !\n");

	neoisp_wr(neoispd, NEO_PIPE_CONF_INT_STAT0, irq_clear);

	if (done) {
		for (i = 0; i < NEOISP_NODES_COUNT; i++) {
			if (buf[i]) {
				buf[i]->vb.sequence = node_group->frame_sequence;
				buf[i]->vb.vb2_buf.timestamp = ts;
				vb2_buffer_done(&buf[i]->vb.vb2_buf, VB2_BUF_STATE_DONE);
			}
		}
		/* Update frame_sequence */
		node_group->frame_sequence++;
		/* Check if there's more to do before going to sleep */
		neoisp_schedule(neoispd, true);
	}

	return IRQ_HANDLED;
}

static int neoisp_sd_subs_evt(struct v4l2_subdev *sd, struct v4l2_fh *fh,
			      struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_FRAME_SYNC:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_subdev_core_ops neoisp_sd_core_ops = {
	.subscribe_event = neoisp_sd_subs_evt,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_pad_ops neoisp_sd_pad_ops = {
	.link_validate = v4l2_subdev_link_validate_default,
};

static const struct v4l2_subdev_ops neoisp_sd_ops = {
	.core = &neoisp_sd_core_ops,
	.pad = &neoisp_sd_pad_ops,
};

static int neoisp_init_subdev(struct neoisp_node_group_s *node_group)
{
	struct neoisp_dev_s *neoispd = node_group->neoisp_dev;
	struct v4l2_subdev *sd = &node_group->sd;
	struct v4l2_ctrl_config *control_cfg;
	struct v4l2_ctrl_handler *hdl;
	u32 i;
	int ret;

	v4l2_subdev_init(sd, &neoisp_sd_ops);
	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_ISP;
	sd->owner = THIS_MODULE;
	sd->dev = neoispd->dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	snprintf(sd->name, sizeof(sd->name), "%s-%d", NEOISP_NAME, node_group->id);

	for (i = 0; i < NEOISP_NODES_COUNT; i++)
		node_group->pad[i].flags =
			node_desc_is_output(&node_desc[i]) ?
			MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, NEOISP_NODES_COUNT, node_group->pad);
	if (ret)
		goto error;

	/* Prepare Supported Params Block control */
	control_cfg = &controls[NEOISP_CTRLS_SUPPORTED_PARAMS_BLOCKS];
	i = 0;
	while (neoispd->info->blocks_list[i] != -1 && i < 64U)
		control_cfg->max |= BIT(neoispd->info->blocks_list[i++]);

	control_cfg->def = control_cfg->max;

	/* Prepare HW capabilities mask */
	control_cfg = &controls[NEOISP_CTRLS_QUERYCAP];
	control_cfg->max = NEO_CAP_ALIGNMENT_MSB;
	control_cfg->def = neoispd->info->capabilities;

	/* Create custom controls */
	hdl = &node_group->hdl;
	v4l2_ctrl_handler_init(hdl, ARRAY_SIZE(controls));
	for (i = 0; i < ARRAY_SIZE(controls); i++) {
		node_group->ctrls[i] = v4l2_ctrl_new_custom(hdl, &controls[i], NULL);
		if (hdl->error) {
			dev_err(neoispd->dev, "Adding control (%d) failed\n", i);
			ret = hdl->error;
			goto error;
		}
	}
	sd->ctrl_handler = hdl;

	ret = v4l2_device_register_subdev(&node_group->v4l2_dev, sd);
	if (ret)
		goto error;

	return 0;

error:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static void node_set_default_format(struct neoisp_node_s *node)
{
	if (node_is_meta(node) && node_is_output(node)) {
		/* Params node - exensible format */
		struct v4l2_format *f = &node->format;

		f->fmt.meta.dataformat = V4L2_META_FMT_NEO_ISP_EXT_PARAMS;
		f->fmt.meta.buffersize = v4l2_isp_buffer_size(NEOISP_EXT_PARAMS_MAX_SIZE);
		f->type = node->buf_type;
	} else if (node_is_meta(node) && node_is_capture(node)) {
		/* Stats node - legacy format */
		struct v4l2_format *f = &node->format;

		f->fmt.meta.dataformat = V4L2_META_FMT_NEO_ISP_EXT_STATS;
		f->fmt.meta.buffersize = v4l2_isp_buffer_size(NEOISP_EXT_STATS_MAX_SIZE);
		f->type = node->buf_type;
	} else {
		struct v4l2_format f = {0};

		if (node_is_capture(node))
			f.fmt.pix_mp.pixelformat = formats_vcap[0].fourcc;
		else
			f.fmt.pix_mp.pixelformat = formats_vout[0].fourcc;

		f.fmt.pix_mp.width = NEOISP_DEF_W;
		f.fmt.pix_mp.height = NEOISP_DEF_H;
		f.type = node->buf_type;
		neoisp_try_fmt(&f, node);
		node->format = f;
	}
	node->crop.width = NEOISP_DEF_W;
	node->crop.height = NEOISP_DEF_H;

	node->neoisp_format =
		neoisp_find_pixel_format_by_node(node->format.fmt.pix_mp.pixelformat, node);
}

/*
 * Initialise a struct neoisp_node_s and register it as /dev/video<N>
 * to represent one of the neoisp's input or output streams.
 */
static int neoisp_init_node(struct neoisp_node_group_s *node_group, u32 id)
{
	bool output = node_desc_is_output(&node_desc[id]);
	struct neoisp_node_s *node = &node_group->node[id];
	struct neoisp_dev_s *neoispd = node_group->neoisp_dev;
	struct media_entity *entity = &node->vfd.entity;
	struct media_pad *mpad;
	struct video_device *vdev = &node->vfd;
	struct vb2_queue *q = &node->queue;
	int ret;

	node->id = id;
	node->node_group = node_group;
	node->buf_type = node_desc[id].buf_type;

	mutex_init(&node->node_lock);
	mutex_init(&node->queue_lock);
	INIT_LIST_HEAD(&node->ready_queue);

	node->format.type = node->buf_type;
	node_set_default_format(node);

	q->type = node->buf_type;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->mem_ops = &vb2_dma_contig_memops;
	q->drv_priv = node;
	if (node->id == NEOISP_PARAMS_NODE)
		q->ops = &neoisp_params_node_queue_ops;
	else
		q->ops = &neoisp_node_queue_ops;

	q->buf_struct_size = sizeof(struct neoisp_buffer_s);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->dev = neoispd->dev;
	/* Get V4L2 to handle node->queue locking */
	q->lock = &node->queue_lock;

	ret = vb2_queue_init(q);
	if (ret < 0) {
		dev_err(neoispd->dev, "vb2_queue_init failed\n");
		return ret;
	}

	*vdev = neoisp_videodev; /* Default initialization */
	snprintf(vdev->name, sizeof(vdev->name), "%s-%d",
		 node_desc[id].ent_name, node_group->id);
	vdev->v4l2_dev = &node_group->v4l2_dev;
	vdev->vfl_dir = output ? VFL_DIR_TX : VFL_DIR_RX;
	/* Get V4L2 to serialise our ioctls */
	vdev->lock = &node->node_lock;
	vdev->queue = &node->queue;
	vdev->device_caps = V4L2_CAP_STREAMING | node_desc[id].caps;

	node->pad.flags = output ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(entity, 1, &node->pad);
	if (ret) {
		dev_err(neoispd->dev,
			"Failed to register media pads for %s device node\n",
			NODE_NAME(node));
		goto err_unregister_queue;
	}

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(neoispd->dev,
			"Failed to register video %s device node\n",
			NODE_NAME(node));
		goto err_unregister_queue;
	}
	video_set_drvdata(vdev, node);

	if (output)
		ret = media_create_pad_link(entity, 0, &node_group->sd.entity,
					    id, node_desc[id].link_flags);
	else
		ret = media_create_pad_link(&node_group->sd.entity, id, entity,
					    0, node_desc[id].link_flags);
	if (ret)
		goto err_unregister_video_dev;

	media_entity_for_each_pad(&node_group->sd.entity, mpad)
		if (mpad->index == id)
			break;
	if (output)
		node->intf_link = media_entity_find_link(&node->pad, mpad);
	else
		node->intf_link = media_entity_find_link(mpad, &node->pad);

	dev_dbg(neoispd->dev,
		"%s device node registered as /dev/video%d\n",
		NODE_NAME(node), node->vfd.num);

	return 0;

err_unregister_video_dev:
	video_unregister_device(&node->vfd);
err_unregister_queue:
	vb2_queue_release(&node->queue);
	return ret;
}

static int neoisp_init_node_group(struct neoisp_dev_s *neoispd,
				  struct media_device *mdev, u32 id)
{
	struct neoisp_node_group_s *node_group = &neoispd->node_group[id];
	struct v4l2_device *v4l2_dev = &node_group->v4l2_dev;
	u32 num_registered = 0;
	int ret;

	dev_dbg(neoispd->dev, "Register nodes for group %u\n", id);

	node_group->id = id;
	node_group->neoisp_dev = neoispd;

	/* Register v4l2_device and media_device */
	v4l2_dev->mdev = mdev;
	strscpy(v4l2_dev->name, NEOISP_NAME, sizeof(v4l2_dev->name));

	/* Register the NEOISP subdevice. */
	ret = neoisp_init_subdev(node_group);
	if (ret)
		goto err_unregister_v4l2;

	/* Create device video nodes */
	for (; num_registered < NEOISP_NODES_COUNT; num_registered++) {
		ret = neoisp_init_node(node_group, num_registered);
		if (ret)
			goto err_unregister_nodes;
	}

	ret = v4l2_device_register_subdev_nodes(v4l2_dev);
	if (ret)
		goto err_unregister_nodes;

	return 0;

err_unregister_nodes:
	media_entity_cleanup(&node_group->sd.entity);
	while (num_registered-- > 0) {
		video_unregister_device(&node_group->node[num_registered].vfd);
		vb2_queue_release(&node_group->node[num_registered].queue);
	}
	v4l2_device_unregister_subdev(&node_group->sd);
err_unregister_v4l2:
	v4l2_device_unregister(v4l2_dev);
	return ret;
}

static void neoisp_destroy_node_group(struct neoisp_node_group_s *node_group)
{
	struct neoisp_dev_s *neoispd = node_group->neoisp_dev;
	int i;

	if (node_group->context) {
		dma_free_coherent(neoispd->dev,
				  sizeof(struct neoisp_context_s),
				  node_group->context,
				  node_group->params_dma_addr);
	}

	if (standalone_mdev)
		media_device_unregister(&node_group->mdev);
	else if (!neoispd->media_registered)
		return;

	v4l2_device_unregister_subdev(&node_group->sd);
	media_entity_cleanup(&node_group->sd.entity);

	for (i = NEOISP_NODES_COUNT - 1; i >= 0; i--) {
		video_unregister_device(&node_group->node[i].vfd);
		vb2_queue_release(&node_group->node[i].queue);
	}

	v4l2_device_unregister(&node_group->v4l2_dev);
}

int neoisp_core_media_register(struct device *dev, struct v4l2_subdev *sd)
{
	struct neoisp_dev_s *neoispd = dev_get_drvdata(dev);
	struct media_device *mdev = sd->v4l2_dev->mdev;
	u32 num_registered = 0;
	int ret;

	if (!neoispd)
		return -EINVAL;

	if (neoispd->media_registered || standalone_mdev)
		return 0;

	for (; num_registered < NEOISP_NODE_GROUPS_COUNT; num_registered++) {
		ret = neoisp_init_node_group(neoispd, mdev, num_registered);
		if (ret)
			goto err_unregister_groups;
	}

	neoispd->media_registered++;
	return 0;

err_unregister_groups:
	while (num_registered-- > 0)
		neoisp_destroy_node_group(&neoispd->node_group[num_registered]);

	return ret;
}
EXPORT_SYMBOL_GPL(neoisp_core_media_register);

static int neoisp_init_devices(struct neoisp_dev_s *neoispd, u32 id)
{
	struct neoisp_node_group_s *node_group = &neoispd->node_group[id];
	struct v4l2_device *v4l2_dev;
	struct media_device *mdev;
	int ret;

	v4l2_dev = &node_group->v4l2_dev;
	strscpy(v4l2_dev->name, NEOISP_NAME, sizeof(v4l2_dev->name));

	ret = v4l2_device_register(neoispd->dev, v4l2_dev);
	if (ret)
		return ret;

	node_group->streaming_map = 0;
	node_group->dummy_buf = NULL;
	node_group->context = dma_alloc_coherent(neoispd->dev,
						 sizeof(struct neoisp_context_s),
						 &node_group->params_dma_addr, GFP_KERNEL);
	if (!node_group->context) {
		dev_err(neoispd->dev, "Unable to allocate cached context buffers.\n");
		ret = -ENOMEM;
		goto err_unregister_v4l2;
	}

	if (!standalone_mdev)
		return 0;

	/* Prepare neoisp media device in standalone mode only */
	mdev = &node_group->mdev;
	mdev->dev = neoispd->dev;
	strscpy(mdev->model, NEOISP_NAME, sizeof(mdev->model));
	snprintf(mdev->bus_info, sizeof(mdev->bus_info),
		 "platform:%s", dev_name(neoispd->dev));
	media_device_init(mdev);

	ret = neoisp_init_node_group(neoispd, mdev, id);
	if (ret)
		goto err_group;

	ret = media_device_register(mdev);
	if (ret)
		goto err_media;

	return 0;

err_media:
	neoisp_destroy_node_group(node_group);
err_group:
	media_device_cleanup(mdev);
err_unregister_v4l2:
	v4l2_device_unregister(v4l2_dev);
	return ret;
}

static void neoisp_init_groups_context(struct neoisp_dev_s *neoispd)
{
	int i;

	for (i = 0; i < NEOISP_NODE_GROUPS_COUNT; i++)
		neoisp_set_default_context(neoispd, i);
}

static void neoisp_init_hw(struct neoisp_dev_s *neoispd)
{
	u32 val;

	neoisp_reset_hw(neoispd, false);
	neoisp_reset_hw(neoispd, true);

	/* Disable bus error if eDMA transfer is used */
	neoisp_wr(neoispd, NEO_PIPE_CONF_REG_XFR_DIS, NEO_PIPE_CONF_REG_XFR_DIS_XFR_ERR_DIS);

	/* Disable debug */
	neoisp_wr(neoispd, NEO_IDBG1_LINE_NUM, NEO_IDBG1_LINE_NUM_LINE_NUM_MASK);
	neoisp_wr(neoispd, NEO_IDBG2_LINE_NUM, NEO_IDBG2_LINE_NUM_LINE_NUM_MASK);

	/* Enable interrupts */
	val = NEO_PIPE_CONF_INT_EN0_EN_FD2 |
		NEO_PIPE_CONF_INT_EN0_EN_DRCD |
		NEO_PIPE_CONF_INT_EN0_EN_BUS_ERR_MASK |
		NEO_PIPE_CONF_INT_EN0_EN_CSI_TERR |
		NEO_PIPE_CONF_INT_EN0_EN_TRIG_ERR;
	neoisp_wr(neoispd, NEO_PIPE_CONF_INT_EN0, val);
}

static int neoisp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct neoisp_dev_s *neoispd;
	int num_groups, ret, irq;

	neoispd = devm_kzalloc(dev, sizeof(*neoispd), GFP_KERNEL);
	if (!neoispd)
		return -ENOMEM;

	INIT_LIST_HEAD(&neoispd->job_queue);

	neoispd->dev = dev;
	neoispd->info = (struct neoisp_info_s *)of_device_get_match_data(dev);

	ret = devm_clk_bulk_get_all(dev, &neoispd->clks);
	if (ret < 0) {
		dev_err(dev, "Unable to get clocks: %d\n", ret);
		return ret;
	}
	neoispd->num_clks = ret;

	/* Get regs address */
	neoispd->mmio = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(neoispd->mmio))
		return PTR_ERR(neoispd->mmio);

	/* Get internal isp memory address */
	neoispd->mmio_tcm = devm_platform_get_and_ioremap_resource(pdev, 1, NULL);
	if (IS_ERR(neoispd->mmio_tcm))
		return PTR_ERR(neoispd->mmio_tcm);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	platform_set_drvdata(pdev, neoispd);

	pm_runtime_set_autosuspend_delay(dev, NEOISP_SUSPEND_TIMEOUT_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err(dev, "Unable to resume the device: %d\n", ret);
		goto err_pm;
	}

	ret = devm_request_irq(dev, irq, neoisp_irq_handler, 0,
			       dev_name(dev), neoispd);
	if (ret < 0) {
		dev_err(dev, "Failed to request irq: %d\n", ret);
		goto err_pm;
	}

	/*
	 * Initialise and register devices for each node_group, including media
	 * device
	 */
	for (num_groups = 0; num_groups < NEOISP_NODE_GROUPS_COUNT; num_groups++) {
		ret = neoisp_init_devices(neoispd, num_groups);
		if (ret)
			goto disable_nodes_err;
	}

	spin_lock_init(&neoispd->hw_lock);
	neoisp_init_hw(neoispd);
	neoisp_init_groups_context(neoispd);

	if (enable_debugfs) {
		neoisp_debugfs_init(neoispd);
		/* Increase pm_runtime counter to prevent suspend */
		pm_runtime_resume_and_get(dev);
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	dev_dbg(dev, "probe: done (%d) debugfs (%x)\n", ret, enable_debugfs);
	return 0;

disable_nodes_err:
	while (num_groups-- > 0)
		neoisp_destroy_node_group(&neoispd->node_group[num_groups]);
err_pm:
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);

	dev_err(dev, "probe: error %d\n", ret);
	return ret;
}

static void neoisp_remove(struct platform_device *pdev)
{
	struct neoisp_dev_s *neoispd = platform_get_drvdata(pdev);
	int i;

	if (enable_debugfs)
		neoisp_debugfs_exit(neoispd);

	for (i = NEOISP_NODE_GROUPS_COUNT - 1; i >= 0; i--) {
		neoisp_destroy_node_group(&neoispd->node_group[i]);

		if (standalone_mdev)
			media_device_cleanup(&neoispd->node_group[i].mdev);
	}

	pm_runtime_dont_use_autosuspend(neoispd->dev);
	pm_runtime_disable(neoispd->dev);
}

static int __maybe_unused neoisp_runtime_suspend(struct device *dev)
{
	struct neoisp_dev_s *neoispd = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(neoispd->num_clks, neoispd->clks);

	return 0;
}

static int __maybe_unused neoisp_runtime_resume(struct device *dev)
{
	int ret;
	struct neoisp_dev_s *neoispd = dev_get_drvdata(dev);

	ret = clk_bulk_prepare_enable(neoispd->num_clks, neoispd->clks);

	if (ret) {
		dev_err(dev, "Failed to resume device. Could not re-enable clocks.\n");
		return ret;
	}

	neoisp_init_hw(neoispd);

	return 0;
}

static int __maybe_unused neoisp_pm_suspend(struct device *dev)
{
	struct neoisp_dev_s *neoispd = dev_get_drvdata(dev);
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(NEOISP_SUSPEND_TIMEOUT_MS);
	while (neoispd->hw_busy) {
		cond_resched();
		if (time_after_eq(jiffies, timeout)) {
			dev_err(dev, "Failed to enter idle on system suspend\n");
			return -EBUSY;
		}
	}

	pm_runtime_force_suspend(dev);

	return 0;
}

static int __maybe_unused neoisp_pm_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}

static const struct dev_pm_ops neoisp_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(neoisp_pm_suspend, neoisp_pm_resume)
	SET_RUNTIME_PM_OPS(neoisp_runtime_suspend, neoisp_runtime_resume, NULL)
};

static const unsigned int neoisp_blocks_list_imx95x[] = {
	NEOISP_PARAM_BLK_PIPE_CONF,
	NEOISP_PARAM_BLK_HEAD_COLOR,
	NEOISP_PARAM_BLK_HDR_DECOMPRESS0,
	NEOISP_PARAM_BLK_HDR_DECOMPRESS1,
	NEOISP_PARAM_BLK_OBWB0,
	NEOISP_PARAM_BLK_OBWB1,
	NEOISP_PARAM_BLK_OBWB2,
	NEOISP_PARAM_BLK_HDR_MERGE,
	NEOISP_PARAM_BLK_RGBIR,
	NEOISP_PARAM_BLK_STAT,
	NEOISP_PARAM_BLK_CTEMP,
	NEOISP_PARAM_BLK_IR_COMPRESS,
	NEOISP_PARAM_BLK_BNR,
	NEOISP_PARAM_BLK_VIGNETTING_CTRL,
	NEOISP_PARAM_BLK_DEMOSAIC,
	NEOISP_PARAM_BLK_RGB2YUV,
	NEOISP_PARAM_BLK_DR_COMP,
	NEOISP_PARAM_BLK_NR,
	NEOISP_PARAM_BLK_AF,
	NEOISP_PARAM_BLK_EE,
	NEOISP_PARAM_BLK_DF,
	NEOISP_PARAM_BLK_CONVMED,
	NEOISP_PARAM_BLK_CAS,
	NEOISP_PARAM_BLK_GCM,
	NEOISP_PARAM_BLK_VIGNETTING_TABLE,
	NEOISP_PARAM_BLK_DRC_GLOBAL_TONEMAP,
	NEOISP_PARAM_BLK_DRC_LOCAL_TONEMAP,
	-1, /* end of list */
};

static const struct neoisp_info_s neoisp_imx95_data = {
	.capabilities = NEO_CAP_ALIGNMENT_MSB,
	.blocks_list = neoisp_blocks_list_imx95x,
};

static const struct neoisp_info_s neoisp_imx952_data = {
	.capabilities = 0,
	.blocks_list = neoisp_blocks_list_imx95x,
};

static const struct of_device_id neoisp_dt_ids[] = {
	{ .compatible = "nxp,imx95-neoisp", .data = &neoisp_imx95_data },
	{ .compatible = "nxp,imx952-neoisp", .data = &neoisp_imx952_data },
	{ },
};
MODULE_DEVICE_TABLE(of, neoisp_dt_ids);

static struct platform_driver neoisp_driver = {
	.probe  = neoisp_probe,
	.remove = neoisp_remove,
	.driver = {
		.name = NEOISP_NAME,
		.pm = &neoisp_pm,
		.of_match_table = neoisp_dt_ids,
	},
};

module_platform_driver(neoisp_driver);

MODULE_DESCRIPTION("NXP NEOISP Hardware");
MODULE_AUTHOR("Antoine Bouyer <antoine.bouyer@nxp.com>");
MODULE_LICENSE("GPL");
