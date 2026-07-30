// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Coda series multi-standard codec IP - debug interface
 *
 * Copyright (C) 2026 CHIPS&MEDIA INC
 */

#include <linux/types.h>
#include "coda-helper.h"
#include "coda-vpu-dbg.h"

static const char *coda_profile_to_str(u32 profile)
{
	switch (profile) {
	case H264_PROFILE_BP:
		return "baseline";
	case H264_PROFILE_MP:
		return "main";
	case H264_PROFILE_HP:
		return "high";
	default:
		return "unknown";
	}
}

static const char *coda_level_to_str(u32 level)
{
	static const char * const names[] = {
		[V4L2_MPEG_VIDEO_H264_LEVEL_1_0] = "1.0",
		[V4L2_MPEG_VIDEO_H264_LEVEL_1B]  = "1b",
		[V4L2_MPEG_VIDEO_H264_LEVEL_1_1] = "1.1",
		[V4L2_MPEG_VIDEO_H264_LEVEL_1_2] = "1.2",
		[V4L2_MPEG_VIDEO_H264_LEVEL_1_3] = "1.3",
		[V4L2_MPEG_VIDEO_H264_LEVEL_2_0] = "2.0",
		[V4L2_MPEG_VIDEO_H264_LEVEL_2_1] = "2.1",
		[V4L2_MPEG_VIDEO_H264_LEVEL_2_2] = "2.2",
		[V4L2_MPEG_VIDEO_H264_LEVEL_3_0] = "3.0",
		[V4L2_MPEG_VIDEO_H264_LEVEL_3_1] = "3.1",
		[V4L2_MPEG_VIDEO_H264_LEVEL_3_2] = "3.2",
		[V4L2_MPEG_VIDEO_H264_LEVEL_4_0] = "4.0",
		[V4L2_MPEG_VIDEO_H264_LEVEL_4_1] = "4.1",
		[V4L2_MPEG_VIDEO_H264_LEVEL_4_2] = "4.2",
		[V4L2_MPEG_VIDEO_H264_LEVEL_5_0] = "5.0",
		[V4L2_MPEG_VIDEO_H264_LEVEL_5_1] = "5.1",
	};

	if (level < ARRAY_SIZE(names) && names[level])
		return names[level];

	return "unknown";
}

static const char *coda_state_to_str(enum vpu_instance_state state)
{
	switch (state) {
	case VPU_INST_STATE_NONE:
		return "none";
	case VPU_INST_STATE_OPEN:
		return "open";
	case VPU_INST_STATE_INIT_SEQ:
		return "init_seq";
	case VPU_INST_STATE_PIC_RUN:
		return "pic_run";
	case VPU_INST_STATE_SEEK:
		return "seek";
	case VPU_INST_STATE_STOP:
		return "stop";
	default:
		return "unknown";
	}
}

static const char *coda_memory_to_str(enum vb2_memory memory)
{
	switch (memory) {
	case VB2_MEMORY_MMAP:
		return "mmap";
	case VB2_MEMORY_USERPTR:
		return "userptr";
	case VB2_MEMORY_DMABUF:
		return "dmabuf";
	default:
		return "unknown";
	}
}

static const char *coda_deblock_mode_to_str(u32 mode)
{
	switch (mode) {
	case V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED:
		return "enabled";
	case V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED:
		return "disabled";
	case V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY:
		return "disabled_at_slice_boundary";
	default:
		return "unknown";
	}
}

static int coda_vpu_dbg_show(struct seq_file *s, void *data)
{
	struct vpu_instance *inst = s->private;
	struct vpu_enc_controls *ctrls = &inst->enc_ctrls;
	struct vb2_queue *vq;
	s64 tmp;
	s64 fps;

	if (!inst->v4l2_fh.m2m_ctx)
		return 0;

	seq_puts(s, "[Encoder]\n");
	seq_printf(s, "%s : product 0x%x, fw_ver %u(r%u)\n",
		   dev_name(inst->vpu_dev->dev), inst->vpu_dev->product_code,
		   inst->vpu_dev->fw_version, inst->vpu_dev->fw_revision);
	seq_printf(s, "state = %s (%d)\n",
		   coda_state_to_str(inst->state), inst->state);

	vq = v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx);
	seq_printf(s, "output (%2d, %2d, %s): fmt = %c%c%c%c %d x %d, %d;\n",
		   vb2_is_streaming(vq),
		   vb2_get_num_buffers(vq),
		   coda_memory_to_str(vq->memory),
		   inst->src_fmt.pixelformat,
		   inst->src_fmt.pixelformat >> 8,
		   inst->src_fmt.pixelformat >> 16,
		   inst->src_fmt.pixelformat >> 24,
		   inst->src_fmt.width,
		   inst->src_fmt.height,
		   vq->last_buffer_dequeued);

	vq = v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx);
	seq_printf(s, "capture(%2d, %2d, %s): fmt = %c%c%c%c %d x %d, %d;\n",
		   vb2_is_streaming(vq),
		   vb2_get_num_buffers(vq),
		   coda_memory_to_str(vq->memory),
		   inst->dst_fmt.pixelformat,
		   inst->dst_fmt.pixelformat >> 8,
		   inst->dst_fmt.pixelformat >> 16,
		   inst->dst_fmt.pixelformat >> 24,
		   inst->dst_fmt.width,
		   inst->dst_fmt.height,
		   vq->last_buffer_dequeued);

	seq_printf(s, "crop: (%d, %d) %d x %d\n",
		   inst->crop.left, inst->crop.top,
		   inst->crop.width, inst->crop.height);

	seq_printf(s, "queued src %d, dst %d, processed %d, eos %d\n",
		   inst->queued_src_buf_num,
		   inst->queued_dst_buf_num,
		   inst->processed_buf_num,
		   inst->eos);

	seq_puts(s, "fps");
	tmp = MSEC_PER_SEC * inst->processed_buf_num;
	if (inst->total_sw_time) {
		fps = DIV_ROUND_CLOSEST(tmp, inst->total_sw_time / NSEC_PER_MSEC);
		seq_printf(s, " sw: %lld;", fps);
	}
	if (inst->total_hw_time) {
		fps = DIV_ROUND_CLOSEST(tmp, inst->total_hw_time / NSEC_PER_MSEC);
		seq_printf(s, " hw: %lld", fps);
	}
	seq_puts(s, "\n");

	if (inst->processed_buf_num) {
		u32 qp_avg = div64_ul(inst->qp_sum, inst->processed_buf_num);

		seq_printf(s, "avg_qp avg: %u, min: %u, max: %u\n",
			   qp_avg, inst->qp_min, inst->qp_max);
	}

	seq_printf(s, "memory usage : %lu\n",
		   imx_mur_long_read(inst->recorder));

	/* Encoder parameters from user controls */
	seq_printf(s, "profile %s, level %s\n",
		   coda_profile_to_str(ctrls->h264_profile),
		   coda_level_to_str(ctrls->h264_level));
	seq_printf(s, "framerate %d, gop_size %d\n",
		   inst->framerate, ctrls->gop_size);
	seq_printf(s, "bitrate %d kbps", ctrls->bitrate / 1000);
	if (inst->processed_buf_num) {
		u64 avg_frame_bits;
		u64 actual_kbps;

		avg_frame_bits = div64_ul(
			inst->total_frame_size * 8,
			inst->processed_buf_num);
		actual_kbps =
			avg_frame_bits * inst->framerate / 1000;
		seq_printf(s, " (actual %llu kbps)",
			   actual_kbps);
	}
	seq_puts(s, "\n");
	seq_printf(s, "rc: frame_rc %d, mb_rc %d, mode %s\n",
		   ctrls->frame_rc_enable,
		   ctrls->mb_rc_enable,
		   ctrls->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR ?
		   "vbr" : "cbr");
	seq_printf(s, "qp: i_frame %d, p_frame %d, min %d, max %d\n",
		   ctrls->h264_i_frame_qp,
		   ctrls->h264_p_frame_qp,
		   ctrls->h264_min_qp,
		   ctrls->h264_max_qp);
	seq_printf(s, "entropy %s, 8x8_transform %d\n",
		   (ctrls->h264_profile >= H264_PROFILE_MP &&
		    ctrls->h264_entropy_mode ==
		    V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC) ?
		   "CABAC" : "CAVLC",
		   ctrls->h264_profile >= H264_PROFILE_HP ?
		   ctrls->h264_8x8_transform : 0);
	seq_printf(s, "constrained_intra_pred %d, chroma_qp_offset %d\n",
		   ctrls->h264_constrained_intra_prediction,
		   ctrls->h264_chroma_qp_index_offset);
	seq_printf(s, "deblock: mode %s",
		   coda_deblock_mode_to_str(ctrls->h264_loop_filter_mode));
	if (ctrls->h264_loop_filter_mode !=
	    V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED)
		seq_printf(s, ", alpha %d, beta %d",
			   ctrls->h264_loop_filter_alpha,
			   ctrls->h264_loop_filter_beta);
	seq_puts(s, "\n");
	seq_printf(s, "cpb_size %d, frame_skip %d\n",
		   ctrls->h264_cpb_size,
		   ctrls->frame_skip_mode);
	seq_printf(s, "slice: mode %d", ctrls->multi_slice_mode);
	if (ctrls->multi_slice_mode !=
	    V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE)
		seq_printf(s, ", max_mb %d", ctrls->multi_slice_max_mb);
	seq_puts(s, "\n");
	seq_printf(s, "intra_refresh: mb %d, period %d\n",
		   ctrls->cyclic_intra_refresh_mb,
		   ctrls->intra_refresh_period);
	seq_printf(s, "flip %d, rotate %d\n",
		   ctrls->flip, ctrls->rotate);
	seq_printf(s, "mv_search: h %d, v %d\n",
		   ctrls->mv_h_search_range,
		   ctrls->mv_v_search_range);
	seq_printf(s, "sar: enable %d", ctrls->h264_vui_sar_enable);
	if (ctrls->h264_vui_sar_enable)
		seq_printf(s, ", idc %d, ext %dx%d",
			   ctrls->h264_vui_sar_idc,
			   ctrls->h264_vui_ext_sar_width,
			   ctrls->h264_vui_ext_sar_height);
	seq_puts(s, "\n");

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(coda_vpu_dbg);

int coda_vpu_create_dbgfs_file(struct vpu_instance *inst)
{
	char name[64];

	if (!inst || !inst->vpu_dev || IS_ERR_OR_NULL(inst->vpu_dev->debugfs))
		return -EINVAL;

	scnprintf(name, sizeof(name), "instance.%d", inst->id);
	inst->debugfs = debugfs_create_file((const char *)name,
					    VERIFY_OCTAL_PERMISSIONS(0444),
					    inst->vpu_dev->debugfs,
					    inst,
					    &coda_vpu_dbg_fops);

	return 0;
}

void coda_vpu_remove_dbgfs_file(struct vpu_instance *inst)
{
	if (!inst || !inst->debugfs)
		return;

	debugfs_remove(inst->debugfs);
	inst->debugfs = NULL;
}
