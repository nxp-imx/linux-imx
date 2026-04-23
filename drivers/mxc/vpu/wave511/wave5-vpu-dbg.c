// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave5 series multi-standard codec IP - debug interface
 *
 * Copyright (C) 2026 CHIPS&MEDIA INC
 */

#include <linux/types.h>
#include "wave5-helper.h"
#include "wave5-vpu-dbg.h"

static char *wave5_flow_name[] = {
	[WAVE5_VPU_FLOW_NONE] = "none",
	[WAVE5_VPU_FLOW_SET_STATE] = "switch state",
	[WAVE5_VPU_FLOW_OUTPUT_ON] = "output on",
	[WAVE5_VPU_FLOW_OUTPUT_OFF] = "output off",
	[WAVE5_VPU_FLOW_CAPTURE_ON] = "capture on",
	[WAVE5_VPU_FLOW_CAPTURE_OFF] = "capture off",
	[WAVE5_VPU_FLOW_START] = "start",
	[WAVE5_VPU_FLOW_STOP] = "stop",
	[WAVE5_VPU_FLOW_SOURCE_CHANGE] = "dynamic source change",
	[WAVE5_VPU_FLOW_EOS] = "eos",
	[WAVE5_VPU_FLOW_MAXIMUM] = "unknown",
};

static int wave5_vpu_dbg_instance(struct seq_file *s, void *data)
{
	struct vpu_instance *inst = s->private;
	struct vpu_attr *p_attr = &inst->dev->attr;
	struct vpu_performance_info *perf = &inst->performance;
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	struct vb2_queue *vq;
	char str[128];
	int num;
	s64 tmp;
	s64 fps;
	int index;

	if (!inst->v4l2_fh.m2m_ctx)
		return 0;

	num = scnprintf(str, sizeof(str), "%s : product 0x%x, fw_ver %d.%d.%d(r%u)\n",
			dev_name(inst->dev->dev), inst->dev->product_code,
			(p_attr->fw_api_version >> 24) & 0xFF,
			(p_attr->fw_api_version >> 16) & 0xFF,
			(p_attr->fw_api_version >> 0) & 0xFFFF,
			p_attr->fw_version);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str), "state = %s (%d)\n",
			state_to_str(inst->state), inst->state);
	if (seq_write(s, str, num))
		return 0;

	vq = v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx);
	num = scnprintf(str, sizeof(str),
			"output (%2d, %2d): fmt = %c%c%c%c %d x %d, %d;\n",
			vb2_is_streaming(vq),
			vb2_get_num_buffers(vq),
			inst->src_fmt.pixelformat,
			inst->src_fmt.pixelformat >> 8,
			inst->src_fmt.pixelformat >> 16,
			inst->src_fmt.pixelformat >> 24,
			inst->src_fmt.width,
			inst->src_fmt.height,
			vq->last_buffer_dequeued);
	if (seq_write(s, str, num))
		return 0;

	vq = v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx);
	num = scnprintf(str, sizeof(str),
			"capture(%2d, %2d): fmt = %c%c%c%c %d x %d, %d;\n",
			vb2_is_streaming(vq),
			vb2_get_num_buffers(vq),
			inst->dst_fmt.pixelformat,
			inst->dst_fmt.pixelformat >> 8,
			inst->dst_fmt.pixelformat >> 16,
			inst->dst_fmt.pixelformat >> 24,
			inst->dst_fmt.width,
			inst->dst_fmt.height,
			vq->last_buffer_dequeued);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str), "fbc : %d; disp %d (register 0x%lx, avail 0x%lx)\n",
			inst->fbc_buf_count, inst->disp_buf_count, inst->disp_buf_mask,
			inst->avail_dst_bufs);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str), "crop: (%d, %d) %d x %d\n",
			inst->conf_win.left,
			inst->conf_win.top,
			inst->conf_win.width,
			inst->conf_win.height);
	if (seq_write(s, str, num))
		return 0;

	if (inst->scaler_info.enable) {
		num = scnprintf(str, sizeof(str), "scale: %d x %d\n",
				inst->scaler_info.width, inst->scaler_info.height);
		if (seq_write(s, str, num))
			return 0;
	}

	num = scnprintf(str, sizeof(str),
			"src %d, dst %d, decode %d, process %d, display %d, sequence %d, skip %d, err %d\n",
			inst->queued_src_buf_num,
			inst->queued_dst_buf_num,
			inst->total_dec_cnt,
			inst->processed_buf_num,
			inst->displayed_buf_num,
			inst->sequence,
			inst->skipped_frame_num,
			inst->error_frame_num);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str),
			"seek %d, source change %d, drain %d:%d(%d), endflag %d, has_stoped %d\n",
			inst->seek_flag,
			inst->dynamic_source_change,
			inst->v4l2_fh.m2m_ctx->out_q_ctx.buffered,
			inst->eos,
			inst->drain_dec_cnt,
			p_dec_info->stream_endflag,
			v4l2_m2m_has_stopped(inst->v4l2_fh.m2m_ctx));
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str), "queued_dec_cmd %d, feed_frame_cnt %d\n",
			atomic_read(&inst->queued_dec_cmd), atomic_read(&inst->feed_frame_cnt));
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str), "fps");
	if (seq_write(s, str, num))
		return 0;
	tmp = MSEC_PER_SEC * inst->processed_buf_num;
	if (perf->ts_last > perf->ts_first + NSEC_PER_MSEC) {
		fps = DIV_ROUND_CLOSEST(tmp, (perf->ts_last - perf->ts_first) / NSEC_PER_MSEC);
		num = scnprintf(str, sizeof(str), " actual: %lld;", fps);
		if (seq_write(s, str, num))
			return 0;
	}
	if (perf->total_sw_time) {
		fps = DIV_ROUND_CLOSEST(tmp, perf->total_sw_time / NSEC_PER_MSEC);
		num = scnprintf(str, sizeof(str), " sw: %lld;", fps);
		if (seq_write(s, str, num))
			return 0;
	}
	if (perf->total_hw_time) {
		fps = DIV_ROUND_CLOSEST(tmp, perf->total_hw_time / NSEC_PER_MSEC);
		num = scnprintf(str, sizeof(str), " hw: %lld", fps);
		if (seq_write(s, str, num))
			return 0;
	}
	num = scnprintf(str, sizeof(str), "\n");
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str), "flow:\n");
	if (seq_write(s, str, num))
		return 0;

	index = inst->flow.index;
	for (int i = 0; i < WAVE5_VPU_FLOW_DEPTH; i++) {
		struct vpu_flow_item *item = &inst->flow.flows[(index + i) % WAVE5_VPU_FLOW_DEPTH];

		if (item->key == WAVE5_VPU_FLOW_NONE || item->key >= WAVE5_VPU_FLOW_MAXIMUM)
			continue;

		if (item->key == WAVE5_VPU_FLOW_SET_STATE)
			num = scnprintf(str, sizeof(str), "    %s, %s -> %s\n",
					wave5_flow_name[item->key],
					state_to_str(item->arg1), state_to_str(item->arg2));
		else
			num = scnprintf(str, sizeof(str), "    %s, %d, %d\n",
					wave5_flow_name[item->key], item->arg1, item->arg2);
		if (seq_write(s, str, num))
			return 0;
	}

	return 0;
}

static int wave5_vpu_dbg_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, wave5_vpu_dbg_instance, inode->i_private);
}

static const struct file_operations wave5_vpu_dbg_fops = {
	.owner = THIS_MODULE,
	.open = wave5_vpu_dbg_open,
	.release = single_release,
	.read = seq_read,
};

int wave5_vpu_create_dbgfs_file(struct vpu_instance *inst)
{
	char name[64];

	if (!inst || !inst->dev || IS_ERR_OR_NULL(inst->dev->debugfs))
		return -EINVAL;

	scnprintf(name, sizeof(name), "instance.%d", inst->id);
	inst->debugfs = debugfs_create_file((const char *)name,
					    VERIFY_OCTAL_PERMISSIONS(0444),
					    inst->dev->debugfs,
					    inst,
					    &wave5_vpu_dbg_fops);

	return 0;
}

void wave5_vpu_remove_dbgfs_file(struct vpu_instance *inst)
{
	if (!inst || !inst->debugfs)
		return;

	debugfs_remove(inst->debugfs);
	inst->debugfs = NULL;
}
