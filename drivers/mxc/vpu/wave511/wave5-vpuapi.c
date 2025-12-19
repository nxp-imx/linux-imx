// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave5 series multi-standard codec IP - helper functions
 *
 * Copyright (C) 2021-2026 CHIPS&MEDIA INC
 */

#include <linux/bug.h>
#include "wave5-vpuapi.h"
#include "wave5-regdefine.h"
#include "wave5-hw.h"
#include "wave5-helper.h"

#define DECODE_ALL_TEMPORAL_LAYERS 0
#define DECODE_ALL_SPATIAL_LAYERS 0

int wave5_vpu_flush_instance(struct vpu_instance *inst)
{
	int ret = 0;
	int retry = 0;

	ret = mutex_lock_interruptible(&inst->dev->hw_lock);
	if (ret)
		return ret;
	do {
		/*
		 * Repeat the FLUSH command until the firmware reports that the
		 * VPU isn't running anymore
		 */
		ret = wave5_vpu_hw_flush_instance(inst);
		if (ret < 0 && ret != -EBUSY) {
			dev_warn(inst->dev->dev, "Flush of %s instance with id: %d fail: %d\n",
				 inst->type == VPU_INST_TYPE_DEC ? "DECODER" : "", inst->id,
				 ret);
			mutex_unlock(&inst->dev->hw_lock);
			return ret;
		}
		if (ret == -EBUSY && retry++ >= MAX_FIRMWARE_CALL_RETRY) {
			dev_warn(inst->dev->dev, "Flush of %s instance with id: %d timed out!\n",
				 inst->type == VPU_INST_TYPE_DEC ? "DECODER" : "", inst->id);
			mutex_unlock(&inst->dev->hw_lock);
			return -ETIMEDOUT;
		}
	} while (ret != 0);
	mutex_unlock(&inst->dev->hw_lock);

	return ret;
}

int wave5_vpu_get_version_info(struct device *dev, u32 *revision, unsigned int *product_id)
{
	int ret;
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	if (!wave5_vpu_is_init(vpu_dev)) {
		ret = -EINVAL;
		goto err_out;
	}

	if (product_id)
		*product_id = vpu_dev->product;
	ret = wave5_vpu_get_version(vpu_dev, revision);

err_out:
	mutex_unlock(&vpu_dev->hw_lock);
	return ret;
}

static int wave5_check_dec_open_param(struct vpu_instance *inst, struct dec_open_param *param)
{
	if (!param->bitstream_buffer)
		return 0;

	if (param->bitstream_buffer % 8) {
		dev_err(inst->dev->dev,
			"Bitstream buffer must be aligned to a multiple of 8\n");
		return -EINVAL;
	}

	if (param->bitstream_buffer_size % 1024 ||
	    param->bitstream_buffer_size < MIN_BITSTREAM_BUFFER_SIZE) {
		dev_err(inst->dev->dev,
			"Bitstream buffer size must be aligned to a multiple of 1024 and have a minimum size of %d\n",
			MIN_BITSTREAM_BUFFER_SIZE);
		return -EINVAL;
	}

	return 0;
}

int wave5_vpu_dec_open(struct vpu_instance *inst, struct dec_open_param *open_param)
{
	struct dec_info *p_dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	ret = wave5_check_dec_open_param(inst, open_param);
	if (ret)
		return ret;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	if (!wave5_vpu_is_init(vpu_dev)) {
		mutex_unlock(&vpu_dev->hw_lock);
		return -ENODEV;
	}

	p_dec_info = &inst->codec_info->dec_info;
	memcpy(&p_dec_info->open_param, open_param, sizeof(struct dec_open_param));

	p_dec_info->reorder_enable = open_param->reorder_enable;
	p_dec_info->temp_id_select_mode = TEMPORAL_ID_MODE_ABSOLUTE;
	p_dec_info->target_temp_id = DECODE_ALL_TEMPORAL_LAYERS;
	p_dec_info->target_spatial_id = DECODE_ALL_SPATIAL_LAYERS;

	ret = wave5_vpu_build_up_dec_param(inst, open_param);
	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

static int reset_auxiliary_buffers(struct vpu_instance *inst, unsigned int index)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;

	if (index >= WAVE5_MAX_FBS)
		return 1;

	if (p_dec_info->vb_mv[index].size == 0 && p_dec_info->vb_fbc_y_tbl[index].size == 0 &&
	    p_dec_info->vb_fbc_c_tbl[index].size == 0)
		return 1;

	wave5_vdi_free_dma_memory(&p_dec_info->vb_mv[index]);
	wave5_vdi_free_dma_memory(&p_dec_info->vb_fbc_y_tbl[index]);
	wave5_vdi_free_dma_memory(&p_dec_info->vb_fbc_c_tbl[index]);

	return 0;
}

int wave5_vpu_dec_close(struct vpu_instance *inst, u32 *fail_res)
{
	struct dec_info *p_dec_info;
	int ret;
	int retry = 0;
	struct vpu_device *vpu_dev = inst->dev;
	int i;

	*fail_res = 0;
	if (!inst->codec_info)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	p_dec_info = &inst->codec_info->dec_info;

	do {
		ret = wave5_vpu_dec_finish_seq(inst, fail_res);
		if (ret < 0 && *fail_res != WAVE5_SYSERR_VPU_STILL_RUNNING) {
			dev_warn(inst->dev->dev, "dec_finish_seq timed out\n");
			goto unlock_and_return;
		}

		if (*fail_res == WAVE5_SYSERR_VPU_STILL_RUNNING &&
		    retry++ >= MAX_FIRMWARE_CALL_RETRY) {
			ret = -ETIMEDOUT;
			goto unlock_and_return;
		}
	} while (ret != 0);

	dev_dbg(inst->dev->dev, "%s: dec_finish_seq complete\n", __func__);

	for (i = 0 ; i < WAVE5_MAX_FBS; i++) {
		ret = reset_auxiliary_buffers(inst, i);
		if (ret) {
			ret = 0;
			break;
		}
	}

	wave5_vdi_free_dma_memory(&p_dec_info->vb_task);

unlock_and_return:
	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave5_vpu_dec_issue_seq_init(struct vpu_instance *inst)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	if (!inst->next_frame)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	p_dec_info->stream_rd_ptr = wave5_get_plane_dma_addr(&inst->next_frame->vb2_buf, 0);
	p_dec_info->stream_wr_ptr = p_dec_info->stream_rd_ptr +
				    wave5_get_plane_payload(&inst->next_frame->vb2_buf, 0);

	ret = wave5_vpu_dec_init_seq(inst);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave5_vpu_dec_complete_seq_init(struct vpu_instance *inst, struct dec_initial_info *info)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave5_vpu_dec_get_seq_info(inst, info);
	if (!ret)
		p_dec_info->initial_info_obtained = true;

	info->rd_ptr = wave5_dec_get_rd_ptr(inst);
	info->wr_ptr = p_dec_info->stream_wr_ptr;

	p_dec_info->initial_info = *info;

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

void wave5_vpu_dec_fill_linear_frame(struct vpu_instance *inst,
				     struct frame_buffer *frame, struct vb2_buffer *vb)
{
	dma_addr_t buf_addr_y = 0;
	dma_addr_t buf_addr_cb = 0;
	dma_addr_t buf_addr_cr = 0;
	u32 buf_size = 0;
	u32 fb_stride;
	u32 luma_size;
	u32 chroma_size;

	fb_stride = inst->dst_fmt.plane_fmt[0].bytesperline;
	luma_size = fb_stride * inst->dst_fmt.height;
	if (inst->output_format == FORMAT_422 ||
	    inst->output_format == FORMAT_422_P10_16BIT_MSB ||
	    inst->output_format == FORMAT_422_P10_16BIT_LSB ||
	    inst->output_format == FORMAT_422_P10_32BIT_MSB ||
	    inst->output_format == FORMAT_422_P10_32BIT_LSB)
		chroma_size = fb_stride * inst->dst_fmt.height / 2;
	else
		chroma_size = fb_stride * inst->dst_fmt.height / 4;

	if (inst->dst_fmt.num_planes == 1) {
		buf_size = vb2_plane_size(vb, 0);
		buf_addr_y = wave5_get_plane_dma_addr(vb, 0);
		buf_addr_cb = buf_addr_y + luma_size;
		buf_addr_cr = buf_addr_cb + chroma_size;
	} else if (inst->dst_fmt.num_planes == 2) {
		buf_size = vb2_plane_size(vb, 0) + vb2_plane_size(vb, 1);
		buf_addr_y = wave5_get_plane_dma_addr(vb, 0);
		buf_addr_cb = wave5_get_plane_dma_addr(vb, 1);
		buf_addr_cr = buf_addr_cb + chroma_size;
	} else if (inst->dst_fmt.num_planes == 3) {
		buf_size = vb2_plane_size(vb, 0) + vb2_plane_size(vb, 1) + vb2_plane_size(vb, 2);
		buf_addr_y = wave5_get_plane_dma_addr(vb, 0);
		buf_addr_cb = wave5_get_plane_dma_addr(vb, 1);
		buf_addr_cr = wave5_get_plane_dma_addr(vb, 2);
	}

	frame->buf_y = buf_addr_y;
	frame->buf_cb = buf_addr_cb;
	frame->buf_cr = buf_addr_cr;
	frame->size = buf_size;
	frame->width = inst->src_fmt.width;
	frame->stride = fb_stride;
	frame->map_type = LINEAR_FRAME_MAP;
	frame->index = vb->index;
	frame->update_fb_info = true;
}

int wave5_vpu_dec_register_frame_buffer_ex(struct vpu_instance *inst, int num_of_decoding_fbs,
					   int stride, int height)
{
	struct dec_info *p_dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;
	struct frame_buffer *fb;

	if (num_of_decoding_fbs >= WAVE5_MAX_FBS || !num_of_decoding_fbs)
		return -EINVAL;

	p_dec_info = &inst->codec_info->dec_info;
	p_dec_info->num_of_decoding_fbs = num_of_decoding_fbs;
	p_dec_info->stride = stride;

	if (!p_dec_info->initial_info_obtained)
		return -EINVAL;

	if (stride < p_dec_info->initial_info.pic_width || (stride % 8 != 0) ||
	    height < p_dec_info->initial_info.pic_height)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	fb = inst->frame_buf;
	ret = wave5_vpu_dec_register_framebuffer(inst, &fb[0], COMPRESSED_FRAME_MAP,
						 p_dec_info->num_of_decoding_fbs);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave5_vpu_dec_register_display_buffer_ex(struct vpu_instance *inst, struct frame_buffer *frame)
{
	struct dec_info *p_dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	if (!frame || frame->index >= WAVE5_MAX_FBS)
		return -EINVAL;

	p_dec_info = &inst->codec_info->dec_info;

	if (!p_dec_info->initial_info_obtained)
		return -EINVAL;

	dev_dbg(inst->dev->dev, "[%d] register linear[%d] %pad, %pad, %pad\n",
		inst->id, frame->index, &frame->buf_y, &frame->buf_cb, &frame->buf_cr);

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave5_vpu_dec_register_displaybuffer(inst, frame);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave5_vpu_dec_start_one_frame(struct vpu_instance *inst, u32 *res_fail)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	if (p_dec_info->stride == 0) /* this means frame buffers have not been registered. */
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	if (inst->next_frame) {
		p_dec_info->stream_rd_ptr = wave5_get_plane_dma_addr(&inst->next_frame->vb2_buf, 0);
		p_dec_info->stream_wr_ptr = p_dec_info->stream_rd_ptr +
					    wave5_get_plane_payload(&inst->next_frame->vb2_buf, 0);
	} else {
		p_dec_info->stream_endflag = true;
		p_dec_info->stream_rd_ptr = p_dec_info->stream_wr_ptr;
	}

	ret = wave5_vpu_decode(inst, res_fail);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave5_vpu_dec_get_output_info(struct vpu_instance *inst, struct dec_output_info *info)
{
	struct dec_info *p_dec_info;
	int ret;
	struct vpu_rect rect_info;
	u32 decoded_index;
	u32 disp_idx;
	u32 max_dec_index;
	struct vpu_device *vpu_dev = inst->dev;
	struct dec_output_info *disp_info;

	if (!info)
		return -EINVAL;

	p_dec_info = &inst->codec_info->dec_info;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	memset(info, 0, sizeof(*info));

	ret = wave5_vpu_dec_get_result(inst, info);
	if (ret) {
		info->rd_ptr = p_dec_info->stream_rd_ptr;
		info->wr_ptr = p_dec_info->stream_wr_ptr;
		goto err_out;
	}

	decoded_index = info->index_frame_decoded;

	/* calculate display frame region */
	rect_info.left = 0;
	rect_info.right = 0;
	rect_info.top = 0;
	rect_info.bottom = 0;

	if (decoded_index < WAVE5_MAX_FBS) {
		if (inst->std == W_HEVC_DEC || inst->std == W_AVC_DEC)
			rect_info = p_dec_info->initial_info.pic_crop_rect;

		if (inst->std == W_HEVC_DEC)
			p_dec_info->dec_out_info[decoded_index].decoded_poc = info->decoded_poc;

		p_dec_info->dec_out_info[decoded_index].rc_decoded = rect_info;
	}
	info->rc_decoded = rect_info;

	disp_idx = info->index_frame_display;
	if (info->index_frame_display >= 0 && info->index_frame_display < WAVE5_MAX_FBS) {
		disp_info = &p_dec_info->dec_out_info[disp_idx];
		if (info->index_frame_display != info->index_frame_decoded) {
			/*
			 * when index_frame_decoded < 0, and index_frame_display >= 0
			 * info->dec_pic_width and info->dec_pic_height are still valid
			 * but those of p_dec_info->dec_out_info[disp_idx] are invalid in VP9
			 */
			info->disp_pic_width = disp_info->dec_pic_width;
			info->disp_pic_height = disp_info->dec_pic_height;
		} else {
			info->disp_pic_width = info->dec_pic_width;
			info->disp_pic_height = info->dec_pic_height;
		}

		info->rc_display = disp_info->rc_decoded;

	} else {
		info->rc_display.left = 0;
		info->rc_display.right = 0;
		info->rc_display.top = 0;
		info->rc_display.bottom = 0;
		info->disp_pic_width = 0;
		info->disp_pic_height = 0;
	}

	p_dec_info->stream_rd_ptr = wave5_dec_get_rd_ptr(inst);
	p_dec_info->frame_display_flag = vpu_read_reg(vpu_dev, W5_RET_DEC_DISP_IDC);

	max_dec_index = (p_dec_info->num_of_decoding_fbs > p_dec_info->num_of_display_fbs) ?
		p_dec_info->num_of_decoding_fbs : p_dec_info->num_of_display_fbs;
	max_dec_index = MIN(max_dec_index, WAVE5_MAX_FBS);

	if (info->index_frame_display >= 0 &&
	    info->index_frame_display < (int)max_dec_index)
		info->disp_frame = p_dec_info->disp_buf[info->index_frame_display];

	info->rd_ptr = p_dec_info->stream_rd_ptr;
	info->wr_ptr = p_dec_info->stream_wr_ptr;
	info->frame_display_flag = p_dec_info->frame_display_flag;

	info->sequence_no = p_dec_info->initial_info.sequence_no;
	if (decoded_index < WAVE5_MAX_FBS)
		p_dec_info->dec_out_info[decoded_index] = *info;

	if (disp_idx < WAVE5_MAX_FBS)
		info->disp_frame.sequence_no = info->sequence_no;

	if (info->sequence_changed) {
		memcpy((void *)&p_dec_info->initial_info, (void *)&p_dec_info->new_seq_info,
		       sizeof(struct dec_initial_info));
		p_dec_info->initial_info.sequence_no++;
	}

err_out:
	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave5_vpu_dec_clr_disp_flag(struct vpu_instance *inst, int index)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	if (index >= p_dec_info->num_of_display_fbs) {
		dev_dbg(inst->dev->dev, "[%d] disp buf[%d] is out of range\n", inst->id, index);
		return 0;
	}

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;
	ret = wave5_dec_clr_disp_flag(inst, index);
	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave5_vpu_dec_set_disp_flag(struct vpu_instance *inst, int index)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret = 0;
	struct vpu_device *vpu_dev = inst->dev;

	if (index >= p_dec_info->num_of_display_fbs)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;
	ret = wave5_dec_set_disp_flag(inst, index);
	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave5_vpu_dec_reset_framebuffer(struct vpu_instance *inst, unsigned int index)
{
	if (index >= WAVE5_MAX_FBS)
		return -EINVAL;

	if (inst->frame_vbuf[index].size == 0)
		return -EINVAL;

	wave5_vdi_free_dma_memory(&inst->frame_vbuf[index]);

	return 0;
}

int wave5_vpu_dec_give_command(struct vpu_instance *inst, enum codec_command cmd, void *parameter)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret = 0;

	switch (cmd) {
	case DEC_GET_QUEUE_STATUS: {
		struct queue_status_info *queue_info = parameter;

		queue_info->instance_queue_count = p_dec_info->instance_queue_count;
		queue_info->report_queue_count = p_dec_info->report_queue_count;
		break;
	}
	case DEC_RESET_FRAMEBUF_INFO: {
		int i;

		for (i = 0; i < WAVE5_MAX_FBS; i++) {
			ret = wave5_vpu_dec_reset_framebuffer(inst, i);
			if (ret)
				break;
		}

		for (i = 0; i < WAVE5_MAX_FBS; i++) {
			ret = reset_auxiliary_buffers(inst, i);
			if (ret)
				break;
		}

		p_dec_info->num_of_display_fbs = 0;
		for (i = 0; i < WAVE5_MAX_FBS; i++)
			memset(&p_dec_info->disp_buf[i], 0, sizeof(struct frame_buffer));

		wave5_vdi_free_dma_memory(&p_dec_info->vb_task);
		break;
	}
	case DEC_GET_SEQ_INFO: {
		struct dec_initial_info *seq_info = parameter;

		*seq_info = p_dec_info->initial_info;
		break;
	}

	default:
		return -EINVAL;
	}

	return ret;
}

bool wave5_vpu_dec_is_cq_done(struct vpu_instance *inst)
{
	if (inst->dynamic_source_change)
		return true;
	return atomic_read(&inst->queued_dec_cmd) ? false : true;
}
