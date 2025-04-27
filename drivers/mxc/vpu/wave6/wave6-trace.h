/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - wave6 driver tracer
 *
 * Copyright (C) 2025 CHIPS&MEDIA INC
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM wave6

#if !defined(__WAVE6_TRACE_H__) || defined(TRACE_HEADER_MULTI_READ)
#define __WAVE6_TRACE_H__

#include <linux/tracepoint.h>
#include <media/videobuf2-v4l2.h>

DECLARE_EVENT_CLASS(register_access,
	TP_PROTO(struct device *dev, u32 addr, u32 value),
	TP_ARGS(dev, addr, value),
	TP_STRUCT__entry(
		__string(name, dev_name(dev))
		__field(u32, addr)
		__field(u32, value)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->addr = addr;
		__entry->value = value;
	),
	TP_printk("%s:0x%03x 0x%08x", __get_str(name), __entry->addr, __entry->value)
);

DEFINE_EVENT(register_access, writel,
	TP_PROTO(struct device *dev, u32 addr, u32 value),
	TP_ARGS(dev, addr, value));
DEFINE_EVENT(register_access, readl,
	TP_PROTO(struct device *dev, u32 addr, u32 value),
	TP_ARGS(dev, addr, value));

TRACE_EVENT(send_command,
	TP_PROTO(struct vpu_device *vpu_dev, u32 id, u32 std, u32 cmd),
	TP_ARGS(vpu_dev, id, std, cmd),
	TP_STRUCT__entry(
		__string(name, dev_name(vpu_dev->dev))
		__field(u32, id)
		__field(u32, std)
		__field(u32, cmd)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->id = id;
		__entry->std = std;
		__entry->cmd = cmd;
	),
	TP_printk("%s: inst id %d, std 0x%x, cmd 0x%x",
		  __get_str(name), __entry->id, __entry->std, __entry->cmd)
);

TRACE_EVENT(irq,
	TP_PROTO(struct vpu_device *vpu_dev, u32 irq),
	TP_ARGS(vpu_dev, irq),
	TP_STRUCT__entry(
		__string(name, dev_name(vpu_dev->dev))
		__field(u32, irq)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->irq = irq;
	),
	TP_printk("%s: irq 0x%x", __get_str(name), __entry->irq)
);

TRACE_EVENT(set_state,
	TP_PROTO(struct vpu_instance *inst, u32 state),
	TP_ARGS(inst, state),
	TP_STRUCT__entry(
		__string(name, dev_name(inst->dev->dev))
		__field(u32, id)
		__string(cur_state, wave6_vpu_instance_state_name(inst->state))
		__string(nxt_state, wave6_vpu_instance_state_name(state))
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->id = inst->id;
		__assign_str(cur_state);
		__assign_str(nxt_state);
	),
	TP_printk("%s: inst[%d] set state %s -> %s",
		  __get_str(name), __entry->id, __get_str(cur_state), __get_str(nxt_state))
);

DECLARE_EVENT_CLASS(inst_internal,
	TP_PROTO(struct vpu_instance *inst, u32 type),
	TP_ARGS(inst, type),
	TP_STRUCT__entry(
		__string(name, dev_name(inst->dev->dev))
		__field(u32, id)
		__string(type, V4L2_TYPE_IS_OUTPUT(type) ? "output" : "capture")
		__field(u32, pixelformat)
		__field(u32, width)
		__field(u32, height)
		__field(u32, buf_cnt_src)
		__field(u32, buf_cnt_dst)
		__field(u32, processed_cnt)
		__field(u32, error_cnt)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->id = inst->id;
		__assign_str(type);
		__entry->pixelformat  = V4L2_TYPE_IS_OUTPUT(type) ? inst->src_fmt.pixelformat :
								    inst->dst_fmt.pixelformat;
		__entry->width = V4L2_TYPE_IS_OUTPUT(type) ? inst->src_fmt.width :
							     inst->dst_fmt.width;
		__entry->height = V4L2_TYPE_IS_OUTPUT(type) ? inst->src_fmt.height :
							      inst->dst_fmt.height;
		__entry->buf_cnt_src = inst->queued_src_buf_num;
		__entry->buf_cnt_dst = inst->queued_dst_buf_num;
		__entry->processed_cnt = inst->processed_buf_num;
		__entry->error_cnt = inst->error_buf_num;
	),
	TP_printk("%s: inst[%d] %s %c%c%c%c %dx%d, input %d, %d, process %d, error %d",
		  __get_str(name), __entry->id, __get_str(type),
		  __entry->pixelformat,
		  __entry->pixelformat >> 8,
		  __entry->pixelformat >> 16,
		  __entry->pixelformat >> 24,
		  __entry->width, __entry->height,
		  __entry->buf_cnt_src, __entry->buf_cnt_dst,
		  __entry->processed_cnt, __entry->error_cnt)
);

DEFINE_EVENT(inst_internal, start_streaming,
	TP_PROTO(struct vpu_instance *inst, u32 type),
	TP_ARGS(inst, type));

DEFINE_EVENT(inst_internal, stop_streaming,
	TP_PROTO(struct vpu_instance *inst, u32 type),
	TP_ARGS(inst, type));

TRACE_EVENT(dec_pic,
	TP_PROTO(struct vpu_instance *inst, u32 srcidx, u32 size),
	TP_ARGS(inst, srcidx, size),
	TP_STRUCT__entry(
		__string(name, dev_name(inst->dev->dev))
		__field(u32, id)
		__field(u32, srcidx)
		__field(u32, start)
		__field(u32, size)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->id = inst->id;
		__entry->srcidx = srcidx;
		__entry->start = inst->codec_info->dec_info.stream_rd_ptr;
		__entry->size = size;
	),
	TP_printk("%s: inst[%d] src[%2d] %8x, %d",
		  __get_str(name), __entry->id, __entry->srcidx, __entry->start, __entry->size)
);

TRACE_EVENT(source_change,
	TP_PROTO(struct vpu_instance *inst, struct dec_initial_info *info),
	TP_ARGS(inst, info),
	TP_STRUCT__entry(
		__string(name, dev_name(inst->dev->dev))
		__field(u32, id)
		__field(u32, width)
		__field(u32, height)
		__field(u32, profile)
		__field(u32, level)
		__field(u32, tier)
		__field(u32, min_fb_cnt)
		__field(u32, disp_delay)
		__field(u32, quantization)
		__field(u32, colorspace)
		__field(u32, xfer_func)
		__field(u32, ycbcr_enc)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->id = inst->id;
		__entry->width = info->pic_width,
		__entry->height = info->pic_height,
		__entry->profile = info->profile,
		__entry->level = info->level;
		__entry->tier = info->tier;
		__entry->min_fb_cnt = info->min_frame_buffer_count;
		__entry->disp_delay = info->frame_buf_delay;
		__entry->quantization = inst->quantization;
		__entry->colorspace = inst->colorspace;
		__entry->xfer_func = inst->xfer_func;
		__entry->ycbcr_enc = inst->ycbcr_enc;
	),
	TP_printk("%s: inst[%d] %dx%d profile %d, %d, %d min_fb %d, delay %d, color %d,%d,%d,%d",
		  __get_str(name), __entry->id,
		  __entry->width, __entry->height,
		  __entry->profile, __entry->level, __entry->tier,
		  __entry->min_fb_cnt, __entry->disp_delay,
		  __entry->quantization,
		  __entry->colorspace, __entry->xfer_func, __entry->ycbcr_enc)
);

TRACE_EVENT(dec_done,
	TP_PROTO(struct vpu_instance *inst, struct dec_output_info *info),
	TP_ARGS(inst, info),
	TP_STRUCT__entry(
		__string(name, dev_name(inst->dev->dev))
		__field(u32, id)
		__field(u32, dec_flag)
		__field(u32, dec_poc)
		__field(u32, disp_flag)
		__field(u32, disp_cnt)
		__field(u32, rel_cnt)
		__field(u32, src_ch)
		__field(u32, eos)
		__field(u32, error)
		__field(u32, warn)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->id = inst->id;
		__entry->dec_flag = info->frame_decoded;
		__entry->dec_poc = info->decoded_poc;
		__entry->disp_flag = info->frame_display;
		__entry->disp_cnt = info->disp_frame_num;
		__entry->rel_cnt = info->release_disp_frame_num;
		__entry->src_ch = info->notification_flags & DEC_NOTI_FLAG_SEQ_CHANGE;
		__entry->eos = info->stream_end;
		__entry->error = info->error_reason;
		__entry->warn = info->warn_info;
	),
	TP_printk("%s: inst[%d] dec %d %d; disp %d(%d); rel %d, src_ch %d, eos %d, error 0x%x 0x%x",
		  __get_str(name), __entry->id,
		  __entry->dec_flag, __entry->dec_poc,
		  __entry->disp_flag, __entry->disp_cnt,
		  __entry->rel_cnt,
		  __entry->src_ch, __entry->eos,
		  __entry->error, __entry->warn)
);

TRACE_EVENT(enc_pic,
	TP_PROTO(struct vpu_instance *inst, struct enc_param *param),
	TP_ARGS(inst, param),
	TP_STRUCT__entry(
		__string(name, dev_name(inst->dev->dev))
		__field(u32, id)
		__field(u32, srcidx)
		__field(u32, buf_y)
		__field(u32, buf_cb)
		__field(u32, buf_cr)
		__field(u32, stride)
		__field(u32, buf_strm)
		__field(u32, size_strm)
		__field(u32, force_type_enable)
		__field(u32, force_type)
		__field(u32, end_flag)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->id = inst->id;
		__entry->srcidx = param->src_idx;
		__entry->buf_y = param->source_frame->buf_y;
		__entry->buf_cb = param->source_frame->buf_cb;
		__entry->buf_cr = param->source_frame->buf_cr;
		__entry->stride = param->source_frame->stride;
		__entry->buf_strm = param->pic_stream_buffer_addr;
		__entry->size_strm = param->pic_stream_buffer_size;
		__entry->force_type_enable = param->force_pic_type_enable;
		__entry->force_type = param->force_pic_type;
		__entry->end_flag = param->src_end;
	),
	TP_printk("%s: inst[%d] src[%2d] %8x %8x %8x (%d); dst %8x(%d); force type %d(%d), end %d",
		  __get_str(name), __entry->id, __entry->srcidx,
		  __entry->buf_y, __entry->buf_cb, __entry->buf_cr, __entry->stride,
		  __entry->buf_strm, __entry->size_strm,
		  __entry->force_type_enable, __entry->force_type,
		  __entry->end_flag)
);

TRACE_EVENT(enc_done,
	TP_PROTO(struct vpu_instance *inst, struct enc_output_info *info),
	TP_ARGS(inst, info),
	TP_STRUCT__entry(
		__string(name, dev_name(inst->dev->dev))
		__field(u32, id)
		__field(u32, srcidx)
		__field(u32, frmidx)
		__field(u32, size)
		__field(u32, type)
		__field(u32, avg_qp)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->id = inst->id;
		__entry->srcidx = info->enc_src_idx;
		__entry->frmidx = info->recon_frame_index;
		__entry->size = info->bitstream_size;
		__entry->type = info->pic_type;
		__entry->avg_qp = info->avg_ctu_qp;
	),
	TP_printk("%s: inst[%d] src %d, frame %d, size %d, type %d, qp %d, eos %d",
		  __get_str(name), __entry->id,
		  __entry->srcidx, __entry->frmidx,
		  __entry->size, __entry->type, __entry->avg_qp,
		  __entry->frmidx == RECON_IDX_FLAG_ENC_END)
);

TRACE_EVENT(s_ctrl,
	TP_PROTO(struct vpu_instance *inst, struct v4l2_ctrl *ctrl),
	TP_ARGS(inst, ctrl),
	TP_STRUCT__entry(
		__string(name, dev_name(inst->dev->dev))
		__field(u32, id)
		__string(ctrl_name, ctrl->name)
		__field(u32, val)
	),
	TP_fast_assign(
		__assign_str(name);
		__entry->id = inst->id;
		__assign_str(ctrl_name);
		__entry->val = ctrl->val;
	),
	TP_printk("%s: inst[%d] %s = %d",
		  __get_str(name), __entry->id, __get_str(ctrl_name), __entry->val)
);

#endif /* __WAVE6_TRACE_H__ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE wave6-trace

/* This part must be outside protection */
#include <trace/define_trace.h>
