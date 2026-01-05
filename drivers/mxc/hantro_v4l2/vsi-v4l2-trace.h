/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM vsiv4l2

#if !defined(__VSI_V4L2_TRACE_H__) || defined(TRACE_HEADER_MULTI_READ)
#define __VSI_V4L2_TRACE_H__

#include <linux/tracepoint.h>
#include "vsi-v4l2-priv.h"

DECLARE_EVENT_CLASS(vsiv4l2_msg,
	TP_PROTO(struct vsi_v4l2_msg *msg),
	TP_ARGS(msg),
	TP_STRUCT__entry(
		__field(u64, inst_id)
		__field(u32, cmd_id)
		__field(u64, seq_id)
		__field(u32, format)
		__field(u32, size)
		__field(u32, type)
		__field(s32, inidx)
		__field(s32, outidx)
	),
	TP_fast_assign(
		__entry->inst_id = msg->inst_id;
		__entry->cmd_id = msg->cmd_id;
		__entry->seq_id = msg->seq_id;
		__entry->format = msg->codec_fmt;
		__entry->size = msg->size;
		__entry->type = msg->param_type;
		__entry->inidx = vsi_v4l2_is_bufferdone_msg(msg) ?
				 msg->params.enc_params.io_buffer.inbufidx : -1;
		__entry->outidx = vsi_v4l2_is_bufferdone_msg(msg) ?
				  msg->params.enc_params.io_buffer.outbufidx : -1;

	),
	TP_printk("[%llx] cmd = %s, seq = %lld, format = %d, size = %d, type = %d, idx %d : %d",
		  __entry->inst_id, vsi_v4l2_cmd_name(__entry->cmd_id),
		  __entry->seq_id == NO_RESPONSE_SEQID ? -1 : __entry->seq_id,
		  __entry->format, __entry->size, __entry->type,
		  __entry->inidx, __entry->outidx)
);

DEFINE_EVENT(vsiv4l2_msg, vsiv4l2_command,
	TP_PROTO(struct vsi_v4l2_msg *msg),
	TP_ARGS(msg));

DEFINE_EVENT(vsiv4l2_msg, vsiv4l2_message,
	TP_PROTO(struct vsi_v4l2_msg *msg),
	TP_ARGS(msg));

TRACE_EVENT(vsiv4l2_source_change,
	TP_PROTO(struct vsi_v4l2_mediacfg *cfg, u64 inst_id, u32 change),
	TP_ARGS(cfg, inst_id, change),
	TP_STRUCT__entry(
		__field(u64, id)
		__field(u32, width)
		__field(u32, height)
		__field(u32, bitdepth)
		__field(u32, dpb_num)
		__field(u32, change)
	),
	TP_fast_assign(
		__entry->id = inst_id;
		__entry->width = cfg->decparams.dec_info.io_buffer.srcwidth;
		__entry->height = cfg->decparams.dec_info.io_buffer.srcheight;
		__entry->bitdepth = cfg->src_pixeldepth;
		__entry->dpb_num = cfg->minbuf_4capture;
		__entry->change = change;
	),
	TP_printk("[%llx] source change: %dx%d %d bits, %d dpbs, change 0x%x",
		  __entry->id, __entry->width, __entry->height, __entry->bitdepth,
		  __entry->dpb_num, __entry->change)
);

TRACE_EVENT(vsiv4l2_set_fmt_enc,
	TP_PROTO(struct v4l2_pix_format_mplane *pixmp, u64 id),
	TP_ARGS(pixmp, id),
	TP_STRUCT__entry(
		__field(u64, id)
		__field(u32, pixelformat)
		__field(u32, width)
		__field(u32, height)
		__field(u32, bytesperline)
		__field(u32, sizeimage_0)
		__field(u32, sizeimage_1)
		__field(u32, sizeimage_2)
	),
	TP_fast_assign(
		__entry->id = id;
		__entry->pixelformat = pixmp->pixelformat;
		__entry->width = pixmp->width;
		__entry->height = pixmp->height;
		__entry->bytesperline = pixmp->plane_fmt[0].bytesperline;
		__entry->sizeimage_0 = pixmp->plane_fmt[0].sizeimage;
		__entry->sizeimage_1 = pixmp->plane_fmt[1].sizeimage;
		__entry->sizeimage_2 = pixmp->plane_fmt[2].sizeimage;
	),
	TP_printk("[%llx] %c%c%c%c %dx%d, bytesperline = %d, sizeimage = %d,%d,%d",
		  __entry->id,
		  __entry->pixelformat,
		  __entry->pixelformat >> 8,
		  __entry->pixelformat >> 16,
		  (__entry->pixelformat >> 24) & 0x7f,
		  __entry->width, __entry->height,
		  __entry->bytesperline,
		  __entry->sizeimage_0, __entry->sizeimage_1, __entry->sizeimage_2)
);

TRACE_EVENT(vsiv4l2_set_fmt_dec,
	TP_PROTO(struct v4l2_pix_format *pix, u64 id),
	TP_ARGS(pix, id),
	TP_STRUCT__entry(
		__field(u64, id)
		__field(u32, pixelformat)
		__field(u32, width)
		__field(u32, height)
		__field(u32, bytesperline)
		__field(u32, sizeimage)
	),
	TP_fast_assign(
		__entry->id = id;
		__entry->pixelformat = pix->pixelformat;
		__entry->width = pix->width;
		__entry->height = pix->height;
		__entry->bytesperline = pix->bytesperline;
		__entry->sizeimage = pix->sizeimage;
	),
	TP_printk("[%llx] %c%c%c%c %dx%d, bytesperline = %d, sizeimage = %d",
		  __entry->id,
		  __entry->pixelformat,
		  __entry->pixelformat >> 8,
		  __entry->pixelformat >> 16,
		  (__entry->pixelformat >> 24) & 0x7f,
		  __entry->width, __entry->height,
		  __entry->bytesperline,
		  __entry->sizeimage)
);

DECLARE_EVENT_CLASS(vsiv4l2_stream,
	TP_PROTO(struct vsi_v4l2_ctx *ctx, u32 type),
	TP_ARGS(ctx, type),
	TP_STRUCT__entry(
		__field(u64, id)
		__field(u32, type)
		__field(u32, status)
		__field(u32, isenc)
	),
	TP_fast_assign(
		__entry->id = ctx->ctxid;
		__entry->type = type;
		__entry->status = ctx->status;
		__entry->isenc = isencoder(ctx);
	),
	TP_printk("[%llx]%s %s, status %s",
		  __entry->id,
		  __entry->isenc ? "enc" : "dec",
		  __entry->type ? (V4L2_TYPE_IS_OUTPUT(__entry->type) ? "output" : "capture") : "",
		  vsi_v4l2_status_name(__entry->status))
);

DEFINE_EVENT(vsiv4l2_stream, vsiv4l2_create_ctx,
	TP_PROTO(struct vsi_v4l2_ctx *ctx, u32 type),
	TP_ARGS(ctx, type));

DEFINE_EVENT(vsiv4l2_stream, vsiv4l2_remove_ctx,
	TP_PROTO(struct vsi_v4l2_ctx *ctx, u32 type),
	TP_ARGS(ctx, type));

DEFINE_EVENT(vsiv4l2_stream, vsiv4l2_stream_on,
	TP_PROTO(struct vsi_v4l2_ctx *ctx, u32 type),
	TP_ARGS(ctx, type));

DEFINE_EVENT(vsiv4l2_stream, vsiv4l2_stream_off,
	TP_PROTO(struct vsi_v4l2_ctx *ctx, u32 type),
	TP_ARGS(ctx, type));

DEFINE_EVENT(vsiv4l2_stream, vsiv4l2_cmd_start,
	TP_PROTO(struct vsi_v4l2_ctx *ctx, u32 type),
	TP_ARGS(ctx, type));

DEFINE_EVENT(vsiv4l2_stream, vsiv4l2_cmd_drain,
	TP_PROTO(struct vsi_v4l2_ctx *ctx, u32 type),
	TP_ARGS(ctx, type));

DEFINE_EVENT(vsiv4l2_stream, vsiv4l2_last,
	TP_PROTO(struct vsi_v4l2_ctx *ctx, u32 type),
	TP_ARGS(ctx, type));

TRACE_EVENT(vsiv4l2_buf_queue,
	TP_PROTO(struct vsi_v4l2_ctx *ctx, u32 type, u32 index),
	TP_ARGS(ctx, type, index),
	TP_STRUCT__entry(
		__field(u64, id)
		__field(u32, type)
		__field(u32, status)
		__field(u32, index)
	),
	TP_fast_assign(
		__entry->id = ctx->ctxid;
		__entry->type = type;
		__entry->status = ctx->status;
		__entry->index = index;
	),
	TP_printk("[%llx] %s, status %s, buf index %d",
		  __entry->id,
		  V4L2_TYPE_IS_OUTPUT(__entry->type) ? "output" : "capture",
		  vsi_v4l2_status_name(__entry->status), __entry->index)
);

TRACE_EVENT(vsiv4l2_set_status,
	TP_PROTO(struct vsi_v4l2_ctx *ctx, s32 status),
	TP_ARGS(ctx, status),
	TP_STRUCT__entry(
		__field(u64, id)
		__field(u32, status_1)
		__field(u32, status_2)
	),
	TP_fast_assign(
		__entry->id = ctx->ctxid;
		__entry->status_1 = ctx->status;
		__entry->status_2 = status;
	),
	TP_printk("[%llx] %s -> %s",
		  __entry->id,
		  vsi_v4l2_status_name(__entry->status_1),
		  vsi_v4l2_status_name(__entry->status_2))
);

#endif /* __VSI_V4L2_TRACE_H__ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE vsi-v4l2-trace

/* This part must be outside protection */
#include <trace/define_trace.h>
