/*
 * Copyright 2018-2020 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file vpu_encoder_b0.c
 *
 * copyright here may be changed later
 *
 *
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/file.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>
#include <linux/mx8_mu.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/debugfs.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>

#include "vpu_encoder_b0.h"
#include "vpu_encoder_ctrl.h"
#include "vpu_encoder_config.h"
#include "vpu_event_msg.h"
#include "vpu_encoder_mem.h"
#include "vpu_encoder_mu.h"
#include "vpu_encoder_pm.h"

#define VPU_ENC_DRIVER_VERSION		"1.0.3"

struct vpu_frame_info {
	struct list_head list;
	MEDIAIP_ENC_PIC_INFO info;
	u32 bytesleft;
	u32 wptr;
	u32 rptr;
	u32 start;
	u32 end;
	bool eos;
	bool is_start;
	unsigned long index;
	struct queue_data *queue;
	s64 timestamp;
};

unsigned int vpu_dbg_level_encoder = LVL_ERR | LVL_WARN | LVL_ALL;
static unsigned int reset_on_hang;
static unsigned int show_detail_index = VPU_DETAIL_INDEX_DFT;
static unsigned long debug_firmware_bitmap;

#define ITEM_NAME(name)		\
				[name] = #name

static char *cmd2str[] = {
	ITEM_NAME(GTB_ENC_CMD_NOOP),
	ITEM_NAME(GTB_ENC_CMD_STREAM_START),
	ITEM_NAME(GTB_ENC_CMD_FRAME_ENCODE),
	ITEM_NAME(GTB_ENC_CMD_FRAME_SKIP),
	ITEM_NAME(GTB_ENC_CMD_STREAM_STOP),
	ITEM_NAME(GTB_ENC_CMD_PARAMETER_UPD),
	ITEM_NAME(GTB_ENC_CMD_TERMINATE),
	ITEM_NAME(GTB_ENC_CMD_SNAPSHOT),
	ITEM_NAME(GTB_ENC_CMD_ROLL_SNAPSHOT),
	ITEM_NAME(GTB_ENC_CMD_LOCK_SCHEDULER),
	ITEM_NAME(GTB_ENC_CMD_UNLOCK_SCHEDULER),
	ITEM_NAME(GTB_ENC_CMD_CONFIGURE_CODEC),
	ITEM_NAME(GTB_ENC_CMD_DEAD_MARK),
	ITEM_NAME(GTB_ENC_CMD_FIRM_RESET),
	ITEM_NAME(GTB_ENC_CMD_RESERVED)
};

static char *event2str[] = {
	ITEM_NAME(VID_API_EVENT_UNDEFINED),
	ITEM_NAME(VID_API_ENC_EVENT_RESET_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_START_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_STOP_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_TERMINATE_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_FRAME_INPUT_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_FRAME_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_FRAME_RELEASE),
	ITEM_NAME(VID_API_ENC_EVENT_PARA_UPD_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_MEM_REQUEST),
	ITEM_NAME(VID_API_ENC_EVENT_FIRMWARE_XCPT),
	ITEM_NAME(VID_API_ENC_EVENT_RESERVED)
};

static int wait_for_boot_done(struct core_device *core, int resume);
static void wait_for_start_done(struct vpu_ctx *ctx);
static void wait_for_stop_done(struct vpu_ctx *ctx);
static int sw_reset_firmware(struct core_device *core, int resume);
static int enable_fps_sts(struct vpu_attr *attr);
static int disable_fps_sts(struct vpu_attr *attr);
static int configure_codec(struct vpu_ctx *ctx);
static struct vpu_frame_info *get_idle_frame(struct queue_data *queue);
static void put_frame_idle(struct vpu_frame_info *frame);
static int inc_frame(struct queue_data *queue);
static void dec_frame(struct vpu_frame_info *frame);
static int submit_input_and_encode(struct vpu_ctx *ctx);
static int process_stream_output(struct vpu_ctx *ctx);
static u32 get_ptr(u32 ptr);
static int is_vpu_enc_poweroff(struct core_device *core);

static char *get_event_str(u32 event)
{
	if (event >= VID_API_ENC_EVENT_RESERVED)
		return "UNKNOWN EVENT";
	return event2str[event];
}

static char *get_cmd_str(u32 cmdid)
{
	if (cmdid >= GTB_ENC_CMD_RESERVED)
		return "UNKNOWN CMD";
	return cmd2str[cmdid];
}

static void vpu_log_event(u_int32 uEvent, u_int32 ctxid)
{
	if (uEvent >= VID_API_ENC_EVENT_RESERVED)
		vpu_err("receive event: 0x%X, ctx id:%d\n",
				uEvent, ctxid);
	else
		vpu_dbg(LVL_EVT, "recevie event: %s, ctx id:%d\n",
				event2str[uEvent], ctxid);
}

static void vpu_log_cmd(u_int32 cmdid, u_int32 ctxid)
{
	if (cmdid >= GTB_ENC_CMD_RESERVED)
		vpu_err("send cmd: 0x%X, ctx id:%d\n",
				cmdid, ctxid);
	else
		vpu_dbg(LVL_CMD, "send cmd: %s ctx id:%d\n",
				cmd2str[cmdid], ctxid);
}

static void count_event(struct vpu_ctx *ctx, u32 event)
{
	struct vpu_attr *attr;

	WARN_ON(!ctx);

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return;

	if (event < VID_API_ENC_EVENT_RESERVED)
		attr->statistic.event[event]++;
	else
		attr->statistic.event[VID_API_ENC_EVENT_RESERVED]++;

	attr->statistic.current_event = event;
	ktime_get_raw_ts64(&attr->statistic.ts_event);
}

static void count_cmd(struct vpu_attr *attr, u32 cmdid)
{
	WARN_ON(!attr);

	if (cmdid < GTB_ENC_CMD_RESERVED)
		attr->statistic.cmd[cmdid]++;
	else
		attr->statistic.cmd[GTB_ENC_CMD_RESERVED]++;
	attr->statistic.current_cmd = cmdid;
	ktime_get_raw_ts64(&attr->statistic.ts_cmd);
}

static void count_yuv_input(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = NULL;

	WARN_ON(!ctx);

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return;

	attr->statistic.yuv_count++;
}

static void count_h264_output(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = NULL;

	WARN_ON(!ctx);

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return;

	attr->statistic.h264_count++;
}

static void count_encoded_frame(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = NULL;

	WARN_ON(!ctx);

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return;

	attr->statistic.encoded_count++;
}

static void count_timestamp_overwrite(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = NULL;

	WARN_ON(!ctx);

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return;

	attr->statistic.timestamp_overwrite++;
}

static void write_vpu_reg(struct vpu_dev *dev, u32 val, off_t reg)
{
	writel(val, dev->regs_base + reg);
}

static u32 read_vpu_reg(struct vpu_dev *dev, off_t reg)
{
	return readl(dev->regs_base + reg);
}

/*
 * v4l2 ioctl() operation
 *
 */
static struct vpu_v4l2_fmt  formats_compressed_enc[] = {
	{
		.name       = "H264 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_AVC,
		.is_yuv     = 0,
	},
};

static struct vpu_v4l2_fmt  formats_yuv_enc[] = {
	{
		.name       = "4:2:0 2 Planes Y/CbCr",
		.fourcc     = V4L2_PIX_FMT_NV12,
		.num_planes	= 2,
		.venc_std   = VPU_PF_YUV420_SEMIPLANAR,
		.is_yuv     = 1,
	},
};

static void vpu_ctx_send_cmd(struct vpu_ctx *ctx, uint32_t cmdid,
				uint32_t cmdnum, uint32_t *local_cmddata);

#define GET_CTX_RPC(ctx, func)	\
		func(&ctx->core_dev->shared_mem, ctx->str_index)

pMEDIAIP_ENC_YUV_BUFFER_DESC get_rpc_yuv_buffer_desc(struct vpu_ctx *ctx)
{
	return GET_CTX_RPC(ctx, rpc_get_yuv_buffer_desc);
}

pBUFFER_DESCRIPTOR_TYPE get_rpc_stream_buffer_desc(struct vpu_ctx *ctx)
{
	return GET_CTX_RPC(ctx, rpc_get_stream_buffer_desc);
}

pMEDIAIP_ENC_EXPERT_MODE_PARAM get_rpc_expert_mode_param(struct vpu_ctx *ctx)
{
	return GET_CTX_RPC(ctx, rpc_get_expert_mode_param);
}

pMEDIAIP_ENC_PARAM get_rpc_enc_param(struct vpu_ctx *ctx)
{
	return GET_CTX_RPC(ctx, rpc_get_enc_param);
}

pMEDIAIP_ENC_MEM_POOL get_rpc_mem_pool(struct vpu_ctx *ctx)
{
	return GET_CTX_RPC(ctx, rpc_get_mem_pool);
}

pENC_ENCODING_STATUS get_rpc_encoding_status(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->core_dev)
		return NULL;
	return GET_CTX_RPC(ctx, rpc_get_encoding_status);
}

pENC_DSA_STATUS_t get_rpc_dsa_status(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->core_dev)
		return NULL;
	return GET_CTX_RPC(ctx, rpc_get_dsa_status);
}

static int vpu_enc_v4l2_ioctl_querycap(struct file *file,
		void *fh,
		struct v4l2_capability *cap)
{
	vpu_log_func();
	strlcpy(cap->driver, "vpu encoder", sizeof(cap->driver));
	strlcpy(cap->card, "vpu encoder", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:", sizeof(cap->bus_info));

	return 0;
}

static int vpu_enc_v4l2_ioctl_enum_fmt_vid_cap(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f)
{
	struct vpu_v4l2_fmt *fmt;

	vpu_log_func();

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	if (f->index >= ARRAY_SIZE(formats_compressed_enc))
		return -EINVAL;

	fmt = &formats_compressed_enc[f->index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	f->flags |= V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}
static int vpu_enc_v4l2_ioctl_enum_fmt_vid_out(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f)
{
	struct vpu_v4l2_fmt *fmt;

	vpu_log_func();

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	if (f->index >= ARRAY_SIZE(formats_yuv_enc))
		return -EINVAL;

	fmt = &formats_yuv_enc[f->index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int vpu_enc_v4l2_ioctl_enum_framesizes(struct file *file, void *fh,
					struct v4l2_frmsizeenum *fsize)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct vpu_dev *vdev = ctx->dev;

	if (!fsize)
		return -EINVAL;

	if (fsize->index)
		return -EINVAL;

	if (!vdev)
		return -EINVAL;

	vpu_log_func();
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.max_width = vdev->supported_size.max_width;
	fsize->stepwise.max_height = vdev->supported_size.max_height;
	fsize->stepwise.min_width = vdev->supported_size.min_width;
	fsize->stepwise.min_height = vdev->supported_size.min_height;
	fsize->stepwise.step_width = vdev->supported_size.step_width;
	fsize->stepwise.step_height = vdev->supported_size.step_height;

	return 0;
}

static int vpu_enc_v4l2_ioctl_enum_frameintervals(struct file *file, void *fh,
						struct v4l2_frmivalenum *fival)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct vpu_dev *vdev = ctx->dev;


	if (!fival || fival->index)
		return -EINVAL;
	if (!vdev)
		return -EINVAL;

	vpu_log_func();

	fival->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;
	fival->stepwise.min.numerator = 1;
	fival->stepwise.min.denominator = 65535;
	fival->stepwise.max.numerator = 65535;
	fival->stepwise.max.denominator = 1;
	fival->stepwise.step.numerator = 1;
	fival->stepwise.step.denominator = 1;

	return 0;
}

static struct queue_data *get_queue_by_v4l2_type(struct vpu_ctx *ctx, u32 type)
{
	struct queue_data *queue = NULL;

	if (!ctx)
		return NULL;

	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		queue = &ctx->q_data[V4L2_SRC];
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		queue = &ctx->q_data[V4L2_DST];
		break;
	default:
		vpu_err("unsupport v4l2 buf type : %d\n", type);
		break;
	}

	return queue;
}

static int vpu_enc_v4l2_ioctl_g_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f)
{
	struct vpu_ctx *ctx =           v4l2_fh_to_ctx(fh);
	struct v4l2_pix_format_mplane   *pix_mp = &f->fmt.pix_mp;
	struct queue_data *q_data;
	unsigned int i;

	q_data = get_queue_by_v4l2_type(ctx, f->type);
	if (!q_data)
		return -EINVAL;
	vpu_dbg(LVL_FUNC, "%s(), %s\n", __func__, q_data->desc);

	if (!q_data->current_fmt) {
		vpu_err("%s's current fmt is NULL\n", q_data->desc);
		return -EINVAL;
	}

	pix_mp->pixelformat = q_data->current_fmt->fourcc;
	pix_mp->num_planes = q_data->current_fmt->num_planes;
	pix_mp->width = q_data->width;
	pix_mp->height = q_data->height;
	pix_mp->field = V4L2_FIELD_ANY;
	for (i = 0; i < pix_mp->num_planes; i++)
		pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];

	if (!V4L2_TYPE_IS_OUTPUT(f->type))
		pix_mp->plane_fmt[0].bytesperline = q_data->width;

	pix_mp->colorspace = ctx->colorspace;
	pix_mp->xfer_func = ctx->xfer_func;
	pix_mp->ycbcr_enc = ctx->ycbcr_enc;
	pix_mp->quantization = ctx->quantization;

	return 0;
}

u32 cpu_phy_to_mu(struct core_device *dev, u32 addr)
{
	return addr - dev->m0_p_fw_space_phy;
}

static int initialize_enc_param(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM param = &attr->param;

	mutex_lock(&ctx->instance_mutex);
	param->eCodecMode = MEDIAIP_ENC_FMT_H264;
	param->tEncMemDesc.uMemPhysAddr = 0;
	param->tEncMemDesc.uMemVirtAddr = 0;
	param->tEncMemDesc.uMemSize     = 0;
	param->uSrcStride = VPU_ENC_WIDTH_DEFAULT;
	param->uSrcWidth = VPU_ENC_WIDTH_DEFAULT;
	param->uSrcHeight = VPU_ENC_HEIGHT_DEFAULT;
	param->uSrcOffset_x = 0;
	param->uSrcOffset_y = 0;
	param->uSrcCropWidth = VPU_ENC_WIDTH_DEFAULT;
	param->uSrcCropHeight = VPU_ENC_HEIGHT_DEFAULT;
	param->uOutWidth = VPU_ENC_WIDTH_DEFAULT;
	param->uOutHeight = VPU_ENC_HEIGHT_DEFAULT;
	param->uMinBitRate = BITRATE_LOW_THRESHOLD;

	attr->fival.numerator = 1;
	attr->fival.denominator = VPU_ENC_FRAMERATE_DEFAULT;
	param->uFrameRate = VPU_ENC_FRAMERATE_DEFAULT;

	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int check_stepwise(u32 val, u32 min, u32 max, u32 step)
{
	if (val < min)
		return -EINVAL;
	if (val > max)
		return -EINVAL;
	if ((val - min) % step)
		return -EINVAL;

	return 0;
}

static int check_size(struct vpu_dev *vdev, u32 width, u32 height)
{
	int ret;

	if (!vdev)
		return -EINVAL;

	ret = check_stepwise(width,
			vdev->supported_size.min_width,
			vdev->supported_size.max_width,
			vdev->supported_size.step_width);
	if (ret) {
		vpu_err("Unsupported frame size : %dx%d\n", width, height);
		return -EINVAL;
	}
	ret = check_stepwise(height,
			vdev->supported_size.min_height,
			vdev->supported_size.max_height,
			vdev->supported_size.step_height);
	if (ret) {
		vpu_err("Unsupported frame size : %dx%d\n", width, height);
		return -EINVAL;
	}

	return 0;
}

static int valid_crop_info(struct queue_data *queue, struct v4l2_rect *rect)
{
	struct vpu_ctx *ctx;
	u32 MIN_WIDTH;
	u32 MIN_HEIGHT;

	if (!queue || !rect || !queue->ctx)
		return -EINVAL;

	ctx = queue->ctx;
	MIN_WIDTH = ctx->dev->supported_size.min_width;
	MIN_HEIGHT = ctx->dev->supported_size.min_height;

	if (rect->left > queue->width - MIN_WIDTH ||
		rect->top > queue->height - MIN_HEIGHT) {
		rect->left = 0;
		rect->top = 0;
		rect->width = queue->width;
		rect->height = queue->height;
		return 0;
	}

	rect->width = min(rect->width, queue->width - rect->left);
	if (rect->width)
		rect->width = max_t(u32, rect->width, MIN_WIDTH);
	else
		rect->width = queue->width;
	rect->height = min(rect->height, queue->height - rect->top);
	if (rect->height)
		rect->height = max_t(u32, rect->height, MIN_HEIGHT);
	else
		rect->height = queue->height;

	return 0;
}

static int check_v4l2_fmt(struct vpu_dev *dev, struct v4l2_format *f)
{
	int ret = -EINVAL;

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		ret = check_size(dev, f->fmt.pix.width, f->fmt.pix.height);
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		ret = check_size(dev, f->fmt.pix_mp.width,
					f->fmt.pix_mp.height);
		break;
	default:
		break;
	}

	return ret;
}

static struct vpu_v4l2_fmt *find_fmt_by_fourcc(struct vpu_v4l2_fmt *fmts,
						unsigned int size,
						u32 fourcc)
{
	unsigned int i;

	if (!fmts || !size)
		return NULL;

	for (i = 0; i < size; i++) {
		if (fmts[i].fourcc == fourcc)
			return &fmts[i];
	}

	return NULL;
}

static char *cvrt_fourcc_to_str(u32 pixelformat)
{
	static char str[5];

	str[0] = pixelformat & 0xff;
	str[1] = (pixelformat >> 8) & 0xff;
	str[2] = (pixelformat >> 16) & 0xff;
	str[3] = (pixelformat >> 24) & 0xff;
	str[4] = '\0';

	return str;
}

static const u8 colorprimaries[] = {
	0,
	V4L2_COLORSPACE_REC709,        /*Rec. ITU-R BT.709-6*/
	0,
	0,
	V4L2_COLORSPACE_470_SYSTEM_M, /*Rec. ITU-R BT.470-6 System M*/
	V4L2_COLORSPACE_470_SYSTEM_BG,/*Rec. ITU-R BT.470-6 System B, G*/
	V4L2_COLORSPACE_SMPTE170M,    /*SMPTE170M*/
	V4L2_COLORSPACE_SMPTE240M,    /*SMPTE240M*/
	V4L2_COLORSPACE_GENERIC_FILM, /*Generic film*/
	V4L2_COLORSPACE_BT2020,       /*Rec. ITU-R BT.2020-2*/
	V4L2_COLORSPACE_ST428         /*SMPTE ST 428-1*/
};

static const u8 colortransfers[] = {
	0,
	V4L2_XFER_FUNC_709,      /*Rec. ITU-R BT.709-6*/
	0,
	0,
	V4L2_XFER_FUNC_GAMMA22,  /*Rec. ITU-R BT.470-6 System M*/
	V4L2_XFER_FUNC_GAMMA28,  /*Rec. ITU-R BT.470-6 System B, G*/
	V4L2_XFER_FUNC_709,      /*SMPTE170M*/
	V4L2_XFER_FUNC_SMPTE240M,/*SMPTE240M*/
	V4L2_XFER_FUNC_LINEAR,   /*Linear transfer characteristics*/
	0,
	0,
	V4L2_XFER_FUNC_XVYCC,    /*IEC 61966-2-4*/
	V4L2_XFER_FUNC_BT1361,   /*Rec. ITU-R BT.1361-0 extended colour gamut*/
	V4L2_XFER_FUNC_SRGB,     /*IEC 61966-2-1 sRGB or sYCC*/
	V4L2_XFER_FUNC_709,      /*Rec. ITU-R BT.2020-2 (10 bit system)*/
	V4L2_XFER_FUNC_709,      /*Rec. ITU-R BT.2020-2 (12 bit system)*/
	V4L2_XFER_FUNC_SMPTE2084,/*SMPTE ST 2084*/
	V4L2_XFER_FUNC_ST428,    /*SMPTE ST 428-1*/
	V4L2_XFER_FUNC_HLG       /*Rec. ITU-R BT.2100-0 hybrid log-gamma (HLG)*/
};

static const u8 colormatrixcoefs[] = {
	0,
	V4L2_YCBCR_ENC_709,             /*Rec. ITU-R BT.709-6*/
	0,
	0,
	V4L2_YCBCR_ENC_BT470_6M,        /*Title 47 Code of Federal Regulations*/
	V4L2_YCBCR_ENC_601,             /*Rec. ITU-R BT.601-7 625*/
	V4L2_YCBCR_ENC_601,             /*Rec. ITU-R BT.601-7 525*/
	V4L2_YCBCR_ENC_SMPTE240M,       /*SMPTE240M*/
	0,
	V4L2_YCBCR_ENC_BT2020,          /*Rec. ITU-R BT.2020-2*/
	V4L2_YCBCR_ENC_BT2020_CONST_LUM /*Rec. ITU-R BT.2020-2 constant*/
};

static int vpu_enc_convert_color_v4l2_aspect_to_iso_aspect(struct vpu_ctx *ctx,
		u32 *primaries, u32 *transfer, u32 *coeffs, u32 *fullrange)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(colorprimaries); i++) {
		if (colorprimaries[i] == ctx->colorspace) {
			if (primaries)
				*primaries = i;
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(colortransfers); i++) {
		if (colortransfers[i] == ctx->xfer_func) {
			if (transfer)
				*transfer = i;
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(colormatrixcoefs); i++) {
		if (colormatrixcoefs[i] == ctx->ycbcr_enc) {
			if (coeffs)
				*coeffs = i;
			break;
		}
	}

	if (fullrange)
		*fullrange = ctx->quantization == V4L2_QUANTIZATION_FULL_RANGE;

	return 0;
}

static bool vpu_enc_check_colorspace(u8 colorspace)
{
	int i;

	if (colorspace == V4L2_COLORSPACE_DEFAULT)
		return FALSE;

	for (i = 0; i < ARRAY_SIZE(colorprimaries); i++) {
		if (colorprimaries[i] == colorspace)
			return true;
	}

	return false;
}

static bool vpu_enc_check_xfer_func(u8 xfer_func)
{
	int i;

	if (xfer_func == V4L2_XFER_FUNC_DEFAULT)
		return false;

	for (i = 0; i < ARRAY_SIZE(colortransfers); i++) {
		if (colortransfers[i] == xfer_func)
			return true;
	}

	return false;
}

static bool vpu_enc_check_ycbcr_enc(u8 ycbcr_enc)
{
	int i;

	if (ycbcr_enc == V4L2_YCBCR_ENC_DEFAULT)
		return false;

	for (i = 0; i < ARRAY_SIZE(colormatrixcoefs); i++) {
		if (ycbcr_enc == colormatrixcoefs[i])
			return true;
	}

	return false;
}

static bool vpu_enc_check_quantization(u8 quantization)
{
	bool support = false;

	switch (quantization) {
	case V4L2_QUANTIZATION_FULL_RANGE:
	case V4L2_QUANTIZATION_LIM_RANGE:
		support = true;
		break;
	default:
		break;
	}

	return support;
}

static int vpu_enc_set_default_color(struct vpu_ctx *ctx, u8 colorspace)
{
	ctx->colorspace = colorspace;

	switch (colorspace) {
	case V4L2_COLORSPACE_REC709:
		ctx->xfer_func = V4L2_XFER_FUNC_709;
		ctx->ycbcr_enc = V4L2_YCBCR_ENC_709;
		ctx->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		break;
	case V4L2_COLORSPACE_470_SYSTEM_M:
	case V4L2_COLORSPACE_470_SYSTEM_BG:
	case V4L2_COLORSPACE_SMPTE170M:
		ctx->xfer_func = V4L2_XFER_FUNC_709;
		ctx->ycbcr_enc = V4L2_YCBCR_ENC_601;
		ctx->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		break;
	case V4L2_COLORSPACE_SMPTE240M:
		ctx->xfer_func = V4L2_XFER_FUNC_SMPTE240M;
		ctx->ycbcr_enc = V4L2_YCBCR_ENC_SMPTE240M;
		ctx->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		break;
	case V4L2_COLORSPACE_BT2020:
		ctx->xfer_func = V4L2_XFER_FUNC_709;
		ctx->ycbcr_enc = V4L2_YCBCR_ENC_BT2020;
		ctx->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		break;
	default:
		ctx->xfer_func = V4L2_XFER_FUNC_709;
		ctx->ycbcr_enc = V4L2_YCBCR_ENC_709;
		ctx->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		break;
	}

	return 0;
}

static int vpu_enc_set_color(struct vpu_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp;

	if (!ctx || !f)
		return -EINVAL;

	pix_mp = &f->fmt.pix_mp;
	if (vpu_enc_check_colorspace(pix_mp->colorspace))
		vpu_enc_set_default_color(ctx, pix_mp->colorspace);
	if (vpu_enc_check_xfer_func(pix_mp->xfer_func))
		ctx->xfer_func = pix_mp->xfer_func;
	if (vpu_enc_check_ycbcr_enc(pix_mp->ycbcr_enc))
		ctx->ycbcr_enc = pix_mp->ycbcr_enc;
	if (vpu_enc_check_quantization(pix_mp->quantization))
		ctx->quantization = pix_mp->quantization;

	return 0;
}

static int set_yuv_queue_fmt(struct queue_data *q_data, struct v4l2_format *f)
{
	struct vpu_v4l2_fmt *fmt = NULL;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int i;

	if (!q_data || !f)
		return -EINVAL;

	fmt = find_fmt_by_fourcc(q_data->supported_fmts, q_data->fmt_count,
				pix_mp->pixelformat);
	if (!fmt) {
		vpu_err("unsupport yuv fmt : %s\n",
				cvrt_fourcc_to_str(pix_mp->pixelformat));
		return -EINVAL;
	}

	q_data->width = pix_mp->width;
	q_data->height = pix_mp->height;
	q_data->rect.left = 0;
	q_data->rect.top = 0;
	q_data->rect.width = pix_mp->width;
	q_data->rect.height = pix_mp->height;
	q_data->sizeimage[0] = pix_mp->width * pix_mp->height;
	q_data->sizeimage[1] = pix_mp->width * pix_mp->height / 2;
	pix_mp->num_planes = fmt->num_planes;
	for (i = 0; i < pix_mp->num_planes; i++)
		pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];

	q_data->current_fmt = fmt;

	vpu_enc_set_color(q_data->ctx, f);

	return 0;
}

static u32 get_enc_minimum_sizeimage(u32 width, u32 height)
{
	const u32 THRESHOLD = 256 * 1024;
	u32 sizeimage;

	sizeimage = width * height / 2;
	if (sizeimage < THRESHOLD)
		sizeimage = THRESHOLD;

	return sizeimage;
}

static int set_enc_queue_fmt(struct queue_data *q_data, struct v4l2_format *f)
{
	struct vpu_v4l2_fmt *fmt = NULL;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	u32 sizeimage;

	if (!q_data || !f)
		return -EINVAL;

	fmt = find_fmt_by_fourcc(q_data->supported_fmts, q_data->fmt_count,
				pix_mp->pixelformat);
	if (!fmt) {
		vpu_err("unsupport encode fmt : %s\n",
				cvrt_fourcc_to_str(pix_mp->pixelformat));
		return -EINVAL;
	}

	q_data->width = pix_mp->width;
	q_data->height = pix_mp->height;
	sizeimage = get_enc_minimum_sizeimage(pix_mp->width, pix_mp->height);
	q_data->sizeimage[0] = max(sizeimage, pix_mp->plane_fmt[0].sizeimage);
	pix_mp->plane_fmt[0].sizeimage = q_data->sizeimage[0];

	q_data->current_fmt = fmt;

	return 0;
}

static int vpu_enc_v4l2_ioctl_s_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f)
{
	struct vpu_ctx                  *ctx = v4l2_fh_to_ctx(fh);
	int                             ret = 0;
	struct queue_data               *q_data;
	pMEDIAIP_ENC_PARAM  pEncParam;
	struct vpu_attr *attr;

	attr = get_vpu_ctx_attr(ctx);
	pEncParam = &attr->param;
	q_data = get_queue_by_v4l2_type(ctx, f->type);
	if (!q_data)
		return -EINVAL;
	vpu_dbg(LVL_FUNC, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			ctx->core_dev->id, ctx->str_index);

	ret = check_v4l2_fmt(ctx->dev, f);
	if (ret)
		return ret;

	mutex_lock(&ctx->instance_mutex);
	if (V4L2_TYPE_IS_OUTPUT(f->type))
		ret = set_yuv_queue_fmt(q_data, f);
	else
		ret = set_enc_queue_fmt(q_data, f);
	mutex_unlock(&ctx->instance_mutex);

	f->fmt.pix_mp.colorspace = ctx->colorspace;
	f->fmt.pix_mp.xfer_func = ctx->xfer_func;
	f->fmt.pix_mp.ycbcr_enc = ctx->ycbcr_enc;
	f->fmt.pix_mp.quantization = ctx->quantization;

	vpu_dbg(LVL_FLOW, "[%d:%d] %s set fmt, %c%c%c%c %dx%d\n",
			ctx->core_dev->id,
			ctx->str_index,
			V4L2_TYPE_IS_OUTPUT(f->type) ? "OUTPUT" : "CAPTURE",
			f->fmt.pix_mp.pixelformat,
			f->fmt.pix_mp.pixelformat >> 8,
			f->fmt.pix_mp.pixelformat >> 16,
			f->fmt.pix_mp.pixelformat >> 24,
			f->fmt.pix_mp.width,
			f->fmt.pix_mp.height);
	return ret;
}

static int vpu_enc_v4l2_ioctl_g_parm(struct file *file, void *fh,
				struct v4l2_streamparm *parm)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct vpu_attr *attr = NULL;
	pMEDIAIP_ENC_PARAM param = NULL;

	if (!parm || !ctx)
		return -EINVAL;

	attr = get_vpu_ctx_attr(ctx);
	param = &attr->param;

	vpu_log_func();
	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.capturemode = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.timeperframe.numerator = attr->fival.numerator;
	parm->parm.capture.timeperframe.denominator = attr->fival.denominator;
	parm->parm.capture.readbuffers = 0;

	return 0;
}

static u32 __calc_coprime(u32 *a, u32 *b)
{
	int m = *a;
	int n = *b;

	if (m == 0)
		return n;
	if (n == 0)
		return m;

	while (n != 0) {
		int tmp = m % n;

		m = n;
		n = tmp;
	}
	*a = (*a) / m;
	*b = (*b) / m;

	return m;
}


static int vpu_enc_v4l2_ioctl_s_parm(struct file *file, void *fh,
				struct v4l2_streamparm *parm)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct vpu_attr *attr = NULL;
	struct v4l2_fract fival;

	if (!parm || !ctx)
		return -EINVAL;

	vpu_log_func();
	attr = get_vpu_ctx_attr(ctx);

	fival.numerator = parm->parm.capture.timeperframe.numerator;
	fival.denominator = parm->parm.capture.timeperframe.denominator;
	if (!fival.numerator || !fival.denominator)
		return -EINVAL;

	__calc_coprime(&fival.numerator, &fival.denominator);
	mutex_lock(&ctx->instance_mutex);
	attr->fival.numerator = fival.numerator;
	attr->fival.denominator = fival.denominator;
	attr->param.uFrameRate =
		DIV_ROUND_CLOSEST(fival.denominator, fival.numerator);
	mutex_unlock(&ctx->instance_mutex);
	vpu_dbg(LVL_FLOW, "[%d:%d] %s set frame interval : %d / %d\n",
			ctx->core_dev->id,
			ctx->str_index,
			V4L2_TYPE_IS_OUTPUT(parm->type) ? "OUTPUT" : "CAPTURE",
			fival.numerator,
			fival.denominator);

	return 0;
}

static int vpu_enc_queue_expbuf(struct queue_data *queue,
				struct v4l2_exportbuffer *buf)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_expbuf(&queue->vb2_q, buf);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_enc_queue_reqbufs(struct queue_data *queue,
				struct v4l2_requestbuffers *reqbuf)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_reqbufs(&queue->vb2_q, reqbuf);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_enc_queue_querybuf(struct queue_data *queue,
				struct v4l2_buffer *buf)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_querybuf(&queue->vb2_q, buf);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_enc_queue_qbuf(struct queue_data *queue,
				struct v4l2_buffer *buf)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
		ret = vb2_qbuf(&queue->vb2_q, queue->ctx->dev->v4l2_dev.mdev, buf);
#else
		ret = vb2_qbuf(&queue->vb2_q, buf);
#endif
	}
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_enc_queue_dqbuf(struct queue_data *queue,
				struct v4l2_buffer *buf, bool nonblocking)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_dqbuf(&queue->vb2_q, buf, nonblocking);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_enc_queue_enable(struct queue_data *queue,
				enum v4l2_buf_type type)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_streamon(&queue->vb2_q, type);
	up(&queue->drv_q_lock);

	return ret;
}

static void clear_queue(struct queue_data *queue)
{
	struct vpu_frame_info *frame;
	struct vpu_frame_info *tmp;
	struct vb2_data_req *p_data_req;
	struct vb2_data_req *p_temp;
	struct vb2_buffer *vb;

	if (!queue)
		return;

	list_for_each_entry_safe(frame, tmp, &queue->frame_q, list) {
		vpu_dbg(LVL_INFO, "drop frame\n");
		put_frame_idle(frame);
	}

	list_for_each_entry_safe(frame, tmp, &queue->frame_idle, list)
		dec_frame(frame);

	list_for_each_entry_safe(p_data_req, p_temp, &queue->drv_q, list) {
		vpu_dbg(LVL_DEBUG, "%s(%d) - list_del(%p)\n", __func__,
				p_data_req->sequence, p_data_req);
		list_del(&p_data_req->list);
	}
	list_for_each_entry(vb, &queue->vb2_q.queued_list, queued_entry) {
		if (vb->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	}

	INIT_LIST_HEAD(&queue->drv_q);
	INIT_LIST_HEAD(&queue->frame_q);
	INIT_LIST_HEAD(&queue->frame_idle);
}

static int vpu_enc_queue_disable(struct queue_data *queue,
				enum v4l2_buf_type type)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_streamoff(&queue->vb2_q, type);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_enc_queue_release(struct queue_data *queue)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited) {
		clear_queue(queue);
		vb2_queue_release(&queue->vb2_q);
	}
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_enc_queue_mmap(struct queue_data *queue,
				struct vm_area_struct *vma)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_mmap(&queue->vb2_q, vma);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_enc_v4l2_ioctl_expbuf(struct file *file,
		void *fh,
		struct v4l2_exportbuffer *buf)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;

	q_data = get_queue_by_v4l2_type(ctx, buf->type);
	if (!q_data)
		return -EINVAL;
	vpu_dbg(LVL_FUNC, "%s(), %s\n", __func__, q_data->desc);

	return vpu_enc_queue_expbuf(q_data, buf);
}

static int vpu_enc_v4l2_ioctl_subscribe_event(struct v4l2_fh *fh,
		const struct v4l2_event_subscription *sub
		)
{
	vpu_log_func();

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return -EINVAL;
	}
}

static int vpu_enc_v4l2_ioctl_reqbufs(struct file *file,
		void *fh,
		struct v4l2_requestbuffers *reqbuf)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	q_data = get_queue_by_v4l2_type(ctx, reqbuf->type);
	if (!q_data)
		return -EINVAL;
	vpu_dbg(LVL_FUNC, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			ctx->core_dev->id, ctx->str_index);
	vpu_dbg(LVL_FLOW, "[%d:%d] %s reqbufs : %d\n",
		ctx->core_dev->id,
		ctx->str_index,
		V4L2_TYPE_IS_OUTPUT(reqbuf->type) ? "OUTPUT" : "CATPURE",
		reqbuf->count);

	ret = vpu_enc_queue_reqbufs(q_data, reqbuf);

	return ret;
}

static int vpu_enc_v4l2_ioctl_querybuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	unsigned int i;
	int ret;

	q_data = get_queue_by_v4l2_type(ctx, buf->type);
	if (!q_data)
		return -EINVAL;
	vpu_dbg(LVL_FUNC, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			ctx->core_dev->id, ctx->str_index);

	ret = vpu_enc_queue_querybuf(q_data, buf);
	if (ret)
		return ret;

	if (buf->memory == V4L2_MEMORY_MMAP) {
		if (V4L2_TYPE_IS_MULTIPLANAR(buf->type)) {
			for (i = 0; i < buf->length; i++)
				buf->m.planes[i].m.mem_offset |= (q_data->type << MMAP_BUF_TYPE_SHIFT);
		} else
			buf->m.offset |= (q_data->type << MMAP_BUF_TYPE_SHIFT);
	}

	return ret;
}

static struct vb2_buffer *cvrt_v4l2_to_vb2_buffer(struct vb2_queue *vq,
						struct v4l2_buffer *buf)
{
	if (!vq || !buf)
		return NULL;

	if (buf->index >= vq->num_buffers)
		return NULL;

	return vq->bufs[buf->index];
}

static u32 get_v4l2_plane_payload(struct v4l2_plane *plane)
{
	return plane->bytesused - plane->data_offset;
}

static void set_v4l2_plane_payload(struct v4l2_plane *plane, u32 size)
{
	plane->bytesused = plane->data_offset + size;
}

static int is_valid_output_mplane_buf(struct queue_data *q_data,
					struct vpu_v4l2_fmt *fmt,
					struct v4l2_buffer *buf)
{
	int i;

	for (i = 0; i < fmt->num_planes; i++) {
		u32 bytesused = get_v4l2_plane_payload(&buf->m.planes[i]);

		if (!bytesused) {
			set_v4l2_plane_payload(&buf->m.planes[i],
						q_data->sizeimage[i]);
			continue;
		}
		if (fmt->is_yuv && bytesused != q_data->sizeimage[i])
			return 0;
	}

	return 1;
}

static int is_valid_output_buf(struct queue_data *q_data,
				struct vpu_v4l2_fmt *fmt,
				struct v4l2_buffer *buf)
{
	if (!buf->bytesused) {
		buf->bytesused = q_data->sizeimage[0];
		return 1;
	}
	if (fmt->is_yuv && buf->bytesused != q_data->sizeimage[0])
		return 0;

	return 1;
}

static int precheck_qbuf(struct queue_data *q_data, struct v4l2_buffer *buf)
{
	struct vb2_buffer *vb = NULL;
	struct vpu_v4l2_fmt *fmt;
	int ret;

	if (!q_data || !buf)
		return -EINVAL;

	if (!q_data->current_fmt)
		return -EINVAL;

	vb = cvrt_v4l2_to_vb2_buffer(&q_data->vb2_q, buf);
	if (!vb) {
		vpu_err("invalid v4l2 buffer index:%d\n", buf->index);
		return -EINVAL;
	}
	if (vb->state != VB2_BUF_STATE_DEQUEUED) {
		vpu_err("invalid buffer state:%d\n", vb->state);
		return -EINVAL;
	}

	if (!V4L2_TYPE_IS_OUTPUT(buf->type))
		return 0;

	fmt = q_data->current_fmt;
	if (V4L2_TYPE_IS_MULTIPLANAR(buf->type))
		ret = is_valid_output_mplane_buf(q_data, fmt, buf);
	else
		ret = is_valid_output_buf(q_data, fmt, buf);
	if (!ret)
		return -EINVAL;

	return 0;
}

static int vpu_enc_v4l2_ioctl_qbuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	q_data = get_queue_by_v4l2_type(ctx, buf->type);
	if (!q_data)
		return -EINVAL;
	vpu_dbg(LVL_FUNC, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			ctx->core_dev->id, ctx->str_index);

	ret = precheck_qbuf(q_data, buf);
	if (ret < 0)
		return ret;

	ret = vpu_enc_queue_qbuf(q_data, buf);
	if (ret)
		return ret;

	if (V4L2_TYPE_IS_OUTPUT(buf->type)) {
		mutex_lock(&ctx->dev->dev_mutex);
		mutex_lock(&ctx->instance_mutex);
		if (!test_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status))
			set_bit(VPU_ENC_STATUS_DATA_READY, &ctx->status);
		configure_codec(ctx);
		mutex_unlock(&ctx->instance_mutex);
		mutex_unlock(&ctx->dev->dev_mutex);
		wait_for_start_done(ctx);

		submit_input_and_encode(ctx);
		count_yuv_input(ctx);
	} else {
		process_stream_output(ctx);
	}

	return ret;
}

static void notify_eos(struct vpu_ctx *ctx)
{
	const struct v4l2_event ev = {
		.type = V4L2_EVENT_EOS
	};

	mutex_lock(&ctx->instance_mutex);
	if (!test_bit(VPU_ENC_STATUS_CLOSED, &ctx->status) &&
		!test_and_set_bit(VPU_ENC_STATUS_EOS_SEND, &ctx->status)) {
		v4l2_event_queue_fh(&ctx->fh, &ev);
		vpu_dbg(LVL_FLOW, "[%d:%d] send eos event\n",
				ctx->core_dev->id, ctx->str_index);
	}
	mutex_unlock(&ctx->instance_mutex);
}

static int send_eos(struct vpu_ctx *ctx)
{
	if (!ctx)
		return -EINVAL;

	if (!test_bit(VPU_ENC_STATUS_START_SEND, &ctx->status)) {
		notify_eos(ctx);
		return 0;
	}

	if (!test_and_set_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status)) {
		reinit_completion(&ctx->stop_cmp);
		vpu_ctx_send_cmd(ctx, GTB_ENC_CMD_STREAM_STOP, 0, NULL);
		vpu_dbg(LVL_FLOW, "[%d:%d] stop stream\n",
				ctx->core_dev->id, ctx->str_index);
	}

	return 0;
}

static int vpu_enc_v4l2_ioctl_dqbuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	q_data = get_queue_by_v4l2_type(ctx, buf->type);
	if (!q_data)
		return -EINVAL;
	vpu_dbg(LVL_FUNC, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			ctx->core_dev->id, ctx->str_index);

	ret = vpu_enc_queue_dqbuf(q_data, buf, file->f_flags & O_NONBLOCK);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (!ret)
			count_h264_output(ctx);
	}

	return ret;
}

static bool format_is_support(struct vpu_v4l2_fmt *format_table,
		unsigned int table_size,
		struct v4l2_format *f)
{
	unsigned int i;

	for (i = 0; i < table_size; i++) {
		if (format_table[i].fourcc == f->fmt.pix_mp.pixelformat)
			return true;
	}
	return false;
}

static int vpu_enc_v4l2_ioctl_try_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct v4l2_pix_format_mplane   *pix_mp = &f->fmt.pix_mp;
	struct queue_data *q_data;

	q_data = get_queue_by_v4l2_type(ctx, f->type);
	if (!q_data)
		return -EINVAL;
	vpu_dbg(LVL_FUNC, "%s(), %s\n", __func__, q_data->desc);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pix_mp->field = V4L2_FIELD_ANY;
		pix_mp->colorspace = V4L2_COLORSPACE_REC709;
		if (!vpu_enc_check_colorspace(pix_mp->colorspace)) {
			pix_mp->colorspace = ctx->colorspace;
			pix_mp->xfer_func = ctx->xfer_func;
			pix_mp->ycbcr_enc = ctx->ycbcr_enc;
			pix_mp->quantization = ctx->quantization;
		} else {
			if (!vpu_enc_check_xfer_func(pix_mp->xfer_func))
				pix_mp->xfer_func = ctx->xfer_func;
			if (!vpu_enc_check_ycbcr_enc(pix_mp->ycbcr_enc))
				pix_mp->ycbcr_enc = ctx->ycbcr_enc;
			if (!vpu_enc_check_quantization(pix_mp->quantization))
				pix_mp->quantization = ctx->quantization;
		}
	} else {
		pix_mp->colorspace = ctx->colorspace;
		pix_mp->xfer_func = ctx->xfer_func;
		pix_mp->ycbcr_enc = ctx->ycbcr_enc;
		pix_mp->quantization = ctx->quantization;
	}

	if (!format_is_support(q_data->supported_fmts, q_data->fmt_count, f))
		return -EINVAL;

	return 0;
}

static int vpu_enc_v4l2_ioctl_g_selection(struct file *file, void *fh,
					struct v4l2_selection *s)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *src = &ctx->q_data[V4L2_SRC];

	if (s->target != V4L2_SEL_TGT_CROP && s->target != V4L2_SEL_TGT_COMPOSE)
		return -EINVAL;

	vpu_log_func();
	s->r = src->rect;

	return 0;
}

static int vpu_enc_v4l2_ioctl_s_selection(struct file *file, void *fh,
					struct v4l2_selection *s)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *src = &ctx->q_data[V4L2_SRC];
	struct vpu_dev *dev = ctx->dev;

	if (s->target != V4L2_SEL_TGT_CROP && s->target != V4L2_SEL_TGT_COMPOSE)
		return -EINVAL;

	vpu_log_func();
	src->rect.left = ALIGN(s->r.left, dev->supported_size.step_width);
	src->rect.top = ALIGN(s->r.top, dev->supported_size.step_height);
	src->rect.width = ALIGN(s->r.width, dev->supported_size.step_width);
	src->rect.height = ALIGN(s->r.height, dev->supported_size.step_height);
	valid_crop_info(src, &src->rect);

	return 0;
}

static int response_stop_stream(struct vpu_ctx *ctx)
{
	struct queue_data *queue;

	if (!ctx)
		return -EINVAL;

	queue = &ctx->q_data[V4L2_SRC];

	down(&queue->drv_q_lock);
	if (!list_empty(&queue->drv_q))
		goto exit;

	if (!test_bit(VPU_ENC_FLAG_WRITEABLE, &queue->rw_flag))
		goto exit;
	if (test_and_clear_bit(VPU_ENC_STATUS_STOP_REQ, &ctx->status))
		send_eos(ctx);
exit:
	up(&queue->drv_q_lock);

	return 0;
}

static int request_eos(struct vpu_ctx *ctx)
{
	WARN_ON(!ctx);

	vpu_dbg(LVL_FLOW, "[%d:%d] request eos\n",
			ctx->core_dev->id, ctx->str_index);
	set_bit(VPU_ENC_STATUS_STOP_REQ, &ctx->status);
	response_stop_stream(ctx);

	return 0;
}

static int set_core_force_release(struct core_device *core)
{
	int i;

	if (!core)
		return -EINVAL;

	for (i = 0; i < core->supported_instance_count; i++) {
		if (!core->ctx[i])
			continue;
		set_bit(VPU_ENC_STATUS_FORCE_RELEASE, &core->ctx[i]->status);
	}

	return 0;
}

static void clear_start_status(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;

	clear_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status);
	clear_bit(VPU_ENC_STATUS_START_SEND, &ctx->status);
	clear_bit(VPU_ENC_STATUS_START_DONE, &ctx->status);
}

static void clear_stop_status(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;

	clear_bit(VPU_ENC_STATUS_STOP_REQ, &ctx->status);
	clear_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status);
	clear_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status);
	clear_bit(VPU_ENC_STATUS_EOS_SEND, &ctx->status);
}

static void reset_core_on_hang(struct core_device *core)
{
	int ret;
	int i;

	for (i = 0; i < core->supported_instance_count; i++)
		clear_start_status(core->ctx[i]);

	ret = sw_reset_firmware(core, 1);
	if (ret)
		vpu_err("reset core[%d] on hang fail\n", core->id);
}

static int set_core_hang(struct core_device *core)
{
	core->hang = true;

	if (reset_on_hang)
		reset_core_on_hang(core);

	return 0;
}

static void clear_core_hang(struct core_device *core)
{
	if (!core)
		return;

	core->hang = false;
}

static void wait_for_start_done(struct vpu_ctx *ctx)
{
	int ret;

	WARN_ON(!ctx);

	if (!test_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status))
		return;
	if (test_bit(VPU_ENC_STATUS_START_DONE, &ctx->status))
		return;
	ret = wait_for_completion_timeout(&ctx->start_cmp,
					msecs_to_jiffies(300));
	if (!ret && !test_bit(VPU_ENC_STATUS_START_DONE, &ctx->status))
		vpu_err("wait for start done timeout\n");
}

static void wait_for_stop_done(struct vpu_ctx *ctx)
{
	int ret;

	WARN_ON(!ctx);

	if (!test_bit(VPU_ENC_STATUS_START_SEND, &ctx->status))
		return;
	if (test_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status))
		return;

	ret = wait_for_completion_timeout(&ctx->stop_cmp,
						msecs_to_jiffies(500));
	if (!ret && !test_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status))
		vpu_err("wait for stop done timeout\n");
}

static int vpu_enc_v4l2_ioctl_encoder_cmd(struct file *file,
		void *fh,
		struct v4l2_encoder_cmd *cmd
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);

	vpu_dbg(LVL_FUNC, "%s(), cmd = %d, (%d, %d)\n", __func__, cmd->cmd,
			ctx->core_dev->id, ctx->str_index);
	switch (cmd->cmd) {
	case V4L2_ENC_CMD_START:
		vpu_dbg(LVL_FLOW, "[%d:%d] start encoder\n",
				ctx->core_dev->id, ctx->str_index);
		vb2_clear_last_buffer_dequeued(&ctx->q_data[V4L2_DST].vb2_q);
		break;
	case V4L2_ENC_CMD_STOP:
		vpu_dbg(LVL_FLOW, "[%d:%d] stop encoder\n",
				ctx->core_dev->id, ctx->str_index);
		request_eos(ctx);
		break;
	case V4L2_ENC_CMD_PAUSE:
		vpu_dbg(LVL_FLOW, "[%d:%d] pause encoder\n",
				ctx->core_dev->id, ctx->str_index);
		break;
	case V4L2_ENC_CMD_RESUME:
		vpu_dbg(LVL_FLOW, "[%d:%d] resume encoder\n",
				ctx->core_dev->id, ctx->str_index);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int vpu_enc_v4l2_ioctl_streamon(struct file *file,
		void *fh,
		enum v4l2_buf_type i
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct vpu_attr *attr;
	struct queue_data *q_data;
	int ret;

	q_data = get_queue_by_v4l2_type(ctx, i);
	if (!q_data)
		return -EINVAL;
	vpu_dbg(LVL_FUNC, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			ctx->core_dev->id, ctx->str_index);
	vpu_dbg(LVL_FLOW, "[%d:%d] %s streamon\n",
			ctx->core_dev->id, ctx->str_index,
			V4L2_TYPE_IS_OUTPUT(i) ? "OUTPUT" : "CAPTURE");

	attr = get_vpu_ctx_attr(ctx);
	if (attr) {
		attr->ts_start[V4L2_SRC] = 0;
		attr->ts_start[V4L2_DST] = 0;
	}

	ret = vpu_enc_queue_enable(q_data, i);
	if (ret)
		return ret;

	if (V4L2_TYPE_IS_OUTPUT(i)) {
		mutex_lock(&ctx->dev->dev_mutex);
		mutex_lock(&ctx->instance_mutex);
		set_bit(VPU_ENC_STATUS_OUTPUT_READY, &ctx->status);
		configure_codec(ctx);
		mutex_unlock(&ctx->instance_mutex);
		mutex_unlock(&ctx->dev->dev_mutex);
		wait_for_start_done(ctx);
	}

	return 0;
}

static int vpu_enc_v4l2_ioctl_streamoff(struct file *file,
		void *fh,
		enum v4l2_buf_type i)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	q_data = get_queue_by_v4l2_type(ctx, i);
	if (!q_data)
		return -EINVAL;

	vpu_dbg(LVL_FUNC, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			ctx->core_dev->id, ctx->str_index);
	vpu_dbg(LVL_FLOW, "[%d:%d] %s streamoff\n",
			ctx->core_dev->id, ctx->str_index,
			V4L2_TYPE_IS_OUTPUT(i) ? "OUTPUT" : "CAPTURE");

	request_eos(ctx);
	wait_for_stop_done(ctx);

	ret = vpu_enc_queue_disable(q_data, i);

	return ret;
}

static const struct v4l2_ioctl_ops vpu_enc_v4l2_ioctl_ops = {
	.vidioc_querycap                = vpu_enc_v4l2_ioctl_querycap,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	.vidioc_enum_fmt_vid_cap	= vpu_enc_v4l2_ioctl_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out	= vpu_enc_v4l2_ioctl_enum_fmt_vid_out,
#else
	.vidioc_enum_fmt_vid_cap_mplane	= vpu_enc_v4l2_ioctl_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out_mplane	= vpu_enc_v4l2_ioctl_enum_fmt_vid_out,
#endif
	.vidioc_enum_framesizes		= vpu_enc_v4l2_ioctl_enum_framesizes,
	.vidioc_enum_frameintervals	= vpu_enc_v4l2_ioctl_enum_frameintervals,
	.vidioc_g_fmt_vid_cap_mplane    = vpu_enc_v4l2_ioctl_g_fmt,
	.vidioc_g_fmt_vid_out_mplane    = vpu_enc_v4l2_ioctl_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane  = vpu_enc_v4l2_ioctl_try_fmt,
	.vidioc_try_fmt_vid_out_mplane  = vpu_enc_v4l2_ioctl_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane    = vpu_enc_v4l2_ioctl_s_fmt,
	.vidioc_s_fmt_vid_out_mplane    = vpu_enc_v4l2_ioctl_s_fmt,
	.vidioc_g_parm			= vpu_enc_v4l2_ioctl_g_parm,
	.vidioc_s_parm			= vpu_enc_v4l2_ioctl_s_parm,
	.vidioc_expbuf                  = vpu_enc_v4l2_ioctl_expbuf,
	.vidioc_g_selection		= vpu_enc_v4l2_ioctl_g_selection,
	.vidioc_s_selection		= vpu_enc_v4l2_ioctl_s_selection,
	.vidioc_encoder_cmd             = vpu_enc_v4l2_ioctl_encoder_cmd,
	.vidioc_subscribe_event         = vpu_enc_v4l2_ioctl_subscribe_event,
	.vidioc_unsubscribe_event       = v4l2_event_unsubscribe,
	.vidioc_reqbufs                 = vpu_enc_v4l2_ioctl_reqbufs,
	.vidioc_querybuf                = vpu_enc_v4l2_ioctl_querybuf,
	.vidioc_qbuf                    = vpu_enc_v4l2_ioctl_qbuf,
	.vidioc_dqbuf                   = vpu_enc_v4l2_ioctl_dqbuf,
	.vidioc_streamon                = vpu_enc_v4l2_ioctl_streamon,
	.vidioc_streamoff               = vpu_enc_v4l2_ioctl_streamoff,
};

static void vpu_core_send_cmd(struct core_device *core, u32 idx,
				u32 cmdid, u32 cmdnum, u32 *local_cmddata)
{
	WARN_ON(!core || idx >= VID_API_NUM_STREAMS);

	vpu_log_cmd(cmdid, idx);
	count_cmd(&core->attr[idx], cmdid);

	mutex_lock(&core->cmd_mutex);
	rpc_send_cmd_buf_encoder(&core->shared_mem, idx, cmdid, cmdnum,
				 local_cmddata);
	mb();
	vpu_enc_mu_send_msg(core, COMMAND, 0xffff);
	mutex_unlock(&core->cmd_mutex);
}

static void vpu_ctx_send_cmd(struct vpu_ctx *ctx, uint32_t cmdid,
				uint32_t cmdnum, uint32_t *local_cmddata)
{
	vpu_core_send_cmd(ctx->core_dev, ctx->str_index,
				cmdid, cmdnum, local_cmddata);
}

static void set_core_fw_status(struct core_device *core, bool status)
{
	core->fw_is_ready = status;
}

static int reset_vpu_core_dev(struct core_device *core_dev)
{
	if (!core_dev)
		return -EINVAL;

	set_core_fw_status(core_dev, false);
	core_dev->firmware_started = false;

	return 0;
}

static int sw_reset_firmware(struct core_device *core, int resume)
{
	int ret = 0;

	WARN_ON(!core);

	vpu_dbg(LVL_INFO, "core[%d] sw reset firmware\n", core->id);

	kfifo_reset(&core->mu_msg_fifo);

	reinit_completion(&core->boot_cmp);
	vpu_core_send_cmd(core, 0, GTB_ENC_CMD_FIRM_RESET, 0, NULL);
	ret = wait_for_boot_done(core, resume);
	if (ret) {
		set_core_hang(core);
		return -EINVAL;
	}
	core->reset_times++;

	return 0;
}

static int process_core_hang(struct core_device *core)
{
	int ret;
	int i;
	int instance_count = 0;

	if (!core->hang)
		return 0;

	for (i = 0; i < core->supported_instance_count; i++) {
		if (core->ctx[i])
			instance_count++;
	}

	if (instance_count)
		return -EBUSY;

	ret = sw_reset_firmware(core, 0);
	if (ret)
		return ret;

	clear_core_hang(core);
	return 0;
}

static void show_codec_configure(pMEDIAIP_ENC_PARAM param,
		pMEDIAIP_ENC_EXPERT_MODE_PARAM pEncExpertModeParam)
{
	if (!param)
		return;

	vpu_dbg(LVL_INFO, "Encoder Parameter:\n");
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Codec Mode", param->eCodecMode);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Profile", param->eProfile);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Level", param->uLevel);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Mem Phys Addr", param->tEncMemDesc.uMemPhysAddr);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Mem Virt Addr", param->tEncMemDesc.uMemVirtAddr);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Mem Size", param->tEncMemDesc.uMemSize);
	vpu_dbg(LVL_INFO, "\t%20s:%16d(%d / %d)\n",
			"Frame Rate", param->uFrameRate,
			pEncExpertModeParam->Config.frame_rate_num,
			pEncExpertModeParam->Config.frame_rate_den);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Stride", param->uSrcStride);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Width", param->uSrcWidth);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Height", param->uSrcHeight);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Offset x", param->uSrcOffset_x);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Offset y", param->uSrcOffset_y);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Crop Width", param->uSrcCropWidth);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Crop Height", param->uSrcCropHeight);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Out Width", param->uOutWidth);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Out Height", param->uOutHeight);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"I Frame Interval", param->uIFrameInterval);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"GOP Length", param->uGopBLength);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Low Latency Mode", param->uLowLatencyMode);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Bitrate Mode", param->eBitRateMode);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Target Bitrate", param->uTargetBitrate);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Min Bitrate", param->uMinBitRate);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Max Bitrate", param->uMaxBitRate);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"QP", param->uInitSliceQP);
}

static void show_firmware_version(struct core_device *core_dev,
				unsigned int level)
{
	pENC_RPC_HOST_IFACE pSharedInterface;

	if (!core_dev)
		return;

	pSharedInterface = core_dev->shared_mem.pSharedInterface;

	vpu_dbg(level, "vpu encoder core[%d] firmware version is %d.%d.%d\n",
			core_dev->id,
			(pSharedInterface->FWVersion & 0x00ff0000) >> 16,
			(pSharedInterface->FWVersion & 0x0000ff00) >> 8,
			pSharedInterface->FWVersion & 0x000000ff);
}

static void update_encode_size(struct vpu_ctx *ctx)
{
	struct queue_data *src = NULL;
	struct queue_data *dst = NULL;
	struct vpu_attr *attr;
	pMEDIAIP_ENC_PARAM  pEncParam;

	if (!ctx)
		return;

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return;

	src = &ctx->q_data[V4L2_SRC];
	dst = &ctx->q_data[V4L2_DST];
	pEncParam = &attr->param;

	pEncParam->uSrcStride           = src->width;
	pEncParam->uSrcWidth            = src->width;
	pEncParam->uSrcHeight           = src->height;
	pEncParam->uSrcOffset_x         = src->rect.left;
	pEncParam->uSrcOffset_y         = src->rect.top;
	pEncParam->uSrcCropWidth        = src->rect.width;
	pEncParam->uSrcCropHeight       = src->rect.height;
	pEncParam->uOutWidth            = min(dst->width, src->rect.width);
	pEncParam->uOutHeight           = min(dst->height, src->rect.height);
}

static void init_ctx_seq_info(struct vpu_ctx *ctx)
{
	int i;

	if (!ctx)
		return;

	ctx->sequence = 0;
	for (i = 0; i < ARRAY_SIZE(ctx->timestams); i++)
		ctx->timestams[i] = VPU_ENC_INVALID_TIMESTAMP;
	ctx->timestamp = VPU_ENC_INVALID_TIMESTAMP;
}

static void fill_ctx_seq(struct vpu_ctx *ctx, struct vb2_data_req *p_data_req)
{
	u_int32 idx;

	WARN_ON(!ctx || !p_data_req);

	p_data_req->sequence = ctx->sequence++;
	idx = p_data_req->sequence % VPU_ENC_SEQ_CAPACITY;
	if (ctx->timestams[idx] != VPU_ENC_INVALID_TIMESTAMP) {
		count_timestamp_overwrite(ctx);
		vpu_dbg(LVL_FRAME, "[%d:%d][%d] overwrite timestamp\n",
			ctx->core_dev->id, ctx->str_index,
			p_data_req->sequence);
	}
	ctx->timestams[idx] = p_data_req->vb2_buf->timestamp;
	if (ctx->timestamp < (s64)p_data_req->vb2_buf->timestamp)
		ctx->timestamp = p_data_req->vb2_buf->timestamp;
}

static s64 get_ctx_seq_timestamp(struct vpu_ctx *ctx, u32 sequence)
{
	s64 timestamp;
	u_int32 idx;

	WARN_ON(!ctx);

	idx = sequence % VPU_ENC_SEQ_CAPACITY;
	timestamp = ctx->timestams[idx];
	ctx->timestams[idx] = VPU_ENC_INVALID_TIMESTAMP;

	return timestamp;
}

static void fill_vb_sequence(struct vb2_buffer *vb, u32 sequence)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->sequence = sequence;
}

static void set_vb_flags(struct vb2_buffer *vb, u32 buffer_flags)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->flags |= buffer_flags;
}

static struct vb2_data_req *find_vb2_data_by_sequence(struct queue_data *queue,
							u32 sequence)
{
	int i;

	for (i = 0; i < queue->vb2_q.num_buffers; i++) {
		if (!queue->vb2_reqs[i].vb2_buf)
			continue;
		if (queue->vb2_reqs[i].sequence == sequence)
			return &queue->vb2_reqs[i];
	}

	return NULL;
}

static void update_stream_desc_rptr(struct vpu_ctx *ctx, u32 rptr)
{
	pBUFFER_DESCRIPTOR_TYPE stream_buffer_desc;

	stream_buffer_desc = get_rpc_stream_buffer_desc(ctx);
	stream_buffer_desc->rptr = rptr;
}

static int do_configure_codec(struct vpu_ctx *ctx)
{
	pBUFFER_DESCRIPTOR_TYPE pEncStrBuffDesc = NULL;
	pMEDIAIP_ENC_EXPERT_MODE_PARAM pEncExpertModeParam = NULL;
	pMEDIAIP_ENC_PARAM enc_param;
	struct vpu_attr *attr;

	if (!ctx || !ctx->core_dev)
		return -EINVAL;

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return -EINVAL;

	if (vpu_enc_alloc_stream(ctx))
		return -ENOMEM;

	update_encode_size(ctx);

	enc_param = get_rpc_enc_param(ctx);
	pEncStrBuffDesc = get_rpc_stream_buffer_desc(ctx);

	pEncStrBuffDesc->start = ctx->encoder_stream.phy_addr;
	pEncStrBuffDesc->wptr = pEncStrBuffDesc->start;
	pEncStrBuffDesc->rptr = pEncStrBuffDesc->start;
	pEncStrBuffDesc->end = ctx->encoder_stream.phy_addr +
				ctx->encoder_stream.size;

	vpu_dbg(LVL_DEBUG,
		"pEncStrBuffDesc:start=%x, wptr=0x%x, rptr=%x, end=%x\n",
		pEncStrBuffDesc->start,
		pEncStrBuffDesc->wptr,
		pEncStrBuffDesc->rptr,
		pEncStrBuffDesc->end);

	pEncExpertModeParam = get_rpc_expert_mode_param(ctx);
	pEncExpertModeParam->Calib.mem_chunk_phys_addr = 0;
	pEncExpertModeParam->Calib.mem_chunk_virt_addr = 0;
	pEncExpertModeParam->Calib.mem_chunk_size = 0;
	pEncExpertModeParam->Calib.cb_base = ctx->encoder_stream.phy_addr;
	pEncExpertModeParam->Calib.cb_size = ctx->encoder_stream.size;

	reinit_completion(&ctx->start_cmp);
	show_firmware_version(ctx->core_dev, LVL_INFO);
	clear_stop_status(ctx);
	memcpy(enc_param, &attr->param, sizeof(attr->param));
	if (!enc_param->uGopBLength)
		enc_param->uLowLatencyMode = 1;
	vpu_ctx_send_cmd(ctx, GTB_ENC_CMD_CONFIGURE_CODEC, 0, NULL);

	show_codec_configure(enc_param, pEncExpertModeParam);
	vpu_dbg(LVL_FLOW, "[%d:%d] configure codec\n",
			ctx->core_dev->id, ctx->str_index);

	return 0;
}

static int check_vpu_ctx_is_ready(struct vpu_ctx *ctx)
{
	if (!ctx)
		return false;

	if (!test_bit(VPU_ENC_STATUS_OUTPUT_READY, &ctx->status))
		return false;
	if (!test_bit(VPU_ENC_STATUS_DATA_READY, &ctx->status))
		return false;

	return true;
}

static bool vpu_enc_check_ctx_is_alive(struct vpu_ctx *ctx)
{
	if (!ctx)
		return false;
	if (test_bit(VPU_ENC_STATUS_CLOSED, &ctx->status))
		return false;
	if (ctx->ctx_released)
		return false;
	if (!ctx->core_dev)
		return false;
	if (ctx->str_index >= ctx->core_dev->supported_instance_count)
		return false;
	if (ctx != ctx->core_dev->ctx[ctx->str_index])
		return false;
	return true;
}

static int configure_codec(struct vpu_ctx *ctx)
{
	if (!vpu_enc_check_ctx_is_alive(ctx))
		return -EINVAL;

	if (!check_vpu_ctx_is_ready(ctx))
		return 0;

	if (ctx->core_dev->snapshot)
		return 0;

	if (test_bit(VPU_ENC_STATUS_SNAPSHOT, &ctx->status))
		return 0;

	if (!test_and_set_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status)) {
		do_configure_codec(ctx);
		clear_bit(VPU_ENC_STATUS_OUTPUT_READY, &ctx->status);
		clear_bit(VPU_ENC_STATUS_DATA_READY, &ctx->status);
	}

	return 0;
}

static void dump_vb2_data(struct vb2_buffer *vb)
{
#ifdef DUMP_DATA
	const int DATA_NUM = 10;
	char *read_data;
	u_int32 read_idx;
	char data_str[1024];
	int num = 0;

	if (!vb)
		return;

	read_data = vb2_plane_vaddr(vb, 0);
	num = scnprintf(data_str, sizeof(data_str),
			"transfer data from virt 0x%p: ", read_data);
	for (read_idx = 0; read_idx < DATA_NUM; read_idx++)
		num += scnprintf(data_str + num, sizeof(data_str) - num,
				" 0x%x", read_data[read_idx]);

	vpu_dbg(LVL_DEBUG, "%s\n", data_str);
#endif
}

static u32 get_vb2_plane_phy_addr(struct vb2_buffer *vb, unsigned int plane_no)
{
	dma_addr_t *dma_addr;

	dma_addr = vb2_plane_cookie(vb, plane_no);
	return *dma_addr + vb->planes[plane_no].data_offset;
}

static void record_start_time(struct vpu_ctx *ctx, enum QUEUE_TYPE type)
{
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	struct timespec64 ts;

	if (!attr)
		return;

	if (attr->ts_start[type])
		return;

	ktime_get_raw_ts64(&ts);
	attr->ts_start[type] = ts.tv_sec * MSEC_PER_SEC +
				ts.tv_nsec / NSEC_PER_MSEC;
}

static bool update_yuv_addr(struct vpu_ctx *ctx)
{
	bool bGotAFrame = FALSE;

	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_SRC];

	pMEDIAIP_ENC_YUV_BUFFER_DESC desc;

	desc = get_rpc_yuv_buffer_desc(ctx);

	if (list_empty(&This->drv_q))
		return bGotAFrame;

	p_data_req = list_first_entry(&This->drv_q, typeof(*p_data_req), list);

	dump_vb2_data(p_data_req->vb2_buf);

	desc->uLumaBase = get_vb2_plane_phy_addr(p_data_req->vb2_buf, 0);
	desc->uChromaBase = get_vb2_plane_phy_addr(p_data_req->vb2_buf, 1);

	if (desc->uLumaBase != 0)
		bGotAFrame = TRUE;

	/*
	 * keeps increasing,
	 * so just a frame input count rather than a Frame buffer ID
	 */
	desc->uFrameID = p_data_req->sequence;
	if (test_and_clear_bit(VPU_ENC_STATUS_KEY_FRAME, &ctx->status))
		desc->uKeyFrame = 1;
	else
		desc->uKeyFrame = 0;
	list_del(&p_data_req->list);

	return bGotAFrame;
}

static void get_kmp_next(const u8 *p, int *next, int size)
{
	int k = -1;
	int j = 0;

	next[0] = -1;
	while (j < size - 1) {
		if (k == -1 || p[j] == p[k]) {
			++k;
			++j;
			next[j] = k;
		} else {
			k = next[k];
		}
	}
}

static int kmp_search(u8 *s, int s_len, const u8 *p, int p_len, int *next)
{
	int i = 0;
	int j = 0;

	while (i < s_len && j < p_len) {
		if (j == -1 || s[i] == p[j]) {
			i++;
			j++;
		} else {
			j = next[j];
		}
	}
	if (j == p_len)
		return i - j;
	else
		return -1;
}

static int get_stuff_data_size(u8 *data, int size)
{
	const u8 pattern[] = VPU_STRM_END_PATTERN;
	int next[] = VPU_STRM_END_PATTERN;
	int index;

	if (size < ARRAY_SIZE(pattern))
		return 0;

	get_kmp_next(pattern, next, ARRAY_SIZE(pattern));
	index =  kmp_search(data, size, pattern, ARRAY_SIZE(pattern), next);
	if (index < 0)
		return 0;
	vpu_dbg(LVL_DEBUG, "find end_of_stream nal\n");
	return size - index;
}

static void count_strip_info(struct vpu_strip_info *info, u32 bytes)
{
	if (!info)
		return;

	info->count++;
	info->total += bytes;
	if (info->max < bytes)
		info->max = bytes;
}

static void strip_stuff_data_on_tail(struct vpu_ctx *ctx, struct vb2_buffer *vb)
{
	u8 *ptr = vb2_plane_vaddr(vb, 0);
	unsigned long bytesused = vb2_get_plane_payload(vb, 0);
	int count = VPU_TAIL_SERACH_SIZE;
	int stuff_size;

	if (count > bytesused)
		count = bytesused;

	if (!count)
		return;

	stuff_size = get_stuff_data_size(ptr + bytesused - count, count);
	if (stuff_size) {
		struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

		if (attr)
			count_strip_info(&attr->statistic.strip_sts.eos,
					stuff_size);

		vpu_dbg(LVL_DEBUG, "strip %d bytes stuff data\n", stuff_size);
		vb2_set_plane_payload(vb, 0, bytesused - stuff_size);
	}
}

static int check_enc_rw_flag(int flag)
{
	int ret = -EINVAL;

	switch (flag) {
	case VPU_ENC_FLAG_WRITEABLE:
	case VPU_ENC_FLAG_READABLE:
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static void set_queue_rw_flag(struct queue_data *queue, int flag)
{
	if (!queue)
		return;

	if (check_enc_rw_flag(flag))
		return;

	set_bit(flag, &queue->rw_flag);
}

static void clear_queue_rw_flag(struct queue_data *queue, int flag)
{
	if (!queue)
		return;

	if (check_enc_rw_flag(flag))
		return;

	clear_bit(flag, &queue->rw_flag);
}

u32 get_free_space(struct vpu_ctx *ctx)
{
	pBUFFER_DESCRIPTOR_TYPE desc = get_rpc_stream_buffer_desc(ctx);
	u32 start = get_ptr(desc->start);
	u32 end = get_ptr(desc->end);
	u32 wptr = get_ptr(desc->wptr);
	u32 rptr = get_ptr(desc->rptr);

	if (rptr > wptr)
		return (rptr - wptr);
	else if (rptr < wptr)
		return (end - start + rptr - wptr);
	else
		return (end - start);

}

bool check_stream_buffer_for_coded_picture(struct vpu_ctx *ctx)
{
	if (get_free_space(ctx) < ctx->cpb_size)
		return false;

	return true;
}

static int submit_input_and_encode(struct vpu_ctx *ctx)
{
	struct queue_data *queue;

	if (!ctx)
		return -EINVAL;

	queue = &ctx->q_data[V4L2_SRC];

	down(&queue->drv_q_lock);

	if (!test_bit(VPU_ENC_FLAG_WRITEABLE, &queue->rw_flag))
		goto exit;

	if (list_empty(&queue->drv_q))
		goto exit;

	if (!check_stream_buffer_for_coded_picture(ctx))
		goto exit;

	if (test_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status))
		goto exit;
	if (!test_bit(VPU_ENC_STATUS_START_DONE, &ctx->status))
		goto exit;

	if (update_yuv_addr(ctx)) {
		vpu_ctx_send_cmd(ctx, GTB_ENC_CMD_FRAME_ENCODE, 0, NULL);
		clear_queue_rw_flag(queue, VPU_ENC_FLAG_WRITEABLE);
		record_start_time(ctx, V4L2_SRC);
	}
exit:
	up(&queue->drv_q_lock);

	return 0;
}

static void add_rptr(struct vpu_frame_info *frame, u32 length)
{
	WARN_ON(!frame);

	frame->rptr += length;
	if (frame->rptr >= frame->end)
		frame->rptr -= (frame->end - frame->start);
}

static void report_frame_type(struct vb2_data_req *p_data_req,
				struct vpu_frame_info *frame)
{
	WARN_ON(!p_data_req || !frame);

	switch (frame->info.ePicType) {
	case MEDIAIP_ENC_PIC_TYPE_IDR_FRAME:
	case MEDIAIP_ENC_PIC_TYPE_I_FRAME:
		p_data_req->buffer_flags = V4L2_BUF_FLAG_KEYFRAME;
		break;
	case MEDIAIP_ENC_PIC_TYPE_P_FRAME:
		p_data_req->buffer_flags = V4L2_BUF_FLAG_PFRAME;
		break;
	case MEDIAIP_ENC_PIC_TYPE_B_FRAME:
		p_data_req->buffer_flags = V4L2_BUF_FLAG_BFRAME;
		break;
	default:
		break;
	}
}

static u32 calc_frame_length(struct vpu_frame_info *frame)
{
	u32 length;
	u32 buffer_size;

	WARN_ON(!frame);

	if (frame->eos)
		return 0;

	buffer_size = frame->end - frame->start;
	if (!buffer_size)
		return 0;

	length = (buffer_size + frame->wptr - frame->rptr) % buffer_size;

	return length;
}

static u32 get_ptr(u32 ptr)
{
	return (ptr | 0x80000000);
}

static void *get_rptr_virt(struct vpu_ctx *ctx, struct vpu_frame_info *frame)
{
	WARN_ON(!ctx || !frame);

	return ctx->encoder_stream.virt_addr + frame->rptr - frame->start;
}

static int find_nal_begin(u8 *data, u32 size)
{
	const u8 pattern[] = VPU_STRM_BEGIN_PATTERN;
	int next[] = VPU_STRM_BEGIN_PATTERN;
	u32 len;
	int index;

	len = ARRAY_SIZE(pattern);
	get_kmp_next(pattern, next, len);
	index = kmp_search(data, size, pattern, len, next);
	if (index > 0 && data[index - 1] == 0)
		index--;

	return index;
}

static int find_frame_start_and_skip(struct vpu_ctx *ctx,
			struct vpu_frame_info *frame, int skip)
{
	u32 length;
	u32 bytesskiped = 0;
	u8 *data = get_rptr_virt(ctx, frame);
	int index;

	length = frame->bytesleft;
	if (frame->rptr + length <= frame->end) {
		index = find_nal_begin(data, length);
		if (index >= 0)
			bytesskiped += index;
		else
			bytesskiped += length;
	} else {
		u32 size = frame->end - frame->rptr;

		index = find_nal_begin(data, size);
		if (index >= 0) {
			bytesskiped += index;
		} else {
			bytesskiped += size;

			data = ctx->encoder_stream.virt_addr;
			size = length - size;
			index = find_nal_begin(data, size);
			if (index >= 0)
				bytesskiped += index;
			else
				bytesskiped += size;
		}
	}

	if (skip && bytesskiped) {
		add_rptr(frame, bytesskiped);
		frame->bytesleft -= bytesskiped;
	}

	return bytesskiped;
}

static int transfer_stream_output(struct vpu_ctx *ctx,
					struct vpu_frame_info *frame,
					struct vb2_data_req *p_data_req)
{
	struct vb2_buffer *vb = NULL;
	u32 length;
	void *pdst;

	WARN_ON(!ctx || !frame || !p_data_req);

	length = frame->bytesleft;

	vb = p_data_req->vb2_buf;
	if (length > vb->planes[0].length)
		length = vb->planes[0].length;
	vb2_set_plane_payload(vb, 0, length);

	pdst = vb2_plane_vaddr(vb, 0);
	if (frame->rptr + length <= frame->end) {
		memcpy(pdst, get_rptr_virt(ctx, frame), length);
		frame->bytesleft -= length;
		add_rptr(frame, length);
	} else {
		u32 offset = frame->end - frame->rptr;

		memcpy(pdst, get_rptr_virt(ctx, frame), offset);
		frame->bytesleft -= offset;
		add_rptr(frame, offset);
		length -= offset;
		memcpy(pdst + offset, get_rptr_virt(ctx, frame), length);
		frame->bytesleft -= length;
		add_rptr(frame, length);
	}
	report_frame_type(p_data_req, frame);
	if (frame->bytesleft)
		return 0;

	strip_stuff_data_on_tail(ctx, p_data_req->vb2_buf);

	return 0;
}

static int append_empty_end_frame(struct vb2_data_req *p_data_req)
{
	struct vb2_buffer *vb = NULL;
	const u8 pattern[] = VPU_STRM_END_PATTERN;
	void *pdst;

	WARN_ON(!p_data_req);

	vb = p_data_req->vb2_buf;
	pdst = vb2_plane_vaddr(vb, 0);
	memcpy(pdst, pattern, ARRAY_SIZE(pattern));

	vb2_set_plane_payload(vb, 0, ARRAY_SIZE(pattern));
	p_data_req->buffer_flags = V4L2_BUF_FLAG_LAST;

	vpu_dbg(LVL_INFO, "append the last frame\n");

	return 0;
}

static s64 calculate_timestamp_for_eos(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = NULL;
	struct v4l2_fract *fival;
	s64 timestamp;
	u64 delta = 0;

	timestamp = ctx->timestamp;
	attr = get_vpu_ctx_attr(ctx);
	fival = &attr->fival;
	if (ctx->timestamp != VPU_ENC_INVALID_TIMESTAMP && fival->denominator) {
		delta = NSEC_PER_SEC * fival->numerator / fival->denominator;
		timestamp += delta;
	}

	vpu_dbg(LVL_INFO, "[%d]eos ts : %lld, delta = %lld, %lld, %d / %d\n",
			ctx->str_index,
			timestamp, delta, ctx->timestamp,
			fival->numerator, fival->denominator);
	return timestamp;
}

static bool is_valid_frame_read_pos(u32 ptr, struct vpu_frame_info *frame)
{
	if (ptr < frame->start || ptr >= frame->end)
		return false;
	if (ptr >= frame->rptr && ptr < frame->wptr)
		return true;
	if (frame->rptr > frame->wptr) {
		if (ptr >= frame->rptr || ptr < frame->wptr)
			return true;
	}

	return false;
}
static int precheck_frame(struct vpu_ctx *ctx, struct vpu_frame_info *frame)
{
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	u32 length;
	int bytesskiped;
	u32 rptr;

	if (frame->eos)
		return 0;
	if (!frame->is_start)
		return 0;

	add_rptr(frame, 0);
	frame->is_start = false;
	rptr = get_ptr(frame->info.uStrBuffWrPtr);
	if (rptr == frame->end)
		rptr = frame->start;

	if (is_valid_frame_read_pos(rptr, frame)) {
		if (rptr != frame->rptr) {
			vpu_dbg(LVL_DEBUG, "frame skip %d bytes\n",
					rptr - frame->rptr);
			count_strip_info(&attr->statistic.strip_sts.fw,
					rptr - frame->rptr);
		}
		frame->rptr = rptr;
	} else {
		vpu_err("[%ld]wrong uStrBuffWrPtr:0x%x\n", frame->index, rptr);
	}

	length = calc_frame_length(frame);
	if (!length || length < frame->bytesleft) {
		vpu_err("[%d][%d]'s frame[%ld] invalid, want %d but %d, drop\n",
				ctx->core_dev->id, ctx->str_index,
				frame->index, frame->bytesleft, length);
		vpu_err("uStrBuffWrPtr = 0x%x, uFrameSize = 0x%x\n",
			frame->info.uStrBuffWrPtr, frame->info.uFrameSize);
		add_rptr(frame, length);
		return -EINVAL;
	}

	bytesskiped = find_frame_start_and_skip(ctx, frame, 1);
	if (!bytesskiped)
		return 0;

	if (attr)
		count_strip_info(&attr->statistic.strip_sts.begin, bytesskiped);

	return 0;
}

static int inc_frame(struct queue_data *queue)
{
	struct vpu_frame_info *frame = NULL;

	if (!queue)
		return -EINVAL;

	frame = vzalloc(sizeof(*frame));
	if (!frame)
		return -EINVAL;

	frame->queue = queue;
	list_add_tail(&frame->list, &queue->frame_idle);
	atomic64_inc(&queue->frame_count);

	vpu_dbg(LVL_DEBUG, "++ frame : %lld\n",
			atomic64_read(&queue->frame_count));

	return 0;
}

static void dec_frame(struct vpu_frame_info *frame)
{
	if (!frame)
		return;
	list_del_init(&frame->list);
	if (frame->queue) {
		atomic64_dec(&frame->queue->frame_count);

		vpu_dbg(LVL_DEBUG, "-- frame : %lld\n",
				atomic64_read(&frame->queue->frame_count));
	}
	VPU_SAFE_RELEASE(frame, vfree);
}

static struct vpu_frame_info *get_idle_frame(struct queue_data *queue)
{
	struct vpu_frame_info *frame = NULL;

	if (!queue)
		return NULL;

	if (list_empty(&queue->frame_idle))
		inc_frame(queue);
	frame = list_first_entry(&queue->frame_idle,
				struct vpu_frame_info, list);
	if (frame)
		list_del_init(&frame->list);

	return frame;
}

static void put_frame_idle(struct vpu_frame_info *frame)
{
	struct queue_data *queue;

	if (!frame)
		return;

	list_del_init(&frame->list);
	memset(&frame->info, 0, sizeof(frame->info));
	frame->bytesleft = 0;
	frame->wptr = 0;
	frame->rptr = 0;
	frame->start = 0;
	frame->end = 0;
	frame->eos = false;
	frame->is_start = false;
	frame->index = 0;
	frame->timestamp = VPU_ENC_INVALID_TIMESTAMP;
	queue = frame->queue;
	if (queue && atomic64_read(&queue->frame_count) <= FRAME_COUNT_THD)
		list_add_tail(&frame->list, &queue->frame_idle);
	else
		dec_frame(frame);
}

static bool process_frame_done(struct queue_data *queue)
{
	struct vpu_ctx *ctx;
	struct vb2_data_req *p_data_req = NULL;
	struct vpu_frame_info *frame = NULL;
	pBUFFER_DESCRIPTOR_TYPE stream_buffer_desc;

	WARN_ON(!queue || !queue->ctx);

	ctx = queue->ctx;

	stream_buffer_desc = get_rpc_stream_buffer_desc(ctx);

	if (list_empty(&queue->frame_q))
		return false;

	frame = list_first_entry(&queue->frame_q, typeof(*frame), list);
	if (!frame)
		return false;

	frame->rptr = get_ptr(stream_buffer_desc->rptr);

	if (precheck_frame(ctx, frame)) {
		update_stream_desc_rptr(ctx, frame->rptr);
		put_frame_idle(frame);
		frame = NULL;
		return true;
	}

	if (list_empty(&queue->drv_q))
		return false;

	p_data_req = list_first_entry(&queue->drv_q, typeof(*p_data_req), list);

	if (frame->eos)
		append_empty_end_frame(p_data_req);
	else
		transfer_stream_output(ctx, frame, p_data_req);

	update_stream_desc_rptr(ctx, frame->rptr);
	fill_vb_sequence(p_data_req->vb2_buf, frame->info.uFrameID);
	set_vb_flags(p_data_req->vb2_buf, p_data_req->buffer_flags);
	p_data_req->vb2_buf->timestamp = frame->timestamp;
	vpu_dbg(LVL_FRAME, "[%d:%d] index : %8d, length : %8ld, ts : %lld%s\n",
			ctx->core_dev->id, ctx->str_index,
			frame->info.uFrameID,
			vb2_get_plane_payload(p_data_req->vb2_buf, 0),
			p_data_req->vb2_buf->timestamp,
			frame->eos ? " (EOS)" : "");
	if (!frame->bytesleft) {
		put_frame_idle(frame);
		frame = NULL;
	}
	list_del(&p_data_req->list);

	if (p_data_req->vb2_buf->state == VB2_BUF_STATE_ACTIVE)
		vb2_buffer_done(p_data_req->vb2_buf, VB2_BUF_STATE_DONE);
	else
		vpu_err("vb2_buf's state is invalid(%d\n)",
				p_data_req->vb2_buf->state);

	return true;
}

static int process_stream_output(struct vpu_ctx *ctx)
{
	struct queue_data *queue = NULL;

	if (!ctx)
		return -EINVAL;

	queue = &ctx->q_data[V4L2_DST];

	down(&queue->drv_q_lock);
	while (1) {
		if (!process_frame_done(queue))
			break;
	}
	up(&queue->drv_q_lock);

	submit_input_and_encode(ctx);

	return 0;
}

static void show_enc_pic_info(MEDIAIP_ENC_PIC_INFO *pEncPicInfo)
{
#ifdef TB_REC_DBG
	vpu_dbg(LVL_DEBUG, "       - Frame ID      : 0x%x\n",
			pEncPicInfo->uFrameID);

	switch (pEncPicInfo->ePicType) {
	case MEDIAIP_ENC_PIC_TYPE_IDR_FRAME:
		vpu_dbg(LVL_DEBUG, "       - Picture Type  : IDR picture\n");
		break;
	case MEDIAIP_ENC_PIC_TYPE_I_FRAME:
		vpu_dbg(LVL_DEBUG, "       - Picture Type  : I picture\n");
		break;
	case MEDIAIP_ENC_PIC_TYPE_P_FRAME:
		vpu_dbg(LVL_DEBUG, "       - Picture Type  : P picture\n");
		break;
	case MEDIAIP_ENC_PIC_TYPE_B_FRAME:
		vpu_dbg(LVL_DEBUG, "       - Picture Type  : B picture\n");
		break;
	default:
		vpu_dbg(LVL_DEBUG, "       - Picture Type  : BI picture\n");
		break;
	}
	vpu_dbg(LVL_DEBUG, "       - Skipped frame : 0x%x\n",
			pEncPicInfo->uSkippedFrame);
	vpu_dbg(LVL_DEBUG, "       - Frame size    : 0x%x\n",
			pEncPicInfo->uFrameSize);
	vpu_dbg(LVL_DEBUG, "       - Frame CRC     : 0x%x\n",
			pEncPicInfo->uFrameCrc);
#endif
}

static int handle_event_start_done(struct vpu_ctx *ctx)
{
	if (!ctx)
		return -EINVAL;

	down(&ctx->q_data[V4L2_SRC].drv_q_lock);
	set_bit(VPU_ENC_STATUS_START_DONE, &ctx->status);
	set_queue_rw_flag(&ctx->q_data[V4L2_SRC], VPU_ENC_FLAG_WRITEABLE);
	up(&ctx->q_data[V4L2_SRC].drv_q_lock);
	submit_input_and_encode(ctx);

	enable_fps_sts(get_vpu_ctx_attr(ctx));
	complete(&ctx->start_cmp);
	vpu_dbg(LVL_FLOW, "[%d:%d] start done\n",
			ctx->core_dev->id, ctx->str_index);

	return 0;
}

static void vpu_enc_config_expert_mode_parm(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = NULL;
	pMEDIAIP_ENC_EXPERT_MODE_PARAM param = NULL;

	if (!ctx)
		return;

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return;

	param = get_rpc_expert_mode_param(ctx);
	if (!param)
		return;

	param->Config.frame_rate_num = attr->fival.numerator;
	param->Config.frame_rate_den = attr->fival.denominator;
	vpu_dbg(LVL_FLOW, "[%d:%d] h264 frame rate: %d/%d\n",
		ctx->core_dev->id, ctx->str_index,
		param->Config.frame_rate_num, param->Config.frame_rate_den);

	param->Config.h264_aspect_ratio_present = attr->h264_vui_sar_enable;
	param->Config.aspect_ratio = attr->h264_vui_sar_idc;
	param->Config.h264_aspect_ratio_sar_width = attr->h264_vui_sar_width;
	param->Config.h264_aspect_ratio_sar_height = attr->h264_vui_sar_height;
	vpu_dbg(LVL_FLOW,
		"[%d:%d] h264 vui sar:enable=%d, idc=%d, width=%d, height=%d\n",
		ctx->core_dev->id, ctx->str_index,
		attr->h264_vui_sar_enable,
		attr->h264_vui_sar_idc,
		attr->h264_vui_sar_width,
		attr->h264_vui_sar_height);

	param->Config.h264_video_type_present = 1;
	param->Config.h264_video_format = 5;
	param->Config.h264_video_colour_descriptor = 1;
	vpu_enc_convert_color_v4l2_aspect_to_iso_aspect(ctx,
			&param->Config.h264_video_colour_primaries,
			&param->Config.h264_video_transfer_char,
			&param->Config.h264_video_matrix_coeff,
			&param->Config.h264_video_full_range);

	vpu_dbg(LVL_FLOW,
		"[%d:%d] primaries=%d, transfer=%d, matrix=%d, fullrange =%d\n",
		ctx->core_dev->id, ctx->str_index,
		param->Config.h264_video_colour_primaries,
		param->Config.h264_video_transfer_char,
		param->Config.h264_video_matrix_coeff,
		param->Config.h264_video_full_range);
}

static int handle_event_mem_request(struct vpu_ctx *ctx,
				MEDIAIP_ENC_MEM_REQ_DATA *req_data)
{
	int ret;

	if (!ctx || !req_data)
		return -EINVAL;

	ret = vpu_enc_alloc_mem(ctx, req_data, get_rpc_mem_pool(ctx));
	if (ret) {
		vpu_err("fail to alloc encoder memory\n");
		return ret;
	}

	vpu_enc_config_expert_mode_parm(ctx);

	vpu_ctx_send_cmd(ctx, GTB_ENC_CMD_STREAM_START, 0, NULL);
	set_bit(VPU_ENC_STATUS_START_SEND, &ctx->status);
	vpu_dbg(LVL_FLOW, "[%d:%d] start stream\n",
			ctx->core_dev->id, ctx->str_index);

	return 0;
}

static bool check_stream_buffer_desc(struct vpu_ctx *ctx,
				pBUFFER_DESCRIPTOR_TYPE stream_buffer_desc)
{
	u32 start;
	u32 end;
	u32 rptr;
	u32 wptr;

	if (!stream_buffer_desc)
		return false;
	start = get_ptr(stream_buffer_desc->start);
	end = get_ptr(stream_buffer_desc->end);
	rptr = get_ptr(stream_buffer_desc->rptr);
	wptr = get_ptr(stream_buffer_desc->wptr);

	if (rptr < start || rptr > end ||
		wptr < start || wptr > end ||
		end <= start ||
		end - start != ctx->encoder_stream.size) {
		vpu_err("stream buffer desc is invalid, s:%x,e:%x,r:%x,w:%x\n",
				start, end, rptr, wptr);
		return false;
	}

	return true;
}

static int handle_event_frame_done(struct vpu_ctx *ctx,
				MEDIAIP_ENC_PIC_INFO *pEncPicInfo)
{
	struct queue_data *queue;
	struct vpu_frame_info *frame;
	pBUFFER_DESCRIPTOR_TYPE stream_buffer_desc;
	s64 timestamp;

	if (!ctx || !pEncPicInfo)
		return -EINVAL;

	vpu_dbg(LVL_DEBUG, "Frame done(%d) - uFrameID = %d\n",
			pEncPicInfo->uPicEncodDone, pEncPicInfo->uFrameID);

	queue = &ctx->q_data[V4L2_DST];
	if (!pEncPicInfo->uPicEncodDone) {
		vpu_err("Pic Encoder Not Done\n");
		return -EINVAL;
	}

	stream_buffer_desc = get_rpc_stream_buffer_desc(ctx);
	if (!check_stream_buffer_desc(ctx, stream_buffer_desc))
		return -EINVAL;

	show_enc_pic_info(pEncPicInfo);
	record_start_time(ctx, V4L2_DST);

	timestamp = get_ctx_seq_timestamp(ctx, pEncPicInfo->uFrameID);

	down(&queue->drv_q_lock);
	frame = get_idle_frame(queue);
	if (frame) {
		struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

		memcpy(&frame->info, pEncPicInfo, sizeof(frame->info));
		frame->bytesleft = frame->info.uFrameSize;
		frame->wptr = get_ptr(stream_buffer_desc->wptr);
		frame->rptr = get_ptr(stream_buffer_desc->rptr);
		frame->start = get_ptr(stream_buffer_desc->start);
		frame->end = get_ptr(stream_buffer_desc->end);
		frame->eos = false;
		frame->is_start = true;
		frame->timestamp = timestamp;
		if (attr)
			frame->index = attr->statistic.encoded_count;

		list_add_tail(&frame->list, &queue->frame_q);
	} else {
		vpu_err("fail to alloc memory for frame info\n");
	}
	up(&queue->drv_q_lock);
	count_encoded_frame(ctx);

	/* Sync the write pointer to the local view of it */
	process_stream_output(ctx);

	return 0;
}

static int handle_event_frame_release(struct vpu_ctx *ctx, u_int32 *uFrameID)
{
	struct queue_data *This = &ctx->q_data[V4L2_SRC];
	struct vb2_data_req *p_data_req = NULL;

	if (!ctx || !uFrameID)
		return -EINVAL;

	This = &ctx->q_data[V4L2_SRC];
	vpu_dbg(LVL_DEBUG, "Frame release - uFrameID = %d\n", *uFrameID);
	p_data_req = find_vb2_data_by_sequence(This, *uFrameID);
	if (!p_data_req) {
		vpu_err("uFrameID[%d] is invalid\n", *uFrameID);
		return -EINVAL;
	}
	if (p_data_req->vb2_buf->state == VB2_BUF_STATE_ACTIVE)
		vb2_buffer_done(p_data_req->vb2_buf, VB2_BUF_STATE_DONE);

	return 0;
}

static int handle_event_stop_done(struct vpu_ctx *ctx)
{
	struct queue_data *queue;
	struct vpu_frame_info *frame;

	WARN_ON(!ctx);
	queue = &ctx->q_data[V4L2_DST];

	disable_fps_sts(get_vpu_ctx_attr(ctx));

	set_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status);
	vpu_dbg(LVL_FLOW, "[%d:%d] stop done\n",
			ctx->core_dev->id, ctx->str_index);

	down(&queue->drv_q_lock);
	frame = get_idle_frame(queue);
	if (frame) {
		frame->eos = true;
		frame->timestamp = calculate_timestamp_for_eos(ctx);
		frame->info.uFrameID = ctx->sequence;
		list_add_tail(&frame->list, &queue->frame_q);
	} else {
		vpu_err("fail to alloc memory for last frame\n");
	}
	up(&queue->drv_q_lock);

	process_stream_output(ctx);

	notify_eos(ctx);
	clear_start_status(ctx);
	init_ctx_seq_info(ctx);
	complete(&ctx->stop_cmp);

	return 0;
}

static void vpu_stop_ctx_asynchronous(struct vpu_ctx *ctx)
{
	if (!ctx || ctx->ctx_released)
		return;
	if (!test_bit(VPU_ENC_STATUS_START_DONE, &ctx->status))
		return;
	if (!test_bit(VPU_ENC_STATUS_CLOSED, &ctx->status))
		return;
	if (test_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status))
		return;
	if (test_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status))
		return;
	request_eos(ctx);
}

static void vpu_enc_event_handler(struct vpu_ctx *ctx,
				u_int32 uEvent, u_int32 *event_data)
{
	vpu_log_event(uEvent, ctx->str_index);
	vpu_enc_check_mem_overstep(ctx);

	switch (uEvent) {
	case VID_API_ENC_EVENT_START_DONE:
		handle_event_start_done(ctx);
		break;
	case VID_API_ENC_EVENT_MEM_REQUEST:
		handle_event_mem_request(ctx,
				(MEDIAIP_ENC_MEM_REQ_DATA *)event_data);
		break;
	case VID_API_ENC_EVENT_PARA_UPD_DONE:
		break;
	case VID_API_ENC_EVENT_FRAME_DONE:
		handle_event_frame_done(ctx,
					(MEDIAIP_ENC_PIC_INFO *)event_data);
		break;
	case VID_API_ENC_EVENT_FRAME_RELEASE:
		handle_event_frame_release(ctx, (u_int32 *)event_data);
		break;
	case VID_API_ENC_EVENT_STOP_DONE:
		handle_event_stop_done(ctx);
		break;
	case VID_API_ENC_EVENT_FRAME_INPUT_DONE:
		down(&ctx->q_data[V4L2_SRC].drv_q_lock);
		set_queue_rw_flag(&ctx->q_data[V4L2_SRC],
				VPU_ENC_FLAG_WRITEABLE);
		up(&ctx->q_data[V4L2_SRC].drv_q_lock);
		response_stop_stream(ctx);
		submit_input_and_encode(ctx);
		break;
	case VID_API_ENC_EVENT_TERMINATE_DONE:
		break;
	case VID_API_ENC_EVENT_RESET_DONE:
		break;
	case VID_API_ENC_EVENT_FIRMWARE_XCPT:
		vpu_err("firmware exception:%s\n", (char *)event_data);
		break;
	default:
		vpu_err("........unknown event : 0x%x\n", uEvent);
		break;
	}
	if (test_bit(VPU_ENC_STATUS_CLOSED, &ctx->status))
		vpu_stop_ctx_asynchronous(ctx);
}

static void get_core_supported_instance_count(struct core_device *core)
{
	pENC_RPC_HOST_IFACE iface;

	iface = core->shared_mem.pSharedInterface;
	core->supported_instance_count =
		min_t(u32, iface->uMaxEncoderStreams, VID_API_NUM_STREAMS);
}

static int re_configure_codecs(struct core_device *core)
{
	int i;

	for (i = 0; i < core->supported_instance_count; i++) {
		struct vpu_ctx *ctx = core->ctx[i];
		struct queue_data *queue;

		if (!ctx || !test_bit(VPU_ENC_STATUS_INITIALIZED, &ctx->status))
			continue;

		mutex_lock(&ctx->instance_mutex);
		queue = &ctx->q_data[V4L2_SRC];
		if (!list_empty(&queue->drv_q)) {
			vpu_dbg(LVL_INFO,
				"re configure codec for core[%d]\n", core->id);
			configure_codec(ctx);
			submit_input_and_encode(ctx);
		}
		mutex_unlock(&ctx->instance_mutex);
	}

	return 0;
}

static int wait_for_boot_done(struct core_device *core, int resume)
{
	int ret;

	if (!core)
		return -EINVAL;

	ret = wait_for_completion_timeout(&core->boot_cmp,
						msecs_to_jiffies(1000));
	if (!ret) {
		vpu_err("error: wait for core[%d] %s done timeout!\n",
				core->id, resume ? "resume" : "start");
		return -EINVAL;
	}

	if (resume)
		re_configure_codecs(core);

	return 0;
}

static void vpu_core_start_done(struct core_device *core)
{
	if (!core)
		return;

	get_core_supported_instance_count(core);
	core->firmware_started = true;
	complete(&core->boot_cmp);

	show_firmware_version(core, LVL_ALL);
}

static struct vpu_ctx *get_ctx_by_index(struct core_device *core, int index)
{
	struct vpu_ctx *ctx = NULL;

	if (!core)
		return NULL;

	if (index < 0 || index >= core->supported_instance_count)
		return NULL;

	ctx = core->ctx[index];
	if (!ctx)
		return NULL;

	if (!test_bit(VPU_ENC_STATUS_INITIALIZED, &ctx->status)) {
		vpu_err("core[%d]'s ctx[%d] is not initialized\n",
				core->id, index);
		return NULL;
	}

	if (test_bit(VPU_ENC_STATUS_CLOSED, &ctx->status))
		vpu_err("core[%d]'s ctx[%d] is closed\n", core->id, index);

	return ctx;
}

static int process_ctx_msg(struct vpu_ctx *ctx, struct msg_header *header)
{
	int ret = 0;
	struct vpu_event_msg *msg = NULL;
	u32 *pdata = NULL;

	if (!ctx || !header)
		return -EINVAL;

	msg = get_idle_msg(ctx);
	if (!msg) {
		vpu_err("get idle msg fail\n");
		ret = -ENOMEM;
		goto error;
	}

	msg->idx = header->idx;
	msg->msgid = header->msgid;
	msg->number = header->msgnum;
	pdata = msg->data;
	ret = 0;
	if (msg->number > ARRAY_SIZE(msg->data)) {
		ret = alloc_msg_ext_buffer(msg, header->msgnum);
		pdata = msg->ext_data;
	}
	rpc_read_msg_array(&ctx->core_dev->shared_mem, pdata, msg->number);
	if (ret) {
		put_idle_msg(ctx, msg);
		return ret;
	}

	push_back_event_msg(ctx, msg);

	return ret;
error:
	rpc_read_msg_array(&ctx->core_dev->shared_mem, NULL, header->msgnum);
	return ret;
}

static void vpu_enc_notify_msg_event(struct core_device *core)
{
	int i;

	for (i = 0; i < core->supported_instance_count; i++) {
		struct vpu_ctx *ctx = core->ctx[i];

		if (!ctx || is_event_msg_empty(ctx))
			continue;
		queue_work(ctx->instance_wq, &ctx->instance_work);
	}
}

static int process_msg(struct core_device *core)
{
	struct msg_header header;
	struct vpu_ctx *ctx = NULL;
	int ret;

	ret = rpc_get_msg_header(&core->shared_mem, &header);
	if (ret)
		return ret;

	if (header.idx >= ARRAY_SIZE(core->ctx)) {
		vpu_err("msg idx(%d) is out of range, msgid = 0x%x\n",
				header.idx, header.msgid);
		rpc_read_msg_array(&core->shared_mem, NULL, header.msgnum);
		return -EINVAL;
	}

	mutex_lock(&core->vdev->dev_mutex);
	ctx = get_ctx_by_index(core, header.idx);
	if (ctx != NULL && !ctx->ctx_released) {
		count_event(ctx, header.msgid);
		process_ctx_msg(ctx, &header);
		queue_work(ctx->instance_wq, &ctx->instance_work);
	} else {
		vpu_err("msg[%d] of ctx[%d] is missed\n",
				header.msgid, header.idx);
		rpc_read_msg_array(&core->shared_mem, NULL, header.msgnum);
	}
	mutex_unlock(&core->vdev->dev_mutex);

	return 0;
}

static void vpu_enc_fw_init(struct core_device *core_dev)
{
	u32 mu_addr;

	vpu_dbg(LVL_ALL, "enable mu for core[%d]\n", core_dev->id);

	mu_addr = cpu_phy_to_mu(core_dev, core_dev->m0_rpc_phy);
	vpu_enc_mu_send_msg(core_dev, RPC_BUF_OFFSET, mu_addr);
	vpu_enc_mu_send_msg(core_dev, BOOT_ADDRESS, core_dev->m0_p_fw_space_phy);
	vpu_enc_mu_send_msg(core_dev, INIT_DONE, 2);
}

extern u_int32 rpc_MediaIPFW_Video_message_check_encoder(struct shared_addr *This);

static void vpu_enc_receive_msg_event(struct core_device *core_dev)
{
	struct shared_addr *This = &core_dev->shared_mem;

	while (rpc_MediaIPFW_Video_message_check_encoder(This) == API_MSG_AVAILABLE)
		process_msg(core_dev);

	if (rpc_MediaIPFW_Video_message_check_encoder(This) == API_MSG_BUFFER_ERROR)
		vpu_err("MSG num is too big to handle\n");

	mutex_lock(&core_dev->vdev->dev_mutex);
	vpu_enc_notify_msg_event(core_dev);
	mutex_unlock(&core_dev->vdev->dev_mutex);
}

static void vpu_enc_handle_msg_data(struct core_device *core_dev, u_int32 data)
{
	if (data == 0xaa)
		vpu_enc_fw_init(core_dev);
	else if (data == 0x55)
		vpu_core_start_done(core_dev);
	else if (data == 0xA5)
		complete(&core_dev->snap_done_cmp);
	else
		vpu_enc_receive_msg_event(core_dev);
}

static void vpu_enc_msg_run_work(struct work_struct *work)
{
	struct core_device *dev = container_of(work, struct core_device, msg_work);
	u_int32 data;

	while (vpu_enc_mu_receive_msg(dev, &data) >= sizeof(u_int32))
		vpu_enc_handle_msg_data(dev, data);
}

static void vpu_enc_msg_instance_work(struct work_struct *work)
{
	struct vpu_ctx *ctx = container_of(work, struct vpu_ctx, instance_work);
	struct vpu_event_msg *msg;

	while (1) {
		msg = pop_event_msg(ctx);
		if (!msg)
			break;
		if (msg->ext_data)
			vpu_enc_event_handler(ctx, msg->msgid, msg->ext_data);
		else
			vpu_enc_event_handler(ctx, msg->msgid, msg->data);

		put_idle_msg(ctx, msg);
	}
}

static int vpu_queue_setup(struct vb2_queue *vq,
		unsigned int *buf_count,
		unsigned int *plane_count,
		unsigned int psize[],
		struct device *allocators[])
{
	struct queue_data  *This = (struct queue_data *)vq->drv_priv;

	vpu_dbg(LVL_BUF, "%s(), %s, (%d, %d)\n", __func__, This->desc,
			This->ctx->core_dev->id, This->ctx->str_index);

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
			*plane_count = 2;
			psize[0] = This->sizeimage[0];//check alignment
			psize[1] = This->sizeimage[1];//check colocated_size
		} else {
			psize[0] = This->sizeimage[0] + This->sizeimage[1];
			*plane_count = 1;
		}
		if (*buf_count < MIN_BUFFER_COUNT)
			*buf_count = MIN_BUFFER_COUNT;
	} else {
		*plane_count = 1;
		psize[0] = This->sizeimage[0];//check alignment
	}

	if (*buf_count > VPU_MAX_BUFFER)
		*buf_count = VPU_MAX_BUFFER;
	if (*buf_count < 1)
		*buf_count = 1;

	return 0;
}

static int vpu_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct queue_data *q_data = (struct queue_data *)vq->drv_priv;

	vpu_dbg(LVL_BUF, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			q_data->ctx->core_dev->id, q_data->ctx->str_index);

	return 0;
}


static int vpu_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct queue_data *q_data = (struct queue_data *)q->drv_priv;

	vpu_dbg(LVL_BUF, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			q_data->ctx->core_dev->id, q_data->ctx->str_index);
	vb2_clear_last_buffer_dequeued(q);

	return 0;
}

static void vpu_stop_streaming(struct vb2_queue *q)
{
	struct queue_data *q_data = (struct queue_data *)q->drv_priv;

	vpu_dbg(LVL_BUF, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			q_data->ctx->core_dev->id, q_data->ctx->str_index);
	clear_queue(q_data);
}

static void vpu_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue    *vq = vb->vb2_queue;
	struct queue_data   *This = (struct queue_data *)vq->drv_priv;
	struct vb2_data_req *data_req;
	struct vpu_ctx *ctx = This->ctx;

	vpu_dbg(LVL_BUF, "%s(), %s, (%d, %d)\n", __func__, This->desc,
			This->ctx->core_dev->id, This->ctx->str_index);

	data_req = &This->vb2_reqs[vb->index];
	data_req->vb2_buf = vb;
	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		fill_ctx_seq(ctx, data_req);
		fill_vb_sequence(vb, data_req->sequence);
	}
	list_add_tail(&data_req->list, &This->drv_q);
}

static bool is_enc_dma_buf(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;

	if (vb->memory != V4L2_MEMORY_MMAP)
		return false;

	if (vq->mem_ops != &vb2_dma_contig_memops)
		return false;

	return true;
}

static int vpu_enc_buf_init(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct queue_data *queue = vq->drv_priv;
	struct vpu_ctx *ctx = queue->ctx;
	int i;

	vpu_dbg(LVL_BUF, "%s(), %s, (%d, %d)\n", __func__, queue->desc,
			ctx->core_dev->id, ctx->str_index);
	if (!is_enc_dma_buf(vb))
		return 0;

	for (i = 0; i < vb->num_planes; i++)
		vpu_enc_add_dma_size(get_vpu_ctx_attr(ctx),
					vb->planes[i].length);

	return 0;
}

static void vpu_enc_buf_cleanup(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct queue_data *queue = vq->drv_priv;
	struct vpu_ctx *ctx = queue->ctx;
	int i;

	vpu_dbg(LVL_BUF, "%s(), %s, (%d, %d)\n", __func__, queue->desc,
			ctx->core_dev->id, ctx->str_index);

	if (!is_enc_dma_buf(vb))
		return;

	for (i = 0; i < vb->num_planes; i++)
		vpu_enc_sub_dma_size(get_vpu_ctx_attr(ctx),
					vb->planes[i].length);
}

static void vpu_prepare(struct vb2_queue *q)
{
	struct queue_data *q_data = (struct queue_data *)q->drv_priv;

	vpu_dbg(LVL_BUF, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			q_data->ctx->core_dev->id, q_data->ctx->str_index);
}

static void vpu_finish(struct vb2_queue *q)
{
	struct queue_data *q_data = (struct queue_data *)q->drv_priv;

	vpu_dbg(LVL_BUF, "%s(), %s, (%d, %d)\n", __func__, q_data->desc,
			q_data->ctx->core_dev->id, q_data->ctx->str_index);
}

static struct vb2_ops vpu_enc_v4l2_qops = {
	.queue_setup        = vpu_queue_setup,
	.buf_init           = vpu_enc_buf_init,
	.buf_cleanup        = vpu_enc_buf_cleanup,
	.wait_prepare       = vpu_prepare,
	.wait_finish        = vpu_finish,
	.buf_prepare        = vpu_buf_prepare,
	.start_streaming    = vpu_start_streaming,
	.stop_streaming     = vpu_stop_streaming,
	.buf_queue          = vpu_buf_queue,
};

static void init_vb2_queue(struct queue_data *This, unsigned int type,
				struct vpu_ctx *ctx,
				const struct vb2_mem_ops *mem_ops,
				gfp_t gfp_flags)
{
	struct vb2_queue  *vb2_q = &This->vb2_q;
	int ret;

	vpu_log_func();

	// initialze driver queue
	INIT_LIST_HEAD(&This->drv_q);
	INIT_LIST_HEAD(&This->frame_q);
	INIT_LIST_HEAD(&This->frame_idle);
	atomic64_set(&This->frame_count, 0);
	// initialize vb2 queue
	vb2_q->type = type;
	vb2_q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	vb2_q->gfp_flags = gfp_flags;
	vb2_q->ops = &vpu_enc_v4l2_qops;
	vb2_q->drv_priv = This;
	if (mem_ops)
		vb2_q->mem_ops = mem_ops;
	else
		vb2_q->mem_ops = &vb2_dma_contig_memops;
	vb2_q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	vb2_q->dev = &ctx->dev->plat_dev->dev;
	ret = vb2_queue_init(vb2_q);
	if (ret)
		vpu_err("%s vb2_queue_init() failed (%d)!\n",
				__func__,
				ret
				);
	else
		This->vb2_q_inited = true;
}

static void vpu_enc_init_output_queue(struct vpu_ctx *ctx,
					struct queue_data *q)
{
	WARN_ON(!ctx);
	WARN_ON(!q);

	vpu_log_func();

	init_vb2_queue(q,
			V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			ctx,
			&vb2_dma_contig_memops,
			GFP_DMA32);
	q->type = V4L2_SRC;
	sema_init(&q->drv_q_lock, 1);
	q->ctx = ctx;

	q->supported_fmts = formats_yuv_enc;
	q->fmt_count = ARRAY_SIZE(formats_yuv_enc);
	q->current_fmt = &formats_yuv_enc[0];

	q->width = VPU_ENC_WIDTH_DEFAULT;
	q->height = VPU_ENC_HEIGHT_DEFAULT;
	q->rect.left = 0;
	q->rect.top = 0;
	q->rect.width = VPU_ENC_WIDTH_DEFAULT;
	q->rect.height = VPU_ENC_HEIGHT_DEFAULT;
	scnprintf(q->desc, sizeof(q->desc), "OUTPUT");
}

static void vpu_enc_init_capture_queue(struct vpu_ctx *ctx,
					struct queue_data *q)
{
	WARN_ON(!ctx);
	WARN_ON(!q);

	vpu_log_func();

	init_vb2_queue(q,
			V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			ctx,
			&vb2_vmalloc_memops, 0);
	q->type = V4L2_DST;
	sema_init(&q->drv_q_lock, 1);
	q->ctx = ctx;

	q->supported_fmts = formats_compressed_enc;
	q->fmt_count = ARRAY_SIZE(formats_compressed_enc);
	q->current_fmt = &formats_compressed_enc[0];

	q->width = VPU_ENC_WIDTH_DEFAULT;
	q->height = VPU_ENC_HEIGHT_DEFAULT;
	scnprintf(q->desc, sizeof(q->desc), "CAPTURE");
}

static void vpu_enc_init_queue_data(struct vpu_ctx *ctx)
{
	vpu_enc_init_output_queue(ctx, &ctx->q_data[V4L2_SRC]);
	vpu_enc_init_capture_queue(ctx, &ctx->q_data[V4L2_DST]);
}

static void vpu_enc_release_queue_data(struct vpu_ctx *ctx)
{
	vpu_enc_queue_release(&ctx->q_data[V4L2_SRC]);
	vpu_enc_queue_release(&ctx->q_data[V4L2_DST]);
}

static void vpu_ctx_power_on(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->core_dev)
		return;

	if (ctx->power_status)
		return;
	pm_runtime_get_sync(ctx->core_dev->generic_dev);
	ctx->power_status = true;
}

static void vpu_ctx_power_off(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->core_dev)
		return;

	if (!ctx->power_status)
		return;
	pm_runtime_put_sync(ctx->core_dev->generic_dev);
	ctx->power_status = false;
}

static int set_vpu_fw_addr(struct vpu_dev *dev, struct core_device *core_dev)
{
	off_t reg_fw_base;

	if (!dev || !core_dev)
		return -EINVAL;

	vpu_enc_mu_enable_rx(core_dev);
	reg_fw_base = core_dev->reg_csr_base;
	write_vpu_reg(dev, core_dev->m0_p_fw_space_phy, reg_fw_base + CSR_CM0Px_ADDR_OFFSET);
	write_vpu_reg(dev, 0x0, reg_fw_base + CSR_CM0Px_CPUWAIT);

	return 0;
}

static void cleanup_firmware_memory(struct core_device *core_dev)
{
	memset_io(core_dev->m0_p_fw_space_vir, 0, core_dev->fw_buf_size);
}

static int vpu_firmware_download(struct vpu_dev *This, u_int32 core_id)
{
	const struct firmware *m0_pfw = NULL;
	const u8 *image;
	unsigned int FW_Size = 0;
	int ret = 0;
	struct core_device *core_dev = &This->core_dev[core_id];
	char *p = This->core_dev[core_id].m0_p_fw_space_vir;

	vpu_log_func();

	ret = request_firmware(&m0_pfw, M0FW_FILENAME, This->generic_dev);
	if (ret) {
		vpu_err("%s() request fw %s failed(%d)\n",
			__func__, M0FW_FILENAME, ret);

		return ret;
	}
	vpu_dbg(LVL_DEBUG, "%s() request fw %s got size(%ld)\n",
			__func__, M0FW_FILENAME, m0_pfw->size);

	image = m0_pfw->data;
	FW_Size = min_t(u32, m0_pfw->size, This->core_dev[core_id].fw_buf_size);
	This->core_dev[core_id].fw_actual_size = FW_Size;

	cleanup_firmware_memory(core_dev);
	memcpy(core_dev->m0_p_fw_space_vir, image, FW_Size);
	p[16] = This->plat_type;
	p[17] = core_id + 1;
	p[18] = 1;
	set_vpu_fw_addr(This, &This->core_dev[core_id]);

	release_firmware(m0_pfw);
	m0_pfw = NULL;

	return ret;
}

static int download_vpu_firmware(struct vpu_dev *dev,
				struct core_device *core_dev)
{
	int ret = 0;

	if (!dev || !core_dev)
		return -EINVAL;

	if (core_dev->fw_is_ready)
		return 0;

	vpu_dbg(LVL_INFO, "download firmware for core[%d]\n", core_dev->id);
	reinit_completion(&core_dev->boot_cmp);
	ret = vpu_firmware_download(dev, core_dev->id);
	if (ret) {
		vpu_err("error: vpu_firmware_download fail\n");
		goto exit;
	}
	wait_for_boot_done(core_dev, 0);
	if (!core_dev->firmware_started) {
		vpu_err("core[%d] start firmware failed\n", core_dev->id);
		ret = -EINVAL;
		goto exit;
	}

	set_core_fw_status(core_dev, true);
	clear_core_hang(core_dev);
exit:
	return ret;
}

static bool is_valid_ctx(struct vpu_ctx *ctx)
{
	if (!ctx)
		return false;
	if (!ctx->dev || !ctx->core_dev)
		return false;
	if (ctx->str_index >= ARRAY_SIZE(ctx->core_dev->ctx))
		return false;

	return true;
}

static void free_instance(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;

	if (is_valid_ctx(ctx))
		ctx->core_dev->ctx[ctx->str_index] = NULL;
	VPU_SAFE_RELEASE(ctx, kfree);
}

static u32 count_free_core_slot(struct core_device *core)
{
	u32 count = 0;
	int i;

	for (i = 0; i < core->supported_instance_count; i++) {
		if (!core->ctx[i])
			count++;
	}

	return count;
}

static struct core_device *find_proper_core(struct vpu_dev *dev)
{
	struct core_device *core = NULL;
	u32 maximum = 0;
	u32 count;
	int i;
	int ret;

	for (i = 0; i < dev->core_num; i++) {
		struct core_device *core_dev = &dev->core_dev[i];

		ret = process_core_hang(core_dev);
		if (ret)
			continue;

		ret = download_vpu_firmware(dev, core_dev);
		if (ret)
			continue;

		if (core_dev->supported_instance_count == 0)
			continue;

		count = count_free_core_slot(core_dev);
		if (count == core_dev->supported_instance_count)
			return core_dev;

		if (maximum < count) {
			core = core_dev;
			maximum = count;
		}
	}

	return core;
}

static int request_instance(struct core_device *core, struct vpu_ctx *ctx)
{
	int found = 0;
	int idx;

	if (!core || !ctx)
		return -EINVAL;

	for (idx = 0; idx < core->supported_instance_count; idx++) {
		if (!core->ctx[idx]) {
			found = 1;
			ctx->core_dev = core;
			ctx->str_index = idx;
			ctx->dev = core->vdev;
			break;
		}
	}

	if (!found) {
		vpu_err("cann't request any instance\n");
		return -EBUSY;
	}

	return 0;
}

static int construct_vpu_ctx(struct vpu_ctx *ctx)
{
	if (!ctx)
		return -EINVAL;

	ctx->ctrl_inited = false;
	mutex_init(&ctx->instance_mutex);
	ctx->ctx_released = false;

	return 0;
}

static struct vpu_ctx *create_and_request_instance(struct vpu_dev *dev)
{
	struct core_device *core = NULL;
	struct vpu_ctx *ctx = NULL;
	int ret;

	if (!dev)
		return NULL;

	core = find_proper_core(dev);
	if (!core)
		return NULL;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	ret = request_instance(core, ctx);
	if (ret < 0) {
		VPU_SAFE_RELEASE(ctx, kfree);
		return NULL;
	}

	construct_vpu_ctx(ctx);
	vpu_ctx_power_on(ctx);
	vpu_dbg(LVL_INFO, "request encoder instance : %d.%d\n",
			ctx->core_dev->id, ctx->str_index);

	return ctx;
}

static int init_vpu_ctx_fh(struct vpu_ctx *ctx, struct vpu_dev *dev)
{
	if (!ctx || !dev)
		return -EINVAL;

	mutex_lock(&ctx->instance_mutex);

	v4l2_fh_init(&ctx->fh, dev->pvpu_encoder_dev);
	v4l2_fh_add(&ctx->fh);
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	clear_bit(VPU_ENC_STATUS_CLOSED, &ctx->status);

	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static void uninit_vpu_ctx_fh(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;

	mutex_lock(&ctx->instance_mutex);

	set_bit(VPU_ENC_STATUS_CLOSED, &ctx->status);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	mutex_unlock(&ctx->instance_mutex);
}

static void cancel_vpu_ctx(struct vpu_ctx *ctx)
{
	cancel_work_sync(&ctx->instance_work);
	cleanup_ctx_msg_queue(ctx);
}

static void uninit_vpu_ctx(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;

	clear_bit(VPU_ENC_STATUS_INITIALIZED, &ctx->status);
	cancel_vpu_ctx(ctx);
	if (ctx->instance_wq) {
		destroy_workqueue(ctx->instance_wq);
		ctx->instance_wq = NULL;
	}
	mutex_lock(&ctx->instance_mutex);
	vpu_enc_free_stream(ctx);

	ctx->ctx_released = true;
	mutex_unlock(&ctx->instance_mutex);
}

static int init_vpu_ctx(struct vpu_ctx *ctx)
{
	INIT_WORK(&ctx->instance_work, vpu_enc_msg_instance_work);
	ctx->instance_wq = alloc_workqueue("vpu_instance",
				WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!ctx->instance_wq) {
		vpu_err("error: unable to alloc workqueue for ctx\n");
		return -ENOMEM;
	}

	init_ctx_msg_queue(ctx);

	vpu_enc_init_queue_data(ctx);
	vpu_enc_set_default_color(ctx, V4L2_COLORSPACE_REC709);
	init_completion(&ctx->start_cmp);
	init_completion(&ctx->stop_cmp);

	set_bit(VPU_ENC_STATUS_INITIALIZED, &ctx->status);
	ctx->core_dev->ctx[ctx->str_index] = ctx;

	return 0;
}

static int show_encoder_param(struct vpu_attr *attr,
		pMEDIAIP_ENC_PARAM param, char *buf, u32 size)
{
	pMEDIAIP_ENC_EXPERT_MODE_PARAM pEncExpertModeParam = NULL;
	int num = 0;

	pEncExpertModeParam = rpc_get_expert_mode_param(&attr->core->shared_mem,
							attr->index);
	num += scnprintf(buf + num, size - num,
			"encoder param:[setting/take effect]\n");
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Codec Mode",
			attr->param.eCodecMode, param->eCodecMode);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Profile",
			attr->param.eProfile, param->eProfile);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Level",
			attr->param.uLevel, param->uLevel);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%2d(%d / %d);%2d(%d / %d)\n",
			"Frame Rate",
			attr->param.uFrameRate,
			attr->fival.numerator,
			attr->fival.denominator,
			param->uFrameRate,
			pEncExpertModeParam->Config.frame_rate_num,
			pEncExpertModeParam->Config.frame_rate_den);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Source Stride",
			attr->param.uSrcStride, param->uSrcStride);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Source Width",
			attr->param.uSrcWidth, param->uSrcWidth);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Source Height",
			attr->param.uSrcHeight, param->uSrcHeight);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Source Offset x",
			attr->param.uSrcOffset_x, param->uSrcOffset_x);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Source Offset y",
			attr->param.uSrcOffset_y, param->uSrcOffset_y);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Source Crop Width",
			attr->param.uSrcCropWidth, param->uSrcCropWidth);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Source Crop Height",
			attr->param.uSrcCropHeight,
			param->uSrcCropHeight);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Out Width",
			attr->param.uOutWidth, param->uOutWidth);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Out Height",
			attr->param.uOutHeight, param->uOutHeight);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "I Frame Interval",
			attr->param.uIFrameInterval,
			param->uIFrameInterval);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Bframes",
			attr->param.uGopBLength, param->uGopBLength);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Low Latency Mode",
			attr->param.uLowLatencyMode,
			param->uLowLatencyMode);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Bitrate Mode",
			attr->param.eBitRateMode, param->eBitRateMode);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Target Bitrate",
			attr->param.uTargetBitrate,
			param->uTargetBitrate);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Min Bitrate",
			attr->param.uMinBitRate, param->uMinBitRate);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "Max Bitrate",
			attr->param.uMaxBitRate, param->uMaxBitRate);
	num += scnprintf(buf + num, size - num,
			"\t%-18s:%10d;%10d\n", "QP",
			attr->param.uInitSliceQP,
			param->uInitSliceQP);

	return num;
}

static int show_queue_buffer_info(struct queue_data *queue, char *buf, u32 size)
{
	int i;
	int num = 0;

	for (i = 0; i < queue->vb2_q.num_buffers; i++) {
		struct vb2_buffer *vb = queue->vb2_q.bufs[i];

		num += scnprintf(buf + num, size - num, " %d", vb->state);
	}

	return num;
}

static int show_cmd_event(struct vpu_statistic *statistic, int i,
				char *buf, u32 size)
{
	int num = 0;

	num += scnprintf(buf + num, PAGE_SIZE - num, "\t(%2d) ", i);

	if (i <= GTB_ENC_CMD_RESERVED)
		num += scnprintf(buf + num, PAGE_SIZE - num, "%-28s:%12ld;",
				get_cmd_str(i), statistic->cmd[i]);
	else
		num += scnprintf(buf + num, PAGE_SIZE - num, "%-28s:%12s;",
				"", "");

	num += scnprintf(buf + num, PAGE_SIZE - num, "    ");
	if (i <= VID_API_ENC_EVENT_RESERVED)
		num += scnprintf(buf + num, PAGE_SIZE - num, "%-34s:%12ld;",
				get_event_str(i), statistic->event[i]);

	num += scnprintf(buf + num, PAGE_SIZE - num, "\n");

	return num;
}

static int show_cmd_event_infos(struct vpu_statistic *statistic,
		char *buf, u32 size)
{
	int num = 0;
	int i;
	int count;

	num += scnprintf(buf + num, size - num, "command/event:\n");

	count = max((int)GTB_ENC_CMD_RESERVED, (int)VID_API_ENC_EVENT_RESERVED);
	for (i = 0; i <= count; i++)
		num += show_cmd_event(statistic, i, buf + num, size - num);

	num += scnprintf(buf + num, size - num, "current status:\n");
	num += scnprintf(buf + num, size - num,
			"\t%-10s:%36s;%10lld.%06ld\n", "commond",
			get_cmd_str(statistic->current_cmd),
			statistic->ts_cmd.tv_sec,
			statistic->ts_cmd.tv_nsec / 1000);
	num += scnprintf(buf + num, size - num,
			"\t%-10s:%36s;%10lld.%06ld\n", "event",
			get_event_str(statistic->current_event),
			statistic->ts_event.tv_sec,
			statistic->ts_event.tv_nsec / 1000);

	return num;
}

static int show_single_fps_info(struct vpu_fps_sts *fps, char *buf, u32 size)
{
	const u32 COEF = VPU_FPS_COEF;
	int num = 0;

	num += scnprintf(buf + num, size - num, "%3ld.", fps->fps / COEF);
	num += scnprintf(buf + num, size - num, "%02ld", fps->fps % COEF);
	if (fps->thd)
		num += scnprintf(buf + num, size - num, "(%ds)", fps->thd);
	else
		num += scnprintf(buf + num, size - num, "(avg)");

	return num;
}

static int show_fps_info(struct vpu_fps_sts *fps, int count,
			char *buf, u32 size)
{
	int i;
	int num = 0;

	for (i = 0; i < count; i++) {
		if (i > 0)
			num += scnprintf(buf + num, size - num, "  ");
		num += show_single_fps_info(&fps[i], buf + num, size - num);
	}

	return num;
}

static int show_frame_sts(struct vpu_statistic *statistic, char *buf, u32 size)
{
	int num = 0;

	num += scnprintf(buf + num, size - num,
			"frame count:\n");
	num += scnprintf(buf + num, size - num, "\t%-24s:%ld\n",
			"dbuf input yuv count", statistic->yuv_count);
	num += scnprintf(buf + num, size - num, "\t%-24s:%ld\n",
			"encode frame count", statistic->encoded_count);
	num += scnprintf(buf + num, size - num, "\t%-24s:%ld\n",
			"dqbuf output h264 count", statistic->h264_count);

	num += scnprintf(buf + num, size - num, "\t%-24s:", "actual fps:");
	num += show_fps_info(statistic->fps, ARRAY_SIZE(statistic->fps),
				buf + num, PAGE_SIZE - num);
	num += scnprintf(buf + num, size - num, "\n");
	num += scnprintf(buf + num, size - num, "\t%-24s:%ld\n",
			"timestamp overwrite", statistic->timestamp_overwrite);

	return num;
}

static int show_strip_info(struct vpu_statistic *statistic, char *buf, u32 size)
{
	int num = 0;

	num += scnprintf(buf + num, size - num,
			"strip data frame count:\n");
	num += scnprintf(buf + num, size - num,
			"\t fw      :%16ld (max : %ld; total : %ld)\n",
			statistic->strip_sts.fw.count,
			statistic->strip_sts.fw.max,
			statistic->strip_sts.fw.total);
	num += scnprintf(buf + num, size - num,
			"\t begin   :%16ld (max : %ld; total : %ld)\n",
			statistic->strip_sts.begin.count,
			statistic->strip_sts.begin.max,
			statistic->strip_sts.begin.total);
	num += scnprintf(buf + num, size - num,
			"\t eos     :%16ld (max : %ld; total : %ld)\n",
			statistic->strip_sts.eos.count,
			statistic->strip_sts.eos.max,
			statistic->strip_sts.eos.total);

	return num;
}

static int show_v4l2_buf_status(struct vpu_ctx *ctx, char *buf, u32 size)
{
	int num = 0;

	num += scnprintf(buf + num, size - num, "V4L2 Buffer Status: ");
	num += scnprintf(buf + num, size - num, "(");
	num += scnprintf(buf + num, size - num,
			" %d:dequeued,", VB2_BUF_STATE_DEQUEUED);
	num += scnprintf(buf + num, size - num,
			" %d:active,", VB2_BUF_STATE_ACTIVE);
	num += scnprintf(buf + num, size - num,
			" %d:done,", VB2_BUF_STATE_DONE);
	num += scnprintf(buf + num, size - num,
			" %d:error", VB2_BUF_STATE_ERROR);
	num += scnprintf(buf + num, size - num, ")\n");
	num += scnprintf(buf + num, size - num, "\tOUTPUT(0x%lx):",
			ctx->q_data[V4L2_SRC].rw_flag);
	num += show_queue_buffer_info(&ctx->q_data[V4L2_SRC],
					buf + num,
					size - num);
	num += scnprintf(buf + num, size - num, "    CAPTURE(0x%lx):",
			ctx->q_data[V4L2_DST].rw_flag);
	num += show_queue_buffer_info(&ctx->q_data[V4L2_DST],
					buf + num,
					size - num);
	num += scnprintf(buf + num, size - num, "\n");

	return num;
}

static int show_instance_status(struct vpu_ctx *ctx, char *buf, u32 size)
{
	int num = 0;

	num += scnprintf(buf + num, size - num, "instance status:\n");
	num += scnprintf(buf + num, size - num,
			"\t%-13s:0x%lx\n", "status", ctx->status);
	num += scnprintf(buf + num, size - num,
			"\t%-13s:%d\n", "frozen count", ctx->frozen_count);

	return num;
}

static int show_instance_stream_buffer_desc(struct vpu_ctx *ctx,
						char *buf, u32 size)
{
	pBUFFER_DESCRIPTOR_TYPE stream_buffer_desc;
	int num = 0;

	stream_buffer_desc = get_rpc_stream_buffer_desc(ctx);
	num += scnprintf(buf + num, size - num,
			"\t%-13s:0x%x\n", "start",
			get_ptr(stream_buffer_desc->start));
	num += scnprintf(buf + num, size - num,
			"\t%-13s:0x%x\n", "end",
			get_ptr(stream_buffer_desc->end));
	num += scnprintf(buf + num, size - num,
			"\t%-13s:0x%x\n", "rptr",
			get_ptr(stream_buffer_desc->rptr));
	num += scnprintf(buf + num, size - num,
			"\t%-13s:0x%x\n", "wptr",
			get_ptr(stream_buffer_desc->wptr));
	num += scnprintf(buf + num, size - num,
			"\t%-13s:0x%x\n", "free space",
			get_free_space(ctx));
	return num;
}

static int show_instance_others(struct vpu_attr *attr, char *buf, u32 size)
{
	int num = 0;
	struct vpu_ctx *ctx = NULL;
	struct vpu_dev *vpudev = attr->core->vdev;

	num += scnprintf(buf + num, size - num, "others:\n");
	if (attr->ts_start[V4L2_SRC] && attr->ts_start[V4L2_DST]) {
		unsigned long latency;

		latency = attr->ts_start[V4L2_DST] - attr->ts_start[V4L2_SRC];
		num += scnprintf(buf + num, size - num,
				"\tlatency(ms)               :%ld\n", latency);
	}

	num += scnprintf(buf + num, size - num,
			"\ttotal dma size            :%lld\n",
			atomic64_read(&attr->total_dma_size));
	num += scnprintf(buf + num, size - num,
			"\ttotal event msg obj count :%ld\n",
			attr->msg_count);
	num += scnprintf(buf + num, size - num,
			"\ttotal msg ext data count  :%lld\n",
			get_total_ext_data_number());

	mutex_lock(&vpudev->dev_mutex);
	ctx = get_vpu_attr_ctx(attr);
	if (ctx) {
		num += scnprintf(buf + num, size - num,
			"\tis the msg queue empty    :%d\n",
			is_event_msg_empty(ctx));
		num += scnprintf(buf + num, size - num,
			"\ttotal frame obj count     :%lld\n",
			atomic64_read(&ctx->q_data[V4L2_DST].frame_count));

		if (test_bit(VPU_ENC_STATUS_HANG, &ctx->status))
			num += scnprintf(buf + num, size - num, "<hang>\n");
	} else {
		num += scnprintf(buf + num, size - num,
			"<instance has been released>\n");
	}
	mutex_unlock(&vpudev->dev_mutex);

	return num;
}

static ssize_t show_instance_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_attr *vpu_attr;
	struct vpu_dev *vpudev;
	struct vpu_statistic *statistic;
	struct vpu_ctx *ctx;
	pMEDIAIP_ENC_PARAM param;
	int num = 0;

	vpu_attr = container_of(attr, struct vpu_attr,  dev_attr);
	vpudev = vpu_attr->core->vdev;

	num += scnprintf(buf + num, PAGE_SIZE,
			"pid: %d; tgid: %d\n", vpu_attr->pid, vpu_attr->tgid);

	param = rpc_get_enc_param(&vpu_attr->core->shared_mem, vpu_attr->index);
	num += show_encoder_param(vpu_attr, param, buf + num, PAGE_SIZE - num);

	statistic = &vpu_attr->statistic;
	num += show_cmd_event_infos(statistic, buf + num, PAGE_SIZE - num);
	num += show_frame_sts(statistic, buf + num, PAGE_SIZE - num);
	num += show_strip_info(statistic, buf + num, PAGE_SIZE - num);

	mutex_lock(&vpudev->dev_mutex);
	ctx = get_vpu_attr_ctx(vpu_attr);
	if (ctx) {
		num += show_v4l2_buf_status(ctx, buf + num, PAGE_SIZE - num);
		num += show_instance_status(ctx, buf + num, PAGE_SIZE - num);
		num += show_instance_stream_buffer_desc(ctx,
							buf + num,
							PAGE_SIZE - num);
	}
	mutex_unlock(&vpudev->dev_mutex);

	num += show_instance_others(vpu_attr, buf + num, PAGE_SIZE - num);

	return num;
}

static ssize_t show_core_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct core_device *core = NULL;
	char *fw = NULL;
	int num = 0;

	core = container_of(attr, struct core_device, core_attr);
	fw = core->m0_p_fw_space_vir;

	num += scnprintf(buf + num, PAGE_SIZE - num,
			"core[%d] info:\n", core->id);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"vpu mu id       : %d\n", core->vpu_mu_id);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"irq             : %d\n", core->irq);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"reg mu mcu      : 0x%08x 0x%08x\n",
			core->reg_base, core->reg_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"reg csr         : 0x%08x 0x%08x\n",
			core->reg_csr_base, core->reg_csr_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"fw space phy    : 0x%08x\n", core->m0_p_fw_space_phy);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"fw space size   : 0x%08x\n", core->fw_buf_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"fw actual size  : 0x%08x\n", core->fw_actual_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"rpc phy         : 0x%08x\n", core->m0_rpc_phy);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"rpc buf size    : 0x%08x\n", core->rpc_buf_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"rpc actual size : 0x%08x\n", core->rpc_actual_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"print buf phy   : 0x%08x\n",
			core->m0_rpc_phy + core->rpc_buf_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"print buf size  : 0x%08x\n", core->print_buf_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"max instance num: %d\n",
			core->supported_instance_count);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"fw_is_ready     : %d\n", core->fw_is_ready);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"firmware_started: %d\n", core->firmware_started);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"hang            : %d\n", core->hang);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"reset times     : %ld\n", core->reset_times);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"heartbeat       : %02x\n", core->vdev->heartbeat);
	if (core->fw_is_ready) {
		pENC_RPC_HOST_IFACE iface = core->shared_mem.pSharedInterface;

		num += scnprintf(buf + num, PAGE_SIZE - num,
			"firmware version: %d.%d.%d\n",
			(iface->FWVersion & 0x00ff0000) >> 16,
			(iface->FWVersion & 0x0000ff00) >> 8,
			iface->FWVersion & 0x000000ff);
		num += scnprintf(buf + num, PAGE_SIZE - num,
			"fw info         : 0x%02x 0x%02x\n", fw[16], fw[17]);
	}
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"driver version  : %s\n", VPU_ENC_DRIVER_VERSION);

	return num;
}

static int show_vb2_memory(struct vb2_buffer *vb, char *buf, u32 size)
{
	int num = 0;
	int i;

	for (i = 0; i < vb->num_planes; i++) {
		num += scnprintf(buf + num, size - num, "0x%8x 0x%x",
				get_vb2_plane_phy_addr(vb, i),
				vb->planes[i].length);
		if (i == vb->num_planes - 1)
			num += scnprintf(buf + num, size - num, "\n");
		else
			num += scnprintf(buf + num, size - num, "; ");
	}

	return num;
}

static int show_queue_memory(struct queue_data *queue, char *buf, u32 size,
				char *prefix)
{
	int num = 0;
	int i;

	num += scnprintf(buf + num, size - num, "%s%4s v4l2buf  :\n", prefix,
			queue->type == V4L2_SRC ? "YUV" : "H264");

	for (i = 0; i < queue->vb2_q.num_buffers; i++) {
		struct vb2_buffer *vb = queue->vb2_q.bufs[i];

		num += scnprintf(buf + num, size - num, "%s%18s", prefix, "");
		num += show_vb2_memory(vb, buf + num, size - num);
	}

	return num;
}

static int show_ctx_memory_details(struct vpu_ctx *ctx, char *buf, u32 size,
				char *prefix)
{
	int num = 0;
	int i;

	if (!ctx)
		return 0;

	num += scnprintf(buf + num, size - num, "%smemory details:\n", prefix);
	num += scnprintf(buf + num, size - num, "%sencFrames    :\n", prefix);
	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES; i++) {
		num += scnprintf(buf + num, size - num, "%s%14s", prefix, "");
		num += scnprintf(buf + num, size - num, "[%d] 0x%8llx 0x%x\n",
				i,
				ctx->encFrame[i].phy_addr,
				ctx->encFrame[i].size);
	}

	num += scnprintf(buf + num, size - num, "%srefFrames    :\n", prefix);
	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES; i++) {
		num += scnprintf(buf + num, size - num, "%s%14s", prefix, "");
		num += scnprintf(buf + num, size - num, "[%d] 0x%8llx 0x%x\n",
				i,
				ctx->refFrame[i].phy_addr,
				ctx->refFrame[i].size);
	}

	num += scnprintf(buf + num, size - num, "%sactFrames    :\n", prefix);
	num += scnprintf(buf + num, size - num, "%s%18s", prefix, "");
	num += scnprintf(buf + num, size - num, "0x%8llx 0x%x\n",
			ctx->actFrame.phy_addr, ctx->actFrame.size);

	num += scnprintf(buf + num, size - num, "%sencoderStream:\n", prefix);
	num += scnprintf(buf + num, size - num, "%s%18s", prefix, "");
	num += scnprintf(buf + num, size - num, "0x%8llx 0x%x\n",
			ctx->encoder_stream.phy_addr, ctx->encoder_stream.size);

	for (i = 0; i < ARRAY_SIZE(ctx->q_data); i++) {
		struct queue_data *queue = &ctx->q_data[i];

		if (queue->vb2_q.memory != V4L2_MEMORY_MMAP)
			continue;
		if (queue->vb2_q.mem_ops != &vb2_dma_contig_memops)
			continue;

		num += show_queue_memory(queue, buf + num, size - num, prefix);
	}

	return num;
}

static ssize_t show_memory_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_dev *vdev = dev_get_drvdata(dev);
	unsigned long total_dma_size = 0;
	int num = 0;
	int i;
	int j;
	int core_id = (show_detail_index >> 8) & 0xff;
	int ctx_id = show_detail_index & 0xff;

	num += scnprintf(buf + num, PAGE_SIZE - num, "dma memory usage:\n");
	for (i = 0; i < vdev->core_num; i++) {
		struct core_device *core = &vdev->core_dev[i];

		num += scnprintf(buf + num, PAGE_SIZE - num, "core[%d]\n", i);

		for (j = 0; j < ARRAY_SIZE(core->attr); j++) {
			struct vpu_attr *attr_loc = &core->attr[j];
			unsigned long size;

			size = atomic64_read(&attr_loc->total_dma_size);
			total_dma_size += size;
			num += scnprintf(buf + num, PAGE_SIZE - num,
					"\t[%d] : %ld\n", j, size);
			if (core_id != i || ctx_id != j)
				continue;
			mutex_lock(&vdev->dev_mutex);
			num += show_ctx_memory_details(core->ctx[j],
							buf + num,
							PAGE_SIZE - num,
							"\t\t");
			mutex_unlock(&vdev->dev_mutex);
		}
	}

	num += scnprintf(buf + num, PAGE_SIZE - num,
			"total dma             : %ld\n", total_dma_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"total reserved memory : %ld\n",
			vdev->reserved_mem.bytesused);
	show_detail_index = VPU_DETAIL_INDEX_DFT;

	return num;
}

static ssize_t store_memory_info_index(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val = VPU_DETAIL_INDEX_DFT;

	if (!kstrtol(buf, 0, &val))
		show_detail_index = val;

	return count;
}
DEVICE_ATTR(meminfo, 0664, show_memory_info, store_memory_info_index);

static ssize_t show_buffer_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_dev *vdev = dev_get_drvdata(dev);
	int num = 0;
	int i;
	int j;

	mutex_lock(&vdev->dev_mutex);
	num += scnprintf(buf + num, PAGE_SIZE - num, "vpu encoder buffers:\t");
	num += scnprintf(buf + num, PAGE_SIZE - num, "(");
	num += scnprintf(buf + num, PAGE_SIZE - num,
			" %d:dequeued,", VB2_BUF_STATE_DEQUEUED);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			" %d:active,", VB2_BUF_STATE_ACTIVE);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			" %d:done,", VB2_BUF_STATE_DONE);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			" %d:error", VB2_BUF_STATE_ERROR);
	num += scnprintf(buf + num, PAGE_SIZE - num, ")\n");

	for (i = 0; i < vdev->core_num; i++) {
		struct core_device *core = &vdev->core_dev[i];

		if (!core->supported_instance_count)
			continue;

		num += scnprintf(buf + num, PAGE_SIZE - num, "core[%d]\n", i);
		for (j = 0; j < core->supported_instance_count; j++) {
			struct vpu_ctx *ctx = core->ctx[j];

			if (!ctx)
				continue;
			num += scnprintf(buf + num, PAGE_SIZE - num,
					"\t[%d]: ", j);
			num += scnprintf(buf + num,
					PAGE_SIZE - num, "OUTPUT:");
			num += show_queue_buffer_info(&ctx->q_data[V4L2_SRC],
							buf + num,
							PAGE_SIZE - num);
			num += scnprintf(buf + num,
					PAGE_SIZE - num, "    CAPTURE:");
			num += show_queue_buffer_info(&ctx->q_data[V4L2_DST],
							buf + num,
							PAGE_SIZE - num);
			num += scnprintf(buf + num, PAGE_SIZE - num, "\n");
		}
	}
	mutex_unlock(&vdev->dev_mutex);

	return num;
}
DEVICE_ATTR(buffer, 0444, show_buffer_info, NULL);

static ssize_t show_fpsinfo(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_dev *vdev = dev_get_drvdata(dev);
	int num = 0;
	int i;
	int j;

	for (i = 0; i < vdev->core_num; i++) {
		struct core_device *core = &vdev->core_dev[i];

		if (!core->supported_instance_count)
			continue;

		num += scnprintf(buf + num, PAGE_SIZE - num, "core[%d]\n", i);
		for (j = 0; j < core->supported_instance_count; j++) {
			struct vpu_attr *attr = &core->attr[j];

			if (!attr->created)
				continue;
			num += scnprintf(buf + num, PAGE_SIZE - num,
					"\t[%d]", j);
			num += scnprintf(buf + num, PAGE_SIZE - num,
					"  %3d(%d / %d)(setting)  ",
					attr->param.uFrameRate,
					attr->fival.numerator,
					attr->fival.denominator);
			num += show_fps_info(attr->statistic.fps,
					ARRAY_SIZE(attr->statistic.fps),
					buf + num, PAGE_SIZE - num);
			num += scnprintf(buf + num, PAGE_SIZE - num, "\n");
		}
	}

	return num;
}
DEVICE_ATTR(fpsinfo, 0444, show_fpsinfo, NULL);

static ssize_t show_vpuinfo(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_dev *vdev = dev_get_drvdata(dev);
	int num = 0;

	num += scnprintf(buf + num, PAGE_SIZE - num,
			"core number          : %d\n", vdev->core_num);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"platform type        : %d\n", vdev->plat_type);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"reg-vpu              : 0x%8x 0x%08x\n",
			vdev->reg_vpu_base, vdev->reg_vpu_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"reg-rpc-system       : 0x%08x\n",
			vdev->reg_rpc_system);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"reserved-memory      : 0x%08lx 0x%08lx\n",
			vdev->reserved_mem.phy_addr, vdev->reserved_mem.size);
	num += scnprintf(buf + num, PAGE_SIZE - num, "supported resolution :");
	num += scnprintf(buf + num, PAGE_SIZE - num, " %dx%d(min);",
			vdev->supported_size.min_width,
			vdev->supported_size.min_height);
	num += scnprintf(buf + num, PAGE_SIZE - num, " %dx%d(step);",
			vdev->supported_size.step_width,
			vdev->supported_size.step_height);
	num += scnprintf(buf + num, PAGE_SIZE - num, " %dx%d(max)\n",
			vdev->supported_size.max_width,
			vdev->supported_size.max_height);

	return num;
}
DEVICE_ATTR(vpuinfo, 0444, show_vpuinfo, NULL);

static void reset_statistic(struct vpu_attr *attr)
{
	if (!attr)
		return;

	memset(&attr->statistic, 0, sizeof(attr->statistic));
	attr->statistic.current_cmd = GTB_ENC_CMD_NOOP;
	attr->statistic.current_event = VID_API_EVENT_UNDEFINED;
}

static int init_vpu_attr_fps_sts(struct vpu_attr *attr)
{
	const unsigned int THDS[] = VPU_FPS_STS_THDS;
	int i;

	for (i = 0; i < VPU_FPS_STS_CNT; i++) {
		if (i < ARRAY_SIZE(THDS))
			attr->statistic.fps[i].thd = THDS[i];
		else
			attr->statistic.fps[i].thd = 0;
	}

	return 0;
}

static int enable_fps_sts(struct vpu_attr *attr)
{
	int i;
	struct vpu_statistic *sts = &attr->statistic;

	sts->fps_sts_enable = true;

	for (i = 0; i < VPU_FPS_STS_CNT; i++) {
		ktime_get_raw_ts64(&sts->fps[i].ts);
		sts->fps[i].frame_number = sts->encoded_count;
	}

	return 0;
}

static int disable_fps_sts(struct vpu_attr *attr)
{
	attr->statistic.fps_sts_enable = false;

	return 0;
}

static int init_vpu_attr(struct vpu_attr *attr)
{
	if (!attr || !attr->core)
		return -EINVAL;

	reset_statistic(attr);
	memset(&attr->param, 0, sizeof(attr->param));
	attr->pid = current->pid;
	attr->tgid = current->tgid;
	if (!attr->created) {
		device_create_file(attr->core->generic_dev, &attr->dev_attr);
		attr->created = true;
	}

	init_vpu_attr_fps_sts(attr);

	return 0;
}

static int release_instance(struct vpu_ctx *ctx)
{
	struct vpu_dev *dev;

	if (!ctx || !ctx->dev)
		return -EINVAL;

	if (!test_bit(VPU_ENC_STATUS_CLOSED, &ctx->status))
		return 0;
	if (!test_bit(VPU_ENC_STATUS_FORCE_RELEASE, &ctx->status)) {
		if (test_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status) &&
			!test_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status))
			return -EINVAL;
	}

	dev = ctx->dev;

	uninit_vpu_ctx(ctx);
	vpu_enc_free_ctrls(ctx);
	vpu_enc_release_queue_data(ctx);
	vpu_enc_free_mem(ctx, get_rpc_mem_pool(ctx));

	vpu_ctx_power_off(ctx);
	free_instance(ctx);

	return 0;
}

static int try_to_release_idle_instance(struct vpu_dev *dev)
{
	int i;
	int j;

	if (!dev)
		return -EINVAL;

	for (i = 0; i < dev->core_num; i++) {
		if (dev->core_dev[i].hang)
			set_core_force_release(&dev->core_dev[i]);
		for (j = 0; j < dev->core_dev[i].supported_instance_count; j++)
			release_instance(dev->core_dev[i].ctx[j]);
	}

	return 0;
}

struct vpu_attr *get_vpu_ctx_attr(struct vpu_ctx *ctx)
{
	WARN_ON(!ctx || !ctx->core_dev);

	if (ctx->str_index >= ctx->core_dev->supported_instance_count)
		return NULL;

	return &ctx->core_dev->attr[ctx->str_index];
}

struct vpu_ctx *get_vpu_attr_ctx(struct vpu_attr *attr)
{
	WARN_ON(!attr || !attr->core);

	if (attr->index >= attr->core->supported_instance_count)
		return NULL;

	return attr->core->ctx[attr->index];
}

static int vpu_enc_v4l2_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_dev *dev = video_get_drvdata(vdev);
	struct vpu_ctx *ctx = NULL;
	int ret;

	vpu_log_func();

	mutex_lock(&dev->dev_mutex);
	try_to_release_idle_instance(dev);

	pm_runtime_get_sync(dev->generic_dev);
	ctx = create_and_request_instance(dev);
	pm_runtime_put_sync(dev->generic_dev);
	if (!ctx) {
		mutex_unlock(&dev->dev_mutex);
		vpu_err("failed to create encoder ctx\n");
		return -ENOMEM;
	}
	vpu_dbg(LVL_FLOW, "[%d:%d] open\n",
			ctx->core_dev->id, ctx->str_index);

	init_vpu_attr(get_vpu_ctx_attr(ctx));
	ret = init_vpu_ctx(ctx);
	if (ret) {
		mutex_unlock(&dev->dev_mutex);
		vpu_err("init vpu ctx fail\n");
		goto error;
	}

	initialize_enc_param(ctx);
	vpu_enc_setup_ctrls(ctx);

	init_vpu_ctx_fh(ctx, dev);
	init_ctx_seq_info(ctx);
	filp->private_data = &ctx->fh;
	mutex_unlock(&dev->dev_mutex);

	return 0;
error:
	mutex_lock(&dev->dev_mutex);
	set_bit(VPU_ENC_STATUS_FORCE_RELEASE, &ctx->status);
	VPU_SAFE_RELEASE(ctx, release_instance);
	mutex_unlock(&dev->dev_mutex);
	return ret;
}

static int vpu_enc_v4l2_release(struct file *filp)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);
	struct vpu_dev *dev = ctx->dev;

	vpu_log_func();

	vpu_dbg(LVL_FLOW, "[%d:%d] close\n",
			ctx->core_dev->id, ctx->str_index);
	wait_for_start_done(ctx);
	request_eos(ctx);
	wait_for_stop_done(ctx);

	mutex_lock(&dev->dev_mutex);

	uninit_vpu_ctx_fh(ctx);
	filp->private_data = NULL;

	VPU_SAFE_RELEASE(ctx, release_instance);
	mutex_unlock(&dev->dev_mutex);

	return 0;
}

static unsigned int vpu_enc_v4l2_poll(struct file *filp, poll_table *wait)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);
	struct vb2_queue *src_q, *dst_q;
	unsigned int rc = 0;

	vpu_dbg(LVL_FUNC, "%s(), event: 0x%lx\n", __func__,
			(unsigned long)poll_requested_events(wait));

	poll_wait(filp, &ctx->fh.wait, wait);

	if (v4l2_event_pending(&ctx->fh)) {
		vpu_dbg(LVL_DEBUG, "%s() v4l2_event_pending\n", __func__);
		rc |= POLLPRI;
	}

	src_q = &ctx->q_data[V4L2_SRC].vb2_q;
	dst_q = &ctx->q_data[V4L2_DST].vb2_q;

	if ((!src_q->streaming || list_empty(&src_q->queued_list))
		&& (!dst_q->streaming || list_empty(&dst_q->queued_list))) {
		rc |= POLLERR;
		return rc;
	}
	if (test_bit(VPU_ENC_STATUS_EOS_SEND, &ctx->status) &&
			!list_empty(&dst_q->done_list))
		rc &= ~POLLPRI;

	poll_wait(filp, &src_q->done_wq, wait);
	if (!list_empty(&src_q->done_list))
		rc |= POLLOUT | POLLWRNORM;
	poll_wait(filp, &dst_q->done_wq, wait);
	if (!list_empty(&dst_q->done_list))
		rc |= POLLIN | POLLRDNORM;
	if (dst_q->last_buffer_dequeued)
		rc |= POLLIN | POLLRDNORM;

	return rc;
}

static int vpu_enc_v4l2_mmap(struct file *filp, struct vm_area_struct *vma)
{
	long ret = -EPERM;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	struct queue_data *q_data;
	enum QUEUE_TYPE type;

	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);

	vpu_log_func();

	if (ctx) {
		type = offset >> MMAP_BUF_TYPE_SHIFT;
		q_data = &ctx->q_data[type];

		offset &= ~MMAP_BUF_TYPE_MASK;
		offset = offset >> PAGE_SHIFT;
		vma->vm_pgoff = offset;
		ret = vpu_enc_queue_mmap(q_data, vma);
	}

	return ret;
}

static const struct v4l2_file_operations vpu_enc_v4l2_fops = {
	.owner = THIS_MODULE,
	.open  = vpu_enc_v4l2_open,
	.unlocked_ioctl = video_ioctl2,
	.release = vpu_enc_v4l2_release,
	.poll = vpu_enc_v4l2_poll,
	.mmap = vpu_enc_v4l2_mmap,
};

static struct video_device vpu_enc_v4l2_videodevice = {
	.name   = "vpu encoder",
	.fops   = &vpu_enc_v4l2_fops,
	.ioctl_ops = &vpu_enc_v4l2_ioctl_ops,
	.vfl_dir = VFL_DIR_M2M,
};

static void vpu_enc_setup(struct vpu_dev *This)
{
	const off_t offset = SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL;
	uint32_t read_data = 0;

	vpu_log_func();

	write_vpu_reg(This, 0x1, offset + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	write_vpu_reg(This, 0xffffffff, 0x70190);
	write_vpu_reg(This, 0xffffffff, offset + SCB_BLK_CTRL_XMEM_RESET_SET);
	write_vpu_reg(This, 0xE, offset + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	write_vpu_reg(This, 0x7, offset + SCB_BLK_CTRL_CACHE_RESET_SET);
	write_vpu_reg(This, 0x102, XMEM_CONTROL);

	read_data = read_vpu_reg(This, 0x70108);
	vpu_dbg(LVL_IRQ, "%s read_data=%x\n", __func__, read_data);
}

static int vpu_enc_enable_hw(struct vpu_dev *This)
{
	vpu_log_func();
	if (This->hw_enable)
		return 0;
	vpu_enc_setup(This);

	This->hw_enable = true;

	return 0;
}

static void vpu_enc_disable_hw(struct vpu_dev *This)
{
	This->hw_enable = false;
	if (This->regs_base) {
		iounmap(This->regs_base);
		This->regs_base = NULL;
	}
}

static int parse_core_info(struct core_device *core, struct device_node *np)
{
	int ret;
	u32 val;

	WARN_ON(!core || !np);

	ret = of_property_read_u32_index(np, "reg", 0, &val);
	if (ret) {
		vpu_err("find reg for core[%d] fail\n", core->id);
		return ret;
	}
	core->reg_base = val;

	ret = of_property_read_u32_index(np, "reg", 1, &val);
	if (ret) {
		vpu_err("find reg for core[%d] fail\n", core->id);
		return ret;
	}
	core->reg_size = val;

	ret = of_property_read_u32_index(np, "reg-csr", 0, &val);
	if (ret) {
		vpu_err("find reg-csr for core[%d] fail\n", core->id);
		return ret;
	}
	core->reg_csr_base = val;

	ret = of_property_read_u32_index(np, "reg-csr", 1, &val);
	if (ret) {
		vpu_err("find reg-csr for core[%d] fail\n", core->id);
		return ret;
	}
	core->reg_csr_size = val;

	ret = of_irq_get(np, 0);
	if (ret < 0) {
		vpu_err("get irq for core[%d] fail\n", core->id);
		return -EINVAL;
	}
	core->irq = ret;

	ret = of_property_read_u32(np, "fsl,vpu_ap_mu_id", &val);
	if (!ret)
		core->vpu_mu_id = val;

	ret = of_property_read_u32(np, "fw-buf-size", &val);
	if (ret) {
		vpu_err("find fw-buf-size for core[%d] fail\n", core->id);
		core->fw_buf_size = M0_BOOT_SIZE_DEFAULT;
	} else {
		core->fw_buf_size = val;
	}
	core->fw_buf_size = max_t(u32, core->fw_buf_size, M0_BOOT_SIZE_MIN);

	ret = of_property_read_u32(np, "rpc-buf-size", &val);
	if (ret) {
		vpu_err("find rpc-buf-size for core[%d] fail\n", core->id);
		core->rpc_buf_size = RPC_SIZE_DEFAULT;
	} else {
		core->rpc_buf_size = val;
	}
	core->rpc_buf_size = max_t(u32, core->rpc_buf_size, RPC_SIZE_MIN);

	ret = of_property_read_u32(np, "print-buf-size", &val);
	if (ret) {
		vpu_err("find print-buf-size for core[%d] fail\n", core->id);
		core->print_buf_size = PRINT_SIZE_DEFAULT;
	} else {
		core->print_buf_size = val;
	}
	core->print_buf_size = max_t(u32, core->print_buf_size, PRINT_SIZE_MIN);

	return 0;
}

static int parse_dt_cores(struct vpu_dev *dev, struct device_node *np)
{
	char core_name[64];
	struct device_node *node = NULL;
	struct core_device *core = NULL;
	int i;
	int ret;

	dev->core_num = 0;
	for (i = 0; i < VPU_ENC_MAX_CORE_NUM; i++) {
		scnprintf(core_name, sizeof(core_name), "core%d", i);
		node = of_find_node_by_name(of_node_get(np), core_name);
		if (!node) {
			vpu_dbg(LVL_INFO, "can't find %s\n", core_name);
			break;
		}

		core = &dev->core_dev[i];
		core->id = i;
		ret = parse_core_info(core, node);
		of_node_put(node);
		node = NULL;
		if (ret) {
			vpu_err("parse core[%d] fail\n", i);
			break;
		}
	}
	if (i == 0)
		return -EINVAL;

	dev->core_num = i;

	return 0;
}

static int parse_dt_info(struct vpu_dev *dev, struct device_node *np)
{
	int ret;
	struct device_node *reserved_node = NULL;
	struct resource reserved_fw;
	struct resource reserved_rpc;
	struct resource reserved_mem;
	u32 fw_total_size = 0;
	u32 rpc_total_size = 0;
	u32 val;
	u32 i;

	if (!dev || !np)
		return -EINVAL;

	ret = of_property_read_u32_index(np, "reg-rpc-system", 0, &val);
	if (ret) {
		vpu_err("get reg-rpc-system fail\n");
		return -EINVAL;
	}
	dev->reg_rpc_system = val;

	reserved_node = of_parse_phandle(np, "boot-region", 0);
	if (!reserved_node) {
		vpu_err("error: boot-region of_parse_phandle error\n");
		return -ENODEV;
	}
	if (of_address_to_resource(reserved_node, 0, &reserved_fw)) {
		vpu_err("error: boot-region of_address_to_resource error\n");
		return -EINVAL;
	}

	reserved_node = of_parse_phandle(np, "rpc-region", 0);
	if (!reserved_node) {
		vpu_err("error: rpc-region of_parse_phandle error\n");
		return -ENODEV;
	}
	if (of_address_to_resource(reserved_node, 0, &reserved_rpc)) {
		vpu_err("error: rpc-region of_address_to_resource error\n");
		return -EINVAL;
	}

	reserved_node = of_parse_phandle(np, "reserved-region", 0);
	if (!reserved_node) {
		vpu_err("error: rpc-region of_parse_phandle error\n");
		return -ENODEV;
	}
	if (of_address_to_resource(reserved_node, 0, &reserved_mem)) {
		vpu_err("error: rpc-region of_address_to_resource error\n");
		return -EINVAL;
	}
	dev->reserved_mem.phy_addr = reserved_mem.start;
	dev->reserved_mem.size = resource_size(&reserved_mem);

	ret = parse_dt_cores(dev, np);
	if (ret) {
		vpu_err("parse cores from dt fail\n");
		return ret;
	}

	fw_total_size = 0;
	rpc_total_size = 0;
	for (i = 0; i < dev->core_num; i++) {
		struct core_device *core = &dev->core_dev[i];

		core->m0_p_fw_space_phy = reserved_fw.start + fw_total_size;
		core->m0_rpc_phy = reserved_rpc.start + rpc_total_size;
		fw_total_size += core->fw_buf_size;
		rpc_total_size += core->rpc_buf_size;
		rpc_total_size += core->print_buf_size;
	}

	if (fw_total_size > resource_size(&reserved_fw)) {
		vpu_err("boot-region's size(0x%llx) is less than wanted:0x%x\n",
				resource_size(&reserved_fw), fw_total_size);
		return -EINVAL;
	}
	if (rpc_total_size > resource_size(&reserved_rpc)) {
		vpu_err("rpc-region's size(0x%llx) is less than wanted:0x%x\n",
				resource_size(&reserved_rpc), rpc_total_size);
		return -EINVAL;
	}

	dev->supported_size.min_width = VPU_ENC_WIDTH_MIN;
	dev->supported_size.max_width = VPU_ENC_WIDTH_MAX;
	dev->supported_size.step_width = VPU_ENC_WIDTH_STEP;
	dev->supported_size.min_height = VPU_ENC_HEIGHT_MIN;
	dev->supported_size.max_height = VPU_ENC_HEIGHT_MAX;
	dev->supported_size.step_height = VPU_ENC_HEIGHT_STEP;

	ret = of_property_read_u32_index(np, "resolution-max", 0, &val);
	if (!ret)
		dev->supported_size.max_width = val;

	ret = of_property_read_u32_index(np, "resolution-max", 1, &val);
	if (!ret)
		dev->supported_size.max_height = val;

	return 0;
}

static int create_vpu_video_device(struct vpu_dev *dev)
{
	int ret;

	dev->pvpu_encoder_dev = video_device_alloc();
	if (!dev->pvpu_encoder_dev) {
		vpu_err("alloc vpu encoder video device fail\n");
		return -ENOMEM;
	}

	strlcpy(dev->pvpu_encoder_dev->name,
		vpu_enc_v4l2_videodevice.name,
		sizeof(dev->pvpu_encoder_dev->name));
	dev->pvpu_encoder_dev->fops = vpu_enc_v4l2_videodevice.fops;
	dev->pvpu_encoder_dev->ioctl_ops = vpu_enc_v4l2_videodevice.ioctl_ops;
	dev->pvpu_encoder_dev->release = video_device_release;
	dev->pvpu_encoder_dev->vfl_dir = vpu_enc_v4l2_videodevice.vfl_dir;
	dev->pvpu_encoder_dev->v4l2_dev = &dev->v4l2_dev;
	dev->pvpu_encoder_dev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE |
					     V4L2_CAP_STREAMING;

	video_set_drvdata(dev->pvpu_encoder_dev, dev);

	ret = video_register_device(dev->pvpu_encoder_dev,
					VFL_TYPE_GRABBER,
					ENCODER_NODE_NUMBER);
	if (ret) {
		vpu_err("unable to register video encoder device\n");
		video_device_release(dev->pvpu_encoder_dev);
		dev->pvpu_encoder_dev = NULL;
		return ret;
	}

	return 0;
}

static int init_vpu_attrs(struct core_device *core)
{
	int i;

	WARN_ON(!core);

	for (i = 0; i < ARRAY_SIZE(core->attr); i++) {
		struct vpu_attr *attr = &core->attr[i];

		attr->core = core;
		attr->index = i;
		scnprintf(attr->name, sizeof(attr->name) - 1, "instance.%d.%d",
				core->id, attr->index);
		sysfs_attr_init(&attr->dev_attr.attr);
		attr->dev_attr.attr.name = attr->name;
		attr->dev_attr.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
		attr->dev_attr.show = show_instance_info;

		atomic64_set(&attr->total_dma_size, 0);

		attr->created = false;
	}

	return 0;
}

static int release_vpu_attrs(struct core_device *core)
{
	int i;

	WARN_ON(!core);

	for (i = 0; i < ARRAY_SIZE(core->attr); i++) {
		struct vpu_attr *attr = &core->attr[i];

		if (!attr->created)
			continue;
		device_remove_file(attr->core->generic_dev, &attr->dev_attr);
	}

	return 0;
}

static int is_ctx_frozen(struct vpu_ctx *ctx)
{
	int is_frozen = 1;
	int i;
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

	for (i = 0; i < GTB_ENC_CMD_RESERVED; i++) {
		if (attr->statistic.cmd[i] != ctx->sts.cmd[i])
			is_frozen = 0;
		ctx->sts.cmd[i] = attr->statistic.cmd[i];
	}

	for (i = 0; i < VID_API_ENC_EVENT_RESERVED; i++) {
		if (attr->statistic.event[i] != ctx->sts.event[i])
			is_frozen = 0;
		ctx->sts.event[i] = attr->statistic.event[i];
	}

	if (ctx->sts.cmd[GTB_ENC_CMD_FRAME_ENCODE] ==
			ctx->sts.event[VID_API_ENC_EVENT_FRAME_DONE])
		is_frozen = 0;

	if (ctx->sts.cmd[GTB_ENC_CMD_CONFIGURE_CODEC] >
			ctx->sts.event[VID_API_ENC_EVENT_MEM_REQUEST])
		is_frozen = 1;
	if (ctx->sts.cmd[GTB_ENC_CMD_STREAM_START] >
			ctx->sts.event[VID_API_ENC_EVENT_START_DONE])
		is_frozen = 1;
	if (ctx->sts.cmd[GTB_ENC_CMD_STREAM_STOP] >
			ctx->sts.event[VID_API_ENC_EVENT_STOP_DONE])
		is_frozen = 1;

	return is_frozen;
}

static bool check_vpu_ctx_is_hang(struct vpu_ctx *ctx)
{
	if (is_ctx_frozen(ctx))
		ctx->frozen_count++;
	else
		ctx->frozen_count = 0;

	if (ctx->frozen_count > VPU_ENC_HANG_THD) {
		set_bit(VPU_ENC_STATUS_HANG, &ctx->status);
		ctx->frozen_count = VPU_ENC_HANG_THD;
	} else if (ctx->frozen_count == 0) {
		clear_bit(VPU_ENC_STATUS_HANG, &ctx->status);
	}

	if (test_bit(VPU_ENC_STATUS_HANG, &ctx->status))
		return true;

	return false;
}

static void check_vpu_core_is_hang(struct core_device *core)
{
	int i;
	unsigned int instance_count = 0;
	unsigned int hang_count = 0;

	for (i = 0; i < core->supported_instance_count; i++) {
		if (!core->ctx[i])
			continue;

		if (check_vpu_ctx_is_hang(core->ctx[i]))
			hang_count++;
		instance_count++;
	}

	if (!instance_count)
		return;
	if (hang_count == instance_count)
		set_core_hang(core);
	else
		clear_core_hang(core);
}

static void handle_vpu_core_watchdog(struct core_device *core)
{
	if (!core->fw_is_ready)
		return;
	if (core->suspend)
		return;
	if (core->snapshot)
		return;

	queue_work(core->workqueue, &core->msg_work);
	vpu_enc_notify_msg_event(core);
	check_vpu_core_is_hang(core);
}

static unsigned long get_timestamp_ns(struct timespec64 *ts)
{
	if (!ts)
		return 0;

	return ts->tv_sec * NSEC_PER_SEC + ts->tv_nsec;
}

static void calc_rt_fps(struct vpu_fps_sts *fps,
			unsigned long number, struct timespec64 *ts)
{
	unsigned long delta_num;
	unsigned long delta_ts;

	if (!fps || !ts)
		return;

	fps->times++;
	if (fps->times < fps->thd)
		return;

	if (number >= fps->frame_number) {
		delta_num = number - fps->frame_number;
		delta_ts = get_timestamp_ns(ts) - get_timestamp_ns(&fps->ts);
		if (!delta_ts)
			return;
		fps->fps = delta_num * NSEC_PER_SEC * VPU_FPS_COEF / delta_ts;
	}
	fps->times = 0;
	if (fps->thd) {
		fps->frame_number = number;
		memcpy(&fps->ts, ts, sizeof(fps->ts));
	}
}

static void statistic_fps_info(struct vpu_statistic *sts)
{
	unsigned long encoded_count = sts->encoded_count;
	struct timespec64 ts;
	int i;

	if (!sts->fps_sts_enable)
		return;
	ktime_get_raw_ts64(&ts);
	for (i = 0; i < VPU_FPS_STS_CNT; i++)
		calc_rt_fps(&sts->fps[i], encoded_count, &ts);
}

static void handle_core_minors(struct core_device *core)
{
	int i;

	for (i = 0; i < core->supported_instance_count; i++)
		statistic_fps_info(&core->attr[i].statistic);
}

static void vpu_enc_watchdog_handler(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct vpu_dev *vdev;
	int i;

	if (!work)
		return;

	dwork = to_delayed_work(work);
	vdev = container_of(dwork, struct vpu_dev, watchdog);

	mutex_lock(&vdev->dev_mutex);
	for (i = 0; i < vdev->core_num; i++)
		handle_vpu_core_watchdog(&vdev->core_dev[i]);
	mutex_unlock(&vdev->dev_mutex);

	for (i = 0; i < vdev->core_num; i++)
		handle_core_minors(&vdev->core_dev[i]);

	vdev->heartbeat++;
	schedule_delayed_work(&vdev->watchdog,
			msecs_to_jiffies(VPU_WATCHDOG_INTERVAL_MS));
}

static int fwlog_show(struct seq_file *s, void *data)
{
	struct core_device *core = s->private;
	int length;
	u32 rptr;
	u32 wptr;
	int ret = 0;

	if (!core->print_buf)
		return 0;

	rptr = core->print_buf->read;
	wptr = core->print_buf->write;

	if (rptr == wptr)
		return 0;
	else if (rptr < wptr)
		length = wptr - rptr;
	else
		length = core->print_buf->bytes + wptr - rptr;

	if (s->count + length >= s->size) {
		s->count = s->size;
		return 0;
	}

	if (rptr + length > core->print_buf->bytes) {
		int num = core->print_buf->bytes - rptr;

		if (seq_write(s, core->print_buf->buffer + rptr, num))
			ret = -1;
		length -= num;
		rptr = 0;
	}

	if (seq_write(s, core->print_buf->buffer + rptr, length))
		ret = -1;
	rptr += length;
	rptr %= core->print_buf->bytes;
	if (!ret)
		core->print_buf->read = rptr;

	return 0;
}

static int fwlog_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, fwlog_show, inode->i_private);
}

static const struct file_operations fwlog_fops = {
	.owner = THIS_MODULE,
	.open = fwlog_open,
	.release = single_release,
	.read = seq_read,
};

static void vpu_enc_create_debugfs_file(struct vpu_dev *dev)
{
	struct core_device *core;
	char name[64];
	int i;

	if (dev->debugfs_root == NULL) {
		dev->debugfs_root = debugfs_create_dir("vpu_windsor", NULL);
		if (!dev->debugfs_root) {
			vpu_err("error: create debugfs_root fail\n");
			return;
		}
	}

	for (i = 0; i < dev->core_num; i++) {
		core = &dev->core_dev[i];

		if (core->debugfs_fwlog)
			continue;

		scnprintf(name, sizeof(name), "vpu_windsor_log%d", i);
		core->debugfs_fwlog = debugfs_create_file((const char *)name,
						VERIFY_OCTAL_PERMISSIONS(0444),
						dev->debugfs_root,
						core,
						&fwlog_fops);
		if (!core->debugfs_fwlog) {
			vpu_err("error: create debugfs_fwlog fail\n");
			continue;
		}
	}
}

static void vpu_enc_remove_debugfs_file(struct vpu_dev *dev)
{
	struct core_device *core;
	int i;

	if (!dev)
		return;

	for (i = 0; i < dev->core_num; i++) {
		core = &dev->core_dev[i];

		debugfs_remove(core->debugfs_fwlog);
		core->debugfs_fwlog = NULL;
	}
	debugfs_remove_recursive(dev->debugfs_root);
	dev->debugfs_root = NULL;
}

static void vpu_enc_init_core_rpc(struct core_device *core_dev)
{
	u32 mu_addr;

	cleanup_firmware_memory(core_dev);
	memset_io(core_dev->m0_rpc_virt, 0, core_dev->rpc_buf_size);

	rpc_init_shared_memory_encoder(&core_dev->shared_mem,
				cpu_phy_to_mu(core_dev, core_dev->m0_rpc_phy),
				core_dev->m0_rpc_virt, core_dev->rpc_buf_size,
				&core_dev->rpc_actual_size);
	rpc_set_system_cfg_value_encoder(core_dev->shared_mem.pSharedInterface,
				core_dev->vdev->reg_rpc_system, core_dev->id);

	if (core_dev->rpc_actual_size > core_dev->rpc_buf_size)
		vpu_err("rpc actual size(0x%x) > (0x%x), may occur overlay\n",
			core_dev->rpc_actual_size, core_dev->rpc_buf_size);

	mu_addr = cpu_phy_to_mu(core_dev, core_dev->m0_rpc_phy + core_dev->rpc_buf_size);
	rpc_set_print_buffer(&core_dev->shared_mem, mu_addr, core_dev->print_buf_size);
	core_dev->print_buf = core_dev->m0_rpc_virt + core_dev->rpc_buf_size;

	reset_vpu_core_dev(core_dev);
}

static void vpu_enc_restore_core_rpc(struct core_device *core_dev)
{
	rpc_restore_shared_memory_encoder(&core_dev->shared_mem,
				cpu_phy_to_mu(core_dev, core_dev->m0_rpc_phy),
				core_dev->m0_rpc_virt);
	core_dev->print_buf = core_dev->m0_rpc_virt + core_dev->rpc_buf_size;
	sw_reset_firmware(core_dev, 0);
	set_core_fw_status(core_dev, true);
}

static int init_vpu_core_dev(struct core_device *core_dev)
{
	int ret = 0;

	if (!core_dev)
		return -EINVAL;

	mutex_init(&core_dev->cmd_mutex);
	init_completion(&core_dev->boot_cmp);
	init_completion(&core_dev->snap_done_cmp);

	core_dev->workqueue = alloc_workqueue("vpu",
					WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!core_dev->workqueue) {
		vpu_err("%s unable to alloc workqueue\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	INIT_WORK(&core_dev->msg_work, vpu_enc_msg_run_work);

	ret = kfifo_alloc(&core_dev->mu_msg_fifo,
			  sizeof(u32) * VID_API_NUM_STREAMS * VID_API_MESSAGE_LIMIT,
			  GFP_KERNEL);
	if (ret) {
		vpu_err("error: fail to alloc mu msg fifo\n");
		goto err_des_work;
	}

	//firmware space for M0
	core_dev->m0_p_fw_space_vir =
		ioremap_wc(core_dev->m0_p_fw_space_phy, core_dev->fw_buf_size);
	if (!core_dev->m0_p_fw_space_vir) {
		vpu_err("failed to remap space for M0 firmware\n");
		ret = -EINVAL;
		goto err_free_fifo;
	}

	core_dev->m0_rpc_virt =
		ioremap_wc(core_dev->m0_rpc_phy,
			core_dev->rpc_buf_size + core_dev->print_buf_size);
	if (!core_dev->m0_rpc_virt) {
		vpu_err("failed to remap space for shared memory\n");
		ret = -EINVAL;
		goto err_free_fifo;
	}

	if (is_vpu_enc_poweroff(core_dev))
		vpu_enc_init_core_rpc(core_dev);
	else
		vpu_enc_restore_core_rpc(core_dev);

	init_vpu_attrs(core_dev);

	scnprintf(core_dev->name, sizeof(core_dev->name) - 1,
			"core.%d", core_dev->id);
	sysfs_attr_init(&core_dev->core_attr.attr);
	core_dev->core_attr.attr.name = core_dev->name;
	core_dev->core_attr.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
	core_dev->core_attr.show = show_core_info;
	device_create_file(core_dev->generic_dev, &core_dev->core_attr);

	return ret;

err_free_fifo:
	kfifo_free(&core_dev->mu_msg_fifo);
err_des_work:
	if (core_dev->workqueue) {
		destroy_workqueue(core_dev->workqueue);
		core_dev->workqueue = NULL;
	}
	return ret;
}

static int uninit_vpu_core_dev(struct core_device *core_dev)
{
	if (!core_dev)
		return -EINVAL;

	vpu_enc_mu_free(core_dev);
	kfifo_free(&core_dev->mu_msg_fifo);

	if (core_dev->core_attr.attr.name)
		device_remove_file(core_dev->generic_dev, &core_dev->core_attr);
	release_vpu_attrs(core_dev);
	if (core_dev->workqueue) {
		cancel_work_sync(&core_dev->msg_work);
		destroy_workqueue(core_dev->workqueue);
		core_dev->workqueue = NULL;
	}

	if (core_dev->m0_p_fw_space_vir) {
		iounmap(core_dev->m0_p_fw_space_vir);
		core_dev->m0_p_fw_space_vir = NULL;
	}
	core_dev->m0_p_fw_space_phy = 0;

	if (core_dev->m0_rpc_virt) {
		iounmap(core_dev->m0_rpc_virt);
		core_dev->m0_rpc_virt = NULL;
	}
	core_dev->m0_rpc_phy = 0;

	if (core_dev->mu_base_virtaddr)
		core_dev->mu_base_virtaddr = NULL;

	if (core_dev->generic_dev) {
		put_device(core_dev->generic_dev);
		core_dev->generic_dev = NULL;
	}

	return 0;
}

static void init_vpu_enc_watchdog(struct vpu_dev *vdev)
{
	if (!vdev)
		return;

	INIT_DELAYED_WORK(&vdev->watchdog, vpu_enc_watchdog_handler);
	schedule_delayed_work(&vdev->watchdog,
			msecs_to_jiffies(VPU_WATCHDOG_INTERVAL_MS));
}

static const struct of_device_id vpu_enc_of_match[];
static int vpu_enc_probe(struct platform_device *pdev)
{
	struct vpu_dev *dev;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *dev_id = NULL;
	struct resource *res = NULL;
	u_int32 i;
	int ret;

	if (!np) {
		vpu_err("error: %s of_node is NULL\n", __func__);
		return -EINVAL;
	}

	dev_id = of_match_device(vpu_enc_of_match, &pdev->dev);
	if (!dev_id) {
		vpu_err("unmatch vpu encoder device\n");
		return -EINVAL;
	}
	vpu_dbg(LVL_INFO, "probe %s\n", dev_id->compatible);

	if (vpu_enc_sc_check_fuse())
		return -EINVAL;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->plat_type = *(enum PLAT_TYPE *)dev_id->data;
	dev->plat_dev = pdev;
	dev->generic_dev = get_device(&pdev->dev);

	ret = vpu_enc_attach_pm_domains(dev);
	if (ret)
		goto error_put_dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		ret = -EINVAL;
		goto error_det_pm;
	}

	dev->reg_vpu_base = res->start;
	dev->reg_vpu_size = resource_size(res);

	ret = parse_dt_info(dev, np);
	if (ret) {
		vpu_err("parse device tree fail\n");
		goto error_det_pm;
	}

	dev->regs_base = ioremap(dev->reg_vpu_base, dev->reg_vpu_size);
	if (!dev->regs_base) {
		vpu_err("%s could not map regs_base\n", __func__);
		ret = PTR_ERR(dev->regs_base);
		goto error_det_pm;
	}

	ret = vpu_enc_init_reserved_memory(&dev->reserved_mem);
	if (ret) {
		vpu_err("%s couldn't init reserved memory\n", __func__);
		goto error_iounmap;
	}

	platform_set_drvdata(pdev, dev);
	mutex_init(&dev->dev_mutex);
	for (i = 0; i < dev->core_num; i++) {
		dev->core_dev[i].id = i;
		dev->core_dev[i].generic_dev = get_device(dev->generic_dev);
		dev->core_dev[i].vdev = dev;
	}

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		vpu_err("fail to request mailbox, ret = %d\n", ret);
		pm_runtime_put_noidle(&pdev->dev);
		pm_runtime_set_suspended(&pdev->dev);
		goto error_pm_runtime_get_sync;
	}

	mutex_init(&dev->dev_mutex);
	for (i = 0; i < dev->core_num; i++) {
		ret = init_vpu_core_dev(&dev->core_dev[i]);
		if (ret)
			break;
	}

	if (i < dev->core_num) {
		if (i == 0)
			goto error_init_core;
		else
			vpu_err("error: core[%d] init failed\n", i);
	}

	dev->core_num = i;
	vpu_enc_enable_hw(dev);

	pm_runtime_put_sync(&pdev->dev);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		vpu_err("%s unable to register v4l2 dev\n", __func__);
		goto error_init_core;
	}

	ret = create_vpu_video_device(dev);
	if (ret) {
		vpu_err("create vpu video device fail\n");
		goto error_unreg_v4l2;
	}


	device_create_file(&pdev->dev, &dev_attr_meminfo);
	device_create_file(&pdev->dev, &dev_attr_buffer);
	device_create_file(&pdev->dev, &dev_attr_fpsinfo);
	device_create_file(&pdev->dev, &dev_attr_vpuinfo);
	vpu_enc_create_debugfs_file(dev);
	init_vpu_enc_watchdog(dev);
	vpu_dbg(LVL_INFO, "VPU Encoder registered\n");

	return 0;

error_unreg_v4l2:
	v4l2_device_unregister(&dev->v4l2_dev);
error_init_core:
	for (i = 0; i < dev->core_num; i++)
		uninit_vpu_core_dev(&dev->core_dev[i]);

	vpu_enc_disable_hw(dev);
	pm_runtime_put_sync(&pdev->dev);
error_pm_runtime_get_sync:
	pm_runtime_disable(&pdev->dev);

	if (dev->pvpu_encoder_dev) {
		video_unregister_device(dev->pvpu_encoder_dev);
		dev->pvpu_encoder_dev = NULL;
	}
	vpu_enc_release_reserved_memory(&dev->reserved_mem);
error_iounmap:
	if (dev->regs_base) {
		iounmap(dev->regs_base);
		dev->regs_base = NULL;
	}
error_det_pm:
	vpu_enc_detach_pm_domains(dev);
error_put_dev:
	if (dev->generic_dev) {
		put_device(dev->generic_dev);
		dev->generic_dev = NULL;
	}

	return ret;
}

static int vpu_enc_remove(struct platform_device *pdev)
{
	struct vpu_dev *dev = platform_get_drvdata(pdev);
	u_int32 i;

	cancel_delayed_work_sync(&dev->watchdog);
	vpu_enc_remove_debugfs_file(dev);
	device_remove_file(&pdev->dev, &dev_attr_vpuinfo);
	device_remove_file(&pdev->dev, &dev_attr_fpsinfo);
	device_remove_file(&pdev->dev, &dev_attr_buffer);
	device_remove_file(&pdev->dev, &dev_attr_meminfo);

	pm_runtime_get_sync(&pdev->dev);
	for (i = 0; i < dev->core_num; i++)
		uninit_vpu_core_dev(&dev->core_dev[i]);

	vpu_enc_disable_hw(dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	if (video_get_drvdata(dev->pvpu_encoder_dev))
		video_unregister_device(dev->pvpu_encoder_dev);

	v4l2_device_unregister(&dev->v4l2_dev);
	vpu_enc_detach_pm_domains(dev);
	vpu_enc_release_reserved_memory(&dev->reserved_mem);
	if (dev->regs_base) {
		iounmap(dev->regs_base);
		dev->regs_base = NULL;
	}
	if (dev->generic_dev) {
		put_device(dev->generic_dev);
		dev->generic_dev = NULL;
	}

	return 0;
}

static int vpu_enc_runtime_suspend(struct device *dev)
{
	int i;
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);

	for (i = 0; i < vpudev->core_num; i++) {
		if (!vpudev->core_dev[i].generic_dev)
			continue;
		vpu_enc_mu_free(&vpudev->core_dev[i]);
	}

	return 0;
}

static int vpu_enc_runtime_resume(struct device *dev)
{
	int i;
	int ret = 0;

	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);

	for (i = 0; i < vpudev->core_num; i++) {
		if (!vpudev->core_dev[i].generic_dev)
			continue;
		ret |= vpu_enc_mu_request(&vpudev->core_dev[i]);
	}

	return ret;
}

static int is_vpu_enc_poweroff(struct core_device *core)
{
	/* the csr register 'CM0Px_CPUWAIT' will be cleared to '1' after
	 * reset(poweoff then poweron)
	 */
	if (read_vpu_reg(core->vdev, core->reg_csr_base + CSR_CM0Px_CPUWAIT) == 1)
		return 1;
	else
		return 0;
}

static int is_need_shapshot(struct vpu_ctx *ctx)
{
	if (!test_bit(VPU_ENC_STATUS_INITIALIZED, &ctx->status))
		return 0;
	if (!test_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status))
		return 0;
	if (test_bit(VPU_ENC_STATUS_CLOSED, &ctx->status))
		return 0;
	if (test_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status))
		return 0;
	if (test_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status))
		return 0;

	return 1;
}

static int vpu_enc_snapshot(struct vpu_ctx *ctx)
{
	int ret;

	if (!ctx)
		return -EINVAL;

	vpu_dbg(LVL_INFO, "core[%d] snapshot\n", ctx->core_dev->id);
	vpu_ctx_send_cmd(ctx, GTB_ENC_CMD_SNAPSHOT, 0, NULL);
	ret = wait_for_completion_timeout(&ctx->core_dev->snap_done_cmp,
						msecs_to_jiffies(1000));
	if (!ret)
		vpu_err("error:wait for snapdone event timeout!\n");
	else
		ctx->core_dev->snapshot = true;

	return 0;
}

static int resume_from_snapshot(struct core_device *core)
{
	int ret = 0;

	if (!core)
		return -EINVAL;
	if (!core->snapshot)
		return 0;

	vpu_dbg(LVL_INFO, "core[%d] resume from snapshot\n", core->id);

	reinit_completion(&core->boot_cmp);
	set_vpu_fw_addr(core->vdev, core);
	ret = wait_for_boot_done(core, 1);
	if (ret) {
		set_core_force_release(core);
		reset_vpu_core_dev(core);
		return -EINVAL;
	}

	return 0;
}

static int re_download_firmware(struct core_device *core)
{
	if (!core)
		return -EINVAL;

	vpu_dbg(LVL_INFO, "re download firmware for core[%d]\n", core->id);

	reset_vpu_core_dev(core);
	return download_vpu_firmware(core->vdev, core);
}

static int suspend_instance(struct vpu_ctx *ctx)
{
	int ret = 0;

	if (!ctx)
		return 0;

	if (test_bit(VPU_ENC_STATUS_STOP_REQ, &ctx->status) ||
		test_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status))
		wait_for_stop_done(ctx);

	mutex_lock(&ctx->instance_mutex);
	if (!ctx->core_dev->snapshot && is_need_shapshot(ctx))
		ret = vpu_enc_snapshot(ctx);
	set_bit(VPU_ENC_STATUS_SNAPSHOT, &ctx->status);
	mutex_unlock(&ctx->instance_mutex);

	return ret;
}

static int resume_instance(struct vpu_ctx *ctx)
{
	if (!ctx)
		return 0;

	clear_bit(VPU_ENC_STATUS_SNAPSHOT, &ctx->status);

	return 0;
}

static int suspend_core(struct core_device *core)
{
	int i;
	int ret = 0;

	WARN_ON(!core);

	core->snapshot = false;

	if (!core->fw_is_ready)
		return 0;

	for (i = 0; i < core->supported_instance_count; i++) {
		if (!core->ctx[i])
			continue;
		ret = suspend_instance(core->ctx[i]);
		if (ret)
			return ret;
	}

	for (i = 0; i < core->supported_instance_count; i++)
		vpu_ctx_power_off(core->ctx[i]);

	core->suspend = true;

	return 0;
}

static int resume_core(struct core_device *core)
{
	int ret = 0;
	u32 instance_count = 0;
	int i;

	WARN_ON(!core);

	if (!core->suspend)
		return 0;

	for (i = 0; i < core->supported_instance_count; i++) {
		if (!core->ctx[i])
			continue;
		instance_count++;
		vpu_ctx_power_on(core->ctx[i]);
		resume_instance(core->ctx[i]);
	}

	if (is_vpu_enc_poweroff(core)) {
		if (!core->vdev->hw_enable)
			vpu_enc_enable_hw(core->vdev);
		if (core->snapshot)
			ret = resume_from_snapshot(core);
		else if (instance_count)
			ret = re_download_firmware(core);
		else
			reset_vpu_core_dev(core);
	} else {
		if (core->snapshot || instance_count)
			ret = sw_reset_firmware(core, 1);
	}

	core->snapshot = false;
	core->suspend = false;

	return ret;
}

static void vpu_enc_cancel_work(struct vpu_dev *vpudev)
{
	int i;
	int j;

	for (i = 0; i < vpudev->core_num; i++) {
		struct core_device *core = &vpudev->core_dev[i];

		if (!core->fw_is_ready)
			continue;
		cancel_work_sync(&core->msg_work);
		for (j = 0; j < core->supported_instance_count; j++) {
			struct vpu_ctx *ctx = core->ctx[j];

			if (!ctx)
				continue;
			cancel_work_sync(&ctx->instance_work);
		}
	}
	cancel_delayed_work_sync(&vpudev->watchdog);
}

static void vpu_enc_resume_work(struct vpu_dev *vpudev)
{
	int i;
	int j;

	for (i = 0; i < vpudev->core_num; i++) {
		struct core_device *core = &vpudev->core_dev[i];

		if (!core->fw_is_ready)
			continue;
		queue_work(core->workqueue, &core->msg_work);
		for (j = 0; j < core->supported_instance_count; j++) {
			struct vpu_ctx *ctx = core->ctx[j];

			if (!ctx)
				continue;
			queue_work(ctx->instance_wq, &ctx->instance_work);
		}
	}
	schedule_delayed_work(&vpudev->watchdog,
			msecs_to_jiffies(VPU_WATCHDOG_INTERVAL_MS));
}

static int __maybe_unused vpu_enc_suspend(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);
	int i;
	int ret = 0;

	vpu_dbg(LVL_INFO, "suspend\n");

	mutex_lock(&vpudev->dev_mutex);
	pm_runtime_get_sync(dev);
	for (i = 0; i < vpudev->core_num; i++) {
		ret = suspend_core(&vpudev->core_dev[i]);
		if (ret)
			break;
	}
	vpu_enc_cancel_work(vpudev);
	pm_runtime_put_sync(dev);
	mutex_unlock(&vpudev->dev_mutex);

	vpu_dbg(LVL_INFO, "suspend done\n");

	return ret;
}

static int __maybe_unused vpu_enc_resume(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);
	int i;
	int ret = 0;

	vpu_dbg(LVL_INFO, "resume\n");

	mutex_lock(&vpudev->dev_mutex);
	pm_runtime_get_sync(dev);
	vpudev->hw_enable = false;
	for (i = 0; i < vpudev->core_num; i++) {
		ret = resume_core(&vpudev->core_dev[i]);
		if (ret)
			break;
	}
	vpudev->hw_enable = true;
	vpu_enc_resume_work(vpudev);
	pm_runtime_put_sync(dev);
	mutex_unlock(&vpudev->dev_mutex);

	vpu_dbg(LVL_INFO, "resume done\n");

	return ret;
}

static const struct dev_pm_ops vpu_enc_pm_ops = {
	SET_RUNTIME_PM_OPS(vpu_enc_runtime_suspend, vpu_enc_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(vpu_enc_suspend, vpu_enc_resume)
};

static enum PLAT_TYPE supported_plat_types[PLAT_TYPE_RESERVED] = {
	[IMX8QXP] = IMX8QXP,
	[IMX8QM] = IMX8QM,
};

static const struct of_device_id vpu_enc_of_match[] = {
	{ .compatible = "nxp,imx8qm-b0-vpuenc",
	  .data = (void *)&supported_plat_types[IMX8QM]
	},
	{ .compatible = "nxp,imx8qxp-b0-vpuenc",
	  .data = (void *)&supported_plat_types[IMX8QXP]
	},
	{}
};
MODULE_DEVICE_TABLE(of, vpu_enc_of_match);

static struct platform_driver vpu_enc_driver = {
	.probe = vpu_enc_probe,
	.remove = vpu_enc_remove,
	.driver = {
		.name = "vpu-b0-encoder",
		.of_match_table = vpu_enc_of_match,
		.pm = &vpu_enc_pm_ops,
	},
};
module_platform_driver(vpu_enc_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux VPU driver for Freescale i.MX/MXC");
MODULE_LICENSE("GPL");
MODULE_VERSION(VPU_ENC_DRIVER_VERSION);

module_param(vpu_dbg_level_encoder, int, 0644);
MODULE_PARM_DESC(vpu_dbg_level_encoder, "Debug level (0-4)");

module_param(reset_on_hang, int, 0644);
MODULE_PARM_DESC(reset_on_hang, "reset on hang (0-1)");

module_param(show_detail_index, int, 0644);
MODULE_PARM_DESC(show_detail_index, "show memory detail info index");

module_param(debug_firmware_bitmap, long, 0644);
MODULE_PARM_DESC(debug_firmware_bitmap, "firmware debug info switch");

