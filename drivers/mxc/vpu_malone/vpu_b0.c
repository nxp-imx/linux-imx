/*
 * Copyright 2018 NXP
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
 * @file vpu-b0.c
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
#include <linux/slab.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>
#include <linux/mx8_mu.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-dma-sg.h>

#include "vpu_b0.h"
#include "insert_startcode.h"
#include "vpu_debug_log.h"
#include "vpu_ts.h"

unsigned int vpu_dbg_level_decoder = LVL_WARN;
static int vpu_frm_depth = INVALID_FRAME_DEPTH;
static int vpu_log_depth = DEFAULT_LOG_DEPTH;
static int vpu_max_bufsize = MAX_BUFFER_SIZE;
static int vpu_frmdbg_ena = DEFAULT_FRMDBG_ENABLE;
static int vpu_frmdbg_level = DEFAULT_FRMDBG_LEVEL;
static int vpu_dbe_num = 1;
static int vpu_frmcrcdump_ena;
static int stream_buffer_threshold;
static int tsm_mode = MODE_AI;
static int tsm_buffer_size = 1024;
static int tsm_use_consumed_length;

/* Generic End of content startcodes to differentiate from those naturally in the stream/file */
#define EOS_GENERIC_HEVC 0x7c010000
#define EOS_GENERIC_JPEG 0xefff0000
#define EOS_GENERIC_MPEG 0xCC010000
static void vpu_api_event_handler(struct vpu_ctx *ctx, u_int32 uStrIdx, u_int32 uEvent, u_int32 *event_data);
static void v4l2_vpu_send_cmd(struct vpu_ctx *ctx, uint32_t idx, uint32_t cmdid, uint32_t cmdnum, uint32_t *local_cmddata);
static int add_scode(struct vpu_ctx *ctx, u_int32 uStrBufIdx, VPU_PADDING_SCODE_TYPE eScodeType, bool bUpdateWr);
static void v4l2_update_stream_addr(struct vpu_ctx *ctx, uint32_t uStrBufIdx);
static int swreset_vpu_firmware(struct vpu_dev *dev, u_int32 idx);
static int find_first_available_instance(struct vpu_dev *dev);
static int remove_instance_file(struct vpu_ctx *ctx);
static void fill_stream_buffer_info(struct vpu_ctx *ctx);
static void set_pic_end_flag(struct vpu_ctx *ctx);
static void clear_pic_end_flag(struct vpu_ctx *ctx);
static void send_skip_event(struct vpu_ctx* ctx);
static void reset_mbi_dcp_count(struct vpu_ctx *ctx);
static bool verify_frame_buffer_size(struct queue_data *q_data,
							struct vb2_data_req *p_data_req);
#define CHECK_BIT(var, pos) (((var) >> (pos)) & 1)

static char *cmd2str[] = {
	"VID_API_CMD_NULL",   /*0x0*/
	"VID_API_CMD_PARSE_NEXT_SEQ", /*0x1*/
	"VID_API_CMD_PARSE_NEXT_I",
	"VID_API_CMD_PARSE_NEXT_IP",
	"VID_API_CMD_PARSE_NEXT_ANY",
	"VID_API_CMD_DEC_PIC",
	"VID_API_CMD_UPDATE_ES_WR_PTR",
	"VID_API_CMD_UPDATE_ES_RD_PTR",
	"VID_API_CMD_UPDATE_UDATA",
	"VID_API_CMD_GET_FSINFO",
	"VID_API_CMD_SKIP_PIC",
	"VID_API_CMD_DEC_CHUNK",  /*0x0b*/
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_START",         /*0x10*/
	"VID_API_CMD_STOP",
	"VID_API_CMD_ABORT",
	"VID_API_CMD_RST_BUF",
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_FS_RELEASE",
	"VID_API_CMD_MEM_REGION_ATTACH",
	"VID_API_CMD_MEM_REGION_DETACH",
	"VID_API_CMD_MVC_VIEW_SELECT",
	"VID_API_CMD_FS_ALLOC",   /*0x19*/
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_DBG_GET_STATUS", /*0x1C*/
	"VID_API_CMD_DBG_START_LOG",
	"VID_API_CMD_DBG_STOP_LOG",
	"VID_API_CMD_DBG_DUMP_LOG",
	"VID_API_CMD_YUV_READY",   /*0x20*/
};

static char *event2str[] = {
	"VID_API_EVENT_NULL",  /*0x0*/
	"VID_API_EVENT_RESET_DONE",  /*0x1*/
	"VID_API_EVENT_SEQ_HDR_FOUND",
	"VID_API_EVENT_PIC_HDR_FOUND",
	"VID_API_EVENT_PIC_DECODED",
	"VID_API_EVENT_FIFO_LOW",
	"VID_API_EVENT_FIFO_HIGH",
	"VID_API_EVENT_FIFO_EMPTY",
	"VID_API_EVENT_FIFO_FULL",
	"VID_API_EVENT_BS_ERROR",
	"VID_API_EVENT_UDATA_FIFO_UPTD",
	"VID_API_EVENT_RES_CHANGE",
	"VID_API_EVENT_FIFO_OVF",
	"VID_API_EVENT_CHUNK_DECODED",  /*0x0D*/
	"VID_API_EVENT_UNDEFINED",
	"VID_API_EVENT_UNDEFINED",
	"VID_API_EVENT_REQ_FRAME_BUFF",  /*0x10*/
	"VID_API_EVENT_FRAME_BUFF_RDY",
	"VID_API_EVENT_REL_FRAME_BUFF",
	"VID_API_EVENT_STR_BUF_RST",
	"VID_API_EVENT_RET_PING",
	"VID_API_EVENT_QMETER",
	"VID_API_EVENT_STR_FMT_CHANGED",
	"VID_API_EVENT_FIRMWARE_XCPT",
	"VID_API_EVENT_START_DONE",
	"VID_API_EVENT_STOPPED",
	"VID_API_EVENT_ABORT_DONE",
	"VID_API_EVENT_FINISHED",
	"VID_API_EVENT_DBG_STAT_UPDATE",
	"VID_API_EVENT_DBG_LOG_STARTED",
	"VID_API_EVENT_DBG_LOG_STOPPED",
	"VID_API_EVENT_DBG_LOG_UPFATED",
	"VID_API_EVENT_DBG_MSG_DEC",  /*0x20*/
	"VID_API_EVENT_DEC_SC_ERR",
	"VID_API_EVENT_CQ_FIFO_DUMP",
	"VID_API_EVENT_DBG_FIFO_DUMP",
	"VID_API_EVENT_DEC_CHECK_RES",
	"VID_API_EVENT_DEC_CFG_INFO",  /*0x25*/
};

static char *bufstat[] = {
	"FRAME_ALLOC",
	"FRAME_FREE",
	"FRAME_DECODED",
	"FRAME_READY",
	"FRAME_RELEASE",
};

static int alloc_vpu_buffer(struct vpu_ctx *ctx);
static bool vpu_dec_is_active(struct vpu_ctx *ctx);

static char *get_event_str(u32 event)
{
	if (event == VID_API_EVENT_SNAPSHOT_DONE)
		return "VID_API_EVENT_SNAPSHOT_DONE";
	else if (event >= ARRAY_SIZE(event2str))
		return "UNKNOWN EVENT";
	return event2str[event];
}

static char *get_cmd_str(u32 cmdid)
{
	if (cmdid == VID_API_CMD_FIRM_RESET)
		return "VID_API_CMD_FIRM_RESET";
	else if (cmdid == VID_API_CMD_SNAPSHOT)
		return "VID_API_CMD_SNAPSHOT";
	else if (cmdid >= ARRAY_SIZE(cmd2str))
		return "UNKNOWN CMD";
	return cmd2str[cmdid];
}

static void vpu_log_event(u_int32 uEvent, u_int32 ctxid)
{
	if (uEvent > ARRAY_SIZE(event2str)-1)
		vpu_dbg(LVL_BIT_EVT, "reveive event: 0x%X, ctx id:%d\n",
				uEvent, ctxid);
	else
		vpu_dbg(LVL_BIT_EVT, "recevie event: %s, ctx id:%d\n",
				event2str[uEvent], ctxid);
}

static void vpu_log_cmd(u_int32 cmdid, u_int32 ctxid)
{
	if (cmdid > ARRAY_SIZE(cmd2str)-1)
		vpu_dbg(LVL_BIT_CMD, "send cmd: 0x%X, ctx id:%d\n",
				cmdid, ctxid);
	else
		vpu_dbg(LVL_BIT_CMD, "send cmd: %s ctx id:%d\n",
				cmd2str[cmdid], ctxid);
}

static void vpu_log_buffer_state(struct vpu_ctx *ctx)
{
	struct vb2_data_req *p_data_req;
	struct queue_data *This;
	int i;

	if (!ctx)
		return;

	This = &ctx->q_data[V4L2_DST];
	down(&This->drv_q_lock);
	for (i = 0; i < VPU_MAX_BUFFER; i++) {
		p_data_req = &This->vb2_reqs[i];
		if (p_data_req->vb2_buf != NULL)
			vpu_dbg(LVL_BIT_BUFFER_STAT,
				"ctx: %d, buffer[%d] status: %s\n",
				ctx->str_index, i, bufstat[p_data_req->status]);
	}
	up(&This->drv_q_lock);
}

static void count_event(struct vpu_statistic *statistic, u32 event)
{
	if (!statistic)
		return;

	if (event < ARRAY_SIZE(event2str))
		statistic->event[event]++;
	else
		statistic->event[VID_API_EVENT_DEC_CFG_INFO + 1]++;

	statistic->current_event = event;
	getrawmonotonic(&statistic->ts_event);
}

static void count_cmd(struct vpu_statistic *statistic, u32 cmdid)
{
	if (!statistic)
		return;

	if (cmdid < ARRAY_SIZE(cmd2str))
		statistic->cmd[cmdid]++;
	else
		statistic->cmd[VID_API_CMD_YUV_READY + 1]++;
	statistic->current_cmd = cmdid;
	getrawmonotonic(&statistic->ts_cmd);
}

static u32 get_greatest_common_divisor(u32 a, u32 b)
{
	u32 tmp;

	if (!a)
		return b;

	while (b) {
		tmp = a % b;
		a = b;
		b = tmp;
	}

	return a;
}

static int find_buffer_id(struct vpu_ctx *ctx, u_int32 addr)
{
	struct queue_data *This;
	struct vb2_data_req *p_data_req;
	u_int32 LumaAddr;
	u_int32 *pphy_address;
	u_int32 i;

	if (!ctx)
		return -1;

	This = &ctx->q_data[V4L2_DST];
	down(&This->drv_q_lock);
	for (i = 0; i < VPU_MAX_BUFFER; i++) {
		p_data_req = &This->vb2_reqs[i];
		if (p_data_req->vb2_buf != NULL) {
			pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 0);
			if (pphy_address != NULL) {
				LumaAddr = *pphy_address + p_data_req->data_offset[0];
				if (LumaAddr == addr) {
					up(&This->drv_q_lock);
					return i;
				}
			} else {
				vpu_dbg(LVL_INFO, "error: %s() ctx[%d] buffer (%d) is NULL\n",
						__func__, ctx->str_index, i);
			}
		}
	}
	up(&This->drv_q_lock);
	vpu_dbg(LVL_ERR, "error: %s() ctx[%d] can't find suitable id based on address(0x%x)\n",
			__func__, ctx->str_index, addr);
	return -1;
}

static void MU_sendMesgToFW(void __iomem *base, MSG_Type type, uint32_t value)
{
	MU_SendMessage(base, 1, value);
	MU_SendMessage(base, 0, type);
}

static u32 get_str_buffer_desc_offset(struct vpu_ctx *ctx)
{
	return DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * ctx->str_index;
}

static pSTREAM_BUFFER_DESCRIPTOR_TYPE get_str_buffer_desc(struct vpu_ctx *ctx)
{
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;

	WARN_ON(!ctx || !ctx->dev);
	pStrBufDesc = ctx->dev->regs_base + get_str_buffer_desc_offset(ctx);
	return pStrBufDesc;
}

static void set_data_req_status(struct vb2_data_req *p_data_req,
				FRAME_BUFFER_STAT status)
{
	vpu_dbg(LVL_BIT_BUFFER_STAT, "Buffer Status [%2d] : %s -> %s\n",
			p_data_req->id,
			bufstat[p_data_req->status],
			bufstat[status]);
	p_data_req->status = status;
}

static u32 vpu_dec_cpu_phy_to_mu(struct vpu_dev *dev, u32 addr)
{
	return addr - dev->m0_p_fw_space_phy;
}

#ifdef DEBUG
static void vpu_log_shared_mem(struct vpu_ctx *ctx)
{
	struct vpu_dev *dev = ctx->dev;
	struct shared_addr *This = &dev->shared_mem;
	pDEC_RPC_HOST_IFACE pSharedInterface = (pDEC_RPC_HOST_IFACE)This->shared_mem_vir;
	MediaIPFW_Video_BufDesc *pMsgDesc = &pSharedInterface->StreamMsgBufferDesc;
	MediaIPFW_Video_BufDesc *pCmdDesc = &pSharedInterface->StreamCmdBufferDesc;
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	u_int32 index = ctx->str_index;

	vpu_dbg(LVL_INFO, "msg: wr: 0x%x, rd: 0x%x, cmd: wr : 0x%x, rd: 0x%x\n",
			pMsgDesc->uWrPtr, pMsgDesc->uRdPtr, pCmdDesc->uWrPtr, pCmdDesc->uRdPtr);

	pStrBufDesc = get_str_buffer_desc(ctx);
	vpu_dbg(LVL_INFO, "data: wptr(0x%x) rptr(0x%x) start(0x%x) end(0x%x) uStrIdx(%d)\n",
			pStrBufDesc->wptr, pStrBufDesc->rptr, pStrBufDesc->start, pStrBufDesc->end, index);
}
#endif
/*
 * v4l2 ioctl() operation
 *
 */
static struct vpu_v4l2_fmt  formats_compressed_dec[] = {
	{
		.name       = "H264 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_AVC,
		.disable    = 0,
	},
	{
		.name       = "VC1 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_VC1_ANNEX_G,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_VC1,
		.disable    = 0,
	},
	{
		.name       = "VC1 RCV Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_VC1_ANNEX_L,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_VC1,
		.disable    = 0,
	},
	{
		.name       = "MPEG2 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_MPEG2,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_MPEG2,
		.disable    = 0,
	},

	{
		.name       = "AVS Encoded Stream",
		.fourcc     = VPU_PIX_FMT_AVS,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_AVS,
		.disable    = 0,
	},
	{
		.name       = "MPEG4 ASP Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_MPEG4,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_ASP,
		.disable    = 0,
	},
	{
		.name       = "DIVX Encoded Stream",
		.fourcc     = VPU_PIX_FMT_DIVX,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_ASP,
		.disable    = 0,
	},
	{
		.name       = "JPEG stills",
		.fourcc     = V4L2_PIX_FMT_JPEG,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_JPEG,
		.disable    = 0,
	},
	{
		.name       = "RV Encoded Stream",
		.fourcc     = VPU_PIX_FMT_RV,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_RV,
		.disable    = 0,
	},
	{
		.name       = "VP6 Encoded Stream",
		.fourcc     = VPU_PIX_FMT_VP6,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_VP6,
		.disable    = 0,
	},
	{
		.name       = "SPK Encoded Stream",
		.fourcc     = VPU_PIX_FMT_SPK,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_SPK,
		.disable    = 0,
	},
	{
		.name       = "H263 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_H263,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_ASP,
		.disable    = 0,
	},
	{
		.name       = "VP8 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_VP8,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_VP8,
		.disable    = 0,
	},
	{
		.name       = "H264/MVC Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_H264_MVC,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_AVC_MVC,
		.disable    = 0,
	},
	{
		.name       = "H265 HEVC Encoded Stream",
		.fourcc     = VPU_PIX_FMT_HEVC,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_HEVC,
		.disable    = 0,
	},
	{
		.name       = "Logo",
		.fourcc     = VPU_PIX_FMT_LOGO,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_UNDEFINED,
		.disable    = 0,
	},
};

static struct vpu_v4l2_fmt  formats_yuv_dec[] = {
	{
		.name       = "4:2:0 2 Planes Y/CbCr",
		.fourcc     = V4L2_PIX_FMT_NV12,
		.num_planes	= 2,
		.vdec_std   = VPU_PF_YUV420_SEMIPLANAR,
		.disable    = 0,
	},
	{
		.name       = "4:2:0 2 Planes Y/CbCr",
		.fourcc     = V4L2_PIX_FMT_NV12_10BIT,
		.num_planes = 2,
		.vdec_std   = VPU_PF_YUV420_SEMIPLANAR,
		.disable    = 0,
	},
};

static int v4l2_ioctl_querycap(struct file *file,
		void *fh,
		struct v4l2_capability *cap
		)
{
	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);
	strlcpy(cap->driver, "vpu B0", sizeof(cap->driver));
	strlcpy(cap->card, "vpu B0", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:", sizeof(cap->bus_info));
	cap->version = KERNEL_VERSION(0, 0, 1);
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int v4l2_ioctl_enum_fmt_vid_cap_mplane(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f
		)
{
	struct vpu_v4l2_fmt *fmt;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);
	if (f->index >= ARRAY_SIZE(formats_yuv_dec))
		return -EINVAL;

	fmt = &formats_yuv_dec[f->index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}
static int v4l2_ioctl_enum_fmt_vid_out_mplane(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f
		)
{
	struct vpu_v4l2_fmt *fmt;
	u_int32 index = 0, i;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (f->index >= ARRAY_SIZE(formats_compressed_dec))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(formats_compressed_dec); i++) {
		fmt = &formats_compressed_dec[i];
		if (fmt->disable == 1)
			continue;
		if (f->index == index)
			break;
		index++;
	}

	if (i == ARRAY_SIZE(formats_compressed_dec))
		return -EINVAL;

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	f->flags |= V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static bool is_10bit_format(struct vpu_ctx *ctx)
{
	WARN_ON(!ctx || !ctx->pSeqinfo);
	if (ctx->pSeqinfo->uBitDepthLuma > 8)
		return true;
	if (ctx->pSeqinfo->uBitDepthChroma > 8)
		return true;
	return false;
}

static void caculate_frame_size(struct vpu_ctx *ctx)
{
	u_int32 width = ctx->pSeqinfo->uHorDecodeRes;
	u_int32 height = ctx->pSeqinfo->uVerDecodeRes;
	u_int32 luma_size;
	u_int32 chroma_size;
	u_int32 chroma_height;
	u_int32 uVertAlign = 512-1;
	bool b10BitFormat = is_10bit_format(ctx);

	struct queue_data *q_data;

	q_data = &ctx->q_data[V4L2_DST];

	width = b10BitFormat?(width + ((width + 3) >> 2)):width;
	width = ((width + uVertAlign) & ~uVertAlign);
	q_data->stride = width;

	height = ((height + uVertAlign) & ~uVertAlign);
	if (ctx->pSeqinfo->uProgressive)
		chroma_height = height >> 1;
	else
		chroma_height = height;
	luma_size = width * height;
	chroma_size = width * chroma_height;
	ctx->q_data[V4L2_DST].sizeimage[0] = luma_size;
	ctx->q_data[V4L2_DST].sizeimage[1] = chroma_size;
}

static int v4l2_ioctl_g_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f
		)
{
	struct vpu_ctx *ctx =           v4l2_fh_to_ctx(fh);
	struct v4l2_pix_format_mplane   *pix_mp = &f->fmt.pix_mp;
	unsigned int i;
	struct queue_data               *q_data;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		q_data = &ctx->q_data[V4L2_DST];
		if (is_10bit_format(ctx))
			pix_mp->pixelformat = V4L2_PIX_FMT_NV12_10BIT;
		else
			pix_mp->pixelformat = V4L2_PIX_FMT_NV12;
		pix_mp->width = ctx->pSeqinfo->uHorRes > 0?ctx->pSeqinfo->uHorRes:q_data->width;
		pix_mp->height = ctx->pSeqinfo->uVerRes > 0?ctx->pSeqinfo->uVerRes:q_data->height;
		if (ctx->pSeqinfo->uProgressive == 1)
			pix_mp->field = V4L2_FIELD_NONE;
		else
			pix_mp->field = V4L2_FIELD_INTERLACED;
		pix_mp->num_planes = 2;
		pix_mp->colorspace = V4L2_COLORSPACE_REC709;

		for (i = 0; i < pix_mp->num_planes; i++) {
			pix_mp->plane_fmt[i].bytesperline = q_data->stride;
			pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];
		}
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		q_data = &ctx->q_data[V4L2_SRC];
		pix_mp->width = q_data->width;
		pix_mp->height = q_data->height;
		pix_mp->field = V4L2_FIELD_NONE;
		pix_mp->num_planes = q_data->num_planes;
		for (i = 0; i < pix_mp->num_planes; i++) {
			pix_mp->plane_fmt[i].bytesperline = q_data->stride;
			pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];

		}
		pix_mp->pixelformat = q_data->fourcc;
	} else
		return -EINVAL;
	return 0;
}

static bool set_video_standard(struct queue_data *q_data,
		struct v4l2_format *f,
		struct vpu_v4l2_fmt *pformat_table,
		uint32_t table_size)
{
	unsigned int i;

	for (i = 0; i < table_size; i++) {
		if (pformat_table[i].fourcc == f->fmt.pix_mp.pixelformat) {
			if (pformat_table[i].disable == 1)
				return false;
			q_data->vdec_std = pformat_table[i].vdec_std;
		}
	}
	return true;
}

#define VPU_DISABLE_BITS (0x7)
void check_fuse_value(struct vpu_dev *dev,
		struct vpu_v4l2_fmt *pformat_table,
		uint32_t table_size)
{
	u_int32 fuse;
	u_int32 val;
	u_int32 i;

	if (!dev)
		return;
	sc_misc_otp_fuse_read(dev->mu_ipcHandle, VPU_DISABLE_BITS, &fuse);
	val = (fuse >> 2) & 0x3UL;
	if (val == 0x1UL) {
		for (i = 0; i < table_size; i++)
			if (pformat_table[i].fourcc == VPU_PIX_FMT_HEVC)
				pformat_table[i].disable = 1;
		vpu_dbg(LVL_WARN, "H265 is disabled\n");
	} else if (val == 0x2UL) {
		for (i = 0; i < table_size; i++)
			if (pformat_table[i].fourcc == V4L2_PIX_FMT_H264)
				pformat_table[i].disable = 1;
		vpu_dbg(LVL_WARN, "H264 is disabled\n");
	} else if (val == 0x3UL) {
		for (i = 0; i < table_size; i++)
			pformat_table[i].disable = 1;
		vpu_dbg(LVL_WARN, "All decoder disabled\n");
	}

}

static int v4l2_ioctl_s_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f
		)
{
	struct vpu_ctx                  *ctx = v4l2_fh_to_ctx(fh);
	struct v4l2_pix_format_mplane   *pix_mp = &f->fmt.pix_mp;
	struct queue_data               *q_data;
	u_int32                         i;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		q_data = &ctx->q_data[V4L2_DST];
		if (!set_video_standard(q_data, f, formats_yuv_dec, ARRAY_SIZE(formats_yuv_dec)))
			return -EINVAL;
		pix_mp->num_planes = 2;
		pix_mp->colorspace = V4L2_COLORSPACE_REC709;
		for (i = 0; i < pix_mp->num_planes; i++) {
			if (ctx->q_data[V4L2_DST].stride > 0)
				pix_mp->plane_fmt[i].bytesperline = ctx->q_data[V4L2_DST].stride;
			if (ctx->q_data[V4L2_DST].sizeimage[i] > 0)
				pix_mp->plane_fmt[i].sizeimage = ctx->q_data[V4L2_DST].sizeimage[i];
		}
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		q_data = &ctx->q_data[V4L2_SRC];
		if (!set_video_standard(q_data, f, formats_compressed_dec, ARRAY_SIZE(formats_compressed_dec)))
			return -EINVAL;
	} else
		return -EINVAL;

	q_data->num_planes = pix_mp->num_planes;
	for (i = 0; i < q_data->num_planes; i++) {
		q_data->stride = pix_mp->plane_fmt[i].bytesperline;
		q_data->sizeimage[i] = pix_mp->plane_fmt[i].sizeimage;
	}
	q_data->fourcc = pix_mp->pixelformat;
	q_data->width = pix_mp->width;
	q_data->height = pix_mp->height;
	q_data->rect.left = 0;
	q_data->rect.top = 0;
	q_data->rect.width = pix_mp->width;
	q_data->rect.height = pix_mp->height;

	return 0;
}

static int vpu_dec_queue_expbuf(struct queue_data *queue,
				struct v4l2_exportbuffer *buf)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_expbuf(&queue->vb2_q, buf);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_dec_queue_reqbufs(struct queue_data *queue,
				struct v4l2_requestbuffers *reqbuf)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_reqbufs(&queue->vb2_q, reqbuf);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_dec_queue_querybuf(struct queue_data *queue,
				struct v4l2_buffer *buf)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_querybuf(&queue->vb2_q, buf);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_dec_queue_qbuf(struct queue_data *queue,
				struct v4l2_buffer *buf)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_qbuf(&queue->vb2_q, buf);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_dec_queue_dqbuf(struct queue_data *queue,
				struct v4l2_buffer *buf, bool nonblocking)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_dqbuf(&queue->vb2_q, buf, nonblocking);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_dec_queue_enable(struct queue_data *queue,
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
	struct vb2_data_req *p_data_req = NULL;
	struct vb2_data_req *p_temp;
	struct vb2_buffer *vb;

	vpu_dbg(LVL_BIT_FUNC, "%s() is called\n", __func__);
	list_for_each_entry_safe(p_data_req, p_temp, &queue->drv_q, list) {
		list_del(&p_data_req->list);
		p_data_req->queued = false;
	}

	list_for_each_entry(vb, &queue->vb2_q.queued_list, queued_entry) {
		if (vb->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	}
	INIT_LIST_HEAD(&queue->drv_q);
}

static int vpu_dec_queue_disable(struct queue_data *queue,
				enum v4l2_buf_type type)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_streamoff(&queue->vb2_q, type);
	up(&queue->drv_q_lock);

	return ret;
}

static int vpu_dec_queue_release(struct queue_data *queue)
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

static int vpu_dec_queue_mmap(struct queue_data *queue,
				struct vm_area_struct *vma)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_mmap(&queue->vb2_q, vma);
	up(&queue->drv_q_lock);

	return ret;
}

static int v4l2_ioctl_expbuf(struct file *file,
		void *fh,
		struct v4l2_exportbuffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	return vpu_dec_queue_expbuf(q_data, buf);
}

static int v4l2_ioctl_subscribe_event(struct v4l2_fh *fh,
		const struct v4l2_event_subscription *sub
		)
{
	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SKIP:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return -EINVAL;
	}
}

static void init_dma_buffer(struct dma_buffer *buffer)
{
	if (!buffer)
		return;

	buffer->dma_phy = 0;
	buffer->dma_virt = NULL;
	buffer->dma_size = 0;
}

static int alloc_dma_buffer(struct vpu_ctx *ctx, struct dma_buffer *buffer)
{
	if (!ctx || !ctx->dev || !buffer)
		return -EINVAL;

	buffer->dma_virt = dma_alloc_coherent(&ctx->dev->plat_dev->dev,
			buffer->dma_size,
			(dma_addr_t *)&buffer->dma_phy,
			GFP_KERNEL | GFP_DMA32);

	if (!buffer->dma_virt) {
		vpu_dbg(LVL_ERR, "error: %s() dma buffer alloc size(%x) fail!\n",
				__func__,  buffer->dma_size);
		return -ENOMEM;
	}

	memset_io(buffer->dma_virt, 0, buffer->dma_size);
	atomic64_add(buffer->dma_size, &ctx->statistic.total_dma_size);
	return 0;
}

static int free_dma_buffer(struct vpu_ctx *ctx, struct dma_buffer *buffer)
{
	if (!ctx || !ctx->dev || !buffer)
		return -EINVAL;

	if (!buffer->dma_virt)
		return -1;

	dma_free_coherent(&ctx->dev->plat_dev->dev,
			buffer->dma_size,
			buffer->dma_virt,
			buffer->dma_phy);

	atomic64_sub(buffer->dma_size, &ctx->statistic.total_dma_size);
	init_dma_buffer(buffer);
	return 0;
}

static u_int32 get_mbi_size(struct queue_data *queue)
{
	u_int32 uAlign = 0x800;
	u_int32 mbi_size;

	mbi_size = (queue->sizeimage[0] + queue->sizeimage[1])/4;
	return ALIGN(mbi_size, uAlign);
}

static int alloc_mbi_buffer(struct vpu_ctx *ctx)
{
	u_int32 ret = 0;
	u_int32 i;

	for (i = 0; i < ctx->mbi_num; i++) {
		ctx->mbi_buffer[i].dma_size = ctx->mbi_size;
		ret = alloc_dma_buffer(ctx, &ctx->mbi_buffer[i]);
		if (ret) {
			vpu_dbg(LVL_ERR, "error: %s() ctx[%d] alloc mbi buffer fail\n",
					__func__, ctx->str_index);
			return ret;
		} else {
			vpu_dbg(LVL_EVENT, "%s() ctx[%d] mbi[%d]: virt: %p, phy: %lld\n",
					__func__, ctx->str_index, i, ctx->mbi_buffer[i].dma_virt, ctx->mbi_buffer[i].dma_phy);
		}
	}

	return ret;
}

static int alloc_dcp_buffer(struct vpu_ctx *ctx, uint32_t index)
{
	struct dma_buffer *dcp_buffer = NULL;
	int ret = 0;

	if (index >= ARRAY_SIZE(ctx->dcp_buffer))
		return -EINVAL;

	dcp_buffer = &ctx->dcp_buffer[index];
	if (dcp_buffer->dma_virt && dcp_buffer->dma_size >= DCP_SIZE)
		return 0;

	free_dma_buffer(ctx, dcp_buffer);
	dcp_buffer->dma_size = DCP_SIZE;
	ret = alloc_dma_buffer(ctx, dcp_buffer);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: alloc dcp buffer fail\n");
		return ret;
	}

	return 0;
}

static int free_mbi_buffers(struct vpu_ctx *ctx)
{
	u_int32 i;

	for (i = 0; i < ARRAY_SIZE(ctx->mbi_buffer); i++)
		free_dma_buffer(ctx, &ctx->mbi_buffer[i]);

	return 0;
}

static int free_decoder_buffer(struct vpu_ctx *ctx)
{
	struct queue_data *queue;
	u_int32 i;

	queue = &ctx->q_data[V4L2_DST];
	down(&queue->drv_q_lock);
	for (i = 0; i < ARRAY_SIZE(ctx->mbi_buffer); i++)
		free_dma_buffer(ctx, &ctx->mbi_buffer[i]);
	up(&queue->drv_q_lock);

	for (i = 0; i < ARRAY_SIZE(ctx->dcp_buffer); i++)
		free_dma_buffer(ctx, &ctx->dcp_buffer[i]);

	queue = &ctx->q_data[V4L2_SRC];
	down(&queue->drv_q_lock);
	free_dma_buffer(ctx, &ctx->stream_buffer);
	free_dma_buffer(ctx, &ctx->udata_buffer);
	up(&queue->drv_q_lock);

	return 0;
}

void clear_vb2_buf(struct queue_data *q_data)
{
	struct vb2_data_req *p_data_req;
	u_int32 i;

	if (!q_data)
		return;

	down(&q_data->drv_q_lock);
	for (i = 0; i < VPU_MAX_BUFFER; i++) {
		p_data_req = &q_data->vb2_reqs[i];
		p_data_req->vb2_buf = NULL;
	}
	up(&q_data->drv_q_lock);
}

static int v4l2_ioctl_reqbufs(struct file *file,
		void *fh,
		struct v4l2_requestbuffers *reqbuf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_BIT_FUNC, "%s(), buffer_type: %d, buffer_count: %d\n",
			__func__, reqbuf->type, reqbuf->count);

	if (reqbuf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (reqbuf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	clear_vb2_buf(q_data);
	if (reqbuf->count == 0)
		ctx->buffer_null = true;
	else
		ctx->buffer_null = false;

	ret = vpu_dec_queue_reqbufs(q_data, reqbuf);
	if (ret) {
		vpu_dbg(LVL_WARN, "warning: %s() can't request (%d) buffer : %d\n",
				__func__, reqbuf->count, ret);
		return ret;
	}
	if (V4L2_TYPE_IS_OUTPUT(reqbuf->type))
		return ret;

	down(&q_data->drv_q_lock);
	free_mbi_buffers(ctx);
	ctx->mbi_count = 0;
	ctx->mbi_num = min_t(u32, reqbuf->count, MAX_MBI_NUM);
	if (ctx->mbi_num > 0) {
		ctx->mbi_size = get_mbi_size(q_data);
		alloc_mbi_buffer(ctx);
	}
	up(&q_data->drv_q_lock);

	return ret;
}

static int v4l2_ioctl_querybuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	unsigned int i;
	int ret;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ret = vpu_dec_queue_querybuf(q_data, buf);
	if (!ret) {
		if (buf->memory == V4L2_MEMORY_MMAP) {
			if (V4L2_TYPE_IS_MULTIPLANAR(buf->type)) {
				for (i = 0; i < buf->length; i++)
					buf->m.planes[i].m.mem_offset |= (q_data->type << MMAP_BUF_TYPE_SHIFT);
			} else
				buf->m.offset |= (q_data->type << MMAP_BUF_TYPE_SHIFT);
		}
	} else
		vpu_dbg(LVL_ERR, "error: %s() return ret=%d\n", __func__, ret);

	return ret;
}

static int v4l2_ioctl_qbuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	struct vb2_data_req *p_data_req;
	int ret;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ctx->total_qbuf_bytes += buf->m.planes[0].bytesused;
		vpu_dbg(LVL_BIT_FRAME_BYTES, "[%d]input bytes : %8d / %16ld\n",
				ctx->str_index,
				buf->m.planes[0].bytesused,
				ctx->total_qbuf_bytes);
		q_data = &ctx->q_data[V4L2_SRC];

		v4l2_update_stream_addr(ctx, 0);
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		q_data = &ctx->q_data[V4L2_DST];
		down(&q_data->drv_q_lock);
		p_data_req = &q_data->vb2_reqs[buf->index];
		p_data_req->data_offset[0] = buf->m.planes[0].data_offset;
		p_data_req->data_offset[1] = buf->m.planes[1].data_offset;
		up(&q_data->drv_q_lock);
	}
	else
		return -EINVAL;

	ret = vpu_dec_queue_qbuf(q_data, buf);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: %s() return ret=%d\n", __func__, ret);
		return ret;
	}
	if (!V4L2_TYPE_IS_OUTPUT(buf->type))
		wake_up_interruptible(&ctx->buffer_wq);
	v4l2_update_stream_addr(ctx, 0);

	return ret;
}

static void vpu_dec_send_ts(struct vpu_ctx *ctx, struct v4l2_buffer *buf)
{
	TSM_TIMESTAMP ts = TSManagerSend2(ctx->tsm, NULL);

	vpu_dbg(LVL_BIT_TS, "[OUTPUT TS]%32lld (%lld)\n",
			ts, getTSManagerFrameInterval(ctx->tsm));
	buf->timestamp = ns_to_timeval(ts);
	buf->flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
}

static int v4l2_ioctl_dqbuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ret = vpu_dec_queue_dqbuf(q_data, buf, file->f_flags & O_NONBLOCK);

	if (ret) {
		vpu_dbg(LVL_ERR, "error: %s() return ret=%d\n", __func__, ret);
		return ret;
	}

	if (q_data->vb2_reqs[buf->index].bfield)
		buf->field = V4L2_FIELD_INTERLACED;
	else
		buf->field = V4L2_FIELD_NONE;
	v4l2_update_stream_addr(ctx, 0);
	if (!V4L2_TYPE_IS_OUTPUT(buf->type) && is_10bit_format(ctx))
		buf->reserved = 1;

	if (!V4L2_TYPE_IS_OUTPUT(buf->type))
		vpu_dec_send_ts(ctx, buf);

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

static int v4l2_ioctl_try_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f
		)
{
	unsigned int table_size;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		table_size = ARRAY_SIZE(formats_compressed_dec);
		if (!format_is_support(formats_compressed_dec, table_size, f))
			return -EINVAL;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		table_size = ARRAY_SIZE(formats_yuv_dec);
		if (!format_is_support(formats_yuv_dec, table_size, f))
			return -EINVAL;
	} else
		return -EINVAL;

	return 0;
}

static int v4l2_ioctl_g_crop(struct file *file,
		void *fh,
		struct v4l2_crop *cr
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);
	cr->c.left = ctx->pSeqinfo->uFrameCropLeftOffset;
	cr->c.top = ctx->pSeqinfo->uFrameCropTopOffset;
	cr->c.width = ctx->pSeqinfo->uHorRes;
	cr->c.height = ctx->pSeqinfo->uVerRes;

	return 0;
}

static int v4l2_ioctl_decoder_cmd(struct file *file,
		void *fh,
		struct v4l2_decoder_cmd *cmd
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	switch (cmd->cmd) {
	case V4L2_DEC_CMD_START:
		break;
	case V4L2_DEC_CMD_STOP: {
		vpu_dbg(LVL_EVENT, "ctx[%d]: receive V4L2_DEC_CMD_STOP\n", ctx->str_index);
		ctx->eos_stop_received = true;
		v4l2_update_stream_addr(ctx, 0);
		}
		break;
	case V4L2_DEC_CMD_PAUSE:
		break;
	case V4L2_DEC_CMD_RESUME:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int v4l2_ioctl_streamon(struct file *file,
		void *fh,
		enum v4l2_buf_type i
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_BIT_FUNC, "%s(), buffer_type: %d\n", __func__, i);

	if (i == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (i == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ctx->firmware_finished = false;

	ret = vpu_dec_queue_enable(q_data, i);
	if (!ret) {
		if (i == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			wake_up_interruptible(&ctx->buffer_wq);
			complete(&ctx->cap_streamon_cmp);
		}

		v4l2_update_stream_addr(ctx, 0);
	} else
		vpu_dbg(LVL_ERR, "error: %s() return ret=%d\n", __func__, ret);

	if (ctx->hang_status) {
		vpu_dbg(LVL_ERR, "%s(): not succeed and some instance are blocked\n", __func__);
		return -EINVAL;
	} else
		return ret;
}

static int v4l2_ioctl_streamoff(struct file *file,
		void *fh,
		enum v4l2_buf_type i
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_BIT_FUNC, "%s(): ctx[%d] buf_type: %d\n",
			__func__, ctx->str_index, i);

	if (i == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (i == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	if (i == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (vpu_dec_is_active(ctx)) {
			int size;
			ctx->wait_rst_done = true;
			vpu_dbg(LVL_INFO, "%s(): send VID_API_CMD_ABORT\n", __func__);

			size = add_scode(ctx, 0, BUFABORT_PADDING_TYPE, false);
			record_log_info(ctx, LOG_PADDING, 0, 0);
			if (size < 0)
				vpu_dbg(LVL_ERR, "%s(): failed to fill abort padding data\n", __func__);
			v4l2_vpu_send_cmd(ctx, ctx->str_index, VID_API_CMD_ABORT, 1, &size);

			wake_up_interruptible(&ctx->buffer_wq);
			if (!wait_for_completion_timeout(&ctx->completion, msecs_to_jiffies(1000))) {
				ctx->hang_status = true;
				vpu_dbg(LVL_ERR, "the path id:%d firmware timeout after send VID_API_CMD_ABORT, stopped(%d), finished(%d), eos_added(%d)\n",
						ctx->str_index, ctx->firmware_stopped,
						ctx->firmware_finished, ctx->eos_stop_added);
			}
			vpu_dbg(LVL_INFO, "receive abort done\n");
		} else {
			vpu_dbg(LVL_ERR,
				"%s() - IGNORE - stopped(%d), finished(%d), eos_added(%d)\n",
				__func__,
				ctx->firmware_stopped,
				ctx->firmware_finished,
				ctx->eos_stop_added);
		}
	}

	ctx->tsm_sync_flag = true;

	ret = vpu_dec_queue_disable(q_data, i);

	if (ctx->hang_status) {
		vpu_dbg(LVL_ERR, "%s(): not succeed and some instance are blocked\n", __func__);
		return -EINVAL;
	} else
		return ret;
}

static int vpu_dec_v4l2_ioctl_g_parm(struct file *file, void *fh,
				struct v4l2_streamparm *parm)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	u32 numerator;
	u32 denominator;

	mutex_lock(&ctx->instance_mutex);

	numerator = ctx->fixed_frame_interval.numerator;
	denominator = ctx->fixed_frame_interval.denominator;
	if (!numerator || !denominator) {
		numerator = ctx->frame_interval.numerator;
		denominator = ctx->frame_interval.denominator;
	}
	if (!numerator || !denominator) {
		numerator = 0;
		denominator = 0;
	}
	parm->parm.capture.timeperframe.numerator = numerator;
	parm->parm.capture.timeperframe.denominator = denominator;

	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static void vpu_dec_set_tsm_frame_rate(struct vpu_ctx *ctx)
{
	u32 numerator;
	u32 denominator;

	WARN_ON(!ctx);

	numerator = ctx->fixed_frame_interval.numerator;
	denominator = ctx->fixed_frame_interval.denominator;
	if (numerator && denominator) {
		setTSManagerFrameRate(ctx->tsm, denominator, numerator);
		return;
	}

	numerator = ctx->frame_interval.numerator;
	denominator = ctx->frame_interval.denominator;
	if (numerator && denominator)
		setTSManagerFrameRate(ctx->tsm, denominator, numerator);
}

static int vpu_dec_v4l2_ioctl_s_parm(struct file *file, void *fh,
				struct v4l2_streamparm *parm)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	u32 gcd;

	if (!parm->parm.capture.timeperframe.numerator ||
			!parm->parm.capture.timeperframe.denominator)
		return -EINVAL;

	gcd = get_greatest_common_divisor(
			parm->parm.capture.timeperframe.numerator,
			parm->parm.capture.timeperframe.denominator);

	mutex_lock(&ctx->instance_mutex);
	ctx->fixed_frame_interval.numerator =
		parm->parm.capture.timeperframe.numerator / gcd;
	ctx->fixed_frame_interval.denominator =
		parm->parm.capture.timeperframe.denominator / gcd;
	vpu_dec_set_tsm_frame_rate(ctx);
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static const struct v4l2_ioctl_ops v4l2_decoder_ioctl_ops = {
	.vidioc_querycap                = v4l2_ioctl_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = v4l2_ioctl_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out_mplane = v4l2_ioctl_enum_fmt_vid_out_mplane,
	.vidioc_g_fmt_vid_cap_mplane    = v4l2_ioctl_g_fmt,
	.vidioc_g_fmt_vid_out_mplane    = v4l2_ioctl_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane  = v4l2_ioctl_try_fmt,
	.vidioc_try_fmt_vid_out_mplane  = v4l2_ioctl_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane    = v4l2_ioctl_s_fmt,
	.vidioc_s_fmt_vid_out_mplane    = v4l2_ioctl_s_fmt,
	.vidioc_g_parm			= vpu_dec_v4l2_ioctl_g_parm,
	.vidioc_s_parm			= vpu_dec_v4l2_ioctl_s_parm,
	.vidioc_expbuf                  = v4l2_ioctl_expbuf,
	.vidioc_g_crop                  = v4l2_ioctl_g_crop,
	.vidioc_decoder_cmd             = v4l2_ioctl_decoder_cmd,
	.vidioc_subscribe_event         = v4l2_ioctl_subscribe_event,
	.vidioc_unsubscribe_event       = v4l2_event_unsubscribe,
	.vidioc_reqbufs                 = v4l2_ioctl_reqbufs,
	.vidioc_querybuf                = v4l2_ioctl_querybuf,
	.vidioc_qbuf                    = v4l2_ioctl_qbuf,
	.vidioc_dqbuf                   = v4l2_ioctl_dqbuf,
	.vidioc_streamon                = v4l2_ioctl_streamon,
	.vidioc_streamoff               = v4l2_ioctl_streamoff,
};

// Set/Get controls - v4l2 control framework

static struct vpu_v4l2_control vpu_controls_dec[] = {
	{
		.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
		.minimum = 1,
		.maximum = 32,
		.step = 1,
		.default_value = 4,
		.is_volatile = true,
	},
};

#define NUM_CTRLS_DEC   ARRAY_SIZE(vpu_controls_dec)

static int v4l2_custom_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);

	vpu_dbg(LVL_BIT_FUNC, "%s() control(%d)\n",
			__func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_USER_RAW_BASE:
		ctx->start_code_bypass = ctrl->val;
		break;
	case V4L2_CID_USER_FRAME_DEPTH:
		vpu_frm_depth = ctrl->val;
		break;
	default:
		vpu_dbg(LVL_ERR, "%s() Invalid costomer control(%d)\n",
				__func__, ctrl->id);
		return -EINVAL;
	}
	return 0;
}

static int v4l2_dec_g_v_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);

	vpu_dbg(LVL_BIT_FUNC, "%s() control(%d)\n",
			__func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ctrl->val = ctx->pSeqinfo->uNumDPBFrms + ctx->pSeqinfo->uNumRefFrms;
		break;
	default:
		vpu_dbg(LVL_ERR, "%s() Invalid control(%d)\n",
				__func__, ctrl->id);
		return -EINVAL;
	}
	return 0;
}

static int add_dec_ctrl(struct vpu_ctx *This)
{
	static const struct v4l2_ctrl_ops vpu_dec_ctrl_ops = {
		.g_volatile_ctrl  = v4l2_dec_g_v_ctrl,
	};
	u_int32 i;

	for (i = 0; i < NUM_CTRLS_DEC; i++) {
		This->ctrls[i] = v4l2_ctrl_new_std(&This->ctrl_handler,
				&vpu_dec_ctrl_ops,
				vpu_controls_dec[i].id,
				vpu_controls_dec[i].minimum,
				vpu_controls_dec[i].maximum,
				vpu_controls_dec[i].step,
				vpu_controls_dec[i].default_value
				);
		if (This->ctrl_handler.error ||
				!This->ctrls[i]) {
			vpu_dbg(LVL_ERR, "%s() v4l2_ctrl_new_std failed(%d) This->ctrls[%d](%p)\n",
					__func__, This->ctrl_handler.error, i, This->ctrls[i]);
			return This->ctrl_handler.error;
		}

		if (vpu_controls_dec[i].is_volatile &&
				This->ctrls[i])
			This->ctrls[i]->flags |= V4L2_CTRL_FLAG_VOLATILE;
	}

	return 0;
}

static int add_custom_ctrl(struct vpu_ctx *This)
{
	static const struct v4l2_ctrl_ops vpu_custom_ctrl_ops = {
		.s_ctrl = v4l2_custom_s_ctrl,
	};
	struct v4l2_ctrl_config cfg;
	struct v4l2_ctrl *ctrl;

	memset(&cfg, 0, sizeof(struct v4l2_ctrl_config));
	cfg.ops = &vpu_custom_ctrl_ops;
	cfg.id = V4L2_CID_USER_RAW_BASE;
	cfg.name = "Raw Ctrl";
	cfg.min = 0;
	cfg.max = 1;
	cfg.step = 1;
	cfg.def = 0;
	cfg.type = V4L2_CTRL_TYPE_INTEGER;

	ctrl = v4l2_ctrl_new_custom(&This->ctrl_handler,
			&cfg, NULL);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "Add custom ctrl fail\n");
		return -EINVAL;
	}

	memset(&cfg, 0, sizeof(struct v4l2_ctrl_config));
	cfg.ops = &vpu_custom_ctrl_ops;
	cfg.id = V4L2_CID_USER_FRAME_DEPTH;
	cfg.name = "frame depth ctrl";
	cfg.min = -1;
	cfg.max = 999;
	cfg.step = 1;
	cfg.def = -1;
	cfg.type = V4L2_CTRL_TYPE_INTEGER;

	ctrl = v4l2_ctrl_new_custom(&This->ctrl_handler,
			&cfg, NULL);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "Add custom ctrl fail\n");
		return -EINVAL;
	}

	return 0;
}

static int ctrls_setup_decoder(struct vpu_ctx *This)
{
	if (!This)
		return -EINVAL;

	v4l2_ctrl_handler_init(&This->ctrl_handler,
			NUM_CTRLS_DEC + 1
			);
	if (This->ctrl_handler.error) {
		vpu_dbg(LVL_ERR, "%s() v4l2_ctrl_handler_init failed(%d)\n",
				__func__, This->ctrl_handler.error);

		return This->ctrl_handler.error;
	} else {
		vpu_dbg(LVL_INFO, "%s() v4l2_ctrl_handler_init ctrls(%ld)\n",
				__func__, NUM_CTRLS_DEC);
		This->ctrl_inited = true;
	}

	add_dec_ctrl(This);
	add_custom_ctrl(This);

	v4l2_ctrl_handler_setup(&This->ctrl_handler);

	return 0;
}

static void ctrls_delete_decoder(struct vpu_ctx *This)
{
	int i;

	if (This->ctrl_inited) {
		v4l2_ctrl_handler_free(&This->ctrl_handler);
		This->ctrl_inited = false;
	}
	for (i = 0; i < NUM_CTRLS_DEC; i++)
		This->ctrls[i] = NULL;
}

static void update_wptr(struct vpu_ctx *ctx,
			pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc,
			u32 wptr)
{
	u32 size;
	u32 length;

	size = pStrBufDesc->end - pStrBufDesc->start;
	if (wptr == pStrBufDesc->wptr)
		length = size;
	else
		length = (wptr + size - pStrBufDesc->wptr) % size;
	ctx->total_write_bytes += length;

	vpu_dbg(LVL_BIT_WPTR, "wptr : 0x%08x -> 0x%08x\n",
			pStrBufDesc->wptr, wptr);
	pStrBufDesc->wptr = wptr;
}

/* Insert either the codec specific EOS type or a special scode to mark that this frame should be flushed/pushed directly for decode */
static int add_scode_vpu(struct vpu_ctx *ctx, u_int32 uStrBufIdx, VPU_PADDING_SCODE_TYPE eScodeType, bool bUpdateWr)
{
	struct vpu_dev *dev = ctx->dev;
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	uint32_t start;
	uint32_t end;
	uint32_t wptr;
	uint32_t rptr;
	uint8_t *pbbuffer;
	uint32_t *plbuffer;
	uint32_t last;
	uint32_t last2 = 0x0;
	uint32_t pad_bytes = 0;
	uint8_t *buffer;

	vpu_dbg(LVL_BIT_FUNC, "enter %s()\n", __func__);
	pStrBufDesc = get_str_buffer_desc(ctx);
	start = pStrBufDesc->start;
	end = pStrBufDesc->end;
	wptr = pStrBufDesc->wptr;
	rptr = pStrBufDesc->rptr;

	if (start != ctx->stream_buffer.dma_phy ||
			end != ctx->stream_buffer.dma_phy + ctx->stream_buffer.dma_size) {
		vpu_dbg(LVL_ERR, "error: %s(), start or end pointer cross-border\n", __func__);
		return 0;
	}
	if (wptr < start || wptr > end) {
		vpu_dbg(LVL_ERR, "error: %s(), wptr pointer cross-border\n", __func__);
		return 0;
	}
	if (rptr < start || rptr > end) {
		vpu_dbg(LVL_ERR, "error: %s(), rptr pointer cross-border\n", __func__);
		return 0;
	}

	buffer = kzalloc(SCODE_SIZE, GFP_KERNEL); //for eos data
	if (!buffer) {
		vpu_dbg(LVL_ERR, "error:  eos buffer alloc fail\n");
		return -1;
	}
	atomic64_add(SCODE_SIZE, &ctx->statistic.total_alloc_size);
	plbuffer = (uint32_t *)buffer;
	if (wptr - start < ctx->stream_buffer.dma_size)
		pbbuffer = (uint8_t *)(ctx->stream_buffer.dma_virt + wptr - start);
	else {
		vpu_dbg(LVL_ERR, "error: return wptr(0x%x), start(0x%x) is not valid\n", wptr, start);
		return -1;
	}

	// Word align
	if (((u_int64)pbbuffer)%4 != 0) {
		int i;
		if (end%4 != 0) {
			vpu_dbg(LVL_ERR, "end address of stream not aligned by 4 bytes !\n");
			return -1;
		}
		pad_bytes = 4 - (((u_int64)pbbuffer)%4);
		for (i = 0; i < pad_bytes; i++)
			pbbuffer[i] = 0;
		pbbuffer += pad_bytes;
		wptr += pad_bytes;
		if (wptr == end) {
			wptr = start;
			pbbuffer = (uint8_t *)ctx->stream_buffer.dma_virt;
		}
	}

	if (eScodeType == BUFABORT_PADDING_TYPE) {
		switch (q_data->vdec_std) {
		case VPU_VIDEO_AVC:
			last = 0x0B010000;
			break;
		case VPU_VIDEO_VC1:
			last = 0x0a010000;
			break;
		case VPU_VIDEO_MPEG2:
			last = 0xb7010000;
			break;
		case VPU_VIDEO_ASP:
			last = 0xb1010000;
			break;
		case VPU_VIDEO_SPK:
		case VPU_VIDEO_VP6:
		case VPU_VIDEO_VP8:
		case VPU_VIDEO_RV:
			last = 0x34010000;
			break;
		case VPU_VIDEO_HEVC:
			last = 0x4A010000;
			last2 = 0x20;
			break;
		case VPU_VIDEO_JPEG:
		default:
			last = 0x0;
			break;
		}
	} else if (eScodeType == EOS_PADDING_TYPE) {
		switch (q_data->vdec_std) {
		case VPU_VIDEO_AVC:
			last = 0x0B010000;
			break;
		case VPU_VIDEO_VC1:
			last = 0x0a010000;
			break;
		case VPU_VIDEO_MPEG2:
			last = EOS_GENERIC_MPEG;
			break;
		case VPU_VIDEO_ASP:
			last = 0xb1010000;
			break;
		case VPU_VIDEO_SPK:
		case VPU_VIDEO_VP6:
		case VPU_VIDEO_VP8:
		case VPU_VIDEO_RV:
			last = 0x34010000;
			break;
		case VPU_VIDEO_JPEG:
			last = EOS_GENERIC_JPEG;
			break;
		case VPU_VIDEO_HEVC:
			last = 0x4A010000;
			last2 = 0x20;
			break;
		default:
			last = 0x0;
			break;
		}
	} else {
		if (q_data->vdec_std == VPU_VIDEO_AVC) {
			last = 0x15010000;
			last2 = 0x0;
		} else {
			/* all other standards do not support the frame flush mechanism so just return */
			vpu_dbg(LVL_WARN, "warning: format(%d) not support frame flush mechanism !\n", q_data->vdec_std);
			return 0;
		}
	}
	plbuffer[0] = last;
	plbuffer[1] = last2;

	if ((wptr == rptr) || (wptr > rptr)) {
		if (end - wptr >= SCODE_SIZE) {
			memcpy(pbbuffer, buffer, SCODE_SIZE);
			wptr += SCODE_SIZE;
			if (wptr == end)
				wptr = start;
		} else {
			memcpy(pbbuffer, buffer, end-wptr);
			memcpy(ctx->stream_buffer.dma_virt, buffer + (end - wptr), SCODE_SIZE - (end - wptr));
			wptr = start + SCODE_SIZE - (end - wptr);
		}
		pad_bytes += SCODE_SIZE;
	} else {
		if (rptr - wptr >= SCODE_SIZE) {
			memcpy(pbbuffer, buffer, SCODE_SIZE);
			wptr += SCODE_SIZE;
			pad_bytes += SCODE_SIZE;
		} else	{
			//shouldn't enter here: suppose space is enough since add_eos() only be called in FIFO LOW
			vpu_dbg(LVL_ERR, "No enough space to insert EOS, size=%d !\n", rptr - wptr);
			memcpy(pbbuffer, buffer, rptr - wptr);
			wptr += (rptr - wptr);
			pad_bytes += (rptr - wptr);
		}
	}
	mb();

	if (bUpdateWr)
		update_wptr(ctx, pStrBufDesc, wptr);
	dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->str_index][uStrBufIdx] =
		(VPU_REG_BASE + get_str_buffer_desc_offset(ctx));
	kfree(buffer);
	atomic64_sub(SCODE_SIZE, &ctx->statistic.total_alloc_size);
	vpu_dbg(LVL_INFO, "%s() done type (%d) MCX address virt=%p, phy=0x%x, index=%d\n",
			__func__, eScodeType, pStrBufDesc, dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->str_index][uStrBufIdx], ctx->str_index);
	return pad_bytes;
}

static int add_scode(struct vpu_ctx *ctx, u_int32 uStrBufIdx, VPU_PADDING_SCODE_TYPE eScodeType, bool bUpdateWr)
{
	int size = 0;

	if (!ctx)
		return 0;

	mutex_lock(&ctx->instance_mutex);
	size = add_scode_vpu(ctx, uStrBufIdx, eScodeType, bUpdateWr);
	if (size > 0)
		set_pic_end_flag(ctx);
	mutex_unlock(&ctx->instance_mutex);
	return size;
}

TB_API_DEC_FMT vpu_format_remap(uint32_t vdec_std)
{
	TB_API_DEC_FMT malone_format = VSys_FrmtNull;

	switch (vdec_std) {
	case VPU_VIDEO_AVC:
		malone_format = VSys_AvcFrmt;
		vpu_dbg(LVL_INFO, "format translated to AVC");
		break;
	case VPU_VIDEO_VC1:
		malone_format = VSys_Vc1Frmt;
		vpu_dbg(LVL_INFO, "format translated to VC1");
		break;
	case VPU_VIDEO_MPEG2:
		malone_format = VSys_Mp2Frmt;
		vpu_dbg(LVL_INFO, "format translated to MP2");
		break;
	case VPU_VIDEO_AVS:
		malone_format = VSys_AvsFrmt;
		vpu_dbg(LVL_INFO, "format translated to AVS");
		break;
	case VPU_VIDEO_ASP:
		malone_format = VSys_AspFrmt;
		vpu_dbg(LVL_INFO, "format translated to ASP");
		break;
	case VPU_VIDEO_JPEG:
		malone_format = VSys_JpgFrmt;
		vpu_dbg(LVL_INFO, "format translated to JPG");
		break;
	case VPU_VIDEO_VP6:
		malone_format = VSys_Vp6Frmt;
		vpu_dbg(LVL_INFO, "format translated to VP6");
		break;
	case VPU_VIDEO_SPK:
		malone_format = VSys_SpkFrmt;
		vpu_dbg(LVL_INFO, "format translated to SPK");
		break;
	case VPU_VIDEO_VP8:
		malone_format = VSys_Vp8Frmt;
		vpu_dbg(LVL_INFO, "format translated to VP8");
		break;
	case VPU_VIDEO_HEVC:
		malone_format = VSys_HevcFrmt;
		vpu_dbg(LVL_INFO, "format translated to HEVC");
		break;
	case VPU_VIDEO_RV:
		malone_format = VSys_RvFrmt;
		vpu_dbg(LVL_INFO, "format translated to RV");
		break;
	case VPU_VIDEO_AVC_MVC:
		malone_format = VSys_AvcFrmt;
		vpu_dbg(LVL_INFO, "format translated to AVC");
		break;
	default:
		malone_format = VSys_FrmtNull;
		vpu_dbg(LVL_INFO, "unspport format");
		break;
	}
	vpu_dbg(LVL_INFO, "\n");

	return malone_format;
}

static void v4l2_vpu_send_cmd(struct vpu_ctx *ctx, uint32_t idx, uint32_t cmdid, uint32_t cmdnum, uint32_t *local_cmddata)
{
	vpu_log_cmd(cmdid, idx);
	count_cmd(&ctx->statistic, cmdid);
	record_log_info(ctx, LOG_COMMAND, cmdid, 0);
	mutex_lock(&ctx->dev->cmd_mutex);
	rpc_send_cmd_buf(&ctx->dev->shared_mem, idx, cmdid, cmdnum, local_cmddata);
	mutex_unlock(&ctx->dev->cmd_mutex);
	mb();
	MU_SendMessage(ctx->dev->mu_base_virtaddr, 0, COMMAND);
}

static void transfer_buffer_to_firmware(struct vpu_ctx *ctx, void *input_buffer, uint32_t buffer_size, uint32_t vdec_std)
{
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	u_int32 uStrBufIdx = 0; //set to be default 0, FIX_ME later
	MediaIPFW_Video_UData *pUdataBuf =
		&ctx->dev->shared_mem.pSharedInterface->UDataBuffer[ctx->str_index];
	pDEC_RPC_HOST_IFACE pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	unsigned int *CurrStrfg = &pSharedInterface->StreamConfig[ctx->str_index];
	u_int32 length;
	MediaIPFW_Video_CodecParams *pCodecPara;

	vpu_dbg(LVL_BIT_FUNC, "enter %s, start_flag %d, index=%d, firmware_started=%d\n",
			__func__, ctx->start_flag, ctx->str_index, ctx->dev->firmware_started);

	vpu_dbg(LVL_WARN, "firmware version is %d.%d.%d\n",
			(pSharedInterface->FWVersion & 0x00ff0000) >> 16,
			(pSharedInterface->FWVersion & 0x0000ff00) >> 8,
			pSharedInterface->FWVersion & 0x000000ff);


	if (ctx->stream_buffer.dma_size < buffer_size + MIN_SPACE) {
		vpu_dbg(LVL_ERR, "circular buffer size is set too small\n");
		return;
	}
	if (!ctx->start_code_bypass)
		length = insert_scode_4_seq(ctx, input_buffer, ctx->stream_buffer.dma_virt, vdec_std, buffer_size);
	else
		length = 0;
	if (length == 0) {
		memcpy(ctx->stream_buffer.dma_virt, input_buffer, buffer_size);
		length = buffer_size;
	}
	vpu_dbg(LVL_INFO, "transfer data from virt 0x%p: size:%d\n",
			ctx->stream_buffer.dma_virt, buffer_size);
	mb();
	pStrBufDesc = get_str_buffer_desc(ctx);
	pStrBufDesc->wptr = ctx->stream_buffer.dma_phy;
	pStrBufDesc->rptr = ctx->stream_buffer.dma_phy;
	pStrBufDesc->start = ctx->stream_buffer.dma_phy;
	pStrBufDesc->end = ctx->stream_buffer.dma_phy + ctx->stream_buffer.dma_size;
	pStrBufDesc->LWM = 0x01;
	ctx->pre_pic_end_addr = pStrBufDesc->start;

	update_wptr(ctx, pStrBufDesc, ctx->stream_buffer.dma_phy + length);
	ctx->dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->str_index][uStrBufIdx] =
		(VPU_REG_BASE + get_str_buffer_desc_offset(ctx));

	vpu_dbg(LVL_INFO, "transfer MCX address virt=%p, phy=0x%x, index=%d\n",
			pStrBufDesc, ctx->dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->str_index][uStrBufIdx], ctx->str_index);
	pUdataBuf->uUDataBase = ctx->udata_buffer.dma_phy;
	pUdataBuf->uUDataSlotSize = ctx->udata_buffer.dma_size;
	VID_STREAM_CONFIG_FORMAT_SET(vpu_format_remap(vdec_std), CurrStrfg);
	if (vdec_std == VPU_VIDEO_JPEG) {
		MediaIPFW_Video_JpegParams *pJpgPara;

		pJpgPara = (MediaIPFW_Video_JpegParams *)ctx->dev->shared_mem.jpeg_mem_vir;
		pJpgPara[ctx->str_index].uJpgMjpegMode = 1; //1:JPGD_MJPEG_MODE_A; 2:JPGD_MJPEG_MODE_B
		pJpgPara[ctx->str_index].uJpgMjpegInterlaced = 0; //0: JPGD_MJPEG_PROGRESSIVE
	}

	pCodecPara = (MediaIPFW_Video_CodecParams *)ctx->dev->shared_mem.codec_mem_vir;
	if (ctx->b_dis_reorder) {
		/* set the shared memory space control with this */
		add_scode(ctx, 0, BUFFLUSH_PADDING_TYPE, true);
		record_log_info(ctx, LOG_PADDING, 0, 0);
		pCodecPara[ctx->str_index].uDispImm = 1;
	} else {
		pCodecPara[ctx->str_index].uDispImm = 0;
	}

	pCodecPara[ctx->str_index].uEnableDbgLog = CHECK_BIT(vpu_frmdbg_ena, ctx->str_index) ? 1 : 0;
	ctx->dev->shared_mem.pSharedInterface->DbgLogDesc.uDecStatusLogLevel = vpu_frmdbg_level;

	/*initialize frame count*/
	ctx->frm_dis_delay = 1;
	ctx->frm_dec_delay = 1;
	ctx->frm_total_num = 1;
	fill_stream_buffer_info(ctx);
}

static void vpu_dec_receive_ts(struct vpu_ctx *ctx,
				struct vb2_buffer *vb,
				int size)
{
	TSM_TIMESTAMP input_ts = -1;

	if (vb->timestamp >= 0)
		input_ts = vb->timestamp;
	vpu_dbg(LVL_BIT_TS, "[INPUT  TS]%32lld\n", input_ts);

	if (ctx->tsm_sync_flag) {
		resyncTSManager(ctx->tsm, input_ts, tsm_mode);
		ctx->tsm_sync_flag = false;
	}
	TSManagerReceive2(ctx->tsm, input_ts, size);
	ctx->total_ts_bytes += size;
}

static void v4l2_transfer_buffer_to_firmware(struct queue_data *This, struct vb2_buffer *vb)
{
	struct vpu_ctx *ctx = container_of(This, struct vpu_ctx, q_data[V4L2_SRC]);
	struct vb2_data_req *p_data_req;
	void *data_mapped;
	uint32_t buffer_size = vb->planes[0].bytesused;
	int ret;

	data_mapped = (void *)vb2_plane_vaddr(vb, 0);

	if (ctx->start_flag == true) {
		ret = alloc_vpu_buffer(ctx);
		if (ret) {
			vpu_dbg(LVL_ERR, "alloc vpu buffer fail\n");
			return;
		}
		transfer_buffer_to_firmware(ctx, data_mapped, buffer_size, This->vdec_std);
		vpu_dec_receive_ts(ctx, vb, vb2_get_plane_payload(vb, 0));
#ifdef HANDLE_EOS
		if (vb->planes[0].bytesused < vb->planes[0].length)
			vpu_dbg(LVL_INFO, "v4l2_transfer_buffer_to_firmware - set stream_feed_complete - DEBUG 1\n");
#endif
		v4l2_vpu_send_cmd(ctx, ctx->str_index, VID_API_CMD_START, 0, NULL);
		p_data_req = list_first_entry(&This->drv_q,
				typeof(*p_data_req), list);
		list_del(&p_data_req->list);
		p_data_req->queued = false;
		if (p_data_req->vb2_buf)
			vb2_buffer_done(p_data_req->vb2_buf,
					VB2_BUF_STATE_DONE);
		ctx->start_flag = false;
	}
}

static u_int32 got_free_space(u_int32 wptr, u_int32 rptr, u_int32 start, u_int32 end)
{
	u_int32 freespace = 0;

	if (wptr == rptr)
		freespace = end - start;
	if (wptr < rptr)
		freespace = rptr - wptr;
	if (wptr > rptr)
		freespace = (end - wptr) + (rptr - start);
	return freespace;
}

static int update_stream_addr(struct vpu_ctx *ctx, void *input_buffer, uint32_t buffer_size, uint32_t uStrBufIdx)
{
	struct vpu_dev *dev = ctx->dev;
	uint32_t index = ctx->str_index;
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	u_int8 payload_header[256] = {0};
	uint32_t nfreespace = 0;
	uint32_t wptr;
	uint32_t rptr;
	uint32_t start;
	uint32_t end;
	void *wptr_virt;
	uint32_t ret = 1;
	u_int32 length = 0;

	vpu_dbg(LVL_BIT_FUNC, "enter %s\n", __func__);

	// changed to virtual address and back
	pStrBufDesc = get_str_buffer_desc(ctx);
	vpu_dbg(LVL_BIT_BUFFER_DESC,
		"%s wptr(%x) rptr(%x) start(%x) end(%x) uStrBufIdx(%d)\n",
		__func__,
		pStrBufDesc->wptr,
		pStrBufDesc->rptr,
		pStrBufDesc->start,
		pStrBufDesc->end,
		uStrBufIdx);
	wptr = pStrBufDesc->wptr;
	rptr = pStrBufDesc->rptr;

	start = pStrBufDesc->start;
	end = pStrBufDesc->end;

	if (start != ctx->stream_buffer.dma_phy ||
			end != ctx->stream_buffer.dma_phy + ctx->stream_buffer.dma_size) {
		vpu_dbg(LVL_ERR, "error: %s(), start or end pointer cross-border\n", __func__);
		return 0;
	}
	if (wptr < start || wptr > end) {
		vpu_dbg(LVL_ERR, "error: %s(), wptr pointer cross-border\n", __func__);
		return 0;
	}
	if (rptr < start || rptr > end) {
		vpu_dbg(LVL_ERR, "error: %s(), rptr pointer cross-border\n", __func__);
		return 0;
	}
	wptr_virt = (void *)ctx->stream_buffer.dma_virt + wptr - start;

	vpu_dbg(LVL_BIT_FUNC, "%s down\n", __func__);

	nfreespace = got_free_space(wptr, rptr, start, end);

	if (!ctx->start_code_bypass)
		length = insert_scode_4_pic(ctx, payload_header, input_buffer, q_data->vdec_std, buffer_size);
	else
		length = 0;

	if (nfreespace - buffer_size - length < MIN_SPACE)
		return 0;

	if (nfreespace >= buffer_size + length) {
		if ((wptr == rptr) || (wptr > rptr)) {
			if (end - wptr >= length) {
				memcpy(wptr_virt, payload_header, length);
				wptr += length;
				wptr_virt += length;
				if (wptr == end) {
					wptr = start;
					wptr_virt = (void *)ctx->stream_buffer.dma_virt;
				}
			} else {
				memcpy(wptr_virt, payload_header, end-wptr);
				memcpy(ctx->stream_buffer.dma_virt, payload_header + (end-wptr), length - (end-wptr));
				wptr = start + length - (end-wptr);
				wptr_virt = (void *)ctx->stream_buffer.dma_virt + length - (end-wptr);
			}
			if (end - wptr >= buffer_size) {
				memcpy(wptr_virt, input_buffer, buffer_size);
				wptr += buffer_size;
				if (wptr == end)
					wptr = start;
			} else {
				memcpy(wptr_virt, input_buffer, end-wptr);
				memcpy(ctx->stream_buffer.dma_virt, input_buffer + (end-wptr), buffer_size - (end-wptr));
				wptr = start + buffer_size - (end-wptr);
			}
		} else {
			memcpy(wptr_virt, payload_header, length);
			wptr += length;
			wptr_virt += length;
			memcpy(wptr_virt, input_buffer, buffer_size);
			wptr += buffer_size;
		}
	} else {
		vpu_dbg(LVL_INFO, "buffer_full: the circular buffer freespace < buffer_size, treat as full");
		return 0; //do not consider this situation now
	}

	mb();
	update_wptr(ctx, pStrBufDesc, wptr);
	vpu_dbg(LVL_BIT_FUNC, "%s up, wptr 0x%x\n", __func__, wptr);

	dev->shared_mem.pSharedInterface->pStreamBuffDesc[index][uStrBufIdx] =
		(VPU_REG_BASE + get_str_buffer_desc_offset(ctx));

	vpu_dbg(LVL_BIT_FUNC, "update address virt=%p, phy=0x%x, index=%d\n",
			pStrBufDesc, dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->str_index][uStrBufIdx], ctx->str_index);
	return ret;
}
static int update_stream_addr_vpu(struct vpu_ctx *ctx, void *input_buffer, uint32_t buffer_size, uint32_t uStrBufIdx)
{
	int size = 0;

	mutex_lock(&ctx->instance_mutex);
	size = update_stream_addr(ctx, input_buffer, buffer_size, uStrBufIdx);
	mutex_unlock(&ctx->instance_mutex);

	return size;
}

static void fill_stream_buffer_info(struct vpu_ctx *ctx)
{
	pDEC_RPC_HOST_IFACE pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	pBUFFER_INFO_TYPE buffer_info = &pSharedInterface->StreamBuffInfo[ctx->str_index];

	if (!ctx)
		return;

	if (ctx->start_code_bypass)
		buffer_info->stream_input_mode = NON_FRAME_LVL;
	else
		buffer_info->stream_input_mode = FRAME_LVL;

	buffer_info->stream_pic_input_count = ctx->frm_total_num;
}

static void set_pic_end_flag(struct vpu_ctx *ctx)
{
	pDEC_RPC_HOST_IFACE pSharedInterface;
	pBUFFER_INFO_TYPE buffer_info;

	if (!ctx)
		return;

	pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	buffer_info = &pSharedInterface->StreamBuffInfo[ctx->str_index];

	if (buffer_info->stream_input_mode == FRAME_LVL)
		buffer_info->stream_pic_end_flag = 0x1;
}

static void clear_pic_end_flag(struct vpu_ctx *ctx)
{
	pDEC_RPC_HOST_IFACE pSharedInterface;
	pBUFFER_INFO_TYPE buffer_info;

	if (!ctx)
		return;

	pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	buffer_info = &pSharedInterface->StreamBuffInfo[ctx->str_index];

	if (buffer_info->stream_input_mode == FRAME_LVL)
		buffer_info->stream_pic_end_flag = 0x0;
}

//warn uStrIdx need to refine how to handle it
static void v4l2_update_stream_addr(struct vpu_ctx *ctx, uint32_t uStrBufIdx)
{
	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_SRC];
	void *input_buffer;
	uint32_t buffer_size;

	down(&This->drv_q_lock);
	while (!list_empty(&This->drv_q)) {
		if (vpu_frm_depth != INVALID_FRAME_DEPTH) //frame depth need to be set by user and then the condition works
			if (ctx->frm_dec_delay >= vpu_frm_depth)
				break;
		p_data_req = list_first_entry(&This->drv_q,
				typeof(*p_data_req), list);

		if (!p_data_req->vb2_buf)
			break;

		buffer_size = p_data_req->vb2_buf->planes[0].bytesused;
		input_buffer = (void *)vb2_plane_vaddr(p_data_req->vb2_buf, 0);
		vpu_dec_receive_ts(ctx, p_data_req->vb2_buf, buffer_size);
		if (!update_stream_addr_vpu(ctx, input_buffer, buffer_size, uStrBufIdx)) {
			up(&This->drv_q_lock);
			vpu_dbg(LVL_INFO, " %s no space to write\n", __func__);
			return;
		} else {
			if (ctx->b_dis_reorder) {
				/* frame successfully written into the stream buffer if in special low latency mode
					mark that this frame should be flushed for decode immediately */
				add_scode(ctx, 0, BUFFLUSH_PADDING_TYPE, true);
				record_log_info(ctx, LOG_PADDING, 0, 0);
			}
			ctx->frm_dec_delay++;
			ctx->frm_dis_delay++;
			ctx->frm_total_num++;
			record_log_info(ctx, LOG_UPDATE_STREAM, 0, buffer_size);
			fill_stream_buffer_info(ctx);
		}
#ifdef HANDLE_EOS
		if (buffer_size < p_data_req->vb2_buf->planes[0].length)
			vpu_dbg(LVL_INFO, "v4l2_transfer_buffer_to_firmware - set stream_feed_complete - DEBUG 2\n");
#endif
		list_del(&p_data_req->list);
		p_data_req->queued = false;
		if (p_data_req->vb2_buf)
			vb2_buffer_done(p_data_req->vb2_buf,
					VB2_BUF_STATE_DONE);
	}
	if (list_empty(&This->drv_q) && ctx->eos_stop_received) {
		if (vpu_dec_is_active(ctx)) {
			vpu_dbg(LVL_EVENT, "ctx[%d]: insert eos directly\n", ctx->str_index);
			if (add_scode(ctx, 0, EOS_PADDING_TYPE, true) >= 0) {
				record_log_info(ctx, LOG_EOS, 0, 0);
				ctx->eos_stop_received = false;
				ctx->eos_stop_added = true;
			}
		} else	{
			vpu_dbg(LVL_ERR, "Firmware already stopped !\n");
		}
	}
	up(&This->drv_q_lock);
}

static void report_buffer_done(struct vpu_ctx *ctx, void *frame_info)
{
	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_DST];
	u_int32 *FrameInfo = (u_int32 *)frame_info;
	u_int32 fs_id = FrameInfo[0x0];
	uint32_t stride = FrameInfo[3];
	bool b10BitFormat = is_10bit_format(ctx);
	int buffer_id;

	vpu_dbg(LVL_BIT_FUNC, "%s() fs_id=%d, ulFsLumaBase[0]=%x, stride=%d, b10BitFormat=%d, ctx->pSeqinfo->uBitDepthLuma=%d\n",
			__func__, fs_id, FrameInfo[1], stride, b10BitFormat, ctx->pSeqinfo->uBitDepthLuma);
	v4l2_update_stream_addr(ctx, 0);

	buffer_id = find_buffer_id(ctx, FrameInfo[1]);
	if (buffer_id == -1) {
		vpu_dbg(LVL_ERR, "%s() ctx[%d] not find buffer id: %d, addr: 0x%x\n",
				__func__, ctx->str_index, fs_id, FrameInfo[1]);
		return;
	}

	if (buffer_id != fs_id) {
		if (fs_id == MEDIA_PLAYER_SKIPPED_FRAME_ID) {
			send_skip_event(ctx);
			ctx->frm_dis_delay--;
			return;
		}
		vpu_dbg(LVL_ERR, "error: find buffer_id(%d) and firmware return id(%d) doesn't match\n",
				buffer_id, fs_id);
	}
	if (ctx->q_data[V4L2_DST].vb2_reqs[buffer_id].status != FRAME_DECODED)
		vpu_dbg(LVL_ERR, "error: buffer(%d) need to set FRAME_READY, but previous state %s is not FRAME_DECODED\n",
				buffer_id, bufstat[ctx->q_data[V4L2_DST].vb2_reqs[buffer_id].status]);

	set_data_req_status(&ctx->q_data[V4L2_DST].vb2_reqs[buffer_id],
				FRAME_READY);
	ctx->frm_dis_delay--;
	vpu_dbg(LVL_INFO, "frame total: %d; depth: %d; delay: [dec, dis] = [%d, %d]\n", ctx->frm_total_num,
		vpu_frm_depth, ctx->frm_dec_delay, ctx->frm_dis_delay);

	down(&This->drv_q_lock);
	p_data_req = &This->vb2_reqs[buffer_id];
	if (p_data_req->vb2_buf) {
		p_data_req->vb2_buf->planes[0].bytesused = This->sizeimage[0];
		p_data_req->vb2_buf->planes[1].bytesused = This->sizeimage[1];
		if (p_data_req->vb2_buf->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(p_data_req->vb2_buf,
					VB2_BUF_STATE_DONE);
		else
			vpu_dbg(LVL_ERR, "warning: wait_rst_done(%d) check buffer(%d) state(%d)\n",
					ctx->wait_rst_done, buffer_id, p_data_req->vb2_buf->state);
	}
	up(&This->drv_q_lock);
	vpu_dbg(LVL_INFO, "leave %s\n", __func__);
}

/*
 * this is used for waiting the right status buffer in the queue list
 * true means find right buffer, false means not
 */
static bool wait_right_buffer(struct queue_data *This)
{
	struct vb2_data_req *p_data_req, *p_temp;

	down(&This->drv_q_lock);
	if (!list_empty(&This->drv_q)) {
		list_for_each_entry_safe(p_data_req, p_temp, &This->drv_q, list)
			if (p_data_req->status == FRAME_ALLOC
					|| p_data_req->status == FRAME_RELEASE) {
				up(&This->drv_q_lock);
				if (verify_frame_buffer_size(This, p_data_req))
					return true;
				else
					return false;
			}
	}
	up(&This->drv_q_lock);

	return false;
}

static void send_skip_event(struct vpu_ctx* ctx)
{
	const struct v4l2_event ev = {
		.type = V4L2_EVENT_SKIP
	};

	if (!ctx)
		return;
	v4l2_event_queue_fh(&ctx->fh, &ev);
}

static void reset_mbi_dcp_count(struct vpu_ctx *ctx)
{
	ctx->mbi_count = 0;
	ctx->dcp_count = 0;
}

static bool verify_frame_buffer_size(struct queue_data *q_data,
							struct vb2_data_req *p_data_req)
{
	uint32_t size_0 = 0;
	uint32_t size_1 = 0;
	struct vpu_ctx *ctx = container_of(q_data, struct vpu_ctx,
							q_data[V4L2_DST]);

	if (!q_data || !p_data_req)
		return false;

	size_0 = p_data_req->vb2_buf->planes[0].length
				- p_data_req->vb2_buf->planes[0].data_offset;
	size_1 = p_data_req->vb2_buf->planes[1].length
		- p_data_req->vb2_buf->planes[1].data_offset;
	if (size_0 >= q_data->sizeimage[0] && size_1 >= q_data->sizeimage[1])
		return true;

	vpu_dbg(LVL_WARN, "warning: %s() ctx[%d] frame buffer size is smaller than need\n",
			__func__, ctx->str_index);
	return false;
}

static u32 get_consumed_pic_bytesused(struct vpu_ctx *ctx,
					u32 pic_start_addr,
					u32 pic_end_addr)
{
	u32 consumed_pic_bytesused;
	u32 pic_size;

	consumed_pic_bytesused = pic_end_addr +
				+ ctx->stream_buffer.dma_size
				- ctx->pre_pic_end_addr;
	consumed_pic_bytesused %= ctx->stream_buffer.dma_size;
	pic_size = pic_end_addr
		+ ctx->stream_buffer.dma_size
		- pic_start_addr;
	pic_size %= ctx->stream_buffer.dma_size;

	vpu_dbg(LVL_BIT_PIC_ADDR, "<0x%08x 0x%08x>, %8d, %8d\n",
		pic_start_addr, pic_end_addr, pic_size, consumed_pic_bytesused);
	/*
	 *WARN_ON(consumed_pic_bytesused < pic_size);
	 */
	if (consumed_pic_bytesused < pic_size)
		vpu_dbg(LVL_ERR,
			"****Error:[%d] invalid uPicStartAddr or uPicEndAddr\n",
			ctx->str_index);

	ctx->total_consumed_bytes += consumed_pic_bytesused;

	ctx->pre_pic_end_addr = pic_end_addr;

	return consumed_pic_bytesused;
}

static int parse_frame_interval_from_seqinfo(struct vpu_ctx *ctx,
					MediaIPFW_Video_SeqInfo *seq_info)
{
	u32 gcd;

	ctx->frame_interval.numerator = 1000;
	ctx->frame_interval.denominator = seq_info->FrameRate;
	if (!seq_info->FrameRate) {
		vpu_dbg(LVL_INFO, "unknown FrameRate(%d)\n",
				seq_info->FrameRate);
		return -EINVAL;
	}

	gcd = get_greatest_common_divisor(ctx->frame_interval.numerator,
					ctx->frame_interval.denominator);
	ctx->frame_interval.numerator /= gcd;
	ctx->frame_interval.denominator /= gcd;

	mutex_lock(&ctx->instance_mutex);
	vpu_dec_set_tsm_frame_rate(ctx);
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static void vpu_api_event_handler(struct vpu_ctx *ctx, u_int32 uStrIdx, u_int32 uEvent, u_int32 *event_data)
{
	struct vpu_dev *dev;
	pDEC_RPC_HOST_IFACE pSharedInterface;
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;

	vpu_log_event(uEvent, uStrIdx);
	count_event(&ctx->statistic, uEvent);

	pStrBufDesc = get_str_buffer_desc(ctx);
	record_log_info(ctx, LOG_EVENT, uEvent, pStrBufDesc->rptr);

	if (ctx == NULL) {
		vpu_dbg(LVL_ERR, "receive event: 0x%X after instance released, ignore it\n", uEvent);
		return;
	}

	if (ctx->firmware_stopped && uEvent != VID_API_EVENT_START_DONE) {
		vpu_dbg(LVL_ERR, "receive event: 0x%X after stopped, ignore it\n", uEvent);
		return;
	}
	dev = ctx->dev;
	pSharedInterface = (pDEC_RPC_HOST_IFACE)dev->shared_mem.shared_mem_vir;

	switch (uEvent) {
	case VID_API_EVENT_START_DONE:
		ctx->firmware_stopped = false;
		ctx->firmware_finished = false;
		break;
	case VID_API_EVENT_STOPPED: {
		vpu_dbg(LVL_INFO, "receive VID_API_EVENT_STOPPED\n");
		ctx->firmware_stopped = true;
		ctx->start_flag = true;
		complete(&ctx->completion);//reduce possibility of abort hang if decoder enter stop automatically
		complete(&ctx->stop_cmp);
		}
		break;
	case VID_API_EVENT_RESET_DONE:
		break;
	case VID_API_EVENT_PIC_DECODED: {
		MediaIPFW_Video_QMeterInfo *pQMeterInfo = (MediaIPFW_Video_QMeterInfo *)dev->shared_mem.qmeter_mem_vir;
		MediaIPFW_Video_PicInfo *pPicInfo = (MediaIPFW_Video_PicInfo *)dev->shared_mem.pic_mem_vir;
		MediaIPFW_Video_PicDispInfo *pDispInfo = &pPicInfo[uStrIdx].DispInfo;
		MediaIPFW_Video_PicPerfInfo *pPerfInfo = &pPicInfo[uStrIdx].PerfInfo;
		MediaIPFW_Video_PicPerfDcpInfo *pPerfDcpInfo = &pPicInfo[uStrIdx].PerfDcpInfo;
		int buffer_id;
		u_int32 uDecFrmId = event_data[7];
		u_int32 uPicStartAddr = event_data[10];
		u_int32 uPicEndAddr = event_data[12];
		struct queue_data *This = &ctx->q_data[V4L2_DST];
		u_int32 uDpbmcCrc;
		size_t wr_size;
		u32 consumed_pic_bytesused = 0;
		struct vb2_data_req *p_data_req = NULL;

		if (ctx->buffer_null == true) {
			vpu_dbg(LVL_INFO, "frame already released\n");
			break;
		}

		if (This->vdec_std == VPU_VIDEO_HEVC)
			uDpbmcCrc = pPerfDcpInfo->uDBEDpbCRC[0];
		else
			uDpbmcCrc = pPerfInfo->uMemCRC;
		if (vpu_frmcrcdump_ena) {
			wr_size = kernel_write(ctx->crc_fp, &uDpbmcCrc, sizeof(uDpbmcCrc), &ctx->pos);
			ctx->pos += wr_size;
		}

		vpu_dbg(LVL_INFO, "PICINFO GET: uPicType:%d uPicStruct:%d uPicStAddr:0x%x uFrameStoreID:%d uPercentInErr:%d, uRbspBytesCount=%d, ulLumBaseAddr[0]=%x, pQMeterInfo:%p, pPicInfo:%p, pDispInfo:%p, pPerfInfo:%p, pPerfDcpInfo:%p, uPicStartAddr=0x%x, uDpbmcCrc:%x\n",
				pPicInfo[uStrIdx].uPicType, pPicInfo[uStrIdx].uPicStruct,
				pPicInfo[uStrIdx].uPicStAddr, pPicInfo[uStrIdx].uFrameStoreID,
				pPicInfo[uStrIdx].uPercentInErr, pPerfInfo->uRbspBytesCount, event_data[0],
				pQMeterInfo, pPicInfo, pDispInfo, pPerfInfo, pPerfDcpInfo, uPicStartAddr, uDpbmcCrc);

		buffer_id = find_buffer_id(ctx, event_data[0]);
		if (buffer_id == -1) {
			vpu_dbg(LVL_ERR, "error: %s() ctx[%d] not find buffer id: %d, addr: 0x%x\n",
					__func__, ctx->str_index, uDecFrmId, event_data[0]);
			break;
		}

		if (buffer_id != uDecFrmId) {
			if (uDecFrmId == MEDIA_PLAYER_SKIPPED_FRAME_ID) {
				send_skip_event(ctx);
				ctx->frm_dec_delay--;
				break;
			}
			vpu_dbg(LVL_ERR, "error: VID_API_EVENT_PIC_DECODED address and id doesn't match\n");
		}
		p_data_req = &This->vb2_reqs[buffer_id];
		if (p_data_req->status != FRAME_FREE)
			vpu_dbg(LVL_ERR, "error: buffer(%d) need to set FRAME_DECODED, but previous state %s is not FRAME_FREE\n",
					buffer_id, bufstat[ctx->q_data[V4L2_DST].vb2_reqs[buffer_id].status]);
		set_data_req_status(p_data_req, FRAME_DECODED);
		if (ctx->pSeqinfo->uProgressive == 1)
			p_data_req->bfield = false;
		else
			p_data_req->bfield = true;

		if (tsm_use_consumed_length)
			consumed_pic_bytesused = get_consumed_pic_bytesused(ctx,
							uPicStartAddr,
							uPicEndAddr);
		vpu_dbg(LVL_BIT_FRAME_BYTES, "Valid bytes : %d\n",
				consumed_pic_bytesused);
		TSManagerValid2(ctx->tsm,
				consumed_pic_bytesused,
				p_data_req->vb2_buf);
		}

		ctx->frm_dec_delay--;
		break;
	case VID_API_EVENT_SEQ_HDR_FOUND: {
		MediaIPFW_Video_SeqInfo *pSeqInfo = (MediaIPFW_Video_SeqInfo *)dev->shared_mem.seq_mem_vir;
//		MediaIPFW_Video_FrameBuffer *pStreamFrameBuffer = &pSharedInterface->StreamFrameBuffer[uStrIdx];
//		MediaIPFW_Video_FrameBuffer *pStreamDCPBuffer = &pSharedInterface->StreamDCPBuffer[uStrIdx];
		MediaIPFW_Video_PitchInfo   *pStreamPitchInfo = &pSharedInterface->StreamPitchInfo[uStrIdx];
		unsigned int num = pSharedInterface->SeqInfoTabDesc.uNumSizeDescriptors;

		if (ctx->pSeqinfo == NULL) {
			ctx->pSeqinfo = kzalloc(sizeof(MediaIPFW_Video_SeqInfo), GFP_KERNEL);
			atomic64_add(sizeof(MediaIPFW_Video_SeqInfo), &ctx->statistic.total_alloc_size);
		}
		else
			vpu_dbg(LVL_INFO, "pSeqinfo is not NULL, need not to realloc\n");
		memcpy(ctx->pSeqinfo, &pSeqInfo[ctx->str_index], sizeof(MediaIPFW_Video_SeqInfo));

		caculate_frame_size(ctx);
		parse_frame_interval_from_seqinfo(ctx, ctx->pSeqinfo);
		vpu_dbg(LVL_INFO, "SEQINFO GET: uHorRes:%d uVerRes:%d uHorDecodeRes:%d uVerDecodeRes:%d uNumDPBFrms:%d, num:%d, uNumRefFrms:%d, uNumDFEAreas:%d\n",
				ctx->pSeqinfo->uHorRes, ctx->pSeqinfo->uVerRes,
				ctx->pSeqinfo->uHorDecodeRes, ctx->pSeqinfo->uVerDecodeRes,
				ctx->pSeqinfo->uNumDPBFrms, num, ctx->pSeqinfo->uNumRefFrms, ctx->pSeqinfo->uNumDFEAreas);
		if (ctx->b_firstseq) {
			const struct v4l2_event ev = {
				.type = V4L2_EVENT_SOURCE_CHANGE,
				.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION
			};
			v4l2_event_queue_fh(&ctx->fh, &ev);

			pStreamPitchInfo->uFramePitch = 0x4000;
			ctx->b_firstseq = false;
		}
		}
		break;
	case VID_API_EVENT_PIC_HDR_FOUND:
		break;
	case VID_API_EVENT_REQ_FRAME_BUFF: {
		MEDIA_PLAYER_FSREQ *pFSREQ = (MEDIA_PLAYER_FSREQ *)event_data;
		u_int32 local_cmddata[10];
		struct vb2_data_req *p_data_req, *p_temp;
		struct queue_data *This = &ctx->q_data[V4L2_DST];
		u_int32 LumaAddr = 0;
		u_int32 ChromaAddr = 0;
		u_int32 *pphy_address;
		struct vb2_data_req;
		bool buffer_flag = false;

		vpu_dbg(LVL_INFO, "VID_API_EVENT_REQ_FRAME_BUFF, type=%d, size=%ld\n", pFSREQ->eType, sizeof(MEDIA_PLAYER_FSREQ));
		if (pFSREQ->eType == MEDIAIP_DCP_REQ) {
			if (ctx->dcp_count >= MAX_DCP_NUM) {
				vpu_dbg(LVL_ERR, "error: request dcp buffers number is over MAX_DCP_NUM\n");
				break;
			}
			if (alloc_dcp_buffer(ctx, ctx->dcp_count))
				break;

			local_cmddata[0] = ctx->dcp_count | (ctx->pSeqinfo->uActiveSeqTag<<24);
			local_cmddata[1] = ctx->dcp_buffer[ctx->dcp_count].dma_phy;
			local_cmddata[2] = DCP_SIZE;
			local_cmddata[3] = 0;
			local_cmddata[4] = 0;
			local_cmddata[5] = 0;
			local_cmddata[6] = pFSREQ->eType;
			v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
			vpu_dbg(LVL_INFO, "VID_API_CMD_FS_ALLOC, ctx[%d] eType=%d, index=%d\n",
					ctx->str_index, pFSREQ->eType, ctx->dcp_count);
			ctx->dcp_count++;
		} else if (pFSREQ->eType == MEDIAIP_MBI_REQ) {
			if (ctx->mbi_count >= ctx->mbi_num) {
				vpu_dbg(LVL_ERR, "error: request mbi buffers number(%d) is over allocted buffer number(%d)\n",
						ctx->mbi_count, ctx->mbi_num);
				break;
			}
			if (ctx->mbi_buffer[ctx->mbi_count].dma_size
					< ctx->mbi_size) {
				vpu_dbg(LVL_WARN, "warning: ctx[%d] alloced mbi buffer size is smaller than need size\n",
						ctx->str_index);
				break;
			}
			local_cmddata[0] = ctx->mbi_count | (ctx->pSeqinfo->uActiveSeqTag<<24);
			local_cmddata[1] = ctx->mbi_buffer[ctx->mbi_count].dma_phy;
			local_cmddata[2] = ctx->mbi_buffer[ctx->mbi_count].dma_size;
			local_cmddata[3] = 0;
			local_cmddata[4] = 0;
			local_cmddata[5] = 0;
			local_cmddata[6] = pFSREQ->eType;
			v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
			vpu_dbg(LVL_INFO, "VID_API_CMD_FS_ALLOC, ctx[%d] eType=%d, index=%d\n",
					ctx->str_index, pFSREQ->eType, ctx->mbi_count);
			ctx->mbi_count++;
		} else {
#if 1
			while (1) {
				if (!wait_event_interruptible_timeout(ctx->buffer_wq,
							((ctx->wait_rst_done == true) || (wait_right_buffer(This) == true)),
							msecs_to_jiffies(5000))) {
					vpu_dbg(LVL_ERR, " warn: wait_event_interruptible_timeout wait 5s timeout\n");
					vpu_log_buffer_state(ctx);
				} else
					break;
			}
#endif

			if (ctx->buffer_null == true) {
				vpu_dbg(LVL_INFO, "frame already released\n");
				break;
			}

			if (!list_empty(&This->drv_q) && !ctx->wait_rst_done) {
				down(&This->drv_q_lock);
				list_for_each_entry_safe(p_data_req, p_temp, &This->drv_q, list) {
					if (p_data_req->status == FRAME_ALLOC
							|| p_data_req->status == FRAME_RELEASE){
						if (!p_data_req->vb2_buf)
							break;
						pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 0);
						if (pphy_address != NULL)
							LumaAddr = *pphy_address + p_data_req->data_offset[0];
						pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 1);
						if (pphy_address != NULL)
							ChromaAddr = *pphy_address + p_data_req->data_offset[1];
						vpu_dbg(LVL_INFO, "%s :LumaAddr(%x) ChromaAddr(%x) buf_id (%d)\n",
								__func__,
								LumaAddr,
								ChromaAddr,
								p_data_req->id
								);
						if (!LumaAddr || !ChromaAddr) {
							LumaAddr = p_data_req->phy_addr[0] + p_data_req->data_offset[0];
							ChromaAddr = p_data_req->phy_addr[1] + p_data_req->data_offset[1];
						}

						local_cmddata[0] = p_data_req->id | (ctx->pSeqinfo->uActiveSeqTag<<24);
						local_cmddata[1] = LumaAddr;
						local_cmddata[2] = local_cmddata[1] + This->sizeimage[0]/2;
						local_cmddata[3] = ChromaAddr;
						local_cmddata[4] = local_cmddata[3] + This->sizeimage[1]/2;
						local_cmddata[5] = ctx->q_data[V4L2_DST].stride;
						local_cmddata[6] = pFSREQ->eType;
						//WARN :need to check the call back VID_API_EVENT_REL_FRAME_BUFF later, when it is received, the corepond id can be released, now just do a temporary workaround
						if (p_data_req->status == FRAME_RELEASE)
							v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_RELEASE, 1, local_cmddata);
						v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
						set_data_req_status(p_data_req,
								FRAME_FREE);
						vpu_dbg(LVL_INFO, "VID_API_CMD_FS_ALLOC, ctx[%d] data_req->vb2_buf=%p, data_req->id=%d\n",
								ctx->str_index, p_data_req->vb2_buf, p_data_req->id);
						list_del(&p_data_req->list);
						p_data_req->queued = false;
						buffer_flag = true;
						break;
					} else {
						vpu_dbg(LVL_INFO, "buffer %d status=0x%x is not right, find next\n", p_data_req->id, p_data_req->status);
						continue;
					}
				}
				up(&This->drv_q_lock);
				if (buffer_flag == false && !ctx->firmware_finished)
					vpu_dbg(LVL_ERR, "error: don't find the right buffer for VID_API_CMD_FS_ALLOC\n");
			} else if (ctx->wait_rst_done) {
				u_int32 i;

				if (ctx->firmware_finished)
					break;
				down(&This->drv_q_lock);
				for (i = 0; i < VPU_MAX_BUFFER; i++) {
					p_data_req = &This->vb2_reqs[i];
					if (p_data_req->status == FRAME_RELEASE)
						break;
				}
				if (i == VPU_MAX_BUFFER || !p_data_req->vb2_buf) {
					vpu_dbg(LVL_ERR, "error: don't find buffer when wait_rst_done is true, ctx->firmware_stopped=%dfin=%d\n", ctx->firmware_stopped, ctx->firmware_finished); //wait_rst_done is true when streamoff or v4l2_release is called
					up(&This->drv_q_lock);
					break;
				}
				if (!verify_frame_buffer_size(This, p_data_req)) {
					up(&This->drv_q_lock);
					break;
				}

				pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 0);
				if (pphy_address != NULL)
					LumaAddr = *pphy_address + p_data_req->data_offset[0];
				pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 1);
				if (pphy_address != NULL)
					ChromaAddr = *pphy_address + p_data_req->data_offset[1];
				vpu_dbg(LVL_INFO, "%s :LumaAddr(%x) ChromaAddr(%x) buf_id (%d)\n",
						__func__,
						LumaAddr,
						ChromaAddr,
						p_data_req->id
						);
				if (!LumaAddr || !ChromaAddr) {
					LumaAddr = p_data_req->phy_addr[0] + p_data_req->data_offset[0];
					ChromaAddr = p_data_req->phy_addr[1] + p_data_req->data_offset[1];
				}
				local_cmddata[0] = p_data_req->id | (ctx->pSeqinfo->uActiveSeqTag<<24);
				local_cmddata[1] = LumaAddr;
				local_cmddata[2] = local_cmddata[1] + This->sizeimage[0]/2;
				local_cmddata[3] = ChromaAddr;
				local_cmddata[4] = local_cmddata[3] + This->sizeimage[1]/2;
				local_cmddata[5] = ctx->q_data[V4L2_DST].stride;
				local_cmddata[6] = pFSREQ->eType;
				//WARN :need to check the call back VID_API_EVENT_REL_FRAME_BUFF later, when it is received, the corepond id can be released, now just do a temporary workaround
				if (p_data_req->status == FRAME_RELEASE)
					v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_RELEASE, 1, local_cmddata);
				v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
				set_data_req_status(p_data_req, FRAME_FREE);
				up(&This->drv_q_lock);
				vpu_dbg(LVL_INFO, "VID_API_CMD_FS_ALLOC, ctx[%d] data_req->vb2_buf=%p, data_req->id=%d\n",
						ctx->str_index, p_data_req->vb2_buf, p_data_req->id);
			} else
				vpu_dbg(LVL_ERR, "error: the list is still empty");
		}
		}
		break;
	case VID_API_EVENT_REL_FRAME_BUFF: {
		MEDIA_PLAYER_FSREL *fsrel = (MEDIA_PLAYER_FSREL *)event_data;
		struct queue_data *This = &ctx->q_data[V4L2_DST];
		struct vb2_data_req *p_data_req;

		if (ctx->buffer_null == true) {
			vpu_dbg(LVL_INFO, "frame already released !!!!!!!!!!!!!!!!!\n");
			break;
		}

		down(&This->drv_q_lock);
		if (fsrel->eType == MEDIAIP_FRAME_REQ) {
			p_data_req = &This->vb2_reqs[fsrel->uFSIdx];

			if (ctx->wait_rst_done != true && p_data_req->status != FRAME_READY) {
				const struct v4l2_event ev = {
					.type = V4L2_EVENT_SKIP
				};
				vpu_dbg(LVL_WARN, "warning: normal release and previous status %s, frame not for display, queue the buffer to list again\n",
						bufstat[p_data_req->status]);

				if ((p_data_req->status == FRAME_DECODED || p_data_req->status == FRAME_FREE)
						&& p_data_req->queued == false) {
					v4l2_event_queue_fh(&ctx->fh, &ev);
					list_add_tail(&p_data_req->list, &This->drv_q);
					p_data_req->queued = true;
				}
			}
			set_data_req_status(p_data_req, FRAME_RELEASE);
		} else if (fsrel->eType == MEDIAIP_MBI_REQ) {
			vpu_dbg(LVL_INFO, "ctx[%d] relase MEDIAIP_MBI_REQ frame[%d]\n",
					ctx->str_index, fsrel->uFSIdx);
		} else if (fsrel->eType == MEDIAIP_DCP_REQ) {
			vpu_dbg(LVL_INFO, "ctx[%d] relase MEDIAIP_DCP_REQ frame[%d]\n",
					ctx->str_index, fsrel->uFSIdx);
		} else {
			vpu_dbg(LVL_WARN, "warning: ctx[%d] release unknown type frame!\n",
					ctx->str_index);
		}
		up(&This->drv_q_lock);
		vpu_dbg(LVL_INFO, "VID_API_EVENT_REL_FRAME_BUFF uFSIdx=%d, eType=%d, size=%ld\n",
				fsrel->uFSIdx, fsrel->eType, sizeof(MEDIA_PLAYER_FSREL));
	} break;
	case VID_API_EVENT_FRAME_BUFF_RDY: {
		u_int32 *FrameInfo = (u_int32 *)event_data;

		//when the buffer is not NULL, do report_buffer_done
		if (ctx->buffer_null == false)
			report_buffer_done(ctx, FrameInfo);
	}
		break;
	case VID_API_EVENT_CHUNK_DECODED:
		break;
	case VID_API_EVENT_FIFO_LOW: {
		u_int32 uStrBufIdx = 0; //use buffer 0 for the stream

		if (ctx->buffer_null == true) {
			vpu_dbg(LVL_INFO, "frame already released !!!!!!!!!!!!!!!!!\n");
			break;
		}
		v4l2_update_stream_addr(ctx, uStrBufIdx);
	} break;
	case VID_API_EVENT_FIFO_HIGH:
		break;
	case  VID_API_EVENT_FIFO_EMPTY:
		break;
	case  VID_API_EVENT_FIFO_FULL:
		break;
	case  VID_API_EVENT_FIFO_OVF:
		break;
	case  VID_API_EVENT_BS_ERROR:
		break;
	case  VID_API_EVENT_UDATA_FIFO_UPTD:
		break;
	case VID_API_EVENT_DBG_STAT_UPDATE:
		break;
	case VID_API_EVENT_DBG_LOG_STARTED:
		break;
	case VID_API_EVENT_DBG_LOG_STOPPED:
		break;
	case VID_API_EVENT_ABORT_DONE: {
		pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;

		pStrBufDesc = get_str_buffer_desc(ctx);
		vpu_dbg(LVL_INFO, "%s AbrtDone StrBuf Curr, wptr(%x) rptr(%x) start(%x) end(%x)\n",
				__func__,
				pStrBufDesc->wptr,
				pStrBufDesc->rptr,
				pStrBufDesc->start,
				pStrBufDesc->end
				);

		ctx->pre_pic_end_addr = pStrBufDesc->rptr;
		vpu_dbg(LVL_BIT_FRAME_BYTES,
				"[%d]total bytes: %ld, %ld, %ld, %ld\n",
				ctx->str_index,
				ctx->total_qbuf_bytes,
				ctx->total_ts_bytes,
				ctx->total_write_bytes,
				ctx->total_consumed_bytes);
		update_wptr(ctx, pStrBufDesc, pStrBufDesc->rptr);
		clear_pic_end_flag(ctx);
		ctx->frm_dis_delay = 0;
		ctx->frm_dec_delay = 0;
		ctx->frm_total_num = 0;
		ctx->total_qbuf_bytes = 0;
		ctx->total_write_bytes = 0;
		ctx->total_consumed_bytes = 0;
		ctx->total_ts_bytes = 0;
		v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_RST_BUF, 0, NULL);
		}
		break;
	case VID_API_EVENT_RES_CHANGE: {
		struct queue_data *This;
		const struct v4l2_event ev = {
			.type = V4L2_EVENT_SOURCE_CHANGE,
			.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION
		};
		This = &ctx->q_data[V4L2_DST];
		down(&This->drv_q_lock);
		reset_mbi_dcp_count(ctx);
		ctx->mbi_size = get_mbi_size(This);
		up(&This->drv_q_lock);
		vpu_dbg(LVL_WARN, "warning: ctx[%d] VID_API_EVENT_RES_CHANGE, seq id: %d\n",
				ctx->str_index, ctx->pSeqinfo->uActiveSeqTag);
		vpu_log_buffer_state(ctx);
		v4l2_event_queue_fh(&ctx->fh, &ev);
		reinit_completion(&ctx->cap_streamon_cmp);
		while (1) {
			if (!wait_for_completion_timeout(&ctx->cap_streamon_cmp,
						msecs_to_jiffies(1000))) {
				if (ctx->ctx_released == true)
					break;
				vpu_dbg(LVL_WARN, "ctx[%d] warning: wait application capture port streamon timeout\n",
						ctx->str_index);
			} else {
				vpu_dbg(LVL_WARN, "ctx[%d] application capture port streamon done\n",
						ctx->str_index);
				break;
			}
		}
		}
		break;
	case VID_API_EVENT_STR_BUF_RST: {
		pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
		struct vb2_data_req *p_data_req;
		struct queue_data *This;
		u_int32 i;

		pStrBufDesc = get_str_buffer_desc(ctx);
		vpu_dbg(LVL_INFO, "%s wptr(%x) rptr(%x) start(%x) end(%x)\n",
				__func__,
				pStrBufDesc->wptr,
				pStrBufDesc->rptr,
				pStrBufDesc->start,
				pStrBufDesc->end
			  );
		ctx->wait_rst_done = false;
		This = &ctx->q_data[V4L2_DST];
		down(&This->drv_q_lock);
		for (i = 0; i < VPU_MAX_BUFFER; i++) {
			p_data_req = &This->vb2_reqs[i];
			if (p_data_req->vb2_buf != NULL)
				if (p_data_req->status != FRAME_RELEASE)
					vpu_dbg(LVL_INFO, "buffer(%d) status is %s when receive VID_API_EVENT_STR_BUF_RST\n",
							i, bufstat[p_data_req->status]);
		}
		up(&This->drv_q_lock);
		if (ctx->b_firstseq)
			v4l2_vpu_send_cmd(ctx, ctx->str_index,
					VID_API_CMD_STOP, 0, NULL);
		else
			complete(&ctx->completion);
		}
		break;
	case VID_API_EVENT_RET_PING:
		break;
	case VID_API_EVENT_STR_FMT_CHANGE:
		break;
	case VID_API_EVENT_FINISHED: {
		const struct v4l2_event ev = {
			.type = V4L2_EVENT_EOS
		};

		if (ctx->eos_stop_added == false)
			vpu_dbg(LVL_ERR, "warning: receive VID_API_EVENT_FINISHED before eos_stop_added set\n");
		ctx->eos_stop_added = false;
		if (ctx->firmware_finished == true)
			vpu_dbg(LVL_ERR, "warning: receive VID_API_EVENT_FINISHED when firmware_finished == true\n");
		ctx->firmware_finished = true;
		vpu_dbg(LVL_INFO, "receive VID_API_EVENT_FINISHED and notfiy app eos\n");
		clear_pic_end_flag(ctx);
		vpu_log_buffer_state(ctx);
		v4l2_event_queue_fh(&ctx->fh, &ev); //notfiy app stream eos reached

	}	break;
	case VID_API_EVENT_FIRMWARE_XCPT: {
		char *xcpt_info = (char*)event_data;
		const struct v4l2_event ev = {
			.type = V4L2_EVENT_EOS
		};
		vpu_dbg(LVL_ERR, "warning: VID_API_EVENT_FIRMWARE_XCPT,exception info: %s\n", xcpt_info);
		ctx->hang_status = true;
		v4l2_event_queue_fh(&ctx->fh, &ev);
		}
		break;
	case VID_API_EVENT_DEC_CFG_INFO:
		break;
	default:
		vpu_dbg(LVL_ERR, "warning: uEvent %d is not handled\n", uEvent);
		break;
	}
	vpu_dbg(LVL_INFO, "leave %s, uEvent %d\n", __func__, uEvent);
}



//This code is added for MU

static irqreturn_t fsl_vpu_mu_isr(int irq, void *This)
{
	struct vpu_dev *dev = This;
	u32 msg;

	MU_ReceiveMsg(dev->mu_base_virtaddr, 0, &msg);
	if (msg == 0xaa) {
		rpc_init_shared_memory(&dev->shared_mem,
				vpu_dec_cpu_phy_to_mu(dev, dev->m0_rpc_phy),
				dev->m0_rpc_virt,
				dev->m0_rpc_size);
		rpc_set_system_cfg_value(dev->shared_mem.pSharedInterface,
					VPU_REG_BASE);

		/*CM0 use relative address*/
		MU_sendMesgToFW(dev->mu_base_virtaddr,
				RPC_BUF_OFFSET,
				vpu_dec_cpu_phy_to_mu(dev,  dev->m0_rpc_phy));
		MU_sendMesgToFW(dev->mu_base_virtaddr,
				BOOT_ADDRESS,
				dev->m0_p_fw_space_phy);
		MU_sendMesgToFW(dev->mu_base_virtaddr, INIT_DONE, 2);

	} else if (msg == 0x55) {
		dev->firmware_started = true;
		complete(&dev->start_cmp);
	}  else if (msg == 0xA5) {
		/*receive snapshot done msg and wakeup complete to suspend*/
		complete(&dev->snap_done_cmp);
	} else
		schedule_work(&dev->msg_work);

	return IRQ_HANDLED;
}

/* Initialization of the MU code. */
static int vpu_mu_init(struct vpu_dev *dev)
{
	struct device_node *np;
	unsigned int	vpu_mu_id;
	u32 irq;
	int ret = 0;

	/*
	 * Get the address of MU to be used for communication with the M0 core
	 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx8-mu0-vpu-m0");
	if (!np) {
		vpu_dbg(LVL_ERR, "error: Cannot find MU entry in device tree\n");
		return -EINVAL;
	}
	dev->mu_base_virtaddr = of_iomap(np, 0);
	WARN_ON(!dev->mu_base_virtaddr);

	ret = of_property_read_u32_index(np,
				"fsl,vpu_ap_mu_id", 0, &vpu_mu_id);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: Cannot get mu_id %d\n", ret);
		return -EINVAL;
	}

	dev->vpu_mu_id = vpu_mu_id;

	irq = of_irq_get(np, 0);

	ret = devm_request_irq(&dev->plat_dev->dev, irq, fsl_vpu_mu_isr,
				IRQF_EARLY_RESUME, "vpu_mu_isr", (void *)dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: request_irq failed %d, error = %d\n", irq, ret);
		return -EINVAL;
	}

	if (!dev->vpu_mu_init) {
		MU_Init(dev->mu_base_virtaddr);
		MU_EnableRxFullInt(dev->mu_base_virtaddr, 0);
		dev->vpu_mu_init = 1;
	}

	return ret;
}

static void release_vpu_ctx(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;
	pm_runtime_put_sync(ctx->dev->generic_dev);
	kfree(ctx);
}

static int release_hang_instance(struct vpu_dev *dev)
{
	u_int32 i;

	if (!dev)
		return -EINVAL;

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++)
		if (dev->ctx[i]) {
			remove_instance_file(dev->ctx[i]);
			destroy_log_info_queue(dev->ctx[i]);
			if (atomic64_read(&dev->ctx[i]->statistic.total_alloc_size) != 0)
				vpu_dbg(LVL_ERR, "error: memory leak for vpu kalloc buffer\n");
			release_vpu_ctx(dev->ctx[i]);
			dev->ctx[i] = NULL;
		}

	return 0;
}

static int get_reset_index(struct vpu_dev *dev)
{
	int idx;

	for (idx = 0; idx < VPU_MAX_NUM_STREAMS; idx++)
		if (CHECK_BIT(dev->instance_mask, idx))
			break;

	return idx;
}

/*
 * Add judge to find if it has available path to decode, if all
 * path hang, reset vpu and then get one index
 */
static int vpu_next_free_instance(struct vpu_dev *dev)
{
	int idx;

	if (dev->hang_mask == dev->instance_mask
			&& dev->instance_mask != 0) {
		idx = get_reset_index(dev);
		if (idx < 0 || idx >= VPU_MAX_NUM_STREAMS)
			return -EBUSY;
		else {
			if (swreset_vpu_firmware(dev, idx))
				return -EINVAL;
			release_hang_instance(dev);
		}
		dev->hang_mask = 0;
		dev->instance_mask = 0;
	}

	idx = ffz(dev->instance_mask);
	if (idx < 0 || idx >= VPU_MAX_NUM_STREAMS)
		return -EBUSY;

	release_vpu_ctx(dev->ctx[idx]);

	return idx;
}

static void send_msg_queue(struct vpu_ctx *ctx, struct event_msg *msg)
{
	u_int32 ret;

	ret = kfifo_in(&ctx->msg_fifo, msg, sizeof(u_int32) * (MSG_WORD_LENGTH + msg->msgnum));
	if (ret != sizeof(u_int32) * (MSG_WORD_LENGTH + msg->msgnum))
		vpu_dbg(LVL_ERR, "There is no memory for msg fifo, ret=%d\n", ret);
}

static bool receive_msg_queue(struct vpu_ctx *ctx, struct event_msg *msg)
{
	u_int32 ret;

	if (kfifo_len(&ctx->msg_fifo) >= sizeof(u_int32) * MSG_WORD_LENGTH) {
		ret = kfifo_out(&ctx->msg_fifo, msg, sizeof(u_int32) * MSG_WORD_LENGTH);
		if (ret != sizeof(u_int32) * MSG_WORD_LENGTH) {
			vpu_dbg(LVL_ERR, "kfifo_out msg word has error, ret=%d\n", ret);
			return false;
		} else {
			if (msg->msgnum > 0) {
				if (kfifo_len(&ctx->msg_fifo) >= sizeof(u_int32) * msg->msgnum) {
					ret = kfifo_out(&ctx->msg_fifo, msg->msgdata, sizeof(u_int32) * msg->msgnum);
					if (ret != sizeof(u_int32) * msg->msgnum) {
						vpu_dbg(LVL_ERR, "kfifo_out msg data has error, ret=%d\n", ret);
						return false;
					} else
						return true;
				} else
					return false;
			} else
				return true;
		}
	} else
		return false;
}

extern u_int32 rpc_MediaIPFW_Video_message_check(struct shared_addr *This);
static void vpu_msg_run_work(struct work_struct *work)
{
	struct vpu_dev *dev = container_of(work, struct vpu_dev, msg_work);
	struct vpu_ctx *ctx;
	struct event_msg msg;
	struct shared_addr *This = &dev->shared_mem;

	if (!dev || !This)
		return;

	memset(&msg, 0, sizeof(struct event_msg));

	while (rpc_MediaIPFW_Video_message_check(This) == API_MSG_AVAILABLE) {
		rpc_receive_msg_buf(This, &msg);
		mutex_lock(&dev->dev_mutex);
		ctx = dev->ctx[msg.idx];
		if (ctx != NULL) {
			mutex_lock(&ctx->instance_mutex);
			if (!ctx->ctx_released) {
				send_msg_queue(ctx, &msg);
				queue_work(ctx->instance_wq, &ctx->instance_work);
			}
			mutex_unlock(&ctx->instance_mutex);
		}
		mutex_unlock(&dev->dev_mutex);
	}
	if (rpc_MediaIPFW_Video_message_check(This) == API_MSG_BUFFER_ERROR)
		vpu_dbg(LVL_ERR, "error: message size is too big to handle\n");
}
static void vpu_msg_instance_work(struct work_struct *work)
{
	struct vpu_ctx *ctx = container_of(work, struct vpu_ctx, instance_work);
	struct event_msg msg;

	if (!ctx)
		return;

	memset(&msg, 0, sizeof(struct event_msg));

	while (receive_msg_queue(ctx, &msg))
		vpu_api_event_handler(ctx, msg.idx, msg.msgid, msg.msgdata);
}

static int vpu_queue_setup(struct vb2_queue *vq,
		unsigned int *buf_count,
		unsigned int *plane_count,
		unsigned int psize[],
		struct device *allocators[])
{
	struct queue_data  *This = (struct queue_data *)vq->drv_priv;

	vpu_dbg(LVL_BIT_FUNC, "%s() is called\n", __func__);

	if ((vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) ||
		(vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		) {
		if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			*plane_count = 2;
			psize[0] = This->sizeimage[0];//check alignment
			psize[1] = This->sizeimage[1];//check colocated_size
		} else {
			psize[0] = This->sizeimage[0] + This->sizeimage[1];
			*plane_count = 1;
		}
	} else {
		*plane_count = 1;
		psize[0] = This->sizeimage[0];
	}
	return 0;
}

static int vpu_buf_prepare(struct vb2_buffer *vb)
{
	vpu_dbg(LVL_BIT_FUNC, "%s() is called\n", __func__);

	return 0;
}


static int vpu_start_streaming(struct vb2_queue *q,
		unsigned int count
		)
{
	int ret = 0;

	vpu_dbg(LVL_BIT_FUNC, "%s() is called\n", __func__);
	return ret;
}


static void vpu_stop_streaming(struct vb2_queue *q)
{
	struct queue_data *queue = (struct queue_data *)q->drv_priv;

	vpu_dbg(LVL_BIT_FUNC, "%s() is called\n", __func__);
	clear_queue(queue);
}

static void vpu_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue    *vq = vb->vb2_queue;
	struct queue_data   *This = (struct queue_data *)vq->drv_priv;
	struct vb2_data_req *data_req;
	u_int32 *pphy_address_0, *pphy_address_1;

	vpu_dbg(LVL_BIT_FUNC, "%s() is called\n", __func__);

	vpu_dbg(LVL_BIT_FUNC, "%s(), vq->type=%d, vb->index=%d\n",
			__func__, vq->type, vb->index);

	data_req = &This->vb2_reqs[vb->index];
	data_req->vb2_buf = vb;
	data_req->id = vb->index;
	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pphy_address_0 = (u_int32 *)vb2_plane_cookie(vb, 0);
		pphy_address_1 = (u_int32 *)vb2_plane_cookie(vb, 1);
		if (pphy_address_0 == NULL || pphy_address_1 == NULL) {
			vpu_dbg(LVL_WARN, "%s() warning: pphy_address == NULL\n",
					__func__);
			return;
		}
		data_req->phy_addr[0] = *pphy_address_0;
		data_req->phy_addr[1] = *pphy_address_1;
	}
	if (data_req->status != FRAME_FREE && data_req->status != FRAME_DECODED) {
		list_add_tail(&data_req->list, &This->drv_q);
		data_req->queued = true;
	} else {
	}
	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		v4l2_transfer_buffer_to_firmware(This, vb);
}

static void vpu_prepare(struct vb2_queue *q)
{
	vpu_dbg(LVL_BIT_FUNC, "%s() is called\n", __func__);
}

static void vpu_finish(struct vb2_queue *q)
{
	vpu_dbg(LVL_BIT_FUNC, "%s() is called\n", __func__);
}

struct vb2_ops v4l2_qops = {
	.queue_setup        = vpu_queue_setup,
	.wait_prepare       = vpu_prepare,
	.wait_finish        = vpu_finish,
	.buf_prepare        = vpu_buf_prepare,
	.start_streaming    = vpu_start_streaming,
	.stop_streaming     = vpu_stop_streaming,
	.buf_queue          = vpu_buf_queue,
};

static void init_vb2_queue(struct queue_data *This, unsigned int type, struct vpu_ctx *ctx)
{
	struct vb2_queue  *vb2_q = &This->vb2_q;
	int ret;
	u_int32 i;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	for (i = 0; i < VPU_MAX_BUFFER; i++)
		This->vb2_reqs[i].status = 0;
	// initialze driver queue
	INIT_LIST_HEAD(&This->drv_q);
	// initialize vb2 queue
	vb2_q->type = type;
	vb2_q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	vb2_q->gfp_flags = GFP_DMA32;
	vb2_q->ops = &v4l2_qops;
	vb2_q->drv_priv = This;
	vb2_q->mem_ops = (struct vb2_mem_ops *)&vb2_dma_contig_memops;
	vb2_q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	vb2_q->dev = &ctx->dev->plat_dev->dev;
	ret = vb2_queue_init(vb2_q);
	if (ret)
		vpu_dbg(LVL_ERR, "error: %s vb2_queue_init() failed (%d)!\n",
				__func__, ret);
	else
		This->vb2_q_inited = true;
}

static void init_queue_data(struct vpu_ctx *ctx)
{
	init_vb2_queue(&ctx->q_data[V4L2_SRC], V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, ctx);
	ctx->q_data[V4L2_SRC].type = V4L2_SRC;
	sema_init(&ctx->q_data[V4L2_SRC].drv_q_lock, 1);
	init_vb2_queue(&ctx->q_data[V4L2_DST], V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, ctx);
	ctx->q_data[V4L2_DST].type = V4L2_DST;
	sema_init(&ctx->q_data[V4L2_DST].drv_q_lock, 1);
}

static void release_queue_data(struct vpu_ctx *ctx)
{
	vpu_dec_queue_release(&ctx->q_data[V4L2_SRC]);
	vpu_dec_queue_release(&ctx->q_data[V4L2_DST]);
}

static void enable_csr_reg(struct vpu_dev *This)
{
	writel(This->m0_p_fw_space_phy, This->csr_base);
	writel(0x0, This->csr_base + 4);
}

static int vpu_firmware_download(struct vpu_dev *This)
{
	unsigned char *image;
	unsigned int FW_Size = 0;
	int ret = 0;
	char *p = This->m0_p_fw_space_vir;

	ret = request_firmware((const struct firmware **)&This->m0_pfw,
			M0FW_FILENAME,
			This->generic_dev
			);

	if (ret) {
		vpu_dbg(LVL_ERR, "error: %s() request fw %s failed(%d)\n",
			__func__, M0FW_FILENAME, ret);

		if (This->m0_pfw) {
			release_firmware(This->m0_pfw);
			This->m0_pfw = NULL;
		}
		return ret;
	} else {
		vpu_dbg(LVL_INFO, "%s() request fw %s got size(%d)\n",
			__func__, M0FW_FILENAME, (int)This->m0_pfw->size);
		image = (uint8_t *)This->m0_pfw->data;
		FW_Size = This->m0_pfw->size;
	}
	memcpy(This->m0_p_fw_space_vir,
			image,
			FW_Size
			);

	p[16] = This->plat_type;
	p[18] = 1;
	enable_csr_reg(This);

	return ret;
}

static int dbglog_show(struct seq_file *s, void *data)
{
#define DBG_UNIT_SIZE (7)

	struct vpu_ctx *ctx = (struct vpu_ctx *)s->private;
	u_int32 *pbuf;
	u_int32 i, line;
#if 0
	seq_printf(s, "dbg log buffer:\n");
#endif
	pbuf = (u_int32 *)ctx->dev->shared_mem.dbglog_mem_vir;
	line = (DBGLOG_SIZE) / (DBG_UNIT_SIZE * 4);
	for (i = 0; i < line; i++) {
#if 0
		seq_printf(s, "[%03d]:%08X %08X %08X %08X-%08X %08X %08X\n",
			i, pbuf[0], pbuf[1], pbuf[2], pbuf[3], pbuf[4], pbuf[5], pbuf[6]);
#else
		seq_printf(s, "%08x %08x %08x %08x %08x %08x %08x\n",
				pbuf[0], pbuf[1], pbuf[2], pbuf[3], pbuf[4], pbuf[5], pbuf[6]);
#endif
		pbuf += 7;
	}
	return 0;
}

static int dbglog_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, dbglog_show, inode->i_private);
}

static struct file_operations dbglog_fops = {
	.owner = THIS_MODULE,
	.open = dbglog_open,
	.read = seq_read,
};

static int create_instance_dbglog_file(struct vpu_ctx *ctx)
{
	if (ctx->dev->debugfs_root == NULL)
		ctx->dev->debugfs_root = debugfs_create_dir("vpu", NULL);

	scnprintf(ctx->dbglog_name, sizeof(ctx->dbglog_name) - 1,
			"instance%d",
			ctx->str_index);

	ctx->dbglog_dir = debugfs_create_dir(ctx->dbglog_name, ctx->dev->debugfs_root);
	if (!ctx->dbglog_dir) {
		vpu_dbg(LVL_ERR, "error: %s() ctx->dbglog_dir == NULL\n", __func__);
		return -EINVAL;
	}
	debugfs_create_file("dbglog", VERIFY_OCTAL_PERMISSIONS(0444),
		ctx->dbglog_dir, ctx, &dbglog_fops);

	return 0;
}


static ssize_t show_instance_command_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_ctx *ctx;
	struct vpu_statistic *statistic;
	int i, size, num = 0;

	ctx = container_of(attr, struct vpu_ctx, dev_attr_instance_command);
	statistic = &ctx->statistic;

	num += scnprintf(buf + num, PAGE_SIZE - num, "command number:\n");
	for (i = VID_API_CMD_NULL; i < VID_API_CMD_YUV_READY + 1; i++) {
		size = scnprintf(buf + num, PAGE_SIZE - num,
				"\t%40s(%2d):%16ld\n",
				cmd2str[i], i, statistic->cmd[i]);
		num += size;
	}

	num += scnprintf(buf + num, PAGE_SIZE - num, "\t%40s    :%16ld\n",
			"UNKNOWN COMMAND", statistic->cmd[VID_API_CMD_YUV_READY + 1]);

	num += scnprintf(buf + num, PAGE_SIZE - num, "current command:\n");
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"%10s:%40s;%10ld.%06ld\n", "command",
			get_cmd_str(statistic->current_cmd),
			statistic->ts_cmd.tv_sec,
			statistic->ts_cmd.tv_nsec / 1000);

	return num;
}

static ssize_t show_instance_event_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_ctx *ctx;
	struct vpu_statistic *statistic;
	int i, size, num = 0;

	ctx = container_of(attr, struct vpu_ctx, dev_attr_instance_event);
	statistic = &ctx->statistic;

	num += scnprintf(buf + num, PAGE_SIZE - num, "event number:\n");
	for (i = VID_API_EVENT_NULL; i < VID_API_EVENT_DEC_CFG_INFO + 1; i++) {
		size = scnprintf(buf + num, PAGE_SIZE - num,
				"\t%40s(%2d):%16ld\n",
				event2str[i], i, statistic->event[i]);
		num += size;
	}

	num += scnprintf(buf + num, PAGE_SIZE - num, "\t%40s    :%16ld\n",
			"UNKNOWN EVENT",
			statistic->event[VID_API_EVENT_DEC_CFG_INFO + 1]);

	num += scnprintf(buf + num, PAGE_SIZE - num, "current event:\n");
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"%10s:%40s;%10ld.%06ld\n", "event",
			get_event_str(statistic->current_event),
			statistic->ts_event.tv_sec,
			statistic->ts_event.tv_nsec / 1000);

	return num;
}

static ssize_t show_instance_buffer_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_ctx *ctx;
	struct vpu_statistic *statistic;
	struct vb2_data_req *p_data_req;
	struct queue_data *This;
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	u_int32 stream_length = 0;
	int i, size, num = 0;

	ctx = container_of(attr, struct vpu_ctx, dev_attr_instance_buffer);
	statistic = &ctx->statistic;

	num += scnprintf(buf + num, PAGE_SIZE - num, "frame buffer status:\n");

	This = &ctx->q_data[V4L2_DST];
	down(&This->drv_q_lock);
	for (i = 0; i < VPU_MAX_BUFFER; i++) {
		p_data_req = &This->vb2_reqs[i];
		if (p_data_req->vb2_buf != NULL) {
			size = scnprintf(buf + num, PAGE_SIZE - num,
					"\t%40s(%2d):%16s:%d\n",
					"buffer",
					i,
					bufstat[p_data_req->status],
					p_data_req->vb2_buf->state);
			num += size;
		}
	}
	up(&This->drv_q_lock);

	num += scnprintf(buf + num, PAGE_SIZE - num, "stream buffer status:\n");

	pStrBufDesc = get_str_buffer_desc(ctx);

	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16x\n", "wptr", pStrBufDesc->wptr);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16x\n", "rptr", pStrBufDesc->rptr);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16x\n", "start", pStrBufDesc->start);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16x\n", "end", pStrBufDesc->end);
	stream_length = ctx->stream_buffer.dma_size -
		got_free_space(pStrBufDesc->wptr, pStrBufDesc->rptr, pStrBufDesc->start, pStrBufDesc->end);

	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16x\n", "stream length", stream_length);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16x\n", "decode dealy frame", ctx->frm_dec_delay);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16x\n", "display delay frame", ctx->frm_dis_delay);

	return num;
}

static ssize_t show_instance_log_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_ctx *ctx;
	struct vpu_statistic *statistic;
	struct vpu_log_info *vpu_info;
	struct vpu_log_info *tem_info;
	int num = 0;

	ctx = container_of(attr, struct vpu_ctx, dev_attr_instance_flow);
	statistic = &ctx->statistic;

	num += scnprintf(buf + num, PAGE_SIZE - num, "log info under depth: %d\n",
			vpu_log_depth);

	mutex_lock(&ctx->instance_mutex);
	if (list_empty(&ctx->log_q))
		goto exit;

	list_for_each_entry_safe(vpu_info, tem_info, &ctx->log_q, list) {
		switch (vpu_info->type) {
		case LOG_EVENT:
			num += scnprintf(buf + num, PAGE_SIZE - num,
				"\t%20s:%40s %20s:%20x\n", "event", get_event_str(vpu_info->log_info[vpu_info->type]),
				"rptr", vpu_info->data);
			break;
		case LOG_COMMAND:
			num += scnprintf(buf + num, PAGE_SIZE - num,
				"\t%20s:%40s\n", "command", get_cmd_str(vpu_info->log_info[vpu_info->type]));
			break;
		case LOG_EOS:
			num += scnprintf(buf + num, PAGE_SIZE - num,
				"\t%20s:%40s\n", "add eos", "done");
			break;
		case LOG_PADDING:
			num += scnprintf(buf + num, PAGE_SIZE - num,
				"\t%20s:%40s\n", "add padding", "done");
			break;
		case LOG_UPDATE_STREAM:
			num += scnprintf(buf + num, PAGE_SIZE - num,
				"\t%20s:%40s %16d\n", "update stream data", "stream size", vpu_info->data);
			break;
		default:
			break;
		}
	}

exit:
	mutex_unlock(&ctx->instance_mutex);
	return num;
}

static int create_instance_command_file(struct vpu_ctx *ctx)
{
	scnprintf(ctx->command_name, sizeof(ctx->command_name) - 1,
			"instance%d_command",
			ctx->str_index);
	sysfs_attr_init(&ctx->dev_attr_instance_command.attr);
	ctx->dev_attr_instance_command.attr.name = ctx->command_name;
	ctx->dev_attr_instance_command.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
	ctx->dev_attr_instance_command.show = show_instance_command_info;

	device_create_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_command);

	return 0;
}

static int create_instance_event_file(struct vpu_ctx *ctx)
{
	scnprintf(ctx->event_name, sizeof(ctx->event_name) - 1,
			"instance%d_event",
			ctx->str_index);
	sysfs_attr_init(&ctx->dev_attr_instance_event.attr);
	ctx->dev_attr_instance_event.attr.name = ctx->event_name;
	ctx->dev_attr_instance_event.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
	ctx->dev_attr_instance_event.show = show_instance_event_info;

	device_create_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_event);

	return 0;
}

static int create_instance_buffer_file(struct vpu_ctx *ctx)
{
	scnprintf(ctx->buffer_name, sizeof(ctx->buffer_name) - 1,
			"instance%d_buffer",
			ctx->str_index);
	sysfs_attr_init(&ctx->dev_attr_instance_buffer.attr);
	ctx->dev_attr_instance_buffer.attr.name = ctx->buffer_name;
	ctx->dev_attr_instance_buffer.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
	ctx->dev_attr_instance_buffer.show = show_instance_buffer_info;

	device_create_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_buffer);

	return 0;
}

static int create_instance_flow_file(struct vpu_ctx *ctx)
{
	scnprintf(ctx->flow_name, sizeof(ctx->flow_name) - 1,
			"instance%d_flow",
			ctx->str_index);
	sysfs_attr_init(&ctx->dev_attr_instance_flow.attr);
	ctx->dev_attr_instance_flow.attr.name = ctx->flow_name;
	ctx->dev_attr_instance_flow.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
	ctx->dev_attr_instance_flow.show = show_instance_log_info;

	device_create_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_flow);

	return 0;
}
static int create_instance_file(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->dev || !ctx->dev->generic_dev)
		return -EINVAL;

	create_instance_command_file(ctx);
	create_instance_event_file(ctx);
	create_instance_buffer_file(ctx);
	create_instance_dbglog_file(ctx);
	create_instance_flow_file(ctx);
	atomic64_set(&ctx->statistic.total_dma_size, 0);
	atomic64_set(&ctx->statistic.total_alloc_size, 0);

	return 0;
}

static int remove_instance_file(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->dev || !ctx->dev->generic_dev)
		return -EINVAL;

	device_remove_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_command);
	device_remove_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_event);
	device_remove_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_buffer);
	if (ctx->dbglog_dir != NULL) {
		debugfs_remove_recursive(ctx->dbglog_dir);
		ctx->dbglog_dir = NULL;
	}
	device_remove_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_flow);

	return 0;
}

static int init_vpu_buffer(struct vpu_ctx *ctx)
{
	u_int32 i;

	if (!ctx)
		return -EINVAL;

	for (i = 0; i < MAX_DCP_NUM; i++)
		init_dma_buffer(&ctx->dcp_buffer[i]);
	ctx->dcp_count = 0;
	for (i = 0; i < MAX_MBI_NUM; i++)
		init_dma_buffer(&ctx->mbi_buffer[i]);
	ctx->mbi_count = 0;
	ctx->mbi_num = 0;
	ctx->mbi_size = 0;
	init_dma_buffer(&ctx->stream_buffer);
	init_dma_buffer(&ctx->udata_buffer);

	return 0;
}

static int alloc_vpu_buffer(struct vpu_ctx *ctx)
{
	u_int32 ret = 0;

	if (!ctx)
		return -EINVAL;

	if (!ctx->stream_buffer.dma_phy) {
		ctx->stream_buffer.dma_size = vpu_max_bufsize;
		ret = alloc_dma_buffer(ctx, &ctx->stream_buffer);
		if (ret) {
			vpu_dbg(LVL_ERR, "error: alloc stream buffer fail!\n");
			return ret;
		}
	}

	if (!ctx->udata_buffer.dma_phy) {
		ctx->udata_buffer.dma_size = UDATA_BUFFER_SIZE;
		ret = alloc_dma_buffer(ctx, &ctx->udata_buffer);
		if (ret) {
			vpu_dbg(LVL_ERR, "error: alloc udata buffer fail!\n");
			free_dma_buffer(ctx, &ctx->stream_buffer);
			return ret;
		}
	}

	return 0;
}

static int open_crc_file(struct vpu_ctx *ctx)
{
	char crc_file[64];
	int ret = 0;

	if (!ctx)
		return -EINVAL;

	scnprintf(crc_file, sizeof(crc_file) - 1,
			"/home/instance%d_crc.txt",
			ctx->str_index);
	ctx->crc_fp = filp_open(crc_file, O_RDWR | O_CREAT, 0664);
	if (IS_ERR(ctx->crc_fp)) {
		vpu_dbg(LVL_ERR, "error: open crc file fail\n");
		ret = -1;
	}
	ctx->pos = 0;

	return ret;
}

static int close_crc_file(struct vpu_ctx *ctx)
{
	int ret = 0;

	if (!ctx)
		return -EINVAL;

	if (!IS_ERR(ctx->crc_fp))
		ret = filp_close(ctx->crc_fp, NULL);
	ctx->pos = 0;

	return ret;
}

static bool vpu_dec_is_active(struct vpu_ctx *ctx)
{
	if (!ctx)
		return false;
	if (ctx->firmware_stopped)
		return false;
	if (ctx->start_flag)
		return false;

	return true;
}

static int v4l2_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_dev *dev = video_get_drvdata(vdev);
	struct vpu_ctx *ctx = NULL;
	int idx;
	int ret = 0;

	pm_runtime_get_sync(dev->generic_dev);
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		pm_runtime_put_sync(dev->generic_dev);
		return -ENOMEM;
	}
	mutex_lock(&dev->dev_mutex);
	idx = vpu_next_free_instance(dev);
	if (idx < 0) {
		ret = idx;
		mutex_unlock(&dev->dev_mutex);
		goto err_find_index;
	}
	set_bit(idx, &dev->instance_mask);
	mutex_unlock(&dev->dev_mutex);
	init_completion(&ctx->completion);
	init_completion(&ctx->stop_cmp);
	init_completion(&ctx->eos_cmp);
	init_completion(&ctx->cap_streamon_cmp);

	v4l2_fh_init(&ctx->fh, video_devdata(filp));
	filp->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	ctx->ctrl_inited = false;
	ctrls_setup_decoder(ctx);
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;

	ctx->instance_wq = alloc_workqueue("vpu_instance", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!ctx->instance_wq) {
		vpu_dbg(LVL_ERR, "error: %s unable to alloc workqueue for ctx\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_wq;
	}
	INIT_WORK(&ctx->instance_work, vpu_msg_instance_work);

	mutex_init(&ctx->instance_mutex);
	if (kfifo_alloc(&ctx->msg_fifo,
				sizeof(struct event_msg) * VID_API_MESSAGE_LIMIT,
				GFP_KERNEL)) {
		vpu_dbg(LVL_ERR, "fail to alloc fifo when open\n");
		ret = -ENOMEM;
		goto err_alloc_fifo;
	}
	ctx->dev = dev;
	ctx->str_index = idx;
	dev->ctx[idx] = ctx;
	ctx->b_firstseq = true;
	ctx->start_flag = true;
	ctx->wait_rst_done = false;
	ctx->firmware_stopped = false;
	ctx->firmware_finished = false;
	ctx->eos_stop_received = false;
	ctx->eos_stop_added = false;
	ctx->buffer_null = true; //this flag is to judge whether the buffer is null is not, it is used for the workaround that when send stop command still can receive buffer ready event, and true means buffer is null, false not
	ctx->ctx_released = false;
	ctx->b_dis_reorder = false;
	ctx->start_code_bypass = false;
	ctx->hang_status = false;
	ctx->tsm = createTSManager(tsm_buffer_size);
	if (!ctx->tsm)
		goto err_create_tsm;
	resyncTSManager(ctx->tsm, 0, tsm_mode);
	ctx->tsm_sync_flag = false;
	create_instance_file(ctx);
	if (vpu_frmcrcdump_ena) {
		ret = open_crc_file(ctx);
		if (ret)
			goto err_open_crc;
	}
	ctx->pSeqinfo = kzalloc(sizeof(MediaIPFW_Video_SeqInfo), GFP_KERNEL);
	if (!ctx->pSeqinfo) {
		vpu_dbg(LVL_ERR, "error: pSeqinfo alloc fail\n");
		ret = -ENOMEM;
		goto err_alloc_seq;
	}
	atomic64_add(sizeof(MediaIPFW_Video_SeqInfo), &ctx->statistic.total_alloc_size);
	ctx->pSeqinfo->uProgressive = 1;
	init_queue_data(ctx);
	init_log_info_queue(ctx);
	create_log_info_queue(ctx, vpu_log_depth);
	init_waitqueue_head(&ctx->buffer_wq);
	mutex_lock(&dev->dev_mutex);
	if (!dev->fw_is_ready) {
		ret = vpu_firmware_download(dev);
		if (ret) {
			vpu_dbg(LVL_ERR, "error: vpu_firmware_download fail\n");
			mutex_unlock(&dev->dev_mutex);
			goto err_firmware_load;
		}
		vpu_dbg(LVL_INFO, "done: vpu_firmware_download\n");
		if (!ctx->dev->firmware_started)
			if (!wait_for_completion_timeout(&ctx->dev->start_cmp, msecs_to_jiffies(10000))) {
				vpu_dbg(LVL_ERR, "error: don't get start interrupt\n");
				ret = -1;
				goto err_firmware_load;
			}
		dev->fw_is_ready = true;
	}
	mutex_unlock(&dev->dev_mutex);
	rpc_set_stream_cfg_value(dev->shared_mem.pSharedInterface, ctx->str_index, vpu_dbe_num);
	init_vpu_buffer(ctx);

	return 0;

err_firmware_load:
	destroy_log_info_queue(ctx);
	kfree(ctx->pSeqinfo);
	ctx->pSeqinfo = NULL;
	atomic64_sub(sizeof(MediaIPFW_Video_SeqInfo), &ctx->statistic.total_alloc_size);
	release_queue_data(ctx);
err_alloc_seq:
	if (vpu_frmcrcdump_ena)
		close_crc_file(ctx);
err_open_crc:
	if (ctx->tsm)
		destroyTSManager(ctx->tsm);
	ctx->tsm = NULL;
err_create_tsm:
	remove_instance_file(ctx);
	kfifo_free(&ctx->msg_fifo);
err_alloc_fifo:
	destroy_workqueue(ctx->instance_wq);
err_alloc_wq:
	ctrls_delete_decoder(ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	clear_bit(ctx->str_index, &dev->instance_mask);
err_find_index:
	if (atomic64_read(&ctx->statistic.total_alloc_size) != 0)
		vpu_dbg(LVL_ERR, "error: memory leak for vpu kalloc buffer\n");
	kfree(ctx);
	pm_runtime_put_sync(dev->generic_dev);
	dev->ctx[idx] = NULL;

	return ret;
}

static int v4l2_release(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_dev *dev = video_get_drvdata(vdev);
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);

	vpu_dbg(LVL_BIT_FUNC,
		"ctx[%d]: %s() - %s, %s, %s, total frame: %d\n",
		ctx->str_index,
		__func__,
		ctx->firmware_stopped ? "stopped" : "not stopped",
		ctx->firmware_finished ? "finished" : "not finished",
		ctx->eos_stop_added ? "eos_added" : "not eos_added",
		ctx->frm_total_num);

	if (vpu_dec_is_active(ctx)) {
		ctx->wait_rst_done = true;
		wake_up_interruptible(&ctx->buffer_wq);  //workaround: to wakeup event handler who still may receive request frame after reset done
		vpu_dbg(LVL_INFO, "v4l2_release() - send VID_API_CMD_STOP\n");
		v4l2_vpu_send_cmd(ctx, ctx->str_index, VID_API_CMD_STOP, 0, NULL);
		if (!wait_for_completion_timeout(&ctx->stop_cmp, msecs_to_jiffies(1000))) {
			ctx->hang_status = true;
			vpu_dbg(LVL_ERR, "the path id:%d firmware hang after send VID_API_CMD_STOP\n", ctx->str_index);
		}
	} else
		vpu_dbg(LVL_INFO, "v4l2_release() - stopped(%d): skip VID_API_CMD_STOP\n", ctx->firmware_stopped);

	mutex_lock(&ctx->instance_mutex);
	ctx->ctx_released = true;
	kfifo_free(&ctx->msg_fifo);
	cancel_work_sync(&ctx->instance_work);
	if (ctx->instance_wq)
		destroy_workqueue(ctx->instance_wq);
	mutex_unlock(&ctx->instance_mutex);

	if (ctx->tsm) {
		destroyTSManager(ctx->tsm);
		ctx->tsm = NULL;
	}
	if (vpu_frmcrcdump_ena)
		close_crc_file(ctx);
	release_queue_data(ctx);
	ctrls_delete_decoder(ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	free_decoder_buffer(ctx);
	if (atomic64_read(&ctx->statistic.total_dma_size) != 0)
		vpu_dbg(LVL_ERR, "error: memory leak for vpu dma alloc buffer\n");
	if (ctx->pSeqinfo) {
		kfree(ctx->pSeqinfo);
		ctx->pSeqinfo = NULL;
		atomic64_sub(sizeof(MediaIPFW_Video_SeqInfo), &ctx->statistic.total_alloc_size);
	}

	if (!ctx->hang_status) { // judge the path is hang or not, if hang, don't clear
		remove_instance_file(ctx);
		destroy_log_info_queue(ctx);
		if (atomic64_read(&ctx->statistic.total_alloc_size) != 0)
			vpu_dbg(LVL_ERR, "error: memory leak for vpu kalloc buffer\n");
		mutex_lock(&dev->dev_mutex);
		clear_bit(ctx->str_index, &dev->instance_mask);
		dev->ctx[ctx->str_index] = NULL;
		release_vpu_ctx(ctx);

		mutex_unlock(&dev->dev_mutex);
	} else {
		mutex_lock(&dev->dev_mutex);
		set_bit(ctx->str_index, &dev->hang_mask);
		mutex_unlock(&dev->dev_mutex);
	}

	return 0;
}

static unsigned int v4l2_poll(struct file *filp, poll_table *wait)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);
	struct vb2_queue *src_q, *dst_q;
	unsigned int rc = 0;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (ctx) {
		poll_wait(filp, &ctx->fh.wait, wait);

		if (v4l2_event_pending(&ctx->fh)) {
			vpu_dbg(LVL_INFO, "%s() v4l2_event_pending\n", __func__);
			rc |= POLLPRI;
		}

		src_q = &ctx->q_data[V4L2_SRC].vb2_q;
		dst_q = &ctx->q_data[V4L2_DST].vb2_q;

		if (ctx->firmware_finished && !list_empty(&dst_q->done_list))
			rc = 0;

		if ((!src_q->streaming || list_empty(&src_q->queued_list))
				&& (!dst_q->streaming || list_empty(&dst_q->queued_list))) {
			rc |= POLLERR;
			return rc;
		}

		poll_wait(filp, &src_q->done_wq, wait);
		if (!list_empty(&src_q->done_list))
			rc |= POLLOUT | POLLWRNORM;
		poll_wait(filp, &dst_q->done_wq, wait);
		if (!list_empty(&dst_q->done_list))
			rc |= POLLIN | POLLRDNORM;
	} else
		rc = POLLERR;

	return rc;
}

static int v4l2_mmap(struct file *filp, struct vm_area_struct *vma)
{
	long ret = -EPERM;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	struct queue_data *q_data;
	enum QUEUE_TYPE type;

	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (ctx) {
		type = offset >> MMAP_BUF_TYPE_SHIFT;
		q_data = &ctx->q_data[type];

		offset &= ~MMAP_BUF_TYPE_MASK;
		offset = offset >> PAGE_SHIFT;
		vma->vm_pgoff = offset;
		ret = vpu_dec_queue_mmap(q_data, vma);
	}

	return ret;
}

static const struct v4l2_file_operations v4l2_decoder_fops = {
	.owner = THIS_MODULE,
	.open  = v4l2_open,
	.unlocked_ioctl = video_ioctl2,
	.release = v4l2_release,
	.poll = v4l2_poll,
	.mmap = v4l2_mmap,
};

static struct video_device v4l2_videodevice_decoder = {
	.name   = "vpu decoder",
	.fops   = &v4l2_decoder_fops,
	.ioctl_ops = &v4l2_decoder_ioctl_ops,
	.vfl_dir = VFL_DIR_M2M,
};

static void vpu_setup(struct vpu_dev *This)
{
	uint32_t read_data = 0;

	vpu_dbg(LVL_BIT_FUNC, "enter %s\n", __func__);
	writel(0x1, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	writel(0xffffffff, This->regs_base + 0x70190);
	writel(0xffffffff, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_XMEM_RESET_SET);

	writel(0xE, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	writel(0x7, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_SET);

	writel(0x1f, This->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_CLOCK_ENABLE_SET);
	writel(0xffffffff, This->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_RESET_SET);

	writel(0x102, This->regs_base + XMEM_CONTROL);

	read_data = readl(This->regs_base+0x70108);
	vpu_dbg(LVL_INFO, "%s read_data=%x\n", __func__, read_data);
}

static void vpu_reset(struct vpu_dev *This)
{
	vpu_dbg(LVL_BIT_FUNC, "enter %s\n", __func__);
	writel(0x7, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_CLR);
	writel(0xffffffff, This->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_RESET_CLR);
}

static int vpu_enable_hw(struct vpu_dev *This)
{
	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);
	vpu_setup(This);
	return 0;
}
static void vpu_disable_hw(struct vpu_dev *This)
{
	vpu_reset(This);
}

static int swreset_vpu_firmware(struct vpu_dev *dev, u_int32 idx)
{
	int ret = 0;

	vpu_dbg(LVL_WARN, "SWRESET: swreset_vpu_firmware\n");
	dev->firmware_started = false;

	v4l2_vpu_send_cmd(dev->ctx[idx], 0, VID_API_CMD_FIRM_RESET, 0, NULL);

	if (!wait_for_completion_timeout(&dev->start_cmp, msecs_to_jiffies(10000))) {
		vpu_dbg(LVL_ERR, "error: %s() fail\n", __func__);
		return -1;
	}
	dev->firmware_started = true;

	return ret;
}

static int parse_dt_info(struct vpu_dev *dev, struct device_node *np)
{
	u_int32 core_type;
	struct resource reserved_res;
	struct device_node *reserved_node;
	u_int32 csr_base;
	int ret;

	if (!dev || !np)
		return -EINVAL;

	ret = of_property_read_u32(np, "core_type", &core_type);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: Cannot get core num %d\n", ret);
		return -EINVAL;
	}
	if (core_type == 2)
		dev->plat_type = IMX8QM;
	else
		dev->plat_type = IMX8QXP;
	reserved_node = of_parse_phandle(np, "boot-region", 0);
	if (!reserved_node) {
		vpu_dbg(LVL_ERR, "error: boot-region of_parse_phandle error\n");
		return -ENODEV;
	}

	if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
		vpu_dbg(LVL_ERR, "error: boot-region of_address_to_resource error\n");
		return -EINVAL;
	}
	dev->m0_p_fw_space_phy = reserved_res.start;
	dev->m0_boot_size = reserved_res.end - reserved_res.start;
	reserved_node = of_parse_phandle(np, "rpc-region", 0);
	if (!reserved_node) {
		vpu_dbg(LVL_ERR, "error: rpc-region of_parse_phandle error\n");
		return -ENODEV;
	}

	if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
		vpu_dbg(LVL_ERR, "error: rpc-region of_address_to_resource error\n");
		return -EINVAL;
	}
	dev->m0_rpc_phy = reserved_res.start;
	dev->m0_rpc_size = reserved_res.end - reserved_res.start;

	ret = of_property_read_u32(np, "reg-csr", &csr_base);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: Cannot get csr offset %d\n", ret);
		return -EINVAL;
	}
	dev->csr_base = ioremap(csr_base, 8); //for csr0 offset and cpuwait

	return 0;
}

static int create_vpu_video_device(struct vpu_dev *dev)
{
	int ret;

	if (!dev)
		return -EINVAL;

	dev->pvpu_decoder_dev = video_device_alloc();
	if (!dev->pvpu_decoder_dev) {
		vpu_dbg(LVL_ERR, "video device alloc for decoder fail\n");
		return -ENOMEM;
	}
	strlcpy(dev->pvpu_decoder_dev->name,
			v4l2_videodevice_decoder.name,
			sizeof(dev->pvpu_decoder_dev->name));
	dev->pvpu_decoder_dev->fops = v4l2_videodevice_decoder.fops;
	dev->pvpu_decoder_dev->ioctl_ops = v4l2_videodevice_decoder.ioctl_ops;
	dev->pvpu_decoder_dev->release = video_device_release;
	dev->pvpu_decoder_dev->vfl_dir = v4l2_videodevice_decoder.vfl_dir;
	dev->pvpu_decoder_dev->v4l2_dev = &dev->v4l2_dev;

	video_set_drvdata(dev->pvpu_decoder_dev, dev);

	ret = video_register_device(dev->pvpu_decoder_dev,
			VFL_TYPE_GRABBER,
			DECODER_NODE_NUMBER);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: %s unable to register video decoder device\n",
				__func__
				);
		video_device_release(dev->pvpu_decoder_dev);
		dev->pvpu_decoder_dev = NULL;
		return ret;
	}

	return 0;
}

static int init_vpudev_parameters(struct vpu_dev *dev)
{
	int ret;

	if (!dev)
		return -EINVAL;

	mutex_init(&dev->dev_mutex);
	mutex_init(&dev->cmd_mutex);
	init_completion(&dev->start_cmp);
	init_completion(&dev->snap_done_cmp);
	dev->firmware_started = false;
	dev->hang_mask = 0;
	dev->instance_mask = 0;

	dev->fw_is_ready = false;

	ret = vpu_mu_init(dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: %s vpu mu init failed\n", __func__);
		return ret;
	}

	//firmware space for M0
	dev->m0_p_fw_space_vir = ioremap_wc(dev->m0_p_fw_space_phy,
			dev->m0_boot_size
			);
	if (!dev->m0_p_fw_space_vir) {
		vpu_dbg(LVL_ERR, "error: failed to remap space for M0 firmware\n");
		return -ENOMEM;
	}

	memset_io(dev->m0_p_fw_space_vir, 0, dev->m0_boot_size);

	dev->m0_rpc_virt = ioremap_wc(dev->m0_rpc_phy,
			dev->m0_rpc_size
			);
	if (!dev->m0_rpc_virt) {
		vpu_dbg(LVL_ERR, "error: failed to remap space for rpc shared memory\n");
		return -ENOMEM;
	}

	memset_io(dev->m0_rpc_virt, 0, dev->m0_rpc_size);

	return 0;
}

static int vpu_probe(struct platform_device *pdev)
{
	struct vpu_dev *dev;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	unsigned int mu_id;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->plat_dev = pdev;
	dev->generic_dev = get_device(&pdev->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->regs_base)) {
		vpu_dbg(LVL_ERR, "error: %s could not map regs_base\n", __func__);
		ret = PTR_ERR(dev->regs_base);
		goto err_put_dev;
	}

	ret = parse_dt_info(dev, np);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: %s parse device tree fail\n", __func__);
		goto err_dev_iounmap;
	}

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: %s unable to register v4l2 dev\n", __func__);
		goto err_dev_iounmap;
	}

	platform_set_drvdata(pdev, dev);

	ret = create_vpu_video_device(dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: %s create vpu video device fail\n", __func__);
		goto err_unreg_v4l2;
	}

	if (!dev->mu_ipcHandle) {
		ret = sc_ipc_getMuID(&mu_id);
		if (ret) {
			vpu_dbg(LVL_ERR, "error: --- sc_ipc_getMuID() cannot obtain mu id SCI error! (%d)\n", ret);
			goto err_rm_vdev;
		}

		ret = sc_ipc_open(&dev->mu_ipcHandle, mu_id);
		if (ret) {
			vpu_dbg(LVL_ERR, "error: --- sc_ipc_getMuID() cannot open MU channel to SCU error! (%d)\n", ret);
			goto err_rm_vdev;
		}
	}

	dev->workqueue = alloc_workqueue("vpu", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!dev->workqueue) {
		vpu_dbg(LVL_ERR, "error: %s unable to alloc workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_rm_vdev;
	}

	INIT_WORK(&dev->msg_work, vpu_msg_run_work);

	check_fuse_value(dev, formats_compressed_dec, ARRAY_SIZE(formats_compressed_dec));
	vpu_enable_hw(dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	ret = init_vpudev_parameters(dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: failed to init parameters for vpudev\n");
		goto err_poweroff;
	}

	pm_runtime_put_sync(&pdev->dev);

	return 0;

err_poweroff:
	destroy_workqueue(dev->workqueue);
	vpu_disable_hw(dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
err_rm_vdev:
	if (dev->pvpu_decoder_dev) {
		video_unregister_device(dev->pvpu_decoder_dev);
		dev->pvpu_decoder_dev = NULL;
	}
err_unreg_v4l2:
	v4l2_device_unregister(&dev->v4l2_dev);
err_dev_iounmap:
	if (dev->regs_base)
		iounmap(dev->regs_base);
err_put_dev:
	if (dev->generic_dev) {
		put_device(dev->generic_dev);
		dev->generic_dev = NULL;
	}

	return ret;
}

static int vpu_remove(struct platform_device *pdev)
{
	struct vpu_dev *dev = platform_get_drvdata(pdev);

	destroy_workqueue(dev->workqueue);
	if (dev->m0_p_fw_space_vir)
		iounmap(dev->m0_p_fw_space_vir);
	if (dev->m0_pfw) {
		release_firmware(dev->m0_pfw);
		dev->m0_pfw = NULL;
	}
	dev->m0_p_fw_space_vir = NULL;
	dev->m0_p_fw_space_phy = 0;
	dev->m0_rpc_virt = NULL;
	dev->m0_rpc_phy = 0;
	if (dev->shared_mem.shared_mem_vir)
		iounmap(dev->shared_mem.shared_mem_vir);
	dev->shared_mem.shared_mem_vir = NULL;
	dev->shared_mem.shared_mem_phy = 0;

	vpu_disable_hw(dev);
	pm_runtime_disable(&pdev->dev);

	if (video_get_drvdata(dev->pvpu_decoder_dev))
		video_unregister_device(dev->pvpu_decoder_dev);

	v4l2_device_unregister(&dev->v4l2_dev);
	return 0;
}

static int vpu_runtime_suspend(struct device *dev)
{
	return 0;
}

static int vpu_runtime_resume(struct device *dev)
{
	return 0;
}

static int find_first_available_instance(struct vpu_dev *dev)
{
	int strIdx, i;

	if (!dev)
		return -EINVAL;

	strIdx = (~dev->hang_mask) & (dev->instance_mask);

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		if (CHECK_BIT(strIdx, i)) {
			strIdx = i;
			break;
		}
	}

	return strIdx;
}

static void v4l2_vpu_send_snapshot(struct vpu_dev *dev)
{
	int strIdx;

	strIdx = find_first_available_instance(dev);
	if (strIdx >= 0 && strIdx < VPU_MAX_NUM_STREAMS)
		v4l2_vpu_send_cmd(dev->ctx[strIdx], strIdx, VID_API_CMD_SNAPSHOT, 0, NULL);
	else
		vpu_dbg(LVL_WARN, "warning: all path hang, need to reset\n");
}

static int vpu_suspend(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);

	if (vpudev->hang_mask != vpudev->instance_mask) {

		/*if there is an available device, send snapshot command to firmware*/
		v4l2_vpu_send_snapshot(vpudev);

		if (!wait_for_completion_timeout(&vpudev->snap_done_cmp, msecs_to_jiffies(1000))) {
			vpu_dbg(LVL_ERR, "error: wait for vpu decoder snapdone event timeout!\n");
			return -1;
		}
	}

	return 0;
}

static bool is_vpu_poweroff(struct vpu_dev *vpudev)
{
	void *mu_cr_addr;
	u_int32 mu_cr;

	if (!vpudev)
		return false;

	mu_cr_addr = vpudev->mu_base_virtaddr + 0x10000 + 0x24;
	mu_cr = readl_relaxed(mu_cr_addr);

	if (mu_cr == 0)// it mean M0+ is already power off/on
		return true;
	else
		return false;

}

static int resume_vpu_register(struct vpu_dev *vpudev)
{
	if (!vpudev)
		return -EINVAL;

	vpu_enable_hw(vpudev);
	MU_Init(vpudev->mu_base_virtaddr);
	MU_EnableRxFullInt(vpudev->mu_base_virtaddr, 0);

	return 0;
}

static int resume_from_vpu_poweroff(struct vpu_dev *vpudev)
{
	int ret = 0;

	enable_csr_reg(vpudev);
	/*wait for firmware resotre done*/
	if (!wait_for_completion_timeout(&vpudev->start_cmp, msecs_to_jiffies(1000))) {
		vpu_dbg(LVL_ERR, "error: wait for vpu decoder resume done timeout!\n");
		ret = -1;
	}

	return ret;
}

static int vpu_resume(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);
	int ret = 0;
	u_int32 idx;

	pm_runtime_get_sync(vpudev->generic_dev);

	resume_vpu_register(vpudev);

	if (vpudev->fw_is_ready == false)
		return 0;

	if (is_vpu_poweroff(vpudev))
		ret = resume_from_vpu_poweroff(vpudev);
	else if (vpudev->hang_mask != vpudev->instance_mask) {
		idx = get_reset_index(vpudev);
		if (idx < VPU_MAX_NUM_STREAMS)
			swreset_vpu_firmware(vpudev, idx);
		else
			return -EINVAL;
	}

	pm_runtime_put_sync(vpudev->generic_dev);

	return ret;
}

static const struct dev_pm_ops vpu_pm_ops = {
	SET_RUNTIME_PM_OPS(vpu_runtime_suspend, vpu_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(vpu_suspend, vpu_resume)
};

static const struct of_device_id vpu_of_match[] = {
	{ .compatible = "nxp,imx8qm-b0-vpudec", },
	{ .compatible = "nxp,imx8qxp-b0-vpudec", },
	{}
}
MODULE_DEVICE_TABLE(of, vpu_of_match);

static struct platform_driver vpu_driver = {
	.probe = vpu_probe,
	.remove = vpu_remove,
	.driver = {
		.name = "vpu-b0",
		.of_match_table = vpu_of_match,
		.pm = &vpu_pm_ops,
	},
};
module_platform_driver(vpu_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux VPU driver for Freescale i.MX/MXC");
MODULE_LICENSE("GPL");

module_param(vpu_dbg_level_decoder, int, 0644);
MODULE_PARM_DESC(vpu_dbg_level_decoder, "Debug level (0-3)");
module_param(vpu_frm_depth, int, 0644);
MODULE_PARM_DESC(vpu_frm_depth, "maximum frame number in data pool");
module_param(vpu_log_depth, int, 0644);
MODULE_PARM_DESC(vpu_log_depth, "maximum log number in queue(0-60)");
module_param(vpu_frmdbg_ena, int, 0644);
MODULE_PARM_DESC(vpu_frmdbg_ena, "enable firmware mask instance dbg log (bit N to mask instance N)");
module_param(vpu_frmdbg_level, int, 0644);
MODULE_PARM_DESC(vpu_frmdbg_level, "firmware debug level (0-2)");
module_param(vpu_max_bufsize, int, 0644);
MODULE_PARM_DESC(vpu_max_bufsize, "maximun stream buffer size");
module_param(vpu_dbe_num, int, 0644);
MODULE_PARM_DESC(vpu_dbe_num, "vpu dbe number(1-2)");
module_param(vpu_frmcrcdump_ena, int, 0644);
MODULE_PARM_DESC(vpu_frmcrcdump_ena, "enable frame crc dump(0-1)");
module_param(stream_buffer_threshold, int, 0644);
MODULE_PARM_DESC(stream_buffer_threshold, "stream buffer threshold");
module_param(tsm_mode, int, 0644);
MODULE_PARM_DESC(tsm_mode, "timestamp manager mode(0 : ai, 1 : fifo)");
module_param(tsm_buffer_size, int, 0644);
MODULE_PARM_DESC(tsm_buffer_size, "timestamp manager buffer size");
module_param(tsm_use_consumed_length, int, 0644);
MODULE_PARM_DESC(tsm_use_consumed_length, "timestamp manager use consumed length");
