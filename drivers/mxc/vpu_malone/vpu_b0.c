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
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>
#include <linux/imx_vpu.h>
#include "vpu_b0.h"
#include "insert_startcode.h"
#include "vpu_debug_log.h"
#include "vpu_pm.h"
#include "vpu_mu.h"

#ifndef TSM_TIMESTAMP_NONE
#define TSM_TIMESTAMP_NONE	-1L
#endif
#ifndef TSM_TS_IS_VALID
#define TSM_TS_IS_VALID(ts)	((ts) >= 0)
#endif

unsigned int vpu_dbg_level_decoder = LVL_WARN;
static int vpu_frm_depth = VPU_FRAME_DEPTH_DEFAULT;
static int vpu_log_depth = DEFAULT_LOG_DEPTH;
static int vpu_max_bufsize = MAX_BUFFER_SIZE;
static int vpu_frmdbg_ena = DEFAULT_FRMDBG_ENABLE;
static int vpu_frmdbg_level = DEFAULT_FRMDBG_LEVEL;
static int vpu_frmdbg_raw = 1;
static int vpu_dbe_num = 1;
static int vpu_frmcrcdump_ena;
static int stream_buffer_threshold = 0x10000;
static int precheck_show_bytes;
static int vpu_show_perf_ena;
static int vpu_show_perf_idx = (1 << VPU_MAX_NUM_STREAMS) - 1;
static int vpu_show_perf_ent;
static int vpu_datadump_ena;
static unsigned short frame_threshold[VPU_MAX_NUM_STREAMS];
module_param_array(frame_threshold, ushort, NULL, 0644);

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
static void send_skip_event(struct vpu_ctx* ctx);
static void reset_mbi_dcp_count(struct vpu_ctx *ctx);
static bool verify_frame_buffer_size(struct queue_data *q_data,
							struct vb2_data_req *p_data_req);
static void add_buffer_to_queue(struct queue_data *q_data, struct vb2_data_req *data_req);
static int send_abort_cmd(struct vpu_ctx *ctx);
static int send_stop_cmd(struct vpu_ctx *ctx);
static int vpu_dec_cmd_reset(struct vpu_ctx *ctx);
static void vpu_dec_event_decode_error(struct vpu_ctx *ctx);
static void vpu_calculate_performance(struct vpu_ctx *ctx, u_int32 uEvent, const char *str);
static void vpu_dec_cancel_work(struct vpu_dev *vpudev);
static void vpu_dec_alloc_mbi_dcp(struct vpu_ctx *ctx);
static bool is_vpu_poweroff(struct vpu_dev *vpudev);

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
	"VID_API_CMD_TS",
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
	"FRAME_SKIP",
};

static int alloc_vpu_buffer(struct vpu_ctx *ctx);
static bool vpu_dec_is_active(struct vpu_ctx *ctx);
static void respond_req_frame(struct vpu_ctx *ctx,
				struct queue_data *queue,
				bool abnormal);
static void release_frame_buffer(struct vpu_ctx *ctx,
				u32 uStrIdx,
				struct vb2_data_req *p_data_req);
static void send_eos_event(struct vpu_ctx *ctx);
static void release_queue_data(struct vpu_ctx *ctx);

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
	ktime_get_raw_ts64(&statistic->ts_event);
}

static void count_cmd(struct vpu_statistic *statistic, u32 cmdid)
{
	if (!statistic)
		return;

	if (cmdid < ARRAY_SIZE(cmd2str))
		statistic->cmd[cmdid]++;
	else
		statistic->cmd[VID_API_CMD_TS + 1]++;
	statistic->current_cmd = cmdid;
	ktime_get_raw_ts64(&statistic->ts_cmd);
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

static void find_pattern_from_vb(struct vpu_dev *dev, unsigned long index,
				struct vb2_buffer *vb, unsigned int plane_no)
{
	u8 *ptr = NULL;
	int ret;

	if (!dev || !dev->precheck_num)
		return;

	if (!vb || plane_no >= vb->num_planes)
		return;

	ptr = vb2_plane_vaddr(vb, plane_no);
	if (!ptr)
		return;

	ret = kmp_search(ptr,
			vb->planes[plane_no].bytesused,
			dev->precheck_pattern,
			dev->precheck_num,
			dev->precheck_next);
	if (ret < 0)
		return;
	vpu_dbg(LVL_WARN, "[%12ld]pattern(%s) found : %d\n",
			index,
			dev->precheck_content,
			ret);
}

static void show_beginning_of_data(unsigned long index,
				struct vb2_buffer *vb, unsigned int plane_no)
{
	u8 *pdata;
	u32 length;
	u32 bytes = 0;
	u32 show_count;
	char temp[1028];
	int i;
	int ret;

	if (!precheck_show_bytes)
		return;

	if (!vb || plane_no >= vb->num_planes)
		return;

	pdata = vb2_plane_vaddr(vb, plane_no);
	length = vb->planes[plane_no].bytesused;
	if (!pdata || !length)
		return;
	show_count = min_t(u32, precheck_show_bytes, length);
	for (i = 0; i < show_count; i++) {
		ret = scnprintf(temp + bytes,
				sizeof(temp) - bytes,
				"%s0x%02x",
				i ? " " : "",
				pdata[i]);
		if (ret == 0)
			break;
		bytes += ret;
		if (bytes >= sizeof(temp))
			break;
	}
	vpu_dbg(LVL_WARN, "[%12ld][%12d/%12d]%s\n", index, i, length, temp);
}

static void precheck_vb_data(struct vpu_ctx *ctx, struct vb2_buffer *vb)
{
	unsigned long index;

	if (!ctx || !vb)
		return;

	index = ctx->q_data[V4L2_SRC].qbuf_count;
	show_beginning_of_data(index, vb, 0);
	find_pattern_from_vb(ctx->dev, index, vb, 0);
}

static bool check_vb_is_changed(struct vb2_data_req *p_data_req, u32 pattern)
{
	u32 luma_addr;
	u32 *pphy_address;

	pphy_address = vb2_plane_cookie(p_data_req->vb2_buf, 0);
	luma_addr = *pphy_address + p_data_req->data_offset[0];
	if (luma_addr != pattern)
		return true;

	return false;
}

static int find_buffer_id(struct vpu_ctx *ctx, u_int32 addr)
{
	struct queue_data *This;
	struct vb2_data_req *p_data_req;
	u_int32 LumaAddr;
	u_int32 i;

	if (!ctx)
		return -1;

	This = &ctx->q_data[V4L2_DST];
	down(&This->drv_q_lock);
	for (i = 0; i < VPU_MAX_BUFFER; i++) {
		p_data_req = &This->vb2_reqs[i];

		if (!p_data_req->vb2_buf)
			continue;

		LumaAddr = p_data_req->phy_addr[0] + p_data_req->data_offset[0];
		if (LumaAddr == addr) {
			if (check_vb_is_changed(p_data_req, LumaAddr))
				vpu_err("ctx[%d] frame buffer[%d] is changed\n",
					ctx->str_index, i);
			break;
		}
	}
	up(&This->drv_q_lock);
	if (i == VPU_MAX_BUFFER) {
		vpu_err("error: ctx[%d] can't find id based on address(0x%x)\n",
			ctx->str_index, addr);
		return -1;
	}
	return i;
}

static u32 get_str_buffer_desc_offset(struct vpu_ctx *ctx)
{
	return DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * ctx->str_index;
}

pSTREAM_BUFFER_DESCRIPTOR_TYPE get_str_buffer_desc(struct vpu_ctx *ctx)
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
		.fourcc     = VPU_PIX_FMT_DIV3,
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
		.fourcc     = V4L2_PIX_FMT_HEVC,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_HEVC,
		.disable    = 0,
	},
	{
		.name       = "Xvid Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_XVID,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_ASP,
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

	return 0;
}

static bool is_10bit_format(struct vpu_ctx *ctx)
{
	WARN_ON(!ctx);
	if (ctx->seqinfo.uBitDepthLuma > 8)
		return true;
	if (ctx->seqinfo.uBitDepthChroma > 8)
		return true;
	return false;
}

static u32 vpu_dec_get_capture_fmt(struct vpu_ctx *ctx)
{
	WARN_ON(!ctx);
	if (is_10bit_format(ctx))
		return V4L2_PIX_FMT_NV12_10BIT;
	else
		return V4L2_PIX_FMT_NV12;
}

static u32 vpu_dec_get_capture_fmt_index(struct vpu_ctx *ctx)
{
	u32 i;
	u32 pixelformat;

	WARN_ON(!ctx);
	pixelformat = vpu_dec_get_capture_fmt(ctx);
	for (i = 0; i < ARRAY_SIZE(formats_yuv_dec); i++) {
		if (formats_yuv_dec[i].fourcc == pixelformat)
			return i;
	}

	return ARRAY_SIZE(formats_yuv_dec);
}

static int v4l2_ioctl_enum_fmt_vid_cap(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f
		)
{
	struct vpu_v4l2_fmt *fmt;
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	u32 fmt_cnt = ARRAY_SIZE(formats_yuv_dec);
	u32 index;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	index = f->index;
	q_data = &ctx->q_data[V4L2_DST];
	down(&q_data->drv_q_lock);
	if (!ctx->b_firstseq) {
		if (!f->index)
			index = vpu_dec_get_capture_fmt_index(ctx);
		else
			index = fmt_cnt;
	}
	up(&q_data->drv_q_lock);

	if (index >= fmt_cnt)
		return -EINVAL;

	fmt = &formats_yuv_dec[index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	vpu_dbg(LVL_INFO, "CAPTURE fmt[%d] %c%c%c%c\n",
			f->index,
			f->pixelformat & 0xff,
			(f->pixelformat >> 8) & 0xff,
			(f->pixelformat >> 16) & 0xff,
			(f->pixelformat >> 24) & 0xff);
	return 0;
}

static bool check_fmt_is_support(struct vpu_ctx *ctx, struct vpu_v4l2_fmt *fmt)
{
	pDEC_RPC_HOST_IFACE pSharedInterface;
	bool support;

	if (!ctx || !ctx->dev || !fmt)
		return false;

	if (fmt->disable)
		return false;

	pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	support = true;

	switch (fmt->fourcc) {
	case VPU_PIX_FMT_DIV3:
	case VPU_PIX_FMT_DIVX:
		if (!(pSharedInterface->FWVersion & VPU_DEC_FMT_DIVX_MASK))
			support = false;
		break;
	case VPU_PIX_FMT_RV:
		if (!(pSharedInterface->FWVersion & VPU_DEC_FMT_RV_MASK))
			support = false;
		break;
	default:
		break;
	}

	return support;
}

static int v4l2_ioctl_enum_fmt_vid_out(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct vpu_v4l2_fmt *fmt;
	u_int32 index = 0, i;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	if (f->index >= ARRAY_SIZE(formats_compressed_dec))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(formats_compressed_dec); i++) {
		fmt = &formats_compressed_dec[i];
		if (!check_fmt_is_support(ctx, fmt))
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
	vpu_dbg(LVL_INFO, "OUTPUT fmt[%d] %c%c%c%c\n",
			f->index,
			f->pixelformat & 0xff,
			(f->pixelformat >> 8) & 0xff,
			(f->pixelformat >> 16) & 0xff,
			(f->pixelformat >> 24) & 0xff);

	return 0;
}

static void calculate_frame_size(struct queue_data *q_data, u32 width, u32 height, bool is_10bit)
{
	u_int32 luma_size;
	u_int32 chroma_size;
	u_int32 chroma_height;

	q_data->width = ALIGN(width, 8);
	q_data->height = ALIGN(height, V4L2_NXP_FRAME_VERTICAL_ALIGN);

	width =  is_10bit ? (width + ((width + 3) >> 2)) : width;
	width = ALIGN(width, V4L2_NXP_FRAME_HORIZONTAL_ALIGN);
	q_data->stride = width;

	height = ALIGN(height, V4L2_NXP_FRAME_VERTICAL_ALIGN);
	chroma_height = height >> 1;

	luma_size = width * height;
	chroma_size = width * chroma_height;

	q_data->sizeimage[0] = luma_size;
	q_data->sizeimage[1] = chroma_size;
}

static void vdec_setup_capture_size(struct vpu_ctx *ctx)
{
	struct queue_data *q_data;

	q_data = &ctx->q_data[V4L2_DST];

	calculate_frame_size(q_data,
			ctx->seqinfo.uHorDecodeRes,
			ctx->seqinfo.uVerDecodeRes,
			is_10bit_format(ctx));
	q_data->width = ctx->seqinfo.uHorRes;
	q_data->height = ctx->seqinfo.uVerRes;
	if (ctx->seqinfo.uProgressive == 1)
		q_data->field = V4L2_FIELD_NONE;
	else
		q_data->field = V4L2_FIELD_INTERLACED;
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

static int vpu_dec_convert_color_iso_aspect_to_v4l2_aspect(struct vpu_ctx *ctx,
		u32 primaries, u32 transfer, u32 coeffs, u32 fullrange)
{
	if (primaries < ARRAY_SIZE(colorprimaries))
		ctx->colorspace = colorprimaries[primaries];

	if (transfer < ARRAY_SIZE(colortransfers))
		ctx->xfer_func = colortransfers[transfer];

	if (coeffs < ARRAY_SIZE(colormatrixcoefs))
		ctx->ycbcr_enc = colormatrixcoefs[coeffs];

	if (fullrange)
		ctx->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	else
		ctx->quantization = V4L2_QUANTIZATION_LIM_RANGE;

	return 0;
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
		down(&q_data->drv_q_lock);
		pix_mp->pixelformat = vpu_dec_get_capture_fmt(ctx);
		pix_mp->width = q_data->width;
		pix_mp->height = q_data->height;
		pix_mp->field = q_data->field;
		pix_mp->num_planes = 2;
		for (i = 0; i < pix_mp->num_planes; i++) {
			pix_mp->plane_fmt[i].bytesperline = q_data->stride;
			pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];
		}
		up(&q_data->drv_q_lock);
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		q_data = &ctx->q_data[V4L2_SRC];

		down(&q_data->drv_q_lock);
		pix_mp->width = q_data->width;
		pix_mp->height = q_data->height;
		pix_mp->field = V4L2_FIELD_NONE;
		pix_mp->num_planes = q_data->num_planes;
		for (i = 0; i < pix_mp->num_planes; i++) {
			pix_mp->plane_fmt[i].bytesperline = q_data->stride;
			pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];

		}
		pix_mp->pixelformat = q_data->fourcc;
		up(&q_data->drv_q_lock);
	} else
		return -EINVAL;

	down(&ctx->q_data[V4L2_DST].drv_q_lock);
	pix_mp->colorspace = ctx->colorspace;
	pix_mp->xfer_func = ctx->xfer_func;
	pix_mp->ycbcr_enc = ctx->ycbcr_enc;
	pix_mp->quantization = ctx->quantization;
	up(&ctx->q_data[V4L2_DST].drv_q_lock);

	vpu_dbg(LVL_BIT_FLOW, "%s g_fmt : %c%c%c%c %d x %d\n",
		V4L2_TYPE_IS_OUTPUT(f->type) ? "OUTPUT" : "CAPTURE",
		pix_mp->pixelformat & 0xff,
		(pix_mp->pixelformat >> 8) & 0xff,
		(pix_mp->pixelformat >> 16) & 0xff,
		(pix_mp->pixelformat >> 24) & 0xff,
		pix_mp->width,
		pix_mp->height);
	return 0;
}

static bool set_video_standard(struct vpu_ctx *ctx,
		struct queue_data *q_data,
		struct v4l2_format *f,
		struct vpu_v4l2_fmt *pformat_table,
		uint32_t table_size)
{
	unsigned int i;

	for (i = 0; i < table_size; i++) {
		if (pformat_table[i].fourcc == f->fmt.pix_mp.pixelformat) {
			if (!check_fmt_is_support(ctx, &pformat_table[i]))
				return false;
			q_data->vdec_std = pformat_table[i].vdec_std;
			q_data->num_planes = pformat_table[i].num_planes;
			q_data->fourcc = f->fmt.pix_mp.pixelformat;
		}
	}
	return true;
}

static void set_output_default_sizeimage(struct queue_data *q_data)
{
	u32 i;

	for (i = 0; i < q_data->num_planes; i++) {
		if (q_data->sizeimage[i])
			continue;
		q_data->sizeimage[i] = q_data->width * q_data->height;
	}
}

static int try_colorspace(struct vpu_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;

	down(&ctx->q_data[V4L2_DST].drv_q_lock);
	if (ctx->b_firstseq) {
		pix_mp->colorspace = V4L2_COLORSPACE_DEFAULT;
		pix_mp->xfer_func = V4L2_XFER_FUNC_DEFAULT;
		pix_mp->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		pix_mp->quantization = V4L2_QUANTIZATION_DEFAULT;
	} else {
		pix_mp->colorspace = ctx->colorspace;
		pix_mp->xfer_func = ctx->xfer_func;
		pix_mp->ycbcr_enc = ctx->ycbcr_enc;
		pix_mp->quantization = ctx->quantization;
	}
	up(&ctx->q_data[V4L2_DST].drv_q_lock);

	return 0;
}

static int set_colorspace(struct vpu_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;

	down(&ctx->q_data[V4L2_DST].drv_q_lock);
	if (ctx->b_firstseq) {
		ctx->colorspace = pix_mp->colorspace;
		ctx->xfer_func = pix_mp->xfer_func;
		ctx->ycbcr_enc = pix_mp->ycbcr_enc;
		ctx->quantization = pix_mp->quantization;
	}
	pix_mp->colorspace = ctx->colorspace;
	pix_mp->xfer_func = ctx->xfer_func;
	pix_mp->ycbcr_enc = ctx->ycbcr_enc;
	pix_mp->quantization = ctx->quantization;
	up(&ctx->q_data[V4L2_DST].drv_q_lock);

	return 0;
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

	vpu_dbg(LVL_BIT_FLOW, "%s s_fmt : %c%c%c%c %d x %d\n",
		V4L2_TYPE_IS_OUTPUT(f->type) ? "OUTPUT" : "CAPTURE",
		pix_mp->pixelformat & 0xff,
		(pix_mp->pixelformat >> 8) & 0xff,
		(pix_mp->pixelformat >> 16) & 0xff,
		(pix_mp->pixelformat >> 24) & 0xff,
		pix_mp->width,
		pix_mp->height);
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		q_data = &ctx->q_data[V4L2_DST];
		if (!set_video_standard(ctx, q_data, f, formats_yuv_dec, ARRAY_SIZE(formats_yuv_dec)))
			return -EINVAL;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		q_data = &ctx->q_data[V4L2_SRC];
		if (!set_video_standard(ctx, q_data, f, formats_compressed_dec, ARRAY_SIZE(formats_compressed_dec)))
			return -EINVAL;
	} else
		return -EINVAL;

	set_colorspace(ctx, f);
	pix_mp->num_planes = q_data->num_planes;

	down(&q_data->drv_q_lock);
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		for (i = 0; i < q_data->num_planes; i++) {
			q_data->stride = pix_mp->plane_fmt[i].bytesperline;
			q_data->sizeimage[i] = pix_mp->plane_fmt[i].sizeimage;
		}
		q_data->width = pix_mp->width;
		q_data->height = pix_mp->height;
		set_output_default_sizeimage(q_data);
	} else if (ctx->b_firstseq) {
		calculate_frame_size(q_data, pix_mp->width, pix_mp->height, false);
		for (i = 0; i < q_data->num_planes; i++) {
			if (q_data->stride < pix_mp->plane_fmt[i].bytesperline)
				q_data->stride = pix_mp->plane_fmt[i].bytesperline;
			else
				pix_mp->plane_fmt[i].bytesperline = q_data->stride;
			if (q_data->sizeimage[i] < pix_mp->plane_fmt[i].sizeimage)
				q_data->sizeimage[i] = pix_mp->plane_fmt[i].sizeimage;
			else
				pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];
		}
	} else {
		for (i = 0; i < q_data->num_planes; i++) {
			pix_mp->plane_fmt[i].bytesperline = q_data->stride;
			pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];
		}
		pix_mp->width = q_data->width;
		pix_mp->height = q_data->height;
	}
	up(&q_data->drv_q_lock);

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		vpu_calculate_performance(ctx, 0xff, "capture set fmt");
	else
		vpu_calculate_performance(ctx, 0xff, "output set fmt");

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
	struct vpu_ctx *ctx;
	int ret = -EINVAL;

	ctx = container_of(queue, struct vpu_ctx, q_data[queue->type]);

	down(&queue->drv_q_lock);
	vpu_dbg(LVL_BIT_FLOW, "ctx[%d] %s %s buffers\n",
			ctx->str_index,
			queue->type ? "CAPTURE" : "OUTPUT",
			reqbuf->count ? "request" : "free");

	if (queue->type == V4L2_SRC)
		vpu_calculate_performance(ctx, 0xff, reqbuf->count ? "output request buffer begin" : "output free buffer begin");
	else
		vpu_calculate_performance(ctx, 0xff, reqbuf->count ? "capture request buffer begin" : "capture free buffer begin");

	if (queue->vb2_q_inited)
		ret = vb2_reqbufs(&queue->vb2_q, reqbuf);

	vpu_dbg(LVL_BIT_FLOW, "ctx[%d] %s %s buffers done\n",
			ctx->str_index,
			queue->type ? "CAPTURE" : "OUTPUT",
			reqbuf->count ? "request" : "free");
	if (queue->type == V4L2_SRC)
		vpu_calculate_performance(ctx, 0xff, reqbuf->count ? "output request buffer finish" : "output free buffer finish");
	else
		vpu_calculate_performance(ctx, 0xff, reqbuf->count ? "capture request buffer finish" : "capture free buffer finish");
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
	struct vb2_buffer *vb;

	if (buf->index >= queue->vb2_q.num_buffers) {
		vpu_err("[%s] buffer index(%d) out of range\n",
				queue->type ? "CAPTURE" : "OUTPUT", buf->index);
		return -EINVAL;
	}
	vb = queue->vb2_q.bufs[buf->index];
	if (vb->state != VB2_BUF_STATE_DEQUEUED) {
		vpu_err("[%s] buffer[%d] has been queued before\n",
				queue->type ? "CAPTURE" : "OUTPUT", buf->index);
		return -EINVAL;
	}

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

static int vpu_dec_queue_dqbuf(struct queue_data *queue,
				struct v4l2_buffer *buf, bool nonblocking)
{
	int ret = -EINVAL;

	down(&queue->drv_q_lock);
	if (queue->vb2_q_inited)
		ret = vb2_dqbuf(&queue->vb2_q, buf, nonblocking);
	if (!ret)
		queue->dqbuf_count++;
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

static void check_queue_is_releasd(struct queue_data *queue, char *desc)
{
	struct vb2_data_req *p_data_req = NULL;
	int i;

	for (i = 0; i < queue->vb2_q.num_buffers; i++) {
		p_data_req = &queue->vb2_reqs[i];
		if (!p_data_req->vb2_buf)
			continue;

		if (p_data_req->status != FRAME_ALLOC)
			vpu_dbg(LVL_WARN,
				"%s:buffer(%d) status is %s when %s\n",
				queue->type ? "CAPTURE" : "OUTPUT",
				i, bufstat[p_data_req->status],
				desc);
	}
}

static void clear_queue(struct queue_data *queue)
{
	struct vb2_data_req *p_data_req = NULL;
	struct vb2_data_req *p_temp;
	struct vb2_buffer *vb;
	struct vpu_ctx *ctx;

	vpu_dbg(LVL_BIT_FUNC, "%s() is called\n", __func__);
	ctx = container_of(queue, struct vpu_ctx, q_data[queue->type]);
	check_queue_is_releasd(queue, "clear queue");

	list_for_each_entry_safe(p_data_req, p_temp, &queue->drv_q, list) {
		list_del(&p_data_req->list);
		p_data_req->queued = false;
	}

	list_for_each_entry(vb, &queue->vb2_q.queued_list, queued_entry) {
		if (vb->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	}
	INIT_LIST_HEAD(&queue->drv_q);
	if (queue->type == V4L2_SRC)
		ctx->eos_stop_received = false;
	vpu_dbg(LVL_BIT_FRAME_COUNT,
		"%s qbuf_count : %ld, dqbuf_count : %ld\n",
		queue->type == V4L2_DST ? "CAPTURE" : "OUTPUT",
		queue->qbuf_count,
		queue->dqbuf_count);
	queue->qbuf_count = 0;
	queue->dqbuf_count = 0;
	queue->process_count = 0;
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
	vpu_dbg(LVL_BIT_FUNC, "%s(), type: 0x%x\n", __func__, sub->type);

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SKIP:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	case V4L2_EVENT_CODEC_ERROR:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	default:
		return -EINVAL;
	}
}

static void vpu_dec_cleanup_event(struct vpu_ctx *ctx)
{
	struct v4l2_event ev;
	int ret;

	while (v4l2_event_pending(&ctx->fh)) {
		ret = v4l2_event_dequeue(&ctx->fh, &ev, 1);
		if (ret)
			break;
	};
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
	if (!ctx || !ctx->dev || !buffer) {
		vpu_dec_event_decode_error(ctx);
		return -EINVAL;
	}

	buffer->dma_virt = dma_alloc_coherent(&ctx->dev->plat_dev->dev,
			buffer->dma_size,
			(dma_addr_t *)&buffer->dma_phy,
			GFP_KERNEL | GFP_DMA32 | __GFP_NOWARN);

	if (!buffer->dma_virt) {
		vpu_err("error: %s() dma buffer alloc size(%x) fail!\n",
				__func__,  buffer->dma_size);
		vpu_dec_event_decode_error(ctx);
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

#define DCP_FIXED_MB_ALLOC_TYPICAL	250
#define DCP_FIXED_MB_ALLOC_WORSTCASE	400
#define DCP_FIXED_MB_ALLOC		DCP_FIXED_MB_ALLOC_TYPICAL
static u32 get_dcp_size(struct vpu_ctx *ctx)
{
	u32 uNumDcpChunks = ((ctx->seqinfo.uNumDFEAreas >> 16) & 0xff);
	u32 uNumMbs;
	u32 width;
	u32 height;
	u32 uTotalBinSize;

	if (!uNumDcpChunks)
		return DCP_SIZE;

	width = ctx->seqinfo.uHorDecodeRes;
	height = ctx->seqinfo.uVerDecodeRes;
	uNumMbs = DIV_ROUND_UP(width, 16) * DIV_ROUND_UP(height, 16);
	uTotalBinSize = uNumMbs * DCP_FIXED_MB_ALLOC * uNumDcpChunks;
	uTotalBinSize = min_t(u32, uTotalBinSize, DCP_SIZE);
	uTotalBinSize = max_t(u32, uTotalBinSize, DCP_SIZE_MINIMUM);

	return uTotalBinSize;
}

static int free_mbi_buffers(struct vpu_ctx *ctx)
{
	u_int32 i;

	for (i = 0; i < ARRAY_SIZE(ctx->mbi_buffer); i++)
		free_dma_buffer(ctx, &ctx->mbi_buffer[i]);

	return 0;
}

static int free_dcp_buffers(struct vpu_ctx *ctx)
{
	u_int32 i;

	for (i = 0; i < ARRAY_SIZE(ctx->dcp_buffer); i++)
		free_dma_buffer(ctx, &ctx->dcp_buffer[i]);

	return 0;
}

static int free_decoder_buffer(struct vpu_ctx *ctx)
{
	struct queue_data *queue;

	queue = &ctx->q_data[V4L2_DST];
	down(&queue->drv_q_lock);
	reset_mbi_dcp_count(ctx);
	up(&queue->drv_q_lock);

	queue = &ctx->q_data[V4L2_SRC];
	down(&queue->drv_q_lock);
	free_dma_buffer(ctx, &ctx->stream_buffer);
	free_dma_buffer(ctx, &ctx->udata_buffer);
	up(&queue->drv_q_lock);

	return 0;
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

	if (vb2_is_streaming(&q_data->vb2_q)) {
		vpu_err("%s reqbufs (%d) during streaming\n",
			q_data->type ? "CAPTURE" : "OUTPUT",
			reqbuf->count);
		return -EBUSY;
	}

	if (reqbuf->count > 0 && !q_data->sizeimage[0]) {
		vpu_err("sizeimage isn't initialized, %s reqbufs fail\n",
			q_data->type ? "CAPTURE" : "OUTPUT");
		return -EINVAL;
	}

	ret = vpu_dec_queue_reqbufs(q_data, reqbuf);
	if (ret) {
		vpu_dbg(LVL_WARN, "warning: %s() can't request (%d) buffer : %d\n",
				__func__, reqbuf->count, ret);
		return ret;
	}
	if (V4L2_TYPE_IS_OUTPUT(reqbuf->type))
		return ret;

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
		vpu_err("error: %s() return ret=%d\n", __func__, ret);

	return ret;
}

static bool is_codec_config_data(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf;

	if (!vb)
		return false;

	vbuf = to_vb2_v4l2_buffer(vb);
	if (vbuf->flags & V4L2_NXP_BUF_FLAG_CODECCONFIG)
		return true;
	return false;
}

static void vpu_dec_receive_ts(struct vpu_ctx *ctx,
				struct vb2_buffer *vb,
				int size)
{
	struct vb2_v4l2_buffer *vbuf;
	s64 input_ts;
	u32 data[3];

	if (!ctx || !vb)
		return;

	vbuf = to_vb2_v4l2_buffer(vb);
	if (vbuf->flags & V4L2_NXP_BUF_FLAG_TIMESTAMP_INVALID) {
		vpu_dbg(LVL_BIT_TS, "[INPUT  TS]Invalid timestamp\n");
		vb->timestamp = TSM_TIMESTAMP_NONE;
	}

	input_ts = vb->timestamp;
	if (input_ts < 0)
		input_ts = TSM_TIMESTAMP_NONE;

	if (TSM_TS_IS_VALID(input_ts) && input_ts == ctx->output_ts)
		vpu_dbg(LVL_BIT_TS, "repetitive timestamp\n");
	if (TSM_TS_IS_VALID(input_ts) && input_ts > ctx->output_ts)
		ctx->output_ts = input_ts;

	if (is_codec_config_data(vb) && !TSM_TS_IS_VALID(input_ts)) {
		vpu_dbg(LVL_BIT_TS, "[INPUT  TS]codec data\n");
		ctx->extra_size += size;
		return;
	}

	if (!is_codec_config_data(vb))
		ctx->statistic.frame_input++;

	size += ctx->extra_size;
	ctx->extra_size = 0;
	vpu_dbg(LVL_BIT_TS, "[INPUT  TS]%32lld\n", input_ts);

	if (input_ts < 0) {
		data[0] = (u32)-1;
		data[1] = 0;
	} else {
		data[0] = input_ts / NSEC_PER_SEC;
		data[1] = input_ts % NSEC_PER_SEC;
	}
	data[2] = size;
	v4l2_vpu_send_cmd(ctx, ctx->str_index, VID_API_CMD_TS, 3, data);
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
	} else {
		vpu_err("qbuf invalid buf type : %d\n", buf->type);
		return -EINVAL;
	}

	ret = vpu_dec_queue_qbuf(q_data, buf);
	if (ret) {
		vpu_err("error: %s() return ret=%d\n", __func__, ret);
		return ret;
	}
	v4l2_update_stream_addr(ctx, 0);

	return ret;
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
		vpu_err("error: %s() return ret=%d\n", __func__, ret);
		return ret;
	}

	if (!V4L2_TYPE_IS_OUTPUT(buf->type)) {
		down(&ctx->q_data[V4L2_SRC].drv_q_lock);
		ctx->statistic.frame_display++;
		up(&ctx->q_data[V4L2_SRC].drv_q_lock);
	}

	if (q_data->vb2_reqs[buf->index].bfield)
		buf->field = V4L2_FIELD_INTERLACED;
	else
		buf->field = V4L2_FIELD_NONE;
	v4l2_update_stream_addr(ctx, 0);
	if (!V4L2_TYPE_IS_OUTPUT(buf->type) && is_10bit_format(ctx))
		buf->reserved = 1;

	if (V4L2_TYPE_IS_OUTPUT(buf->type))
		buf->flags &= ~V4L2_NXP_BUF_MASK_FLAGS;

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
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
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

	try_colorspace(ctx, f);

	return 0;
}

static int vpu_dec_v4l2_ioctl_g_selection(struct file *file, void *fh,
					struct v4l2_selection *s)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	if (s->target != V4L2_SEL_TGT_CROP && s->target != V4L2_SEL_TGT_COMPOSE)
		return -EINVAL;

	s->r.left = ctx->seqinfo.uFrameCropLeftOffset;
	s->r.top = ctx->seqinfo.uFrameCropTopOffset;
	s->r.width = ctx->seqinfo.uHorRes;
	s->r.height = ctx->seqinfo.uVerRes;

	return 0;
}

static int v4l2_ioctl_decoder_cmd(struct file *file,
		void *fh,
		struct v4l2_decoder_cmd *cmd
		)
{
	int ret = 0;
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
	case V4L2_DEC_CMD_RESET:
		v4l2_update_stream_addr(ctx, 0);
		mutex_lock(&ctx->fw_flow_mutex);
		ret = vpu_dec_cmd_reset(ctx);
		mutex_unlock(&ctx->fw_flow_mutex);
		break;
	default:
		return -EINVAL;
	}
	return ret;
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

	vpu_dbg(LVL_BIT_FLOW, "ctx[%d] %s on\n", ctx->str_index,
		V4L2_TYPE_IS_OUTPUT(i) ? "OUTPUT" : "CAPTURE");

	if (V4L2_TYPE_IS_OUTPUT(i)) {
		ctx->firmware_finished = false;
		ctx->eos_stop_added = false;
	}

	ret = vpu_dec_queue_enable(q_data, i);
	if (!ret)
		v4l2_update_stream_addr(ctx, 0);
	else
		vpu_err("error: %s %s() return ret=%d\n", V4L2_TYPE_IS_OUTPUT(i) ? "OUTPUT" : "CAPTURE", __func__, ret);

	if (ctx->hang_status) {
		vpu_err("%s(): not succeed and some instance are blocked\n", __func__);
		return -EINVAL;
	}
	if (ret)
		return ret;

	down(&q_data->drv_q_lock);
	q_data->enable = true;
	if (!V4L2_TYPE_IS_OUTPUT(i))
		respond_req_frame(ctx, q_data, false);
	up(&q_data->drv_q_lock);

	return ret;
}

static bool is_need_abort(struct vpu_ctx *ctx, enum v4l2_buf_type type)
{
	bool src_status = vb2_is_streaming(&ctx->q_data[V4L2_SRC].vb2_q);

	if (V4L2_TYPE_IS_OUTPUT(type))
		return false;
	if (!vpu_dec_is_active(ctx))
		return false;

	if (src_status)
		ctx->seek_flag = false;
	else
		ctx->seek_flag = true;
	if (ctx->wait_res_change_done) {
		if (src_status)
			return false;
		vpu_dbg(LVL_INFO,
			"ctx[%d] seek in res change\n", ctx->str_index);
	}
	return true;
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

	vpu_dbg(LVL_BIT_FLOW, "ctx[%d] %s off\n", ctx->str_index,
		V4L2_TYPE_IS_OUTPUT(i) ? "OUTPUT" : "CAPTURE");

	down(&q_data->drv_q_lock);
	q_data->enable = false;
	up(&q_data->drv_q_lock);

	if (is_need_abort(ctx, i)) {
		mutex_lock(&ctx->fw_flow_mutex);
		send_abort_cmd(ctx);
		mutex_unlock(&ctx->fw_flow_mutex);
	}

	if (V4L2_TYPE_IS_OUTPUT(i))
		ctx->output_ts = TSM_TIMESTAMP_NONE;
	else
		ctx->capture_ts = TSM_TIMESTAMP_NONE;

	ret = vpu_dec_queue_disable(q_data, i);
	if (ctx->hang_status) {
		vpu_err("%s(): not succeed and some instance are blocked\n", __func__);
		ret = -EINVAL;
	}

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
		numerator = 1;
		denominator = 0;
	}
	parm->parm.capture.timeperframe.numerator = numerator;
	parm->parm.capture.timeperframe.denominator = denominator;

	mutex_unlock(&ctx->instance_mutex);
	vpu_dbg(LVL_BIT_FLOW, "%s g_parm : %d / %d\n",
			V4L2_TYPE_IS_OUTPUT(parm->type) ? "OUTPUT" : "CAPTURE",
			parm->parm.capture.timeperframe.numerator,
			parm->parm.capture.timeperframe.denominator);

	return 0;
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
	mutex_unlock(&ctx->instance_mutex);

	vpu_dbg(LVL_BIT_FLOW, "%s s_parm : %d / %d\n",
			V4L2_TYPE_IS_OUTPUT(parm->type) ? "OUTPUT" : "CAPTURE",
			parm->parm.capture.timeperframe.numerator,
			parm->parm.capture.timeperframe.denominator);

	return 0;
}

static const struct v4l2_ioctl_ops v4l2_decoder_ioctl_ops = {
	.vidioc_querycap                = v4l2_ioctl_querycap,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	.vidioc_enum_fmt_vid_cap	= v4l2_ioctl_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out	= v4l2_ioctl_enum_fmt_vid_out,
#else
	.vidioc_enum_fmt_vid_cap_mplane = v4l2_ioctl_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out_mplane = v4l2_ioctl_enum_fmt_vid_out,
#endif
	.vidioc_g_fmt_vid_cap_mplane    = v4l2_ioctl_g_fmt,
	.vidioc_g_fmt_vid_out_mplane    = v4l2_ioctl_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane  = v4l2_ioctl_try_fmt,
	.vidioc_try_fmt_vid_out_mplane  = v4l2_ioctl_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane    = v4l2_ioctl_s_fmt,
	.vidioc_s_fmt_vid_out_mplane    = v4l2_ioctl_s_fmt,
	.vidioc_g_parm			= vpu_dec_v4l2_ioctl_g_parm,
	.vidioc_s_parm			= vpu_dec_v4l2_ioctl_s_parm,
	.vidioc_expbuf                  = v4l2_ioctl_expbuf,
	.vidioc_g_selection		= vpu_dec_v4l2_ioctl_g_selection,
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

static	struct v4l2_ctrl_config vpu_custom_s_cfg[] = {
	{
		.id = V4L2_CID_DIS_REORDER,
		.name = "frame disable reoder ctrl",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	},
	{
		.id = V4L2_CID_NON_FRAME,
		.name = "stream input non frame mode",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	}
};

#define CNT_STAND_G_CTRLS	ARRAY_SIZE(vpu_controls_dec)
#define CNT_CUSTOM_S_CFG	ARRAY_SIZE(vpu_custom_s_cfg)
#define CNT_CTRLS_DEC		(CNT_STAND_G_CTRLS + CNT_CUSTOM_S_CFG)

static int v4l2_custom_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);

	vpu_dbg(LVL_BIT_FUNC, "%s() control(%d)\n",
			__func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_DIS_REORDER:
		ctx->b_dis_reorder = ctrl->val;
		break;
	case V4L2_CID_NON_FRAME:
		if (ctrl->val) {
			ctx->stream_input_mode = NON_FRAME_LVL;
			ctx->start_code_bypass = true;
		} else {
			ctx->stream_input_mode = FRAME_LVL;
			ctx->start_code_bypass = false;
		}
		break;
	default:
		vpu_err("%s() Invalid costomer control(%d)\n",
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
		ctrl->val = ctx->seqinfo.uNumDPBFrms + ctx->seqinfo.uNumRefFrms;
		break;
	default:
		vpu_err("%s() Invalid control(%d)\n",
				__func__, ctrl->id);
		return -EINVAL;
	}
	return 0;
}

static int add_stand_g_ctrl(struct vpu_ctx *This)
{
	static const struct v4l2_ctrl_ops vpu_dec_ctrl_ops = {
		.g_volatile_ctrl  = v4l2_dec_g_v_ctrl,
	};
	u_int32 i;
	struct v4l2_ctrl *ctrl;

	if (!This)
		return -EINVAL;

	for (i = 0; i < CNT_STAND_G_CTRLS; i++) {
		ctrl = v4l2_ctrl_new_std(&This->ctrl_handler,
					 &vpu_dec_ctrl_ops,
					 vpu_controls_dec[i].id,
					 vpu_controls_dec[i].minimum,
					 vpu_controls_dec[i].maximum,
					 vpu_controls_dec[i].step,
					 vpu_controls_dec[i].default_value);
		if (This->ctrl_handler.error || !ctrl) {
			vpu_err("%s() v4l2_ctrl_new_std[%d] failed: %d\n",
				__func__, i, This->ctrl_handler.error);
			return This->ctrl_handler.error;
		}

		if (vpu_controls_dec[i].is_volatile)
			ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;
	}

	ctrl = NULL;
	return 0;
}

static int add_custom_s_ctrl(struct vpu_ctx *This)
{
	static const struct v4l2_ctrl_ops vpu_custom_ctrl_ops = {
		.s_ctrl = v4l2_custom_s_ctrl,
	};
	uint32_t i;
	struct v4l2_ctrl *ctrl;

	if (!This)
		return -EINVAL;

	for (i = 0; i < CNT_CUSTOM_S_CFG; i++) {
		vpu_custom_s_cfg[i].ops = &vpu_custom_ctrl_ops;

		ctrl = v4l2_ctrl_new_custom(&This->ctrl_handler,
					    &vpu_custom_s_cfg[i], NULL);

		if (This->ctrl_handler.error || !ctrl) {
			vpu_err("%s() v4l2_ctrl_new_std[%d] failed: %d\n",
				__func__, i, This->ctrl_handler.error);
			return This->ctrl_handler.error;
		}
	}

	ctrl = NULL;
	return 0;
}

static int add_dec_ctrl(struct vpu_ctx *This)
{
	if (!This)
		return -EINVAL;

	add_stand_g_ctrl(This);
	add_custom_s_ctrl(This);

	return 0;
}


static int ctrls_setup_decoder(struct vpu_ctx *This)
{
	if (!This)
		return -EINVAL;

	v4l2_ctrl_handler_init(&This->ctrl_handler, CNT_CTRLS_DEC);
	if (This->ctrl_handler.error) {
		vpu_err("%s() v4l2_ctrl_handler_init failed(%d)\n",
				__func__, This->ctrl_handler.error);

		return This->ctrl_handler.error;
	} else {
		vpu_dbg(LVL_INFO, "%s() v4l2_ctrl_handler_init ctrls(%ld)\n",
				__func__, CNT_CTRLS_DEC);
		This->ctrl_inited = true;
	}

	add_dec_ctrl(This);

	v4l2_ctrl_handler_setup(&This->ctrl_handler);

	return 0;
}

static void ctrls_delete_decoder(struct vpu_ctx *This)
{
	if (!This)
		return;

	if (This->ctrl_inited) {
		v4l2_ctrl_handler_free(&This->ctrl_handler);
		This->ctrl_inited = false;
	}
}

static void update_wptr(struct vpu_ctx *ctx,
			pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc,
			u32 wptr)
{
	u32 size;
	u32 length;

	size = pStrBufDesc->end - pStrBufDesc->start;
	length = (wptr + size - pStrBufDesc->wptr) % size;
	ctx->total_write_bytes += length;

	vpu_dbg(LVL_BIT_FRAME_BYTES, "[%d]receive bytes : %8d / %16ld\n",
			ctx->str_index, length, ctx->total_write_bytes);
	vpu_dbg(LVL_BIT_WPTR, "ctx[%d] wptr : 0x%08x -> 0x%08x\n",
			ctx->str_index, pStrBufDesc->wptr, wptr);
	pStrBufDesc->wptr = wptr;
}

/* Insert either the codec specific EOS type or a special scode to mark that this frame should be flushed/pushed directly for decode */
static int add_scode_vpu(struct vpu_ctx *ctx, u_int32 uStrBufIdx, VPU_PADDING_SCODE_TYPE eScodeType, bool bUpdateWr)
{
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
		vpu_err("error: %s(), start or end pointer cross-border\n", __func__);
		return 0;
	}
	if (wptr < start || wptr > end) {
		vpu_err("error: %s(), wptr pointer cross-border\n", __func__);
		return 0;
	}
	if (rptr < start || rptr > end) {
		vpu_err("error: %s(), rptr pointer cross-border\n", __func__);
		return 0;
	}

	buffer = kzalloc(SCODE_SIZE, GFP_KERNEL);
	if (!buffer)
		return 0;

	atomic64_add(SCODE_SIZE, &ctx->statistic.total_alloc_size);
	plbuffer = (uint32_t *)buffer;
	if (wptr - start < ctx->stream_buffer.dma_size)
		pbbuffer = (uint8_t *)(ctx->stream_buffer.dma_virt + wptr - start);
	else {
		vpu_err("error: return wptr(0x%x), start(0x%x) is not valid\n", wptr, start);
		goto error;
	}

	if (((u_int64)pbbuffer)%4 != 0) {
		int i;
		if (end%4 != 0) {
			vpu_err("end address of stream not aligned by 4 bytes !\n");
			goto error;
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
		case VPU_VIDEO_AVS:
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
		case VPU_VIDEO_AVS:
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
			goto error;
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
			pad_bytes += SCODE_SIZE;
		} else {
			if (rptr - start > SCODE_SIZE - (end - wptr)) {
				memcpy(pbbuffer, buffer, end - wptr);
				memcpy(ctx->stream_buffer.dma_virt, buffer + (end - wptr), SCODE_SIZE - (end - wptr));
				wptr = start + SCODE_SIZE - (end - wptr);
				pad_bytes += SCODE_SIZE;
			} else {
				vpu_err("No enough space to insert padding data, size=%d, rptr(%x), wptr(%x)\n",
					(end - wptr) + (rptr - start), rptr, wptr);
			}
		}
	} else {
		if (rptr - wptr >= SCODE_SIZE) {
			memcpy(pbbuffer, buffer, SCODE_SIZE);
			wptr += SCODE_SIZE;
			pad_bytes += SCODE_SIZE;
		} else	{
			vpu_err("No enough space to insert padding data, size=%d, rptr(%x), wptr(%x)\n",
				rptr - wptr, rptr, wptr);
		}
	}
	mb();

	if (bUpdateWr)
		update_wptr(ctx, pStrBufDesc, wptr);
	kfree(buffer);
	atomic64_sub(SCODE_SIZE, &ctx->statistic.total_alloc_size);

	return pad_bytes;
error:
	kfree(buffer);
	atomic64_sub(SCODE_SIZE, &ctx->statistic.total_alloc_size);
	return 0;
}

static int add_scode(struct vpu_ctx *ctx, u_int32 uStrBufIdx, VPU_PADDING_SCODE_TYPE eScodeType, bool bUpdateWr)
{
	int size = 0;

	if (!ctx)
		return 0;

	if (eScodeType == EOS_PADDING_TYPE)
		vpu_dbg(LVL_BIT_FLOW, "ctx[%d] add eos\n", ctx->str_index);

	mutex_lock(&ctx->instance_mutex);
	size = add_scode_vpu(ctx, uStrBufIdx, eScodeType, bUpdateWr);
	if (size > 0) {
		if (!(eScodeType == BUFFLUSH_PADDING_TYPE && ctx->b_dis_reorder))
			set_pic_end_flag(ctx);
	} else {
		size = 0;
	}
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

static void vpu_dec_send_cmd(struct vpu_dev *dev, u32 idx, u32 cmdid,
				u32 cmdnum, u32 *local_cmddata)
{
	WARN_ON(!dev || idx >= VPU_MAX_NUM_STREAMS);

	if (!rpc_check_is_ready(&dev->shared_mem, idx))
		vpu_err("[%d] is not ready\n", idx);
	mutex_lock(&dev->cmd_mutex);
	rpc_send_cmd_buf(&dev->shared_mem, idx, cmdid, cmdnum, local_cmddata);
	mb();
	vpu_mu_send_msg(dev, COMMAND, 0xffff);
	mutex_unlock(&dev->cmd_mutex);
}

static void do_send_cmd_to_firmware(struct vpu_ctx *ctx,
				uint32_t idx, uint32_t cmdid,
				uint32_t cmdnum, uint32_t *local_cmddata)
{
	vpu_log_cmd(cmdid, idx);
	count_cmd(&ctx->statistic, cmdid);
	record_log_info(ctx, LOG_COMMAND, cmdid, 0);

	vpu_dec_send_cmd(ctx->dev, idx, cmdid, cmdnum, local_cmddata);
}

static struct vpu_dec_cmd_request vpu_dec_cmds[] = {
	{
		.request = VID_API_CMD_START,
		.response = VID_API_EVENT_START_DONE,
		.block = true,
	},
	{
		.request = VID_API_CMD_STOP,
		.response = VID_API_EVENT_STOPPED,
		.block = true,
	},
	{
		.request = VID_API_CMD_ABORT,
		.response = VID_API_EVENT_ABORT_DONE,
		.block = true,
	},
	{
		.request = VID_API_CMD_RST_BUF,
		.response = VID_API_EVENT_STR_BUF_RST,
		.block = true,
	}
};

static struct vpu_dec_cmd_request *get_cmd_request(struct vpu_ctx *ctx,
							u32 cmdid)
{
	struct vpu_dec_cmd_request *request;
	int i;

	request = vzalloc(sizeof(*request));
	if (!request)
		return NULL;

	atomic64_add(sizeof(*request), &ctx->statistic.total_alloc_size);
	request->request = cmdid;
	request->response = VID_API_EVENT_INVALID;
	request->block = false;
	for (i = 0; i < ARRAY_SIZE(vpu_dec_cmds); i++) {
		if (vpu_dec_cmds[i].request == cmdid) {
			memcpy(request, &vpu_dec_cmds[i], sizeof(*request));
			break;
		}
	}

	return request;
}

static void put_cmd_request(struct vpu_ctx *ctx,
				struct vpu_dec_cmd_request *request)
{
	if (!request)
		return;

	atomic64_sub(sizeof(*request), &ctx->statistic.total_alloc_size);
	vfree(request);
}

static void vpu_dec_cleanup_cmd(struct vpu_ctx *ctx)
{
	struct vpu_dec_cmd_request *request;
	struct vpu_dec_cmd_request *tmp;

	mutex_lock(&ctx->cmd_lock);
	if (ctx->pending) {
		vpu_err("ctx[%d]'s cmd(%s) is not finished yet\n",
			ctx->str_index, get_cmd_str(ctx->pending->request));
		put_cmd_request(ctx, ctx->pending);
		ctx->pending = NULL;
	}
	list_for_each_entry_safe(request, tmp, &ctx->cmd_q, list) {
		list_del_init(&request->list);
		vpu_err("cmd(%s) of ctx[%d] is missed\n",
				get_cmd_str(request->request), ctx->str_index);
		put_cmd_request(ctx, request);
	}
	mutex_unlock(&ctx->cmd_lock);
}

static void process_cmd_request(struct vpu_ctx *ctx)
{
	struct vpu_dec_cmd_request *request;
	struct vpu_dec_cmd_request *tmp;

	if (ctx->pending)
		return;

	list_for_each_entry_safe(request, tmp, &ctx->cmd_q, list) {
		list_del_init(&request->list);
		do_send_cmd_to_firmware(ctx,
					request->idx,
					request->request,
					request->num,
					request->data);
		if (request->block &&
				request->response != VID_API_EVENT_INVALID) {
			ctx->pending = request;
			break;
		}
		put_cmd_request(ctx, request);
	}
}

static void vpu_dec_request_cmd(struct vpu_ctx *ctx,
				uint32_t idx, uint32_t cmdid,
				uint32_t cmdnum, uint32_t *local_cmddata)
{
	struct vpu_dec_cmd_request *request;
	u32 i;

	if (cmdnum > VPU_DEC_CMD_DATA_MAX_NUM) {
		vpu_err("cmd(%s)'s data number(%d) > %d, drop it\n",
			get_cmd_str(cmdid), cmdnum, VPU_DEC_CMD_DATA_MAX_NUM);
		return;
	}

	request = get_cmd_request(ctx, cmdid);
	if (!request) {
		vpu_err("cmd(%s) of ctx[%d] is missed\n",
				get_cmd_str(cmdid), idx);
		return;
	}

	request->idx = idx;
	request->num = cmdnum;
	for (i = 0; i < cmdnum && i < ARRAY_SIZE(request->data); i++)
		request->data[i] = local_cmddata[i];

	mutex_lock(&ctx->cmd_lock);
	list_add_tail(&request->list, &ctx->cmd_q);
	process_cmd_request(ctx);
	mutex_unlock(&ctx->cmd_lock);
}

static void vpu_dec_response_cmd(struct vpu_ctx *ctx, u32 event)
{
	mutex_lock(&ctx->cmd_lock);
	if (ctx->pending && event == ctx->pending->response) {
		put_cmd_request(ctx, ctx->pending);
		ctx->pending = NULL;
	}

	process_cmd_request(ctx);
	mutex_unlock(&ctx->cmd_lock);
}

static void vpu_dec_clear_pending_cmd(struct vpu_ctx *ctx)
{
	mutex_lock(&ctx->cmd_lock);
	if (ctx->pending) {
		put_cmd_request(ctx, ctx->pending);
		ctx->pending = NULL;
	}
	mutex_unlock(&ctx->cmd_lock);
}

static void v4l2_vpu_send_cmd(struct vpu_ctx *ctx,
				uint32_t idx, uint32_t cmdid,
				uint32_t cmdnum, uint32_t *local_cmddata)
{
	vpu_dec_request_cmd(ctx, idx, cmdid, cmdnum, local_cmddata);
}

static void dump_input_data_to_local(struct vpu_ctx *ctx, void *src, u_int32 len)
{
	struct file *fp;
	char input_file[64];

	if (!vpu_datadump_ena)
		return;

	scnprintf(input_file, sizeof(input_file) - 1,
			"/data/vpu_input_data_%d.bin", ctx->str_index);

	if (ctx->first_dump_data_flag) {
		fp = filp_open(input_file, O_RDWR | O_TRUNC | O_CREAT, 0644);
		ctx->first_dump_data_flag = false;
	} else {
		fp = filp_open(input_file, O_RDWR | O_APPEND | O_CREAT, 0644);
	}
	if (IS_ERR(fp)) {
		vpu_dbg(LVL_WARN, "warning: open file(%s) fail\n", input_file);
		return;
	}

	kernel_write(fp, src, len, &(fp->f_pos));
	filp_close(fp, NULL);
}

static int send_start_cmd(struct vpu_ctx *ctx)
{
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	u_int32 uStrBufIdx = 0; //set to be default 0, FIX_ME later
	MediaIPFW_Video_UData *pUdataBuf;
	pDEC_RPC_HOST_IFACE pSharedInterface;
	MediaIPFW_Video_CodecParams *pCodecPara;
	unsigned int *CurrStrfg;
	u32 vdec_std;

	if (!ctx)
		return -EINVAL;
	if (vpu_dec_is_active(ctx))
		return -EINVAL;

	pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	pUdataBuf = &pSharedInterface->UDataBuffer[ctx->str_index];
	pCodecPara = ctx->dev->shared_mem.codec_mem_vir;
	CurrStrfg = &pSharedInterface->StreamConfig[ctx->str_index];
	vdec_std = ctx->q_data[V4L2_SRC].vdec_std;

	vpu_dbg(LVL_WARN, "firmware version is %d.%d.%d\n",
			(pSharedInterface->FWVersion & 0x00ff0000) >> 16,
			(pSharedInterface->FWVersion & 0x0000ff00) >> 8,
			pSharedInterface->FWVersion & 0x000000ff);

	pStrBufDesc = get_str_buffer_desc(ctx);
	pStrBufDesc->wptr = ctx->stream_buffer.dma_phy;
	pStrBufDesc->rptr = ctx->stream_buffer.dma_phy;
	pStrBufDesc->start = ctx->stream_buffer.dma_phy;
	pStrBufDesc->end = ctx->stream_buffer.dma_phy + ctx->stream_buffer.dma_size;
	pStrBufDesc->LWM = 0x01;
	ctx->beginning = pStrBufDesc->start;

	pSharedInterface->pStreamBuffDesc[ctx->str_index][uStrBufIdx] =
		(VPU_REG_BASE + get_str_buffer_desc_offset(ctx));
	pUdataBuf->uUDataBase = ctx->udata_buffer.dma_phy;
	pUdataBuf->uUDataSlotSize = ctx->udata_buffer.dma_size;
	VID_STREAM_CONFIG_FORMAT_SET(vpu_format_remap(vdec_std), CurrStrfg);

	if (vdec_std == VPU_VIDEO_JPEG) {
		MediaIPFW_Video_JpegParams *pJpgPara;

		pJpgPara = (MediaIPFW_Video_JpegParams *)ctx->dev->shared_mem.jpeg_mem_vir;
		pJpgPara[ctx->str_index].uJpgMjpegMode = 1; //1:JPGD_MJPEG_MODE_A; 2:JPGD_MJPEG_MODE_B
		pJpgPara[ctx->str_index].uJpgMjpegInterlaced = 0; //0: JPGD_MJPEG_PROGRESSIVE
	}

	if (ctx->b_dis_reorder)
		pCodecPara[ctx->str_index].uDispImm = 1;
	else
		pCodecPara[ctx->str_index].uDispImm = 0;

	pCodecPara[ctx->str_index].uEnableDbgLog = CHECK_BIT(vpu_frmdbg_ena, ctx->str_index) ? 1 : 0;
	pSharedInterface->DbgLogDesc.uDecStatusLogLevel = vpu_frmdbg_level;

	ctx->frm_dis_delay = 0;
	ctx->frm_dec_delay = 0;
	ctx->frm_total_num = 0;
	fill_stream_buffer_info(ctx);

	v4l2_vpu_send_cmd(ctx, ctx->str_index, VID_API_CMD_START, 0, NULL);
	ctx->firmware_stopped = false;
	ctx->first_data_flag = true;

	vpu_calculate_performance(ctx, 0xff, "send start cmd");

	return 0;
}

static void activate_vpu_dec(struct vpu_ctx *ctx)
{
	int ret;

	vpu_calculate_performance(ctx, 0xff, "alloc stream buffer begin");
	ret = alloc_vpu_buffer(ctx);
	if (ret) {
		vpu_err("alloc vpu buffer fail\n");
		return;
	}
	vpu_calculate_performance(ctx, 0xff, "alloc stream buffer finish");

	send_start_cmd(ctx);
}

u_int32 got_free_space(u_int32 wptr, u_int32 rptr, u_int32 start, u_int32 end)
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

static u32 got_used_space(u32 wptr, u32 rptr, u32 start, u32 end)
{
	u32 stream_size = 0;

	if (wptr == rptr)
		stream_size = 0;
	else if (rptr < wptr)
		stream_size = wptr - rptr;
	else
		stream_size = (end - rptr) + (wptr - start);

	return stream_size;
}

int copy_buffer_to_stream(struct vpu_ctx *ctx, void *buffer, uint32_t length)
{
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	void *wptr_virt;
	uint32_t wptr;
	uint32_t rptr;
	uint32_t start;
	uint32_t end;

	if (!ctx || !buffer || !length)
		return 0;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	pStrBufDesc = get_str_buffer_desc(ctx);
	wptr = pStrBufDesc->wptr;
	rptr = pStrBufDesc->rptr;
	start = pStrBufDesc->start;
	end = pStrBufDesc->end;

	if (start != ctx->stream_buffer.dma_phy ||
			end != ctx->stream_buffer.dma_phy + ctx->stream_buffer.dma_size) {
		vpu_err("error: %s(), start or end pointer cross-border\n", __func__);
		return 0;
	}
	if (wptr < start || wptr > end) {
		vpu_err("error: %s(), wptr pointer cross-border\n", __func__);
		return 0;
	}
	if (rptr < start || rptr > end) {
		vpu_err("error: %s(), rptr pointer cross-border\n", __func__);
		return 0;
	}


	wptr_virt = (void *)ctx->stream_buffer.dma_virt + wptr - start;
	if ((wptr == rptr) || (wptr > rptr)) {
		if (end - wptr >= length) {
			memcpy(wptr_virt, buffer, length);
			wptr += length;
			if (wptr == end)
				wptr = start;
		} else {
			memcpy(wptr_virt, buffer, end-wptr);
			memcpy(ctx->stream_buffer.dma_virt, buffer + (end-wptr), length - (end-wptr));
			wptr = start + length - (end - wptr);
		}
	} else {
		memcpy(wptr_virt, buffer, length);
		wptr += length;
	}
	dump_input_data_to_local(ctx, buffer, length);
	mb();
	update_wptr(ctx, pStrBufDesc, wptr);
	return length;
}

static int send_abort_cmd(struct vpu_ctx *ctx)
{
	int size;

	if (!ctx)
		return -EINVAL;

	if (!vpu_dec_is_active(ctx))
		return 0;

	ctx->wait_rst_done = true;
	vpu_dbg(LVL_BIT_FLOW, "ctx[%d] send ABORT CMD\n", ctx->str_index);
	size = add_scode(ctx, 0, BUFABORT_PADDING_TYPE, false);
	record_log_info(ctx, LOG_PADDING, 0, 0);
	if (size <= 0)
		vpu_err("%s(): failed to fill abort padding data\n", __func__);
	reinit_completion(&ctx->completion);
	v4l2_vpu_send_cmd(ctx, ctx->str_index, VID_API_CMD_ABORT, 1, &size);
	if (!wait_for_completion_timeout(&ctx->completion, msecs_to_jiffies(1000))) {
		ctx->hang_status = true;
		vpu_err("the path id: %d firmware timeout after send %s\n",
			ctx->str_index, get_cmd_str(VID_API_CMD_ABORT));
		vpu_dec_clear_pending_cmd(ctx);
		return -EBUSY;
	}

	return 0;
}

static int send_stop_cmd(struct vpu_ctx *ctx)
{
	if (!ctx)
		return -EINVAL;

	if (!vpu_dec_is_active(ctx))
		return 0;

	ctx->wait_rst_done = true;
	vpu_dbg(LVL_BIT_FLOW, "ctx[%d] send STOP CMD\n", ctx->str_index);
	reinit_completion(&ctx->stop_cmp);
	v4l2_vpu_send_cmd(ctx, ctx->str_index, VID_API_CMD_STOP, 0, NULL);
	if (!wait_for_completion_timeout(&ctx->stop_cmp, msecs_to_jiffies(1000))) {

		vpu_dec_clear_pending_cmd(ctx);
		ctx->hang_status = true;
		vpu_err("the path id: %d firmware timeout after send %s\n",
			ctx->str_index, get_cmd_str(VID_API_CMD_STOP));
		return -EBUSY;
	}

	return 0;
}

static int vpu_dec_cmd_reset(struct vpu_ctx *ctx)
{
	int ret = 0;

	if (!ctx)
		return -EPERM;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);
	vpu_dbg(LVL_BIT_FLOW, "ctx[%d] reset decoder\n", ctx->str_index);

	if (ctx->hang_status) {
		vpu_dbg(LVL_WARN, "warning: %s() failed. hang_status: %d\n",
			__func__, ctx->hang_status);
		return -EPERM;
	}

	ret = send_abort_cmd(ctx);
	if (ret)
		return ret;
	ret = send_stop_cmd(ctx);
	if (ret)
		return ret;

	return 0;
}

static void vpu_dec_event_decode_error(struct vpu_ctx *ctx)
{
	const struct v4l2_event ev = {
		.id = 0,
		.type = V4L2_EVENT_CODEC_ERROR
	};

	if (!ctx)
		return;

	vpu_dbg(LVL_BIT_FLOW, "send decode error event\n");
	v4l2_event_queue_fh(&ctx->fh, &ev);
}

static int update_stream_addr(struct vpu_ctx *ctx, void *input_buffer, uint32_t buffer_size, uint32_t uStrBufIdx)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	uint32_t copy_length = 0;
	uint32_t input_offset = 0;
	struct VPU_FMT_INFO_ARV *arv_frame;
	uint32_t i;

	if (q_data->vdec_std != VPU_VIDEO_RV) {
		if (!check_free_size_pic(ctx, buffer_size))
			return 0;

		copy_length += insert_scode(ctx, SCODE_NEW_PICTURE, buffer_size, input_buffer);
		copy_length += copy_buffer_to_stream(ctx, input_buffer, buffer_size);
	} else {
		arv_frame = get_arv_info(ctx, input_buffer, buffer_size);
		if (!arv_frame) {
			vpu_dbg(LVL_WARN, "warning: %s() get arv frame info failed\n", __func__);
			return -1;
		}
		if (!check_free_size_pic(ctx, buffer_size + arv_frame->slice_num * 16)) {
			put_arv_info(arv_frame);
			arv_frame = NULL;
			return 0;
		}

		arv_frame->packlen = 20 + 8 * arv_frame->slice_num;

		if (arv_frame->packlen >  buffer_size - input_offset) {
			put_arv_info(arv_frame);
			arv_frame = NULL;
			return -1;
		}

		copy_length += insert_scode(ctx, SCODE_NEW_PICTURE, buffer_size, input_buffer);
		copy_length += copy_buffer_to_stream(ctx, input_buffer + input_offset, arv_frame->packlen);
		input_offset += arv_frame->packlen;
		for (i = 0; i < arv_frame->slice_num; i++) {
			if (i == arv_frame->slice_num - 1)
				arv_frame->packlen = arv_frame->data_len - arv_frame->slice_offset[i];
			else
				arv_frame->packlen = arv_frame->slice_offset[i+1] - arv_frame->slice_offset[i];
			copy_length += insert_scode(ctx, SCODE_NEW_SLICE, arv_frame->packlen, input_buffer + input_offset);
			copy_length += copy_buffer_to_stream(ctx, input_buffer + input_offset, arv_frame->packlen);
			input_offset += arv_frame->packlen;
		}

		put_arv_info(arv_frame);
		arv_frame = NULL;
	}

	return copy_length;
}

static int update_stream_addr_vpu(struct vpu_ctx *ctx, void *input_buffer, uint32_t buffer_size, uint32_t uStrBufIdx)
{
	int size = 0;

	if (ctx->wait_rst_done == true)
		return size;

	mutex_lock(&ctx->instance_mutex);
	size = update_stream_addr(ctx, input_buffer, buffer_size, uStrBufIdx);
	mutex_unlock(&ctx->instance_mutex);

	return size;
}

static int update_stream_addr_bypass(struct vpu_ctx *ctx,
					void *input_buffer,
					uint32_t buffer_size)
{
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	uint32_t nfreespace = 0;
	uint32_t wptr;
	uint32_t rptr;
	uint32_t start;
	uint32_t end;

	pStrBufDesc = get_str_buffer_desc(ctx);
	wptr = pStrBufDesc->wptr;
	rptr = pStrBufDesc->rptr;
	start = pStrBufDesc->start;
	end = pStrBufDesc->end;
	nfreespace = got_free_space(wptr, rptr, start, end);

	if (nfreespace < (buffer_size + MIN_SPACE)) {
		vpu_dbg(LVL_INFO, "buffer_full: the circular buffer freespace < buffer_size\n");
		return 0;
	}
	return copy_buffer_to_stream(ctx, input_buffer, buffer_size);
}

static void fill_stream_buffer_info(struct vpu_ctx *ctx)
{
	pDEC_RPC_HOST_IFACE pSharedInterface;
	pBUFFER_INFO_TYPE buffer_info;
	int idx;

	if (!ctx || ctx->str_index < 0 || ctx->str_index >= VPU_MAX_NUM_STREAMS)
		return;

	idx = ctx->str_index;
	pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	buffer_info = &pSharedInterface->StreamBuffInfo[idx];

	buffer_info->stream_input_mode = ctx->stream_input_mode;
	if (ctx->stream_input_mode == NON_FRAME_LVL)
		buffer_info->stream_buffer_threshold = stream_buffer_threshold;
	else if (frame_threshold[idx] > 0)
		buffer_info->stream_buffer_threshold = frame_threshold[idx];
	else
		buffer_info->stream_buffer_threshold = 0;
}

static void set_pic_end_flag(struct vpu_ctx *ctx)
{
	pDEC_RPC_HOST_IFACE pSharedInterface;
	pBUFFER_INFO_TYPE buffer_info;

	if (!ctx)
		return;

	pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	buffer_info = &pSharedInterface->StreamBuffInfo[ctx->str_index];

	buffer_info->stream_pic_end_flag = 0x1;
}

static bool vpu_dec_stream_is_ready(struct vpu_ctx *ctx)
{
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	u32 stream_size = 0;

	WARN_ON(!ctx);

	if (ctx->fifo_low)
		return true;

	pStrBufDesc = get_str_buffer_desc(ctx);
	stream_size = got_used_space(pStrBufDesc->wptr,
					pStrBufDesc->rptr,
					pStrBufDesc->start,
					pStrBufDesc->end);

	/*
	 *frame depth need to be set by user and then the condition works
	 */
	if (vpu_frm_depth > 0 && ctx->stream_input_mode == FRAME_LVL) {
		if (ctx->frm_dec_delay >= vpu_frm_depth)
			return false;
	}

	return true;
}

static bool verify_decoded_frames(struct vpu_ctx *ctx)
{
	pDEC_RPC_HOST_IFACE pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	pBUFFER_INFO_TYPE buffer_info = &pSharedInterface->StreamBuffInfo[ctx->str_index];

	if (ctx->stream_input_mode != FRAME_LVL)
		return true;

	if (buffer_info->stream_pic_input_count != buffer_info->stream_pic_parsed_count) {
		vpu_dbg(LVL_WARN,
			"ctx[%d] amount inconsistent between input(%d) and output(%d)\n",
			ctx->str_index, buffer_info->stream_pic_input_count,
			buffer_info->stream_pic_parsed_count);
		return false;
	}

	return true;
}

static bool is_valid_frame(struct vpu_ctx *ctx, struct vb2_buffer *vb)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];

	if (!ctx || !vb)
		return false;
	if (ctx->first_data_flag && single_seq_info_format(q_data))
		return false;
	if (is_codec_config_data(vb))
		return false;

	return true;
}

static int increase_frame_num(struct vpu_ctx *ctx, struct vb2_buffer *vb)
{
	if (!ctx || !vb)
		return -EINVAL;

	if (is_valid_frame(ctx, vb)) {
		ctx->frm_dec_delay++;
		ctx->frm_dis_delay++;
		ctx->frm_total_num++;
		fill_stream_buffer_info(ctx);
	}
	ctx->first_data_flag = false;

	return 0;
}

static void enqueue_stream_data(struct vpu_ctx *ctx, uint32_t uStrBufIdx)
{
	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_SRC];
	void *input_buffer;
	uint32_t buffer_size;
	int frame_bytes;

	while (!list_empty(&This->drv_q)) {
		struct vb2_buffer *vb;

		if (!vpu_dec_stream_is_ready(ctx)) {
			vpu_dbg(LVL_INFO,
				"[%d] stream is not ready\n", ctx->str_index);
			break;
		}
		p_data_req = list_first_entry(&This->drv_q,
				typeof(*p_data_req), list);

		if (!p_data_req->vb2_buf)
			break;

		vb = p_data_req->vb2_buf;
		buffer_size = vb->planes[0].bytesused;
		input_buffer = (void *)vb2_plane_vaddr(vb, 0);

		if (ctx->start_code_bypass)
			frame_bytes = update_stream_addr_bypass(ctx, input_buffer, buffer_size);
		else if (ctx->first_data_flag || is_codec_config_data(vb))
			frame_bytes = insert_scode(ctx, SCODE_NEW_SEQUENCE, buffer_size, input_buffer);
		else
			frame_bytes = update_stream_addr_vpu(ctx, input_buffer, buffer_size, uStrBufIdx);

		if (frame_bytes == 0) {
			vpu_dbg(LVL_INFO, " %s no space to write\n", __func__);
			return;
		} else if (frame_bytes < 0) {
			vpu_dbg(LVL_WARN, "warning: incorrect input buffer data\n");
		} else {
			if (ctx->b_dis_reorder && !is_codec_config_data(vb)) {
				/* frame successfully written into the stream buffer if in special low latency mode
					mark that this frame should be flushed for decode immediately */
				frame_bytes += add_scode(ctx,
							 0,
							 BUFFLUSH_PADDING_TYPE,
							 true);
				record_log_info(ctx, LOG_PADDING, 0, 0);
			}

			increase_frame_num(ctx, vb);

			record_log_info(ctx, LOG_UPDATE_STREAM, 0, buffer_size);
			vpu_dec_receive_ts(ctx, vb, frame_bytes);
			This->process_count++;
		}

		list_del(&p_data_req->list);
		p_data_req->queued = false;
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	}
	if (list_empty(&This->drv_q) && ctx->eos_stop_received) {
		if (vpu_dec_is_active(ctx) && ctx->statistic.frame_input) {
			vpu_dbg(LVL_EVENT, "ctx[%d]: insert eos directly\n", ctx->str_index);
			if (ctx->statistic.eos_cnt) {
				vpu_dbg(LVL_WARN,
					"Warning : repeated eos at frame %ld (%ld), previous eos has been insert when input %ld frames(%ld bytes)\n",
					ctx->statistic.frame_input,
					ctx->total_write_bytes,
					ctx->statistic.eos_frames, ctx->statistic.eos_bytes);
			}
			ctx->statistic.eos_cnt++;
			ctx->statistic.eos_frames = ctx->statistic.frame_input;
			ctx->statistic.eos_bytes = ctx->total_write_bytes;

			if (add_scode(ctx, 0, EOS_PADDING_TYPE, true) > 0) {
				record_log_info(ctx, LOG_EOS, 0, 0);
				ctx->eos_stop_received = false;
				ctx->eos_stop_added = true;
			}
		} else	{
			ctx->eos_stop_received = false;
			send_eos_event(ctx);
		}
	}
}

//warn uStrIdx need to refine how to handle it
static void v4l2_update_stream_addr(struct vpu_ctx *ctx, uint32_t uStrBufIdx)
{
	struct queue_data *This = &ctx->q_data[V4L2_SRC];

	down(&This->drv_q_lock);
	enqueue_stream_data(ctx, uStrBufIdx);
	up(&This->drv_q_lock);
}

static void report_buffer_done(struct vpu_ctx *ctx, void *frame_info)
{
	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_DST];
	u_int32 *FrameInfo = (u_int32 *)frame_info;
	u_int32 fs_id = FrameInfo[0x0];
	uint32_t stride = FrameInfo[3];
	s64 timestamp = ((s32)FrameInfo[9] * NSEC_PER_SEC) + FrameInfo[10];
	bool b10BitFormat = is_10bit_format(ctx);
	int buffer_id;

	vpu_dbg(LVL_BIT_FUNC, "%s() fs_id=%d, ulFsLumaBase[0]=%x, stride=%d, b10BitFormat=%d, ctx->seqinfo.uBitDepthLuma=%d\n",
			__func__, fs_id, FrameInfo[1], stride, b10BitFormat, ctx->seqinfo.uBitDepthLuma);
	vpu_dbg(LVL_BIT_TS, "[OUTPUT TS]%32lld\n", timestamp);
	ctx->capture_ts = timestamp;
	v4l2_update_stream_addr(ctx, 0);

	buffer_id = find_buffer_id(ctx, FrameInfo[1]);
	if (buffer_id == -1) {
		vpu_err("%s() ctx[%d] not find buffer id: %d, addr: 0x%x\n",
				__func__, ctx->str_index, fs_id, FrameInfo[1]);
		return;
	}

	if (buffer_id != fs_id) {
		if (fs_id == MEDIA_PLAYER_SKIPPED_FRAME_ID) {
			down(&This->drv_q_lock);
			p_data_req = &This->vb2_reqs[buffer_id];
			set_data_req_status(p_data_req, FRAME_SKIP);
			up(&This->drv_q_lock);

			send_skip_event(ctx);
			ctx->frm_dis_delay--;
			return;
		}
		vpu_err("error: find buffer_id(%d) and firmware return id(%d) doesn't match\n",
				buffer_id, fs_id);
	}

	ctx->frm_dis_delay--;
	vpu_dbg(LVL_INFO, "frame total: %d; depth: %d; delay: [dec, dis] = [%d, %d]\n", ctx->frm_total_num,
		vpu_frm_depth, ctx->frm_dec_delay, ctx->frm_dis_delay);

	down(&This->drv_q_lock);
	p_data_req = &This->vb2_reqs[buffer_id];

	if (p_data_req->status != FRAME_DECODED)
		vpu_err("error: buffer(%d) need to set FRAME_READY, but previous state %s is not FRAME_DECODED\n",
				buffer_id, bufstat[p_data_req->status]);
	set_data_req_status(p_data_req, FRAME_READY);
	if (p_data_req->vb2_buf) {
		p_data_req->vb2_buf->planes[0].bytesused = This->sizeimage[0];
		p_data_req->vb2_buf->planes[1].bytesused = This->sizeimage[1];
		p_data_req->vb2_buf->timestamp = timestamp;
		if (p_data_req->vb2_buf->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(p_data_req->vb2_buf,
					VB2_BUF_STATE_DONE);
		else
			vpu_err("warning: wait_rst_done(%d) check buffer(%d) state(%d)\n",
					ctx->wait_rst_done, buffer_id, p_data_req->vb2_buf->state);
	}
	up(&This->drv_q_lock);
	vpu_dbg(LVL_INFO, "leave %s\n", __func__);
}

static void send_skip_event(struct vpu_ctx *ctx)
{
	const struct v4l2_event ev = {
		.id = 0,
		.type = V4L2_EVENT_SKIP,
		.u.data[0] = 0xff,
	};

	if (!ctx)
		return;

	ctx->statistic.skipped_frame_count++;
	vpu_dbg(LVL_INFO, "send skip event\n");
	v4l2_event_queue_fh(&ctx->fh, &ev);
}

static void send_eos_event(struct vpu_ctx *ctx)
{
	const struct v4l2_event ev = {
		.id = 0,
		.type = V4L2_EVENT_EOS
	};

	if (!ctx)
		return;

	vpu_dbg(LVL_BIT_FLOW, "send eos event\n");
	v4l2_event_queue_fh(&ctx->fh, &ev);
}

static void send_source_change_event(struct vpu_ctx *ctx)
{
	const struct v4l2_event ev = {
		.id = 0,
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION
	};

	if (!ctx)
		return;

	ctx->res_change_send_count++;
	vpu_dbg(LVL_BIT_FLOW, "ctx[%d] send source change event : %dx%d\n",
		ctx->str_index, ctx->seqinfo.uHorRes, ctx->seqinfo.uVerRes);
	v4l2_event_queue_fh(&ctx->fh, &ev);
}

static void reset_mbi_dcp_count(struct vpu_ctx *ctx)
{
	free_mbi_buffers(ctx);
	ctx->mbi_index = 0;
	ctx->mbi_count = 0;
	ctx->req_mbi_count = 0;
	free_dcp_buffers(ctx);
	ctx->dcp_index = 0;
	ctx->dcp_count = 0;
	ctx->req_dcp_count = 0;
}

static void reset_frame_buffer(struct vpu_ctx *ctx)
{
	struct queue_data *queue;
	struct vb2_data_req *p_data_req = NULL;
	int i;

	queue = &ctx->q_data[V4L2_DST];
	for (i = 0; i < queue->vb2_q.num_buffers; i++) {
		p_data_req = &queue->vb2_reqs[i];

		if (p_data_req->status == FRAME_ALLOC ||
		    p_data_req->status == FRAME_RELEASE)
			continue;
		set_data_req_status(p_data_req, FRAME_RELEASE);
		release_frame_buffer(ctx, ctx->str_index, p_data_req);
	}
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

static void add_buffer_to_queue(struct queue_data *q_data, struct vb2_data_req *data_req)
{
	if (!q_data || !data_req)
		return;
	if (data_req->queued == true)
		return;
	if (data_req->vb2_buf->state != VB2_BUF_STATE_ACTIVE)
		return;

	list_add_tail(&data_req->list, &q_data->drv_q);
	data_req->queued = true;
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

	return 0;
}

static struct vb2_data_req *get_frame_buffer(struct queue_data *queue)
{
	struct vb2_data_req *p_data_req;
	struct vb2_data_req *p_temp;
	bool found = false;

	list_for_each_entry_safe(p_data_req, p_temp, &queue->drv_q, list) {
		if (!p_data_req->vb2_buf)
			continue;
		if (p_data_req->status != FRAME_ALLOC)
			continue;
		if (verify_frame_buffer_size(queue, p_data_req)) {
			list_del(&p_data_req->list);
			p_data_req->queued = false;
			found = true;
			break;
		}
	}

	if (!found)
		return NULL;

	return p_data_req;
}

static void respond_req_frame_abnormal(struct vpu_ctx *ctx)
{
	u32 local_cmddata[10];

	ctx->req_frame_count--;
	if (ctx->firmware_stopped)
		return;

	memset(local_cmddata, 0, sizeof(local_cmddata));
	local_cmddata[0] = (ctx->seqinfo.uActiveSeqTag + 0xf0)<<24;
	local_cmddata[6] = MEDIAIP_FRAME_REQ;
	v4l2_vpu_send_cmd(ctx, ctx->str_index,
			VID_API_CMD_FS_ALLOC, 7, local_cmddata);
}

static bool alloc_frame_buffer(struct vpu_ctx *ctx,
				struct queue_data *queue)
{
	struct vb2_data_req *p_data_req;
	u32 local_cmddata[10];
	dma_addr_t LumaAddr = 0;
	dma_addr_t ChromaAddr = 0;

	if (!ctx || !queue->enable || ctx->b_firstseq || ctx->firmware_stopped)
		return false;

	p_data_req = get_frame_buffer(queue);
	if (!p_data_req)
		return false;

	LumaAddr = p_data_req->phy_addr[0] + p_data_req->data_offset[0];
	ChromaAddr = p_data_req->phy_addr[1] + p_data_req->data_offset[1];
	vpu_dbg(LVL_INFO, "%s() :LumaAddr(%llx) ChromaAddr(%llx) buf_id (%d)\n",
			__func__, LumaAddr, ChromaAddr, p_data_req->id);

	p_data_req->seq_tag = ctx->seqinfo.uActiveSeqTag;
	memset(local_cmddata, 0, sizeof(local_cmddata));
	local_cmddata[0] = p_data_req->id | (p_data_req->seq_tag << 24);
	local_cmddata[1] = LumaAddr;
	local_cmddata[2] = local_cmddata[1] + queue->sizeimage[0]/2;
	local_cmddata[3] = ChromaAddr;
	local_cmddata[4] = local_cmddata[3] + queue->sizeimage[1]/2;
	local_cmddata[5] = queue->stride;
	local_cmddata[6] = MEDIAIP_FRAME_REQ;

	ctx->req_frame_count--;
	v4l2_vpu_send_cmd(ctx, ctx->str_index,
			VID_API_CMD_FS_ALLOC, 7, local_cmddata);
	set_data_req_status(p_data_req, FRAME_FREE);
	vpu_dbg(LVL_INFO,
		"VID_API_CMD_FS_ALLOC, ctx[%d] vb2_buf=%p, id=%d\n",
		ctx->str_index, p_data_req->vb2_buf, p_data_req->id);

	return true;
}

static bool alloc_mbi_to_firmware(struct vpu_ctx *ctx,
				struct queue_data *queue)
{
	struct dma_buffer *buffer;
	u_int32 local_cmddata[10];
	u32 index;

	if (!ctx || !queue || !ctx->mbi_size)
		return false;

	buffer = &ctx->mbi_buffer[ctx->mbi_index];
	if (!buffer->dma_virt || buffer->dma_size < ctx->mbi_size)
		return false;

	index = ctx->str_index;
	local_cmddata[0] = ctx->mbi_index | (ctx->seqinfo.uActiveSeqTag<<24);
	local_cmddata[1] = ctx->mbi_buffer[ctx->mbi_index].dma_phy;
	local_cmddata[2] = ctx->mbi_buffer[ctx->mbi_index].dma_size;
	local_cmddata[3] = 0;
	local_cmddata[4] = 0;
	local_cmddata[5] = 0;
	local_cmddata[6] = MEDIAIP_MBI_REQ;
	v4l2_vpu_send_cmd(ctx, index, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
	vpu_dbg(LVL_INFO, "VID_API_CMD_FS_ALLOC, ctx[%d] eType=%d, index=%d\n",
		ctx->str_index, MEDIAIP_MBI_REQ, ctx->mbi_index);
	ctx->mbi_index++;

	return true;
}

static bool alloc_dcp_to_firmware(struct vpu_ctx *ctx,
				struct queue_data *queue)
{
	struct dma_buffer *buffer;
	u_int32 local_cmddata[10];
	u32 index;

	if (!ctx || !queue || !ctx->dcp_size)
		return false;

	buffer = &ctx->dcp_buffer[ctx->dcp_index];
	if (!buffer->dma_virt || buffer->dma_size < ctx->dcp_size)
		return false;

	index = ctx->str_index;
	local_cmddata[0] = ctx->dcp_index | (ctx->seqinfo.uActiveSeqTag<<24);
	local_cmddata[1] = ctx->dcp_buffer[ctx->dcp_index].dma_phy;
	local_cmddata[2] = ctx->dcp_size;
	local_cmddata[3] = 0;
	local_cmddata[4] = 0;
	local_cmddata[5] = 0;
	local_cmddata[6] = MEDIAIP_DCP_REQ;
	v4l2_vpu_send_cmd(ctx, index, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
	vpu_dbg(LVL_INFO, "VID_API_CMD_FS_ALLOC, ctx[%d] eType=%d, index=%d\n",
		ctx->str_index, MEDIAIP_DCP_REQ, ctx->dcp_index);
	ctx->dcp_index++;
	return true;
}

static void check_wait_res_changed(struct vpu_ctx *ctx)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_DST];
	struct vb2_data_req *p_data_req;
	struct vb2_data_req *p_temp;

	if (!q_data->enable)
		return;

	list_for_each_entry_safe(p_data_req, p_temp, &q_data->drv_q, list) {
		if (!p_data_req->vb2_buf)
			continue;
		if (verify_frame_buffer_size(q_data, p_data_req)) {
			ctx->res_change_done_count++;
			ctx->wait_res_change_done = false;
			vpu_dbg(LVL_BIT_FLOW,
				"ctx[%d] res change done, %d, %d, %d\n",
				ctx->str_index,
				ctx->res_change_occu_count,
				ctx->res_change_send_count,
				ctx->res_change_done_count);
				vpu_calculate_performance(ctx, 0xff, "first provide buffer");
			break;
		}
	}
}

static void respond_req_frame(struct vpu_ctx *ctx,
				struct queue_data *queue,
				bool abnormal)
{
	if (ctx->wait_res_change_done) {
		if (ctx->seek_flag)
			return;
		check_wait_res_changed(ctx);
	}

	if (ctx->wait_res_change_done)
		return;

	while (ctx->req_frame_count > 0) {
		if (abnormal) {
			respond_req_frame_abnormal(ctx);
			continue;
		}
		if (!queue->enable)
			break;
		if (!alloc_frame_buffer(ctx, queue))
			break;
	}
	while (ctx->mbi_index < ctx->mbi_count) {
		if (!queue->enable)
			break;
		if (!alloc_mbi_to_firmware(ctx, queue))
			break;
	}
	while (ctx->dcp_index < ctx->dcp_count) {
		if (!queue->enable)
			break;
		if (!alloc_dcp_to_firmware(ctx, queue))
			break;
	}
}

static void release_frame_buffer(struct vpu_ctx *ctx,
				u32 uStrIdx,
				struct vb2_data_req *p_data_req)
{
	u32 local_cmddata[1];

	local_cmddata[0] = p_data_req->id | (p_data_req->seq_tag << 24);
	v4l2_vpu_send_cmd(ctx, uStrIdx,
			VID_API_CMD_FS_RELEASE, 1, local_cmddata);
	set_data_req_status(p_data_req, FRAME_ALLOC);
}

static void get_seq_info(MediaIPFW_Video_SeqInfo *pSeqInfo,
			u32 *event_data,
			MediaIPFW_Video_SeqInfo *pRpcSeqInfo)
{
	memset(pSeqInfo, 0, sizeof(*pSeqInfo));

	if (event_data && event_data[0]) {
		pSeqInfo->uNumRefFrms = event_data[0];
		pSeqInfo->uNumDPBFrms = event_data[1];
		pSeqInfo->uNumDFEAreas = event_data[2];
		pSeqInfo->uProgressive = event_data[3];
		pSeqInfo->uVerRes = event_data[4];
		pSeqInfo->uHorRes = event_data[5];
		pSeqInfo->uParWidth = event_data[6];
		pSeqInfo->uParHeight = event_data[7];
		pSeqInfo->FrameRate = event_data[8];
		pSeqInfo->UDispAspRatio = event_data[9];
		pSeqInfo->uLevelIDC = event_data[10];
		pSeqInfo->uVerDecodeRes = event_data[11];
		pSeqInfo->uHorDecodeRes = event_data[12];
		pSeqInfo->uBitDepthLuma = event_data[13];
		pSeqInfo->uBitDepthChroma = event_data[14];
		pSeqInfo->uChromaFmt = event_data[15];
		pSeqInfo->uColorDesc = event_data[16];
		pSeqInfo->uTransferChars = event_data[17];
		pSeqInfo->uMatrixCoeffs = event_data[18];
		pSeqInfo->uVideoFullRangeFlag = event_data[19];
		pSeqInfo->uVUIPresent = event_data[20];
		pSeqInfo->uMVCNumViews = event_data[21];
		pSeqInfo->uFrameCropValid = event_data[22];
		pSeqInfo->uFrameCropLeftOffset = event_data[23];
		pSeqInfo->uFrameCropRightOffset = event_data[24];
		pSeqInfo->uFrameCropTopOffset = event_data[25];
		pSeqInfo->uFrameCropBottomOffset = event_data[25];
		pSeqInfo->uActiveSeqTag = event_data[27];
		return;
	}

	memcpy(pSeqInfo, pRpcSeqInfo, sizeof(*pSeqInfo));
}

static bool check_seq_info_is_valid(u32 ctx_id, MediaIPFW_Video_SeqInfo *info)
{
	if (!info)
		return false;

	if (!info->uHorRes || !info->uVerRes) {
		vpu_err("ctx[%d] invalid seq info : %d x %d\n",
			ctx_id, info->uHorRes, info->uVerRes);
		return false;
	}
	if (info->uHorRes > VPU_DEC_MAX_WIDTH ||
			info->uVerRes > VPU_DEC_MAX_HEIGTH) {
		vpu_err("ctx[%d] invalid seq info : %d x %d\n",
			ctx_id, info->uHorRes, info->uVerRes);
		return false;
	}

	return true;
}

static bool check_res_is_changed(struct vpu_ctx *ctx,
				MediaIPFW_Video_SeqInfo *pSeqInfo)
{
	if (ctx->seqinfo.uHorDecodeRes != pSeqInfo->uHorDecodeRes)
		return true;
	if (ctx->seqinfo.uVerDecodeRes != pSeqInfo->uVerDecodeRes)
		return true;
	if (ctx->seqinfo.uNumRefFrms != pSeqInfo->uNumRefFrms)
		return true;
	if (ctx->seqinfo.uNumDPBFrms != pSeqInfo->uNumDPBFrms)
		return true;
	if (ctx->seqinfo.uBitDepthLuma != pSeqInfo->uBitDepthLuma)
		return true;
	if (ctx->seqinfo.uBitDepthChroma != pSeqInfo->uBitDepthChroma)
		return true;

	return false;
}

static bool check_is_need_reset_after_abort(struct vpu_ctx *ctx)
{
	if (!ctx)
		return false;
	if (ctx->b_firstseq)
		return true;
	if (!ctx->frame_decoded)
		return true;

	return false;
}

static struct vpu_dec_perf_queue *get_dec_perf_queue(struct vpu_ctx *ctx)
{
	struct vpu_dec_perf_queue *perf;

	perf = vzalloc(sizeof(*perf));
	if (!perf)
		return NULL;

	atomic64_add(sizeof(*perf), &ctx->statistic.total_alloc_size);

	return perf;
}

static void put_dec_perf_queue(struct vpu_ctx *ctx, struct vpu_dec_perf_queue *perf)
{
	if (!perf)
		return;

	vfree(perf);
	atomic64_sub(sizeof(*perf), &ctx->statistic.total_alloc_size);
}

static void cleanup_perf_queue(struct vpu_ctx *ctx)
{
	struct vpu_dec_perf_queue *perf;
	struct vpu_dec_perf_queue *tmp;

	mutex_lock(&ctx->perf_lock);
	list_for_each_entry_safe(perf, tmp, &ctx->perf_q, list) {
		list_del_init(&perf->list);
		put_dec_perf_queue(ctx, perf);
	}
	mutex_unlock(&ctx->perf_lock);
}

static void vpu_calculate_performance(struct vpu_ctx *ctx,  u_int32 uEvent, const char *str)
{
	u_int64 Time;
	u_int64 total_Time;
	struct timespec64 ts;
	struct vpu_dec_perf_queue *perf;

	if (!vpu_show_perf_ena)
		return;
	if (!(vpu_show_perf_idx & (1<<ctx->str_index)))
		return;

	ktime_get_real_ts64(&ts);
	Time = ((ts.tv_sec * 1000000000ULL) + ts.tv_nsec) / 1000000ULL;

	switch (uEvent) {
	case VID_API_EVENT_PIC_DECODED:
		if (ctx->statistic.event[VID_API_EVENT_PIC_DECODED] == 1) {
			ctx->perf_time.first_decoded_time = Time - 1;
			ctx->perf_time.last_decoded_time = Time - 1;
		}

		ctx->perf_time.cur_decoded_interv = Time - ctx->perf_time.last_decoded_time;
		total_Time = Time - ctx->perf_time.first_decoded_time;
		ctx->perf_time.decoded_fps = ctx->statistic.event[VID_API_EVENT_PIC_DECODED] * 1000ULL / total_Time;
		ctx->perf_time.last_decoded_time = Time;
		if (vpu_show_perf_ent & VPU_DECODED_EVENT_PERF_MASK)
			vpu_dbg(LVL_WARN, "[%2d] dec[%8ld]  interv: %8ld ms  fps: %ld\n",
				ctx->str_index,
				ctx->statistic.event[VID_API_EVENT_PIC_DECODED],
				ctx->perf_time.cur_decoded_interv,
				ctx->perf_time.decoded_fps);

		break;
	case VID_API_EVENT_FRAME_BUFF_RDY:
		if (ctx->statistic.event[VID_API_EVENT_FRAME_BUFF_RDY] == 1) {
			ctx->perf_time.first_ready_time = Time - 1;
			ctx->perf_time.last_ready_time = Time - 1;
		}

		ctx->perf_time.cur_ready_interv = Time - ctx->perf_time.last_ready_time;
		total_Time = Time - ctx->perf_time.first_ready_time;
		ctx->perf_time.ready_fps = ctx->statistic.event[VID_API_EVENT_FRAME_BUFF_RDY] * 1000ULL / total_Time;
		ctx->perf_time.last_ready_time = Time;
		if (vpu_show_perf_ent & VPU_READY_EVENT_PERF_MASK)
			vpu_dbg(LVL_WARN, "[%2d] rdy[%8ld]  interv: %8ld ms  fps: %ld\n",
				ctx->str_index,
				ctx->statistic.event[VID_API_EVENT_FRAME_BUFF_RDY],
				ctx->perf_time.cur_ready_interv,
				ctx->perf_time.ready_fps);

		break;
	default:
		break;
	}

	if (ctx->statistic.event[VID_API_EVENT_FRAME_BUFF_RDY] > 1)
		return;

	if (str != NULL) {
		perf = get_dec_perf_queue(ctx);
		if (!perf) {
			vpu_err("get_dec_perf_queue fail\n");
			return;
		}

		scnprintf(perf->str, (VPU_MAX_STEP_STRING_LENGTH - 1), str);
		perf->time = Time;

		mutex_lock(&ctx->perf_lock);
		list_add_tail(&perf->list, &ctx->perf_q);
		mutex_unlock(&ctx->perf_lock);
	}
}

static void vpu_ctx_show_statstic(struct vpu_ctx *ctx, const char *desc)
{
	char buf[256];
	int num = 0;
	int size = sizeof(buf);
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc = get_str_buffer_desc(ctx);

	num += scnprintf(buf + num, size - num, "[%d] ", ctx->str_index);
	num += scnprintf(buf + num, size - num, "%s,", desc ? desc : "");
	num += scnprintf(buf + num, size - num, "%c%c%c%c ",
			ctx->q_data[V4L2_SRC].fourcc,
			ctx->q_data[V4L2_SRC].fourcc >> 8,
			ctx->q_data[V4L2_SRC].fourcc >> 16,
			ctx->q_data[V4L2_SRC].fourcc >> 24);
	num += scnprintf(buf + num, size - num, "%dx%d,",
			ctx->q_data[V4L2_DST].width,
			ctx->q_data[V4L2_DST].height);
	num += scnprintf(buf + num, size - num, "input:%ld,",
			ctx->statistic.frame_input);
	num += scnprintf(buf + num, size - num, "parse:%ld,",
			ctx->statistic.frame_hdr);
	num += scnprintf(buf + num, size - num, "decode:%ld,",
			ctx->statistic.frame_decoded);
	num += scnprintf(buf + num, size - num, "ready:%ld,",
			ctx->statistic.frame_ready);
	num += scnprintf(buf + num, size - num, "skip:%ld,",
			ctx->statistic.skipped_frame_count);
	num += scnprintf(buf + num, size - num, "disp:%ld,",
			ctx->statistic.frame_display);
	num += scnprintf(buf + num, size - num, "bytes:%ld %ld,",
			ctx->total_qbuf_bytes, ctx->total_write_bytes);
	num += scnprintf(buf + num, size - num, "eos:%ld %ld %ld",
			ctx->statistic.eos_cnt,
			ctx->statistic.eos_frames,
			ctx->statistic.eos_bytes);
	num += scnprintf(buf + num, size - num,
			",wptr:0x%x 0x%x 0x%x",
			pStrBufDesc->wptr - pStrBufDesc->start,
			pStrBufDesc->rptr - pStrBufDesc->start,
			ctx->beginning - pStrBufDesc->start);
	vpu_dbg(LVL_BIT_STRM_INFO, "%s\n", buf);
};

static void vpu_ctx_clear_statistic_on_abort(struct vpu_ctx *ctx)
{

	vpu_ctx_show_statstic(ctx, "abt");

	ctx->statistic.eos_cnt = 0;
	ctx->statistic.eos_frames = 0;
	ctx->statistic.eos_bytes = 0;
	ctx->statistic.frame_input = 0;
	ctx->statistic.frame_hdr = 0;
	ctx->statistic.frame_decoded = 0;
	ctx->statistic.frame_ready = 0;
	ctx->statistic.frame_display = 0;
};

static void vpu_api_event_handler(struct vpu_ctx *ctx, u_int32 uStrIdx, u_int32 uEvent, u_int32 *event_data)
{
	struct vpu_dev *dev;
	pDEC_RPC_HOST_IFACE pSharedInterface;
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;

	vpu_log_event(uEvent, uStrIdx);

	pStrBufDesc = get_str_buffer_desc(ctx);
	record_log_info(ctx, LOG_EVENT, uEvent, pStrBufDesc->rptr);

	if (ctx == NULL) {
		vpu_err("receive event: 0x%X after instance released, ignore it\n", uEvent);
		return;
	}

	if (ctx->firmware_stopped) {
		switch (uEvent) {
		case VID_API_EVENT_START_DONE:
		case VID_API_EVENT_FIRMWARE_XCPT:
			break;
		case VID_API_EVENT_FIFO_LOW:
			return;
		default:
			vpu_err("receive event: 0x%X after stopped, ignore it\n", uEvent);
			return;
		}
	}

	dev = ctx->dev;
	pSharedInterface = (pDEC_RPC_HOST_IFACE)dev->shared_mem.shared_mem_vir;

	vpu_dec_response_cmd(ctx, uEvent);
	switch (uEvent) {
	case VID_API_EVENT_START_DONE:
		vpu_dbg(LVL_BIT_FLOW, "ctx[%d] START DONE\n", ctx->str_index);
		ctx->firmware_stopped = false;
		ctx->firmware_finished = false;
		ctx->req_frame_count = 0;
		reset_mbi_dcp_count(ctx);
		vpu_calculate_performance(ctx, uEvent, "start done");
		break;
	case VID_API_EVENT_STOPPED: {
		vpu_dbg(LVL_INFO, "receive VID_API_EVENT_STOPPED\n");
		vpu_dbg(LVL_BIT_FLOW,
			"ctx[%d] STOPPED, output qbuf(%ld), dqbuf(%ld)\n",
			ctx->str_index,
			ctx->q_data[V4L2_SRC].qbuf_count,
			ctx->q_data[V4L2_SRC].dqbuf_count);
		ctx->firmware_stopped = true;
		ctx->frame_decoded = false;
		ctx->wait_rst_done = false;
		/* This also can fix an Andorid case indirectly:
		 * seek in the beginning, but has not do capture port
		 * streamoff/on when receive res changed event, then will cause
		 * seek_flag status incorrect.
		 * If abort before receive seq_hdr_found evnt will call stop cmd
		 * to fw, then will reset seek_flag and wait_res_change_done.
		 */
		ctx->wait_res_change_done = false;
		ctx->seek_flag = false;
		ctx->res_change_occu_count = 0;
		ctx->res_change_send_count = 0;
		ctx->res_change_done_count = 0;
		ctx->fifo_low = false;
		down(&ctx->q_data[V4L2_DST].drv_q_lock);
		ctx->b_firstseq = true;
		respond_req_frame(ctx, &ctx->q_data[V4L2_DST], true);
		reset_mbi_dcp_count(ctx);
		memset(&ctx->seqinfo, 0, sizeof(MediaIPFW_Video_SeqInfo));
		up(&ctx->q_data[V4L2_DST].drv_q_lock);
		vpu_dec_cleanup_event(ctx);
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
		struct queue_data *This = &ctx->q_data[V4L2_DST];
		u_int32 uDpbmcCrc;
		size_t wr_size;
		struct vb2_data_req *p_data_req = NULL;
		u32 consumed_count = event_data[13];

		down(&ctx->q_data[V4L2_SRC].drv_q_lock);
		ctx->statistic.frame_decoded++;
		up(&ctx->q_data[V4L2_SRC].drv_q_lock);

		if (This->vdec_std == VPU_VIDEO_HEVC)
			uDpbmcCrc = pPerfDcpInfo->uDBEDpbCRC[0];
		else
			uDpbmcCrc = pPerfInfo->uMemCRC;
		if (vpu_frmcrcdump_ena) {
			wr_size = kernel_write(ctx->crc_fp, &uDpbmcCrc, sizeof(uDpbmcCrc), &ctx->pos);
			ctx->pos += wr_size;
		}

		vpu_dbg(LVL_BIT_FRAME_BYTES, "PICINFO GET: uPicType:%d uPicStruct:%d uPicStAddr:0x%x uFrameStoreID:%d uPercentInErr:%d, uRbspBytesCount=%d, uDpbmcCrc:%x\n",
			pPicInfo[uStrIdx].uPicType, pPicInfo[uStrIdx].uPicStruct,
			pPicInfo[uStrIdx].uPicStAddr, pPicInfo[uStrIdx].uFrameStoreID,
			pPicInfo[uStrIdx].uPercentInErr, pPerfInfo->uRbspBytesCount,
			uDpbmcCrc);
		vpu_dbg(LVL_BIT_FRAME_BYTES, "PICINFO GET: ulLumBaseAddr[0]=%x, uPicStartAddr=0x%x, pQMeterInfo:%p, pPicInfo:%p, pDispInfo:%p, pPerfInfo:%p, pPerfDcpInfo:%p\n",
			event_data[0], uPicStartAddr, pQMeterInfo, pPicInfo,
			pDispInfo, pPerfInfo, pPerfDcpInfo);

		buffer_id = find_buffer_id(ctx, event_data[0]);
		if (buffer_id == -1) {
			vpu_err("error: %s() ctx[%d] not find buffer id: %d, addr: 0x%x\n",
					__func__, ctx->str_index, uDecFrmId, event_data[0]);
			break;
		}

		p_data_req = &This->vb2_reqs[buffer_id];
		set_data_req_status(p_data_req, FRAME_DECODED);
		if (ctx->seqinfo.uProgressive == 1)
			p_data_req->bfield = false;
		else
			p_data_req->bfield = true;

		This->process_count++;
		if (ctx->statistic.event[VID_API_EVENT_PIC_DECODED] == 1)
			vpu_calculate_performance(ctx, uEvent, "first decoded");
		else
			vpu_calculate_performance(ctx, uEvent, NULL);

		if (ctx->frm_dec_delay >= consumed_count)
			ctx->frm_dec_delay -= consumed_count;
		else
			ctx->frm_dec_delay = 0;
		ctx->fifo_low = false;
		ctx->frame_decoded = true;
		v4l2_update_stream_addr(ctx, 0);
		}
		break;
	case VID_API_EVENT_SEQ_HDR_FOUND: {
		MediaIPFW_Video_SeqInfo *pSeqInfo = (MediaIPFW_Video_SeqInfo *)dev->shared_mem.seq_mem_vir;
		MediaIPFW_Video_SeqInfo info;
//		MediaIPFW_Video_FrameBuffer *pStreamFrameBuffer = &pSharedInterface->StreamFrameBuffer[uStrIdx];
//		MediaIPFW_Video_FrameBuffer *pStreamDCPBuffer = &pSharedInterface->StreamDCPBuffer[uStrIdx];
		MediaIPFW_Video_PitchInfo   *pStreamPitchInfo = &pSharedInterface->StreamPitchInfo[uStrIdx];
		unsigned int num = pSharedInterface->SeqInfoTabDesc.uNumSizeDescriptors;
		int wait_times = 0;

		get_seq_info(&info, event_data, &pSeqInfo[ctx->str_index]);
		if (!check_seq_info_is_valid(ctx->str_index, &info)) {
			vpu_dec_event_decode_error(ctx);
			break;
		}

		while (ctx->wait_res_change_done && wait_times++ < 100) {
			if (!vpu_dec_is_active(ctx))
				break;
			if (ctx->wait_rst_done)
				break;
			mdelay(10);
		}
		if (!vpu_dec_is_active(ctx))
			break;
		if (ctx->wait_res_change_done)
			vpu_dbg(LVL_WARN, "warning: ctx[%d] update seq info when waiting res change\n",
				ctx->str_index);

		down(&ctx->q_data[V4L2_DST].drv_q_lock);
		respond_req_frame(ctx, &ctx->q_data[V4L2_DST], true);
		if (check_res_is_changed(ctx, &info))
			ctx->res_change_occu_count++;
		memcpy(&ctx->seqinfo, &info, sizeof(MediaIPFW_Video_SeqInfo));
		up(&ctx->q_data[V4L2_DST].drv_q_lock);

		parse_frame_interval_from_seqinfo(ctx, &ctx->seqinfo);
		vpu_dbg(LVL_BIT_FLOW,
			"ctx[%d] SEQINFO GET: uHorRes:%d uVerRes:%d uHorDecodeRes:%d uVerDecodeRes:%d\n",
			ctx->str_index,
			ctx->seqinfo.uHorRes,
			ctx->seqinfo.uVerRes,
			ctx->seqinfo.uHorDecodeRes,
			ctx->seqinfo.uVerDecodeRes);
		vpu_dbg(LVL_BIT_FLOW,
			"ctx[%d] SEQINFO GET: uNumDPBFrms:%d, num:%d, uNumRefFrms:%d, uNumDFEAreas:%d, scan lines: %s\n",
			ctx->str_index,
			ctx->seqinfo.uNumDPBFrms,
			num,
			ctx->seqinfo.uNumRefFrms,
			ctx->seqinfo.uNumDFEAreas & 0xff,
			ctx->seqinfo.uProgressive ? "progressive" : "interlaced");
		vpu_dbg(LVL_BIT_FLOW,
			"uColorDesc = %d, uTransferChars = %d, uMatrixCoeffs = %d, uVideoFullRangeFlag = %d, uVUIPresent = %d\n",
			ctx->seqinfo.uColorDesc,
			ctx->seqinfo.uTransferChars,
			ctx->seqinfo.uMatrixCoeffs,
			ctx->seqinfo.uVideoFullRangeFlag,
			ctx->seqinfo.uVUIPresent);

		down(&ctx->q_data[V4L2_DST].drv_q_lock);
		vpu_dec_convert_color_iso_aspect_to_v4l2_aspect(ctx,
				ctx->seqinfo.uColorDesc,
				ctx->seqinfo.uTransferChars,
				ctx->seqinfo.uMatrixCoeffs,
				ctx->seqinfo.uVideoFullRangeFlag);
		vdec_setup_capture_size(ctx);
		ctx->dcp_size = get_dcp_size(ctx);
		if (ctx->b_firstseq) {
			ctx->b_firstseq = false;
			ctx->mbi_size = get_mbi_size(&ctx->q_data[V4L2_DST]);
			reset_frame_buffer(ctx);
			ctx->q_data[V4L2_DST].enable = false;
			ctx->wait_res_change_done = true;
			send_source_change_event(ctx);
			pStreamPitchInfo->uFramePitch = 0x4000;
			vpu_calculate_performance(ctx, uEvent, "seq_hdr_found");
		}
		up(&ctx->q_data[V4L2_DST].drv_q_lock);
		}
		break;
	case VID_API_EVENT_PIC_HDR_FOUND:
		down(&ctx->q_data[V4L2_SRC].drv_q_lock);
		ctx->statistic.frame_hdr++;
		up(&ctx->q_data[V4L2_SRC].drv_q_lock);
		break;
	case VID_API_EVENT_REQ_FRAME_BUFF: {
		MEDIA_PLAYER_FSREQ *pFSREQ = (MEDIA_PLAYER_FSREQ *)event_data;
		struct queue_data *This = &ctx->q_data[V4L2_DST];

		vpu_dbg(LVL_INFO, "VID_API_EVENT_REQ_FRAME_BUFF, type=%d, size=%ld\n", pFSREQ->eType, sizeof(MEDIA_PLAYER_FSREQ));
		down(&This->drv_q_lock);
		switch (pFSREQ->eType) {
		case MEDIAIP_DCP_REQ:
			ctx->req_dcp_count++;
			if (ctx->req_dcp_count > MAX_DCP_NUM) {
				vpu_err("error: request dcp count over %d\n",
						MAX_DCP_NUM);
				ctx->req_dcp_count = MAX_DCP_NUM;
			}
			break;
		case MEDIAIP_MBI_REQ:
			ctx->req_mbi_count++;
			if (ctx->req_mbi_count > MAX_MBI_NUM) {
				vpu_err("error: request mbi count over %d\n",
						MAX_MBI_NUM);
				ctx->req_mbi_count = MAX_MBI_NUM;
			}
			break;
		default:
			ctx->req_frame_count++;
			break;
		}
		vpu_dec_alloc_mbi_dcp(ctx);
		respond_req_frame(ctx, This, false);
		up(&This->drv_q_lock);
		}
		break;
	case VID_API_EVENT_REL_FRAME_BUFF: {
		MEDIA_PLAYER_FSREL *fsrel = (MEDIA_PLAYER_FSREL *)event_data;
		struct queue_data *This = &ctx->q_data[V4L2_DST];
		struct vb2_data_req *p_data_req;

		down(&This->drv_q_lock);
		if (fsrel->eType == MEDIAIP_FRAME_REQ) {
			p_data_req = &This->vb2_reqs[fsrel->uFSIdx];
			if (!p_data_req->vb2_buf) {
				vpu_dbg(LVL_INFO,
					"frame[%d] buffer already freed!\n",
					p_data_req->id);
				up(&This->drv_q_lock);
				break;
			}

			if (ctx->wait_rst_done != true && p_data_req->status != FRAME_READY) {
				vpu_dbg(LVL_INFO, "warning: normal release and previous status %s, frame not for display, queue the buffer to list again\n",
						bufstat[p_data_req->status]);

				if (p_data_req->status == FRAME_DECODED)
					send_skip_event(ctx);
			}
			if (p_data_req->status != FRAME_ALLOC) {
				set_data_req_status(p_data_req, FRAME_RELEASE);
				release_frame_buffer(ctx, uStrIdx, p_data_req);
			} else {
				vpu_dbg(LVL_INFO,
					"frame[%d] already released\n",
					p_data_req->id);
			}
			add_buffer_to_queue(This, p_data_req);
			respond_req_frame(ctx, This, false);
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

		down(&ctx->q_data[V4L2_SRC].drv_q_lock);
		ctx->statistic.frame_ready++;
		up(&ctx->q_data[V4L2_SRC].drv_q_lock);

		report_buffer_done(ctx, FrameInfo);
		if (ctx->statistic.event[VID_API_EVENT_FRAME_BUFF_RDY] == 1)
			vpu_calculate_performance(ctx, uEvent, "first ready");
		else
			vpu_calculate_performance(ctx, uEvent, NULL);
	}
		break;
	case VID_API_EVENT_CHUNK_DECODED:
		break;
	case VID_API_EVENT_FIFO_LOW: {
		u_int32 uStrBufIdx = 0; //use buffer 0 for the stream

		ctx->fifo_low = true;
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
		struct queue_data *queue = &ctx->q_data[V4L2_SRC];

		pStrBufDesc = get_str_buffer_desc(ctx);
		vpu_dbg(LVL_INFO, "%s AbrtDone StrBuf Curr, wptr(%x) rptr(%x) start(%x) end(%x)\n",
				__func__,
				pStrBufDesc->wptr,
				pStrBufDesc->rptr,
				pStrBufDesc->start,
				pStrBufDesc->end);

		down(&queue->drv_q_lock);
		vpu_ctx_clear_statistic_on_abort(ctx);

		vpu_dbg(LVL_BIT_FLOW,
			"ctx[%d] ABORT DONE, output qbuf(%ld/%ld),dqbuf(%ld)\n",
			ctx->str_index,
			queue->process_count,
			queue->qbuf_count,
			queue->dqbuf_count);
		vpu_dbg(LVL_BIT_FRAME_BYTES,
				"[%d]total bytes: %ld, %ld\n",
				ctx->str_index,
				ctx->total_qbuf_bytes,
				ctx->total_write_bytes);
		update_wptr(ctx, pStrBufDesc, pStrBufDesc->rptr);
		ctx->beginning = pStrBufDesc->rptr;
		ctx->total_qbuf_bytes = 0;
		ctx->total_write_bytes = 0;
		up(&queue->drv_q_lock);

		ctx->frm_dis_delay = 0;
		ctx->frm_dec_delay = 0;
		ctx->frm_total_num = 0;
		v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_RST_BUF, 0, NULL);
		}
		break;
	case VID_API_EVENT_RES_CHANGE: {
		struct queue_data *This;

		vpu_dbg(LVL_BIT_FLOW, "ctx[%d] RES CHANGE\n", ctx->str_index);
		This = &ctx->q_data[V4L2_DST];
		down(&This->drv_q_lock);
		reset_mbi_dcp_count(ctx);
		ctx->mbi_size = get_mbi_size(This);
		ctx->dcp_size = get_dcp_size(ctx);
		reset_frame_buffer(ctx);
		up(&This->drv_q_lock);
		vpu_dbg(LVL_BIT_FLOW,
			"warning: ctx[%d] RES_CHANGE event, seq id: %d\n",
			ctx->str_index, ctx->seqinfo.uActiveSeqTag);
		vpu_log_buffer_state(ctx);
		down(&This->drv_q_lock);
		This->enable = false;
		up(&This->drv_q_lock);
		ctx->wait_res_change_done = true;
		send_source_change_event(ctx);
		}
		break;
	case VID_API_EVENT_STR_BUF_RST: {
		pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
		struct queue_data *This;

		pStrBufDesc = get_str_buffer_desc(ctx);
		vpu_dbg(LVL_INFO, "%s wptr(%x) rptr(%x) start(%x) end(%x)\n",
				__func__,
				pStrBufDesc->wptr,
				pStrBufDesc->rptr,
				pStrBufDesc->start,
				pStrBufDesc->end
			  );
		This = &ctx->q_data[V4L2_DST];
		down(&This->drv_q_lock);
		check_queue_is_releasd(This, "EVENT_STR_BUF_RST is received");
		up(&This->drv_q_lock);
		if (check_is_need_reset_after_abort(ctx)) {
			vpu_dbg(LVL_BIT_FLOW,
				"Force reset ctx[%d]\n", ctx->str_index);
			v4l2_vpu_send_cmd(ctx, ctx->str_index,
					VID_API_CMD_STOP, 0, NULL);
		} else {
			ctx->wait_rst_done = false;
			complete(&ctx->completion);
		}
		vpu_dbg(LVL_BIT_FLOW, "ctx[%d] STR_BUF_RST\n", ctx->str_index);
		}
		break;
	case VID_API_EVENT_RET_PING:
		break;
	case VID_API_EVENT_STR_FMT_CHANGE:
		break;
	case VID_API_EVENT_FINISHED: {
		down(&ctx->q_data[V4L2_SRC].drv_q_lock);
		vpu_ctx_show_statstic(ctx, "eos");
		up(&ctx->q_data[V4L2_SRC].drv_q_lock);

		if (ctx->firmware_finished == true)
			vpu_err("warning: receive VID_API_EVENT_FINISHED when firmware_finished == true\n");
		ctx->firmware_finished = true;
		verify_decoded_frames(ctx);
		vpu_dbg(LVL_BIT_FLOW, "ctx[%d] FINISHED\n", ctx->str_index);
		vpu_dbg(LVL_INFO, "receive VID_API_EVENT_FINISHED and notfiy app eos\n");
		vpu_log_buffer_state(ctx);
		send_eos_event(ctx); //notfiy app stream eos reached
	}	break;
	case VID_API_EVENT_FIRMWARE_XCPT: {
		char *xcpt_info = (char*)event_data;

		vpu_err("ctx[%d] warning: VID_API_EVENT_FIRMWARE_XCPT,exception info: %s\n",
				ctx->str_index, xcpt_info);
		ctx->hang_status = true;
		vpu_dec_event_decode_error(ctx);
		}
		break;
	case VID_API_EVENT_DEC_CFG_INFO:
		break;
	case VID_API_EVENT_UNSUPPORTED_STREAM:
		vpu_dbg(LVL_WARN, "warning: HW unsupprot the format or stream\n");
		vpu_dec_event_decode_error(ctx);
		break;
	default:
		vpu_err("warning: uEvent %d is not handled\n", uEvent);
		break;
	}
	vpu_dbg(LVL_INFO, "leave %s, uEvent %d\n", __func__, uEvent);
}

static void release_vpu_ctx(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;

	remove_instance_file(ctx);
	vpu_dec_cleanup_cmd(ctx);
	release_queue_data(ctx);
	free_decoder_buffer(ctx);
	destroy_log_info_queue(ctx);
	cleanup_perf_queue(ctx);

	if (atomic64_read(&ctx->statistic.total_alloc_size) != 0)
		vpu_err("error: memory leak for vpu kalloc buffer\n");
	if (atomic64_read(&ctx->statistic.total_dma_size) != 0)
		vpu_err("error: memory leak for vpu dma buffer\n");

	mutex_destroy(&ctx->instance_mutex);
	mutex_destroy(&ctx->cmd_lock);
	mutex_destroy(&ctx->perf_lock);
	mutex_destroy(&ctx->fw_flow_mutex);
	clear_bit(ctx->str_index, &ctx->dev->instance_mask);
	ctx->dev->ctx[ctx->str_index] = NULL;
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

	if (dev->hang_mask == dev->instance_mask && dev->instance_mask != 0) {
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
	if (idx < 0 || idx >= VPU_MAX_NUM_STREAMS) {
		vpu_err("no free instance\n");
		return -EBUSY;
	}

	release_vpu_ctx(dev->ctx[idx]);

	return idx;
}

static void send_msg_queue(struct vpu_ctx *ctx, struct event_msg *msg)
{
	u_int32 ret;

	ret = kfifo_in(&ctx->msg_fifo, msg, sizeof(u_int32) * (MSG_WORD_LENGTH + msg->msgnum));
	if (ret != sizeof(u_int32) * (MSG_WORD_LENGTH + msg->msgnum))
		vpu_err("There is no memory for msg fifo, ret=%d\n", ret);
}

static bool receive_msg_queue(struct vpu_ctx *ctx, struct event_msg *msg)
{
	u_int32 ret;

	if (kfifo_len(&ctx->msg_fifo) >= sizeof(u_int32) * MSG_WORD_LENGTH) {
		ret = kfifo_out(&ctx->msg_fifo, msg, sizeof(u_int32) * MSG_WORD_LENGTH);
		if (ret != sizeof(u_int32) * MSG_WORD_LENGTH) {
			vpu_err("kfifo_out msg word has error, ret=%d\n", ret);
			return false;
		} else {
			if (msg->msgnum > 0) {
				if (kfifo_len(&ctx->msg_fifo) >= sizeof(u_int32) * msg->msgnum) {
					ret = kfifo_out(&ctx->msg_fifo, msg->msgdata, sizeof(u_int32) * msg->msgnum);
					if (ret != sizeof(u_int32) * msg->msgnum) {
						vpu_err("kfifo_out msg data has error, ret=%d\n", ret);
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

static void vpu_receive_msg_event(struct vpu_dev *dev)
{
	struct event_msg msg;
	struct shared_addr *This;
	struct vpu_ctx *ctx;

	This = &dev->shared_mem;
	if (!This)
		return;

	memset(&msg, 0, sizeof(struct event_msg));
	while (rpc_MediaIPFW_Video_message_check(This) == API_MSG_AVAILABLE) {
		rpc_receive_msg_buf(This, &msg);

		mutex_lock(&dev->dev_mutex);
		ctx = dev->ctx[msg.idx];
		if (ctx)
			count_event(&ctx->statistic, msg.msgid);
		if (ctx != NULL && !ctx->ctx_released) {
			send_msg_queue(ctx, &msg);
			queue_work(ctx->instance_wq, ctx->instance_work);
			queue_delayed_work(ctx->instance_wq,
					ctx->delayed_instance_work,
					msecs_to_jiffies(10));
		} else {
			vpu_err("msg [%d] %d is missed!%s\n",
				msg.idx, msg.msgid,
				ctx == NULL ? " ctx is NULL" : " ctx released");
		}
		mutex_unlock(&dev->dev_mutex);
	}
	if (rpc_MediaIPFW_Video_message_check(This) == API_MSG_BUFFER_ERROR)
		vpu_err("error: message size is too big to handle\n");
}

static void vpu_handle_msg_data(struct vpu_dev *dev, u32 data)
{
	if (data == 0xaa) {
		mutex_lock(&dev->cmd_mutex);
		vpu_mu_send_msg(dev, RPC_BUF_OFFSET,
				vpu_dec_cpu_phy_to_mu(dev, dev->m0_rpc_phy));
		vpu_mu_send_msg(dev, BOOT_ADDRESS, dev->m0_p_fw_space_phy);
		vpu_mu_send_msg(dev, INIT_DONE, 2);
		mutex_unlock(&dev->cmd_mutex);
	} else if (data == 0x55) {
		dev->firmware_started = true;
		complete(&dev->start_cmp);
	}  else if (data == 0xA5) {
		/*receive snapshot done msg and wakeup complete to suspend*/
		complete(&dev->snap_done_cmp);
	} else {
		vpu_receive_msg_event(dev);
	}
}

static void vpu_msg_run_work(struct work_struct *work)
{
	struct vpu_dev *dev = container_of(work, struct vpu_dev, msg_work);
	u32 data;

	while (vpu_mu_receive_msg(dev, &data) >= sizeof(u_int32))
		vpu_handle_msg_data(dev, data);
}

static void vpu_msg_run_delayed_work(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct vpu_dev *dev;

	dwork = to_delayed_work(work);
	dev = container_of(dwork, struct vpu_dev, delayed_msg_work);
	if (!kfifo_len(&dev->mu_msg_fifo))
		return;

	queue_work(dev->workqueue, &dev->msg_work);
}

static void vpu_msg_instance_work(struct work_struct *work)
{
	struct vpu_ctx_work *ctx_work;
	struct vpu_ctx *ctx;
	struct event_msg msg;

	ctx_work = container_of(work, struct vpu_ctx_work, instance_work);
	ctx = ctx_work->dev->ctx[ctx_work->str_index];
	if (!ctx || ctx->ctx_released)
		return;

	memset(&msg, 0, sizeof(struct event_msg));

	while (receive_msg_queue(ctx, &msg)) {
		vpu_api_event_handler(ctx, msg.idx, msg.msgid, msg.msgdata);
		memset(&msg, 0, sizeof(struct event_msg));
	}
}

static void vpu_msg_delayed_instance_work(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct vpu_ctx_work *ctx_work;
	struct vpu_ctx *ctx;

	dwork = to_delayed_work(work);
	ctx_work = container_of(dwork, struct vpu_ctx_work,
				delayed_instance_work);
	ctx = ctx_work->dev->ctx[ctx_work->str_index];
	if (!ctx || ctx->ctx_released || !kfifo_len(&ctx->msg_fifo))
		return;

	queue_work(ctx->instance_wq, ctx->instance_work);
}

static bool vpu_dec_alloc_buffer_item(struct vpu_ctx *ctx,
					u32 index, u32 count, u32 size,
					struct dma_buffer *buffers,
					const char *desc)
{
	struct dma_buffer buffer;
	int ret;

	if (index >= count)
		return false;

	if (buffers[index].dma_virt && buffers[index].dma_size >= size)
		return true;

	free_dma_buffer(ctx, &buffers[index]);

	vpu_dbg(LVL_BIT_FLOW, "alloc %s[%d], size = %d\n", desc, index, size);
	buffer.dma_size = size;

	ret = alloc_dma_buffer(ctx, &buffer);
	if (ret) {
		vpu_err("error: alloc %s buffer[%d] fail\n", desc, index);
		return false;
	}
	memcpy(&buffers[index], &buffer, sizeof(buffer));
	return true;
}

static void vpu_dec_alloc_mbi_dcp(struct vpu_ctx *ctx)
{
	struct queue_data *queue = &ctx->q_data[V4L2_DST];
	int ret;

	if (ctx->b_firstseq)
		return;

	while (ctx->mbi_count < ctx->req_mbi_count) {
		ret = vpu_dec_alloc_buffer_item(ctx,
						ctx->mbi_count,
						ctx->req_mbi_count,
						ctx->mbi_size,
						ctx->mbi_buffer,
						"mbi");
		if (!ret)
			break;
		ctx->mbi_count++;
	}
	while (ctx->dcp_count < ctx->req_dcp_count) {
		ret = vpu_dec_alloc_buffer_item(ctx,
						ctx->dcp_count,
						ctx->req_dcp_count,
						ctx->dcp_size,
						ctx->dcp_buffer,
						"dcp");
		if (!ret)
			break;
		ctx->dcp_count++;
	}
	respond_req_frame(ctx, queue, false);
}

static int vpu_queue_setup(struct vb2_queue *vq,
		unsigned int *buf_count,
		unsigned int *plane_count,
		unsigned int psize[],
		struct device *allocators[])
{
	struct queue_data  *This = (struct queue_data *)vq->drv_priv;
	struct vpu_ctx *ctx = NULL;

	vpu_dbg(LVL_BIT_FUNC, "%s() is called\n", __func__);

	ctx = container_of(This, struct vpu_ctx, q_data[This->type]);
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

	if (!V4L2_TYPE_IS_OUTPUT(vq->type))
		ctx->seek_flag = false;

	return 0;
}

static int vpu_buf_init(struct vb2_buffer *vb)
{
	struct vb2_queue    *vq = vb->vb2_queue;
	struct queue_data   *queue = (struct queue_data *)vq->drv_priv;
	struct vb2_data_req *p_data_req;

	p_data_req = &queue->vb2_reqs[vb->index];
	p_data_req->vb2_buf = vb;
	p_data_req->id = vb->index;
	set_data_req_status(p_data_req, FRAME_ALLOC);

	return 0;
}

static void vpu_buf_cleanup(struct vb2_buffer *vb)
{
	struct vb2_queue    *vq = vb->vb2_queue;
	struct queue_data   *queue = (struct queue_data *)vq->drv_priv;
	struct vb2_data_req *p_data_req;

	p_data_req = &queue->vb2_reqs[vb->index];
	p_data_req->vb2_buf = NULL;
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
	struct vpu_ctx *ctx = NULL;

	vpu_dbg(LVL_BIT_FUNC, "%s() is called\n", __func__);

	vpu_dbg(LVL_BIT_FUNC, "%s(), vq->type=%d, vb->index=%d\n",
			__func__, vq->type, vb->index);

	ctx = container_of(This, struct vpu_ctx, q_data[This->type]);
	data_req = &This->vb2_reqs[vb->index];
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

	add_buffer_to_queue(This, data_req);

	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		if (!vpu_dec_is_active(ctx))
			activate_vpu_dec(ctx);
		precheck_vb_data(ctx, vb);
		enqueue_stream_data(ctx, 0);
	} else {
		respond_req_frame(ctx, This, false);
	}

	This->qbuf_count++;
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
	.buf_init           = vpu_buf_init,
	.buf_cleanup        = vpu_buf_cleanup,
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
	vb2_q->ops = &v4l2_qops;
	vb2_q->drv_priv = This;
	if (V4L2_TYPE_IS_OUTPUT(type)) {
		vb2_q->mem_ops = &vb2_vmalloc_memops;
		vb2_q->gfp_flags = GFP_KERNEL;
	} else {
		vb2_q->mem_ops = &vb2_dma_contig_memops;
		vb2_q->gfp_flags = GFP_KERNEL | GFP_DMA32 | __GFP_NOWARN;
	}
	vb2_q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	vb2_q->dev = &ctx->dev->plat_dev->dev;
	ret = vb2_queue_init(vb2_q);
	if (ret)
		vpu_err("error: %s vb2_queue_init() failed (%d)!\n",
				__func__, ret);
	else
		This->vb2_q_inited = true;
}

static void init_queue_data(struct vpu_ctx *ctx)
{
	init_vb2_queue(&ctx->q_data[V4L2_SRC], V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, ctx);
	ctx->q_data[V4L2_SRC].type = V4L2_SRC;
	ctx->q_data[V4L2_SRC].ctx = ctx;
	sema_init(&ctx->q_data[V4L2_SRC].drv_q_lock, 1);
	init_vb2_queue(&ctx->q_data[V4L2_DST], V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, ctx);
	ctx->q_data[V4L2_DST].type = V4L2_DST;
	ctx->q_data[V4L2_DST].ctx = ctx;
	sema_init(&ctx->q_data[V4L2_DST].drv_q_lock, 1);
}

static void release_queue_data(struct vpu_ctx *ctx)
{
	vpu_dec_queue_release(&ctx->q_data[V4L2_SRC]);
	vpu_dec_queue_release(&ctx->q_data[V4L2_DST]);
}

static void enable_csr_reg(struct vpu_dev *This)
{
	writel(This->m0_p_fw_space_phy, This->csr_base + CSR_CM0Px_ADDR_OFFSET);
	writel(0x0, This->csr_base + CSR_CM0Px_CPUWAIT);
}

static void cleanup_firmware_memory(struct vpu_dev *vpudev)
{
	if (!vpudev->need_cleanup_firmware)
		return;
	memset_io(vpudev->m0_p_fw_space_vir, 0, vpudev->m0_boot_size);
	vpudev->need_cleanup_firmware = false;
}

static int vpu_firmware_download(struct vpu_dev *This)
{
	unsigned char *image;
	unsigned int FW_Size = 0;
	int ret = 0;
	char *p = This->m0_p_fw_space_vir;

	This->firmware_started = false;
	ret = request_firmware((const struct firmware **)&This->m0_pfw,
			M0FW_FILENAME,
			This->generic_dev
			);

	if (ret || !This->m0_pfw) {
		vpu_err("error: %s() request fw %s failed(%d)\n",
			__func__, M0FW_FILENAME, ret);

		if (This->m0_pfw) {
			release_firmware(This->m0_pfw);
			This->m0_pfw = NULL;
		}

		return ret;
	}

	vpu_dbg(LVL_INFO, "%s() request fw %s got size(%d)\n",
		__func__, M0FW_FILENAME, (int)This->m0_pfw->size);
	image = (uint8_t *)This->m0_pfw->data;
	FW_Size = This->m0_pfw->size;

	cleanup_firmware_memory(This);
	memcpy(This->m0_p_fw_space_vir, image, FW_Size);

	p[16] = This->plat_type;
	p[18] = 1;
	enable_csr_reg(This);
	This->need_cleanup_firmware = true;

	release_firmware(This->m0_pfw);
	This->m0_pfw = NULL;

	return ret;
}

static int dbglog_show(struct seq_file *s, void *data)
{
#define DBG_UNIT_SIZE		(7)
	struct vpu_dev *dev = s->private;
	u_int32 *pbuf;
	u_int32 line;
	int length;

	pbuf = (u_int32 *)dev->shared_mem.dbglog_mem_vir;
	line = (DBGLOG_SIZE) / (DBG_UNIT_SIZE * sizeof(u_int32));
	if (!line)
		return 0;

	if (!vpu_frmdbg_raw) {
		u_int32 i;

		length = 9 * DBG_UNIT_SIZE * line + 1;
		if (s->count + length >= s->size) {
			s->count = s->size;
			return 0;
		}
		for (i = 0; i < line; i++) {
			seq_printf(s, "%08x %08x %08x %08x %08x %08x %08x\n",
				pbuf[0], pbuf[1], pbuf[2], pbuf[3],
				pbuf[4], pbuf[5], pbuf[6]);
			pbuf += DBG_UNIT_SIZE;
		}

		return 0;
	}

	length = DBG_UNIT_SIZE * sizeof(u_int32) * line;
	if (s->count + length >= s->size) {
		s->count = s->size;
		return 0;
	}

	return seq_write(s, (void *)pbuf, length);
}

static int dbglog_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, dbglog_show, inode->i_private);
}

static struct file_operations dbglog_fops = {
	.owner = THIS_MODULE,
	.open = dbglog_open,
	.release = single_release,
	.read = seq_read,
};

static int create_dbglog_file(struct vpu_dev *dev)
{
	if (dev->debugfs_root == NULL) {
		dev->debugfs_root = debugfs_create_dir("vpu", NULL);
		if (!dev->debugfs_root) {
			vpu_err("error: create debugfs_root fail\n");
			return -EINVAL;
		}
	}

	if (dev->debugfs_dbglog)
		return 0;

	dev->debugfs_dbglog = debugfs_create_file("dbglog",
			VERIFY_OCTAL_PERMISSIONS(0444),
			dev->debugfs_root,
			dev,
			&dbglog_fops);
	if (!dev->debugfs_dbglog) {
		vpu_err("error: create debugfs_dbglog fail\n");
		return -EINVAL;
	}

	return 0;
}

static int fwlog_show(struct seq_file *s, void *data)
{
	struct vpu_dev *dev = s->private;
	int length;
	u32 rptr;
	u32 wptr;
	int ret = 0;

	if (!dev->print_buf)
		return 0;

	rptr = dev->print_buf->read;
	wptr = dev->print_buf->write;

	if (rptr == wptr)
		return 0;
	else if (rptr < wptr)
		length = wptr - rptr;
	else
		length = dev->print_buf->bytes + wptr - rptr;

	if (s->count + length >= s->size) {
		s->count = s->size;
		return 0;
	}

	if (rptr + length > dev->print_buf->bytes) {
		int num = dev->print_buf->bytes - rptr;

		if (seq_write(s, dev->print_buf->buffer + rptr, num))
			ret = -1;
		length -= num;
		rptr = 0;
	}
	if (seq_write(s, dev->print_buf->buffer + rptr, length))
		ret = -1;
	rptr += length;
	rptr %= dev->print_buf->bytes;
	if (!ret)
		dev->print_buf->read = rptr;

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

static int create_fwlog_file(struct vpu_dev *dev)
{
	if (dev->debugfs_root == NULL) {
		dev->debugfs_root = debugfs_create_dir("vpu", NULL);
		if (!dev->debugfs_root) {
			vpu_err("error: create debugfs_root fail\n");
			return -EINVAL;
		}
	}

	if (dev->debugfs_fwlog)
		return 0;
	dev->debugfs_fwlog = debugfs_create_file("vpu_malone_log",
						VERIFY_OCTAL_PERMISSIONS(0444),
						dev->debugfs_root,
						dev,
						&fwlog_fops);
	if (!dev->debugfs_fwlog) {
		vpu_err("error: create debugfs_fwlog fail\n");
		return -EINVAL;
	}

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
	for (i = VID_API_CMD_NULL; i < VID_API_CMD_TS + 1; i++) {
		size = scnprintf(buf + num, PAGE_SIZE - num,
				"\t%40s(%2d):%16ld\n",
				cmd2str[i], i, statistic->cmd[i]);
		num += size;
	}

	num += scnprintf(buf + num, PAGE_SIZE - num, "\t%40s    :%16ld\n",
			"UNKNOWN COMMAND", statistic->cmd[VID_API_CMD_TS + 1]);

	num += scnprintf(buf + num, PAGE_SIZE - num, "current command:\n");
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"%10s:%40s;%10lld.%06ld\n", "command",
			get_cmd_str(statistic->current_cmd),
			statistic->ts_cmd.tv_sec,
			statistic->ts_cmd.tv_nsec / 1000);

	if (ctx->pending)
		num += scnprintf(buf + num, PAGE_SIZE - num, "pending\n");

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
			"%10s:%40s;%10lld.%06ld\n", "event",
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
	pDEC_RPC_HOST_IFACE pSharedInterface;
	pBUFFER_INFO_TYPE buffer_info;

	ctx = container_of(attr, struct vpu_ctx, dev_attr_instance_buffer);
	statistic = &ctx->statistic;
	pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	buffer_info = &pSharedInterface->StreamBuffInfo[ctx->str_index];

	This = &ctx->q_data[V4L2_SRC];
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"output buffer status(%d):\n", This->vb2_q.num_buffers);
	for (i = 0; i < This->vb2_q.num_buffers; i++) {
		p_data_req = &This->vb2_reqs[i];
		if (!p_data_req->vb2_buf)
			continue;
		if (!p_data_req->queued)
			continue;
		num += scnprintf(buf + num, PAGE_SIZE - num,
					"\t%40s(%2d):queued\n",
					"buffer", i);
	}

	This = &ctx->q_data[V4L2_DST];
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"frame buffer status(%d):\n", This->vb2_q.num_buffers);
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
	if (ctx->stream_buffer.dma_size)
		stream_length = got_used_space(pStrBufDesc->wptr,
						pStrBufDesc->rptr,
						pStrBufDesc->start,
						pStrBufDesc->end);

	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d / %16d\n", "stream length",
			stream_length,
			ctx->stream_buffer.dma_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "decode delay frame",
			ctx->frm_dec_delay);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "display delay frame",
			ctx->frm_dis_delay);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "total frame number",
			ctx->frm_total_num);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%6lld,%09lld\n", "output timestamp(ns)",
			ctx->output_ts / NSEC_PER_SEC,
			ctx->output_ts % NSEC_PER_SEC);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%6lld,%09lld\n", "capture timestamp(ns)",
			ctx->capture_ts / NSEC_PER_SEC,
			ctx->capture_ts % NSEC_PER_SEC);

	This = &ctx->q_data[V4L2_SRC];
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16ld\n", "output qbuf count",
			This->qbuf_count);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16ld\n", "output dqbuf count",
			This->dqbuf_count);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16ld\n", "output write count",
			This->process_count);

	This = &ctx->q_data[V4L2_DST];
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16ld\n", "capture qbuf count",
			This->qbuf_count);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16ld\n", "capture dqbuf count",
			This->dqbuf_count);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16ld\n", "skipped frame count",
			ctx->statistic.skipped_frame_count);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16x\n", "beginning",
			ctx->beginning);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "request frame count",
			ctx->req_frame_count);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%12s%c%c%c%c\n",
			"output pixel format", "",
			ctx->q_data[V4L2_SRC].fourcc & 0xff,
			(ctx->q_data[V4L2_SRC].fourcc >> 8) & 0xff,
			(ctx->q_data[V4L2_SRC].fourcc >> 16) & 0xff,
			(ctx->q_data[V4L2_SRC].fourcc >> 24) & 0xff);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%12s%c%c%c%c\n",
			"capture pixel format", "",
			ctx->q_data[V4L2_DST].fourcc & 0xff,
			(ctx->q_data[V4L2_DST].fourcc >> 8) & 0xff,
			(ctx->q_data[V4L2_DST].fourcc >> 16) & 0xff,
			(ctx->q_data[V4L2_DST].fourcc >> 24) & 0xff);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%13d(%d)\n", "output status",
			ctx->q_data[V4L2_SRC].enable,
			vb2_is_streaming(&ctx->q_data[V4L2_SRC].vb2_q));
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%13d(%d)\n", "capture status",
			ctx->q_data[V4L2_DST].enable,
			vb2_is_streaming(&ctx->q_data[V4L2_DST].vb2_q));
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s: %dx%d(%dx%d), %d(DPB), %d(Ref), %d(DFE)\n",
			"seqinfo",
			ctx->seqinfo.uHorRes,
			ctx->seqinfo.uVerRes,
			ctx->seqinfo.uHorDecodeRes,
			ctx->seqinfo.uVerDecodeRes,
			ctx->seqinfo.uNumDPBFrms,
			ctx->seqinfo.uNumRefFrms,
			ctx->seqinfo.uNumDFEAreas & 0xff);

	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d/%16d/%16d(0x%x)\n", "mbi_count",
			ctx->mbi_index, ctx->mbi_count, ctx->req_mbi_count,
			ctx->mbi_size);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d/%16d/%16d(0x%x, %d, %d)\n", "dcp_count",
			ctx->dcp_index, ctx->dcp_count, ctx->req_dcp_count,
			ctx->dcp_size,
			ctx->seqinfo.uNumDFEAreas >> 16,
			DCP_FIXED_MB_ALLOC);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "stream_pic_input_count",
			buffer_info->stream_pic_input_count);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "stream_pic_parsed_count",
			buffer_info->stream_pic_parsed_count);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "stream_pic_end_flag",
			buffer_info->stream_pic_end_flag);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "stream_input_mode",
			buffer_info->stream_input_mode);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "stream_buffer_threshold",
			buffer_info->stream_buffer_threshold);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "start_code_bypass",
			ctx->start_code_bypass);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "ctx released",
			ctx->ctx_released);
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "msg kfifo length",
			kfifo_len(&ctx->msg_fifo));
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16d\n", "mu kfifo length",
			kfifo_len(&ctx->dev->mu_msg_fifo));
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%16lld,%16lld\n", "memory used",
			atomic64_read(&ctx->statistic.total_alloc_size),
			atomic64_read(&ctx->statistic.total_dma_size));
	num += scnprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%12d.%d.%d\n", "firmware version",
			(pSharedInterface->FWVersion & 0x00ff0000) >> 16,
			(pSharedInterface->FWVersion & 0x0000ff00) >> 8,
			pSharedInterface->FWVersion & 0x000000ff);

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

static ssize_t show_instance_perf_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_ctx *ctx;
	struct vpu_statistic *statistic;
	int num = 0;
	u_int64 start_time = 0;
	u_int64 prev_time = 0;
	u_int64 interv = 0;
	u_int64 total = 0;
	struct vpu_dec_perf_queue *perf;
	struct vpu_dec_perf_queue *tmp;
	int i = 0;

	ctx = container_of(attr, struct vpu_ctx, dev_attr_instance_perf);
	statistic = &ctx->statistic;

	if (!vpu_show_perf_ena)
		return num;
	if (!(vpu_show_perf_idx & (1<<ctx->str_index)))
		return num;

	num += scnprintf(buf + num, PAGE_SIZE - num, "beginning:\n");
	num += scnprintf(
		buf + num, PAGE_SIZE - num,
		"unit: ms \t\t time-point \t   interval    total\n");

	mutex_lock(&ctx->perf_lock);
	list_for_each_entry_safe(perf, tmp, &ctx->perf_q, list) {
		if (i == 0) {
			start_time = perf->time;
			prev_time = perf->time;
		}

		interv = perf->time - prev_time;
		total = perf->time - start_time;
		num += scnprintf(buf + num, PAGE_SIZE - num,
				 "%40s: %8ld %8ld\n", perf->str, interv, total);
		prev_time = perf->time;

		if (++i > 50) {
			num += scnprintf(buf + num, PAGE_SIZE - num,
				"Too many initialization steps, omitting the following\n");
			break;
		}
	}
	mutex_unlock(&ctx->perf_lock);

	num += scnprintf(buf + num, PAGE_SIZE - num, "decoded:\n");
	num += scnprintf(buf + num, PAGE_SIZE - num, "\t count: %8ld \t fps: %8ld\n",
			ctx->statistic.event[VID_API_EVENT_PIC_DECODED],
			ctx->perf_time.decoded_fps);
	num += scnprintf(buf + num, PAGE_SIZE - num, "ready:\n");
	num += scnprintf(buf + num, PAGE_SIZE - num, "\t count: %8ld \t fps: %8ld\n",
			ctx->statistic.event[VID_API_EVENT_FRAME_BUFF_RDY],
			ctx->perf_time.ready_fps);

	return num;
}

static ssize_t precheck_pattern_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_dev *vdev = dev_get_drvdata(dev);
	int num = 0;

	if (vdev->precheck_num)
		num = scnprintf(buf, PAGE_SIZE, "%s\n", vdev->precheck_content);

	return num;
}

static ssize_t precheck_pattern_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct vpu_dev *vdev = dev_get_drvdata(dev);
	long val;
	int num = 0;
	int bytes = 0;
	const char *delim = " ,;";
	char strbuf[1024];
	char *token;
	char *cur;

	strncpy(strbuf, buf, sizeof(strbuf) - 1);
	cur = strbuf;
	while ((token = strsep(&cur, delim))) {
		if (!strlen(token))
			continue;
		if (kstrtol((const char *)token, 0, &val))
			continue;
		vdev->precheck_pattern[num] = val;
		bytes += scnprintf(vdev->precheck_content + bytes,
					sizeof(vdev->precheck_content) - bytes,
					"%s0x%02x",
					num ? " " : "",
					vdev->precheck_pattern[num]);
		num++;
		if (num >= ARRAY_SIZE(vdev->precheck_pattern))
			break;
	}
	get_kmp_next(vdev->precheck_pattern, vdev->precheck_next, num);
	if (num >= 3)
		vdev->precheck_num = num;
	else
		vdev->precheck_num = 0;

	return count;
}
DEVICE_ATTR_RW(precheck_pattern);

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

static int create_instance_perf_file(struct vpu_ctx *ctx)
{
	scnprintf(ctx->perf_name, sizeof(ctx->perf_name) - 1,
			"instance%d_perf",
			ctx->str_index);
	sysfs_attr_init(&ctx->dev_attr_instance_perf.attr);
	ctx->dev_attr_instance_perf.attr.name = ctx->perf_name;
	ctx->dev_attr_instance_perf.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
	ctx->dev_attr_instance_perf.show = show_instance_perf_info;

	device_create_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_perf);

	return 0;
}

static int create_instance_file(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->dev || !ctx->dev->generic_dev)
		return -EINVAL;

	create_instance_command_file(ctx);
	create_instance_event_file(ctx);
	create_instance_buffer_file(ctx);
	create_instance_flow_file(ctx);
	create_instance_perf_file(ctx);


	return 0;
}

static int remove_instance_file(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->dev || !ctx->dev->generic_dev)
		return -EINVAL;

	device_remove_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_command);
	device_remove_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_event);
	device_remove_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_buffer);
	device_remove_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_flow);
	device_remove_file(ctx->dev->generic_dev, &ctx->dev_attr_instance_perf);

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
			vpu_err("error: alloc stream buffer fail!\n");
			return ret;
		}
	}

	if (!ctx->udata_buffer.dma_phy) {
		ctx->udata_buffer.dma_size = UDATA_BUFFER_SIZE;
		ret = alloc_dma_buffer(ctx, &ctx->udata_buffer);
		if (ret) {
			vpu_err("error: alloc udata buffer fail!\n");
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
			"/data/instance%d_crc.txt",
			ctx->str_index);
	ctx->crc_fp = filp_open(crc_file, O_RDWR | O_CREAT | O_TRUNC, 0644);
	if (IS_ERR(ctx->crc_fp)) {
		vpu_err("error: open crc file fail\n");
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
	bool status = true;

	if (!ctx)
		return false;
	if (ctx->firmware_stopped)
		return false;
	mutex_lock(&ctx->cmd_lock);
	if (ctx->pending && ctx->pending->request == VID_API_CMD_STOP)
		status = false;
	mutex_unlock(&ctx->cmd_lock);

	return status;
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
		vpu_err("failed to create decoder ctx\n");
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

	v4l2_fh_init(&ctx->fh, video_devdata(filp));
	filp->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	ctx->ctrl_inited = false;
	ctrls_setup_decoder(ctx);
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;

	ctx->instance_wq = alloc_workqueue("vpu_instance", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!ctx->instance_wq) {
		vpu_err("error: %s unable to alloc workqueue for ctx\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_wq;
	}
	ctx->instance_work = &dev->ctx_work[idx].instance_work;
	ctx->delayed_instance_work = &dev->ctx_work[idx].delayed_instance_work;

	mutex_init(&ctx->instance_mutex);
	mutex_init(&ctx->cmd_lock);
	mutex_init(&ctx->perf_lock);
	mutex_init(&ctx->fw_flow_mutex);
	atomic64_set(&ctx->statistic.total_dma_size, 0);
	atomic64_set(&ctx->statistic.total_alloc_size, 0);

	ctx->msg_buffer_size = sizeof(struct event_msg) * VID_API_MESSAGE_LIMIT;
	if (!is_power_of_2(ctx->msg_buffer_size))
		ctx->msg_buffer_size = roundup_pow_of_two(ctx->msg_buffer_size);
	ctx->msg_buffer = vzalloc(ctx->msg_buffer_size);
	if (!ctx->msg_buffer) {
		vpu_err("fail to alloc fifo when open\n");
		ret = -ENOMEM;
		goto err_alloc_fifo;
	}
	atomic64_add(ctx->msg_buffer_size, &ctx->statistic.total_alloc_size);
	if (kfifo_init(&ctx->msg_fifo, ctx->msg_buffer, ctx->msg_buffer_size)) {
		vpu_err("fail to init fifo when open\n");
		ret = -EINVAL;
		goto err_init_kfifo;
	}
	ctx->dev = dev;
	ctx->str_index = idx;
	dev->ctx[idx] = ctx;
	ctx->b_firstseq = true;
	ctx->wait_rst_done = false;
	ctx->wait_res_change_done = false;
	ctx->firmware_stopped = true;
	ctx->firmware_finished = false;
	ctx->frame_decoded = false;
	ctx->eos_stop_received = false;
	ctx->eos_stop_added = false;
	ctx->ctx_released = false;
	ctx->b_dis_reorder = false;
	ctx->start_code_bypass = false;
	ctx->stream_input_mode = FRAME_LVL;
	ctx->hang_status = false;
	ctx->first_dump_data_flag = true;
	ctx->colorspace = V4L2_COLORSPACE_REC709;
	ctx->xfer_func = V4L2_XFER_FUNC_709;
	ctx->ycbcr_enc = V4L2_YCBCR_ENC_709;
	ctx->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	INIT_LIST_HEAD(&ctx->cmd_q);
	INIT_LIST_HEAD(&ctx->perf_q);
	ctx->output_ts = TSM_TIMESTAMP_NONE;
	ctx->capture_ts = TSM_TIMESTAMP_NONE;
	create_instance_file(ctx);
	if (vpu_frmcrcdump_ena) {
		ret = open_crc_file(ctx);
		if (ret)
			goto err_open_crc;
	}
	ctx->seqinfo.uProgressive = 1;

	init_queue_data(ctx);
	init_log_info_queue(ctx);
	create_log_info_queue(ctx, vpu_log_depth);
	mutex_lock(&dev->dev_mutex);
	if (!dev->fw_is_ready) {
		ret = vpu_firmware_download(dev);
		if (ret) {
			vpu_err("error: vpu_firmware_download fail\n");
			mutex_unlock(&dev->dev_mutex);
			goto err_firmware_load;
		}
		vpu_dbg(LVL_INFO, "done: vpu_firmware_download\n");
		if (!ctx->dev->firmware_started) {
			reinit_completion(&ctx->dev->start_cmp);
			if (!wait_for_completion_timeout(&ctx->dev->start_cmp, msecs_to_jiffies(1000))) {
				vpu_err("error: don't get start interrupt\n");
				ret = -1;
				mutex_unlock(&dev->dev_mutex);
				goto err_firmware_load;
			}
		}
		dev->fw_is_ready = true;
	}
	create_fwlog_file(ctx->dev);
	create_dbglog_file(ctx->dev);
	mutex_unlock(&dev->dev_mutex);
	rpc_init_instance(&dev->shared_mem, ctx->str_index);
	rpc_set_stream_cfg_value(dev->shared_mem.pSharedInterface, ctx->str_index, vpu_dbe_num);
	init_vpu_buffer(ctx);

	vpu_dbg(LVL_BIT_FLOW, "<%d> ctx[%d] open\n",
			current->pid, ctx->str_index);
	vpu_calculate_performance(ctx, 0xff, "open device");

	return 0;

err_firmware_load:
	destroy_log_info_queue(ctx);
	cleanup_perf_queue(ctx);
	release_queue_data(ctx);

	if (vpu_frmcrcdump_ena)
		close_crc_file(ctx);
err_open_crc:
	remove_instance_file(ctx);
	dev->ctx[idx] = NULL;
err_init_kfifo:
	vfree(ctx->msg_buffer);
	atomic64_sub(ctx->msg_buffer_size, &ctx->statistic.total_alloc_size);
	ctx->msg_buffer = NULL;
	ctx->msg_buffer_size = 0;
err_alloc_fifo:
	mutex_destroy(&ctx->instance_mutex);
	mutex_destroy(&ctx->cmd_lock);
	mutex_destroy(&ctx->perf_lock);
	mutex_destroy(&ctx->fw_flow_mutex);
	destroy_workqueue(ctx->instance_wq);
err_alloc_wq:
	ctrls_delete_decoder(ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	clear_bit(ctx->str_index, &dev->instance_mask);
err_find_index:
	if (atomic64_read(&ctx->statistic.total_alloc_size) != 0)
		vpu_err("error: memory leak for vpu kalloc buffer\n");
	kfree(ctx);
	pm_runtime_put_sync(dev->generic_dev);

	return ret;
}

static void vpu_dec_disable(struct vpu_ctx *ctx, struct queue_data *queue)
{
	bool enable = false;

	down(&queue->drv_q_lock);
	if (queue->enable) {
		enable = true;
		queue->enable = false;
	}
	up(&queue->drv_q_lock);

	if (!enable)
		return;

	vpu_dbg(LVL_BIT_FLOW, "Pls stream off %s of ctx[%d] before release\n",
		V4L2_TYPE_IS_OUTPUT(queue->vb2_q.type) ? "Output" : "Capture",
		ctx->str_index);
	if (!V4L2_TYPE_IS_OUTPUT(queue->vb2_q.type)) {
		mutex_lock(&ctx->fw_flow_mutex);
		send_abort_cmd(ctx);
		mutex_unlock(&ctx->fw_flow_mutex);
		ctx->capture_ts = TSM_TIMESTAMP_NONE;
	} else {
		ctx->output_ts = TSM_TIMESTAMP_NONE;
	}

	vpu_dec_queue_disable(queue, queue->vb2_q.type);
}

static int v4l2_release(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_dev *dev = video_get_drvdata(vdev);
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);

	vpu_dbg(LVL_BIT_FLOW, "<%d> ctx[%d] close\n",
			current->pid, ctx->str_index);
	vpu_dbg(LVL_BIT_FUNC,
		"ctx[%d]: %s() - %s, %s, %s, total frame: %d\n",
		ctx->str_index,
		__func__,
		ctx->firmware_stopped ? "stopped" : "not stopped",
		ctx->firmware_finished ? "finished" : "not finished",
		ctx->eos_stop_added ? "eos_added" : "not eos_added",
		ctx->frm_total_num);

	vpu_dec_disable(ctx, &ctx->q_data[V4L2_SRC]);
	vpu_dec_disable(ctx, &ctx->q_data[V4L2_DST]);

	mutex_lock(&ctx->fw_flow_mutex);
	send_stop_cmd(ctx);
	mutex_unlock(&ctx->fw_flow_mutex);

	mutex_lock(&ctx->dev->dev_mutex);
	ctx->ctx_released = true;
	mutex_unlock(&ctx->dev->dev_mutex);

	cancel_delayed_work_sync(ctx->delayed_instance_work);
	cancel_work_sync(ctx->instance_work);
	vfree(ctx->msg_buffer);
	atomic64_sub(ctx->msg_buffer_size, &ctx->statistic.total_alloc_size);
	ctx->msg_buffer = NULL;
	ctx->msg_buffer_size = 0;
	if (ctx->instance_wq)
		destroy_workqueue(ctx->instance_wq);

	if (vpu_frmcrcdump_ena)
		close_crc_file(ctx);
	ctrls_delete_decoder(ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	mutex_lock(&dev->dev_mutex);
	if (!ctx->hang_status)
		release_vpu_ctx(ctx);
	else
		set_bit(ctx->str_index, &dev->hang_mask);
	mutex_unlock(&dev->dev_mutex);

	return 0;
}

static unsigned int v4l2_poll(struct file *filp, poll_table *wait)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);
	struct vb2_queue *src_q, *dst_q;
	unsigned int rc = 0;

	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);

	poll_wait(filp, &ctx->fh.wait, wait);

	if (v4l2_event_pending(&ctx->fh)) {
		vpu_dbg(LVL_INFO, "%s() v4l2_event_pending\n", __func__);
		rc |= POLLPRI;
	}

	src_q = &ctx->q_data[V4L2_SRC].vb2_q;
	dst_q = &ctx->q_data[V4L2_DST].vb2_q;

	if (ctx->firmware_finished && !src_q->streaming && !dst_q->streaming)
		return POLLERR;

	if ((ctx->firmware_finished || ctx->wait_res_change_done) &&
	     !list_empty(&dst_q->done_list))
		rc = 0;

	poll_wait(filp, &src_q->done_wq, wait);
	if (!list_empty(&src_q->done_list))
		rc |= POLLOUT | POLLWRNORM;
	poll_wait(filp, &dst_q->done_wq, wait);
	if (!list_empty(&dst_q->done_list))
		rc |= POLLIN | POLLRDNORM;

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

static int vpu_enable_hw(struct vpu_dev *This)
{
	vpu_dbg(LVL_BIT_FUNC, "%s()\n", __func__);
	vpu_setup(This);
	return 0;
}
static void vpu_disable_hw(struct vpu_dev *This)
{
}

static int swreset_vpu_firmware(struct vpu_dev *dev, u_int32 idx)
{
	int ret = 0;

	if (!dev)
		return 0;

	vpu_dbg(LVL_WARN, "SWRESET: swreset_vpu_firmware\n");
	dev->firmware_started = false;
	kfifo_reset(&dev->mu_msg_fifo);

	reinit_completion(&dev->start_cmp);
	vpu_dec_send_cmd(dev, 0, VID_API_CMD_FIRM_RESET, 0, NULL);
	if (!wait_for_completion_timeout(&dev->start_cmp, msecs_to_jiffies(10000))) {
		vpu_err("error: %s() fail\n", __func__);
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
		vpu_err("error: Cannot get core num %d\n", ret);
		return -EINVAL;
	}
	if (core_type == 2)
		dev->plat_type = IMX8QM;
	else
		dev->plat_type = IMX8QXP;
	reserved_node = of_parse_phandle(np, "boot-region", 0);
	if (!reserved_node) {
		vpu_err("error: boot-region of_parse_phandle error\n");
		return -ENODEV;
	}

	if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
		vpu_err("error: boot-region of_address_to_resource error\n");
		return -EINVAL;
	}
	dev->m0_p_fw_space_phy = reserved_res.start;
	dev->m0_boot_size = resource_size(&reserved_res);
	reserved_node = of_parse_phandle(np, "rpc-region", 0);
	if (!reserved_node) {
		vpu_err("error: rpc-region of_parse_phandle error\n");
		return -ENODEV;
	}

	if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
		vpu_err("error: rpc-region of_address_to_resource error\n");
		return -EINVAL;
	}
	dev->m0_rpc_phy = reserved_res.start;
	dev->m0_rpc_size = resource_size(&reserved_res);

	if (rpc_check_memory_region(dev->m0_p_fw_space_phy,
				dev->m0_rpc_phy,
				dev->m0_rpc_size) != VPU_RPC_MEMORY_UNCACHED) {
		vpu_err("rpc region<0x%x, 0x%x> isn't uncached for mu\n",
				dev->m0_rpc_phy, dev->m0_rpc_size);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "reg-csr", &csr_base);
	if (ret) {
		vpu_err("error: Cannot get csr offset %d\n", ret);
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
		vpu_err("video device alloc for decoder fail\n");
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
	dev->pvpu_decoder_dev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE |
					     V4L2_CAP_STREAMING;

	video_set_drvdata(dev->pvpu_decoder_dev, dev);

	ret = video_register_device(dev->pvpu_decoder_dev,
			VFL_TYPE_VIDEO,
			DECODER_NODE_NUMBER);
	if (ret) {
		vpu_err("error: %s unable to register video decoder device\n",
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
	mutex_init(&dev->dev_mutex);
	mutex_init(&dev->cmd_mutex);
	init_completion(&dev->start_cmp);
	init_completion(&dev->snap_done_cmp);
	dev->firmware_started = false;
	dev->need_cleanup_firmware = true;
	dev->hang_mask = 0;
	dev->instance_mask = 0;

	//firmware space for M0
	dev->m0_p_fw_space_vir = ioremap_wc(dev->m0_p_fw_space_phy,
			dev->m0_boot_size
			);
	if (!dev->m0_p_fw_space_vir) {
		vpu_err("error: failed to remap space for M0 firmware\n");
		return -ENOMEM;
	}

	dev->m0_rpc_virt = ioremap_wc(dev->m0_rpc_phy,
			dev->m0_rpc_size
			);
	if (!dev->m0_rpc_virt) {
		vpu_err("error: failed to remap space for rpc shared memory\n");
		return -ENOMEM;
	}

	return 0;
}

static void vpu_dec_init_rpc(struct vpu_dev *dev)
{
	cleanup_firmware_memory(dev);
	memset_io(dev->m0_rpc_virt, 0, dev->m0_rpc_size);
	rpc_init_shared_memory(&dev->shared_mem,
				vpu_dec_cpu_phy_to_mu(dev, dev->m0_rpc_phy),
				dev->m0_rpc_virt,
				dev->m0_rpc_size);
	dev->print_buf = dev->m0_rpc_virt + M0_PRINT_OFFSET;
	rpc_set_system_cfg_value(dev->shared_mem.pSharedInterface,
					VPU_REG_BASE);
	dev->fw_is_ready = false;
}

static void vpu_dec_restore_rpc(struct vpu_dev *dev)
{
	vpu_dbg(LVL_WARN, "restore vpu decoder\n");
	rpc_restore_shared_memory(&dev->shared_mem,
				vpu_dec_cpu_phy_to_mu(dev, dev->m0_rpc_phy),
				dev->m0_rpc_virt);
	dev->print_buf = dev->m0_rpc_virt + M0_PRINT_OFFSET;
	swreset_vpu_firmware(dev, 0);
	dev->fw_is_ready = true;
}

static void vpu_dec_init_ctx_work(struct vpu_dev *dev)
{
	int i;

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		struct vpu_ctx_work *ctx_work = &dev->ctx_work[i];

		ctx_work->str_index = i;
		ctx_work->dev = dev;
		INIT_WORK(&ctx_work->instance_work, vpu_msg_instance_work);
		INIT_DELAYED_WORK(&ctx_work->delayed_instance_work,
				vpu_msg_delayed_instance_work);
	}
}

static int vpu_probe(struct platform_device *pdev)
{
	struct vpu_dev *dev;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->generic_dev = get_device(&pdev->dev);
	dev->plat_dev = pdev;
	ret = vpu_attach_pm_domains(dev);
	if (ret)
		goto err_put_dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->regs_base)) {
		vpu_err("error: %s could not map regs_base\n", __func__);
		ret = PTR_ERR(dev->regs_base);
		goto err_det_pm;
	}

	ret = parse_dt_info(dev, np);
	if (ret) {
		vpu_err("error: %s parse device tree fail\n", __func__);
		goto err_dev_iounmap;
	}

	platform_set_drvdata(pdev, dev);

	ret = vpu_sc_check_fuse(dev, formats_compressed_dec, ARRAY_SIZE(formats_compressed_dec));
	if (ret)
		goto err_dev_iounmap;

	dev->mu_msg_buffer_size =
		sizeof(u_int32) * VPU_MAX_NUM_STREAMS * VID_API_MESSAGE_LIMIT;
	if (!is_power_of_2(dev->mu_msg_buffer_size))
		dev->mu_msg_buffer_size = roundup_pow_of_two(dev->mu_msg_buffer_size);
	dev->mu_msg_buffer = vzalloc(dev->mu_msg_buffer_size);
	if (!dev->mu_msg_buffer) {
		vpu_err("error: fail to alloc mu msg fifo\n");
		goto err_dev_iounmap;
	}
	ret = kfifo_init(&dev->mu_msg_fifo,
			dev->mu_msg_buffer, dev->mu_msg_buffer_size);
	if (ret) {
		vpu_err("error: fail to init mu msg fifo\n");
		goto err_free_fifo;
	}

	dev->workqueue = alloc_workqueue("vpu", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!dev->workqueue) {
		vpu_err("error: %s unable to alloc workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_free_fifo;
	}

	INIT_WORK(&dev->msg_work, vpu_msg_run_work);
	INIT_DELAYED_WORK(&dev->delayed_msg_work, vpu_msg_run_delayed_work);

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		vpu_err("failed to request mailbox, ret = %d\n", ret);
		pm_runtime_put_noidle(&pdev->dev);
		pm_runtime_set_suspended(&pdev->dev);
		goto err_pm_runtime_get_sync;
	}

	ret = init_vpudev_parameters(dev);
	if (ret) {
		vpu_err("error: failed to init parameters for vpudev\n");
		goto err_poweroff;
	}

	vpu_enable_hw(dev);
	if (is_vpu_poweroff(dev))
		vpu_dec_init_rpc(dev);
	else
		vpu_dec_restore_rpc(dev);

	pm_runtime_put_sync(&pdev->dev);
	device_create_file(&pdev->dev, &dev_attr_precheck_pattern);
	vpu_dec_init_ctx_work(dev);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		vpu_err("error: %s unable to register v4l2 dev\n", __func__);
		goto err_poweroff;
	}
	ret = create_vpu_video_device(dev);
	if (ret) {
		vpu_err("error: %s create vpu video device fail\n", __func__);
		goto err_unreg_v4l2;
	}

	return 0;

err_unreg_v4l2:
	v4l2_device_unregister(&dev->v4l2_dev);
err_poweroff:
	destroy_workqueue(dev->workqueue);
	vpu_disable_hw(dev);
	pm_runtime_put_sync(&pdev->dev);
err_pm_runtime_get_sync:
	pm_runtime_disable(&pdev->dev);
err_free_fifo:
	vfree(dev->mu_msg_buffer);
	dev->mu_msg_buffer = NULL;
	dev->mu_msg_buffer_size = 0;
err_dev_iounmap:
err_det_pm:
	vpu_detach_pm_domains(dev);
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

	device_remove_file(&pdev->dev, &dev_attr_precheck_pattern);
	debugfs_remove_recursive(dev->debugfs_root);
	dev->debugfs_root = NULL;
	dev->debugfs_dbglog = NULL;
	dev->debugfs_fwlog = NULL;
	vfree(dev->mu_msg_buffer);
	dev->mu_msg_buffer = NULL;
	dev->mu_msg_buffer_size = 0;
	vpu_dec_cancel_work(dev);
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

	vpu_detach_pm_domains(dev);
	if (dev->generic_dev) {
		put_device(dev->generic_dev);
		dev->generic_dev = NULL;
	}

	return 0;
}

static int vpu_runtime_suspend(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);

	if (vpudev->generic_dev)
		vpu_mu_free(vpudev);

	return 0;
}

static int vpu_runtime_resume(struct device *dev)
{
	int ret = 0;
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);

	if (vpudev->generic_dev)
		ret = vpu_mu_request(vpudev);

	return ret;
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

static void vpu_dec_cancel_work(struct vpu_dev *vpudev)
{
	int i;

	mutex_lock(&vpudev->dev_mutex);
	vpudev->suspend = true;
	mutex_unlock(&vpudev->dev_mutex);

	cancel_delayed_work_sync(&vpudev->delayed_msg_work);
	cancel_work_sync(&vpudev->msg_work);
	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		struct vpu_ctx_work *ctx_work = &vpudev->ctx_work[i];

		cancel_delayed_work_sync(&ctx_work->delayed_instance_work);
		cancel_work_sync(&ctx_work->instance_work);
	}
}

static void vpu_dec_resume_work(struct vpu_dev *vpudev)
{
	int i;

	mutex_lock(&vpudev->dev_mutex);
	vpudev->suspend = false;
	queue_work(vpudev->workqueue, &vpudev->msg_work);
	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		struct vpu_ctx *ctx = vpudev->ctx[i];

		if (!ctx || ctx->ctx_released)
			continue;
		queue_work(ctx->instance_wq, ctx->instance_work);
	}
	mutex_unlock(&vpudev->dev_mutex);
}

static int __maybe_unused vpu_suspend(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);
	int ret = 0;

	vpu_dbg(LVL_INFO, "suspend\n");

	if (vpudev->hang_mask != vpudev->instance_mask) {

		/*if there is an available device, send snapshot command to firmware*/
		reinit_completion(&vpudev->snap_done_cmp);
		v4l2_vpu_send_snapshot(vpudev);
		if (!wait_for_completion_timeout(&vpudev->snap_done_cmp, msecs_to_jiffies(1000))) {
			vpu_err("error: wait for vpu decoder snapdone event timeout!\n");
			ret = -1;
		}
	}

	vpu_dec_cancel_work(vpudev);
	vpu_dbg(LVL_INFO, "suspend done\n");

	return ret;
}

static bool is_vpu_poweroff(struct vpu_dev *vpudev)
{
	/* the csr register 'CM0Px_CPUWAIT' will be cleared to '1' after
	 * reset(poweoff then poweron)
	 */
	if (readl_relaxed(vpudev->csr_base + CSR_CM0Px_CPUWAIT) == 1)
		return true;
	else
		return false;
}

static int resume_vpu_register(struct vpu_dev *vpudev)
{
	if (!vpudev)
		return -EINVAL;

	vpu_enable_hw(vpudev);
	vpu_mu_enable_rx(vpudev);

	return 0;
}

static int resume_from_snapshot(struct vpu_dev *vpudev)
{
	int ret = 0;

	reinit_completion(&vpudev->start_cmp);
	enable_csr_reg(vpudev);
	/*wait for firmware resotre done*/
	if (!wait_for_completion_timeout(&vpudev->start_cmp, msecs_to_jiffies(1000))) {
		vpu_err("error: wait for vpu decoder resume done timeout!\n");
		ret = -1;
	}

	return ret;
}

static int resume_from_vpu_poweroff(struct vpu_dev *vpudev)
{
	int ret = 0;

	if (vpudev->hang_mask != vpudev->instance_mask)
		ret = resume_from_snapshot(vpudev);
	else
		vpudev->fw_is_ready = false;

	return ret;
}

static int __maybe_unused vpu_resume(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);
	int ret = 0;
	u_int32 idx;

	vpu_dbg(LVL_INFO, "resume\n");
	pm_runtime_get_sync(vpudev->generic_dev);

	resume_vpu_register(vpudev);

	if (vpudev->fw_is_ready == false)
		goto exit;

	if (is_vpu_poweroff(vpudev))
		ret = resume_from_vpu_poweroff(vpudev);
	else if (vpudev->hang_mask != vpudev->instance_mask) {
		idx = get_reset_index(vpudev);
		if (idx < VPU_MAX_NUM_STREAMS)
			swreset_vpu_firmware(vpudev, idx);
		else
			ret = -EINVAL;
	}

	vpu_dec_resume_work(vpudev);

exit:
	pm_runtime_put_sync(vpudev->generic_dev);
	vpu_dbg(LVL_INFO, "resume done\n");

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
};
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
module_param(vpu_frmdbg_raw, int, 0644);
MODULE_PARM_DESC(vpu_frmdbg_raw, "dump dbglog with raw data or not");
module_param(vpu_max_bufsize, int, 0644);
MODULE_PARM_DESC(vpu_max_bufsize, "maximun stream buffer size");
module_param(vpu_dbe_num, int, 0644);
MODULE_PARM_DESC(vpu_dbe_num, "vpu dbe number(1-2)");
module_param(vpu_frmcrcdump_ena, int, 0644);
MODULE_PARM_DESC(vpu_frmcrcdump_ena, "enable frame crc dump(0-1)");
module_param(stream_buffer_threshold, int, 0644);
MODULE_PARM_DESC(stream_buffer_threshold, "stream buffer threshold");
module_param(precheck_show_bytes, int, 0644);
MODULE_PARM_DESC(precheck_show_bytes, "show the beginning of content");
module_param(vpu_show_perf_ena, int, 0644);
MODULE_PARM_DESC(vpu_show_perf_ena, "enable show vpu decode performance(0-1)");
module_param(vpu_show_perf_idx, int, 0644);
MODULE_PARM_DESC(vpu_show_perf_idx, "show performance of which instance(bit N to mask instance N)");
module_param(vpu_show_perf_ent, int, 0644);
MODULE_PARM_DESC(vpu_show_perf_ent, "show performance of which event(1: decoded, 2: ready)");
module_param(vpu_datadump_ena, int, 0644);
MODULE_PARM_DESC(vpu_datadump_ena, "enable dump input frame data (0-1)");
