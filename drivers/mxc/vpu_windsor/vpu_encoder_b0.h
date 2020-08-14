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
 * @file vpu_encoder_b0.h
 *
 * @brief VPU ENCODER B0 definition
 *
 */
#ifndef __VPU_ENCODER_B0_H__
#define __VPU_ENCODER_B0_H__

#include <linux/irqreturn.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>
#ifdef CONFIG_IMX_SCU
#include <linux/firmware/imx/ipc.h>
#include <linux/firmware/imx/svc/misc.h>
#else
#include <soc/imx8/sc/svc/irq/api.h>
#include <soc/imx8/sc/ipc.h>
#include <soc/imx8/sc/sci.h>
#endif
#include <linux/mx8_mu.h>
#include <media/v4l2-event.h>
#include <linux/mailbox_client.h>
#include <linux/kfifo.h>

#include "vpu_encoder_rpc.h"
#include "vpu_encoder_config.h"

extern unsigned int vpu_dbg_level_encoder;

#define v4l2_fh_to_ctx(__fh) \
	container_of(__fh, struct vpu_ctx, fh)
#define v4l2_ctrl_to_ctx(__ctrl) \
	container_of((__ctrl)->handler, struct vpu_ctx, ctrl_handler)

#define VPU_ENC_MAX_CORE_NUM		2
#define VPU_MAX_BUFFER			32
#define M0FW_FILENAME			"vpu/vpu_fw_imx8_enc.bin"
#define MMAP_BUF_TYPE_SHIFT		28
#define MMAP_BUF_TYPE_MASK		0xF0000000
#define M0_BOOT_SIZE_DEFAULT		0x1000000
#define M0_BOOT_SIZE_MIN		0x100000
#define RPC_SIZE_DEFAULT		0x80000
#define RPC_SIZE_MIN			0x20000
#define PRINT_SIZE_DEFAULT		0x80000
#define PRINT_SIZE_MIN			0x20000
#define STREAM_SIZE			0x300000

#define MIN_BUFFER_COUNT		3
#define BITRATE_COEF			1024
#define BITRATE_LOW_THRESHOLD		(16)
#define BITRATE_HIGH_THRESHOLD		(240 * 1024)
#define BITRATE_DEFAULT_TARGET		(2 * 1024)
#define BITRATE_DEFAULT_PEAK		(8 * 1024)
#define GOP_H_THRESHOLD			300
#define GOP_L_THRESHOLD			1
#define GOP_DEFAULT			30
#define BFRAMES_H_THRESHOLD		4
#define BFRAMES_L_THRESHOLD		0
#define BFRAMES_DEFAULT			0
#define QP_MAX				51
#define QP_MIN				0
#define QP_DEFAULT			25
#define CPB_CTRL_UNIT			1024
#define CPB_COUNT			3

#define VPU_DISABLE_BITS		0x7
#define VPU_ENCODER_MASK		0x1

#define ENCODER_NODE_NUMBER 13 //use /dev/video13 as encoder node
struct vpu_v4l2_control {
	uint32_t id;
	enum v4l2_ctrl_type type;
	uint32_t minimum;
	uint32_t maximum;
	uint32_t step;
	uint32_t default_value;
	uint32_t menu_skip_mask;
	bool is_volatile;
};

typedef enum{
	INIT_DONE = 1,
	RPC_BUF_OFFSET,
	BOOT_ADDRESS,
	COMMAND,
	EVENT
} MSG_Type;

enum PLAT_TYPE {
	IMX8QXP = 0,
	IMX8QM  = 1,
	IMX8DM,
	IMX8DX,
	PLAT_TYPE_RESERVED
};

enum QUEUE_TYPE {
	V4L2_SRC = 0,
	V4L2_DST = 1,
};

enum vpu_video_standard {
	VPU_VIDEO_UNDEFINED = 0,
	VPU_VIDEO_AVC = 1,
	VPU_VIDEO_VC1 = 2,
	VPU_VIDEO_MPEG2 = 3,
	VPU_VIDEO_AVS = 4,
	VPU_VIDEO_ASP = 5,
	VPU_VIDEO_JPEG = 6,
	VPU_VIDEO_RV8 = 7,
	VPU_VIDEO_RV9 = 8,
	VPU_VIDEO_VP6 = 9,
	VPU_VIDEO_SPK = 10,
	VPU_VIDEO_VP8 = 11,
	VPU_VIDEO_AVC_MVC = 12,
	VPU_VIDEO_HEVC = 13,
	VPU_VIDEO_VP9 = 14,
};

#define VPU_PIX_FMT_AVS         v4l2_fourcc('A', 'V', 'S', '0')
#define VPU_PIX_FMT_ASP         v4l2_fourcc('A', 'S', 'P', '0')
#define VPU_PIX_FMT_RV8         v4l2_fourcc('R', 'V', '8', '0')
#define VPU_PIX_FMT_RV9         v4l2_fourcc('R', 'V', '9', '0')
#define VPU_PIX_FMT_VP6         v4l2_fourcc('V', 'P', '6', '0')
#define VPU_PIX_FMT_SPK         v4l2_fourcc('S', 'P', 'K', '0')
#define VPU_PIX_FMT_HEVC        v4l2_fourcc('H', 'E', 'V', 'C')
#define VPU_PIX_FMT_VP9         v4l2_fourcc('V', 'P', '9', '0')
#define VPU_PIX_FMT_LOGO        v4l2_fourcc('L', 'O', 'G', 'O')

#define VPU_PIX_FMT_TILED_8     v4l2_fourcc('Z', 'T', '0', '8')
#define VPU_PIX_FMT_TILED_10    v4l2_fourcc('Z', 'T', '1', '0')

enum vpu_pixel_format {
	VPU_HAS_COLOCATED = 0x00000001,
	VPU_HAS_SPLIT_FLD = 0x00000002,
	VPU_PF_MASK       = ~(VPU_HAS_COLOCATED | VPU_HAS_SPLIT_FLD),

	VPU_IS_TILED      = 0x000000100,
	VPU_HAS_10BPP     = 0x00000200,

	VPU_IS_PLANAR     = 0x00001000,
	VPU_IS_SEMIPLANAR = 0x00002000,
	VPU_IS_PACKED     = 0x00004000,

	// Merged definitions using above flags:
	VPU_PF_UNDEFINED  = 0,
	VPU_PF_YUV420_SEMIPLANAR = 0x00010000 | VPU_IS_SEMIPLANAR,
	VPU_PF_YUV420_PLANAR = 0x00020000 | VPU_IS_PLANAR,
	VPU_PF_UYVY = 0x00040000 | VPU_IS_PACKED,
	VPU_PF_TILED_8BPP = 0x00080000 | VPU_IS_TILED | VPU_IS_SEMIPLANAR,
	VPU_PF_TILED_10BPP = 0x00100000 | VPU_IS_TILED | VPU_IS_SEMIPLANAR | VPU_HAS_10BPP,
};

struct vpu_ctx;
struct core_device;
struct vpu_dev;
struct vpu_v4l2_fmt {
	char *name;
	unsigned int fourcc;
	unsigned int num_planes;
	unsigned int venc_std;
	unsigned int is_yuv;
};

struct vb2_data_req {
	struct list_head  list;
	struct vb2_buffer *vb2_buf;
	u_int32 sequence;
	u_int32 buffer_flags;
};

enum ENC_RW_FLAG {
	VPU_ENC_FLAG_WRITEABLE,
	VPU_ENC_FLAG_READABLE
};

struct queue_data {
	unsigned int width;
	unsigned int height;
	unsigned int sizeimage[VB2_MAX_PLANES];
	struct v4l2_rect rect;
	int buf_type; // v4l2_buf_type
	bool vb2_q_inited;
	struct vb2_queue vb2_q;    // vb2 queue
	struct list_head drv_q;    // driver queue
	struct semaphore drv_q_lock;
	struct vb2_data_req vb2_reqs[VPU_MAX_BUFFER];
	enum QUEUE_TYPE type;
	struct vpu_v4l2_fmt *supported_fmts;
	unsigned int fmt_count;
	struct vpu_v4l2_fmt *current_fmt;
	unsigned long rw_flag;
	struct list_head frame_q;
	atomic64_t frame_count;
	struct list_head frame_idle;
	struct vpu_ctx *ctx;
	char desc[64];
};

struct vpu_strip_info {
	unsigned long count;
	unsigned long max;
	unsigned long total;
};

struct vpu_fps_sts {
	unsigned int thd;
	unsigned int times;
	unsigned long frame_number;
	struct timespec64 ts;
	unsigned long fps;
};

struct vpu_statistic {
	unsigned long cmd[GTB_ENC_CMD_RESERVED + 1];
	unsigned long event[VID_API_ENC_EVENT_RESERVED + 1];
	unsigned long current_cmd;
	unsigned long current_event;
	struct timespec64 ts_cmd;
	struct timespec64 ts_event;
	unsigned long yuv_count;
	unsigned long encoded_count;
	unsigned long h264_count;
	struct {
		struct vpu_strip_info fw;
		struct vpu_strip_info begin;
		struct vpu_strip_info eos;
	} strip_sts;
	bool fps_sts_enable;
	struct vpu_fps_sts fps[VPU_FPS_STS_CNT];
	unsigned long timestamp_overwrite;
};

struct vpu_attr {
	struct device_attribute dev_attr;
	char name[64];
	u32 index;
	struct core_device *core;

	pid_t pid;
	pid_t tgid;

	struct vpu_statistic statistic;
	MEDIAIP_ENC_PARAM param;
	struct v4l2_fract fival;
	u32 h264_vui_sar_enable;
	u32 h264_vui_sar_idc;
	u32 h264_vui_sar_width;
	u32 h264_vui_sar_height;

	unsigned long ts_start[2];
	unsigned long msg_count;
	atomic64_t total_dma_size;

	bool created;
};

struct print_buf_desc {
	u32 start_h_phy;
	u32 start_h_vir;
	u32 start_m;
	u32 bytes;
	u32 read;
	u32 write;
	char buffer[0];
};

#ifdef CONFIG_IMX_SCU
struct vpu_sc_msg_misc {
	struct imx_sc_rpc_msg hdr;
	u32 word;
} __packed;
#endif

struct vpu_sc_chan {
	struct core_device *dev;
	char name[20];
	struct mbox_client cl;
	struct mbox_chan *ch;
};

struct core_device {
	void *m0_p_fw_space_vir;
	u_int32 m0_p_fw_space_phy;
	u32 fw_buf_size;
	u32 fw_actual_size;
	void *m0_rpc_virt;
	u_int32 m0_rpc_phy;
	u32 rpc_buf_size;
	u32 print_buf_size;
	u32 rpc_actual_size;
	struct print_buf_desc *print_buf;

	struct mutex cmd_mutex;
	bool fw_is_ready;
	bool firmware_started;
	struct completion boot_cmp;
	struct completion snap_done_cmp;
	struct workqueue_struct *workqueue;
	struct work_struct msg_work;
	void __iomem *mu_base_virtaddr;
	unsigned int vpu_mu_id;
	int vpu_mu_init;

	u32 supported_instance_count;
	struct vpu_ctx *ctx[VID_API_NUM_STREAMS];
	struct vpu_attr attr[VID_API_NUM_STREAMS];
	struct shared_addr shared_mem;
	u32 id;
	u32 reg_base;
	u32 reg_size;
	u32 reg_csr_base;
	u32 reg_csr_size;
	int irq;
	struct device *generic_dev;
	struct vpu_dev *vdev;
	bool snapshot;
	bool suspend;
	bool hang;
	struct device_attribute core_attr;
	char name[64];
	unsigned long reset_times;

	struct kfifo mu_msg_fifo;

	/* reserve for kernel version 5.4 or later */
	struct vpu_sc_chan sc_chan_tx0;
	struct vpu_sc_chan sc_chan_tx1;
	struct vpu_sc_chan sc_chan_rx;

	struct dentry *debugfs_fwlog;
};

struct vpu_enc_mem_item {
	struct list_head list;
	void *virt_addr;
	unsigned long phy_addr;
	unsigned long size;
	unsigned long offset;
};

struct vpu_enc_mem_info {
	void *virt_addr;
	unsigned long phy_addr;
	unsigned long size;
	unsigned long bytesused;
	struct list_head memorys;
	struct mutex lock;
};

struct vpu_dev {
	struct device *generic_dev;
	struct v4l2_device v4l2_dev;
	struct video_device *pvpu_encoder_dev;
	struct platform_device *plat_dev;
	struct clk *clk_m0;
	u32 reg_vpu_base;
	u32 reg_vpu_size;
	u32 reg_rpc_system;
	void __iomem *regs_base;
	struct mutex dev_mutex;
	struct core_device core_dev[VPU_ENC_MAX_CORE_NUM];
	u_int32 plat_type;
	u_int32 core_num;
	bool hw_enable;

	struct delayed_work watchdog;
	u8 heartbeat;

	struct {
		u32 min_width;
		u32 max_width;
		u32 step_width;
		u32 min_height;
		u32 max_height;
		u32 step_height;
	} supported_size;
	struct {
		u32 min;
		u32 max;
		u32 step;
	} supported_fps;
	struct vpu_enc_mem_info reserved_mem;

	/* reserve for kernel version 5.4 or later */
	struct device *pd_vpu;
	struct device *pd_enc1;
	struct device *pd_enc2;
	struct device *pd_mu1;
	struct device *pd_mu2;
	struct device_link *pd_vpu_link;
	struct device_link *pd_enc1_link;
	struct device_link *pd_enc2_link;
	struct device_link *pd_mu1_link;
	struct device_link *pd_mu2_link;
	struct dentry *debugfs_root;
};

struct buffer_addr {
	void *virt_addr;
	dma_addr_t phy_addr;
	u_int32 size;
};

enum {
	VPU_ENC_STATUS_INITIALIZED,
	VPU_ENC_STATUS_OUTPUT_READY = 18,
	VPU_ENC_STATUS_DATA_READY = 19,
	VPU_ENC_STATUS_SNAPSHOT = 20,
	VPU_ENC_STATUS_FORCE_RELEASE = 21,
	VPU_ENC_STATUS_EOS_SEND = 22,
	VPU_ENC_STATUS_START_SEND = 23,
	VPU_ENC_STATUS_START_DONE = 24,
	VPU_ENC_STATUS_STOP_REQ = 25,
	VPU_ENC_STATUS_STOP_SEND = 26,
	VPU_ENC_STATUS_STOP_DONE = 27,
	VPU_ENC_STATUS_CLOSED = 28,
	VPU_ENC_STATUS_CONFIGURED = 29,
	VPU_ENC_STATUS_HANG = 30,
	VPU_ENC_STATUS_KEY_FRAME = 31
};

struct vpu_ctx {
	struct vpu_dev *dev;
	struct v4l2_fh fh;

	struct v4l2_ctrl_handler ctrl_handler;
	bool ctrl_inited;

	int str_index;
	unsigned long status;
	struct queue_data q_data[2];
	struct mutex instance_mutex;
	struct work_struct instance_work;
	struct workqueue_struct *instance_wq;
	bool ctx_released;
	struct buffer_addr encoder_stream;
	struct buffer_addr encFrame[MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES];
	struct buffer_addr refFrame[MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES];
	struct buffer_addr actFrame;
	struct buffer_addr enc_buffer;
	MEDIAIP_ENC_MEM_REQ_DATA mem_req;
	struct core_device *core_dev;

	struct completion start_cmp;
	struct completion stop_cmp;
	bool power_status;

	struct list_head msg_q;
	struct list_head idle_q;

	struct vpu_statistic sts;
	unsigned int frozen_count;
	u_int32 sequence;
	s64 timestams[VPU_ENC_SEQ_CAPACITY];
	u32 cpb_size;
	s64 timestamp;
	u8 colorspace;
	u8 xfer_func;
	u8 ycbcr_enc;
	u8 quantization;
};

#define LVL_ERR		(1 << 0)
#define LVL_WARN	(1 << 1)
#define LVL_ALL		(1 << 2)
#define LVL_IRQ		(1 << 3)
#define LVL_INFO	(1 << 4)
#define LVL_CMD		(1 << 5)
#define LVL_EVT		(1 << 6)
#define LVL_DEBUG	(1 << 7)
#define LVL_CTRL	(1 << 8)
#define LVL_RPC		(1 << 9)
#define LVL_MSG		(1 << 10)
#define LVL_MEM		(1 << 11)
#define LVL_BUF		(1 << 12)
#define LVL_FLOW	(1 << 13)
#define LVL_FRAME	(1 << 14)
#define LVL_FUNC	(1 << 16)

#ifndef TAG
#define TAG	"[VPU Encoder] "
#endif

#define vpu_dbg(level, fmt, arg...) \
	do { \
		if ((vpu_dbg_level_encoder & (level)) || ((level) & LVL_ERR)) \
			pr_info(TAG""fmt, ## arg); \
	} while (0)

#define vpu_err(fmt, arg...)	vpu_dbg(LVL_ERR, fmt, ##arg)
#define vpu_log_func()		vpu_dbg(LVL_FUNC, "%s()\n", __func__)

u32 cpu_phy_to_mu(struct core_device *dev, u32 addr);
struct vpu_attr *get_vpu_ctx_attr(struct vpu_ctx *ctx);
struct vpu_ctx *get_vpu_attr_ctx(struct vpu_attr *attr);

#ifndef VPU_SAFE_RELEASE
#define VPU_SAFE_RELEASE(p, func)	\
	do {\
		if (p) {\
			func(p);\
			p = NULL;\
		} \
	} while (0)
#endif

#endif
