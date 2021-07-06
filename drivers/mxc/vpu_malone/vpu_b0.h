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
 * @file vpu_b0.h
 *
 * @brief VPU B0 definition
 *
 */
#ifndef __VPU_B0_H
#define __VPU_B0_H

#include <linux/version.h>
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
#include <linux/mailbox_client.h>
#include <media/v4l2-event.h>
#include <linux/kfifo.h>
#include "vpu_rpc.h"

extern unsigned int vpu_dbg_level_decoder;

#define v4l2_fh_to_ctx(__fh) \
	container_of(__fh, struct vpu_ctx, fh)
#define v4l2_ctrl_to_ctx(__ctrl) \
	container_of((__ctrl)->handler, struct vpu_ctx, ctrl_handler)

#define MIN_SPACE (SCODE_SIZE + 64)
#define SCODE_SIZE (4096)

#define VPU_MAX_BUFFER 32
#define M0FW_FILENAME "vpu/vpu_fw_imx8_dec.bin"
#define MMAP_BUF_TYPE_SHIFT 28
#define MMAP_BUF_TYPE_MASK 0xF0000000
#define DCP_SIZE 0x3000000
#define DCP_SIZE_MINIMUM	0x100000
#define MAX_BUFFER_SIZE 0xc00000
#define UDATA_BUFFER_SIZE 0x1000
#define MAX_DCP_NUM 2
#define MAX_MBI_NUM 18 // same with MEDIA_PLAYER_MAX_MBI_UNIT defined in firmware
#define MAX_TIMEOUT_COUNT 10
#define VPU_REG_BASE 0x40000000
#define VPU_MAX_STEP_STRING_LENGTH 40
#define VPU_DISABLE_BITS (0x7)

#define V4L2_PIX_FMT_NV12_10BIT    v4l2_fourcc('N', 'T', '1', '2') /*  Y/CbCr 4:2:0 for 10bit  */
#define	VPU_FRAME_DEPTH_MAX     512
#define VPU_FRAME_DEPTH_DEFAULT 256
#define DECODER_NODE_NUMBER 12 // use /dev/video12 as vpu decoder
#define DEFAULT_LOG_DEPTH 20
#define DEFAULT_FRMDBG_ENABLE 0
#define DEFAULT_FRMDBG_LEVEL 0
#define VPU_DEC_CMD_DATA_MAX_NUM	16
#define VPU_DEC_MAX_WIDTH		8188
#define VPU_DEC_MAX_HEIGTH		8188
#define VPU_DEC_FMT_DIVX_MASK		(1 << 20)
#define VPU_DEC_FMT_RV_MASK		(1 << 21)

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
	VPU_VIDEO_RV = 7,
	VPU_VIDEO_VP6 = 8,
	VPU_VIDEO_SPK = 9,
	VPU_VIDEO_VP8 = 10,
	VPU_VIDEO_AVC_MVC = 11,
	VPU_VIDEO_HEVC = 12,
};

typedef enum{
	EOS_PADDING_TYPE = 1,
	BUFFLUSH_PADDING_TYPE = 2,
	BUFABORT_PADDING_TYPE = 3,
} VPU_PADDING_SCODE_TYPE;

#define VPU_PIX_FMT_AVS         v4l2_fourcc('A', 'V', 'S', '0')
#define VPU_PIX_FMT_ASP         v4l2_fourcc('A', 'S', 'P', '0')
#define VPU_PIX_FMT_RV          v4l2_fourcc('R', 'V', '0', '0')
#define VPU_PIX_FMT_VP6         v4l2_fourcc('V', 'P', '6', '0')
#define VPU_PIX_FMT_SPK         v4l2_fourcc('S', 'P', 'K', '0')
#define VPU_PIX_FMT_DIV3        v4l2_fourcc('D', 'I', 'V', '3')
#define VPU_PIX_FMT_DIVX        v4l2_fourcc('D', 'I', 'V', 'X')
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

enum ARV_FRAME_TYPE {
	ARV_8 = 0,
	ARV_9,
	ARV_10,
};

struct VPU_FMT_INFO_ARV {
	u_int32				data_len;
	u_int32				slice_num;
	u_int32				*slice_offset;
	u_int32				packlen;
	enum ARV_FRAME_TYPE	type;
};

struct vpu_v4l2_fmt {
	char *name;
	unsigned int fourcc;
	unsigned int num_planes;
	unsigned int vdec_std;
	unsigned int disable;
};

struct vb2_data_req {
	struct list_head  list;
	struct vb2_buffer *vb2_buf;
	int id;
	u_int32 status;
	bool bfield;
	bool queued;
	u_int32 phy_addr[2]; //0 for luma, 1 for chroma
	u_int32 data_offset[2]; //0 for luma, 1 for chroma
	u32 seq_tag;
};

struct queue_data {
	unsigned int width;
	unsigned int height;
	unsigned int stride;
	unsigned int field;
	unsigned int num_planes;
	unsigned int sizeimage[2];
	unsigned int fourcc;
	unsigned int vdec_std;
	int buf_type; // v4l2_buf_type
	bool vb2_q_inited;
	struct vb2_queue vb2_q;    // vb2 queue
	struct list_head drv_q;    // driver queue
	struct semaphore drv_q_lock;
	struct vb2_data_req vb2_reqs[VPU_MAX_BUFFER];
	enum QUEUE_TYPE type;
	unsigned long qbuf_count;
	unsigned long dqbuf_count;
	unsigned long process_count;
	bool enable;
	struct vpu_ctx *ctx;
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
struct vpu_imx_sc_msg_misc {
	struct imx_sc_rpc_msg hdr;
	u32 word;
} __packed;
#endif

struct vpu_sc_chan {
	struct vpu_dev *dev;
	char name[20];
	struct mbox_client cl;
	struct mbox_chan *ch;
};

struct vpu_ctx_work {
	struct work_struct instance_work;
	struct delayed_work delayed_instance_work;
	int str_index;
	struct vpu_dev *dev;
};

struct vpu_ctx;
struct vpu_dev {
	struct device *generic_dev;
	struct v4l2_device v4l2_dev;
	struct video_device *pvpu_decoder_dev;
	struct platform_device *plat_dev;
	struct firmware *m0_pfw;
	void *m0_p_fw_space_vir;
	u_int32 m0_p_fw_space_phy;
	u_int32 m0_boot_size;
	void *m0_rpc_virt;
	u_int32 m0_rpc_phy;
	u_int32 m0_rpc_size;
	struct mutex dev_mutex;
	struct mutex cmd_mutex;
	bool fw_is_ready;
	bool firmware_started;
	bool need_cleanup_firmware;
	bool suspend;
	struct completion start_cmp;
	struct completion snap_done_cmp;
	struct workqueue_struct *workqueue;
	struct work_struct msg_work;
	struct delayed_work delayed_msg_work;
	unsigned long instance_mask;
	unsigned long hang_mask; //this is used to deal with hang issue to reset firmware
	struct clk *vpu_clk;
	void __iomem *mu_base_virtaddr;
	unsigned int vpu_mu_id;
	int vpu_mu_init;
	u_int32 plat_type;

	struct clk *clk_m0;
	void __iomem *regs_base;
	void __iomem *csr_base;
	u_int32 cm_offset;

	struct shared_addr shared_mem;
	struct vpu_ctx *ctx[VPU_MAX_NUM_STREAMS];
	struct vpu_ctx_work ctx_work[VPU_MAX_NUM_STREAMS];
	struct dentry *debugfs_root;
	struct dentry *debugfs_dbglog;
	struct dentry *debugfs_fwlog;

	struct print_buf_desc *print_buf;
	u8 precheck_pattern[64];
	int precheck_next[64];
	int precheck_num;
	char precheck_content[1024];

	struct kfifo mu_msg_fifo;
	void *mu_msg_buffer;
	unsigned int mu_msg_buffer_size;
	u_int32 vpu_irq;

	/* reserve for kernel version 5.4 or later */
	struct vpu_sc_chan sc_chan_tx0;
	struct vpu_sc_chan sc_chan_tx1;
	struct vpu_sc_chan sc_chan_rx;
	struct device *pd_vpu;
	struct device *pd_dec;
	struct device *pd_mu;
	struct device_link *pd_vpu_link;
	struct device_link *pd_dec_link;
	struct device_link *pd_mu_link;
};

struct vpu_statistic {
	unsigned long cmd[VID_API_CMD_TS + 2];
	unsigned long event[VID_API_EVENT_DEC_CFG_INFO + 2];
	unsigned long current_cmd;
	unsigned long current_event;
	unsigned long skipped_frame_count;
	struct timespec64 ts_cmd;
	struct timespec64 ts_event;
	atomic64_t total_dma_size;
	atomic64_t total_alloc_size;
	unsigned long frame_input;
	unsigned long frame_hdr;
	unsigned long frame_decoded;
	unsigned long frame_ready;
	unsigned long frame_display;
	unsigned long eos_cnt;
	unsigned long eos_frames;
	unsigned long eos_bytes;
};

struct dma_buffer {
	dma_addr_t dma_phy;
	void *dma_virt;
	u_int32 dma_size;
};

struct vpu_dec_cmd_request {
	struct list_head list;
	u32 request;
	u32 response;
	bool block;
	u32 idx;
	u32 num;
	u32 data[VPU_DEC_CMD_DATA_MAX_NUM];
};

struct vpu_dec_perf_time {
	u_int64 open_time;

	u_int64 first_decoded_time;
	u_int64 last_decoded_time;
	u_int64 cur_decoded_interv;
	u_int64 decoded_fps;

	u_int64 first_ready_time;
	u_int64 last_ready_time;
	u_int64 cur_ready_interv;
	u_int64 ready_fps;
};

struct vpu_dec_perf_queue {
	struct list_head list;
	char str[VPU_MAX_STEP_STRING_LENGTH];
	u_int64 time;
};

struct vpu_ctx {
	struct vpu_dev *dev;
	struct v4l2_fh fh;

	struct vpu_statistic statistic;
	struct device_attribute dev_attr_instance_command;
	char command_name[64];
	struct device_attribute dev_attr_instance_event;
	char event_name[64];
	struct device_attribute dev_attr_instance_buffer;
	char buffer_name[64];
	struct device_attribute dev_attr_instance_flow;
	char flow_name[64];
	struct device_attribute dev_attr_instance_perf;
	char perf_name[64];
	struct v4l2_ctrl_handler ctrl_handler;
	bool ctrl_inited;
	struct list_head log_q;

	int str_index;
	struct queue_data q_data[2];
	struct kfifo msg_fifo;
	void *msg_buffer;
	unsigned int msg_buffer_size;
	struct mutex instance_mutex;
	struct work_struct *instance_work;
	struct delayed_work *delayed_instance_work;
	struct workqueue_struct *instance_wq;
	struct completion completion;
	struct completion stop_cmp;
	struct completion eos_cmp;
	MediaIPFW_Video_SeqInfo seqinfo;
	bool b_dis_reorder;
	bool b_firstseq;
	bool wait_rst_done;
	bool wait_res_change_done;
	bool seek_flag;
	bool firmware_stopped;
	bool firmware_finished;
	bool eos_stop_received;
	bool eos_stop_added;
	bool ctx_released;
	bool start_code_bypass;
	STREAM_INPUT_MODE stream_input_mode;
	bool hang_status;
	bool fifo_low;
	bool frame_decoded;
	bool first_dump_data_flag;
	bool first_data_flag;
	u32 req_frame_count;
	u32 req_mbi_count;
	u32 req_dcp_count;
	u32 mbi_count;
	u32 mbi_size;
	u32 dcp_count;
	u32 dcp_size;
	u32 mbi_index;
	u32 dcp_index;
	struct dma_buffer dpb_buffer;
	struct dma_buffer dcp_buffer[MAX_DCP_NUM];
	struct dma_buffer mbi_buffer[MAX_MBI_NUM];
	struct dma_buffer stream_buffer;
	struct dma_buffer udata_buffer;
	enum ARV_FRAME_TYPE arv_type;
	u32 beginning;

	struct file *crc_fp;
	loff_t pos;

	int frm_dis_delay;
	int frm_dec_delay;
	int frm_total_num;

	long total_qbuf_bytes;
	long total_write_bytes;
	u32 extra_size;
	s64 output_ts;
	s64 capture_ts;

	struct v4l2_fract fixed_frame_interval;
	struct v4l2_fract frame_interval;

	struct list_head cmd_q;
	struct vpu_dec_cmd_request *pending;
	struct mutex cmd_lock;

	struct vpu_dec_perf_time perf_time;
	int res_change_occu_count;
	int res_change_send_count;
	int res_change_done_count;

	struct list_head perf_q;
	struct mutex perf_lock;

	struct mutex fw_flow_mutex;

	u8 colorspace;
	u8 xfer_func;
	u8 ycbcr_enc;
	u8 quantization;
};

#define LVL_WARN		(1 << 0)
#define LVL_EVENT		(1 << 1)
#define LVL_INFO		(1 << 2)
#define LVL_BIT_CMD		(1 << 4)
#define LVL_BIT_EVT		(1 << 5)
#define LVL_BIT_TS		(1 << 6)
#define LVL_BIT_FRAME_BYTES	(1 << 7)
#define LVL_BIT_WPTR		(1 << 8)
#define LVL_BIT_PIC_ADDR	(1 << 9)
#define LVL_BIT_BUFFER_STAT	(1 << 10)
#define LVL_BIT_BUFFER_DESC	(1 << 11)
#define LVL_BIT_FUNC		(1 << 12)
#define LVL_BIT_FLOW		(1 << 13)
#define LVL_BIT_FRAME_COUNT	(1 << 14)
#define LVL_BIT_STRM_INFO	(1 << 15)

#define vpu_err(fmt, arg...) pr_info("[VPU Decoder] " fmt, ## arg)

#define vpu_dbg(level, fmt, arg...) \
	do { \
		if (vpu_dbg_level_decoder & (level)) \
			pr_info("[VPU Decoder] " fmt, ## arg); \
	} while (0)

#define V4L2_NXP_BUF_FLAG_CODECCONFIG		0x00200000
#define V4L2_NXP_BUF_FLAG_TIMESTAMP_INVALID	0x00400000

#define V4L2_NXP_BUF_MASK_FLAGS		(V4L2_NXP_BUF_FLAG_CODECCONFIG | \
					 V4L2_NXP_BUF_FLAG_TIMESTAMP_INVALID)

#define VPU_DECODED_EVENT_PERF_MASK		(1 << 0)
#define VPU_READY_EVENT_PERF_MASK		(1 << 1)

#define V4L2_NXP_FRAME_VERTICAL_ALIGN		512
#define V4L2_NXP_FRAME_HORIZONTAL_ALIGN		512

#define VPU_IMX_DECODER_FUSE_OFFSET		14

pSTREAM_BUFFER_DESCRIPTOR_TYPE get_str_buffer_desc(struct vpu_ctx *ctx);
u_int32 got_free_space(u_int32 wptr, u_int32 rptr, u_int32 start, u_int32 end);
int copy_buffer_to_stream(struct vpu_ctx *ctx, void *buffer, uint32_t length);

#endif
