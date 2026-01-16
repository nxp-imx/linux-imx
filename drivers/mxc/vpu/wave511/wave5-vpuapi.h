/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave5 series multi-standard codec IP - helper definitions
 *
 * Copyright (C) 2021-2026 CHIPS&MEDIA INC
 */

#ifndef VPUAPI_H_INCLUDED
#define VPUAPI_H_INCLUDED

#include <linux/idr.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ctrls.h>
#include "wave5-vpuerror.h"
#include "wave5-vpuconfig.h"
#include "wave5-vdi.h"
#include "wave5-vpu-ctrl.h"

enum product_id {
	PRODUCT_ID_515,
	PRODUCT_ID_521,
	PRODUCT_ID_511,
	PRODUCT_ID_517,
	PRODUCT_ID_NONE,
};

struct vpu_attr;

enum vpu_instance_type {
	VPU_INST_TYPE_DEC = 0
};

enum vpu_instance_state {
	VPU_INST_STATE_NONE = 0,
	VPU_INST_STATE_OPEN = 1,
	VPU_INST_STATE_INIT_SEQ = 2,
	VPU_INST_STATE_PIC_RUN = 3,
	VPU_INST_STATE_STOP = 4,
	VPU_INST_STATE_ERROR = 5
};

/* Maximum available on hardware. */
#define WAVE5_MAX_FBS 31

#define MAX_REG_FRAME (WAVE5_MAX_FBS * 2)

#define WAVE5_DEC_HEVC_BUF_SIZE(_w, _h) (DIV_ROUND_UP(_w, 64) * DIV_ROUND_UP(_h, 64) * 256 + 64)
#define WAVE5_DEC_AVC_BUF_SIZE(_w, _h) ((((ALIGN(_w, 256) / 16) * (ALIGN(_h, 16) / 16)) + 16) * 80)

#define WAVE5_FBC_LUMA_TABLE_SIZE(_w, _h) (ALIGN(_h, 64) * ALIGN(_w, 256) / 32)
#define WAVE5_FBC_CHROMA_TABLE_SIZE(_w, _h) (ALIGN((_h), 64) * ALIGN((_w) / 2, 256) / 32)

/*
 * common struct and definition
 */
enum cod_std {
	STD_AVC = 0,
	STD_HEVC = 12,
	STD_MAX
};

enum wave_std {
	W_HEVC_DEC = 0x00,
	W_AVC_DEC = 0x02,
	STD_UNKNOWN = 0xFF
};

/************************************************************************/
/* PROFILE & LEVEL */
/************************************************************************/
/* HEVC */
#define HEVC_PROFILE_MAIN 1
#define HEVC_PROFILE_MAIN10 2
#define HEVC_PROFILE_STILLPICTURE 3
#define HEVC_PROFILE_MAIN10_STILLPICTURE 2

/************************************************************************/
/* error codes */
/************************************************************************/

/************************************************************************/
/* utility macros */
/************************************************************************/

/* Initialize sequence firmware command mode */
#define INIT_SEQ_NORMAL				1

/* Decode firmware command mode */
#define DEC_PIC_NORMAL				0

/* bitstream_buffer_size */
#define MIN_BITSTREAM_BUFFER_SIZE		1024
#define MIN_BITSTREAM_BUFFER_SIZE_WAVE521	(1024 * 64)

#define BUFFER_MARGIN				4096

#define MAX_FIRMWARE_CALL_RETRY			10

/*
 * Parameters of DEC_SET_SEQ_CHANGE_MASK
 */
#define SEQ_CHANGE_ENABLE_PROFILE BIT(5)
#define SEQ_CHANGE_ENABLE_SIZE BIT(16)
#define SEQ_CHANGE_ENABLE_BITDEPTH BIT(18)
#define SEQ_CHANGE_ENABLE_DPB_COUNT BIT(19)
#define SEQ_CHANGE_ENABLE_ASPECT_RATIO BIT(21)
#define SEQ_CHANGE_ENABLE_VIDEO_SIGNAL BIT(23)
#define SEQ_CHANGE_ENABLE_VUI_TIMING_INFO BIT(29)

#define SEQ_CHANGE_ENABLE_ALL_HEVC (SEQ_CHANGE_ENABLE_PROFILE | \
		SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT)

#define SEQ_CHANGE_ENABLE_ALL_AVC (SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT | \
		SEQ_CHANGE_ENABLE_ASPECT_RATIO | \
		SEQ_CHANGE_ENABLE_VIDEO_SIGNAL | \
		SEQ_CHANGE_ENABLE_VUI_TIMING_INFO)

#define DISPLAY_IDX_FLAG_SEQ_END -1
#define DISPLAY_IDX_FLAG_NO_FB -3
#define DECODED_IDX_FLAG_NO_FB -1
#define DECODED_IDX_FLAG_SKIP -2

enum codec_command {
	DEC_GET_QUEUE_STATUS,
	DEC_RESET_FRAMEBUF_INFO,
	DEC_GET_SEQ_INFO,
};

enum frame_buffer_format {
	FORMAT_ERR = -1,
	FORMAT_420 = 0, /* 8bit */
	FORMAT_422, /* 8bit */
	FORMAT_224, /* 8bit */
	FORMAT_444, /* 8bit */
	FORMAT_400, /* 8bit */

	/* little endian perspective */
	/* | addr 0 | addr 1 | */
	FORMAT_420_P10_16BIT_MSB = 5, /* lsb |000000xx|xxxxxxxx | msb */
	FORMAT_420_P10_16BIT_LSB, /* lsb |xxxxxxx |xx000000 | msb */
	FORMAT_420_P10_32BIT_MSB, /* lsb |00xxxxxxxxxxxxxxxxxxxxxxxxxxx| msb */
	FORMAT_420_P10_32BIT_LSB, /* lsb |xxxxxxxxxxxxxxxxxxxxxxxxxxx00| msb */

	/* 4:2:2 packed format */
	/* little endian perspective */
	/* | addr 0 | addr 1 | */
	FORMAT_422_P10_16BIT_MSB, /* lsb |000000xx |xxxxxxxx | msb */
	FORMAT_422_P10_16BIT_LSB, /* lsb |xxxxxxxx |xx000000 | msb */
	FORMAT_422_P10_32BIT_MSB, /* lsb |00xxxxxxxxxxxxxxxxxxxxxxxxxxx| msb */
	FORMAT_422_P10_32BIT_LSB, /* lsb |xxxxxxxxxxxxxxxxxxxxxxxxxxx00| msb */

	FORMAT_YUYV, /* 8bit packed format : Y0U0Y1V0 Y2U1Y3V1 ... */
	FORMAT_YUYV_P10_16BIT_MSB,
	FORMAT_YUYV_P10_16BIT_LSB,
	FORMAT_YUYV_P10_32BIT_MSB,
	FORMAT_YUYV_P10_32BIT_LSB,

	FORMAT_YVYU, /* 8bit packed format : Y0V0Y1U0 Y2V1Y3U1 ... */
	FORMAT_YVYU_P10_16BIT_MSB,
	FORMAT_YVYU_P10_16BIT_LSB,
	FORMAT_YVYU_P10_32BIT_MSB,
	FORMAT_YVYU_P10_32BIT_LSB,

	FORMAT_UYVY, /* 8bit packed format : U0Y0V0Y1 U1Y2V1Y3 ... */
	FORMAT_UYVY_P10_16BIT_MSB,
	FORMAT_UYVY_P10_16BIT_LSB,
	FORMAT_UYVY_P10_32BIT_MSB,
	FORMAT_UYVY_P10_32BIT_LSB,

	FORMAT_VYUY, /* 8bit packed format : V0Y0U0Y1 V1Y2U1Y3 ... */
	FORMAT_VYUY_P10_16BIT_MSB,
	FORMAT_VYUY_P10_16BIT_LSB,
	FORMAT_VYUY_P10_32BIT_MSB,
	FORMAT_VYUY_P10_32BIT_LSB,

	FORMAT_MAX,
};

enum packed_format_num {
	NOT_PACKED = 0,
	PACKED_YUYV,
	PACKED_YVYU,
	PACKED_UYVY,
	PACKED_VYUY,
};

enum wave5_interrupt_bit {
	INT_WAVE5_INIT_VPU = 0,
	INT_WAVE5_WAKEUP_VPU = 1,
	INT_WAVE5_SLEEP_VPU = 2,
	INT_WAVE5_CREATE_INSTANCE = 3,
	INT_WAVE5_FLUSH_INSTANCE = 4,
	INT_WAVE5_DESTROY_INSTANCE = 5,
	INT_WAVE5_INIT_SEQ = 6,
	INT_WAVE5_SET_FRAMEBUF = 7,
	INT_WAVE5_DEC_PIC = 8,
	INT_WAVE5_REQ_WORK_BUF = 12,
	INT_WAVE5_DEC_QUERY = 14,
	INT_WAVE5_BSBUF_EMPTY = 15,
};

enum pic_type {
	PIC_TYPE_I = 0,
	PIC_TYPE_P = 1,
	PIC_TYPE_B = 2,
	PIC_TYPE_IDR = 5, /* H.264/H.265 IDR (Instantaneous Decoder Refresh) picture */
	PIC_TYPE_MAX /* no meaning */
};

enum sw_reset_mode {
	SW_RESET_SAFETY,
	SW_RESET_FORCE,
	SW_RESET_ON_BOOT
};

enum tiled_map_type {
	LINEAR_FRAME_MAP = 0, /* linear frame map type */
	COMPRESSED_FRAME_MAP = 17, /* compressed frame map type*/
};

enum temporal_id_mode {
	TEMPORAL_ID_MODE_ABSOLUTE,
	TEMPORAL_ID_MODE_RELATIVE,
};

struct vpu_attr {
	u32 product_id;
	char product_name[8]; /* product name in ascii code */
	u32 product_version;
	u32 fw_version;
	u32 fw_api_version;
	u32 customer_id;
	u32 support_decoders; /* bitmask */
	u32 support_backbone: 1;
	u32 support_avc10bit_dec: 1;
	u32 support_hevc10bit_dec: 1;
	u32 support_vcore_backbone: 1;
	u32 support_vcpu_backbone: 1;
	u32 support_scaler: 1;
};

struct frame_buffer {
	dma_addr_t buf_y;
	dma_addr_t buf_cb;
	dma_addr_t buf_cr;
	enum tiled_map_type map_type;
	unsigned int stride; /* horizontal stride for the given frame buffer */
	unsigned int width; /* width of the given frame buffer */
	unsigned int height; /* height of the given frame buffer */
	size_t size; /* size of the given frame buffer */
	unsigned int sequence_no;
	unsigned int index;
	bool update_fb_info;
};

struct vpu_rect {
	unsigned int left; /* horizontal pixel offset from left edge */
	unsigned int top; /* vertical pixel offset from top edge */
	unsigned int right; /* horizontal pixel offset from right edge */
	unsigned int bottom; /* vertical pixel offset from bottom edge */
};

struct color_param {
	bool video_signal_type_present;
	bool color_range;
	bool color_description_present;
	u8 matrix_coefficients;
	u8 transfer_characteristics;
	u8 color_primaries;
};

/*
 * decode struct and definition
 */

struct dec_scaler_info {
	bool enable;
	int width;
	int height;
};

struct dec_open_param {
	dma_addr_t bitstream_buffer;
	size_t bitstream_buffer_size;
	bool reorder_enable;
};

struct dec_initial_info {
	u32 pic_width;
	u32 pic_height;
	struct vpu_rect pic_crop_rect;
	u32 min_frame_buffer_count; /* between 1 to 16 */
	u32 reorder_delay;

	u32 profile;
	u32 luma_bitdepth; /* bit-depth of the luma sample */
	u32 chroma_bitdepth; /* bit-depth of the chroma sample */
	u32 err_reason;
	u32 warn_info;
	dma_addr_t rd_ptr; /* read pointer of bitstream buffer */
	dma_addr_t wr_ptr; /* write pointer of bitstream buffer */
	u32 sequence_no;
	u32 vlc_buf_size;
	u32 param_buf_size;
	struct color_param color;
};

struct dec_output_info {
	/**
	 * This is a frame buffer index for the picture to be displayed at the moment
	 * among frame buffers which are registered using vpu_dec_register_frame_buffer().
	 * Frame data that will be displayed is stored in the frame buffer with this index
	 * When there is no display delay, this index is always the equal to
	 * index_frame_decoded, however, if displaying is delayed (for display
	 * reordering in AVC or B-frames in VC1), this index might be different to
	 * index_frame_decoded. By checking this index, HOST applications can easily figure
	 * out whether sequence decoding has been finished or not.
	 *
	 * -3(0xFFFD) or -2(0xFFFE) : when a display output cannot be given due to picture
	 * reordering or skip option
	 * -1(0xFFFF) : when there is no more output for display at the end of sequence
	 * decoding
	 */
	s32 index_frame_display;
	/**
	 * This is the frame buffer index of the decoded picture among the frame buffers which were
	 * registered using vpu_dec_register_frame_buffer(). The currently decoded frame is stored
	 * into the frame buffer specified by this index.
	 *
	 * -2 : indicates that no decoded output is generated because decoder meets EOS
	 * (end of sequence) or skip
	 * -1 : indicates that the decoder fails to decode a picture because there is no available
	 * frame buffer
	 */
	s32 index_frame_decoded;
	s32 index_frame_decoded_for_tiled;
	u32 nal_type;
	unsigned int pic_type;
	struct vpu_rect rc_display;
	unsigned int disp_pic_width;
	unsigned int disp_pic_height;
	struct vpu_rect rc_decoded;
	u32 dec_pic_width;
	u32 dec_pic_height;
	s32 decoded_poc;
	int temporal_id; /* temporal ID of the picture */
	dma_addr_t rd_ptr; /* stream buffer read pointer for the current decoder instance */
	dma_addr_t wr_ptr; /* stream buffer write pointer for the current decoder instance */
	struct frame_buffer disp_frame;
	u32 frame_display_flag; /* it reports a frame buffer flag to be displayed */
	/**
	 * this variable reports that sequence has been changed while H.264/AVC stream decoding.
	 * if it is 1, HOST application can get the new sequence information by calling
	 * vpu_dec_get_initial_info() or wave5_vpu_dec_issue_seq_init().
	 *
	 * for H.265/HEVC decoder, each bit has a different meaning as follows.
	 *
	 * sequence_changed[5] : it indicates that the profile_idc has been changed
	 * sequence_changed[16] : it indicates that the resolution has been changed
	 * sequence_changed[19] : it indicates that the required number of frame buffer has
	 * been changed.
	 */
	unsigned long frame_cycle; /* reports the number of cycles for processing a frame */
	u32 sequence_no;

	u32 dec_host_cmd_tick; /* tick of DEC_PIC command for the picture */
	u32 dec_seek_start_tick;
	u32 dec_seek_end_tick;
	u32 dec_parse_start_tick;
	u32 dec_parse_end_tick;
	u32 dec_decode_start_tick;
	u32 dec_decode_end_tick; /* end tick of decoding slices of the picture */

	u32 sequence_changed;

	u32 err_reason;
	u32 warn_info;
};

struct queue_status_info {
	u32 instance_queue_count;
	u32 report_queue_count;
};

struct sec_axi_info {
	bool use_ip_enable;
	bool use_bit_enable;
	bool use_lf_row_enable;
	bool use_scaler_enable;
};

struct dec_info {
	struct dec_open_param open_param;
	struct dec_initial_info initial_info;
	struct dec_initial_info new_seq_info; /* temporal new sequence information */
	dma_addr_t stream_wr_ptr;
	dma_addr_t stream_rd_ptr;
	u32 frame_display_flag;
	struct vpu_buf vb_mv[WAVE5_MAX_FBS];
	struct vpu_buf vb_fbc_y_tbl[WAVE5_MAX_FBS];
	struct vpu_buf vb_fbc_c_tbl[WAVE5_MAX_FBS];
	struct frame_buffer disp_buf[WAVE5_MAX_FBS];
	unsigned int num_of_decoding_fbs: 7;
	unsigned int num_of_display_fbs: 7;
	unsigned int stride;
	struct sec_axi_info sec_axi_info;
	dma_addr_t user_data_buf_addr;
	u32 user_data_enable;
	u32 user_data_buf_size;
	struct vpu_buf vb_task;
	struct dec_output_info dec_out_info[WAVE5_MAX_FBS];
	u32 seq_change_mask;
	enum temporal_id_mode temp_id_select_mode;
	u32 target_temp_id;
	u32 target_spatial_id;
	u32 instance_queue_count;
	u32 report_queue_count;
	u32 cycle_per_tick;
	u32 vlc_buf_size;
	u32 param_buf_size;
	bool initial_info_obtained;
	bool reorder_enable;
	bool first_cycle_check;
	u32 stream_endflag: 1;
};

struct vpu_device {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *v4l2_m2m_dec_dev;
	struct list_head instances;
	spinlock_t inst_lock; /* lock instance list */
	struct video_device *video_dev_dec;
	struct mutex dev_lock; /* lock for the src, dst v4l2 queues */
	struct mutex hw_lock; /* lock hw configurations */
	int irq;
	const struct wave5_match_data *res;
	enum product_id product;
	struct vpu_attr attr;
	struct vpu_buf common_mem;
	struct vpu_buf temp_vbuf;
	u32 last_performance_cycles;
	struct device *ctrl;
	void __iomem *vdb_register;
	u32 product_code;
	struct wave5_vpu_entity entity;
	bool active;
	struct clk_bulk_data *clks;
	struct hrtimer hrtimer;
	struct kthread_work work;
	struct kthread_worker *worker;
	int vpu_poll_interval;
	int num_clks;
	struct reset_control *resets;

	struct dentry *debugfs;
};

struct vpu_instance;

struct vpu_instance_ops {
	void (*finish_process)(struct vpu_instance *inst);
};

struct vpu_performance_info {
	ktime_t ts_first;
	ktime_t ts_last;
	s64 latency_first;
	s64 latency_max;
	s64 min_process_time;
	s64 max_process_time;
	u64 total_sw_time;
	u64 total_hw_time;
	u32 first_hw_time;
};

struct vpu_instance {
	struct list_head list;
	struct v4l2_fh v4l2_fh;
	struct v4l2_m2m_dev *v4l2_m2m_dev;
	struct v4l2_ctrl_handler v4l2_ctrl_hdl;
	struct vpu_device *dev;
	struct completion irq_done;
	bool enable;
	atomic_t refcount;
	wait_queue_head_t wq_irq;

	struct v4l2_pix_format_mplane src_fmt;
	struct v4l2_pix_format_mplane dst_fmt;
	enum v4l2_colorspace colorspace;
	enum v4l2_xfer_func xfer_func;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;

	enum vpu_instance_state state;
	enum vpu_instance_type type;
	const struct vpu_instance_ops *ops;
	spinlock_t state_spinlock; /* This protects the instance state */

	enum wave_std std;
	s32 id;
	union {
		struct dec_info dec_info;
	} *codec_info;
	unsigned long disp_buf_mask;
	unsigned long disp_buf_seek;
	struct frame_buffer frame_buf[WAVE5_MAX_FBS];
	struct vpu_buf frame_vbuf[WAVE5_MAX_FBS];
	u32 fbc_buf_count;
	u32 disp_buf_count;
	u32 queued_src_buf_num;
	u32 queued_dst_buf_num;
	u32 sequence;
	struct v4l2_rect conf_win;
	u64 timestamp;
	ktime_t ts_input;
	ktime_t ts_start;
	ktime_t ts_last_end;
	enum frame_buffer_format output_format;
	bool cbcr_interleave;
	bool nv21;
	bool eos;
	bool seek_flag;
	bool dynamic_source_change;
	bool needs_reallocation;
	struct dec_scaler_info scaler_info;
	u32 sram_size;
	atomic_t feed_frame_cnt;
	atomic_t queued_dec_cmd;

	struct vb2_v4l2_buffer *next_frame;
	bool retry_flag;

	u32 skiped_frame_num;
	u32 error_frame_num;
	u32 processed_buf_num;
	u32 displayed_buf_num;
	u32 total_dec_cnt;
	u32 drain_dec_cnt;
	struct vpu_performance_info performance;

	struct dentry *debugfs;
};

void wave5_vdi_write_register(struct vpu_device *vpu_dev, u32 addr, u32 data);
u32 wave5_vdi_read_register(struct vpu_device *vpu_dev, u32 addr);

int wave5_vpu_flush_instance(struct vpu_instance *inst);
int wave5_vpu_get_version_info(struct device *dev, u32 *revision, unsigned int *product_id);
int wave5_vpu_dec_open(struct vpu_instance *inst, struct dec_open_param *open_param);
int wave5_vpu_dec_close(struct vpu_instance *inst, u32 *fail_res);
int wave5_vpu_dec_issue_seq_init(struct vpu_instance *inst);
int wave5_vpu_dec_complete_seq_init(struct vpu_instance *inst, struct dec_initial_info *info);
void wave5_vpu_dec_fill_linear_frame(struct vpu_instance *inst,
				     struct frame_buffer *frame, struct vb2_buffer *vb);
int wave5_vpu_dec_register_frame_buffer_ex(struct vpu_instance *inst, int num_of_decoding_fbs,
					   int stride, int height);
int wave5_vpu_dec_register_display_buffer_ex(struct vpu_instance *inst,
					     struct frame_buffer *frame);
int wave5_vpu_dec_start_one_frame(struct vpu_instance *inst, u32 *res_fail);
int wave5_vpu_dec_get_output_info(struct vpu_instance *inst, struct dec_output_info *info);
int wave5_vpu_dec_reset_framebuffer(struct vpu_instance *inst, unsigned int index);
void wave5_vpu_dec_reset_disp_buf(struct vpu_instance *inst);
int wave5_vpu_dec_give_command(struct vpu_instance *inst, enum codec_command cmd, void *parameter);
int wave5_vpu_dec_clr_disp_flag(struct vpu_instance *inst, int index);
int wave5_vpu_dec_set_disp_flag(struct vpu_instance *inst, int index);
bool wave5_vpu_dec_is_cq_done(struct vpu_instance *inst);

#endif
