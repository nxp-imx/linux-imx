/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Coda series multi-standard codec IP - helper definitions
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#ifndef __CODA_VPUAPI_H__
#define __CODA_VPUAPI_H__

#include <linux/kfifo.h>
#include <linux/idr.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ctrls.h>
#include "coda-vpuconfig.h"
#include "coda-vdi.h"

#define DEFAULT_FRAME_RATE 30
#define DEFAULT_INTERVAL_MODE 3
#define DEFAULT_IDR_INTERVAL 1
#define DEFAULT_INTRA_COST_WEIGHT 800
#define DEFAULT_INTRA_QP -1
#define DEFAULT_FIELD_REFERENCE_MODE 1
#define DEFAULT_EN_LINE_BUFFER_INTERRUPT 0
#define DEFAULT_EN_FIELD_ENCODING 0
#define DEFAULT_EN_CONSECUTIVE_INTRA_REFRESH 1
#define DEFAULT_EN_COUNT_INTRA_MB 1

/* H.264 profile for encoder*/
#define H264_PROFILE_BP 0
#define H264_PROFILE_MP 1
#define H264_PROFILE_HP 2

#define H264_SPS_TYPE 0
#define H264_PPS_TYPE 1

#define H264_VUI_SAR_IDC_EXTENDED 255

enum vpu_instance_type {
	VPU_INST_TYPE_DEC = 0,
	VPU_INST_TYPE_ENC = 1
};

enum vpu_instance_state {
	VPU_INST_STATE_NONE = 0,
	VPU_INST_STATE_OPEN = 1,
	VPU_INST_STATE_INIT_SEQ = 2,
	VPU_INST_STATE_PIC_RUN = 3,
	VPU_INST_STATE_SEEK = 4,
	VPU_INST_STATE_STOP = 5
};

/*
 * common struct and definition
 */
enum coda_product_id {
	PRODUCT_ID_980 = 0,
	PRODUCT_ID_960 = 1,
	PRODUCT_ID_NONE,
};

enum coda_std {
	AVC_ENC = 8,
	MP4_ENC = 11,
	STD_UNKNOWN,
};

enum coda_tiled_map_type {
	LINEAR_FRAME_MAP = 0, /* Linear frame map type */
	TILED_FRAME_V_MAP = 1, /* Tiled frame vertical map type */
	TILED_FRAME_H_MAP = 2, /* Tiled frame horizontal map type */
	TILED_FIELD_V_MAP = 3, /* Tiled field vertical map type */
	TILED_MIXED_V_MAP = 4, /* Tiled mixed vertical map type */
	TILED_FRAME_MB_RASTER_MAP = 5, /* Tiled frame MB raster map type */
	TILED_FIELD_MB_RASTER_MAP = 6, /* Tiled field MB raster map type */
	TILED_FRAME_NO_BANK_MAP = 7, /* Tiled frame no bank map */
	TILED_FIELD_NO_BANK_MAP = 8, /* Tiled field no bank map */
	LINEAR_FIELD_MAP = 9, /* Linear field map type */
	CODA_TILED_MAP_TYPE_MAX = 10,
	TILED_MAP_TYPE_MAX
};

enum coda_sw_reset_mode {
	SW_RESET_SAFETY,
	SW_RESET_FORCE,
	SW_RESET_ON_BOOT
};

enum coda_rate_control_type {
	RATE_CONTROL_TYPE_NONE,
	RATE_CONTROL_TYPE_CBR,
	RATE_CONTROL_TYPE_ABR
};

enum coda_slice_size_mode {
	SLICE_SIZE_MODE_NONE,
	SLICE_SIZE_MODE_BIT,
	SLICE_SIZE_MODE_MB
};

enum coda_me_search_range_x {
	ME_SEARCH_RANGE_X_64X64,
	ME_SEARCH_RANGE_X_48X48,
	ME_SEARCH_RANGE_X_32X32,
	ME_SEARCH_RANGE_X_16X16
};

enum coda_me_search_range_y {
	ME_SEARCH_RANGE_Y_48X48,
	ME_SEARCH_RANGE_Y_32X32,
	ME_SEARCH_RANGE_Y_16X16
};

enum coda_frame_buffer_format {
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

	/* 4:4:4 packed format */
	/* little endian perspective */
	/* | addr 0 | addr 1 | */
	FORMAT_444_P10_16BIT_MSB, /* lsb |000000xx |xxxxxxxx | msb */
	FORMAT_444_P10_16BIT_LSB, /* lsb |xxxxxxxx |xx000000 | msb */
	FORMAT_444_P10_32BIT_MSB, /* lsb |00xxxxxxxxxxxxxxxxxxxxxxxxxxx| msb */
	FORMAT_444_P10_32BIT_LSB, /* lsb |xxxxxxxxxxxxxxxxxxxxxxxxxxx00| msb */

	/* 4:0:0 packed format */
	/* little endian perspective */
	/* | addr 0 | addr 1 | */
	FORMAT_400_P10_16BIT_MSB, /* lsb |000000xx |xxxxxxxx | msb */
	FORMAT_400_P10_16BIT_LSB, /* lsb |xxxxxxxx |xx000000 | msb */
	FORMAT_400_P10_32BIT_MSB, /* lsb |00xxxxxxxxxxxxxxxxxxxxxxxxxxx| msb */
	FORMAT_400_P10_32BIT_LSB, /* lsb |xxxxxxxxxxxxxxxxxxxxxxxxxxx00| msb */

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

	FORMAT_RGB_32BIT_PACKED = 90,
	FORMAT_YUV444_32BIT_PACKED,
	FORMAT_RGB_P10_32BIT_PACKED,
	FORMAT_YUV444_P10_32BIT_PACKED,

	FORMAT_RGB_24BIT_PACKED = 95,
	FORMAT_YUV444_24BIT_PACKED,
	FORMAT_YUV444_24BIT,

	FORMAT_MAX,
};

enum coda_pic_type {
	PIC_TYPE_I = 0, /* I picture */
	PIC_TYPE_KEY = 0, /* KEY frame for AV1*/
	PIC_TYPE_P = 1, /* P picture */
	PIC_TYPE_INTER = 1, /* Inter frame for AV1*/
	PIC_TYPE_B = 2, /* B picture (except VC1) */
	PIC_TYPE_REPEAT = 2, /* Repeat frame (VP9 only) */
	PIC_TYPE_AV1_INTRA = 2, /* Intra only frame (AV1 only) */
	PIC_TYPE_VC1_BI = 2, /* VC1 BI picture (VC1 only) */
	PIC_TYPE_VC1_B = 3, /* VC1 B picture (VC1 only) */
	PIC_TYPE_D = 3, /* D picture in MPEG2 that is only composed of DC coefficients */
	PIC_TYPE_S = 3, /* S picture in MPEG4 that is an acronym of Sprite and used for GMC */
	PIC_TYPE_AVS2_F = 3, /* F picture in AVS2 */
	PIC_TYPE_AV1_SWITCH = 3, /* Switch frame (AV1 only) */
	PIC_TYPE_VC1_P_SKIP = 4, /* VC1 P skip picture (VC1 only) */
	PIC_TYPE_MP4_P_SKIP_NOT_CODED = 4, /* Not Coded P Picture in MPEG4 packed mode */
	PIC_TYPE_AVS2_S = 4, /* S picture in AVS2 */
	PIC_TYPE_IDR = 5, /* H.264/H.265 IDR picture */
	PIC_TYPE_AVS2_G = 5, /* G picture in AVS2 */
	PIC_TYPE_AVS2_GB = 6, /* GB picture in AVS2 */
	PIC_TYPE_MAX /* No Meaning */
};

enum coda_rotation_angle {
	ROT_ANG_0 = 0,
	ROT_ANG_90 = 90,
	ROT_ANG_180 = 180,
	ROT_ANG_270 = 270
};

enum coda_mirror_direction {
	MIR_DIR_NONE, /* no mirroring */
	MIR_DIR_VER, /* vertical mirroring */
	MIR_DIR_HOR, /* horizontal mirroring */
	MIR_DIR_HOR_VER /* horizontal and vertical mirroring */
};

struct coda_sec_axi_info {
	bool bit_enable;
	bool ip_enable;
	bool dbk_y_enable;
	bool dbk_c_enable;
	dma_addr_t bit_buf;
	dma_addr_t ip_buf;
	dma_addr_t dbk_y_buf;
	dma_addr_t dbk_c_buf;
};

struct coda_frame_buffer {
	dma_addr_t buf_y;
	dma_addr_t buf_cb;
	dma_addr_t buf_cr;
	dma_addr_t buf_y_bot;
	dma_addr_t buf_cb_bot;
	dma_addr_t buf_cr_bot;
	enum coda_tiled_map_type map_type;
	unsigned int stride;
	unsigned int height;
	int index;
};

struct coda_tiled_map_config {
	int xy2axi_luma_map[32];
	int xy2axi_chr_map[32];
	int xy2axi_config;
	int tb_separate_map;
	int top_bot_split;
	int tiled_map;
	int conv_linear;
};

struct coda_enc_report_info {
	u32 size;
	u32 ext_data; /* mv type for mv info, slice num for slice info */
	size_t buf_offset;
};

struct coda_rect {
	u32 left;
	u32 top;
	u32 right;
	u32 bottom;
};

struct coda_sar_info {
	bool enable;
	u8 idc;
	u16 width;
	u16 height;
};

struct coda_color_param {
	bool color_range;
	u8 matrix_coefficients;
	u8 transfer_characteristics;
	u8 color_primaries;
};

struct coda_enc_open_param {
	u32 pic_width;
	u32 pic_height;
	u32 profile;
	u32 level;
	u32 framerate;
	enum coda_mirror_direction mir_dir;
	enum coda_rotation_angle rot_ang;
	size_t sec_axi_size;
	dma_addr_t sec_axi_base;
	u32 me_blk_mode;
	u32 me_use_zero_pmv;
	u32 me_search_range_y;
	u32 me_search_range_x;
	s32 chroma_qp_offset;
	u32 deblk_filter_idc;
	s32 deblk_filter_offset_beta;
	s32 deblk_filter_offset_alpha;
	u32 slice_size_mode;
	u32 slice_size;
	u32 idr_interval;
	u32 rc_gop_i_qp_offset;
	u32 en_rc_gop_i_qp_offset;
	u32 gop_size;
	u32 initial_delay;
	u32 bitrate;
	u32 max_intra_size;
	u32 intra_refresh_mb_num;
	int intra_qp;
	u32 min_qp;
	u32 max_qp;
	u32 field_ref_mode;
	u32 gamma;
	u32 mb_interval;
	u32 interval_mode;
	u32 intra_cost_weight;
	u32 rate_control_type;
	u32 cabac_init_idc;
	enum coda_tiled_map_type map_type;
	struct coda_rect conf_win;
	struct coda_sar_info sar;
	struct coda_color_param color;

	u32 en_cabac_mode: 1;
	u32 en_transform_8x8: 1;
	u32 en_constrained_intra_pred: 1;
	u32 en_consecutive_intra_refresh: 1;
	u32 en_count_intra_mb: 1;
	u32 en_field_seq_intra_refresh: 1;
	u32 en_aud: 1;
	u32 en_field_encoding: 1;
	u32 en_line_buf_int: 1;
	u32 en_frame_skip: 1;
	u32 en_strict_cbr: 1;
	u32 en_max_intra_size: 1;
};

struct coda_enc_param {
	u32 bitrate;
	u32 quant_param;
	u32 field_run;
	u32 skip_picture: 1;
	int src_idx;
	u64 pts;
	struct coda_frame_buffer *source_frame;
	bool force_i_picture;
	dma_addr_t bitstream_buf;
	u32 bitstream_buf_size;
	u32 src_end_flag;
};

struct coda_enc_output_info {
	int pic_type;
	dma_addr_t wr_ptr;
	dma_addr_t rd_ptr;
	dma_addr_t bitstream_buf;
	int bitstream_buf_size;
	int frame_cycle; /* this variable reports the number of cycles for processing a frame. */
	int enc_src_idx;
	int recon_frame_index;
	u64 pts;
	u32 num_of_slice;
	u32 bitstream_wrap_around;
	u32 encoding_success;
	u32 avg_ctu_qp;
	struct coda_enc_report_info mb_info;
	struct coda_enc_report_info mv_info;
	struct coda_enc_report_info slice_info;
	struct coda_enc_report_info cost_info;
};

struct coda_enc_info {
	struct coda_enc_open_param open_param;
	int pic_width;
	int pic_height;
	int stride;
	int frame_buf_num;
	int frame_buf_height;
	u8 frame_addr[CODA_MAX_FBS][3][4];
	dma_addr_t stream_wr_ptr;
	dma_addr_t stream_rd_ptr;
	u32 src_index;
	int stream_end_flag;
	struct coda_sec_axi_info sec_axi;
	enum coda_frame_buffer_format wtl_format;
	struct coda_tiled_map_config map_cfg;
	u32 cache_config;
	dma_addr_t bitstream_buf;
	u32 bitstream_buf_size;
	u32 header_size;
	u64 pts_map[32];
};

struct vpu_enc_controls {
	u32 flip;
	u32 rotate;
	u32 gop_size;
	u32 multi_slice_mode;
	u32 multi_slice_max_mb;
	u32 cyclic_intra_refresh_mb;
	u32 intra_refresh_period;
	u32 mv_h_search_range;
	u32 mv_v_search_range;
	u32 bitrate_mode;
	u32 bitrate;
	u32 frame_rc_enable;
	u32 mb_rc_enable;
	u32 frame_skip_mode;
	u32 force_key_frame;
	u32 h264_cpb_size;
	u32 h264_profile;
	u32 h264_level;
	u32 h264_i_frame_qp;
	u32 h264_p_frame_qp;
	u32 h264_min_qp;
	u32 h264_max_qp;
	u32 h264_entropy_mode;
	u32 h264_8x8_transform;
	u32 h264_constrained_intra_prediction;
	s32 h264_chroma_qp_index_offset;
	u32 h264_loop_filter_mode;
	s32 h264_loop_filter_beta;
	s32 h264_loop_filter_alpha;
	u32 h264_vui_sar_enable;
	u32 h264_vui_sar_idc;
	u32 h264_vui_ext_sar_width;
	u32 h264_vui_ext_sar_height;
};

struct vpu_instance;

struct vpu_instance_ops {
	int (*start_process)(struct vpu_instance *inst);
	void (*stop_process)(struct vpu_instance *inst);
	void (*finish_process)(struct vpu_instance *inst);
};

struct vpu_instance {
	struct v4l2_fh v4l2_fh;
	struct v4l2_ctrl_handler v4l2_ctrl_hdl;
	struct vpu_device *vpu_dev;
	struct v4l2_pix_format_mplane src_fmt;
	struct v4l2_pix_format_mplane dst_fmt;
	struct v4l2_rect crop;
	struct v4l2_rect codec_rect;
	enum v4l2_colorspace colorspace;
	enum v4l2_xfer_func xfer_func;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;

	enum vpu_instance_state state;
	enum vpu_instance_type type;
	const struct vpu_instance_ops *ops;

	enum coda_std std;
	int id;
	union {
		struct coda_enc_info enc_info;
	} *codec_info;
	struct vpu_buf work_vbuf;
	struct vpu_buf report_vbuf;
	struct vpu_buf frame_vbuf[CODA_MAX_FBS];
	struct coda_frame_buffer frame_buf[CODA_MAX_FBS];
	bool monochrome;
	bool cbcr_interleave;
	bool nv21;
	unsigned int framerate;
	struct vpu_enc_controls enc_ctrls;

	u32 queued_src_buf_num;
	u32 queued_dst_buf_num;
	bool eos;

	ktime_t ts_start;
	ktime_t ts_finish;
	u64 total_sw_time;
	u32 processed_buf_num;
};

struct vpu_device {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct list_head instances;
	struct video_device *video_dev_enc;
	const char *fw_name;
	struct mutex dev_lock; /* lock for the src, dst v4l2 queues */
	struct mutex hw_lock; /* lock hw configurations */
	int irq;
	struct vpu_buf common_mem;
	struct gen_pool *sram_pool;
	struct vpu_buf sram_buf;
	void __iomem *reg_base;
	u32 product_code;
	enum coda_product_id product_id;
	struct completion irq_done;
	struct ida inst_ida;
	struct delayed_work task_timer;
	struct kthread_work work;
	struct kthread_worker *worker;
	int num_clks;
	struct clk_bulk_data *clks;
	u32 reg_bk[64];
};

int coda_vpuapi_init_with_bitcode(struct device *dev, u8 *code, size_t size);
int coda_vpuapi_sleep_wake(struct device *dev, bool sleep);
int coda_vpuapi_get_version_info(struct device *dev, u32 *version, u32 *revision);
int coda_vpuapi_enc_open(struct vpu_instance *inst, struct coda_enc_open_param *open_param);
int coda_vpuapi_enc_close(struct vpu_instance *inst, u32 *fail_res);
int coda_vpuapi_enc_issue_seq_init(struct vpu_instance *inst);
int coda_vpuapi_enc_register_frame_buffer(struct vpu_instance *inst, unsigned int num,
					  unsigned int stride, int height,
					  enum coda_tiled_map_type map_type);
int coda_vpuapi_enc_start_one_frame(struct vpu_instance *inst, struct coda_enc_param *param,
				    u32 *fail_res);
int coda_vpuapi_enc_get_output_info(struct vpu_instance *inst, struct coda_enc_output_info *info);

#endif
