/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - wave6 helper interface
 *
 * Copyright (C) 2025 CHIPS&MEDIA INC
 */

#ifndef __WAVE6_VPUAPI_H__
#define __WAVE6_VPUAPI_H__

#include <linux/kfifo.h>
#include <linux/idr.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ctrls.h>
#include "wave6-vpuerror.h"
#include "wave6-vpuconfig.h"
#include "wave6-vdi.h"
#include "wave6-vpu-ctrl.h"

struct vpu_attr;

enum vpu_instance_type {
	VPU_INST_TYPE_DEC	= 0,
	VPU_INST_TYPE_ENC	= 1
};

enum vpu_instance_state {
	VPU_INST_STATE_NONE	= 0,
	VPU_INST_STATE_OPEN	= 1,
	VPU_INST_STATE_INIT_SEQ	= 2,
	VPU_INST_STATE_PIC_RUN	= 3,
	VPU_INST_STATE_SEEK	= 4,
	VPU_INST_STATE_STOP	= 5
};

#define WAVE6_MAX_FBS 31

#define WAVE6_DEC_HEVC_MVCOL_BUF_SIZE(_w, _h) \
	((ALIGN((_w), 256) / 16) * (ALIGN((_h), 64) / 16) * 1 * 16)
#define WAVE6_DEC_AVC_MVCOL_BUF_SIZE(_w, _h) \
	((ALIGN((_w), 64) / 16) * (ALIGN((_h), 16) / 16) * 5 * 16)
#define WAVE6_FBC_LUMA_TABLE_SIZE(_w, _h) \
	(ALIGN((_w), 256) * ALIGN((_h), 64) / 32)
#define WAVE6_FBC_CHROMA_TABLE_SIZE(_w, _h) \
	(ALIGN(((_w) / 2), 256) * ALIGN((_h), 64) / 32)
#define WAVE6_ENC_AVC_MVCOL_BUF_SIZE(_w, _h) \
	((ALIGN((_w), 512) / 512) * (ALIGN((_h), 16) / 16) * 16)
#define WAVE6_ENC_HEVC_MVCOL_BUF_SIZE(_w, _h) \
	((ALIGN((_w), 64) / 64) * (ALIGN((_h), 64) / 64) * 128)
#define WAVE6_ENC_SUBSAMPLED_SIZE(_w, _h) \
	(ALIGN(((_w) / 4), 16) * ALIGN(((_h) / 4), 32))

enum codec_std {
	W_HEVC_DEC	= 0x00,
	W_HEVC_ENC	= 0x01,
	W_AVC_DEC	= 0x02,
	W_AVC_ENC	= 0x03,
	STD_UNKNOWN	= 0xFF
};

#define HEVC_PROFILE_MAIN 1
#define HEVC_PROFILE_MAIN10 2
#define HEVC_PROFILE_STILLPICTURE 3
#define HEVC_PROFILE_MAIN10_STILLPICTURE 2

#define H264_PROFILE_BP 1
#define H264_PROFILE_MP 2
#define H264_PROFILE_EXTENDED 3
#define H264_PROFILE_HP 4
#define H264_PROFILE_HIGH10 5

#define H264_VUI_SAR_IDC_EXTENDED 255

#define DEC_REFRESH_TYPE_NON_IRAP 0
#define DEC_REFRESH_TYPE_IDR 2

#define DEFAULT_TEMP_LAYER_CNT 1
#define DEFAULT_RC_INITIAL_LEVEL 8
#define DEFAULT_RC_INITIAL_QP -1
#define DEFAULT_PIC_RC_MAX_DQP 3
#define DEFAULT_EN_ADAPTIVE_ROUND 1
#define DEFAULT_Q_ROUND_INTER 85
#define DEFAULT_Q_ROUND_INTRA 171
#define DEFAULT_EN_INTRA_TRANS_SKIP 1
#define DEFAULT_EN_ME_CENTER 1
#define DEFAULT_INTRA_4X4 3
#define DEFAULT_EN_AUTO_LEVEL_ADJUSTING 1
#define DEFAULT_NUM_TICKS_POC_DIFF 100
#define DEFAULT_RC_UPDATE_SPEED_CBR 64
#define DEFAULT_RC_UPDATE_SPEED_VBR 16
#define DEFAULT_VUI_VIDEO_SIGNAL_TYPE_PRESENT_FLAG 1
#define DEFAULT_VUI_COLOR_DESCRIPTION_PRESENT_FLAG 1

#define SEQ_CHANGE_ENABLE_PROFILE BIT(5)
#define SEQ_CHANGE_ENABLE_SIZE BIT(16)
#define SEQ_CHANGE_ENABLE_CONF_WIN_OFFSET BIT(17)
#define SEQ_CHANGE_ENABLE_BITDEPTH BIT(18)
#define SEQ_CHANGE_ENABLE_DPB_COUNT BIT(19)
#define SEQ_CHANGE_ENABLE_VIDEO_SIGNAL BIT(23)

#define SEQ_CHANGE_ENABLE_ALL_HEVC (SEQ_CHANGE_ENABLE_PROFILE | \
		SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_CONF_WIN_OFFSET | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT | \
		SEQ_CHANGE_ENABLE_VIDEO_SIGNAL)

#define SEQ_CHANGE_ENABLE_ALL_AVC (SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_CONF_WIN_OFFSET | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT | \
		SEQ_CHANGE_ENABLE_VIDEO_SIGNAL)

#define DEC_NOTI_FLAG_NO_FB 0x2
#define DEC_NOTI_FLAG_SEQ_CHANGE 0x1

#define RECON_IDX_FLAG_ENC_END -1
#define RECON_IDX_FLAG_ENC_DELAY -2
#define RECON_IDX_FLAG_HEADER_ONLY -3
#define RECON_IDX_FLAG_CHANGE_PARAM -4

enum codec_command {
	ENABLE_ROTATION,
	ENABLE_MIRRORING,
	SET_MIRROR_DIRECTION,
	SET_ROTATION_ANGLE,
	DEC_RESET_FRAMEBUF_INFO,
	DEC_GET_SEQ_INFO,
};

enum cb_cr_order {
	CBCR_ORDER_NORMAL,
	CBCR_ORDER_REVERSED
};

enum mirror_direction {
	MIRDIR_NONE,
	MIRDIR_VER,
	MIRDIR_HOR,
	MIRDIR_HOR_VER
};

enum chroma_format {
	YUV400,
	YUV420,
	YUV422,
	YUV444,
};

enum csc_format_order {
	CSC_FMT_ORDER_RGB	= 0,
	CSC_FMT_ORDER_RBG	= 1,
	CSC_FMT_ORDER_GRB	= 2,
	CSC_FMT_ORDER_GBR	= 3,
	CSC_FMT_ORDER_BGR	= 4,
	CSC_FMT_ORDER_BRG	= 5,

	CSC_FMT_ORDER_ARGB	= 0,
	CSC_FMT_ORDER_ARBG	= 1,
	CSC_FMT_ORDER_AGRB	= 2,
	CSC_FMT_ORDER_AGBR	= 3,
	CSC_FMT_ORDER_ABGR	= 4,
	CSC_FMT_ORDER_ABRG	= 5,
	CSC_FMT_ORDER_RGBA	= 8,
	CSC_FMT_ORDER_RBGA	= 9,
	CSC_FMT_ORDER_GRBA	= 10,
	CSC_FMT_ORDER_GBRA	= 11,
	CSC_FMT_ORDER_BGRA	= 12,
	CSC_FMT_ORDER_BRGA	= 13,
};

enum frame_buffer_format {
	FORMAT_ERR = -1,

	FORMAT_420 = 0,
	FORMAT_422,
	FORMAT_224,
	FORMAT_444,
	FORMAT_400,

	FORMAT_420_P10_16BIT_MSB = 5,
	FORMAT_420_P10_16BIT_LSB,
	FORMAT_420_P10_32BIT_MSB,
	FORMAT_420_P10_32BIT_LSB,

	FORMAT_422_P10_16BIT_MSB,
	FORMAT_422_P10_16BIT_LSB,
	FORMAT_422_P10_32BIT_MSB,
	FORMAT_422_P10_32BIT_LSB,

	FORMAT_444_P10_16BIT_MSB,
	FORMAT_444_P10_16BIT_LSB,
	FORMAT_444_P10_32BIT_MSB,
	FORMAT_444_P10_32BIT_LSB,

	FORMAT_400_P10_16BIT_MSB,
	FORMAT_400_P10_16BIT_LSB,
	FORMAT_400_P10_32BIT_MSB,
	FORMAT_400_P10_32BIT_LSB,

	FORMAT_YUYV,
	FORMAT_YUYV_P10_16BIT_MSB,
	FORMAT_YUYV_P10_16BIT_LSB,
	FORMAT_YUYV_P10_32BIT_MSB,
	FORMAT_YUYV_P10_32BIT_LSB,

	FORMAT_YVYU,
	FORMAT_YVYU_P10_16BIT_MSB,
	FORMAT_YVYU_P10_16BIT_LSB,
	FORMAT_YVYU_P10_32BIT_MSB,
	FORMAT_YVYU_P10_32BIT_LSB,

	FORMAT_UYVY,
	FORMAT_UYVY_P10_16BIT_MSB,
	FORMAT_UYVY_P10_16BIT_LSB,
	FORMAT_UYVY_P10_32BIT_MSB,
	FORMAT_UYVY_P10_32BIT_LSB,

	FORMAT_VYUY,
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

enum pic_type {
	PIC_TYPE_I = 0,
	PIC_TYPE_P = 1,
	PIC_TYPE_B = 2,
	PIC_TYPE_IDR = 5,
	PIC_TYPE_MAX
};

enum enc_force_pic_type {
	ENC_FORCE_PIC_TYPE_I = 0,
	ENC_FORCE_PIC_TYPE_P = 1,
	ENC_FORCE_PIC_TYPE_B = 2,
	ENC_FORCE_PIC_TYPE_IDR = 3,
	ENC_FORCE_PIC_TYPE_DISABLED = 4,
};

enum bitstream_mode {
	BS_MODE_INTERRUPT,
	BS_MODE_RESERVED,
	BS_MODE_PIC_END,
};

enum display_mode {
	DISP_MODE_DISP_ORDER,
	DISP_MODE_DEC_ORDER,
};

enum sw_reset_mode {
	SW_RESET_SAFETY,
	SW_RESET_FORCE,
	SW_RESET_ON_BOOT
};

enum tiled_map_type {
	LINEAR_FRAME_MAP = 0,
	COMPRESSED_FRAME_MAP = 17,
};

enum temporal_id_mode {
	TEMPORAL_ID_MODE_ABSOLUTE,
	TEMPORAL_ID_MODE_RELATIVE,
};

enum aux_buffer_type {
	AUX_BUF_FBC_Y_TBL,
	AUX_BUF_FBC_C_TBL,
	AUX_BUF_MV_COL,
	AUX_BUF_SUB_SAMPLE,
	AUX_BUF_TYPE_MAX,
};

enum intra_refresh_mode {
	INTRA_REFRESH_NONE = 0,
	INTRA_REFRESH_ROW = 1,
	INTRA_REFRESH_COLUMN = 2,
};

struct vpu_attr {
	u32 product_id;
	char product_name[8];
	u32 product_version;
	u32 fw_version;
	u32 fw_revision;
	u32 support_decoders;
	u32 support_encoders;
	u32 support_bitstream_mode;
	bool support_avc10bit_enc;
	bool support_hevc10bit_enc;
	bool support_dual_core;
};

struct frame_buffer {
	dma_addr_t buf_y;
	dma_addr_t buf_cb;
	dma_addr_t buf_cr;
	enum tiled_map_type map_type;
	unsigned int stride;
	unsigned int width;
	unsigned int height;
	int index;
	u32 luma_bitdepth: 4;
	u32 chroma_bitdepth: 4;
	u32 chroma_format_idc: 2;
};

struct vpu_rect {
	u32 left;
	u32 top;
	u32 right;
	u32 bottom;
};

struct sar_info {
	u32 enable;
	u32 idc;
	u32 width;
	u32 height;
};

struct aux_buffer {
	int index;
	int size;
	dma_addr_t addr;
};

struct aux_buffer_info {
	int num;
	struct aux_buffer *buf_array;
	enum aux_buffer_type type;
};

struct instance_buffer {
	dma_addr_t temp_base;
	u32 temp_size;
	dma_addr_t ar_base;
};

struct report_cycle {
	u32 host_cmd_s;
	u32 host_cmd_e;
	u32 proc_s;
	u32 proc_e;
	u32 vpu_s;
	u32 vpu_e;
	u32 frame_cycle;
	u32 proc_cycle;
	u32 vpu_cycle;
};

struct color_param {
	u32 chroma_sample_position;
	u32 color_range;
	u32 matrix_coefficients;
	u32 transfer_characteristics;
	u32 color_primaries;
	bool color_description_present;
	bool video_signal_type_present;
};

struct sec_axi_info {
	bool use_dec_ip;
	bool use_dec_lf_row;
	bool use_enc_rdo;
	bool use_enc_lf;
};

struct dec_aux_buffer_size_info {
	int width;
	int height;
	enum aux_buffer_type type;
};

struct dec_scaler_info {
	bool enable;
	int width;
	int height;
	u32 scale_mode;
};

struct dec_open_param {
	enum cb_cr_order cbcr_order;
	enum endian_mode frame_endian;
	enum endian_mode stream_endian;
	enum bitstream_mode bs_mode;
	enum display_mode disp_mode;
	bool enable_non_ref_fbc_write;
	u32 ext_addr_vcpu: 8;
	bool is_secure_inst;
	u32 inst_priority: 5;
	struct instance_buffer inst_buffer;
};

struct dec_initial_info {
	u32 pic_width;
	u32 pic_height;
	u32 f_rate_numerator;
	u32 f_rate_denominator;
	struct vpu_rect pic_crop_rect;
	u32 min_frame_buffer_count;
	u32 req_mv_buffer_count;
	u32 frame_buf_delay;
	u32 profile;
	u32 level;
	u32 tier;
	bool is_ext_sar;
	u32 aspect_rate_info;
	u32 bitrate;
	u32 chroma_format_idc;
	u32 luma_bitdepth;
	u32 chroma_bitdepth;
	u32 err_reason;
	int warn_info;
	u32 seq_change_info;
	dma_addr_t rd_ptr;
	dma_addr_t wr_ptr;
	unsigned int sequence_no;
	struct color_param color;
};

#define WAVE_SKIPMODE_WAVE_NONE 0
#define WAVE_SKIPMODE_NON_IRAP 1
#define WAVE_SKIPMODE_NON_REF 2

struct dec_param {
	int skipframe_mode;
	bool decode_cra_as_bla;
	bool disable_film_grain;
	u64 timestamp;
};

struct h265_rp_sei {
	unsigned int exist;
	int recovery_poc_cnt;
	bool exact_match;
	bool broken_link;
};

struct dec_output_info {
	int nal_type;
	int pic_type;
	int num_of_err_m_bs;
	int num_of_tot_m_bs;
	int num_of_err_m_bs_in_disp;
	int num_of_tot_m_bs_in_disp;
	int disp_pic_width;
	int disp_pic_height;
	int dec_pic_width;
	int dec_pic_height;
	int decoded_poc;
	int display_poc;
	struct h265_rp_sei h265_rp_sei;
	dma_addr_t rd_ptr;
	dma_addr_t wr_ptr;
	dma_addr_t byte_pos_frame_start;
	dma_addr_t byte_pos_frame_end;
	dma_addr_t frame_decoded_addr;
	dma_addr_t frame_display_addr;
	int error_reason;
	int warn_info;
	unsigned int sequence_no;
	struct report_cycle cycle;
	dma_addr_t release_disp_frame_addr[WAVE6_MAX_FBS];
	dma_addr_t disp_frame_addr[WAVE6_MAX_FBS];
	u64 timestamp;
	u32 notification_flags;
	u32 release_disp_frame_num: 5;
	u32 disp_frame_num: 5;
	u32 ctu_size: 2;
	bool frame_display;
	bool frame_decoded;
	bool stream_end;
	bool last_frame_in_au;
	bool decoding_success;
};

struct dec_info {
	struct dec_open_param open_param;
	struct dec_initial_info initial_info;
	dma_addr_t stream_wr_ptr;
	dma_addr_t stream_rd_ptr;
	bool stream_end;
	struct vpu_buf vb_mv[WAVE6_MAX_FBS];
	struct vpu_buf vb_fbc_y_tbl[WAVE6_MAX_FBS];
	struct vpu_buf vb_fbc_c_tbl[WAVE6_MAX_FBS];
	struct frame_buffer disp_buf[WAVE6_MAX_FBS];
	bool initial_info_obtained;
	struct sec_axi_info sec_axi_info;
	struct dec_output_info dec_out_info[WAVE6_MAX_FBS];
	int seq_change_mask;
	u32 cycle_per_tick;
	enum frame_buffer_format wtl_format;
};

#define MAX_CUSTOM_LAMBDA_NUM 52
#define MAX_NUM_TEMPORAL_LAYER 7
#define MAX_GOP_NUM 8
#define MAX_NUM_CHANGEABLE_TEMPORAL_LAYER 4

struct custom_gop_pic_param {
	int pic_type;
	int poc_offset;
	int pic_qp;
	int use_multi_ref_p;
	int ref_poc_l0;
	int ref_poc_l1;
	int temporal_id;
};

struct custom_gop_param {
	int custom_gop_size;
	struct custom_gop_pic_param pic_param[MAX_GOP_NUM];
};

struct temporal_layer_param {
	bool change_qp;
	u32 qp_i;
	u32 qp_p;
	u32 qp_b;
};

struct enc_aux_buffer_size_info {
	int width;
	int height;
	enum aux_buffer_type type;
};

struct enc_scaler_info {
	bool enable;
	int width;
	int height;
	int coef_mode;
};

struct enc_codec_param {
	u32 internal_bit_depth;
	u32 decoding_refresh_type;
	u32 idr_period;
	u32 intra_period;
	u32 gop_preset_idx;
	u32 frame_rate;
	u32 bitrate;
	u32 cpb_size;
	u32 hvs_qp_scale_div2;
	u32 max_delta_qp;
	int rc_initial_qp;
	u32 rc_update_speed;
	u32 max_bitrate;
	u32 rc_mode;
	u32 rc_initial_level;
	u32 pic_rc_max_dqp;
	u32 bg_th_diff;
	u32 bg_th_mean_diff;
	int bg_delta_qp;
	u32 intra_refresh_mode;
	u32 intra_refresh_arg;
	int beta_offset_div2;
	int tc_offset_div2;
	u32 qp;
	u32 min_qp_i;
	u32 max_qp_i;
	u32 min_qp_p;
	u32 max_qp_p;
	u32 min_qp_b;
	u32 max_qp_b;
	int cb_qp_offset;
	int cr_qp_offset;
	u32 q_round_intra;
	u32 q_round_inter;
	int lambda_dqp_intra;
	int lambda_dqp_inter;
	u32 slice_mode;
	u32 slice_arg;
	u32 level;
	u32 tier;
	u32 profile;
	struct vpu_rect conf_win;
	u32 forced_idr_header;
	u16 custom_lambda_ssd[MAX_CUSTOM_LAMBDA_NUM];
	u16 custom_lambda_sad[MAX_CUSTOM_LAMBDA_NUM];
	struct custom_gop_param gop_param;
	struct temporal_layer_param temp_layer[MAX_NUM_CHANGEABLE_TEMPORAL_LAYER];
	u32 temp_layer_cnt;
	u32 report_mv_histo_threshold0;
	u32 report_mv_histo_threshold1;
	u32 report_mv_histo_threshold2;
	u32 report_mv_histo_threshold3;
	enum endian_mode custom_map_endian;
	u32 num_units_in_tick;
	u32 time_scale;
	u32 num_ticks_poc_diff_one;
	struct color_param color;
	struct sar_info sar;
	u32 max_intra_pic_bit;
	u32 max_inter_pic_bit;
	u32 intra_4x4;

	u32 en_constrained_intra_pred: 1;
	u32 en_long_term: 1;
	u32 en_intra_trans_skip: 1;
	u32 en_me_center: 1;
	u32 en_rate_control: 1;
	u32 en_transform8x8: 1;
	u32 en_hvs_qp: 1;
	u32 en_bg_detect: 1;
	u32 en_temporal_mvp: 1;
	u32 en_cabac: 1;
	u32 en_dbk: 1;
	u32 en_sao: 1;
	u32 en_lf_cross_slice_boundary: 1;
	u32 en_scaling_list: 1;
	u32 en_adaptive_round: 1;
	u32 en_qp_map: 1;
	u32 en_mode_map: 1;
	u32 en_q_round_offset: 1;
	u32 en_still_picture: 1;
	u32 en_strong_intra_smoothing: 1;
	u32 en_custom_lambda: 1;
	u32 en_report_mv_histo: 1;
	u32 dis_coef_clear: 1;
	u32 en_cu_level_rate_control: 1;
	u32 en_vbv_overflow_drop_frame: 1;
	u32 en_auto_level_adjusting: 1;
};

struct enc_open_param {
	int pic_width;
	int pic_height;
	struct enc_codec_param codec_param;
	enum cb_cr_order cbcr_order;
	enum endian_mode stream_endian;
	enum endian_mode source_endian;
	bool line_buf_int_en;
	enum frame_buffer_format src_format;
	enum frame_buffer_format output_format;
	bool enable_non_ref_fbc_write;
	bool enc_hrd_rbsp_in_vps;
	u32 hrd_rbsp_data_size;
	dma_addr_t hrd_rbsp_data_addr;
	u32 ext_addr_vcpu: 8;
	bool is_secure_inst;
	u32 inst_priority: 5;
	struct instance_buffer inst_buffer;
	bool enc_aud;
};

struct enc_initial_info {
	u32 min_frame_buffer_count;
	u32 min_src_frame_count;
	u32 req_mv_buffer_count;
	int max_latency_pictures;
	int err_reason;
	int warn_info;
};

struct enc_csc_param {
	u32 format_order;
	s32 coef_ry;
	s32 coef_gy;
	s32 coef_by;
	s32 coef_rcb;
	s32 coef_gcb;
	s32 coef_bcb;
	s32 coef_rcr;
	s32 coef_gcr;
	s32 coef_bcr;
	u32 offset_y;
	u32 offset_cb;
	u32 offset_cr;
};

union wave6_enc_custom_map_option {
	struct {
		u32 custom_roi_map_enable:1;
		u32 use_ctu_force_mode:1;
		u32 reserved:30;
	} field;
	u32 data;
};

struct enc_param {
	struct frame_buffer *source_frame;
	bool skip_picture;
	dma_addr_t pic_stream_buffer_addr;
	int pic_stream_buffer_size;
	bool force_pic_qp_enable;
	int force_pic_qp_i;
	int force_pic_qp_p;
	int force_pic_qp_b;
	bool force_pic_type_enable;
	int force_pic_type;
	int src_idx;
	bool src_end;
	u32 bitrate;
	struct enc_csc_param csc;
	u64 timestamp;
	union wave6_enc_custom_map_option custom_map_opt;
	dma_addr_t custom_map_addr;
};

struct enc_report_fme_sum {
	u32 lower_x0;
	u32 higher_x0;
	u32 lower_y0;
	u32 higher_y0;
	u32 lower_x1;
	u32 higher_x1;
	u32 lower_y1;
	u32 higher_y1;
};

struct enc_report_mv_histo {
	u32 cnt0;
	u32 cnt1;
	u32 cnt2;
	u32 cnt3;
	u32 cnt4;
};

struct enc_output_info {
	dma_addr_t bitstream_buffer;
	u32 bitstream_size;
	int bitstream_wrap_around;
	int pic_type;
	int num_of_slices;
	int recon_frame_index;
	struct frame_buffer recon_frame;
	dma_addr_t rd_ptr;
	dma_addr_t wr_ptr;
	int pic_skipped;
	int num_of_intra;
	int num_of_merge;
	int num_of_skip_block;
	int avg_ctu_qp;
	int enc_pic_byte;
	int enc_gop_pic_idx;
	int enc_pic_poc;
	int enc_src_idx;
	int enc_vcl_nut;
	int enc_pic_cnt;
	int error_reason;
	int warn_info;
	u32 pic_distortion_low;
	u32 pic_distortion_high;
	bool non_ref_pic;
	bool encoding_success;
	struct enc_report_fme_sum fme_sum;
	struct enc_report_mv_histo mv_histo;
	struct report_cycle cycle;
	u64 timestamp;
	dma_addr_t src_y_addr;
	dma_addr_t custom_map_addr;
	dma_addr_t prefix_sei_nal_addr;
	dma_addr_t suffix_sei_nal_addr;
};

enum gop_preset_idx {
	PRESET_IDX_CUSTOM_GOP = 0,
	PRESET_IDX_ALL_I = 1,
	PRESET_IDX_IPP = 2,
	PRESET_IDX_IBBB = 3,
	PRESET_IDX_IBPBP = 4,
	PRESET_IDX_IBBBP = 5,
	PRESET_IDX_IPPPP = 6,
	PRESET_IDX_IBBBB = 7,
	PRESET_IDX_RA_IB = 8,
	PRESET_IDX_IPP_SINGLE = 9,
	PRESET_IDX_MAX,
};

struct enc_info {
	struct enc_open_param open_param;
	struct enc_initial_info initial_info;
	int num_frame_buffers;
	int stride;
	bool rotation_enable;
	bool mirror_enable;
	enum mirror_direction mirror_direction;
	int rotation_angle;
	bool initial_info_obtained;
	struct sec_axi_info sec_axi_info;
	bool line_buf_int_en;
	struct vpu_buf vb_mv[WAVE6_MAX_FBS];
	struct vpu_buf vb_fbc_y_tbl[WAVE6_MAX_FBS];
	struct vpu_buf vb_fbc_c_tbl[WAVE6_MAX_FBS];
	struct vpu_buf vb_sub_sam_buf[WAVE6_MAX_FBS];
	u32 cycle_per_tick;
	u32 width;
	u32 height;
	struct enc_scaler_info scaler_info;
	int color_format;
};

struct h264_enc_controls {
	u32 profile;
	u32 level;
	u32 min_qp;
	u32 max_qp;
	u32 i_frame_qp;
	u32 p_frame_qp;
	u32 b_frame_qp;
	u32 loop_filter_mode;
	u32 loop_filter_beta;
	u32 loop_filter_alpha;
	u32 _8x8_transform;
	u32 constrained_intra_prediction;
	u32 chroma_qp_index_offset;
	u32 entropy_mode;
	u32 i_period;
	u32 vui_sar_enable;
	u32 vui_sar_idc;
	u32 vui_ext_sar_width;
	u32 vui_ext_sar_height;
	u32 cpb_size;
};

struct hevc_enc_controls {
	u32 profile;
	u32 level;
	u32 min_qp;
	u32 max_qp;
	u32 i_frame_qp;
	u32 p_frame_qp;
	u32 b_frame_qp;
	u32 loop_filter_mode;
	u32 lf_beta_offset_div2;
	u32 lf_tc_offset_div2;
	u32 refresh_type;
	u32 refresh_period;
	u32 const_intra_pred;
	u32 strong_smoothing;
	u32 tmv_prediction;
};

struct enc_controls {
	u32 rot_angle;
	u32 mirror_direction;
	u32 bitrate;
	u32 bitrate_mode;
	u32 gop_size;
	u32 frame_rc_enable;
	u32 mb_rc_enable;
	u32 slice_mode;
	u32 slice_max_mb;
	u32 prepend_spspps_to_idr;
	u32 intra_refresh_period;
	struct h264_enc_controls h264;
	struct hevc_enc_controls hevc;
	u32 force_key_frame;
	u32 frame_skip_mode;
};

struct vpu_irq {
	u32 status;
	u32 inst_idc;
};

struct vpu_device {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct list_head instances;
	spinlock_t inst_lock; /* lock instance list */
	struct video_device *video_dev_dec;
	struct video_device *video_dev_enc;
	struct mutex hw_lock; /* lock hw configurations */
	int irq;
	u32 fw_version;
	u32 fw_revision;
	u32 hw_version;
	struct vpu_attr	attr;
	u32 last_performance_cycles;
	void __iomem *reg_base;
	struct device *ctrl;
	int product_code;
	struct vpu_buf temp_vbuf;
	struct clk_bulk_data *clks;
	int num_clks;
	struct clk *clk_vpu;
	struct kfifo irq_fifo;
	struct delayed_work task_timer;
	struct wave6_vpu_entity entity;
	bool active;
	const struct wave6_match_data *res;
	struct dentry *debugfs;
	struct imx_mur_node *recorder;

	bool force_dma_sync;
	struct device *trusty_dev;
};

struct vpu_instance;

struct vpu_instance_ops {
	int (*start_process)(struct vpu_instance *inst);
	void (*finish_process)(struct vpu_instance *inst, bool error);
};

struct vpu_performance_info {
	ktime_t ts_start;
	ktime_t ts_first;
	ktime_t ts_last;
	s64 latency_first;
	s64 latency_max;
	s64 min_process_time;
	s64 max_process_time;
	u64 total_sw_time;
	u64 total_hw_time;
};

struct vpu_roi_map_info {
	struct v4l2_area ctu;		/*ctu size in pixels*/
	u32 num_ctu_col;
	u32 num_ctu_row;
	u32 num_ctu;

	struct v4l2_area group;		/*group size in ctu*/
	u32 num_group_col;
	u32 num_group_row;
	u32 custom_map_size;
};

struct vpu_instance {
	struct list_head list;
	struct v4l2_fh v4l2_fh;
	struct v4l2_ctrl_handler v4l2_ctrl_hdl;
	struct vpu_device *dev;
	struct mutex queue_lock; /* the lock for the src,dst v4l2 queues */
	struct completion irq_done;
	struct v4l2_pix_format_mplane src_fmt;
	struct v4l2_pix_format_mplane dst_fmt;
	struct v4l2_rect crop;
	struct v4l2_rect codec_rect;
	enum v4l2_colorspace colorspace;
	enum v4l2_xfer_func xfer_func;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;

	enum vpu_instance_state state;
	enum vpu_instance_state state_in_seek;
	enum vpu_instance_type type;
	const struct vpu_instance_ops *ops;

	enum codec_std std;
	u32 id;
	union {
		struct enc_info enc_info;
		struct dec_info dec_info;
	} *codec_info;
	struct mutex fbc_lock; /* the lock for internal buffers */
	struct frame_buffer frame_buf[WAVE6_MAX_FBS];
	struct vpu_buf frame_y_vbuf[WAVE6_MAX_FBS];
	struct vpu_buf frame_c_vbuf[WAVE6_MAX_FBS];
	u32 allocated_fb_num;
	u32 registered_fb_num;
	u32 queued_src_buf_num;
	u32 queued_dst_buf_num;
	u32 processed_buf_num;
	u32 error_buf_num;
	u32 sequence;
	bool cbcr_interleave;
	bool nv21;
	bool eos;

	struct vpu_buf aux_vbuf[AUX_BUF_TYPE_MAX][WAVE6_MAX_FBS];
	struct vpu_buf ar_vbuf;
	bool secure_mode;
	enum display_mode disp_mode;

	unsigned int frame_rate;
	struct enc_controls enc_ctrls;
	struct dec_scaler_info scaler_info;
	bool error_recovery;

	struct vpu_performance_info performance;

	struct dentry *debugfs;

	int roi_mode;
	struct vpu_buf custom_qp_map;
	struct vpu_roi_map_info roi_info;

	struct imx_mur_node *recorder;
};

void wave6_vdi_writel(struct vpu_device *vpu_device, unsigned int addr, unsigned int data);
unsigned int wave6_vdi_readl(struct vpu_device *vpu_dev, unsigned int addr);
unsigned int wave6_vdi_convert_endian(unsigned int endian);
int wave6_allocate_secure_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb);
void wave6_free_secure_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb);

int wave6_vpu_dec_open(struct vpu_instance *inst, struct dec_open_param *pop);
int wave6_vpu_dec_close(struct vpu_instance *inst, u32 *fail_res);
int wave6_vpu_dec_issue_seq_init(struct vpu_instance *inst);
int wave6_vpu_dec_complete_seq_init(struct vpu_instance *inst, struct dec_initial_info *info);
int wave6_vpu_dec_get_aux_buffer_size(struct vpu_instance *inst,
				      struct dec_aux_buffer_size_info info,
				      uint32_t *size);
int wave6_vpu_dec_register_aux_buffer(struct vpu_instance *inst, struct aux_buffer_info info);
int wave6_vpu_dec_register_frame_buffer_ex(struct vpu_instance *inst,
					   struct frame_buffer fb);
int wave6_vpu_dec_register_display_buffer_ex(struct vpu_instance *inst,
					     struct frame_buffer fb);
int wave6_vpu_dec_start_one_frame(struct vpu_instance *inst, struct dec_param *param,
				  u32 *res_fail);
int wave6_vpu_dec_get_output_info(struct vpu_instance *inst, struct dec_output_info *info);
void wave6_vpu_dec_set_rd_ptr(struct vpu_instance *inst, dma_addr_t addr, bool update_wr_ptr);
int wave6_vpu_dec_give_command(struct vpu_instance *inst, enum codec_command cmd, void *parameter);
void wave6_vpu_dec_get_bitstream_buffer(struct vpu_instance *inst, dma_addr_t *p_rd_ptr,
					dma_addr_t *p_wr_ptr);
int wave6_vpu_dec_update_bitstream_buffer(struct vpu_instance *inst, int size);
int wave6_vpu_dec_flush_instance(struct vpu_instance *inst);

int wave6_vpu_enc_open(struct vpu_instance *inst, struct enc_open_param *enc_op_param);
int wave6_vpu_enc_close(struct vpu_instance *inst, u32 *fail_res);
int wave6_vpu_enc_issue_seq_init(struct vpu_instance *inst);
int wave6_vpu_enc_issue_seq_change(struct vpu_instance *inst, bool *changed);
int wave6_vpu_enc_complete_seq_init(struct vpu_instance *inst, struct enc_initial_info *info);
int wave6_vpu_enc_get_aux_buffer_size(struct vpu_instance *inst,
				      struct enc_aux_buffer_size_info info,
				      uint32_t *size);
int wave6_vpu_enc_register_aux_buffer(struct vpu_instance *inst, struct aux_buffer_info info);
int wave6_vpu_enc_register_frame_buffer_ex(struct vpu_instance *inst, int num, unsigned int stride,
					   int height, enum tiled_map_type map_type);
int wave6_vpu_enc_start_one_frame(struct vpu_instance *inst, struct enc_param *param,
				  u32 *fail_res);
int wave6_vpu_enc_get_output_info(struct vpu_instance *inst, struct enc_output_info *info);
int wave6_vpu_enc_give_command(struct vpu_instance *inst, enum codec_command cmd, void *parameter);
const char *wave6_vpu_get_aux_name(enum aux_buffer_type type);

#endif /* __WAVE6_VPUAPI_H__ */
