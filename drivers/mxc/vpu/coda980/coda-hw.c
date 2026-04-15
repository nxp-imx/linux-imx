// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Coda series multi-standard codec IP - coda backend logic
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#include <linux/iopoll.h>
#include "coda-vpu.h"
#include "coda-hw.h"
#include "coda-regdefine.h"

#define VPU_BUSY_CHECK_TIMEOUT 10000000

#define VPU_FRAME_ENDIAN VDI_LITTLE_ENDIAN
#define VPU_STREAM_ENDIAN VDI_LITTLE_ENDIAN
#define VPU_SOURCE_ENDIAN VDI_LITTLE_ENDIAN

#define ENABLE_ZERO_PADDING BIT(31)
#define ENABLE_BUF_PIC_RESET BIT(4)
#define ENABLE_DYNAMIC_BUF_ALLOC BIT(5)
#define ENABLE_CBCR_INTERLEAVE BIT(2)
#define ENABLE_BWB BIT(15)
#define ENABLE_RC_QP_MIN BIT(14)
#define ENABLE_RC_QP_MAX BIT(6)
#define ENABLE_VUI_PARAMETER_PRESENT_FLAG BIT(0)

#define REPORT_ENCODING_INFO 0
#define ME_LINEBUFFER_MODE 0
#define LONG_BURST_MODE 1
#define DMA_REQUEST_MODE 2

#define MAX_AVC_LEVEL_LIST 16

#define GEN_XY2AXI(INV, ZER, TBX, XY, BIT)	\
	((INV) << 7 | (ZER) << 6 | (TBX) << 5 | (XY) << 4 | (BIT))
#define GEN_CONFIG_UPPER(A, B, C, D, E)	\
	((A) << 20 | (B) << 19 | (C) << 18 | (D) << 17 | (E) << 16)
#define GEN_CONFIG_LOWER(F, G, H, I)	\
	((F) << 12 | (G) << 8 | (H) << 4 | (I))
#define GEN_CONFIG(A, B, C, D, E, F, G, H, I)	\
	(GEN_CONFIG_UPPER(A, B, C, D, E) | GEN_CONFIG_LOWER(F, G, H, I))

#define X_SEL 0
#define Y_SEL 1

struct avc_level {
	u32 level_idc;
	u32 max_mbps;
	u32 max_fs;
	u32 max_br;
};

static const struct avc_level avc_level_list[MAX_AVC_LEVEL_LIST] = {
	{10,     1485,     99,     64},
	{ 9,     1485,     99,    128},
	{11,     3000,    396,    192},
	{12,     6000,    396,    384},
	{13,    11880,    396,    768},
	{20,    11880,    396,   2000},
	{21,    19800,    792,   4000},
	{22,    20250,   1620,   4000},
	{30,    40500,   1620,  10000},
	{31,   108000,   3600,  14000},
	{32,   216000,   5120,  20000},
	{40,   245760,   8192,  20000},
	{41,   245760,   8192,  50000},
	{42,   522240,   8704,  50000},
	{50,   589824,  22080, 135000},
	{51,   983040,  36864, 240000}
};

static u32 coda_hw_enc_calc_level(struct coda_enc_open_param *param)
{
	u32 mb_w, mb_h;
	u32 mbps, fs, br;
	u32 level;
	u32 pic_width, pic_height;

	pic_width = param->pic_width;
	pic_height = param->pic_height;
	if (param->rot_ang == ROT_ANG_90 || param->rot_ang == ROT_ANG_270) {
		pic_width = param->pic_height;
		pic_height = param->pic_width;
	}

	if (param->en_field_encoding) {
		mb_w = pic_width >> 4;
		mb_h = (pic_height + 31) >> 5;
		fs = mb_w * mb_h * 2;
		mb_h = mb_h * 2;
	} else {
		mb_w = pic_width >> 4;
		mb_h = pic_height >> 4;
		fs = mb_w * mb_h;
	}

	mbps = fs * param->framerate;
	br = param->bitrate;

	for (level = 0; level < MAX_AVC_LEVEL_LIST; level++) {
		const struct avc_level *l = &avc_level_list[level];

		if (mbps <= l->max_mbps &&
		    fs <= l->max_fs &&
		    br <= l->max_br &&
		    ((mb_w * mb_w) <= (l->max_fs * 8)) &&
		    ((mb_h * mb_h) <= (l->max_fs * 8)))
			break;
	}

	if (level == MAX_AVC_LEVEL_LIST)
		level = MAX_AVC_LEVEL_LIST - 1;

	return level;
}

static u32 coda_hw_enc_calc_rot_mir_mode(struct coda_enc_open_param *param)
{
	u32 rot_mir_mode = 0;

	switch (param->rot_ang) {
	case ROT_ANG_0:
		rot_mir_mode |= 0x0;
		break;
	case ROT_ANG_90:
		rot_mir_mode |= 0x1;
		break;
	case ROT_ANG_180:
		rot_mir_mode |= 0x2;
		break;
	case ROT_ANG_270:
		rot_mir_mode |= 0x3;
		break;
	}
	switch (param->mir_dir) {
	case MIR_DIR_NONE:
		rot_mir_mode |= 0x0;
		break;
	case MIR_DIR_VER:
		rot_mir_mode |= 0x4;
		break;
	case MIR_DIR_HOR:
		rot_mir_mode |= 0x8;
		break;
	case MIR_DIR_HOR_VER:
		rot_mir_mode |= 0xC;
		break;
	}

	return rot_mir_mode;
}

static struct coda_rect coda_hw_enc_calc_conf_win(struct coda_enc_open_param *param)
{
	struct coda_rect conf_win;
	u32 conf_right, conf_left, conf_top, conf_bot;
	u32 pad_right, pad_bot;
	u32 rot_mir_mode = 0;

	pad_right = ALIGN(param->pic_width, 16) - param->pic_width;
	pad_bot = ALIGN(param->pic_height, 16) - param->pic_height;

	if (param->conf_win.right > 0)
		conf_right = param->conf_win.right + pad_right;
	else
		conf_right = pad_right;

	if (param->conf_win.bottom > 0)
		conf_bot = param->conf_win.bottom + pad_bot;
	else
		conf_bot = pad_bot;

	conf_top = param->conf_win.top;
	conf_left = param->conf_win.left;

	conf_win.top = conf_top;
	conf_win.left = conf_left;
	conf_win.bottom = conf_bot;
	conf_win.right = conf_right;

	rot_mir_mode = coda_hw_enc_calc_rot_mir_mode(param);

	if (rot_mir_mode == 1 || rot_mir_mode == 15) {
		conf_win.top = conf_right;
		conf_win.left = conf_top;
		conf_win.bottom = conf_left;
		conf_win.right = conf_bot;
	} else if (rot_mir_mode == 2 || rot_mir_mode == 12) {
		conf_win.top = conf_bot;
		conf_win.left = conf_right;
		conf_win.bottom = conf_top;
		conf_win.right = conf_left;
	} else if (rot_mir_mode == 3 || rot_mir_mode == 13) {
		conf_win.top = conf_left;
		conf_win.left = conf_bot;
		conf_win.bottom = conf_right;
		conf_win.right = conf_top;
	} else if (rot_mir_mode == 4 || rot_mir_mode == 10) {
		conf_win.top = conf_bot;
		conf_win.bottom = conf_top;
	} else if (rot_mir_mode == 8 || rot_mir_mode == 6) {
		conf_win.left = conf_right;
		conf_win.right = conf_left;
	} else if (rot_mir_mode == 5 || rot_mir_mode == 11) {
		conf_win.top = conf_left;
		conf_win.left = conf_top;
		conf_win.bottom = conf_right;
		conf_win.right = conf_bot;
	} else if (rot_mir_mode == 7 || rot_mir_mode == 9) {
		conf_win.top = conf_right;
		conf_win.left = conf_bot;
		conf_win.bottom = conf_left;
		conf_win.right = conf_top;
	}

	return conf_win;
}

static int coda_hw_wait_vpu_busy(struct vpu_device *vpu, unsigned int addr)
{
	u32 data;

	return read_poll_timeout(coda_vdi_readl, data, data == 0,
				 0, VPU_BUSY_CHECK_TIMEOUT, false, vpu->dev, addr);
}

static int coda_hw_wait_bus_busy(struct vpu_device *vpu, unsigned int addr)
{
	u32 data;

	return read_poll_timeout(coda_vdi_readl, data, data == 0x77,
				 0, VPU_BUSY_CHECK_TIMEOUT, false, vpu->dev, addr);
}

static u32 coda_hw_get_product_id(struct vpu_device *vpu)
{
	u32 product_id = PRODUCT_ID_NONE;
	u32 reg_val;

	reg_val = vpu_read_reg(vpu->dev, VPU_PRODUCT_CODE);

	switch (reg_val) {
	case CODA980_CODE:
		product_id = PRODUCT_ID_980;
		break;
	default:
		dev_err(vpu->dev, "Invalid product id (%x)\n", reg_val);
		break;
	}

	return product_id;
}

bool coda_hw_is_init(struct vpu_device *vpu)
{
	return vpu_read_reg(vpu->dev, BIT_CUR_PC) != 0;
}

int coda_hw_reset(struct vpu_device *vpu, enum coda_sw_reset_mode reset_mode)
{
	u32 cmd;
	int ret;

	if (reset_mode != SW_RESET_ON_BOOT) {
		cmd = vpu_read_reg(vpu->dev, BIT_RUN_COMMAND);
		if (cmd == ENC_SEQ_INIT || cmd == PIC_RUN) {
			if (vpu_read_reg(vpu->dev, BIT_BUSY_FLAG) ||
			    vpu_read_reg(vpu->dev, BIT_INT_REASON)) {
				// stop all of pipeline
				vpu_write_reg(vpu->dev, MBC_SET_SUBBLK_EN, ((1 << 20) | 0));

				// force to set the end of Bitstream to be decoded.
				cmd = vpu_read_reg(vpu->dev, BIT_BIT_STREAM_PARAM);
				cmd |= 1 << 2;
				vpu_write_reg(vpu->dev, BIT_BIT_STREAM_PARAM, cmd);

				cmd = vpu_read_reg(vpu->dev, BIT_RD_PTR);
				vpu_write_reg(vpu->dev, BIT_WR_PTR, cmd);

				ret = coda_hw_wait_vpu_busy(vpu, BIT_INT_REASON);
				if (ret)
					return -ETIMEDOUT;

				// clear HW signal
				vpu_write_reg(vpu->dev, BIT_INT_REASON, 0);
				vpu_write_reg(vpu->dev, BIT_INT_CLEAR, 1);
			}
		}
	}

	// Waiting for completion of BWB transaction first
	ret = coda_hw_wait_vpu_busy(vpu, GDI_BWB_STATUS);
	if (ret)
		return -ETIMEDOUT;

	// Waiting for completion of bus transaction
	// Step1 : No more request
	// no more request {3'b0,no_more_req_sec,3'b0,no_more_req}
	vpu_write_reg(vpu->dev, GDI_BUS_CTRL, 0x11);
	ret = coda_hw_wait_bus_busy(vpu, GDI_BUS_STATUS);
	if (ret) {
		vpu_write_reg(vpu->dev, GDI_BUS_CTRL, 0x00);
		return -ETIMEDOUT;
	}

	cmd = 0;
	// Software Reset Trigger
	if (reset_mode != SW_RESET_ON_BOOT)
		cmd =  VPU_SW_RESET_BPU_CORE | VPU_SW_RESET_BPU_BUS;

	cmd |= VPU_SW_RESET_VCE_CORE | VPU_SW_RESET_VCE_BUS;
	// If you reset GDI, tiled map should be reconfigured
	if (reset_mode == SW_RESET_ON_BOOT)
		cmd |= VPU_SW_RESET_GDI_CORE | VPU_SW_RESET_GDI_BUS;

	vpu_write_reg(vpu->dev, BIT_SW_RESET, cmd);

	// wait until reset is done
	if (coda_hw_wait_vpu_busy(vpu, BIT_SW_RESET_STATUS) != 0) {
		vpu_write_reg(vpu->dev, BIT_SW_RESET, 0x00);
		vpu_write_reg(vpu->dev, GDI_BUS_CTRL, 0x00);
		return -ETIMEDOUT;
	}

	vpu_write_reg(vpu->dev, BIT_SW_RESET, 0);

	// Step3 : must clear GDI_BUS_CTRL after done SW_RESET
	vpu_write_reg(vpu->dev, GDI_BUS_CTRL, 0x00);

	return 0;
}

static void coda_hw_load_bit_code(struct vpu_device *vpu)
{
	u8 *code;
	int i, pos;
	u32 val;

	if (!vpu->common_mem.vaddr)
		return;

	code = (u8 *)vpu->common_mem.vaddr;

	vpu_write_reg(vpu->dev, BIT_CODE_RUN, 0);

	for (i = 0; i < (CODA_SDRAM_SIZE / 2); i++) {
		pos = ((i & ~3) << 1) + 7 - 2 * (i & 3);
		val = (i << 16) | (code[pos] << 8) | code[pos - 1];

		vpu_write_reg(vpu->dev, BIT_CODE_DOWN, val);
	}

	dev_err(vpu->dev, "completed to download bit code : %d\n", i);
}

static void coda_hw_bit_issue_cmd(struct vpu_device *vpu, struct vpu_instance *inst, u32 cmd)
{
	int inst_idx = 0;
	int cdc_mode = 0;
	int aux_mode = 0;

	if (inst) {
		inst_idx = inst->id;
		cdc_mode = inst->std;

		vpu_write_reg(vpu->dev, BIT_WORK_BUF_ADDR, inst->work_vbuf.daddr);
	}

	vpu_write_reg(vpu->dev, BIT_BUSY_FLAG, 1);
	vpu_write_reg(vpu->dev, BIT_RUN_INDEX, inst_idx);
	vpu_write_reg(vpu->dev, BIT_RUN_COD_STD, cdc_mode);
	vpu_write_reg(vpu->dev, BIT_RUN_AUX_STD, aux_mode);
	vpu_write_reg(vpu->dev, BIT_RUN_COMMAND, cmd);
}

int coda_hw_init(struct vpu_device *vpu, u8 *code, size_t size)
{
	dma_addr_t temp_buf;
	dma_addr_t para_buf;
	dma_addr_t code_buf;
	int ret;
	u32 data;

	code_buf = vpu->common_mem.daddr;
	temp_buf = code_buf + CODA_CODE_BUF_SIZE;
	para_buf = temp_buf + CODA_TEMP_BUF_SIZE;

	ret = coda_vdi_write_memory(&vpu->common_mem, 0, code, size, VDI_LITTLE_ENDIAN);
	if (ret <= 0)  {
		dev_err(vpu->dev, "failed to copy code buf (%x)\n", ret);
		return ret;
	}

	coda_hw_load_bit_code(vpu);

	vpu_write_reg(vpu->dev, BIT_PARA_BUF_ADDR, para_buf);
	vpu_write_reg(vpu->dev, BIT_CODE_BUF_ADDR, code_buf);
	vpu_write_reg(vpu->dev, BIT_TEMP_BUF_ADDR, temp_buf);

	vpu_write_reg(vpu->dev, BIT_BIT_STREAM_CTRL, VPU_STREAM_ENDIAN);
	vpu_write_reg(vpu->dev, BIT_FRAME_MEM_CTRL, ENABLE_CBCR_INTERLEAVE | VPU_FRAME_ENDIAN);

	vpu_write_reg(vpu->dev, BIT_BIT_STREAM_PARAM, 0);

	vpu_write_reg(vpu->dev, BIT_AXI_SRAM_USE, 0);
	vpu_write_reg(vpu->dev, BIT_INT_ENABLE, 0);
	vpu_write_reg(vpu->dev, BIT_ROLLBACK_STATUS, 0);

	data = BIT(INT_BIT_BIT_BUF_FULL);
	data |= BIT(INT_BIT_SEQ_INIT);
	data |= BIT(INT_BIT_PIC_RUN);
	vpu_write_reg(vpu->dev, BIT_INT_ENABLE, data);
	vpu_write_reg(vpu->dev, BIT_INT_CLEAR, 0x1);
	vpu_write_reg(vpu->dev, BIT_BUSY_FLAG, 0x1);
	vpu_write_reg(vpu->dev, BIT_CODE_RESET, 1);
	vpu_write_reg(vpu->dev, BIT_CODE_RUN, 1);

	ret = coda_hw_wait_vpu_busy(vpu, BIT_BUSY_FLAG);
	if (ret) {
		dev_err(vpu->dev, "timeout for checking  BIT_BUSY_FLAG (%x)\n", ret);
		return -ETIMEDOUT;
	}

	return 0;
}

int coda_hw_sleep_wake(struct vpu_device *vpu, bool sleep)
{
	int i;
	int ret;

	if (sleep) {
		ret = coda_hw_wait_vpu_busy(vpu, BIT_BUSY_FLAG);
		if (ret) {
			dev_err(vpu->dev, "fail to wait vpu idle in sleep\n");
			return -ETIMEDOUT;
		}
		for (i = 0; i < 64; i++)
			vpu->reg_bk[i] = vpu_read_reg(vpu->dev,
						      (BIT_BASE + 0x100 + (i * 4)));
	} else {
		coda_hw_load_bit_code(vpu);

		for (i = 0; i < 64; i++)
			vpu_write_reg(vpu->dev,
				      (BIT_BASE + 0x100 + (i * 4)), vpu->reg_bk[i]);

		vpu_write_reg(vpu->dev, BIT_BUSY_FLAG, 1);
		vpu_write_reg(vpu->dev, BIT_CODE_RESET, 1);
		vpu_write_reg(vpu->dev, BIT_CODE_RUN, 1);

		ret = coda_hw_wait_vpu_busy(vpu, BIT_BUSY_FLAG);
		if (ret) {
			dev_err(vpu->dev, "fail to load bit code for wake\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

int coda_hw_get_version(struct vpu_device *vpu, u32 *version, u32 *revision)
{
	int ret;

	vpu_write_reg(vpu->dev, RET_FW_VER_NUM, 0);

	coda_hw_bit_issue_cmd(vpu, NULL, FIRMWARE_GET);
	ret = coda_hw_wait_vpu_busy(vpu, BIT_BUSY_FLAG);
	if (ret)
		return -ETIMEDOUT;

	vpu->product_code = vpu_read_reg(vpu->dev, VPU_PRODUCT_CODE);
	vpu->product_id = coda_hw_get_product_id(vpu);

	if (version)
		*version = vpu_read_reg(vpu->dev, RET_FW_VER_NUM);

	if (revision)
		*revision = vpu_read_reg(vpu->dev, RET_FW_CODE_REV);

	return 0;
}

int coda_hw_build_up_enc_param(struct vpu_instance *inst,
			       struct coda_enc_open_param *open_param)
{
	struct coda_enc_info *p_enc_info;

	if (!inst->codec_info)
		return -EINVAL;
	if (!open_param)
		return -EINVAL;

	p_enc_info = &inst->codec_info->enc_info;

	open_param->level = max(open_param->level, coda_hw_enc_calc_level(open_param));
	open_param->conf_win = coda_hw_enc_calc_conf_win(open_param);

	p_enc_info->open_param = *open_param;

	return 0;
}

int coda_hw_enc_init_seq(struct vpu_instance *inst)
{
	struct coda_enc_info *p_enc_info;
	struct coda_enc_open_param *p_open_param;
	struct vpu_device *vpu = inst->vpu_dev;
	bool en_gamma, en_intra_qp;
	u32 reg_val;

	if (!inst->codec_info)
		return -EINVAL;
	p_enc_info = &inst->codec_info->enc_info;
	p_open_param = &p_enc_info->open_param;

	//TODO check bistream buffer information is needed.
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_BB_START, 0);
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_BB_SIZE, 0);

	reg_val = p_open_param->pic_width << 16 | p_open_param->pic_height;
	if (p_open_param->rot_ang == ROT_ANG_90 || p_open_param->rot_ang == ROT_ANG_270)
		reg_val = p_open_param->pic_height << 16 | p_open_param->pic_width;
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_SRC_SIZE, reg_val);
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_SRC_F_RATE, p_open_param->framerate);

	if (inst->std == AVC_ENC) {
		vpu_write_reg(vpu->dev, CMD_ENC_SEQ_COD_STD, (p_open_param->profile << 4));

		reg_val = ((p_open_param->deblk_filter_offset_beta & 0xF) << 12) |
			  ((p_open_param->deblk_filter_offset_alpha & 0xF) << 8) |
			  (p_open_param->deblk_filter_idc << 6) |
			  (p_open_param->en_constrained_intra_pred << 5) |
			  (p_open_param->chroma_qp_offset & 0x1F);
		vpu_write_reg(vpu->dev, CMD_ENC_SEQ_264_PARA, reg_val);
	}

	reg_val = (ME_LINEBUFFER_MODE << 9) |
		  (p_open_param->me_blk_mode << 5) |
		  (p_open_param->me_use_zero_pmv << 4) |
		  (p_open_param->me_search_range_y << 2) |
		  p_open_param->me_search_range_x;
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_ME_OPTION, reg_val);

	reg_val = (p_open_param->slice_size << 2) | p_open_param->slice_size_mode;
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_SLICE_MODE, reg_val);

	if (p_open_param->rate_control_type) {
		if (inst->std == AVC_ENC) {
			reg_val = (!p_open_param->en_frame_skip << 31) |
				  (p_open_param->initial_delay << 16);
			vpu_write_reg(vpu->dev, CMD_ENC_SEQ_RC_PARA, reg_val);

			reg_val = (p_open_param->en_strict_cbr << 22) |
				  (p_open_param->bitrate << 4) |
				  p_open_param->rate_control_type;
			vpu_write_reg(vpu->dev, CMD_ENC_SEQ_RC_PARA2, reg_val);

			reg_val = (p_open_param->en_max_intra_size << 16) |
				  p_open_param->max_intra_size;
			vpu_write_reg(vpu->dev, CMD_ENC_SEQ_RC_MAX_INTRA_SIZE, reg_val);
		}
	} else {
		vpu_write_reg(vpu->dev, CMD_ENC_SEQ_RC_PARA, 0);
		vpu_write_reg(vpu->dev, CMD_ENC_SEQ_RC_PARA2, 0);
		vpu_write_reg(vpu->dev, CMD_ENC_SEQ_RC_MAX_INTRA_SIZE, 0);
	}

	reg_val = (p_open_param->idr_interval << 21) |
		  (p_open_param->en_rc_gop_i_qp_offset << 20) |
		  (p_open_param->rc_gop_i_qp_offset << 16) |
		  p_open_param->gop_size;
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_GOP_NUM, reg_val);

	reg_val = 0;
	if (p_open_param->min_qp)
		reg_val |= ENABLE_RC_QP_MIN | p_open_param->min_qp << 8;
	if (p_open_param->max_qp)
		reg_val |= ENABLE_RC_QP_MAX | p_open_param->max_qp;
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_QP_RANGE_SET, reg_val);

	reg_val = (p_open_param->en_field_seq_intra_refresh << 18) |
		  (p_open_param->en_count_intra_mb << 17) |
		  (p_open_param->en_consecutive_intra_refresh << 16) |
		  p_open_param->intra_refresh_mb_num;
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_INTRA_REFRESH, reg_val);

	en_intra_qp = (p_open_param->intra_qp > 0) ? true : false;
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_INTRA_QP, p_open_param->intra_qp);

	en_gamma = (p_open_param->gamma > 0) ? true : false;
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_RC_GAMMA, p_open_param->gamma);

	reg_val = (p_open_param->field_ref_mode << 11) |
		  (p_open_param->en_field_encoding << 10) |
		  (en_gamma << 7) |
		  (en_intra_qp << 5) |
		  (p_open_param->en_aud << 2);
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_OPTION, reg_val);
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_RC_INTERVAL_MODE, (p_open_param->mb_interval << 2) |
							      p_open_param->interval_mode);
	vpu_write_reg(vpu->dev, CMD_ENC_SEQ_INTRA_WEIGHT, p_open_param->intra_cost_weight);
	vpu_write_reg(vpu->dev, BIT_WR_PTR, p_enc_info->stream_wr_ptr);
	vpu_write_reg(vpu->dev, BIT_RD_PTR, p_enc_info->stream_rd_ptr);

	reg_val = ENABLE_BWB | (p_open_param->map_type << 9) | (inst->monochrome << 6) |
		  (inst->cbcr_interleave << 2) | VPU_FRAME_ENDIAN;
	vpu_write_reg(vpu->dev, BIT_FRAME_MEM_CTRL, reg_val);

	reg_val = (p_open_param->en_line_buf_int << 6) | ENABLE_DYNAMIC_BUF_ALLOC |
		  ENABLE_BUF_PIC_RESET | VPU_STREAM_ENDIAN;
	vpu_write_reg(vpu->dev, BIT_BIT_STREAM_CTRL, reg_val);

	coda_hw_bit_issue_cmd(vpu, inst, ENC_SEQ_INIT);

	return 0;
}

int coda_hw_enc_get_seq_info(struct vpu_instance *inst)
{
	struct coda_enc_info *p_enc_info;
	struct vpu_device *vpu = inst->vpu_dev;
	int ret = 0;

	if (!inst->codec_info)
		return -EINVAL;

	p_enc_info = &inst->codec_info->enc_info;
	if (vpu_read_reg(vpu->dev, RET_ENC_SEQ_SUCCESS) != 1)
		return -EIO;

	p_enc_info->stream_wr_ptr = vpu_read_reg(vpu->dev, BIT_WR_PTR);
	p_enc_info->stream_end_flag = vpu_read_reg(vpu->dev, BIT_BIT_STREAM_PARAM);

	return ret;
}

static void coda_hw_config_tiled_map(struct vpu_instance *inst)
{
	struct coda_enc_info *p_enc_info;
	struct coda_tiled_map_config *map_cfg;
	const int luma_map = 0x40;
	const int chro_map = 0x40;
	int width;
	int width_chr;
	int i;

	if (!inst->codec_info)
		return;
	p_enc_info = &inst->codec_info->enc_info;
	map_cfg = &p_enc_info->map_cfg;

	for (i = 0; i < 32; i = i + 1) {
		map_cfg->xy2axi_luma_map[i] = luma_map;
		map_cfg->xy2axi_chr_map[i] = chro_map;
	}

	width = p_enc_info->stride;
	width = (width > p_enc_info->frame_buf_height) ? width : p_enc_info->frame_buf_height;
	width_chr = (inst->cbcr_interleave) ? width : width / 2;

	map_cfg->xy2axi_config = 0;
	switch (p_enc_info->open_param.map_type) {
	case LINEAR_FRAME_MAP:
	case LINEAR_FIELD_MAP:
		map_cfg->xy2axi_config = 0;
		break;
	case TILED_FRAME_V_MAP:
		map_cfg->xy2axi_luma_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_luma_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_luma_map[5] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_luma_map[6] = GEN_XY2AXI(0, 0, 0, Y_SEL, 3);
		map_cfg->xy2axi_luma_map[7] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);
		map_cfg->xy2axi_luma_map[8] = GEN_XY2AXI(0, 0, 0, X_SEL, 4);
		map_cfg->xy2axi_luma_map[9] = GEN_XY2AXI(0, 0, 0, X_SEL, 5);
		map_cfg->xy2axi_luma_map[10] = GEN_XY2AXI(0, 0, 0, X_SEL, 6);
		map_cfg->xy2axi_luma_map[11] = GEN_XY2AXI(0, 0, 0, Y_SEL, 4);
		map_cfg->xy2axi_luma_map[12] = GEN_XY2AXI(0, 0, 0, X_SEL, 7);
		map_cfg->xy2axi_luma_map[13] = GEN_XY2AXI(0, 0, 0, Y_SEL, 5);
		if (width <= 512) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width <= 1024) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width <= 2048) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 11);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[23] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		}

		map_cfg->xy2axi_chr_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_chr_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_chr_map[5] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_chr_map[6] = GEN_XY2AXI(0, 0, 0, Y_SEL, 3);
		map_cfg->xy2axi_chr_map[7] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);
		map_cfg->xy2axi_chr_map[8] = GEN_XY2AXI(0, 0, 0, X_SEL, 4);
		map_cfg->xy2axi_chr_map[9] = GEN_XY2AXI(0, 0, 0, X_SEL, 5);
		map_cfg->xy2axi_chr_map[10] = GEN_XY2AXI(0, 0, 0, X_SEL, 6);
		map_cfg->xy2axi_chr_map[11] = GEN_XY2AXI(0, 0, 0, Y_SEL, 5);
		map_cfg->xy2axi_chr_map[12] = GEN_XY2AXI(1, 0, 0, X_SEL, 7);
		map_cfg->xy2axi_chr_map[13] = GEN_XY2AXI(1, 0, 0, Y_SEL, 4);
		if (width_chr <= 512) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width_chr <= 1024) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width_chr <= 2048) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 11);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[23] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		}

		map_cfg->xy2axi_config = GEN_CONFIG(0, 0, 0, 1, 1, 15, 0, 15, 0);
		break;
	case TILED_FRAME_H_MAP:
		map_cfg->xy2axi_luma_map[3] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);
		map_cfg->xy2axi_luma_map[4] = GEN_XY2AXI(0, 0, 0, X_SEL, 4);
		map_cfg->xy2axi_luma_map[5] = GEN_XY2AXI(0, 0, 0, X_SEL, 5);
		map_cfg->xy2axi_luma_map[6] = GEN_XY2AXI(0, 0, 0, X_SEL, 6);
		map_cfg->xy2axi_luma_map[7] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_luma_map[8] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_luma_map[9] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_luma_map[10] = GEN_XY2AXI(0, 0, 0, Y_SEL, 3);
		map_cfg->xy2axi_luma_map[11] = GEN_XY2AXI(0, 0, 0, Y_SEL, 4);
		map_cfg->xy2axi_luma_map[12] = GEN_XY2AXI(0, 0, 0, X_SEL, 7);
		map_cfg->xy2axi_luma_map[13] = GEN_XY2AXI(0, 0, 0, Y_SEL, 5);
		if (width <= 512) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width <= 1024) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width <= 2048) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 11);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[23] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		}

		map_cfg->xy2axi_chr_map[3] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);
		map_cfg->xy2axi_chr_map[4] = GEN_XY2AXI(0, 0, 0, X_SEL, 4);
		map_cfg->xy2axi_chr_map[5] = GEN_XY2AXI(0, 0, 0, X_SEL, 5);
		map_cfg->xy2axi_chr_map[6] = GEN_XY2AXI(0, 0, 0, X_SEL, 6);
		map_cfg->xy2axi_chr_map[7] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_chr_map[8] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_chr_map[9] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_chr_map[10] = GEN_XY2AXI(0, 0, 0, Y_SEL, 3);
		map_cfg->xy2axi_chr_map[11] = GEN_XY2AXI(0, 0, 0, Y_SEL, 5);
		map_cfg->xy2axi_chr_map[12] = GEN_XY2AXI(1, 0, 0, X_SEL, 7);
		map_cfg->xy2axi_chr_map[13] = GEN_XY2AXI(1, 0, 0, Y_SEL, 4);
		if (width_chr <= 512) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width_chr <= 1024) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width_chr <= 2048) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 11);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[23] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		}

		map_cfg->xy2axi_config = GEN_CONFIG(0, 0, 0, 1, 0, 15, 15, 15, 15);
		break;
	case TILED_FIELD_V_MAP:
		map_cfg->xy2axi_luma_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_luma_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_luma_map[5] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_luma_map[6] = GEN_XY2AXI(0, 0, 0, Y_SEL, 3);
		map_cfg->xy2axi_luma_map[7] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);
		map_cfg->xy2axi_luma_map[8] = GEN_XY2AXI(0, 0, 0, X_SEL, 4);
		map_cfg->xy2axi_luma_map[9] = GEN_XY2AXI(0, 0, 0, X_SEL, 5);
		map_cfg->xy2axi_luma_map[10] = GEN_XY2AXI(0, 0, 0, X_SEL, 6);
		map_cfg->xy2axi_luma_map[11] = GEN_XY2AXI(0, 0, 0, Y_SEL, 4);
		map_cfg->xy2axi_luma_map[12] = GEN_XY2AXI(0, 0, 0, X_SEL, 7);
		map_cfg->xy2axi_luma_map[13] = GEN_XY2AXI(0, 0, 1, Y_SEL, 5);
		if (width <= 512) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width <= 1024) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width <= 2048) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 11);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[23] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		}

		map_cfg->xy2axi_chr_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_chr_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_chr_map[5] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_chr_map[6] = GEN_XY2AXI(0, 0, 0, Y_SEL, 3);
		map_cfg->xy2axi_chr_map[7] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);
		map_cfg->xy2axi_chr_map[8] = GEN_XY2AXI(0, 0, 0, X_SEL, 4);
		map_cfg->xy2axi_chr_map[9] = GEN_XY2AXI(0, 0, 0, X_SEL, 5);
		map_cfg->xy2axi_chr_map[10] = GEN_XY2AXI(0, 0, 0, X_SEL, 6);
		map_cfg->xy2axi_chr_map[11] = GEN_XY2AXI(0, 0, 0, Y_SEL, 5);
		map_cfg->xy2axi_chr_map[12] = GEN_XY2AXI(1, 0, 0, X_SEL, 7);
		map_cfg->xy2axi_chr_map[13] = GEN_XY2AXI(1, 0, 1, Y_SEL, 4);
		if (width_chr <= 512) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width_chr <= 1024) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width_chr <= 2048) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 11);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[23] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		}

		map_cfg->xy2axi_config = GEN_CONFIG(0, 1, 1, 1, 1, 15, 15, 15, 15);
		break;
	case TILED_MIXED_V_MAP:
		map_cfg->xy2axi_luma_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_luma_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_luma_map[5] = GEN_XY2AXI(0, 0, 0, Y_SEL, 3);
		map_cfg->xy2axi_luma_map[6] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_luma_map[7] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);
		map_cfg->xy2axi_luma_map[8] = GEN_XY2AXI(0, 0, 0, X_SEL, 4);
		map_cfg->xy2axi_luma_map[9] = GEN_XY2AXI(0, 0, 0, X_SEL, 5);
		map_cfg->xy2axi_luma_map[10] = GEN_XY2AXI(0, 0, 0, X_SEL, 6);
		map_cfg->xy2axi_luma_map[11] = GEN_XY2AXI(0, 0, 0, Y_SEL, 4);
		map_cfg->xy2axi_luma_map[12] = GEN_XY2AXI(0, 0, 0, X_SEL, 7);
		map_cfg->xy2axi_luma_map[13] = GEN_XY2AXI(0, 0, 0, Y_SEL, 5);
		if (width <= 512) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width <= 1024) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width <= 2048) {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else {
			map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 11);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[23] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		}

		map_cfg->xy2axi_chr_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_chr_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_chr_map[5] = GEN_XY2AXI(0, 0, 0, Y_SEL, 3);
		map_cfg->xy2axi_chr_map[6] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_chr_map[7] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);
		map_cfg->xy2axi_chr_map[8] = GEN_XY2AXI(0, 0, 0, X_SEL, 4);
		map_cfg->xy2axi_chr_map[9] = GEN_XY2AXI(0, 0, 0, X_SEL, 5);
		map_cfg->xy2axi_chr_map[10] = GEN_XY2AXI(0, 0, 0, X_SEL, 6);
		map_cfg->xy2axi_chr_map[11] = GEN_XY2AXI(0, 0, 0, Y_SEL, 5);
		map_cfg->xy2axi_chr_map[12] = GEN_XY2AXI(1, 0, 0, X_SEL, 7);
		map_cfg->xy2axi_chr_map[13] = GEN_XY2AXI(1, 0, 0, Y_SEL, 4);
		if (width_chr <= 512) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width_chr <= 1024) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width_chr <= 2048) {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else {
			map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
			map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 11);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[23] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		}

		map_cfg->xy2axi_config = GEN_CONFIG(0, 0, 1, 1, 1, 7, 7, 7, 7);
		break;
	case TILED_FRAME_MB_RASTER_MAP:
		map_cfg->xy2axi_luma_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_luma_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_luma_map[5] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_luma_map[6] = GEN_XY2AXI(0, 0, 0, Y_SEL, 3);
		map_cfg->xy2axi_luma_map[7] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);

		map_cfg->xy2axi_chr_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_chr_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_chr_map[5] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_chr_map[6] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);

		map_cfg->xy2axi_config = GEN_CONFIG(0, 0, 0, 1, 1, 15, 0, 7, 0);
		break;
	case TILED_FIELD_MB_RASTER_MAP:
		map_cfg->xy2axi_luma_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_luma_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_luma_map[5] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_luma_map[6] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);

		map_cfg->xy2axi_chr_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_chr_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_chr_map[5] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);

		map_cfg->xy2axi_config = GEN_CONFIG(0, 1, 1, 1, 1, 7, 7, 3, 3);
		break;
	case TILED_FRAME_NO_BANK_MAP:
	case TILED_FIELD_NO_BANK_MAP:
		map_cfg->xy2axi_luma_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_luma_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_luma_map[5] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_luma_map[6] = GEN_XY2AXI(0, 0, 0, Y_SEL, 3);
		map_cfg->xy2axi_luma_map[7] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);
		map_cfg->xy2axi_luma_map[8] = GEN_XY2AXI(0, 0, 0, Y_SEL, 4);
		map_cfg->xy2axi_luma_map[9] = GEN_XY2AXI(0, 0, 0, Y_SEL, 5);
		map_cfg->xy2axi_luma_map[10] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
		map_cfg->xy2axi_luma_map[11] = GEN_XY2AXI(0, 0, 0, X_SEL, 4);
		map_cfg->xy2axi_luma_map[12] = GEN_XY2AXI(0, 0, 0, X_SEL, 5);
		map_cfg->xy2axi_luma_map[13] = GEN_XY2AXI(0, 0, 0, X_SEL, 6);
		map_cfg->xy2axi_luma_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 7);
		map_cfg->xy2axi_luma_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
		if (width <= 512) {
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width <= 1024) {
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width <= 2048) {
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else {
			map_cfg->xy2axi_luma_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_luma_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_luma_map[18] = GEN_XY2AXI(0, 0, 0, X_SEL, 11);
			map_cfg->xy2axi_luma_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_luma_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_luma_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_luma_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_luma_map[23] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		}

		map_cfg->xy2axi_chr_map[3] = GEN_XY2AXI(0, 0, 0, Y_SEL, 0);
		map_cfg->xy2axi_chr_map[4] = GEN_XY2AXI(0, 0, 0, Y_SEL, 1);
		map_cfg->xy2axi_chr_map[5] = GEN_XY2AXI(0, 0, 0, Y_SEL, 2);
		map_cfg->xy2axi_chr_map[6] = GEN_XY2AXI(0, 0, 0, Y_SEL, 3);
		map_cfg->xy2axi_chr_map[7] = GEN_XY2AXI(0, 0, 0, X_SEL, 3);
		map_cfg->xy2axi_chr_map[8] = GEN_XY2AXI(0, 0, 0, Y_SEL, 4);
		map_cfg->xy2axi_chr_map[9] = GEN_XY2AXI(0, 0, 0, Y_SEL, 5);
		map_cfg->xy2axi_chr_map[10] = GEN_XY2AXI(0, 0, 0, Y_SEL, 6);
		map_cfg->xy2axi_chr_map[11] = GEN_XY2AXI(0, 0, 0, X_SEL, 4);
		map_cfg->xy2axi_chr_map[12] = GEN_XY2AXI(0, 0, 0, X_SEL, 5);
		map_cfg->xy2axi_chr_map[13] = GEN_XY2AXI(0, 0, 0, X_SEL, 6);
		map_cfg->xy2axi_chr_map[14] = GEN_XY2AXI(0, 0, 0, X_SEL, 7);
		map_cfg->xy2axi_chr_map[15] = GEN_XY2AXI(0, 0, 0, X_SEL, 8);
		if (width_chr <= 512) {
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width_chr <= 1024) {
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else if (width_chr <= 2048) {
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		} else {
			map_cfg->xy2axi_chr_map[16] = GEN_XY2AXI(0, 0, 0, X_SEL, 9);
			map_cfg->xy2axi_chr_map[17] = GEN_XY2AXI(0, 0, 0, X_SEL, 10);
			map_cfg->xy2axi_chr_map[18] = GEN_XY2AXI(0, 0, 0, X_SEL, 11);
			map_cfg->xy2axi_chr_map[19] = GEN_XY2AXI(0, 0, 0, Y_SEL, 7);
			map_cfg->xy2axi_chr_map[20] = GEN_XY2AXI(0, 0, 0, Y_SEL, 8);
			map_cfg->xy2axi_chr_map[21] = GEN_XY2AXI(0, 0, 0, Y_SEL, 9);
			map_cfg->xy2axi_chr_map[22] = GEN_XY2AXI(0, 0, 0, Y_SEL, 10);
			map_cfg->xy2axi_chr_map[23] = GEN_XY2AXI(0, 0, 0, Y_SEL, 11);
		}

		if (p_enc_info->open_param.map_type == TILED_FRAME_NO_BANK_MAP)
			map_cfg->xy2axi_config = GEN_CONFIG(0, 0, 0, 1, 1, 15, 0, 15, 0);
		else
			map_cfg->xy2axi_config = GEN_CONFIG(0, 1, 1, 1, 1, 15, 15, 15, 15);
		break;
	default:
		map_cfg->xy2axi_config = 0;
		break;
	}

	map_cfg->tb_separate_map = (map_cfg->xy2axi_config >> 19) & 0x1;
	map_cfg->top_bot_split = (map_cfg->xy2axi_config >> 18) & 0x1;
	map_cfg->tiled_map = (map_cfg->xy2axi_config >> 17) & 0x1;
}

static void coda_hw_config_sec_axi(struct coda_enc_info *p_enc_info)
{
	struct coda_sec_axi_info *sec_axi = &p_enc_info->sec_axi;
	u32 width = p_enc_info->stride;
	u32 mb_num_x = ((width & 0xFFFF) + 15) / 16;
	u32 buf_size;
	u32 total_buf_size = 0;

	if (!p_enc_info->open_param.sec_axi_size)
		return;

	buf_size = (mb_num_x > 128) ? ALIGN((mb_num_x * 16), 1024) : 0;
	if (buf_size > p_enc_info->open_param.sec_axi_size)
		return;

	sec_axi->bit_enable = true;
	sec_axi->bit_buf = p_enc_info->open_param.sec_axi_base;
	total_buf_size += buf_size;

	buf_size = ALIGN((mb_num_x * 64), 1024);
	if (total_buf_size + buf_size > p_enc_info->open_param.sec_axi_size)
		return;

	sec_axi->ip_enable = true;
	sec_axi->ip_buf = p_enc_info->open_param.sec_axi_base + total_buf_size;
	total_buf_size += buf_size;

	buf_size = ALIGN((mb_num_x * 64), 1024);
	if (total_buf_size + buf_size > p_enc_info->open_param.sec_axi_size)
		return;

	sec_axi->dbk_y_enable = true;
	sec_axi->dbk_y_buf = p_enc_info->open_param.sec_axi_base + total_buf_size;
	total_buf_size += buf_size;

	buf_size = ALIGN((mb_num_x * 64), 1024);
	if (total_buf_size + buf_size > p_enc_info->open_param.sec_axi_size)
		return;

	sec_axi->dbk_c_enable = true;
	sec_axi->dbk_c_buf = p_enc_info->open_param.sec_axi_base + total_buf_size;
	total_buf_size += buf_size;
}

int coda_hw_enc_register_frame_buffer(struct vpu_instance *inst, struct coda_frame_buffer *fb_arr)
{
	struct coda_enc_info *p_enc_info;
	struct coda_tiled_map_config *map_cfg;
	struct vpu_device *vpu = inst->vpu_dev;
	int i;
	int ret;
	u32 reg_val;

	if (!inst->codec_info)
		return -EINVAL;
	p_enc_info = &inst->codec_info->enc_info;
	map_cfg = &p_enc_info->map_cfg;

	coda_hw_config_sec_axi(p_enc_info);
	coda_hw_config_tiled_map(inst);

	for (i = 0; i < 32; i++)
		vpu_write_reg(vpu->dev, GDI_XY2AXI_LUM_BIT00 + 4 * i, map_cfg->xy2axi_luma_map[i]);
	for (i = 0; i < 32; i++)
		vpu_write_reg(vpu->dev, GDI_XY2AXI_CHR_BIT00 + 4 * i, map_cfg->xy2axi_chr_map[i]);
	vpu_write_reg(vpu->dev, GDI_XY2AXI_CONFIG, map_cfg->xy2axi_config);

	reg_val = ENABLE_BWB | (p_enc_info->open_param.map_type << 9) | (inst->monochrome << 6) |
		  (inst->cbcr_interleave << 2) | VPU_FRAME_ENDIAN;
	vpu_write_reg(vpu->dev, BIT_FRAME_MEM_CTRL, reg_val);

	for (i = 0; i < p_enc_info->frame_buf_num; i++) {
		p_enc_info->frame_addr[i][0][0] = (fb_arr[i].buf_y >> 24) & 0xFF;
		p_enc_info->frame_addr[i][0][1] = (fb_arr[i].buf_y >> 16) & 0xFF;
		p_enc_info->frame_addr[i][0][2] = (fb_arr[i].buf_y >> 8) & 0xFF;
		p_enc_info->frame_addr[i][0][3] = (fb_arr[i].buf_y >> 0) & 0xFF;
		p_enc_info->frame_addr[i][1][0] = (fb_arr[i].buf_cb >> 24) & 0xFF;
		p_enc_info->frame_addr[i][1][1] = (fb_arr[i].buf_cb >> 16) & 0xFF;
		p_enc_info->frame_addr[i][1][2] = (fb_arr[i].buf_cb >> 8) & 0xFF;
		p_enc_info->frame_addr[i][1][3] = (fb_arr[i].buf_cb >> 0) & 0xFF;
		p_enc_info->frame_addr[i][2][0] = (fb_arr[i].buf_cr >> 24) & 0xFF;
		p_enc_info->frame_addr[i][2][1] = (fb_arr[i].buf_cr >> 16) & 0xFF;
		p_enc_info->frame_addr[i][2][2] = (fb_arr[i].buf_cr >> 8) & 0xFF;
		p_enc_info->frame_addr[i][2][3] = (fb_arr[i].buf_cr >> 0) & 0xFF;
	}
	ret = coda_vdi_write_memory(&vpu->common_mem,
				    CODA_CODE_BUF_SIZE + CODA_TEMP_BUF_SIZE,
				    (u8 *)p_enc_info->frame_addr,
				    sizeof(p_enc_info->frame_addr), VDI_BIG_ENDIAN);
	if (ret <= 0)
		return ret;

	for (i = 0; i < p_enc_info->frame_buf_num; i++) {
		p_enc_info->frame_addr[i][0][0] = (fb_arr[i].buf_y_bot >> 24) & 0xFF;
		p_enc_info->frame_addr[i][0][1] = (fb_arr[i].buf_y_bot >> 16) & 0xFF;
		p_enc_info->frame_addr[i][0][2] = (fb_arr[i].buf_y_bot >> 8) & 0xFF;
		p_enc_info->frame_addr[i][0][3] = (fb_arr[i].buf_y_bot >> 0) & 0xFF;
		p_enc_info->frame_addr[i][1][0] = (fb_arr[i].buf_cb_bot >> 24) & 0xFF;
		p_enc_info->frame_addr[i][1][1] = (fb_arr[i].buf_cb_bot >> 16) & 0xFF;
		p_enc_info->frame_addr[i][1][2] = (fb_arr[i].buf_cb_bot >> 8) & 0xFF;
		p_enc_info->frame_addr[i][1][3] = (fb_arr[i].buf_cb_bot >> 0) & 0xFF;
		p_enc_info->frame_addr[i][2][0] = (fb_arr[i].buf_cr_bot >> 24) & 0xFF;
		p_enc_info->frame_addr[i][2][1] = (fb_arr[i].buf_cr_bot >> 16) & 0xFF;
		p_enc_info->frame_addr[i][2][2] = (fb_arr[i].buf_cr_bot >> 8) & 0xFF;
		p_enc_info->frame_addr[i][2][3] = (fb_arr[i].buf_cr_bot >> 0) & 0xFF;
	}
	ret = coda_vdi_write_memory(&vpu->common_mem,
				    CODA_CODE_BUF_SIZE + CODA_TEMP_BUF_SIZE + 384 + 128,
				    (u8 *)p_enc_info->frame_addr,
				    sizeof(p_enc_info->frame_addr), VDI_BIG_ENDIAN);
	if (ret <= 0)
		return ret;

	vpu_write_reg(vpu->dev, CMD_SET_FRAME_BUF_NUM, p_enc_info->frame_buf_num);
	vpu_write_reg(vpu->dev, CMD_SET_FRAME_BUF_STRIDE, p_enc_info->stride);
	vpu_write_reg(vpu->dev, CMD_SET_FRAME_AXI_BIT_ADDR, p_enc_info->sec_axi.bit_buf);
	vpu_write_reg(vpu->dev, CMD_SET_FRAME_AXI_IPACDC_ADDR, p_enc_info->sec_axi.ip_buf);
	vpu_write_reg(vpu->dev, CMD_SET_FRAME_AXI_DBKY_ADDR, p_enc_info->sec_axi.dbk_y_buf);
	vpu_write_reg(vpu->dev, CMD_SET_FRAME_AXI_DBKC_ADDR, p_enc_info->sec_axi.dbk_c_buf);

	p_enc_info->cache_config = (p_enc_info->open_param.map_type == LINEAR_FRAME_MAP) ? 0x7E8 :
											   0x7EC;
	vpu_write_reg(vpu->dev, CMD_SET_FRAME_CACHE_CONFIG, p_enc_info->cache_config);

	coda_hw_bit_issue_cmd(vpu, inst, SET_FRAME_BUF);
	ret = coda_hw_wait_vpu_busy(vpu, BIT_BUSY_FLAG);
	if (ret)
		return -ETIMEDOUT;

	if (vpu_read_reg(vpu->dev, RET_SET_FRAME_SUCCESS) != 1)
		return -EIO;

	return 0;
}

static int coda_hw_enc_encode_header(struct vpu_instance *inst, u32 type)
{
	struct coda_enc_info *p_enc_info;
	struct coda_enc_open_param *p_open_param;
	struct vpu_device *vpu = inst->vpu_dev;
	bool en_frame_crop = false;
	u32 reg_val;
	u32 header_size;
	int ret;

	if (!inst->codec_info)
		return -EINVAL;
	p_enc_info = &inst->codec_info->enc_info;
	p_open_param = &p_enc_info->open_param;

	p_enc_info->stream_wr_ptr = p_enc_info->bitstream_buf;
	p_enc_info->stream_rd_ptr = p_enc_info->bitstream_buf;

	reg_val = (p_open_param->en_line_buf_int << 6) | ENABLE_DYNAMIC_BUF_ALLOC |
		  ENABLE_BUF_PIC_RESET | VPU_STREAM_ENDIAN;
	vpu_write_reg(vpu->dev, BIT_BIT_STREAM_CTRL, reg_val);
	vpu_write_reg(vpu->dev, BIT_WR_PTR, p_enc_info->stream_wr_ptr);
	vpu_write_reg(vpu->dev, BIT_RD_PTR, p_enc_info->stream_rd_ptr);

	vpu_write_reg(vpu->dev, CMD_ENC_HEADER_BB_START, p_enc_info->bitstream_buf);
	vpu_write_reg(vpu->dev, CMD_ENC_HEADER_BB_SIZE, p_enc_info->bitstream_buf_size / 1024);

	if (type == H264_SPS_TYPE) {
		vpu_write_reg(vpu->dev, CMD_ENC_HEADER_PROFILE, p_open_param->profile);
		vpu_write_reg(vpu->dev, CMD_ENC_HEADER_CHROMA_FORMAT, inst->monochrome);

		reg_val = (p_open_param->field_ref_mode << 1) | p_open_param->en_field_encoding;
		vpu_write_reg(vpu->dev, CMD_ENC_HEADER_FIELD_FLAG, reg_val);

		if (p_open_param->conf_win.top || p_open_param->conf_win.left ||
		    p_open_param->conf_win.bottom || p_open_param->conf_win.right) {
			reg_val = (p_open_param->conf_win.left << 16) |
				  p_open_param->conf_win.right;
			vpu_write_reg(vpu->dev, CMD_ENC_HEADER_FRAME_CROP_H, reg_val);
			reg_val = (p_open_param->conf_win.top << 16) |
				  p_open_param->conf_win.bottom;
			vpu_write_reg(vpu->dev, CMD_ENC_HEADER_FRAME_CROP_V, reg_val);
			en_frame_crop = true;
		}

		reg_val = (p_open_param->sar.idc << 2) | (p_open_param->sar.enable << 1) |
			  ENABLE_VUI_PARAMETER_PRESENT_FLAG;
		vpu_write_reg(vpu->dev, CMD_ENC_HEADER_VUI_INFO, reg_val);
		reg_val = (p_open_param->sar.width << 16) | p_open_param->sar.height;
		vpu_write_reg(vpu->dev, CMD_ENC_HEADER_VUI_EXTENDED_SAR, reg_val);
		reg_val = (p_open_param->color.color_range << 24) |
			  (p_open_param->color.color_primaries << 16) |
			  (p_open_param->color.transfer_characteristics << 8) |
			  p_open_param->color.matrix_coefficients;
		vpu_write_reg(vpu->dev, CMD_ENC_HEADER_VUI_COLOR_INFO, reg_val);
	} else if (type == H264_PPS_TYPE) {
		vpu_write_reg(vpu->dev, CMD_ENC_HEADER_CABAC_MODE, p_open_param->en_cabac_mode);
		vpu_write_reg(vpu->dev, CMD_ENC_HEADER_CABAC_INIT_IDC,
			      p_open_param->cabac_init_idc);
		vpu_write_reg(vpu->dev, CMD_ENC_HEADER_TRANSFORM_8X8,
			      p_open_param->en_transform_8x8);
	}

	reg_val = ENABLE_ZERO_PADDING |
		  (avc_level_list[p_open_param->level].level_idc << 8) |
		  (en_frame_crop << 2) | type;
	vpu_write_reg(vpu->dev, CMD_ENC_HEADER_CODE, reg_val);

	coda_hw_bit_issue_cmd(vpu, inst, ENCODE_HEADER);
	ret = coda_hw_wait_vpu_busy(vpu, BIT_BUSY_FLAG);
	if (ret)
		return -ETIMEDOUT;

	if (vpu_read_reg(vpu->dev, RET_ENC_HEADER_SUCCESS) != 1)
		return -EIO;

	p_enc_info->stream_wr_ptr = vpu_read_reg(vpu->dev, BIT_WR_PTR);
	p_enc_info->stream_rd_ptr = vpu_read_reg(vpu->dev, BIT_RD_PTR);
	header_size = p_enc_info->stream_wr_ptr - p_enc_info->stream_rd_ptr;

	p_enc_info->header_size += header_size;
	p_enc_info->bitstream_buf += header_size;
	p_enc_info->bitstream_buf_size -= header_size;

	return 0;
}

static void coda_hw_set_report_buf_addr_info(struct vpu_instance *inst)
{
	u8 addr_info[CODA_REPORT_BUF_SIZE_ADDR_INFO] = {0, };
	dma_addr_t addr;

	/* Set report buffer address for MB info*/
	addr = inst->report_vbuf.daddr + CODA_REPORT_BUF_SIZE_ADDR_INFO;

	addr_info[4] = (addr >> 24) & 0xFF;
	addr_info[5] = (addr >> 16) & 0xFF;
	addr_info[6] = (addr >> 8) & 0xFF;
	addr_info[7] = (addr >> 0) & 0xFF;

	/* Set report buffer address for MV info*/
	addr += CODA_REPORT_BUF_SIZE_MB_INFO;

	addr_info[12] = (addr >> 24) & 0xFF;
	addr_info[13] = (addr >> 16) & 0xFF;
	addr_info[14] = (addr >> 8) & 0xFF;
	addr_info[15] = (addr >> 0) & 0xFF;

	/* Set report buffer address for Slice info*/
	addr += CODA_REPORT_BUF_SIZE_MV_INFO;

	addr_info[20] = (addr >> 24) & 0xFF;
	addr_info[21] = (addr >> 16) & 0xFF;
	addr_info[22] = (addr >> 8) & 0xFF;
	addr_info[23] = (addr >> 0) & 0xFF;

	/* Set report buffer address for Cost info*/
	addr += CODA_REPORT_BUF_SIZE_SLICE_INFO;

	addr_info[28] = (addr >> 24) & 0xFF;
	addr_info[29] = (addr >> 16) & 0xFF;
	addr_info[30] = (addr >> 8) & 0xFF;
	addr_info[31] = (addr >> 0) & 0xFF;

	coda_vdi_write_memory(&inst->report_vbuf, 0, addr_info,
			      CODA_REPORT_BUF_SIZE_ADDR_INFO, VDI_BIG_ENDIAN);
}

static void coda_hw_get_report_buf_info(struct vpu_instance *inst,
					struct coda_enc_output_info *info)
{
	struct vpu_device *vpu = inst->vpu_dev;
	u8 addr_info[CODA_REPORT_BUF_SIZE_ADDR_INFO] = { 0 };
	u8 *report_data;
	int i;

	coda_vdi_read_memory(&inst->report_vbuf, 0, addr_info,
			     CODA_REPORT_BUF_SIZE_ADDR_INFO, VDI_BIG_ENDIAN);

	/* Get MB information*/
	info->mb_info.size = ((u32)addr_info[2] << 8) | (addr_info[3] << 0);
	info->mb_info.buf_offset = CODA_REPORT_BUF_SIZE_ADDR_INFO;
	info->mb_info.size = min(info->mb_info.size, CODA_REPORT_BUF_SIZE_MB_INFO);

	report_data = kzalloc(ALIGN(info->mb_info.size, 8), GFP_KERNEL);
	coda_vdi_read_memory(&inst->report_vbuf, info->mb_info.buf_offset,
			     report_data, ALIGN(info->mb_info.size, 8), VDI_BIG_ENDIAN);

	/* [mb_idx]qp:slice_boundary */
	for (i = 0; i < info->mb_info.size; i++) {
		u32 qp, slice_boundary;

		qp = report_data[i] & 0x3F;
		slice_boundary = (report_data[i] >> 6) & 1;
		dev_dbg(vpu->dev, "[%5d]%5d:%5d", i, qp, slice_boundary);
	}

	kfree(report_data);

	/* Get MV information*/
	info->mv_info.size = ((u32)addr_info[10] << 8) | (addr_info[11] << 0);
	info->mv_info.ext_data = addr_info[9];
	info->mv_info.buf_offset = CODA_REPORT_BUF_SIZE_ADDR_INFO + CODA_REPORT_BUF_SIZE_MB_INFO;
	info->mv_info.size = min(info->mv_info.size, CODA_REPORT_BUF_SIZE_MV_INFO);

	report_data = kzalloc(ALIGN(info->mv_info.size, 8), GFP_KERNEL);
	coda_vdi_read_memory(&inst->report_vbuf, info->mv_info.buf_offset,
			     report_data, ALIGN(info->mv_info.size, 8), VDI_BIG_ENDIAN);

	/* [mb_idx]is_intra:mb_x:mb_y */
	for (i = 0; i < info->mv_info.size / 4; i++) {
		u32 pos;
		s16 mb_x, mb_y;
		bool is_intra;

		pos = i * 4;
		mb_x = ((u32)report_data[pos] << 8) | report_data[pos + 1];
		mb_y = ((u32)report_data[pos + 2] << 8) | report_data[pos + 3];

		is_intra = mb_x & 0x8000;
		if (is_intra) {
			mb_x = 0;
			mb_y = 0;
		} else {
			mb_x = (mb_x & 0x7FFF) | ((mb_x << 1) & 0x8000);
		}

		dev_dbg(vpu->dev, "[%5d]%5d:%5d:%5d", i, is_intra, mb_x, mb_y);
	}

	kfree(report_data);

	/* Get Slice information*/
	info->slice_info.size = ((u32)addr_info[18] << 8) | (addr_info[19] << 0);
	info->slice_info.ext_data = addr_info[17];
	info->slice_info.buf_offset = CODA_REPORT_BUF_SIZE_ADDR_INFO +
				      CODA_REPORT_BUF_SIZE_MB_INFO + CODA_REPORT_BUF_SIZE_MV_INFO;
	info->slice_info.size = min(info->slice_info.size, CODA_REPORT_BUF_SIZE_SLICE_INFO);

	report_data = kzalloc(ALIGN(info->slice_info.size, 8), GFP_KERNEL);
	coda_vdi_read_memory(&inst->report_vbuf, info->slice_info.buf_offset,
			     report_data, ALIGN(info->slice_info.size, 8), VDI_BIG_ENDIAN);

	/* [slice_idx]mb_addr:slice_bits */
	for (i = 0; i < info->slice_info.size / 8; i++) {
		u32 pos;
		u32 mb_addr, slice_bits;

		pos = i * 8;
		mb_addr = ((u32)report_data[pos + 2] << 8) | report_data[pos + 3];
		slice_bits = ((u32)report_data[pos + 4] << 24) |
			     ((u32)report_data[pos + 5] << 16) |
			     ((u32)report_data[pos + 6] << 8) |
			     (report_data[pos + 7]);
		dev_dbg(vpu->dev, "[%5d]%5d:%5d", i, mb_addr, slice_bits);
	}

	kfree(report_data);

	/* Get Cost information*/
	info->cost_info.size = ((u32)addr_info[26] << 8) | (addr_info[27] << 0);
	info->cost_info.buf_offset = CODA_REPORT_BUF_SIZE_ADDR_INFO +
				     CODA_REPORT_BUF_SIZE_MB_INFO + CODA_REPORT_BUF_SIZE_MV_INFO +
				     CODA_REPORT_BUF_SIZE_SLICE_INFO;
	info->cost_info.size = min(info->cost_info.size, CODA_REPORT_BUF_SIZE_COST_INFO);

	report_data = kzalloc(ALIGN(info->cost_info.size, 8), GFP_KERNEL);
	coda_vdi_read_memory(&inst->report_vbuf, info->cost_info.buf_offset,
			     report_data, ALIGN(info->cost_info.size, 8), VDI_BIG_ENDIAN);

	/* [mb_idx]intra_cost:inter_cost */
	for (i = 0; i < info->cost_info.size / 8; i++) {
		u32 pos;
		u32 intra_cost, inter_cost;

		pos = i * 8;
		intra_cost = ((u32)report_data[pos] << 24) | ((u32)report_data[pos + 1] << 16) |
			     ((u32)report_data[pos + 2] << 8) | (report_data[pos + 3]);
		inter_cost = ((u32)report_data[pos + 4] << 24) | ((u32)report_data[pos + 5] << 16) |
			     ((u32)report_data[pos + 6] << 8) | ((u32)report_data[pos + 7]);
		dev_dbg(vpu->dev, "[%5d]%5d:%5d", i, intra_cost, inter_cost);
	}

	kfree(report_data);
}

static int coda_hw_enc_param_change(struct vpu_instance *inst, struct coda_enc_param *option)
{
	struct coda_enc_info *p_enc_info;
	struct vpu_device *vpu = inst->vpu_dev;
	int param_change_enable = 0;
	int ret;

	if (!inst->codec_info)
		return -EINVAL;
	p_enc_info = &inst->codec_info->enc_info;
	if (p_enc_info->open_param.bitrate != option->bitrate) {
		param_change_enable |= BIT(PARAM_CHANGE_ENABLE_BITRATE);
		vpu_write_reg(vpu->dev, CMD_ENC_PARAM_CHANGE_BITRATE, option->bitrate);
	}

	if (!param_change_enable)
		return 0;

	vpu_write_reg(vpu->dev, CMD_ENC_PARAM_CHANGE_ENABLE, param_change_enable);

	coda_hw_bit_issue_cmd(vpu, inst, ENC_PARAM_CHANGE);
	ret = coda_hw_wait_vpu_busy(vpu, BIT_BUSY_FLAG);
	if (ret)
		return -ETIMEDOUT;

	if (vpu_read_reg(vpu->dev, RET_ENC_PARAM_CHANGE_SUCCESS) != 1)
		return -EIO;

	p_enc_info->open_param.bitrate = option->bitrate;

	return ret;
}

int coda_hw_enc_encode(struct vpu_instance *inst, struct coda_enc_param *option, u32 *fail_res)
{
	struct coda_enc_info *p_enc_info;
	struct coda_tiled_map_config *map_cfg;
	struct coda_frame_buffer *source_frame = option->source_frame;
	struct vpu_device *vpu = inst->vpu_dev;
	int i;
	u32 rot_mir_mode = 0;
	int reg_val, ret;

	if (!inst->codec_info)
		return -EINVAL;
	p_enc_info = &inst->codec_info->enc_info;
	map_cfg = &p_enc_info->map_cfg;

	ret = coda_hw_enc_param_change(inst, option);
	if (ret)
		return ret;

	if (option->force_i_picture) {
		ret = coda_hw_enc_encode_header(inst, H264_SPS_TYPE);
		if (ret)
			return ret;

		ret = coda_hw_enc_encode_header(inst, H264_PPS_TYPE);
		if (ret)
			return ret;
	}

	p_enc_info->stream_wr_ptr = p_enc_info->bitstream_buf;
	p_enc_info->stream_rd_ptr = p_enc_info->bitstream_buf;
	p_enc_info->src_index = option->src_idx;

	//TODO ROI command.

	for (i = 0; i < 32; i++)
		vpu_write_reg(vpu->dev, GDI_XY2AXI_LUM_BIT00 + 4 * i, map_cfg->xy2axi_luma_map[i]);
	for (i = 0; i < 32; i++)
		vpu_write_reg(vpu->dev, GDI_XY2AXI_CHR_BIT00 + 4 * i, map_cfg->xy2axi_chr_map[i]);
	vpu_write_reg(vpu->dev, GDI_XY2AXI_CONFIG, map_cfg->xy2axi_config);

	rot_mir_mode = coda_hw_enc_calc_rot_mir_mode(&p_enc_info->open_param);
	reg_val = ((inst->nv21 & 0x01) << 21) |
		  ((inst->cbcr_interleave & 0x01) << 18) |
		  ((VPU_SOURCE_ENDIAN & 0x03) << 16) |
		  ((DMA_REQUEST_MODE & 0x03) << 5) |
		  ((LONG_BURST_MODE & 0x01) << 4) |
		  rot_mir_mode;
	vpu_write_reg(vpu->dev, CMD_ENC_PIC_ROT_MODE, reg_val);
	vpu_write_reg(vpu->dev, CMD_ENC_PIC_QS, option->quant_param);

	vpu_write_reg(vpu->dev, CMD_ENC_PIC_PARA_BASE_ADDR, inst->report_vbuf.daddr);
	coda_hw_set_report_buf_addr_info(inst);

	if (!option->skip_picture) {
		vpu_write_reg(vpu->dev, CMD_ENC_PIC_SRC_INDEX,
			      source_frame->index + p_enc_info->frame_buf_num);
		vpu_write_reg(vpu->dev, CMD_ENC_PIC_SRC_STRIDE, source_frame->stride);
		vpu_write_reg(vpu->dev, CMD_ENC_PIC_SRC_ADDR_Y, source_frame->buf_y);
		vpu_write_reg(vpu->dev, CMD_ENC_PIC_SRC_ADDR_CB, source_frame->buf_cb);
		vpu_write_reg(vpu->dev, CMD_ENC_PIC_SRC_ADDR_CR, source_frame->buf_cr);
		vpu_write_reg(vpu->dev, CMD_ENC_PIC_SRC_BOTTOM_Y, source_frame->buf_y_bot);
		vpu_write_reg(vpu->dev, CMD_ENC_PIC_SRC_BOTTOM_CB, source_frame->buf_cb_bot);
		vpu_write_reg(vpu->dev, CMD_ENC_PIC_SRC_BOTTOM_CR, source_frame->buf_cr_bot);
	}

	reg_val = (option->field_run << 8) |
		  (REPORT_ENCODING_INFO << 3) |
		  (option->force_i_picture << 1) |
		  option->skip_picture;
	vpu_write_reg(vpu->dev, CMD_ENC_PIC_OPTION, reg_val);
	//vpu_write_reg(vpu->dev, CMD_ENC_PIC_SUB_FRAME_SYNC, 0);
	vpu_write_reg(vpu->dev, CMD_ENC_PIC_BB_START, p_enc_info->bitstream_buf);
	vpu_write_reg(vpu->dev, CMD_ENC_PIC_BB_SIZE, p_enc_info->bitstream_buf_size / 1024);

	reg_val = (p_enc_info->sec_axi.bit_enable << 0) |
		  (p_enc_info->sec_axi.ip_enable << 1) |
		  (p_enc_info->sec_axi.dbk_y_enable << 2) |
		  (p_enc_info->sec_axi.dbk_c_enable << 3) |
		  (p_enc_info->sec_axi.bit_enable << 8) |
		  (p_enc_info->sec_axi.ip_enable << 9) |
		  (p_enc_info->sec_axi.dbk_y_enable << 10) |
		  (p_enc_info->sec_axi.dbk_c_enable << 11);
	vpu_write_reg(vpu->dev, BIT_AXI_SRAM_USE, reg_val);

	vpu_write_reg(vpu->dev, BIT_WR_PTR, p_enc_info->stream_wr_ptr);
	vpu_write_reg(vpu->dev, BIT_RD_PTR, p_enc_info->stream_rd_ptr);
	vpu_write_reg(vpu->dev, BIT_BIT_STREAM_PARAM, p_enc_info->stream_end_flag);

	dev_dbg(vpu->dev, "[%d] ENC_PIC bs 0x%pad src[%d] 0x%pad src_end %d force %d\n",
		inst->id, &p_enc_info->bitstream_buf, source_frame->index, &source_frame->buf_y,
		p_enc_info->stream_end_flag, option->force_i_picture);

	reg_val = ENABLE_BWB | (p_enc_info->open_param.map_type << 9) | (inst->monochrome << 6) |
		  (inst->cbcr_interleave << 2) | VPU_FRAME_ENDIAN;
	vpu_write_reg(vpu->dev, BIT_FRAME_MEM_CTRL, reg_val);

	reg_val = (p_enc_info->open_param.en_line_buf_int << 6) | ENABLE_DYNAMIC_BUF_ALLOC |
		  ENABLE_BUF_PIC_RESET | VPU_STREAM_ENDIAN;
	vpu_write_reg(vpu->dev, BIT_BIT_STREAM_CTRL, reg_val);

	vpu_write_reg(vpu->dev, BIT_ME_LINEBUFFER_MODE, ME_LINEBUFFER_MODE);

	coda_hw_bit_issue_cmd(vpu, inst, PIC_RUN);

	return 0;
}

int coda_hw_enc_get_result(struct vpu_instance *inst, struct coda_enc_output_info *info)
{
	struct coda_enc_info *p_enc_info;
	struct vpu_device *vpu = inst->vpu_dev;

	if (!inst->codec_info)
		return -EINVAL;
	p_enc_info = &inst->codec_info->enc_info;

	if (vpu_read_reg(vpu->dev, RET_ENC_PIC_SUCCESS) != 1)
		return -EIO;

	p_enc_info->stream_end_flag = vpu_read_reg(vpu->dev, BIT_BIT_STREAM_PARAM);
	p_enc_info->stream_wr_ptr = vpu_read_reg(vpu->dev, BIT_WR_PTR);
	p_enc_info->stream_rd_ptr = vpu_read_reg(vpu->dev, BIT_RD_PTR) - p_enc_info->header_size;
	if (p_enc_info->header_size)
		p_enc_info->header_size = 0;

	info->encoding_success = 1;
	info->pic_type = vpu_read_reg(vpu->dev, RET_ENC_PIC_TYPE);
	info->num_of_slice = vpu_read_reg(vpu->dev, RET_ENC_PIC_SLICE_NUM);
	info->bitstream_wrap_around = vpu_read_reg(vpu->dev, RET_ENC_PIC_FLAG);
	info->recon_frame_index = vpu_read_reg(vpu->dev, RET_ENC_PIC_FRAME_IDX);
	info->frame_cycle = vpu_read_reg(vpu->dev, BIT_FRAME_CYCLE);
	info->avg_ctu_qp = vpu_read_reg(vpu->dev, RET_ENC_PIC_AVG_QP);
	info->enc_src_idx = p_enc_info->src_index;
	info->bitstream_buf = p_enc_info->stream_rd_ptr;
	info->bitstream_buf_size = p_enc_info->stream_wr_ptr - p_enc_info->stream_rd_ptr;
	info->rd_ptr = p_enc_info->stream_rd_ptr;
	info->wr_ptr = p_enc_info->stream_wr_ptr;

	coda_hw_get_report_buf_info(inst, info);

	return 0;
}

int coda_hw_enc_finish_seq(struct vpu_instance *inst, u32 *fail_res)
{
	struct vpu_device *vpu = inst->vpu_dev;
	int ret;

	coda_hw_bit_issue_cmd(vpu, inst, ENC_SEQ_END);
	ret = coda_hw_wait_vpu_busy(vpu, BIT_BUSY_FLAG);
	if (ret)
		return -ETIMEDOUT;

	return 0;
}
