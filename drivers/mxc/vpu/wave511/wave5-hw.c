// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave5 series multi-standard codec IP - wave5 backend logic
 *
 * Copyright (C) 2026 CHIPS&MEDIA INC
 */

#include <linux/iopoll.h>
#include <linux/bitfield.h>
#include "wave5-vpu.h"
#include "wave5-hw.h"
#include "wave5-regdefine.h"

#define FIO_TIMEOUT			10000000
#define FIO_CTRL_READY			BIT(31)
#define FIO_CTRL_WRITE			BIT(16)
#define QUEUE_REPORT_MASK		0xffff

/* Decoder support fields */
#define FEATURE_AVC10BIT_DEC		BIT(9)
#define FEATURE_AVC_DEC			BIT(8)
#define FEATURE_HEVC10BIT_DEC		BIT(1)
#define FEATURE_HEVC_DEC		BIT(0)

#define FEATURE_SCALER			BIT(7)
#define W521_FEATURE_BACKBONE		BIT(16)
#define W521_FEATURE_VCORE_BACKBONE	BIT(22)
#define W521_FEATURE_VCPU_BACKBONE	BIT(28)

#define REMAP_CTRL_MAX_SIZE_BITS	((W5_REMAP_MAX_SIZE >> 12) & 0x1ff)
#define REMAP_CTRL_REGISTER_VALUE(index)	(				\
	(BIT(31) | ((index) << 12) | BIT(11) | REMAP_CTRL_MAX_SIZE_BITS)	\
)

#define FASTIO_ADDRESS_MASK		GENMASK(15, 0)
#define SEQ_PARAM_PROFILE_MASK		GENMASK(30, 24)

static void _wave5_print_reg_err(struct vpu_device *vpu_dev, u32 reg_fail_reason,
				 const char *func);
#define PRINT_REG_ERR(dev, reason)	_wave5_print_reg_err((dev), (reason), __func__)

static inline const char *cmd_to_str(int cmd, bool is_dec)
{
	switch (cmd) {
	case W5_INIT_VPU:
		return "W5_INIT_VPU";
	case W5_WAKEUP_VPU:
		return "W5_WAKEUP_VPU";
	case W5_SLEEP_VPU:
		return "W5_SLEEP_VPU";
	case W5_CREATE_INSTANCE:
		return "W5_CREATE_INSTANCE";
	case W5_FLUSH_INSTANCE:
		return "W5_FLUSH_INSTANCE";
	case W5_DESTROY_INSTANCE:
		return "W5_DESTROY_INSTANCE";
	case W5_INIT_SEQ:
		return "W5_INIT_SEQ";
	case W5_SET_FB:
		return "W5_SET_FB";
	case W5_DEC_PIC:
		if (is_dec)
			return "W5_DEC_PIC";
		return "";
	case W5_QUERY:
		return "W5_QUERY";
	case W5_UPDATE_BS:
		return "W5_UPDATE_BS";
	case W5_MAX_VPU_COMD:
		return "W5_MAX_VPU_COMD";
	default:
		return "UNKNOWN";
	}
}

static void _wave5_print_reg_err(struct vpu_device *vpu_dev, u32 reg_fail_reason,
				 const char *func)
{
	struct device *dev = vpu_dev->dev;
	u32 reg_val;

	switch (reg_fail_reason) {
	case WAVE5_SYSERR_QUEUEING_FAIL:
		reg_val = vpu_read_reg(vpu_dev, W5_RET_QUEUE_FAIL_REASON);
		dev_dbg(dev, "%s: queueing failure: 0x%x\n", func, reg_val);
		break;
	case WAVE5_SYSERR_RESULT_NOT_READY:
		dev_err(dev, "%s: result not ready: 0x%x\n", func, reg_fail_reason);
		break;
	case WAVE5_SYSERR_ACCESS_VIOLATION_HW:
		dev_err(dev, "%s: access violation: 0x%x\n", func, reg_fail_reason);
		break;
	case WAVE5_SYSERR_WATCHDOG_TIMEOUT:
		dev_err(dev, "%s: watchdog timeout: 0x%x\n", func, reg_fail_reason);
		break;
	case WAVE5_SYSERR_BUS_ERROR:
		dev_err(dev, "%s: bus error: 0x%x\n", func, reg_fail_reason);
		break;
	case WAVE5_SYSERR_DOUBLE_FAULT:
		dev_err(dev, "%s: double fault: 0x%x\n", func, reg_fail_reason);
		break;
	case WAVE5_SYSERR_VPU_STILL_RUNNING:
		dev_dbg(dev, "%s: still running: 0x%x\n", func, reg_fail_reason);
		break;
	case WAVE5_SYSERR_VLC_BUF_FULL:
		dev_err(dev, "%s: vlc buf full: 0x%x\n", func, reg_fail_reason);
		break;
	case WAVE5_SYSERR_MAX_INST_CNT_EXCEED:
		dev_err(dev, "%s: inst cnt exceed: 0x%x\n", func, reg_fail_reason);
		break;
	default:
		dev_err(dev, "%s: failure:: 0x%x\n", func, reg_fail_reason);
		break;
	}
}

static int wave5_wait_fio_readl(struct vpu_device *vpu_dev, u32 addr, u32 val)
{
	u32 ctrl;
	int ret;

	ctrl = addr & 0xffff;
	wave5_vdi_write_register(vpu_dev, W5_VPU_FIO_CTRL_ADDR, ctrl);
	ret = read_poll_timeout(wave5_vdi_read_register, ctrl, ctrl & FIO_CTRL_READY,
				VPU_POLL_CHECK_INTERVAL, FIO_TIMEOUT, false,
				vpu_dev, W5_VPU_FIO_CTRL_ADDR);
	if (ret)
		return ret;

	if (wave5_vdi_read_register(vpu_dev, W5_VPU_FIO_DATA) != val)
		return -ETIMEDOUT;

	return 0;
}

static void wave5_fio_writel(struct vpu_device *vpu_dev, unsigned int addr, unsigned int data)
{
	int ret;
	unsigned int ctrl;

	wave5_vdi_write_register(vpu_dev, W5_VPU_FIO_DATA, data);
	ctrl = FIELD_GET(FASTIO_ADDRESS_MASK, addr);
	ctrl |= FIO_CTRL_WRITE;
	wave5_vdi_write_register(vpu_dev, W5_VPU_FIO_CTRL_ADDR, ctrl);
	ret = read_poll_timeout(wave5_vdi_read_register, ctrl, ctrl & FIO_CTRL_READY,
				VPU_POLL_CHECK_INTERVAL, FIO_TIMEOUT, false,
				vpu_dev, W5_VPU_FIO_CTRL_ADDR);
	if (ret)
		dev_dbg_ratelimited(vpu_dev->dev, "FIO write timeout: addr=0x%x data=%x\n",
				    ctrl, data);
}

static int wave5_wait_bus_busy(struct vpu_device *vpu_dev, unsigned int addr)
{
	u32 gdi_status_check_value = 0x3f;

	if (vpu_dev->product_code == WAVE515_CODE)
		gdi_status_check_value = 0x0738;
	if (vpu_dev->product_code == WAVE521C_CODE ||
	    vpu_dev->product_code == WAVE521_CODE ||
	    vpu_dev->product_code == WAVE521E1_CODE)
		gdi_status_check_value = 0x00ff1f3f;

	return wave5_wait_fio_readl(vpu_dev, addr, gdi_status_check_value);
}

static int wave5_wait_vpu_busy(struct vpu_device *vpu_dev, unsigned int addr)
{
	u32 data;

	return read_poll_timeout(wave5_vdi_read_register, data, data == 0,
				 VPU_POLL_CHECK_INTERVAL, VPU_BUSY_CHECK_TIMEOUT,
				 false, vpu_dev, addr);
}

static int wave5_wait_vcpu_bus_busy(struct vpu_device *vpu_dev, unsigned int addr)
{
	return wave5_wait_fio_readl(vpu_dev, addr, 0);
}

bool wave5_vpu_is_init(struct vpu_device *vpu_dev)
{
	return vpu_read_reg(vpu_dev, W5_VCPU_CUR_PC) != 0;
}

void wave5_vpu_check_state(struct vpu_device *vpu_dev)
{
	if (vpu_dev->ctrl) {
		int state = wave5_vpu_ctrl_get_state(vpu_dev->ctrl);

		if (state == WAVE5_VPU_STATE_PREPARE)
			wave5_vpu_ctrl_wait_done(vpu_dev->ctrl);
	} else {
		u32 val;
		int ret;

		ret = read_poll_timeout(vpu_read_reg, val, val != 0,
					VPU_POLL_CHECK_INTERVAL,
					VPU_BUSY_CHECK_TIMEOUT, false,
					vpu_dev, W5_VCPU_CUR_PC);
		if (!ret && vpu_dev->entity.on_boot)
			vpu_dev->entity.on_boot(vpu_dev->dev);
	}
}

unsigned int wave5_vpu_get_product_id(struct vpu_device *vpu_dev)
{
	u32 val = vpu_read_reg(vpu_dev, W5_VPU_RET_PRODUCT_VERSION);

	switch (val) {
	case WAVE515_CODE:
		return PRODUCT_ID_515;
	case WAVE517_CODE:
		return PRODUCT_ID_517;
	case WAVE521C_CODE:
		return PRODUCT_ID_521;
	case WAVE521_CODE:
	case WAVE521C_DUAL_CODE:
	case WAVE521E1_CODE:
	case WAVE511_CODE:
	case WAVE537_CODE:
		dev_err(vpu_dev->dev, "Unsupported product id (%x)\n", val);
		break;
	default:
		dev_err(vpu_dev->dev, "Invalid product id (%x)\n", val);
		break;
	}

	return PRODUCT_ID_NONE;
}

static void wave5_bit_issue_command(struct vpu_device *vpu_dev, struct vpu_instance *inst, u32 cmd)
{
	u32 instance_index;
	u32 codec_mode;

	if (inst) {
		instance_index = inst->id;
		codec_mode = inst->std;

		vpu_write_reg(vpu_dev, W5_CMD_INSTANCE_INFO, (codec_mode << 16) |
			      (instance_index & 0xffff));
		vpu_write_reg(vpu_dev, W5_VPU_BUSY_STATUS, 1);
	}

	vpu_write_reg(vpu_dev, W5_COMMAND, cmd);

	if (inst) {
		dev_dbg(vpu_dev->dev, "%s: cmd=0x%x (%s)\n", __func__, cmd,
			cmd_to_str(cmd, inst->type == VPU_INST_TYPE_DEC));
	} else {
		dev_dbg(vpu_dev->dev, "%s: cmd=0x%x\n", __func__, cmd);
	}

	vpu_write_reg(vpu_dev, W5_VPU_HOST_INT_REQ, 1);
}

static int wave5_vpu_firmware_command_queue_error_check(struct vpu_device *dev, u32 *fail_res)
{
	u32 reason = 0;

	/* Check if we were able to add a command into the VCPU QUEUE */
	if (!vpu_read_reg(dev, W5_RET_SUCCESS)) {
		reason = vpu_read_reg(dev, W5_RET_FAIL_REASON);
		PRINT_REG_ERR(dev, reason);

		/*
		 * The fail_res argument will be either NULL or 0.
		 * If the fail_res argument is NULL, then just return -EIO.
		 * Otherwise, assign the reason to fail_res, so that the
		 * calling function can use it.
		 */
		if (fail_res)
			*fail_res = reason;

		if (reason == WAVE5_SYSERR_VPU_STILL_RUNNING)
			return -EBUSY;

		return -EIO;
	}

	return 0;
}

static int send_firmware_command(struct vpu_instance *inst, u32 cmd, bool check_success,
				 u32 *queue_status, u32 *fail_result)
{
	int ret;

	wave5_bit_issue_command(inst->dev, inst, cmd);
	ret = wave5_wait_vpu_busy(inst->dev, W5_VPU_BUSY_STATUS);
	if (ret) {
		dev_warn(inst->dev->dev, "%s: command: '%s', timed out\n", __func__,
			 cmd_to_str(cmd, inst->type == VPU_INST_TYPE_DEC));
		return -ETIMEDOUT;
	}

	if (queue_status)
		*queue_status = vpu_read_reg(inst->dev, W5_RET_QUEUE_STATUS);

	/* In some cases we want to send multiple commands before checking
	 * whether they are queued properly
	 */
	if (!check_success)
		return 0;

	return wave5_vpu_firmware_command_queue_error_check(inst->dev, fail_result);
}

static int wave5_send_query(struct vpu_device *vpu_dev, struct vpu_instance *inst,
			    enum query_opt query_opt)
{
	int ret;

	vpu_write_reg(vpu_dev, W5_QUERY_OPTION, query_opt);
	vpu_write_reg(vpu_dev, W5_VPU_BUSY_STATUS, 1);
	wave5_bit_issue_command(vpu_dev, inst, W5_QUERY);

	ret = wave5_wait_vpu_busy(vpu_dev, W5_VPU_BUSY_STATUS);
	if (ret) {
		dev_warn(vpu_dev->dev, "command: 'W5_QUERY', timed out opt=0x%x\n", query_opt);
		return ret;
	}

	return wave5_vpu_firmware_command_queue_error_check(vpu_dev, NULL);
}

static void setup_wave5_interrupts(struct vpu_device *vpu_dev)
{
	u32 reg_val = 0;

	if (vpu_dev->attr.support_decoders) {
		/* Decoder interrupt */
		reg_val |= BIT(INT_WAVE5_INIT_SEQ);
		reg_val |= BIT(INT_WAVE5_DEC_PIC);
		reg_val |= BIT(INT_WAVE5_BSBUF_EMPTY);
		reg_val |= BIT(INT_WAVE5_REQ_WORK_BUF);
	}

	return vpu_write_reg(vpu_dev, W5_VPU_VINT_ENABLE, reg_val);
}

static int setup_wave5_properties(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	struct vpu_attr *p_attr = &vpu_dev->attr;
	u32 reg_val;
	u8 *str;
	int ret;
	u32 hw_config_def0, hw_config_def1, hw_config_feature;

	ret = wave5_send_query(vpu_dev, NULL, GET_VPU_INFO);
	if (ret)
		return ret;

	reg_val = vpu_read_reg(vpu_dev, W5_RET_PRODUCT_NAME);
	str = (u8 *)&reg_val;
	p_attr->product_name[0] = str[3];
	p_attr->product_name[1] = str[2];
	p_attr->product_name[2] = str[1];
	p_attr->product_name[3] = str[0];
	p_attr->product_name[4] = 0;

	p_attr->product_id = wave5_vpu_get_product_id(vpu_dev);
	p_attr->product_version = vpu_read_reg(vpu_dev, W5_RET_PRODUCT_VERSION);
	p_attr->fw_version = vpu_read_reg(vpu_dev, W5_RET_FW_VERSION);
	p_attr->fw_api_version = vpu_read_reg(vpu_dev, W5_RET_FW_API_VERSION);
	p_attr->customer_id = vpu_read_reg(vpu_dev, W5_RET_CUSTOMER_ID);
	hw_config_def0 = vpu_read_reg(vpu_dev, W5_RET_STD_DEF0);
	hw_config_def1 = vpu_read_reg(vpu_dev, W5_RET_STD_DEF1);
	hw_config_feature = vpu_read_reg(vpu_dev, W5_RET_CONF_FEATURE);

	p_attr->support_decoders = FIELD_GET(FEATURE_AVC_DEC, hw_config_feature) << STD_AVC;
	p_attr->support_decoders |= FIELD_GET(FEATURE_HEVC_DEC, hw_config_feature) << STD_HEVC;
	p_attr->support_avc10bit_dec = FIELD_GET(FEATURE_AVC10BIT_DEC, hw_config_feature);
	p_attr->support_hevc10bit_dec = FIELD_GET(FEATURE_HEVC10BIT_DEC, hw_config_feature);
	if (vpu_dev->product_code != WAVE515_CODE) {
		p_attr->support_backbone = FIELD_GET(W521_FEATURE_BACKBONE,
						     hw_config_def0);
		p_attr->support_vcpu_backbone = FIELD_GET(W521_FEATURE_VCPU_BACKBONE,
							  hw_config_def0);
		p_attr->support_vcore_backbone = FIELD_GET(W521_FEATURE_VCORE_BACKBONE,
							   hw_config_def0);
	}
	p_attr->support_scaler = FIELD_GET(FEATURE_SCALER, hw_config_def0);

	setup_wave5_interrupts(vpu_dev);

	return 0;
}

int wave5_vpu_get_version(struct vpu_device *vpu_dev, u32 *revision)
{
	struct vpu_attr *p_attr = &vpu_dev->attr;
	int ret;

	ret = setup_wave5_properties(vpu_dev->dev);
	if (ret)
		return ret;

	if (revision) {
		*revision = p_attr->fw_version;
		return 0;
	}

	return -EINVAL;
}

/*
 * Firmware may use virtual command, whose depth is WAVE5_MAX_VIRT_QUE_DEPTH.
 * so the total CQ size should be cq_depth() + WAVE5_MAX_VIRT_QUE_DEPTH.
 */
static u32 wave5_vpu_cq_size(struct vpu_device *dev)
{
	return wave5_vpu_cq_depth(dev) + WAVE5_MAX_VIRT_QUE_DEPTH;
}

int wave5_vpu_build_up_dec_param(struct vpu_instance *inst,
				 struct dec_open_param *param)
{
	int ret;
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	struct vpu_device *vpu_dev = inst->dev;

	p_dec_info->cycle_per_tick = 256;
	p_dec_info->sec_axi_info.use_bit_enable = true;
	p_dec_info->sec_axi_info.use_ip_enable = true;
	p_dec_info->sec_axi_info.use_lf_row_enable = true;
	if (inst->dev->attr.support_scaler)
		p_dec_info->sec_axi_info.use_scaler_enable = true;

	switch (inst->std) {
	case W_HEVC_DEC:
		p_dec_info->seq_change_mask = SEQ_CHANGE_ENABLE_ALL_HEVC;
		p_dec_info->user_data_enable = BIT(10) | BIT(15);
		break;
	case W_AVC_DEC:
		p_dec_info->seq_change_mask = SEQ_CHANGE_ENABLE_ALL_AVC;
		break;
	default:
		return -EINVAL;
	}

	if (inst->dev->product_code != WAVE515_CODE)
		vpu_write_reg(inst->dev, W5_CMD_DEC_VCORE_INFO, 1);

	vpu_write_reg(inst->dev, W5_CMD_ADDR_TEMP_BASE, vpu_dev->temp_vbuf.daddr);
	vpu_write_reg(inst->dev, W5_CMD_TEMP_SIZE, vpu_dev->temp_vbuf.size);
	vpu_write_reg(inst->dev, W5_CMD_DEC_BS_START_ADDR, 0);
	vpu_write_reg(inst->dev, W5_CMD_DEC_BS_SIZE, 0);
	vpu_write_reg(inst->dev, W5_CMD_ERR_CONCEAL, 0);
	vpu_write_reg(inst->dev, W5_CMD_USER_DEFINED_ID, 0);

	/* NOTE: SDMA reads MSB first */
	vpu_write_reg(inst->dev, W5_CMD_BS_PARAM, BITSTREAM_ENDIANNESS_BIG_ENDIAN);

	if (inst->dev->product_code != WAVE515_CODE) {
		/* This register must be reset explicitly */
		vpu_write_reg(inst->dev, W5_CMD_EXT_ADDR, 0);
		vpu_write_reg(inst->dev, W5_CMD_NUM_CQ_DEPTH_M1,
			      wave5_vpu_cq_size(inst->dev) - 1);
	}

	ret = send_firmware_command(inst, W5_CREATE_INSTANCE, true, NULL, NULL);
	if (ret)
		return ret;

	inst->id = vpu_read_reg(inst->dev, W5_RET_INSTANCE_ID);
	inst->sram_size = vpu_read_reg(inst->dev, W5_RET_SEC_AXI_SIZE);
	dev_dbg(inst->dev->dev, "inst id = %d, sram size = 0x%x\n", inst->id, inst->sram_size);

	return 0;
}

int wave5_vpu_hw_flush_instance(struct vpu_instance *inst)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	u32 instance_queue_count, report_queue_count;
	u32 reg_val = 0;
	u32 fail_res = 0;
	int ret;

	ret = send_firmware_command(inst, W5_FLUSH_INSTANCE, true, &reg_val, &fail_res);
	if (ret)
		return ret;

	instance_queue_count = (reg_val >> 16) & 0xff;
	report_queue_count = (reg_val & QUEUE_REPORT_MASK);
	if (instance_queue_count != 0 || report_queue_count != 0) {
		dev_warn(inst->dev->dev,
			 "FLUSH_INSTANCE cmd didn't reset the amount of queued commands & reports");
	}

	/* reset our local copy of the counts */
	p_dec_info->instance_queue_count = 0;
	p_dec_info->report_queue_count = 0;

	return 0;
}

static u32 get_bitstream_options(struct dec_info *info)
{
	u32 bs_option = BSOPTION_ENABLE_EXPLICIT_END | BSOPTION_RD_PTR_VALID_FLAG;

	if (info->stream_endflag)
		bs_option |= BSOPTION_HIGHLIGHT_STREAM_END;
	return bs_option;
}

int wave5_vpu_dec_init_seq(struct vpu_instance *inst)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	u32 bs_option, cmd_option = INIT_SEQ_NORMAL;
	u32 reg_val, fail_res;
	int ret;

	dev_dbg(inst->dev->dev, "[%d] INIT_SEQ, rd_ptr %pad, wr_ptr %pad\n",
		inst->id, &p_dec_info->stream_rd_ptr, &p_dec_info->stream_wr_ptr);
	vpu_write_reg(inst->dev, W5_BS_RD_PTR, p_dec_info->stream_rd_ptr);
	vpu_write_reg(inst->dev, W5_BS_WR_PTR, p_dec_info->stream_wr_ptr);

	bs_option = get_bitstream_options(p_dec_info);

	/* Without RD_PTR_VALID_FLAG Wave515 ignores RD_PTR value */
	if (inst->dev->product_code == WAVE515_CODE)
		bs_option |= BSOPTION_RD_PTR_VALID_FLAG;

	vpu_write_reg(inst->dev, W5_BS_OPTION, bs_option);

	vpu_write_reg(inst->dev, W5_COMMAND_OPTION, cmd_option);
	vpu_write_reg(inst->dev, W5_CMD_DEC_USER_MASK, p_dec_info->user_data_enable);
	vpu_write_reg(inst->dev, W5_CMD_DEC_USER_BASE, 0);
	vpu_write_reg(inst->dev, W5_CMD_DEC_USER_SIZE, 0);
	vpu_write_reg(inst->dev, W5_CMD_DEC_USER_PARAM, 0);

	ret = send_firmware_command(inst, W5_INIT_SEQ, true, &reg_val, &fail_res);
	if (ret)
		return ret;

	p_dec_info->instance_queue_count = (reg_val >> 16) & 0xff;
	p_dec_info->report_queue_count = (reg_val & QUEUE_REPORT_MASK);

	dev_dbg(inst->dev->dev, "%s: init seq sent (queue %u : %u)\n", __func__,
		p_dec_info->instance_queue_count, p_dec_info->report_queue_count);

	return 0;
}

static void wave5_vpu_dec_get_hdr10_info(struct vpu_instance *inst,
					 struct v4l2_ctrl_hdr10_cll_info *hdr10_cll,
					 struct v4l2_ctrl_hdr10_mastering_display *hdr10_display)
{
	u32 reg_val;

	if (hdr10_cll) {
		reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_HDR10_CLL);
		hdr10_cll->max_content_light_level = FIELD_GET(GENMASK_U32(31, 16), reg_val);
		hdr10_cll->max_pic_average_light_level = FIELD_GET(GENMASK_U32(15, 0), reg_val);
	}

	if (!hdr10_display)
		return;

	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_HDR10_MD_PRIMARIES_G);
	hdr10_display->display_primaries_x[0] = FIELD_GET(GENMASK_U32(31, 16), reg_val);
	hdr10_display->display_primaries_y[0] = FIELD_GET(GENMASK_U32(15, 0), reg_val);
	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_HDR10_MD_PRIMARIES_B);
	hdr10_display->display_primaries_x[1] = FIELD_GET(GENMASK_U32(31, 16), reg_val);
	hdr10_display->display_primaries_y[1] = FIELD_GET(GENMASK_U32(15, 0), reg_val);
	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_HDR10_MD_PRIMARIES_R);
	hdr10_display->display_primaries_x[2] = FIELD_GET(GENMASK_U32(31, 16), reg_val);
	hdr10_display->display_primaries_y[2] = FIELD_GET(GENMASK_U32(15, 0), reg_val);
	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_HDR10_MD_WHITE);
	hdr10_display->white_point_x = FIELD_GET(GENMASK_U32(31, 16), reg_val);
	hdr10_display->white_point_y = FIELD_GET(GENMASK_U32(15, 0), reg_val);
	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_HDR10_MD_MAX_LUM);
	hdr10_display->max_display_mastering_luminance = reg_val;
	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_HDR10_MD_MIN_LUM);
	hdr10_display->min_display_mastering_luminance = reg_val;
}

static void wave5_get_dec_seq_result(struct vpu_instance *inst, struct dec_initial_info *info)
{
	u32 reg_val;
	u32 profile_compatibility_flag;
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;

	p_dec_info->stream_rd_ptr = wave5_dec_get_rd_ptr(inst);
	info->rd_ptr = p_dec_info->stream_rd_ptr;

	p_dec_info->frame_display_flag = vpu_read_reg(inst->dev, W5_RET_DEC_DISP_IDC);

	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_PIC_SIZE);
	info->pic_width = ((reg_val >> 16) & 0xffff);
	info->pic_height = (reg_val & 0xffff);
	info->min_frame_buffer_count = vpu_read_reg(inst->dev, W5_RET_DEC_NUM_REQUIRED_FB);
	info->reorder_delay = vpu_read_reg(inst->dev, W5_RET_DEC_NUM_REORDER_DELAY);

	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_CROP_LEFT_RIGHT);
	info->pic_crop_rect.left = (reg_val >> 16) & 0xffff;
	info->pic_crop_rect.right = reg_val & 0xffff;
	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_CROP_TOP_BOTTOM);
	info->pic_crop_rect.top = (reg_val >> 16) & 0xffff;
	info->pic_crop_rect.bottom = reg_val & 0xffff;

	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_COLOR_SAMPLE_INFO);
	info->luma_bitdepth = reg_val & 0xf;
	info->chroma_bitdepth = (reg_val >> 4) & 0xf;
	info->c_fmt_idc = (reg_val >> 8) & 0xf;

	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_SEQ_PARAM);
	profile_compatibility_flag = (reg_val >> 12) & 0xff;
	info->profile = (reg_val >> 24) & 0x1f;
	info->hevc_vps_extension_flag = (reg_val >> 20) & 0x1;

	if (inst->std == W_HEVC_DEC) {
		/* guessing profile */
		if (!info->profile) {
			if ((profile_compatibility_flag & 0x06) == 0x06)
				info->profile = HEVC_PROFILE_MAIN; /* main profile */
			else if (profile_compatibility_flag & 0x04)
				info->profile = HEVC_PROFILE_MAIN10; /* main10 profile */
			else if (profile_compatibility_flag & 0x08)
				/* main still picture profile */
				info->profile = HEVC_PROFILE_STILLPICTURE;
			else
				info->profile = HEVC_PROFILE_MAIN; /* for old version HM */
		}
	} else if (inst->std == W_AVC_DEC) {
		info->profile = FIELD_GET(SEQ_PARAM_PROFILE_MASK, reg_val);
	}

	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_VUI_INFO);
	if (reg_val) {
		info->color.video_signal_type_present = (reg_val >> 26) & 0x1;
		info->color.color_range = (reg_val >> 25) & 0x1;
		info->color.color_description_present = (reg_val >> 24) & 0x1;
		info->color.color_primaries = (reg_val >> 16) & 0xFF;
		info->color.transfer_characteristics = (reg_val >> 8) & 0xFF;
		info->color.matrix_coefficients = reg_val & 0xFF;
	}

	if (inst->dev->product_code != WAVE515_CODE) {
		info->vlc_buf_size = vpu_read_reg(inst->dev, W5_RET_VLC_BUF_SIZE);
		info->param_buf_size = vpu_read_reg(inst->dev, W5_RET_PARAM_BUF_SIZE);
		info->vlc_buf_size = ALIGN(info->vlc_buf_size, TASK_BUF_ALIGNMENT);
		info->param_buf_size = ALIGN(info->param_buf_size, TASK_BUF_ALIGNMENT);
		p_dec_info->vlc_buf_size = info->vlc_buf_size;
		p_dec_info->param_buf_size = info->param_buf_size;
	}

	wave5_vpu_dec_get_hdr10_info(inst, &info->hdr10_cll_info, &info->hdr10_mastering_display);
}

int wave5_vpu_dec_get_seq_info(struct vpu_instance *inst, struct dec_initial_info *info)
{
	int ret;
	u32 reg_val;
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;

	vpu_write_reg(inst->dev, W5_CMD_DEC_ADDR_REPORT_BASE, p_dec_info->user_data_buf_addr);
	vpu_write_reg(inst->dev, W5_CMD_DEC_REPORT_SIZE, p_dec_info->user_data_buf_size);
	vpu_write_reg(inst->dev, W5_CMD_DEC_REPORT_PARAM, REPORT_PARAM_ENDIANNESS_BIG_ENDIAN);

	/* send QUERY cmd */
	ret = wave5_send_query(inst->dev, inst, GET_RESULT);
	if (ret)
		return ret;

	reg_val = vpu_read_reg(inst->dev, W5_RET_QUEUE_STATUS);

	p_dec_info->instance_queue_count = (reg_val >> 16) & 0xff;
	p_dec_info->report_queue_count = (reg_val & QUEUE_REPORT_MASK);

	dev_dbg(inst->dev->dev, "%s: init seq complete (queue %u : %u)\n", __func__,
		p_dec_info->instance_queue_count, p_dec_info->report_queue_count);

	/* this is not a fatal error, set ret to -EIO but don't return immediately */
	if (vpu_read_reg(inst->dev, W5_RET_DEC_DECODING_SUCCESS) != 1) {
		info->err_reason = vpu_read_reg(inst->dev, W5_RET_DEC_ERR_INFO);
		ret = -EIO;
	} else {
		info->warn_info = vpu_read_reg(inst->dev, W5_RET_DEC_WARN_INFO);
	}

	wave5_get_dec_seq_result(inst, info);

	return ret;
}

u32 wave5_vpu_dec_calc_task_buf_size(struct vpu_instance *inst)
{
	struct dec_info *p_dec_info;

	if (!inst || !inst->codec_info)
		return 0;

	if (inst->dev->product_code == WAVE515_CODE)
		return 0;

	p_dec_info = &inst->codec_info->dec_info;
	return (p_dec_info->vlc_buf_size * VLC_BUF_NUM) + SZ_64K +
	       (p_dec_info->param_buf_size * wave5_vpu_cq_size(inst->dev));
}

int wave5_vpu_dec_register_framebuffer(struct vpu_instance *inst,
				       struct frame_buffer *fb_arr,
				       unsigned int count)
{
	int ret;
	struct dec_info *p_dec_info;
	struct dec_initial_info *init_info;
	size_t remain, idx, j, i, cnt_8_chunk;
	u32 start_no, end_no;
	u32 reg_val, pic_size;
	u32 addr_y, addr_cb, addr_cr;
	bool justified = WTL_RIGHT_JUSTIFIED;
	u32 format_no = WTL_PIXEL_8BIT;
	u32 pixel_order = 1;

	if (!inst || !inst->codec_info)
		return -EINVAL;

	p_dec_info = &inst->codec_info->dec_info;
	init_info = &p_dec_info->initial_info;

	if (inst->dev->product_code != WAVE515_CODE) {
		vpu_write_reg(inst->dev, W5_CMD_SET_FB_ADDR_TASK_BUF,
			      p_dec_info->vb_task.daddr);
		vpu_write_reg(inst->dev, W5_CMD_SET_FB_TASK_BUF_SIZE,
			      p_dec_info->vb_task.size);
	}

	pic_size = (init_info->pic_width << 16) | (init_info->pic_height);
	vpu_write_reg(inst->dev, W5_PIC_SIZE, pic_size);

	reg_val = (pixel_order << 23) |
		  (justified << 22) |
		  (format_no << 20) |
		  (fb_arr[0].stride);
	vpu_write_reg(inst->dev, W5_COMMON_PIC_INFO, reg_val);

	remain = count;
	cnt_8_chunk = DIV_ROUND_UP(count, 8);
	idx = 0;
	for (j = 0; j < cnt_8_chunk; j++) {
		reg_val = (j == cnt_8_chunk - 1) << 4 | ((j == 0) << 3);
		vpu_write_reg(inst->dev, W5_SFB_OPTION, reg_val);
		start_no = j * 8;
		end_no = start_no + ((remain >= 8) ? 8 : remain) - 1;

		vpu_write_reg(inst->dev, W5_SET_FB_NUM, (start_no << 8) | end_no);

		for (i = 0; i < 8 && i < remain; i++) {
			addr_y = fb_arr[i + start_no].buf_y;
			addr_cb = fb_arr[i + start_no].buf_cb;
			addr_cr = fb_arr[i + start_no].buf_cr;
			vpu_write_reg(inst->dev, W5_ADDR_LUMA_BASE0 + (i << 4), addr_y);
			vpu_write_reg(inst->dev, W5_ADDR_CB_BASE0 + (i << 4), addr_cb);
			/* luma FBC offset table */
			vpu_write_reg(inst->dev, W5_ADDR_FBC_Y_OFFSET0 + (i << 4),
				      p_dec_info->vb_fbc_y_tbl[idx].daddr);
			/* chroma FBC offset table */
			vpu_write_reg(inst->dev, W5_ADDR_FBC_C_OFFSET0 + (i << 4),
				      p_dec_info->vb_fbc_c_tbl[idx].daddr);
			vpu_write_reg(inst->dev, W5_ADDR_MV_COL0 + (i << 2),
				      p_dec_info->vb_mv[idx].daddr);
			idx++;
		}
		remain -= i;

		ret = send_firmware_command(inst, W5_SET_FB, false, NULL, NULL);
		if (ret)
			return ret;
	}

	reg_val = vpu_read_reg(inst->dev, W5_RET_SUCCESS);
	if (!reg_val)
		return -EIO;

	return 0;
}

int wave5_vpu_dec_register_displaybuffer(struct vpu_instance *inst,
					 struct frame_buffer *fb_arr)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	struct dec_initial_info *init_info = &p_dec_info->initial_info;
	int ret;
	u32 reg_val;
	bool justified = WTL_RIGHT_JUSTIFIED;
	u32 format_no = WTL_PIXEL_8BIT;
	u32 color_format = 0;
	u32 pixel_order = 1;
	u32 bwb_flag = 1;
	u32 scaler_flag = inst->scaler_info.enable;
	bool update_fb = p_dec_info->disp_buf[fb_arr->index].update_fb_info;

	reg_val = (init_info->pic_width << 16) | (init_info->pic_height);
	if (inst->scaler_info.enable)
		reg_val = (inst->scaler_info.width << 16) | (inst->scaler_info.height);
	vpu_write_reg(inst->dev, W5_PIC_SIZE, reg_val);

	if (inst->output_format == FORMAT_422 ||
	    inst->output_format == FORMAT_422_P10_16BIT_MSB ||
	    inst->output_format == FORMAT_422_P10_16BIT_LSB ||
	    inst->output_format == FORMAT_422_P10_32BIT_MSB ||
	    inst->output_format == FORMAT_422_P10_32BIT_LSB)
		color_format = 1;

	switch (inst->output_format) {
	case FORMAT_420_P10_16BIT_MSB:
	case FORMAT_422_P10_16BIT_MSB:
		justified = WTL_RIGHT_JUSTIFIED;
		format_no = WTL_PIXEL_16BIT;
		break;
	case FORMAT_420_P10_16BIT_LSB:
	case FORMAT_422_P10_16BIT_LSB:
		justified = WTL_LEFT_JUSTIFIED;
		format_no = WTL_PIXEL_16BIT;
		break;
	case FORMAT_420_P10_32BIT_MSB:
	case FORMAT_422_P10_32BIT_MSB:
		justified = WTL_RIGHT_JUSTIFIED;
		format_no = WTL_PIXEL_32BIT;
		break;
	case FORMAT_420_P10_32BIT_LSB:
	case FORMAT_422_P10_32BIT_LSB:
		justified = WTL_LEFT_JUSTIFIED;
		format_no = WTL_PIXEL_32BIT;
		break;
	default:
		break;
	}

	if (update_fb)
		reg_val = fb_arr->stride << 16;
	else
		reg_val = (scaler_flag << 29) |
			  (bwb_flag << 28) |
			  (pixel_order << 23) |
			  (justified << 22) |
			  (format_no << 20) |
			  (color_format << 19) |
			  (inst->nv21 << 17) |
			  (inst->cbcr_interleave << 16) |
			  (fb_arr->stride);
	vpu_write_reg(inst->dev, W5_COMMON_PIC_INFO, reg_val);

	reg_val = (1 << 4) |
		  ((!p_dec_info->num_of_display_fbs && !update_fb) << 3) |
		  update_fb;
	vpu_write_reg(inst->dev, W5_SFB_OPTION, reg_val);

	if (update_fb)
		reg_val = (0xFF << 16) | (fb_arr->index << 8) | 0xFF;
	else
		reg_val = (fb_arr->index << 8) | fb_arr->index;
	vpu_write_reg(inst->dev, W5_SET_FB_NUM, reg_val);

	vpu_write_reg(inst->dev, W5_ADDR_LUMA_BASE0, fb_arr->buf_y);
	vpu_write_reg(inst->dev, W5_ADDR_CB_BASE0, fb_arr->buf_cb);
	vpu_write_reg(inst->dev, W5_ADDR_CR_BASE0, fb_arr->buf_cr);
	vpu_write_reg(inst->dev, W5_ADDR_FBC_C_OFFSET0, 0);
	vpu_write_reg(inst->dev, W5_ADDR_MV_COL0, 0);
	vpu_write_reg(inst->dev, W5_ADDR_FBC_Y_BASE, 0);
	vpu_write_reg(inst->dev, W5_ADDR_FBC_C_BASE, 0);
	vpu_write_reg(inst->dev, W5_ADDR_FBC_Y_OFFSET, 0);
	vpu_write_reg(inst->dev, W5_ADDR_FBC_C_OFFSET, 0);

	ret = send_firmware_command(inst, W5_SET_FB, true, NULL, NULL);
	if (ret)
		return ret;

	if (!update_fb)
		p_dec_info->num_of_display_fbs++;

	p_dec_info->disp_buf[fb_arr->index] = *fb_arr;

	return 0;
}

static u32 wave5_vpu_dec_validate_sec_axi(struct vpu_instance *inst)
{
	u32 bitdepth = inst->codec_info->dec_info.initial_info.luma_bitdepth;
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	u32 bit_size = 0, ip_size = 0, lf_size = 0, scl_size = 0, ret = 0;
	u32 sram_size = inst->sram_size;
	u32 width = inst->src_fmt.width;

	if (!sram_size)
		return 0;

	if (inst->std == W_AVC_DEC && bitdepth == 8) {
		ip_size = ALIGN(width, 16) * 2;
		lf_size = ALIGN(width, 32) * 4 + ALIGN(width, 32) * 2;
	} else if (inst->std == W_AVC_DEC && bitdepth == 10) {
		ip_size = ALIGN((ALIGN(width, 16) * 10), 128) / 4;
		lf_size = ALIGN(width, 32) * 5 + ALIGN(width, 32) * 3;
	} else if (inst->std == W_HEVC_DEC && bitdepth == 8) {
		bit_size = ALIGN((ALIGN(width, 64) / 4 * 9), 16);
		ip_size = ALIGN(width, 16) * 2;
		lf_size = ALIGN(width, 32) * 5 + ALIGN(width, 32) * 3;
	} else if (inst->std == W_HEVC_DEC && bitdepth == 10) {
		bit_size = ALIGN((ALIGN(width, 64) / 4 * 9), 16);
		ip_size = ALIGN((ALIGN(width, 16) * 10), 128) / 4;
		lf_size = ALIGN(width, 32) * 7 + ALIGN(width, 32) * 4;
	} else {
		return 0;
	}
	scl_size = ALIGN(width, 512) * 5;

	if (p_dec_info->sec_axi_info.use_bit_enable && sram_size >= bit_size) {
		ret |= BIT(0);
		sram_size -= bit_size;
	}

	if (p_dec_info->sec_axi_info.use_ip_enable && sram_size >= ip_size) {
		ret |= BIT(9);
		sram_size -= ip_size;
	}

	if (p_dec_info->sec_axi_info.use_lf_row_enable && sram_size >= lf_size) {
		ret |= BIT(15);
		sram_size -= lf_size;
	}

	if (p_dec_info->sec_axi_info.use_scaler_enable && sram_size >= scl_size) {
		ret |= BIT(5);
		sram_size -= scl_size;
	}

	dev_dbg(inst->dev->dev,
		"sec_axi ret = 0x%x, sram_size 0x%x -> 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
		ret, inst->sram_size, sram_size, bit_size, ip_size, lf_size, scl_size);
	return ret;
}

int wave5_vpu_decode(struct vpu_instance *inst, u32 *fail_res)
{
	u32 reg_val;
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret;

	dev_dbg(inst->dev->dev, "[%d] DEC_PIC, rd_ptr %pad, wr_ptr %pad\n",
		inst->id, &p_dec_info->stream_rd_ptr, &p_dec_info->stream_wr_ptr);
	vpu_write_reg(inst->dev, W5_BS_RD_PTR, p_dec_info->stream_rd_ptr);
	vpu_write_reg(inst->dev, W5_BS_WR_PTR, p_dec_info->stream_wr_ptr);

	vpu_write_reg(inst->dev, W5_BS_OPTION, get_bitstream_options(p_dec_info));

	/* secondary AXI */
	reg_val = wave5_vpu_dec_validate_sec_axi(inst);
	vpu_write_reg(inst->dev, W5_USE_SEC_AXI, reg_val);

	/* set attributes of user buffer */
	vpu_write_reg(inst->dev, W5_CMD_DEC_USER_MASK, p_dec_info->user_data_enable);
	vpu_write_reg(inst->dev, W5_CMD_DEC_USER_BASE, 0);
	vpu_write_reg(inst->dev, W5_CMD_DEC_USER_SIZE, 0);
	vpu_write_reg(inst->dev, W5_CMD_DEC_USER_PARAM, 0);

	vpu_write_reg(inst->dev, W5_COMMAND_OPTION, DEC_PIC_NORMAL);
	vpu_write_reg(inst->dev, W5_CMD_DEC_TEMPORAL_ID_PLUS1,
		      (p_dec_info->target_spatial_id << 9) |
		      (p_dec_info->temp_id_select_mode << 8) | p_dec_info->target_temp_id);
	vpu_write_reg(inst->dev, W5_CMD_SEQ_CHANGE_ENABLE_FLAG, p_dec_info->seq_change_mask);
	/* When reordering is disabled we force the latency of the framebuffers */
	vpu_write_reg(inst->dev, W5_CMD_DEC_FORCE_FB_LATENCY_PLUS1, !p_dec_info->reorder_enable);

	ret = send_firmware_command(inst, W5_DEC_PIC, true, &reg_val, fail_res);
	if (ret == -ETIMEDOUT)
		return ret;

	p_dec_info->instance_queue_count = (reg_val >> 16) & 0xff;
	p_dec_info->report_queue_count = (reg_val & QUEUE_REPORT_MASK);

	dev_dbg(inst->dev->dev, "%s: dec pic sent (queue %u : %u)\n", __func__,
		p_dec_info->instance_queue_count, p_dec_info->report_queue_count);

	if (ret)
		return ret;

	return 0;
}

int wave5_vpu_dec_get_result(struct vpu_instance *inst, struct dec_output_info *result)
{
	int ret;
	u32 index, nal_unit_type, reg_val, sub_layer_info;
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	struct vpu_device *vpu_dev = inst->dev;
	u32 decode_start_tick;

	vpu_write_reg(inst->dev, W5_CMD_DEC_ADDR_REPORT_BASE, p_dec_info->user_data_buf_addr);
	vpu_write_reg(inst->dev, W5_CMD_DEC_REPORT_SIZE, p_dec_info->user_data_buf_size);
	vpu_write_reg(inst->dev, W5_CMD_DEC_REPORT_PARAM, REPORT_PARAM_ENDIANNESS_BIG_ENDIAN);

	/* send QUERY cmd */
	ret = wave5_send_query(vpu_dev, inst, GET_RESULT);
	if (ret)
		return ret;

	reg_val = vpu_read_reg(inst->dev, W5_RET_QUEUE_STATUS);

	p_dec_info->instance_queue_count = (reg_val >> 16) & 0xff;
	p_dec_info->report_queue_count = (reg_val & QUEUE_REPORT_MASK);

	dev_dbg(inst->dev->dev, "%s: dec pic complete (queue %u : %u)\n", __func__,
		p_dec_info->instance_queue_count, p_dec_info->report_queue_count);

	if (vpu_read_reg(inst->dev, W5_RET_DEC_DECODING_SUCCESS) != 1)
		result->err_reason = vpu_read_reg(inst->dev, W5_RET_DEC_ERR_INFO);
	else
		result->warn_info = vpu_read_reg(inst->dev, W5_RET_DEC_WARN_INFO);

	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_PIC_TYPE);

	nal_unit_type = (reg_val >> 4) & 0x3f;

	if (inst->std == W_HEVC_DEC) {
		if (reg_val & 0x04)
			result->pic_type = PIC_TYPE_B;
		else if (reg_val & 0x02)
			result->pic_type = PIC_TYPE_P;
		else if (reg_val & 0x01)
			result->pic_type = PIC_TYPE_I;
		else
			result->pic_type = PIC_TYPE_MAX;
		if ((nal_unit_type == 19 || nal_unit_type == 20) && result->pic_type == PIC_TYPE_I)
			/* IDR_W_RADL, IDR_N_LP */
			result->pic_type = PIC_TYPE_IDR;
	} else if (inst->std == W_AVC_DEC) {
		if (reg_val & 0x04)
			result->pic_type = PIC_TYPE_B;
		else if (reg_val & 0x02)
			result->pic_type = PIC_TYPE_P;
		else if (reg_val & 0x01)
			result->pic_type = PIC_TYPE_I;
		else
			result->pic_type = PIC_TYPE_MAX;
		if (nal_unit_type == 5 && result->pic_type == PIC_TYPE_I)
			result->pic_type = PIC_TYPE_IDR;
	}
	index = vpu_read_reg(inst->dev, W5_RET_DEC_DISPLAY_INDEX);
	result->index_frame_display = index;
	index = vpu_read_reg(inst->dev, W5_RET_DEC_DECODED_INDEX);
	result->index_frame_decoded = index;
	result->index_frame_decoded_for_tiled = index;

	sub_layer_info = vpu_read_reg(inst->dev, W5_RET_DEC_SUB_LAYER_INFO);
	result->temporal_id = sub_layer_info & 0x7;

	if (inst->std == W_HEVC_DEC || inst->std == W_AVC_DEC) {
		result->decoded_poc = -1;
		if (result->index_frame_decoded >= 0 ||
		    result->index_frame_decoded == DECODED_IDX_FLAG_SKIP)
			result->decoded_poc = vpu_read_reg(inst->dev, W5_RET_DEC_PIC_POC);
	}

	result->sequence_changed = vpu_read_reg(inst->dev, W5_RET_DEC_NOTIFICATION);
	reg_val = vpu_read_reg(inst->dev, W5_RET_DEC_PIC_SIZE);
	result->dec_pic_width = reg_val >> 16;
	result->dec_pic_height = reg_val & 0xffff;

	if (result->sequence_changed) {
		memcpy((void *)&p_dec_info->new_seq_info, (void *)&p_dec_info->initial_info,
		       sizeof(struct dec_initial_info));
		wave5_get_dec_seq_result(inst, &p_dec_info->new_seq_info);
	}

	result->dec_host_cmd_tick = vpu_read_reg(inst->dev, W5_RET_DEC_HOST_CMD_TICK);
	result->dec_seek_start_tick = vpu_read_reg(inst->dev, W5_RET_DEC_SEEK_START_TICK);
	result->dec_seek_end_tick = vpu_read_reg(inst->dev, W5_RET_DEC_SEEK_END_TICK);
	result->dec_parse_start_tick = vpu_read_reg(inst->dev, W5_RET_DEC_PARSING_START_TICK);
	result->dec_parse_end_tick = vpu_read_reg(inst->dev, W5_RET_DEC_PARSING_END_TICK);
	result->dec_decode_start_tick = vpu_read_reg(inst->dev, W5_RET_DEC_DECODING_START_TICK);
	result->dec_decode_end_tick = vpu_read_reg(inst->dev, W5_RET_DEC_DECODING_END_TICK);

	decode_start_tick = max(result->dec_host_cmd_tick, vpu_dev->last_performance_cycles);
	result->frame_cycle = (result->dec_decode_end_tick - decode_start_tick) *
			      p_dec_info->cycle_per_tick;
	if (result->dec_decode_end_tick)
		vpu_dev->last_performance_cycles = result->dec_decode_end_tick;

	/* no remaining command. reset frame cycle. */
	if (p_dec_info->instance_queue_count == 0 && p_dec_info->report_queue_count == 0)
		vpu_dev->last_performance_cycles = 0;

	wave5_vpu_dec_get_hdr10_info(inst,
				     &p_dec_info->initial_info.hdr10_cll_info,
				     &p_dec_info->initial_info.hdr10_mastering_display);

	return 0;
}

int wave5_vpu_reset(struct device *dev, enum sw_reset_mode reset_mode)
{
	u32 val = 0;
	int ret = 0;
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	struct vpu_attr *p_attr = &vpu_dev->attr;
	/* VPU doesn't send response. force to set BUSY flag to 0. */
	vpu_write_reg(vpu_dev, W5_VPU_BUSY_STATUS, 0);

	val = vpu_read_reg(vpu_dev, W5_VPU_RET_VPU_CONFIG0);
	if ((val >> 16) & 0x1)
		p_attr->support_backbone = true;
	if ((val >> 22) & 0x1)
		p_attr->support_vcore_backbone = true;
	if ((val >> 28) & 0x1)
		p_attr->support_vcpu_backbone = true;

	/* waiting for completion of bus transaction */
	if (p_attr->support_backbone) {
		dev_dbg(dev, "%s: backbone supported\n", __func__);

		if (p_attr->support_vcore_backbone) {
			if (p_attr->support_vcpu_backbone) {
				/* step1 : disable request */
				wave5_fio_writel(vpu_dev, W5_BACKBONE_BUS_CTRL_VCPU, 0xFF);

				/* step2 : waiting for completion of bus transaction */
				ret = wave5_wait_vcpu_bus_busy(vpu_dev,
							       W5_BACKBONE_BUS_STATUS_VCPU);
				if (ret) {
					wave5_fio_writel(vpu_dev, W5_BACKBONE_BUS_CTRL_VCPU, 0x00);
					return ret;
				}
			}
			/* step1 : disable request */
			wave5_fio_writel(vpu_dev, W5_BACKBONE_BUS_CTRL_VCORE0, 0x7);

			/* step2 : waiting for completion of bus transaction */
			if (wave5_wait_bus_busy(vpu_dev, W5_BACKBONE_BUS_STATUS_VCORE0)) {
				wave5_fio_writel(vpu_dev, W5_BACKBONE_BUS_CTRL_VCORE0, 0x00);
				return -EBUSY;
			}
		} else {
			/* step1 : disable request */
			wave5_fio_writel(vpu_dev, W5_COMBINED_BACKBONE_BUS_CTRL, 0x7);

			/* step2 : waiting for completion of bus transaction */
			if (wave5_wait_bus_busy(vpu_dev, W5_COMBINED_BACKBONE_BUS_STATUS)) {
				wave5_fio_writel(vpu_dev, W5_COMBINED_BACKBONE_BUS_CTRL, 0x00);
				return -EBUSY;
			}
		}
	} else {
		dev_dbg(dev, "%s: backbone NOT supported\n", __func__);
		/* step1 : disable request */
		wave5_fio_writel(vpu_dev, W5_GDI_BUS_CTRL, 0x100);

		/* step2 : waiting for completion of bus transaction */
		ret = wave5_wait_bus_busy(vpu_dev, W5_GDI_BUS_STATUS);
		if (ret) {
			wave5_fio_writel(vpu_dev, W5_GDI_BUS_CTRL, 0x00);
			return ret;
		}
	}

	switch (reset_mode) {
	case SW_RESET_ON_BOOT:
	case SW_RESET_FORCE:
	case SW_RESET_SAFETY:
		val = W5_RST_BLOCK_ALL;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(vpu_dev->dev, "vpu reset, val = 0x%x\n", val);
	/* step3 : must clear GDI_BUS_CTRL after done SW_RESET */
	if (p_attr->support_backbone) {
		if (p_attr->support_vcore_backbone) {
			if (p_attr->support_vcpu_backbone)
				wave5_fio_writel(vpu_dev, W5_BACKBONE_BUS_CTRL_VCPU, 0x00);
			wave5_fio_writel(vpu_dev, W5_BACKBONE_BUS_CTRL_VCORE0, 0x00);
		} else {
			wave5_fio_writel(vpu_dev, W5_COMBINED_BACKBONE_BUS_CTRL, 0x00);
		}
	} else {
		wave5_fio_writel(vpu_dev, W5_GDI_BUS_CTRL, 0x00);
	}

	return ret;
}

int wave5_vpu_dec_finish_seq(struct vpu_instance *inst, u32 *fail_res)
{
	return send_firmware_command(inst, W5_DESTROY_INSTANCE, true, NULL, fail_res);
}

int wave5_dec_clr_disp_flag(struct vpu_instance *inst, unsigned int index)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret;

	vpu_write_reg(inst->dev, W5_CMD_DEC_CLR_DISP_IDC, BIT(index));
	vpu_write_reg(inst->dev, W5_CMD_DEC_SET_DISP_IDC, 0);

	ret = wave5_send_query(inst->dev, inst, UPDATE_DISP_FLAG);
	if (ret)
		return ret;

	p_dec_info->frame_display_flag = vpu_read_reg(inst->dev, W5_RET_DEC_DISP_IDC);

	return 0;
}

int wave5_dec_set_disp_flag(struct vpu_instance *inst, unsigned int index)
{
	int ret;

	vpu_write_reg(inst->dev, W5_CMD_DEC_CLR_DISP_IDC, 0);
	vpu_write_reg(inst->dev, W5_CMD_DEC_SET_DISP_IDC, BIT(index));

	ret = wave5_send_query(inst->dev, inst, UPDATE_DISP_FLAG);
	if (ret)
		return ret;

	return 0;
}

int wave5_vpu_clear_interrupt(struct vpu_device *dev, u32 flags)
{
	u32 interrupt_reason;

	interrupt_reason = vpu_read_reg(dev, W5_VPU_VINT_REASON_USR);
	interrupt_reason &= ~flags;
	vpu_write_reg(dev, W5_VPU_VINT_REASON_USR, interrupt_reason);

	return 0;
}

dma_addr_t wave5_dec_get_rd_ptr(struct vpu_instance *inst)
{
	int ret;

	ret = wave5_send_query(inst->dev, inst, GET_BS_RD_PTR);
	if (ret)
		return inst->codec_info->dec_info.stream_rd_ptr;

	return vpu_read_reg(inst->dev, W5_RET_QUERY_DEC_BS_RD_PTR);
}
