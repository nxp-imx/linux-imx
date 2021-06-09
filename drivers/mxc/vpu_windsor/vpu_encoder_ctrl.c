/*
 * Copyright 2018-2019 NXP
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
 * @file vpu_encoder_ctrl.c
 *
 * copyright here may be changed later
 *
 */

#define TAG	"[VPU Encoder Ctrl]\t "
#include <media/v4l2-ctrls.h>
#include <linux/imx_vpu.h>

#include "vpu_encoder_b0.h"
#include "vpu_encoder_ctrl.h"

// H264 level is maped like level 5.1 to uLevel 51, except level 1b to uLevel 14
const u_int32 h264_level[] = {
	[V4L2_MPEG_VIDEO_H264_LEVEL_1_0] = 10,
	[V4L2_MPEG_VIDEO_H264_LEVEL_1B]  = 14,
	[V4L2_MPEG_VIDEO_H264_LEVEL_1_1] = 11,
	[V4L2_MPEG_VIDEO_H264_LEVEL_1_2] = 12,
	[V4L2_MPEG_VIDEO_H264_LEVEL_1_3] = 13,
	[V4L2_MPEG_VIDEO_H264_LEVEL_2_0] = 20,
	[V4L2_MPEG_VIDEO_H264_LEVEL_2_1] = 21,
	[V4L2_MPEG_VIDEO_H264_LEVEL_2_2] = 22,
	[V4L2_MPEG_VIDEO_H264_LEVEL_3_0] = 30,
	[V4L2_MPEG_VIDEO_H264_LEVEL_3_1] = 31,
	[V4L2_MPEG_VIDEO_H264_LEVEL_3_2] = 32,
	[V4L2_MPEG_VIDEO_H264_LEVEL_4_0] = 40,
	[V4L2_MPEG_VIDEO_H264_LEVEL_4_1] = 41,
	[V4L2_MPEG_VIDEO_H264_LEVEL_4_2] = 42,
	[V4L2_MPEG_VIDEO_H264_LEVEL_5_0] = 50,
	[V4L2_MPEG_VIDEO_H264_LEVEL_5_1] = 51
};

static int set_h264_profile(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM  param = &attr->param;

	mutex_lock(&ctx->instance_mutex);
	switch (ctrl->val) {
	case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
		vpu_dbg(LVL_CTRL, "set h264 profile baseline\n");
		param->eProfile = MEDIAIP_ENC_PROF_H264_BP;
		break;
	case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
		vpu_dbg(LVL_CTRL, "set h264 profile main\n");
		param->eProfile = MEDIAIP_ENC_PROF_H264_MP;
		break;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
		vpu_dbg(LVL_CTRL, "set h264 profile high\n");
		param->eProfile = MEDIAIP_ENC_PROF_H264_HP;
		break;
	default:
		vpu_err("not support H264 profile %d, set to main\n",
				ctrl->val);
		param->eProfile = MEDIAIP_ENC_PROF_H264_MP;
		break;
	}
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_h264_level(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM  param = &attr->param;

	mutex_lock(&ctx->instance_mutex);
	param->uLevel = h264_level[ctrl->val];
	mutex_unlock(&ctx->instance_mutex);

	vpu_dbg(LVL_CTRL, "set h264 level to %d (%d)\n",
			ctrl->val, h264_level[ctrl->val]);

	return 0;
}

static int set_bitrate_mode(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM  param = &attr->param;

	mutex_lock(&ctx->instance_mutex);
	switch (ctrl->val) {
	case V4L2_MPEG_VIDEO_BITRATE_MODE_VBR:
		vpu_dbg(LVL_CTRL, "set bitrate mode VBR\n");
		param->eBitRateMode =
				MEDIAIP_ENC_BITRATECONTROLMODE_CONSTANT_QP;
		break;
	case V4L2_MPEG_VIDEO_BITRATE_MODE_CBR:
		vpu_dbg(LVL_CTRL, "set bitrate mode CBR\n");
		param->eBitRateMode = MEDIAIP_ENC_BITRATECONTROLMODE_CBR;
		break;
	default:
		vpu_err("not support bitrate mode %d, set to cbr\n",
				ctrl->val);
		param->eBitRateMode = MEDIAIP_ENC_BITRATECONTROLMODE_CBR;
		break;
	}
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_bitrate(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM  param = &attr->param;

	vpu_dbg(LVL_CTRL, "set bitrate %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	param->uTargetBitrate = ctrl->val / BITRATE_COEF;
	if (param->uMaxBitRate < param->uTargetBitrate)
		param->uMaxBitRate = param->uTargetBitrate;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_bitrate_peak(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM  param = &attr->param;

	vpu_dbg(LVL_CTRL, "set peak bitrate %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	param->uMaxBitRate = ctrl->val / BITRATE_COEF;
	if (param->uTargetBitrate > param->uMaxBitRate)
		param->uTargetBitrate = param->uMaxBitRate;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_gop_size(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM  param = &attr->param;

	vpu_dbg(LVL_CTRL, "set gop size %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	param->uIFrameInterval = ctrl->val;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_i_period(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM  param = &attr->param;

	vpu_dbg(LVL_CTRL, "set iframe interval %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	param->uIFrameInterval = ctrl->val;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int get_gop_size(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM  param = &attr->param;

	vpu_dbg(LVL_CTRL, "get gop size\n");
	ctrl->val = param->uIFrameInterval;

	return 0;
}

static int set_b_frames(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM  param = &attr->param;

	vpu_dbg(LVL_CTRL, "set bframes %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	param->uGopBLength = ctrl->val;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_qp(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM  param = &attr->param;

	vpu_dbg(LVL_CTRL, "set qp %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	param->uInitSliceQP = ctrl->val;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int get_min_buffers_for_output(struct v4l2_ctrl *ctrl)
{
	vpu_dbg(LVL_CTRL, "get min buffers for output\n");

	ctrl->val = MIN_BUFFER_COUNT;

	return 0;
}

static int set_display_re_ordering(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM  param = &attr->param;

	vpu_dbg(LVL_CTRL, "set lowlatencymode %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	if (ctrl->val)
		param->uLowLatencyMode = 1;
	else
		param->uLowLatencyMode = 0;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_force_key_frame(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);

	vpu_dbg(LVL_CTRL, "force key frame\n");
	set_bit(VPU_ENC_STATUS_KEY_FRAME, &ctx->status);

	return 0;
}

static int set_h264_cpb_size(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);

	ctx->cpb_size = ctrl->val * CPB_CTRL_UNIT;

	return 0;
}

static int set_h264_vui_sar_enable(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

	vpu_dbg(LVL_CTRL, "set h264 vui sar enable %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	if (ctrl->val)
		attr->h264_vui_sar_enable = 1;
	else
		attr->h264_vui_sar_enable = 0;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_h264_vui_sar_idc(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

	vpu_dbg(LVL_CTRL, "set h264 vui sar idc %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	if (ctrl->val < V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED)
		attr->h264_vui_sar_idc = ctrl->val;
	else
		attr->h264_vui_sar_idc = VPU_ENC_H264_EXTENDED_SAR;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_h264_vui_sar_width(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

	vpu_dbg(LVL_CTRL, "set h264 vui sar width %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	attr->h264_vui_sar_width = ctrl->val;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_h264_vui_sar_height(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

	vpu_dbg(LVL_CTRL, "set h264 vui sar height %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	attr->h264_vui_sar_height = ctrl->val;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_sc_enable(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

	vpu_dbg(LVL_CTRL, "set sc enable %d\n", ctrl->val);
	mutex_lock(&ctx->instance_mutex);
	attr->sc_enable = ctrl->val;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int add_ctrl_h264_profile(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_h264_profile_ops = {
		.s_ctrl = set_h264_profile,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std_menu(&ctx->ctrl_handler,
				      &ctrl_h264_profile_ops,
				      V4L2_CID_MPEG_VIDEO_H264_PROFILE,
				      V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
				      0xa,
				      V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);
	if (!ctrl) {
		vpu_err("add ctrl h264 profile fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_h264_level(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_h264_level_ops = {
		.s_ctrl = set_h264_level,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std_menu(&ctx->ctrl_handler,
				      &ctrl_h264_level_ops,
				      V4L2_CID_MPEG_VIDEO_H264_LEVEL,
				      V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
				      0x0,
				      V4L2_MPEG_VIDEO_H264_LEVEL_4_0);
	if (!ctrl) {
		vpu_err("add ctrl h264 level fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_bitrate_mode(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_bitrate_mode_ops = {
		.s_ctrl = set_bitrate_mode,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std_menu(&ctx->ctrl_handler,
				      &ctrl_bitrate_mode_ops,
				      V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
				      V4L2_MPEG_VIDEO_BITRATE_MODE_CBR,
				      0x0,
				      V4L2_MPEG_VIDEO_BITRATE_MODE_VBR);
	if (!ctrl) {
		vpu_err("add ctrl bitrate mode fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_bitrate(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_bitrate_ops = {
		.s_ctrl = set_bitrate,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_bitrate_ops,
				 V4L2_CID_MPEG_VIDEO_BITRATE,
				 BITRATE_LOW_THRESHOLD * BITRATE_COEF,
				 BITRATE_HIGH_THRESHOLD * BITRATE_COEF,
				 BITRATE_COEF,
				 BITRATE_DEFAULT_TARGET * BITRATE_COEF);
	if (!ctrl) {
		vpu_err("add ctrl bitrate fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_bitrate_peak(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_bitrate_ops = {
		.s_ctrl = set_bitrate_peak,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_bitrate_ops,
				 V4L2_CID_MPEG_VIDEO_BITRATE_PEAK,
				 BITRATE_LOW_THRESHOLD * BITRATE_COEF,
				 BITRATE_HIGH_THRESHOLD * BITRATE_COEF,
				 BITRATE_COEF,
				 BITRATE_DEFAULT_PEAK * BITRATE_COEF);
	if (!ctrl) {
		vpu_err("add ctrl bitrate peak fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_gop_size(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_gop_ops = {
		.s_ctrl = set_gop_size,
		.g_volatile_ctrl = get_gop_size,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_gop_ops,
				 V4L2_CID_MPEG_VIDEO_GOP_SIZE,
				 GOP_L_THRESHOLD,
				 GOP_H_THRESHOLD,
				 1,
				 GOP_DEFAULT);
	if (!ctrl) {
		vpu_err("add ctrl gop size fail\n");
		return -EINVAL;
	}

	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;
	ctrl->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	return 0;
}

static int add_ctrl_i_period(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_i_period_ops = {
		.s_ctrl = set_i_period,
		.g_volatile_ctrl = get_gop_size,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
			&ctrl_i_period_ops,
			V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
			GOP_L_THRESHOLD,
			GOP_H_THRESHOLD,
			1,
			GOP_DEFAULT);

	if (!ctrl) {
		vpu_err("add ctrl i period fail\n");
		return -EINVAL;
	}

	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;
	ctrl->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	return 0;
}

static int add_ctrl_b_frames(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_b_frames = {
		.s_ctrl = set_b_frames,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
			&ctrl_b_frames,
			V4L2_CID_MPEG_VIDEO_B_FRAMES,
			BFRAMES_L_THRESHOLD,
			BFRAMES_H_THRESHOLD,
			1,
			BFRAMES_DEFAULT);

	if (!ctrl) {
		vpu_err("add ctrl b frames fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_i_frame_qp(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_iframe_qp_ops = {
		.s_ctrl = set_qp,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_iframe_qp_ops,
				 V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
				 QP_MIN,
				 QP_MAX,
				 1,
				 QP_DEFAULT);
	if (!ctrl) {
		vpu_err("add ctrl h264 I frame qp fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_p_frame_qp(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_pframe_qp_ops = {
		.s_ctrl = set_qp,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_pframe_qp_ops,
				 V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
				 QP_MIN,
				 QP_MAX,
				 1,
				 QP_DEFAULT);
	if (!ctrl) {
		vpu_err("add ctrl h264 P frame qp fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_b_frame_qp(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_bframe_qp_ops = {
		.s_ctrl = set_qp,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_bframe_qp_ops,
				 V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP,
				 QP_MIN,
				 QP_MAX,
				 1,
				 QP_DEFAULT);
	if (!ctrl) {
		vpu_err("add ctrl h264 B frame qp fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_min_buffers_for_output(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_min_buffers_ops = {
		.g_volatile_ctrl = get_min_buffers_for_output,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_min_buffers_ops,
				 V4L2_CID_MIN_BUFFERS_FOR_OUTPUT,
				 1,
				 32,
				 1,
				 MIN_BUFFER_COUNT);
	if (!ctrl) {
		vpu_err("add ctrl min buffers for output fail\n");
		return -EINVAL;
	}

	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	return 0;
}

static int add_ctrl_display_re_ordering(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops re_ordering_ops = {
		.s_ctrl = set_display_re_ordering,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
			&re_ordering_ops,
			V4L2_CID_MPEG_VIDEO_H264_ASO,
			0, 1, 1, 0);
	if (!ctrl) {
		vpu_err("add ctrl display re ordering fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_force_key_frame(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops force_key_frame_ops = {
		.s_ctrl = set_force_key_frame,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
			&force_key_frame_ops,
			V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
			0, 0, 0, 0);
	if (!ctrl) {
		vpu_err("add ctrl force key frame fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_h264_cpb_size(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_cpb_size_ops = {
		.s_ctrl = set_h264_cpb_size,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				&ctrl_cpb_size_ops,
				V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE,
				64, 10240, 1, 1024);

	if (!ctrl) {
		vpu_err("add ctrl h264 cpb size fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_h264_vui_sar_enable(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_vui_sar_enable_ops = {
		.s_ctrl = set_h264_vui_sar_enable,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				&ctrl_vui_sar_enable_ops,
				V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE,
				0, 1, 1, 1);
	if (!ctrl) {
		vpu_err("add ctrl h264 vui sar enable fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_h264_vui_sar_idc(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_vui_sar_idc_ops = {
		.s_ctrl = set_h264_vui_sar_idc,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std_menu(&ctx->ctrl_handler,
				&ctrl_vui_sar_idc_ops,
				V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC,
				V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED,
				0x0,
				V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_1x1);


	if (!ctrl) {
		vpu_err("add ctrl h264 vui sar idc fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_h264_vui_sar_width(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_vui_sar_width_ops = {
		.s_ctrl = set_h264_vui_sar_width,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				&ctrl_vui_sar_width_ops,
				V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH,
				0, USHRT_MAX, 1, 1);

	if (!ctrl) {
		vpu_err("add ctrl h264 vui sar width fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_h264_vui_sar_height(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_vui_sar_height_ops = {
		.s_ctrl = set_h264_vui_sar_height,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				&ctrl_vui_sar_height_ops,
				V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT,
				0, USHRT_MAX, 1, 1);
	if (!ctrl) {
		vpu_err("add ctrl h264 vui sar height fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_sc_enable(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops sc_ops = {
		.s_ctrl = set_sc_enable,
	};
	struct v4l2_ctrl_config config = {
		.id = V4L2_CID_SC_ENABLE,
		.name = "scene change detect enable",
		.ops = &sc_ops,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_custom(&ctx->ctrl_handler, &config, NULL);
	if (!ctrl) {
		vpu_err("add ctrl sc enable fail\n");
		return -EINVAL;
	}

	return 0;
}

static int vpu_enc_register_ctrls(struct vpu_ctx *ctx)
{
	add_ctrl_h264_profile(ctx);
	add_ctrl_h264_level(ctx);
	add_ctrl_bitrate_mode(ctx);
	add_ctrl_bitrate(ctx);
	add_ctrl_bitrate_peak(ctx);
	add_ctrl_gop_size(ctx);
	add_ctrl_i_period(ctx);
	add_ctrl_b_frames(ctx);
	add_ctrl_i_frame_qp(ctx);
	add_ctrl_p_frame_qp(ctx);
	add_ctrl_b_frame_qp(ctx);
	add_ctrl_min_buffers_for_output(ctx);
	add_ctrl_display_re_ordering(ctx);
	add_ctrl_force_key_frame(ctx);
	add_ctrl_h264_cpb_size(ctx);
	add_ctrl_h264_vui_sar_enable(ctx);
	add_ctrl_h264_vui_sar_idc(ctx);
	add_ctrl_h264_vui_sar_width(ctx);
	add_ctrl_h264_vui_sar_height(ctx);
	add_ctrl_sc_enable(ctx);

	return 0;
}

int vpu_enc_setup_ctrls(struct vpu_ctx *ctx)
{
	vpu_log_func();

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 11);
	vpu_enc_register_ctrls(ctx);
	if (ctx->ctrl_handler.error) {
		vpu_err("control initialization error (%d)\n",
			ctx->ctrl_handler.error);
		return -EINVAL;
	}
	ctx->ctrl_inited = true;
	return v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
}

int vpu_enc_free_ctrls(struct vpu_ctx *ctx)
{
	vpu_log_func();

	if (ctx->ctrl_inited) {
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		ctx->ctrl_inited = false;
	}

	return 0;
}
