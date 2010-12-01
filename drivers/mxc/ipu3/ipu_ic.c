/*
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * @file ipu_ic.c
 *
 * @brief IPU IC functions
 *
 * @ingroup IPU
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/io.h>
#include <linux/ipu.h>

#include "ipu_prv.h"
#include "ipu_regs.h"
#include "ipu_param_mem.h"

enum {
	IC_TASK_VIEWFINDER,
	IC_TASK_ENCODER,
	IC_TASK_POST_PROCESSOR
};

static void _init_csc(uint8_t ic_task, ipu_color_space_t in_format,
		      ipu_color_space_t out_format, int csc_index);
static bool _calc_resize_coeffs(uint32_t inSize, uint32_t outSize,
				uint32_t *resizeCoeff,
				uint32_t *downsizeCoeff);

void _ipu_vdi_set_top_field_man(bool top_field_0)
{
	uint32_t reg;

	reg = __raw_readl(VDI_C);
	if (top_field_0)
		reg &= ~VDI_C_TOP_FIELD_MAN_1;
	else
		reg |= VDI_C_TOP_FIELD_MAN_1;
	__raw_writel(reg, VDI_C);
}

void _ipu_vdi_set_motion(ipu_motion_sel motion_sel)
{
	uint32_t reg;

	reg = __raw_readl(VDI_C);
	reg &= ~(VDI_C_MOT_SEL_FULL | VDI_C_MOT_SEL_MED | VDI_C_MOT_SEL_LOW);
	if (motion_sel == HIGH_MOTION)
		reg |= VDI_C_MOT_SEL_FULL;
	else if (motion_sel == MED_MOTION)
		reg |= VDI_C_MOT_SEL_MED;
	else
		reg |= VDI_C_MOT_SEL_LOW;

	__raw_writel(reg, VDI_C);
}

void ic_dump_register(void)
{
	printk(KERN_DEBUG "IC_CONF = \t0x%08X\n", __raw_readl(IC_CONF));
	printk(KERN_DEBUG "IC_PRP_ENC_RSC = \t0x%08X\n",
	       __raw_readl(IC_PRP_ENC_RSC));
	printk(KERN_DEBUG "IC_PRP_VF_RSC = \t0x%08X\n",
	       __raw_readl(IC_PRP_VF_RSC));
	printk(KERN_DEBUG "IC_PP_RSC = \t0x%08X\n", __raw_readl(IC_PP_RSC));
	printk(KERN_DEBUG "IC_IDMAC_1 = \t0x%08X\n", __raw_readl(IC_IDMAC_1));
	printk(KERN_DEBUG "IC_IDMAC_2 = \t0x%08X\n", __raw_readl(IC_IDMAC_2));
	printk(KERN_DEBUG "IC_IDMAC_3 = \t0x%08X\n", __raw_readl(IC_IDMAC_3));
}

void _ipu_ic_enable_task(ipu_channel_t channel)
{
	uint32_t ic_conf;

	ic_conf = __raw_readl(IC_CONF);
	switch (channel) {
	case CSI_PRP_VF_MEM:
	case MEM_PRP_VF_MEM:
		ic_conf |= IC_CONF_PRPVF_EN;
		break;
	case MEM_VDI_PRP_VF_MEM:
		ic_conf |= IC_CONF_PRPVF_EN;
		break;
	case MEM_ROT_VF_MEM:
		ic_conf |= IC_CONF_PRPVF_ROT_EN;
		break;
	case CSI_PRP_ENC_MEM:
	case MEM_PRP_ENC_MEM:
		ic_conf |= IC_CONF_PRPENC_EN;
		break;
	case MEM_ROT_ENC_MEM:
		ic_conf |= IC_CONF_PRPENC_ROT_EN;
		break;
	case MEM_PP_MEM:
		ic_conf |= IC_CONF_PP_EN;
		break;
	case MEM_ROT_PP_MEM:
		ic_conf |= IC_CONF_PP_ROT_EN;
		break;
	default:
		break;
	}
	__raw_writel(ic_conf, IC_CONF);
}

void _ipu_ic_disable_task(ipu_channel_t channel)
{
	uint32_t ic_conf;

	ic_conf = __raw_readl(IC_CONF);
	switch (channel) {
	case CSI_PRP_VF_MEM:
	case MEM_PRP_VF_MEM:
		ic_conf &= ~IC_CONF_PRPVF_EN;
		break;
	case MEM_VDI_PRP_VF_MEM:
		ic_conf &= ~IC_CONF_PRPVF_EN;
		break;
	case MEM_ROT_VF_MEM:
		ic_conf &= ~IC_CONF_PRPVF_ROT_EN;
		break;
	case CSI_PRP_ENC_MEM:
	case MEM_PRP_ENC_MEM:
		ic_conf &= ~IC_CONF_PRPENC_EN;
		break;
	case MEM_ROT_ENC_MEM:
		ic_conf &= ~IC_CONF_PRPENC_ROT_EN;
		break;
	case MEM_PP_MEM:
		ic_conf &= ~IC_CONF_PP_EN;
		break;
	case MEM_ROT_PP_MEM:
		ic_conf &= ~IC_CONF_PP_ROT_EN;
		break;
	default:
		break;
	}
	__raw_writel(ic_conf, IC_CONF);
}

void _ipu_vdi_init(ipu_channel_t channel, ipu_channel_params_t *params)
{
	uint32_t reg;
	uint32_t pixel_fmt;

	reg = ((params->mem_prp_vf_mem.in_height-1) << 16) |
	  (params->mem_prp_vf_mem.in_width-1);
	__raw_writel(reg, VDI_FSIZE);

	/* Full motion, only vertical filter is used
	   Burst size is 4 accesses */
	if (params->mem_prp_vf_mem.in_pixel_fmt ==
	     IPU_PIX_FMT_UYVY ||
	     params->mem_prp_vf_mem.in_pixel_fmt ==
	     IPU_PIX_FMT_YUYV)
		pixel_fmt = VDI_C_CH_422;
	else
		pixel_fmt = VDI_C_CH_420;

	reg = __raw_readl(VDI_C);
	reg |= pixel_fmt;
	switch (channel) {
	case MEM_VDI_PRP_VF_MEM:
		reg |= VDI_C_BURST_SIZE2_4;
		break;
	case MEM_VDI_PRP_VF_MEM_P:
		reg |= VDI_C_BURST_SIZE1_4 | VDI_C_VWM1_SET_1 | VDI_C_VWM1_CLR_2;
		break;
	case MEM_VDI_PRP_VF_MEM_N:
		reg |= VDI_C_BURST_SIZE3_4 | VDI_C_VWM3_SET_1 | VDI_C_VWM3_CLR_2;
		break;
	default:
		break;
	}
	__raw_writel(reg, VDI_C);

	if (params->mem_prp_vf_mem.field_fmt == V4L2_FIELD_INTERLACED_TB)
		_ipu_vdi_set_top_field_man(false);
	else if (params->mem_prp_vf_mem.field_fmt == V4L2_FIELD_INTERLACED_BT)
		_ipu_vdi_set_top_field_man(true);

	_ipu_vdi_set_motion(params->mem_prp_vf_mem.motion_sel);

	reg = __raw_readl(IC_CONF);
	reg &= ~IC_CONF_RWS_EN;
	__raw_writel(reg, IC_CONF);
}

void _ipu_vdi_uninit(void)
{
	__raw_writel(0, VDI_FSIZE);
	__raw_writel(0, VDI_C);
}

void _ipu_ic_init_prpvf(ipu_channel_params_t *params, bool src_is_csi)
{
	uint32_t reg, ic_conf;
	uint32_t downsizeCoeff, resizeCoeff;
	ipu_color_space_t in_fmt, out_fmt;

	/* Setup vertical resizing */
	_calc_resize_coeffs(params->mem_prp_vf_mem.in_height,
			    params->mem_prp_vf_mem.out_height,
			    &resizeCoeff, &downsizeCoeff);
	reg = (downsizeCoeff << 30) | (resizeCoeff << 16);

	/* Setup horizontal resizing */
	/* Upadeted for IC split case */
	if (!(params->mem_prp_vf_mem.outh_resize_ratio)) {
		_calc_resize_coeffs(params->mem_prp_vf_mem.in_width,
				params->mem_prp_vf_mem.out_width,
				&resizeCoeff, &downsizeCoeff);
		reg |= (downsizeCoeff << 14) | resizeCoeff;
	} else
		reg |= params->mem_prp_vf_mem.outh_resize_ratio;

	__raw_writel(reg, IC_PRP_VF_RSC);

	ic_conf = __raw_readl(IC_CONF);

	/* Setup color space conversion */
	in_fmt = format_to_colorspace(params->mem_prp_vf_mem.in_pixel_fmt);
	out_fmt = format_to_colorspace(params->mem_prp_vf_mem.out_pixel_fmt);
	if (in_fmt == RGB) {
		if ((out_fmt == YCbCr) || (out_fmt == YUV)) {
			/* Enable RGB->YCBCR CSC1 */
			_init_csc(IC_TASK_VIEWFINDER, RGB, out_fmt, 1);
			ic_conf |= IC_CONF_PRPVF_CSC1;
		}
	}
	if ((in_fmt == YCbCr) || (in_fmt == YUV)) {
		if (out_fmt == RGB) {
			/* Enable YCBCR->RGB CSC1 */
			_init_csc(IC_TASK_VIEWFINDER, YCbCr, RGB, 1);
			ic_conf |= IC_CONF_PRPVF_CSC1;
		} else {
			/* TODO: Support YUV<->YCbCr conversion? */
		}
	}

	if (params->mem_prp_vf_mem.graphics_combine_en) {
		ic_conf |= IC_CONF_PRPVF_CMB;

		if (!(ic_conf & IC_CONF_PRPVF_CSC1)) {
			/* need transparent CSC1 conversion */
			_init_csc(IC_TASK_VIEWFINDER, RGB, RGB, 1);
			ic_conf |= IC_CONF_PRPVF_CSC1;  /* Enable RGB->RGB CSC */
		}
		in_fmt = format_to_colorspace(params->mem_prp_vf_mem.in_g_pixel_fmt);
		out_fmt = format_to_colorspace(params->mem_prp_vf_mem.out_pixel_fmt);
		if (in_fmt == RGB) {
			if ((out_fmt == YCbCr) || (out_fmt == YUV)) {
				/* Enable RGB->YCBCR CSC2 */
				_init_csc(IC_TASK_VIEWFINDER, RGB, out_fmt, 2);
				ic_conf |= IC_CONF_PRPVF_CSC2;
			}
		}
		if ((in_fmt == YCbCr) || (in_fmt == YUV)) {
			if (out_fmt == RGB) {
				/* Enable YCBCR->RGB CSC2 */
				_init_csc(IC_TASK_VIEWFINDER, YCbCr, RGB, 2);
				ic_conf |= IC_CONF_PRPVF_CSC2;
			} else {
				/* TODO: Support YUV<->YCbCr conversion? */
			}
		}

		if (params->mem_prp_vf_mem.global_alpha_en) {
			ic_conf |= IC_CONF_IC_GLB_LOC_A;
			reg = __raw_readl(IC_CMBP_1);
			reg &= ~(0xff);
			reg |= params->mem_prp_vf_mem.alpha;
			__raw_writel(reg, IC_CMBP_1);
		} else
			ic_conf &= ~IC_CONF_IC_GLB_LOC_A;

		if (params->mem_prp_vf_mem.key_color_en) {
			ic_conf |= IC_CONF_KEY_COLOR_EN;
			__raw_writel(params->mem_prp_vf_mem.key_color,
					IC_CMBP_2);
		} else
			ic_conf &= ~IC_CONF_KEY_COLOR_EN;
	} else {
		ic_conf &= ~IC_CONF_PRPVF_CMB;
	}

	if (src_is_csi)
		ic_conf &= ~IC_CONF_RWS_EN;
	else
		ic_conf |= IC_CONF_RWS_EN;

	__raw_writel(ic_conf, IC_CONF);
}

void _ipu_ic_uninit_prpvf(void)
{
	uint32_t reg;

	reg = __raw_readl(IC_CONF);
	reg &= ~(IC_CONF_PRPVF_EN | IC_CONF_PRPVF_CMB |
		 IC_CONF_PRPVF_CSC2 | IC_CONF_PRPVF_CSC1);
	__raw_writel(reg, IC_CONF);
}

void _ipu_ic_init_rotate_vf(ipu_channel_params_t *params)
{
}

void _ipu_ic_uninit_rotate_vf(void)
{
	uint32_t reg;
	reg = __raw_readl(IC_CONF);
	reg &= ~IC_CONF_PRPVF_ROT_EN;
	__raw_writel(reg, IC_CONF);
}

void _ipu_ic_init_prpenc(ipu_channel_params_t *params, bool src_is_csi)
{
	uint32_t reg, ic_conf;
	uint32_t downsizeCoeff, resizeCoeff;
	ipu_color_space_t in_fmt, out_fmt;

	/* Setup vertical resizing */
	_calc_resize_coeffs(params->mem_prp_enc_mem.in_height,
			    params->mem_prp_enc_mem.out_height,
			    &resizeCoeff, &downsizeCoeff);
	reg = (downsizeCoeff << 30) | (resizeCoeff << 16);

	/* Setup horizontal resizing */
	/* Upadeted for IC split case */
	if (!(params->mem_prp_enc_mem.outh_resize_ratio)) {
		_calc_resize_coeffs(params->mem_prp_enc_mem.in_width,
				params->mem_prp_enc_mem.out_width,
				&resizeCoeff, &downsizeCoeff);
		reg |= (downsizeCoeff << 14) | resizeCoeff;
	} else
		reg |= params->mem_prp_enc_mem.outh_resize_ratio;

	__raw_writel(reg, IC_PRP_ENC_RSC);

	ic_conf = __raw_readl(IC_CONF);

	/* Setup color space conversion */
	in_fmt = format_to_colorspace(params->mem_prp_enc_mem.in_pixel_fmt);
	out_fmt = format_to_colorspace(params->mem_prp_enc_mem.out_pixel_fmt);
	if (in_fmt == RGB) {
		if ((out_fmt == YCbCr) || (out_fmt == YUV)) {
			/* Enable RGB->YCBCR CSC1 */
			_init_csc(IC_TASK_ENCODER, RGB, out_fmt, 1);
			ic_conf |= IC_CONF_PRPENC_CSC1;
		}
	}
	if ((in_fmt == YCbCr) || (in_fmt == YUV)) {
		if (out_fmt == RGB) {
			/* Enable YCBCR->RGB CSC1 */
			_init_csc(IC_TASK_ENCODER, YCbCr, RGB, 1);
			ic_conf |= IC_CONF_PRPENC_CSC1;
		} else {
			/* TODO: Support YUV<->YCbCr conversion? */
		}
	}

	if (src_is_csi)
		ic_conf &= ~IC_CONF_RWS_EN;
	else
		ic_conf |= IC_CONF_RWS_EN;

	__raw_writel(ic_conf, IC_CONF);
}

void _ipu_ic_uninit_prpenc(void)
{
	uint32_t reg;

	reg = __raw_readl(IC_CONF);
	reg &= ~(IC_CONF_PRPENC_EN | IC_CONF_PRPENC_CSC1);
	__raw_writel(reg, IC_CONF);
}

void _ipu_ic_init_rotate_enc(ipu_channel_params_t *params)
{
}

void _ipu_ic_uninit_rotate_enc(void)
{
	uint32_t reg;

	reg = __raw_readl(IC_CONF);
	reg &= ~(IC_CONF_PRPENC_ROT_EN);
	__raw_writel(reg, IC_CONF);
}

void _ipu_ic_init_pp(ipu_channel_params_t *params)
{
	uint32_t reg, ic_conf;
	uint32_t downsizeCoeff, resizeCoeff;
	ipu_color_space_t in_fmt, out_fmt;

	/* Setup vertical resizing */
	if (!(params->mem_pp_mem.outv_resize_ratio)) {
		_calc_resize_coeffs(params->mem_pp_mem.in_height,
			    params->mem_pp_mem.out_height,
			    &resizeCoeff, &downsizeCoeff);
		reg = (downsizeCoeff << 30) | (resizeCoeff << 16);
	} else {
		reg = (params->mem_pp_mem.outv_resize_ratio) << 16;
	}

	/* Setup horizontal resizing */
	/* Upadeted for IC split case */
	if (!(params->mem_pp_mem.outh_resize_ratio)) {
		_calc_resize_coeffs(params->mem_pp_mem.in_width,
							params->mem_pp_mem.out_width,
							&resizeCoeff, &downsizeCoeff);
		reg |= (downsizeCoeff << 14) | resizeCoeff;
	} else {
		reg |= params->mem_pp_mem.outh_resize_ratio;
	}

	__raw_writel(reg, IC_PP_RSC);

	ic_conf = __raw_readl(IC_CONF);

	/* Setup color space conversion */
	in_fmt = format_to_colorspace(params->mem_pp_mem.in_pixel_fmt);
	out_fmt = format_to_colorspace(params->mem_pp_mem.out_pixel_fmt);
	if (in_fmt == RGB) {
		if ((out_fmt == YCbCr) || (out_fmt == YUV)) {
			/* Enable RGB->YCBCR CSC1 */
			_init_csc(IC_TASK_POST_PROCESSOR, RGB, out_fmt, 1);
			ic_conf |= IC_CONF_PP_CSC1;
		}
	}
	if ((in_fmt == YCbCr) || (in_fmt == YUV)) {
		if (out_fmt == RGB) {
			/* Enable YCBCR->RGB CSC1 */
			_init_csc(IC_TASK_POST_PROCESSOR, YCbCr, RGB, 1);
			ic_conf |= IC_CONF_PP_CSC1;
		} else {
			/* TODO: Support YUV<->YCbCr conversion? */
		}
	}

	if (params->mem_pp_mem.graphics_combine_en) {
		ic_conf |= IC_CONF_PP_CMB;

		if (!(ic_conf & IC_CONF_PP_CSC1)) {
			/* need transparent CSC1 conversion */
			_init_csc(IC_TASK_POST_PROCESSOR, RGB, RGB, 1);
			ic_conf |= IC_CONF_PP_CSC1;  /* Enable RGB->RGB CSC */
		}

		in_fmt = format_to_colorspace(params->mem_pp_mem.in_g_pixel_fmt);
		out_fmt = format_to_colorspace(params->mem_pp_mem.out_pixel_fmt);
		if (in_fmt == RGB) {
			if ((out_fmt == YCbCr) || (out_fmt == YUV)) {
				/* Enable RGB->YCBCR CSC2 */
				_init_csc(IC_TASK_POST_PROCESSOR, RGB, out_fmt, 2);
				ic_conf |= IC_CONF_PP_CSC2;
			}
		}
		if ((in_fmt == YCbCr) || (in_fmt == YUV)) {
			if (out_fmt == RGB) {
				/* Enable YCBCR->RGB CSC2 */
				_init_csc(IC_TASK_POST_PROCESSOR, YCbCr, RGB, 2);
				ic_conf |= IC_CONF_PP_CSC2;
			} else {
				/* TODO: Support YUV<->YCbCr conversion? */
			}
		}

		if (params->mem_pp_mem.global_alpha_en) {
			ic_conf |= IC_CONF_IC_GLB_LOC_A;
			reg = __raw_readl(IC_CMBP_1);
			reg &= ~(0xff00);
			reg |= (params->mem_pp_mem.alpha << 8);
			__raw_writel(reg, IC_CMBP_1);
		} else
			ic_conf &= ~IC_CONF_IC_GLB_LOC_A;

		if (params->mem_pp_mem.key_color_en) {
			ic_conf |= IC_CONF_KEY_COLOR_EN;
			__raw_writel(params->mem_pp_mem.key_color,
					IC_CMBP_2);
		} else
			ic_conf &= ~IC_CONF_KEY_COLOR_EN;
	} else {
		ic_conf &= ~IC_CONF_PP_CMB;
	}

	__raw_writel(ic_conf, IC_CONF);
}

void _ipu_ic_uninit_pp(void)
{
	uint32_t reg;

	reg = __raw_readl(IC_CONF);
	reg &= ~(IC_CONF_PP_EN | IC_CONF_PP_CSC1 | IC_CONF_PP_CSC2 |
		 IC_CONF_PP_CMB);
	__raw_writel(reg, IC_CONF);
}

void _ipu_ic_init_rotate_pp(ipu_channel_params_t *params)
{
}

void _ipu_ic_uninit_rotate_pp(void)
{
	uint32_t reg;
	reg = __raw_readl(IC_CONF);
	reg &= ~IC_CONF_PP_ROT_EN;
	__raw_writel(reg, IC_CONF);
}

int _ipu_ic_idma_init(int dma_chan, uint16_t width, uint16_t height,
		      int burst_size, ipu_rotate_mode_t rot)
{
	u32 ic_idmac_1, ic_idmac_2, ic_idmac_3;
	u32 temp_rot = bitrev8(rot) >> 5;
	bool need_hor_flip = false;

	if ((burst_size != 8) && (burst_size != 16)) {
		dev_dbg(g_ipu_dev, "Illegal burst length for IC\n");
		return -EINVAL;
	}

	width--;
	height--;

	if (temp_rot & 0x2)	/* Need horizontal flip */
		need_hor_flip = true;

	ic_idmac_1 = __raw_readl(IC_IDMAC_1);
	ic_idmac_2 = __raw_readl(IC_IDMAC_2);
	ic_idmac_3 = __raw_readl(IC_IDMAC_3);
	if (dma_chan == 22) {	/* PP output - CB2 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB2_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB2_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PP_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PP_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PP_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PP_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PP_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PP_WIDTH_OFFSET;

	} else if (dma_chan == 11) {	/* PP Input - CB5 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB5_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB5_BURST_16;
	} else if (dma_chan == 47) {	/* PP Rot input */
		ic_idmac_1 &= ~IC_IDMAC_1_PP_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PP_ROT_OFFSET;
	}

	if (dma_chan == 12) {	/* PRP Input - CB6 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB6_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB6_BURST_16;
	}

	if (dma_chan == 20) {	/* PRP ENC output - CB0 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB0_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB0_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PRPENC_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PRPENC_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PRPENC_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PRPENC_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PRPENC_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PRPENC_WIDTH_OFFSET;

	} else if (dma_chan == 45) {	/* PRP ENC Rot input */
		ic_idmac_1 &= ~IC_IDMAC_1_PRPENC_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PRPENC_ROT_OFFSET;
	}

	if (dma_chan == 21) {	/* PRP VF output - CB1 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB1_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB1_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PRPVF_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PRPVF_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PRPVF_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PRPVF_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PRPVF_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PRPVF_WIDTH_OFFSET;

	} else if (dma_chan == 46) {	/* PRP VF Rot input */
		ic_idmac_1 &= ~IC_IDMAC_1_PRPVF_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PRPVF_ROT_OFFSET;
	}

	if (dma_chan == 14) {	/* PRP VF graphics combining input - CB3 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB3_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB3_BURST_16;
	} else if (dma_chan == 15) {	/* PP graphics combining input - CB4 */
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB4_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB4_BURST_16;
	}

	__raw_writel(ic_idmac_1, IC_IDMAC_1);
	__raw_writel(ic_idmac_2, IC_IDMAC_2);
	__raw_writel(ic_idmac_3, IC_IDMAC_3);

	return 0;
}

static void _init_csc(uint8_t ic_task, ipu_color_space_t in_format,
		      ipu_color_space_t out_format, int csc_index)
{

/*     Y = R *  .299 + G *  .587 + B *  .114;
       U = R * -.169 + G * -.332 + B *  .500 + 128.;
       V = R *  .500 + G * -.419 + B * -.0813 + 128.;*/
	static const uint32_t rgb2ycbcr_coeff[4][3] = {
		{0x004D, 0x0096, 0x001D},
		{0x01D5, 0x01AB, 0x0080},
		{0x0080, 0x0195, 0x01EB},
		{0x0000, 0x0200, 0x0200},	/* A0, A1, A2 */
	};

	/* transparent RGB->RGB matrix for combining
	 */
	static const uint32_t rgb2rgb_coeff[4][3] = {
		{0x0080, 0x0000, 0x0000},
		{0x0000, 0x0080, 0x0000},
		{0x0000, 0x0000, 0x0080},
		{0x0000, 0x0000, 0x0000},	/* A0, A1, A2 */
	};

/*     R = (1.164 * (Y - 16)) + (1.596 * (Cr - 128));
       G = (1.164 * (Y - 16)) - (0.392 * (Cb - 128)) - (0.813 * (Cr - 128));
       B = (1.164 * (Y - 16)) + (2.017 * (Cb - 128); */
	static const uint32_t ycbcr2rgb_coeff[4][3] = {
		{149, 0, 204},
		{149, 462, 408},
		{149, 255, 0},
		{8192 - 446, 266, 8192 - 554},	/* A0, A1, A2 */
	};

	uint32_t param;
	uint32_t *base = NULL;

	if (ic_task == IC_TASK_ENCODER) {
		base = ipu_tpmem_base + 0x2008 / 4;
	} else if (ic_task == IC_TASK_VIEWFINDER) {
		if (csc_index == 1)
			base = ipu_tpmem_base + 0x4028 / 4;
		else
			base = ipu_tpmem_base + 0x4040 / 4;
	} else if (ic_task == IC_TASK_POST_PROCESSOR) {
		if (csc_index == 1)
			base = ipu_tpmem_base + 0x6060 / 4;
		else
			base = ipu_tpmem_base + 0x6078 / 4;
	} else {
		BUG();
	}

	if ((in_format == YCbCr) && (out_format == RGB)) {
		/* Init CSC (YCbCr->RGB) */
		param = (ycbcr2rgb_coeff[3][0] << 27) |
			(ycbcr2rgb_coeff[0][0] << 18) |
			(ycbcr2rgb_coeff[1][1] << 9) | ycbcr2rgb_coeff[2][2];
		__raw_writel(param, base++);
		/* scale = 2, sat = 0 */
		param = (ycbcr2rgb_coeff[3][0] >> 5) | (2L << (40 - 32));
		__raw_writel(param, base++);

		param = (ycbcr2rgb_coeff[3][1] << 27) |
			(ycbcr2rgb_coeff[0][1] << 18) |
			(ycbcr2rgb_coeff[1][0] << 9) | ycbcr2rgb_coeff[2][0];
		__raw_writel(param, base++);
		param = (ycbcr2rgb_coeff[3][1] >> 5);
		__raw_writel(param, base++);

		param = (ycbcr2rgb_coeff[3][2] << 27) |
			(ycbcr2rgb_coeff[0][2] << 18) |
			(ycbcr2rgb_coeff[1][2] << 9) | ycbcr2rgb_coeff[2][1];
		__raw_writel(param, base++);
		param = (ycbcr2rgb_coeff[3][2] >> 5);
		__raw_writel(param, base++);
	} else if ((in_format == RGB) && (out_format == YCbCr)) {
		/* Init CSC (RGB->YCbCr) */
		param = (rgb2ycbcr_coeff[3][0] << 27) |
			(rgb2ycbcr_coeff[0][0] << 18) |
			(rgb2ycbcr_coeff[1][1] << 9) | rgb2ycbcr_coeff[2][2];
		__raw_writel(param, base++);
		/* scale = 1, sat = 0 */
		param = (rgb2ycbcr_coeff[3][0] >> 5) | (1UL << 8);
		__raw_writel(param, base++);

		param = (rgb2ycbcr_coeff[3][1] << 27) |
			(rgb2ycbcr_coeff[0][1] << 18) |
			(rgb2ycbcr_coeff[1][0] << 9) | rgb2ycbcr_coeff[2][0];
		__raw_writel(param, base++);
		param = (rgb2ycbcr_coeff[3][1] >> 5);
		__raw_writel(param, base++);

		param = (rgb2ycbcr_coeff[3][2] << 27) |
			(rgb2ycbcr_coeff[0][2] << 18) |
			(rgb2ycbcr_coeff[1][2] << 9) | rgb2ycbcr_coeff[2][1];
		__raw_writel(param, base++);
		param = (rgb2ycbcr_coeff[3][2] >> 5);
		__raw_writel(param, base++);
	} else if ((in_format == RGB) && (out_format == RGB)) {
		/* Init CSC */
		param =
		    (rgb2rgb_coeff[3][0] << 27) | (rgb2rgb_coeff[0][0] << 18) |
		    (rgb2rgb_coeff[1][1] << 9) | rgb2rgb_coeff[2][2];
		__raw_writel(param, base++);
		/* scale = 2, sat = 0 */
		param = (rgb2rgb_coeff[3][0] >> 5) | (2UL << 8);
		__raw_writel(param, base++);

		param =
		    (rgb2rgb_coeff[3][1] << 27) | (rgb2rgb_coeff[0][1] << 18) |
		    (rgb2rgb_coeff[1][0] << 9) | rgb2rgb_coeff[2][0];
		__raw_writel(param, base++);
		param = (rgb2rgb_coeff[3][1] >> 5);
		__raw_writel(param, base++);

		param =
		    (rgb2rgb_coeff[3][2] << 27) | (rgb2rgb_coeff[0][2] << 18) |
		    (rgb2rgb_coeff[1][2] << 9) | rgb2rgb_coeff[2][1];
		__raw_writel(param, base++);
		param = (rgb2rgb_coeff[3][2] >> 5);
		__raw_writel(param, base++);
	} else {
		dev_err(g_ipu_dev, "Unsupported color space conversion\n");
	}
}

static bool _calc_resize_coeffs(uint32_t inSize, uint32_t outSize,
				uint32_t *resizeCoeff,
				uint32_t *downsizeCoeff)
{
	uint32_t tempSize;
	uint32_t tempDownsize;

	/* Input size cannot be more than 4096 */
	/* Output size cannot be more than 1024 */
	if ((inSize > 4096) || (outSize > 1024))
		return false;

	/* Cannot downsize more than 8:1 */
	if ((outSize << 3) < inSize)
		return false;

	/* Compute downsizing coefficient */
	/* Output of downsizing unit cannot be more than 1024 */
	tempDownsize = 0;
	tempSize = inSize;
	while (((tempSize > 1024) || (tempSize >= outSize * 2)) &&
	       (tempDownsize < 2)) {
		tempSize >>= 1;
		tempDownsize++;
	}
	*downsizeCoeff = tempDownsize;

	/* compute resizing coefficient using the following equation:
	   resizeCoeff = M*(SI -1)/(SO - 1)
	   where M = 2^13, SI - input size, SO - output size    */
	*resizeCoeff = (8192L * (tempSize - 1)) / (outSize - 1);
	if (*resizeCoeff >= 16384L) {
		dev_err(g_ipu_dev, "Warning! Overflow on resize coeff.\n");
		*resizeCoeff = 0x3FFF;
	}

	dev_dbg(g_ipu_dev, "resizing from %u -> %u pixels, "
		"downsize=%u, resize=%u.%lu (reg=%u)\n", inSize, outSize,
		*downsizeCoeff, (*resizeCoeff >= 8192L) ? 1 : 0,
		((*resizeCoeff & 0x1FFF) * 10000L) / 8192L, *resizeCoeff);

	return true;
}

void _ipu_vdi_toggle_top_field_man()
{
	uint32_t reg;
	uint32_t mask_reg;

	reg = __raw_readl(VDI_C);
	mask_reg = reg & VDI_C_TOP_FIELD_MAN_1;
	if (mask_reg == VDI_C_TOP_FIELD_MAN_1)
		reg &= ~VDI_C_TOP_FIELD_MAN_1;
	else
		reg |= VDI_C_TOP_FIELD_MAN_1;

	__raw_writel(reg, VDI_C);
}

