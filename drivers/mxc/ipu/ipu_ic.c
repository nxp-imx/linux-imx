/*
 * Copyright 2005-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

extern int g_ipu_hw_rev;
static void _init_csc(uint8_t ic_task, ipu_color_space_t in_format,
		      ipu_color_space_t out_format);
static bool _calc_resize_coeffs(uint32_t inSize, uint32_t outSize,
				uint32_t *resizeCoeff,
				uint32_t *downsizeCoeff);

void _ipu_ic_enable_task(ipu_channel_t channel)
{
	uint32_t ic_conf;

	ic_conf = __raw_readl(IC_CONF);
	switch (channel) {
	case CSI_PRP_VF_ADC:
	case MEM_PRP_VF_ADC:
	case CSI_PRP_VF_MEM:
	case MEM_PRP_VF_MEM:
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
	case MEM_PP_ADC:
	case MEM_PP_MEM:
		ic_conf |= IC_CONF_PP_EN;
		break;
	case MEM_ROT_PP_MEM:
		ic_conf |= IC_CONF_PP_ROT_EN;
		break;
	case CSI_MEM:
		ic_conf |= IC_CONF_RWS_EN | IC_CONF_PRPENC_EN;
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
	case CSI_PRP_VF_ADC:
	case MEM_PRP_VF_ADC:
	case CSI_PRP_VF_MEM:
	case MEM_PRP_VF_MEM:
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
	case MEM_PP_ADC:
	case MEM_PP_MEM:
		ic_conf &= ~IC_CONF_PP_EN;
		break;
	case MEM_ROT_PP_MEM:
		ic_conf &= ~IC_CONF_PP_ROT_EN;
		break;
	case CSI_MEM:
		ic_conf &= ~(IC_CONF_RWS_EN | IC_CONF_PRPENC_EN);
		break;
	default:
		break;
	}
	__raw_writel(ic_conf, IC_CONF);
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
	_calc_resize_coeffs(params->mem_prp_vf_mem.in_width,
			    params->mem_prp_vf_mem.out_width,
			    &resizeCoeff, &downsizeCoeff);
	reg |= (downsizeCoeff << 14) | resizeCoeff;

	__raw_writel(reg, IC_PRP_VF_RSC);

	ic_conf = __raw_readl(IC_CONF);

	/* Setup color space conversion */
	in_fmt = format_to_colorspace(params->mem_prp_vf_mem.in_pixel_fmt);
	out_fmt = format_to_colorspace(params->mem_prp_vf_mem.out_pixel_fmt);
	if (in_fmt == RGB) {
		if ((out_fmt == YCbCr) || (out_fmt == YUV)) {
			_init_csc(IC_TASK_VIEWFINDER, RGB, out_fmt);
			ic_conf |= IC_CONF_PRPVF_CSC1;	/* Enable RGB->YCBCR CSC */
		}
	}
	if ((in_fmt == YCbCr) || (in_fmt == YUV)) {
		if (out_fmt == RGB) {
			_init_csc(IC_TASK_VIEWFINDER, YCbCr, RGB);
			ic_conf |= IC_CONF_PRPVF_CSC1;	/* Enable YCBCR->RGB CSC */
		} else {
			/* TODO: Support YUV<->YCbCr conversion? */
		}
	}

	if (params->mem_prp_vf_mem.graphics_combine_en) {
		ic_conf |= IC_CONF_PRPVF_CMB;

		/* need transparent CSC1 conversion */
		_init_csc(IC_TASK_POST_PROCESSOR, RGB, RGB);
		ic_conf |= IC_CONF_PRPVF_CSC1;	/* Enable RGB->RGB CSC */

		if (params->mem_prp_vf_mem.global_alpha_en) {
			ic_conf |= IC_CONF_IC_GLB_LOC_A;
		} else {
			ic_conf &= ~IC_CONF_IC_GLB_LOC_A;
		}

		if (params->mem_prp_vf_mem.key_color_en) {
			ic_conf |= IC_CONF_KEY_COLOR_EN;
		} else {
			ic_conf &= ~IC_CONF_KEY_COLOR_EN;
		}
	} else {
		ic_conf &= ~IC_CONF_PP_CMB;
	}

#ifndef CONFIG_VIRTIO_SUPPORT	/* Setting RWS_EN doesn't work in Virtio */
	if (src_is_csi) {
		ic_conf &= ~IC_CONF_RWS_EN;
	} else {
		ic_conf |= IC_CONF_RWS_EN;
	}
#endif
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

void _ipu_ic_init_csi(ipu_channel_params_t *params)
{
	uint32_t reg;
	reg = __raw_readl(IC_CONF);
	reg &= ~IC_CONF_CSI_MEM_WR_EN;
	__raw_writel(reg, IC_CONF);
}

void _ipu_ic_uninit_csi(void)
{
	uint32_t reg;
	reg = __raw_readl(IC_CONF);
	reg &= ~(IC_CONF_RWS_EN | IC_CONF_PRPENC_EN);
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
	_calc_resize_coeffs(params->mem_prp_enc_mem.in_width,
			    params->mem_prp_enc_mem.out_width,
			    &resizeCoeff, &downsizeCoeff);
	reg |= (downsizeCoeff << 14) | resizeCoeff;

	__raw_writel(reg, IC_PRP_ENC_RSC);

	ic_conf = __raw_readl(IC_CONF);

	/* Setup color space conversion */
	in_fmt = format_to_colorspace(params->mem_prp_enc_mem.in_pixel_fmt);
	out_fmt = format_to_colorspace(params->mem_prp_enc_mem.out_pixel_fmt);
	if (in_fmt == RGB) {
		if ((out_fmt == YCbCr) || (out_fmt == YUV)) {
			/* TODO: ERROR! */
		}
	}
	if ((in_fmt == YCbCr) || (in_fmt == YUV)) {
		if (out_fmt == RGB) {
			_init_csc(IC_TASK_ENCODER, YCbCr, RGB);
			ic_conf |= IC_CONF_PRPENC_CSC1;	/* Enable YCBCR->RGB CSC */
		} else {
			/* TODO: Support YUV<->YCbCr conversion? */
		}
	}

	if (src_is_csi) {
		ic_conf &= ~IC_CONF_RWS_EN;
	} else {
		ic_conf |= IC_CONF_RWS_EN;
	}

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
	_calc_resize_coeffs(params->mem_pp_mem.in_height,
			    params->mem_pp_mem.out_height,
			    &resizeCoeff, &downsizeCoeff);
	reg = (downsizeCoeff << 30) | (resizeCoeff << 16);

	/* Setup horizontal resizing */
	_calc_resize_coeffs(params->mem_pp_mem.in_width,
			    params->mem_pp_mem.out_width,
			    &resizeCoeff, &downsizeCoeff);
	reg |= (downsizeCoeff << 14) | resizeCoeff;

	__raw_writel(reg, IC_PP_RSC);

	ic_conf = __raw_readl(IC_CONF);

	/* Setup color space conversion */
	in_fmt = format_to_colorspace(params->mem_pp_mem.in_pixel_fmt);
	out_fmt = format_to_colorspace(params->mem_pp_mem.out_pixel_fmt);
	if (in_fmt == RGB) {
		if ((out_fmt == YCbCr) || (out_fmt == YUV)) {
			_init_csc(IC_TASK_POST_PROCESSOR, RGB, out_fmt);
			ic_conf |= IC_CONF_PP_CSC2;	/* Enable RGB->YCBCR CSC */
		}
	}
	if ((in_fmt == YCbCr) || (in_fmt == YUV)) {
		if (out_fmt == RGB) {
			_init_csc(IC_TASK_POST_PROCESSOR, YCbCr, RGB);
			ic_conf |= IC_CONF_PP_CSC1;	/* Enable YCBCR->RGB CSC */
		} else {
			/* TODO: Support YUV<->YCbCr conversion? */
		}
	}

	if (params->mem_pp_mem.graphics_combine_en) {
		ic_conf |= IC_CONF_PP_CMB;

		/* need transparent CSC1 conversion */
		_init_csc(IC_TASK_POST_PROCESSOR, RGB, RGB);
		ic_conf |= IC_CONF_PP_CSC1;	/* Enable RGB->RGB CSC */

		if (params->mem_pp_mem.global_alpha_en) {
			ic_conf |= IC_CONF_IC_GLB_LOC_A;
		} else {
			ic_conf &= ~IC_CONF_IC_GLB_LOC_A;
		}

		if (params->mem_pp_mem.key_color_en) {
			ic_conf |= IC_CONF_KEY_COLOR_EN;
		} else {
			ic_conf &= ~IC_CONF_KEY_COLOR_EN;
		}
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

static void _init_csc(uint8_t ic_task, ipu_color_space_t in_format,
		      ipu_color_space_t out_format)
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

	uint32_t param[2];
	uint32_t address = 0;

	if (g_ipu_hw_rev > 1) {
		if (ic_task == IC_TASK_VIEWFINDER) {
			address = 0x645 << 3;
		} else if (ic_task == IC_TASK_ENCODER) {
			address = 0x321 << 3;
		} else if (ic_task == IC_TASK_POST_PROCESSOR) {
			address = 0x96C << 3;
		} else {
			BUG();
		}
	} else {
		if (ic_task == IC_TASK_VIEWFINDER) {
			address = 0x5a5 << 3;
		} else if (ic_task == IC_TASK_ENCODER) {
			address = 0x2d1 << 3;
		} else if (ic_task == IC_TASK_POST_PROCESSOR) {
			address = 0x87c << 3;
		} else {
			BUG();
		}
	}

	if ((in_format == YCbCr) && (out_format == RGB)) {
		/* Init CSC1 (YCbCr->RGB) */
		param[0] =
		    (ycbcr2rgb_coeff[3][0] << 27) | (ycbcr2rgb_coeff[0][0] <<
						     18) |
		    (ycbcr2rgb_coeff[1][1] << 9) | ycbcr2rgb_coeff[2][2];
		/* scale = 2, sat = 0 */
		param[1] = (ycbcr2rgb_coeff[3][0] >> 5) | (2L << (40 - 32));
		_ipu_write_param_mem(address, param, 2);
		dev_dbg(g_ipu_dev,
			"addr 0x%04X: word0 = 0x%08X, word1 = 0x%08X\n",
			address, param[0], param[1]);

		param[0] =
		    (ycbcr2rgb_coeff[3][1] << 27) | (ycbcr2rgb_coeff[0][1] <<
						     18) |
		    (ycbcr2rgb_coeff[1][0] << 9) | ycbcr2rgb_coeff[2][0];
		param[1] = (ycbcr2rgb_coeff[3][1] >> 5);
		address += 1L << 3;
		_ipu_write_param_mem(address, param, 2);
		dev_dbg(g_ipu_dev,
			"addr 0x%04X: word0 = 0x%08X, word1 = 0x%08X\n",
			address, param[0], param[1]);

		param[0] =
		    (ycbcr2rgb_coeff[3][2] << 27) | (ycbcr2rgb_coeff[0][2] <<
						     18) |
		    (ycbcr2rgb_coeff[1][2] << 9) | ycbcr2rgb_coeff[2][1];
		param[1] = (ycbcr2rgb_coeff[3][2] >> 5);
		address += 1L << 3;
		_ipu_write_param_mem(address, param, 2);
		dev_dbg(g_ipu_dev,
			"addr 0x%04X: word0 = 0x%08X, word1 = 0x%08X\n",
			address, param[0], param[1]);
	} else if ((in_format == RGB) && (out_format == YCbCr)) {
		/* Init CSC1 (RGB->YCbCr) */
		param[0] =
		    (rgb2ycbcr_coeff[3][0] << 27) | (rgb2ycbcr_coeff[0][0] <<
						     18) |
		    (rgb2ycbcr_coeff[1][1] << 9) | rgb2ycbcr_coeff[2][2];
		/* scale = 1, sat = 0 */
		param[1] = (rgb2ycbcr_coeff[3][0] >> 5) | (1UL << 8);
		_ipu_write_param_mem(address, param, 2);
		dev_dbg(g_ipu_dev,
			"addr 0x%04X: word0 = 0x%08X, word1 = 0x%08X\n",
			address, param[0], param[1]);

		param[0] =
		    (rgb2ycbcr_coeff[3][1] << 27) | (rgb2ycbcr_coeff[0][1] <<
						     18) |
		    (rgb2ycbcr_coeff[1][0] << 9) | rgb2ycbcr_coeff[2][0];
		param[1] = (rgb2ycbcr_coeff[3][1] >> 5);
		address += 1L << 3;
		_ipu_write_param_mem(address, param, 2);
		dev_dbg(g_ipu_dev,
			"addr 0x%04X: word0 = 0x%08X, word1 = 0x%08X\n",
			address, param[0], param[1]);

		param[0] =
		    (rgb2ycbcr_coeff[3][2] << 27) | (rgb2ycbcr_coeff[0][2] <<
						     18) |
		    (rgb2ycbcr_coeff[1][2] << 9) | rgb2ycbcr_coeff[2][1];
		param[1] = (rgb2ycbcr_coeff[3][2] >> 5);
		address += 1L << 3;
		_ipu_write_param_mem(address, param, 2);
		dev_dbg(g_ipu_dev,
			"addr 0x%04X: word0 = 0x%08X, word1 = 0x%08X\n",
			address, param[0], param[1]);
	} else if ((in_format == RGB) && (out_format == RGB)) {
		/* Init CSC1 */
		param[0] =
		    (rgb2rgb_coeff[3][0] << 27) | (rgb2rgb_coeff[0][0] << 18) |
		    (rgb2rgb_coeff[1][1] << 9) | rgb2rgb_coeff[2][2];
		/* scale = 2, sat = 0 */
		param[1] = (rgb2rgb_coeff[3][0] >> 5) | (2UL << 8);

		_ipu_write_param_mem(address, param, 2);

		dev_dbg(g_ipu_dev,
			"addr 0x%04X: word0 = 0x%08X, word1 = 0x%08X\n",
			address, param[0], param[1]);

		param[0] =
		    (rgb2rgb_coeff[3][1] << 27) | (rgb2rgb_coeff[0][1] << 18) |
		    (rgb2rgb_coeff[1][0] << 9) | rgb2rgb_coeff[2][0];
		param[1] = (rgb2rgb_coeff[3][1] >> 5);

		address += 1L << 3;
		_ipu_write_param_mem(address, param, 2);

		dev_dbg(g_ipu_dev,
			"addr 0x%04X: word0 = 0x%08X, word1 = 0x%08X\n",
			address, param[0], param[1]);

		param[0] =
		    (rgb2rgb_coeff[3][2] << 27) | (rgb2rgb_coeff[0][2] << 18) |
		    (rgb2rgb_coeff[1][2] << 9) | rgb2rgb_coeff[2][1];
		param[1] = (rgb2rgb_coeff[3][2] >> 5);

		address += 1L << 3;
		_ipu_write_param_mem(address, param, 2);

		dev_dbg(g_ipu_dev,
			"addr 0x%04X: word0 = 0x%08X, word1 = 0x%08X\n",
			address, param[0], param[1]);
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

	/* Cannot downsize more than 8:1 */
	if ((outSize << 3) < inSize) {
		return false;
	}
	/* compute downsizing coefficient */
	tempDownsize = 0;
	tempSize = inSize;
	while ((tempSize >= outSize * 2) && (tempDownsize < 2)) {
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
