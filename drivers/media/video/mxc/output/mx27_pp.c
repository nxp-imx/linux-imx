/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mx27_pp.c
 *
 * @brief MX27 V4L2 Video Output Driver
 *
 * Video4Linux2 Output Device using MX27 eMMA Post-processing functionality.
 *
 * @ingroup MXC_V4L2_OUTPUT
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#include "mx27_pp.h"
#include "mxc_v4l2_output.h"

#define SCALE_RETRY	32	/* to be more relax, less precise */
#define PP_SKIP		1
#define PP_TBL_MAX	40

static unsigned short scale_tbl[PP_TBL_MAX];
static int g_hlen, g_vlen;

static emma_pp_cfg g_pp_cfg;
static int g_disp_num = 0;
static char pp_dev[] = "emma_pp";

/*!
 * @brief PP resizing routines
 */
static int gcd(int x, int y);
static int ratio(int x, int y, int *den);
static int scale_0d(int k, int coeff, int base, int nxt);
static int scale_1d(int inv, int outv, int k);
static int scale_1d_smart(int *inv, int *outv, int index);
static int scale_2d(emma_pp_scale * sz);

static irqreturn_t pp_isr(int irq, void *dev_id);
static int set_output_addr(emma_pp_cfg * cfg, vout_data * vout);
static int pphw_reset(void);
static int pphw_enable(int flag);
static int pphw_ptr(emma_pp_cfg * cfg);
static int pphw_outptr(emma_pp_cfg * cfg);
static int pphw_cfg(emma_pp_cfg * cfg);
static int pphw_isr(void);
static void pphw_init(void);
static void pphw_exit(void);

#define PP_DUMP(reg)	pr_debug("%s\t = 0x%08X\n", #reg, __raw_readl(reg))
void pp_dump(void)
{
	PP_DUMP(PP_CNTL);
	PP_DUMP(PP_INTRCNTL);
	PP_DUMP(PP_INTRSTATUS);
	PP_DUMP(PP_SOURCE_Y_PTR);
	PP_DUMP(PP_SOURCE_CB_PTR);
	PP_DUMP(PP_SOURCE_CR_PTR);
	PP_DUMP(PP_DEST_RGB_PTR);
	PP_DUMP(PP_QUANTIZER_PTR);
	PP_DUMP(PP_PROCESS_FRAME_PARA);
	PP_DUMP(PP_SOURCE_FRAME_WIDTH);
	PP_DUMP(PP_DEST_DISPLAY_WIDTH);
	PP_DUMP(PP_DEST_IMAGE_SIZE);
	PP_DUMP(PP_DEST_FRAME_FMT_CNTL);
	PP_DUMP(PP_RESIZE_INDEX);
	PP_DUMP(PP_CSC_COEF_0123);
	PP_DUMP(PP_CSC_COEF_4);
}

/*!
 * @brief Set PP input address.
 * @param ptr	The pointer to the Y value of input
 * @return	Zero on success, others on failure
 */
int pp_ptr(unsigned long ptr)
{
	g_pp_cfg.ptr.y = ptr;
	g_pp_cfg.ptr.u = g_pp_cfg.ptr.v = g_pp_cfg.ptr.qp = 0;

	return pphw_ptr(&g_pp_cfg);
}

/*!
 * @brief Enable or disable PP.
 * @param flag	Zero to disable PP, others to enable PP
 * @return	Zero on success, others on failure
 */
int pp_enable(int flag)
{
	return pphw_enable(flag);
}

/*!
 * @brief Get the display No. of last completed PP frame.
 * @return	The display No. of last completed PP frame.
 */
int pp_num_last(void)
{
	return (g_disp_num ? 0 : 1);
}

/*!
 * @brief Initialize PP.
 * @param vout	Pointer to _vout_data structure
 * @return	Zero on success, others on failure
 */
int pp_init(vout_data * vout)
{
	pphw_init();
	pphw_enable(0);
	enable_irq(MXC_INT_EMMAPP);
	return request_irq(MXC_INT_EMMAPP, pp_isr, 0, pp_dev, vout);
}

/*!
 * @brief Deinitialize PP.
 * @param vout	Pointer to _vout_data structure
 */
void pp_exit(vout_data * vout)
{
	disable_irq(MXC_INT_EMMAPP);
	free_irq(MXC_INT_EMMAPP, vout);
	pphw_enable(0);
	pphw_exit();
}

/*!
 * @brief Configure PP.
 * @param vout	Pointer to _vout_data structure
 * @return	Zero on success, others on failure
 */
int pp_cfg(vout_data * vout)
{
	if (!vout)
		return -1;

	/* PP accepts YUV420 input only */
	if (vout->v2f.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) {
		pr_debug("unsupported pixel format.\n");
		return -1;
	}

	g_pp_cfg.operation = 0;

	memset(g_pp_cfg.csc_table, 0, sizeof(g_pp_cfg.csc_table));

	/* Convert output pixel format to PP required format */
	switch (vout->v4l2_fb.fmt.pixelformat) {
	case V4L2_PIX_FMT_BGR32:
		g_pp_cfg.red_width = 8;
		g_pp_cfg.green_width = 8;
		g_pp_cfg.blue_width = 8;
		g_pp_cfg.red_offset = 8;
		g_pp_cfg.green_offset = 16;
		g_pp_cfg.blue_offset = 24;
		g_pp_cfg.rgb_resolution = 32;
		break;
	case V4L2_PIX_FMT_RGB32:
		g_pp_cfg.red_width = 8;
		g_pp_cfg.green_width = 8;
		g_pp_cfg.blue_width = 8;
		g_pp_cfg.red_offset = 24;
		g_pp_cfg.green_offset = 16;
		g_pp_cfg.blue_offset = 8;
		g_pp_cfg.rgb_resolution = 32;
		break;
	case V4L2_PIX_FMT_YUYV:
		g_pp_cfg.red_width = 0;
		g_pp_cfg.green_width = 0;
		g_pp_cfg.blue_width = 0;
		g_pp_cfg.red_offset = 0;
		g_pp_cfg.green_offset = 0;
		g_pp_cfg.blue_offset = PP_PIX_YUYV;
		g_pp_cfg.rgb_resolution = 16;
		break;
	case V4L2_PIX_FMT_UYVY:
		g_pp_cfg.red_width = 0;
		g_pp_cfg.green_width = 0;
		g_pp_cfg.blue_width = 0;
		g_pp_cfg.red_offset = 0;
		g_pp_cfg.green_offset = 0;
		g_pp_cfg.blue_offset = PP_PIX_UYVY;
		g_pp_cfg.rgb_resolution = 16;
		break;
	case V4L2_PIX_FMT_RGB565:
	default:
		g_pp_cfg.red_width = 5;
		g_pp_cfg.green_width = 6;
		g_pp_cfg.blue_width = 5;
		g_pp_cfg.red_offset = 11;
		g_pp_cfg.green_offset = 5;
		g_pp_cfg.blue_offset = 0;
		g_pp_cfg.rgb_resolution = 16;
		break;
	}

	if (vout->ipu_buf[0] != -1)
		g_pp_cfg.ptr.y =
		    (unsigned int)vout->queue_buf_paddr[vout->ipu_buf[0]];
	else
		g_pp_cfg.ptr.y = 0;

	g_pp_cfg.ptr.u = g_pp_cfg.ptr.v = g_pp_cfg.ptr.qp = 0;

	g_pp_cfg.dim.in.width = vout->v2f.fmt.pix.width;
	g_pp_cfg.dim.in.height = vout->v2f.fmt.pix.height;
	g_pp_cfg.dim.out.width = vout->crop_current.width;
	g_pp_cfg.dim.out.height = vout->crop_current.height;
	g_pp_cfg.dim.num.width = 0;
	g_pp_cfg.dim.num.height = 0;
	g_pp_cfg.dim.den.width = 0;
	g_pp_cfg.dim.den.height = 0;

	if (scale_2d(&g_pp_cfg.dim)) {
		pr_debug("unsupported resize ratio.\n");
		return -1;
	}

	g_pp_cfg.dim.out.width = vout->crop_current.width;
	g_pp_cfg.dim.out.height = vout->crop_current.height;

	g_pp_cfg.in_y_stride = 0;
	if (set_output_addr(&g_pp_cfg, vout)) {
		pr_debug("failed to set pp output address.\n");
		return -1;
	}

	return pphw_cfg(&g_pp_cfg);
}

irqreturn_t mxc_v4l2out_pp_in_irq_handler(int irq, void *dev_id);

/*!
 * @brief PP IRQ handler.
 */
static irqreturn_t pp_isr(int irq, void *dev_id)
{
	int status;
	vout_data *vout = dev_id;

	status = pphw_isr();
	if ((status & 0x1) == 0) {	/* Not frame complete interrupt */
		pr_debug("not pp frame complete interrupt\n");
		return IRQ_HANDLED;
	}

	if (vout->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		g_disp_num = g_disp_num ? 0 : 1;
		g_pp_cfg.outptr = (unsigned int)vout->display_bufs[g_disp_num];
		pphw_outptr(&g_pp_cfg);
	}

	return mxc_v4l2out_pp_in_irq_handler(irq, dev_id);
}

/*!
 * @brief Set PP output address.
 * @param cfg	Pointer to emma_pp_cfg structure
 * @param vout	Pointer to _vout_data structure
 * @return	Zero on success, others on failure
 */
static int set_output_addr(emma_pp_cfg * cfg, vout_data * vout)
{
	if (vout->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		g_disp_num = 0;
		cfg->outptr = (unsigned int)vout->display_bufs[g_disp_num];
		cfg->out_stride = vout->crop_current.width;
		return 0;
	} else {
		struct fb_info *fb;

		fb = registered_fb[vout->output_fb_num[vout->cur_disp_output]];
		if (!fb)
			return -1;

		cfg->outptr = fb->fix.smem_start;
		cfg->outptr += vout->crop_current.top * fb->var.xres_virtual
		    * (fb->var.bits_per_pixel >> 3)
		    + vout->crop_current.left * (fb->var.bits_per_pixel >> 3);
		cfg->out_stride = fb->var.xres_virtual;

		return 0;
	}
}

/*!
 * @brief Get maximum common divisor.
 * @param x	First input value
 * @param y	Second input value
 * @return	Maximum common divisor of x and y
 */
static int gcd(int x, int y)
{
	int k;

	if (x < y) {
		k = x;
		x = y;
		y = k;
	}

	while ((k = x % y)) {
		x = y;
		y = k;
	}

	return y;
}

/*!
 * @brief Get ratio.
 * @param x	First input value
 * @param y	Second input value
 * @param den	Denominator of the ratio (corresponding to y)
 * @return	Numerator of the ratio (corresponding to x)
 */
static int ratio(int x, int y, int *den)
{
	int g;

	if (!x || !y)
		return 0;

	g = gcd(x, y);
	*den = y / g;

	return x / g;
}

/*!
 * @brief Build PP coefficient entry
 * Build one or more coefficient entries for PP coefficient table based
 * on given coefficient.
 *
 * @param k	The index of the coefficient in coefficient table
 * @param coeff	The weighting coefficient
 * @param base	The base of the coefficient
 * @param nxt	Number of pixels to be read
 *
 * @return	The index of the next coefficient entry on success
 *		-1 on failure
 */
static int scale_0d(int k, int coeff, int base, int nxt)
{
	if (k >= PP_TBL_MAX) {
		/* no more space in table */
		pr_debug("no space in scale table, k = %d\n", k);
		return -1;
	}

	coeff = ((coeff << BC_COEF) + (base >> 1)) / base;

	/*
	 * Valid values for weighting coefficient are 0, 2 to 30, and 31.
	 * A value of 31 is treated as 32 and therefore 31 is an
	 * invalid co-efficient.
	 */
	if (coeff >= SZ_COEF - 1)
		coeff--;
	else if (coeff == 1)
		coeff++;
	coeff = coeff << BC_NXT;

	if (nxt < SZ_NXT) {
		coeff |= nxt;
		coeff <<= 1;
		coeff |= 1;
	} else {
		/*
		 * src inc field is 2 bit wide, for 4+, use special
		 * code 0:0:1 to prevent dest inc
		 */
		coeff |= PP_SKIP;
		coeff <<= 1;
		coeff |= 1;
		nxt -= PP_SKIP;
		do {
			pr_debug("tbl = %03X\n", coeff);
			scale_tbl[k++] = coeff;
			coeff = (nxt > PP_SKIP) ? PP_SKIP : nxt;
			coeff <<= 1;
		} while ((nxt -= PP_SKIP) > 0);
	}
	pr_debug("tbl = %03X\n", coeff);
	scale_tbl[k++] = coeff;

	return k;
}

/*
 * @brief Build PP coefficient table
 * Build PP coefficient table for one dimension (width or height)
 * based on given input and output resolution
 *
 * @param inv	input resolution
 * @param outv	output resolution
 * @param k	index of free table entry
 *
 * @return	The index of the next free coefficient entry on success
 *		-1 on failure
 */
static int scale_1d(int inv, int outv, int k)
{
	int v;			/* overflow counter */
	int coeff, nxt;		/* table output */

	if (inv == outv)
		return scale_0d(k, 1, 1, 1);	/* force scaling */

	if (inv * 4 < outv) {
		pr_debug("upscale err: ratio should be in range 1:1 to 1:4\n");
		return -1;
	}

	v = 0;
	if (inv < outv) {
		/* upscale: mix <= 2 input pixels per output pixel */
		do {
			coeff = outv - v;
			v += inv;
			if (v >= outv) {
				v -= outv;
				nxt = 1;
			} else
				nxt = 0;
			pr_debug("upscale: coeff = %d/%d nxt = %d\n", coeff,
				 outv, nxt);
			k = scale_0d(k, coeff, outv, nxt);
			if (k < 0)
				return -1;
		} while (v);
	} else if (inv >= 2 * outv) {
		/* PP doesn't support resize ratio > 2:1 except 4:1. */
		if ((inv != 2 * outv) && (inv != 4 * outv))
			return -1;
		/* downscale: >=2:1 bilinear approximation */
		coeff = inv - 2 * outv;
		v = 0;
		nxt = 0;
		do {
			v += coeff;
			nxt = 2;
			while (v >= outv) {
				v -= outv;
				nxt++;
			}
			pr_debug("downscale: coeff = 1/2 nxt = %d\n", nxt);
			k = scale_0d(k, 1, 2, nxt);
			if (k < 0)
				return -1;
		} while (v);
	} else {
		/* downscale: bilinear */
		int in_pos_inc = 2 * outv;
		int out_pos = inv;
		int out_pos_inc = 2 * inv;
		int init_carry = inv - outv;
		int carry = init_carry;

		v = outv + in_pos_inc;
		do {
			coeff = v - out_pos;
			out_pos += out_pos_inc;
			carry += out_pos_inc;
			for (nxt = 0; v < out_pos; nxt++) {
				v += in_pos_inc;
				carry -= in_pos_inc;
			}
			pr_debug("downscale: coeff = %d/%d nxt = %d\n", coeff,
				 in_pos_inc, nxt);
			k = scale_0d(k, coeff, in_pos_inc, nxt);
			if (k < 0)
				return -1;
		} while (carry != init_carry);
	}
	return k;
}

/*
 * @brief Build PP coefficient table
 * Build PP coefficient table for one dimension (width or height)
 * based on given input and output resolution. The given input
 * and output resolution might be not supported due to hardware
 * limits. In this case this functin rounds the input and output
 * to closest possible values and return them to caller.
 *
 * @param inv	input resolution, might be modified after the call
 * @param outv	output resolution, might be modified after the call
 * @param k	index of free table entry
 *
 * @return	The index of the next free coefficient entry on success
 *		-1 on failure
 */
static int scale_1d_smart(int *inv, int *outv, int index)
{
	int len, num, den, retry;
	static int num1, den1;

	if (!inv || !outv)
		return -1;

	/* Both should be non-zero */
	if (!(*inv) || !(*outv))
		return -1;

	retry = SCALE_RETRY;

	do {
		num = ratio(*inv, *outv, &den);
		pr_debug("num = %d, den = %d\n", num, den);
		if (!num)
			continue;

		if (index != 0) {
			/*
			 * We are now resizing height. Check to see if the
			 * resize ratio for width can be reused by height
			 */
			if ((num == num1) && (den == den1))
				return index;
		}

		if ((len = scale_1d(num, den, index)) < 0)
			/* increase output dimension to try another ratio */
			(*outv)++;
		else {
			if (index == 0) {
				/*
				 * We are now resizing width. The same resize
				 * ratio may be reused by height, so save the
				 * ratio.
				 */
				num1 = num;
				den1 = den;
			}
			return len;
		}
	} while (retry--);

	pr_debug("pp scale err\n");
	return -1;
}

/*
 * @brief Build PP coefficient table for both width and height
 * Build PP coefficient table for both width and height based on
 * given resizing ratios.
 *
 * @param sz	Structure contains resizing ratio informations
 *
 * @return	0 on success, others on failure
 */
static int scale_2d(emma_pp_scale * sz)
{
	int inv, outv;

	/* horizontal resizing. parameter check - must provide in size */
	if (!sz->in.width)
		return -1;

	/* Resizing based on num:den */
	inv = sz->num.width;
	outv = sz->den.width;

	if ((g_hlen = scale_1d_smart(&inv, &outv, 0)) > 0) {
		/* Resizing succeeded */
		sz->den.width = outv;
		sz->out.width = (sz->in.width * outv) / inv;
	} else {
		/* Resizing based on in:out */
		inv = sz->in.width;
		outv = sz->out.width;

		if ((g_hlen = scale_1d_smart(&inv, &outv, 0)) > 0) {
			/* Resizing succeeded */
			sz->out.width = outv;
			sz->num.width = ratio(sz->in.width, sz->out.width,
					      &sz->den.width);
		} else
			return -1;
	}

	sz->out.width &= ~1;

	/* vertical resizing. parameter check - must provide in size */
	if (!sz->in.height)
		return -1;

	/* Resizing based on num:den */
	inv = sz->num.height;
	outv = sz->den.height;

	if ((g_vlen = scale_1d_smart(&inv, &outv, g_hlen)) > 0) {
		/* Resizing succeeded */
		sz->den.height = outv;
		sz->out.height = (sz->in.height * outv) / inv;
	} else {
		/* Resizing based on in:out */
		inv = sz->in.height;
		outv = sz->out.height;

		if ((g_vlen = scale_1d_smart(&inv, &outv, g_hlen)) > 0) {
			/* Resizing succeeded */
			sz->out.height = outv;
			sz->num.height = ratio(sz->in.height, sz->out.height,
					       &sz->den.height);
		} else
			return -1;
	}

	return 0;
}

/*!
 * @brief Set PP resizing registers.
 * @param sz	Pointer to pp scaling structure
 * @return	Zero on success, others on failure
 */
static int pphw_scale(emma_pp_scale * sz)
{
	__raw_writel((sz->out.width << 16) | sz->out.height,
		     PP_DEST_IMAGE_SIZE);
	__raw_writel(((g_hlen - 1) << 16) | (g_vlen ==
					     g_hlen ? 0 : (g_hlen << 8)) |
		     (g_vlen - 1), PP_RESIZE_INDEX);
	for (g_hlen = 0; g_hlen < g_vlen; g_hlen++)
		__raw_writel(scale_tbl[g_hlen],
			     PP_RESIZE_COEF_TBL + g_hlen * 4);

	return 0;
}

/*!
 * @brief Reset PP.
 * @return	Zero on success, others on failure
 */
static int pphw_reset(void)
{
	int i;

	__raw_writel(0x100, PP_CNTL);

	/* timeout */
	for (i = 0; i < 1000; i++) {
		if (!(__raw_readl(PP_CNTL) & 0x100)) {
			pr_debug("pp reset over\n");
			break;
		}
	}

	/* check reset value */
	if (__raw_readl(PP_CNTL) != 0x876) {
		pr_debug("pp reset value err = 0x%08X\n", __raw_readl(PP_CNTL));
		return -1;
	}

	return 0;
}

/*!
 * @brief Enable or disable PP.
 * @param flag	Zero to disable PP, others to enable PP
 * @return	Zero on success, others on failure
 */
static int pphw_enable(int flag)
{
	int ret = 0;

	if (flag)
		__raw_writel(__raw_readl(PP_CNTL) | 1, PP_CNTL);
	else
		ret = pphw_reset();

	return ret;
}

/*!
 * @brief Set PP input address.
 * @param cfg	The pointer to PP configuration parameter
 * @return	Zero on success, others on failure
 */
static int pphw_ptr(emma_pp_cfg * cfg)
{
	if (!cfg->ptr.u) {
		int size;

		/* yuv - packed */
		size = PP_CALC_Y_SIZE(cfg);
		cfg->ptr.u = cfg->ptr.y + size;
		cfg->ptr.v = cfg->ptr.u + (size >> 2);

		/* yuv packed with qp appended */
		if (!cfg->ptr.qp)
			cfg->ptr.qp = cfg->ptr.v + (size >> 2);
	}
	__raw_writel(cfg->ptr.y, PP_SOURCE_Y_PTR);
	__raw_writel(cfg->ptr.u, PP_SOURCE_CB_PTR);
	__raw_writel(cfg->ptr.v, PP_SOURCE_CR_PTR);
	__raw_writel(cfg->ptr.qp, PP_QUANTIZER_PTR);

	return 0;
}

/*!
 * @brief Set PP output address.
 * @param cfg	The pointer to PP configuration parameter
 * @return	Zero on success, others on failure
 */
static int pphw_outptr(emma_pp_cfg * cfg)
{
	__raw_writel(cfg->outptr, PP_DEST_RGB_PTR);
	return 0;
}

/*!
 * @brief Configuration PP.
 * @param cfg	The pointer to PP configuration parameter
 * @return	Zero on success, others on failure
 */
static int pphw_cfg(emma_pp_cfg * cfg)
{
	int rt;
	register int r;

	pphw_scale(&cfg->dim);

	if (!cfg->in_y_stride)
		cfg->in_y_stride = cfg->dim.in.width;

	if (!cfg->out_stride)
		cfg->out_stride = cfg->dim.out.width;

	r = __raw_readl(PP_CNTL) & ~EN_MASK;

	/* config parms */
	r |= cfg->operation & EN_MASK;
	if (cfg->operation & EN_MACROBLOCK) {
		/* Macroblock Mode */
		r |= 0x0200;
		__raw_writel(0x06, PP_INTRCNTL);
	} else {
		/* Frame mode */
		__raw_writel(0x05, PP_INTRCNTL);
	}

	if (cfg->red_width | cfg->green_width | cfg->blue_width) {
		/* color conversion to be performed */
		r |= EN_CSC;
		if (!(cfg->red_offset | cfg->green_offset)) {
			/* auto offset B:G:R LSb to Msb */
			cfg->green_offset = cfg->blue_offset + cfg->blue_width;
			cfg->red_offset = cfg->green_offset + cfg->green_width;
		}
		if (!cfg->rgb_resolution) {
			/* derive minimum resolution required */
			int w, w2;

			w = cfg->red_offset + cfg->red_width;
			w2 = cfg->blue_offset + cfg->blue_width;
			if (w < w2)
				w = w2;
			w2 = cfg->green_offset + cfg->green_width;
			if (w < w2)
				w = w2;
			if (w > 16)
				w = 24;
			else if (w > 8)
				w = 16;
			else
				w = 8;
			cfg->rgb_resolution = w;
		}
		/* 00,11 - 32 bpp, 10 - 16 bpp, 01 - 8 bpp */
		r &= ~0xC00;
		if (cfg->rgb_resolution < 32)
			r |= (cfg->rgb_resolution << 7);
		__raw_writel((cfg->red_offset << 26) |
			     (cfg->green_offset << 21) |
			     (cfg->blue_offset << 16) |
			     (cfg->red_width << 8) |
			     (cfg->green_width << 4) |
			     cfg->blue_width, PP_DEST_FRAME_FMT_CNTL);
	} else {
		/* add YUV422 formatting */
		static const unsigned int _422[] = {
			0x62000888,
			0x60100888,
			0x43080888,
			0x41180888
		};

		__raw_writel(_422[(cfg->blue_offset >> 3) & 3],
			     PP_DEST_FRAME_FMT_CNTL);
		cfg->rgb_resolution = 16;
		r &= ~0xC00;
		r |= (cfg->rgb_resolution << 7);
	}

	/* add csc formatting */
	if (!cfg->csc_table[1]) {
		static const unsigned short _csc[][6] = {
			{0x80, 0xb4, 0x2c, 0x5b, 0x0e4, 0},
			{0x95, 0xcc, 0x32, 0x68, 0x104, 1},
			{0x80, 0xca, 0x18, 0x3c, 0x0ec, 0},
			{0x95, 0xe5, 0x1b, 0x44, 0x10e, 1},
		};
		memcpy(cfg->csc_table, _csc[cfg->csc_table[0]],
		       sizeof(_csc[0]));
	}
	__raw_writel((cfg->csc_table[0] << 24) |
		     (cfg->csc_table[1] << 16) |
		     (cfg->csc_table[2] << 8) |
		     cfg->csc_table[3], PP_CSC_COEF_0123);
	__raw_writel((cfg->csc_table[5] ? (1 << 9) : 0) | cfg->csc_table[4],
		     PP_CSC_COEF_4);

	__raw_writel(r, PP_CNTL);

	pphw_ptr(cfg);
	pphw_outptr(cfg);

	/*
	 * #MB in a row = input_width / 16pix
	 * 1 byte per QP per MB
	 * QP must be formatted to be 4-byte aligned
	 * YUV lines are to be 4-byte aligned as well
	 * So Y is 8 byte aligned, as U = V = Y/2 for 420
	 * MPEG MBs are 16x16 anyway
	 */
	__raw_writel((cfg->dim.in.width << 16) | cfg->dim.in.height,
		     PP_PROCESS_FRAME_PARA);
	__raw_writel(cfg->in_y_stride | (PP_CALC_QP_WIDTH(cfg) << 16),
		     PP_SOURCE_FRAME_WIDTH);

	/* in bytes */
	rt = cfg->rgb_resolution >> 3;
	if (rt == 3)
		rt = 4;
	__raw_writel(cfg->out_stride * rt, PP_DEST_DISPLAY_WIDTH);

	pp_dump();
	return 0;
}

/*!
 * @brief Check PP interrupt status.
 * @return	PP interrupt status
 */
static int pphw_isr(void)
{
	unsigned long status;

	pr_debug("pp: in isr.\n");
	status = __raw_readl(PP_INTRSTATUS) & 7;
	if (!status) {
		pr_debug("pp: not my isr err.\n");
		return status;
	}

	if (status & 4)
		pr_debug("pp: isr state error.\n");

	/* clear interrupt status */
	__raw_writel(status, PP_INTRSTATUS);

	return status;
}

static struct clk *emma_clk;

/*!
 * @brief PP module clock enable
 */
static void pphw_init(void)
{
	emma_clk = clk_get(NULL, "emma_clk");
	clk_enable(emma_clk);
}

/*!
 * @brief PP module clock disable
 */
static void pphw_exit(void)
{
	clk_disable(emma_clk);
	clk_put(emma_clk);
}
