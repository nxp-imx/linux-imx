/*
 * Freescale STMP378X PxP driver
 *
 * Author: Matt Porter <mporter@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008-2009 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/videodev2.h>

#include <media/videobuf-dma-contig.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>

#include <mach/platform.h>
#include <mach/regs-pxp.h>
#include <mach/lcdif.h>

#include "pxp.h"

#define PXP_DRIVER_NAME			"stmp3xxx-pxp"
#define PXP_DRIVER_MAJOR		1
#define PXP_DRIVER_MINOR		0

#define PXP_DEF_BUFS			2
#define PXP_MIN_PIX			8

#define V4L2_OUTPUT_TYPE_INTERNAL	4

#define PXP_WAITCON	((__raw_readl(HW_PXP_NEXT_ADDR) & BM_PXP_NEXT_ENABLED) \
				!= BM_PXP_NEXT_ENABLED)

#define REG_OFFSET	0x10
#define REGS1_NUMS	16
#define REGS2_NUMS	5
#define REGS3_NUMS	32
static u32 regs1[REGS1_NUMS];
static u32 regs2[REGS2_NUMS];
static u32 regs3[REGS3_NUMS];

static struct pxp_data_format pxp_s0_formats[] = {
	{
		.name = "24-bit RGB",
		.bpp = 4,
		.fourcc = V4L2_PIX_FMT_RGB24,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.ctrl_s0_fmt = BV_PXP_CTRL_S0_FORMAT__RGB888,
	}, {
		.name = "16-bit RGB 5:6:5",
		.bpp = 2,
		.fourcc = V4L2_PIX_FMT_RGB565,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.ctrl_s0_fmt = BV_PXP_CTRL_S0_FORMAT__RGB565,
	}, {
		.name = "16-bit RGB 5:5:5",
		.bpp = 2,
		.fourcc = V4L2_PIX_FMT_RGB555,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.ctrl_s0_fmt = BV_PXP_CTRL_S0_FORMAT__RGB555,
	}, {
		.name = "YUV 4:2:0 Planar",
		.bpp = 2,
		.fourcc = V4L2_PIX_FMT_YUV420,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.ctrl_s0_fmt = BV_PXP_CTRL_S0_FORMAT__YUV420,
	}, {
		.name = "YUV 4:2:2 Planar",
		.bpp = 2,
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.ctrl_s0_fmt = BV_PXP_CTRL_S0_FORMAT__YUV422,
	},
};

struct v4l2_queryctrl pxp_controls[] = {
	{
		.id 		= V4L2_CID_HFLIP,
		.type 		= V4L2_CTRL_TYPE_BOOLEAN,
		.name 		= "Horizontal Flip",
		.minimum 	= 0,
		.maximum 	= 1,
		.step 		= 1,
		.default_value	= 0,
		.flags		= 0,
	}, {
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Vertical Flip",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
		.flags		= 0,
	}, {
		.id		= V4L2_CID_PRIVATE_BASE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Rotation",
		.minimum	= 0,
		.maximum	= 270,
		.step		= 90,
		.default_value	= 0,
		.flags		= 0,
	}, {
		.id		= V4L2_CID_PRIVATE_BASE + 1,
		.name		= "Background Color",
		.minimum	= 0,
		.maximum	= 0xFFFFFF,
		.step		= 1,
		.default_value	= 0,
		.flags		= 0,
		.type		= V4L2_CTRL_TYPE_INTEGER,
	}, {
		.id		= V4L2_CID_PRIVATE_BASE + 2,
		.name		= "Set S0 Chromakey",
		.minimum	= -1,
		.maximum	= 0xFFFFFF,
		.step		= 1,
		.default_value	= -1,
		.flags		= 0,
		.type		= V4L2_CTRL_TYPE_INTEGER,
	}, {
		.id		= V4L2_CID_PRIVATE_BASE + 3,
		.name		= "YUV Colorspace",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
		.flags		= 0,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
	},
};

static void pxp_set_ctrl(struct pxps *pxp)
{
	u32 ctrl;

	ctrl = BF(pxp->s0_fmt->ctrl_s0_fmt, PXP_CTRL_S0_FORMAT);
	ctrl |=
	BF(BV_PXP_CTRL_OUTPUT_RGB_FORMAT__RGB888, PXP_CTRL_OUTPUT_RGB_FORMAT);
	ctrl |= BM_PXP_CTRL_CROP;

	if (pxp->scaling)
		ctrl |= BM_PXP_CTRL_SCALE;
	if (pxp->vflip)
		ctrl |= BM_PXP_CTRL_VFLIP;
	if (pxp->hflip)
		ctrl |= BM_PXP_CTRL_HFLIP;
	if (pxp->rotate)
		ctrl |= BF(pxp->rotate/90, PXP_CTRL_ROTATE);

	ctrl |= BM_PXP_CTRL_IRQ_ENABLE;
	if (pxp->active)
		ctrl |= BM_PXP_CTRL_ENABLE;

	__raw_writel(ctrl, HW_PXP_CTRL_ADDR);
	pxp->regs_virt->ctrl = ctrl;
}

static void pxp_set_rgbbuf(struct pxps *pxp)
{
	pxp->regs_virt->rgbbuf = pxp->outb_phys;
	/* Always equal to the FB size */
	pxp->regs_virt->rgbsize = BF(pxp->fb.fmt.width, PXP_RGBSIZE_WIDTH) |
				BF(pxp->fb.fmt.height, PXP_RGBSIZE_HEIGHT);
}

static void pxp_set_s0colorkey(struct pxps *pxp)
{
	/* Low and high are set equal. V4L does not allow a chromakey range */
	if (pxp->s0_chromakey == -1) {
		/* disable color key */
		pxp->regs_virt->s0colorkeylow = 0xFFFFFF;
		pxp->regs_virt->s0colorkeyhigh = 0;
	} else {
		pxp->regs_virt->s0colorkeylow = pxp->s0_chromakey;
		pxp->regs_virt->s0colorkeyhigh = pxp->s0_chromakey;
	}
}

static void pxp_set_s1colorkey(struct pxps *pxp)
{
	/* Low and high are set equal. V4L does not allow a chromakey range */
	if (pxp->s1_chromakey_state != 0 && pxp->s1_chromakey != -1) {
		pxp->regs_virt->olcolorkeylow = pxp->s1_chromakey;
		pxp->regs_virt->olcolorkeyhigh = pxp->s1_chromakey;
	} else {
		/* disable color key */
		pxp->regs_virt->olcolorkeylow = 0xFFFFFF;
		pxp->regs_virt->olcolorkeyhigh = 0;
	}
}

static void pxp_set_oln(struct pxps *pxp)
{
	pxp->regs_virt->ol0.ol = (u32)pxp->fb.base;
	pxp->regs_virt->ol0.olsize =
		BF(pxp->fb.fmt.width >> 3, PXP_OLnSIZE_WIDTH) |
		BF(pxp->fb.fmt.height >> 3, PXP_OLnSIZE_HEIGHT);
}

static void pxp_set_olparam(struct pxps *pxp)
{
	u32 olparam;
	struct v4l2_pix_format *fmt = &pxp->fb.fmt;

	olparam = BF(pxp->global_alpha, PXP_OLnPARAM_ALPHA);
	if (fmt->pixelformat == V4L2_PIX_FMT_RGB24)
		olparam |=
		BF(BV_PXP_OLnPARAM_FORMAT__RGB888, PXP_OLnPARAM_FORMAT);
	else
		olparam |=
		BF(BV_PXP_OLnPARAM_FORMAT__RGB565, PXP_OLnPARAM_FORMAT);
	if (pxp->global_alpha_state)
		olparam |= BF(BV_PXP_OLnPARAM_ALPHA_CNTL__Override,
					PXP_OLnPARAM_ALPHA_CNTL);
	if (pxp->s1_chromakey_state)
		olparam |= BM_PXP_OLnPARAM_ENABLE_COLORKEY;
	if (pxp->overlay_state)
		olparam |= BM_PXP_OLnPARAM_ENABLE;

	pxp->regs_virt->ol0.olparam = olparam;
}

static void pxp_set_s0param(struct pxps *pxp)
{
	u32 s0param;

	s0param = BF(pxp->drect.left >> 3, PXP_S0PARAM_XBASE);
	s0param |= BF(pxp->drect.top >> 3, PXP_S0PARAM_YBASE);
	s0param |= BF(pxp->s0_width >> 3, PXP_S0PARAM_WIDTH);
	s0param |= BF(pxp->s0_height >> 3, PXP_S0PARAM_HEIGHT);
	pxp->regs_virt->s0param = s0param;
}

static void pxp_set_s0crop(struct pxps *pxp)
{
	u32 s0crop;

	s0crop = BF(pxp->srect.left >> 3, PXP_S0CROP_XBASE);
	s0crop |= BF(pxp->srect.top >> 3, PXP_S0CROP_YBASE);
	s0crop |= BF(pxp->drect.width >> 3, PXP_S0CROP_WIDTH);
	s0crop |= BF(pxp->drect.height >> 3, PXP_S0CROP_HEIGHT);
	pxp->regs_virt->s0crop = s0crop;
}

static int pxp_set_scaling(struct pxps *pxp)
{
	int ret = 0;
	u32 xscale, yscale, s0scale;

	if ((pxp->s0_fmt->fourcc != V4L2_PIX_FMT_YUV420) &&
		(pxp->s0_fmt->fourcc != V4L2_PIX_FMT_YUV422P)) {
		pxp->scaling = 0;
		ret = -EINVAL;
		goto out;
	}

	if ((pxp->srect.width == pxp->drect.width) &&
		(pxp->srect.height == pxp->drect.height)) {
		pxp->scaling = 0;
		goto out;
	}

	pxp->scaling = 1;
	xscale = pxp->srect.width * 0x1000 / pxp->drect.width;
	yscale = pxp->srect.height * 0x1000 / pxp->drect.height;
	s0scale = BF(yscale, PXP_S0SCALE_YSCALE) |
		  BF(xscale, PXP_S0SCALE_XSCALE);
	pxp->regs_virt->s0scale = s0scale;

out:
	pxp_set_ctrl(pxp);

	return ret;
}

static int pxp_set_fbinfo(struct pxps *pxp)
{
	struct fb_var_screeninfo var;
	struct fb_fix_screeninfo fix;
	struct v4l2_framebuffer *fb = &pxp->fb;
	int err;

	err = stmp3xxxfb_get_info(&var, &fix);

	fb->fmt.width = var.xres;
	fb->fmt.height = var.yres;
	if (var.bits_per_pixel == 16)
		fb->fmt.pixelformat = V4L2_PIX_FMT_RGB565;
	else
		fb->fmt.pixelformat = V4L2_PIX_FMT_RGB24;
	fb->base = (void *)fix.smem_start;
	return err;
}

static void pxp_set_s0bg(struct pxps *pxp)
{
	pxp->regs_virt->s0background = pxp->s0_bgcolor;
}

static void pxp_set_csc(struct pxps *pxp)
{
	if (pxp->yuv) {
		/* YUV colorspace */
		__raw_writel(0x04030000, HW_PXP_CSCCOEFF0_ADDR);
		__raw_writel(0x01230208, HW_PXP_CSCCOEFF1_ADDR);
		__raw_writel(0x076b079c, HW_PXP_CSCCOEFF2_ADDR);
	} else {
		/* YCrCb colorspace */
		__raw_writel(0x84ab01f0, HW_PXP_CSCCOEFF0_ADDR);
		__raw_writel(0x01230204, HW_PXP_CSCCOEFF1_ADDR);
		__raw_writel(0x0730079c, HW_PXP_CSCCOEFF2_ADDR);
	}
}

static int pxp_set_cstate(struct pxps *pxp, struct v4l2_control *vc)
{

	if (vc->id == V4L2_CID_HFLIP)
		pxp->hflip = vc->value;
	else if (vc->id == V4L2_CID_VFLIP)
		pxp->vflip = vc->value;
	else if (vc->id == V4L2_CID_PRIVATE_BASE) {
		if (vc->value % 90)
			return -ERANGE;
		pxp->rotate = vc->value;
	} else if (vc->id == V4L2_CID_PRIVATE_BASE + 1) {
		pxp->s0_bgcolor = vc->value;
		pxp_set_s0bg(pxp);
	} else if (vc->id == V4L2_CID_PRIVATE_BASE + 2) {
		pxp->s0_chromakey = vc->value;
		pxp_set_s0colorkey(pxp);
	} else if (vc->id == V4L2_CID_PRIVATE_BASE + 3) {
		pxp->yuv = vc->value;
		pxp_set_csc(pxp);
	}

	pxp_set_ctrl(pxp);

	return 0;
}

static int pxp_get_cstate(struct pxps *pxp, struct v4l2_control *vc)
{
	if (vc->id == V4L2_CID_HFLIP)
		vc->value = pxp->hflip;
	else if (vc->id == V4L2_CID_VFLIP)
		vc->value = pxp->vflip;
	else if (vc->id == V4L2_CID_PRIVATE_BASE)
		vc->value = pxp->rotate;
	else if (vc->id == V4L2_CID_PRIVATE_BASE + 1)
		vc->value = pxp->s0_bgcolor;
	else if (vc->id == V4L2_CID_PRIVATE_BASE + 2)
		vc->value = pxp->s0_chromakey;
	else if (vc->id == V4L2_CID_PRIVATE_BASE + 3)
		vc->value = pxp->yuv;

	return 0;
}

static int pxp_enumoutput(struct file *file, void *fh,
			struct v4l2_output *o)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	if ((o->index < 0) || (o->index > 1))
		return -EINVAL;

	memset(o, 0, sizeof(struct v4l2_output));
	if (o->index == 0) {
		strcpy(o->name, "PxP Display Output");
		pxp->output = 0;
	} else {
		strcpy(o->name, "PxP Virtual Output");
		pxp->output = 1;
	}
	o->type = V4L2_OUTPUT_TYPE_INTERNAL;
	o->std = 0;
	o->reserved[0] = pxp->outb_phys;

	return 0;
}

static int pxp_g_output(struct file *file, void *fh,
			unsigned int *i)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	*i = pxp->output;

	return 0;
}

static int pxp_s_output(struct file *file, void *fh,
			unsigned int i)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));
	struct v4l2_pix_format *fmt = &pxp->fb.fmt;
	int bpp;

	if ((i < 0) || (i > 1))
		return -EINVAL;

	if (pxp->outb)
		goto out;

	/* Output buffer is same format as fbdev */
	if (fmt->pixelformat == V4L2_PIX_FMT_RGB24)
		bpp = 4;
	else
		bpp = 2;

	pxp->outb = kmalloc(fmt->width * fmt->height * bpp, GFP_KERNEL);
	pxp->outb_phys = virt_to_phys(pxp->outb);
	dma_map_single(NULL, pxp->outb,
			fmt->width * fmt->height * bpp, DMA_TO_DEVICE);

out:
	pxp_set_rgbbuf(pxp);

	return 0;
}

static int pxp_enum_fmt_video_output(struct file *file, void *fh,
				struct v4l2_fmtdesc *fmt)
{
	enum v4l2_buf_type type = fmt->type;
	int index = fmt->index;

	if ((fmt->index < 0) || (fmt->index >= ARRAY_SIZE(pxp_s0_formats)))
		return -EINVAL;

	memset(fmt, 0, sizeof(struct v4l2_fmtdesc));
	fmt->index = index;
	fmt->type = type;
	fmt->pixelformat = pxp_s0_formats[index].fourcc;
	strcpy(fmt->description, pxp_s0_formats[index].name);

	return 0;
}

static int pxp_g_fmt_video_output(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct v4l2_pix_format *pf = &f->fmt.pix;
	struct pxps *pxp = video_get_drvdata(video_devdata(file));
	struct pxp_data_format *fmt = pxp->s0_fmt;

	pf->width = pxp->s0_width;
	pf->height = pxp->s0_height;
	pf->pixelformat = fmt->fourcc;
	pf->field = V4L2_FIELD_NONE;
	pf->bytesperline = fmt->bpp * pf->width;
	pf->sizeimage = pf->bytesperline * pf->height;
	pf->colorspace = fmt->colorspace;
	pf->priv = 0;

	return 0;
}

static struct pxp_data_format *pxp_get_format(struct v4l2_format *f)
{
	struct pxp_data_format *fmt;
	int i;

	for (i = 0; i < ARRAY_SIZE(pxp_s0_formats); i++) {
		fmt = &pxp_s0_formats[i];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			break;
	}

	if (i == ARRAY_SIZE(pxp_s0_formats))
		return NULL;

	return &pxp_s0_formats[i];
}

static int pxp_try_fmt_video_output(struct file *file, void *fh,
				struct v4l2_format *f)
{
	int w = f->fmt.pix.width;
	int h = f->fmt.pix.height;
	struct pxp_data_format *fmt = pxp_get_format(f);

	if (!fmt)
		return -EINVAL;

	w = min(w, 2040);
	w = max(w, 8);
	h = min(h, 2040);
	h = max(h, 8);
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.width = w;
	f->fmt.pix.height = h;
	f->fmt.pix.pixelformat = fmt->fourcc;

	return 0;
}

static int pxp_s_fmt_video_output(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));
	struct v4l2_pix_format *pf = &f->fmt.pix;
	int ret = pxp_try_fmt_video_output(file, fh, f);

	if (ret == 0) {
		pxp->s0_fmt = pxp_get_format(f);
		pxp->s0_width = pf->width;
		pxp->s0_height = pf->height;
		pxp_set_ctrl(pxp);
		pxp_set_s0param(pxp);
	}

	return ret;
}

static int pxp_g_fmt_output_overlay(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));
	struct v4l2_window *wf = &f->fmt.win;

	memset(wf, 0, sizeof(struct v4l2_window));
	wf->chromakey = pxp->s1_chromakey;
	wf->global_alpha = pxp->global_alpha;
	wf->field = V4L2_FIELD_NONE;
	wf->clips = NULL;
	wf->clipcount = 0;
	wf->bitmap = NULL;
	wf->w.left = pxp->srect.left;
	wf->w.top = pxp->srect.top;
	wf->w.width = pxp->srect.width;
	wf->w.height = pxp->srect.height;

	return 0;
}

static int pxp_try_fmt_output_overlay(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));
	struct v4l2_window *wf = &f->fmt.win;
	struct v4l2_rect srect;
	u32 s1_chromakey = wf->chromakey;
	u8 global_alpha = wf->global_alpha;

	memcpy(&srect, &(wf->w), sizeof(struct v4l2_rect));

	pxp_g_fmt_output_overlay(file, fh, f);

	wf->chromakey = s1_chromakey;
	wf->global_alpha = global_alpha;

	/* Constrain parameters to the input buffer */
	wf->w.left = srect.left;
	wf->w.top = srect.top;
	wf->w.width = min(srect.width, ((__s32)pxp->s0_width - wf->w.left));
	wf->w.height = min(srect.height, ((__s32)pxp->s0_height - wf->w.top));

	return 0;
}

static int pxp_s_fmt_output_overlay(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));
	struct v4l2_window *wf = &f->fmt.win;
	int ret = pxp_try_fmt_output_overlay(file, fh, f);

	if (ret == 0) {
		pxp->srect.left = wf->w.left;
		pxp->srect.top = wf->w.top;
		pxp->srect.width = wf->w.width;
		pxp->srect.height = wf->w.height;
		pxp->global_alpha = wf->global_alpha;
		pxp->s1_chromakey = wf->chromakey;
		pxp_set_s0param(pxp);
		pxp_set_s0crop(pxp);
		pxp_set_scaling(pxp);
		pxp_set_olparam(pxp);
		pxp_set_s1colorkey(pxp);
	}

	return ret;
}

static int pxp_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *r)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	return videobuf_reqbufs(&pxp->s0_vbq, r);
}

static int pxp_querybuf(struct file *file, void *priv,
			struct v4l2_buffer *b)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	return videobuf_querybuf(&pxp->s0_vbq, b);
}

static int pxp_qbuf(struct file *file, void *priv,
			struct v4l2_buffer *b)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	return videobuf_qbuf(&pxp->s0_vbq, b);
}

static int pxp_dqbuf(struct file *file, void *priv,
			struct v4l2_buffer *b)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	return videobuf_dqbuf(&pxp->s0_vbq, b, file->f_flags & O_NONBLOCK);
}

static int pxp_streamon(struct file *file, void *priv,
			enum v4l2_buf_type t)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));
	int ret = 0;

	if ((t != V4L2_BUF_TYPE_VIDEO_OUTPUT))
		return -EINVAL;

	ret = videobuf_streamon(&pxp->s0_vbq);

	if (!ret && (pxp->output == 0))
		stmp3xxxfb_cfg_pxp(1, pxp->outb_phys);

	return ret;
}

static int pxp_streamoff(struct file *file, void *priv,
			enum v4l2_buf_type t)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));
	int ret = 0;

	if ((t != V4L2_BUF_TYPE_VIDEO_OUTPUT))
		return -EINVAL;

	ret = videobuf_streamoff(&pxp->s0_vbq);

	if (!ret)
		stmp3xxxfb_cfg_pxp(0, 0);

	return ret;
}

static int pxp_buf_setup(struct videobuf_queue *q,
			unsigned int *count, unsigned *size)
{
	struct pxps *pxp = q->priv_data;

	*size = pxp->s0_width * pxp->s0_height * pxp->s0_fmt->bpp;

	if (0 == *count)
		*count = PXP_DEF_BUFS;

	return 0;
}

static void pxp_buf_free(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	if (in_interrupt())
		BUG();

	videobuf_dma_contig_free(q, vb);

	vb->state = VIDEOBUF_NEEDS_INIT;
}

static int pxp_buf_prepare(struct videobuf_queue *q,
			struct videobuf_buffer *vb,
			enum v4l2_field field)
{
	struct pxps *pxp = q->priv_data;
	int ret = 0;

	vb->width = pxp->s0_width;
	vb->height = pxp->s0_height;
	vb->size = vb->width * vb->height * pxp->s0_fmt->bpp;
	vb->field = V4L2_FIELD_NONE;
	vb->state = VIDEOBUF_NEEDS_INIT;

	ret = videobuf_iolock(q, vb, NULL);
	if (ret)
		goto fail;
	vb->state = VIDEOBUF_PREPARED;

	return 0;

fail:
	pxp_buf_free(q, vb);
	return ret;
}

static void pxp_buf_next(struct pxps *pxp)
{
	dma_addr_t Y, U, V;

	if (pxp->active) {
		pxp->active->state = VIDEOBUF_ACTIVE;
		Y = videobuf_to_dma_contig(pxp->active);
		pxp->regs_virt->s0buf = Y;
		if ((pxp->s0_fmt->fourcc == V4L2_PIX_FMT_YUV420) ||
		    (pxp->s0_fmt->fourcc == V4L2_PIX_FMT_YUV422P)) {
			int s = 1;	/* default to YUV 4:2:2 */
			if (pxp->s0_fmt->fourcc == V4L2_PIX_FMT_YUV420)
				s = 2;
			U = Y + (pxp->s0_width * pxp->s0_height);
			V = U + ((pxp->s0_width * pxp->s0_height) >> s);
			pxp->regs_virt->s0ubuf = U;
			pxp->regs_virt->s0vbuf = V;
		}
		pxp->regs_virt->ctrl =
			__raw_readl(HW_PXP_CTRL_ADDR) | BM_PXP_CTRL_ENABLE;
	}

	__raw_writel(pxp->regs_phys, HW_PXP_NEXT_ADDR);
}

static void pxp_next_handle(struct work_struct *w)
{
	struct pxps *pxp = container_of(w, struct pxps, work);
	struct pxp_buffer *buf, *next;
	unsigned long flags;

	if (pxp->next_queue_ended == 1)
		return;

	spin_lock_irqsave(&pxp->lock, flags);

	while (!list_empty(&pxp->nextq)) {
		spin_unlock_irqrestore(&pxp->lock, flags);

		if (!wait_event_interruptible_timeout(pxp->done, PXP_WAITCON,
					5 * HZ) || signal_pending(current)) {
			spin_lock_irqsave(&pxp->lock, flags);
			list_for_each_entry_safe(buf, next, &pxp->nextq, queue)
				list_del(&buf->queue);
			spin_unlock_irqrestore(&pxp->lock, flags);
			pxp->next_queue_ended = 1;
			return;
		}

		spin_lock_irqsave(&pxp->lock, flags);
		buf = list_entry(pxp->nextq.next,
					struct pxp_buffer,
					queue);
		list_del_init(&buf->queue);
		pxp->active = &buf->vb;
		pxp->active->state = VIDEOBUF_QUEUED;
		pxp_buf_next(pxp);
	}

	spin_unlock_irqrestore(&pxp->lock, flags);
}

static void pxp_buf_queue(struct videobuf_queue *q,
			struct videobuf_buffer *vb)
{
	struct pxps *pxp = q->priv_data;
	struct pxp_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&pxp->lock, flags);

	if (list_empty(&pxp->outq)) {
		list_add_tail(&vb->queue, &pxp->outq);
		vb->state = VIDEOBUF_QUEUED;

		pxp->active = vb;
		pxp_buf_next(pxp);
	} else {
		list_add_tail(&vb->queue, &pxp->outq);

		buf = container_of(vb, struct pxp_buffer, vb);
		list_add_tail(&buf->queue, &pxp->nextq);
		queue_work(pxp->workqueue, &pxp->work);
	}

	spin_unlock_irqrestore(&pxp->lock, flags);
}

static void pxp_buf_release(struct videobuf_queue *q,
			struct videobuf_buffer *vb)
{
	pxp_buf_free(q, vb);
}

static struct videobuf_queue_ops pxp_vbq_ops = {
	.buf_setup	= pxp_buf_setup,
	.buf_prepare	= pxp_buf_prepare,
	.buf_queue	= pxp_buf_queue,
	.buf_release	= pxp_buf_release,
};

static int pxp_querycap(struct file *file, void *fh,
			struct v4l2_capability *cap)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	memset(cap, 0, sizeof(*cap));
	strcpy(cap->driver, "pxp");
	strcpy(cap->card, "pxp");
	strlcpy(cap->bus_info, dev_name(&pxp->pdev->dev), sizeof(cap->bus_info));

	cap->version = (PXP_DRIVER_MAJOR << 8) + PXP_DRIVER_MINOR;

	cap->capabilities = V4L2_CAP_VIDEO_OUTPUT |
				V4L2_CAP_VIDEO_OUTPUT_OVERLAY |
				V4L2_CAP_STREAMING;

	return 0;
}

static int pxp_g_fbuf(struct file *file, void *priv,
			struct v4l2_framebuffer *fb)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	memset(fb, 0, sizeof(*fb));

	fb->capability = V4L2_FBUF_CAP_EXTERNOVERLAY |
			 V4L2_FBUF_CAP_CHROMAKEY |
			 V4L2_FBUF_CAP_LOCAL_ALPHA |
			 V4L2_FBUF_CAP_GLOBAL_ALPHA;

	if (pxp->global_alpha_state)
		fb->flags |= V4L2_FBUF_FLAG_GLOBAL_ALPHA;
	if (pxp->local_alpha_state)
		fb->flags |= V4L2_FBUF_FLAG_LOCAL_ALPHA;
	if (pxp->s1_chromakey_state)
		fb->flags |= V4L2_FBUF_FLAG_CHROMAKEY;

	return 0;
}

static int pxp_s_fbuf(struct file *file, void *priv,
			struct v4l2_framebuffer *fb)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	pxp->overlay_state =
		(fb->flags & V4L2_FBUF_FLAG_OVERLAY) != 0;
	pxp->global_alpha_state =
		(fb->flags & V4L2_FBUF_FLAG_GLOBAL_ALPHA) != 0;
	pxp->local_alpha_state =
		(fb->flags & V4L2_FBUF_FLAG_LOCAL_ALPHA) != 0;
	/* Global alpha overrides local alpha if both are requested */
	if (pxp->global_alpha_state && pxp->local_alpha_state)
		pxp->local_alpha_state = 0;
	pxp->s1_chromakey_state =
		(fb->flags & V4L2_FBUF_FLAG_CHROMAKEY) != 0;

	pxp_set_olparam(pxp);
	pxp_set_s0crop(pxp);
	pxp_set_scaling(pxp);

	return 0;
}

static int pxp_g_crop(struct file *file, void *fh,
			struct v4l2_crop *c)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	if (c->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY)
		return -EINVAL;

	c->c.left = pxp->drect.left;
	c->c.top = pxp->drect.top;
	c->c.width = pxp->drect.width;
	c->c.height = pxp->drect.height;

	return 0;
}

static int pxp_s_crop(struct file *file, void *fh,
			struct v4l2_crop *c)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));
	int l = c->c.left;
	int t = c->c.top;
	int w = c->c.width;
	int h = c->c.height;
	int fbw = pxp->fb.fmt.width;
	int fbh = pxp->fb.fmt.height;

	if (c->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY)
		return -EINVAL;

	/* Constrain parameters to FB limits */
	w = min(w, fbw);
	w = max(w, PXP_MIN_PIX);
	h = min(h, fbh);
	h = max(h, PXP_MIN_PIX);
	if ((l + w) > fbw)
		l = 0;
	if ((t + h) > fbh)
		t = 0;

	/* Round up values to PxP pixel block */
	l = roundup(l, PXP_MIN_PIX);
	t = roundup(t, PXP_MIN_PIX);
	w = roundup(w, PXP_MIN_PIX);
	h = roundup(h, PXP_MIN_PIX);

	pxp->drect.left = l;
	pxp->drect.top = t;
	pxp->drect.width = w;
	pxp->drect.height = h;

	pxp_set_s0param(pxp);
	pxp_set_s0crop(pxp);
	pxp_set_scaling(pxp);

	return 0;
}

static int pxp_queryctrl(struct file *file, void *priv,
			 struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pxp_controls); i++)
		if (qc->id && qc->id == pxp_controls[i].id) {
			memcpy(qc, &(pxp_controls[i]), sizeof(*qc));
			return 0;
		}

	return -EINVAL;
}

static int pxp_g_ctrl(struct file *file, void *priv,
			 struct v4l2_control *vc)
{
	int i;

	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	for (i = 0; i < ARRAY_SIZE(pxp_controls); i++)
		if (vc->id == pxp_controls[i].id)
			return pxp_get_cstate(pxp, vc);

	return -EINVAL;
}

static int pxp_s_ctrl(struct file *file, void *priv,
			 struct v4l2_control *vc)
{
	int i;
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	for (i = 0; i < ARRAY_SIZE(pxp_controls); i++)
		if (vc->id == pxp_controls[i].id) {
			if (vc->value < pxp_controls[i].minimum ||
			    vc->value > pxp_controls[i].maximum)
				return -ERANGE;
			return pxp_set_cstate(pxp, vc);
		}

	return -EINVAL;
}

void pxp_release(struct video_device *vfd)
{
	struct pxps *pxp = video_get_drvdata(vfd);

	spin_lock(&pxp->lock);
	video_device_release(vfd);
	spin_unlock(&pxp->lock);
}

static int pxp_hw_init(struct pxps *pxp)
{
	struct fb_var_screeninfo var;
	struct fb_fix_screeninfo fix;
	int err;

	err = stmp3xxxfb_get_info(&var, &fix);
	if (err)
		return err;

	/* Pull PxP out of reset */
	__raw_writel(0, HW_PXP_CTRL_ADDR);

	/* Config defaults */
	pxp->active = NULL;

	pxp->s0_fmt = &pxp_s0_formats[0];
	pxp->drect.left = pxp->srect.left = 0;
	pxp->drect.top = pxp->srect.top = 0;
	pxp->drect.width = pxp->srect.width = pxp->s0_width = var.xres;
	pxp->drect.height = pxp->srect.height = pxp->s0_height = var.yres;
	pxp->s0_bgcolor = 0;

	pxp->output = 0;
	err = pxp_set_fbinfo(pxp);
	if (err)
		return err;

	pxp->scaling = 0;
	pxp->hflip = 0;
	pxp->vflip = 0;
	pxp->rotate = 0;
	pxp->yuv = 0;

	pxp->overlay_state = 0;
	pxp->global_alpha_state = 0;
	pxp->global_alpha = 0;
	pxp->local_alpha_state = 0;
	pxp->s1_chromakey_state = 0;
	pxp->s1_chromakey = -1;
	pxp->s0_chromakey = -1;

	/* Write default h/w config */
	pxp_set_ctrl(pxp);
	pxp_set_s0param(pxp);
	pxp_set_s0crop(pxp);
	pxp_set_oln(pxp);
	pxp_set_olparam(pxp);
	pxp_set_s0colorkey(pxp);
	pxp_set_s1colorkey(pxp);
	pxp_set_csc(pxp);

	return 0;
}

static int pxp_open(struct file *file)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));
	int ret = 0;

	mutex_lock(&pxp->mutex);
	pxp->users++;

	if (pxp->users > 1) {
		pxp->users--;
		ret = -EBUSY;
		goto out;
	}
out:
	mutex_unlock(&pxp->mutex);
	if (ret)
		return ret;

	pxp->next_queue_ended = 0;
	pxp->workqueue = create_singlethread_workqueue("pxp");

	videobuf_queue_dma_contig_init(&pxp->s0_vbq,
				&pxp_vbq_ops,
				&pxp->pdev->dev,
				&pxp->lock,
				V4L2_BUF_TYPE_VIDEO_OUTPUT,
				V4L2_FIELD_NONE,
				sizeof(struct pxp_buffer),
				pxp);

	return 0;
}

static int pxp_close(struct file *file)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));

	if (pxp->workqueue)
		destroy_workqueue(pxp->workqueue);

	videobuf_stop(&pxp->s0_vbq);
	videobuf_mmap_free(&pxp->s0_vbq);
	pxp->active = NULL;

	mutex_lock(&pxp->mutex);
	pxp->users--;
	mutex_unlock(&pxp->mutex);

	return 0;
}

static int pxp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct pxps *pxp = video_get_drvdata(video_devdata(file));
	int ret;

	ret = videobuf_mmap_mapper(&pxp->s0_vbq, vma);

	return ret;
}

static const struct v4l2_file_operations pxp_fops = {
	.owner		= THIS_MODULE,
	.open		= pxp_open,
	.release	= pxp_close,
	.ioctl		= video_ioctl2,
	.mmap		= pxp_mmap,
};

static const struct v4l2_ioctl_ops pxp_ioctl_ops = {
	.vidioc_querycap		= pxp_querycap,

	.vidioc_reqbufs			= pxp_reqbufs,
	.vidioc_querybuf		= pxp_querybuf,
	.vidioc_qbuf			= pxp_qbuf,
	.vidioc_dqbuf			= pxp_dqbuf,

	.vidioc_streamon		= pxp_streamon,
	.vidioc_streamoff		= pxp_streamoff,

	.vidioc_enum_output		= pxp_enumoutput,
	.vidioc_g_output		= pxp_g_output,
	.vidioc_s_output		= pxp_s_output,

	.vidioc_enum_fmt_vid_out	= pxp_enum_fmt_video_output,
	.vidioc_try_fmt_vid_out		= pxp_try_fmt_video_output,
	.vidioc_g_fmt_vid_out		= pxp_g_fmt_video_output,
	.vidioc_s_fmt_vid_out		= pxp_s_fmt_video_output,

	.vidioc_try_fmt_vid_out_overlay	= pxp_try_fmt_output_overlay,
	.vidioc_g_fmt_vid_out_overlay	= pxp_g_fmt_output_overlay,
	.vidioc_s_fmt_vid_out_overlay	= pxp_s_fmt_output_overlay,

	.vidioc_g_fbuf			= pxp_g_fbuf,
	.vidioc_s_fbuf			= pxp_s_fbuf,

	.vidioc_g_crop			= pxp_g_crop,
	.vidioc_s_crop			= pxp_s_crop,

	.vidioc_queryctrl		= pxp_queryctrl,
	.vidioc_g_ctrl			= pxp_g_ctrl,
	.vidioc_s_ctrl			= pxp_s_ctrl,
};

static const struct video_device pxp_template = {
	.name				= "PxP",
	.vfl_type 			= V4L2_CAP_VIDEO_OUTPUT |
					  V4L2_CAP_VIDEO_OVERLAY |
					  V4L2_CAP_STREAMING,
	.fops				= &pxp_fops,
	.release			= pxp_release,
	.minor				= -1,
	.ioctl_ops			= &pxp_ioctl_ops,
};

static irqreturn_t pxp_irq(int irq, void *dev_id)
{
	struct pxps *pxp = (struct pxps *)dev_id;
	struct videobuf_buffer *vb;
	unsigned long flags;

	spin_lock_irqsave(&pxp->lock, flags);

	__raw_writel(BM_PXP_STAT_IRQ, HW_PXP_STAT_CLR_ADDR);

	if (list_empty(&pxp->outq)) {
		pr_warning("irq: outq empty!!!\n");
		goto out;
	}

	vb = list_entry(pxp->outq.next,
				struct videobuf_buffer,
				queue);
	list_del_init(&vb->queue);

	vb->state = VIDEOBUF_DONE;
	do_gettimeofday(&vb->ts);
	vb->field_count++;

	wake_up(&vb->done);
	wake_up(&pxp->done);
out:
	spin_unlock_irqrestore(&pxp->lock, flags);

	return IRQ_HANDLED;
}

static int pxp_notifier_callback(struct notifier_block *self,
		       unsigned long event, void *data)
{
	struct pxps *pxp = container_of(self, struct pxps, nb);

	switch (event) {
	case STMP3XXX_LCDIF_PANEL_INIT:
		pxp_set_fbinfo(pxp);
		break;
	default:
		break;
	}
	return 0;
}

static int pxp_probe(struct platform_device *pdev)
{
	struct pxps *pxp;
	struct resource *res;
	int irq;
	int err = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || irq < 0) {
		err = -ENODEV;
		goto exit;
	}

	pxp = kzalloc(sizeof(*pxp), GFP_KERNEL);
	if (!pxp) {
		dev_err(&pdev->dev, "failed to allocate control object\n");
		err = -ENOMEM;
		goto exit;
	}

	dev_set_drvdata(&pdev->dev, pxp);
	pxp->res = res;
	pxp->irq = irq;

	pxp->regs_virt = dma_alloc_coherent(NULL,
				PAGE_ALIGN(sizeof(struct pxp_registers)),
				&pxp->regs_phys, GFP_KERNEL);
	if (pxp->regs_virt == NULL) {
		dev_err(&pdev->dev, "failed to allocate pxp_register object\n");
		err = -ENOMEM;
		goto exit;
	}

	init_waitqueue_head(&pxp->done);

	INIT_WORK(&pxp->work, pxp_next_handle);
	INIT_LIST_HEAD(&pxp->outq);
	INIT_LIST_HEAD(&pxp->nextq);
	spin_lock_init(&pxp->lock);
	mutex_init(&pxp->mutex);

	if (!request_mem_region(res->start, res->end - res->start + 1,
				PXP_DRIVER_NAME)) {
		err = -EBUSY;
		goto freepxp;
	}

	pxp->regs = (void __iomem *)res->start; /* it is already ioremapped */
	pxp->pdev = pdev;

	err = request_irq(pxp->irq, pxp_irq, 0, PXP_DRIVER_NAME, pxp);

	if (err) {
		dev_err(&pdev->dev, "interrupt register failed\n");
		goto release;
	}

	pxp->vdev = video_device_alloc();
	if (!pxp->vdev) {
		dev_err(&pdev->dev, "video_device_alloc() failed\n");
		err = -ENOMEM;
		goto freeirq;
	}

	memcpy(pxp->vdev, &pxp_template, sizeof(pxp_template));
	video_set_drvdata(pxp->vdev, pxp);

	err = video_register_device(pxp->vdev, VFL_TYPE_GRABBER, 0);
	if (err) {
		dev_err(&pdev->dev, "failed to register video device\n");
		goto freevdev;
	}

	err = pxp_hw_init(pxp);
	if (err) {
		dev_err(&pdev->dev, "failed to initialize hardware\n");
		goto freevdev;
	}

	pxp->nb.notifier_call = pxp_notifier_callback,
	stmp3xxx_lcdif_register_client(&pxp->nb);
	dev_info(&pdev->dev, "initialized\n");

exit:
	return err;

freevdev:
	video_device_release(pxp->vdev);

freeirq:
	free_irq(pxp->irq, pxp);

release:
	release_mem_region(res->start, res->end - res->start + 1);

freepxp:
	kfree(pxp);

	return err;
}

static int __devexit pxp_remove(struct platform_device *pdev)
{
	struct pxps *pxp = platform_get_drvdata(pdev);

	stmp3xxx_lcdif_unregister_client(&pxp->nb);
	video_unregister_device(pxp->vdev);
	video_device_release(pxp->vdev);

	if (pxp->regs_virt)
		dma_free_coherent(0, PAGE_ALIGN(sizeof(struct pxp_registers)),
				pxp->regs_virt, pxp->regs_phys);
	kfree(pxp->outb);
	kfree(pxp);

	return 0;
}

#ifdef CONFIG_PM
static int pxp_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;

	while (__raw_readl(HW_PXP_CTRL_ADDR) & BM_PXP_CTRL_ENABLE)
		;

	for (i = 0; i < REGS1_NUMS; i++)
		regs1[i] = __raw_readl(HW_PXP_CTRL_ADDR + REG_OFFSET * i);

	for (i = 0; i < REGS2_NUMS; i++)
		regs2[i] = __raw_readl(HW_PXP_PAGETABLE_ADDR + REG_OFFSET * i);

	for (i = 0; i < REGS3_NUMS; i++)
		regs3[i] = __raw_readl(HW_PXP_OLn_ADDR(0) + REG_OFFSET * i);

	__raw_writel(BM_PXP_CTRL_SFTRST, HW_PXP_CTRL_ADDR);

	return 0;
}

static int pxp_resume(struct platform_device *pdev)
{
	int i;

	/* Pull PxP out of reset */
	__raw_writel(0, HW_PXP_CTRL_ADDR);

	for (i = 0; i < REGS1_NUMS; i++)
		__raw_writel(regs1[i], HW_PXP_CTRL_ADDR + REG_OFFSET * i);

	for (i = 0; i < REGS2_NUMS; i++)
		__raw_writel(regs2[i], HW_PXP_PAGETABLE_ADDR + REG_OFFSET * i);

	for (i = 0; i < REGS3_NUMS; i++)
		__raw_writel(regs3[i], HW_PXP_OLn_ADDR(0) + REG_OFFSET * i);

	return 0;
}
#else
#define	pxp_suspend	NULL
#define	pxp_resume	NULL
#endif

static struct platform_driver pxp_driver = {
	.driver 	= {
		.name	= PXP_DRIVER_NAME,
	},
	.probe		= pxp_probe,
	.remove		= __exit_p(pxp_remove),
	.suspend	= pxp_suspend,
	.resume		= pxp_resume,
};


static int __devinit pxp_init(void)
{
	return platform_driver_register(&pxp_driver);
}

static void __exit pxp_exit(void)
{
	platform_driver_unregister(&pxp_driver);
}

module_init(pxp_init);
module_exit(pxp_exit);

MODULE_DESCRIPTION("STMP37xx PxP driver");
MODULE_AUTHOR("Matt Porter <mporter@embeddedalley.com>");
MODULE_LICENSE("GPL");
