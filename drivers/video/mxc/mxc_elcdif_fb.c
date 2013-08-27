/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
/*
 * Based on drivers/video/mxc/mxc_ipuv3_fb.c, drivers/video/mxs/lcdif.c
 *	    and arch/arm/mach-mx28/include/mach/lcdif.h.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/fsl_devices.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/mxcfb.h>
#include <linux/uaccess.h>

#include <mach/hardware.h>

#include "elcdif_regs.h"

/*  ELCDIF Pixel format definitions */
/*  Four-character-code (FOURCC) */
#define fourcc(a, b, c, d) \
	(((__u32)(a)<<0)|((__u32)(b)<<8)|((__u32)(c)<<16)|((__u32)(d)<<24))

/*
 * ELCDIF RGB Formats
 */
#define ELCDIF_PIX_FMT_RGB332  fourcc('R', 'G', 'B', '1')
#define ELCDIF_PIX_FMT_RGB555  fourcc('R', 'G', 'B', 'O')
#define ELCDIF_PIX_FMT_RGB565  fourcc('R', 'G', 'B', 'P')
#define ELCDIF_PIX_FMT_RGB666  fourcc('R', 'G', 'B', '6')
#define ELCDIF_PIX_FMT_BGR666  fourcc('B', 'G', 'R', '6')
#define ELCDIF_PIX_FMT_BGR24   fourcc('B', 'G', 'R', '3')
#define ELCDIF_PIX_FMT_RGB24   fourcc('R', 'G', 'B', '3')
#define ELCDIF_PIX_FMT_BGR32   fourcc('B', 'G', 'R', '4')
#define ELCDIF_PIX_FMT_BGRA32  fourcc('B', 'G', 'R', 'A')
#define ELCDIF_PIX_FMT_RGB32   fourcc('R', 'G', 'B', '4')
#define ELCDIF_PIX_FMT_RGBA32  fourcc('R', 'G', 'B', 'A')
#define ELCDIF_PIX_FMT_ABGR32  fourcc('A', 'B', 'G', 'R')

struct mxc_elcdif_fb_data {
	int cur_blank;
	int next_blank;
	int output_pix_fmt;
	int dma_irq;
	bool wait4vsync;
	bool wait4framedone;
	bool panning;
	bool running;
	struct completion vsync_complete;
	struct completion frame_done_complete;
	ktime_t vsync_nf_timestamp;
	struct semaphore flip_sem;
	struct fb_var_screeninfo var;
	u32 pseudo_palette[16];
};

struct elcdif_signal_cfg {
	unsigned clk_pol:1;	/* true = falling edge */
	unsigned enable_pol:1;	/* true = active high */
	unsigned Hsync_pol:1;	/* true = active high */
	unsigned Vsync_pol:1;	/* true = active high */
};

struct mxcfb_mode {
	int dev_mode;
	int num_modes;
	struct fb_videomode *mode;
};

static int mxc_elcdif_fb_blank(int blank, struct fb_info *info);
static int mxc_elcdif_fb_map_video_memory(struct fb_info *info);
static int mxc_elcdif_fb_unmap_video_memory(struct fb_info *info);
static char *fb_mode;
static unsigned long default_bpp = 16;
static void __iomem *elcdif_base;
static struct device *g_elcdif_dev;
static bool g_elcdif_axi_clk_enable;
static bool g_elcdif_pix_clk_enable;
static struct clk *g_elcdif_axi_clk;
static struct clk *g_elcdif_pix_clk;
static struct mxcfb_mode mxc_disp_mode;

static void dump_fb_videomode(struct fb_videomode *m)
{
	pr_debug("fb_videomode = %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
		m->refresh, m->xres, m->yres, m->pixclock, m->left_margin,
		m->right_margin, m->upper_margin, m->lower_margin,
		m->hsync_len, m->vsync_len, m->sync, m->vmode, m->flag);
}

static inline void setup_dotclk_panel(u32 pixel_clk,
				      u16 v_pulse_width,
				      u16 v_period,
				      u16 v_wait_cnt,
				      u16 v_active,
				      u16 h_pulse_width,
				      u16 h_period,
				      u16 h_wait_cnt,
				      u16 h_active,
				      int in_pixel_format,
				      int out_pixel_format,
				      struct elcdif_signal_cfg sig_cfg,
				      int enable_present)
{
	u32 val, rounded_pixel_clk;
	u32 rounded_parent_clk;
	struct clk *clk_parent;

	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	/* Init clocking */
	dev_dbg(g_elcdif_dev, "pixel clk = %d\n", pixel_clk);

	clk_parent = clk_get_parent(g_elcdif_pix_clk);
	if (clk_parent == NULL)
		dev_err(g_elcdif_dev, "%s Failed get clk parent\n", __func__);

	rounded_pixel_clk = pixel_clk * 2;

	rounded_parent_clk = clk_round_rate(clk_parent,
				rounded_pixel_clk);

	while (rounded_pixel_clk < rounded_parent_clk) {
		/* the max divider from parent to di is 8 */
		if (rounded_parent_clk / pixel_clk < 8)
			rounded_pixel_clk += pixel_clk * 2;
	}

	clk_set_rate(clk_parent, rounded_pixel_clk);
	rounded_pixel_clk =
		clk_round_rate(g_elcdif_pix_clk, pixel_clk);
	clk_set_rate(g_elcdif_pix_clk, rounded_pixel_clk);

	msleep(5);

	__raw_writel(BM_ELCDIF_CTRL_DATA_SHIFT_DIR,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);

	__raw_writel(BM_ELCDIF_CTRL_SHIFT_NUM_BITS,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);

	__raw_writel(BM_ELCDIF_CTRL2_OUTSTANDING_REQS,
		     elcdif_base + HW_ELCDIF_CTRL2_CLR);
	__raw_writel(BF_ELCDIF_CTRL2_OUTSTANDING_REQS
		    (BV_ELCDIF_CTRL2_OUTSTANDING_REQS__REQ_16),
		     elcdif_base + HW_ELCDIF_CTRL2_SET);

	/* Recover on underflow */
	__raw_writel(BM_ELCDIF_CTRL1_RECOVER_ON_UNDERFLOW,
		     elcdif_base + HW_ELCDIF_CTRL1_SET);

	/* Configure the input pixel format */
	__raw_writel(BM_ELCDIF_CTRL_WORD_LENGTH |
		     BM_ELCDIF_CTRL_INPUT_DATA_SWIZZLE |
		     BM_ELCDIF_CTRL_DATA_FORMAT_16_BIT |
		     BM_ELCDIF_CTRL_DATA_FORMAT_18_BIT |
		     BM_ELCDIF_CTRL_DATA_FORMAT_24_BIT,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	__raw_writel(BM_ELCDIF_CTRL1_BYTE_PACKING_FORMAT,
		     elcdif_base + HW_ELCDIF_CTRL1_CLR);
	switch (in_pixel_format) {
	case ELCDIF_PIX_FMT_RGB565:
		__raw_writel(BF_ELCDIF_CTRL1_BYTE_PACKING_FORMAT(0xF),
			     elcdif_base + HW_ELCDIF_CTRL1_SET);
		__raw_writel(BF_ELCDIF_CTRL_WORD_LENGTH(0) |
			     BF_ELCDIF_CTRL_INPUT_DATA_SWIZZLE(0),
			     elcdif_base + HW_ELCDIF_CTRL_SET);
		break;
	case ELCDIF_PIX_FMT_RGB24:
		__raw_writel(BF_ELCDIF_CTRL1_BYTE_PACKING_FORMAT(0xF),
			     elcdif_base + HW_ELCDIF_CTRL1_SET);
		__raw_writel(BF_ELCDIF_CTRL_WORD_LENGTH(3) |
			     BF_ELCDIF_CTRL_INPUT_DATA_SWIZZLE(0),
			     elcdif_base + HW_ELCDIF_CTRL_SET);
		break;
	case ELCDIF_PIX_FMT_RGB32:
		__raw_writel(BF_ELCDIF_CTRL1_BYTE_PACKING_FORMAT(0x7),
			     elcdif_base + HW_ELCDIF_CTRL1_SET);
		__raw_writel(BF_ELCDIF_CTRL_WORD_LENGTH(3) |
			     BF_ELCDIF_CTRL_INPUT_DATA_SWIZZLE(0),
			     elcdif_base + HW_ELCDIF_CTRL_SET);
		break;
	default:
		dev_err(g_elcdif_dev, "ELCDIF unsupported input pixel format "
		       "%d\n", in_pixel_format);
		break;
	}

	/* Configure the output pixel format */
	__raw_writel(BM_ELCDIF_CTRL_LCD_DATABUS_WIDTH,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	switch (out_pixel_format) {
	case ELCDIF_PIX_FMT_RGB565:
		__raw_writel(BF_ELCDIF_CTRL_LCD_DATABUS_WIDTH(0),
			     elcdif_base + HW_ELCDIF_CTRL_SET);
		break;
	case ELCDIF_PIX_FMT_RGB666:
		__raw_writel(BF_ELCDIF_CTRL_LCD_DATABUS_WIDTH(2),
			     elcdif_base + HW_ELCDIF_CTRL_SET);
		break;
	case ELCDIF_PIX_FMT_RGB24:
		__raw_writel(BF_ELCDIF_CTRL_LCD_DATABUS_WIDTH(3),
			     elcdif_base + HW_ELCDIF_CTRL_SET);
		break;
	default:
		dev_err(g_elcdif_dev, "ELCDIF unsupported output pixel format "
		       "%d\n", out_pixel_format);
		break;
	}

	val = __raw_readl(elcdif_base + HW_ELCDIF_TRANSFER_COUNT);
	val &= ~(BM_ELCDIF_TRANSFER_COUNT_V_COUNT |
		 BM_ELCDIF_TRANSFER_COUNT_H_COUNT);
	val |= BF_ELCDIF_TRANSFER_COUNT_H_COUNT(h_active) |
	       BF_ELCDIF_TRANSFER_COUNT_V_COUNT(v_active);
	__raw_writel(val, elcdif_base + HW_ELCDIF_TRANSFER_COUNT);

	__raw_writel(BM_ELCDIF_CTRL_VSYNC_MODE,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	__raw_writel(BM_ELCDIF_CTRL_WAIT_FOR_VSYNC_EDGE,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	__raw_writel(BM_ELCDIF_CTRL_DVI_MODE,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	__raw_writel(BM_ELCDIF_CTRL_DOTCLK_MODE,
		     elcdif_base + HW_ELCDIF_CTRL_SET);
	__raw_writel(BM_ELCDIF_CTRL_BYPASS_COUNT,
		     elcdif_base + HW_ELCDIF_CTRL_SET);

	val = __raw_readl(elcdif_base + HW_ELCDIF_VDCTRL0);
	val &= ~(BM_ELCDIF_VDCTRL0_VSYNC_POL |
		 BM_ELCDIF_VDCTRL0_HSYNC_POL |
		 BM_ELCDIF_VDCTRL0_ENABLE_POL |
		 BM_ELCDIF_VDCTRL0_DOTCLK_POL);
	if (sig_cfg.Vsync_pol)
		val |= BM_ELCDIF_VDCTRL0_VSYNC_POL;
	if (sig_cfg.Hsync_pol)
		val |= BM_ELCDIF_VDCTRL0_HSYNC_POL;
	if (sig_cfg.clk_pol)
		val |= BM_ELCDIF_VDCTRL0_DOTCLK_POL;
	if (sig_cfg.enable_pol)
		val |= BM_ELCDIF_VDCTRL0_ENABLE_POL;
	__raw_writel(val, elcdif_base + HW_ELCDIF_VDCTRL0);

	/* vsync is output */
	val = __raw_readl(elcdif_base + HW_ELCDIF_VDCTRL0);
	val &= ~(BM_ELCDIF_VDCTRL0_VSYNC_OEB);
	__raw_writel(val, elcdif_base + HW_ELCDIF_VDCTRL0);

	/*
	 * need enable sig for true RGB i/f.  Or, if not true RGB, leave it
	 * zero.
	 */
	if (enable_present) {
		val = __raw_readl(elcdif_base + HW_ELCDIF_VDCTRL0);
		val |= BM_ELCDIF_VDCTRL0_ENABLE_PRESENT;
		__raw_writel(val, elcdif_base + HW_ELCDIF_VDCTRL0);
	}

	/*
	 * For DOTCLK mode, count VSYNC_PERIOD in terms of complete hz lines
	 */
	val = __raw_readl(elcdif_base + HW_ELCDIF_VDCTRL0);
	val &= ~(BM_ELCDIF_VDCTRL0_VSYNC_PERIOD_UNIT |
		 BM_ELCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT);
	val |= BM_ELCDIF_VDCTRL0_VSYNC_PERIOD_UNIT |
	       BM_ELCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT;
	__raw_writel(val, elcdif_base + HW_ELCDIF_VDCTRL0);

	__raw_writel(BM_ELCDIF_VDCTRL0_VSYNC_PULSE_WIDTH,
		     elcdif_base + HW_ELCDIF_VDCTRL0_CLR);
	__raw_writel(v_pulse_width, elcdif_base + HW_ELCDIF_VDCTRL0_SET);

	__raw_writel(BF_ELCDIF_VDCTRL1_VSYNC_PERIOD(v_period),
		     elcdif_base + HW_ELCDIF_VDCTRL1);

	__raw_writel(BF_ELCDIF_VDCTRL2_HSYNC_PULSE_WIDTH(h_pulse_width) |
		     BF_ELCDIF_VDCTRL2_HSYNC_PERIOD(h_period),
		     elcdif_base + HW_ELCDIF_VDCTRL2);

	val = __raw_readl(elcdif_base + HW_ELCDIF_VDCTRL4);
	val &= ~BM_ELCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT;
	val |= BF_ELCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT(h_active);
	__raw_writel(val, elcdif_base + HW_ELCDIF_VDCTRL4);

	val = __raw_readl(elcdif_base + HW_ELCDIF_VDCTRL3);
	val &= ~(BM_ELCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT |
		 BM_ELCDIF_VDCTRL3_VERTICAL_WAIT_CNT);
	val |= BF_ELCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT(h_wait_cnt) |
	       BF_ELCDIF_VDCTRL3_VERTICAL_WAIT_CNT(v_wait_cnt);
	__raw_writel(val, elcdif_base + HW_ELCDIF_VDCTRL3);

	val = __raw_readl(elcdif_base + HW_ELCDIF_VDCTRL4);
	val |= BM_ELCDIF_VDCTRL4_SYNC_SIGNALS_ON;
	__raw_writel(val, elcdif_base + HW_ELCDIF_VDCTRL4);

	return;
}

static inline void release_dotclk_panel(void)
{
	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	__raw_writel(BM_ELCDIF_CTRL_DOTCLK_MODE,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	__raw_writel(0, elcdif_base + HW_ELCDIF_VDCTRL0);
	__raw_writel(0, elcdif_base + HW_ELCDIF_VDCTRL1);
	__raw_writel(0, elcdif_base + HW_ELCDIF_VDCTRL2);
	__raw_writel(0, elcdif_base + HW_ELCDIF_VDCTRL3);

	return;
}

static inline void setup_dvi_panel(u16 h_active, u16 v_active,
				   u16 h_blanking, u16 v_lines,
				   u16 v1_blank_start, u16 v1_blank_end,
				   u16 v2_blank_start, u16 v2_blank_end,
				   u16 f1_start, u16 f1_end,
				   u16 f2_start, u16 f2_end)
{
	u32 val;

	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	/* 32bit packed format (RGB) */
	__raw_writel(BM_ELCDIF_CTRL1_BYTE_PACKING_FORMAT,
		     elcdif_base + HW_ELCDIF_CTRL1_CLR);
	__raw_writel(BF_ELCDIF_CTRL1_BYTE_PACKING_FORMAT(0x7) |
		     BM_ELCDIF_CTRL1_RECOVER_ON_UNDERFLOW,
		     elcdif_base + HW_ELCDIF_CTRL1_SET);

	val = __raw_readl(elcdif_base + HW_ELCDIF_TRANSFER_COUNT);
	val &= ~(BM_ELCDIF_TRANSFER_COUNT_V_COUNT |
		 BM_ELCDIF_TRANSFER_COUNT_H_COUNT);
	val |= BF_ELCDIF_TRANSFER_COUNT_H_COUNT(h_active) |
	       BF_ELCDIF_TRANSFER_COUNT_V_COUNT(v_active);
	__raw_writel(val, elcdif_base + HW_ELCDIF_TRANSFER_COUNT);

	/* set elcdif to DVI mode */
	__raw_writel(BM_ELCDIF_CTRL_DVI_MODE,
		     elcdif_base + HW_ELCDIF_CTRL_SET);
	__raw_writel(BM_ELCDIF_CTRL_VSYNC_MODE,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	__raw_writel(BM_ELCDIF_CTRL_DOTCLK_MODE,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);

	__raw_writel(BM_ELCDIF_CTRL_BYPASS_COUNT,
		     elcdif_base + HW_ELCDIF_CTRL_SET);
	/* convert input RGB -> YCbCr */
	__raw_writel(BM_ELCDIF_CTRL_RGB_TO_YCBCR422_CSC,
		     elcdif_base + HW_ELCDIF_CTRL_SET);
	/* interlace odd and even fields */
	__raw_writel(BM_ELCDIF_CTRL1_INTERLACE_FIELDS,
		     elcdif_base + HW_ELCDIF_CTRL1_SET);

	__raw_writel(BM_ELCDIF_CTRL_WORD_LENGTH |
		     BM_ELCDIF_CTRL_INPUT_DATA_SWIZZLE |
		     BM_ELCDIF_CTRL_LCD_DATABUS_WIDTH,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	__raw_writel(BF_ELCDIF_CTRL_WORD_LENGTH(3) |	/* 24 bit */
		      BM_ELCDIF_CTRL_DATA_SELECT |	/* data mode */
		      BF_ELCDIF_CTRL_INPUT_DATA_SWIZZLE(0) |	/* no swap */
		      BF_ELCDIF_CTRL_LCD_DATABUS_WIDTH(1),	/* 8 bit */
		      elcdif_base + HW_ELCDIF_CTRL_SET);

	/* ELCDIF_DVI */
	/* set frame size */
	val = __raw_readl(elcdif_base + HW_ELCDIF_DVICTRL0);
	__raw_writel(val, elcdif_base + HW_ELCDIF_DVICTRL0);

	/* set start/end of field-1 and start of field-2 */
	val = __raw_readl(elcdif_base + HW_ELCDIF_DVICTRL1);
	val &= ~(BM_ELCDIF_DVICTRL1_F1_START_LINE |
		 BM_ELCDIF_DVICTRL1_F1_END_LINE |
		 BM_ELCDIF_DVICTRL1_F2_START_LINE);
	val |= BF_ELCDIF_DVICTRL1_F1_START_LINE(f1_start) |
	       BF_ELCDIF_DVICTRL1_F1_END_LINE(f1_end) |
	       BF_ELCDIF_DVICTRL1_F2_START_LINE(f2_start);
	__raw_writel(val, elcdif_base + HW_ELCDIF_DVICTRL1);

	/* set first vertical blanking interval and end of filed-2 */
	val = __raw_readl(elcdif_base + HW_ELCDIF_DVICTRL2);
	val &= ~(BM_ELCDIF_DVICTRL2_F2_END_LINE |
		 BM_ELCDIF_DVICTRL2_V1_BLANK_START_LINE |
		 BM_ELCDIF_DVICTRL2_V1_BLANK_END_LINE);
	val |= BF_ELCDIF_DVICTRL2_F2_END_LINE(f2_end) |
	       BF_ELCDIF_DVICTRL2_V1_BLANK_START_LINE(v1_blank_start) |
	       BF_ELCDIF_DVICTRL2_V1_BLANK_END_LINE(v1_blank_end);
	__raw_writel(val, elcdif_base + HW_ELCDIF_DVICTRL2);

	/* set second vertical blanking interval */
	val = __raw_readl(elcdif_base + HW_ELCDIF_DVICTRL3);
	val &= ~(BM_ELCDIF_DVICTRL3_V2_BLANK_START_LINE |
		      BM_ELCDIF_DVICTRL3_V2_BLANK_END_LINE);
	val |= BF_ELCDIF_DVICTRL3_V2_BLANK_START_LINE(v2_blank_start) |
	       BF_ELCDIF_DVICTRL3_V2_BLANK_END_LINE(v2_blank_end);
	__raw_writel(val, elcdif_base + HW_ELCDIF_DVICTRL3);

	/* fill the rest area black color if the input frame
	 * is not 720 pixels/line
	 */
	if (h_active != 720) {
		/* the input frame can't be less then (720-256) pixels/line */
		if (720 - h_active > 0xff)
			h_active = 720 - 0xff;

		val = __raw_readl(elcdif_base + HW_ELCDIF_DVICTRL4);
		val &= ~(BM_ELCDIF_DVICTRL4_H_FILL_CNT |
			      BM_ELCDIF_DVICTRL4_Y_FILL_VALUE |
			      BM_ELCDIF_DVICTRL4_CB_FILL_VALUE |
			      BM_ELCDIF_DVICTRL4_CR_FILL_VALUE);
		val |= BF_ELCDIF_DVICTRL4_H_FILL_CNT(720 - h_active) |
		       BF_ELCDIF_DVICTRL4_Y_FILL_VALUE(16) |
		       BF_ELCDIF_DVICTRL4_CB_FILL_VALUE(128) |
		       BF_ELCDIF_DVICTRL4_CR_FILL_VALUE(128);
		__raw_writel(val, elcdif_base + HW_ELCDIF_DVICTRL4);
	}

	/* Color Space Conversion RGB->YCbCr */
	val = __raw_readl(elcdif_base + HW_ELCDIF_CSC_COEFF0);
	val &= ~(BM_ELCDIF_CSC_COEFF0_C0 |
		      BM_ELCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER);
	val |= BF_ELCDIF_CSC_COEFF0_C0(0x41) |
	       BF_ELCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER(3);
	__raw_writel(val, elcdif_base + HW_ELCDIF_CSC_COEFF0);

	val = __raw_readl(elcdif_base + HW_ELCDIF_CSC_COEFF1);
	val &= ~(BM_ELCDIF_CSC_COEFF1_C1 | BM_ELCDIF_CSC_COEFF1_C2);
	val |= BF_ELCDIF_CSC_COEFF1_C1(0x81) |
	       BF_ELCDIF_CSC_COEFF1_C2(0x19);
	__raw_writel(val, elcdif_base + HW_ELCDIF_CSC_COEFF1);

	val = __raw_readl(elcdif_base + HW_ELCDIF_CSC_COEFF2);
	val &= ~(BM_ELCDIF_CSC_COEFF2_C3 | BM_ELCDIF_CSC_COEFF2_C4);
	val |= BF_ELCDIF_CSC_COEFF2_C3(0x3DB) |
	       BF_ELCDIF_CSC_COEFF2_C4(0x3B6);
	__raw_writel(val, elcdif_base + HW_ELCDIF_CSC_COEFF2);

	val = __raw_readl(elcdif_base + HW_ELCDIF_CSC_COEFF3);
	val &= ~(BM_ELCDIF_CSC_COEFF3_C5 | BM_ELCDIF_CSC_COEFF3_C6);
	val |= BF_ELCDIF_CSC_COEFF3_C5(0x70) |
	       BF_ELCDIF_CSC_COEFF3_C6(0x70);
	__raw_writel(val, elcdif_base + HW_ELCDIF_CSC_COEFF3);

	val = __raw_readl(elcdif_base + HW_ELCDIF_CSC_COEFF4);
	val &= ~(BM_ELCDIF_CSC_COEFF4_C7 | BM_ELCDIF_CSC_COEFF4_C8);
	val |= BF_ELCDIF_CSC_COEFF4_C7(0x3A2) |
	       BF_ELCDIF_CSC_COEFF4_C8(0x3EE);
	__raw_writel(val, elcdif_base + HW_ELCDIF_CSC_COEFF4);

	val = __raw_readl(elcdif_base + HW_ELCDIF_CSC_OFFSET);
	val &= ~(BM_ELCDIF_CSC_OFFSET_CBCR_OFFSET |
		 BM_ELCDIF_CSC_OFFSET_Y_OFFSET);
	val |= BF_ELCDIF_CSC_OFFSET_CBCR_OFFSET(0x80) |
	       BF_ELCDIF_CSC_OFFSET_Y_OFFSET(0x10);
	__raw_writel(val, elcdif_base + HW_ELCDIF_CSC_OFFSET);

	val = __raw_readl(elcdif_base + HW_ELCDIF_CSC_LIMIT);
	val &= ~(BM_ELCDIF_CSC_LIMIT_CBCR_MIN |
		 BM_ELCDIF_CSC_LIMIT_CBCR_MAX |
		 BM_ELCDIF_CSC_LIMIT_Y_MIN |
		 BM_ELCDIF_CSC_LIMIT_Y_MAX);
	val |= BF_ELCDIF_CSC_LIMIT_CBCR_MIN(16) |
	       BF_ELCDIF_CSC_LIMIT_CBCR_MAX(240) |
	       BF_ELCDIF_CSC_LIMIT_Y_MIN(16) |
	       BF_ELCDIF_CSC_LIMIT_Y_MAX(235);
	__raw_writel(val, elcdif_base + HW_ELCDIF_CSC_LIMIT);

	return;
}

static inline void release_dvi_panel(void)
{
	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	__raw_writel(BM_ELCDIF_CTRL_DVI_MODE,
			elcdif_base + HW_ELCDIF_CTRL_CLR);
	return;
}

static inline void mxc_init_elcdif(void)
{
	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	__raw_writel(BM_ELCDIF_CTRL_CLKGATE,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	/* Reset controller */
	__raw_writel(BM_ELCDIF_CTRL_SFTRST,
		     elcdif_base + HW_ELCDIF_CTRL_SET);
	udelay(10);

	/* Take controller out of reset */
	__raw_writel(BM_ELCDIF_CTRL_SFTRST | BM_ELCDIF_CTRL_CLKGATE,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);

	/* Setup the bus protocol */
	__raw_writel(BM_ELCDIF_CTRL1_MODE86,
		     elcdif_base + HW_ELCDIF_CTRL1_CLR);
	__raw_writel(BM_ELCDIF_CTRL1_BUSY_ENABLE,
		     elcdif_base + HW_ELCDIF_CTRL1_CLR);

	/* Take display out of reset */
	__raw_writel(BM_ELCDIF_CTRL1_RESET,
		     elcdif_base + HW_ELCDIF_CTRL1_SET);

	/* VSYNC is an input by default */
	__raw_writel(BM_ELCDIF_VDCTRL0_VSYNC_OEB,
		     elcdif_base + HW_ELCDIF_VDCTRL0_SET);

	/* Reset display */
	__raw_writel(BM_ELCDIF_CTRL1_RESET,
		     elcdif_base + HW_ELCDIF_CTRL1_CLR);
	udelay(10);
	__raw_writel(BM_ELCDIF_CTRL1_RESET,
		     elcdif_base + HW_ELCDIF_CTRL1_SET);
	udelay(10);

	return;
}

void mxcfb_elcdif_register_mode(const struct fb_videomode *modedb,
	int num_modes, int dev_mode)
{
	struct fb_videomode *mode;

	mode = kzalloc(num_modes * sizeof(struct fb_videomode), GFP_KERNEL);

	if (!mode) {
		dev_err(g_elcdif_dev, "%s Failed to allocate mode data\n", __func__);
		return;
	}

	if (mxc_disp_mode.num_modes)
		memcpy(mode, mxc_disp_mode.mode,
			mxc_disp_mode.num_modes * sizeof(struct fb_videomode));
	if (modedb)
		memcpy(mode + mxc_disp_mode.num_modes, modedb,
			num_modes * sizeof(struct fb_videomode));

	if (mxc_disp_mode.num_modes)
		kfree(mxc_disp_mode.mode);

	mxc_disp_mode.mode = mode;
	mxc_disp_mode.num_modes += num_modes;
	mxc_disp_mode.dev_mode = dev_mode;

	return;
}
EXPORT_SYMBOL(mxcfb_elcdif_register_mode);

int mxc_elcdif_frame_addr_setup(dma_addr_t phys)
{
	int ret = 0;

	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	__raw_writel(BM_ELCDIF_CTRL_ELCDIF_MASTER,
		     elcdif_base + HW_ELCDIF_CTRL_SET);

	__raw_writel(phys, elcdif_base + HW_ELCDIF_CUR_BUF);
	__raw_writel(phys, elcdif_base + HW_ELCDIF_NEXT_BUF);
	return ret;
}

static inline void mxc_elcdif_dma_release(void)
{
	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	__raw_writel(BM_ELCDIF_CTRL_ELCDIF_MASTER,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	return;
}

static inline void mxc_elcdif_run(struct mxc_elcdif_fb_data *data)
{
	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	__raw_writel(BM_ELCDIF_CTRL_ELCDIF_MASTER,
		     elcdif_base + HW_ELCDIF_CTRL_SET);
	__raw_writel(BM_ELCDIF_CTRL_RUN,
		     elcdif_base + HW_ELCDIF_CTRL_SET);

	data->running = true;

	return;
}

static inline void mxc_elcdif_stop(void)
{
	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	__raw_writel(BM_ELCDIF_CTRL_RUN,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	__raw_writel(BM_ELCDIF_CTRL_ELCDIF_MASTER,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);
	msleep(1);
	__raw_writel(BM_ELCDIF_CTRL_CLKGATE, elcdif_base + HW_ELCDIF_CTRL_SET);
	return;
}

static int mxc_elcdif_blank_panel(int blank)
{
	int ret = 0, count;

	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	switch (blank) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		__raw_writel(BM_ELCDIF_CTRL_BYPASS_COUNT,
				elcdif_base + HW_ELCDIF_CTRL_CLR);
		for (count = 10000; count; count--) {
			if (__raw_readl(elcdif_base + HW_ELCDIF_STAT) &
			    BM_ELCDIF_STAT_TXFIFO_EMPTY)
				break;
			msleep(1);
		}
		break;

	case FB_BLANK_UNBLANK:
		__raw_writel(BM_ELCDIF_CTRL_BYPASS_COUNT,
			      elcdif_base + HW_ELCDIF_CTRL_SET);
		break;

	default:
		dev_err(g_elcdif_dev, "unknown blank parameter\n");
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int mxc_elcdif_init_panel(void)
{
	if (!g_elcdif_axi_clk_enable) {
		clk_enable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = true;
	}

	/*
	 * Make sure we do a high-to-low transition to reset the panel.
	 * First make it low for 100 msec, hi for 10 msec, low for 10 msec,
	 * then hi.
	 */
	__raw_writel(BM_ELCDIF_CTRL1_RESET,
		     elcdif_base + HW_ELCDIF_CTRL1_CLR);	/* low */
	msleep(100);
	__raw_writel(BM_ELCDIF_CTRL1_RESET,
		     elcdif_base + HW_ELCDIF_CTRL1_SET);	/* high */
	msleep(10);
	__raw_writel(BM_ELCDIF_CTRL1_RESET,
		     elcdif_base + HW_ELCDIF_CTRL1_CLR);	/* low */

	/* For the Samsung, Reset must be held low at least 30 uSec
	 * Therefore, we'll hold it low for about 10 mSec just to be sure.
	 * Then we'll wait 1 mSec afterwards.
	 */
	msleep(10);
	__raw_writel(BM_ELCDIF_CTRL1_RESET,
		     elcdif_base + HW_ELCDIF_CTRL1_SET);	/* high */
	msleep(1);

	return 0;
}

static uint32_t bpp_to_pixfmt(struct fb_info *fbi)
{
	uint32_t pixfmt = 0;

	if (fbi->var.nonstd)
		return fbi->var.nonstd;

	switch (fbi->var.bits_per_pixel) {
	case 32:
		pixfmt = ELCDIF_PIX_FMT_RGB32;
		break;
	case 24:
		pixfmt = ELCDIF_PIX_FMT_RGB24;
		break;
	case 18:
		pixfmt = ELCDIF_PIX_FMT_RGB666;
		break;
	case 16:
		pixfmt = ELCDIF_PIX_FMT_RGB565;
		break;
	case 8:
		pixfmt = ELCDIF_PIX_FMT_RGB332;
		break;
	}
	return pixfmt;
}

static int mxc_elcdif_fb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ypanstep = 1;

	return 0;
}

static irqreturn_t lcd_irq_handler(int irq, void *dev_id)
{
	struct mxc_elcdif_fb_data *data = dev_id;
	u32 status_lcd = __raw_readl(elcdif_base + HW_ELCDIF_CTRL1);
	dev_dbg(g_elcdif_dev, "%s: irq %d\n", __func__, irq);

	if ((status_lcd & BM_ELCDIF_CTRL1_VSYNC_EDGE_IRQ) &&
		data->wait4vsync) {
		dev_dbg(g_elcdif_dev, "%s: VSYNC irq\n", __func__);
		__raw_writel(BM_ELCDIF_CTRL1_VSYNC_EDGE_IRQ_EN,
			     elcdif_base + HW_ELCDIF_CTRL1_CLR);
		data->wait4vsync = 0;
		data->vsync_nf_timestamp = ktime_get();
		complete(&data->vsync_complete);
	}
	if ((status_lcd & BM_ELCDIF_CTRL1_CUR_FRAME_DONE_IRQ) &&
		data->wait4framedone) {
		dev_dbg(g_elcdif_dev, "%s: frame done irq\n", __func__);
		__raw_writel(BM_ELCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN,
			     elcdif_base + HW_ELCDIF_CTRL1_CLR);
		if (data->panning) {
			up(&data->flip_sem);
			data->panning = 0;
		}
		data->wait4framedone = 0;
		complete(&data->frame_done_complete);
	}
	if (status_lcd & BM_ELCDIF_CTRL1_UNDERFLOW_IRQ) {
		dev_dbg(g_elcdif_dev, "%s: underflow irq\n", __func__);
		__raw_writel(BM_ELCDIF_CTRL1_UNDERFLOW_IRQ,
			     elcdif_base + HW_ELCDIF_CTRL1_CLR);
	}
	if (status_lcd & BM_ELCDIF_CTRL1_OVERFLOW_IRQ) {
		dev_dbg(g_elcdif_dev, "%s: overflow irq\n", __func__);
		__raw_writel(BM_ELCDIF_CTRL1_OVERFLOW_IRQ,
			     elcdif_base + HW_ELCDIF_CTRL1_CLR);
	}
	return IRQ_HANDLED;
}

static inline u_int _chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int mxc_elcdif_fb_setcolreg(u_int regno, u_int red, u_int green,
				 u_int blue, u_int transp,
				 struct fb_info *fbi)
{
	unsigned int val;
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (fbi->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (fbi->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = fbi->pseudo_palette;

			val = _chan_to_field(red, &fbi->var.red);
			val |= _chan_to_field(green, &fbi->var.green);
			val |= _chan_to_field(blue, &fbi->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}
	return ret;
}

/**
   This function compare the fb parameter see whether it was different
   parameter for hardware, if it was different parameter, the hardware
   will reinitialize. All will compared except x/y offset.
 */
static bool mxc_elcdif_fb_par_equal(struct fb_info *fbi, struct mxc_elcdif_fb_data *data)
{
	/* Here we set the xoffset, yoffset to zero, and compare two
	 * var see have different or not. */
	struct fb_var_screeninfo oldvar = data->var;
	struct fb_var_screeninfo newvar = fbi->var;

	if ((fbi->var.activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW &&
	    fbi->var.activate & FB_ACTIVATE_FORCE)
		return false;

	oldvar.xoffset = newvar.xoffset = 0;
	oldvar.yoffset = newvar.yoffset = 0;

	return memcmp(&oldvar, &newvar, sizeof(struct fb_var_screeninfo)) == 0;
}

/*
 * This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the
 * change in par. For this driver it doesn't do much.
 *
 */
static int mxc_elcdif_fb_set_par(struct fb_info *fbi)
{
	struct mxc_elcdif_fb_data *data = (struct mxc_elcdif_fb_data *)fbi->par;
	struct elcdif_signal_cfg sig_cfg;
	int mem_len;

	dev_dbg(fbi->device, "Reconfiguring framebuffer\n");

	/* If parameter no change, don't reconfigure. */
	if (mxc_elcdif_fb_par_equal(fbi, data) && (data->running == true))
		return 0;

	sema_init(&data->flip_sem, 1);

	/* release prev panel */
	if (!g_elcdif_pix_clk_enable) {
		clk_enable(g_elcdif_pix_clk);
		g_elcdif_pix_clk_enable = true;
	}
	mxc_elcdif_blank_panel(FB_BLANK_POWERDOWN);
	mxc_elcdif_stop();
	release_dotclk_panel();
	mxc_elcdif_dma_release();
	mxc_elcdif_fb_set_fix(fbi);
	if (g_elcdif_pix_clk_enable) {
		clk_disable(g_elcdif_pix_clk);
		g_elcdif_pix_clk_enable = false;
	}

	mem_len = fbi->var.yres_virtual * fbi->fix.line_length;
	if (!fbi->fix.smem_start || (mem_len > fbi->fix.smem_len)) {
		if (fbi->fix.smem_start)
			mxc_elcdif_fb_unmap_video_memory(fbi);

		if (mxc_elcdif_fb_map_video_memory(fbi) < 0)
			return -ENOMEM;
	}

	if (data->next_blank != FB_BLANK_UNBLANK)
		return 0;

	/* init next panel */
	if (!g_elcdif_pix_clk_enable) {
		clk_enable(g_elcdif_pix_clk);
		g_elcdif_pix_clk_enable = true;
	}
	mxc_init_elcdif();
	mxc_elcdif_init_panel();

	dev_dbg(fbi->device, "pixclock = %u Hz\n",
		(u32) (PICOS2KHZ(fbi->var.pixclock) * 1000UL));

	memset(&sig_cfg, 0, sizeof(sig_cfg));
	if (fbi->var.sync & FB_SYNC_HOR_HIGH_ACT)
		sig_cfg.Hsync_pol = true;
	if (fbi->var.sync & FB_SYNC_VERT_HIGH_ACT)
		sig_cfg.Vsync_pol = true;
	if (fbi->var.sync & FB_SYNC_CLK_LAT_FALL)
		sig_cfg.clk_pol = true;
	if (!(fbi->var.sync & FB_SYNC_OE_LOW_ACT))
		sig_cfg.enable_pol = true;

	setup_dotclk_panel((PICOS2KHZ(fbi->var.pixclock)) * 1000UL,
			   fbi->var.vsync_len,
			   fbi->var.upper_margin + fbi->var.yres +
			   fbi->var.lower_margin + fbi->var.vsync_len,
			   fbi->var.upper_margin + fbi->var.vsync_len,
			   fbi->var.yres,
			   fbi->var.hsync_len,
			   fbi->var.left_margin + fbi->var.xres +
			   fbi->var.right_margin + fbi->var.hsync_len,
			   fbi->var.left_margin + fbi->var.hsync_len,
			   fbi->var.xres,
			   bpp_to_pixfmt(fbi),
			   data->output_pix_fmt,
			   sig_cfg,
			   1);
	mxc_elcdif_frame_addr_setup(fbi->fix.smem_start);
	mxc_elcdif_run(data);
	mxc_elcdif_blank_panel(FB_BLANK_UNBLANK);

	fbi->mode = (struct fb_videomode *)fb_match_mode(&fbi->var,
							 &fbi->modelist);
	/* Clear activate as not Reconfiguring framebuffer again */
	if ((fbi->var.activate & FB_ACTIVATE_FORCE) &&
		(fbi->var.activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW)
		fbi->var.activate = FB_ACTIVATE_NOW;

	data->var = fbi->var;

	return 0;
}

static int mxc_elcdif_fb_check_var(struct fb_var_screeninfo *var,
				 struct fb_info *info)
{
	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
	    (var->bits_per_pixel != 16) && (var->bits_per_pixel != 8))
		var->bits_per_pixel = default_bpp;

	switch (var->bits_per_pixel) {
	case 8:
		var->red.length = 3;
		var->red.offset = 5;
		var->red.msb_right = 0;

		var->green.length = 3;
		var->green.offset = 2;
		var->green.msb_right = 0;

		var->blue.length = 2;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 16:
		var->red.length = 5;
		var->red.offset = 11;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;
		break;
	}

	var->height = -1;
	var->width = -1;
	var->grayscale = 0;

	return 0;
}

static int mxc_elcdif_fb_wait_for_vsync(struct fb_info *info)
{
	struct mxc_elcdif_fb_data *data =
				(struct mxc_elcdif_fb_data *)info->par;
	int ret = 0;

	if (data->cur_blank != FB_BLANK_UNBLANK) {
		dev_err(info->device, "can't wait for VSYNC when fb "
			"is blank\n");
		return -EINVAL;
	}

	init_completion(&data->vsync_complete);

	__raw_writel(BM_ELCDIF_CTRL1_VSYNC_EDGE_IRQ,
		elcdif_base + HW_ELCDIF_CTRL1_CLR);
	data->wait4vsync = 1;
	__raw_writel(BM_ELCDIF_CTRL1_VSYNC_EDGE_IRQ_EN,
		elcdif_base + HW_ELCDIF_CTRL1_SET);
	ret = wait_for_completion_interruptible_timeout(
				&data->vsync_complete, 1 * HZ);
	if (ret == 0) {
		dev_err(info->device,
			"MXC ELCDIF wait for vsync timeout\n");
		data->wait4vsync = 0;
		ret = -ETIME;
	} else if (ret > 0) {
		ret = 0;
	}
	return ret;
}

static int mxc_elcdif_fb_wait_for_frame_done(struct fb_info *info)
{
	struct mxc_elcdif_fb_data *data =
				(struct mxc_elcdif_fb_data *)info->par;
	int ret = 0;

	if (data->cur_blank != FB_BLANK_UNBLANK) {
		dev_err(info->device, "can't wait for frame done when fb "
			"is blank\n");
		return -EINVAL;
	}

	init_completion(&data->frame_done_complete);

	__raw_writel(BM_ELCDIF_CTRL1_CUR_FRAME_DONE_IRQ,
		elcdif_base + HW_ELCDIF_CTRL1_CLR);
	data->wait4framedone = 1;
	__raw_writel(BM_ELCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN,
		elcdif_base + HW_ELCDIF_CTRL1_SET);
	ret = wait_for_completion_interruptible_timeout(
				&data->frame_done_complete, 1 * HZ);
	if (ret == 0) {
		dev_err(info->device,
			"MXC ELCDIF wait for frame done timeout\n");
		data->wait4framedone = 0;
		ret = -ETIME;
	} else if (ret > 0) {
		ret = 0;
	}
	return ret;
}

static int mxc_elcdif_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	int ret = -EINVAL;

	switch (cmd) {
	case MXCFB_WAIT_FOR_VSYNC:
		{
			struct mxc_elcdif_fb_data *data =
				(struct mxc_elcdif_fb_data *)info->par;
			long long timestamp;
			ret = mxc_elcdif_fb_wait_for_vsync(info);
			timestamp = ktime_to_ns(data->vsync_nf_timestamp);
			if ((ret == 0) && copy_to_user((void *)arg,
					&timestamp, sizeof(timestamp))) {
				ret = -EFAULT;
				break;
			}
		}

		break;
	case MXCFB_GET_FB_BLANK:
		{
			struct mxc_elcdif_fb_data *data =
				(struct mxc_elcdif_fb_data *)info->par;

			if (put_user(data->cur_blank, (__u32 __user *)arg))
				return -EFAULT;
			break;
		}
	default:
		break;
	}
	return ret;
}

static int mxc_elcdif_fb_blank(int blank, struct fb_info *info)
{
	struct mxc_elcdif_fb_data *data =
				(struct mxc_elcdif_fb_data *)info->par;
	int ret = 0;

	if (data->cur_blank == blank)
		return ret;

	data->next_blank = blank;

	if (!g_elcdif_pix_clk_enable) {
		clk_enable(g_elcdif_pix_clk);
		g_elcdif_pix_clk_enable = true;
	}
	ret = mxc_elcdif_blank_panel(blank);
	if (ret == 0)
		data->cur_blank = blank;
	else
		return ret;

	if (blank == FB_BLANK_UNBLANK) {
		info->var.activate = (info->var.activate & ~FB_ACTIVATE_MASK) |
				FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;
		ret = mxc_elcdif_fb_set_par(info);
		if (ret)
			return ret;
	}

	if (data->cur_blank != FB_BLANK_UNBLANK) {
		if (g_elcdif_axi_clk_enable) {
			clk_disable(g_elcdif_axi_clk);
			g_elcdif_axi_clk_enable = false;
		}
		if (g_elcdif_pix_clk_enable) {
			clk_disable(g_elcdif_pix_clk);
			g_elcdif_pix_clk_enable = false;
		}
	} else {
		if (!g_elcdif_axi_clk_enable) {
			clk_enable(g_elcdif_axi_clk);
			g_elcdif_axi_clk_enable = true;
		}
		if (!g_elcdif_pix_clk_enable) {
			clk_enable(g_elcdif_pix_clk);
			g_elcdif_pix_clk_enable = true;
		}
	}

	return ret;
}

static int mxc_elcdif_fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct mxc_elcdif_fb_data *data =
				(struct mxc_elcdif_fb_data *)info->par;
	unsigned long base = 0;

	if (data->cur_blank != FB_BLANK_UNBLANK) {
		dev_err(info->device, "can't do pan display when fb "
			"is blank\n");
		return -EINVAL;
	}

	if (var->xoffset > 0) {
		dev_dbg(info->device, "x panning not supported\n");
		return -EINVAL;
	}

	if ((var->yoffset + var->yres > var->yres_virtual)) {
		dev_err(info->device, "y panning exceeds\n");
		return -EINVAL;
	}

	/* update framebuffer visual */
	base = (var->yoffset * var->xres_virtual + var->xoffset);
	base = (var->bits_per_pixel) * base / 8;
	base += info->fix.smem_start;

	if (down_timeout(&data->flip_sem, HZ / 2)) {
		dev_err(info->device, "timeout when waiting for flip irq\n");
		return -ETIMEDOUT;
	}

	__raw_writel(base, elcdif_base + HW_ELCDIF_NEXT_BUF);

	data->panning = 1;
	return mxc_elcdif_fb_wait_for_frame_done(info);
}

static struct fb_ops mxc_elcdif_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mxc_elcdif_fb_check_var,
	.fb_set_par = mxc_elcdif_fb_set_par,
	.fb_setcolreg = mxc_elcdif_fb_setcolreg,
	.fb_ioctl = mxc_elcdif_fb_ioctl,
	.fb_blank = mxc_elcdif_fb_blank,
	.fb_pan_display = mxc_elcdif_fb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

/*!
 * Allocates the DRAM memory for the frame buffer.      This buffer is remapped
 * into a non-cached, non-buffered, memory region to allow palette and pixel
 * writes to occur without flushing the cache.  Once this area is remapped,
 * all virtual memory access to the video memory should occur at the new region.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxc_elcdif_fb_map_video_memory(struct fb_info *fbi)
{
	if (fbi->fix.smem_len < fbi->var.yres_virtual * fbi->fix.line_length)
		fbi->fix.smem_len = fbi->var.yres_virtual *
				    fbi->fix.line_length;

	fbi->screen_base = dma_alloc_writecombine(fbi->device,
				fbi->fix.smem_len,
				(dma_addr_t *)&fbi->fix.smem_start,
				GFP_DMA);
	if (fbi->screen_base == 0) {
		dev_err(fbi->device, "Unable to allocate framebuffer memory\n");
		fbi->fix.smem_len = 0;
		fbi->fix.smem_start = 0;
		return -EBUSY;
	}

	dev_dbg(fbi->device, "allocated fb @ paddr=0x%08X, size=%d.\n",
		(uint32_t) fbi->fix.smem_start, fbi->fix.smem_len);

	fbi->screen_size = fbi->fix.smem_len;

	/* Clear the screen */
	memset((char *)fbi->screen_base, 0, fbi->fix.smem_len);

	return 0;
}

/*!
 * De-allocates the DRAM memory for the frame buffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxc_elcdif_fb_unmap_video_memory(struct fb_info *fbi)
{
	dma_free_writecombine(fbi->device, fbi->fix.smem_len,
			      fbi->screen_base, fbi->fix.smem_start);
	fbi->screen_base = 0;
	fbi->fix.smem_start = 0;
	fbi->fix.smem_len = 0;
	return 0;
}

static int mxc_elcdif_fb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mxc_elcdif_fb_data *data;
	struct resource *res;
	struct fb_info *fbi;
	struct mxc_fb_platform_data *pdata = pdev->dev.platform_data;
	const struct fb_videomode *mode;
	struct fb_videomode m;
	int num;

	fbi = framebuffer_alloc(sizeof(struct mxc_elcdif_fb_data), &pdev->dev);
	if (fbi == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	data = (struct mxc_elcdif_fb_data *)fbi->par;
	data->cur_blank = data->next_blank = FB_BLANK_UNBLANK;

	fbi->var.activate = FB_ACTIVATE_NOW;
	fbi->fbops = &mxc_elcdif_fb_ops;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->pseudo_palette = data->pseudo_palette;

	ret = fb_alloc_cmap(&fbi->cmap, 16, 0);
	if (ret)
		goto out;

	g_elcdif_dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot get IRQ resource\n");
		ret = -ENODEV;
		goto err0;
	}
	data->dma_irq = res->start;

	ret = request_irq(data->dma_irq, lcd_irq_handler, 0,
			  "mxc_elcdif_fb", data);
	if (ret) {
		dev_err(&pdev->dev, "request_irq (%d) failed with error %d\n",
				data->dma_irq, ret);
		goto err0;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENODEV;
		goto err1;
	}
	elcdif_base = ioremap(res->start, SZ_4K);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		fbi->fix.smem_len = res->end - res->start + 1;
		fbi->fix.smem_start = res->start;
		fbi->screen_base = ioremap(fbi->fix.smem_start,
					   fbi->fix.smem_len);
	}

	strcpy(fbi->fix.id, "mxc_elcdif_fb");

	fbi->var.xres = 800;
	fbi->var.yres = 480;

	if (pdata && !data->output_pix_fmt)
		data->output_pix_fmt = pdata->interface_pix_fmt;

	INIT_LIST_HEAD(&fbi->modelist);

	if (pdata && pdata->mode && pdata->num_modes)
		fb_videomode_to_modelist(pdata->mode, pdata->num_modes,
				&fbi->modelist);

	if (mxc_disp_mode.num_modes) {
		int i;
		mode = mxc_disp_mode.mode;
		num = mxc_disp_mode.num_modes;

		for (i = 0; i < num; i++) {
			/*
			 * FIXME now we do not support interlaced
			 * mode for ddc mode
			 */
			if ((mxc_disp_mode.dev_mode
				& MXC_DISP_DDC_DEV) &&
				(mode[i].vmode & FB_VMODE_INTERLACED))
				continue;
			else {
				dev_dbg(&pdev->dev, "Added mode %d:", i);
				dev_dbg(&pdev->dev,
					"xres = %d, yres = %d, freq = %d, vmode = %d, flag = %d\n",
					mode[i].xres, mode[i].yres,	mode[i].refresh, mode[i].vmode,
					mode[i].flag);
				fb_add_videomode(&mode[i], &fbi->modelist);
			}
		}
	}

	if (!fb_mode && pdata && pdata->mode_str)
		fb_mode = pdata->mode_str;

	if (fb_mode) {
		dev_dbg(&pdev->dev, "default video mode %s\n", fb_mode);
		ret = fb_find_mode(&fbi->var, fbi, fb_mode, NULL, 0, NULL,
				   default_bpp);
		if ((ret == 1) || (ret == 2)) {
			fb_var_to_videomode(&m, &fbi->var);
			dump_fb_videomode(&m);
			mode = fb_find_nearest_mode(&m,
				&fbi->modelist);
			fb_videomode_to_var(&fbi->var, mode);
		} else if (pdata && pdata->mode && pdata->num_modes) {
			ret = fb_find_mode(&fbi->var, fbi, fb_mode, pdata->mode,
					pdata->num_modes, NULL, default_bpp);
			if (!ret) {
				dev_err(fbi->device,
					"No valid video mode found");
				goto err2;
			}
		} else {
			dev_err(fbi->device,
				"No valid video mode found");
			goto err2;
		}
	}

	mxc_elcdif_fb_check_var(&fbi->var, fbi);

	fbi->var.xres_virtual = fbi->var.xres;
	fbi->var.yres_virtual = fbi->var.yres * 3;

	mxc_elcdif_fb_set_fix(fbi);

	if (!res || !res->end)
		if (mxc_elcdif_fb_map_video_memory(fbi) < 0) {
			ret = -ENOMEM;
			goto err2;
		}

	g_elcdif_axi_clk = clk_get(g_elcdif_dev, "elcdif_axi");
	if (g_elcdif_axi_clk == NULL) {
		dev_err(&pdev->dev, "can't get ELCDIF axi clk\n");
		ret = -ENODEV;
		goto err3;
	}
	g_elcdif_pix_clk = clk_get(g_elcdif_dev, "elcdif_pix");
	if (g_elcdif_pix_clk == NULL) {
		dev_err(&pdev->dev, "can't get ELCDIF pix clk\n");
		ret = -ENODEV;
		goto err3;
	}
	/*
	 * Set an appropriate pixel clk rate first, so that we can
	 * access ELCDIF registers.
	 */
	clk_set_rate(g_elcdif_pix_clk, 25000000);

	fbi->var.activate |= FB_ACTIVATE_FORCE;
	console_lock();
	fbi->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_set_var(fbi, &fbi->var);
	fbi->flags &= ~FBINFO_MISC_USEREVENT;
	console_unlock();

	if (data->cur_blank == FB_BLANK_UNBLANK) {
		console_lock();
		fb_blank(fbi, FB_BLANK_UNBLANK);
		console_unlock();
	}

	ret = register_framebuffer(fbi);
	if (ret)
		goto err3;

	platform_set_drvdata(pdev, fbi);

	return 0;
err3:
	mxc_elcdif_fb_unmap_video_memory(fbi);
err2:
	iounmap(elcdif_base);
err1:
	free_irq(data->dma_irq, data);
err0:
	fb_dealloc_cmap(&fbi->cmap);
	framebuffer_release(fbi);
out:
	return ret;
}

static int mxc_elcdif_fb_remove(struct platform_device *pdev)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mxc_elcdif_fb_data *data = (struct mxc_elcdif_fb_data *)fbi->par;

	mxc_elcdif_fb_blank(FB_BLANK_POWERDOWN, fbi);
	mxc_elcdif_stop();
	release_dotclk_panel();
	mxc_elcdif_dma_release();

	if (g_elcdif_axi_clk_enable) {
		clk_disable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = false;
	}
	if (g_elcdif_pix_clk_enable) {
		clk_disable(g_elcdif_pix_clk);
		g_elcdif_pix_clk_enable = false;
	}
	clk_put(g_elcdif_axi_clk);
	clk_put(g_elcdif_pix_clk);

	free_irq(data->dma_irq, data);
	mxc_elcdif_fb_unmap_video_memory(fbi);

	if (&fbi->cmap)
		fb_dealloc_cmap(&fbi->cmap);

	unregister_framebuffer(fbi);
	framebuffer_release(fbi);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int mxc_elcdif_fb_suspend(struct platform_device *pdev,
			       pm_message_t state)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mxc_elcdif_fb_data *data = (struct mxc_elcdif_fb_data *)fbi->par;
	int saved_blank;

	console_lock();
	fb_set_suspend(fbi, 1);
	saved_blank = data->cur_blank;
	mxc_elcdif_fb_blank(FB_BLANK_POWERDOWN, fbi);
	data->next_blank = saved_blank;
	if (!g_elcdif_pix_clk_enable) {
		clk_enable(g_elcdif_pix_clk);
		g_elcdif_pix_clk_enable = true;
	}
	mxc_elcdif_stop();
	mxc_elcdif_dma_release();
	if (g_elcdif_pix_clk_enable) {
		clk_disable(g_elcdif_pix_clk);
		g_elcdif_pix_clk_enable = false;
	}
	if (g_elcdif_axi_clk_enable) {
		clk_disable(g_elcdif_axi_clk);
		g_elcdif_axi_clk_enable = false;
	}
	data->running = false;
	console_unlock();
	return 0;
}

static int mxc_elcdif_fb_resume(struct platform_device *pdev)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mxc_elcdif_fb_data *data = (struct mxc_elcdif_fb_data *)fbi->par;

	console_lock();
	mxc_elcdif_fb_blank(data->next_blank, fbi);
	fb_set_suspend(fbi, 0);
	console_unlock();

	return 0;
}
#else
#define	mxc_elcdif_fb_suspend	NULL
#define	mxc_elcdif_fb_resume	NULL
#endif

static struct platform_driver mxc_elcdif_fb_driver = {
	.probe		= mxc_elcdif_fb_probe,
	.remove		= mxc_elcdif_fb_remove,
	.suspend	= mxc_elcdif_fb_suspend,
	.resume		= mxc_elcdif_fb_resume,
	.driver		= {
		.name   = "mxc_elcdif_fb",
		.owner	= THIS_MODULE,
	},
};

/*
 * Parse user specified options (`video=trident:')
 * example:
 * 	video=trident:800x600,bpp=16,noaccel
 */
int mxc_elcdif_fb_setup(char *options)
{
	char *opt;
	if (!options || !*options)
		return 0;
	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;

		if (!strncmp(opt, "bpp=", 4))
			default_bpp = simple_strtoul(opt + 4, NULL, 0);
		else
			fb_mode = opt;
	}
	return 0;
}

static int __init mxc_elcdif_fb_init(void)
{
	char *option = NULL;

	if (fb_get_options("mxc_elcdif_fb", &option))
		return -ENODEV;
	mxc_elcdif_fb_setup(option);

	return platform_driver_register(&mxc_elcdif_fb_driver);
}

static void __exit mxc_elcdif_fb_exit(void)
{
	platform_driver_unregister(&mxc_elcdif_fb_driver);
}

module_init(mxc_elcdif_fb_init);
module_exit(mxc_elcdif_fb_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC ELCDIF Framebuffer Driver");
MODULE_LICENSE("GPL");
