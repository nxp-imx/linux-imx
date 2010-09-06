/*
 * Freescale MXS LCDIF interfaces
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _ARCH_ARM_LCDIF_H
#define _ARCH_ARM_LCDIF_H

#include <linux/types.h>
#include <linux/fb.h>
#include <linux/list.h>
#include <linux/backlight.h>
#include <linux/dma-mapping.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

#include <mach/device.h>
#include <mach/hardware.h>

#include "regs-lcdif.h"

#define REGS_LCDIF_BASE IO_ADDRESS(LCDIF_PHYS_ADDR)

enum {
	SPI_MOSI = 0,
	SPI_SCLK,
	SPI_CS,
};

struct mxs_lcd_dma_chain_info {
	dma_addr_t *dma_addr_p;
	unsigned offset;
};

enum {
	MXS_LCD_PANEL_SYSTEM = 0,
	MXS_LCD_PANEL_VSYNC,
	MXS_LCD_PANEL_DOTCLK,
	MXS_LCD_PANEL_DVI,
};

struct mxs_platform_bl_data;
struct mxs_platform_fb_entry {
	char name[16];
	u16 x_res;
	u16 y_res;
	u16 bpp;
	u32 cycle_time_ns;
	int lcd_type;
	int (*init_panel) (struct device *, dma_addr_t, int,
			   struct mxs_platform_fb_entry *);
	void (*release_panel) (struct device *, struct mxs_platform_fb_entry *);
	int (*blank_panel) (int);
	void (*run_panel) (void);
	void (*stop_panel) (void);
	int (*pan_display) (dma_addr_t);
	int (*update_panel) (void *, struct mxs_platform_fb_entry *);
	struct list_head link;
	struct mxs_platform_bl_data *bl_data;
};

struct mxs_platform_fb_data {
	struct list_head list;
	struct mxs_platform_fb_entry *cur;
	struct mxs_platform_fb_entry *next;
};

#define MXS_LCDIF_PANEL_INIT	1
#define MXS_LCDIF_PANEL_RELEASE	2

struct mxs_platform_bl_data {
	struct list_head list;
	struct regulator *regulator;
	int bl_gpio;
	int bl_max_intensity;
	int bl_cons_intensity;
	int bl_default_intensity;
	int (*init_bl) (struct mxs_platform_bl_data *);
	int (*set_bl_intensity) (struct mxs_platform_bl_data *,
				 struct backlight_device *, int);
	void (*free_bl) (struct mxs_platform_bl_data *);
};

static inline void mxs_lcd_register_entry(struct mxs_platform_fb_entry
					  *pentry, struct mxs_platform_fb_data
					  *pdata)
{
	list_add_tail(&pentry->link, &pdata->list);
	if (!pdata->cur)
		pdata->cur = pentry;
}

static inline void mxs_lcd_move_pentry_up(struct mxs_platform_fb_entry
					  *pentry, struct mxs_platform_fb_data
					  *pdata)
{
	list_move(&pentry->link, &pdata->list);
}

static inline int mxs_lcd_iterate_pdata(struct mxs_platform_fb_data
					*pdata,
					int (*func) (struct
						     mxs_platform_fb_entry
						     * pentry, void *data,
						     int ret_prev), void *data)
{
	struct mxs_platform_fb_entry *pentry;
	int ret = 0;
	list_for_each_entry(pentry, &pdata->list, link) {
		ret = func(pentry, data, ret);
	}
	return ret;
}

static inline void mxs_lcd_set_bl_pdata(struct mxs_platform_bl_data
					*pdata)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-bl", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;

	pdev->dev.platform_data = pdata;
}

void mxs_init_lcdif(void);
int mxs_lcdif_dma_init(struct device *dev, dma_addr_t phys, int memsize);
void mxs_lcdif_dma_release(void);
void mxs_lcdif_run(void);
void mxs_lcdif_stop(void);
int mxs_lcdif_pan_display(dma_addr_t addr);

int mxs_lcdif_register_client(struct notifier_block *nb);
void mxs_lcdif_unregister_client(struct notifier_block *nb);
void mxs_lcdif_notify_clients(unsigned long event,
			      struct mxs_platform_fb_entry *pentry);

#ifndef FBIO_WAITFORVSYNC
#define FBIO_WAITFORVSYNC		_IOW('F', 0x20, u_int32_t)
#endif

static inline void setup_dotclk_panel(u16 v_pulse_width,
				      u16 v_period,
				      u16 v_wait_cnt,
				      u16 v_active,
				      u16 h_pulse_width,
				      u16 h_period,
				      u16 h_wait_cnt,
				      u16 h_active, int enable_present)
{
	u32 val;

	__raw_writel(BM_LCDIF_CTRL_DATA_SHIFT_DIR,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);

	__raw_writel(BM_LCDIF_CTRL_SHIFT_NUM_BITS,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);

	__raw_writel(BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);
	__raw_writel(BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(7) |
		     BM_LCDIF_CTRL1_RECOVER_ON_UNDERFLOW,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);
	val &= ~(BM_LCDIF_TRANSFER_COUNT_V_COUNT |
		 BM_LCDIF_TRANSFER_COUNT_H_COUNT);
	val |= BF_LCDIF_TRANSFER_COUNT_H_COUNT(h_active) |
	    BF_LCDIF_TRANSFER_COUNT_V_COUNT(v_active);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);

	__raw_writel(BM_LCDIF_CTRL_VSYNC_MODE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	__raw_writel(BM_LCDIF_CTRL_WAIT_FOR_VSYNC_EDGE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	__raw_writel(BM_LCDIF_CTRL_DVI_MODE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	__raw_writel(BM_LCDIF_CTRL_DOTCLK_MODE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	__raw_writel(BM_LCDIF_CTRL_BYPASS_COUNT,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);

	__raw_writel(BM_LCDIF_CTRL_WORD_LENGTH |
		     BM_LCDIF_CTRL_INPUT_DATA_SWIZZLE |
		     BM_LCDIF_CTRL_LCD_DATABUS_WIDTH,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	__raw_writel(BF_LCDIF_CTRL_WORD_LENGTH(3) |/* 24 bit */
		     BM_LCDIF_CTRL_DATA_SELECT |/* data mode */
		     BF_LCDIF_CTRL_INPUT_DATA_SWIZZLE(0) |/* no swap */
		     BF_LCDIF_CTRL_LCD_DATABUS_WIDTH(3),/* 24 bit */
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);
	val &= ~(BM_LCDIF_VDCTRL0_VSYNC_POL |
		 BM_LCDIF_VDCTRL0_HSYNC_POL |
		 BM_LCDIF_VDCTRL0_ENABLE_POL | BM_LCDIF_VDCTRL0_DOTCLK_POL);
	val |= BM_LCDIF_VDCTRL0_ENABLE_POL | BM_LCDIF_VDCTRL0_DOTCLK_POL;
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);
	val &= ~(BM_LCDIF_VDCTRL0_VSYNC_OEB);
	/* vsync is output */
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);

	/*
	 * need enable sig for true RGB i/f.  Or, if not true RGB, leave it
	 * zero.
	 */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);
	val |= BM_LCDIF_VDCTRL0_ENABLE_PRESENT;
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);

	/*
	 * For DOTCLK mode, count VSYNC_PERIOD in terms of complete hz lines
	 */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);
	val &= ~(BM_LCDIF_VDCTRL0_VSYNC_PERIOD_UNIT |
		 BM_LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT);
	val |= BM_LCDIF_VDCTRL0_VSYNC_PERIOD_UNIT |
	    BM_LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT;
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);

	__raw_writel(BM_LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH,
		     REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0_CLR);
	__raw_writel(v_pulse_width, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0_SET);

	__raw_writel(BF_LCDIF_VDCTRL1_VSYNC_PERIOD(v_period),
		     REGS_LCDIF_BASE + HW_LCDIF_VDCTRL1);

	__raw_writel(BF_LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH(h_pulse_width) |
		     BF_LCDIF_VDCTRL2_HSYNC_PERIOD(h_period),
		     REGS_LCDIF_BASE + HW_LCDIF_VDCTRL2);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL4);
	val &= ~BM_LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT;
	val |= BF_LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT(h_active);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL4);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL3);
	val &= ~(BM_LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT |
		 BM_LCDIF_VDCTRL3_VERTICAL_WAIT_CNT);
	val |= BF_LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT(h_wait_cnt) |
	    BF_LCDIF_VDCTRL3_VERTICAL_WAIT_CNT(v_wait_cnt);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL3);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL4);
	val |= BM_LCDIF_VDCTRL4_SYNC_SIGNALS_ON;
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL4);
}

static inline void release_dotclk_panel(void)
{
	__raw_writel(BM_LCDIF_CTRL_DOTCLK_MODE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	__raw_writel(0, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);
	__raw_writel(0, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL1);
	__raw_writel(0, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL2);
	__raw_writel(0, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL3);
}

static inline void setup_dvi_panel(u16 h_active, u16 v_active,
				   u16 h_blanking, u16 v_lines,
				   u16 v1_blank_start, u16 v1_blank_end,
				   u16 v2_blank_start, u16 v2_blank_end,
				   u16 f1_start, u16 f1_end,
				   u16 f2_start, u16 f2_end)
{
	u32 val;
	/* 32bit packed format (RGB) */
	__raw_writel(BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);
	__raw_writel(BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x7) |
		      BM_LCDIF_CTRL1_RECOVER_ON_UNDERFLOW,
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);
	val &= ~(BM_LCDIF_TRANSFER_COUNT_V_COUNT |
			BM_LCDIF_TRANSFER_COUNT_H_COUNT);
	val |= BF_LCDIF_TRANSFER_COUNT_H_COUNT(h_active) |
			BF_LCDIF_TRANSFER_COUNT_V_COUNT(v_active);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);

	/* set lcdif to DVI mode */
	__raw_writel(BM_LCDIF_CTRL_DVI_MODE,
		REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	__raw_writel(BM_LCDIF_CTRL_VSYNC_MODE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	__raw_writel(BM_LCDIF_CTRL_DOTCLK_MODE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);

	__raw_writel(BM_LCDIF_CTRL_BYPASS_COUNT,
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	/* convert input RGB -> YCbCr */
	__raw_writel(BM_LCDIF_CTRL_RGB_TO_YCBCR422_CSC,
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	/* interlace odd and even fields */
	__raw_writel(BM_LCDIF_CTRL1_INTERLACE_FIELDS,
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);

	__raw_writel(BM_LCDIF_CTRL_WORD_LENGTH |
			BM_LCDIF_CTRL_INPUT_DATA_SWIZZLE |
			BM_LCDIF_CTRL_LCD_DATABUS_WIDTH,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	__raw_writel(BF_LCDIF_CTRL_WORD_LENGTH(3) |	/* 24 bit */
		      BM_LCDIF_CTRL_DATA_SELECT |	/* data mode */
		      BF_LCDIF_CTRL_INPUT_DATA_SWIZZLE(0) |	/* no swap */
		      BF_LCDIF_CTRL_LCD_DATABUS_WIDTH(1),	/* 8 bit */
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);

	/* LCDIF_DVI */
	/* set frame size */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL0);
	val &= ~(BM_LCDIF_DVICTRL0_H_ACTIVE_CNT |
		      BM_LCDIF_DVICTRL0_H_BLANKING_CNT |
		      BM_LCDIF_DVICTRL0_V_LINES_CNT);
	val |= BF_LCDIF_DVICTRL0_H_ACTIVE_CNT(1440) |
		      BF_LCDIF_DVICTRL0_H_BLANKING_CNT(h_blanking) |
		      BF_LCDIF_DVICTRL0_V_LINES_CNT(v_lines);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_DVICTRL0);

	/* set start/end of field-1 and start of field-2 */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL1);
	val &= ~(BM_LCDIF_DVICTRL1_F1_START_LINE |
		      BM_LCDIF_DVICTRL1_F1_END_LINE |
		      BM_LCDIF_DVICTRL1_F2_START_LINE);
	val |= BF_LCDIF_DVICTRL1_F1_START_LINE(f1_start) |
		BF_LCDIF_DVICTRL1_F1_END_LINE(f1_end) |
		BF_LCDIF_DVICTRL1_F2_START_LINE(f2_start);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_DVICTRL1);

	/* set first vertical blanking interval and end of filed-2 */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL2);
	val &= ~(BM_LCDIF_DVICTRL2_F2_END_LINE |
		      BM_LCDIF_DVICTRL2_V1_BLANK_START_LINE |
		      BM_LCDIF_DVICTRL2_V1_BLANK_END_LINE);
	val |= BF_LCDIF_DVICTRL2_F2_END_LINE(f2_end) |
		      BF_LCDIF_DVICTRL2_V1_BLANK_START_LINE(v1_blank_start) |
		      BF_LCDIF_DVICTRL2_V1_BLANK_END_LINE(v1_blank_end);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_DVICTRL2);

	/* set second vertical blanking interval */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL3);
	val &= ~(BM_LCDIF_DVICTRL3_V2_BLANK_START_LINE |
		      BM_LCDIF_DVICTRL3_V2_BLANK_END_LINE);
	val |= BF_LCDIF_DVICTRL3_V2_BLANK_START_LINE(v2_blank_start) |
		      BF_LCDIF_DVICTRL3_V2_BLANK_END_LINE(v2_blank_end);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_DVICTRL3);

	/* fill the rest area black color if the input frame
	 * is not 720 pixels/line
	 */
	if (h_active != 720) {
		/* the input frame can't be less then (720-256) pixels/line */
		if (720 - h_active > 0xff)
			h_active = 720 - 0xff;

		val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL4);
		val &= ~(BM_LCDIF_DVICTRL4_H_FILL_CNT |
			      BM_LCDIF_DVICTRL4_Y_FILL_VALUE |
			      BM_LCDIF_DVICTRL4_CB_FILL_VALUE |
			      BM_LCDIF_DVICTRL4_CR_FILL_VALUE);
		val |= BF_LCDIF_DVICTRL4_H_FILL_CNT(720 - h_active) |
			      BF_LCDIF_DVICTRL4_Y_FILL_VALUE(16) |
			      BF_LCDIF_DVICTRL4_CB_FILL_VALUE(128) |
			      BF_LCDIF_DVICTRL4_CR_FILL_VALUE(128);
		__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_DVICTRL4);
	}

	/* Color Space Conversion RGB->YCbCr */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF0);
	val &= ~(BM_LCDIF_CSC_COEFF0_C0 |
		      BM_LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER);
	val |= BF_LCDIF_CSC_COEFF0_C0(0x41) |
		      BF_LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER(3);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF0);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF1);
	val &= ~(BM_LCDIF_CSC_COEFF1_C1 | BM_LCDIF_CSC_COEFF1_C2);
	val |= BF_LCDIF_CSC_COEFF1_C1(0x81) |
		      BF_LCDIF_CSC_COEFF1_C2(0x19);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF1);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF2);
	val &= ~(BM_LCDIF_CSC_COEFF2_C3 | BM_LCDIF_CSC_COEFF2_C4);
	val |= BF_LCDIF_CSC_COEFF2_C3(0x3DB) |
		      BF_LCDIF_CSC_COEFF2_C4(0x3B6);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF2);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF3);
	val &= ~(BM_LCDIF_CSC_COEFF3_C5 | BM_LCDIF_CSC_COEFF3_C6);
	val |= BF_LCDIF_CSC_COEFF3_C5(0x70) |
		      BF_LCDIF_CSC_COEFF3_C6(0x70);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF3);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF4);
	val &= ~(BM_LCDIF_CSC_COEFF4_C7 | BM_LCDIF_CSC_COEFF4_C8);
	val |= BF_LCDIF_CSC_COEFF4_C7(0x3A2) | BF_LCDIF_CSC_COEFF4_C8(0x3EE);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF4);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_OFFSET);
	val &= ~(BM_LCDIF_CSC_OFFSET_CBCR_OFFSET
		| BM_LCDIF_CSC_OFFSET_Y_OFFSET);
	val |= BF_LCDIF_CSC_OFFSET_CBCR_OFFSET(0x80) |
		      BF_LCDIF_CSC_OFFSET_Y_OFFSET(0x10);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_OFFSET);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_LIMIT);
	val &= ~(BM_LCDIF_CSC_LIMIT_CBCR_MIN |
		      BM_LCDIF_CSC_LIMIT_CBCR_MAX |
		      BM_LCDIF_CSC_LIMIT_Y_MIN |
		      BM_LCDIF_CSC_LIMIT_Y_MAX);
	val |= BF_LCDIF_CSC_LIMIT_CBCR_MIN(16) |
		      BF_LCDIF_CSC_LIMIT_CBCR_MAX(240) |
		      BF_LCDIF_CSC_LIMIT_Y_MIN(16) |
		      BF_LCDIF_CSC_LIMIT_Y_MAX(235);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_LIMIT);
}

static inline void release_dvi_panel(void)
{
	__raw_writel(BM_LCDIF_CTRL_DVI_MODE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
}
#endif /* _ARCH_ARM_LCDIF_H */
