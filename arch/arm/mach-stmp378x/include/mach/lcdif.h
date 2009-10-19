/*
 * Freescale STMP378X LCDIF interfaces
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef _ARCH_ARM_LCDIF_H
#define _ARCH_ARM_LCDIF_H

#include <linux/types.h>
#include <linux/fb.h>
#include <linux/list.h>
#include <linux/backlight.h>
#include <linux/dma-mapping.h>
#include <linux/regulator/consumer.h>
#include <mach/dma.h>
#include <mach/platform.h>

#include "regs-lcdif.h"
#include "regs-apbh.h"

enum {
	SPI_MOSI = 0,
	SPI_SCLK,
	SPI_CS,
};

struct stmp3xxx_lcd_dma_chain_info {
	dma_addr_t *dma_addr_p;
	unsigned offset;
};

enum {
	STMP3XXX_LCD_PANEL_SYSTEM = 0,
	STMP3XXX_LCD_PANEL_VSYNC,
	STMP3XXX_LCD_PANEL_DOTCLK,
	STMP3XXX_LCD_PANEL_DVI,
};

struct stmp3xxx_platform_bl_data;
struct stmp3xxx_platform_fb_entry {
	char name[16];
	u16 x_res;
	u16 y_res;
	u16 bpp;
	u32 cycle_time_ns;
	int lcd_type;
	int (*init_panel) (struct device * dev, dma_addr_t phys, int memsize,
			   struct stmp3xxx_platform_fb_entry * pentry);
	void (*release_panel) (struct device * dev,
			       struct stmp3xxx_platform_fb_entry * pentry);
	int (*blank_panel) (int blank);
	void (*run_panel) (void);
	void (*stop_panel) (void);
	int (*pan_display) (dma_addr_t phys);
	int (*update_panel) (void *p,
			     struct stmp3xxx_platform_fb_entry * pentry);
	struct list_head link;
	struct stmp3xxx_platform_bl_data *bl_data;
};

struct stmp3xxx_platform_fb_data {
	struct list_head list;
	struct stmp3xxx_platform_fb_entry *cur;
};

#define STMP3XXX_LCDIF_PANEL_INIT	1
#define STMP3XXX_LCDIF_PANEL_RELEASE	2

struct stmp3xxx_platform_bl_data {
	struct list_head list;
	struct regulator *regulator;
	int bl_gpio;
	int bl_max_intensity;
	int bl_cons_intensity;
	int bl_default_intensity;
	int (*init_bl) (struct stmp3xxx_platform_bl_data * data);
	int (*set_bl_intensity) (struct stmp3xxx_platform_bl_data * data,
				 struct backlight_device * bd, int suspended);
	void (*free_bl) (struct stmp3xxx_platform_bl_data * data);
};

static inline void stmp3xxx_lcd_register_entry(struct stmp3xxx_platform_fb_entry
					       *pentry,
					       struct stmp3xxx_platform_fb_data
					       *pdata)
{
	list_add_tail(&pentry->link, &pdata->list);
	if (!pdata->cur)
		pdata->cur = pentry;
}

static inline void stmp3xxx_lcd_move_pentry_up(struct stmp3xxx_platform_fb_entry
					       *pentry,
					       struct stmp3xxx_platform_fb_data
					       *pdata)
{
	list_del(&pentry->link);
	list_add(&pentry->link, &pdata->list);
}

static inline int stmp3xxx_lcd_iterate_pdata(struct stmp3xxx_platform_fb_data
					     *pdata,
					     int (*func) (struct
							  stmp3xxx_platform_fb_entry
							  * pentry, void *data,
							  int ret_prev),
					     void *data)
{
	struct stmp3xxx_platform_fb_entry *pentry;
	int ret = 0;
	list_for_each_entry(pentry, &pdata->list, link) {
		ret = func(pentry, data, ret);
	}
	return ret;
}

static inline void stmp3xxx_lcd_set_bl_pdata(struct stmp3xxx_platform_bl_data
					     *pdata)
{
	extern struct platform_device stmp3xxx_backlight;
	stmp3xxx_backlight.dev.platform_data = pdata;
}

void stmp3xxx_init_lcdif(void);
int stmp3xxx_lcdif_dma_init(struct device *dev, dma_addr_t phys, int memsize,
			    int lcd_master);
void stmp3xxx_lcdif_dma_release(void);
void stmp3xxx_lcdif_run(void);
void stmp3xxx_lcdif_stop(void);
int stmp3xxx_lcdif_pan_display(dma_addr_t addr);

int stmp3xxx_lcdif_register_client(struct notifier_block *nb);
void stmp3xxx_lcdif_unregister_client(struct notifier_block *nb);
void stmp3xxx_lcdif_notify_clients(unsigned long event,
				   struct stmp3xxx_platform_fb_entry *pentry);

#ifndef FBIO_WAITFORVSYNC
#define FBIO_WAITFORVSYNC		_IOW('F', 0x20, u_int32_t)
#endif

#define LCD_DMA_CHANNEL		0

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

	stmp3xxx_clearl(BM_LCDIF_CTRL_DATA_SHIFT_DIR,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);

	stmp3xxx_clearl(BM_LCDIF_CTRL_SHIFT_NUM_BITS,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);

	stmp3xxx_clearl(BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL1);
	stmp3xxx_setl(BF(7, LCDIF_CTRL1_BYTE_PACKING_FORMAT) |
		      BM_LCDIF_CTRL1_RECOVER_ON_UNDERFLOW,
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL1);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);
	val &= ~(BM_LCDIF_TRANSFER_COUNT_V_COUNT |
			BM_LCDIF_TRANSFER_COUNT_H_COUNT);
	val |= BF(h_active, LCDIF_TRANSFER_COUNT_H_COUNT) |
			BF(v_active, LCDIF_TRANSFER_COUNT_V_COUNT);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);

	stmp3xxx_clearl(BM_LCDIF_CTRL_VSYNC_MODE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	stmp3xxx_clearl(BM_LCDIF_CTRL_WAIT_FOR_VSYNC_EDGE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	stmp3xxx_clearl(BM_LCDIF_CTRL_DVI_MODE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	stmp3xxx_setl(BM_LCDIF_CTRL_DOTCLK_MODE,
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	stmp3xxx_setl(BM_LCDIF_CTRL_BYPASS_COUNT,
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL);

	stmp3xxx_clearl(BM_LCDIF_CTRL_WORD_LENGTH |
			BM_LCDIF_CTRL_INPUT_DATA_SWIZZLE |
			BM_LCDIF_CTRL_LCD_DATABUS_WIDTH,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	stmp3xxx_setl(BF(3, LCDIF_CTRL_WORD_LENGTH) |	/* 24 bit */
		      BM_LCDIF_CTRL_DATA_SELECT |	/* data mode */
		      BF(0, LCDIF_CTRL_INPUT_DATA_SWIZZLE) |	/* no swap */
		      BF(3, LCDIF_CTRL_LCD_DATABUS_WIDTH),	/* 24 bit */
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);
	val &= ~(BM_LCDIF_VDCTRL0_VSYNC_POL |
			BM_LCDIF_VDCTRL0_HSYNC_POL |
			BM_LCDIF_VDCTRL0_ENABLE_POL |
			BM_LCDIF_VDCTRL0_DOTCLK_POL);
	val |= BM_LCDIF_VDCTRL0_ENABLE_POL |
			BM_LCDIF_VDCTRL0_DOTCLK_POL;
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);
	val &= ~(BM_LCDIF_VDCTRL0_VSYNC_OEB);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);	/* vsync is output */

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

	stmp3xxx_clearl(BM_LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH,
			REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);
	stmp3xxx_setl(v_pulse_width, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);

	stmp3xxx_clearl(BM_LCDIF_VDCTRL1_VSYNC_PERIOD,
			REGS_LCDIF_BASE + HW_LCDIF_VDCTRL1);
	stmp3xxx_setl(v_period, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL1);

	stmp3xxx_clearl(BM_LCDIF_VDCTRL2_HSYNC_PERIOD |
			BM_LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH,
			REGS_LCDIF_BASE + HW_LCDIF_VDCTRL2);

	stmp3xxx_setl(BF(h_pulse_width, LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH) |
		      BF(h_period, LCDIF_VDCTRL2_HSYNC_PERIOD),
		      REGS_LCDIF_BASE + HW_LCDIF_VDCTRL2);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL4);
	val &= ~BM_LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT;
	val |= BF(h_active, LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL4);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL3);
	val &= ~(BM_LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT |
			BM_LCDIF_VDCTRL3_VERTICAL_WAIT_CNT);
	val |= BF(h_wait_cnt, LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT) |
			BF(v_wait_cnt, LCDIF_VDCTRL3_VERTICAL_WAIT_CNT);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL3);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_VDCTRL4);
	val |= BM_LCDIF_VDCTRL4_SYNC_SIGNALS_ON;
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL4);
}

static inline void release_dotclk_panel(void)
{
	stmp3xxx_clearl(BM_LCDIF_CTRL_DOTCLK_MODE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	__raw_writel(0, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);
	__raw_writel(0, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL1);
	__raw_writel(0, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL2);
	__raw_writel(0, REGS_LCDIF_BASE + HW_LCDIF_VDCTRL3);
}

static inline void dotclk_dma_chain_init(int memsize, dma_addr_t video_phys,
					 struct stmp3xxx_dma_descriptor
					 *video_dma_descriptor,
					 struct stmp3xxx_lcd_dma_chain_info
					 *dma_chain_info,
					 unsigned *dma_chain_info_pos)
{
	unsigned i, bytes_left;
	dma_addr_t phys = video_phys;
	bytes_left = memsize;

	for (i = 0; bytes_left > 0; ++i) {
		unsigned this_chain = bytes_left < 0xff00 ? bytes_left : 0xff00;
		/* Count of 0 in the DMA word means 65536 */
		unsigned xfer_count = this_chain & 65535;
		stmp3xxx_dma_allocate_command(STMP3XXX_DMA
					      (LCD_DMA_CHANNEL,
					       STMP3XXX_BUS_APBH),
					      &video_dma_descriptor[i]);
		if (i != 0) {
			/* Chain previous command to this one */
			video_dma_descriptor[i - 1].command->next =
			    video_dma_descriptor[i].handle;
			/* Enable DMA chaining, disable IRQ and semaphore
			 * on previous command
			 */
			video_dma_descriptor[i - 1].command->cmd &=
			    ~(BM_APBH_CHn_CMD_IRQONCMPLT |
			      BM_APBH_CHn_CMD_SEMAPHORE);
		}
		video_dma_descriptor[i].command->cmd =
						BF(xfer_count, APBH_CHn_CMD_XFER_COUNT) |
						BF(1, APBH_CHn_CMD_CMDWORDS) |
						BM_APBH_CHn_CMD_CHAIN |
						BF(2, APBH_CHn_CMD_COMMAND);	/* DMA read */
		video_dma_descriptor[i].command->pio_words[0] =
		    BM_LCDIF_CTRL_RUN |
		    BF(1, LCDIF_CTRL_INPUT_DATA_SWIZZLE) |
		    BM_LCDIF_CTRL_DATA_SHIFT_DIR |
		    BM_LCDIF_CTRL_DOTCLK_MODE |
		    BM_LCDIF_CTRL_BYPASS_COUNT | BM_LCDIF_CTRL_DATA_SELECT;
		video_dma_descriptor[i].command->buf_ptr = phys;
		dma_chain_info[*dma_chain_info_pos].dma_addr_p =
		    &video_dma_descriptor[i].command->buf_ptr;
		dma_chain_info[*dma_chain_info_pos].offset = phys - video_phys;
		++*dma_chain_info_pos;
		phys += this_chain;
		bytes_left -= this_chain;
	}
	video_dma_descriptor[i - 1].command->next =
	    video_dma_descriptor[0].handle;
	pr_debug("%s: Used %u DMA chains to cover %u bytes\n", __func__, i,
		 memsize);
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
	stmp3xxx_clearl(BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL1);
	stmp3xxx_setl(BF(0x7, LCDIF_CTRL1_BYTE_PACKING_FORMAT) |
		      BM_LCDIF_CTRL1_RECOVER_ON_UNDERFLOW,
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL1);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);
	val &= ~(BM_LCDIF_TRANSFER_COUNT_V_COUNT |
			BM_LCDIF_TRANSFER_COUNT_H_COUNT);
	val |= BF(h_active, LCDIF_TRANSFER_COUNT_H_COUNT) |
			BF(v_active, LCDIF_TRANSFER_COUNT_V_COUNT);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);

	/* set lcdif to DVI mode */
	stmp3xxx_setl(BM_LCDIF_CTRL_DVI_MODE, REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	stmp3xxx_clearl(BM_LCDIF_CTRL_VSYNC_MODE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	stmp3xxx_clearl(BM_LCDIF_CTRL_DOTCLK_MODE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);

	stmp3xxx_setl(BM_LCDIF_CTRL_BYPASS_COUNT,
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	/* convert input RGB -> YCbCr */
	stmp3xxx_setl(BM_LCDIF_CTRL_RGB_TO_YCBCR422_CSC,
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	/* interlace odd and even fields */
	stmp3xxx_setl(BM_LCDIF_CTRL1_INTERLACE_FIELDS,
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL1);

	stmp3xxx_clearl(BM_LCDIF_CTRL_WORD_LENGTH |
			BM_LCDIF_CTRL_INPUT_DATA_SWIZZLE |
			BM_LCDIF_CTRL_LCD_DATABUS_WIDTH,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	stmp3xxx_setl(BF(3, LCDIF_CTRL_WORD_LENGTH) |	/* 24 bit */
		      BM_LCDIF_CTRL_DATA_SELECT |	/* data mode */
		      BF(0, LCDIF_CTRL_INPUT_DATA_SWIZZLE) |	/* no swap */
		      BF(1, LCDIF_CTRL_LCD_DATABUS_WIDTH),	/* 8 bit */
		      REGS_LCDIF_BASE + HW_LCDIF_CTRL);

	/* LCDIF_DVI */
	/* set frame size */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL0);
	val &= ~(BM_LCDIF_DVICTRL0_H_ACTIVE_CNT |
		      BM_LCDIF_DVICTRL0_H_BLANKING_CNT |
		      BM_LCDIF_DVICTRL0_V_LINES_CNT);
	val |= BF(1440, LCDIF_DVICTRL0_H_ACTIVE_CNT) |
		      BF(h_blanking, LCDIF_DVICTRL0_H_BLANKING_CNT) |
		      BF(v_lines, LCDIF_DVICTRL0_V_LINES_CNT);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_DVICTRL0);

	/* set start/end of field-1 and start of field-2 */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL1);
	val &= ~(BM_LCDIF_DVICTRL1_F1_START_LINE |
		      BM_LCDIF_DVICTRL1_F1_END_LINE |
		      BM_LCDIF_DVICTRL1_F2_START_LINE);
	val |= BF(f1_start, LCDIF_DVICTRL1_F1_START_LINE) |
		BF(f1_end, LCDIF_DVICTRL1_F1_END_LINE) |
		BF(f2_start, LCDIF_DVICTRL1_F2_START_LINE);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_DVICTRL1);

	/* set first vertical blanking interval and end of filed-2 */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL2);
	val &= ~(BM_LCDIF_DVICTRL2_F2_END_LINE |
		      BM_LCDIF_DVICTRL2_V1_BLANK_START_LINE |
		      BM_LCDIF_DVICTRL2_V1_BLANK_END_LINE);
	val |= BF(f2_end, LCDIF_DVICTRL2_F2_END_LINE) |
		      BF(v1_blank_start, LCDIF_DVICTRL2_V1_BLANK_START_LINE) |
		      BF(v1_blank_end, LCDIF_DVICTRL2_V1_BLANK_END_LINE);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_DVICTRL2);

	/* set second vertical blanking interval */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_DVICTRL3);
	val &= ~(BM_LCDIF_DVICTRL3_V2_BLANK_START_LINE |
		      BM_LCDIF_DVICTRL3_V2_BLANK_END_LINE);
	val |= BF(v2_blank_start, LCDIF_DVICTRL3_V2_BLANK_START_LINE) |
		      BF(v2_blank_end, LCDIF_DVICTRL3_V2_BLANK_END_LINE);
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
		val |= BF(720 - h_active, LCDIF_DVICTRL4_H_FILL_CNT) |
			      BF(16, LCDIF_DVICTRL4_Y_FILL_VALUE) |
			      BF(128, LCDIF_DVICTRL4_CB_FILL_VALUE) |
			      BF(128, LCDIF_DVICTRL4_CR_FILL_VALUE);
		__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_DVICTRL4);
	}

	/* Color Space Conversion RGB->YCbCr */
	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF0);
	val &= ~(BM_LCDIF_CSC_COEFF0_C0 |
		      BM_LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER);
	val |= BF(0x41, LCDIF_CSC_COEFF0_C0) |
		      BF(3, LCDIF_CSC_COEFF0_CSC_SUBSAMPLE_FILTER);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF0);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF1);
	val &= ~(BM_LCDIF_CSC_COEFF1_C1 | BM_LCDIF_CSC_COEFF1_C2);
	val |= BF(0x81, LCDIF_CSC_COEFF1_C1) |
		      BF(0x19, LCDIF_CSC_COEFF1_C2);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF1);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF2);
	val &= ~(BM_LCDIF_CSC_COEFF2_C3 | BM_LCDIF_CSC_COEFF2_C4);
	val |= BF(0x3DB, LCDIF_CSC_COEFF2_C3) |
		      BF(0x3B6, LCDIF_CSC_COEFF2_C4);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF2);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF3);
	val &= ~(BM_LCDIF_CSC_COEFF3_C5 | BM_LCDIF_CSC_COEFF3_C6);
	val |= BF(0x70, LCDIF_CSC_COEFF3_C5) |
		      BF(0x70, LCDIF_CSC_COEFF3_C6);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF3);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF4);
	val &= ~(BM_LCDIF_CSC_COEFF4_C7 | BM_LCDIF_CSC_COEFF4_C8);
	val |= BF(0x3A2, LCDIF_CSC_COEFF4_C7) | BF(0x3EE, LCDIF_CSC_COEFF4_C8);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_COEFF4);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_OFFSET);
	val &= ~(BM_LCDIF_CSC_OFFSET_CBCR_OFFSET | BM_LCDIF_CSC_OFFSET_Y_OFFSET);
	val |= BF(0x80, LCDIF_CSC_OFFSET_CBCR_OFFSET) |
		      BF(0x10, LCDIF_CSC_OFFSET_Y_OFFSET);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_OFFSET);

	val = __raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CSC_LIMIT);
	val &= ~(BM_LCDIF_CSC_LIMIT_CBCR_MIN |
		      BM_LCDIF_CSC_LIMIT_CBCR_MAX |
		      BM_LCDIF_CSC_LIMIT_Y_MIN |
		      BM_LCDIF_CSC_LIMIT_Y_MAX);
	val |= BF(16, LCDIF_CSC_LIMIT_CBCR_MIN) |
		      BF(240, LCDIF_CSC_LIMIT_CBCR_MAX) |
		      BF(16, LCDIF_CSC_LIMIT_Y_MIN) |
		      BF(235, LCDIF_CSC_LIMIT_Y_MAX);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_CSC_LIMIT);
}

static inline void release_dvi_panel(void)
{
	stmp3xxx_clearl(BM_LCDIF_CTRL_DVI_MODE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);
}

#endif /* _ARCH_ARM_LCDIF_H */
