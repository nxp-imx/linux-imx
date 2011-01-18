/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
/*
 * Based on STMP378X LCDIF
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/cpufreq.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/dmaengine.h>
#include <linux/pxp_dma.h>
#include <linux/mxcfb.h>
#include <linux/mxcfb_epdc_kernel.h>
#include <linux/gpio.h>
#include <linux/regulator/driver.h>
#include <linux/fsl_devices.h>

#include <linux/time.h>

#include "epdc_regs.h"

/*
 * Enable this define to have a default panel
 * loaded during driver initialization
 */
/*#define DEFAULT_PANEL_HW_INIT*/

#define NUM_SCREENS_MIN	2
#define EPDC_NUM_LUTS 16
#define EPDC_MAX_NUM_UPDATES 20
#define INVALID_LUT -1

#define DEFAULT_TEMP_INDEX	0
#define DEFAULT_TEMP		20 /* room temp in deg Celsius */

#define INIT_UPDATE_MARKER	0x12345678
#define PAN_UPDATE_MARKER	0x12345679

#define POWER_STATE_OFF	0
#define POWER_STATE_ON	1

static unsigned long default_bpp = 16;

struct update_marker_data {
	u32 update_marker;
	struct completion update_completion;
	int lut_num;
};

/* This structure represents a list node containing both
 * a memory region allocated as an output buffer for the PxP
 * update processing task, and the update description (mode, region, etc.) */
struct update_data_list {
	struct list_head list;
	struct mxcfb_update_data upd_data;/* Update parameters */
	dma_addr_t phys_addr;		/* Pointer to phys address of processed Y buf */
	void *virt_addr;
	u32 epdc_offs;			/* Add to buffer pointer to resolve alignment */
	u32 size;
	int lut_num;			/* Assigned before update is processed into working buffer */
	int collision_mask;		/* Set when update results in collision */
					/* Represents other LUTs that we collide with */
	struct update_marker_data *upd_marker_data;
	u32 update_order;		/* Numeric ordering value for update */
	u32 fb_offset;			/* FB offset associated with update */
};

struct mxc_epdc_fb_data {
	struct fb_info info;
	struct fb_var_screeninfo epdc_fb_var; /* Internal copy of screeninfo
						so we can sync changes to it */
	u32 pseudo_palette[16];
	char fw_str[24];
	struct list_head list;
	struct mxc_epdc_fb_mode *cur_mode;
	struct mxc_epdc_fb_platform_data *pdata;
	int blank;
	ssize_t map_size;
	dma_addr_t phys_start;
	u32 fb_offset;
	int default_bpp;
	int native_width;
	int native_height;
	int num_screens;
	int epdc_irq;
	struct device *dev;
	int power_state;
	struct clk *epdc_clk_axi;
	struct clk *epdc_clk_pix;
	struct regulator *display_regulator;
	struct regulator *vcom_regulator;
	bool fw_default_load;

	/* FB elements related to EPDC updates */
	bool in_init;
	bool hw_ready;
	bool waiting_for_idle;
	u32 auto_mode;
	u32 upd_scheme;
	struct update_data_list *upd_buf_queue;
	struct update_data_list *upd_buf_free_list;
	struct update_data_list *upd_buf_collision_list;
	struct update_data_list *cur_update;
	spinlock_t queue_lock;
	int trt_entries;
	int temp_index;
	u8 *temp_range_bounds;
	struct mxcfb_waveform_modes wv_modes;
	u32 *waveform_buffer_virt;
	u32 waveform_buffer_phys;
	u32 waveform_buffer_size;
	u32 *working_buffer_virt;
	u32 working_buffer_phys;
	u32 working_buffer_size;
	u32 order_cnt;
	struct update_marker_data update_marker_array[EPDC_MAX_NUM_UPDATES];
	u32 lut_update_order[EPDC_NUM_LUTS];
	struct completion updates_done;
	struct delayed_work epdc_done_work;
	struct workqueue_struct *epdc_submit_workqueue;
	struct work_struct epdc_submit_work;
	bool waiting_for_wb;
	bool waiting_for_lut;
	struct completion update_res_free;
	struct mutex power_mutex;
	bool powering_down;
	int pwrdown_delay;

	/* FB elements related to PxP DMA */
	struct completion pxp_tx_cmpl;
	struct pxp_channel *pxp_chan;
	struct pxp_config_data pxp_conf;
	struct dma_async_tx_descriptor *txd;
	dma_cookie_t cookie;
	struct scatterlist sg[2];
	struct mutex pxp_mutex; /* protects access to PxP */
};

struct waveform_data_header {
	unsigned int wi0;
	unsigned int wi1;
	unsigned int wi2;
	unsigned int wi3;
	unsigned int wi4;
	unsigned int wi5;
	unsigned int wi6;
	unsigned int xwia:24;
	unsigned int cs1:8;
	unsigned int wmta:24;
	unsigned int fvsn:8;
	unsigned int luts:8;
	unsigned int mc:8;
	unsigned int trc:8;
	unsigned int reserved0_0:8;
	unsigned int eb:8;
	unsigned int sb:8;
	unsigned int reserved0_1:8;
	unsigned int reserved0_2:8;
	unsigned int reserved0_3:8;
	unsigned int reserved0_4:8;
	unsigned int reserved0_5:8;
	unsigned int cs2:8;
};

struct mxcfb_waveform_data_file {
	struct waveform_data_header wdh;
	u32 *data;	/* Temperature Range Table + Waveform Data */
};

void __iomem *epdc_base;

struct mxc_epdc_fb_data *g_fb_data;

/* forward declaration */
static int mxc_epdc_fb_get_temp_index(struct mxc_epdc_fb_data *fb_data,
						int temp);
static void mxc_epdc_fb_flush_updates(struct mxc_epdc_fb_data *fb_data);
static int mxc_epdc_fb_blank(int blank, struct fb_info *info);
static int mxc_epdc_fb_init_hw(struct fb_info *info);
static int pxp_process_update(struct mxc_epdc_fb_data *fb_data,
			      u32 src_width, u32 src_height,
			      struct mxcfb_rect *update_region,
			      int x_start_offs);
static int pxp_complete_update(struct mxc_epdc_fb_data *fb_data, u32 *hist_stat);

static void draw_mode0(struct mxc_epdc_fb_data *fb_data);
static bool is_free_list_full(struct mxc_epdc_fb_data *fb_data);


#ifdef DEBUG
static void dump_pxp_config(struct mxc_epdc_fb_data *fb_data,
			    struct pxp_config_data *pxp_conf)
{
	dev_err(fb_data->dev, "S0 fmt 0x%x",
		pxp_conf->s0_param.pixel_fmt);
	dev_err(fb_data->dev, "S0 width 0x%x",
		pxp_conf->s0_param.width);
	dev_err(fb_data->dev, "S0 height 0x%x",
		pxp_conf->s0_param.height);
	dev_err(fb_data->dev, "S0 ckey 0x%x",
		pxp_conf->s0_param.color_key);
	dev_err(fb_data->dev, "S0 ckey en 0x%x",
		pxp_conf->s0_param.color_key_enable);

	dev_err(fb_data->dev, "OL0 combine en 0x%x",
		pxp_conf->ol_param[0].combine_enable);
	dev_err(fb_data->dev, "OL0 fmt 0x%x",
		pxp_conf->ol_param[0].pixel_fmt);
	dev_err(fb_data->dev, "OL0 width 0x%x",
		pxp_conf->ol_param[0].width);
	dev_err(fb_data->dev, "OL0 height 0x%x",
		pxp_conf->ol_param[0].height);
	dev_err(fb_data->dev, "OL0 ckey 0x%x",
		pxp_conf->ol_param[0].color_key);
	dev_err(fb_data->dev, "OL0 ckey en 0x%x",
		pxp_conf->ol_param[0].color_key_enable);
	dev_err(fb_data->dev, "OL0 alpha 0x%x",
		pxp_conf->ol_param[0].global_alpha);
	dev_err(fb_data->dev, "OL0 alpha en 0x%x",
		pxp_conf->ol_param[0].global_alpha_enable);
	dev_err(fb_data->dev, "OL0 local alpha en 0x%x",
		pxp_conf->ol_param[0].local_alpha_enable);

	dev_err(fb_data->dev, "Out fmt 0x%x",
		pxp_conf->out_param.pixel_fmt);
	dev_err(fb_data->dev, "Out width 0x%x",
		pxp_conf->out_param.width);
	dev_err(fb_data->dev, "Out height 0x%x",
		pxp_conf->out_param.height);

	dev_err(fb_data->dev,
		"drect left 0x%x right 0x%x width 0x%x height 0x%x",
		pxp_conf->proc_data.drect.left, pxp_conf->proc_data.drect.top,
		pxp_conf->proc_data.drect.width,
		pxp_conf->proc_data.drect.height);
	dev_err(fb_data->dev,
		"srect left 0x%x right 0x%x width 0x%x height 0x%x",
		pxp_conf->proc_data.srect.left, pxp_conf->proc_data.srect.top,
		pxp_conf->proc_data.srect.width,
		pxp_conf->proc_data.srect.height);
	dev_err(fb_data->dev, "Scaling en 0x%x", pxp_conf->proc_data.scaling);
	dev_err(fb_data->dev, "HFlip en 0x%x", pxp_conf->proc_data.hflip);
	dev_err(fb_data->dev, "VFlip en 0x%x", pxp_conf->proc_data.vflip);
	dev_err(fb_data->dev, "Rotation 0x%x", pxp_conf->proc_data.rotate);
	dev_err(fb_data->dev, "BG Color 0x%x", pxp_conf->proc_data.bgcolor);
}

static void dump_epdc_reg(void)
{
	printk(KERN_DEBUG "\n\n");
	printk(KERN_DEBUG "EPDC_CTRL 0x%x\n", __raw_readl(EPDC_CTRL));
	printk(KERN_DEBUG "EPDC_WVADDR 0x%x\n", __raw_readl(EPDC_WVADDR));
	printk(KERN_DEBUG "EPDC_WB_ADDR 0x%x\n", __raw_readl(EPDC_WB_ADDR));
	printk(KERN_DEBUG "EPDC_RES 0x%x\n", __raw_readl(EPDC_RES));
	printk(KERN_DEBUG "EPDC_FORMAT 0x%x\n", __raw_readl(EPDC_FORMAT));
	printk(KERN_DEBUG "EPDC_FIFOCTRL 0x%x\n", __raw_readl(EPDC_FIFOCTRL));
	printk(KERN_DEBUG "EPDC_UPD_ADDR 0x%x\n", __raw_readl(EPDC_UPD_ADDR));
	printk(KERN_DEBUG "EPDC_UPD_FIXED 0x%x\n", __raw_readl(EPDC_UPD_FIXED));
	printk(KERN_DEBUG "EPDC_UPD_CORD 0x%x\n", __raw_readl(EPDC_UPD_CORD));
	printk(KERN_DEBUG "EPDC_UPD_SIZE 0x%x\n", __raw_readl(EPDC_UPD_SIZE));
	printk(KERN_DEBUG "EPDC_UPD_CTRL 0x%x\n", __raw_readl(EPDC_UPD_CTRL));
	printk(KERN_DEBUG "EPDC_TEMP 0x%x\n", __raw_readl(EPDC_TEMP));
	printk(KERN_DEBUG "EPDC_TCE_CTRL 0x%x\n", __raw_readl(EPDC_TCE_CTRL));
	printk(KERN_DEBUG "EPDC_TCE_SDCFG 0x%x\n", __raw_readl(EPDC_TCE_SDCFG));
	printk(KERN_DEBUG "EPDC_TCE_GDCFG 0x%x\n", __raw_readl(EPDC_TCE_GDCFG));
	printk(KERN_DEBUG "EPDC_TCE_HSCAN1 0x%x\n", __raw_readl(EPDC_TCE_HSCAN1));
	printk(KERN_DEBUG "EPDC_TCE_HSCAN2 0x%x\n", __raw_readl(EPDC_TCE_HSCAN2));
	printk(KERN_DEBUG "EPDC_TCE_VSCAN 0x%x\n", __raw_readl(EPDC_TCE_VSCAN));
	printk(KERN_DEBUG "EPDC_TCE_OE 0x%x\n", __raw_readl(EPDC_TCE_OE));
	printk(KERN_DEBUG "EPDC_TCE_POLARITY 0x%x\n", __raw_readl(EPDC_TCE_POLARITY));
	printk(KERN_DEBUG "EPDC_TCE_TIMING1 0x%x\n", __raw_readl(EPDC_TCE_TIMING1));
	printk(KERN_DEBUG "EPDC_TCE_TIMING2 0x%x\n", __raw_readl(EPDC_TCE_TIMING2));
	printk(KERN_DEBUG "EPDC_TCE_TIMING3 0x%x\n", __raw_readl(EPDC_TCE_TIMING3));
	printk(KERN_DEBUG "EPDC_IRQ_MASK 0x%x\n", __raw_readl(EPDC_IRQ_MASK));
	printk(KERN_DEBUG "EPDC_IRQ 0x%x\n", __raw_readl(EPDC_IRQ));
	printk(KERN_DEBUG "EPDC_STATUS_LUTS 0x%x\n", __raw_readl(EPDC_STATUS_LUTS));
	printk(KERN_DEBUG "EPDC_STATUS_NEXTLUT 0x%x\n", __raw_readl(EPDC_STATUS_NEXTLUT));
	printk(KERN_DEBUG "EPDC_STATUS_COL 0x%x\n", __raw_readl(EPDC_STATUS_COL));
	printk(KERN_DEBUG "EPDC_STATUS 0x%x\n", __raw_readl(EPDC_STATUS));
	printk(KERN_DEBUG "EPDC_DEBUG 0x%x\n", __raw_readl(EPDC_DEBUG));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT0 0x%x\n", __raw_readl(EPDC_DEBUG_LUT0));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT1 0x%x\n", __raw_readl(EPDC_DEBUG_LUT1));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT2 0x%x\n", __raw_readl(EPDC_DEBUG_LUT2));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT3 0x%x\n", __raw_readl(EPDC_DEBUG_LUT3));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT4 0x%x\n", __raw_readl(EPDC_DEBUG_LUT4));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT5 0x%x\n", __raw_readl(EPDC_DEBUG_LUT5));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT6 0x%x\n", __raw_readl(EPDC_DEBUG_LUT6));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT7 0x%x\n", __raw_readl(EPDC_DEBUG_LUT7));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT8 0x%x\n", __raw_readl(EPDC_DEBUG_LUT8));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT9 0x%x\n", __raw_readl(EPDC_DEBUG_LUT9));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT10 0x%x\n", __raw_readl(EPDC_DEBUG_LUT10));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT11 0x%x\n", __raw_readl(EPDC_DEBUG_LUT11));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT12 0x%x\n", __raw_readl(EPDC_DEBUG_LUT12));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT13 0x%x\n", __raw_readl(EPDC_DEBUG_LUT13));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT14 0x%x\n", __raw_readl(EPDC_DEBUG_LUT14));
	printk(KERN_DEBUG "EPDC_DEBUG_LUT15 0x%x\n", __raw_readl(EPDC_DEBUG_LUT15));
	printk(KERN_DEBUG "EPDC_GPIO 0x%x\n", __raw_readl(EPDC_GPIO));
	printk(KERN_DEBUG "EPDC_VERSION 0x%x\n", __raw_readl(EPDC_VERSION));
	printk(KERN_DEBUG "\n\n");
}

static void dump_update_data(struct device *dev,
			     struct update_data_list *upd_data_list)
{
	dev_err(dev,
		"X = %d, Y = %d, Width = %d, Height = %d, WaveMode = %d, LUT = %d, Coll Mask = %d\n",
		upd_data_list->upd_data.update_region.left,
		upd_data_list->upd_data.update_region.top,
		upd_data_list->upd_data.update_region.width,
		upd_data_list->upd_data.update_region.height,
		upd_data_list->upd_data.waveform_mode, upd_data_list->lut_num,
		upd_data_list->collision_mask);
}

static void dump_collision_list(struct mxc_epdc_fb_data *fb_data)
{
	struct update_data_list *plist;

	dev_err(fb_data->dev, "Collision List:\n");
	if (list_empty(&fb_data->upd_buf_collision_list->list))
		dev_err(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_collision_list->list, list) {
		dev_err(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
		dump_update_data(fb_data->dev, plist);
	}
}

static void dump_free_list(struct mxc_epdc_fb_data *fb_data)
{
	struct update_data_list *plist;

	dev_err(fb_data->dev, "Free List:\n");
	if (list_empty(&fb_data->upd_buf_free_list->list))
		dev_err(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_free_list->list, list) {
		dev_err(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
		dump_update_data(fb_data->dev, plist);
	}
}

static void dump_queue(struct mxc_epdc_fb_data *fb_data)
{
	struct update_data_list *plist;

	dev_err(fb_data->dev, "Queue:\n");
	if (list_empty(&fb_data->upd_buf_queue->list))
		dev_err(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_queue->list, list) {
		dev_err(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
		dump_update_data(fb_data->dev, plist);
	}
}

static void dump_all_updates(struct mxc_epdc_fb_data *fb_data)
{
	dump_free_list(fb_data);
	dump_queue(fb_data);
	dump_collision_list(fb_data);
	dev_err(fb_data->dev, "Current update being processed:\n");
	if (fb_data->cur_update == NULL)
		dev_err(fb_data->dev, "No current update\n");
	else
		dump_update_data(fb_data->dev, fb_data->cur_update);
}
#else
static inline void dump_pxp_config(struct mxc_epdc_fb_data *fb_data,
				   struct pxp_config_data *pxp_conf) {}
static inline void dump_epdc_reg(void) {}
static inline void dump_update_data(struct device *dev,
			     struct update_data_list *upd_data_list) {}
static inline void dump_collision_list(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_free_list(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_queue(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_all_updates(struct mxc_epdc_fb_data *fb_data) {}

#endif


/********************************************************
 * Start Low-Level EPDC Functions
 ********************************************************/

static inline void epdc_lut_complete_intr(u32 lut_num, bool enable)
{
	if (enable)
		__raw_writel(1 << lut_num, EPDC_IRQ_MASK_SET);
	else
		__raw_writel(1 << lut_num, EPDC_IRQ_MASK_CLEAR);
}

static inline void epdc_working_buf_intr(bool enable)
{
	if (enable)
		__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ, EPDC_IRQ_MASK_SET);
	else
		__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ, EPDC_IRQ_MASK_CLEAR);
}

static inline void epdc_clear_working_buf_irq(void)
{
	__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ | EPDC_IRQ_LUT_COL_IRQ,
		     EPDC_IRQ_CLEAR);
}

static inline void epdc_set_temp(u32 temp)
{
	__raw_writel(temp, EPDC_TEMP);
}

static inline void epdc_set_screen_res(u32 width, u32 height)
{
	u32 val = (height << EPDC_RES_VERTICAL_OFFSET) | width;
	__raw_writel(val, EPDC_RES);
}

static inline void epdc_set_update_addr(u32 addr)
{
	__raw_writel(addr, EPDC_UPD_ADDR);
}

static inline void epdc_set_update_coord(u32 x, u32 y)
{
	u32 val = (y << EPDC_UPD_CORD_YCORD_OFFSET) | x;
	__raw_writel(val, EPDC_UPD_CORD);
}

static inline void epdc_set_update_dimensions(u32 width, u32 height)
{
	u32 val = (height << EPDC_UPD_SIZE_HEIGHT_OFFSET) | width;
	__raw_writel(val, EPDC_UPD_SIZE);
}

static void epdc_submit_update(u32 lut_num, u32 waveform_mode, u32 update_mode,
			       bool use_test_mode, u32 np_val)
{
	u32 reg_val = 0;

	if (use_test_mode) {
		reg_val |=
		    ((np_val << EPDC_UPD_FIXED_FIXNP_OFFSET) &
		     EPDC_UPD_FIXED_FIXNP_MASK) | EPDC_UPD_FIXED_FIXNP_EN;

		__raw_writel(reg_val, EPDC_UPD_FIXED);

		reg_val = EPDC_UPD_CTRL_USE_FIXED;
	} else {
		__raw_writel(reg_val, EPDC_UPD_FIXED);
	}

	reg_val |=
	    ((lut_num << EPDC_UPD_CTRL_LUT_SEL_OFFSET) &
	     EPDC_UPD_CTRL_LUT_SEL_MASK) |
	    ((waveform_mode << EPDC_UPD_CTRL_WAVEFORM_MODE_OFFSET) &
	     EPDC_UPD_CTRL_WAVEFORM_MODE_MASK) |
	    update_mode;

	__raw_writel(reg_val, EPDC_UPD_CTRL);
}

static inline bool epdc_is_lut_complete(u32 lut_num)
{
	u32 val = __raw_readl(EPDC_IRQ);
	bool is_compl = val & (1 << lut_num) ? true : false;

	return is_compl;
}

static inline void epdc_clear_lut_complete_irq(u32 lut_num)
{
	__raw_writel(1 << lut_num, EPDC_IRQ_CLEAR);
}

static inline bool epdc_is_lut_active(u32 lut_num)
{
	u32 val = __raw_readl(EPDC_STATUS_LUTS);
	bool is_active = val & (1 << lut_num) ? true : false;

	return is_active;
}

static inline bool epdc_any_luts_active(void)
{
	bool any_active = __raw_readl(EPDC_STATUS_LUTS) ? true : false;

	return any_active;
}

static inline bool epdc_any_luts_available(void)
{
	bool luts_available =
	    (__raw_readl(EPDC_STATUS_NEXTLUT) &
	     EPDC_STATUS_NEXTLUT_NEXT_LUT_VALID) ? true : false;
	return luts_available;
}

static inline int epdc_get_next_lut(void)
{
	u32 val =
	    __raw_readl(EPDC_STATUS_NEXTLUT) &
	    EPDC_STATUS_NEXTLUT_NEXT_LUT_MASK;
	return val;
}

static inline bool epdc_is_working_buffer_busy(void)
{
	u32 val = __raw_readl(EPDC_STATUS);
	bool is_busy = (val & EPDC_STATUS_WB_BUSY) ? true : false;

	return is_busy;
}

static inline bool epdc_is_working_buffer_complete(void)
{
	u32 val = __raw_readl(EPDC_IRQ);
	bool is_compl = (val & EPDC_IRQ_WB_CMPLT_IRQ) ? true : false;

	return is_compl;
}

static inline bool epdc_is_collision(void)
{
	u32 val = __raw_readl(EPDC_IRQ);
	return (val & EPDC_IRQ_LUT_COL_IRQ) ? true : false;
}

static inline int epdc_get_colliding_luts(void)
{
	u32 val = __raw_readl(EPDC_STATUS_COL);
	return val;
}

static void epdc_set_horizontal_timing(u32 horiz_start, u32 horiz_end,
				       u32 hsync_width, u32 hsync_line_length)
{
	u32 reg_val =
	    ((hsync_width << EPDC_TCE_HSCAN1_LINE_SYNC_WIDTH_OFFSET) &
	     EPDC_TCE_HSCAN1_LINE_SYNC_WIDTH_MASK)
	    | ((hsync_line_length << EPDC_TCE_HSCAN1_LINE_SYNC_OFFSET) &
	       EPDC_TCE_HSCAN1_LINE_SYNC_MASK);
	__raw_writel(reg_val, EPDC_TCE_HSCAN1);

	reg_val =
	    ((horiz_start << EPDC_TCE_HSCAN2_LINE_BEGIN_OFFSET) &
	     EPDC_TCE_HSCAN2_LINE_BEGIN_MASK)
	    | ((horiz_end << EPDC_TCE_HSCAN2_LINE_END_OFFSET) &
	       EPDC_TCE_HSCAN2_LINE_END_MASK);
	__raw_writel(reg_val, EPDC_TCE_HSCAN2);
}

static void epdc_set_vertical_timing(u32 vert_start, u32 vert_end,
				     u32 vsync_width)
{
	u32 reg_val =
	    ((vert_start << EPDC_TCE_VSCAN_FRAME_BEGIN_OFFSET) &
	     EPDC_TCE_VSCAN_FRAME_BEGIN_MASK)
	    | ((vert_end << EPDC_TCE_VSCAN_FRAME_END_OFFSET) &
	       EPDC_TCE_VSCAN_FRAME_END_MASK)
	    | ((vsync_width << EPDC_TCE_VSCAN_FRAME_SYNC_OFFSET) &
	       EPDC_TCE_VSCAN_FRAME_SYNC_MASK);
	__raw_writel(reg_val, EPDC_TCE_VSCAN);
}

void epdc_init_settings(struct mxc_epdc_fb_data *fb_data)
{
	struct mxc_epdc_fb_mode *epdc_mode = fb_data->cur_mode;
	struct fb_var_screeninfo *screeninfo = &fb_data->epdc_fb_var;
	u32 reg_val;
	int num_ce;

	/* Reset */
	__raw_writel(EPDC_CTRL_SFTRST, EPDC_CTRL_SET);
	while (!(__raw_readl(EPDC_CTRL) & EPDC_CTRL_CLKGATE))
		;
	__raw_writel(EPDC_CTRL_SFTRST, EPDC_CTRL_CLEAR);

	/* Enable clock gating (clear to enable) */
	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_CLEAR);
	while (__raw_readl(EPDC_CTRL) & (EPDC_CTRL_SFTRST | EPDC_CTRL_CLKGATE))
		;

	/* EPDC_CTRL */
	reg_val = __raw_readl(EPDC_CTRL);
	reg_val &= ~EPDC_CTRL_UPD_DATA_SWIZZLE_MASK;
	reg_val |= EPDC_CTRL_UPD_DATA_SWIZZLE_NO_SWAP;
	reg_val &= ~EPDC_CTRL_LUT_DATA_SWIZZLE_MASK;
	reg_val |= EPDC_CTRL_LUT_DATA_SWIZZLE_NO_SWAP;
	__raw_writel(reg_val, EPDC_CTRL_SET);

	/* EPDC_FORMAT - 2bit TFT and 4bit Buf pixel format */
	reg_val = EPDC_FORMAT_TFT_PIXEL_FORMAT_2BIT
	    | EPDC_FORMAT_BUF_PIXEL_FORMAT_P4N
	    | ((0x0 << EPDC_FORMAT_DEFAULT_TFT_PIXEL_OFFSET) &
	       EPDC_FORMAT_DEFAULT_TFT_PIXEL_MASK);
	__raw_writel(reg_val, EPDC_FORMAT);

	/* EPDC_FIFOCTRL (disabled) */
	reg_val =
	    ((100 << EPDC_FIFOCTRL_FIFO_INIT_LEVEL_OFFSET) &
	     EPDC_FIFOCTRL_FIFO_INIT_LEVEL_MASK)
	    | ((200 << EPDC_FIFOCTRL_FIFO_H_LEVEL_OFFSET) &
	       EPDC_FIFOCTRL_FIFO_H_LEVEL_MASK)
	    | ((100 << EPDC_FIFOCTRL_FIFO_L_LEVEL_OFFSET) &
	       EPDC_FIFOCTRL_FIFO_L_LEVEL_MASK);
	__raw_writel(reg_val, EPDC_FIFOCTRL);

	/* EPDC_TEMP - Use default temp to get index */
	epdc_set_temp(mxc_epdc_fb_get_temp_index(fb_data, DEFAULT_TEMP));

	/* EPDC_RES */
	epdc_set_screen_res(epdc_mode->vmode->xres, epdc_mode->vmode->yres);

	/*
	 * EPDC_TCE_CTRL
	 * VSCAN_HOLDOFF = 4
	 * VCOM_MODE = MANUAL
	 * VCOM_VAL = 0
	 * DDR_MODE = DISABLED
	 * LVDS_MODE_CE = DISABLED
	 * LVDS_MODE = DISABLED
	 * DUAL_SCAN = DISABLED
	 * SDDO_WIDTH = 8bit
	 * PIXELS_PER_SDCLK = 4
	 */
	reg_val =
	    ((epdc_mode->vscan_holdoff << EPDC_TCE_CTRL_VSCAN_HOLDOFF_OFFSET) &
	     EPDC_TCE_CTRL_VSCAN_HOLDOFF_MASK)
	    | EPDC_TCE_CTRL_PIXELS_PER_SDCLK_4;
	__raw_writel(reg_val, EPDC_TCE_CTRL);

	/* EPDC_TCE_HSCAN */
	epdc_set_horizontal_timing(screeninfo->left_margin,
				   screeninfo->right_margin,
				   screeninfo->hsync_len,
				   screeninfo->hsync_len);

	/* EPDC_TCE_VSCAN */
	epdc_set_vertical_timing(screeninfo->upper_margin,
				 screeninfo->lower_margin,
				 screeninfo->vsync_len);

	/* EPDC_TCE_OE */
	reg_val =
	    ((epdc_mode->sdoed_width << EPDC_TCE_OE_SDOED_WIDTH_OFFSET) &
	     EPDC_TCE_OE_SDOED_WIDTH_MASK)
	    | ((epdc_mode->sdoed_delay << EPDC_TCE_OE_SDOED_DLY_OFFSET) &
	       EPDC_TCE_OE_SDOED_DLY_MASK)
	    | ((epdc_mode->sdoez_width << EPDC_TCE_OE_SDOEZ_WIDTH_OFFSET) &
	       EPDC_TCE_OE_SDOEZ_WIDTH_MASK)
	    | ((epdc_mode->sdoez_delay << EPDC_TCE_OE_SDOEZ_DLY_OFFSET) &
	       EPDC_TCE_OE_SDOEZ_DLY_MASK);
	__raw_writel(reg_val, EPDC_TCE_OE);

	/* EPDC_TCE_TIMING1 */
	__raw_writel(0x0, EPDC_TCE_TIMING1);

	/* EPDC_TCE_TIMING2 */
	reg_val =
	    ((epdc_mode->gdclk_hp_offs << EPDC_TCE_TIMING2_GDCLK_HP_OFFSET) &
	     EPDC_TCE_TIMING2_GDCLK_HP_MASK)
	    | ((epdc_mode->gdsp_offs << EPDC_TCE_TIMING2_GDSP_OFFSET_OFFSET) &
	       EPDC_TCE_TIMING2_GDSP_OFFSET_MASK);
	__raw_writel(reg_val, EPDC_TCE_TIMING2);

	/* EPDC_TCE_TIMING3 */
	reg_val =
	    ((epdc_mode->gdoe_offs << EPDC_TCE_TIMING3_GDOE_OFFSET_OFFSET) &
	     EPDC_TCE_TIMING3_GDOE_OFFSET_MASK)
	    | ((epdc_mode->gdclk_offs << EPDC_TCE_TIMING3_GDCLK_OFFSET_OFFSET) &
	       EPDC_TCE_TIMING3_GDCLK_OFFSET_MASK);
	__raw_writel(reg_val, EPDC_TCE_TIMING3);

	/*
	 * EPDC_TCE_SDCFG
	 * SDCLK_HOLD = 1
	 * SDSHR = 1
	 * NUM_CE = 1
	 * SDDO_REFORMAT = FLIP_PIXELS
	 * SDDO_INVERT = DISABLED
	 * PIXELS_PER_CE = display horizontal resolution
	 */
	num_ce = epdc_mode->num_ce;
	if (num_ce == 0)
		num_ce = 1;
	reg_val = EPDC_TCE_SDCFG_SDCLK_HOLD | EPDC_TCE_SDCFG_SDSHR
	    | ((num_ce << EPDC_TCE_SDCFG_NUM_CE_OFFSET) &
	       EPDC_TCE_SDCFG_NUM_CE_MASK)
	    | EPDC_TCE_SDCFG_SDDO_REFORMAT_FLIP_PIXELS
	    | ((epdc_mode->vmode->xres/num_ce << EPDC_TCE_SDCFG_PIXELS_PER_CE_OFFSET) &
	       EPDC_TCE_SDCFG_PIXELS_PER_CE_MASK);
	__raw_writel(reg_val, EPDC_TCE_SDCFG);

	/*
	 * EPDC_TCE_GDCFG
	 * GDRL = 1
	 * GDOE_MODE = 0;
	 * GDSP_MODE = 0;
	 */
	reg_val = EPDC_TCE_SDCFG_GDRL;
	__raw_writel(reg_val, EPDC_TCE_GDCFG);

	/*
	 * EPDC_TCE_POLARITY
	 * SDCE_POL = ACTIVE LOW
	 * SDLE_POL = ACTIVE HIGH
	 * SDOE_POL = ACTIVE HIGH
	 * GDOE_POL = ACTIVE HIGH
	 * GDSP_POL = ACTIVE LOW
	 */
	reg_val = EPDC_TCE_POLARITY_SDLE_POL_ACTIVE_HIGH
	    | EPDC_TCE_POLARITY_SDOE_POL_ACTIVE_HIGH
	    | EPDC_TCE_POLARITY_GDOE_POL_ACTIVE_HIGH;
	__raw_writel(reg_val, EPDC_TCE_POLARITY);

	/* EPDC_IRQ_MASK */
	__raw_writel(EPDC_IRQ_TCE_UNDERRUN_IRQ, EPDC_IRQ_MASK);

	/*
	 * EPDC_GPIO
	 * PWRCOM = ?
	 * PWRCTRL = ?
	 * BDR = ?
	 */
	reg_val = ((0 << EPDC_GPIO_PWRCTRL_OFFSET) & EPDC_GPIO_PWRCTRL_MASK)
	    | ((0 << EPDC_GPIO_BDR_OFFSET) & EPDC_GPIO_BDR_MASK);
	__raw_writel(reg_val, EPDC_GPIO);
}

static void epdc_powerup(struct mxc_epdc_fb_data *fb_data)
{
	int ret = 0;
	mutex_lock(&fb_data->power_mutex);

	/*
	 * If power down request is pending, clear
	 * powering_down to cancel the request.
	 */
	if (fb_data->powering_down)
		fb_data->powering_down = false;

	if (fb_data->power_state == POWER_STATE_ON) {
		mutex_unlock(&fb_data->power_mutex);
		return;
	}

	dev_dbg(fb_data->dev, "EPDC Powerup\n");

	/* Enable pins used by EPDC */
	if (fb_data->pdata->enable_pins)
		fb_data->pdata->enable_pins();

	/* Enable clocks to EPDC */
	clk_enable(fb_data->epdc_clk_axi);
	clk_enable(fb_data->epdc_clk_pix);

	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_CLEAR);

	/* Enable power to the EPD panel */
	ret = regulator_enable(fb_data->display_regulator);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable DISPLAY regulator."
			"err = 0x%x\n", ret);
		mutex_unlock(&fb_data->power_mutex);
		return;
	}
	ret = regulator_enable(fb_data->vcom_regulator);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable VCOM regulator."
			"err = 0x%x\n", ret);
		mutex_unlock(&fb_data->power_mutex);
		return;
	}

	fb_data->power_state = POWER_STATE_ON;

	mutex_unlock(&fb_data->power_mutex);
}

static void epdc_powerdown(struct mxc_epdc_fb_data *fb_data)
{
	mutex_lock(&fb_data->power_mutex);

	/* If powering_down has been cleared, a powerup
	 * request is pre-empting this powerdown request.
	 */
	if (!fb_data->powering_down
		|| (fb_data->power_state == POWER_STATE_OFF)) {
		mutex_unlock(&fb_data->power_mutex);
		return;
	}

	dev_dbg(fb_data->dev, "EPDC Powerdown\n");

	/* Disable power to the EPD panel */
	regulator_disable(fb_data->vcom_regulator);
	regulator_disable(fb_data->display_regulator);

	/* Disable clocks to EPDC */
	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_SET);
	clk_disable(fb_data->epdc_clk_pix);
	clk_disable(fb_data->epdc_clk_axi);

	/* Disable pins used by EPDC (to prevent leakage current) */
	if (fb_data->pdata->disable_pins)
		fb_data->pdata->disable_pins();

	fb_data->power_state = POWER_STATE_OFF;
	fb_data->powering_down = false;

	mutex_unlock(&fb_data->power_mutex);
}

static void epdc_init_sequence(struct mxc_epdc_fb_data *fb_data)
{
	/* Initialize EPDC, passing pointer to EPDC registers */
	epdc_init_settings(fb_data);
	__raw_writel(fb_data->waveform_buffer_phys, EPDC_WVADDR);
	__raw_writel(fb_data->working_buffer_phys, EPDC_WB_ADDR);
	epdc_powerup(fb_data);
	draw_mode0(fb_data);
	epdc_powerdown(fb_data);
}

static int mxc_epdc_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset < info->fix.smem_len) {
		/* mapping framebuffer memory */
		len = info->fix.smem_len - offset;
		vma->vm_pgoff = (info->fix.smem_start + offset) >> PAGE_SHIFT;
	} else
		return -EINVAL;

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_flags |= VM_IO | VM_RESERVED;

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(info->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

static int mxc_epdc_fb_setcolreg(u_int regno, u_int red, u_int green,
				 u_int blue, u_int transp, struct fb_info *info)
{
	if (regno >= 256)	/* no. of hw registers */
		return 1;
	/*
	 * Program hardware... do anything you want with transp
	 */

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

#define CNVT_TOHW(val, width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		red = CNVT_TOHW(red, 8);	/* expect 8 bit DAC */
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {

		if (regno >= 16)
			return 1;

		((u32 *) (info->pseudo_palette))[regno] =
		    (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset) |
		    (transp << info->var.transp.offset);
	}
	return 0;
}

static void adjust_coordinates(struct mxc_epdc_fb_data *fb_data,
	struct mxcfb_rect *update_region, struct mxcfb_rect *adj_update_region)
{
	struct fb_var_screeninfo *screeninfo = &fb_data->epdc_fb_var;
	u32 rotation = fb_data->epdc_fb_var.rotate;
	u32 temp;

	/* If adj_update_region == NULL, pass result back in update_region */
	/* If adj_update_region == valid, use it to pass back result */
	if (adj_update_region)
		switch (rotation) {
		case FB_ROTATE_UR:
			adj_update_region->top = update_region->top;
			adj_update_region->left = update_region->left;
			adj_update_region->width = update_region->width;
			adj_update_region->height = update_region->height;
			break;
		case FB_ROTATE_CW:
			adj_update_region->top = update_region->left;
			adj_update_region->left = screeninfo->yres -
				(update_region->top + update_region->height);
			adj_update_region->width = update_region->height;
			adj_update_region->height = update_region->width;
			break;
		case FB_ROTATE_UD:
			adj_update_region->width = update_region->width;
			adj_update_region->height = update_region->height;
			adj_update_region->top = screeninfo->yres -
				(update_region->top + update_region->height);
			adj_update_region->left = screeninfo->xres -
				(update_region->left + update_region->width);
			break;
		case FB_ROTATE_CCW:
			adj_update_region->left = update_region->top;
			adj_update_region->top = screeninfo->xres -
				(update_region->left + update_region->width);
			adj_update_region->width = update_region->height;
			adj_update_region->height = update_region->width;
			break;
		}
	else
		switch (rotation) {
		case FB_ROTATE_UR:
			/* No adjustment needed */
			break;
		case FB_ROTATE_CW:
			temp = update_region->top;
			update_region->top = update_region->left;
			update_region->left = screeninfo->yres -
				(temp + update_region->height);
			temp = update_region->width;
			update_region->width = update_region->height;
			update_region->height = temp;
			break;
		case FB_ROTATE_UD:
			update_region->top = screeninfo->yres -
				(update_region->top + update_region->height);
			update_region->left = screeninfo->xres -
				(update_region->left + update_region->width);
			break;
		case FB_ROTATE_CCW:
			temp = update_region->left;
			update_region->left = update_region->top;
			update_region->top = screeninfo->xres -
				(temp + update_region->width);
			temp = update_region->width;
			update_region->width = update_region->height;
			update_region->height = temp;
			break;
		}
}

/*
 * Set fixed framebuffer parameters based on variable settings.
 *
 * @param       info     framebuffer information pointer
 */
static int mxc_epdc_fb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	if (var->grayscale)
		fix->visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	else
		fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ypanstep = 1;

	return 0;
}

/*
 * This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the
 * change in par. For this driver it doesn't do much.
 *
 */
static int mxc_epdc_fb_set_par(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	struct pxp_config_data *pxp_conf = &fb_data->pxp_conf;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;
	struct fb_var_screeninfo *screeninfo = &fb_data->info.var;
	struct mxc_epdc_fb_mode *epdc_modes = fb_data->pdata->epdc_mode;
	int i;
	int ret;
	unsigned long flags;

	/*
	 * Can't change the FB parameters until current updates have completed.
	 * This function returns when all active updates are done.
	 */
	mxc_epdc_fb_flush_updates(fb_data);

	spin_lock_irqsave(&fb_data->queue_lock, flags);
	fb_data->epdc_fb_var = *screeninfo;
	spin_unlock_irqrestore(&fb_data->queue_lock, flags);

	mutex_lock(&fb_data->pxp_mutex);

	/*
	 * Update PxP config data (used to process FB regions for updates)
	 * based on FB info and processing tasks required
	 */

	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = proc_data->srect.width = screeninfo->xres;
	proc_data->drect.height = proc_data->srect.height = screeninfo->yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = screeninfo->rotate;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;

	/*
	 * configure S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	if (screeninfo->grayscale)
		pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_GREY;
	else {
		switch (screeninfo->bits_per_pixel) {
		case 16:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		case 24:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB24;
			break;
		case 32:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB32;
			break;
		default:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		}
	}
	pxp_conf->s0_param.width = screeninfo->xres_virtual;
	pxp_conf->s0_param.height = screeninfo->yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = screeninfo->xres;
	pxp_conf->out_param.height = screeninfo->yres;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GREY;

	mutex_unlock(&fb_data->pxp_mutex);

	/*
	 * If HW not yet initialized, check to see if we are being sent
	 * an initialization request.
	 */
	if (!fb_data->hw_ready) {
		struct fb_videomode mode;
		bool found_match = false;
		u32 xres_temp;

		fb_var_to_videomode(&mode, screeninfo);

		/* When comparing requested fb mode,
		   we need to use unrotated dimensions */
		if ((screeninfo->rotate == FB_ROTATE_CW) ||
			(screeninfo->rotate == FB_ROTATE_CCW)) {
			xres_temp = mode.xres;
			mode.xres = mode.yres;
			mode.yres = xres_temp;
		}

		/* Match videomode against epdc modes */
		for (i = 0; i < fb_data->pdata->num_modes; i++) {
			if (!fb_mode_is_equal(epdc_modes[i].vmode, &mode))
				continue;
			fb_data->cur_mode = &epdc_modes[i];
			found_match = true;
			break;
		}

		if (!found_match) {
			dev_err(fb_data->dev,
				"Failed to match requested video mode\n");
			return EINVAL;
		}

		/* Found a match - Grab timing params */
		screeninfo->left_margin = mode.left_margin;
		screeninfo->right_margin = mode.right_margin;
		screeninfo->upper_margin = mode.upper_margin;
		screeninfo->lower_margin = mode.lower_margin;
		screeninfo->hsync_len = mode.hsync_len;
		screeninfo->vsync_len = mode.vsync_len;

		/* Initialize EPDC settings and init panel */
		ret =
		    mxc_epdc_fb_init_hw((struct fb_info *)fb_data);
		if (ret) {
			dev_err(fb_data->dev,
				"Failed to load panel waveform data\n");
			return ret;
		}
	}

	mxc_epdc_fb_set_fix(info);

	return 0;
}

static int mxc_epdc_fb_check_var(struct fb_var_screeninfo *var,
				 struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
	    (var->bits_per_pixel != 16) && (var->bits_per_pixel != 8))
		var->bits_per_pixel = default_bpp;

	switch (var->bits_per_pixel) {
	case 8:
		if (var->grayscale != 0) {
			/*
			 * For 8-bit grayscale, R, G, and B offset are equal.
			 *
			 */
			var->red.length = 8;
			var->red.offset = 0;
			var->red.msb_right = 0;

			var->green.length = 8;
			var->green.offset = 0;
			var->green.msb_right = 0;

			var->blue.length = 8;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
		} else {
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
		}
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

	switch (var->rotate) {
	case FB_ROTATE_UR:
	case FB_ROTATE_UD:
		var->xres = fb_data->native_width;
		var->yres = fb_data->native_height;
		break;
	case FB_ROTATE_CW:
	case FB_ROTATE_CCW:
		var->xres = fb_data->native_height;
		var->yres = fb_data->native_width;
		break;
	default:
		/* Invalid rotation value */
		var->rotate = 0;
		dev_dbg(fb_data->dev, "Invalid rotation request\n");
		return -EINVAL;
	}

	var->xres_virtual = ALIGN(var->xres, 32);
	var->yres_virtual = ALIGN(var->yres, 128) * fb_data->num_screens;

	var->height = -1;
	var->width = -1;

	return 0;
}

void mxc_epdc_fb_set_waveform_modes(struct mxcfb_waveform_modes *modes,
	struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	memcpy(&fb_data->wv_modes, modes, sizeof(modes));
}
EXPORT_SYMBOL(mxc_epdc_fb_set_waveform_modes);

static int mxc_epdc_fb_get_temp_index(struct mxc_epdc_fb_data *fb_data, int temp)
{
	int i;
	int index = -1;

	if (fb_data->trt_entries == 0) {
		dev_err(fb_data->dev,
			"No TRT exists...using default temp index\n");
		return DEFAULT_TEMP_INDEX;
	}

	/* Search temperature ranges for a match */
	for (i = 0; i < fb_data->trt_entries - 1; i++) {
		if ((temp >= fb_data->temp_range_bounds[i])
			&& (temp < fb_data->temp_range_bounds[i+1])) {
			index = i;
			break;
		}
	}

	if (index < 0) {
		dev_err(fb_data->dev,
			"No TRT index match...using default temp index\n");
		return DEFAULT_TEMP_INDEX;
	}

	dev_dbg(fb_data->dev, "Using temperature index %d\n", index);

	return index;
}

int mxc_epdc_fb_set_temperature(int temperature, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	unsigned long flags;

	/* Store temp index. Used later when configuring updates. */
	spin_lock_irqsave(&fb_data->queue_lock, flags);
	fb_data->temp_index = mxc_epdc_fb_get_temp_index(fb_data, temperature);
	spin_unlock_irqrestore(&fb_data->queue_lock, flags);

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_temperature);

int mxc_epdc_fb_set_auto_update(u32 auto_mode, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	dev_dbg(fb_data->dev, "Setting auto update mode to %d\n", auto_mode);

	if ((auto_mode == AUTO_UPDATE_MODE_AUTOMATIC_MODE)
		|| (auto_mode == AUTO_UPDATE_MODE_REGION_MODE))
		fb_data->auto_mode = auto_mode;
	else {
		dev_err(fb_data->dev, "Invalid auto update mode parameter.\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_auto_update);

int mxc_epdc_fb_set_upd_scheme(u32 upd_scheme, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	dev_dbg(fb_data->dev, "Setting optimization level to %d\n", upd_scheme);

	/*
	 * Can't change the scheme until current updates have completed.
	 * This function returns when all active updates are done.
	 */
	mxc_epdc_fb_flush_updates(fb_data);

	if ((upd_scheme == UPDATE_SCHEME_SNAPSHOT)
		|| (upd_scheme == UPDATE_SCHEME_QUEUE)
		|| (upd_scheme == UPDATE_SCHEME_QUEUE_AND_MERGE))
		fb_data->upd_scheme = upd_scheme;
	else {
		dev_err(fb_data->dev, "Invalid update scheme specified.\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_upd_scheme);

static int epdc_process_update(struct update_data_list *upd_data_list,
				   struct mxc_epdc_fb_data *fb_data)
{
	struct mxcfb_rect *src_upd_region; /* Region of src buffer for update */
	struct mxcfb_rect pxp_upd_region;
	u32 src_width, src_height;
	u32 offset_from_8, bytes_per_pixel;
	u32 post_rotation_xcoord, post_rotation_ycoord, width_pxp_blocks;
	u32 pxp_input_offs, pxp_output_offs, pxp_output_shift;
	int x_start_offs = 0;
	u32 hist_stat = 0;

	int ret;

	/*
	 * Gotta do a whole bunch of buffer ptr manipulation to
	 * work around HW restrictions for PxP & EPDC
	 */

	/*
	 * Are we using FB or an alternate (overlay)
	 * buffer for source of update?
	 */
	if (upd_data_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER) {
		src_width = upd_data_list->upd_data.alt_buffer_data.width;
		src_height = upd_data_list->upd_data.alt_buffer_data.height;
		src_upd_region = &upd_data_list->upd_data.alt_buffer_data.alt_update_region;
	} else {
		src_width = fb_data->epdc_fb_var.xres_virtual;
		src_height = fb_data->epdc_fb_var.yres;
		src_upd_region = &upd_data_list->upd_data.update_region;
	}

	/*
	 * Compute buffer offset to account for
	 * PxP limitation (must read 8x8 pixel blocks)
	 */
	offset_from_8 = src_upd_region->left & 0x7;
	bytes_per_pixel = fb_data->epdc_fb_var.bits_per_pixel/8;
	if ((offset_from_8 * fb_data->epdc_fb_var.bits_per_pixel/8 % 4) != 0) {
		/* Leave a gap between PxP input addr and update region pixels */
		pxp_input_offs =
			(src_upd_region->top * src_width + src_upd_region->left)
			* bytes_per_pixel & 0xFFFFFFFC;
		/* Update region should change to reflect relative position to input ptr */
		pxp_upd_region.top = 0;
		pxp_upd_region.left = (offset_from_8 & 0x3) % bytes_per_pixel;
	} else {
		pxp_input_offs =
			(src_upd_region->top * src_width + src_upd_region->left)
			* bytes_per_pixel;
		/* Update region should change to reflect relative position to input ptr */
		pxp_upd_region.top = 0;
		pxp_upd_region.left = 0;
	}

	/*
	 * We want PxP processing region to start at the first pixel
	 * that we have to process in each row, but PxP alignment
	 * restricts the input mem address to be 32-bit aligned
	 *
	 * We work around this by using x_start_offs
	 * to offset from our starting pixel location to the
	 * first pixel that is in the relevant update region
	 * for each row.
	 */
	x_start_offs = pxp_upd_region.left & 0x7;

	/* Update region to meet 8x8 pixel requirement */
	pxp_upd_region.width = ALIGN(src_upd_region->width, 8);
	pxp_upd_region.height = ALIGN(src_upd_region->height, 8);
	pxp_upd_region.top &= ~0x7;
	pxp_upd_region.left &= ~0x7;

	switch (fb_data->epdc_fb_var.rotate) {
	case FB_ROTATE_UR:
	default:
		post_rotation_xcoord = pxp_upd_region.left;
		post_rotation_ycoord = pxp_upd_region.top;
		width_pxp_blocks = pxp_upd_region.width;
		break;
	case FB_ROTATE_CW:
		width_pxp_blocks = pxp_upd_region.height;
		post_rotation_xcoord = width_pxp_blocks - src_upd_region->height;
		post_rotation_ycoord = pxp_upd_region.left;
		break;
	case FB_ROTATE_UD:
		width_pxp_blocks = pxp_upd_region.width;
		post_rotation_xcoord = width_pxp_blocks - src_upd_region->width - pxp_upd_region.left;
		post_rotation_ycoord = pxp_upd_region.height - src_upd_region->height - pxp_upd_region.top;
		break;
	case FB_ROTATE_CCW:
		width_pxp_blocks = pxp_upd_region.height;
		post_rotation_xcoord = pxp_upd_region.top;
		post_rotation_ycoord = pxp_upd_region.width - src_upd_region->width - pxp_upd_region.left;
		break;
	}

	pxp_output_offs = post_rotation_ycoord * width_pxp_blocks
		+ post_rotation_xcoord;

	pxp_output_shift = ALIGN(pxp_output_offs, 8) - pxp_output_offs;

	upd_data_list->epdc_offs = pxp_output_offs + pxp_output_shift;

	mutex_lock(&fb_data->pxp_mutex);

	/* Source address either comes from alternate buffer
	   provided in update data, or from the framebuffer. */
	if (upd_data_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER)
		sg_dma_address(&fb_data->sg[0]) =
			upd_data_list->upd_data.alt_buffer_data.phys_addr
				+ pxp_input_offs;
	else {
		sg_dma_address(&fb_data->sg[0]) =
			fb_data->info.fix.smem_start + upd_data_list->fb_offset
			+ pxp_input_offs;
		sg_set_page(&fb_data->sg[0],
			virt_to_page(fb_data->info.screen_base),
			fb_data->info.fix.smem_len,
			offset_in_page(fb_data->info.screen_base));
	}

	/* Update sg[1] to point to output of PxP proc task */
	sg_dma_address(&fb_data->sg[1]) = upd_data_list->phys_addr + pxp_output_offs;
	sg_set_page(&fb_data->sg[1], virt_to_page(upd_data_list->virt_addr),
		    upd_data_list->size,
		    offset_in_page(upd_data_list->virt_addr));

	/*
	 * Set PxP LUT transform type based on update flags.
	 */
	fb_data->pxp_conf.proc_data.lut_transform = 0;
	if (upd_data_list->upd_data.flags & EPDC_FLAG_ENABLE_INVERSION)
		fb_data->pxp_conf.proc_data.lut_transform |= PXP_LUT_INVERT;
	if (upd_data_list->upd_data.flags & EPDC_FLAG_FORCE_MONOCHROME)
		fb_data->pxp_conf.proc_data.lut_transform |=
			PXP_LUT_BLACK_WHITE;

	/*
	 * Toggle inversion processing if 8-bit
	 * inverted is the current pixel format.
	 */
	if (fb_data->epdc_fb_var.grayscale == GRAYSCALE_8BIT_INVERTED)
		fb_data->pxp_conf.proc_data.lut_transform ^= PXP_LUT_INVERT;

	/* This is a blocking call, so upon return PxP tx should be done */
	ret = pxp_process_update(fb_data, src_width, src_height,
		&pxp_upd_region, x_start_offs);
	if (ret) {
		dev_err(fb_data->dev, "Unable to submit PxP update task.\n");
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}

	mutex_unlock(&fb_data->pxp_mutex);

	/* If needed, enable EPDC HW while ePxP is processing */
	if ((fb_data->power_state == POWER_STATE_OFF)
		|| fb_data->powering_down) {
		epdc_powerup(fb_data);
	}

	mutex_lock(&fb_data->pxp_mutex);

	/* This is a blocking call, so upon return PxP tx should be done */
	ret = pxp_complete_update(fb_data, &hist_stat);
	if (ret) {
		dev_err(fb_data->dev, "Unable to complete PxP update task.\n");
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}

	mutex_unlock(&fb_data->pxp_mutex);

	/* Update waveform mode from PxP histogram results */
	if (upd_data_list->upd_data.waveform_mode == WAVEFORM_MODE_AUTO) {
		if (hist_stat & 0x1)
			upd_data_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_du;
		else if (hist_stat & 0x2)
			upd_data_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc4;
		else if (hist_stat & 0x4)
			upd_data_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc8;
		else if (hist_stat & 0x8)
			upd_data_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc16;
		else
			upd_data_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc32;

		dev_dbg(fb_data->dev, "hist_stat = 0x%x, new waveform = 0x%x\n",
			hist_stat, upd_data_list->upd_data.waveform_mode);
	}

	return 0;

}

static bool epdc_submit_merge(struct update_data_list *upd_data_list,
				struct update_data_list *update_to_merge)
{
	struct mxcfb_update_data *a, *b;
	struct mxcfb_rect *arect, *brect;
	struct mxcfb_rect combine;

	a = &upd_data_list->upd_data;
	b = &update_to_merge->upd_data;
	arect = &upd_data_list->upd_data.update_region;
	brect = &update_to_merge->upd_data.update_region;

	if ((a->waveform_mode != b->waveform_mode
		&& a->waveform_mode != WAVEFORM_MODE_AUTO) ||
		a->update_mode != b->update_mode ||
		(a->flags & EPDC_FLAG_USE_ALT_BUFFER) ||
		(b->flags & EPDC_FLAG_USE_ALT_BUFFER) ||
		arect->left > (brect->left + brect->width) ||
		brect->left > (arect->left + arect->width) ||
		arect->top > (brect->top + brect->height) ||
		brect->top > (arect->top + arect->height) ||
		(upd_data_list->fb_offset != update_to_merge->fb_offset) ||
		(b->update_marker != 0 && a->update_marker != 0))
		return false;

	combine.left = arect->left < brect->left ? arect->left : brect->left;
	combine.top = arect->top < brect->top ? arect->top : brect->top;
	combine.width = (arect->left + arect->width) >
			(brect->left + brect->width) ?
			(arect->left + arect->width - combine.left) :
			(brect->left + brect->width - combine.left);
	combine.height = (arect->top + arect->height) >
			(brect->top + brect->height) ?
			(arect->top + arect->height - combine.top) :
			(brect->top + brect->height - combine.top);

	arect->left = combine.left;
	arect->top = combine.top;
	arect->width = combine.width;
	arect->height = combine.height;

	/* Preserve marker value for merged update */
	if (b->update_marker != 0) {
		a->update_marker = b->update_marker;
		upd_data_list->upd_marker_data =
			update_to_merge->upd_marker_data;
	}

	/* Merged update should take on the earliest order */
	upd_data_list->update_order =
		(upd_data_list->update_order > update_to_merge->update_order) ?
		upd_data_list->update_order : update_to_merge->update_order;

	return true;
}

static void epdc_submit_work_func(struct work_struct *work)
{
	int temp_index;
	struct update_data_list *next_update;
	struct update_data_list *temp;
	unsigned long flags;
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data, epdc_submit_work);
	struct update_data_list *upd_data_list = NULL;
	struct mxcfb_rect adj_update_region;

	/* Protect access to buffer queues and to update HW */
	spin_lock_irqsave(&fb_data->queue_lock, flags);

	/*
	 * Are any of our collision updates able to go now?
	 * Go through all updates in the collision list and check to see
	 * if the collision mask has been fully cleared
	 */
	list_for_each_entry_safe(next_update, temp,
				&fb_data->upd_buf_collision_list->list, list) {

		if (next_update->collision_mask != 0)
			continue;

		dev_dbg(fb_data->dev, "A collision update is ready to go!\n");
		/*
		 * We have a collision cleared, so select it for resubmission.
		 * If an update is already selected, attempt to merge.
		 */
		if (!upd_data_list) {
			upd_data_list = next_update;
			list_del_init(&next_update->list);
			if (fb_data->upd_scheme == UPDATE_SCHEME_QUEUE)
				/* If not merging, we have our update */
				break;
		} else if (epdc_submit_merge(upd_data_list, next_update)) {
			dev_dbg(fb_data->dev,
				"Update merged [work queue]\n");
			list_del_init(&next_update->list);
			/* Add to free buffer list */
			list_add_tail(&next_update->list,
				 &fb_data->upd_buf_free_list->list);
		} else
			dev_dbg(fb_data->dev,
				"Update not merged [work queue]\n");
	}

	/*
	 * Skip update queue only if we found a collision
	 * update and we are not merging
	 */
	if (!((fb_data->upd_scheme == UPDATE_SCHEME_QUEUE) &&
		upd_data_list)) {
		/*
		 * If we didn't find a collision update ready to go,
		 * we try to grab one from the update queue
		 */
		list_for_each_entry_safe(next_update, temp,
					&fb_data->upd_buf_queue->list, list) {

			dev_dbg(fb_data->dev, "Found a pending update!\n");

			if (!upd_data_list) {
				upd_data_list = next_update;
				list_del_init(&next_update->list);
				if (fb_data->upd_scheme == UPDATE_SCHEME_QUEUE)
					/* If not merging, we have an update */
					break;
			} else if (epdc_submit_merge(upd_data_list,
					next_update)) {
				dev_dbg(fb_data->dev,
					"Update merged [work queue]\n");
				list_del_init(&next_update->list);
				/* Add to free buffer list */
				list_add_tail(&next_update->list,
					 &fb_data->upd_buf_free_list->list);
			} else
				dev_dbg(fb_data->dev,
					"Update not merged [work queue]\n");
		}
	}

	/* Release buffer queues */
	spin_unlock_irqrestore(&fb_data->queue_lock, flags);

	/* Is update list empty? */
	if (!upd_data_list)
		return;

	/* Perform PXP processing - EPDC power will also be enabled */
	if (epdc_process_update(upd_data_list, fb_data)) {
		dev_dbg(fb_data->dev, "PXP processing error.\n");
		/* Protect access to buffer queues and to update HW */
		spin_lock_irqsave(&fb_data->queue_lock, flags);
		/* Add to free buffer list */
		list_add_tail(&upd_data_list->list,
			 &fb_data->upd_buf_free_list->list);
		/* Release buffer queues */
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		return;
	}

	/* Get rotation-adjusted coordinates */
	adjust_coordinates(fb_data, &upd_data_list->upd_data.update_region,
		&adj_update_region);

	/* Protect access to buffer queues and to update HW */
	spin_lock_irqsave(&fb_data->queue_lock, flags);

	/*
	 * Is the working buffer idle?
	 * If the working buffer is busy, we must wait for the resource
	 * to become free. The IST will signal this event.
	 */
	if (fb_data->cur_update != NULL) {
		dev_dbg(fb_data->dev, "working buf busy!\n");

		/* Initialize event signalling an update resource is free */
		init_completion(&fb_data->update_res_free);

		fb_data->waiting_for_wb = true;

		/* Leave spinlock while waiting for WB to complete */
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		wait_for_completion(&fb_data->update_res_free);
		spin_lock_irqsave(&fb_data->queue_lock, flags);
	}

	/*
	 * If there are no LUTs available,
	 * then we must wait for the resource to become free.
	 * The IST will signal this event.
	 */
	if (!epdc_any_luts_available()) {
		dev_dbg(fb_data->dev, "no luts available!\n");

		/* Initialize event signalling an update resource is free */
		init_completion(&fb_data->update_res_free);

		fb_data->waiting_for_lut = true;

		/* Leave spinlock while waiting for LUT to free up */
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		wait_for_completion(&fb_data->update_res_free);
		spin_lock_irqsave(&fb_data->queue_lock, flags);
	}


	/* LUTs are available, so we get one here */
	fb_data->cur_update = upd_data_list;
	fb_data->cur_update->lut_num = epdc_get_next_lut();

	/* Associate LUT with update marker */
	if ((fb_data->cur_update->upd_marker_data)
		&& (fb_data->cur_update->upd_marker_data->update_marker != 0))
		fb_data->cur_update->upd_marker_data->lut_num =
						fb_data->cur_update->lut_num;

	/* Mark LUT with order */
	fb_data->lut_update_order[fb_data->cur_update->lut_num] =
		fb_data->cur_update->update_order;

	/* Enable Collision and WB complete IRQs */
	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(fb_data->cur_update->lut_num, true);

	/* Program EPDC update to process buffer */
	if (fb_data->cur_update->upd_data.temp != TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
				fb_data->cur_update->upd_data.temp);
		epdc_set_temp(temp_index);
	}
	epdc_set_update_addr(fb_data->cur_update->phys_addr
				+ fb_data->cur_update->epdc_offs);
	epdc_set_update_coord(adj_update_region.left, adj_update_region.top);
	epdc_set_update_dimensions(adj_update_region.width,
				   adj_update_region.height);
	epdc_submit_update(fb_data->cur_update->lut_num,
			   fb_data->cur_update->upd_data.waveform_mode,
			   fb_data->cur_update->upd_data.update_mode, false, 0);

	/* Release buffer queues */
	spin_unlock_irqrestore(&fb_data->queue_lock, flags);
}


int mxc_epdc_fb_send_update(struct mxcfb_update_data *upd_data,
				   struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	struct update_data_list *upd_data_list = NULL;
	unsigned long flags;
	int i;
	struct mxcfb_rect *screen_upd_region; /* Region on screen to update */
	int temp_index;
	int ret;

	/* Has EPDC HW been initialized? */
	if (!fb_data->hw_ready) {
		dev_err(fb_data->dev, "Display HW not properly initialized."
			"  Aborting update.\n");
		return -EPERM;
	}

	/* Check validity of update params */
	if ((upd_data->update_mode != UPDATE_MODE_PARTIAL) &&
		(upd_data->update_mode != UPDATE_MODE_FULL)) {
		dev_err(fb_data->dev,
			"Update mode 0x%x is invalid.  Aborting update.\n",
			upd_data->update_mode);
		return -EINVAL;
	}
	if ((upd_data->waveform_mode > 255) &&
		(upd_data->waveform_mode != WAVEFORM_MODE_AUTO)) {
		dev_err(fb_data->dev,
			"Update waveform mode 0x%x is invalid."
			"  Aborting update.\n",
			upd_data->waveform_mode);
		return -EINVAL;
	}
	if ((upd_data->update_region.left + upd_data->update_region.width > fb_data->epdc_fb_var.xres) ||
		(upd_data->update_region.top + upd_data->update_region.height > fb_data->epdc_fb_var.yres)) {
		dev_err(fb_data->dev,
			"Update region is outside bounds of framebuffer."
			"Aborting update.\n");
		return -EINVAL;
	}
	if ((upd_data->flags & EPDC_FLAG_USE_ALT_BUFFER) &&
		((upd_data->update_region.width != upd_data->alt_buffer_data.alt_update_region.width) ||
		(upd_data->update_region.height != upd_data->alt_buffer_data.alt_update_region.height))) {
		dev_err(fb_data->dev,
			"Alternate update region dimensions must match screen update region dimensions.\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&fb_data->queue_lock, flags);

	/*
	 * If we are waiting to go into suspend, or the FB is blanked,
	 * we do not accept new updates
	 */
	if ((fb_data->waiting_for_idle) ||
		(fb_data->blank != FB_BLANK_UNBLANK)) {
		dev_dbg(fb_data->dev, "EPDC not active."
			"Update request abort.\n");
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		return -EPERM;
	}

	/*
	 * Get available intermediate (PxP output) buffer to hold
	 * processed update region
	 */
	if (list_empty(&fb_data->upd_buf_free_list->list)) {
		dev_err(fb_data->dev,
			"No free intermediate buffers available.\n");
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		return -ENOMEM;
	}

	/* Grab first available buffer and delete it from the free list */
	upd_data_list =
	    list_entry(fb_data->upd_buf_free_list->list.next,
		       struct update_data_list, list);

	list_del_init(&upd_data_list->list);

	/* copy update parameters to the current update data object */
	memcpy(&upd_data_list->upd_data, upd_data,
	       sizeof(struct mxcfb_update_data));
	memcpy(&upd_data_list->upd_data.update_region, &upd_data->update_region,
	       sizeof(struct mxcfb_rect));

	upd_data_list->fb_offset = fb_data->fb_offset;
	/* If marker specified, associate it with a completion */
	if (upd_data->update_marker != 0) {
		/* Find available update marker and set it up */
		for (i = 0; i < EPDC_MAX_NUM_UPDATES; i++) {
			/* Marker value set to 0 signifies it is not currently in use */
			if (fb_data->update_marker_array[i].update_marker == 0) {
				fb_data->update_marker_array[i].update_marker = upd_data->update_marker;
				init_completion(&fb_data->update_marker_array[i].update_completion);
				upd_data_list->upd_marker_data = &fb_data->update_marker_array[i];
				break;
			}
		}
	} else {
		if (upd_data_list->upd_marker_data)
			upd_data_list->upd_marker_data->update_marker = 0;
	}

	upd_data_list->update_order = fb_data->order_cnt++;

	if (fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT) {
		/* Queued update scheme processing */

		/* Add processed Y buffer to update list */
		list_add_tail(&upd_data_list->list,
				  &fb_data->upd_buf_queue->list);

		spin_unlock_irqrestore(&fb_data->queue_lock, flags);

		/* Signal workqueue to handle new update */
		queue_work(fb_data->epdc_submit_workqueue,
			&fb_data->epdc_submit_work);

		return 0;
	}

	/* Snapshot update scheme processing */

	spin_unlock_irqrestore(&fb_data->queue_lock, flags);

	/*
	 * Hold on to original screen update region, which we
	 * will ultimately use when telling EPDC where to update on panel
	 */
	screen_upd_region = &upd_data_list->upd_data.update_region;

	ret = epdc_process_update(upd_data_list, fb_data);
	if (ret) {
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}

	/* Pass selected waveform mode back to user */
	upd_data->waveform_mode = upd_data_list->upd_data.waveform_mode;

	/* Get rotation-adjusted coordinates */
	adjust_coordinates(fb_data, &upd_data_list->upd_data.update_region,
		NULL);

	/* Grab lock for queue manipulation and update submission */
	spin_lock_irqsave(&fb_data->queue_lock, flags);

	/*
	 * Is the working buffer idle?
	 * If either the working buffer is busy, or there are no LUTs available,
	 * then we return and let the ISR handle the update later
	 */
	if ((fb_data->cur_update != NULL) || !epdc_any_luts_available()) {
		/* Add processed Y buffer to update list */
		list_add_tail(&upd_data_list->list,
			      &fb_data->upd_buf_queue->list);

		/* Return and allow the update to be submitted by the ISR. */
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		return 0;
	}

	/* Save current update */
	fb_data->cur_update = upd_data_list;

	/* LUTs are available, so we get one here */
	upd_data_list->lut_num = epdc_get_next_lut();

	/* Associate LUT with update marker */
	if (upd_data_list->upd_marker_data)
		if (upd_data_list->upd_marker_data->update_marker != 0)
			upd_data_list->upd_marker_data->lut_num = upd_data_list->lut_num;

	/* Mark LUT as containing new update */
	fb_data->lut_update_order[upd_data_list->lut_num] =
		upd_data_list->update_order;

	/* Clear status and Enable LUT complete and WB complete IRQs */
	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(fb_data->cur_update->lut_num, true);

	/* Program EPDC update to process buffer */
	epdc_set_update_addr(upd_data_list->phys_addr + upd_data_list->epdc_offs);
	epdc_set_update_coord(screen_upd_region->left, screen_upd_region->top);
	epdc_set_update_dimensions(screen_upd_region->width,
		screen_upd_region->height);
	if (upd_data_list->upd_data.temp != TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			upd_data_list->upd_data.temp);
		epdc_set_temp(temp_index);
	} else
		epdc_set_temp(fb_data->temp_index);

	epdc_submit_update(upd_data_list->lut_num,
			   upd_data_list->upd_data.waveform_mode,
			   upd_data_list->upd_data.update_mode, false, 0);

	spin_unlock_irqrestore(&fb_data->queue_lock, flags);
	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_send_update);

int mxc_epdc_fb_wait_update_complete(u32 update_marker, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	int ret;
	int i;

	/* 0 is an invalid update_marker value */
	if (update_marker == 0)
		return -EINVAL;

	/* Wait for completion associated with update_marker requested */
	for (i = 0; i < EPDC_MAX_NUM_UPDATES; i++) {
		if (fb_data->update_marker_array[i].update_marker == update_marker) {
			dev_dbg(fb_data->dev, "Waiting for marker %d\n", update_marker);
			ret = wait_for_completion_timeout(&fb_data->update_marker_array[i].update_completion, msecs_to_jiffies(5000));
			if (!ret)
				dev_err(fb_data->dev, "Timed out waiting for update completion\n");

			dev_dbg(fb_data->dev, "marker %d signalled!\n", update_marker);

			/* Reset marker so it can be reused */
			fb_data->update_marker_array[i].update_marker = 0;

			break;
		}
	}

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_wait_update_complete);

int mxc_epdc_fb_set_pwrdown_delay(u32 pwrdown_delay,
					    struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	fb_data->pwrdown_delay = pwrdown_delay;

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_pwrdown_delay);

int mxc_epdc_get_pwrdown_delay(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	return fb_data->pwrdown_delay;
}
EXPORT_SYMBOL(mxc_epdc_get_pwrdown_delay);

static int mxc_epdc_fb_ioctl(struct fb_info *info, unsigned int cmd,
			     unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = -EINVAL;

	switch (cmd) {
	case MXCFB_SET_WAVEFORM_MODES:
		{
			struct mxcfb_waveform_modes modes;
			if (!copy_from_user(&modes, argp, sizeof(modes))) {
				mxc_epdc_fb_set_waveform_modes(&modes, info);
				ret = 0;
			}
			break;
		}
	case MXCFB_SET_TEMPERATURE:
		{
			int temperature;
			if (!get_user(temperature, (int32_t __user *) arg))
				ret = mxc_epdc_fb_set_temperature(temperature,
					info);
			break;
		}
	case MXCFB_SET_AUTO_UPDATE_MODE:
		{
			u32 auto_mode = 0;
			if (!get_user(auto_mode, (__u32 __user *) arg))
				ret = mxc_epdc_fb_set_auto_update(auto_mode,
					info);
			break;
		}
	case MXCFB_SET_UPDATE_SCHEME:
		{
			u32 upd_scheme = 0;
			if (!get_user(upd_scheme, (__u32 __user *) arg))
				ret = mxc_epdc_fb_set_upd_scheme(upd_scheme,
					info);
			break;
		}
	case MXCFB_SEND_UPDATE:
		{
			struct mxcfb_update_data upd_data;
			if (!copy_from_user(&upd_data, argp,
				sizeof(upd_data))) {
				ret = mxc_epdc_fb_send_update(&upd_data, info);
				if (ret == 0 && copy_to_user(argp, &upd_data,
					sizeof(upd_data)))
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}

			break;
		}
	case MXCFB_WAIT_FOR_UPDATE_COMPLETE:
		{
			u32 update_marker = 0;
			if (!get_user(update_marker, (__u32 __user *) arg))
				ret =
				    mxc_epdc_fb_wait_update_complete(update_marker,
					info);
			break;
		}

	case MXCFB_SET_PWRDOWN_DELAY:
		{
			int delay = 0;
			if (!get_user(delay, (__u32 __user *) arg))
				ret =
				    mxc_epdc_fb_set_pwrdown_delay(delay, info);
			break;
		}

	case MXCFB_GET_PWRDOWN_DELAY:
		{
			int pwrdown_delay = mxc_epdc_get_pwrdown_delay(info);
			if (put_user(pwrdown_delay,
				(int __user *)argp))
				ret = -EFAULT;
			ret = 0;
			break;
		}
	default:
		break;
	}
	return ret;
}

static void mxc_epdc_fb_update_pages(struct mxc_epdc_fb_data *fb_data,
				     u16 y1, u16 y2)
{
	struct mxcfb_update_data update;

	/* Do partial screen update, Update full horizontal lines */
	update.update_region.left = 0;
	update.update_region.width = fb_data->epdc_fb_var.xres;
	update.update_region.top = y1;
	update.update_region.height = y2 - y1;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.update_mode = UPDATE_MODE_FULL;
	update.update_marker = 0;
	update.temp = TEMP_USE_AMBIENT;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, &fb_data->info);
}

/* this is called back from the deferred io workqueue */
static void mxc_epdc_fb_deferred_io(struct fb_info *info,
				    struct list_head *pagelist)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	struct page *page;
	unsigned long beg, end;
	int y1, y2, miny, maxy;

	if (fb_data->auto_mode != AUTO_UPDATE_MODE_AUTOMATIC_MODE)
		return;

	miny = INT_MAX;
	maxy = 0;
	list_for_each_entry(page, pagelist, lru) {
		beg = page->index << PAGE_SHIFT;
		end = beg + PAGE_SIZE - 1;
		y1 = beg / info->fix.line_length;
		y2 = end / info->fix.line_length;
		if (y2 >= fb_data->epdc_fb_var.yres)
			y2 = fb_data->epdc_fb_var.yres - 1;
		if (miny > y1)
			miny = y1;
		if (maxy < y2)
			maxy = y2;
	}

	mxc_epdc_fb_update_pages(fb_data, miny, maxy);
}

void mxc_epdc_fb_flush_updates(struct mxc_epdc_fb_data *fb_data)
{
	unsigned long flags;
	/* Grab queue lock to prevent any new updates from being submitted */
	spin_lock_irqsave(&fb_data->queue_lock, flags);

	if (!is_free_list_full(fb_data)) {
		/* Initialize event signalling updates are done */
		init_completion(&fb_data->updates_done);
		fb_data->waiting_for_idle = true;

		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		/* Wait for any currently active updates to complete */
		wait_for_completion_timeout(&fb_data->updates_done, msecs_to_jiffies(2000));
		spin_lock_irqsave(&fb_data->queue_lock, flags);
		fb_data->waiting_for_idle = false;
	}

	spin_unlock_irqrestore(&fb_data->queue_lock, flags);
}

static int mxc_epdc_fb_blank(int blank, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	dev_dbg(fb_data->dev, "blank = %d\n", blank);

	if (fb_data->blank == blank)
		return 0;

	fb_data->blank = blank;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		mxc_epdc_fb_flush_updates(fb_data);
		break;
	}
	return 0;
}

static int mxc_epdc_fb_pan_display(struct fb_var_screeninfo *var,
				   struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	u_int y_bottom;
	unsigned long flags;

	dev_dbg(info->device, "%s: var->xoffset %d, info->var.xoffset %d\n",
		 __func__, var->xoffset, info->var.xoffset);
	/* check if var is valid; also, xpan is not supported */
	if (!var || (var->xoffset != info->var.xoffset) ||
	    (var->yoffset + var->yres > var->yres_virtual)) {
		dev_dbg(info->device, "x panning not supported\n");
		return -EINVAL;
	}

	if ((fb_data->epdc_fb_var.xoffset == var->xoffset) &&
		(fb_data->epdc_fb_var.yoffset == var->yoffset))
		return 0;	/* No change, do nothing */

	spin_lock_irqsave(&fb_data->queue_lock, flags);

	y_bottom = var->yoffset;

	if (!(var->vmode & FB_VMODE_YWRAP))
		y_bottom += var->yres;

	if (y_bottom > info->var.yres_virtual) {
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		return -EINVAL;
	}

	fb_data->fb_offset = (var->yoffset * var->xres_virtual + var->xoffset)
		* (var->bits_per_pixel) / 8;

	fb_data->epdc_fb_var.xoffset = var->xoffset;
	fb_data->epdc_fb_var.yoffset = var->yoffset;

	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;

	spin_unlock_irqrestore(&fb_data->queue_lock, flags);

	return 0;
}

static struct fb_ops mxc_epdc_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mxc_epdc_fb_check_var,
	.fb_set_par = mxc_epdc_fb_set_par,
	.fb_setcolreg = mxc_epdc_fb_setcolreg,
	.fb_pan_display = mxc_epdc_fb_pan_display,
	.fb_ioctl = mxc_epdc_fb_ioctl,
	.fb_mmap = mxc_epdc_fb_mmap,
	.fb_blank = mxc_epdc_fb_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static struct fb_deferred_io mxc_epdc_fb_defio = {
	.delay = HZ,
	.deferred_io = mxc_epdc_fb_deferred_io,
};

static void epdc_done_work_func(struct work_struct *work)
{
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data,
			epdc_done_work.work);
	epdc_powerdown(fb_data);
}

static bool is_free_list_full(struct mxc_epdc_fb_data *fb_data)
{
	int count = 0;
	struct update_data_list *plist;

	/* Count buffers in free buffer list */
	list_for_each_entry(plist, &fb_data->upd_buf_free_list->list, list)
		count++;

	/* Check to see if all buffers are in this list */
	if (count == EPDC_MAX_NUM_UPDATES)
		return true;
	else
		return false;
}

static irqreturn_t mxc_epdc_irq_handler(int irq, void *dev_id)
{
	struct mxc_epdc_fb_data *fb_data = dev_id;
	struct update_data_list *collision_update;
	struct mxcfb_rect *next_upd_region;
	unsigned long flags;
	int temp_index;
	u32 luts_completed_mask;
	u32 temp_mask;
	u32 lut;
	bool ignore_collision = false;
	int i, j;

	/*
	 * If we just completed one-time panel init, bypass
	 * queue handling, clear interrupt and return
	 */
	if (fb_data->in_init) {
		if (epdc_is_working_buffer_complete()) {
			epdc_working_buf_intr(false);
			epdc_clear_working_buf_irq();
			dev_dbg(fb_data->dev, "Cleared WB for init update\n");
		}

		if (epdc_is_lut_complete(0)) {
			epdc_lut_complete_intr(0, false);
			epdc_clear_lut_complete_irq(0);
			fb_data->in_init = false;
			dev_dbg(fb_data->dev, "Cleared LUT complete for init update\n");
		}

		return IRQ_HANDLED;
	}

	if (!(__raw_readl(EPDC_IRQ_MASK) & __raw_readl(EPDC_IRQ)))
		return IRQ_HANDLED;

	if (__raw_readl(EPDC_IRQ) & EPDC_IRQ_TCE_UNDERRUN_IRQ) {
		dev_err(fb_data->dev, "TCE underrun!  Panel may lock up.\n");
		return IRQ_HANDLED;
	}

	/* Protect access to buffer queues and to update HW */
	spin_lock_irqsave(&fb_data->queue_lock, flags);

	/* Free any LUTs that have completed */
	luts_completed_mask = 0;
	for (i = 0; i < EPDC_NUM_LUTS; i++) {
		if (!epdc_is_lut_complete(i))
			continue;

		dev_dbg(fb_data->dev, "\nLUT %d completed\n", i);

		/* Disable IRQ for completed LUT */
		epdc_lut_complete_intr(i, false);

		/*
		 * Go through all updates in the collision list and
		 * unmask any updates that were colliding with
		 * the completed LUT.
		 */
		list_for_each_entry(collision_update,
				    &fb_data->upd_buf_collision_list->
				    list, list) {
			collision_update->collision_mask =
			    collision_update->collision_mask & ~(1 << i);
		}

		epdc_clear_lut_complete_irq(i);

		luts_completed_mask |= 1 << i;

		fb_data->lut_update_order[i] = 0;

		/* Signal completion if submit workqueue needs a LUT */
		if (fb_data->waiting_for_lut) {
			complete(&fb_data->update_res_free);
			fb_data->waiting_for_lut = false;
		}

		/* Signal completion if anyone waiting on this LUT */
		for (j = 0; j < EPDC_MAX_NUM_UPDATES; j++) {
			if (fb_data->update_marker_array[j].lut_num != i)
				continue;

			/* Signal completion of update */
			dev_dbg(fb_data->dev,
				"Signaling marker %d\n",
				fb_data->update_marker_array[j].update_marker);
			complete(&fb_data->update_marker_array[j].update_completion);
			/* Ensure this doesn't get signaled again inadvertently */
			fb_data->update_marker_array[j].lut_num = INVALID_LUT;
		}
	}

	/* Check to see if all updates have completed */
	if (is_free_list_full(fb_data) &&
		(fb_data->cur_update == NULL) &&
		!epdc_any_luts_active()) {

		if (fb_data->pwrdown_delay != FB_POWERDOWN_DISABLE) {
			/*
			 * Set variable to prevent overlapping
			 * enable/disable requests
			 */
			fb_data->powering_down = true;

			/* Schedule task to disable EPDC HW until next update */
			schedule_delayed_work(&fb_data->epdc_done_work,
				msecs_to_jiffies(fb_data->pwrdown_delay));

			/* Reset counter to reduce chance of overflow */
			fb_data->order_cnt = 0;
		}

		if (fb_data->waiting_for_idle)
			complete(&fb_data->updates_done);
	}

	/* Is Working Buffer busy? */
	if (epdc_is_working_buffer_busy()) {
		/* Can't submit another update until WB is done */
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		return IRQ_HANDLED;
	}

	/*
	 * Were we waiting on working buffer?
	 * If so, update queues and check for collisions
	 */
	if (fb_data->cur_update != NULL) {
		dev_dbg(fb_data->dev, "\nWorking buffer completed\n");

		/* Signal completion if submit workqueue was waiting on WB */
		if (fb_data->waiting_for_wb) {
			complete(&fb_data->update_res_free);
			fb_data->waiting_for_lut = false;
		}

		/* Was there a collision? */
		if (epdc_is_collision()) {
			/* Check list of colliding LUTs, and add to our collision mask */
			fb_data->cur_update->collision_mask =
			    epdc_get_colliding_luts();

			/* Clear collisions that just completed */
			fb_data->cur_update->collision_mask &= ~luts_completed_mask;

			dev_dbg(fb_data->dev, "\nCollision mask = 0x%x\n",
			       epdc_get_colliding_luts());

			/*
			 * If we collide with newer updates, then
			 * we don't need to re-submit the update. The
			 * idea is that the newer updates should take
			 * precedence anyways, so we don't want to
			 * overwrite them.
			 */
			for (temp_mask = fb_data->cur_update->collision_mask, lut = 0;
				temp_mask != 0;
				lut++, temp_mask = temp_mask >> 1) {
				if (!(temp_mask & 0x1))
					continue;

				if (fb_data->lut_update_order[lut] >=
					fb_data->cur_update->update_order) {
					dev_dbg(fb_data->dev, "Ignoring collision with newer update.\n");
					ignore_collision = true;
					break;
				}
			}

			if (ignore_collision) {
				/* Add to free buffer list */
				list_add_tail(&fb_data->cur_update->list,
					 &fb_data->upd_buf_free_list->list);
			} else {
				/*
				 * If update has a marker, clear the LUT, since we
				 * don't want to signal that it is complete.
				 */
				if (fb_data->cur_update->upd_marker_data)
					if (fb_data->cur_update->upd_marker_data->update_marker != 0)
						fb_data->cur_update->upd_marker_data->lut_num = INVALID_LUT;

				/* Move to collision list */
				list_add_tail(&fb_data->cur_update->list,
					 &fb_data->upd_buf_collision_list->list);
			}
		} else {
			/* Add to free buffer list */
			list_add_tail(&fb_data->cur_update->list,
				 &fb_data->upd_buf_free_list->list);
		}
		/* Clear current update */
		fb_data->cur_update = NULL;

		/* Clear IRQ for working buffer */
		epdc_working_buf_intr(false);
		epdc_clear_working_buf_irq();
	}

	if (fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT) {
		/* Queued update scheme processing */

		/* Schedule task to submit collision and pending update */
		if (!fb_data->powering_down)
			queue_work(fb_data->epdc_submit_workqueue,
				&fb_data->epdc_submit_work);

		/* Release buffer queues */
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);

		return IRQ_HANDLED;
	}

	/* Snapshot update scheme processing */

	/* Check to see if any LUTs are free */
	if (!epdc_any_luts_available()) {
		dev_dbg(fb_data->dev, "No luts available.\n");
		spin_unlock_irqrestore(&fb_data->queue_lock, flags);
		return IRQ_HANDLED;
	}

	/*
	 * Are any of our collision updates able to go now?
	 * Go through all updates in the collision list and check to see
	 * if the collision mask has been fully cleared
	 */
	list_for_each_entry(collision_update,
			    &fb_data->upd_buf_collision_list->list, list) {

		if (collision_update->collision_mask != 0)
			continue;

		dev_dbg(fb_data->dev, "A collision update is ready to go!\n");
		/*
		 * We have a collision cleared, so select it
		 * and we will retry the update
		 */
		fb_data->cur_update = collision_update;
		list_del_init(&fb_data->cur_update->list);
		break;
	}

	/*
	 * If we didn't find a collision update ready to go,
	 * we try to grab one from the update queue
	 */
	if (fb_data->cur_update == NULL) {
		/* Is update list empty? */
		if (list_empty(&fb_data->upd_buf_queue->list)) {
			dev_dbg(fb_data->dev, "No pending updates.\n");

			/* No updates pending, so we are done */
			spin_unlock_irqrestore(&fb_data->queue_lock, flags);
			return IRQ_HANDLED;
		} else {
			dev_dbg(fb_data->dev, "Found a pending update!\n");

			/* Process next item in update list */
			fb_data->cur_update =
			    list_entry(fb_data->upd_buf_queue->list.next,
				       struct update_data_list, list);
			list_del_init(&fb_data->cur_update->list);
		}
	}

	/* LUTs are available, so we get one here */
	fb_data->cur_update->lut_num = epdc_get_next_lut();

	/* Associate LUT with update marker */
	if ((fb_data->cur_update->upd_marker_data)
		&& (fb_data->cur_update->upd_marker_data->update_marker != 0))
		fb_data->cur_update->upd_marker_data->lut_num =
						fb_data->cur_update->lut_num;

	/* Mark LUT as containing new update */
	fb_data->lut_update_order[fb_data->cur_update->lut_num] =
		fb_data->cur_update->update_order;

	/* Enable Collision and WB complete IRQs */
	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(fb_data->cur_update->lut_num, true);

	/* Program EPDC update to process buffer */
	next_upd_region = &fb_data->cur_update->upd_data.update_region;
	if (fb_data->cur_update->upd_data.temp != TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data, fb_data->cur_update->upd_data.temp);
		epdc_set_temp(temp_index);
	} else
		epdc_set_temp(fb_data->temp_index);
	epdc_set_update_addr(fb_data->cur_update->phys_addr + fb_data->cur_update->epdc_offs);
	epdc_set_update_coord(next_upd_region->left, next_upd_region->top);
	epdc_set_update_dimensions(next_upd_region->width,
				   next_upd_region->height);
	epdc_submit_update(fb_data->cur_update->lut_num,
			   fb_data->cur_update->upd_data.waveform_mode,
			   fb_data->cur_update->upd_data.update_mode, false, 0);

	/* Release buffer queues */
	spin_unlock_irqrestore(&fb_data->queue_lock, flags);

	return IRQ_HANDLED;
}

static void draw_mode0(struct mxc_epdc_fb_data *fb_data)
{
	u32 *upd_buf_ptr;
	int i;
	struct fb_var_screeninfo *screeninfo = &fb_data->epdc_fb_var;
	u32 xres, yres;

	upd_buf_ptr = (u32 *)fb_data->info.screen_base;

	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(0, true);
	fb_data->in_init = true;

	/* Use unrotated (native) width/height */
	if ((screeninfo->rotate == FB_ROTATE_CW) ||
		(screeninfo->rotate == FB_ROTATE_CCW)) {
		xres = screeninfo->yres;
		yres = screeninfo->xres;
	} else {
		xres = screeninfo->xres;
		yres = screeninfo->yres;
	}

	/* Program EPDC update to process buffer */
	epdc_set_update_addr(fb_data->phys_start);
	epdc_set_update_coord(0, 0);
	epdc_set_update_dimensions(xres, yres);
	epdc_submit_update(0, fb_data->wv_modes.mode_init, UPDATE_MODE_FULL, true, 0xFF);

	dev_dbg(fb_data->dev, "Mode0 update - Waiting for LUT to complete...\n");

	/* Will timeout after ~4-5 seconds */

	for (i = 0; i < 40; i++) {
		if (!epdc_is_lut_active(0)) {
			dev_dbg(fb_data->dev, "Mode0 init complete\n");
			return;
		}
		msleep(100);
	}

	dev_err(fb_data->dev, "Mode0 init failed!\n");

	return;
}


static void mxc_epdc_fb_fw_handler(const struct firmware *fw,
						     void *context)
{
	struct mxc_epdc_fb_data *fb_data = context;
	int ret;
	struct mxcfb_waveform_data_file *wv_file;
	int wv_data_offs;
	int i;
	struct mxcfb_update_data update;
	struct fb_var_screeninfo *screeninfo = &fb_data->epdc_fb_var;
	u32 xres, yres;

	if (fw == NULL) {
		/* If default FW file load failed, we give up */
		if (fb_data->fw_default_load)
			return;

		/* Try to load default waveform */
		dev_dbg(fb_data->dev,
			"Can't find firmware. Trying fallback fw\n");
		fb_data->fw_default_load = true;
		ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			"imx/epdc.fw", fb_data->dev, GFP_KERNEL, fb_data,
			mxc_epdc_fb_fw_handler);
		if (ret)
			dev_err(fb_data->dev,
				"Failed request_firmware_nowait err %d\n", ret);

		return;
	}

	wv_file = (struct mxcfb_waveform_data_file *)fw->data;

	/* Get size and allocate temperature range table */
	fb_data->trt_entries = wv_file->wdh.trc + 1;
	fb_data->temp_range_bounds = kzalloc(fb_data->trt_entries, GFP_KERNEL);

	for (i = 0; i < fb_data->trt_entries; i++)
		dev_dbg(fb_data->dev, "trt entry #%d = 0x%x\n", i, *((u8 *)&wv_file->data + i));

	/* Copy TRT data */
	memcpy(fb_data->temp_range_bounds, &wv_file->data, fb_data->trt_entries);

	/* Set default temperature index using TRT and room temp */
	fb_data->temp_index = mxc_epdc_fb_get_temp_index(fb_data, DEFAULT_TEMP);

	/* Get offset and size for waveform data */
	wv_data_offs = sizeof(wv_file->wdh) + fb_data->trt_entries + 1;
	fb_data->waveform_buffer_size = fw->size - wv_data_offs;

	/* Allocate memory for waveform data */
	fb_data->waveform_buffer_virt = dma_alloc_coherent(fb_data->dev,
						fb_data->waveform_buffer_size,
						&fb_data->waveform_buffer_phys,
						GFP_DMA);
	if (fb_data->waveform_buffer_virt == NULL) {
		dev_err(fb_data->dev, "Can't allocate mem for waveform!\n");
		return;
	}

	memcpy(fb_data->waveform_buffer_virt, (u8 *)(fw->data) + wv_data_offs,
		fb_data->waveform_buffer_size);

	release_firmware(fw);

	/* Enable clocks to access EPDC regs */
	clk_enable(fb_data->epdc_clk_axi);

	/* Enable pix clk for EPDC */
	clk_enable(fb_data->epdc_clk_pix);
	clk_set_rate(fb_data->epdc_clk_pix, fb_data->cur_mode->vmode->pixclock);

	epdc_init_sequence(fb_data);

	/* Disable clocks */
	clk_disable(fb_data->epdc_clk_axi);
	clk_disable(fb_data->epdc_clk_pix);

	fb_data->hw_ready = true;

	/* Use unrotated (native) width/height */
	if ((screeninfo->rotate == FB_ROTATE_CW) ||
		(screeninfo->rotate == FB_ROTATE_CCW)) {
		xres = screeninfo->yres;
		yres = screeninfo->xres;
	} else {
		xres = screeninfo->xres;
		yres = screeninfo->yres;
	}

	update.update_region.left = 0;
	update.update_region.width = xres;
	update.update_region.top = 0;
	update.update_region.height = yres;
	update.update_mode = UPDATE_MODE_FULL;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.update_marker = INIT_UPDATE_MARKER;
	update.temp = TEMP_USE_AMBIENT;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, &fb_data->info);

	/* Block on initial update */
	ret = mxc_epdc_fb_wait_update_complete(update.update_marker,
		&fb_data->info);
	if (ret < 0)
		dev_err(fb_data->dev,
			"Wait for update complete failed.  Error = 0x%x", ret);
}

static int mxc_epdc_fb_init_hw(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	int ret;

	/*
	 * Create fw search string based on ID string in selected videomode.
	 * Format is "imx/epdc_[panel string].fw"
	 */
	if (fb_data->cur_mode) {
		strcat(fb_data->fw_str, "imx/epdc_");
		strcat(fb_data->fw_str, fb_data->cur_mode->vmode->name);
		strcat(fb_data->fw_str, ".fw");
	}

	fb_data->fw_default_load = false;

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				fb_data->fw_str, fb_data->dev, GFP_KERNEL,
				fb_data, mxc_epdc_fb_fw_handler);
	if (ret)
		dev_dbg(fb_data->dev,
			"Failed request_firmware_nowait err %d\n", ret);

	return ret;
}

static ssize_t store_update(struct device *device,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct mxcfb_update_data update;
	struct fb_info *info = dev_get_drvdata(device);
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	if (strncmp(buf, "direct", 6) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_du;
	else if (strncmp(buf, "gc16", 4) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc16;
	else if (strncmp(buf, "gc4", 3) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc4;

	/* Now, request full screen update */
	update.update_region.left = 0;
	update.update_region.width = fb_data->epdc_fb_var.xres;
	update.update_region.top = 0;
	update.update_region.height = fb_data->epdc_fb_var.yres;
	update.update_mode = UPDATE_MODE_FULL;
	update.temp = TEMP_USE_AMBIENT;
	update.update_marker = 0;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, info);

	return count;
}

static struct device_attribute fb_attrs[] = {
	__ATTR(update, S_IRUGO|S_IWUSR, NULL, store_update),
};

int __devinit mxc_epdc_fb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mxc_epdc_fb_data *fb_data;
	struct resource *res;
	struct fb_info *info;
	char *options, *opt;
	char *panel_str = NULL;
	char name[] = "mxcepdcfb";
	struct fb_videomode *vmode;
	int xres_virt, yres_virt, buf_size;
	int xres_virt_rot, yres_virt_rot, buf_size_rot;
	struct fb_var_screeninfo *var_info;
	struct fb_fix_screeninfo *fix_info;
	struct pxp_config_data *pxp_conf;
	struct pxp_proc_data *proc_data;
	struct scatterlist *sg;
	struct update_data_list *upd_list;
	struct update_data_list *plist, *temp_list;
	int i;
	unsigned long x_mem_size = 0;
#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	struct mxcfb_update_data update;
#endif

	fb_data = (struct mxc_epdc_fb_data *)framebuffer_alloc(
			sizeof(struct mxc_epdc_fb_data), &pdev->dev);
	if (fb_data == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	/* Get platform data and check validity */
	fb_data->pdata = pdev->dev.platform_data;
	if ((fb_data->pdata == NULL) || (fb_data->pdata->num_modes < 1)
		|| (fb_data->pdata->epdc_mode == NULL)
		|| (fb_data->pdata->epdc_mode->vmode == NULL)) {
		ret = -EINVAL;
		goto out_fbdata;
	}

	if (fb_get_options(name, &options)) {
		ret = -ENODEV;
		goto out_fbdata;
	}

	if (options)
		while ((opt = strsep(&options, ",")) != NULL) {
			if (!*opt)
				continue;

			if (!strncmp(opt, "bpp=", 4))
				fb_data->default_bpp =
					simple_strtoul(opt + 4, NULL, 0);
			else if (!strncmp(opt, "x_mem=", 6))
				x_mem_size = memparse(opt + 6, NULL);
			else
				panel_str = opt;
		}

	fb_data->dev = &pdev->dev;

	if (!fb_data->default_bpp)
		fb_data->default_bpp = 16;

	/* Set default (first defined mode) before searching for a match */
	fb_data->cur_mode = &fb_data->pdata->epdc_mode[0];

	if (panel_str)
		for (i = 0; i < fb_data->pdata->num_modes; i++)
			if (!strcmp(fb_data->pdata->epdc_mode[i].vmode->name,
						panel_str)) {
				fb_data->cur_mode =
					&fb_data->pdata->epdc_mode[i];
				break;
			}

	vmode = fb_data->cur_mode->vmode;

	platform_set_drvdata(pdev, fb_data);
	info = &fb_data->info;

	/* Allocate color map for the FB */
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret)
		goto out_fbdata;

	dev_dbg(&pdev->dev, "resolution %dx%d, bpp %d\n",
		vmode->xres, vmode->yres, fb_data->default_bpp);

	/*
	 * GPU alignment restrictions dictate framebuffer parameters:
	 * - 32-byte alignment for buffer width
	 * - 128-byte alignment for buffer height
	 * => 4K buffer alignment for buffer start
	 */
	xres_virt = ALIGN(vmode->xres, 32);
	yres_virt = ALIGN(vmode->yres, 128);
	buf_size = PAGE_ALIGN(xres_virt * yres_virt * fb_data->default_bpp/8);

	/*
	 * Have to check to see if aligned buffer size when rotated
	 * is bigger than when not rotated, and use the max
	 */
	xres_virt_rot = ALIGN(vmode->yres, 32);
	yres_virt_rot = ALIGN(vmode->xres, 128);
	buf_size_rot = PAGE_ALIGN(xres_virt_rot * yres_virt_rot
						* fb_data->default_bpp/8);
	buf_size = (buf_size > buf_size_rot) ? buf_size : buf_size_rot;

	/* Compute the number of screens needed based on X memory requested */
	if (x_mem_size > 0) {
		fb_data->num_screens = DIV_ROUND_UP(x_mem_size, buf_size);
		if (fb_data->num_screens < NUM_SCREENS_MIN)
			fb_data->num_screens = NUM_SCREENS_MIN;
		else if (buf_size * fb_data->num_screens > SZ_16M)
			fb_data->num_screens = SZ_16M / buf_size;
	} else
		fb_data->num_screens = NUM_SCREENS_MIN;

	fb_data->map_size = buf_size * fb_data->num_screens;
	dev_dbg(&pdev->dev, "memory to allocate: %d\n", fb_data->map_size);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENODEV;
		goto out_cmap;
	}

	epdc_base = ioremap(res->start, SZ_4K);
	if (epdc_base == NULL) {
		ret = -ENOMEM;
		goto out_cmap;
	}

	/* Allocate FB memory */
	info->screen_base = dma_alloc_writecombine(&pdev->dev,
						  fb_data->map_size,
						  &fb_data->phys_start,
						  GFP_DMA);

	if (info->screen_base == NULL) {
		ret = -ENOMEM;
		goto out_mapregs;
	}
	dev_dbg(&pdev->dev, "allocated at %p:0x%x\n", info->screen_base,
		fb_data->phys_start);

	var_info = &info->var;
	var_info->activate = FB_ACTIVATE_TEST;
	var_info->bits_per_pixel = fb_data->default_bpp;
	var_info->xres = vmode->xres;
	var_info->yres = vmode->yres;
	var_info->xres_virtual = xres_virt;
	/* Additional screens allow for panning  and buffer flipping */
	var_info->yres_virtual = yres_virt * fb_data->num_screens;

	var_info->pixclock = vmode->pixclock;
	var_info->left_margin = vmode->left_margin;
	var_info->right_margin = vmode->right_margin;
	var_info->upper_margin = vmode->upper_margin;
	var_info->lower_margin = vmode->lower_margin;
	var_info->hsync_len = vmode->hsync_len;
	var_info->vsync_len = vmode->vsync_len;
	var_info->vmode = FB_VMODE_NONINTERLACED;

	switch (fb_data->default_bpp) {
	case 32:
	case 24:
		var_info->red.offset = 16;
		var_info->red.length = 8;
		var_info->green.offset = 8;
		var_info->green.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.length = 8;
		break;

	case 16:
		var_info->red.offset = 11;
		var_info->red.length = 5;
		var_info->green.offset = 5;
		var_info->green.length = 6;
		var_info->blue.offset = 0;
		var_info->blue.length = 5;
		break;

	case 8:
		/*
		 * For 8-bit grayscale, R, G, and B offset are equal.
		 *
		 */
		var_info->grayscale = GRAYSCALE_8BIT;

		var_info->red.length = 8;
		var_info->red.offset = 0;
		var_info->red.msb_right = 0;
		var_info->green.length = 8;
		var_info->green.offset = 0;
		var_info->green.msb_right = 0;
		var_info->blue.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.msb_right = 0;
		break;

	default:
		dev_err(&pdev->dev, "unsupported bitwidth %d\n",
			fb_data->default_bpp);
		ret = -EINVAL;
		goto out_dma_fb;
	}

	fix_info = &info->fix;

	strcpy(fix_info->id, "mxc_epdc_fb");
	fix_info->type = FB_TYPE_PACKED_PIXELS;
	fix_info->visual = FB_VISUAL_TRUECOLOR;
	fix_info->xpanstep = 0;
	fix_info->ypanstep = 0;
	fix_info->ywrapstep = 0;
	fix_info->accel = FB_ACCEL_NONE;
	fix_info->smem_start = fb_data->phys_start;
	fix_info->smem_len = fb_data->map_size;
	fix_info->ypanstep = 0;

	fb_data->native_width = vmode->xres;
	fb_data->native_height = vmode->yres;

	info->fbops = &mxc_epdc_fb_ops;
	info->var.activate = FB_ACTIVATE_NOW;
	info->pseudo_palette = fb_data->pseudo_palette;
	info->screen_size = info->fix.smem_len;
	info->flags = FBINFO_FLAG_DEFAULT;

	mxc_epdc_fb_set_fix(info);

	fb_data->auto_mode = AUTO_UPDATE_MODE_REGION_MODE;
	fb_data->upd_scheme = UPDATE_SCHEME_SNAPSHOT;

	/* Initialize our internal copy of the screeninfo */
	fb_data->epdc_fb_var = *var_info;
	fb_data->fb_offset = 0;

	/* Allocate head objects for our lists */
	fb_data->upd_buf_queue =
	    kzalloc(sizeof(struct update_data_list), GFP_KERNEL);
	fb_data->upd_buf_collision_list =
	    kzalloc(sizeof(struct update_data_list), GFP_KERNEL);
	fb_data->upd_buf_free_list =
	    kzalloc(sizeof(struct update_data_list), GFP_KERNEL);
	if ((fb_data->upd_buf_queue == NULL) || (fb_data->upd_buf_free_list == NULL)
	    || (fb_data->upd_buf_collision_list == NULL)) {
		ret = -ENOMEM;
		goto out_dma_fb;
	}

	/*
	 * Initialize lists for update requests, update collisions,
	 * and available update (PxP output) buffers
	 */
	INIT_LIST_HEAD(&fb_data->upd_buf_queue->list);
	INIT_LIST_HEAD(&fb_data->upd_buf_free_list->list);
	INIT_LIST_HEAD(&fb_data->upd_buf_collision_list->list);

	/* Allocate update buffers and add them to the list */
	for (i = 0; i < EPDC_MAX_NUM_UPDATES; i++) {
		upd_list = kzalloc(sizeof(*upd_list), GFP_KERNEL);
		if (upd_list == NULL) {
			ret = -ENOMEM;
			goto out_upd_buffers;
		}

		/* Clear update data structure */
		memset(&upd_list->upd_data, 0,
		       sizeof(struct mxcfb_update_data));

		/*
		 * Each update buffer is 1 byte per pixel, and can
		 * be as big as the full-screen frame buffer
		 */
		upd_list->size = info->var.xres * info->var.yres;

		/* Allocate memory for PxP output buffer */
		upd_list->virt_addr =
		    dma_alloc_coherent(fb_data->info.device, upd_list->size,
				       &upd_list->phys_addr, GFP_DMA);
		if (upd_list->virt_addr == NULL) {
			kfree(upd_list);
			ret = -ENOMEM;
			goto out_upd_buffers;
		}

		/* Add newly allocated buffer to free list */
		list_add(&upd_list->list, &fb_data->upd_buf_free_list->list);

		dev_dbg(fb_data->info.device, "allocated %d bytes @ 0x%08X\n",
			upd_list->size, upd_list->phys_addr);
	}

	fb_data->working_buffer_size = vmode->yres * vmode->xres * 2;
	/* Allocate memory for EPDC working buffer */
	fb_data->working_buffer_virt =
	    dma_alloc_coherent(&pdev->dev, fb_data->working_buffer_size,
			       &fb_data->working_buffer_phys, GFP_DMA);
	if (fb_data->working_buffer_virt == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for working buf!\n");
		ret = -ENOMEM;
		goto out_upd_buffers;
	}

	/* Initialize EPDC pins */
	if (fb_data->pdata->get_pins)
		fb_data->pdata->get_pins();

	fb_data->epdc_clk_axi = clk_get(fb_data->dev, "epdc_axi");
	if (IS_ERR(fb_data->epdc_clk_axi)) {
		dev_err(&pdev->dev, "Unable to get EPDC AXI clk."
			"err = 0x%x\n", (int)fb_data->epdc_clk_axi);
		ret = -ENODEV;
		goto out_upd_buffers;
	}
	fb_data->epdc_clk_pix = clk_get(fb_data->dev, "epdc_pix");
	if (IS_ERR(fb_data->epdc_clk_pix)) {
		dev_err(&pdev->dev, "Unable to get EPDC pix clk."
			"err = 0x%x\n", (int)fb_data->epdc_clk_pix);
		ret = -ENODEV;
		goto out_upd_buffers;
	}

	fb_data->in_init = false;

	fb_data->hw_ready = false;

	/*
	 * Set default waveform mode values.
	 * Should be overwritten via ioctl.
	 */
	fb_data->wv_modes.mode_init = 0;
	fb_data->wv_modes.mode_du = 1;
	fb_data->wv_modes.mode_gc4 = 3;
	fb_data->wv_modes.mode_gc8 = 2;
	fb_data->wv_modes.mode_gc16 = 2;
	fb_data->wv_modes.mode_gc32 = 2;

	/* Initialize markers */
	for (i = 0; i < EPDC_MAX_NUM_UPDATES; i++) {
		fb_data->update_marker_array[i].update_marker = 0;
		fb_data->update_marker_array[i].lut_num = INVALID_LUT;
	}

	/* Initialize all LUTs to inactive */
	for (i = 0; i < EPDC_NUM_LUTS; i++)
		fb_data->lut_update_order[i] = 0;

	/* Retrieve EPDC IRQ num */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot get IRQ resource\n");
		ret = -ENODEV;
		goto out_dma_work_buf;
	}
	fb_data->epdc_irq = res->start;

	/* Register IRQ handler */
	ret = request_irq(fb_data->epdc_irq, mxc_epdc_irq_handler, 0,
			"fb_dma", fb_data);
	if (ret) {
		dev_err(&pdev->dev, "request_irq (%d) failed with error %d\n",
			fb_data->epdc_irq, ret);
		ret = -ENODEV;
		goto out_dma_work_buf;
	}

	INIT_DELAYED_WORK(&fb_data->epdc_done_work, epdc_done_work_func);
	fb_data->epdc_submit_workqueue = create_rt_workqueue("submit");
	INIT_WORK(&fb_data->epdc_submit_work, epdc_submit_work_func);

	info->fbdefio = &mxc_epdc_fb_defio;
#ifdef CONFIG_FB_MXC_EINK_AUTO_UPDATE_MODE
	fb_deferred_io_init(info);
#endif

	/* get pmic regulators */
	fb_data->display_regulator = regulator_get(NULL, "DISPLAY");
	if (IS_ERR(fb_data->display_regulator)) {
		dev_err(&pdev->dev, "Unable to get display PMIC regulator."
			"err = 0x%x\n", (int)fb_data->display_regulator);
		ret = -ENODEV;
		goto out_irq;
	}
	fb_data->vcom_regulator = regulator_get(NULL, "VCOM");
	if (IS_ERR(fb_data->vcom_regulator)) {
		regulator_put(fb_data->display_regulator);
		dev_err(&pdev->dev, "Unable to get VCOM regulator."
			"err = 0x%x\n", (int)fb_data->vcom_regulator);
		ret = -ENODEV;
		goto out_irq;
	}

	if (device_create_file(info->dev, &fb_attrs[0]))
		dev_err(&pdev->dev, "Unable to create file from fb_attrs\n");

	fb_data->cur_update = NULL;

	spin_lock_init(&fb_data->queue_lock);

	mutex_init(&fb_data->pxp_mutex);

	mutex_init(&fb_data->power_mutex);

	/* PxP DMA interface */
	dmaengine_get();

	/*
	 * Fill out PxP config data structure based on FB info and
	 * processing tasks required
	 */
	pxp_conf = &fb_data->pxp_conf;
	proc_data = &pxp_conf->proc_data;

	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = proc_data->srect.width = fb_data->info.var.xres;
	proc_data->drect.height = proc_data->srect.height = fb_data->info.var.yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = 0;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;

	/*
	 * We initially configure PxP for RGB->YUV conversion,
	 * and only write out Y component of the result.
	 */

	/*
	 * Initialize S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
	pxp_conf->s0_param.width = fb_data->info.var.xres_virtual;
	pxp_conf->s0_param.height = fb_data->info.var.yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize OL0 channel parameters
	 * No overlay will be used for PxP operation
	 */
	for (i = 0; i < 8; i++) {
		pxp_conf->ol_param[i].combine_enable = false;
		pxp_conf->ol_param[i].width = 0;
		pxp_conf->ol_param[i].height = 0;
		pxp_conf->ol_param[i].pixel_fmt = PXP_PIX_FMT_RGB565;
		pxp_conf->ol_param[i].color_key_enable = false;
		pxp_conf->ol_param[i].color_key = -1;
		pxp_conf->ol_param[i].global_alpha_enable = false;
		pxp_conf->ol_param[i].global_alpha = 0;
		pxp_conf->ol_param[i].local_alpha_enable = false;
	}

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = fb_data->info.var.xres;
	pxp_conf->out_param.height = fb_data->info.var.yres;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GREY;

	/*
	 * Ensure this is set to NULL here...we will initialize pxp_chan
	 * later in our thread.
	 */
	fb_data->pxp_chan = NULL;

	/* Initialize Scatter-gather list containing 2 buffer addresses. */
	sg = fb_data->sg;
	sg_init_table(sg, 2);

	/*
	 * For use in PxP transfers:
	 * sg[0] holds the FB buffer pointer
	 * sg[1] holds the Output buffer pointer (configured before TX request)
	 */
	sg_dma_address(&sg[0]) = info->fix.smem_start;
	sg_set_page(&sg[0], virt_to_page(info->screen_base),
		    info->fix.smem_len, offset_in_page(info->screen_base));

	fb_data->order_cnt = 0;
	fb_data->waiting_for_wb = false;
	fb_data->waiting_for_lut = false;
	fb_data->waiting_for_idle = false;
	fb_data->blank = FB_BLANK_UNBLANK;
	fb_data->power_state = POWER_STATE_OFF;
	fb_data->powering_down = false;
	fb_data->pwrdown_delay = 0;

	/* Register FB */
	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&pdev->dev,
			"register_framebuffer failed with error %d\n", ret);
		goto out_dmaengine;
	}

	g_fb_data = fb_data;

#ifdef DEFAULT_PANEL_HW_INIT
	ret = mxc_epdc_fb_init_hw((struct fb_info *)fb_data);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize HW!\n");
	}
#endif

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	/* If FB console included, update display to show logo */
	update.update_region.left = 0;
	update.update_region.width = info->var.xres;
	update.update_region.top = 0;
	update.update_region.height = info->var.yres;
	update.update_mode = UPDATE_MODE_PARTIAL;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.update_marker = INIT_UPDATE_MARKER;
	update.temp = TEMP_USE_AMBIENT;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, info);

	ret = mxc_epdc_fb_wait_update_complete(update.update_marker, info);
	if (ret < 0)
		dev_err(fb_data->dev,
			"Wait for update complete failed.  Error = 0x%x", ret);
#endif

	goto out;

out_dmaengine:
	dmaengine_put();
out_irq:
	free_irq(fb_data->epdc_irq, fb_data);
out_dma_work_buf:
	dma_free_writecombine(&pdev->dev, fb_data->working_buffer_size,
		fb_data->working_buffer_virt, fb_data->working_buffer_phys);
	if (fb_data->pdata->put_pins)
		fb_data->pdata->put_pins();
out_upd_buffers:
	list_for_each_entry_safe(plist, temp_list, &fb_data->upd_buf_free_list->list, list) {
		list_del(&plist->list);
		dma_free_writecombine(&pdev->dev, plist->size, plist->virt_addr,
				      plist->phys_addr);
		kfree(plist);
	}
out_dma_fb:
	dma_free_writecombine(&pdev->dev, fb_data->map_size, info->screen_base,
			      fb_data->phys_start);

out_mapregs:
	iounmap(epdc_base);
out_cmap:
	fb_dealloc_cmap(&info->cmap);
out_fbdata:
	kfree(fb_data);
out:
	return ret;
}

static int mxc_epdc_fb_remove(struct platform_device *pdev)
{
	struct update_data_list *plist, *temp_list;
	struct mxc_epdc_fb_data *fb_data = platform_get_drvdata(pdev);

	mxc_epdc_fb_blank(FB_BLANK_POWERDOWN, &fb_data->info);

	flush_workqueue(fb_data->epdc_submit_workqueue);
	destroy_workqueue(fb_data->epdc_submit_workqueue);

	regulator_put(fb_data->display_regulator);
	regulator_put(fb_data->vcom_regulator);

	unregister_framebuffer(&fb_data->info);
	free_irq(fb_data->epdc_irq, fb_data);

	dma_free_writecombine(&pdev->dev, fb_data->working_buffer_size,
				fb_data->working_buffer_virt,
				fb_data->working_buffer_phys);
	if (fb_data->waveform_buffer_virt != NULL)
		dma_free_writecombine(&pdev->dev, fb_data->waveform_buffer_size,
				fb_data->waveform_buffer_virt,
				fb_data->waveform_buffer_phys);
	list_for_each_entry_safe(plist, temp_list, &fb_data->upd_buf_free_list->list, list) {
		list_del(&plist->list);
		dma_free_writecombine(&pdev->dev, plist->size, plist->virt_addr,
				      plist->phys_addr);
		kfree(plist);
	}
#ifdef CONFIG_FB_MXC_EINK_AUTO_UPDATE_MODE
	fb_deferred_io_cleanup(&fb_data->info);
#endif

	dma_free_writecombine(&pdev->dev, fb_data->map_size, fb_data->info.screen_base,
			      fb_data->phys_start);

	if (fb_data->pdata->put_pins)
		fb_data->pdata->put_pins();

	/* Release PxP-related resources */
	if (fb_data->pxp_chan != NULL)
		dma_release_channel(&fb_data->pxp_chan->dma_chan);

	dmaengine_put();

	iounmap(epdc_base);

	fb_dealloc_cmap(&fb_data->info.cmap);

	framebuffer_release(&fb_data->info);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int mxc_epdc_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mxc_epdc_fb_data *data = platform_get_drvdata(pdev);
	int ret;

	ret = mxc_epdc_fb_blank(FB_BLANK_POWERDOWN, &data->info);
	if (ret)
		goto out;

out:
	return ret;
}

static int mxc_epdc_fb_resume(struct platform_device *pdev)
{
	struct mxc_epdc_fb_data *data = platform_get_drvdata(pdev);

	mxc_epdc_fb_blank(FB_BLANK_UNBLANK, &data->info);
	return 0;
}
#else
#define mxc_epdc_fb_suspend	NULL
#define mxc_epdc_fb_resume	NULL
#endif

static struct platform_driver mxc_epdc_fb_driver = {
	.probe = mxc_epdc_fb_probe,
	.remove = mxc_epdc_fb_remove,
	.suspend = mxc_epdc_fb_suspend,
	.resume = mxc_epdc_fb_resume,
	.driver = {
		   .name = "mxc_epdc_fb",
		   .owner = THIS_MODULE,
		   },
};

/* Callback function triggered after PxP receives an EOF interrupt */
static void pxp_dma_done(void *arg)
{
	struct pxp_tx_desc *tx_desc = to_tx_desc(arg);
	struct dma_chan *chan = tx_desc->txd.chan;
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);
	struct mxc_epdc_fb_data *fb_data = pxp_chan->client;

	/* This call will signal wait_for_completion_timeout() in send_buffer_to_pxp */
	complete(&fb_data->pxp_tx_cmpl);
}

/* Function to request PXP DMA channel */
static int pxp_chan_init(struct mxc_epdc_fb_data *fb_data)
{
	dma_cap_mask_t mask;
	struct dma_chan *chan;

	/*
	 * Request a free channel
	 */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_PRIVATE, mask);
	chan = dma_request_channel(mask, NULL, NULL);
	if (!chan) {
		dev_err(fb_data->dev, "Unsuccessfully received channel!!!!\n");
		return -EBUSY;
	}

	dev_dbg(fb_data->dev, "Successfully received channel.\n");

	fb_data->pxp_chan = to_pxp_channel(chan);

	fb_data->pxp_chan->client = fb_data;

	init_completion(&fb_data->pxp_tx_cmpl);

	return 0;
}

/*
 * Function to call PxP DMA driver and send our latest FB update region
 * through the PxP and out to an intermediate buffer.
 * Note: This is a blocking call, so upon return the PxP tx should be complete.
 */
static int pxp_process_update(struct mxc_epdc_fb_data *fb_data,
			      u32 src_width, u32 src_height,
			      struct mxcfb_rect *update_region,
			      int x_start_offs)
{
	dma_cookie_t cookie;
	struct scatterlist *sg = fb_data->sg;
	struct dma_chan *dma_chan;
	struct pxp_tx_desc *desc;
	struct dma_async_tx_descriptor *txd;
	struct pxp_config_data *pxp_conf = &fb_data->pxp_conf;
	struct pxp_proc_data *proc_data = &fb_data->pxp_conf.proc_data;
	int i, ret;
	int length;

	dev_dbg(fb_data->dev, "Starting PxP Send Buffer\n");

	/* First, check to see that we have acquired a PxP Channel object */
	if (fb_data->pxp_chan == NULL) {
		/*
		 * PxP Channel has not yet been created and initialized,
		 * so let's go ahead and try
		 */
		ret = pxp_chan_init(fb_data);
		if (ret) {
			/*
			 * PxP channel init failed, and we can't use the
			 * PxP until the PxP DMA driver has loaded, so we abort
			 */
			dev_err(fb_data->dev, "PxP chan init failed\n");
			return -ENODEV;
		}
	}

	/*
	 * Init completion, so that we
	 * can be properly informed of the completion
	 * of the PxP task when it is done.
	 */
	init_completion(&fb_data->pxp_tx_cmpl);

	dev_dbg(fb_data->dev, "sg[0] = 0x%x, sg[1] = 0x%x\n",
		sg_dma_address(&sg[0]), sg_dma_address(&sg[1]));

	dma_chan = &fb_data->pxp_chan->dma_chan;

	txd = dma_chan->device->device_prep_slave_sg(dma_chan, sg, 2,
						     DMA_TO_DEVICE,
						     DMA_PREP_INTERRUPT);
	if (!txd) {
		dev_err(fb_data->info.device,
			"Error preparing a DMA transaction descriptor.\n");
		return -EIO;
	}

	txd->callback_param = txd;
	txd->callback = pxp_dma_done;

	/*
	 * Configure PxP for processing of new update region
	 * The rest of our config params were set up in
	 * probe() and should not need to be changed.
	 */
	pxp_conf->s0_param.width = src_width;
	pxp_conf->s0_param.height = src_height;
	proc_data->srect.top = update_region->top;
	proc_data->srect.left = update_region->left + x_start_offs;
	proc_data->srect.width = update_region->width;
	proc_data->srect.height = update_region->height;

	/*
	 * Because only YUV/YCbCr image can be scaled, configure
	 * drect equivalent to srect, as such do not perform scaling.
	 */
	proc_data->drect.top = 0;
	proc_data->drect.left = 0;
	proc_data->drect.width = proc_data->srect.width;
	proc_data->drect.height = proc_data->srect.height;

	/* PXP expects rotation in terms of degrees */
	proc_data->rotate = fb_data->epdc_fb_var.rotate * 90;
	if (proc_data->rotate > 270)
		proc_data->rotate = 0;

	pxp_conf->out_param.width = update_region->width;
	pxp_conf->out_param.height = update_region->height;

	desc = to_tx_desc(txd);
	length = desc->len;
	for (i = 0; i < length; i++) {
		if (i == 0) {/* S0 */
			memcpy(&desc->proc_data, proc_data, sizeof(struct pxp_proc_data));
			pxp_conf->s0_param.paddr = sg_dma_address(&sg[0]);
			memcpy(&desc->layer_param.s0_param, &pxp_conf->s0_param,
				sizeof(struct pxp_layer_param));
		} else if (i == 1) {
			pxp_conf->out_param.paddr = sg_dma_address(&sg[1]);
			memcpy(&desc->layer_param.out_param, &pxp_conf->out_param,
				sizeof(struct pxp_layer_param));
		}
		/* TODO: OverLay */

		desc = desc->next;
	}

	/* Submitting our TX starts the PxP processing task */
	cookie = txd->tx_submit(txd);
	dev_dbg(fb_data->info.device, "%d: Submit %p #%d\n", __LINE__, txd,
		cookie);
	if (cookie < 0) {
		dev_err(fb_data->info.device, "Error sending FB through PxP\n");
		return -EIO;
	}

	fb_data->txd = txd;

	/* trigger ePxP */
	dma_async_issue_pending(dma_chan);

	return 0;
}

static int pxp_complete_update(struct mxc_epdc_fb_data *fb_data, u32 *hist_stat)
{
	int ret;
	/*
	 * Wait for completion event, which will be set
	 * through our TX callback function.
	 */
	ret = wait_for_completion_timeout(&fb_data->pxp_tx_cmpl, HZ / 10);
	if (ret <= 0) {
		dev_info(fb_data->info.device,
			 "PxP operation failed due to %s\n",
			 ret < 0 ? "user interrupt" : "timeout");
		dma_release_channel(&fb_data->pxp_chan->dma_chan);
		fb_data->pxp_chan = NULL;
		return ret ? : -ETIMEDOUT;
	}

	*hist_stat = to_tx_desc(fb_data->txd)->hist_status;
	dma_release_channel(&fb_data->pxp_chan->dma_chan);
	fb_data->pxp_chan = NULL;

	dev_dbg(fb_data->dev, "TX completed\n");

	return 0;
}

static int __init mxc_epdc_fb_init(void)
{
	return platform_driver_register(&mxc_epdc_fb_driver);
}
late_initcall(mxc_epdc_fb_init);


static void __exit mxc_epdc_fb_exit(void)
{
	platform_driver_unregister(&mxc_epdc_fb_driver);
}
module_exit(mxc_epdc_fb_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC EPDC framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("fb");
