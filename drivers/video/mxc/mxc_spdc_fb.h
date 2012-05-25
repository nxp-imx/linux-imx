/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __MXC_SPDC_FB_H__
#define __MXC_SPDC_FB_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/dmaengine.h>
#include <linux/pxp_dma.h>
#include <linux/mxcfb.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/regulator/driver.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <mach/epdc.h>
#include <mach/dma.h>
#include <linux/platform_device.h>

#include <linux/fsl_devices.h>
#include <linux/mxcfb.h>

/*************************************
*Register addresses
*************************************/
#define SPDC_DISP_TRIGGER		(0x000)
#define SPDC_UPDATA_X_Y			(0x004)
#define SPDC_UPDATE_W_H			(0x008)
#define SPDC_LUT_PARA_UPDATE		(0x00C)
#define SPDC_OPERATE			(0x010)
#define SPDC_PANEL_INIT_SET		(0x014)
#define SPDC_TEMP_INFO			(0x018)
#define SPDC_NEXT_BUF			(0x01C)
#define SPDC_CURRENT_BUF		(0x020)
#define SPDC_PRE_BUF			(0x024)
#define SPDC_CNT_BUF			(0x028)
#define SPDC_LUT_BUF			(0x02C)
#define SPDC_INT_ENABLE			(0x030)
#define SPDC_INT_STA_CLR		(0x034)
#define SPDC_INIT_STA_CLR		(0x038)
#define SPDC_STATUS			(0x03C)
#define SPDC_DISP_VER			(0x040)
#define SPDC_TCON_VER			(0x044)
#define SPDC_SW_GATE_CLK		(0x048)

/*
 * Register field definitions
 */
enum {
	SPDC_DISP_TRIGGER_ENABLE = 0x1,
	SPDC_DISP_TRIGGER_FLASH = 0x10,

	/*SPDC clock gate*/
	SPDC_SW_GATE_CLK_ENABLE = 0x1,

	/* waveform mode mask */
	SPDC_WAV_MODE_MASK = (0x7 << 1),

	/* SPDC interrupt IRQ mask define */
	SPDC_IRQ_FRAME_UPDATE = (0x1 << 0),
	SPDC_IRQ_TCON_INIT = (0x1 << 1),
	SPDC_IRQ_LUT_DOWNLOAD = (0x1 << 2),
	SPDC_IRQ_ERR = (0x1 << 3),
	SPDC_IRQ_ALL_MASK = 0xF,

	/* SPDC interrupt status */
	SPDC_IRQ_STA_FRAME_UPDATE = (0x1 << 0),
	SPDC_IRQ_STA_TCON_INIT = (0x1 << 1),
	SPDC_IRQ_STA_LUT_DOWNLOAD = (0x1 << 2),
	SPDC_IRQ_STA_ERR = (0x1 << 3),
	SPDC_IRQ_STA_ALL_MASK = 0xF,

	/* SPDC update coordinate angle */
	SPDC_UPDATE_W_H_MAX_SIZE = ((0x1 << 12) - 1),
	SPDC_UPDATE_X_Y_MAX_SIZE = ((0x1 << 12) - 1),

	/* SPDC TCON status */
	SPDC_PANEL_STAUTS_BUSY = 0x1,
	SPDC_TCON_STATUS_IDLE = (0x4 << 4),

	/* SPDC EPD Operation mode */
	SPDC_NO_OPERATION = 0,
	SPDC_DISP_REFRESH = 1,
	SPDC_DEEP_REFRESH = 0x2,
	SPDC_DISP_RESET   = 0x4,
	SPDC_SW_TCON_RESET = 0x5,
	SPDC_SW_TCON_RESET_SET = 0x80000000,
	SPDC_FULL_REFRESH = 0x6,

	/* SPDC Concurrency mechanism: ACC */
	SPDC_LUT_MODE_OFFSET = 0xb3,
	SPDC_LUT_ACC_MODE = 0x4,

	/* SPDC waveform */
	SPDC_WAVEFORM_LUT_OFFSET_ADDR = 0x7600,
	SPDC_LUT_PARA_LENTH = 0x100,
	SPDC_LUT_FROM_MEM = 1,
	SPDC_LUT_TO_MEM = 0,

	/* SPDC submit update status */
	SPDC_CONCUR_UPD = 0x4,
	SPDC_QUEUE_UPD = 0x8,
	SPDC_CONCUR_QUEUE = 0xc,
};

#define	SPDC_DRIVER_NAME	"imx_spdc_fb"

/**
 * SPDC EPD waveform display trigger mode
 */
enum {
	SPDC_WAV_MODE_0	= 0,
	SPDC_WAV_MODE_1,
	SPDC_WAV_MODE_2,
	SPDC_WAV_MODE_3,
	SPDC_WAV_MODE_4,
	SPDC_WAV_MODE_5,
	SPDC_WAV_MODE_DEFAULT = SPDC_WAV_MODE_0,
};

/**
 * SPDC controller ip version
 */
struct mxc_epd_disp_version {
	u8 epd_type;
	u8 lut_ver;
	u16 product_id;
};

struct mxc_spdc_version {
	struct mxc_epd_disp_version disp_ver;
	u8 tcon_ver;
};

struct spdc_buffer_addr {
	u32 next_buf_phys_addr;
	u32 cur_buf_phys_addr;
	u32 pre_buf_phys_addr;
	u32 frm_cnt_buf_phys_addr;
	u32 lut_buf_phys_addr;
};

struct partial_refresh_param {
	struct spdc_buffer_addr buf_addr;
	struct mxcfb_rect update_region;
	int temper;
	u32 blocking;
	u32 wave_mode;
	u32 stride;
	u32 flash; /* only for waveform mode7 */
	u32 concur; /* Concurrency mechanism: ACC */
};

/**
 * SPDC lut data
 */
struct mxc_spdc_lut_para {
	u8 lut_data[SPDC_LUT_PARA_LENTH];
	u8 lut_addr[SPDC_LUT_PARA_LENTH];
	bool lut_para_update_sta;
};

#define PORTRAIT	"portrait"
#define LANDSCAPE	"Landscape"
#define RESERVED	"Reserved"
struct mxc_spdc_resolution_map_para {
	u16 resolution;
	u16 res_x;
	u16 res_y;
	char res_name[12];
};

struct update_marker_data {
	struct list_head full_list;
	struct list_head upd_list;
	u32 update_marker;
	struct completion update_completion;
	int lut_num;
	bool collision_test;
	bool waiting;
};

struct update_desc_list {
	struct list_head list;
	struct mxcfb_update_data upd_data;
	u32 spdc_offs;
	u32 spdc_stride;
	struct list_head upd_marker_list;
	u32 update_order;
};

/* This structure represents a list node containing both
 * a memory region allocated as an output buffer for the PxP
 * update processing task, and the update
 * description (mode, region, etc.)
 */
struct update_data_list {
	struct list_head list;
	dma_addr_t phys_addr;/* Pointer to phys address of processed Y buf */
	void *virt_addr;
	struct update_desc_list *update_desc;
	int lut_num;
	u64 collision_mask; /* SPDC cannot support collision detect,
			     * align with EPDC driver struct */
};

typedef struct mxc_spdc_fb_param {
	struct fb_info info;
	struct fb_var_screeninfo spdc_fb_var;

	char fw_str[24];
	u32 pseudo_palette[16];
	struct mxc_spdc_version spdc_ver;
	struct imx_spdc_fb_mode *cur_mode;
	struct imx_spdc_fb_platform_data *pdata;
	struct partial_refresh_param fresh_param;
	int blank;
	bool updates_active;

	u32 fb_offset;
	u32 max_pix_size;
	ssize_t map_size;
	int native_width;
	int native_height;
	int default_bpp;

	dma_addr_t phys_start;
	void *virt_start;
	struct device *dev;

	int num_screens;
	u32 order_cnt;
	int max_num_updates;
	u32 auto_mode;
	u32 upd_scheme;
	u32 operation_mode;
	bool is_deep_fresh;
	struct list_head full_marker_list;
	struct list_head upd_pending_list;
	struct list_head upd_buf_queue;
	struct list_head upd_buf_free_list;
	struct list_head upd_buf_preprocess_list;
	u32 upd_preprocess_num;
	int submit_upd_sta;
	struct update_data_list *cur_update;
	struct mutex queue_mutex;
	struct imx_spdc_panel_init_set panel_set;
	struct mxc_spdc_lut_para lut_para;
	struct mxcfb_waveform_modes wv_modes;

	dma_addr_t phy_next_buf;
	dma_addr_t phy_pre_buf;
	dma_addr_t phy_current_buf;
	dma_addr_t phy_cnt_buf;
	dma_addr_t phy_lut_buf;
	void *virt_next_buf;
	void *virt_current_buf;
	void *virt_pre_buf;
	void *virt_cnt_buf;
	void *virt_lut_buf;
	u32 next_buf_size;
	u32 current_buf_size;
	u32 pre_buf_size;
	u32 cnt_buf_size;
	u32 lut_buf_size;

	dma_addr_t *phys_addr_updbuf;
	void **virt_addr_updbuf;
	u32 upd_buffer_num;
	u32 max_num_buffers;

	/* copy the processed data to next buffer relative region */
	dma_addr_t phys_addr_copybuf;
	void *virt_addr_copybuf;

	int spdc_irq;
	char __iomem *hwp;
	struct clk *spdc_clk_axi;
	struct clk *spdc_clk_pix;
	struct regulator *display_regulator;
	struct regulator *vcom_regulator;
	struct regulator *v3p3_regulator;

	spinlock_t lock;
	int power_state;
	bool powering_down;
	int pwrdown_delay;
	int wait_for_powerdown;
	struct completion powerdown_compl;
	struct mutex power_mutex;
	bool fw_default_load;
	bool hw_ready;
	bool hw_initializing;
	bool waiting_for_idle;
	bool waiting_for_wb;
	struct completion update_res_free;
	struct completion lut_down;
	struct completion init_finish;
	struct completion update_finish;
	struct completion updates_done;
	struct delayed_work spdc_done_work;
	struct workqueue_struct *spdc_submit_workqueue;
	struct work_struct spdc_submit_work;
	struct workqueue_struct *spdc_intr_workqueue;
	struct work_struct spdc_intr_work;

	/* FB elements related to PxP DMA */
	struct completion pxp_tx_cmpl;
	struct pxp_channel *pxp_chan;
	struct pxp_config_data pxp_conf;
	struct dma_async_tx_descriptor *txd;
	dma_cookie_t cookie;
	struct scatterlist sg[2];
	struct mutex pxp_mutex; /* protects access to PxP */
} mxc_spdc_t;

#endif
