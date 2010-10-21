/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#ifndef _PXP_DMA
#define _PXP_DMA

#include <linux/types.h>

#ifndef __KERNEL__
typedef unsigned long dma_addr_t;
typedef unsigned char bool;
#endif

/*  PXP Pixel format definitions */
/*  Four-character-code (FOURCC) */
#define fourcc(a, b, c, d)\
	(((__u32)(a)<<0)|((__u32)(b)<<8)|((__u32)(c)<<16)|((__u32)(d)<<24))

/*!
 * @name PXP Pixel Formats
 *
 * Pixel formats are defined with ASCII FOURCC code. The pixel format codes are
 * the same used by V4L2 API.
 */

/*! @} */
/*! @name RGB Formats */
/*! @{ */
#define PXP_PIX_FMT_RGB332  fourcc('R', 'G', 'B', '1')	/*!<  8  RGB-3-3-2    */
#define PXP_PIX_FMT_RGB555  fourcc('R', 'G', 'B', 'O')	/*!< 16  RGB-5-5-5    */
#define PXP_PIX_FMT_RGB565  fourcc('R', 'G', 'B', 'P')	/*!< 1 6  RGB-5-6-5   */
#define PXP_PIX_FMT_RGB666  fourcc('R', 'G', 'B', '6')	/*!< 18  RGB-6-6-6    */
#define PXP_PIX_FMT_BGR666  fourcc('B', 'G', 'R', '6')	/*!< 18  BGR-6-6-6    */
#define PXP_PIX_FMT_BGR24   fourcc('B', 'G', 'R', '3')	/*!< 24  BGR-8-8-8    */
#define PXP_PIX_FMT_RGB24   fourcc('R', 'G', 'B', '3')	/*!< 24  RGB-8-8-8    */
#define PXP_PIX_FMT_BGR32   fourcc('B', 'G', 'R', '4')	/*!< 32  BGR-8-8-8-8  */
#define PXP_PIX_FMT_BGRA32  fourcc('B', 'G', 'R', 'A')	/*!< 32  BGR-8-8-8-8  */
#define PXP_PIX_FMT_RGB32   fourcc('R', 'G', 'B', '4')	/*!< 32  RGB-8-8-8-8  */
#define PXP_PIX_FMT_RGBA32  fourcc('R', 'G', 'B', 'A')	/*!< 32  RGB-8-8-8-8  */
#define PXP_PIX_FMT_ABGR32  fourcc('A', 'B', 'G', 'R')	/*!< 32  ABGR-8-8-8-8 */
/*! @} */
/*! @name YUV Interleaved Formats */
/*! @{ */
#define PXP_PIX_FMT_YUYV    fourcc('Y', 'U', 'Y', 'V')	/*!< 16 YUV 4:2:2 */
#define PXP_PIX_FMT_UYVY    fourcc('U', 'Y', 'V', 'Y')	/*!< 16 YUV 4:2:2 */
#define PXP_PIX_FMT_Y41P    fourcc('Y', '4', '1', 'P')	/*!< 12 YUV 4:1:1 */
#define PXP_PIX_FMT_YUV444  fourcc('Y', '4', '4', '4')	/*!< 24 YUV 4:4:4 */
/* two planes -- one Y, one Cb + Cr interleaved  */
#define PXP_PIX_FMT_NV12    fourcc('N', 'V', '1', '2')	/* 12  Y/CbCr 4:2:0  */
/*! @} */
/*! @name YUV Planar Formats */
/*! @{ */
#define PXP_PIX_FMT_GREY    fourcc('G', 'R', 'E', 'Y')	/*!< 8  Greyscale */
#define PXP_PIX_FMT_YVU410P fourcc('Y', 'V', 'U', '9')	/*!< 9  YVU 4:1:0 */
#define PXP_PIX_FMT_YUV410P fourcc('Y', 'U', 'V', '9')	/*!< 9  YUV 4:1:0 */
#define PXP_PIX_FMT_YVU420P fourcc('Y', 'V', '1', '2')	/*!< 12 YVU 4:2:0 */
#define PXP_PIX_FMT_YUV420P fourcc('I', '4', '2', '0')	/*!< 12 YUV 4:2:0 */
#define PXP_PIX_FMT_YUV420P2 fourcc('Y', 'U', '1', '2')	/*!< 12 YUV 4:2:0 */
#define PXP_PIX_FMT_YVU422P fourcc('Y', 'V', '1', '6')	/*!< 16 YVU 4:2:2 */
#define PXP_PIX_FMT_YUV422P fourcc('4', '2', '2', 'P')	/*!< 16 YUV 4:2:2 */
/*! @} */

#define PXP_LUT_NONE			0x0
#define PXP_LUT_INVERT			0x1
#define PXP_LUT_BLACK_WHITE		0x2

#define NR_PXP_VIRT_CHANNEL	16

#define PXP_IOC_MAGIC  'P'

#define PXP_IOC_GET_CHAN      _IOR(PXP_IOC_MAGIC, 0, struct pxp_mem_desc)
#define PXP_IOC_PUT_CHAN      _IOW(PXP_IOC_MAGIC, 1, struct pxp_mem_desc)
#define PXP_IOC_CONFIG_CHAN   _IOW(PXP_IOC_MAGIC, 2, struct pxp_mem_desc)
#define PXP_IOC_START_CHAN    _IOW(PXP_IOC_MAGIC, 3, struct pxp_mem_desc)
#define PXP_IOC_GET_PHYMEM    _IOWR(PXP_IOC_MAGIC, 4, struct pxp_mem_desc)
#define PXP_IOC_PUT_PHYMEM    _IOW(PXP_IOC_MAGIC, 5, struct pxp_mem_desc)
#define PXP_IOC_WAIT4CMPLT    _IOWR(PXP_IOC_MAGIC, 6, struct pxp_mem_desc)

/* Order significant! */
enum pxp_channel_status {
	PXP_CHANNEL_FREE,
	PXP_CHANNEL_INITIALIZED,
	PXP_CHANNEL_READY,
};

struct rect {
	int top;		/* Upper left coordinate of rectangle */
	int left;
	int width;
	int height;
};

struct pxp_layer_param {
	unsigned short width;
	unsigned short height;
	unsigned int pixel_fmt;

	/* layers combining parameters
	 * (these are ignored for S0 and output
	 * layers, and only apply for OL layer)
	 */
	bool combine_enable;
	__u32 color_key_enable;
	__u32 color_key;
	bool global_alpha_enable;
	__u8 global_alpha;
	bool local_alpha_enable;

	dma_addr_t paddr;
};

struct pxp_proc_data {
	/* S0 Transformation Info */
	int scaling;
	int hflip;
	int vflip;
	int rotate;
	int yuv;

	/* Source rectangle (srect) defines the sub-rectangle
	 * within S0 to undergo processing.
	 */
	struct rect srect;
	/* Dest rect (drect) defines how to position the processed
	 * source rectangle (after resizing) within the output frame,
	 * whose dimensions are defined in pxp->pxp_conf_state.out_param
	 */
	struct rect drect;

	/* Current S0 configuration */
	__u32 bgcolor;

	/* Output overlay support */
	int overlay_state;

	/* LUT transformation on Y data */
	int lut_transform;
};

struct pxp_config_data {
	struct pxp_layer_param s0_param;
	struct pxp_layer_param ol_param[8];
	struct pxp_layer_param out_param;
	struct pxp_proc_data proc_data;
	int layer_nr;

	/* Users don't touch */
	int chan_id;
};

struct pxp_mem_desc {
	__u32 size;
	dma_addr_t phys_addr;
	__u32 cpu_addr;		/* cpu address to free the dma mem */
	__u32 virt_uaddr;		/* virtual user space address */
};

#ifdef __KERNEL__

struct pxp_tx_desc {
	struct dma_async_tx_descriptor txd;
	struct list_head tx_list;
	struct list_head list;
	int len;
	union {
		struct pxp_layer_param s0_param;
		struct pxp_layer_param out_param;
		struct pxp_layer_param ol_param;
	} layer_param;
	struct pxp_proc_data proc_data;

	u32 hist_status;	/* Histogram output status */

	struct pxp_tx_desc *next;
};

struct pxp_channel {
	struct dma_chan dma_chan;
	dma_cookie_t completed;	/* last completed cookie */
	enum pxp_channel_status status;
	void *client;		/* Only one client per channel */
	unsigned int n_tx_desc;
	struct pxp_tx_desc *desc;	/* allocated tx-descriptors */
	struct list_head active_list;	/* active tx-descriptors */
	struct list_head free_list;	/* free tx-descriptors */
	struct list_head queue;	/* queued tx-descriptors */
	struct list_head list;	/* track queued channel number */
	spinlock_t lock;	/* protects sg[0,1], queue */
	struct mutex chan_mutex;	/* protects status, cookie, free_list */
	int active_buffer;
	unsigned int eof_irq;
	char eof_name[16];	/* EOF IRQ name for request_irq()  */
};

struct pxp_irq_info {
	wait_queue_head_t waitq;
	int irq_pending;
	int hist_status;
};

#define to_tx_desc(tx) container_of(tx, struct pxp_tx_desc, txd)
#define to_pxp_channel(d) container_of(d, struct pxp_channel, dma_chan)

void pxp_txd_ack(struct dma_async_tx_descriptor *txd,
		 struct pxp_channel *pxp_chan);
#endif

#endif
