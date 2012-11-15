/*
 * MXC  ALSA Soc Driver
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 */

/*
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

#ifndef _IMX_PCM_H
#define _IMX_PCM_H

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/mxc_asrc.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/dma.h>

#define IMX_DEFAULT_DMABUF_SIZE		(64 * 1024)
#define IMX_SSI_DMABUF_SIZE			(64 * 1024)
#define IMX_ESAI_DMABUF_SIZE		(256 * 1024)
#define IMX_SPDIF_DMABUF_SIZE		(64 * 1024)

struct imx_pcm_runtime_data {
	int period_bytes;
	int periods;
	int dma;
	unsigned long offset;
	unsigned long size;
	void *buf;
	int period_time;
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *dma_chan;
	struct imx_dma_data dma_data;
	int asrc_enable;
	struct asrc_p2p_ops *asrc_pcm_p2p_ops_ko;

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_IMX_HAVE_PLATFORM_IMX_ASRC)
	enum asrc_pair_index asrc_index;
	struct dma_async_tx_descriptor *asrc_desc;
	struct dma_chan *asrc_dma_chan;
	struct imx_dma_data asrc_dma_data;
	struct dma_async_tx_descriptor *asrc_p2p_desc;
	struct dma_chan *asrc_p2p_dma_chan;
	struct imx_dma_data asrc_p2p_dma_data;
	struct asrc_p2p_params *p2p;
#endif
};
#endif
