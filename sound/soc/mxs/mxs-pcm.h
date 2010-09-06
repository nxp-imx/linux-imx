/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef _MXS_PCM_H
#define _MXS_PCM_H

struct mxs_pcm_dma_params {
	char *name;
	int dma_bus;	/* DMA bus */
	int dma_ch;	/* DMA channel number */
	int irq;	/* DMA interrupt number */
};
struct mxs_runtime_data {
	u32 dma_ch;
	u32 dma_period;
	u32 dma_totsize;
	unsigned long appl_ptr_bytes;
	int format;
	struct mxs_pcm_dma_params *params;
	struct mxs_dma_desc *dma_desc_array[255];
};

extern struct snd_soc_platform mxs_soc_platform;

#endif
