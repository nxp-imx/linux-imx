/*
 * Copyright (C) 2004-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_DMA_H__
#define __ASM_ARCH_MXC_DMA_H__

#include <linux/scatterlist.h>
#include <linux/device.h>
#include <linux/dmaengine.h>

#define MXC_DMA_DYNAMIC_CHANNEL   255

#define MXC_DMA_DONE		  0x0
#define MXC_DMA_REQUEST_TIMEOUT   0x1
#define MXC_DMA_TRANSFER_ERROR    0x2

/*
 * This enumerates peripheral types. Used for SDMA.
 */
enum sdma_peripheral_type {
	IMX_DMATYPE_SSI,	/* MCU domain SSI */
	IMX_DMATYPE_SSI_SP,	/* Shared SSI */
	IMX_DMATYPE_MMC,	/* MMC */
	IMX_DMATYPE_SDHC,	/* SDHC */
	IMX_DMATYPE_UART,	/* MCU domain UART */
	IMX_DMATYPE_UART_SP,	/* Shared UART */
	IMX_DMATYPE_FIRI,	/* FIRI */
	IMX_DMATYPE_CSPI,	/* MCU domain CSPI */
	IMX_DMATYPE_CSPI_SP,	/* Shared CSPI */
	IMX_DMATYPE_SIM,	/* SIM */
	IMX_DMATYPE_ATA,	/* ATA */
	IMX_DMATYPE_CCM,	/* CCM */
	IMX_DMATYPE_EXT,	/* External peripheral */
	IMX_DMATYPE_MSHC,	/* Memory Stick Host Controller */
	IMX_DMATYPE_MSHC_SP,	/* Shared Memory Stick Host Controller */
	IMX_DMATYPE_DSP,	/* DSP */
	IMX_DMATYPE_MEMORY,	/* Memory */
	IMX_DMATYPE_FIFO_MEMORY,/* FIFO type Memory */
	IMX_DMATYPE_SPDIF,	/* SPDIF */
	IMX_DMATYPE_IPU_MEMORY,	/* IPU Memory */
	IMX_DMATYPE_ASRC,	/* ASRC */
	IMX_DMATYPE_ESAI,	/* ESAI */
	IMX_DMATYPE_HDMI,
};

enum imx_dma_prio {
	DMA_PRIO_HIGH = 0,
	DMA_PRIO_MEDIUM = 1,
	DMA_PRIO_LOW = 2
};

struct imx_dma_data {
	int dma_request; /* DMA request line */
	int dma_request_p2p;
	enum sdma_peripheral_type peripheral_type;
	int priority;
	void *private;
};

struct imx_pcm_dma_params {
	enum sdma_peripheral_type peripheral_type;
	int dma;
	unsigned long dma_addr;
	int burstsize;
};

static inline int imx_dma_is_ipu(struct dma_chan *chan)
{
	return !strcmp(dev_name(chan->device->dev), "ipu-core");
}

static inline int imx_dma_is_pxp(struct dma_chan *chan)
{
	return !strcmp(dev_name(chan->device->dev), "imx-pxp");
}

static inline int imx_dma_is_general_purpose(struct dma_chan *chan)
{
	return !strcmp(dev_name(chan->device->dev), "imx-sdma") ||
		!strcmp(dev_name(chan->device->dev), "imx-dma");
}

struct mxs_dma_data {
	int chan_irq;
};

#define MXS_DMA_F_APPEND	(1 << 0)
#define MXS_DMA_F_WAIT4END	(1 << 1)
static inline int mxs_dma_is_apbh(struct dma_chan *chan)
{
	return !strcmp(dev_name(chan->device->dev), "mxs-dma-apbh");
}

static inline int mxs_dma_is_apbx(struct dma_chan *chan)
{
	return !strcmp(dev_name(chan->device->dev), "mxs-dma-apbx");
}

void sdma_set_event_pending(struct dma_chan *chan);

#endif
