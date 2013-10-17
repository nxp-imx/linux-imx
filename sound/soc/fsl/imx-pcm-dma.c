/*
 * imx-pcm-dma-mx2.c  --  ALSA Soc Audio Layer
 *
 * Copyright 2009 Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This code is based on code copyrighted by Freescale,
 * Liam Girdwood, Javier Martin and probably others.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/types.h>
#include <linux/platform_data/dma-imx.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "imx-pcm.h"

static bool filter(struct dma_chan *chan, void *param)
{
	struct snd_dmaengine_dai_dma_data *dma_data = param;

	if (!imx_dma_is_general_purpose(chan))
		return false;

	chan->private = dma_data->filter_data;

	return true;
}

static const struct snd_pcm_hardware imx_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
		   SNDRV_PCM_FMTBIT_S24_LE |
		   SNDRV_PCM_FMTBIT_S20_3LE,
	.rate_min = 8000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = IMX_DEFAULT_DMABUF_SIZE,
	.period_bytes_min = 128,
	.period_bytes_max = 65535, /* Limited by SDMA engine */
	.periods_min = 2,
	.periods_max = 255,
	.fifo_size = 0,
};

static void imx_pcm_dma_set_config_from_dai_data(
	const struct snd_pcm_substream *substream,
	const struct snd_dmaengine_dai_dma_data *dma_data,
	struct dma_slave_config *slave_config)
{
	struct imx_dma_data *filter_data = dma_data->filter_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		slave_config->dst_addr = dma_data->addr;
		slave_config->dst_maxburst = dma_data->maxburst;
		if (dma_data->addr_width != DMA_SLAVE_BUSWIDTH_UNDEFINED)
			slave_config->dst_addr_width = dma_data->addr_width;
	} else {
		slave_config->src_addr = dma_data->addr;
		slave_config->src_maxburst = dma_data->maxburst;
		if (dma_data->addr_width != DMA_SLAVE_BUSWIDTH_UNDEFINED)
			slave_config->src_addr_width = dma_data->addr_width;
	}

	slave_config->slave_id = dma_data->slave_id;

	/*
	 * In dma binding mode, there is no filter_data, so dma_request need to be
	 * set to zero.
	*/
	if (filter_data) {
		slave_config->dma_request0 = filter_data->dma_request0;
		slave_config->dma_request1 = filter_data->dma_request1;
	} else {
		slave_config->dma_request0 = 0;
		slave_config->dma_request1 = 0;
	}
}

static int imx_pcm_dma_prepare_slave_config(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct dma_slave_config *slave_config)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_dmaengine_dai_dma_data *dma_data;
	int ret;

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	ret = snd_hwparams_to_dma_slave_config(substream, params, slave_config);
	if (ret)
		return ret;

	imx_pcm_dma_set_config_from_dai_data(substream, dma_data,
		slave_config);

	return 0;
}

static const struct snd_dmaengine_pcm_config imx_dmaengine_pcm_config = {
	.pcm_hardware = &imx_pcm_hardware,
	.prepare_slave_config = imx_pcm_dma_prepare_slave_config,
	.compat_filter_fn = filter,
	.prealloc_buffer_size = IMX_DEFAULT_DMABUF_SIZE,
};

int imx_pcm_dma_init(struct platform_device *pdev, unsigned int flags, size_t size)
{
	struct snd_dmaengine_pcm_config *config;
	struct snd_pcm_hardware *pcm_hardware;

	config = devm_kzalloc(&pdev->dev,
			sizeof(struct snd_dmaengine_pcm_config), GFP_KERNEL);
	*config = imx_dmaengine_pcm_config;
	if (size)
		config->prealloc_buffer_size = size;

	pcm_hardware = devm_kzalloc(&pdev->dev,
			sizeof(struct snd_pcm_hardware), GFP_KERNEL);
	*pcm_hardware = imx_pcm_hardware;
	if (size)
		pcm_hardware->buffer_bytes_max = size;

	config->pcm_hardware = pcm_hardware;

	return snd_dmaengine_pcm_register(&pdev->dev, config, flags);
}
EXPORT_SYMBOL_GPL(imx_pcm_dma_init);

void imx_pcm_dma_exit(struct platform_device *pdev)
{
	snd_dmaengine_pcm_unregister(&pdev->dev);
}
EXPORT_SYMBOL_GPL(imx_pcm_dma_exit);
