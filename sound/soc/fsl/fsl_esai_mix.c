// SPDX-License-Identifier: GPL-2.0
// Copyright 2019 NXP
/*
 * Support mix two streams for ESAI
 *
 */
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>

#include "imx-pcm.h"
#include "fsl_esai_client.h"
#include "fsl_esai.h"
#include "fsl_esai_mix.h"

int fsl_esai_mix_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct fsl_esai_mix *mix)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_dmaengine_dai_dma_data *dma_data;
	struct dma_slave_config config;
	int err = 0;

	mix->channels   = params_channels(params);
	mix->word_width = snd_pcm_format_physical_width(params_format(params)) / 8;

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	if (!dma_data)
		return 0;

	/* fills in addr_width and direction */
	err = snd_hwparams_to_dma_slave_config(substream, params, &config);
	if (err)
		return err;

	snd_dmaengine_pcm_set_config_from_dai_data(substream,
						   dma_data,
						   &config);

	return dmaengine_slave_config(mix->chan, &config);
}

int fsl_esai_mix_open(struct snd_pcm_substream *substream, struct fsl_esai_mix *mix)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_dmaengine_dai_dma_data *dma_data;

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	mix->chan = dma_request_slave_channel(rtd->cpu_dai->dev,
					      dma_data->chan_name);

	return 0;
}

int fsl_esai_mix_close(struct snd_pcm_substream *substream,
		       struct fsl_esai_mix *mix)
{
	dmaengine_synchronize(mix->chan);
	dma_release_channel(mix->chan);

	return 0;
}

static void fsl_esai_mix_buffer_from_fe_tx(struct snd_pcm_substream *substream, int size, bool elapse)
{
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	struct fsl_esai_mix *mix = &esai->mix[tx];
	struct fsl_esai_client *client;
	struct fsl_esai_client_dma *client_dma;
	struct snd_soc_dpcm *dpcm;
	unsigned long flags;
	int sample_offset = 0;
	int channel_cnt = 0;
	int i = 0, j = 0;
	int dst_idx;
	u16 *src16;
	u16 *dst16;

	for (j = 0; j < MAX_CLIENT_NUM; j++) {
		mix->fe_substream[j] = NULL;
		mix->client[j] = NULL;
	}

	/* Get the active client */
	spin_lock_irqsave(&rtd->card->dpcm_lock, flags);
	for_each_dpcm_fe(rtd, substream->stream, dpcm) {
		if (dpcm->be != rtd)
			continue;

		mix->fe_substream[i] = snd_soc_dpcm_get_substream(dpcm->fe, substream->stream);
		mix->client[i] = snd_soc_dai_get_drvdata(dpcm->fe->cpu_dai);

		i++;
		if (i >= MAX_CLIENT_NUM)
			break;
	}
	spin_unlock_irqrestore(&rtd->card->dpcm_lock, flags);

	/* mix->word_width == client->word_width */
	/* Mix the internal buffer */
	while (sample_offset * mix->word_width < size) {
		dst16 = (u16 *)(mix->dma_buffer.area + mix->buffer_offset);
		for (channel_cnt = 0; channel_cnt < mix->channels; channel_cnt++)
			*dst16++ = 0;

		for (i = 0;  i < mix->client_cnt;  i++) {
			if (!mix->client[i])
				continue;

			client = mix->client[i];
			client_dma = &client->dma[tx];

			/* check client is active ? */
			if (client_dma->active) {
				src16 = (u16 *)(client_dma->dma_buffer.area + client_dma->buffer_offset);
				dst16 = (u16 *)(mix->dma_buffer.area + mix->buffer_offset);

				/* mix the data and reorder it for correct pin */
				for (j = 0; j < client_dma->channels; j++) {
					dst_idx = client->id + j * mix->sdo_cnt;
					dst16[dst_idx] = *src16++;
				}

				client_dma->buffer_offset += client_dma->channels * client_dma->word_width;
				client_dma->buffer_offset = client_dma->buffer_offset % client_dma->buffer_bytes;
			}
		}

		sample_offset += mix->channels;
		mix->buffer_offset += mix->channels * mix->word_width;
		mix->buffer_offset = mix->buffer_offset % mix->buffer_bytes;
	}

	/* update the pointer of client buffer */
	for (i = 0;  i < mix->client_cnt;  i++) {
		if (elapse && mix->fe_substream[i])
			snd_pcm_period_elapsed(mix->fe_substream[i]);
	}
}

static void fsl_esai_split_buffer_from_be_rx(struct snd_pcm_substream *substream, int size, bool elapse)
{
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	struct fsl_esai_mix *mix = &esai->mix[tx];
	struct fsl_esai_client *client;
	struct fsl_esai_client_dma *client_dma;
	struct snd_soc_dpcm *dpcm;
	unsigned long flags;
	int sample_offset = 0;
	int i = 0, j = 0;
	int src_idx;
	u16 *src16;
	u16 *dst16;

	for (j = 0; j < MAX_CLIENT_NUM; j++) {
		mix->fe_substream[j] = NULL;
		mix->client[j] = NULL;
	}
	/* Get the active client */
	spin_lock_irqsave(&rtd->card->dpcm_lock, flags);
	for_each_dpcm_fe(rtd, substream->stream, dpcm) {
		if (dpcm->be != rtd)
			continue;

		mix->fe_substream[i] = snd_soc_dpcm_get_substream(dpcm->fe, substream->stream);
		mix->client[i] = snd_soc_dai_get_drvdata(dpcm->fe->cpu_dai);

		i++;
		if (i >= MAX_CLIENT_NUM)
			break;
	}
	spin_unlock_irqrestore(&rtd->card->dpcm_lock, flags);

	/* mix->word_width == client->word_width */
	/* split the internal buffer */
	while (sample_offset * mix->word_width < size) {
		for (i = 0;  i < mix->client_cnt;  i++) {
			if (!mix->client[i])
				continue;

			client = mix->client[i];
			client_dma = &client->dma[tx];

			if (client_dma->active) {
				dst16 = (u16 *)(client_dma->dma_buffer.area + client_dma->buffer_offset);
				src16 = (u16 *)(mix->dma_buffer.area + mix->buffer_offset);

				/* split the data to corret client*/
				for (j = 0; j < client_dma->channels; j++) {
					src_idx = client->id + j * mix->sdi_cnt;
					*dst16++ = src16[src_idx];
				}

				client_dma->buffer_offset += client_dma->channels * client_dma->word_width;
				client_dma->buffer_offset = client_dma->buffer_offset % client_dma->buffer_bytes;
			}
		}

		sample_offset += mix->channels;
		mix->buffer_offset += mix->channels * mix->word_width;
		mix->buffer_offset = mix->buffer_offset % mix->buffer_bytes;
	}

	/* update the pointer of client buffer */
	for (i = 0;  i < mix->client_cnt;  i++) {
		if (elapse && mix->fe_substream[i])
			snd_pcm_period_elapsed(mix->fe_substream[i]);
	}
}

/* call back of dma event */
static void fsl_esai_mix_dma_complete(void *arg)
{
	struct snd_pcm_substream *substream = arg;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_esai *esai = snd_soc_dai_get_drvdata(cpu_dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct fsl_esai_mix *mix = &esai->mix[tx];

	if (tx)
		fsl_esai_mix_buffer_from_fe_tx(substream, mix->period_bytes, true);
	else
		fsl_esai_split_buffer_from_be_rx(substream, mix->period_bytes, true);
}

static int fsl_esai_mix_prepare_and_submit(struct snd_pcm_substream *substream,
					   struct fsl_esai_mix *mix)
{
	struct dma_async_tx_descriptor *desc;
	enum dma_transfer_direction direction;
	unsigned long flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

	direction = snd_pcm_substream_to_dma_direction(substream);

	/* ping-pong buffer for mix */
	desc = dmaengine_prep_dma_cyclic(mix->chan,
					 mix->dma_buffer.addr,
					 mix->buffer_bytes,
					 mix->period_bytes,
					 direction, flags);
	if (!desc)
		return -ENOMEM;

	desc->callback = fsl_esai_mix_dma_complete;
	desc->callback_param = substream;
	dmaengine_submit(desc);

	mix->buffer_offset = 0;

	/* Mix the tx buffer */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		fsl_esai_mix_buffer_from_fe_tx(substream, mix->buffer_bytes, false);

	return 0;
}

int fsl_esai_mix_trigger(struct snd_pcm_substream *substream, int cmd,
			 struct fsl_esai_mix *mix)
{
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = fsl_esai_mix_prepare_and_submit(substream, mix);
		if (ret)
			return ret;
		dma_async_issue_pending(mix->chan);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
		dmaengine_terminate_async(mix->chan);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int fsl_esai_mix_probe(struct device *dev, struct fsl_esai_mix *mix_rx, struct fsl_esai_mix *mix_tx)
{
	int ret = 0;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/**
	 * initialize info for mixing
	 * two clients, TX0 pin is for client 0, TX1 pin is for client 1
	 * total supported channel is 4.
	 */
	mix_tx->client_cnt = 2;
	mix_tx->sdo_cnt = 2;
	mix_tx->sdi_cnt = 2;
	mix_tx->channels = 4;
	mix_tx->buffer_bytes = 2048 * mix_tx->client_cnt * 2;
	mix_tx->period_bytes = 2048 * mix_tx->client_cnt;

	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				  dev,
				  IMX_SSI_DMABUF_SIZE * mix_tx->client_cnt,
				  &mix_tx->dma_buffer);
	if (ret)
		return ret;

	/**
	 * initialize info for mixing
	 * two clients, TX0 pin is for client 0, TX1 pin is for client 1
	 * total supported channel is 4.
	 */
	mix_rx->client_cnt = 2;
	mix_rx->sdo_cnt = 2;
	mix_rx->sdi_cnt = 2;
	mix_rx->channels = 4;
	mix_rx->buffer_bytes = 2048 * mix_rx->client_cnt * 2;
	mix_rx->period_bytes = 2048 * mix_rx->client_cnt;

	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				  dev,
				  IMX_SSI_DMABUF_SIZE * mix_rx->client_cnt,
				  &mix_rx->dma_buffer);
	if (ret)
		return ret;

	return ret;
}

int fsl_esai_mix_remove(struct device *dev, struct fsl_esai_mix *mix_rx, struct fsl_esai_mix *mix_tx)
{
	snd_dma_free_pages(&mix_tx->dma_buffer);
	snd_dma_free_pages(&mix_rx->dma_buffer);

	return 0;
}
MODULE_LICENSE("GPL");
