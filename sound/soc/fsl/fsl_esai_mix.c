// SPDX-License-Identifier: GPL-2.0
// Copyright 2019 NXP
/*
 * Support mix two streams for ESAI
 *
 */
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
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

static int fsl_esai_mix_pointer(struct fsl_esai_mix *mix)
{
	struct dma_tx_state state;
	enum dma_status status;
	unsigned int buf_size;
	unsigned int pos = 0;

	status = dmaengine_tx_status(mix->chan, mix->cookie, &state);
	if (status == DMA_IN_PROGRESS || status == DMA_PAUSED) {
		buf_size = mix->buffer_bytes;
		if (state.residue > 0 && state.residue <= buf_size)
			pos = buf_size - state.residue;
	}

	return pos;
}

static int fsl_esai_tx_avail(struct fsl_esai_mix *mix)
{
	int avail;

	mix->buffer_read_offset = fsl_esai_mix_pointer(mix);

	avail = mix->buffer_bytes - mix->buffer_write_offset + mix->buffer_read_offset;
	if (avail < 0)
		avail += mix->buffer_bytes;
	else if (avail > mix->buffer_bytes)
		avail -=  mix->buffer_bytes;

	return avail;
}

static int fsl_esai_rx_avail(struct fsl_esai_mix *mix)
{
	int avail;

	mix->buffer_write_offset = fsl_esai_mix_pointer(mix);

	avail = mix->buffer_bytes - mix->buffer_read_offset + mix->buffer_write_offset;
	if (avail < 0)
		avail += mix->buffer_bytes;
	else if (avail > mix->buffer_bytes)
		avail = avail -  mix->buffer_bytes;

	return avail;
}

static void fsl_esai_mix_buffer_from_fe_tx(struct snd_pcm_substream *substream, bool elapse)
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
	int client_chn = 0;
	int mix_chn = 0;
	int sdo_cnt = 0;
	int loop_cnt = 0;
	int avail = 0;
	int size = 0;
	int id = 0;
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

	avail = fsl_esai_tx_avail(mix);
	if (avail >= mix->buffer_bytes && elapse)
		dev_err(rtd->cpu_dai->dev, "mix underrun\n");

	while (avail >= mix->period_bytes) {
		size = mix->period_bytes;
		/* mix->word_width == client->word_width */
		/* Mix the internal buffer */
		dst16 = (u16 *)(mix->dma_buffer.area + (mix->buffer_write_offset % mix->buffer_bytes));
		memset(dst16, 0, size);

		for (i = 0;  i < mix->client_cnt;  i++) {
			if (!mix->client[i])
				continue;

			client = mix->client[i];
			client_dma = &client->dma[tx];

			/* check client is active ? */
			if (client_dma->active) {
				sample_offset = 0;
				id = client->id;
				sdo_cnt = mix->sdo_cnt;
				client_chn = client_dma->channels;
				mix_chn = mix->channels;
				loop_cnt = size / mix->word_width / mix_chn;

				src16 = (u16 *)(client_dma->dma_buffer.area + client_dma->buffer_offset);
				dst16 = (u16 *)(mix->dma_buffer.area + (mix->buffer_write_offset % mix->buffer_bytes));
				while (sample_offset < loop_cnt) {

					/* mix the data and reorder it for correct pin */
					for (j = 0; j < client_chn; j++) {
						dst_idx = id + j * sdo_cnt;
						dst16[dst_idx] = *src16++;
					}

					sample_offset++;
					dst16 = dst16 + mix_chn;
				}

				sample_offset = client_dma->buffer_offset + size / mix->client_cnt;
				sample_offset = sample_offset % client_dma->buffer_bytes;
				client_dma->buffer_offset = sample_offset;

				if (elapse && mix->fe_substream[i])
					snd_pcm_period_elapsed(mix->fe_substream[i]);
			}
		}

		mix->buffer_write_offset += size;
		if (mix->buffer_write_offset >= mix->buffer_bytes)
			mix->buffer_write_offset -= mix->buffer_bytes;

		avail -= mix->period_bytes;
	}
}

static void fsl_esai_split_buffer_from_be_rx(struct snd_pcm_substream *substream, bool elapse)
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
	int client_chn = 0;
	int mix_chn = 0;
	int sdi_cnt = 0;
	int loop_cnt = 0;
	int id = 0;
	int size = 0;
	int avail = 0;
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

	avail = fsl_esai_rx_avail(mix);
	if (avail >= mix->buffer_bytes && elapse)
		dev_err(rtd->cpu_dai->dev, "mix overrun\n");

	while (avail >= mix->period_bytes) {
		size = mix->period_bytes;
		/* mix->word_width == client->word_width */
		/* split the internal buffer */
		for (i = 0;  i < mix->client_cnt;  i++) {
			if (!mix->client[i])
				continue;

			client = mix->client[i];
			client_dma = &client->dma[tx];

			if (client_dma->active) {
				sample_offset = 0;
				id = client->id;
				sdi_cnt = mix->sdi_cnt;
				client_chn = client_dma->channels;
				mix_chn = mix->channels;
				loop_cnt = size / mix->word_width / mix_chn;

				dst16 = (u16 *)(client_dma->dma_buffer.area + client_dma->buffer_offset);
				src16 = (u16 *)(mix->dma_buffer.area + (mix->buffer_read_offset % mix->buffer_bytes));
				while (sample_offset < loop_cnt) {

					/* split the data to corret client*/
					for (j = 0; j < client_chn; j++) {
						src_idx = id + j * sdi_cnt;
						*dst16++ = src16[src_idx];
					}

					sample_offset++;
					src16 = src16 + mix_chn;
				}
				client_dma->buffer_offset += size / mix->client_cnt;
				client_dma->buffer_offset = client_dma->buffer_offset % client_dma->buffer_bytes;

				if (elapse && mix->fe_substream[i])
					snd_pcm_period_elapsed(mix->fe_substream[i]);
			}
		}

		mix->buffer_read_offset += size;
		if (mix->buffer_read_offset >= mix->buffer_bytes)
			mix->buffer_read_offset -= mix->buffer_bytes;

		avail -= mix->period_bytes;
	}
}

static void fsl_esai_mix_tx_worker(struct work_struct *work)
{
	struct fsl_esai_mix *mix;

	mix = container_of(work, struct fsl_esai_mix, work);
	fsl_esai_mix_buffer_from_fe_tx(mix->substream, true);
}

static void fsl_esai_mix_rx_worker(struct work_struct *work)
{
	struct fsl_esai_mix *mix;

	mix = container_of(work, struct fsl_esai_mix, work);
	fsl_esai_split_buffer_from_be_rx(mix->substream, true);
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

	mix->substream = substream;

	queue_work(mix->mix_wq, &mix->work);
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
	mix->cookie = dmaengine_submit(desc);

	/* Mix the tx buffer */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mix->buffer_read_offset = 0;
		mix->buffer_write_offset = 0;
		fsl_esai_mix_buffer_from_fe_tx(substream, false);
	} else {
		mix->buffer_read_offset = 0;
		mix->buffer_write_offset = 0;
	}

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
	mix_tx->buffer_bytes = 2048 * mix_tx->client_cnt * 4;
	mix_tx->period_bytes = 2048 * mix_tx->client_cnt;
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				  dev,
				  IMX_SSI_DMABUF_SIZE * mix_tx->client_cnt,
				  &mix_tx->dma_buffer);
	if (ret)
		return ret;

	mix_tx->mix_wq = alloc_ordered_workqueue("esai_mix_tx", WQ_HIGHPRI | WQ_UNBOUND | WQ_FREEZABLE);
	if (IS_ERR(mix_tx->mix_wq))
		dev_err(dev, "failed  create easi mix tx thread\n");

	INIT_WORK(&mix_tx->work, fsl_esai_mix_tx_worker);

	/**
	 * initialize info for mixing
	 * two clients, TX0 pin is for client 0, TX1 pin is for client 1
	 * total supported channel is 4.
	 */
	mix_rx->client_cnt = 2;
	mix_rx->sdo_cnt = 2;
	mix_rx->sdi_cnt = 2;
	mix_rx->channels = 4;
	mix_rx->buffer_bytes = 2048 * mix_rx->client_cnt * 4;
	mix_rx->period_bytes = 2048 * mix_rx->client_cnt;
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				  dev,
				  IMX_SSI_DMABUF_SIZE * mix_rx->client_cnt,
				  &mix_rx->dma_buffer);
	if (ret)
		return ret;

	mix_rx->mix_wq = alloc_ordered_workqueue("esai_mix_rx", WQ_HIGHPRI | WQ_UNBOUND | WQ_FREEZABLE);
	if (IS_ERR(mix_rx->mix_wq))
		dev_err(dev, "failed  create easi mix tx thread\n");

	INIT_WORK(&mix_rx->work, fsl_esai_mix_rx_worker);

	return ret;
}

int fsl_esai_mix_remove(struct device *dev, struct fsl_esai_mix *mix_rx, struct fsl_esai_mix *mix_tx)
{
	destroy_workqueue(mix_rx->mix_wq);
	destroy_workqueue(mix_rx->mix_wq);

	snd_dma_free_pages(&mix_tx->dma_buffer);
	snd_dma_free_pages(&mix_rx->dma_buffer);

	return 0;
}
MODULE_LICENSE("GPL");
