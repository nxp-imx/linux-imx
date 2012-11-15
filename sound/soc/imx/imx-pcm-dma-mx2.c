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
#include <linux/delay.h>
#include <linux/mxc_asrc.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/dma.h>

#include "imx-ssi.h"
#include "imx-pcm.h"

struct asrc_p2p_ops *asrc_pcm_p2p_ops;

void asrc_p2p_hook(struct asrc_p2p_ops *asrc_p2p_ct)
{
	asrc_pcm_p2p_ops = asrc_p2p_ct;
	return ;
}
EXPORT_SYMBOL(asrc_p2p_hook);

static void audio_dma_irq(void *data)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_pcm_runtime_data *iprtd = runtime->private_data;

	iprtd->offset += iprtd->period_bytes;
	iprtd->offset %= iprtd->period_bytes * iprtd->periods;

	snd_pcm_period_elapsed(substream);
}

static bool filter(struct dma_chan *chan, void *param)
{
	struct imx_pcm_runtime_data *iprtd = param;

	if (!imx_dma_is_general_purpose(chan))
		return false;

        chan->private = &iprtd->dma_data;

        return true;
}
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_IMX_HAVE_PLATFORM_IMX_ASRC)
static bool asrc_filter(struct dma_chan *chan, void *param)
{
	struct imx_pcm_runtime_data *iprtd = param;
	if (!imx_dma_is_general_purpose(chan))
		return false;
	chan->private = &iprtd->asrc_dma_data;
	return true;
}
static bool asrc_p2p_filter(struct dma_chan *chan, void *param)
{
	struct imx_pcm_runtime_data *iprtd = param;
	if (!imx_dma_is_general_purpose(chan))
		return false;
    chan->private = &iprtd->asrc_p2p_dma_data;
    return true;
}
static int imx_ssi_asrc_dma_alloc(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_pcm_dma_params *dma_params;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_pcm_runtime_data *iprtd = runtime->private_data;
	struct dma_slave_config slave_config;

	dma_cap_mask_t mask;
	enum dma_slave_buswidth buswidth;
	int ret;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S24_LE:
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		goto error;
	}

	dma_params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	/*config m2p dma channel*/
	iprtd->asrc_dma_data.peripheral_type = IMX_DMATYPE_ASRC;
	iprtd->asrc_dma_data.priority = DMA_PRIO_HIGH;
	iprtd->asrc_dma_data.dma_request =
			iprtd->asrc_pcm_p2p_ops_ko->
				asrc_p2p_get_dma_request(iprtd->asrc_index, 1);
	iprtd->asrc_dma_chan = dma_request_channel(mask, asrc_filter, iprtd);

	if (!iprtd->asrc_dma_chan)
		goto error;

	slave_config.direction = DMA_TO_DEVICE;
	slave_config.dst_addr = iprtd->asrc_pcm_p2p_ops_ko->
					asrc_p2p_per_addr(iprtd->asrc_index, 1);
	slave_config.dst_addr_width = buswidth;
	slave_config.dst_maxburst = dma_params->burstsize * buswidth;

	ret = dmaengine_slave_config(iprtd->asrc_dma_chan, &slave_config);
	if (ret)
		goto error;
	/*config p2p dma channel*/
	iprtd->asrc_p2p_dma_data.peripheral_type = IMX_DMATYPE_ASRC;
	iprtd->asrc_p2p_dma_data.priority = DMA_PRIO_HIGH;
	iprtd->asrc_p2p_dma_data.dma_request =
			iprtd->asrc_pcm_p2p_ops_ko->
				asrc_p2p_get_dma_request(iprtd->asrc_index, 0);
	iprtd->asrc_p2p_dma_data.dma_request_p2p = dma_params->dma;
	iprtd->asrc_p2p_dma_chan =
		dma_request_channel(mask, asrc_p2p_filter, iprtd);
	if (!iprtd->asrc_p2p_dma_chan)
		goto error;

	switch (iprtd->p2p->p2p_width) {
	case ASRC_WIDTH_16_BIT:
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case ASRC_WIDTH_24_BIT:
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		return -EINVAL;
	}

	slave_config.direction = DMA_DEV_TO_DEV;
	slave_config.src_addr = iprtd->asrc_pcm_p2p_ops_ko->
					asrc_p2p_per_addr(iprtd->asrc_index, 0);
	slave_config.src_addr_width = buswidth;
	slave_config.src_maxburst = dma_params->burstsize * buswidth;
	slave_config.dst_addr = dma_params->dma_addr;
	slave_config.dst_addr_width = buswidth;
	slave_config.dst_maxburst = dma_params->burstsize * buswidth;

	ret = dmaengine_slave_config(iprtd->asrc_p2p_dma_chan, &slave_config);
	if (ret)
		goto error;

	return 0;
error:
	if (iprtd->asrc_dma_chan) {
		dma_release_channel(iprtd->asrc_dma_chan);
		iprtd->asrc_dma_chan = NULL;
	}
	if (iprtd->asrc_p2p_dma_chan) {
		dma_release_channel(iprtd->asrc_p2p_dma_chan);
		iprtd->asrc_p2p_dma_chan = NULL;
	}
	return -EINVAL;
}
#endif

static int imx_ssi_dma_alloc(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_pcm_dma_params *dma_params;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_pcm_runtime_data *iprtd = runtime->private_data;
	struct dma_slave_config slave_config;
	dma_cap_mask_t mask;
	enum dma_slave_buswidth buswidth;
	int ret;

	dma_params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	iprtd->dma_data.peripheral_type = dma_params->peripheral_type;
	iprtd->dma_data.priority = DMA_PRIO_HIGH;
	iprtd->dma_data.dma_request = dma_params->dma;

	/* Try to grab a DMA channel */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	iprtd->dma_chan = dma_request_channel(mask, filter, iprtd);
	if (!iprtd->dma_chan)
		return -EINVAL;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S24_LE:
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		return 0;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		slave_config.direction = DMA_TO_DEVICE;
		slave_config.dst_addr = dma_params->dma_addr;
		slave_config.dst_addr_width = buswidth;
		slave_config.dst_maxburst = dma_params->burstsize * buswidth;
	} else {
		slave_config.direction = DMA_FROM_DEVICE;
		slave_config.src_addr = dma_params->dma_addr;
		slave_config.src_addr_width = buswidth;
		slave_config.src_maxburst = dma_params->burstsize * buswidth;
	}

	ret = dmaengine_slave_config(iprtd->dma_chan, &slave_config);
	if (ret)
		return ret;

	return 0;
}

static int snd_imx_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_pcm_runtime_data *iprtd = runtime->private_data;
	unsigned long dma_addr;
	struct dma_chan *chan;
	struct imx_pcm_dma_params *dma_params;
	int ret;

	dma_params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	if (iprtd->asrc_enable) {
		ret = imx_ssi_asrc_dma_alloc(substream, params);
		if (ret)
			return ret;
		chan = iprtd->asrc_p2p_dma_chan;
		iprtd->asrc_p2p_desc =
			chan->device->device_prep_dma_cyclic(chan, 0xffff,
							64,
							64,
							DMA_DEV_TO_DEV);
		if (!iprtd->asrc_p2p_desc) {
			dev_err(&chan->dev->device,
					"cannot prepare slave dma\n");
			return -EINVAL;
		}
		chan = iprtd->asrc_dma_chan;
	} else {
		ret = imx_ssi_dma_alloc(substream, params);
		if (ret)
			return ret;
		chan = iprtd->dma_chan;
	}

	iprtd->size = params_buffer_bytes(params);
	iprtd->periods = params_periods(params);
	iprtd->period_bytes = params_period_bytes(params);
	iprtd->offset = 0;
	iprtd->period_time = HZ / (params_rate(params) /
					params_period_size(params));

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	dma_addr = runtime->dma_addr;

	iprtd->buf = (unsigned int *)substream->dma_buffer.area;

	if (iprtd->asrc_enable) {
		iprtd->asrc_desc =
			chan->device->device_prep_dma_cyclic(chan, dma_addr,
				iprtd->period_bytes * iprtd->periods,
				iprtd->period_bytes,
				substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
				DMA_TO_DEVICE : DMA_FROM_DEVICE);
		if (!iprtd->asrc_desc) {
			dev_err(&chan->dev->device,
					"cannot prepare slave dma\n");
			return -EINVAL;
		}

		iprtd->asrc_desc->callback = audio_dma_irq;
		iprtd->asrc_desc->callback_param = substream;
	} else {
		iprtd->desc = chan->device->device_prep_dma_cyclic(
			chan, dma_addr,
			iprtd->period_bytes * iprtd->periods,
			iprtd->period_bytes,
			substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
			DMA_TO_DEVICE : DMA_FROM_DEVICE);
		if (!iprtd->desc) {
			dev_err(&chan->dev->device,
					"cannot prepare slave dma\n");
			return -EINVAL;
		}

		iprtd->desc->callback = audio_dma_irq;
		iprtd->desc->callback_param = substream;
	}

	return 0;
}

static int snd_imx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_pcm_runtime_data *iprtd = runtime->private_data;

	if (iprtd->asrc_enable) {
		if (iprtd->asrc_dma_chan) {
			dma_release_channel(iprtd->asrc_dma_chan);
			iprtd->asrc_dma_chan = NULL;
		}
		if (iprtd->asrc_p2p_dma_chan) {
			dma_release_channel(iprtd->asrc_p2p_dma_chan);
			iprtd->asrc_p2p_dma_chan = NULL;
		}
		iprtd->dma_chan = NULL;
	} else {
		if (iprtd->dma_chan) {
			dma_release_channel(iprtd->dma_chan);
			iprtd->dma_chan = NULL;
		}
	}

	return 0;
}

static int snd_imx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_pcm_dma_params *dma_params;

	dma_params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	return 0;
}

static int snd_imx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_pcm_runtime_data *iprtd = runtime->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (iprtd->asrc_enable) {
			dmaengine_submit(iprtd->asrc_p2p_desc);
			dmaengine_submit(iprtd->asrc_desc);
			iprtd->asrc_pcm_p2p_ops_ko->
					asrc_p2p_start_conv(iprtd->asrc_index);
			mdelay(1);
		} else {
			dmaengine_submit(iprtd->desc);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (iprtd->asrc_enable) {
			dmaengine_terminate_all(iprtd->asrc_dma_chan);
			dmaengine_terminate_all(iprtd->asrc_p2p_dma_chan);
			iprtd->asrc_pcm_p2p_ops_ko->
					asrc_p2p_stop_conv(iprtd->asrc_index);
		} else {
			dmaengine_terminate_all(iprtd->dma_chan);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t snd_imx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_pcm_runtime_data *iprtd = runtime->private_data;

	pr_debug("%s: %ld %ld\n", __func__, iprtd->offset,
			bytes_to_frames(substream->runtime, iprtd->offset));

	return bytes_to_frames(substream->runtime, iprtd->offset);
}

static struct snd_pcm_hardware snd_imx_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
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

static int snd_imx_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_pcm_runtime_data *iprtd;
	int ret;

	iprtd = kzalloc(sizeof(*iprtd), GFP_KERNEL);
	if (iprtd == NULL)
		return -ENOMEM;
	if (!strcmp(rtd->dai_link->name, "HiFi_ASRC")) {
		iprtd->asrc_enable = true;
		iprtd->p2p =
			(struct asrc_p2p_params *)snd_soc_pcm_get_drvdata(rtd);
		iprtd->asrc_index = -1;
		if (!asrc_pcm_p2p_ops) {
			pr_err("ASRC is not loaded!\n");
			return -EINVAL;
		}
		iprtd->asrc_pcm_p2p_ops_ko = asrc_pcm_p2p_ops;
	}

	runtime->private_data = iprtd;

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		kfree(iprtd);
		return ret;
	}

	if (!strncmp(rtd->cpu_dai->name, "imx-ssi", strlen("imx-ssi")))
		snd_imx_hardware.buffer_bytes_max = IMX_SSI_DMABUF_SIZE;
	else if (!strncmp(rtd->cpu_dai->name, "imx-esai", strlen("imx-esai")))
		snd_imx_hardware.buffer_bytes_max = IMX_ESAI_DMABUF_SIZE;
	else if (!strncmp(rtd->cpu_dai->name, "imx-spdif", strlen("imx-spdif")))
		snd_imx_hardware.buffer_bytes_max = IMX_SPDIF_DMABUF_SIZE;
	else
		snd_imx_hardware.buffer_bytes_max = IMX_DEFAULT_DMABUF_SIZE;

	snd_soc_set_runtime_hwparams(substream, &snd_imx_hardware);

	return 0;
}

static int snd_imx_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_pcm_runtime_data *iprtd = runtime->private_data;

	kfree(iprtd);

	return 0;
}

static struct snd_pcm_ops imx_pcm_ops = {
	.open		= snd_imx_open,
	.close		= snd_imx_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= snd_imx_pcm_hw_params,
	.hw_free	= snd_imx_pcm_hw_free,
	.prepare	= snd_imx_pcm_prepare,
	.trigger	= snd_imx_pcm_trigger,
	.pointer	= snd_imx_pcm_pointer,
	.mmap		= snd_imx_pcm_mmap,
};

static struct snd_soc_platform_driver imx_soc_platform_mx2 = {
	.ops		= &imx_pcm_ops,
	.pcm_new	= imx_pcm_new,
	.pcm_free	= imx_pcm_free,
};

static int __devinit imx_soc_platform_probe(struct platform_device *pdev)
{
	struct imx_ssi *ssi = platform_get_drvdata(pdev);

	if (ssi->dma_params_tx.burstsize == 0
			&& ssi->dma_params_rx.burstsize == 0) {
		ssi->dma_params_tx.burstsize = 6;
		ssi->dma_params_rx.burstsize = 4;
	}

	return snd_soc_register_platform(&pdev->dev, &imx_soc_platform_mx2);
}

static int __devexit imx_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver imx_pcm_driver = {
	.driver = {
			.name = "imx-pcm-audio",
			.owner = THIS_MODULE,
	},
	.probe = imx_soc_platform_probe,
	.remove = __devexit_p(imx_soc_platform_remove),
};

static int __init snd_imx_pcm_init(void)
{
	return platform_driver_register(&imx_pcm_driver);
}
module_init(snd_imx_pcm_init);

static void __exit snd_imx_pcm_exit(void)
{
	platform_driver_unregister(&imx_pcm_driver);
}
module_exit(snd_imx_pcm_exit);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-pcm-audio");
