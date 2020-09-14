// SPDX-License-Identifier: GPL-2.0
// Copyright 2019 NXP
/*
 * Just allocate memory for FE
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

#include "fsl_esai_client.h"
#include "imx-pcm.h"

static struct snd_pcm_hardware fsl_esai_client_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID,
	.buffer_bytes_max = IMX_SSI_DMABUF_SIZE,
	.period_bytes_min = 2048,	/* fix period size, for alignment in FE/BE */
	.period_bytes_max = 2048,
	.periods_min = 8,		/* periods > 4 ( 4 buffer in BE) */
	.periods_max = 255,
	.fifo_size = 0,
};

static int fsl_esai_client_pcm_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_esai_client *client = snd_soc_dai_get_drvdata(cpu_dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	snd_pcm_set_runtime_buffer(substream, &client->dma[tx].dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	client->dma[tx].channels   = params_channels(params);
	client->dma[tx].word_width = snd_pcm_format_physical_width(params_format(params)) / 8;

	return 0;
}

static int fsl_esai_client_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static snd_pcm_uframes_t fsl_esai_client_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_esai_client *client = snd_soc_dai_get_drvdata(cpu_dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	return bytes_to_frames(substream->runtime, client->dma[tx].buffer_offset);
}

static int fsl_esai_client_pcm_open(struct snd_pcm_substream *substream)
{
	int ret;

	snd_soc_set_runtime_hwparams(substream, &fsl_esai_client_pcm_hardware);

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	return 0;
}

int fsl_esai_client_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_esai_client *client = snd_soc_dai_get_drvdata(cpu_dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* These info are needed by esai mix*/
		client->dma[tx].buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
		client->dma[tx].period_bytes = snd_pcm_lib_period_bytes(substream);
		client->dma[tx].buffer_offset = 0;
		client->dma[tx].active = true;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
		client->dma[tx].active = false;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fsl_esai_client_pcm_mmap(struct snd_pcm_substream *substream,
				    struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_wc(substream->pcm->card->dev, vma,
			   runtime->dma_area,
			   runtime->dma_addr,
			   runtime->dma_bytes);
}

static struct snd_pcm_ops fsl_esai_client_pcm_ops = {
	.open		= fsl_esai_client_pcm_open,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= fsl_esai_client_pcm_hw_params,
	.hw_free	= fsl_esai_client_pcm_hw_free,
	.pointer	= fsl_esai_client_pcm_pointer,
	.mmap		= fsl_esai_client_pcm_mmap,
	.trigger        = fsl_esai_client_pcm_trigger,
};

static int fsl_esai_client_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static const struct snd_soc_component_driver fsl_esai_client_pcm_platform = {
	.name           = "fsl_esai_client_pcm",
	.ops		= &fsl_esai_client_pcm_ops,
	.pcm_new	= fsl_esai_client_pcm_new,
};

static const struct snd_soc_component_driver fsl_esai_client_component = {
	.name		= "fsl-esai-client",
};

/* Fix the sample rate/format/channels, for we don't support
 * conversion when do mixing.
 */
#define FSL_ESAI_CLIENT_RATES	SNDRV_PCM_RATE_48000
#define FSL_ESAI_CLIENT_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE)

static struct snd_soc_dai_driver fsl_esai_client_dai_template = {
	.name = "fsl-esai-client-dai",
	.playback = {
		.stream_name	= "CLIENT-Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= FSL_ESAI_CLIENT_RATES,
		.formats	= FSL_ESAI_CLIENT_FORMATS,
	},
	.capture = {
		.stream_name	= "CLIENT-Capture",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates = FSL_ESAI_CLIENT_RATES,
		.formats = FSL_ESAI_CLIENT_FORMATS,
	},
};

static int fsl_esai_client_probe(struct platform_device *pdev)
{
	struct fsl_esai_client *client;
	int ret;

	client = devm_kzalloc(&pdev->dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	memcpy(&client->cpu_dai_drv, &fsl_esai_client_dai_template,
	       sizeof(fsl_esai_client_dai_template));

	platform_set_drvdata(pdev, client);

	ret = of_property_read_u32(pdev->dev.of_node, "fsl,client-id", &client->id);
	if (ret < 0)
		return ret;

	if (client->id == 0) {
		client->cpu_dai_drv.name = "fsl-client0-dai";
		client->cpu_dai_drv.playback.stream_name = "CLIENT0-Playback";
		client->cpu_dai_drv.capture.stream_name = "CLIENT0-Capture";
	}

	if (client->id == 1) {
		client->cpu_dai_drv.name = "fsl-client1-dai";
		client->cpu_dai_drv.playback.stream_name = "CLIENT1-Playback";
		client->cpu_dai_drv.capture.stream_name = "CLIENT1-Capture";
	}

	/* alloc memory for user read data from it. rx. */
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				  &pdev->dev,
				  fsl_esai_client_pcm_hardware.buffer_bytes_max,
				  &client->dma[0].dma_buffer);
	if (ret)
		return ret;

	/* alloc memory for user write data from it. tx. */
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				  &pdev->dev,
				  fsl_esai_client_pcm_hardware.buffer_bytes_max,
				  &client->dma[1].dma_buffer);
	if (ret)
		return ret;

	ret = devm_snd_soc_register_component(&pdev->dev,
					      &fsl_esai_client_component, &client->cpu_dai_drv, 1);
	if (ret < 0)
		return ret;

	return devm_snd_soc_register_component(&pdev->dev, &fsl_esai_client_pcm_platform, NULL, 0);
}

static int fsl_esai_client_remove(struct platform_device *pdev)
{
	struct fsl_esai_client *client = platform_get_drvdata(pdev);

	snd_dma_free_pages(&client->dma[1].dma_buffer);
	snd_dma_free_pages(&client->dma[0].dma_buffer);

	return 0;
}

static const struct of_device_id fsl_esai_client_ids[] = {
	{ .compatible = "fsl,esai-client", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_esai_client_ids);

static struct platform_driver fsl_esai_client_driver = {
	.probe = fsl_esai_client_probe,
	.remove = fsl_esai_client_remove,
	.driver = {
		.name = "fsl-esai-client",
		.of_match_table = fsl_esai_client_ids,
	},
};
module_platform_driver(fsl_esai_client_driver);

MODULE_DESCRIPTION("client cpu dai Interface");
MODULE_ALIAS("platform:fsl-esai-client");
MODULE_LICENSE("GPL");
