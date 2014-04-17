/*
 * Copyright (C) 2010-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_i2c.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mxc_asrc.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/dmaengine_pcm.h>

#include "fsl_asrc.h"
#include "imx-pcm.h"

static bool filter(struct dma_chan *chan, void *param)
{
	if (!imx_dma_is_general_purpose(chan))
		return false;

	chan->private = param;

	return true;
}

static int asrc_p2p_request_channel(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai     = rtd->cpu_dai;
	struct fsl_asrc_p2p *asrc_p2p   = snd_soc_dai_get_drvdata(cpu_dai);
	enum dma_slave_buswidth buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
	struct snd_dmaengine_dai_dma_data *dma_params_be = NULL;
	struct snd_dmaengine_dai_dma_data *dma_params_fe = NULL;
	struct imx_dma_data *fe_filter_data = NULL;
	struct imx_dma_data *be_filter_data = NULL;

	struct dma_slave_config slave_config;
	dma_cap_mask_t mask;
	struct dma_chan *chan;
	int ret;
	struct snd_soc_dpcm *dpcm;

	/* find the be for this fe stream */
	list_for_each_entry(dpcm, &rtd->dpcm[substream->stream].be_clients, list_be) {
		if (dpcm->fe == rtd) {
			struct snd_soc_pcm_runtime *be = dpcm->be;
			struct snd_soc_dai *dai = be->cpu_dai;
			struct snd_pcm_substream *be_substream;
			be_substream = snd_soc_dpcm_get_substream(be, substream->stream);
			dma_params_be = snd_soc_dai_get_dma_data(dai, be_substream);
			break;
		}
	}

	if (!dma_params_be) {
		dev_err(rtd->card->dev, "can not get be substream\n");
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_params_fe = &asrc_p2p->dma_params_tx;
	else
		dma_params_fe = &asrc_p2p->dma_params_rx;

	fe_filter_data = dma_params_fe->filter_data;
	be_filter_data = dma_params_be->filter_data;

	if (asrc_p2p->p2p_width == 16)
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
	else
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;

	/* reconfig memory to FIFO dma request */
	dma_params_fe->addr = asrc_p2p->asrc_ops.asrc_p2p_per_addr(
						asrc_p2p->asrc_index, 1);
	fe_filter_data->dma_request0 = asrc_p2p->dmarx[asrc_p2p->asrc_index];
	dma_params_fe->maxburst = dma_params_be->maxburst;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_CYCLIC, mask);

	/* config p2p dma channel */
	asrc_p2p->asrc_p2p_dma_data.peripheral_type = IMX_DMATYPE_ASRC;
	asrc_p2p->asrc_p2p_dma_data.priority        = DMA_PRIO_HIGH;
	asrc_p2p->asrc_p2p_dma_data.dma_request1    = asrc_p2p->dmatx[asrc_p2p->asrc_index];
	/* need to get target device's dma dma_addr, burstsize */
	asrc_p2p->asrc_p2p_dma_data.dma_request0    = be_filter_data->dma_request0;

	/* Request channel */
	asrc_p2p->asrc_p2p_dma_chan =
		dma_request_channel(mask, filter, &asrc_p2p->asrc_p2p_dma_data);

	if (!asrc_p2p->asrc_p2p_dma_chan) {
		dev_err(rtd->card->dev, "can not request dma channel\n");
		goto error;
	}
	chan = asrc_p2p->asrc_p2p_dma_chan;

	/*
	 * Buswidth is not used in the sdma for p2p. Here we set the maxburst fix to
	 * twice of dma_params's burstsize.
	 */
	slave_config.direction      = DMA_DEV_TO_DEV;
	slave_config.src_addr       = asrc_p2p->asrc_ops.asrc_p2p_per_addr(asrc_p2p->asrc_index, 0);
	slave_config.src_addr_width = buswidth;
	slave_config.src_maxburst   = dma_params_be->maxburst * 2;
	slave_config.dst_addr       = dma_params_be->addr;
	slave_config.dst_addr_width = buswidth;
	slave_config.dst_maxburst   = dma_params_be->maxburst * 2;
	slave_config.dma_request0   = be_filter_data->dma_request0;
	slave_config.dma_request1   = asrc_p2p->dmatx[asrc_p2p->asrc_index];

	ret = dmaengine_slave_config(asrc_p2p->asrc_p2p_dma_chan,
							&slave_config);
	if (ret) {
		dev_err(rtd->card->dev, "can not config dma channel\n");
		goto error;
	}

	return 0;
error:
	if (asrc_p2p->asrc_p2p_dma_chan) {
		dma_release_channel(asrc_p2p->asrc_p2p_dma_chan);
		asrc_p2p->asrc_p2p_dma_chan = NULL;
	}

	return -EINVAL;
}

static int config_asrc(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai     = rtd->cpu_dai;
	struct fsl_asrc_p2p *asrc_p2p   = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int rate    = params_rate(params);
	unsigned int channel = params_channels(params);
	struct asrc_config config = {0};
	int p2p_word_width = 0;
	int word_width = 0;
	int ret = 0;
	if ((channel != 2) && (channel != 4) && (channel != 6)) {
		dev_err(cpu_dai->dev, "param channel is not correct\n");
		return -EINVAL;
	}

	ret = asrc_p2p->asrc_ops.asrc_p2p_req_pair(channel, &asrc_p2p->asrc_index);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "Fail to request asrc pair\n");
		return -EINVAL;
	}

	if (asrc_p2p->p2p_width == 16)
		p2p_word_width = ASRC_WIDTH_16_BIT;
	else
		p2p_word_width = ASRC_WIDTH_24_BIT;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_U16:
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		word_width = ASRC_WIDTH_16_BIT;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S20_3BE:
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_3BE:
	case SNDRV_PCM_FORMAT_S24_BE:
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_U24_BE:
	case SNDRV_PCM_FORMAT_U24_LE:
	case SNDRV_PCM_FORMAT_U24_3BE:
	case SNDRV_PCM_FORMAT_U24_3LE:
		word_width =  ASRC_WIDTH_24_BIT;
		break;
	case SNDRV_PCM_FORMAT_S8:
	case SNDRV_PCM_FORMAT_U8:
	case SNDRV_PCM_FORMAT_S32:
	case SNDRV_PCM_FORMAT_U32:
	default:
		dev_err(cpu_dai->dev, "Format is not support!\n");
		return -EINVAL;
	}

	config.input_word_width   = word_width;
	config.output_word_width  = p2p_word_width;
	config.pair               = asrc_p2p->asrc_index;
	config.channel_num        = channel;
	config.input_sample_rate  = rate;
	config.output_sample_rate = asrc_p2p->p2p_rate;
	config.inclk              = INCLK_NONE;

	switch (asrc_p2p->per_dev) {
	case SSI1:
		config.outclk    = OUTCLK_SSI1_TX;
		break;
	case SSI2:
		config.outclk    = OUTCLK_SSI2_TX;
		break;
	case SSI3:
		config.outclk    = OUTCLK_SSI3_TX;
		break;
	case ESAI:
		config.outclk    = OUTCLK_ESAI_TX;
		break;
	default:
		dev_err(cpu_dai->dev, "peripheral device is not correct\n");
		return -EINVAL;
	}

	ret = asrc_p2p->asrc_ops.asrc_p2p_config_pair(&config);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "Fail to config asrc\n");
		return ret;
	}

	return 0;
}

static int fsl_asrc_p2p_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *cpu_dai)
{
	int ret = 0;

	ret = config_asrc(substream, params);
	if (ret < 0)
		return ret;

	return asrc_p2p_request_channel(substream);
}

static int fsl_asrc_p2p_hw_free(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *cpu_dai)
{
	struct fsl_asrc_p2p *asrc_p2p = snd_soc_dai_get_drvdata(cpu_dai);

	if (asrc_p2p->asrc_p2p_dma_chan) {
		/* Release p2p dma resource */
		dma_release_channel(asrc_p2p->asrc_p2p_dma_chan);
		asrc_p2p->asrc_p2p_dma_chan = NULL;
	}

	if (asrc_p2p->asrc_index != -1) {
		asrc_p2p->asrc_ops.asrc_p2p_release_pair(asrc_p2p->asrc_index);
		asrc_p2p->asrc_ops.asrc_p2p_finish_conv(asrc_p2p->asrc_index);
	}
	asrc_p2p->asrc_index = -1;

	return 0;
}

static int fsl_asrc_dma_prepare_and_submit(struct snd_pcm_substream *substream,
					struct fsl_asrc_p2p *asrc_p2p)
{
	struct dma_async_tx_descriptor *desc = asrc_p2p->asrc_p2p_desc;
	struct dma_chan *chan = asrc_p2p->asrc_p2p_dma_chan;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;

	desc = dmaengine_prep_dma_cyclic(chan, 0xffff, 64, 64, DMA_DEV_TO_DEV, 0);
	if (!desc) {
		dev_err(dev, "failed to prepare slave dma\n");
		return -EINVAL;
	}

	dmaengine_submit(desc);

	return 0;
}

static int fsl_asrc_p2p_trigger(struct snd_pcm_substream *substream, int cmd,
			    struct snd_soc_dai *cpu_dai)
{
	struct fsl_asrc_p2p *asrc_p2p = snd_soc_dai_get_drvdata(cpu_dai);
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = fsl_asrc_dma_prepare_and_submit(substream, asrc_p2p);
		if (ret)
			return ret;
		dma_async_issue_pending(asrc_p2p->asrc_p2p_dma_chan);
		asrc_p2p->asrc_ops.asrc_p2p_start_conv(asrc_p2p->asrc_index);

		/* Output enough data to content the DMA burstsize of BE */
		mdelay(1);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
		dmaengine_terminate_all(asrc_p2p->asrc_p2p_dma_chan);
		asrc_p2p->asrc_ops.asrc_p2p_stop_conv(asrc_p2p->asrc_index);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fsl_asrc_p2p_startup(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *cpu_dai)
{
	struct fsl_asrc_p2p *asrc_p2p   = snd_soc_dai_get_drvdata(cpu_dai);

	asrc_p2p->substream[substream->stream] = substream;
	return 0;
}

static void fsl_asrc_p2p_shutdown(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *cpu_dai)
{
	struct fsl_asrc_p2p *asrc_p2p   = snd_soc_dai_get_drvdata(cpu_dai);

	asrc_p2p->substream[substream->stream] = NULL;
}

#define IMX_ASRC_RATES  SNDRV_PCM_RATE_8000_192000

#define IMX_ASRC_FORMATS \
	(SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S16_LE | \
	SNDRV_PCM_FORMAT_S20_3LE)

static struct snd_soc_dai_ops fsl_asrc_p2p_dai_ops = {
	.startup      = fsl_asrc_p2p_startup,
	.shutdown     = fsl_asrc_p2p_shutdown,
	.trigger      = fsl_asrc_p2p_trigger,
	.hw_params    = fsl_asrc_p2p_hw_params,
	.hw_free      = fsl_asrc_p2p_hw_free,
};

static int fsl_asrc_p2p_dai_probe(struct snd_soc_dai *dai)
{
	struct fsl_asrc_p2p *asrc_p2p = snd_soc_dai_get_drvdata(dai);

	dai->playback_dma_data = &asrc_p2p->dma_params_tx;
	dai->capture_dma_data = &asrc_p2p->dma_params_rx;

	return 0;
}

static struct snd_soc_dai_driver fsl_asrc_p2p_dai = {
	.probe = fsl_asrc_p2p_dai_probe,
	.playback = {
		.stream_name = "asrc-Playback",
		.channels_min = 1,
		.channels_max = 10,
		.rates = IMX_ASRC_RATES,
		.formats = IMX_ASRC_FORMATS,
	},
	.capture  = {
		.stream_name = "asrc-Capture",
		.channels_min = 1,
		.channels_max = 4,
		.rates = IMX_ASRC_RATES,
		.formats = IMX_ASRC_FORMATS,
	},
	.ops      = &fsl_asrc_p2p_dai_ops,
};

static const struct snd_soc_component_driver fsl_asrc_p2p_component = {
	.name		= "fsl-asrc-p2p",
};

static bool fsl_asrc_p2p_check_xrun(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_dmaengine_dai_dma_data *dma_params_be = NULL;
	struct snd_pcm_substream *be_substream;
	struct snd_soc_dpcm *dpcm;
	int ret = 0;

	/* find the be for this fe stream */
	list_for_each_entry(dpcm, &rtd->dpcm[substream->stream].be_clients, list_be) {
		struct snd_soc_pcm_runtime *be = dpcm->be;
		struct snd_soc_dai *dai = be->cpu_dai;

		if (dpcm->fe != rtd)
			continue;

		be_substream = snd_soc_dpcm_get_substream(be, substream->stream);
		dma_params_be = snd_soc_dai_get_dma_data(dai, be_substream);
		if (dma_params_be->check_xrun && dma_params_be->check_xrun(be_substream))
			ret = 1;
	}

	return ret;
}

static int stop_lock_stream(struct snd_pcm_substream *substream)
{
	if (substream) {
		snd_pcm_stream_lock_irq(substream);
		if (substream->runtime->status->state == SNDRV_PCM_STATE_RUNNING)
			substream->ops->trigger(substream, SNDRV_PCM_TRIGGER_STOP);
	}
	return 0;
}

static int start_unlock_stream(struct snd_pcm_substream *substream)
{
	if (substream) {
		if (substream->runtime->status->state == SNDRV_PCM_STATE_RUNNING)
			substream->ops->trigger(substream, SNDRV_PCM_TRIGGER_START);
		snd_pcm_stream_unlock_irq(substream);
	}
	return 0;
}

static void fsl_asrc_p2p_reset(struct snd_pcm_substream *substream, bool stop)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_asrc_p2p *asrc_p2p = snd_soc_dai_get_drvdata(cpu_dai);
	struct snd_dmaengine_dai_dma_data *dma_params_be = NULL;
	struct snd_soc_dpcm *dpcm;
	struct snd_pcm_substream *be_substream;

	if (stop) {
		stop_lock_stream(asrc_p2p->substream[0]);
		stop_lock_stream(asrc_p2p->substream[1]);
	}

	/* find the be for this fe stream */
	list_for_each_entry(dpcm, &rtd->dpcm[substream->stream].be_clients, list_be) {
		struct snd_soc_pcm_runtime *be = dpcm->be;
		struct snd_soc_dai *dai = be->cpu_dai;

		if (dpcm->fe != rtd)
			continue;

		be_substream = snd_soc_dpcm_get_substream(be, substream->stream);
		dma_params_be = snd_soc_dai_get_dma_data(dai, be_substream);
		dma_params_be->device_reset(be_substream, 0);
		break;
	}

	if (stop) {
		start_unlock_stream(asrc_p2p->substream[1]);
		start_unlock_stream(asrc_p2p->substream[0]);
	}
}


/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int fsl_asrc_p2p_probe(struct platform_device *pdev)
{
	struct fsl_asrc_p2p *asrc_p2p;
	struct device_node *np = pdev->dev.of_node;
	const char *p;
	const uint32_t *iprop_rate, *iprop_width;
	int ret = 0;

	if (!of_device_is_available(np)) {
		dev_err(&pdev->dev, "There is no device node\n");
		return -ENODEV;
	}

	asrc_p2p = devm_kzalloc(&pdev->dev, sizeof(struct fsl_asrc_p2p), GFP_KERNEL);
	if (!asrc_p2p) {
		dev_err(&pdev->dev, "can not alloc memory\n");
		return -ENOMEM;
	}
	asrc_p2p->asrc_ops.asrc_p2p_start_conv      = asrc_start_conv;
	asrc_p2p->asrc_ops.asrc_p2p_stop_conv       = asrc_stop_conv;
	asrc_p2p->asrc_ops.asrc_p2p_per_addr        = asrc_get_per_addr;
	asrc_p2p->asrc_ops.asrc_p2p_req_pair        = asrc_req_pair;
	asrc_p2p->asrc_ops.asrc_p2p_config_pair     = asrc_config_pair;
	asrc_p2p->asrc_ops.asrc_p2p_release_pair    = asrc_release_pair;
	asrc_p2p->asrc_ops.asrc_p2p_finish_conv     = asrc_finish_conv;

	asrc_p2p->asrc_index = -1;

	iprop_rate = of_get_property(np, "fsl,p2p-rate", NULL);
	if (iprop_rate)
		asrc_p2p->p2p_rate = be32_to_cpup(iprop_rate);
	else {
		dev_err(&pdev->dev, "There is no p2p-rate in dts\n");
		return -EINVAL;
	}
	iprop_width = of_get_property(np, "fsl,p2p-width", NULL);
	if (iprop_width)
		asrc_p2p->p2p_width = be32_to_cpup(iprop_width);

	if (asrc_p2p->p2p_width != 16 && asrc_p2p->p2p_width != 24) {
		dev_err(&pdev->dev, "p2p_width is not acceptable\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np,
			"fsl,asrc-dma-tx-events", asrc_p2p->dmatx, 3);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get fsl,asrc-dma-tx-events.\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np,
			"fsl,asrc-dma-rx-events", asrc_p2p->dmarx, 3);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get fsl,asrc-dma-rx-events.\n");
		return -EINVAL;
	}

	asrc_p2p->filter_data_tx.peripheral_type = IMX_DMATYPE_ASRC;
	asrc_p2p->filter_data_rx.peripheral_type = IMX_DMATYPE_ASRC;

	asrc_p2p->dma_params_tx.filter_data = &asrc_p2p->filter_data_tx;
	asrc_p2p->dma_params_rx.filter_data = &asrc_p2p->filter_data_rx;
	asrc_p2p->dma_params_tx.check_xrun = fsl_asrc_p2p_check_xrun;
	asrc_p2p->dma_params_rx.check_xrun = fsl_asrc_p2p_check_xrun;
	asrc_p2p->dma_params_tx.device_reset = fsl_asrc_p2p_reset;
	asrc_p2p->dma_params_rx.device_reset = fsl_asrc_p2p_reset;

	platform_set_drvdata(pdev, asrc_p2p);

	p = strrchr(np->full_name, '/') + 1;
	strcpy(asrc_p2p->name, p);
	fsl_asrc_p2p_dai.name = asrc_p2p->name;

	ret = snd_soc_register_component(&pdev->dev, &fsl_asrc_p2p_component,
					 &fsl_asrc_p2p_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "register DAI failed\n");
		goto failed_register;
	}

	asrc_p2p->soc_platform_pdev = platform_device_register_simple(
					"imx-pcm-asrc", -1, NULL, 0);
	if (IS_ERR(asrc_p2p->soc_platform_pdev)) {
		ret = PTR_ERR(asrc_p2p->soc_platform_pdev);
		goto failed_pdev_alloc;
	}

	ret = imx_pcm_dma_init(asrc_p2p->soc_platform_pdev, SND_DMAENGINE_PCM_FLAG_NO_RESIDUE |
				     SND_DMAENGINE_PCM_FLAG_NO_DT |
				     SND_DMAENGINE_PCM_FLAG_COMPAT,
				     IMX_ASRC_DMABUF_SIZE);
	if (ret) {
		dev_err(&pdev->dev, "init pcm dma failed\n");
		goto failed_pcm_init;
	}

	return 0;

failed_pcm_init:
	platform_device_unregister(asrc_p2p->soc_platform_pdev);
failed_pdev_alloc:
	snd_soc_unregister_component(&pdev->dev);
failed_register:

	return ret;
}

static int fsl_asrc_p2p_remove(struct platform_device *pdev)
{
	struct fsl_asrc_p2p *asrc_p2p = platform_get_drvdata(pdev);

	imx_pcm_dma_exit(asrc_p2p->soc_platform_pdev);
	platform_device_unregister(asrc_p2p->soc_platform_pdev);
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static const struct of_device_id fsl_asrc_p2p_dt_ids[] = {
	{ .compatible = "fsl,imx6q-asrc-p2p", },
	{ /* sentinel */ }
};

static struct platform_driver fsl_asrc_p2p_driver = {
	.probe = fsl_asrc_p2p_probe,
	.remove = fsl_asrc_p2p_remove,
	.driver = {
		.name = "fsl-asrc-p2p",
		.owner = THIS_MODULE,
		.of_match_table = fsl_asrc_p2p_dt_ids,
	},
};
module_platform_driver(fsl_asrc_p2p_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX ASoC ASRC P2P driver");
MODULE_ALIAS("platform:fsl-asrc-p2p");
MODULE_LICENSE("GPL");
