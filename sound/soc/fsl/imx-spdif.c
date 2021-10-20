// SPDX-License-Identifier: GPL-2.0+
//
// Copyright (C) 2013 Freescale Semiconductor, Inc.

#include <linux/module.h>
#include <linux/of_platform.h>
#include <sound/soc.h>
#include "fsl_spdif.h"

#define SUPPORT_RATE_NUM 10

struct imx_spdif_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	u32 support_rates[SUPPORT_RATE_NUM];
	u32 support_rates_num;
};

#define CLK_8K_FREQ    24576000
#define CLK_11K_FREQ   22579200

static int imx_spdif_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_spdif_data *data = snd_soc_card_get_drvdata(card);
	static struct snd_pcm_hw_constraint_list constraint_rates;
	int ret;

	if (!data->support_rates_num)
		return 0;

	constraint_rates.list = data->support_rates;
	constraint_rates.count = data->support_rates_num;

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &constraint_rates);
	if (ret)
		return ret;

	return 0;
}

static int imx_spdif_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct device *dev = rtd->card->dev;
	int ret = 0;
	u64 rate = params_rate(params);
	unsigned int freq;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		freq = do_div(rate, 8000) ? CLK_11K_FREQ : CLK_8K_FREQ;
		ret = snd_soc_dai_set_sysclk(cpu_dai, STC_TXCLK_SPDIF_ROOT,
			freq, SND_SOC_CLOCK_OUT);
		if (ret)
			dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
	}

	return ret;
}

static struct snd_soc_ops imx_spdif_ops = {
	.startup = imx_spdif_startup,
	.hw_params = imx_spdif_hw_params,
};

static int imx_spdif_audio_probe(struct platform_device *pdev)
{
	struct device_node *spdif_np, *np = pdev->dev.of_node;
	struct imx_spdif_data *data;
	struct snd_soc_dai_link_component *comp;
	int ret = 0, i;

	spdif_np = of_parse_phandle(np, "spdif-controller", 0);
	if (!spdif_np) {
		dev_err(&pdev->dev, "failed to find spdif-controller\n");
		ret = -EINVAL;
		goto end;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	comp = devm_kzalloc(&pdev->dev, 3 * sizeof(*comp), GFP_KERNEL);
	if (!data || !comp) {
		ret = -ENOMEM;
		goto end;
	}

	data->dai.cpus		= &comp[0];
	data->dai.codecs	= &comp[1];
	data->dai.platforms	= &comp[2];

	data->dai.num_cpus	= 1;
	data->dai.num_codecs	= 1;
	data->dai.num_platforms	= 1;

	data->dai.name = "S/PDIF PCM";
	data->dai.stream_name = "S/PDIF PCM";
	data->dai.codecs->dai_name = "snd-soc-dummy-dai";
	data->dai.codecs->name = "snd-soc-dummy";
	data->dai.cpus->of_node = spdif_np;
	data->dai.platforms->of_node = spdif_np;
	data->dai.playback_only = true;
	data->dai.capture_only = true;
	data->dai.ops = &imx_spdif_ops;

	if (of_property_read_bool(np, "spdif-out"))
		data->dai.capture_only = false;

	if (of_property_read_bool(np, "spdif-in"))
		data->dai.playback_only = false;

	if (data->dai.playback_only && data->dai.capture_only) {
		dev_err(&pdev->dev, "no enabled S/PDIF DAI link\n");
		goto end;
	}

	for (i = 0; i < SUPPORT_RATE_NUM; i++) {
		ret = of_property_read_u32_index(pdev->dev.of_node,
						 "fsl,constraint-rate",
						 i, &data->support_rates[i]);
		if (!ret)
			data->support_rates_num = i + 1;
		else
			break;
	}

	data->card.dev = &pdev->dev;
	data->card.dai_link = &data->dai;
	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;

	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto end;

	snd_soc_card_set_drvdata(&data->card, data);
	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret && ret != -EPROBE_DEFER)
		dev_err(&pdev->dev, "snd_soc_register_card failed: %d\n", ret);

end:
	of_node_put(spdif_np);

	return ret;
}

static const struct of_device_id imx_spdif_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-spdif", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_spdif_dt_ids);

static struct platform_driver imx_spdif_driver = {
	.driver = {
		.name = "imx-spdif",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_spdif_dt_ids,
	},
	.probe = imx_spdif_audio_probe,
};

module_platform_driver(imx_spdif_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX S/PDIF machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-spdif");
