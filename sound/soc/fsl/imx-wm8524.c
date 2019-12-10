/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/syscon.h>

struct imx_priv {
	struct snd_soc_dai_link dai[3];
	struct platform_device *pdev;
	struct platform_device *asrc_pdev;
	struct snd_soc_card card;
	struct clk *codec_clk;
	unsigned int clk_frequency;
	u32 asrc_rate;
	u32 asrc_format;
};

static const struct snd_soc_dapm_widget imx_wm8524_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line Out Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct device *dev = card->dev;
	unsigned int fmt;
	int ret = 0;

	fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 2,
					params_physical_width(params));
	if (ret) {
		dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops imx_hifi_ops = {
	.hw_params = imx_hifi_hw_params,
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"Playback",  NULL, "CPU-Playback"},
	{"CPU-Playback",  NULL, "ASRC-Playback"},
};

static int be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_card *card = rtd->card;
	struct imx_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_interval *rate;
	struct snd_mask *mask;

	if (!priv->asrc_pdev)
		return -EINVAL;

	rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	rate->max = priv->asrc_rate;
	rate->min = priv->asrc_rate;

	mask = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	snd_mask_none(mask);
	snd_mask_set(mask, priv->asrc_format);

	return 0;
}

static int imx_wm8524_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd = list_first_entry(
		&card->rtd_list, struct snd_soc_pcm_runtime, list);
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;

	priv->clk_frequency = clk_get_rate(priv->codec_clk);

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, priv->clk_frequency,
							SND_SOC_CLOCK_IN);

	return 0;
}

static int imx_wm8524_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np = NULL;
	struct device_node *asrc_np = NULL;
	struct platform_device *asrc_pdev = NULL;
	struct platform_device *cpu_pdev;
	struct imx_priv *priv;
	struct platform_device *codec_pdev = NULL;
	int ret;
	u32 width;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdev = pdev;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	asrc_np = of_parse_phandle(pdev->dev.of_node, "asrc-controller", 0);
	if (asrc_np) {
		asrc_pdev = of_find_device_by_node(asrc_np);
		priv->asrc_pdev = asrc_pdev;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_pdev = of_find_device_by_node(codec_np);
	if (!codec_pdev || !codec_pdev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	priv->codec_clk = devm_clk_get(&codec_pdev->dev, "mclk");
	if (IS_ERR(priv->codec_clk)) {
		ret = PTR_ERR(priv->codec_clk);
		dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
		goto fail;
	}

	priv->dai[0].name               = "HiFi";
	priv->dai[0].stream_name        = "HiFi";
	priv->dai[0].codec_dai_name     = "wm8524-hifi",
	priv->dai[0].ops                = &imx_hifi_ops,
	priv->dai[0].codec_of_node      = codec_np;
	priv->dai[0].cpu_dai_name = dev_name(&cpu_pdev->dev);
	priv->dai[0].platform_of_node = cpu_np;
	priv->dai[0].playback_only	= 1;

	priv->card.late_probe = imx_wm8524_late_probe;
	priv->card.num_links = 1;
	priv->card.dev = &pdev->dev;
	priv->card.owner = THIS_MODULE;
	priv->card.dapm_widgets = imx_wm8524_dapm_widgets;
	priv->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8524_dapm_widgets);
	priv->card.dai_link = priv->dai;
	priv->card.dapm_routes = audio_map;
	priv->card.num_dapm_routes = 1;

	/*if there is no asrc controller, we only enable one device*/
	if (asrc_pdev) {
		priv->dai[1].name = "HiFi-ASRC-FE";
		priv->dai[1].stream_name = "HiFi-ASRC-FE";
		priv->dai[1].codec_dai_name = "snd-soc-dummy-dai";
		priv->dai[1].codec_name = "snd-soc-dummy";
		priv->dai[1].cpu_of_node    = asrc_np;
		priv->dai[1].platform_of_node   = asrc_np;
		priv->dai[1].dynamic   = 1;
		priv->dai[1].dpcm_playback  = 1;
		priv->dai[1].dpcm_capture   = 0;

		priv->dai[2].name = "HiFi-ASRC-BE";
		priv->dai[2].stream_name = "HiFi-ASRC-BE";
		priv->dai[2].codec_dai_name  = "wm8524-hifi";
		priv->dai[2].codec_of_node   = codec_np;
		priv->dai[2].cpu_of_node     = cpu_np;
		priv->dai[2].platform_name   = "snd-soc-dummy";
		priv->dai[2].no_pcm          = 1;
		priv->dai[2].dpcm_playback  = 1;
		priv->dai[2].dpcm_capture   = 0;
		priv->dai[2].ops = &imx_hifi_ops,
		priv->dai[2].be_hw_params_fixup = be_hw_params_fixup,
		priv->card.num_links = 3;
		priv->card.dai_link = &priv->dai[0];
		priv->card.num_dapm_routes += 1;

		ret = of_property_read_u32(asrc_np, "fsl,asrc-rate",
					   &priv->asrc_rate);
		if (ret) {
			dev_err(&pdev->dev, "failed to get output rate\n");
			ret = -EINVAL;
			goto fail;
		}

		ret = of_property_read_u32(asrc_np, "fsl,asrc-width", &width);
		if (ret) {
			dev_err(&pdev->dev, "failed to get output rate\n");
			ret = -EINVAL;
			goto fail;
		}

		if (width == 24)
			priv->asrc_format = SNDRV_PCM_FORMAT_S24_LE;
		else
			priv->asrc_format = SNDRV_PCM_FORMAT_S16_LE;
	}

	ret = snd_soc_of_parse_card_name(&priv->card, "model");
	if (ret)
		goto fail;

	ret = snd_soc_of_parse_audio_routing(&priv->card, "audio-routing");
	if (ret)
		goto fail;

	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = devm_snd_soc_register_card(&pdev->dev, &priv->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	ret = 0;
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static const struct of_device_id imx_wm8524_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-wm8524", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8524_dt_ids);

static struct platform_driver imx_wm8524_driver = {
	.driver = {
		.name = "imx-wm8524",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm8524_dt_ids,
	},
	.probe = imx_wm8524_probe,
};
module_platform_driver(imx_wm8524_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX WM8524 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm8524");
