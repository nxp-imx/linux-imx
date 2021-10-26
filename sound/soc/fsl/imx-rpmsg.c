// SPDX-License-Identifier: GPL-2.0+
// Copyright 2017-2020 NXP

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/extcon-provider.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <sound/simple_card_utils.h>
#include "imx-pcm-rpmsg.h"

struct imx_rpmsg {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	struct asoc_simple_jack hp_jack;
	unsigned int sysclk;
};

static const struct snd_soc_dapm_widget imx_rpmsg_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("Main MIC", NULL),
};
#ifdef CONFIG_EXTCON
struct extcon_dev *rpmsg_edev;
static const unsigned int imx_rpmsg_extcon_cables[] = {
	EXTCON_JACK_LINE_OUT,
	EXTCON_NONE,
};
#endif

static int imx_rpmsg_late_probe(struct snd_soc_card *card)
{
	struct imx_rpmsg *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd = list_first_entry(&card->rtd_list,
							   struct snd_soc_pcm_runtime, list);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct device *dev = card->dev;
	int ret;

	if (data->sysclk) {
		ret = snd_soc_dai_set_sysclk(codec_dai, 0, data->sysclk, SND_SOC_CLOCK_IN);
		if (ret && ret != -ENOTSUPP) {
			dev_err(dev, "failed to set sysclk in %s\n", __func__);
			return ret;
		}
	}

	return 0;
}

static int imx_rpmsg_probe(struct platform_device *pdev)
{
	struct snd_soc_dai_link_component *dlc;
	struct device *dev = pdev->dev.parent;
	/* rpmsg_pdev is the platform device for the rpmsg node that probed us */
	struct platform_device *rpmsg_pdev = to_platform_device(dev);
	struct device_node *np = rpmsg_pdev->dev.of_node;
	struct of_phandle_args args;
	const char *platform_name;
	const char *model_string;
	struct imx_rpmsg *data;
	int ret = 0;

	dlc = devm_kzalloc(&pdev->dev, 3 * sizeof(*dlc), GFP_KERNEL);
	if (!dlc)
		return -ENOMEM;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	ret = of_reserved_mem_device_init_by_idx(&pdev->dev, np, 0);
	if (ret)
		dev_warn(&pdev->dev, "no reserved DMA memory\n");

	data->dai.cpus = &dlc[0];
	data->dai.num_cpus = 1;
	data->dai.platforms = &dlc[1];
	data->dai.num_platforms = 1;
	data->dai.codecs = &dlc[2];
	data->dai.num_codecs = 1;

	data->dai.name = "rpmsg hifi";
	data->dai.stream_name = "rpmsg hifi";
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS;

	/* Optional codec node */
	of_property_read_string(np, "model", &model_string);
	ret = of_parse_phandle_with_fixed_args(np, "audio-codec", 0, 0, &args);
	if (ret) {
		if (of_device_is_compatible(np, "fsl,imx7ulp-rpmsg-audio")) {
			data->dai.codecs->dai_name = "rpmsg-wm8960-hifi";
			data->dai.codecs->name = RPMSG_CODEC_DRV_NAME_WM8960;
		} else if (of_device_is_compatible(np, "fsl,imx8mm-rpmsg-audio") &&
				!strcmp("ak4497-audio", model_string)) {
			data->dai.codecs->dai_name = "rpmsg-ak4497-aif";
			data->dai.codecs->name = RPMSG_CODEC_DRV_NAME_AK4497;
		} else {
			data->dai.codecs->dai_name = "snd-soc-dummy-dai";
			data->dai.codecs->name = "snd-soc-dummy";
		}
	} else {
		struct clk *clk;

		data->dai.codecs->of_node = args.np;
		ret = snd_soc_get_dai_name(&args, &data->dai.codecs->dai_name);
		if (ret) {
			dev_err(&pdev->dev, "Unable to get codec_dai_name\n");
			goto fail;
		}

		clk = devm_get_clk_from_child(&pdev->dev, args.np, NULL);
		if (!IS_ERR(clk))
			data->sysclk = clk_get_rate(clk);
	}

	data->dai.cpus->dai_name = dev_name(&rpmsg_pdev->dev);
	data->dai.platforms->name = IMX_PCM_DRV_NAME;
	if (!of_property_read_string(np, "fsl,platform", &platform_name))
		data->dai.platforms->name = platform_name;

	data->dai.playback_only = true;
	data->dai.capture_only = true;
	data->card.num_links = 1;
	data->card.dai_link = &data->dai;

	if (of_property_read_bool(np, "fsl,rpmsg-out"))
		data->dai.capture_only = false;

	if (of_property_read_bool(np, "fsl,rpmsg-in"))
		data->dai.playback_only = false;

	if (data->dai.playback_only && data->dai.capture_only) {
		dev_err(&pdev->dev, "no enabled rpmsg DAI link\n");
		ret = -EINVAL;
		goto fail;
	}

	data->card.dev = &pdev->dev;
	data->card.owner = THIS_MODULE;
	data->card.dapm_widgets = imx_rpmsg_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_rpmsg_dapm_widgets);
	data->card.late_probe = imx_rpmsg_late_probe;
	/*
	 * Inoder to use common api to get card name and audio routing.
	 * Use parent of_node for this device, revert it after finishing using
	 */
	data->card.dev->of_node = np;

	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;

	if (of_property_read_bool(np, "audio-routing")) {
		ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
		if (ret) {
			dev_err(&pdev->dev, "failed to parse audio-routing: %d\n", ret);
			goto fail;
		}
	}

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);
	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "snd_soc_register_card failed\n");
		goto fail;
	}

#ifdef CONFIG_EXTCON
	rpmsg_edev  = devm_extcon_dev_allocate(&pdev->dev, imx_rpmsg_extcon_cables);
	if (IS_ERR(rpmsg_edev)) {
		dev_err(&pdev->dev, "failed to allocate extcon device\n");
		goto fail;
	}
	ret = devm_extcon_dev_register(&pdev->dev,rpmsg_edev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register extcon device\n");
		goto fail;
	}
	extcon_set_state_sync(rpmsg_edev, EXTCON_JACK_LINE_OUT, 1);
#endif

	data->hp_jack.pin.pin = "Headphone Jack";
	data->hp_jack.pin.mask = SND_JACK_HEADPHONE;
	snd_soc_card_jack_new(&data->card, "Headphone Jack", SND_JACK_HEADPHONE,
			      &data->hp_jack.jack, &data->hp_jack.pin, 1);
	snd_soc_jack_report(&data->hp_jack.jack, SND_JACK_HEADPHONE, SND_JACK_HEADPHONE);
fail:
	pdev->dev.of_node = NULL;
	return ret;
}

static struct platform_driver imx_rpmsg_driver = {
	.driver = {
		.name = "imx-audio-rpmsg",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = imx_rpmsg_probe,
};
module_platform_driver(imx_rpmsg_driver);

MODULE_DESCRIPTION("Freescale SoC Audio RPMSG Machine Driver");
MODULE_AUTHOR("Shengjiu Wang <shengjiu.wang@nxp.com>");
MODULE_ALIAS("platform:imx-audio-rpmsg");
MODULE_LICENSE("GPL v2");
