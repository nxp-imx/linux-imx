/*
 * Copyright (C) 2017 NXP
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
#include <linux/pinctrl/consumer.h>
#include "fsl_rpmsg_i2s.h"

struct imx_rpmsg_data {
	struct snd_soc_dai_link dai[1];
	struct snd_soc_card card;
};

static const struct snd_soc_dapm_widget imx_wm8960_dapm_widgets[] = {
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

static int imx_rpmsg_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np;
	struct device_node *codec_np = NULL;
	struct platform_device *cpu_pdev;
	struct imx_rpmsg_data *data;
	struct fsl_rpmsg_i2s         *rpmsg_i2s;
	struct snd_soc_dai_link_component *dlc;
	int ret;

	dlc = devm_kzalloc(&pdev->dev, 3 * sizeof(*dlc), GFP_KERNEL);
	if (!dlc)
		return -ENOMEM;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find rpmsg platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	ret = of_reserved_mem_device_init_by_idx(&pdev->dev, pdev->dev.of_node, 0);
	if (ret)
		dev_warn(&pdev->dev, "no reserved DMA memory for rpmsg device\n");

	rpmsg_i2s = platform_get_drvdata(cpu_pdev);

	if (rpmsg_i2s->codec_in_dt) {
		codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
		if (!codec_np) {
			dev_err(&pdev->dev, "phandle missing or invalid\n");
			ret = -EINVAL;
			goto fail;
		}
	}

	data->dai[0].cpus = &dlc[0];
	data->dai[0].num_cpus = 1;
	data->dai[0].platforms = &dlc[1];
	data->dai[0].num_platforms = 1;
	data->dai[0].codecs = &dlc[2];
	data->dai[0].num_codecs = 1;

	data->dai[0].name = "rpmsg hifi";
	data->dai[0].stream_name = "rpmsg hifi";
	data->dai[0].dai_fmt = SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS;

	if (rpmsg_i2s->codec_wm8960) {
		if (rpmsg_i2s->codec_in_dt) {
			data->dai[0].codecs->of_node = codec_np;
			data->dai[0].codecs->dai_name = "rpmsg-wm8960-hifi";
		} else {
			data->dai[0].codecs->dai_name = "rpmsg-wm8960-hifi";
			data->dai[0].codecs->name = "rpmsg-audio-codec-wm8960";
		}
	}

	if (rpmsg_i2s->codec_pcm512x) {
		if (rpmsg_i2s->codec_in_dt) {
			data->dai[0].codecs->of_node = codec_np;
			data->dai[0].codecs->dai_name = "rpmsg-pcm512x-hifi";
		} else {
			data->dai[0].codecs->dai_name = "rpmsg-pcm512x-hifi";
			data->dai[0].codecs->name = "rpmsg-audio-codec-pcm512x";
		}
	}

	if (rpmsg_i2s->codec_dummy) {
		data->dai[0].codecs->dai_name = "snd-soc-dummy-dai";
		data->dai[0].codecs->name = "snd-soc-dummy";
	}

	if (rpmsg_i2s->codec_ak4497) {
		data->dai[0].codecs->dai_name = "rpmsg-ak4497-aif";
		data->dai[0].codecs->name = "rpmsg-audio-codec-ak4497";
		data->dai[0].dai_fmt = SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS;
	}

	data->dai[0].cpus->dai_name = dev_name(&cpu_pdev->dev);
	data->dai[0].platforms->of_node = cpu_np;
	data->dai[0].playback_only = true;
	data->dai[0].capture_only = true;
	data->card.num_links = 1;
	data->card.dai_link = data->dai;

	if (of_property_read_bool(pdev->dev.of_node, "rpmsg-out"))
		data->dai[0].capture_only = false;

	if (of_property_read_bool(pdev->dev.of_node, "rpmsg-in"))
		data->dai[0].playback_only = false;

	if (data->dai[0].playback_only && data->dai[0].capture_only) {
		dev_err(&pdev->dev, "no enabled rpmsg DAI link\n");
		ret = -EINVAL;
		goto fail;
	}

	data->card.dev = &pdev->dev;
	data->card.owner = THIS_MODULE;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;

	if (rpmsg_i2s->codec_wm8960) {
		ret = snd_soc_of_parse_audio_routing(&data->card,
						"audio-routing");
		if (ret)
			goto fail;

		data->card.dapm_widgets = imx_wm8960_dapm_widgets;
		data->card.num_dapm_widgets =
				ARRAY_SIZE(imx_wm8960_dapm_widgets);
	}

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);
	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
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

fail:
	if (cpu_np)
		of_node_put(cpu_np);
	return ret;
}

static const struct of_device_id imx_rpmsg_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-rpmsg", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_rpmsg_dt_ids);

static struct platform_driver imx_rpmsg_driver = {
	.driver = {
		.name = "imx-audio-rpmsg",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_rpmsg_dt_ids,
	},
	.probe = imx_rpmsg_probe,
};
module_platform_driver(imx_rpmsg_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX rpmsg audio ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-rpmsg");
