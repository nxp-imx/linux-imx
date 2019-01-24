/*
 * ASoC Driver for Google's AIY Edge TPU dev board
 *
 * Author: June Tate-Gans <jtgans@google.com>
 *         Copyright 2018
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/rt5645.h"
#include "../fsl/fsl_sai.h"

#define PLATFORM_CLOCK 2457600
static unsigned long codec_clock = PLATFORM_CLOCK;

static struct snd_soc_jack headset_jack;

static struct snd_soc_jack_pin headset_jack_pin = {
	.pin = "Headphone Jack",
	.mask = 0xFFFFF,
	.invert = 0
};

static int card_init(struct snd_soc_pcm_runtime *rtd) {
	int ret;

	rt5645_sel_asrc_clk_src(rtd->codec,
				RT5645_DA_STEREO_FILTER |
				RT5645_AD_STEREO_FILTER |
				RT5645_DA_MONO_L_FILTER |
				RT5645_DA_MONO_R_FILTER,
				RT5645_CLK_SEL_I2S1_ASRC);

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, RT5645_SCLK_S_MCLK,
				     codec_clock, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(rtd->card->dev, "can't set sysclk: %d\n", ret);
		return ret;
	}

	ret = snd_soc_card_jack_new(rtd->card, "Headphone Jack",
				    SND_JACK_HEADSET,
				    &headset_jack, &headset_jack_pin, 1);
	if (ret < 0) {
		dev_err(rtd->card->dev, "can't add headphone jack: %d\n", ret);
		return ret;
	}

	return rt5645_set_jack_detect(rtd->codec, &headset_jack, NULL, NULL);
}

static int hw_params(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params) {
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int freq = params_rate(params) * 512;

	/* set codec PLL source to the 24.576MHz (MCLK) platform clock */
	ret = snd_soc_dai_set_pll(rtd->codec_dai, 0, RT5645_PLL1_S_MCLK,
				  codec_clock, freq);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec pll: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, RT5645_SCLK_S_PLL1, freq,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec sysclk in: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, RT5645_SCLK_S_PLL1, freq,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec sysclk out: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(rtd->cpu_dai, FSL_SAI_CLK_MAST1, freq,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set cpu sysclk out: %d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops ops = {
	.hw_params = hw_params,
};

static struct snd_soc_dai_link card_dai[] = {
	{
		.name = "rt5645",
		.stream_name = "Google AIY Edge TPU HiFi",
		.codec_dai_name = "rt5645-aif1",
		.dai_fmt = SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS,
		.ops = &ops,
		.init = card_init
	},
};

static const struct snd_soc_dapm_widget card_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_MIC("Headphone Mic", NULL),
};

static const struct snd_soc_dapm_route audio_routes[] = {
	{"AIF1 Capture", NULL, "CPU-Capture"},
	{"micbias1", NULL, "Headphone Mic"},
	{"IN1P", NULL, "micbias1"},
	{"Speaker", NULL, "SPOL"},
	{"Speaker", NULL, "SPOR"},
	{"DMIC L1", NULL, "Internal Mic"},
	{"DMIC R1", NULL, "Internal Mic"},
	{"Headphone Jack", NULL, "HPOR"},
	{"Headphone Jack", NULL, "HPOL"},
};

static const struct snd_kcontrol_new card_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Internal Mic"),
	SOC_DAPM_PIN_SWITCH("Headphone Mic"),
};

static struct snd_soc_card snd_edgetpu_card = {
	.name = "snd-edgetpu-card",
	.owner = THIS_MODULE,
	.dai_link = card_dai,
	.num_links = ARRAY_SIZE(card_dai),
	.dapm_routes = audio_routes,
	.num_dapm_routes = ARRAY_SIZE(audio_routes),
	.dapm_widgets = card_widgets,
	.num_dapm_widgets = ARRAY_SIZE(card_widgets),
	.controls = card_controls,
	.num_controls = ARRAY_SIZE(card_controls),
	.fully_routed = true,
};

static int probe(struct platform_device *pdev) {
	int ret = 0;
	struct snd_soc_dai_link *dai = &card_dai[0];
	struct snd_soc_card *card = &snd_edgetpu_card;
	struct device *dev = &pdev->dev;
	struct i2c_client *codec_dev;
	struct device_node *i2s_node;
	struct clk *codec_clk;

	if (!dev) {
		printk(KERN_ERR "edgetpu-audio-card: no device for this platform_device?!\n");
		return -EINVAL;
	}

	card->dev = dev;

	if (!dev->of_node) {
		dev_err(dev, "this device requires a devicetree node!\n");
		return -EINVAL;
	}

	dai->codec_name = NULL;
	dai->codec_of_node = of_parse_phandle(dev->of_node, "audio-codec", 0);

	if (!dai->codec_of_node) {
		dev_err(dev, "can't parse codec node\n");
		return -EINVAL;
	}

	codec_dev = of_find_i2c_device_by_node(dai->codec_of_node);

	if (!codec_dev) {
		dev_err(dev, "can't find codec device!\n");
		return -EINVAL;
	}

	codec_clk = clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(codec_clk)) {
		dev_warn(dev, "can't find clock -- using defaults!\n");
	} else {
		codec_clock = clk_get_rate(codec_clk);
		clk_put(codec_clk);
		dev_info(dev, "clock set to %ld\n", codec_clock);
	}

	i2s_node = of_parse_phandle(dev->of_node, "audio-cpu", 0);

	if (!i2s_node) {
		dev_err(dev, "can't parse cpu node\n");
		return -EINVAL;
	}

	dai->cpu_dai_name = NULL;
	dai->cpu_of_node = i2s_node;
	dai->platform_name = NULL;
	dai->platform_of_node = i2s_node;

	ret = snd_soc_of_parse_card_name(card, "google,model");
	if (ret < 0) {
		dev_err(dev, "can't parse card name: %d\n", ret);
		return ret;
	}

	ret = devm_snd_soc_register_card(dev, card);
	if (ret < 0) {
		dev_err(dev, "can't register card: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id of_match[] = {
	{
		.compatible = "google,edgetpu-audio-card",
	},
	{},
};
MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver card_driver = {
	.driver = {
		.name = "edgetpu-audio-card",
		.owner = THIS_MODULE,
		.of_match_table = of_match,
	},
	.probe = probe,
};
module_platform_driver(card_driver);

MODULE_AUTHOR("June Tate-Gans <jtgans@google.com>");
MODULE_DESCRIPTION("ASoC Driver for Google AIY Voice Bonnet");
MODULE_LICENSE("GPL v2");
