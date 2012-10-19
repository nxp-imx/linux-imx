/*
 * sound/soc/imx/3ds-sgtl5000.c --  SoC audio for i.MX 3ds boards with
 *                                  sgtl5000 codec
 *
 * Copyright 2009 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fsl_devices.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
#include <mach/audmux.h>

#include "../codecs/sgtl5000.h"
#include "imx-ssi.h"

static struct imx_sgtl5000_priv {
	int sysclk;
	int hw;
	struct platform_device *pdev;
} card_priv;

static struct snd_soc_jack hs_jack;
static struct snd_soc_card imx_sgtl5000;

/* Headphones jack detection DAPM pins */
static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Ext Spk",
		.mask = SND_JACK_HEADPHONE,
		.invert = 1,
	},
};

/* Headphones jack detection gpios */
static struct snd_soc_jack_gpio hs_jack_gpios[] = {
	[0] = {
		/* gpio is set on per-platform basis */
		.name           = "hp-gpio",
		.report         = SND_JACK_HEADPHONE,
		.debounce_time	= 200,
	},
};

static int sgtl5000_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	u32 dai_format;
	int ret;
	unsigned int channels = params_channels(params);

	snd_soc_dai_set_sysclk(codec_dai, SGTL5000_SYSCLK, card_priv.sysclk, 0);

	snd_soc_dai_set_sysclk(codec_dai, SGTL5000_LRCLK, params_rate(params), 0);

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;


	/* TODO: The SSI driver should figure this out for us */
	switch (channels) {
	case 2:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffc, 0xfffffffc, 2, 0);
		break;
	case 1:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffe, 0xfffffffe, 1, 0);
		break;
	default:
		return -EINVAL;
	}

	/* set cpu DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
		SND_SOC_DAIFMT_CBM_CFM;
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops imx_sgtl5000_hifi_ops = {
	.hw_params = sgtl5000_params,
};

static int sgtl5000_jack_func;
static int sgtl5000_spk_func;
static int sgtl5000_line_in_func;

static const char *jack_function[] = { "off", "on"};

static const char *spk_function[] = { "off", "on" };

static const char *line_in_function[] = { "off", "on" };

static const struct soc_enum sgtl5000_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, line_in_function),
};

static int sgtl5000_get_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = sgtl5000_jack_func;
	return 0;
}

static int sgtl5000_set_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (sgtl5000_jack_func == ucontrol->value.enumerated.item[0])
		return 0;

	sgtl5000_jack_func = ucontrol->value.enumerated.item[0];
	if (sgtl5000_jack_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphone Jack");

	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int sgtl5000_get_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = sgtl5000_spk_func;
	return 0;
}

static int sgtl5000_set_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (sgtl5000_spk_func == ucontrol->value.enumerated.item[0])
		return 0;

	sgtl5000_spk_func = ucontrol->value.enumerated.item[0];
	if (sgtl5000_spk_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Ext Spk");

	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int sgtl5000_get_line_in(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = sgtl5000_line_in_func;
	return 0;
}

static int sgtl5000_set_line_in(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (sgtl5000_line_in_func == ucontrol->value.enumerated.item[0])
		return 0;

	sgtl5000_line_in_func = ucontrol->value.enumerated.item[0];
	if (sgtl5000_line_in_func)
		snd_soc_dapm_enable_pin(&codec->dapm, "Line In Jack");
	else
		snd_soc_dapm_disable_pin(&codec->dapm, "Line In Jack");

	snd_soc_dapm_sync(&codec->dapm);
	return 1;
}

static int spk_amp_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct imx_sgtl5000_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->amp_enable == NULL)
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event))
		plat->amp_enable(1);
	else
		plat->amp_enable(0);

	return 0;
}

/* imx_3stack card dapm widgets */
static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", spk_amp_event),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static const struct snd_kcontrol_new sgtl5000_machine_controls[] = {
	SOC_ENUM_EXT("Jack Function", sgtl5000_enum[0], sgtl5000_get_jack,
		     sgtl5000_set_jack),
	SOC_ENUM_EXT("Speaker Function", sgtl5000_enum[1], sgtl5000_get_spk,
		     sgtl5000_set_spk),
	SOC_ENUM_EXT("Line In Function", sgtl5000_enum[1], sgtl5000_get_line_in,
		     sgtl5000_set_line_in),
};

/* imx_3stack machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {

	/* Mic Jack --> MIC_IN (with automatic bias) */
	{"MIC_IN", NULL, "Mic Jack"},

	/* Line in Jack --> LINE_IN */
	{"LINE_IN", NULL, "Line In Jack"},

	/* HP_OUT --> Headphone Jack */
	{"Headphone Jack", NULL, "HP_OUT"},

	/* LINE_OUT --> Ext Speaker */
	{"Ext Spk", NULL, "LINE_OUT"},
};

static int imx_3stack_sgtl5000_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret;

	ret = snd_soc_add_controls(codec, sgtl5000_machine_controls,
			ARRAY_SIZE(sgtl5000_machine_controls));
	if (ret)
		return ret;

	/* Add imx_3stack specific widgets */
	snd_soc_dapm_new_controls(&codec->dapm, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));

	/* Set up imx_3stack specific audio path audio_map */
	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_disable_pin(&codec->dapm, "Line In Jack");
	snd_soc_dapm_disable_pin(&codec->dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(&codec->dapm, "Ext Spk");
	snd_soc_dapm_sync(&codec->dapm);

	if (hs_jack_gpios[0].gpio != -1) {
		/* Jack detection API stuff */
		ret = snd_soc_jack_new(codec, "Headphone Jack",
				       SND_JACK_HEADPHONE, &hs_jack);
		if (ret)
			return ret;

		ret = snd_soc_jack_add_pins(&hs_jack, ARRAY_SIZE(hs_jack_pins),
					hs_jack_pins);
		if (ret) {
			printk(KERN_ERR "failed to call  snd_soc_jack_add_pins\n");
			return ret;
		}

		ret = snd_soc_jack_add_gpios(&hs_jack,
					ARRAY_SIZE(hs_jack_gpios), hs_jack_gpios);
		if (ret)
			printk(KERN_WARNING "failed to call snd_soc_jack_add_gpios\n");
	}

	return 0;
}

static struct snd_soc_dai_link imx_sgtl5000_dai[] = {
	{
		.name		= "HiFi",
		.stream_name	= "HiFi",
		.codec_dai_name	= "sgtl5000",
		.codec_name	= "sgtl5000.1-000a",
		.cpu_dai_name	= "imx-ssi.1",
		.platform_name	= "imx-pcm-audio.1",
		.init		= imx_3stack_sgtl5000_init,
		.ops		= &imx_sgtl5000_hifi_ops,
	},
};

static struct snd_soc_card imx_sgtl5000 = {
	.name		= "sgtl5000-audio",
	.dai_link	= imx_sgtl5000_dai,
	.num_links	= ARRAY_SIZE(imx_sgtl5000_dai),
};

static struct platform_device *imx_sgtl5000_snd_device;

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	slave = slave - 1;
	master = master - 1;

	/* SSI0 mastered by port 5 */
	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(master) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

static int __devinit imx_sgtl5000_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	int ret = 0;

	card_priv.pdev = pdev;

	imx_audmux_config(plat->src_port, plat->ext_port);

	ret = -EINVAL;
	if (plat->init && plat->init())
		return ret;

	card_priv.sysclk = plat->sysclk;

	hs_jack_gpios[0].gpio = plat->hp_gpio;
	hs_jack_gpios[0].invert = plat->hp_active_low;

	return 0;
}

static int imx_sgtl5000_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->finit)
		plat->finit();

	return 0;
}

static struct platform_driver imx_sgtl5000_audio_driver = {
	.probe = imx_sgtl5000_probe,
	.remove = imx_sgtl5000_remove,
	.driver = {
		   .name = "imx-sgtl5000",
		   },
};

static int __init imx_sgtl5000_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_sgtl5000_audio_driver);
	if (ret)
		return -ENOMEM;

	if (machine_is_mx35_3ds() || machine_is_mx6q_sabrelite())
		imx_sgtl5000_dai[0].codec_name = "sgtl5000.0-000a";
	else
		imx_sgtl5000_dai[0].codec_name = "sgtl5000.1-000a";

	imx_sgtl5000_snd_device = platform_device_alloc("soc-audio", 1);
	if (!imx_sgtl5000_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_sgtl5000_snd_device, &imx_sgtl5000);

	ret = platform_device_add(imx_sgtl5000_snd_device);

	if (ret) {
		printk(KERN_ERR "ASoC: Platform device allocation failed\n");
		platform_device_put(imx_sgtl5000_snd_device);
	}

	return ret;
}

static void __exit imx_sgtl5000_exit(void)
{
	platform_driver_unregister(&imx_sgtl5000_audio_driver);
	platform_device_unregister(imx_sgtl5000_snd_device);
}

module_init(imx_sgtl5000_init);
module_exit(imx_sgtl5000_exit);

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("PhyCORE ALSA SoC driver");
MODULE_LICENSE("GPL");
