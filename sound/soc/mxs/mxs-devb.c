/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <mach/dma.h>
#include <mach/device.h>

#include "mxs-dai.h"
#include "mxs-pcm.h"
#include "../codecs/sgtl5000.h"

struct mxs_evk_priv {
	int sysclk;
	int hw;
	struct platform_device *pdev;
};

static struct mxs_evk_priv card_priv;

static int mxs_evk_audio_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct mxs_evk_priv *priv = &card_priv;
	unsigned int rate = params_rate(params);
	int ret = 0;

	u32 dai_format;

	/* only need to do this once as capture and playback are sync */
	if (priv->hw)
		return 0;
	priv->hw = 1;
	priv->sysclk = 512 * rate;

	snd_soc_dai_set_sysclk(codec_dai, SGTL5000_SYSCLK, (priv->sysclk)/2, 0);
	snd_soc_dai_set_sysclk(codec_dai, SGTL5000_LRCLK, rate, 0);

	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSP_SYS_MCLK, 256);
	/* set codec to slave mode */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;
	/* set cpu_dai to master mode for playback, slave mode for record */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set the SAIF system clock as output */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, priv->sysclk, \
		SND_SOC_CLOCK_OUT);

	return 0;
}

static int mxs_evk_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void mxs_evk_shutdown(struct snd_pcm_substream *substream)
{
	struct mxs_evk_priv *priv = &card_priv;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;

	if (cpu_dai->playback.active || cpu_dai->capture.active)
		priv->hw = 1;
	else
		priv->hw = 0;
}

/*
 * mxs_evk SGTL5000 audio DAI opserations.
 */
static struct snd_soc_ops mxs_evk_ops = {
	.startup = mxs_evk_startup,
	.shutdown = mxs_evk_shutdown,
	.hw_params = mxs_evk_audio_hw_params,
};

/* mxs_evk machine connections to the codec pins */
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

static const char *jack_function[] = { "off", "on"};

static const char *spk_function[] = { "off", "on" };

static const char *line_in_function[] = { "off", "on" };

static const struct soc_enum sgtl5000_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, line_in_function),
};

/* mxs_evk card dapm widgets */
static const struct snd_soc_dapm_widget mxs_evk_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static int mxs_evk_sgtl5000_init(struct snd_soc_codec *codec)
{
	/* Add mxs_evk specific widgets */
	snd_soc_dapm_new_controls(codec, mxs_evk_dapm_widgets,
				  ARRAY_SIZE(mxs_evk_dapm_widgets));

	/* Set up mxs_evk specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;
}

/* mxs_evk digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link mxs_evk_dai = {
	.name = "SGTL5000",
	.stream_name = "SGTL5000",
	.codec_dai = &sgtl5000_dai,
	.init = mxs_evk_sgtl5000_init,
	.ops = &mxs_evk_ops,
};

static int mxs_evk_card_remove(struct platform_device *pdev)
{
	struct mxs_evk_priv *priv = &card_priv;
	struct mxs_audio_platform_data *plat;
	if (priv->pdev) {
		plat = priv->pdev->dev.platform_data;
		if (plat->finit)
			plat->finit();
	}

	return 0;
}

static struct snd_soc_card snd_soc_card_mxs_evk = {
	.name = "mxs-evk",
	.platform = &mxs_soc_platform,
	.dai_link = &mxs_evk_dai,
	.num_links = 1,
	.remove = mxs_evk_card_remove,
};

static struct snd_soc_device mxs_evk_snd_devdata = {
	.card = &snd_soc_card_mxs_evk,
	.codec_dev = &soc_codec_dev_sgtl5000,
};

static int __devinit mxs_evk_sgtl5000_probe(struct platform_device *pdev)
{
	struct mxs_audio_platform_data *plat = pdev->dev.platform_data;
	struct mxs_saif *saif_select;
	int ret = -EINVAL;
	if (plat->init && plat->init())
		goto err_plat_init;
	mxs_evk_dai.cpu_dai = &mxs_saif_dai[0];
	saif_select = (struct mxs_saif *)mxs_evk_dai.cpu_dai->private_data;
	saif_select->stream_mapping = PLAYBACK_SAIF0_CAPTURE_SAIF1;
	saif_select->saif_mclk = plat->saif_mclock;
	saif_select->saif_clk = SAIF0;
	return 0;
err_plat_init:
	if (plat->finit)
		plat->finit();
	return ret;
}

static int mxs_evk_sgtl5000_remove(struct platform_device *pdev)
{
	struct mxs_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->finit)
		plat->finit();
	return 0;
}

static struct platform_driver mxs_evk_sgtl5000_audio_driver = {
	.probe = mxs_evk_sgtl5000_probe,
	.remove = mxs_evk_sgtl5000_remove,
	.driver = {
		   .name = "mxs-sgtl5000",
		   },
};

static struct platform_device *mxs_evk_snd_device;

static int __init mxs_evk_init(void)
{
	int ret;

	ret = platform_driver_register(&mxs_evk_sgtl5000_audio_driver);
	if (ret)
		return -ENOMEM;

	mxs_evk_snd_device = platform_device_alloc("soc-audio", 1);
	if (!mxs_evk_snd_device)
		return -ENOMEM;

	platform_set_drvdata(mxs_evk_snd_device, &mxs_evk_snd_devdata);
	mxs_evk_snd_devdata.dev = &mxs_evk_snd_device->dev;
	ret = platform_device_add(mxs_evk_snd_device);

	if (ret)
		platform_device_put(mxs_evk_snd_device);

	return ret;
}

static void __exit mxs_evk_exit(void)
{
	platform_driver_unregister(&mxs_evk_sgtl5000_audio_driver);
	platform_device_unregister(mxs_evk_snd_device);
}

module_init(mxs_evk_init);
module_exit(mxs_evk_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SGTL5000 Driver for MXS EVK");
MODULE_LICENSE("GPL");
