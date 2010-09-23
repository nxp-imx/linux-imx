/*
 * imx-3stack-ak5702.c  --  SoC audio for imx_3stack
 *
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <mach/hardware.h>
#include <mach/clock.h>

#include "imx-pcm.h"
#include "imx-esai.h"
#include "../codecs/ak5702.h"

struct imx_3stack_pcm_state {
	int lr_clk_active;
};

static struct imx_3stack_pcm_state clk_state;

static int imx_3stack_startup(struct snd_pcm_substream *substream)
{
	clk_state.lr_clk_active++;
	return 0;
}

static void imx_3stack_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *pcm_link = rtd->dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;

	/* disable the PLL if there are no active Rx channels */
	if (!codec_dai->active)
		snd_soc_dai_set_pll(codec_dai, 0, 0, 0, 0);
	clk_state.lr_clk_active--;
}

static int imx_3stack_surround_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *pcm_link = rtd->dai;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	unsigned int rate = params_rate(params);
	struct imx_esai *esai_mode = (struct imx_esai *)cpu_dai->private_data;
	u32 dai_format;

	if (clk_state.lr_clk_active > 1)
		return 0;

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM;

	esai_mode->sync_mode = 0;
	esai_mode->network_mode = 1;

	/* set codec DAI configuration */
	snd_soc_dai_set_fmt(codec_dai, dai_format);

	/* set cpu DAI configuration */
	snd_soc_dai_set_fmt(cpu_dai, dai_format);

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai, 0xffffffff, 0xffffffff, 2, 0);

	/* set the ESAI system clock as input */
	snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_IN);

	/* set codec BCLK division */
	snd_soc_dai_set_clkdiv(codec_dai, AK5702_BCLK_CLKDIV,
			       AK5702_BCLK_DIV_32);

	snd_soc_dai_set_sysclk(codec_dai, 0, rate, SND_SOC_CLOCK_OUT);

	snd_soc_dai_set_pll(codec_dai, 1, 0, 12000000, 0);
	return 0;
}

/*
 * imx_3stack ak5702 DAI opserations.
 */
static struct snd_soc_ops imx_3stack_surround_ops = {
	.startup = imx_3stack_startup,
	.shutdown = imx_3stack_shutdown,
	.hw_params = imx_3stack_surround_hw_params,
};

/* imx_3stack card dapm widgets */
static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

/* example card audio map */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Line in jack */
	{"ADCA Left Input", NULL, "Line In Jack"},
	{"ADCA Right Input", NULL, "Line In Jack"},
	{"ADCB Left Input", NULL, "Line In Jack"},
	{"ADCB Right Input", NULL, "Line In Jack"},
};

static int imx_3stack_ak5702_init(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	snd_soc_dapm_sync(codec);
	return 0;
}

static struct snd_soc_dai_link imx_3stack_dai = {
	.name = "ak5702",
	.stream_name = "ak5702",
	.codec_dai = &ak5702_dai,
	.init = imx_3stack_ak5702_init,
	.ops = &imx_3stack_surround_ops,
};

static int imx_3stack_card_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	kfree(socdev->codec_data);
	return 0;
}

static struct snd_soc_card snd_soc_card_imx_3stack = {
	.name = "imx-3stack",
	.platform = &imx_soc_platform,
	.dai_link = &imx_3stack_dai,
	.num_links = 1,
	.remove = imx_3stack_card_remove,
};

static struct snd_soc_device imx_3stack_snd_devdata = {
	.card = &snd_soc_card_imx_3stack,
	.codec_dev = &soc_codec_dev_ak5702,
};

static int __devinit imx_3stack_ak5702_probe(struct platform_device *pdev)
{
	struct ak5702_setup_data *setup;

	imx_3stack_dai.cpu_dai = &imx_esai_dai[2];

	setup = kzalloc(sizeof(struct ak5702_setup_data), GFP_KERNEL);
	setup->i2c_bus = 1;
	setup->i2c_address = 0x13;
	imx_3stack_snd_devdata.codec_data = setup;

	return 0;
}

static int imx_3stack_ak5702_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver imx_3stack_ak5702_driver = {
	.probe = imx_3stack_ak5702_probe,
	.remove = imx_3stack_ak5702_remove,
	.driver = {
		   .name = "imx-3stack-ak5702",
		   },
};

static struct platform_device *imx_3stack_snd_device;

static int __init imx_3stack_asoc_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_3stack_ak5702_driver);
	if (ret)
		return -ENOMEM;

	imx_3stack_snd_device = platform_device_alloc("soc-audio", -1);
	if (!imx_3stack_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_3stack_snd_device, &imx_3stack_snd_devdata);
	imx_3stack_snd_devdata.dev = &imx_3stack_snd_device->dev;
	ret = platform_device_add(imx_3stack_snd_device);
	if (ret)
		platform_device_put(imx_3stack_snd_device);

	return ret;
}

static void __exit imx_3stack_asoc_exit(void)
{
	platform_driver_unregister(&imx_3stack_ak5702_driver);
	platform_device_unregister(imx_3stack_snd_device);
}

module_init(imx_3stack_asoc_init);
module_exit(imx_3stack_asoc_exit);

/* Module information */
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ALSA SoC ak5702 imx_3stack");
MODULE_LICENSE("GPL");
