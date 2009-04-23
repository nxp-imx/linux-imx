/*
 * imx-3stack-wm8580.c  --  SoC 5.1 audio for imx_3stack
 *
 * Copyright 2008-2009 Freescale  Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

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
#include <mach/mxc.h>

#include "imx-pcm.h"
#include "imx-esai.h"
#include "../codecs/wm8580.h"

struct imx_3stack_pcm_state {
	int lr_clk_active;
};

extern void gpio_activate_esai_ports(void);
extern void gpio_deactivate_esai_ports(void);

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

	/* disable the PLL if there are no active Tx or Rx channels */
	if (!codec_dai->active)
		snd_soc_dai_set_pll(codec_dai, 0, 0, 0);
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
	u32 dai_format;
	unsigned int pll_out = 0, lrclk_ratio = 0;

	if (clk_state.lr_clk_active > 1)
		return 0;

	switch (rate) {
	case 8000:
		lrclk_ratio = 5;
		pll_out = 6144000;
		break;
	case 11025:
		lrclk_ratio = 4;
		pll_out = 5644800;
		break;
	case 16000:
		lrclk_ratio = 3;
		pll_out = 6144000;
		break;
	case 32000:
		lrclk_ratio = 3;
		pll_out = 12288000;
		break;
	case 48000:
		lrclk_ratio = 2;
		pll_out = 12288000;
		break;
	case 64000:
		lrclk_ratio = 1;
		pll_out = 12288000;
		break;
	case 96000:
		lrclk_ratio = 2;
		pll_out = 24576000;
		break;
	case 128000:
		lrclk_ratio = 1;
		pll_out = 24576000;
		break;
	case 22050:
		lrclk_ratio = 4;
		pll_out = 11289600;
		break;
	case 44100:
		lrclk_ratio = 2;
		pll_out = 11289600;
		break;
	case 88200:
		lrclk_ratio = 0;
		pll_out = 11289600;
		break;
	case 176400:
		lrclk_ratio = 0;
		pll_out = 22579200;
		break;
	case 192000:
		lrclk_ratio = 0;
		pll_out = 24576000;
		break;
	default:
		pr_info("Rate not support.\n");
		return -EINVAL;;
	}

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_ASYNC;

	dai_format |= SND_SOC_DAIFMT_TDM;

	/* set codec DAI configuration */
	snd_soc_dai_set_fmt(codec_dai, dai_format);

	/* set cpu DAI configuration */
	snd_soc_dai_set_fmt(cpu_dai, dai_format);

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai, 0xffffffff, 32);

	/* set the ESAI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_IN);

	snd_soc_dai_set_clkdiv(codec_dai, WM8580_MCLK, WM8580_CLKSRC_PLLA);
	snd_soc_dai_set_clkdiv(codec_dai, WM8580_DAC_CLKSEL,
			       WM8580_CLKSRC_PLLA);

	/* set codec LRCLK and BCLK */
	snd_soc_dai_set_sysclk(codec_dai, WM8580_BCLK_CLKDIV, 0,
			       SND_SOC_CLOCK_OUT);
	snd_soc_dai_set_sysclk(codec_dai, WM8580_LRCLK_CLKDIV, lrclk_ratio,
			       SND_SOC_CLOCK_OUT);

	snd_soc_dai_set_pll(codec_dai, 1, 12000000, pll_out);
	return 0;
}

/*
 * imx_3stack wm8580 HiFi DAI opserations.
 */
static struct snd_soc_ops imx_3stack_surround_ops = {
	.startup = imx_3stack_startup,
	.shutdown = imx_3stack_shutdown,
	.hw_params = imx_3stack_surround_hw_params,
};

/* imx_3stack machine dapm widgets */
static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line Out Jack", NULL),
};

/* example machine audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {

	/* Line out jack */
	{"Line Out Jack", NULL, "VOUT1L"},
	{"Line Out Jack", NULL, "VOUT1R"},
	{"Line Out Jack", NULL, "VOUT2L"},
	{"Line Out Jack", NULL, "VOUT2R"},
	{"Line Out Jack", NULL, "VOUT3L"},
	{"Line Out Jack", NULL, "VOUT3R"},

};

static int imx_3stack_wm8580_init(struct snd_soc_codec *codec)
{

	snd_soc_dapm_new_controls(codec, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link imx_3stack_dai = {
	.name = "wm8580",
	.stream_name = "wm8580",
	.cpu_dai = &imx_esai_dai,
	.codec_dai = wm8580_dai,
	.init = imx_3stack_wm8580_init,
	.ops = &imx_3stack_surround_ops,
};

static int imx_3stack_machine_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	kfree(socdev->codec_data);
	return 0;
}

static struct snd_soc_machine snd_soc_machine_imx_3stack = {
	.name = "imx-3stack",
	.dai_link = &imx_3stack_dai,
	.num_links = 1,
	.remove = imx_3stack_machine_remove,
};

static struct snd_soc_device imx_3stack_snd_devdata = {
	.machine = &snd_soc_machine_imx_3stack,
	.platform = &imx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8580,
};

/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int __devinit imx_3stack_wm8580_probe(struct platform_device *pdev)
{
	struct wm8580_setup_data *setup;

	imx_esai_dai.name = "imx-esai-tx";

	setup = kzalloc(sizeof(struct wm8580_setup_data), GFP_KERNEL);
	setup->spi = 1;
	imx_3stack_snd_devdata.codec_data = setup;

	/* Configure audio port 3 */
	gpio_activate_esai_ports();

	return 0;
}

static int __devexit imx_3stack_wm8580_remove(struct platform_device *pdev)
{
	gpio_deactivate_esai_ports();
	return 0;
}

static struct platform_driver imx_3stack_wm8580_driver = {
	.probe = imx_3stack_wm8580_probe,
	.remove = __devexit_p(imx_3stack_wm8580_remove),
	.driver = {
		   .name = "imx-3stack-wm8580",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_3stack_snd_device;

static int __init imx_3stack_asoc_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_3stack_wm8580_driver);
	if (ret < 0)
		goto exit;
	imx_3stack_snd_device = platform_device_alloc("soc-audio", 1);
	if (!imx_3stack_snd_device)
		goto err_device_alloc;
	platform_set_drvdata(imx_3stack_snd_device, &imx_3stack_snd_devdata);
	imx_3stack_snd_devdata.dev = &imx_3stack_snd_device->dev;
	ret = platform_device_add(imx_3stack_snd_device);
	if (0 == ret)
		goto exit;

	platform_device_put(imx_3stack_snd_device);
      err_device_alloc:
	platform_driver_unregister(&imx_3stack_wm8580_driver);
      exit:
	return ret;
}

static void __exit imx_3stack_asoc_exit(void)
{
	platform_driver_unregister(&imx_3stack_wm8580_driver);
	platform_device_unregister(imx_3stack_snd_device);
}

module_init(imx_3stack_asoc_init);
module_exit(imx_3stack_asoc_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC wm8580 imx_3stack");
MODULE_LICENSE("GPL");
