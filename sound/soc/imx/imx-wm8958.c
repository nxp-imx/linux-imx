/*
 * imx-wm8958.c
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/jack.h>
#include <mach/dma.h>
#include <mach/clock.h>
#include <mach/audmux.h>

#include "imx-ssi.h"
#include "../codecs/wm8994.h"
#include <linux/mfd/wm8994/registers.h>

struct imx_priv {
	int sysclk;         /*mclk from the outside*/
	int codec_sysclk;
	int dai_hifi;
	struct platform_device *pdev;
	struct wm8994 *wm8958;
};
static struct imx_priv card_priv;
static struct snd_soc_card snd_soc_card_imx;
struct clk *codec_mclk;
static struct snd_soc_jack hs_jack;

/* Headphones jack detection DAPM pins */
static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

/* Headphones jack detection gpios */
static struct snd_soc_jack_gpio hs_jack_gpios[] = {
	[0] = {
		/* gpio is set on per-platform basis */
		.name		= "hp-gpio",
		.report		= SND_JACK_HEADPHONE,
		.debounce_time	= 200,
	},
};

static int imx_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	if (!codec_dai->active)
		clk_enable(codec_mclk);

	return 0;
}

static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	if (!codec_dai->active)
		clk_disable(codec_mclk);

	return;
}

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_priv *priv = &card_priv;
	unsigned int channels = params_channels(params);
	int ret = 0;
	unsigned int pll_out;
	u32 dai_format;

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 2, 32);

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	if (params_rate(params) == 8000 || params_rate(params) == 11025)
		pll_out = params_rate(params) * 512;
	else
		pll_out = params_rate(params) * 256;

	ret =
	    snd_soc_dai_set_pll(codec_dai, WM8994_FLL1, WM8994_FLL_SRC_MCLK1,
				priv->sysclk, pll_out);
	if (ret < 0)
		return ret;

	ret =
	    snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_FLL1, pll_out,
				   SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return 0;
}

/* imx card dapm widgets */
static const struct snd_soc_dapm_widget imx_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

/* imx machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* ----input ------------------- */
	/* Mic Jack --> MIC_IN (with automatic bias) */
	{"MICBIAS2", NULL, "Headset Mic"},
	{"IN1LP", NULL, "MICBIAS2"},
	{"IN1LN", NULL, "Headset Mic"},

	{"MICBIAS1", NULL, "Main Mic"},
	{"IN1RP", NULL, "MICBIAS1"},
	{"IN1RN", NULL, "Main Mic"},

	/* ----output------------------- */
	/* HP_OUT --> Headphone Jack */
	{"Headphone Jack", NULL, "HPOUT1L"},
	{"Headphone Jack", NULL, "HPOUT1R"},

	/* LINE_OUT --> Ext Speaker */
	{"Ext Spk", NULL, "SPKOUTLP"},
	{"Ext Spk", NULL, "SPKOUTLN"},
	{"Ext Spk", NULL, "SPKOUTRP"},
	{"Ext Spk", NULL, "SPKOUTRN"},

};

static int imx_wm8958_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret;

/* Add imx specific widgets */
	snd_soc_dapm_new_controls(&codec->dapm, imx_dapm_widgets,
				  ARRAY_SIZE(imx_dapm_widgets));

	/* Set up imx specific audio path audio_map */
	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");

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

static struct snd_soc_ops imx_hifi_ops = {
	.startup = imx_hifi_startup,
	.shutdown = imx_hifi_shutdown,
	.hw_params = imx_hifi_hw_params,
};

static struct snd_soc_dai_link imx_dai[] = {
	{
		.name = "HiFi",
		.stream_name = "HiFi",
		.codec_dai_name	= "wm8994-aif1",
		.codec_name	= "wm8994-codec",
		.cpu_dai_name	= "imx-ssi.1",
		.platform_name	= "imx-pcm-audio.1",
		.init		= imx_wm8958_init,
		.ops		= &imx_hifi_ops,
	},
};

static struct snd_soc_card snd_soc_card_imx = {
	.name		= "wm8958-audio",
	.dai_link	= imx_dai,
	.num_links	= ARRAY_SIZE(imx_dai),
};

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	slave = slave - 1;
	master = master - 1;

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

/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int __devinit imx_wm8958_probe(struct platform_device *pdev)
{

	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_priv *priv = &card_priv;
	struct wm8994 *wm8958 = plat->priv;
	int ret = 0;

	codec_mclk = clk_get(NULL, "clko_clk");
	if (IS_ERR(codec_mclk)) {
		printk(KERN_ERR "can't get CLKO clock.\n");
		return PTR_ERR(codec_mclk);
	}

	priv->pdev = pdev;
	priv->wm8958 = wm8958;

	imx_audmux_config(plat->src_port, plat->ext_port);

	if (plat->init && plat->init()) {
		ret = -EINVAL;
		return ret;
	}

	priv->sysclk = plat->sysclk;
	hs_jack_gpios[0].gpio = plat->hp_gpio;
	hs_jack_gpios[0].invert = plat->hp_active_low;

	return ret;
}

static int __devexit imx_wm8958_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->finit)
		plat->finit();

	clk_disable(codec_mclk);
	clk_put(codec_mclk);

	return 0;
}

static struct platform_driver imx_wm8958_driver = {
	.probe = imx_wm8958_probe,
	.remove = imx_wm8958_remove,
	.driver = {
		   .name = "imx-wm8958",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_snd_device;

static int __init imx_asoc_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_wm8958_driver);
	if (ret < 0)
		goto exit;

	imx_snd_device = platform_device_alloc("soc-audio", -1);
	if (!imx_snd_device)
		goto err_device_alloc;

	platform_set_drvdata(imx_snd_device, &snd_soc_card_imx);

	ret = platform_device_add(imx_snd_device);

	if (0 == ret)
		goto exit;

	platform_device_put(imx_snd_device);

err_device_alloc:
	platform_driver_unregister(&imx_wm8958_driver);
exit:
	return ret;
}

static void __exit imx_asoc_exit(void)
{
	platform_driver_unregister(&imx_wm8958_driver);
	platform_device_unregister(imx_snd_device);
}

module_init(imx_asoc_init);
module_exit(imx_asoc_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC imx wm8958");
MODULE_LICENSE("GPL");
