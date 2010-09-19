/*
 * Copyright (C) 2010 Freescale  Semiconductor, Inc. All Rights Reserved.
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
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/soc-dai.h>

#include <mach/hardware.h>
#include <mach/clock.h>

#include "imx-pcm.h"
#include "imx-esai.h"
#include "../codecs/cs42888.h"

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
#include <linux/mxc_asrc.h>
#endif

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
static unsigned int asrc_rates[] = {
	0,
	8000,
	11025,
	16000,
	22050,
	32000,
	44100,
	48000,
	64000,
	88200,
	96000,
	176400,
	192000,
};

struct asrc_esai {
	unsigned int cpu_dai_rates;
	unsigned int codec_dai_rates;
	enum asrc_pair_index asrc_index;
	unsigned int output_sample_rate;
};

static struct asrc_esai asrc_esai_data;

#endif

struct imx_3stack_pcm_state {
	int lr_clk_active;
};

static struct imx_3stack_pcm_state clk_state;
unsigned int mclk_freq;

static int imx_3stack_startup(struct snd_pcm_substream *substream)
{
	clk_state.lr_clk_active++;
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	if (asrc_esai_data.output_sample_rate >= 32000) {
		struct snd_soc_pcm_runtime *rtd = substream->private_data;
		struct snd_soc_dai_link *pcm_link = rtd->dai;
		struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
		struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
		asrc_esai_data.cpu_dai_rates = cpu_dai->playback.rates;
		asrc_esai_data.codec_dai_rates = codec_dai->playback.rates;
		cpu_dai->playback.rates =
		    SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT;
		codec_dai->playback.rates =
		    SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT;
	}
#endif

	return 0;
}

static void imx_3stack_shutdown(struct snd_pcm_substream *substream)
{
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *pcm_link = rtd->dai;

	if (asrc_esai_data.output_sample_rate >= 32000) {
		struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
		codec_dai->playback.rates = asrc_esai_data.codec_dai_rates;
		cpu_dai->playback.rates = asrc_esai_data.cpu_dai_rates;
		asrc_release_pair(asrc_esai_data.asrc_index);
	}
#endif

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
	unsigned int lrclk_ratio = 0;
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	unsigned int channel = params_channels(params);
#endif
	struct imx_esai *esai_mode = (struct imx_esai *)cpu_dai->private_data;
	if (clk_state.lr_clk_active > 1)
		return 0;

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	if (asrc_esai_data.output_sample_rate >= 32000) {
		unsigned int asrc_input_rate = rate;
		struct mxc_runtime_data *pcm_data =
		    substream->runtime->private_data;
		struct asrc_config config;
		int retVal = 0;;

		retVal = asrc_req_pair(channel, &asrc_esai_data.asrc_index);
		if (retVal < 0) {
			pr_err("Fail to request asrc pair\n");
			return -1;
		}

		config.pair = asrc_esai_data.asrc_index;
		config.channel_num = channel;
		config.input_sample_rate = asrc_input_rate;
		config.output_sample_rate = asrc_esai_data.output_sample_rate;
		config.inclk = INCLK_NONE;
		config.word_width = 32;
		config.outclk = OUTCLK_ESAI_TX;
		retVal = asrc_config_pair(&config);
		if (retVal < 0) {
			pr_err("Fail to config asrc\n");
			asrc_release_pair(asrc_esai_data.asrc_index);
			return retVal;
		}
		rate = asrc_esai_data.output_sample_rate;
		pcm_data->asrc_index = asrc_esai_data.asrc_index;
		pcm_data->asrc_enable = 1;
	}
#endif

	switch (rate) {
	case 32000:
		lrclk_ratio = 3;
		break;
	case 48000:
		lrclk_ratio = 3;
		break;
	case 64000:
		lrclk_ratio = 1;
		break;
	case 96000:
		lrclk_ratio = 1;
		break;
	case 128000:
		lrclk_ratio = 1;
		break;
	case 44100:
		lrclk_ratio = 3;
		break;
	case 88200:
		lrclk_ratio = 1;
		break;
	case 176400:
		lrclk_ratio = 0;
		break;
	case 192000:
		lrclk_ratio = 0;
		break;
	default:
		pr_info("Rate not support.\n");
		return -EINVAL;;
	}

	dai_format = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;

	esai_mode->sync_mode = 0;
	esai_mode->network_mode = 1;

	/* set cpu DAI configuration */
	snd_soc_dai_set_fmt(cpu_dai, dai_format);
	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai, 0x3, 0x3, 2, 32);
	/* set the ESAI system clock as output */
	snd_soc_dai_set_sysclk(cpu_dai, ESAI_CLK_EXTAL,
		mclk_freq, SND_SOC_CLOCK_OUT);
	/* set the ratio */
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_TX_DIV_PSR, 1);
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_TX_DIV_PM, 0);
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_TX_DIV_FP, lrclk_ratio);
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_RX_DIV_PSR, 1);
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_RX_DIV_PM, 0);
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_RX_DIV_FP, lrclk_ratio);

	/* set codec DAI configuration */
	snd_soc_dai_set_fmt(codec_dai, dai_format);
	/* set codec Master clock */
	snd_soc_dai_set_sysclk(codec_dai, 0, mclk_freq, SND_SOC_CLOCK_IN);

	return 0;
}

static struct snd_soc_ops imx_3stack_surround_ops = {
	.startup = imx_3stack_startup,
	.shutdown = imx_3stack_shutdown,
	.hw_params = imx_3stack_surround_hw_params,
};

static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line Out Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Line out jack */
	{"Line Out Jack", NULL, "AOUT1L"},
	{"Line Out Jack", NULL, "AOUT1R"},
	{"Line Out Jack", NULL, "AOUT2L"},
	{"Line Out Jack", NULL, "AOUT2R"},
	{"Line Out Jack", NULL, "AOUT3L"},
	{"Line Out Jack", NULL, "AOUT3R"},
	{"Line Out Jack", NULL, "AOUT4L"},
	{"Line Out Jack", NULL, "AOUT4R"},
	{"AIN1L", NULL, "Line In Jack"},
	{"AIN1R", NULL, "Line In Jack"},
	{"AIN2L", NULL, "Line In Jack"},
	{"AIN2R", NULL, "Line In Jack"},
};

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
static int asrc_func;

static const char *asrc_function[] = {
	"disable", "32KHz", "44.1KHz",
	"48KHz", "64KHz", "88.2KHz", "96KHz", "176.4KHz", "192KHz"
};

static const struct soc_enum asrc_enum[] = {
	SOC_ENUM_SINGLE_EXT(9, asrc_function),
};

static int asrc_get_rate(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = asrc_func;
	return 0;
}

static int asrc_set_rate(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	if (asrc_func == ucontrol->value.enumerated.item[0])
		return 0;

	asrc_func = ucontrol->value.enumerated.item[0];
	asrc_esai_data.output_sample_rate = asrc_rates[asrc_func + 4];

	return 1;
}

static const struct snd_kcontrol_new asrc_controls[] = {
	SOC_ENUM_EXT("ASRC", asrc_enum[0], asrc_get_rate,
		     asrc_set_rate),
};

#endif

static int imx_3stack_cs42888_init(struct snd_soc_codec *codec)
{

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	int i;
	int ret;
	for (i = 0; i < ARRAY_SIZE(asrc_controls); i++) {
		ret = snd_ctl_add(codec->card,
				  snd_soc_cnew(&asrc_controls[i], codec, NULL));
		if (ret < 0)
			return ret;
	}
	asrc_esai_data.output_sample_rate = asrc_rates[asrc_func + 4];
#endif

	snd_soc_dapm_new_controls(codec, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link imx_3stack_dai = {
	.name = "cs42888",
	.stream_name = "cs42888",
	.codec_dai = &cs42888_dai,
	.init = imx_3stack_cs42888_init,
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
	.codec_dev = &soc_codec_device_cs42888,
};

/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int __devinit imx_3stack_cs42888_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat_data = pdev->dev.platform_data;
	imx_3stack_dai.cpu_dai = &imx_esai_dai[2];
	imx_3stack_dai.cpu_dai->dev = &pdev->dev;
	mclk_freq = plat_data->sysclk;
	if (!(mclk_freq % 44100)) {
		imx_3stack_dai.codec_dai->playback.rates =
						    SNDRV_PCM_RATE_44100 |\
						    SNDRV_PCM_RATE_88200 |\
						    SNDRV_PCM_RATE_176400;
		imx_3stack_dai.codec_dai->capture.rates =
						    SNDRV_PCM_RATE_44100 |\
						    SNDRV_PCM_RATE_88200 |\
						    SNDRV_PCM_RATE_176400;
	}
	if (!(mclk_freq % 48000)) {
		imx_3stack_dai.codec_dai->playback.rates =
						    SNDRV_PCM_RATE_48000 |\
						    SNDRV_PCM_RATE_96000 |\
						    SNDRV_PCM_RATE_192000;
		imx_3stack_dai.codec_dai->capture.rates =
						    SNDRV_PCM_RATE_48000 |\
						    SNDRV_PCM_RATE_96000 |\
						    SNDRV_PCM_RATE_192000;
	}

	return 0;
}

static int __devexit imx_3stack_cs42888_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver imx_3stack_cs42888_driver = {
	.probe = imx_3stack_cs42888_probe,
	.remove = __devexit_p(imx_3stack_cs42888_remove),
	.driver = {
		   .name = "imx-3stack-cs42888",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_3stack_snd_device;

static int __init imx_3stack_asoc_init(void)
{
	int ret;
	ret = platform_driver_register(&imx_3stack_cs42888_driver);
	if (ret < 0)
		goto exit;

	imx_3stack_snd_device = platform_device_alloc("soc-audio", 1);
	if (!imx_3stack_snd_device)
		goto err_device_alloc;
	platform_set_drvdata(imx_3stack_snd_device, &imx_3stack_snd_devdata);
	imx_3stack_snd_devdata.dev = &imx_3stack_snd_device->dev;
	ret = platform_device_add(imx_3stack_snd_device);
	if (0 == ret && snd_soc_card_imx_3stack.codec != NULL)
		goto exit;

	platform_device_unregister(imx_3stack_snd_device);
err_device_alloc:
	platform_driver_unregister(&imx_3stack_cs42888_driver);
exit:
	return ret;
}

static void __exit imx_3stack_asoc_exit(void)
{
	platform_driver_unregister(&imx_3stack_cs42888_driver);
	platform_device_unregister(imx_3stack_snd_device);
}

module_init(imx_3stack_asoc_init);
module_exit(imx_3stack_asoc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ALSA SoC cs42888 imx_3stack");
MODULE_LICENSE("GPL");
