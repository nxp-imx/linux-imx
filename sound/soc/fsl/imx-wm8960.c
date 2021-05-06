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
#include <linux/mfd/syscon.h>
#include "../codecs/wm8960.h"
#include "fsl_sai.h"

#ifdef CONFIG_EXTCON
struct extcon_dev *wm8960_edev;

static const unsigned int imx_wm8960_extcon_cables[] = {
	EXTCON_JACK_MICROPHONE,
	EXTCON_JACK_HEADPHONE,
	EXTCON_NONE,
};
#endif

struct imx_wm8960_data {
	enum of_gpio_flags hp_active_low;
	enum of_gpio_flags mic_active_low;
	bool is_headset_jack;
	struct platform_device *pdev;
	struct platform_device *asrc_pdev;
	struct snd_soc_card card;
	struct clk *codec_clk;
	unsigned int clk_frequency;
	bool is_codec_master;
	bool is_codec_rpmsg;
	bool is_playback_only;
	bool is_capture_only;
	bool is_stream_in_use[2];
	bool is_stream_opened[2];
	struct regmap *gpr;
	unsigned int hp_det[2];
	u32 asrc_rate;
	u32 asrc_format;
	struct snd_soc_jack imx_hp_jack;
	struct snd_soc_jack_pin imx_hp_jack_pin;
	struct snd_soc_jack_gpio imx_hp_jack_gpio;
	struct snd_soc_jack imx_mic_jack;
	struct snd_soc_jack_pin imx_mic_jack_pin;
	struct snd_soc_jack_gpio imx_mic_jack_gpio;
	struct snd_soc_dai_link imx_wm8960_dai[3];
};

static int hp_jack_status_check(void *data)
{
	struct imx_wm8960_data *imx_data = (struct imx_wm8960_data *)data;
	struct snd_soc_jack *jack = &imx_data->imx_hp_jack;
	struct snd_soc_dapm_context *dapm = &jack->card->dapm;
	int hp_status, ret;

	hp_status = gpio_get_value_cansleep(imx_data->imx_hp_jack_gpio.gpio);

	if (hp_status != imx_data->hp_active_low) {
#ifdef CONFIG_EXTCON
		extcon_set_state_sync(wm8960_edev, EXTCON_JACK_HEADPHONE, 1);
#endif
		snd_soc_dapm_disable_pin(dapm, "Ext Spk");
		if (imx_data->is_headset_jack) {
			snd_soc_dapm_enable_pin(dapm, "Mic Jack");
			snd_soc_dapm_disable_pin(dapm, "Main MIC");
		}
		ret = imx_data->imx_hp_jack_gpio.report;
	} else {
#ifdef CONFIG_EXTCON
		extcon_set_state_sync(wm8960_edev, EXTCON_JACK_HEADPHONE, 0);
#endif
		snd_soc_dapm_enable_pin(dapm, "Ext Spk");
		if (imx_data->is_headset_jack) {
			snd_soc_dapm_disable_pin(dapm, "Mic Jack");
			snd_soc_dapm_enable_pin(dapm, "Main MIC");
		}
		ret = 0;
	}

	return ret;
}

static int mic_jack_status_check(void *data)
{
	struct imx_wm8960_data *imx_data = (struct imx_wm8960_data *)data;
	struct snd_soc_jack *jack = &imx_data->imx_mic_jack;
	struct snd_soc_dapm_context *dapm = &jack->card->dapm;
	int mic_status, ret;

	mic_status = gpio_get_value_cansleep(imx_data->imx_mic_jack_gpio.gpio);

	if (mic_status != imx_data->mic_active_low) {
		snd_soc_dapm_disable_pin(dapm, "Main MIC");
		ret = imx_data->imx_mic_jack_gpio.report;
	} else {
		snd_soc_dapm_enable_pin(dapm, "Main MIC");
		ret = 0;
	}

	return ret;
}

static const struct snd_soc_dapm_widget imx_wm8960_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("Main MIC", NULL),
};

static int imx_wm8960_jack_init(struct snd_soc_card *card,
		struct snd_soc_jack *jack, struct snd_soc_jack_pin *pin,
		struct snd_soc_jack_gpio *gpio)
{
	int ret;

	ret = snd_soc_card_jack_new(card, pin->pin, pin->mask, jack, pin, 1);
	if (ret) {
		return ret;
	}

	ret = snd_soc_jack_add_gpios(jack, 1, gpio);
	if (ret)
		return ret;

	return 0;
}

static ssize_t headphone_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	int hp_status;

	/* Check if headphone is plugged in */
	hp_status = gpio_get_value_cansleep(data->imx_hp_jack_gpio.gpio);

	if (hp_status != data->hp_active_low)
		strcpy(buf, "Headphone\n");
	else
		strcpy(buf, "Speaker\n");

	return strlen(buf);
}

static ssize_t micphone_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	int mic_status;

	/* Check if headphone is plugged in */
	mic_status = gpio_get_value_cansleep(data->imx_mic_jack_gpio.gpio);

	if (mic_status != data->mic_active_low)
		strcpy(buf, "Mic Jack\n");
	else
		strcpy(buf, "Main MIC\n");

	return strlen(buf);
}
static DEVICE_ATTR_RO(headphone);
static DEVICE_ATTR_RO(micphone);

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct device *dev = card->dev;
	unsigned int sample_rate = params_rate(params);
	unsigned int pll_out;
	unsigned int fmt;
	int ret = 0;

	data->is_stream_in_use[tx] = true;

	if (data->is_stream_in_use[!tx])
		return 0;

	if (data->is_codec_master)
		fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
	else
		fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}
	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret) {
		dev_err(dev, "failed to set codec dai fmt: %d\n", ret);
		return ret;
	}

	if (!data->is_codec_master) {
		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 2, params_width(params));
		if (ret) {
			dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_OUT);
		if (ret) {
			dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
			return ret;
		}
		return 0;
	} else {
		ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_IN);
		if (ret) {
			dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
			return ret;
		}
	}

	data->clk_frequency = clk_get_rate(data->codec_clk);

	/* Set codec pll */
	if (params_width(params) == 24)
		pll_out = sample_rate * 768;
	else
		pll_out = sample_rate * 512;

	ret = snd_soc_dai_set_pll(codec_dai, WM8960_SYSCLK_AUTO, 0, data->clk_frequency, pll_out);
	if (ret)
		return ret;
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8960_SYSCLK_AUTO, pll_out, 0);

	return ret;
}

static int imx_hifi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = rtd->card;
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct device *dev = card->dev;
	int ret;

	data->is_stream_in_use[tx] = false;

	if (data->is_codec_master && !data->is_stream_in_use[!tx]) {
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF);
		if (ret)
			dev_warn(dev, "failed to set codec dai fmt: %d\n", ret);
	}

	return 0;
}

static u32 imx_wm8960_rates[] = { 8000, 16000, 32000, 48000 };
static struct snd_pcm_hw_constraint_list imx_wm8960_rate_constraints = {
	.count = ARRAY_SIZE(imx_wm8960_rates),
	.list = imx_wm8960_rates,
};

static int imx_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct fsl_sai *sai = dev_get_drvdata(cpu_dai->dev);
	int ret = 0;

	data->is_stream_opened[tx] = true;
	if (data->is_stream_opened[tx] != sai->is_stream_opened[tx] ||
	    data->is_stream_opened[!tx] != sai->is_stream_opened[!tx]) {
		data->is_stream_opened[tx] = false;
		return -EBUSY;
	}

	if (!data->is_codec_master) {
		ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE, &imx_wm8960_rate_constraints);
		if (ret)
			return ret;
	}

	return ret;
}

static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	data->is_stream_opened[tx] = false;
}

static struct snd_soc_ops imx_hifi_ops = {
	.hw_params = imx_hifi_hw_params,
	.hw_free = imx_hifi_hw_free,
	.startup   = imx_hifi_startup,
	.shutdown  = imx_hifi_shutdown,
};

static int imx_wm8960_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd = list_first_entry(
		&card->rtd_list, struct snd_soc_pcm_runtime, list);
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);

	/*
	 * codec ADCLRC pin configured as GPIO, DACLRC pin is used as a frame
	 * clock for ADCs and DACs
	 */
	snd_soc_component_update_bits(codec_dai->component, WM8960_IFACE2, 1<<6, 1<<6);

	/* GPIO1 used as headphone detect output */
	snd_soc_component_update_bits(codec_dai->component, WM8960_ADDCTL4, 7<<4, 3<<4);

	/* Enable headphone jack detect */
	snd_soc_component_update_bits(codec_dai->component, WM8960_ADDCTL2, 1<<6, 1<<6);
	snd_soc_component_update_bits(codec_dai->component, WM8960_ADDCTL2, 1<<5, data->hp_det[1]<<5);
	snd_soc_component_update_bits(codec_dai->component, WM8960_ADDCTL4, 3<<2, data->hp_det[0]<<2);
	snd_soc_component_update_bits(codec_dai->component, WM8960_ADDCTL1, 3, 3);

	return 0;
}

static int be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_card *card = rtd->card;
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	struct snd_interval *rate;
	struct snd_mask *mask;

	if (!data->asrc_pdev)
		return -EINVAL;

	rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	rate->max = rate->min = data->asrc_rate;

	mask = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	snd_mask_none(mask);
	snd_mask_set(mask, data->asrc_format);

	return 0;
}

SND_SOC_DAILINK_DEFS(hifi,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "wm8960-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(hifi_fe,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(hifi_be,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "wm8960-hifi")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()));

static int of_parse_gpr(struct platform_device *pdev,
			struct imx_wm8960_data *data)
{
	int ret;
	struct of_phandle_args args;

	if (of_device_is_compatible(pdev->dev.of_node,
				    "fsl,imx7d-evk-wm8960"))
		return 0;

	ret = of_parse_phandle_with_fixed_args(pdev->dev.of_node,
					       "gpr", 3, 0, &args);
	if (ret) {
		dev_warn(&pdev->dev, "failed to get gpr property\n");
		return ret;
	}

	data->gpr = syscon_node_to_regmap(args.np);
	if (IS_ERR(data->gpr)) {
		ret = PTR_ERR(data->gpr);
		dev_err(&pdev->dev, "failed to get gpr regmap\n");
		return ret;
	}

	regmap_update_bits(data->gpr, args.args[0], args.args[1],
			   args.args[2]);

	return 0;
}

static int imx_wm8960_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np = NULL, *codec_np = NULL;
	struct platform_device *cpu_pdev;
	struct imx_wm8960_data *data;
	struct platform_device *asrc_pdev = NULL;
	struct device_node *asrc_np;
	u32 width;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	data->pdev = pdev;
	data->imx_hp_jack_pin.pin = "Headphone Jack";
	data->imx_hp_jack_pin.mask = SND_JACK_HEADPHONE;

	data->imx_hp_jack_gpio.name = "headphone detect";
	data->imx_hp_jack_gpio.report = SND_JACK_HEADPHONE;
	data->imx_hp_jack_gpio.debounce_time = 250;
	data->imx_hp_jack_gpio.invert = 0;

	data->imx_mic_jack_pin.pin = "Mic Jack";
	data->imx_mic_jack_pin.mask = SND_JACK_MICROPHONE;

	data->imx_mic_jack_gpio.name = "mic detect";
	data->imx_mic_jack_gpio.report = SND_JACK_MICROPHONE;
	data->imx_mic_jack_gpio.debounce_time = 250;
	data->imx_mic_jack_gpio.invert = 0;

	if (of_property_read_bool(pdev->dev.of_node, "codec-rpmsg"))
		data->is_codec_rpmsg = true;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	if (data->is_codec_rpmsg) {
		struct platform_device *codec_dev;

		codec_dev = of_find_device_by_node(codec_np);
		if (!codec_dev || !codec_dev->dev.driver) {
			dev_err(&pdev->dev, "failed to find codec platform device\n");
			ret = -EINVAL;
			goto fail;
		}

		data->codec_clk = devm_clk_get(&codec_dev->dev, "mclk");
		if (IS_ERR(data->codec_clk)) {
			ret = PTR_ERR(data->codec_clk);
			dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
			goto fail;
		}
	} else {
		struct i2c_client *codec_dev;

		codec_dev = of_find_i2c_device_by_node(codec_np);
		if (!codec_dev || !codec_dev->dev.driver) {
			dev_dbg(&pdev->dev, "failed to find codec platform device\n");
			ret = -EPROBE_DEFER;
			goto fail;
		}

		data->codec_clk = devm_clk_get(&codec_dev->dev, "mclk");
		if (IS_ERR(data->codec_clk)) {
			ret = PTR_ERR(data->codec_clk);
			dev_err(&pdev->dev, "failed to get codec clk: %d\n", ret);
			goto fail;
		}
	}

	if (of_property_read_bool(pdev->dev.of_node, "codec-master"))
		data->is_codec_master = true;

	if (of_property_read_bool(pdev->dev.of_node, "capture-only"))
		data->is_capture_only = true;

	if (of_property_read_bool(pdev->dev.of_node, "playback-only"))
		data->is_playback_only = true;

	if (data->is_capture_only && data->is_playback_only) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "failed for playback only and capture only\n");
		goto fail;
	}

	data->imx_wm8960_dai[0].name = "HiFi";
	data->imx_wm8960_dai[0].stream_name = "HiFi";
	data->imx_wm8960_dai[0].ops = &imx_hifi_ops;
	data->imx_wm8960_dai[0].cpus = hifi_cpus;
	data->imx_wm8960_dai[0].num_cpus = ARRAY_SIZE(hifi_cpus);
	data->imx_wm8960_dai[0].codecs = hifi_codecs;
	data->imx_wm8960_dai[0].num_codecs = ARRAY_SIZE(hifi_codecs);
	data->imx_wm8960_dai[0].platforms = hifi_platforms;
	data->imx_wm8960_dai[0].num_platforms = ARRAY_SIZE(hifi_platforms);

	data->imx_wm8960_dai[1].name = "HiFi-ASRC-FE";
	data->imx_wm8960_dai[1].stream_name = "HiFi-ASRC-FE";
	data->imx_wm8960_dai[1].dynamic = 1;
	data->imx_wm8960_dai[1].ignore_pmdown_time = 1;
	data->imx_wm8960_dai[1].dpcm_playback = 1;
	data->imx_wm8960_dai[1].dpcm_capture = 1;
	data->imx_wm8960_dai[1].dpcm_merged_chan = 1;
	data->imx_wm8960_dai[1].cpus = hifi_fe_cpus;
	data->imx_wm8960_dai[1].num_cpus = ARRAY_SIZE(hifi_fe_cpus);
	data->imx_wm8960_dai[1].codecs = hifi_fe_codecs;
	data->imx_wm8960_dai[1].num_codecs = ARRAY_SIZE(hifi_fe_codecs);
	data->imx_wm8960_dai[1].platforms = hifi_fe_platforms;
	data->imx_wm8960_dai[1].num_platforms = ARRAY_SIZE(hifi_fe_platforms);

	data->imx_wm8960_dai[2].name = "HiFi-ASRC-BE";
	data->imx_wm8960_dai[2].stream_name = "HiFi-ASRC-BE";
	data->imx_wm8960_dai[2].no_pcm = 1;
	data->imx_wm8960_dai[2].ignore_pmdown_time = 1;
	data->imx_wm8960_dai[2].dpcm_playback = 1;
	data->imx_wm8960_dai[2].dpcm_capture = 1;
	data->imx_wm8960_dai[2].ops = &imx_hifi_ops;
	data->imx_wm8960_dai[2].be_hw_params_fixup = be_hw_params_fixup;
	data->imx_wm8960_dai[2].cpus = hifi_be_cpus;
	data->imx_wm8960_dai[2].num_cpus = ARRAY_SIZE(hifi_be_cpus);
	data->imx_wm8960_dai[2].codecs = hifi_be_codecs;
	data->imx_wm8960_dai[2].num_codecs = ARRAY_SIZE(hifi_be_codecs);
	data->imx_wm8960_dai[2].platforms = hifi_be_platforms;
	data->imx_wm8960_dai[2].num_platforms = ARRAY_SIZE(hifi_be_platforms);

	if (data->is_capture_only) {
		data->imx_wm8960_dai[0].capture_only = true;
		data->imx_wm8960_dai[1].capture_only = true;
		data->imx_wm8960_dai[2].capture_only = true;
	}

	if (data->is_playback_only) {
		data->imx_wm8960_dai[0].playback_only = true;
		data->imx_wm8960_dai[1].playback_only = true;
		data->imx_wm8960_dai[2].playback_only = true;
	}

	ret = of_parse_gpr(pdev, data);
	if (ret)
		goto fail;

	of_property_read_u32_array(pdev->dev.of_node, "hp-det", data->hp_det, 2);

	asrc_np = of_parse_phandle(pdev->dev.of_node, "asrc-controller", 0);
	if (asrc_np) {
		asrc_pdev = of_find_device_by_node(asrc_np);
		data->asrc_pdev = asrc_pdev;
	}

	data->card.dai_link = data->imx_wm8960_dai;

	if (data->is_codec_rpmsg) {
		data->imx_wm8960_dai[0].codecs->name     = "rpmsg-audio-codec-wm8960";
		data->imx_wm8960_dai[0].codecs->dai_name = "rpmsg-wm8960-hifi";
	} else
		data->imx_wm8960_dai[0].codecs->of_node	= codec_np;

	data->imx_wm8960_dai[0].cpus->dai_name = dev_name(&cpu_pdev->dev);
	data->imx_wm8960_dai[0].platforms->of_node = cpu_np;

	if (!asrc_pdev) {
		data->card.num_links = 1;
	} else {
		data->imx_wm8960_dai[1].cpus->of_node = asrc_np;
		data->imx_wm8960_dai[1].platforms->of_node = asrc_np;
		if (data->is_codec_rpmsg) {
			data->imx_wm8960_dai[2].codecs->name     = "rpmsg-audio-codec-wm8960";
			data->imx_wm8960_dai[2].codecs->dai_name = "rpmsg-wm8960-hifi";
		} else
			data->imx_wm8960_dai[2].codecs->of_node	= codec_np;
		data->imx_wm8960_dai[2].cpus->dai_name = dev_name(&cpu_pdev->dev);
		data->card.num_links = 3;

		ret = of_property_read_u32(asrc_np, "fsl,asrc-rate",
				&data->asrc_rate);
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
			data->asrc_format = SNDRV_PCM_FORMAT_S24_LE;
		else
			data->asrc_format = SNDRV_PCM_FORMAT_S16_LE;
	}

	data->card.dev = &pdev->dev;
	data->card.owner = THIS_MODULE;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;
	data->card.dapm_widgets = imx_wm8960_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8960_dapm_widgets);

	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;

	data->card.late_probe = imx_wm8960_late_probe;

	snd_soc_card_set_drvdata(&data->card, data);
	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	data->imx_hp_jack_gpio.gpio = of_get_named_gpio_flags(pdev->dev.of_node,
							      "hp-det-gpios", 0,
							      &data->hp_active_low);

	data->imx_mic_jack_gpio.gpio = of_get_named_gpio_flags(pdev->dev.of_node,
							       "mic-det-gpios", 0,
							       &data->mic_active_low);

	if (gpio_is_valid(data->imx_hp_jack_gpio.gpio) &&
	    gpio_is_valid(data->imx_mic_jack_gpio.gpio) &&
	    data->imx_hp_jack_gpio.gpio == data->imx_mic_jack_gpio.gpio)
		data->is_headset_jack = true;

	if (gpio_is_valid(data->imx_hp_jack_gpio.gpio)) {
		if (data->is_headset_jack) {
			data->imx_hp_jack_pin.mask |= SND_JACK_MICROPHONE;
			data->imx_hp_jack_gpio.report |= SND_JACK_MICROPHONE;
		}

		data->imx_hp_jack_gpio.jack_status_check = hp_jack_status_check;
		data->imx_hp_jack_gpio.data = data;
		ret = imx_wm8960_jack_init(&data->card, &data->imx_hp_jack,
					   &data->imx_hp_jack_pin, &data->imx_hp_jack_gpio);
		if (ret) {
			dev_warn(&pdev->dev, "hp jack init failed (%d)\n", ret);
			goto out;
		}

		ret = device_create_file(&pdev->dev, &dev_attr_headphone);
		if (ret)
			dev_warn(&pdev->dev, "create hp attr failed (%d)\n", ret);
	}

	if (gpio_is_valid(data->imx_mic_jack_gpio.gpio)) {
		if (!data->is_headset_jack) {
			data->imx_mic_jack_gpio.jack_status_check = mic_jack_status_check;
			data->imx_mic_jack_gpio.data = data;
			ret = imx_wm8960_jack_init(&data->card, &data->imx_mic_jack,
						   &data->imx_mic_jack_pin, &data->imx_mic_jack_gpio);
			if (ret) {
				dev_warn(&pdev->dev, "mic jack init failed (%d)\n", ret);
				goto out;
			}
		}

		ret = device_create_file(&pdev->dev, &dev_attr_micphone);
		if (ret)
			dev_warn(&pdev->dev, "create mic attr failed (%d)\n", ret);
	}

#ifdef CONFIG_EXTCON
	wm8960_edev  = devm_extcon_dev_allocate(&pdev->dev, imx_wm8960_extcon_cables);
	if (IS_ERR(wm8960_edev)) {
		dev_err(&pdev->dev, "failed to allocate extcon device\n");
		goto fail;
	}
	ret = devm_extcon_dev_register(&pdev->dev,wm8960_edev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register extcon device\n");
		goto fail;
	}
#endif
out:
	ret = 0;
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_wm8960_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_micphone);
	device_remove_file(&pdev->dev, &dev_attr_headphone);

	return 0;
}

static const struct of_device_id imx_wm8960_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-wm8960", },
	{ .compatible = "fsl,imx7d-evk-wm8960"  },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8960_dt_ids);

static struct platform_driver imx_wm8960_driver = {
	.driver = {
		.name = "imx-wm8960",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm8960_dt_ids,
	},
	.probe = imx_wm8960_probe,
	.remove = imx_wm8960_remove,
};
static int __init imx_wm8960_init(void)
{
	return platform_driver_register(&imx_wm8960_driver);
}
late_initcall_sync(imx_wm8960_init);

static void __exit imx_wm8960_exit(void)
{
	platform_driver_unregister(&imx_wm8960_driver);
}
module_exit(imx_wm8960_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX WM8960 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm8960");
