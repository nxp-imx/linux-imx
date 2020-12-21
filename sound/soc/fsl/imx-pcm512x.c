/* i.MX pcm512x audio support
 *
 * Copyright 2020 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/soc-dapm.h>

#include "fsl_sai.h"
#include "../codecs/pcm512x.h"

#define DAC_CLK_EXT_44K 22579200UL
#define DAC_CLK_EXT_48K 24576000UL

struct imx_pcm512x_data {
	struct snd_soc_dai_link dai_link[3];
	struct snd_soc_card card;
	struct snd_soc_codec_conf *codec_conf;
	struct gpio_desc *mute_gpio;
	int num_codec_conf;
	unsigned int slots;
	unsigned int slot_width;
	unsigned int daifmt;
	bool dac_sclk;
	bool dac_pluspro;
	bool dac_led_status;
	bool dac_gain_limit;
	bool dac_gpio_unmute;
	bool dac_auto_mute;
	bool one2one_ratio;
	bool tdm_mode;
};

enum ext_osc {
	DAC_CLK_INT,
	DAC_CLK_EXT_44EN,
	DAC_CLK_EXT_48EN,
};

static const struct imx_pcm512x_fs_map {
	unsigned int rmin;
	unsigned int rmax;
	unsigned int wmin;
	unsigned int wmax;
} fs_map[] = {
	/* Normal, < 32kHz */
	{ .rmin = 8000,   .rmax = 24000,  .wmin = 1024, .wmax = 1024, },
	/* Normal, 32kHz */
	{ .rmin = 32000,  .rmax = 32000,  .wmin = 256,  .wmax = 1024, },
	/* Normal */
	{ .rmin = 44100,  .rmax = 48000,  .wmin = 256,  .wmax = 768,  },
	/* Double */
	{ .rmin = 88200,  .rmax = 96000,  .wmin = 256,  .wmax = 512,  },
	/* Quad */
	{ .rmin = 176400, .rmax = 192000, .wmin = 128,  .wmax = 256,  },
	/* Oct */
	{ .rmin = 352800, .rmax = 384000, .wmin = 32,   .wmax = 128,  },
	/* Hex */
	{ .rmin = 705600, .rmax = 768000, .wmin = 16,   .wmax = 64,   },
};

static const u32 pcm512x_rates[] = {
	8000, 11025, 16000, 22050, 32000,
	44100, 48000, 64000, 88200, 96000,
	176400, 192000, 352800, 384000,
};

static int imx_pcm512x_select_ext_clk(struct snd_soc_component *comp,
				      int ext_osc)
{
	switch(ext_osc) {
	case DAC_CLK_INT:
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x24, 0x00);
		break;
	case DAC_CLK_EXT_44EN:
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x24, 0x20);
		break;
	case DAC_CLK_EXT_48EN:
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x24, 0x04);
		break;
	}

	return 0;
}

static bool imx_pcm512x_is_sclk(struct snd_soc_component *comp)
{
	unsigned int sclk;

	snd_soc_component_read(comp, PCM512x_RATE_DET_4, &sclk);

	return (!(sclk & 0x40));
}

static bool imx_pcm512x_is_sclk_sleep(struct snd_soc_component *comp)
{
	msleep(2);
	return imx_pcm512x_is_sclk(comp);
}

static int imx_pcm512x_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(card);
	int ret;

	if (data->dac_gain_limit) {
		ret = snd_soc_limit_volume(card, "Digital Playback Volume", 207);
		if (ret)
			dev_warn(card->dev, "fail to set volume limit");
	}

	return 0;
}

static int imx_pcm512x_set_bias_level(struct snd_soc_card *card,
	struct snd_soc_dapm_context *dapm, enum snd_soc_bias_level level)
{
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *codec_dai;

	rtd = snd_soc_get_pcm_runtime(card, card->dai_link[0].name);
	codec_dai = rtd->codec_dai;

	if (dapm->dev != codec_dai->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		if (dapm->bias_level != SND_SOC_BIAS_STANDBY)
			break;
		/* unmute amp */
		gpiod_set_value_cansleep(data->mute_gpio, 1);
		break;
	case SND_SOC_BIAS_STANDBY:
		if (dapm->bias_level != SND_SOC_BIAS_PREPARE)
			break;
		/* mute amp */
		gpiod_set_value_cansleep(data->mute_gpio, 0);
	default:
		break;
	}

	return 0;
}

static unsigned long pcm512x_get_mclk_rate(struct snd_pcm_substream *substream,
					  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(rtd->card);
	unsigned int width = data->slots * data->slot_width;
	unsigned int rate = params_rate(params);
	int i;

	for (i = 0; i < ARRAY_SIZE(fs_map); i++) {
		if (rate >= fs_map[i].rmin && rate <= fs_map[i].rmax) {
			width = max(width, fs_map[i].wmin);
			width = min(width, fs_map[i].wmax);
			/* Adjust SAI bclk:mclk ratio */
			width *= data->one2one_ratio ? 1 : 2;

			return rate * width;
		}
	}

	/* Let DAI manage clk frequency by default */
	return 0;
}

static int imx_pcm512x_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_component *comp = codec_dai->component;
	struct snd_soc_card *card = rtd->card;
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(card);
	unsigned int channels = params_channels(params);
	unsigned int sample_rate = params_rate(params);
	unsigned long mclk_freq;
	int ret, i;

	data->slots = 2;
	data->slot_width = params_physical_width(params);

	ret = snd_soc_dai_set_fmt(cpu_dai, data->daifmt);
	if (ret) {
		dev_err(card->dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai,
				       BIT(channels) - 1, BIT(channels) - 1,
				       data->slots, data->slot_width);
	if (ret) {
		dev_err(card->dev, "failed to set cpu dai tdm slot: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, data->slots * data->slot_width);
	if (ret) {
		dev_err(card->dev, "failed to set cpu dai bclk ratio\n");
		return ret;
	}

	for (i = 0; i < rtd->num_codecs; i++) {
		struct snd_soc_dai *codec_dai = rtd->codec_dai;

		ret = snd_soc_dai_set_fmt(codec_dai, data->daifmt);
		if (ret) {
			dev_err(card->dev, "failed to set codec dai[%d] fmt: %d\n",
					i, ret);
			return ret;
		}

		ret = snd_soc_dai_set_bclk_ratio(codec_dai,
					data->slots * data->slot_width);
		if (ret) {
			dev_err(card->dev, "failed to set cpu dai bclk ratio\n");
			return ret;
		}
	}

	/* set MCLK freq */
	if (data->dac_pluspro && data->dac_sclk) {
		if (do_div(sample_rate, 8000)) {
			mclk_freq = DAC_CLK_EXT_44K;
			imx_pcm512x_select_ext_clk(comp, DAC_CLK_EXT_44EN);
			ret = snd_soc_dai_set_sysclk(codec_dai,
				PCM512x_SYSCLK_MCLK1, mclk_freq, SND_SOC_CLOCK_IN);
		} else {
			mclk_freq = DAC_CLK_EXT_48K;
			imx_pcm512x_select_ext_clk(comp, DAC_CLK_EXT_48EN);
			ret = snd_soc_dai_set_sysclk(codec_dai,
				PCM512x_SYSCLK_MCLK2, mclk_freq, SND_SOC_CLOCK_IN);
		}
		if (ret < 0)
			dev_err(card->dev, "failed to set cpu dai mclk rate (%lu): %d\n",
				mclk_freq, ret);
	} else {
		mclk_freq = pcm512x_get_mclk_rate(substream, params);
		ret = snd_soc_dai_set_sysclk(cpu_dai, FSL_SAI_CLK_MAST1,
					     mclk_freq, SND_SOC_CLOCK_OUT);
		if (ret < 0)
			dev_err(card->dev, "failed to set cpu dai mclk1 rate (%lu): %d\n",
				mclk_freq, ret);
	}

	return ret;
}

static int imx_pcm512x_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(card);
	static struct snd_pcm_hw_constraint_list constraint_rates;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_component *comp = codec_dai->component;
	bool ext_44sclk, ext_48sclk, ext_nosclk;
	int ret;

	constraint_rates.list = pcm512x_rates;
	constraint_rates.count = ARRAY_SIZE(pcm512x_rates);

	ret = snd_pcm_hw_constraint_list(runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &constraint_rates);
	if (ret)
		return ret;

	if (data->dac_led_status) {
		snd_soc_component_update_bits(comp, PCM512x_GPIO_EN, 0x08, 0x08);
		snd_soc_component_update_bits(comp, PCM512x_GPIO_OUTPUT_4, 0x0f, 0x02);
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x08, 0x08);
	}

	if (data->dac_sclk) {
		snd_soc_component_update_bits(comp, PCM512x_GPIO_EN, 0x24, 0x24);
		snd_soc_component_update_bits(comp, PCM512x_GPIO_OUTPUT_3, 0x0f, 0x02);
		snd_soc_component_update_bits(comp, PCM512x_GPIO_OUTPUT_6, 0x0f, 0x02);

		imx_pcm512x_select_ext_clk(comp, DAC_CLK_EXT_44EN);
		ext_44sclk = imx_pcm512x_is_sclk_sleep(comp);

		imx_pcm512x_select_ext_clk(comp, DAC_CLK_EXT_48EN);
		ext_48sclk = imx_pcm512x_is_sclk_sleep(comp);

		imx_pcm512x_select_ext_clk(comp, DAC_CLK_INT);
		ext_nosclk = imx_pcm512x_is_sclk_sleep(comp);

		data->dac_pluspro = (ext_44sclk && ext_48sclk && !ext_nosclk);
	}

	return 0;
}

static void imx_pcm512x_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_component *comp = codec_dai->component;

	if (data->dac_led_status)
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x08, 0x00);
}

static struct snd_soc_ops imx_pcm512x_ops = {
	.hw_params = imx_pcm512x_hw_params,
	.startup = imx_pcm512x_startup,
	.shutdown = imx_pcm512x_shutdown,
};

SND_SOC_DAILINK_DEFS(hifi,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "pcm512x-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link_component pcm512x_codecs[] = {
	{
		/* Playback */
		.dai_name = "pcm512x-hifi",
	},
};

static struct snd_soc_dai_link imx_pcm512x_dai[] = {
	{
		.name = "pcm512x-audio",
		.stream_name = "audio",
		.codecs = pcm512x_codecs,
		.num_codecs = 1,
		.ops = &imx_pcm512x_ops,
		.init = imx_pcm512x_dai_init,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(hifi),
	},
};

static int imx_pcm512x_probe(struct platform_device *pdev)
{
	struct device_node *bitclkmaster, *framemaster = NULL;
	struct device_node *cpu_np, *codec_np = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct platform_device *cpu_pdev = NULL;
	struct snd_soc_dai_link_component *comp;
	struct imx_pcm512x_data *data;
	struct i2c_client *codec_dev;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	comp = devm_kzalloc(&pdev->dev, 3 * sizeof(*comp), GFP_KERNEL);
	if (!comp)
		return -ENOMEM;

	cpu_np = of_parse_phandle(np, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "audio dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np = of_parse_phandle(np, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "audio codec phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev || !codec_dev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}

	data->dac_gain_limit = of_property_read_bool(np, "dac,24db_digital_gain");
	data->dac_auto_mute = of_property_read_bool(np, "dac,auto_mute_amp");
	data->dac_gpio_unmute = of_property_read_bool(np, "dac,unmute_amp");
	data->dac_led_status = of_property_read_bool(np, "dac,led_status");

	if (data->dac_auto_mute || data->dac_gpio_unmute) {
		data->mute_gpio = devm_gpiod_get_optional(&pdev->dev,
						"mute-amp", GPIOD_OUT_LOW);
		if (IS_ERR(data->mute_gpio)) {
			dev_err(&pdev->dev, "failed to get mute amp gpio\n");
			ret = PTR_ERR(data->mute_gpio);
			goto fail;
		}
	}

	if (data->dac_auto_mute && data->dac_gpio_unmute)
		data->card.set_bias_level = imx_pcm512x_set_bias_level;

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	memcpy(data->dai_link, imx_pcm512x_dai,
	       sizeof(struct snd_soc_dai_link) * ARRAY_SIZE(data->dai_link));

	data->card.owner = THIS_MODULE;
	data->card.dev = &pdev->dev;
	data->card.dai_link = data->dai_link;
	data->card.num_links = 1;

	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret) {
		dev_err(&pdev->dev, "failed to find card model name\n");
		goto fail;
	}

	if (of_property_read_bool(np, "audio-routing")) {
		ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
		if (ret) {
			dev_err(&pdev->dev, "failed to parse audio-routing\n");
			goto fail;
		}
	}

	if (of_property_read_bool(np, "audio-widgets")) {
		ret = snd_soc_of_parse_audio_simple_widgets(&data->card, "audio-widgets");
		if (ret) {
			dev_err(&pdev->dev, "failed to parse audio-widgets\n");
			goto fail;
		}
	}

	data->daifmt = snd_soc_of_parse_daifmt(np, NULL, &bitclkmaster, &framemaster);
	data->daifmt &= ~SND_SOC_DAIFMT_MASTER_MASK;

	if (codec_np == bitclkmaster)
		data->daifmt |= (codec_np == framemaster) ?
			SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBM_CFS;
	else
		data->daifmt |= (codec_np == framemaster) ?
			SND_SOC_DAIFMT_CBS_CFM : SND_SOC_DAIFMT_CBS_CFS;

	if (!bitclkmaster)
		of_node_put(bitclkmaster);
	if (!framemaster)
		of_node_put(framemaster);

	if (of_property_read_bool(codec_np, "clocks"))
		data->dac_sclk = true;

	data->dai_link[0].cpus = &comp[0];
	data->dai_link[0].num_cpus = 1;
	data->dai_link[0].codecs = &comp[1];
	data->dai_link[0].num_codecs = 1;
	data->dai_link[0].platforms = &comp[2];
	data->dai_link[0].num_platforms = 1;

	data->dai_link[0].cpus->of_node = cpu_np;
	data->dai_link[0].platforms->of_node = cpu_np;
	data->dai_link[0].codecs->of_node = codec_np;
	data->dai_link[0].codecs->dai_name = "pcm512x-hifi";
	data->dai_link[0].dai_fmt = data->daifmt;

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	if (data->dac_gpio_unmute && data->dac_auto_mute)
		gpiod_set_value_cansleep(data->mute_gpio, 1);

	ret = 0;
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_pcm512x_remove(struct platform_device *pdev)
{
	struct imx_pcm512x_data *data = platform_get_drvdata(pdev);

	if (data->mute_gpio)
		gpiod_set_value_cansleep(data->mute_gpio, 0);

	return snd_soc_unregister_card(&data->card);
}

static const struct of_device_id imx_pcm512x_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-pcm512x", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx_pcm512x_dt_ids);

static struct platform_driver imx_pcm512x_driver = {
	.driver = {
		.name = "imx-pcm512x",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_pcm512x_dt_ids,
	},
	.probe = imx_pcm512x_probe,
	.remove = imx_pcm512x_remove,
};
module_platform_driver(imx_pcm512x_driver);

MODULE_DESCRIPTION("NXP i.MX pcm512x ASoC machine driver");
MODULE_AUTHOR("Adrian Alonso <adrian.alonso@nxp.com>");
MODULE_ALIAS("platform:imx-pcm512x");
MODULE_LICENSE("GPL");
