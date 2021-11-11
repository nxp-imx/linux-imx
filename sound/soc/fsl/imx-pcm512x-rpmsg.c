/* i.MX pcm512x audio support
 *
 * Copyright 2020-2021 NXP
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
#include <linux/of_reserved_mem.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>
#include <sound/simple_card_utils.h>
#include <linux/extcon-provider.h>

#include "imx-pcm-rpmsg.h"
#include "../codecs/pcm512x.h"

#define DAC_CLK_EXT_44K 22579200UL
#define DAC_CLK_EXT_48K 24576000UL

struct imx_pcm512x_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	struct simple_util_jack hp_jack;
	unsigned int sysclk;
	struct gpio_desc *mute_gpio;
	unsigned int slots;
	unsigned int slot_width;
	unsigned int daifmt;
	bool dac_sclk;
	bool dac_pluspro;
	bool dac_led_status;
	bool dac_gain_limit;
	bool dac_gpio_unmute;
	bool dac_auto_mute;
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

#ifdef CONFIG_EXTCON
static const unsigned int imx_rpmsg_extcon_cables[] = {
	EXTCON_JACK_LINE_OUT,
	EXTCON_NONE,
};
#endif

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

	sclk = snd_soc_component_read(comp, PCM512x_RATE_DET_4);

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

	rtd = snd_soc_get_pcm_runtime(card, card->dai_link);
	codec_dai = snd_soc_rtd_to_codec(rtd, 0);

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
		fallthrough;
	default:
		break;
	}

	return 0;
}

static int imx_pcm512x_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_component *comp = codec_dai->component;
	struct snd_soc_card *card = rtd->card;
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(card);
	unsigned int sample_rate = params_rate(params);
	unsigned long mclk_freq;
	int ret, i;

	data->slots = 2;
	data->slot_width = params_physical_width(params);

	for (i = 0; i < rtd->dai_link->num_codecs; i++) {
		struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, i);

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
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
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
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
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
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "rpmsg-pcm512x-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link imx_pcm512x_dai[] = {
	{
		.name = "rpmsg-audio-codec-pcm512x",
		.stream_name = "rpmsg hifi",
		.num_codecs = 1,
		.ops = &imx_pcm512x_ops,
		.init = imx_pcm512x_dai_init,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(hifi),
	},
};

static int  tpa6130init(struct snd_soc_component *component){

	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(component), "HPLEFT");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(component), "HPRIGHT");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(component), "LEFTIN");
	snd_soc_dapm_ignore_suspend(snd_soc_component_get_dapm(component), "RIGHTIN");

	return 0;
}

/* aux device for optional headphone amp */
static struct snd_soc_aux_dev hifiberry_dacplus_aux_devs[] = {
        {
                .dlc = {
                        .name = "tpa6130a2.2-0060",
                },
		.init = tpa6130init,
        },
};

static int hb_hp_detect(void)
{
        struct i2c_adapter *adap = i2c_get_adapter(2);
        int ret;
        struct i2c_client tpa_i2c_client = {
                .addr = 0x60,
                .adapter = adap,
        };

        if (!adap)
                return -EPROBE_DEFER;   /* I2C module not yet available */

        ret = i2c_smbus_read_byte(&tpa_i2c_client) >= 0;
        i2c_put_adapter(adap);
        return ret;
};

static const struct snd_soc_dapm_widget imx_rpmsg_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("Main MIC", NULL),
};

int imx_pcm512x_rpmsg_init_data(struct platform_device *pdev, void ** _data){

	struct imx_pcm512x_data *data;

	if(*_data != NULL){
		 devm_kfree(&pdev->dev, *_data);
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memcpy(&data->dai, imx_pcm512x_dai, sizeof(struct snd_soc_dai_link));

	*_data = data;

	return 0;
}

int imx_pcm512x_rpmsg_probe(struct platform_device *pdev, void *_data){

	struct imx_pcm512x_data *data = (struct imx_pcm512x_data *)_data;
	struct snd_soc_dai *cpu_dai;
	struct device_node *np = NULL;
	struct device_node *bitclkmaster = NULL, *framemaster = NULL, *codec_np = NULL;
	int    ret = 0;

	data->dai.cpus->dai_name = pdev->dev.platform_data;
	cpu_dai = snd_soc_find_dai(data->dai.cpus);
	if (!cpu_dai) {
		ret = -EPROBE_DEFER;
		goto fail;
	}
	np = cpu_dai->dev->of_node;
	if (!np) {
		dev_err(&pdev->dev, "failed to parse CPU DAI device node\n");
		ret = -ENODEV;
		goto fail;
	}

	codec_np = data->dai.codecs->of_node;
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

        /* probe for head phone amp */
        ret = hb_hp_detect();
        if (ret) {
		dev_info(&pdev->dev, "found phone amp \n");
                data->card.aux_dev = hifiberry_dacplus_aux_devs;
                data->card.num_aux_devs =
                                ARRAY_SIZE(hifiberry_dacplus_aux_devs);
        }

	data->daifmt = snd_soc_daifmt_parse_format(np, NULL);
	snd_soc_daifmt_parse_clock_provider_as_phandle(np, NULL, &bitclkmaster, &framemaster);

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

	data->dai.dai_fmt = data->daifmt;


	if (data->dac_gpio_unmute && data->dac_auto_mute)
		gpiod_set_value_cansleep(data->mute_gpio, 1);

fail:
	return ret;
}

EXPORT_SYMBOL(imx_pcm512x_rpmsg_init_data);
EXPORT_SYMBOL(imx_pcm512x_rpmsg_probe);

MODULE_DESCRIPTION("NXP i.MX pcm512x rpmsg ASoC machine driver");
MODULE_AUTHOR("Tom Zheng <haidong.zheng@nxp.com>");
MODULE_LICENSE("GPL v2");



