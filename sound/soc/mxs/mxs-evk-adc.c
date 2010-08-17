/*
 * ASoC driver for Freescale MXS EVK board
 *
 * Author: Vladislav Buzov <vbuzov@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
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
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm/dma.h>
#include <mach/hardware.h>

#include "../codecs/mxs-adc-codec.h"
#include "mxs-adc.h"
#include "mxs-pcm.h"

/* mxs evk machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* HPR/HPL OUT --> Headphone Jack */
	{"Headphone Jack", NULL, "HPR"},
	{"Headphone Jack", NULL, "HPL"},

	/* SPEAKER OUT --> Ext Speaker */
	{"Ext Spk", NULL, "SPEAKER"},
};

static int mxs_evk_jack_func;
static int mxs_evk_spk_func;

static const char *jack_function[] = { "off", "on"};

static const char *spk_function[] = { "off", "on" };


static const struct soc_enum mxs_evk_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static int mxs_evk_get_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = mxs_evk_jack_func;
	return 0;
}

static int mxs_evk_set_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (mxs_evk_jack_func == ucontrol->value.enumerated.item[0])
		return 0;

	mxs_evk_jack_func = ucontrol->value.enumerated.item[0];
	if (mxs_evk_jack_func)
		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");

	snd_soc_dapm_sync(codec);
	return 1;
}

static int mxs_evk_get_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = mxs_evk_spk_func;
	return 0;
}

static int mxs_evk_set_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (mxs_evk_spk_func == ucontrol->value.enumerated.item[0])
		return 0;

	mxs_evk_spk_func = ucontrol->value.enumerated.item[0];
	if (mxs_evk_spk_func)
		snd_soc_dapm_enable_pin(codec, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(codec, "Ext Spk");

	snd_soc_dapm_sync(codec);
	return 1;
}
/* mxs evk card dapm widgets */
static const struct snd_soc_dapm_widget mxs_evk_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static const struct snd_kcontrol_new mxs_evk_controls[] = {
	SOC_ENUM_EXT("HP Playback Switch", mxs_evk_enum[0], mxs_evk_get_jack,
		     mxs_evk_set_jack),
	SOC_ENUM_EXT("Speaker Playback Switch", mxs_evk_enum[1],
		     mxs_evk_get_spk, mxs_evk_set_spk),
};

static int mxs_evk_codec_init(struct snd_soc_codec *codec)
{
	int i, ret;
	/* Add mxs evk specific controls */
	snd_soc_add_controls(codec, mxs_evk_controls,
		ARRAY_SIZE(mxs_evk_controls));

	/* Add mxs evk specific widgets */
	snd_soc_dapm_new_controls(codec, mxs_evk_dapm_widgets,
				  ARRAY_SIZE(mxs_evk_dapm_widgets));

	/* Set up mxs evk specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);
	/* default on */
	mxs_evk_jack_func = 1;
	mxs_evk_spk_func = 1;

	return ret;
}
/* mxs evk dac/adc audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link mxs_evk_codec_dai = {
	.name = "MXS ADC/DAC",
	.stream_name = "MXS ADC/DAC",
	.cpu_dai = &mxs_adc_dai,
	.codec_dai = &mxs_codec_dai,
	.init = mxs_evk_codec_init,
};

/* mxs evk audio machine driver */
static struct snd_soc_card snd_soc_card_mxs_evk = {
	.name = "MXS EVK",
	.platform = &mxs_soc_platform,
	.dai_link = &mxs_evk_codec_dai,
	.num_links = 1,
};

/* mxs evk audio subsystem */
static struct snd_soc_device mxs_evk_snd_devdata = {
	.card = &snd_soc_card_mxs_evk,
	.codec_dev = &soc_codec_dev_mxs,
};

static struct platform_device *mxs_evk_snd_device;

static int __init mxs_evk_adc_init(void)
{
	int ret = 0;

	mxs_evk_snd_device = platform_device_alloc("soc-audio", 0);
	if (!mxs_evk_snd_device)
		return -ENOMEM;

	platform_set_drvdata(mxs_evk_snd_device,
			&mxs_evk_snd_devdata);
	mxs_evk_snd_devdata.dev = &mxs_evk_snd_device->dev;
	mxs_evk_snd_device->dev.platform_data =
		&mxs_evk_snd_devdata;

	ret = platform_device_add(mxs_evk_snd_device);
	if (ret)
		platform_device_put(mxs_evk_snd_device);

	return ret;
}

static void __exit mxs_evk_adc_exit(void)
{
	platform_device_unregister(mxs_evk_snd_device);
}

module_init(mxs_evk_adc_init);
module_exit(mxs_evk_adc_exit);

MODULE_AUTHOR("Vladislav Buzov");
MODULE_DESCRIPTION("MXS EVK board ADC/DAC driver");
MODULE_LICENSE("GPL");
