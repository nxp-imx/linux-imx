/*
 * ak5702.c  --  AK5702 Soc Audio driver
 *
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "ak5702.h"

#define AK5702_VERSION "0.1"

/* codec private data */
struct ak5702_priv {
	unsigned int sysclk;
};

/*
 * ak5702 register cache
 */
static const u16 ak5702_reg[AK5702_CACHEREGNUM] = {
	0x0000, 0x0024, 0x0000, 0x0001, 0x0023, 0x001f,
	0x0000, 0x0001, 0x0091, 0x0000, 0x00e1, 0x0000,
	0x00a0, 0x0000, 0x0000, 0x0000, 0x0001, 0x0020,
	0x0000, 0x0000, 0x0001, 0x0091, 0x0000, 0x00e1,
	0x0000,
};

/*
 * read ak5702 register cache
 */
static inline unsigned int ak5702_read_reg_cache(struct snd_soc_codec *codec,
						 unsigned int reg)
{
	u16 *cache = codec->reg_cache;

	if (reg >= AK5702_CACHEREGNUM)
		return -1;
	return cache[reg];
}

static inline unsigned int ak5702_read(struct snd_soc_codec *codec,
				       unsigned int reg)
{
	u8 data;
	data = reg;

	if (i2c_master_send(codec->control_data, &data, 1) != 1)
		return -EIO;
	if (i2c_master_recv(codec->control_data, &data, 1) != 1)
		return -EIO;

	return data;
};

/*
 * write ak5702 register cache
 */
static inline void ak5702_write_reg_cache(struct snd_soc_codec *codec,
					  u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;

	if (reg >= AK5702_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the AK5702 register space
 */
static int ak5702_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int value)
{
	u8 data[2];

	data[0] = reg & 0xff;
	data[1] = value & 0xff;

	ak5702_write_reg_cache(codec, reg, value);
	if (i2c_master_send(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -EIO;
}

static const char *ak5702_mic_gain[] = { "0dB", "+15dB", "+30dB", "+36dB" };
static const char *ak5702_adca_left_type[] = {
	"Single-ended", "Full-differential" };
static const char *ak5702_adca_right_type[] = {
	"Single-ended", "Full-differential" };
static const char *ak5702_adcb_left_type[] = {
	"Single-ended", "Full-differential" };
static const char *ak5702_adcb_right_type[] = {
	"Single-ended", "Full-differential" };
static const char *ak5702_adca_left_input[] = { "LIN1", "LIN2" };
static const char *ak5702_adca_right_input[] = { "RIN1", "RIN2" };
static const char *ak5702_adcb_left_input[] = { "LIN3", "LIN4" };
static const char *ak5702_adcb_right_input[] = { "RIN3", "RIN4" };

static const struct soc_enum ak5702_enum[] = {
	SOC_ENUM_SINGLE(AK5702_MICG1, 0, 4, ak5702_mic_gain),
	SOC_ENUM_SINGLE(AK5702_MICG2, 0, 4, ak5702_mic_gain),

	SOC_ENUM_SINGLE(AK5702_SIG1, 0, 2, ak5702_adca_left_input),
	SOC_ENUM_SINGLE(AK5702_SIG1, 1, 2, ak5702_adca_right_input),
	SOC_ENUM_SINGLE(AK5702_SIG2, 0, 2, ak5702_adcb_left_input),
	SOC_ENUM_SINGLE(AK5702_SIG1, 1, 2, ak5702_adcb_right_input),

	SOC_ENUM_SINGLE(AK5702_SIG1, 2, 2, ak5702_adca_left_type),
	SOC_ENUM_SINGLE(AK5702_SIG1, 3, 2, ak5702_adca_right_type),
	SOC_ENUM_SINGLE(AK5702_SIG2, 2, 2, ak5702_adcb_left_type),
	SOC_ENUM_SINGLE(AK5702_SIG2, 3, 2, ak5702_adcb_right_type),
};

static const struct snd_kcontrol_new ak5702_snd_controls[] = {
	SOC_SINGLE("ADCA Left Vol", AK5702_LVOL1, 0, 242, 0),
	SOC_SINGLE("ADCA Right Vol", AK5702_RVOL1, 0, 242, 0),
	SOC_SINGLE("ADCB Left Vol", AK5702_LVOL2, 0, 242, 0),
	SOC_SINGLE("ADCB Right Vol", AK5702_RVOL2, 0, 242, 0),

	SOC_ENUM("MIC-AmpA Gain", ak5702_enum[0]),
	SOC_ENUM("MIC-AmpB Gain", ak5702_enum[1]),

	SOC_ENUM("ADCA Left Source", ak5702_enum[2]),
	SOC_ENUM("ADCA Right Source", ak5702_enum[3]),
	SOC_ENUM("ADCB Left Source", ak5702_enum[4]),
	SOC_ENUM("ADCB Right Source", ak5702_enum[5]),

	SOC_ENUM("ADCA Left Type", ak5702_enum[6]),
	SOC_ENUM("ADCA Right Type", ak5702_enum[7]),
	SOC_ENUM("ADCB Left Type", ak5702_enum[8]),
	SOC_ENUM("ADCB Right Type", ak5702_enum[9]),
};

/* ak5702 dapm widgets */
static const struct snd_soc_dapm_widget ak5702_dapm_widgets[] = {
	SND_SOC_DAPM_ADC("ADCA Left", "Capture", AK5702_PM1, 0, 0),
	SND_SOC_DAPM_ADC("ADCA Right", "Capture", AK5702_PM1, 1, 0),
	SND_SOC_DAPM_ADC("ADCB Left", "Capture", AK5702_PM2, 0, 0),
	SND_SOC_DAPM_ADC("ADCB Right", "Capture", AK5702_PM2, 1, 0),

	SND_SOC_DAPM_INPUT("ADCA Left Input"),
	SND_SOC_DAPM_INPUT("ADCA Right Input"),
	SND_SOC_DAPM_INPUT("ADCB Left Input"),
	SND_SOC_DAPM_INPUT("ADCB Right Input"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"ADCA Left", NULL, "ADCA Left Input"},
	{"ADCA Right", NULL, "ADCA Right Input"},
	{"ADCB Left", NULL, "ADCB Left Input"},
	{"ADCB Right", NULL, "ADCB Right Input"},
};

static int ak5702_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, ak5702_dapm_widgets,
				  ARRAY_SIZE(ak5702_dapm_widgets));
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	return 0;
}

static int ak5702_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 fs = 0;
	u8 value;

	switch (freq) {
	case 8000:
		fs = 0x0;
		break;
	case 11025:
		fs = 0x05;
		break;
	case 12000:
		fs = 0x01;
		break;
	case 16000:
		fs = 0x02;
		break;
	case 22050:
		fs = 0x07;
		break;
	case 24000:
		fs = 0x03;
		break;
	case 32000:
		fs = 0x0a;
		break;
	case 44100:
		fs = 0x0f;
		break;
	case 48000:
		fs = 0x0b;
		break;
	default:
		return -EINVAL;
	}

	value = ak5702_read_reg_cache(codec, AK5702_FS1);
	value &= (~AK5702_FS1_FS_MASK);
	value |= fs;
	ak5702_write(codec, AK5702_FS1, value);
	return 0;
}

static int ak5702_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 fmt1 = 0;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		fmt1 = AK5702_FMT1_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		fmt1 = AK5702_FMT1_MSB;
		break;
	default:
		return -EINVAL;
	}

	ak5702_write(codec, AK5702_FMT1, fmt1);
	ak5702_write(codec, AK5702_FMT2, AK5702_FMT2_STEREO);
	return 0;
}

static int ak5702_set_dai_pll(struct snd_soc_dai *codec_dai, int pll_id,
			      int source, unsigned int freq_in,
			      unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 reg = 0;

	reg = ak5702_read_reg_cache(codec, AK5702_PLL1);
	switch (pll_id) {
	case AK5702_PLL_POWERDOWN:
		reg &= (~AK5702_PLL1_PM_MASK);
		reg |= AK5702_PLL1_POWERDOWN;
		break;
	case AK5702_PLL_MASTER:
		reg &= (~AK5702_PLL1_MODE_MASK);
		reg |= AK5702_PLL1_MASTER;
		reg |= AK5702_PLL1_POWERUP;
		break;
	case AK5702_PLL_SLAVE:
		reg &= (~AK5702_PLL1_MODE_MASK);
		reg |= AK5702_PLL1_SLAVE;
		reg |= AK5702_PLL1_POWERUP;
		break;
	default:
		return -ENODEV;
	}

	switch (freq_in) {
	case 11289600:
		reg &= (~AK5702_PLL1_PLL_MASK);
		reg |= AK5702_PLL1_11289600;
		break;
	case 12000000:
		reg &= (~AK5702_PLL1_PLL_MASK);
		reg |= AK5702_PLL1_12000000;
		break;
	case 12288000:
		reg &= (~AK5702_PLL1_PLL_MASK);
		reg |= AK5702_PLL1_12288000;
		break;
	case 19200000:
		reg &= (~AK5702_PLL1_PLL_MASK);
		reg |= AK5702_PLL1_19200000;
		break;
	default:
		return -ENODEV;
	}

	ak5702_write(codec, AK5702_PLL1, reg);
	return 0;
}

static int ak5702_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
				 int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 reg = 0;

	if (div_id == AK5702_BCLK_CLKDIV) {
		reg = ak5702_read_reg_cache(codec, AK5702_FS1);
		switch (div) {
		case AK5702_BCLK_DIV_32:
			reg &= (~AK5702_FS1_BCKO_MASK);
			reg |= AK5702_FS1_BCKO_32FS;
			ak5702_write(codec, AK5702_FS1, reg);
			break;
		case AK5702_BCLK_DIV_64:
			reg &= (~AK5702_FS1_BCKO_MASK);
			reg |= AK5702_FS1_BCKO_64FS;
			ak5702_write(codec, AK5702_FS1, reg);
			break;
		default:
			return -EINVAL;
		}
	}
	return 0;
}

static int ak5702_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	u8 reg = 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		reg = ak5702_read_reg_cache(codec, AK5702_PM1);
		ak5702_write(codec, AK5702_PM1, reg | AK5702_PM1_PMVCM);
		reg = ak5702_read_reg_cache(codec, AK5702_PLL1);
		reg = reg | AK5702_PLL1_POWERUP | AK5702_PLL1_MASTER;
		ak5702_write(codec, AK5702_PLL1, reg);
		break;
	case SND_SOC_BIAS_STANDBY:
		reg = ak5702_read_reg_cache(codec, AK5702_PM1);
		ak5702_write(codec, AK5702_PM1, reg | AK5702_PM1_PMVCM);
		reg = ak5702_read_reg_cache(codec, AK5702_PLL1);
		ak5702_write(codec, AK5702_PLL1, reg & (~AK5702_PLL1_POWERUP));
		break;
	case SND_SOC_BIAS_OFF:
		reg = ak5702_read_reg_cache(codec, AK5702_PM1);
		ak5702_write(codec, AK5702_PM1, reg & (~AK5702_PM1_PMVCM));
		break;
	}

	codec->bias_level = level;
	return 0;
}

static int ak5702_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	ak5702_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int ak5702_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	int i;

	/* Bring the codec back up to standby first to minimise pop/clicks */
	ak5702_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	ak5702_set_bias_level(codec, codec->suspend_bias_level);

	/* Sync back everything else */
	for (i = 0; i < ARRAY_SIZE(ak5702_reg); i++)
		ak5702_write(codec, i, ak5702_reg[i]);

	return 0;
}

#define AK5702_RATES	SNDRV_PCM_RATE_8000_48000
#define AK5702_FORMATS	SNDRV_PCM_FMTBIT_S16_LE

struct snd_soc_dai_ops ak5702_ops = {
	.set_fmt = ak5702_set_dai_fmt,
	.set_sysclk = ak5702_set_dai_sysclk,
	.set_clkdiv = ak5702_set_dai_clkdiv,
	.set_pll = ak5702_set_dai_pll,
};

struct snd_soc_dai ak5702_dai = {
	.name = "AK5702",
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 1,
		    .channels_max = 4,
		    .rates = AK5702_RATES,
		    .formats = AK5702_FORMATS,
		    },
	.ops = &ak5702_ops,
};
EXPORT_SYMBOL_GPL(ak5702_dai);

static struct snd_soc_codec *ak5702_codec;

static int ak5702_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = ak5702_codec;
	int ret = 0;
	u8 reg = 0;

	socdev->card->codec = ak5702_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "ak5702: failed to create pcms\n");
		goto pcm_err;
	}

	/* power on device */
	reg = ak5702_read_reg_cache(codec, AK5702_PM1);
	reg |= AK5702_PM1_PMVCM;
	ak5702_write(codec, AK5702_PM1, reg);

	/* initialize ADC */
	reg = AK5702_SIG1_L_LIN1 | AK5702_SIG1_R_RIN2;
	ak5702_write(codec, AK5702_SIG1, reg);
	reg = AK5702_SIG2_L_LIN3 | AK5702_SIG2_R_RIN4;
	ak5702_write(codec, AK5702_SIG2, reg);

	reg = ak5702_read_reg_cache(codec, AK5702_PM1);
	reg = reg | AK5702_PM1_PMADAL | AK5702_PM1_PMADAR;
	ak5702_write(codec, AK5702_PM1, reg);
	reg = ak5702_read_reg_cache(codec, AK5702_PM2);
	reg = reg | AK5702_PM2_PMADBL | AK5702_PM2_PMADBR;
	ak5702_write(codec, AK5702_PM2, reg);

	/* initialize volume */
	ak5702_write(codec, AK5702_MICG1, AK5702_MICG1_INIT);
	ak5702_write(codec, AK5702_MICG2, AK5702_MICG2_INIT);
	ak5702_write(codec, AK5702_VOL1, AK5702_VOL1_IVOLAC);
	ak5702_write(codec, AK5702_VOL2, AK5702_VOL2_IVOLBC);
	ak5702_write(codec, AK5702_LVOL1, AK5702_LVOL1_INIT);
	ak5702_write(codec, AK5702_RVOL1, AK5702_RVOL1_INIT);
	ak5702_write(codec, AK5702_LVOL2, AK5702_LVOL2_INIT);
	ak5702_write(codec, AK5702_RVOL2, AK5702_RVOL2_INIT);

	snd_soc_add_controls(codec, ak5702_snd_controls,
			     ARRAY_SIZE(ak5702_snd_controls));
	ak5702_add_widgets(codec);

	return ret;

pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static int ak5702_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct ak5702_priv *ak5702;
	struct snd_soc_codec *codec;
	int ret;

	if (ak5702_codec) {
		dev_err(&client->dev,
			"Multiple AK5702 devices not supported\n");
		return -ENOMEM;
	}

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	ak5702 = kzalloc(sizeof(struct ak5702_priv), GFP_KERNEL);
	if (ak5702 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	snd_soc_codec_set_drvdata(codec, ak5702);
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	i2c_set_clientdata(client, codec);
	codec->control_data = client;

	codec->dev = &client->dev;
	codec->name = "AK5702";
	codec->owner = THIS_MODULE;
	codec->read = ak5702_read_reg_cache;
	codec->write = ak5702_write;
	codec->set_bias_level = ak5702_set_bias_level;
	codec->dai = &ak5702_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(ak5702_reg);
	codec->reg_cache = (void *)&ak5702_reg;
	if (codec->reg_cache == NULL)
		return -ENOMEM;

	ak5702_codec = codec;
	ak5702_dai.dev = &client->dev;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_dai(&ak5702_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAIs: %d\n", ret);
		return ret;
	}

	return ret;
}

static __devexit int ak5702_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	struct ak5702_priv *ak5702 = snd_soc_codec_get_drvdata(codec);

	snd_soc_unregister_dai(&ak5702_dai);
	snd_soc_unregister_codec(codec);
	kfree(codec);
	kfree(ak5702);
	ak5702_codec = NULL;
	return 0;
}

static const struct i2c_device_id ak5702_i2c_id[] = {
	{"ak5702-i2c", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ak5702_i2c_id);

static struct i2c_driver ak5702_i2c_driver = {
	.driver = {
		   .name = "ak5702-i2c",
		   .owner = THIS_MODULE,
		   },
	.probe = ak5702_i2c_probe,
	.remove = __devexit_p(ak5702_i2c_remove),
	.id_table = ak5702_i2c_id,
};
#endif

static int ak5702_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec->control_data)
		ak5702_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&ak5702_i2c_driver);
#endif
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_ak5702 = {
	.probe = ak5702_probe,
	.remove = ak5702_remove,
	.suspend = ak5702_suspend,
	.resume = ak5702_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak5702);

static int __init ak5702_modinit(void)
{
	return i2c_add_driver(&ak5702_i2c_driver);
}
module_init(ak5702_modinit);

static void __exit ak5702_exit(void)
{
	i2c_del_driver(&ak5702_i2c_driver);
}
module_exit(ak5702_exit);

MODULE_DESCRIPTION("Soc AK5702 driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
