/*
 * TAS5760 ASoC speaker amplifier driver
 *
 * Copyright (c) 2015 Imagination Technolgies Inc
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "tas5760.h"


struct tas5760_hw_params {
	bool pbtl_mode;
	bool pbtl_right_channel;
	bool volume_fading;
	unsigned int digital_clip;
	unsigned int digital_boost;
	unsigned int analog_gain;
	unsigned int pwm_rate;
};

struct tas5760_private {
	struct regmap			*regmap;
	unsigned int			format;
	struct tas5760_hw_params hw_params;
	struct gpio_desc		*spk_sd_gpio;
};

static bool tas5760_accessible_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x07:
	case 0x09 ... 0x0F:
		return false;
	default:
		return true;
	}
}

static bool tas5760_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS5760_DEVICE_ID:
	case TAS5760_ERR_STATUS:
		return true;
	}

	return false;
}
static bool tas5760_writeable_reg(struct device *dev, unsigned int reg)
{
	return tas5760_accessible_reg(dev, reg) &&
			!tas5760_volatile_reg(dev, reg);
}


static int tas5760_set_sysclk(struct snd_soc_dai *dai,
			      int clk_id, unsigned int freq, int dir)
{
	struct device *dev = dai->codec->dev;

	dev_dbg(dev, "clkid:%d, freq:%d, dir:%d", clk_id, freq, dir);
	return 0;
}

static int tas5760_set_dai_fmt(struct snd_soc_dai *dai, unsigned int format)
{
	struct tas5760_private *priv = snd_soc_codec_get_drvdata(dai->codec);
	struct device *dev = dai->codec->dev;

	dev_dbg(dev, "format:%d\n", format);
	priv->format = format;

	return 0;
}

static int tas5760_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	return 0;
}

static int tas5760_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct tas5760_private *priv = snd_soc_codec_get_drvdata(dai->codec);

	return regmap_update_bits(priv->regmap,
				  TAS5760_VOL_CTRL,
				  TAS5760_MUTE_MASK,
				  mute ? TAS5760_MUTE_MASK : 0);
}

static int tas5760_init(struct device *dev, struct tas5760_private *priv)
{

	struct device_node *np = dev->of_node;
	struct device_node *hw_config = of_get_child_by_name(np, "hw-params");
	int ret, val;

	if (!hw_config)
		dev_warn(dev, "No hardware config for init: %ld\n",
			 PTR_ERR(hw_config));
	else {

		/* Mute the Device */
		ret = regmap_update_bits(priv->regmap,
				  TAS5760_VOL_CTRL,
				  TAS5760_MUTE_MASK, TAS5760_MUTE_MASK);

		/* BTL/PBTL mode configuration */
		priv->hw_params.pbtl_mode = of_property_read_bool(hw_config,
						"pbtl-mode");
		dev_dbg(dev, "pbtl mode:%d\n", priv->hw_params.pbtl_mode);

		/* Channel Selection for PBTL mode */
		priv->hw_params.pbtl_right_channel = of_property_read_bool(
					hw_config, "pbtl-right-channel");
		dev_dbg(dev, "pbtl right channel:%d\n",
					priv->hw_params.pbtl_right_channel);

		ret = regmap_update_bits(priv->regmap, TAS5760_ANALOG_CTRL,
					TAS5760_PBTL_MASK,
					TAS5760_PBTL_EN_VAL(priv->hw_params.pbtl_mode) |
					TAS5760_PBTL_CH_VAL(priv->hw_params.pbtl_right_channel));
		if (ret < 0)
			return ret;

		/* Volume Fading */
		priv->hw_params.volume_fading = of_property_read_bool(hw_config,
						"volume-fading");
		dev_dbg(dev, "volume fading:%d\n", priv->hw_params.volume_fading);
		ret = regmap_update_bits(priv->regmap, TAS5760_VOL_CTRL,
					TAS5760_VOL_FADE_MASK,
					TAS5760_VOL_FADE_VAL(priv->hw_params.volume_fading));
		if (ret < 0)
			return ret;

		/* Digital Clip level */
		if (of_property_read_u32(hw_config, "digital-clip",
					&priv->hw_params.digital_clip) == 0) {
			if (priv->hw_params.digital_clip > TAS5760_DIGITAL_CLIP_MAX_VAL) {
				dev_err(dev, "Invalid value for digital clip\n");
				priv->hw_params.digital_clip = 0;
				return -EINVAL;
			}
			dev_dbg(dev, "digital clip:%d\n", priv->hw_params.digital_clip);
			ret = regmap_update_bits(priv->regmap, TAS5760_DIGITAL_CLIP_CTRL1,
					TAS5760_DIGITAL_CLIP_MASK,
					TAS5760_DIGITAL_CLIP1_VAL(priv->hw_params.digital_clip));
			if (ret < 0)
				return ret;

			ret = regmap_write(priv->regmap, TAS5760_DIGITAL_CLIP_CTRL2,
					TAS5760_DIGITAL_CLIP2_VAL(priv->hw_params.digital_clip));
			if (ret < 0)
				return ret;

			ret = regmap_update_bits(priv->regmap, TAS5760_POWER_CTRL,
					TAS5760_DIGITAL_CLIP_MASK,
					TAS5760_DIGITAL_CLIP3_VAL(priv->hw_params.digital_clip));
			if (ret < 0)
				return ret;
		}
		/* Digital Boost */
		if (of_property_read_u32(hw_config, "digital-boost",
					&priv->hw_params.digital_boost) == 0) {
			dev_dbg(dev, "digital boost:%d\n",
					priv->hw_params.digital_boost);
			switch (priv->hw_params.digital_boost) {
			/* Values in dB */
			case 0:
				val = 0x00;
				break;
			case 6:
				val = 0x01;
				break;
			case 12:
				val = 0x02;
				break;
			case 18:
				val = 0x03;
				break;
			default:
				dev_err(dev, "Invalid value for digital-boost\n");
				priv->hw_params.digital_boost = 0;
				return -EINVAL;
			}
			ret = regmap_update_bits(priv->regmap, TAS5760_DIGITAL_CTRL,
					TAS5760_DIGITAL_BOOST_MASK,
					TAS5760_DIGITAL_BOOST_VAL(val));
			if (ret < 0)
				return ret;
		}
		/* Analog gain */
		if (of_property_read_u32(hw_config, "analog-gain",
					&priv->hw_params.analog_gain) == 0) {
			dev_dbg(dev, "analog gain:%d\n",
					priv->hw_params.analog_gain);
			switch (priv->hw_params.analog_gain) {
			/* Values in 0.1 dBV*/
			case 192:
				val = 0x0;
				break;
			case 226:
				val = 0x01;
				break;
			case 250:
				val = 0x02;
				break;
			default:
				dev_err(dev, "Invalid value for analog gain\n");
				priv->hw_params.analog_gain = 0;
				return -EINVAL;
			}
			ret = regmap_update_bits(priv->regmap, TAS5760_ANALOG_CTRL,
					TAS5760_ANALOG_GAIN_MASK,
					TAS5760_ANALOG_GAIN_VAL(val));
			if (ret < 0)
				return ret;
		}
		/* PWM rate */
		if (of_property_read_u32(hw_config, "pwm-rate",
					&priv->hw_params.pwm_rate) == 0) {
			if (priv->hw_params.pwm_rate > TAS5760_PWM_RATE_MAX) {
				priv->hw_params.pwm_rate = 0;
				return -EINVAL;
			}
			dev_dbg(dev, "pwm rate:%02x\n", priv->hw_params.pwm_rate);
			ret = regmap_update_bits(priv->regmap, TAS5760_ANALOG_CTRL,
					TAS5760_PWM_RATE_MASK,
					TAS5760_PWM_RATE_VAL(priv->hw_params.pwm_rate));
			if (ret < 0)
				return ret;
		}
		/* Unmute the device */
		ret = regmap_update_bits(priv->regmap,
				  TAS5760_VOL_CTRL,
				  TAS5760_MUTE_MASK, 0);
		if (ret < 0)
			return ret;
	}

	if (!IS_ERR(priv->spk_sd_gpio))
		/* Enable the speaker amplifier */
		gpiod_set_value(priv->spk_sd_gpio, 1);
	else {
		ret = regmap_update_bits(priv->regmap, TAS5760_POWER_CTRL,
							TAS5760_SPK_SD_MASK,
							TAS5760_SPK_SD_MASK);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct i2c_device_id tas5760_i2c_id[] = {
	{ "tas5760", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas5760_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id tas5760_dt_ids[] = {
	{ .compatible = "ti,tas5760", },
	{ }
};
MODULE_DEVICE_TABLE(of, tas5760_dt_ids);
#endif

static int tas5760_probe(struct snd_soc_codec *codec)
{
	struct tas5760_private *priv = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	ret = tas5760_init(codec->dev, priv);
	if (ret < 0)
		return ret;

	return 0;
}

static int tas5760_remove(struct snd_soc_codec *codec)
{
	struct tas5760_private *priv = snd_soc_codec_get_drvdata(codec);
	int ret;
	/* Mute the Device */
	ret = regmap_update_bits(priv->regmap,
				  TAS5760_VOL_CTRL,
				  TAS5760_MUTE_MASK,
				  TAS5760_MUTE_MASK);
	if (ret < 0)
		return ret;

	/* Put the device in shutdown */
	if (!IS_ERR(priv->spk_sd_gpio))
		gpiod_set_value(priv->spk_sd_gpio, 0);
	else {
		ret = regmap_update_bits(priv->regmap,
					TAS5760_POWER_CTRL,
					TAS5760_SPK_SD_MASK,
					0);
		if (ret < 0)
			return ret;
	}

	return 0;
};

/* TAS5760 controls */
static const DECLARE_TLV_DB_SCALE(tas5760_dac_tlv, -10350, 50, 1);

static const struct snd_kcontrol_new tas5760_controls[] = {
	SOC_DOUBLE_R_TLV("Channel 1/2 Playback Volume",
				TAS5760_VOL_CTRL_L, TAS5760_VOL_CTRL_R,
				0, 0xff, 0, tas5760_dac_tlv),
};

static struct snd_soc_codec_driver soc_codec_dev_tas5760 = {
	.probe			= tas5760_probe,
	.remove			= tas5760_remove,
	.controls		= tas5760_controls,
	.num_controls	= ARRAY_SIZE(tas5760_controls),
};


static const struct snd_soc_dai_ops tas5760_dai_ops = {
	.set_sysclk	= tas5760_set_sysclk,
	.set_fmt	= tas5760_set_dai_fmt,
	.hw_params	= tas5760_hw_params,
	.digital_mute	= tas5760_digital_mute,
};


static const struct regmap_config tas5760_regmap = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= TAS5760_MAX_REG,
	.cache_type			= REGCACHE_RBTREE,
	.volatile_reg		= tas5760_volatile_reg,
	.writeable_reg		= tas5760_writeable_reg,
	.readable_reg		= tas5760_accessible_reg,
};

static struct snd_soc_dai_driver tas5760_dai = {
	.name = "tas5760-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats =	SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S18_3LE |
					SNDRV_PCM_FMTBIT_U20_3LE |
					SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &tas5760_dai_ops,
};


static int tas5760_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct tas5760_private *priv;
	struct device *dev = &client->dev;
	int i = -1, ret;


	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);

	/* Initalise the register maps for tas5760 */
	priv->regmap = devm_regmap_init_i2c(client, &tas5760_regmap);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&client->dev, "Failed to create regmap: %d\n", ret);
		return ret;
	}

	/* Identify the device */
	ret = regmap_read(priv->regmap, TAS5760_DEVICE_ID, &i);
	if (ret < 0)
		return ret;

	if (i == 0x00)
		dev_dbg(dev, "Identified TAS5760 codec chip\n");
	else {
		dev_err(dev, "TAS5760 codec could not be identified");
		return -ENODEV;
	}

	/* Get the spk_sd gpio */
	priv->spk_sd_gpio = devm_gpiod_get(dev, "spk-sd");
	if (!IS_ERR(priv->spk_sd_gpio)) {
		/* Set the direction as output */
		gpiod_direction_output(priv->spk_sd_gpio, 1);
		/* Keep the speaker amplifier in shutdown */
		gpiod_set_value(priv->spk_sd_gpio, 0);
	} else {

		dev_warn(dev, "error requesting spk_sd_gpio: %ld\n",
			 PTR_ERR(priv->spk_sd_gpio));
		/* Use the SPK_SD bit to shutdown */
		ret = regmap_update_bits(priv->regmap, TAS5760_POWER_CTRL,
							TAS5760_SPK_SD_MASK,
							0);
		if (ret < 0)
			return ret;
	}
	return snd_soc_register_codec(&client->dev, &soc_codec_dev_tas5760,
				      &tas5760_dai, 1);
}

static int tas5760_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}


static struct i2c_driver tas5760_i2c_driver = {
	.driver = {
		.name	= "tas5760",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tas5760_dt_ids),
	},
	.id_table	= tas5760_i2c_id,
	.probe		= tas5760_i2c_probe,
	.remove		= tas5760_i2c_remove,
};


module_i2c_driver(tas5760_i2c_driver);

MODULE_AUTHOR("Imagination Technologies Pvt Ltd.");
MODULE_DESCRIPTION("Texas Instruments TAS5760 ALSA SoC Codec Driver");
MODULE_LICENSE("GPL");
