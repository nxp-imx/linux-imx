/*
 * sgtl5000.c  --  SGTL5000 ALSA SoC Audio driver
 *
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <mach/hardware.h>

#include "sgtl5000.h"

struct sgtl5000_priv {
	int sysclk;
	int master;
	int fmt;
	int rev;
	int lrclk;
	int capture_channels;
	int playback_active;
	int capture_active;
	int clock_on;		/* clock enable status */
	int need_clk_for_access; /* need clock on because doing access */
	int need_clk_for_bias;   /* need clock on due to bias level */
	int (*clock_enable) (int enable);
	struct regulator *reg_vddio;
	struct regulator *reg_vdda;
	struct regulator *reg_vddd;
	int vddio;		/* voltage of VDDIO (mv) */
	int vdda;		/* voltage of vdda (mv) */
	int vddd;		/* voltage of vddd (mv), 0 if not connected */
	struct snd_pcm_substream *master_substream;
	struct snd_pcm_substream *slave_substream;
};

static int sgtl5000_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level);

#define SGTL5000_MAX_CACHED_REG SGTL5000_CHIP_SHORT_CTRL
static u16 sgtl5000_regs[(SGTL5000_MAX_CACHED_REG >> 1) + 1];

/*
 * Schedule clock to be turned off or turn clock on.
 */
static void sgtl5000_clock_gating(struct snd_soc_codec *codec, int enable)
{
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);

	if (sgtl5000->clock_enable == NULL)
		return;

	if (enable == 0) {
		if (!sgtl5000->need_clk_for_access &&
		    !sgtl5000->need_clk_for_bias)
			schedule_delayed_work(&codec->delayed_work,
					      msecs_to_jiffies(300));
	} else {
		if (!sgtl5000->clock_on) {
			sgtl5000->clock_enable(1);
			sgtl5000->clock_on = 1;
		}
	}
}

static unsigned int sgtl5000_read_reg_cache(struct snd_soc_codec *codec,
					    unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	unsigned int offset = reg >> 1;
	if (offset >= ARRAY_SIZE(sgtl5000_regs))
		return -EINVAL;
	pr_debug("r r:%02x,v:%04x\n", reg, cache[offset]);
	return cache[offset];
}

static unsigned int sgtl5000_hw_read(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);
	struct i2c_client *client = codec->control_data;
	int i2c_ret;
	u16 value;
	u8 buf0[2], buf1[2];
	u16 addr = client->addr;
	u16 flags = client->flags;
	struct i2c_msg msg[2] = {
		{addr, flags, 2, buf0},
		{addr, flags | I2C_M_RD, 2, buf1},
	};

	sgtl5000->need_clk_for_access = 1;
	sgtl5000_clock_gating(codec, 1);
	buf0[0] = (reg & 0xff00) >> 8;
	buf0[1] = reg & 0xff;
	i2c_ret = i2c_transfer(client->adapter, msg, 2);
	sgtl5000->need_clk_for_access = 0;
	sgtl5000_clock_gating(codec, 0);
	if (i2c_ret < 0) {
		pr_err("%s: read reg error : Reg 0x%02x\n", __func__, reg);
		return 0;
	}

	value = buf1[0] << 8 | buf1[1];

	pr_debug("r r:%02x,v:%04x\n", reg, value);
	return value;
}

static unsigned int sgtl5000_read(struct snd_soc_codec *codec, unsigned int reg)
{
	if ((reg == SGTL5000_CHIP_ID) ||
	    (reg == SGTL5000_CHIP_ADCDAC_CTRL) ||
	    (reg == SGTL5000_CHIP_ANA_STATUS) ||
	    (reg > SGTL5000_MAX_CACHED_REG))
		return sgtl5000_hw_read(codec, reg);
	else
		return sgtl5000_read_reg_cache(codec, reg);
}

static inline void sgtl5000_write_reg_cache(struct snd_soc_codec *codec,
					    u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	unsigned int offset = reg >> 1;
	if (offset < ARRAY_SIZE(sgtl5000_regs))
		cache[offset] = value;
}

static int sgtl5000_write(struct snd_soc_codec *codec, unsigned int reg,
			  unsigned int value)
{
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);
	struct i2c_client *client = codec->control_data;
	u16 addr = client->addr;
	u16 flags = client->flags;
	u8 buf[4];
	int i2c_ret;
	struct i2c_msg msg = { addr, flags, 4, buf };

	sgtl5000->need_clk_for_access = 1;
	sgtl5000_clock_gating(codec, 1);
	sgtl5000_write_reg_cache(codec, reg, value);
	pr_debug("w r:%02x,v:%04x\n", reg, value);
	buf[0] = (reg & 0xff00) >> 8;
	buf[1] = reg & 0xff;
	buf[2] = (value & 0xff00) >> 8;
	buf[3] = value & 0xff;

	i2c_ret = i2c_transfer(client->adapter, &msg, 1);
	sgtl5000->need_clk_for_access = 0;
	sgtl5000_clock_gating(codec, 0);
	if (i2c_ret < 0) {
		pr_err("%s: write reg error : Reg 0x%02x = 0x%04x\n",
		       __func__, reg, value);
		return -EIO;
	}

	return i2c_ret;
}

static void sgtl5000_sync_reg_cache(struct snd_soc_codec *codec)
{
	int reg;
	for (reg = 0; reg <= SGTL5000_MAX_CACHED_REG; reg += 2)
		sgtl5000_write_reg_cache(codec, reg,
					 sgtl5000_hw_read(codec, reg));
}

static int sgtl5000_restore_reg(struct snd_soc_codec *codec, unsigned int reg)
{
	unsigned int cached_val, hw_val;

	cached_val = sgtl5000_read_reg_cache(codec, reg);
	hw_val = sgtl5000_hw_read(codec, reg);

	if (hw_val != cached_val)
		return sgtl5000_write(codec, reg, cached_val);

	return 0;
}

static int all_reg[] = {
	SGTL5000_CHIP_ID,
	SGTL5000_CHIP_DIG_POWER,
	SGTL5000_CHIP_CLK_CTRL,
	SGTL5000_CHIP_I2S_CTRL,
	SGTL5000_CHIP_SSS_CTRL,
	SGTL5000_CHIP_ADCDAC_CTRL,
	SGTL5000_CHIP_DAC_VOL,
	SGTL5000_CHIP_PAD_STRENGTH,
	SGTL5000_CHIP_ANA_ADC_CTRL,
	SGTL5000_CHIP_ANA_HP_CTRL,
	SGTL5000_CHIP_ANA_CTRL,
	SGTL5000_CHIP_LINREG_CTRL,
	SGTL5000_CHIP_REF_CTRL,
	SGTL5000_CHIP_MIC_CTRL,
	SGTL5000_CHIP_LINE_OUT_CTRL,
	SGTL5000_CHIP_LINE_OUT_VOL,
	SGTL5000_CHIP_ANA_POWER,
	SGTL5000_CHIP_PLL_CTRL,
	SGTL5000_CHIP_CLK_TOP_CTRL,
	SGTL5000_CHIP_ANA_STATUS,
	SGTL5000_CHIP_SHORT_CTRL,
};

#ifdef DEBUG
static void dump_reg(struct snd_soc_codec *codec)
{
	int i, reg;
	printk(KERN_DEBUG "dump begin\n");
	for (i = 0; i < 21; i++) {
		reg = sgtl5000_read(codec, all_reg[i]);
		printk(KERN_DEBUG "d r %04x, v %04x\n", all_reg[i], reg);
	}
	printk(KERN_DEBUG "dump end\n");
}
#else
static void dump_reg(struct snd_soc_codec *codec)
{
}
#endif

static const char *adc_mux_text[] = {
	"MIC_IN", "LINE_IN"
};

static const char *dac_mux_text[] = {
	"DAC", "LINE_IN"
};

static const struct soc_enum adc_enum =
SOC_ENUM_SINGLE(SGTL5000_CHIP_ANA_CTRL, 2, 2, adc_mux_text);

static const struct soc_enum dac_enum =
SOC_ENUM_SINGLE(SGTL5000_CHIP_ANA_CTRL, 6, 2, dac_mux_text);

static const struct snd_kcontrol_new adc_mux =
SOC_DAPM_ENUM("ADC Mux", adc_enum);

static const struct snd_kcontrol_new dac_mux =
SOC_DAPM_ENUM("DAC Mux", dac_enum);

static const struct snd_soc_dapm_widget sgtl5000_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("LINE_IN"),
	SND_SOC_DAPM_INPUT("MIC_IN"),

	SND_SOC_DAPM_OUTPUT("HP_OUT"),
	SND_SOC_DAPM_OUTPUT("LINE_OUT"),

	SND_SOC_DAPM_PGA("HP", SGTL5000_CHIP_ANA_CTRL, 4, 1, NULL, 0),
	SND_SOC_DAPM_PGA("LO", SGTL5000_CHIP_ANA_CTRL, 8, 1, NULL, 0),

	SND_SOC_DAPM_MUX("ADC Mux", SND_SOC_NOPM, 0, 0, &adc_mux),
	SND_SOC_DAPM_MUX("DAC Mux", SND_SOC_NOPM, 0, 0, &dac_mux),

	SND_SOC_DAPM_ADC("ADC", "Capture", SGTL5000_CHIP_DIG_POWER, 6, 0),
	SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"ADC Mux", "LINE_IN", "LINE_IN"},
	{"ADC Mux", "MIC_IN", "MIC_IN"},
	{"ADC", NULL, "ADC Mux"},
	{"DAC Mux", "DAC", "DAC"},
	{"DAC Mux", "LINE_IN", "LINE_IN"},
	{"LO", NULL, "DAC"},
	{"HP", NULL, "DAC Mux"},
	{"LINE_OUT", NULL, "LO"},
	{"HP_OUT", NULL, "HP"},
};

static int sgtl5000_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, sgtl5000_dapm_widgets,
				  ARRAY_SIZE(sgtl5000_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int dac_info_volsw(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xfc - 0x3c;
	return 0;
}

static int dac_get_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg, l, r;

	reg = sgtl5000_read(codec, SGTL5000_CHIP_DAC_VOL);
	l = (reg & SGTL5000_DAC_VOL_LEFT_MASK) >> SGTL5000_DAC_VOL_LEFT_SHIFT;
	r = (reg & SGTL5000_DAC_VOL_RIGHT_MASK) >> SGTL5000_DAC_VOL_RIGHT_SHIFT;
	l = l < 0x3c ? 0x3c : l;
	l = l > 0xfc ? 0xfc : l;
	r = r < 0x3c ? 0x3c : r;
	r = r > 0xfc ? 0xfc : r;
	l = 0xfc - l;
	r = 0xfc - r;

	ucontrol->value.integer.value[0] = l;
	ucontrol->value.integer.value[1] = r;

	return 0;
}

static int dac_put_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg, l, r;

	l = ucontrol->value.integer.value[0];
	r = ucontrol->value.integer.value[1];

	l = l < 0 ? 0 : l;
	l = l > 0xfc - 0x3c ? 0xfc - 0x3c : l;
	r = r < 0 ? 0 : r;
	r = r > 0xfc - 0x3c ? 0xfc - 0x3c : r;
	l = 0xfc - l;
	r = 0xfc - r;

	reg = l << SGTL5000_DAC_VOL_LEFT_SHIFT |
	    r << SGTL5000_DAC_VOL_RIGHT_SHIFT;

	sgtl5000_write(codec, SGTL5000_CHIP_DAC_VOL, reg);

	return 0;
}

static const char *mic_gain_text[] = {
	"0dB", "20dB", "30dB", "40dB"
};

static const char *adc_m6db_text[] = {
	"No Change", "Reduced by 6dB"
};

static const struct soc_enum mic_gain =
SOC_ENUM_SINGLE(SGTL5000_CHIP_MIC_CTRL, 0, 4, mic_gain_text);

static const struct soc_enum adc_m6db =
SOC_ENUM_SINGLE(SGTL5000_CHIP_ANA_ADC_CTRL, 8, 2, adc_m6db_text);

static const struct snd_kcontrol_new sgtl5000_snd_controls[] = {
	SOC_ENUM("MIC GAIN", mic_gain),
	SOC_DOUBLE("Capture Volume", SGTL5000_CHIP_ANA_ADC_CTRL, 0, 4, 0xf, 0),
	SOC_ENUM("Capture Vol Reduction", adc_m6db),
	{.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = "Playback Volume",
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
	 SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = dac_info_volsw,
	 .get = dac_get_volsw,
	 .put = dac_put_volsw,
	 },
	SOC_DOUBLE("Headphone Volume", SGTL5000_CHIP_ANA_HP_CTRL, 0, 8, 0x7f,
		   1),
};

static int sgtl5000_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 adcdac_ctrl;

	adcdac_ctrl = sgtl5000_read(codec, SGTL5000_CHIP_ADCDAC_CTRL);

	if (mute) {
		adcdac_ctrl |= SGTL5000_DAC_MUTE_LEFT;
		adcdac_ctrl |= SGTL5000_DAC_MUTE_RIGHT;
	} else {
		adcdac_ctrl &= ~SGTL5000_DAC_MUTE_LEFT;
		adcdac_ctrl &= ~SGTL5000_DAC_MUTE_RIGHT;
	}

	sgtl5000_write(codec, SGTL5000_CHIP_ADCDAC_CTRL, adcdac_ctrl);
	if (!mute)
		dump_reg(codec);
	return 0;
}

static int sgtl5000_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);
	u16 i2sctl = 0;
	pr_debug("%s:fmt=%08x\n", __func__, fmt);
	sgtl5000->master = 0;
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		i2sctl |= SGTL5000_I2S_MASTER;
		sgtl5000->master = 1;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
		return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		i2sctl |= SGTL5000_I2S_MODE_PCM;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		i2sctl |= SGTL5000_I2S_MODE_PCM;
		i2sctl |= SGTL5000_I2S_LRALIGN;
		break;
	case SND_SOC_DAIFMT_I2S:
		i2sctl |= SGTL5000_I2S_MODE_I2S_LJ;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		i2sctl |= SGTL5000_I2S_MODE_RJ;
		i2sctl |= SGTL5000_I2S_LRPOL;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		i2sctl |= SGTL5000_I2S_MODE_I2S_LJ;
		i2sctl |= SGTL5000_I2S_LRALIGN;
		break;
	default:
		return -EINVAL;
	}
	sgtl5000->fmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	/* Clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
	case SND_SOC_DAIFMT_NB_IF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
	case SND_SOC_DAIFMT_IB_NF:
		i2sctl |= SGTL5000_I2S_SCLK_INV;
		break;
	default:
		return -EINVAL;
	}
	sgtl5000_write(codec, SGTL5000_CHIP_I2S_CTRL, i2sctl);

	return 0;
}

static int sgtl5000_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				   int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);

	switch (clk_id) {
	case SGTL5000_SYSCLK:
		sgtl5000->sysclk = freq;
		break;
	case SGTL5000_LRCLK:
		sgtl5000->lrclk = freq;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sgtl5000_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);
	int reg;

	reg = sgtl5000_read(codec, SGTL5000_CHIP_DIG_POWER);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg |= SGTL5000_I2S_IN_POWERUP;
	else
		reg |= SGTL5000_I2S_OUT_POWERUP;
	sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, reg);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		reg |= SGTL5000_ADC_POWERUP;
		if (sgtl5000->capture_channels == 1)
			reg &= ~SGTL5000_ADC_STEREO;
		else
			reg |= SGTL5000_ADC_STEREO;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
	}

	return 0;
}

static int sgtl5000_pcm_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);
	struct snd_pcm_runtime *master_runtime;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sgtl5000->playback_active++;
	else
		sgtl5000->capture_active++;

	/* The DAI has shared clocks so if we already have a playback or
	 * capture going then constrain this substream to match it.
	 */
	if (sgtl5000->master_substream) {
		master_runtime = sgtl5000->master_substream->runtime;

		if (master_runtime->rate != 0) {
			pr_debug("Constraining to %dHz\n",
				 master_runtime->rate);
			snd_pcm_hw_constraint_minmax(substream->runtime,
						     SNDRV_PCM_HW_PARAM_RATE,
						     master_runtime->rate,
						     master_runtime->rate);
		}

		if (master_runtime->sample_bits != 0) {
			pr_debug("Constraining to %d bits\n",
				 master_runtime->sample_bits);
			snd_pcm_hw_constraint_minmax(substream->runtime,
						     SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
						     master_runtime->sample_bits,
						     master_runtime->sample_bits);
		}

		sgtl5000->slave_substream = substream;
	} else
		sgtl5000->master_substream = substream;

	return 0;
}

static void sgtl5000_pcm_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);
	int reg, dig_pwr, ana_pwr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sgtl5000->playback_active--;
	else
		sgtl5000->capture_active--;

	if (sgtl5000->master_substream == substream)
		sgtl5000->master_substream = sgtl5000->slave_substream;

	sgtl5000->slave_substream = NULL;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ana_pwr = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		ana_pwr &= ~(SGTL5000_ADC_POWERUP | SGTL5000_ADC_STEREO);
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, ana_pwr);
	}

	dig_pwr = sgtl5000_read(codec, SGTL5000_CHIP_DIG_POWER);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dig_pwr &= ~SGTL5000_I2S_IN_POWERUP;
	else
		dig_pwr &= ~SGTL5000_I2S_OUT_POWERUP;
	sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, dig_pwr);

	if (!sgtl5000->playback_active && !sgtl5000->capture_active) {
		reg = sgtl5000_read(codec, SGTL5000_CHIP_I2S_CTRL);
		reg &= ~SGTL5000_I2S_MASTER;
		sgtl5000_write(codec, SGTL5000_CHIP_I2S_CTRL, reg);
	}
}

/*
 * Set PCM DAI bit size and sample rate.
 * input: params_rate, params_fmt
 */
static int sgtl5000_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);
	int channels = params_channels(params);
	int clk_ctl = 0;
	int pll_ctl = 0;
	int i2s_ctl;
	int div2 = 0;
	int reg;
	int sys_fs;

	pr_debug("%s channels=%d\n", __func__, channels);

	if (!sgtl5000->sysclk) {
		pr_err("%s: set sysclk first!\n", __func__);
		return -EFAULT;
	}

	if (substream == sgtl5000->slave_substream) {
		pr_debug("Ignoring hw_params for slave substream\n");
		return 0;
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		sgtl5000->capture_channels = channels;

	switch (sgtl5000->lrclk) {
	case 8000:
	case 16000:
		sys_fs = 32000;
		break;
	case 11025:
	case 22050:
		sys_fs = 44100;
		break;
	default:
		sys_fs = sgtl5000->lrclk;
		break;
	}

	switch (sys_fs / sgtl5000->lrclk) {
	case 4:
		clk_ctl |= SGTL5000_RATE_MODE_DIV_4 << SGTL5000_RATE_MODE_SHIFT;
		break;
	case 2:
		clk_ctl |= SGTL5000_RATE_MODE_DIV_2 << SGTL5000_RATE_MODE_SHIFT;
		break;
	default:
		break;
	}

	switch (sys_fs) {
	case 32000:
		clk_ctl |= SGTL5000_SYS_FS_32k << SGTL5000_SYS_FS_SHIFT;
		break;
	case 44100:
		clk_ctl |= SGTL5000_SYS_FS_44_1k << SGTL5000_SYS_FS_SHIFT;
		break;
	case 48000:
		clk_ctl |= SGTL5000_SYS_FS_48k << SGTL5000_SYS_FS_SHIFT;
		break;
	case 96000:
		clk_ctl |= SGTL5000_SYS_FS_96k << SGTL5000_SYS_FS_SHIFT;
		break;
	default:
		pr_err("%s: sample rate %d not supported\n", __func__,
		       sgtl5000->lrclk);
		return -EFAULT;
	}
	/* SGTL5000 rev1 has a IC bug to prevent switching to MCLK from PLL. */
	if (!sgtl5000->master) {
		sys_fs = sgtl5000->lrclk;
		clk_ctl = SGTL5000_RATE_MODE_DIV_1 << SGTL5000_RATE_MODE_SHIFT;
		if (sys_fs * 256 == sgtl5000->sysclk)
			clk_ctl |= SGTL5000_MCLK_FREQ_256FS << \
				SGTL5000_MCLK_FREQ_SHIFT;
		else if (sys_fs * 384 == sgtl5000->sysclk && sys_fs != 96000)
			clk_ctl |= SGTL5000_MCLK_FREQ_384FS << \
				SGTL5000_MCLK_FREQ_SHIFT;
		else if (sys_fs * 512 == sgtl5000->sysclk && sys_fs != 96000)
			clk_ctl |= SGTL5000_MCLK_FREQ_512FS << \
				SGTL5000_MCLK_FREQ_SHIFT;
		else {
			pr_err("%s: PLL not supported in slave mode\n",
			       __func__);
			return -EINVAL;
		}
	} else
		clk_ctl |= SGTL5000_MCLK_FREQ_PLL << SGTL5000_MCLK_FREQ_SHIFT;

	if ((clk_ctl & SGTL5000_MCLK_FREQ_MASK) == SGTL5000_MCLK_FREQ_PLL) {
		u64 out, t;
		unsigned int in, int_div, frac_div;
		if (sgtl5000->sysclk > 17000000) {
			div2 = 1;
			in = sgtl5000->sysclk / 2;
		} else {
			div2 = 0;
			in = sgtl5000->sysclk;
		}
		if (sys_fs == 44100)
			out = 180633600;
		else
			out = 196608000;
		t = do_div(out, in);
		int_div = out;
		t *= 2048;
		do_div(t, in);
		frac_div = t;
		pll_ctl = int_div << SGTL5000_PLL_INT_DIV_SHIFT |
		    frac_div << SGTL5000_PLL_FRAC_DIV_SHIFT;
	}

	i2s_ctl = sgtl5000_read(codec, SGTL5000_CHIP_I2S_CTRL);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (sgtl5000->fmt == SND_SOC_DAIFMT_RIGHT_J)
			return -EINVAL;
		i2s_ctl |= SGTL5000_I2S_DLEN_16 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_32FS <<
		    SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		i2s_ctl |= SGTL5000_I2S_DLEN_20 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_64FS <<
		    SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		i2s_ctl |= SGTL5000_I2S_DLEN_24 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_64FS <<
		    SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		if (sgtl5000->fmt == SND_SOC_DAIFMT_RIGHT_J)
			return -EINVAL;
		i2s_ctl |= SGTL5000_I2S_DLEN_32 << SGTL5000_I2S_DLEN_SHIFT;
		i2s_ctl |= SGTL5000_I2S_SCLKFREQ_64FS <<
		    SGTL5000_I2S_SCLKFREQ_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	pr_debug("\nfs=%d,clk_ctl=%04x,pll_ctl=%04x,i2s_ctl=%04x,div2=%d\n",
		 sgtl5000->lrclk, clk_ctl, pll_ctl, i2s_ctl, div2);

	if ((clk_ctl & SGTL5000_MCLK_FREQ_MASK) == SGTL5000_MCLK_FREQ_PLL) {
		sgtl5000_write(codec, SGTL5000_CHIP_PLL_CTRL, pll_ctl);
		reg = sgtl5000_read(codec, SGTL5000_CHIP_CLK_TOP_CTRL);
		if (div2)
			reg |= SGTL5000_INPUT_FREQ_DIV2;
		else
			reg &= ~SGTL5000_INPUT_FREQ_DIV2;
		sgtl5000_write(codec, SGTL5000_CHIP_CLK_TOP_CTRL, reg);
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		reg |= SGTL5000_PLL_POWERUP | SGTL5000_VCOAMP_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
	}
	sgtl5000_write(codec, SGTL5000_CHIP_CLK_CTRL, clk_ctl);
	sgtl5000_write(codec, SGTL5000_CHIP_I2S_CTRL, i2s_ctl);

	return 0;
}

static void sgtl5000_mic_bias(struct snd_soc_codec *codec, int enable)
{
	int reg, bias_r = 0;
	if (enable)
		bias_r = SGTL5000_BIAS_R_4k << SGTL5000_BIAS_R_SHIFT;
	reg = sgtl5000_read(codec, SGTL5000_CHIP_MIC_CTRL);
	if ((reg & SGTL5000_BIAS_R_MASK) != bias_r) {
		reg &= ~SGTL5000_BIAS_R_MASK;
		reg |= bias_r;
		sgtl5000_write(codec, SGTL5000_CHIP_MIC_CTRL, reg);
	}
}

static int sgtl5000_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level)
{
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);
	u16 reg, ana_pwr;
	int delay = 0;
	pr_debug("dapm level %d\n", level);
	switch (level) {
	case SND_SOC_BIAS_ON:		/* full On */
		if (codec->bias_level == SND_SOC_BIAS_ON)
			break;

		sgtl5000_mic_bias(codec, 1);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		reg |= SGTL5000_VAG_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
		msleep(400);

		break;

	case SND_SOC_BIAS_PREPARE:	/* partial On */
		/* Keep clock on while in PREPARE or BIAS_ON state */
		sgtl5000->need_clk_for_bias = 1;
		sgtl5000_clock_gating(codec, 1);

		if (codec->bias_level == SND_SOC_BIAS_PREPARE)
			break;

		sgtl5000_mic_bias(codec, 0);

		/* must power up hp/line out before vag & dac to
		   avoid pops. */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		if (reg & SGTL5000_VAG_POWERUP)
			delay = 600;
		reg &= ~SGTL5000_VAG_POWERUP;
		reg |= SGTL5000_DAC_POWERUP;
		reg |= SGTL5000_HP_POWERUP;
		reg |= SGTL5000_LINE_OUT_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
		if (delay)
			msleep(delay);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_DIG_POWER);
		reg |= SGTL5000_DAC_EN;
		sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, reg);

		break;

	case SND_SOC_BIAS_STANDBY:	/* Off, with power */
		/* soc doesn't do PREPARE state after record so make sure
		   that anything that needs to be turned OFF gets turned off. */
		if (codec->bias_level == SND_SOC_BIAS_STANDBY)
			break;

		/* soc calls digital_mute to unmute before record but doesn't
		   call digital_mute to mute after record. */
		sgtl5000_digital_mute(&sgtl5000_dai, 1);

		sgtl5000_mic_bias(codec, 0);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		if (reg & SGTL5000_VAG_POWERUP) {
			reg &= ~SGTL5000_VAG_POWERUP;
			sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
			msleep(400);
		}
		reg &= ~SGTL5000_DAC_POWERUP;
		reg &= ~SGTL5000_HP_POWERUP;
		reg &= ~SGTL5000_LINE_OUT_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);

		reg = sgtl5000_read(codec, SGTL5000_CHIP_DIG_POWER);
		reg &= ~SGTL5000_DAC_EN;
		sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, reg);

		sgtl5000->need_clk_for_bias = 0;
		sgtl5000_clock_gating(codec, 0);

		break;

	case SND_SOC_BIAS_OFF:	/* Off, without power */
		sgtl5000->need_clk_for_bias = 1;
		sgtl5000_clock_gating(codec, 1);

		/* must power down hp/line out after vag & dac to
		   avoid pops. */
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		ana_pwr = reg;
		reg &= ~SGTL5000_VAG_POWERUP;

		/* Workaround for sgtl5000 rev 0x11 chip audio suspend failure
		   issue on mx25 */
		/* reg &= ~SGTL5000_REFTOP_POWERUP; */

		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
		msleep(600);

		reg &= ~SGTL5000_HP_POWERUP;
		reg &= ~SGTL5000_LINE_OUT_POWERUP;
		reg &= ~SGTL5000_DAC_POWERUP;
		reg &= ~SGTL5000_ADC_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);

		/* save ANA POWER register value for resume */
		sgtl5000_write_reg_cache(codec, SGTL5000_CHIP_ANA_POWER,
					 ana_pwr);

		sgtl5000->need_clk_for_bias = 0;
		sgtl5000_clock_gating(codec, 0);

		break;
	}
	codec->bias_level = level;
	return 0;
}

#define SGTL5000_RATES (SNDRV_PCM_RATE_8000 |\
		      SNDRV_PCM_RATE_11025 |\
		      SNDRV_PCM_RATE_16000 |\
		      SNDRV_PCM_RATE_22050 |\
		      SNDRV_PCM_RATE_32000 |\
		      SNDRV_PCM_RATE_44100 |\
		      SNDRV_PCM_RATE_48000 |\
		      SNDRV_PCM_RATE_96000)

#define SGTL5000_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE)

struct snd_soc_dai_ops sgtl5000_ops = {
	.prepare = sgtl5000_pcm_prepare,
	.startup = sgtl5000_pcm_startup,
	.shutdown = sgtl5000_pcm_shutdown,
	.hw_params = sgtl5000_pcm_hw_params,
	.digital_mute = sgtl5000_digital_mute,
	.set_fmt = sgtl5000_set_dai_fmt,
	.set_sysclk = sgtl5000_set_dai_sysclk
};

struct snd_soc_dai sgtl5000_dai = {
	.name = "SGTL5000",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 2,
		     .channels_max = 2,
		     .rates = SGTL5000_RATES,
		     .formats = SGTL5000_FORMATS,
		     },
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 2,
		    .channels_max = 2,
		    .rates = SGTL5000_RATES,
		    .formats = SGTL5000_FORMATS,
		    },
	.ops = &sgtl5000_ops,
};
EXPORT_SYMBOL_GPL(sgtl5000_dai);

/*
 * Delayed work that turns off the audio clock after a delay.
 */
static void sgtl5000_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
		container_of(work, struct snd_soc_codec, delayed_work.work);
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);

	if (!sgtl5000->need_clk_for_access &&
	    !sgtl5000->need_clk_for_bias &&
	    sgtl5000->clock_on) {
		sgtl5000->clock_enable(0);
		sgtl5000->clock_on = 0;
	}
}

static int sgtl5000_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	sgtl5000_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int sgtl5000_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	unsigned int i;

	/* Restore refs first in same order as in sgtl5000_init */
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_LINREG_CTRL);
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_ANA_POWER);
	msleep(10);
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_REF_CTRL);
	sgtl5000_restore_reg(codec, SGTL5000_CHIP_LINE_OUT_CTRL);

	/* Restore everythine else */
	for (i = 1; i < sizeof(all_reg) / sizeof(int); i++)
		sgtl5000_restore_reg(codec, all_reg[i]);

	sgtl5000_write(codec, SGTL5000_DAP_CTRL, 0);

	/* Bring the codec back up to standby first to minimise pop/clicks */
	sgtl5000_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	if (codec->suspend_bias_level == SND_SOC_BIAS_ON)
		sgtl5000_set_bias_level(codec, SND_SOC_BIAS_PREPARE);
	sgtl5000_set_bias_level(codec, codec->suspend_bias_level);

	return 0;
}

static struct snd_soc_codec *sgtl5000_codec;

/*
 * initialise the SGTL5000 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int sgtl5000_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = sgtl5000_codec;
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);
	struct sgtl5000_setup_data *setup = socdev->codec_data;
	u16 reg, ana_pwr, lreg_ctrl, ref_ctrl, lo_ctrl, short_ctrl, sss;
	int vag;
	int ret = 0;

	socdev->card->codec = sgtl5000_codec;

	if ((setup != NULL) && (setup->clock_enable != NULL)) {
		sgtl5000->clock_enable = setup->clock_enable;
		sgtl5000->need_clk_for_bias = 1;
		INIT_DELAYED_WORK(&codec->delayed_work, sgtl5000_work);
	}

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms\n");
		return ret;
	}

	/* reset value */
	ana_pwr = SGTL5000_DAC_STEREO |
	    SGTL5000_LINREG_SIMPLE_POWERUP |
	    SGTL5000_STARTUP_POWERUP |
	    SGTL5000_ADC_STEREO | SGTL5000_REFTOP_POWERUP;
	lreg_ctrl = 0;
	ref_ctrl = 0;
	lo_ctrl = 0;
	short_ctrl = 0;
	sss = SGTL5000_DAC_SEL_I2S_IN << SGTL5000_DAC_SEL_SHIFT;

	/* workaround for rev 0x11: use vddd linear regulator */
	if (!sgtl5000->vddd || (sgtl5000->rev >= 0x11)) {
		/* set VDDD to 1.2v */
		lreg_ctrl |= 0x8 << SGTL5000_LINREG_VDDD_SHIFT;
		/* power internal linear regulator */
		ana_pwr |= SGTL5000_LINEREG_D_POWERUP;
	} else {
		/* turn of startup power */
		ana_pwr &= ~SGTL5000_STARTUP_POWERUP;
		ana_pwr &= ~SGTL5000_LINREG_SIMPLE_POWERUP;
	}
	if (sgtl5000->vddio < 3100 && sgtl5000->vdda < 3100) {
		/* Enable VDDC charge pump */
		ana_pwr |= SGTL5000_VDDC_CHRGPMP_POWERUP;
	}
	if (sgtl5000->vddio >= 3100 && sgtl5000->vdda >= 3100) {
		/* VDDC use VDDIO rail */
		lreg_ctrl |= SGTL5000_VDDC_ASSN_OVRD;
		if (sgtl5000->vddio >= 3100)
			lreg_ctrl |= SGTL5000_VDDC_MAN_ASSN_VDDIO <<
			    SGTL5000_VDDC_MAN_ASSN_SHIFT;
	}
	/* If PLL is powered up (such as on power cycle) leave it on. */
	reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
	ana_pwr |= reg & (SGTL5000_PLL_POWERUP | SGTL5000_VCOAMP_POWERUP);

	/* set ADC/DAC ref voltage to vdda/2 */
	vag = sgtl5000->vdda / 2;
	if (vag <= SGTL5000_ANA_GND_BASE)
		vag = 0;
	else if (vag >= SGTL5000_ANA_GND_BASE + SGTL5000_ANA_GND_STP *
		 (SGTL5000_ANA_GND_MASK >> SGTL5000_ANA_GND_SHIFT))
		vag = SGTL5000_ANA_GND_MASK >> SGTL5000_ANA_GND_SHIFT;
	else
		vag = (vag - SGTL5000_ANA_GND_BASE) / SGTL5000_ANA_GND_STP;
	ref_ctrl |= vag << SGTL5000_ANA_GND_SHIFT;

	/* set line out ref voltage to vddio/2 */
	vag = sgtl5000->vddio / 2;
	if (vag <= SGTL5000_LINE_OUT_GND_BASE)
		vag = 0;
	else if (vag >= SGTL5000_LINE_OUT_GND_BASE + SGTL5000_LINE_OUT_GND_STP *
		 SGTL5000_LINE_OUT_GND_MAX)
		vag = SGTL5000_LINE_OUT_GND_MAX;
	else
		vag = (vag - SGTL5000_LINE_OUT_GND_BASE) /
		    SGTL5000_LINE_OUT_GND_STP;
	lo_ctrl |= vag << SGTL5000_LINE_OUT_GND_SHIFT;

	/* enable small pop */
	ref_ctrl |= SGTL5000_SMALL_POP;

	/* Controls the output bias current for the lineout */
	lo_ctrl |=
	    (SGTL5000_LINE_OUT_CURRENT_360u << SGTL5000_LINE_OUT_CURRENT_SHIFT);

	/* set short detect */
	/* keep default */

	/* set routing */
	/* keep default, bypass DAP */

	sgtl5000_write(codec, SGTL5000_CHIP_LINREG_CTRL, lreg_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, ana_pwr);
	msleep(10);

	/* For rev 0x11, if vddd linear reg has been enabled, we have
	   to disable simple reg to get proper VDDD voltage.  */
	if ((ana_pwr & SGTL5000_LINEREG_D_POWERUP) && (sgtl5000->rev >= 0x11)) {
		ana_pwr &= ~SGTL5000_LINREG_SIMPLE_POWERUP;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, ana_pwr);
		msleep(10);
	}

	sgtl5000_write(codec, SGTL5000_CHIP_REF_CTRL, ref_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_LINE_OUT_CTRL, lo_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_SHORT_CTRL, short_ctrl);
	sgtl5000_write(codec, SGTL5000_CHIP_SSS_CTRL, sss);
	sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, 0);

	reg = SGTL5000_DAC_VOL_RAMP_EN |
	    SGTL5000_DAC_MUTE_RIGHT | SGTL5000_DAC_MUTE_LEFT;
	sgtl5000_write(codec, SGTL5000_CHIP_ADCDAC_CTRL, reg);

#ifdef CONFIG_ARCH_MXC
	if (cpu_is_mx25())
		sgtl5000_write(codec, SGTL5000_CHIP_PAD_STRENGTH, 0x01df);
	else
#endif
		sgtl5000_write(codec, SGTL5000_CHIP_PAD_STRENGTH, 0x015f);

	reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_ADC_CTRL);
	reg &= ~SGTL5000_ADC_VOL_M6DB;
	reg &= ~(SGTL5000_ADC_VOL_LEFT_MASK | SGTL5000_ADC_VOL_RIGHT_MASK);
	reg |= (0xf << SGTL5000_ADC_VOL_LEFT_SHIFT)
	    | (0xf << SGTL5000_ADC_VOL_RIGHT_SHIFT);
	sgtl5000_write(codec, SGTL5000_CHIP_ANA_ADC_CTRL, reg);

	reg = SGTL5000_LINE_OUT_MUTE | SGTL5000_HP_MUTE |
	    SGTL5000_HP_ZCD_EN | SGTL5000_ADC_ZCD_EN;
	sgtl5000_write(codec, SGTL5000_CHIP_ANA_CTRL, reg);

	sgtl5000_write(codec, SGTL5000_CHIP_MIC_CTRL, 0);
	sgtl5000_write(codec, SGTL5000_CHIP_CLK_TOP_CTRL, 0);
	/* disable DAP */
	sgtl5000_write(codec, SGTL5000_DAP_CTRL, 0);
	/* TODO: initialize DAP */

	snd_soc_add_controls(codec, sgtl5000_snd_controls,
			     ARRAY_SIZE(sgtl5000_snd_controls));
	sgtl5000_add_widgets(codec);

	sgtl5000_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);

	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}

/* power down chip */
static int sgtl5000_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec->control_data)
		sgtl5000_set_bias_level(codec, SND_SOC_BIAS_OFF);
	run_delayed_work(&codec->delayed_work);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_sgtl5000 = {
	.probe = sgtl5000_probe,
	.remove = sgtl5000_remove,
	.suspend = sgtl5000_suspend,
	.resume = sgtl5000_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_sgtl5000);

static __devinit int sgtl5000_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct sgtl5000_priv *sgtl5000;
	struct snd_soc_codec *codec;
	struct regulator *reg;
	int ret = 0;
	u32 val;

	if (sgtl5000_codec) {
		dev_err(&client->dev,
			"Multiple SGTL5000 devices not supported\n");
		return -ENOMEM;
	}

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	sgtl5000 = kzalloc(sizeof(struct sgtl5000_priv), GFP_KERNEL);
	if (sgtl5000 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	snd_soc_codec_set_drvdata(codec, sgtl5000);
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	i2c_set_clientdata(client, codec);
	codec->control_data = client;

	reg = regulator_get(&client->dev, "VDDIO");
	if (!IS_ERR(reg))
		sgtl5000->reg_vddio = reg;

	reg = regulator_get(&client->dev, "VDDA");
	if (!IS_ERR(reg))
		sgtl5000->reg_vdda = reg;

	reg = regulator_get(&client->dev, "VDDD");
	if (!IS_ERR(reg))
		sgtl5000->reg_vddd = reg;

	if (sgtl5000->reg_vdda) {
		sgtl5000->vdda =
		    regulator_get_voltage(sgtl5000->reg_vdda) / 1000;
		regulator_enable(sgtl5000->reg_vdda);
	}
	if (sgtl5000->reg_vddio) {
		sgtl5000->vddio =
		    regulator_get_voltage(sgtl5000->reg_vddio) / 1000;
		regulator_enable(sgtl5000->reg_vddio);
	}
	if (sgtl5000->reg_vddd) {
		sgtl5000->vddd =
		    regulator_get_voltage(sgtl5000->reg_vddd) / 1000;
		regulator_enable(sgtl5000->reg_vddd);
	} else {
		sgtl5000->vddd = 0; /* use internal regulator */
	}

	msleep(1);

	val = sgtl5000_read(codec, SGTL5000_CHIP_ID);
	if (((val & SGTL5000_PARTID_MASK) >> SGTL5000_PARTID_SHIFT) !=
	    SGTL5000_PARTID_PART_ID) {
		pr_err("Device with ID register %x is not a SGTL5000\n", val);
		ret = -ENODEV;
		goto err_codec_reg;
	}

	sgtl5000->rev = (val & SGTL5000_REVID_MASK) >> SGTL5000_REVID_SHIFT;
	dev_info(&client->dev, "SGTL5000 revision %d\n", sgtl5000->rev);

	codec->dev = &client->dev;
	codec->name = "SGTL5000";
	codec->owner = THIS_MODULE;
	codec->read = sgtl5000_read_reg_cache;
	codec->write = sgtl5000_write;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = sgtl5000_set_bias_level;
	codec->dai = &sgtl5000_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(sgtl5000_regs);
	codec->reg_cache_step = 2;
	codec->reg_cache = (void *)&sgtl5000_regs;

	sgtl5000_sync_reg_cache(codec);

	sgtl5000_codec = codec;
	sgtl5000_dai.dev = &client->dev;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err_codec_reg;
	}

	ret = snd_soc_register_dai(&sgtl5000_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAIs: %d\n", ret);
		goto err_codec_reg;
	}

	return 0;

err_codec_reg:
	if (sgtl5000->reg_vddd)
		regulator_disable(sgtl5000->reg_vddd);
	if (sgtl5000->reg_vdda)
		regulator_disable(sgtl5000->reg_vdda);
	if (sgtl5000->reg_vddio)
		regulator_disable(sgtl5000->reg_vddio);
	if (sgtl5000->reg_vddd)
		regulator_put(sgtl5000->reg_vddd);
	if (sgtl5000->reg_vdda)
		regulator_put(sgtl5000->reg_vdda);
	if (sgtl5000->reg_vddio)
		regulator_put(sgtl5000->reg_vddio);
	kfree(sgtl5000);
	kfree(codec);
	return ret;
}

static __devexit int sgtl5000_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	struct sgtl5000_priv *sgtl5000 = snd_soc_codec_get_drvdata(codec);

	if (client->dev.platform_data)
		clk_disable((struct clk *)client->dev.platform_data);

	snd_soc_unregister_dai(&sgtl5000_dai);
	snd_soc_unregister_codec(codec);

	if (sgtl5000->reg_vddio) {
		regulator_disable(sgtl5000->reg_vddio);
		regulator_put(sgtl5000->reg_vddio);
	}
	if (sgtl5000->reg_vddd) {
		regulator_disable(sgtl5000->reg_vddd);
		regulator_put(sgtl5000->reg_vddd);
	}
	if (sgtl5000->reg_vdda) {
		regulator_disable(sgtl5000->reg_vdda);
		regulator_put(sgtl5000->reg_vdda);
	}

	kfree(codec);
	kfree(sgtl5000);
	sgtl5000_codec = NULL;
	return 0;
}

static const struct i2c_device_id sgtl5000_id[] = {
	{"sgtl5000-i2c", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, sgtl5000_id);

static struct i2c_driver sgtl5000_i2c_driver = {
	.driver = {
		   .name = "sgtl5000-i2c",
		   .owner = THIS_MODULE,
		   },
	.probe = sgtl5000_i2c_probe,
	.remove = __devexit_p(sgtl5000_i2c_remove),
	.id_table = sgtl5000_id,
};

static int __init sgtl5000_modinit(void)
{
	return i2c_add_driver(&sgtl5000_i2c_driver);
}
module_init(sgtl5000_modinit);

static void __exit sgtl5000_exit(void)
{
	i2c_del_driver(&sgtl5000_i2c_driver);
}
module_exit(sgtl5000_exit);

MODULE_DESCRIPTION("ASoC SGTL5000 driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
