/*
 * imx-3stack-wm8350.c  --  i.MX 3Stack Driver for Wolfson WM8350 Codec
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Copyright (C) 2007-2010 Freescale Semiconductor, Inc.
 *
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    19th Jun 2007   Initial version.
 *
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
#include <linux/err.h>
#include <linux/mfd/wm8350/core.h>
#include <linux/mfd/wm8350/audio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <mach/dma.h>
#include <mach/clock.h>

#include "../codecs/wm8350.h"
#include "imx-ssi.h"
#include "imx-pcm.h"

void gpio_activate_audio_ports(void);

/* SSI BCLK and LRC master */
#define WM8350_SSI_MASTER	1

struct imx_3stack_priv {
	int lr_clk_active;
	int playback_active;
	int capture_active;
	struct platform_device *pdev;
	struct wm8350 *wm8350;
};

static struct imx_3stack_priv machine_priv;

struct _wm8350_audio {
	unsigned int channels;
	snd_pcm_format_t format;
	unsigned int rate;
	unsigned int sysclk;
	unsigned int bclkdiv;
	unsigned int clkdiv;
	unsigned int lr_rate;
};

/* in order of power consumption per rate (lowest first) */
static const struct _wm8350_audio wm8350_audio[] = {
	/* 16bit mono modes */

	{1, SNDRV_PCM_FORMAT_S16_LE, 8000, 12288000,
	WM8350_BCLK_DIV_48, WM8350_DACDIV_6, 32,},
	{1, SNDRV_PCM_FORMAT_S16_LE, 16000, 12288000,
	 WM8350_BCLK_DIV_24, WM8350_DACDIV_6, 32,},
	{1, SNDRV_PCM_FORMAT_S16_LE, 32000, 12288000,
	 WM8350_BCLK_DIV_12, WM8350_DACDIV_3, 32,},
	{1, SNDRV_PCM_FORMAT_S16_LE, 48000, 12288000,
	 WM8350_BCLK_DIV_8, WM8350_DACDIV_2, 32,},
	{1, SNDRV_PCM_FORMAT_S16_LE, 96000, 24576000,
	 WM8350_BCLK_DIV_8, WM8350_DACDIV_2, 32,},
	{1, SNDRV_PCM_FORMAT_S16_LE, 11025, 11289600,
	 WM8350_BCLK_DIV_32, WM8350_DACDIV_4, 32,},
	{1, SNDRV_PCM_FORMAT_S16_LE, 22050, 11289600,
	 WM8350_BCLK_DIV_16, WM8350_DACDIV_4, 32,},
	{1, SNDRV_PCM_FORMAT_S16_LE, 44100, 11289600,
	 WM8350_BCLK_DIV_8, WM8350_DACDIV_2, 32,},
	{1, SNDRV_PCM_FORMAT_S16_LE, 88200, 22579200,
	 WM8350_BCLK_DIV_8, WM8350_DACDIV_2, 32,},

	/* 16 bit stereo modes */
	{2, SNDRV_PCM_FORMAT_S16_LE, 8000, 12288000,
	 WM8350_BCLK_DIV_48, WM8350_DACDIV_6, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 16000, 12288000,
	 WM8350_BCLK_DIV_24, WM8350_DACDIV_3, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 32000, 12288000,
	 WM8350_BCLK_DIV_12, WM8350_DACDIV_1_5, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 48000, 12288000,
	 WM8350_BCLK_DIV_8, WM8350_DACDIV_1, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 96000, 24576000,
	 WM8350_BCLK_DIV_8, WM8350_DACDIV_1, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 11025, 11289600,
	 WM8350_BCLK_DIV_32, WM8350_DACDIV_4, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 22050, 11289600,
	 WM8350_BCLK_DIV_16, WM8350_DACDIV_2, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 44100, 11289600,
	 WM8350_BCLK_DIV_8, WM8350_DACDIV_1, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 88200, 22579200,
	 WM8350_BCLK_DIV_8, WM8350_DACDIV_1, 32,},

	/* 24bit stereo modes */
	{2, SNDRV_PCM_FORMAT_S24_LE, 48000, 12288000,
	 WM8350_BCLK_DIV_4, WM8350_DACDIV_1, 64,},
	{2, SNDRV_PCM_FORMAT_S24_LE, 96000, 24576000,
	 WM8350_BCLK_DIV_4, WM8350_DACDIV_1, 64,},
	{2, SNDRV_PCM_FORMAT_S24_LE, 44100, 11289600,
	 WM8350_BCLK_DIV_4, WM8350_DACDIV_1, 64,},
	{2, SNDRV_PCM_FORMAT_S24_LE, 88200, 22579200,
	 WM8350_BCLK_DIV_4, WM8350_DACDIV_1, 64,},
};

#if WM8350_SSI_MASTER
static int imx_3stack_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->card->codec;
	struct wm8350 *wm8350 = codec->control_data;
	struct imx_3stack_priv *priv = &machine_priv;

	/* In master mode the LR clock can come from either the DAC or ADC.
	 * We use the LR clock from whatever stream is enabled first.
	 */

	if (!priv->lr_clk_active) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			wm8350_clear_bits(wm8350, WM8350_CLOCK_CONTROL_2,
					  WM8350_LRC_ADC_SEL);
		else
			wm8350_set_bits(wm8350, WM8350_CLOCK_CONTROL_2,
					WM8350_LRC_ADC_SEL);
	}
	priv->lr_clk_active++;
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		priv->capture_active = 1;
	else
		priv->playback_active = 1;
	return 0;
}
#else
#define imx_3stack_startup NULL
#endif

static int imx_3stack_audio_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct imx_3stack_priv *priv = &machine_priv;
	int ret = 0;
	int i, found = 0;
	snd_pcm_format_t format = params_format(params);
	unsigned int rate = params_rate(params);
	unsigned int channels = params_channels(params);
	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;
	u32 dai_format;

	/* only need to do this once as capture and playback are sync */
	if (priv->lr_clk_active > 1)
		return 0;

	/* find the correct audio parameters */
	for (i = 0; i < ARRAY_SIZE(wm8350_audio); i++) {
		if (rate == wm8350_audio[i].rate &&
		    format == wm8350_audio[i].format &&
		    channels == wm8350_audio[i].channels) {
			found = 1;
			break;
		}
	}
	if (!found) {
		printk(KERN_ERR "%s: invalid params\n", __func__);
		return -EINVAL;
	}

#if WM8350_SSI_MASTER
	/* codec FLL input is 32768 kHz from MCLK */
	snd_soc_dai_set_pll(codec_dai, 0, 32768, wm8350_audio[i].sysclk);
#else
	/* codec FLL input is rate from DAC LRC */
	snd_soc_dai_set_pll(codec_dai, 0, rate, wm8350_audio[i].sysclk);
#endif

#if WM8350_SSI_MASTER
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM;

	ssi_mode->sync_mode = 1;
	if (channels == 1)
		ssi_mode->network_mode = 0;
	else
		ssi_mode->network_mode = 1;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		dai_format &= ~SND_SOC_DAIFMT_INV_MASK;
		/* Invert frame to switch mic from right channel to left */
		dai_format |= SND_SOC_DAIFMT_NB_IF;
	}

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels);

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set 32KHZ as the codec system clock for DAC and ADC */
	snd_soc_dai_set_sysclk(codec_dai, WM8350_MCLK_SEL_PLL_32K,
			       wm8350_audio[i].sysclk, SND_SOC_CLOCK_IN);
#else
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels);

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set DAC LRC as the codec system clock for DAC and ADC */
	snd_soc_dai_set_sysclk(codec_dai, WM8350_MCLK_SEL_PLL_DAC,
			       wm8350_audio[i].sysclk, SND_SOC_CLOCK_IN);
#endif

	/* set the SSI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);

	/* set codec BCLK division for sample rate */
	snd_soc_dai_set_clkdiv(codec_dai, WM8350_BCLK_CLKDIV,
			       wm8350_audio[i].bclkdiv);

	/* DAI is synchronous and clocked with DAC LRCLK & ADC LRC */
	snd_soc_dai_set_clkdiv(codec_dai,
			       WM8350_DACLR_CLKDIV,
			       wm8350_audio[i].lr_rate);
	snd_soc_dai_set_clkdiv(codec_dai,
			       WM8350_ADCLR_CLKDIV,
			       wm8350_audio[i].lr_rate);

	/* now configure DAC and ADC clocks */
	snd_soc_dai_set_clkdiv(codec_dai,
			       WM8350_DAC_CLKDIV, wm8350_audio[i].clkdiv);

	snd_soc_dai_set_clkdiv(codec_dai,
			       WM8350_ADC_CLKDIV, wm8350_audio[i].clkdiv);

	return 0;
}

static void imx_3stack_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->card->codec;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct imx_3stack_priv *priv = &machine_priv;
	struct wm8350 *wm8350 = codec->control_data;

	/* disable the PLL if there are no active Tx or Rx channels */
	if (!codec_dai->active)
		snd_soc_dai_set_pll(codec_dai, 0, 0, 0);
	priv->lr_clk_active--;

	/*
	 * We need to keep track of active streams in master mode and
	 * switch LRC source if necessary.
	 */

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		priv->capture_active = 0;
	else
		priv->playback_active = 0;

	if (priv->capture_active)
		wm8350_set_bits(wm8350, WM8350_CLOCK_CONTROL_2,
				WM8350_LRC_ADC_SEL);
	else if (priv->playback_active)
		wm8350_clear_bits(wm8350, WM8350_CLOCK_CONTROL_2,
				  WM8350_LRC_ADC_SEL);
}

/*
 * imx_3stack WM8350 HiFi DAI operations.
 */
static struct snd_soc_ops imx_3stack_ops = {
	.startup = imx_3stack_startup,
	.shutdown = imx_3stack_shutdown,
	.hw_params = imx_3stack_audio_hw_params,
};

static void imx_3stack_init_dam(int ssi_port, int dai_port)
{
	unsigned int ssi_ptcr = 0;
	unsigned int dai_ptcr = 0;
	unsigned int ssi_pdcr = 0;
	unsigned int dai_pdcr = 0;
	/* WM8350 uses SSI1 or SSI2 via AUDMUX port dai_port for audio */

	/* reset port ssi_port & dai_port */
	__raw_writel(0, DAM_PTCR(ssi_port));
	__raw_writel(0, DAM_PTCR(dai_port));
	__raw_writel(0, DAM_PDCR(ssi_port));
	__raw_writel(0, DAM_PDCR(dai_port));

	/* set to synchronous */
	ssi_ptcr |= AUDMUX_PTCR_SYN;
	dai_ptcr |= AUDMUX_PTCR_SYN;

#if WM8350_SSI_MASTER
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TFSDIR;
	ssi_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, dai_port);

	/* set Tx Clock direction and source dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TCLKDIR;
	ssi_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, dai_port);
#else
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  ssi_port --> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TFSDIR;
	dai_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, ssi_port);

	/* set Tx Clock direction and source ssi_port--> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TCLKDIR;
	dai_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, ssi_port);
#endif

	__raw_writel(ssi_ptcr, DAM_PTCR(ssi_port));
	__raw_writel(dai_ptcr, DAM_PTCR(dai_port));
	__raw_writel(ssi_pdcr, DAM_PDCR(ssi_port));
	__raw_writel(dai_pdcr, DAM_PDCR(dai_port));
}

static const struct snd_soc_dapm_route audio_map[] = {
	/* SiMIC --> IN1LN (with automatic bias) via SP1 */
	{"IN1RP", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "SiMIC"},

	/* Mic 1 Jack --> IN1LN and IN1LP (with automatic bias) */
	{"IN1LN", NULL, "Mic Bias"},
	{"IN1LP", NULL, "Mic1 Jack"},
	{"Mic Bias", NULL, "Mic1 Jack"},

	/* Mic 2 Jack --> IN1RN and IN1RP (with automatic bias) */
	{"IN1RN", NULL, "Mic2 Jack"},
	{"IN1RP", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic2 Jack"},

	/* Line in Jack --> AUX (L+R) */
	{"IN3R", NULL, "Line In Jack"},
	{"IN3L", NULL, "Line In Jack"},

	/* Out1 --> Headphone Jack */
	{"Headphone Jack", NULL, "OUT1R"},
	{"Headphone Jack", NULL, "OUT1L"},

	/* Out1 --> Line Out Jack */
	{"Line Out Jack", NULL, "OUT2R"},
	{"Line Out Jack", NULL, "OUT2L"},
};

static int wm8350_jack_func;
static int wm8350_spk_func;

static void headphone_detect_handler(struct work_struct *work)
{
	struct imx_3stack_priv *priv = &machine_priv;
	struct platform_device *pdev = priv->pdev;
	struct wm8350 *wm8350 = priv->wm8350;

	sysfs_notify(&pdev->dev.kobj, NULL, "headphone");
	wm8350_unmask_irq(wm8350, WM8350_IRQ_CODEC_JCK_DET_R);
}

static DECLARE_DELAYED_WORK(hp_event, headphone_detect_handler);

static void imx_3stack_jack_handler(struct wm8350 *wm8350, int irq, void *data)
{
	wm8350_mask_irq(wm8350, WM8350_IRQ_CODEC_JCK_DET_R);
	schedule_delayed_work(&hp_event, msecs_to_jiffies(200));
}

static ssize_t show_headphone(struct device_driver *dev, char *buf)
{
	struct imx_3stack_priv *priv = &machine_priv;
	u16 reg;

	reg = wm8350_reg_read(priv->wm8350, WM8350_JACK_PIN_STATUS);

	if (reg & WM8350_JACK_R_LVL)
		strcpy(buf, "speaker\n");
	else
		strcpy(buf, "headphone\n");

	return strlen(buf);
}

static DRIVER_ATTR(headphone, S_IRUGO | S_IWUSR, show_headphone, NULL);

static const char *jack_function[] = { "off", "on"
};

static const char *spk_function[] = { "off", "on" };

static const struct soc_enum wm8350_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static int wm8350_get_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = wm8350_jack_func;
	return 0;
}

static int wm8350_set_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (wm8350_jack_func == ucontrol->value.enumerated.item[0])
		return 0;

	wm8350_jack_func = ucontrol->value.enumerated.item[0];
	if (wm8350_jack_func)
		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");

	snd_soc_dapm_sync(codec);
	return 1;
}

static int wm8350_get_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = wm8350_spk_func;
	return 0;
}

static int wm8350_set_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (wm8350_spk_func == ucontrol->value.enumerated.item[0])
		return 0;

	wm8350_spk_func = ucontrol->value.enumerated.item[0];
	if (wm8350_spk_func)
		snd_soc_dapm_enable_pin(codec, "Line Out Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Line Out Jack");

	snd_soc_dapm_sync(codec);
	return 1;
}

static int spk_amp_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct imx_3stack_priv *priv = &machine_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->amp_enable == NULL)
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event))
		plat->amp_enable(1);
	else
		plat->amp_enable(0);

	return 0;
}

static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("SiMIC", NULL),
	SND_SOC_DAPM_MIC("Mic1 Jack", NULL),
	SND_SOC_DAPM_MIC("Mic2 Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", spk_amp_event),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static const struct snd_kcontrol_new wm8350_machine_controls[] = {
	SOC_ENUM_EXT("Jack Function", wm8350_enum[0], wm8350_get_jack,
		     wm8350_set_jack),
	SOC_ENUM_EXT("Speaker Function", wm8350_enum[1], wm8350_get_spk,
		     wm8350_set_spk),
};

static int imx_3stack_wm8350_init(struct snd_soc_codec *codec)
{
	struct imx_3stack_priv *priv = &machine_priv;
	struct wm8350 *wm8350 = priv->wm8350;
	int i, ret;

	codec->control_data = wm8350;

	/* Add imx_3stack specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8350_machine_controls); i++) {
		ret = snd_ctl_add(codec->card,
				  snd_soc_cnew(&wm8350_machine_controls[i],
					       codec, NULL));
		if (ret < 0)
			return ret;
	}

	/* Add imx_3stack specific widgets */
	snd_soc_dapm_new_controls(codec, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));

	/* Set up imx_3stack specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;

}

/* imx_3stack digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link imx_3stack_dai = {
	.name = "WM8350",
	.stream_name = "WM8350",
	.codec_dai = &wm8350_dai,
	.init = imx_3stack_wm8350_init,
	.ops = &imx_3stack_ops,
};

static int imx_3stack_machine_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct imx_3stack_priv *priv = &machine_priv;
	struct wm8350 *wm8350 = priv->wm8350;

	socdev->codec_data = wm8350;

	return 0;
}

static struct snd_soc_card snd_soc_card_imx_3stack = {
	.name = "imx-3stack",
	.platform = &imx_soc_platform,
	.dai_link = &imx_3stack_dai,
	.num_links = 1,
	.probe = imx_3stack_machine_probe,
};

static struct snd_soc_device imx_3stack_snd_devdata = {
	.card = &snd_soc_card_imx_3stack,
	.codec_dev = &soc_codec_dev_wm8350,
};

static int __devinit imx_3stack_wm8350_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_3stack_priv *priv = &machine_priv;
	struct wm8350 *wm8350 = plat->priv;
	struct snd_soc_dai *wm8350_cpu_dai;
	int ret = 0;
	u16 reg;

	priv->pdev = pdev;
	priv->wm8350 = wm8350;

	gpio_activate_audio_ports();
	imx_3stack_init_dam(plat->src_port, plat->ext_port);

	if (plat->src_port == 2)
		wm8350_cpu_dai =  imx_ssi_dai[2];
	else
		wm8350_cpu_dai = imx_ssi_dai[0];

	imx_3stack_dai.cpu_dai = wm8350_cpu_dai;

	ret = driver_create_file(pdev->dev.driver, &driver_attr_headphone);
	if (ret < 0) {
		pr_err("%s:failed to create driver_attr_headphone\n", __func__);
		return ret;
	}

	/* enable slow clock gen for jack detect */
	reg = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_4);
	wm8350_reg_write(wm8350, WM8350_POWER_MGMT_4, reg | WM8350_TOCLK_ENA);
	/* enable jack detect */
	reg = wm8350_reg_read(wm8350, WM8350_JACK_DETECT);
	wm8350_reg_write(wm8350, WM8350_JACK_DETECT, reg | WM8350_JDR_ENA);
	wm8350_register_irq(wm8350, WM8350_IRQ_CODEC_JCK_DET_R,
			    imx_3stack_jack_handler, NULL);
	wm8350_unmask_irq(wm8350, WM8350_IRQ_CODEC_JCK_DET_R);

	wm8350_jack_func = 1;
	wm8350_spk_func = 1;

	return 0;
}

static int imx_3stack_wm8350_remove(struct platform_device *pdev)
{
	struct imx_3stack_priv *priv = &machine_priv;
	struct wm8350 *wm8350 = priv->wm8350;

	wm8350_mask_irq(wm8350, WM8350_IRQ_CODEC_JCK_DET_R);
	wm8350_free_irq(wm8350, WM8350_IRQ_CODEC_JCK_DET_R);

	return 0;
}

static struct platform_driver imx_3stack_wm8350_audio_driver = {
	.probe = imx_3stack_wm8350_probe,
	.remove = __devexit_p(imx_3stack_wm8350_remove),
	.driver = {
		   .name = "wm8350-imx-3stack-audio",
		   },
};

static struct platform_device *imx_3stack_snd_device;

static int __init imx_3stack_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_3stack_wm8350_audio_driver);
	if (ret)
		return -ENOMEM;

	imx_3stack_snd_device = platform_device_alloc("soc-audio", -1);
	if (!imx_3stack_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_3stack_snd_device, &imx_3stack_snd_devdata);
	imx_3stack_snd_devdata.dev = &imx_3stack_snd_device->dev;
	ret = platform_device_add(imx_3stack_snd_device);

	if (ret)
		platform_device_put(imx_3stack_snd_device);

	return ret;
}

static void __exit imx_3stack_exit(void)
{
	platform_driver_unregister(&imx_3stack_wm8350_audio_driver);
	platform_device_unregister(imx_3stack_snd_device);
}

module_init(imx_3stack_init);
module_exit(imx_3stack_exit);

MODULE_AUTHOR("Liam Girdwood");
MODULE_DESCRIPTION("PMIC WM8350 Driver for i.MX 3STACK");
MODULE_LICENSE("GPL");
