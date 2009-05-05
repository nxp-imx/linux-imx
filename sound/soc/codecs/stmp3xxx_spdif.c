/*
 * ALSA SoC STMP3xxx SPDIF transmitter driver
 *
 * Vladimir Barinov <vbarinov@embeddedalley.com>
 *
 * Copyright 2008 SigmaTel, Inc
 * Copyright 2008 Embedded Alley Solutions, Inc
 * Copyright 2008-2009 Freescale Semiconductor, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program  is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <asm/dma.h>

#include <mach/regs-spdif.h>

#include "stmp3xxx_spdif.h"

#define STMP3XXX_VERSION	"0.1"
struct stmp3xxx_codec_priv {
	struct device *dev;
	struct clk *clk;
};

/*
 * ALSA API
 */
static u32 spdif_regmap[] = {
	HW_SPDIF_CTRL_ADDR,
	HW_SPDIF_STAT_ADDR,
	HW_SPDIF_FRAMECTRL_ADDR,
	HW_SPDIF_SRR_ADDR,
	HW_SPDIF_DEBUG_ADDR,
	HW_SPDIF_DATA_ADDR,
	HW_SPDIF_VERSION_ADDR,
};

/*
 * ALSA core supports only 16 bit registers. It means we have to simulate it
 * by virtually splitting a 32bit SPDIF registers into two halves
 * high (bits 31:16) and low (bits 15:0). The routins abow detects which part
 * of 32bit register is accessed.
 */
static int stmp3xxx_codec_write(struct snd_soc_codec *codec,
				unsigned int reg, unsigned int value)
{
	unsigned int reg_val;
	unsigned int mask = 0xffff;

	if (reg >= SPDIF_REGNUM)
		return -EIO;

	if (reg & 0x1) {
		mask <<= 16;
		value <<= 16;
	}

	reg_val = __raw_readl(spdif_regmap[reg >> 1]);
	reg_val = (reg_val & ~mask) | value;
	__raw_writel(reg_val, spdif_regmap[reg >> 1]);

	return 0;
}

static unsigned int stmp3xxx_codec_read(struct snd_soc_codec *codec,
					unsigned int reg)
{
	unsigned int reg_val;

	if (reg >= SPDIF_REGNUM)
		return -1;

	reg_val = __raw_readl(spdif_regmap[reg >> 1]);
	if (reg & 1)
		reg_val >>= 16;

	return reg_val & 0xffff;
}

/* Codec controls */
static const struct snd_kcontrol_new stmp3xxx_snd_controls[] = {
	SOC_SINGLE("PRO", SPDIF_FRAMECTRL_L, 0, 0x1, 0),
	SOC_SINGLE("AUDIO", SPDIF_FRAMECTRL_L, 1, 0x1, 0),
	SOC_SINGLE("COPY", SPDIF_FRAMECTRL_L, 2, 0x1, 0),
	SOC_SINGLE("PRE", SPDIF_FRAMECTRL_L, 3, 0x1, 0),
	SOC_SINGLE("CC", SPDIF_FRAMECTRL_L, 4, 0x7F, 0),
	SOC_SINGLE("L", SPDIF_FRAMECTRL_L, 12, 0x1, 0),
	SOC_SINGLE("V", SPDIF_FRAMECTRL_L, 13, 0x1, 0),
	SOC_SINGLE("USER DATA", SPDIF_FRAMECTRL_L, 14, 0x1, 0),
	SOC_SINGLE("AUTO MUTE", SPDIF_FRAMECTRL_H, 16, 0x1, 0),
	SOC_SINGLE("V CONFIG", SPDIF_FRAMECTRL_H, 17, 0x1, 0),
};

/* add non dapm controls */
static int stmp3xxx_codec_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(stmp3xxx_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
		snd_soc_cnew(&stmp3xxx_snd_controls[i],
			     codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

struct spdif_srr {
	u32 rate;
	u32 basemult;
	u32 rate_factor;
};

static struct spdif_srr srr_values[] = {
	{96000, 0x2, 0x0BB80},
	{88200, 0x2, 0x0AC44},
	{64000, 0x2, 0x07D00},
	{48000, 0x1, 0x0BB80},
	{44100, 0x1, 0x0AC44},
	{32000, 0x1, 0x07D00},
};

static inline int get_srr_values(int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(srr_values); i++)
		if (srr_values[i].rate == rate)
			return i;

	return -1;
}

static int stmp3xxx_codec_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int i;
	u32 srr_value = 0;
	u32 basemult;

	i = get_srr_values(params_rate(params));
	if (i < 0)
		printk(KERN_WARNING "%s doesn't support rate %d\n",
		       codec->name, params_rate(params));
	else {
		basemult = srr_values[i].basemult;

		srr_value = BF_SPDIF_SRR_BASEMULT(basemult) |
			    BF_SPDIF_SRR_RATE(srr_values[i].rate_factor);

		if (playback)
			HW_SPDIF_SRR_WR(srr_value);
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (playback)
			HW_SPDIF_CTRL_SET(BM_SPDIF_CTRL_WORD_LENGTH);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		if (playback)
			HW_SPDIF_CTRL_CLR(BM_SPDIF_CTRL_WORD_LENGTH);
		break;
	default:
		printk(KERN_WARNING "%s doesn't support format %d\n",
		       codec->name, params_format(params));
	}

	return 0;
}

static void
stmp3xxx_codec_spdif_enable(struct stmp3xxx_codec_priv *stmp3xxx_spdif)
{
	/* Move SPDIF codec out of reset */
	HW_SPDIF_CTRL_CLR(BM_SPDIF_CTRL_SFTRST);

	/* Ungate SPDIF clocks */
	HW_SPDIF_CTRL_CLR(BM_SPDIF_CTRL_CLKGATE);

	/* 16 bit word length */
	HW_SPDIF_CTRL_SET(BM_SPDIF_CTRL_WORD_LENGTH);
}

static void
stmp3xxx_codec_spdif_disable(struct stmp3xxx_codec_priv *stmp3xxx_spdif)
{
	/* Gate SPDIF clocks */
	HW_SPDIF_CTRL_SET(BM_SPDIF_CTRL_CLKGATE);
}

static void stmp3xxx_codec_init(struct snd_soc_codec *codec)
{
	struct stmp3xxx_codec_priv *stmp3xxx_spdif = codec->private_data;

	/* Soft reset SPDIF block */
	HW_SPDIF_CTRL_SET(BM_SPDIF_CTRL_SFTRST);
	while (!(HW_SPDIF_CTRL_RD() & BM_SPDIF_CTRL_CLKGATE));

	stmp3xxx_codec_spdif_enable(stmp3xxx_spdif);

	stmp3xxx_codec_add_controls(codec);
}

static void stmp3xxx_codec_exit(struct snd_soc_codec *codec)
{
	struct stmp3xxx_codec_priv *stmp3xxx_spdif = codec->private_data;

	stmp3xxx_codec_spdif_disable(stmp3xxx_spdif);
}

#define STMP3XXX_SPDIF_RATES	(SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_64000 | \
				 SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)
#define STMP3XXX_SPDIF_FORMATS  (SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE)

struct snd_soc_dai stmp3xxx_spdif_codec_dai = {
	.name = "stmp3xxx spdif",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = STMP3XXX_SPDIF_RATES,
		.formats = STMP3XXX_SPDIF_FORMATS,
	},
	.ops = {
		.hw_params = stmp3xxx_codec_hw_params,
	},
};
EXPORT_SYMBOL_GPL(stmp3xxx_spdif_codec_dai);

static int stmp3xxx_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct stmp3xxx_codec_priv *stmp3xxx_spdif;
	int ret = 0;

	printk(KERN_INFO
		"STMP3XXX SPDIF Audio Transmitter %s\n", STMP3XXX_VERSION);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	stmp3xxx_spdif =
		kzalloc(sizeof(struct stmp3xxx_codec_priv), GFP_KERNEL);
	if (stmp3xxx_spdif == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->name = "stmp3xxx spdif";
	codec->owner = THIS_MODULE;
	codec->private_data = stmp3xxx_spdif;
	codec->read = stmp3xxx_codec_read;
	codec->write = stmp3xxx_codec_write;
	codec->dai = &stmp3xxx_spdif_codec_dai;
	codec->num_dai = 1;
	socdev->codec = codec;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to create pcms\n", __func__);
		goto pcm_err;
	}

	/* Turn on audio clock */
	stmp3xxx_spdif->dev = &pdev->dev;
	stmp3xxx_spdif->clk = clk_get(stmp3xxx_spdif->dev, "spdif");
	if (IS_ERR(stmp3xxx_spdif->clk)) {
		ret = PTR_ERR(stmp3xxx_spdif->clk);
		printk(KERN_ERR "%s: Clocks initialization failed\n", __func__);
		goto clk_err;
	}
	clk_enable(stmp3xxx_spdif->clk);

	stmp3xxx_codec_init(codec);

	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register card\n", __func__);
		goto card_err;
	}

	return ret;

card_err:
	clk_disable(stmp3xxx_spdif->clk);
	clk_put(stmp3xxx_spdif->clk);
clk_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(socdev->codec);
	return ret;
}

static int stmp3xxx_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct stmp3xxx_codec_priv *stmp3xxx_spdif;

	if (codec == NULL)
		return 0;

	stmp3xxx_spdif = codec->private_data;

	clk_disable(stmp3xxx_spdif->clk);
	clk_put(stmp3xxx_spdif->clk);

	stmp3xxx_codec_exit(codec);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	kfree(socdev->codec);

	return 0;
}

#ifdef CONFIG_PM
static int stmp3xxx_codec_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct stmp3xxx_codec_priv *stmp3xxx_spdif;
	int ret = -EINVAL;

	if (codec == NULL)
		goto out;

	stmp3xxx_spdif = codec->private_data;

	stmp3xxx_codec_spdif_disable(stmp3xxx_spdif);
	clk_disable(stmp3xxx_spdif->clk);
	ret = 0;

out:
	return ret;
}

static int stmp3xxx_codec_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct stmp3xxx_codec_priv *stmp3xxx_spdif;
	int ret = -EINVAL;

	if (codec == NULL)
		goto out;

	stmp3xxx_spdif = codec->private_data;
	clk_enable(stmp3xxx_spdif->clk);

	/* Soft reset SPDIF block */
	HW_SPDIF_CTRL_SET(BM_SPDIF_CTRL_SFTRST);
	while (!(HW_SPDIF_CTRL_RD() & BM_SPDIF_CTRL_CLKGATE));

	stmp3xxx_codec_spdif_enable(stmp3xxx_spdif);

	ret = 0;

out:
	return ret;
}
#else
#define stmp3xxx_codec_suspend	NULL
#define stmp3xxx_codec_resume	NULL
#endif /* CONFIG_PM */

struct snd_soc_codec_device soc_spdif_codec_dev_stmp3xxx = {
	.probe		= stmp3xxx_codec_probe,
	.remove		= stmp3xxx_codec_remove,
	.suspend	= stmp3xxx_codec_suspend,
	.resume		= stmp3xxx_codec_resume,
};
EXPORT_SYMBOL_GPL(soc_spdif_codec_dev_stmp3xxx);

MODULE_DESCRIPTION("STMP3XXX SPDIF transmitter");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
