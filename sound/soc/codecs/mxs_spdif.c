/*
 * ALSA SoC MXS SPDIF codec driver
 *
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc.
 *
 * Based on stmp3xxx_spdif.h by:
 * Vladimir Barinov <vbarinov@embeddedalley.com>
 *
 * Copyright 2008 SigmaTel, Inc
 * Copyright 2008 Embedded Alley Solutions, Inc
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program  is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <asm/dma.h>
#include <mach/hardware.h>

#include "mxs_spdif.h"

#define REGS_SPDIF_BASE IO_ADDRESS(SPDIF_PHYS_ADDR)

#define BF(value, field) (((value) << BP_##field) & BM_##field)

struct mxs_codec_priv {
	struct clk *clk;
	struct snd_soc_codec codec;
};

/*
 * ALSA API
 */
static void __iomem *spdif_regmap[] = {
	REGS_SPDIF_BASE + HW_SPDIF_CTRL,
	REGS_SPDIF_BASE + HW_SPDIF_STAT,
	REGS_SPDIF_BASE + HW_SPDIF_FRAMECTRL,
	REGS_SPDIF_BASE + HW_SPDIF_SRR,
	REGS_SPDIF_BASE + HW_SPDIF_DEBUG,
	REGS_SPDIF_BASE + HW_SPDIF_DATA,
	REGS_SPDIF_BASE + HW_SPDIF_VERSION,
};

/*
 * ALSA core supports only 16 bit registers. It means we have to simulate it
 * by virtually splitting a 32bit SPDIF registers into two halves
 * high (bits 31:16) and low (bits 15:0). The routins abow detects which part
 * of 32bit register is accessed.
 */
static int mxs_codec_write(struct snd_soc_codec *codec,
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

static unsigned int mxs_codec_read(struct snd_soc_codec *codec,
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
static const struct snd_kcontrol_new mxs_snd_controls[] = {
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

static int mxs_codec_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
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

		srr_value = BF(basemult, SPDIF_SRR_BASEMULT) |
		    BF(srr_values[i].rate_factor, SPDIF_SRR_RATE);

		if (playback)
			__raw_writel(srr_value, REGS_SPDIF_BASE + HW_SPDIF_SRR);
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (playback)
			__raw_writel(BM_SPDIF_CTRL_WORD_LENGTH,
				     REGS_SPDIF_BASE + HW_SPDIF_CTRL_SET);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		if (playback)
			__raw_writel(BM_SPDIF_CTRL_WORD_LENGTH,
				     REGS_SPDIF_BASE + HW_SPDIF_CTRL_CLR);
		break;
	default:
		printk(KERN_WARNING "%s doesn't support format %d\n",
		       codec->name, params_format(params));
	}

	return 0;
}

static void mxs_codec_spdif_enable(struct mxs_codec_priv *mxs_spdif)
{
	/* Move SPDIF codec out of reset */
	__raw_writel(BM_SPDIF_CTRL_SFTRST, REGS_SPDIF_BASE + HW_SPDIF_CTRL_CLR);

	/* Ungate SPDIF clocks */
	__raw_writel(BM_SPDIF_CTRL_CLKGATE,
		     REGS_SPDIF_BASE + HW_SPDIF_CTRL_CLR);

	/* 16 bit word length */
	__raw_writel(BM_SPDIF_CTRL_WORD_LENGTH,
		     REGS_SPDIF_BASE + HW_SPDIF_CTRL_SET);
}

static void mxs_codec_spdif_disable(struct mxs_codec_priv *mxs_spdif)
{
	/* Gate SPDIF clocks */
	__raw_writel(BM_SPDIF_CTRL_CLKGATE,
		     REGS_SPDIF_BASE + HW_SPDIF_CTRL_SET);
}

static void mxs_codec_init(struct snd_soc_codec *codec)
{
	struct mxs_codec_priv *mxs_spdif = codec->drvdata;

	/* Soft reset SPDIF block */
	__raw_writel(BM_SPDIF_CTRL_SFTRST, REGS_SPDIF_BASE + HW_SPDIF_CTRL_SET);

	while (!(__raw_readl(REGS_SPDIF_BASE + HW_SPDIF_CTRL)
		 & BM_SPDIF_CTRL_CLKGATE))
		;

	mxs_codec_spdif_enable(mxs_spdif);

	snd_soc_add_controls(codec, mxs_snd_controls,
			     ARRAY_SIZE(mxs_snd_controls));
}

static void mxs_codec_exit(struct snd_soc_codec *codec)
{
	struct mxs_codec_priv *mxs_spdif = codec->drvdata;

	mxs_codec_spdif_disable(mxs_spdif);
}

struct snd_soc_dai_ops mxs_spdif_codec_dai_ops = {
	.hw_params = mxs_codec_hw_params,
};

struct snd_soc_dai mxs_spdif_codec_dai = {
	.name = "mxs spdif",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 2,
		     .channels_max = 2,
		     .rates = MXS_SPDIF_RATES,
		     .formats = MXS_SPDIF_FORMATS,
		     },
	.ops = &mxs_spdif_codec_dai_ops,
};
EXPORT_SYMBOL_GPL(mxs_spdif_codec_dai);

static struct snd_soc_codec *mxs_spdif_codec;

static int mxs_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	socdev->card->codec = mxs_spdif_codec;
	codec = mxs_spdif_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms\n");
		return ret;
	}

	mxs_codec_init(codec);

	return 0;
}

static int mxs_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	mxs_codec_exit(codec);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

#ifdef CONFIG_PM
static int mxs_codec_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	struct mxs_codec_priv *mxs_spdif;
	int ret = -EINVAL;

	if (codec == NULL)
		goto out;

	mxs_spdif = codec->drvdata;

	mxs_codec_spdif_disable(mxs_spdif);
	clk_disable(mxs_spdif->clk);
	ret = 0;

out:
	return ret;
}

static int mxs_codec_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	struct mxs_codec_priv *mxs_spdif;
	int ret = -EINVAL;

	if (codec == NULL)
		goto out;

	mxs_spdif = codec->drvdata;
	clk_enable(mxs_spdif->clk);

	/* Soft reset SPDIF block */
	__raw_writel(BM_SPDIF_CTRL_SFTRST, REGS_SPDIF_BASE + HW_SPDIF_CTRL_SET);
	while (!(__raw_readl(REGS_SPDIF_BASE + HW_SPDIF_CTRL)
		 & BM_SPDIF_CTRL_CLKGATE))
		;

	mxs_codec_spdif_enable(mxs_spdif);

	ret = 0;

out:
	return ret;
}
#else
#define mxs_codec_suspend	NULL
#define mxs_codec_resume	NULL
#endif /* CONFIG_PM */

struct snd_soc_codec_device soc_spdif_codec_dev_mxs = {
	.probe = mxs_codec_probe,
	.remove = mxs_codec_remove,
	.suspend = mxs_codec_suspend,
	.resume = mxs_codec_resume,
};
EXPORT_SYMBOL_GPL(soc_spdif_codec_dev_mxs);

static int __init mxs_spdif_probe(struct platform_device *pdev)
{
	struct snd_soc_codec *codec;
	struct mxs_codec_priv *mxs_spdif;
	int ret = 0;

	dev_info(&pdev->dev, "MXS SPDIF Audio Transmitter\n");

	mxs_spdif = kzalloc(sizeof(struct mxs_codec_priv), GFP_KERNEL);
	if (mxs_spdif == NULL)
		return -ENOMEM;

	codec = &mxs_spdif->codec;
	codec->name = "mxs spdif";
	codec->owner = THIS_MODULE;
	codec->drvdata = mxs_spdif;
	codec->read = mxs_codec_read;
	codec->write = mxs_codec_write;
	codec->dai = &mxs_spdif_codec_dai;
	codec->num_dai = 1;

	platform_set_drvdata(pdev, mxs_spdif);
	mxs_spdif_codec = codec;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* Turn on audio clock */
	mxs_spdif->clk = clk_get(&pdev->dev, "spdif");
	if (IS_ERR(mxs_spdif->clk)) {
		ret = PTR_ERR(mxs_spdif->clk);
		dev_err(&pdev->dev, "Clocks initialization failed\n");
		goto clk_err;
	}
	clk_enable(mxs_spdif->clk);

	ret = snd_soc_register_codec(codec);
	if (ret) {
		dev_err(&pdev->dev, "failed to register card\n");
		goto card_err;
	}

	ret = snd_soc_register_dai(&mxs_spdif_codec_dai);
	if (ret) {
		dev_err(&pdev->dev, "failed to register card\n");
		goto dai_err;
	}

	return 0;

dai_err:
	snd_soc_unregister_codec(codec);
card_err:
	clk_disable(mxs_spdif->clk);
	clk_put(mxs_spdif->clk);
clk_err:
	kfree(mxs_spdif);
	return ret;
}

static int __devexit mxs_spdif_remove(struct platform_device *pdev)
{
	struct mxs_codec_priv *mxs_spdif = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = &mxs_spdif->codec;

	snd_soc_unregister_codec(codec);

	clk_disable(mxs_spdif->clk);
	clk_put(mxs_spdif->clk);

	kfree(mxs_spdif);

	return 0;
}

struct platform_driver mxs_spdif_driver = {
	.driver = {
		   .name = "mxs-spdif",
		   },
	.probe = mxs_spdif_probe,
	.remove = __devexit_p(mxs_spdif_remove),
};

static int __init mxs_spdif_init(void)
{
	return platform_driver_register(&mxs_spdif_driver);
}

static void __exit mxs_spdif_exit(void)
{
	return platform_driver_unregister(&mxs_spdif_driver);
}

module_init(mxs_spdif_init);
module_exit(mxs_spdif_exit);

MODULE_DESCRIPTION("MXS SPDIF transmitter");
MODULE_AUTHOR("Vladimir Barinov");
MODULE_LICENSE("GPL");
