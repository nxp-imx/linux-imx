/*
 * imx-ac97.c  --  AC97 driver for Freescale IMX
 *
 *
 * Copyright 2009-2010 Freescale Semiconductor, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    26th Nov. 2009   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <mach/clock.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <linux/delay.h>

#include "imx-ssi.h"
#include "imx-pcm.h"

static DEFINE_MUTEX(ac97_mutex);
static DECLARE_COMPLETION(ac97_completion);

struct imx_ac97_runtime {
	u32 id;
	u32 playback;
	u32 capture;
};

static struct imx_ac97_runtime imx_ac97_para;
static struct imx_ssi imx_ac97_data[2];

#define IMX_SSI_DUMP 0
#if IMX_SSI_DUMP
#define SSI_DUMP() \
	do { \
		printk(KERN_INFO "dump @ %s\n", __func__); \
		printk(KERN_INFO "scr %x\t, %x\n", \
			__raw_readl(SSI1_SCR), __raw_readl(SSI2_SCR));   \
		printk(KERN_INFO "sisr %x\t, %x\n", \
			__raw_readl(SSI1_SISR), __raw_readl(SSI2_SISR)); \
		printk(KERN_INFO "stcr %x\t, %x\n", \
			__raw_readl(SSI1_STCR), __raw_readl(SSI2_STCR)); \
		printk(KERN_INFO "srcr %x\t, %x\n", \
			__raw_readl(SSI1_SRCR), __raw_readl(SSI2_SRCR)); \
		printk(KERN_INFO "stccr %x\t, %x\n", \
			__raw_readl(SSI1_STCCR), __raw_readl(SSI2_STCCR)); \
		printk(KERN_INFO "srccr %x\t, %x\n", \
			__raw_readl(SSI1_SRCCR), __raw_readl(SSI2_SRCCR)); \
		printk(KERN_INFO "sfcsr %x\t, %x\n", \
			__raw_readl(SSI1_SFCSR), __raw_readl(SSI2_SFCSR)); \
		printk(KERN_INFO "stmsk %x\t, %x\n", \
			__raw_readl(SSI1_STMSK), __raw_readl(SSI2_STMSK)); \
		printk(KERN_INFO "srmsk %x\t, %x\n", \
			__raw_readl(SSI1_SRMSK), __raw_readl(SSI2_SRMSK)); \
		printk(KERN_INFO "sier %x\t, %x\n", \
			__raw_readl(SSI1_SIER), __raw_readl(SSI2_SIER)); \
		printk(KERN_INFO "sacnt %x\t, %x\n", \
			__raw_readl(SSI1_SACNT), __raw_readl(SSI2_SACNT)); \
		printk(KERN_INFO "sacdd %x\t, %x\n", \
			__raw_readl(SSI1_SACADD), __raw_readl(SSI2_SACADD)); \
		printk(KERN_INFO "sacdat %x\t, %x\n", \
			__raw_readl(SSI1_SACDAT), __raw_readl(SSI2_SACDAT)); \
		printk(KERN_INFO "satag %x\t, %x\n", \
			__raw_readl(SSI1_SATAG), __raw_readl(SSI2_SATAG)); \
		printk(KERN_INFO "saccst %x\t, %x\n", \
			__raw_readl(SSI1_SACCST), __raw_readl(SSI2_SACCST)); \
		printk(KERN_INFO "saccen %x\t, %x\n", \
			__raw_readl(SSI1_SACCEN), __raw_readl(SSI2_SACCEN)); \
		printk(KERN_INFO "saccdis %x\t, %x\n", \
			__raw_readl(SSI1_SACCDIS), __raw_readl(SSI2_SACCDIS)); \
	}  while (0);
#else
#define SSI_DUMP()
#endif

/*
 * Read register value from codec, the read command is sent from
 * AC97 slot 2 - 3. The register value is sent back at slot 2 -3
 * in next RX frame.
 */
static unsigned short imx_ac97_read(struct snd_ac97 *ac97, unsigned short reg)
{
	u32 sacdat, sier, scr;
	mutex_lock(&ac97_mutex);
	if (imx_ac97_para.id == IMX_DAI_AC97_1) {
		sier = __raw_readl(SSI1_SIER);
		scr = __raw_readl(SSI1_SCR);
		__raw_writel((reg << 12), SSI1_SACADD);
		__raw_writel(sier | SSI_SIER_CMDDU_EN, SSI1_SIER);
		__raw_writel(SSI_SACNT_RD | __raw_readl(SSI1_SACNT),
			     SSI1_SACNT);
		__raw_writel(scr | SSI_SCR_TE | SSI_SCR_RE, SSI1_SCR);
		wait_for_completion_timeout(&ac97_completion, HZ);
		sacdat = __raw_readl(SSI1_SACDAT);
		__raw_writel(sier, SSI1_SIER);
		__raw_writel(scr, SSI1_SCR);
	} else {
		sier = __raw_readl(SSI2_SIER);
		scr = __raw_readl(SSI2_SCR);
		__raw_writel((reg << 12), SSI2_SACADD);
		__raw_writel(sier | SSI_SIER_CMDDU_EN, SSI2_SIER);
		__raw_writel(SSI_SACNT_RD | __raw_readl(SSI2_SACNT),
			     SSI2_SACNT);
		__raw_writel(scr | SSI_SCR_TE | SSI_SCR_RE, SSI2_SCR);
		wait_for_completion_timeout(&ac97_completion, HZ);
		sacdat = __raw_readl(SSI2_SACDAT);
		__raw_writel(sier, SSI2_SIER);
		__raw_writel(scr, SSI2_SCR);
	}
	mutex_unlock(&ac97_mutex);
	return (unsigned short)(sacdat >> 4);

}

/*
 * This fucntion is used to send command to codec.
 */
static void imx_ac97_write(struct snd_ac97 *ac97, unsigned short reg,
			   unsigned short val)
{
	u32 scr;
	mutex_lock(&ac97_mutex);
	if (imx_ac97_para.id == IMX_DAI_AC97_1) {
		scr = __raw_readl(SSI1_SCR);
		__raw_writel(reg << 12, SSI1_SACADD);
		__raw_writel(val << 4, SSI1_SACDAT);
		__raw_writel(SSI_SACNT_WR | __raw_readl(SSI1_SACNT),
			     SSI1_SACNT);
		__raw_writel(scr | SSI_SCR_TE | SSI_SCR_RE, SSI1_SCR);
		udelay(100);
		__raw_writel(scr, SSI1_SCR);
	} else {
		scr = __raw_readl(SSI2_SCR);
		__raw_writel(reg << 12, SSI2_SACADD);
		__raw_writel(val << 4, SSI2_SACDAT);
		__raw_writel(SSI_SACNT_WR | __raw_readl(SSI2_SACNT),
			     SSI2_SACNT);
		__raw_writel(scr | SSI_SCR_TE | SSI_SCR_RE, SSI2_SCR);
		udelay(100);
		__raw_writel(scr, SSI1_SCR);
	}
	mutex_unlock(&ac97_mutex);

}

struct snd_ac97_bus_ops soc_ac97_ops = {
	.read = imx_ac97_read,
	.write = imx_ac97_write,
};
EXPORT_SYMBOL_GPL(soc_ac97_ops);

static struct clk *ssi1_clk;
static struct clk *ssi2_clk;

static int imx_ac97_hw_tx_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *cpu_dai)
{
	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;
	u32 channel = params_channels(params);
	u32 stccr, sier;

	if (cpu_dai->id == IMX_DAI_AC97_1) {
		stccr =
		    __raw_readl(SSI1_STCCR) & ~(SSI_STCCR_WL_MASK |
						SSI_STCCR_DC_MASK);
		sier = __raw_readl(SSI1_SIER);
	} else {
		stccr =
		    __raw_readl(SSI2_STCCR) & ~(SSI_STCCR_WL_MASK |
						SSI_STCCR_DC_MASK);
		sier = __raw_readl(SSI2_SIER);
	}

	/* DAI data (word) size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		stccr |= SSI_STCCR_WL(16) | SSI_STCCR_DC(0x0C);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		stccr |= SSI_STCCR_WL(20) | SSI_STCCR_DC(0x0C);
		break;
	}
	if (ssi_mode->ac97_rx_slots)
		__raw_writel(ssi_mode->ac97_rx_slots, SSI1_SACCEN);
	else
		__raw_writel(((1 << channel) - 1) << (10 - channel),
			     SSI1_SACCEN);

	sier |= SSI_SIER_TDMAE;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		__raw_writel(stccr, SSI1_STCCR);
		__raw_writel(sier, SSI1_SIER);
	} else {
		__raw_writel(stccr, SSI2_STCCR);
		__raw_writel(sier, SSI2_SIER);
	}

	return 0;
}

static int imx_ac97_hw_rx_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *cpu_dai)
{
	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;
	u32 channel = params_channels(params);
	u32 srccr, sier;

	if (cpu_dai->id == IMX_DAI_AC97_1) {
		srccr = __raw_readl(SSI1_SRCCR);
		sier = __raw_readl(SSI1_SIER);
	} else {
		srccr = __raw_readl(SSI2_SRCCR);
		sier = __raw_readl(SSI2_SIER);
	}
	srccr &= ~(SSI_SRCCR_WL_MASK | SSI_SRCCR_DC_MASK);

	/* DAI data (word) size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		srccr |= SSI_SRCCR_WL(16) | SSI_SRCCR_DC(0x0C);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		srccr |= SSI_SRCCR_WL(20) | SSI_SRCCR_DC(0x0C);
		break;
	}

	if (ssi_mode->ac97_tx_slots)
		__raw_writel(ssi_mode->ac97_tx_slots, SSI1_SACCEN);
	else
		__raw_writel(((1 << channel) - 1) << (10 - channel),
			     SSI1_SACCEN);

	/* enable interrupts */
	sier |= SSI_SIER_RDMAE;

	if (cpu_dai->id == IMX_DAI_AC97_1) {
		__raw_writel(srccr, SSI1_SRCCR);
		__raw_writel(sier, SSI1_SIER);
	} else {
		__raw_writel(srccr, SSI2_SRCCR);
		__raw_writel(sier, SSI2_SIER);
	}
	return 0;
}

static int imx_ac97_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *cpu_dai)
{
	u32 rate = params_rate(params);
	u32 sacnt;

	if (cpu_dai->id == IMX_DAI_AC97_1)
		sacnt = __raw_readl(SSI1_SACNT);
	else
		sacnt = __raw_readl(SSI2_SACNT);

	switch (rate) {
	case 8000:
		sacnt |= SSI_SACNT_FV | SSI_SACNT_FRDIV(0x05);
		break;
	case 11050:
		sacnt |= SSI_SACNT_FV | SSI_SACNT_FRDIV(0x03);
		break;
	case 16000:
		sacnt |= SSI_SACNT_FV | SSI_SACNT_FRDIV(0x02);
		break;
	case 22050:
		sacnt |= SSI_SACNT_FV | SSI_SACNT_FRDIV(0x01);
		break;
	case 44100:
	case 48000:
		sacnt |= SSI_SACNT_FV | SSI_SACNT_FRDIV(0x00);
		break;
	}

	if (cpu_dai->id == IMX_DAI_AC97_1)
		__raw_writel(sacnt, SSI1_SACNT);
	else
		__raw_writel(sacnt, SSI2_SACNT);

	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return imx_ac97_hw_tx_params(substream, params, cpu_dai);
	else
		return imx_ac97_hw_rx_params(substream, params, cpu_dai);
}

static int imx_ac97_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	u32 scr, i, reg;

	if (cpu_dai->id == IMX_DAI_AC97_1)
		scr = __raw_readl(SSI1_SCR);
	else
		scr = __raw_readl(SSI2_SCR);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			for (i = 0; i < 12; i++)
				__raw_writel(0x0, SSI1_STX0);
		} else {
			for (i = 0; i < 12; i++)
				reg = __raw_readl(SSI1_SRX0);
		}
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			imx_ac97_para.playback = 1;
		else
			imx_ac97_para.capture = 1;
		scr |= SSI_SCR_TE | SSI_SCR_RE;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			imx_ac97_para.playback = 0;
		else
			imx_ac97_para.capture = 0;

		if ((imx_ac97_para.playback == 0)
		    && (imx_ac97_para.capture == 0))
			scr &= ~(SSI_SCR_TE | SSI_SCR_RE);
		break;
	default:
		return -EINVAL;
	}
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		__raw_writel(scr, SSI1_SCR);
	else
		__raw_writel(scr, SSI2_SCR);
	SSI_DUMP();
	return 0;
}

#ifdef CONFIG_PM
static int imx_ac97_suspend(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;

	/* do we need to disable any clocks? */

	return 0;
}

static int imx_ac97_resume(struct snd_soc_dai *dai)
{
	if (!dai->active)
		return 0;

	/* do we need to enable any clocks? */

	return 0;
}
#else
#define imx_ac97_suspend	NULL
#define imx_ac97_resume	NULL
#endif

static irqreturn_t ssi1_irq(int irq, void *dev_id)
{
	u32 sier, sisr, reg;
	sier = __raw_readl(SSI1_SIER);
	sisr = __raw_readl(SSI1_SISR);
	if ((sier & SSI_SIER_CMDDU_EN) && (sisr & SSI_SISR_CMDDU)) {
		reg = __raw_readl(SSI1_SACADD);
		reg = __raw_readl(SSI1_SACDAT);
		complete(&ac97_completion);
	}
	return IRQ_HANDLED;
}

static irqreturn_t ssi2_irq(int irq, void *dev_id)
{
	u32 sier, sisr, reg;
	sier = __raw_readl(SSI1_SIER);
	sisr = __raw_readl(SSI1_SISR);
	if ((sier & SSI_SIER_CMDDU_EN) && (sisr & SSI_SISR_CMDDU)) {
		reg = __raw_readl(SSI2_SACADD);
		reg = __raw_readl(SSI2_SACDAT);
		complete(&ac97_completion);
	}
	return IRQ_HANDLED;
}

static int imx_ac97_probe(struct platform_device *pdev, struct snd_soc_dai *dai)
{

	u32 stccr;
	stccr = SSI_STCCR_WL(16) | SSI_STCCR_DC(0x0C);

	if (!strcmp(dai->name, "imx-ac97-1")) {
		dai->id = IMX_DAI_AC97_1;
		ssi1_clk = clk_get(NULL, "ssi_clk.0");
		clk_enable(ssi1_clk);

		__raw_writel(SSI_SCR_SSIEN, SSI1_SCR);

		__raw_writel((SSI_SFCSR_RFWM1(SSI_RXFIFO_WATERMARK) |
			      SSI_SFCSR_RFWM0(SSI_RXFIFO_WATERMARK) |
			      SSI_SFCSR_TFWM1(SSI_TXFIFO_WATERMARK) |
			      SSI_SFCSR_TFWM0(SSI_TXFIFO_WATERMARK)),
			     SSI1_SFCSR);

		__raw_writel(stccr, SSI1_STCCR);
		__raw_writel(stccr, SSI1_SRCCR);
		__raw_writel(SSI_SACNT_AC97EN, SSI1_SACNT);
	} else if (!strcmp(dai->name, "imx-ac97-2")) {
		dai->id = IMX_DAI_AC97_2;
		ssi2_clk = clk_get(NULL, "ssi_clk.1");
		clk_enable(ssi2_clk);

		__raw_writel(SSI_SCR_SSIEN, SSI2_SCR);

		__raw_writel((SSI_SFCSR_RFWM1(SSI_RXFIFO_WATERMARK) |
			      SSI_SFCSR_RFWM0(SSI_RXFIFO_WATERMARK) |
			      SSI_SFCSR_TFWM1(SSI_TXFIFO_WATERMARK) |
			      SSI_SFCSR_TFWM0(SSI_TXFIFO_WATERMARK)),
			     SSI2_SFCSR);

		__raw_writel(stccr, SSI2_STCCR);
		__raw_writel(stccr, SSI2_SRCCR);

		__raw_writel(SSI_SACNT_AC97EN, SSI2_SACNT);
	} else {
		printk(KERN_ERR "%s: invalid device %s\n", __func__, dai->name);
		return -ENODEV;
	}

	if (!strcmp(dai->name, "imx-ac97-1")) {
		if (request_irq(MXC_INT_SSI1, ssi1_irq, 0, "ssi1", dai)) {
			printk(KERN_ERR
			       "%s: failure requesting irq %s\n",
			       __func__, "ssi1");
			return -EBUSY;
		}
	}

	if (!strcmp(dai->name, "imx-ac97-2")) {
		if (request_irq(MXC_INT_SSI2, ssi2_irq, 0, "ssi2", dai)) {
			printk(KERN_ERR
			       "%s: failure requesting irq %s\n",
			       __func__, "ssi2");
			return -EBUSY;
		}
	}

	return 0;
}

static void imx_ac97_remove(struct platform_device *pdev,
			    struct snd_soc_dai *dai)
{
	if (!strcmp(dai->name, "imx-ac97-1"))
		free_irq(MXC_INT_SSI1, dai);

	if (!strcmp(dai->name, "imx-ac97-2"))
		free_irq(MXC_INT_SSI2, dai);
}

#define IMX_AC97_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)

#define IMX_AC97_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE)

static struct snd_soc_dai_ops imx_ac97_dai_ops = {
	.hw_params = imx_ac97_hw_params,
	.trigger = imx_ac97_trigger,
};

struct snd_soc_dai imx_ac97_dai[] = {
	{
	 .name = "imx-ac97-1",
	 .id = 0,
	 .ac97_control = 1,
	 .probe = imx_ac97_probe,
	 .remove = imx_ac97_remove,
	 .suspend = imx_ac97_suspend,
	 .resume = imx_ac97_resume,
	 .playback = {
		      .stream_name = "AC97 Playback",
		      .channels_min = 1,
		      .channels_max = 10,
		      .rates = IMX_AC97_RATES,
		      .formats = IMX_AC97_FORMATS,
		      },
	 .capture = {
		     .stream_name = "AC97 Capture",
		     .channels_min = 1,
		     .channels_max = 6,
		     .rates = IMX_AC97_RATES,
		     .formats = IMX_AC97_FORMATS,
		     },
	 .ops = &imx_ac97_dai_ops,
	 .private_data = &imx_ac97_data[IMX_DAI_AC97_1],
	 },
	{
	 .name = "imx-ac97-2",
	 .id = 1,
	 .ac97_control = 1,
	 .probe = imx_ac97_probe,
	 .remove = imx_ac97_remove,
	 .suspend = imx_ac97_suspend,
	 .resume = imx_ac97_resume,
	 .playback = {
		      .stream_name = "AC97 Playback",
		      .channels_min = 1,
		      .channels_max = 10,
		      .rates = IMX_AC97_RATES,
		      .formats = IMX_AC97_FORMATS,
		      },
	 .capture = {
		     .stream_name = "AC97 Capture",
		     .channels_min = 1,
		     .channels_max = 6,
		     .rates = IMX_AC97_RATES,
		     .formats = IMX_AC97_FORMATS,
		     },
	 .ops = &imx_ac97_dai_ops,
	 .private_data = &imx_ac97_data[IMX_DAI_AC97_2],
	 },
};
EXPORT_SYMBOL_GPL(imx_ac97_dai);

static int __init imx_ac97_init(void)
{
	return snd_soc_register_dais(imx_ac97_dai, ARRAY_SIZE(imx_ac97_dai));
}

static void __exit imx_ac97_exit(void)
{
	snd_soc_unregister_dais(imx_ac97_dai, ARRAY_SIZE(imx_ac97_dai));
}

module_init(imx_ac97_init);
module_exit(imx_ac97_exit);

MODULE_DESCRIPTION("i.MX ASoC AC97 driver");
MODULE_LICENSE("GPL");
