/*
 * ALSA SoC SPDIF Audio Layer for MXS
 *
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc.
 *
 * Based on stmp3xxx_spdif_dai.c
 * Vladimir Barinov <vbarinov@embeddedalley.com>
 * Copyright 2008 SigmaTel, Inc
 * Copyright 2008 Embedded Alley Solutions, Inc
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program  is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <mach/dma.h>
#include <mach/hardware.h>

#include "../codecs/mxs_spdif.h"
#include "mxs-pcm.h"

#define REGS_SPDIF_BASE IO_ADDRESS(SPDIF_PHYS_ADDR)

struct mxs_pcm_dma_params mxs_spdif = {
	.name = "mxs spdif",
	.dma_ch	= MXS_DMA_CHANNEL_AHB_APBX_SPDIF,
	.irq = IRQ_SPDIF_DMA,
};

static irqreturn_t mxs_err_irq(int irq, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	u32 ctrl_reg = 0;
	u32 overflow_mask;
	u32 underflow_mask;

	if (playback) {
		ctrl_reg = __raw_readl(REGS_SPDIF_BASE + HW_SPDIF_CTRL);
		underflow_mask = BM_SPDIF_CTRL_FIFO_UNDERFLOW_IRQ;
		overflow_mask = BM_SPDIF_CTRL_FIFO_OVERFLOW_IRQ;
	}

	if (ctrl_reg & underflow_mask) {
		printk(KERN_DEBUG "underflow detected SPDIF\n");

		if (playback)
			__raw_writel(BM_SPDIF_CTRL_FIFO_UNDERFLOW_IRQ,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL_CLR);
	} else if (ctrl_reg & overflow_mask) {
		printk(KERN_DEBUG "overflow detected SPDIF\n");

		if (playback)
			__raw_writel(BM_SPDIF_CTRL_FIFO_OVERFLOW_IRQ,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL_CLR);
	} else
		printk(KERN_WARNING "Unknown SPDIF error interrupt\n");

	return IRQ_HANDLED;
}

static int mxs_spdif_trigger(struct snd_pcm_substream *substream, int cmd,
				  struct snd_soc_dai *dai)
{
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (playback)
			__raw_writel(BM_SPDIF_CTRL_RUN,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL_SET);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (playback)
			__raw_writel(BM_SPDIF_CTRL_RUN,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL_CLR);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int mxs_spdif_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int irq;
	int ret;

	if (playback) {
		irq = IRQ_SPDIF_ERROR;
		snd_soc_dai_set_dma_data(cpu_dai, substream, &mxs_spdif);
	}

	ret = request_irq(irq, mxs_err_irq, 0, "Mxs SPDIF Error",
			  substream);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request SPDIF error irq %d\n",
		       __func__, IRQ_SPDIF_ERROR);
		return ret;
	}

	/* Enable error interrupt */
	if (playback) {
		__raw_writel(BM_SPDIF_CTRL_FIFO_OVERFLOW_IRQ,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL_CLR);
		__raw_writel(BM_SPDIF_CTRL_FIFO_UNDERFLOW_IRQ,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL_CLR);
		__raw_writel(BM_SPDIF_CTRL_FIFO_ERROR_IRQ_EN,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL_SET);
	}

	return 0;
}

static void mxs_spdif_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;

	/* Disable error interrupt */
	if (playback) {
		__raw_writel(BM_SPDIF_CTRL_FIFO_ERROR_IRQ_EN,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL_CLR);
		free_irq(IRQ_SPDIF_ERROR, substream);
	}
}

#ifdef CONFIG_PM
static int mxs_spdif_suspend(struct snd_soc_dai *cpu_dai)
{
	return 0;
}

static int mxs_spdif_resume(struct snd_soc_dai *cpu_dai)
{
	return 0;
}
#else
#define mxs_spdif_suspend	NULL
#define mxs_spdif_resume	NULL
#endif /* CONFIG_PM */

struct snd_soc_dai_ops mxs_spdif_dai_ops = {
	.startup = mxs_spdif_startup,
	.shutdown = mxs_spdif_shutdown,
	.trigger = mxs_spdif_trigger,
};

struct snd_soc_dai mxs_spdif_dai = {
	.name = "mxs-spdif",
	.id = 0,
	.suspend = mxs_spdif_suspend,
	.resume = mxs_spdif_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXS_SPDIF_RATES,
		.formats = MXS_SPDIF_FORMATS,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXS_SPDIF_RATES,
		.formats = MXS_SPDIF_FORMATS,
	},
	.ops = &mxs_spdif_dai_ops,
};
EXPORT_SYMBOL_GPL(mxs_spdif_dai);

static int __init mxs_spdif_dai_init(void)
{
	return snd_soc_register_dai(&mxs_spdif_dai);
}

static void __exit mxs_spdif_dai_exit(void)
{
	snd_soc_unregister_dai(&mxs_spdif_dai);
}
module_init(mxs_spdif_dai_init);
module_exit(mxs_spdif_dai_exit);

MODULE_AUTHOR("Vladimir Barinov");
MODULE_DESCRIPTION("MXS SPDIF DAI");
MODULE_LICENSE("GPL");
