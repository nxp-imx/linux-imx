/*
 * ALSA SoC SPDIF Audio Layer for STMP3xxx processor familiy
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/dma.h>
#include <mach/regs-spdif.h>
#include "stmp3xxx_pcm.h"
#include <mach/platform.h>

#define STMP3XXX_SPDIF_RATES	(SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_64000 | \
				 SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)
#define STMP3XXX_SPDIF_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S32_LE)

static struct stmp3xxx_pcm_dma_params stmp3xxx_spdif = {
	.name = "stmp3xxx spdif",
	.dma_bus = STMP3XXX_BUS_APBX,
	.dma_ch	= 2,
	.irq = IRQ_SPDIF_DMA,
};

static irqreturn_t stmp3xxx_err_irq(int irq, void *dev_id)
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
			stmp3xxx_clearl(BM_SPDIF_CTRL_FIFO_UNDERFLOW_IRQ,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL);
	} else if (ctrl_reg & overflow_mask) {
		printk(KERN_DEBUG "overflow detected SPDIF\n");

		if (playback)
			stmp3xxx_clearl(BM_SPDIF_CTRL_FIFO_OVERFLOW_IRQ,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL);
	} else
		printk(KERN_WARNING "Unknown SPDIF error interrupt\n");

	return IRQ_HANDLED;
}

static int stmp3xxx_spdif_trigger(struct snd_pcm_substream *substream, int cmd,
				  struct snd_soc_dai *dai)
{
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (playback)
			stmp3xxx_setl(BM_SPDIF_CTRL_RUN,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (playback)
			stmp3xxx_clearl(BM_SPDIF_CTRL_RUN,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL);
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

static int stmp3xxx_spdif_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int irq;
	int ret;

	if (playback) {
		irq = IRQ_SPDIF_ERROR;
		cpu_dai->dma_data = &stmp3xxx_spdif;
	}

	ret = request_irq(irq, stmp3xxx_err_irq, 0, "STMP3xxx SPDIF Error",
			  substream);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request SPDIF error irq %d\n",
		       __func__, IRQ_SPDIF_ERROR);
		return ret;
	}

	/* Enable error interrupt */
	if (playback) {
		stmp3xxx_clearl(BM_SPDIF_CTRL_FIFO_OVERFLOW_IRQ,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL);
		stmp3xxx_clearl(BM_SPDIF_CTRL_FIFO_UNDERFLOW_IRQ,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL);
		stmp3xxx_setl(BM_SPDIF_CTRL_FIFO_ERROR_IRQ_EN,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL);
	}

	return 0;
}

static void stmp3xxx_spdif_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;

	/* Disable error interrupt */
	if (playback) {
		stmp3xxx_clearl(BM_SPDIF_CTRL_FIFO_ERROR_IRQ_EN,
				REGS_SPDIF_BASE + HW_SPDIF_CTRL);
		free_irq(IRQ_SPDIF_ERROR, substream);
	}
}

#ifdef CONFIG_PM
static int stmp3xxx_spdif_suspend(struct snd_soc_dai *cpu_dai)
{
	return 0;
}

static int stmp3xxx_spdif_resume(struct snd_soc_dai *cpu_dai)
{
	return 0;
}
#else
#define stmp3xxx_spdif_suspend	NULL
#define stmp3xxx_spdif_resume	NULL
#endif /* CONFIG_PM */

struct snd_soc_dai_ops stmp3xxx_spdif_dai_ops = {
	.startup = stmp3xxx_spdif_startup,
	.shutdown = stmp3xxx_spdif_shutdown,
	.trigger = stmp3xxx_spdif_trigger,
};

struct snd_soc_dai stmp3xxx_spdif_dai = {
	.name = "stmp3xxx spdif",
	.id = 0,
	.suspend = stmp3xxx_spdif_suspend,
	.resume = stmp3xxx_spdif_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = STMP3XXX_SPDIF_RATES,
		.formats = STMP3XXX_SPDIF_FORMATS,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = STMP3XXX_SPDIF_RATES,
		.formats = STMP3XXX_SPDIF_FORMATS,
	},
	.ops = &stmp3xxx_spdif_dai_ops,
};
EXPORT_SYMBOL_GPL(stmp3xxx_spdif_dai);

static int __init stmp3xxx_spdif_dai_init(void)
{
	return snd_soc_register_dai(&stmp3xxx_spdif_dai);
}

static void __exit stmp3xxx_spdif_dai_exit(void)
{
	snd_soc_unregister_dai(&stmp3xxx_spdif_dai);
}
module_init(stmp3xxx_spdif_dai_init);
module_exit(stmp3xxx_spdif_dai_exit);

MODULE_AUTHOR("Vladimir Barinov");
MODULE_DESCRIPTION("stmp3xxx SPDIF DAI");
MODULE_LICENSE("GPL");
