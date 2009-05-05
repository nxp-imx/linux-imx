/*
 * ASoC Audio Layer for Freescale STMP37XX/STMP378X ADC/DAC
 *
 * Author: Vladislav Buzov <vbuzov@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <mach/regs-audioin.h>
#include <mach/regs-audioout.h>
#include "stmp3xxx_pcm.h"

#define STMP3XXX_ADC_RATES	SNDRV_PCM_RATE_8000_192000
#define STMP3XXX_ADC_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S32_LE)

struct stmp3xxx_pcm_dma_params stmp3xxx_audio_in = {
	.name = "stmp3xxx adc",
	.dma_bus = STMP3XXX_BUS_APBX,
	.dma_ch	= 0,
	.irq = IRQ_ADC_DMA,
};

struct stmp3xxx_pcm_dma_params stmp3xxx_audio_out = {
	.name = "stmp3xxx dac",
	.dma_bus = STMP3XXX_BUS_APBX,
	.dma_ch	= 1,
	.irq = IRQ_DAC_DMA,
};

static irqreturn_t stmp3xxx_err_irq(int irq, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	u32 ctrl_reg;
	u32 overflow_mask;
	u32 underflow_mask;

	if (playback) {
		ctrl_reg = HW_AUDIOOUT_CTRL_RD();
		underflow_mask = BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ;
		overflow_mask = BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ;
	} else {
		ctrl_reg = HW_AUDIOIN_CTRL_RD();
		underflow_mask = BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ;
		overflow_mask = BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ;
	}

	if (ctrl_reg & underflow_mask) {
		printk(KERN_DEBUG "%s underflow detected\n",
		       playback ? "DAC" : "ADC");

		if (playback)
			HW_AUDIOOUT_CTRL_CLR(
				BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ);
		else
			HW_AUDIOIN_CTRL_CLR(
				BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ);

	} else if (ctrl_reg & overflow_mask) {
		printk(KERN_DEBUG "%s overflow detected\n",
		       playback ? "DAC" : "ADC");

		if (playback)
			HW_AUDIOOUT_CTRL_CLR(
				BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ);
		else
			HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ);
	} else
		printk(KERN_WARNING "Unknown DAC error interrupt\n");

	return IRQ_HANDLED;
}

static int stmp3xxx_adc_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (playback)
			HW_AUDIOOUT_CTRL_SET(BM_AUDIOOUT_CTRL_RUN);
		else
			HW_AUDIOIN_CTRL_SET(BM_AUDIOIN_CTRL_RUN);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		if (playback)
			HW_AUDIOOUT_CTRL_CLR(BM_AUDIOOUT_CTRL_RUN);
		else
			HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_RUN);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int stmp3xxx_adc_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int irq;
	int ret;

	if (playback) {
		irq = IRQ_DAC_ERROR;
		cpu_dai->dma_data = &stmp3xxx_audio_out;
	} else {
		irq = IRQ_ADC_ERROR;
		cpu_dai->dma_data = &stmp3xxx_audio_in;
	}

	ret = request_irq(irq, stmp3xxx_err_irq, 0, "STMP3xxx DAC/ADC Error",
			  substream);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request ADC/DAC error irq %d\n",
		       __func__, IRQ_DAC_ERROR);
		return ret;
	}

	/* Enable error interrupt */
	if (playback) {
		HW_AUDIOOUT_CTRL_CLR(BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ);
		HW_AUDIOOUT_CTRL_CLR(BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ);
		HW_AUDIOOUT_CTRL_SET(BM_AUDIOOUT_CTRL_FIFO_ERROR_IRQ_EN);
	} else {
		HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ);
		HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ);
		HW_AUDIOIN_CTRL_SET(BM_AUDIOIN_CTRL_FIFO_ERROR_IRQ_EN);
	}

	return 0;
}

static void stmp3xxx_adc_shutdown(struct snd_pcm_substream *substream)
{
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;

	/* Disable error interrupt */
	if (playback) {
		HW_AUDIOOUT_CTRL_CLR(BM_AUDIOOUT_CTRL_FIFO_ERROR_IRQ_EN);
		free_irq(IRQ_DAC_ERROR, substream);
	} else {
		HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_FIFO_ERROR_IRQ_EN);
		free_irq(IRQ_ADC_ERROR, substream);
	}
}

#ifdef CONFIG_PM
static int stmp3xxx_adc_suspend(struct platform_device *pdev,
				struct snd_soc_dai *cpu_dai)
{
	return 0;
}

static int stmp3xxx_adc_resume(struct platform_device *pdev,
				struct snd_soc_dai *cpu_dai)
{
	return 0;
}
#else
#define stmp3xxx_adc_suspend	NULL
#define stmp3xxx_adc_resume	NULL
#endif /* CONFIG_PM */

struct snd_soc_dai stmp3xxx_adc_dai = {
	.name = "stmp3xxx adc/dac",
	.id = 0,
	.type = SND_SOC_DAI_PCM,
	.suspend = stmp3xxx_adc_suspend,
	.resume = stmp3xxx_adc_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = STMP3XXX_ADC_RATES,
		.formats = STMP3XXX_ADC_FORMATS,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = STMP3XXX_ADC_RATES,
		.formats = STMP3XXX_ADC_FORMATS,
	},
	.ops = {
		.startup = stmp3xxx_adc_startup,
		.shutdown = stmp3xxx_adc_shutdown,
		.trigger = stmp3xxx_adc_trigger,
	},
};
EXPORT_SYMBOL_GPL(stmp3xxx_adc_dai);

MODULE_AUTHOR("Vladislav Buzov");
MODULE_DESCRIPTION("stmp3xxx dac/adc DAI");
MODULE_LICENSE("GPL");
