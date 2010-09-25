/*
 * ASoC Audio Layer for Freescale MXS ADC/DAC
 *
 * Author: Vladislav Buzov <vbuzov@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <mach/dma.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/regs-audioin.h>
#include <mach/regs-audioout.h>
#include <mach/dmaengine.h>

#include "mxs-pcm.h"

#define MXS_ADC_RATES	SNDRV_PCM_RATE_8000_192000
#define MXS_ADC_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S32_LE)
#define ADC_VOLUME_MIN  0x37

struct mxs_pcm_dma_params mxs_audio_in = {
	.name = "mxs-audio-in",
	.dma_ch	= MXS_DMA_CHANNEL_AHB_APBX_AUDIOADC,
	.irq = IRQ_ADC_DMA,
};

struct mxs_pcm_dma_params mxs_audio_out = {
	.name = "mxs-audio-out",
	.dma_ch	= MXS_DMA_CHANNEL_AHB_APBX_AUDIODAC,
	.irq = IRQ_DAC_DMA,
};

static struct delayed_work work;
static struct delayed_work adc_ramp_work;
static struct delayed_work dac_ramp_work;
static bool adc_ramp_done = 1;
static bool dac_ramp_done = 1;

static void mxs_adc_schedule_work(struct delayed_work *work)
{
	schedule_delayed_work(work, HZ / 10);
}
static void mxs_adc_work(struct work_struct *work)
{
	/* disable irq */
	disable_irq(IRQ_HEADPHONE_SHORT);

	while (true) {
		__raw_writel(BM_AUDIOOUT_PWRDN_HEADPHONE,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_PWRDN_CLR);
		msleep(10);
		if ((__raw_readl(REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL)
			& BM_AUDIOOUT_ANACTRL_SHORT_LR_STS) != 0) {
			/* rearm the short protection */
			__raw_writel(BM_AUDIOOUT_ANACTRL_SHORTMODE_LR,
				REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_CLR);
			__raw_writel(BM_AUDIOOUT_ANACTRL_SHORT_LR_STS,
				REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_CLR);
			__raw_writel(BF_AUDIOOUT_ANACTRL_SHORTMODE_LR(0x1),
				REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_SET);

			__raw_writel(BM_AUDIOOUT_PWRDN_HEADPHONE,
				REGS_AUDIOOUT_BASE + HW_AUDIOOUT_PWRDN_SET);
			printk(KERN_WARNING "WARNING : Headphone LR short!\r\n");
		} else {
			printk(KERN_WARNING "INFO : Headphone LR no longer short!\r\n");
			break;
		}
		msleep(1000);
	}

	/* power up the HEADPHONE and un-mute the HPVOL */
	__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
	      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_HPVOL_CLR);
	__raw_writel(BM_AUDIOOUT_PWRDN_HEADPHONE,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_PWRDN_CLR);

	/* enable irq for next short detect*/
	enable_irq(IRQ_HEADPHONE_SHORT);
}

static void mxs_adc_schedule_ramp_work(struct delayed_work *work)
{
	schedule_delayed_work(work, msecs_to_jiffies(2));
	adc_ramp_done = 0;
}

static void mxs_adc_ramp_work(struct work_struct *work)
{
	u32 reg = 0;
	u32 reg1 = 0;
	u32 reg2 = 0;
	u32 l, r;
	u32 ll, rr;
	int i;

	reg = __raw_readl(REGS_AUDIOIN_BASE + \
		HW_AUDIOIN_ADCVOLUME);

	reg1 = reg & ~BM_AUDIOIN_ADCVOLUME_VOLUME_LEFT;
	reg1 = reg1 & ~BM_AUDIOIN_ADCVOLUME_VOLUME_RIGHT;
	/* minimize adc volume */
	reg2 = reg1 |
	    BF_AUDIOIN_ADCVOLUME_VOLUME_LEFT(ADC_VOLUME_MIN) |
	    BF_AUDIOIN_ADCVOLUME_VOLUME_RIGHT(ADC_VOLUME_MIN);
	__raw_writel(reg2,
		REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOLUME);
	msleep(1);

	l = (reg & BM_AUDIOIN_ADCVOLUME_VOLUME_LEFT) >>
		BP_AUDIOIN_ADCVOLUME_VOLUME_LEFT;
	r = (reg & BM_AUDIOIN_ADCVOLUME_VOLUME_RIGHT) >>
		BP_AUDIOIN_ADCVOLUME_VOLUME_RIGHT;

	/* fade in adc vol */
	for (i = ADC_VOLUME_MIN; (i < l) || (i < r);) {
		i += 0x8;
		ll = i < l ? i : l;
		rr = i < r ? i : r;
		reg2 = reg1 |
		    BF_AUDIOIN_ADCVOLUME_VOLUME_LEFT(ll) |
		    BF_AUDIOIN_ADCVOLUME_VOLUME_RIGHT(rr);
		__raw_writel(reg2,
		    REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOLUME);
		msleep(1);
	}
	adc_ramp_done = 1;
}

static void mxs_dac_schedule_ramp_work(struct delayed_work *work)
{
	schedule_delayed_work(work, msecs_to_jiffies(2));
	dac_ramp_done = 0;
}

static void mxs_dac_ramp_work(struct work_struct *work)
{
	u32 reg = 0;
	u32 reg1 = 0;
	u32 l, r;
	u32 ll, rr;
	int i;

	/* unmute hp and speaker */
	__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
		REGS_AUDIOOUT_BASE + HW_AUDIOOUT_HPVOL_CLR);
	__raw_writel(BM_AUDIOOUT_SPEAKERCTRL_MUTE,
		REGS_AUDIOOUT_BASE + HW_AUDIOOUT_SPEAKERCTRL_CLR);

	reg = __raw_readl(REGS_AUDIOOUT_BASE + \
			HW_AUDIOOUT_HPVOL);

	reg1 = reg & ~BM_AUDIOOUT_HPVOL_VOL_LEFT;
	reg1 = reg1 & ~BM_AUDIOOUT_HPVOL_VOL_RIGHT;

	l = (reg & BM_AUDIOOUT_HPVOL_VOL_LEFT) >>
		BP_AUDIOOUT_HPVOL_VOL_LEFT;
	r = (reg & BM_AUDIOOUT_HPVOL_VOL_RIGHT) >>
		BP_AUDIOOUT_HPVOL_VOL_RIGHT;
	/* fade in hp vol */
	for (i = 0x7f; i > 0 ;) {
		i -= 0x8;
		ll = i > (int)l ? i : l;
		rr = i > (int)r ? i : r;
		reg = reg1 | BF_AUDIOOUT_HPVOL_VOL_LEFT(ll)
			| BF_AUDIOOUT_HPVOL_VOL_RIGHT(rr);
		__raw_writel(reg,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_HPVOL);
		msleep(1);
	}
	dac_ramp_done = 1;
}

static irqreturn_t mxs_short_irq(int irq, void *dev_id)
{
	__raw_writel(BM_AUDIOOUT_ANACTRL_SHORTMODE_LR,
		REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_CLR);
	__raw_writel(BM_AUDIOOUT_ANACTRL_SHORT_LR_STS,
		REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_CLR);
	__raw_writel(BF_AUDIOOUT_ANACTRL_SHORTMODE_LR(0x1),
		REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_SET);

	__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
	      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_HPVOL_SET);
	__raw_writel(BM_AUDIOOUT_PWRDN_HEADPHONE,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_PWRDN_SET);
	__raw_writel(BM_AUDIOOUT_ANACTRL_HP_CLASSAB,
		REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_SET);

	mxs_adc_schedule_work(&work);
	return IRQ_HANDLED;
}
static irqreturn_t mxs_err_irq(int irq, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	u32 ctrl_reg;
	u32 overflow_mask;
	u32 underflow_mask;

	if (playback) {
		ctrl_reg = __raw_readl(REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL);
		underflow_mask = BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ;
		overflow_mask = BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ;
	} else {
		ctrl_reg = __raw_readl(REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL);
		underflow_mask = BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ;
		overflow_mask = BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ;
	}

	if (ctrl_reg & underflow_mask) {
		printk(KERN_DEBUG "%s underflow detected\n",
		       playback ? "DAC" : "ADC");

		if (playback)
			__raw_writel(
				BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ,
				REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_CLR);
		else
			__raw_writel(
				BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ,
				REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_CLR);

	} else if (ctrl_reg & overflow_mask) {
		printk(KERN_DEBUG "%s overflow detected\n",
		       playback ? "DAC" : "ADC");

		if (playback)
			__raw_writel(
				BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ,
				REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_CLR);
		else
			__raw_writel(BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ,
				REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_CLR);
	} else
		printk(KERN_WARNING "Unknown DAC error interrupt\n");

	return IRQ_HANDLED;
}

static int mxs_adc_trigger(struct snd_pcm_substream *substream,
				int cmd,
				struct snd_soc_dai *dai)
{
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	struct mxs_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;
	u32 xfer_count1 = 0;
	u32 xfer_count2 = 0;
	u32 cur_bar1 = 0;
	u32 cur_bar2 = 0;
	u32 reg;
	struct mxs_dma_info dma_info;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:

		if (playback) {
			/* enable the fifo error interrupt */
		    __raw_writel(BM_AUDIOOUT_CTRL_FIFO_ERROR_IRQ_EN,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_SET);
			/* write a data to data reg to trigger the transfer */
		    __raw_writel(0x0,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DATA);
		    mxs_dac_schedule_ramp_work(&dac_ramp_work);
		} else {
		    mxs_dma_get_info(prtd->dma_ch, &dma_info);
		    cur_bar1 = dma_info.buf_addr;
		    xfer_count1 = dma_info.xfer_count;

		    __raw_writel(BM_AUDIOIN_CTRL_RUN,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_SET);
		    udelay(100);

		    mxs_dma_get_info(prtd->dma_ch, &dma_info);
		    cur_bar2 = dma_info.buf_addr;
		    xfer_count2 = dma_info.xfer_count;

		    /* check if DMA getting stuck */
		    if ((xfer_count1 == xfer_count2) && (cur_bar1 == cur_bar2))
			/* read a data from data reg to trigger the receive */
			reg = __raw_readl(REGS_AUDIOIN_BASE + HW_AUDIOIN_DATA);

		    mxs_adc_schedule_ramp_work(&adc_ramp_work);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:

		if (playback) {
			if (dac_ramp_done == 0) {
				cancel_delayed_work(&dac_ramp_work);
				dac_ramp_done = 1;
			}
			__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
			  REGS_AUDIOOUT_BASE + HW_AUDIOOUT_HPVOL_SET);
			__raw_writel(BM_AUDIOOUT_SPEAKERCTRL_MUTE,
			  REGS_AUDIOOUT_BASE + HW_AUDIOOUT_SPEAKERCTRL_SET);
			/* disable the fifo error interrupt */
			__raw_writel(BM_AUDIOOUT_CTRL_FIFO_ERROR_IRQ_EN,
				REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_CLR);
			mdelay(50);
		} else {
			if (adc_ramp_done == 0) {
				cancel_delayed_work(&adc_ramp_work);
				adc_ramp_done = 1;
			}
			__raw_writel(BM_AUDIOIN_CTRL_RUN,
				REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_CLR);
		}
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

static int mxs_adc_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int irq;
	int irq_short;
	int ret;

	INIT_DELAYED_WORK(&work, mxs_adc_work);
	INIT_DELAYED_WORK(&adc_ramp_work, mxs_adc_ramp_work);
	INIT_DELAYED_WORK(&dac_ramp_work, mxs_dac_ramp_work);

	if (playback) {
		irq = IRQ_DAC_ERROR;
		snd_soc_dai_set_dma_data(dai, substream, &mxs_audio_out);
	} else {
		irq = IRQ_ADC_ERROR;
		snd_soc_dai_set_dma_data(dai, substream, &mxs_audio_in);
	}

	ret = request_irq(irq, mxs_err_irq, 0, "MXS DAC and ADC Error",
			  substream);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request ADC/DAC error irq %d\n",
		       __func__, IRQ_DAC_ERROR);
		return ret;
	}

	irq_short = IRQ_HEADPHONE_SHORT;
	ret = request_irq(irq_short, mxs_short_irq,
		IRQF_DISABLED | IRQF_SHARED, "MXS DAC and ADC HP SHORT", substream);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request ADC/DAC HP SHORT irq %d\n",
		       __func__, IRQ_DAC_ERROR);
		return ret;
	}

	/* Enable error interrupt */
	if (playback) {
		__raw_writel(BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ,
				REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_CLR);
		__raw_writel(BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ,
				REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_CLR);
	} else {
		__raw_writel(BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_CLR);
		__raw_writel(BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_CLR);
		__raw_writel(BM_AUDIOIN_CTRL_FIFO_ERROR_IRQ_EN,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_SET);
	}

	return 0;
}

static void mxs_adc_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;

	/* Disable error interrupt */
	if (playback) {
		__raw_writel(BM_AUDIOOUT_CTRL_FIFO_ERROR_IRQ_EN,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_CLR);
		free_irq(IRQ_DAC_ERROR, substream);
	} else {
		__raw_writel(BM_AUDIOIN_CTRL_FIFO_ERROR_IRQ_EN,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_CLR);
		free_irq(IRQ_ADC_ERROR, substream);
	}
}

#ifdef CONFIG_PM
static int mxs_adc_suspend(struct snd_soc_dai *cpu_dai)
{
	return 0;
}

static int mxs_adc_resume(struct snd_soc_dai *cpu_dai)
{
	return 0;
}
#else
#define mxs_adc_suspend	NULL
#define mxs_adc_resume	NULL
#endif /* CONFIG_PM */

struct snd_soc_dai_ops mxs_adc_dai_ops = {
	.startup = mxs_adc_startup,
	.shutdown = mxs_adc_shutdown,
	.trigger = mxs_adc_trigger,
};

struct snd_soc_dai mxs_adc_dai = {
	.name = "mxs adc/dac",
	.id = 0,
	.suspend = mxs_adc_suspend,
	.resume = mxs_adc_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXS_ADC_RATES,
		.formats = MXS_ADC_FORMATS,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = MXS_ADC_RATES,
		.formats = MXS_ADC_FORMATS,
	},
	.ops = &mxs_adc_dai_ops,
};
EXPORT_SYMBOL_GPL(mxs_adc_dai);

static int __init mxs_adc_dai_init(void)
{
	return snd_soc_register_dai(&mxs_adc_dai);
}

static void __exit mxs_adc_dai_exit(void)
{
	snd_soc_unregister_dai(&mxs_adc_dai);
}
module_init(mxs_adc_dai_init);
module_exit(mxs_adc_dai_exit);

MODULE_AUTHOR("Vladislav Buzov");
MODULE_DESCRIPTION("MXS ADC/DAC DAI");
MODULE_LICENSE("GPL");
