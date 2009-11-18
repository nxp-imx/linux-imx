/*
 * ASoC PCM interface for Freescale STMP37XX/STMP378X ADC/DAC
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/platform.h>

#include <mach/regs-apbx.h>

#include "stmp3xxx_pcm.h"

static const struct snd_pcm_hardware stmp3xxx_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	.channels_min		= 2,
	.channels_max		= 2,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8192,
	.periods_min		= 1,
	.periods_max		= 255,
	.buffer_bytes_max	= 64 * 1024,
	.fifo_size		= 32,
};

/*
 * Required to request DMA channels
 */
struct device *stmp3xxx_pcm_dev;

struct stmp3xxx_runtime_data {
	u32 dma_ch;
	u32 dma_period;
	u32 dma_totsize;

	struct stmp3xxx_pcm_dma_params *params;
	struct stmp3xxx_dma_descriptor *dma_desc_array;
};

static irqreturn_t stmp3xxx_pcm_dma_irq(int irq, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct stmp3xxx_runtime_data *prtd = substream->runtime->private_data;

#ifdef CONFIG_ARCH_STMP37XX
	u32 err_mask = 1 << (16 + prtd->params->dma_ch);
#endif
#ifdef CONFIG_ARCH_STMP378X
	u32 err_mask = 1 << prtd->params->dma_ch;
#endif
	u32 irq_mask = 1 << prtd->params->dma_ch;

#ifdef CONFIG_ARCH_STMP37XX
	if (__raw_readl(REGS_APBX_BASE + HW_APBX_CTRL1) & err_mask) {
#endif
#ifdef CONFIG_ARCH_STMP378X
	if (__raw_readl(REGS_APBX_BASE + HW_APBX_CTRL2) & err_mask) {
#endif
		printk(KERN_WARNING "%s: DMA audio channel %d (%s) error\n",
		       __func__, prtd->params->dma_ch, prtd->params->name);
#ifdef CONFIG_ARCH_STMP37XX
		__raw_writel(err_mask, REGS_APBX_BASE + HW_APBX_CTRL1_CLR);
#endif
#ifdef CONFIG_ARCH_STMP378X
		__raw_writel(err_mask, REGS_APBX_BASE + HW_APBX_CTRL2_CLR);
#endif
	} else if (__raw_readl(REGS_APBX_BASE + HW_APBX_CTRL1) & irq_mask) {
		stmp3xxx_dma_clear_interrupt(prtd->dma_ch);
		snd_pcm_period_elapsed(substream);
	} else
		printk(KERN_WARNING "%s: Unknown interrupt\n", __func__);

	return IRQ_HANDLED;
}

/*
 * Make a circular DMA descriptor list
 */
static int stmp3xxx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct stmp3xxx_runtime_data *prtd = runtime->private_data;
	dma_addr_t dma_buffer_phys;
	int periods_num, playback, i;

	playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	periods_num = prtd->dma_totsize / prtd->dma_period;
	dma_buffer_phys = runtime->dma_addr;

	/* Reset DMA channel, enable interrupt */
	stmp3xxx_dma_reset_channel(prtd->dma_ch);

	/* Set up a DMA chain to sent DMA buffer */
	for (i = 0; i < periods_num; i++) {
		int next = (i + 1) % periods_num;
		u32 cmd = 0;

		/* Link with previous command */
		prtd->dma_desc_array[i].command->next =
				prtd->dma_desc_array[next].handle;

		prtd->dma_desc_array[i].next_descr =
				&prtd->dma_desc_array[next];

		cmd = BF(prtd->dma_period, APBX_CHn_CMD_XFER_COUNT) |
		      BM_APBX_CHn_CMD_IRQONCMPLT |
		      BM_APBX_CHn_CMD_CHAIN;

		/* Set DMA direction */
		if (playback)
			cmd |= BF(BV_APBX_CHn_CMD_COMMAND__DMA_READ,
					APBX_CHn_CMD_COMMAND);
		else
			cmd |= BF(BV_APBX_CHn_CMD_COMMAND__DMA_WRITE,
					APBX_CHn_CMD_COMMAND);

		prtd->dma_desc_array[i].command->cmd = cmd;
		prtd->dma_desc_array[i].command->buf_ptr = dma_buffer_phys;

		/* Next data chunk */
		dma_buffer_phys += prtd->dma_period;
	}

	return 0;
}

/*
 * Stop circular DMA descriptor list
 * We should not stop DMA in a middle of current transaction once we receive
 * stop request from ALSA core. This function finds the next DMA descriptor
 * and set it up to decrement DMA channel semaphore. So the current transaction
 * is the last data transfer.
 */
static void stmp3xxx_pcm_stop(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct stmp3xxx_runtime_data *prtd = runtime->private_data;
	dma_addr_t pos;
	int desc;

	/* Freez DMA channel for a moment */
	stmp3xxx_dma_freeze(prtd->dma_ch);

	/* Find current DMA descriptor */
	pos = __raw_readl(REGS_APBX_BASE +
			HW_APBX_CHn_BAR(prtd->params->dma_ch));
	desc = (pos - runtime->dma_addr) / prtd->dma_period;

	/* Set up the next descriptor to decrement DMA channel sempahore */
	prtd->dma_desc_array[desc].next_descr->command->cmd
					= BM_APBX_CHn_CMD_SEMAPHORE;

	/* Let the current DMA transaction finish */
	stmp3xxx_dma_unfreeze(prtd->dma_ch);
}

static int stmp3xxx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct stmp3xxx_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

	switch (cmd) {

	case SNDRV_PCM_TRIGGER_START:
		stmp3xxx_dma_go(prtd->dma_ch, prtd->dma_desc_array, 1);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		stmp3xxx_pcm_stop(substream);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		stmp3xxx_dma_unfreeze(prtd->dma_ch);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		stmp3xxx_dma_freeze(prtd->dma_ch);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static snd_pcm_uframes_t
stmp3xxx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct stmp3xxx_runtime_data *prtd = runtime->private_data;
	unsigned int offset;
	dma_addr_t pos;

	pos = __raw_readl(REGS_APBX_BASE +
			HW_APBX_CHn_BAR(prtd->params->dma_ch));
	offset = bytes_to_frames(runtime, pos - runtime->dma_addr);

	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;
}

static int stmp3xxx_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	struct stmp3xxx_runtime_data *prtd = substream->runtime->private_data;

	prtd->dma_period = params_period_bytes(hw_params);
	prtd->dma_totsize = params_buffer_bytes(hw_params);

	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int stmp3xxx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int stmp3xxx_pcm_dma_request(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct stmp3xxx_runtime_data *prtd = runtime->private_data;
	struct stmp3xxx_pcm_dma_params *dma_data = rtd->dai->cpu_dai->dma_data;
	int desc_num = stmp3xxx_pcm_hardware.periods_max;
	int desc;
	int ret;

	if (!dma_data)
		return -ENODEV;

	prtd->params = dma_data;
	prtd->dma_ch = STMP3XXX_DMA(dma_data->dma_ch, dma_data->dma_bus);

	ret = stmp3xxx_dma_request(prtd->dma_ch, stmp3xxx_pcm_dev,
				   prtd->params->name);
	if (ret) {
		printk(KERN_ERR "%s: Failed to request DMA channel (%d:%d)\n",
		       __func__, dma_data->dma_bus, dma_data->dma_ch);
		return ret;
	}

	/* Allocate memory for data and pio DMA descriptors */
	prtd->dma_desc_array =
		kzalloc(sizeof(struct stmp3xxx_dma_descriptor) * desc_num,
			GFP_KERNEL);
	if (prtd->dma_desc_array == NULL) {
		printk(KERN_ERR "%s: Unable to allocate memory\n", __func__);
		stmp3xxx_dma_release(prtd->dma_ch);
		return -ENOMEM;
	}

	for (desc = 0; desc < desc_num; desc++) {
		ret = stmp3xxx_dma_allocate_command(prtd->dma_ch,
					    &prtd->dma_desc_array[desc]);
		if (ret) {
			printk(KERN_ERR"%s Unable to allocate DMA command %d\n",
			       __func__, desc);
			goto err;
		}
	}

	ret = request_irq(prtd->params->irq, stmp3xxx_pcm_dma_irq, 0,
			  "STMP3xxx PCM DMA", substream);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request DMA irq %d\n", __func__,
		       prtd->params->irq);
		goto err;
	}


	/* Enable completion interrupt */
	stmp3xxx_dma_clear_interrupt(prtd->dma_ch);
	stmp3xxx_dma_enable_interrupt(prtd->dma_ch);

	return 0;

err:
	while (--desc >= 0)
		stmp3xxx_dma_free_command(prtd->dma_ch,
					  &prtd->dma_desc_array[desc]);
	kfree(prtd->dma_desc_array);
	stmp3xxx_dma_release(prtd->dma_ch);

	return ret;
}

static int stmp3xxx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct stmp3xxx_runtime_data *prtd;
	int ret;

	/* Ensure that buffer size is a multiple of the period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
					SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	snd_soc_set_runtime_hwparams(substream, &stmp3xxx_pcm_hardware);

	prtd = kzalloc(sizeof(struct stmp3xxx_runtime_data), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	runtime->private_data = prtd;

	ret = stmp3xxx_pcm_dma_request(substream);
	if (ret) {
		printk(KERN_ERR "stmp3xxx_pcm: Failed to request channels\n");
		kfree(prtd);
		return ret;
	}

	return 0;
}

static int stmp3xxx_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct stmp3xxx_runtime_data *prtd = runtime->private_data;
	int desc_num = stmp3xxx_pcm_hardware.periods_max;
	int desc;

	/* Free DMA irq */
	free_irq(prtd->params->irq, substream);

	/* Free DMA channel */
	for (desc = 0; desc < desc_num; desc++)
		stmp3xxx_dma_free_command(prtd->dma_ch,
					  &prtd->dma_desc_array[desc]);
	kfree(prtd->dma_desc_array);
	stmp3xxx_dma_release(prtd->dma_ch);

	/* Free private runtime data */
	kfree(prtd);

	return 0;
}

static int stmp3xxx_pcm_mmap(struct snd_pcm_substream *substream,
			     struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_coherent(NULL, vma, runtime->dma_area,
				 runtime->dma_addr, runtime->dma_bytes);
}

struct snd_pcm_ops stmp3xxx_pcm_ops = {
	.open		= stmp3xxx_pcm_open,
	.close		= stmp3xxx_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= stmp3xxx_pcm_hw_params,
	.hw_free	= stmp3xxx_pcm_hw_free,
	.prepare	= stmp3xxx_pcm_prepare,
	.trigger	= stmp3xxx_pcm_trigger,
	.pointer	= stmp3xxx_pcm_pointer,
	.mmap		= stmp3xxx_pcm_mmap,
};

static u64 stmp3xxx_pcm_dma_mask = DMA_BIT_MASK(32);

static int stmp3xxx_pcm_new(struct snd_card *card,
			    struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	size_t size = stmp3xxx_pcm_hardware.buffer_bytes_max;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &stmp3xxx_pcm_dma_mask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, NULL,
					      size, size);

	return 0;
}

static void stmp3xxx_pcm_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

/*
 * We need probe/remove callbacks to setup stmp3xxx_pcm_dev
 */
static int stmp3xxx_pcm_probe(struct platform_device *pdev)
{
	stmp3xxx_pcm_dev = &pdev->dev;
	return 0;
}

static int stmp3xxx_pcm_remove(struct platform_device *pdev)
{
	stmp3xxx_pcm_dev = NULL;
	return 0;
}

struct snd_soc_platform stmp3xxx_soc_platform = {
	.name		= "STMP3xxx Audio",
	.pcm_ops	= &stmp3xxx_pcm_ops,
	.probe		= stmp3xxx_pcm_probe,
	.remove		= stmp3xxx_pcm_remove,
	.pcm_new	= stmp3xxx_pcm_new,
	.pcm_free	= stmp3xxx_pcm_free,
};
EXPORT_SYMBOL_GPL(stmp3xxx_soc_platform);

static int __init stmp3xxx_pcm_init(void)
{
	return snd_soc_register_platform(&stmp3xxx_soc_platform);
}

static void __exit stmp3xxx_pcm_exit(void)
{
	snd_soc_unregister_platform(&stmp3xxx_soc_platform);
}
module_init(stmp3xxx_pcm_init);
module_exit(stmp3xxx_pcm_exit);

MODULE_AUTHOR("Vladislav Buzov");
MODULE_DESCRIPTION("STMP3xxx DMA Module");
MODULE_LICENSE("GPL");
