/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/dmaengine.h>

#include "mxs-pcm.h"
static const struct snd_pcm_hardware mxs_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S20_3LE |
				  SNDRV_PCM_FMTBIT_S24_LE |
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
struct device *mxs_pcm_dev;

static irqreturn_t mxs_pcm_dma_irq(int irq, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxs_runtime_data *prtd = substream->runtime->private_data;
	struct mxs_dma_info dma_info;
	void *pdma;
	unsigned long prev_appl_offset, appl_count, cont, appl_ptr_bytes;

	mxs_dma_get_info(prtd->dma_ch, &dma_info);

	if (dma_info.status) {
		printk(KERN_WARNING "%s: DMA audio channel %d (%s) error\n",
			__func__, prtd->params->dma_ch, prtd->params->name);
		mxs_dma_ack_irq(prtd->dma_ch);
	} else {
		if ((prtd->params->dma_ch == MXS_DMA_CHANNEL_AHB_APBX_SPDIF) &&
		    (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) &&
		    (runtime->access == SNDRV_PCM_ACCESS_MMAP_INTERLEAVED) &&
		    ((prtd->format == SNDRV_PCM_FORMAT_S24_LE)
		     || (prtd->format == SNDRV_PCM_FORMAT_S20_3LE))) {

			appl_ptr_bytes =
			    frames_to_bytes(runtime,
					    runtime->control->appl_ptr);

			appl_count = appl_ptr_bytes - prtd->appl_ptr_bytes;
			prev_appl_offset =
			    prtd->appl_ptr_bytes % prtd->dma_totsize;
			cont = prtd->dma_totsize - prev_appl_offset;

			if (appl_count > cont) {
				pdma = runtime->dma_area + prev_appl_offset;
				memmove(pdma + 1, pdma, cont - 1);
				pdma = runtime->dma_area;
				memmove(pdma + 1, pdma, appl_count - cont - 1);
			} else {
				pdma = runtime->dma_area + prev_appl_offset;
				memmove(pdma + 1, pdma, appl_count - 1);
			}
			prtd->appl_ptr_bytes = appl_ptr_bytes;
		}
		mxs_dma_ack_irq(prtd->dma_ch);
		snd_pcm_period_elapsed(substream);
	}
	return IRQ_HANDLED;
}

/*
 * Make a circular DMA descriptor list
 */
static int mxs_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxs_runtime_data *prtd = runtime->private_data;
	dma_addr_t dma_buffer_phys;
	int periods_num, playback, i;

	playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	periods_num = prtd->dma_totsize / prtd->dma_period;

	dma_buffer_phys = runtime->dma_addr;

	/* Reset DMA channel, enable interrupt */
	mxs_dma_reset(prtd->dma_ch);

	/* Set up a DMA chain to sent DMA buffer */
	for (i = 0; i < periods_num; i++) {
		int ret;
		/* Link with previous command */
		prtd->dma_desc_array[i]->cmd.cmd.bits.bytes = prtd->dma_period;
		prtd->dma_desc_array[i]->cmd.cmd.bits.irq = 1;
		prtd->dma_desc_array[i]->cmd.cmd.bits.dec_sem = 0;
		prtd->dma_desc_array[i]->cmd.cmd.bits.chain = 1;
		/* Set DMA direction */
		if (playback)
			prtd->dma_desc_array[i]->cmd.cmd.bits.command = \
				DMA_READ;
		else
			prtd->dma_desc_array[i]->cmd.cmd.bits.command = \
				DMA_WRITE;

		prtd->dma_desc_array[i]->cmd.address = dma_buffer_phys;

		ret = mxs_dma_desc_append(prtd->dma_ch, \
			prtd->dma_desc_array[i]);
		if (ret) {
			printk(KERN_ERR "%s: Failed to append DMA descriptor\n",
		       __func__);
			return ret;
		}
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
static void mxs_pcm_stop(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxs_runtime_data *prtd = runtime->private_data;
	struct mxs_dma_info dma_info;
	int desc;

	int periods_num = prtd->dma_totsize / prtd->dma_period;
	/* Freez DMA channel for a moment */
	mxs_dma_freeze(prtd->dma_ch);
	mxs_dma_get_info(prtd->dma_ch, &dma_info);

	desc = (dma_info.buf_addr - runtime->dma_addr) / prtd->dma_period;

	if (desc >= periods_num)
		desc = 0;
	else if (desc < 0)
		desc = 0;

	/* Set up the next descriptor to decrement DMA channel sempahore */
	prtd->dma_desc_array[(desc + 1)%periods_num]->cmd.cmd.bits.bytes = 0;
	prtd->dma_desc_array[(desc + 1)%periods_num]->cmd.cmd.bits.pio_words = \
		0;
	prtd->dma_desc_array[(desc + 1)%periods_num]->cmd.cmd.bits.dec_sem = 1;
	prtd->dma_desc_array[(desc + 1)%periods_num]->cmd.cmd.bits.irq = 0;
	prtd->dma_desc_array[(desc + 1)%periods_num]->cmd.cmd.bits.command = \
		NO_DMA_XFER;

	mxs_dma_unfreeze(prtd->dma_ch);

	mxs_dma_disable(prtd->dma_ch);
}

static int mxs_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxs_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;
	switch (cmd) {

	case SNDRV_PCM_TRIGGER_START:
		if ((prtd->params->dma_ch == MXS_DMA_CHANNEL_AHB_APBX_SPDIF) &&
		    (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) &&
		    (runtime->access == SNDRV_PCM_ACCESS_MMAP_INTERLEAVED) &&
		    ((prtd->format == SNDRV_PCM_FORMAT_S24_LE)
		     || (prtd->format == SNDRV_PCM_FORMAT_S20_3LE))) {
			prtd->appl_ptr_bytes =
			    frames_to_bytes(runtime,
					    runtime->control->appl_ptr);
			memmove(runtime->dma_area + 1, runtime->dma_area,
				prtd->appl_ptr_bytes - 1);
		}
		mxs_dma_enable(prtd->dma_ch);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		mxs_pcm_stop(substream);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		mxs_dma_unfreeze(prtd->dma_ch);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		mxs_dma_freeze(prtd->dma_ch);
		mdelay(30);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static snd_pcm_uframes_t
mxs_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxs_runtime_data *prtd = runtime->private_data;
	struct mxs_dma_info dma_info;
	unsigned int offset;
	dma_addr_t pos;

	mxs_dma_get_info(prtd->params->dma_ch, &dma_info);
	pos = dma_info.buf_addr;

	offset = bytes_to_frames(runtime, pos - runtime->dma_addr);

	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;
}

static int mxs_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	struct mxs_runtime_data *prtd = substream->runtime->private_data;

	prtd->dma_period = params_period_bytes(hw_params);
	pr_debug("The period size is :%d\n", prtd->dma_period);
	prtd->dma_totsize = params_buffer_bytes(hw_params);
	prtd->format = params_format(hw_params);

	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int mxs_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int mxs_pcm_dma_request(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mxs_runtime_data *prtd = runtime->private_data;
	struct mxs_pcm_dma_params *dma_data =
		snd_soc_dai_get_dma_data(rtd->dai->cpu_dai, substream);
	int desc_num = mxs_pcm_hardware.periods_max;
	int desc;
	int ret;

	if (!dma_data)
		return -ENODEV;

	prtd->params = dma_data;
	prtd->dma_ch = dma_data->dma_ch;

	ret = mxs_dma_request(prtd->dma_ch, mxs_pcm_dev,
				   prtd->params->name);
	if (ret) {
		printk(KERN_ERR "%s: Failed to request DMA channel (%d:%d)\n",
		       __func__, dma_data->dma_bus, dma_data->dma_ch);
		return ret;
	}

	/* Allocate memory for data and pio DMA descriptors */
	for (desc = 0; desc < desc_num; desc++) {
		prtd->dma_desc_array[desc] = mxs_dma_alloc_desc();
		if (prtd->dma_desc_array[desc] == NULL) {
			printk(KERN_ERR"%s Unable to allocate DMA command %d\n",
			       __func__, desc);
			goto err;
		}
	}

	ret = request_irq(prtd->params->irq, mxs_pcm_dma_irq, 0,
			  "MXS PCM DMA", substream);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request DMA irq %d\n", __func__,
		       prtd->params->irq);
		goto err;
	}
	/* Enable completion interrupt */
	mxs_dma_ack_irq(prtd->dma_ch);
	mxs_dma_enable_irq(prtd->dma_ch, 1);

	return 0;

err:
	while (--desc >= 0)
		mxs_dma_free_desc(prtd->dma_desc_array[desc]);
	mxs_dma_release(prtd->dma_ch, mxs_pcm_dev);

	return ret;
}

static int mxs_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxs_runtime_data *prtd;
	int ret;

	/* Ensure that buffer size is a multiple of the period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
					SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	snd_soc_set_runtime_hwparams(substream, &mxs_pcm_hardware);

	prtd = kzalloc(sizeof(struct mxs_runtime_data), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	runtime->private_data = prtd;

	ret = mxs_pcm_dma_request(substream);
	if (ret) {
		printk(KERN_ERR "mxs_pcm: Failed to request channels\n");
		kfree(prtd);
		return ret;
	}
	return 0;
}

static int mxs_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxs_runtime_data *prtd = runtime->private_data;
	int desc_num = mxs_pcm_hardware.periods_max;
	int desc;
	int timeo = 20;

	static LIST_HEAD(list);
	mxs_dma_disable(prtd->dma_ch);

	/* Wait until the DMA chain is finished. */
	while (mxs_dma_read_semaphore(prtd->dma_ch)) {
		if (!timeo--)
			break;
		pr_debug("The sema is not zero now\n");
		msleep(10);
	}
	if (timeo <= 0)
		pr_warn("Is the DMA channel dead?\n");

	/* Free DMA irq */
	free_irq(prtd->params->irq, substream);
	mxs_dma_get_cooked(prtd->dma_ch, &list);
	/* Free DMA channel*/
	for (desc = 0; desc < desc_num; desc++)
		mxs_dma_free_desc(prtd->dma_desc_array[desc]);
	mxs_dma_release(prtd->dma_ch, mxs_pcm_dev);

	/* Free private runtime data */
	kfree(prtd);
	return 0;
}

static int mcs_pcm_copy(struct snd_pcm_substream *substream, int channel,
			snd_pcm_uframes_t hwoff, void __user *buf,
			snd_pcm_uframes_t frames)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxs_runtime_data *prtd = runtime->private_data;
	char *hwbuf = runtime->dma_area + frames_to_bytes(runtime, hwoff);
	unsigned long count = frames_to_bytes(runtime, frames);

	/* For S/PDIF 24-bit playback, fix the buffer.  Code taken from
	   snd_pcm_lib_write_transfer() and snd_pcm_lib_read_transfer()
	   in sound/core/pcm_lib.c */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if ((prtd->params->dma_ch == MXS_DMA_CHANNEL_AHB_APBX_SPDIF) &&
		    ((prtd->format == SNDRV_PCM_FORMAT_S24_LE)
		     || (prtd->format == SNDRV_PCM_FORMAT_S20_3LE))) {
			if (copy_from_user(hwbuf + 1, buf, count - 1))
				return -EFAULT;
		} else {
			if (copy_from_user(hwbuf, buf, count))
				return -EFAULT;
		}
	} else {
		if (copy_to_user(buf, hwbuf, count))
			return -EFAULT;
	}

	return 0;
}

static int mxs_pcm_mmap(struct snd_pcm_substream *substream,
			     struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_coherent(NULL, vma, runtime->dma_area,
				 runtime->dma_addr, runtime->dma_bytes);
}

struct snd_pcm_ops mxs_pcm_ops = {
	.open		= mxs_pcm_open,
	.close		= mxs_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= mxs_pcm_hw_params,
	.hw_free	= mxs_pcm_hw_free,
	.prepare	= mxs_pcm_prepare,
	.trigger	= mxs_pcm_trigger,
	.pointer	= mxs_pcm_pointer,
	.copy		= mcs_pcm_copy,
	.mmap		= mxs_pcm_mmap,
};

static u64 mxs_pcm_dma_mask = DMA_BIT_MASK(32);

static int mxs_pcm_new(struct snd_card *card,
			    struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	size_t size = mxs_pcm_hardware.buffer_bytes_max;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &mxs_pcm_dma_mask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV, NULL,
					      size, size);

	return 0;
}

static void mxs_pcm_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

/*
 * We need probe/remove callbacks to setup mxs_pcm_dev
 */
static int mxs_pcm_probe(struct platform_device *pdev)
{
	mxs_pcm_dev = &pdev->dev;
	return 0;
}

static int mxs_pcm_remove(struct platform_device *pdev)
{
	mxs_pcm_dev = NULL;
	return 0;
}

struct snd_soc_platform mxs_soc_platform = {
	.name		= "MXS Audio",
	.pcm_ops	= &mxs_pcm_ops,
	.probe		= mxs_pcm_probe,
	.remove		= mxs_pcm_remove,
	.pcm_new	= mxs_pcm_new,
	.pcm_free	= mxs_pcm_free,
};
EXPORT_SYMBOL_GPL(mxs_soc_platform);

static int __init mxs_pcm_init(void)
{
	return snd_soc_register_platform(&mxs_soc_platform);
}

static void __exit mxs_pcm_exit(void)
{
	snd_soc_unregister_platform(&mxs_soc_platform);
}
module_init(mxs_pcm_init);
module_exit(mxs_pcm_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXS DMA Module");
MODULE_LICENSE("GPL");
