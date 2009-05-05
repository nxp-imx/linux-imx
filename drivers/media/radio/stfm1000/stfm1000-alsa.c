/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/math64.h>

#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/version.h>      /* for KERNEL_VERSION MACRO     */
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <mach/regs-dri.h>
#include <mach/regs-apbx.h>
#include <mach/regs-clkctrl.h>

#include "stfm1000.h"

#define STFM1000_PERIODS	16

static int stfm1000_snd_volume_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;			/* two channels */
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 20;
	return 0;
}

static int stfm1000_snd_volume_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct stfm1000 *stfm1000 = snd_kcontrol_chip(kcontrol);

	(void)stfm1000;
	ucontrol->value.integer.value[0] = 0;	/* left */
	ucontrol->value.integer.value[1] = 0;	/* right */
	return 0;
}

static int stfm1000_snd_volume_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct stfm1000 *stfm1000 = snd_kcontrol_chip(kcontrol);
	int change;
	int left, right;

	(void)stfm1000;

	left = ucontrol->value.integer.value[0];
	if (left < 0)
		left = 0;
	if (left > 20)
		left = 20;
	right = ucontrol->value.integer.value[1];
	if (right < 0)
		right = 0;
	if (right > 20)
		right = 20;

	change = 1;
	return change;
}

static struct snd_kcontrol_new stfm1000_snd_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Radio Volume",
		.index = 0,
		.info = stfm1000_snd_volume_info,
		.get = stfm1000_snd_volume_get,
		.put = stfm1000_snd_volume_put,
		.private_value = 0,
	},
};

static struct snd_pcm_hardware stfm1000_snd_capture = {

	.info =                 SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats =		SNDRV_PCM_FMTBIT_S16_LE,
	.rates =		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
	.rate_min =		44100,
	.rate_max =		48000,
	.channels_min =		2,
	.channels_max =		2,
	.buffer_bytes_max =	SZ_256K,
	.period_bytes_min =	SZ_4K,
	.period_bytes_max =	SZ_4K,
	.periods_min =		STFM1000_PERIODS,
	.periods_max =		STFM1000_PERIODS,
};

static int stfm1000_snd_capture_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct stfm1000 *stfm1000 = snd_pcm_substream_chip(substream);
	int err;

	/* should never happen, just a sanity check */
	BUG_ON(stfm1000 == NULL);

	mutex_lock(&stfm1000->deffered_work_lock);
	stfm1000->read_count  = 0;
	stfm1000->read_offset = 0;

	stfm1000->substream = substream;
	runtime->private_data = stfm1000;
	runtime->hw = stfm1000_snd_capture;

	mutex_unlock(&stfm1000->deffered_work_lock);

	err = snd_pcm_hw_constraint_integer(runtime,
		SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0) {
		printk(KERN_ERR "%s: snd_pcm_hw_constraint_integer "
			"SNDRV_PCM_HW_PARAM_PERIODS failed\n", __func__);
		return err;
	}

	err = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIODS, 2);
	if (err < 0) {
		printk(KERN_ERR "%s: snd_pcm_hw_constraint_integer "
			"SNDRV_PCM_HW_PARAM_PERIODS failed\n", __func__);
		return err;
	}

	return 0;
}

static int stfm1000_snd_capture_close(struct snd_pcm_substream *substream)
{
	struct stfm1000 *stfm1000 = snd_pcm_substream_chip(substream);

	(void)stfm1000;	/* nothing */
	return 0;
}

static int stfm1000_snd_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *hw_params)
{
	struct stfm1000 *stfm1000 = snd_pcm_substream_chip(substream);
	unsigned int period_size, periods;
	int ret;

	periods = params_periods(hw_params);
	period_size = params_period_bytes(hw_params);

	if (period_size < 0x100 || period_size > 0x10000)
		return -EINVAL;
	if (periods < STFM1000_PERIODS)
		return -EINVAL;
	if (period_size * periods > 1024 * 1024)
		return -EINVAL;

	stfm1000->blocks  = periods;
	stfm1000->blksize = period_size;
	stfm1000->bufsize = params_buffer_bytes(hw_params);

	ret = snd_pcm_lib_malloc_pages(substream, stfm1000->bufsize);
	if (ret < 0) {	/* 0 & 1 are valid returns */
		printk(KERN_ERR "%s: snd_pcm_lib_malloc_pages() failed\n",
			__func__);
		return ret;
	}

	/* the dri buffer is twice as large as the audio buffer */
	stfm1000->dri_bufsz = (stfm1000->bufsize / 4) *
		sizeof(struct stfm1000_dri_sample);
	stfm1000->dri_buf = dma_alloc_coherent(&stfm1000->radio.dev,
			stfm1000->dri_bufsz, &stfm1000->dri_phys, GFP_KERNEL);
	if (stfm1000->dri_buf == NULL) {
		printk(KERN_ERR "%s: dma_alloc_coherent() failed\n", __func__);
		snd_pcm_lib_free_pages(substream);
		return -ENOMEM;
	}

	return ret;
}

static int stfm1000_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct stfm1000 *stfm1000 = snd_pcm_substream_chip(substream);

	if (stfm1000->dri_buf) {
		dma_free_coherent(&stfm1000->radio.dev,
			(stfm1000->bufsize / 4) *
			sizeof(struct stfm1000_dri_sample),
			stfm1000->dri_buf, stfm1000->dri_phys);
		stfm1000->dri_buf = NULL;
		stfm1000->dri_phys = 0;
	}
	snd_pcm_lib_free_pages(substream);
	return 0;
}


static int stfm1000_snd_capture_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct stfm1000 *stfm1000 = snd_pcm_substream_chip(substream);

	stfm1000->substream = substream;

	if (snd_pcm_format_width(runtime->format) != 16 ||
		!snd_pcm_format_signed(runtime->format) ||
			snd_pcm_format_big_endian(runtime->format)) {
		printk(KERN_INFO "STFM1000: ALSA capture_prepare illegal format\n");
		return -EINVAL;
	}

	/* really shouldn't happen */
	BUG_ON(stfm1000->blocks > stfm1000->desc_num);

	mutex_lock(&stfm1000->deffered_work_lock);

	if (stfm1000->now_recording != 0) {
		printk(KERN_INFO "STFM1000: ALSA capture_prepare still running\n");
		mutex_unlock(&stfm1000->deffered_work_lock);
		return -EBUSY;
	}
	stfm1000->now_recording = 1;

	mutex_unlock(&stfm1000->deffered_work_lock);

	return 0;

}

static void stfm1000_snd_capture_trigger_start(struct work_struct *work)
{
	struct stfm1000 *stfm1000;

	stfm1000 = container_of(work, struct stfm1000,
		snd_capture_start_work.work);

	mutex_lock(&stfm1000->deffered_work_lock);

	BUG_ON(stfm1000->now_recording != 1);

	stfm1000_bring_up(stfm1000);

	mutex_unlock(&stfm1000->deffered_work_lock);
}

static void stfm1000_snd_capture_trigger_stop(struct work_struct *work)
{
	struct stfm1000 *stfm1000;

	stfm1000 = container_of(work, struct stfm1000,
		snd_capture_stop_work.work);

	mutex_lock(&stfm1000->deffered_work_lock);

	stfm1000->stopping_recording = 1;

	stfm1000_take_down(stfm1000);

	BUG_ON(stfm1000->now_recording != 1);
	stfm1000->now_recording = 0;

	stfm1000->stopping_recording = 0;

	mutex_unlock(&stfm1000->deffered_work_lock);
}

static int execute_non_atomic(work_func_t fn, struct execute_work *ew)
{
	if (!in_atomic() && !in_interrupt()) {
		fn(&ew->work);
		return 0;
	}

	INIT_WORK(&ew->work, fn);
	schedule_work(&ew->work);

	return 1;
}

static int stfm1000_snd_capture_trigger(struct snd_pcm_substream *substream,
	int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct stfm1000 *stfm1000 = runtime->private_data;
	int err = 0;

	(void)stfm1000;

	switch (cmd) {

	case SNDRV_PCM_TRIGGER_START:
		execute_non_atomic(stfm1000_snd_capture_trigger_start,
			&stfm1000->snd_capture_start_work);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		execute_non_atomic(stfm1000_snd_capture_trigger_stop,
			&stfm1000->snd_capture_stop_work);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		stmp3xxx_dma_unfreeze(stfm1000->dma_ch);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		stmp3xxx_dma_freeze(stfm1000->dma_ch);
		break;

	default:
		err = -EINVAL;
		break;
	}

	return err;
}

static snd_pcm_uframes_t
stfm1000_snd_capture_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct stfm1000 *stfm1000 = runtime->private_data;

	if (stfm1000->read_count) {
		stfm1000->read_count  -= snd_pcm_lib_period_bytes(substream);
		stfm1000->read_offset += snd_pcm_lib_period_bytes(substream);
		if (stfm1000->read_offset == substream->runtime->dma_bytes)
			stfm1000->read_offset = 0;
	}

	return bytes_to_frames(runtime, stfm1000->read_offset);
}

static struct snd_pcm_ops stfm1000_snd_capture_ops = {
	.open = stfm1000_snd_capture_open,
	.close = stfm1000_snd_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = stfm1000_snd_hw_params,
	.hw_free = stfm1000_snd_hw_free,
	.prepare = stfm1000_snd_capture_prepare,
	.trigger = stfm1000_snd_capture_trigger,
	.pointer = stfm1000_snd_capture_pointer,
};

static void stfm1000_snd_free(struct snd_card *card)
{
	struct stfm1000 *stfm1000 = card->private_data;

	free_irq(IRQ_DRI_ATTENTION, stfm1000);
	free_irq(IRQ_DRI_DMA, stfm1000);
}

static int stfm1000_alsa_instance_init(struct stfm1000 *stfm1000)
{
	int ret, i;
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_kcontrol *ctl;

	mutex_init(&stfm1000->deffered_work_lock);

	/* request dma channel */
	stfm1000->desc_num = STFM1000_PERIODS;
	stfm1000->dma_ch = STMP3xxx_DMA(5, STMP3XXX_BUS_APBX);
	ret = stmp3xxx_dma_request(stfm1000->dma_ch, &stfm1000->radio.dev,
			"stmp3xxx dri");
	if (ret != 0) {
		printk(KERN_ERR "%s: stmp3xxx_dma_request failed\n", __func__);
		goto err;
	}

	stfm1000->dma = kzalloc(sizeof(*stfm1000->dma) * stfm1000->desc_num,
			GFP_KERNEL);
	if (stfm1000->dma == NULL) {
		printk(KERN_ERR "%s: stmp3xxx_dma_request failed\n", __func__);
		ret = -ENOMEM;
		goto err_rel_dma;
	}

	for (i = 0; i < stfm1000->desc_num; i++) {
		ret = stmp3xxx_dma_allocate_command(stfm1000->dma_ch,
				&stfm1000->dma[i]);
		if (ret != 0) {
			printk(KERN_ERR "%s: stmp3xxx_dma_allocate_command "
				"failed\n", __func__);
			goto err_free_dma;
		}
	}

	/* allocate ALSA card structure (we only need an extra pointer
	 * back to stfm1000) */
	card = snd_card_new(-1, NULL, THIS_MODULE, 0);
	if (card == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "%s: snd_card_new failed\n", __func__);
		goto err_free_dma;
	}
	stfm1000->card = card;
	card->private_data = stfm1000;	/* point back */

	/* mixer controls */
	strcpy(card->driver, "stfm1000");
	card->private_free = stfm1000_snd_free;

	strcpy(card->mixername, "stfm1000 mixer");
	for (i = 0; i < ARRAY_SIZE(stfm1000_snd_controls); i++) {
		ctl = snd_ctl_new1(&stfm1000_snd_controls[i], stfm1000);
		if (ctl == NULL) {
			printk(KERN_ERR "%s: snd_ctl_new1 failed\n", __func__);
			goto err_free_controls;
		}
		ret = snd_ctl_add(card, ctl);
		if (ret != 0) {
			printk(KERN_ERR "%s: snd_ctl_add failed\n", __func__);
			goto err_free_controls;
		}
	}

	/* PCM */
	ret = snd_pcm_new(card, "STFM1000 PCM", 0, 0, 1, &pcm);
	if (ret != 0) {
		printk(KERN_ERR "%s: snd_ctl_add failed\n", __func__);
		goto err_free_controls;
	}
	stfm1000->pcm = pcm;
	pcm->private_data = stfm1000;	/* point back */

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
		&stfm1000_snd_capture_ops);
	pcm->info_flags = 0;
	strcpy(pcm->name, "STFM1000 PCM");

	snd_card_set_dev(card, &stfm1000->radio.dev);
	strcpy(card->shortname, "STFM1000");

	ret = snd_pcm_lib_preallocate_pages_for_all(stfm1000->pcm,
		SNDRV_DMA_TYPE_CONTINUOUS, card->dev, SZ_256K, SZ_256K);
	if (ret != 0) {
		printk(KERN_ERR "%s: snd_pcm_lib_preallocate_pages_for_all "
			"failed\n", __func__);
		goto err_free_pcm;
	}

	ret = request_irq(IRQ_DRI_DMA, stfm1000_dri_dma_irq, 0, "stfm1000",
		stfm1000);
	if (ret != 0) {
		printk(KERN_ERR "%s: request_irq failed\n", __func__);
		goto err_free_prealloc;
	}

	ret = request_irq(IRQ_DRI_ATTENTION, stfm1000_dri_attn_irq, 0,
		"stfm1000", stfm1000);
	if (ret != 0) {
		printk(KERN_ERR "%s: request_irq failed\n", __func__);
		goto err_rel_irq;
	}

	ret = snd_card_register(stfm1000->card);
	if (ret != 0) {
		printk(KERN_ERR "%s: snd_card_register failed\n", __func__);
		goto err_rel_irq2;
	}

	/* Enable completion interrupt */
	stmp3xxx_dma_clear_interrupt(stfm1000->dma_ch);
	stmp3xxx_dma_enable_interrupt(stfm1000->dma_ch);

	printk(KERN_INFO "%s/alsa: %s registered\n", "STFM1000",
		card->longname);

	return 0;

err_rel_irq2:
	free_irq(IRQ_DRI_ATTENTION, stfm1000);

err_rel_irq:
	free_irq(IRQ_DRI_DMA, stfm1000);

err_free_prealloc:
	snd_pcm_lib_preallocate_free_for_all(stfm1000->pcm);

err_free_pcm:
	/* XXX TODO */

err_free_controls:
	/* XXX TODO */

/* err_free_card: */
	snd_card_free(stfm1000->card);

err_free_dma:
	for (i = stfm1000->desc_num - 1; i >= 0; i--) {
		if (stfm1000->dma[i].command != NULL)
			stmp3xxx_dma_free_command(stfm1000->dma_ch,
				&stfm1000->dma[i]);
	}

err_rel_dma:
	stmp3xxx_dma_release(stfm1000->dma_ch);
err:
	return ret;
}

static void stfm1000_alsa_instance_release(struct stfm1000 *stfm1000)
{
	int i;

	stmp3xxx_dma_clear_interrupt(stfm1000->dma_ch);
	stmp3xxx_arch_dma_reset_channel(stfm1000->dma_ch);

	snd_card_free(stfm1000->card);

	for (i = stfm1000->desc_num - 1; i >= 0; i--)
		stmp3xxx_dma_free_command(stfm1000->dma_ch, &stfm1000->dma[i]);

	kfree(stfm1000->dma);

	stmp3xxx_dma_release(stfm1000->dma_ch);
}

static void stfm1000_alsa_dma_irq(struct stfm1000 *stfm1000)
{
	struct snd_pcm_runtime *runtime;
	int desc;
	s16 *src, *dst;

	if (stfm1000->stopping_recording)
		return;

	if (stfm1000->read_count >= stfm1000->blksize *
			(stfm1000->blocks - 2)) {
		printk(KERN_ERR "irq: overrun %d - Blocks in %d\n",
			stfm1000->read_count, stfm1000->blocks);
		return;
	}

	/* someone has brutally killed user-space */
	if (stfm1000->substream == NULL ||
			stfm1000->substream->runtime == NULL)
		return;

	BUG_ON(stfm1000->substream == NULL);
	BUG_ON(stfm1000->substream->runtime == NULL);

	desc = stfm1000->read_offset / stfm1000->blksize;
	runtime = stfm1000->substream->runtime;

	if (runtime->dma_area == NULL)
		printk(KERN_INFO "runtime->dma_area = NULL\n");
	BUG_ON(runtime->dma_area == NULL);
	if (stfm1000->dri_buf == NULL)
		printk(KERN_INFO "stfm1000->dri_buf = NULL\n");
	BUG_ON(stfm1000->dri_buf == NULL);

	if (desc >= stfm1000->blocks) {
		printk(KERN_INFO "desc=%d ->blocks=%d\n",
				desc, stfm1000->blocks);
		printk(KERN_INFO "->read_offset=%x ->blksize=%x\n",
				stfm1000->read_offset, stfm1000->blksize);
	}
	BUG_ON(desc >= stfm1000->blocks);

	src = stfm1000->dri_buf + desc * (stfm1000->blksize * 2);
	dst = (void *)runtime->dma_area + desc * stfm1000->blksize;

	/* perform filtering */
	stfm1000_decode_block(stfm1000, src, dst, stfm1000->blksize / 4);

	stfm1000->read_count += stfm1000->blksize;

	if (stfm1000->read_count >=
		snd_pcm_lib_period_bytes(stfm1000->substream))
		snd_pcm_period_elapsed(stfm1000->substream);
}

static void stfm1000_alsa_attn_irq(struct stfm1000 *stfm1000)
{
	/* nothing */
}

struct stfm1000_alsa_ops stfm1000_default_alsa_ops = {
	.init = stfm1000_alsa_instance_init,
	.release = stfm1000_alsa_instance_release,
	.dma_irq = stfm1000_alsa_dma_irq,
	.attn_irq = stfm1000_alsa_attn_irq,
};

static int stfm1000_alsa_init(void)
{
	struct stfm1000 *stfm1000 = NULL;
	struct list_head *list;
	int ret;

	stfm1000_alsa_ops = &stfm1000_default_alsa_ops;

	list_for_each(list, &stfm1000_devlist) {
		stfm1000 = list_entry(list, struct stfm1000, devlist);
		ret = (*stfm1000_alsa_ops->init)(stfm1000);
		if (ret != 0) {
			printk(KERN_ERR "stfm1000 ALSA driver for DMA sound "
				"failed init.\n");
			return ret;
		}
		stfm1000->alsa_initialized = 1;
	}

	printk(KERN_INFO "stfm1000 ALSA driver for DMA sound loaded\n");

	return 0;
}

static void stfm1000_alsa_exit(void)
{
	struct stfm1000 *stfm1000 = NULL;
	struct list_head *list;

	list_for_each(list, &stfm1000_devlist) {
		stfm1000 = list_entry(list, struct stfm1000, devlist);

		if (!stfm1000->alsa_initialized)
			continue;

		stfm1000_take_down(stfm1000);
		(*stfm1000_alsa_ops->release)(stfm1000);
		stfm1000->alsa_initialized = 0;
	}

	printk(KERN_INFO "stfm1000 ALSA driver for DMA sound unloaded\n");
}

/* We initialize this late, to make sure the sound system is up and running */
late_initcall(stfm1000_alsa_init);
module_exit(stfm1000_alsa_exit);

MODULE_AUTHOR("Pantelis Antoniou");
MODULE_DESCRIPTION("An ALSA PCM driver for the STFM1000 chip.");
MODULE_LICENSE("GPL");
