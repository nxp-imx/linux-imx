/*
 * imx-hdmi-dma.c  --  HDMI DMA driver for ALSA Soc Audio Layer
 *
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
 *
 * based on imx-pcm-dma-mx2.c
 * Copyright 2009 Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This code is based on code copyrighted by Freescale,
 * Liam Girdwood, Javier Martin and probably others.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <linux/mfd/mxc-hdmi-core.h>
#include <mach/mxc_hdmi.h>
#include "imx-hdmi.h"

struct imx_hdmi_dma_runtime_data {
	struct snd_pcm_substream *tx_substream;

	unsigned long buffer_bytes;
	void *buf;
	int period_time;

	int periods;
	int period_bytes;
	int dma_period_bytes;
	int buffer_ratio;

	unsigned long offset;

	int channels;
	snd_pcm_format_t format;
	int rate;
	int sample_align;
	int sample_bits;

	int frame_idx;

	int irq;
	struct clk *isfr_clk;
	struct clk *iahb_clk;

	bool tx_active;
	spinlock_t irq_lock;
};

hdmi_audio_header_t iec_header;

/*
 * Note that the period size for DMA != period size for ALSA because the
 * driver adds iec frame info to the audio samples (in hdmi_dma_copy).
 *
 * Each 4 byte subframe = 1 byte of iec data + 3 byte audio sample.
 *
 * A 16 bit audio sample becomes 32 bits including the frame info. Ratio=2
 * A 24 bit audio sample becomes 32 bits including the frame info. Ratio=3:4
 * If the 24 bit raw audio is in 32 bit words, the
 *
 *  Original  Packed into  subframe  Ratio of size        Format
 *   sample    how many      size    of DMA buffer
 *   (bits)      bits                to ALSA buffer
 *  --------  -----------  --------  --------------  ------------------------
 *     16         16          32          2          SNDRV_PCM_FORMAT_S16_LE
 *     24         24          32          1.33       SNDRV_PCM_FORMAT_S24_3LE*
 *     24         32          32          1          SNDRV_PCM_FORMAT_S24_LE
 *
 * *so SNDRV_PCM_FORMAT_S24_3LE is not supported.
 */

/*
 * The minimum dma period is one IEC audio frame (192 * 4 * channels).
 * The maximum dma period for the HDMI DMA is 8K.
 *
 *   channels       minimum          maximum
 *                 dma period       dma period
 *   --------  ------------------   ----------
 *       2     192 * 4 * 2 = 1536   * 4 = 6144
 *       4     192 * 4 * 4 = 3072   * 2 = 6144
 *       6     192 * 4 * 6 = 4608   * 1 = 4608
 *       8     192 * 4 * 8 = 6144   * 1 = 6144
 *
 * Bottom line:
 * 1. Must keep the ratio of DMA buffer to ALSA buffer consistent.
 * 2. frame_idx is saved in the private data, so even if a frame cannot be
 *    transmitted in a period, it can be continued in the next period.  This
 *    is necessary for 6 ch.
 */
#define HDMI_DMA_PERIOD_BYTES		(6144)
#define HDMI_DMA_BUF_SIZE		(64 * 1024)

struct imx_hdmi_dma_runtime_data *hdmi_dma_priv;

#ifdef DEBUG
static void dumpregs(void)
{
	pr_debug("\n");
	pr_debug("HDMI_AHB_DMA_CONF0           0x%02x\n", hdmi_readb(HDMI_AHB_DMA_CONF0));
	pr_debug("HDMI_AHB_DMA_START           0x%02x\n", hdmi_readb(HDMI_AHB_DMA_START));
	pr_debug("HDMI_AHB_DMA_STOP            0x%02x\n", hdmi_readb(HDMI_AHB_DMA_STOP));
	pr_debug("HDMI_AHB_DMA_THRSLD          0x%02x\n", hdmi_readb(HDMI_AHB_DMA_THRSLD));
	pr_debug("HDMI_AHB_DMA_STRADDR[0-3]    0x%08x\n", hdmi_read4(HDMI_AHB_DMA_STRADDR0));
	pr_debug("HDMI_AHB_DMA_STPADDR[0-3]    0x%08x\n", hdmi_read4(HDMI_AHB_DMA_STPADDR0));
	pr_debug("HDMI_AHB_DMA_BSTADDR[0-3]    0x%08x\n", hdmi_read4(HDMI_AHB_DMA_BSTADDR0));
	pr_debug("HDMI_AHB_DMA_MBLENGTH0       0x%02x\n", hdmi_readb(HDMI_AHB_DMA_MBLENGTH0));
	pr_debug("HDMI_AHB_DMA_MBLENGTH1       0x%02x\n", hdmi_readb(HDMI_AHB_DMA_MBLENGTH1));
	pr_debug("HDMI_AHB_DMA_STAT            0x%02x\n", hdmi_readb(HDMI_AHB_DMA_STAT));
	pr_debug("HDMI_AHB_DMA_INT             0x%02x\n", hdmi_readb(HDMI_AHB_DMA_INT));
	pr_debug("HDMI_AHB_DMA_MASK            0x%02x\n", hdmi_readb(HDMI_AHB_DMA_MASK));
	pr_debug("HDMI_AHB_DMA_POL             0x%02x\n", hdmi_readb(HDMI_AHB_DMA_POL));
	pr_debug("HDMI_AHB_DMA_CONF1           0x%02x\n", hdmi_readb(HDMI_AHB_DMA_CONF1));
	pr_debug("HDMI_AHB_DMA_BUFFSTAT        0x%02x\n", hdmi_readb(HDMI_AHB_DMA_BUFFSTAT));
	pr_debug("HDMI_AHB_DMA_BUFFINT         0x%02x\n", hdmi_readb(HDMI_AHB_DMA_BUFFINT));
	pr_debug("HDMI_AHB_DMA_BUFFMASK        0x%02x\n", hdmi_readb(HDMI_AHB_DMA_BUFFMASK));
	pr_debug("HDMI_AHB_DMA_BUFFPOL         0x%02x\n", hdmi_readb(HDMI_AHB_DMA_BUFFPOL));
	pr_debug("HDMI_IH_MUTE_AHBDMAAUD_STAT0 0x%02x\n", hdmi_readb(HDMI_IH_MUTE_AHBDMAAUD_STAT0));
	pr_debug("HDMI_IH_AHBDMAAUD_STAT0      0x%02x\n", hdmi_readb(HDMI_IH_AHBDMAAUD_STAT0));
	pr_debug("HDMI_IH_MUTE                 0x%02x\n", hdmi_readb(HDMI_IH_MUTE));
	pr_debug("\n");
}

static void dumprtd(struct imx_hdmi_dma_runtime_data *rtd)
{
	pr_debug("\n");
	pr_debug("channels         = %d\n", rtd->channels);
	pr_debug("periods          = %d\n", rtd->periods);
	pr_debug("period_bytes     = %d\n", rtd->period_bytes);
	pr_debug("dma period_bytes = %d\n", rtd->dma_period_bytes);
	pr_debug("buffer_ratio     = %d\n", rtd->buffer_ratio);
	pr_debug("dma buf addr     = 0x%08x\n", (int)rtd->buf);
	pr_debug("dma buf size     = %d\n", (int)rtd->buffer_bytes);
	pr_debug("sample_rate      = %d\n", (int)rtd->rate);
}
#else
static void dumpregs(void)
{
}

static void dumprtd(struct imx_hdmi_dma_runtime_data *rtd)
{
}
#endif

static void hdmi_dma_start(void)
{
	hdmi_mask_writeb(1, HDMI_AHB_DMA_START,
			HDMI_AHB_DMA_START_START_OFFSET,
			HDMI_AHB_DMA_START_START_MASK);
}

static void hdmi_dma_stop(void)
{
	hdmi_mask_writeb(1, HDMI_AHB_DMA_STOP,
			HDMI_AHB_DMA_STOP_STOP_OFFSET,
			HDMI_AHB_DMA_STOP_STOP_MASK);
}

static void hdmi_fifo_reset(void)
{
	hdmi_mask_writeb(1, HDMI_AHB_DMA_CONF0,
			 HDMI_AHB_DMA_CONF0_SW_FIFO_RST_OFFSET,
			 HDMI_AHB_DMA_CONF0_SW_FIFO_RST_MASK);
}

/*
 * Conditions for DMA to work:
 * ((final_addr - initial_addr)>>2)+1) < 2k.  So max period is 8k.
 * (inital_addr & 0x3) == 0
 * (final_addr  & 0x3) == 0x3
 *
 * The DMA Period should be an integer multiple of the IEC 60958 audio
 * frame size, which is 768 bytes (192 * 4).
 */
static void hdmi_dma_set_addr(int start_addr, int dma_period_bytes)
{
	int final_addr = start_addr + dma_period_bytes - 1;

	hdmi_write4(start_addr, HDMI_AHB_DMA_STRADDR0);
	hdmi_write4(final_addr, HDMI_AHB_DMA_STPADDR0);
}

static u8 hdmi_dma_get_irq_status(void)
{
	return hdmi_readb(HDMI_IH_AHBDMAAUD_STAT0);
}

static void hdmi_dma_clear_irq_status(u8 status)
{
	hdmi_writeb(status, HDMI_IH_AHBDMAAUD_STAT0);
}

static void hdmi_dma_irq_mask(int mask)
{
	if (mask)
		hdmi_writeb(0xff, HDMI_AHB_DMA_MASK);
	else
		hdmi_writeb((u8)~HDMI_AHB_DMA_DONE, HDMI_AHB_DMA_MASK);
}

static void hdmi_dma_irq_mute(int mute)
{
	if (mute)
		hdmi_writeb(0xff, HDMI_IH_MUTE_AHBDMAAUD_STAT0);
	else
		hdmi_writeb(0x00, HDMI_IH_MUTE_AHBDMAAUD_STAT0);
}

/* Add frame information for one pcm subframe */
static u32 hdmi_dma_add_frame_info(struct imx_hdmi_dma_runtime_data *rtd,
				   u32 pcm_data, int subframe_idx)
{
	hdmi_audio_dma_data_t subframe;
	int i;

	subframe.U = 0;
	iec_header.B.channel = subframe_idx;

	/* fill b (start-of-block) */
	subframe.B.b = (rtd->frame_idx == 0) ? 1 : 0;

	/* fill c (channel status) */
	if (rtd->frame_idx < 42)
		subframe.B.c = (iec_header.U >> rtd->frame_idx) & 0x1;
	else
		subframe.B.c = 0;

	/* fill p (parity) */
	for (i = 0 ; i < rtd->sample_bits ; i++)
		subframe.B.p ^= (pcm_data >> i) & 0x01;
	subframe.B.p ^= subframe.B.c;
	subframe.B.p ^= subframe.B.u;
	subframe.B.p ^= subframe.B.v;

	/* fill data */
	if (rtd->sample_bits == 16)
		subframe.B.data = pcm_data << 8;
	else
		subframe.B.data = pcm_data;

	return subframe.U;
}

/* Increment the frame index.  We save frame_idx in case a frame
 * spans more than one dma period. */
static void hdmi_dma_incr_frame_idx(struct imx_hdmi_dma_runtime_data *rtd)
{
	rtd->frame_idx++;
	if (rtd->frame_idx == 192)
		rtd->frame_idx = 0;
}

static irqreturn_t hdmi_dma_isr(int irq, void *dev_id)
{
	struct imx_hdmi_dma_runtime_data *rtd = dev_id;
	struct snd_pcm_substream *substream = rtd->tx_substream;
	unsigned int status;
	unsigned long flags;

	spin_lock_irqsave(&rtd->irq_lock, flags);

	hdmi_dma_irq_mute(1);
	status = hdmi_dma_get_irq_status();
	hdmi_dma_clear_irq_status(status);

	if (rtd->tx_active && (status & HDMI_IH_AHBDMAAUD_STAT0_DONE)) {
		rtd->offset += rtd->period_bytes;
		rtd->offset %= rtd->period_bytes * rtd->periods;

		snd_pcm_period_elapsed(substream);

		hdmi_dma_set_addr(substream->dma_buffer.addr +
				  (rtd->offset * rtd->buffer_ratio),
				rtd->dma_period_bytes);

		hdmi_dma_start();
	}

	hdmi_dma_irq_mute(0);

	spin_unlock_irqrestore(&rtd->irq_lock, flags);
	return IRQ_HANDLED;
}

static void hdmi_dma_enable_hlock(int enable)
{
	hdmi_mask_writeb(enable, HDMI_AHB_DMA_CONF0,
			 HDMI_AHB_DMA_CONF0_EN_HLOCK_OFFSET,
			 HDMI_AHB_DMA_CONF0_EN_HLOCK_MASK);
}

static void hdmi_dma_set_incr_type(int incr_type)
{
	u8 value = hdmi_readb(HDMI_AHB_DMA_CONF0) &
			~(HDMI_AHB_DMA_CONF0_BURST_MODE |
			  HDMI_AHB_DMA_CONF0_INCR_TYPE_MASK);

	switch (incr_type) {
	case 1:
		break;
	case 4:
		value |= HDMI_AHB_DMA_CONF0_BURST_MODE;
		break;
	case 8:
		value |= HDMI_AHB_DMA_CONF0_BURST_MODE |
			 HDMI_AHB_DMA_CONF0_INCR8;
		break;
	case 16:
		value |= HDMI_AHB_DMA_CONF0_BURST_MODE |
			 HDMI_AHB_DMA_CONF0_INCR16;
		break;
	default:
		pr_err("%s: invalid increment type: %d", __func__, incr_type);
		BUG();
		return;
	}

	hdmi_writeb(value, HDMI_AHB_DMA_CONF0);
}

static void hdmi_dma_enable_channels(int channels)
{
	switch (channels) {
	case 2:
		hdmi_writeb(0x03, HDMI_AHB_DMA_CONF1);
		break;
	case 4:
		hdmi_writeb(0x0f, HDMI_AHB_DMA_CONF1);
		break;
	case 6:
		hdmi_writeb(0x3f, HDMI_AHB_DMA_CONF1);
		break;
	case 8:
		hdmi_writeb(0xff, HDMI_AHB_DMA_CONF1);
		break;
	default:
		WARN(1, "%s - invalid audio channels number: %d\n",
			__func__, channels);
		break;
	}
}

static void hdmi_dma_configure_dma(int channels)
{
	hdmi_dma_enable_hlock(1);
	hdmi_dma_set_incr_type(4);
	hdmi_writeb(64, HDMI_AHB_DMA_THRSLD);
	hdmi_dma_enable_channels(channels);
}


static void hdmi_dma_init_iec_header(void)
{
	iec_header.U = 0;

	iec_header.B.consumer = 0;		/* Consumer use */
	iec_header.B.linear_pcm = 0;		/* linear pcm audio */
	iec_header.B.copyright = 1;		/* no copyright */
	iec_header.B.pre_emphasis = 0;		/* 2 channels without pre-emphasis */
	iec_header.B.mode = 0;			/* Mode 0 */

	iec_header.B.category_code = 0;

	iec_header.B.source = 2;		/* stereo */
	iec_header.B.channel = 0;

	iec_header.B.sample_freq = 0x02;	/* 48 KHz */
	iec_header.B.clock_acc = 0;		/* Level II */

	iec_header.B.word_length = 0x02;	/* 16 bits */
	iec_header.B.org_sample_freq = 0x0D;	/* 48 KHz */

	iec_header.B.cgms_a = 0;	/* Copying is permitted without restriction */
}

static int hdmi_dma_update_iec_header(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_hdmi_dma_runtime_data *rtd = runtime->private_data;

	iec_header.B.source = rtd->channels;

	switch (rtd->rate) {
	case 32000:
		iec_header.B.sample_freq = 0x03;
		iec_header.B.org_sample_freq = 0x0C;
		break;
	case 44100:
		iec_header.B.sample_freq = 0x00;
		iec_header.B.org_sample_freq = 0x0F;
		break;
	case 48000:
		iec_header.B.sample_freq = 0x02;
		iec_header.B.org_sample_freq = 0x0D;
		break;
	case 88200:
		iec_header.B.sample_freq = 0x08;
		iec_header.B.org_sample_freq = 0x07;
		break;
	case 96000:
		iec_header.B.sample_freq = 0x0A;
		iec_header.B.org_sample_freq = 0x05;
		break;
	case 176400:
		iec_header.B.sample_freq = 0x0C;
		iec_header.B.org_sample_freq = 0x03;
		break;
	case 192000:
		iec_header.B.sample_freq = 0x0E;
		iec_header.B.org_sample_freq = 0x01;
		break;
	default:
		pr_err("HDMI Audio sample rate error");
		return -EFAULT;
	}

	switch (rtd->format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		iec_header.B.word_length = 0x02;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iec_header.B.word_length = 0x0b;
		break;
	default:
		return -EFAULT;
	}

	return 0;
}

/*
 * The HDMI block transmits the audio data without adding any of the audio
 * frame bits.  So we have to copy the raw dma data from the ALSA buffer
 * to the DMA buffer, adding the frame information.
 */
static int hdmi_dma_copy(struct snd_pcm_substream *substream, int channel,
			snd_pcm_uframes_t pos, void __user *buf,
			snd_pcm_uframes_t frames)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_hdmi_dma_runtime_data *rtd = runtime->private_data;
	unsigned int count = frames_to_bytes(runtime, frames);
	unsigned int pos_bytes = frames_to_bytes(runtime, pos);
	u32 *hw_buf;
	int subframe_idx;
	u32 pcm_data;

	/* Copy pcm data from userspace and add frame info. */
	hw_buf = (u32 *)(rtd->buf + (pos_bytes * rtd->buffer_ratio));

	while (count > 0) {
		for (subframe_idx = 1 ; subframe_idx <= rtd->channels ; subframe_idx++) {

			if (copy_from_user(&pcm_data, buf, rtd->sample_align))
				return -EFAULT;

			buf += rtd->sample_align;
			count -= rtd->sample_align;

			/* Save the header info to the audio dma buffer */
			*hw_buf++ = hdmi_dma_add_frame_info(rtd, pcm_data, subframe_idx);
		}
		hdmi_dma_incr_frame_idx(rtd);
	}

	return 0;
}

static int hdmi_dma_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_hdmi_dma_runtime_data *rtd = runtime->private_data;

	rtd->buffer_bytes = params_buffer_bytes(params);
	rtd->periods = params_periods(params);
	rtd->period_bytes = params_period_bytes(params);
	rtd->channels = params_channels(params);
	rtd->format = params_format(params);
	rtd->rate = params_rate(params);

	rtd->offset = 0;
	rtd->period_time = HZ / (params_rate(params) / params_period_size(params));
	rtd->buf = (unsigned int *)substream->dma_buffer.area;

	switch (rtd->format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		rtd->buffer_ratio = 2;
		rtd->sample_align = 2;
		rtd->sample_bits = 16;
		break;

	case SNDRV_PCM_FORMAT_S24_LE:	/* 24 bit audio in 32 bit word */
		rtd->buffer_ratio = 1;
		rtd->sample_align = 4;
		rtd->sample_bits = 24;
		break;
	default:
		pr_err("%s HDMI Audio invalid sample format (%d)\n",
			__func__, rtd->format);
		return -EINVAL;
	}

	rtd->dma_period_bytes = rtd->period_bytes * rtd->buffer_ratio;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	hdmi_dma_configure_dma(rtd->channels);
	hdmi_dma_set_addr(substream->dma_buffer.addr, rtd->dma_period_bytes);

	dumprtd(rtd);

	hdmi_dma_update_iec_header(substream);

	return 0;
}

static int hdmi_dma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_hdmi_dma_runtime_data *rtd = runtime->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		rtd->frame_idx = 0;
		dumpregs();
		hdmi_dma_irq_mask(0);
		hdmi_dma_priv->tx_active = true;
		hdmi_dma_start();
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		hdmi_dma_priv->tx_active = false;
		hdmi_dma_stop();
		hdmi_dma_irq_mask(1);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t hdmi_dma_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_hdmi_dma_runtime_data *rtd = runtime->private_data;

	return bytes_to_frames(substream->runtime, rtd->offset);
}

static struct snd_pcm_hardware snd_imx_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME,
	.formats = MXC_HDMI_FORMATS_PLAYBACK,
	.rate_min = 32000,
	.channels_min = 2,
	.channels_max = 8,
	.buffer_bytes_max = HDMI_DMA_BUF_SIZE / 2,
	.period_bytes_min = HDMI_DMA_PERIOD_BYTES / 2,
	.period_bytes_max = HDMI_DMA_PERIOD_BYTES / 2,
	.periods_min = 4,
	.periods_max = 255,
	.fifo_size = 0,
};

static void hdmi_dma_irq_enable(struct imx_hdmi_dma_runtime_data *rtd)
{
	unsigned long flags;

	hdmi_writeb(0xff, HDMI_AHB_DMA_POL);
	hdmi_writeb(0xff, HDMI_AHB_DMA_BUFFPOL);

	spin_lock_irqsave(&hdmi_dma_priv->irq_lock, flags);

	hdmi_dma_clear_irq_status(0xff);
	hdmi_irq_enable(hdmi_dma_priv->irq);
	hdmi_dma_irq_mute(0);
	hdmi_dma_irq_mask(0);

	spin_unlock_irqrestore(&hdmi_dma_priv->irq_lock, flags);

}

static void hdmi_dma_irq_disable(struct imx_hdmi_dma_runtime_data *rtd)
{
	unsigned long flags;

	spin_lock_irqsave(&rtd->irq_lock, flags);

	hdmi_dma_irq_mask(1);
	hdmi_dma_irq_mute(1);
	hdmi_irq_disable(rtd->irq);
	hdmi_dma_clear_irq_status(0xff);

	spin_unlock_irqrestore(&rtd->irq_lock, flags);
}

static int hdmi_dma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	runtime->private_data = hdmi_dma_priv;

	clk_enable(hdmi_dma_priv->isfr_clk);
	clk_enable(hdmi_dma_priv->iahb_clk);

	pr_debug("%s hdmi clks: isfr:%d iahb:%d\n", __func__,
		(int)clk_get_rate(hdmi_dma_priv->isfr_clk),
		(int)clk_get_rate(hdmi_dma_priv->iahb_clk));

	hdmi_fifo_reset();

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	snd_soc_set_runtime_hwparams(substream, &snd_imx_hardware);

	hdmi_dma_irq_enable(hdmi_dma_priv);

	return 0;
}

static int hdmi_dma_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_hdmi_dma_runtime_data *rtd = runtime->private_data;

	hdmi_dma_irq_disable(rtd);

	clk_disable(rtd->iahb_clk);
	clk_disable(rtd->isfr_clk);

	return 0;
}

static struct snd_pcm_ops imx_hdmi_dma_pcm_ops = {
	.open		= hdmi_dma_open,
	.close		= hdmi_dma_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= hdmi_dma_hw_params,
	.trigger	= hdmi_dma_trigger,
	.pointer	= hdmi_dma_pointer,
	.copy		= hdmi_dma_copy,
};

static int imx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = HDMI_DMA_BUF_SIZE;

	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->bytes = size;

	hdmi_dma_priv->tx_substream = substream;

	return 0;
}

static u64 hdmi_dmamask = DMA_BIT_MASK(32);

static int imx_hdmi_dma_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
				struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &hdmi_dmamask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	ret = imx_pcm_preallocate_dma_buffer(pcm,
		SNDRV_PCM_STREAM_PLAYBACK);
	if (ret)
		goto out;
out:
	return ret;
}

static void imx_hdmi_dma_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

static struct snd_soc_platform_driver imx_soc_platform_mx2 = {
	.ops		= &imx_hdmi_dma_pcm_ops,
	.pcm_new	= imx_hdmi_dma_pcm_new,
	.pcm_free	= imx_hdmi_dma_pcm_free,
};

static int __devinit imx_soc_platform_probe(struct platform_device *pdev)
{
	struct imx_hdmi *hdmi_drvdata = platform_get_drvdata(pdev);
	int ret = 0;

	hdmi_dma_priv = kzalloc(sizeof(*hdmi_dma_priv), GFP_KERNEL);
	if (hdmi_dma_priv == NULL)
		return -ENOMEM;

	hdmi_dma_priv->tx_active = false;
	spin_lock_init(&hdmi_dma_priv->irq_lock);
	hdmi_dma_priv->irq = hdmi_drvdata->irq;

	hdmi_dma_init_iec_header();

	/* Initialize IRQ at HDMI core level */
	hdmi_irq_init();

	hdmi_dma_priv->isfr_clk = clk_get(&pdev->dev, "hdmi_isfr_clk");
	if (IS_ERR(hdmi_dma_priv->isfr_clk)) {
		ret = PTR_ERR(hdmi_dma_priv->isfr_clk);
		dev_err(&pdev->dev, "Unable to get HDMI isfr clk: %d\n", ret);
		goto e_clk_get1;
	}

	hdmi_dma_priv->iahb_clk = clk_get(&pdev->dev, "hdmi_iahb_clk");
	if (IS_ERR(hdmi_dma_priv->iahb_clk)) {
		ret = PTR_ERR(hdmi_dma_priv->iahb_clk);
		dev_err(&pdev->dev, "Unable to get HDMI ahb clk: %d\n", ret);
		goto e_clk_get2;
	}

	/* The HDMI block's irq line is shared with HDMI video. */
	if (request_irq(hdmi_dma_priv->irq, hdmi_dma_isr, IRQF_SHARED,
			"hdmi dma", hdmi_dma_priv)) {
		dev_err(&pdev->dev, "MXC hdmi: failed to request irq %d\n",
		       hdmi_dma_priv->irq);
		ret = -EBUSY;
		goto e_irq;
	}

	ret = snd_soc_register_platform(&pdev->dev, &imx_soc_platform_mx2);
	if (ret)
		goto e_irq;

	return 0;

e_irq:
	clk_put(hdmi_dma_priv->iahb_clk);
e_clk_get2:
	clk_put(hdmi_dma_priv->isfr_clk);
e_clk_get1:
	kfree(hdmi_dma_priv);

	return ret;
}

static int __devexit imx_soc_platform_remove(struct platform_device *pdev)
{
	free_irq(hdmi_dma_priv->irq, hdmi_dma_priv);
	snd_soc_unregister_platform(&pdev->dev);
	kfree(hdmi_dma_priv);
	return 0;
}

static struct platform_driver imx_hdmi_dma_driver = {
	.driver = {
			.name = "imx-hdmi-soc-audio",
			.owner = THIS_MODULE,
	},
	.probe = imx_soc_platform_probe,
	.remove = __devexit_p(imx_soc_platform_remove),
};

static int __init hdmi_dma_init(void)
{
	return platform_driver_register(&imx_hdmi_dma_driver);
}
module_init(hdmi_dma_init);

static void __exit hdmi_dma_exit(void)
{
	platform_driver_unregister(&imx_hdmi_dma_driver);
}
module_exit(hdmi_dma_exit);
