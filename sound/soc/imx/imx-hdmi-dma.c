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
#include <linux/delay.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <linux/mfd/mxc-hdmi-core.h>
#include <mach/mxc_hdmi.h>
#include "imx-hdmi.h"

#include <linux/dmaengine.h>
#include <mach/dma.h>
#include <linux/iram_alloc.h>
#include <linux/io.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>

#define HDMI_DMA_BURST_UNSPECIFIED_LEGNTH	0
#define HDMI_DMA_BURST_INCR4			1
#define HDMI_DMA_BURST_INCR8			2
#define HDMI_DMA_BURST_INCR16			3



#define HDMI_BASE_ADDR 0x00120000

struct hdmi_sdma_script_data {
	int control_reg_addr;
	int status_reg_addr;
	int dma_start_addr;
	u32 buffer[20];
};

struct imx_hdmi_sdma_params {
	u32 buffer_num;
	dma_addr_t phyaddr;
};


struct imx_hdmi_dma_runtime_data {
	struct snd_pcm_substream *tx_substream;

	unsigned long buffer_bytes;
	struct snd_dma_buffer hw_buffer;
	unsigned long appl_bytes;
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

	/*SDMA part*/
	unsigned int dma_event;
	struct dma_chan *dma_channel;
	struct imx_dma_data dma_data;
	struct hdmi_sdma_script_data *hdmi_sdma_t;
	dma_addr_t phy_hdmi_sdma_t;
	struct dma_async_tx_descriptor *desc;
	struct imx_hdmi_sdma_params sdma_params;
};

/* bit 0:0:0:b:p(0):c:(u)0:(v)0*/
/* max 8 channels supported; channels are interleaved*/
static unsigned char g_packet_head_table[48*8];

void hdmi_dma_copy_16_neon_lut(unsigned short *src, unsigned int *dst,
			int samples, unsigned char *lookup_table);
void hdmi_dma_copy_16_neon_fast(unsigned short *src, unsigned int *dst,
			int samples);
void hdmi_dma_copy_24_neon_lut(unsigned int *src, unsigned int *dst,
			int samples, unsigned char *lookup_table);
void hdmi_dma_copy_24_neon_fast(unsigned int *src, unsigned int *dst,
			int samples);

hdmi_audio_header_t iec_header;
EXPORT_SYMBOL(iec_header);

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
#define HDMI_PCM_BUF_SIZE		(64 * 1024)

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
	pr_debug("hw dma buffer    = 0x%08x\n", (int)rtd->hw_buffer.addr);
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
	u8 regvalue;
	regvalue = hdmi_readb(HDMI_AHB_DMA_MASK);

	if (mask) {
		regvalue |= HDMI_AHB_DMA_DONE;
		hdmi_writeb(regvalue, HDMI_AHB_DMA_MASK);
	} else {
		regvalue &= (u8)~HDMI_AHB_DMA_DONE;
		hdmi_writeb(regvalue, HDMI_AHB_DMA_MASK);
	}
}

static void hdmi_mask(int mask)
{
	u8 regvalue;
	regvalue = hdmi_readb(HDMI_AHB_DMA_MASK);

	if (mask) {
		regvalue |= HDMI_AHB_DMA_ERROR | HDMI_AHB_DMA_FIFO_EMPTY;
		hdmi_writeb(regvalue, HDMI_AHB_DMA_MASK);
	} else {
		regvalue &= (u8)~(HDMI_AHB_DMA_ERROR | HDMI_AHB_DMA_FIFO_EMPTY);
		hdmi_writeb(regvalue, HDMI_AHB_DMA_MASK);
	}
}


static void hdmi_dma_irq_mute(int mute)
{
	if (mute)
		hdmi_writeb(0xff, HDMI_IH_MUTE_AHBDMAAUD_STAT0);
	else
		hdmi_writeb(0x00, HDMI_IH_MUTE_AHBDMAAUD_STAT0);
}

int odd_ones(unsigned a)
{
	a ^= a >> 8;
	a ^= a >> 4;
	a ^= a >> 2;
	a ^= a >> 1;

	return a & 1;
}

/* Add frame information for one pcm subframe */
static u32 hdmi_dma_add_frame_info(struct imx_hdmi_dma_runtime_data *rtd,
				   u32 pcm_data, int subframe_idx)
{
	hdmi_audio_dma_data_t subframe;

	subframe.U = 0;
	iec_header.B.channel = subframe_idx;

	/* fill b (start-of-block) */
	subframe.B.b = (rtd->frame_idx == 0) ? 1 : 0;

	/* fill c (channel status) */
	if (rtd->frame_idx < 42)
		subframe.B.c = (iec_header.U >> rtd->frame_idx) & 0x1;
	else
		subframe.B.c = 0;

	subframe.B.p = odd_ones(pcm_data);
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

static void init_table(int channels)
{
	int i;
	int ch = 0;
	unsigned char *p = g_packet_head_table;

	for (i = 0; i < 48; i++) {
		int b = 0;
		if (i == 0)
			b = 1;

		for (ch = 0; ch < channels; ch++) {
			int c = 0;
			if (i < 42) {
				iec_header.B.channel = ch+1;
				c = (iec_header.U >> i) & 0x1;
			}
			/* preset bit p as c */
			*p++ = (b << 4) | (c << 2) | (c << 3);
		}
	}
}

#if 1

/* C code optimization for IEC head */
static void hdmi_dma_copy_16_c_lut(unsigned short *src, unsigned int *dst, int samples, unsigned char *lookup_table)
{
	int i;
	unsigned int sample;
	unsigned int p;
	unsigned int head;

	for (i = 0; i < samples; i++) {
		/* get source sample */
		sample = *src++;

		/* xor every bit */
		p = sample ^ (sample >> 8);
		p ^= (p >> 4);
		p ^= (p >> 2);
		p ^= (p >> 1);
		p &= 1;	/* only want last bit */
		p <<= 3; /* bit p */

		/* get packet header */
		head = *lookup_table++;

		/* fix head */
		head ^= p;

		/* store */
		*dst++ = (head << 24) | (sample << 8);
	}
}

static void hdmi_dma_copy_16_c_fast(unsigned short *src, unsigned int *dst, int samples)
{
	int i;
	unsigned int sample;
	unsigned int p;

	for (i = 0; i < samples; i++) {
		/* get source sample */
		sample = *src++;

		/* xor every bit */
		p = sample ^ (sample >> 8);
		p ^= (p >> 4);
		p ^= (p >> 2);
		p ^= (p >> 1);
		p &= 1;	/* only want last bit */
		p <<= 3; /* bit p */

		/* store */
		*dst++ = (p << 24) | (sample << 8);
	}
}

static void hdmi_dma_copy_16(unsigned short *src, unsigned int *dest, int framecount, int channelcount)
{
	/* split input frames into 192-frame each */
	int count_in_192 = (framecount + 191) / 192;
	int i;

	for (i = 0; i < count_in_192; i++) {
		int count;
		int samples;

		/* handles frame index [0, 48) */
		count = (framecount < 48) ? framecount : 48;
		samples = count * channelcount;
		hdmi_dma_copy_16_c_lut(src, dest, samples, g_packet_head_table);
		framecount -= count;
		if (framecount == 0)
			break;

		src  += samples;
		dest += samples;

		/* handles frame index [48, 192) */
		count = (framecount < 192 - 48) ? framecount : 192 - 48;
		samples = count * channelcount;
		hdmi_dma_copy_16_c_fast(src, dest, samples);
		framecount -= count;
		src  += samples;
		dest += samples;
	}
}

#else

/* NEON optimization for IEC head*/

/* Convert pcm samples to iec samples suitable for HDMI transfer.
* PCM sample is 16 bits length.
* Frame index always starts from 0.
* Channel count can be 1, 2, 4, 6, or 8
* Sample count (frame_count * channel_count) is multipliable by 8.
*/
static void hdmi_dma_copy_16(u16 *src, u32 *dest, int framecount, int channelcount)
{
	/* split input frames into 192-frame each */
	int count_in_192 = (framecount + 191) / 192;
	int i;

	for (i = 0; i < count_in_192; i++) {
		int count;
		int samples;

		/* handles frame index [0, 48) */
		count = (framecount < 48) ? framecount : 48;
		samples = count * channelcount;
		hdmi_dma_copy_16_neon_lut(src, dest, samples, g_packet_head_table);
		framecount -= count;
		if (framecount == 0)
			break;

		src  += samples;
		dest += samples;

		/* handles frame index [48, 192) */
		count = (framecount < 192 - 48) ? framecount : 192 - 48;
		samples = count * channelcount;
		hdmi_dma_copy_16_neon_fast(src, dest, samples);
		framecount -= count;
		src  += samples;
		dest += samples;
	}
}

/* Convert pcm samples to iec samples suitable for HDMI transfer.
* PCM sample is 24 bits length.
* Frame index always starts from 0.
* Channel count can be 1, 2, 4, 6, or 8
* Sample count (frame_count * channel_count) is multipliable by 8.
*/
static void hdmi_dma_copy_24(u32 *src, u32 *dest, int framecount, int channelcount)
{
	/* split input frames into 192-frame each */
	int count_in_192 = (framecount + 191) / 192;
	int i;

	for (i = 0; i < count_in_192; i++) {
		int count;
		int samples;

		/* handles frame index [0, 48) */
		count = (framecount < 48) ? framecount : 48;
		samples = count * channelcount;
		hdmi_dma_copy_24_neon_lut(src, dest, samples, g_packet_head_table);
		framecount -= count;
		if (framecount == 0)
			break;

		src  += samples;
		dest += samples;

		/* handles frame index [48, 192) */
		count = (framecount < 192 - 48) ? framecount : 192 - 48;
		samples = count * channelcount;
		hdmi_dma_copy_24_neon_fast(src, dest, samples);
		framecount -= count;
		src  += samples;
		dest += samples;
	}
}
#endif

static void hdmi_dma_mmap_copy(struct snd_pcm_substream *substream,
				int offset, int count)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_hdmi_dma_runtime_data *rtd = runtime->private_data;
	u32 framecount;
/*	u32 *src32; */
	u32 *dest;
	u16 *src16;

	framecount =  count/(rtd->sample_align * rtd->channels);

	/* hw_buffer is the destination for pcm data plus frame info. */
	dest = (u32 *)(rtd->hw_buffer.area + (offset * rtd->buffer_ratio));

	switch (rtd->format) {

	case SNDRV_PCM_FORMAT_S16_LE:
		/* dma_buffer is the mmapped buffer we are copying pcm from. */
		src16 = (u16 *)(runtime->dma_area + offset);
		hdmi_dma_copy_16(src16, dest, framecount, rtd->channels);
		break;

/* 24bit not support now. */
/*
	case SNDRV_PCM_FORMAT_S24_LE:
		src32 = (u32 *)(runtime->dma_area + offset);
		hdmi_dma_copy_24(src32, dest, framecount, rtd->channels);
		break;
*/
	default:
		pr_err("%s HDMI Audio invalid sample format (%d)\n",
						__func__, rtd->format);
		return;
	}
}

#ifdef CONFIG_ARCH_MX6
static void hdmi_sdma_isr(void *data)
{
	struct imx_hdmi_dma_runtime_data *rtd = data;
	struct snd_pcm_substream *substream = rtd->tx_substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long offset, count, appl_bytes;
	unsigned long flags;

	spin_lock_irqsave(&rtd->irq_lock, flags);

	if (runtime && runtime->dma_area && rtd->tx_active) {

		rtd->offset += rtd->period_bytes;
		rtd->offset %= rtd->period_bytes * rtd->periods;

		/* For mmap access, need to copy data from dma_buffer
		 * to hw_buffer and add the frame info. */
		if (runtime->access == SNDRV_PCM_ACCESS_MMAP_INTERLEAVED) {
			appl_bytes = frames_to_bytes(runtime,
						runtime->status->hw_ptr);
			appl_bytes += 2 * rtd->period_bytes;
			offset = appl_bytes % rtd->buffer_bytes;
			count = rtd->period_bytes;
			hdmi_dma_mmap_copy(substream, offset, count);
		}
		snd_pcm_period_elapsed(substream);

	}

	spin_unlock_irqrestore(&rtd->irq_lock, flags);

	return;
}
#endif

static irqreturn_t hdmi_dma_isr(int irq, void *dev_id)
{
	struct imx_hdmi_dma_runtime_data *rtd = dev_id;
	struct snd_pcm_substream *substream = rtd->tx_substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long offset, count, appl_bytes;
	unsigned long flags;
	unsigned int status;

	spin_lock_irqsave(&rtd->irq_lock, flags);

	hdmi_dma_irq_mute(1);
	status = hdmi_dma_get_irq_status();
	hdmi_dma_clear_irq_status(status);

	if (rtd->tx_active && (status & HDMI_IH_AHBDMAAUD_STAT0_DONE)) {
		rtd->offset += rtd->period_bytes;
		rtd->offset %= rtd->period_bytes * rtd->periods;

		/* For mmap access, need to copy data from dma_buffer
		 * to hw_buffer and add the frame info. */
		if (runtime->access == SNDRV_PCM_ACCESS_MMAP_INTERLEAVED) {
			appl_bytes = frames_to_bytes(runtime,
						runtime->status->hw_ptr);
			appl_bytes += 2 * rtd->period_bytes;
			offset = appl_bytes % rtd->buffer_bytes;
			count = rtd->period_bytes;
			hdmi_dma_mmap_copy(substream, offset, count);
		}
		snd_pcm_period_elapsed(substream);

		hdmi_dma_set_addr(rtd->hw_buffer.addr +
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
	case HDMI_DMA_BURST_UNSPECIFIED_LEGNTH:
		break;
	case HDMI_DMA_BURST_INCR4:
		value |= HDMI_AHB_DMA_CONF0_BURST_MODE;
		break;
	case HDMI_DMA_BURST_INCR8:
		value |= HDMI_AHB_DMA_CONF0_BURST_MODE |
			 HDMI_AHB_DMA_CONF0_INCR8;
		break;
	case HDMI_DMA_BURST_INCR16:
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

static void hdmi_dma_set_thrsld_incrtype(int channels)
{
	int rev = hdmi_readb(HDMI_REVISION_ID);

	switch (rev) {
	case 0x0a:
		{
			switch (channels) {
			case 2:
				hdmi_dma_set_incr_type(HDMI_DMA_BURST_INCR4);
				hdmi_writeb(126, HDMI_AHB_DMA_THRSLD);
				break;
			case 4:
			case 6:
			case 8:
				hdmi_dma_set_incr_type(HDMI_DMA_BURST_INCR4);
				hdmi_writeb(124, HDMI_AHB_DMA_THRSLD);
				break;
			default:
				pr_debug("unsupport channel!\r\n");
			}
		}
		break;
	case 0x1a:
		hdmi_writeb(128, HDMI_AHB_DMA_THRSLD);
		hdmi_dma_set_incr_type(HDMI_DMA_BURST_INCR8);
		break;
	default:
		pr_debug("error:unrecognized hdmi controller!\r\n");
		break;
	}

	pr_debug("HDMI_AHB_DMA_THRSLD          0x%02x\n", hdmi_readb(HDMI_AHB_DMA_THRSLD));
}

static void hdmi_dma_configure_dma(int channels)
{
	hdmi_dma_enable_hlock(1);
	hdmi_dma_set_thrsld_incrtype(channels);
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

	/* Copy pcm data from userspace and add frame info.
	 * Destination is hw_buffer. */
	hw_buf = (u32 *)(rtd->hw_buffer.area + (pos_bytes * rtd->buffer_ratio));

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

#ifdef CONFIG_ARCH_MX6
static bool hdmi_filter(struct dma_chan *chan, void *param)
{

	if (!imx_dma_is_general_purpose(chan))
		return false;

	chan->private = param;
	return true;
}


static int hdmi_init_sdma_buffer(struct imx_hdmi_dma_runtime_data *params)
{
	int i;
	u32 *tmp_addr1, *tmp_addr2;

	if (!params->hdmi_sdma_t) {
		dev_err(&params->dma_channel->dev->device,
				"hdmi private addr invalid!!!\n");
		return -EINVAL;
	}

	params->hdmi_sdma_t->control_reg_addr =
			HDMI_BASE_ADDR + HDMI_AHB_DMA_START;
	params->hdmi_sdma_t->status_reg_addr =
			HDMI_BASE_ADDR + HDMI_IH_AHBDMAAUD_STAT0;
	params->hdmi_sdma_t->dma_start_addr =
			HDMI_BASE_ADDR + HDMI_AHB_DMA_STRADDR0;

	tmp_addr1 = (u32 *)params->hdmi_sdma_t + 3;
	tmp_addr2 = (u32 *)params->hdmi_sdma_t + 4;
	for (i = 0; i < params->sdma_params.buffer_num; i++) {
		*tmp_addr1 = params->hw_buffer.addr +
				i * params->period_bytes * params->buffer_ratio;
		*tmp_addr2 = *tmp_addr1 + params->dma_period_bytes - 1;
		tmp_addr1 += 2;
		tmp_addr2 += 2;
	}

	return 0;
}

static int hdmi_sdma_alloc(struct imx_hdmi_dma_runtime_data *params)
{
	dma_cap_mask_t mask;

	params->dma_data.peripheral_type = IMX_DMATYPE_HDMI;
	params->dma_data.priority = DMA_PRIO_HIGH;
	params->dma_data.dma_request = MX6Q_DMA_REQ_EXT_DMA_REQ_0;
	params->dma_data.private = &params->sdma_params;

	/* Try to grab a DMA channel */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	params->dma_channel = dma_request_channel(
			mask, hdmi_filter, &params->dma_data);
	if (params->dma_channel == NULL) {
		dev_err(&params->dma_channel->dev->device,
			"HDMI:unable to alloc dma_channel channel\n");
		return -EBUSY;
	}
	return 0;
}

static int hdmi_sdma_config(struct imx_hdmi_dma_runtime_data *params)
{
	struct dma_slave_config slave_config;
	int ret;

	slave_config.direction = DMA_TRANS_NONE;
	ret = dmaengine_slave_config(params->dma_channel, &slave_config);
	if (ret) {
		dev_err(&params->dma_channel->dev->device,
					"%s failed\r\n", __func__);
		return -EINVAL;
	}

	params->desc =
		params->dma_channel->device->device_prep_dma_cyclic(
					params->dma_channel, 0,
					0,
					0,
					DMA_TRANS_NONE);
	if (!params->desc) {
		dev_err(&params->dma_channel->dev->device,
				"cannot prepare slave dma\n");
		return -EINVAL;
	}

	params->desc->callback = hdmi_sdma_isr;
	params->desc->callback_param = (void *)hdmi_dma_priv;

	return 0;
}
#endif

static int hdmi_dma_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_hdmi_dma_runtime_data *params = runtime->private_data;
	if (hdmi_SDMA_check() && (params->dma_channel)) {
		dma_release_channel(params->dma_channel);
		params->dma_channel = NULL;
	}
	return 0;
}

static int hdmi_dma_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_hdmi_dma_runtime_data *rtd = runtime->private_data;
#ifdef CONFIG_ARCH_MX6
	int err;
#endif

	rtd->buffer_bytes = params_buffer_bytes(params);
	rtd->periods = params_periods(params);
	rtd->period_bytes = params_period_bytes(params);
	rtd->channels = params_channels(params);
	rtd->format = params_format(params);
	rtd->rate = params_rate(params);

	rtd->offset = 0;
	rtd->period_time = HZ / (params_rate(params) / params_period_size(params));

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
#ifdef CONFIG_ARCH_MX6
	if (hdmi_SDMA_check()) {
		rtd->sdma_params.buffer_num = rtd->periods;
		rtd->sdma_params.phyaddr = rtd->phy_hdmi_sdma_t;

		/* Allocate SDMA channel for HDMI */
		err = hdmi_init_sdma_buffer(rtd);
		if (err)
			return err;

		err = hdmi_sdma_alloc(rtd);
		if (err)
			return err;

		err = hdmi_sdma_config(rtd);
		if (err)
			return err;
	}
#endif

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	hdmi_dma_configure_dma(rtd->channels);
	hdmi_dma_set_addr(rtd->hw_buffer.addr, rtd->dma_period_bytes);

	dumprtd(rtd);

	hdmi_dma_update_iec_header(substream);

	/* Init par for mmap optimizate */
	init_table(rtd->channels);

	rtd->appl_bytes = 0;

	return 0;
}

static int hdmi_dma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct imx_hdmi_dma_runtime_data *rtd = runtime->private_data;
	unsigned long offset,  count, space_to_end, appl_bytes;
	unsigned int status;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		rtd->frame_idx = 0;
		if (runtime->access == SNDRV_PCM_ACCESS_MMAP_INTERLEAVED) {
			appl_bytes = frames_to_bytes(runtime,
						runtime->status->hw_ptr);
			offset = appl_bytes % rtd->buffer_bytes;
			count = rtd->buffer_bytes;
			space_to_end = rtd->buffer_bytes - offset;

			if (count <= space_to_end) {
				hdmi_dma_mmap_copy(substream, offset, count);
			} else {
				hdmi_dma_mmap_copy(substream,
						offset, space_to_end);
				hdmi_dma_mmap_copy(substream,
						0, count - space_to_end);
			}
		}
		dumpregs();

		hdmi_fifo_reset();
		udelay(1);

		status = hdmi_dma_get_irq_status();
		hdmi_dma_clear_irq_status(status);

		hdmi_dma_priv->tx_active = true;
		hdmi_dma_start();
		hdmi_dma_irq_mask(0);
		hdmi_set_dma_mode(1);
		if (hdmi_SDMA_check())
			dmaengine_submit(rtd->desc);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		hdmi_dma_priv->tx_active = false;
		hdmi_dma_stop();
		hdmi_set_dma_mode(0);
		hdmi_dma_irq_mask(1);
		if (hdmi_SDMA_check())
			dmaengine_terminate_all(rtd->dma_channel);
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
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME,
	.formats = MXC_HDMI_FORMATS_PLAYBACK,
	.rate_min = 32000,
	.channels_min = 2,
	.channels_max = 8,
	.buffer_bytes_max = HDMI_PCM_BUF_SIZE,
	.period_bytes_min = HDMI_DMA_PERIOD_BYTES / 2,
	.period_bytes_max = HDMI_DMA_PERIOD_BYTES / 2,
	.periods_min = 8,
	.periods_max = 8,
	.fifo_size = 0,
};

static void hdmi_dma_irq_enable(struct imx_hdmi_dma_runtime_data *rtd)
{
	unsigned long flags;

	hdmi_writeb(0xff, HDMI_AHB_DMA_POL);
	hdmi_writeb(0xff, HDMI_AHB_DMA_BUFFPOL);

	spin_lock_irqsave(&hdmi_dma_priv->irq_lock, flags);

	hdmi_dma_clear_irq_status(0xff);
	if (hdmi_SDMA_check())
		hdmi_dma_irq_mute(1);
	else
		hdmi_dma_irq_mute(0);
	hdmi_dma_irq_mask(0);

	hdmi_mask(0);

	spin_unlock_irqrestore(&hdmi_dma_priv->irq_lock, flags);

}

static void hdmi_dma_irq_disable(struct imx_hdmi_dma_runtime_data *rtd)
{
	unsigned long flags;

	spin_lock_irqsave(&rtd->irq_lock, flags);

	hdmi_dma_irq_mask(1);
	hdmi_dma_irq_mute(1);
	hdmi_dma_clear_irq_status(0xff);

	hdmi_mask(1);

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

	ret = mxc_hdmi_register_audio(substream);
	if (ret < 0) {
		pr_err("ERROR: HDMI is not ready!\n");
		return ret;
	}

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
	mxc_hdmi_unregister_audio(substream);

	clk_disable(rtd->iahb_clk);
	clk_disable(rtd->isfr_clk);

	return 0;
}

static struct snd_pcm_ops imx_hdmi_dma_pcm_ops = {
	.open		= hdmi_dma_open,
	.close		= hdmi_dma_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= hdmi_dma_hw_params,
	.hw_free	= hdmi_dma_hw_free,
	.trigger	= hdmi_dma_trigger,
	.pointer	= hdmi_dma_pointer,
	.copy		= hdmi_dma_copy,
};

static int imx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	struct snd_dma_buffer *hw_buffer = &hdmi_dma_priv->hw_buffer;

	/* The 'dma_buffer' is the buffer alsa knows about.
	 * It contains only raw audio. */
	buf->area = dma_alloc_writethrough(pcm->card->dev,
					   HDMI_PCM_BUF_SIZE,
					   &buf->addr, GFP_KERNEL);

	buf->bytes = HDMI_PCM_BUF_SIZE;
	if (!buf->area)
		return -ENOMEM;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	hdmi_dma_priv->tx_substream = substream;

	/* For mmap access, isr will copy from
	 * the dma_buffer to the hw_buffer */
	hw_buffer->area = dma_alloc_writethrough(pcm->card->dev,
						HDMI_DMA_BUF_SIZE,
						&hw_buffer->addr, GFP_KERNEL);
	if (!hw_buffer->area)
		return -ENOMEM;

	hw_buffer->bytes = HDMI_DMA_BUF_SIZE;

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

	/* free each dma_buffer */
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

	/* free the hw_buffer */
	buf = &hdmi_dma_priv->hw_buffer;
	if (buf->area) {
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

	if (hdmi_SDMA_check()) {
		/*To alloc a buffer non cacheable for hdmi script use*/
		hdmi_dma_priv->hdmi_sdma_t =
			dma_alloc_noncacheable(NULL,
				sizeof(struct hdmi_sdma_script_data),
				&hdmi_dma_priv->phy_hdmi_sdma_t,
				GFP_KERNEL);
		if (hdmi_dma_priv->hdmi_sdma_t == NULL)
			return -ENOMEM;
	}

	hdmi_dma_priv->tx_active = false;
	spin_lock_init(&hdmi_dma_priv->irq_lock);
	hdmi_dma_priv->irq = hdmi_drvdata->irq;

	hdmi_dma_init_iec_header();

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
	if (!hdmi_SDMA_check()) {
		if (request_irq(hdmi_dma_priv->irq, hdmi_dma_isr, IRQF_SHARED,
				"hdmi dma", hdmi_dma_priv)) {
			dev_err(&pdev->dev,
					"MXC hdmi: failed to request irq %d\n",
					hdmi_dma_priv->irq);
			ret = -EBUSY;
			goto e_irq;
		}
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
	if (hdmi_SDMA_check()) {
		dma_free_coherent(NULL,
				sizeof(struct hdmi_sdma_script_data),
				hdmi_dma_priv->hdmi_sdma_t,
				hdmi_dma_priv->phy_hdmi_sdma_t);
	}
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
