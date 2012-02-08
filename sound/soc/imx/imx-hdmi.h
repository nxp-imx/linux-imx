/*
 * MXC HDMI ALSA Soc Codec Driver
 *
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
 */

/*
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

#ifndef __IMX_HDMI_H
#define __IMX_HDMI_H

#define DRV_NAME "imx-hdmi"

struct imx_hdmi {
	int irq;
	struct platform_device *pdev;
	struct platform_device *soc_platform_pdev;
};

#define HDMI_MAX_RATES 7
#define HDMI_MAX_SAMPLE_SIZE 3
#define HDMI_MAX_CHANNEL_CONSTRAINTS 4

#define MXC_HDMI_RATES_PLAYBACK			(SNDRV_PCM_RATE_32000 | \
						 SNDRV_PCM_RATE_44100 | \
						 SNDRV_PCM_RATE_48000 | \
						 SNDRV_PCM_RATE_88200 | \
						 SNDRV_PCM_RATE_96000 | \
						 SNDRV_PCM_RATE_176400 | \
						 SNDRV_PCM_RATE_192000)

#define MXC_HDMI_FORMATS_PLAYBACK		(SNDRV_PCM_FMTBIT_S16_LE | \
						 SNDRV_PCM_FMTBIT_S24_LE)

typedef union {
	uint64_t  U;
	struct {
		unsigned consumer:1;
		unsigned linear_pcm:1;
		unsigned copyright:1;
		unsigned pre_emphasis:3;
		unsigned mode:2;

		unsigned category_code:8;

		unsigned source:4;
		unsigned channel:4;

		unsigned sample_freq:4;
		unsigned clock_acc:2;
		unsigned reserved0:2;

		unsigned word_length:4;
		unsigned org_sample_freq:4;

		unsigned cgms_a:2;
		unsigned reserved1:6;

		unsigned reserved2:8;

		unsigned reserved3:8;
	} B;
	unsigned char status[8];
} hdmi_audio_header_t;

typedef union {
	uint32_t  U;
	struct {
		unsigned data:24;
		unsigned v:1;
		unsigned u:1;
		unsigned c:1;
		unsigned p:1;
		unsigned b:1;
		unsigned reserved:3;
	} B;
} hdmi_audio_dma_data_t;

extern hdmi_audio_header_t iec_header;

#endif /* __IMX_HDMI_H */
