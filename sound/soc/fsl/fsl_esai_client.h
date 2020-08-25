/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __FSL_ESAI_CLIENT_H
#define __FSL_ESAI_CLIENT_H

/**
 * fsl_esai_client_dma: esai dma client
 * @dma_buffer: structure of dma buffer
 * @buffer_bytes: buffer size in bytes
 * @period_bytes: period size in bytes
 * @period_num: period number
 * @buffer_offset: read offset of buffer
 * @channels: channel number
 * @word_width: word width in bytes
 * @active: dma transfer is active
 */
struct fsl_esai_client_dma {
	struct snd_dma_buffer dma_buffer;
	int   buffer_bytes;
	int   period_bytes;
	int   period_num;
	int   buffer_offset;
	int   channels;
	int   word_width;
	bool  active;
};

/**
 * fsl_esai_client: esai client
 * @cpu_dai_drv: CPU DAI driver for this device
 * @dma: dma instance for playback and capture
 * @id: client index
 */
struct fsl_esai_client {
	struct snd_soc_dai_driver cpu_dai_drv;
	struct fsl_esai_client_dma dma[2];
	int id;
};

#endif /* __FSL_ESAI_CLIENT_H */
