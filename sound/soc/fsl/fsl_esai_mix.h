/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _FSL_ESAI_MIX_H
#define _FSL_ESAI_MIX_H

/* maximum client number is 4; */
#define MAX_CLIENT_NUM 4

/**
 * fsl_esai_mix: esai mix/split data
 * @chan: dma channel
 * @fe_substream: handler of front end substream
 * @client: handler of client
 * @dma_buffer: structure of dma buffer
 * @buffer_offset: read offset of buffer
 * @buffer_bytes: buffer size in bytes
 * @period_bytes: period size in bytes
 * @period_num: period number
 * @word_width: word width in bytes
 * @channels: channel number
 * @client_cnt: client number, default 2.
 * @sdo_cnt: output pin number of esai
 * @sdi_cnt: input pin number of esai
 * @active: mixer is enabled or not
 */
struct fsl_esai_mix {
	struct dma_chan *chan;
	struct snd_pcm_substream *fe_substream[MAX_CLIENT_NUM];
	struct fsl_esai_client *client[MAX_CLIENT_NUM];
	struct snd_dma_buffer dma_buffer;
	struct workqueue_struct  *mix_wq;
	struct work_struct       work;
	struct snd_pcm_substream *substream;
	dma_cookie_t cookie;
	u32 buffer_read_offset;
	u32 buffer_write_offset;
	u32 buffer_bytes;
	u32 period_bytes;
	u32 period_num;
	u32 word_width;
	u32 channels;
	u32 client_cnt;
	u32 sdo_cnt;
	u32 sdi_cnt;
	atomic_t active;
};

int fsl_esai_mix_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct fsl_esai_mix *mix);
int fsl_esai_mix_open(struct snd_pcm_substream *substream, struct fsl_esai_mix *mix);
int fsl_esai_mix_close(struct snd_pcm_substream *substream, struct fsl_esai_mix *mix);
int fsl_esai_mix_trigger(struct snd_pcm_substream *substream, int cmd,
			 struct fsl_esai_mix *mix);
int fsl_esai_mix_probe(struct device *dev, struct fsl_esai_mix *mix_rx, struct fsl_esai_mix *mix_tx);
int fsl_esai_mix_remove(struct device *dev, struct fsl_esai_mix *mix_rx, struct fsl_esai_mix *mix_tx);

#endif /* _FSL_ESAI_MIX_H */
