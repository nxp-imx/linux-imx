// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// Copyright 2021 NXP
//
// Author: Daniel Baluta <daniel.baluta@nxp.com>

#include <sound/soc.h>
#include <sound/sof.h>
#include <sound/compress_driver.h>
#include "sof-audio.h"
#include "sof-priv.h"

static void sof_set_transferred_bytes(struct sof_compr_stream *sstream,
				      u64 host_pos, u64 buffer_size)
{
	u64 prev_pos;
	unsigned int copied;

	div64_u64_rem(sstream->copied_total, buffer_size, &prev_pos);

	if (host_pos < prev_pos)
		copied = (buffer_size - prev_pos) + host_pos;
	else
		copied = host_pos - prev_pos;

	sstream->copied_total += copied;
}

static void snd_sof_compr_fragment_elapsed_work(struct work_struct *work)
{
	struct snd_sof_pcm_stream *sps =
		container_of(work, struct snd_sof_pcm_stream,
			     period_elapsed_work);

	snd_compr_fragment_elapsed(sps->cstream);
}

void snd_sof_compr_init_elapsed_work(struct work_struct *work)
{
	INIT_WORK(work, snd_sof_compr_fragment_elapsed_work);
}

/*
 * sof compr fragment elapse, this could be called in irq thread context
 */
void snd_sof_compr_fragment_elapsed(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct sof_compr_stream *sstream = runtime->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_AUDIO_PCM_DRV_NAME);
	struct snd_sof_pcm *spcm;

	spcm = snd_sof_find_spcm_dai(component, rtd);
	if (!spcm) {
		dev_err(component->dev,
			"error: fragment elapsed for unknown stream!\n");
		return;
	}

	sof_set_transferred_bytes(sstream, spcm->stream[cstream->direction].posn.host_posn,
				  runtime->buffer_size);

	/* use the same workqueue-based solution as for PCM, cf. snd_sof_pcm_elapsed */
	schedule_work(&spcm->stream[cstream->direction].period_elapsed_work);
}

static int create_page_table(struct snd_soc_component *component,
			     struct snd_compr_stream *cstream,
			     unsigned char *dma_area, size_t size)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_dma_buffer *dmab = cstream->runtime->dma_buffer_p;
	int dir = cstream->direction;
	struct snd_sof_pcm *spcm;

	spcm = snd_sof_find_spcm_dai(component, rtd);
	if (!spcm)
		return -EINVAL;

	return snd_sof_create_page_table(component->dev, dmab,
					 spcm->stream[dir].page_table.area, size);
}

int sof_compr_open(struct snd_soc_component *component,
		   struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct sof_compr_stream *sstream;
	struct snd_sof_pcm *spcm;
	int dir;

	sstream = kzalloc(sizeof(*sstream), GFP_KERNEL);
	if (!sstream)
		return -ENOMEM;

	spcm = snd_sof_find_spcm_dai(component, rtd);
	if (!spcm) {
		kfree(sstream);
		return -EINVAL;
	}

	dir = cstream->direction;

	if (spcm->stream[dir].cstream) {
		kfree(sstream);
		return -EBUSY;
	}

	spcm->stream[dir].cstream = cstream;
	spcm->stream[dir].posn.host_posn = 0;
	spcm->stream[dir].posn.dai_posn = 0;
	spcm->prepared[dir] = false;

	runtime->private_data = sstream;

	return 0;
}

int sof_compr_free(struct snd_soc_component *component,
		   struct snd_compr_stream *cstream)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct sof_compr_stream *sstream = runtime->private_data;
	struct sof_ipc_stream stream;
	struct sof_ipc_reply reply;
	struct snd_sof_pcm *spcm;
	int ret = 0;

	spcm = snd_sof_find_spcm_dai(component, rtd);
	if (!spcm)
		return -EINVAL;

	stream.hdr.size = sizeof(stream);
	stream.hdr.cmd = SOF_IPC_GLB_STREAM_MSG | SOF_IPC_STREAM_PCM_FREE;
	stream.comp_id = spcm->stream[cstream->direction].comp_id;

	if (spcm->prepared[cstream->direction]) {
		ret = sof_ipc_tx_message(sdev->ipc, stream.hdr.cmd,
					 &stream, sizeof(stream),
					 &reply, sizeof(reply));
		if (!ret)
			spcm->prepared[cstream->direction] = false;
	}

	cancel_work_sync(&spcm->stream[cstream->direction].period_elapsed_work);
	spcm->stream[cstream->direction].cstream = NULL;
	kfree(sstream);

	return ret;
}

int sof_compr_set_params(struct snd_soc_component *component,
			 struct snd_compr_stream *cstream, struct snd_compr_params *params)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd_pcm = cstream->private_data;
	struct snd_compr_runtime *rtd = cstream->runtime;
	struct sof_compr_stream *sstream = rtd->private_data;
	struct sof_ipc_pcm_params_reply ipc_params_reply;
	struct sof_ipc_pcm_params pcm;
	struct snd_sof_pcm *spcm;
	int ret;

	spcm = snd_sof_find_spcm_dai(component, rtd_pcm);
	if (!spcm)
		return -EINVAL;

	cstream->dma_buffer.dev.type = SNDRV_DMA_TYPE_DEV_SG;
	cstream->dma_buffer.dev.dev = sdev->dev;
	ret = snd_compr_malloc_pages(cstream, rtd->buffer_size);
	if (ret < 0)
		return ret;

	create_page_table(component, cstream, rtd->dma_area, rtd->dma_bytes);

	pcm.params.buffer.pages = PFN_UP(rtd->dma_bytes);
	pcm.hdr.size = sizeof(pcm);
	pcm.hdr.cmd = SOF_IPC_GLB_STREAM_MSG | SOF_IPC_STREAM_PCM_PARAMS;

	pcm.comp_id = spcm->stream[cstream->direction].comp_id;
	pcm.params.hdr.size = sizeof(pcm.params);
	pcm.params.buffer.phy_addr = spcm->stream[cstream->direction].page_table.addr;
	pcm.params.buffer.size = rtd->dma_bytes;
	pcm.params.direction = cstream->direction;
	pcm.params.channels = params->codec.ch_out;
	pcm.params.rate = params->codec.sample_rate;
	pcm.params.buffer_fmt = SOF_IPC_BUFFER_INTERLEAVED;
	pcm.params.frame_fmt = SOF_IPC_FRAME_S32_LE;
	pcm.params.sample_container_bytes =
		snd_pcm_format_physical_width(SNDRV_PCM_FORMAT_S32) >> 3;
	pcm.params.host_period_bytes = params->buffer.fragment_size;

	ret = sof_ipc_tx_message(sdev->ipc, pcm.hdr.cmd, &pcm, sizeof(pcm),
				 &ipc_params_reply, sizeof(ipc_params_reply));
	if (ret < 0) {
		dev_err(component->dev, "error ipc failed\n");
		return ret;
	}

	sstream->posn_offset = sdev->stream_box.offset + ipc_params_reply.posn_offset;
	sstream->sample_rate = params->codec.sample_rate;
	spcm->prepared[cstream->direction] = true;

	return 0;
}

int sof_compr_get_params(struct snd_soc_component *component,
			 struct snd_compr_stream *cstream, struct snd_codec *params)
{
	return 0;
}

int sof_compr_trigger(struct snd_soc_component *component,
		      struct snd_compr_stream *cstream, int cmd)
{
	struct sof_ipc_stream stream;
	struct sof_ipc_reply reply;
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pcm *spcm;

	spcm = snd_sof_find_spcm_dai(component, rtd);
	if (!spcm)
		return -EINVAL;

	stream.hdr.size = sizeof(stream);
	stream.hdr.cmd = SOF_IPC_GLB_STREAM_MSG;
	stream.comp_id = spcm->stream[cstream->direction].comp_id;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_START;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_STOP;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_PAUSE;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_RELEASE;
		break;
	default:
		dev_err(component->dev, "error: unhandled trigger cmd %d\n", cmd);
		break;
	}

	return sof_ipc_tx_message(sdev->ipc, stream.hdr.cmd,
				  &stream, sizeof(stream),
				  &reply, sizeof(reply));
}

int sof_compr_copy(struct snd_soc_component *component,
		   struct snd_compr_stream *cstream,
		   char __user *buf, size_t count)
{
	struct snd_compr_runtime *rtd = cstream->runtime;
	unsigned int offset, n;
	void *ptr;
	int ret;

	if (count > rtd->buffer_size)
		count = rtd->buffer_size;

	div_u64_rem(rtd->total_bytes_available, rtd->buffer_size, &offset);
	ptr = rtd->dma_area + offset;
	n = rtd->buffer_size - offset;

	if (count < n) {
		ret = copy_from_user(ptr, buf, count);
	} else {
		ret = copy_from_user(ptr, buf, n);
		ret += copy_from_user(rtd->dma_area, buf + n, count - n);
	}

	return count - ret;
}

static int sof_compr_pointer(struct snd_soc_component *component,
			     struct snd_compr_stream *cstream,
			     struct snd_compr_tstamp *tstamp)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct sof_compr_stream *sstream = runtime->private_data;

	tstamp->sampling_rate = sstream->sample_rate;
	tstamp->copied_total = sstream->copied_total;

	return 0;
}

struct snd_compress_ops sof_compressed_ops = {
	.open		= sof_compr_open,
	.free		= sof_compr_free,
	.set_params	= sof_compr_set_params,
	.get_params	= sof_compr_get_params,
	.trigger	= sof_compr_trigger,
	.pointer	= sof_compr_pointer,
	.copy		= sof_compr_copy,
};
EXPORT_SYMBOL(sof_compressed_ops);
