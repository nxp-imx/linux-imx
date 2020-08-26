/*
 * imx-rpmsg-platform.c  --  ALSA Soc Audio Layer
 *
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>

#include "imx-pcm.h"
#include "fsl_rpmsg_i2s.h"
#include "../../core/pcm_local.h"

#define DRV_NAME	"imx_pcm_rpmsg"

struct i2s_info *i2s_info_g;
EXPORT_SYMBOL(i2s_info_g);

static struct snd_pcm_hardware imx_rpmsg_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_NO_PERIOD_WAKEUP |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME,
	.buffer_bytes_max = IMX_SAI_DMABUF_SIZE,
	.period_bytes_min = 512,
	.period_bytes_max = 65532, /* Limited by SDMA engine */
	.periods_min = 2,
	.periods_max = 6000,
	.fifo_size = 0,
};

static int imx_rpmsg_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info  *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg *rpmsg;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_HW_PARAM];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_HW_PARAM];

	rpmsg->send_msg.param.rate = params_rate(params);

	if (params_format(params) == SNDRV_PCM_FORMAT_S16_LE)
		rpmsg->send_msg.param.format   = RPMSG_S16_LE;
	else if (params_format(params) == SNDRV_PCM_FORMAT_S24_LE)
		rpmsg->send_msg.param.format   = RPMSG_S24_LE;
	else if (params_format(params) == SNDRV_PCM_FORMAT_DSD_U16_LE)
		rpmsg->send_msg.param.format   = SNDRV_PCM_FORMAT_DSD_U16_LE;
	else if (params_format(params) == SNDRV_PCM_FORMAT_DSD_U32_LE)
		rpmsg->send_msg.param.format   = SNDRV_PCM_FORMAT_DSD_U32_LE;
	else
		rpmsg->send_msg.param.format   = RPMSG_S32_LE;

	if (params_channels(params) == 1)
		rpmsg->send_msg.param.channels = RPMSG_CH_LEFT;
	else
		rpmsg->send_msg.param.channels = RPMSG_CH_STEREO;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_HW_PARAM;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_HW_PARAM;

	i2s_info->send_message(rpmsg, i2s_info);

	return 0;
}

static int imx_rpmsg_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static snd_pcm_uframes_t imx_rpmsg_pcm_pointer(
				struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	unsigned int pos = 0;
	struct i2s_rpmsg *rpmsg;
	int buffer_tail = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_PERIOD_DONE + I2S_TYPE_A_NUM];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_PERIOD_DONE + I2S_TYPE_A_NUM];

	buffer_tail = rpmsg->recv_msg.param.buffer_tail;
	pos = buffer_tail * snd_pcm_lib_period_bytes(substream);

	return bytes_to_frames(substream->runtime, pos);
}

static void imx_rpmsg_timer_callback(struct timer_list *t)
{
	struct stream_timer  *stream_timer =
			from_timer(stream_timer, t, timer);
	struct snd_pcm_substream *substream = stream_timer->substream;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	unsigned long flags;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_PERIOD_DONE + I2S_TYPE_A_NUM];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_PERIOD_DONE + I2S_TYPE_A_NUM];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_PERIOD_DONE;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_PERIOD_DONE;

	spin_lock_irqsave(&i2s_info->wq_lock, flags);
	if (i2s_info->work_write_index != i2s_info->work_read_index) {
		int index = i2s_info->work_write_index;
		memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
		queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
		i2s_info->work_write_index++;
		i2s_info->work_write_index %= WORK_MAX_NUM;
	} else
		i2s_info->msg_drop_count[substream->stream]++;
	spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
}

static int imx_rpmsg_pcm_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	int cmd;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_OPEN];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_OPEN];

	imx_rpmsg_pcm_hardware.buffer_bytes_max =
					i2s_info->prealloc_buffer_size;
	imx_rpmsg_pcm_hardware.period_bytes_max =
			imx_rpmsg_pcm_hardware.buffer_bytes_max / 2;

	snd_soc_set_runtime_hwparams(substream, &imx_rpmsg_pcm_hardware);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_OPEN;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_OPEN;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		cmd = I2S_TX_PERIOD_DONE + I2S_TYPE_A_NUM;
		i2s_info->rpmsg[cmd].send_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[cmd].recv_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[I2S_TX_POINTER].recv_msg.param.buffer_offset = 0;
	} else {
		cmd = I2S_RX_PERIOD_DONE + I2S_TYPE_A_NUM;
		i2s_info->rpmsg[cmd].send_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[cmd].recv_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[I2S_RX_POINTER].recv_msg.param.buffer_offset = 0;
	}

	i2s_info->send_message(rpmsg, i2s_info);

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	i2s_info->msg_drop_count[substream->stream] = 0;

	/*create thread*/
	i2s_info->stream_timer[substream->stream].substream = substream;

	timer_setup(&i2s_info->stream_timer[substream->stream].timer,
					imx_rpmsg_timer_callback, 0);

	return ret;
}

static int imx_rpmsg_pcm_close(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_CLOSE];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_CLOSE];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_CLOSE;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_CLOSE;
	flush_workqueue(i2s_info->rpmsg_wq);
	i2s_info->send_message(rpmsg, i2s_info);

	del_timer(&i2s_info->stream_timer[substream->stream].timer);

	rtd->dai_link->ignore_suspend = 0;

	if (i2s_info->msg_drop_count[substream->stream])
		dev_warn(rtd->dev, "Msg is dropped!, number is %d\n",
			i2s_info->msg_drop_count[substream->stream]);

	return ret;
}

static int imx_rpmsg_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);

	/* NON-MMAP mode, NONBLOCK, Version 2, enable lpa in dts
	 * four condition to determine the lpa is enabled.
	 */
	if ((runtime->access == SNDRV_PCM_ACCESS_RW_INTERLEAVED ||
		runtime->access == SNDRV_PCM_ACCESS_RW_NONINTERLEAVED) &&
			rpmsg_i2s->version == 2 &&
			rpmsg_i2s->enable_lpa) {
		rtd->dai_link->ignore_suspend = 1;
		rpmsg_i2s->force_lpa = 1;
	} else
		rpmsg_i2s->force_lpa = 0;

	return 0;
}

static int imx_rpmsg_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_wc(substream->pcm->card->dev, vma,
			   runtime->dma_area,
			   runtime->dma_addr,
			   runtime->dma_bytes);
}

static void imx_rpmsg_pcm_dma_complete(void *arg)
{
	struct snd_pcm_substream *substream = arg;

	snd_pcm_period_elapsed(substream);
}

static int imx_rpmsg_pcm_prepare_and_submit(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg   *rpmsg;
	unsigned long flags;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_BUFFER];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_BUFFER];


	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_BUFFER;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_BUFFER;

	rpmsg->send_msg.param.buffer_addr = substream->runtime->dma_addr;
	rpmsg->send_msg.param.buffer_size = snd_pcm_lib_buffer_bytes(substream);
	rpmsg->send_msg.param.period_size = snd_pcm_lib_period_bytes(substream);
	rpmsg->send_msg.param.buffer_tail = 0;

	i2s_info->num_period[substream->stream] =
			rpmsg->send_msg.param.buffer_size /
				rpmsg->send_msg.param.period_size;

	i2s_info->callback[substream->stream] = imx_rpmsg_pcm_dma_complete;
	i2s_info->callback_param[substream->stream] = substream;

	spin_lock_irqsave(&i2s_info->wq_lock, flags);
	if (i2s_info->work_write_index != i2s_info->work_read_index) {
		int index = i2s_info->work_write_index;
		memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
		queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
		i2s_info->work_write_index++;
		i2s_info->work_write_index %= WORK_MAX_NUM;
		spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
	} else {
		spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
		return -EPIPE;
	}

	return 0;
}

static int imx_rpmsg_async_issue_pending(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	unsigned long flags;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_START];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_START];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_START;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_START;

	spin_lock_irqsave(&i2s_info->wq_lock, flags);
	if (i2s_info->work_write_index != i2s_info->work_read_index) {
		int index = i2s_info->work_write_index;
		memcpy(&i2s_info->work_list[index].msg, rpmsg,
						sizeof(struct i2s_rpmsg_s));
		queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
		i2s_info->work_write_index++;
		i2s_info->work_write_index %= WORK_MAX_NUM;
		spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
	} else {
		spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
		return -EPIPE;
	}

	return 0;
}

static int imx_rpmsg_restart(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai         *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s       *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info            *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	unsigned long flags;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_RESTART];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_RESTART];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_RESTART;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_RESTART;

	spin_lock_irqsave(&i2s_info->wq_lock, flags);
	if (i2s_info->work_write_index != i2s_info->work_read_index) {
		int index = i2s_info->work_write_index;
		memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
		queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
		i2s_info->work_write_index++;
		i2s_info->work_write_index %= WORK_MAX_NUM;
		spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
	} else {
		spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
		return -EPIPE;
	}
	return 0;
}

static int imx_rpmsg_pause(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai         *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s       *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info            *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	unsigned long flags;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_PAUSE];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_PAUSE];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_PAUSE;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_PAUSE;

	spin_lock_irqsave(&i2s_info->wq_lock, flags);
	if (i2s_info->work_write_index != i2s_info->work_read_index) {
		int index = i2s_info->work_write_index;
		memcpy(&i2s_info->work_list[index].msg, rpmsg,
						sizeof(struct i2s_rpmsg_s));
		queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
		i2s_info->work_write_index++;
		i2s_info->work_write_index %= WORK_MAX_NUM;
		spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
	} else {
		spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
		return -EPIPE;
	}
	return 0;
}

static int imx_rpmsg_terminate_all(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai         *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s       *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info            *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	int cmd;
	unsigned long flags;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_TERMINATE];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_TERMINATE];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_TERMINATE;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_TERMINATE;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		cmd = I2S_TX_PERIOD_DONE + I2S_TYPE_A_NUM;
		i2s_info->rpmsg[cmd].send_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[cmd].recv_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[I2S_TX_POINTER].recv_msg.param.buffer_offset = 0;
	} else {
		cmd = I2S_RX_PERIOD_DONE + I2S_TYPE_A_NUM;
		i2s_info->rpmsg[cmd].send_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[cmd].recv_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[I2S_RX_POINTER].recv_msg.param.buffer_offset = 0;
	}

	del_timer(&i2s_info->stream_timer[substream->stream].timer);

	spin_lock_irqsave(&i2s_info->wq_lock, flags);
	if (i2s_info->work_write_index != i2s_info->work_read_index) {
		int index = i2s_info->work_write_index;
		memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
		queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
		i2s_info->work_write_index++;
		i2s_info->work_write_index %= WORK_MAX_NUM;
		spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
	} else {
		spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
		return -EPIPE;
	}

	return 0;
}

int imx_rpmsg_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai     *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s   *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = imx_rpmsg_pcm_prepare_and_submit(substream);
		if (ret)
			return ret;
		ret = imx_rpmsg_async_issue_pending(substream);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		if (rpmsg_i2s->force_lpa)
			break;
		/* fall through */
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = imx_rpmsg_restart(substream);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (!rpmsg_i2s->force_lpa) {
			if (runtime->info & SNDRV_PCM_INFO_PAUSE)
				ret = imx_rpmsg_pause(substream);
			else
				ret = imx_rpmsg_terminate_all(substream);
		}
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = imx_rpmsg_pause(substream);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		ret = imx_rpmsg_terminate_all(substream);
		break;
	default:
		return -EINVAL;
	}

	if (ret)
		return ret;

	return 0;
}

int imx_rpmsg_pcm_ack(struct snd_pcm_substream *substream)
{
	/*send the hw_avail size through rpmsg*/
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai         *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s       *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info            *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg           *rpmsg;
	int buffer_tail = 0;
	int writen_num = 0;
	snd_pcm_sframes_t avail;
	unsigned long flags;

	if (!rpmsg_i2s->force_lpa)
		return 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_PERIOD_DONE + I2S_TYPE_A_NUM];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_PERIOD_DONE + I2S_TYPE_A_NUM];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_PERIOD_DONE;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_PERIOD_DONE;

	rpmsg->send_msg.header.type = I2S_TYPE_C;

	buffer_tail = (frames_to_bytes(runtime, runtime->control->appl_ptr) %
				snd_pcm_lib_buffer_bytes(substream));
	buffer_tail = buffer_tail /  snd_pcm_lib_period_bytes(substream);

	if (buffer_tail != rpmsg->send_msg.param.buffer_tail) {
		writen_num = buffer_tail - rpmsg->send_msg.param.buffer_tail;
		if (writen_num < 0)
			writen_num += runtime->periods;

		rpmsg->send_msg.param.buffer_tail = buffer_tail;

		spin_lock_irqsave(&i2s_info->lock[substream->stream], flags);
		memcpy(&i2s_info->period_done_msg[substream->stream], rpmsg,
				sizeof(struct i2s_rpmsg_s));

		i2s_info->period_done_msg_enabled[substream->stream] = true;
		spin_unlock_irqrestore(&i2s_info->lock[substream->stream], flags);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			avail = snd_pcm_playback_hw_avail(runtime);
		else
			avail = snd_pcm_capture_hw_avail(runtime);

		if ((avail - writen_num * runtime->period_size) <= runtime->period_size) {
			spin_lock_irqsave(&i2s_info->wq_lock, flags);
			if (i2s_info->work_write_index != i2s_info->work_read_index) {
				int index = i2s_info->work_write_index;
				memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
				queue_work(i2s_info->rpmsg_wq,
					&i2s_info->work_list[index].work);
				i2s_info->work_write_index++;
				i2s_info->work_write_index %= WORK_MAX_NUM;
			} else
				i2s_info->msg_drop_count[substream->stream]++;
			spin_unlock_irqrestore(&i2s_info->wq_lock, flags);
		} else {
			if (rpmsg_i2s->force_lpa && !timer_pending(&i2s_info->stream_timer[substream->stream].timer)) {
				int time_msec;
				time_msec = (int)(runtime->period_size*1000/runtime->rate);
				mod_timer(&i2s_info->stream_timer[substream->stream].timer,
					     jiffies + msecs_to_jiffies(time_msec));
			}
		}
	}

	return 0;
}

static struct snd_pcm_ops imx_rpmsg_pcm_ops = {
	.open		= imx_rpmsg_pcm_open,
	.close		= imx_rpmsg_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= imx_rpmsg_pcm_hw_params,
	.hw_free	= imx_rpmsg_pcm_hw_free,
	.trigger	= imx_rpmsg_pcm_trigger,
	.pointer	= imx_rpmsg_pcm_pointer,
	.mmap		= imx_rpmsg_pcm_mmap,
	.ack		= imx_rpmsg_pcm_ack,
	.prepare	= imx_rpmsg_pcm_prepare,
};

static int imx_rpmsg_pcm_preallocate_dma_buffer(struct snd_pcm *pcm,
	int stream, int size)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_wc(pcm->card->dev, size,
				 &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;
	return 0;
}

static void imx_rpmsg_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = SNDRV_PCM_STREAM_PLAYBACK;
			stream < SNDRV_PCM_STREAM_LAST; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_wc(pcm->card->dev, buf->bytes,
			    buf->area, buf->addr);
		buf->area = NULL;
	}
}

static int imx_rpmsg_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info *i2s_info =  &rpmsg_i2s->i2s_info;
	int ret;

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = imx_rpmsg_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK,
			i2s_info->prealloc_buffer_size);
		if (ret)
			goto out;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = imx_rpmsg_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE,
			i2s_info->prealloc_buffer_size);
		if (ret)
			goto out;
	}

out:
	/* free preallocated buffers in case of error */
	if (ret)
		imx_rpmsg_pcm_free_dma_buffers(pcm);

	return ret;
}

static struct snd_soc_component_driver imx_rpmsg_soc_component = {
	.name		= DRV_NAME,
	.ops		= &imx_rpmsg_pcm_ops,
	.pcm_new	= imx_rpmsg_pcm_new,
	.pcm_free	= imx_rpmsg_pcm_free_dma_buffers,
};

int imx_rpmsg_platform_register(struct device *dev)
{
	struct fsl_rpmsg_i2s  *rpmsg_i2s = dev_get_drvdata(dev);
	struct snd_soc_component *component;
	int ret;

	i2s_info_g	=  &rpmsg_i2s->i2s_info;

	ret = devm_snd_soc_register_component(dev, &imx_rpmsg_soc_component,
					       NULL, 0);
	if (ret)
		return ret;

	component = snd_soc_lookup_component(dev, DRV_NAME);
	if (!component)
		return -EINVAL;

#ifdef CONFIG_DEBUG_FS
	component->debugfs_prefix = "dma";
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(imx_rpmsg_platform_register);

MODULE_LICENSE("GPL");
