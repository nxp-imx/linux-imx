// SPDX-License-Identifier: GPL-2.0+
//
// DSP driver compress implementation
//
// Copyright (c) 2012-2013 by Tensilica Inc. ALL RIGHTS RESERVED.
// Copyright 2018-2020 NXP

#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include <sound/core.h>
#include <sound/compress_driver.h>

#include "fsl_dsp.h"
#include "fsl_dsp_platform.h"
#include "fsl_dsp_xaf_api.h"

#define NUM_CODEC 3
#define MIN_FRAGMENT 1
#define MAX_FRAGMENT 1
#define MIN_FRAGMENT_SIZE (4 * 1024)
#define MAX_FRAGMENT_SIZE (4 * 1024)

void dsp_platform_process(struct work_struct *w)
{
	struct xf_client *client = container_of(w, struct xf_client, work);
	struct xf_proxy  *proxy  = client->proxy;
	struct xf_message *rmsg;

	while (1) {
		rmsg = xf_cmd_recv(proxy, &client->wait, &client->queue, 1);

		if (!proxy->is_active || IS_ERR(rmsg))
			return;
		if (rmsg->opcode == XF_EMPTY_THIS_BUFFER) {
			client->consume_bytes += rmsg->length;
			atomic_inc(&client->buffer_cnt);
			snd_compr_fragment_elapsed(client->cstream);

			if (rmsg->buffer == NULL && rmsg->length == 0)
				snd_compr_drain_notify(client->cstream);

		} else {
			memcpy(&client->m, rmsg, sizeof(struct xf_message));
			complete(&client->compr_complete);
		}

		xf_msg_free(proxy, rmsg);
		xf_unlock(&proxy->lock);
	}
}

static int dsp_platform_compr_open(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_dsp  *dsp_priv = snd_soc_component_get_drvdata(component);
	struct dsp_data *drv = &dsp_priv->dsp_data;

	if (drv->client)
		return -EBUSY;
	drv->client = xf_client_alloc(dsp_priv);
	if (IS_ERR(drv->client))
		return PTR_ERR(drv->client);

	fsl_dsp_open_func(dsp_priv, drv->client);

	drv->client->proxy = &dsp_priv->proxy;

	cpu_dai->driver->ops->startup(NULL, cpu_dai);

	drv->client->cstream = cstream;

	INIT_WORK(&drv->client->work, dsp_platform_process);

	return 0;
}

static int dsp_platform_compr_free(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_dsp  *dsp_priv = snd_soc_component_get_drvdata(component);
	struct dsp_data *drv = &dsp_priv->dsp_data;
	struct xf_proxy *p_proxy = &dsp_priv->proxy;
	int ret;

	if (cstream->runtime->state != SNDRV_PCM_STATE_PAUSED &&
		cstream->runtime->state != SNDRV_PCM_STATE_DRAINING) {
		if (dsp_priv->dsp_is_lpa) {
			ret = xaf_comp_flush(drv->client, &drv->component[0]);
			if (ret) {
				dev_err(component->dev, "Fail to flush component, err = %d\n", ret);
				return ret;
			}

			ret = xaf_comp_flush(drv->client, &drv->component[1]);
			if (ret) {
				dev_err(component->dev, "Fail to flush component, err = %d\n", ret);
				return ret;
			}
		}

		ret = xaf_comp_delete(drv->client, &drv->component[1]);
		if (ret) {
			dev_err(component->dev, "Fail to delete component, err = %d\n", ret);
			return ret;
		}

		ret = xaf_comp_delete(drv->client, &drv->component[0]);
		if (ret) {
			dev_err(component->dev, "Fail to delete component, err = %d\n", ret);
			return ret;
		}
		xf_pool_free(drv->client, p_proxy->aux);
	}

	cpu_dai->driver->ops->shutdown(NULL, cpu_dai);

	drv->client->proxy->is_active = 0;
	wake_up(&drv->client->wait);
	cancel_work_sync(&drv->client->work);

	fsl_dsp_close_func(drv->client);
	drv->client = NULL;

	return 0;
}

static int dsp_platform_compr_set_params(struct snd_compr_stream *cstream,
                                        struct snd_compr_params *params)
{
	/* accroding to the params, load the library and create component*/
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct fsl_dsp  *dsp_priv = snd_soc_component_get_drvdata(component);
	struct dsp_data *drv = &dsp_priv->dsp_data;
	struct xf_proxy *p_proxy = &dsp_priv->proxy;
	struct xf_set_param_msg s_param;
	int ret;

	switch (params->codec.id) {
	case SND_AUDIOCODEC_PCM:
		drv->codec_type = CODEC_PCM_DEC;
		atomic_set(&drv->client->buffer_cnt, 2);
		break;
	case SND_AUDIOCODEC_MP3:
		drv->codec_type = CODEC_MP3_DEC;
		break;
	case SND_AUDIOCODEC_AAC:
		drv->codec_type = CODEC_AAC_DEC;
		break;
	default:
		dev_err(component->dev, "codec not supported, id =%d\n", params->codec.id);
		return -EINVAL;
	}

	/* ...create auxiliary buffers pool for control commands */
	ret = xf_pool_alloc(drv->client,
				p_proxy,
				XA_AUX_POOL_SIZE,
				XA_AUX_POOL_MSG_LENGTH,
				XF_POOL_AUX,
				&p_proxy->aux);
	if (ret) {
		dev_err(component->dev, "xf_pool_alloc failed");
		return ret;
	}

	/* ...create pipeline */
	ret = xaf_pipeline_create(&drv->pipeline);
	if (ret) {
		dev_err(component->dev, "create pipeline error\n");
		goto err_pool_alloc;
	}

	/* ...create component */
	ret = xaf_comp_create(drv->client, p_proxy, &drv->component[0],
			      drv->codec_type);
	if (ret) {
		dev_err(component->dev,
			"create component failed type = %d, err = %d\n",
			drv->codec_type, ret);
		goto err_pool_alloc;
	}
	if (sysfs_streq(dsp_priv->audio_iface, "sai"))
		drv->renderer_type = RENDER_SAI;
	else
		drv->renderer_type = RENDER_ESAI;
	ret = xaf_comp_create(drv->client, p_proxy, &drv->component[1],
			      drv->renderer_type);
	if (ret) {
		dev_err(component->dev,
			"create component failed, type = %d, err = %d\n",
			drv->renderer_type, ret);
		goto err_comp0_create;
	}

	/* ...add component into pipeline */
	ret = xaf_comp_add(&drv->pipeline, &drv->component[0]);
	if (ret) {
		dev_err(component->dev,
			"add component failed, type = %d, err = %d\n",
			drv->codec_type, ret);
		goto err_comp0_create;
	}

	ret = xaf_comp_add(&drv->pipeline, &drv->component[1]);
	if (ret) {
		dev_err(component->dev,
			"add component failed, type = %d, err = %d\n",
			drv->renderer_type, ret);
		goto err_comp1_create;
	}

	ret = xaf_connect(drv->client,
			&drv->component[0],
			&drv->component[1],
			1,
			OUTBUF_SIZE);
	if (ret) {
		dev_err(component->dev, "Failed to connect component, err = %d\n", ret);
		goto err_comp1_create;
	}

	drv->client->input_bytes = 0;
	drv->client->consume_bytes = 0;
	drv->client->offset = 0;
	drv->client->ping_pong_offset = 0;

	if (drv->codec_type == CODEC_PCM_DEC) {
		s_param.id = XA_PCM_CONFIG_PARAM_IN_PCM_WIDTH;
		if (params->codec.format == SNDRV_PCM_FORMAT_S32_LE)
			s_param.mixData.value = 32;
		else
			s_param.mixData.value = 16;
		ret = xaf_comp_set_config(drv->client, &drv->component[0], 1, &s_param);
		if (ret) {
			dev_err(component->dev,
				"set param[cmd:0x%x|val:0x%x] error, err = %d\n",
				s_param.id, s_param.mixData.value, ret);
			goto err_comp1_create;
		}
	}

	s_param.id = XA_RENDERER_CONFIG_PARAM_SAMPLE_RATE;
	s_param.mixData.value = params->codec.sample_rate;
	ret = xaf_comp_set_config(drv->client, &drv->component[1], 1, &s_param);
	if (ret) {
		dev_err(component->dev,
			"set param[cmd:0x%x|val:0x%x] error, err = %d\n",
			s_param.id, s_param.mixData.value, ret);
		goto err_comp1_create;
	}

	s_param.id = XA_RENDERER_CONFIG_PARAM_CHANNELS;
	s_param.mixData.value = params->codec.ch_out;
	ret = xaf_comp_set_config(drv->client, &drv->component[1], 1, &s_param);
	if (ret) {
		dev_err(component->dev,
			"set param[cmd:0x%x|val:0x%x] error, err = %d\n",
			s_param.id, s_param.mixData.value, ret);
		goto err_comp1_create;
	}

	s_param.id = XA_RENDERER_CONFIG_PARAM_PCM_WIDTH;
	s_param.mixData.value = 16;
	ret = xaf_comp_set_config(drv->client, &drv->component[1], 1, &s_param);
	if (ret) {
		dev_err(component->dev,
			"set param[cmd:0x%x|val:0x%x] error, err = %d\n",
			s_param.id, s_param.mixData.value, ret);
		goto err_comp1_create;
	}
	return 0;

err_comp1_create:
	xaf_comp_delete(drv->client, &drv->component[1]);
err_comp0_create:
	xaf_comp_delete(drv->client, &drv->component[0]);
err_pool_alloc:
	xf_pool_free(drv->client, p_proxy->aux);

	return ret;
}

static int dsp_platform_compr_trigger_start(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct fsl_dsp *dsp_priv = snd_soc_component_get_drvdata(component);
	struct dsp_data *drv = &dsp_priv->dsp_data;
	struct xaf_comp *p_comp = &drv->component[0];
	int ret;

	if (!dsp_priv->dsp_is_lpa) {
		ret = xaf_comp_process(drv->client,
				p_comp,
				p_comp->inptr,
				drv->client->input_bytes,
				XF_EMPTY_THIS_BUFFER);

		schedule_work(&drv->client->work);
	}

	return 0;
}

static int dsp_platform_compr_trigger_stop(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct fsl_dsp *dsp_priv = snd_soc_component_get_drvdata(component);
	struct dsp_data *drv = &dsp_priv->dsp_data;
	int ret;

	ret = xaf_comp_flush(drv->client, &drv->component[0]);
	if (ret) {
		dev_err(component->dev, "Fail to flush component, err = %d\n", ret);
		return ret;
	}

	ret = xaf_comp_flush(drv->client, &drv->component[1]);
	if (ret) {
		dev_err(component->dev, "Fail to flush component, err = %d\n", ret);
		return ret;
	}
	drv->client->input_bytes = 0;
	drv->client->consume_bytes = 0;
	drv->client->offset = 0;
	drv->client->ping_pong_offset = 0;

	return 0;
}

static int dsp_platform_compr_trigger_drain(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct fsl_dsp *dsp_priv = snd_soc_component_get_drvdata(component);
	struct dsp_data *drv = &dsp_priv->dsp_data;
	struct xaf_comp *p_comp = &drv->component[0];
	int ret;

	ret = xaf_comp_process(drv->client, p_comp, NULL, 0,
			       XF_EMPTY_THIS_BUFFER);

	schedule_work(&drv->client->work);
	return 0;
}

static int dsp_platform_compr_trigger_pause(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct fsl_dsp *dsp_priv = snd_soc_component_get_drvdata(component);
	struct xf_proxy  *proxy  = &dsp_priv->proxy;
	int ret;

	ret = xf_cmd_send_pause(proxy);
	if (ret) {
		dev_err(component->dev, "trigger pause err = %d\n", ret);
		return ret;
	}
	return 0;
}

static int dsp_platform_compr_trigger_pause_release(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct fsl_dsp *dsp_priv = snd_soc_component_get_drvdata(component);
	struct xf_proxy  *proxy  = &dsp_priv->proxy;
	int ret;

	ret = xf_cmd_send_pause_release(proxy);
	if (ret) {
		dev_err(component->dev, "trigger pause release err = %d\n", ret);
		return ret;
	}

	return 0;
}

static int dsp_platform_compr_trigger(struct snd_compr_stream *cstream, int cmd)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = dsp_platform_compr_trigger_start(cstream);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		ret = dsp_platform_compr_trigger_stop(cstream);
		break;
	case SND_COMPR_TRIGGER_DRAIN:
		ret = dsp_platform_compr_trigger_drain(cstream);
		break;
	case SND_COMPR_TRIGGER_PARTIAL_DRAIN:
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = dsp_platform_compr_trigger_pause(cstream);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = dsp_platform_compr_trigger_pause_release(cstream);
		break;
	}

	/*send command*/
	return ret;
}

static int dsp_platform_compr_pointer(struct snd_compr_stream *cstream,
					struct snd_compr_tstamp *tstamp)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct fsl_dsp *dsp_priv = snd_soc_component_get_drvdata(component);
	struct dsp_data *drv = &dsp_priv->dsp_data;
	struct xf_get_param_msg g_param[2];
	int ret;

	g_param[0].id = XA_RENDERER_CONFIG_PARAM_SAMPLE_RATE;
	g_param[1].id = XA_RENDERER_CONFIG_PARAM_CONSUMED;
	ret = xaf_comp_get_config(drv->client, &drv->component[1], 2, &g_param);
	if (ret) {
		dev_err(component->dev,
			"get param[cmd:0x%x|val:0x%x] error, err = %d\n",
			g_param[0].id, g_param[0].mixData.value, ret);
		goto out;
	}

	if ((drv->codec_type != CODEC_PCM_DEC && drv->client->input_bytes != drv->client->consume_bytes)
			|| (drv->codec_type == CODEC_PCM_DEC && atomic_read(&drv->client->buffer_cnt) <= 0))
		tstamp->copied_total = drv->client->input_bytes+drv->client->offset-4096;
	else
		tstamp->copied_total = drv->client->input_bytes+drv->client->offset;
	tstamp->byte_offset = drv->client->input_bytes;
	tstamp->pcm_frames = 0x900;
	tstamp->pcm_io_frames = g_param[1].mixData.value;
	tstamp->sampling_rate = g_param[0].mixData.value;

out:
	return 0;
}

static int dsp_platform_compr_copy(struct snd_compr_stream *cstream,
					char __user *buf,
					size_t count)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct fsl_dsp *dsp_priv = snd_soc_component_get_drvdata(component);
	struct dsp_data *drv = &dsp_priv->dsp_data;
	struct xaf_comp *p_comp = &drv->component[0];
	int copied = 0;
	int ret;

	if (drv->client->input_bytes == drv->client->consume_bytes) {
		if (count > INBUF_SIZE){
			ret = copy_from_user(p_comp->inptr, buf, INBUF_SIZE);
			if (ret) {
				dev_err(component->dev, "failed to get message from user space\n");
				return -EFAULT;
			}
			copied = INBUF_SIZE;
		} else {
			ret = copy_from_user(p_comp->inptr, buf, count);
			if (ret) {
				dev_err(component->dev, "failed to get message from user space\n");
				return -EFAULT;
			}
			copied = count;
		}
		drv->client->input_bytes += copied;

		if (cstream->runtime->state == SNDRV_PCM_STATE_RUNNING && copied) {
		        ret = xaf_comp_process(drv->client, p_comp,
					       p_comp->inptr, copied,
					       XF_EMPTY_THIS_BUFFER);
			schedule_work(&drv->client->work);
		}
	}

	return copied;
}

static int dsp_platform_compr_lpa_pcm_copy(struct snd_compr_stream *cstream,
					char __user *buf,
					size_t count)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct fsl_dsp *dsp_priv = snd_soc_component_get_drvdata(component);
	struct dsp_data *drv = &dsp_priv->dsp_data;
	struct xaf_comp *p_comp = &drv->component[0];
	int copied = 0;
	int ret;

	if (atomic_read(&drv->client->buffer_cnt) > 0) {
		if (drv->client->offset+count >= (INBUF_SIZE_LPA_PCM>>1)-4096 || !buf) {
			/* buf == NULL and count == 1 is for drain and */
			/* suspend as tinycompress drain is blocking call */
			copied = count;
			if (!buf)
				copied = 0;
			if (buf) {
				ret = copy_from_user(p_comp->inptr+drv->client->ping_pong_offset+drv->client->offset, buf, copied);
				if (ret) {
					dev_err(component->dev, "failed to get message from user space\n");
					return -EFAULT;
				}
			}

			if (cstream->runtime->state == SNDRV_PCM_STATE_RUNNING) {
				ret = xaf_comp_process(drv->client, p_comp,
						p_comp->inptr+drv->client->ping_pong_offset, drv->client->offset+copied,
						XF_EMPTY_THIS_BUFFER);

				schedule_work(&drv->client->work);
				drv->client->input_bytes += drv->client->offset+copied;
				drv->client->offset = 0;
				atomic_dec(&drv->client->buffer_cnt);
				if (drv->client->ping_pong_offset)
					drv->client->ping_pong_offset = 0;
				else
					drv->client->ping_pong_offset = INBUF_SIZE_LPA_PCM>>1;
			}
			if (!buf)
				copied = count;
		} else {
			ret = copy_from_user(p_comp->inptr+drv->client->ping_pong_offset+drv->client->offset, buf, count);
			if (ret) {
				dev_err(component->dev, "failed to get message from user space\n");
				return -EFAULT;
			}
			copied = count;
			drv->client->offset += copied;
		}
	}

	return copied;
}

static int dsp_platform_compr_lpa_copy(struct snd_compr_stream *cstream,
					char __user *buf,
					size_t count)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, FSL_DSP_COMP_NAME);
	struct fsl_dsp *dsp_priv = snd_soc_component_get_drvdata(component);
	struct dsp_data *drv = &dsp_priv->dsp_data;
	struct xaf_comp *p_comp = &drv->component[0];
	int copied = 0;
	int ret;

	if (drv->codec_type == CODEC_PCM_DEC)
		return dsp_platform_compr_lpa_pcm_copy(cstream, buf, count);

	if (drv->client->input_bytes == drv->client->consume_bytes) {
		if (drv->client->offset+count >= INBUF_SIZE_LPA-4096 || !buf) {
			/* buf == NULL and count == 1 is for drain and */
			/* suspend as tinycompress drain is blocking call */
			copied = count;
			if (!buf)
				copied = 0;
			if (buf) {
				ret = copy_from_user(p_comp->inptr+drv->client->offset, buf, copied);
				if (ret) {
					dev_err(component->dev, "failed to get message from user space\n");
					return -EFAULT;
				}
			}

			if (cstream->runtime->state == SNDRV_PCM_STATE_RUNNING) {
				ret = xaf_comp_process(drv->client, p_comp,
						p_comp->inptr, drv->client->offset+copied,
						XF_EMPTY_THIS_BUFFER);

				schedule_work(&drv->client->work);
				drv->client->input_bytes += drv->client->offset+copied;
				drv->client->offset = 0;
			}
			if (!buf)
				copied = count;
		} else {
			ret = copy_from_user(p_comp->inptr+drv->client->offset, buf, count);
			if (ret) {
				dev_err(component->dev, "failed to get message from user space\n");
				return -EFAULT;
			}
			copied = count;
			drv->client->offset += copied;
		}
	}

	return copied;
}

static int dsp_platform_compr_get_caps(struct snd_compr_stream *cstream,
					struct snd_compr_caps *caps)
{
	caps->num_codecs = NUM_CODEC;
	caps->min_fragment_size = MIN_FRAGMENT_SIZE;  /* 50KB */
	caps->max_fragment_size = MAX_FRAGMENT_SIZE;  /* 1024KB */
	caps->min_fragments = MIN_FRAGMENT;
	caps->max_fragments = MAX_FRAGMENT;
	caps->codecs[0] = SND_AUDIOCODEC_MP3;
	caps->codecs[1] = SND_AUDIOCODEC_AAC;
	caps->codecs[2] = SND_AUDIOCODEC_PCM;

	return 0;
}

static struct snd_compr_codec_caps caps_pcm = {
	.num_descriptors = 1,
	.descriptor[0].max_ch = 2,
	.descriptor[0].sample_rates[0] = 192000,
	.descriptor[0].sample_rates[1] = 176400,
	.descriptor[0].sample_rates[2] = 96000,
	.descriptor[0].sample_rates[3] = 88200,
	.descriptor[0].sample_rates[4] = 48000,
	.descriptor[0].sample_rates[5] = 44100,
	.descriptor[0].sample_rates[6] = 32000,
	.descriptor[0].sample_rates[7] = 16000,
	.descriptor[0].sample_rates[8] = 8000,
	.descriptor[0].num_sample_rates = 9,
	.descriptor[0].bit_rate[0] = 320,
	.descriptor[0].bit_rate[1] = 192,
	.descriptor[0].num_bitrates = 2,
	.descriptor[0].profiles = SND_AUDIOPROFILE_PCM,
	.descriptor[0].modes = 0,
	.descriptor[0].formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
};

static struct snd_compr_codec_caps caps_mp3 = {
	.num_descriptors = 1,
	.descriptor[0].max_ch = 2,
	.descriptor[0].sample_rates[0] = 48000,
	.descriptor[0].sample_rates[1] = 44100,
	.descriptor[0].sample_rates[2] = 32000,
	.descriptor[0].sample_rates[3] = 16000,
	.descriptor[0].sample_rates[4] = 8000,
	.descriptor[0].num_sample_rates = 5,
	.descriptor[0].bit_rate[0] = 320,
	.descriptor[0].bit_rate[1] = 192,
	.descriptor[0].num_bitrates = 2,
	.descriptor[0].profiles = 0,
	.descriptor[0].modes = SND_AUDIOCHANMODE_MP3_STEREO,
	.descriptor[0].formats = 0,
};

static struct snd_compr_codec_caps caps_aac = {
	.num_descriptors = 2,
	.descriptor[1].max_ch = 2,
	.descriptor[0].sample_rates[0] = 48000,
	.descriptor[0].sample_rates[1] = 44100,
	.descriptor[0].sample_rates[2] = 32000,
	.descriptor[0].sample_rates[3] = 16000,
	.descriptor[0].sample_rates[4] = 8000,
	.descriptor[0].num_sample_rates = 5,
	.descriptor[1].bit_rate[0] = 320,
	.descriptor[1].bit_rate[1] = 192,
	.descriptor[1].num_bitrates = 2,
	.descriptor[1].profiles = 0,
	.descriptor[1].modes = 0,
	.descriptor[1].formats =
			(SND_AUDIOSTREAMFORMAT_MP4ADTS |
				SND_AUDIOSTREAMFORMAT_RAW),
};

static int dsp_platform_compr_get_codec_caps(struct snd_compr_stream *cstream,
						struct snd_compr_codec_caps *codec)
{
	if (codec->codec == SND_AUDIOCODEC_MP3)
		*codec = caps_mp3;
	else if (codec->codec == SND_AUDIOCODEC_AAC)
		*codec = caps_aac;
	else if (codec->codec == SND_AUDIOCODEC_PCM)
		*codec = caps_pcm;
	else
		return -EINVAL;

	return 0;
}

static int dsp_platform_compr_set_metadata(struct snd_compr_stream *cstream,
						struct snd_compr_metadata *metadata)
{
	return 0;
}

const struct snd_compr_ops dsp_platform_compr_ops = {
	.open = dsp_platform_compr_open,
	.free = dsp_platform_compr_free,
	.set_params = dsp_platform_compr_set_params,
	.set_metadata = dsp_platform_compr_set_metadata,
	.trigger = dsp_platform_compr_trigger,
	.pointer = dsp_platform_compr_pointer,
	.copy = dsp_platform_compr_copy,
	.get_caps = dsp_platform_compr_get_caps,
	.get_codec_caps = dsp_platform_compr_get_codec_caps,
};

const struct snd_compr_ops dsp_platform_compr_lpa_ops = {
	.open = dsp_platform_compr_open,
	.free = dsp_platform_compr_free,
	.set_params = dsp_platform_compr_set_params,
	.set_metadata = dsp_platform_compr_set_metadata,
	.trigger = dsp_platform_compr_trigger,
	.pointer = dsp_platform_compr_pointer,
	.copy = dsp_platform_compr_lpa_copy,
	.get_caps = dsp_platform_compr_get_caps,
	.get_codec_caps = dsp_platform_compr_get_codec_caps,
};
