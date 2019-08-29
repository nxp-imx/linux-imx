// SPDX-License-Identifier: GPL-2.0
// Copyright 2019 NXP

struct fsl_easrc_m2m {
	struct fsl_easrc *easrc;
	struct fsl_easrc_context *ctx;
	struct completion complete[2];
	struct dma_block dma_block[2];
	unsigned int ctx_hold;
	unsigned int easrc_active;
	unsigned int first_convert;
	unsigned int sg_nodes[2];
	struct scatterlist sg[2][9];
	struct dma_async_tx_descriptor *desc[2];
	spinlock_t lock;  /* protect mem resource */
};

void fsl_easrc_get_status(struct fsl_easrc_context *ctx,
			  struct asrc_status_flags *flags)
{
	flags->overload_error = 0;
}

#define mxc_easrc_dma_umap_in(dev, m2m) \
	dma_unmap_sg(dev, m2m->sg[IN], m2m->sg_nodes[IN], \
					DMA_MEM_TO_DEV) \

#define mxc_easrc_dma_umap_out(dev, m2m) \
	dma_unmap_sg(dev, m2m->sg[OUT], m2m->sg_nodes[OUT], \
					DMA_DEV_TO_MEM) \

#define EASRC_xPUT_DMA_CALLBACK(dir) \
	((dir == IN) ? fsl_easrc_input_dma_callback \
			: fsl_easrc_output_dma_callback)

#define DIR_STR(dir) dir == IN ? "in" : "out"

static void fsl_easrc_input_dma_callback(void *data)
{
	struct fsl_easrc_m2m *m2m = (struct fsl_easrc_m2m *)data;

	complete(&m2m->complete[IN]);
}

static void fsl_easrc_output_dma_callback(void *data)
{
	struct fsl_easrc_m2m *m2m = (struct fsl_easrc_m2m *)data;

	complete(&m2m->complete[OUT]);
}

static int fsl_allocate_dma_buf(struct fsl_easrc_m2m *m2m)
{
	struct dma_block *input = &m2m->dma_block[IN];
	struct dma_block *output = &m2m->dma_block[OUT];

	input->dma_vaddr = kzalloc(input->length, GFP_KERNEL);
	if (!input->dma_vaddr)
		return -ENOMEM;

	output->dma_vaddr = kzalloc(output->length, GFP_KERNEL);
	if (!output->dma_vaddr)
		goto alloc_fail;

	return 0;

alloc_fail:
	kfree(input->dma_vaddr);

	return -ENOMEM;
}

static int fsl_easrc_dmaconfig(struct fsl_easrc_m2m *m2m,
			       struct dma_chan *chan,
			       u32 dma_addr, void *buf_addr, u32 buf_len,
			       bool dir, int bits)
{
	struct dma_async_tx_descriptor *desc = m2m->desc[dir];
	struct fsl_easrc *easrc = m2m->easrc;
	struct fsl_easrc_context *ctx = m2m->ctx;
	struct device *dev = &easrc->pdev->dev;
	unsigned int sg_nent = m2m->sg_nodes[dir];
	struct scatterlist *sg = m2m->sg[dir];
	struct dma_slave_config slave_config;
	enum dma_slave_buswidth buswidth;
	int ret, i;

	switch (bits) {
	case 16:
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case 24:
		buswidth = DMA_SLAVE_BUSWIDTH_3_BYTES;
		break;
	default:
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;
	}

	if (dir == IN) {
		slave_config.direction = DMA_MEM_TO_DEV;
		slave_config.dst_addr = dma_addr;
		slave_config.dst_addr_width = buswidth;
		slave_config.dst_maxburst =
			ctx->in_params.fifo_wtmk * ctx->channels;
	} else {
		slave_config.direction = DMA_DEV_TO_MEM;
		slave_config.src_addr = dma_addr;
		slave_config.src_addr_width = buswidth;
		slave_config.src_maxburst =
			ctx->out_params.fifo_wtmk * ctx->channels;
	}

	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret) {
		dev_err(dev, "failed to config dmaengine for %sput task: %d\n",
			DIR_STR(dir), ret);
		return -EINVAL;
	}

	sg_init_table(sg, sg_nent);
	switch (sg_nent) {
	case 1:
		sg_init_one(sg, buf_addr, buf_len);
		break;
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
		for (i = 0; i < (sg_nent - 1); i++)
			sg_set_buf(&sg[i],
				   buf_addr + i * m2m->dma_block[dir].max_buf_size,
				   m2m->dma_block[dir].max_buf_size);

		sg_set_buf(&sg[i],
			   buf_addr + i * m2m->dma_block[dir].max_buf_size,
			   buf_len - i * m2m->dma_block[dir].max_buf_size);
		break;
	default:
		dev_err(dev, "invalid input DMA nodes number: %d\n", sg_nent);
		return -EINVAL;
	}

	ret = dma_map_sg(dev, sg, sg_nent, slave_config.direction);
	if (ret != sg_nent) {
		dev_err(dev, "failed to map DMA sg for %sput task\n",
			DIR_STR(dir));
		return -EINVAL;
	}

	desc = dmaengine_prep_slave_sg(chan, sg, sg_nent,
				       slave_config.direction,
				       DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(dev, "failed to prepare dmaengine for %sput task\n",
			DIR_STR(dir));
		return -EINVAL;
	}

	m2m->desc[dir] = desc;
	m2m->desc[dir]->callback = EASRC_xPUT_DMA_CALLBACK(dir);

	desc->callback = EASRC_xPUT_DMA_CALLBACK(dir);
	desc->callback_param = m2m;

	return 0;
}

static long fsl_easrc_calc_outbuf_len(struct fsl_easrc_m2m *m2m,
				      struct asrc_convert_buffer *pbuf)
{
	struct fsl_easrc_context *ctx = m2m->ctx;
	unsigned int out_length;
	unsigned int in_width, out_width;
	unsigned int channels = ctx->channels;
	unsigned int in_samples, out_samples;

	in_width = snd_pcm_format_physical_width(ctx->in_params.sample_format) / 8;
	out_width = snd_pcm_format_physical_width(ctx->out_params.sample_format) / 8;

	in_samples = pbuf->input_buffer_length / (in_width * channels);
	out_samples = ctx->out_params.sample_rate * in_samples /
				ctx->in_params.sample_rate;
	out_length = out_samples * out_width * channels;

	if (out_samples <= ctx->out_missed_sample) {
		out_length = 0;
		ctx->out_missed_sample -= out_samples;
	} else {
		out_length -= ctx->out_missed_sample * out_width * channels;
		ctx->out_missed_sample = 0;
	}

	return out_length;
}

static long fsl_easrc_prepare_io_buffer(struct fsl_easrc_m2m *m2m,
					struct asrc_convert_buffer *buf,
					bool dir)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct device *dev = &easrc->pdev->dev;
	struct fsl_easrc_context *ctx = m2m->ctx;
	struct dma_chan *dma_chan = ctx->dma_chan[dir];
	unsigned int *dma_len = &m2m->dma_block[dir].length;
	unsigned int *sg_nodes = &m2m->sg_nodes[dir];
	void *dma_vaddr = m2m->dma_block[dir].dma_vaddr;
	enum asrc_pair_index index = m2m->ctx->index;
	unsigned int buf_len, bits;
	u32 fifo_addr;
	void __user *buf_vaddr;

	if (dir == IN) {
		buf_vaddr = (void __user *)buf->input_buffer_vaddr;
		buf_len = buf->input_buffer_length;
		bits = snd_pcm_format_physical_width(ctx->in_params.sample_format);
		fifo_addr = easrc->paddr + REG_EASRC_WRFIFO(index);
	} else {
		buf_vaddr = (void __user *)buf->output_buffer_vaddr;
		buf_len = buf->output_buffer_length;
		bits = snd_pcm_format_physical_width(ctx->out_params.sample_format);
		fifo_addr = easrc->paddr + REG_EASRC_RDFIFO(index);
	}

	if (buf_len > EASRC_DMA_BUFFER_SIZE ||
	    (dir == IN && (buf_len % (bits / 8)))) {
		dev_err(dev, "%sput buffer size is error: [%d]\n",
			DIR_STR(dir), buf_len);
		return -EINVAL;
	}

	if (dir == IN && copy_from_user(dma_vaddr, buf_vaddr, buf_len))
		return -EFAULT;

	*dma_len = buf_len;

	if (dir == OUT)
		*dma_len = fsl_easrc_calc_outbuf_len(m2m, buf);

	if (*dma_len <= 0)
		return 0;

	*sg_nodes = *dma_len / m2m->dma_block[dir].max_buf_size + 1;

	return fsl_easrc_dmaconfig(m2m, dma_chan, fifo_addr, dma_vaddr,
				*dma_len, dir, bits);
}

static long fsl_easrc_prepare_buffer(struct fsl_easrc_m2m *m2m,
				     struct asrc_convert_buffer *buf)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct device *dev = &easrc->pdev->dev;
	int ret;

	ret = fsl_easrc_prepare_io_buffer(m2m, buf, IN);
	if (ret) {
		dev_err(dev, "failed to prepare input buffer %d\n", ret);
		return ret;
	}

	ret = fsl_easrc_prepare_io_buffer(m2m, buf, OUT);
	if (ret) {
		dev_err(dev, "failed to prepare output buffer %d\n", ret);
		return ret;
	}

	return 0;
}

int fsl_easrc_process_buffer_pre(struct fsl_easrc_m2m *m2m, bool dir)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct device *dev = &easrc->pdev->dev;

	if (!wait_for_completion_interruptible_timeout(&m2m->complete[dir],
						       10 * HZ)) {
		dev_err(dev, "%sput DMA task timeout\n", DIR_STR(dir));
		return -ETIME;
	} else if (signal_pending(current)) {
		dev_err(dev, "%sput task forcibly aborted\n", DIR_STR(dir));
		return -ERESTARTSYS;
	}

	return 0;
}

static unsigned int fsl_easrc_get_output_FIFO_size(struct fsl_easrc_m2m *m2m)
{
	struct fsl_easrc *easrc = m2m->easrc;
	enum asrc_pair_index index = m2m->ctx->index;
	u32 val;

	regmap_read(easrc->regmap, REG_EASRC_SFS(index), &val);

	val &= EASRC_SFS_NSGO_MASK;

	return val >> EASRC_SFS_NSGO_SHIFT;
}

static void fsl_easrc_read_last_FIFO(struct fsl_easrc_m2m *m2m)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct dma_block *output = &m2m->dma_block[OUT];
	struct fsl_easrc_context *ctx = m2m->ctx;
	enum asrc_pair_index index = m2m->ctx->index;
	u32 i, reg, size, t_size = 0, width;
	u32 *reg32 = NULL;
	u16 *reg16 = NULL;
	u8  *reg24 = NULL;

	width = snd_pcm_format_physical_width(ctx->out_params.sample_format);

	if (width == 32)
		reg32 = output->dma_vaddr + output->length;
	else if (width == 16)
		reg16 = output->dma_vaddr + output->length;
	else
		reg24 = output->dma_vaddr + output->length;
retry:
	size = fsl_easrc_get_output_FIFO_size(m2m);
	for (i = 0; i < size * ctx->channels; i++) {
		regmap_read(easrc->regmap, REG_EASRC_RDFIFO(index), &reg);

		if (reg32) {
			*(reg32) = reg;
			reg32++;
		} else if (reg16) {
			*(reg16) = (u16)reg;
			reg16++;
		} else {
			*reg24++ = (u8)reg;
			*reg24++ = (u8)(reg >> 8);
			*reg24++ = (u8)(reg >> 16);
		}
	}
	t_size += size;

	if (size)
		goto retry;

	if (reg32)
		output->length += t_size * ctx->channels * 4;
	else if (reg16)
		output->length += t_size * ctx->channels * 2;
	else
		output->length += t_size * ctx->channels * 3;
}

static long fsl_easrc_process_buffer(struct fsl_easrc_m2m *m2m,
				     struct asrc_convert_buffer *buf)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct device *dev = &easrc->pdev->dev;
	unsigned long lock_flags;
	int ret;

	/* Check input task first */
	ret = fsl_easrc_process_buffer_pre(m2m, IN);
	if (ret) {
		mxc_easrc_dma_umap_in(dev, m2m);
		if (m2m->dma_block[OUT].length > 0)
			mxc_easrc_dma_umap_out(dev, m2m);
		return ret;
	}

	/* ...then output task*/
	if (m2m->dma_block[OUT].length > 0) {
		ret = fsl_easrc_process_buffer_pre(m2m, OUT);
		if (ret) {
			mxc_easrc_dma_umap_in(dev, m2m);
			mxc_easrc_dma_umap_out(dev, m2m);
			return ret;
		}
	}

	mxc_easrc_dma_umap_in(dev, m2m);
	if (m2m->dma_block[OUT].length > 0)
		mxc_easrc_dma_umap_out(dev, m2m);

	spin_lock_irqsave(&m2m->lock, lock_flags);
	if (!m2m->ctx_hold) {
		spin_unlock_irqrestore(&m2m->lock, lock_flags);
		return -EFAULT;
	}
	spin_unlock_irqrestore(&m2m->lock, lock_flags);

	/* Fetch the remaining data */
	fsl_easrc_read_last_FIFO(m2m);

	/* Update final lengths after getting last FIFO */
	buf->input_buffer_length = m2m->dma_block[IN].length;
	buf->output_buffer_length = m2m->dma_block[OUT].length;

	if (copy_to_user((void __user *)buf->output_buffer_vaddr,
			 m2m->dma_block[OUT].dma_vaddr,
			 m2m->dma_block[OUT].length))
		return -EFAULT;

	return 0;
}

void fsl_easrc_submit_dma(struct fsl_easrc_m2m *m2m)
{
	/* Submit DMA request */
	dmaengine_submit(m2m->desc[IN]);
	dma_async_issue_pending(m2m->desc[IN]->chan);

	if (m2m->dma_block[OUT].length > 0) {
		dmaengine_submit(m2m->desc[OUT]);
		dma_async_issue_pending(m2m->desc[OUT]->chan);
	}
}

static long fsl_easrc_ioctl_req_context(struct fsl_easrc_m2m *m2m,
					void __user *user)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct device *dev = &easrc->pdev->dev;
	struct asrc_req req;
	unsigned long lock_flags;
	long ret;

	ret = copy_from_user(&req, user, sizeof(req));
	if (ret) {
		dev_err(dev, "failed to get req from user space:%ld\n", ret);
		return ret;
	}

	ret = fsl_easrc_request_context(m2m->ctx, req.chn_num);
	if (ret < 0) {
		dev_err(dev, "failed to request context:%ld\n", ret);
		return ret;
	}

	/* request context returns the context id in case of success */
	spin_lock_irqsave(&m2m->lock, lock_flags);
	m2m->ctx_hold = 1;
	req.index = m2m->ctx->index;
	req.supported_in_format = FSL_EASRC_FORMATS;
	req.supported_out_format = FSL_EASRC_FORMATS |
				   SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE;
	spin_unlock_irqrestore(&m2m->lock, lock_flags);

	ret = copy_to_user(user, &req, sizeof(req));
	if (ret) {
		dev_err(dev, "failed to send req to user space: %ld\n", ret);
		return ret;
	}

	return 0;
}

static long fsl_easrc_ioctl_config_context(struct fsl_easrc_m2m *m2m,
					   void __user *user)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct fsl_easrc_context *ctx = m2m->ctx;
	enum asrc_pair_index index = m2m->ctx->index;
	struct device *dev = &easrc->pdev->dev;
	struct asrc_config config;
	int ret;
	int in_word_size, out_word_size;

	ret = copy_from_user(&config, user, sizeof(config));
	if (ret) {
		dev_err(dev, "failed to get config from user space: %d\n", ret);
		return ret;
	}

	/* set context configuration parameters received from userspace */
	ctx->in_params.sample_rate = config.input_sample_rate;
	ctx->out_params.sample_rate = config.output_sample_rate;

	ctx->in_params.fifo_wtmk = FSL_EASRC_INPUTFIFO_WML;
	ctx->out_params.fifo_wtmk = FSL_EASRC_OUTPUTFIFO_WML;

	ctx->in_params.sample_format = config.input_format;
	ctx->out_params.sample_format = config.output_format;

	ctx->channels = config.channel_num;
	ctx->rs_init_mode = 0x2;
	ctx->pf_init_mode = 0x2;

	ret = fsl_easrc_set_ctx_format(ctx,
				       &ctx->in_params.sample_format,
				       &ctx->out_params.sample_format);
	if (ret)
		return ret;

	ret = fsl_easrc_config_context(easrc, index);
	if (ret) {
		dev_err(dev, "failed to config context %d\n", ret);
		return ret;
	}

	ctx->in_params.iterations = 1;
	ctx->in_params.group_len = ctx->channels;
	ctx->in_params.access_len = ctx->channels;
	ctx->out_params.iterations = 1;
	ctx->out_params.group_len = ctx->channels;
	ctx->out_params.access_len = ctx->channels;

	/* You can also call fsl_easrc_set_ctx_organziation for
	 * sample interleaving support
	 */
	ret = fsl_easrc_set_ctx_organziation(ctx);
	if (ret) {
		dev_err(dev, "failed to set fifo organization\n");
		return ret;
	}

	in_word_size = snd_pcm_format_physical_width(config.input_format) / 8;
	out_word_size = snd_pcm_format_physical_width(config.output_format) / 8;

	/* allocate dma buffers */
	m2m->dma_block[IN].length = EASRC_DMA_BUFFER_SIZE;
	m2m->dma_block[IN].max_buf_size = rounddown(EASRC_MAX_BUFFER_SIZE,
						    in_word_size * ctx->channels);
	m2m->dma_block[OUT].length = EASRC_DMA_BUFFER_SIZE;
	m2m->dma_block[OUT].max_buf_size = rounddown(EASRC_MAX_BUFFER_SIZE,
						     out_word_size * ctx->channels);

	ret = fsl_allocate_dma_buf(m2m);
	if (ret) {
		dev_err(dev, "failed to allocate DMA buffers: %d\n", ret);
		return ret;
	}

	ctx->dma_chan[IN] = fsl_easrc_get_dma_channel(ctx, IN);
	if (!ctx->dma_chan[IN]) {
		dev_err(dev, "[ctx%d] failed to get input DMA channel\n",
			m2m->ctx->index);
		return -EBUSY;
	}
	ctx->dma_chan[OUT] = fsl_easrc_get_dma_channel(ctx, OUT);
	if (!ctx->dma_chan[OUT]) {
		dev_err(dev, "[ctx%d] failed to get output DMA channel\n",
			m2m->ctx->index);
		return -EBUSY;
	}

	ret = copy_to_user(user, &config, sizeof(config));
	if (ret) {
		dev_err(dev, "failed to send config to user: %d\n", ret);
		return ret;
	}

	return 0;
}

static long fsl_easrc_ioctl_release_context(struct fsl_easrc_m2m *m2m,
					    void __user *user)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct fsl_easrc_context *ctx = m2m->ctx;
	struct device *dev = &easrc->pdev->dev;
	enum asrc_pair_index index;
	unsigned long lock_flags;
	int ret;

	ret = copy_from_user(&index, user, sizeof(index));
	if (ret) {
		dev_err(dev,
			"[ctx%d] failed to get index from user space %d\n",
			m2m->ctx->index, ret);
		return ret;
	}

	if (index != m2m->ctx->index) {
		dev_err(dev,
			"[ctx%d] releasing wrong context - %d\n",
			m2m->ctx->index, index);
		return -EINVAL;
	}

	if (m2m->easrc_active) {
		m2m->easrc_active = 0;
		fsl_easrc_stop_context(ctx);
	}

	spin_lock_irqsave(&m2m->lock, lock_flags);
	m2m->ctx_hold = 0;
	spin_unlock_irqrestore(&m2m->lock, lock_flags);

	if (ctx->dma_chan[IN])
		dma_release_channel(ctx->dma_chan[IN]);
	if (ctx->dma_chan[OUT])
		dma_release_channel(ctx->dma_chan[OUT]);

	ctx->dma_chan[IN] = NULL;
	ctx->dma_chan[OUT] = NULL;

	/* free buffers allocated in config context*/
	kfree(m2m->dma_block[IN].dma_vaddr);
	kfree(m2m->dma_block[OUT].dma_vaddr);

	fsl_easrc_release_context(ctx);

	return 0;
}

static long fsl_easrc_ioctl_convert(struct fsl_easrc_m2m *m2m,
				    void __user *user)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct device *dev = &easrc->pdev->dev;
	struct fsl_easrc_context *ctx = m2m->ctx;
	struct asrc_convert_buffer buf;
	int ret;

	ret = copy_from_user(&buf, user, sizeof(buf));
	if (ret) {
		dev_err(dev, "failed to get buf from user space: %d\n", ret);
		return ret;
	}

	/* fsl_easrc_calc_last_period_size(ctx, &buf); */
	ret = fsl_easrc_prepare_buffer(m2m, &buf);
	if (ret) {
		dev_err(dev, "failed to prepare buffer\n");
		return ret;
	}

	init_completion(&m2m->complete[IN]);
	init_completion(&m2m->complete[OUT]);

	fsl_easrc_submit_dma(m2m);

	if (m2m->first_convert) {
		fsl_easrc_start_context(ctx);
		m2m->first_convert = 0;
	}

	ret = fsl_easrc_process_buffer(m2m, &buf);
	if (ret) {
		dev_err(dev, "failed to process buffer %d\n", ret);
		return ret;
	}

	ret = copy_to_user(user, &buf, sizeof(buf));
	if (ret) {
		dev_err(dev, "failed to send buffer to user: %d\n", ret);
		return ret;
	}

	return 0;
}

static long fsl_easrc_ioctl_start_conv(struct fsl_easrc_m2m *m2m,
				       void __user *user)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct device *dev = &easrc->pdev->dev;
	enum asrc_pair_index index;
	int ret;

	ret = copy_from_user(&index, user, sizeof(index));
	if (ret) {
		dev_err(dev, "failed to get index from user space: %d\n",
			ret);
		return ret;
	}

	if (index != m2m->ctx->index) {
		dev_err(dev, "[ctx%d] attempting to start wrong context%d\n",
			m2m->ctx->index, index);
		return -EINVAL;
	}

	m2m->easrc_active = 1;
	m2m->first_convert = 1;

	return 0;
}

static long fsl_easrc_ioctl_stop_conv(struct fsl_easrc_m2m *m2m,
				      void __user *user)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct fsl_easrc_context *ctx = m2m->ctx;
	struct device *dev = &easrc->pdev->dev;
	enum asrc_pair_index index;
	int ret;

	ret = copy_from_user(&index, user, sizeof(index));
	if (ret) {
		dev_err(dev, "failed to get index from user space: %d\n",
			ret);
		return ret;
	}

	if (index != m2m->ctx->index) {
		dev_err(dev, "[ctx%d] attempting to start wrong context%d\n",
			m2m->ctx->index, index);
		return -EINVAL;
	}

	dmaengine_terminate_all(ctx->dma_chan[IN]);
	dmaengine_terminate_all(ctx->dma_chan[OUT]);

	fsl_easrc_stop_context(ctx);
	m2m->easrc_active = 0;

	return 0;
}

static long fsl_easrc_ioctl_status(struct fsl_easrc_m2m *m2m,
				   void __user *user)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct device *dev = &easrc->pdev->dev;
	struct fsl_easrc_context *ctx = m2m->ctx;
	struct asrc_status_flags flags;
	int ret;

	ret = copy_from_user(&flags, user, sizeof(flags));
	if (ret) {
		dev_err(dev,
			"[ctx%d] failed to get flags from user space: %d\n",
			m2m->ctx->index, ret);
		return ret;
	}

	if (m2m->ctx->index != flags.index) {
		dev_err(dev, "[ctx%d] getting status for other context: %d\n",
			m2m->ctx->index, flags.index);
		return -EINVAL;
	}

	fsl_easrc_get_status(ctx, &flags);

	ret = copy_to_user(user, &flags, sizeof(flags));
	if (ret)
		dev_err(dev, "[ctx%d] failed to send flags to user space\n",
			m2m->ctx->index);

	return ret;
}

static long fsl_easrc_ioctl_flush(struct fsl_easrc_m2m *m2m,
				  void __user *user)
{
	struct fsl_easrc *easrc = m2m->easrc;
	struct device *dev = &easrc->pdev->dev;
	struct fsl_easrc_context *ctx = m2m->ctx;

	/* Release DMA and request again */
	dma_release_channel(ctx->dma_chan[IN]);
	dma_release_channel(ctx->dma_chan[OUT]);

	ctx->dma_chan[IN] = fsl_easrc_get_dma_channel(ctx, IN);
	if (!ctx->dma_chan[IN]) {
		dev_err(dev, "failed to request input task DMA channel\n");
		return -EBUSY;
	}

	ctx->dma_chan[OUT] = fsl_easrc_get_dma_channel(ctx, OUT);
	if (!ctx->dma_chan[OUT]) {
		dev_err(dev, "failed to request output task DMA channel\n");
		return -EBUSY;
	}

	return 0;
}

static int fsl_easrc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *easrc_miscdev = file->private_data;
	struct fsl_easrc *easrc = dev_get_drvdata(easrc_miscdev->parent);
	struct fsl_easrc_m2m *m2m;
	struct fsl_easrc_context *ctx;
	struct device *dev = &easrc->pdev->dev;
	int ret;

	ret = signal_pending(current);
	if (ret) {
		dev_err(dev, "current process has a signal pending\n");
		return ret;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	/* set the pointer to easrc private data */
	m2m = kzalloc(sizeof(*m2m), GFP_KERNEL);
	if (!m2m) {
		ret = -ENOMEM;
		goto out;
	}
	/* just save the pointer to easrc private data */
	m2m->easrc = easrc;
	m2m->ctx = ctx;
	ctx->easrc = easrc;
	ctx->private_data = m2m;

	spin_lock_init(&m2m->lock);

	/* context structs are already allocated in fsl_easrc->ctx[i] */
	file->private_data = m2m;

	pm_runtime_get_sync(dev);

	return 0;
out:
	kfree(ctx);
	return ret;
}

static int fsl_easrc_close(struct inode *inode, struct file *file)
{
	struct fsl_easrc_m2m *m2m = file->private_data;
	struct fsl_easrc *easrc = m2m->easrc;
	struct fsl_easrc_context *ctx = m2m->ctx;
	struct device *dev = &easrc->pdev->dev;
	unsigned long lock_flags;

	if (m2m->easrc_active) {
		m2m->easrc_active = 0;
		dmaengine_terminate_all(ctx->dma_chan[IN]);
		dmaengine_terminate_all(ctx->dma_chan[OUT]);

		fsl_easrc_stop_context(ctx);
		fsl_easrc_input_dma_callback((void *)m2m);
		fsl_easrc_output_dma_callback((void *)m2m);
	}

	if (!ctx)
		goto null_ctx;

	spin_lock_irqsave(&m2m->lock, lock_flags);
	if (m2m->ctx_hold) {
		m2m->ctx_hold = 0;
		spin_unlock_irqrestore(&m2m->lock, lock_flags);

		if (ctx->dma_chan[IN])
			dma_release_channel(ctx->dma_chan[IN]);
		if (ctx->dma_chan[OUT])
			dma_release_channel(ctx->dma_chan[OUT]);

		kfree(m2m->dma_block[IN].dma_vaddr);
		kfree(m2m->dma_block[OUT].dma_vaddr);

		fsl_easrc_release_context(ctx);
	} else {
		spin_unlock_irqrestore(&m2m->lock, lock_flags);
	}

null_ctx:
	spin_lock_irqsave(&easrc->lock, lock_flags);
	kfree(m2m);
	kfree(ctx);
	file->private_data = NULL;
	spin_unlock_irqrestore(&easrc->lock, lock_flags);

	pm_runtime_put_sync(dev);

	return 0;
}

static long fsl_easrc_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct fsl_easrc_m2m *m2m = file->private_data;
	struct fsl_easrc *easrc = m2m->easrc;
	void __user *user = (void __user *)arg;
	long ret = 0;

	switch (cmd) {
	case ASRC_REQ_PAIR:
		ret = fsl_easrc_ioctl_req_context(m2m, user);
		break;
	case ASRC_CONFIG_PAIR:
		ret = fsl_easrc_ioctl_config_context(m2m, user);
		break;
	case ASRC_RELEASE_PAIR:
		ret = fsl_easrc_ioctl_release_context(m2m, user);
		break;
	case ASRC_CONVERT:
		ret = fsl_easrc_ioctl_convert(m2m, user);
		break;
	case ASRC_START_CONV:
		ret = fsl_easrc_ioctl_start_conv(m2m, user);
		break;
	case ASRC_STOP_CONV:
		ret = fsl_easrc_ioctl_stop_conv(m2m, user);
		break;
	case ASRC_STATUS:
		ret = fsl_easrc_ioctl_status(m2m, user);
		break;
	case ASRC_FLUSH:
		ret = fsl_easrc_ioctl_flush(m2m, user);
		break;
	default:
		dev_err(&easrc->pdev->dev, "invalid ioctl command\n");
	}

	return ret;
}

static const struct file_operations easrc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = fsl_easrc_ioctl,
	.open = fsl_easrc_open,
	.release = fsl_easrc_close,
};

static int fsl_easrc_m2m_init(struct fsl_easrc *easrc)
{
	struct device *dev = &easrc->pdev->dev;
	int ret;

	easrc->easrc_miscdev.fops = &easrc_fops;
	easrc->easrc_miscdev.parent = dev;
	easrc->easrc_miscdev.name = easrc->name;
	easrc->easrc_miscdev.minor = MISC_DYNAMIC_MINOR;
	ret = misc_register(&easrc->easrc_miscdev);
	if (ret)
		dev_err(dev, "failed to register char device %d\n", ret);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static void fsl_easrc_m2m_suspend(struct fsl_easrc *easrc)
{
	struct fsl_easrc_context *ctx;
	struct fsl_easrc_m2m *m2m;
	unsigned long lock_flags;
	int i;

	for (i = 0; i < EASRC_CTX_MAX_NUM; i++) {
		spin_lock_irqsave(&easrc->lock, lock_flags);
		ctx = easrc->ctx[i];
		if (!ctx || !ctx->private_data) {
			spin_unlock_irqrestore(&easrc->lock, lock_flags);
			continue;
		}
		m2m = ctx->private_data;

		if (!completion_done(&m2m->complete[IN])) {
			if (ctx->dma_chan[IN])
				dmaengine_terminate_all(ctx->dma_chan[IN]);
			fsl_easrc_input_dma_callback((void *)m2m);
		}
		if (!completion_done(&m2m->complete[OUT])) {
			if (ctx->dma_chan[OUT])
				dmaengine_terminate_all(ctx->dma_chan[OUT]);
			fsl_easrc_output_dma_callback((void *)m2m);
		}

		m2m->first_convert = 1;
		fsl_easrc_stop_context(ctx);
		spin_unlock_irqrestore(&easrc->lock, lock_flags);
	}
}

static void fsl_easrc_m2m_resume(struct fsl_easrc *easrc)
{
	/* null */
}
#endif
