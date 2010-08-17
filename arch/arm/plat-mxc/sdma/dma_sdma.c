/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file plat-mxc/sdma/dma_sdma.c
 * @brief Front-end to the DMA handling.  This handles the allocation/freeing
 * of DMA channels, and provides a unified interface to the machines
 * DMA facilities. This file contains functions for Smart DMA.
 *
 * @ingroup SDMA
 */

#include <linux/init.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <mach/dma.h>
#include <mach/hardware.h>

#ifdef CONFIG_MXC_SDMA_API

static mxc_dma_channel_t mxc_sdma_channels[MAX_DMA_CHANNELS];
static mxc_dma_channel_private_t mxc_sdma_private[MAX_DMA_CHANNELS];

extern struct clk *mxc_sdma_ahb_clk, *mxc_sdma_ipg_clk;

/*!
 * Tasket to handle processing the channel buffers
 *
 * @param arg channel id
 */
static void mxc_sdma_channeltasklet(unsigned long arg)
{
	dma_request_t request_t;
	dma_channel_params chnl_param;
	mxc_dma_channel_t *chnl_info;
	mxc_dma_channel_private_t *data_priv;
	int bd_intr = 0, error = MXC_DMA_DONE;

	chnl_info = &mxc_sdma_channels[arg];
	data_priv = chnl_info->private;
	chnl_param =
	    mxc_sdma_get_channel_params(chnl_info->channel)->chnl_params;

	mxc_dma_get_config(arg, &request_t, data_priv->buf_tail);

	while (request_t.bd_done == 0) {
		bd_intr = mxc_dma_get_bd_intr(arg, data_priv->buf_tail);
		data_priv->buf_tail += 1;
		if (data_priv->buf_tail >= chnl_param.bd_number) {
			data_priv->buf_tail = 0;
		}
		chnl_info->active = 0;
		if (request_t.bd_error) {
			error = MXC_DMA_TRANSFER_ERROR;
		}

		if (bd_intr != 0) {
			chnl_info->cb_fn(chnl_info->cb_args, error,
					 request_t.count);
			error = MXC_DMA_DONE;
		}

		if (data_priv->buf_tail == chnl_info->curr_buf) {
			break;
		}
		memset(&request_t, 0, sizeof(dma_request_t));
		mxc_dma_get_config(arg, &request_t, data_priv->buf_tail);
	}
}

/*!
 * This function is generally called by the driver at open time.
 * The DMA driver would do any initialization steps that is required
 * to get the channel ready for data transfer.
 *
 * @param channel_id   a pre-defined id. The peripheral driver would specify
 *                     the id associated with its peripheral. This would be
 *                     used by the DMA driver to identify the peripheral
 *                     requesting DMA and do the necessary setup on the
 *                     channel associated with the particular peripheral.
 *                     The DMA driver could use static or dynamic DMA channel
 *                     allocation.
 * @param dev_name     module name or device name
 * @return returns a negative number on error if request for a DMA channel did not
 *         succeed, returns the channel number to be used on success.
 */
int mxc_dma_request_ext(mxc_dma_device_t channel_id, char *dev_name,
			struct dma_channel_info *info)
{
	mxc_sdma_channel_params_t *chnl;
	mxc_dma_channel_private_t *data_priv;
	int ret = 0, i = 0, channel_num = 0;
	mxc_sdma_channel_ext_params_t *p;

	chnl = mxc_sdma_get_channel_params(channel_id);
	if (chnl == NULL) {
		return -EINVAL;
	}

	if (info) {
		if (!chnl->chnl_params.ext)
			return -EINVAL;
		p = (mxc_sdma_channel_ext_params_t *)chnl;
		memcpy(&p->chnl_ext_params.info, info, sizeof(info));
	}


	/* Enable the SDMA clock */
	clk_enable(mxc_sdma_ahb_clk);
	clk_enable(mxc_sdma_ipg_clk);

	channel_num = chnl->channel_num;
	if (chnl->channel_num == MXC_DMA_DYNAMIC_CHANNEL) {
		/* Get the first free channel */
		for (i = (MAX_DMA_CHANNELS - 1); i > 0; i--) {
			/* See if channel is available */
			if ((mxc_sdma_channels[i].dynamic != 1)
			    || (mxc_sdma_channels[i].lock != 0)) {
				continue;
			}
			channel_num = i;
			/* Check to see if we can get this channel */
			ret = mxc_request_dma(&channel_num, dev_name);
			if (ret == 0) {
				break;
			} else {
				continue;
			}
		}
		if (ret != 0) {
			/* No free channel */
			goto err_ret;
		}
	} else {
		if (mxc_sdma_channels[chnl->channel_num].lock == 1) {
			ret = -ENODEV;
			goto err_ret;
		}
		ret = mxc_request_dma(&channel_num, dev_name);
		if (ret != 0) {
			goto err_ret;
		}
	}

	ret = mxc_dma_setup_channel(channel_num, &chnl->chnl_params);

	if (ret == 0) {
		if (chnl->chnl_priority != MXC_SDMA_DEFAULT_PRIORITY) {
			ret =
			    mxc_dma_set_channel_priority(channel_num,
							 chnl->chnl_priority);
			if (ret != 0) {
				pr_info("Failed to set channel prority,\
					  continue with the existing \
					  priority\n");
				goto err_ret;
			}
		}
		mxc_sdma_channels[channel_num].lock = 1;
		if ((chnl->chnl_params.transfer_type == per_2_emi)
		    || (chnl->chnl_params.transfer_type == dsp_2_emi)) {
			mxc_sdma_channels[channel_num].mode = MXC_DMA_MODE_READ;
		} else {
			mxc_sdma_channels[channel_num].mode =
			    MXC_DMA_MODE_WRITE;
		}
		mxc_sdma_channels[channel_num].channel = channel_id;
		data_priv = mxc_sdma_channels[channel_num].private;
		tasklet_init(&data_priv->chnl_tasklet,
			     mxc_sdma_channeltasklet, channel_num);
		if ((channel_id == MXC_DMA_ATA_RX)
		    || (channel_id == MXC_DMA_ATA_TX)) {
			data_priv->intr_after_every_bd = 0;
		} else {
			data_priv->intr_after_every_bd = 1;
		}
	}
      err_ret:
	if (ret != 0) {
		clk_disable(mxc_sdma_ahb_clk);
		clk_disable(mxc_sdma_ipg_clk);
		channel_num = -ENODEV;
	}

	return channel_num;
}

/*!
 * This function is generally called by the driver at close time. The DMA
 * driver would do any cleanup associated with this channel.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @return returns a negative number on error or 0 on success
 */
int mxc_dma_free(int channel_num)
{
	mxc_dma_channel_private_t *data_priv;

	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}

	if (mxc_sdma_channels[channel_num].lock != 1) {
		return -ENODEV;
	}

	mxc_free_dma(channel_num);

	/* Disable the SDMA clock */
	clk_disable(mxc_sdma_ahb_clk);
	clk_disable(mxc_sdma_ipg_clk);

	mxc_sdma_channels[channel_num].lock = 0;
	mxc_sdma_channels[channel_num].active = 0;
	mxc_sdma_channels[channel_num].curr_buf = 0;
	data_priv = mxc_sdma_channels[channel_num].private;
	data_priv->buf_tail = 0;
	tasklet_kill(&data_priv->chnl_tasklet);

	return 0;
}

/*!
 * Callback function called from the SDMA Interrupt routine
 *
 * @param arg driver specific argument that was registered
 */
static void mxc_dma_chnl_callback(void *arg)
{
	int priv;
	mxc_dma_channel_private_t *data_priv;

	priv = (int)arg;
	data_priv = mxc_sdma_channels[priv].private;
	/* Process the buffers in a tasklet */
	tasklet_schedule(&data_priv->chnl_tasklet);
}

/*!
 * This function would just configure the buffers specified by the user into
 * dma channel. The caller must call mxc_dma_enable to start this transfer.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @param dma_buf      an array of physical addresses to the user defined
 *                     buffers. The caller must guarantee the dma_buf is
 *                     available until the transfer is completed.
 * @param num_buf      number of buffers in the array
 * @param mode         specifies whether this is READ or WRITE operation
 * @return This function returns a negative number on error if buffer could not be
 *         added with DMA for transfer. On Success, it returns 0
 */
int mxc_dma_config(int channel_num, mxc_dma_requestbuf_t *dma_buf,
		   int num_buf, mxc_dma_mode_t mode)
{
	int ret = 0, i = 0, prev_buf;
	mxc_dma_channel_t *chnl_info;
	mxc_dma_channel_private_t *data_priv;
	mxc_sdma_channel_params_t *chnl;
	dma_channel_params chnl_param;
	dma_request_t request_t;

	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}

	if (num_buf <= 0) {
		return -EINVAL;
	}

	chnl_info = &mxc_sdma_channels[channel_num];
	data_priv = chnl_info->private;
	if (chnl_info->lock != 1) {
		return -ENODEV;
	}

	/* Check to see if all buffers are taken */
	if (chnl_info->active == 1) {
		return -EBUSY;
	}

	chnl = mxc_sdma_get_channel_params(chnl_info->channel);
	chnl_param = chnl->chnl_params;

	/* Re-setup the SDMA channel if the transfer direction is changed */
	if ((chnl_param.peripheral_type != MEMORY) && (mode != chnl_info->mode)) {
		if (chnl_param.peripheral_type == DSP) {
			if (mode == MXC_DMA_MODE_READ) {
				chnl_param.transfer_type = dsp_2_emi;
			} else {
				chnl_param.transfer_type = emi_2_dsp;
			}
		} else if (chnl_param.peripheral_type == FIFO_MEMORY) {
			if (mode == MXC_DMA_MODE_READ)
				chnl_param.per_address = MXC_FIFO_MEM_SRC_FIXED;
			else
				chnl_param.per_address =
				    MXC_FIFO_MEM_DEST_FIXED;
		} else {
			if (mode == MXC_DMA_MODE_READ) {
				chnl_param.transfer_type = per_2_emi;
			} else {
				chnl_param.transfer_type = emi_2_per;
			}
		}
		chnl_param.callback = mxc_dma_chnl_callback;
		chnl_param.arg = (void *)channel_num;
		ret = mxc_dma_setup_channel(channel_num, &chnl_param);
		if (ret != 0) {
			return ret;
		}
		if (chnl->chnl_priority != MXC_SDMA_DEFAULT_PRIORITY) {
			ret =
			    mxc_dma_set_channel_priority(channel_num,
							 chnl->chnl_priority);
			if (ret != 0) {
				pr_info("Failed to set channel prority,\
					  continue with the existing \
					  priority\n");
			}
		}
		chnl_info->mode = mode;
	}

	for (i = 0; i < num_buf; i++, dma_buf++) {
		/* Check to see if all buffers are taken */
		if (chnl_info->active == 1) {
			break;
		}
		request_t.destAddr = (__u8 *) dma_buf->dst_addr;
		request_t.sourceAddr = (__u8 *) dma_buf->src_addr;
		if (chnl_param.peripheral_type == ASRC)
			request_t.count = dma_buf->num_of_bytes / 4;
		else
			request_t.count = dma_buf->num_of_bytes;
		request_t.bd_cont = 1;
		ret = mxc_dma_set_config(channel_num, &request_t,
					 chnl_info->curr_buf);
		if (ret != 0) {
			break;
		}
		if (data_priv->intr_after_every_bd == 0) {
			if (i == num_buf - 1) {
				mxc_dma_set_bd_intr(channel_num,
						    chnl_info->curr_buf, 1);
			} else {
				mxc_dma_set_bd_intr(channel_num,
						    chnl_info->curr_buf, 0);
			}
		}

		prev_buf = chnl_info->curr_buf;
		chnl_info->curr_buf += 1;
		if (chnl_info->curr_buf >= chnl_param.bd_number) {
			chnl_info->curr_buf = 0;
		}
		if (chnl_info->curr_buf == data_priv->buf_tail) {
			if ((data_priv->intr_after_every_bd == 0)
			    && (i != num_buf - 1)) {
				/*
				 * Set the BD_INTR flag on the last BD that
				 * was queued
				 */
				mxc_dma_set_bd_intr(channel_num, prev_buf, 1);
			}
			chnl_info->active = 1;
		}
	}

	if (i == 0) {
		return -EBUSY;
	}
	return 0;
}

/*!
 * This function would just configure the scatterlist specified by the
 * user into dma channel. This is a slight variation of mxc_dma_config(),
 * it is provided for the convenience of drivers that have a scatterlist
 * passed into them. It is the calling driver's responsibility to have the
 * correct physical address filled in the "dma_address" field of the
 * scatterlist.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @param sg           a scatterlist of buffers. The caller must guarantee
 *                     the dma_buf is available until the transfer is
 *                     completed.
 * @param num_buf      number of buffers in the array
 * @param num_of_bytes total number of bytes to transfer. If set to 0, this
 *                     would imply to use the length field of the scatterlist
 *                     for each DMA transfer. Else it would calculate the size
 *                     for each DMA transfer.
 * @param mode         specifies whether this is READ or WRITE operation
 * @return This function returns a negative number on error if buffer could not
 *         be added with DMA for transfer. On Success, it returns 0
 */
int mxc_dma_sg_config(int channel_num, struct scatterlist *sg,
		      int num_buf, int num_of_bytes, mxc_dma_mode_t mode)
{
	int ret = 0, i = 0;
	mxc_dma_requestbuf_t *dma_buf;

	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}

	if (mxc_sdma_channels[channel_num].lock != 1) {
		return -ENODEV;
	}

	dma_buf =
	    (mxc_dma_requestbuf_t *) kmalloc(num_buf *
					     sizeof(mxc_dma_requestbuf_t),
					     GFP_KERNEL);

	if (dma_buf == NULL) {
		return -EFAULT;
	}

	for (i = 0; i < num_buf; i++) {
		if (mode == MXC_DMA_MODE_READ) {
			(dma_buf + i)->dst_addr = sg->dma_address;
		} else {
			(dma_buf + i)->src_addr = sg->dma_address;
		}

		if ((num_of_bytes > sg->length) || (num_of_bytes == 0)) {
			(dma_buf + i)->num_of_bytes = sg->length;
		} else {
			(dma_buf + i)->num_of_bytes = num_of_bytes;
		}
		sg++;
		num_of_bytes -= (dma_buf + i)->num_of_bytes;
	}

	ret = mxc_dma_config(channel_num, dma_buf, num_buf, mode);
	kfree(dma_buf);
	return ret;
}

/*!
 * This function is provided if the driver would like to set/change its
 * callback function.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @param callback     a callback function to provide notification on transfer
 *                     completion, user could specify NULL if he does not wish
 *                     to be notified
 * @param arg          an argument that gets passed in to the callback
 *                     function, used by the user to do any driver specific
 *                     operations.
 * @return this function returns a negative number on error if the callback
 *         could not be set for the channel or 0 on success
 */
int mxc_dma_callback_set(int channel_num,
			 mxc_dma_callback_t callback, void *arg)
{
	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}

	if (mxc_sdma_channels[channel_num].lock != 1) {
		return -ENODEV;
	}

	mxc_sdma_channels[channel_num].cb_fn = callback;
	mxc_sdma_channels[channel_num].cb_args = arg;

	mxc_dma_set_callback(channel_num, mxc_dma_chnl_callback,
			     (void *)channel_num);

	return 0;
}

/*!
 * This stops the DMA channel and any ongoing transfers. Subsequent use of
 * mxc_dma_enable() will restart the channel and restart the transfer.
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @return returns a negative number on error or 0 on success
 */
int mxc_dma_disable(int channel_num)
{
	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}

	if (mxc_sdma_channels[channel_num].lock != 1) {
		return -ENODEV;
	}

	mxc_dma_stop(channel_num);
	return 0;
}

/*!
 * This starts DMA transfer. Or it restarts DMA on a stopped channel
 * previously stopped with mxc_dma_disable().
 *
 * @param channel_num  the channel number returned at request time. This
 *                     would be used by the DMA driver to identify the calling
 *                     driver and do the necessary cleanup on the channel
 *                     associated with the particular peripheral
 * @return returns a negative number on error or 0 on success
 */
int mxc_dma_enable(int channel_num)
{
	if ((channel_num >= MAX_DMA_CHANNELS) || (channel_num < 0)) {
		return -EINVAL;
	}

	if (mxc_sdma_channels[channel_num].lock != 1) {
		return -ENODEV;
	}

	mxc_dma_start(channel_num);
	return 0;
}

/*!
 * Initializes dma structure with dma_operations
 *
 * @param   dma           dma structure
 * @return  returns 0 on success
 */
static int __init mxc_dma_init(void)
{
	int i;
	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		mxc_sdma_channels[i].active = 0;
		mxc_sdma_channels[i].lock = 0;
		mxc_sdma_channels[i].curr_buf = 0;
		mxc_sdma_channels[i].dynamic = 1;
		mxc_sdma_private[i].buf_tail = 0;
		mxc_sdma_channels[i].private = &mxc_sdma_private[i];
	}
	/*
	 * Make statically allocated channels unavailable for dynamic channel
	 * requests
	 */
	mxc_get_static_channels(mxc_sdma_channels);

	return 0;
}

arch_initcall(mxc_dma_init);

#else
int mxc_request_dma(int *channel, const char *devicename)
{
	return -ENODEV;
}

int mxc_dma_setup_channel(int channel, dma_channel_params *p)
{
	return -ENODEV;
}

int mxc_dma_set_channel_priority(unsigned int channel, unsigned int priority)
{
	return -ENODEV;
}

int mxc_dma_set_config(int channel, dma_request_t *p, int bd_index)
{
	return -ENODEV;
}

int mxc_dma_get_config(int channel, dma_request_t *p, int bd_index)
{
	return -ENODEV;
}

int mxc_dma_start(int channel)
{
	return -ENODEV;
}

int mxc_dma_stop(int channel)
{
	return -ENODEV;
}

void mxc_free_dma(int channel)
{
}

void mxc_dma_set_callback(int channel, dma_callback_t callback, void *arg)
{
}

void *sdma_malloc(size_t size)
{
	return 0;
}

void sdma_free(void *buf)
{
}

void *sdma_phys_to_virt(unsigned long buf)
{
	return 0;
}

unsigned long sdma_virt_to_phys(void *buf)
{
	return 0;
}

int mxc_dma_request(mxc_dma_device_t channel_id, char *dev_name)
{
	return -ENODEV;
}

int mxc_dma_free(int channel_num)
{
	return -ENODEV;
}

int mxc_dma_config(int channel_num, mxc_dma_requestbuf_t *dma_buf,
		   int num_buf, mxc_dma_mode_t mode)
{
	return -ENODEV;
}

int mxc_dma_sg_config(int channel_num, struct scatterlist *sg,
		      int num_buf, int num_of_bytes, mxc_dma_mode_t mode)
{
	return -ENODEV;
}

int mxc_dma_callback_set(int channel_num, mxc_dma_callback_t callback,
			 void *arg)
{
	return -ENODEV;
}

int mxc_dma_disable(int channel_num)
{
	return -ENODEV;
}

int mxc_dma_enable(int channel_num)
{
	return -ENODEV;
}

EXPORT_SYMBOL(mxc_request_dma);
EXPORT_SYMBOL(mxc_dma_setup_channel);
EXPORT_SYMBOL(mxc_dma_set_channel_priority);
EXPORT_SYMBOL(mxc_dma_set_config);
EXPORT_SYMBOL(mxc_dma_get_config);
EXPORT_SYMBOL(mxc_dma_start);
EXPORT_SYMBOL(mxc_dma_stop);
EXPORT_SYMBOL(mxc_free_dma);
EXPORT_SYMBOL(mxc_dma_set_callback);
EXPORT_SYMBOL(sdma_malloc);
EXPORT_SYMBOL(sdma_free);
EXPORT_SYMBOL(sdma_phys_to_virt);
EXPORT_SYMBOL(sdma_virt_to_phys);

#endif

EXPORT_SYMBOL(mxc_dma_request_ext);
EXPORT_SYMBOL(mxc_dma_free);
EXPORT_SYMBOL(mxc_dma_config);
EXPORT_SYMBOL(mxc_dma_sg_config);
EXPORT_SYMBOL(mxc_dma_callback_set);
EXPORT_SYMBOL(mxc_dma_disable);
EXPORT_SYMBOL(mxc_dma_enable);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Linux SDMA API");
MODULE_LICENSE("GPL");
