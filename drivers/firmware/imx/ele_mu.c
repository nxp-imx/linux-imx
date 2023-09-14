// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021-2023 NXP
 * Author: Alice Guo <alice.guo@nxp.com>
 * Author: Pankaj Gupta <pankaj.gupta@nxp.com>
 */

#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/dev_printk.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/firmware/imx/ele_fw_api.h>
#include <linux/firmware/imx/ele_base_msg.h>
#include <linux/firmware/imx/ele_mu_ioctl.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sys_soc.h>
#include <linux/workqueue.h>

#include "ele_mu.h"

#define ELE_PING_INTERVAL	(3600 * HZ)
#define ELE_TRNG_STATE_OK	0x203
#define ELE_GET_INFO_BUFF_SZ	0x100
#define ELE_GET_INFO_READ_SZ	0xA0
#define ELE_IMEM_SIZE		0x10000
#define ELE_IMEM_STATE_OK	0xCA
#define ELE_IMEM_STATE_BAD	0xFE
#define ELE_IMEM_STATE_WORD	0x27
#define ELE_IMEM_STATE_MASK	0x00ff0000

#define RESERVED_DMA_POOL	BIT(1)

struct ele_mu_priv *ele_priv_export;

struct imx_info {
	bool socdev;
	/* platform specific flag to enable/disable the Sentinel True RNG */
	bool enable_ele_trng;
	bool reserved_dma_ranges;
	bool init_fw;
};

static const struct imx_info imx8ulp_info = {
	.socdev = true,
	.enable_ele_trng = false,
	.reserved_dma_ranges = true,
	.init_fw = false,
};

static const struct imx_info imx93_info = {
	.socdev = false,
	.enable_ele_trng = true,
	.reserved_dma_ranges = true,
	.init_fw = true,
};

static const struct of_device_id ele_mu_match[] = {
	{ .compatible = "fsl,imx-ele", .data = (void *)&imx8ulp_info},
	{ .compatible = "fsl,imx93-ele", .data = (void *)&imx93_info},
	{},
};

int get_ele_mu_priv(struct ele_mu_priv **export)
{
	if (!ele_priv_export)
		return -EPROBE_DEFER;

	*export = ele_priv_export;
	return 0;
}
EXPORT_SYMBOL_GPL(get_ele_mu_priv);

/*
 * Callback called by mailbox FW when data are received
 */
static void ele_mu_rx_callback(struct mbox_client *c, void *msg)
{
	struct device *dev = c->dev;
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	struct ele_mu_device_ctx *dev_ctx;
	bool is_response = false;
	int msg_size;
	struct mu_hdr header;

	dev_dbg(dev, "Message received on mailbox\n");

	/* The function can be called with NULL msg */
	if (!msg) {
		dev_err(dev, "Message is invalid\n");
		return;
	}

	if (IS_ERR(msg)) {
		dev_err(dev, "Error during reception of message: %ld\n",
				PTR_ERR(msg));
		return;
	}

	header.tag = ((u8 *)msg)[3];
	header.command = ((u8 *)msg)[2];
	header.size = ((u8 *)msg)[1];
	header.ver = ((u8 *)msg)[0];

	dev_dbg(dev, "Selecting device\n");

	/* Incoming command: wake up the receiver if any. */
	if (header.tag == priv->cmd_tag) {
		dev_dbg(dev, "Selecting cmd receiver\n");
		dev_ctx = priv->cmd_receiver_dev;
	} else if (header.tag == priv->rsp_tag) {
		if (priv->waiting_rsp_dev) {
			dev_dbg(dev, "Selecting rsp waiter\n");
			dev_ctx = priv->waiting_rsp_dev;
			is_response = true;
		} else {
			/* Reading the EdgeLock Enclave response
			 * to the command sent by other
			 * linux kernel services.
			 */
			spin_lock(&priv->lock);
			priv->rx_msg = *(struct ele_api_msg *)msg;
			complete(&priv->done);
			spin_unlock(&priv->lock);
			return;
		}
	} else {
		dev_err(dev, "Failed to select a device for message: %.8x\n",
				*((u32 *) &header));
		return;
	}

	if (!dev_ctx) {
		dev_err(dev, "No device context selected for message: %.8x\n",
				*((u32 *)&header));
		return;
	}
	/* Init reception */
	msg_size = header.size;
	if (msg_size > MAX_RECV_SIZE) {
		devctx_err(dev_ctx, "Message is too big (%d > %d)", msg_size,
				MAX_RECV_SIZE);
		return;
	}

	memcpy(dev_ctx->temp_resp, msg, msg_size * sizeof(u32));
	dev_ctx->temp_resp_size = msg_size;

	/* Allow user to read */
	dev_ctx->pending_hdr = dev_ctx->temp_resp[0];
	wake_up_interruptible(&dev_ctx->wq);

	if (is_response)
		priv->waiting_rsp_dev = NULL;

}

static void ele_ping_handler(struct work_struct *work)
{
	int ret;

	ret = ele_ping();
	if (ret)
		pr_err("ping ele failed, try again!\n");

	/* reschedule the delay work */
	schedule_delayed_work(to_delayed_work(work), ELE_PING_INTERVAL);
}
static DECLARE_DELAYED_WORK(ele_ping_work, ele_ping_handler);

static int imx_soc_device_register(struct platform_device *pdev)
{
	struct soc_device_attribute *attr;
	struct soc_device *dev;
	struct gen_pool *sram_pool;
	u32 *get_info_data;
	phys_addr_t get_info_addr;
	u8 major_ver, minor_ver;
	u32 v[4];
	int err;

	err = read_common_fuse(OTP_UNIQ_ID, v, true);
	if (err)
		return err;

	sram_pool = of_gen_pool_get(pdev->dev.of_node, "sram-pool", 0);
	if (!sram_pool) {
		pr_err("Unable to get sram pool\n");
		return -EINVAL;
	}

	get_info_data = (u32 *)gen_pool_alloc(sram_pool, 0x100);
	if (!get_info_data) {
		pr_err("Unable to alloc sram from sram pool\n");
		return -ENOMEM;
	}

	get_info_addr = gen_pool_virt_to_phys(sram_pool, (ulong)get_info_data);

	attr = kzalloc(sizeof(*attr), GFP_KERNEL);
	if (!attr)
		return -ENOMEM;

	err = ele_get_info(get_info_addr, ELE_GET_INFO_READ_SZ);
	if (err) {
		attr->revision = kasprintf(GFP_KERNEL, "A0");
	} else {
		major_ver = (get_info_data[1] & 0xffff0000) >> 24;
		minor_ver = (get_info_data[1] & 0xffff0000) & 0xFF;
		if (minor_ver)
			attr->revision = kasprintf(GFP_KERNEL, "%x.%x", major_ver, minor_ver);
		else
			attr->revision = kasprintf(GFP_KERNEL, "%x", major_ver);
	}

	gen_pool_free(sram_pool, (unsigned long)get_info_data, 0x100);

	err = of_property_read_string(of_root, "model", &attr->machine);
	if (err) {
		kfree(attr);
		return -EINVAL;
	}
	attr->family = kasprintf(GFP_KERNEL, "Freescale i.MX");

	attr->serial_number = kasprintf(GFP_KERNEL, "%016llX", (u64)v[3] << 32 | v[0]);
	attr->soc_id = kasprintf(GFP_KERNEL, "i.MX8ULP");

	dev = soc_device_register(attr);
	if (IS_ERR(dev)) {
		kfree(attr->soc_id);
		kfree(attr->serial_number);
		kfree(attr->revision);
		kfree(attr->family);
		kfree(attr->machine);
		kfree(attr);
		return PTR_ERR(dev);
	}

	return 0;
}

static int ele_do_start_rng(void)
{
	int ret;
	int count = 5;

	ret = ele_get_trng_state();
	if (ret < 0) {
		pr_err("Failed to get trng state\n");
		return ret;
	} else if (ret != ELE_TRNG_STATE_OK) {
		/* call start rng */
		ret = ele_start_rng();
		if (ret) {
			pr_err("Failed to start rng\n");
			return ret;
		}

		/* poll get trng state API 5 times or while trng state != 0x203 */
		do {
			msleep(10);
			ret = ele_get_trng_state();
			if (ret < 0) {
				pr_err("Failed to get trng state\n");
				return ret;
			}
			count--;
		} while ((ret != ELE_TRNG_STATE_OK) && count);
		if (ret != ELE_TRNG_STATE_OK)
			return -EIO;
	}

	return 0;
}

/*
 * File operations for user-space
 */

/* Write a message to the MU. */
static ssize_t ele_mu_fops_write(struct file *fp, const char __user *buf,
				  size_t size, loff_t *ppos)
{
	struct ele_mu_device_ctx *dev_ctx = container_of(fp->private_data,
							 struct ele_mu_device_ctx,
							 miscdev);
	struct ele_mu_priv *ele_mu_priv = dev_ctx->priv;
	u32 nb_words = 0;
	struct mu_hdr header;
	int err;

	devctx_dbg(dev_ctx, "write from buf (%p)%ld, ppos=%lld\n", buf, size,
		   ((ppos) ? *ppos : 0));

	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	if (dev_ctx->status != MU_OPENED) {
		err = -EINVAL;
		goto exit;
	}

	if (size < 4) {//sizeof(struct she_mu_hdr)) {
		devctx_err(dev_ctx, "User buffer too small(%ld < %x)\n", size, 0x4);
		//devctx_err(dev_ctx, "User buffer too small(%ld < %lu)\n", size, ()0x4);
			  // sizeof(struct she_mu_hdr));
		err = -ENOSPC;
		goto exit;
	}

	if (size > MAX_MESSAGE_SIZE_BYTES) {
		devctx_err(dev_ctx, "User buffer too big(%ld > %lu)\n", size,
			   MAX_MESSAGE_SIZE_BYTES);
		err = -ENOSPC;
		goto exit;
	}

	/* Copy data to buffer */
	err = (int)copy_from_user(dev_ctx->temp_cmd, buf, size);
	if (err) {
		err = -EFAULT;
		devctx_err(dev_ctx, "Fail copy message from user\n");
		goto exit;
	}

	print_hex_dump_debug("from user ", DUMP_PREFIX_OFFSET, 4, 4,
			     dev_ctx->temp_cmd, size, false);

	header = *((struct mu_hdr *) (&dev_ctx->temp_cmd[0]));

	/* Check the message is valid according to tags */
	if (header.tag == ele_mu_priv->cmd_tag) {
		/*
		 * unlocked in ele_mu_fops_read when the
		 * response to this command is read.
		 */
		mutex_lock(&ele_mu_priv->mu_cmd_lock);
		ele_mu_priv->waiting_rsp_dev = dev_ctx;
	} else if (header.tag == ele_mu_priv->rsp_tag) {
		/* Check the device context can send the command */
		if (dev_ctx != ele_mu_priv->cmd_receiver_dev) {
			devctx_err(dev_ctx,
				   "This channel is not configured to send response to SECO\n");
			err = -EPERM;
			goto exit;
		}
	} else {
		devctx_err(dev_ctx, "The message does not have a valid TAG\n");
		err = -EINVAL;
		goto exit;
	}

	/*
	 * Check that the size passed as argument matches the size
	 * carried in the message.
	 */
	nb_words = header.size;
	if (nb_words * sizeof(u32) != size) {
		err = -EINVAL;
		devctx_err(dev_ctx, "User buffer too small\n");
		goto exit;
	}

	mutex_lock(&ele_mu_priv->mu_lock);

	/* Send message */
	devctx_dbg(dev_ctx, "sending message\n");
	err = mbox_send_message(ele_mu_priv->tx_chan, dev_ctx->temp_cmd);
	if (err < 0) {
		devctx_err(dev_ctx, "Failed to send message\n");
		goto unlock;
	}

	err = nb_words * (u32)sizeof(u32);

unlock:
	mutex_unlock(&ele_mu_priv->mu_lock);

exit:
	if (err < 0)
		mutex_unlock(&ele_mu_priv->mu_cmd_lock);
	up(&dev_ctx->fops_lock);
	return err;
}

/*
 * Read a message from the MU.
 * Blocking until a message is available.
 */
static ssize_t ele_mu_fops_read(struct file *fp, char __user *buf,
				 size_t size, loff_t *ppos)
{
	struct ele_mu_device_ctx *dev_ctx = container_of(fp->private_data,
							 struct ele_mu_device_ctx,
							 miscdev);
	struct ele_mu_priv *ele_mu_priv = dev_ctx->priv;
	u32 data_size = 0, size_to_copy = 0;
	struct ele_buf_desc *b_desc;
	int err;
	struct mu_hdr header = {0};

	devctx_dbg(dev_ctx, "read to buf %p(%ld), ppos=%lld\n", buf, size,
		   ((ppos) ? *ppos : 0));

	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	if (dev_ctx->status != MU_OPENED) {
		err = -EINVAL;
		goto exit;
	}

	/* Wait until the complete message is received on the MU. */
	err = wait_event_interruptible(dev_ctx->wq, dev_ctx->pending_hdr != 0);
	if (err) {
		devctx_err(dev_ctx, "Err[0x%x]:Interrupted by signal.\n", err);
		goto exit;
	}

	devctx_dbg(dev_ctx, "%s %s\n", __func__,
		   "message received, start transmit to user");

	/* Check that the size passed as argument is larger than
	 * the one carried in the message.
	 */
	data_size = dev_ctx->temp_resp_size * sizeof(u32);
	size_to_copy = data_size;
	if (size_to_copy > size) {
		devctx_dbg(dev_ctx, "User buffer too small (%ld < %d)\n",
			   size, size_to_copy);
		size_to_copy = size;
	}

	/* We may need to copy the output data to user before
	 * delivering the completion message.
	 */
	while (!list_empty(&dev_ctx->pending_out)) {
		b_desc = list_first_entry_or_null(&dev_ctx->pending_out,
						  struct ele_buf_desc,
						  link);
		if (b_desc->usr_buf_ptr && b_desc->shared_buf_ptr) {
			devctx_dbg(dev_ctx, "Copy output data to user\n");
			err = (int)copy_to_user(b_desc->usr_buf_ptr,
						b_desc->shared_buf_ptr,
						b_desc->size);
			if (err) {
				devctx_err(dev_ctx,
					   "Failed to copy output data to user\n");
				err = -EFAULT;
				goto exit;
			}
		}

		if (b_desc->shared_buf_ptr)
			memset(b_desc->shared_buf_ptr, 0, b_desc->size);

		__list_del_entry(&b_desc->link);
		devm_kfree(dev_ctx->dev, b_desc);
	}

	header = *((struct mu_hdr *) (&dev_ctx->temp_resp[0]));

	/* Copy data from the buffer */
	print_hex_dump_debug("to user ", DUMP_PREFIX_OFFSET, 4, 4,
			     dev_ctx->temp_resp, size_to_copy, false);
	err = (int)copy_to_user(buf, dev_ctx->temp_resp, size_to_copy);
	if (err) {
		devctx_err(dev_ctx, "Failed to copy to user\n");
		err = -EFAULT;
		goto exit;
	}

	err = size_to_copy;

	/* free memory allocated on the shared buffers. */
	dev_ctx->secure_mem.pos = 0;
	dev_ctx->non_secure_mem.pos = 0;

	dev_ctx->pending_hdr = 0;

exit:
	/*
	 * Clean the used Shared Memory space,
	 * whether its Input Data copied from user buffers, or
	 * Data received from FW.
	 */
	while (!list_empty(&dev_ctx->pending_in) ||
	       !list_empty(&dev_ctx->pending_out)) {
		if (!list_empty(&dev_ctx->pending_in))
			b_desc = list_first_entry_or_null(&dev_ctx->pending_in,
							  struct ele_buf_desc,
							  link);
		else
			b_desc = list_first_entry_or_null(&dev_ctx->pending_out,
							  struct ele_buf_desc,
							  link);

		if (b_desc->shared_buf_ptr)
			memset(b_desc->shared_buf_ptr, 0, b_desc->size);

		__list_del_entry(&b_desc->link);
		devm_kfree(dev_ctx->dev, b_desc);
	}
	if (header.tag == ele_mu_priv->rsp_tag)
		mutex_unlock(&ele_mu_priv->mu_cmd_lock);

	up(&dev_ctx->fops_lock);
	return err;
}

/* Give access to EdgeLock Enclave, to the memory we want to share */
static int ele_mu_setup_ele_mem_access(struct ele_mu_device_ctx *dev_ctx,
					     u64 addr, u32 len)
{
	/* Assuming EdgeLock Enclave has access to all the memory regions */
	int ret = 0;

	if (ret) {
		devctx_err(dev_ctx, "Fail find memreg\n");
		goto exit;
	}

	if (ret) {
		devctx_err(dev_ctx, "Fail set permission for resource\n");
		goto exit;
	}

exit:
	return ret;
}

static int ele_mu_ioctl_get_mu_info(struct ele_mu_device_ctx *dev_ctx,
				    unsigned long arg)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev_ctx->dev);
	struct ele_mu_ioctl_get_mu_info info;
	int err = -EINVAL;

	info.ele_mu_id = (u8)priv->ele_mu_id;
	info.interrupt_idx = 0;
	info.tz = 0;
	info.did = (u8)priv->ele_mu_did;

	devctx_dbg(dev_ctx,
		   "info [mu_idx: %d, irq_idx: %d, tz: 0x%x, did: 0x%x]\n",
		   info.ele_mu_id, info.interrupt_idx, info.tz, info.did);

	err = (int)copy_to_user((u8 *)arg, &info,
		sizeof(info));
	if (err) {
		devctx_err(dev_ctx, "Failed to copy mu info to user\n");
		err = -EFAULT;
		goto exit;
	}

exit:
	return err;
}

/*
 * Copy a buffer of daa to/from the user and return the address to use in
 * messages
 */
static int ele_mu_ioctl_setup_iobuf_handler(struct ele_mu_device_ctx *dev_ctx,
					    unsigned long arg)
{
	struct ele_buf_desc *b_desc;
	struct ele_mu_ioctl_setup_iobuf io = {0};
	struct ele_shared_mem *shared_mem;
	int err = -EINVAL;
	u32 pos;

	err = (int)copy_from_user(&io,
		(u8 *)arg,
		sizeof(io));
	if (err) {
		devctx_err(dev_ctx, "Failed copy iobuf config from user\n");
		err = -EFAULT;
		goto exit;
	}

	devctx_dbg(dev_ctx, "io [buf: %p(%d) flag: %x]\n",
		   io.user_buf, io.length, io.flags);

	if (io.length == 0 || !io.user_buf) {
		/*
		 * Accept NULL pointers since some buffers are optional
		 * in SECO commands. In this case we should return 0 as
		 * pointer to be embedded into the message.
		 * Skip all data copy part of code below.
		 */
		io.ele_addr = 0;
		goto copy;
	}

	/* Select the shared memory to be used for this buffer. */
	if (io.flags & SECO_MU_IO_FLAGS_USE_SEC_MEM) {
		/* App requires to use secure memory for this buffer.*/
		devctx_err(dev_ctx, "Failed allocate SEC MEM memory\n");
		err = -EFAULT;
		goto exit;
	} else {
		/* No specific requirement for this buffer. */
		shared_mem = &dev_ctx->non_secure_mem;
	}

	/* Check there is enough space in the shared memory. */
	if (io.length >= shared_mem->size - shared_mem->pos) {
		devctx_err(dev_ctx, "Not enough space in shared memory\n");
		err = -ENOMEM;
		goto exit;
	}

	/* Allocate space in shared memory. 8 bytes aligned. */
	pos = shared_mem->pos;
	shared_mem->pos += round_up(io.length, 8u);
	io.ele_addr = (u64)shared_mem->dma_addr + pos;

	if ((io.flags & SECO_MU_IO_FLAGS_USE_SEC_MEM) &&
	    !(io.flags & SECO_MU_IO_FLAGS_USE_SHORT_ADDR)) {
		/*Add base address to get full address.*/
		devctx_err(dev_ctx, "Failed allocate SEC MEM memory\n");
		err = -EFAULT;
		goto exit;
	}

	memset(shared_mem->ptr + pos, 0, io.length);
	if ((io.flags & SECO_MU_IO_FLAGS_IS_INPUT) ||
	    (io.flags & SECO_MU_IO_FLAGS_IS_IN_OUT)) {
		/*
		 * buffer is input:
		 * copy data from user space to this allocated buffer.
		 */
		err = (int)copy_from_user(shared_mem->ptr + pos, io.user_buf,
					  io.length);
		if (err) {
			devctx_err(dev_ctx,
				   "Failed copy data to shared memory\n");
			err = -EFAULT;
			goto exit;
		}
	}

	b_desc = devm_kmalloc(dev_ctx->dev, sizeof(*b_desc), GFP_KERNEL);
	if (!b_desc) {
		err = -ENOMEM;
		devctx_err(dev_ctx,
			   "Failed allocating mem for pending buffer\n"
			   );
		goto exit;
	}

	b_desc->shared_buf_ptr = shared_mem->ptr + pos;
	b_desc->usr_buf_ptr = io.user_buf;
	b_desc->size = io.length;

	if (io.flags & SECO_MU_IO_FLAGS_IS_INPUT) {
		/*
		 * buffer is input:
		 * add an entry in the "pending input buffers" list so
		 * that copied data can be cleaned from shared memory
		 * later.
		 */
		list_add_tail(&b_desc->link, &dev_ctx->pending_in);
	} else {
		/*
		 * buffer is output:
		 * add an entry in the "pending out buffers" list so data
		 * can be copied to user space when receiving ELE
		 * response.
		 */
		list_add_tail(&b_desc->link, &dev_ctx->pending_out);
	}
copy:
	/* Provide the EdgeLock Enclave address to user space only if success. */
	err = (int)copy_to_user((u8 *)arg, &io,
		sizeof(io));
	if (err) {
		devctx_err(dev_ctx, "Failed to copy iobuff setup to user\n");
		err = -EFAULT;
		goto exit;
	}
exit:
	return err;
}



/* Open a char device. */
static int ele_mu_fops_open(struct inode *nd, struct file *fp)
{
	struct ele_mu_device_ctx *dev_ctx = container_of(fp->private_data,
							    struct ele_mu_device_ctx,
							    miscdev);
	int err;

	/* Avoid race if opened at the same time */
	if (down_trylock(&dev_ctx->fops_lock))
		return -EBUSY;

	/* Authorize only 1 instance. */
	if (dev_ctx->status != MU_FREE) {
		err = -EBUSY;
		goto exit;
	}

	/*
	 * Allocate some memory for data exchanges with S40x.
	 * This will be used for data not requiring secure memory.
	 */
	dev_ctx->non_secure_mem.ptr = dmam_alloc_coherent(dev_ctx->dev,
					MAX_DATA_SIZE_PER_USER,
					&dev_ctx->non_secure_mem.dma_addr,
					GFP_KERNEL);
	if (!dev_ctx->non_secure_mem.ptr) {
		err = -ENOMEM;
		devctx_err(dev_ctx, "Failed to map shared memory with S40x\n");
		goto exit;
	}

	err = ele_mu_setup_ele_mem_access(dev_ctx,
					  dev_ctx->non_secure_mem.dma_addr,
					  MAX_DATA_SIZE_PER_USER);
	if (err) {
		err = -EPERM;
		devctx_err(dev_ctx,
			   "Failed to share access to shared memory\n");
		goto free_coherent;
	}

	dev_ctx->non_secure_mem.size = MAX_DATA_SIZE_PER_USER;
	dev_ctx->non_secure_mem.pos = 0;
	dev_ctx->status = MU_OPENED;

	dev_ctx->pending_hdr = 0;

	goto exit;

free_coherent:
	dmam_free_coherent(dev_ctx->priv->dev, MAX_DATA_SIZE_PER_USER,
			   dev_ctx->non_secure_mem.ptr,
			   dev_ctx->non_secure_mem.dma_addr);

exit:
	up(&dev_ctx->fops_lock);
	return err;
}

/* Close a char device. */
static int ele_mu_fops_close(struct inode *nd, struct file *fp)
{
	struct ele_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct ele_mu_device_ctx, miscdev);
	struct ele_mu_priv *priv = dev_ctx->priv;
	struct ele_buf_desc *b_desc;

	/* Avoid race if closed at the same time */
	if (down_trylock(&dev_ctx->fops_lock))
		return -EBUSY;

	/* The device context has not been opened */
	if (dev_ctx->status != MU_OPENED)
		goto exit;

	/* check if this device was registered as command receiver. */
	if (priv->cmd_receiver_dev == dev_ctx)
		priv->cmd_receiver_dev = NULL;

	/* check if this device was registered as waiting response. */
	if (priv->waiting_rsp_dev == dev_ctx) {
		priv->waiting_rsp_dev = NULL;
		mutex_unlock(&priv->mu_cmd_lock);
	}

	/* Unmap secure memory shared buffer. */
	if (dev_ctx->secure_mem.ptr)
		devm_iounmap(dev_ctx->dev, dev_ctx->secure_mem.ptr);

	dev_ctx->secure_mem.ptr = NULL;
	dev_ctx->secure_mem.dma_addr = 0;
	dev_ctx->secure_mem.size = 0;
	dev_ctx->secure_mem.pos = 0;

	/* Free non-secure shared buffer. */
	dmam_free_coherent(dev_ctx->priv->dev, MAX_DATA_SIZE_PER_USER,
			   dev_ctx->non_secure_mem.ptr,
			   dev_ctx->non_secure_mem.dma_addr);

	dev_ctx->non_secure_mem.ptr = NULL;
	dev_ctx->non_secure_mem.dma_addr = 0;
	dev_ctx->non_secure_mem.size = 0;
	dev_ctx->non_secure_mem.pos = 0;

	while (!list_empty(&dev_ctx->pending_in) ||
	       !list_empty(&dev_ctx->pending_out)) {
		if (!list_empty(&dev_ctx->pending_in))
			b_desc = list_first_entry_or_null(&dev_ctx->pending_in,
							  struct ele_buf_desc,
							  link);
		else
			b_desc = list_first_entry_or_null(&dev_ctx->pending_out,
							  struct ele_buf_desc,
							  link);

		if (b_desc->shared_buf_ptr)
			memset(b_desc->shared_buf_ptr, 0, b_desc->size);

		__list_del_entry(&b_desc->link);
		devm_kfree(dev_ctx->dev, b_desc);
	}

	dev_ctx->status = MU_FREE;

exit:
	up(&dev_ctx->fops_lock);
	return 0;
}

/* IOCTL entry point of a char device */
static long ele_mu_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct ele_mu_device_ctx *dev_ctx = container_of(fp->private_data,
							 struct ele_mu_device_ctx,
							 miscdev);
	struct ele_mu_priv *ele_mu_priv = dev_ctx->priv;
	int err = -EINVAL;

	/* Prevent race during change of device context */
	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	switch (cmd) {
	case ELE_MU_IOCTL_ENABLE_CMD_RCV:
		if (!ele_mu_priv->cmd_receiver_dev) {
			ele_mu_priv->cmd_receiver_dev = dev_ctx;
			err = 0;
		};
		break;
	case ELE_MU_IOCTL_GET_MU_INFO:
		err = ele_mu_ioctl_get_mu_info(dev_ctx, arg);
		break;
	case ELE_MU_IOCTL_SHARED_BUF_CFG:
		devctx_err(dev_ctx, "ELE_MU_IOCTL_SHARED_BUF_CFG not supported [0x%x].\n", err);
		break;
	case ELE_MU_IOCTL_SETUP_IOBUF:
		err = ele_mu_ioctl_setup_iobuf_handler(dev_ctx, arg);
		break;
	case ELE_MU_IOCTL_SIGNED_MESSAGE:
		devctx_err(dev_ctx, "ELE_MU_IOCTL_SIGNED_MESSAGE not supported [0x%x].\n", err);
		break;
	default:
		err = -EINVAL;
		devctx_dbg(dev_ctx, "IOCTL %.8x not supported\n", cmd);
	}

	up(&dev_ctx->fops_lock);
	return (long)err;
}

/* Char driver setup */
static const struct file_operations ele_mu_fops = {
	.open		= ele_mu_fops_open,
	.owner		= THIS_MODULE,
	.release	= ele_mu_fops_close,
	.unlocked_ioctl = ele_mu_ioctl,
	.read		= ele_mu_fops_read,
	.write		= ele_mu_fops_write,
};

/* interface for managed res to free a mailbox channel */
static void if_mbox_free_channel(void *mbox_chan)
{
	mbox_free_channel(mbox_chan);
}

/* interface for managed res to unregister a char device */
static void if_misc_deregister(void *miscdevice)
{
	misc_deregister(miscdevice);
}

static int ele_mu_request_channel(struct device *dev,
				 struct mbox_chan **chan,
				 struct mbox_client *cl,
				 const char *name)
{
	struct mbox_chan *t_chan;
	int ret = 0;

	t_chan = mbox_request_channel_byname(cl, name);
	if (IS_ERR(t_chan)) {
		ret = PTR_ERR(t_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev,
				"Failed to request chan %s ret %d\n", name,
				ret);
		goto exit;
	}

	ret = devm_add_action(dev, if_mbox_free_channel, t_chan);
	if (ret) {
		dev_err(dev, "failed to add devm removal of mbox %s\n", name);
		goto exit;
	}

	*chan = t_chan;

exit:
	return ret;
}

static int ele_mu_probe(struct platform_device *pdev)
{
	struct ele_mu_device_ctx *dev_ctx;
	struct device *dev = &pdev->dev;
	struct ele_mu_priv *priv;
	struct device_node *np;
	const struct of_device_id *of_id = of_match_device(ele_mu_match, dev);
	struct imx_info *info = (of_id != NULL) ? (struct imx_info *)of_id->data
						: NULL;
	int max_nb_users = 0;
	char *devname;
	int ret;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		dev_err(dev, "Fail allocate mem for private data\n");
		goto exit;
	}
	priv->dev = dev;
	dev_set_drvdata(dev, priv);

	/*
	 * Get the address of MU to be used for communication with the SCU
	 */
	np = pdev->dev.of_node;
	if (!np) {
		dev_err(dev, "Cannot find MU User entry in device tree\n");
		ret = -EOPNOTSUPP;
		goto exit;
	}

	/* Initialize the mutex. */
	mutex_init(&priv->mu_cmd_lock);
	mutex_init(&priv->mu_lock);

	/* TBD */
	priv->cmd_receiver_dev = NULL;
	priv->waiting_rsp_dev = NULL;

	ret = of_property_read_u32(np, "fsl,ele_mu_did", &priv->ele_mu_did);
	if (ret) {
		ret = -EINVAL;
		dev_err(dev, "%s: Not able to read ele_mu_did", __func__);
		goto exit;
	}

	ret = of_property_read_u32(np, "fsl,ele_mu_id", &priv->ele_mu_id);
	if (ret) {
		ret = -EINVAL;
		dev_err(dev, "%s: Not able to read ele_mu_id", __func__);
		goto exit;
	}

	ret = of_property_read_u32(np, "fsl,ele_mu_max_users", &max_nb_users);
	if (ret) {
		dev_warn(dev, "%s: Not able to read mu_max_user", __func__);
		max_nb_users = S4_MUAP_DEFAULT_MAX_USERS;
	}

	ret = of_property_read_u8(np, "fsl,cmd_tag", &priv->cmd_tag);
	if (ret) {
		dev_warn(dev, "%s: Not able to read cmd_tag", __func__);
		priv->cmd_tag = DEFAULT_MESSAGING_TAG_COMMAND;
	}

	ret = of_property_read_u8(np, "fsl,rsp_tag", &priv->rsp_tag);
	if (ret) {
		dev_warn(dev, "%s: Not able to read rsp_tag", __func__);
		priv->rsp_tag = DEFAULT_MESSAGING_TAG_RESPONSE;
	}

	/* Mailbox client configuration */
	priv->ele_mb_cl.dev		= dev;
	priv->ele_mb_cl.tx_block	= false;
	priv->ele_mb_cl.knows_txdone	= true;
	priv->ele_mb_cl.rx_callback	= ele_mu_rx_callback;

	ret = ele_mu_request_channel(dev, &priv->tx_chan, &priv->ele_mb_cl, "tx");
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request tx channel\n");

		goto exit;
	}

	ret = ele_mu_request_channel(dev, &priv->rx_chan, &priv->ele_mb_cl, "rx");
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request rx channel\n");

		goto exit;
	}

	priv->max_dev_ctx = max_nb_users;
	priv->ctxs = devm_kzalloc(dev, sizeof(dev_ctx) * max_nb_users, GFP_KERNEL);

	/* Create users */
	for (i = 0; i < max_nb_users; i++) {
		dev_ctx = devm_kzalloc(dev, sizeof(*dev_ctx), GFP_KERNEL);
		if (!dev_ctx) {
			ret = -ENOMEM;
			dev_err(dev,
				"Fail to allocate memory for device context\n");
			goto exit;
		}

		dev_ctx->dev = dev;
		dev_ctx->status = MU_FREE;
		dev_ctx->priv = priv;

		priv->ctxs[i] = dev_ctx;

		/* Default value invalid for an header. */
		init_waitqueue_head(&dev_ctx->wq);

		INIT_LIST_HEAD(&dev_ctx->pending_out);
		INIT_LIST_HEAD(&dev_ctx->pending_in);
		sema_init(&dev_ctx->fops_lock, 1);

		devname = devm_kasprintf(dev, GFP_KERNEL, "ele_mu%d_ch%d",
					 priv->ele_mu_id, i);
		if (!devname) {
			ret = -ENOMEM;
			dev_err(dev,
				"Fail to allocate memory for misc dev name\n");
			goto exit;
		}

		dev_ctx->miscdev.name = devname;
		dev_ctx->miscdev.minor = MISC_DYNAMIC_MINOR;
		dev_ctx->miscdev.fops = &ele_mu_fops;
		dev_ctx->miscdev.parent = dev;
		ret = misc_register(&dev_ctx->miscdev);
		if (ret) {
			dev_err(dev, "failed to register misc device %d\n",
				ret);
			goto exit;
		}

		ret = devm_add_action(dev, if_misc_deregister,
				      &dev_ctx->miscdev);
		if (ret) {
			dev_err(dev,
				"failed[%d] to add action to the misc-dev\n",
				ret);
			goto exit;
		}
	}

	init_completion(&priv->done);
	spin_lock_init(&priv->lock);

	ele_priv_export = priv;

	if (info && info->reserved_dma_ranges) {
		ret = of_reserved_mem_device_init(dev);
		if (ret) {
			dev_err(dev, "failed to init reserved memory region %d\n", ret);
			priv->flags &= (~RESERVED_DMA_POOL);
			goto exit;
		}
		priv->flags |= RESERVED_DMA_POOL;
	}

	if (info && info->init_fw) {
		/* start initializing ele fw */
		ret = ele_init_fw();
		if (ret)
			dev_err(dev, "Failed to initialize ele fw.\n");
	}

	if (info && info->socdev) {
		ret = imx_soc_device_register(pdev);
		if (ret) {
			dev_err(dev,
				"failed[%d] to register SoC device\n", ret);
			goto exit;
		}

		/* allocate buffer where sentinel store encrypted IMEM */
		priv->imem.buf = dmam_alloc_coherent(dev, ELE_IMEM_SIZE,
						     &priv->imem.phyaddr,
						     GFP_KERNEL);
		if (!priv->imem.buf) {
			dev_err(dev, "Failed to alloc buffer to store encrypted IMEM\n");
			ret = -ENOMEM;
			goto exit;
		}
	}

	/* start ele rng */
	ret = ele_do_start_rng();
	if (ret)
		dev_err(dev, "Failed to start ele rng\n");

	if (!ret && info && info->enable_ele_trng) {
		ret = ele_trng_init(dev);
		if (ret)
			dev_err(dev, "Failed to init ele-trng\n");
	}

	/*
	 * A ELE ping request must be send at least once every day(24 hours),
	 * so setup a delay work with 1 hour interval to ping sentinel periodically.
	 */
	schedule_delayed_work(&ele_ping_work, ELE_PING_INTERVAL);

	dev_set_drvdata(dev, priv);
	return devm_of_platform_populate(dev);

exit:
	/* if execution control reaches here, ele-mu probe fail.
	 * hence doing the cleanup
	 */
	if (priv && (priv->flags & RESERVED_DMA_POOL)) {
		of_reserved_mem_device_release(dev);
		priv->flags &= (~RESERVED_DMA_POOL);
	}
	return ret;
}

static int ele_mu_remove(struct platform_device *pdev)
{
	struct ele_mu_priv *priv;

	cancel_delayed_work_sync(&ele_ping_work);
	priv = dev_get_drvdata(&pdev->dev);
	mbox_free_channel(priv->tx_chan);
	mbox_free_channel(priv->rx_chan);

	/* free the buffer in ele-mu remove, previously allocated
	 * in ele-mu probe to store encrypted IMEM
	 */
	if (priv->imem.buf) {
		dmam_free_coherent(&pdev->dev, ELE_IMEM_SIZE, priv->imem.buf, priv->imem.phyaddr);
		priv->imem.buf = NULL;
	}

	if (priv->flags & RESERVED_DMA_POOL) {
		of_reserved_mem_device_release(&pdev->dev);
		priv->flags &= (~RESERVED_DMA_POOL);
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ele_mu_suspend(struct device *dev)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	const struct of_device_id *of_id = of_match_device(ele_mu_match, dev);
	struct imx_info *info = (of_id != NULL) ? (struct imx_info *)of_id->data : NULL;

	if (info && info->socdev) {
		int ret;

		/* EXPORT command will save encrypted IMEM to given address,
		 * so later in resume, IMEM can be restored from the given address.
		 * size must be at least 64 kB.
		 */
		ret = ele_service_swap(priv->imem.phyaddr, ELE_IMEM_SIZE, ELE_IMEM_EXPORT);
		if (ret < 0)
			dev_err(dev, "Failed to export IMEM\n");
		else {
			priv->imem.size = ret;
			dev_info(dev, "Exported %d bytes of encrypted IMEM\n", ret);
		}
	}

	return 0;
}

static int ele_mu_resume(struct device *dev)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	int i;
	const struct of_device_id *of_id = of_match_device(ele_mu_match, dev);
	struct imx_info *info = (of_id != NULL) ? (struct imx_info *)of_id->data : NULL;

	for (i = 0; i < priv->max_dev_ctx; i++)
		wake_up_interruptible(&priv->ctxs[i]->wq);

	if (info && info->socdev) {
		struct gen_pool *sram_pool;
		u32 *get_info_buf;
		phys_addr_t get_info_phyaddr;
		u32 imem_state;
		int ret;

		ret = ele_do_start_rng();
		if (ret)
			return ret;

		/* allocate buffer to get info from sentinel */
		sram_pool = of_gen_pool_get(dev->of_node, "sram-pool", 0);
		if (!sram_pool) {
			dev_err(dev, "Unable to get sram pool\n");
			return -EINVAL;
		}

		get_info_buf = (u32 *)gen_pool_alloc(sram_pool, ELE_GET_INFO_BUFF_SZ);
		if (!get_info_buf) {
			dev_err(dev, "Unable to alloc sram from sram pool\n");
			return -ENOMEM;
		}
		get_info_phyaddr = gen_pool_virt_to_phys(sram_pool, (ulong)get_info_buf);

		/* get info from sentinel */
		ret = ele_get_info(get_info_phyaddr, ELE_GET_INFO_READ_SZ);
		if (ret) {
			dev_err(dev, "Failed to get info from sentinel\n");
			goto exit;
		}

		/* Get IMEM state, if 0xFE then import IMEM */
		imem_state = (get_info_buf[ELE_IMEM_STATE_WORD] & ELE_IMEM_STATE_MASK) >> 16;
		if (imem_state == ELE_IMEM_STATE_BAD) {
			/* IMPORT command will restore IMEM from the given address,
			 * here size is the actual size returned by sentinel
			 * during the export operation
			 */
			ret = ele_service_swap(priv->imem.phyaddr,
					       priv->imem.size,
					       ELE_IMEM_IMPORT);
			if (ret) {
				dev_err(dev, "Failed to import IMEM\n");
				goto exit;
			}
		} else
			goto exit;

		/* After importing IMEM, check if IMEM state is 0xCA to make sure
		 * IMEM is fully loaded and ELE functionality can be used
		 */
		ret = ele_get_info(get_info_phyaddr, ELE_GET_INFO_READ_SZ);
		if (ret) {
			dev_err(dev, "Failed to get info from sentinel\n");
			goto exit;
		}

		imem_state = (get_info_buf[ELE_IMEM_STATE_WORD] & ELE_IMEM_STATE_MASK) >> 16;
		if (imem_state == ELE_IMEM_STATE_OK)
			dev_info(dev, "Successfully restored IMEM\n");
		else
			dev_err(dev, "Failed to restore IMEM\n");

exit:
	gen_pool_free(sram_pool, (ulong)get_info_buf, ELE_GET_INFO_BUFF_SZ);
	}

	return 0;
}
#endif

static const struct dev_pm_ops ele_mu_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(ele_mu_suspend, ele_mu_resume)
};

static struct platform_driver ele_mu_driver = {
	.driver = {
		.name = "fsl-ele-mu",
		.of_match_table = ele_mu_match,
		.pm = &ele_mu_pm,
	},
	.probe = ele_mu_probe,
	.remove = ele_mu_remove,
};
MODULE_DEVICE_TABLE(of, ele_mu_match);

module_platform_driver(ele_mu_driver);

MODULE_AUTHOR("Pankaj Gupta <pankaj.gupta@nxp.com>");
MODULE_DESCRIPTION("iMX Secure Enclave MU Driver.");
MODULE_LICENSE("GPL v2");
