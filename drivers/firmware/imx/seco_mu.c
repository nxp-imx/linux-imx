// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2019-2020 NXP
 */

/*
 * This driver allows to send messages to the SECO using a shared mailbox. The
 * messages must follow the protocol defined.
 */

/*
 * Architecture of the driver:
 *
 *                                     Non-Secure           +   Secure
 *                                                          |
 *                                                          |
 *                   +---------+      +-------------+       |
 *                   |seco_mu.c+<---->+imx-mailbox.c|       |
 *                   |         |      |  mailbox.c  +<-->+------+    +------+
 *                   +---+-----+      +-------------+    | MU X +<-->+ SECO |
 *                       |                               +------+    +------+
 *                       +----------------+                 |
 *                       |                |                 |
 *                       v                v                 |
 *                   logical           logical              |
 *                   receiver          waiter               |
 *                      +                 +                 |
 *                      |                 |                 |
 *                      |                 |                 |
 *                      |            +----+------+          |
 *                      |            |           |          |
 *                      |            |           |          |
 *               device_ctx     device_ctx     device_ctx   |
 *                                                          |
 *                 User 0        User 1       User Y        |
 *                 +------+      +------+     +------+      |
 *                 |misc.c|      |misc.c|     |misc.c|      |
 * kernel space    +------+      +------+     +------+      |
 *                                                          |
 *  +------------------------------------------------------ |
 *                     |             |           |          |
 * userspace     /dev/seco_muXch0    |           |          |
 *                          /dev/seco_muXch1     |          |
 *                                        /dev/seco_muXchY  |
 *                                                          |
 *
 * When a user sends a command to the seco, it registers its device_ctx as
 * waiter of a response from SECO
 *
 * A user can be registered as receiver of command by the SECO.
 *
 * When a message is received, the driver select the device_ctx receiving the
 * message depending on the tag in the message. It selects the device_ctx
 * accordingly.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>
#include <linux/firmware/imx/sci.h>
#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/firmware/imx/seco_mu_ioctl.h>
#include <linux/mailbox_client.h>

#define MAX_RECV_SIZE 31
#define MAX_RECV_SIZE_BYTES (MAX_RECV_SIZE * sizeof(u32))
#define MAX_MESSAGE_SIZE 31
#define MAX_MESSAGE_SIZE_BYTES (MAX_MESSAGE_SIZE * sizeof(u32))
#define MESSAGE_SIZE(hdr) (((struct she_mu_hdr *)(&(hdr)))->size)
#define MESSAGE_TAG(hdr) (((struct she_mu_hdr *)(&(hdr)))->tag)

#define DEFAULT_MESSAGING_TAG_COMMAND           (0x17u)
#define DEFAULT_MESSAGING_TAG_RESPONSE          (0xe1u)

#define SECURE_RAM_BASE_ADDRESS	(0x31800000ULL)
#define SECURE_RAM_BASE_ADDRESS_SCU	(0x20800000u)
#define SECURE_RAM_SIZE	(0x10000ULL)

#define SECO_MU_DEFAULT_MAX_USERS 4

#define SECO_MU_INTERRUPT_INDEX	(0u)
#define SECO_DEFAULT_MU_INDEX	(1u)
#define SECO_DEFAULT_TZ		(0u)
#define DEFAULT_DID		(0u)

#define MAX_DATA_SIZE_PER_USER  (65 * 1024)

/* Header of the messages exchange with the SECO */
struct she_mu_hdr {
	u8 ver;
	u8 size;
	u8 command;
	u8 tag;
}  __packed;

/* Status of a char device */
enum mu_device_status_t {
	MU_FREE,
	MU_OPENED
};

struct seco_shared_mem {
	dma_addr_t dma_addr;
	u32 size;
	u32 pos;
	u8 *ptr;
};

struct seco_out_buffer_desc {
	u8 *out_ptr;
	u8 *out_usr_ptr;
	u32 out_size;
	struct list_head link;
};

/* Private struct for each char device instance. */
struct seco_mu_device_ctx {
	struct device *dev;
	struct seco_mu_priv *mu_priv;
	struct miscdevice miscdev;

	enum mu_device_status_t status;
	wait_queue_head_t wq;
	struct semaphore fops_lock;

	u32 pending_hdr;
	struct list_head pending_out;

	struct seco_shared_mem secure_mem;
	struct seco_shared_mem non_secure_mem;

	u32 temp_cmd[MAX_MESSAGE_SIZE];
	u32 temp_resp[MAX_RECV_SIZE];
	u32 temp_resp_size;
	struct notifier_block scu_notify;
	bool v2x_reset;
};

/* Private struct for seco MU driver. */
struct seco_mu_priv {
	struct seco_mu_device_ctx *cmd_receiver_dev;
	struct seco_mu_device_ctx *waiting_rsp_dev;
	/*
	 * prevent parallel access to the MU registers
	 * e.g. a user trying to send a command while the other one is
	 * sending a response.
	 */
	struct mutex mu_lock;
	/*
	 * prevent a command to be sent on the MU while another one is still
	 * processing. (response to a command is allowed)
	 */
	struct mutex mu_cmd_lock;
	struct device *dev;
	u32 seco_mu_id;
	u8 cmd_tag;
	u8 rsp_tag;

	struct mbox_client cl;
	struct mbox_chan *tx_chan;
	struct mbox_chan *rx_chan;

	struct imx_sc_ipc *ipc_scu;
	u8 seco_part_owner;

	int max_ctx;
	struct seco_mu_device_ctx **ctxs;
};

/* macro to log operation of a misc device */
#define miscdev_dbg(p_miscdev, fmt, va_args...)                                \
	({                                                                     \
		struct miscdevice *_p_miscdev = p_miscdev;                     \
		dev_dbg((_p_miscdev)->parent, "%s: " fmt, (_p_miscdev)->name,  \
		##va_args);                                                    \
	})

#define miscdev_info(p_miscdev, fmt, va_args...)                               \
	({                                                                     \
		struct miscdevice *_p_miscdev = p_miscdev;                     \
		dev_info((_p_miscdev)->parent, "%s: " fmt, (_p_miscdev)->name, \
		##va_args);                                                    \
	})

#define miscdev_err(p_miscdev, fmt, va_args...)                                \
	({                                                                     \
		struct miscdevice *_p_miscdev = p_miscdev;                     \
		dev_err((_p_miscdev)->parent, "%s: " fmt, (_p_miscdev)->name,  \
		##va_args);                                                    \
	})

/* macro to log operation of a device context */
#define devctx_dbg(p_devctx, fmt, va_args...) \
	miscdev_dbg(&((p_devctx)->miscdev), fmt, ##va_args)
#define devctx_info(p_devctx, fmt, va_args...) \
	miscdev_info(&((p_devctx)->miscdev), fmt, ##va_args)
#define devctx_err(p_devctx, fmt, va_args...) \
	miscdev_err((&(p_devctx)->miscdev), fmt, ##va_args)

#define IMX_SC_RM_PERM_FULL         7U	/* Full access */

/* Give access to SECU to the memory we want to share */
static int seco_mu_setup_seco_memory_access(struct seco_mu_device_ctx *dev_ctx,
					    u64 addr, u32 len)
{
	struct seco_mu_priv *priv = dev_get_drvdata(dev_ctx->dev);
	int ret;
	u8 mr;

	ret = imx_sc_rm_find_memreg(priv->ipc_scu, &mr, addr, addr + len);
	if (ret) {
		devctx_err(dev_ctx, "Fail find memreg\n");
		goto exit;
	}

	ret = imx_sc_rm_set_memreg_permissions(priv->ipc_scu, mr,
					       priv->seco_part_owner,
					       IMX_SC_RM_PERM_FULL);
	if (ret) {
		devctx_err(dev_ctx, "Fail set permission for resource\n");
		goto exit;
	}

exit:
	return ret;
}

/*
 * File operations for user-space
 */
/* Open a char device. */
static int seco_mu_fops_open(struct inode *nd, struct file *fp)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
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
	 * Allocate some memory for data exchanges with SECO.
	 * This will be used for data not requiring secure memory.
	 */
	dev_ctx->non_secure_mem.ptr = dmam_alloc_coherent(dev_ctx->dev,
					MAX_DATA_SIZE_PER_USER,
					&dev_ctx->non_secure_mem.dma_addr,
					GFP_KERNEL);
	if (!dev_ctx->non_secure_mem.ptr) {
		err = -ENOMEM;
		devctx_err(dev_ctx, "Failed to map shared memory with SECO\n");
		goto exit;
	}

	err = seco_mu_setup_seco_memory_access(dev_ctx,
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
	dev_ctx->v2x_reset = 0;

	goto exit;

free_coherent:
	dmam_free_coherent(dev_ctx->mu_priv->dev, MAX_DATA_SIZE_PER_USER,
			   dev_ctx->non_secure_mem.ptr,
			   dev_ctx->non_secure_mem.dma_addr);

exit:
	up(&dev_ctx->fops_lock);
	return err;
}

/* Close a char device. */
static int seco_mu_fops_close(struct inode *nd, struct file *fp)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	struct seco_mu_priv *mu_priv = dev_ctx->mu_priv;
	struct seco_out_buffer_desc *out_buf_desc;

	/* Avoid race if closed at the same time */
	if (down_trylock(&dev_ctx->fops_lock))
		return -EBUSY;

	/* The device context has not been opened */
	if (dev_ctx->status != MU_OPENED)
		goto exit;

	/* check if this device was registered as command receiver. */
	if (mu_priv->cmd_receiver_dev == dev_ctx)
		mu_priv->cmd_receiver_dev = NULL;

	/* check if this device was registered as waiting response. */
	if (mu_priv->waiting_rsp_dev == dev_ctx) {
		mu_priv->waiting_rsp_dev = NULL;
		mutex_unlock(&mu_priv->mu_cmd_lock);
	}

	/* Unmap secure memory shared buffer. */
	if (dev_ctx->secure_mem.ptr)
		devm_iounmap(dev_ctx->dev, dev_ctx->secure_mem.ptr);

	dev_ctx->secure_mem.ptr = NULL;
	dev_ctx->secure_mem.dma_addr = 0;
	dev_ctx->secure_mem.size = 0;
	dev_ctx->secure_mem.pos = 0;

	/* Free non-secure shared buffer. */
	dmam_free_coherent(dev_ctx->mu_priv->dev, MAX_DATA_SIZE_PER_USER,
			   dev_ctx->non_secure_mem.ptr,
			   dev_ctx->non_secure_mem.dma_addr);

	dev_ctx->non_secure_mem.ptr = NULL;
	dev_ctx->non_secure_mem.dma_addr = 0;
	dev_ctx->non_secure_mem.size = 0;
	dev_ctx->non_secure_mem.pos = 0;

	while (!list_empty(&dev_ctx->pending_out)) {
		out_buf_desc = list_first_entry_or_null(&dev_ctx->pending_out,
						struct seco_out_buffer_desc,
						link);
		__list_del_entry(&out_buf_desc->link);
		devm_kfree(dev_ctx->dev, out_buf_desc);
	}

	dev_ctx->status = MU_FREE;

exit:
	up(&dev_ctx->fops_lock);
	return 0;
}

/* Write a message to the MU. */
static ssize_t seco_mu_fops_write(struct file *fp, const char __user *buf,
				  size_t size, loff_t *ppos)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	struct seco_mu_priv *mu_priv = dev_ctx->mu_priv;
	u32 nb_words = 0, header;
	int err;

	devctx_dbg(dev_ctx, "write from buf (%p)%ld, ppos=%lld\n", buf, size,
		   ((ppos) ? *ppos : 0));

	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	if (dev_ctx->status != MU_OPENED) {
		err = -EINVAL;
		goto exit;
	}

	if (size < sizeof(struct she_mu_hdr)) {
		devctx_err(dev_ctx, "User buffer too small(%ld < %lu)\n", size,
			   sizeof(struct she_mu_hdr));
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

	header = dev_ctx->temp_cmd[0];

	/* Check the message is valid according to tags */
	if (MESSAGE_TAG(header) == mu_priv->cmd_tag) {
		/*
		 * unlocked in seco_mu_receive_work_handler when the
		 * response to this command is received.
		 */
		mutex_lock(&mu_priv->mu_cmd_lock);
		mu_priv->waiting_rsp_dev = dev_ctx;
	} else if (MESSAGE_TAG(header) == mu_priv->rsp_tag) {
		/* Check the device context can send the command */
		if (dev_ctx != mu_priv->cmd_receiver_dev) {
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
	nb_words = MESSAGE_SIZE(header);
	if (nb_words * sizeof(u32) != size) {
		devctx_err(dev_ctx, "User buffer too small\n");
		goto exit;
	}

	mutex_lock(&mu_priv->mu_lock);

	/* Send message */
	devctx_dbg(dev_ctx, "sending message\n");
	err = mbox_send_message(mu_priv->tx_chan, dev_ctx->temp_cmd);
	if (err < 0) {
		devctx_err(dev_ctx, "Failed to send message\n");
		goto unlock;
	}

	err = nb_words * (u32)sizeof(u32);

unlock:
	mutex_unlock(&mu_priv->mu_lock);

exit:
	up(&dev_ctx->fops_lock);
	return err;
}

/*
 * Read a message from the MU.
 * Blocking until a message is available.
 */
static ssize_t seco_mu_fops_read(struct file *fp, char __user *buf,
				 size_t size, loff_t *ppos)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	u32 data_size = 0, size_to_copy = 0;
	struct seco_out_buffer_desc *b_desc;
	int err;

	devctx_dbg(dev_ctx, "read to buf %p(%ld), ppos=%lld\n", buf, size,
		   ((ppos) ? *ppos : 0));

	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	if (dev_ctx->status != MU_OPENED) {
		err = -EINVAL;
		goto exit;
	}

	if (dev_ctx->v2x_reset) {
		err = -EINVAL;
		goto exit;
	}

	/* Wait until the complete message is received on the MU. */
	err = wait_event_interruptible(dev_ctx->wq, dev_ctx->pending_hdr != 0);
	if (err) {
		devctx_err(dev_ctx, "Interrupted by signal\n");
		goto exit;
	}

	if (dev_ctx->v2x_reset) {
		err = -EINVAL;
		dev_ctx->v2x_reset = 0;
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
						  struct seco_out_buffer_desc,
						  link);
		if (b_desc->out_usr_ptr && b_desc->out_ptr) {
			devctx_dbg(dev_ctx, "Copy output data to user\n");
			err = (int)copy_to_user(b_desc->out_usr_ptr,
						b_desc->out_ptr,
						b_desc->out_size);
			if (err) {
				devctx_err(dev_ctx,
					   "Failed to copy output data to user\n");
				err = -EFAULT;
				goto exit;
			}
		}
		__list_del_entry(&b_desc->link);
		devm_kfree(dev_ctx->dev, b_desc);
	}

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
	up(&dev_ctx->fops_lock);
	return err;
}

/* Configure the shared memory according to user config */
static int
seco_mu_ioctl_shared_mem_cfg_handler(struct seco_mu_device_ctx *dev_ctx,
				     unsigned long arg)
{
	struct seco_mu_ioctl_shared_mem_cfg cfg;
	int err = -EINVAL;
	u64 high_boundary;

	/* Check if not already configured. */
	if (dev_ctx->secure_mem.dma_addr != 0u) {
		devctx_err(dev_ctx, "Shared memory not configured\n");
		goto exit;
	}

	err = (int)copy_from_user(&cfg, (u8 *)arg,
		sizeof(cfg));
	if (err) {
		devctx_err(dev_ctx, "Fail copy shared memory config to user\n");
		err = -EFAULT;
		goto exit;
	}

	devctx_dbg(dev_ctx, "cfg offset: %u(%d)\n", cfg.base_offset, cfg.size);

	high_boundary = cfg.base_offset;
	if (high_boundary > SECURE_RAM_SIZE) {
		devctx_err(dev_ctx, "base offset is over secure memory\n");
		err = -ENOMEM;
		goto exit;
	}

	high_boundary += cfg.size;
	if (high_boundary > SECURE_RAM_SIZE) {
		devctx_err(dev_ctx, "total memory is over secure memory\n");
		err = -ENOMEM;
		goto exit;
	}

	dev_ctx->secure_mem.dma_addr = (dma_addr_t)cfg.base_offset;
	dev_ctx->secure_mem.size = cfg.size;
	dev_ctx->secure_mem.pos = 0;
	dev_ctx->secure_mem.ptr = devm_ioremap(dev_ctx->dev,
					(phys_addr_t)(SECURE_RAM_BASE_ADDRESS +
					(u64)dev_ctx->secure_mem.dma_addr),
					dev_ctx->secure_mem.size);
	if (!dev_ctx->secure_mem.ptr) {
		devctx_err(dev_ctx, "Failed to map secure memory\n");
		err = -ENOMEM;
		goto exit;
	}

exit:
	return err;
}

/*
 * Copy a buffer of daa to/from the user and return the address to use in
 * messages
 */
static int seco_mu_ioctl_setup_iobuf_handler(struct seco_mu_device_ctx *dev_ctx,
					     unsigned long arg)
{
	struct seco_out_buffer_desc *out_buf_desc;
	struct seco_mu_ioctl_setup_iobuf io;
	struct seco_shared_mem *shared_mem;
	int err = -EINVAL;
	u32 pos;
	u8 *addr;

	struct seco_mu_priv *priv = dev_get_drvdata(dev_ctx->dev);

	err = (int)copy_from_user(&io,
		(u8 *)arg,
		sizeof(io));
	if (err) {
		devctx_err(dev_ctx, "Failed copy iobuf config from user\n");
		err = -EFAULT;
		goto exit;
	}

	/* Function call to retrieve MU Buffer address */
	if (io.flags & SECO_MU_IO_FLAGS_SHE_V2X)
		addr = get_mu_buf(priv->tx_chan);

	devctx_dbg(dev_ctx, "io [buf: %p(%d) flag: %x]\n",
		   io.user_buf, io.length, io.flags);

	if (io.length == 0 || !io.user_buf) {
		/*
		 * Accept NULL pointers since some buffers are optional
		 * in SECO commands. In this case we should return 0 as
		 * pointer to be embedded into the message.
		 * Skip all data copy part of code below.
		 */
		io.seco_addr = 0;
		goto copy;
	}

	/* Select the shared memory to be used for this buffer. */
	if (!(io.flags & SECO_MU_IO_FLAGS_SHE_V2X)) {
		if (io.flags & SECO_MU_IO_FLAGS_USE_SEC_MEM) {
			/* App requires to use secure memory for this buffer.*/
			shared_mem = &dev_ctx->secure_mem;
		} else {
			/* No specific requirement for this buffer. */
			shared_mem = &dev_ctx->non_secure_mem;
		}
	}

	/* Check there is enough space in the shared memory. */
	if (!(io.flags & SECO_MU_IO_FLAGS_SHE_V2X) &&
	     (io.length >= shared_mem->size - shared_mem->pos)) {
		devctx_err(dev_ctx, "Not enough space in shared memory\n");
		err = -ENOMEM;
		goto exit;
	}

	if (!(io.flags & SECO_MU_IO_FLAGS_SHE_V2X)) {
		/* Allocate space in shared memory. 8 bytes aligned. */
		pos = shared_mem->pos;
		shared_mem->pos += round_up(io.length, 8u);
		io.seco_addr = (u64)shared_mem->dma_addr + pos;
	} else {
		if (io.length > MAX_DATA_SIZE_PER_USER) {
			devctx_err(dev_ctx, "Buffer length exceeded the max limit\n");
			err = -ENOMEM;
			goto exit;
		}

		io.seco_addr = (u64)addr;
	}

	if ((io.flags & SECO_MU_IO_FLAGS_USE_SEC_MEM) &&
	    !(io.flags & SECO_MU_IO_FLAGS_USE_SHORT_ADDR))
		/*Add base address to get full address.*/
		io.seco_addr += SECURE_RAM_BASE_ADDRESS_SCU;

	if (io.flags & SECO_MU_IO_FLAGS_IS_INPUT) {
		/*
		 * buffer is input:
		 * copy data from user space to this allocated buffer.
		 */
		if (io.flags & SECO_MU_IO_FLAGS_SHE_V2X) {
			err = (int)copy_from_user(addr, io.user_buf, io.length);
		} else {
			err = (int)copy_from_user(shared_mem->ptr + pos,
						  io.user_buf,
						  io.length);
		}
		if (err) {
			devctx_err(dev_ctx,
				   "Failed copy data to shared memory\n");
			err = -EFAULT;
			goto exit;
		}
	} else {
		/*
		 * buffer is output:
		 * add an entry in the "pending buffers" list so data
		 * can be copied to user space when receiving SECO
		 * response.
		 */
		out_buf_desc = devm_kmalloc(dev_ctx->dev, sizeof(*out_buf_desc),
					    GFP_KERNEL);
		if (!out_buf_desc) {
			err = -ENOMEM;
			devctx_err(dev_ctx,
				   "Failed allocating mem for pending buffer\n"
				   );
			goto exit;
		}

		if (io.flags & SECO_MU_IO_FLAGS_SHE_V2X)
			out_buf_desc->out_ptr = addr;
		else
			out_buf_desc->out_ptr = shared_mem->ptr + pos;
		out_buf_desc->out_usr_ptr = io.user_buf;
		out_buf_desc->out_size = io.length;
		list_add_tail(&out_buf_desc->link, &dev_ctx->pending_out);
	}

copy:
	/* Provide the seco address to user space only if success. */
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

/* Retrieve info about the MU */
static int seco_mu_ioctl_get_mu_info_handler(struct seco_mu_device_ctx *dev_ctx,
					     unsigned long arg)
{
	struct seco_mu_priv *priv = dev_get_drvdata(dev_ctx->dev);
	struct seco_mu_ioctl_get_mu_info info;
	int err = -EINVAL;

	info.seco_mu_idx = (u8)priv->seco_mu_id;
	info.interrupt_idx = SECO_MU_INTERRUPT_INDEX;
	info.tz = SECO_DEFAULT_TZ;

	err = imx_sc_rm_get_did(priv->ipc_scu, &info.did);
	if (err) {
		devctx_err(dev_ctx, "Get did failed\n");
		goto exit;
	}

	devctx_dbg(dev_ctx,
		   "info [mu_idx: %d, irq_idx: %d, tz: 0x%x, did: 0x%x]\n",
		   info.seco_mu_idx, info.interrupt_idx, info.tz, info.did);

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

static int seco_mu_ioctl_signed_msg_handler(struct seco_mu_device_ctx *dev_ctx,
					    unsigned long arg)
{
	struct seco_shared_mem *shared_mem = &dev_ctx->non_secure_mem;
	struct seco_mu_priv *priv = dev_get_drvdata(dev_ctx->dev);
	struct seco_mu_ioctl_signed_message msg;
	int err = -EINVAL;
	u64 addr;
	u32 pos;

	err = (int)copy_from_user(&msg,
		(u8 *)arg,
		sizeof(msg));
	if (err) {
		devctx_err(dev_ctx, "Failed to copy from user: %d\n", err);
		err = -EFAULT;
		goto exit;
	}

	/* Check there is enough space in the shared memory. */
	if (msg.msg_size >= shared_mem->size - shared_mem->pos) {
		devctx_err(dev_ctx, "Not enough mem: %d left, %d required\n",
			   shared_mem->size - shared_mem->pos, msg.msg_size);
		err = -ENOMEM;
		goto exit;
	}

	/* Allocate space in shared memory. 8 bytes aligned. */
	pos = shared_mem->pos;

	/* get physical address from the pos */
	addr = (u64)shared_mem->dma_addr + pos;

	/* copy signed message from user space to this allocated buffer */
	err = (int)copy_from_user(shared_mem->ptr + pos, msg.message,
				  msg.msg_size);
	if (err) {
		devctx_err(dev_ctx, "Failed to signed message from user: %d\n",
			   err);
		err = -EFAULT;
		goto exit;
	}

	/* Send the message to SECO through SCU */
	msg.error_code = imx_sc_seco_sab_msg(priv->ipc_scu, addr);

	err = (int)copy_to_user((u8 *)arg, &msg,
		sizeof(msg));
	if (err) {
		devctx_err(dev_ctx, "Failed to copy to user: %d\n", err);
		err = -EFAULT;
		goto exit;
	}

exit:
	return err;
}

/* IOCTL entry point of a char device */
static long seco_mu_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	struct seco_mu_priv *mu_priv = dev_ctx->mu_priv;
	int err = -EINVAL;

	/* Prevent race during change of device context */
	if (down_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	switch (cmd) {
	case SECO_MU_IOCTL_ENABLE_CMD_RCV:
		if (!mu_priv->cmd_receiver_dev) {
			devctx_dbg(dev_ctx, "setting as receiver\n");
			mu_priv->cmd_receiver_dev = dev_ctx;
			err = 0;
		};
		break;
	case SECO_MU_IOCTL_SHARED_BUF_CFG:
		err = seco_mu_ioctl_shared_mem_cfg_handler(dev_ctx, arg);
		break;
	case SECO_MU_IOCTL_SETUP_IOBUF:
		err = seco_mu_ioctl_setup_iobuf_handler(dev_ctx, arg);
		break;
	case SECO_MU_IOCTL_GET_MU_INFO:
		err = seco_mu_ioctl_get_mu_info_handler(dev_ctx, arg);
		break;
	case SECO_MU_IOCTL_SIGNED_MESSAGE:
		err = seco_mu_ioctl_signed_msg_handler(dev_ctx, arg);
		break;
	default:
		err = -EINVAL;
		devctx_dbg(dev_ctx, "IOCTL %.8x not supported\n", cmd);
	}

	up(&dev_ctx->fops_lock);
	return (long)err;
}

/*
 * Callback called by mailbox FW when data are received
 */
static void seco_mu_rx_callback(struct mbox_client *c, void *msg)
{
	struct device *dev = c->dev;
	struct seco_mu_priv *priv = dev_get_drvdata(dev);
	struct seco_mu_device_ctx *dev_ctx;
	bool is_response = false;
	int msg_size;
	u32 header;

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

	header = *(u32 *)msg;

	dev_dbg(dev, "Selecting device\n");

	/* Incoming command: wake up the receiver if any. */
	if (MESSAGE_TAG(header) == priv->cmd_tag) {
		dev_dbg(dev, "Selecting cmd receiver\n");
		dev_ctx = priv->cmd_receiver_dev;
	} else if (MESSAGE_TAG(header) == priv->rsp_tag) {
		dev_dbg(dev, "Selecting rsp waiter\n");
		dev_ctx = priv->waiting_rsp_dev;
		is_response = true;
	} else {
		dev_err(dev, "Failed to select a device for message: %.8x\n",
			header);
		return;
	}

	if (!dev_ctx) {
		dev_err(dev, "No device context selected for message: %.8x\n",
			header);
		return;
	}

	/* Init reception */
	msg_size = MESSAGE_SIZE(header);
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

	if (is_response) {
		/* Allow user to send new command */
		mutex_unlock(&priv->mu_cmd_lock);
	}
}

#define SECO_FW_VER_FEAT_MASK		(0x0000FFF0u)
#define SECO_FW_VER_FEAT_SHIFT		(0x04u)
#define SECO_FW_VER_FEAT_MIN_ALL_MU	(0x04u)

/*
 * Get SECO FW version and check if it supports receiving commands on all MUs
 * The version is retrieved through SCU since this is the only communication
 * channel to SECO always present.
 */
static int seco_mu_check_all_mu_supported(struct device *dev)
{
	struct seco_mu_priv *priv = dev_get_drvdata(dev);
	u32 seco_ver;
	int ret;

	ret = imx_sc_seco_build_info(priv->ipc_scu, &seco_ver, NULL);
	if (ret) {
		dev_err(dev, "failed to retrieve SECO build info\n");
		goto exit;
	}

	if (((seco_ver & SECO_FW_VER_FEAT_MASK) >> SECO_FW_VER_FEAT_SHIFT)
		< SECO_FW_VER_FEAT_MIN_ALL_MU) {
		dev_err(dev, "current SECO FW do not support MU with Linux\n");
		ret = -ENOTSUPP;
		goto exit;
	}

exit:
	return ret;
}

/* Char driver setup */
static const struct file_operations seco_mu_fops = {
	.open		= seco_mu_fops_open,
	.owner		= THIS_MODULE,
	.read		= seco_mu_fops_read,
	.release	= seco_mu_fops_close,
	.write		= seco_mu_fops_write,
	.unlocked_ioctl = seco_mu_ioctl,
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

static int seco_mu_request_channel(struct device *dev,
				   struct mbox_chan **chan,
				   const char *name)
{
	struct seco_mu_priv *priv = dev_get_drvdata(dev);
	struct mbox_chan *t_chan;
	int ret = 0;

	t_chan = mbox_request_channel_byname(&priv->cl, name);
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

static int imx_sc_v2x_reset_notify(struct notifier_block *nb,
                                      unsigned long event, void *group)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(nb,
					struct seco_mu_device_ctx, scu_notify);

	if (!(event & IMX_SC_IRQ_V2X_RESET))
		return 0;

	dev_ctx->v2x_reset = true;

	wake_up_interruptible(&dev_ctx->wq);
	return 0;
}
/* Driver probe.*/
static int seco_mu_probe(struct platform_device *pdev)
{
	struct seco_mu_device_ctx *dev_ctx;
	struct device *dev = &pdev->dev;
	struct seco_mu_priv *priv;
	struct device_node *np;
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
		ret = -ENOTSUPP;
		goto exit;
	}

	ret = imx_scu_get_handle(&priv->ipc_scu);
	if (ret) {
		dev_err(dev, "Fail to retrieve IPC handle\n");
		goto exit;
	}

	ret = imx_sc_rm_get_resource_owner(priv->ipc_scu, IMX_SC_R_SECO,
					   &priv->seco_part_owner);
	if (ret) {
		dev_err(dev, "Fail get owner of SECO resource\n");
		goto exit;
	}

	ret = seco_mu_check_all_mu_supported(dev);
	if (ret) {
		dev_err(dev, "Fail seco_mu_check_all_mu_supported\n");
		goto exit;
	}

	/* Initialize the mutex. */
	mutex_init(&priv->mu_cmd_lock);
	mutex_init(&priv->mu_lock);

	priv->cmd_receiver_dev = NULL;
	priv->waiting_rsp_dev = NULL;

	ret = of_property_read_u32(np, "fsl,seco_mu_id", &priv->seco_mu_id);
	if (ret) {
		dev_warn(dev, "%s: Not able to read mu_id", __func__);
		priv->seco_mu_id = SECO_DEFAULT_MU_INDEX;
	}

	ret = of_property_read_u32(np, "fsl,seco_max_users", &max_nb_users);
	if (ret) {
		dev_warn(dev, "%s: Not able to read mu_max_user", __func__);
		max_nb_users = SECO_MU_DEFAULT_MAX_USERS;
	}

	ret = of_property_read_u8(np, "fsl,cmd_tag", &priv->cmd_tag);
	if (ret)
		priv->cmd_tag = DEFAULT_MESSAGING_TAG_COMMAND;

	ret = of_property_read_u8(np, "fsl,rsp_tag", &priv->rsp_tag);
	if (ret)
		priv->rsp_tag = DEFAULT_MESSAGING_TAG_RESPONSE;

	/* Mailbox client configuration */
	priv->cl.dev = dev;
	priv->cl.knows_txdone = true;
	priv->cl.rx_callback = seco_mu_rx_callback;

	ret = seco_mu_request_channel(dev, &priv->tx_chan, "txdb");
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request txdb channel\n");

		goto exit;
	}

	ret = seco_mu_request_channel(dev, &priv->rx_chan, "rxdb");
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request rxdb channel\n");

		goto exit;
	}

	priv->max_ctx = max_nb_users;
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
		dev_ctx->mu_priv = priv;

		priv->ctxs[i] = dev_ctx;

		/* Default value invalid for an header. */
		init_waitqueue_head(&dev_ctx->wq);

		INIT_LIST_HEAD(&dev_ctx->pending_out);
		sema_init(&dev_ctx->fops_lock, 1);

		devname = devm_kasprintf(dev, GFP_KERNEL, "seco_mu%d_ch%d",
					 priv->seco_mu_id, i);
		if (!devname) {
			ret = -ENOMEM;
			dev_err(dev,
				"Fail to allocate memory for misc dev name\n");
			goto exit;
		}

		dev_ctx->miscdev.name = devname;
		dev_ctx->miscdev.minor	= MISC_DYNAMIC_MINOR;
		dev_ctx->miscdev.fops = &seco_mu_fops;
		dev_ctx->miscdev.parent = dev;
		ret = misc_register(&dev_ctx->miscdev);
		if (ret) {
			dev_err(dev, "failed to register misc device %d\n",
				ret);
			goto exit;
		}

		ret = devm_add_action(dev, if_misc_deregister,
				      &dev_ctx->miscdev);

		dev_ctx->scu_notify.notifier_call = imx_sc_v2x_reset_notify;

		ret = imx_scu_irq_register_notifier(&dev_ctx->scu_notify);
		if (ret) {
			dev_err(&pdev->dev, "v2x reqister scu notifier failed.\n");
			return ret;
		}

		if (ret)
			dev_warn(dev,
				 "failed to add managed removal of miscdev\n");
	}

	ret = imx_scu_irq_group_enable(IMX_SC_IRQ_GROUP_WAKE,
					IMX_SC_IRQ_V2X_RESET, true);
	if (ret) {
		dev_warn(&pdev->dev, "v2x Enable irq failed.\n");
		return ret;
	}

exit:
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int secu_mu_resume(struct device *dev)
{
	struct seco_mu_priv *priv = dev_get_drvdata(dev);
	int i=0;

	for (i = 0; i < priv->max_ctx; i++) {
		priv->ctxs[i]->v2x_reset = true;
		wake_up_interruptible(&priv->ctxs[i]->wq);
	}
	return 0;
}
#endif

static const struct of_device_id seco_mu_match[] = {
	{
		.compatible = "fsl,imx-seco-mu",
	},
	{},
};
MODULE_DEVICE_TABLE(of, seco_mu_match);

static const struct dev_pm_ops secu_mu_pm = {
        SET_SYSTEM_SLEEP_PM_OPS(NULL, secu_mu_resume)
};

static struct platform_driver seco_mu_driver = {
	.driver = {
		.name = "seco_mu",
		.of_match_table = seco_mu_match,
		.pm = &secu_mu_pm,
	},
	.probe       = seco_mu_probe,
};

module_platform_driver(seco_mu_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IMX Seco MU");
MODULE_AUTHOR("NXP");
