// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2019 NXP
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
#include <soc/imx8/sc/sci.h>
#include <soc/imx8/sc/svc/seco/api.h>
#include <uapi/linux/seco_mu_ioctl.h>

struct she_mu_hdr {
	uint8_t ver;
	uint8_t size;
	uint8_t command;
	uint8_t tag;
};

#define MESSAGE_SIZE(hdr) (((struct she_mu_hdr *)(&(hdr)))->size)
#define MESSAGE_TAG(hdr) (((struct she_mu_hdr *)(&(hdr)))->tag)

#define MESSAGING_TAG_COMMAND           (0x17u)
#define MESSAGING_TAG_RESPONSE          (0xe1u)


enum mu_device_status_t {
	MU_FREE,
	MU_OPENED
};

/* Offsets of MUs registers */
#define MU_TR_OFFSET(n)		(4u*(n))
#define MU_RR_OFFSET(n)		(0x10u + 4u*(n))
#define MU_SR_OFFSET		(0x20u)
#define MU_CR_OFFSET		(0x24u)
/* number of MU_TR registers */
#define MU_TR_COUNT		(4u)
/*  number of MU_RR registers */
#define MU_RR_COUNT		(4u)

/* SR Bit Fields */
#define MU_SR_TE_MASK(n)	(1UL << (23u - ((n)&3u)))
#define MU_SR_RF_MASK(n)	(1UL << (27u - ((n)&3u)))
#define MU_SR_GIP_MASK(n)	(1UL << (31u - ((n)&3u)))

/* CR Bit fields */
#define MU_CR_Fn_MASK		(0x7U)
#define MU_CR_GIRn_MASK		(0xF0000U)
#define MU_CR_GIR_MASK(n)	(1UL << (19u - ((n)&3u)))
#define MU_CR_TIEn_MASK		(0xF00000U)
#define MU_CR_RIEn_MASK		(0xF000000U)
#define MU_CR_GIEn_MASK		(0xF0000000U)
#define MU_CR_GIE_MASK(n)	(1UL << (31u - ((n)&3u)))

#define SECURE_RAM_BASE_ADDRESS	(0x31800000ULL)
#define SECURE_RAM_BASE_ADDRESS_SCU	(0x20800000u)
#define SECURE_RAM_SIZE		(64*1024) /* 64 kBytes*/

#define SECO_MU_INTERRUPT_INDEX	(0u)
#define SECO_DEFAULT_MU_INDEX	(1u)
#define SECO_DEFAULT_TZ		(0u)
#define DEFAULT_DID		(0u)

#define MAX_DATA_SIZE_PER_USER  (16*1024)

struct seco_shared_mem {
	dma_addr_t dma_addr;
	uint32_t size;
	uint32_t pos;
	uint8_t *ptr;
};

struct seco_out_buffer_desc {
	uint8_t *out_ptr;
	uint8_t *out_usr_ptr;
	uint32_t out_size;
	struct list_head link;
};

/* Private struct for each char device instance. */
struct seco_mu_device_ctx {
	enum mu_device_status_t status;
	wait_queue_head_t wq;
	struct miscdevice miscdev;
	struct seco_mu_priv *mu_priv;
	uint32_t pending_hdr;
	struct list_head pending_out;
	struct seco_shared_mem secure_mem;
	struct seco_shared_mem non_secure_mem;
};

/* Private struct for seco MU driver. */
struct seco_mu_priv {
	struct seco_mu_device_ctx *cmd_receiver_dev;
	struct seco_mu_device_ctx *waiting_rsp_dev;
	struct work_struct seco_mu_work;
	/*
	 * prevent parallel access to the MU registers
	 * e.g. a user trying to send a command while the other one is
	 * sending a response.
	 */
	struct mutex mu_regs_lock;
	/*
	 * prevent a command to be sent on the MU while another one is still
	 * processing. (response to a command is allowed)
	 */
	struct mutex mu_cmd_lock;
	void __iomem *mu_base;
	struct device *dev;
	uint32_t seco_mu_id;
};

/*
 * MU registers access
 */

/* Initialize MU: disable all interrupts except GI0 and clear status bits.*/
static void seco_mu_init(void __iomem *mu_base)
{
	uint32_t cr = readl(mu_base + MU_CR_OFFSET);
	/* Disable all interrupts and clear flags. */
	cr &= ~(MU_CR_GIEn_MASK | MU_CR_RIEn_MASK | MU_CR_TIEn_MASK
				| MU_CR_GIRn_MASK | MU_CR_Fn_MASK);

	/* Enable only general purpose interrupt 0. */
	cr |= (uint32_t)MU_CR_GIE_MASK(SECO_MU_INTERRUPT_INDEX);

	writel(cr, mu_base + MU_CR_OFFSET);
}

/* Write a 32bits word to a given TR register of the MU.*/
static void seco_mu_send_data(void __iomem *mu_base, uint8_t regid,
				uint32_t data)
{
	while (!(readl(mu_base + MU_SR_OFFSET)
		& MU_SR_TE_MASK(regid))) {
	};
	writel(data, mu_base + MU_TR_OFFSET(regid));
}

/* Return RF (Receive Full) status for given register of the MU.*/
static uint32_t seco_mu_get_rf(void __iomem *mu_base, uint8_t regid)
{
	return (uint32_t)(readl(mu_base + MU_SR_OFFSET)
				& MU_SR_RF_MASK(regid));
}

/* Read a 32bits word from a given RR register of the MU. */
static uint32_t seco_mu_receive_data(void __iomem *mu_base, uint8_t regid)
{
	while (!seco_mu_get_rf(mu_base,
				regid)) {
	};
	return readl(mu_base + MU_RR_OFFSET(regid));
}

/* Request an interrupt on the other side of the MU. */
static void seco_mu_set_gir(void __iomem *mu_base, uint8_t id)
{
	uint32_t cr = readl(mu_base + MU_CR_OFFSET);

	cr |= (uint32_t)MU_CR_GIR_MASK(id);
	writel(cr, mu_base + MU_CR_OFFSET);
}

/* Check if an interrupt is pending.*/
static uint32_t seco_mu_get_gip(void __iomem *mu_base, uint8_t id)
{
	return (uint32_t)(readl(mu_base + MU_SR_OFFSET) & MU_SR_GIP_MASK(id));
}

/* Clear an interrupt pending status bit. */
static void seco_mu_clear_gip(void __iomem *mu_base, uint8_t id)
{
	/* Bits in this register are either read-only or "write 1 to clear".
	 * So the write below has only effect to clear GIP(id) bit.
	 */
	writel(MU_SR_GIP_MASK(id), mu_base + MU_SR_OFFSET);
}


static long setup_seco_memory_access(uint64_t addr, uint32_t len)
{
	sc_ipc_t ipc;
	uint32_t mu_id;
	sc_err_t sciErr;
	sc_rm_mr_t mr;
	sc_rm_pt_t pt;
	long err = -EPERM;

	sciErr = (sc_err_t)sc_ipc_getMuID(&mu_id);

	if (sciErr == SC_ERR_NONE) {
		do {
			sciErr = sc_ipc_open(&ipc, mu_id);
			if (sciErr != SC_ERR_NONE)
				break;

			sciErr = sc_rm_find_memreg(ipc,	&mr, addr, addr+len);
			if (sciErr != SC_ERR_NONE)
				break;

			/* Find partition owning SECO. */
			sciErr = sc_rm_get_resource_owner(ipc, SC_R_SECO, &pt);
			if (sciErr != SC_ERR_NONE)
				break;

			/* Let SECO partition access this memory area. */
			sciErr = sc_rm_set_memreg_permissions(ipc, mr, pt,
							SC_RM_PERM_FULL);
			if (sciErr != SC_ERR_NONE)
				break;
			/* Success. */
			err = 0;
		} while (0);

		sc_ipc_close(ipc);
	}

	return err;
}

/*
 * File operations for user-space
 */
/* Open a device. */
static int seco_mu_open(struct inode *nd, struct file *fp)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	long err;

	/* Authorize only 1 instance. */
	if (dev_ctx->status != MU_FREE)
		return -EBUSY;

	/*
	 * Allocate some memory for data exchanges with SECO.
	 * This will be used for data not requiring secure memory.
	 */
	dev_ctx->non_secure_mem.ptr = dma_alloc_coherent(
					dev_ctx->mu_priv->dev,
					MAX_DATA_SIZE_PER_USER,
					&(dev_ctx->non_secure_mem.dma_addr),
					GFP_KERNEL);
	if (!dev_ctx->non_secure_mem.ptr)
		return -ENOMEM;

	err = setup_seco_memory_access(dev_ctx->non_secure_mem.dma_addr,
					MAX_DATA_SIZE_PER_USER);
	if (err)
		return -EPERM;


	dev_ctx->non_secure_mem.size = MAX_DATA_SIZE_PER_USER;
	dev_ctx->non_secure_mem.pos = 0;
	dev_ctx->status = MU_OPENED;

	return 0;
}

/* Close a device. */
static int seco_mu_close(struct inode *nd, struct file *fp)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	struct seco_mu_priv *mu_priv;
	struct seco_out_buffer_desc *out_buf_desc;

	do {
		mu_priv = dev_ctx->mu_priv;
		if (!mu_priv)
			break;

		dev_ctx->status = MU_FREE;

		/* check if this device was registered as command receiver. */
		if (mu_priv->cmd_receiver_dev == dev_ctx)
			mu_priv->cmd_receiver_dev = NULL;

		/* Unmap secure memory shared buffer. */
		iounmap(dev_ctx->secure_mem.ptr);
		dev_ctx->secure_mem.ptr = NULL;
		dev_ctx->secure_mem.dma_addr = 0;
		dev_ctx->secure_mem.size = 0;
		dev_ctx->secure_mem.pos = 0;

		/* Free non-secure shared buffer. */
		dma_free_coherent(dev_ctx->mu_priv->dev,
					MAX_DATA_SIZE_PER_USER,
					dev_ctx->non_secure_mem.ptr,
					dev_ctx->non_secure_mem.dma_addr);
		dev_ctx->non_secure_mem.ptr = NULL;
		dev_ctx->non_secure_mem.dma_addr = 0;
		dev_ctx->non_secure_mem.size = 0;
		dev_ctx->non_secure_mem.pos = 0;

		while (!list_empty(&dev_ctx->pending_out)) {
			out_buf_desc = list_first_entry_or_null(
				&dev_ctx->pending_out,
				struct seco_out_buffer_desc,
				link);
			__list_del_entry(&out_buf_desc->link);
			kfree(out_buf_desc);
		}

	} while (0);

	return 0;
}

/* Write a message to the MU. */
static ssize_t seco_mu_write(struct file *fp, const char __user *buf,
						size_t size, loff_t *ppos)
{
	uint32_t data, data_idx = 0, nb_words;
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	int err;
	struct seco_mu_priv *mu_priv;

	do {
		mu_priv = dev_ctx->mu_priv;
		if (!mu_priv)
			break;

		err = (int)copy_from_user(&data, buf, sizeof(uint32_t));
		if (err)
			break;

		/* Check that the size passed as argument matches the size
		 * carried in the message.
		 */
		nb_words = MESSAGE_SIZE(data);
		if (nb_words * sizeof(uint32_t) != size)
			break;

		if (MESSAGE_TAG(data) == MESSAGING_TAG_COMMAND) {
			/*
			 * unlocked in seco_mu_receive_work_handler when the
			 * response to this command is received.
			 */
			mutex_lock(&(mu_priv->mu_cmd_lock));
			mu_priv->waiting_rsp_dev = dev_ctx;
		}

		mutex_lock(&(mu_priv->mu_regs_lock));
		seco_mu_send_data(mu_priv->mu_base, 0, data);
		seco_mu_set_gir(mu_priv->mu_base, 0);
		data_idx = 1;

		while (data_idx < nb_words) {
			err = (int)copy_from_user(&data,
					buf + data_idx*sizeof(uint32_t),
					sizeof(uint32_t));
			if (err)
				break;

			seco_mu_send_data(mu_priv->mu_base,
					(uint8_t)(data_idx % MU_TR_COUNT),
					data);
			data_idx++;
		}

		mutex_unlock(&(mu_priv->mu_regs_lock));
	} while (0);

	data_idx = data_idx * (uint32_t)sizeof(uint32_t);

	return (ssize_t)(data_idx);
}

/* Read a message from the MU.
 * Blocking until a message is available.
 */
static ssize_t seco_mu_read(struct file *fp, char __user *buf,
					size_t size, loff_t *ppos)
{
	uint32_t data, data_idx = 0, nb_words;
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	struct seco_mu_priv *mu_priv;
	struct seco_out_buffer_desc *b_desc;
	int err;

	do {
		mu_priv = dev_ctx->mu_priv;
		if (!mu_priv)
			break;

		/* Wait until a message is received on the MU. */
		err = wait_event_interruptible(dev_ctx->wq,
				dev_ctx->pending_hdr != 0);
		if (err)
			break;

		/* Check that the size passed as argument is larger than
		 * the one carried in the message.
		 */
		nb_words = MESSAGE_SIZE(dev_ctx->pending_hdr);
		if (nb_words * sizeof(uint32_t) > size)
			break;

		/* We may need to copy the output data to user before
		 * delivering the completion message.
		 */
		while (!list_empty(&dev_ctx->pending_out)) {
			b_desc = list_first_entry_or_null(
					&dev_ctx->pending_out,
					struct seco_out_buffer_desc,
					link);
			if (b_desc->out_usr_ptr && b_desc->out_ptr) {
				err = (int)copy_to_user(b_desc->out_usr_ptr,
							b_desc->out_ptr,
							b_desc->out_size);
			}
			__list_del_entry(&b_desc->link);
			kfree(b_desc);
			if (err)
				break;
		}

		err = (int)copy_to_user(buf, &dev_ctx->pending_hdr,
					sizeof(uint32_t));
		if (err)
			break;
		dev_ctx->pending_hdr = 0;

		data_idx = 1;

		/* No need to lock access to the MU here.
		 * We can come here only if an interrupt from MU was received
		 * and dispatched to the expected used.
		 * No other interrupt will be issued until the message is fully
		 * read. So there is no risk of 2 threads reading from the MU at
		 * the same time.
		 */
		while (data_idx < nb_words) {
			data = seco_mu_receive_data(mu_priv->mu_base,
					(uint8_t)(data_idx % MU_TR_COUNT));
			err = (int)copy_to_user(buf + data_idx*sizeof(uint32_t),
						&data,
						sizeof(uint32_t));
			if (err)
				break;
			data_idx++;
		}

		/* free memory allocated on the shared buffers. */
		dev_ctx->secure_mem.pos = 0;
		dev_ctx->non_secure_mem.pos = 0;

	} while (0);

	data_idx = data_idx * (uint32_t)sizeof(uint32_t);

	return (ssize_t)(data_idx);
}

static long seco_mu_ioctl_shared_mem_cfg_handler (
				struct seco_mu_device_ctx *dev_ctx,
				unsigned long arg)
{
	long err = -EINVAL;
	struct seco_mu_ioctl_shared_mem_cfg cfg;

	do {
		/* Check if not already configured. */
		if (dev_ctx->secure_mem.dma_addr != 0u)
			break;

		err = (long)copy_from_user(&cfg, (uint8_t *)arg,
			sizeof(struct seco_mu_ioctl_shared_mem_cfg));
		if (err)
			break;
		dev_ctx->secure_mem.dma_addr = (dma_addr_t)cfg.base_offset;
		dev_ctx->secure_mem.size = cfg.size;
		dev_ctx->secure_mem.pos = 0;
		dev_ctx->secure_mem.ptr = ioremap_nocache(
			(phys_addr_t)(SECURE_RAM_BASE_ADDRESS +
				(uint64_t)dev_ctx->secure_mem.dma_addr),
			dev_ctx->secure_mem.size);
	} while (0);

	return err;
}

static long seco_mu_ioctl_setup_iobuf_handler (
				struct seco_mu_device_ctx *dev_ctx,
				unsigned long arg)
{
	int err = -EINVAL;
	uint32_t pos;
	struct seco_mu_ioctl_setup_iobuf io;
	struct seco_shared_mem *shared_mem;
	struct seco_out_buffer_desc *out_buf_desc;

	do {
		err = (int)copy_from_user(&io,
			(uint8_t *)arg,
			sizeof(struct seco_mu_ioctl_setup_iobuf));
		if (err)
			break;

		if ((io.length == 0) || (io.user_buf == NULL)) {
			/*
			 * Accept NULL pointers since some buffers are optional
			 * in SECO commands. In this case we should return 0 as
			 * pointer to be embedded into the message.
			 * Skip all data copy part of code below.
			 */
			io.seco_addr = 0;
			break;
		}

		/* Select the shared memory to be used for this buffer. */
		if (io.flags & SECO_MU_IO_FLAGS_USE_SEC_MEM) {
			/* App requires to use secure memory for this buffer.*/
			shared_mem = &dev_ctx->secure_mem;
		} else {
			/* No specific requirement for this buffer. */
			shared_mem = &dev_ctx->non_secure_mem;
		}

		/* Check there is enough space in the shared memory. */
		if (io.length >= shared_mem->size - shared_mem->pos) {
			err = -ENOMEM;
			break;
		}

		/* Allocate space in shared memory. 8 bytes aligned. */
		pos = shared_mem->pos;
		shared_mem->pos += round_up(io.length, 8u);
		io.seco_addr = (uint64_t)shared_mem->dma_addr + pos;

		if ((io.flags & SECO_MU_IO_FLAGS_USE_SEC_MEM) &&
			!(io.flags & SECO_MU_IO_FLAGS_USE_SHORT_ADDR))
			/*Add base address to get full address.*/
			io.seco_addr += SECURE_RAM_BASE_ADDRESS_SCU;

		if (io.flags & SECO_MU_IO_FLAGS_IS_INPUT) {
			/*
			 * buffer is input:
			 * copy data from user space to this allocated buffer.
			 */
			err = (int)copy_from_user(shared_mem->ptr + pos,
							io.user_buf,
							io.length);
			if (err)
				break;
		} else {
			/*
			 * buffer is output:
			 * add an entry in the "pending buffers" list so data
			 * can be copied to user space when receiving SECO
			 * response.
			 */
			out_buf_desc = kmalloc(
					sizeof(struct seco_out_buffer_desc),
					GFP_KERNEL);
			if (!out_buf_desc) {
				err = -ENOMEM;
				break;
			}

			out_buf_desc->out_ptr = shared_mem->ptr + pos;
			out_buf_desc->out_usr_ptr = io.user_buf;
			out_buf_desc->out_size = io.length;
			list_add_tail(&out_buf_desc->link,
					&dev_ctx->pending_out);
			err = 0;
		}
	} while (0);

	if (err == 0) {
		/* Provide the seco address to user space only if success. */
		err = (int)copy_to_user((uint8_t *)arg, &io,
			sizeof(struct seco_mu_ioctl_setup_iobuf));
	}

	return (long)err;
}

static uint8_t get_did(void)
{
	sc_rm_did_t did;
	sc_ipc_t ipc;
	uint32_t mu_id;
	sc_err_t sciErr;

	sciErr = (sc_err_t)sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE)
		return DEFAULT_DID;

	sciErr = sc_ipc_open(&ipc, mu_id);
	if (sciErr != SC_ERR_NONE)
		return DEFAULT_DID;

	did = sc_rm_get_did(ipc);

	sc_ipc_close(ipc);

	return (uint8_t)did;
}

static long seco_mu_ioctl_get_mu_info_handler (
				struct seco_mu_device_ctx *dev_ctx,
				unsigned long arg)
{
	int err = -EINVAL;
	struct seco_mu_ioctl_get_mu_info info;

	info.seco_mu_idx = (uint8_t)dev_ctx->mu_priv->seco_mu_id;
	info.interrupt_idx = SECO_MU_INTERRUPT_INDEX;
	info.tz = SECO_DEFAULT_TZ;
	info.did = get_did();

	err = (int)copy_to_user((uint8_t *)arg, &info,
		sizeof(struct seco_mu_ioctl_get_mu_info));

	return (long)err;
}

static long seco_mu_ioctl_signed_msg_handler (
				struct seco_mu_device_ctx *dev_ctx,
				unsigned long arg)
{
	int err = -EINVAL;
	struct seco_mu_ioctl_signed_message msg;

	do {
		err = (int)copy_from_user(&msg,
			(uint8_t *)arg,
			sizeof(struct seco_mu_ioctl_signed_message));
		if (err)
			break;

		/*TODO:  Call SCU RPC. */

		msg.error_code = 0;

		err = (int)copy_to_user((uint8_t *)arg, &msg,
			sizeof(struct seco_mu_ioctl_signed_message));
	} while (0);

	return (long)err;
}



static long seco_mu_ioctl(struct file *fp,
				unsigned int cmd, unsigned long arg)
{
	struct seco_mu_device_ctx *dev_ctx = container_of(fp->private_data,
					struct seco_mu_device_ctx, miscdev);
	struct seco_mu_priv *mu_priv = dev_ctx->mu_priv;
	long err = -EINVAL;

	/* Prevent execution of a ioctl while a message is being sent. */
	mutex_lock(&(mu_priv->mu_regs_lock));
	switch (cmd) {
	case SECO_MU_IOCTL_ENABLE_CMD_RCV:
		if (!mu_priv->cmd_receiver_dev) {
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
	}

	mutex_unlock(&(mu_priv->mu_regs_lock));

	return err;
}


/*
 * Interrupts handling (ISR + work handler)
 */

/* Work handler to process arriving messages out of the ISR.
 * - Read the full message
 * - Store the message the receive queue of the MU that issued last command.
 * - exception is for incoming commands that are intended for storage
 * - wake up the associated blocking read.
 */
static void seco_mu_receive_work_handler(struct work_struct *work)
{
	uint32_t hdr;
	struct seco_mu_priv *mu_priv = container_of(work, struct seco_mu_priv,
							seco_mu_work);
	struct seco_mu_device_ctx *dev_ctx = NULL;

	if (mu_priv) {
		/* Read the header from RR[0] and extract message tag. */
		hdr = seco_mu_receive_data(mu_priv->mu_base, 0);

		/* Incoming command: wake up the receiver if any. */
		if (MESSAGE_TAG(hdr) == MESSAGING_TAG_COMMAND) {
			dev_ctx = mu_priv->cmd_receiver_dev;
		} else if (MESSAGE_TAG(hdr) == MESSAGING_TAG_RESPONSE) {
			/* This is a response. */
			dev_ctx = mu_priv->waiting_rsp_dev;
			mu_priv->waiting_rsp_dev = NULL;
			/*
			 * The response to the previous command is received.
			 * Allow following command to be sent on the MU.
			 */
			mutex_unlock(&(mu_priv->mu_cmd_lock));
		}

		if (dev_ctx) {
			/* Store the hdr for the read operation.*/
			dev_ctx->pending_hdr = hdr;
			wake_up_interruptible(&dev_ctx->wq);
		}
	}
}


/* Interrupt handler. */
static irqreturn_t seco_mu_isr(int irq, void *param)
{
	struct seco_mu_priv *priv = (struct seco_mu_priv *)param;

	/* GI 0 handling (loop to avoid race condition). */
	if (seco_mu_get_gip(priv->mu_base, SECO_MU_INTERRUPT_INDEX) != 0u) {
		seco_mu_clear_gip(priv->mu_base, SECO_MU_INTERRUPT_INDEX);
		(void)schedule_work(&priv->seco_mu_work);
	}

	return IRQ_HANDLED;
}

#define SECO_FW_VER_FEAT_MASK		(0x0000FFF0u)
#define SECO_FW_VER_FEAT_SHIFT		(0x04u)
#define SECO_FW_VER_FEAT_MIN_ALL_MU	(0x04u)

/*
 * Get SECO FW version and check if it supports receiving commands on all MUs
 * The version is retrieved through SCU since this is the only communication
 * channel to SECO always present.
 */
static int seco_check_all_mu_supported(struct platform_device *pdev)
{
	uint32_t seco_ver;
	sc_ipc_t ipc;
	uint32_t mu_id;
	sc_err_t sciErr;

	sciErr = (sc_err_t)sc_ipc_getMuID(&mu_id);

	if (sciErr != SC_ERR_NONE) {
		dev_info(&pdev->dev, "Cannot obtain SCU MU ID\n");
		return -EPROBE_DEFER;
	}

	sciErr = sc_ipc_open(&ipc, mu_id);

	if (sciErr != SC_ERR_NONE) {
		dev_info(&pdev->dev, "Cannot open MU channel to SCU\n");
		return -EPROBE_DEFER;
	}

	sc_seco_build_info(ipc, &seco_ver, NULL);

	sc_ipc_close(ipc);

	if (((seco_ver & SECO_FW_VER_FEAT_MASK) >> SECO_FW_VER_FEAT_SHIFT)
		< SECO_FW_VER_FEAT_MIN_ALL_MU) {
		dev_info(&pdev->dev, "current SECO FW do not support SHE\n");
		return -ENOTSUPP;
	}
	return 0;
}

/*
 * Driver setup
 */
static const struct file_operations seco_mu_fops = {
	.open		= seco_mu_open,
	.owner		= THIS_MODULE,
	.read		= seco_mu_read,
	.release	= seco_mu_close,
	.write		= seco_mu_write,
	.unlocked_ioctl = seco_mu_ioctl,
};


/* Driver probe.*/
#define SECO_DEV_NAME_MAX_LEN 16
static int seco_mu_probe(struct platform_device *pdev)
{
	struct device_node *np;
	int ret;
	int irq, err, i;
	struct seco_mu_priv *priv;
	struct seco_mu_device_ctx *dev_ctx;
	int max_nb_users = 0;
	char devname[SECO_DEV_NAME_MAX_LEN];

	priv = devm_kzalloc(&pdev->dev, sizeof(struct seco_mu_priv),
				GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/*
	 * Get the address of MU to be used for communication with the SCU
	 */
	np = pdev->dev.of_node;
	if (!np) {
		(void)pr_info("Cannot find MU User entry in device tree\n");
		return -ENOTSUPP;
	}

	ret = seco_check_all_mu_supported(pdev);
	if (ret)
		return ret;

	/* Initialize the mutex. */
	mutex_init(&priv->mu_regs_lock);
	mutex_init(&priv->mu_cmd_lock);

	priv->dev = &pdev->dev;

	priv->cmd_receiver_dev = NULL;
	priv->waiting_rsp_dev = NULL;

	/* Init work handler to process messages out of the IRQ handler.*/
	INIT_WORK(&priv->seco_mu_work, seco_mu_receive_work_handler);

	irq = of_irq_get(np, 0);
	if (irq > 0) {
		/* setup the IRQ. */
		err = devm_request_irq(&pdev->dev, irq, seco_mu_isr,
				  IRQF_EARLY_RESUME, "seco_mu_isr", priv);
		if (err) {
			pr_err("%s: request_irq %d failed: %d\n",
					__func__, irq, err);
			return err;
		}
		(void)irq_set_irq_wake(irq, 1);
	}

	err = of_property_read_u32(np, "fsl,seco_mu_id", &priv->seco_mu_id);
	if (err)
		priv->seco_mu_id = SECO_DEFAULT_MU_INDEX;

	err = of_property_read_u32(np, "fsl,seco_max_users", &max_nb_users);
	if (err)
		max_nb_users = 2; /* 2 users max by default. */

	/* initialize the MU. */
	priv->mu_base = of_iomap(np, 0);
	if (priv->mu_base == NULL)
		return -EIO;
	seco_mu_init(priv->mu_base);

	for (i = 0; i < max_nb_users; i++) {
		dev_ctx = devm_kzalloc(&pdev->dev,
				sizeof(struct seco_mu_device_ctx),
				GFP_KERNEL);

		dev_ctx->status = MU_FREE;
		dev_ctx->mu_priv = priv;
		/* Default value invalid for an header. */
		dev_ctx->pending_hdr = 0;
		init_waitqueue_head(&dev_ctx->wq);

		dev_ctx->secure_mem.ptr = NULL;
		dev_ctx->secure_mem.dma_addr = 0;
		dev_ctx->secure_mem.size = 0;
		dev_ctx->secure_mem.pos = 0;
		dev_ctx->non_secure_mem.ptr = NULL;
		dev_ctx->non_secure_mem.dma_addr = 0;
		dev_ctx->non_secure_mem.size = 0;
		dev_ctx->non_secure_mem.pos = 0;

		INIT_LIST_HEAD(&dev_ctx->pending_out);

		snprintf(devname,
			SECO_DEV_NAME_MAX_LEN,
			"seco_mu%d_ch%d", priv->seco_mu_id, i);
		dev_ctx->miscdev.name = devname;
		dev_ctx->miscdev.minor	= MISC_DYNAMIC_MINOR,
		dev_ctx->miscdev.fops = &seco_mu_fops,
		dev_ctx->miscdev.parent = &pdev->dev,
		ret = misc_register(&dev_ctx->miscdev);
		if (ret) {
			dev_err(&pdev->dev,
			 "failed to register misc device %d\n",
			ret);
			iounmap(priv->mu_base);
			return ret;
		}
	}

	return 0;
}

static int seco_mu_remove(struct platform_device *pdev)
{
	struct seco_mu_priv *priv = platform_get_drvdata(pdev);

	iounmap(priv->mu_base);

	return 0;
}

static const struct of_device_id seco_mu_match[] = {
	{
		.compatible = "fsl,imx8-seco-mu",
	},
	{},
};
MODULE_DEVICE_TABLE(of, seco_mu_match);

static struct platform_driver seco_mu_driver = {
	.driver = {
		.name = "seco_mu",
		.of_match_table = seco_mu_match,
	},
	.probe       = seco_mu_probe,
	.remove      = seco_mu_remove,
};

module_platform_driver(seco_mu_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IMX Seco MU");
MODULE_AUTHOR("NXP");
