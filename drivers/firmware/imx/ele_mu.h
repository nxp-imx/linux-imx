/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021-2023 NXP
 */

#ifndef ELE_MU_H
#define ELE_MU_H

#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/mailbox_client.h>

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

#define MAX_RECV_SIZE 31
#define MAX_RECV_SIZE_BYTES (MAX_RECV_SIZE * sizeof(u32))
#define MAX_MESSAGE_SIZE 31
#define MAX_MESSAGE_SIZE_BYTES (MAX_MESSAGE_SIZE * sizeof(u32))
#define ELE_SUCCESS_IND			0xD6
#define ELE_FAILURE_IND			0x29

#define ELE_MSG_DATA_NUM		10

#define MSG_TAG(x)			(((x) & 0xff000000) >> 24)
#define MSG_COMMAND(x)			(((x) & 0x00ff0000) >> 16)
#define MSG_SIZE(x)			(((x) & 0x0000ff00) >> 8)
#define MSG_VER(x)			((x) & 0x000000ff)
#define RES_STATUS(x)			((x) & 0x000000ff)
#define MAX_DATA_SIZE_PER_USER		(65 * 1024)
#define S4_DEFAULT_MUAP_INDEX		(2)
#define S4_MUAP_DEFAULT_MAX_USERS	(4)

#define DEFAULT_MESSAGING_TAG_COMMAND           (0x17u)
#define DEFAULT_MESSAGING_TAG_RESPONSE          (0xe1u)

#define SECO_MU_IO_FLAGS_IS_INPUT	(0x01u)
#define SECO_MU_IO_FLAGS_IS_OUTPUT	(0x00u)
#define SECO_MU_IO_FLAGS_IS_IN_OUT	(0X08u)
#define SECO_MU_IO_FLAGS_USE_SEC_MEM	(0x02u)
#define SECO_MU_IO_FLAGS_USE_SHORT_ADDR	(0x04u)

struct ele_imem_buf {
	u8 *buf;
	phys_addr_t phyaddr;
	u32 size;
};

struct ele_buf_desc {
	u8 *shared_buf_ptr;
	u8 *usr_buf_ptr;
	u32 size;
	struct list_head link;
};

/* Status of a char device */
enum mu_device_status_t {
	MU_FREE,
	MU_OPENED
};

struct ele_shared_mem {
	dma_addr_t dma_addr;
	u32 size;
	u32 pos;
	u8 *ptr;
};

/* Private struct for each char device instance. */
struct ele_mu_device_ctx {
	struct device *dev;
	struct ele_mu_priv *priv;
	struct miscdevice miscdev;

	enum mu_device_status_t status;
	wait_queue_head_t wq;
	struct semaphore fops_lock;

	u32 pending_hdr;
	struct list_head pending_in;
	struct list_head pending_out;

	struct ele_shared_mem secure_mem;
	struct ele_shared_mem non_secure_mem;

	u32 temp_cmd[MAX_MESSAGE_SIZE];
	u32 temp_resp[MAX_RECV_SIZE];
	u32 temp_resp_size;
	struct notifier_block ele_notify;
};

/* Header of the messages exchange with the EdgeLock Enclave */
struct mu_hdr {
	u8 ver;
	u8 size;
	u8 command;
	u8 tag;
}  __packed;

struct ele_api_msg {
	u32 header; /* u8 Tag; u8 Command; u8 Size; u8 Ver; */
	u32 data[ELE_MSG_DATA_NUM];
};

struct ele_mu_priv {
	struct ele_mu_device_ctx *cmd_receiver_dev;
	struct ele_mu_device_ctx *waiting_rsp_dev;
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
	u32 ele_mu_did;
	u32 ele_mu_id;
	u8 cmd_tag;
	u8 rsp_tag;

	struct mbox_client ele_mb_cl;
	struct mbox_chan *tx_chan, *rx_chan;
	struct ele_api_msg tx_msg, rx_msg;
	struct completion done;
	spinlock_t lock;
	/* Flag to retain the state of initialization done at
	 * the time of ele-mu probe.
	 */
	int flags;
	int max_dev_ctx;
	struct ele_mu_device_ctx **ctxs;
	struct ele_imem_buf imem;
};

int get_ele_mu_priv(struct ele_mu_priv **export);

int imx_ele_msg_send_rcv(struct ele_mu_priv *priv);
#ifdef CONFIG_IMX_ELE_TRNG
int ele_trng_init(struct device *dev);
#else
static inline int ele_trng_init(struct device *dev)
{
	return 0;
}
#endif

#endif
