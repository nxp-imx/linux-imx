/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 NXP
 */

#ifndef SENTNL_MU_H
#define SENTNL_MU_H

#include <linux/miscdevice.h>
#include <linux/semaphore.h>

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
#define SECO_MU_IO_FLAGS_USE_SEC_MEM	(0x02u)
#define SECO_MU_IO_FLAGS_USE_SHORT_ADDR	(0x04u)

struct sentnl_obuf_desc {
	u8 *out_ptr;
	u8 *out_usr_ptr;
	u32 out_size;
	struct list_head link;
};

/* Status of a char device */
enum mu_device_status_t {
	MU_FREE,
	MU_OPENED
};

struct sentnl_shared_mem {
	dma_addr_t dma_addr;
	u32 size;
	u32 pos;
	u8 *ptr;
};

/* Private struct for each char device instance. */
struct sentnl_mu_device_ctx {
	struct device *dev;
	struct sentnl_mu_priv *priv;
	struct miscdevice miscdev;

	enum mu_device_status_t status;
	wait_queue_head_t wq;
	struct semaphore fops_lock;

	u32 pending_hdr;
	struct list_head pending_out;

	struct sentnl_shared_mem secure_mem;
	struct sentnl_shared_mem non_secure_mem;

	u32 temp_cmd[MAX_MESSAGE_SIZE];
	u32 temp_resp[MAX_RECV_SIZE];
	u32 temp_resp_size;
	struct notifier_block sentnl_notify;
};

/* Header of the messages exchange with the SENTINEL */
struct mu_hdr {
	u8 ver;
	u8 size;
	u8 command;
	u8 tag;
}  __packed;

struct sentnl_api_msg {
	u32 header; /* u8 Tag; u8 Command; u8 Size; u8 Ver; */
	u32 data[SENTNL_MSG_DATA_NUM];
};

struct sentnl_mu_priv {
	struct sentnl_mu_device_ctx *cmd_receiver_dev;
	struct sentnl_mu_device_ctx *waiting_rsp_dev;
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
	u32 sentnl_mu_id;
	u8 cmd_tag;
	u8 rsp_tag;

	struct mbox_client sentnl_mb_cl;
	struct mbox_chan *tx_chan, *rx_chan;
	struct sentnl_api_msg tx_msg, rx_msg;
	struct completion done;
	spinlock_t lock;
};

int get_sentnl_mu_priv(struct sentnl_mu_priv **export);
#endif
