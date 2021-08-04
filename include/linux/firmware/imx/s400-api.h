/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 NXP
 */

#ifndef __S400_API_H
#define __S400_API_H

#include <linux/completion.h>
#include <linux/mailbox_client.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>


#define MAX_RECV_SIZE 31
#define MAX_RECV_SIZE_BYTES (MAX_RECV_SIZE * sizeof(u32))
#define MAX_MESSAGE_SIZE 31
#define MAX_MESSAGE_SIZE_BYTES (MAX_MESSAGE_SIZE * sizeof(u32))

#define S400_OEM_CNTN_AUTH_REQ		0x87
#define S400_VERIFY_IMAGE_REQ		0x88
#define S400_RELEASE_CONTAINER_REQ	0x89
#define S400_READ_FUSE_REQ		0x97
#define OTP_UNIQ_ID			0x01

#define S400_VERSION			0x6
#define S400_SUCCESS_IND		0xD6
#define S400_FAILURE_IND		0x29

#define S400_MSG_DATA_NUM		10

#define S400_OEM_CNTN_AUTH_REQ_SIZE	3
#define S400_VERIFY_IMAGE_REQ_SIZE	2
#define S400_RELEASE_CONTAINER_REQ_SIZE	1

struct s400_api_msg {
	u32 header; /* u8 Tag; u8 Command; u8 Size; u8 Ver; */
	u32 data[S400_MSG_DATA_NUM];
};

/* Status of a char device */
enum mu_device_status_t {
	MU_FREE,
	MU_OPENED
};

struct s4_shared_mem {
	dma_addr_t dma_addr;
	u32 size;
	u32 pos;
	u8 *ptr;
};

/* Private struct for each char device instance. */
struct s4_mu_device_ctx {
	struct device *dev;
	struct imx_s400_api *s400_muap_priv;
	struct miscdevice miscdev;

	enum mu_device_status_t status;
	wait_queue_head_t wq;
	struct semaphore fops_lock;

	u32 pending_hdr;
	struct list_head pending_out;

	struct s4_shared_mem secure_mem;
	struct s4_shared_mem non_secure_mem;

	u32 temp_cmd[MAX_MESSAGE_SIZE];
	u32 temp_resp[MAX_RECV_SIZE];
	u32 temp_resp_size;
	struct notifier_block s4_notify;
};

struct imx_s400_api {
	struct s4_mu_device_ctx *cmd_receiver_dev;
	struct s4_mu_device_ctx *waiting_rsp_dev;
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
	u32 s4_muap_id;
	u8 cmd_tag;
	u8 rsp_tag;

	struct mbox_client tx_cl, rx_cl;
	struct mbox_chan *tx_chan, *rx_chan;
	struct s400_api_msg tx_msg, rx_msg;
	struct completion done;
	spinlock_t lock;
};

int get_imx_s400_api(struct imx_s400_api **export);
int imx_s400_api_call(struct imx_s400_api *s400_api, void *value);

#endif
