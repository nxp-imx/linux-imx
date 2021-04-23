/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 NXP
 */

#ifndef __S400_API_H
#define __S400_API_H

#include <linux/completion.h>
#include <linux/mailbox_client.h>

#define S400_READ_FUSE_REQ		0x97
#define OTP_UNIQ_ID			0x01
#define S400_SUCCESS_IND		0xD6

#define S400_MSG_DATA_NUM		10

struct s400_api_msg {
	u32 header; /* u8 Tag; u8 Command; u8 Size; u8 Ver; */
	u32 data[S400_MSG_DATA_NUM];
};

struct imx_s400_api {
	struct mbox_client tx_cl, rx_cl;
	struct mbox_chan *tx_chan, *rx_chan;
	struct s400_api_msg tx_msg, rx_msg;
	struct completion done;
	spinlock_t lock;
};

int get_imx_s400_api(struct imx_s400_api **export);
int imx_s400_api_call(struct imx_s400_api *s400_api, void *value);

#endif
