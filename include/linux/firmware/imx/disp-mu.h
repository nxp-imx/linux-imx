/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 NXP
 *
 * Header file for the Display-Mix MU implementation.
 */

#ifndef _SC_DISPMU_H
#define _SC_DISPMU_H

#include <linux/device.h>
#include <linux/types.h>
#include <linux/mailbox_client.h>

#define DISP_MU_CHAN_NUM	2
#define MU_CHAN_TX		0
#define MU_CHAN_RX		1
#define MBOX_TX_NAME		"tx"
#define MBOX_RX_NAME		"rx"
#define MAX_TX_TIMEOUT		(msecs_to_jiffies(1000))
#define MAX_RX_TIMEOUT		(msecs_to_jiffies(3000))

struct disp_mu_client {
	struct disp_mu_dev *mu;

	void (*rx_callback)(struct disp_mu_client *cl, void *msg);
};

struct disp_mu_chan {
	struct disp_mu_dev *mu_dev;

	struct mbox_client cl;
	struct mbox_chan *ch;
	int idx;
};

struct disp_mu_dev {
	struct disp_mu_client *cl;

	struct disp_mu_chan chans[DISP_MU_CHAN_NUM];
	struct mutex lock; /* lock for multiple clients */
	struct completion done;

	bool need_reply;
	u32 reply[4];
};

struct disp_mu_ipc {
	struct disp_mu_dev mu_dev;
	struct device *dev;
};

int imx_dispmu_client_register(struct disp_mu_client *cl);
int imx_dispmu_client_unregister(struct disp_mu_client *cl);
int imx_dispmu_send_msg(struct disp_mu_client *cl, void *msg, void **reply);

#endif /* _SC_DISPMU_H */
