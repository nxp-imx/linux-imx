// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 * Author: Pankaj <pankaj.gupta@nxp.com>
	   Alice Guo <alice.guo@nxp.com>
 */

#include <linux/types.h>
#include <linux/completion.h>
#include <linux/mailbox_client.h>

#include <linux/firmware/imx/sentnl_base_msg.h>
#include <linux/firmware/imx/sentnl_mu_ioctl.h>

#include "sentnl_mu.h"

/* Fill a command message header with a given command ID and length in bytes. */
static int plat_fill_cmd_msg_hdr(struct mu_hdr *hdr, uint8_t cmd, uint32_t len)
{
	struct sentnl_mu_priv *priv = NULL;
	int err = 0;

	err = get_sentnl_mu_priv(&priv);
	if (err) {
		pr_err("Error: iMX Sentinel MU is not probed successfully.\n");
		return err;
	}
	hdr->tag = priv->cmd_tag;
	hdr->ver = MESSAGING_VERSION_6;
	hdr->command = cmd;
	hdr->size = (uint8_t)(len / sizeof(uint32_t));

	return err;
}

static int imx_sentnl_msg_send_rcv(struct sentnl_mu_priv *priv)
{
	unsigned int wait;
	int err = 0;

	mutex_lock(&priv->mu_cmd_lock);
	mutex_lock(&priv->mu_lock);

	err = mbox_send_message(priv->tx_chan, &priv->tx_msg);
	if (err < 0) {
		pr_err("Error: mbox_send_message failure.\n");
		mutex_unlock(&priv->mu_lock);
		return err;
	}
	mutex_unlock(&priv->mu_lock);

	wait = msecs_to_jiffies(1000);
	if (!wait_for_completion_timeout(&priv->done, wait)) {
		mutex_unlock(&priv->mu_cmd_lock);
		pr_err("Error: wait_for_completion timed out.\n");
		return -ETIMEDOUT;
	}

	/* As part of func sentnl_mu_rx_callback() execution,
	 * response will copied to sentnl_msg->rsp_msg.
	 *
	 * Lock: (mutex_unlock(&sentnl_mu_priv->mu_cmd_lock),
	 * will be unlocked if it is a response.
	 */
	return err;
}

static int read_otp_uniq_id(struct sentnl_mu_priv *priv, u32 *value)
{
	unsigned int tag, command, size, ver, status;

	tag = MSG_TAG(priv->rx_msg.header);
	command = MSG_COMMAND(priv->rx_msg.header);
	size = MSG_SIZE(priv->rx_msg.header);
	ver = MSG_VER(priv->rx_msg.header);
	status = RES_STATUS(priv->rx_msg.data[0]);

	if (tag == 0xe1 && command == SENTNL_READ_FUSE_REQ &&
	    size == 0x07 && ver == SENTNL_VERSION && status == SENTNL_SUCCESS_IND) {
		value[0] = priv->rx_msg.data[1];
		value[1] = priv->rx_msg.data[2];
		value[2] = priv->rx_msg.data[3];
		value[3] = priv->rx_msg.data[4];
		return 0;
	}

	return -EINVAL;
}

static int read_fuse_word(struct sentnl_mu_priv *priv, u32 *value)
{
	unsigned int tag, command, size, ver, status;

	tag = MSG_TAG(priv->rx_msg.header);
	command = MSG_COMMAND(priv->rx_msg.header);
	size = MSG_SIZE(priv->rx_msg.header);
	ver = MSG_VER(priv->rx_msg.header);
	status = RES_STATUS(priv->rx_msg.data[0]);

	if (tag == 0xe1 && command == SENTNL_READ_FUSE_REQ &&
	    size == 0x03 && ver == 0x06 && status == SENTNL_SUCCESS_IND) {
		value[0] = priv->rx_msg.data[1];
		return 0;
	}

	return -EINVAL;
}

int read_common_fuse(uint16_t fuse_id, u32 *value)
{
	struct sentnl_mu_priv *priv = NULL;
	int err = 0;

	err = get_sentnl_mu_priv(&priv);
	if (err) {
		pr_err("Error: iMX Sentinel MU is not probed successfully.\n");
		return err;
	}
	err = plat_fill_cmd_msg_hdr((struct mu_hdr *)&priv->tx_msg.header, SENTNL_READ_FUSE_REQ, 8);
	if (err) {
		pr_err("Error: plat_fill_cmd_msg_hdr failed.\n");
		return err;
	}

	priv->tx_msg.data[0] = fuse_id;
	err = imx_sentnl_msg_send_rcv(priv);
	if (err < 0)
		return err;

	switch (fuse_id) {
	case OTP_UNIQ_ID:
		err = read_otp_uniq_id(priv, value);
		break;
	default:
		err = read_fuse_word(priv, value);
		break;
	}

	return err;
}
EXPORT_SYMBOL_GPL(read_common_fuse);
