// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021-2023 NXP
 */

#include <linux/types.h>
#include <linux/completion.h>

#include <linux/firmware/imx/v2x_base_msg.h>

#include "ele_common.h"

/*
 * v2x_start_rng() - prepare and send the command to start
 *                   initialization of the ELE RNG context
 *
 * returns:  0 on success.
 */
int v2x_start_rng(struct device *dev)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	int ret;
	unsigned int status;

	mutex_lock(&priv->mu_cmd_lock);
	ret = imx_se_alloc_tx_rx_buf(priv);
	if (ret)
		goto exit;

	ret = plat_fill_cmd_msg_hdr(priv,
				    (struct mu_hdr *)&priv->tx_msg->header,
				    V2X_START_RNG_REQ,
				    V2X_START_RNG_REQ_MSG_SZ,
				    true);
	if (ret)
		goto exit;

	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				V2X_START_RNG_REQ,
				V2X_START_RNG_RSP_MSG_SZ,
				true);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		/* Initialization in progress for:
		 * P-TRNG at bit 0
		 * S-TRNG at bit 1
		 * Any of the bit is set, it in progress.
		 */
		if (priv->rx_msg->data[1] & 0x3)
			goto exit;

		dev_err(dev, "Command Id[%d], Response Failure = 0x%x",
			V2X_START_RNG_REQ, status);
		ret = -1;
	}

exit:
	imx_se_free_tx_rx_buf(priv);
	mutex_unlock(&priv->mu_cmd_lock);

	return ret;
}
