// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2023 NXP
 */

#include <linux/dma-mapping.h>
#include <linux/firmware/imx/ele_fw_api.h>

#include "ele_mu.h"

/* Fill a command message header with a given command ID and length in bytes. */
static int plat_fill_cmd_msg_hdr(struct mu_hdr *hdr, uint8_t cmd, uint32_t len)
{
	struct ele_mu_priv *priv = NULL;
	int err = 0;

	err = get_ele_mu_priv(&priv);
	if (err) {
		pr_err("Error: iMX EdgeLock Enclave MU is not probed successfully.\n");
		return err;
	}
	hdr->tag = priv->cmd_tag;
	hdr->ver = MESSAGING_VERSION_7;
	hdr->command = cmd;
	hdr->size = (uint8_t)(len / sizeof(uint32_t));

	return err;
}

/*
 * ele_get_random() - prepare and send the command to proceed
 *                    with a random number generation operation
 *
 * returns:  size of the rondom number generated
 */
int ele_get_random(struct hwrng *rng, void *data, size_t len, bool wait)
{
	struct ele_mu_priv *priv;
	unsigned int tag, command, size, ver, status;
	dma_addr_t dst_dma;
	u8 *buf;
	int ret;

	/* access ele_mu_priv data structure pointer*/
	ret = get_ele_mu_priv(&priv);
	if (ret)
		return ret;

	buf = dmam_alloc_coherent(priv->dev, len, &dst_dma, GFP_KERNEL);
	if (!buf) {
		dev_err(priv->dev, "Failed to map destination buffer memory\n");
		return -ENOMEM;
	}

	ret = plat_fill_cmd_msg_hdr((struct mu_hdr *)&priv->tx_msg.header, ELE_GET_RANDOM_REQ, 16);
	if (ret)
		goto exit;

	priv->tx_msg.data[0] = 0x0;
	priv->tx_msg.data[1] = dst_dma;
	priv->tx_msg.data[2] = len;
	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	tag = MSG_TAG(priv->rx_msg.header);
	command = MSG_COMMAND(priv->rx_msg.header);
	size = MSG_SIZE(priv->rx_msg.header);
	ver = MSG_VER(priv->rx_msg.header);
	status = RES_STATUS(priv->rx_msg.data[0]);
	if (tag == 0xe1 && command == ELE_GET_RANDOM_REQ && size == 0x02 &&
	    ver == 0x07 && status == 0xd6) {
		memcpy(data, buf, len);
		ret = len;
	} else
		ret = -EINVAL;

exit:
	dmam_free_coherent(priv->dev, len, buf, dst_dma);
	return ret;
}

int ele_init_fw(void)
{
	struct ele_mu_priv *priv;
	int ret;
	unsigned int tag, command, size, ver, status;

	ret = get_ele_mu_priv(&priv);
	if (ret)
		return ret;

	ret = plat_fill_cmd_msg_hdr((struct mu_hdr *)&priv->tx_msg.header, ELE_INIT_FW_REQ, 4);
	if (ret)
		return ret;

	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		return ret;

	tag = MSG_TAG(priv->rx_msg.header);
	command = MSG_COMMAND(priv->rx_msg.header);
	size = MSG_SIZE(priv->rx_msg.header);
	ver = MSG_VER(priv->rx_msg.header);
	status = RES_STATUS(priv->rx_msg.data[0]);

	if (tag == 0xe1 && command == ELE_INIT_FW_REQ && size == 0x02 &&
	    ver == MESSAGING_VERSION_7 && status == 0xd6)
		return 0;

	return -EINVAL;
}
