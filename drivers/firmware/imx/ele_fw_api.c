// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2023-2024 NXP
 */

#include <linux/dma-mapping.h>

#include "ele_common.h"
#include "ele_fw_api.h"

struct ele_rng_msg {
	u32 header; /* u8 Tag; u8 Command; u8 Size; u8 Ver; */
	u16 rsv;
	u16 flags;
	u32 data[2];
};

int ele_init_fw(struct device *dev)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	unsigned int status;
	int ret;

	mutex_lock(&priv->mu_cmd_lock);
	ret = imx_se_alloc_tx_rx_buf(priv);
	if (ret)
		goto exit;

	ret = plat_fill_cmd_msg_hdr(priv,
				    (struct mu_hdr *)&priv->tx_msg->header,
				    ELE_INIT_FW_REQ, ELE_INIT_FW_REQ_SZ,
				    false);
	if (ret)
		goto exit;

	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_INIT_FW_REQ,
				ELE_INIT_FW_RSP_SZ,
				false);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_INIT_FW_REQ, status);
		ret = -1;
	}

exit:
	imx_se_free_tx_rx_buf(priv);
	mutex_unlock(&priv->mu_cmd_lock);

	return ret;
}

/*
 * ele_get_random() - prepare and send the command to proceed
 *                    with a random number generation operation
 *
 * returns:  size of the rondom number generated
 */
int ele_get_random(struct device *dev,
		   void *data, size_t len)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	struct ele_rng_msg *tx_msg;
	unsigned int status;
	dma_addr_t dst_dma;
	u8 *buf;
	int ret;

	/* As per RBG3(RS) construction mentioned in NIST SP800-90C,
	 * CTR_DRBG generates 128(full entropy) bits after reseeding
	 * the CTR_DRBG with 256 bits of entropy. so splitting the
	 * user rng request in multiple of 128 bits & enforce reseed
	 * for every iteration.
	 */
	len = ELE_RNG_MAX_SIZE;
	buf = dmam_alloc_coherent(priv->dev, len, &dst_dma, GFP_KERNEL);
	if (!buf) {
		dev_err(priv->dev, "Failed to map destination buffer memory\n");
		return -ENOMEM;
	}

	mutex_lock(&priv->mu_cmd_lock);
	ret = imx_se_alloc_tx_rx_buf(priv);
	if (ret) {
		ret = -ENOMEM;
		goto exit;
	}

	tx_msg = (struct ele_rng_msg *)priv->tx_msg;
	ret = plat_fill_cmd_msg_hdr(priv,
				    (struct mu_hdr *)&tx_msg->header,
				    ELE_GET_RANDOM_REQ, ELE_GET_RANDOM_REQ_SZ,
				    false);
	if (ret)
		goto exit;

	/* bit 1(blocking reseed): wait for trng entropy,
	 * then reseed rng context.
	 */
	if (get_se_soc_id(dev) != SOC_ID_OF_IMX95)
		tx_msg->flags = BIT(1);

	tx_msg->data[0] = dst_dma;
	tx_msg->data[1] = len;
	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_GET_RANDOM_REQ,
				ELE_GET_RANDOM_RSP_SZ,
				false);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_GET_RANDOM_REQ, status);
		ret = -1;
	} else {
		memcpy(data, buf, len);
		ret = len;
	}

exit:
	imx_se_free_tx_rx_buf(priv);
	mutex_unlock(&priv->mu_cmd_lock);
	dmam_free_coherent(priv->dev, len, buf, dst_dma);

	return ret;
}
