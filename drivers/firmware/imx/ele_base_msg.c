// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021-2024 NXP
 */

#include <linux/types.h>
#include <linux/completion.h>

#include <linux/firmware/imx/ele_base_msg.h>
#include <linux/firmware/imx/ele_mu_ioctl.h>

#include "ele_common.h"

int ele_get_info(struct device *dev, phys_addr_t addr, u32 data_size)
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
				    ELE_GET_INFO_REQ,
				    ELE_GET_INFO_REQ_MSG_SZ,
				    true);
	if (ret)
		goto exit;

	priv->tx_msg->data[0] = upper_32_bits(addr);
	priv->tx_msg->data[1] = lower_32_bits(addr);
	priv->tx_msg->data[2] = data_size;
	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_GET_INFO_REQ,
				ELE_GET_INFO_RSP_MSG_SZ,
				true);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_GET_INFO_REQ, status);
		ret = -1;
	}

exit:
	imx_se_free_tx_rx_buf(priv);
	mutex_unlock(&priv->mu_cmd_lock);

	return ret;
}

int ele_get_v2x_fw_state(struct device *dev, uint32_t *state)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	int ret;
	unsigned int status;
	struct mu_hdr *hdr;

	mutex_lock(&priv->mu_cmd_lock);
	ret = imx_se_alloc_tx_rx_buf(priv);
	if (ret)
		goto exit;

	hdr = (struct mu_hdr *)&priv->tx_msg->header;

	ret = plat_fill_cmd_msg_hdr(priv,
				    hdr,
				    ELE_GET_STATE, ELE_GET_STATE_REQ_SZ,
				    true);
	if (ret) {
		pr_err("Error: plat_fill_cmd_msg_hdr failed.\n");
		goto exit;
	}

	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_GET_STATE,
				ELE_GET_STATE_RSP_SZ,
				true);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_GET_STATE, status);
		ret = -1;
	} else {
		*state = 0xFF & priv->rx_msg->data[1];
	}
exit:
	imx_se_free_tx_rx_buf(priv);
	mutex_unlock(&priv->mu_cmd_lock);
	return ret;
}

int ele_write_fuse(struct device *dev, uint16_t fuse_id, u32 value, bool lock)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	unsigned int status, ind;
	int ret;

	mutex_lock(&priv->mu_cmd_lock);
	ret = imx_se_alloc_tx_rx_buf(priv);
	if (ret)
		goto exit;

	ret = plat_fill_cmd_msg_hdr(priv,
				    (struct mu_hdr *)&priv->tx_msg->header,
				    ELE_WRITE_FUSE, ELE_WRITE_FUSE_REQ_MSG_SZ,
				    true);
	if (ret) {
		pr_err("Error: plat_fill_cmd_msg_hdr failed.\n");
		goto exit;
	}

	priv->tx_msg->data[0] = (32 << 16) | (fuse_id << 5);
	if (lock)
		priv->tx_msg->data[0] |= BIT(31);
	priv->tx_msg->data[1] = value;

	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_WRITE_FUSE,
				ELE_WRITE_FUSE_RSP_MSG_SZ,
				true);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	ind = RES_IND(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(dev, "Command Id[%d], Status=0x%x, Indicator=0x%x",
			ELE_WRITE_FUSE, status, ind);
		ret = -1;
	}

exit:
	imx_se_free_tx_rx_buf(priv);
	mutex_unlock(&priv->mu_cmd_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(ele_write_fuse);

int ele_ping(struct device *dev)
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
				    ELE_PING_REQ, ELE_PING_REQ_SZ,
				    true);
	if (ret) {
		pr_err("Error: plat_fill_cmd_msg_hdr failed.\n");
		goto exit;
	}

	ret = imx_ele_msg_send_rcv(priv);
	if (ret)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_PING_REQ,
				ELE_PING_RSP_SZ,
				true);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_PING_REQ, status);
		ret = -1;
	}
exit:
	imx_se_free_tx_rx_buf(priv);

	mutex_unlock(&priv->mu_cmd_lock);
	return ret;
}

/*
 * ele_get_trng_state() - prepare and send the command to read
 *                        crypto lib and TRNG state
 * TRNG state
 *  0x1		TRNG is in program mode
 *  0x2		TRNG is still generating entropy
 *  0x3		TRNG entropy is valid and ready to be read
 *  0x4		TRNG encounter an error while generating entropy
 *
 * CSAL state
 *  0x0		Crypto Lib random context initialization is not done yet
 *  0x1		Crypto Lib random context initialization is on-going
 *  0x2		Crypto Lib random context initialization succeed
 *  0x3		Crypto Lib random context initialization failed
 *
 * returns: csal and trng state.
 *
 */
int ele_get_trng_state(struct device *dev)
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
				    ELE_GET_TRNG_STATE_REQ,
				    ELE_GET_TRNG_STATE_REQ_MSG_SZ,
				    true);
	if (ret) {
		pr_err("Error: plat_fill_cmd_msg_hdr failed.\n");
		goto exit;
	}

	ret = imx_ele_msg_send_rcv(priv);
	if (ret)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_GET_TRNG_STATE_REQ,
				ELE_GET_TRNG_STATE_RSP_MSG_SZ,
				true);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_GET_TRNG_STATE_REQ, status);
		ret = -1;
	} else
		ret = (priv->rx_msg->data[1] & CSAL_TRNG_STATE_MASK);

exit:
	imx_se_free_tx_rx_buf(priv);

	mutex_unlock(&priv->mu_cmd_lock);
	return ret;
}

/*
 * ele_start_rng() - prepare and send the command to start
 *                   initialization of the ELE RNG context
 *
 * returns:  0 on success.
 */
int ele_start_rng(struct device *dev)
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
				    ELE_START_RNG_REQ,
				    ELE_START_RNG_REQ_MSG_SZ,
				    true);
	if (ret)
		goto exit;

	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_START_RNG_REQ,
				ELE_START_RNG_RSP_MSG_SZ,
				true);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_START_RNG_REQ, status);
		ret = -1;
	}

exit:
	imx_se_free_tx_rx_buf(priv);

	mutex_unlock(&priv->mu_cmd_lock);
	return ret;
}

int ele_service_swap(struct device *dev,
		     phys_addr_t addr,
		     u32 addr_size, u16 flag)
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
				    ELE_SERVICE_SWAP_REQ,
				    ELE_SERVICE_SWAP_REQ_MSG_SZ,
				    true);
	if (ret)
		goto exit;

	priv->tx_msg->data[0] = flag;
	priv->tx_msg->data[1] = addr_size;
	priv->tx_msg->data[2] = ELE_NONE_VAL;
	priv->tx_msg->data[3] = lower_32_bits(addr);
	priv->tx_msg->data[4] = plat_add_msg_crc((uint32_t *)&priv->tx_msg[0],
						 ELE_SERVICE_SWAP_REQ_MSG_SZ);
	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_SERVICE_SWAP_REQ,
				ELE_SERVICE_SWAP_RSP_MSG_SZ,
				true);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_SERVICE_SWAP_REQ, status);
		ret = -1;
	} else {
		if (flag == ELE_IMEM_EXPORT)
			ret = priv->rx_msg->data[1];
		else
			ret = 0;
	}
exit:
	imx_se_free_tx_rx_buf(priv);
	mutex_unlock(&priv->mu_cmd_lock);

	return ret;
}

int ele_fw_authenticate(struct device *dev, phys_addr_t addr)
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
				    ELE_FW_AUTH_REQ,
				    ELE_FW_AUTH_REQ_SZ,
				    true);
	if (ret)
		goto exit;

	priv->tx_msg->data[0] = addr;
	priv->tx_msg->data[1] = 0x0;
	priv->tx_msg->data[2] = addr;

	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_FW_AUTH_REQ,
				ELE_FW_AUTH_RSP_MSG_SZ,
				true);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_FW_AUTH_REQ, status);
		ret = -1;
	}
exit:
	imx_se_free_tx_rx_buf(priv);
	mutex_unlock(&priv->mu_cmd_lock);

	return ret;
}

static int read_otp_uniq_id(struct ele_mu_priv *priv, u32 *value)
{
	int ret;
	unsigned int status;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_READ_FUSE_REQ,
				ELE_READ_FUSE_OTP_UNQ_ID_RSP_MSG_SZ,
				true);
	if (ret)
		return ret;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(priv->dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_READ_FUSE_REQ, status);
		ret = -1;
	} else {
		value[0] = priv->rx_msg->data[1];
		value[1] = priv->rx_msg->data[2];
		value[2] = priv->rx_msg->data[3];
		value[3] = priv->rx_msg->data[4];

		ret = 0;
	}

	return ret;
}

static int read_fuse_word(struct ele_mu_priv *priv, u32 *value)
{
	int ret;
	unsigned int status;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				ELE_READ_FUSE_REQ,
				ELE_READ_FUSE_RSP_MSG_SZ,
				true);
	if (ret)
		return ret;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(priv->dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_READ_FUSE_REQ, status);
		ret = -1;
	} else {
		value[0] = priv->rx_msg->data[1];

		ret = 0;
	}

	return ret;
}

/**
 * read_common_fuse() - Brief description of function.
 * @struct device *dev: Device to send the request to read fuses.
 * @uint16_t fuse_id: Fuse identifier to read.
 * @u32 *value: unsigned integer array to store the fused-values.
 *
 * Secure-enclave like EdgeLock Enclave, manages the fuse. This API
 * requests FW to read the common fuses. FW sends the read value as
 * response.
 *
 * Context: This function takes two mutex locks: one on the command
 *          and second on the message unit.
 *          such that multiple commands cannot be sent.
 *          for the device Describes whether the function can sleep, what locks it takes,
 *          releases, or expects to be held. It can extend over multiple
 *          lines.
 * Return: Describe the return value of function_name.
 *
 * The return value description can also have multiple paragraphs, and should
 * be placed at the end of the comment block.
 */
int read_common_fuse(struct device *dev,
		     uint16_t fuse_id, u32 *value)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&priv->mu_cmd_lock);
	ret = imx_se_alloc_tx_rx_buf(priv);
	if (ret)
		goto exit;

	ret = plat_fill_cmd_msg_hdr(priv,
				    (struct mu_hdr *)&priv->tx_msg->header,
				    ELE_READ_FUSE_REQ,
				    ELE_READ_FUSE_REQ_MSG_SZ,
				    true);
	if (ret) {
		pr_err("Error: plat_fill_cmd_msg_hdr failed.\n");
		goto exit;
	}

	priv->tx_msg->data[0] = fuse_id;
	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	switch (fuse_id) {
	case OTP_UNIQ_ID:
		ret = read_otp_uniq_id(priv, value);
		break;
	default:
		ret = read_fuse_word(priv, value);
		break;
	}

exit:
	imx_se_free_tx_rx_buf(priv);

	mutex_unlock(&priv->mu_cmd_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(read_common_fuse);


int ele_voltage_change_req(struct device *dev, bool start)
{
	struct ele_mu_priv *priv = dev_get_drvdata(dev);
	int ret;
	unsigned int status;
	uint8_t cmd = start ? ELE_VOLT_CHANGE_START_REQ : ELE_VOLT_CHANGE_FINISH_REQ;

	if (start)
		mutex_lock(&priv->mu_cmd_lock);

	ret = imx_se_alloc_tx_rx_buf(priv);
	if (ret)
		goto exit;

	ret = plat_fill_cmd_msg_hdr(priv,
				    (struct mu_hdr *)&priv->tx_msg->header,
				    cmd,
				    ELE_VOLT_CHANGE_REQ_MSG_SZ,
				    true);
	if (ret) {
		pr_err("Error: plat_fill_cmd_msg_hdr failed.\n");
		goto exit;
	}

	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		goto exit;

	ret  = validate_rsp_hdr(priv,
				priv->rx_msg->header,
				cmd,
				ELE_VOLT_CHANGE_RSP_MSG_SZ,
				true);
	if (ret)
		goto exit;

	status = RES_STATUS(priv->rx_msg->data[0]);
	if (status != priv->success_tag) {
		dev_err(dev, "Command Id[%d], Response Failure = 0x%x",
			ELE_VOLT_CHANGE_START_REQ, status);
		ret = -1;
	}

exit:
	imx_se_free_tx_rx_buf(priv);

	if (!start || ret)
		mutex_unlock(&priv->mu_cmd_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(ele_voltage_change_req);
