// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware/imx/ele_base_msg.h>

#include "ele_common.h"
#include "se_fw.h"

int imx_se_alloc_tx_rx_buf(struct ele_mu_priv *priv)
{
	int ret = 0;

	priv->tx_msg = devm_kzalloc(priv->dev,
				    sizeof(*priv->tx_msg),
				    GFP_KERNEL);
	if (!priv->tx_msg) {
		ret = -ENOMEM;
		dev_err(priv->dev, "Fail allocate mem for tx_msg.\n");
		return ret;
	}

	priv->rx_msg = devm_kzalloc(priv->dev,
				    sizeof(*priv->rx_msg),
				    GFP_KERNEL);

	if (!priv->rx_msg) {
		ret = -ENOMEM;
		dev_err(priv->dev, "Fail allocate mem for rx_msg.\n");
		return ret;
	}

	return ret;
}

void imx_se_free_tx_rx_buf(struct ele_mu_priv *priv)
{
	if (priv->tx_msg)
		devm_kfree(priv->dev, priv->tx_msg);

	if (priv->rx_msg)
		devm_kfree(priv->dev, priv->rx_msg);
}

uint32_t plat_add_msg_crc(uint32_t *msg, uint32_t msg_len)
{
	uint32_t i;
	uint32_t crc = 0;
	uint32_t nb_words = msg_len / (uint32_t)sizeof(uint32_t);

	for (i = 0; i < nb_words - 1; i++)
		crc ^= *(msg + i);

	return crc;
}

int imx_ele_msg_send_rcv(struct ele_mu_priv *priv)
{
	unsigned int wait;
	int err;

	mutex_lock(&priv->mu_lock);

	err = mbox_send_message(priv->tx_chan, priv->tx_msg);
	if (err < 0) {
		pr_err("Error: mbox_send_message failure.\n");
		mutex_unlock(&priv->mu_lock);
		return err;
	}
	err = 0;

	mutex_unlock(&priv->mu_lock);

	wait = msecs_to_jiffies(1000);
	if (!wait_for_completion_timeout(&priv->done, wait)) {
		pr_err("Error: wait_for_completion timed out.\n");
		err = -ETIMEDOUT;
	}


	return err;
}

/* Fill a command message header with a given command ID and length in bytes. */
int plat_fill_cmd_msg_hdr(struct ele_mu_priv *priv,
			  struct mu_hdr *hdr,
			  uint8_t cmd,
			  uint32_t len,
			  bool is_base_api)
{
	hdr->tag = priv->cmd_tag;
	hdr->ver = (is_base_api) ? priv->base_api_ver : priv->fw_api_ver;
	hdr->command = cmd;
	hdr->size = len >> 2;

	return 0;
}

int validate_rsp_hdr(struct ele_mu_priv *priv, unsigned int header,
		     uint8_t msg_id, uint8_t sz, bool is_base_api)
{
	unsigned int tag, command, size, ver;
	int ret = -EINVAL;

	tag = MSG_TAG(header);
	command = MSG_COMMAND(header);
	size = MSG_SIZE(header);
	ver = MSG_VER(header);

	do {
		if (tag != priv->rsp_tag) {
			dev_err(priv->dev,
				"MSG[0x%x] Hdr: Resp tag mismatch. (0x%x != 0x%x)",
				msg_id, tag, priv->rsp_tag);
			break;
		}

		if (command != msg_id) {
			dev_err(priv->dev,
				"MSG Header: Cmd id mismatch. (0x%x != 0x%x)",
				command, msg_id);
			break;
		}

		if (size != (sz >> 2)) {
			dev_err(priv->dev,
				"MSG[0x%x] Hdr: Cmd size mismatch. (0x%x != 0x%x)",
				msg_id, size, (sz >> 2));
			break;
		}

		if (is_base_api && (ver != priv->base_api_ver)) {
			dev_err(priv->dev,
				"MSG[0x%x] Hdr: Base API Vers mismatch. (0x%x != 0x%x)",
				msg_id, ver, priv->base_api_ver);
			break;
		} else if (!is_base_api && ver != priv->fw_api_ver) {
			dev_err(priv->dev,
				"MSG[0x%x] Hdr: FW API Vers mismatch. (0x%x != 0x%x)",
				msg_id, ver, priv->fw_api_ver);
			break;
		}

		ret = 0;

	} while (false);

	return ret;
}

int ele_do_start_rng(struct device *dev)
{
	int ret;
	int count = ELE_GET_TRNG_STATE_RETRY_COUNT;

	ret = ele_get_trng_state(dev);
	if (ret < 0) {
		dev_err(dev, "Failed to get trng state\n");
		return ret;
	} else if (ret != ELE_TRNG_STATE_OK) {
		/* call start rng */
		ret = ele_start_rng(dev);
		if (ret) {
			dev_err(dev, "Failed to start rng\n");
			return ret;
		}

		/* poll get trng state API, ELE_GET_TRNG_STATE_RETRY_COUNT times
		 * or while trng state != 0x203
		 */
		do {
			msleep(10);
			ret = ele_get_trng_state(dev);
			if (ret < 0) {
				dev_err(dev, "Failed to get trng state\n");
				return ret;
			}
			count--;
		} while ((ret != ELE_TRNG_STATE_OK) && count);
		if (ret != ELE_TRNG_STATE_OK)
			return -EIO;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
int save_imem(struct device *dev)
{
	int ret;
	struct ele_mu_priv *priv = dev_get_drvdata(dev);

	/* EXPORT command will save encrypted IMEM to given address,
	 * so later in resume, IMEM can be restored from the given
	 * address.
	 *
	 * Size must be at least 64 kB.
	 */
	ret = ele_service_swap(dev,
			       priv->imem.phyaddr,
			       ELE_IMEM_SIZE,
			       ELE_IMEM_EXPORT);
	if (ret < 0)
		dev_err(dev, "Failed to export IMEM\n");
	else
		dev_info(dev,
				"Exported %d bytes of encrypted IMEM\n",
				ret);

	return ret;
}

int restore_imem(struct device *dev,
		 uint8_t *pool_name)
{
	int ret;
	u32 imem_state;
	u32 *get_info_buf = NULL;
	phys_addr_t get_info_phyaddr = 0;
	struct ele_mu_priv *priv = dev_get_drvdata(dev);

	get_info_phyaddr
		= pool_name ? get_phy_buf_mem_pool(dev,
						   pool_name,
						   &get_info_buf,
						   DEVICE_GET_INFO_SZ)
			    : 0x0;

	if (!get_info_buf) {
		dev_err(dev, "Unable to alloc sram from sram pool\n");
		return -ENOMEM;
	}

	ret = ele_do_start_rng(dev);
	if (ret)
		goto exit;

	/* get info from ELE */
	ret = ele_get_info(dev, get_info_phyaddr, ELE_GET_INFO_READ_SZ);
	if (ret) {
		dev_err(dev, "Failed to get info from ELE.\n");
		goto exit;
	}

	/* Get IMEM state, if 0xFE then import IMEM */
	imem_state = (get_info_buf[ELE_IMEM_STATE_WORD]
			& ELE_IMEM_STATE_MASK) >> 16;
	if (imem_state == ELE_IMEM_STATE_BAD) {
		/* IMPORT command will restore IMEM from the given
		 * address, here size is the actual size returned by ELE
		 * during the export operation
		 */
		ret = ele_service_swap(dev,
				       priv->imem.phyaddr,
				       priv->imem.size,
				       ELE_IMEM_IMPORT);
		if (ret) {
			dev_err(dev, "Failed to import IMEM\n");
			goto exit;
		}
	} else
		goto exit;

	/* After importing IMEM, check if IMEM state is equal to 0xCA
	 * to ensure IMEM is fully loaded and
	 * ELE functionality can be used.
	 */
	ret = ele_get_info(dev, get_info_phyaddr, ELE_GET_INFO_READ_SZ);
	if (ret) {
		dev_err(dev, "Failed to get info from ELE.\n");
		goto exit;
	}

	imem_state = (get_info_buf[ELE_IMEM_STATE_WORD]
			& ELE_IMEM_STATE_MASK) >> 16;
	if (imem_state == ELE_IMEM_STATE_OK)
		dev_info(dev, "Successfully restored IMEM\n");
	else
		dev_err(dev, "Failed to restore IMEM\n");

exit:
	if (pool_name && get_info_buf)
		free_phybuf_mem_pool(dev, pool_name,
				get_info_buf, DEVICE_GET_INFO_SZ);

	return ret;
}
#endif
