// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ELE Random Number Generator Driver NXP's Platforms
 *
 * Author: Gaurav Jain: <gaurav.jain@nxp.com>
 *
 * Copyright 2022 NXP
 */

#include <linux/hw_random.h>
#include <linux/firmware/imx/ele_base_msg.h>
#include "ele_mu.h"

#define CSAL_STATE_SHIFT	8
#define CSAL_STATE_MASK		0x0000ff00
#define TRNG_STATE_MASK		0x000000ff

struct ele_trng {
	struct hwrng rng;
};

/* Fill a command message header with a given command ID and length in bytes. */
static int plat_fill_rng_msg_hdr(struct mu_hdr *hdr, uint8_t cmd, uint32_t len)
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
	int ret;
	unsigned int tag, command, size, ver, status;
	phys_addr_t addr = virt_to_phys(data);

	/* access ele_mu_priv data structure pointer*/
	ret = get_ele_mu_priv(&priv);
	if (ret)
		return ret;

	ret = plat_fill_rng_msg_hdr((struct mu_hdr *)&priv->tx_msg.header, ELE_GET_RANDOM_REQ, 16);
	if (ret)
		return ret;

	priv->tx_msg.data[0] = 0x0;
	priv->tx_msg.data[1] = lower_32_bits(addr);
	priv->tx_msg.data[2] = len;
	ret = imx_ele_msg_send_rcv(priv);
	if (ret < 0)
		return ret;

	tag = MSG_TAG(priv->rx_msg.header);
	command = MSG_COMMAND(priv->rx_msg.header);
	size = MSG_SIZE(priv->rx_msg.header);
	ver = MSG_VER(priv->rx_msg.header);
	status = RES_STATUS(priv->rx_msg.data[0]);
	if (tag == 0xe1 && command == ELE_GET_RANDOM_REQ && size == 0x02 &&
	    ver == 0x07 && status == 0xd6) {
		return len;
	}

	return -EINVAL;
}

int ele_trng_init(struct device *dev)
{
	struct ele_trng *trng;
	int ret, csal_state, trng_state;

	ret = ele_get_trng_state();
	if (ret < 0) {
		dev_err(dev, "Failed to get trng state\n");
		return ret;
	}

	csal_state = (ret & CSAL_STATE_MASK) >> CSAL_STATE_SHIFT;
	trng_state = ret & TRNG_STATE_MASK;
	if (csal_state != 0x2 || trng_state != 0x3) {
		ret = ele_start_rng();
		if (ret) {
			dev_err(dev, "Failed to start rng\n");
			return ret;
		}
	}

	trng = devm_kzalloc(dev, sizeof(*trng), GFP_KERNEL);
	if (!trng)
		return -ENOMEM;

	trng->rng.name    = "ele-trng";
	trng->rng.read    = ele_get_random;
	trng->rng.priv    = (unsigned long)trng;
	trng->rng.quality = 1024;

	dev_info(dev, "registering ele-trng\n");

	ret = devm_hwrng_register(dev, &trng->rng);
	if (ret)
		return ret;

	dev_info(dev, "Successfully registered ele-trng\n");
	return 0;
}
