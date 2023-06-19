// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ELE Random Number Generator Driver NXP's Platforms
 *
 * Author: Gaurav Jain: <gaurav.jain@nxp.com>
 *
 * Copyright 2022 NXP
 */

#include <linux/firmware/imx/ele_fw_api.h>

#include "ele_mu.h"

struct ele_trng {
	struct hwrng rng;
};

int ele_trng_init(struct device *dev)
{
	struct ele_trng *trng;
	int ret;

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
