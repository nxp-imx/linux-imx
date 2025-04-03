// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC switch driver
 * Copyright 2025 NXP
 */

#include "netc_switch.h"

struct netc_switch_platform {
	char compatible[128];
	u16 revision;
	const struct netc_switch_info *info;
};

static void imx94_switch_phylink_get_caps(int port, struct phylink_config *config)
{
	config->mac_capabilities |= MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
				    MAC_10 | MAC_100 | MAC_1000FD;

	switch (port) {
	case 0 ... 1:
		__set_bit(PHY_INTERFACE_MODE_SGMII,
			  config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_1000BASEX,
			  config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_2500BASEX,
			  config->supported_interfaces);
		config->mac_capabilities |= MAC_2500FD;
		fallthrough;
	case 2:
		__set_bit(PHY_INTERFACE_MODE_MII, config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_RMII, config->supported_interfaces);
		if (port == 2)
			__set_bit(PHY_INTERFACE_MODE_REVMII, config->supported_interfaces);

		phy_interface_set_rgmii(config->supported_interfaces);
		break;
	case 3: /* CPU port */
		__set_bit(PHY_INTERFACE_MODE_INTERNAL, config->supported_interfaces);
		config->mac_capabilities |= MAC_2500FD;
	}
}

static const struct netc_switch_info imx94_info = {
	.cpu_port_num = 1,
	.usr_port_num = 3,
	.tmr_devfn = 1,
	.phylink_get_caps = imx94_switch_phylink_get_caps,
};

static const struct netc_switch_platform netc_platforms[] = {
	{ .compatible = "nxp,imx94-netc-switch",
	  .revision = NETC_SWITCH_REV_4_3,
	  .info = &imx94_info,
	},
	{ },
};

static const struct netc_switch_info *netc_switch_get_info(struct netc_switch *priv)
{
	struct device_node *node = priv->dev->of_node;
	int i;

	/* Matches based on compatible string */
	for (i = 0; i < ARRAY_SIZE(netc_platforms); i++) {
		if (of_device_is_compatible(node, netc_platforms[i].compatible) > 0)
			return netc_platforms[i].info;
	}

	/* Matches based on IP revision, some platform may have no device node */
	for (i = 0; i < ARRAY_SIZE(netc_platforms); i++) {
		if (priv->revision == netc_platforms[i].revision)
			return netc_platforms[i].info;
	}

	return NULL;
}

int netc_switch_platform_probe(struct netc_switch *priv)
{
	const struct netc_switch_info *info;
	struct device *dev = priv->dev;

	info = netc_switch_get_info(priv);
	if (!info) {
		dev_err(dev, "Cannot find switch platform info\n");
		return -EINVAL;
	}

	priv->info = info;
	priv->num_ports = info->usr_port_num + info->cpu_port_num;

	return 0;
}
