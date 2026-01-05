// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC switch driver
 * Copyright 2025 NXP
 */

#include "netc_switch.h"

/* MANT = bits 11:4, EXP = bits 3:0, threshold = MANT * 2 ^ EXP */
#define IMX94_BP_MAX_THRESH		0x334
#define IMX94_PORT_FC_THRESH_ON		0x533
#define IMX94_PORT_FC_THRESH_OFF	0x3c3
#define IMX94_PORT_BF_MAPPING(i)	((i) << 24 | (i) << 16 | (i) << 8 | (i))

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

static void imx94_switch_bpt_init(struct netc_switch *priv)
{
	int i;

	for (i = 0; i < priv->caps.num_bp; i++) {
		struct bpt_cfge_data *cfge = &priv->bpt_list[i];

		cfge->max_thresh = cpu_to_le16(IMX94_BP_MAX_THRESH);
		ntmp_bpt_update_entry(&priv->ntmp.cbdrs, i, cfge);
	}

	/* For i.MX94, each port has two dedicated buffer pools,
	 * the indexes are port->index * 2 ~ port->index * 2 + 1.
	 * IPV 0 ~ 3 map to the first buffer pool
	 * IPV 4 ~ 7 map to the second buffer pool
	 */
	for (i = 0; i < priv->num_ports; i++) {
		struct netc_port *port = priv->ports[i];
		int j = i * 2;

		netc_port_wr(port, NETC_PBPMCR0, IMX94_PORT_BF_MAPPING(j));
		netc_port_wr(port, NETC_PBPMCR1, IMX94_PORT_BF_MAPPING(j + 1));
	}
}

static void imx94_port_tx_pause_config(struct netc_port *port, bool en)
{
	struct netc_switch *priv = port->switch_priv;
	int port_id = port->index;
	int i, num_bps;

	num_bps = priv->caps.num_bp / priv->num_ports;
	for (i = 0; i < num_bps; i++) {
		int j = port_id * num_bps + i;
		struct bpt_cfge_data *cfge;

		cfge = &priv->bpt_list[j];
		if (en) {
			cfge->fc_on_thresh = cpu_to_le16(IMX94_PORT_FC_THRESH_ON);
			cfge->fc_off_thresh = cpu_to_le16(IMX94_PORT_FC_THRESH_OFF);
			cfge->fccfg_sbpen = FIELD_PREP(BPT_FC_CFG, BPT_FC_CFG_EN_BPFC);
			cfge->fc_ports = cpu_to_le32(BIT(port_id));
		} else {
			cfge->fc_on_thresh = cpu_to_le16(0);
			cfge->fc_off_thresh = cpu_to_le16(0);
			cfge->fccfg_sbpen = 0;
			cfge->fc_ports = cpu_to_le32(0);
		}

		ntmp_bpt_update_entry(&priv->ntmp.cbdrs, j, cfge);
	}
}

static const struct netc_switch_info imx94_info = {
	.cpu_port_num = 1,
	.usr_port_num = 3,
	.tmr_devfn = 1,
	.sysclk_freq = NETC_SYSCLK_333M,
	.phylink_get_caps = imx94_switch_phylink_get_caps,
	.bpt_init = imx94_switch_bpt_init,
	.port_tx_pause_config = imx94_port_tx_pause_config,
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
