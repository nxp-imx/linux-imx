// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2024-2025 NXP */

#include <linux/fsl/enetc_mdio.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/regulator/consumer.h>

#include "enetc_pf_common.h"
#include "enetc_msg.h"

static void enetc_set_si_hw_addr(struct enetc_pf *pf, int si,
				 const u8 *mac_addr)
{
	struct enetc_hw *hw = &pf->si->hw;

	pf->ops->set_si_primary_mac(hw, si, mac_addr);
}

static void enetc_get_si_hw_addr(struct enetc_pf *pf, int si, u8 *mac_addr)
{
	struct enetc_hw *hw = &pf->si->hw;

	pf->ops->get_si_primary_mac(hw, si, mac_addr);
}

int enetc_pf_set_mac_addr(struct net_device *ndev, void *addr)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct sockaddr *saddr = addr;

	if (!is_valid_ether_addr(saddr->sa_data))
		return -EADDRNOTAVAIL;

	eth_hw_addr_set(ndev, saddr->sa_data);
	enetc_set_si_hw_addr(pf, 0, saddr->sa_data);

	return 0;
}
EXPORT_SYMBOL_GPL(enetc_pf_set_mac_addr);

static int enetc_setup_mac_address(struct device_node *np, struct enetc_pf *pf,
				   int si)
{
	struct device *dev = &pf->si->pdev->dev;
	struct net_device *ndev = pf->si->ndev;
	u8 mac_addr[ETH_ALEN] = { 0 };
	int err;

	/* (0) try to get the MAC address from netdev */
	if (ndev && ndev->dev_addr && is_valid_ether_addr(ndev->dev_addr))
		memcpy(mac_addr, ndev->dev_addr, ETH_ALEN);

	/* (1) try to get the MAC address from the device tree */
	if (is_zero_ether_addr(mac_addr) && np) {
		err = of_get_mac_address(np, mac_addr);
		if (err == -EPROBE_DEFER)
			return err;
	}

	/* (2) bootloader supplied MAC address */
	if (is_zero_ether_addr(mac_addr))
		enetc_get_si_hw_addr(pf, si, mac_addr);

	/* (3) choose a random one */
	if (is_zero_ether_addr(mac_addr)) {
		eth_random_addr(mac_addr);
		dev_info(dev, "no MAC address specified for SI%d, using %pM\n",
			 si, mac_addr);
	}

	enetc_set_si_hw_addr(pf, si, mac_addr);

	/* si == 0 is PF */
	if (!si)
		memcpy(pf->mac_addr_base, mac_addr, ETH_ALEN);

	return 0;
}

int enetc_setup_mac_addresses(struct device_node *np, struct enetc_pf *pf)
{
	int err, i;

	/* The PF might take its MAC from the device tree */
	err = enetc_setup_mac_address(np, pf, 0);
	if (err)
		return err;

	for (i = 0; i < pf->total_vfs; i++) {
		if (is_enetc_rev1(pf->si)) {
			err = enetc_setup_mac_address(NULL, pf, i + 1);
			if (err)
				return err;
		} else {
			u8 mac_addr[ETH_ALEN];

			memcpy(mac_addr, pf->mac_addr_base, ETH_ALEN);
			eth_addr_add(mac_addr, i + 1);
			if (!is_valid_ether_addr(mac_addr)) {
				eth_random_addr(mac_addr);
				dev_info(&pf->si->pdev->dev,
					 "SI%d: Invalid MAC addr, using %pM\n",
					 i + 1, mac_addr);
			}
			enetc_set_si_hw_addr(pf, i + 1, mac_addr);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(enetc_setup_mac_addresses);

void enetc_pf_netdev_setup(struct enetc_si *si, struct net_device *ndev,
			   const struct net_device_ops *ndev_ops)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(si);

	SET_NETDEV_DEV(ndev, &si->pdev->dev);
	priv->ndev = ndev;
	priv->si = si;
	priv->dev = &si->pdev->dev;
	si->ndev = ndev;

	priv->msg_enable = (NETIF_MSG_WOL << 1) - 1;
	priv->sysclk_freq = si->drvdata->sysclk_freq;
	priv->max_frags = si->drvdata->max_frags;
	priv->shared_tx_rings = si->drvdata->shared_tx_rings;
	ndev->netdev_ops = ndev_ops;
	enetc_set_ethtool_ops(ndev);
	ndev->watchdog_timeo = 5 * HZ;
	ndev->max_mtu = ENETC_MAX_MTU;

	ndev->hw_features = NETIF_F_SG | NETIF_F_RXCSUM |
			    NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX |
			    NETIF_F_HW_VLAN_CTAG_FILTER |
			    NETIF_F_HW_CSUM | NETIF_F_TSO | NETIF_F_TSO6 |
			    NETIF_F_GSO_UDP_L4;
	ndev->features = NETIF_F_HIGHDMA | NETIF_F_SG | NETIF_F_RXCSUM |
			 NETIF_F_HW_VLAN_CTAG_TX |
			 NETIF_F_HW_VLAN_CTAG_RX |
			 NETIF_F_HW_CSUM | NETIF_F_TSO | NETIF_F_TSO6 |
			 NETIF_F_GSO_UDP_L4;
	ndev->vlan_features = NETIF_F_SG | NETIF_F_HW_CSUM |
			      NETIF_F_TSO | NETIF_F_TSO6;

	ndev->priv_flags |= IFF_UNICAST_FLT;

	if (si->drvdata->tx_csum)
		priv->active_offloads |= ENETC_F_TXCSUM;

	if (si->hw_features & ENETC_SI_F_LSO)
		priv->active_offloads |= ENETC_F_LSO;

	if (si->hw_features & ENETC_SI_F_RSC)
		ndev->hw_features |= NETIF_F_LRO;

	if (si->num_rss) {
		ndev->hw_features |= NETIF_F_RXHASH;
		ndev->features |= NETIF_F_RXHASH;
	}

	ndev->xdp_features = NETDEV_XDP_ACT_BASIC | NETDEV_XDP_ACT_REDIRECT |
			     NETDEV_XDP_ACT_NDO_XMIT | NETDEV_XDP_ACT_RX_SG |
			     NETDEV_XDP_ACT_NDO_XMIT_SG |
			     NETDEV_XDP_ACT_XSK_ZEROCOPY;

	ndev->xdp_zc_max_segs = priv->max_frags;
	ndev->xdp_metadata_ops = &enetc_xdp_metadata_ops;
	ndev->xsk_tx_metadata_ops = &enetc_xsk_tx_metadata_ops;

	if (!is_enetc_rev1(si)) {
		ndev->features |= NETIF_F_HW_TC;
		ndev->hw_features |= NETIF_F_HW_TC;
	} else if (si->hw_features & ENETC_SI_F_PSFP && pf->ops->enable_psfp &&
		   !pf->ops->enable_psfp(priv)) {
		priv->active_offloads |= ENETC_F_QCI;
		ndev->features |= NETIF_F_HW_TC;
		ndev->hw_features |= NETIF_F_HW_TC;
	}

	if (!enetc_is_pseudo_mac(si))
		ndev->hw_features |= NETIF_F_LOOPBACK;

	/* pick up primary MAC address from SI */
	enetc_load_primary_mac_addr(&si->hw, ndev);
}
EXPORT_SYMBOL_GPL(enetc_pf_netdev_setup);

static int enetc_get_mdio_base(struct enetc_si *si, bool imdio)
{
	if (is_enetc_rev1(si))
		return imdio ? ENETC_PM_IMDIO_BASE : ENETC_EMDIO_BASE;

	return imdio ? ENETC4_PM_IMDIO_BASE : ENETC4_EMDIO_BASE;
}

static int enetc_mdio_probe(struct enetc_pf *pf, struct device_node *np)
{
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_mdio_priv *mdio_priv;
	struct mii_bus *bus;
	int err;

	bus = devm_mdiobus_alloc_size(dev, sizeof(*mdio_priv));
	if (!bus)
		return -ENOMEM;

	bus->name = "Freescale ENETC MDIO Bus";
	bus->read = enetc_mdio_read_c22;
	bus->write = enetc_mdio_write_c22;
	bus->read_c45 = enetc_mdio_read_c45;
	bus->write_c45 = enetc_mdio_write_c45;
	bus->parent = dev;
	mdio_priv = bus->priv;
	mdio_priv->hw = &pf->si->hw;
	mdio_priv->mdio_base = enetc_get_mdio_base(pf->si, false);
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));

	err = of_mdiobus_register(bus, np);
	if (err)
		return dev_err_probe(dev, err, "cannot register MDIO bus\n");

	pf->mdio = bus;

	return 0;
}

static void enetc_mdio_remove(struct enetc_pf *pf)
{
	if (pf->mdio)
		mdiobus_unregister(pf->mdio);
}

static int enetc_imdio_create(struct enetc_pf *pf)
{
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_mdio_priv *mdio_priv;
	struct phylink_pcs *phylink_pcs;
	struct mii_bus *bus;
	int err;

	bus = mdiobus_alloc_size(sizeof(*mdio_priv));
	if (!bus)
		return -ENOMEM;

	bus->name = "Freescale ENETC internal MDIO Bus";
	bus->read = enetc_mdio_read_c22;
	bus->write = enetc_mdio_write_c22;
	bus->read_c45 = enetc_mdio_read_c45;
	bus->write_c45 = enetc_mdio_write_c45;
	bus->parent = dev;
	bus->phy_mask = ~0;
	mdio_priv = bus->priv;
	mdio_priv->hw = &pf->si->hw;
	mdio_priv->mdio_base = enetc_get_mdio_base(pf->si, true);
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-imdio", dev_name(dev));

	mdio_priv->regulator = devm_regulator_get_optional(dev, "serdes");
	if (IS_ERR(mdio_priv->regulator)) {
		err = PTR_ERR(mdio_priv->regulator);
		if (err == -EPROBE_DEFER)
			goto free_mdio_bus;
		mdio_priv->regulator = NULL;
	}

	if (mdio_priv->regulator) {
		err = regulator_enable(mdio_priv->regulator);
		if (err) {
			dev_err(dev, "fail to enable phy-supply\n");
			goto free_mdio_bus;
		}
	}

	err = mdiobus_register(bus);
	if (err) {
		dev_err(dev, "cannot register internal MDIO bus (%d)\n", err);
		goto disable_regulator;
	}

	phylink_pcs = pf->ops->create_pcs(pf, bus);
	if (IS_ERR(phylink_pcs)) {
		err = PTR_ERR(phylink_pcs);
		dev_err(dev, "cannot create ENETC pcs (%d)\n", err);
		goto unregister_mdiobus;
	}

	pf->imdio = bus;
	pf->pcs = phylink_pcs;

	return 0;

unregister_mdiobus:
	mdiobus_unregister(bus);
disable_regulator:
	if (mdio_priv->regulator)
		regulator_disable(mdio_priv->regulator);
free_mdio_bus:
	mdiobus_free(bus);
	return err;
}

static void enetc_imdio_remove(struct enetc_pf *pf)
{
	struct mii_bus *imdio = pf->imdio;

	if (pf->pcs)
		pf->ops->destroy_pcs(pf->pcs);

	if (imdio) {
		struct enetc_mdio_priv *mdio_priv = imdio->priv;

		mdiobus_unregister(imdio);
		if (mdio_priv && mdio_priv->regulator)
			regulator_disable(mdio_priv->regulator);

		mdiobus_free(imdio);
	}
}

static bool enetc_port_has_pcs(struct enetc_pf *pf)
{
	return (pf->if_mode == PHY_INTERFACE_MODE_SGMII ||
		pf->if_mode == PHY_INTERFACE_MODE_1000BASEX ||
		pf->if_mode == PHY_INTERFACE_MODE_2500BASEX ||
		pf->if_mode == PHY_INTERFACE_MODE_USXGMII ||
		pf->if_mode == PHY_INTERFACE_MODE_10GBASER ||
		pf->if_mode == PHY_INTERFACE_MODE_XGMII);
}

int enetc_mdiobus_create(struct enetc_pf *pf, struct device_node *node)
{
	struct device_node *mdio_np;
	int err;

	if (!node)
		return 0;

	mdio_np = of_get_child_by_name(node, "mdio");
	if (mdio_np) {
		err = enetc_mdio_probe(pf, mdio_np);

		of_node_put(mdio_np);
		if (err)
			return err;
	}

	if (enetc_port_has_pcs(pf)) {
		err = enetc_imdio_create(pf);
		if (err) {
			enetc_mdio_remove(pf);
			return err;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(enetc_mdiobus_create);

void enetc_mdiobus_destroy(struct enetc_pf *pf)
{
	enetc_mdio_remove(pf);
	enetc_imdio_remove(pf);
}
EXPORT_SYMBOL_GPL(enetc_mdiobus_destroy);

static struct {
	int speed;
	unsigned long mac_caps;
} enetc_pseudo_mac_phylink_caps[] = {
	{ SPEED_25000, MAC_25000FD },
	{ SPEED_20000, MAC_20000FD },
	{ SPEED_10000, MAC_10000FD },
	{ SPEED_5000,  MAC_5000FD  },
	{ SPEED_2500,  MAC_2500FD  },
	{ SPEED_1000,  MAC_1000FD  },
	{ SPEED_100,   MAC_100FD   },
	{ SPEED_10,    MAC_10FD    },
};

static unsigned long enetc_get_pseudo_mac_caps(void)
{
	unsigned long mac_caps = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(enetc_pseudo_mac_phylink_caps); i++)
		mac_caps |= enetc_pseudo_mac_phylink_caps[i].mac_caps;

	return mac_caps;
}

int enetc_phylink_match_pseudo_mac_speed(int speed)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(enetc_pseudo_mac_phylink_caps); i++)
		if (enetc_pseudo_mac_phylink_caps[i].speed <= speed)
			return enetc_pseudo_mac_phylink_caps[i].speed;

	return SPEED_UNKNOWN;
}
EXPORT_SYMBOL_GPL(enetc_phylink_match_pseudo_mac_speed);

int enetc_phylink_create(struct enetc_ndev_priv *priv,
			 const struct phylink_mac_ops *ops)
{
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct phylink *phylink;
	unsigned long mac_caps;
	int err;

	pf->phylink_config.dev = &priv->ndev->dev;
	pf->phylink_config.type = PHYLINK_NETDEV;

	if (!enetc_is_pseudo_mac(priv->si))
		mac_caps = MAC_10 | MAC_100 | MAC_1000 | MAC_2500FD |
			   MAC_10000FD;
	else
		mac_caps = enetc_get_pseudo_mac_caps();

	mac_caps |= MAC_ASYM_PAUSE | MAC_SYM_PAUSE;

	pf->phylink_config.mac_capabilities = mac_caps;
	__set_bit(PHY_INTERFACE_MODE_INTERNAL,
		  pf->phylink_config.supported_interfaces);

	if (!enetc_is_pseudo_mac(priv->si)) {
		__set_bit(PHY_INTERFACE_MODE_RMII,
			  pf->phylink_config.supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_SGMII,
			  pf->phylink_config.supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_1000BASEX,
			  pf->phylink_config.supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_2500BASEX,
			  pf->phylink_config.supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_USXGMII,
			  pf->phylink_config.supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_10GBASER,
			  pf->phylink_config.supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_XGMII,
			  pf->phylink_config.supported_interfaces);
		phy_interface_set_rgmii(pf->phylink_config.supported_interfaces);
	}

	phylink = phylink_create(&pf->phylink_config, enetc_fwnode(priv),
				 pf->if_mode, ops);
	if (IS_ERR(phylink)) {
		err = PTR_ERR(phylink);
		return err;
	}

	priv->phylink = phylink;

	return 0;
}
EXPORT_SYMBOL_GPL(enetc_phylink_create);

void enetc_phylink_destroy(struct enetc_ndev_priv *priv)
{
	phylink_destroy(priv->phylink);
}
EXPORT_SYMBOL_GPL(enetc_phylink_destroy);

void enetc_set_default_rss_key(struct enetc_pf *pf)
{
	u8 hash_key[ENETC_RSSHASH_KEY_SIZE] = {0};

	/* set up hash key */
	get_random_bytes(hash_key, ENETC_RSSHASH_KEY_SIZE);
	enetc_set_rss_key(pf->si, hash_key);
}
EXPORT_SYMBOL_GPL(enetc_set_default_rss_key);

void enetc_set_si_vlan_ht_filter(struct enetc_si *si, int si_id, u64 hash)
{
	struct enetc_hw *hw = &si->hw;
	int high_reg_off, low_reg_off;

	if (is_enetc_rev1(si)) {
		low_reg_off = ENETC_PSIVHFR0(si_id);
		high_reg_off = ENETC_PSIVHFR1(si_id);
	} else {
		low_reg_off = ENETC4_PSIVHFR0(si_id);
		high_reg_off = ENETC4_PSIVHFR1(si_id);
	}

	enetc_port_wr(hw, low_reg_off, lower_32_bits(hash));
	enetc_port_wr(hw, high_reg_off, upper_32_bits(hash));
}
EXPORT_SYMBOL_GPL(enetc_set_si_vlan_ht_filter);

int enetc_vlan_rx_add_vid(struct net_device *ndev, __be16 prot, u16 vid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	int idx;

	__set_bit(vid, si->active_vlans);

	idx = enetc_vid_hash_idx(vid);
	if (!__test_and_set_bit(idx, si->vlan_ht_filter))
		enetc_set_si_vlan_ht_filter(si, 0, *si->vlan_ht_filter);

	return 0;
}
EXPORT_SYMBOL_GPL(enetc_vlan_rx_add_vid);

int enetc_vlan_rx_del_vid(struct net_device *ndev, __be16 prot, u16 vid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;

	if (__test_and_clear_bit(vid, si->active_vlans)) {
		enetc_refresh_vlan_ht_filter(si);
		enetc_set_si_vlan_ht_filter(si, 0, *si->vlan_ht_filter);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(enetc_vlan_rx_del_vid);

void enetc_reset_taprio_stats(struct enetc_ndev_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_tx_rings; i++)
		priv->tx_ring[i]->stats.win_drop = 0;
}
EXPORT_SYMBOL_GPL(enetc_reset_taprio_stats);

void enetc_taprio_stats(struct net_device *ndev,
			struct tc_taprio_qopt_stats *stats)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	u64 window_drops = 0;
	int i;

	for (i = 0; i < priv->num_tx_rings; i++)
		window_drops += priv->tx_ring[i]->stats.win_drop;

	stats->window_drops = window_drops;
}
EXPORT_SYMBOL_GPL(enetc_taprio_stats);

void enetc_taprio_queue_stats(struct net_device *ndev,
			      struct tc_taprio_qopt_queue_stats *queue_stats)
{
	struct tc_taprio_qopt_stats *stats = &queue_stats->stats;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int queue = queue_stats->queue;

	stats->window_drops = priv->tx_ring[queue]->stats.win_drop;
}
EXPORT_SYMBOL_GPL(enetc_taprio_queue_stats);

int enetc_qos_query_caps(struct net_device *ndev, void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct tc_query_caps_base *base = type_data;
	struct enetc_si *si = priv->si;

	switch (base->type) {
	case TC_SETUP_QDISC_MQPRIO: {
		struct tc_mqprio_caps *caps = base->caps;

		caps->validate_queue_counts = true;

		return 0;
	}
	case TC_SETUP_QDISC_TAPRIO: {
		struct tc_taprio_caps *caps = base->caps;

		if (si->hw_features & ENETC_SI_F_QBV)
			caps->supports_queue_max_sdu = true;

		return 0;
	}
	default:
		return -EOPNOTSUPP;
	}
}
EXPORT_SYMBOL_GPL(enetc_qos_query_caps);

#if IS_ENABLED(CONFIG_PCI_IOV)
int enetc_sriov_configure(struct pci_dev *pdev, int num_vfs)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_pf *pf;
	int err;

	if (is_enetc_proxy_pf(pdev)) {
		if (!num_vfs) {
			pci_disable_sriov(pdev);

			return 0;
		}

		err = pci_enable_sriov(pdev, num_vfs);
		if (err < 0) {
			dev_err(&pdev->dev, "pci_enable_sriov err %pe\n",
				ERR_PTR(err));
			return err;
		}

		return num_vfs;
	}

	pf = enetc_si_priv(si);

	if (!num_vfs) {
		pci_disable_sriov(pdev);
		enetc_msg_psi_free(pf);
		pf->num_vfs = 0;
	} else {
		pf->num_vfs = num_vfs;

		err = enetc_msg_psi_init(pf);
		if (err) {
			dev_err(&pdev->dev, "enetc_msg_psi_init (%d)\n", err);
			goto err_msg_psi;
		}

		err = pci_enable_sriov(pdev, num_vfs);
		if (err) {
			dev_err(&pdev->dev, "pci_enable_sriov err %d\n", err);
			goto err_en_sriov;
		}
	}

	return num_vfs;

err_en_sriov:
	enetc_msg_psi_free(pf);
err_msg_psi:
	pf->num_vfs = 0;

	return err;
}
EXPORT_SYMBOL_GPL(enetc_sriov_configure);

void enetc_pf_send_link_status_msg(struct enetc_pf *pf, bool up)
{
	struct device *dev = &pf->si->pdev->dev;
	union enetc_pf_msg pf_msg;
	u16 ms_mask = 0;
	int i, err;

	for (i = 0; i < pf->num_vfs; i++)
		if (pf->vf_link_status_notify[i])
			ms_mask |= PSIMSGSR_MS(i);

	if (!ms_mask)
		return;

	pf_msg.class_id = ENETC_MSG_CLASS_ID_LINK_STATUS;
	pf_msg.class_code = up ? ENETC_PF_NC_LINK_STATUS_UP :
				 ENETC_PF_NC_LINK_STATUS_DOWN;

	err = enetc_pf_send_msg(pf, pf_msg.code, ms_mask);
	if (err)
		dev_err(dev, "PF notifies link status failed\n");
}
EXPORT_SYMBOL_GPL(enetc_pf_send_link_status_msg);
#endif

int enetc_pf_set_vf_trust(struct net_device *ndev, int vf, bool setting)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_vf_state *vf_state;

	if (vf >= pf->total_vfs)
		return -EINVAL;

	vf_state = &pf->vf_state[vf];

	if (setting)
		vf_state->flags |= ENETC_VF_FLAG_TRUSTED;
	else
		vf_state->flags &= ~ENETC_VF_FLAG_TRUSTED;

	return 0;
}
EXPORT_SYMBOL_GPL(enetc_pf_set_vf_trust);

int enetc_pf_set_vf_mac(struct net_device *ndev, int vf, u8 *mac)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_vf_state *vf_state;

	if (vf >= pf->total_vfs)
		return -EINVAL;

	if (!is_valid_ether_addr(mac))
		return -EADDRNOTAVAIL;

	vf_state = &pf->vf_state[vf];
	vf_state->flags |= ENETC_VF_FLAG_PF_SET_MAC;
	enetc_set_si_hw_addr(pf, vf + 1, mac);

	return 0;
}
EXPORT_SYMBOL_GPL(enetc_pf_set_vf_mac);

MODULE_DESCRIPTION("NXP ENETC PF common functionality driver");
MODULE_LICENSE("Dual BSD/GPL");
