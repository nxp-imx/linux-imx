// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2019 NXP */

#include <linux/fsl/mc.h>
#include <linux/msi.h>
#include <linux/acpi.h>
#include <linux/property.h>

#include "dpaa2-eth.h"
#include "dpaa2-mac.h"

#define phylink_to_dpaa2_mac(config) \
	container_of((config), struct dpaa2_mac, phylink_config)

static int phy_mode(enum dpmac_eth_if eth_if, phy_interface_t *if_mode)
{
	*if_mode = PHY_INTERFACE_MODE_NA;

	switch (eth_if) {
	case DPMAC_ETH_IF_RGMII:
		*if_mode = PHY_INTERFACE_MODE_RGMII;
		break;
	case DPMAC_ETH_IF_USXGMII:
		*if_mode = PHY_INTERFACE_MODE_USXGMII;
		break;
	case DPMAC_ETH_IF_QSGMII:
		*if_mode = PHY_INTERFACE_MODE_QSGMII;
		break;
	case DPMAC_ETH_IF_SGMII:
		*if_mode = PHY_INTERFACE_MODE_SGMII;
		break;
	case DPMAC_ETH_IF_XFI:
		*if_mode = PHY_INTERFACE_MODE_10GBASER;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static bool dpaa2_mac_is_type_phy(struct dpaa2_mac *mac)
{
	if (mac &&
	    (mac->attr.link_type == DPMAC_LINK_TYPE_PHY ||
	     mac->attr.link_type == DPMAC_LINK_TYPE_BACKPLANE))
		return true;

	return false;
}

static struct fwnode_handle *dpaa2_mac_get_node(struct device *dev,
						u16 dpmac_id)
{
	struct fwnode_handle *fwnode, *parent = NULL, *child  = NULL;
	struct device_node *dpmacs = NULL;
	int err;
	u32 id;

	fwnode = dev_fwnode(dev->parent);
	if (is_of_node(fwnode)) {
		dpmacs = of_find_node_by_name(NULL, "dpmacs");
		if (!dpmacs)
			return NULL;
		parent = of_fwnode_handle(dpmacs);
	} else if (is_acpi_node(fwnode)) {
		parent = fwnode;
	}

	if (!parent)
		return NULL;

	fwnode_for_each_child_node(parent, child) {
		err = -EINVAL;
		if (is_acpi_device_node(child))
			err = acpi_get_local_address(ACPI_HANDLE_FWNODE(child), &id);
		else if (is_of_node(child))
			err = of_property_read_u32(to_of_node(child), "reg", &id);
		if (err)
			continue;

		if (id == dpmac_id) {
			of_node_put(dpmacs);
			return child;
		}
	}
	of_node_put(dpmacs);

	return NULL;
}

static int dpaa2_mac_get_if_mode(struct fwnode_handle *dpmac_node,
				 struct dpmac_attr attr)
{
	phy_interface_t if_mode;
	int err;

	err = fwnode_get_phy_mode(dpmac_node);
	if (err > 0)
		return err;

	err = phy_mode(attr.eth_if, &if_mode);
	if (!err)
		return if_mode;

	return err;
}

static bool dpaa2_mac_phy_mode_mismatch(struct dpaa2_mac *mac,
					phy_interface_t interface)
{
	switch (interface) {
	/* We can switch between SGMII and 1000BASE-X at runtime with
	 * pcs-lynx
	 */
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		if (mac->pcs &&
		    (mac->if_mode == PHY_INTERFACE_MODE_SGMII ||
		     mac->if_mode == PHY_INTERFACE_MODE_1000BASEX))
			return false;
		return interface != mac->if_mode;

	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_QSGMII:
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		return (interface != mac->if_mode);
	default:
		return true;
	}
}

static void dpaa2_mac_validate(struct phylink_config *config,
			       unsigned long *supported,
			       struct phylink_link_state *state)
{
	struct dpaa2_mac *mac = phylink_to_dpaa2_mac(config);
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	if (state->interface != PHY_INTERFACE_MODE_NA &&
	    dpaa2_mac_phy_mode_mismatch(mac, state->interface)) {
		goto empty_set;
	}

	phylink_set_port_modes(mask);
	phylink_set(mask, Autoneg);
	phylink_set(mask, Pause);
	phylink_set(mask, Asym_Pause);

	switch (state->interface) {
	case PHY_INTERFACE_MODE_NA:
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_USXGMII:
		phylink_set(mask, 10000baseT_Full);
		phylink_set(mask, 10000baseCR_Full);
		phylink_set(mask, 10000baseSR_Full);
		phylink_set(mask, 10000baseLR_Full);
		phylink_set(mask, 10000baseLRM_Full);
		phylink_set(mask, 10000baseER_Full);
		if (state->interface == PHY_INTERFACE_MODE_10GBASER)
			break;
		phylink_set(mask, 5000baseT_Full);
		phylink_set(mask, 2500baseT_Full);
		fallthrough;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		phylink_set(mask, 1000baseX_Full);
		phylink_set(mask, 1000baseT_Full);
		if (state->interface == PHY_INTERFACE_MODE_1000BASEX)
			break;
		phylink_set(mask, 100baseT_Full);
		phylink_set(mask, 10baseT_Full);
		break;
	default:
		goto empty_set;
	}

	linkmode_and(supported, supported, mask);
	linkmode_and(state->advertising, state->advertising, mask);

	return;

empty_set:
	linkmode_zero(supported);
}

static void dpaa2_mac_config(struct phylink_config *config, unsigned int mode,
			     const struct phylink_link_state *state)
{
	struct dpaa2_mac *mac = phylink_to_dpaa2_mac(config);
	struct dpmac_link_state *dpmac_state = &mac->state;
	int err;

	if (state->an_enabled)
		dpmac_state->options |= DPMAC_LINK_OPT_AUTONEG;
	else
		dpmac_state->options &= ~DPMAC_LINK_OPT_AUTONEG;

	err = dpmac_set_link_state(mac->mc_io, 0,
				   mac->mc_dev->mc_handle, dpmac_state);
	if (err)
		netdev_err(mac->net_dev, "%s: dpmac_set_link_state() = %d\n",
			   __func__, err);
}

static void dpaa2_mac_link_up(struct phylink_config *config,
			      struct phy_device *phy,
			      unsigned int mode, phy_interface_t interface,
			      int speed, int duplex,
			      bool tx_pause, bool rx_pause)
{
	struct dpaa2_mac *mac = phylink_to_dpaa2_mac(config);
	struct dpmac_link_state *dpmac_state = &mac->state;
	int err;

	dpmac_state->up = 1;

	dpmac_state->rate = speed;

	if (duplex == DUPLEX_HALF)
		dpmac_state->options |= DPMAC_LINK_OPT_HALF_DUPLEX;
	else if (duplex == DUPLEX_FULL)
		dpmac_state->options &= ~DPMAC_LINK_OPT_HALF_DUPLEX;

	if (rx_pause)
		dpmac_state->options |= DPMAC_LINK_OPT_PAUSE;
	else
		dpmac_state->options &= ~DPMAC_LINK_OPT_PAUSE;

	if (rx_pause ^ tx_pause)
		dpmac_state->options |= DPMAC_LINK_OPT_ASYM_PAUSE;
	else
		dpmac_state->options &= ~DPMAC_LINK_OPT_ASYM_PAUSE;

	err = dpmac_set_link_state(mac->mc_io, 0,
				   mac->mc_dev->mc_handle, dpmac_state);
	if (err)
		netdev_err(mac->net_dev, "%s: dpmac_set_link_state() = %d\n",
			   __func__, err);
}

static void dpaa2_mac_link_down(struct phylink_config *config,
				unsigned int mode,
				phy_interface_t interface)
{
	struct dpaa2_mac *mac = phylink_to_dpaa2_mac(config);
	struct dpmac_link_state *dpmac_state = &mac->state;
	int err;

	dpmac_state->up = 0;
	err = dpmac_set_link_state(mac->mc_io, 0,
				   mac->mc_dev->mc_handle, dpmac_state);
	if (err)
		netdev_err(mac->net_dev, "dpmac_set_link_state() = %d\n", err);
}

static const struct phylink_mac_ops dpaa2_mac_phylink_ops = {
	.validate = dpaa2_mac_validate,
	.mac_config = dpaa2_mac_config,
	.mac_link_up = dpaa2_mac_link_up,
	.mac_link_down = dpaa2_mac_link_down,
};

static int dpaa2_pcs_create(struct dpaa2_mac *mac,
			    struct fwnode_handle *dpmac_node,
			    int id)
{
	struct mdio_device *mdiodev;
	struct fwnode_handle *node;

	node = fwnode_find_reference(dpmac_node, "pcs-handle", 0);
	if (IS_ERR(node)) {
		/* do not error out on old DTS files */
		netdev_warn(mac->net_dev, "pcs-handle node not found\n");
		return 0;
	}

	if (!fwnode_device_is_available(node)) {
		netdev_err(mac->net_dev, "pcs-handle node not available\n");
		fwnode_handle_put(node);
		return -ENODEV;
	}

	mdiodev = fwnode_mdio_find_device(node);
	fwnode_handle_put(node);
	if (!mdiodev)
		return -EPROBE_DEFER;

	mac->pcs = lynx_pcs_create(mdiodev);
	if (!mac->pcs) {
		netdev_err(mac->net_dev, "lynx_pcs_create() failed\n");
		put_device(&mdiodev->dev);
		return -ENOMEM;
	}

	return 0;
}

static void dpaa2_pcs_destroy(struct dpaa2_mac *mac)
{
	struct lynx_pcs *pcs = mac->pcs;

	if (pcs) {
		struct device *dev = &pcs->mdio->dev;
		lynx_pcs_destroy(pcs);
		put_device(dev);
		mac->pcs = NULL;
	}
}

int dpaa2_mac_connect(struct dpaa2_mac *mac)
{
	struct net_device *net_dev = mac->net_dev;
	struct fwnode_handle *dpmac_node;
	struct phylink *phylink;
	int err;

	mac->if_link_type = mac->attr.link_type;

	dpmac_node = mac->fw_node;
	if (!dpmac_node) {
		netdev_err(net_dev, "No dpmac@%d node found.\n", mac->attr.id);
		return -ENODEV;
	}

	err = dpaa2_mac_get_if_mode(dpmac_node, mac->attr);
	if (err < 0)
		return -EINVAL;
	mac->if_mode = err;

	/* The MAC does not have the capability to add RGMII delays so
	 * error out if the interface mode requests them and there is no PHY
	 * to act upon them
	 */
	if (of_phy_is_fixed_link(to_of_node(dpmac_node)) &&
	    (mac->if_mode == PHY_INTERFACE_MODE_RGMII_ID ||
	     mac->if_mode == PHY_INTERFACE_MODE_RGMII_RXID ||
	     mac->if_mode == PHY_INTERFACE_MODE_RGMII_TXID)) {
		netdev_err(net_dev, "RGMII delay not supported\n");
		return -EINVAL;
	}

	if ((mac->attr.link_type == DPMAC_LINK_TYPE_PHY &&
	     mac->attr.eth_if != DPMAC_ETH_IF_RGMII) ||
	    mac->attr.link_type == DPMAC_LINK_TYPE_BACKPLANE) {
		err = dpaa2_pcs_create(mac, dpmac_node, mac->attr.id);
		if (err)
			return err;
	}

	mac->phylink_config.dev = &net_dev->dev;
	mac->phylink_config.type = PHYLINK_NETDEV;

	phylink = phylink_create(&mac->phylink_config,
				 dpmac_node, mac->if_mode,
				 &dpaa2_mac_phylink_ops);
	if (IS_ERR(phylink)) {
		err = PTR_ERR(phylink);
		goto err_pcs_destroy;
	}
	mac->phylink = phylink;

	if (mac->pcs)
		phylink_set_pcs(mac->phylink, &mac->pcs->pcs);

	rtnl_lock();
	err = phylink_fwnode_phy_connect(mac->phylink, dpmac_node, 0);
	rtnl_unlock();
	if (err) {
		netdev_err(net_dev, "phylink_fwnode_phy_connect() = %d\n", err);
		goto err_phylink_destroy;
	}

	return 0;

err_phylink_destroy:
	phylink_destroy(mac->phylink);
err_pcs_destroy:
	dpaa2_pcs_destroy(mac);

	return err;
}

void dpaa2_mac_disconnect(struct dpaa2_mac *mac)
{
	if (!mac->phylink)
		return;

	rtnl_lock();
	phylink_disconnect_phy(mac->phylink);
	rtnl_unlock();
	phylink_destroy(mac->phylink);
	dpaa2_pcs_destroy(mac);
}

int dpaa2_mac_open(struct dpaa2_mac *mac)
{
	struct fsl_mc_device *dpmac_dev = mac->mc_dev;
	struct net_device *net_dev = mac->net_dev;
	int err;

	err = dpmac_open(mac->mc_io, 0, dpmac_dev->obj_desc.id,
			 &dpmac_dev->mc_handle);
	if (err || !dpmac_dev->mc_handle) {
		netdev_err(net_dev, "dpmac_open() = %d\n", err);
		return -ENODEV;
	}

	err = dpmac_get_attributes(mac->mc_io, 0, dpmac_dev->mc_handle,
				   &mac->attr);
	if (err) {
		netdev_err(net_dev, "dpmac_get_attributes() = %d\n", err);
		goto err_close_dpmac;
	}

	/* Find the device node representing the MAC device and link the device
	 * behind the associated netdev to it.
	 */
	mac->fw_node = dpaa2_mac_get_node(&mac->mc_dev->dev, mac->attr.id);
	net_dev->dev.of_node = to_of_node(mac->fw_node);

	return 0;

err_close_dpmac:
	dpmac_close(mac->mc_io, 0, dpmac_dev->mc_handle);
	return err;
}

void dpaa2_mac_close(struct dpaa2_mac *mac)
{
	struct fsl_mc_device *dpmac_dev = mac->mc_dev;

	dpmac_close(mac->mc_io, 0, dpmac_dev->mc_handle);
	if (mac->fw_node)
		fwnode_handle_put(mac->fw_node);
}

static char dpaa2_mac_ethtool_stats[][ETH_GSTRING_LEN] = {
	[DPMAC_CNT_ING_ALL_FRAME]		= "[mac] rx all frames",
	[DPMAC_CNT_ING_GOOD_FRAME]		= "[mac] rx frames ok",
	[DPMAC_CNT_ING_ERR_FRAME]		= "[mac] rx frame errors",
	[DPMAC_CNT_ING_FRAME_DISCARD]		= "[mac] rx frame discards",
	[DPMAC_CNT_ING_UCAST_FRAME]		= "[mac] rx u-cast",
	[DPMAC_CNT_ING_BCAST_FRAME]		= "[mac] rx b-cast",
	[DPMAC_CNT_ING_MCAST_FRAME]		= "[mac] rx m-cast",
	[DPMAC_CNT_ING_FRAME_64]		= "[mac] rx 64 bytes",
	[DPMAC_CNT_ING_FRAME_127]		= "[mac] rx 65-127 bytes",
	[DPMAC_CNT_ING_FRAME_255]		= "[mac] rx 128-255 bytes",
	[DPMAC_CNT_ING_FRAME_511]		= "[mac] rx 256-511 bytes",
	[DPMAC_CNT_ING_FRAME_1023]		= "[mac] rx 512-1023 bytes",
	[DPMAC_CNT_ING_FRAME_1518]		= "[mac] rx 1024-1518 bytes",
	[DPMAC_CNT_ING_FRAME_1519_MAX]		= "[mac] rx 1519-max bytes",
	[DPMAC_CNT_ING_FRAG]			= "[mac] rx frags",
	[DPMAC_CNT_ING_JABBER]			= "[mac] rx jabber",
	[DPMAC_CNT_ING_ALIGN_ERR]		= "[mac] rx align errors",
	[DPMAC_CNT_ING_OVERSIZED]		= "[mac] rx oversized",
	[DPMAC_CNT_ING_VALID_PAUSE_FRAME]	= "[mac] rx pause",
	[DPMAC_CNT_ING_BYTE]			= "[mac] rx bytes",
	[DPMAC_CNT_EGR_GOOD_FRAME]		= "[mac] tx frames ok",
	[DPMAC_CNT_EGR_UCAST_FRAME]		= "[mac] tx u-cast",
	[DPMAC_CNT_EGR_MCAST_FRAME]		= "[mac] tx m-cast",
	[DPMAC_CNT_EGR_BCAST_FRAME]		= "[mac] tx b-cast",
	[DPMAC_CNT_EGR_ERR_FRAME]		= "[mac] tx frame errors",
	[DPMAC_CNT_EGR_UNDERSIZED]		= "[mac] tx undersized",
	[DPMAC_CNT_EGR_VALID_PAUSE_FRAME]	= "[mac] tx b-pause",
	[DPMAC_CNT_EGR_BYTE]			= "[mac] tx bytes",
};

#define DPAA2_MAC_NUM_STATS	ARRAY_SIZE(dpaa2_mac_ethtool_stats)

int dpaa2_mac_get_sset_count(void)
{
	return DPAA2_MAC_NUM_STATS;
}

void dpaa2_mac_get_strings(u8 *data)
{
	u8 *p = data;
	int i;

	for (i = 0; i < DPAA2_MAC_NUM_STATS; i++) {
		strlcpy(p, dpaa2_mac_ethtool_stats[i], ETH_GSTRING_LEN);
		p += ETH_GSTRING_LEN;
	}
}

void dpaa2_mac_get_ethtool_stats(struct dpaa2_mac *mac, u64 *data)
{
	struct fsl_mc_device *dpmac_dev = mac->mc_dev;
	int i, err;
	u64 value;

	for (i = 0; i < DPAA2_MAC_NUM_STATS; i++) {
		err = dpmac_get_counter(mac->mc_io, 0, dpmac_dev->mc_handle,
					i, &value);
		if (err) {
			netdev_err_once(mac->net_dev,
					"dpmac_get_counter error %d\n", err);
			*(data + i) = U64_MAX;
			continue;
		}
		*(data + i) = value;
	}
}

struct dpaa2_mac_link_mode_map {
	u64 dpmac_lm;
	enum ethtool_link_mode_bit_indices ethtool_lm;
};

static const struct dpaa2_mac_link_mode_map dpaa2_mac_lm_map[] = {
	{DPMAC_ADVERTISED_10BASET_FULL, ETHTOOL_LINK_MODE_10baseT_Full_BIT},
	{DPMAC_ADVERTISED_100BASET_FULL, ETHTOOL_LINK_MODE_100baseT_Full_BIT},
	{DPMAC_ADVERTISED_1000BASET_FULL, ETHTOOL_LINK_MODE_1000baseT_Full_BIT},
	{DPMAC_ADVERTISED_10000BASET_FULL, ETHTOOL_LINK_MODE_10000baseT_Full_BIT},
	{DPMAC_ADVERTISED_AUTONEG, ETHTOOL_LINK_MODE_Autoneg_BIT},
};

static void dpaa2_mac_ksettings_change(struct dpaa2_mac *priv)
{
	struct fsl_mc_device *mc_dev = priv->mc_dev;
	struct dpmac_link_cfg link_cfg = { 0 };
	int err, i;

	err = dpmac_get_link_cfg(priv->mc_io, 0,
				 mc_dev->mc_handle,
				 &link_cfg);
	if (err) {
		dev_err(&mc_dev->dev, "dpmac_get_link_cfg() = %d\n", err);
		return;
	}

	/* There are some circumstances (MC bugs) when the advertising is all
	 * zeroes. Just skip any configuration if this happens.
	 */
	if (!link_cfg.advertising)
		return;

	phylink_ethtool_ksettings_get(priv->phylink, &priv->kset);

	priv->kset.base.speed = link_cfg.rate;
	priv->kset.base.duplex = !!(link_cfg.options & DPMAC_LINK_OPT_HALF_DUPLEX);

	ethtool_link_ksettings_zero_link_mode(&priv->kset, advertising);
	for (i = 0; i < ARRAY_SIZE(dpaa2_mac_lm_map); i++) {
		if (link_cfg.advertising & dpaa2_mac_lm_map[i].dpmac_lm)
			__set_bit(dpaa2_mac_lm_map[i].ethtool_lm,
				  priv->kset.link_modes.advertising);
	}

	if (link_cfg.options & DPMAC_LINK_OPT_AUTONEG) {
		priv->kset.base.autoneg = AUTONEG_ENABLE;
		__set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT,
			  priv->kset.link_modes.advertising);
	} else {
		priv->kset.base.autoneg = AUTONEG_DISABLE;
		__clear_bit(ETHTOOL_LINK_MODE_Autoneg_BIT,
			    priv->kset.link_modes.advertising);
	}

	phylink_ethtool_ksettings_set(priv->phylink, &priv->kset);
}

static irqreturn_t dpaa2_mac_irq_handler(int irq_num, void *arg)
{
	struct device *dev = (struct device *)arg;
	struct fsl_mc_device *dpmac_dev;
	struct net_device *net_dev;
	struct dpaa2_mac *priv;
	u32 status = ~0;
	int err;

	dpmac_dev = to_fsl_mc_device(dev);
	net_dev = dev_get_drvdata(dev);
	priv = netdev_priv(net_dev);

	err = dpmac_get_irq_status(priv->mc_io, 0, dpmac_dev->mc_handle,
				  DPMAC_IRQ_INDEX, &status);
	if (err) {
		netdev_err(net_dev, "dpmac_get_irq_status() = %d\n", err);
		return IRQ_HANDLED;
	}

	rtnl_lock();
	if (status & DPMAC_IRQ_EVENT_LINK_CFG_REQ)
		dpaa2_mac_ksettings_change(priv);

	if (status & DPMAC_IRQ_EVENT_LINK_DOWN_REQ)
		phylink_stop(priv->phylink);

	if (status & DPMAC_IRQ_EVENT_LINK_UP_REQ)
		phylink_start(priv->phylink);
	rtnl_unlock();

	dpmac_clear_irq_status(priv->mc_io, 0, dpmac_dev->mc_handle,
			       DPMAC_IRQ_INDEX, status);

	return IRQ_HANDLED;
}

static int dpaa2_mac_setup_irqs(struct fsl_mc_device *mc_dev)
{
	struct fsl_mc_device_irq *irq;
	int err = 0;

	err = fsl_mc_allocate_irqs(mc_dev);
	if (err) {
		dev_err(&mc_dev->dev, "fsl_mc_allocate_irqs err %d\n", err);
		return err;
	}

	irq = mc_dev->irqs[0];
	err = devm_request_threaded_irq(&mc_dev->dev, irq->msi_desc->irq,
					NULL, &dpaa2_mac_irq_handler,
					IRQF_NO_SUSPEND | IRQF_ONESHOT,
					dev_name(&mc_dev->dev), &mc_dev->dev);
	if (err) {
		dev_err(&mc_dev->dev, "devm_request_threaded_irq err %d\n",
			err);
		goto free_irq;
	}

	err = dpmac_set_irq_mask(mc_dev->mc_io, 0, mc_dev->mc_handle,
				 DPMAC_IRQ_INDEX, DPMAC_IRQ_EVENT_LINK_CFG_REQ |
				 DPMAC_IRQ_EVENT_LINK_UP_REQ |
				 DPMAC_IRQ_EVENT_LINK_DOWN_REQ);
	if (err) {
		dev_err(&mc_dev->dev, "dpmac_set_irq_mask err %d\n", err);
		goto free_irq;
	}
	err = dpmac_set_irq_enable(mc_dev->mc_io, 0, mc_dev->mc_handle,
				   DPMAC_IRQ_INDEX, 1);
	if (err) {
		dev_err(&mc_dev->dev, "dpmac_set_irq_enable err %d\n", err);
		goto free_irq;
	}

	return 0;

free_irq:
	fsl_mc_free_irqs(mc_dev);

	return err;
}

static void dpaa2_mac_teardown_irqs(struct fsl_mc_device *mc_dev)
{
	int err;

	err = dpmac_set_irq_enable(mc_dev->mc_io, 0, mc_dev->mc_handle,
				   DPMAC_IRQ_INDEX, 0);
	if (err)
		dev_err(&mc_dev->dev, "dpmac_set_irq_enable err %d\n", err);

	fsl_mc_free_irqs(mc_dev);
}

#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS

static int dpaa2_mac_netdev_open(struct net_device *net_dev)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	if (dpaa2_mac_is_type_phy(priv))
		phylink_start(priv->phylink);

	return 0;
}

static int dpaa2_mac_netdev_stop(struct net_device *net_dev)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	if (!dpaa2_mac_is_type_phy(priv))
		return 0;

	phylink_stop(priv->phylink);

	return 0;
}

static netdev_tx_t dpaa2_mac_drop_frame(struct sk_buff *skb,
					struct net_device *net_dev)
{
	/* These interfaces don't support I/O, they are only
	 * for control and debu
	 */
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static void dpaa2_mac_get_drvinfo(struct net_device *net_dev,
				  struct ethtool_drvinfo *drvinfo)
{
	strlcpy(drvinfo->driver, KBUILD_MODNAME, sizeof(drvinfo->driver));
	strlcpy(drvinfo->bus_info, dev_name(net_dev->dev.parent->parent),
		sizeof(drvinfo->bus_info));
}

static int dpaa2_mac_get_link_ksettings(struct net_device *net_dev,
					struct ethtool_link_ksettings *ks)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	if (dpaa2_mac_is_type_phy(priv))
		return phylink_ethtool_ksettings_get(priv->phylink, ks);

	return -EOPNOTSUPP;
}

static int dpaa2_mac_set_link_ksettings(struct net_device *net_dev,
					const struct ethtool_link_ksettings *ks)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	if (!dpaa2_mac_is_type_phy(priv))
		return -EOPNOTSUPP;

	return phylink_ethtool_ksettings_set(priv->phylink, ks);
}

static int dpaa2_mac_ioctl(struct net_device *net_dev, struct ifreq *rq, int cmd)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	if (!dpaa2_mac_is_type_phy(priv))
		return -EOPNOTSUPP;

	return phylink_mii_ioctl(priv->phylink, rq, cmd);
}

static void dpaa2_mac_ethtool_get_strings(struct net_device *net_dev,
					  u32 stringset, u8 *data)
{
	if (stringset != ETH_SS_STATS)
		return;

	dpaa2_mac_get_strings(data);
}

static void dpaa2_mac_ethtool_get_stats(struct net_device *net_dev,
					struct ethtool_stats *stats,
					u64 *data)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	dpaa2_mac_get_ethtool_stats(priv, data);
}

static int dpaa2_mac_ethtool_get_sset_count(struct net_device *dev, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	return dpaa2_mac_get_sset_count();
}

static const struct net_device_ops dpaa2_mac_ndo_ops = {
	.ndo_open		= &dpaa2_mac_netdev_open,
	.ndo_stop		= &dpaa2_mac_netdev_stop,
	.ndo_start_xmit		= &dpaa2_mac_drop_frame,
	.ndo_do_ioctl		= &dpaa2_mac_ioctl,
};

static const struct ethtool_ops dpaa2_mac_ethtool_ops = {
	.get_drvinfo		= &dpaa2_mac_get_drvinfo,
	.get_link_ksettings	= &dpaa2_mac_get_link_ksettings,
	.set_link_ksettings	= &dpaa2_mac_set_link_ksettings,
	.get_strings		= &dpaa2_mac_ethtool_get_strings,
	.get_ethtool_stats	= &dpaa2_mac_ethtool_get_stats,
	.get_sset_count		= &dpaa2_mac_ethtool_get_sset_count,
};
#endif

static int dpaa2_mac_probe(struct fsl_mc_device *mc_dev)
{
	struct device *dev = &mc_dev->dev;
	struct fsl_mc_device *peer_dev;
	struct net_device *net_dev;
	struct dpaa2_mac *priv;
	int err;

	/* If the DPMAC is connected to a DPNI from the currect DPRC, then
	 * there is no need for this standalone MAC driver, the dpaa2-eth will
	 * take care of anything related to MAC/PHY
	 */
	peer_dev = fsl_mc_get_endpoint(mc_dev);
	if (!IS_ERR_OR_NULL(peer_dev) && peer_dev->dev.type == &fsl_mc_bus_dpni_type)
		return -EPROBE_DEFER;

	net_dev = alloc_etherdev(sizeof(*priv));
	if (!net_dev) {
		dev_err(dev, "alloc_etherdev error\n");
		return -ENOMEM;
	}

	priv = netdev_priv(net_dev);
	priv->mc_dev = mc_dev;
	priv->net_dev = net_dev;

	SET_NETDEV_DEV(net_dev, dev);

#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
	snprintf(net_dev->name, IFNAMSIZ, "mac%d", mc_dev->obj_desc.id);

	/* register netdev just to make it visible to the user */
	net_dev->netdev_ops = &dpaa2_mac_ndo_ops;
	net_dev->ethtool_ops = &dpaa2_mac_ethtool_ops;

	err = register_netdev(net_dev);
	if (err) {
		dev_err(dev, "register_netdev error %d\n", err);
		goto free_netdev;
	}
#endif

	dev_set_drvdata(dev, net_dev);

	err = fsl_mc_portal_allocate(mc_dev, 0, &mc_dev->mc_io);
	if (err) {
		if (err == -ENXIO)
			err = -EPROBE_DEFER;
		else
			dev_err(dev, "fsl_mc_portal_allocate err %d\n", err);
		goto unregister_netdev;
	}
	priv->mc_io = mc_dev->mc_io;

	err = dpmac_open(mc_dev->mc_io, 0, mc_dev->obj_desc.id,
			 &mc_dev->mc_handle);
	if (err || !mc_dev->mc_handle) {
		dev_err(dev, "dpmac_open error: %d\n", err);
		err = -ENODEV;
		goto free_portal;
	}

	err = dpmac_get_attributes(mc_dev->mc_io, 0, mc_dev->mc_handle, &priv->attr);
	if (err) {
		dev_err(dev, "dpmac_get_attributes() = %d\n", err);
		goto free_portal;
	}

	err = dpaa2_mac_setup_irqs(mc_dev);
	if (err) {
		err = -EFAULT;
		goto free_portal;
	}

	err = dpaa2_mac_open(priv);
	if (err)
		goto free_portal;

	if (dpaa2_mac_is_type_phy(priv)) {
		err = dpaa2_mac_connect(priv);
		if (err) {
			dev_err(dev, "Error connecting to the MAC endpoint\n");
			goto free_portal;
		}
	}

	return 0;
free_portal:
	fsl_mc_portal_free(mc_dev->mc_io);
unregister_netdev:
#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
	unregister_netdev(net_dev);
free_netdev:
#endif
	free_netdev(net_dev);

	return err;
}

static int dpaa2_mac_remove(struct fsl_mc_device *mc_dev)
{
	struct device *dev = &mc_dev->dev;
	struct net_device *net_dev = dev_get_drvdata(dev);
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	dpaa2_mac_teardown_irqs(mc_dev);

	if (dpaa2_mac_is_type_phy(priv)) {
		dpaa2_mac_close(priv);
		dpaa2_mac_disconnect(priv);
	}

	fsl_mc_portal_free(mc_dev->mc_io);
	dev_set_drvdata(dev, NULL);
#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
	unregister_netdev(net_dev);
#endif
	free_netdev(net_dev);

	return 0;
}

static const struct fsl_mc_device_id dpaa2_mac_match_id_table[] = {
	{
		.vendor = FSL_MC_VENDOR_FREESCALE,
		.obj_type = "dpmac",
	},
	{ .vendor = 0x0 }
};
MODULE_DEVICE_TABLE(fslmc, dpaa2_mac_match_id_table);

static struct fsl_mc_driver dpaa2_mac_driver = {
	.driver = {
		.name = "fsl_dpaa2_mac",
		.owner = THIS_MODULE,
	},
	.probe = dpaa2_mac_probe,
	.remove = dpaa2_mac_remove,
	.match_id_table = dpaa2_mac_match_id_table
};

module_fsl_mc_driver(dpaa2_mac_driver);
