// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC switch driver
 * Copyright 2025 NXP
 */

#include <linux/clk.h>
#include <linux/etherdevice.h>
#include <linux/fsl/enetc_mdio.h>
#include <linux/if_bridge.h>
#include <linux/if_hsr.h>
#include <linux/if_vlan.h>
#include <linux/of_mdio.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/unaligned.h>

#include "netc_switch.h"

#define NETC_SUPPORTED_HSR_FEATURES \
	(NETIF_F_HW_HSR_TAG_INS | NETIF_F_HW_HSR_TAG_RM | \
	 NETIF_F_HW_HSR_FWD | NETIF_F_HW_HSR_DUP)

static struct netc_fdb_entry *netc_lookup_fdb_entry(struct netc_switch *priv,
						    const unsigned char *addr,
						    u16 vid)
{
	struct netc_fdb_entry *entry;

	hlist_for_each_entry(entry, &priv->fdb_list, node)
		if (ether_addr_equal(entry->keye.mac_addr, addr) &&
		    le16_to_cpu(entry->keye.fid) == vid)
			return entry;

	return NULL;
}

void netc_destroy_fdb_list(struct netc_switch *priv)
{
	struct netc_fdb_entry *entry;
	struct hlist_node *tmp;

	hlist_for_each_entry_safe(entry, tmp, &priv->fdb_list, node)
		netc_del_fdb_entry(entry);
}

static struct netc_vlan_entry *netc_lookup_vlan_entry(struct netc_switch *priv,
						      u16 vid)
{
	struct netc_vlan_entry *entry;

	hlist_for_each_entry(entry, &priv->vlan_list, node)
		if (entry->vid == vid)
			return entry;

	return NULL;
}

void netc_destroy_vlan_list(struct netc_switch *priv)
{
	struct netc_vlan_entry *entry;
	struct hlist_node *tmp;

	hlist_for_each_entry_safe(entry, tmp, &priv->vlan_list, node)
		netc_del_vlan_entry(entry);
}

static enum dsa_tag_protocol netc_get_tag_protocol(struct dsa_switch *ds, int port,
						   enum dsa_tag_protocol mprot)
{
	struct netc_switch *priv = ds->priv;

	return priv->tag_proto;
}

static void netc_twostep_tstamp_handler(struct dsa_switch *ds, int port_id,
					u8 ts_req_id, u64 ts)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	struct sk_buff *skb, *skb_tmp, *skb_match = NULL;
	struct netc_switch *priv = port->switch_priv;
	struct skb_shared_hwtstamps hwtstamps;

	spin_lock(&port->ts_req_id_lock);

	skb_queue_walk_safe(&port->skb_txtstamp_queue, skb, skb_tmp) {
		if (NETC_SKB_CB(skb)->ts_req_id != ts_req_id)
			continue;

		__skb_unlink(skb, &port->skb_txtstamp_queue);
		skb_match = skb;
		break;
	}

	spin_unlock(&port->ts_req_id_lock);

	if (!skb_match) {
		dev_dbg_ratelimited(priv->dev,
				    "Port %d received an expired Tx timestamp response (ts_req_id %u)",
				    port_id, ts_req_id);
		return;
	}

	hwtstamps.hwtstamp = ns_to_ktime(ts);
	skb_complete_tx_timestamp(skb_match, &hwtstamps);
}

static int netc_connect_tag_protocol(struct dsa_switch *ds,
				     enum dsa_tag_protocol proto)
{
	struct netc_tagger_data *tagger_data;
	struct netc_switch *priv = ds->priv;

	if (proto != priv->tag_proto)
		return -EPROTONOSUPPORT;

	tagger_data = ds->tagger_data;
	tagger_data->twostep_tstamp_handler = netc_twostep_tstamp_handler;

	return 0;
}

void netc_mac_port_wr(struct netc_port *port, u32 reg, u32 val)
{
	if (is_netc_pseudo_port(port))
		return;

	netc_port_wr(port, reg, val);
	if (port->caps.pmac)
		netc_port_wr(port, reg + NETC_PMAC_OFFSET, val);
}

u32 netc_mac_port_rd(struct netc_port *port, u32 reg)
{
	if (is_netc_pseudo_port(port))
		return 0;

	return netc_port_rd(port, reg);
}

static void netc_switch_get_capabilities(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	u32 val;

	val = netc_base_rd(regs, NETC_BPCAPR);
	priv->caps.num_bp = BPCAPR_GET_NUM_BP(val);
	priv->caps.num_sbp = BPCAPR_GET_NUM_SBP(val);
}

static void netc_port_get_capability(struct netc_port *port)
{
	u32 val;

	val = netc_port_rd(port, NETC_PMCAPR);
	if (val & PMCAPR_HD)
		port->caps.half_duplex = true;

	if (FIELD_GET(PMCAPR_FP, val) == FP_SUPPORT)
		port->caps.pmac = true;

	val = netc_port_rd(port, NETC_PCAPR);
	if (val & PCAPR_LINK_TYPE)
		port->caps.pseudo_link = true;
}

static int netc_port_get_index_from_dt(struct device_node *node,
				       struct device *dev, u32 *index)
{
	/* Get switch port number from DT */
	if (of_property_read_u32(node, "reg", index) < 0) {
		dev_err(dev, "The reg property isn't defined in DT node\n");
		of_node_put(node);
		return -ENODEV;
	}

	return 0;
}

static int netc_port_get_info_from_dt(struct netc_port *port,
				      struct device_node *node,
				      struct device *dev)
{
	phy_interface_t phy_mode;
	int err;

	/* Get PHY mode from DT */
	err = of_get_phy_mode(node, &phy_mode);
	if (err) {
		dev_err(dev, "Failed to get phy mode for port %d\n",
			port->index);
		of_node_put(node);
		return err;
	}

	if (of_find_property(node, "clock-names", NULL)) {
		port->ref_clk = devm_get_clk_from_child(dev, node, "ref");
		if (IS_ERR(port->ref_clk)) {
			dev_err(dev, "Port %d cannot get reference clock\n", port->index);
			return PTR_ERR(port->ref_clk);
		}
	}

	port->phy_mode = phy_mode;

	return 0;
}

static bool netc_port_has_pcs(phy_interface_t phy_mode)
{
	return (phy_mode == PHY_INTERFACE_MODE_SGMII ||
		phy_mode == PHY_INTERFACE_MODE_2500BASEX);
}

static int netc_port_create_internal_mdiobus(struct netc_port *port)
{
	struct netc_switch *priv = port->switch_priv;
	struct enetc_mdio_priv *mdio_priv;
	struct device *dev = priv->dev;
	void __iomem *port_iobase;
	struct phylink_pcs *pcs;
	struct enetc_hw *hw;
	struct mii_bus *bus;
	int err, xpcs_ver;

	port_iobase = port->iobase;
	hw = enetc_hw_alloc(dev, port_iobase);
	if (IS_ERR(hw)) {
		dev_err(dev, "Failed to allocate ENETC HW structure\n");
		return PTR_ERR(hw);
	}

	bus = mdiobus_alloc_size(sizeof(*mdio_priv));
	if (!bus)
		return -ENOMEM;

	bus->name = "NXP NETC Switch internal MDIO Bus";
	bus->read = enetc_mdio_read_c22;
	bus->write = enetc_mdio_write_c22;
	bus->read_c45 = enetc_mdio_read_c45;
	bus->write_c45 = enetc_mdio_write_c45;
	bus->parent = dev;
	bus->phy_mask = ~0;
	mdio_priv = bus->priv;
	mdio_priv->hw = hw;
	mdio_priv->mdio_base = NETC_IMDIO_BASE;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-p%d-imdio",
		 dev_name(dev), port->index);

	err = mdiobus_register(bus);
	if (err) {
		dev_err(dev, "Failed to register internal MDIO bus (%d)\n",
			err);
		goto free_mdiobus;
	}

	switch (priv->revision) {
	case NETC_SWITCH_REV_4_3:
		xpcs_ver = DW_XPCS_VER_MX94;
		break;
	default:
		err = -EOPNOTSUPP;
		dev_err(dev, "unsupported xpcs version\n");
		goto unregister_mdiobus;
	}

	netc_xpcs_port_init(port->index);
	pcs = xpcs_create_mdiodev_with_phy(bus, 0, 16, port->index, xpcs_ver,
					   port->phy_mode);
	if (IS_ERR(pcs)) {
		err = PTR_ERR(pcs);
		dev_err(dev, "cannot create xpcs mdiodev (%d)\n", err);
		goto unregister_mdiobus;
	}

	port->imdio = bus;
	port->pcs = pcs;

	return 0;

unregister_mdiobus:
	mdiobus_unregister(bus);
free_mdiobus:
	mdiobus_free(bus);

	return err;
}

static void netc_port_remove_internal_mdiobus(struct netc_port *port)
{
	struct phylink_pcs *pcs = port->pcs;
	struct mii_bus *bus = port->imdio;

	if (pcs)
		xpcs_pcs_destroy(pcs);

	if (bus) {
		mdiobus_unregister(bus);
		mdiobus_free(bus);
	}
}

static void netc_remove_all_ports_internal_mdiobus(struct dsa_switch *ds)
{
	struct netc_switch *priv = ds->priv;
	int i;

	for (i = 0; i < ds->num_ports; i++) {
		struct netc_port *port = priv->ports[i];

		if (!is_netc_pseudo_port(port) &&
		    netc_port_has_pcs(port->phy_mode))
			netc_port_remove_internal_mdiobus(port);
	}
}

static void netc_port_init_ptp_ipft_eid(struct netc_port *port)
{
	int i;

	for (i = 0; i < NETC_PTP_MAX; i++)
		port->ptp_ipft_eid[i] = NTMP_NULL_ENTRY_ID;
}

static int netc_init_all_ports(struct dsa_switch *ds)
{
	struct device_node *switch_node, *ports;
	struct netc_switch *priv = ds->priv;
	struct device *dev = priv->dev;
	struct netc_port *port;
	struct dsa_port *dp;
	int i, err;

	priv->ports = devm_kcalloc(dev, ds->num_ports,
				   sizeof(struct netc_port *),
				   GFP_KERNEL);
	if (!priv->ports)
		return -ENOMEM;

	for (i = 0; i < ds->num_ports; i++) {
		port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
		if (!port)
			return -ENOMEM;

		port->index = i;
		port->switch_priv = priv;
		port->iobase = priv->regs.port + PORT_IOBASE(i);
		netc_port_init_ptp_ipft_eid(port);
		priv->ports[i] = port;

		netc_port_get_capability(port);

		if (port->caps.pmac)
			mutex_init(&port->mm_lock);

		if (!port->caps.pseudo_link) {
			spin_lock_init(&port->ts_req_id_lock);
			skb_queue_head_init(&port->skb_txtstamp_queue);
		}
	}

	switch_node = dev->of_node;
	ports = of_get_child_by_name(switch_node, "ports");
	if (!ports)
		ports = of_get_child_by_name(switch_node, "ethernet-ports");
	if (!ports) {
		dev_err(dev, "No ports or ethernet-ports child node in switch node\n");
		return -ENODEV;
	}

	for_each_available_child_of_node_scoped(ports, child) {
		u32 index;

		err = netc_port_get_index_from_dt(child, dev, &index);
		if (err < 0)
			goto remove_internal_mdiobus;

		port = priv->ports[index];
		err = netc_port_get_info_from_dt(port, child, dev);
		if (err)
			goto remove_internal_mdiobus;

		dp = dsa_to_port(ds, index);
		if (!dp) {
			err = -ENODEV;
			goto remove_internal_mdiobus;
		}

		port->dp = dp;
		if (!is_netc_pseudo_port(port) &&
		    netc_port_has_pcs(port->phy_mode)) {
			err = netc_port_create_internal_mdiobus(port);
			if (err)
				goto remove_internal_mdiobus;
		}
	}

	of_node_put(ports);

	return 0;

remove_internal_mdiobus:
	of_node_put(ports);
	netc_remove_all_ports_internal_mdiobus(ds);

	return err;
}

static void netc_init_ntmp_tbl_versions(struct netc_switch *priv)
{
	struct ntmp_user *user = &priv->user;

	/* All tables default to version 0 */
	memset(&user->tbl, 0, sizeof(user->tbl));

	if (priv->revision == NETC_SWITCH_REV_4_3)
		user->tbl.ist_ver = 1;
}

static int netc_init_all_cbdrs(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	struct ntmp_user *user = &priv->user;
	int i, j, err;

	user->cbdr_num = NETC_CBDR_NUM;
	user->dev = priv->dev;
	user->ring = kcalloc(user->cbdr_num, sizeof(struct netc_cbdr),
			     GFP_KERNEL);
	if (!user->ring)
		return -ENOMEM;

	/* Set the system attributes of reads and writes of command
	 * descriptor and data.
	 */
	netc_base_wr(regs, NETC_CCAR, NETC_DEFAULT_CMD_CACHE_ATTR);

	for (i = 0; i < user->cbdr_num; i++) {
		struct netc_cbdr *cbdr = &user->ring[i];
		struct netc_cbdr_regs cbdr_regs;

		cbdr_regs.pir = regs->base + NETC_CBDRPIR(i);
		cbdr_regs.cir = regs->base + NETC_CBDRCIR(i);
		cbdr_regs.mr = regs->base + NETC_CBDRMR(i);
		cbdr_regs.bar0 = regs->base + NETC_CBDRBAR0(i);
		cbdr_regs.bar1 = regs->base + NETC_CBDRBAR1(i);
		cbdr_regs.lenr = regs->base + NETC_CBDRLENR(i);

		err = ntmp_init_cbdr(cbdr, user->dev, &cbdr_regs);
		if (err) {
			for (j = 0; j < i; j++)
				ntmp_free_cbdr(&user->ring[j]);

			kfree(user->ring);
			user->dev = NULL;

			return err;
		}
	}

	return 0;
}

static void netc_remove_all_cbdrs(struct netc_switch *priv)
{
	struct ntmp_user *user = &priv->user;
	int i;

	for (i = 0; i < NETC_CBDR_NUM; i++)
		ntmp_free_cbdr(&user->ring[i]);

	kfree(user->ring);
	user->dev = NULL;
}

static void netc_get_ntmp_capabilities(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	struct ntmp_user *user = &priv->user;
	u32 val;

	val = netc_base_rd(regs, NETC_ETTCAPR);
	user->caps.ett_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_ECTCAPR);
	user->caps.ect_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_RPITCAPR);
	user->caps.rpt_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_ISCITCAPR);
	user->caps.isct_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_ISITCAPR);
	user->caps.ist_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_SGIITCAPR);
	user->caps.sgit_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_SGCLITCAPR);
	user->caps.sgclt_num_words = NETC_GET_NUM_WORDS(val);

	val = netc_base_rd(regs, NETC_ISQGITCAPR);
	user->caps.isgt_num_entries = NETC_GET_NUM_ENTRIES(val);
}

static int netc_init_ntmp_bitmaps(struct netc_switch *priv)
{
	struct ntmp_user *user = &priv->user;

	user->ett_bitmap_size = user->caps.ett_num_entries / priv->num_ports;
	user->ett_gid_bitmap = bitmap_zalloc(user->ett_bitmap_size, GFP_KERNEL);
	if (!user->ett_gid_bitmap)
		return -ENOMEM;

	user->ect_bitmap_size = user->caps.ect_num_entries / priv->num_ports;
	user->ect_gid_bitmap = bitmap_zalloc(user->ect_bitmap_size, GFP_KERNEL);
	if (!user->ect_gid_bitmap)
		goto free_ett_gid_bitmap;

	user->ist_eid_bitmap = bitmap_zalloc(user->caps.ist_num_entries,
					     GFP_KERNEL);
	if (!user->ist_eid_bitmap)
		goto free_ect_gid_bitmap;

	user->rpt_eid_bitmap = bitmap_zalloc(user->caps.rpt_num_entries,
					     GFP_KERNEL);
	if (!user->rpt_eid_bitmap)
		goto free_ist_eid_bitmap;

	user->sgit_eid_bitmap = bitmap_zalloc(user->caps.sgit_num_entries,
					      GFP_KERNEL);
	if (!user->sgit_eid_bitmap)
		goto free_rpt_eid_bitmap;

	user->isct_eid_bitmap = bitmap_zalloc(user->caps.isct_num_entries,
					      GFP_KERNEL);
	if (!user->isct_eid_bitmap)
		goto free_sgit_eid_bitmap;

	user->sgclt_word_bitmap = bitmap_zalloc(user->caps.sgclt_num_words,
						GFP_KERNEL);
	if (!user->sgclt_word_bitmap)
		goto free_isct_eid_bitmap;

	user->isgt_eid_bitmap = bitmap_zalloc(user->caps.isgt_num_entries,
					      GFP_KERNEL);
	if (!user->isgt_eid_bitmap)
		goto free_sgclt_word_bitmap;

	return 0;

free_sgclt_word_bitmap:
	bitmap_free(user->sgclt_word_bitmap);
	user->sgclt_word_bitmap = NULL;
free_isct_eid_bitmap:
	bitmap_free(user->isct_eid_bitmap);
	user->isct_eid_bitmap = NULL;
free_sgit_eid_bitmap:
	bitmap_free(user->sgit_eid_bitmap);
	user->sgit_eid_bitmap = NULL;
free_rpt_eid_bitmap:
	bitmap_free(user->rpt_eid_bitmap);
	user->rpt_eid_bitmap = NULL;
free_ist_eid_bitmap:
	bitmap_free(user->ist_eid_bitmap);
	user->ist_eid_bitmap = NULL;
free_ect_gid_bitmap:
	bitmap_free(user->ect_gid_bitmap);
	user->ect_gid_bitmap = NULL;
free_ett_gid_bitmap:
	bitmap_free(user->ett_gid_bitmap);
	user->ett_gid_bitmap = NULL;

	return -ENOMEM;
}

static void netc_free_ntmp_bitmaps(struct netc_switch *priv)
{
	struct ntmp_user *user = &priv->user;

	bitmap_free(user->isgt_eid_bitmap);
	user->isgt_eid_bitmap = NULL;

	bitmap_free(user->sgclt_word_bitmap);
	user->sgclt_word_bitmap = NULL;

	bitmap_free(user->isct_eid_bitmap);
	user->isct_eid_bitmap = NULL;

	bitmap_free(user->sgit_eid_bitmap);
	user->sgit_eid_bitmap = NULL;

	bitmap_free(user->rpt_eid_bitmap);
	user->rpt_eid_bitmap = NULL;

	bitmap_free(user->ist_eid_bitmap);
	user->ist_eid_bitmap = NULL;

	bitmap_free(user->ect_gid_bitmap);
	user->ect_gid_bitmap = NULL;

	bitmap_free(user->ett_gid_bitmap);
	user->ett_gid_bitmap = NULL;
}

struct pci_dev *netc_get_ptp_timer(struct netc_switch *priv)
{
	struct pci_bus *bus = priv->pdev->bus;
	u32 devfn = priv->info->tmr_devfn;

	return pci_get_domain_bus_and_slot(pci_domain_nr(bus),
					   bus->number, devfn);
}

static u64 netc_switch_adjust_base_time(struct ntmp_user *user, u64 base_time,
					u32 cycle_time)
{
	struct netc_switch *priv = ntmp_to_netc_switch(user);
	u64 current_time, delta, n;
	struct pci_dev *tmr_dev;

	tmr_dev = netc_get_ptp_timer(priv);
	if (!tmr_dev)
		return base_time;

	current_time = netc_timer_get_current_time(tmr_dev);
	pci_dev_put(tmr_dev);
	if (base_time >= current_time)
		return base_time;

	delta = current_time - base_time;
	n = DIV_ROUND_UP_ULL(delta, cycle_time);
	base_time += (n * (u64)cycle_time);

	return base_time;
}

static u32 netc_switch_get_tgst_free_words(struct ntmp_user *user)
{
	struct netc_switch *priv = ntmp_to_netc_switch(user);
	struct netc_switch_regs *regs = &priv->regs;
	u32 words_in_use;
	u32 total_words;

	total_words = netc_base_rd(regs, NETC_TGSTCAPR);
	total_words = NETC_GET_NUM_WORDS(total_words);

	words_in_use = netc_base_rd(regs, NETC_TGSTMOR);
	words_in_use = NETC_GET_NUM_WORDS(words_in_use);

	return total_words - words_in_use;
}

static const struct ntmp_ops ntmp_ops = {
	.adjust_base_time = netc_switch_adjust_base_time,
	.get_tgst_free_words = netc_switch_get_tgst_free_words,
};

static int netc_init_ntmp_user(struct netc_switch *priv)
{
	struct ntmp_user *user = &priv->user;
	int err;

	user->dev_type = NETC_DEV_SWITCH;
	user->ops = &ntmp_ops;
	netc_init_ntmp_tbl_versions(priv);
	netc_get_ntmp_capabilities(priv);

	err = netc_init_ntmp_bitmaps(priv);
	if (err)
		return err;

	err = netc_init_all_cbdrs(priv);
	if (err)
		goto free_ntmp_bitmaps;

	INIT_HLIST_HEAD(&user->flower_list);
	mutex_init(&user->flower_lock);

	return 0;

free_ntmp_bitmaps:
	netc_free_ntmp_bitmaps(priv);

	return err;
}

static void netc_deinit_ntmp_user(struct netc_switch *priv)
{
	netc_destroy_flower_list(priv);
	mutex_destroy(&priv->user.flower_lock);
	netc_remove_all_cbdrs(priv);
	netc_free_ntmp_bitmaps(priv);
}

static void netc_clean_fdbt_aging_entries(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct netc_switch *priv;

	priv = container_of(dwork, struct netc_switch, fdbt_clean);

	/* We should first update the activity element in FDB table */
	scoped_guard(mutex, &priv->fdbt_lock) {
		ntmp_fdbt_update_activity_element(&priv->user);

		/* After the activity element is updated, we delete the aging
		 * entries in the FDB table.
		 */
		ntmp_fdbt_delete_aging_entries(&priv->user,
					       priv->fdbt_aging_act_cnt);
	}

	schedule_delayed_work(&priv->fdbt_clean, priv->fdbt_acteu_interval);
}

static void netc_switch_dos_default_config(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	u32 val;

	val = DOSL2CR_SAMEADDR | DOSL2CR_MSAMCC;
	netc_base_wr(regs, NETC_DOSL2CR, val);

	val = DOSL3CR_SAMEADDR | DOSL3CR_IPSAMCC;
	netc_base_wr(regs, NETC_DOSL3CR, val);
}

static void netc_switch_vfht_default_config(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	u32 val;

	val = netc_base_rd(regs, NETC_VFHTDECR2);

	/* if no match is found in the VLAN Filter table, then VFHTDECR2[MLO]
	 * will take effect. VFHTDECR2[MLO] is set to "Software MAC learning
	 * secure" by default. Notice BPCR[MLO] will override VFHTDECR2[MLO]
	 * if its value is not zero.
	 */
	val = u32_replace_bits(val, MLO_SW_SEC, VFHTDECR2_MLO);
	val = u32_replace_bits(val, MFO_NO_MATCH_DISCARD, VFHTDECR2_MFO);
	netc_base_wr(regs, NETC_VFHTDECR2, val);
}

static void netc_switch_isit_key_config(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	u32 val;

	/* Key construction rule 0: PORT + SMAC + VID */
	val = ISIDKCCR0_VALID | ISIDKCCR0_PORTP | ISIDKCCR0_SMACP |
	      ISIDKCCR0_OVIDP;
	netc_base_wr(regs, NETC_ISIDKCCR0(0), val);

	/* Key construction rule 1: PORT + DMAC + VID */
	val = ISIDKCCR0_VALID | ISIDKCCR0_PORTP | ISIDKCCR0_DMACP |
	      ISIDKCCR0_OVIDP;
	netc_base_wr(regs, NETC_ISIDKCCR0(1), val);
}

static void netc_port_set_max_frame_size(struct netc_port *port,
					 u32 max_frame_size)
{
	u32 val;

	val = PM_MAXFRAM & max_frame_size;
	netc_mac_port_wr(port, NETC_PM_MAXFRM(0), val);
}

void netc_switch_fixed_config(struct netc_switch *priv)
{
	netc_switch_dos_default_config(priv);
	netc_switch_vfht_default_config(priv);
	netc_switch_isit_key_config(priv);
}

static void netc_port_set_tc_max_sdu(struct netc_port *port,
				     int tc, u32 max_sdu)
{
	u32 val = max_sdu & PTCTMSDUR_MAXSDU;

	val = u32_replace_bits(val, SDU_TYPE_MPDU, PTCTMSDUR_SDU_TYPE);
	netc_port_wr(port, NETC_PTCTMSDUR(tc), val);
}

void netc_port_set_all_tc_msdu(struct netc_port *port, u32 *max_sdu)
{
	u32 overhead = ETH_FCS_LEN + VLAN_ETH_HLEN;
	int tc;

	if (dsa_port_is_cpu(port->dp))
		overhead += NETC_TAG_MAX_LEN;

	for (tc = 0; tc < NETC_TC_NUM; tc++) {
		u32 msdu = NETC_MAX_FRAME_LEN;

		if (max_sdu && max_sdu[tc])
			msdu = max_sdu[tc] + overhead;

		if (msdu > NETC_MAX_FRAME_LEN)
			msdu = NETC_MAX_FRAME_LEN;

		netc_port_set_tc_max_sdu(port, tc, msdu);
	}
}

static void netc_port_set_mlo(struct netc_port *port, int mlo)
{
	u32 val, old_val;

	old_val = netc_port_rd(port, NETC_BPCR);
	val = u32_replace_bits(old_val, mlo, BPCR_MLO);
	if (old_val != val)
		netc_port_wr(port, NETC_BPCR, val);
}

void netc_port_fixed_config(struct netc_port *port)
{
	u32 pqnt = 0xffff, qth = 0xff00;
	u32 val;

	/* Default IPV and DR setting */
	val = netc_port_rd(port, NETC_PQOSMR);
	val |= PQOSMR_VS | PQOSMR_VE;
	netc_port_wr(port, NETC_PQOSMR, val);

	/* Enable L2 and L3 DOS */
	val = netc_port_rd(port, NETC_PCR);
	val |= PCR_L2DOSE | PCR_L3DOSE;
	netc_port_wr(port, NETC_PCR, val);

	/* Enable ISIT key construction rule 0 and 1 */
	val = netc_port_rd(port, NETC_PISIDCR);
	val |= PISIDCR_KC0EN | PISIDCR_KC1EN;
	netc_port_wr(port, NETC_PISIDCR, val);

	/* Enable ingress port filter table lookup */
	netc_port_wr(port, NETC_PIPFCR, PIPFCR_EN);

	/* Set the quanta value of TX PAUSE frame */
	netc_mac_port_wr(port, NETC_PM_PAUSE_QUANTA(0), pqnt);

	/* When a quanta timer counts down and reaches this value,
	 * the MAC sends a refresh PAUSE frame with the programmed
	 * full quanta value if a pause condition still exists.
	 */
	netc_mac_port_wr(port, NETC_PM_PAUSE_TRHESH(0), qth);
}

static void netc_port_enable_mac_station_move(struct netc_port *port, bool enabled)
{
	u32 val, old_val;

	old_val = netc_port_rd(port, NETC_BPCR);
	val = u32_replace_bits(old_val, enabled ? 0 : 1, BPCR_STAMVD);
	if (old_val != val)
		netc_port_wr(port, NETC_BPCR, val);
}

static void netc_port_set_group(struct netc_port *port, u32 groupid)
{
	u32 val, old_val;

	old_val = netc_port_rd(port, NETC_PGCR);
	val = u32_replace_bits(old_val, groupid, PGCR_PGID);
	if (old_val != val)
		netc_port_wr(port, NETC_PGCR, val);
}

static void netc_port_default_config(struct netc_port *port)
{
	u32 val;

	netc_port_fixed_config(port);

	/* Default VLAN unware */
	val = netc_port_rd(port, NETC_BPDVR);
	if (!(val & BPDVR_RXVAM)) {
		val |= BPDVR_RXVAM;
		netc_port_wr(port, NETC_BPDVR, val);
	}

	if (dsa_port_is_user(port->dp)) {
		netc_port_set_mlo(port, MLO_DISABLE);
	} else {
		val = netc_port_rd(port, NETC_BPCR) | BPCR_SRCPRND;
		val = u32_replace_bits(val, MLO_HW, BPCR_MLO);
		netc_port_wr(port, NETC_BPCR, val);
	}

	netc_port_set_max_frame_size(port, NETC_MAX_FRAME_LEN);
	netc_port_set_all_tc_msdu(port, NULL);
}

static int netc_switch_bpt_default_config(struct netc_switch *priv)
{
	struct bpt_cfge_data *cfge;

	priv->bpt_list = devm_kcalloc(priv->dev, priv->caps.num_bp,
				      sizeof(*cfge), GFP_KERNEL);
	if (!priv->bpt_list)
		return -ENOMEM;

	if (priv->info->bpt_init)
		priv->info->bpt_init(priv);

	return 0;
}

static int netc_setup(struct dsa_switch *ds)
{
	struct netc_switch *priv = ds->priv;
	struct netc_port *port;
	int i, err;

	netc_switch_get_capabilities(priv);

	err = netc_init_all_ports(ds);
	if (err)
		return err;

	err = netc_init_ntmp_user(priv);
	if (err)
		goto free_internal_mdiobus;

	INIT_HLIST_HEAD(&priv->fdb_list);
	mutex_init(&priv->fdbt_lock);
	INIT_HLIST_HEAD(&priv->vlan_list);
	mutex_init(&priv->vft_lock);
	priv->fdbt_acteu_interval = NETC_FDBT_CLEAN_INTERVAL;
	priv->fdbt_aging_act_cnt = NETC_FDBT_AGING_ACT_CNT;
	INIT_DELAYED_WORK(&priv->fdbt_clean, netc_clean_fdbt_aging_entries);

	netc_switch_fixed_config(priv);

	/* default setting for ports */
	for (i = 0; i < priv->num_ports; i++) {
		port = priv->ports[i];
		if (port->dp)
			netc_port_default_config(port);
	}

	err = netc_switch_bpt_default_config(priv);
	if (err)
		goto free_ntmp_user;

	schedule_delayed_work(&priv->fdbt_clean, priv->fdbt_acteu_interval);

	ds->fdb_isolation = true;

	return 0;

free_ntmp_user:
	netc_deinit_ntmp_user(priv);
free_internal_mdiobus:
	netc_remove_all_ports_internal_mdiobus(ds);

	return err;
}

static void netc_destroy_all_lists(struct netc_switch *priv)
{
	netc_destroy_fdb_list(priv);
	mutex_destroy(&priv->fdbt_lock);
	netc_destroy_vlan_list(priv);
	mutex_destroy(&priv->vft_lock);
}

static void netc_free_ports_taprio(struct netc_switch *priv)
{
	int i;

	for (i = 0; i < priv->num_ports; i++)
		netc_port_free_taprio(priv->ports[i]);
}

static void netc_teardown(struct dsa_switch *ds)
{
	struct netc_switch *priv = ds->priv;

	cancel_delayed_work_sync(&priv->fdbt_clean);
	netc_destroy_all_lists(priv);
	netc_deinit_ntmp_user(priv);
	netc_remove_all_ports_internal_mdiobus(ds);
	netc_free_ports_taprio(priv);
}

static bool netc_port_is_emdio_consumer(struct device_node *node)
{
	struct device_node *mdio_node;

	/* If the port node has phy-handle property and it does
	 * not contain a mdio child node, then the port is the
	 * EMDIO consumer.
	 */
	mdio_node = of_get_child_by_name(node, "mdio");
	if (!mdio_node)
		return true;

	of_node_put(mdio_node);

	return false;
}

/* Currently, phylink_of_phy_connect() is called by dsa_user_create(),
 * so if the switch uses the external MDIO controller (like the EMDIO
 * function) to manage the external PHYs. The MDIO bus may not be
 * created when phylink_of_phy_connect() is called, so it will return
 * an error and cause the switch driver to fail to probe.
 * This workaround can be removed when DSA phylink_of_phy_connect()
 * calls are moved from probe() to ndo_open().
 */
static int netc_switch_check_emdio_is_ready(struct device *dev)
{
	struct device_node *ports, *phy_node;
	struct phy_device *phydev;
	int err = 0;

	ports = of_get_child_by_name(dev->of_node, "ports");
	if (!ports)
		ports = of_get_child_by_name(dev->of_node, "ethernet-ports");

	if (!ports) {
		dev_err(dev, "Cannot find the ethernet-ports or ports node\n");
		return -EINVAL;
	}

	for_each_available_child_of_node_scoped(ports, child) {
		/* If the node does not have phy-handle property, then the
		 * port does not connect to a PHY, so the port is not the
		 * EMDIO consumer.
		 */
		phy_node = of_parse_phandle(child, "phy-handle", 0);
		if (!phy_node)
			continue;

		/* Note that from the hardware perspective, the switch ports
		 * do not support share the MDIO bus defined under one port.
		 * Ecah port can only access its own external PHY through its
		 * port MDIO bus.
		 */
		if (!netc_port_is_emdio_consumer(child)) {
			of_node_put(phy_node);
			continue;
		}

		phydev = of_phy_find_device(phy_node);
		of_node_put(phy_node);
		if (!phydev) {
			err = -EPROBE_DEFER;
			goto out;
		}

		put_device(&phydev->mdio.dev);
	}

out:
	of_node_put(ports);

	return err;
}

static int netc_switch_pci_init(struct pci_dev *pdev)
{
	struct netc_switch *priv __free(kfree) = NULL;
	struct device *dev = &pdev->dev;
	struct netc_switch_regs *regs;
	int err, len;

	pcie_flr(pdev);
	err = pci_enable_device_mem(pdev);
	if (err)
		return dev_err_probe(dev, err, "Failed to enable device\n");

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (err) {
		dev_err(dev, "Failed to configure DMA, err=%d\n", err);
		goto disable_pci_device;
	}

	err = pci_request_mem_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(dev, "Failed to request memory regions, err=%d\n", err);
		goto disable_pci_device;
	}

	pci_set_master(pdev);
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto relase_mem_regions;
	}

	priv->pdev = pdev;
	priv->dev = dev;

	regs = &priv->regs;
	len = pci_resource_len(pdev, NETC_REGS_BAR);
	regs->base = ioremap(pci_resource_start(pdev, NETC_REGS_BAR), len);
	if (!regs->base) {
		err = -ENXIO;
		dev_err(dev, "ioremap() failed\n");
		goto relase_mem_regions;
	}

	regs->port = regs->base + NETC_REGS_PORT_BASE;
	regs->global = regs->base + NETC_REGS_GLOBAL_BASE;
	pci_set_drvdata(pdev, no_free_ptr(priv));

	return 0;

relase_mem_regions:
	pci_release_mem_regions(pdev);
disable_pci_device:
	pci_disable_device(pdev);

	return err;
}

static void netc_switch_pci_destroy(struct pci_dev *pdev)
{
	struct netc_switch *priv;

	priv = pci_get_drvdata(pdev);

	iounmap(priv->regs.base);
	kfree(priv);
	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

static void netc_switch_get_ip_revision(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	u32 val;

	val = netc_glb_rd(regs, NETC_IPBRR0);
	priv->revision = val & IPBRR0_IP_REV;
}

static int netc_add_or_update_ett_entry(struct netc_switch *priv, bool add,
					bool untagged, u32 ett_eid, u32 ect_eid)
{
	struct ntmp_user *user = &priv->user;
	struct ett_cfge_data ett_cfge = {};
	u32 vuda_sqta = FMTEID_VUDA_SQTA;
	u16 efm_cfg = 0;

	if (ect_eid != NTMP_NULL_ENTRY_ID) {
		/* Increase egress frame counter */
		efm_cfg |= FIELD_PREP(ETT_ECA, ETT_ECA_INC);
		ett_cfge.ec_eid = cpu_to_le32(ect_eid);
	}

	/* If egress rule is VLAN untagged */
	if (untagged) {
		/* delete outer VLAN tag */
		vuda_sqta |= FIELD_PREP(FMTEID_VUDA, FMTEID_VUDA_DEL_OTAG);
		/* length change: twos-complement notation */
		efm_cfg |= FIELD_PREP(ETT_EFM_LEN_CHANGE, ETT_FRM_LEN_DEL_VLAN);
	}

	ett_cfge.efm_eid = cpu_to_le32(vuda_sqta);
	ett_cfge.efm_cfg = cpu_to_le16(efm_cfg);

	return ntmp_ett_add_or_update_entry(user, ett_eid,
					    add, &ett_cfge);
}

int netc_add_ett_group_entries(struct netc_switch *priv,
			       u32 untagged_port_bitmap,
			       u32 ett_base_eid,
			       u32 ect_base_eid)
{
	struct ntmp_user *user = &priv->user;
	u32 ett_eid = ett_base_eid;
	int i, err;

	for (i = 0; i < priv->num_ports; i++, ett_eid++) {
		bool untagged = !!(untagged_port_bitmap & BIT(i));
		u32 ect_eid = NTMP_NULL_ENTRY_ID;

		if (ect_base_eid != NTMP_NULL_ENTRY_ID)
			ect_eid = ect_base_eid + i;

		err = netc_add_or_update_ett_entry(priv, true, untagged,
						   ett_eid, ect_eid);
		if (err)
			goto clear_ett_entries;
	}

	return 0;

clear_ett_entries:
	for (i--, ett_eid--; i >= 0; i--, ett_eid--)
		ntmp_ett_delete_entry(user, ett_eid);

	return err;
}

static int netc_switch_add_vlan_egress_rule(struct netc_switch *priv,
					    struct netc_vlan_entry *entry)
{
	struct ntmp_user *user = &priv->user;
	u32 ect_eid = NTMP_NULL_ENTRY_ID;
	u32 ett_eid, ett_gid, ect_gid;
	int i, err;

	/* step1: find available ect entries and update these entries */
	ect_gid = ntmp_lookup_free_eid(user->ect_gid_bitmap,
				       user->ect_bitmap_size);
	if (ect_gid == NTMP_NULL_ENTRY_ID) {
		dev_warn(priv->dev, "No ECT entries available\n");
	} else {
		ect_eid = ect_gid * priv->num_ports;
		for (i = 0; i < priv->num_ports; i++)
			/* Reset the counters of ECT entry */
			ntmp_ect_update_entry(user, ect_eid + i);
	}

	/* step2: find available ett entries and add these entries */
	ett_gid = ntmp_lookup_free_eid(user->ett_gid_bitmap,
				       user->ett_bitmap_size);
	if (ett_gid == NTMP_NULL_ENTRY_ID) {
		dev_err(priv->dev, "No free ETT entries found\n");
		err = -ENOSPC;
		goto clear_ect_gid;
	}

	ett_eid = ett_gid * priv->num_ports;
	err = netc_add_ett_group_entries(priv, entry->untagged_port_bitmap,
					 ett_eid, ect_eid);
	if (err)
		goto clear_ett_gid;

	entry->cfge.et_eid = cpu_to_le32(ett_eid);
	entry->ect_gid = ect_gid;

	return 0;

clear_ett_gid:
	ntmp_clear_eid_bitmap(user->ett_gid_bitmap, ett_gid);

clear_ect_gid:
	/* ECT is a static index table, no need to delete the entries */
	if (ect_gid != NTMP_NULL_ENTRY_ID)
		ntmp_clear_eid_bitmap(user->ect_gid_bitmap, ect_gid);

	return err;
}

void netc_switch_delete_vlan_egress_rule(struct netc_switch *priv,
					 struct netc_vlan_entry *entry)
{
	struct ntmp_user *user = &priv->user;
	u32 ett_eid, ett_gid;
	int i;

	ett_eid = le32_to_cpu(entry->cfge.et_eid);
	if (ett_eid == NTMP_NULL_ENTRY_ID)
		return;

	ett_gid = ett_eid / priv->num_ports;
	ntmp_clear_eid_bitmap(user->ett_gid_bitmap, ett_gid);
	for (i = 0; i < priv->num_ports; i++)
		ntmp_ett_delete_entry(user, ett_eid + i);

	entry->cfge.et_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);

	if (entry->ect_gid == NTMP_NULL_ENTRY_ID)
		return;

	ntmp_clear_eid_bitmap(user->ect_gid_bitmap, entry->ect_gid);
	entry->ect_gid = NTMP_NULL_ENTRY_ID;
}

static int netc_port_update_vlan_egress_rule(struct netc_port *port,
					     struct netc_vlan_entry *entry)
{
	bool untagged = !!(entry->untagged_port_bitmap & BIT(port->index));
	u32 ett_eid = le32_to_cpu(entry->cfge.et_eid);
	struct netc_switch *priv = port->switch_priv;
	u32 ect_eid = NTMP_NULL_ENTRY_ID;

	if (ett_eid == NTMP_NULL_ENTRY_ID)
		return 0;

	if (entry->ect_gid != NTMP_NULL_ENTRY_ID) {
		ect_eid = entry->ect_gid * priv->num_ports + port->index;
		ntmp_ect_update_entry(&priv->user, ect_eid);
	}

	ett_eid += port->index;

	return netc_add_or_update_ett_entry(priv, false, untagged,
					    ett_eid, ect_eid);
}

static int netc_port_update_vlan_egress_seq_tag(struct netc_port *port,
						u16 vid, bool removed)
{
	struct netc_switch *priv = port->switch_priv;
	struct ett_cfge_data ett_cfge = {};
	struct netc_vlan_entry *entry;
	bool vlan_removed = false;
	u32 efmid_old;
	u32 ett_eid;
	int err;
	u8 len;

	guard(mutex)(&priv->vft_lock);

	entry = netc_lookup_vlan_entry(priv, vid);
	if (!entry)
		return 0;

	ett_eid = le32_to_cpu(entry->cfge.et_eid);
	ett_eid += port->index;
	err = ntmp_ett_query_entry(&priv->user, ett_eid, &ett_cfge);
	if (err)
		return err;

	if ((ett_cfge.efm_eid & FRMEOD_VARA_VID) ||
	    !(ett_cfge.efm_eid & FMTEID_VUDA_SQTA))
		return -EOPNOTSUPP;

	efmid_old = ett_cfge.efm_eid;
	ett_cfge.efm_eid &= ~FMTEID_SQTA;
	if (removed)
		ett_cfge.efm_eid |= FIELD_PREP(FMTEID_SQTA, FMTEID_SQTA_DEL);

	if (ett_cfge.efm_eid == efmid_old)
		return 0;

	if ((ett_cfge.efm_eid & FMTEID_VUDA) == FMTEID_VUDA_DEL_OTAG)
		vlan_removed = true;

	if (removed && vlan_removed)
		len = ETT_FRM_LEN_DEL_VLAN_RTAG;
	else if (removed)
		len = ETT_FRM_LEN_DEL_RTAG;
	else if (vlan_removed)
		len = ETT_FRM_LEN_DEL_VLAN;
	else
		len = 0;

	ett_cfge.efm_cfg &= ~ETT_EFM_LEN_CHANGE;
	ett_cfge.efm_cfg |= FIELD_PREP(ETT_EFM_LEN_CHANGE, len);

	/* Update the ETT entry */
	return ntmp_ett_add_or_update_entry(&priv->user, ett_eid, false, &ett_cfge);
}

static int netc_port_add_vlan_entry(struct netc_port *port, u16 vid,
				    bool untagged)
{
	struct netc_switch *priv = port->switch_priv;
	struct netc_vlan_entry *entry __free(kfree);
	u32 bitmap_stg, eta_port_bitmap = 0;
	u16 cfg = 0;
	int i, err;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->vid = vid;
	entry->ect_gid = NTMP_NULL_ENTRY_ID;
	entry->cfge.et_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
	bitmap_stg = BIT(port->index) | VFT_STG_ID(0);
	entry->cfge.bitmap_stg = cpu_to_le32(bitmap_stg);
	entry->cfge.fid = cpu_to_le16(vid);

	if (vid == NETC_STANDALONE_PVID) {
		cfg |= FIELD_PREP(VFT_MLO, MLO_DISABLE);
		cfg |= FIELD_PREP(VFT_MFO, MFO_NO_MATCH_DISCARD);
		entry->cfge.cfg = cpu_to_le16(cfg);
	} else {
		cfg |= FIELD_PREP(VFT_MLO, MLO_HW);
		cfg |= FIELD_PREP(VFT_MFO, MFO_NO_MATCH_FLOOD);
		entry->cfge.cfg = cpu_to_le16(cfg);

		for (i = 0; i < priv->num_ports; i++)
			eta_port_bitmap |= BIT(i);

		if (untagged && vid != NETC_VLAN_UNAWARE_PVID)
			entry->untagged_port_bitmap = BIT(port->index);

		entry->cfge.eta_port_bitmap = cpu_to_le32(eta_port_bitmap);

		err = netc_switch_add_vlan_egress_rule(priv, entry);
		if (err)
			return err;
	}

	err = ntmp_vft_add_entry(&priv->user, &entry->entry_id, vid,
				 &entry->cfge);
	if (err)
		goto delete_vlan_egress_rule;

	netc_add_vlan_entry(priv, no_free_ptr(entry));

	return 0;

delete_vlan_egress_rule:
	if (vid != NETC_STANDALONE_PVID)
		netc_switch_delete_vlan_egress_rule(priv, entry);

	return err;
}

static bool netc_port_vlan_egress_rule_changed(struct netc_vlan_entry *entry,
					       int port_id, bool untagged)
{
	bool port_untagged = !!(entry->untagged_port_bitmap & BIT(port_id));
	u16 vid = entry->vid;

	if (vid == NETC_STANDALONE_PVID || vid == NETC_VLAN_UNAWARE_PVID)
		return false;

	if (port_untagged == untagged)
		return false;

	return true;
}

static int netc_port_set_vlan_entry(struct netc_port *port, u16 vid,
				    bool untagged)
{
	struct netc_switch *priv = port->switch_priv;
	struct netc_vlan_entry *entry;
	int port_id = port->index;
	bool rule_changed;
	int err;

	guard(mutex)(&priv->vft_lock);

	entry = netc_lookup_vlan_entry(priv, vid);
	if (!entry) {
		err = netc_port_add_vlan_entry(port, vid, untagged);
		if (err)
			dev_err(priv->dev,
				"Failed to add VLAN %u entry for port:%d\n",
				vid, port_id);

		return err;
	}

	rule_changed = netc_port_vlan_egress_rule_changed(entry, port_id, untagged);
	if (rule_changed) {
		entry->untagged_port_bitmap ^= BIT(port_id);
		err = netc_port_update_vlan_egress_rule(port, entry);
		if (err) {
			dev_err(priv->dev,
				"Port:%d failed to update VLAN %u egress rule\n",
				port_id, vid);

			goto restore_untagged_bitmap;
		}
	}

	if (entry->cfge.bitmap_stg & cpu_to_le32(BIT(port_id)))
		return 0;

	entry->cfge.bitmap_stg ^= cpu_to_le32(BIT(port_id));
	err = ntmp_vft_update_entry(&priv->user, vid, &entry->cfge);
	if (err) {
		dev_err(priv->dev, "Port:%d failed to update VLAN %u entry\n",
			port_id, vid);

		goto restore_bitmap_stg;
	}

	return 0;

restore_bitmap_stg:
	entry->cfge.bitmap_stg ^= cpu_to_le32(BIT(port_id));
restore_untagged_bitmap:
	if (rule_changed)
		entry->untagged_port_bitmap ^= BIT(port_id);

	return err;
}

static int netc_port_del_vlan_entry(struct netc_port *port, u16 vid)
{
	struct netc_switch *priv = port->switch_priv;
	struct netc_vlan_entry *entry;
	int port_id = port->index;
	u32 vlan_port_bitmap;
	int err;

	guard(mutex)(&priv->vft_lock);
	entry = netc_lookup_vlan_entry(priv, vid);
	if (!entry)
		return 0;

	vlan_port_bitmap = le32_to_cpu(entry->cfge.bitmap_stg) &
			   VFT_PORT_MEMBERSHIP;
	/* If the VLAN only belongs to the current port */
	if (vlan_port_bitmap == BIT(port_id)) {
		ntmp_vft_delete_entry(&priv->user, vid);
		if (vid != NETC_STANDALONE_PVID)
			netc_switch_delete_vlan_egress_rule(priv, entry);

		netc_del_vlan_entry(entry);

		return 0;
	}

	if (!(vlan_port_bitmap & BIT(port_id)))
		return 0;

	entry->cfge.bitmap_stg ^= cpu_to_le32(BIT(port_id));
	err = ntmp_vft_update_entry(&priv->user, vid, &entry->cfge);
	if (err) {
		entry->cfge.bitmap_stg ^= cpu_to_le32(BIT(port_id));

		return err;
	}

	entry->untagged_port_bitmap &= ~BIT(port_id);

	return 0;
}

static int netc_port_add_fdb_entry(struct netc_port *port,
				   const unsigned char *addr, u16 vid)
{
	struct netc_switch *priv = port->switch_priv;
	struct netc_fdb_entry *entry __free(kfree);
	struct fdbt_keye_data *keye;
	struct fdbt_cfge_data *cfge;
	int port_id = port->index;
	u32 cfg = 0;
	int err;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	keye = &entry->keye;
	cfge = &entry->cfge;
	ether_addr_copy(keye->mac_addr, addr);
	keye->fid = cpu_to_le16(vid);

	cfge->port_bitmap = cpu_to_le32(BIT(port_id));
	cfge->cfg = cpu_to_le32(cfg);
	cfge->et_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);

	err = ntmp_fdbt_add_entry(&priv->user, &entry->entry_id, keye, cfge);
	if (err)
		return err;

	netc_add_fdb_entry(priv, no_free_ptr(entry));

	return 0;
}

static int netc_port_set_fdb_entry(struct netc_port *port,
				   const unsigned char *addr, u16 vid)
{
	struct netc_switch *priv = port->switch_priv;
	struct netc_fdb_entry *entry;
	int port_id = port->index;
	u32 port_bitmap;
	int err;

	guard(mutex)(&priv->fdbt_lock);

	entry = netc_lookup_fdb_entry(priv, addr, vid);
	if (!entry) {
		err = netc_port_add_fdb_entry(port, addr, vid);
		if (err)
			dev_err(priv->dev, "Failed to add FDB entry for port:%d\n",
				port_id);

		return err;
	}

	port_bitmap = le32_to_cpu(entry->cfge.port_bitmap);
	/* If the entry has existed on the port, return 0 directly */
	if (unlikely(port_bitmap & BIT(port_id)))
		return 0;

	/* If the entry has already existed, but not exists on this port,
	 * we need to update the port bitmap. In general, it should only
	 * be valid for multicast or broadcast address.
	 */
	port_bitmap ^= BIT(port_id);
	entry->cfge.port_bitmap = cpu_to_le32(port_bitmap);
	err = ntmp_fdbt_update_entry(&priv->user, entry->entry_id, &entry->cfge);
	if (err) {
		port_bitmap ^= BIT(port_id);
		entry->cfge.port_bitmap = cpu_to_le32(port_bitmap);
		dev_err(priv->dev, "Failed to set FDB entry for port:%d\n",
			port_id);
	}

	return err;
}

static int netc_port_del_fdb_entry(struct netc_port *port,
				   const unsigned char *addr, u16 vid)
{
	struct netc_switch *priv = port->switch_priv;
	struct ntmp_user *user = &priv->user;
	struct netc_fdb_entry *entry;
	int port_id = port->index;
	u32 port_bitmap;
	int err;

	guard(mutex)(&priv->fdbt_lock);

	entry = netc_lookup_fdb_entry(priv, addr, vid);
	if (unlikely(!entry))
		return 0;

	port_bitmap = le32_to_cpu(entry->cfge.port_bitmap);
	if (unlikely(!(port_bitmap & BIT(port_id))))
		return 0;

	if (port_bitmap != BIT(port_id)) {
		/* If the entry also exists on other ports, we need to
		 * update the entry in the FDB table.
		 */
		port_bitmap ^= BIT(port_id);
		entry->cfge.port_bitmap = cpu_to_le32(port_bitmap);
		err = ntmp_fdbt_update_entry(user, entry->entry_id, &entry->cfge);
		if (err) {
			port_bitmap ^= BIT(port_id);
			entry->cfge.port_bitmap = cpu_to_le32(port_bitmap);

			return err;
		}

	} else {
		/* If the entry only exists on this port, just delete
		 * it from the FDB table.
		 */
		err = ntmp_fdbt_delete_entry(user, entry->entry_id);
		if (err)
			return err;

		netc_del_fdb_entry(entry);
	}

	return 0;
}

static int netc_port_add_bcast_fdb_entry(struct netc_port *port, u16 vid)
{
	const u8 bcast[ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	return netc_port_set_fdb_entry(port, bcast, vid);
}

static int netc_port_del_bcast_fdb_entry(struct netc_port *port, u16 vid)
{
	const u8 bcast[ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	return netc_port_del_fdb_entry(port, bcast, vid);
}

static int netc_port_enable(struct dsa_switch *ds, int port_id,
			    struct phy_device *phy)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	int err;

	err = netc_port_set_vlan_entry(port, NETC_STANDALONE_PVID, false);
	if (err) {
		dev_err(ds->dev,
			"Failed to set VLAN %d entry for port:%d\n",
			NETC_STANDALONE_PVID, port_id);
		return err;
	}

	/* If the user port as a standalone port, then its PVID is 0,
	 * MLO is set to "disable MAC learning" and MFO is set to
	 * "discard frames if no matching entry found in FDB table".
	 * Therefore, we need to add a broadcast FDB entry on the CPU
	 * port so that the broadcast frames receivced on the user
	 * port can be forwarded to the CPU port.
	 */
	if (dsa_is_cpu_port(ds, port_id)) {
		err = netc_port_add_bcast_fdb_entry(port, NETC_STANDALONE_PVID);
		if (err) {
			dev_err(ds->dev,
				"Failed to set broadcast FDB entry for port:%d\n",
				port_id);
			goto del_standalone_vlan_entry;
		}

		err = netc_port_set_vlan_entry(port, NETC_VLAN_UNAWARE_PVID, false);
		if (err) {
			dev_err(ds->dev,
				"Failed to set VLAN %d entry for port:%d\n",
				NETC_VLAN_UNAWARE_PVID, port_id);
			goto del_bcast_fdb_entry;
		}
	}

	err = clk_prepare_enable(port->ref_clk);
	if (err) {
		dev_err(ds->dev,
			"Enable enet_ref_clk of port %d failed\n", port_id);
		goto del_unaware_vlan_entry;
	}

	port->enabled = true;

	return 0;

del_unaware_vlan_entry:
	if (dsa_is_cpu_port(ds, port_id))
		netc_port_del_vlan_entry(port, NETC_VLAN_UNAWARE_PVID);
del_bcast_fdb_entry:
	if (dsa_is_cpu_port(ds, port_id))
		netc_port_del_bcast_fdb_entry(port, NETC_STANDALONE_PVID);
del_standalone_vlan_entry:
	netc_port_del_vlan_entry(port, NETC_STANDALONE_PVID);

	return err;
}

static void netc_port_disable(struct dsa_switch *ds, int port_id)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);

	clk_disable_unprepare(port->ref_clk);

	if (dsa_is_cpu_port(ds, port_id)) {
		netc_port_del_vlan_entry(port, NETC_VLAN_UNAWARE_PVID);
		netc_port_del_bcast_fdb_entry(port, NETC_STANDALONE_PVID);
	}

	netc_port_del_vlan_entry(port, NETC_STANDALONE_PVID);
	port->enabled = false;
}

static void netc_port_stp_state_set(struct dsa_switch *ds, int port_id, u8 state)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	u32 val;

	if (state > BR_STATE_BLOCKING)
		return;

	/* mapping of STP protocol states to NETC STG_STATE filed states */
	if (state == BR_STATE_DISABLED || state == BR_STATE_LISTENING ||
	    state == BR_STATE_BLOCKING)
		val = NETC_STG_STATE_DISABLED;
	else if (state == BR_STATE_LEARNING)
		val = NETC_STG_STATE_LEARNING;
	else
		val = NETC_STG_STATE_FORWARDING;

	netc_port_wr(port, NETC_BPSTGSR, val);
}

static int netc_port_change_mtu(struct dsa_switch *ds, int port_id, int new_mtu)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	u32 max_frame_size = new_mtu + VLAN_ETH_HLEN + ETH_FCS_LEN;

	if (dsa_is_cpu_port(ds, port_id))
		max_frame_size += NETC_TAG_MAX_LEN;

	netc_port_set_max_frame_size(port, max_frame_size);

	return 0;
}

static int netc_port_max_mtu(struct dsa_switch *ds, int port_id)
{
	return NETC_MAX_FRAME_LEN - VLAN_ETH_HLEN - ETH_FCS_LEN;
}

static struct net_device *netc_classify_db(struct dsa_db db)
{
	switch (db.type) {
	case DSA_DB_PORT:
		return NULL;
	case DSA_DB_BRIDGE:
		return db.bridge.dev;
	default:
		return ERR_PTR(-EOPNOTSUPP);
	}
}

static int netc_port_fdb_add(struct dsa_switch *ds, int port_id,
			     const unsigned char *addr, u16 vid,
			     struct dsa_db db)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	struct net_device *bridge = netc_classify_db(db);

	if (IS_ERR(bridge))
		return PTR_ERR(bridge);

	if (!vid) {
		if (!bridge)
			vid = NETC_STANDALONE_PVID;
		else
			vid = NETC_VLAN_UNAWARE_PVID;
	}

	return netc_port_set_fdb_entry(port, addr, vid);
}

static int netc_port_fdb_del(struct dsa_switch *ds, int port_id,
			     const unsigned char *addr, u16 vid,
			     struct dsa_db db)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	struct net_device *bridge = netc_classify_db(db);

	if (IS_ERR(bridge))
		return PTR_ERR(bridge);

	if (!vid) {
		if (!bridge)
			vid = NETC_STANDALONE_PVID;
		else
			vid = NETC_VLAN_UNAWARE_PVID;
	}

	return netc_port_del_fdb_entry(port, addr, vid);
}

static int netc_port_fdb_dump(struct dsa_switch *ds, int port_id,
			      dsa_fdb_dump_cb_t *cb, void *data)
{
	struct fdbt_entry_data *entry_data __free(kfree);
	struct netc_switch *priv = ds->priv;
	u32 resume_eid = NTMP_NULL_ENTRY_ID;
	struct fdbt_keye_data *keye;
	struct fdbt_cfge_data *cfge;
	u32 entry_id, cfg;
	bool is_static;
	u16 vid;
	int err;

	entry_data = kmalloc(sizeof(*entry_data), GFP_KERNEL);
	if (!entry_data)
		return -ENOMEM;

	keye = &entry_data->keye;
	cfge = &entry_data->cfge;
	guard(mutex)(&priv->fdbt_lock);
	do {
		memset(entry_data, 0, sizeof(*entry_data));
		err = ntmp_fdbt_search_port_entry(&priv->user, port_id,
						  &resume_eid, &entry_id,
						  entry_data);
		if (err || entry_id == NTMP_NULL_ENTRY_ID)
			break;

		cfg = le32_to_cpu(cfge->cfg);
		is_static = (cfg & FDBT_DYNAMIC) ? false : true;
		vid = le16_to_cpu(keye->fid);
		if (vid == NETC_VLAN_UNAWARE_PVID)
			vid = 0;

		err = cb(keye->mac_addr, vid, is_static, data);
		if (err)
			break;
	} while (resume_eid != NTMP_NULL_ENTRY_ID);

	return err;
}

static int netc_port_mdb_add(struct dsa_switch *ds, int port_id,
			     const struct switchdev_obj_port_mdb *mdb,
			     struct dsa_db db)
{
	return netc_port_fdb_add(ds, port_id, mdb->addr, mdb->vid, db);
}

static int netc_port_mdb_del(struct dsa_switch *ds, int port_id,
			     const struct switchdev_obj_port_mdb *mdb,
			     struct dsa_db db)
{
	return netc_port_fdb_del(ds, port_id, mdb->addr, mdb->vid, db);
}

static bool netc_user_ports_all_standalone(struct netc_switch *priv)
{
	struct dsa_switch *ds = priv->ds;
	struct dsa_port *dp;

	dsa_switch_for_each_user_port(dp, ds)
		if (dsa_port_bridge_dev_get(dp))
			return false;

	return true;
}

static bool netc_user_ports_vlan_aware(struct netc_switch *priv)
{
	struct dsa_switch *ds = priv->ds;
	struct netc_port *port;
	struct dsa_port *dp;

	dsa_switch_for_each_user_port(dp, ds) {
		port = NETC_PORT(priv, dp->index);
		if (port->vlan_aware)
			return true;
	}

	return false;
}

static void netc_cpu_port_set_vlan_filtering(struct netc_switch *priv)
{
	struct dsa_switch *ds = priv->ds;
	struct netc_port *port;
	struct dsa_port *dp;
	bool vlan_aware;
	u16 pvid;
	u32 val;

	vlan_aware = netc_user_ports_vlan_aware(priv);

	dsa_switch_for_each_available_port(dp, ds) {
		port = NETC_PORT(priv, dp->index);
		if (dsa_port_is_cpu(dp)) {
			if (netc_user_ports_all_standalone(priv)) {
				pvid = NETC_STANDALONE_PVID;
				port->pvid = NETC_STANDALONE_PVID;
				port->vlan_aware = 0;
			} else {
				pvid = vlan_aware ? port->pvid :
				       NETC_VLAN_UNAWARE_PVID;
				port->vlan_aware = vlan_aware ? 1 : 0;
			}

			val = netc_port_rd(port, NETC_BPDVR);
			val = u32_replace_bits(val, port->vlan_aware ? 0 : 1,
					       BPDVR_RXVAM);
			val = u32_replace_bits(val, pvid, BPDVR_VID);
			netc_port_wr(port, NETC_BPDVR, val);
		}
	}
}

static int netc_port_vlan_filtering(struct dsa_switch *ds, int port_id,
				    bool vlan_aware, struct netlink_ext_ack *extack)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	u16 pvid;
	u32 val;

	if (!port->bridge) {
		pvid = NETC_STANDALONE_PVID;
		port->pvid = NETC_STANDALONE_PVID;
		port->vlan_aware = 0;
	} else {
		pvid = vlan_aware ? port->pvid : NETC_VLAN_UNAWARE_PVID;
		port->vlan_aware = vlan_aware ? 1 : 0;
	}

	val = netc_port_rd(port, NETC_BPDVR);
	val = u32_replace_bits(val, port->vlan_aware ? 0 : 1, BPDVR_RXVAM);
	val = u32_replace_bits(val, pvid, BPDVR_VID);
	netc_port_wr(port, NETC_BPDVR, val);

	netc_cpu_port_set_vlan_filtering(ds->priv);

	return 0;
}

static void netc_port_set_pvid(struct netc_port *port, u16 pvid)
{
	u32 val;

	val = netc_port_rd(port, NETC_BPDVR);
	val = u32_replace_bits(val, pvid, BPDVR_VID);
	netc_port_wr(port, NETC_BPDVR, val);
}

static int netc_port_vlan_add(struct dsa_switch *ds, int port_id,
			      const struct switchdev_obj_port_vlan *vlan,
			      struct netlink_ext_ack *extack)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	bool is_pvid, untagged;
	u16 pvid;
	int err;

	untagged = !!(vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED);
	err = netc_port_set_vlan_entry(port, vlan->vid, untagged);
	if (err)
		return err;

	is_pvid = !!(vlan->flags & BRIDGE_VLAN_INFO_PVID);
	/* BRIDGE_VLAN_INFO_PVID won't be set for CPU port due to
	 * commit b9499904f363, so we set VID 1 as the PVID of CPU
	 * port and it is unchangeable.
	 */
	if (dsa_is_cpu_port(ds, port_id) &&
	    vlan->vid == NETC_CPU_PORT_PVID)
		is_pvid = true;

	if (is_pvid) {
		port->pvid = vlan->vid;
		pvid = vlan->vid;
		if (!port->vlan_aware)
			pvid = NETC_VLAN_UNAWARE_PVID;

		netc_port_set_pvid(port, pvid);
	} else {
		/* delete PVID */
		if (port->pvid == vlan->vid) {
			port->pvid = 0;

			if (port->vlan_aware)
				netc_port_set_pvid(port, 0);
		}
	}

	return 0;
}

static int netc_port_vlan_del(struct dsa_switch *ds, int port_id,
			      const struct switchdev_obj_port_vlan *vlan)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	int err;

	err = netc_port_del_vlan_entry(port, vlan->vid);
	if (err)
		return err;

	if (port->pvid == vlan->vid) {
		port->pvid = 0;

		if (port->vlan_aware)
			netc_port_set_pvid(port, 0);
	}

	return 0;
}

static int netc_set_ageing_time(struct dsa_switch *ds, unsigned int msecs)
{
	struct netc_switch *priv = ds->priv;
	u32 secs = msecs / 1000;
	u32 act_cnt, interval;

	if (!secs)
		secs = 1;

	for (interval = 1; interval <= secs; interval++) {
		act_cnt = secs / interval;
		if (act_cnt <= FDBT_MAX_ACT_CNT)
			break;
	}

	priv->fdbt_acteu_interval = interval * HZ;
	priv->fdbt_aging_act_cnt = act_cnt;

	return 0;
}

static void netc_port_remove_dynamic_entries(struct netc_port *port)
{
	struct netc_switch *priv = port->switch_priv;

	guard(mutex)(&priv->fdbt_lock);
	ntmp_fdbt_delete_port_dynamic_entries(&priv->user, port->index);
}

static void netc_port_fast_age(struct dsa_switch *ds, int port_id)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);

	netc_port_remove_dynamic_entries(port);
}

static int netc_add_sequence_generate(struct netc_switch *priv, u32 entry_id,
				      u8 tag)
{
	struct ntmp_user *user = &priv->user;
	struct isgt_cfge_data cfge = {};

	cfge.sq_tag = cpu_to_le16(FIELD_PREP(ISGT_SQ_TAG, tag));

	return ntmp_isgt_add_or_update_entry(user, entry_id, true, &cfge);
}

int netc_port_set_hsr(struct netc_port *port, enum netc_port_hsr_type type)
{
	struct netc_switch *priv = port->switch_priv;
	bool remove_seq_tag = false;
	bool is_sr_port = false;
	bool enable_sdf = true;
	u32 isgt_eid = 0xffff;
	u32 val, old_val;
	u8 pathid = 0;
	u8 stp_state;
	int err;

	if (type == NETC_HSR_DISABLED) {
		netc_port_del_vlan_entry(port, NETC_VLAN_UNAWARE_PVID);
		port->pvid = NETC_STANDALONE_PVID;
		netc_port_set_mlo(port, MLO_DISABLE);

		if (port->hsr_data.isgt_eid != 0xffff) {
			ntmp_isgt_delete_entry(&priv->user,
					       port->hsr_data.isgt_eid);
			ntmp_clear_eid_bitmap(priv->user.isgt_eid_bitmap,
					      port->hsr_data.isgt_eid);
			port->hsr_data.isgt_eid = 0xffff;
		}
	} else {
		err = netc_port_set_vlan_entry(port, NETC_VLAN_UNAWARE_PVID, false);
		if (err)
			return err;

		/* Clean the ISGT_EID bitmap when switch is resumed. */
		if (port->hsr_data.isgt_eid != 0xffff) {
			ntmp_clear_eid_bitmap(priv->user.isgt_eid_bitmap,
					      port->hsr_data.isgt_eid);
			port->hsr_data.isgt_eid = 0xffff;
		}

		port->pvid = NETC_VLAN_UNAWARE_PVID;
		netc_port_set_mlo(port, MLO_NOT_OVERRIDE);
	}

	netc_port_set_pvid(port, port->pvid);

	if (netif_carrier_ok(port->dp->user))
		stp_state = BR_STATE_FORWARDING;
	else
		stp_state = BR_STATE_DISABLED;

	netc_port_stp_state_set(port->dp->ds, port->dp->index, stp_state);

	if (type == NETC_HSR_PORT_A || type == NETC_HSR_PORT_B) {
		netc_port_set_group(port, NETC_PGID_HSR);
		netc_port_enable_mac_station_move(port, false);

		pathid = (type == NETC_HSR_PORT_A) ? 0 : 1;
		is_sr_port = true;
	} else if (type == NETC_HSR_REDBOX_INTERLINK || type == NETC_HSR_UPPER) {
		remove_seq_tag = true;

		isgt_eid = ntmp_lookup_free_eid(priv->user.isgt_eid_bitmap,
						priv->user.caps.isgt_num_entries);

		if (isgt_eid == NTMP_NULL_ENTRY_ID) {
			dev_warn(priv->dev, "No ISGT entries available\n");
			return -ENOSPC;
		}

		err = netc_add_sequence_generate(priv, isgt_eid, ISGT_SQ_TAG_HSR);
		if (err)
			goto clear_isgt_eid;

		err = netc_port_update_vlan_egress_seq_tag(port, port->pvid, true);
		if (err)
			goto del_isgt_entries;
	} else {
		enable_sdf = false;
		netc_port_set_group(port, 0);
		netc_port_enable_mac_station_move(port, true);
	}

	old_val = netc_port_rd(port, NETC_PSRCR);
	val = u32_replace_bits(old_val, is_sr_port, PSRCR_SR_PORT);
	val = u32_replace_bits(val, enable_sdf, PSRCR_SDFA);
	val = u32_replace_bits(val, remove_seq_tag, PSRCR_TX_SQTA);
	val = u32_replace_bits(val, pathid, PSRCR_PATHID);
	val = u32_replace_bits(val, isgt_eid, PSRCR_ISQG_EID);
	if (old_val != val)
		netc_port_wr(port, NETC_PSRCR, val);

	port->hsr_data.type = type;
	port->hsr_data.isgt_eid = isgt_eid;

	return 0;

del_isgt_entries:
	ntmp_isgt_delete_entry(&priv->user, isgt_eid);
clear_isgt_eid:
	ntmp_clear_eid_bitmap(priv->user.isgt_eid_bitmap, isgt_eid);

	return err;
}

static int netc_port_hsr_join(struct dsa_switch *ds, int port_id,
			      struct net_device *hsr,
			      struct netlink_ext_ack *extack)
{
	struct net_device *user = dsa_to_port(ds, port_id)->user;
	struct netc_switch *priv = ds->priv;
	struct dsa_port *cpu_dp, *hsr_dp;
	enum netc_port_hsr_type type = 0;
	struct netc_port *port;
	enum hsr_version ver;
	int err;

	err = hsr_get_version(hsr, &ver);
	if (err)
		return err;

	if (!(ver == HSR_V0 || ver == HSR_V1)) {
		NL_SET_ERR_MSG_MOD(extack, "Only HSR v0/1 can be offloaded");
		return -EOPNOTSUPP;
	}

	dsa_hsr_foreach_port(hsr_dp, ds, hsr)
		type++;

	port = priv->ports[port_id];
	err = netc_port_set_hsr(port, type);
	if (err)
		return err;

	if (type == NETC_HSR_PORT_B) {
		dsa_switch_for_each_cpu_port(cpu_dp, ds) {
			port = priv->ports[cpu_dp->index];
			err = netc_port_set_hsr(port, NETC_HSR_UPPER);
			if (err)
				goto disable_hsr;
		}
	}

	user->features |= NETC_SUPPORTED_HSR_FEATURES;
	port->hsr_enabled = true;

	return 0;

disable_hsr:
	netc_port_set_hsr(priv->ports[port_id], NETC_HSR_DISABLED);

	return err;
}

static int netc_port_hsr_leave(struct dsa_switch *ds, int port_id,
			       struct net_device *hsr)
{
	struct net_device *user = dsa_to_port(ds, port_id)->user;
	enum netc_port_hsr_type type = NETC_HSR_DISABLED;
	struct netc_switch *priv = ds->priv;
	struct dsa_port *cpu_dp, *hsr_dp;
	struct netc_port *port;
	int hsr_num = 0;

	user->features &= ~NETC_SUPPORTED_HSR_FEATURES;
	port = priv->ports[port_id];
	port->hsr_enabled = false;

	netc_port_set_hsr(port, type);

	dsa_hsr_foreach_port(hsr_dp, ds, hsr)
		hsr_num++;

	if (!hsr_num) {
		dsa_switch_for_each_cpu_port(cpu_dp, ds) {
			port = priv->ports[cpu_dp->index];
			netc_port_set_hsr(port, type);
		}
	}

	return 0;
}

static int netc_port_bridge_join(struct dsa_switch *ds, int port_id,
				 struct dsa_bridge bridge,
				 bool *tx_fwd_offload,
				 struct netlink_ext_ack *extack)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	int err;

	err = netc_port_set_vlan_entry(port, NETC_VLAN_UNAWARE_PVID, false);
	if (err)
		return err;

	port->bridge = bridge.dev;
	netc_port_set_mlo(port, MLO_NOT_OVERRIDE);

	return 0;
}

static void netc_port_bridge_leave(struct dsa_switch *ds, int port_id,
				   struct dsa_bridge bridge)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);

	netc_port_set_mlo(port, MLO_DISABLE);
	port->bridge = NULL;
	netc_port_del_vlan_entry(port, NETC_VLAN_UNAWARE_PVID);
}

static int netc_port_setup_tc(struct dsa_switch *ds, int port_id,
			      enum tc_setup_type type, void *type_data)
{
	struct netc_switch *priv = ds->priv;

	if (!dsa_is_user_port(ds, port_id))
		return -EOPNOTSUPP;

	switch (type) {
	case TC_QUERY_CAPS:
		return netc_tc_query_caps(type_data);
	case TC_SETUP_QDISC_MQPRIO:
		return netc_tc_setup_mqprio(priv, port_id, type_data);
	case TC_SETUP_QDISC_CBS:
		return netc_tc_setup_cbs(priv, port_id, type_data);
	case TC_SETUP_QDISC_TAPRIO:
		return netc_tc_setup_taprio(priv, port_id, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

static int netc_port_cls_flower_add(struct dsa_switch *ds, int port_id,
				    struct flow_cls_offload *cls, bool ingress)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);

	if (!ingress)
		return -EOPNOTSUPP;

	return netc_port_flow_cls_replace(port, cls);
}

static int netc_port_cls_flower_del(struct dsa_switch *ds, int port_id,
				    struct flow_cls_offload *cls, bool ingress)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);

	if (!ingress)
		return -EOPNOTSUPP;

	return netc_port_flow_cls_destroy(port, cls);
}

static int netc_port_cls_flower_stats(struct dsa_switch *ds, int port_id,
				      struct flow_cls_offload *cls, bool ingress)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);

	if (!ingress)
		return -EOPNOTSUPP;

	return netc_port_flow_cls_stats(port, cls);
}

static void netc_phylink_get_caps(struct dsa_switch *ds, int port_id,
				  struct phylink_config *config)
{
	struct netc_switch *priv = ds->priv;

	if (priv->info && priv->info->phylink_get_caps)
		priv->info->phylink_get_caps(port_id, config);
}

static struct phylink_pcs *netc_mac_select_pcs(struct phylink_config *config,
					       phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct netc_switch *priv = dp->ds->priv;

	return priv->ports[dp->index]->pcs;
}

static void netc_port_set_mac_mode(struct netc_port *port,
				   unsigned int mode, phy_interface_t phy_mode)
{
	u32 val;

	val = netc_mac_port_rd(port, NETC_PM_IF_MODE(0));
	val &= ~(PM_IF_MODE_IFMODE | PM_IF_MODE_ENA);

	switch (phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val |= IFMODE_RGMII;
		break;
	case PHY_INTERFACE_MODE_RMII:
		val |= IFMODE_RMII;
		break;
	case PHY_INTERFACE_MODE_REVMII:
		val |= PM_IF_MODE_REVMII;
		fallthrough;
	case PHY_INTERFACE_MODE_MII:
		val |= IFMODE_MII;
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
		val |= IFMODE_SGMII;
		break;
	default:
		break;
	}

	netc_mac_port_wr(port, NETC_PM_IF_MODE(0), val);
}

static void netc_mac_config(struct phylink_config *config, unsigned int mode,
			    const struct phylink_link_state *state)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct netc_switch *priv = dp->ds->priv;

	netc_port_set_mac_mode(priv->ports[dp->index], mode, state->interface);
}

static void netc_port_set_speed(struct netc_port *port, int speed)
{
	u32 old_val = netc_port_rd(port, NETC_PCR);
	u32 val = old_val & (~PCR_PSPEED);

	switch (speed) {
	case SPEED_10:
	case SPEED_100:
	case SPEED_1000:
	case SPEED_2500:
		val |= PSPEED_SET_VAL(speed);
		break;
	default:
		dev_err(port->switch_priv->dev,
			"Unsupported MAC speed:%d\n", speed);
		return;
	}

	port->speed = speed;
	if (val != old_val)
		netc_port_wr(port, NETC_PCR, val);
}

/* If the RGMII device does not support the In-Band Status (IBS), we need
 * the MAC driver to get the link speed and duplex mode from the PHY driver.
 * The MAC driver then sets the MAC for the correct speed and duplex mode
 * to match the PHY. The PHY driver gets the link status and speed and duplex
 * information from the PHY via the MDIO/MDC interface.
 */
static void netc_port_force_set_rgmii_mac(struct netc_port *port,
					  int speed, int duplex)
{
	u32 old_val, val;

	old_val = netc_mac_port_rd(port, NETC_PM_IF_MODE(0));
	val = old_val & ~(PM_IF_MODE_ENA | PM_IF_MODE_M10 | PM_IF_MODE_REVMII);

	switch (speed) {
	case SPEED_1000:
		val = u32_replace_bits(val, SSP_1G, PM_IF_MODE_SSP);
		break;
	case SPEED_100:
		val = u32_replace_bits(val, SSP_100M, PM_IF_MODE_SSP);
		break;
	case SPEED_10:
		val = u32_replace_bits(val, SSP_10M, PM_IF_MODE_SSP);
	}

	val = u32_replace_bits(val, duplex == DUPLEX_FULL ? 0 : 1,
			       PM_IF_MODE_HD);

	if (old_val == val)
		return;

	netc_mac_port_wr(port, NETC_PM_IF_MODE(0), val);
}

static void net_port_set_rmii_mii_mac(struct netc_port *port,
				      int speed, int duplex)
{
	u32 old_val, val;

	old_val = netc_mac_port_rd(port, NETC_PM_IF_MODE(0));
	val = old_val & ~(PM_IF_MODE_ENA | PM_IF_MODE_SSP);

	switch (speed) {
	case SPEED_100:
		val &= ~PM_IF_MODE_M10;
		break;
	case SPEED_10:
		val |= PM_IF_MODE_M10;
	}

	val = u32_replace_bits(val, duplex == DUPLEX_FULL ? 0 : 1,
			       PM_IF_MODE_HD);

	if (old_val == val)
		return;

	netc_mac_port_wr(port, NETC_PM_IF_MODE(0), val);
}

void netc_port_set_tx_pause(struct netc_port *port, bool tx_pause)
{
	struct netc_switch *priv = port->switch_priv;

	if (priv->info->port_tx_pause_config)
		priv->info->port_tx_pause_config(port, tx_pause);
}

static void netc_port_set_rx_pause(struct netc_port *port, bool rx_pause)
{
	u32 old_val, val;

	old_val = netc_mac_port_rd(port, NETC_PM_CMD_CFG(0));
	val = u32_replace_bits(old_val, rx_pause ? 0 : 1, PM_CMD_CFG_PAUSE_IGN);

	if (old_val == val)
		return;

	netc_mac_port_wr(port, NETC_PM_CMD_CFG(0), val);
}

static void netc_port_enable_mac_path(struct netc_port *port,
				      bool enable)
{
	u32 por = enable ? 0 : (PCR_TXDIS | PCR_RXDIS);
	u32 val;

	netc_port_wr(port, NETC_POR, por);
	val = netc_mac_port_rd(port, NETC_PM_CMD_CFG(0));
	if (enable)
		val |= PM_CMD_CFG_TX_EN | PM_CMD_CFG_RX_EN;
	else
		val &= ~(PM_CMD_CFG_TX_EN | PM_CMD_CFG_RX_EN);

	netc_mac_port_wr(port, NETC_PM_CMD_CFG(0), val);
}

static void netc_port_update_mm_link_state(struct netc_port *port, bool link_up)
{
	u32 val;

	if (!port->caps.pmac)
		return;

	guard(mutex)(&port->mm_lock);

	val = netc_port_rd(port, NETC_MAC_MERGE_MMCSR);
	if (link_up) {
		val &= ~MAC_MERGE_MMCSR_LINK_FAIL;
		if (port->offloads & NETC_FLAG_QBU)
			val = u32_replace_bits(val, MMCSR_ME_FP_4B_BOUNDARY,
					       MAC_MERGE_MMCSR_ME);
	} else {
		val |= MAC_MERGE_MMCSR_LINK_FAIL;
		if (port->offloads & NETC_FLAG_QBU)
			val = u32_replace_bits(val, 0, MAC_MERGE_MMCSR_ME);
	}

	netc_port_wr(port, NETC_MAC_MERGE_MMCSR, val);
	netc_port_mm_commit_preemptible_tcs(port);
}

static void netc_mac_link_up(struct phylink_config *config,
			     struct phy_device *phy, unsigned int mode,
			     phy_interface_t interface, int speed, int duplex,
			     bool tx_pause, bool rx_pause)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct netc_switch *priv = dp->ds->priv;
	struct netc_port *port;

	port = NETC_PORT(priv, dp->index);
	netc_port_set_speed(port, speed);

	if (phy_interface_mode_is_rgmii(interface))
		netc_port_force_set_rgmii_mac(port, speed, duplex);

	if (interface == PHY_INTERFACE_MODE_RMII ||
	    interface == PHY_INTERFACE_MODE_REVMII ||
	    interface == PHY_INTERFACE_MODE_MII) {
		net_port_set_rmii_mii_mac(port, speed, duplex);
	}

	if (port->offloads & NETC_FLAG_QBU && duplex == DUPLEX_FULL) {
		/* When preemption is enabled, generation of PAUSE frames
		 * must be disabled, as stated in the IEEE 802.3 standard.
		 */
		tx_pause = false;
	}

	port->tx_pause = tx_pause ? 1 : 0;
	netc_port_set_tx_pause(port, tx_pause);
	netc_port_set_rx_pause(port, rx_pause);
	netc_port_enable_mac_path(port, true);
	netc_port_update_mm_link_state(port, true);

	if (dp->user->features & NETIF_F_HW_HSR_FWD)
		netc_port_stp_state_set(dp->ds, dp->index, BR_STATE_FORWARDING);
}

static void netc_mac_link_down(struct phylink_config *config, unsigned int mode,
			       phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct netc_switch *priv = dp->ds->priv;
	struct netc_port *port;

	port = NETC_PORT(priv, dp->index);
	netc_port_update_mm_link_state(port, false);
	netc_port_enable_mac_path(port, false);
	netc_port_remove_dynamic_entries(port);

	if (dp->user->features & NETIF_F_HW_HSR_FWD)
		netc_port_stp_state_set(dp->ds, dp->index, BR_STATE_DISABLED);
}

static void netc_port_disable_tx_lpi(struct netc_switch *priv,
				     int port_id)
{
	struct netc_port *port = NETC_PORT(priv, port_id);

	netc_mac_port_wr(port, NETC_PM_SLEEP_TIMER(0), 0);
	netc_mac_port_wr(port, NETC_PM_LPWAKE_TIMER(0), 0);
}

static void netc_port_enable_tx_lpi(struct netc_switch *priv,
				    int port_id, u32 timer)
{
	struct netc_port *port = NETC_PORT(priv, port_id);
	u64 clk_freq = priv->info->sysclk_freq;
	u32 sleep_cycles, lpwake_cycles;

	sleep_cycles = netc_us_to_cycles(clk_freq, timer);
	lpwake_cycles = netc_us_to_cycles(clk_freq, NETC_LPWAKE_US);

	netc_mac_port_wr(port, NETC_PM_SLEEP_TIMER(0), sleep_cycles);
	netc_mac_port_wr(port, NETC_PM_LPWAKE_TIMER(0), lpwake_cycles);
}

static void netc_mac_disable_tx_lpi(struct phylink_config *config)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct netc_switch *priv = dp->ds->priv;

	netc_port_disable_tx_lpi(priv, dp->index);
}

static int netc_mac_enable_tx_lpi(struct phylink_config *config,
				  u32 timer, bool tx_clock_stop)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct netc_switch *priv = dp->ds->priv;

	netc_port_enable_tx_lpi(priv, dp->index, timer);

	return 0;
}

static const struct phylink_mac_ops netc_phylink_mac_ops = {
	.mac_select_pcs		= netc_mac_select_pcs,
	.mac_config		= netc_mac_config,
	.mac_link_up		= netc_mac_link_up,
	.mac_link_down		= netc_mac_link_down,
	.mac_disable_tx_lpi	= netc_mac_disable_tx_lpi,
	.mac_enable_tx_lpi	= netc_mac_enable_tx_lpi,
};

static const struct dsa_switch_ops netc_switch_ops = {
	.get_tag_protocol		= netc_get_tag_protocol,
	.connect_tag_protocol		= netc_connect_tag_protocol,
	.setup				= netc_setup,
	.teardown			= netc_teardown,
	.port_enable			= netc_port_enable,
	.port_disable			= netc_port_disable,
	.port_stp_state_set		= netc_port_stp_state_set,
	.phylink_get_caps		= netc_phylink_get_caps,
	.port_change_mtu		= netc_port_change_mtu,
	.port_max_mtu			= netc_port_max_mtu,
	.port_fdb_add			= netc_port_fdb_add,
	.port_fdb_del			= netc_port_fdb_del,
	.port_fdb_dump			= netc_port_fdb_dump,
	.port_mdb_add			= netc_port_mdb_add,
	.port_mdb_del			= netc_port_mdb_del,
	.port_vlan_filtering		= netc_port_vlan_filtering,
	.port_vlan_add			= netc_port_vlan_add,
	.port_vlan_del			= netc_port_vlan_del,
	.set_ageing_time		= netc_set_ageing_time,
	.port_fast_age			= netc_port_fast_age,
	.port_bridge_join		= netc_port_bridge_join,
	.port_bridge_leave		= netc_port_bridge_leave,
	.port_setup_tc			= netc_port_setup_tc,
	.cls_flower_add			= netc_port_cls_flower_add,
	.cls_flower_del			= netc_port_cls_flower_del,
	.cls_flower_stats		= netc_port_cls_flower_stats,
	.get_mm				= netc_port_get_mm,
	.set_mm				= netc_port_set_mm,
	.get_mm_stats			= netc_port_get_mm_stats,
	.get_ts_info			= netc_get_ts_info,
	.port_hwtstamp_set		= netc_port_hwtstamp_set,
	.port_hwtstamp_get		= netc_port_hwtstamp_get,
	.port_rxtstamp			= netc_port_rxtstamp,
	.port_txtstamp			= netc_port_txtstamp,
	.get_pause_stats		= netc_port_get_pause_stats,
	.get_rmon_stats			= netc_port_get_rmon_stats,
	.get_eth_ctrl_stats		= netc_port_get_eth_ctrl_stats,
	.get_eth_mac_stats		= netc_port_get_eth_mac_stats,
	.support_eee			= dsa_supports_eee,
	.set_mac_eee			= netc_port_set_mac_eee,
	.resume				= netc_resume,
	.suspend			= netc_suspend,
	.port_hsr_join			= netc_port_hsr_join,
	.port_hsr_leave			= netc_port_hsr_leave,
};

static int netc_switch_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct netc_switch *priv;
	struct dsa_switch *ds;
	int err;

	if (!node) {
		dev_info(dev, "No DTS bindings or device is disabled, skipping\n");
		return -ENODEV;
	}

	err = netc_switch_check_emdio_is_ready(dev);
	if (err)
		return err;

	err = netc_switch_pci_init(pdev);
	if (err)
		return err;

	priv = pci_get_drvdata(pdev);
	netc_switch_get_ip_revision(priv);

	err = netc_switch_platform_probe(priv);
	if (err)
		goto destroy_netc_switch;

	ds = kzalloc(sizeof(*ds), GFP_KERNEL);
	if (!ds) {
		err = -ENOMEM;
		dev_err(dev, "Failed to allocate DSA switch\n");
		goto destroy_netc_switch;
	}

	ds->dev = dev;
	ds->num_ports = priv->num_ports;
	ds->num_tx_queues = NETC_TC_NUM;
	ds->ops = &netc_switch_ops;
	ds->phylink_mac_ops = &netc_phylink_mac_ops;
	ds->priv = priv;

	priv->ds = ds;
	priv->tag_proto = DSA_TAG_PROTO_NETC;

	err = dsa_register_switch(ds);
	if (err) {
		dev_err(dev, "Failed to register DSA switch, err=%d\n", err);
		goto free_ds;
	}

	netc_create_debugfs(priv);

	return 0;

free_ds:
	kfree(ds);
destroy_netc_switch:
	netc_switch_pci_destroy(pdev);

	return err;
}

static void netc_switch_remove(struct pci_dev *pdev)
{
	struct netc_switch *priv = pci_get_drvdata(pdev);

	if (!priv)
		return;

	netc_remove_debugfs(priv);
	dsa_unregister_switch(priv->ds);
	kfree(priv->ds);
	netc_switch_pci_destroy(pdev);
}

static int netc_switch_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct netc_switch *priv;

	priv = pci_get_drvdata(pdev);

	return dsa_switch_suspend(priv->ds);
}

static int netc_switch_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct netc_switch *priv;

	priv = pci_get_drvdata(pdev);

	return dsa_switch_resume(priv->ds);
}

static const struct pci_device_id netc_switch_ids[] = {
	{ PCI_DEVICE(NETC_SWITCH_VENDOR_ID, NETC_SWITCH_DEVICE_ID) },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, netc_switch_ids);

static DEFINE_SIMPLE_DEV_PM_OPS(netc_switch_pm_ops, netc_switch_suspend,
				netc_switch_resume);

static struct pci_driver netc_switch_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= netc_switch_ids,
	.probe		= netc_switch_probe,
	.remove		= netc_switch_remove,
	.driver.pm	= pm_ptr(&netc_switch_pm_ops),
};
module_pci_driver(netc_switch_driver);

MODULE_DESCRIPTION("NXP NETC Switch driver");
MODULE_LICENSE("Dual BSD/GPL");
