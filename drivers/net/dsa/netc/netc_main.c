// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC switch driver
 * Copyright 2025 NXP
 */

#include <linux/clk.h>
#include <linux/etherdevice.h>
#include <linux/fsl/enetc_mdio.h>
#include <linux/if_bridge.h>
#include <linux/if_vlan.h>
#include <linux/of_mdio.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/unaligned.h>

#include "netc_switch.h"

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

static inline void netc_add_fdb_entry(struct netc_switch *priv,
				      struct netc_fdb_entry *entry)
{
	hlist_add_head(&entry->node, &priv->fdb_list);
}

static inline void netc_del_fdb_entry(struct netc_fdb_entry *entry)
{
	hlist_del(&entry->node);
	kfree(entry);
}

static void netc_destroy_fdb_list(struct netc_switch *priv)
{
	struct netc_fdb_entry *entry;
	struct hlist_node *tmp;

	guard(mutex)(&priv->fdbt_lock);
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

static inline void netc_add_vlan_entry(struct netc_switch *priv,
				       struct netc_vlan_entry *entry)
{
	hlist_add_head(&entry->node, &priv->vlan_list);
}

static inline void netc_del_vlan_entry(struct netc_vlan_entry *entry)
{
	hlist_del(&entry->node);
	kfree(entry);
}

static void netc_destroy_vlan_list(struct netc_switch *priv)
{
	struct netc_vlan_entry *entry;
	struct hlist_node *tmp;

	guard(mutex)(&priv->vft_lock);
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

static u32 netc_mac_port_rd(struct netc_port *port, u32 reg)
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
		phy_mode == PHY_INTERFACE_MODE_1000BASEX ||
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
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;

	/* All tables default to version 0 */
	memset(&cbdrs->tbl, 0, sizeof(cbdrs->tbl));

	if (priv->revision == NETC_SWITCH_REV_4_3)
		cbdrs->tbl.ist_ver = 1;
}

static int netc_init_all_cbdrs(struct netc_switch *priv)
{
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	struct netc_switch_regs *regs = &priv->regs;
	int i, j, err;

	cbdrs->cbdr_num = NETC_CBDR_NUM;
	cbdrs->cbdr_size = NETC_CBDR_BD_NUM;
	cbdrs->ring = kcalloc(cbdrs->cbdr_num, sizeof(*cbdrs->ring),
			      GFP_KERNEL);
	if (!cbdrs->ring)
		return -ENOMEM;

	cbdrs->dma_dev = priv->dev;

	netc_init_ntmp_tbl_versions(priv);

	/* Set the system attributes of reads and writes of command
	 * descriptor and data.
	 */
	netc_base_wr(regs, NETC_CCAR, NETC_DEFAULT_CMD_CACHE_ATTR);

	for (i = 0; i < cbdrs->cbdr_num; i++) {
		struct netc_cbdr *cbdr = &cbdrs->ring[i];
		struct netc_cbdr_regs cbdr_regs;

		cbdr_regs.pir = regs->base + NETC_CBDRPIR(i);
		cbdr_regs.cir = regs->base + NETC_CBDRCIR(i);
		cbdr_regs.mr = regs->base + NETC_CBDRMR(i);
		cbdr_regs.bar0 = regs->base + NETC_CBDRBAR0(i);
		cbdr_regs.bar1 = regs->base + NETC_CBDRBAR1(i);
		cbdr_regs.lenr = regs->base + NETC_CBDRLENR(i);

		err = netc_setup_cbdr(cbdrs->dma_dev, cbdrs->cbdr_size,
				      &cbdr_regs, cbdr);
		if (err) {
			for (j = 0; j < i; j++)
				netc_teardown_cbdr(cbdrs->dma_dev,
						   &cbdrs->ring[j]);

			kfree(cbdrs->ring);
			cbdrs->dma_dev = NULL;
			return err;
		}
	}

	return 0;
}

static void netc_remove_all_cbdrs(struct netc_switch *priv)
{
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	int i;

	for (i = 0; i < NETC_CBDR_NUM; i++)
		netc_teardown_cbdr(cbdrs->dma_dev, &cbdrs->ring[i]);

	cbdrs->dma_dev = NULL;
	kfree(cbdrs->ring);
}

static void netc_get_ntmp_capabilities(struct netc_switch *priv)
{
	struct netc_switch_regs *regs = &priv->regs;
	struct ntmp_priv *ntmp = &priv->ntmp;
	u32 val;

	val = netc_base_rd(regs, NETC_ETTCAPR);
	ntmp->caps.ett_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_ECTCAPR);
	ntmp->caps.ect_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_RPITCAPR);
	ntmp->caps.rpt_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_ISCITCAPR);
	ntmp->caps.isct_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_ISITCAPR);
	ntmp->caps.ist_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_SGIITCAPR);
	ntmp->caps.sgit_num_entries = NETC_GET_NUM_ENTRIES(val);

	val = netc_base_rd(regs, NETC_SGCLITCAPR);
	ntmp->caps.sgclt_num_words = NETC_GET_NUM_WORDS(val);
}

static int netc_init_ntmp_bitmaps(struct netc_switch *priv)
{
	struct ntmp_priv *ntmp = &priv->ntmp;

	ntmp->ett_bitmap_size = ntmp->caps.ett_num_entries / priv->num_ports;
	ntmp->ett_eid_bitmap = bitmap_zalloc(ntmp->ett_bitmap_size, GFP_KERNEL);
	if (!ntmp->ett_eid_bitmap)
		return -ENOMEM;

	ntmp->ect_bitmap_size = ntmp->caps.ect_num_entries / priv->num_ports;
	ntmp->ect_eid_bitmap = bitmap_zalloc(ntmp->ect_bitmap_size, GFP_KERNEL);
	if (!ntmp->ect_eid_bitmap)
		goto free_ett_eid_bitmap;

	ntmp->ist_eid_bitmap = bitmap_zalloc(ntmp->caps.ist_num_entries,
					     GFP_KERNEL);
	if (!ntmp->ist_eid_bitmap)
		goto free_ect_eid_bitmap;

	ntmp->rpt_eid_bitmap = bitmap_zalloc(ntmp->caps.rpt_num_entries,
					     GFP_KERNEL);
	if (!ntmp->rpt_eid_bitmap)
		goto free_ist_eid_bitmap;

	ntmp->sgit_eid_bitmap = bitmap_zalloc(ntmp->caps.sgit_num_entries,
					      GFP_KERNEL);
	if (!ntmp->sgit_eid_bitmap)
		goto free_rpt_eid_bitmap;

	ntmp->isct_eid_bitmap = bitmap_zalloc(ntmp->caps.isct_num_entries,
					      GFP_KERNEL);
	if (!ntmp->isct_eid_bitmap)
		goto free_sgit_eid_bitmap;

	ntmp->sgclt_word_bitmap = bitmap_zalloc(ntmp->caps.sgclt_num_words,
						GFP_KERNEL);
	if (!ntmp->sgclt_word_bitmap)
		goto free_isct_eid_bitmap;

	return 0;

free_isct_eid_bitmap:
	bitmap_free(ntmp->isct_eid_bitmap);
	ntmp->isct_eid_bitmap = NULL;
free_sgit_eid_bitmap:
	bitmap_free(ntmp->sgit_eid_bitmap);
	ntmp->sgit_eid_bitmap = NULL;
free_rpt_eid_bitmap:
	bitmap_free(ntmp->rpt_eid_bitmap);
	ntmp->rpt_eid_bitmap = NULL;
free_ist_eid_bitmap:
	bitmap_free(ntmp->ist_eid_bitmap);
	ntmp->ist_eid_bitmap = NULL;
free_ect_eid_bitmap:
	bitmap_free(ntmp->ect_eid_bitmap);
	ntmp->ect_eid_bitmap = NULL;
free_ett_eid_bitmap:
	bitmap_free(ntmp->ett_eid_bitmap);
	ntmp->ett_eid_bitmap = NULL;

	return -ENOMEM;
}

static void netc_free_ntmp_bitmaps(struct netc_switch *priv)
{
	struct ntmp_priv *ntmp = &priv->ntmp;

	bitmap_free(ntmp->sgclt_word_bitmap);
	ntmp->sgclt_word_bitmap = NULL;

	bitmap_free(ntmp->isct_eid_bitmap);
	ntmp->isct_eid_bitmap = NULL;

	bitmap_free(ntmp->sgit_eid_bitmap);
	ntmp->sgit_eid_bitmap = NULL;

	bitmap_free(ntmp->rpt_eid_bitmap);
	ntmp->rpt_eid_bitmap = NULL;

	bitmap_free(ntmp->ist_eid_bitmap);
	ntmp->ist_eid_bitmap = NULL;

	bitmap_free(ntmp->ect_eid_bitmap);
	ntmp->ect_eid_bitmap = NULL;

	bitmap_free(ntmp->ett_eid_bitmap);
	ntmp->ett_eid_bitmap = NULL;
}

struct pci_dev *netc_switch_get_timer(struct netc_switch *priv)
{
	int domain = pci_domain_nr(priv->pdev->bus);
	u32 devfn = priv->info->tmr_devfn;
	u8 bus = priv->pdev->bus->number;

	return pci_get_domain_bus_and_slot(domain, bus, devfn);
}

static u64 netc_switch_adjust_base_time(struct ntmp_priv *ntmp, u64 base_time,
					u32 cycle_time)
{
	struct netc_switch *priv = ntmp_to_netc_switch(ntmp);
	u64 current_time, delta, n;
	struct pci_dev *tmr_dev;

	tmr_dev = netc_switch_get_timer(priv);
	if (!tmr_dev)
		return base_time;

	current_time = netc_timer_get_current_time(tmr_dev);
	if (base_time >= current_time)
		return base_time;

	delta = current_time - base_time;
	n = DIV_ROUND_UP_ULL(delta, cycle_time);
	base_time += (n * (u64)cycle_time);

	return base_time;
}

static u32 netc_switch_get_tgst_free_words(struct ntmp_priv *ntmp)
{
	struct netc_switch *priv = ntmp_to_netc_switch(ntmp);
	struct netc_switch_regs *regs = &priv->regs;
	u32 words_in_use;
	u32 total_words;

	total_words = netc_base_rd(regs, NETC_TGSTCAPR);
	total_words = NETC_GET_NUM_WORDS(total_words);

	words_in_use = netc_base_rd(regs, NETC_TGSTMOR);
	words_in_use = NETC_GET_NUM_WORDS(words_in_use);

	return total_words - words_in_use;
}

static int netc_init_ntmp_priv(struct netc_switch *priv)
{
	struct ntmp_priv *ntmp = &priv->ntmp;
	int err;

	ntmp->dev_type = NETC_DEV_SWITCH;

	err = netc_init_all_cbdrs(priv);
	if (err)
		return err;

	netc_get_ntmp_capabilities(priv);
	err = netc_init_ntmp_bitmaps(priv);
	if (err)
		goto free_all_cbdrs;

	ntmp->adjust_base_time = netc_switch_adjust_base_time;
	ntmp->get_tgst_free_words = netc_switch_get_tgst_free_words;

	INIT_HLIST_HEAD(&ntmp->flower_list);
	mutex_init(&ntmp->flower_lock);

	return 0;

free_all_cbdrs:
	netc_remove_all_cbdrs(priv);

	return err;
}

static void netc_deinit_ntmp_priv(struct netc_switch *priv)
{
	netc_destroy_flower_list(priv);
	mutex_destroy(&priv->ntmp.flower_lock);
	netc_free_ntmp_bitmaps(priv);
	netc_remove_all_cbdrs(priv);
}

static void netc_clean_fdbt_aging_entries(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct netc_switch *priv;

	priv = container_of(dwork, struct netc_switch, fdbt_clean);

	/* We should first update the activity element in FDB table */
	scoped_guard(mutex, &priv->fdbt_lock) {
		ntmp_fdbt_update_activity_element(&priv->ntmp.cbdrs);

		/* After the activity element is updated, we delete the aging
		 * entries in the FDB table.
		 */
		ntmp_fdbt_delete_aging_entries(&priv->ntmp.cbdrs,
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

static void netc_port_set_tc_max_sdu(struct netc_port *port,
				     int tc, u32 max_sdu)
{
	u32 val;

	val = max_sdu + ETH_HLEN + ETH_FCS_LEN;
	if (dsa_port_is_cpu(port->dp))
		val += NETC_TAG_MAX_LEN;

	val &= PTCTMSDUR_MAXSDU;
	val = u32_replace_bits(val, SDU_TYPE_MPDU, PTCTMSDUR_SDU_TYPE);
	netc_port_wr(port, NETC_PTCTMSDUR(tc), val);
}

void netc_port_set_all_tc_msdu(struct netc_port *port, u32 *max_sdu)
{
	u32 msdu = NETC_MAX_FRAME_LEN;
	int tc;

	for (tc = 0; tc < NETC_TC_NUM; tc++) {
		if (max_sdu)
			msdu = max_sdu[tc] + VLAN_ETH_HLEN;

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

static void netc_port_default_config(struct netc_port *port)
{
	u32 pqnt = 0xffff, qth = 0xffff / 2;
	u32 val;

	/* Default VLAN unware */
	val = netc_port_rd(port, NETC_BPDVR);
	if (!(val & BPDVR_RXVAM)) {
		val |= BPDVR_RXVAM;
		netc_port_wr(port, NETC_BPDVR, val);
	}

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

	/* Default buffer pool mapping */
	netc_port_wr(port, NETC_PBPMCR0, NETC_DEFULT_BUFF_POOL_MAP0);
	netc_port_wr(port, NETC_PBPMCR1, NETC_DEFULT_BUFF_POOL_MAP1);

	if (dsa_port_is_user(port->dp)) {
		/* Enable ingress port filter table lookup */
		netc_port_wr(port, NETC_PIPFCR, PIPFCR_EN);

		/* Set the quanta value of tx PAUSE frame */
		netc_port_wr(port, NETC_PM_PAUSE_QUANTA(0), pqnt);

		/* When a quanta timer counts down and reaches this value, the MAC
		 * sends a refresh PAUSE frame with the programmed full quanta value
		 * if a pause condition still exists.
		 */
		netc_port_wr(port, NETC_PM_PAUSE_TRHESH(0), qth);
		netc_port_set_mlo(port, MLO_DISABLE);
	} else {
		val = netc_port_rd(port, NETC_BPCR);
		val |= BPCR_SRCPRND;
		netc_port_wr(port, NETC_BPCR, val);

		netc_port_set_mlo(port, MLO_HW);
	}

	netc_port_set_max_frame_size(port, NETC_MAX_FRAME_LEN);
	netc_port_set_all_tc_msdu(port, NULL);
}

static int netc_switch_bpt_default_config(struct netc_switch *priv)
{
	struct bpt_cfge_data *cfge;
	int i;

	priv->bpt_list = devm_kcalloc(priv->dev, priv->caps.num_bp,
				      sizeof(*cfge), GFP_KERNEL);
	if (!priv->bpt_list)
		return -ENOMEM;

	guard(mutex)(&priv->bpt_lock);
	for (i = 0; i < priv->caps.num_bp; i++) {
		cfge = &priv->bpt_list[i];
		/* FC enabled using only buffer pool FC state */
		cfge->fccfg_sbpen = FIELD_PREP(BPT_FC_CFG, BPT_FC_CFG_EN_BPFC);
		cfge->fc_on_thresh = cpu_to_le16(NETC_PORT_FC_ON_THRESH);
		cfge->fc_off_thresh = cpu_to_le16(NETC_PORT_FC_OFF_THRESH);

		ntmp_bpt_update_entry(&priv->ntmp.cbdrs, i, cfge);
	}

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

	err = netc_init_ntmp_priv(priv);
	if (err)
		goto free_internal_mdiobus;

	INIT_HLIST_HEAD(&priv->fdb_list);
	mutex_init(&priv->fdbt_lock);
	INIT_HLIST_HEAD(&priv->vlan_list);
	mutex_init(&priv->vft_lock);
	priv->fdbt_acteu_interval = NETC_FDBT_CLEAN_INTERVAL;
	priv->fdbt_aging_act_cnt = NETC_FDBT_AGING_ACT_CNT;
	INIT_DELAYED_WORK(&priv->fdbt_clean, netc_clean_fdbt_aging_entries);
	mutex_init(&priv->bpt_lock);

	netc_switch_dos_default_config(priv);
	netc_switch_vfht_default_config(priv);
	netc_switch_isit_key_config(priv);

	/* default setting for ports */
	for (i = 0; i < priv->num_ports; i++) {
		port = priv->ports[i];
		if (port->dp)
			netc_port_default_config(port);
	}

	err = netc_switch_bpt_default_config(priv);
	if (err)
		goto deinit_ntmp_priv;

	schedule_delayed_work(&priv->fdbt_clean, priv->fdbt_acteu_interval);

	ds->fdb_isolation = true;

	return 0;

deinit_ntmp_priv:
	netc_deinit_ntmp_priv(priv);
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

static void netc_teardown(struct dsa_switch *ds)
{
	struct netc_switch *priv = ds->priv;

	cancel_delayed_work_sync(&priv->fdbt_clean);
	netc_destroy_all_lists(priv);
	netc_deinit_ntmp_priv(priv);
	netc_remove_all_ports_internal_mdiobus(ds);
}

static bool netc_switch_is_emdio_consumer(struct device_node *ports)
{
	struct device_node *phy_node, *mdio_node;

	for_each_available_child_of_node_scoped(ports, child) {
		/* If the node does not have phy-handle property, then
		 * the port does not connect to a PHY, so the port is
		 * not the EMDIO consumer.
		 */
		phy_node = of_parse_phandle(child, "phy-handle", 0);
		if (!phy_node)
			continue;

		of_node_put(phy_node);
		/* If the port node has phy-handle property and it does
		 * not contain a mdio child node, then the switch is the
		 * EMDIO consumer.
		 */
		mdio_node = of_get_child_by_name(child, "mdio");
		if (!mdio_node)
			return true;

		of_node_put(mdio_node);

		return false;
	}

	return false;
}

static int netc_switch_add_emdio_consumer(struct device *dev)
{
	struct phy_device *phydev = NULL, *last_phydev = NULL;
	struct device_node *node = dev->of_node;
	struct device_node *ports, *phy_node;
	struct device_link *link;
	int err = 0;

	ports = of_get_child_by_name(node, "ports");
	if (!ports)
		ports = of_get_child_by_name(node, "ethernet-ports");
	if (!ports)
		return 0;

	if (!netc_switch_is_emdio_consumer(ports))
		goto out;

	for_each_available_child_of_node_scoped(ports, child) {
		phy_node = of_parse_phandle(child, "phy-handle", 0);
		if (!phy_node)
			continue;

		phydev = of_phy_find_device(phy_node);
		of_node_put(phy_node);
		if (!phydev) {
			err = -EPROBE_DEFER;
			goto out;
		}

		if (last_phydev) {
			put_device(&last_phydev->mdio.dev);
			last_phydev = phydev;
		}
	}

	if (phydev) {
		link = device_link_add(dev, phydev->mdio.bus->parent,
				       DL_FLAG_PM_RUNTIME |
				       DL_FLAG_AUTOREMOVE_SUPPLIER);
		put_device(&phydev->mdio.dev);
		if (!link) {
			err = -EINVAL;
			goto out;
		}
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

static int netc_switch_add_vlan_egress_rule(struct netc_switch *priv,
					    struct netc_vlan_entry *entry)
{
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	struct ett_cfge_data ett_cfge = {0};
	u32 ett_base_eid, ect_base_eid;
	u32 ett_eid, vuda_sqta;
	u16 efm_cfg;
	int i, err;

	/* step1: find available ect entries and update these entries */
	ect_base_eid = ntmp_lookup_free_eid(priv->ntmp.ect_eid_bitmap,
					    priv->ntmp.ect_bitmap_size);
	if (ect_base_eid == NTMP_NULL_ENTRY_ID) {
		dev_warn(priv->dev, "No ECT entries available\n");
	} else {
		ect_base_eid *= priv->num_ports;
		for (i = 0; i < priv->num_ports; i++)
			/* Reset the counters of ECT entry */
			ntmp_ect_update_entry(cbdrs, ect_base_eid + i);
	}

	/* step2: find available ett entries and add these entries */
	ett_base_eid = ntmp_lookup_free_eid(priv->ntmp.ett_eid_bitmap,
					    priv->ntmp.ett_bitmap_size);
	if (ett_base_eid == NTMP_NULL_ENTRY_ID) {
		dev_err(priv->dev, "No free ETT entries found\n");
		err = -ENOSPC;
		goto clear_ect_eid;
	}

	ett_base_eid *=  priv->num_ports;
	ett_eid = ett_base_eid;
	for (i = 0; i < priv->num_ports; i++, ett_eid++) {
		/* Specify the FMT entry ID format */
		vuda_sqta = FMTEID_VUDA_SQTA;
		efm_cfg = 0;

		if (ect_base_eid != NTMP_NULL_ENTRY_ID) {
			/* Increase egress frame counter */
			efm_cfg |= FIELD_PREP(ETT_ECA, ETT_ECA_INC);
			ett_cfge.ec_eid = cpu_to_le32(ett_eid);
		}

		/* If egress rule is VLAN untagged */
		if (entry->untagged_port_bitmap & BIT(i)) {
			/* delete outer VLAN tag */
			vuda_sqta |= FIELD_PREP(FMTEID_VUDA,
						FMTEID_VUDA_DEL_OTAG);
			/* length change: twos-complement notation */
			efm_cfg |= FIELD_PREP(ETT_EFM_LEN_CHANGE,
					      ETT_FRM_LEN_DEL_VLAN);
		}

		ett_cfge.efm_eid = cpu_to_le32(vuda_sqta);
		ett_cfge.esqa_tgt_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
		ett_cfge.efm_cfg = cpu_to_le16(efm_cfg);

		/* Add an ETT entry */
		err = ntmp_ett_add_or_update_entry(cbdrs, ett_eid, true, &ett_cfge);
		if (err)
			goto clear_ett_entries;
	}

	entry->cfge.et_eid = cpu_to_le32(ett_base_eid);
	entry->ect_base_eid = ect_base_eid;

	return 0;

clear_ett_entries:
	ntmp_clear_eid_bitmap(priv->ntmp.ett_eid_bitmap, ett_base_eid);
	for (i--, ett_eid--; i >= 0; i--, ett_eid--)
		ntmp_ett_delete_entry(cbdrs, ett_eid);

clear_ect_eid:
	/* ECT is a static index table, no need to delete the entries */
	ntmp_clear_eid_bitmap(priv->ntmp.ect_eid_bitmap, ect_base_eid);

	return err;
}

static void netc_switch_delete_vlan_egress_rule(struct netc_switch *priv,
						struct netc_vlan_entry *entry)
{
	u32 ett_eid_bit, ect_eid_bit;
	u32 ett_eid, ect_eid;
	int i;

	ett_eid = le32_to_cpu(entry->cfge.et_eid);
	if (ett_eid == NTMP_NULL_ENTRY_ID)
		return;

	ett_eid_bit = ett_eid / priv->num_ports;
	ntmp_clear_eid_bitmap(priv->ntmp.ett_eid_bitmap, ett_eid_bit);
	for (i = 0; i < priv->num_ports; i++) {
		ett_eid += i;
		ntmp_ett_delete_entry(&priv->ntmp.cbdrs, ett_eid);
	}

	entry->cfge.et_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);

	ect_eid = entry->ect_base_eid;
	if (ect_eid == NTMP_NULL_ENTRY_ID)
		return;

	ect_eid_bit = ect_eid / priv->num_ports;
	ntmp_clear_eid_bitmap(priv->ntmp.ect_eid_bitmap, ect_eid_bit);
	entry->ect_base_eid = NTMP_NULL_ENTRY_ID;
}

static int netc_port_update_vlan_egress_rule(struct netc_port *port,
					     struct netc_vlan_entry *entry)
{
	struct netc_switch *priv = port->switch_priv;
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	struct ett_cfge_data ett_cfge = {0};
	u32 ett_eid, ect_eid, vuda_sqta;
	u16 efm_cfg = 0;

	ett_eid = le32_to_cpu(entry->cfge.et_eid);
	if (ett_eid == NTMP_NULL_ENTRY_ID)
		return 0;

	ett_eid += port->index;
	ect_eid = entry->ect_base_eid;
	if (ect_eid != NTMP_NULL_ENTRY_ID) {
		ect_eid += port->index;
		ntmp_ect_update_entry(cbdrs, ect_eid);

		efm_cfg |= FIELD_PREP(ETT_ECA, ETT_ECA_INC);
		ett_cfge.ec_eid = cpu_to_le32(ect_eid);
	}

	/* Specify the FMT entry ID format */
	vuda_sqta = FMTEID_VUDA_SQTA;
	/* If egress rule is VLAN untagged */
	if (entry->untagged_port_bitmap & BIT(port->index)) {
		/* delete outer VLAN tag */
		vuda_sqta |= FIELD_PREP(FMTEID_VUDA, FMTEID_VUDA_DEL_OTAG);
		/* length change: twos-complement notation */
		efm_cfg |= FIELD_PREP(ETT_EFM_LEN_CHANGE, ETT_FRM_LEN_DEL_VLAN);
	}

	ett_cfge.efm_cfg = cpu_to_le16(efm_cfg);
	ett_cfge.efm_eid = cpu_to_le32(vuda_sqta);
	ett_cfge.esqa_tgt_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);

	/* Add an ETT entry */
	return ntmp_ett_add_or_update_entry(cbdrs, ett_eid, false, &ett_cfge);
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
	entry->ect_base_eid = NTMP_NULL_ENTRY_ID;
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

	err = ntmp_vft_add_entry(&priv->ntmp.cbdrs, &entry->entry_id,
				 vid, &entry->cfge);
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
	err = ntmp_vft_update_entry(&priv->ntmp.cbdrs, vid, &entry->cfge);
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
		ntmp_vft_delete_entry(&priv->ntmp.cbdrs, vid);
		if (vid != NETC_STANDALONE_PVID)
			netc_switch_delete_vlan_egress_rule(priv, entry);

		netc_del_vlan_entry(entry);

		return 0;
	}

	if (!(vlan_port_bitmap & BIT(port_id)))
		return 0;

	entry->cfge.bitmap_stg ^= cpu_to_le32(BIT(port_id));
	err = ntmp_vft_update_entry(&priv->ntmp.cbdrs, vid, &entry->cfge);
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

	err = ntmp_fdbt_add_entry(&priv->ntmp.cbdrs, &entry->entry_id,
				  keye, cfge);
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
	err = ntmp_fdbt_update_entry(&priv->ntmp.cbdrs, entry->entry_id,
				     &entry->cfge);
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
		err = ntmp_fdbt_update_entry(&priv->ntmp.cbdrs, entry->entry_id,
					     &entry->cfge);
		if (err) {
			port_bitmap ^= BIT(port_id);
			entry->cfge.port_bitmap = cpu_to_le32(port_bitmap);

			return err;
		}

	} else {
		/* If the entry only exists on this port, just delete
		 * it from the FDB table.
		 */
		err = ntmp_fdbt_delete_entry(&priv->ntmp.cbdrs, entry->entry_id);
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

static struct net_device *netc_port_get_net_device(struct netc_port *port)
{
	if (dsa_port_is_cpu(port->dp))
		return port->dp->conduit;
	else
		return port->dp->user;
}

static void netc_port_set_mac_address(struct netc_port *port)
{
	struct net_device *ndev;
	u32 upper;
	u16 lower;

	ndev = netc_port_get_net_device(port);
	lower = get_unaligned_le16(ndev->dev_addr + 4);
	upper = get_unaligned_le32(ndev->dev_addr);

	netc_port_wr(port, NETC_PMAR0, upper);
	netc_port_wr(port, NETC_PMAR1, lower);
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

	netc_port_set_mac_address(port);
	netc_port_wr(port, NETC_POR, 0);

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

	netc_port_wr(port, NETC_POR, PCR_TXDIS | PCR_RXDIS);
	clk_disable_unprepare(port->ref_clk);

	if (dsa_is_cpu_port(ds, port_id)) {
		netc_port_del_vlan_entry(port, NETC_VLAN_UNAWARE_PVID);
		netc_port_del_bcast_fdb_entry(port, NETC_STANDALONE_PVID);
	}

	netc_port_del_vlan_entry(port, NETC_STANDALONE_PVID);
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
	u32 max_frame_size = new_mtu + ETH_HLEN + ETH_FCS_LEN;

	netc_port_set_max_frame_size(port, max_frame_size);

	return 0;
}

static int netc_port_max_mtu(struct dsa_switch *ds, int port_id)
{
	int mtu = NETC_MAX_FRAME_LEN - ETH_HLEN - ETH_FCS_LEN;

	if (dsa_is_cpu_port(ds, port_id))
		mtu -= NETC_TAG_MAX_LEN;

	return mtu;
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
	struct fdbt_query_data *entry_data __free(kfree);
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
		err = ntmp_fdbt_search_port_entry(&priv->ntmp.cbdrs, port_id,
						  &resume_eid, &entry_id, entry_data);
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
	struct netc_port *port;
	int i;

	for (i = 0; i < priv->num_ports; i++) {
		port = priv->ports[i];
		if (dsa_port_is_user(port->dp) && port->bridge)
			return false;
	}

	return true;
}

static bool netc_user_ports_vlan_aware(struct netc_switch *priv)
{
	struct netc_port *port;
	int i;

	for (i = 0; i < priv->num_ports; i++) {
		port = priv->ports[i];
		if (dsa_port_is_user(port->dp) && port->vlan_aware)
			return true;
	}

	return false;
}

static void netc_cpu_port_set_vlan_filtering(struct netc_switch *priv)
{
	struct netc_port *port;
	bool vlan_aware;
	u16 pvid;
	u32 val;
	int i;

	vlan_aware = netc_user_ports_vlan_aware(priv);

	for (i = 0; i < priv->num_ports; i++) {
		port = priv->ports[i];
		if (dsa_port_is_cpu(port->dp)) {
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
	ntmp_fdbt_delete_port_dynamic_entries(&priv->ntmp.cbdrs, port->index);
}

static void netc_port_fast_age(struct dsa_switch *ds, int port_id)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);

	netc_port_remove_dynamic_entries(port);
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

static int netc_suspend(struct dsa_switch *ds)
{
	struct netc_switch *priv = NETC_PRIV(ds);
	struct pci_dev *pdev = priv->pdev;
	int port_id;

	cancel_delayed_work_sync(&priv->fdbt_clean);

	for (port_id = 0; port_id < ds->num_ports; port_id++) {
		struct netc_port *port = NETC_PORT(priv, port_id);
		struct net_device *ndev;

		if (!port->dp)
			continue;

		ndev = netc_port_get_net_device(port);
		if (netif_running(ndev))
			netc_port_disable(ds, port_id);
	}

	netc_destroy_fdb_list(priv);
	netc_destroy_vlan_list(priv);
	netc_deinit_ntmp_priv(priv);
	pci_disable_device(pdev);

	return 0;
}

static int netc_resume(struct dsa_switch *ds)
{
	struct netc_switch *priv = NETC_PRIV(ds);
	struct pci_dev *pdev = priv->pdev;
	struct device *dev = &pdev->dev;
	struct netc_port *cpu_port;
	int port_id, err;

	pcie_flr(pdev);
	err = pci_enable_device_mem(pdev);
	if (err)
		return dev_err_probe(dev, err, "Failed to enable device\n");

	pci_set_master(pdev);

	err = netc_init_ntmp_priv(priv);
	if (err)
		return err;

	netc_switch_dos_default_config(priv);
	netc_switch_vfht_default_config(priv);
	netc_switch_isit_key_config(priv);

	err = netc_switch_bpt_default_config(priv);
	if (err)
		goto deinit_ntmp_priv;

	cpu_port = NETC_PORT(priv, ds->num_ports - 1);
	for (port_id = 0; port_id < ds->num_ports; port_id++) {
		struct netc_port *port = NETC_PORT(priv, port_id);
		struct net_device *ndev;
		u16 pvid;

		if (!port->dp)
			continue;

		netc_port_default_config(port);
		ndev = netc_port_get_net_device(port);
		if (netif_running(ndev)) {
			err = netc_port_enable(ds, port_id, NULL);
			if (err)
				goto deinit_ntmp_priv;

			if (port->bridge)
				pvid = port->vlan_aware ? NETC_CPU_PORT_PVID :
				       NETC_VLAN_UNAWARE_PVID;
			else
				pvid = NETC_STANDALONE_PVID;

			err = netc_port_set_fdb_entry(cpu_port, ndev->dev_addr,
						      pvid);
			if (err)
				goto deinit_ntmp_priv;
		}
	}

	schedule_delayed_work(&priv->fdbt_clean, priv->fdbt_acteu_interval);

	return 0;

deinit_ntmp_priv:
	netc_deinit_ntmp_priv(priv);

	return err;
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
		/* We need to enable auto-negotiation for the MAC
		 * if its RGMII interface support In-Band status.
		 */
		if (phylink_autoneg_inband(mode))
			val |= PM_IF_MODE_ENA;
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
	u32 val;

	if (port->speed == speed)
		return;

	val = netc_port_rd(port, NETC_PCR);
	val &= ~PCR_PSPEED;

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

static void netc_port_set_hd_flow_control(struct netc_port *port,
					  bool enable)
{
	u32 old_val, val;

	if (!port->caps.half_duplex)
		return;

	old_val = netc_mac_port_rd(port, NETC_PM_CMD_CFG(0));
	val = u32_replace_bits(old_val, enable ? 1 : 0, PM_CMD_CFG_HD_FCEN);
	if (val == old_val)
		return;

	netc_mac_port_wr(port, NETC_PM_CMD_CFG(0), val);
}

void netc_port_set_tx_pause(struct netc_port *port, bool tx_pause)
{
	struct netc_switch *priv = port->switch_priv;
	struct bpt_cfge_data *cfge;
	int i;

	guard(mutex)(&priv->bpt_lock);
	for (i = 0; i < priv->caps.num_bp; i++) {
		cfge = &priv->bpt_list[i];
		if (tx_pause)
			cfge->fc_ports |= cpu_to_le32(BIT(port->index));
		else
			cfge->fc_ports &= cpu_to_le32(~BIT(port->index));

		ntmp_bpt_update_entry(&priv->ntmp.cbdrs, i, cfge);
	}
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
	u32 val;

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
	bool hd_fc = false;

	port = NETC_PORT(priv, dp->index);
	netc_port_set_speed(port, speed);

	if (phy_interface_mode_is_rgmii(interface) &&
	    !phylink_autoneg_inband(mode)) {
		netc_port_force_set_rgmii_mac(port, speed, duplex);
	}

	if (interface == PHY_INTERFACE_MODE_RMII ||
	    interface == PHY_INTERFACE_MODE_REVMII ||
	    interface == PHY_INTERFACE_MODE_MII) {
		net_port_set_rmii_mii_mac(port, speed, duplex);
	}

	if (duplex == DUPLEX_HALF) {
		if (tx_pause || rx_pause)
			hd_fc = true;

		/* As per 802.3 annex 31B, PAUSE frames are only supported
		 * when the link is configured for full duplex operation.
		 */
		tx_pause = false;
		rx_pause = false;
	} else if (duplex == DUPLEX_FULL) {
		/* When preemption is enabled, generation of PAUSE frames
		 * must be disabled, as stated in the IEEE 802.3 standard.
		 */
		if (port->offloads & NETC_FLAG_QBU)
			tx_pause = false;
	}

	port->tx_pause = tx_pause ? 1 : 0;
	netc_port_set_hd_flow_control(port, hd_fc);
	netc_port_set_tx_pause(port, tx_pause);
	netc_port_set_rx_pause(port, rx_pause);
	netc_port_enable_mac_path(port, true);
	netc_port_update_mm_link_state(port, true);
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
}

static const struct phylink_mac_ops netc_phylink_mac_ops = {
	.mac_select_pcs		= netc_mac_select_pcs,
	.mac_config		= netc_mac_config,
	.mac_link_up		= netc_mac_link_up,
	.mac_link_down		= netc_mac_link_down,
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
	.resume				= netc_resume,
	.suspend			= netc_suspend,
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

	err = netc_switch_add_emdio_consumer(dev);
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

static int __maybe_unused netc_switch_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct netc_switch *priv;

	priv = pci_get_drvdata(pdev);

	return dsa_switch_suspend(priv->ds);
}

static int __maybe_unused netc_switch_resume(struct device *dev)
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

static SIMPLE_DEV_PM_OPS(netc_switch_pm_ops, netc_switch_suspend,
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
