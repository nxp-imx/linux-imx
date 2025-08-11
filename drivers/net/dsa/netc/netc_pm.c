// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Power Management of NXP NETC switch driver
 * Copyright 2025 NXP
 */

#include <linux/clk.h>

#include "netc_switch.h"

static void netc_port_save_config_to_db(struct netc_port *port)
{
	struct netc_port_db *db = &port->db;
	int i;

	if (!port->dp)
		return;

	db->bpdvr = netc_port_rd(port, NETC_BPDVR);
	db->bpcr = netc_port_rd(port, NETC_BPCR);
	db->maxfrm = netc_mac_port_rd(port, NETC_PM_MAXFRM(0));
	db->bpstgsr = netc_port_rd(port, NETC_BPSTGSR);
	db->ptgscr = netc_port_rd(port, NETC_PTGSCR);

	for (i = 0; i < NETC_TC_NUM; i++) {
		db->ptctmsdur[i] = netc_port_rd(port, NETC_PTCTMSDUR(i));
		db->ptccbsr1[i] = netc_port_rd(port, NETC_PTCCBSR1(i));
		db->ptccbsr2[i] = netc_port_rd(port, NETC_PTCCBSR2(i));
	}

	db->mmcsr = netc_port_rd(port, NETC_MAC_MERGE_MMCSR);
	db->ptp_filter = port->ptp_filter;
	port->ptp_filter = HWTSTAMP_FILTER_NONE;
}

static int netc_port_restore_config_from_db(struct netc_port *port)
{
	struct netc_port_db *db = &port->db;
	int i;

	netc_port_wr(port, NETC_BPDVR, db->bpdvr);
	netc_port_wr(port, NETC_BPCR, db->bpcr);
	netc_mac_port_wr(port, NETC_PM_MAXFRM(0), db->maxfrm);
	netc_port_wr(port, NETC_BPSTGSR, db->bpstgsr);

	for (i = 0; i < NETC_TC_NUM; i++) {
		netc_port_wr(port, NETC_PTCTMSDUR(i), db->ptctmsdur[i]);
		netc_port_wr(port, NETC_PTCCBSR1(i), db->ptccbsr1[i]);
		netc_port_wr(port, NETC_PTCCBSR2(i), db->ptccbsr2[i]);
	}

	netc_port_wr(port, NETC_PTGSCR, db->ptgscr);
	netc_port_wr(port, NETC_MAC_MERGE_MMCSR, db->mmcsr);

	return netc_port_set_ptp_filter(port, db->ptp_filter);
}

static void netc_restore_bpt_entries(struct netc_switch *priv)
{
	int i;

	mutex_lock(&priv->bpt_lock);

	for (i = 0; i < priv->caps.num_bp; i++) {
		struct bpt_cfge_data *cfge = &priv->bpt_list[i];

		ntmp_bpt_update_entry(&priv->ntmp.cbdrs, i, cfge);
	}

	mutex_unlock(&priv->bpt_lock);
}

static int netc_restore_vlan_egress_rule(struct netc_switch *priv,
					 struct netc_vlan_entry *entry)
{
	u32 ett_base_eid = le32_to_cpu(entry->cfge.et_eid);
	u32 ect_base_eid = NTMP_NULL_ENTRY_ID;

	if (entry->ect_gid != NTMP_NULL_ENTRY_ID)
		ect_base_eid = entry->ect_gid * priv->num_ports;

	return netc_add_ett_group_entries(priv, entry->untagged_port_bitmap,
					  ett_base_eid, ect_base_eid);
}

static int netc_restore_vlan_entry(struct netc_switch *priv,
				   struct netc_vlan_entry *entry)
{
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	struct device *dev = priv->dev;
	u16 vid = entry->vid;
	int err;

	if (vid != NETC_STANDALONE_PVID) {
		err = netc_restore_vlan_egress_rule(priv, entry);
		if (err) {
			dev_err(dev, "Failed to restore VLAN %u egress rule\n",
				vid);
			return err;
		}
	}

	err = ntmp_vft_add_entry(cbdrs, &entry->entry_id, vid, &entry->cfge);
	if (err) {
		dev_err(dev, "Failed to restore VFT entry, VLAN %u\n", vid);
		goto del_vlan_egress_rule;
	}

	return 0;

del_vlan_egress_rule:
	if (vid != NETC_STANDALONE_PVID)
		netc_switch_delete_vlan_egress_rule(priv, entry);

	return err;
}

static int netc_restore_vlan_entries(struct netc_switch *priv)
{
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	struct netc_vlan_entry *entry, *tmp;
	int err;

	mutex_lock(&priv->vft_lock);

	hlist_for_each_entry(entry, &priv->vlan_list, node) {
		err = netc_restore_vlan_entry(priv, entry);
		if (err)
			goto del_vlan_entries;
	}

	mutex_unlock(&priv->vft_lock);

	return 0;

del_vlan_entries:
	hlist_for_each_entry(tmp, &priv->vlan_list, node) {
		if (tmp == entry)
			break;

		ntmp_vft_delete_entry(cbdrs, tmp->vid);

		if (tmp->vid != NETC_STANDALONE_PVID)
			netc_switch_delete_vlan_egress_rule(priv, tmp);
	}

	netc_destroy_vlan_list(priv);
	bitmap_zero(priv->ntmp.ect_gid_bitmap, priv->ntmp.ect_bitmap_size);
	bitmap_zero(priv->ntmp.ett_gid_bitmap, priv->ntmp.ett_bitmap_size);

	mutex_unlock(&priv->vft_lock);

	return err;
}

static void netc_remove_vlan_entries(struct netc_switch *priv)
{
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	struct netc_vlan_entry *entry;
	struct hlist_node *tmp;

	mutex_lock(&priv->vft_lock);

	hlist_for_each_entry_safe(entry, tmp, &priv->vlan_list, node) {
		ntmp_vft_delete_entry(cbdrs, entry->vid);

		if (entry->vid != NETC_STANDALONE_PVID)
			netc_switch_delete_vlan_egress_rule(priv, entry);

		netc_del_vlan_entry(entry);
	}

	mutex_unlock(&priv->vft_lock);
}

static int netc_restore_fdbt_entries(struct netc_switch *priv)
{
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	struct netc_fdb_entry *entry, *tmp;
	int err;

	mutex_lock(&priv->fdbt_lock);

	hlist_for_each_entry(entry, &priv->fdb_list, node) {
		err = ntmp_fdbt_add_entry(cbdrs, &entry->entry_id,
					  &entry->keye, &entry->cfge);
		if (err) {
			dev_err(priv->dev,
				"Failed to restore FDBT entry, mac: %pm vid: %u\n",
				entry->keye.mac_addr, le16_to_cpu(entry->keye.fid));
			goto del_fdb_entries;
		}
	}

	mutex_unlock(&priv->fdbt_lock);

	return 0;

del_fdb_entries:
	hlist_for_each_entry(tmp, &priv->fdb_list, node) {
		if (tmp == entry)
			break;

		ntmp_fdbt_delete_entry(cbdrs, tmp->entry_id);
	}

	netc_destroy_fdb_list(priv);

	mutex_unlock(&priv->fdbt_lock);

	return err;
}

static void netc_remove_fdbt_entries(struct netc_switch *priv)
{
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	struct netc_fdb_entry *entry;
	struct hlist_node *tmp;

	mutex_lock(&priv->fdbt_lock);

	hlist_for_each_entry_safe(entry, tmp, &priv->fdb_list, node) {
		ntmp_fdbt_delete_entry(cbdrs, entry->entry_id);
		netc_del_fdb_entry(entry);
	}

	mutex_unlock(&priv->fdbt_lock);
}

static int netc_port_restore_taprio(struct netc_port *port)
{
	struct netc_switch *priv = port->switch_priv;
	u32 entry_id = port->index;

	if (!port->taprio)
		return 0;

	return netc_setup_taprio(&priv->ntmp, entry_id, port->taprio);
}

static int netc_port_restore_config(struct netc_port *port)
{
	int err;

	if (!port->dp)
		return 0;

	netc_port_fixed_config(port);

	err = netc_port_restore_config_from_db(port);
	if (err)
		return err;

	err = netc_port_restore_taprio(port);
	if (err)
		goto del_ptp_filter;

	return 0;

del_ptp_filter:
	netc_port_set_ptp_filter(port, HWTSTAMP_FILTER_NONE);

	return err;
}

static void netc_port_remove_config(struct netc_port *port)
{
	if (!port->dp)
		return;

	if (port->taprio)
		netc_port_reset_taprio(port);

	netc_port_set_ptp_filter(port, HWTSTAMP_FILTER_NONE);
}

static int netc_restore_ports_config(struct netc_switch *priv)
{
	int i, err;

	for (i = 0; i < priv->num_ports; i++) {
		err = netc_port_restore_config(NETC_PORT(priv, i));
		if (err)
			goto del_ports_config;
	}

	return 0;

del_ports_config:
	while (--i >= 0)
		netc_port_remove_config(NETC_PORT(priv, i));

	return err;
}

static void netc_remove_ports_config(struct netc_switch *priv)
{
	int i;

	for (i = 0; i < priv->num_ports; i++)
		netc_port_remove_config(NETC_PORT(priv, i));
}

static void netc_enable_all_cdbrs(struct netc_switch *priv)
{
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	struct netc_switch_regs *regs = &priv->regs;
	int i;

	netc_base_wr(regs, NETC_CCAR, NETC_DEFAULT_CMD_CACHE_ATTR);

	for (i = 0; i < cbdrs->cbdr_num; i++)
		netc_enable_cbdr(&cbdrs->ring[i]);
}

static int netc_restore_hw_config(struct netc_switch *priv)
{
	int err;

	netc_enable_all_cdbrs(priv);
	netc_switch_fixed_config(priv);
	netc_restore_bpt_entries(priv);

	/* Restore VLAN filter table entries */
	err = netc_restore_vlan_entries(priv);
	if (err)
		return err;

	/* Restore FDB table */
	err = netc_restore_fdbt_entries(priv);
	if (err)
		goto del_vlan_entries;

	err = netc_restore_ports_config(priv);
	if (err)
		goto del_fdb_entries;

	err = netc_restore_flower_list_config(&priv->ntmp);
	if (err)
		goto del_ports_config;

	return 0;

del_ports_config:
	netc_remove_ports_config(priv);
del_fdb_entries:
	netc_remove_fdbt_entries(priv);
del_vlan_entries:
	netc_remove_vlan_entries(priv);

	return err;
}

static void netc_disable_port_clk(struct netc_port *port)
{
	if (!port->enabled)
		return;

	clk_disable_unprepare(port->ref_clk);
}

static int netc_enable_port_clk(struct netc_port *port)
{
	if (!port->enabled)
		return 0;

	return clk_prepare_enable(port->ref_clk);
}

int netc_suspend(struct dsa_switch *ds)
{
	struct netc_switch *priv = ds->priv;
	struct pci_dev *pdev = priv->pdev;
	bool power_off;
	int i;

	/* NETC keeps power in suspend mode if WOL is enabled. If
	 * WOL is not enabled, we assume that NETC will be powered
	 * off in suspend mode, even though it may not actually be
	 * powered off. Because currently there is no helper function
	 * to query whether NETC will be powered off in suspend mode.
	 */
	power_off = !(netc_ierb_may_wakeonlan() > 0);
	cancel_delayed_work_sync(&priv->fdbt_clean);

	for (i = 0; i < priv->num_ports; i++) {
		struct netc_port *port = NETC_PORT(priv, i);

		if (power_off) {
			netc_port_save_config_to_db(port);
			netc_port_remove_config(port);
		}

		netc_disable_port_clk(port);
	}

	if (power_off) {
		netc_clear_flower_table_restored_flag(&priv->ntmp);
	} else {
		pci_save_state(pdev);
		pci_set_power_state(pdev, PCI_D3hot);
	}

	pci_disable_device(pdev);

	return 0;
}

int netc_resume(struct dsa_switch *ds)
{
	struct netc_switch *priv = ds->priv;
	struct pci_dev *pdev = priv->pdev;
	struct device *dev = &pdev->dev;
	bool power_off;
	int i, err;

	power_off = !(netc_ierb_may_wakeonlan() > 0);

	/* If WOL is not enabled, we assume that NETC is powered off
	 * in suspend mode, and then restore the switch configuration
	 * when it resumes. But in fact NETC may not be powered off,
	 * for example, the system suspend fails, or NETC remains
	 * powered on for other reasons. But we do not know that NETC
	 * is not powered off in suspend mode. In this case, the switch
	 * still retains its configuration, which will cause the
	 * configuration recovery to fail. Therefore, we need to reset
	 * the switch through FLR and then restore the configuration.
	 */
	if (power_off)
		pcie_flr(pdev);

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Failed to enable device\n");
		return err;
	}

	pci_set_master(pdev);

	if (power_off) {
		err = netc_restore_hw_config(priv);
		if (err) {
			dev_err(dev, "Failed to restore configurations\n");
			return err;
		}
	} else {
		pci_set_power_state(pdev, PCI_D0);
		pci_restore_state(pdev);
	}

	for (i = 0; i < priv->num_ports; i++) {
		err = netc_enable_port_clk(NETC_PORT(priv, i));
		if (err) {
			dev_err(dev, "Failed to enable port %d clock\n", i);
			return err;
		}
	}

	schedule_delayed_work(&priv->fdbt_clean, priv->fdbt_acteu_interval);

	return 0;
}
