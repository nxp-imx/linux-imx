// SPDX-License-Identifier: GPL-2.0+
/* Copyright 2025 NXP */

#include <linux/device.h>
#include <linux/fsl/netc_lib.h>
#include <linux/string_choices.h>

#include "enetc_pf.h"
#include "enetc4_debugfs.h"
#include "enetc4_tc.h"

#define DEFINE_ENETC_DEBUGFS(name, write_op)			\
static int name##_open(struct inode *inode, struct file *file)		\
{									\
	return single_open(file, name##_show, inode->i_private);	\
}									\
									\
static const struct file_operations name##_fops = {			\
	.owner		= THIS_MODULE,					\
	.open		= name##_open,					\
	.read		= seq_read,					\
	.write		= enetc_##write_op##_write,			\
	.llseek		= seq_lseek,					\
	.release	= single_release,				\
}

static void enetc_show_si_mac_hash_filter(struct seq_file *s, int i)
{
	struct enetc_si *si = s->private;
	struct enetc_hw *hw = &si->hw;
	u32 hash_h, hash_l;

	hash_l = enetc_port_rd(hw, ENETC4_PSIUMHFR0(i));
	hash_h = enetc_port_rd(hw, ENETC4_PSIUMHFR1(i));
	seq_printf(s, "SI %d unicast MAC hash filter: 0x%08x%08x\n",
		   i, hash_h, hash_l);

	hash_l = enetc_port_rd(hw, ENETC4_PSIMMHFR0(i));
	hash_h = enetc_port_rd(hw, ENETC4_PSIMMHFR1(i));
	seq_printf(s, "SI %d multicast MAC hash filter: 0x%08x%08x\n",
		   i, hash_h, hash_l);
}

static int enetc_mac_filter_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	struct enetc_hw *hw = &si->hw;
	struct maft_entry_data maft;
	struct enetc_pf *pf;
	int i, err, num_si;
	u32 val;

	pf = enetc_si_priv(si);
	num_si = pf->caps.num_vsi + 1;

	val = enetc_port_rd(hw, ENETC4_PSIPMMR);
	for (i = 0; i < num_si; i++) {
		seq_printf(s, "SI %d Unicast Promiscuous mode: %s\n", i,
			   str_enabled_disabled(PSIPMMR_SI_MAC_UP(i) & val));
		seq_printf(s, "SI %d Multicast Promiscuous mode: %s\n", i,
			   str_enabled_disabled(PSIPMMR_SI_MAC_MP(i) & val));
	}

	/* MAC hash filter table */
	for (i = 0; i < num_si; i++)
		enetc_show_si_mac_hash_filter(s, i);

	if (!pf->num_mfe)
		return 0;

	/* MAC address filter table */
	seq_puts(s, "MAC address filter table\n");
	for (i = 0; i < pf->num_mfe; i++) {
		memset(&maft, 0, sizeof(maft));
		err = ntmp_maft_query_entry(&si->ntmp_user, i, &maft);
		if (err)
			return err;

		seq_printf(s, "Entry %d, MAC: %pM, SI bitmap: 0x%04x\n", i,
			   maft.keye.mac_addr, le16_to_cpu(maft.cfge.si_bitmap));
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_mac_filter);

static int enetc_tgst_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	int port;
	u32 val;

	val = enetc_port_rd(&si->hw, ENETC4_PTGSCR);
	if (!(val & PTGSCR_TGE)) {
		seq_puts(s, "Time Gating Disable\n");

		return 0;
	}

	port = enetc4_pf_to_port(si);
	if (port < 0)
		return -EINVAL;

	return netc_show_tgst_entry(&si->ntmp_user, s, port);
}
DEFINE_SHOW_ATTRIBUTE(enetc_tgst_entry);

static int enetc_flower_list_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	struct netc_flower_rule *rule;
	struct ntmp_user *user;

	user = &si->ntmp_user;
	mutex_lock(&user->flower_lock);

	hlist_for_each_entry(rule, &user->flower_list, node) {
		seq_printf(s, "Cookie:0x%lx\n", rule->cookie);
		seq_printf(s, "Flower type:%d\n", rule->flower_type);

		switch (rule->flower_type) {
		case FLOWER_TYPE_PSFP:
			netc_show_psfp_flower(s, rule);
			break;
		case FLOWER_TYPE_POLICE:
		case FLOWER_TYPE_DROP:
			netc_show_ipft_flower(s, rule);
			break;
		default:
			break;
		}

		seq_puts(s, "\n");
	}

	mutex_unlock(&user->flower_lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_flower_list);

static ssize_t enetc_isit_eid_write(struct file *filp, const char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.isit_eid);
}

static int enetc_isit_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_isit_entry(&si->ntmp_user, s, si->dbg_params.isit_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_isit_entry, isit_eid);

static ssize_t enetc_ist_eid_write(struct file *filp, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.ist_eid);
}

static int enetc_ist_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_ist_entry(&si->ntmp_user, s, si->dbg_params.ist_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_ist_entry, ist_eid);

static ssize_t enetc_isft_eid_write(struct file *filp, const char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.isft_eid);
}

static int enetc_isft_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_isft_entry(&si->ntmp_user, s, si->dbg_params.isft_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_isft_entry, isft_eid);

static ssize_t enetc_sgit_eid_write(struct file *filp, const char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.sgit_eid);
}

static int enetc_sgit_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_sgit_entry(&si->ntmp_user, s, si->dbg_params.sgit_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_sgit_entry, sgit_eid);

static ssize_t enetc_sgclt_eid_write(struct file *filp, const char __user *buffer,
				     size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.sgclt_eid);
}

static int enetc_sgclt_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_sgclt_entry(&si->ntmp_user, s, si->dbg_params.sgclt_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_sgclt_entry, sgclt_eid);

static ssize_t enetc_isct_eid_write(struct file *filp, const char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.isct_eid);
}

static int enetc_isct_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_isct_entry(&si->ntmp_user, s, si->dbg_params.isct_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_isct_entry, isct_eid);

static ssize_t enetc_rpt_eid_write(struct file *filp, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos, &si->dbg_params.rpt_eid);
}

static int enetc_rpt_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_rpt_entry(&si->ntmp_user, s, si->dbg_params.rpt_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_rpt_entry, rpt_eid);

static int enetc_clsrule_eid_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	struct enetc_ndev_priv *priv;
	int i;

	priv = netdev_priv(si->ndev);
	for (i = 0; i < si->max_ipf_entries + si->num_fs_entries; i++) {
		if (!priv->cls_rules[i].used)
			continue;

		if (priv->cls_rules[i].is_rfs)
			continue;

		seq_printf(s, "IPFT entry ID: 0x%x\n",
			   priv->cls_rules[i].entry_id);
	}

	seq_puts(s, "\n");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_clsrule_eid);

static ssize_t enetc_ipft_eid_write(struct file *filp,
				    const char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct enetc_si *si = s->private;

	return netc_kstrtouint(buffer, count, ppos,
			       &si->dbg_params.ipft_eid);
}

static int enetc_ipft_entry_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;

	return netc_show_ipft_entry(&si->ntmp_user, s,
				    si->dbg_params.ipft_eid);
}

DEFINE_ENETC_DEBUGFS(enetc_ipft_entry, ipft_eid);

static int enetc_show_rfst_entry(struct ntmp_user *user, struct seq_file *s,
				 u32 entry_id)
{
	struct rfst_entry_data *rfst_entry __free(kfree);
	struct rfst_keye_data *keye;
	struct rfst_cfge_data *cfge;
	int err;
	u32 cfg;

	rfst_entry = kzalloc(sizeof(*rfst_entry), GFP_KERNEL);
	if (!rfst_entry)
		return -ENOMEM;

	err = ntmp_rfst_query_entry(user, entry_id, rfst_entry);
	if (err)
		return err;

	keye = &rfst_entry->keye;
	cfge = &rfst_entry->cfge;
	cfg = le32_to_cpu(cfge->cfg);

	seq_printf(s, "Show RFS table entry:%u\n", entry_id);

	/* KEYE_DATA */
	seq_printf(s, "IP Source Address:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->source_ip_addr[0]),
		   ntohl(keye->source_ip_addr[1]),
		   ntohl(keye->source_ip_addr[2]),
		   ntohl(keye->source_ip_addr[3]));
	seq_printf(s, "IP Source Address mask:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->source_ip_addr_mask[0]),
		   ntohl(keye->source_ip_addr_mask[1]),
		   ntohl(keye->source_ip_addr_mask[2]),
		   ntohl(keye->source_ip_addr_mask[3]));
	seq_printf(s, "IP Destination Address:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->dest_ip_addr[0]), ntohl(keye->dest_ip_addr[1]),
		   ntohl(keye->dest_ip_addr[2]), ntohl(keye->dest_ip_addr[3]));
	seq_printf(s, "IP Destination Address mask:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->dest_ip_addr_mask[0]),
		   ntohl(keye->dest_ip_addr_mask[1]),
		   ntohl(keye->dest_ip_addr_mask[2]),
		   ntohl(keye->dest_ip_addr_mask[3]));
	seq_printf(s, "L4 Source Port:%u, mask:0x%04x\n",
		   ntohs(keye->l4_source_port),
		   ntohs(keye->l4_source_port_mask));
	seq_printf(s, "L4 Destination Port:%u, mask:0x%04x\n",
		   ntohs(keye->l4_dest_port), ntohs(keye->l4_dest_port_mask));
	seq_printf(s, "L3/L4 Protocols:0x%04x\n", keye->l3_l4_protocol);

	/* STSE_DATA */
	seq_printf(s, "Match Count: %llu\n",
		   le64_to_cpu(rfst_entry->matched_frames));

	/* CFGE_DATA */
	seq_printf(s, "Result: %lu\n", FIELD_GET(RFST_RESULT, cfg));
	seq_printf(s, "Mode: %lu\n", FIELD_GET(RFST_MODE, cfg));

	seq_puts(s, "\n");

	return err;
}

static int enetc_rfst_show(struct seq_file *s, void *data)
{
	struct enetc_si *si = s->private;
	struct enetc_ndev_priv *priv;
	int i, err;
	u32 val;

	val = enetc_port_rd(&si->hw, ENETC4_PRFSMR);
	seq_printf(s, "RFS is %s\n", is_en(val & PRFSMR_RFSE));

	priv = netdev_priv(si->ndev);
	for (i = 0; i < si->max_ipf_entries + si->num_fs_entries; i++) {
		if (priv->cls_rules[i].is_rfs && priv->cls_rules[i].used) {
			err = enetc_show_rfst_entry(&si->ntmp_user, s,
						    priv->cls_rules[i].entry_id);
			if (err)
				return err;
		}
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(enetc_rfst);

void enetc_create_debugfs(struct enetc_si *si)
{
	struct net_device *ndev = si->ndev;
	struct dentry *root;

	root = debugfs_create_dir(netdev_name(ndev), NULL);
	if (IS_ERR(root))
		return;

	si->debugfs_root = root;

	debugfs_create_file("mac_filter", 0444, root, si, &enetc_mac_filter_fops);
	debugfs_create_file("tgst_entry", 0444, root, si, &enetc_tgst_entry_fops);
	debugfs_create_file("isit_entry", 0600, root, si, &enetc_isit_entry_fops);
	debugfs_create_file("ist_entry", 0600, root, si, &enetc_ist_entry_fops);
	debugfs_create_file("isft_entry", 0600, root, si, &enetc_isft_entry_fops);
	debugfs_create_file("sgit_entry", 0600, root, si, &enetc_sgit_entry_fops);
	debugfs_create_file("sgclt_entry", 0600, root, si, &enetc_sgclt_entry_fops);
	debugfs_create_file("isct_entry", 0600, root, si, &enetc_isct_entry_fops);
	debugfs_create_file("rpt_entry", 0600, root, si, &enetc_rpt_entry_fops);
	debugfs_create_file("rfst_entry", 0444, root, si, &enetc_rfst_fops);
	debugfs_create_file("flower_list", 0444, root, si, &enetc_flower_list_fops);
	debugfs_create_file("clsrule_eid", 0444, root, si, &enetc_clsrule_eid_fops);
	debugfs_create_file("ipft_entry", 0600, root, si, &enetc_ipft_entry_fops);
}

void enetc_remove_debugfs(struct enetc_si *si)
{
	debugfs_remove(si->debugfs_root);
	si->debugfs_root = NULL;
}
