// SPDX-License-Identifier: GPL-2.0+
/* Copyright 2024 NXP
 */
#include <linux/device.h>

#include "netc_switch.h"

#define DEFINE_NETC_DEBUGFS(name, write_op)			\
static int name##_open(struct inode *inode, struct file *file)		\
{									\
	return single_open(file, name##_show, inode->i_private);	\
}									\
									\
static const struct file_operations name##_fops = {			\
	.owner		= THIS_MODULE,					\
	.open		= name##_open,					\
	.read		= seq_read,					\
	.write		= netc_##write_op##_write,			\
	.llseek		= seq_lseek,					\
	.release	= single_release,				\
}

static int netc_bpt_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;
	struct bpt_query_data qdata;
	struct bpt_bpse_data *bpse;
	struct bpt_cfge_data *cfge;
	int i, err;

	bpse = &qdata.bpse;
	cfge = &qdata.cfge;

	for (i = 0; i < priv->caps.num_bp; i++) {
		memset(&qdata, 0, sizeof(qdata));

		err = ntmp_bpt_query_entry(&priv->ntmp.cbdrs, i, &qdata);
		if (err)
			return err;

		seq_printf(s, "Show Buffer Pool Table entry %d\n", i);
		seq_puts(s, "Buffer Pool State Element Data:\n");
		seq_printf(s, "Amount Used:0x%x\n",
			   le32_to_cpu(bpse->amount_used));
		seq_printf(s, "Amount Used High Watermark:0x%x\n",
			   le32_to_cpu(bpse->amount_used_hwm));
		seq_printf(s, "Flow Control state: %s\n",
			   is_en(bpse->bpd_fc_state & BPT_FC_STATE));
		seq_printf(s, "Buffer Pool Disabled: %s\n",
			   is_yes(bpse->bpd_fc_state & BPT_BPD));

		seq_puts(s, "Buffer Pool Configuration Element Data:\n");
		seq_printf(s, "Shared Buffer Pool Enable: %s\n",
			   is_yes(cfge->fccfg_sbpen & BPT_SBP_EN));
		seq_printf(s, "Flow Control Configuration: %lu\n",
			   FIELD_GET(BPT_FC_CFG, cfge->fccfg_sbpen));
		seq_printf(s, "Priority Flow Control Vector: %u\n",
			   cfge->pfc_vector);
		seq_printf(s, "Maximum Threshold: 0x%x\n",
			   le16_to_cpu(cfge->max_thresh));
		seq_printf(s, "Flow Control On Threshold: 0x%x\n",
			   le16_to_cpu(cfge->fc_on_thresh));
		seq_printf(s, "Flow Control Off Threshold: 0x%x\n",
			   le16_to_cpu(cfge->fc_off_thresh));
		seq_printf(s, "Shared Buffer Pool Threshold: 0x%x\n",
			   le16_to_cpu(cfge->sbp_thresh));
		seq_printf(s, "Shared Buffer Pool Entry ID: 0x%x\n",
			   le32_to_cpu(cfge->sbp_eid));
		seq_printf(s, "Flow Control Ports: 0x%x\n",
			   le32_to_cpu(cfge->fc_ports));
		seq_puts(s, "\n");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(netc_bpt);

static int netc_sbpt_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;
	struct sbpt_sbpse_data *sbpse;
	struct sbpt_query_data qdata;
	struct sbpt_cfge_data *cfge;
	int i, err;

	sbpse = &qdata.sbpse;
	cfge = &qdata.cfge;

	for (i = 0; i < priv->caps.num_sbp; i++) {
		memset(&qdata, 0, sizeof(qdata));

		err = ntmp_sbpt_query_entry(&priv->ntmp.cbdrs, i, &qdata);
		if (err)
			return err;

		seq_printf(s, "Show Shared Buffer Pool Table entry %d\n", i);
		seq_puts(s, "Shared Buffer Pool State Element Data:\n");
		seq_printf(s, "Amount Used:0x%x\n",
			   le32_to_cpu(sbpse->amount_used));
		seq_printf(s, "Amount Used High Watermark:0x%x\n",
			   le32_to_cpu(sbpse->amount_used_hwm));
		seq_printf(s, "Flow Control state: %s\n",
			   is_en(sbpse->fc_state & SBPT_FC_STATE));

		seq_puts(s, "Shared Buffer Pool Configuration Element Data:\n");
		seq_printf(s, "Maximum Threshold: 0x%x\n",
			   le16_to_cpu(cfge->max_thresh));
		seq_printf(s, "Flow Control On Threshold: 0x%x\n",
			   le16_to_cpu(cfge->fc_on_thresh));
		seq_printf(s, "Flow Control Off Threshold: 0x%x\n",
			   le16_to_cpu(cfge->fc_off_thresh));
		seq_puts(s, "\n");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(netc_sbpt);

static int netc_port_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;
	u32 port = priv->dbg_params.port;

	seq_printf(s, "Switch debug port ID is %u\n", port);
	seq_puts(s, "\n");

	return 0;
}

static ssize_t netc_dbg_port_write(struct file *filp,
				   const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;

	return netc_kstrtouint(buffer, count, ppos, &priv->dbg_params.port);
}

DEFINE_NETC_DEBUGFS(netc_port, dbg_port);

static int netc_fdbt_show(struct seq_file *s, void *data)
{
	struct fdbt_query_data *qdata __free(kfree);
	struct netc_switch *priv = s->private;
	u32 resume_eid = NTMP_NULL_ENTRY_ID;
	struct fdbt_keye_data *keye;
	struct fdbt_cfge_data *cfge;
	struct fdbt_acte_data *acte;
	u32 entry_id, port, cfg;
	bool fdbt_empty = true;
	int err;

	port = priv->dbg_params.port;
	if (port >= priv->num_ports) {
		seq_puts(s, "Wrong port index\n");
		return -EINVAL;
	}

	qdata = kzalloc(sizeof(*qdata), GFP_KERNEL);
	if (!qdata)
		return -ENOMEM;

	keye = &qdata->keye;
	cfge = &qdata->cfge;
	acte = &qdata->acte;

	seq_printf(s, "Show Port %u FDB table\n", port);

	guard(mutex)(&priv->fdbt_lock);
	do {
		memset(qdata, 0, sizeof(*qdata));
		err = ntmp_fdbt_search_port_entry(&priv->ntmp.cbdrs, port,
						  &resume_eid, &entry_id, qdata);
		if (err) {
			seq_puts(s, "FDB table search failed\n");
			break;
		}

		if (entry_id == NTMP_NULL_ENTRY_ID) {
			if (fdbt_empty)
				seq_puts(s, "No entries found in FDB table\n");
			break;
		}

		cfg = le32_to_cpu(cfge->cfg);
		seq_printf(s, "FDB entry ID: 0x%x\n", entry_id);
		seq_printf(s, "MAC address: %pM\n", keye->mac_addr);
		seq_printf(s, "Filtering ID: %u\n", le16_to_cpu(keye->fid));
		seq_printf(s, "Port Bitmap: 0x%x\n", le32_to_cpu(cfge->port_bitmap));
		seq_printf(s, "Override ET_EID: %lu\n", cfg & FDBT_OETEID);
		seq_printf(s, "Egress Port: %lu\n", FIELD_GET(FDBT_EPORT, cfg));
		seq_printf(s, "Ingress Mirroring Enable: %s\n",
			   is_yes(cfg & FDBT_IMIRE));
		seq_printf(s, "Cut-Through Disable: %lu\n", FIELD_GET(FDBT_CTD, cfg));
		seq_printf(s, "Dynamic Entry: %s\n", is_yes(cfg & FDBT_DYNAMIC));
		seq_printf(s, "Timestamp Capture Enable: %s\n",
			   is_yes(cfg & FDBT_TIMECAPE));
		seq_printf(s, "ET_EID: 0x%x\n", le32_to_cpu(cfge->et_eid));
		seq_printf(s, "Activity Counter: %lu\n", acte->act & FDBT_ACT_CNT);
		seq_printf(s, "Activity Flag: %u\n",
			   (acte->act & FDBT_ACT_FLAG) ? 1 : 0);
		seq_puts(s, "\n");

		if (fdbt_empty)
			fdbt_empty = false;
	} while (resume_eid != NTMP_NULL_ENTRY_ID);

	return err;
}
DEFINE_SHOW_ATTRIBUTE(netc_fdbt);

static void netc_show_vft_entry(struct seq_file *s, u32 entry_id, u16 vid,
				struct vft_cfge_data *cfge)
{
	u32 bitmap_stg = le32_to_cpu(cfge->bitmap_stg);
	u16 cfg = le16_to_cpu(cfge->cfg);

	seq_printf(s, "VLAN filter table entry ID: 0x%x\n", entry_id);
	seq_printf(s, "VLAN ID: %u\n", vid);

	seq_printf(s, "Port Membership Bitmap: 0x%x\n", bitmap_stg & 0xffffff);
	seq_printf(s, "Spanning Tree Group Member ID: %u\n", bitmap_stg >> 24);
	seq_printf(s, "Filtering ID: %u\n", le16_to_cpu(cfge->fid));
	seq_printf(s, "MAC Learning Options: %lu\n", cfg & VFT_MLO);
	seq_printf(s, "MAC Forwarding Options: %lu\n", FIELD_GET(VFT_MFO, cfg));
	seq_printf(s, "IP Multicast Filtering Enable: %s\n",
		   is_yes(cfg & VFT_IPMFE));
	seq_printf(s, "IP Multicast Flooding Enable: %s\n",
		   is_yes(cfg & VFT_IPMFLE));
	seq_printf(s, "Port Group Action: %s\n", is_en(cfg & VFT_PGA));
	seq_printf(s, "Signature Duplicate Filtering Action: %s\n",
		   is_en(cfg & VFT_SFDA));
	seq_printf(s, "Override Signature Duplicate Filtering Action: %s\n",
		   is_en(cfg & VFT_OSFDA));
	seq_printf(s, "FDB Activity Flag Set Source: %s\n",
		   (cfg & VFT_FDBAFSS) ? "MAC Learning" : "MAC Forwarding");
	seq_printf(s, "Egress Treatment Applicability Port Bitmap: 0x%x\n",
		   le32_to_cpu(cfge->eta_port_bitmap));
	seq_printf(s, "Egress Treatment Entry ID: 0x%x\n",
		   le32_to_cpu(cfge->et_eid));
	seq_puts(s, "\n");
}

static int netc_vft_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;
	u32 resume_eid = NTMP_NULL_ENTRY_ID;
	struct vft_cfge_data cfge;
	bool vft_empty = true;
	u32 entry_id = 0;
	u16 vid = 0;
	int err;

	seq_puts(s, "Show VLAN filter table\n");

	guard(mutex)(&priv->vft_lock);
	do {
		memset(&cfge, 0, sizeof(cfge));
		err = ntmp_vft_search_entry(&priv->ntmp.cbdrs, &resume_eid,
					    &entry_id, &vid, &cfge);
		if (err) {
			seq_puts(s, "VLAN filter table search failed\n");
			break;
		}

		if (entry_id == NTMP_NULL_ENTRY_ID) {
			if (vft_empty)
				seq_puts(s, "No entries found in VLAN filter table\n");
			break;
		}

		if (vft_empty)
			vft_empty = false;

		netc_show_vft_entry(s, entry_id, vid, &cfge);
	} while (resume_eid != NTMP_NULL_ENTRY_ID);

	return err;
}
DEFINE_SHOW_ATTRIBUTE(netc_vft);

static ssize_t netc_vft_vid_write(struct file *filp, const char __user *buffer,
				  size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;
	char cmd_buffer[256];
	int len, err;

	if (*ppos != 0 || !count)
		return -EINVAL;

	if (count >= sizeof(cmd_buffer))
		return -ENOSPC;

	len = simple_write_to_buffer(cmd_buffer, sizeof(cmd_buffer) - 1,
				     ppos, buffer, count);
	if (len < 0)
		return len;

	cmd_buffer[len] = '\0';
	err = kstrtou16(cmd_buffer, 10, &priv->dbg_params.vft_vid);
	if (err)
		return err;

	return len;
}

static int netc_vft_entry_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;
	u16 vid = priv->dbg_params.vft_vid;
	struct vft_cfge_data cfge = {0};
	u32 entry_id = 0;
	int err;

	if (vid >= VLAN_N_VID) {
		seq_printf(s, "Wrong VLAN ID: %u\n", vid);
		return -EINVAL;
	}

	scoped_guard(mutex, &priv->vft_lock) {
		err = ntmp_vft_query_entry_by_vid(&priv->ntmp.cbdrs, vid,
						  &entry_id, &cfge);
		if (err) {
			seq_puts(s, "Query VLAN filter table failed\n");
			return err;
		}
	}

	if (entry_id == NTMP_NULL_ENTRY_ID) {
		seq_printf(s, "VLAN ID: %u entry is not found in VFT\n", vid);
		return 0;
	}

	seq_printf(s, "Show VFT VLAN ID: %u entry\n", vid);
	netc_show_vft_entry(s, entry_id, vid, &cfge);

	return 0;
}

DEFINE_NETC_DEBUGFS(netc_vft_entry, vft_vid);

static ssize_t netc_ett_eid_write(struct file *filp, const char __user *buffer,
				  size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;

	return netc_kstrtouint(buffer, count, ppos, &priv->dbg_params.ett_eid);
}

static int netc_ett_entry_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;
	struct ett_cfge_data cfge = {0};
	u32 entry_id;
	u16 efm_cfg;
	int err;

	entry_id = priv->dbg_params.ett_eid;
	if (entry_id >= priv->ntmp.caps.ett_num_entries) {
		seq_printf(s, "Wrong ETT entry ID: 0x%x\n", entry_id);
		return -EINVAL;
	}

	err = ntmp_ett_query_entry(&priv->ntmp.cbdrs, entry_id, &cfge);
	if (err) {
		seq_puts(s, "Query Egress Treatment table failed\n");
		return err;
	}

	efm_cfg = le16_to_cpu(cfge.efm_cfg);
	seq_printf(s, "Show ETT entry ID: 0x%x\n", entry_id);
	seq_printf(s, "Egress Frame Modification Mode: %lu\n",
		   FIELD_GET(ETT_EFM_MODE, efm_cfg));
	seq_printf(s, "Egress Sequence Actions: %lu\n",
		   FIELD_GET(ETT_ESQA, efm_cfg));
	seq_printf(s, "Egress Counter Action: %lu\n",
		   FIELD_GET(ETT_ECA, efm_cfg));
	seq_printf(s, "Egress Frame Modification Length Change: %lu\n",
		   FIELD_GET(ETT_EFM_LEN_CHANGE, efm_cfg));
	seq_printf(s, "Egress Frame Modification Data Length: %u\n",
		   le16_to_cpu(cfge.efm_data_len));
	seq_printf(s, "Egress Frame Modification Entry ID: 0x%x\n",
		   le32_to_cpu(cfge.efm_eid));
	seq_printf(s, "Egress Count Table Entry ID: 0x%x\n",
		   le32_to_cpu(cfge.ec_eid));
	seq_printf(s, "Egress Sequence Actions Target Entry ID: 0x%x\n",
		   le32_to_cpu(cfge.esqa_tgt_eid));
	seq_puts(s, "\n");

	return 0;
}

DEFINE_NETC_DEBUGFS(netc_ett_entry, ett_eid);

static ssize_t netc_ect_eid_write(struct file *filp, const char __user *buffer,
				  size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;

	return netc_kstrtouint(buffer, count, ppos, &priv->dbg_params.ect_eid);
}

static int netc_ect_entry_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;
	struct ect_stse_data stse = {0};
	u32 entry_id;
	int err;

	entry_id = priv->dbg_params.ect_eid;
	if (entry_id >= priv->ntmp.caps.ect_num_entries) {
		seq_printf(s, "Wrong ECT entry ID: 0x%x\n", entry_id);
		return -EINVAL;
	}

	err = ntmp_ect_query_entry(&priv->ntmp.cbdrs, entry_id, &stse, false);
	if (err) {
		seq_puts(s, "Query Egress Count table failed\n");
		return err;
	}

	seq_printf(s, "Show ECT entry ID: 0x%x\n", entry_id);
	seq_printf(s, "Enqueued Frame Count: %llu\n", le64_to_cpu(stse.enq_frm_cnt));
	seq_printf(s, "Rejected Frame Count: %llu\n", le64_to_cpu(stse.rej_frm_cnt));
	seq_puts(s, "\n");

	return 0;
}

DEFINE_NETC_DEBUGFS(netc_ect_entry, ect_eid);

static int netc_flower_list_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;
	struct netc_flower_rule *rule;

	guard(mutex)(&priv->ntmp.flower_lock);
	hlist_for_each_entry(rule, &priv->ntmp.flower_list, node) {
		seq_printf(s, "Port: %u, cookie:0x%lx\n", rule->port_id, rule->cookie);
		seq_printf(s, "Flower type:%d\n", rule->flower_type);

		switch (rule->flower_type) {
		case FLOWER_TYPE_PSFP:
			netc_show_psfp_flower(s, rule);
			break;
		case FLOWER_TYPE_TRAP:
		case FLOWER_TYPE_REDIRECT:
		case FLOWER_TYPE_POLICE:
			netc_show_ipft_flower(s, rule);
			break;
		default:
			break;
		}

		seq_puts(s, "\n");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(netc_flower_list);

static ssize_t netc_isit_eid_write(struct file *filp, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;

	return netc_kstrtouint(buffer, count, ppos, &priv->dbg_params.isit_eid);
}

static int netc_isit_entry_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;

	return netc_show_isit_entry(&priv->ntmp, s, priv->dbg_params.isit_eid);
}

DEFINE_NETC_DEBUGFS(netc_isit_entry, isit_eid);

static ssize_t netc_ist_eid_write(struct file *filp, const char __user *buffer,
				  size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;

	return netc_kstrtouint(buffer, count, ppos, &priv->dbg_params.ist_eid);
}

static int netc_ist_entry_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;

	return netc_show_ist_entry(&priv->ntmp, s, priv->dbg_params.ist_eid);
}

DEFINE_NETC_DEBUGFS(netc_ist_entry, ist_eid);

static ssize_t netc_isft_eid_write(struct file *filp, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;

	return netc_kstrtouint(buffer, count, ppos, &priv->dbg_params.isft_eid);
}

static int netc_isft_entry_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;

	return netc_show_isft_entry(&priv->ntmp, s, priv->dbg_params.isft_eid);
}

DEFINE_NETC_DEBUGFS(netc_isft_entry, isft_eid);

static ssize_t netc_sgit_eid_write(struct file *filp, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;

	return netc_kstrtouint(buffer, count, ppos, &priv->dbg_params.sgit_eid);
}

static int netc_sgit_entry_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;

	return netc_show_sgit_entry(&priv->ntmp, s, priv->dbg_params.sgit_eid);
}

DEFINE_NETC_DEBUGFS(netc_sgit_entry, sgit_eid);

static ssize_t netc_sgclt_eid_write(struct file *filp, const char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;

	return netc_kstrtouint(buffer, count, ppos, &priv->dbg_params.sgclt_eid);
}

static int netc_sgclt_entry_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;

	return netc_show_sgclt_entry(&priv->ntmp, s, priv->dbg_params.sgclt_eid);
}

DEFINE_NETC_DEBUGFS(netc_sgclt_entry, sgclt_eid);

static ssize_t netc_isct_eid_write(struct file *filp, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;

	return netc_kstrtouint(buffer, count, ppos, &priv->dbg_params.isct_eid);
}

static int netc_isct_entry_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;

	return netc_show_isct_entry(&priv->ntmp, s, priv->dbg_params.isct_eid);
}

DEFINE_NETC_DEBUGFS(netc_isct_entry, isct_eid);

static ssize_t netc_rpt_eid_write(struct file *filp, const char __user *buffer,
				  size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;

	return netc_kstrtouint(buffer, count, ppos, &priv->dbg_params.rpt_eid);
}

static int netc_rpt_entry_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;

	return netc_show_rpt_entry(&priv->ntmp, s, priv->dbg_params.rpt_eid);
}

DEFINE_NETC_DEBUGFS(netc_rpt_entry, rpt_eid);

static ssize_t netc_ipft_eid_write(struct file *filp, const char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct seq_file *s = filp->private_data;
	struct netc_switch *priv = s->private;

	return netc_kstrtouint(buffer, count, ppos, &priv->dbg_params.ipft_eid);
}

static int netc_ipft_entry_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;

	return netc_show_ipft_entry(&priv->ntmp, s, priv->dbg_params.ipft_eid);
}

DEFINE_NETC_DEBUGFS(netc_ipft_entry, ipft_eid);

static int netc_tgst_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;
	u32 port_id = priv->dbg_params.port;
	struct netc_port *port;
	u32 val;

	if (port_id >= priv->num_ports) {
		seq_puts(s, "Wrong port index\n");
		return -EINVAL;
	}

	port = NETC_PORT(priv, port_id);
	val = netc_port_rd(port, NETC_PTGSCR);
	if (!(val & PTGSCR_TGE)) {
		seq_puts(s, "Time Gating Disable\n");
		return 0;
	}

	return netc_show_tgst_entry(&priv->ntmp, s, port_id);
}
DEFINE_SHOW_ATTRIBUTE(netc_tgst);

static int netc_counter_show(struct seq_file *s, void *data)
{
	struct netc_switch *priv = s->private;
	u32 port_id = priv->dbg_params.port;
	struct netc_port *port;
	u64 val64;
	u32 val;

	if (port_id >= priv->num_ports) {
		seq_puts(s, "Wrong port index\n");
		return -EINVAL;
	}

	port = NETC_PORT(priv, port_id);

	seq_printf(s, "Show Switch Port %u Counters\n", port_id);
	val = netc_port_rd(port, NETC_PRXDCR);
	seq_printf(s, "PRXDCR: 0x%x\n", val);
	val = netc_port_rd(port, NETC_PRXDCRRR);
	seq_printf(s, "PRXDCRRR: 0x%x\n", val);
	val = netc_port_rd(port, NETC_PRXDCRR0);
	seq_printf(s, "PRXDCRR0: 0x%x\n", val);
	netc_port_wr(port, NETC_PRXDCRR0, val);
	val = netc_port_rd(port, NETC_PRXDCRR1);
	seq_printf(s, "PRXDCRR1: 0x%x\n\n", val);
	netc_port_wr(port, NETC_PRXDCRR1, val);

	val = netc_port_rd(port, NETC_PTXDCR);
	seq_printf(s, "PTXDCR: 0x%x\n", val);
	val = netc_port_rd(port, NETC_PTXDCRRR);
	seq_printf(s, "PTXDCRRR: 0x%x\n", val);
	val = netc_port_rd(port, NETC_PTXDCRR0);
	seq_printf(s, "PTXDCRR0: 0x%x\n", val);
	netc_port_wr(port, NETC_PTXDCRR0, val);
	val = netc_port_rd(port, NETC_PTXDCRR1);
	seq_printf(s, "PTXDCRR1: 0x%x\n\n", val);
	netc_port_wr(port, NETC_PTXDCRR1, val);

	val = netc_port_rd(port, NETC_BPDCR);
	seq_printf(s, "BPDCR: 0x%x\n", val);
	val = netc_port_rd(port, NETC_BPDCRRR);
	seq_printf(s, "BPDCRRR: 0x%x\n", val);
	val = netc_port_rd(port, NETC_BPDCRR0);
	seq_printf(s, "BPDCRR0: 0x%x\n", val);
	netc_port_wr(port, NETC_BPDCRR0, val);
	val = netc_port_rd(port, NETC_BPDCRR1);
	seq_printf(s, "BPDCRR1: 0x%x\n\n", val);
	netc_port_wr(port, NETC_BPDCRR1, val);

	if (is_netc_pseudo_port(port)) {
		val64 = netc_port_rd64(port, NETC_PPMROCR);
		seq_printf(s, "PPMROCR: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PPMRUFCR);
		seq_printf(s, "PPMRUFCR: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PPMRMFCR);
		seq_printf(s, "PPMRMFCR: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PPMRBFCR);
		seq_printf(s, "PPMRBFCR: 0x%llx\n\n", val64);

		val64 = netc_port_rd64(port, NETC_PPMTOCR);
		seq_printf(s, "PPMTOCR: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PPMTUFCR);
		seq_printf(s, "PPMTUFCR: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PPMTMFCR);
		seq_printf(s, "PPMTMFCR: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PPMTBFCR);
		seq_printf(s, "PPMTBFCR: 0x%llx\n", val64);
	} else {
		val64 = netc_port_rd64(port, NETC_PM_RFRM(0));
		seq_printf(s, "PM0_RFRM: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PM_RERR(0));
		seq_printf(s, "PM0_RERR: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PM_RUCA(0));
		seq_printf(s, "PM0_RUCA: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PM_RMCA(0));
		seq_printf(s, "PM0_RMCA: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PM_RBCA(0));
		seq_printf(s, "PM0_RBCA: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PM_RDRP(0));
		seq_printf(s, "PM0_RDRP: 0x%llx\n\n", val64);

		val64 = netc_port_rd64(port, NETC_PM_TFRM(0));
		seq_printf(s, "PM0_TFRM: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PM_TERR(0));
		seq_printf(s, "PM0_TERR: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PM_TUCA(0));
		seq_printf(s, "PM0_TUCA: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PM_TMCA(0));
		seq_printf(s, "PM0_TMCA: 0x%llx\n", val64);
		val64 = netc_port_rd64(port, NETC_PM_TBCA(0));
		seq_printf(s, "PM0_TBCA: 0x%llx\n", val64);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(netc_counter);

void netc_create_debugfs(struct netc_switch *priv)
{
	struct dentry *root;

	root = debugfs_create_dir("netc_switch", NULL);
	if (IS_ERR(root))
		return;

	priv->debugfs_root = root;
	debugfs_create_file("bpt_dump", 0444, root, priv, &netc_bpt_fops);
	debugfs_create_file("sbpt_dump", 0444, root, priv, &netc_sbpt_fops);
	debugfs_create_file("dbg_port", 0600, root, priv, &netc_port_fops);
	debugfs_create_file("fdbt_dump", 0444, root, priv, &netc_fdbt_fops);
	debugfs_create_file("vft_dump", 0444, root, priv, &netc_vft_fops);
	debugfs_create_file("vft_entry", 0600, root, priv, &netc_vft_entry_fops);
	debugfs_create_file("ett_entry", 0600, root, priv, &netc_ett_entry_fops);
	debugfs_create_file("ect_entry", 0600, root, priv, &netc_ect_entry_fops);
	debugfs_create_file("flower_list", 0444, root, priv, &netc_flower_list_fops);
	debugfs_create_file("isit_entry", 0600, root, priv, &netc_isit_entry_fops);
	debugfs_create_file("ist_entry", 0600, root, priv, &netc_ist_entry_fops);
	debugfs_create_file("isft_entry", 0600, root, priv, &netc_isft_entry_fops);
	debugfs_create_file("sgit_entry", 0600, root, priv, &netc_sgit_entry_fops);
	debugfs_create_file("sgclt_entry", 0600, root, priv, &netc_sgclt_entry_fops);
	debugfs_create_file("isct_entry", 0600, root, priv, &netc_isct_entry_fops);
	debugfs_create_file("rpt_entry", 0600, root, priv, &netc_rpt_entry_fops);
	debugfs_create_file("ipft_entry", 0600, root, priv, &netc_ipft_entry_fops);
	debugfs_create_file("tgst_dump", 0444, root, priv, &netc_tgst_fops);
	debugfs_create_file("port_counter", 0444, root, priv, &netc_counter_fops);
}

void netc_remove_debugfs(struct netc_switch *priv)
{
	debugfs_remove_recursive(priv->debugfs_root);
	priv->debugfs_root = NULL;
}
