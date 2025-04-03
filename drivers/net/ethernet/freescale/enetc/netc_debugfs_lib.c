// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NETC NTMP (NETC Table Management Protocol) 2.0 driver
 * Copyright 2024 NXP
 */
#include <linux/fsl/netc_lib.h>

#include "ntmp_private.h"

int netc_kstrtouint(const char __user *buffer, size_t count,
		    loff_t *ppos, u32 *val)
{
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
	err = kstrtouint(cmd_buffer, 16, val);
	if (err)
		return err;

	return len;
}
EXPORT_SYMBOL_GPL(netc_kstrtouint);

void netc_show_psfp_flower(struct seq_file *s, struct netc_flower_rule *rule)
{
	struct ntmp_isit_entry *isit_entry = rule->key_tbl->isit_entry;
	struct ntmp_ist_entry *ist_entry = rule->key_tbl->ist_entry;
	struct ntmp_isft_entry *isft_entry = rule->isft_entry;
	u32 rpt_eid, sgit_eid, isct_eid;

	seq_printf(s, "ISIT entry ID:0x%x\n", isit_entry->entry_id);
	seq_printf(s, "IST entry ID: 0x%x\n", ist_entry->entry_id);

	if (isft_entry) {
		rpt_eid = le32_to_cpu(isft_entry->cfge.rp_eid);
		isct_eid = le32_to_cpu(isft_entry->cfge.isc_eid);
		sgit_eid = le32_to_cpu(isft_entry->cfge.sgi_eid);
		seq_printf(s, "ISFT entry ID: 0x%x\n", isft_entry->entry_id);
	} else {
		rpt_eid = le32_to_cpu(ist_entry->cfge.rp_eid);
		isct_eid = le32_to_cpu(ist_entry->cfge.isc_eid);
		sgit_eid = le32_to_cpu(ist_entry->cfge.sgi_eid);
	}

	seq_printf(s, "RPT entry ID: 0x%x\n", rpt_eid);
	seq_printf(s, "SGIT entry ID: 0x%x\n", sgit_eid);
	seq_printf(s, "ISCT entry ID: 0x%x\n", isct_eid);
	seq_printf(s, "SGCLT entry ID: 0x%x\n", rule->sgclt_eid);
}
EXPORT_SYMBOL_GPL(netc_show_psfp_flower);

int netc_show_isit_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct ntmp_isit_entry *isit_entry __free(kfree);
	struct isit_keye_data *keye;
	u32 key_aux;
	int i, err;

	isit_entry = kzalloc(sizeof(*isit_entry), GFP_KERNEL);
	if (!isit_entry)
		return -ENOMEM;

	err = ntmp_isit_query_entry(&priv->cbdrs, entry_id, isit_entry);
	if (err) {
		seq_printf(s, "Query ISIT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	keye = &isit_entry->keye;
	key_aux = le32_to_cpu(keye->key_aux);
	seq_printf(s, "Show ingress stream identification table entry 0x%x\n",
		   entry_id);
	seq_printf(s, "Key type: %lu, Source Port ID: %lu, IS_EID: %u\n",
		   FIELD_GET(ISIT_KEY_TYPE, key_aux),
		   FIELD_GET(ISIT_SRC_PORT_ID, key_aux),
		   le32_to_cpu(isit_entry->is_eid));
	seq_puts(s, "Keys: ");
	for (i = 0; i < ISIT_FRAME_KEY_LEN; i++)
		seq_printf(s, "%02x", keye->frame_key[i]);
	seq_puts(s, "\n\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_isit_entry);

int netc_show_ist_entry(struct ntmp_priv *priv, struct seq_file *s,
			u32 entry_id)
{
	struct ist_cfge_data *cfge __free(kfree);
	u32 bitmap_evmeid, cfg;
	u16 switch_cfg;
	int err;

	cfge = kzalloc(sizeof(*cfge), GFP_KERNEL);
	if (!cfge)
		return -ENOMEM;

	err = ntmp_ist_query_entry(&priv->cbdrs, entry_id, cfge);
	if (err) {
		seq_printf(s, "Query IST entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	switch_cfg = le16_to_cpu(cfge->switch_cfg);
	bitmap_evmeid = le32_to_cpu(cfge->bitmap_evmeid);
	cfg = le32_to_cpu(cfge->cfg);
	seq_printf(s, "Show ingress stream table entry 0x%x\n", entry_id);
	seq_printf(s, "Stream Filtering: %s, Report Receive Timestamp: %s\n",
		   is_en(cfg & IST_SFE), is_en(cfg & IST_RRT));
	seq_printf(s, "OIPV: %s, IPV: %lu, ODR: %s, DR: %lu\n",
		   is_en(cfg & IST_OIPV), FIELD_GET(IST_IPV, cfg),
		   is_en(cfg & IST_ODR), FIELD_GET(IST_DR, cfg));
	seq_printf(s, "IMIRE: %s, TIMECAPE: %s, SPPD: %s, ISQGA: %lu\n",
		   is_en(cfg & IST_IMIRE), is_en(cfg & IST_TIMERCAPE),
		   is_en(cfg & IST_SPPD), FIELD_GET(IST_ISQGA, cfg));
	seq_printf(s, "ORP: %s, OSGI: %s, Host Reason:%lu\n",
		   is_en(cfg & IST_ORP), is_en(cfg & IST_OSGI),
		   FIELD_GET(IST_HR, cfg));

	switch (priv->cbdrs.tbl.ist_ver) {
	case NTMP_TBL_VER0:
		seq_printf(s, "Forwarding Action: %lu, SDU type:%lu\n",
			   FIELD_GET(IST_V0_FA, cfg),
			   FIELD_GET(IST_V0_SDU_TYPE, cfg));
		break;
	case NTMP_TBL_VER1:
		seq_printf(s, "Forwarding Action: %lu, SDU type:%lu\n",
			   FIELD_GET(IST_V1_FA, cfg),
			   FIELD_GET(IST_V1_SDU_TYPE, cfg));
		seq_printf(s, "SDFA: %lu, OSDFA: %s\n",
			   FIELD_GET(IST_SDFA, cfg), is_en(cfg & IST_OSDFA));
		break;
	default:
		break;
	}

	seq_printf(s, "MSDU :%u\n", le16_to_cpu(cfge->msdu));
	seq_printf(s, "IFME_LEN_CHANGE: 0x%lx, Egress Port: %lu\n",
		   FIELD_GET(IST_IFME_LEN_CHANGE, switch_cfg),
		   FIELD_GET(IST_EPORT, switch_cfg));
	seq_printf(s, "Override ET_EID: %lu, CTD: %lu\n",
		   FIELD_GET(IST_OETEID, switch_cfg),
		   FIELD_GET(IST_CTD, switch_cfg));
	seq_printf(s, "ISQG_EID: 0x%x, RP_EID: 0x%x\n", le32_to_cpu(cfge->isqg_eid),
		   le32_to_cpu(cfge->rp_eid));
	seq_printf(s, "SGI_EID: 0x%x, IFM_EID: 0x%x\n", le32_to_cpu(cfge->sgi_eid),
		   le32_to_cpu(cfge->ifm_eid));
	seq_printf(s, "ET_EID: 0x%x, ISC_EID: 0x%x\n", le32_to_cpu(cfge->et_eid),
		   le32_to_cpu(cfge->isc_eid));
	seq_printf(s, "Egress Port bitmap: 0x%lx, Event Monitor Event ID: %lu\n",
		   bitmap_evmeid & IST_EGRESS_PORT_BITMAP,
		   FIELD_GET(IST_EVMEID, bitmap_evmeid));
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_ist_entry);

int netc_show_isft_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct ntmp_isft_entry *isft_entry __free(kfree);
	struct isft_cfge_data *cfge;
	struct isft_keye_data *keye;
	u16 cfg;
	int err;

	isft_entry = kzalloc(sizeof(*isft_entry), GFP_KERNEL);
	if (!isft_entry)
		return -ENOMEM;

	keye = &isft_entry->keye;
	cfge = &isft_entry->cfge;
	err = ntmp_isft_query_entry(&priv->cbdrs, entry_id, isft_entry);
	if (err) {
		seq_printf(s, "Query ISFT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	cfg = le16_to_cpu(cfge->cfg);
	seq_printf(s, "Show ingress stream filter table entry 0x%x\n", entry_id);
	seq_printf(s, "IS_EID: 0x%x, PCP: %u\n",
		   le32_to_cpu(keye->is_eid), keye->pcp);
	seq_printf(s, "OIPV: %s, IPV: %lu, ODR: %s, DR: %lu\n",
		   is_en(cfg & ISFT_OIPV), FIELD_GET(ISFT_IPV, cfg),
		   is_en(cfg & ISFT_ODR), FIELD_GET(ISFT_DR, cfg));
	seq_printf(s, "IMIRE: %s, TIMECAPE:%s, OSGI: %s, CTD: %s\n",
		   is_en(cfg & ISFT_IMIRE), is_en(cfg & ISFT_TIMECAPE),
		   is_en(cfg & ISFT_OSGI), is_yes(cfg & ISFT_CTD));
	seq_printf(s, "ORP: %s, SDU type: %lu, MSDU: %u\n",
		   is_en(cfg & ISFT_ORP), FIELD_GET(ISFT_SDU_TYPE, cfg),
		   le16_to_cpu(cfge->msdu));
	seq_printf(s, "RP_EID: 0x%x, SGI_EID: 0x%x, ISC_EID: 0x%x\n",
		   le32_to_cpu(cfge->rp_eid), le32_to_cpu(cfge->sgi_eid),
		   le32_to_cpu(cfge->isc_eid));
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_isft_entry);

int netc_show_sgit_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct ntmp_sgit_entry *sgit_entry __free(kfree);
	struct sgit_acfge_data *acfge;
	struct sgit_icfge_data *icfge;
	struct sgit_sgise_data *sgise;
	struct sgit_cfge_data *cfge;
	int err;

	sgit_entry = kzalloc(sizeof(*sgit_entry), GFP_KERNEL);
	if (!sgit_entry)
		return -ENOMEM;

	err = ntmp_sgit_query_entry(&priv->cbdrs, entry_id, sgit_entry);
	if (err) {
		seq_printf(s, "Query SGIT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	acfge = &sgit_entry->acfge;
	icfge = &sgit_entry->icfge;
	sgise = &sgit_entry->sgise;
	cfge = &sgit_entry->cfge;
	seq_printf(s, "Show stream gate instance table entry 0x%x\n", entry_id);
	seq_printf(s, "OPER_SGCL_EID: 0x%x, CONFIG_CHANGE_TIME: %llu\n",
		   le32_to_cpu(sgise->oper_sgcl_eid),
		   le64_to_cpu(sgise->config_change_time));
	seq_printf(s, "OPER_BASE_TIME: %llu, OPER_CYCLE_TIME_EXT: %u\n",
		   le64_to_cpu(sgise->oper_base_time),
		   le32_to_cpu(sgise->oper_cycle_time_ext));
	seq_printf(s, "OEX: %lu, IRX: %lu, state: %lu\n", sgise->info & SGIT_OEX,
		   FIELD_GET(SGIT_IRX, sgise->info),
		   FIELD_GET(SGIT_STATE, sgise->info));
	seq_printf(s, "OEXEN: %s, IRXEN: %s, SDU type:%lu\n",
		   is_en(cfge->cfg & SGIT_OEXEN),
		   is_en(cfge->cfg & SGIT_IRXEN),
		   FIELD_GET(SGIT_SDU_TYPE, cfge->cfg));
	seq_printf(s, "OIPV: %s, IPV: %lu, GST: %s, CTD: %s\n",
		   is_en(icfge->icfg & SGIT_OIPV),
		   FIELD_GET(SGIT_IPV, icfge->icfg),
		   icfge->icfg & SGIT_GST ? "open" : "closed",
		   is_yes(icfge->icfg & SGIT_CTD));
	seq_printf(s, "ADMIN_SGCL_EID: 0x%x, ADMIN_BASE_TIME: %llu\n",
		   le32_to_cpu(acfge->admin_sgcl_eid),
		   le64_to_cpu(acfge->admin_base_time));
	seq_printf(s, "ADMIN_CYCLE_TIME_EXT: %u\n",
		   le32_to_cpu(acfge->admin_cycle_time_ext));
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_sgit_entry);

int netc_show_sgclt_entry(struct ntmp_priv *priv, struct seq_file *s,
			  u32 entry_id)
{
	struct ntmp_sgclt_entry *sgclt_entry __free(kfree);
	u32 sgclt_data_size, sgclt_cfge_size;
	struct sgclt_cfge_data *cfge;
	int i, err;

	sgclt_cfge_size = struct_size_t(struct sgclt_cfge_data, ge,
					SGCLT_MAX_GE_NUM);
	sgclt_data_size = struct_size(sgclt_entry, cfge.ge,
				      SGCLT_MAX_GE_NUM);
	sgclt_entry = kzalloc(sgclt_data_size, GFP_KERNEL);
	if (!sgclt_entry)
		return -ENOMEM;

	err = ntmp_sgclt_query_entry(&priv->cbdrs, entry_id, sgclt_entry,
				     sgclt_cfge_size);
	if (err) {
		seq_printf(s, "Query SGCLT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	cfge = &sgclt_entry->cfge;
	seq_printf(s, "Show stream gate control list table entry 0x%x\n", entry_id);
	seq_printf(s, "REF_COUNT: %u, CYCLE_TIME: %u, LIST_LENGTH: %u\n",
		   sgclt_entry->ref_count, le32_to_cpu(cfge->cycle_time),
		   cfge->list_length);
	seq_printf(s, "EXT_OIPV: %s, EXT_IPV: %lu, EXT_CTD: %s, EXT_GTST: %s\n",
		   is_en(cfge->ext_cfg & SGCLT_EXT_OIPV),
		   FIELD_GET(SGCLT_EXT_IPV, cfge->ext_cfg),
		   is_yes(cfge->ext_cfg & SGCLT_EXT_CTD),
		   cfge->ext_cfg & SGCLT_EXT_GTST ? "open" : "closed");

	for (i = 0; i < cfge->list_length + 1; i++) {
		u32 cfg = le32_to_cpu(cfge->ge[i].cfg);

		seq_printf(s, "Gate Entry: %d, Time Interval: %u\n",
			   i, le32_to_cpu(cfge->ge[i].interval));
		seq_printf(s, "IOMEN: %s, IOM: %lu\n",
			   is_en(cfg & SGCLT_IOMEN),
			   FIELD_GET(SGCLT_IOM, cfg));
		seq_printf(s, "OIPV: %s, IPV: %lu, CTD: %s, GTST: %s\n",
			   is_en(cfg & SGCLT_OIPV), FIELD_GET(SGCLT_IPV, cfg),
			   is_yes(cfg & SGCLT_CTD),
			   cfg & SGCLT_GTST ? "open" : "closed");
	}
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_sgclt_entry);

int netc_show_isct_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct isct_stse_data *stse __free(kfree);
	u32 sg_drop_cnt;
	int err;

	stse = kzalloc(sizeof(*stse), GFP_KERNEL);
	if (!stse)
		return -ENOMEM;

	err = ntmp_isct_operate_entry(&priv->cbdrs, entry_id,
				      NTMP_CMD_QUERY, stse);
	if (err) {
		seq_printf(s, "Query ISCT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	sg_drop_cnt = le32_to_cpu(stse->sg_drop_count);
	/* Workaround for ERR052134 on i.MX95 platform */
	if (priv->errata & NTMP_ERR052134) {
		u32 tmp;

		sg_drop_cnt >>= 9;

		tmp = le32_to_cpu(stse->resv3) & 0x1ff;
		sg_drop_cnt |= (tmp << 23);
	}

	seq_printf(s, "Show ingress stream count table entry 0x%x\n", entry_id);
	seq_printf(s, "RX_COUNT: %u, MSDU_DROP_COUNT: %u\n",
		   le32_to_cpu(stse->rx_count), le32_to_cpu(stse->msdu_drop_count));
	seq_printf(s, "POLICER_DROP_COUNT: %u, SG_DROP_COUNT: %u\n",
		   le32_to_cpu(stse->policer_drop_count), sg_drop_cnt);
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_isct_entry);

int netc_show_rpt_entry(struct ntmp_priv *priv, struct seq_file *s,
			u32 entry_id)
{
	struct ntmp_rpt_entry *rpt_entry __free(kfree);
	struct rpt_cfge_data *cfge;
	struct rpt_stse_data *stse;
	u32 bcf_bcs, bef_bes;
	u16 cfg;
	int err;

	rpt_entry = kzalloc(sizeof(*rpt_entry), GFP_KERNEL);
	if (!rpt_entry)
		return -ENOMEM;

	err = ntmp_rpt_query_entry(&priv->cbdrs, entry_id, rpt_entry);
	if (err) {
		seq_printf(s, "Query RPT entry ID (0x%x) failed\n", entry_id);
		return err;
	}

	cfge = &rpt_entry->cfge;
	stse = &rpt_entry->stse;
	bcf_bcs = le32_to_cpu(stse->bcf_bcs);
	bef_bes = le32_to_cpu(stse->bef_bes);
	cfg = le16_to_cpu(cfge->cfg);
	seq_printf(s, "Show rate policer table entry 0x%x\n", entry_id);
	seq_printf(s, "BYTE_COUNT: %llu, DROP_FRAMES: %u\n",
		   le64_to_cpu(stse->byte_count), le32_to_cpu(stse->drop_frames));
	seq_printf(s, "DR0_GRN_FRAMES: %u, DR1_GRN_FRAMES: %u\n",
		   le32_to_cpu(stse->dr0_grn_frames),
		   le32_to_cpu(stse->dr1_grn_frames));
	seq_printf(s, "DR2_YLW_FRAMES: %u, REMARK_YLW_FRAMES: %u\n",
		   le32_to_cpu(stse->dr2_ylw_frames),
		   le32_to_cpu(stse->remark_ylw_frames));
	seq_printf(s, "DR3_RED_FRAMES: %u, REMARK_RED_FRAMES: %u\n",
		   le32_to_cpu(stse->dr3_red_frames),
		   le32_to_cpu(stse->remark_red_frames));
	seq_printf(s, "LTS: 0x%x, BCI: %u, BEI: %u\n", le32_to_cpu(stse->lts),
		   le32_to_cpu(stse->bci), le32_to_cpu(stse->bei));
	seq_printf(s, "BCS: %lu, BCF: 0x%lx\n", FIELD_GET(RPT_BCS, bcf_bcs),
		   FIELD_GET(RPT_BCF, bcf_bcs));
	seq_printf(s, "BEF: %lu, BEI: 0x%lx\n", FIELD_GET(RPT_BES, bef_bes),
		   FIELD_GET(RPT_BEF, bef_bes));
	seq_printf(s, "CIR: %u, CBS: %u, EIR: %u, EBS: %u\n",
		   le32_to_cpu(cfge->cir), le32_to_cpu(cfge->cbs),
		   le32_to_cpu(cfge->eir), le32_to_cpu(cfge->ebs));
	seq_printf(s, "MREN: %s, DOY: %s, CM: %s, CF: %lu\n",
		   is_en(cfg & RPT_MREN), is_en(cfg & RPT_DOY),
		   cfg & RPT_DOY ? "aware" : "blind",
		   FIELD_GET(RPT_CF, cfg));
	seq_printf(s, "NDOR: %s, SDU type:%lu, FEN: %s, MR: %u\n",
		   is_en(cfg & RPT_NDOR), FIELD_GET(RPT_SDU_TYPE, cfg),
		   is_en(rpt_entry->fee.fen & RPT_FEN),
		   rpt_entry->pse.mr);
	seq_puts(s, "\n");

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_rpt_entry);

int netc_show_ipft_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct ntmp_ipft_entry *ipft_entry __free(kfree);
	struct ipft_keye_data *keye;
	struct ipft_cfge_data *cfge;
	u16 dscp, src_port;
	int i, err;
	u32 cfg;

	ipft_entry = kzalloc(sizeof(*ipft_entry), GFP_KERNEL);
	if (!ipft_entry)
		return -ENOMEM;

	err = ntmp_ipft_query_entry(&priv->cbdrs, entry_id, false, ipft_entry);
	if (err)
		return err;

	keye = &ipft_entry->keye;
	cfge = &ipft_entry->cfge;

	cfg = le32_to_cpu(cfge->cfg);
	dscp = le16_to_cpu(keye->dscp);

	seq_printf(s, "Show ingress port filter table entry:%u\n", entry_id);

	/* KEYE_DATA */
	seq_printf(s, "Precedence:%u, Frame attribute flags:0x%04x, mask:0x%04x\n",
		   keye->precedence, keye->frm_attr_flags, keye->frm_attr_flags_mask);
	seq_printf(s, "DSCP:0x%lx, mask:0x%lx\n", FIELD_GET(IPFT_DSCP, dscp),
		   FIELD_GET(IPFT_DSCP_MASK, dscp));

	if (priv->dev_type == NETC_DEV_SWITCH) {
		u8 port_id, port_mask;

		src_port = le16_to_cpu(keye->src_port);
		port_id = FIELD_GET(IPFT_SRC_PORT, src_port);
		port_mask = FIELD_GET(IPFT_SRC_PORT_MASK, src_port);
		seq_printf(s, "Switch Source Port ID:%d, mask:0x%02x\n",
			   port_id, port_mask);
	}

	seq_printf(s, "Outer VLAN TCI:0x%04x, mask:0x%04x\n",
		   ntohs(keye->outer_vlan_tci), ntohs(keye->outer_vlan_tci_mask));
	seq_printf(s, "Inner VLAN TCI:0x%04x, mask:0x%04x\n",
		   ntohs(keye->inner_vlan_tci), ntohs(keye->inner_vlan_tci_mask));
	seq_printf(s, "Destination MAC:%pM\n", keye->dmac);
	seq_printf(s, "Destination MAC mask:%pM\n", keye->dmac_mask);
	seq_printf(s, "Source MAC:%pM\n", keye->smac);
	seq_printf(s, "Source MAC mask:%pM\n", keye->smac_mask);
	seq_printf(s, "Ether Type:0x%04x, mask:0x%04x\n", ntohs(keye->ethertype),
		   ntohs(keye->ethertype_mask));
	seq_printf(s, "IP protocol:%u, mask:0x%02x\n",  keye->ip_protocol,
		   keye->ip_protocol_mask);
	seq_printf(s, "IP Source Address:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->ip_src[0]), ntohl(keye->ip_src[1]),
		   ntohl(keye->ip_src[2]), ntohl(keye->ip_src[3]));
	seq_printf(s, "IP Source Address mask:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->ip_src_mask[0]), ntohl(keye->ip_src_mask[1]),
		   ntohl(keye->ip_src_mask[2]), ntohl(keye->ip_src_mask[3]));
	seq_printf(s, "IP Destination Address:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->ip_dst[0]), ntohl(keye->ip_dst[1]),
		   ntohl(keye->ip_dst[2]), ntohl(keye->ip_dst[3]));
	seq_printf(s, "IP Destination Address mask:%08x:%08x:%08x:%08x\n",
		   ntohl(keye->ip_dst_mask[0]), ntohl(keye->ip_dst_mask[1]),
		   ntohl(keye->ip_dst_mask[2]), ntohl(keye->ip_dst_mask[3]));
	seq_printf(s, "L4 Source Port:%x, mask:0x%04x\n",
		   ntohs(keye->l4_src_port), ntohs(keye->l4_src_port_mask));
	seq_printf(s, "L4 Destination Port:%x, mask:0x%04x\n",
		   ntohs(keye->l4_dst_port), ntohs(keye->l4_dst_port_mask));
	for (i = 0; i < IPFT_MAX_PLD_LEN; i = i + 6) {
		seq_printf(s, "Payload %d~%d: %02x %02x %02x %02x %02x %02x\n",
			   i, i + 5, keye->byte[i].data, keye->byte[i + 1].data,
			   keye->byte[i + 2].data, keye->byte[i + 3].data,
			   keye->byte[i + 4].data, keye->byte[i + 5].data);
		seq_printf(s, "Payload Mask %d~%d: %02x %02x %02x %02x %02x %02x\n",
			   i, i + 5, keye->byte[i].mask, keye->byte[i + 1].mask,
			   keye->byte[i + 2].mask, keye->byte[i + 3].mask,
			   keye->byte[i + 4].mask, keye->byte[i + 5].mask);
	}

	/* STSE_DATA */
	seq_printf(s, "Match Count:%llu\n", le64_to_cpu(ipft_entry->match_count));

	/* CFGE_DATA */
	seq_printf(s, "Override internal Priority %s: %lu\n",
		   is_en(cfg & IPFT_OIPV), FIELD_GET(IPFT_IPV, cfg));
	seq_printf(s, "Override Drop Resilience %s: %lu\n",
		   is_en(IPFT_ODR & cfg), FIELD_GET(IPFT_DR, cfg));
	seq_printf(s, "Filter Forwarding Action: %lu\n",
		   FIELD_GET(IPFT_FLTFA, cfg));
	seq_printf(s, "Filter Action: %lu\n", FIELD_GET(IPFT_FLTA, cfg));
	seq_printf(s, "Relative Precedent Resolution: %lu\n",
		   FIELD_GET(IPFT_RPR, cfg));
	seq_printf(s, "Target For Selected Filter Action: 0x%x\n",
		   le32_to_cpu(cfge->flta_tgt));

	if (priv->dev_type == NETC_DEV_SWITCH) {
		seq_printf(s, "Ingress Mirroring %s, Cut through disable: %s\n",
			   is_en(cfg & IPFT_IMIRE), is_yes(cfg & IPFT_CTD));
		seq_printf(s, "Host Reason: %lu, Timestamp Capture %s\n",
			   FIELD_GET(IPFT_HR, cfg), is_en(cfg & IPFT_TIMECAPE));
		seq_printf(s, "Report Receive Timestamp: %s\n",
			   is_yes(cfg & IPFT_RRT));
		seq_printf(s, "Event monitor event ID: %lu\n",
			   FIELD_GET(IPFT_EVMEID, cfg));
	} else {
		seq_printf(s, "Wake-on-LAN Trigger %s\n",
			   is_en(cfg & IPFT_WOLTE));
		seq_printf(s, "Bypass L2 Filtering: %s\n",
			   is_yes(cfg & IPFT_BL2F));
	}

	seq_puts(s, "\n");

	return err;
}
EXPORT_SYMBOL_GPL(netc_show_ipft_entry);

int netc_show_tgst_entry(struct ntmp_priv *priv, struct seq_file *s,
			 u32 entry_id)
{
	struct tgst_query_data *qdata __free(kfree);
	int i, err;

	qdata = kzalloc(sizeof(*qdata), GFP_KERNEL);
	if (!qdata)
		return -ENOMEM;

	err = ntmp_tgst_query_entry(&priv->cbdrs, entry_id, qdata);
	if (err)
		return err;

	seq_puts(s, "Dump Time Gate Scheduling Table Entry:\n");
	seq_printf(s, "Entry ID:%d\n", entry_id);
	seq_printf(s, "Admin Base Time:%llu\n", le64_to_cpu(qdata->admin_bt));
	seq_printf(s, "Admin Cycle Time:%u\n", le32_to_cpu(qdata->admin_ct));
	seq_printf(s, "Admin Cycle Extend Time:%u\n",
		   le32_to_cpu(qdata->admin_ct_ext));
	seq_printf(s, "Admin Control List Length:%u\n",
		   le16_to_cpu(qdata->admin_cl_len));
	for (i = 0; i < le16_to_cpu(qdata->admin_cl_len); i++) {
		seq_printf(s, "Gate Entry %d info:\n", i);
		seq_printf(s, "\tAdmin time interval:%u\n",
			   le32_to_cpu(qdata->cfge_ge[i].interval));
		seq_printf(s, "\tAdmin Traffic Class states:%02x\n",
			   qdata->cfge_ge[i].tc_state);
		seq_printf(s, "\tAdministrative gate operation type:%u\n",
			   qdata->cfge_ge[i].hr_cb);
	}

	seq_printf(s, "Config Change Time:%llu\n", le64_to_cpu(qdata->oper_cfg_ct));
	seq_printf(s, "Config Change Error:%llu\n", le64_to_cpu(qdata->oper_cfg_ce));
	seq_printf(s, "Operation Base Time:%llu\n", le64_to_cpu(qdata->oper_bt));
	seq_printf(s, "Operation Cycle Time:%u\n", le32_to_cpu(qdata->oper_ct));
	seq_printf(s, "Operation Cycle Extend Time:%u\n",
		   le32_to_cpu(qdata->oper_ct_ext));
	seq_printf(s, "Operation Control List Length:%u\n",
		   le16_to_cpu(qdata->oper_cl_len));
	for (i = 0; i < le16_to_cpu(qdata->oper_cl_len); i++) {
		seq_printf(s, "Gate Entry %d info:\n", i);
		seq_printf(s, "\tOperation time interval:%u\n",
			   le32_to_cpu(qdata->olse_ge[i].interval));
		seq_printf(s, "\tOperation Traffic Class states:%02x\n",
			   qdata->olse_ge[i].tc_state);
		seq_printf(s, "\tOperation gate operation type:%u\n",
			   qdata->olse_ge[i].hr_cb);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(netc_show_tgst_entry);

void netc_show_ipft_flower(struct seq_file *s, struct netc_flower_rule *rule)
{
	struct ntmp_ipft_entry *ipft_entry = rule->key_tbl->ipft_entry;
	struct ntmp_ist_entry *ist_entry = rule->key_tbl->ist_entry;
	u32 ipft_cfg = le32_to_cpu(ipft_entry->cfge.cfg);
	u32 rpt_eid = NTMP_NULL_ENTRY_ID;

	seq_printf(s, "IPFT entry ID:0x%x\n", ipft_entry->entry_id);
	if (ist_entry) {
		seq_printf(s, "IST entry ID: 0x%x\n", ist_entry->entry_id);
		seq_printf(s, "ISCT entry ID: 0x%x\n", ist_entry->cfge.isc_eid);
		rpt_eid = le32_to_cpu(ist_entry->cfge.rp_eid);
	}

	if (FIELD_GET(IPFT_FLTA, ipft_cfg) == IPFT_FLTA_RP)
		rpt_eid = le32_to_cpu(ipft_entry->cfge.flta_tgt);

	if (rpt_eid != NTMP_NULL_ENTRY_ID)
		seq_printf(s, "RPT entry ID: 0x%x\n", rpt_eid);
}
EXPORT_SYMBOL_GPL(netc_show_ipft_flower);
