// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC switch driver
 * Copyright 2025 NXP
 */

#include "netc_switch.h"

#define NETC_IPFT_KEYS	(BIT_ULL(FLOW_DISSECTOR_KEY_VLAN) | \
			 BIT_ULL(FLOW_DISSECTOR_KEY_CVLAN) | \
			 BIT_ULL(FLOW_DISSECTOR_KEY_BASIC) | \
			 BIT_ULL(FLOW_DISSECTOR_KEY_IPV4_ADDRS) | \
			 BIT_ULL(FLOW_DISSECTOR_KEY_IPV6_ADDRS) | \
			 BIT_ULL(FLOW_DISSECTOR_KEY_PORTS))

static const struct netc_flower netc_flow_filter[] = {
	{
		BIT_ULL(FLOW_ACTION_GATE),
		BIT_ULL(FLOW_ACTION_POLICE),
		BIT_ULL(FLOW_DISSECTOR_KEY_ETH_ADDRS) |
		BIT_ULL(FLOW_DISSECTOR_KEY_VLAN),
		FLOWER_TYPE_PSFP
	},
	{
		BIT_ULL(FLOW_ACTION_TRAP),
		BIT_ULL(FLOW_ACTION_REDIRECT) |
		BIT_ULL(FLOW_ACTION_POLICE),
		NETC_IPFT_KEYS,
		FLOWER_TYPE_TRAP
	},
	{
		BIT_ULL(FLOW_ACTION_REDIRECT),
		BIT_ULL(FLOW_ACTION_POLICE),
		NETC_IPFT_KEYS,
		FLOWER_TYPE_REDIRECT
	},
	{
		BIT_ULL(FLOW_ACTION_POLICE),
		0,
		NETC_IPFT_KEYS,
		FLOWER_TYPE_POLICE
	},
};

int netc_tc_query_caps(struct tc_query_caps_base *base)
{
	switch (base->type) {
	case TC_SETUP_QDISC_MQPRIO: {
		struct tc_mqprio_caps *caps = base->caps;

		caps->validate_queue_counts = true;

		return 0;
	}
	case TC_SETUP_QDISC_TAPRIO: {
		struct tc_taprio_caps *caps = base->caps;

		caps->supports_queue_max_sdu = true;

		return 0;
	}
	default:
		return -EOPNOTSUPP;
	}
}

static void netc_port_change_preemptible_tcs(struct netc_port *port,
					     unsigned long preemptible_tcs)
{
	if (!port->caps.pmac)
		return;

	port->preemptible_tcs = preemptible_tcs;
	netc_port_mm_commit_preemptible_tcs(port);
}

static void netc_port_reset_mqprio(struct netc_port *port)
{
	struct net_device *ndev = port->dp->user;

	netdev_reset_tc(ndev);
	netif_set_real_num_tx_queues(ndev, NETC_TC_NUM);
	netc_port_change_preemptible_tcs(port, 0);
}

int netc_tc_setup_mqprio(struct netc_switch *priv, int port_id,
			 struct tc_mqprio_qopt_offload *mqprio)
{
	struct netc_port *port = NETC_PORT(priv, port_id);
	struct tc_mqprio_qopt *qopt = &mqprio->qopt;
	struct net_device *ndev = port->dp->user;
	struct netlink_ext_ack *extack;
	u8 num_tc = qopt->num_tc;
	int tc, err;

	extack = mqprio->extack;

	if (!num_tc) {
		netc_port_reset_mqprio(port);
		return 0;
	}

	err = netdev_set_num_tc(ndev, num_tc);
	if (err)
		return err;

	for (tc = 0; tc < num_tc; tc++) {
		if (qopt->count[tc] != 1) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Only one TXQ per TC supported");
			return -EINVAL;
		}

		err = netdev_set_tc_queue(ndev, tc, 1, qopt->offset[tc]);
		if (err)
			goto reset_mqprio;
	}

	err = netif_set_real_num_tx_queues(ndev, num_tc);
	if (err)
		goto reset_mqprio;

	netc_port_change_preemptible_tcs(port, mqprio->preemptible_tcs);

	return 0;

reset_mqprio:
	netc_port_reset_mqprio(port);

	return err;
}

static bool netc_port_tc_cbs_is_enable(struct netc_port *port, int tc)
{
	return !!(netc_port_rd(port, NETC_PTCCBSR2(tc)) & PTCCBSR2_CBSE);
}

static void netc_port_enable_time_gating(struct netc_port *port, bool en)
{
	u32 old_val, val;

	old_val = netc_port_rd(port, NETC_PTGSCR);
	val = u32_replace_bits(old_val, en ? 1 : 0, PTGSCR_TGE);
	if (val != old_val)
		netc_port_wr(port, NETC_PTGSCR, val);
}

static void netc_port_set_tc_cbs_params(struct netc_port *port, int tc,
					bool en, u32 idleslope)
{
	if (en) {
		u32 val = PTCCBSR2_CBSE;

		val |= idleslope & PTCCBSR2_IDLESLOPE;

		netc_port_wr(port, NETC_PTCCBSR1(tc), 0xffffffff);
		netc_port_wr(port, NETC_PTCCBSR2(tc), val);
	} else {
		netc_port_wr(port, NETC_PTCCBSR1(tc), 0);
		netc_port_wr(port, NETC_PTCCBSR2(tc), 0);
	}
}

static u32 netc_port_get_tc_cbs_idleslope(struct netc_port *port, int tc)
{
	return netc_port_rd(port, NETC_PTCCBSR2(tc)) & PTCCBSR2_IDLESLOPE;
}

static int netc_port_setup_cbs(struct netc_port *port,
			       struct tc_cbs_qopt_offload *cbs)
{
	struct net_device *ndev = port->dp->user;
	u8 num_tc = netdev_get_num_tc(ndev);
	u8 top_prio_tc, second_prio_tc, tc;
	u32 total_idleslope;

	top_prio_tc = num_tc - 1;
	second_prio_tc = num_tc - 2;
	tc = netdev_txq_to_tc(ndev, cbs->queue);
	if (tc != top_prio_tc && tc != second_prio_tc)
		return -EOPNOTSUPP;

	if (!cbs->enable) {
		/* Make sure the other TC that are numerically lower than
		 * this TC have been disabled.
		 */
		if (tc == top_prio_tc &&
		    netc_port_tc_cbs_is_enable(port, second_prio_tc)) {
			netdev_err(ndev, "Disable TC%d before disable TC%d\n",
				   second_prio_tc, tc);
			return -EINVAL;
		}

		netc_port_set_tc_cbs_params(port, tc, false, 0);

		if (tc == top_prio_tc) {
			if (!(port->offloads & NETC_FLAG_QBV))
				netc_port_enable_time_gating(port, false);

			port->offloads &= ~NETC_FLAG_QAV;
		}

		return 0;
	}

	/* The unit of idleslope and sendslope is kbps. The sendslope should be
	 * a negative number, it can be calculated as follows, IEEE 802.1Q-2014
	 * Section 8.6.8.2 item g):
	 * sendslope = idleslope - port_transmit_rate
	 */
	if (cbs->idleslope - cbs->sendslope != port->speed * 1000L ||
	    cbs->idleslope < 0 || cbs->sendslope > 0)
		return -EOPNOTSUPP;

	total_idleslope = cbs->idleslope;
	/* Make sure the credit-based shaper of highest priority TC has been
	 * enabled before the secondary priority TC.
	 */
	if (tc == second_prio_tc) {
		if (!netc_port_tc_cbs_is_enable(port, top_prio_tc)) {
			netdev_err(ndev, "Enable TC%d first before enable TC%d\n",
				   top_prio_tc, second_prio_tc);
			return -EINVAL;
		}
		total_idleslope += netc_port_get_tc_cbs_idleslope(port, top_prio_tc);
	}

	/* The unit of port speed is Mbps */
	if (total_idleslope > port->speed * 1000L) {
		netdev_err(ndev,
			   "The total bandwidth of CBS can't exceed the link rate\n");
		return -EINVAL;
	}

	/* If CBS is going to be used in combination with frame preemption, then time
	 * gate scheduling should be enabled for the port.
	 */
	if (port->offloads & NETC_FLAG_QBU)
		netc_port_enable_time_gating(port, true);

	netc_port_set_tc_cbs_params(port, tc, true, cbs->idleslope);

	port->offloads |= NETC_FLAG_QAV;

	return 0;
}

int netc_tc_setup_cbs(struct netc_switch *priv, int port_id,
		      struct tc_cbs_qopt_offload *cbs)
{
	return netc_port_setup_cbs(priv->ports[port_id], cbs);
}

static bool netc_port_get_tge_status(struct netc_port *port)
{
	u32 val;

	val = netc_port_rd(port, NETC_PTGSCR);
	if (val & PTGSCR_TGE)
		return true;

	return false;
}

static int netc_port_setup_taprio(struct netc_port *port,
				  struct tc_taprio_qopt_offload *taprio)
{
	struct netc_switch *priv = port->switch_priv;
	u32 entry_id = port->index;
	bool tge;
	int err;

	/* Set the maximum frame size for each traffic class */
	netc_port_set_all_tc_msdu(port, taprio->max_sdu);

	tge = netc_port_get_tge_status(port);
	if (!tge)
		netc_port_enable_time_gating(port, true);

	err = netc_setup_taprio(&priv->ntmp, entry_id, taprio);
	if (err)
		goto disable_time_gating;

	port->offloads |= NETC_FLAG_QBV;

	return 0;

disable_time_gating:
	if (!tge)
		netc_port_enable_time_gating(port, false);

	netc_port_set_all_tc_msdu(port, NULL);

	return err;
}

static int netc_tc_taprio_replace(struct netc_switch *priv, int port_id,
				  struct tc_taprio_qopt_offload *taprio)
{
	struct netc_port *port = NETC_PORT(priv, port_id);
	struct netlink_ext_ack *extack = taprio->extack;
	int err;

	err = netc_tc_setup_mqprio(priv, port_id, &taprio->mqprio);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Setup mqprio failed");
		return err;
	}

	err = netc_port_setup_taprio(port, taprio);
	if (err)
		netc_port_reset_mqprio(port);

	return err;
}

static int netc_port_reset_taprio(struct netc_port *port)
{
	/* Remove both operational and administrative gate control list from
	 * the corresponding table entry by disabling time gate scheduling on
	 * the port.
	 */
	netc_port_enable_time_gating(port, false);

	/* Time gate scheduling should be enabled for the port if credit-based
	 * shaper is going to be used in combination with frame preemption.
	 */
	if (port->offloads & NETC_FLAG_QAV && port->offloads & NETC_FLAG_QBU)
		netc_port_enable_time_gating(port, true);

	/* Reset TC max SDU */
	netc_port_set_all_tc_msdu(port, NULL);

	port->offloads &= ~NETC_FLAG_QBV;

	return 0;
}

static int netc_tc_taprio_destroy(struct netc_switch *priv, int port_id)
{
	struct netc_port *port = NETC_PORT(priv, port_id);

	netc_port_reset_taprio(port);
	netc_port_reset_mqprio(port);

	return 0;
}

int netc_tc_setup_taprio(struct netc_switch *priv, int port_id,
			 struct tc_taprio_qopt_offload *taprio)
{
	switch (taprio->cmd) {
	case TAPRIO_CMD_REPLACE:
		return netc_tc_taprio_replace(priv, port_id, taprio);
	case TAPRIO_CMD_DESTROY:
		return netc_tc_taprio_destroy(priv, port_id);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct netc_flower *netc_parse_tc_flower(u64 actions, u64 keys)
{
	u64 key_acts, all_acts;
	int i;

	for (i = 0; i < ARRAY_SIZE(netc_flow_filter); i++) {
		key_acts = netc_flow_filter[i].key_acts;
		all_acts = netc_flow_filter[i].key_acts |
			   netc_flow_filter[i].opt_acts;

		/* key_acts must be matched */
		if ((actions & key_acts) == key_acts &&
		    (actions & all_acts) == actions &&
		    keys & netc_flow_filter[i].keys)
			return &netc_flow_filter[i];
	}

	return NULL;
}

static int netc_add_trap_redirect_key_tbl(struct ntmp_priv *ntmp,
					  struct netc_flower_key_tbl **key_tbl,
					  struct ipft_keye_data *ipft_key,
					  u64 actions, int redirect_port,
					  struct netlink_ext_ack *extack)
{
	struct ntmp_ist_entry *ist_entry __free(kfree) = NULL;
	struct netc_flower_key_tbl *new_tbl __free(kfree);
	struct ntmp_ipft_entry *ipft_entry __free(kfree);
	struct ipft_cfge_data *ipft_cfge;
	struct ist_cfge_data *ist_cfge;
	u32 ipft_cfg = 0;
	u32 ist_cfg = 0;
	u8 fa;

	new_tbl = kzalloc(sizeof(*new_tbl), GFP_KERNEL);
	if (!new_tbl)
		return -ENOMEM;

	ipft_entry = kzalloc(sizeof(*ipft_entry), GFP_KERNEL);
	if (!ipft_entry)
		return -ENOMEM;

	ipft_cfge = &ipft_entry->cfge;
	ipft_entry->keye = *ipft_key;

	if (actions & BIT_ULL(FLOW_ACTION_REDIRECT)) {
		if (redirect_port < 0) {
			NL_SET_ERR_MSG_MOD(extack, "Invalid redirected port");
			return -EINVAL;
		}

		ist_entry = kzalloc(sizeof(*ist_entry), GFP_KERNEL);
		if (!ist_entry)
			return -ENOMEM;

		ist_entry->entry_id = ntmp_lookup_free_eid(ntmp->ist_eid_bitmap,
							   ntmp->caps.ist_num_entries);
		if (ist_entry->entry_id == NTMP_NULL_ENTRY_ID) {
			NL_SET_ERR_MSG_MOD(extack, "No available IST entry is found");
			return -ENOSPC;
		}

		fa = IST_SWITCH_FA_SF;
		if (actions & BIT_ULL(FLOW_ACTION_TRAP)) {
			fa = IST_SWITCH_FA_SF_COPY;
			ist_cfg |= FIELD_PREP(IST_HR, NETC_HR_TRAP);
			ist_cfg |= IST_RRT;
			ist_cfg |= IST_TIMERCAPE;
		}

		switch (ntmp->cbdrs.tbl.ist_ver) {
		case NTMP_TBL_VER1:
			ist_cfg |= FIELD_PREP(IST_V1_FA, fa);
			ist_cfg |= FIELD_PREP(IST_V1_SDU_TYPE, SDU_TYPE_MPDU);
			break;
		case NTMP_TBL_VER0:
			ist_cfg |= FIELD_PREP(IST_V0_FA, fa);
			ist_cfg |= FIELD_PREP(IST_V0_SDU_TYPE, SDU_TYPE_MPDU);
			break;
		default:
			NL_SET_ERR_MSG_MOD(extack, "Unknown IST version");
			ntmp_clear_eid_bitmap(ntmp->ist_eid_bitmap,
					      ist_entry->entry_id);

			return -EINVAL;
		}

		ipft_cfg |= FIELD_PREP(IPFT_FLTFA, IPFT_FLTFA_PERMIT);
		ipft_cfg |= FIELD_PREP(IPFT_FLTA, IPFT_FLTA_IS);
		ipft_cfge->flta_tgt = cpu_to_le32(ist_entry->entry_id);

		ist_cfge = &ist_entry->cfge;
		ist_cfge->cfg = cpu_to_le32(ist_cfg);
		ist_cfge->bitmap_evmeid = cpu_to_le32(BIT(redirect_port) & 0xffffff);

		netc_init_ist_entry_eids(ntmp, ist_entry);
	} else {
		ipft_cfg |= FIELD_PREP(IPFT_FLTFA, IPFT_FLTFA_REDIRECT);
		ipft_cfg |= FIELD_PREP(IPFT_HR, NETC_HR_TRAP);
		ipft_cfg |= IPFT_TIMECAPE;
		ipft_cfg |= IPFT_RRT;
	}

	ipft_cfge->cfg = cpu_to_le32(ipft_cfg);
	new_tbl->tbl_type = FLOWER_KEY_TBL_IPFT;

	new_tbl->ipft_entry = no_free_ptr(ipft_entry);
	new_tbl->ist_entry = no_free_ptr(ist_entry);
	*key_tbl = no_free_ptr(new_tbl);

	return 0;
}

static int netc_set_trap_redirect_tables(struct ntmp_priv *ntmp,
					 struct ntmp_ipft_entry *ipft_entry,
					 struct ntmp_ist_entry *ist_entry,
					 struct ntmp_isct_entry *isct_entry,
					 struct ntmp_rpt_entry *rpt_entry)
{
	struct netc_cbdrs *cbdrs = &ntmp->cbdrs;
	int err;

	if (rpt_entry) {
		err = ntmp_rpt_add_or_update_entry(cbdrs, rpt_entry);
		if (err)
			return err;
	}

	if (isct_entry) {
		err = ntmp_isct_operate_entry(cbdrs, isct_entry->entry_id,
					      NTMP_CMD_ADD, NULL);
		if (err)
			goto delete_rpt_entry;
	}

	if (ist_entry) {
		err = ntmp_ist_add_or_update_entry(cbdrs, ist_entry);
		if (err)
			goto delete_isct_entry;
	}

	err = ntmp_ipft_add_entry(cbdrs, &ipft_entry->entry_id, ipft_entry);
	if (err)
		goto delete_ist_entry;

	return 0;

delete_ist_entry:
	if (ist_entry)
		ntmp_ist_delete_entry(cbdrs, ist_entry->entry_id);
delete_isct_entry:
	if (isct_entry)
		ntmp_isct_operate_entry(cbdrs, ist_entry->entry_id,
					NTMP_CMD_DELETE, NULL);
delete_rpt_entry:
	if (rpt_entry)
		ntmp_rpt_delete_entry(cbdrs, rpt_entry->entry_id);

	return err;
}

static int netc_setup_trap_redirect(struct ntmp_priv *ntmp, int port_id,
				    struct flow_cls_offload *f)
{
	struct flow_rule *cls_rule = flow_cls_offload_flow_rule(f);
	struct ntmp_isct_entry *isct_entry __free(kfree) = NULL;
	struct ntmp_rpt_entry *rpt_entry __free(kfree) = NULL;
	struct netc_flower_rule *rule __free(kfree) = NULL;
	struct netlink_ext_ack *extack = f->common.extack;
	struct ipft_keye_data *ipft_keye __free(kfree);
	struct flow_action_entry *redirect_act = NULL;
	struct flow_action_entry *police_act = NULL;
	struct netc_flower_key_tbl *key_tbl = NULL;
	struct flow_action_entry *trap_act = NULL;
	struct flow_action_entry *action_entry;
	struct ntmp_ipft_entry *ipft_entry;
	struct netc_flower_rule *old_rule;
	u32 isct_eid = NTMP_NULL_ENTRY_ID;
	struct ntmp_ist_entry *ist_entry;
	unsigned long cookie = f->cookie;
	u16 prio = f->common.prio;
	struct dsa_port *to_dp;
	int redirect_port = -1;
	u64 actions = 0;
	u16 msdu = 0;
	int i, err;

	guard(mutex)(&ntmp->flower_lock);
	if (netc_find_flower_rule_by_cookie(ntmp, port_id, cookie)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Cannot add new rule with same cookie");
		return -EINVAL;
	}

	rule = kzalloc(sizeof(*rule), GFP_KERNEL);
	if (!rule)
		return -ENOMEM;

	rule->port_id = port_id;
	rule->cookie = cookie;

	flow_action_for_each(i, action_entry, &cls_rule->action)
		if (action_entry->id == FLOW_ACTION_TRAP) {
			trap_act = action_entry;
			actions |= BIT_ULL(FLOW_ACTION_TRAP);
		} else if (action_entry->id == FLOW_ACTION_REDIRECT) {
			redirect_act = action_entry;
			actions |= BIT_ULL(FLOW_ACTION_REDIRECT);
		} else if (action_entry->id == FLOW_ACTION_POLICE) {
			police_act = action_entry;
			actions |= BIT_ULL(FLOW_ACTION_POLICE);
		}

	if (!trap_act && !redirect_act) {
		NL_SET_ERR_MSG_MOD(extack, "Invalid actions");
		return -EINVAL;
	} else if (trap_act) {
		rule->flower_type = FLOWER_TYPE_TRAP;
	} else {
		rule->flower_type = FLOWER_TYPE_REDIRECT;
	}

	ipft_keye = kzalloc(sizeof(*ipft_keye), GFP_KERNEL);
	if (!ipft_keye)
		return -ENOMEM;

	err = netc_ipft_keye_construct(cls_rule, port_id, prio,
				       ipft_keye, extack);
	if (err)
		return err;

	old_rule = netc_find_flower_rule_by_key(ntmp, FLOWER_KEY_TBL_IPFT,
						ipft_keye);
	if (old_rule) {
		NL_SET_ERR_MSG_MOD(extack,
				   "The IPFT key has been used by existing rule");
		return -EINVAL;
	}

	if (redirect_act) {
		to_dp = dsa_port_from_netdev(redirect_act->dev);
		if (IS_ERR(to_dp)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Destination is not a switch port");
			return -EOPNOTSUPP;
		}

		redirect_port = to_dp->index;
	}

	if (police_act) {
		err = netc_police_entry_validate(ntmp, &cls_rule->action,
						 police_act, extack);
		if (err)
			return err;

		msdu = police_act->police.mtu;

		if (police_act->police.rate_bytes_ps ||
		    police_act->police.burst) {
			rpt_entry = kzalloc(sizeof(*rpt_entry), GFP_KERNEL);
			if (!rpt_entry) {
				err = -ENOMEM;
				goto clear_rpt_eid_bit;
			}

			netc_rpt_entry_config(police_act, rpt_entry);
		}
	}

	err = netc_add_trap_redirect_key_tbl(ntmp, &key_tbl, ipft_keye, actions,
					     redirect_port, extack);
	if (err)
		goto clear_rpt_eid_bit;

	ipft_entry = key_tbl->ipft_entry;
	ist_entry = key_tbl->ist_entry;

	if (ist_entry) {
		isct_eid = ntmp_lookup_free_eid(ntmp->isct_eid_bitmap,
						ntmp->caps.isct_num_entries);
		if (isct_eid == NTMP_NULL_ENTRY_ID) {
			NL_SET_ERR_MSG_MOD(extack, "No available ISCT entry is found");
			err = -ENOSPC;
			goto free_key_tbl;
		}

		isct_entry = kzalloc(sizeof(*isct_entry), GFP_KERNEL);
		if (!isct_entry) {
			err = -ENOMEM;
			goto clear_isct_eid_bit;
		}

		isct_entry->entry_id = isct_eid;
		ist_entry->cfge.isc_eid = cpu_to_le32(isct_eid);
		ist_entry->cfge.msdu = cpu_to_le16(msdu);

		if (rpt_entry) {
			u32 ist_cfg = le32_to_cpu(ist_entry->cfge.cfg) | IST_ORP;

			ist_entry->cfge.cfg = cpu_to_le32(ist_cfg);
			ist_entry->cfge.rp_eid = cpu_to_le32(rpt_entry->entry_id);
		}
	} else if (rpt_entry) {
		u32 ipft_cfg = le32_to_cpu(ipft_entry->cfge.cfg);

		ipft_cfg = u32_replace_bits(ipft_cfg, IPFT_FLTA_RP, IPFT_FLTA);
		ipft_entry->cfge.cfg = cpu_to_le32(ipft_cfg);
		ipft_entry->cfge.flta_tgt = cpu_to_le32(rpt_entry->entry_id);
	}

	err = netc_set_trap_redirect_tables(ntmp, ipft_entry, ist_entry,
					    isct_entry, rpt_entry);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Failed to add new table entries");
		goto clear_isct_eid_bit;
	}

	rule->lastused = jiffies;
	rule->key_tbl = key_tbl;

	hlist_add_head(&no_free_ptr(rule)->node, &ntmp->flower_list);

	return 0;

clear_isct_eid_bit:
	if (isct_eid != NTMP_NULL_ENTRY_ID)
		ntmp_clear_eid_bitmap(ntmp->isct_eid_bitmap, isct_eid);
free_key_tbl:
	netc_free_flower_key_tbl(ntmp, key_tbl);
clear_rpt_eid_bit:
	if (police_act)
		ntmp_clear_eid_bitmap(ntmp->rpt_eid_bitmap,
				      police_act->hw_index);

	return err;
}

int netc_port_flow_cls_replace(struct netc_port *port,
			       struct flow_cls_offload *f)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(f);
	struct netlink_ext_ack *extack = f->common.extack;
	struct netc_switch *priv = port->switch_priv;
	struct flow_action *action = &rule->action;
	struct flow_dissector *dissector;
	const struct netc_flower *flower;
	struct flow_action_entry *entry;
	u64 actions = 0;
	int i;

	dissector = rule->match.dissector;

	if (!flow_action_has_entries(action)) {
		NL_SET_ERR_MSG_MOD(extack, "At least one action is needed");
		return -EINVAL;
	}

	if (!flow_action_basic_hw_stats_check(action, extack))
		return -EOPNOTSUPP;

	flow_action_for_each(i, entry, action)
		actions |= BIT_ULL(entry->id);

	flower = netc_parse_tc_flower(actions, dissector->used_keys);
	if (!flower) {
		NL_SET_ERR_MSG_MOD(extack, "Unsupported actions or keys");
		return -EOPNOTSUPP;
	}

	switch (flower->type) {
	case FLOWER_TYPE_PSFP:
		return netc_setup_psfp(&priv->ntmp, port->index, f);
	case FLOWER_TYPE_TRAP:
	case FLOWER_TYPE_REDIRECT:
		return netc_setup_trap_redirect(&priv->ntmp, port->index, f);
	case FLOWER_TYPE_POLICE:
		return netc_setup_police(&priv->ntmp, port->index, f);
	default:
		NL_SET_ERR_MSG_MOD(extack, "Unsupported flower type");
		return -EOPNOTSUPP;
	}
}

static void netc_delete_trap_redirect_flower_rule(struct ntmp_priv *ntmp,
						  struct netc_flower_rule *rule)
{
	struct netc_flower_key_tbl *key_tbl = rule->key_tbl;
	struct netc_cbdrs *cbdrs = &ntmp->cbdrs;
	struct ntmp_ipft_entry *ipft_entry;
	struct ntmp_ist_entry *ist_entry;
	u32 rpt_eid = NTMP_NULL_ENTRY_ID;
	u32 isct_eid;

	ipft_entry = key_tbl->ipft_entry;
	ist_entry = key_tbl->ist_entry;

	ntmp_ipft_delete_entry(cbdrs, ipft_entry->entry_id);

	if (ist_entry) {
		ntmp_ist_delete_entry(cbdrs, ist_entry->entry_id);

		isct_eid = le32_to_cpu(ist_entry->cfge.isc_eid);
		if (isct_eid != NTMP_NULL_ENTRY_ID) {
			ntmp_isct_operate_entry(cbdrs, isct_eid,
						NTMP_CMD_DELETE, NULL);
			ntmp_clear_eid_bitmap(ntmp->isct_eid_bitmap, isct_eid);
		}

		rpt_eid = le32_to_cpu(ist_entry->cfge.rp_eid);
	} else {
		u32 ipft_cfg = le32_to_cpu(ipft_entry->cfge.cfg);

		if (FIELD_GET(IPFT_FLTA, ipft_cfg) == IPFT_FLTA_RP)
			rpt_eid = le32_to_cpu(ipft_entry->cfge.flta_tgt);
	}

	if (rpt_eid != NTMP_NULL_ENTRY_ID) {
		ntmp_rpt_delete_entry(cbdrs, rpt_eid);
		ntmp_clear_eid_bitmap(ntmp->rpt_eid_bitmap, rpt_eid);
	}

	netc_free_flower_key_tbl(ntmp, key_tbl);

	hlist_del(&rule->node);
	kfree(rule);
}

static void netc_delete_flower_rule(struct ntmp_priv *ntmp,
				    struct netc_flower_rule *rule)
{
	switch (rule->flower_type) {
	case FLOWER_TYPE_PSFP:
		netc_delete_psfp_flower_rule(ntmp, rule);
		break;
	case FLOWER_TYPE_TRAP:
	case FLOWER_TYPE_REDIRECT:
		netc_delete_trap_redirect_flower_rule(ntmp, rule);
		break;
	case FLOWER_TYPE_POLICE:
		netc_delete_police_flower_rule(ntmp, rule);
		break;
	default:
		break;
	}
}

int netc_port_flow_cls_destroy(struct netc_port *port,
			       struct flow_cls_offload *f)
{
	struct netlink_ext_ack *extack = f->common.extack;
	struct netc_switch *priv = port->switch_priv;
	struct ntmp_priv *ntmp = &priv->ntmp;
	unsigned long cookie = f->cookie;
	struct netc_flower_rule *rule;

	guard(mutex)(&ntmp->flower_lock);
	rule = netc_find_flower_rule_by_cookie(ntmp, port->index, cookie);
	if (!rule) {
		NL_SET_ERR_MSG_MOD(extack, "Cannot find the rule");
		return -EINVAL;
	}

	netc_delete_flower_rule(ntmp, rule);

	return 0;
}

static int netc_trap_redirect_flower_stat(struct ntmp_priv *ntmp,
					  struct netc_flower_rule *rule,
					  u64 *byte_cnt, u64 *pkt_cnt,
					  u64 *drop_cnt)
{
	struct ntmp_ipft_entry *ipft_entry = rule->key_tbl->ipft_entry;
	struct ntmp_ist_entry *ist_entry = rule->key_tbl->ist_entry;
	struct ntmp_ipft_entry *ipft_query __free(kfree) = NULL;
	u32 isct_eid = NTMP_NULL_ENTRY_ID;
	struct isct_stse_data stse = {0};
	int err;

	if (ist_entry)
		isct_eid = le32_to_cpu(ist_entry->cfge.isc_eid);

	if (isct_eid != NTMP_NULL_ENTRY_ID) {
		err = ntmp_isct_operate_entry(&ntmp->cbdrs, isct_eid,
					      NTMP_CMD_QU, &stse);
		if (err)
			return err;

		*pkt_cnt = le32_to_cpu(stse.rx_count);
		*drop_cnt = le32_to_cpu(stse.msdu_drop_count) +
			    le32_to_cpu(stse.sg_drop_count) +
			    le32_to_cpu(stse.policer_drop_count);
	} else {
		ipft_query = kzalloc(sizeof(*ipft_query), GFP_KERNEL);
		if (!ipft_query)
			return -ENOMEM;

		err = ntmp_ipft_query_entry(&ntmp->cbdrs, ipft_entry->entry_id,
					    true, ipft_query);
		if (err)
			return err;

		*pkt_cnt = le64_to_cpu(ipft_query->match_count);
	}

	return 0;
}

int netc_port_flow_cls_stats(struct netc_port *port,
			     struct flow_cls_offload *f)
{
	struct netlink_ext_ack *extack = f->common.extack;
	struct netc_switch *priv = port->switch_priv;
	u64 pkt_cnt = 0, drop_cnt = 0, byte_cnt = 0;
	struct ntmp_priv *ntmp = &priv->ntmp;
	unsigned long cookie = f->cookie;
	struct netc_flower_rule *rule;
	int err;

	guard(mutex)(&ntmp->flower_lock);
	rule = netc_find_flower_rule_by_cookie(ntmp, port->index, cookie);
	if (!rule) {
		NL_SET_ERR_MSG_MOD(extack, "Cannot find the rule");
		return -EINVAL;
	}

	switch (rule->flower_type) {
	case FLOWER_TYPE_PSFP:
		err = netc_psfp_flower_stat(ntmp, rule, &byte_cnt,
					    &pkt_cnt, &drop_cnt);
		if (err)
			goto err_out;
		break;
	case FLOWER_TYPE_TRAP:
	case FLOWER_TYPE_REDIRECT:
		err = netc_trap_redirect_flower_stat(ntmp, rule, &byte_cnt,
						     &pkt_cnt, &drop_cnt);
		if (err)
			goto err_out;
		break;
	case FLOWER_TYPE_POLICE:
		err = netc_police_flower_stat(ntmp, rule, &pkt_cnt);
		if (err)
			goto err_out;
		break;
	default:
		NL_SET_ERR_MSG_MOD(extack, "Unknown flower type");
		return -EINVAL;
	}

	flow_stats_update(&f->stats, byte_cnt, pkt_cnt, drop_cnt,
			  rule->lastused, FLOW_ACTION_HW_STATS_IMMEDIATE);
	rule->lastused = jiffies;

	return 0;

err_out:
	NL_SET_ERR_MSG_MOD(extack, "Failed to get statistics");

	return err;
}

void netc_destroy_flower_list(struct netc_switch *priv)
{
	struct ntmp_priv *ntmp = &priv->ntmp;
	struct netc_flower_rule *rule;
	struct hlist_node *tmp;

	guard(mutex)(&ntmp->flower_lock);
	hlist_for_each_entry_safe(rule, tmp, &ntmp->flower_list, node)
		netc_delete_flower_rule(ntmp, rule);
}
