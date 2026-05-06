// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2025 NXP
 */

#include <linux/fsl/netc_lib.h>
#include <net/pkt_cls.h>

#include "enetc_pf_common.h"
#include "enetc4_tc.h"

#define ENETC_GENERIC_KEYS	(BIT_ULL(FLOW_DISSECTOR_KEY_ETH_ADDRS) | \
				 BIT_ULL(FLOW_DISSECTOR_KEY_VLAN) | \
				 BIT_ULL(FLOW_DISSECTOR_KEY_CVLAN) | \
				 BIT_ULL(FLOW_DISSECTOR_KEY_BASIC) | \
				 BIT_ULL(FLOW_DISSECTOR_KEY_IPV4_ADDRS) | \
				 BIT_ULL(FLOW_DISSECTOR_KEY_IPV6_ADDRS) | \
				 BIT_ULL(FLOW_DISSECTOR_KEY_PORTS))

static LIST_HEAD(enetc4_block_cb_list);

static const struct netc_flower enetc4_flower[] = {
	{
		BIT_ULL(FLOW_ACTION_GATE),
		BIT_ULL(FLOW_ACTION_POLICE),
		BIT_ULL(FLOW_DISSECTOR_KEY_ETH_ADDRS) |
		BIT_ULL(FLOW_DISSECTOR_KEY_VLAN),
		FLOWER_TYPE_PSFP
	},
	{
		BIT_ULL(FLOW_ACTION_POLICE),
		0,
		ENETC_GENERIC_KEYS,
		FLOWER_TYPE_POLICE
	},
	{
		BIT_ULL(FLOW_ACTION_DROP),
		0,
		ENETC_GENERIC_KEYS,
		FLOWER_TYPE_DROP
	},
};

static bool enetc4_tc_cbs_is_enable(struct enetc_hw *hw, int tc)
{
	u32 val;

	val = enetc_port_rd(hw, ENETC4_PTCCBSR0(tc));
	if (val & PTCCBSR0_CBSE)
		return true;

	return false;
}

static void enetc4_set_ptccbsr(struct enetc_hw *hw, int tc,
			       bool en, u32 bw, u32 hi_credit)
{
	if (en) {
		u32 val = PTCCBSR0_CBSE;

		val |= (bw / 10) & PTCCBSR0_BW;
		val |= (bw % 10) << 16;

		enetc_port_wr(hw, ENETC4_PTCCBSR1(tc), hi_credit);
		enetc_port_wr(hw, ENETC4_PTCCBSR0(tc), val);

	} else {
		enetc_port_wr(hw, ENETC4_PTCCBSR1(tc), 0);
		enetc_port_wr(hw, ENETC4_PTCCBSR0(tc), 0);
	}
}

static u32 enetc4_get_tc_cbs_bw(struct enetc_hw *hw, int tc)
{
	u32 val, bw;

	val = enetc_port_rd(hw, ENETC4_PTCCBSR0(tc));
	bw = (val & PTCCBSR0_BW) * 10 + PTCCBSR0_FRACT_GET(val);

	return bw;
}

static u32 enetc4_get_tc_msdu(struct enetc_hw *hw, int tc)
{
	return enetc_port_rd(hw, ENETC4_PTCTMSDUR(tc)) & PTCTMSDUR_MAXSDU;
}

static int enetc4_setup_tc_cbs(struct net_device *ndev, void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct tc_cbs_qopt_offload *cbs = type_data;
	u32 port_transmit_rate = priv->speed;
	u8 tc_nums = netdev_get_num_tc(ndev);
	u64 sysclk_freq = priv->sysclk_freq;
	u32 hi_credit_bit, hi_credit_reg;
	u8 high_prio_tc, second_prio_tc;
	struct enetc_si *si = priv->si;
	struct enetc_hw *hw = &si->hw;
	u32 max_interference_size;
	u32 port_frame_max_size;
	u32 bw, bw_sum;
	u8 tc;

	high_prio_tc = tc_nums - 1;
	second_prio_tc = tc_nums - 2;

	tc = netdev_txq_to_tc(ndev, cbs->queue);

	/* Support highest prio and second prio tc in cbs mode */
	if (tc != high_prio_tc && tc != second_prio_tc)
		return -EOPNOTSUPP;

	if (!cbs->enable) {
		/* Make sure the other TC that are numerically
		 * lower than this TC have been disabled.
		 */
		if (tc == high_prio_tc &&
		    enetc4_tc_cbs_is_enable(hw, second_prio_tc)) {
			dev_err(&ndev->dev,
				"Disable TC%d before disable TC%d\n",
				second_prio_tc, tc);
			return -EINVAL;
		}

		enetc4_set_ptccbsr(hw, tc, false, 0, 0);

		return 0;
	}

	/* The unit of idleslope and sendslope is kbps. And the sendslope should be
	 * a negative number, it can be calculated as follows, IEEE 802.1Q-2014
	 * Section 8.6.8.2 item g):
	 * sendslope = idleslope - port_transmit_rate
	 */
	if (cbs->idleslope - cbs->sendslope != port_transmit_rate * 1000L ||
	    cbs->idleslope < 0 || cbs->sendslope > 0)
		return -EOPNOTSUPP;

	port_frame_max_size = ndev->mtu + VLAN_ETH_HLEN + ETH_FCS_LEN;

	/* The unit of port_transmit_rate is Mbps, the unit of bw is 1/1000 */
	bw = cbs->idleslope / port_transmit_rate;
	bw_sum = bw;

	/* Make sure the credit-based shaper of highest priority TC has been enabled
	 * before the secondary priority TC.
	 */
	if (tc == second_prio_tc) {
		if (!enetc4_tc_cbs_is_enable(hw, high_prio_tc)) {
			dev_err(&ndev->dev,
				"Enable TC%d first before enable TC%d\n",
				high_prio_tc, second_prio_tc);
			return -EINVAL;
		}
		bw_sum += enetc4_get_tc_cbs_bw(hw, high_prio_tc);
	}

	if (bw_sum >= 1000) {
		dev_err(&ndev->dev,
			"The sum of all CBS Bandwidth can't exceed 1000\n");
		return -EINVAL;
	}

	/* For the AVB Class A (highest priority TC), the max_interfrence_size is
	 * maximum sized frame for the port.
	 * For the AVB Class B (second highest priority TC), the max_interfrence_size
	 * is calculated as below:
	 *
	 *      max_interference_size = (Ra * M0) / (R0 - Ra) + MA + M0
	 *
	 *	- RA: idleSlope for AVB Class A
	 *	- R0: port transmit rate
	 *	- M0: maximum sized frame for the port
	 *	- MA: maximum sized frame for AVB Class A
	 */

	if (tc == high_prio_tc) {
		max_interference_size = port_frame_max_size * 8;
	} else {
		u32 m0, ma;
		u64 ra, r0;

		m0 = port_frame_max_size * 8;
		ma = enetc4_get_tc_msdu(hw, high_prio_tc) * 8;
		ra = enetc4_get_tc_cbs_bw(hw, high_prio_tc) *
		     port_transmit_rate * 1000ULL;
		r0 = port_transmit_rate * 1000000ULL;
		max_interference_size = m0 + ma + (u32)div_u64(ra * m0, r0 - ra);
	}

	/* hiCredit bits calculate by:
	 *
	 * max_interference_size * (idleslope / port_transmit_rate)
	 */
	hi_credit_bit = max_interference_size * bw / 1000;

	/* Number of credits per bit is calculated as follows:
	 *
	 * (enetClockFrequency / port_transmit_rate) * 100
	 */
	hi_credit_reg = (u32)div_u64(sysclk_freq * 1000ULL * hi_credit_bit,
				     port_transmit_rate * 1000000ULL);

	enetc4_set_ptccbsr(hw, tc, true, bw, hi_credit_reg);

	return 0;
}

static int enetc4_devfn_to_port(struct pci_dev *pdev)
{
	switch (pdev->devfn) {
	case 0:
		return 0;
	case 64:
		return 1;
	case 128:
		return 2;
	default:
		return -1;
	}
}

int enetc4_pf_to_port(struct enetc_si *si)
{
	switch (si->revision) {
	case ENETC_REV_4_1:
	case ENETC_REV_4_3:
	case ENETC_REV_4_6:
		return enetc4_devfn_to_port(si->pdev);
	default:
		return -1;
	}
}

static void enetc4_pf_set_tc_msdu(struct enetc_hw *hw, u32 *max_sdu)
{
	int tc;

	for (tc = 0; tc < 8; tc++) {
		u32 val = ENETC_MAC_MAXFRM_SIZE;

		if (max_sdu[tc])
			val = max_sdu[tc] + VLAN_ETH_HLEN;

		val = u32_replace_bits(val, SDU_TYPE_MPDU, PTCTMSDUR_SDU_TYPE);
		enetc_port_wr(hw, ENETC4_PTCTMSDUR(tc), val);
	}
}

void enetc4_pf_reset_tc_msdu(struct enetc_hw *hw)
{
	u32 val = ENETC_MAC_MAXFRM_SIZE;
	int tc;

	val = u32_replace_bits(val, SDU_TYPE_MPDU, PTCTMSDUR_SDU_TYPE);

	for (tc = 0; tc < ENETC_NUM_TC; tc++)
		enetc_port_wr(hw, ENETC4_PTCTMSDUR(tc), val);
}

static void enetc4_pf_set_time_gating(struct enetc_hw *hw, bool en)
{
	u32 old_val, val;

	old_val = enetc_port_rd(hw, ENETC4_PTGSCR);
	val = u32_replace_bits(old_val, en ? 1 : 0, PTGSCR_TGE);
	if (val != old_val)
		enetc_port_wr(hw, ENETC4_PTGSCR, val);
}

static int enetc4_taprio_replace(struct net_device *ndev,
				 struct tc_taprio_qopt_offload *offload)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	struct enetc_hw *hw = &si->hw;
	int port, err;

	port = enetc4_pf_to_port(si);
	if (port < 0)
		return -EINVAL;

	err = enetc_setup_tc_mqprio(ndev, &offload->mqprio);
	if (err)
		return err;

	/* Set the maximum frame size for each traffic class */
	enetc4_pf_set_tc_msdu(hw, offload->max_sdu);
	enetc4_pf_set_time_gating(hw, true);

	err = netc_setup_taprio(&si->ntmp_user, port, offload);
	if (err)
		goto err_setup_taprio;

	priv->active_offloads |= ENETC_F_QBV;

	return 0;

err_setup_taprio:
	enetc4_pf_set_time_gating(hw, false);
	enetc4_pf_reset_tc_msdu(hw);
	enetc_reset_tc_mqprio(ndev);

	return err;
}

static void enetc4_taprio_destroy(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_hw *hw = &priv->si->hw;

	enetc4_pf_set_time_gating(hw, false);
	enetc4_pf_reset_tc_msdu(hw);
	enetc_reset_tc_mqprio(ndev);
	enetc_reset_taprio_stats(priv);
	priv->active_offloads &= ~ENETC_F_QBV;
}

static int enetc4_setup_tc_taprio(struct net_device *ndev, void *type_data)
{
	struct tc_taprio_qopt_offload *offload = type_data;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	int err = 0;

	if (!(si->hw_features & ENETC_SI_F_QBV))
		return -EOPNOTSUPP;

	switch (offload->cmd) {
	case TAPRIO_CMD_REPLACE:
		err = enetc4_taprio_replace(ndev, offload);
		break;
	case TAPRIO_CMD_DESTROY:
		enetc4_taprio_destroy(ndev);
		break;
	case TAPRIO_CMD_STATS:
		enetc_taprio_stats(ndev, &offload->stats);
		break;
	case TAPRIO_CMD_QUEUE_STATS:
		enetc_taprio_queue_stats(ndev, &offload->queue_stats);
		break;
	default:
		err = -EOPNOTSUPP;
	}

	return err;
}

static void enetc4_pf_set_tc_tsd(struct enetc_hw *hw, int tc, bool en)
{
	enetc_port_wr(hw, ENETC4_PTCTSDR(tc), en ? PTCTSDR_TSDE : 0);
}

static int enetc4_setup_tc_txtime(struct net_device *ndev, void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct tc_etf_qopt_offload *qopt = type_data;
	u8 tc_nums = netdev_get_num_tc(ndev);
	struct enetc_hw *hw = &priv->si->hw;
	int i, tc;

	if (!tc_nums)
		return -EOPNOTSUPP;

	if (qopt->queue < 0 || qopt->queue >= ndev->real_num_tx_queues)
		return -EINVAL;

	tc = netdev_txq_to_tc(ndev, qopt->queue);
	if (tc < 0 || tc >= tc_nums)
		return -EINVAL;

	/* Accordiing to the NETC block guide, all traffic on the traffic class
	 * should use time specific departure operation.
	 */
	for (i = 0; i < ndev->tc_to_txq[tc].count; i++) {
		u16 offset = ndev->tc_to_txq[tc].offset + i;

		priv->tx_ring[offset]->tsd_enable = qopt->enable;
	}

	enetc4_pf_set_tc_tsd(hw, tc, qopt->enable);

	return 0;
}

static const struct netc_flower *enetc4_parse_tc_flower(u64 actions, u64 keys)
{
	u64 key_acts, all_acts;
	int i;

	for (i = 0; i < ARRAY_SIZE(enetc4_flower); i++) {
		key_acts = enetc4_flower[i].key_acts;
		all_acts = enetc4_flower[i].key_acts |
			   enetc4_flower[i].opt_acts;

		/* key_acts must be matched */
		if ((actions & key_acts) == key_acts &&
		    (actions & all_acts) == actions &&
		    keys & enetc4_flower[i].keys)
			return &enetc4_flower[i];
	}

	return NULL;
}

static int enetc4_config_cls_flower(struct enetc_ndev_priv *priv,
				    struct flow_cls_offload *f)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(f);
	struct netlink_ext_ack *extack = f->common.extack;
	struct flow_action *action = &rule->action;
	struct flow_dissector *dissector;
	const struct netc_flower *flower;
	struct flow_action_entry *entry;
	struct enetc_si *si = priv->si;
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

	flower = enetc4_parse_tc_flower(actions, dissector->used_keys);
	if (!flower) {
		NL_SET_ERR_MSG_MOD(extack, "Unsupported actions or keys");
		return -EOPNOTSUPP;
	}

	switch (flower->type) {
	case FLOWER_TYPE_PSFP:
		if (!(si->hw_features & ENETC_SI_F_PSFP))
			return -EOPNOTSUPP;

		return netc_setup_psfp(&si->ntmp_user, 0, f);
	case FLOWER_TYPE_POLICE:
		return netc_setup_police(&si->ntmp_user, -1, f);
	case FLOWER_TYPE_DROP:
		return netc_setup_drop(&si->ntmp_user, -1, f);
	default:
		NL_SET_ERR_MSG_MOD(extack, "Unsupported flower type");
		return -EOPNOTSUPP;
	}
}

static void enetc4_destroy_flower_rule(struct ntmp_user *user,
				       struct netc_flower_rule *rule)
{
	switch (rule->flower_type) {
	case FLOWER_TYPE_PSFP:
		netc_delete_psfp_flower_rule(user, rule);
		break;
	case FLOWER_TYPE_POLICE:
		netc_delete_police_flower_rule(user, rule);
		break;
	case FLOWER_TYPE_DROP:
		netc_delete_drop_flower_rule(user, rule);
		break;
	default:
		break;
	}
}

void enetc4_clear_flower_list(struct enetc_si *si)
{
	struct ntmp_user *user = &si->ntmp_user;
	struct netc_flower_rule *rule;
	struct hlist_node *tmp;

	mutex_lock(&user->flower_lock);

	hlist_for_each_entry_safe(rule, tmp, &user->flower_list, node)
		enetc4_destroy_flower_rule(user, rule);

	mutex_unlock(&user->flower_lock);
}

static int enetc4_destroy_cls_flower(struct enetc_ndev_priv *priv,
				     struct flow_cls_offload *f)
{
	struct netlink_ext_ack *extack = f->common.extack;
	struct ntmp_user *user = &priv->si->ntmp_user;
	unsigned long cookie = f->cookie;
	struct netc_flower_rule *rule;
	int err = 0;

	mutex_lock(&user->flower_lock);
	rule = netc_find_flower_rule_by_cookie(user, 0, cookie);
	if (!rule) {
		NL_SET_ERR_MSG_MOD(extack, "Cannot find the rule");
		err = -EINVAL;
		goto unlock_flower;
	}

	enetc4_destroy_flower_rule(user, rule);

unlock_flower:
	mutex_unlock(&user->flower_lock);

	return err;
}

static int enetc4_get_cls_flower_stats(struct enetc_ndev_priv *priv,
				       struct flow_cls_offload *f)
{
	struct netlink_ext_ack *extack = f->common.extack;
	struct ntmp_user *user = &priv->si->ntmp_user;
	unsigned long cookie = f->cookie;
	struct netc_flower_rule *rule;
	u64 pkt_cnt, drop_cnt = 0;
	u64 byte_cnt = 0;
	int err;

	mutex_lock(&user->flower_lock);
	rule = netc_find_flower_rule_by_cookie(user, 0, cookie);
	if (!rule) {
		NL_SET_ERR_MSG_MOD(extack, "Cannot find the rule");
		err = -EINVAL;
		goto unlock_flower;
	}

	switch (rule->flower_type) {
	case FLOWER_TYPE_PSFP:
		err = netc_psfp_flower_stat(user, rule, &byte_cnt,
					    &pkt_cnt, &drop_cnt);
		if (err) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Failed to get statistics of PSFP");
			goto unlock_flower;
		}
		break;
	case FLOWER_TYPE_POLICE:
	case FLOWER_TYPE_DROP:
		err = netc_ipft_flower_stat(user, rule, &pkt_cnt);
		if (err)
			goto unlock_flower;
		break;
	default:
		NL_SET_ERR_MSG_MOD(extack, "Unknown flower type");
		err = -EINVAL;
		goto unlock_flower;
	}

	flow_stats_update(&f->stats, byte_cnt, pkt_cnt, drop_cnt,
			  rule->lastused, FLOW_ACTION_HW_STATS_IMMEDIATE);
	rule->lastused = jiffies;

unlock_flower:
	mutex_unlock(&user->flower_lock);

	return err;
}

static int enetc4_setup_tc_cls_flower(struct enetc_ndev_priv *priv,
				      struct flow_cls_offload *f)
{
	switch (f->command) {
	case FLOW_CLS_REPLACE:
		return enetc4_config_cls_flower(priv, f);
	case FLOW_CLS_DESTROY:
		return enetc4_destroy_cls_flower(priv, f);
	case FLOW_CLS_STATS:
		return enetc4_get_cls_flower_stats(priv, f);
	default:
		return -EOPNOTSUPP;
	}
}

static int enetc4_setup_tc_block_cb(enum tc_setup_type type, void *type_data,
				    void *cb_priv)
{
	struct net_device *ndev = cb_priv;
	struct enetc_ndev_priv *priv;

	if (!tc_can_offload(ndev))
		return -EOPNOTSUPP;

	priv = netdev_priv(ndev);
	switch (type) {
	case TC_SETUP_CLSFLOWER:
		return enetc4_setup_tc_cls_flower(priv, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

static int enetc4_setup_tc_block(struct net_device *ndev, void *type_data)
{
	struct flow_block_offload *f = type_data;

	return flow_block_cb_setup_simple(f, &enetc4_block_cb_list,
					  enetc4_setup_tc_block_cb,
					  ndev, ndev, true);
}

int enetc4_pf_setup_tc(struct net_device *ndev, enum tc_setup_type type,
		       void *type_data)
{
	switch (type) {
	case TC_QUERY_CAPS:
		return enetc_qos_query_caps(ndev, type_data);
	case TC_SETUP_QDISC_MQPRIO:
		return enetc_setup_tc_mqprio(ndev, type_data);
	case TC_SETUP_QDISC_CBS:
		return enetc4_setup_tc_cbs(ndev, type_data);
	case TC_SETUP_QDISC_TAPRIO:
		return enetc4_setup_tc_taprio(ndev, type_data);
	case TC_SETUP_QDISC_ETF:
		return enetc4_setup_tc_txtime(ndev, type_data);
	case TC_SETUP_BLOCK:
		return enetc4_setup_tc_block(ndev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}
