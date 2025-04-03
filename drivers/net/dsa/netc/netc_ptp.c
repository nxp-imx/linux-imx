// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC switch driver
 * Copyright 2024 NXP
 */

#include <linux/ptp_classify.h>
#include "netc_switch.h"

#define NETC_TS_REQ_ID_NUM		(NETC_MAX_TS_REQ_ID + 1)
#define NETC_PTP_TX_TSTAMP_TIMEOUT	(5 * HZ)

int netc_get_ts_info(struct dsa_switch *ds, int port_id,
		     struct kernel_ethtool_ts_info *info)
{
	struct netc_switch *priv = NETC_PRIV(ds);
	u32 devfn = priv->info->tmr_devfn;
	u32 bus = priv->pdev->bus->number;
	struct pci_dev *tmr_pdev;
	int domain;

	domain = pci_domain_nr(priv->pdev->bus);
	tmr_pdev = pci_get_domain_bus_and_slot(domain, bus, devfn);
	info->phc_index = netc_timer_get_phc_index(tmr_pdev);

	info->so_timestamping |= SOF_TIMESTAMPING_TX_SOFTWARE |
				 SOF_TIMESTAMPING_RX_SOFTWARE |
				 SOF_TIMESTAMPING_SOFTWARE |
				 SOF_TIMESTAMPING_TX_HARDWARE |
				 SOF_TIMESTAMPING_RX_HARDWARE |
				 SOF_TIMESTAMPING_RAW_HARDWARE;

	info->tx_types = BIT(HWTSTAMP_TX_OFF) | BIT(HWTSTAMP_TX_ON) |
			 BIT(HWTSTAMP_TX_ONESTEP_SYNC);

	info->rx_filters = BIT(HWTSTAMP_FILTER_NONE) |
			   BIT(HWTSTAMP_FILTER_PTP_V2_EVENT) |
			   BIT(HWTSTAMP_FILTER_PTP_V2_L2_EVENT) |
			   BIT(HWTSTAMP_FILTER_PTP_V2_L4_EVENT);

	return 0;
}

static void netc_port_del_ptp_filter(struct netc_port *port)
{
	struct netc_switch *priv = port->switch_priv;
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	u32 entry_id;
	int i;

	for (i = 0; i < NETC_PTP_MAX; i++) {
		entry_id = port->ptp_ipft_eid[i];
		if (entry_id != NTMP_NULL_ENTRY_ID) {
			ntmp_ipft_delete_entry(cbdrs, entry_id);
			port->ptp_ipft_eid[i] = NTMP_NULL_ENTRY_ID;
		}
	}
}

static int netc_port_ipft_ptp_keye_construct(struct ipft_keye_data *keye,
					     int port, enum netc_ptp_type type)
{
	u16 src_port, frm_attr_flags;

	src_port = FIELD_PREP(IPFT_SRC_PORT, port);
	src_port |= IPFT_SRC_PORT_MASK;
	keye->src_port = cpu_to_le16(src_port);

	switch (type) {
	case NETC_PTP_L2:
		keye->ethertype = htons(ETH_P_1588);
		keye->ethertype_mask = htons(0xffff);
		break;
	case NETC_PTP_L4_IPV4_EVENT:
	case NETC_PTP_L4_IPV4_GENERAL:
	case NETC_PTP_L4_IPV6_EVENT:
	case NETC_PTP_L4_IPV6_GENERAL:
		frm_attr_flags = IPFT_FAF_IP_HDR | FIELD_PREP(IPFT_FAF_L4_CODE,
				 IPFT_FAF_UDP_HDR);
		if (type == NETC_PTP_L4_IPV6_EVENT ||
		    type == NETC_PTP_L4_IPV6_GENERAL)
			frm_attr_flags |= IPFT_FAF_IP_VER6;

		keye->frm_attr_flags = cpu_to_le16(frm_attr_flags);
		keye->frm_attr_flags_mask = keye->frm_attr_flags;
		keye->ip_protocol = IPPROTO_UDP;
		keye->ip_protocol_mask = 0xff;

		if (type == NETC_PTP_L4_IPV4_EVENT ||
		    type == NETC_PTP_L4_IPV6_EVENT)
			keye->l4_dst_port = htons(PTP_EV_PORT);
		else
			keye->l4_dst_port = htons(PTP_GEN_PORT);

		keye->l4_dst_port_mask = htons(0xffff);
		break;
	default:
		return -ERANGE;
	}

	return 0;
}

static int netc_port_add_ipft_ptp_entry(struct netc_port *port,
					enum netc_ptp_type type)
{
	struct ntmp_ipft_entry *ipft_entry __free(kfree);
	struct netc_switch *priv = port->switch_priv;
	struct netc_cbdrs *cbdrs = &priv->ntmp.cbdrs;
	struct ipft_keye_data *ipft_keye;
	u32 cfg;
	int err;

	ipft_entry = kzalloc(sizeof(*ipft_entry), GFP_KERNEL);
	if (!ipft_entry)
		return -ENOMEM;

	ipft_keye = &ipft_entry->keye;
	err = netc_port_ipft_ptp_keye_construct(ipft_keye, port->index, type);
	if (err)
		return err;

	cfg = FIELD_PREP(IPFT_FLTFA, IPFT_FLTFA_REDIRECT);
	cfg |= FIELD_PREP(IPFT_HR, NETC_HR_TRAP);
	cfg |= IPFT_TIMECAPE;
	cfg |= IPFT_RRT;
	ipft_entry->cfge.cfg = cpu_to_le32(cfg);

	err = ntmp_ipft_add_entry(cbdrs, &ipft_entry->entry_id, ipft_entry);
	if (err)
		return err;

	port->ptp_ipft_eid[type] = ipft_entry->entry_id;

	return 0;
}

static int netc_port_add_l2_ptp_filter(struct netc_port *port)
{
	return netc_port_add_ipft_ptp_entry(port, NETC_PTP_L2);
}

static int netc_port_add_l4_ptp_filter(struct netc_port *port)
{
	int err;

	err = netc_port_add_ipft_ptp_entry(port, NETC_PTP_L4_IPV4_EVENT);
	if (err)
		return err;

	err = netc_port_add_ipft_ptp_entry(port, NETC_PTP_L4_IPV4_GENERAL);
	if (err)
		goto del_ptp_filter;

	err = netc_port_add_ipft_ptp_entry(port, NETC_PTP_L4_IPV6_EVENT);
	if (err)
		goto del_ptp_filter;

	err = netc_port_add_ipft_ptp_entry(port, NETC_PTP_L4_IPV6_GENERAL);
	if (err)
		goto del_ptp_filter;

	return 0;

del_ptp_filter:
	netc_port_del_ptp_filter(port);

	return err;
}

static int netc_port_add_l2_l4_ptp_filter(struct netc_port *port)
{
	int err;

	err = netc_port_add_l2_ptp_filter(port);
	if (err)
		return err;

	err = netc_port_add_l4_ptp_filter(port);
	if (err)
		netc_port_del_ptp_filter(port);

	return err;
}

static int netc_port_set_ptp_filter(struct netc_port *port, int ptp_filter)
{
	int err;

	if (port->ptp_filter == ptp_filter)
		return 0;

	switch (ptp_filter) {
	case HWTSTAMP_FILTER_NONE:
		netc_port_del_ptp_filter(port);
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
		err = netc_port_add_l2_ptp_filter(port);
		if (err)
			return err;

		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
		err = netc_port_add_l4_ptp_filter(port);
		if (err)
			return err;

		break;
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
		err = netc_port_add_l2_l4_ptp_filter(port);
		if (err)
			return err;

		break;
	default:
		return -ERANGE;
	}

	port->ptp_filter = ptp_filter;

	return 0;
}

int netc_port_hwtstamp_set(struct dsa_switch *ds, int port_id,
			   struct ifreq *ifr)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	struct hwtstamp_config config;
	int ptp_filter, err;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	switch (config.tx_type) {
	case HWTSTAMP_TX_ON:
		port->offloads |= NETC_FLAG_TX_TSTAMP;
		break;
	case HWTSTAMP_TX_ONESTEP_SYNC:
		port->offloads |= NETC_FLAG_TX_ONESTEP_SYNC;
		break;
	case HWTSTAMP_TX_OFF:
		port->offloads &= !(NETC_FLAG_TX_TSTAMP |
				    NETC_FLAG_TX_ONESTEP_SYNC);
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		ptp_filter = HWTSTAMP_FILTER_NONE;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		ptp_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		ptp_filter = HWTSTAMP_FILTER_PTP_V2_L2_EVENT;
		break;
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		ptp_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		break;
	default:
		return -ERANGE;
	}

	err = netc_port_set_ptp_filter(port, ptp_filter);
	if (err)
		return err;

	config.rx_filter = ptp_filter;

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ? -EFAULT : 0;
}

int netc_port_hwtstamp_get(struct dsa_switch *ds, int port_id,
			   struct ifreq *ifr)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	struct hwtstamp_config config;

	config.flags = 0;

	if (port->offloads & NETC_FLAG_TX_ONESTEP_SYNC)
		config.tx_type = HWTSTAMP_TX_ONESTEP_SYNC;
	else if (port->offloads & NETC_FLAG_TX_TSTAMP)
		config.tx_type = HWTSTAMP_TX_ON;
	else
		config.tx_type = HWTSTAMP_TX_OFF;

	config.rx_filter = port->ptp_filter;

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ? -EFAULT : 0;
}

static void netc_port_set_onestep_control(struct netc_port *port,
					  bool udp, int offset)
{
	u32 val = PM_SINGLE_STEP_EN;

	val |= offset << 7 & PM_SINGLE_STEP_OFFSET;
	val = u32_replace_bits(val, udp ? 1 : 0, PM_SINGLE_STEP_CH);
	netc_mac_port_wr(port, NETC_PM_SINGLE_STEP(0), val);
}

static int netc_port_txtstamp_onestep_sync(struct dsa_switch *ds, int port_id,
					   struct sk_buff *skb,
					   unsigned int ptp_class)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	u16 correction_offset, timestamp_offset;
	struct netc_switch *priv = ds->priv;
	struct ptp_header *ptp_hdr;
	struct pci_dev *tmr_dev;
	unsigned int pkt_type;
	u8 msg_type, twostep;
	bool is_udp;
	u8 *pkt_hdr;
	u64 ts, sec;
	u32 ns;

	ptp_hdr = ptp_parse_header(skb, ptp_class);
	if (!ptp_hdr)
		return -EINVAL;

	msg_type = ptp_get_msgtype(ptp_hdr, ptp_class);
	twostep = ptp_hdr->flag_field[0] & 0x2;

	if (msg_type != PTP_MSGTYPE_SYNC || twostep != 0)
		return -EINVAL;

	pkt_type = ptp_class & PTP_CLASS_PMASK;
	if (pkt_type == PTP_CLASS_IPV4 || pkt_type == PTP_CLASS_IPV6)
		is_udp = true;
	else
		is_udp = false;

	pkt_hdr = skb_mac_header(skb);
	correction_offset = (u8 *)&ptp_hdr->correction - pkt_hdr;
	timestamp_offset = (u8 *)ptp_hdr + sizeof(*ptp_hdr) - pkt_hdr;

	tmr_dev = netc_switch_get_timer(priv);
	ts = netc_timer_get_current_time(tmr_dev);
	if (!ts)
		return -EINVAL;

	NETC_SKB_CB(skb)->tstamp = ts;
	NETC_SKB_CB(skb)->ptp_flag = NETC_PTP_FLAG_ONESTEP;

	/* Update originTimestamp field of Sync packet
	 * - 48 bits seconds field
	 * - 32 bits nanseconds field
	 */
	sec = div_u64_rem(ts, NSEC_PER_SEC, &ns);
	*(__be16 *)(pkt_hdr + timestamp_offset) = htons((sec >> 32) & 0xffff);
	*(__be32 *)(pkt_hdr + timestamp_offset + 2) = htonl(sec & 0xffffffff);
	*(__be32 *)(pkt_hdr + timestamp_offset + 6) = htonl(ns);

	netc_port_set_onestep_control(port, is_udp, correction_offset);

	return 0;
}

static int netc_port_txtstamp_twostep(struct netc_port *port,
				      struct sk_buff *clone)
{
	DECLARE_BITMAP(ts_req_id_bitmap, NETC_TS_REQ_ID_NUM);
	struct netc_switch *priv = port->switch_priv;
	struct sk_buff *skb, *skb_tmp;
	unsigned long ts_req_id;

	spin_lock(&port->ts_req_id_lock);

	skb_queue_walk_safe(&port->skb_txtstamp_queue, skb, skb_tmp) {
		if (time_before(NETC_SKB_CB(skb)->ptp_tx_time +
				NETC_PTP_TX_TSTAMP_TIMEOUT, jiffies)) {
			dev_dbg_ratelimited(priv->dev,
					    "port %d ts_req_id %u which seems lost\n",
					    port->index, NETC_SKB_CB(skb)->ts_req_id);

			__skb_unlink(skb, &port->skb_txtstamp_queue);
			kfree_skb(skb);
		} else {
			__set_bit(NETC_SKB_CB(skb)->ts_req_id, ts_req_id_bitmap);
		}
	}

	ts_req_id = find_first_zero_bit(ts_req_id_bitmap, NETC_TS_REQ_ID_NUM);
	if (ts_req_id == NETC_TS_REQ_ID_NUM) {
		spin_unlock(&port->ts_req_id_lock);
		return -EBUSY;
	}

	NETC_SKB_CB(clone)->ts_req_id = ts_req_id;
	NETC_SKB_CB(clone)->ptp_tx_time = jiffies;
	skb_shinfo(clone)->tx_flags |= SKBTX_IN_PROGRESS;
	skb_queue_tail(&port->skb_txtstamp_queue, clone);

	spin_unlock(&port->ts_req_id_lock);

	return 0;
}

bool netc_port_rxtstamp(struct dsa_switch *ds, int port,
			struct sk_buff *skb, unsigned int type)
{
	struct skb_shared_hwtstamps *hwtstamps = skb_hwtstamps(skb);
	u64 ts = NETC_SKB_CB(skb)->tstamp;

	hwtstamps->hwtstamp = ns_to_ktime(ts);

	return false;
}

void netc_port_txtstamp(struct dsa_switch *ds, int port_id,
			struct sk_buff *skb)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	unsigned int ptp_class;
	bool twostep = false;

	ptp_class = ptp_classify_raw(skb);
	if (ptp_class == PTP_CLASS_NONE)
		return;

	if (port->offloads & NETC_FLAG_TX_ONESTEP_SYNC) {
		if (!netc_port_txtstamp_onestep_sync(ds, port_id, skb, ptp_class))
			return;

		/* Fall back to two-step timestamping */
		twostep = true;
	}

	if (port->offloads & NETC_FLAG_TX_TSTAMP || twostep) {
		struct sk_buff *clone = skb_clone_sk(skb);

		if (unlikely(!clone))
			return;

		if (netc_port_txtstamp_twostep(port, clone)) {
			kfree_skb(clone);
			return;
		}

		NETC_SKB_CB(skb)->clone = clone;
		NETC_SKB_CB(skb)->ptp_flag = NETC_PTP_FLAG_TWOSTEP;
	}
}
