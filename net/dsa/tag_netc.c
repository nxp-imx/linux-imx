// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 */

#include <linux/dsa/tag_netc.h>

#include "tag.h"

#define NETC_NAME		"nxp_netc"

#define NETC_TAG_FORWARD			0

#define NETC_TAG_TO_PORT			1
#define NETC_TAG_TP_SUBTYPE_NO_TS		0
#define NETC_TAG_TP_SUBTYPE_ONE_STEP		1
#define NETC_TAG_TP_SUBTYPE_TWO_STEP		2

#define NETC_TAG_TO_HOST			2
#define NETC_TAG_TH_SUBTYPE_NO_TS		0
#define NETC_TAG_TH_SUBTYPE_WITH_TS		1
#define NETC_TAG_TH_SUBTYPE_TS_RESP		2

#define NETC_TAG_FWD_SUBTYPE_NORMAL_LEN		6
#define NETC_TAG_TP_SUBTYPE_NO_TS_LEN		6
#define NETC_TAG_TP_SUBTYPE_ONE_STEP_LEN	10
#define NETC_TAG_TP_SUBTYPE_TWO_STEP_LEN	6
#define NETC_TAG_TP_SUBTYPE_BOTH_TS_LEN		10
#define NETC_TAG_TH_SUBTYPE_NO_TS_LEN		6
#define NETC_TAG_TH_SUBTYPE_WITH_TS_LEN		14
#define NETC_TAG_TH_SUBTYPE_TS_RESP_LEN		14

#define NETC_QOS_VALID		1

#define NETC_SWITCH_ETHERTYPE			0xfd3a
#define NETC_SWITCH_ID				1
#define NETC_MAX_TX_TIMESTAMP			0x3fffffff

#pragma pack(1)

/**
 * struct netc_tag_cmn - common tag format of NXP NETC switch tag
 * @tpid: Tag Protocol Identifier
 * @type: The type of the NXP switch tag.
 * @subtype: The subType is used to further distinguish the tag information
 *	     within a particular type of an NXP NETC switch tag.
 * @dr: Drop Resilience (DR) assigned to the frame.
 * @ipv: Internal Priority Value (IPV) assigned to the frame.
 * @qv: Indicates whether the values in the DR and IPV fields are valid.
 * @port: For Forward and To_Host tags, an indication of the switch port number
 *	  where the frame originated. For To_Port tag, it indicates the switch
 *	  port number where the frame is to be transmitted.
 * @sw: For Forward and To_Host tags, an indication of the switch ID where the
 *	frame originated. For To_Port tag, it indicates the switch ID where the
 *	frame is to be transmitted.
 */

struct netc_tag_cmn {
	__be16	tpid;
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u8	subtype:4,
		type:4;
	u8	qv:1,
		resv0:1,
		ipv:3,
		resv1:1,
		dr:2;
	u8	sw:3, // switch ID
		port:5;
#elif defined(__BIG_ENDIAN_BITFIELD)
	u8	type:4,
		subtype:4;
	u8	dr:2,
		resv1:1,
		ipv:3,
		resv0:1,
		qv:1;
	u8	port:5,
		sw:3;
#endif
};

struct netc_tag_tp_subtype0 {
	struct netc_tag_cmn cmn;
	u8 resv;
};

struct netc_tag_tp_subtype1 {
	struct netc_tag_cmn cmn;
	u8 resv;
	__be32 timestamp;
};

struct netc_tag_tp_subtype2 {
	struct netc_tag_cmn cmn;
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u8	ts_req_id:4,
		resv:4;
#elif defined(__BIG_ENDIAN_BITFIELD)
	u8	resv:4,
		ts_req_id:4;
#endif
};

struct netc_tag_th_subtype1 {
	struct netc_tag_cmn cmn;
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u8	resv:4,
		host_reason:4;
#elif defined(__BIG_ENDIAN_BITFIELD)
	u8	host_reason:4,
		resv:4;
#endif
	__be64 timestamp;
};

struct netc_tag_th_subtype2 {
	struct netc_tag_cmn cmn;
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u8	ts_req_id:4,
		host_reason:4;
#elif defined(__BIG_ENDIAN_BITFIELD)
	u8	host_reason,
		ts_req_id:4;
#endif
	__be64 timestamp;
};

#pragma pack()

static void netc_common_tag_config(struct netc_tag_cmn *cmn_tag, u8 type,
				   u8 subtype, u8 port, u8 ipv)
{
	u16 tpid = NETC_SWITCH_ETHERTYPE;
	u8 sw = NETC_SWITCH_ID;
	u8 qv = NETC_QOS_VALID;

	cmn_tag->tpid = htons(tpid);
	cmn_tag->type = type;
	cmn_tag->subtype = subtype;
	cmn_tag->port = port;
	cmn_tag->sw = sw;
	cmn_tag->ipv = ipv;
	cmn_tag->qv = qv;
}

static void *netc_fill_common_tp_tag(struct sk_buff *skb,
				     struct net_device *ndev,
				     u8 subtype, int tag_len)
{
	struct dsa_port *dp = dsa_user_to_port(ndev);
	u16 queue = skb_get_queue_mapping(skb);
	u8 ipv = netdev_txq_to_tc(ndev, queue);
	u8 port = dp->index;
	void *tag;

	skb_push(skb, tag_len);
	dsa_alloc_etype_header(skb, tag_len);

	tag = dsa_etype_header_pos_tx(skb);
	memset(tag, 0, tag_len);
	netc_common_tag_config(tag, NETC_TAG_TO_PORT,
			       subtype, port, ipv);

	return tag;
}

/* To_Port NXP switch tag.
 * SubType=0 - No request to perform timestamping.
 */
static void netc_fill_tp_tag_subtype0(struct sk_buff *skb,
				      struct net_device *ndev)
{
	int tag_len = NETC_TAG_TP_SUBTYPE_NO_TS_LEN;
	u8 subtype = NETC_TAG_TP_SUBTYPE_NO_TS;

	netc_fill_common_tp_tag(skb, ndev, subtype, tag_len);
}

/* To_Port NXP switch tag.
 * SubType=1 - Request to perform one-step timestamping.
 */
static void netc_fill_tp_tag_subtype1(struct sk_buff *skb,
				      struct net_device *ndev)
{
	u32 ts = NETC_SKB_CB(skb)->tstamp & NETC_MAX_TX_TIMESTAMP;
	int tag_len = NETC_TAG_TP_SUBTYPE_ONE_STEP_LEN;
	u8 subtype = NETC_TAG_TP_SUBTYPE_ONE_STEP;
	struct netc_tag_tp_subtype1 *tag;

	tag = netc_fill_common_tp_tag(skb, ndev, subtype, tag_len);
	tag->timestamp = htonl(ts);
}

/* To_Port NXP switch tag.
 * SubType=2 -  Request to perform two-step timestamping.
 */
static void netc_fill_tp_tag_subtype2(struct sk_buff *skb,
				      struct net_device *ndev)
{
	struct sk_buff *clone = NETC_SKB_CB(skb)->clone;
	int tag_len = NETC_TAG_TP_SUBTYPE_TWO_STEP_LEN;
	u8 ts_req_id = NETC_SKB_CB(clone)->ts_req_id;
	u8 subtype = NETC_TAG_TP_SUBTYPE_TWO_STEP;
	struct netc_tag_tp_subtype2 *tag;

	tag = netc_fill_common_tp_tag(skb, ndev, subtype, tag_len);
	tag->ts_req_id = ts_req_id;
}

static struct sk_buff *netc_xmit(struct sk_buff *skb,
				 struct net_device *ndev)
{
	struct sk_buff *clone = NETC_SKB_CB(skb)->clone;
	u8 ptp_flag = NETC_SKB_CB(skb)->ptp_flag;

	if (ptp_flag == NETC_PTP_FLAG_ONESTEP)
		netc_fill_tp_tag_subtype1(skb, ndev);
	else if (ptp_flag == NETC_PTP_FLAG_TWOSTEP && clone)
		netc_fill_tp_tag_subtype2(skb, ndev);
	else
		netc_fill_tp_tag_subtype0(skb, ndev);

	return skb;
}

static void netc_rx_tstamp_process(struct netc_tag_th_subtype1 *tag,
				   struct sk_buff *skb)
{
	u64 ts = __be64_to_cpu(tag->timestamp);

	NETC_SKB_CB(skb)->tstamp = ts;
}

static void netc_twostep_tstamp_process(struct netc_tag_th_subtype2 *tag,
					struct dsa_switch *ds)
{
	struct netc_tagger_data *tagger_data;
	int port = tag->cmn.port;
	u8 ts_req_id;
	u64 ts;

	tagger_data = ds->tagger_data;
	if (!tagger_data->twostep_tstamp_handler)
		return;

	ts = __be64_to_cpu(tag->timestamp);
	ts_req_id = tag->ts_req_id;

	tagger_data->twostep_tstamp_handler(ds, port, ts_req_id, ts);
}

static struct sk_buff *netc_rcv(struct sk_buff *skb,
				struct net_device *ndev)
{
	struct dsa_port *dp = ndev->dsa_ptr;
	struct dsa_switch *ds = dp->ds;
	struct netc_tag_cmn *cmn_tag;
	int tag_len;
	void *tag;

	tag = dsa_etype_header_pos_rx(skb);
	cmn_tag = tag;
	if (ntohs(cmn_tag->tpid) != NETC_SWITCH_ETHERTYPE) {
		netdev_err_once(ndev, "Unknown TPID 0x%04x\n",
				ntohs(cmn_tag->tpid));
		return NULL;
	}

	if (cmn_tag->type == NETC_TAG_FORWARD) {
		tag_len = NETC_TAG_FWD_SUBTYPE_NORMAL_LEN;
	} else if (cmn_tag->type == NETC_TAG_TO_HOST) {
		switch (cmn_tag->subtype) {
		case NETC_TAG_TH_SUBTYPE_NO_TS:
			tag_len = NETC_TAG_TH_SUBTYPE_NO_TS_LEN;
			break;
		case NETC_TAG_TH_SUBTYPE_WITH_TS:
			tag_len = NETC_TAG_TH_SUBTYPE_WITH_TS_LEN;
			netc_rx_tstamp_process(tag, skb);

			break;
		case NETC_TAG_TH_SUBTYPE_TS_RESP:
			netc_twostep_tstamp_process(tag, ds);

			return NULL;
		default:
			netdev_err_once(ndev, "To_Host tag: Unknown subtype %d\n",
					cmn_tag->subtype);
			return NULL;
		}
	} else {
		netdev_err_once(ndev, "Error tag type:%d\n", cmn_tag->type);
		return NULL;
	}

	if (cmn_tag->qv)
		skb->priority = cmn_tag->ipv;

	skb->dev = dsa_conduit_find_user(ndev, 0, cmn_tag->port);
	if (!skb->dev)
		return NULL;

	if (cmn_tag->type == NETC_TAG_FORWARD)
		dsa_default_offload_fwd_mark(skb);

	/* Remove Switch tag from the frame */
	skb_pull_rcsum(skb, tag_len);
	dsa_strip_etype_header(skb, tag_len);

	return skb;
}

static int netc_connect(struct dsa_switch *ds)
{
	struct netc_tagger_data *tagger_data;

	tagger_data = kzalloc(sizeof(*tagger_data), GFP_KERNEL);
	if (!tagger_data)
		return -ENOMEM;

	ds->tagger_data = tagger_data;

	return 0;
}

static void netc_disconnect(struct dsa_switch *ds)
{
	struct netc_tagger_data *tagger_data = ds->tagger_data;

	kfree(tagger_data);
	ds->tagger_data = NULL;
}

static void netc_flow_dissect(const struct sk_buff *skb, __be16 *proto,
			      int *offset)
{
	struct netc_tag_cmn *cmn_tag = (struct netc_tag_cmn *)(skb->data - 2);
	int tag_len;

	if (cmn_tag->type == NETC_TAG_TO_HOST) {
		if (cmn_tag->subtype == NETC_TAG_TH_SUBTYPE_WITH_TS)
			tag_len = NETC_TAG_TH_SUBTYPE_WITH_TS_LEN;
		else if (cmn_tag->subtype == NETC_TAG_TH_SUBTYPE_TS_RESP)
			tag_len = NETC_TAG_TH_SUBTYPE_TS_RESP_LEN;
		else
			tag_len = NETC_TAG_TH_SUBTYPE_NO_TS_LEN;
	} else {
		tag_len = NETC_TAG_FWD_SUBTYPE_NORMAL_LEN;
	}

	*offset = tag_len;
	*proto = ((__be16 *)skb->data)[(tag_len / 2) - 1];
}

static const struct dsa_device_ops netc_netdev_ops = {
	.name			= NETC_NAME,
	.proto			= DSA_TAG_PROTO_NETC,
	.xmit			= netc_xmit,
	.rcv			= netc_rcv,
	.connect		= netc_connect,
	.disconnect		= netc_disconnect,
	.needed_headroom	= NETC_TAG_MAX_LEN,
	.flow_dissect		= netc_flow_dissect,
};

MODULE_ALIAS_DSA_TAG_DRIVER(DSA_TAG_PROTO_NETC, NETC_NAME);
module_dsa_tag_driver(netc_netdev_ops);

MODULE_LICENSE("GPL");
