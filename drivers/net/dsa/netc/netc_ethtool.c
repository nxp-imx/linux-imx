// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC switch driver
 * Copyright 2025 NXP
 */

#include <linux/ethtool_netlink.h>

#include "netc_switch.h"

int netc_port_get_mm(struct dsa_switch *ds, int port_id,
		     struct ethtool_mm_state *state)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	u32 val, rafs, lafs;

	if (!port->caps.pmac)
		return -EOPNOTSUPP;

	guard(mutex)(&port->mm_lock);

	val = netc_port_rd(port, NETC_MAC_MERGE_MMCSR);
	if (MMCSR_GET_ME(val) == MMCSR_ME_FP_1B_BOUNDARY ||
	    MMCSR_GET_ME(val) == MMCSR_ME_FP_4B_BOUNDARY)
		state->pmac_enabled = true;
	else
		state->pmac_enabled = false;

	switch (MMCSR_GET_VSTS(val)) {
	case 0:
		state->verify_status = ETHTOOL_MM_VERIFY_STATUS_DISABLED;
		break;
	case 2:
		state->verify_status = ETHTOOL_MM_VERIFY_STATUS_VERIFYING;
		break;
	case 3:
		state->verify_status = ETHTOOL_MM_VERIFY_STATUS_SUCCEEDED;
		break;
	case 4:
		state->verify_status = ETHTOOL_MM_VERIFY_STATUS_FAILED;
		break;
	case 5:
	default:
		state->verify_status = ETHTOOL_MM_VERIFY_STATUS_UNKNOWN;
		break;
	}

	rafs = MMCSR_GET_RAFS(val);
	state->tx_min_frag_size = ethtool_mm_frag_size_add_to_min(rafs);
	lafs = MMCSR_GET_LAFS(val);
	state->rx_min_frag_size = ethtool_mm_frag_size_add_to_min(lafs);
	state->tx_enabled = !!(val & MAC_MERGE_MMCSR_LPE);
	state->tx_active = state->tx_enabled &&
			   (state->verify_status == ETHTOOL_MM_VERIFY_STATUS_SUCCEEDED ||
			    state->verify_status == ETHTOOL_MM_VERIFY_STATUS_DISABLED);
	state->verify_enabled = !(val & MAC_MERGE_MMCSR_VDIS);
	state->verify_time = MMCSR_GET_VT(val);
	state->max_verify_time = (MAC_MERGE_MMCSR_VT >> 23) - 1;

	return 0;
}

static int netc_port_mm_wait_verify_status(struct netc_port *port, int verify_time)
{
	int timeout = verify_time * USEC_PER_MSEC * NETC_MM_VERIFY_RETRIES;
	u32 val;

	return read_poll_timeout(netc_port_rd, val, MMCSR_GET_VSTS(val) == 3,
				 USEC_PER_MSEC, timeout, true, port,
				 NETC_MAC_MERGE_MMCSR);
}

void netc_port_mm_commit_preemptible_tcs(struct netc_port *port)
{
	int preemptible_tcs = 0, err;
	u32 val;

	val = netc_port_rd(port, NETC_MAC_MERGE_MMCSR);
	if (MMCSR_GET_ME(val) != MMCSR_ME_FP_1B_BOUNDARY &&
	    MMCSR_GET_ME(val) != MMCSR_ME_FP_4B_BOUNDARY)
		goto end;

	if (!(val & MAC_MERGE_MMCSR_VDIS)) {
		err = netc_port_mm_wait_verify_status(port, MMCSR_GET_VT(val));
		if (err)
			goto end;
	}

	preemptible_tcs = port->preemptible_tcs;

end:
	netc_port_wr(port, NETC_PFPCR, preemptible_tcs);
}

static void netc_port_restart_emac_rx(struct netc_port *port)
{
	u32 val;

	val = netc_port_rd(port, NETC_PM_CMD_CFG(0));
	netc_port_wr(port, NETC_PM_CMD_CFG(0), val & ~PM_CMD_CFG_RX_EN);

	if (val & PM_CMD_CFG_RX_EN)
		netc_port_wr(port, NETC_PM_CMD_CFG(0), val);
}

int netc_port_set_mm(struct dsa_switch *ds, int port_id,
		     struct ethtool_mm_cfg *cfg,
		     struct netlink_ext_ack *extack)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	u32 add_frag_size, val;
	int err;

	if (!port->caps.pmac)
		return -EOPNOTSUPP;

	err = ethtool_mm_frag_size_min_to_add(cfg->tx_min_frag_size,
					      &add_frag_size, extack);
	if (err)
		return err;

	guard(mutex)(&port->mm_lock);

	val = netc_port_rd(port, NETC_MAC_MERGE_MMCSR);
	val = u32_replace_bits(val, cfg->verify_enabled ? 0 : 1,
			       MAC_MERGE_MMCSR_VDIS);

	if (cfg->tx_enabled)
		port->offloads |= NETC_FLAG_QBU;
	else
		port->offloads &= ~NETC_FLAG_QBU;

	/* If link is up, enable/disable MAC Merge right away */
	if (!(val & MAC_MERGE_MMCSR_LINK_FAIL)) {
		if (port->offloads & NETC_FLAG_QBU || cfg->pmac_enabled) {
			val = u32_replace_bits(val, MMCSR_ME_FP_4B_BOUNDARY,
					       MAC_MERGE_MMCSR_ME);

			/* When preemption is enabled, generation of PAUSE must
			 * be disabled.
			 */
			netc_port_set_tx_pause(port, false);
		} else {
			netc_port_set_tx_pause(port, port->tx_pause);
			val = u32_replace_bits(val, 0, MAC_MERGE_MMCSR_ME);
		}
	}

	val = u32_replace_bits(val, cfg->verify_time, MAC_MERGE_MMCSR_VT);
	val = u32_replace_bits(val, add_frag_size, MAC_MERGE_MMCSR_RAFS);

	netc_port_wr(port, NETC_MAC_MERGE_MMCSR, val);

	/* TODO: Do we really need to re-enable the Rx of the eMAC? */
	netc_port_restart_emac_rx(port);

	netc_port_mm_commit_preemptible_tcs(port);

	return 0;
}

void netc_port_get_mm_stats(struct dsa_switch *ds, int port_id,
			    struct ethtool_mm_stats *stats)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);

	if (!port->caps.pmac)
		return;

	stats->MACMergeFrameAssErrorCount = netc_port_rd(port, NETC_MAC_MERGE_MMFAECR);
	stats->MACMergeFrameSmdErrorCount = netc_port_rd(port, NETC_MAC_MERGE_MMFSECR);
	stats->MACMergeFrameAssOkCount = netc_port_rd(port, NETC_MAC_MERGE_MMFAOCR);
	stats->MACMergeFragCountRx = netc_port_rd(port, NETC_MAC_MERGE_MMFCRXR);
	stats->MACMergeFragCountTx = netc_port_rd(port, NETC_MAC_MERGE_MMFCTXR);
	stats->MACMergeHoldCount = netc_port_rd(port, NETC_MAC_MERGE_MMHCR);
}

static void netc_port_pause_stats(struct netc_port *port, enum netc_port_mac mac,
				  struct ethtool_pause_stats *pause_stats)
{
	if (mac == NETC_PORT_PMAC && !port->caps.pmac)
		return;

	pause_stats->tx_pause_frames = netc_port_rd64(port, NETC_PM_TXPF(mac));
	pause_stats->rx_pause_frames = netc_port_rd64(port, NETC_PM_RXPF(mac));
}

void netc_port_get_pause_stats(struct dsa_switch *ds, int port_id,
			       struct ethtool_pause_stats *pause_stats)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	struct net_device *ndev;

	switch (pause_stats->src) {
	case ETHTOOL_MAC_STATS_SRC_EMAC:
		netc_port_pause_stats(port, NETC_PORT_EMAC, pause_stats);
		break;
	case ETHTOOL_MAC_STATS_SRC_PMAC:
		netc_port_pause_stats(port, NETC_PORT_PMAC, pause_stats);
		break;
	case ETHTOOL_MAC_STATS_SRC_AGGREGATE:
		ndev = dsa_to_port(ds, port_id)->user;
		ethtool_aggregate_pause_stats(ndev, pause_stats);
		break;
	}
}

static const struct ethtool_rmon_hist_range netc_rmon_ranges[] = {
	{   64,   64 },
	{   65,  127 },
	{  128,  255 },
	{  256,  511 },
	{  512, 1023 },
	{ 1024, 1522 },
	{ 1523, NETC_MAX_FRAME_LEN },
	{},
};

static void netc_port_rmon_stats(struct netc_port *port, enum netc_port_mac mac,
				 struct ethtool_rmon_stats *rmon_stats)
{
	if (mac == NETC_PORT_PMAC && !port->caps.pmac)
		return;

	rmon_stats->undersize_pkts = netc_port_rd64(port, NETC_PM_RUND(mac));
	rmon_stats->oversize_pkts = netc_port_rd64(port, NETC_PM_ROVR(mac));
	rmon_stats->fragments = netc_port_rd64(port, NETC_PM_RFRG(mac));
	rmon_stats->jabbers = netc_port_rd64(port, NETC_PM_RJBR(mac));

	rmon_stats->hist[0] = netc_port_rd64(port, NETC_PM_R64(mac));
	rmon_stats->hist[1] = netc_port_rd64(port, NETC_PM_R127(mac));
	rmon_stats->hist[2] = netc_port_rd64(port, NETC_PM_R255(mac));
	rmon_stats->hist[3] = netc_port_rd64(port, NETC_PM_R511(mac));
	rmon_stats->hist[4] = netc_port_rd64(port, NETC_PM_R1023(mac));
	rmon_stats->hist[5] = netc_port_rd64(port, NETC_PM_R1522(mac));
	rmon_stats->hist[6] = netc_port_rd64(port, NETC_PM_R1523X(mac));

	rmon_stats->hist_tx[0] = netc_port_rd64(port, NETC_PM_T64(mac));
	rmon_stats->hist_tx[1] = netc_port_rd64(port, NETC_PM_T127(mac));
	rmon_stats->hist_tx[2] = netc_port_rd64(port, NETC_PM_T255(mac));
	rmon_stats->hist_tx[3] = netc_port_rd64(port, NETC_PM_T511(mac));
	rmon_stats->hist_tx[4] = netc_port_rd64(port, NETC_PM_T1023(mac));
	rmon_stats->hist_tx[5] = netc_port_rd64(port, NETC_PM_T1522(mac));
	rmon_stats->hist_tx[6] = netc_port_rd64(port, NETC_PM_T1523X(mac));
}

void netc_port_get_rmon_stats(struct dsa_switch *ds, int port_id,
			      struct ethtool_rmon_stats *rmon_stats,
			      const struct ethtool_rmon_hist_range **ranges)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	struct net_device *ndev;

	*ranges = netc_rmon_ranges;

	switch (rmon_stats->src) {
	case ETHTOOL_MAC_STATS_SRC_EMAC:
		netc_port_rmon_stats(port, NETC_PORT_EMAC, rmon_stats);
		break;
	case ETHTOOL_MAC_STATS_SRC_PMAC:
		netc_port_rmon_stats(port, NETC_PORT_PMAC, rmon_stats);
		break;
	case ETHTOOL_MAC_STATS_SRC_AGGREGATE:
		ndev = dsa_to_port(ds, port_id)->user;
		ethtool_aggregate_rmon_stats(ndev, rmon_stats);
		break;
	}
}

static void netc_port_ctrl_stats(struct netc_port *port, enum netc_port_mac mac,
				 struct ethtool_eth_ctrl_stats *mac_stats)
{
	if (mac == NETC_PORT_PMAC && !port->caps.pmac)
		return;

	mac_stats->MACControlFramesTransmitted = netc_port_rd64(port, NETC_PM_TCNP(mac));
	mac_stats->MACControlFramesReceived = netc_port_rd64(port, NETC_PM_RCNP(mac));
}

void netc_port_get_eth_ctrl_stats(struct dsa_switch *ds, int port_id,
				  struct ethtool_eth_ctrl_stats *ctrl_stats)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	struct net_device *ndev;

	switch (ctrl_stats->src) {
	case ETHTOOL_MAC_STATS_SRC_EMAC:
		netc_port_ctrl_stats(port, NETC_PORT_EMAC, ctrl_stats);
		break;
	case ETHTOOL_MAC_STATS_SRC_PMAC:
		netc_port_ctrl_stats(port, NETC_PORT_PMAC, ctrl_stats);
		break;
	case ETHTOOL_MAC_STATS_SRC_AGGREGATE:
		ndev = dsa_to_port(ds, port_id)->user;
		ethtool_aggregate_ctrl_stats(ndev, ctrl_stats);
		break;
	}
}

static void netc_port_mac_stats(struct netc_port *port, enum netc_port_mac mac,
				struct ethtool_eth_mac_stats *mac_stats)
{
	if (mac == NETC_PORT_PMAC && !port->caps.pmac)
		return;

	mac_stats->FramesTransmittedOK = netc_port_rd64(port, NETC_PM_TFRM(mac));
	mac_stats->SingleCollisionFrames = netc_port_rd64(port, NETC_PM_TSCOL(mac));
	mac_stats->MultipleCollisionFrames = netc_port_rd64(port, NETC_PM_TMCOL(mac));
	mac_stats->FramesReceivedOK = netc_port_rd64(port, NETC_PM_RFRM(mac));
	mac_stats->FrameCheckSequenceErrors = netc_port_rd64(port, NETC_PM_RFCS(mac));
	mac_stats->AlignmentErrors = netc_port_rd64(port, NETC_PM_RALN(mac));
	mac_stats->OctetsTransmittedOK = netc_port_rd64(port, NETC_PM_TEOCT(mac));
	mac_stats->FramesWithDeferredXmissions = netc_port_rd64(port, NETC_PM_TDFR(mac));
	mac_stats->LateCollisions = netc_port_rd64(port, NETC_PM_TLCOL(mac));
	mac_stats->FramesAbortedDueToXSColls = netc_port_rd64(port, NETC_PM_TECOL(mac));
	mac_stats->FramesLostDueToIntMACXmitError = netc_port_rd64(port, NETC_PM_TERR(mac));
	mac_stats->OctetsReceivedOK = netc_port_rd64(port, NETC_PM_REOCT(mac));
	mac_stats->FramesLostDueToIntMACRcvError = netc_port_rd64(port, NETC_PM_RDRNTP(mac));
	mac_stats->MulticastFramesXmittedOK = netc_port_rd64(port, NETC_PM_TMCA(mac));
	mac_stats->BroadcastFramesXmittedOK = netc_port_rd64(port, NETC_PM_TBCA(mac));
	mac_stats->MulticastFramesReceivedOK = netc_port_rd64(port, NETC_PM_RMCA(mac));
	mac_stats->BroadcastFramesReceivedOK = netc_port_rd64(port, NETC_PM_RBCA(mac));
}

void netc_port_get_eth_mac_stats(struct dsa_switch *ds, int port_id,
				 struct ethtool_eth_mac_stats *mac_stats)
{
	struct netc_port *port = NETC_PORT(NETC_PRIV(ds), port_id);
	struct net_device *ndev;

	switch (mac_stats->src) {
	case ETHTOOL_MAC_STATS_SRC_EMAC:
		netc_port_mac_stats(port, NETC_PORT_EMAC, mac_stats);
		break;
	case ETHTOOL_MAC_STATS_SRC_PMAC:
		netc_port_mac_stats(port, NETC_PORT_PMAC, mac_stats);
		break;
	case ETHTOOL_MAC_STATS_SRC_AGGREGATE:
		ndev = dsa_to_port(ds, port_id)->user;
		ethtool_aggregate_mac_stats(ndev, mac_stats);
		break;
	}
}
