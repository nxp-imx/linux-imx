/*
 * Copyright (c) 2011-2014,2016-2017 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

#include <ol_cfg.h>
#include <ol_if_athvar.h>
#include <vos_types.h>
#include <vos_getBin.h>

unsigned int vow_config = 0;
module_param(vow_config, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(vow_config, "Do VoW Configuration");

#ifdef QCA_SUPPORT_TXRX_DRIVER_TCP_DEL_ACK
/**
 * ol_cfg_update_del_ack_params() - update delayed ack params
 *
 * @cfg_ctx: cfg context
 * @cfg_param: parameters
 *
 * Return: none
 */
void ol_cfg_update_del_ack_params(struct txrx_pdev_cfg_t *cfg_ctx,
				struct txrx_pdev_cfg_param_t cfg_param)
{
	cfg_ctx->del_ack_enable = cfg_param.del_ack_enable;
	cfg_ctx->del_ack_timer_value = cfg_param.del_ack_timer_value;
	cfg_ctx->del_ack_pkt_count = cfg_param.del_ack_pkt_count;
}
#endif

#ifdef QCA_SUPPORT_TXRX_HL_BUNDLE
/**
 * ol_cfg_update_bundle_params() - update tx bundle params
 * @cfg_ctx: cfg context
 * @cfg_param: parameters
 *
 * Return: none
 */
void ol_cfg_update_bundle_params(struct txrx_pdev_cfg_t *cfg_ctx,
				struct txrx_pdev_cfg_param_t cfg_param)
{
	cfg_ctx->pkt_bundle_timer_value = cfg_param.pkt_bundle_timer_value;
	cfg_ctx->pkt_bundle_size = cfg_param.pkt_bundle_size;
}
#else
void ol_cfg_update_bundle_params(struct txrx_pdev_cfg_t *cfg_ctx,
				struct txrx_pdev_cfg_param_t cfg_param)
{
	return;
}
#endif


/* FIX THIS -
 * For now, all these configuration parameters are hardcoded.
 * Many of these should actually be determined dynamically instead.
 */

ol_pdev_handle ol_pdev_cfg_attach(adf_os_device_t osdev,
                                   struct txrx_pdev_cfg_param_t cfg_param)
{
	struct txrx_pdev_cfg_t *cfg_ctx;
	int i;

	cfg_ctx = adf_os_mem_alloc(osdev, sizeof(*cfg_ctx));
	if (!cfg_ctx) {
		printk(KERN_ERR "cfg ctx allocation failed\n");
		return NULL;
	}

#ifdef CONFIG_HL_SUPPORT
	cfg_ctx->is_high_latency = 1;
	/* 802.1Q and SNAP / LLC headers are accounted for elsewhere */
	cfg_ctx->tx_download_size = 1500;
	cfg_ctx->tx_free_at_download = 0;
#else
	/*
	 * Need to change HTT_LL_TX_HDR_SIZE_IP accordingly.
	 * Include payload, up to the end of UDP header for IPv4 case
	 */
	cfg_ctx->tx_download_size = 16;
#endif
	/* temporarily diabled PN check for Riva/Pronto */
	cfg_ctx->rx_pn_check = 1;
#if CFG_TGT_DEFAULT_RX_SKIP_DEFRAG_TIMEOUT_DUP_DETECTION_CHECK
	cfg_ctx->defrag_timeout_check = 1;
#else
	cfg_ctx->defrag_timeout_check = 0;
#endif
	cfg_ctx->max_peer_id = 511;
	cfg_ctx->max_vdev = CFG_TGT_NUM_VDEV;
	cfg_ctx->pn_rx_fwd_check = 1;
	if (VOS_MONITOR_MODE == vos_get_conparam())
		cfg_ctx->frame_type = wlan_frm_fmt_raw;
	else
		cfg_ctx->frame_type = wlan_frm_fmt_802_3;
	cfg_ctx->max_thruput_mbps = 800;
	cfg_ctx->max_nbuf_frags = 1;
	cfg_ctx->vow_config = vow_config;
	cfg_ctx->target_tx_credit = CFG_TGT_NUM_MSDU_DESC;
	cfg_ctx->throttle_period_ms = 40;
	cfg_ctx->dutycycle_level[0] = THROTTLE_DUTY_CYCLE_LEVEL0;
	cfg_ctx->dutycycle_level[1] = THROTTLE_DUTY_CYCLE_LEVEL1;
	cfg_ctx->dutycycle_level[2] = THROTTLE_DUTY_CYCLE_LEVEL2;
	cfg_ctx->dutycycle_level[3] = THROTTLE_DUTY_CYCLE_LEVEL3;
	cfg_ctx->rx_fwd_disabled = 0;
	cfg_ctx->is_packet_log_enabled = 0;
#ifdef WLAN_FEATURE_TSF_PLUS
	cfg_ctx->is_ptp_rx_opt_enabled = 0;
#endif
	cfg_ctx->is_full_reorder_offload = cfg_param.is_full_reorder_offload;
#ifdef IPA_UC_OFFLOAD
	cfg_ctx->ipa_uc_rsc.uc_offload_enabled = cfg_param.is_uc_offload_enabled;
	cfg_ctx->ipa_uc_rsc.tx_max_buf_cnt = cfg_param.uc_tx_buffer_count;
	cfg_ctx->ipa_uc_rsc.tx_buf_size = cfg_param.uc_tx_buffer_size;
	cfg_ctx->ipa_uc_rsc.rx_ind_ring_size = cfg_param.uc_rx_indication_ring_count;
	cfg_ctx->ipa_uc_rsc.tx_partition_base = cfg_param.uc_tx_partition_base;
#endif /* IPA_UC_OFFLOAD */

	ol_cfg_update_bundle_params(cfg_ctx, cfg_param);
	ol_cfg_update_del_ack_params(cfg_ctx, cfg_param);
	ol_cfg_update_ptp_params(cfg_ctx, cfg_param);

	for (i = 0; i < OL_TX_NUM_WMM_AC; i++) {
		cfg_ctx->ac_specs[i].wrr_skip_weight =
			cfg_param.ac_specs[i].wrr_skip_weight;
		cfg_ctx->ac_specs[i].credit_threshold =
			cfg_param.ac_specs[i].credit_threshold;
		cfg_ctx->ac_specs[i].send_limit =
			cfg_param.ac_specs[i].send_limit;
		cfg_ctx->ac_specs[i].credit_reserve =
			cfg_param.ac_specs[i].credit_reserve;
		cfg_ctx->ac_specs[i].discard_weight =
			cfg_param.ac_specs[i].discard_weight;
	}

	return (ol_pdev_handle) cfg_ctx;
}

#ifdef QCA_SUPPORT_TXRX_DRIVER_TCP_DEL_ACK
/**
 * ol_cfg_get_del_ack_timer_value() - get delayed ack timer value
 * @pdev: pdev handle
 *
 * Return: timer value
 */
int ol_cfg_get_del_ack_timer_value(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;

	return cfg->del_ack_timer_value;
}

/**
 * ol_cfg_get_del_ack_enable_value() - get delayed ack enable value
 * @pdev: pdev handle
 *
 * Return: enable/disable
 */
int ol_cfg_get_del_ack_enable_value(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;

	return cfg->del_ack_enable;
}

/**
 * ol_cfg_get_del_ack_count_value() - get delayed ack count value
 * @pdev: pdev handle
 *
 * Return: count value
 */
int ol_cfg_get_del_ack_count_value(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;

	return cfg->del_ack_pkt_count;
}
#endif

#ifdef QCA_SUPPORT_TXRX_HL_BUNDLE
/**
 * ol_cfg_get_bundle_timer_value() - get bundle timer value
 * @pdev: pdev handle
 *
 * Return: bundle timer value
 */
int ol_cfg_get_bundle_timer_value(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->pkt_bundle_timer_value;
}

/**
 * ol_cfg_get_bundle_size() - get bundle size value
 * @pdev: pdev handle
 *
 * Return: bundle size value
 */
int ol_cfg_get_bundle_size(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->pkt_bundle_size;
}
#endif

int ol_cfg_is_high_latency(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->is_high_latency;
}

int ol_cfg_max_peer_id(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	/*
	 * TBDXXX - this value must match the peer table
	 * size allocated in FW
	 */
	return cfg->max_peer_id;
}

int ol_cfg_max_vdevs(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->max_vdev;
}

int ol_cfg_rx_pn_check(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->rx_pn_check;
}

int ol_cfg_rx_fwd_check(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->pn_rx_fwd_check;
}

void ol_set_cfg_rx_fwd_disabled(ol_pdev_handle pdev, u_int8_t disable_rx_fwd)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	cfg->rx_fwd_disabled = disable_rx_fwd;
}

void ol_set_cfg_packet_log_enabled(ol_pdev_handle pdev, u_int8_t val)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	cfg->is_packet_log_enabled = val;
}

u_int8_t ol_cfg_is_packet_log_enabled(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->is_packet_log_enabled;
}

#ifdef WLAN_FEATURE_TSF_PLUS
void ol_set_cfg_ptp_rx_opt_enabled(ol_pdev_handle pdev, u_int8_t val)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;

	cfg->is_ptp_rx_opt_enabled = val;
}

u_int8_t ol_cfg_is_ptp_rx_opt_enabled(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;

	return cfg->is_ptp_rx_opt_enabled;
}
/**
 * ol_cfg_is_ptp_enabled() - check if ptp feature is enabled
 * @pdev: cfg handle to PDEV
 *
 * Return: is_ptp_enabled
 */
a_bool_t ol_cfg_is_ptp_enabled(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;

	return cfg->is_ptp_enabled;
}
/**
 * ol_cfg_update_ptp_params() - update ptp params
 * @cfg_ctx: cfg context
 * @cfg_param: parameters
 *
 * Return: none
 */
void ol_cfg_update_ptp_params(struct txrx_pdev_cfg_t *cfg_ctx,
				struct txrx_pdev_cfg_param_t cfg_param)
{
	cfg_ctx->is_ptp_enabled = cfg_param.is_ptp_enabled;
}
#endif

int ol_cfg_rx_fwd_disabled(ol_pdev_handle pdev)
{
#if defined(ATHR_WIN_NWF)
	/* for Windows, let the OS handle the forwarding */
	return 1;
#else
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->rx_fwd_disabled;
#endif
}

int ol_cfg_rx_fwd_inter_bss(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->rx_fwd_inter_bss;
}

enum wlan_frm_fmt ol_cfg_frame_type(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->frame_type;
}

int ol_cfg_max_thruput_mbps(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->max_thruput_mbps;
}

int ol_cfg_netbuf_frags_max(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->max_nbuf_frags;
}

int ol_cfg_tx_free_at_download(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->tx_free_at_download;
}

void ol_cfg_set_tx_free_at_download(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	cfg->tx_free_at_download = 1;
}
u_int16_t ol_cfg_target_tx_credit(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
#ifndef CONFIG_HL_SUPPORT
	u_int16_t vow_max_sta =  (cfg->vow_config & 0xffff0000) >> 16;
	u_int16_t vow_max_desc_persta = cfg->vow_config & 0x0000ffff;

	return (cfg->target_tx_credit +
		(vow_max_sta * vow_max_desc_persta));
#else
	return cfg->target_tx_credit;
#endif
}

int ol_cfg_tx_download_size(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->tx_download_size;
}

int ol_cfg_rx_host_defrag_timeout_duplicate_check(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->defrag_timeout_check;
}

int ol_cfg_throttle_period_ms(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->throttle_period_ms;
}

int ol_cfg_throttle_duty_cycle_level(ol_pdev_handle pdev, int level)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->dutycycle_level[level];
}


int ol_cfg_is_full_reorder_offload(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->is_full_reorder_offload;
}

#ifdef IPA_UC_OFFLOAD
unsigned int ol_cfg_ipa_uc_offload_enabled(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return (unsigned int)cfg->ipa_uc_rsc.uc_offload_enabled;
}

unsigned int ol_cfg_ipa_uc_tx_buf_size(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->ipa_uc_rsc.tx_buf_size;
}

unsigned int ol_cfg_ipa_uc_tx_max_buf_cnt(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->ipa_uc_rsc.tx_max_buf_cnt;
}

unsigned int ol_cfg_ipa_uc_rx_ind_ring_size(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->ipa_uc_rsc.rx_ind_ring_size;
}

unsigned int ol_cfg_ipa_uc_tx_partition_base(ol_pdev_handle pdev)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;
	return cfg->ipa_uc_rsc.tx_partition_base;
}
#endif /* IPA_UC_OFFLOAD */

/**
 * ol_cfg_get_wrr_skip_weight() - brief Query for the param of wrr_skip_weight
 * @pdev: handle to the physical device.
 * @ac: access control, it will be BE, BK, VI, VO
 *
 * Return: wrr_skip_weight for specified ac.
 */
int ol_cfg_get_wrr_skip_weight(ol_pdev_handle pdev, int ac)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;

	if (ac >= OL_TX_WMM_AC_BE && ac <= OL_TX_WMM_AC_VO)
		return cfg->ac_specs[ac].wrr_skip_weight;
	else
		return 0;
}

/**
 * ol_cfg_get_credit_threshold() - Query for the param of credit_threshold
 * @pdev: handle to the physical device.
 * @ac: access control, it will be BE, BK, VI, VO
 *
 * Return: credit_threshold for specified ac.
 */
uint32_t ol_cfg_get_credit_threshold(ol_pdev_handle pdev, int ac)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;

	if (ac >= OL_TX_WMM_AC_BE && ac <= OL_TX_WMM_AC_VO)
		return cfg->ac_specs[ac].credit_threshold;
	else
		return 0;
}


/**
 * ol_cfg_get_send_limit() - Query for the param of send_limit
 * @pdev: handle to the physical device.
 * @ac: access control, it will be BE, BK, VI, VO
 *
 * Return: send_limit for specified ac.
 */
uint16_t ol_cfg_get_send_limit(ol_pdev_handle pdev, int ac)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;

	if (ac >= OL_TX_WMM_AC_BE && ac <= OL_TX_WMM_AC_VO)
		return cfg->ac_specs[ac].send_limit;
	else
		return 0;
}


/**
 * ol_cfg_get_credit_reserve() - Query for the param of credit_reserve
 * @pdev: handle to the physical device.
 * @ac: access control, it will be BE, BK, VI, VO
 *
 * Return: credit_reserve for specified ac.
 */
int ol_cfg_get_credit_reserve(ol_pdev_handle pdev, int ac)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;

	if (ac >= OL_TX_WMM_AC_BE && ac <= OL_TX_WMM_AC_VO)
		return cfg->ac_specs[ac].credit_reserve;
	else
		return 0;
}

/**
 * ol_cfg_get_discard_weight() - Query for the param of discard_weight
 * @pdev: handle to the physical device.
 * @ac: access control, it will be BE, BK, VI, VO
 *
 * Return: discard_weight for specified ac.
 */
int ol_cfg_get_discard_weight(ol_pdev_handle pdev, int ac)
{
	struct txrx_pdev_cfg_t *cfg = (struct txrx_pdev_cfg_t *)pdev;

	if (ac >= OL_TX_WMM_AC_BE && ac <= OL_TX_WMM_AC_VO)
		return cfg->ac_specs[ac].discard_weight;
	else
		return 0;
}

