/*
 * Copyright (c) 2015,2017 The Linux Foundation. All rights reserved.
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

#if !defined WLAN_HDD_TSF_H
#define WLAN_HDD_TSF_H

/*---------------------------------------------------------------------------
  Include files
  -------------------------------------------------------------------------*/
#include <wlan_hdd_includes.h>
#include "wlan_hdd_cfg.h"

/*---------------------------------------------------------------------------
  Preprocessor definitions and constants
  -------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
  Function declarations and documentation
  -------------------------------------------------------------------------*/

#ifdef WLAN_FEATURE_TSF

/**
 * wlan_hdd_tsf_init() - set gpio and callbacks for
 *     capturing tsf and init tsf_plus
 * @hdd_ctx: pointer to the hdd_context_t
 *
 * This function set the callback to sme module, the callback will be
 * called when a tsf event is reported by firmware; set gpio number
 * to FW, FW will toggle this gpio when received a CAP_TSF command;
 * do tsf_plus init
 *
 * Return: nothing
 */
void wlan_hdd_tsf_init(hdd_context_t *hdd_ctx);

/**
 * wlan_hdd_tsf_deinit() - reset callbacks for capturing tsf, deinit tsf_plus
 * @hdd_ctx: pointer to the hdd_context_t
 *
 * This function reset the callback to sme module, and deinit tsf_plus
 *
 * Return: nothing
 */
void wlan_hdd_tsf_deinit(hdd_context_t *hdd_ctx);

/**
 * hdd_capture_tsf() - capture tsf
 * @adapter: pointer to adapter
 * @buf: pointer to uplayer buf
 * @len : the length of buf
 *
 * This function returns tsf value to uplayer.
 *
 * Return: Describe the execute result of this routine
 */
int hdd_capture_tsf(hdd_adapter_t *adapter, uint32_t *buf, int len);

/**
 * hdd_indicate_tsf() - return tsf to uplayer
 * @adapter: pointer to adapter
 * @buf: pointer to uplayer buf
 * @len : the length of buf
 *
 * This function returns tsf value to uplayer.
 *
 * Return: Describe the execute result of this routine
 */
int hdd_indicate_tsf(hdd_adapter_t *adapter, uint32_t *buf, int len);
#else
static inline void
wlan_hdd_tsf_init(hdd_context_t *hdd_ctx)
{
}

static inline void wlan_hdd_tsf_deinit(hdd_context_t *hdd_ctx)
{
}

static inline int hdd_indicate_tsf(hdd_adapter_t *adapter,
	uint32_t *buf, int len)
{
	return -ENOTSUPP;
}

static inline int
hdd_capture_tsf(hdd_adapter_t *adapter, uint32_t *buf, int len)
{
	return -ENOTSUPP;
}
#endif

#if defined(WLAN_FEATURE_TSF_PLUS) && defined(WLAN_FEATURE_TSF)
#define HDD_TSF_IS_PTP_ENABLED(hdd_ctx) \
	((hdd_ctx) && (hdd_ctx)->cfg_ini && \
	((hdd_ctx)->cfg_ini->tsf_ptp_options))

#define HDD_TSF_IS_TX_SET(hdd_ctx) \
	((hdd_ctx) && (hdd_ctx)->cfg_ini && \
	(((hdd_ctx)->cfg_ini->tsf_ptp_options & CFG_SET_TSF_PTP_OPT_TX) == \
	CFG_SET_TSF_PTP_OPT_TX))

#define HDD_TSF_IS_RX_SET(hdd_ctx) \
	((hdd_ctx) && (hdd_ctx)->cfg_ini && \
	(((hdd_ctx)->cfg_ini->tsf_ptp_options & CFG_SET_TSF_PTP_OPT_RX) == \
	CFG_SET_TSF_PTP_OPT_RX))

#define HDD_TSF_IS_RAW_SET(hdd_ctx) \
	((hdd_ctx) && (hdd_ctx)->cfg_ini && \
	(((hdd_ctx)->cfg_ini->tsf_ptp_options & CFG_SET_TSF_PTP_OPT_RAW) == \
	CFG_SET_TSF_PTP_OPT_RAW))

#define HDD_TSF_IS_DBG_FS_SET(hdd_ctx) \
	((hdd_ctx) && (hdd_ctx)->cfg_ini && \
	(((hdd_ctx)->cfg_ini->tsf_ptp_options & CFG_SET_TSF_DBG_FS) == \
	CFG_SET_TSF_DBG_FS))

/**
 * hdd_start_tsf_sync() - start tsf sync
 * @adapter: pointer to adapter
 *
 * This function initialize and start TSF synchronization
 *
 * Return: Describe the execute result of this routine
 */
int hdd_start_tsf_sync(hdd_adapter_t *adapter);

/**
 * hdd_stop_tsf_sync() - stop tsf sync
 * @adapter: pointer to adapter
 *
 * This function stop and de-initialize TSF synchronization
 *
 * Return: Describe the execute result of this routine
 */
int hdd_stop_tsf_sync(hdd_adapter_t *adapter);

/**
 * hdd_tsf_notify_wlan_state_change() -
 *     notify tsf module of wlan connection state
 * @old_state: old wlan state
 * @new_state: new wlan state
 *
 * This function check the old and new connection state, determine whether
 * to start or stop tsf sync
 *
 * Return: nothing
 */
void hdd_tsf_notify_wlan_state_change(hdd_adapter_t *adapter,
	eConnectionState old_state,
	eConnectionState new_state);

/**
 * hdd_tx_timestamp() - time stamp TX netbuf
 *
 * @netbuf: pointer to a TX netbuf
 * @target_time: TX time for the netbuf
 *
 * This function  get corresponding host time from target time,
 * and time stamp the TX netbuf with this time
 *
 * Return: Describe the execute result of this routine
 */
int hdd_tx_timestamp(adf_nbuf_t netbuf, uint64_t target_time);

/**
 * hdd_rx_timestamp() - time stamp RX netbuf
 *
 * @netbuf: pointer to a RX netbuf
 * @target_time: RX time for the netbuf
 *
 * This function get corresponding host time from target time,
 * and time stamp the RX netbuf with this time
 *
 * Return: Describe the execute result of this routine
 */
int hdd_rx_timestamp(adf_nbuf_t netbuf, uint64_t target_time);

/**
 * hdd_tsf_record_sk_for_skb() - save sk for skb
 *
 * @hdd_ctx: pointer to hdd context
 * @nbuf: pointer to a TX netbuf
 *
 * This function record netbuf->sk in netbuf->tstamp,
 * in case netbuf->sk will be set to NULL.
 *
 * Return: nothing
 */
void
hdd_tsf_record_sk_for_skb(hdd_context_t *hdd_ctx, adf_nbuf_t nbuf);
#else
static inline int hdd_start_tsf_sync(hdd_adapter_t *adapter)
{
	return -ENOTSUPP;
}

static inline int hdd_stop_tsf_sync(hdd_adapter_t *adapter)
{
	return -ENOTSUPP;
}

static inline
void hdd_tsf_notify_wlan_state_change(hdd_adapter_t *adapter,
	eConnectionState old_state,
	eConnectionState new_state)
{
}

static inline
int hdd_tx_timestamp(adf_nbuf_t netbuf, uint64_t target_time)
{
	return -ENOTSUPP;
}

static inline
int hdd_rx_timestamp(adf_nbuf_t netbuf, uint64_t target_time)
{
	return -ENOTSUPP;
}

static inline void
hdd_tsf_record_sk_for_skb(hdd_context_t *hdd_ctx, adf_nbuf_t nbuf)
{
}
#endif

#endif
