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

/**
 * @file ol_tx.h
 * @brief Internal definitions for the high-level tx module.
 */
#ifndef _OL_TX__H_
#define _OL_TX__H_

#include <adf_nbuf.h>    /* adf_nbuf_t */
#include <adf_os_lock.h>
#include <ol_txrx_api.h> /* ol_txrx_vdev_handle */
#include <ol_txrx_types.h>  /* ol_tx_desc_t, ol_txrx_msdu_info_t */

/**
 * struct ol_tx_per_pkt_stats - tx stats info of each packet sent out
 * @seq_no: sequential number
 * @chan_freq: channel freqency that this packet used
 * @bandwidth: channel bandwidth
 * @datarate: data rate
 * @tx_power: tx power
 * @mac_address: source MAC address that this packet used
 */
struct ol_tx_per_pkt_stats {
	uint32_t seq_no;
	uint32_t chan_freq;
	uint32_t bandwidth;
	uint8_t datarate;
	uint8_t tx_power;
	uint8_t mac_address[6];
};

#ifdef WLAN_FEATURE_DSRC
void
ol_per_pkt_tx_stats_enable(bool enable);

bool
ol_per_pkt_tx_stats_enabled(void);

void
ol_tx_stats_ring_enque_host(uint32_t msdu_id, uint32_t chan_freq,
			   uint32_t bandwidth, uint8_t *mac_address,
			   uint8_t datarate);

void
ol_tx_stats_ring_enque_comp(uint32_t msdu_id, uint32_t tx_power);

int
ol_tx_stats_ring_deque(struct ol_tx_per_pkt_stats *stats);
#else
static inline void
ol_per_pkt_tx_stats_enable(bool enable)
{
}

static inline bool
ol_per_pkt_tx_stats_enabled(void)
{
	return false;
}

static inline void
ol_tx_stats_ring_enque_host(uint32_t msdu_id, uint32_t chan_freq,
			   uint32_t bandwidth, uint8_t *mac_address,
			   uint8_t datarate)
{
}

static inline void
ol_tx_stats_ring_enque_comp(uint32_t msdu_id, uint32_t tx_power)
{
}

static inline int
ol_tx_stats_ring_deque(struct ol_tx_per_pkt_stats *stats)
{
	return 0;
}
#endif /* WLAN_FEATURE_DSRC */

adf_nbuf_t
ol_tx_ll(ol_txrx_vdev_handle vdev, adf_nbuf_t msdu_list);

adf_nbuf_t
ol_tx_ll_queue(ol_txrx_vdev_handle vdev, adf_nbuf_t msdu_list);

#ifdef QCA_SUPPORT_TXRX_DRIVER_TCP_DEL_ACK
adf_os_enum_hrtimer_t
ol_tx_hl_vdev_tcp_del_ack_timer(adf_os_hrtimer_t *timer);

void tcp_del_ack_tasklet(unsigned long data);

void ol_tx_hl_del_ack_queue_flush_all(struct ol_txrx_vdev_t *vdev);
#else
static inline
void ol_tx_hl_del_ack_queue_flush_all(struct ol_txrx_vdev_t *vdev)
{
	return;
}
#endif

#ifdef QCA_SUPPORT_TXRX_HL_BUNDLE
void
ol_tx_hl_vdev_bundle_timer(void *vdev);


void
ol_tx_hl_queue_flush_all(struct ol_txrx_vdev_t* vdev);

adf_nbuf_t
ol_tx_hl_queue(struct ol_txrx_vdev_t* vdev, adf_nbuf_t msdu_list);

#else

static inline void ol_tx_hl_vdev_bundle_timer(void *vdev)
{
	return;
}

static inline void
ol_tx_hl_queue_flush_all(struct ol_txrx_vdev_t* vdev)
{
	return;
}

#endif

#ifdef QCA_SUPPORT_TXRX_VDEV_LL_TXQ
#define OL_TX_LL ol_tx_ll_queue
#else
#define OL_TX_LL ol_tx_ll
#endif

#ifdef QCA_SUPPORT_TXRX_HL_BUNDLE
#define OL_TX_HL ol_tx_hl_queue
#else
#define OL_TX_HL ol_tx_hl
#endif

void ol_tx_vdev_ll_pause_queue_send(void *context);

adf_nbuf_t
ol_tx_non_std_ll(
    ol_txrx_vdev_handle data_vdev,
    enum ol_tx_spec tx_spec,
    adf_nbuf_t msdu_list);

adf_nbuf_t
ol_tx_hl(ol_txrx_vdev_handle vdev, adf_nbuf_t msdu_list);

adf_nbuf_t
ol_tx_non_std_hl(
    ol_txrx_vdev_handle data_vdev,
    enum ol_tx_spec tx_spec,
    adf_nbuf_t msdu_list);

adf_nbuf_t
ol_tx_reinject(struct ol_txrx_vdev_t *vdev, adf_nbuf_t msdu, u_int16_t peer_id);

void
ol_txrx_mgmt_tx_complete(void *ctxt, adf_nbuf_t netbuf, int err);

#ifdef QCA_SUPPORT_TXRX_VDEV_LL_TXQ
/**
 * ol_tx_vdev_ll_pause_start_timer() - Start ll-q pause timer for specific virtual device
 * @vdev: the virtual device
 *
 *  When system comes out of suspend, it is necessary to start the timer
 *  which will ensure to pull out all the queued packets after expiry.
 *  This function restarts the ll-pause timer, for the specific vdev device.
 *
 *
 * Return: None
 */
void
ol_tx_vdev_ll_pause_start_timer(struct ol_txrx_vdev_t *vdev);
#endif

void
ol_tx_pdev_ll_pause_queue_send_all(struct ol_txrx_pdev_t *pdev);

struct ol_txrx_vdev_t *
ol_txrx_get_vdev_from_vdev_id(uint8_t vdev_id);

ol_txrx_vdev_handle
ol_txrx_get_vdev_from_vdev_id(uint8_t vdev_id);

#endif /* _OL_TX__H_ */

