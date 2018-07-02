/*
 * Copyright (c) 2012-2018 The Linux Foundation. All rights reserved.
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

/**===========================================================================

  \file  wlan_hdd_tx_rx.c

  \brief Linux HDD Tx/RX APIs

  ==========================================================================*/

/*---------------------------------------------------------------------------
  Include files
  -------------------------------------------------------------------------*/

/* Needs to be removed when completely root-caused */
#define IPV6_MCAST_WAR 1

#include <wlan_hdd_tx_rx.h>
#include <wlan_hdd_softap_tx_rx.h>
#include <wlan_hdd_dp_utils.h>
#include <wlan_hdd_tsf.h>

#include <wlan_qct_tl.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <vos_sched.h>
#ifdef IPV6_MCAST_WAR
#include <linux/if_ether.h>
#endif

#include <linux/inetdevice.h>
#include <wlan_hdd_p2p.h>
#include <linux/wireless.h>
#include <net/cfg80211.h>
#include <net/ieee80211_radiotap.h>
#include "sapApi.h"

#ifdef FEATURE_WLAN_TDLS
#include "wlan_hdd_tdls.h"
#endif

#ifdef IPA_OFFLOAD
#include <wlan_hdd_ipa.h>
#endif
#include "adf_trace.h"

#include "wlan_hdd_nan_datapath.h"

/*---------------------------------------------------------------------------
  Preprocessor definitions and constants
  -------------------------------------------------------------------------*/
const v_U8_t hddWmmAcToHighestUp[] = {
   SME_QOS_WMM_UP_RESV,
   SME_QOS_WMM_UP_EE,
   SME_QOS_WMM_UP_VI,
   SME_QOS_WMM_UP_NC
};

//Mapping Linux AC interpretation to TL AC.
const v_U8_t hdd_QdiscAcToTlAC[] = {
   WLANTL_AC_VO,
   WLANTL_AC_VI,
   WLANTL_AC_BE,
   WLANTL_AC_BK,
};

/*---------------------------------------------------------------------------
  Function definitions and documentation
  -------------------------------------------------------------------------*/

/**============================================================================
  @brief hdd_flush_tx_queues() - Utility function to flush the TX queues

  @param pAdapter : [in] pointer to adapter context
  @return         : VOS_STATUS_E_FAILURE if any errors encountered
                  : VOS_STATUS_SUCCESS otherwise
  ===========================================================================*/
static VOS_STATUS hdd_flush_tx_queues( hdd_adapter_t *pAdapter )
{
   VOS_STATUS status = VOS_STATUS_SUCCESS;
   v_SINT_t i = -1;
   hdd_list_node_t *anchor = NULL;
   skb_list_node_t *pktNode = NULL;
   struct sk_buff *skb = NULL;

   pAdapter->isVosLowResource = VOS_FALSE;

   while (++i != NUM_TX_QUEUES)
   {
      //Free up any packets in the Tx queue
      SPIN_LOCK_BH(&pAdapter->wmm_tx_queue[i].lock);
      while (true)
      {
         status = hdd_list_remove_front( &pAdapter->wmm_tx_queue[i], &anchor );
         if(VOS_STATUS_E_EMPTY != status)
         {
            pktNode = list_entry(anchor, skb_list_node_t, anchor);
            skb = pktNode->skb;
            ++pAdapter->stats.tx_dropped;
            kfree_skb(skb);
            continue;
         }
         break;
      }
      SPIN_UNLOCK_BH(&pAdapter->wmm_tx_queue[i].lock);
      /* Back pressure is no longer in effect */
      pAdapter->isTxSuspended[i] = VOS_FALSE;
   }

   return status;
}

/**============================================================================
  @brief hdd_flush_ibss_tx_queues() - Utility function to flush the TX queues
                                      in IBSS mode

  @param pAdapter : [in] pointer to adapter context
                  : [in] Station Id
  @return         : VOS_STATUS_E_FAILURE if any errors encountered
                  : VOS_STATUS_SUCCESS otherwise
  ===========================================================================*/
void hdd_flush_ibss_tx_queues( hdd_adapter_t *pAdapter, v_U8_t STAId)
{
   v_U8_t i;
   v_SIZE_t size = 0;
   v_U8_t skbStaIdx;
   skb_list_node_t *pktNode = NULL;
   hdd_list_node_t *tmp = NULL, *next = NULL;
   struct netdev_queue *txq;
   struct sk_buff *skb = NULL;

   if (NULL == pAdapter)
   {
       VOS_TRACE( VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_ERROR,
              FL("pAdapter is NULL %u"), STAId);
       VOS_ASSERT(0);
       return;
   }

   for (i = 0; i < NUM_TX_QUEUES; i++)
   {
      SPIN_LOCK_BH(&pAdapter->wmm_tx_queue[i].lock);

      if ( list_empty( &pAdapter->wmm_tx_queue[i].anchor ) )
      {
         SPIN_UNLOCK_BH(&pAdapter->wmm_tx_queue[i].lock);
         continue;
      }

      /* Iterate through the queue and identify the data for STAId */
      list_for_each_safe(tmp, next, &pAdapter->wmm_tx_queue[i].anchor)
      {
         pktNode = list_entry(tmp, skb_list_node_t, anchor);
         skb = pktNode->skb;

         /* Get the STAId from data */
         skbStaIdx = *(v_U8_t *)(((v_U8_t *)(skb->data)) - 1);
         if (skbStaIdx == STAId) {
            /* Data for STAId is freed along with the queue node */

            list_del(tmp);
            kfree_skb(skb);

            pAdapter->wmm_tx_queue[i].count--;
         }
      }

      /* Restart the queue only-if suspend and the queue was flushed */
      hdd_list_size( &pAdapter->wmm_tx_queue[i], &size );
      txq = netdev_get_tx_queue(pAdapter->dev, i);

      if (VOS_TRUE == pAdapter->isTxSuspended[i] &&
          size <= HDD_TX_QUEUE_LOW_WATER_MARK &&
          netif_tx_queue_stopped(txq) )
      {
         hddLog(LOG1, FL("Enabling queue for queue %d"), i);
         netif_tx_start_queue(txq);
         pAdapter->isTxSuspended[i] = VOS_FALSE;
      }

      SPIN_UNLOCK_BH(&pAdapter->wmm_tx_queue[i].lock);
   }
}

#ifdef QCA_LL_TX_FLOW_CT
/**============================================================================
  @brief hdd_tx_resume_timer_expired_handler() - Resume OS TX Q timer expired
      handler.
      If Blocked OS Q is not resumed during timeout period, to prevent
      permanent stall, resume OS Q forcefully.

  @param adapter_context : [in] pointer to vdev adapter

  @return         : NONE
  ===========================================================================*/
void hdd_tx_resume_timer_expired_handler(void *adapter_context)
{
   hdd_adapter_t *pAdapter = (hdd_adapter_t *)adapter_context;

   if (!pAdapter)
   {
      /* INVALID ARG */
      return;
   }

   hddLog(LOG1, FL("Enabling queues"));
   wlan_hdd_netif_queue_control(pAdapter, WLAN_WAKE_ALL_NETIF_QUEUE,
            WLAN_DATA_FLOW_CONTROL);
   pAdapter->hdd_stats.hddTxRxStats.txflow_unpause_cnt++;
   pAdapter->hdd_stats.hddTxRxStats.is_txflow_paused = FALSE;

   return;
}

/**============================================================================
  @brief hdd_tx_resume_cb() - Resume OS TX Q.
      Q was stopped due to WLAN TX path low resource condition

  @param adapter_context : [in] pointer to vdev adapter
  @param tx_resume       : [in] TX Q resume trigger

  @return         : NONE
  ===========================================================================*/
void hdd_tx_resume_cb(void *adapter_context,
                        v_BOOL_t tx_resume)
{
   hdd_adapter_t *pAdapter = (hdd_adapter_t *)adapter_context;
   hdd_station_ctx_t *hdd_sta_ctx = NULL;

   if (!pAdapter)
   {
      /* INVALID ARG */
      return;
   }

   hdd_sta_ctx = WLAN_HDD_GET_STATION_CTX_PTR(pAdapter);

   /* Resume TX  */
   if (VOS_TRUE == tx_resume)
   {
       if (VOS_TIMER_STATE_STOPPED !=
          vos_timer_getCurrentState(&pAdapter->tx_flow_control_timer))
       {
          vos_timer_stop(&pAdapter->tx_flow_control_timer);
       }
       hddLog(LOG1, FL("Enabling queues"));
       wlan_hdd_netif_queue_control(pAdapter,
            WLAN_WAKE_ALL_NETIF_QUEUE,
            WLAN_DATA_FLOW_CONTROL);
       pAdapter->hdd_stats.hddTxRxStats.txflow_unpause_cnt++;
       pAdapter->hdd_stats.hddTxRxStats.is_txflow_paused = FALSE;

   }
#if defined(CONFIG_PER_VDEV_TX_DESC_POOL)
    else if (VOS_FALSE == tx_resume)  /* Pause TX  */
    {
        hddLog(LOG1, FL("Disabling queues"));
        wlan_hdd_netif_queue_control(pAdapter,
            WLAN_STOP_ALL_NETIF_QUEUE,
            WLAN_DATA_FLOW_CONTROL);
        if (VOS_TIMER_STATE_STOPPED ==
            vos_timer_getCurrentState(&pAdapter->tx_flow_control_timer))
        {
            VOS_STATUS status;
            status = vos_timer_start(&pAdapter->tx_flow_control_timer,
                          WLAN_HDD_TX_FLOW_CONTROL_OS_Q_BLOCK_TIME);
            if ( !VOS_IS_STATUS_SUCCESS(status) )
            {
                VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                "%s: Failed to start tx_flow_control_timer", __func__);
            }
            else
            {
                pAdapter->hdd_stats.hddTxRxStats.txflow_timer_cnt++;
            }
        }
        pAdapter->hdd_stats.hddTxRxStats.txflow_pause_cnt++;
        pAdapter->hdd_stats.hddTxRxStats.is_txflow_paused = TRUE;

    }
#endif

   return;
}
#endif /* QCA_LL_TX_FLOW_CT */

/**
 * hdd_drop_skb() - drop a packet
 * @adapter: pointer to hdd adapter
 * @skb: pointer to OS packet
 *
 * Return: Nothing to return
 */
void hdd_drop_skb(hdd_adapter_t *adapter, struct sk_buff *skb)
{
	DPTRACE(adf_dp_trace(skb, ADF_DP_TRACE_DROP_PACKET_RECORD,
			(uint8_t *)skb->data, skb->len, ADF_TX));
	if (skb->len > ADF_DP_TRACE_RECORD_SIZE)
		DPTRACE(adf_dp_trace(skb, ADF_DP_TRACE_DROP_PACKET_RECORD,
				(uint8_t *)&skb->data[ADF_DP_TRACE_RECORD_SIZE],
				(skb->len - ADF_DP_TRACE_RECORD_SIZE), ADF_TX));

	++adapter->stats.tx_dropped;
	++adapter->hdd_stats.hddTxRxStats.txXmitDropped;
	kfree_skb(skb);
}

/**
 * hdd_drop_skb_list() - drop packet list
 * @adapter: pointer to hdd adapter
 * @skb: pointer to OS packet list
 * @is_update_ac_stats: macro to update ac stats
 *
 * Return: Nothing to return
 */
void hdd_drop_skb_list(hdd_adapter_t *adapter, struct sk_buff *skb,
                                               bool is_update_ac_stats)
{
	WLANTL_ACEnumType ac;
	struct sk_buff *skb_next;

	while (skb) {
		DPTRACE(adf_dp_trace(skb, ADF_DP_TRACE_DROP_PACKET_RECORD,
				(uint8_t *)skb->data, skb->len, ADF_TX));
		if (skb->len > ADF_DP_TRACE_RECORD_SIZE)
			DPTRACE(adf_dp_trace(skb,
				ADF_DP_TRACE_DROP_PACKET_RECORD,
				(uint8_t *)&skb->data[ADF_DP_TRACE_RECORD_SIZE],
				(skb->len - ADF_DP_TRACE_RECORD_SIZE), ADF_TX));

		++adapter->stats.tx_dropped;
		++adapter->hdd_stats.hddTxRxStats.txXmitDropped;
		if (is_update_ac_stats == TRUE) {
			ac = hdd_QdiscAcToTlAC[skb->queue_mapping];
			++adapter->hdd_stats.hddTxRxStats.txXmitDroppedAC[ac];
		}
		skb_next = skb->next;
		kfree_skb(skb);
		skb = skb_next;
	}
}

/**
 * wlan_hdd_classify_pkt() - classify skb packet type.
 * @data: Pointer to skb
 *
 * This function classifies skb packet type.
 *
 * Return: none
 */
void wlan_hdd_classify_pkt(struct sk_buff *skb)
{
	/* classify broadcast/multicast packet */
	if (adf_nbuf_is_bcast_pkt(skb))
		ADF_NBUF_SET_BCAST(skb);
	else if (adf_nbuf_is_multicast_pkt(skb))
		ADF_NBUF_SET_MCAST(skb);

	/* classify eapol/arp/dhcp/wai packet */
	if (adf_nbuf_is_eapol_pkt(skb))
		ADF_NBUF_SET_EAPOL(skb);
	else if (adf_nbuf_is_ipv4_arp_pkt(skb))
		ADF_NBUF_SET_ARP(skb);
	else if (adf_nbuf_is_dhcp_pkt(skb))
		ADF_NBUF_SET_DHCP(skb);
	else if (adf_nbuf_is_wai_pkt(skb))
		ADF_NBUF_SET_WAPI(skb);
}

/**
 * hdd_get_transmit_sta_id() - function to retrieve station id to be used for
 * sending traffic towards a particular destination address. The destination
 * address can be unicast, multicast or broadcast
 *
 * @adapter: Handle to adapter context
 * @dst_addr: Destination address
 * @station_id: station id
 *
 * Returns: None
 */
static void hdd_get_transmit_sta_id(hdd_adapter_t *adapter,
				v_MACADDR_t *dst_addr, uint8_t *station_id)
{
	bool mcbc_addr = false;
	hdd_station_ctx_t *sta_ctx = WLAN_HDD_GET_STATION_CTX_PTR(adapter);

	hdd_get_peer_sta_id(sta_ctx, dst_addr, station_id);
	if (*station_id == HDD_WLAN_INVALID_STA_ID) {
		if (vos_is_macaddr_broadcast(dst_addr) ||
				vos_is_macaddr_group(dst_addr)) {
			hddLog(LOG1,
				"Received MC/BC packet for transmission");
			mcbc_addr = true;
		}
	}

	if (adapter->device_mode == WLAN_HDD_IBSS) {
		/*
		 * This check is necessary to make sure station id is not
		 * overwritten for UC traffic in IBSS mode
		 */
		if (mcbc_addr)
			*station_id = IBSS_BROADCAST_STAID;
	} else if (adapter->device_mode == WLAN_HDD_OCB) {
		*station_id = sta_ctx->conn_info.staId[0];
	} else if (adapter->device_mode == WLAN_HDD_NDI) {
		/*
		 * This check is necessary to make sure station id is not
		 * overwritten for UC traffic in NAN data mode
		 */
		if (mcbc_addr)
			*station_id = sta_ctx->broadcast_staid;
	} else {
		/* For the rest, traffic is directed to AP/P2P GO */
           if (eConnectionState_Associated == sta_ctx->conn_info.connState)
		*station_id = sta_ctx->conn_info.staId[0];
	}
}

/**============================================================================
  @brief hdd_hard_start_xmit() - Function registered with the Linux OS for
  transmitting packets. This version of the function directly passes the packet
  to Transport Layer.

  @param skb      : [in]  pointer to OS packet (sk_buff)
  @param dev      : [in] pointer to network device

  @return         : NET_XMIT_DROP if packets are dropped
                  : NET_XMIT_SUCCESS if packet is enqueued successfully
  ===========================================================================*/
int __hdd_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
   VOS_STATUS status;
   WLANTL_ACEnumType ac;
   sme_QosWmmUpType up;
   hdd_adapter_t *pAdapter =  WLAN_HDD_GET_PRIV_PTR(dev);
   v_BOOL_t granted;
   v_U8_t STAId = WLAN_MAX_STA_COUNT;
   hdd_station_ctx_t *pHddStaCtx = &pAdapter->sessionCtx.station;
   struct sk_buff *skb_next, *list_head = NULL, *list_tail = NULL;
   void *vdev_handle = NULL, *vdev_temp;
   bool is_update_ac_stats = FALSE;
   v_MACADDR_t *pDestMacAddress = NULL;
   hdd_context_t *hddCtxt = WLAN_HDD_GET_CTX(pAdapter);
#ifdef QCA_PKT_PROTO_TRACE
   v_U8_t proto_type = 0;
#endif /* QCA_PKT_PROTO_TRACE */

#ifdef QCA_WIFI_FTM
   if (hdd_get_conparam() == VOS_FTM_MODE) {
       while (skb) {
           skb_next = skb->next;
           kfree_skb(skb);
           skb = skb_next;
       }
       return NETDEV_TX_OK;
   }
#endif

   ++pAdapter->hdd_stats.hddTxRxStats.txXmitCalled;

   if(vos_is_logp_in_progress(VOS_MODULE_ID_VOSS, NULL)) {
       VOS_TRACE( VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_WARN,
            "LOPG in progress, dropping the packet\n");
       goto drop_list;
   }

   while (skb) {
       skb_next = skb->next;
       /* memset skb control block */
       vos_mem_zero(skb->cb, sizeof(skb->cb));
       wlan_hdd_classify_pkt(skb);

       pDestMacAddress = (v_MACADDR_t*)skb->data;
       STAId = HDD_WLAN_INVALID_STA_ID;

       hdd_tsf_record_sk_for_skb(hddCtxt, skb);

/*
* The TCP TX throttling logic is changed a little after 3.19-rc1 kernel,
* the TCP sending limit will be smaller, which will throttle the TCP packets
* to the host driver. The TCP UP LINK throughput will drop heavily.
* In order to fix this issue, need to orphan the socket buffer asap, which will
* call skb's destructor to notify the TCP stack that the SKB buffer is
* unowned. And then the TCP stack will pump more packets to host driver.
*
* The TX packets might be dropped for UDP case in the iperf testing.
* So need to be protected by follow control.
*/
#ifdef QCA_LL_TX_FLOW_CT
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3,19,0))
       //remove if condition for improving SCC TCP TX KPI
      //if (pAdapter->tx_flow_low_watermark > 0) {
          skb_orphan(skb);
     // }
#endif
#else
      /*
       * For PTP feature enabled system, need to orphan the socket buffer asap
       * otherwise the latency will become unacceptable
       */
      if (hdd_cfg_is_ptp_opt_enable(hddCtxt))
          skb_orphan(skb);
#endif

       /* use self peer directly in monitor mode */
       if (VOS_MONITOR_MODE != vos_get_conparam()) {
           hdd_get_transmit_sta_id(pAdapter, pDestMacAddress, &STAId);
           if (STAId == HDD_WLAN_INVALID_STA_ID) {
               hddLog(LOG1, "Invalid station id, transmit operation suspended");
               goto drop_pkt;
           }

           vdev_temp = tlshim_peer_validity(hddCtxt->pvosContext, STAId);
       } else {
           vdev_temp =
               tlshim_selfpeer_vdev(hddCtxt->pvosContext);
       }
       if (!vdev_temp)
           goto drop_pkt;

       vdev_handle = vdev_temp;

#ifdef QCA_LL_TX_FLOW_CT
       if ((pAdapter->hdd_stats.hddTxRxStats.is_txflow_paused != TRUE) &&
            VOS_FALSE ==
              WLANTL_GetTxResource(hddCtxt->pvosContext,
				   pAdapter->sessionId,
				   pAdapter->tx_flow_low_watermark,
				   pAdapter->tx_flow_high_watermark_offset)) {
           hddLog(LOG1, FL("Disabling queues"));
           wlan_hdd_netif_queue_control(pAdapter, WLAN_STOP_ALL_NETIF_QUEUE,
                    WLAN_DATA_FLOW_CONTROL);
           if ((pAdapter->tx_flow_timer_initialized == TRUE) &&
               (VOS_TIMER_STATE_STOPPED ==
                vos_timer_getCurrentState(&pAdapter->tx_flow_control_timer))) {
               vos_timer_start(&pAdapter->tx_flow_control_timer,
                               WLAN_HDD_TX_FLOW_CONTROL_OS_Q_BLOCK_TIME);
               pAdapter->hdd_stats.hddTxRxStats.txflow_timer_cnt++;
               pAdapter->hdd_stats.hddTxRxStats.txflow_pause_cnt++;
               pAdapter->hdd_stats.hddTxRxStats.is_txflow_paused = TRUE;
           }
       }
#endif /* QCA_LL_TX_FLOW_CT */

       /* Get TL AC corresponding to Qdisc queue index/AC */
       ac = hdd_QdiscAcToTlAC[skb->queue_mapping];

       /*
        * user priority from IP header, which is already extracted and set from
        * select_queue call back function
        */
       up = skb->priority;

       ++pAdapter->hdd_stats.hddTxRxStats.txXmitClassifiedAC[ac];
#ifdef HDD_WMM_DEBUG
       VOS_TRACE( VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_FATAL,
                  "%s: Classified as ac %d up %d", __func__, ac, up);
#endif /* HDD_WMM_DEBUG */

       if (HDD_PSB_CHANGED == pAdapter->psbChanged)
       {
           /*
            * Function which will determine acquire admittance for a
            * WMM AC is required or not based on psb configuration done
            * in the framework
            */
           hdd_wmm_acquire_access_required(pAdapter, ac);
       }

       /*
        * Make sure we already have access to this access category
        * or it is EAPOL or WAPI frame during initial authentication which
        * can have artifically boosted higher qos priority.
        */

       if (((pAdapter->psbChanged & (1 << ac)) &&
             likely(
             pAdapter->hddWmmStatus.wmmAcStatus[ac].wmmAcAccessAllowed)) ||
           ((pHddStaCtx->conn_info.uIsAuthenticated == VOS_FALSE) &&
             (ADF_NBUF_GET_IS_EAPOL(skb) || ADF_NBUF_GET_IS_WAPI(skb))))
       {
           granted = VOS_TRUE;
       }
       else
       {
           status = hdd_wmm_acquire_access( pAdapter, ac, &granted );
           pAdapter->psbChanged |= (1 << ac);
       }

       if (!granted) {
           bool isDefaultAc = VOS_FALSE;
           /*
            * ADDTS request for this AC is sent, for now
            * send this packet through next available lower
            * Access category until ADDTS negotiation completes.
            */
           while (!likely(
                   pAdapter->hddWmmStatus.wmmAcStatus[ac].wmmAcAccessAllowed)) {
               switch(ac) {
               case WLANTL_AC_VO:
                    ac = WLANTL_AC_VI;
                    up = SME_QOS_WMM_UP_VI;
                    break;
               case WLANTL_AC_VI:
                    ac = WLANTL_AC_BE;
                    up = SME_QOS_WMM_UP_BE;
                    break;
               case WLANTL_AC_BE:
                    ac = WLANTL_AC_BK;
                    up = SME_QOS_WMM_UP_BK;
                    break;
               default:
                    ac = WLANTL_AC_BK;
                    up = SME_QOS_WMM_UP_BK;
                    isDefaultAc = VOS_TRUE;
                    break;
               }
               if (isDefaultAc)
                   break;
           }
           skb->priority = up;
           skb->queue_mapping = hddLinuxUpToAcMap[up];
       }

#ifdef QCA_PKT_PROTO_TRACE
       if ((hddCtxt->cfg_ini->gEnableDebugLog & VOS_PKT_TRAC_TYPE_EAPOL) ||
           (hddCtxt->cfg_ini->gEnableDebugLog & VOS_PKT_TRAC_TYPE_DHCP))
       {
           proto_type = vos_pkt_get_proto_type(skb,
                        hddCtxt->cfg_ini->gEnableDebugLog, 0);
           switch (proto_type) {
           case VOS_PKT_TRAC_TYPE_EAPOL:
               vos_pkt_trace_buf_update("ST:T:EPL");
               break;
           case VOS_PKT_TRAC_TYPE_DHCP:
               hdd_dhcp_pkt_trace_buf_update(skb, TX_PATH, STA);
               break;
           case VOS_PKT_TRAC_TYPE_ARP:
               vos_pkt_trace_buf_update("ST:T:ARP");
               break;
           case VOS_PKT_TRAC_TYPE_NS:
               vos_pkt_trace_buf_update("ST:T:NS");
               break;
           case VOS_PKT_TRAC_TYPE_NA:
               vos_pkt_trace_buf_update("ST:T:NA");
               break;
           default:
               break;
           }
       }
#endif /* QCA_PKT_PROTO_TRACE */

       pAdapter->stats.tx_bytes += skb->len;
       ++pAdapter->stats.tx_packets;

       if (!list_head) {
           list_head = skb;
           list_tail = skb;
       } else {
           list_tail->next = skb;
           list_tail = list_tail->next;
       }

       adf_dp_trace_log_pkt(pAdapter->sessionId, skb, ADF_TX);
       NBUF_SET_PACKET_TRACK(skb, NBUF_TX_PKT_DATA_TRACK);
       NBUF_UPDATE_TX_PKT_COUNT(skb, NBUF_TX_PKT_HDD);

       adf_dp_trace_set_track(skb, ADF_TX);
       DPTRACE(adf_dp_trace(skb, ADF_DP_TRACE_HDD_TX_PACKET_PTR_RECORD,
                 (uint8_t *)&skb->data, sizeof(skb->data), ADF_TX));
       DPTRACE(adf_dp_trace(skb, ADF_DP_TRACE_HDD_TX_PACKET_RECORD,
                 (uint8_t *)skb->data, adf_nbuf_len(skb), ADF_TX));

       if (adf_nbuf_len(skb) > ADF_DP_TRACE_RECORD_SIZE)
            DPTRACE(adf_dp_trace(skb, ADF_DP_TRACE_HDD_TX_PACKET_RECORD,
                    (uint8_t *)&skb->data[ADF_DP_TRACE_RECORD_SIZE],
                    (adf_nbuf_len(skb) - ADF_DP_TRACE_RECORD_SIZE),
                    ADF_TX));

       skb = skb_next;
       continue;

drop_pkt:
       hdd_drop_skb(pAdapter, skb);
       skb = skb_next;
   } /* end of while */

   if (!vdev_handle) {
       VOS_TRACE(VOS_MODULE_ID_HDD_SAP_DATA, VOS_TRACE_LEVEL_INFO,
                 "%s: All packets dropped in the list", __func__);
       return NETDEV_TX_OK;
   }

   list_tail->next = NULL;

   /*
    * TODO: Should we stop net queues when txrx returns non-NULL?.
    */
   skb = WLANTL_SendSTA_DataFrame(hddCtxt->pvosContext, vdev_handle, list_head
#ifdef QCA_PKT_PROTO_TRACE
                                 , proto_type
#endif /* QCA_PKT_PROTO_TRACE */
                                 );
   if (skb != NULL) {
        VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_WARN,
                  "%s: Failed to send packet to txrx",
                   __func__);
        is_update_ac_stats = TRUE;
        goto drop_list;
   }
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,7,0))
   netif_trans_update(dev);
#else
   dev->trans_start = jiffies;
#endif
   return NETDEV_TX_OK;

drop_list:

   hdd_drop_skb_list(pAdapter, skb, is_update_ac_stats);
   return NETDEV_TX_OK;
}

int hdd_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	int ret;

	vos_ssr_protect(__func__);
	ret = __hdd_hard_start_xmit(skb, dev);
	vos_ssr_unprotect(__func__);
	return ret;
}

/**
 * hdd_get_peer_sta_id() - Get the StationID using the Peer Mac address
 * @sta_ctx: pointer to HDD Station Context
 * @peer_mac_addr: pointer to Peer Mac address
 * @sta_id: pointer to Station Index
 *
 * Returns: VOS_STATUS_SUCCESS on success, VOS_STATUS_E_FAILURE on error
 */
VOS_STATUS hdd_get_peer_sta_id(hdd_station_ctx_t *sta_ctx,
			     v_MACADDR_t *peer_mac_addr,
			     uint8_t *sta_id)
{
	v_U8_t idx;

	for (idx = 0; idx < HDD_MAX_NUM_IBSS_STA; idx++) {
		if (vos_mem_compare(&sta_ctx->conn_info.peerMacAddress[idx],
				peer_mac_addr, sizeof(v_MACADDR_t))) {
			*sta_id = sta_ctx->conn_info.staId[idx];
			return VOS_STATUS_SUCCESS;
		}
	}

	return VOS_STATUS_E_FAILURE;
}

/**
 * hdd_get_peer_idx() - Get the idx for given address in peer table
 * @sta_ctx: pointer to HDD Station Context
 * @addr: pointer to Peer Mac address
 *
 * Return: index when success else INVALID_PEER_IDX
 */
int hdd_get_peer_idx(hdd_station_ctx_t *sta_ctx, v_MACADDR_t *addr)
{
	uint8_t idx;

	for (idx = 0; idx < HDD_MAX_NUM_IBSS_STA; idx++) {
		if (sta_ctx->conn_info.staId[idx] == 0)
			continue;
		if (!vos_mem_compare(&sta_ctx->conn_info.peerMacAddress[idx],
				addr, sizeof(v_MACADDR_t)))
			continue;
		return idx;
	}

	return INVALID_PEER_IDX;
}

/**
 * wlan_display_tx_timeout_stats() - HDD tx timeout stats display handler
 * @adapter: hdd adapter
 *
 * Function called by tx timeout handler to display the stats when timeout
 * occurs during trabsmission.
 *
 * Return: none
 */
void wlan_display_tx_timeout_stats(hdd_adapter_t *adapter)
{
	hdd_context_t *hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	uint8_t pause_bitmap = 0;
	vos_time_t pause_timestamp = 0;
	A_STATUS status;

	VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_INFO,
		  "carrier state: %d", netif_carrier_ok(adapter->dev));

	/* to display the neif queue pause/unpause history */
	wlan_hdd_display_netif_queue_history(hdd_ctx);

	/* to display the count of packets at diferent layers */
	adf_nbuf_tx_desc_count_display();

	/* printing last 100 records from DPTRACE */
	adf_dp_trace_dump_all(100);

	/* to print the pause bitmap of local ll queues */
	status = tlshim_get_ll_queue_pause_bitmap(adapter->sessionId,
			&pause_bitmap, &pause_timestamp);
	if (status != A_OK)
		hddLog(LOGE, FL("vdev is NULL for vdev id %d"),
		       adapter->sessionId);
	else
		hddLog(LOGE,
		       FL("LL vdev queues pause bitmap: %d, last pause timestamp %lu"),
		       pause_bitmap, pause_timestamp);

	/*
	 * To invoke the bug report to flush driver as well as fw logs
	 * when timeout happens. It happens when it has been more
	 * than 5 minutes and the timeout has happened at least 3 times
	 * since the last generation of bug report.
	 */
	adapter->bug_report_count++;
	if (adapter->bug_report_count >= HDD_BUG_REPORT_MIN_COUNT &&
	   (jiffies_to_msecs(jiffies - adapter->last_tx_jiffies) >=
	    HDD_BUG_REPORT_MIN_TIME)) {
		adapter->bug_report_count = 0;
		adapter->last_tx_jiffies = jiffies;
		vos_flush_logs(WLAN_LOG_TYPE_FATAL,
			       WLAN_LOG_INDICATOR_HOST_DRIVER,
			       WLAN_LOG_REASON_HDD_TIME_OUT,
			       DUMP_VOS_TRACE);
	}
}

/**
 * __hdd_tx_timeout() - HDD tx timeout handler
 * @dev: pointer to net_device structure
 *
 * Function called by OS if there is any timeout during transmission.
 * Since HDD simply enqueues packet and returns control to OS right away,
 * this would never be invoked
 *
 * Return: none
 */
static void __hdd_tx_timeout(struct net_device *dev)
{
   hdd_adapter_t *pAdapter =  WLAN_HDD_GET_PRIV_PTR(dev);
   struct netdev_queue *txq;
   int i = 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,7,0))
   hddLog(LOGE, FL("Transmission timeout occurred jiffies %lu"),jiffies);
#else
   hddLog(LOGE, FL("Transmission timeout occurred jiffies %lu trans_start %lu"),
          jiffies, dev->trans_start);
#endif
   DPTRACE(adf_dp_trace(NULL, ADF_DP_TRACE_HDD_TX_TIMEOUT,
                        NULL, 0, ADF_TX));
   /*
    * Getting here implies we disabled the TX queues for too long. Queues are
    * disabled either because of disassociation or low resource scenarios. In
    * case of disassociation it is ok to ignore this. But if associated, we have
    * do possible recovery here.
    */

   VOS_TRACE( VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_INFO,
              "num_bytes AC0: %d AC1: %d AC2: %d AC3: %d",
              pAdapter->wmm_tx_queue[0].count,
              pAdapter->wmm_tx_queue[1].count,
              pAdapter->wmm_tx_queue[2].count,
              pAdapter->wmm_tx_queue[3].count);

   VOS_TRACE( VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_INFO,
              "tx_suspend AC0: %d AC1: %d AC2: %d AC3: %d",
              pAdapter->isTxSuspended[0],
              pAdapter->isTxSuspended[1],
              pAdapter->isTxSuspended[2],
              pAdapter->isTxSuspended[3]);

   for (i = 0; i < NUM_TX_QUEUES; i++) {
      txq = netdev_get_tx_queue(dev, i);
      hddLog(LOGE, FL("Queue%d status: %d txq->trans_start %lu"),
             i, netif_tx_queue_stopped(txq), txq->trans_start);
   }
   wlan_display_tx_timeout_stats(pAdapter);
}

/**
 * hdd_tx_timeout() - Wrapper function to protect __hdd_tx_timeout from SSR
 * @dev: pointer to net_device structure
 *
 * Function called by OS if there is any timeout during transmission.
 * Since HDD simply enqueues packet and returns control to OS right away,
 * this would never be invoked
 *
 * Return: none
 */
void hdd_tx_timeout(struct net_device *dev)
{
	vos_ssr_protect(__func__);
	__hdd_tx_timeout(dev);
	vos_ssr_unprotect(__func__);
}

/**
 * __hdd_stats() - Function registered with the Linux OS for
 *			device TX/RX statistics
 * @dev: pointer to net_device structure
 *
 * Return: pointer to net_device_stats structure
 */
static struct net_device_stats *__hdd_stats(struct net_device *dev)
{
	hdd_adapter_t *pAdapter =  WLAN_HDD_GET_PRIV_PTR(dev);
	return &pAdapter->stats;
}

/**
 * hdd_stats() - SSR wrapper for __hdd_stats
 * @dev: pointer to net_device structure
 *
 * Return: pointer to net_device_stats structure
 */
struct net_device_stats* hdd_stats(struct net_device *dev)
{
	struct net_device_stats *dev_stats;

	vos_ssr_protect(__func__);
	dev_stats = __hdd_stats(dev);
	vos_ssr_unprotect(__func__);

	return dev_stats;
}

/**============================================================================
  @brief hdd_init_tx_rx() - Init function to initialize Tx/RX
  modules in HDD

  @param pAdapter : [in] pointer to adapter context
  @return         : VOS_STATUS_E_FAILURE if any errors encountered
                  : VOS_STATUS_SUCCESS otherwise
  ===========================================================================*/
VOS_STATUS hdd_init_tx_rx( hdd_adapter_t *pAdapter )
{
   VOS_STATUS status = VOS_STATUS_SUCCESS;
   v_SINT_t i = -1;

   if ( NULL == pAdapter )
   {
      VOS_TRACE( VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_ERROR,
              FL("pAdapter is NULL"));
      VOS_ASSERT(0);
      return VOS_STATUS_E_FAILURE;
   }

   pAdapter->isVosOutOfResource = VOS_FALSE;
   pAdapter->isVosLowResource = VOS_FALSE;

   //vos_mem_zero(&pAdapter->stats, sizeof(struct net_device_stats));
   //Will be zeroed out during alloc

   while (++i != NUM_TX_QUEUES)
   {
      pAdapter->isTxSuspended[i] = VOS_FALSE;
      hdd_list_init( &pAdapter->wmm_tx_queue[i], HDD_TX_QUEUE_MAX_LEN);
   }

   return status;
}


/**============================================================================
  @brief hdd_deinit_tx_rx() - Deinit function to clean up Tx/RX
  modules in HDD

  @param pAdapter : [in] pointer to adapter context
  @return         : VOS_STATUS_E_FAILURE if any errors encountered
                  : VOS_STATUS_SUCCESS otherwise
  ===========================================================================*/
VOS_STATUS hdd_deinit_tx_rx( hdd_adapter_t *pAdapter )
{
   VOS_STATUS status = VOS_STATUS_SUCCESS;
   v_SINT_t i = -1;

   if ( NULL == pAdapter )
   {
      VOS_TRACE( VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_ERROR,
              FL("pAdapter is NULL"));
      VOS_ASSERT(0);
      return VOS_STATUS_E_FAILURE;
   }

   status = hdd_flush_tx_queues(pAdapter);
   if (VOS_STATUS_SUCCESS != status)
       VOS_TRACE( VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_WARN,
          FL("failed to flush tx queues"));

   while (++i != NUM_TX_QUEUES)
   {
      //Free up actual list elements in the Tx queue
      hdd_list_destroy( &pAdapter->wmm_tx_queue[i] );
   }

   return status;
}


/**============================================================================
  @brief hdd_disconnect_tx_rx() - Disconnect function to clean up Tx/RX
  modules in HDD

  @param pAdapter : [in] pointer to adapter context
  @return         : VOS_STATUS_E_FAILURE if any errors encountered
                  : VOS_STATUS_SUCCESS otherwise
  ===========================================================================*/
VOS_STATUS hdd_disconnect_tx_rx( hdd_adapter_t *pAdapter )
{
   return hdd_flush_tx_queues(pAdapter);
}


/**============================================================================
  @brief hdd_IsEAPOLPacket() - Checks the packet is EAPOL or not.

  @param pVosPacket : [in] pointer to vos packet
  @return         : VOS_TRUE if the packet is EAPOL
                  : VOS_FALSE otherwise
  ===========================================================================*/

v_BOOL_t hdd_IsEAPOLPacket( vos_pkt_t *pVosPacket )
{
    VOS_STATUS vosStatus  = VOS_STATUS_SUCCESS;
    v_BOOL_t   fEAPOL     = VOS_FALSE;
    void       *pBuffer   = NULL;


    vosStatus = vos_pkt_peek_data( pVosPacket, (v_SIZE_t)HDD_ETHERTYPE_802_1_X_FRAME_OFFSET,
                          &pBuffer, HDD_ETHERTYPE_802_1_X_SIZE );
    if (VOS_IS_STATUS_SUCCESS( vosStatus ) )
    {
       if (pBuffer && vos_be16_to_cpu( *(unsigned short*)pBuffer) == HDD_ETHERTYPE_802_1_X )
       {
          fEAPOL = VOS_TRUE;
       }
    }

   return fEAPOL;
}


#ifdef FEATURE_WLAN_WAPI // Need to update this function
/**============================================================================
  @brief hdd_IsWAIPacket() - Checks the packet is WAI or not.

  @param pVosPacket : [in] pointer to vos packet
  @return         : VOS_TRUE if the packet is WAI
                  : VOS_FALSE otherwise
  ===========================================================================*/

v_BOOL_t hdd_IsWAIPacket( vos_pkt_t *pVosPacket )
{
    VOS_STATUS vosStatus  = VOS_STATUS_SUCCESS;
    v_BOOL_t   fIsWAI     = VOS_FALSE;
    void       *pBuffer   = NULL;

    // Need to update this function
    vosStatus = vos_pkt_peek_data( pVosPacket, (v_SIZE_t)HDD_ETHERTYPE_802_1_X_FRAME_OFFSET,
                          &pBuffer, HDD_ETHERTYPE_802_1_X_SIZE );

    if (VOS_IS_STATUS_SUCCESS( vosStatus ) )
    {
       if (pBuffer && vos_be16_to_cpu( *((unsigned short*)pBuffer)) == HDD_ETHERTYPE_WAI)
       {
          fIsWAI = VOS_TRUE;
       }
    }

   return fIsWAI;
}
#endif /* FEATURE_WLAN_WAPI */

#ifdef IPV6_MCAST_WAR
/*
 * Return TRUE if the packet is to be dropped
 */
static inline
bool drop_ip6_mcast(struct sk_buff *skb)
{
    struct ethhdr *eth;

    eth = eth_hdr(skb);
    if (unlikely(skb->pkt_type == PACKET_MULTICAST)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
       if (unlikely(ether_addr_equal(eth->h_source, skb->dev->dev_addr)))
#else
       if (unlikely(!compare_ether_addr(eth->h_source, skb->dev->dev_addr)))
#endif
            return true;
    }
    return false;
}
#else
#define drop_ip6_mcast(_a) 0
#endif

#ifdef CONFIG_HL_SUPPORT
/*
 * hdd_move_radiotap_header_forward - move radiotap header to head of skb
 * @skb: skb to be modified
 *
 * For HL monitor mode, radiotap is appended to tail when update radiotap
 * info in htt layer. Need to copy it ahead of skb before indicating to OS.
 */
static VOS_STATUS hdd_move_radiotap_header_forward(struct sk_buff *skb)
{
	adf_nbuf_t msdu = (adf_nbuf_t)skb;
	struct ieee80211_radiotap_header *rthdr;
	uint8_t rtap_len;

	if (!adf_nbuf_put_tail(msdu,
		sizeof(struct ieee80211_radiotap_header)))
		return VOS_STATUS_E_NOMEM;
	else {
		rthdr = (struct ieee80211_radiotap_header *)
		(adf_nbuf_data(msdu) + adf_nbuf_len(msdu) -
		sizeof(struct ieee80211_radiotap_header));
		rtap_len = rthdr->it_len;
		if (!adf_nbuf_put_tail(msdu,
			  rtap_len -
			  sizeof(struct ieee80211_radiotap_header)))
			return VOS_STATUS_E_NOMEM;
		else {
			adf_nbuf_push_head(msdu, rtap_len);
			adf_os_mem_copy(adf_nbuf_data(msdu), rthdr, rtap_len);
			adf_nbuf_trim_tail(msdu, rtap_len);
		}
	}
	return VOS_STATUS_SUCCESS;
}
#else
static inline VOS_STATUS hdd_move_radiotap_header_forward(struct sk_buff *skb)
{
    /* no-op */
	return VOS_STATUS_SUCCESS;
}
#endif

/**
 * hdd_mon_rx_packet_cbk() - Receive callback registered with TLSHIM.
 * @vosContext: [in] pointer to VOS context
 * @staId:      [in] Station Id
 * @rxBuf:      [in] pointer to rx adf_nbuf
 *
 * TL will call this to notify the HDD when one or more packets were
 * received for a registered STA.
 * Return: VOS_STATUS_E_FAILURE if any errors encountered, VOS_STATUS_SUCCESS
 * otherwise
 */
VOS_STATUS hdd_mon_rx_packet_cbk(v_VOID_t *vos_ctx, adf_nbuf_t rx_buf,
				 uint8_t sta_id)
{
	hdd_adapter_t *adapter = NULL;
	hdd_context_t *hdd_ctx = NULL;
	int rxstat;
	struct sk_buff *skb = NULL;
	struct sk_buff *skb_next;
	unsigned int cpu_index;
	hdd_adapter_list_node_t *adapter_node = NULL;

	/* Sanity check on inputs */
	if ((NULL == vos_ctx) || (NULL == rx_buf)) {
		VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_ERROR,
			  "%s: Null params being passed", __func__);
		return VOS_STATUS_E_FAILURE;
	}

	hdd_ctx = vos_get_context(VOS_MODULE_ID_HDD, vos_ctx);
	if (NULL == hdd_ctx) {
		VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_ERROR,
			  "%s: HDD adapter context is Null", __func__);
		return VOS_STATUS_E_FAILURE;
	}

	hdd_get_front_adapter(hdd_ctx, &adapter_node);
	adapter = adapter_node->pAdapter;
	if ((NULL == adapter) || (WLAN_HDD_ADAPTER_MAGIC != adapter->magic)) {
		VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_ERROR,
			  "invalid adapter %pK for sta Id %d", adapter, sta_id);
		return VOS_STATUS_E_FAILURE;
	}

	cpu_index = wlan_hdd_get_cpu();

	/* walk the chain until all are processed */
	skb = (struct sk_buff *) rx_buf;
	while (NULL != skb) {
		skb_next = skb->next;
		skb->dev = adapter->dev;

		if(hdd_move_radiotap_header_forward(skb)) {
			skb = skb_next;
			continue;
		}

		++adapter->hdd_stats.hddTxRxStats.rxPackets[cpu_index];
		++adapter->stats.rx_packets;
		adapter->stats.rx_bytes += skb->len;
		/*
		 * If this is not a last packet on the chain
		 * Just put packet into backlog queue, not scheduling RX sirq
		 */
		if (skb->next) {
			rxstat = netif_rx(skb);
		} else {
			/*
			 * This is the last packet on the chain
			 * Scheduling rx sirq
			 */
			rxstat = netif_rx_ni(skb);
		}

		if (NET_RX_SUCCESS == rxstat)
			++adapter->
				hdd_stats.hddTxRxStats.rxDelivered[cpu_index];
		else
			++adapter->hdd_stats.hddTxRxStats.rxRefused[cpu_index];

		skb = skb_next;
	}

	//adapter->dev->last_rx = jiffies;

	return VOS_STATUS_SUCCESS;
}

VOS_STATUS hdd_vir_mon_rx_cbk(v_VOID_t *vos_ctx, adf_nbuf_t rx_buf,
			      uint8_t sta_id)
{
	hdd_adapter_t *adapter = NULL;
	hdd_context_t *hdd_ctx = NULL;
	int rxstat;
	struct sk_buff *skb = NULL;
	struct sk_buff *skb_next;

	/* Sanity check on inputs */
	if ((NULL == vos_ctx) || (NULL == rx_buf)) {
		VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_ERROR,
			  "%s: Null params being passed", __func__);
		return VOS_STATUS_E_FAILURE;
	}

	hdd_ctx = vos_get_context(VOS_MODULE_ID_HDD, vos_ctx);
	if (NULL == hdd_ctx) {
		VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_ERROR,
			  "%s: HDD adapter context is Null", __func__);
		return VOS_STATUS_E_FAILURE;
	}

	adapter = hdd_get_adapter(hdd_ctx, WLAN_HDD_MONITOR);
	if ((NULL == adapter) || (WLAN_HDD_ADAPTER_MAGIC != adapter->magic)) {
		VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_ERROR,
			  "invalid adapter %p for sta Id %d", adapter, sta_id);
		return VOS_STATUS_E_FAILURE;
	}

	/* walk the chain until all are processed */
	skb = (struct sk_buff *)rx_buf;
	while (NULL != skb) {
		skb_next = skb->next;
		skb->dev = adapter->dev;

		/*
		 * If this is not a last packet on the chain
		 * Just put packet into backlog queue, not scheduling RX sirq
		 */
		if (skb->next) {
			rxstat = netif_rx(skb);
		} else {
			/*
			 * This is the last packet on the chain
			 * Scheduling rx sirq
			 */
			rxstat = netif_rx_ni(skb);
		}

		skb = skb_next;
	}

	//adapter->dev->last_rx = jiffies;

	return VOS_STATUS_SUCCESS;
}

/**
 * hdd_is_arp_local() - check if local or non local arp
 * @skb: pointer to sk_buff
 *
 * Return: true if local arp or false otherwise.
 */
static bool hdd_is_arp_local(struct sk_buff *skb)
{

	struct arphdr *arp;
	struct in_ifaddr **ifap = NULL;
	struct in_ifaddr *ifa = NULL;
	struct in_device *in_dev;
	unsigned char *arp_ptr;
	__be32 tip;

	arp = (struct arphdr *)skb->data;
	if (arp->ar_op == htons(ARPOP_REQUEST)) {
		if ((in_dev = __in_dev_get_rtnl(skb->dev)) != NULL) {
			for (ifap = &in_dev->ifa_list; (ifa = *ifap) != NULL;
				ifap = &ifa->ifa_next) {
				if (!strcmp(skb->dev->name,
				    ifa->ifa_label))
					break;
			}
		}

		if (ifa && ifa->ifa_local) {
			arp_ptr = (unsigned char *)(arp + 1);
			arp_ptr += (skb->dev->addr_len + 4 + skb->dev->addr_len);
			memcpy(&tip, arp_ptr, 4);
			hddLog(VOS_TRACE_LEVEL_INFO, "ARP packets: local IP: %x dest IP: %x\n",
					ifa->ifa_local, tip);
			if (ifa->ifa_local != tip)
				return false;
		}
	}
	return true;
}

#ifdef WLAN_FEATURE_TSF_PLUS
static inline void hdd_tsf_timestamp_rx(hdd_context_t *hdd_ctx,
					adf_nbuf_t netbuf,
					uint64_t target_time)
{
	if (!HDD_TSF_IS_RX_SET(hdd_ctx))
		return;

	hdd_rx_timestamp(netbuf, target_time);
}
#else
static inline void hdd_tsf_timestamp_rx(hdd_context_t *hdd_ctx,
					adf_nbuf_t netbuf,
					uint64_t target_time)
{
}
#endif

/**============================================================================
  @brief hdd_rx_packet_cbk() - Receive callback registered with TL.
  TL will call this to notify the HDD when one or more packets were
  received for a registered STA.

  @param vosContext      : [in] pointer to VOS context
  @param staId           : [in] Station Id
  @param rxBuf           : [in] pointer to rx adf_nbuf

  @return                : VOS_STATUS_E_FAILURE if any errors encountered,
                         : VOS_STATUS_SUCCESS otherwise
  ===========================================================================*/
VOS_STATUS hdd_rx_packet_cbk(v_VOID_t *vosContext,
                             adf_nbuf_t rxBuf, v_U8_t staId)
{
   hdd_adapter_t *pAdapter = NULL;
   hdd_context_t *pHddCtx = NULL;
   int rxstat;
   struct sk_buff *skb = NULL;
   struct sk_buff *skb_next;
   unsigned int cpu_index;
#ifdef QCA_PKT_PROTO_TRACE
   v_U8_t proto_type = 0;
#endif /* QCA_PKT_PROTO_TRACE */
   hdd_station_ctx_t *pHddStaCtx = NULL;
   bool wake_lock = false;

   //Sanity check on inputs
   if ((NULL == vosContext) || (NULL == rxBuf))
   {
      VOS_TRACE( VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_ERROR,"%s: Null params being passed", __func__);
      return VOS_STATUS_E_FAILURE;
   }

   pHddCtx = (hdd_context_t *)vos_get_context( VOS_MODULE_ID_HDD, vosContext );
   if ( NULL == pHddCtx )
   {
      VOS_TRACE( VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_ERROR,"%s: HDD adapter context is Null", __func__);
      return VOS_STATUS_E_FAILURE;
   }

   pAdapter = pHddCtx->sta_to_adapter[staId];
   if ((NULL == pAdapter) || (WLAN_HDD_ADAPTER_MAGIC != pAdapter->magic)) {
      hddLog(LOGE, FL("invalid adapter %pK for sta Id %d"), pAdapter, staId);
      return VOS_STATUS_E_FAILURE;
   }

   if (WLAN_HDD_ADAPTER_MAGIC != pAdapter->magic) {
       VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_FATAL,
          "Magic cookie(%x) for adapter sanity verification is invalid",
          pAdapter->magic);
       return VOS_STATUS_E_FAILURE;
   }

   cpu_index = wlan_hdd_get_cpu();

   // walk the chain until all are processed
   skb = (struct sk_buff *) rxBuf;
   pHddStaCtx = WLAN_HDD_GET_STATION_CTX_PTR(pAdapter);
   while (NULL != skb) {
      skb_next = skb->next;

      if (((pHddStaCtx->conn_info.proxyARPService) &&
         cfg80211_is_gratuitous_arp_unsolicited_na(skb)) ||
         vos_is_load_unload_in_progress(VOS_MODULE_ID_VOSS, NULL)) {
            ++pAdapter->hdd_stats.hddTxRxStats.rxDropped[cpu_index];
            VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_INFO,
               "%s: Dropping HS 2.0 Gratuitous ARP or Unsolicited NA"
               " else dropping as Driver load/unload is in progress",
               __func__);
            adf_nbuf_free(skb);
            skb = skb_next;
            continue;
      }

      DPTRACE(adf_dp_trace(skb,
              ADF_DP_TRACE_RX_HDD_PACKET_PTR_RECORD,
              adf_nbuf_data_addr(skb),
              sizeof(adf_nbuf_data(skb)), ADF_RX));
      DPTRACE(adf_dp_trace(skb,
              ADF_DP_TRACE_HDD_RX_PACKET_RECORD,
              (uint8_t *)skb->data, adf_nbuf_len(skb), ADF_RX));

      if (adf_nbuf_len(skb) > ADF_DP_TRACE_RECORD_SIZE)
          DPTRACE(adf_dp_trace(skb,
                  ADF_DP_TRACE_HDD_RX_PACKET_RECORD,
                  (uint8_t *)&skb->data[ADF_DP_TRACE_RECORD_SIZE],
                  (adf_nbuf_len(skb) - ADF_DP_TRACE_RECORD_SIZE),
                  ADF_RX));

#ifdef QCA_PKT_PROTO_TRACE
      if ((pHddCtx->cfg_ini->gEnableDebugLog & VOS_PKT_TRAC_TYPE_EAPOL) ||
          (pHddCtx->cfg_ini->gEnableDebugLog & VOS_PKT_TRAC_TYPE_DHCP)) {
         proto_type = vos_pkt_get_proto_type(skb,
                        pHddCtx->cfg_ini->gEnableDebugLog, 0);
         switch (proto_type) {
         case VOS_PKT_TRAC_TYPE_EAPOL:
             vos_pkt_trace_buf_update("ST:R:EPL");
             break;
         case VOS_PKT_TRAC_TYPE_DHCP:
             hdd_dhcp_pkt_trace_buf_update(skb, RX_PATH, STA);
             break;
         case VOS_PKT_TRAC_TYPE_ARP:
             vos_pkt_trace_buf_update("ST:R:ARP");
             break;
         case VOS_PKT_TRAC_TYPE_NS:
             vos_pkt_trace_buf_update("ST:R:NS");
             break;
         case VOS_PKT_TRAC_TYPE_NA:
             vos_pkt_trace_buf_update("ST:R:NA");
             break;
         default:
             break;
         }
      }
#endif /* QCA_PKT_PROTO_TRACE */

      skb->dev = pAdapter->dev;
      skb->protocol = eth_type_trans(skb, skb->dev);

      /* Check & drop mcast packets (for IPV6) as required */
      if (drop_ip6_mcast(skb)) {
         hddLog(VOS_TRACE_LEVEL_INFO, "MAC Header:");
         VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_INFO,
                            skb_mac_header(skb), 16);
         ++pAdapter->hdd_stats.hddTxRxStats.rxDropped[cpu_index];
         VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_INFO,
                   "%s: Dropping multicast to self NA", __func__);
         adf_nbuf_free(skb);
         skb = skb_next;
         continue;
      }

      ++pAdapter->hdd_stats.hddTxRxStats.rxPackets[cpu_index];
      ++pAdapter->stats.rx_packets;
      pAdapter->stats.rx_bytes += skb->len;

      /**
       * Remove SKB from internal tracking table before submitting it
       * to stack.
       */
      adf_net_buf_debug_release_skb(skb);

      if (!wake_lock) {
         if (skb->protocol == htons(ETH_P_ARP)) {
            if (hdd_is_arp_local(skb))
               wake_lock = true;
         }
         else
            wake_lock = true;
      }

      hdd_tsf_timestamp_rx(pHddCtx, skb, ktime_to_us(skb->tstamp));

      /*
       * If this is not a last packet on the chain
       * Just put packet into backlog queue, not scheduling RX sirq
       */
      if (skb->next) {
         rxstat = netif_rx(skb);
      } else {
          if ((pHddCtx->cfg_ini->rx_wakelock_timeout) &&
              (PACKET_BROADCAST != skb->pkt_type) &&
              (PACKET_MULTICAST != skb->pkt_type))
                wake_lock = true;

          if (wake_lock && pHddStaCtx->conn_info.uIsAuthenticated)
             vos_wake_lock_timeout_acquire(&pHddCtx->rx_wake_lock,
                            pHddCtx->cfg_ini->rx_wakelock_timeout,
                            WIFI_POWER_EVENT_WAKELOCK_HOLD_RX);
          /*
           * This is the last packet on the chain
           * Scheduling rx sirq
           */
          rxstat = netif_rx_ni(skb);
      }

      if (NET_RX_SUCCESS == rxstat)
         ++pAdapter->hdd_stats.hddTxRxStats.rxDelivered[cpu_index];
      else
         ++pAdapter->hdd_stats.hddTxRxStats.rxRefused[cpu_index];

      skb = skb_next;
   }

   //pAdapter->dev->last_rx = jiffies;

   return VOS_STATUS_SUCCESS;
}

/**
 * hdd_reason_type_to_string() - return string conversion of reason type
 * @reason: reason type
 *
 * This utility function helps log string conversion of reason type.
 *
 * Return: string conversion of device mode, if match found;
 *        "Unknown" otherwise.
 */
const char *hdd_reason_type_to_string(enum netif_reason_type reason)
{
	switch (reason) {
	CASE_RETURN_STRING(WLAN_CONTROL_PATH);
	CASE_RETURN_STRING(WLAN_DATA_FLOW_CONTROL);
	default:
		return "Invalid";
	}
}

/**
 * hdd_action_type_to_string() - return string conversion of action type
 * @action: action type
 *
 * This utility function helps log string conversion of action_type.
 *
 * Return: string conversion of device mode, if match found;
 *        "Unknown" otherwise.
 */
const char *hdd_action_type_to_string(enum netif_action_type action)
{
	switch (action) {
	CASE_RETURN_STRING(WLAN_STOP_ALL_NETIF_QUEUE);
	CASE_RETURN_STRING(WLAN_START_ALL_NETIF_QUEUE);
	CASE_RETURN_STRING(WLAN_WAKE_ALL_NETIF_QUEUE);
	CASE_RETURN_STRING(WLAN_STOP_ALL_NETIF_QUEUE_N_CARRIER);
	CASE_RETURN_STRING(WLAN_START_ALL_NETIF_QUEUE_N_CARRIER);
	CASE_RETURN_STRING(WLAN_NETIF_TX_DISABLE);
	CASE_RETURN_STRING(WLAN_NETIF_TX_DISABLE_N_CARRIER);
	CASE_RETURN_STRING(WLAN_NETIF_CARRIER_ON);
	CASE_RETURN_STRING(WLAN_NETIF_CARRIER_OFF);
	default:
		return "Invalid";
	}
}

/**
 * wlan_hdd_update_queue_oper_stats - update queue operation statistics
 * @adapter: adapter handle
 * @action: action type
 * @reason: reason type
 */
static void wlan_hdd_update_queue_oper_stats(hdd_adapter_t *adapter,
	enum netif_action_type action, enum netif_reason_type reason)
{
	switch (action) {
	case WLAN_STOP_ALL_NETIF_QUEUE:
	case WLAN_STOP_ALL_NETIF_QUEUE_N_CARRIER:
	case WLAN_NETIF_TX_DISABLE:
	case WLAN_NETIF_TX_DISABLE_N_CARRIER:
		adapter->queue_oper_stats[reason].pause_count++;
		break;
	case WLAN_START_ALL_NETIF_QUEUE:
	case WLAN_WAKE_ALL_NETIF_QUEUE:
	case WLAN_START_ALL_NETIF_QUEUE_N_CARRIER:
		adapter->queue_oper_stats[reason].unpause_count++;
		break;
	default:
		break;
	}
}

/**
 * wlan_hdd_update_txq_timestamp() - update txq timestamp
 * @dev: net device
 *
 * Return: none
 */
static void wlan_hdd_update_txq_timestamp(struct net_device *dev)
{
	struct netdev_queue *txq;
	int i;
	bool unlock;

	for (i = 0; i < NUM_TX_QUEUES; i++) {
		txq = netdev_get_tx_queue(dev, i);
		unlock = __netif_tx_trylock(txq);
		txq_trans_update(txq);
		if (unlock == true)
			__netif_tx_unlock(txq);
	}
}

/**
 * wlan_hdd_update_unpause_time() - update unpause time
 * @adapter: hdd adapter handle
 *
 * Return: none
 */
static void wlan_hdd_update_unpause_time(hdd_adapter_t *adapter)
{
	adf_os_time_t curr_time = vos_system_ticks();

	adapter->total_unpause_time += curr_time - adapter->last_time;
	adapter->last_time = curr_time;
}

/**
 * wlan_hdd_update_pause_time() - update pause time
 * @adapter: hdd adapter handle
 *
 * Return: none
 */
static void wlan_hdd_update_pause_time(hdd_adapter_t *adapter)
{
	adf_os_time_t curr_time = vos_system_ticks();

	adapter->total_pause_time += curr_time - adapter->last_time;
	adapter->last_time = curr_time;
}

/**
 * wlan_hdd_netif_queue_control() - Use for netif_queue related actions
 * @adapter: adapter handle
 * @action: action type
 * @reason: reason type
 *
 * This is single function which is used for netif_queue related
 * actions like start/stop of network queues and on/off carrier
 * option.
 *
 * Return: None
 */
void wlan_hdd_netif_queue_control(hdd_adapter_t *adapter,
		enum netif_action_type action, enum netif_reason_type reason)
{
	if ((!adapter) || (WLAN_HDD_ADAPTER_MAGIC != adapter->magic) ||
	    (!adapter->dev)) {
		hddLog(LOGE, FL("adapter is invalid"));
		return;
	}
	hddLog(LOG1, FL("action is %d reason is %d"),action,reason);

	switch (action) {
	case WLAN_NETIF_CARRIER_ON:
		netif_carrier_on(adapter->dev);
		break;

	case WLAN_NETIF_CARRIER_OFF:
		netif_carrier_off(adapter->dev);
		break;

	case WLAN_STOP_ALL_NETIF_QUEUE:
		SPIN_LOCK_BH(&adapter->pause_map_lock);
		if (!adapter->pause_map) {
			netif_tx_stop_all_queues(adapter->dev);
			wlan_hdd_update_txq_timestamp(adapter->dev);
			wlan_hdd_update_unpause_time(adapter);
		}
		adapter->pause_map |= (1 << reason);
		SPIN_UNLOCK_BH(&adapter->pause_map_lock);
		break;

	case WLAN_START_ALL_NETIF_QUEUE:
		SPIN_LOCK_BH(&adapter->pause_map_lock);
		adapter->pause_map &= ~(1 << reason);
		if (!adapter->pause_map) {
			netif_tx_start_all_queues(adapter->dev);
			wlan_hdd_update_pause_time(adapter);
		}
		SPIN_UNLOCK_BH(&adapter->pause_map_lock);
		break;

	case WLAN_WAKE_ALL_NETIF_QUEUE:
		SPIN_LOCK_BH(&adapter->pause_map_lock);
		adapter->pause_map &= ~(1 << reason);
		if (!adapter->pause_map) {
			netif_tx_wake_all_queues(adapter->dev);
			wlan_hdd_update_pause_time(adapter);
		}
		SPIN_UNLOCK_BH(&adapter->pause_map_lock);
		break;

	case WLAN_STOP_ALL_NETIF_QUEUE_N_CARRIER:
		SPIN_LOCK_BH(&adapter->pause_map_lock);
		if (!adapter->pause_map) {
			netif_tx_stop_all_queues(adapter->dev);
			wlan_hdd_update_txq_timestamp(adapter->dev);
			wlan_hdd_update_unpause_time(adapter);
		}
		adapter->pause_map |= (1 << reason);
		netif_carrier_off(adapter->dev);
		SPIN_UNLOCK_BH(&adapter->pause_map_lock);
		break;

	case WLAN_START_ALL_NETIF_QUEUE_N_CARRIER:
		SPIN_LOCK_BH(&adapter->pause_map_lock);
		netif_carrier_on(adapter->dev);
		adapter->pause_map &= ~(1 << reason);
		if (!adapter->pause_map) {
			netif_tx_start_all_queues(adapter->dev);
			wlan_hdd_update_pause_time(adapter);
		}
		SPIN_UNLOCK_BH(&adapter->pause_map_lock);
		break;

	case WLAN_NETIF_TX_DISABLE:
		SPIN_LOCK_BH(&adapter->pause_map_lock);
		if (!adapter->pause_map) {
			netif_tx_disable(adapter->dev);
			wlan_hdd_update_txq_timestamp(adapter->dev);
			wlan_hdd_update_unpause_time(adapter);
		}
		adapter->pause_map |= (1 << reason);
		SPIN_UNLOCK_BH(&adapter->pause_map_lock);
		break;

	case WLAN_NETIF_TX_DISABLE_N_CARRIER:
		SPIN_LOCK_BH(&adapter->pause_map_lock);
		if (!adapter->pause_map) {
			netif_tx_disable(adapter->dev);
			wlan_hdd_update_txq_timestamp(adapter->dev);
			wlan_hdd_update_unpause_time(adapter);
		}
		adapter->pause_map |= (1 << reason);
		netif_carrier_off(adapter->dev);
		SPIN_UNLOCK_BH(&adapter->pause_map_lock);
		break;

	default:
		hddLog(LOGE, FL("unsupported netif queue action %d"), action);
	}

	wlan_hdd_update_queue_oper_stats(adapter, action, reason);

	adapter->queue_oper_history[adapter->history_index].time =
						vos_system_ticks();
	adapter->queue_oper_history[adapter->history_index].netif_action =
						action;
	adapter->queue_oper_history[adapter->history_index].netif_reason =
						reason;
	adapter->queue_oper_history[adapter->history_index].pause_map =
						adapter->pause_map;
	if (++adapter->history_index == WLAN_HDD_MAX_HISTORY_ENTRY)
		adapter->history_index = 0;

}

#ifdef QCA_PKT_PROTO_TRACE
/**
 * hdd_dhcp_pkt_trace_buf_update() - Update protocol trace buffer with DHCP
 * packet info.
 * @skb: skb pointer
 * @is_transmission: packet is in transmission or in rx
 * @is_sta: tx/rx by STA mode
 *
 * Return: None
 */
void hdd_dhcp_pkt_trace_buf_update (struct sk_buff *skb, int is_transmission,
				    int is_sta)
{
	char tbuf[20];
	if ((skb->data[DHCP_OPTION53_OFFSET] == DHCP_OPTION53) &&
	   (skb->data[DHCP_OPTION53_LENGTH_OFFSET] ==
	   DHCP_OPTION53_LENGTH)) {

		switch (skb->data[DHCP_OPTION53_STATUS_OFFSET]) {
		case DHCPDISCOVER:
			snprintf(tbuf, sizeof(tbuf),
				"%s:%s:DHCP DIS",
				is_sta?"ST":"HA",
				is_transmission?"T":"R");
			break;
		case DHCPREQUEST:
			snprintf(tbuf, sizeof(tbuf),
				"%s:%s:DHCP REQ",
				is_sta?"ST":"HA",
				is_transmission?"T":"R");
			break;
		case DHCPOFFER:
			snprintf(tbuf, sizeof(tbuf),
				"%s:%s:DHCP OFF",
				is_sta?"ST":"HA",
				is_transmission?"T":"R");
			break;
		case DHCPACK:
			snprintf(tbuf, sizeof(tbuf),
				"%s:%s:DHCP ACK",
				is_sta?"ST":"HA",
				is_transmission?"T":"R");
			break;
		case DHCPNAK:
			snprintf(tbuf, sizeof(tbuf),
				"%s:%s:DHCP NAK",
				is_sta?"ST":"HA",
				is_transmission?"T":"R");
			break;
		case DHCPRELEASE:
			snprintf(tbuf, sizeof(tbuf),
				"%s:%s:DHCP REL",
				is_sta?"ST":"HA",
				is_transmission?"T":"R");
			break;
		case DHCPINFORM:
			snprintf(tbuf, sizeof(tbuf),
				"%s:%s:DHCP INF",
				is_sta?"ST":"HA",
				is_transmission?"T":"R");
			break;
		case DHCPDECLINE:
			snprintf(tbuf, sizeof(tbuf),
				"%s:%s:DHCP DELC",
				is_sta?"ST":"HA",
				is_transmission?"T":"R");
			break;
		default:
			snprintf(tbuf, sizeof(tbuf),
				"%s:%s:DHCP INVL",
				is_sta?"ST":"HA",
				is_transmission?"T":"R");
			break;
		}
		vos_pkt_trace_buf_update(tbuf);
		VOS_TRACE(VOS_MODULE_ID_HDD_DATA, VOS_TRACE_LEVEL_INFO,
			  FL("%s"), tbuf);
	}
}
#endif
#ifdef FEATURE_BUS_BANDWIDTH
/**
 * hdd_rst_tcp_delack() - Reset tcp delack value to original level
 * @hdd_context_t : HDD context
 *
 * This is single function which is used for reseting TCP delack
 * value to its original value.
 *
 * Return: None
 */
void hdd_rst_tcp_delack(hdd_context_t *hdd_ctx)
{
	enum cnss_bus_width_type  next_level = CNSS_BUS_WIDTH_LOW;

	hdd_ctx->rx_high_ind_cnt = 0;
	wlan_hdd_send_svc_nlink_msg(hdd_ctx->radio_index, WLAN_SVC_WLAN_TP_IND,
				&next_level, sizeof(next_level));
}
#else
void hdd_rst_tcp_delack(hdd_context_t *hdd_ctx)
{
	return;
}
#endif /* FEATURE_BUS_BANDWIDTH */
