/*
 * Copyright (c) 2013-2017 The Linux Foundation. All rights reserved.
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

#if !defined(WLAN_HDD_ASSOC_H__)
#define WLAN_HDD_ASSOC_H__
#include <wlan_hdd_mib.h>
#include <sme_Api.h>
#include <linux/ieee80211.h>

#define HDD_MAX_NUM_IBSS_STA          ( 32 )
#ifdef FEATURE_WLAN_TDLS
#define HDD_MAX_NUM_TDLS_STA          ( 8 )
#define HDD_MAX_NUM_TDLS_STA_P_UAPSD_OFFCHAN  ( 1 )
#define TDLS_STA_INDEX_VALID(staId) \
                          (((staId) >= 1) && ((staId) < 0xFF))
#endif
#define TKIP_COUNTER_MEASURE_STARTED 1
#define TKIP_COUNTER_MEASURE_STOPED  0
/* Timeout (in ms) for Link to Up before Registering Station */
#define ASSOC_LINKUP_TIMEOUT 60

/* In pronto case, IBSS owns the first peer for bss peer.
   In Rome case, IBSS uses the 2nd peer as bss peer */
#define IBSS_BROADCAST_STAID 1

/* Timeout in ms for peer info request completion */
#define IBSS_PEER_INFO_REQ_TIMOEUT 1000

#define INVALID_PEER_IDX -1

/**
 * enum eConnectionState - connection state values at HDD
 * @eConnectionState_NotConnected: Not associated in Infra or participating in
 *   an IBSS / Ad-hoc network
 * @eConnectionState_Connecting: While connection in progress
 * @eConnectionState_Associated: Associated in an Infrastructure network
 * @eConnectionState_IbssDisconnected: Participating in an IBSS network though
 *   disconnected (no partner STA in IBSS)
 * @eConnectionState_IbssConnected: Participating in an IBSS network with
 *   partner stations also present
 * @eConnectionState_Disconnecting: Disconnecting in an Infrastructure network.
 * @eConnectionState_NdiDisconnected: NDI in disconnected state - no peers
 * @eConnectionState_NdiConnected: NDI in connected state - at least one peer
 */
typedef enum {
   eConnectionState_NotConnected,
   eConnectionState_Connecting,
   eConnectionState_Associated,
   eConnectionState_IbssDisconnected,
   eConnectionState_IbssConnected,
   eConnectionState_Disconnecting,
   eConnectionState_NdiDisconnected,
   eConnectionState_NdiConnected,
}eConnectionState;

/**
 * struct hdd_conn_flag - connection flags
 * @ht_present: ht element present or not
 * @vht_present: vht element present or not
 * @hs20_present: hs20 element present or not
 * @ht_op_present: ht operation present or not
 * @vht_op_present: vht operation present or not
 */
struct hdd_conn_flag {
	uint8_t ht_present:1;
	uint8_t vht_present:1;
	uint8_t hs20_present:1;
	uint8_t ht_op_present:1;
	uint8_t vht_op_present:1;
	uint8_t reserved:3;
};

/*defines for tx_BF_cap_info */
#define TX_BF_CAP_INFO_TX_BF			0x00000001
#define TX_BF_CAP_INFO_RX_STAG_RED_SOUNDING	0x00000002
#define TX_BF_CAP_INFO_TX_STAG_RED_SOUNDING	0x00000004
#define TX_BF_CAP_INFO_RX_ZFL			0x00000008
#define TX_BF_CAP_INFO_TX_ZFL			0x00000010
#define TX_BF_CAP_INFO_IMP_TX_BF		0x00000020
#define TX_BF_CAP_INFO_CALIBRATION		0x000000c0
#define TX_BF_CAP_INFO_CALIBRATION_SHIFT	6
#define TX_BF_CAP_INFO_EXP_CSIT_BF		0x00000100
#define TX_BF_CAP_INFO_EXP_UNCOMP_STEER_MAT	0x00000200
#define TX_BF_CAP_INFO_EXP_BF_CSI_FB		0x00001c00
#define TX_BF_CAP_INFO_EXP_BF_CSI_FB_SHIFT	10
#define TX_BF_CAP_INFO_EXP_UNCMP_STEER_MAT	0x0000e000
#define TX_BF_CAP_INFO_EXP_UNCMP_STEER_MAT_SHIFT 13
#define TX_BF_CAP_INFO_EXP_CMP_STEER_MAT_FB	0x00070000
#define TX_BF_CAP_INFO_EXP_CMP_STEER_MAT_FB_SHIFT 16
#define TX_BF_CAP_INFO_CSI_NUM_BF_ANT		0x00180000
#define TX_BF_CAP_INFO_CSI_NUM_BF_ANT_SHIFT	18
#define TX_BF_CAP_INFO_UNCOMP_STEER_MAT_BF_ANT	0x00600000
#define TX_BF_CAP_INFO_UNCOMP_STEER_MAT_BF_ANT_SHIFT 20
#define TX_BF_CAP_INFO_COMP_STEER_MAT_BF_ANT	0x01800000
#define TX_BF_CAP_INFO_COMP_STEER_MAT_BF_ANT_SHIFT 22
#define TX_BF_CAP_INFO_RSVD			0xfe000000

/* defines for antenna selection info */
#define ANTENNA_SEL_INFO			0x01
#define ANTENNA_SEL_INFO_EXP_CSI_FB_TX		0x02
#define ANTENNA_SEL_INFO_ANT_ID_FB_TX		0x04
#define ANTENNA_SEL_INFO_EXP_CSI_FB		0x08
#define ANTENNA_SEL_INFO_ANT_ID_FB		0x10
#define ANTENNA_SEL_INFO_RX_AS			0x20
#define ANTENNA_SEL_INFO_TX_SOUNDING_PPDU	0x40
#define ANTENNA_SEL_INFO_RSVD			0x80

/**
 * struct rate_info - bitrate information
 *
 * Information about a receiving or transmitting bitrate
 *
 * @flags: bitflag of flags from &enum rate_info_flags
 * @mcs: mcs index if struct describes a 802.11n bitrate
 * @legacy: bitrate in 100kbit/s for 802.11abg
 * @nss: number of streams (VHT only)
 */
struct rate_info_ex {
	uint8_t flags;
	uint8_t mcs;
	uint16_t legacy;
	uint8_t nss;
};

/**This structure stores the connection information */
typedef struct connection_info_s
{
   /** connection state of the NIC.*/
   eConnectionState connState;

   /** BSS type of the current connection.   Comes from the MIB at the
       time the connect request is issued in combination with the BssDescription
      from the associated entity.*/

   eMib_dot11DesiredBssType connDot11DesiredBssType;
   /** BSSID */
   tCsrBssid bssId;

   /** SSID Info*/
   tCsrSSIDInfo SSID;

   /** Station ID */
   v_U8_t staId[ HDD_MAX_NUM_IBSS_STA ];
   /** Peer Mac Address of the IBSS Stations */
   v_MACADDR_t peerMacAddress[ HDD_MAX_NUM_IBSS_STA ];
   /** Auth Type */
   eCsrAuthType   authType;

   /** Unicast Encryption Type */
   eCsrEncryptionType ucEncryptionType;

   /** Multicast Encryption Type */
   eCsrEncryptionType mcEncryptionType;

   /** Keys */
   tCsrKeys Keys;

   /** Operation Channel  */
   v_U8_t operationChannel;

    /** Remembers authenticated state */
   v_U8_t uIsAuthenticated;

   /** Dot11Mode */
   tANI_U32 dot11Mode;

   v_U8_t proxyARPService;

   /** NSS and RateFlags used for this connection */
   uint8_t   nss;
   uint32_t  rate_flags;

   /** Channel frequency */
   uint32_t freq;

   /** txrate structure holds nss & datarate info */
   struct rate_info_ex txrate;

   /** holds noise information */
   int8_t noise;

   /** holds ht capabilities info */
   struct ieee80211_ht_cap ht_caps;

   /** holds vht capabilities info */
   struct ieee80211_vht_cap vht_caps;

   /** flag conn info params is present or not */
   struct hdd_conn_flag conn_flag;

   /** holds passpoint/hs20 info */
   tDot11fIEhs20vendor_ie hs20vendor_ie;

   /** HT operation info */
   struct ieee80211_ht_operation ht_operation;

   /** VHT operation info */
   struct ieee80211_vht_operation vht_operation;

   /** roaming counter */
   uint32_t roam_count;

   /** holds rssi info */
   int8_t signal;

   /** holds assoc fail reason */
   int32_t assoc_status_code;

   /** holds last SSID info */
   tCsrSSIDInfo last_ssid;

   /** holds last auth type */
   eCsrAuthType last_auth_type;

   /* ptk installed state */
   bool ptk_installed;

   /* gtk installed state */
   bool gtk_installed;
}connection_info_t;

/*Forward declaration of Adapter*/
typedef struct hdd_adapter_s hdd_adapter_t;
typedef struct hdd_context_s hdd_context_t;
typedef struct hdd_station_ctx hdd_station_ctx_t;
typedef struct hdd_ap_ctx_s  hdd_ap_ctx_t;
typedef struct hdd_mon_ctx_s  hdd_mon_ctx_t;

typedef enum
{
   ePeerConnected = 1,
   ePeerDisconnected
}ePeerStatus;

extern v_BOOL_t hdd_connIsConnected( hdd_station_ctx_t *pHddStaCtx );
extern bool hdd_is_connecting(hdd_station_ctx_t *hdd_sta_ctx);
eCsrBand hdd_connGetConnectedBand( hdd_station_ctx_t *pHddStaCtx );
extern eHalStatus hdd_smeRoamCallback( void *pContext, tCsrRoamInfo *pRoamInfo, v_U32_t roamId,
                                eRoamCmdStatus roamStatus, eCsrRoamResult roamResult );

v_BOOL_t hdd_connGetConnectedBssType( hdd_station_ctx_t *pHddCtx,
        eMib_dot11DesiredBssType *pConnectedBssType );

int hdd_SetGENIEToCsr( hdd_adapter_t *pAdapter, eCsrAuthType *RSNAuthType );

int hdd_set_csr_auth_type( hdd_adapter_t *pAdapter, eCsrAuthType RSNAuthType );
VOS_STATUS hdd_roamRegisterTDLSSTA(hdd_adapter_t *pAdapter,
                                   const tANI_U8 *peerMac, tANI_U16 staId,
                                   tANI_U8 ucastSig, uint8_t qos);
void hdd_PerformRoamSetKeyComplete(hdd_adapter_t *pAdapter);

VOS_STATUS hdd_roamDeregisterTDLSSTA(hdd_adapter_t *adapter, uint8_t staId);

#if defined(FEATURE_WLAN_ESE) && defined(FEATURE_WLAN_ESE_UPLOAD)
void hdd_indicateEseBcnReportNoResults(const hdd_adapter_t *pAdapter,
                                       const tANI_U16 measurementToken,
                                       const tANI_BOOLEAN flag,
                                       const tANI_U8 numBss);
#endif /* FEATURE_WLAN_ESE && FEATURE_WLAN_ESE_UPLOAD */

VOS_STATUS hdd_roamRegisterSTA(hdd_adapter_t *adapter, tCsrRoamInfo *roam_info,
			       uint8_t sta_id, v_MACADDR_t *peer_mac_addr,
			       tSirBssDescription *bss_desc);

bool hdd_save_peer(hdd_station_ctx_t *sta_ctx, uint8_t sta_id,
		   v_MACADDR_t *peer_mac_addr);
void hdd_delete_peer(hdd_station_ctx_t *sta_ctx, uint8_t sta_id);

int hdd_get_peer_idx(hdd_station_ctx_t *sta_ctx, v_MACADDR_t *addr);
VOS_STATUS hdd_roamDeregisterSTA(hdd_adapter_t *adapter, uint8_t sta_id);

#endif
