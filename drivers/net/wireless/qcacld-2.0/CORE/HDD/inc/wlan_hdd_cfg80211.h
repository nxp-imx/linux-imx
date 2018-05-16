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

#if !defined( HDD_CFG80211_H__ )
#define HDD_CFG80211_H__


/**===========================================================================

  \file  wlan_hdd_cfg80211.h

  \brief cfg80211 functions declarations

  ==========================================================================*/

/* $HEADER$ */


//value for initial part of frames and number of bytes to be compared
#define GAS_INITIAL_REQ "\x04\x0a"
#define GAS_INITIAL_REQ_SIZE 2

#define GAS_INITIAL_RSP "\x04\x0b"
#define GAS_INITIAL_RSP_SIZE 2

#define GAS_COMEBACK_REQ "\x04\x0c"
#define GAS_COMEBACK_REQ_SIZE 2

#define GAS_COMEBACK_RSP "\x04\x0d"
#define GAS_COMEBACK_RSP_SIZE 2

#define P2P_PUBLIC_ACTION_FRAME "\x04\x09\x50\x6f\x9a\x09"
#define P2P_PUBLIC_ACTION_FRAME_SIZE 6

#define P2P_ACTION_FRAME "\x7f\x50\x6f\x9a\x09"
#define P2P_ACTION_FRAME_SIZE 5

#define SA_QUERY_FRAME_REQ "\x08\x00"
#define SA_QUERY_FRAME_REQ_SIZE 2

#define SA_QUERY_FRAME_RSP "\x08\x01"
#define SA_QUERY_FRAME_RSP_SIZE 2

#define HDD_P2P_WILDCARD_SSID "DIRECT-" //TODO Put it in proper place;
#define HDD_P2P_WILDCARD_SSID_LEN 7

#define WNM_BSS_ACTION_FRAME "\x0a\x07"
#define WNM_BSS_ACTION_FRAME_SIZE 2

#define WNM_NOTIFICATION_FRAME "\x0a\x1a"
#define WNM_NOTIFICATION_FRAME_SIZE 2

#define WPA_OUI_TYPE   "\x00\x50\xf2\x01"
#define BLACKLIST_OUI_TYPE   "\x00\x50\x00\x00"
#define WHITELIST_OUI_TYPE   "\x00\x50\x00\x01"
#define WPA_OUI_TYPE_SIZE  4
#define WMM_OUI_TYPE   "\x00\x50\xf2\x02\x01"
#define WMM_OUI_TYPE_SIZE  5

#define WLAN_BSS_MEMBERSHIP_SELECTOR_VHT_PHY 126
#define WLAN_BSS_MEMBERSHIP_SELECTOR_HT_PHY 127
#define BASIC_RATE_MASK   0x80
#define RATE_MASK         0x7f

#ifdef WLAN_ENABLE_AGEIE_ON_SCAN_RESULTS
/* GPS application requirement */
#define QCOM_VENDOR_IE_ID 221
#define QCOM_OUI1         0x00
#define QCOM_OUI2         0xA0
#define QCOM_OUI3         0xC6
#define QCOM_VENDOR_IE_AGE_TYPE  0x100
#define QCOM_VENDOR_IE_AGE_LEN   (sizeof(qcom_ie_age) - 2)

#ifdef FEATURE_WLAN_TDLS
#define WLAN_IS_TDLS_SETUP_ACTION(action) \
         ((SIR_MAC_TDLS_SETUP_REQ <= action) && (SIR_MAC_TDLS_SETUP_CNF >= action))
#if !defined (TDLS_MGMT_VERSION2)
#define TDLS_MGMT_VERSION2 0
#endif
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)) \
            || defined(BACKPORTED_CHANNEL_SWITCH_PRESENT)
#define CHANNEL_SWITCH_SUPPORTED
#endif


#define MAX_CHANNEL (NUM_2_4GHZ_CHANNELS + NUM_5GHZ_CHANNELS)

#define IS_CHANNEL_VALID(channel) ((channel >= 0 && channel < 15) \
                     || (channel >= 36 && channel <= 184))

typedef struct {
   u8 element_id;
   u8 len;
   u8 oui_1;
   u8 oui_2;
   u8 oui_3;
   u32 type;
   u32 age;
   u32 tsf_delta;
}__attribute__((packed)) qcom_ie_age ;
#endif

#ifndef NL80211_AUTHTYPE_FILS_SK
#define NL80211_AUTHTYPE_FILS_SK 5
#endif
#ifndef NL80211_AUTHTYPE_FILS_SK_PFS
#define NL80211_AUTHTYPE_FILS_SK_PFS 6
#endif
#ifndef NL80211_AUTHTYPE_FILS_PK
#define NL80211_AUTHTYPE_FILS_PK 7
#endif
#ifndef WLAN_AKM_SUITE_FILS_SHA256
#define WLAN_AKM_SUITE_FILS_SHA256 0x000FAC0E
#endif
#ifndef WLAN_AKM_SUITE_FILS_SHA384
#define WLAN_AKM_SUITE_FILS_SHA384 0x000FAC0F
#endif
#ifndef WLAN_AKM_SUITE_FT_FILS_SHA256
#define WLAN_AKM_SUITE_FT_FILS_SHA256 0x000FAC10
#endif
#ifndef WLAN_AKM_SUITE_FT_FILS_SHA384
#define WLAN_AKM_SUITE_FT_FILS_SHA384 0x000FAC11
#endif

/* Vendor id to be used in vendor specific command and events
 * to user space.
 * NOTE: The authoritative place for definition of QCA_NL80211_VENDOR_ID,
 * vendor subcmd definitions prefixed with QCA_NL80211_VENDOR_SUBCMD, and
 * qca_wlan_vendor_attr is open source file src/common/qca-vendor.h in
 * git://w1.fi/srv/git/hostap.git; the values here are just a copy of that
 */

#define QCA_NL80211_VENDOR_ID                          0x001374
#define MAX_REQUEST_ID         0xFFFFFFFF

enum qca_nl80211_vendor_subcmds {
    QCA_NL80211_VENDOR_SUBCMD_UNSPEC = 0,
    QCA_NL80211_VENDOR_SUBCMD_TEST = 1,
    /* subcmds 2..8 not yet allocated */
    QCA_NL80211_VENDOR_SUBCMD_ROAMING = 9,
    QCA_NL80211_VENDOR_SUBCMD_AVOID_FREQUENCY = 10,
    QCA_NL80211_VENDOR_SUBCMD_DFS_CAPABILITY =  11,
    QCA_NL80211_VENDOR_SUBCMD_NAN =  12,
    QCA_NL80211_VENDOR_SUBCMD_STATS_EXT = 13,
    /* subcommands for link layer statistics start here */
    QCA_NL80211_VENDOR_SUBCMD_LL_STATS_SET = 14,
    QCA_NL80211_VENDOR_SUBCMD_LL_STATS_GET = 15,
    QCA_NL80211_VENDOR_SUBCMD_LL_STATS_CLR = 16,
    QCA_NL80211_VENDOR_SUBCMD_LL_STATS_RADIO_RESULTS = 17,
    QCA_NL80211_VENDOR_SUBCMD_LL_STATS_IFACE_RESULTS = 18,
    QCA_NL80211_VENDOR_SUBCMD_LL_STATS_PEERS_RESULTS = 19,
    /* subcommands for extscan start here */
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_START = 20,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_STOP = 21,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_GET_VALID_CHANNELS = 22,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_GET_CAPABILITIES = 23,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_GET_CACHED_RESULTS = 24,
    /* Used when report_threshold is reached in scan cache. */
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SCAN_RESULTS_AVAILABLE = 25,
    /* Used to report scan results when each probe rsp. is received,
     * if report_events enabled in wifi_scan_cmd_params.
     */
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_FULL_SCAN_RESULT = 26,
    /* Indicates progress of scanning state-machine. */
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SCAN_EVENT = 27,
    /* Indicates BSSID Hotlist. */
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_HOTLIST_AP_FOUND = 28,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SET_BSSID_HOTLIST = 29,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_RESET_BSSID_HOTLIST = 30,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SIGNIFICANT_CHANGE = 31,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SET_SIGNIFICANT_CHANGE = 32,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_RESET_SIGNIFICANT_CHANGE = 33,
    /* EXT TDLS */
    QCA_NL80211_VENDOR_SUBCMD_TDLS_ENABLE = 34,
    QCA_NL80211_VENDOR_SUBCMD_TDLS_DISABLE = 35,
    QCA_NL80211_VENDOR_SUBCMD_TDLS_GET_STATUS = 36,
    QCA_NL80211_VENDOR_SUBCMD_TDLS_STATE = 37,
    /* Get supported features */
    QCA_NL80211_VENDOR_SUBCMD_GET_SUPPORTED_FEATURES = 38,

    /* Set scanning_mac_oui */
    QCA_NL80211_VENDOR_SUBCMD_SCANNING_MAC_OUI = 39,
    /* Set nodfs_flag */
    QCA_NL80211_VENDOR_SUBCMD_NO_DFS_FLAG = 40,

    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_HOTLIST_AP_LOST = 41,

    /* Get Concurrency Matrix */
    QCA_NL80211_VENDOR_SUBCMD_GET_CONCURRENCY_MATRIX = 42,

    /* Get the security keys for key management offload */
    QCA_NL80211_VENDOR_SUBCMD_KEY_MGMT_SET_KEY = 50,

    /* Send the roaming and authentication info after roaming */
    QCA_NL80211_VENDOR_SUBCMD_KEY_MGMT_ROAM_AUTH = 51,

    QCA_NL80211_VENDOR_SUBCMD_APFIND = 52,

    /* Deprecated */
    QCA_NL80211_VENDOR_SUBCMD_OCB_SET_SCHED = 53,

    QCA_NL80211_VENDOR_SUBCMD_DO_ACS = 54,

    /* Get the supported features by the driver */
    QCA_NL80211_VENDOR_SUBCMD_GET_FEATURES = 55,

    /* Off loaded DFS events */
    QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_STARTED = 56,
    QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_FINISHED = 57,
    QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_ABORTED = 58,
    QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_NOP_FINISHED = 59,
    QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_RADAR_DETECTED = 60,

    /* Get Wifi Specific Info */
    QCA_NL80211_VENDOR_SUBCMD_GET_WIFI_INFO = 61,
    /* Start Wifi Logger */
    QCA_NL80211_VENDOR_SUBCMD_WIFI_LOGGER_START = 62,

    /* FW Memory Dump feature is deprecated */

    QCA_NL80211_VENDOR_SUBCMD_ROAM = 64,

    /*
     * APIs corresponding to the sub commands 65-68 are deprecated.
     * These sub commands are reserved and not supposed to be used
     * for any other purpose
     */
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_PNO_SET_LIST = 69,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_PNO_SET_PASSPOINT_LIST = 70,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_PNO_RESET_PASSPOINT_LIST = 71,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_PNO_NETWORK_FOUND = 72,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_PNO_PASSPOINT_NETWORK_FOUND = 73,

    /* Wi-Fi Configuration subcommands */
    QCA_NL80211_VENDOR_SUBCMD_SET_WIFI_CONFIGURATION = 74,
    QCA_NL80211_VENDOR_SUBCMD_GET_WIFI_CONFIGURATION = 75,

    QCA_NL80211_VENDOR_SUBCMD_GET_LOGGER_FEATURE_SET = 76,
    QCA_NL80211_VENDOR_SUBCMD_GET_RING_DATA = 77,
    QCA_NL80211_VENDOR_SUBCMD_TDLS_GET_CAPABILITIES = 78,

    QCA_NL80211_VENDOR_SUBCMD_OFFLOADED_PACKETS = 79,
    QCA_NL80211_VENDOR_SUBCMD_MONITOR_RSSI = 80,
    QCA_NL80211_VENDOR_SUBCMD_NDP = 81,


    /* NS Offload enable/disable cmd */
    QCA_NL80211_VENDOR_SUBCMD_ND_OFFLOAD = 82,

	QCA_NL80211_VENDOR_SUBCMD_PACKET_FILTER = 83,
	QCA_NL80211_VENDOR_SUBCMD_GET_BUS_SIZE = 84,

	QCA_NL80211_VENDOR_SUBCMD_GET_WAKE_REASON_STATS = 85,

	/* OCB commands */
	QCA_NL80211_VENDOR_SUBCMD_OCB_SET_CONFIG = 92,
	QCA_NL80211_VENDOR_SUBCMD_OCB_SET_UTC_TIME = 93,
	QCA_NL80211_VENDOR_SUBCMD_OCB_START_TIMING_ADVERT = 94,
	QCA_NL80211_VENDOR_SUBCMD_OCB_STOP_TIMING_ADVERT = 95,
	QCA_NL80211_VENDOR_SUBCMD_OCB_GET_TSF_TIMER = 96,
	QCA_NL80211_VENDOR_SUBCMD_DCC_GET_STATS = 97,
	QCA_NL80211_VENDOR_SUBCMD_DCC_CLEAR_STATS = 98,
	QCA_NL80211_VENDOR_SUBCMD_DCC_UPDATE_NDL = 99,
	QCA_NL80211_VENDOR_SUBCMD_DCC_STATS_EVENT = 100,

	/* subcommand to get link properties */
	QCA_NL80211_VENDOR_SUBCMD_LINK_PROPERTIES = 101,
	QCA_NL80211_VENDOR_SUBCMD_SETBAND = 105,
	QCA_NL80211_VENDOR_SUBCMD_SET_TXPOWER_SCALE = 109,
	QCA_NL80211_VENDOR_SUBCMD_SET_TXPOWER_SCALE_DECR_DB = 115,
	QCA_NL80211_VENDOR_SUBCMD_ACS_POLICY = 116,
	QCA_NL80211_VENDOR_SUBCMD_STA_CONNECT_ROAM_POLICY = 117,
	QCA_NL80211_VENDOR_SUBCMD_SET_SAP_CONFIG  = 118,
	QCA_NL80211_VENDOR_SUBCMD_GET_STATION = 121,

	/* subcommand for link layer statistics extension */
	QCA_NL80211_VENDOR_SUBCMD_LL_STATS_EXT = 127,
	/* subcommand to get chain rssi value */
	QCA_NL80211_VENDOR_SUBCMD_GET_CHAIN_RSSI = 138,
	QCA_NL80211_VENDOR_SUBCMD_CHIP_PWRSAVE_FAILURE = 148,
	/* subcommand to flush peer tids */
	QCA_NL80211_VENDOR_SUBCMD_PEER_FLUSH_PENDING  = 162,
};

/**
 * enum qca_wlan_802_11_mode - dot11 mode
 * @QCA_WLAN_802_11_MODE_INVALID: Invalid dot11 mode
 * @QCA_WLAN_802_11_MODE_11A: mode A
 * @QCA_WLAN_802_11_MODE_11B: mode B
 * @QCA_WLAN_802_11_MODE_11G: mode G
 * @QCA_WLAN_802_11_MODE_11N: mode N
 * @QCA_WLAN_802_11_MODE_11AC: mode AC
 */
enum qca_wlan_802_11_mode {
	QCA_WLAN_802_11_MODE_INVALID,
	QCA_WLAN_802_11_MODE_11A,
	QCA_WLAN_802_11_MODE_11B,
	QCA_WLAN_802_11_MODE_11G,
	QCA_WLAN_802_11_MODE_11N,
	QCA_WLAN_802_11_MODE_11AC,
};

/**
 * enum qca_wlan_auth_type - Authentication key management type
 * @QCA_WLAN_AUTH_TYPE_INVALID: Invalid key management type
 * @QCA_WLAN_AUTH_TYPE_OPEN: Open key
 * @QCA_WLAN_AUTH_TYPE_SHARED: shared key
 * @QCA_WLAN_AUTH_TYPE_WPA: wpa key
 * @QCA_WLAN_AUTH_TYPE_WPA_PSK: wpa psk key
 * @QCA_WLAN_AUTH_TYPE_WPA_NONE: wpa none key
 * @QCA_WLAN_AUTH_TYPE_RSN: rsn key
 * @QCA_WLAN_AUTH_TYPE_RSN_PSK: rsn psk key
 * @QCA_WLAN_AUTH_TYPE_FT: ft key
 * @QCA_WLAN_AUTH_TYPE_FT_PSK: ft psk key
 * @QCA_WLAN_AUTH_TYPE_SHA256: shared 256 key
 * @QCA_WLAN_AUTH_TYPE_SHA256_PSK: shared 256 psk
 * @QCA_WLAN_AUTH_TYPE_WAI: wai key
 * @QCA_WLAN_AUTH_TYPE_WAI_PSK wai psk key
 * @QCA_WLAN_AUTH_TYPE_CCKM_WPA: cckm wpa key
 * @QCA_WLAN_AUTH_TYPE_CCKM_RSN: cckm rsn key
 */
enum qca_wlan_auth_type {
	QCA_WLAN_AUTH_TYPE_INVALID,
	QCA_WLAN_AUTH_TYPE_OPEN,
	QCA_WLAN_AUTH_TYPE_SHARED,
	QCA_WLAN_AUTH_TYPE_WPA,
	QCA_WLAN_AUTH_TYPE_WPA_PSK,
	QCA_WLAN_AUTH_TYPE_WPA_NONE,
	QCA_WLAN_AUTH_TYPE_RSN,
	QCA_WLAN_AUTH_TYPE_RSN_PSK,
	QCA_WLAN_AUTH_TYPE_FT,
	QCA_WLAN_AUTH_TYPE_FT_PSK,
	QCA_WLAN_AUTH_TYPE_SHA256,
	QCA_WLAN_AUTH_TYPE_SHA256_PSK,
	QCA_WLAN_AUTH_TYPE_WAI,
	QCA_WLAN_AUTH_TYPE_WAI_PSK,
	QCA_WLAN_AUTH_TYPE_CCKM_WPA,
	QCA_WLAN_AUTH_TYPE_CCKM_RSN,
	QCA_WLAN_AUTH_TYPE_AUTOSWITCH,
};

enum qca_nl80211_vendor_subcmds_index {
#if defined(FEATURE_WLAN_CH_AVOID) || defined(FEATURE_WLAN_FORCE_SAP_SCC)
    QCA_NL80211_VENDOR_SUBCMD_AVOID_FREQUENCY_INDEX = 0,
#endif /* FEATURE_WLAN_CH_AVOID || FEATURE_WLAN_FORCE_SAP_SCC */

#ifdef WLAN_FEATURE_NAN
    QCA_NL80211_VENDOR_SUBCMD_NAN_INDEX,
#endif /* WLAN_FEATURE_NAN */

#ifdef WLAN_FEATURE_STATS_EXT
    QCA_NL80211_VENDOR_SUBCMD_STATS_EXT_INDEX,
#endif /* WLAN_FEATURE_STATS_EXT */

#ifdef FEATURE_WLAN_EXTSCAN
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_START_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_STOP_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_GET_CAPABILITIES_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_GET_CACHED_RESULTS_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SCAN_RESULTS_AVAILABLE_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_FULL_SCAN_RESULT_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SCAN_EVENT_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_HOTLIST_AP_FOUND_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SET_BSSID_HOTLIST_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_RESET_BSSID_HOTLIST_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SIGNIFICANT_CHANGE_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SET_SIGNIFICANT_CHANGE_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_RESET_SIGNIFICANT_CHANGE_INDEX,
#endif /* FEATURE_WLAN_EXTSCAN */

#ifdef WLAN_FEATURE_LINK_LAYER_STATS
    QCA_NL80211_VENDOR_SUBCMD_LL_STATS_SET_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_LL_STATS_GET_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_LL_STATS_CLR_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_LL_RADIO_STATS_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_LL_IFACE_STATS_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_LL_PEER_INFO_STATS_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_LL_STATS_EXT_INDEX,
#endif /* WLAN_FEATURE_LINK_LAYER_STATS */
    /* EXT TDLS */
    QCA_NL80211_VENDOR_SUBCMD_TDLS_STATE_CHANGE_INDEX,
    /* ACS OBSS Coex*/
    QCA_NL80211_VENDOR_SUBCMD_DO_ACS_INDEX,
#ifdef WLAN_FEATURE_ROAM_OFFLOAD
    QCA_NL80211_VENDOR_SUBCMD_KEY_MGMT_ROAM_AUTH_INDEX,
#endif
    /* DFS */
    QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_STARTED_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_FINISHED_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_ABORTED_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_NOP_FINISHED_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_RADAR_DETECTED_INDEX,
#ifdef FEATURE_WLAN_EXTSCAN
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_HOTLIST_AP_LOST_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_PNO_NETWORK_FOUND_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_PNO_PASSPOINT_NETWORK_FOUND_INDEX,
#endif /* FEATURE_WLAN_EXTSCAN */

    /* OCB events */
    QCA_NL80211_VENDOR_SUBCMD_DCC_STATS_EVENT_INDEX,
    QCA_NL80211_VENDOR_SUBCMD_MONITOR_RSSI_INDEX,
#ifdef WLAN_FEATURE_NAN_DATAPATH
    QCA_NL80211_VENDOR_SUBCMD_NDP_INDEX,
#endif /* WLAN_FEATURE_NAN_DATAPATH */
    QCA_NL80211_VENDOR_SUBCMD_PWR_SAVE_FAIL_DETECTED_INDEX,
};

/* EXT TDLS */
enum qca_wlan_vendor_attr_tdls_enable {
    QCA_WLAN_VENDOR_ATTR_TDLS_ENABLE_INVALID = 0,
    /* An array of 6 x Unsigned 8-bit value */
    QCA_WLAN_VENDOR_ATTR_TDLS_ENABLE_MAC_ADDR,
    /* signed 32-bit value, but lets keep as unsigned for now */
    QCA_WLAN_VENDOR_ATTR_TDLS_ENABLE_CHANNEL,
    QCA_WLAN_VENDOR_ATTR_TDLS_ENABLE_GLOBAL_OPERATING_CLASS,
    QCA_WLAN_VENDOR_ATTR_TDLS_ENABLE_MAX_LATENCY_MS,
    QCA_WLAN_VENDOR_ATTR_TDLS_ENABLE_MIN_BANDWIDTH_KBPS,
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_TDLS_ENABLE_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_TDLS_ENABLE_MAX =
         QCA_WLAN_VENDOR_ATTR_TDLS_ENABLE_AFTER_LAST - 1,
};

enum qca_wlan_vendor_attr_tdls_disable {
    QCA_WLAN_VENDOR_ATTR_TDLS_DISABLE_INVALID = 0,
    /* An array of 6 x Unsigned 8-bit value */
    QCA_WLAN_VENDOR_ATTR_TDLS_DISABLE_MAC_ADDR,
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_TDLS_DISABLE_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_TDLS_DISABLE_MAX =
       QCA_WLAN_VENDOR_ATTR_TDLS_DISABLE_AFTER_LAST - 1,
};

enum qca_wlan_vendor_attr_tdls_get_status {
    QCA_WLAN_VENDOR_ATTR_TDLS_GET_STATUS_INVALID = 0,
    /* An array of 6 x Unsigned 8-bit value */
    QCA_WLAN_VENDOR_ATTR_TDLS_GET_STATUS_MAC_ADDR,
    /* signed 32-bit value, but lets keep as unsigned for now */
    QCA_WLAN_VENDOR_ATTR_TDLS_GET_STATUS_STATE,
    QCA_WLAN_VENDOR_ATTR_TDLS_GET_STATUS_REASON,
    QCA_WLAN_VENDOR_ATTR_TDLS_GET_STATUS_CHANNEL,
    QCA_WLAN_VENDOR_ATTR_TDLS_GET_STATUS_GLOBAL_OPERATING_CLASS,
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_TDLS_GET_STATUS_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_TDLS_GET_STATUS_MAX =
      QCA_WLAN_VENDOR_ATTR_TDLS_GET_STATUS_AFTER_LAST - 1,
};

enum qca_wlan_vendor_attr_tdls_state {
    QCA_WLAN_VENDOR_ATTR_TDLS_STATE_INVALID = 0,
    /* An array of 6 x Unsigned 8-bit value */
    QCA_WLAN_VENDOR_ATTR_TDLS_STATE_MAC_ADDR,
    /* signed 32-bit value, but lets keep as unsigned for now */
    QCA_WLAN_VENDOR_ATTR_TDLS_NEW_STATE,
    QCA_WLAN_VENDOR_ATTR_TDLS_STATE_REASON,
    QCA_WLAN_VENDOR_ATTR_TDLS_STATE_CHANNEL,
    QCA_WLAN_VENDOR_ATTR_TDLS_STATE_GLOBAL_OPERATING_CLASS,
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_TDLS_STATE_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_TDLS_STATE_MAX =
        QCA_WLAN_VENDOR_ATTR_TDLS_STATE_AFTER_LAST - 1,
};

/* enum's to provide TDLS capabilites */
enum qca_wlan_vendor_attr_get_tdls_capabilities {
	QCA_WLAN_VENDOR_ATTR_TDLS_GET_CAPS_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_TDLS_GET_CAPS_MAX_CONC_SESSIONS = 1,
	QCA_WLAN_VENDOR_ATTR_TDLS_GET_CAPS_FEATURES_SUPPORTED = 2,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_TDLS_GET_CAPS_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_TDLS_GET_CAPS_MAX =
		QCA_WLAN_VENDOR_ATTR_TDLS_GET_CAPS_AFTER_LAST - 1,
};

enum qca_wlan_vendor_attr {
    QCA_WLAN_VENDOR_ATTR_INVALID = 0,
    /* used by QCA_NL80211_VENDOR_SUBCMD_DFS_CAPABILITY */
    QCA_WLAN_VENDOR_ATTR_DFS = 1,
    /* used by QCA_NL80211_VENDOR_SUBCMD_NAN */
    QCA_WLAN_VENDOR_ATTR_NAN = 2,
    /* used by QCA_NL80211_VENDOR_SUBCMD_STATS_EXT */
    QCA_WLAN_VENDOR_ATTR_STATS_EXT = 3,
    /* used by QCA_NL80211_VENDOR_SUBCMD_STATS_EXT */
    QCA_WLAN_VENDOR_ATTR_IFINDEX = 4,

    /* used by QCA_NL80211_VENDOR_SUBCMD_ROAMING */
    QCA_WLAN_VENDOR_ATTR_ROAMING_POLICY = 5,
    QCA_WLAN_VENDOR_ATTR_MAC_ADDR = 6,

    /* used by QCA_NL80211_VENDOR_SUBCMD_GET_FEATURES */
    QCA_WLAN_VENDOR_ATTR_FEATURE_FLAGS = 7,

    /* Unsigned 32-bit value from enum qca_set_band */
    QCA_WLAN_VENDOR_ATTR_SETBAND_VALUE = 12,

    /* used by QCA_NL80211_VENDOR_SUBCMD_GET_CHAIN_RSSI */
    QCA_WLAN_VENDOR_ATTR_CHAIN_INDEX = 26,
    QCA_WLAN_VENDOR_ATTR_CHAIN_RSSI = 27,

    /* Used in QCA_NL80211_VENDOR_SUBCMD_STATS_EXT command
     * to report frame aggregation statistics to userspace.
     */
    QCA_WLAN_VENDOR_ATTR_RX_AGGREGATION_STATS_HOLES_NUM = 34,
    QCA_WLAN_VENDOR_ATTR_RX_AGGREGATION_STATS_HOLES_INFO = 35,

    /* Used in QCA_NL80211_VENDOR_SUBCMD_GET_CHAIN_RSSI command
     * to report the corresponding antenna index to the chain rssi value
     */
    QCA_WLAN_VENDOR_ATTR_ANTENNA_INFO = 40,

    /* keep last */
    QCA_WLAN_VENDOR_ATTR_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_MAX =
        QCA_WLAN_VENDOR_ATTR_AFTER_LAST - 1
};

#ifdef FEATURE_WLAN_EXTSCAN
enum qca_wlan_vendor_attr_extscan_config_params
{
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SUBCMD_CONFIG_PARAM_INVALID = 0,

    /* Unsigned 32-bit value; Middleware provides it to the driver. Middle ware
     * either gets it from caller, e.g., framework, or generates one if
     * framework doesn't provide it.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SUBCMD_CONFIG_PARAM_REQUEST_ID,

    /* NL attributes for data used by
     * QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_GET_VALID_CHANNELS sub command.
     */
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_GET_VALID_CHANNELS_CONFIG_PARAM_WIFI_BAND,
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_GET_VALID_CHANNELS_CONFIG_PARAM_MAX_CHANNELS,

    /* NL attributes for input params used by
     * QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_START sub command.
     */

    /* Unsigned 32-bit value; channel frequency */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_CHANNEL_SPEC_CHANNEL,
    /* Unsigned 32-bit value; dwell time in ms. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_CHANNEL_SPEC_DWELL_TIME,
    /* Unsigned 8-bit value; 0: active; 1: passive; N/A for DFS */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_CHANNEL_SPEC_PASSIVE,
    /* Unsigned 8-bit value; channel class */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_CHANNEL_SPEC_CLASS,

    /* Unsigned 8-bit value; bucket index, 0 based */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC_INDEX,
    /* Unsigned 8-bit value; band. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC_BAND,
    /* Unsigned 32-bit value; desired period, in ms. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC_PERIOD,
    /* Unsigned 8-bit value; report events semantics. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC_REPORT_EVENTS,
    /* Unsigned 32-bit value.
     * Followed by a nested array of EXTSCAN_CHANNEL_SPEC_* attributes.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC_NUM_CHANNEL_SPECS,

    /* Array of QCA_WLAN_VENDOR_ATTR_EXTSCAN_CHANNEL_SPEC_* attributes.
     * Array size: QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC_NUM_CHANNEL_SPECS
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_CHANNEL_SPEC,

    /* Unsigned 32-bit value; base timer period in ms. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SCAN_CMD_PARAMS_BASE_PERIOD,
    /* Unsigned 32-bit value; number of APs to store in each scan in the
     * BSSID/RSSI history buffer (keep the highest RSSI APs).
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SCAN_CMD_PARAMS_MAX_AP_PER_SCAN,
    /* Unsigned 8-bit value; in %, when scan buffer is this much full, wake up
     * APPS.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SCAN_CMD_PARAMS_REPORT_THRESHOLD_PERCENT,
    /* Unsigned 8-bit value; number of scan bucket specs; followed by a nested
     * array of_EXTSCAN_BUCKET_SPEC_* attributes and values. The size of the
     * array is determined by NUM_BUCKETS.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SCAN_CMD_PARAMS_NUM_BUCKETS,

    /* Array of QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC_* attributes.
     * Array size: QCA_WLAN_VENDOR_ATTR_EXTSCAN_SCAN_CMD_PARAMS_NUM_BUCKETS
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC,

    /* Unsigned 8-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_GET_CACHED_SCAN_RESULTS_CONFIG_PARAM_FLUSH,
    /* Unsigned 32-bit value; maximum number of results to be returned. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_GET_CACHED_SCAN_RESULTS_CONFIG_PARAM_MAX,

    /* An array of 6 x Unsigned 8-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_AP_THRESHOLD_PARAM_BSSID,
    /* Signed 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_AP_THRESHOLD_PARAM_RSSI_LOW,
    /* Signed 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_AP_THRESHOLD_PARAM_RSSI_HIGH,
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_AP_THRESHOLD_PARAM_CHANNEL,

    /* Number of hotlist APs as unsigned 32-bit value, followed by a nested
     * array of AP_THRESHOLD_PARAM attributes and values. The size of the
     * array is determined by NUM_AP.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_BSSID_HOTLIST_PARAMS_NUM_AP,

    /* Array of QCA_WLAN_VENDOR_ATTR_EXTSCAN_AP_THRESHOLD_PARAM_* attributes.
     * Array size: QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC_NUM_CHANNEL_SPECS
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_AP_THRESHOLD_PARAM,

    /* Unsigned 32bit value; number of samples for averaging RSSI. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SIGNIFICANT_CHANGE_PARAMS_RSSI_SAMPLE_SIZE,
    /* Unsigned 32bit value; number of samples to confirm AP loss. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SIGNIFICANT_CHANGE_PARAMS_LOST_AP_SAMPLE_SIZE,
    /* Unsigned 32bit value; number of APs breaching threshold. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SIGNIFICANT_CHANGE_PARAMS_MIN_BREACHING,
    /* Unsigned 32bit value; number of APs. Followed by an array of
     * AP_THRESHOLD_PARAM attributes. Size of the array is NUM_AP.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SIGNIFICANT_CHANGE_PARAMS_NUM_AP,
    /* Unsigned 32bit value; number of samples to confirm AP loss. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_BSSID_HOTLIST_PARAMS_LOST_AP_SAMPLE_SIZE,

    /* Unsigned 32-bit value. If max_period is non zero or different than
     * period, then this bucket is an exponential backoff bucket.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC_MAX_PERIOD,
    /* Unsigned 32-bit value. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC_BASE,
    /* Unsigned 32-bit value. For exponential back off bucket, number of scans
     * to performed for a given period.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_BUCKET_SPEC_STEP_COUNT,
    /* Unsigned 8-bit value; in number of scans, wake up AP after these
     * many scans.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SCAN_CMD_PARAMS_REPORT_THRESHOLD_NUM_SCANS,

    /* NL attributes for data used by
     * QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SET_SSID_HOTLIST sub command.
     */
    /* Unsigned 32bit value; number of samples to confirm SSID loss. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SSID_HOTLIST_PARAMS_LOST_SSID_SAMPLE_SIZE,
    /* Number of hotlist SSIDs as unsigned 32-bit value, followed by a nested
     * array of SSID_THRESHOLD_PARAM_* attributes and values. The size of the
     * array is determined by NUM_SSID.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SSID_HOTLIST_PARAMS_NUM_SSID,
    /* Array of QCA_WLAN_VENDOR_ATTR_EXTSCAN_SSID_THRESHOLD_PARAM_* attributes.
     * Array size: QCA_WLAN_VENDOR_ATTR_EXTSCAN_SSID_HOTLIST_PARAMS_NUM_SSID
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SSID_THRESHOLD_PARAM,

    /* An array of 33 x Unsigned 8-bit value; NULL terminated SSID */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SSID_THRESHOLD_PARAM_SSID,
    /* Unsigned 8-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SSID_THRESHOLD_PARAM_BAND,
    /* Signed 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SSID_THRESHOLD_PARAM_RSSI_LOW,
    /* Signed 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SSID_THRESHOLD_PARAM_RSSI_HIGH,

    /* Unsigned 32-bit value; a bitmask w/additional extscan config flag. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_CONFIGURATION_FLAGS,

    /* keep last */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SUBCMD_CONFIG_PARAM_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_SUBCMD_CONFIG_PARAM_MAX =
        QCA_WLAN_VENDOR_ATTR_EXTSCAN_SUBCMD_CONFIG_PARAM_AFTER_LAST - 1,
};

enum qca_wlan_vendor_attr_extscan_results
{
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_INVALID = 0,

    /* Unsigned 32-bit value; must match the request Id supplied by Wi-Fi HAL
      * in the corresponding subcmd NL msg
      */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_REQUEST_ID,

    /* Unsigned 32-bit value; used to indicate the status response from
      * firmware/driver for the vendor sub-command.
      */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_STATUS,

    /* EXTSCAN Valid Channels attributes */
    /* Unsigned 32bit value; followed by a nested array of CHANNELS.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_NUM_CHANNELS,
    /* An array of NUM_CHANNELS x Unsigned 32bit value integers representing
     * channel numbers
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CHANNELS,

    /* EXTSCAN Capabilities attributes */
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_SCAN_CACHE_SIZE,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_SCAN_BUCKETS,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_AP_CACHE_PER_SCAN,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_RSSI_SAMPLE_SIZE,
    /* Signed 32bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_SCAN_REPORTING_THRESHOLD,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_HOTLIST_BSSIDS,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_SIGNIFICANT_WIFI_CHANGE_APS,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_BSSID_HISTORY_ENTRIES,

    /* EXTSCAN Attributes used with
     * QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SCAN_RESULTS_AVAILABLE sub-command.
     */

    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_NUM_RESULTS_AVAILABLE,


    /* EXTSCAN attributes used with
     * QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_FULL_SCAN_RESULT sub-command.
     */

    /* An array of NUM_RESULTS_AVAILABLE x
     * QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_*
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_LIST,

    /* Unsigned 64-bit value; age of sample at the time of retrieval */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_TIME_STAMP,
    /* 33 x unsigned 8-bit value; NULL terminated SSID */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_SSID,
    /* An array of 6 x Unsigned 8-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_BSSID,
    /* Unsigned 32-bit value; channel frequency in MHz */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_CHANNEL,
    /* Signed 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_RSSI,
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_RTT,
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_RTT_SD,
    /* Unsigned 16-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_BEACON_PERIOD,
    /* Unsigned 16-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_CAPABILITY,
    /* Unsigned 32-bit value; size of the IE DATA blob */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_IE_LENGTH,
    /* An array of IE_LENGTH x Unsigned 8-bit value; blob of all the
     * information elements found in the beacon; this data should be a
     * packed list of wifi_information_element objects, one after the other.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_IE_DATA,
    /* Unsigned 8-bit value; set by driver to indicate more scan results are
     * available.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_RESULT_MORE_DATA,

    /* Use attr QCA_WLAN_VENDOR_ATTR_EXTSCAN_NUM_RESULTS_AVAILABLE
     * to indicate number of wifi scan results/bssids retrieved by the scan.
     * Also, use QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_LIST to indicate the list
     * of wifi scan results returned for each cached result block.
     */

    /* EXTSCAN attributes for
     * QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SCAN_EVENT sub-command.
     */
    /* Unsigned 8-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_EVENT_TYPE,
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SCAN_EVENT_STATUS,

    /* EXTSCAN attributes for
     * QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_HOTLIST_AP_FOUND sub-command.
     */
    /* Use attr QCA_WLAN_VENDOR_ATTR_EXTSCAN_NUM_RESULTS_AVAILABLE
     * to indicate number of results.
     */

    /* EXTSCAN attributes for
     * QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_SIGNIFICANT_CHANGE sub-command.
     */
    /* An array of 6 x Unsigned 8-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SIGNIFICANT_CHANGE_RESULT_BSSID,
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SIGNIFICANT_CHANGE_RESULT_CHANNEL,
    /* Unsigned 32-bit value.
      */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SIGNIFICANT_CHANGE_RESULT_NUM_RSSI,
    /* A nested array of signed 32-bit RSSI values. Size of the array is
     * determined by (NUM_RSSI of SIGNIFICANT_CHANGE_RESULT_NUM_RSSI.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_SIGNIFICANT_CHANGE_RESULT_RSSI_LIST,

    /* EXTSCAN attributes used with
     * QCA_NL80211_VENDOR_SUBCMD_EXTSCAN_GET_CACHED_RESULTS sub-command.
     */
    /* Use attr QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_NUM_RESULTS_AVAILABLE
     * to indicate number of gscan cached results returned.
     * Also, use QCA_WLAN_VENDOR_ATTR_EXTSCAN_CACHED_RESULTS_LIST to indicate
     *  the list of gscan cached results.
     */

    /* An array of NUM_RESULTS_AVAILABLE x
     * QCA_NL80211_VENDOR_ATTR_EXTSCAN_CACHED_RESULTS_*
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_CACHED_RESULTS_LIST,
    /* Unsigned 32-bit value; a unique identifier for the scan unit. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_CACHED_RESULTS_SCAN_ID,
    /* Unsigned 32-bit value; a bitmask w/additional information about scan. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_CACHED_RESULTS_FLAGS,
    /* Use attr QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_NUM_RESULTS_AVAILABLE
     * to indicate number of wifi scan results/bssids retrieved by the scan.
     * Also, use QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_LIST to indicate the list
     * of wifi scan results returned for each cached result block.
     */

    /* EXTSCAN attributes for
     * QCA_NL80211_VENDOR_SUBCMD_PNO_NETWORK_FOUND sub-command.
     */
    /* Use QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_NUM_RESULTS_AVAILABLE for number
     * of results.
     * Use QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_LIST to indicate the nested
     * list of wifi scan results returned for each wifi_passpoint_match_result block.
     * Array size: QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_NUM_RESULTS_AVAILABLE.
     */

    /* EXTSCAN attributes for
     * QCA_NL80211_VENDOR_SUBCMD_PNO_PASSPOINT_NETWORK_FOUND sub-command.
     */
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_PNO_RESULTS_PASSPOINT_NETWORK_FOUND_NUM_MATCHES,
    /* A nested array of
     * QCA_WLAN_VENDOR_ATTR_EXTSCAN_PNO_RESULTS_PASSPOINT_MATCH_*
     * attributes. Array size =
     * *_ATTR_EXTSCAN_PNO_RESULTS_PASSPOINT_NETWORK_FOUND_NUM_MATCHES.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_PNO_RESULTS_PASSPOINT_MATCH_RESULT_LIST,

    /* Unsigned 32-bit value; network block id for the matched network */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_PNO_RESULTS_PASSPOINT_MATCH_ID,
    /* Use QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_LIST to indicate the nested
     * list of wifi scan results returned for each wifi_passpoint_match_result block.
     */
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_PNO_RESULTS_PASSPOINT_MATCH_ANQP_LEN,
    /* An array size of PASSPOINT_MATCH_ANQP_LEN of unsigned 8-bit values;
     * ANQP data in the information_element format.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_PNO_RESULTS_PASSPOINT_MATCH_ANQP,

    /* Unsigned 32bit value; a EXTSCAN Capabilities attribute. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_HOTLIST_SSIDS,
    /* Unsigned 32bit value; a EXTSCAN Capabilities attribute. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_NUM_EPNO_NETS,
    /* Unsigned 32bit value; a EXTSCAN Capabilities attribute. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_NUM_EPNO_NETS_BY_SSID,
    /* Unsigned 32bit value; a EXTSCAN Capabilities attribute. */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_CAPABILITIES_MAX_NUM_WHITELISTED_SSID,

    /* Use attr QCA_WLAN_VENDOR_ATTR_EXTSCAN_NUM_RESULTS_AVAILABLE
     * to indicate number of results.
     */
    /* Unsigned 32bit value, Bit mask of all buckets scanned in the
     * current EXTSCAN CYCLE. For e.g. If fw scan is going to scan
     * following buckets 0, 1, 2 in current cycle then it will be
     * (0x111).
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_BUCKETS_SCANNED,

    /*
     * Unsigned 32bit value; a EXTSCAN Capabilities attribute to send maximum
     * number of blacklist bssid's that firmware can support.
     */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_MAX_NUM_BLACKLISTED_BSSID,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_PAD,
#endif
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_MAX =
        QCA_WLAN_VENDOR_ATTR_EXTSCAN_RESULTS_AFTER_LAST - 1,
};

#endif

#ifdef WLAN_FEATURE_LINK_LAYER_STATS

enum qca_wlan_vendor_attr_ll_stats_set
{
    QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_INVALID = 0,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_CONFIG_MPDU_SIZE_THRESHOLD = 1,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_CONFIG_AGGRESSIVE_STATS_GATHERING,
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_MAX =
    QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_AFTER_LAST - 1
};

enum qca_wlan_vendor_attr_ll_stats_get
{
    QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_INVALID = 0,
    /* Unsigned 32bit value provided by the caller issuing the GET stats
     * command. When reporting the stats results, the driver uses the same
     * value to indicate which GET request the results correspond to.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_CONFIG_REQ_ID,
    /* Unsigned 34bit value - bit mask to identify what
     * statistics are requested for retrieval.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_CONFIG_REQ_MASK,
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_MAX =
        QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_AFTER_LAST - 1
};

enum qca_wlan_vendor_attr_ll_stats_clr
{
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_INVALID = 0,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_REQ_MASK,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_STOP_REQ,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_RSP_MASK,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_STOP_RSP,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_MAX =
        QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_AFTER_LAST - 1
};

/**
 * enum qca_wlan_vendor_attr_ll_stats_results_type - ll stats result type
 *
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_INVALID: Initial invalid value
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_RADIO: Link layer stats type radio
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_IFACE: Link layer stats type interface
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_PEER: Link layer stats type peer
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_AFTER_LAST: Last value
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_MAX: Max value
 */
enum qca_wlan_vendor_attr_ll_stats_results_type {
	QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_INVALID = 0,

	QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_RADIO = 1,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_IFACE,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_PEER,

	QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_MAX =
		QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_AFTER_LAST - 1
};

enum qca_wlan_vendor_attr_ll_stats_results
{
    QCA_WLAN_VENDOR_ATTR_LL_STATS_INVALID = 0,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RESULTS_REQ_ID = 1,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_BEACON_RX,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_MGMT_RX,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_MGMT_ACTION_RX,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_MGMT_ACTION_TX,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_RSSI_MGMT,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_RSSI_DATA,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_RSSI_ACK,
    /* Attributes of type QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_* are
     * nested within the interface stats.
     */

    /* Interface mode, e.g., STA, SOFTAP, IBSS, etc.
     * Type = enum wifi_interface_mode */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_MODE,
    /* Interface MAC address. An array of 6 Unsigned int8 */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_MAC_ADDR,
    /* Type = enum wifi_connection_state,
     * e.g., DISCONNECTED, AUTHENTICATING, etc.
     * valid for STA, CLI only.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_STATE,
    /* Type = enum wifi_roam_state. Roaming state,
     * e.g., IDLE or ACTIVE (is that valid for STA only?)
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_ROAMING,
    /* Unsigned 32bit value. WIFI_CAPABILITY_XXX */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_CAPABILITIES,
    /* NULL terminated SSID. An array of 33 Unsigned 8bit values */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_SSID,
    /* BSSID. An array of 6 Unsigned 8bit values */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_BSSID,
    /* Country string advertised by AP. An array of 3 Unsigned 8bit values */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_AP_COUNTRY_STR,
    /* Country string for this association. An array of 3 Unsigned 8bit values*/
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_COUNTRY_STR,

    /* Attributes of type QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_* could
     * be nested within the interface stats.
     */

    /* Type = enum wifi_traffic_ac, e.g., V0, VI, BE and BK */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_AC,
    /* Unsigned int 32 value corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_TX_MPDU,
    /* Unsigned int 32 value corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RX_MPDU,
    /* Unsigned int 32 value corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_TX_MCAST,
    /* Unsigned int 32 value corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RX_MCAST,
    /* Unsigned int 32 value corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RX_AMPDU,
    /* Unsigned int 32 value corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_TX_AMPDU,
    /* Unsigned int 32 value corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_MPDU_LOST,
    /* Unsigned int 32 value corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RETRIES,
    /* Unsigned int 32 value corresponding to respective AC  */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RETRIES_SHORT,
    /* Unsigned int 32 values corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RETRIES_LONG,
    /* Unsigned int 32 values corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_CONTENTION_TIME_MIN,
    /* Unsigned int 32 values corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_CONTENTION_TIME_MAX,
    /* Unsigned int 32 values corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_CONTENTION_TIME_AVG,
    /* Unsigned int 32 values corresponding to respective AC */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_CONTENTION_NUM_SAMPLES,
    /* Unsigned 32bit value. Number of peers */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_NUM_PEERS,

    /* Attributes of type QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO_* are
     * nested within the interface stats.
     */

    /* Type = enum wifi_peer_type. Peer type, e.g., STA, AP, P2P GO etc. */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO_TYPE,
    /* MAC addr corresponding to respective peer.
     *  An array of 6 Unsigned 8bit values.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO_MAC_ADDRESS,
    /* Unsigned int 32bit value representing capabilities
     * corresponding to respective peer.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO_CAPABILITIES,
    /* Unsigned 32bit value. Number of rates */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO_NUM_RATES,

    /* Attributes nested within the rate stats.*/
    /* Unsigned 8bit value */
    /* Unsigned int 8bit value; 0: OFDM, 1:CCK, 2:HT 3:VHT 4..7 reserved */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_PREAMBLE,
    /* Unsigned int 8bit value; 0:1x1, 1:2x2, 3:3x3, 4:4x4 */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_NSS,
    /* Unsigned int 8bit value; 0:20MHz, 1:40Mhz, 2:80Mhz, 3:160Mhz */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_BW,
    /* Unsigned int 8bit value; OFDM/CCK rate code would be as per IEEE Std
     * in the units of 0.5mbps HT/VHT it would be mcs index */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_MCS_INDEX,

    /* Unsigned 32bit value. Bit rate in units of 100Kbps */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_BIT_RATE,

    /* Attributes of type QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_* could be
     * nested within the peer info stats.
     */

    /* Unsigned int 32bit value. Number of successfully transmitted data pkts,
     * i.e., with ACK received  *corresponding to the respective rate.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_TX_MPDU,
    /* Unsigned int 32bit value. Number of received data pkts
     * corresponding to the respective rate. */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_RX_MPDU,
    /* Unsigned int 32bit value. Number of data pkts losses, i.e.,
     * no ACK received corresponding to *the respective rate.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_MPDU_LOST,
    /* Unsigned int 32bit value. Total number of data pkt retries for
     *   the respective rate.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_RETRIES,
    /* Unsigned int 32bit value. Total number of short data pkt retries for
      the respective rate. */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_RETRIES_SHORT,
    /* Unsigned int 32bit value. Total number of long data pkt retries for
     * the respective rate.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_RETRIES_LONG,

    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ID,
    /* Unsigned 32bit value. Total number of msecs the radio is awake
     *  accruing over time.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME,
    /* Unsigned 32bit value. Total number of msecs the radio is
     * transmitting accruing over time.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_TX_TIME,
    /* Unsigned 32bit value. Total number of msecs the radio is
     * in active receive accruing over time.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_RX_TIME,
    /* Unsigned 32bit value. Total number of msecs the radio is
     * awake due to all scan accruing over time.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_SCAN,
    /* Unsigned 32bit value. Total number of msecs the radio is
     *   awake due to NAN accruing over time.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_NBD,
    /* Unsigned 32bit value. Total number of msecs the radio is
     * awake due to GSCAN accruing over time.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_GSCAN,
    /* Unsigned 32bit value. Total number of msecs the radio is
     * awake due to roam scan accruing over time.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_ROAM_SCAN,
    /* Unsigned 32bit value. Total number of msecs the radio is
     * awake due to PNO scan accruing over time.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_PNO_SCAN,
    /* Unsigned 32bit value. Total number of msecs the radio is
     * awake due to HS2.0 scans and GAS exchange accruing over time.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_HS20,
    /* Unsigned 32bit value. Number of channels. */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_NUM_CHANNELS,

    /* Attributes of type QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_INFO_
     * could be nested within the channel stats.
     */

    /* Type = enum wifi_channel_width. Channel width, e.g., 20, 40, 80, etc.*/
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_INFO_WIDTH,
    /* Unsigned 32bit value. Primary 20MHz channel. */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_INFO_CENTER_FREQ,
    /* Unsigned 32bit value. Center frequency (MHz) first segment. */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_INFO_CENTER_FREQ0,
    /* Unsigned 32bit value. Center frequency (MHz) second segment. */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_INFO_CENTER_FREQ1,

    /* Attributes of type QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_ could be
     * nested within the radio stats.
     */

    /* Unsigned int 32bit value representing total number of msecs the radio
     * s awake on that *channel accruing over time, corresponding to
     * the respective channel.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_ON_TIME,
    /* Unsigned int 32bit value representing total number of msecs the
     * CCA register is busy accruing  *over time corresponding to the
     * respective channel.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_CCA_BUSY_TIME,

    QCA_WLAN_VENDOR_ATTR_LL_STATS_NUM_RADIOS,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_CH_INFO,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO_RATE_INFO,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_INFO,

    /* Unsigned 8bit value. Used by the driver; if set to 1, it indicates that
     * more stats, e.g., peers or radio, are to follow in the next
     * QCA_NL80211_VENDOR_SUBCMD_LL_STATS_*_RESULTS event.
     * Otherwise, it is set to 0.
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RESULTS_MORE_DATA,

    /* Unsigned 64bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_AVERAGE_TSF_OFFSET,

    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_LEAKY_AP_DETECTED,

    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_LEAKY_AP_AVG_NUM_FRAMES_LEAKED,

    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_LEAKY_AP_GUARD_TIME,

    /* Unsigned 32bit value to indicate ll stats result type */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE,

    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_NUM_TX_LEVELS,

    /* Unsigned 32bit value
     * Number of msecs the radio spent in transmitting for each power level
     */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_TX_TIME_PER_LEVEL,

    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_RTS_SUCC_CNT,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_RTS_FAIL_CNT,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_PPDU_SUCC_CNT,
    /* Unsigned 32bit value */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_PPDU_FAIL_CNT,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
    QCA_WLAN_VENDOR_ATTR_LL_STATS_PAD,
#endif
    QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_PENDING_MSDU = 83,
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_LL_STATS_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_LL_STATS_MAX =
    QCA_WLAN_VENDOR_ATTR_LL_STATS_AFTER_LAST -1
};

#endif /* WLAN_FEATURE_LINK_LAYER_STATS */

enum qca_wlan_vendor_attr_get_supported_features {
    QCA_WLAN_VENDOR_ATTR_FEATURE_SET_INVALID = 0,
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_FEATURE_SET = 1,
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_FEATURE_SET_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_FEATURE_SET_MAX =
        QCA_WLAN_VENDOR_ATTR_FEATURE_SET_AFTER_LAST - 1,
};

enum qca_wlan_vendor_attr_set_scanning_mac_oui {
    QCA_WLAN_VENDOR_ATTR_SET_SCANNING_MAC_OUI_INVALID = 0,
    /* An array of 3 x Unsigned 8-bit value */
    QCA_WLAN_VENDOR_ATTR_SET_SCANNING_MAC_OUI = 1,
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_SET_SCANNING_MAC_OUI_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_SET_SCANNING_MAC_OUI_MAX =
        QCA_WLAN_VENDOR_ATTR_SET_SCANNING_MAC_OUI_AFTER_LAST - 1,
};

enum qca_wlan_vendor_attr_set_no_dfs_flag
{
    QCA_WLAN_VENDOR_ATTR_SET_NO_DFS_FLAG_INVALID = 0,
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_SET_NO_DFS_FLAG = 1,
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_SET_NO_DFS_FLAG_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_SET_NO_DFS_FLAG_MAX =
    QCA_WLAN_VENDOR_ATTR_SET_NO_DFS_FLAG_AFTER_LAST - 1,
};

/**
 * enum qca_wlan_vendor_attr_roam_auth - vendor event for roaming
 * @QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_BSSID: BSSID of the roamed AP
 * @QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_REQ_IE: Request IE
 * @QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_RESP_IE: Response IE
 * @QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_AUTHORIZED: Authorization Status
 * @QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_KEY_REPLAY_CTR: Replay Counter
 * @QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_PTK_KCK: KCK of the PTK
 * @QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_PTK_KEK: KEK of the PTK
 */
enum qca_wlan_vendor_attr_roam_auth {
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_BSSID,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_REQ_IE,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_RESP_IE,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_AUTHORIZED,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_KEY_REPLAY_CTR,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_PTK_KCK,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_PTK_KEK,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_MAX =
		QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_AFTER_LAST - 1
};

/* NL attributes for data used by
 * QCA_NL80211_VENDOR_SUBCMD_GET_CONCURRENCY_MATRIX sub command.
 */
enum qca_wlan_vendor_attr_get_concurrency_matrix {
    QCA_WLAN_VENDOR_ATTR_GET_CONCURRENCY_MATRIX_INVALID = 0,
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_GET_CONCURRENCY_MATRIX_CONFIG_PARAM_SET_SIZE_MAX = 1,
    /* Unsigned 32-bit value */
    QCA_WLAN_VENDOR_ATTR_GET_CONCURRENCY_MATRIX_RESULTS_SET_SIZE = 2,
    /* An array of SET_SIZE x Unsigned 32bit values representing
     * concurrency combinations.
     */
    QCA_WLAN_VENDOR_ATTR_GET_CONCURRENCY_MATRIX_RESULTS_SET = 3,
    /* keep last */
    QCA_WLAN_VENDOR_ATTR_GET_CONCURRENCY_MATRIX_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_GET_CONCURRENCY_MATRIX_MAX =
        QCA_WLAN_VENDOR_ATTR_GET_CONCURRENCY_MATRIX_AFTER_LAST - 1,
};

enum qca_wlan_epno_type {
	QCA_WLAN_EPNO,
	QCA_WLAN_PNO
};

enum qca_wlan_vendor_attr_pno_config_params {
	QCA_WLAN_VENDOR_ATTR_PNO_INVALID = 0,
	/* NL attributes for data used by
	 * QCA_NL80211_VENDOR_SUBCMD_PNO_SET_PASSPOINT_LIST sub command.
	 */
	/* Unsigned 32-bit value */
	QCA_WLAN_VENDOR_ATTR_PNO_PASSPOINT_LIST_PARAM_NUM = 1,
	/* Array of nested QCA_WLAN_VENDOR_ATTR_PNO_PASSPOINT_NETWORK_PARAM_*
	 * attributes. Array size =
	 * QCA_WLAN_VENDOR_ATTR_PNO_PASSPOINT_LIST_PARAM_NUM.
	 */
	QCA_WLAN_VENDOR_ATTR_PNO_PASSPOINT_LIST_PARAM_NETWORK_ARRAY = 2,

	/* Unsigned 32-bit value */
	QCA_WLAN_VENDOR_ATTR_PNO_PASSPOINT_NETWORK_PARAM_ID = 3,
	/* An array of 256 x Unsigned 8-bit value; NULL terminated UTF8 encoded
	 * realm, 0 if unspecified.
	 */
	QCA_WLAN_VENDOR_ATTR_PNO_PASSPOINT_NETWORK_PARAM_REALM = 4,
	/* An array of 16 x Unsigned 32-bit value; roaming consortium ids
	 * to match, 0 if unspecified.
	 */
	QCA_WLAN_VENDOR_ATTR_PNO_PASSPOINT_NETWORK_PARAM_ROAM_CNSRTM_ID = 5,
	/* An array of 6 x Unsigned 8-bit value; mcc/mnc combination, 0s if
	 *  unspecified.
	 */
	QCA_WLAN_VENDOR_ATTR_PNO_PASSPOINT_NETWORK_PARAM_ROAM_PLMN = 6,

	/* NL attributes for data used by
	 * QCA_NL80211_VENDOR_SUBCMD_PNO_SET_LIST sub command.
	 */
	/* Unsigned 32-bit value */
	QCA_WLAN_VENDOR_ATTR_PNO_SET_LIST_PARAM_NUM_NETWORKS = 7,
	/* Array of nested
	 * QCA_WLAN_VENDOR_ATTR_PNO_SET_LIST_PARAM_EPNO_NETWORK_*
	 * attributes. Array size =
	 *		QCA_WLAN_VENDOR_ATTR_PNO_SET_LIST_PARAM_NUM_NETWORKS.
	 */
	QCA_WLAN_VENDOR_ATTR_PNO_SET_LIST_PARAM_EPNO_NETWORKS_LIST = 8,
	/* An array of 33 x Unsigned 8-bit value; NULL terminated SSID */
	QCA_WLAN_VENDOR_ATTR_PNO_SET_LIST_PARAM_EPNO_NETWORK_SSID = 9,
	/* Signed 8-bit value; threshold for considering this SSID as found,
	 * required granularity for this threshold is 4dBm to 8dBm
	 * This attribute is obsolete.
	 */
	QCA_WLAN_VENDOR_ATTR_PNO_SET_LIST_PARAM_EPNO_NETWORK_RSSI_THRESHOLD = 10,
	/* Unsigned 8-bit value; WIFI_PNO_FLAG_XXX */
	QCA_WLAN_VENDOR_ATTR_PNO_SET_LIST_PARAM_EPNO_NETWORK_FLAGS = 11,
	/* Unsigned 8-bit value; auth bit field for matching WPA IE */
	QCA_WLAN_VENDOR_ATTR_PNO_SET_LIST_PARAM_EPNO_NETWORK_AUTH_BIT = 12,

	/* Unsigned 8-bit to indicate ePNO type;
	 * It takes values from qca_wlan_epno_type
	 */
	QCA_WLAN_VENDOR_ATTR_PNO_SET_LIST_PARAM_EPNO_TYPE = 13,

	/* Nested attribute to send the channel list */
	QCA_WLAN_VENDOR_ATTR_PNO_SET_LIST_PARAM_EPNO_CHANNEL_LIST = 14,

	/* Unsigned 32-bit value; indicates the Interval between PNO scan
	 * cycles in msec
	 */
	QCA_WLAN_VENDOR_ATTR_PNO_SET_LIST_PARAM_EPNO_SCAN_INTERVAL = 15,
	/* Signed 32-bit value; minimum 5GHz RSSI for a BSSID to be
	 * considered
	 */
	QCA_WLAN_VENDOR_ATTR_EPNO_MIN5GHZ_RSSI = 16,
	/* Signed 32-bit value; minimum 2.4GHz RSSI for a BSSID to
	 * be considered
	 */
	QCA_WLAN_VENDOR_ATTR_EPNO_MIN24GHZ_RSSI = 17,
	/* Signed 32-bit value; the maximum score that a network
	 * can have before bonuses
	 */
	QCA_WLAN_VENDOR_ATTR_EPNO_INITIAL_SCORE_MAX = 18,
	/* Signed 32-bit value; only report when there is a network's
	 * score this much higher han the current connection
	 */
	QCA_WLAN_VENDOR_ATTR_EPNO_CURRENT_CONNECTION_BONUS = 19,
	/* Signed 32-bit value; score bonus for all networks with
	 * the same network flag
	 */
	QCA_WLAN_VENDOR_ATTR_EPNO_SAME_NETWORK_BONUS = 20,
	/* Signed 32-bit value; score bonus for networks that are
	 * not open
	 */
	QCA_WLAN_VENDOR_ATTR_EPNO_SECURE_BONUS = 21,
	/* Signed 32-bit value; 5GHz RSSI score bonus
	 * applied to all 5GHz networks
	 */
	QCA_WLAN_VENDOR_ATTR_EPNO_BAND5GHZ_BONUS = 22,

        /* Unsigned 32-bit value, representing the PNO Request ID */
        QCA_WLAN_VENDOR_ATTR_PNO_CONFIG_REQUEST_ID = 23,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_PNO_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_PNO_MAX =
		QCA_WLAN_VENDOR_ATTR_PNO_AFTER_LAST - 1,
};

enum qca_wlan_vendor_attr_roaming_config_params {
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_INVALID = 0,

	QCA_WLAN_VENDOR_ATTR_ROAMING_SUBCMD = 1,
	QCA_WLAN_VENDOR_ATTR_ROAMING_REQ_ID = 2,

	/* Attributes for wifi_set_ssid_white_list */
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_WHITE_LIST_SSID_NUM_NETWORKS = 3,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_WHITE_LIST_SSID_LIST = 4,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_WHITE_LIST_SSID = 5,

	/* Attributes for set_roam_params */
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_A_BAND_BOOST_THRESHOLD = 6,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_A_BAND_PENALTY_THRESHOLD = 7,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_A_BAND_BOOST_FACTOR = 8,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_A_BAND_PENALTY_FACTOR = 9,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_A_BAND_MAX_BOOST = 10,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_LAZY_ROAM_HISTERESYS = 11,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_ALERT_ROAM_RSSI_TRIGGER = 12,

	/* Attribute for set_lazy_roam */
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_SET_LAZY_ROAM_ENABLE = 13,

	/* Attribute for set_lazy_roam with preferences */
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_SET_BSSID_PREFS = 14,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_SET_LAZY_ROAM_NUM_BSSID = 15,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_SET_LAZY_ROAM_BSSID = 16,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_SET_LAZY_ROAM_RSSI_MODIFIER = 17,

	/* Attribute for set_ blacklist bssid params */
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_SET_BSSID_PARAMS = 18,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_SET_BSSID_PARAMS_NUM_BSSID = 19,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_SET_BSSID_PARAMS_BSSID = 20,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_MAX =
		QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_AFTER_LAST - 1,
};

/*
 * QCA_NL80211_VENDOR_SUBCMD_ROAM sub commands.
 */
enum qca_wlan_vendor_attr_roam_subcmd
{
	QCA_WLAN_VENDOR_ATTR_ROAM_SUBCMD_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_ROAM_SUBCMD_SSID_WHITE_LIST = 1,
	QCA_WLAN_VENDOR_ATTR_ROAM_SUBCMD_SET_EXTSCAN_ROAM_PARAMS = 2,
	QCA_WLAN_VENDOR_ATTR_ROAM_SUBCMD_SET_LAZY_ROAM = 3,
	QCA_WLAN_VENDOR_ATTR_ROAM_SUBCMD_SET_BSSID_PREFS = 4,
	QCA_WLAN_VENDOR_ATTR_ROAM_SUBCMD_SET_BSSID_PARAMS = 5,
	QCA_WLAN_VENDOR_ATTR_ROAM_SUBCMD_SET_BLACKLIST_BSSID = 6,

	/* KEEP LAST */
	QCA_WLAN_VENDOR_ATTR_ROAM_SUBCMD_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_ROAM_SUBCMD_MAX =
		QCA_WLAN_VENDOR_ATTR_ROAM_SUBCMD_AFTER_LAST - 1,
};

/**
 * enum qca_wlan_vendor_attr_get_wifi_info - value for get_wifi sub commands
 * @QCA_WLAN_VENDOR_ATTR_WIFI_INFO_GET_INVALID - invalid
 * @QCA_WLAN_VENDOR_ATTR_WIFI_INFO_DRIVER_VERSION - get driver version
 * @QCA_WLAN_VENDOR_ATTR_WIFI_INFO_FIRMWARE_VERSION - get firmware version
 * @QCA_WLAN_VENDOR_ATTR_WIFI_INFO_RADIO_INDEX - get radio index
 *
 * enum values are used for QCA_NL80211_VENDOR_SUBCMD_GET_WIFI_INFO sub command.
 */
enum qca_wlan_vendor_attr_get_wifi_info {
	QCA_WLAN_VENDOR_ATTR_WIFI_INFO_GET_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_WIFI_INFO_DRIVER_VERSION     = 1,
	QCA_WLAN_VENDOR_ATTR_WIFI_INFO_FIRMWARE_VERSION   = 2,
	QCA_WLAN_VENDOR_ATTR_WIFI_INFO_RADIO_INDEX        = 3,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_WIFI_INFO_GET_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_WIFI_INFO_GET_MAX  =
		QCA_WLAN_VENDOR_ATTR_WIFI_INFO_GET_AFTER_LAST - 1,
};

/**
 * enum qca_wlan_vendor_attr_get_logger_features - value for logger
 *						   supported features
 * @QCA_WLAN_VENDOR_ATTR_LOGGER_INVALID - Invalid
 * @QCA_WLAN_VENDOR_ATTR_LOGGER_SUPPORTED - Indicate the supported features
 * @QCA_WLAN_VENDOR_ATTR_LOGGER_AFTER_LAST - To keep track of the last enum
 * @QCA_WLAN_VENDOR_ATTR_LOGGER_MAX - max value possible for this type
 *
 * enum values are used for NL attributes for data used by
 * QCA_NL80211_VENDOR_SUBCMD_GET_LOGGER_FEATURE_SET sub command.
 */
enum qca_wlan_vendor_attr_get_logger_features {
	QCA_WLAN_VENDOR_ATTR_LOGGER_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_LOGGER_SUPPORTED = 1,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_LOGGER_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_LOGGER_MAX =
		QCA_WLAN_VENDOR_ATTR_LOGGER_AFTER_LAST - 1,
};

/* NL attributes for data used by
 * QCA_NL80211_VENDOR_SUBCMD_LINK_PROPERTIES.
 */
enum qca_wlan_vendor_attr_link_properties {
	QCA_WLAN_VENDOR_ATTR_LINK_PROPERTIES_INVALID    = 0,
	/* Unsigned 8bit value for specifying nof spatial streams */
	QCA_WLAN_VENDOR_ATTR_LINK_PROPERTIES_NSS        = 1,
	/* Unsigned 8bit value for the rate flags */
	QCA_WLAN_VENDOR_ATTR_LINK_PROPERTIES_RATE_FLAGS = 2,
	/* Unsigned 32bit value for operating frequency */
	QCA_WLAN_VENDOR_ATTR_LINK_PROPERTIES_FREQ       = 3,

	/* KEEP LAST */
	QCA_WLAN_VENDOR_ATTR_LINK_PROPERTIES_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_LINK_PROPERTIES_MAX =
		QCA_WLAN_VENDOR_ATTR_LINK_PROPERTIES_AFTER_LAST - 1,
};

/**
 * enum qca_wlan_vendor_attr_nd_offload - vendor NS offload support
 *
 * @QCA_WLAN_VENDOR_ATTR_ND_OFFLOAD_INVALID - Invalid
 * @QCA_WLAN_VENDOR_ATTR_ND_OFFLOAD_FLAG - Flag to set NS offload
 * @QCA_WLAN_VENDOR_ATTR_ND_OFFLOAD_AFTER_LAST - To keep track of the last enum
 * @QCA_WLAN_VENDOR_ATTR_ND_OFFLOAD_MAX - max value possible for this type
 *
 * enum values are used for NL attributes for data used by
 * QCA_NL80211_VENDOR_SUBCMD_ND_OFFLOAD sub command.
 */
enum qca_wlan_vendor_attr_nd_offload {
	QCA_WLAN_VENDOR_ATTR_ND_OFFLOAD_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_ND_OFFLOAD_FLAG,

	/* Keep last */
	QCA_WLAN_VENDOR_ATTR_ND_OFFLOAD_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_ND_OFFLOAD_MAX =
		QCA_WLAN_VENDOR_ATTR_ND_OFFLOAD_AFTER_LAST - 1,
};

/**
 * enum qca_wlan_vendor_features - vendor device/driver features
 * @QCA_WLAN_VENDOR_FEATURE_KEY_MGMT_OFFLOAD: Device supports key
 * management offload, a mechanism where the station's firmware
 * does the exchange with the AP to establish the temporal keys
 * after roaming, rather than having the supplicant do it.
 */
enum qca_wlan_vendor_features {
	QCA_WLAN_VENDOR_FEATURE_KEY_MGMT_OFFLOAD = 0,
	/* Additional features need to be added above this */
        NUM_QCA_WLAN_VENDOR_FEATURES
};

/**
 * enum qca_wlan_vendor_attr_flush_pending - Attributes for
 * flush pending traffic in firmware.
 *
 * @QCA_WLAN_VENDOR_ATTR_PEER_ADDR: Configure peer mac address.
 * @QCA_WLAN_VENDOR_ATTR_AC: Configure access category the pending
 *  packets using. It is u8 value with bit0~3 represent AC_BE, AC_BK,
 *  AC_VI, AC_VO respectively. Set the corresponding bit to 1 to flush
 *  packets with access category.
 */
enum qca_wlan_vendor_attr_flush_pending{
	QCA_WLAN_VENDOR_ATTR_FLUSH_PENDING_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_PEER_ADDR = 1,
	QCA_WLAN_VENDOR_ATTR_AC = 2,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_FLUSH_PENDING_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_FLUSH_PENDING_MAX =
	QCA_WLAN_VENDOR_ATTR_FLUSH_PENDING_AFTER_LAST - 1,
};

/* Feature defines */
#define WIFI_FEATURE_INFRA              0x0001   /* Basic infrastructure mode */
#define WIFI_FEATURE_INFRA_5G           0x0002   /* Support for 5 GHz Band */
#define WIFI_FEATURE_HOTSPOT            0x0004   /* Support for GAS/ANQP */
#define WIFI_FEATURE_P2P                0x0008   /* Wifi-Direct */
#define WIFI_FEATURE_SOFT_AP            0x0010   /* Soft AP */
#define WIFI_FEATURE_EXTSCAN            0x0020   /* Extended Scan APIs */
#define WIFI_FEATURE_NAN                0x0040   /* Neighbor Awareness
                                                    Networking */
#define WIFI_FEATURE_D2D_RTT            0x0080   /* Device-to-device RTT */
#define WIFI_FEATURE_D2AP_RTT           0x0100   /* Device-to-AP RTT */
#define WIFI_FEATURE_BATCH_SCAN         0x0200   /* Batched Scan (legacy) */
#define WIFI_FEATURE_PNO                0x0400   /* Preferred network offload */
#define WIFI_FEATURE_ADDITIONAL_STA     0x0800   /* Support for two STAs */
#define WIFI_FEATURE_TDLS               0x1000   /* Tunnel directed link
                                                    setup */
#define WIFI_FEATURE_TDLS_OFFCHANNEL    0x2000   /* Support for TDLS off
                                                    channel */
#define WIFI_FEATURE_EPR                0x4000   /* Enhanced power reporting */
#define WIFI_FEATURE_AP_STA             0x8000   /* Support for AP STA
                                                    Concurrency */
#define WIFI_FEATURE_LINK_LAYER_STATS   0x10000  /* Link layer stats */
#define WIFI_FEATURE_LOGGER             0x20000  /* WiFi Logger */
#define WIFI_FEATURE_HAL_EPNO           0x40000  /* WiFi PNO enhanced */
#define WIFI_FEATURE_RSSI_MONITOR       0x80000  /* RSSI Monitor */
#define WIFI_FEATURE_MKEEP_ALIVE        0x100000  /* WiFi mkeep_alive */
#define WIFI_FEATURE_CONFIG_NDO         0x200000  /* ND offload configure */
#define WIFI_FEATURE_TX_TRANSMIT_POWER  0x400000  /* Tx transmit power levels */
#define WIFI_FEATURE_CONTROL_ROAMING    0x800000  /* Enable/Disable roaming */
#define WIFI_FEATURE_IE_WHITELIST       0x1000000 /* Support Probe IE white listing */
#define WIFI_FEATURE_SCAN_RAND          0x2000000 /* Support MAC & Probe Sequence Number randomization */


/* Add more features here */
#define WIFI_TDLS_SUPPORT			BIT(0)
#define WIFI_TDLS_EXTERNAL_CONTROL_SUPPORT	BIT(1)
#define WIIF_TDLS_OFFCHANNEL_SUPPORT		BIT(2)

/**
 * enum wifi_logger_supported_features - values for supported logger features
 * @WIFI_LOGGER_PER_PACKET_TX_RX_STATUS_SUPPORTED - Per packet statistics
 * @WIFI_LOGGER_CONNECT_EVENT_SUPPORTED - Logging of Connectivity events
 * @WIFI_LOGGER_POWER_EVENT_SUPPORTED - Power of driver
 * @WIFI_LOGGER_WAKE_LOCK_SUPPORTED - Wakelock of driver
 * @WIFI_LOGGER_WATCHDOG_TIMER_SUPPORTED - monitor FW health
 */
enum wifi_logger_supported_features {
	WIFI_LOGGER_PER_PACKET_TX_RX_STATUS_SUPPORTED = (1 << (1)),
	WIFI_LOGGER_CONNECT_EVENT_SUPPORTED = (1 << (2)),
	WIFI_LOGGER_POWER_EVENT_SUPPORTED = (1 << (3)),
	WIFI_LOGGER_WAKE_LOCK_SUPPORTED = (1 << (4)),
	WIFI_LOGGER_VERBOSE_SUPPORTED = (1 << (5)),
	WIFI_LOGGER_WATCHDOG_TIMER_SUPPORTED = (1 << (6)),
};


#if defined(FEATURE_WLAN_CH_AVOID) || defined(FEATURE_WLAN_FORCE_SAP_SCC)
#define HDD_MAX_AVOID_FREQ_RANGES   4
typedef struct sHddAvoidFreqRange
{
   u32 startFreq;
   u32 endFreq;
} tHddAvoidFreqRange;

typedef struct sHddAvoidFreqList
{
   u32 avoidFreqRangeCount;
   tHddAvoidFreqRange avoidFreqRange[HDD_MAX_AVOID_FREQ_RANGES];
} tHddAvoidFreqList;
#endif /* FEATURE_WLAN_CH_AVOID || FEATURE_WLAN_FORCE_SAP_SCC */

enum qca_wlan_vendor_attr_acs_offload {
       QCA_WLAN_VENDOR_ATTR_ACS_CHANNEL_INVALID = 0,
       QCA_WLAN_VENDOR_ATTR_ACS_PRIMARY_CHANNEL,
       QCA_WLAN_VENDOR_ATTR_ACS_SECONDARY_CHANNEL,
       QCA_WLAN_VENDOR_ATTR_ACS_HW_MODE,
       QCA_WLAN_VENDOR_ATTR_ACS_HT_ENABLED,
       QCA_WLAN_VENDOR_ATTR_ACS_HT40_ENABLED,
       QCA_WLAN_VENDOR_ATTR_ACS_VHT_ENABLED,
       QCA_WLAN_VENDOR_ATTR_ACS_CHWIDTH,
       QCA_WLAN_VENDOR_ATTR_ACS_CH_LIST,
       QCA_WLAN_VENDOR_ATTR_ACS_VHT_SEG0_CENTER_CHANNEL,
       QCA_WLAN_VENDOR_ATTR_ACS_VHT_SEG1_CENTER_CHANNEL,
       /* keep last */
       QCA_WLAN_VENDOR_ATTR_ACS_AFTER_LAST,
       QCA_WLAN_VENDOR_ATTR_ACS_MAX =
              QCA_WLAN_VENDOR_ATTR_ACS_AFTER_LAST - 1
};

enum qca_wlan_vendor_acs_hw_mode {
        QCA_ACS_MODE_IEEE80211B,
        QCA_ACS_MODE_IEEE80211G,
        QCA_ACS_MODE_IEEE80211A,
        QCA_ACS_MODE_IEEE80211AD,
};

#define CFG_NON_AGG_RETRY_MAX                  (31)
#define CFG_AGG_RETRY_MAX                      (31)
#define CFG_MGMT_RETRY_MAX                     (31)
#define CFG_CTRL_RETRY_MAX                     (31)
#define CFG_PROPAGATION_DELAY_MAX              (63)
#define CFG_PROPAGATION_DELAY_BASE             (64)
#define CFG_AGG_RETRY_MIN                      (5)

/**
 * enum qca_access_policy - access control policy
 *
 * Access control policy is applied on the configured IE
 * (QCA_WLAN_VENDOR_ATTR_CONFIG_ACCESS_POLICY_IE).
 * To be set with QCA_WLAN_VENDOR_ATTR_CONFIG_ACCESS_POLICY.
 *
 * @QCA_ACCESS_POLICY_ACCEPT_UNLESS_LISTED: Deny Wi-Fi Connections which match
 *	with the specific configuration (IE) set, i.e. allow all the
 *	connections which do not match the configuration.
 * @QCA_ACCESS_POLICY_DENY_UNLESS_LISTED: Accept Wi-Fi Connections which match
 *	with the specific configuration (IE) set, i.e. deny all the
 *	connections which do not match the configuration.
 */
enum qca_access_policy {
	QCA_ACCESS_POLICY_ACCEPT_UNLESS_LISTED,
	QCA_ACCESS_POLICY_DENY_UNLESS_LISTED,
};

/**
 * enum qca_wlan_vendor_config: wifi config attr
 *
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_INVALID: invalid config
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_MODULATED_DTIM: dynamic dtim
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_STATS_AVG_FACTOR: stats avg. factor
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_GUARD_TIME: guard time
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_TX_RATE: configure max tx rate
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_TX_MPDU_AGGREGATION:
 *                                   Tx aggregation size (8-bit unsigned value)
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_RX_MPDU_AGGREGATION:
 *                                   Rx aggregation size (8-bit unsigned value)
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_NON_AGG_RETRY:
 *                                   Non aggregrate/11g sw retry threshold
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_AGG_RETRY: aggregrate sw retry threshold
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_MGMT_RETRY: management frame sw retry threshold
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_CTRL_RETRY: control frame sw retry threshold
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_PROPAGATION_DELAY:
 *			     propagation delay for 2G/5G band(Units in us)
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_TX_FAIL_COUNT: Unsigned 32-bit value to
 * configure the number of unicast TX fail packet count.
 * The peer is disconnected once this threshold is reached.
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_SCAN_DEFAULT_IES: Update the default scan IEs
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_COMMAND:
 *                         Unsigned 32-bit value attribute for generic commands
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_VALUE:
 *                  Unsigned 32-bit data attribute for generic command response
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_DATA:
 *                  Unsigned 32-bit data attribute for generic command response
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_LENGTH:
 *                                     Unsigned 32-bit length attribute for
 *                                     QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_DATA
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_FLAGS:
 * Unsigned 32-bit flags attribute for QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_DATA
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_ACCESS_POLICY: Vendor IE access policy
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_ACCESS_POLICY_IE_LIST: Vendor IE to be used
 *                                                     with access policy
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_IFINDEX: interface index for vdev specific
 *                                       parameters
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_QPOWER: Unsigned 8bit length attribute to update
 *                                      power save config to turn off/on qpower
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_RX_REORDER_TIMEOUT_VOICE:
 *                  32-bit unsigned value to set reorder timeout for AC_VO
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_RX_REORDER_TIMEOUT_VIDEO:
 *                  32-bit unsigned value to set reorder timeout for AC_VI
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_RX_REORDER_TIMEOUT_BESTEFFORT:
 *                  32-bit unsigned value to set reorder timeout for AC_BE
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_RX_REORDER_TIMEOUT_BACKGROUND:
 *                  32-bit unsigned value to set reorder timeout for AC_BK
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_RX_BLOCKSIZE_PEER_MAC:
 *                  6-byte MAC address to point out the specific peer
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_RX_BLOCKSIZE_WINLIMIT:
 *                  32-bit unsigned value to set window size for specific peer
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_PROPAGATION_ABS_DELAY:
 *                  32-bit unsigned value to configure the propagation absolute
 *                  delay for 2G/5G band (units in us)
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_LAST: last config
 * @QCA_WLAN_VENDOR_ATTR_CONFIG_MAX: max config
 */
enum qca_wlan_vendor_config {
	QCA_WLAN_VENDOR_ATTR_CONFIG_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_CONFIG_MODULATED_DTIM,
	QCA_WLAN_VENDOR_ATTR_CONFIG_STATS_AVG_FACTOR,
	QCA_WLAN_VENDOR_ATTR_CONFIG_GUARD_TIME,
	QCA_WLAN_VENDOR_ATTR_CONFIG_FINE_TIME_MEASUREMENT,
	QCA_WLAN_VENDOR_ATTR_CONFIG_TX_RATE,
	QCA_WLAN_VENDOR_ATTR_CONFIG_PENALIZE_AFTER_NCONS_BEACON_MISS,
	QCA_WLAN_VENDOR_ATTR_CONFIG_CHANNEL_AVOIDANCE_IND,
	QCA_WLAN_VENDOR_ATTR_CONFIG_TX_MPDU_AGGREGATION,
	QCA_WLAN_VENDOR_ATTR_CONFIG_RX_MPDU_AGGREGATION,
	QCA_WLAN_VENDOR_ATTR_CONFIG_NON_AGG_RETRY,
	QCA_WLAN_VENDOR_ATTR_CONFIG_AGG_RETRY,
	QCA_WLAN_VENDOR_ATTR_CONFIG_MGMT_RETRY,
	QCA_WLAN_VENDOR_ATTR_CONFIG_CTRL_RETRY,
	QCA_WLAN_VENDOR_ATTR_CONFIG_PROPAGATION_DELAY,
	QCA_WLAN_VENDOR_ATTR_CONFIG_TX_FAIL_COUNT,
	/* Attribute used to set scan default IEs to the driver.
	 *
	 * These IEs can be used by scan operations that will be initiated by
	 * the driver/firmware.
	 *
	 * For further scan requests coming to the driver, these IEs should be
	 * merged with the IEs received along with scan request coming to the
	 * driver. If a particular IE is present in the scan default IEs but not
	 * present in the scan request, then that IE should be added to the IEs
	 * sent in the Probe Request frames for that scan request. */
	QCA_WLAN_VENDOR_ATTR_CONFIG_SCAN_DEFAULT_IES,
	/* Unsigned 32-bit attribute for generic commands */
	QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_COMMAND,
	/* Unsigned 32-bit value attribute for generic commands */
	QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_VALUE,
	/* Unsigned 32-bit data attribute for generic command response */
	QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_DATA,
	/* Unsigned 32-bit length attribute for
	 * QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_DATA */
	QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_LENGTH,
	/* Unsigned 32-bit flags attribute for
	 * QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_DATA */
	QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_FLAGS,
	/* Unsigned 32-bit, defining the access policy.
	 * See enum qca_access_policy. Used with
	 * QCA_WLAN_VENDOR_ATTR_CONFIG_ACCESS_POLICY_IE_LIST. */
	QCA_WLAN_VENDOR_ATTR_CONFIG_ACCESS_POLICY,
	/* Sets the list of full set of IEs for which a specific access policy
	 * has to be applied. Used along with
	 * QCA_WLAN_VENDOR_ATTR_CONFIG_ACCESS_POLICY to control the access.
	 * Zero length payload can be used to clear this access constraint. */
	QCA_WLAN_VENDOR_ATTR_CONFIG_ACCESS_POLICY_IE_LIST,
	/* Unsigned 32-bit, specifies the interface index (netdev) for which the
	 * corresponding configurations are applied. If the interface index is
	 * not specified, the configurations are attributed to the respective
	 * wiphy. */
	QCA_WLAN_VENDOR_ATTR_CONFIG_IFINDEX,
	/* Unsigned 8-bit, for setting qpower dynamically */
	QCA_WLAN_VENDOR_ATTR_CONFIG_QPOWER,

	/* 32-bit unsigned value to trigger antenna diversity features:
	 * 1-Enable, 0-Disable */
	QCA_WLAN_VENDOR_ATTR_CONFIG_ANT_DIV_ENA = 27,
	/* 32-bit unsigned value to configure specific chain antenna */
	QCA_WLAN_VENDOR_ATTR_CONFIG_ANT_DIV_CHAIN = 28,
	/* 32-bit unsigned value to trigger cycle selftest
	 * 1-Enable, 0-Disable */
	QCA_WLAN_VENDOR_ATTR_CONFIG_ANT_DIV_SELFTEST = 29,
	/* 32-bit unsigned to configure the cycle time of selftest
	 * the unit is micro-second */
	QCA_WLAN_VENDOR_ATTR_CONFIG_ANT_DIV_SELFTEST_INTVL = 30,

	QCA_WLAN_VENDOR_ATTR_CONFIG_RX_REORDER_TIMEOUT_VOICE = 31,
	QCA_WLAN_VENDOR_ATTR_CONFIG_RX_REORDER_TIMEOUT_VIDEO = 32,
	QCA_WLAN_VENDOR_ATTR_CONFIG_RX_REORDER_TIMEOUT_BESTEFFORT = 33,
	QCA_WLAN_VENDOR_ATTR_CONFIG_RX_REORDER_TIMEOUT_BACKGROUND = 34,
	QCA_WLAN_VENDOR_ATTR_CONFIG_RX_BLOCKSIZE_PEER_MAC = 35,
	QCA_WLAN_VENDOR_ATTR_CONFIG_RX_BLOCKSIZE_WINLIMIT = 36,
	QCA_WLAN_VENDOR_ATTR_CONFIG_SUB20_CHAN_WIDTH = 39,
	QCA_WLAN_VENDOR_ATTR_CONFIG_PROPAGATION_ABS_DELAY = 40,
	/* 32-bit unsigned value to set probe period*/
	QCA_WLAN_VENDOR_ATTR_CONFIG_ANT_DIV_PROBE_PERIOD = 41,
	/* 32-bit unsigned value to set stay period*/
	QCA_WLAN_VENDOR_ATTR_CONFIG_ANT_DIV_STAY_PERIOD = 42,
	/* 32-bit unsigned value to set snr diff*/
	QCA_WLAN_VENDOR_ATTR_CONFIG_ANT_DIV_SNR_DIFF = 43,
	/* 32-bit unsigned value to set probe dewll time*/
	QCA_WLAN_VENDOR_ATTR_CONFIG_ANT_DIV_PROBE_DWELL_TIME = 44,
	/* 32-bit unsigned value to set mgmt snr weight*/
	QCA_WLAN_VENDOR_ATTR_CONFIG_ANT_DIV_MGMT_SNR_WEIGHT = 45,
	/* 32-bit unsigned value to set data snr weight*/
	QCA_WLAN_VENDOR_ATTR_CONFIG_ANT_DIV_DATA_SNR_WEIGHT = 46,
	/* 32-bit unsigned value to set ack snr weight*/
	QCA_WLAN_VENDOR_ATTR_CONFIG_ANT_DIV_ACK_SNR_WEIGHT = 47,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_CONFIG_LAST,
	QCA_WLAN_VENDOR_ATTR_CONFIG_MAX =
		QCA_WLAN_VENDOR_ATTR_CONFIG_LAST - 1
};

/**
 * enum qca_wlan_vendor_attr_wifi_logger_start - Enum for wifi logger starting
 * @QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_START_INVALID: Invalid attribute
 * @QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_RING_ID: Ring ID
 * @QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_VERBOSE_LEVEL: Verbose level
 * @QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_FLAGS: Flag
 * @QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_START_AFTER_LAST: Last value
 * @QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_START_MAX: Max value
 */
enum qca_wlan_vendor_attr_wifi_logger_start {
	QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_START_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_RING_ID = 1,
	QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_VERBOSE_LEVEL = 2,
	QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_FLAGS = 3,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_START_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_START_MAX =
		QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_START_AFTER_LAST - 1,
};

/*
 * enum qca_wlan_vendor_attr_wifi_logger_get_ring_data - Get ring data
 * @QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_GET_RING_DATA_INVALID: Invalid attribute
 * @QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_GET_RING_DATA_ID: Ring ID
 * @QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_GET_RING_DATA_AFTER_LAST: Last value
 * @QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_GET_RING_DATA_MAX: Max value
 */
enum qca_wlan_vendor_attr_wifi_logger_get_ring_data {
	QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_GET_RING_DATA_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_GET_RING_DATA_ID = 1,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_GET_RING_DATA_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_GET_RING_DATA_MAX =
		QCA_WLAN_VENDOR_ATTR_WIFI_LOGGER_GET_RING_DATA_AFTER_LAST - 1,
};

#ifdef WLAN_FEATURE_OFFLOAD_PACKETS
/**
 * enum wlan_offloaded_packets_control - control commands
 * @WLAN_START_OFFLOADED_PACKETS: start offloaded packets
 * @WLAN_STOP_OFFLOADED_PACKETS: stop offloaded packets
 *
 */
enum wlan_offloaded_packets_control {
	WLAN_START_OFFLOADED_PACKETS = 1,
	WLAN_STOP_OFFLOADED_PACKETS  = 2
};

/**
 * enum qca_wlan_vendor_attr_offloaded_packets - offloaded packets
 * @QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_INVALID: invalid
 * @QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_SENDING_CONTROL: control
 * @QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_REQUEST_ID: request id
 * @QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_IP_PACKET_DATA: ip packet data
 * @QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_SRC_MAC_ADDR: src mac address
 * @QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_DST_MAC_ADDR: destination mac address
 * @QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_PERIOD: period in milli seconds
 * @QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_AFTER_LAST: after last
 * @QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_MAX: max
 */
enum qca_wlan_vendor_attr_offloaded_packets {
	QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_SENDING_CONTROL,
	QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_REQUEST_ID,

	/* Packet in hex format */
	QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_IP_PACKET_DATA,
	QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_SRC_MAC_ADDR,
	QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_DST_MAC_ADDR,
	QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_PERIOD,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_MAX =
		QCA_WLAN_VENDOR_ATTR_OFFLOADED_PACKETS_AFTER_LAST - 1,
};
#endif

/**
 * enum qca_wlan_rssi_monitoring_control - rssi control commands
 * @QCA_WLAN_RSSI_MONITORING_CONTROL_INVALID: invalid
 * @QCA_WLAN_RSSI_MONITORING_START: rssi monitoring start
 * @QCA_WLAN_RSSI_MONITORING_STOP: rssi monitoring stop
 */
enum qca_wlan_rssi_monitoring_control {
	QCA_WLAN_RSSI_MONITORING_CONTROL_INVALID = 0,
	QCA_WLAN_RSSI_MONITORING_START,
	QCA_WLAN_RSSI_MONITORING_STOP,
};

/**
 * enum qca_wlan_vendor_attr_rssi_monitoring - rssi monitoring
 * @QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_INVALID: Invalid
 * @QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_CONTROL: control
 * @QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_MAX_RSSI: max rssi
 * @QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_MIN_RSSI: min rssi
 * @QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_CUR_BSSID: current bssid
 * @QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_CUR_RSSI: current rssi
 * @QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_AFTER_LAST: after last
 * @QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_MAX: max
 */
enum qca_wlan_vendor_attr_rssi_monitoring {
	QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_INVALID = 0,

	QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_CONTROL,
	QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_REQUEST_ID,

	QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_MAX_RSSI,
	QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_MIN_RSSI,

	/* attributes to be used/received in callback */
	QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_CUR_BSSID,
	QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_CUR_RSSI,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_MAX =
		QCA_WLAN_VENDOR_ATTR_RSSI_MONITORING_AFTER_LAST - 1,
};

/**
 * qca_chip_power_save_failure_reason: Power save failure reason
 * @QCA_CHIP_POWER_SAVE_FAILURE_REASON_PROTOCOL: Indicates power save failure
 * due to protocol/module.
 * @QCA_CHIP_POWER_SAVE_FAILURE_REASON_HARDWARE: power save failure
 * due to hardware
 */
enum qca_chip_power_save_failure_reason {
        QCA_CHIP_POWER_SAVE_FAILURE_REASON_PROTOCOL = 0,
        QCA_CHIP_POWER_SAVE_FAILURE_REASON_HARDWARE = 1,
};

/**
 * qca_attr_chip_power_save_failure: attributes to vendor subcmd
 * @QCA_NL80211_VENDOR_SUBCMD_CHIP_PWRSAVE_FAILURE. This carry the requisite
 * information leading to the power save failure.
 * @QCA_ATTR_CHIP_POWER_SAVE_FAILURE_INVALID : invalid
 * @QCA_ATTR_CHIP_POWER_SAVE_FAILURE_REASON : power save failure reason
 * represented by enum qca_chip_power_save_failure_reason
 * @QCA_ATTR_CHIP_POWER_SAVE_FAILURE_LAST : Last
 * @QCA_ATTR_CHIP_POWER_SAVE_FAILURE_MAX : Max value
 */
enum qca_attr_chip_power_save_failure {
	QCA_ATTR_CHIP_POWER_SAVE_FAILURE_INVALID = 0,

	QCA_ATTR_CHIP_POWER_SAVE_FAILURE_REASON = 1,

	/* keep last */
	QCA_ATTR_CHIP_POWER_SAVE_FAILURE_LAST,
	QCA_ATTR_CHIP_POWER_SAVE_FAILURE_MAX =
		QCA_ATTR_CHIP_POWER_SAVE_FAILURE_LAST - 1,
};

/**
 * enum set_reset_packet_filter - set packet filter control commands
 * @QCA_WLAN_SET_PACKET_FILTER: Set Packet Filter
 * @QCA_WLAN_GET_PACKET_FILTER: Get Packet filter
 */
enum set_reset_packet_filter {
	QCA_WLAN_SET_PACKET_FILTER = 1,
	QCA_WLAN_GET_PACKET_FILTER = 2,
};

/**
 * enum qca_wlan_vendor_attr_packet_filter - BPF control commands
 * @QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_INVALID: Invalid
 * @QCA_WLAN_VENDOR_ATTR_SET_RESET_PACKET_FILTER: Filter ID
 * @QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_VERSION: Filter Version
 * @QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_SIZE: Total Length
 * @QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_CURRENT_OFFSET: Current offset
 * @QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_PROGRAM: length of BPF instructions
 */
enum qca_wlan_vendor_attr_packet_filter {
	QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_SET_RESET_PACKET_FILTER,
	QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_VERSION,
	QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_ID,
	QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_SIZE,
	QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_CURRENT_OFFSET,
	QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_PROGRAM,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_MAX =
	QCA_WLAN_VENDOR_ATTR_PACKET_FILTER_AFTER_LAST - 1,
};

/**
 * enum qca_wlan_vendor_drv_info - WLAN driver info
 * @QCA_WLAN_VENDOR_ATTR_DRV_INFO_INVALID: Invalid
 * @QCA_WLAN_VENDOR_ATTR_DRV_INFO_BUS_SIZE: Maximum Message size info
 * between Firmware & Host.
 */
enum qca_wlan_vendor_drv_info {
	QCA_WLAN_VENDOR_ATTR_DRV_INFO_INVALID = 0,

	QCA_WLAN_VENDOR_ATTR_DRV_INFO_BUS_SIZE,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_DRV_INFO_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_DRV_INFO_MAX =
		QCA_WLAN_VENDOR_ATTR_DRV_INFO_AFTER_LAST - 1,
};

/**
 * enum qca_wlan_vendor_attr_wake_stats - wake lock stats
 * @QCA_WLAN_VENDOR_ATTR_GET_WAKE_STATS_INVALID: invalid
 * @QCA_WLAN_VENDOR_ATTR_TOTAL_CMD_EVENT_WAKE:
 * @QCA_WLAN_VENDOR_ATTR_CMD_EVENT_WAKE_CNT_PTR:
 * @QCA_WLAN_VENDOR_ATTR_CMD_EVENT_WAKE_CNT_SZ:
 * @QCA_WLAN_VENDOR_ATTR_TOTAL_DRIVER_FW_LOCAL_WAKE:
 * @QCA_WLAN_VENDOR_ATTR_DRIVER_FW_LOCAL_WAKE_CNT_PTR:
 * @QCA_WLAN_VENDOR_ATTR_DRIVER_FW_LOCAL_WAKE_CNT_SZ:
 * @QCA_WLAN_VENDOR_ATTR_TOTAL_RX_DATA_WAKE:
 * total rx wakeup count
 * @QCA_WLAN_VENDOR_ATTR_RX_UNICAST_CNT:
 * Total rx unicast packet which woke up host
 * @QCA_WLAN_VENDOR_ATTR_RX_MULTICAST_CNT:
 * Total rx multicast packet which woke up host
 * @QCA_WLAN_VENDOR_ATTR_RX_BROADCAST_CNT:
 * Total rx broadcast packet which woke up host
 * @QCA_WLAN_VENDOR_ATTR_ICMP_PKT:
 * wake icmp packet count
 * @QCA_WLAN_VENDOR_ATTR_ICMP6_PKT:
 * wake icmp6 packet count
 * @QCA_WLAN_VENDOR_ATTR_ICMP6_RA:
 * wake icmp6 RA packet count
 * @QCA_WLAN_VENDOR_ATTR_ICMP6_NA:
 * wake icmp6 NA packet count
 * @QCA_WLAN_VENDOR_ATTR_ICMP6_NS:
 * wake icmp6 NS packet count
 * @QCA_WLAN_VENDOR_ATTR_ICMP4_RX_MULTICAST_CNT:
 * Rx wake packet count due to ipv4 multicast
 * @QCA_WLAN_VENDOR_ATTR_ICMP6_RX_MULTICAST_CNT:
 * Rx wake packet count due to ipv6 multicast
 * @QCA_WLAN_VENDOR_ATTR_OTHER_RX_MULTICAST_CNT:
 * Rx wake packet count due to non-ipv4 and non-ipv6 packets
 * @QCA_WLAN_VENDOR_ATTR_RSSI_BREACH_CNT:
 * wake rssi breach packet count
 * @QCA_WLAN_VENDOR_ATTR_LOW_RSSI_CNT:
 * wake low rssi packet count
 * @QCA_WLAN_VENDOR_ATTR_GSCAN_CNT:
 * wake gscan packet count
 * @QCA_WLAN_VENDOR_ATTR_PNO_COMPLETE_CNT:
 * wake pno complete packet count
 * @QCA_WLAN_VENDOR_ATTR_PNO_MATCH_CNT:
 * wake pno match packet count
 */
enum qca_wlan_vendor_attr_wake_stats {
	QCA_WLAN_VENDOR_ATTR_GET_WAKE_STATS_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_TOTAL_CMD_EVENT_WAKE,
	QCA_WLAN_VENDOR_ATTR_CMD_EVENT_WAKE_CNT_PTR,
	QCA_WLAN_VENDOR_ATTR_CMD_EVENT_WAKE_CNT_SZ,
	QCA_WLAN_VENDOR_ATTR_TOTAL_DRIVER_FW_LOCAL_WAKE,
	QCA_WLAN_VENDOR_ATTR_DRIVER_FW_LOCAL_WAKE_CNT_PTR,
	QCA_WLAN_VENDOR_ATTR_DRIVER_FW_LOCAL_WAKE_CNT_SZ,
	QCA_WLAN_VENDOR_ATTR_TOTAL_RX_DATA_WAKE,
	QCA_WLAN_VENDOR_ATTR_RX_UNICAST_CNT,
	QCA_WLAN_VENDOR_ATTR_RX_MULTICAST_CNT,
	QCA_WLAN_VENDOR_ATTR_RX_BROADCAST_CNT,
	QCA_WLAN_VENDOR_ATTR_ICMP_PKT,
	QCA_WLAN_VENDOR_ATTR_ICMP6_PKT,
	QCA_WLAN_VENDOR_ATTR_ICMP6_RA,
	QCA_WLAN_VENDOR_ATTR_ICMP6_NA,
	QCA_WLAN_VENDOR_ATTR_ICMP6_NS,
	QCA_WLAN_VENDOR_ATTR_ICMP4_RX_MULTICAST_CNT,
	QCA_WLAN_VENDOR_ATTR_ICMP6_RX_MULTICAST_CNT,
	QCA_WLAN_VENDOR_ATTR_OTHER_RX_MULTICAST_CNT,
	QCA_WLAN_VENDOR_ATTR_RSSI_BREACH_CNT,
	QCA_WLAN_VENDOR_ATTR_LOW_RSSI_CNT,
	QCA_WLAN_VENDOR_ATTR_GSCAN_CNT,
	QCA_WLAN_VENDOR_ATTR_PNO_COMPLETE_CNT,
	QCA_WLAN_VENDOR_ATTR_PNO_MATCH_CNT,
	/* keep last */
	QCA_WLAN_VENDOR_GET_WAKE_STATS_AFTER_LAST,
	QCA_WLAN_VENDOR_GET_WAKE_STATS_MAX =
		QCA_WLAN_VENDOR_GET_WAKE_STATS_AFTER_LAST - 1,
};

/**
 * enum dfs_mode - state of DFS mode
 * @DFS_MODE_NONE: DFS mode attribute is none
 * @DFS_MODE_ENABLE:  DFS mode is enabled
 * @DFS_MODE_DISABLE: DFS mode is disabled
 * @DFS_MODE_DEPRIORITIZE: Deprioritize DFS channels in scanning
 */
enum dfs_mode {
	DFS_MODE_NONE,
	DFS_MODE_ENABLE,
	DFS_MODE_DISABLE,
	DFS_MODE_DEPRIORITIZE
};

/**
 * enum qca_wlan_vendor_attr_acs_config - Config params for ACS
 * @QCA_WLAN_VENDOR_ATTR_ACS_MODE_INVALID: Invalid
 * @QCA_WLAN_VENDOR_ATTR_ACS_DFS_MODE: Dfs mode for ACS
 * QCA_WLAN_VENDOR_ATTR_ACS_CHANNEL_HINT: channel_hint for ACS
 * QCA_WLAN_VENDOR_ATTR_ACS_DFS_AFTER_LAST: after_last
 * QCA_WLAN_VENDOR_ATTR_ACS_DFS_MAX: max attribute
 */
enum qca_wlan_vendor_attr_acs_config {
	QCA_WLAN_VENDOR_ATTR_ACS_MODE_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_ACS_DFS_MODE,
	QCA_WLAN_VENDOR_ATTR_ACS_CHANNEL_HINT,

	QCA_WLAN_VENDOR_ATTR_ACS_DFS_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_ACS_DFS_MAX =
		QCA_WLAN_VENDOR_ATTR_ACS_DFS_AFTER_LAST - 1,

};

/**
 * enum qca_wlan_vendor_attr_sta_connect_roam_policy_config -
 *                        config params for sta roam policy
 * @QCA_WLAN_VENDOR_ATTR_STA_CONNECT_ROAM_POLICY_INVALID: Invalid
 * @QCA_WLAN_VENDOR_ATTR_STA_DFS_MODE: If sta should skip Dfs channels
 * @QCA_WLAN_VENDOR_ATTR_STA_SKIP_UNSAFE_CHANNEL:
 * If sta should skip unsafe channels or not in scanning
 * @QCA_WLAN_VENDOR_ATTR_STA_CONNECT_ROAM_POLICY_LAST:
 * @QCA_WLAN_VENDOR_ATTR_STA_CONNECT_ROAM_POLICY_MAX: max attribute
 */
enum qca_wlan_vendor_attr_sta_connect_roam_policy_config {
	QCA_WLAN_VENDOR_ATTR_STA_CONNECT_ROAM_POLICY_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_STA_DFS_MODE,
	QCA_WLAN_VENDOR_ATTR_STA_SKIP_UNSAFE_CHANNEL,

	QCA_WLAN_VENDOR_ATTR_STA_CONNECT_ROAM_POLICY_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_STA_CONNECT_ROAM_POLICY_MAX =
		QCA_WLAN_VENDOR_ATTR_STA_CONNECT_ROAM_POLICY_AFTER_LAST - 1,
};

 /**
  * enum qca_wlan_vendor_attr_sap_config - config params for sap configuration
  * @QCA_WLAN_VENDOR_ATTR_SAP_CONFIG_INVALID: invalid
  * @QCA_WLAN_VENDOR_ATTR_SAP_CONFIG_CHANNEL: Channel on which SAP should start
  * @QCA_WLAN_VENDOR_ATTR_SAP_CONFIG_AFTER_LAST: after last
  * @QCA_WLAN_VENDOR_ATTR_SAP_CONFIG_MAX: max attribute
  */
enum qca_wlan_vendor_attr_sap_config {
	QCA_WLAN_VENDOR_ATTR_SAP_CONFIG_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_SAP_CONFIG_CHANNEL,

	QCA_WLAN_VENDOR_ATTR_SAP_CONFIG_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_SAP_CONFIG_MAX =
		QCA_WLAN_VENDOR_ATTR_SAP_CONFIG_AFTER_LAST - 1,
};

/**
 * enum qca_wlan_vendor_attr_get_station - Sub commands used by
 * QCA_NL80211_VENDOR_SUBCMD_GET_STATION to get the corresponding
 * station information. The information obtained through these
 * commands signify the current info in connected state and
 * latest cached information during the connected state , if queried
 * when in disconnected state.
 *
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INVALID: Invalid attribute
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO: bss info
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_ASSOC_FAIL_REASON: assoc fail reason
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_REMOTE: remote peer info
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_AFTER_LAST: After last
 */
enum qca_wlan_vendor_attr_get_station {
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO = 1,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_ASSOC_FAIL_REASON = 2,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_REMOTE = 3,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_GET_STATION_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_MAX =
		QCA_WLAN_VENDOR_ATTR_GET_STATION_AFTER_LAST - 1,
};

/**
 * enum qca_wlan_vendor_attr_get_station_info - Station Info queried
 * through QCA_NL80211_VENDOR_SUBCMD_GET_STATION.
 *
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_INVALID: Invalid Attribute
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_LINK_STANDARD_NL80211_ATTR:
 *  Get the standard NL attributes Nested with this attribute.
 *  Ex : Query BW , BITRATE32 , NSS , Signal , Noise of the Link -
 *  NL80211_ATTR_SSID / NL80211_ATTR_SURVEY_INFO (Connected Channel) /
 *  NL80211_ATTR_STA_INFO
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AP_STANDARD_NL80211_ATTR:
 *  Get the standard NL attributes Nested with this attribute.
 *  Ex : Query HT/VHT Capability advertized by the AP.
 *  NL80211_ATTR_VHT_CAPABILITY / NL80211_ATTR_HT_CAPABILITY
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ROAM_COUNT:
 *  Number of successful Roam attempts before a
 *  disconnect, Unsigned 32 bit value
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AKM:
 *  Authentication Key Management Type used for the connected session.
 *  Signified by enum qca_wlan_auth_type
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_802_11_MODE: 802.11 Mode of the
 *  connected Session, signified by enum qca_wlan_802_11_mode
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AP_HS20_INDICATION:
 *  HS20 Indication Element
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ASSOC_FAIL_REASON:
 *  Status Code Corresponding to the Association Failure.
 *  Unsigned 32 bit value.
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_MAX_PHY_RATE:
 *  max phy rate for remote peer
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_PACKETS:
 *  tx packets for remote peer (interface layer)
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_BYTES:
 *  tx bytes for remote peer (interface layer)
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_PACKETS:
 *  rx packets for remote peer (interface layer)
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_BYTES:
 *  rx bytes for remote peer (interface layer)
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_LAST_TX_RATE:
 *  last tx rate for remote peer (in kbps)
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_LAST_RX_RATE:
 *  last rx rate for remote peer (in kbps)
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_WMM:
 *  wmm for remote peer
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_SUPPORTED_MODE:
 *  supported mode for remote peer
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_AMPDU:
 *  ampdu for remote peer
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_STBC:
 *  tx stbc for remote peer
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_STBC:
 *  rx stbc for remote peer
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_CH_WIDTH:
 *  ch widht for remote peer
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_SGI_ENABLE:
 *  sgi enable for remote peer
 * @QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AFTER_LAST: After last
 */
enum qca_wlan_vendor_attr_get_station_info {
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_LINK_STANDARD_NL80211_ATTR,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AP_STANDARD_NL80211_ATTR,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ROAM_COUNT,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AKM,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_802_11_MODE,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AP_HS20_INDICATION,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_HT_OPERATION,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_VHT_OPERATION,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ASSOC_FAIL_REASON,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_MAX_PHY_RATE,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_PACKETS,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_BYTES,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_PACKETS,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_BYTES,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_LAST_TX_RATE,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_LAST_RX_RATE,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_WMM,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_SUPPORTED_MODE,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_AMPDU,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_STBC,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_STBC,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_CH_WIDTH,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_SGI_ENABLE,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_PAD,
#endif
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_MAX =
		QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AFTER_LAST - 1,
};

/* define short names for get station info attributes */
#define LINK_INFO_STANDARD_NL80211_ATTR \
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_LINK_STANDARD_NL80211_ATTR
#define AP_INFO_STANDARD_NL80211_ATTR \
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AP_STANDARD_NL80211_ATTR
#define INFO_ROAM_COUNT \
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ROAM_COUNT
#define INFO_AKM \
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AKM
#define WLAN802_11_MODE \
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_802_11_MODE
#define AP_INFO_HS20_INDICATION \
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AP_HS20_INDICATION
#define HT_OPERATION \
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_HT_OPERATION
#define VHT_OPERATION \
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_VHT_OPERATION
#define INFO_ASSOC_FAIL_REASON \
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ASSOC_FAIL_REASON

/** enum qca_vendor_attr_txpower_scale - vendor sub commands index
 * @QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_INVALID: invalid value
 * @QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE: scaling value
 * @QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_AFTER_LAST: last value
 * @QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_MAX: max value
 */
enum qca_vendor_attr_txpower_scale {
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_INVALID,
	/* 8-bit unsigned value to indicate the scaling of tx power */
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_MAX =
		QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_AFTER_LAST - 1
};

/**
 * enum qca_vendor_attr_txpower_scale_decr_db - vendor sub commands index
 * @QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_DECR_DB_INVALID: invalid value
 * @QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_DECR_DB: scaling value
 * @QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_DECR_DB_AFTER_LAST: last value
 * @QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_DECR_DB_MAX: max value
 */
enum qca_vendor_attr_txpower_scale_decr_db {
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_DECR_DB_INVALID,
	/* 8-bit unsigned value to indicate the scaling of tx power */
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_DECR_DB,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_DECR_DB_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_DECR_DB_MAX =
		QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_DECR_DB_AFTER_LAST - 1
};

/**
 * enum qca_wlan_vendor_attr_ll_stats_ext - Attributes for MAC layer monitoring
 *    offload which is an extension for LL_STATS.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_CFG_PERIOD: Monitoring period. Unit in ms.
 *    If MAC counters do not exceed the threshold, FW will report monitored
 *    link layer counters periodically as this setting. The first report is
 *    always triggered by this timer.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_CFG_THRESHOLD: It is a percentage (1-99).
 *    For each MAC layer counter, FW holds two copies. One is the current value.
 *    The other is the last report. Once a current counter's increment is larger
 *    than the threshold, FW will indicate that counter to host even if the
 *    monitoring timer does not expire.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_CHG: Peer STA power state change
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TID: TID of MSDU
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_NUM_MSDU: Count of MSDU with the same
 *    failure code.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_STATUS: TX failure code
 *    1: TX packet discarded
 *    2: No ACK
 *    3: Postpone
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_MAC_ADDRESS: peer MAC address
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_STATE: Peer STA current state
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_GLOBAL: Global threshold.
 *    Threshold for all monitored parameters. If per counter dedicated threshold
 *    is not enabled, this threshold will take effect.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_EVENT_MODE: Indicate what triggers this
 *    event, PERORID_TIMEOUT == 1, THRESH_EXCEED == 0.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IFACE_ID: interface ID
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_ID: peer ID
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BITMAP: bitmap for TX counters
 *    Bit0: TX counter unit in MSDU
 *    Bit1: TX counter unit in MPDU
 *    Bit2: TX counter unit in PPDU
 *    Bit3: TX counter unit in byte
 *    Bit4: Dropped MSDUs
 *    Bit5: Dropped Bytes
 *    Bit6: MPDU retry counter
 *    Bit7: MPDU failure counter
 *    Bit8: PPDU failure counter
 *    Bit9: MPDU aggregation counter
 *    Bit10: MCS counter for ACKed MPDUs
 *    Bit11: MCS counter for Failed MPDUs
 *    Bit12: TX Delay counter
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BITMAP: bitmap for RX counters
 *    Bit0: MAC RX counter unit in MPDU
 *    Bit1: MAC RX counter unit in byte
 *    Bit2: PHY RX counter unit in PPDU
 *    Bit3: PHY RX counter unit in byte
 *    Bit4: Disorder counter
 *    Bit5: Retry counter
 *    Bit6: Duplication counter
 *    Bit7: Discard counter
 *    Bit8: MPDU aggregation size counter
 *    Bit9: MCS counter
 *    Bit10: Peer STA power state change (wake to sleep) counter
 *    Bit11: Peer STA power save counter, total time in PS mode
 *    Bit12: Probe request counter
 *    Bit13: Other management frames counter
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CCA_BSS_BITMAP: bitmap for CCA
 *    Bit0: Idle time
 *    Bit1: TX time
 *    Bit2: time RX in current bss
 *    Bit3: Out of current bss time
 *    Bit4: Wireless medium busy time
 *    Bit5: RX in bad condition time
 *    Bit6: TX in bad condition time
 *    Bit7: time wlan card not availbe
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_SIGNAL_BITMAP: bitmap for signal
 *    Bit0: Per channel SNR counter
 *    Bit1: Per channel noise floor counter
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_NUM: number of peers
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CHANNEL_NUM: number of channels
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_AC_RX_NUM: number of RX stats
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CCA_BSS: per channel BSS CCA stats
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER: container for per PEER stats
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MSDU: Number of total TX MSDUs
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MPDU: Number of total TX MPDUs
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_PPDU: Number of total TX PPDUs
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BYTES: bytes of TX data
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP: Number of dropped TX packets
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP_BYTES: Bytes dropped
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_RETRY: waiting time without an ACK
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_NO_ACK: number of MPDU not-ACKed
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_NO_BACK: number of PPDU not-ACKed
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_AGGR_NUM:
 *    aggregation stats buffer length
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_SUCC_MCS_NUM: length of mcs stats
 *    buffer for ACKed MPDUs.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_FAIL_MCS_NUM: length of mcs stats
 *    buffer for failed MPDUs.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_DELAY_ARRAY_SIZE:
 *    length of delay stats array.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_AGGR: TX aggregation stats
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_SUCC_MCS: MCS stats for ACKed MPDUs
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_FAIL_MCS: MCS stats for failed MPDUs
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DELAY: tx delay stats
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU: MPDUs received
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_BYTES: bytes received
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU: PPDU received
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU_BYTES: PPDU bytes received
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_LOST: packets lost
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_RETRY: number of RX packets
 *    flagged as retransmissions
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DUP: number of RX packets
 *    flagged as duplicated
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DISCARD: number of RX
 *    packets discarded
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_AGGR_NUM: length of RX aggregation
 *    stats buffer.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MCS_NUM: length of RX mcs
 *    stats buffer.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MCS: RX mcs stats buffer
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_AGGR: aggregation stats buffer
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_TIMES: times STAs go to sleep
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_DURATION: STAS' total sleep time
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PROBE_REQ: number of probe
 *    requests received
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MGMT: number of other mgmt
 *    frames received
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IDLE_TIME: Percentage of idle time
 *    there is no TX, nor RX, nor interference.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_TIME: percentage of time
 *    transmitting packets.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_TIME: percentage of time
 *    for receiving.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BUSY: percentage of time
 *    interference detected.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BAD: percentage of time
 *    receiving packets with errors.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BAD: percentage of time
 *    TX no-ACK.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_NO_AVAIL: percentage of time
 *    the chip is unable to work in normal conditions.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IN_BSS_TIME: percentage of time
 *    receiving packets in current BSS.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_OUT_BSS_TIME: percentage of time
 *    receiving packets not in current BSS.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_ANT_NUM: number of antennas
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_SIGNAL:
 *    This is a container for per antenna signal stats.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_SNR: per antenna SNR value
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_NF: per antenna NF value
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IFACE_RSSI_BEACON: RSSI of beacon
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IFACE_SNR_BEACON: SNR of beacon
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_REPORT_TIME: u64
 *    Absolute timestamp from 1970/1/1, unit in ms. After receiving the
 *    message, user layer APP could call gettimeofday to get another
 *    timestamp and calculate transfer delay for the message.
 * @QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_MEASUREMENT_TIME: u32
 *    Real period for this measurement, unit in us.
 */
enum qca_wlan_vendor_attr_ll_stats_ext {
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_INVALID = 0,

	/* Attributes for configurations */
	QCA_WLAN_VENDOR_ATTR_LL_STATS_CFG_PERIOD,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_CFG_THRESHOLD,

	/* Peer STA power state change */
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_CHG,

	/* TX failure event */
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TID,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_NUM_MSDU,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_STATUS,

	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_STATE,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_MAC_ADDRESS,

	/* MAC counters */
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_GLOBAL,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_EVENT_MODE,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IFACE_ID,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_ID,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BITMAP,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BITMAP,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CCA_BSS_BITMAP,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_SIGNAL_BITMAP,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_NUM,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CHANNEL_NUM,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CCA_BSS,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER,

	/* Sub-attribute for PEER_AC_TX */
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MSDU,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MPDU,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_PPDU,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BYTES,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP_BYTES,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_RETRY,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_NO_ACK,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_NO_BACK,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_AGGR_NUM,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_SUCC_MCS_NUM,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_FAIL_MCS_NUM,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_AGGR,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_SUCC_MCS,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_FAIL_MCS,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_DELAY_ARRAY_SIZE,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DELAY,

	/* sub-attribute for PEER_AC_RX */
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_BYTES,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU_BYTES,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_LOST,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_RETRY,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DUP,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DISCARD,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_AGGR_NUM,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MCS_NUM,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MCS,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_AGGR,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_TIMES,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_DURATION,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PROBE_REQ,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MGMT,

	/* sub-attribute for CCA_BSS */
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IDLE_TIME,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_TIME,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_TIME,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BUSY,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BAD,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BAD,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_NO_AVAIL,

	/* sub-attribtue for BSS_RX_TIME */
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IN_BSS_TIME,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_OUT_BSS_TIME,

	/* sub-attribute for PEER_SIGNAL */
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_ANT_NUM,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_SIGNAL,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_SNR,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_NF,

	/* sub-attribute for IFACE_BSS */
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IFACE_RSSI_BEACON,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IFACE_SNR_BEACON,

	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_REPORT_TIME,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_MEASUREMENT_TIME,

	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_LAST,
	QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_MAX =
		QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_LAST - 1,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,7,0))
/**
 * enum ieee80211_band - supported frequency bands
 *
 * The bands are assigned this way because the supported
 * bitrates differ in these bands.
 *
 * @IEEE80211_BAND_2GHZ: 2.4GHz ISM band
 * @IEEE80211_BAND_5GHZ: around 5GHz band (4.9-5.7)
 * @IEEE80211_BAND_60GHZ: around 60 GHz band (58.32 - 64.80 GHz)
 * @IEEE80211_NUM_BANDS: number of defined bands
 */
enum ieee80211_band {
	IEEE80211_BAND_2GHZ = NL80211_BAND_2GHZ,
	IEEE80211_BAND_5GHZ = NL80211_BAND_5GHZ,
	IEEE80211_BAND_60GHZ = NL80211_BAND_60GHZ,

	/* keep last */
	IEEE80211_NUM_BANDS
};
#endif

struct cfg80211_bss* wlan_hdd_cfg80211_update_bss_db( hdd_adapter_t *pAdapter,
                                      tCsrRoamInfo *pRoamInfo
                                      );

int wlan_hdd_cfg80211_update_bss(struct wiphy *wiphy,
			hdd_adapter_t *pAdapter);

#ifdef FEATURE_WLAN_LFR
int wlan_hdd_cfg80211_pmksa_candidate_notify(
                    hdd_adapter_t *pAdapter, tCsrRoamInfo *pRoamInfo,
                    int index, bool preauth );
#endif

#ifdef FEATURE_WLAN_LFR_METRICS
VOS_STATUS wlan_hdd_cfg80211_roam_metrics_preauth(hdd_adapter_t *pAdapter,
                                                  tCsrRoamInfo *pRoamInfo);

VOS_STATUS wlan_hdd_cfg80211_roam_metrics_preauth_status(
    hdd_adapter_t *pAdapter, tCsrRoamInfo *pRoamInfo, bool preauth_status);

VOS_STATUS wlan_hdd_cfg80211_roam_metrics_handover(hdd_adapter_t *pAdapter,
                                                   tCsrRoamInfo *pRoamInfo);
#endif

#ifdef FEATURE_WLAN_WAPI
void wlan_hdd_cfg80211_set_key_wapi(hdd_adapter_t* pAdapter,
                                    u8 key_index, const u8 *mac_addr,
                                    const u8 *key , int key_Len);
#endif
struct wiphy *wlan_hdd_cfg80211_wiphy_alloc(int priv_size);

int wlan_hdd_cfg80211_scan( struct wiphy *wiphy,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)) && !defined(WITH_BACKPORTS)
                            struct net_device *dev,
#endif
                            struct cfg80211_scan_request *request);

int wlan_hdd_cfg80211_init(struct device *dev,
                               struct wiphy *wiphy,
                               hdd_config_t *pCfg
                                         );
void wlan_hdd_cfg80211_deinit(struct wiphy *wiphy);

void wlan_hdd_update_wiphy(struct wiphy *wiphy,
                           hdd_context_t *ctx);

int wlan_hdd_cfg80211_register( struct wiphy *wiphy);
void wlan_hdd_cfg80211_register_frames(hdd_adapter_t* pAdapter);

void wlan_hdd_cfg80211_deregister_frames(hdd_adapter_t* pAdapter);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)) || defined(WITH_BACKPORTS)
void wlan_hdd_linux_reg_notifier(struct wiphy *wiphy, struct regulatory_request *request);
#else
int wlan_hdd_linux_reg_notifier(struct wiphy *wiphy, struct regulatory_request *request);
#endif

extern v_VOID_t hdd_connSetConnectionState(hdd_adapter_t *pAdapter,
                                            eConnectionState connState);
VOS_STATUS wlan_hdd_validate_operation_channel(hdd_adapter_t *pAdapter,int channel);
#ifdef FEATURE_WLAN_TDLS
int wlan_hdd_cfg80211_send_tdls_discover_req(struct wiphy *wiphy,
                            struct net_device *dev, u8 *peer);
#endif
#ifdef WLAN_FEATURE_GTK_OFFLOAD
extern void wlan_hdd_cfg80211_update_replayCounterCallback(void *callbackContext,
                            tpSirGtkOffloadGetInfoRspParams pGtkOffloadGetInfoRsp);
#endif
void* wlan_hdd_change_country_code_cb(void *pAdapter);
void hdd_select_cbmode(hdd_adapter_t *pAdapter, v_U8_t operationChannel,
                uint16_t *ch_width);
void hdd_select_mon_cbmode(hdd_adapter_t *adapter, v_U8_t operation_channel,
                           uint16_t *ch_width);

v_U8_t* wlan_hdd_cfg80211_get_ie_ptr(const v_U8_t *pIes,
                                     int length,
                                     v_U8_t eid);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
#define CFG80211_DEL_STA_V2
#endif

#ifdef CFG80211_DEL_STA_V2
int wlan_hdd_cfg80211_del_station(struct wiphy *wiphy,
                                  struct net_device *dev,
                                  struct station_del_parameters *param);
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)) || defined(WITH_BACKPORTS)
int wlan_hdd_cfg80211_del_station(struct wiphy *wiphy,
                                  struct net_device *dev, const u8 *mac);
#else
int wlan_hdd_cfg80211_del_station(struct wiphy *wiphy,
                                  struct net_device *dev, u8 *mac);
#endif
#endif

#if  defined(QCA_WIFI_FTM)     && defined(CONFIG_NL80211_TESTMODE)
void wlan_hdd_testmode_rx_event(void *buf, size_t buf_len);
#endif

void hdd_suspend_wlan(void (*callback)(void *callbackContext, boolean suspended),
                      void *callbackContext, bool thermal);
void hdd_resume_wlan(bool thermal);

#if defined(FEATURE_WLAN_CH_AVOID) || defined(FEATURE_WLAN_FORCE_SAP_SCC)
int wlan_hdd_send_avoid_freq_event(hdd_context_t *pHddCtx,
                                   tHddAvoidFreqList *pAvoidFreqList);
#endif /* FEATURE_WLAN_CH_AVOID || FEATURE_WLAN_FORCE_SAP_SCC*/

#ifdef FEATURE_WLAN_EXTSCAN
void wlan_hdd_cfg80211_extscan_callback(void *ctx,
                                      const tANI_U16 evType,
                                      void *pMsg);
#endif /* FEATURE_WLAN_EXTSCAN */

void wlan_hdd_cfg80211_chainrssi_callback(void *ctx, void *pmsg);

void hdd_rssi_threshold_breached(void *hddctx,
				 struct rssi_breach_event *data);

struct cfg80211_bss* wlan_hdd_cfg80211_update_bss_list(
   hdd_adapter_t *pAdapter, tSirMacAddr bssid);
int __wlan_hdd_cfg80211_suspend_wlan(struct wiphy *wiphy,
                                   struct cfg80211_wowlan *wow, bool thermal);

int wlan_hdd_cfg80211_suspend_wlan(struct wiphy *wiphy,
                                   struct cfg80211_wowlan *wow);
int __wlan_hdd_cfg80211_resume_wlan(struct wiphy *wiphy, bool thermal);
int wlan_hdd_cfg80211_resume_wlan(struct wiphy *wiphy);

bool hdd_system_suspend_state_set(hdd_context_t *hdd_ctx, bool state);
int hdd_thermal_suspend_state(hdd_context_t *hdd_ctx);

void wlan_hdd_cfg80211_acs_ch_select_evt(hdd_adapter_t *adapter);
#ifdef WLAN_FEATURE_ROAM_OFFLOAD
int wlan_hdd_send_roam_auth_event(hdd_context_t *hdd_ctx_ptr, uint8_t *bssid,
		uint8_t *req_rsn_ie, uint32_t req_rsn_length,
		uint8_t *rsp_rsn_ie, uint32_t rsp_rsn_length,
		tCsrRoamInfo *roam_info_ptr);
#else
static inline int wlan_hdd_send_roam_auth_event(hdd_context_t *hdd_ctx_ptr,
	uint8_t *bssid, uint8_t *req_rsn_ie, uint32_t req_rsn_length,
		uint8_t *rsp_rsn_ie, uint32_t rsp_rsn_length,
		tCsrRoamInfo *roam_info_ptr)
{
	return 0;
}
#endif
int wlan_hdd_disable_dfs_chan_scan(hdd_context_t *pHddCtx,
                                   hdd_adapter_t *pAdapter,
                                   u32 no_dfs_flag);

int wlan_hdd_cfg80211_update_apies(hdd_adapter_t* pHostapdAdapter);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
#define SUPPORT_WDEV_CFG80211_VENDOR_EVENT_ALLOC
#endif

#if !(defined (SUPPORT_WDEV_CFG80211_VENDOR_EVENT_ALLOC))
static inline struct sk_buff *
backported_cfg80211_vendor_event_alloc(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					int approxlen,
					int event_idx, gfp_t gfp)
{
	return cfg80211_vendor_event_alloc(wiphy, approxlen, event_idx, gfp);
}
#define cfg80211_vendor_event_alloc backported_cfg80211_vendor_event_alloc
#endif

void hdd_get_bpf_offload_cb(void *hdd_context, struct sir_bpf_get_offload *);
void hdd_init_bpf_completion(void);

#ifdef WLAN_FEATURE_LINK_LAYER_STATS
void wlan_hdd_clear_link_layer_stats(hdd_adapter_t *adapter);
#else
static inline void wlan_hdd_clear_link_layer_stats(hdd_adapter_t *adapter) {}
#endif

struct cfg80211_bss *wlan_hdd_cfg80211_inform_bss_frame(hdd_adapter_t *pAdapter,
		tSirBssDescription *bss_desc);

#if defined(CFG80211_DISCONNECTED_V2) || \
(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0))
static inline void wlan_hdd_cfg80211_indicate_disconnect(struct net_device *dev,
							bool locally_generated,
							int reason)
{
	cfg80211_disconnected(dev, reason, NULL, 0,
				locally_generated, GFP_KERNEL);
}
#else
static inline void wlan_hdd_cfg80211_indicate_disconnect(struct net_device *dev,
							bool locally_generated,
							int reason)
{
	cfg80211_disconnected(dev, reason, NULL, 0,
				GFP_KERNEL);
}
#endif

/**
 * enum wlan_hdd_scan_type_for_randomization - type of scan
 * @WLAN_HDD_HOST_SCAN: refers to scan request from cfg80211_ops "scan"
 * @WLAN_HDD_PNO_SCAN: refers to scan request is from "sched_scan_start"
 *
 * driver uses this enum to identify source of scan
 *
 */
enum wlan_hdd_scan_type_for_randomization {
	WLAN_HDD_HOST_SCAN,
	WLAN_HDD_PNO_SCAN,
};

int wlan_hdd_try_disconnect(hdd_adapter_t *pAdapter);

/**
 * wlan_hdd_cfg80211_scan_block_cb() - scan block work handler
 * @work: Pointer to work
 *
 * This function is used to do scan block work handler
 *
 * Return: None
 */
void wlan_hdd_cfg80211_scan_block_cb(struct work_struct *work);
#endif
