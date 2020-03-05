
/** @file moal_eth_ioctl.h
 *
 * @brief This file contains definition for private IOCTL call.
 *
  *
  * Copyright 2014-2020 NXP
  *
  * This software file (the File) is distributed by NXP
  * under the terms of the GNU General Public License Version 2, June 1991
  * (the License).  You may use, redistribute and/or modify the File in
  * accordance with the terms and conditions of the License, a copy of which
  * is available by writing to the Free Software Foundation, Inc.,
  * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
  * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
  *
  * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
  * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
  * this warranty disclaimer.
  *
  */

/********************************************************
Change log:
    01/05/2012: initial version
********************************************************/
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#include    "moal_cfg80211.h"
#endif

#ifndef _WOAL_ETH_PRIV_H_
#define _WOAL_ETH_PRIV_H_

/** Command disabled */
#define CMD_DISABLED                    0
/** Command enabled */
#define CMD_ENABLED                     1
/** Command get */
#define CMD_GET                         2

/** 2K bytes */
#define WOAL_2K_BYTES       2000

/** NXP private command identifier string */
#define CMD_NXP     "MRVL_CMD"

/** Private command: Version */
#define PRIV_CMD_VERSION    "version"
/** Private command: Band cfg */
#define PRIV_CMD_BANDCFG    "bandcfg"
/** Private command: Host cmd */
#define PRIV_CMD_HOSTCMD    "hostcmd"
/** Private command: Custom IE config*/
#define PRIV_CMD_CUSTOMIE   "customie"
/** Private command: HT Tx Cfg */
#define PRIV_CMD_HTTXCFG    "httxcfg"
/** Private command: HT Cap Info */
#define PRIV_CMD_HTCAPINFO  "htcapinfo"
/** Private command: Add BA para */
#define PRIV_CMD_ADDBAPARA  "addbapara"
/** Private command: Aggragation priority table */
#define PRIV_CMD_AGGRPRIOTBL    "aggrpriotbl"
/** Private command: Add BA reject cfg */
#define PRIV_CMD_ADDBAREJECT    "addbareject"
/** Private command: Delete BA */
#define PRIV_CMD_DELBA      "delba"
/** Private command: Reject Addba Req */
#define PRIV_CMD_REJECTADDBAREQ  "rejectaddbareq"
/** Private command: 11AC Cfg */
#define PRIV_CMD_VHTCFG     "vhtcfg"
/** Private command: 11AC Oper Mode Cfg */
#define PRIV_CMD_OPERMODECFG     "opermodecfg"
#define PRIV_CMD_DATARATE   "getdatarate"
#define PRIV_CMD_TXRATECFG  "txratecfg"
#define PRIV_CMD_GETLOG     "getlog"

#define PRIV_CMD_DPDLOG_ENABLE  "dpdlog"
#define PRIV_CMD_GET_DPDLOG     "getdpdlog"

#define PRIV_CMD_ESUPPMODE  "esuppmode"
#define PRIV_CMD_PASSPHRASE "passphrase"
#define PRIV_CMD_DEAUTH     "deauth"
#ifdef UAP_SUPPORT
#define PRIV_CMD_AP_DEAUTH     "apdeauth"
#define PRIV_CMD_GET_STA_LIST     "getstalist"
#define PRIV_CMD_BSS_CONFIG   "bssconfig"
#endif
#if defined(WIFI_DIRECT_SUPPORT)
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
#define PRIV_CMD_BSSROLE    "bssrole"
#endif
#endif
#ifdef STA_SUPPORT
#define PRIV_CMD_GETSCANTABLE   "getscantable"
#define PRIV_CMD_SETUSERSCAN    "setuserscan"
#define PRIV_CMD_EXTCAPCFG      "extcapcfg"
#define PRIV_CMD_CANCELSCAN     "cancelscan"
#endif
#define PRIV_CMD_DEEPSLEEP      "deepsleep"
#define PRIV_CMD_IPADDR         "ipaddr"
#define PRIV_CMD_WPSSESSION     "wpssession"
#define PRIV_CMD_OTPUSERDATA    "otpuserdata"
#define PRIV_CMD_COUNTRYCODE    "countrycode"
#define PRIV_CMD_TCPACKENH      "tcpackenh"
#ifdef REASSOCIATION
#define PRIV_CMD_ASSOCESSID      "assocessid"
#define PRIV_CMD_ASSOCBSSID      "assocessid_bssid"
#endif
#define PRIV_CMD_WAKEUPREASON     "wakeupreason"
#ifdef STA_SUPPORT
#define PRIV_CMD_LISTENINTERVAL "listeninterval"
#endif
#ifdef DEBUG_LEVEL1
#define PRIV_CMD_DRVDBG         "drvdbg"
#endif
#define PRIV_CMD_HSCFG          "hscfg"
#define PRIV_CMD_HSSETPARA      "hssetpara"
#define PRIV_CMD_MGMT_FILTER    "mgmtfilter"
#define PRIV_CMD_SCANCFG        "scancfg"
#define PRIV_CMD_SET_BSS_MODE   "setbssmode"
#ifdef STA_SUPPORT
#define PRIV_CMD_SET_AP         "setap"
#define PRIV_CMD_SET_POWER      "setpower"
#define PRIV_CMD_SET_ESSID      "setessid"
#define PRIV_CMD_SET_AUTH       "setauth"
#define PRIV_CMD_GET_AP         "getap"
#define PRIV_CMD_GET_POWER      "getpower"
#define PRIV_CMD_PSMODE         "psmode"
#endif
#define PRIV_CMD_WARMRESET      "warmreset"
#define PRIV_CMD_TXPOWERCFG     "txpowercfg"
#define PRIV_CMD_PSCFG          "pscfg"
#define PRIV_CMD_BCNTIMEOUTCFG     "bcntimeoutcfg"
#define PRIV_CMD_SLEEPPD        "sleeppd"
#define PRIV_CMD_TXCONTROL      "txcontrol"
#define PRIV_CMD_REGRDWR        "regrdwr"
#define PRIV_CMD_RDEEPROM       "rdeeprom"
#define PRIV_CMD_MEMRDWR        "memrdwr"
#define PRIV_CMD_ARPFILTER      "arpfilter"
#define PRIV_CMD_MGMT_FRAME_CTRL  "mgmtframectrl"
#define PRIV_CMD_QCONFIG        "qconfig"
#define PRIV_CMD_ADDTS          "addts"
#define PRIV_CMD_DELTS          "delts"
#define PRIV_CMD_QSTATUS        "qstatus"
#define PRIV_CMD_TS_STATUS      "ts_status"
#define PRIV_CMD_QOS_CFG        "qoscfg"
#define PRIV_CMD_MAC_CTRL       "macctrl"
#define PRIV_CMD_GETWAP         "getwap"
#define PRIV_CMD_REGION_CODE    "regioncode"
#define PRIV_CMD_CFPINFO        "cfpinfo"
#define PRIV_CMD_FWMACADDR      "fwmacaddr"
#define PRIV_CMD_OFFCHANNEL     "offchannel"
#define PRIV_CMD_DSCP_MAP       "dscpmap"
/** Private command: Verext */
#define PRIV_CMD_VEREXT         "verext"
#if defined(STA_SUPPORT) && defined(STA_WEXT)
#define PRIV_CMD_RADIO_CTRL     "radioctrl"
#endif
#define PRIV_CMD_WMM_CFG        "wmmcfg"
#if defined(STA_SUPPORT)
#define PRIV_CMD_11D_CFG        "11dcfg"
#define PRIV_CMD_11D_CLR_TBL    "11dclrtbl"
#endif
#define PRIV_CMD_WWS_CFG        "wwscfg"
#if defined(REASSOCIATION)
#define PRIV_CMD_REASSOCTRL     "reassoctrl"
#endif
#define PRIV_CMD_TXBUF_CFG          "txbufcfg"
#ifdef STA_SUPPORT
#define PRIV_CMD_AUTH_TYPE          "authtype"
#endif
#define PRIV_CMD_POWER_CONS         "powercons"
#define PRIV_CMD_HT_STREAM_CFG      "htstreamcfg"
#define PRIV_CMD_MIMO_SWITCH        "mimoswitch"
#define PRIV_CMD_THERMAL            "thermal"
#define PRIV_CMD_BCN_INTERVAL       "bcninterval"
#ifdef STA_SUPPORT
#define PRIV_CMD_GET_SIGNAL         "getsignal"
#define PRIV_CMD_SIGNALEXT_CFG      "signalextcfg"
#define PRIV_CMD_GET_SIGNAL_EXT_V2  "getsignalextv2"
#define PRIV_CMD_GET_SIGNAL_EXT     "getsignalext"
#endif
#if defined(STA_SUPPORT)
#define PRIV_CMD_PMFCFG         "pmfcfg"
#endif
#define PRIV_CMD_INACTIVITYTO   "inactivityto"
#define PRIV_CMD_AMSDU_AGGR_CTRL    "amsduaggrctrl"
#define PRIV_CMD_TX_BF_CAP          "httxbfcap"
#define PRIV_CMD_SLEEP_PARAMS       "sleepparams"
#define PRIV_CMD_NET_MON            "netmon"
#if defined(DFS_TESTING_SUPPORT)
#define PRIV_CMD_DFS_TESTING        "dfstesting"
#endif
#define PRIV_CMD_CFP_CODE           "cfpcode"
#define PRIV_CMD_CWMODE             "cwmode"
#define PRIV_CMD_ANT_CFG            "antcfg"
#define PRIV_CMD_SYSCLOCK       "sysclock"
#define PRIV_CMD_ASSOCIATE      "associate"
#define PRIV_CMD_TX_BF_CFG      "httxbfcfg"
#define PRIV_CMD_PORT_CTRL      "port_ctrl"
#define PRIV_CMD_PB_BYPASS      "pb_bypass"
#define PRIV_CMD_FW_WAKEUP_METHOD   "fwwakeupmethod"
#ifdef RX_PACKET_COALESCE
#define PRIV_CMD_RX_COAL_CFG "rxpktcoal_cfg"
#endif
#define PRIV_CMD_MULTI_CHAN_CFG "mc_cfg"
#define PRIV_CMD_MULTI_CHAN_POLICY "mc_policy"
#define PRIV_CMD_DRCS_CFG "mc_cfg_ext"
#if defined(WIFI_DIRECT_SUPPORT)
#if defined(UAP_CFG80211)
#define PRIV_CMD_CFG_NOA            "cfg_noa"
#define PRIV_CMD_CFG_OPP_PS         "cfg_opp_ps"
#endif
#endif
#define PRIV_CMD_DFS_REPEATER_CFG       "dfs_repeater"
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#define PRIV_CMD_MIRACAST_CFG       "miracastcfg"
#endif
#endif
#define PRIV_CMD_COEX_RX_WINSIZE    "coex_rx_winsize"
#define PRIV_CMD_TX_AGGR_CTRL "txaggrctrl"
#define PRIV_CMD_AUTO_TDLS          "autotdls"

#define PRIV_CMD_PCIE_REG_RW        "pcieregrw"
#define PRIV_CMD_PCIE_BAR0_REG_RW        "pciebar0regrw"

#define PRIV_CMD_GET_SENSOR_TEMP        "get_sensor_temp"

#define PRIV_CMD_GET_CHNRGPWR        	"get_chnrgpwr"
#define PRIV_CMD_GET_TXPWR_LIMIT        "get_txpwrlimit"
#define PRIV_CMD_GET_CFG_CHAN_LIST      "getcfgchanlist"

#define PRIV_CMD_11K_CFG        "11k_enable"
#define PRIV_CMD_11K_NEIGHBOR_REPORT        "neighbor_report"
#if defined(UAP_SUPPORT)
#define PRIV_CMD_EXTEND_CHAN_SWITCH        "channel_switch"
#endif

#define PRIV_CMD_TDLS_IDLE_TIME          "tdls_idle_time"

#define PRIV_CMD_DYN_BW          "dyn_bw"

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
#define PRIV_CMD_DFS_OFFLOAD            "dfs_offload"
#endif

#define PRIV_CMD_AUTO_ARP	"auto_arp"

#define PRIV_CMD_PER_PKT_CFG    "per_pkt_cfg"

#define PRIV_CMD_DEAUTH_CTRL    "ctrldeauth"

#define PRIV_CMD_TX_RX_HISTOGRAM        "txrxhistogram"

/**Private command ID to set/get independent reset*/
#define PRIV_CMD_IND_RST_CFG            "indrstcfg"

#define TX_AMPDU_RTS_CTS              0
#define TX_AMPDU_CTS_2_SELF           1
#define TX_AMPDU_DISABLE_PROTECTION   2
/**Private command to set tx ampdu protection mode */
#define PRIV_CMD_TX_AMPDU_PROT_MODE   "tx_ampdu_prot_mode"

/** Private command ID for Android default commands */
#define WOAL_ANDROID_DEF_CMD        (SIOCDEVPRIVATE + 1)

/** Private command ID to send TLD configuration */
#define WOAL_TDLS_CONFIG            (SIOCDEVPRIVATE + 5)

/** Private command ID to pass mgmt frame */
#define WOAL_MGMT_FRAME_TX          WOAL_MGMT_FRAME_TX_IOCTL

/** Private command ID to pass custom IE list */
#define WOAL_CUSTOM_IE_CFG          (SIOCDEVPRIVATE + 13)

/** Private command ID for Android ICS priv CMDs */
#define WOAL_ANDROID_PRIV_CMD       (SIOCDEVPRIVATE + 14)

/** Private command ID to get BSS type */
#define WOAL_GET_BSS_TYPE           (SIOCDEVPRIVATE + 15)

/** Private command ID for robustcoex */
#define PRIV_CMD_ROBUSTCOEX           "robustcoex"

#define PRIV_CMD_BOOTSLEEP            "bootsleep"

#define PRIV_CMD_GET_CORRELATED_TIME "GET_CORRELATED_TIME"

int woal_do_ioctl(struct net_device *dev, struct ifreq *req, int cmd);

/*
 * For android private commands, fixed value of ioctl is used.
 * Internally commands are differentiated using strings.
 *
 * application needs to specify "total_len" of data for copy_from_user
 * kernel updates "used_len" during copy_to_user
 */
/** Private command structure from app */
#ifdef USERSPACE_32BIT_OVER_KERNEL_64BIT
typedef struct _android_wifi_priv_cmd {
    /** Buffer pointer */
	t_u64 buf;
    /** buffer updated by driver */
	int used_len;
    /** buffer sent by application */
	int total_len;
} __attribute__ ((packed))
     android_wifi_priv_cmd;
#else
typedef struct _android_wifi_priv_cmd {
    /** Buffer pointer */
	char *buf;
    /** buffer updated by driver */
	int used_len;
    /** buffer sent by application */
	int total_len;
} android_wifi_priv_cmd;
#endif

#ifndef IFNAMSIZ
#define IFNAMSIZ 16
#endif

/* Maximum size of the ESSID and NICKN strings */
#define MW_ESSID_MAX_SIZE   32

/* Modes of operation */
#define MW_MODE_AUTO    0	/* Let the driver decides */
#define MW_MODE_ADHOC   1	/* Single cell network */
#define MW_MODE_INFRA   2	/* Multi cell network, roaming, ... */
#define MW_MODE_MASTER  3	/* Synchronisation master or Access Point */
#define MW_MODE_REPEAT  4	/* Wireless Repeater (forwarder) */
#define MW_MODE_SECOND  5	/* Secondary master/repeater (backup) */
#define MW_MODE_MONITOR 6	/* Passive monitor (listen only) */
#define MW_MODE_MESH    7	/* Mesh (IEEE 802.11s) network */

#define MW_POWER_TYPE       0xF000	/* Type of parameter */
#define MW_POWER_PERIOD     0x1000	/* Value is a period/duration of  */
#define MW_POWER_TIMEOUT    0x2000	/* Value is a timeout (to go asleep) */

#define MW_AUTH_INDEX       0x0FFF
#define MW_AUTH_FLAGS       0xF000
#define MW_AUTH_WPA_VERSION     0
#define MW_AUTH_CIPHER_PAIRWISE     1
#define MW_AUTH_CIPHER_GROUP        2
#define MW_AUTH_KEY_MGMT        3
#define MW_AUTH_TKIP_COUNTERMEASURES    4
#define MW_AUTH_DROP_UNENCRYPTED    5
#define MW_AUTH_80211_AUTH_ALG      6
#define MW_AUTH_WPA_ENABLED     7
#define MW_AUTH_RX_UNENCRYPTED_EAPOL    8
#define MW_AUTH_ROAMING_CONTROL     9
#define MW_AUTH_PRIVACY_INVOKED     10
#define MW_AUTH_CIPHER_GROUP_MGMT   11
#define MW_AUTH_MFP         12

#define MW_AUTH_CIPHER_NONE 0x00000001
#define MW_AUTH_CIPHER_WEP40    0x00000002
#define MW_AUTH_CIPHER_TKIP 0x00000004
#define MW_AUTH_CIPHER_CCMP 0x00000008
#define MW_AUTH_CIPHER_WEP104   0x00000010
#define MW_AUTH_CIPHER_AES_CMAC 0x00000020

#define MW_AUTH_ALG_OPEN_SYSTEM 0x00000001
#define MW_AUTH_ALG_SHARED_KEY  0x00000002
#define MW_AUTH_ALG_LEAP    0x00000004

/* Generic format for most parameters that fit in an int */
struct mw_param {
  /** The value of the parameter itself */
	t_s32 value;
  /** Hardware should not use auto select */
	t_u8 fixed;
  /** Disable the feature */
	t_u8 disabled;
  /** Various specifc flags (if any) */
	t_u16 flags;
};

/*
 *  For all data larger than 16 octets, we need to use a
 *  pointer to memory allocated in user space.
 */
struct mw_point {
  /** Pointer to the data  (in user space) */
	t_u8 *pointer;
  /** number of fields or size in bytes */
	t_u16 length;
  /** Optional params */
	t_u16 flags;
};

/*
 * This structure defines the payload of an ioctl, and is used
 * below.
 */
union mwreq_data {
    /** Config - generic */
	char name[IFNAMSIZ];

    /** Extended network name */
	struct mw_point essid;
    /** Operation mode */
	t_u32 mode;
    /** PM duration/timeout */
	struct mw_param power;
    /** Access point address */
	struct sockaddr ap_addr;
    /** Other small parameters */
	struct mw_param param;
    /** Other large parameters */
	struct mw_point data;
};

 /* The structure to exchange data for ioctl */
struct mwreq {
	union {
	/** if name, e.g. "eth0" */
		char ifrn_name[IFNAMSIZ];
	} ifr_ifrn;

    /** Data part */
	union mwreq_data u;
};

/** woal HT capacity info structure */
typedef struct woal_priv_ht_cap_info {
    /** HT capacity info for bg */
	t_u32 ht_cap_info_bg;
    /** HT capacity info for a */
	t_u32 ht_cap_info_a;
} woal_ht_cap_info;

/** woal private addba structure */
typedef struct woal_priv_addba {
    /** Time out */
	t_u32 time_out;
    /** Transmission window size */
	t_u32 tx_win_size;
    /** Receiver window size */
	t_u32 rx_win_size;
    /** Tx amsdu */
	t_u32 tx_amsdu;
    /** Rx amsdu */
	t_u32 rx_amsdu;
} woal_addba;

/** data structure for cmd txratecfg */
typedef struct woal_priv_tx_rate_cfg {
   /** LG rate: 0, HT rate: 1, VHT rate: 2 */
	t_u32 rate_format;
    /** Rate/MCS index (0xFF: auto) */
	t_u32 rate_index;
    /** Data rate */
	t_u32 rate;
    /** NSS */
	t_u32 nss;
} woal_tx_rate_cfg;

/** woal embedded supplicant structure */
typedef struct woal_priv_esuppmode_cfg {
    /** RSN mode */
	t_u16 rsn_mode;
    /** Pairwise cipher */
	t_u8 pairwise_cipher;
    /** Group cipher */
	t_u8 group_cipher;
} woal_esuppmode_cfg;

mlan_status woal_set_ap_wps_p2p_ie(moal_private *priv, t_u8 *ie, size_t len);
mlan_status woal_ioctl_aggr_prio_tbl(moal_private *priv, t_u32 action,
				     mlan_ds_11n_aggr_prio_tbl *aggr_prio_tbl);

int woal_android_priv_cmd(struct net_device *dev, struct ifreq *req);

#define PRIV_CMD_ACS_SCAN "acs"

/** Type definition of mlan_acs_scan */
typedef struct _acs_result {
    /** Best Channel Number */
	t_u8 best_ch;
    /** Channel Statistics Number */
	t_u8 ch_stats_num;
    /** Channel Statistics */
	ChStat_t ch_stats[0];
} acs_result, *pacs_result;

#define TLV_TYPE_PER_PKT_CFG 0x0001
#define TX_PKT_CTRL  MBIT(0)
#define RX_PKT_INFO  MBIT(1)

#define  FLAG_TX_HISTOGRAM            0x01
#define  FLAG_RX_HISTOGRAM            0x02
#define  DISABLE_TX_RX_HISTOGRAM      0x00
#define  ENABLE_TX_RX_HISTOGRAM       0x01
#define  GET_TX_RX_HISTOGRAM          0x02
#define  PRIV_CMD_TX_RX_HISTOGRAM     "txrxhistogram"
/** TX and RX histogram statistic parameters*/
typedef struct _tx_rx_histogram {
    /** Enable or disable get tx/rx histogram statistic */
	t_u8 enable;
    /** Choose to get TX, RX or both histogram statistic */
	t_u8 action;
} __ATTRIB_PACK__ tx_rx_histogram;

/** Enum for different CW mode type */
typedef enum _cw_modes_e {
	CWMODE_DISABLE,
	CWMODE_TXCONTPKT,
	CWMODE_TXCONTWAVE,
} cw_modes_e;

/** wlan_ieee80211_chan */
typedef struct {
    /** center freq */
	t_u16 center_freq;
    /** chan num */
	t_u16 hw_value;
    /** chan flags */
	t_u32 flags;
    /** max power */
	int max_power;
} __ATTRIB_PACK__ wlan_ieee80211_chan;

/** wlan_ieee80211_chan_list*/
typedef struct {
    /** num of chan */
	t_u8 num_chan;
    /** chan_list */
	wlan_ieee80211_chan chan_list[0];
} __ATTRIB_PACK__ wlan_ieee80211_chan_list;
#endif /** _WOAL_ETH_PRIV_H_ */
