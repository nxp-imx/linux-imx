
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
#include "moal_cfg80211.h"
#endif

#ifndef _WOAL_ETH_PRIV_H_
#define _WOAL_ETH_PRIV_H_

/** Command disabled */
#define CMD_DISABLED 0
/** Command enabled */
#define CMD_ENABLED 1
/** Command get */
#define CMD_GET 2

/** 2K bytes */
#define WOAL_2K_BYTES 2000

/** NXP private command identifier string */
#define CMD_NXP "MRVL_CMD"

/** Private command: Version */
#define PRIV_CMD_VERSION "version"
/** Private command: Band cfg */
#define PRIV_CMD_BANDCFG "bandcfg"
/** Private command: Host cmd */
#define PRIV_CMD_HOSTCMD "hostcmd"
/** Private command: Custom IE config*/
#define PRIV_CMD_CUSTOMIE "customie"
/** Private command: HT Tx Cfg */
#define PRIV_CMD_HTTXCFG "httxcfg"
/** Private command: HT Cap Info */
#define PRIV_CMD_HTCAPINFO "htcapinfo"
/** Private command: Add BA para */
#define PRIV_CMD_ADDBAPARA "addbapara"
/** Private command: Aggragation priority table */
#define PRIV_CMD_AGGRPRIOTBL "aggrpriotbl"
/** Private command: Add BA reject cfg */
#define PRIV_CMD_ADDBAREJECT "addbareject"
/** Private command: Delete BA */
#define PRIV_CMD_DELBA "delba"
/** Private command: Reject Addba Req */
#define PRIV_CMD_REJECTADDBAREQ "rejectaddbareq"
/** Private command: 11AC Cfg */
#define PRIV_CMD_VHTCFG "vhtcfg"
/** Private command: 11AC Oper Mode Cfg */
#define PRIV_CMD_OPERMODECFG "opermodecfg"
#define PRIV_CMD_DATARATE "getdatarate"
#define PRIV_CMD_TXRATECFG "txratecfg"
#define PRIV_CMD_GETLOG "getlog"
#define PRIV_CMD_ESUPPMODE "esuppmode"
#define PRIV_CMD_PASSPHRASE "passphrase"
#define PRIV_CMD_DEAUTH "deauth"
#ifdef UAP_SUPPORT
#define PRIV_CMD_AP_DEAUTH "apdeauth"
#define PRIV_CMD_GET_STA_LIST "getstalist"
#define PRIV_CMD_BSS_CONFIG "bssconfig"
#endif
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
#define PRIV_CMD_BSSROLE "bssrole"
#endif
#endif
#ifdef STA_SUPPORT
#define PRIV_CMD_GETSCANTABLE "getscantable"
#define PRIV_CMD_GETCHANSTATS "getchanstats"
typedef struct _chan_stats {
	/** Number of records in the chan_stats */
	t_u32 num_in_chan_stats;
	/** channel statistics */
	ChanStatistics_t stats[];
} chan_stats;
#define PRIV_CMD_SETUSERSCAN "setuserscan"
#define PRIV_CMD_EXTCAPCFG "extcapcfg"
#define PRIV_CMD_CANCELSCAN "cancelscan"
#endif
#define PRIV_CMD_DEEPSLEEP "deepsleep"
#define PRIV_CMD_IPADDR "ipaddr"
#define PRIV_CMD_WPSSESSION "wpssession"
#define PRIV_CMD_OTPUSERDATA "otpuserdata"
#define PRIV_CMD_COUNTRYCODE "countrycode"
#define PRIV_CMD_TCPACKENH "tcpackenh"
#ifdef REASSOCIATION
#define PRIV_CMD_ASSOCESSID "assocessid"
#define PRIV_CMD_ASSOCBSSID "assocessid_bssid"
#endif
#define PRIV_CMD_WAKEUPREASON "wakeupreason"
#ifdef STA_SUPPORT
#define PRIV_CMD_LISTENINTERVAL "listeninterval"
#endif
#ifdef DEBUG_LEVEL1
#define PRIV_CMD_DRVDBG "drvdbg"
#endif
#define PRIV_CMD_HSCFG "hscfg"
#define PRIV_CMD_HSSETPARA "hssetpara"
#define PRIV_CMD_MGMT_FILTER "mgmtfilter"
#define PRIV_CMD_SCANCFG "scancfg"
#define PRIV_CMD_GETNLNUM "getnlnum"
#define PRIV_CMD_AGGRCTRL "aggrctrl"
#ifdef USB
#define PRIV_CMD_USBAGGRCTRL "usbaggrctrl"
#endif
#define PRIV_CMD_SET_BSS_MODE "setbssmode"
#ifdef STA_SUPPORT
#define PRIV_CMD_SET_AP "setap"
#define PRIV_CMD_SET_POWER "setpower"
#define PRIV_CMD_SET_ESSID "setessid"
#define PRIV_CMD_SET_AUTH "setauth"
#define PRIV_CMD_GET_AP "getap"
#define PRIV_CMD_GET_POWER "getpower"
#define PRIV_CMD_PSMODE "psmode"
#endif
#define PRIV_CMD_WARMRESET "warmreset"
#define PRIV_CMD_TXPOWERCFG "txpowercfg"
#define PRIV_CMD_PSCFG "pscfg"
#define PRIV_CMD_BCNTIMEOUTCFG "bcntimeoutcfg"
#define PRIV_CMD_SLEEPPD "sleeppd"
#define PRIV_CMD_TXCONTROL "txcontrol"
#define PRIV_CMD_REGRDWR "regrdwr"
#define PRIV_CMD_RDEEPROM "rdeeprom"
#define PRIV_CMD_MEMRDWR "memrdwr"
#ifdef SDIO
#define PRIV_CMD_SDCMD52RW "sdcmd52rw"
#endif
#define PRIV_CMD_HOTSPOTCFG "hotspotcfg"
#define PRIV_CMD_MGMT_FRAME_CTRL "mgmtframectrl"
#define PRIV_CMD_QCONFIG "qconfig"
#define PRIV_CMD_ADDTS "addts"
#define PRIV_CMD_DELTS "delts"
#define PRIV_CMD_QSTATUS "qstatus"
#define PRIV_CMD_TS_STATUS "ts_status"
#define PRIV_CMD_QOS_CFG "qoscfg"
#define PRIV_CMD_MAC_CTRL "macctrl"
#define PRIV_CMD_GETWAP "getwap"
#define PRIV_CMD_REGION_CODE "regioncode"
#define PRIV_CMD_CFPINFO "cfpinfo"
#define PRIV_CMD_FWMACADDR "fwmacaddr"
#define PRIV_CMD_OFFCHANNEL "offchannel"
#define PRIV_CMD_DSCP_MAP "dscpmap"
/** Private command: Verext */
#define PRIV_CMD_VEREXT "verext"
#ifdef CONFIG_USB_SUSPEND
#define PRIV_CMD_USB_SUSPEND "usbsuspend"
#define PRIV_CMD_USB_RESUME "usbresume"
#endif /* CONFIG_USB_SUSPEND */
#if defined(STA_SUPPORT) && defined(STA_WEXT)
#define PRIV_CMD_RADIO_CTRL "radioctrl"
#endif
#define PRIV_CMD_WMM_CFG "wmmcfg"
#define PRIV_CMD_MIN_BA_THRESH_CFG "min_ba_threshold"
#if defined(STA_SUPPORT)
#define PRIV_CMD_11D_CFG "11dcfg"
#define PRIV_CMD_11D_CLR_TBL "11dclrtbl"
#endif
#ifndef OPCHAN
#define PRIV_CMD_WWS_CFG "wwscfg"
#endif
#if defined(REASSOCIATION)
#define PRIV_CMD_REASSOCTRL "reassoctrl"
#endif
#define PRIV_CMD_TXBUF_CFG "txbufcfg"
#ifdef STA_SUPPORT
#define PRIV_CMD_AUTH_TYPE "authtype"
#endif
#define PRIV_CMD_POWER_CONS "powercons"
#define PRIV_CMD_HT_STREAM_CFG "htstreamcfg"
#define PRIV_CMD_MIMO_SWITCH "mimoswitch"
#define PRIV_CMD_THERMAL "thermal"
#define PRIV_CMD_BCN_INTERVAL "bcninterval"
#ifdef STA_SUPPORT
#define PRIV_CMD_GET_SIGNAL "getsignal"
#define PRIV_CMD_SIGNALEXT_CFG "signalextcfg"
#define PRIV_CMD_GET_SIGNAL_EXT_V2 "getsignalextv2"
#define PRIV_CMD_GET_SIGNAL_EXT "getsignalext"
#endif
#if defined(STA_SUPPORT)
#define PRIV_CMD_PMFCFG "pmfcfg"
#endif
#define PRIV_CMD_INACTIVITYTO "inactivityto"
#define PRIV_CMD_AMSDU_AGGR_CTRL "amsduaggrctrl"
#define PRIV_CMD_TX_BF_CAP "httxbfcap"
#ifdef SDIO
#define PRIV_CMD_SDIO_CLOCK "sdioclock"
#endif
#ifdef SDIO
#define PRIV_CMD_MPA_CTRL "mpactrl"
#endif
#define PRIV_CMD_SLEEP_PARAMS "sleepparams"
#define PRIV_CMD_DFS_TESTING "dfstesting"
#define PRIV_CMD_DFS53_CFG "dfs53cfg"
#define PRIV_CMD_CFP_CODE "cfpcode"
#define PRIV_CMD_CWMODE "cwmode"
#define PRIV_CMD_ANT_CFG "antcfg"
#define PRIV_CMD_SYSCLOCK "sysclock"
#define PRIV_CMD_GET_KEY "getkey"
#define PRIV_CMD_ASSOCIATE "associate"
#define PRIV_CMD_TX_BF_CFG "httxbfcfg"
#define PRIV_CMD_PORT_CTRL "port_ctrl"
#define PRIV_CMD_PB_BYPASS "pb_bypass"
#ifdef SDIO
#define PRIV_CMD_SD_CMD53_RW "sdcmd53rw"
#endif
#ifdef RX_PACKET_COALESCE
#define PRIV_CMD_RX_COAL_CFG "rxpktcoal_cfg"
#endif
#ifdef WIFI_DIRECT_SUPPORT
#if defined(UAP_CFG80211)
#define PRIV_CMD_CFG_NOA "cfg_noa"
#define PRIV_CMD_CFG_OPP_PS "cfg_opp_ps"
#endif
#endif
#define PRIV_CMD_DFS_REPEATER_CFG "dfs_repeater"
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#define PRIV_CMD_MIRACAST_CFG "miracastcfg"
#endif
#endif
#define PRIV_CMD_COEX_RX_WINSIZE "coex_rx_winsize"

#ifdef PCIE
#define PRIV_CMD_PCIE_REG_RW "pcieregrw"
#define PRIV_CMD_PCIE_BAR0_REG_RW "pciebar0regrw"
#endif

#define PRIV_CMD_GET_SENSOR_TEMP "get_sensor_temp"

#define PRIV_CMD_GET_CHNRGPWR "get_chnrgpwr"
#define PRIV_CMD_GET_TXPWR_LIMIT "get_txpwrlimit"
#define PRIV_CMD_GET_CFG_CHAN_LIST "getcfgchanlist"
#if defined(UAP_SUPPORT)
#define PRIV_CMD_EXTEND_CHAN_SWITCH "channel_switch"
#endif

#define PRIV_CMD_DYN_BW "dyn_bw"

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
#define PRIV_CMD_DFS_OFFLOAD "dfs_offload"
#endif

#define PRIV_CMD_AUTO_ARP "auto_arp"

#define PRIV_CMD_PER_PKT_CFG "per_pkt_cfg"

#define PRIV_CMD_DEAUTH_CTRL "ctrldeauth"

#define PRIV_CMD_TX_RX_HISTOGRAM "txrxhistogram"

/**Private command ID to set/get independent reset*/
#define PRIV_CMD_IND_RST_CFG "indrstcfg"

#define PRIV_CMD_ARB_CFG "arb"

/**Private command to configure static rx abort config */
#define PRIV_CMD_RX_ABORT_CFG "rx_abort_cfg"
/**Private command to configure dynamic rx abort config */
#define PRIV_CMD_RX_ABORT_CFG_EXT "rx_abort_cfg_ext"
#define TX_AMPDU_RTS_CTS 0
#define TX_AMPDU_CTS_2_SELF 1
#define TX_AMPDU_DISABLE_PROTECTION 2
#define TX_AMPDU_DYNAMIC_RTS_CTS 3
/**Private command to set tx ampdu protection mode */
#define PRIV_CMD_TX_AMPDU_PROT_MODE "tx_ampdu_prot_mode"
/**Private command to configure tx rate adapt config */
#define PRIV_CMD_RATE_ADAPT_CFG "rate_adapt_cfg"
#define CCK_DESENSE_MODE_DISABLED 0
#define CCK_DESENSE_MODE_DYNAMIC 1
#define CCK_DESENSE_MODE_DYN_ENH 2
/**Private command to configure cck desense config */
#define PRIV_CMD_CCK_DESENSE_CFG "cck_desense_cfg"
#define PRIV_CMD_DOT11MC_UNASSOC_FTM_CFG "dot11mc_unassoc_ftm_cfg"

/** Private command ID for Android default commands */
#define WOAL_ANDROID_DEF_CMD (SIOCDEVPRIVATE + 1)

/** Private command ID to pass mgmt frame */
#define WOAL_MGMT_FRAME_TX WOAL_MGMT_FRAME_TX_IOCTL

/** Private command ID to pass custom IE list */
#define WOAL_CUSTOM_IE_CFG (SIOCDEVPRIVATE + 13)

/** Private command ID for Android ICS priv CMDs */
#define WOAL_ANDROID_PRIV_CMD (SIOCDEVPRIVATE + 14)

/** Private command ID to get BSS type */
#define WOAL_GET_BSS_TYPE (SIOCDEVPRIVATE + 15)

/** Private command ID for robustcoex */
#define PRIV_CMD_ROBUSTCOEX "robustcoex"

#define PRIV_CMD_DMCS "dmcs"

#if defined(PCIE)
#define PRIV_CMD_SSU "ssu"
/** ssu_params_ctrl */
typedef struct _ssu_params_cfg {
	/* ssu mode */
	t_u8 ssu_mode;
	/* 0-3; # of FFT samples to skip*/
	t_u32 nskip;
	/* 0-3: # of FFT samples selected to dump */
	t_u32 nsel;
	/* 0-3: Down sample ADC input for buffering*/
	t_u32 adcdownsample;
	/* 0-1: Mask out ADC Data from spectral packet */
	t_u32 mask_adc_pkt;
	/* 0-1: Enable 16-Bit FFT output data precision in spectral packet */
	t_u32 out_16bits;
	/* 0-1: Enable power spectrum in dB for spectral packe */
	t_u32 spec_pwr_enable;
	/* 0-1: Enable spectral packet rate reduction in DB output format */
	t_u32 rate_deduction;
	/* 0-7: Number of spectral packets over which spectral data is to be
	 * averaged. */
	t_u32 n_pkt_avg;
} __attribute__((packed)) ssu_params_cfg;
#endif

#define PRIV_CMD_BOOTSLEEP "bootsleep"

/** Private command: 11AX Cfg */
#define PRIV_CMD_11AXCFG "11axcfg"
/** Private command: 11AX Cmd */
#define PRIV_CMD_11AXCMDCFG "11axcmd"
/** Private command: Range ext Cmd */
#define PRIV_CMD_RANGE_EXT "range_ext"
/** Private command: TWT Setup Cfg */
#define PRIV_CMD_TWT_SETUP "twt_setup"
/** Private command: TWT Teardown Cfg */
#define PRIV_CMD_TWT_TEARDOWN "twt_teardown"

#define PRIV_CMD_LPM "lpm"

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
} __attribute__((packed)) android_wifi_priv_cmd;
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
#define MW_ESSID_MAX_SIZE 32

/* Modes of operation */
#define MW_MODE_AUTO 0 /* Let the driver decides */
#define MW_MODE_ADHOC 1 /* Single cell network */
#define MW_MODE_INFRA 2 /* Multi cell network, roaming, ... */
#define MW_MODE_MASTER 3 /* Synchronisation master or Access Point */
#define MW_MODE_REPEAT 4 /* Wireless Repeater (forwarder) */
#define MW_MODE_SECOND 5 /* Secondary master/repeater (backup) */
#define MW_MODE_MONITOR 6 /* Passive monitor (listen only) */
#define MW_MODE_MESH 7 /* Mesh (IEEE 802.11s) network */

#define MW_POWER_TYPE 0xF000 /* Type of parameter */
#define MW_POWER_PERIOD 0x1000 /* Value is a period/duration of  */
#define MW_POWER_TIMEOUT 0x2000 /* Value is a timeout (to go asleep) */

#define MW_AUTH_INDEX 0x0FFF
#define MW_AUTH_FLAGS 0xF000
#define MW_AUTH_WPA_VERSION 0
#define MW_AUTH_CIPHER_PAIRWISE 1
#define MW_AUTH_CIPHER_GROUP 2
#define MW_AUTH_KEY_MGMT 3
#define MW_AUTH_TKIP_COUNTERMEASURES 4
#define MW_AUTH_DROP_UNENCRYPTED 5
#define MW_AUTH_80211_AUTH_ALG 6
#define MW_AUTH_WPA_ENABLED 7
#define MW_AUTH_RX_UNENCRYPTED_EAPOL 8
#define MW_AUTH_ROAMING_CONTROL 9
#define MW_AUTH_PRIVACY_INVOKED 10
#define MW_AUTH_CIPHER_GROUP_MGMT 11
#define MW_AUTH_MFP 12

#define MW_AUTH_CIPHER_NONE 0x00000001
#define MW_AUTH_CIPHER_WEP40 0x00000002
#define MW_AUTH_CIPHER_TKIP 0x00000004
#define MW_AUTH_CIPHER_CCMP 0x00000008
#define MW_AUTH_CIPHER_WEP104 0x00000010
#define MW_AUTH_CIPHER_AES_CMAC 0x00000020

#define MW_AUTH_ALG_OPEN_SYSTEM 0x00000001
#define MW_AUTH_ALG_SHARED_KEY 0x00000002
#define MW_AUTH_ALG_LEAP 0x00000004

/* Generic format for most parameters that fit in an int */
struct mw_param {
	t_s32 value; /* The value of the parameter itself */
	t_u8 fixed; /* Hardware should not use auto select */
	t_u8 disabled; /* Disable the feature */
	t_u16 flags; /* Various specifc flags (if any) */
};

/*
 *  For all data larger than 16 octets, we need to use a
 *  pointer to memory allocated in user space.
 */
struct mw_point {
	t_u8 *pointer; /* Pointer to the data  (in user space) */
	t_u16 length; /* number of fields or size in bytes */
	t_u16 flags; /* Optional params */
};

/*
 * This structure defines the payload of an ioctl, and is used
 * below.
 */
union mwreq_data {
	/* Config - generic */
	char name[IFNAMSIZ];

	struct mw_point essid; /* Extended network name */
	t_u32 mode; /* Operation mode */
	struct mw_param power; /* PM duration/timeout */
	struct sockaddr ap_addr; /* Access point address */
	struct mw_param param; /* Other small parameters */
	struct mw_point data; /* Other large parameters */
};

/* The structure to exchange data for ioctl */
struct mwreq {
	union {
		char ifrn_name[IFNAMSIZ]; /* if name, e.g. "eth0" */
	} ifr_ifrn;

	/* Data part */
	union mwreq_data u;
};

typedef struct woal_priv_ht_cap_info {
	t_u32 ht_cap_info_bg;
	t_u32 ht_cap_info_a;
} woal_ht_cap_info;

typedef struct woal_priv_addba {
	t_u32 time_out;
	t_u32 tx_win_size;
	t_u32 rx_win_size;
	t_u32 tx_amsdu;
	t_u32 rx_amsdu;
} woal_addba;

typedef struct _txrate_setting {
	t_u16 preamble : 2; /*BIT1-BIT0:
			     *  For legacy 11b: preamble type
			     *    00    = long
			     *    01    = short
			     *    10/11  = reserved
			     *  For legacy 11g: reserved
			     *  For 11n: Green field PPDU indicator
			     *    00 = HT-mix
			     *    01 = HT-GF
			     *    10/11 = reserved.
			     *  For 11ac: reserved.
			     *  For 11ax:
			     *    00 = HE-SU
			     *    01 = HE-EXT-SU
			     *    10 = HE-MU
			     *    11 = HE trigger based
			     */
	t_u16 bandwidth : 3; /* BIT2- BIT4
			      * For 11n and 11ac traffic: Bandwidth
			      *    0 = 20Mhz
			      *    1 = 40Mhz
			      *    2 = 80 Mhz
			      *    3 = 160 Mhz
			      *    4-7 = reserved
			      *  For legacy rate : BW>0 implies non-HT
			      * duplicates. For HE SU PPDU: 0 = 20Mhz 1 = 40Mhz
			      *    2 = 80 Mhz
			      *    3 = 160 Mhz
			      *    4-7 = reserved
			      *  For HE ER SU PPDU:
			      *    0 = 242-tone RU
			      *    1 = upper frequency 106 tone RU within the
			      * primary 20 Mhz. For HE MU PPDU: 0 = 20Mhz. 1 =
			      * 40Mhz. 2 = 80Mhz non-preamble puncturing mode 3
			      * = 160Mhz and 80+80 Mhz non-preamble. 4 = for
			      * preemble puncturing in 80 Mhz , where in the
			      * preamble only the secondary 20Mhz is punctured.
			      *    5 = for preemble puncturing in 80 Mhz ,
			      *        where in the preamble only one of the two
			      * 20Mhz subchannels in the secondary 40Mhz is
			      * punctured. 6 = for preemble puncturing in 160
			      * Mhz or 80 Mhz + 80 Mhz, where in the primary 80
			      * Mhz of the preamble only the secondary 20 Mhz is
			      * punctured. 7 = for preemble puncturing in 160
			      * Mhz or 80 Mhz + 80 Mhz, where in the primary 80
			      * Mhz of the preamble the primary 40 Mhz is
			      * present.
			      */
	t_u16 shortGI : 2; /*BIT5- BIT6
			    *  For legacy: not used
			    *  For 11n: 00 = normal, 01 =shortGI, 10/11 =
			    * reserved For 11ac: SGI map to VHT-SIG-A2[0]
			    *           VHT-SIG-A2[1] is set to 1 if short guard
			    * interval is used and NSYM mod 10 = 9, otherwise
			    * set to 0. For 11ax: 00 = 1xHELTF+GI0.8usec 01 =
			    * 2xHELTF+GI0.8usec 10 = 2xHELTF+GI1.6usec 11 =
			    * 4xHELTF+GI0.8 usec if both DCM and STBC are 1
			    *                4xHELTF+GI3.2 usec otherwise
			    */
	t_u16 stbc : 1; // BIT7, 0: no STBC; 1: STBC
	t_u16 dcm : 1; // BIT8, 0: no DCM; 1: DCM used.
	t_u16 adv_coding : 1; // BIT9, 0: BCC; 1: LDPC.
	t_u16 doppler : 2; /* BIT11-BIT10,
			      00: Doppler0
			      01: Doppler 1 with Mma =10
			      10: Doppler 1 with Mma =20
			   */
	t_u16 max_pktext : 2; /*BIT12-BIT13:
			       * Max packet extension
			       *  0 - 0 usec
			       *  1 - 8 usec
			       *  2 - 16 usec.
			       */
	t_u16 reserverd : 2; // BIT14-BIT15
} __ATTRIB_PACK__ txrate_setting;

/** data structure for cmd txratecfg */
typedef struct woal_priv_tx_rate_cfg {
	/* LG rate: 0, HT rate: 1, VHT rate: 2 */
	t_u32 rate_format;
	/** Rate/MCS index (0xFF: auto) */
	t_u32 rate_index;
	/** Data rate */
	t_u32 rate;
	/** NSS */
	t_u32 nss;
	/** Rate Setting */
	t_u16 rate_setting;
} woal_tx_rate_cfg;

typedef struct woal_priv_esuppmode_cfg {
	/* RSN mode */
	t_u16 rsn_mode;
	/* Pairwise cipher */
	t_u8 pairwise_cipher;
	/* Group cipher */
	t_u8 group_cipher;
} woal_esuppmode_cfg;

mlan_status woal_set_ap_wps_p2p_ie(moal_private *priv, t_u8 *ie, size_t len);
mlan_status woal_ioctl_aggr_prio_tbl(moal_private *priv, t_u32 action,
				     mlan_ds_11n_aggr_prio_tbl *aggr_prio_tbl);

int woal_android_priv_cmd(struct net_device *dev, struct ifreq *req);

#define PRIV_CMD_CLOUD_KEEP_ALIVE "cloud_keep_alive"
/** cloud keep alive parameters */
typedef struct _cloud_keep_alive {
	/** id */
	t_u8 mkeep_alive_id;
	/** enable/disable of this id */
	t_u8 enable;
	/** enable/disable reset*/
	t_u8 reset;
	/** Reserved */
	t_u8 reserved;
	/** Destination MAC address */
	t_u8 dst_mac[ETH_ALEN];
	/** Source MAC address */
	t_u8 src_mac[ETH_ALEN];
	/** packet send period */
	t_u32 sendInterval;
	/** packet retry interval */
	t_u32 retryInterval;
	/** packet retry count */
	t_u8 retryCount;
	/** packet length */
	t_u8 pkt_len;
	/** packet content */
	t_u8 pkt[255];
} __ATTRIB_PACK__ cloud_keep_alive;

#define TLV_TYPE_PER_PKT_CFG 0x0001
#define TX_PKT_CTRL MBIT(0)
#define RX_PKT_INFO MBIT(1)

#define FLAG_TX_HISTOGRAM 0x01
#define FLAG_RX_HISTOGRAM 0x02
#define DISABLE_TX_RX_HISTOGRAM 0x00
#define ENABLE_TX_RX_HISTOGRAM 0x01
#define GET_TX_RX_HISTOGRAM 0x02
#define PRIV_CMD_TX_RX_HISTOGRAM "txrxhistogram"
/** TX and RX histogram statistic parameters*/
typedef struct _tx_rx_histogram {
	/** Enable or disable get tx/rx histogram statistic */
	t_u8 enable;
	/** Choose to get TX, RX or both histogram statistic */
	t_u8 action;
} __ATTRIB_PACK__ tx_rx_histogram;

/* Enum for different CW mode type */
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
	/** dfs_state */
	t_u8 dfs_state;
} __ATTRIB_PACK__ wlan_ieee80211_chan;

/** wlan_ieee80211_chan_list*/
typedef struct {
	/** num of chan */
	t_u8 num_chan;
	/** chan_list */
	wlan_ieee80211_chan chan_list[];
} __ATTRIB_PACK__ wlan_ieee80211_chan_list;

#define PRIV_CMD_TP_STATE "tp_state"
#endif /* _WOAL_ETH_PRIV_H_ */
