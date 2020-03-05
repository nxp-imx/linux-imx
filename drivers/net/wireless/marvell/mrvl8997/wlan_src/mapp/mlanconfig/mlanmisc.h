/** @file  mlanmisc.h
  *
  * @brief This file contains command definitions for application
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
/************************************************************************
Change log:
     03/10/2009: initial version
************************************************************************/

#ifndef _MLANMISC_H_
#define _MLANMISC_H_

/** Maximum size of IEEE Information Elements */
#define IEEE_MAX_IE_SIZE  256

/** Maximum scan response buffer size */
#define SCAN_RESP_BUF_SIZE 2000

#ifdef FALSE
#undef FALSE
#endif

#ifdef TRUE
#undef TRUE
#endif

#ifndef MIN
/** Find minimum value */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
/** Find maximum value */
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif /* MAX */

/** Type enumeration of WMM AC_QUEUES */
typedef enum _mlan_wmm_ac_e {
	WMM_AC_BK,
	WMM_AC_BE,
	WMM_AC_VI,
	WMM_AC_VO
} __ATTRIB_PACK__ mlan_wmm_ac_e;

/** Maximum length of SSID */
#define MRVDRV_MAX_SSID_LENGTH          32

/** Enumeration for scan mode */
enum {
	MLAN_SCAN_MODE_UNCHANGED = 0,
	MLAN_SCAN_MODE_BSS,
	MLAN_SCAN_MODE_IBSS,
	MLAN_SCAN_MODE_ANY
};

/** Enumeration for scan type */
enum {
	MLAN_SCAN_TYPE_UNCHANGED = 0,
	MLAN_SCAN_TYPE_ACTIVE,
	MLAN_SCAN_TYPE_PASSIVE
};

/** Length of ethernet address */
#ifndef ETH_ALEN
#define ETH_ALEN            6
#endif
/** Maximum length of SSID list */
#define MRVDRV_MAX_SSID_LIST_LENGTH         10

/** Maximum number of channels that can be sent in a setuserscan ioctl */
#define WLAN_IOCTL_USER_SCAN_CHAN_MAX  50

/** IEEE Type definitions  */
typedef enum _IEEEtypes_ElementId_e {
	SSID = 0,
	SUPPORTED_RATES = 1,
	FH_PARAM_SET = 2,
	DS_PARAM_SET = 3,
	CF_PARAM_SET = 4,

	IBSS_PARAM_SET = 6,

	COUNTRY_INFO = 7,

	POWER_CONSTRAINT = 32,
	POWER_CAPABILITY = 33,
	TPC_REQUEST = 34,
	TPC_REPORT = 35,
	SUPPORTED_CHANNELS = 36,
	CHANNEL_SWITCH_ANN = 37,
	QUIET = 40,
	IBSS_DFS = 41,
	HT_CAPABILITY = 45,
	HT_OPERATION = 61,
	BSSCO_2040 = 72,
	OVERLAPBSSSCANPARAM = 74,
	EXT_CAPABILITY = 127,

	VHT_CAPABILITY = 191,
	VHT_OPERATION = 192,
	EXT_BSS_LOAD = 193,
	BW_CHANNEL_SWITCH = 194,
	VHT_TX_POWER_ENV = 195,
	EXT_POWER_CONSTR = 196,
	AID_INFO = 197,
	QUIET_CHAN = 198,

	ERP_INFO = 42,
	EXTENDED_SUPPORTED_RATES = 50,

	VENDOR_SPECIFIC_221 = 221,
	WMM_IE = VENDOR_SPECIFIC_221,

	WPS_IE = VENDOR_SPECIFIC_221,

	WPA_IE = VENDOR_SPECIFIC_221,
	RSN_IE = 48,
} __ATTRIB_PACK__ IEEEtypes_ElementId_e;

/** Capability Bit Map*/
#ifdef BIG_ENDIAN_SUPPORT
typedef struct _IEEEtypes_CapInfo_t {
	t_u8 rsrvd1:2;
	t_u8 dsss_ofdm:1;
	t_u8 rsvrd2:2;
	t_u8 short_slot_time:1;
	t_u8 rsrvd3:1;
	t_u8 spectrum_mgmt:1;
	t_u8 chan_agility:1;
	t_u8 pbcc:1;
	t_u8 short_preamble:1;
	t_u8 privacy:1;
	t_u8 cf_poll_rqst:1;
	t_u8 cf_pollable:1;
	t_u8 ibss:1;
	t_u8 ess:1;
} __ATTRIB_PACK__ IEEEtypes_CapInfo_t, *pIEEEtypes_CapInfo_t;
#else
typedef struct _IEEEtypes_CapInfo_t {
    /** Capability Bit Map : ESS */
	t_u8 ess:1;
    /** Capability Bit Map : IBSS */
	t_u8 ibss:1;
    /** Capability Bit Map : CF pollable */
	t_u8 cf_pollable:1;
    /** Capability Bit Map : CF poll request */
	t_u8 cf_poll_rqst:1;
    /** Capability Bit Map : privacy */
	t_u8 privacy:1;
    /** Capability Bit Map : Short preamble */
	t_u8 short_preamble:1;
    /** Capability Bit Map : PBCC */
	t_u8 pbcc:1;
    /** Capability Bit Map : Channel agility */
	t_u8 chan_agility:1;
    /** Capability Bit Map : Spectrum management */
	t_u8 spectrum_mgmt:1;
    /** Capability Bit Map : Reserved */
	t_u8 rsrvd3:1;
    /** Capability Bit Map : Short slot time */
	t_u8 short_slot_time:1;
    /** Capability Bit Map : APSD */
	t_u8 apsd:1;
    /** Capability Bit Map : Reserved */
	t_u8 rsvrd2:1;
    /** Capability Bit Map : DSS OFDM */
	t_u8 dsss_ofdm:1;
    /** Capability Bit Map : Reserved */
	t_u8 rsrvd1:2;
} __ATTRIB_PACK__ IEEEtypes_CapInfo_t, *pIEEEtypes_CapInfo_t;
#endif /* BIG_ENDIAN_SUPPORT */

/** IEEE IE header */
typedef struct _IEEEtypes_Header_t {
    /** Element ID */
	t_u8 element_id;
    /** Length */
	t_u8 len;
} __ATTRIB_PACK__ IEEEtypes_Header_t, *pIEEEtypes_Header_t;

/** IEEE IE header */
#define IEEE_HEADER_LEN   sizeof(IEEEtypes_Header_t)

/** Vendor specific IE header */
typedef struct _IEEEtypes_VendorHeader_t {
    /** Element ID */
	t_u8 element_id;
    /** Length */
	t_u8 len;
    /** OUI */
	t_u8 oui[3];
    /** OUI type */
	t_u8 oui_type;
    /** OUI subtype */
	t_u8 oui_subtype;
    /** Version */
	t_u8 version;
} __ATTRIB_PACK__ IEEEtypes_VendorHeader_t, *pIEEEtypes_VendorHeader_t;

/** Vendor specific IE */
typedef struct _IEEEtypes_VendorSpecific_t {
    /** Vendor specific IE header */
	IEEEtypes_VendorHeader_t vend_hdr;
    /** IE Max - size of previous fields */
	t_u8 data[IEEE_MAX_IE_SIZE - sizeof(IEEEtypes_VendorHeader_t)];
} __ATTRIB_PACK__ IEEEtypes_VendorSpecific_t, *pIEEEtypes_VendorSpecific_t;

/** IEEE IE */
typedef struct _IEEEtypes_Generic_t {
    /** Generic IE header */
	IEEEtypes_Header_t ieee_hdr;
    /** IE Max - size of previous fields */
	t_u8 data[IEEE_MAX_IE_SIZE - sizeof(IEEEtypes_Header_t)];
} __ATTRIB_PACK__ IEEEtypes_Generic_t, *pIEEEtypes_Generic_t;

/** Size of a TSPEC.  Used to allocate necessary buffer space in commands */
#define WMM_TSPEC_SIZE              63

/** Maximum number of AC QOS queues available in the driver/firmware */
#define MAX_AC_QUEUES               4

/** Maximum number of User Priorities */
#define MAX_USER_PRIORITIES         8

/** Extra IE bytes allocated in messages for appended IEs after a TSPEC */
#define WMM_ADDTS_EXTRA_IE_BYTES    256

/**
 *  @brief Enumeration for the command result from an ADDTS or DELTS command
 */
typedef enum {
	TSPEC_RESULT_SUCCESS = 0,
	TSPEC_RESULT_EXEC_FAILURE = 1,
	TSPEC_RESULT_TIMEOUT = 2,
	TSPEC_RESULT_DATA_INVALID = 3,
} __ATTRIB_PACK__ mlan_wmm_tspec_result_e;

/**
 *  @brief Enumeration for the action field in the Queue configure command
 */
typedef enum {
	WMM_QUEUE_CONFIG_ACTION_GET = 0,
	WMM_QUEUE_CONFIG_ACTION_SET = 1,
	WMM_QUEUE_CONFIG_ACTION_DEFAULT = 2,

	WMM_QUEUE_CONFIG_ACTION_MAX
} __ATTRIB_PACK__ mlan_wmm_queue_config_action_e;

/**
 *   @brief Enumeration for the action field in the queue stats command
 */
typedef enum {
	WMM_STATS_ACTION_START = 0,
	WMM_STATS_ACTION_STOP = 1,
	WMM_STATS_ACTION_GET_CLR = 2,
	WMM_STATS_ACTION_SET_CFG = 3,	/* Not currently used */
	WMM_STATS_ACTION_GET_CFG = 4,	/* Not currently used */

	WMM_STATS_ACTION_MAX
} __ATTRIB_PACK__ mlan_wmm_stats_action_e;

/** Data structure of WMM Aci/Aifsn */
typedef struct _IEEEtypes_WmmAciAifsn_t {
#ifdef BIG_ENDIAN_SUPPORT
    /** Reserved */
	t_u8 reserved:1;
    /** Aci */
	t_u8 aci:2;
    /** Acm */
	t_u8 acm:1;
    /** Aifsn */
	t_u8 aifsn:4;
#else
    /** Aifsn */
	t_u8 aifsn:4;
    /** Acm */
	t_u8 acm:1;
    /** Aci */
	t_u8 aci:2;
    /** Reserved */
	t_u8 reserved:1;
#endif
} __ATTRIB_PACK__ IEEEtypes_WmmAciAifsn_t, *pIEEEtypes_WmmAciAifsn_t;

/** Data structure of WMM ECW */
typedef struct _IEEEtypes_WmmEcw_t {
#ifdef BIG_ENDIAN_SUPPORT
    /** Maximum Ecw */
	t_u8 ecw_max:4;
    /** Minimum Ecw */
	t_u8 ecw_min:4;
#else
    /** Minimum Ecw */
	t_u8 ecw_min:4;
    /** Maximum Ecw */
	t_u8 ecw_max:4;
#endif
} __ATTRIB_PACK__ IEEEtypes_WmmEcw_t, *pIEEEtypes_WmmEcw_t;

/** Data structure of WMM AC parameters  */
typedef struct _IEEEtypes_WmmAcParameters_t {
	IEEEtypes_WmmAciAifsn_t aci_aifsn;  /**< AciAifSn */
	IEEEtypes_WmmEcw_t ecw;		    /**< Ecw */
	t_u16 tx_op_limit;		    /**< Tx op limit */
} __ATTRIB_PACK__ IEEEtypes_WmmAcParameters_t, *pIEEEtypes_WmmAcParameters_t;

/** Data structure of WMM Info IE  */
typedef struct _IEEEtypes_WmmInfo_t {

    /**
     * WMM Info IE - Vendor Specific Header:
     *   element_id  [221/0xdd]
     *   Len         [7]
     *   Oui         [00:50:f2]
     *   OuiType     [2]
     *   OuiSubType  [0]
     *   Version     [1]
     */
	IEEEtypes_VendorHeader_t vend_hdr;

    /** QoS information */
	IEEEtypes_WmmQosInfo_t qos_info;

} __ATTRIB_PACK__ IEEEtypes_WmmInfo_t, *pIEEEtypes_WmmInfo_t;

/** Data structure of WMM parameter IE  */
typedef struct _IEEEtypes_WmmParameter_t {
    /**
     * WMM Parameter IE - Vendor Specific Header:
     *   element_id  [221/0xdd]
     *   Len         [24]
     *   Oui         [00:50:f2]
     *   OuiType     [2]
     *   OuiSubType  [1]
     *   Version     [1]
     */
	IEEEtypes_VendorHeader_t vend_hdr;

    /** QoS information */
	IEEEtypes_WmmQosInfo_t qos_info;
    /** Reserved */
	t_u8 reserved;

    /** AC Parameters Record WMM_AC_BE, WMM_AC_BK, WMM_AC_VI, WMM_AC_VO */
	IEEEtypes_WmmAcParameters_t ac_params[MAX_AC_QUEUES];

} __ATTRIB_PACK__ IEEEtypes_WmmParameter_t, *pIEEEtypes_WmmParameter_t;

/**
 *  @brief IOCTL structure to send an ADDTS request and retrieve the response.
 *
 *  IOCTL structure from the application layer relayed to firmware to
 *    instigate an ADDTS management frame with an appropriate TSPEC IE as well
 *    as any additional IEs appended in the ADDTS Action frame.
 *
 *  @sa wlan_wmm_addts_req_ioctl
 */
typedef struct {
	mlan_wmm_tspec_result_e commandResult;
					   /**< Firmware execution result */

	t_u32 timeout_ms;		/**< Timeout value in milliseconds */
	t_u8 ieeeStatusCode;		/**< IEEE status code */

	t_u32 ieDataLen;
	t_u8 ieData[WMM_TSPEC_SIZE
				 /**< TSPEC to send in the ADDTS */
		    + WMM_ADDTS_EXTRA_IE_BYTES];
					     /**< ADDTS extra IE buf */
} wlan_ioctl_wmm_addts_req_t;

/**
 *  @brief IOCTL structure to send a DELTS request.
 *
 *  IOCTL structure from the application layer relayed to firmware to
 *    instigate an DELTS management frame with an appropriate TSPEC IE.
 *
 *  @sa wlan_wmm_delts_req_ioctl
 */
typedef struct {
	mlan_wmm_tspec_result_e commandResult;
					    /**< Firmware execution result */
	t_u8 ieeeReasonCode;	  /**< IEEE reason code sent, unused for WMM */

	t_u32 ieDataLen;
	t_u8 ieData[WMM_TSPEC_SIZE];
				  /**< TSPEC to send in the DELTS */
} wlan_ioctl_wmm_delts_req_t;

/**
 *  @brief IOCTL structure to configure a specific AC Queue's parameters
 *
 *  IOCTL structure from the application layer relayed to firmware to
 *    get, set, or default the WMM AC queue parameters.
 *
 *  - msduLifetimeExpiry is ignored if set to 0 on a set command
 *
 *  @sa wlan_wmm_queue_config_ioctl
 */
typedef struct {
	mlan_wmm_queue_config_action_e action;
					   /**< Set, Get, or Default */
	mlan_wmm_ac_e accessCategory;	   /**< WMM_AC_BK(0) to WMM_AC_VO(3) */
	t_u16 msduLifetimeExpiry;	   /**< lifetime expiry in TUs */
	t_u8 supportedRates[10];	   /**< Not supported yet */
} wlan_ioctl_wmm_queue_config_t;

/** Number of bins in the histogram for the HostCmd_DS_WMM_QUEUE_STATS */
#define WMM_STATS_PKTS_HIST_BINS  7

/**
 *  @brief IOCTL structure to start, stop, and get statistics for a WMM AC
 *
 *  IOCTL structure from the application layer relayed to firmware to
 *    start or stop statistical collection for a given AC.  Also used to
 *    retrieve and clear the collected stats on a given AC.
 *
 *  @sa wlan_wmm_queue_stats_ioctl
 */
typedef struct {
	mlan_wmm_stats_action_e action;
				     /**< Start, Stop, or Get  */
	t_u8 userPriority;
			 /**< User Priority (0 to 7) */
	t_u16 pktCount;	 /**< Number of successful packets transmitted */
	t_u16 pktLoss;	 /**< Packets lost; not included in pktCount   */
	t_u32 avgQueueDelay;
			 /**< Average Queue delay in microseconds */
	t_u32 avgTxDelay;/**< Average Transmission delay in microseconds */
	t_u16 usedTime;	 /**< Calculated used time - units of 32 microsec */
	t_u16 policedTime;
			 /**< Calculated policed time - units of 32 microsec */

    /** @brief Queue Delay Histogram; number of packets per queue delay range
     *
     *  [0] -  0ms <= delay < 5ms
     *  [1] -  5ms <= delay < 10ms
     *  [2] - 10ms <= delay < 20ms
     *  [3] - 20ms <= delay < 30ms
     *  [4] - 30ms <= delay < 40ms
     *  [5] - 40ms <= delay < 50ms
     *  [6] - 50ms <= delay < msduLifetime (TUs)
     */
	t_u16 delayHistogram[WMM_STATS_PKTS_HIST_BINS];
} wlan_ioctl_wmm_queue_stats_t;

/**
 *  @brief IOCTL and command sub structure for a Traffic stream status.
 */
typedef struct {
	t_u8 tid;	    /**< TSID: Range: 0->7 */
	t_u8 valid;	    /**< TSID specified is valid  */
	t_u8 accessCategory;/**< AC TSID is active on */
	t_u8 userPriority;  /**< UP specified for the TSID */

	t_u8 psb;	    /**< Power save mode for TSID: 0 (legacy), 1 (UAPSD) */
	t_u8 flowDir;	    /**< Upstream (0), Downlink(1), Bidirectional(3) */
	t_u16 mediumTime;   /**< Medium time granted for the TSID */
} __ATTRIB_PACK__ HostCmd_DS_WMM_TS_STATUS,
	wlan_ioctl_wmm_ts_status_t, wlan_cmd_wmm_ts_status_t;

/**
 *  @brief IOCTL sub structure for a specific WMM AC Status
 */
typedef struct {
    /** WMM Acm */
	t_u8 wmmAcm;
    /** Flow required flag */
	t_u8 flowRequired;
    /** Flow created flag */
	t_u8 flowCreated;
    /** Disabled flag */
	t_u8 disabled;
    /** delivery enabled */
	t_u8 deliveryEnabled;
    /** trigger enabled */
	t_u8 triggerEnabled;
} wlan_ioctl_wmm_queue_status_ac_t;

/**
 *  @brief IOCTL structure to retrieve the WMM AC Queue status
 *
 *  IOCTL structure from the application layer to retrieve:
 *     - ACM bit setting for the AC
 *     - Firmware status (flow required, flow created, flow disabled)
 *
 *  @sa wlan_wmm_queue_status_ioctl
 */
typedef struct {
    /** WMM AC queue status */
	wlan_ioctl_wmm_queue_status_ac_t acStatus[MAX_AC_QUEUES];
} wlan_ioctl_wmm_queue_status_t;

typedef struct _wlan_get_scan_table_fixed {
    /** BSSID of this network */
	t_u8 bssid[MLAN_MAC_ADDR_LENGTH];
    /** Channel this beacon/probe response was detected */
	t_u8 channel;
    /** RSSI for the received packet */
	t_u8 rssi;
    /** TSF value from the firmware at packet reception */
	t_u64 network_tsf;
} wlan_get_scan_table_fixed;

/**
 *  Structure passed in the wlan_ioctl_get_scan_table_info for each
 *    BSS returned in the WLAN_GET_SCAN_RESP IOCTL
 */
typedef struct _wlan_ioctl_get_scan_table_entry {
    /**
     *  Fixed field length included in the response.
     *
     *  Length value is included so future fixed fields can be added to the
     *   response without breaking backwards compatibility.  Use the length
     *   to find the offset for the bssInfoLength field, not a sizeof() calc.
     */
	t_u32 fixed_field_length;

    /**
     *  Length of the BSS Information (probe resp or beacon) that
     *    follows after the fixed_field_length
     */
	t_u32 bss_info_length;

    /**
     *  Always present, fixed length data fields for the BSS
     */
	wlan_get_scan_table_fixed fixed_fields;

	/*
	 *  Probe response or beacon scanned for the BSS.
	 *
	 *  Field layout:
	 *   - TSF              8 octets
	 *   - Beacon Interval  2 octets
	 *   - Capability Info  2 octets
	 *
	 *   - IEEE Infomation Elements; variable number & length per 802.11 spec
	 */
	/* t_u8  bss_info_buffer[1]; */
} wlan_ioctl_get_scan_table_entry;

/**
 *  Structure to store BSS info (probe resp or beacon) & IEEE IE info for each
 *  BSS returned in WLAN_GET_SCAN_RESP IOCTL
 */
typedef struct _wlan_ioctl_get_bss_info {
	/**
	 *  Length of the BSS Information (probe resp or beacon) that
	 *    follows after the fixed_field
	 */
	t_u32 bss_info_length;

	/**
	 *  Probe response or beacon scanned for the BSS.
	 *
	 *  Field layout:
	 */
	/** TSF              8 octets */
	t_u8 tsf[8];
	/** Beacon Interval  2 octets */
	t_u16 beacon_interval;
	/** Capability Info  2 octets */
	IEEEtypes_CapInfo_t cap_info;

	/**
	 *  IEEE Infomation Elements; variable number & length per 802.11 spec
	 */
	/** SSID */
	char ssid[MRVDRV_MAX_SSID_LENGTH + 1];
	/** SSID Length */
	t_u32 ssid_len;
	/** WMM Capability */
	char wmm_cap;
	/** WPS Capability */
	char wps_cap;
	/** Privacy Capability - WEP/WPA/RSN */
	char priv_cap;
	/** HT (11N) Capability */
	char ht_cap;
	/** VHT (11AC) Capability */
	char vht_cap[2];
	/* 802.11k Capability */
	char dot11k_cap;
	/** 802.11r Capability */
	char dot11r_cap;
} wlan_ioctl_get_bss_info;

/**
 *  Structure to save of scan table info for each BSS returned
 * in WLAN_GET_SCAN_RESP IOCTL
 */
struct wlan_ioctl_get_scan_list {
	/** fixed info */
	wlan_ioctl_get_scan_table_entry fixed_buf;
	/** variable info - BSS info (probe resp or beacon) & IEEE IE info */
	wlan_ioctl_get_bss_info bss_info_buf;
	/** pointer to next node in list */
	struct wlan_ioctl_get_scan_list *next;
};

/**
 *  Sructure to retrieve the scan table
 */
typedef struct {
    /**
     *  - Zero based scan entry to start retrieval in command request
     *  - Number of scans entries returned in command response
     */
	t_u32 scan_number;
    /**
     * Buffer marker for multiple wlan_ioctl_get_scan_table_entry structures.
     *   Each struct is padded to the nearest 32 bit boundary.
     */
	t_u8 scan_table_entry_buf[1];

} wlan_ioctl_get_scan_table_info;

typedef struct {
	t_u8 chan_number;
		       /**< Channel Number to scan */
	t_u8 radio_type;
		       /**< Radio type: 'B/G' Band = 0, 'A' Band = 1 */
	t_u8 scan_type;/**< Scan type: Active = 1, Passive = 2 */
	t_u8 reserved;/**< Reserved */
	t_u32 scan_time;
		       /**< Scan duration in milliseconds; if 0 default used */
} __ATTRIB_PACK__ wlan_ioctl_user_scan_chan;

typedef struct {
	char ssid[MRVDRV_MAX_SSID_LENGTH + 1];
					    /**< SSID */
	t_u8 max_len;			       /**< Maximum length of SSID */
} __ATTRIB_PACK__ wlan_ioctl_user_scan_ssid;

typedef struct {

    /** Flag set to keep the previous scan table intact */
	t_u8 keep_previous_scan;	/* Do not erase the existing scan results */

    /** BSS mode to be sent in the firmware command */
	t_u8 bss_mode;

    /** Configure the number of probe requests for active chan scans */
	t_u8 num_probes;

    /** Reserved */
	t_u8 reserved;

    /** BSSID filter sent in the firmware command to limit the results */
	t_u8 specific_bssid[ETH_ALEN];

    /** SSID filter list used in the to limit the scan results */
	wlan_ioctl_user_scan_ssid ssid_list[MRVDRV_MAX_SSID_LIST_LENGTH];

    /** Variable number (fixed maximum) of channels to scan up */
	wlan_ioctl_user_scan_chan chan_list[WLAN_IOCTL_USER_SCAN_CHAN_MAX];

} __ATTRIB_PACK__ wlan_ioctl_user_scan_cfg;

int process_setuserscan(int argc, char *argv[]);
int process_getscantable(int argc, char *argv[]);
int process_getscantable_idx(wlan_ioctl_get_scan_table_info *prsp_info_req);

#endif /* _MLANMISC_H_ */
