/** @file  mlan2040misc.h
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
     06/24/2009: initial version
************************************************************************/
#ifndef _COEX_MISC_H_
#define _COEX_MISC_H_

/** MLAN MAC Address Length */
#define MLAN_MAC_ADDR_LENGTH     (6)
/** Size of command buffer */
#define MRVDRV_SIZE_OF_CMD_BUFFER       (2 * 1024)

/** Command RET code, MSB is set to 1 */
#define HostCmd_RET_BIT                 0x8000
/** General purpose action : Get */
#define HostCmd_ACT_GEN_GET             0x0000
/** General purpose action : Set */
#define HostCmd_ACT_GEN_SET             0x0001
/** TLV Id for 2040Coex IE */
#define TLV_ID_2040COEX                    0x48
/** TLV Id for 2040BSS intolarent channel report IE */
#define TLV_ID_2040BSS_INTOL_CHAN_REPORT   0x49
/** Host-command for 2040coex command */
#define HostCmd_CMD_11N_2040COEX           0x00e9
/** Maximum scan response buffer size */
#define SCAN_RESP_BUF_SIZE 2000

/** Maximum length of SSID */
#define MRVDRV_MAX_SSID_LENGTH          32

/** Length of ethernet address */
#ifndef ETH_ALEN
#define ETH_ALEN            6
#endif
/** Maximum length of SSID list */
#define MRVDRV_MAX_SSID_LIST_LENGTH         10
/** Default scan interval in second*/
#define DEFAULT_SCAN_INTERVAL 300

/** BIT value */
#define MBIT(x)    (((t_u32)1) << (x))

/** Check intolerent bit set */
#define IS_INTOL_BIT_SET(cap_info) (cap_info & MBIT(14))

/** Check OBSS non-HT STAs present bit set */
#define IS_NON_HT_STA_PRESENT(ht_info) (ht_info.field3 & MBIT(4))

/** IEEE Type definitions  */
typedef enum _IEEEtypes_ElementId_e {
	SSID = 0,
	SUPPORTED_RATES = 1,
	FH_PARAM_SET = 2,
	DS_PARAM_SET = 3,
	CF_PARAM_SET = 4,
	IBSS_PARAM_SET = 6,
	HT_CAPABILITY = 45,
	HT_OPERATION = 61,
	BSSCO_2040 = 72,
	OVERLAPBSSSCANPARAM = 74,
	EXT_CAPABILITY = 127,
	ERP_INFO = 42,
	EXTENDED_SUPPORTED_RATES = 50,
	VENDOR_SPECIFIC_221 = 221,
	WMM_IE = VENDOR_SPECIFIC_221,
	RSN_IE = 48,
} __attribute__ ((packed))
     IEEEtypes_ElementId_e;

/** HT Capabilities Data */
     typedef struct _HTCap_t {
    /** HT Capabilities Info field */
	     t_u16 ht_cap_info;
    /** A-MPDU Parameters field */
	     t_u8 ampdu_param;
    /** Supported MCS Set field */
	     t_u8 supported_mcs_set[16];
    /** HT Extended Capabilities field */
	     t_u16 ht_ext_cap;
    /** Transmit Beamforming Capabilities field */
	     t_u32 tx_bf_cap;
    /** Antenna Selection Capability field */
	     t_u8 asel;
    /** Reserved set to 0 */
	     t_u16 reserved;
     } __attribute__ ((packed))
     HTCap_t, *pHTCap_t;

/** HT Information Data */
     typedef struct _HTInfo_t {
    /** Primary channel */
	     t_u8 pri_chan;
    /** Field 2 */
	     t_u8 field2;
    /** Field 3 */
	     t_u16 field3;
    /** Field 4 */
	     t_u16 field4;
    /** Bitmap indicating MCSs supported by all HT STAs in the BSS */
	     t_u8 basic_mcs_set[16];
    /** Reserved set to 0 */
	     t_u16 reserved;
     } __attribute__ ((packed))
     HTInfo_t, *pHTInfo_t;

/** 20/40 BSS Coexistence Data */
     typedef struct _BSSCo2040_t {
    /** 20/40 BSS Coexistence value */
	     t_u8 bss_co_2040_value;
    /** Reserve field */
	     t_u8 reserved[3];
     } __attribute__ ((packed))
     BSSCo2040_t, *pBSSCo2040_t;

/** Extended Capabilities Data */
     typedef struct _ExtCap_t {
    /** Extended Capabilities value */
	     t_u8 ext_cap_value;
    /** Reserved field */
	     t_u8 reserved[3];
     } __attribute__ ((packed))
     ExtCap_t, *pExtCap_t;

/** Overlapping BSS Scan Parameters Data */
     typedef struct _OverlapBSSScanParam_t {
    /** OBSS Scan Passive Dwell */
	     t_u16 obss_scan_passive_dwell;
    /** OBSS Scan Active Dwell */
	     t_u16 obss_scan_active_dwell;
    /** BSS Channel Width Trigger Scan Interval */
	     t_u16 bss_chan_width_trigger_scan_int;
    /** OBSS Scan Passive Total Per Channel */
	     t_u16 obss_scan_passive_total;
    /** OBSS Scan Active Total Per Channel */
	     t_u16 obss_scan_active_total;
    /** BSS Width Channel Transition Delay Factor */
	     t_u16 bss_width_chan_trans_delay;
    /** OBSS Scan Activity Threshold */
	     t_u16 obss_scan_active_threshold;
     } __attribute__ ((packed))
     OBSSScanParam_t, *pOBSSScanParam_t;

/** IEEEtypes_CapInfo_t structure*/
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
     } __attribute__ ((packed))
     IEEEtypes_CapInfo_t, *pIEEEtypes_CapInfo_t;

     typedef struct {
	     t_u8 chan_number;
		       /**< Channel Number to scan */
	     t_u8 radio_type;
		       /**< Radio type: 'B/G' Band = 0, 'A' Band = 1 */
	     t_u8 scan_type;
		       /**< Scan type: Active = 1, Passive = 2 */
	     t_u8 reserved;
		      /**< Reserved */
	     t_u32 scan_time;
		       /**< Scan duration in milliseconds; if 0 default used */
     } __attribute__ ((packed))
     wlan_ioctl_user_scan_chan;

     typedef struct {
	     char ssid[MRVDRV_MAX_SSID_LENGTH + 1];
					    /**< SSID */
	     t_u8 max_len;		       /**< Maximum length of SSID */
     } __attribute__ ((packed))
     wlan_ioctl_user_scan_ssid;

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

     } __attribute__ ((packed))
     wlan_ioctl_user_scan_cfg;

/** IEEE IE header */
     typedef struct _IEEEtypes_Header_t {
    /** Element ID */
	     t_u8 element_id;
    /** Length */
	     t_u8 len;
     } __attribute__ ((packed))
     IEEEtypes_Header_t, *pIEEEtypes_Header_t;

/** HT Capabilities IE */
     typedef struct _IEEEtypes_HTCap_t {
    /** Generic IE header */
	     IEEEtypes_Header_t ieee_hdr;
    /** HTCap struct */
	     HTCap_t ht_cap;
     } __attribute__ ((packed))
     IEEEtypes_HTCap_t, *pIEEEtypes_HTCap_t;

/** HT Information IE */
     typedef struct _IEEEtypes_HTInfo_t {
    /** Generic IE header */
	     IEEEtypes_Header_t ieee_hdr;
    /** HTInfo struct */
	     HTInfo_t ht_info;
     } __attribute__ ((packed))
     IEEEtypes_HTInfo_t, *pIEEEtypes_HTInfo_t;

/** 20/40 BSS Coexistence IE */
     typedef struct _IEEEtypes_2040BSSCo_t {
    /** Generic IE header */
	     IEEEtypes_Header_t ieee_hdr;
    /** BSSCo2040_t struct */
	     BSSCo2040_t bss_co_2040;
     } __attribute__ ((packed))
     IEEEtypes_2040BSSCo_t, *pIEEEtypes_2040BSSCo_t;

/** Extended Capabilities IE */
     typedef struct _IEEEtypes_ExtCap_t {
    /** Generic IE header */
	     IEEEtypes_Header_t ieee_hdr;
    /** ExtCap_t struct */
	     ExtCap_t ext_cap;
     } __attribute__ ((packed))
     IEEEtypes_ExtCap_t, *pIEEEtypes_ExtCap_t;

/** Overlapping BSS Scan Parameters IE */
     typedef struct _IEEEtypes_OverlapBSSScanParam_t {
    /** Generic IE header */
	     IEEEtypes_Header_t ieee_hdr;
    /** OBSSScanParam_t struct */
	     OBSSScanParam_t obss_scan_param;
     } __attribute__ ((packed))
     IEEEtypes_OverlapBSSScanParam_t, *pIEEEtypes_OverlapBSSScanParam_t;

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
     *    follows starting at bssInfoBuffer
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
	     /* t_u8 bss_info_buffer[1]; */
     } wlan_ioctl_get_scan_table_entry;

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

/* Define general hostcmd data structure */
/** HostCmd_DS_GEN */
     typedef struct _HostCmd_DS_GEN {
    /** Command */
	     t_u16 command;
    /** Size */
	     t_u16 size;
    /** Sequence number */
	     t_u16 seq_num;
    /** Result */
	     t_u16 result;
     } __attribute__ ((packed))
     HostCmd_DS_GEN;

/** Size of HostCmd_DS_GEN */
#define S_DS_GEN    sizeof(HostCmd_DS_GEN)

/** TLV related data structures*/
/** MrvlIEtypesHeader_t */
     typedef struct _MrvlIEtypesHeader {
    /** Header type */
	     t_u16 type;
    /** Header length */
	     t_u16 len;
     } __attribute__ ((packed))
     MrvlIEtypesHeader_t;

/** _MrvlIETypes_2040BssIntolerantChannelReport_t */
     typedef struct _MrvlIETypes_2040BssIntolerantChannelReport_t {
    /** Header */
	     IEEEtypes_Header_t header;
    /** regulatory class */
	     t_u8 reg_class;
    /** channel numbers for legacy AP */
	     t_u8 chan_num[1];
     } __attribute__ ((packed))
     MrvlIETypes_2040BssIntolerantChannelReport_t;

/** MrvlIETypes_2040COEX_t */
     typedef struct _MrvlIETypes_2040COEX_t {
    /** Header */
	     IEEEtypes_Header_t header;
    /** 2040 coex element */
	     t_u8 coex_elem;
     } __attribute__ ((packed))
     MrvlIETypes_2040COEX_t;

     typedef struct _HostCmd_DS_CMD_11N_2040COEX {
	/** 2040 coex element */
	     MrvlIETypes_2040COEX_t coex;
	/** 2040 BSS intolerant channel report*/
	     MrvlIETypes_2040BssIntolerantChannelReport_t chan_intol_report;
     } __attribute__ ((packed))
     HostCmd_DS_CMD_11N_2040COEX;

/** Maximum number of channel per regulatory class */
#define MAX_CHAN 20
     typedef struct _class_chan_t {
	/** Regulatory class */
	     t_u8 reg_class;
	/** Channel numbers */
	     t_u8 channels[MAX_CHAN];
	/** Total number of channels */
	     t_u8 total_chan;
     } class_chan_t;

     typedef struct _region_class_chan_t {
    /** Regulatory domain */
	     int reg_domain;
    /** Channel numbers */
	     class_chan_t *class_chan_list;
    /** Number of class channel table entry */
	     int num_class_chan_entry;
     } region_class_chan_t;

     int process_host_cmd_resp(char *cmd_name, t_u8 *buf);
     void prepare_coex_cmd_buff(t_u8 *buf, t_u8 *chan_list, t_u8 num_of_chan,
				t_u8 reg_class, t_u8 is_intol_ap_present);
     int invoke_coex_command(void);

#endif /* _COEX_MISC_H_ */
