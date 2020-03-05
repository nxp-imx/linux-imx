/** @file  mlanconfig.h
  *
  * @brief This file contains definitions for application
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
     11/26/2008: initial version
************************************************************************/
#ifndef _MLANCONFIG_H_
#define _MLANCONFIG_H_

/** Include header files */
#include    <stdio.h>
#include    <ctype.h>
#include    <unistd.h>
#include    <string.h>
#include    <stdlib.h>
#include    <sys/socket.h>
#include    <sys/ioctl.h>
#include    <errno.h>
#include    <linux/if.h>
#include    <linux/wireless.h>
#include    <sys/types.h>
#include    <linux/if_ether.h>
#include    <time.h>

#if (BYTE_ORDER == LITTLE_ENDIAN)
#undef BIG_ENDIAN_SUPPORT
#endif

/** Type definition: boolean */
typedef enum { FALSE, TRUE } boolean;

/**
 * This macro specifies the attribute pack used for structure packing
 */
#ifndef __ATTRIB_PACK__
#define __ATTRIB_PACK__  __attribute__((packed))
#endif

/** 16 bits byte swap */
#define swap_byte_16(x) \
((t_u16)((((t_u16)(x) & 0x00ffU) << 8) | \
         (((t_u16)(x) & 0xff00U) >> 8)))

/** 32 bits byte swap */
#define swap_byte_32(x) \
((t_u32)((((t_u32)(x) & 0x000000ffUL) << 24) | \
         (((t_u32)(x) & 0x0000ff00UL) <<  8) | \
         (((t_u32)(x) & 0x00ff0000UL) >>  8) | \
         (((t_u32)(x) & 0xff000000UL) >> 24)))

/** Convert to correct endian format */
#ifdef 	BIG_ENDIAN_SUPPORT
/** CPU to little-endian convert for 16-bit */
#define 	cpu_to_le16(x)	swap_byte_16(x)
/** CPU to little-endian convert for 32-bit */
#define		cpu_to_le32(x)  swap_byte_32(x)
/** Little-endian to CPU convert for 16-bit */
#define 	le16_to_cpu(x)	swap_byte_16(x)
/** Little-endian to CPU convert for 32-bit */
#define		le32_to_cpu(x)  swap_byte_32(x)
#else
/** Do nothing */
#define		cpu_to_le16(x)	(x)
/** Do nothing */
#define		cpu_to_le32(x)  (x)
/** Do nothing */
#define 	le16_to_cpu(x)	(x)
/** Do nothing */
#define 	le32_to_cpu(x)	(x)
#endif

/** Character, 1 byte */
typedef char t_s8;
/** Unsigned character, 1 byte */
typedef unsigned char t_u8;

/** Short integer */
typedef signed short t_s16;
/** Unsigned short integer */
typedef unsigned short t_u16;

/** Integer */
typedef signed int t_s32;
/** Unsigned integer */
typedef unsigned int t_u32;

/** Long long integer */
typedef signed long long t_s64;
/** Unsigned long long integer */
typedef unsigned long long t_u64;

/** Void pointer (4-bytes) */
typedef void t_void;

/** Success */
#define MLAN_STATUS_SUCCESS         (0)
/** Failure */
#define MLAN_STATUS_FAILURE         (-1)

t_s8 *mlan_config_get_line(FILE * fp, t_s8 *s, t_s32 size, int *line);
int get_priv_ioctl(char *ioctl_name, int *ioctl_val, int *subioctl_val);
int fparse_for_hex(FILE * fp, t_u8 *dst);

/**
 * Hex or Decimal to Integer
 * @param   num string to convert into decimal or hex
 */
#define A2HEXDECIMAL(num)  \
    (strncasecmp("0x", (num), 2)?(unsigned int) strtoll((num),NULL,0):a2hex((num)))

/** Convert character to integer */
#define CHAR2INT(x) (((x) >= 'A') ? ((x) - 'A' + 10) : ((x) - '0'))

/** Convert TLV header from little endian format to CPU format */
#define endian_convert_tlv_header_in(x)            \
    {                                               \
        (x)->tag = le16_to_cpu((x)->tag);       \
        (x)->length = le16_to_cpu((x)->length); \
    }

/** Convert TLV header to little endian format from CPU format */
#define endian_convert_tlv_header_out(x)            \
    {                                               \
        (x)->tag = cpu_to_le16((x)->tag);       \
        (x)->length = cpu_to_le16((x)->length); \
    }
/** Private command ID to pass custom IE list */
#define CUSTOM_IE_CFG          (SIOCDEVPRIVATE + 13)
/* TLV Definitions */
/** TLV header */
#define TLVHEADER       /** Tag */      \
                        t_u16 tag;      \
                        /** Length */   \
                        t_u16 length

/** Maximum IE buffer length */
#define MAX_IE_BUFFER_LEN 256

/** TLV: Management IE list */
#define MRVL_MGMT_IE_LIST_TLV_ID          (PROPRIETARY_TLV_BASE_ID + 0x69)	//0x0169

/** TLV: Max Management IE */
#define MRVL_MAX_MGMT_IE_TLV_ID           (PROPRIETARY_TLV_BASE_ID + 0xaa)	//0x01aa

/** custom IE info */
typedef struct _custom_ie_info {
    /** size of buffer */
	t_u16 buf_size;
    /** no of buffers of buf_size */
	t_u16 buf_count;
} __ATTRIB_PACK__ custom_ie_info;

/** TLV buffer : custom IE */
typedef struct _tlvbuf_max_mgmt_ie {
    /** Header */
	TLVHEADER;
    /** No of tuples */
	t_u16 count;
    /** custom IE info tuples */
	custom_ie_info info[0];
} __ATTRIB_PACK__ tlvbuf_max_mgmt_ie;

/** custom IE */
typedef struct _custom_ie {
    /** IE Index */
	t_u16 ie_index;
    /** Mgmt Subtype Mask */
	t_u16 mgmt_subtype_mask;
    /** IE Length */
	t_u16 ie_length;
    /** IE buffer */
	t_u8 ie_buffer[0];
} __ATTRIB_PACK__ custom_ie;

/** TLV buffer : custom IE */
typedef struct _tlvbuf_custom_ie {
    /** Header */
	TLVHEADER;
    /** custom IE data */
	custom_ie ie_data[0];
} __ATTRIB_PACK__ tlvbuf_custom_ie;

/** Maximum length of lines in configuration file */
#define MAX_CONFIG_LINE                 1024
/** Ethernet address length */
#define ETH_ALEN                        6
/** MAC BROADCAST */
#define MAC_BROADCAST   0x1FF
/** MAC MULTICAST */
#define MAC_MULTICAST   0x1FE

/** pkt_header */
typedef struct _pkt_header {
    /** pkt_len */
	t_u32 pkt_len;
    /** pkt_type */
	t_u32 TxPktType;
    /** tx control */
	t_u32 TxControl;
} pkt_header;

/** wlan_802_11_header packet from FW with length */
typedef struct _wlan_mgmt_frame_tx {
    /** Packet Length */
	t_u16 frm_len;
    /** Frame Control */
	t_u16 frm_ctl;
    /** Duration ID */
	t_u16 duration_id;
    /** Address1 */
	t_u8 addr1[ETH_ALEN];
    /** Address2 */
	t_u8 addr2[ETH_ALEN];
    /** Address3 */
	t_u8 addr3[ETH_ALEN];
    /** Sequence Control */
	t_u16 seq_ctl;
    /** Address4 */
	t_u8 addr4[ETH_ALEN];
    /** Frame payload */
	t_u8 payload[0];
} __ATTRIB_PACK__ wlan_mgmt_frame_tx;

/** frame tx ioctl number */
#define FRAME_TX_IOCTL               (SIOCDEVPRIVATE + 12)

/** band BG */
#define BAND_BG        0
/** band A */
#define BAND_A         1
/** secondary channel is above */
#define SECOND_CHANNEL_ABOVE    0x1
/** secondary channel is below */
#define SECOND_CHANNEL_BELOW    0x3
/** NO PERIODIC SWITCH */
#define NO_PERIODIC_SWITCH      0
/** Enable periodic channel switch */
#define ENABLE_PERIODIC_SWITCH  1
/** Min channel value for BG band */
#define MIN_BG_CHANNEL 1
/** Max channel value for BG band */
#define MAX_BG_CHANNEL 14
/** Max channel value for A band */
#define MIN_A_CHANNEL 36
/** Max channel value for A band */
#define MAX_A_CHANNEL 252

/** Host Command ioctl number */
#define TDLS_IOCTL               (SIOCDEVPRIVATE + 5)
/** TDLS action definitions */
/** Action ID for TDLS config */
#define ACTION_TDLS_CONFIG       0x0000
/** Action ID for TDLS setinfo request */
#define ACTION_TDLS_SETINFO      0x0001
/** Action ID for TDLS Discovery request */
#define ACTION_TDLS_DISCOVERY    0x0002
/** Action ID for TDLS setup request */
#define ACTION_TDLS_SETUP        0x0003
/** Action ID for TDLS Teardown request */
#define ACTION_TDLS_TEARDOWN     0x0004
/** Action ID for TDLS power mode */
#define ACTION_TDLS_POWER_MODE   0x0005
/**Action ID for init TDLS Channel Switch*/
#define ACTION_TDLS_INIT_CHAN_SWITCH     0x06
/** Action ID for stop TDLS Channel Switch */
#define ACTION_TDLS_STOP_CHAN_SWITCH     0x07
/** Action ID for configure CS related parameters */
#define ACTION_TDLS_CS_PARAMS            0x08
/** Action ID for TDLS Disable Channel switch */
#define ACTION_TDLS_CS_DISABLE          0x09
/** Action ID for TDLS Link status */
#define ACTION_TDLS_LINK_STATUS  0x000A
/** Action ID for TDLS CS immediate return */
#define ACTION_TDLS_DEBUG_CS_RET_IM 0xFFF7
/** Action ID for TDLS Stop RX */
#define ACTION_TDLS_DEBUG_STOP_RX 0xFFF8
/** Action ID for TDLS Allow weak security */
#define ACTION_TDLS_DEBUG_ALLOW_WEAK_SECURITY 0xFFF9
/** Action ID for TDLS Ignore key lifetime expiry */
#define ACTION_TDLS_DEBUG_IGNORE_KEY_EXPIRY 0xFFFA
/** Action ID for TDLS Higher/Lower mac Test */
#define ACTION_TDLS_DEBUG_HIGHER_LOWER_MAC 0xFFFB
/** Action ID for TDLS Prohibited Test */
#define ACTION_TDLS_DEBUG_SETUP_PROHIBITED 0xFFFC
/** Action ID for TDLS Existing link Test */
#define ACTION_TDLS_DEBUG_SETUP_SAME_LINK 0xFFFD
/** Action ID for TDLS Fail Setup Confirm */
#define ACTION_TDLS_DEBUG_FAIL_SETUP_CONFIRM 0xFFFE
/** Action ID for TDLS WRONG BSS Test */
#define ACTION_TDLS_DEBUG_WRONG_BSS 0xFFFF

/** TLV type : Rates */
#define TLV_TYPE_RATES                          0x0001
/** TLV type : Domain */
#define TLV_TYPE_DOMAIN                         0x0007
/** TLV type : Supported channels */
#define TLV_TYPE_SUPPORTED_CHANNELS             0x0024
/** TLV type : HT Capabilities */
#define TLV_TYPE_HT_CAP                         0x002d
/** TLV type : Qos Info */
#define TLV_TYPE_QOSINFO                        0x002e
/** TLV type : RSN IE */
#define TLV_TYPE_RSN_IE                         0x0030
/** TLV type : extended supported rates */
#define TLV_EXTENDED_SUPPORTED_RATES            0x0032
/** TLV type :  timeout interval    */
#define TLV_TIMEOUT_INTERVAL                    0x0038
/** TLV type : Regulatory classes */
#define TLV_TYPE_REGULATORY_CLASSES             0x003b
/** TLV type : HT Information */
#define TLV_TYPE_HT_INFO                        0x003d
/** TLV type : 20/40 BSS Coexistence */
#define TLV_TYPE_2040BSS_COEXISTENCE            0x0048
/** TLv Type : Link identifier */
#define TLV_LINK_IDENTIFIER                     0x0065
/** TLV type : Extended capabilities */
#define TLV_TYPE_EXTCAP                         0x007f

/** TLV type : VHT capabilities */
#define TLV_TYPE_VHT_CAP		        0x00BF
/** TLV type : VHT operations */
#define TLV_TYPE_VHT_OPER	                0x00C0
/** Length of Basic MCS MAP */
#define VHT_MCS_MAP_LEN    			2

/** Country code length */
#define COUNTRY_CODE_LEN                        3
/** Length of Group Cipher Suite */
#define GROUP_CIPHER_SUITE_LEN     4
/** Length of Pairwise Cipher Suite */
#define PAIRWISE_CIPHER_SUITE_LEN  4
/** Length of AKM Suite */
#define AKM_SUITE_LEN          4
/** PMKID length */
#define PMKID_LEN              16
/** Maximum number of pairwise_cipher_suite */
#define MAX_PAIRWISE_CIPHER_SUITE_COUNT    2
/** Maximum number of AKM suite */
#define MAX_AKM_SUITE_COUNT    2
/** Maximum number of PMKID list count */
#define MAX_PMKID_COUNT    2
/** Length of MCS set */
#define MCS_SET_LEN    16
/** Version in RSN IE */
#define VERSION_RSN_IE    0x0001

/** tdls setinfo */
typedef struct _tdls_setinfo {
    /** Action */
	t_u16 action;
    /** (TLV + capInfo) length */
	t_u16 tlv_len;
    /** Capability Info */
	t_u16 cap_info;
    /** tdls info */
	t_u8 tlv_buffer[0];
} __ATTRIB_PACK__ tdls_setinfo;

/** tdls discovery */
typedef struct _tdls_discovery {
    /** Action */
	t_u16 action;
    /** Peer MAC address */
	t_u8 peer_mac[ETH_ALEN];
} __ATTRIB_PACK__ tdls_discovery;

/** tdls link status */
typedef struct _tdls_links_status {
    /** Action */
	t_u16 action;
} __ATTRIB_PACK__ tdls_link_status;

/** tdls discovery response */
typedef struct _tdls_discovery_resp {
    /** Action */
	t_u16 action;
    /** payload length */
	t_u16 payload_len;
    /** peer mac Address */
	t_u8 peer_mac[ETH_ALEN];
    /** RSSI */
	signed char rssi;
    /** Cap Info */
	t_u16 cap_info;
    /** TLV buffer */
	t_u8 tlv_buffer[0];
} __ATTRIB_PACK__ tdls_discovery_resp;

/** tdls each link rate information */
typedef struct _tdls_link_rate_info {
    /** Tx Data Rate */
	t_u8 tx_data_rate;
    /** Tx Rate HT info*/
	t_u8 tx_rate_htinfo;
} __ATTRIB_PACK__ tdls_link_rate_info;

/** tdls each link status */
typedef struct _tdls_each_link_status {
    /** peer mac Address */
	t_u8 peer_mac[ETH_ALEN];
    /** Link Flags */
	t_u8 link_flags;
    /** Traffic Status */
	t_u8 traffic_status;
    /** Tx Failure Count */
	t_u8 tx_fail_count;
    /** Channel Number */
	t_u32 active_channel;
    /** Last Data RSSI in dBm */
	t_s16 data_rssi_last;
    /** Last Data NF in dBm */
	t_s16 data_nf_last;
    /** AVG DATA RSSI in dBm */
	t_s16 data_rssi_avg;
    /** AVG DATA NF in dBm */
	t_s16 data_nf_avg;
	union {
	/** tdls rate info */
		tdls_link_rate_info rate_info;
	/** final rate value*/
		t_u16 final_data_rate;
	} u;
    /** Security Method */
	t_u8 security_method;
    /** Key Lifetime */
	t_u32 key_lifetime;
    /** Key Length */
	t_u8 key_length;
    /** actual key */
	t_u8 key[0];
} __ATTRIB_PACK__ tdls_each_link_status;

/** tdls link status response */
typedef struct _tdls_link_status_resp {
    /** Action */
	t_u16 action;
    /** payload length */
	t_u16 payload_len;
    /** number of links */
	t_u8 active_links;
    /** structure for link status */
	tdls_each_link_status link_stats[1];
} __ATTRIB_PACK__ tdls_link_status_resp;

/** tdls setup */
typedef struct _tdls_setup {
    /** Action */
	t_u16 action;
    /** Peer MAC address */
	t_u8 peer_mac[ETH_ALEN];
    /** Time to wait for response from peer*/
	t_u32 wait_time;
    /** Key Life Time */
	t_u32 key_life_time;
} __ATTRIB_PACK__ tdls_setup;

/** tdls tear down */
typedef struct _tdls_tear_down {
    /** Action */
	t_u16 action;
    /** Peer MAC address */
	t_u8 peer_mac[ETH_ALEN];
    /** Reason code */
	t_u16 reason_code;
} __ATTRIB_PACK__ tdls_teardown;

/** tdls power mode */
typedef struct _tdls_power_mode {
    /** Action */
	t_u16 action;
    /** Peer MAC address */
	t_u8 peer_mac[ETH_ALEN];
    /** Power mode */
	t_u16 power_mode;
} __ATTRIB_PACK__ tdls_powermode;

/** tdls channel switch info */
typedef struct _tdls_channel_switch {
    /** Action */
	t_u16 action;
    /** peer mac Address */
	t_u8 peer_mac[ETH_ALEN];
    /** Channel Switch primary channel no */
	t_u8 primary_channel;
    /** Channel Switch secondary channel offset */
	t_u8 secondary_channel_offset;
    /** Channel Switch Band */
	t_u8 band;
    /** Channel Switch time*/
	t_u16 switch_time;
    /** Channel Switch timeout*/
	t_u16 switch_timeout;
    /** Channel Regulatory class*/
	t_u8 regulatory_class;
    /** Channel Switch periodicity*/
	t_u8 periodicity;
} __ATTRIB_PACK__ tdls_channel_switch;

/** tdls stop channel switch */
typedef struct _tdls_stop_chan_switch {
    /** Action */
	t_u16 action;
    /** Peer MAC address */
	t_u8 peer_mac[ETH_ALEN];
} __ATTRIB_PACK__ tdls_stop_chan_switch;

/** tdls disable channel switch */
typedef struct _tdls_disable_cs {
    /** Action */
	t_u16 action;
    /** Data*/
	t_u16 data;
} __ATTRIB_PACK__ tdls_disable_cs, tdls_config;

/** tdls channel switch parameters */
typedef struct _tdls_cs_params {
    /** Action */
	t_u16 action;
    /** unit time, multiples of 10ms */
	t_u8 unit_time;
    /** threshold for other link */
	t_u8 threshold_otherlink;
    /** threshold for direct link */
	t_u8 threshold_directlink;
} __ATTRIB_PACK__ tdls_cs_params;

/** tdls debug */
typedef struct _tdls_debug {
    /** Action */
	t_u16 action;
    /** Data */
	t_u8 data[0];
} __ATTRIB_PACK__ tdls_debug;

/** TLV header */
#define TLVHEADER       /** Tag */      \
                        t_u16 tag;      \
                        /** Length */   \
                        t_u16 length

/** Length of TLV header */
#define TLVHEADER_LEN  4

/** Data structure for subband set */
typedef struct _IEEEtypes_SubbandSet_t {
    /** First channel */
	t_u8 first_chan;
    /** Number of channels */
	t_u8 no_of_chan;
    /** Maximum Tx power */
	t_u8 max_tx_pwr;
} __ATTRIB_PACK__ IEEEtypes_SubbandSet_t, *pIEEEtypes_SubbandSet_t;

/** tlvbuf_DomainParamSet_t */
typedef struct _tlvbuf_DomainParamSet {
    /** Header */
	TLVHEADER;
    /** Country code */
	t_u8 country_code[COUNTRY_CODE_LEN];
    /** Set of subbands */
	IEEEtypes_SubbandSet_t sub_band[0];
} __ATTRIB_PACK__ tlvbuf_DomainParamSet_t;

/** Data structure of WMM QoS information */
typedef struct _IEEEtypes_WmmQosInfo_t {
#ifdef BIG_ENDIAN_SUPPORT
    /** QoS UAPSD */
	t_u8 qos_uapsd:1;
    /** Reserved */
	t_u8 reserved:3;
    /** Parameter set count */
	t_u8 para_set_count:4;
#else
    /** Parameter set count */
	t_u8 para_set_count:4;
    /** Reserved */
	t_u8 reserved:3;
    /** QoS UAPSD */
	t_u8 qos_uapsd:1;
#endif				/* BIG_ENDIAN_SUPPORT */
} __ATTRIB_PACK__ IEEEtypes_WmmQosInfo_t;

/** Qos Info TLV */
typedef struct _tlvbuf_QosInfo_t {
    /** Header */
	TLVHEADER;
    /** QosInfo */
	union {
	/** QosInfo bitfield */
		IEEEtypes_WmmQosInfo_t qos_info;
	/** QosInfo byte */
		t_u8 qos_info_byte;
	} u;
} __ATTRIB_PACK__ tlvbuf_QosInfo_t;

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
} __ATTRIB_PACK__ HTCap_t, *pHTCap_t;

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
} __ATTRIB_PACK__ HTInfo_t, *pHTInfo_t;

/** 20/40 BSS Coexistence Data */
typedef struct _BSSCo2040_t {
    /** 20/40 BSS Coexistence value */
	t_u8 bss_co_2040_value;
} __ATTRIB_PACK__ BSSCo2040_t, *pBSSCo2040_t;

/** HT Capabilities element */
typedef struct _tlvbuf_HTCap_t {
    /** Header */
	TLVHEADER;
    /** HTCap struct */
	HTCap_t ht_cap;
} __ATTRIB_PACK__ tlvbuf_HTCap_t;

/** HT Information element */
typedef struct _tlvbuf_HTInfo_t {
    /** Header */
	TLVHEADER;

    /** HTInfo struct */
	HTInfo_t ht_info;
} __ATTRIB_PACK__ tlvbuf_HTInfo_t;

/** VHT MCS rate set field, refer to 802.11ac */
typedef struct _VHT_MCS_set {
	t_u16 rx_mcs_map;
	t_u16 rx_max_rate;	/* bit 29-31 reserved */
	t_u16 tx_mcs_map;
	t_u16 tx_max_rate;	/* bit 61-63 reserved */
} __ATTRIB_PACK__ VHT_MCS_set_t, *pVHT_MCS_set_t;

typedef struct _VHT_capa {
	t_u32 vht_cap_info;
	VHT_MCS_set_t mcs_sets;
} __ATTRIB_PACK__ VHT_capa_t, *pVHT_capa_t;

/** VHT Capabilities IE */
typedef struct _IEEEtypes_VHTCap_t {
    /** Header */
	TLVHEADER;
    /** VHTInfo struct */
	VHT_capa_t vht_cap;
} __ATTRIB_PACK__ tlvbuf_VHTCap_t, *ptlvbuf_VHTCap_t;

/** VHT Operations IE */
typedef struct _IEEEtypes_VHTOprat_t {
    /** Header */
	TLVHEADER;
    /** VHTOpra struct */
	t_u8 chan_width;
	t_u8 chan_cf1;
	t_u8 chan_cf2;
    /** Basic MCS set map, each 2 bits stands for a Nss */
	t_u8 basic_mcs_map[VHT_MCS_MAP_LEN];
} __ATTRIB_PACK__ tlvbuf_VHTOpra_t, *ptlvbuf_VHTOpra_t;

/** 20/40 BSS Coexistence element */
typedef struct _tlvbuf_2040BSSCo_t {
    /** Header */
	TLVHEADER;

    /** BSSCo2040_t struct */
	BSSCo2040_t bss_co_2040;
} __ATTRIB_PACK__ tlvbuf_2040BSSCo_t;

/** Extended Capabilities element */
typedef struct _tlvbuf_ExtCap_t {
    /** Header */
	TLVHEADER;
    /** ExtCap_t struct */
	t_u8 ext_cap[0];
} __ATTRIB_PACK__ tlvbuf_ExtCap_t;

/** tlvbuf_RatesParamSet_t */
typedef struct _tlvbuf_RatesParamSet_t {
    /** Header */
	TLVHEADER;
    /** Rates */
	t_u8 rates[0];
} __ATTRIB_PACK__ tlvbuf_RatesParamSet_t;

/*  IEEE Supported Channel sub-band description (7.3.2.19) */
/*  Sub-band description used in the supported channels element. */
typedef struct _IEEEtypes_SupportChan_Subband_t {
	t_u8 start_chan;/**< Starting channel in the subband */
	t_u8 num_chans;	/**< Number of channels in the subband */

} __ATTRIB_PACK__ IEEEtypes_SupportChan_Subband_t;

/*  IEEE Supported Channel element (7.3.2.19) */
/**
 *  Sent in association requests. Details the sub-bands and number
 *    of channels supported in each subband
 */
typedef struct _tlvbuf_SupportedChannels_t {
    /** Header */
	TLVHEADER;
		 /**< IEEE Element ID = 36 */
    /** Configured sub-bands information in the element */
	IEEEtypes_SupportChan_Subband_t subband[0];
} __ATTRIB_PACK__ tlvbuf_SupportedChannels_t;

/*  IEEE Supported Regulatory Classes description (7.3.2.54) */
typedef struct _IEEEtypes_RegulatoryClass_t {
    /** current regulatory class */
	t_u8 cur_regulatory_class;
    /** supported regulatory class list */
	t_u8 regulatory_classes_list[0];

} __ATTRIB_PACK__ IEEEtypes_RegulatoryClass_t;

/*  IEEE Supported Regulatory Classes TLV (7.3.2.54) */
typedef struct _tlvbuf_RegulatoryClass_t {
    /** Header */
	TLVHEADER;
		 /**< IEEE Element ID = 59 */
    /** supported regulatory class */
	IEEEtypes_RegulatoryClass_t regulatory_class;
} __ATTRIB_PACK__ tlvbuf_RegulatoryClass_t;

/** tlvbuf_RsnParamSet_t */
typedef struct _tlvbuf_RsnParamSet_t {
    /** Header */
	TLVHEADER;
    /** Version */
	t_u16 version;
    /** Group Cipher Suite */
	t_u8 group_cipher_suite[4];
    /** Pairwise Cipher Suite count */
	t_u16 pairwise_cipher_count;
    /** Pairwise Cipher Suite */
	t_u8 pairwise_cipher_suite[0];
    /** AKM Suite Sount */
	t_u16 akm_suite_count;
    /** AKM Suite */
	t_u8 akm_suite[0];
    /** RSN Capability */
	t_u16 rsn_capability;
    /** PMKID Count */
	t_u16 pmkid_count;
    /** PMKID list */
	t_u8 pmkid_list[0];
} __ATTRIB_PACK__ tlvbuf_RsnParamSet_t;

extern t_s32 sockfd;  /**< socket */
extern t_s8 dev_name[IFNAMSIZ + 1];   /**< device name */

#endif /* _MLANCONFIG_H_ */
