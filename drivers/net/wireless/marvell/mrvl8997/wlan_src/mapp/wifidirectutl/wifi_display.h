/** @file  wifi_display.h
 *
 *  @brief Header file for wifidirectutl application
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
    09/22/11: Initial creation
************************************************************************/
#ifndef _WIFIDISPLAY_H
#define _WIFIDISPLAY_H

#include "wifidirectutl.h"
/* coupled sink capability*/
#define MAX_COUPLED_SINK_CAPABILITY 2
#define MAX_CLIENTS 5
#define VENDOR_SPECIFIC_IE_TAG 0xDD
/** TLV : WifiDisplay devie info sublement ID */
#define TLV_TYPE_WIFIDISPLAY_DEVICE_INFO  0x00
/** TLV : WifiDisplay Assoc BSSID sublement ID */
#define TLV_TYPE_WIFIDISPLAY_ASSOC_BSSID  0x01
/** TLV : WifiDisplay Coupled Sink sublement ID */
#define TLV_TYPE_WIFIDISPLAY_COUPLED_SINK 0x06
/** TLV : WifiDisplay Session Info sublement ID */
#define TLV_TYPE_SESSION_INFO_SUBELEM     0x09
/** TLV : WifiDisplay Alternate MAC sublement ID */
#define TLV_TYPE_WIFIDISPLAY_ALTERNATE_MAC 0x10
/** Host Command ID : wfd display mode config */
#define HostCmd_CMD_WFD_DISPLAY_MODE_CONFIG         0x0106
/** Maximum ie index creation allowed with custom ie commands */
#define MAX_MGMT_IE_INDEX 12
/** Define len of OUI for WFD */
#define WIFIDISPLAY_OUI_LEN 3
/** Define OUI type for WFD */
#define WIFIDISPLAY_OUI_TYPE 1
/** Level of wifidisplay parameters in the wifidisplay.conf file */
typedef enum {
	DISPLAY_DEVICE_INFO = 1,
	DISPLAY_ASSOCIATED_BSSID,
	DISPLAY_COUPLED_SINK,
	DISPLAY_SESSION_INFO,
	DISPLAY_ALTERNATE_MAC_ADDR,
} display_param_level;
/** Valid Wifi display input parameters */
typedef enum {
	WFD_DEVICE_INFO,
	WFD_SESSION_MGMT_CONTROL_PORT,
	WFD_DEVICE_THROUGHPUT,
	WFD_COUPLED_SINK,
} display_valid_inputs;

/** AP CMD header */
#define WIFI_DISPLAY_CMD_HEADER     /** Buf Size */         \
                        t_u32 buf_size;         \
                        /** Command Code */     \
                        t_u16 cmd_code;         \
                        /** Size */             \
                        t_u16 size;             \
                        /** Sequence Number */  \
                        t_u16 seq_num;          \
                        /** Result */           \
                        t_s16 result

/** TLV buffer : WifiDisplay device info parameters*/
typedef PACK_START struct tlvbuf_wfdisplay_device_info {
    /** Tag */
	t_u8 tag;
    /** Length */
	t_u16 length;
    /** action */
    /** Device Info Bitmap */
	t_u8 display_device_info[2];
    /** Control Port for WFD communication */
	t_u8 session_mgmt_control_port[2];
    /** WFD Maximum Device Throuput */
	t_u8 wfd_device_throuput[2];
} PACK_END tlvbuf_wfdisplay_device_info;

/** TLV buffer : WifiDisplay Assoc BSSID paramenters*/
typedef PACK_START struct tlvbuf_wfdisplay_assoc_bssid {
    /** Tag */
	t_u8 tag;
    /** Length */
	t_u16 length;
    /** associated BSSID */
	t_u8 assoc_bssid[ETH_ALEN];
} PACK_END tlvbuf_wfdisplay_assoc_bssid;

/** TLV buffer : WifiDisplay Alaternate MAC paramenters*/
typedef PACK_START struct tlvbuf_wfdisplay_alternate_mac {
    /** Tag */
	t_u8 tag;
    /** Length */
	t_u16 length;
    /** Alternate MAC */
	t_u8 alternate_mac[ETH_ALEN];
} PACK_END tlvbuf_wfdisplay_alternate_mac;

/** TLV buffer : WifiDisplay coupled sink parameters*/
typedef PACK_START struct tlvbuf_wfdisplay_coupled_sink {
    /** Tag */
	t_u8 tag;
    /** Length */
	t_u16 length;
    /** Coupled Sink Bitmap */
	t_u8 coupled_sink;
    /** Coupled Peer MAC */
	t_u8 peer_mac[ETH_ALEN];
} PACK_END tlvbuf_wfdisplay_coupled_sink;

/** TLV buffer : WifiDirect Device Info parameters*/
struct WifiDisplayDeviceInfoDesc {
	t_u8 length;
	t_u8 dev_address[ETH_ALEN];
	t_u8 AssociatedBSSID[ETH_ALEN];
	t_u8 dev_info[2];
	t_u8 max_throughput[2];
	t_u8 coupled_sink[7];
};

/** TLV buffer : WifiDirect Session Info parameters*/
typedef PACK_START struct tlvbuf_wifi_display_session_info {
    /** Tag */
	t_u8 tag;
    /** Length */
	t_u16 length;
    /** Device Info Descriptor pointer */
	t_u8 *WFDDevInfoDesc;
} PACK_END tlvbuf_wifi_display_session_info;
/** TLV buffer : WifiDirect Wifi Display IE format*/
typedef PACK_START struct tlvbuf_wifidisplay_ie_format {
    /** Tag */
	t_u8 ElemId;
    /** Length */
	t_u8 length;
    /** Vendor Specific OUI */
	t_u8 Oui[3];
    /** Vendor Specific OUI */
	t_u8 OuiType;
    /** Vendor Specific OUI */
	t_u8 wfd_ie[1];
} PACK_END tlvbuf_wifidisplay_ie_format;

/** HostCmd_CMD_WIFI_DISPLAY_MODE_CONFIG */
typedef PACK_START struct _wifi_display_mode_config {
	WIFI_DISPLAY_CMD_HEADER;
    /** Action */
	t_u16 action;		/* 0 = ACT_GET; 1 = ACT_SET; */
    /** wifi display mode data */
	t_u16 mode;
} PACK_END wifi_display_mode_config;

/** HostCmd_CMD_WIFIDISPLAY_SERVICE_DISCOVERY request */
typedef PACK_START struct _wifidisplay_discovery_request {
    /** Header */
	WIFI_DISPLAY_CMD_HEADER;
    /** Peer mac address */
	t_u8 peer_mac_addr[ETH_ALEN];
    /** Category */
	t_u8 category;
    /** Action */
	t_u8 action;
    /** Dialog taken */
	t_u8 dialog_taken;
    /** Advertize protocol IE */
	t_u8 advertize_protocol_ie[4];
    /** Query request Length */
	t_u16 query_len;
    /** Information identifier */
	t_u8 info_id[2];
    /** Request Length */
	t_u16 request_len;
    /** OUI */
	t_u8 oui[3];
    /** OUI sub type */
	t_u8 oui_sub_type;
    /** Service update indicator */
	t_u16 service_update_indicator;
    /** Vendor Length */
	t_u16 vendor_len;
    /** Service protocol */
	t_u8 service_protocol;
    /** Service transaction Id */
	t_u8 service_transaction_id;
    /** Query Data */
	t_u8 disc_query[0];
} PACK_END wifidisplay_discovery_request;

/** HostCmd_CMD_WIFIDISPLAY_SERVICE_DISCOVERY response */
typedef PACK_START struct _wifidisplay_discovery_response {
    /** Header */
	WIFI_DISPLAY_CMD_HEADER;
    /** Peer mac address */
	t_u8 peer_mac_addr[ETH_ALEN];
    /** Category */
	t_u8 category;
    /** Action */
	t_u8 action;
    /** Dialog taken */
	t_u8 dialog_taken;
    /** Status code */
	t_u16 status_code;
    /** GAS comback reply */
	t_u16 gas_reply;
    /** Advertize protocol IE */
	t_u8 advertize_protocol_ie[4];
    /** Query response Length */
	t_u16 query_len;
    /** Information identifier */
	t_u8 info_id[2];
    /** Response Length */
	t_u16 response_len;
    /** OUI */
	t_u8 oui[3];
    /** OUI sub type */
	t_u8 oui_sub_type;
    /** Service update indicator */
	t_u16 service_update_indicator;
    /** Vendor Length */
	t_u16 vendor_len;
    /** Service protocol */
	t_u8 service_protocol;
    /** Service transaction Id */
	t_u8 service_transaction_id;
    /** Discovery status code */
	t_u8 disc_status_code;
    /** Response Data */
	t_u8 disc_resp[0];
} PACK_END wifidisplay_discovery_response;

int is_wifidisplay_input_valid(display_valid_inputs cmd, int argc,
			       char *argv[]);
void wifidisplaycmd_config(int argc, char *argv[]);
void wifidisplay_file_params_config(char *file_name, char *cmd_name,
				    t_u8 *pbuf, t_u16 *ie_len_wifidisplay);
void wifidisplay_cmd_status(int argc, char *argv[]);
void wifidisplaycmd_service_discovery(int argc, char *argv[]);
void wifidisplay_update_custom_ie(int argc, char *argv[]);
void wifidisplay_update_coupledsink_bitmap(int argc, char *argv[]);
#endif /* _WIFIDISPLAY_H */
